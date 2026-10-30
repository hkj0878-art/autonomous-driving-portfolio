#!/usr/bin/env python3
"""Sequential isolated LIO-SAM experiment with strictly scoped save destination."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import numpy as np
import yaml
from pcd_to_tum import convert
from record_traj import finalize_loop_pairs
ROOT=Path(__file__).resolve().parents[1]
PLAYBACK_DISCOVERY_DELAY_S=3.0

def safe_destination(run, home=None):
    home=Path(home or os.environ['HOME'])
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}',run): raise ValueError('Unsafe run name')
    target=ROOT/'results'/run/'map'
    if ROOT != home/'SLAM': raise ValueError('Service HOME prefix does not match workspace')
    for p in [home,ROOT,ROOT/'results',target.parent,target]:
        if p.is_symlink() or p.resolve()!=p: raise ValueError('Symlink destination rejected')
    destination=f'/SLAM/results/{run}/map'
    if home.as_posix()+destination!=str(target): raise ValueError('Destination mismatch')
    return destination

def stop(p):
    if p and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try: p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('run'); parser.add_argument('params'); parser.add_argument('bag',nargs='?',default=str(ROOT/'data/lio_bag')); parser.add_argument('rate',nargs='?',type=float,default=1.)
    args=parser.parse_args(); destination=safe_destination(args.run)
    if not 0<args.rate<=1: raise ValueError('rate must be in (0,1]')
    lock=(ROOT/'results/.run.lock').open('w'); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    directory=ROOT/'results'/args.run; directory.mkdir(exist_ok=False)
    params=Path(args.params).resolve(); bag=Path(args.bag).resolve()
    v=yaml.safe_load(params.read_text())['/**']['ros__parameters']
    if v['savePCD'] is not False or v['gpsTopic']!='odometry/gps': raise ValueError('Unsafe savePCD/GPS config')
    (directory/'params.yaml').write_bytes(params.read_bytes())
    os.environ['ROS_DOMAIN_ID']='83'; os.environ['ROS_LOCALHOST_ONLY']='1'; os.environ['RCUTILS_COLORIZED_OUTPUT']='0'
    os.environ['ROS_LOG_DIR']=str(directory/'ros_logs')
    meta=dict(run=args.run,params_sha256=hashlib.sha256(params.read_bytes()).hexdigest(),git_commit=subprocess.check_output(['git','-C',str(ROOT/'ws/src/LIO-SAM'),'rev-parse','HEAD'],text=True).strip(),bag=str(bag),rate=args.rate,loop_enabled=v['loopClosureEnableFlag'],destination=destination,success=False)
    launch=rec=player=None; start=time.monotonic(); log=(directory/'run.log').open('w',buffering=1)
    def note(s): print(s,flush=True); log.write(s+'\n')
    def spawn(cmd):
        note('COMMAND '+repr(cmd)); return subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    import rclpy
    from lio_sam.srv import SaveMap
    rclpy.init(); node=rclpy.create_node('experiment_controller'); client=node.create_client(SaveMap,'/lio_sam/save_map')
    try:
        launch=spawn(['ros2','launch',str(ROOT/'launch/lio_sam_nox.launch.py'),f'params_file:={params}','use_sim_time:=true','rviz:=false'])
        if not client.wait_for_service(timeout_sec=30): raise RuntimeError('save_map service not ready')
        rec=spawn(['/home/hong/SLAM/.venv/bin/python',str(ROOT/'tools/record_traj.py'),str(directory)])
        deadline=time.monotonic()+20
        while node.count_subscribers('/lio_sam/mapping/odometry')<1:
            if time.monotonic()>deadline: raise RuntimeError('Recorder not ready')
            rclpy.spin_once(node,timeout_sec=.1)
        deadline=time.monotonic()+30
        required={'/points_raw':1,'/imu_raw':2}
        while any(node.count_subscribers(topic)<count for topic,count in required.items()):
            if time.monotonic()>deadline: raise RuntimeError('Sensor subscribers not ready')
            rclpy.spin_once(node,timeout_sec=.1)
        meta['sensor_subscribers']={topic:node.count_subscribers(topic) for topic in required}
        meta['playback_discovery_delay_s']=PLAYBACK_DISCOVERY_DELAY_S
        meta['gps_publishers']=node.count_publishers('/odometry/gps')
        if meta['gps_publishers']!=0: raise RuntimeError('Unexpected GPS input publisher')
        note('GPS factor disabled: /odometry/gps publishers = 0; bag play restricts topics to points/IMU (GNSS excluded).')
        play_start=time.monotonic()
        player=spawn(['ros2','bag','play',str(bag),'--clock','100','--delay',str(PLAYBACK_DISCOVERY_DELAY_S),'--rate',str(args.rate),'--topics','/points_raw','/imu_raw','--disable-keyboard-controls'])
        deadline=time.monotonic()+108.9/args.rate+180
        while player.poll() is None:
            if launch.poll() is not None or rec.poll() is not None: raise RuntimeError('Launch/recorder exited during playback')
            if time.monotonic()>deadline: raise RuntimeError('Playback timeout')
            rclpy.spin_once(node,timeout_sec=.2)
        if player.returncode: raise RuntimeError(f'Playback failed {player.returncode}')
        meta['playback_wall_seconds']=time.monotonic()-play_start
        # Allow mapping backlog and loop worker to settle, while clock remains at bag end.
        drain=time.monotonic(); prior=None; stable=0
        while time.monotonic()-drain<90:
            rclpy.spin_once(node,timeout_sec=1)
            status=json.loads((directory/'recorder_status.json').read_text())
            current=(status['online_count'],status['loop_events'])
            stable=stable+1 if current==prior else 0; prior=current
            if stable>=10 and time.monotonic()-drain>=15: break
        meta['drain_seconds']=time.monotonic()-drain
        if not status['online_count']: raise RuntimeError('No mapping odometry')
        # Validate immediately before service's destructive implementation.
        if safe_destination(args.run)!=destination: raise RuntimeError('Destination changed')
        req=SaveMap.Request(); req.resolution=0.; req.destination=destination
        future=client.call_async(req); rclpy.spin_until_future_complete(node,future,timeout_sec=120)
        if not future.done() or not future.result().success: raise RuntimeError('save_map failed')
        for name in ('GlobalMap','CornerMap','SurfMap','trajectory','transformations'):
            if (directory/f'map/{name}.pcd').stat().st_size<100: raise RuntimeError(f'Missing/empty {name}')
        poses=convert(directory/'map/transformations.pcd',directory/'keyframes_final_tum.txt')
        steps=np.linalg.norm(np.diff(poses[:,1:4],axis=0),axis=1); speeds=steps/np.diff(poses[:,0])
        meta.update(status); meta.update(keyframes=len(poses),distance_m=float(steps.sum()),endpoint_m=float(np.linalg.norm(poses[-1,1:4]-poses[0,1:4])),max_speed_mps=float(speeds.max()),final_keyframe_stamp=float(poses[-1,0]))
        meta['sanity_pass']=bool(238.7*.9<=steps.sum()<=238.7*1.1 and speeds.max()<=30)
        meta['success']=meta['sanity_pass'] and (meta['loop_events']>=1 if meta['loop_enabled'] else meta['loop_events']==0)
        note('RESULT '+json.dumps(meta))
    except Exception as exc:
        meta['error']=repr(exc); note('FAILED '+repr(exc))
    finally:
        stop(player); stop(rec); stop(launch)
        if (directory/"keyframes_final_tum.txt").exists() and (directory/"loop_pairs.json").exists():
            finalize_loop_pairs(directory)
        meta['processing_wall_seconds']=time.monotonic()-start
        log.flush(); content=(directory/'run.log').read_text()
        meta['imu_velocity_resets']=content.count('Large velocity, reset IMU-preintegration')
        meta['invalid_tf_warnings']=content.count('TF_NO_FRAME_ID')
        meta['crash_detected']=bool(re.search(r'process has died|Segmentation fault|double free|terminate called',content))
        if meta['crash_detected']: meta['success']=False
        (directory/'run_meta.json').write_text(json.dumps(meta,indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown(); log.close()
    return 0 if meta['success'] else 1
if __name__=='__main__': raise SystemExit(main())
