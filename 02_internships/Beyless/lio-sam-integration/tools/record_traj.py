#!/usr/bin/env python3
"""Record online mapping poses and maximum cumulative loop marker edge count."""
import argparse
import json
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray
import numpy as np

def marker_pairs(message, received_wall, received_sim=None):
    """Upstream LINE_LIST is [current, previous], repeated in current-index order."""
    pairs=[]
    for marker in message.markers:
        if marker.ns != "loop_edges" or marker.type != 5 or marker.action != 0:
            continue
        if len(marker.points) % 2: raise ValueError("Odd loop LINE_LIST endpoint count")
        for i in range(0,len(marker.points),2):
            pairs.append(dict(endpoints=[[p.x,p.y,p.z] for p in marker.points[i:i+2]],
                              marker_stamp=marker.header.stamp.sec+marker.header.stamp.nanosec*1e-9,
                              frame_id=marker.header.frame_id,received_wall=received_wall,received_sim=received_sim))
    return pairs

def finalize_loop_pairs(directory):
    directory=Path(directory); path=directory/"loop_pairs.json"
    obj=json.loads(path.read_text()); poses=np.loadtxt(directory/"keyframes_final_tum.txt",ndmin=2)
    for pair in obj["pairs"]:
        distances=np.linalg.norm(np.asarray(pair["endpoints"])[:,None,:]-poses[None,:,1:4],axis=2)
        indices=np.argmin(distances,axis=1)
        pair.update(keyframe_indices=indices.tolist(),matching_distances_m=distances[np.arange(2),indices].tolist(),
                    keyframe_stamps=poses[indices,0].tolist())
    obj["matching_note"]="Latest cumulative marker snapshot; independent nearest final keyframe in odom coordinates, endpoint order current/previous. Inspect matching distances for ambiguity."
    path.write_text(json.dumps(obj,indent=2)+"\n")
    return obj

class Recorder(Node):
    def __init__(self, directory):
        super().__init__('trajectory_recorder')
        self.directory=Path(directory); self.f=(self.directory/'odom_online_tum.txt').open('w',buffering=1)
        self.clock=None; self.lags=[]; self.loop_pairs=[]; self.loop_snapshots=[]
        self.create_subscription(Clock,"/clock",self.tick,qos_profile_sensor_data)
        self.count=0; self.loops=0; self.first=None; self.last=None; self.last_wall=None
        self.create_subscription(Odometry,'/lio_sam/mapping/odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(MarkerArray,'/lio_sam/mapping/loop_closure_constraints',self.markers,10)
        self.create_timer(1.,self.status)
    def tick(self,m):
        self.clock=m.clock.sec+m.clock.nanosec*1e-9
    def odom(self,m):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9; p=m.pose.pose.position; q=m.pose.pose.orientation
        self.f.write(f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
        if self.clock is not None: self.lags.append(self.clock-t)
        self.count+=1; self.first=t if self.first is None else self.first; self.last=t; self.last_wall=time.time()
    def markers(self,m):
        pairs=marker_pairs(m,time.time(),self.clock)
        if pairs:
            self.loops=max(self.loops,len(pairs)); self.loop_pairs=pairs
            self.loop_snapshots.append(pairs)
    def status(self):
        obj=dict(online_count=self.count,loop_events=self.loops,first_stamp=self.first,last_stamp=self.last,last_receive_wall=self.last_wall, max_sim_lag_s=max(self.lags,default=None), mean_sim_lag_s=sum(self.lags)/len(self.lags) if self.lags else None)
        pair_tmp=self.directory/'loop_pairs.tmp'
        pair_tmp.write_text(json.dumps(dict(pairs=self.loop_pairs,snapshots=self.loop_snapshots)))
        pair_tmp.replace(self.directory/'loop_pairs.json')
        tmp=self.directory/'recorder_status.tmp'; tmp.write_text(json.dumps(obj)); tmp.replace(self.directory/'recorder_status.json')

def main():
    p=argparse.ArgumentParser(); p.add_argument('directory'); args=p.parse_args()
    rclpy.init(); node=Recorder(args.directory)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.status(); node.f.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
