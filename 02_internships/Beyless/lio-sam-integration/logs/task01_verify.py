"""Run after sourcing env.sh; check discovery, TF, and five-second survival."""

import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).resolve().parents[1]
os.environ['ROS_LOG_DIR'] = str(ROOT / 'logs/task01_ros')
# Isolate this smoke test from other ROS applications on the workstation.
os.environ['ROS_DOMAIN_ID'] = '81'
expected = {'lio_sam_' + name for name in (
    'imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization')}
log_path = ROOT / 'logs/task01_launch.log'
rclpy.init()
probe = rclpy.create_node('task01_verifier')
transforms = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscription = probe.create_subscription(
    TFMessage, '/tf_static', lambda msg: transforms.extend(msg.transforms), qos)
with log_path.open('w') as output:
    process = subprocess.Popen(
        ['ros2', 'launch', str(ROOT / 'launch/lio_sam_nox.launch.py')],
        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 30
        while True:
            rclpy.spin_once(probe, timeout_sec=0.1)
            names = set(probe.get_node_names())
            assert process.poll() is None, 'Launch exited during startup'
            if expected <= names and transforms:
                break
            assert time.monotonic() < deadline, f'Startup timeout: {names}'
        started = time.monotonic()
        while time.monotonic() - started < 5:
            rclpy.spin_once(probe, timeout_sec=0.1)
            assert process.poll() is None, 'Launch exited during observation'
            assert expected <= set(probe.get_node_names()), 'A node disappeared'
            contents = log_path.read_text()
            assert not re.search(r'process has died|error while loading shared libraries|\[ERROR\]|Segmentation fault', contents), contents
        tf = next(t for t in transforms
                  if t.header.frame_id == 'map' and t.child_frame_id == 'odom')
        assert (tf.transform.translation.x, tf.transform.translation.y,
                tf.transform.translation.z) == (0.0, 0.0, 0.0)
        assert (tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w) == (0.0, 0.0, 0.0, 1.0)
        print('PASS: four LIO-SAM nodes discovered and alive for >=5 seconds')
        print('PASS: map -> odom identity received on /tf_static')
        print('PASS: no process death, launch errors, or shared-library errors')
    finally:
        if process.poll() is None:
            # Let launch forward SIGINT once to its children.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        probe.destroy_node()
        rclpy.shutdown()
        print(f'Launch shutdown exit code: {process.returncode}')
