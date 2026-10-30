#!/usr/bin/env python3
"""Read-only diagnostic of prescribed LIO-SAM accelerometer/orientation consistency."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from preprocess_bag import reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=yaml.safe_load((ROOT/'config/params_base.yaml').read_text())['/**']['ros__parameters']
    ext=np.array(p['extrinsicRot']).reshape(3,3); rpy=np.array(p['extrinsicRPY']).reshape(3,3)
    r=reader(ROOT/'data/lio_bag'); original=[]; configured=[]
    while r.has_next() and len(original)<100:
        topic,data,_=r.read_next()
        if topic!='/imu_raw': continue
        m=deserialize_message(data,Imu); q=m.orientation; a=m.linear_acceleration
        rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(); acc=np.array([a.x,a.y,a.z])
        original.append(rot@acc); configured.append(rot@rpy@ext@acc)
    vector=np.mean(configured,axis=0)
    result=dict(first_100_imu_mean_world_acc_original=np.mean(original,axis=0).tolist(),first_100_imu_mean_world_acc_configured=vector.tolist(),configured_gravity_tilt_deg=float(np.degrees(np.arccos(vector[2]/np.linalg.norm(vector)))),formula='q_final=q_from*extQRPY; acc_out=extRot*acc; world_acc=R(q_from)*extrinsicRPY*extrinsicRot*acc',note='v2 approved correction: extrinsicRot=R.T, extrinsicRPY=R.')
    (ROOT/'results/imu_frame_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
