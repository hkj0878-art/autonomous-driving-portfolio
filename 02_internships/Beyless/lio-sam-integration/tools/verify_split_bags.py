#!/usr/bin/env python3
"""Read-only, streaming payload/timestamp comparison with full processed bag."""
import hashlib
import json
from pathlib import Path
import sqlite3
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
ROOT=Path(__file__).resolve().parents[1]

def connection(name):
    directory=ROOT/'data'/name
    meta=yaml.safe_load((directory/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    assert len(meta['relative_file_paths'])==1
    return sqlite3.connect((directory/meta['relative_file_paths'][0]).as_uri()+'?mode=ro',uri=True)

def rows(c,topic):
    return c.execute('SELECT timestamp,data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name=?) ORDER BY timestamp,id',(topic,))

def verify():
    full=connection('lio_bag'); outputs={}
    for phase,name in enumerate(('lio_bag_even','lio_bag_odd')):
        split=connection(name); counts={}
        for topic in ('/points_raw','/imu_raw','/gps/fix'):
            reference=rows(full,topic); actual=iter(rows(split,topic)); count=0; digest=hashlib.sha256()
            for i,(stamp,payload) in enumerate(reference):
                if topic=='/points_raw' and i%2!=phase: continue
                got=next(actual)
                assert got[0]==stamp,(name,topic,i,'timestamp')
                if topic=='/points_raw':
                    # Re-serialization can differ in unspecified CDR alignment padding.
                    assert deserialize_message(got[1],PointCloud2)==deserialize_message(payload,PointCloud2),(name,topic,i)
                else:
                    assert got[1]==payload,(name,topic,i,'payload')
                digest.update(payload); count+=1
            assert next(actual,None) is None
            counts[topic]={'count':count,'sha256':digest.hexdigest()}
        outputs[name]=counts; split.close()
    full.close()
    (ROOT/'results/split_verification.json').write_text(json.dumps(outputs,indent=2)+'\n'); print(json.dumps(outputs,indent=2))
if __name__=='__main__': verify()
