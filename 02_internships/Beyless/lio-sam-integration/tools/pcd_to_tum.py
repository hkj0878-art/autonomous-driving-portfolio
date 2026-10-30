#!/usr/bin/env python3
"""Read PCL ASCII/binary (uncompressed) PointTypePose, convert Rz Ry Rx to TUM."""
import argparse
from pathlib import Path
import numpy as np

def read_pcd(path):
    with open(path, 'rb') as f:
        h = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError('Missing PCD DATA header')
            s = line.decode('ascii').strip().split()
            if not s or s[0].startswith('#'): continue
            h[s[0]] = s[1:]
            if s[0] == 'DATA': break
        counts = list(map(int, h.get('COUNT', ['1']*len(h['FIELDS']))))
        types = {('F',4):'<f4',('F',8):'<f8',('U',4):'<u4',('U',2):'<u2',('I',4):'<i4',('U',1):'u1'}
        dtype = np.dtype([(n, types[(t,int(z))], (c,)) if c != 1 else (n,types[(t,int(z))]) for n,t,z,c in zip(h['FIELDS'],h['TYPE'],h['SIZE'],counts)])
        n = int(h['POINTS'][0])
        if h['DATA'] == ['binary']:
            a = np.frombuffer(f.read(), dtype=dtype)
        elif h['DATA'] == ['ascii']:
            a = np.loadtxt(f, dtype=dtype, ndmin=1)
        else: raise ValueError('Unsupported compressed PCD')
        if len(a) != n: raise ValueError('PCD point count mismatch')
        return a

def pose_to_tum(a):
    required = ('x','y','z','roll','pitch','yaw','time')
    if not set(required).issubset(a.dtype.names): raise ValueError('Missing pose fields')
    r,p,y = [a[k].astype(float)/2 for k in ('roll','pitch','yaw')]
    cr,cp,cy,sr,sp,sy = np.cos(r),np.cos(p),np.cos(y),np.sin(r),np.sin(p),np.sin(y)
    q = np.column_stack((sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy))
    out = np.column_stack((a['time'],a['x'],a['y'],a['z'],q))
    if not np.isfinite(out).all() or len(out)<2 or (np.diff(out[:,0])<=0).any(): raise ValueError('Invalid/nonmonotonic poses')
    return out

def convert(source, target):
    a = pose_to_tum(read_pcd(source))
    np.savetxt(target,a,fmt='%.9f',header='timestamp tx ty tz qx qy qz qw')
    return a

if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('source'); p.add_argument('target'); args=p.parse_args()
    convert(args.source,args.target)
