import json
import numpy as np
import pytest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tools.record_traj import marker_pairs, finalize_loop_pairs


def test_upstream_marker_layout_and_final_matching(tmp_path):
    nodes=Marker(ns='loop_nodes',type=7)
    edge=Marker(ns='loop_edges',type=5,action=0)
    edge.header.frame_id='odom';edge.header.stamp.sec=42
    edge.points=[Point(x=5.01),Point(x=.02),Point(x=10.01),Point(x=5.02)]
    pairs=marker_pairs(MarkerArray(markers=[nodes,edge]),123.,42.)
    assert len(pairs)==2 and pairs[0]['endpoints'][0][0]==5.01
    assert pairs[0]['received_wall']==123. and pairs[0]['marker_stamp']==42.
    (tmp_path/'loop_pairs.json').write_text(json.dumps(dict(pairs=pairs,snapshots=[pairs])))
    np.savetxt(tmp_path/'keyframes_final_tum.txt',[[1,0,0,0,0,0,0,1],[2,5,0,0,0,0,0,1],[3,10,0,0,0,0,0,1]])
    result=finalize_loop_pairs(tmp_path)
    assert [p['keyframe_indices'] for p in result['pairs']]==[[1,0],[2,1]]
    np.testing.assert_allclose(result['pairs'][0]['matching_distances_m'],[.01,.02])
    assert result['pairs'][1]['keyframe_stamps']==[3,2]


def test_invalid_and_empty_markers():
    assert marker_pairs(MarkerArray(),1)==[]
    edge=Marker(ns='loop_edges',type=5,points=[Point()])
    with pytest.raises(ValueError,match='Odd'):marker_pairs(MarkerArray(markers=[edge]),1)
