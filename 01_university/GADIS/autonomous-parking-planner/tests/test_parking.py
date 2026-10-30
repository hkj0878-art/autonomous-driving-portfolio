from math import pi
from gadis_core import Pose2D, to_local, to_global, cubic_bezier, hybrid_astar
from gadis_core.parking import ParkingGoals, ParkingStateMachine, build_parking_path


def test_frame_roundtrip():
    p = Pose2D(11, 5, 0.7)
    origin = Pose2D(9, 1, pi/4)
    q = to_global(to_local(p, origin), origin)
    assert abs(p.x-q.x) < 1e-8 and abs(p.y-q.y) < 1e-8


def test_bezier_and_parking():
    g = ParkingGoals(Pose2D(2,0,0),Pose2D(2,-2,-pi/2),Pose2D(4,-2,0))
    path = build_parking_path(Pose2D(0,0,0),g)
    assert len(path) > 160
    assert abs(path[0].x) < 1e-8 and abs(path[-1].x-4)<1e-8
    state = ParkingStateMachine(g)
    assert state.update(Pose2D(0,0)) == 'D'
    assert state.update(g.mid) == 'R'
    assert state.update(g.goal) == 'D'
    assert state.update(g.goal2) == 'P'


def test_hybrid_astar_open_area():
    path = hybrid_astar(Pose2D(0,0),Pose2D(3,0),set(),max_iterations=4000)
    assert path and abs(path[-1].x-3) <= 0.7
