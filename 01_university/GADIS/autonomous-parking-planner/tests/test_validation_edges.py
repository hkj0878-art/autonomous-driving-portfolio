"""Offline invariants for reconstructed, non-vehicle-certified planning code."""
from math import pi
import pytest
from gadis_core import Pose2D, to_local, to_global, cubic_bezier, hybrid_astar
from gadis_core.parking import ParkingGoals, ParkingStateMachine


def test_rotated_frame_roundtrip():
    origin = Pose2D(-10.2, 7.5, -pi / 3)
    value = Pose2D(3.4, 9.1, 0.8)
    result = to_global(to_local(value, origin), origin)
    assert result.x == pytest.approx(value.x, abs=1e-9)
    assert result.y == pytest.approx(value.y, abs=1e-9)
    assert result.yaw == pytest.approx(value.yaw, abs=1e-9)


def test_bezier_endpoints_and_invalid_count():
    pts = [Pose2D(0, 0), Pose2D(1, 2), Pose2D(2, 2), Pose2D(3, 0)]
    result = cubic_bezier(pts, n=160)
    assert len(result) == 160
    assert result[0].x == pytest.approx(0)
    assert result[-1].x == pytest.approx(3)
    with pytest.raises(ValueError):
        cubic_bezier(pts, n=1)


def test_hybrid_astar_avoids_occupied_cells():
    obs = {(round(1.5 / 0.5), round(0.0 / 0.5))}
    result = hybrid_astar(Pose2D(0, 0, 0), Pose2D(4, 0, 0), obs,
                          max_iterations=25000, goal_tolerance=0.7)
    assert result, 'planner should find a detour in open surroundings'
    for p in result:
        assert (round(p.x / 0.5), round(p.y / 0.5)) not in obs


def test_hybrid_astar_rejects_invalid_inputs_and_blocked_goal():
    with pytest.raises(ValueError):
        hybrid_astar(Pose2D(0, 0), Pose2D(1, 0), set(), wheelbase=0)
    with pytest.raises(ValueError):
        hybrid_astar(Pose2D(0, 0), Pose2D(1, 0), set(), max_steer=pi/2)
    assert hybrid_astar(Pose2D(0, 0), Pose2D(1, 0), {(2, 0)}) == []


def test_parking_state_machine_one_transition_per_update_and_threshold():
    g = ParkingGoals(Pose2D(1, 0), Pose2D(1, -1), Pose2D(2, -1))
    with pytest.raises(ValueError):
        ParkingStateMachine(g, arrive_m=0)
    m = ParkingStateMachine(g)
    assert m.update(Pose2D(1, 0)) == 'R'
    assert m.update(Pose2D(1, 0)) == 'R'
    assert m.update(Pose2D(1, -1)) == 'D'
    assert m.update(Pose2D(2, -1)) == 'P'
    assert m.update(Pose2D(2, -1)) == 'P'
