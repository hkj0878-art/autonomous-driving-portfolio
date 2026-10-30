"""Offline tests for reconstructed path tracking geometry."""
from math import pi
import pytest
from gadis_core import Pose2D, to_global, to_local
from gadis_core.control import pure_pursuit, menger_curvature, adaptive_lookahead


def test_turn_direction_and_lookahead_bounds():
    left = [Pose2D(1, 0.25), Pose2D(2, 1), Pose2D(3, 2)]
    right = [Pose2D(1, -0.25), Pose2D(2, -1), Pose2D(3, -2)]
    steer_left, _ = pure_pursuit(left, Pose2D(0, 0))
    steer_right, _ = pure_pursuit(right, Pose2D(0, 0))
    assert steer_left > 0
    assert steer_right < 0
    assert 1.5 <= adaptive_lookahead(100) <= 5


def test_curvature_collinear_and_invalid_input():
    assert menger_curvature(Pose2D(0, 0), Pose2D(1, 0), Pose2D(2, 0)) == pytest.approx(0)
    assert menger_curvature(Pose2D(1, 0), Pose2D(0, 1), Pose2D(-1, 0)) == pytest.approx(1)
    with pytest.raises(ValueError):
        pure_pursuit([], Pose2D(0, 0))
    with pytest.raises(ValueError):
        pure_pursuit([Pose2D(1, 0)], Pose2D(0, 0), wheelbase=0)


def test_coordinate_rotation_with_nonzero_heading():
    ego = Pose2D(10, -4, pi / 2)
    result = to_local(Pose2D(10, -1, 0), ego)
    assert result.x == pytest.approx(3)
    assert result.y == pytest.approx(0, abs=1e-9)
    back = to_global(result, ego)
    assert back.y == pytest.approx(-1)
