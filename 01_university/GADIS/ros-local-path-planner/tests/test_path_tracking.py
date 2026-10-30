from gadis_core import Pose2D

def test_pure_pursuit_straight_and_turn():
    from gadis_core.control import pure_pursuit, adaptive_lookahead
    straight=[Pose2D(x,0) for x in range(1,8)]
    steer, ld=pure_pursuit(straight, Pose2D(0,0))
    assert abs(steer)<1e-8 and ld>0
    assert adaptive_lookahead(0.6)<adaptive_lookahead(0.)
