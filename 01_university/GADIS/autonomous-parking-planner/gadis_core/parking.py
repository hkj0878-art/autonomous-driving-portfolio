"""2025년 9월 대화의 mid/goal/goal2 고정, D→R→D→P 재구성.

목표점 산출은 원본 콘 기반 grid 알고리즘과 동일하다고 주장하지 않음.
"""
from dataclasses import dataclass
from math import cos, hypot, sin
from .planning import Pose2D, cubic_bezier

@dataclass(frozen=True)
class ParkingGoals:
    mid: Pose2D
    goal: Pose2D
    goal2: Pose2D


def build_parking_path(start: Pose2D, goals: ParkingGoals, n: int = 160) -> list[Pose2D]:
    # 초기 진입 직선 + 3차 Bézier + 자세 보정 직선의 개념 재구성.
    d = hypot(goals.mid.x-start.x,goals.mid.y-start.y)
    v = max(0.2, min(d/3, 2.0))
    cp = [start, Pose2D(start.x+v*cos(start.yaw),start.y+v*sin(start.yaw)),
          Pose2D(goals.mid.x-v*cos(goals.mid.yaw), goals.mid.y-v*sin(goals.mid.yaw)), goals.mid]
    approach = cubic_bezier(cp, n)
    n2 = max(2, n//4)
    reverse = [Pose2D(goals.mid.x+(goals.goal.x-goals.mid.x)*i/(n2-1),
                      goals.mid.y+(goals.goal.y-goals.mid.y)*i/(n2-1), goals.goal.yaw) for i in range(n2)]
    exit_seg = [Pose2D(goals.goal.x+(goals.goal2.x-goals.goal.x)*i/(n2-1),
                       goals.goal.y+(goals.goal2.y-goals.goal.y)*i/(n2-1), goals.goal2.yaw) for i in range(n2)]
    return approach + reverse[1:] + exit_seg[1:]

class ParkingStateMachine:
    """상태 변경은 거리 기반. 제어와 연결하려면 별도 안전 게이트 필요."""
    def __init__(self, goals: ParkingGoals, arrive_m: float=0.4):
        if arrive_m <= 0:
            raise ValueError('arrive_m must be positive')
        self.goals = goals
        self.arrive_m = arrive_m
        self.stage = 0
        self.gear = 'D'

    def update(self, ego: Pose2D) -> str:
        targets = [self.goals.mid, self.goals.goal, self.goals.goal2]
        if self.stage < 3:
            p = targets[self.stage]
            if hypot(ego.x-p.x, ego.y-p.y) <= self.arrive_m:
                self.stage += 1
                self.gear = ['D','R','D','P'][self.stage]
        return self.gear
