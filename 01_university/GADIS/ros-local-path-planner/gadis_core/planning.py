"""GADIS Planning 핵심 재구성. 이전 코드 원문이 아닌 기록 기반 독립 구현."""
from __future__ import annotations
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from math import atan2, cos, hypot, isfinite, pi, sin, tan

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


def to_local(point: Pose2D, origin: Pose2D) -> Pose2D:
    """map→base_link: yaw를 포함한 정방향 좌표 변환."""
    dx, dy = point.x - origin.x, point.y - origin.y
    c, s = cos(origin.yaw), sin(origin.yaw)
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, point.yaw - origin.yaw)


def to_global(point: Pose2D, origin: Pose2D) -> Pose2D:
    c, s = cos(origin.yaw), sin(origin.yaw)
    return Pose2D(origin.x + c * point.x - s * point.y,
                  origin.y + s * point.x + c * point.y, point.yaw + origin.yaw)


def cubic_bezier(control_points: list[Pose2D], n: int = 160) -> list[Pose2D]:
    """4개 제어점의 3차 Bézier, 접선에서 heading 산출."""
    if len(control_points) != 4 or n < 2:
        raise ValueError('4 control points and at least 2 samples required')
    p0, p1, p2, p3 = control_points
    result = []
    for k in range(n):
        t = k / (n - 1)
        s = 1 - t
        x = s**3*p0.x + 3*s*s*t*p1.x + 3*s*t*t*p2.x + t**3*p3.x
        y = s**3*p0.y + 3*s*s*t*p1.y + 3*s*t*t*p2.y + t**3*p3.y
        dx = 3*s*s*(p1.x-p0.x) + 6*s*t*(p2.x-p1.x) + 3*t*t*(p3.x-p2.x)
        dy = 3*s*s*(p1.y-p0.y) + 6*s*t*(p2.y-p1.y) + 3*t*t*(p3.y-p2.y)
        result.append(Pose2D(x, y, atan2(dy, dx) if hypot(dx, dy) > 1e-9 else 0.0))
    return result


def _angle_wrap(value: float) -> float:
    return (value + pi) % (2*pi) - pi


def hybrid_astar(
    start: Pose2D,
    goal: Pose2D,
    obstacles: set[tuple[int, int]],
    resolution: float = 0.5,
    wheelbase: float = 1.0,
    step: float = 0.5,
    max_steer: float = 0.45,
    heading_bins: int = 36,
    max_iterations: int = 30000,
    goal_tolerance: float = 0.7,
) -> list[Pose2D]:
    """교육용 Hybrid A* (x,y,yaw,gear). 실제 차량용 충돌검증/최적화 미포함.

    obstacles는 map 좌표의 격자 점유 셀로 간주하며 차폭 팽창은 호출측에서 수행.
    """
    if (not all(isfinite(v) for v in (start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw, resolution, wheelbase, step, max_steer, goal_tolerance))
            or min(resolution, wheelbase, step, goal_tolerance) <= 0
            or max_steer <= 0 or max_steer >= pi / 2
            or heading_bins < 4 or max_iterations <= 0):
        raise ValueError('invalid planner configuration')
    def key(p: Pose2D):
        return (round(p.x / resolution), round(p.y / resolution),
                int(round((_angle_wrap(p.yaw) + pi)/(2*pi)*heading_bins)) % heading_bins)
    if ((round(start.x / resolution), round(start.y / resolution)) in obstacles or
            (round(goal.x / resolution), round(goal.y / resolution)) in obstacles):
        return []
    sequence = count()
    nodes: list[tuple[Pose2D, int | None, int]] = [(start, None, 1)]
    opened = [(hypot(start.x-goal.x,start.y-goal.y), next(sequence), 0, 0.)]
    best = {(key(start), 1): 0.0}
    iterations = 0
    while opened and iterations < max_iterations:
        _, _, index, g = heappop(opened)
        pose, _, prev_gear = nodes[index]
        if g > best.get((key(pose), prev_gear), float('inf')) + 1e-9:
            continue
        if hypot(pose.x-goal.x, pose.y-goal.y) <= goal_tolerance and abs(_angle_wrap(pose.yaw-goal.yaw)) < 0.6:
            path = []
            while index is not None:
                p, parent, _ = nodes[index]
                path.append(p)
                index = parent
            return path[::-1]
        for gear in (1, -1):
            for steer in (-max_steer, 0., max_steer):
                dist = step * gear
                theta = pose.yaw + dist * tan(steer) / wheelbase
                new = Pose2D(pose.x + dist * cos((pose.yaw+theta)/2),
                             pose.y + dist * sin((pose.yaw+theta)/2), _angle_wrap(theta))
                blocked = False
                for frac in (0.25,0.5,0.75,1.0):
                    sx = pose.x + frac*(new.x-pose.x)
                    sy = pose.y + frac*(new.y-pose.y)
                    if (round(sx/resolution), round(sy/resolution)) in obstacles:
                        blocked = True
                        break
                if blocked:
                    continue
                ng = g + step * (1.3 if gear == -1 else 1) + (0.5 if gear != prev_gear else 0) + abs(steer)*0.05
                node_key = (key(new), gear)
                if ng >= best.get(node_key, float('inf')) - 1e-9:
                    continue
                best[node_key] = ng
                nodes.append((new,index,gear))
                ni = len(nodes)-1
                heappush(opened,(ng + hypot(new.x-goal.x,new.y-goal.y), next(sequence),ni,ng))
        iterations += 1
    return []
