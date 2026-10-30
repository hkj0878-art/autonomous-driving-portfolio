"""GADIS Pure Pursuit 곡률 기반 LD 예제(새 구현, 실제 액추에이터 구동 금지)."""
from math import atan, atan2, hypot, sin
from .planning import Pose2D, to_local


def menger_curvature(a: Pose2D,b: Pose2D,c: Pose2D) -> float:
    """3점 원 기반 곡률; 2025-07 Menger curvature 기록 반영."""
    ab=hypot(a.x-b.x,a.y-b.y);bc=hypot(b.x-c.x,b.y-c.y);ca=hypot(c.x-a.x,c.y-a.y)
    if min(ab,bc,ca)<1e-8: return 0.
    area2=abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))
    return 2*area2/(ab*bc*ca)


def adaptive_lookahead(kappa:float, min_ld:float=1.5,max_ld:float=5.0,gain:float=2.0) -> float:
    """곡률 커지면 LD 감소 (함수형은 최종 원문 미확인)."""
    return min(max_ld,max(min_ld, max_ld/(1+gain*abs(kappa))))


def pure_pursuit(path_map:list[Pose2D], ego_map:Pose2D,wheelbase:float=1.5) -> tuple[float,float]:
    """반환 (steer[rad], selected_lookahead[m]); map 경로는 내부에서 ego local 변환."""
    if not path_map or wheelbase<=0: raise ValueError('path and wheelbase required')
    pts=[to_local(p,ego_map) for p in path_map]
    if len(pts)>=3:
        kappa=menger_curvature(pts[0],pts[len(pts)//2],pts[-1])
    else: kappa=0.0
    ld=adaptive_lookahead(kappa)
    forward=[p for p in pts if p.x>0]
    if not forward: return 0.,ld
    target=min(forward,key=lambda p:abs(hypot(p.x,p.y)-ld))
    alpha=atan2(target.y,target.x)
    actual_ld=max(0.1,hypot(target.x,target.y))
    steer=atan(2*wheelbase*sin(alpha)/actual_ld)
    return steer,ld
