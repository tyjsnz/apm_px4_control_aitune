"""算法注册表：algo 标识 -> 规划器类。"""
from __future__ import annotations

from .apf import APFPlanner
from .astar import AStarPlanner
from .base import Planner, PlanningError
from .bspline import BSplinePlanner
from .dubins import DubinsPlanner
from .minjerk import MinJerkPlanner
from .pso import PSOPlanner
from .rrtstar import RRTStarPlanner
from .straight import StraightPlanner

REGISTRY: dict[str, type[Planner]] = {
    p.algo: p for p in (
        StraightPlanner,
        AStarPlanner,
        RRTStarPlanner,
        APFPlanner,
        DubinsPlanner,
        BSplinePlanner,
        MinJerkPlanner,
        PSOPlanner,
    )
}


def available() -> list[tuple[str, str]]:
    return [(p.algo, p.label) for p in REGISTRY.values()]


__all__ = ["REGISTRY", "available", "Planner", "PlanningError"]
