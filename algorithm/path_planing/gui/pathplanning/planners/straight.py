"""算法1: 直线航路(大圆/等距投影直线插值) —— 基准算法。"""
from __future__ import annotations

import numpy as np

from .base import Planner


class StraightPlanner(Planner):
    algo = "straight"
    label = "直线航路(基准)"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        d = float(np.linalg.norm(goal - start))
        n = int(max(50, min(2000, d / 2.0)))     # 约 2 米一个航路点
        t = np.linspace(0.0, 1.0, n)[:, None]
        return start[None, :] * (1 - t) + goal[None, :] * t
