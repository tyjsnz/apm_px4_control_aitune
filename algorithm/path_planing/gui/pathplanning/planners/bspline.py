"""算法6: 三次B样条平滑 (参考路径 + 光顺)。

流程
----
1. 先用 A* 生成一条无碰撞的参考折线(无障时即直线)；
2. 用 scipy 的平滑样条 splprep(三次B样条) 拟合：s 越大越平滑但偏离参考路径越远；
3. 从"最平滑"开始逐级减小 s，做碰撞检测，取第一个无碰撞的平滑度。

本质是"拐角切割(corner-cutting)"：把 A*/折线的折角替换为曲率连续的样条曲线，
是工程上最常用的航迹平滑后处理手段。
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import splprep, splev

from ..space import path_is_free, resample
from .astar import AStarPlanner
from .base import Planner


class BSplinePlanner(Planner):
    algo = "bspline"
    label = "三次B样条平滑"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        # 1) 参考路径
        ref_planner = AStarPlanner(self.frame, self.obstacles, safety_m=self.safety,
                                   **self.params)
        ref = ref_planner.plan_raw(start, goal, None, None)
        ref = resample(ref, ds=max(5.0, float(np.linalg.norm(goal - start)) / 60.0))
        if len(ref) < 4:                       # 点太少补点(样条需要足够控制点)
            ref = resample(ref, ds=max(1.0, np.linalg.norm(goal - start) / 40.0))
        if len(ref) < 4:
            return ref

        dist = float(np.linalg.norm(goal - start))
        n = len(ref)
        # 平滑因子阶梯: 期望最大偏差约 2% 航程 起步
        s0 = max(n * (0.02 * dist) ** 2, 1e-6)
        ladder = [s0 * f for f in (1.0, 0.3, 0.1, 0.03, 0.01, 0.003, 0.001, 0.0)]
        used_s, pts = None, None
        for s in ladder:
            try:
                tck, _ = splprep([ref[:, 0], ref[:, 1]], s=float(s), k=3)
            except Exception:
                continue
            u = np.linspace(0.0, 1.0, max(200, int(dist)))
            x, y = splev(u, tck)
            cand = np.stack([x, y], axis=1)
            if path_is_free(cand, self.obstacles, self.safety):
                used_s, pts = s, cand
                break
        if pts is None:                        # 所有平滑度都撞: 退回参考折线
            self._info = {"used_s": None, "note": "样条平滑碰撞, 退回A*参考路径"}
            return ref
        self._info = {"used_s": float(used_s), "ref_points": int(n)}
        return pts

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
