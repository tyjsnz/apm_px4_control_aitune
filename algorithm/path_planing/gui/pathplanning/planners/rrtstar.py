"""算法3: RRT* 快速随机最优树 + 捷径平滑。

流程
----
1. 从起点不断随机撒点(带目标偏置)，从最近节点朝采样点延伸不超过 step；
2. 碰撞检测(线段按 2 m 采样检查禁飞圆)；
3. 在半径 r 内"选最优父节点 + rewire 改线"，使代价(累积长度)渐近最优；
4. 连到目标后回溯路径，再做随机捷径(shortcut)平滑消除绕路。
"""
from __future__ import annotations

import math

import numpy as np

from .base import PlanningError, Planner, shortcut_smooth, workspace_bounds


class RRTStarPlanner(Planner):
    algo = "rrtstar"
    label = "RRT* 采样最优树"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        obstacles, safety = self.obstacles, self.safety
        p = self.params
        dist = float(np.linalg.norm(goal - start))
        step = float(np.clip(dist / 20.0, 5.0, 40.0))
        max_iter = int(p.get("max_iter", 2500))
        goal_bias = float(p.get("goal_bias", 0.10))
        rewire_r = step * 3.0
        lo, hi = workspace_bounds(start, goal, obstacles, margin=60.0)

        def edge_free(a, b):
            d = float(np.linalg.norm(b - a))
            n = max(2, int(d / 2.0) + 1)
            t = np.linspace(0.0, 1.0, n)[:, None]
            pts = a[None, :] * (1 - t) + b[None, :] * t
            for ob in obstacles:
                if np.min(np.linalg.norm(pts - ob.center, axis=1)) < ob.radius_m + safety:
                    return False
            return True

        xs = [start.copy()]
        cost = [0.0]
        parent = [-1]
        reached_idx = -1
        best_cost = math.inf

        for it in range(max_iter):
            if self.rng.random() < goal_bias:
                q_rand = goal.copy()
            else:
                q_rand = self.rng.uniform(lo, hi)
            pts = np.asarray(xs)
            d = np.linalg.norm(pts - q_rand, axis=1)
            i_near = int(np.argmin(d))
            q_near = pts[i_near]
            v = q_rand - q_near
            nrm = float(np.linalg.norm(v))
            if nrm < 1e-9:
                continue
            q_new = q_near + v / nrm * min(step, nrm)
            if not edge_free(q_near, q_new):
                continue

            # --- RRT*: 选最优父节点 ---
            dd = np.linalg.norm(pts - q_new, axis=1)
            nbrs = np.where(dd <= rewire_r)[0]
            c_new = cost[i_near] + float(dd[i_near])
            best_p = i_near
            for j in nbrs:
                cj = cost[j] + float(dd[j])
                if cj < c_new and edge_free(pts[j], q_new):
                    best_p, c_new = int(j), cj
            xs.append(q_new)
            cost.append(c_new)
            parent.append(best_p)
            idx = len(xs) - 1

            # --- rewire: 用新节点改善邻居 ---
            for j in nbrs:
                cj = c_new + float(dd[j])
                if cj < cost[j] and edge_free(q_new, pts[j]):
                    parent[j] = idx
                    cost[j] = cj

            # --- 尝试连接目标 ---
            dg = float(np.linalg.norm(goal - q_new))
            if dg <= step * 1.5 and edge_free(q_new, goal):
                total = c_new + dg
                if total < best_cost:
                    best_cost = total
                    reached_idx = idx

        if reached_idx < 0:
            raise PlanningError(f"RRT* 在 {max_iter} 次迭代内未连接到目标")

        chain, cur = [], reached_idx
        while cur != -1:
            chain.append(xs[cur])
            cur = parent[cur]
        path = np.asarray(chain[::-1], dtype=float)
        path = np.vstack([start, path, goal])
        path = shortcut_smooth(path, obstacles, safety=safety, step=2.0, rounds=6)
        self._info = {"nodes": len(xs), "iterations": max_iter,
                      "raw_cost_m": best_cost, "step_m": step}
        return path

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
