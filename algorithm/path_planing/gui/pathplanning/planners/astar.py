"""算法2: A* 栅格搜索 + 视线拉直(等价于 Theta*/String-Pulling 平滑)。

流程
----
1. 把规划空间离散成二维栅格(分辨率随航程自适应)；
2. 8 邻域 A*，代价 = 欧氏距离，启发式 = 直线距离(可采纳)；
3. 对搜索出的折线做"视线拉直"：贪心地把能直连的路点直接连起来，
   消除栅格搜索产生的之字形拐角 —— 这是 A* 输出平滑度的关键。
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .base import PlanningError, Planner, string_pull, workspace_bounds

SQRT2 = math.sqrt(2.0)


class AStarPlanner(Planner):
    algo = "astar"
    label = "A*栅格 + 视线拉直"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        obstacles, safety = self.obstacles, self.safety
        dist = float(np.linalg.norm(goal - start))
        res = float(np.clip(dist / 400.0, 2.0, 25.0))     # 栅格分辨率(米)
        lo, hi = workspace_bounds(start, goal, obstacles, margin=max(40.0, res * 2))
        nx = int(math.ceil((hi[0] - lo[0]) / res)) + 1
        ny = int(math.ceil((hi[1] - lo[1]) / res)) + 1
        while nx * ny > 400_000:                          # 限制规模, 航程很长时放粗
            res *= 2
            nx = int(math.ceil((hi[0] - lo[0]) / res)) + 1
            ny = int(math.ceil((hi[1] - lo[1]) / res)) + 1

        # --- 栅格占用: 距禁飞圆边界 >= safety + 半格 对角 才算可行 ---
        xs = lo[0] + np.arange(nx) * res
        ys = lo[1] + np.arange(ny) * res
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        free = np.ones((nx, ny), dtype=bool)
        infl = safety + 0.5 * res * SQRT2                 # 保守膨胀半个对角
        for ob in obstacles:
            d = np.hypot(gx - ob.xy[0], gy - ob.xy[1])
            free &= d >= (ob.radius_m + infl)

        def to_idx(p):
            i = int(round((p[0] - lo[0]) / res))
            j = int(round((p[1] - lo[1]) / res))
            return min(max(i, 0), nx - 1), min(max(j, 0), ny - 1)

        si, sj = to_idx(start)
        gi, gj = to_idx(goal)
        free[si, sj] = True                               # 起点/终点本身放开
        free[gi, gj] = True

        def h(i, j):
            dx, dy = abs(i - gi), abs(j - gj)
            return (dx + dy) + (SQRT2 - 2) * min(dx, dy)  # octile 启发式

        start_i = si * ny + sj
        goal_i = gi * ny + gj
        g = np.full(nx * ny, np.inf)
        parent = np.full(nx * ny, -1, dtype=np.int64)
        g[start_i] = 0.0
        openq = [(h(si, sj), 0.0, start_i)]
        closed = np.zeros(nx * ny, dtype=bool)
        nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
        found = False
        while openq:
            _, gc, cur = heapq.heappop(openq)
            if closed[cur]:
                continue
            closed[cur] = True
            if cur == goal_i:
                found = True
                break
            i, j = divmod(cur, ny)
            for di, dj, w in nbrs:
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny) or not free[ni, nj]:
                    continue
                # 禁止对角"穿角"
                if di and dj and not (free[i + di, j] and free[i, j + dj]):
                    continue
                k = ni * ny + nj
                ng = gc + w * res
                if ng < g[k]:
                    g[k] = ng
                    parent[k] = cur
                    heapq.heappush(openq, (ng + h(ni, nj), ng, k))
        if not found:
            raise PlanningError("A* 未找到可行路径(目标被禁飞区包围?)")

        # --- 回溯 ---
        chain, cur = [], goal_i
        while cur != -1:
            i, j = divmod(cur, ny)
            chain.append([lo[0] + i * res, lo[1] + j * res])
            if cur == start_i:
                break
            cur = parent[cur]
        pts = np.asarray(chain[::-1], dtype=float)
        pts = string_pull(pts, obstacles, safety=safety, step=max(res / 2, 1.0))
        self._info = {"grid_res_m": res, "grid_cells": int(nx * ny)}
        return pts

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
