"""算法8: 粒子群优化(PSO)路径优化 —— 智能优化方法。

原理
----
把路径表示成"控制点序列"(起终点固定)，控制点之间用 Catmull-Rom 样条插值
成光滑曲线。目标函数(越小越好)：

    J = w_len  * 路径长度
      + w_bend * Σ(相邻段航向差)²      -- 打折项, 惩罚之字形/急转弯
      + w_col  * Σ max(0, 禁飞余量)²   -- 碰撞惩罚

粒子群在控制点空间里搜索：每个粒子携带一组控制点(12 维)，按个体最优 pbest
与群体最优 gbest 更新速度/位置，迭代收敛。

与 A*/RRT 这类"先搜索后平滑"不同，PSO 直接在连续空间里同时优化
"短 + 平滑 + 安全"，无障时最优解自然退化为直线。
"""
from __future__ import annotations

import numpy as np

from ..space import path_is_free, path_min_clearance
from .astar import AStarPlanner
from .base import Planner, workspace_bounds

N_CTRL = 8                 # 控制点总数(首尾固定 => 6 个自由点, 12 个变量)


def catmull_rom(P: np.ndarray, per_seg: int = 16) -> np.ndarray:
    """过控制点的 Catmull-Rom 样条密集采样。"""
    P = np.asarray(P, dtype=float)
    n = len(P)
    if n < 2:
        return P.copy()
    if n == 2:
        t = np.linspace(0.0, 1.0, 2 * per_seg)[:, None]
        return P[0] * (1 - t) + P[1] * t
    ext = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(n - 1):
        p0, p1, p2, p3 = ext[i], ext[i + 1], ext[i + 2], ext[i + 3]
        t = np.linspace(0.0, 1.0, per_seg, endpoint=(i == n - 2))[:, None]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                          + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                          + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    return np.vstack(out)


class PSOPlanner(Planner):
    algo = "pso"
    label = "粒子群优化(PSO)"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        p = self.params
        n_particles = int(p.get("particles", 50))
        n_iters = int(p.get("iters", 200))
        w_len = float(p.get("w_len", 1.0))
        w_bend = float(p.get("w_bend", 50.0))
        w_col = float(p.get("w_col", 1000.0))
        lo, hi = workspace_bounds(start, goal, self.obstacles, margin=40.0)

        n_free = N_CTRL - 2
        dim = n_free * 2
        lo_v = np.tile(lo, n_free)          # 变量维(lo/hi 广播到 12 维)
        hi_v = np.tile(hi, n_free)
        rng = self.rng

        def decode(x: np.ndarray) -> np.ndarray:
            ctrl = np.empty((N_CTRL, 2))
            ctrl[0], ctrl[-1] = start, goal
            ctrl[1:-1] = x.reshape(n_free, 2)
            return ctrl

        def cost(x: np.ndarray, col_w: float) -> float:
            path = catmull_rom(decode(x), per_seg=16)
            d = np.diff(path, axis=0)
            seg = np.linalg.norm(d, axis=1)
            L = float(seg.sum())
            heading = np.arctan2(d[:, 1], d[:, 0])
            bend = float(np.sum(np.diff(heading) ** 2))
            pen = 0.0
            for ob in self.obstacles:
                dd = np.linalg.norm(path - ob.center, axis=1)
                over = np.maximum(0.0, (ob.radius_m + self.safety) - dd)
                pen += float((over ** 2).sum())
            return w_len * L + w_bend * bend + col_w * pen

        # --- PSO 初始化 ---
        # 1/3 直线+扰动, 1/3 A*引导(有障时), 1/3 均匀随机(保证全局探索)
        straight = np.linspace(0, 1, N_CTRL)[1:-1, None] * (goal - start) + start
        spread = max(float(np.linalg.norm(goal - start)) * 0.15, 20.0)
        seed_ctrl = None
        if self.obstacles:
            ref = AStarPlanner(self.frame, self.obstacles, safety_m=self.safety,
                               **self.params).plan_raw(start, goal, None, None)
            seg = np.concatenate([[0.0], np.cumsum(
                np.linalg.norm(np.diff(ref, axis=0), axis=1))])
            tgt = np.linspace(0.0, seg[-1], N_CTRL)
            seed_ctrl = np.stack([np.interp(tgt, seg, ref[:, 0]),
                                  np.interp(tgt, seg, ref[:, 1])], axis=1)[1:-1]

        third = n_particles // 3
        Xs = []
        Xs.append(straight.ravel() + rng.normal(0, spread / 4, (third, dim)))
        if seed_ctrl is not None:
            Xs.append(seed_ctrl.ravel() + rng.normal(0, spread / 8, (third, dim)))
            Xs.append(rng.uniform(lo_v, hi_v, (n_particles - 2 * third, dim)))
        else:
            Xs.append(straight.ravel() + rng.normal(0, spread / 2, (third, dim)))
            Xs.append(rng.uniform(lo_v, hi_v, (n_particles - 2 * third, dim)))
        X = np.clip(np.vstack(Xs), lo_v, hi_v)
        V = rng.uniform(-(hi_v - lo_v), (hi_v - lo_v), (n_particles, dim)) * 0.05

        def repair(x: np.ndarray) -> np.ndarray:
            """把落在禁飞圆内的控制点沿径向推出到圆外(留 5 m 余量), 无违规时为恒等。"""
            ctrl = decode(x).copy()
            for i in range(1, N_CTRL - 1):
                for ob in self.obstacles:
                    dv = ctrl[i] - ob.center
                    r = ob.radius_m + self.safety + 5.0
                    n = float(np.linalg.norm(dv))
                    if n < r:
                        ctrl[i] = ob.center + (dv / n if n > 1e-9
                                               else np.array([1.0, 0.0])) * r
            return ctrl[1:-1].ravel()

        col_w = w_col
        for _attempt in range(3):                     # 约束不满足时加重惩罚重跑
            Pbest = X.copy()
            Pb = np.array([cost(x, col_w) for x in Pbest])
            k0 = int(np.argmin(Pb))
            Pbest_g, Pbest_gb = Pbest[k0].copy(), Pb[k0]
            for _ in range(n_iters):
                r1 = rng.random((n_particles, dim))
                r2 = rng.random((n_particles, dim))
                V = (0.72 * V
                     + 1.49 * r1 * (Pbest - X)
                     + 1.49 * r2 * (Pbest_g - X))
                V = np.clip(V, -(hi_v - lo_v) * 0.2, (hi_v - lo_v) * 0.2)
                X = np.clip(X + V, lo_v, hi_v)
                C = np.array([cost(x, col_w) for x in X])
                better = C < Pb
                Pbest[better] = X[better]
                Pb[better] = C[better]
                k = int(np.argmin(Pb))
                if Pb[k] < Pbest_gb:
                    Pbest_g, Pbest_gb = Pbest[k].copy(), Pb[k]
            # 约束修复: 越界控制点径向推出(恒等或可行化)
            Pbest_g = repair(Pbest_g)
            Pbest_gb = cost(Pbest_g, col_w)
            if path_is_free(catmull_rom(decode(Pbest_g), per_seg=16),
                            self.obstacles, self.safety):
                break
            col_w *= 10.0
            X[:third] = np.clip(straight.ravel()
                                + rng.normal(0, spread / 4, (third, dim)), lo_v, hi_v)

        path = catmull_rom(decode(Pbest_g), per_seg=max(
            8, int(np.linalg.norm(goal - start) / 60.0)))
        self._info = {"best_cost": round(float(Pbest_gb), 2),
                      "particles": n_particles, "iters": n_iters,
                      "col_weight": col_w,
                      "min_clearance_m": round(path_min_clearance(path, self.obstacles), 2)
                      if self.obstacles else None}
        return path

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
