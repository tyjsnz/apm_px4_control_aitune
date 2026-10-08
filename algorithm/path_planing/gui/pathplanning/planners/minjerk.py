"""算法7: 最小jerk(五次多项式)轨迹 —— 段式五次多项式连接航路点。

原理
----
每个航路点之间用一条五次多项式 p(t)=a0+a1t+..+a5t^5 连接，要求：
    * 过所有航路点；
    * 拼接处位置/速度/加速度/加加速度(C3)连续；
    * 起终点速度、加速度为 0；
在满足上述等式约束下，最小化目标 J = ∫ (d³p/dt³)² dt (jerk 平方积分)。

这是"二次目标 + 线性等式约束"的 QP，用 KKT 系统
    [ 2Q  Aᵀ ] [c ]   [0]
    [ A   0  ] [λ ] = [b]
直接用 numpy 求解，无需第三方优化器；x/y 两轴解耦各解一次。

无障时航路点只有起终两点 -> 单条五次多项式，几何轨迹是完美直线，
但速度/加速度剖面平滑(两端为 0)，这正是高速飞行体期望的走廊；
有障时给出 C3 连续、jerk 最小的绕行曲线。
"""
from __future__ import annotations

import math

import numpy as np

from ..space import path_is_free, path_length, resample
from .astar import AStarPlanner
from .base import Planner

V_CRUISE = 30.0     # 巡航速度假设(m/s), 用于时间分配
A_MAX = 8.0         # 最大加速度(m/s²)


def _jerk_Q(T: float, m: int = 24) -> np.ndarray:
    """单段 Q = ∫ j jᵀ dt, j(t) = [0,0,0,6,24t,60t²] (五次基函数的三阶导)。"""
    t = np.linspace(0.0, T, m)
    J = np.zeros((m, 6))
    J[:, 3] = 6.0
    J[:, 4] = 24.0 * t
    J[:, 5] = 60.0 * t * t
    dt = T / (m - 1) if m > 1 else T
    w = np.full(m, dt)
    w[0] = w[-1] = dt / 2.0                      # 梯形积分
    return (J * w[:, None]).T @ J


def _build_constraints(W, T):
    """按固定行序构造约束矩阵 A 与右端项 (bx, by)。"""
    n = len(W) - 1
    n_eq = 5 * n + 1
    A = np.zeros((n_eq, 6 * n))
    bx = np.zeros(n_eq)
    by = np.zeros(n_eq)
    r = 0

    def setrow(i, f, vx=0.0, vy=0.0):
        nonlocal r
        A[r, 6 * i: 6 * i + 6] = f
        bx[r], by[r] = vx, vy
        r += 1

    # 起终点速度、加速度 = 0
    setrow(0, [0, 1, 0, 0, 0, 0])
    setrow(0, [0, 0, 2, 0, 0, 0])
    Tl = T[-1]
    setrow(n - 1, [0, 1, 2 * Tl, 3 * Tl**2, 4 * Tl**3, 5 * Tl**4])
    setrow(n - 1, [0, 0, 2, 6 * Tl, 12 * Tl**2, 20 * Tl**3])
    # 各段端点位置 = 航路点
    for i in range(n):
        Ti = T[i]
        setrow(i, [1, 0, 0, 0, 0, 0], W[i, 0], W[i, 1])
        setrow(i, [1, Ti, Ti**2, Ti**3, Ti**4, Ti**5], W[i + 1, 0], W[i + 1, 1])
    # 内部节点 C1/C2/C3 连续 (右端项为 0)
    for i in range(n - 1):
        Ti = T[i]
        pairs = (
            ([0, 1, 2 * Ti, 3 * Ti**2, 4 * Ti**3, 5 * Ti**4], [0, 1, 0, 0, 0, 0]),
            ([0, 0, 2, 6 * Ti, 12 * Ti**2, 20 * Ti**3], [0, 0, 2, 0, 0, 0]),
            ([0, 0, 0, 6, 24 * Ti, 120 * Ti**2], [0, 0, 0, 6, 0, 0]),
        )
        for f_end, f_start in pairs:
            A[r, 6 * i: 6 * i + 6] = f_end
            A[r, 6 * (i + 1): 6 * (i + 1) + 6] = -np.asarray(f_start, dtype=float)
            r += 1
    assert r == n_eq, (r, n_eq)
    return A, bx, by


def minjerk_trajectory(waypoints, v_cruise=V_CRUISE, a_max=A_MAX):
    """段式五次多项式最小jerk轨迹。

    返回 (密集采样点 (M,2), 总时间 s, 最大速度 m/s)。
    """
    W = np.asarray(waypoints, dtype=float)
    n = len(W) - 1
    if n < 1:
        return W.copy(), 0.0, 0.0

    d = np.linalg.norm(np.diff(W, axis=0), axis=1)
    # 时间分配: 受巡航速度与加减速距离双重限制
    T = np.maximum(d / v_cruise, 2.0 * np.sqrt(np.maximum(d, 1e-6) / a_max))

    A, bx, by = _build_constraints(W, T)
    n_eq = A.shape[0]
    N = 6 * n
    Q = np.zeros((N, N))
    for i in range(n):
        Q[6 * i: 6 * i + 6, 6 * i: 6 * i + 6] = _jerk_Q(float(T[i]))
    eps = 1e-9 * max(float(np.trace(Q)), 1.0)
    KKT = np.block([[2 * Q + eps * np.eye(N), A.T],
                    [A, np.zeros((n_eq, n_eq))]])

    rhs = np.zeros((N + n_eq, 2))
    rhs[N:, 0] = bx
    rhs[N:, 1] = by
    sol = np.linalg.solve(KKT, rhs)
    C = sol[:N].reshape(n, 6, 2)                 # (段, 5次系数, 轴)

    pts, total_t = [], 0.0
    for i in range(n):
        m = max(8, int(np.linalg.norm(W[i + 1] - W[i])))
        ts = np.linspace(0.0, float(T[i]), m)
        basis = np.stack([ts**k for k in range(6)], axis=1)
        seg = basis @ C[i]
        pts.append(seg[1:] if i else seg)
        total_t += float(T[i])
    P = np.vstack(pts)

    v = np.linalg.norm(np.diff(P, axis=0), axis=1)
    dt = total_t / max(len(P) - 1, 1)
    vmax = float(v.max() / dt) if len(P) > 1 else 0.0
    return P, total_t, vmax


class MinJerkPlanner(Planner):
    algo = "minjerk"
    label = "最小jerk五次多项式"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        if self.obstacles:
            ref = AStarPlanner(self.frame, self.obstacles, safety_m=self.safety,
                               **self.params).plan_raw(start, goal, None, None)
            # 航路点沿参考路径取, 最多约 40 个(控制 KKT 规模)
            waypoints = resample(ref, ds=max(1.0, path_length(ref) / 39.0))
        else:
            waypoints = np.vstack([start, goal])

        P, T, vmax = minjerk_trajectory(waypoints)
        if self.obstacles and not path_is_free(P, self.obstacles, self.safety):
            # 多项式在弯道处切割进禁飞区 -> 退回 A* 参考折线
            self._info = {"note": "多项式角点切割越界, 已退回A*参考折线",
                          "waypoints": int(len(waypoints))}
            return waypoints
        self._info = {"waypoints": int(len(waypoints)), "total_time_s": round(T, 2),
                      "vmax_mps": round(vmax, 2)}
        return P

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
