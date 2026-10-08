"""算法5: Dubins 曲线 —— 定转弯半径(最小转弯半径 R)约束下的最短路径。

原理
----
固定翼/垂起飞机在高速前飞模式下不能原地转弯，只能以半径 >= R 的圆弧转向。
Dubins(1957)证明：满足曲率约束 |k| <= 1/R 的两条切线圆弧+直线组合中，
最短路径一定属于以下 6 种结构之一：

    LSL  左弧-直线-左弧        RSR  右弧-直线-右弧
    LSR  左弧-直线-右弧        RSL  右弧-直线-左弧
    LRL  左弧-右弧-左弧(小半径)  RLR  右弧-左弧-右弧(小半径)

本实现采用几何构造法：分别以起终点为圆心做半径 R 的左/右转弯圆，
求两圆的外公切线(LSL/RSR)与内公切线(LSR/RSL)，以及三圆弧结构
(RLR/LRL，仅当两圆心距离 <= 4R 时存在)，枚举全部可行解取最短者。

输入航向：命令行的航向角是地理航向(正北=0°、顺时针)，
本函数内部换算为数学角(正东=0°、逆时针)。
"""
from __future__ import annotations

import math

import numpy as np

from .base import Planner

TWO_PI = 2.0 * math.pi
mod2pi = lambda x: x % TWO_PI
rot90 = lambda v: np.array([-v[1], v[0]])      # 逆时针旋转90°
rotm90 = lambda v: np.array([v[1], -v[0]])     # 顺时针旋转90°


def _sweep(h0: float, h1: float, dirn: int) -> float:
    """从 h0 转到 h1 所扫过的角度(dirn=+1 逆时针/-1 顺时针), 取 [0, 2π)。"""
    return mod2pi(dirn * (h1 - h0))


def _heading_on(c, T, R, dirn) -> float:
    """圆上 T 点沿 dirn 方向运动时的航向(数学角)。"""
    rad = (np.asarray(T, float) - np.asarray(c, float)) / R
    v = rot90(rad) if dirn > 0 else rotm90(rad)
    return math.atan2(v[1], v[0])


def _external_tangent(c1, c2, R, side):
    """等半径两圆的外公切线; side=+1 圆心在行进方向左侧(左转圆), -1 右侧。"""
    d = np.asarray(c2, float) - np.asarray(c1, float)
    dist = float(np.linalg.norm(d))
    if dist < 1e-9:
        return None
    u = d / dist
    n = rot90(u) if side > 0 else rotm90(u)
    T1 = np.asarray(c1, float) - R * n
    T2 = np.asarray(c2, float) - R * n
    lam = float((T2 - T1) @ u)
    if lam <= 1e-9:
        return None
    return u, T1, T2, lam


def _internal_tangent(c1, c2, R, side1):
    """等半径两圆的内公切线(行进时圆心分居两侧)。
    由 cross(c2-c1, u) = 2R*side1 解出方向 u 的两个候选。"""
    d = np.asarray(c2, float) - np.asarray(c1, float)
    dist = float(np.linalg.norm(d))
    C = 2.0 * R * side1
    if dist < abs(C) + 1e-9:
        return []                                   # 内公切线不存在
    delta = math.atan2(d[1], d[0])
    s = math.asin(C / dist)
    out = []
    side2 = -side1
    for th in (delta + s, delta + math.pi - s):
        u = np.array([math.cos(th), math.sin(th)])
        n1 = rot90(u) if side1 > 0 else rotm90(u)
        n2 = rot90(u) if side2 > 0 else rotm90(u)
        T1 = np.asarray(c1, float) - R * n1
        T2 = np.asarray(c2, float) - R * n2
        lam = float((T2 - T1) @ u)
        if lam > 1e-9:
            out.append((u, T1, T2, lam))
    return out


def _middle_circle(c1, c2, R):
    """RLR/LRL 的中间圆: 与两圆外切(|Cm-c|=2R)，返回两个候选圆心。"""
    d = np.asarray(c2, float) - np.asarray(c1, float)
    dist = float(np.linalg.norm(d))
    if dist < 1e-9 or dist > 4.0 * R + 1e-9:
        return []
    M = (np.asarray(c1, float) + np.asarray(c2, float)) / 2.0
    a = dist / 2.0
    h = math.sqrt(max((2.0 * R) ** 2 - a * a, 0.0))
    e = d / dist
    perp = rot90(e)
    return [M + h * perp, M - h * perp]


def dubins_candidates(p1, th1, p2, th2, R):
    """枚举全部 Dubins 候选解。

    返回 [(总长, [(类型,'L'/'R'/'S', 长度), ...]), ...]，按总长升序。
    th1/th2 为数学角(弧度)。
    """
    p1 = np.asarray(p1, float)
    p2 = np.asarray(p2, float)
    n1s = np.array([math.cos(th1), math.sin(th1)])
    n2s = np.array([math.cos(th2), math.sin(th2)])
    CL1, CR1 = p1 + R * rot90(n1s), p1 + R * rotm90(n1s)
    CL2, CR2 = p2 + R * rot90(n2s), p2 + R * rotm90(n2s)
    cands = []

    def _push(segs):
        """圆弧段换算为弧长(米), 与直线段统一单位后再排序。"""
        segs = [(k, (v if k == "S" else v * R)) for k, v in segs]
        cands.append((sum(v for _, v in segs), segs))

    # --- 4 种切线型: LSL / RSR / LSR / RSL ---
    for kind, c1, c2, side in (("LSL", CL1, CL2, +1), ("RSR", CR1, CR2, -1)):
        tan = _external_tangent(c1, c2, R, side)
        if tan:
            u, T1, T2, lam = tan
            dirn = side
            h1 = _heading_on(c1, T1, R, dirn)
            segs = [((kind[0]), _sweep(th1, h1, dirn)),
                    ("S", lam),
                    ((kind[2]), _sweep(float(math.atan2(u[1], u[0])), th2, dirn))]
            _push(segs)

    for kind, c1, c2, side1 in (("LSR", CL1, CR2, +1), ("RSL", CR1, CL2, -1)):
        for u, T1, T2, lam in _internal_tangent(c1, c2, R, side1):
            d_start = kind[0]
            d_goal = kind[2]
            dir1 = side1
            dir2 = -side1
            h1 = _heading_on(c1, T1, R, dir1)
            segs = [(d_start, _sweep(th1, h1, dir1)),
                    ("S", lam),
                    (d_goal, _sweep(float(math.atan2(u[1], u[0])), th2, dir2))]
            _push(segs)

    # --- 2 种三圆弧型: RLR / LRL ---
    for kind, c1, c2 in (("RLR", CR1, CR2), ("LRL", CL1, CL2)):
        dir1 = +1 if kind[0] == "L" else -1
        dirm = -dir1
        dir2 = dir1
        for Cm in _middle_circle(c1, c2, R):
            u1 = Cm - c1
            u1 = u1 / np.linalg.norm(u1)
            P1 = c1 + R * u1
            u2 = Cm - c2
            u2 = u2 / np.linalg.norm(u2)
            P2 = c2 + R * u2
            h_start = _heading_on(c1, P1, R, dir1)
            ha = _heading_on(Cm, P1, R, dirm)
            hb = _heading_on(Cm, P2, R, dirm)
            h_goal = _heading_on(c2, P2, R, dir2)
            segs = [(kind[0], _sweep(th1, h_start, dir1)),
                    (kind[1], _sweep(ha, hb, dirm)),
                    (kind[2], _sweep(h_goal, th2, dir2))]
            _push(segs)

    cands.sort(key=lambda c: c[0])
    return cands


def sample_dubins(p1, th1, segs, R, ds=1.0):
    """按弧长 ds 采样生成路径点。"""
    pts = [np.asarray(p1, float).copy()]
    pos = np.asarray(p1, float).copy()
    h = th1
    for kind, val in segs:
        if kind == "S":
            n = max(1, int(math.ceil(val / ds)))
            step = val / n
            for _ in range(n):
                pos = pos + step * np.array([math.cos(h), math.sin(h)])
                pts.append(pos.copy())
        else:
            dirn = 1.0 if kind == "L" else -1.0
            c = pos + R * (rot90(np.array([math.cos(h), math.sin(h)])) if dirn > 0
                           else rotm90(np.array([math.cos(h), math.sin(h)])))
            phi0 = math.atan2(pos[1] - c[1], pos[0] - c[0])
            ang = val / R
            n = max(1, int(math.ceil(val / ds)))
            for i in range(1, n + 1):
                phi = phi0 + dirn * ang * i / n
                pts.append(c + R * np.array([math.cos(phi), math.sin(phi)]))
            h = h + dirn * ang
            pos = pts[-1].copy()
    return np.asarray(pts, dtype=float)


class DubinsPlanner(Planner):
    algo = "dubins"
    label = "Dubins曲线(定转弯半径)"
    need_heading = True

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        R = float(self.params.get("radius", 30.0))     # 最小转弯半径(米)
        ds = float(self.params.get("ds", 1.0))

        # 地理航向(度) -> 数学角(弧度)
        def to_math(hdeg, fallback):
            if hdeg is None:
                return fallback
            return math.radians(90.0 - float(hdeg))

        dx, dy = goal - start
        bearing_math = math.atan2(dy, dx)              # 起点->目标的数学角
        th1 = to_math(start_heading, bearing_math)
        th2 = to_math(goal_heading, th1)

        cands = dubins_candidates(start, th1, goal, th2, R)
        if not cands:
            raise RuntimeError("Dubins 无可行解(半径过大?)")
        best_len, best_segs = cands[0]
        pts = sample_dubins(start, th1, best_segs, R, ds=ds)
        if len(pts) < 2:
            pts = np.vstack([start, goal])
        self._info = {"radius_m": R, "structure": "".join(k for k, _ in best_segs),
                      "path_len_m": best_len, "n_candidates": len(cands)}
        return pts

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
