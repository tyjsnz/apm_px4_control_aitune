"""路径几何工具：折线长度、等弧长重采样、视线(LOS)通视检查、安全余量检查。"""
from __future__ import annotations

import numpy as np


def dedupe(pts: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """去掉连续重复点。"""
    if len(pts) == 0:
        return pts
    keep = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - keep[-1]) > eps:
            keep.append(p)
    return np.asarray(keep, dtype=float)


def seg_lengths(pts: np.ndarray) -> np.ndarray:
    """每段长度, shape (N-1,)。"""
    return np.linalg.norm(np.diff(pts, axis=0), axis=1)


def path_length(pts: np.ndarray) -> float:
    if len(pts) < 2:
        return 0.0
    return float(seg_lengths(pts).sum())


def resample(pts: np.ndarray, ds: float = 1.0) -> np.ndarray:
    """按弧长等间距重采样(间隔 ds 米)，用于统一评估指标。"""
    pts = dedupe(np.asarray(pts, dtype=float))
    if len(pts) < 2:
        return pts
    s = np.concatenate([[0.0], np.cumsum(seg_lengths(pts))])
    total = s[-1]
    if total < ds:
        return pts.copy()
    n = max(2, int(np.floor(total / ds)) + 1)
    s_new = np.linspace(0.0, total, n)
    x = np.interp(s_new, s, pts[:, 0])
    y = np.interp(s_new, s, pts[:, 1])
    return np.stack([x, y], axis=1)


def sample_segment(a: np.ndarray, b: np.ndarray, step: float) -> np.ndarray:
    """在线段 a-b 上按 step 间隔采样(含端点)。"""
    d = float(np.linalg.norm(b - a))
    n = max(2, int(np.ceil(d / step)) + 1)
    t = np.linspace(0.0, 1.0, n)
    return a[None, :] + t[:, None] * (b - a)[None, :]


def los_free(a, b, obstacles, step: float = 2.0, safety: float = 0.0) -> bool:
    """视线通视检查：线段 a-b 是否全程不进入任何禁飞圆(含安全余量)。"""
    if not obstacles:
        return True
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    pts = sample_segment(a, b, step)
    for ob in obstacles:
        c, r = ob.center, ob.radius_m + safety
        # 点到线段的最小距离: 先算投影参数, 再夹紧到 [0,1]
        ab = b - a
        denom = float(ab @ ab)
        if denom < 1e-12:
            t = 0.0
        else:
            t = np.clip((pts - a) @ ab / denom, 0.0, 1.0)
        proj = a + t[:, None] * ab
        dmin = float(np.linalg.norm(proj - c, axis=1).min())
        if dmin < r:
            return False
    return True


def path_min_clearance(pts: np.ndarray, obstacles) -> float:
    """路径到最近禁飞圆边界的最小距离(米)；无障碍返回 +inf。"""
    if not obstacles or len(pts) < 2:
        return float("inf")
    best = float("inf")
    for ob in obstacles:
        c, r = ob.center, ob.radius_m
        a = pts[:-1]
        b = pts[1:]
        ab = b - a
        denom = (ab * ab).sum(axis=1)
        denom[denom < 1e-12] = 1e-12
        t = np.clip(((c[None, :] - a) * ab).sum(axis=1) / denom, 0.0, 1.0)
        proj = a + t[:, None] * ab
        d = float(np.linalg.norm(proj - c, axis=1).min()) - r
        best = min(best, d)
    return best


def path_is_free(pts: np.ndarray, obstacles, safety: float = 0.0) -> bool:
    return path_min_clearance(pts, obstacles) >= safety
