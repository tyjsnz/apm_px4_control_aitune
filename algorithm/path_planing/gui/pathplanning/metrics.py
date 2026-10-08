"""路径质量评估指标：直线度、平滑度、曲率、多余转弯等。

所有指标都在"等弧长重采样(默认 1 m)"后的路径上计算，保证不同算法可比。

指标定义
--------
length_m          路径总长度 (米)
direct_m          起终点直线距离 (米)
straightness      直线度 = direct / length，∈(0,1]，1 = 完美直线
excess_turn_deg   多余转弯 = Σ|Δ航向| - |总航向变化| (度)。
                  航向单调变化(哪怕绕大弯)时 ≈ 0；之字形抖动时急剧增大。
smoothness        平滑度 = exp(-excess_turn_deg / 45)，∈(0,1]，1 = 无抖动
max_curvature     最大曲率 (1/m)，1/它 = 最小转弯半径，用于判断飞机能不能飞
rms_curvature     均方根曲率 (1/m)
score             综合评分 = straightness * smoothness，用于排序"更趋向平滑直线"
"""
from __future__ import annotations

import math

import numpy as np

from .space import path_length, path_min_clearance, resample


def _smooth_heading(h: np.ndarray, w: int = 5) -> np.ndarray:
    """航向低通(窗口 w 个采样): 抑制亚度级数值抖动, 同时完整保留真实转弯
    (滑动平均对阶跃型转弯只展宽、不改变总转角)。"""
    if len(h) < w:
        return h
    half = w // 2
    hp = np.concatenate([np.full(half, h[0]), h, np.full(half, h[-1])])
    return np.convolve(hp, np.ones(w) / w, mode="valid")


def evaluate(pts: np.ndarray, obstacles=None, ds: float = 1.0) -> dict:
    pts = resample(np.asarray(pts, dtype=float), ds=ds)
    out: dict = {"n_points": int(len(pts))}
    if len(pts) < 2:
        out.update(dict(length_m=0.0, direct_m=0.0, straightness=0.0,
                        excess_turn_deg=0.0, smoothness=0.0, max_curvature=0.0,
                        rms_curvature=0.0, clearance_m=float("inf"),
                        score=0.0))
        return out

    length = path_length(pts)
    direct = float(np.linalg.norm(pts[-1] - pts[0]))
    straightness = direct / length if length > 1e-9 else 1.0

    d = np.diff(pts, axis=0)
    heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    heading_s = _smooth_heading(heading)
    turns = np.abs(np.diff(heading_s))
    total_turn = float(turns.sum())
    net_turn = abs(float(heading_s[-1] - heading_s[0]))
    excess = max(0.0, total_turn - net_turn)
    smoothness = math.exp(-math.degrees(excess) / 45.0)

    if len(pts) > 2:
        kappa = np.gradient(heading_s, ds)        # 曲率 = d航向/d弧长
        max_k = float(np.abs(kappa).max())
        rms_k = float(np.sqrt(np.mean(kappa ** 2)))
    else:
        max_k = rms_k = 0.0

    clearance = path_min_clearance(pts, obstacles) if obstacles else float("inf")

    out.update(
        length_m=length,
        direct_m=direct,
        straightness=straightness,
        excess_turn_deg=math.degrees(excess),
        smoothness=smoothness,
        max_curvature=max_k,
        min_turn_radius_m=(1.0 / max_k if max_k > 1e-9 else float("inf")),
        rms_curvature=rms_k,
        clearance_m=clearance,
        score=straightness * smoothness,
    )
    return out


def rank(results: dict[str, dict]) -> list[tuple[str, dict]]:
    """排序: 先按可行性(可行 > 带警告 > 失败), 同级按评分从高到低。
    评分 = 直线度 × 平滑度, 越接近 1 越接近『平滑直线』。"""
    order = {"ok": 0, "partial": 1, "warn": 1, "failed": 2}
    return sorted(results.items(),
                  key=lambda kv: (order.get(kv[1].get("status", "ok"), 0),
                                  -kv[1].get("score", 0.0)))
