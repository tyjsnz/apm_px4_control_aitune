"""所有规划器的公共基类与共享工具。"""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod

import numpy as np

from ..models import Obstacle, PlanResult
from ..space import los_free, path_is_free


class Planner(ABC):
    """规划器基类。

    子类只需实现 plan_raw()，返回 (N,2) 局部坐标数组。
    基类统一负责计时、结果封装、坐标校验。
    """
    algo: str = "base"          # 唯一标识(命令行用)
    label: str = "未命名算法"     # 中文名
    need_heading: bool = False  # 是否需要航向输入(Dubins)

    def __init__(self, frame, obstacles: list[Obstacle] | None = None,
                 safety_m: float = 0.0, seed: int | None = None, **params):
        self.frame = frame
        self.obstacles: list[Obstacle] = obstacles or []
        self.safety = safety_m
        self.params = params
        self.rng = np.random.default_rng(seed)

    # ---- 主入口 -------------------------------------------------------
    def plan(self, start_xy, goal_xy,
             start_heading: float | None = None,
             goal_heading: float | None = None) -> PlanResult:
        start = np.asarray(start_xy, dtype=float)
        goal = np.asarray(goal_xy, dtype=float)
        t0 = time.perf_counter()
        try:
            pts = self.plan_raw(start, goal, start_heading, goal_heading)
            status, msg = "ok", ""
        except PlanningError as e:
            pts, status, msg = np.array([start, goal]), "failed", str(e)
        except Exception as e:                       # pragma: no cover - 兜底
            pts, status, msg = np.array([start, goal]), "failed", f"算法异常: {e}"
        ms = (time.perf_counter() - t0) * 1000.0

        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        # 端点强制吸附到真实起终点
        if len(pts):
            pts[0], pts[-1] = start, goal
        if status == "ok" and not path_is_free(pts, self.obstacles, self.safety):
            status, msg = "warn", (msg + " 结果存在低于安全余量的航段。").strip()
        return PlanResult(algo=self.algo, label=self.label, points=pts,
                          status=status, message=msg, compute_ms=ms,
                          extra=self.last_extra())

    @abstractmethod
    def plan_raw(self, start: np.ndarray, goal: np.ndarray,
                 start_heading: float | None, goal_heading: float | None) -> np.ndarray:
        ...

    def last_extra(self) -> dict:
        return {}


class PlanningError(RuntimeError):
    """规划失败(不可达、陷入局部极小等)。"""


# ---- 共享工具函数 -------------------------------------------------------

def workspace_bounds(start, goal, obstacles, margin: float = 60.0):
    """规划空间范围：覆盖起终点与所有障碍并留出边距。"""
    pts = [np.asarray(start, float), np.asarray(goal, float)]
    for ob in obstacles:
        c, r = ob.center, ob.radius_m + margin
        pts.append(c - r)
        pts.append(c + r)
    lo = np.min(np.stack(pts), axis=0) - margin
    hi = np.max(np.stack(pts), axis=0) + margin
    return lo, hi


def los_path_free(a, b, obstacles, step: float = 2.0, safety: float = 0.0) -> bool:
    return los_free(a, b, obstacles, step=step, safety=safety)


def string_pull(pts: np.ndarray, obstacles, safety: float = 0.0,
                step: float = 2.0) -> np.ndarray:
    """“拉直”平滑：贪心保留仍能通视的最远路点，消除折线拐角。"""
    pts = np.asarray(pts, dtype=float)
    if len(pts) <= 2:
        return pts
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            if los_path_free(pts[i], pts[j], obstacles, step=step, safety=safety):
                break
            j -= 1
        out.append(pts[j])
        i = j
    return np.asarray(out, dtype=float)


def shortcut_smooth(pts: np.ndarray, obstacles, safety: float = 0.0,
                    step: float = 2.0, rounds: int = 4) -> np.ndarray:
    """随机捷径平滑(RRT 常用)：随机取两点若可直连则跳过中间点。"""
    pts = np.asarray(pts, dtype=float)
    if len(pts) <= 2:
        return pts
    rng = np.random.default_rng(0)
    for _ in range(rounds):
        changed, i = False, 0
        while i < len(pts) - 2:
            j = len(pts) - 1
            best = -1
            while j > i + 1:
                if los_path_free(pts[i], pts[j], obstacles, step=step, safety=safety):
                    best = j
                    break
                j -= 1
            if best > 0:
                pts = np.delete(pts, slice(i + 1, best), axis=0)
                i, changed = best, True
            else:
                i += 1
        if not changed:
            break
    return pts
