"""数据模型：禁飞区障碍物、规划结果。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class Obstacle:
    """圆形禁飞区(禁飞圆)。坐标用 GPS，内部换算成局部平面。"""
    lat: float
    lon: float
    radius_m: float
    label: str = ""

    def __post_init__(self):
        self.xy: tuple[float, float] = (0.0, 0.0)  # 由 bind_frame 填充

    def bind_frame(self, frame) -> None:
        self.xy = frame.to_xy(self.lat, self.lon)

    @property
    def center(self) -> np.ndarray:
        return np.asarray(self.xy, dtype=float)

    def dist_to(self, p) -> float:
        """点 p 到禁飞圆边界的有符号距离(圆内为负)。"""
        return float(np.linalg.norm(np.asarray(p, dtype=float) - self.center) - self.radius_m)


@dataclass
class PlanResult:
    """一个算法的一次规划结果。"""
    algo: str                       # 算法标识
    label: str                      # 中文显示名
    points: np.ndarray              # (N,2) 局部平面坐标, 米
    status: str = "ok"              # ok / partial / failed
    message: str = ""               # 备注(局部极小、未找到路径等)
    compute_ms: float = 0.0         # 规划耗时(毫秒)
    extra: dict[str, Any] = field(default_factory=dict)  # 算法私有信息

    def gps_points(self, frame) -> list[tuple[float, float]]:
        """转成 GPS (lat, lon) 集合。"""
        return [frame.to_latlon(float(x), float(y)) for x, y in self.points]
