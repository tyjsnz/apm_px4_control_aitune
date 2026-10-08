"""GPS(WGS-84 经纬度) 与 局部平面坐标(米) 的相互转换。

采用以参考点为中心的等距圆柱(equirectangular)投影：
    x = R * cos(lat0) * (lon - lon0)   (向东为正, 单位: 米)
    y = R * (lat - lat0)               (向北为正, 单位: 米)

在数公里尺度内该投影的长度/角度畸变 < 0.1%，对无人机路径规划完全够用。
所有算法都在局部平面坐标 (x, y) 中规划，最后再转回 GPS 输出。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6378137.0          # WGS-84 赤道半径
M_PER_DEG_LAT = 111132.92 - 559.82 * math.cos(2 * math.radians(34.0))  # 近似


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """两 GPS 点间的大圆距离(米)。"""
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dlat, dlon = rlat2 - rlat1, rlon2 - rlon1
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """从点1到点2的地理方位角(度, 正北=0, 顺时针为正)。"""
    rlat1, rlon1, rlat2, rlon2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dlon = rlon2 - rlon1
    y = math.sin(dlon) * math.cos(rlat2)
    x = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


@dataclass
class LocalFrame:
    """局部平面坐标系：原点取在 lat0/lon0，x 向东、y 向北，单位米。"""
    lat0: float
    lon0: float

    def to_xy(self, lat: float, lon: float) -> tuple[float, float]:
        x = EARTH_RADIUS_M * math.cos(math.radians(self.lat0)) * math.radians(lon - self.lon0)
        y = EARTH_RADIUS_M * math.radians(lat - self.lat0)
        return x, y

    def to_latlon(self, x: float, y: float) -> tuple[float, float]:
        lat = self.lat0 + math.degrees(y / EARTH_RADIUS_M)
        lon = self.lon0 + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(self.lat0))))
        return lat, lon

    @classmethod
    def from_points(cls, pts_latlon: list[tuple[float, float]]) -> "LocalFrame":
        """以所有点的质心为原点。"""
        lat0 = sum(p[0] for p in pts_latlon) / len(pts_latlon)
        lon0 = sum(p[1] for p in pts_latlon) / len(pts_latlon)
        return cls(lat0, lon0)
