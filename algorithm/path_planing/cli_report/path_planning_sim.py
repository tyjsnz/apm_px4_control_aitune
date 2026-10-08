#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径规划仿真：输入 地面端 -> 空中目标 GPS 坐标，运行 10 种路径规划算法并对比结果。

场景：起点(原点)=地面端，终点=空中目标；默认不考虑禁飞区。
     结果为二维水平投影航路（高度/爬升剖面不在本仿真范围）。

用法：
    python path_planning_sim.py --start 39.905,116.409 --end 39.970,116.480
    python path_planning_sim.py --start ... --end ... ^
        --start-hdg 60 --end-hdg 150 --turn-radius 500   (指定起飞/抵达航向)
    python path_planning_sim.py --start ... --end ... ^
        --obstacles "39.93,116.44,39.95,116.46"          (可选禁飞区)
    python path_planning_sim.py                           (缺省参数则交互式录入)

参数：
    --start/--end      "纬度,经度" 十进制度（地面端 / 空中目标）
    --start-hdg        地面端起飞航向(度,0=正北)，缺省=对准目标方位
    --end-hdg          目标点抵达航向(度,0=正北)，缺省=同起飞航向
    --turn-radius      Dubins 最小转弯半径(米)，默认 500
    --obstacles        矩形禁飞区，分号分隔："lat1,lon1,lat2,lon2;..."
    --grid-res         栅格分辨率(米)，缺省按直线距离自动
    --seed             随机种子(RRT/GA)，默认 42
    --outdir           输出目录，缺省为脚本所在目录
    --show-waypoints   在控制台打印每个算法的全部航点

输出（默认写到脚本所在目录）：
    planning_report.html   仿真报告：算法功能/描述、指标对比、路径图、航点表
    waypoints.csv          全部算法航点(GPS)：algorithm,seq,lat,lon,...
    waypoints.json         同上(JSON)

仅依赖 Python 标准库，不依赖本项目其它文件。
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import random
import sys
import time
import unicodedata
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

EARTH_R = 6371008.8  # 平均地球半径(米)


class PlanningError(RuntimeError):
    """某算法规划失败。"""


# ========================= 地理坐标工具 =========================
def haversine(lat1, lon1, lat2, lon2):
    """两点大圆距离(米)。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """起始方位角(度, 0=正北, 顺时针)。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def geo_to_enu(lat, lon, lat0, lon0):
    """经纬度 -> 以 (lat0,lon0) 为原点的局部平面坐标(米, x 东 y 北)。"""
    x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R
    y = math.radians(lat - lat0) * EARTH_R
    return x, y


def enu_to_geo(x, y, lat0, lon0):
    """局部平面坐标 -> 经纬度（geo_to_enu 的逆变换）。"""
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * max(1e-9, math.cos(math.radians(lat0)))))
    return lat, lon


# ========================= 平面几何工具 =========================
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_seg_dist(p, a, b):
    """点到线段的距离。"""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-18:
        return dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def heading_enu(a, b):
    """平面航向(度, 0=北, 顺时针)。"""
    return (math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) + 360.0) % 360.0


def angle_between(u, v):
    """两向量夹角(度, 0~180)。"""
    nu, nv = math.hypot(*u), math.hypot(*v)
    if nu < 1e-12 or nv < 1e-12:
        return 0.0
    c = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / (nu * nv)))
    return math.degrees(math.acos(c))


def simplify(points, eps):
    """去掉与前后点近似共线的中间点(垂距 <= eps 米则删除)。"""
    if len(points) < 3:
        return list(points)
    out = [points[0]]
    last = points[0]
    i = 1
    while i < len(points) - 1:
        if point_seg_dist(points[i], last, points[i + 1]) <= eps:
            i += 1
        else:
            out.append(points[i])
            last = points[i]
            i += 1
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out


def densify(points, max_gap):
    """航点加密：任一段长超过 max_gap 则插入中间点（不改变路径形状）。"""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        d = dist(a, b)
        if d > 1e-9:
            n = max(1, int(math.ceil(d / max_gap)))
            for k in range(1, n):
                t = k / n
                out.append((a[0] + (b[0] - a[0]) * t,
                            a[1] + (b[1] - a[1]) * t))
        out.append(b)
    return out


# ========================= 禁飞区(矩形) =========================
@dataclass
class Rect:
    minx: float
    miny: float
    maxx: float
    maxy: float

    def contains(self, p, margin=0.0):
        return (self.minx - margin <= p[0] <= self.maxx + margin
                and self.miny - margin <= p[1] <= self.maxy + margin)


def seg_intersects_rect(a, b, r):
    """线段 a-b 是否与矩形(含边界)相交——Liang-Barsky 裁剪。"""
    x0, y0 = a
    x1, y1 = b
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - r.minx), (dx, r.maxx - x0),
                 (-dy, y0 - r.miny), (dy, r.maxy - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            if t > t1:
                return False
            t0 = max(t0, t)
        else:
            if t < t0:
                return False
            t1 = min(t1, t)
    return t0 <= t1


def path_blocked(a, b, rects):
    return any(seg_intersects_rect(a, b, r) for r in rects)


def parse_obstacles(spec, lat0, lon0):
    """"lat1,lon1,lat2,lon2;..." -> ENU Rect 列表（校验取值范围）。"""
    rects = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(v) for v in chunk.replace("，", ",").split(",")]
        if len(parts) != 4:
            raise ValueError(f"禁飞区格式错误: {chunk}（应为 lat1,lon1,lat2,lon2）")
        la1, lo1, la2, lo2 = parts
        if not (-90 <= la1 <= 90 and -90 <= la2 <= 90):
            raise ValueError(f"禁飞区纬度超范围: {chunk}")
        if not (-180 <= lo1 <= 180 and -180 <= lo2 <= 180):
            raise ValueError(f"禁飞区经度超范围: {chunk}")
        x1, y1 = geo_to_enu(la1, lo1, lat0, lon0)
        x2, y2 = geo_to_enu(la2, lo2, lat0, lon0)
        rects.append(Rect(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))
    return rects


# ========================= 均匀栅格地图(8 邻域) =========================
class Grid:
    def __init__(self, bounds, res, rects, start, goal):
        self.x0, self.y0, self.x1, self.y1 = bounds
        self.res = res
        self.nx = int(math.floor((self.x1 - self.x0) / res)) + 1
        self.ny = int(math.floor((self.y1 - self.y0) / res)) + 1
        self.blocked: set[tuple[int, int]] = set()
        if rects:
            # 膨胀 0.75 格：保证相邻格中心连线也不会擦入禁飞区
            margin = res * 0.75
            for i in range(self.nx):
                x = self.x0 + i * res
                for j in range(self.ny):
                    y = self.y0 + j * res
                    if any(r.contains((x, y), margin) for r in rects):
                        self.blocked.add((i, j))
        for p in (start, goal):  # 起终点格强制可通行
            self.blocked.discard(self.cell(p))

    def cell(self, p):
        i = int(round((p[0] - self.x0) / self.res))
        j = int(round((p[1] - self.y0) / self.res))
        return (max(0, min(self.nx - 1, i)),
                max(0, min(self.ny - 1, j)))

    def center(self, c):
        return self.x0 + c[0] * self.res, self.y0 + c[1] * self.res

    def neighbors(self, c):
        i, j = c
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < self.nx and 0 <= nj < self.ny:
                step = self.res if di == 0 or dj == 0 else self.res * math.sqrt(2)
                yield (ni, nj), step


# ========================= 评估指标 =========================
@dataclass
class Metrics:
    length_m: float       # 路径总长(大圆累计)
    direct_m: float       # 起终点直线距离(大圆)
    straightness: float   # 直线度 = 直线距离 / 路径长度 (0,1]
    max_xtrack_m: float   # 最大横向偏差
    rms_xtrack_m: float   # 横向偏差 RMS
    n_wp: int             # 航点数
    total_turn_deg: float  # 转角总和
    max_turn_deg: float   # 最大单点转角


def compute_metrics(enu, geo, start_geo, end_geo):
    length = 0.0
    for i in range(len(geo) - 1):
        length += haversine(geo[i][0], geo[i][1], geo[i + 1][0], geo[i + 1][1])
    direct = haversine(start_geo[0], start_geo[1], end_geo[0], end_geo[1])
    straightness = direct / length if length > 1e-9 else 1.0
    xtracks = [point_seg_dist(p, enu[0], enu[-1]) for p in enu]
    rms = math.sqrt(sum(d * d for d in xtracks) / len(xtracks))
    total_turn, max_turn = 0.0, 0.0
    for i in range(1, len(enu) - 1):
        h1 = heading_enu(enu[i - 1], enu[i])
        h2 = heading_enu(enu[i], enu[i + 1])
        turn = abs((h2 - h1 + 540.0) % 360.0 - 180.0)
        total_turn += turn
        max_turn = max(max_turn, turn)
    return Metrics(
        length_m=length,
        direct_m=direct,
        straightness=min(1.0, straightness),
        max_xtrack_m=max(xtracks),
        rms_xtrack_m=rms,
        n_wp=len(enu),
        total_turn_deg=total_turn,
        max_turn_deg=max_turn,
    )


@dataclass
class PlanResult:
    key: str
    name: str
    points_enu: list = field(default_factory=list)
    points_geo: list = field(default_factory=list)
    metrics: Metrics | None = None
    stats: dict = field(default_factory=dict)
    error: str | None = None
    elapsed_ms: float = 0.0


def timed_exec(fn):
    """执行一个规划函数 fn() -> (pts, stats)，统一计时并捕获规划异常。"""
    t0 = time.perf_counter()
    try:
        pts, stats = fn()
        return pts, stats, None, (time.perf_counter() - t0) * 1000
    except PlanningError as exc:
        return [], {}, str(exc), (time.perf_counter() - t0) * 1000
    except Exception as exc:  # noqa: BLE001 - 仿真脚本，单算法失败不影响整体
        return [], {}, f"内部错误: {exc}", (time.perf_counter() - t0) * 1000


# ========================= 算法 1：直线插值(基准) =========================
def plan_linear(start, goal, n_seg):
    pts = []
    for i in range(n_seg + 1):
        t = i / n_seg
        pts.append((start[0] + (goal[0] - start[0]) * t,
                    start[1] + (goal[1] - start[1]) * t))
    pts[0], pts[-1] = start, goal
    return pts, {"segments": n_seg}


# ========================= 算法 2/3：A* 与 Dijkstra(同一栅格) =========================
def grid_search(grid, start, goal, use_heuristic):
    """在 Grid 上搜索路径；返回 (航点列表 | None, 扩展节点数)。"""
    import heapq

    sc, gc = grid.cell(start), grid.cell(goal)
    res = grid.res

    def h(c):
        if not use_heuristic:
            return 0.0
        dx, dy = abs(c[0] - gc[0]), abs(c[1] - gc[1])
        # octile 距离：正交步*res + 对角步*(sqrt2-1)*res
        return res * ((dx + dy) + (math.sqrt(2) - 2) * min(dx, dy))

    counter = 0
    openq = [(h(sc), 0.0, counter, sc)]
    g = {sc: 0.0}
    came: dict = {}
    closed: set = set()
    expanded = 0
    while openq:
        _, _, _, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == gc:
            break
        for nb, step_cost in grid.neighbors(cur):
            if nb in closed or nb in grid.blocked:
                continue
            ng = g[cur] + step_cost
            if ng < g.get(nb, math.inf) - 1e-12:
                g[nb] = ng
                came[nb] = cur
                counter += 1
                heapq.heappush(openq, (ng + h(nb), ng, counter, nb))
    if gc != sc and gc not in came:
        return None, expanded
    path = [gc]
    while path[-1] != sc:
        if path[-1] not in came:
            return None, expanded
        path.append(came[path[-1]])
    path.reverse()
    pts = [grid.center(c) for c in path]
    pts[0], pts[-1] = start, goal
    return simplify(pts, 1.0), expanded


def plan_astar(grid, start, goal):
    pts, expanded = grid_search(grid, start, goal, True)
    if pts is None:
        raise PlanningError("A* 未找到路径（起终点可能被禁飞区隔断）")
    return pts, {"expanded_nodes": expanded}


def plan_dijkstra(grid, start, goal):
    pts, expanded = grid_search(grid, start, goal, False)
    if pts is None:
        raise PlanningError("Dijkstra 未找到路径（起终点可能被禁飞区隔断）")
    return pts, {"expanded_nodes": expanded}


# ========================= 算法 4/5：RRT 与 RRT+捷径平滑 =========================
def shortcut(path, rects, rng, iters=300):
    """路径捷径平滑：随机取两点，若连线无阻挡则删掉中间航点。"""
    pts = list(path)
    for _ in range(iters):
        if len(pts) <= 2:
            break
        a = rng.randrange(len(pts) - 1)
        b = rng.randrange(a + 1, len(pts))
        if b - a < 2:
            continue
        if not path_blocked(pts[a], pts[b], rects):
            pts[a + 1:b] = []
    return simplify(pts, 1.0)


def plan_rrt(start, goal, rects, bounds, step, rng,
             max_iter=4000, goal_bias=0.15, do_smooth=False):
    nodes = [start]
    parent = [-1]
    minx, miny, maxx, maxy = bounds
    reached = False
    it = 0
    for it in range(1, max_iter + 1):
        target = goal if rng.random() < goal_bias else (
            rng.uniform(minx, maxx), rng.uniform(miny, maxy))
        ni = min(range(len(nodes)), key=lambda k: dist(nodes[k], target))
        near = nodes[ni]
        d = dist(near, target)
        if d < 1e-9:
            continue
        if d <= step:
            newp = target
        else:
            newp = (near[0] + (target[0] - near[0]) * step / d,
                    near[1] + (target[1] - near[1]) * step / d)
        if path_blocked(near, newp, rects) or any(r.contains(newp) for r in rects):
            continue
        nodes.append(newp)
        parent.append(ni)
        if dist(newp, goal) <= step and not path_blocked(newp, goal, rects):
            nodes.append(goal)
            parent.append(len(nodes) - 2)
            reached = True
            break
    if not reached:  # 退化处理：从最接近终点的树节点直连终点
        ni = min(range(len(nodes)), key=lambda k: dist(nodes[k], goal))
        if (dist(nodes[ni], goal) > 1e-9
                and not path_blocked(nodes[ni], goal, rects)
                and not any(r.contains(goal) for r in rects)):
            nodes.append(goal)
            parent.append(ni)
            reached = True
    if not reached:
        raise PlanningError(f"RRT 在 {max_iter} 次采样内未连通终点")
    path = []
    idx = len(nodes) - 1
    while idx != -1:
        path.append(nodes[idx])
        idx = parent[idx]
    path.reverse()
    if do_smooth:
        path = shortcut(path, rects, rng)
    return simplify(path, 0.3), {"iterations": it, "tree_nodes": len(nodes)}


# ========================= 算法 6：人工势场法 APF =========================
def plan_apf(start, goal, rects, spacing,
             k_att=1.0, k_rep=8.0e5, d0=600.0, max_iter=60000, tol=2.0):
    step = max(2.0, spacing / 10)
    pos = start
    rec = [pos]
    approx = False
    reached = False
    for _ in range(max_iter):
        d_goal = dist(pos, goal)
        if d_goal <= tol:
            reached = True
            break
        # 引力：单位向量指向终点
        fx = (goal[0] - pos[0]) / d_goal * k_att
        fy = (goal[1] - pos[1]) / d_goal * k_att
        # 斥力：禁飞区最近点方向，随距离急剧增大，作用半径 d0
        for r in rects:
            cx = min(max(pos[0], r.minx), r.maxx)
            cy = min(max(pos[1], r.miny), r.maxy)
            d = dist(pos, (cx, cy))
            if 1e-6 < d < d0:
                mag = k_rep * (1.0 / d - 1.0 / d0) / (d * d)
                fx += (pos[0] - cx) / d * mag
                fy += (pos[1] - cy) / d * mag
        nf = math.hypot(fx, fy)
        if nf < 1e-12:
            break
        fx, fy = fx / nf, fy / nf
        newp = None
        s = step
        for _try in range(6):  # 碰撞则步长折半重试
            cand = (pos[0] + fx * s, pos[1] + fy * s)
            if not path_blocked(pos, cand, rects) and not any(r.contains(cand) for r in rects):
                newp = cand
                break
            s *= 0.5
        if newp is None:
            break  # 卡在障碍边缘
        # 航点记录：间距达到 spacing，或方向变化 > 5 度
        if len(rec) >= 2:
            v1 = (rec[-1][0] - rec[-2][0], rec[-1][1] - rec[-2][1])
            v2 = (newp[0] - rec[-1][0], newp[1] - rec[-1][1])
            turn = angle_between(v1, v2)
        else:
            turn = 180.0
        if dist(rec[-1], newp) >= spacing or turn > 5.0:
            rec.append(newp)
        pos = newp
    if not reached:
        if path_blocked(pos, goal, rects) or any(r.contains(goal) for r in rects):
            raise PlanningError("人工势场法未到达终点（可能陷入局部极小）")
        approx = True  # 末段无法由势场走完，直连补全
    if dist(rec[-1], goal) > 0.5:
        rec.append(goal)
    return rec, {"approx_tail": approx}


# ========================= 算法 7：遗传算法 GA =========================
def plan_ga(start, goal, bounds, rng, rects=(),
            n_genes=8, pop=100, gens=250, elite=10, mut_rate=0.3, mut_sigma=0.05):
    minx, miny, maxx, maxy = bounds
    sx = (maxx - minx) or 1.0
    sy = (maxy - miny) or 1.0

    def fitness(ind):
        pts = [start] + ind + [goal]
        L = sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        if rects:
            bad = sum(1 for i in range(len(pts) - 1)
                      if path_blocked(pts[i], pts[i + 1], rects))
            if bad:  # 不可行解强惩罚：先淘汰穿越者，再比长度
                return L + bad * 1e7
        return L

    population = [[(rng.uniform(minx, maxx), rng.uniform(miny, maxy))
                   for _ in range(n_genes)] for _ in range(pop)]
    population.sort(key=fitness)
    best = list(population[0])
    for _ in range(gens):
        population.sort(key=fitness)
        if fitness(population[0]) < fitness(best):
            best = list(population[0])
        nxt = population[:elite]
        while len(nxt) < pop:
            a = min(rng.sample(population, 3), key=fitness)
            b = min(rng.sample(population, 3), key=fitness)
            child = [ga_ if rng.random() < 0.5 else gb for ga_, gb in zip(a, b)]
            for gi in range(len(child)):
                if rng.random() < mut_rate:
                    child[gi] = (child[gi][0] + rng.gauss(0, mut_sigma * sx),
                                 child[gi][1] + rng.gauss(0, mut_sigma * sy))
                child[gi] = (min(max(child[gi][0], minx), maxx),
                             min(max(child[gi][1], miny), maxy))
            nxt.append(child)
        population = nxt
    pts = simplify([start] + best + [goal], 0.5)
    return pts, {"generations": gens, "population": pop,
                 "best_length_m": round(fitness(best), 2)}


# ===== 算法 8：Catmull-Rom 样条(向心参数化) 与 A* 捷径+样条平滑 =====
def plan_spline(ctrl, sample_step, alpha=0.5):
    """以 ctrl 为控制点做向心参数化 Catmull-Rom 样条（非均匀节点）。

    均匀参数化在控制点间距悬殊（如 A* 长直段接短台阶）时会严重过冲，
    弦长^alpha 的向心参数化可抑制过冲与尖角畸变。
    """
    pts = list(ctrl)
    while len(pts) < 4:  # 控制点不足则对分加密
        dens = [pts[0]]
        for i in range(len(pts) - 1):
            dens.append(((pts[i][0] + pts[i + 1][0]) / 2,
                         (pts[i][1] + pts[i + 1][1]) / 2))
            dens.append(pts[i + 1])
        pts = dens
    # 全局节点：t[i+1] = t[i] + 弦长^alpha
    t = [0.0]
    for i in range(1, len(pts)):
        t.append(t[i - 1] + max(1e-9, dist(pts[i - 1], pts[i])) ** alpha)
    # 首尾镜像虚拟控制点（与端点重合，切线退化为半个弦方向）
    t_ext0 = t[0] - max(1e-9, dist(pts[0], pts[1])) ** alpha
    t_ext1 = t[-1] + max(1e-9, dist(pts[-2], pts[-1])) ** alpha
    P = [pts[0]] + pts + [pts[-1]]
    T = [t_ext0] + t + [t_ext1]

    def tangent(i):
        """节点 T[i] 处切线：过前后三点的抛物线在中点的精确导数。"""
        ta, tb, tc = T[i - 1], T[i], T[i + 1]
        d10 = ((P[i][0] - P[i - 1][0]) / (tb - ta),
               (P[i][1] - P[i - 1][1]) / (tb - ta))
        d21 = ((P[i + 1][0] - P[i][0]) / (tc - tb),
               (P[i + 1][1] - P[i][1]) / (tc - tb))
        a = (tb - ta) / (tc - ta)
        b = (tc - tb) / (tc - ta)
        return (a * d21[0] + b * d10[0], a * d21[1] + b * d10[1])

    out = [pts[0]]
    for i in range(len(P) - 3):
        p1, p2 = P[i + 1], P[i + 2]
        m1, m2 = tangent(i + 1), tangent(i + 2)
        seg = T[i + 2] - T[i + 1]  # 段长(全局 t)
        n = max(2, int(math.ceil(dist(p1, p2) / sample_step)))
        for k in range(1, n + 1):
            s = k / n
            s2, s3 = s * s, s * s * s
            h00 = 2 * s3 - 3 * s2 + 1
            h10 = s3 - 2 * s2 + s
            h01 = -2 * s3 + 3 * s2
            h11 = s3 - s2
            out.append((h00 * p1[0] + h10 * seg * m1[0]
                        + h01 * p2[0] + h11 * seg * m2[0],
                        h00 * p1[1] + h10 * seg * m1[1]
                        + h01 * p2[1] + h11 * seg * m2[1]))
    return simplify(out, 1.0)


def plan_smooth(ctrl_pts, rects, sample_step, seed):
    """算法 8：A* 路径 -> 视线捷径拉直 -> 向心样条圆滑转角。

    圆滑结果若与禁飞区相交则回退为捷径路径（安全优先）。
    """
    ctrl = shortcut(ctrl_pts, rects, random.Random(seed), iters=300)
    smooth = plan_spline(ctrl, sample_step)
    for i in range(len(smooth) - 1):
        if path_blocked(smooth[i], smooth[i + 1], rects):
            return ctrl, {"control_points": len(ctrl), "collision_fallback": True}
    return smooth, {"control_points": len(ctrl)}


# ===== 算法 9/10：杜宾斯(Dubins) 最短航迹 与 贝塞尔(Bezier) 平滑航迹 =====
def _norm_ang(a):
    """角度归一化到 [0, 2π)。"""
    while a < 0.0:
        a += 2 * math.pi
    while a >= 2 * math.pi:
        a -= 2 * math.pi
    return a


def _arc_sweep(phi_from, phi_to, ccw):
    """圆弧扫角：逆时针=归一化(to-from)，顺时针=归一化(from-to)。"""
    return _norm_ang(phi_to - phi_from) if ccw else _norm_ang(phi_from - phi_to)


def _dubins_candidates(gx, gy, phi, r):
    """枚举 Dubins 六类候选（起点位姿固定为原点、航向 0，目标位姿 (gx,gy,phi)）。

    返回 [(总长, 类型名, sample(step)->航点列表)]；CSC 4 类 + CCC 2 类。
    """
    out = []

    # ---------- CSC：LSL / RSR / LSR / RSL ----------
    def solve_csc(s1, s3):
        # 首/末圆圆心（单位法向：L=左法向, R=右法向）
        c1 = (0.0, r if s1 == "L" else -r)
        if s3 == "L":
            n3 = (-math.sin(phi), math.cos(phi))
        else:
            n3 = (math.sin(phi), -math.cos(phi))
        c2 = (gx + r * n3[0], gy + r * n3[1])
        dx, dy = c2[0] - c1[0], c2[1] - c1[1]
        d = math.hypot(dx, dy)
        psi = math.atan2(dy, dx)
        phi_s = math.atan2(-c1[1], -c1[0])       # 起点在首圆上的极角
        phi_g = math.atan2(gy - c2[1], gx - c2[0])  # 目标在末圆上的极角
        if s1 == s3:                             # 同侧外切线
            if d < 1e-9:                         # 起终点恰在同一圆上 -> 单弧
                t1 = _arc_sweep(phi_s, phi_g, s1 == "L")
                def sample_one(step, _c=c1, _p=phi_s, _s=t1, _ccw=(s1 == "L")):
                    pts = []
                    n = max(1, int(math.ceil(max(_s, 1e-9) * r / step)))
                    dn = 1.0 if _ccw else -1.0
                    for k in range(1, n + 1):
                        a = _p + dn * _s * k / n
                        pts.append((_c[0] + r * math.cos(a),
                                    _c[1] + r * math.sin(a)))
                    return pts
                return t1 * r, sample_one
            tau, p = psi, d                      # 直线方向平行于圆心连线
        else:                                    # 异侧内切线，需 |Δ| >= 2r
            if d < 2 * r - 1e-9:
                return None
            delta = math.asin(min(1.0, 2 * r / d))
            # LSR: Δ·n̂(u)=-2r -> τ=ψ+δ   RSL: Δ·n̂(u)=+2r -> τ=ψ-δ
            tau = psi + delta if s1 == "L" else psi - delta
            p = dx * math.cos(tau) + dy * math.sin(tau)
            if p < -1e-9:
                return None
            p = max(0.0, p)
        # 首弧扫角（切点极角：L->(sinτ,-cosτ)  R->(-sinτ,cosτ)）
        st, ct = math.sin(tau), math.cos(tau)
        ax, ay = (st, -ct) if s1 == "L" else (-st, ct)
        phi_a = math.atan2(ay, ax)
        t1 = _arc_sweep(phi_s, phi_a, s1 == "L")
        # 末弧扫角（切点侧由 s3 决定：跨侧切线时与 s1 相反）
        bx, by = (st, -ct) if s3 == "L" else (-st, ct)
        phi_b = math.atan2(by, bx)
        t2 = _arc_sweep(phi_b, phi_g, s3 == "L")

        def sample_csc(step, _s1=s1, _s3=s3, _t1=t1, _p=p, _t2=t2,
                       _c1=c1, _phi_s=phi_s):
            pts = []
            # 首弧
            ccw1 = _s1 == "L"
            n = max(1, int(math.ceil(_t1 * r / step)))
            dn = 1.0 if ccw1 else -1.0
            for k in range(1, n + 1):
                a = _phi_s + dn * _t1 * k / n
                pts.append((_c1[0] + r * math.cos(a), _c1[1] + r * math.sin(a)))
            # 直线段（当前航向 = τ）
            px, py = pts[-1]
            h = _t1 if ccw1 else -_t1
            if _p > 1e-9:
                m = max(1, int(math.ceil(_p / step)))
                for k in range(1, m + 1):
                    dd = _p * k / m
                    pts.append((px + math.cos(h) * dd, py + math.sin(h) * dd))
            # 末弧：圆心由直末点+航向反推
            b = pts[-1]
            ccw3 = _s3 == "L"
            if ccw3:
                cx, cy = b[0] - r * math.sin(h), b[1] + r * math.cos(h)
                pb = h - math.pi / 2
            else:
                cx, cy = b[0] + r * math.sin(h), b[1] - r * math.cos(h)
                pb = h + math.pi / 2
            n2 = max(1, int(math.ceil(_t2 * r / step)))
            dn2 = 1.0 if ccw3 else -1.0
            for k in range(1, n2 + 1):
                a = pb + dn2 * _t2 * k / n2
                pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
            return pts

        return r * t1 + p + r * t2, sample_csc

    for s1 in ("L", "R"):
        for s3 in ("L", "R"):
            got = solve_csc(s1, s3)
            if got is not None:
                out.append((got[0], s1 + "S" + s3, got[1]))

    # ---------- CCC：RLR / LRL（中间圆与首尾圆外切，圆心距 = 2r） ----------
    def solve_ccc(s1):
        results = []
        c1 = (0.0, r if s1 == "L" else -r)
        if s1 == "L":
            n3 = (-math.sin(phi), math.cos(phi))
        else:
            n3 = (math.sin(phi), -math.cos(phi))
        c3 = (gx + r * n3[0], gy + r * n3[1])
        d13 = math.hypot(c3[0] - c1[0], c3[1] - c1[1])
        if d13 > 4 * r + 1e-9 or d13 < 1e-9:
            return results
        psi = math.atan2(c3[1] - c1[1], c3[0] - c1[0])
        mid = ((c1[0] + c3[0]) / 2.0, (c1[1] + c3[1]) / 2.0)
        h = math.sqrt(max(0.0, 4.0 * r * r - (d13 / 2.0) ** 2))
        nx, ny = -math.sin(psi), math.cos(psi)
        smid = "R" if s1 == "L" else "L"
        for sign in ((1.0, -1.0) if h > 1e-9 else (1.0,)):
            c2 = (mid[0] + sign * h * nx, mid[1] + sign * h * ny)
            d21 = math.hypot(c2[0] - c1[0], c2[1] - c1[1]) or 1e-12
            d23 = math.hypot(c3[0] - c2[0], c3[1] - c2[1]) or 1e-12
            t1p = (c1[0] + r * (c2[0] - c1[0]) / d21,
                   c1[1] + r * (c2[1] - c1[1]) / d21)   # 圆1-圆2 切点
            t2p = (c2[0] + r * (c3[0] - c2[0]) / d23,
                   c2[1] + r * (c3[1] - c2[1]) / d23)   # 圆2-圆3 切点
            phi_s = math.atan2(-c1[1], -c1[0])
            phi_t1c1 = math.atan2(t1p[1] - c1[1], t1p[0] - c1[0])
            phi_t1c2 = math.atan2(t1p[1] - c2[1], t1p[0] - c2[0])
            phi_t2c2 = math.atan2(t2p[1] - c2[1], t2p[0] - c2[0])
            phi_t2c3 = math.atan2(t2p[1] - c3[1], t2p[0] - c3[0])
            phi_g = math.atan2(gy - c3[1], gx - c3[0])
            t1 = _arc_sweep(phi_s, phi_t1c1, s1 == "L")
            t2 = _arc_sweep(phi_t1c2, phi_t2c2, smid == "L")
            t3 = _arc_sweep(phi_t2c3, phi_g, s1 == "L")

            def sample_ccc(step, _c1=c1, _c2=c2, _c3=c3, _s1=s1, _sm=smid,
                           _t1=t1, _t2=t2, _t3=t3, _ps=phi_s,
                           _p1=phi_t1c2, _p2=phi_t2c3):
                pts = []
                for cen, p0, sw, ccw in (
                        (_c1, _ps, _t1, _s1 == "L"),
                        (_c2, _p1, _t2, _sm == "L"),
                        (_c3, _p2, _t3, _s1 == "L")):
                    n = max(1, int(math.ceil(max(sw, 1e-9) * r / step)))
                    dn = 1.0 if ccw else -1.0
                    for k in range(1, n + 1):
                        a = p0 + dn * sw * k / n
                        pts.append((cen[0] + r * math.cos(a),
                                    cen[1] + r * math.sin(a)))
                return pts

            results.append((r * (t1 + t2 + t3), s1 + smid + s1, sample_ccc))
        return results

    for s1 in ("L", "R"):
        out.extend(solve_ccc(s1))
    return out


def plan_dubins(start, goal, start_hdg_deg, end_hdg_deg, turn_radius,
                sample_step, rects=()):
    """算法 9：杜宾斯(Dubins) 最短可行航迹。

    只含左转(L)/直行(S)/右转(R)三种原语，转弯半径下限 = turn_radius；
    枚举 LSL/RSR/LSR/RSL/RLR/LRL 取最短。航向对准目标时解退化为直线。
    start/end_hdg_deg 为绝对航向（度，0=正北，顺时针）。不避障。
    """
    if turn_radius <= 0:
        raise PlanningError("Dubins 最小转弯半径必须 > 0")
    th0 = math.radians(start_hdg_deg % 360.0)
    th1 = math.radians(end_hdg_deg % 360.0)
    # 变换到起点位姿系：x 轴=起飞航向，y 轴=航向左侧（罗盘角->数学角取反）
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    X = dx * math.sin(th0) + dy * math.cos(th0)
    Y = dy * math.sin(th0) - dx * math.cos(th0)
    Phi = _norm_ang(th0 - th1)
    cands = _dubins_candidates(X, Y, Phi, turn_radius)
    if not cands:
        raise PlanningError("Dubins 无可行解（检查转弯半径/航向设置）")
    cands.sort(key=lambda c: c[0])
    best_len, best_type, best_sample = cands[0]
    step = max(4.0, min(sample_step, turn_radius / 8.0))
    frame = best_sample(step)
    pts = [(start[0] + x * math.sin(th0) - y * math.cos(th0),
            start[1] + x * math.cos(th0) + y * math.sin(th0))
           for x, y in frame]
    pts[0], pts[-1] = start, goal
    pts = simplify(pts, 0.2)   # 小阈值：直线段可合并，圆弧不被过度拉直
    for i in range(len(pts) - 1):
        if path_blocked(pts[i], pts[i + 1], rects):
            raise PlanningError("Dubins 航迹穿越禁飞区（该算法不避障）")
    return pts, {"type": best_type, "turn_radius_m": turn_radius,
                 "start_hdg": round(start_hdg_deg % 360.0, 1),
                 "end_hdg": round(end_hdg_deg % 360.0, 1)}


def plan_bezier(start, goal, start_hdg_deg, end_hdg_deg, sample_step,
                rects=()):
    """算法 10：三次贝塞尔(Bezier) 平滑航迹。

    P0=起点, P3=目标, P1/P2 沿起/终点航向各延伸 0.4×直线距离：
    端点与端点切向精确匹配起/终点航向。航向对准目标时控制点共线 -> 直线；
    航向不同时为曲率连续的 S 形。不避障、不保证最短。
    """
    th0 = math.radians(start_hdg_deg % 360.0)
    th1 = math.radians(end_hdg_deg % 360.0)
    d_line = dist(start, goal)
    d = 0.4 * d_line
    # 罗盘航向 -> 方位单位向量 (x 东, y 北)
    p1 = (start[0] + d * math.sin(th0), start[1] + d * math.cos(th0))
    p2 = (goal[0] - d * math.sin(th1), goal[1] - d * math.cos(th1))
    n = max(40, int(math.ceil(d_line / max(10.0, sample_step))))
    pts = []
    for k in range(n + 1):
        t = k / n
        u = 1.0 - t
        b0 = u * u * u
        b1 = 3.0 * u * u * t
        b2 = 3.0 * u * t * t
        b3 = t * t * t
        pts.append((b0 * start[0] + b1 * p1[0] + b2 * p2[0] + b3 * goal[0],
                    b0 * start[1] + b1 * p1[1] + b2 * p2[1] + b3 * goal[1]))
    pts[0], pts[-1] = start, goal
    pts = simplify(pts, 0.5)
    for i in range(len(pts) - 1):
        if path_blocked(pts[i], pts[i + 1], rects):
            raise PlanningError("贝塞尔航迹穿越禁飞区（该算法不避障）")
    return pts, {"ctrl_offset": round(d, 1),
                 "start_hdg": round(start_hdg_deg % 360.0, 1),
                 "end_hdg": round(end_hdg_deg % 360.0, 1)}


# ========================= 算法说明(功能/描述) =========================
ALGO_DOCS = [
    dict(key="linear", name="直线插值(基准)", short="基准直线", color="#94a3b8", dash=True,
         func="在起终点之间等间距插值布点，构成理论最短路径基准。",
         desc="假设空域不受限制，沿两点连线均匀生成航点。不做任何避障与动力学考虑，"
              "仅作对照基准：其它算法路径长度不可能短于它，直线度上限为 1.0000。"),
    dict(key="astar", name="A* 栅格寻路", short="A*", color="#2563eb",
         func="在离散栅格上搜索累积代价最小路径，用启发式估计引导搜索朝向终点。",
         desc="空域划分为均匀栅格（8 邻域：正交代价 1、对角代价 √2），f=g+h，h 采用 "
              "octile 距离（剩余对角步估计）。启发式使搜索集中在终点方向，扩展节点少；"
              "无障碍时路径贴近直线，但受栅格方向限制呈台阶状（量化误差）。"),
    dict(key="dijkstra", name="Dijkstra 栅格寻路", short="Dijkstra", color="#7c3aed",
         func="不带启发式，从起点向外一圈圈均匀扩展，保证得到全局最短路径。",
         desc="与 A* 使用同一张 8 连通栅格，代价只含 g(n)。以起点为中心逐圈展开，"
              "扩展节点数远多于 A*（本次运行数据可直接对比两者效率）；同一张图上找到的"
              "路径代价与 A* 相同，仅因同代价出队顺序不同导致形状略有差异。"),
    dict(key="rrt", name="RRT 快速扩展随机树", short="RRT", color="#dc2626",
         func="随机采样、逐步向采样点长树，快速覆盖自由空间，适合高维与复杂约束空间。",
         desc="从起点维护一棵树：每次随机取点（15% 目标偏置），找最近节点并沿方向延伸"
              "固定步长加入树中，直至连通终点。路径由树枝拼接，呈折线锯齿、转弯多，"
              "直线度通常最低；优点是对空间形状不敏感、无需栅格化、实现简单。"),
    dict(key="rrt_smooth", name="RRT + 捷径平滑", short="RRT+平滑", color="#f59e0b",
         func="在 RRT 原始路径上反复用直线段替换中间航点，把折线拉直并缩短。",
         desc="随机选取路径上两点，若连线不穿越禁飞区则删除中间点，迭代多轮后再做共线"
              "点简化。这是工程上最常用的路径后处理手段，可显著减少转弯、提高直线度，"
              "代价是需要额外的碰撞检测。"),
    dict(key="apf", name="人工势场法(APF)", short="APF", color="#059669",
         func="把终点视为引力源、禁飞区视为斥力源，沿合力方向连续运动直至终点。",
         desc="合力 F = 引力 + 斥力（斥力在作用半径 d0 内随距离减小急剧增大）。无障碍时"
              "合力始终指向终点，轨迹即为直线；有障碍时在斥力作用下平滑绕行。计算量小、"
              "路径天然平滑，但目标被障碍物遮挡时可能陷入局部极小。"),
    dict(key="ga", name="遗传算法(GA)", short="GA", color="#0891b2",
         func="把中间航点序列编码为染色体，通过选择/交叉/变异迭代进化，最小化路径总长。",
         desc="种群 100、进化 250 代、精英保留 10，变异幅度 5%，适应度 = 路径总长；"
              "穿越禁飞区的个体施加强惩罚（先可行、后最短）。无禁飞区时搜索范围限定在"
              "起终点走廊（包围盒外扩 15%）加速收敛，有禁飞区时放宽到与栅格一致的全范围。"
              "初始随机航点被逐步拉向起终点连线，收敛到接近直线的解。全局随机优化、"
              "不依赖栅格，结果受随机种子影响（本脚本固定种子，可复现）。"),
    dict(key="smooth", name="A* + 路径平滑(捷径+样条)", short="A*平滑", color="#db2777",
         func="对 A* 路径先做视线捷径拉直，再用向心 Catmull-Rom 样条圆滑转角。",
         desc="捷径法反复尝试用直线段替换中间航点（要求连线不穿越禁飞区），把栅格折线"
              "拉直；再以剩余航点为控制点做向心参数化样条，按固定弧长采样，得到曲率"
              "连续、可直接跟踪的航路。若圆滑后与禁飞区相交则自动回退为捷径路径"
              "（安全优先）。这是“栅格规划 + 后处理平滑”的典型工程组合。"),
    dict(key="dubins", name="杜宾斯(Dubins)最短航迹", short="Dubins", color="#ea580c",
         func="在最小转弯半径约束下，求 地面端→目标(含起飞/抵达航向)的最短可行航迹。",
         desc="只允许左转(L)、直行(S)、右转(R)三种原语，枚举 LSL/RSR/LSR/RSL/RLR/LRL "
              "六类候选取最短。起飞/抵达航向对准目标方位时解退化为直线；要求以指定姿态"
              "接近目标（如拦截机以特定方位进抵）时，输出“圆弧+直线+圆弧”的最短可行"
              "航迹，圆弧半径不低于 --turn-radius。运动学约束下最短路径的经典解，"
              "确定性、可复现；该算法不避障。"),
    dict(key="bezier", name="贝塞尔(Bezier)平滑航迹", short="贝塞尔", color="#0d9488",
         func="用三次贝塞尔曲线生成端点切向匹配起/终点航向的曲率连续平滑航迹。",
         desc="控制点 P1/P2 分别沿起飞、抵达航向延伸（0.4×直线距离），曲线端点与端点"
              "切向精确匹配航向。航向对准目标时四点共线、轨迹即直线；航向不同时为平滑 "
              "S 形曲线。计算极快、形状由少数参数控制，常用于航迹生成与轨迹平滑；"
              "但不保证最短，也没有转弯半径下限。该算法不避障。"),
]

METRIC_DOCS = [
    ("直线度", "起终点直线距离 ÷ 路径总长，取值 (0,1]，越接近 1 越接近直线（1.0000=理想直线）。"),
    ("路径长度", "沿规划航点的大圆距离累计（米）。"),
    ("最大横向偏差", "任一航点到起终点连线的最大垂距（米），反映偏离直线的程度。"),
    ("RMS 横向偏差", "全部航点横向偏差的均方根（米）。"),
    ("航点数", "输出航点数量（含起终点）。"),
    ("转角总和 / 最大转角", "相邻航段航向变化绝对值之和与单点最大值（度），越小越平直。"),
    ("扩展节点数", "栅格搜索类算法(A*/Dijkstra)实际弹出的节点数，衡量搜索开销。"),
]


def waypoint_rows(geo_pts):
    """航点 GPS 数据行：序号、纬度、经度、距上一点(米)、上一点到本点航向(度)。"""
    rows = []
    for i, (lat, lon) in enumerate(geo_pts):
        if i == 0:
            rows.append(dict(seq=1, lat=lat, lon=lon, dist=0.0, bearing=None))
        else:
            plat, plon = geo_pts[i - 1]
            rows.append(dict(seq=i + 1, lat=lat, lon=lon,
                             dist=haversine(plat, plon, lat, lon),
                             bearing=bearing_deg(plat, plon, lat, lon)))
    return rows


# ========================= SVG 绘图 =========================
def _tw(s, size=12.0):
    """估算文本像素宽度（中文按 size，西文按 0.6*size）。"""
    return sum(size if ord(c) > 127 else size * 0.6 for c in s)


def nice_step(span):
    if span <= 0:
        return 1.0
    raw = span / 6.0
    base = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if raw <= m * base:
            return m * base
    return 10 * base


def svg_chart(series, start, goal, rects, title="", width=880, height=540, legend=True):
    """series: [{name, color, pts[(x,y)...], dash?}] -> 内联 SVG 字符串（北向朝上）。"""
    pts_all = [p for s in series for p in s["pts"]]
    if not pts_all:
        return "<p class='muted'>(无路径可绘制)</p>"
    for r in rects:
        pts_all += [(r.minx, r.miny), (r.maxx, r.maxy)]
    xs = [p[0] for p in pts_all]
    ys = [p[1] for p in pts_all]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    if maxx - minx < 1:
        minx -= 1.0
        maxx += 1.0
    if maxy - miny < 1:
        miny -= 1.0
        maxy += 1.0
    pad = 46.0
    s = min((width - 2 * pad) / (maxx - minx), (height - 2 * pad) / (maxy - miny))
    ox = pad + ((width - 2 * pad) - (maxx - minx) * s) / 2
    oy = pad + ((height - 2 * pad) - (maxy - miny) * s) / 2

    def X(x):
        return ox + (x - minx) * s

    def Y(y):
        return height - (oy + (y - miny) * s)

    out = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" class="chart">',
           f'<rect width="{width}" height="{height}" fill="#ffffff" stroke="#e2e8f0"/>']
    step = nice_step(maxx - minx)
    gx = math.ceil(minx / step) * step
    while gx <= maxx:
        out.append(f'<line x1="{X(gx):.1f}" y1="{Y(miny):.1f}" x2="{X(gx):.1f}" '
                   f'y2="{Y(maxy):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{X(gx):.1f}" y="{height - 12}" font-size="10" fill="#94a3b8" '
                   f'text-anchor="middle">{gx / 1000:.1f}km</text>')
        gx += step
    step_y = nice_step(maxy - miny)
    gy = math.ceil(miny / step_y) * step_y
    while gy <= maxy:
        out.append(f'<line x1="{X(minx):.1f}" y1="{Y(gy):.1f}" x2="{X(maxx):.1f}" '
                   f'y2="{Y(gy):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="6" y="{Y(gy) + 3:.1f}" font-size="10" '
                   f'fill="#94a3b8">{gy / 1000:.1f}km</text>')
        gy += step_y
    for r in rects:  # 禁飞区
        out.append(f'<rect x="{X(r.minx):.1f}" y="{Y(r.maxy):.1f}" '
                   f'width="{X(r.maxx) - X(r.minx):.1f}" height="{Y(r.miny) - Y(r.maxy):.1f}" '
                   f'fill="#94a3b8" fill-opacity="0.35" stroke="#64748b" stroke-dasharray="4 3"/>')
    for s_ in series:
        if not s_["pts"]:
            continue
        poly = " ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in s_["pts"])
        dash = ' stroke-dasharray="7 5"' if s_.get("dash") else ""
        out.append(f'<polyline points="{poly}" fill="none" stroke="{s_["color"]}" '
                   f'stroke-width="2.4"{dash} stroke-linejoin="round" stroke-linecap="round"/>')
        if 0 < len(s_["pts"]) <= 60:
            for p in s_["pts"]:
                out.append(f'<circle cx="{X(p[0]):.1f}" cy="{Y(p[1]):.1f}" r="2.2" '
                           f'fill="{s_["color"]}"/>')
    out.append(f'<circle cx="{X(start[0]):.1f}" cy="{Y(start[1]):.1f}" r="6" '
               f'fill="#16a34a" stroke="#fff" stroke-width="2"/>')
    out.append(f'<text x="{X(start[0]) + 9:.1f}" y="{Y(start[1]) + 4:.1f}" font-size="12" '
               f'font-weight="bold" fill="#15803d">地面端</text>')
    out.append(f'<circle cx="{X(goal[0]):.1f}" cy="{Y(goal[1]):.1f}" r="6" '
               f'fill="#dc2626" stroke="#fff" stroke-width="2"/>')
    out.append(f'<text x="{X(goal[0]) + 9:.1f}" y="{Y(goal[1]) + 4:.1f}" font-size="12" '
               f'font-weight="bold" fill="#b91c1c">空中目标</text>')
    if legend and len(series) > 1:
        bw = max(_tw(s_["name"]) for s in series) + 46
        bh = 10 + 18 * len(series)
        out.insert(2, f'<rect x="10" y="10" width="{bw:.0f}" height="{bh:.0f}" '
                      f'fill="#ffffff" fill-opacity="0.88" stroke="#e2e8f0" rx="4"/>')
        for i, s_ in enumerate(series):
            yy = 24 + i * 18
            dash = ' stroke-dasharray="6 4"' if s_.get("dash") else ""
            out.append(f'<line x1="18" y1="{yy}" x2="44" y2="{yy}" stroke="{s_["color"]}" '
                       f'stroke-width="3"{dash}/>')
            out.append(f'<text x="50" y="{yy + 4}" font-size="12" '
                       f'fill="#334155">{html.escape(s_["name"])}</text>')
    if title:
        out.append(f'<text x="{width - 10}" y="20" text-anchor="end" font-size="12" '
                   f'fill="#64748b">{html.escape(title)}</text>')
    out.append("</svg>")
    return "".join(out)


def svg_bars(items, width=880):
    """直线度偏离条形图：items=[(name, straightness, color)]，条越短越直。"""
    row_h = 32
    height = 40 + row_h * len(items) + 8
    label_w = 170.0
    vals = [1.0 - v for _, v, _ in items]
    maxv = max(max(vals), 1e-9)
    out = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" class="chart">',
           f'<rect width="{width}" height="{height}" fill="#ffffff" stroke="#e2e8f0"/>',
           '<text x="10" y="22" font-size="13" fill="#334155">'
           '偏离直线程度 (1 − 直线度)：条越短越接近直线</text>']
    for i, (name, sv, color) in enumerate(items):
        y = 40 + i * row_h
        v = 1.0 - sv
        bw = max(1.5, v / maxv * (width - label_w - 130))
        out.append(f'<text x="8" y="{y + 17}" font-size="12" fill="#334155">'
                   f'{html.escape(name)}</text>')
        out.append(f'<rect x="{label_w}" y="{y + 6}" width="{bw:.1f}" height="{row_h - 14}" '
                   f'fill="{color}" rx="3"/>')
        out.append(f'<text x="{label_w + bw + 6:.1f}" y="{y + 17}" font-size="11" '
                   f'fill="#64748b">{v * 100:.4f}%</text>')
    out.append("</svg>")
    return "".join(out)


# ========================= HTML 仿真报告 =========================
_CSS = """
*{box-sizing:border-box}
body{font-family:"Microsoft YaHei","PingFang SC",sans-serif;margin:0;
 background:#f1f5f9;color:#0f172a}
.wrap{max-width:1120px;margin:0 auto;padding:24px 16px 64px}
h1{font-size:26px;margin:6px 0 4px}
h2{font-size:20px;margin:34px 0 12px;border-left:5px solid #2563eb;padding-left:10px}
h3{font-size:16px;margin:16px 0 6px}
.muted{color:#64748b}
.meta{background:#fff;border:1px solid #e2e8f0;border-radius:8px;padding:12px 16px;
 font-size:14px;line-height:1.9}
.verdict{margin-top:14px;background:#ecfdf5;border:1px solid #6ee7b7;border-radius:8px;
 padding:14px 16px;font-size:15px;line-height:1.8}
table{border-collapse:collapse;width:100%;background:#fff;font-size:13px}
th,td{border:1px solid #e2e8f0;padding:7px 9px;text-align:right}
th{background:#f8fafc;text-align:center;font-weight:600}
td:first-child,th:first-child{text-align:left}
tr:nth-child(even) td{background:#fafcff}
.chart{width:100%;height:auto;display:block;margin:8px 0 4px;
 border-radius:8px;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.card{background:#fff;border:1px solid #e2e8f0;border-radius:10px;padding:6px 18px 16px;
 margin:16px 0}
.tag{display:inline-block;background:#eff6ff;color:#1d4ed8;border-radius:4px;
 padding:1px 8px;font-size:12px;margin-left:8px;vertical-align:middle}
.tag.err{background:#fef2f2;color:#b91c1c}
details{margin-top:8px}
summary{cursor:pointer;color:#1d4ed8;font-size:13px}
table.small{font-size:12px;margin-top:6px}
table.small td,table.small th{padding:4px 8px}
.note{background:#fffbeb;border:1px solid #fde68a;border-radius:8px;padding:10px 14px;
 font-size:13px;line-height:1.8}
ul{line-height:1.9;font-size:14px}
code{background:#f1f5f9;border-radius:3px;padding:1px 5px;font-size:12px}
"""


def _fmt(v, nd=2):
    return "—" if v is None else f"{v:.{nd}f}"


def build_report(ctx):
    """ctx: dict，含输入、指标表、算法结果等 -> HTML 字符串。"""
    esc = html.escape
    results = ctx["results"]
    ranked = [r for r in results if r.metrics]
    ranked.sort(key=lambda r: r.metrics.straightness, reverse=True)

    parts = [
        "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'>",
        "<meta name='viewport' content='width=device-width,initial-scale=1'>",
        f"<title>路径规划仿真报告 - {esc(ctx['stamp'])}</title>",
        f"<style>{_CSS}</style></head><body><div class='wrap'>",
        "<h1>路径规划仿真报告</h1>",
        f"<div class='muted'>生成时间 {esc(ctx['stamp'])} ｜ 随机种子 {ctx['seed']} "
        f"｜ 栅格分辨率 {ctx['grid_res']:.0f} m</div>",
        "<div class='meta'>",
        f"<b>地面端</b> ({ctx['start'][0]:.6f}, {ctx['start'][1]:.6f}) ｜ "
        f"<b>空中目标</b> ({ctx['end'][0]:.6f}, {ctx['end'][1]:.6f}) ｜ "
        f"<b>直线距离</b> {ctx['direct'] / 1000:.3f} km<br>",
        f"<b>场景</b> 地面端 → 空中目标（GPS 二维水平投影；默认不考虑禁飞区，"
        f"高度/爬升剖面不在本仿真范围）<br>",
        f"<b>起飞/抵达航向</b> {ctx['start_hdg']:.1f}° / {ctx['end_hdg']:.1f}°"
        f"{'（缺省=对准目标方位）' if ctx.get('hdg_auto') else '（自定义）'} ｜ "
        f"<b>Dubins 最小转弯半径</b> {ctx['turn_radius']:.0f} m ｜ "
        f"<b>禁飞区</b> {esc(ctx['obstacles_text'])}</div>",
    ]
    if ctx["rects"]:
        parts.append("<div class='note' style='margin-top:10px'>存在禁飞区："
                     "<b>直线插值(基准)</b>与<b>杜宾斯/贝塞尔</b>不进行避障、"
                     "可能穿越禁飞区（若穿越，这两个算法会报告规划失败），仅作几何/"
                     "运动学对照；其余算法均按禁飞区绕行规划。</div>")

    # 结论
    valid = [r for r in results if r.metrics and r.key != "linear"]
    if valid:
        best = max(valid, key=lambda r: r.metrics.straightness)
        worst = min(valid, key=lambda r: r.metrics.straightness)
        bs = best.metrics.straightness
        tied = "、".join(esc(r.name) for r in valid
                         if abs(r.metrics.straightness - bs) <= 1e-4)
        parts.append(
            f"<div class='verdict'><b>最接近直线的规划算法：【{tied}】</b>，"
            f"直线度 <b>{bs:.4f}</b>（偏离 {(1 - bs) * 100:.4f}%）；"
            f"偏离最大的是【{esc(worst.name)}】"
            f"（直线度 {worst.metrics.straightness:.4f}）。"
            f"对照基准（理想直线）直线度 = "
            f"{next((r.metrics.straightness for r in results if r.key == 'linear'), 1.0):.4f}。"
            f"<br>规律：航向对准目标时，杜宾斯/贝塞尔/APF/GA/平滑类输出均≈基准直线；"
            f"一旦指定起飞或抵达航向≠目标方位，杜宾斯以最小转弯半径圆弧过渡"
            f"（最短可行）、贝塞尔输出平滑 S 形，直线度随之下降；A*、Dijkstra 受栅格"
            f"台阶量化影响略降；RRT 随机生长的折线最不直，须经捷径平滑拉直。"
            f"存在禁飞区时各绕行算法直线度整体下降，应综合比较绕行代价与转弯幅度。"
            f"</div>")

    # 指标说明
    parts.append("<h2>1. 评估指标说明</h2><ul>")
    for name, desc in METRIC_DOCS:
        parts.append(f"<li><b>{esc(name)}</b>：{esc(desc)}</li>")
    parts.append("</ul>")

    # 总表
    parts.append("<h2>2. 算法仿真结果对比（按直线度降序）</h2>")
    parts.append("<table><tr><th>算法</th><th>直线度</th><th>路径长度(m)</th>"
                 "<th>直线距离(m)</th><th>最大横向偏差(m)</th><th>RMS偏差(m)</th>"
                 "<th>航点数</th><th>转角总和(°)</th><th>最大转角(°)</th>"
                 "<th>扩展节点</th><th>耗时(ms)</th></tr>")
    for r in ranked:
        m = r.metrics
        ex = r.stats.get("expanded_nodes", "—")
        parts.append(
            f"<tr><td>{esc(r.name)}</td><td><b>{m.straightness:.4f}</b></td>"
            f"<td>{m.length_m:.1f}</td><td>{m.direct_m:.1f}</td>"
            f"<td>{m.max_xtrack_m:.1f}</td><td>{m.rms_xtrack_m:.1f}</td>"
            f"<td>{m.n_wp}</td><td>{m.total_turn_deg:.1f}</td>"
            f"<td>{m.max_turn_deg:.1f}</td><td>{ex}</td>"
            f"<td>{r.elapsed_ms:.0f}</td></tr>")
    for r in results:
        if r.error:
            parts.append(f"<tr><td>{esc(r.name)}</td><td colspan='11' style='text-align:left;"
                         f"color:#b91c1c'>规划失败：{esc(r.error)}</td></tr>")
    parts.append("</table>")

    # 对比图
    parts.append("<h2>3. 路径形态对比</h2>")
    series = [dict(name=d["short"], color=d["color"], dash=d.get("dash", False),
                   pts=next((r.points_enu for r in results if r.key == d["key"]), []))
              for d in ALGO_DOCS]
    parts.append(svg_chart(series, ctx["enu_start"], ctx["enu_goal"], ctx["rects"],
                           title="全部算法叠加"))
    bar_colors = {d["key"]: d["color"] for d in ALGO_DOCS}
    parts.append(svg_bars([(r.name, r.metrics.straightness,
                            bar_colors.get(r.key, "#64748b"))
                           for r in sorted((x for x in results if x.metrics),
                                           key=lambda x: x.metrics.straightness,
                                           reverse=True)]))

    # 各算法详情
    parts.append("<h2>4. 各算法详情与航点 GPS 数据</h2>")
    for d in ALGO_DOCS:
        r = next(x for x in results if x.key == d["key"])
        tag = (f"<span class='tag err'>失败</span>" if r.error else
               f"<span class='tag'>直线度 {_fmt(r.metrics and r.metrics.straightness, 4)}"
               f"</span>")
        parts.append(f"<div class='card'><h3>{esc(d['name'])}{tag}</h3>")
        parts.append(f"<p style='font-size:14px;line-height:1.8;margin:4px 0'>"
                     f"<b>功能：</b>{esc(d['func'])}<br><b>描述：</b>{esc(d['desc'])}</p>")
        if r.error:
            parts.append(f"<p style='color:#b91c1c'>规划失败：{esc(r.error)}</p></div>")
            continue
        parts.append(svg_chart(
            [dict(name=d["name"], color=d["color"], dash=d.get("dash", False),
                  pts=r.points_enu)],
            ctx["enu_start"], ctx["enu_goal"], ctx["rects"], legend=False))
        m = r.metrics
        stats = " ｜ ".join(f"{k}={v}" for k, v in r.stats.items())
        parts.append(f"<div class='muted' style='font-size:13px'>路径长度 "
                     f"{m.length_m:.1f} m ｜ 最大横偏 {m.max_xtrack_m:.1f} m ｜ "
                     f"航点 {m.n_wp} 个 ｜ 转角和 {m.total_turn_deg:.1f}°"
                     + (f" ｜ {esc(stats)}" if stats else "") + "</div>")
        rows = waypoint_rows(r.points_geo)
        parts.append("<details><summary>展开航点 GPS 数据（共 "
                     f"{len(rows)} 个）</summary><table class='small'>"
                     "<tr><th>序号</th><th>纬度</th><th>经度</th>"
                     "<th>距上一点(m)</th><th>航向(°)</th></tr>")
        for row in rows:
            b = "—" if row["bearing"] is None else f"{row['bearing']:.1f}"
            parts.append(f"<tr><td>{row['seq']}</td><td>{row['lat']:.7f}</td>"
                         f"<td>{row['lon']:.7f}</td><td>{row['dist']:.1f}</td>"
                         f"<td>{b}</td></tr>")
        parts.append("</table></details></div>")

    # 算法说明汇总
    parts.append("<h2>5. 算法功能与描述汇总</h2><table>"
                 "<tr><th>算法</th><th>功能</th><th>描述</th></tr>")
    for d in ALGO_DOCS:
        parts.append(f"<tr><td>{esc(d['name'])}</td><td style='text-align:left'>"
                     f"{esc(d['func'])}</td><td style='text-align:left'>"
                     f"{esc(d['desc'])}</td></tr>")
    parts.append("</table>")

    parts.append("<h2>6. 选型建议</h2><div class='note'>"
                 "<b>地面端起飞→空中目标（本场景）</b>：起飞/抵达航向缺省对准目标方位，"
                 "此时杜宾斯与贝塞尔都退化为直线（=基准）；需要以指定方位进抵目标"
                 "（拦截姿态）时选<b>杜宾斯</b>——满足最小转弯半径的最短可行航迹、"
                 "确定性可复现；只需平滑连续曲率、不苛求最短时选<b>贝塞尔</b>。<br>"
                 "<b>只看谁最接近直线</b>：空域畅通时杜宾斯/贝塞尔(航向对准)、"
                 "人工势场(APF)、遗传算法(GA)及平滑类输出几乎与基准直线重合；"
                 "栅格类(A*/Dijkstra)因台阶量化略差，但给出全局最短路径的严格保证；"
                 "RRT 原始路径最不直，必须配合捷径平滑。<br>"
                 "<b>需要绕禁飞区时</b>：A*/Dijkstra 可靠且可复现，"
                 "推荐 A* + 路径平滑(捷径+样条)输出可跟踪航路；"
                 "APF 平滑但可能陷入局部极小；"
                 "GA 需多次运行取优；RRT+平滑在复杂障碍下表现好但结果随机；"
                 "杜宾斯/贝塞尔不避障。<br>"
                 "<b>工程组合建议</b>：A*（全局最优骨架）→ 捷径/样条平滑（拉直与平滑）"
                 "→ 航向/转弯约束检查（可用杜宾斯复核末段进近），"
                 "是无人机航路规划的常见做法。</div>")

    parts.append(f"<h2>7. 输出文件</h2><ul><li><code>planning_report.html</code>（本报告）"
                 "</li><li><code>waypoints.csv</code> — 全部算法航点 GPS 数据</li>"
                 "<li><code>waypoints.json</code> — 同上，JSON 结构</li></ul>")
    parts.append("</div></body></html>")
    return "".join(parts)


# ========================= 航点文件输出 =========================
def write_waypoints(results, outdir):
    csv_path = outdir / "waypoints.csv"
    json_path = outdir / "waypoints.json"
    data = {}
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh)
        w.writerow(["algorithm", "seq", "lat", "lon",
                    "dist_from_prev_m", "bearing_deg"])
        for r in results:
            if r.error:
                continue
            rows = waypoint_rows(r.points_geo)
            data[r.name] = [
                dict(seq=row["seq"], lat=round(row["lat"], 7),
                     lon=round(row["lon"], 7),
                     dist_from_prev_m=round(row["dist"], 2),
                     bearing_deg=None if row["bearing"] is None
                     else round(row["bearing"], 1))
                for row in rows]
            for row in rows:
                w.writerow([r.name, row["seq"], f"{row['lat']:.7f}",
                            f"{row['lon']:.7f}", f"{row['dist']:.1f}",
                            "" if row["bearing"] is None
                            else f"{row['bearing']:.1f}"])
    with open(json_path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    return csv_path, json_path


# ========================= 控制台输出 =========================
def _dw(s):
    """显示宽度（东亚宽字符按 2 列）。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1
               for c in str(s))


def _pad(s, width):
    s = str(s)
    return s + " " * max(0, width - _dw(s))


def print_console(ctx, results, csv_path, json_path, report_path):
    line = "=" * 78
    print(line)
    print("路径规划仿真  path_planning_sim.py")
    print(line)
    print(f"地面端: ({ctx['start'][0]:.6f}, {ctx['start'][1]:.6f})   "
          f"空中目标: ({ctx['end'][0]:.6f}, {ctx['end'][1]:.6f})")
    print(f"直线距离(大圆): {ctx['direct'] / 1000:.3f} km   "
          f"栅格分辨率: {ctx['grid_res']:.0f} m   "
          f"随机种子: {ctx['seed']}")
    hdg_note = "（缺省=对准目标方位）" if ctx.get("hdg_auto") else "（自定义）"
    print(f"起飞/抵达航向: {ctx['start_hdg']:.1f}° / {ctx['end_hdg']:.1f}°"
          f"{hdg_note}   Dubins 最小转弯半径: {ctx['turn_radius']:.0f} m")
    print(f"禁飞区: {ctx['obstacles_text']}")
    print()
    print("【算法仿真结果】（按直线度降序，1.0000 = 理想直线）")
    header = ("算法", "直线度", "路径长(m)", "最大横偏(m)",
              "航点数", "转角和(°)", "扩展节点")
    widths = (28, 8, 11, 11, 7, 10, 9)
    print("  " + "  ".join(_pad(h, w) for h, w in zip(header, widths)))
    ranked = sorted((r for r in results if r.metrics),
                    key=lambda r: r.metrics.straightness, reverse=True)
    for r in ranked:
        m = r.metrics
        ex = r.stats.get("expanded_nodes", "—")
        row = (r.name, f"{m.straightness:.4f}", f"{m.length_m:.1f}",
               f"{m.max_xtrack_m:.1f}", m.n_wp,
               f"{m.total_turn_deg:.1f}", ex)
        print("  " + "  ".join(_pad(v, w) for v, w in zip(row, widths)))
    for r in results:
        if r.error:
            print(f"  {_pad(r.name, widths[0])}  规划失败: {r.error}")
    print()
    valid = [r for r in results if r.metrics and r.key != "linear"]
    if valid:
        best = max(valid, key=lambda r: r.metrics.straightness)
        worst = min(valid, key=lambda r: r.metrics.straightness)
        bs = best.metrics.straightness
        tied = [r.name for r in valid
                if abs(r.metrics.straightness - bs) <= 1e-4]
        print("【最接近直线的规划算法】")
        print(f"  {'、'.join(tied)}   直线度 = {bs:.4f}  （偏离 {(1 - bs) * 100:.4f}%）")
        print(f"  偏离最大: {worst.name}   "
              f"直线度 = {worst.metrics.straightness:.4f}")
        print("  对照: 直线插值基准 1.0000")
    print()
    print("【各算法功能与描述】")
    for i, d in enumerate(ALGO_DOCS, 1):
        print(f"  {i}. {d['name']}")
        print(f"     功能：{d['func']}")
        print(f"     描述：{d['desc']}")
    print()
    print("【结论与选型建议】")
    print("  · 地面端→空中目标(本场景)：航向对准目标时 Dubins/贝塞尔/APF/GA/平滑类")
    print("    均≈基准直线(1.0000)；需以指定方位进抵目标选杜宾斯(最短可行+转弯半径下限)，")
    print("    只需平滑连续曲率选贝塞尔(S 形，不保证最短)；两者均不避障。")
    print("  · 只论贴合直线：A* / Dijkstra 因栅格台阶量化略低(≈0.97)；")
    print("    RRT 原始折线最不直，须捷径平滑。")
    print("  · 需绕禁飞区：A* 给出全局最优且可复现，建议 A* + 路径平滑(捷径+样条)输出航路；")
    print("    APF 平滑但可能陷入局部极小；GA 随机、需固定种子复现；RRT+平滑最灵活。")
    print("  · 工程常用组合：A*（全局骨架）→ 捷径/样条平滑（拉直平滑）→ 转弯约束校验。")
    print()
    print("【输出文件】")
    print(f"  报告: {report_path}")
    print(f"  航点: {csv_path}")
    print(f"        {json_path}")
    print("  （加 --show-waypoints 可在控制台打印全部航点）")


def print_waypoints(results):
    for r in results:
        print()
        print(f"── {r.name} " + "─" * max(0, 60 - _dw(r.name)))
        if r.error:
            print(f"  规划失败: {r.error}")
            continue
        for row in waypoint_rows(r.points_geo):
            b = "  ---" if row["bearing"] is None else f"{row['bearing']:6.1f}"
            print(f"  #{row['seq']:<3d}  lat={row['lat']:.7f}  "
                  f"lon={row['lon']:.7f}  d={row['dist']:8.1f} m  hdg={b}°")


# ========================= 主流程 =========================
def parse_coord(s, label):
    parts = s.replace("，", ",").split(",")
    if len(parts) != 2:
        raise ValueError(f"{label}格式应为 '纬度,经度'，收到: {s!r}")
    lat, lon = float(parts[0]), float(parts[1])
    if not -90 <= lat <= 90:
        raise ValueError(f"{label}纬度超范围 [-90,90]: {lat}")
    if not -180 <= lon <= 180:
        raise ValueError(f"{label}经度超范围 [-180,180]: {lon}")
    return lat, lon


def _ask(prompt, default):
    try:
        got = input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        got = ""
    return got or default


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="地面端→空中目标 GPS 路径规划仿真（10 种算法对比）")
    ap.add_argument("--start", help='地面端 "纬度,经度"，如 39.905,116.409')
    ap.add_argument("--end", help='空中目标 "纬度,经度"，如 39.970,116.480')
    ap.add_argument("--start-hdg", type=float, default=None,
                    help="起飞航向(度,0=正北)，缺省=对准目标方位")
    ap.add_argument("--end-hdg", type=float, default=None,
                    help="目标点抵达航向(度,0=正北)，缺省=同起飞航向")
    ap.add_argument("--turn-radius", type=float, default=500.0,
                    help="Dubins 最小转弯半径(米)，默认 500")
    ap.add_argument("--obstacles", default="",
                    help='矩形禁飞区 "lat1,lon1,lat2,lon2;..."')
    ap.add_argument("--grid-res", type=float, default=0.0,
                    help="栅格分辨率(米)，0=按距离自动")
    ap.add_argument("--seed", type=int, default=42, help="随机种子(默认 42)")
    ap.add_argument("--outdir", default="", help="输出目录(默认脚本所在目录)")
    ap.add_argument("--show-waypoints", action="store_true",
                    help="控制台打印全部航点")
    args = ap.parse_args(argv)
    if args.turn_radius <= 0:
        print("参数错误: --turn-radius 必须 > 0", file=sys.stderr)
        return 2

    try:
        start_s = args.start or _ask(
            '请输入地面端 GPS(纬度,经度) [39.905,116.409]: ', "39.905,116.409")
        end_s = args.end or _ask(
            '请输入空中目标 GPS(纬度,经度) [39.970,116.480]: ', "39.970,116.480")
        start = parse_coord(start_s, "地面端")
        end = parse_coord(end_s, "空中目标")
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2

    # ENU 投影：原点取起点，故 start=(0,0)
    sx, sy = geo_to_enu(start[0], start[1], start[0], start[1])
    gx, gy = geo_to_enu(end[0], end[1], start[0], start[1])
    enu_start, enu_goal = (sx, sy), (gx, gy)
    straight = dist(enu_start, enu_goal)
    if straight < 10:
        print("参数错误: 起终点距离过近(<10 m)，无法仿真", file=sys.stderr)
        return 2
    try:
        rects = parse_obstacles(args.obstacles, start[0], start[1]) \
            if args.obstacles.strip() else []
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2

    grid_res = args.grid_res if args.grid_res > 0 \
        else min(500.0, max(30.0, straight / 150.0))
    pad = max(500.0, straight * 0.25)
    bounds = (min(enu_start[0], enu_goal[0]) - pad,
              min(enu_start[1], enu_goal[1]) - pad,
              max(enu_start[0], enu_goal[0]) + pad,
              max(enu_start[1], enu_goal[1]) + pad)
    n_seg = max(8, min(60, int(straight / 500)))
    rrt_step = max(grid_res, straight / 100)
    apf_spacing = max(grid_res * 0.5, straight / 20)
    max_gap = max(50.0, straight / 20)  # 统一航点加密：输出点距不超过该值
    # 起飞/抵达航向（度,0=正北）：缺省 = 对准目标方位（ENU 平面航向）
    auto_hdg = heading_enu(enu_start, enu_goal)
    start_hdg = auto_hdg if args.start_hdg is None else args.start_hdg
    end_hdg = start_hdg if args.end_hdg is None else args.end_hdg
    hdg_auto = args.start_hdg is None and args.end_hdg is None

    print(f"正在仿真：直线距离 {straight / 1000:.2f} km，"
          f"栅格 {grid_res:.0f} m，禁飞区 {len(rects)} 个 ...")

    results: list[PlanResult] = []
    grid = None
    astar_pts = None

    def run(key, name, fn):
        nonlocal astar_pts
        pts, stats, err, ms = timed_exec(fn)
        res = PlanResult(key=key, name=name, points_enu=pts,
                         stats=stats, error=err, elapsed_ms=ms)
        if not err and len(pts) >= 2:
            pts = densify(pts, max_gap)
            res.points_enu = pts
            geo = [enu_to_geo(x, y, start[0], start[1]) for x, y in pts]
            geo[0], geo[-1] = start, end
            res.points_geo = geo
            res.metrics = compute_metrics(pts, geo, start, end)
        results.append(res)
        if key == "astar" and not err:
            astar_pts = pts
        return res

    # 1) 基准直线
    run("linear", "直线插值(基准)",
        lambda: plan_linear(enu_start, enu_goal, n_seg))
    # 2) 栅格(供 A*/Dijkstra 共用)
    try:
        grid = Grid(bounds, grid_res, rects, enu_start, enu_goal)
    except Exception as exc:  # noqa: BLE001
        print(f"栅格构建失败: {exc}", file=sys.stderr)
        return 2
    run("astar", "A* 栅格寻路", lambda: plan_astar(grid, enu_start, enu_goal))
    run("dijkstra", "Dijkstra 栅格寻路",
        lambda: plan_dijkstra(grid, enu_start, enu_goal))
    # 3) RRT / RRT+捷径平滑
    run("rrt", "RRT 快速扩展随机树",
        lambda: plan_rrt(enu_start, enu_goal, rects, bounds, rrt_step,
                         random.Random(args.seed)))
    run("rrt_smooth", "RRT + 捷径平滑",
        lambda: plan_rrt(enu_start, enu_goal, rects, bounds, rrt_step,
                         random.Random(args.seed + 1), do_smooth=True))
    # 4) 人工势场
    run("apf", "人工势场法(APF)",
        lambda: plan_apf(enu_start, enu_goal, rects, apf_spacing))
    # 5) 遗传算法。无禁飞区时限定在起终点走廊(包围盒外扩 15%)加速收敛；
    #    有禁飞区时使用与栅格一致的全范围，便于绕行。
    if rects:
        ga_bounds = bounds
    else:
        ga_pad = 0.15 * straight
        ga_bounds = (min(enu_start[0], enu_goal[0]) - ga_pad,
                     min(enu_start[1], enu_goal[1]) - ga_pad,
                     max(enu_start[0], enu_goal[0]) + ga_pad,
                     max(enu_start[1], enu_goal[1]) + ga_pad)
    run("ga", "遗传算法(GA)",
        lambda: plan_ga(enu_start, enu_goal, ga_bounds,
                        random.Random(args.seed + 2), rects=rects))
    # 6) A* 路径的捷径 + 样条平滑
    def _smooth():
        if astar_pts is None:
            raise PlanningError("依赖 A* 结果，A* 规划失败")
        return plan_smooth(astar_pts, rects, grid_res / 2, args.seed + 3)

    run("smooth", "A* + 路径平滑(捷径+样条)", _smooth)
    # 7) 杜宾斯：最小转弯半径约束下的最短可行航迹
    run("dubins", "杜宾斯(Dubins)最短航迹",
        lambda: plan_dubins(enu_start, enu_goal, start_hdg, end_hdg,
                            args.turn_radius, grid_res / 2, rects))
    # 8) 贝塞尔：端点切向匹配航向的平滑曲线
    run("bezier", "贝塞尔(Bezier)平滑航迹",
        lambda: plan_bezier(enu_start, enu_goal, start_hdg, end_hdg,
                            grid_res / 2, rects))

    # 输出
    outdir = Path(args.outdir) if args.outdir else Path(__file__).resolve().parent
    outdir.mkdir(parents=True, exist_ok=True)
    csv_path, json_path = write_waypoints(results, outdir)
    ctx = dict(
        start=start, end=end, enu_start=enu_start, enu_goal=enu_goal,
        direct=haversine(start[0], start[1], end[0], end[1]),
        grid_res=grid_res, seed=args.seed, rects=rects, results=results,
        stamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        obstacles_text=(args.obstacles if args.obstacles.strip() else "无"),
        start_hdg=start_hdg % 360.0, end_hdg=end_hdg % 360.0,
        turn_radius=args.turn_radius, hdg_auto=hdg_auto,
    )
    report_path = outdir / "planning_report.html"
    report_path.write_text(build_report(ctx), encoding="utf-8")

    print()
    print_console(ctx, results, csv_path, json_path, report_path)
    if args.show_waypoints:
        print()
        print("【全部航点 GPS 数据】")
        print_waypoints(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
