#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""多算法 GPS 路径规划仿真 —— 命令行入口。

示例:
    python main.py --list
    python main.py --start 34.230,108.940 --goal 34.255,108.975
    python main.py --start 34.230,108.940 --goal 34.255,108.975 --demo-obstacles
    python main.py --start 34.230,108.940 --goal 34.255,108.975 \
                   --algo astar,rrtstar,apf --obstacles my_obs.json
    python main.py                      # 交互模式
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

# Windows 控制台默认 GBK, 统一改为 UTF-8 输出, 避免中文/符号编码错误
for _stream in (sys.stdout, sys.stderr):
    try:
        if _stream.encoding and _stream.encoding.lower() != "utf-8":
            _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import numpy as np

from pathplanning import (LocalFrame, Obstacle, REGISTRY, available,
                          bearing_deg, evaluate)
from pathplanning.metrics import rank

DEFAULT_START = (34.230, 108.940)
DEFAULT_GOAL = (34.255, 108.975)


# ---------------- 输入解析 ----------------

def parse_latlon(s: str) -> tuple[float, float]:
    parts = [p for p in s.replace(",", " ").split() if p]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"坐标格式应为 '纬度,经度': {s!r}")
    return float(parts[0]), float(parts[1])


def load_obstacles(path: str) -> list[Obstacle]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    obs = []
    for i, d in enumerate(data):
        obs.append(Obstacle(lat=float(d["lat"]), lon=float(d["lon"]),
                            radius_m=float(d["radius_m"]),
                            label=d.get("label", f"禁飞{i + 1}")))
    return obs


def demo_obstacles(start_ll, goal_ll, frame: LocalFrame) -> list[Obstacle]:
    """在起终点连线附近自动摆 2 个禁飞圆(压在线上)，用于算法对比演示。"""
    s = np.array(frame.to_xy(*start_ll))
    g = np.array(frame.to_xy(*goal_ll))
    d = float(np.linalg.norm(g - s))
    u = (g - s) / d
    perp = np.array([-u[1], u[0]])
    r = max(0.075 * d, 40.0)
    specs = [(0.42, +0.055 * d), (0.66, -0.050 * d)]
    out = []
    for k, (t, off) in enumerate(specs):
        c = s + u * (t * d) + perp * off
        lat, lon = frame.to_latlon(float(c[0]), float(c[1]))
        out.append(Obstacle(lat=lat, lon=lon, radius_m=r, label=f"禁飞{k + 1}"))
    return out


# ---------------- 输出 ----------------

def dwidth(s: str) -> int:
    """终端显示宽度(中文按2列)。"""
    return sum(2 if ord(ch) > 0x2E80 else 1 for ch in s)


def print_table(rows: list[dict], obstacles) -> None:
    heads = ["算法", "状态", "长度m", "直线度", "平滑度", "多余转弯°",
             "最小转弯m", "最近障碍m", "航点数", "耗时ms", "评分"]
    fmts = ["{}", "{}", "{:.1f}", "{:.4f}", "{:.4f}", "{:.2f}",
            "{:.1f}", "{}", "{}", "{:.1f}", "{:.4f}"]
    table = []
    for r in rows:
        cells = [r["label"], r["status"], r["length_m"], r["straightness"],
                 r["smoothness"], r["excess_turn_deg"],
                 r["min_turn_radius_m"],
                 ("—" if not obstacles else f"{r['clearance_m']:.1f}"),
                 r["n_points"], r["compute_ms"], r["score"]]
        table.append([f.format(c) if not isinstance(c, str) else c
                      for f, c in zip(fmts, cells)])
    widths = [max(dwidth(h), *(dwidth(row[i]) for row in table))
              for i, h in enumerate(heads)]
    line = "  ".join(h + " " * (w - dwidth(h)) for h, w in zip(heads, widths))
    print(line)
    print("-" * len(line))
    for row in table:
        print("  ".join(c + " " * (w - dwidth(c)) for c, w in zip(row, widths)))


def save_outputs(out_dir, results, metrics, frame, obstacles, config):
    os.makedirs(out_dir, exist_ok=True)
    # 1) 每个算法的 GPS 航点集合
    for name, res in results.items():
        gps = res.gps_points(frame)
        with open(os.path.join(out_dir, f"gps_{name}.csv"), "w",
                  newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["idx", "lat", "lon", "x_m", "y_m"])
            for i, ((la, lo), (x, y)) in enumerate(zip(gps, res.points)):
                w.writerow([i, f"{la:.8f}", f"{lo:.8f}", f"{x:.3f}", f"{y:.3f}"])
    # 2) 指标汇总
    keys = ["label", "algo", "status", "message", "n_points", "length_m",
            "direct_m", "straightness", "smoothness", "excess_turn_deg",
            "max_curvature", "min_turn_radius_m", "rms_curvature",
            "clearance_m", "score", "compute_ms"]
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for m in metrics:
            w.writerow({k: ("" if m.get(k) is None or
                            (isinstance(m.get(k), float) and math.isinf(m.get(k)))
                            else m.get(k)) for k in keys})
    # 3) JSON(含配置、指标、GPS 集合)
    payload = {"config": config,
               "results": {m["algo"]: {
                               "label": m["label"],
                               "status": results[m["algo"]].status,
                               "message": results[m["algo"]].message,
                               "compute_ms": round(results[m["algo"]].compute_ms, 2),
                               "extra": results[m["algo"]].extra,
                               "metrics": m,
                               "gps": [[round(la, 7), round(lo, 7)] for la, lo in
                                       results[m["algo"]].gps_points(frame)]}
                           for m in metrics}}
    with open(os.path.join(out_dir, "results.json"), "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return payload


# ---------------- 主流程 ----------------

def run(start_ll, goal_ll, algo_names, obstacles_ll, args):
    frame = LocalFrame.from_points([start_ll, goal_ll] +
                                   [(o[0], o[1]) for o in obstacles_ll])
    obstacles = [Obstacle(lat, lon, r, lb or f"禁飞{i + 1}")
                 for i, (lat, lon, r, lb) in enumerate(obstacles_ll)]
    for ob in obstacles:
        ob.bind_frame(frame)
    start_xy = frame.to_xy(*start_ll)
    goal_xy = frame.to_xy(*goal_ll)

    d = math.dist(start_xy, goal_xy)
    print(f"\n起点 GPS : {start_ll[0]:.6f}, {start_ll[1]:.6f}")
    print(f"目标 GPS : {goal_ll[0]:.6f}, {goal_ll[1]:.6f}")
    print(f"直线距离 : {d:.1f} m ({d / 1000:.3f} km), 方位角 "
          f"{bearing_deg(*start_ll, *goal_ll):.1f}°")
    print(f"禁飞区   : {len(obstacles)} 个")
    for ob in obstacles:
        print(f"   - {ob.label}: {ob.lat:.6f},{ob.lon:.6f} r={ob.radius_m:.0f}m")

    results, metrics = {}, []
    for name in algo_names:
        cls = REGISTRY[name]
        params = {}
        if cls.need_heading:
            params["radius"] = args.turn_radius
        planner = cls(frame, obstacles, safety_m=args.safety, seed=args.seed,
                      **params)
        res = planner.plan(start_xy, goal_xy,
                           start_heading=args.start_heading,
                           goal_heading=args.goal_heading)
        m = evaluate(res.points, obstacles)
        results[name] = res
        metrics.append({"label": res.label, "algo": name, **m,
                        "status": res.status, "message": res.message,
                        "compute_ms": res.compute_ms})
        note = f" [{res.message}]" if res.message else ""
        print(f"  ✓ {res.label:<14} 状态={res.status} 耗时={res.compute_ms:.1f}ms{note}")

    by_algo = {m["algo"]: m for m in metrics}
    ordered = rank(by_algo)
    metrics_sorted = [by_algo[k] for k, _ in ordered]
    print("\n" + "=" * 100)
    print("规划结果指标(先按可行性排序: 可行>警告>失败; 同级按综合评分, 评分越接近1越接近『平滑直线』)")
    print("=" * 100)
    print_table(metrics_sorted, obstacles)

    print("\n评分排名(可行结果优先, 越靠前越趋向平滑直线):")
    for i, (k, m) in enumerate(ordered, 1):
        st = "" if m["status"] == "ok" else f"  [{m['status']}: 不避障/失败]"
        print(f"  {i}. {m['label']:<16} 评分={m['score']:.4f}  "
              f"(直线度 {m['straightness']:.4f} × 平滑度 {m['smoothness']:.4f}){st}")
    failed = [m for m in metrics if m["status"] == "failed"]
    if failed:
        print("\n注意: " + ", ".join(f"{m['label']}失败({m['message']})"
                                    for m in failed))

    out_dir = args.out
    config = {"start": list(start_ll), "goal": list(goal_ll),
              "start_heading": args.start_heading, "goal_heading": args.goal_heading,
              "safety_m": args.safety, "seed": args.seed,
              "turn_radius_m": args.turn_radius, "algos": list(algo_names),
              "obstacles": [{"lat": o[0], "lon": o[1], "radius_m": o[2],
                             "label": o[3]} for o in obstacles_ll]}
    save_outputs(out_dir, results, metrics_sorted, frame, obstacles, config)

    if not args.no_plot:
        from pathplanning import viz
        plot_res = {m["algo"]: results[m["algo"]] for m in metrics_sorted}
        viz.plot_paths(plot_res, obstacles, start_xy, goal_xy,
                       os.path.join(out_dir, "paths.png"),
                       title=f"路径对比  {d:.0f}m")
        viz.plot_metrics(metrics_sorted, os.path.join(out_dir, "metrics.png"))
        viz.plot_detail(plot_res, obstacles, start_xy, goal_xy,
                        os.path.join(out_dir, "detail.png"))
        print(f"\n图像已保存: {os.path.join(out_dir, 'paths.png')} 等")

    print(f"GPS 航点/指标/JSON 已保存到目录: {out_dir}/")
    print(f"  每个算法一个 gps_<algo>.csv, 汇总 summary.csv, 全量 results.json")
    return results, metrics_sorted


def interactive(args):
    print("=" * 60)
    print(" 多算法 GPS 路径规划仿真 (垂起FPV)")
    print("=" * 60)
    s = input(f"起点 GPS '纬度,经度' [{DEFAULT_START[0]},{DEFAULT_START[1]}]: ").strip()
    start = parse_latlon(s) if s else DEFAULT_START
    g = input(f"目标 GPS '纬度,经度' [{DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]}]: ").strip()
    goal = parse_latlon(g) if g else DEFAULT_GOAL
    print("可选算法:")
    for a, lb in available():
        print(f"   {a:<10} {lb}")
    a = input("选择算法(逗号分隔, 或 all) [all]: ").strip() or "all"
    obs = input("自动生成示例禁飞区? (y/n) [y]: ").strip().lower()
    if obs != "n":
        args.demo_obstacles = True
    return start, goal, a


def main(argv=None):
    ap = argparse.ArgumentParser(description="多算法 GPS 路径规划仿真")
    ap.add_argument("--start", type=parse_latlon, help="起点 GPS '纬度,经度'")
    ap.add_argument("--goal", type=parse_latlon, help="目标 GPS '纬度,经度'")
    ap.add_argument("--algo", default="all",
                    help="算法, 逗号分隔或 all (默认 all)")
    ap.add_argument("--obstacles", help="禁飞区 JSON 文件")
    ap.add_argument("--demo-obstacles", action="store_true",
                    help="自动生成示例禁飞区(压在直连线上)")
    ap.add_argument("--start-heading", type=float, default=None,
                    help="初始航向(地理角度, 正北=0顺时针), 默认朝向目标")
    ap.add_argument("--goal-heading", type=float, default=None,
                    help="末端航向, 默认与初始航向一致")
    ap.add_argument("--turn-radius", type=float, default=30.0,
                    help="Dubins 最小转弯半径(米), 默认 30")
    ap.add_argument("--safety", type=float, default=0.0,
                    help="禁飞区额外安全余量(米), 默认 0")
    ap.add_argument("--seed", type=int, default=42, help="随机种子")
    ap.add_argument("--out", default="out", help="输出目录, 默认 out/")
    ap.add_argument("--no-plot", action="store_true", help="不生成图像")
    ap.add_argument("--list", action="store_true", help="列出算法后退出")
    args = ap.parse_args(argv)

    if args.list:
        print("可用算法:")
        for a, lb in available():
            print(f"   {a:<10} {lb}")
        return 0

    if args.start is None or args.goal is None:
        args.start, args.goal, args.algo = interactive(args)

    if args.algo.strip().lower() == "all":
        algo_names = list(REGISTRY.keys())
    else:
        algo_names = [a.strip() for a in args.algo.split(",") if a.strip()]
        bad = [a for a in algo_names if a not in REGISTRY]
        if bad:
            print(f"未知算法: {bad}, 可用: {list(REGISTRY)}")
            return 2

    obstacles_ll: list[tuple] = []          # (lat, lon, radius, label)
    if args.obstacles:
        for ob in load_obstacles(args.obstacles):
            obstacles_ll.append((ob.lat, ob.lon, ob.radius_m, ob.label))
    if args.demo_obstacles:
        fr = LocalFrame.from_points([args.start, args.goal])
        obstacles_ll = [(o.lat, o.lon, o.radius_m, o.label)
                        for o in demo_obstacles(args.start, args.goal, fr)]

    run(args.start, args.goal, algo_names, obstacles_ll, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
