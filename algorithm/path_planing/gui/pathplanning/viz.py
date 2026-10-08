"""可视化：路径对比图、指标柱状图、各算法子图。

直接用 matplotlib.figure.Figure 出图(不经过 pyplot 的 figure manager)，
因此在任何后端下都能无窗口地保存 PNG：命令行 Agg 离线出图、GUI TkAgg 内嵌画布均可用。
"""
from __future__ import annotations

import math
import os

import matplotlib

# 命令行出图默认离线 Agg；GUI 启动时会设 FLIGHT_GUI=1，此时保留交互后端(TkAgg)。
if os.environ.get("FLIGHT_GUI") != "1":
    matplotlib.use("Agg")

import numpy as np
from matplotlib.figure import Figure
from matplotlib.patches import Circle

# 中文字体
for f in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS"):
    if any(f.lower() == x.name.lower()
           for x in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.sans-serif"] = [f]
        break
matplotlib.rcParams["axes.unicode_minus"] = False

COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
          "#42d4f4", "#f032e6", "#bfef45", "#fabed4", "#469990"]


def _draw_obstacles(ax, obstacles):
    for i, ob in enumerate(obstacles):
        ax.add_patch(Circle(ob.xy, ob.radius_m, fc="#ffcccc", ec="#cc0000",
                            lw=1.2, alpha=0.75, zorder=2,
                            label="禁飞区" if i == 0 else "_nolegend_"))
        if ob.label:
            ax.text(ob.xy[0], ob.xy[1], ob.label, ha="center", va="center",
                    fontsize=8, color="#990000", zorder=3)


def plot_paths(results, obstacles, start, goal, path_png, title=""):
    fig = Figure(figsize=(10, 8))
    ax = fig.add_subplot(111)
    _draw_obstacles(ax, obstacles)
    for i, (name, res) in enumerate(results.items()):
        c = COLORS[i % len(COLORS)]
        p = res.points
        ax.plot(p[:, 0], p[:, 1], color=c, lw=1.8, label=res.label, zorder=4)
        ax.plot(p[0, 0], p[0, 1], "o", color=c, ms=6, zorder=5)
    ax.plot(*start, "s", color="black", ms=9, zorder=6, label="起点")
    ax.plot(*goal, "*", color="gold", ms=16, mec="black", zorder=6, label="目标")
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.5)
    ax.set_xlabel("东向 x (m)")
    ax.set_ylabel("北向 y (m)")
    ax.set_title(title or "各算法规划路径对比 (局部坐标, 北向上)")
    ax.legend(loc="best", fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(path_png, dpi=140)


def plot_metrics(metric_table, path_png, title=""):
    """metric_table: list[dict] 已按评分排序。"""
    names = [m["label"] for m in metric_table]
    y = np.arange(len(names))
    fig = Figure(figsize=(15, max(4, 0.5 * len(names) + 1.5)))

    ax = fig.add_subplot(1, 3, 1)
    ax.barh(y, [m["straightness"] for m in metric_table], color="#4363d8", alpha=.85)
    ax.set_title("直线度 direct/length (越高越直)")
    ax.set_yticks(y); ax.set_yticklabels(names, fontsize=9)
    ax.invert_yaxis(); ax.set_xlim(0, 1.05)

    ax = fig.add_subplot(1, 3, 2)
    ax.barh(y, [m["smoothness"] for m in metric_table], color="#3cb44b", alpha=.85)
    ax.set_title("平滑度 exp(-多余转弯/45°) (越高越平滑)")
    ax.set_yticks(y); ax.set_yticklabels([])
    ax.invert_yaxis(); ax.set_xlim(0, 1.05)

    ax = fig.add_subplot(1, 3, 3)
    ax.barh(y, [m["score"] for m in metric_table], color="#f58231", alpha=.9)
    ax.set_title("综合评分 = 直线度 × 平滑度")
    ax.set_yticks(y); ax.set_yticklabels([])
    ax.invert_yaxis(); ax.set_xlim(0, 1.05)
    for i, m in enumerate(metric_table):
        ax.text(m["score"] + 0.01, i, f"{m['score']:.3f}", va="center", fontsize=9)

    fig.suptitle(title or "路径质量指标对比", fontsize=13)
    fig.tight_layout()
    fig.savefig(path_png, dpi=140)


def plot_detail(results, obstacles, start, goal, path_png, title=""):
    """每个算法一张子图, 显示 GPS 航点分布。"""
    n = len(results)
    if n == 0:
        return
    cols = min(3, n)
    rows = math.ceil(n / cols)
    fig = Figure(figsize=(5 * cols, 4.4 * rows))
    for k, (name, res) in enumerate(results.items()):
        ax = fig.add_subplot(rows, cols, k + 1)
        _draw_obstacles(ax, obstacles)
        p = res.points
        ax.plot(p[:, 0], p[:, 1], "-", color=COLORS[k % len(COLORS)], lw=1.6)
        step = max(1, len(p) // 60)
        ax.plot(p[::step, 0], p[::step, 1], ".", color=COLORS[k % len(COLORS)], ms=3)
        ax.plot(*start, "s", color="black", ms=6)
        ax.plot(*goal, "*", color="gold", ms=11, mec="black")
        ax.set_aspect("equal"); ax.grid(True, ls=":", alpha=0.4)
        ax.set_title(f"{res.label}  ({res.status})", fontsize=10)
    fig.suptitle(title or "各算法路径与航点(点)分布", fontsize=13)
    fig.tight_layout()
    fig.savefig(path_png, dpi=140)
