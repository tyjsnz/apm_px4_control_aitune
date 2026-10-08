#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""GPS 路径规划图形化仿真界面 (Tkinter + Matplotlib)。

功能:
  * 输入起点/目标 GPS，勾选算法，点击画布设起点/目标/禁飞区
  * 后台线程跑 8 种规划算法，指标表实时排名
  * 路径生成动画(逐条生长) + 四旋翼飞行回放动画(可暂停/调速/换算法)
  * 导出 GPS 航点 CSV / summary.csv / results.json / 对比 PNG

运行:
    python gui.py
"""
from __future__ import annotations

import json
import math
import os
import queue
import sys
import threading
import traceback

# ---- 后端与输出编码必须在导入 pyplot 之前设定 ----
os.environ.setdefault("FLIGHT_GUI", "1")
import matplotlib

matplotlib.use("TkAgg")

for _stream in (sys.stdout, sys.stderr):
    try:
        if _stream.encoding and _stream.encoding.lower() != "utf-8":
            _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon
from matplotlib.transforms import Affine2D

from main import demo_obstacles, save_outputs
from pathplanning import (REGISTRY, LocalFrame, Obstacle, available,
                          bearing_deg, evaluate)
from pathplanning import viz
from pathplanning.metrics import rank

COLORS = viz.COLORS
DEFAULT_START = (34.230, 108.940)
DEFAULT_GOAL = (34.255, 108.975)

STATUS_CN = {"ok": "可行", "warn": "警告", "failed": "失败", "partial": "部分"}


# ---------------------------------------------------------------- 说明文档
ALGO_HELP = {
"straight": """【直线航路(基准)】
原理: 起点到目标的几何直线，按方位角等距采样。
特点: 理论最优的"平滑直线"(直线度=1、多余转弯=0)，但完全没有避障能力，
      压到禁飞区即判"警告"。用作对照组，衡量其它算法绕障付出的代价。""",

"astar": """【A*栅格 + 视线拉直】
原理: 米制栅格 8 邻域 A* 搜索，代价 = 距离 + 转弯惩罚，启发项 = 欧氏距离
      (可采纳、无偏)；禁飞圆膨胀后标记为障碍。
平滑: 输出折线后做视线(LOS)拉直——从当前点贪心连到最远的可通视点
      (string-pulling)，再限幅拐点，得到"长直线 + 少量圆滑拐弯"。
特点: 解稳定、可复现、无随机性；障碍场景评分通常最高。
缺点: 拐点仍是折线(曲率不连续)，实飞需再加平滑层。""",

"rrtstar": """【RRT* 采样最优树 (Informed RRT*)】
原理: 随机撒点→最近节点延伸固定步长→半径 r 内 rewiring 使父代代价最优，
      得到渐近最优树；再用椭圆 informed 采样(以起终点为焦点)加速收敛。
平滑: 回溯路径后做视线拉直 + 转角平滑，去除树的锯齿抖动。
特点: 适合高维/复杂空间；随机性(固定 --seed 可复现)；解随迭代提升。
缺点: 质量不稳定、可能绕远，需要调步长/近邻半径/迭代数。""",

"apf": """【人工势场法 APF】
原理: 目标产生引力 F=k·Δ，禁飞圆产生斥力 F=k_rep(1/d−1/d0)/d²，
      按合力做梯度下降前向积分，进目标容差半径即成功。
工程要点(调参定版):
  · k_rep=2e6, d0=80m: 80m 外斥力可忽略，进圈后迅速主导
  · step=1.0m, max_turn=90°, beta=0.6: 小步长 + 航向限幅 + 阻尼防折返
  · eq_ratio=0.3: 引力斥力近平衡(局部极小)时立即触发逃逸机动，
    防止两点间来回振荡
特点: 实时性极好(每步 O(障碍数))，积分曲线天然连续。
缺点: 本质缺陷是局部极小；绕障距离大、直线度损失最多。""",

"dubins": """【Dubins 曲线 (定转弯半径)】
原理: 在最小转弯半径 R 约束下，连接两带航向端点的最短路径必为
      LSL / LSR / RSL / RSR / RLR / LRL 六种字之一。本实现用几何构造法:
      对起终圆求外/内公切线，拼三圆弧或双圆弧(已用 8505 个随机端点对
      验证端点/航向误差 = 0)。
特点: 圆弧+直线组合，曲率有界、航向连续，可飞性最好——直段就是完美直线，
      转弯是恒定半径圆弧，最符合垂起高速直飞。
缺点: 默认不避障(判"警告")；起终航向受限(--start-heading 可指定)。""",

"bspline": """【三次 B 样条平滑】
原理: 先用引导算法(有障碍 A*、无障碍直线)取控制点，再拟合三次均匀 B 样条
      P(t)=ΣN_{i,3}(t)·P_i；基函数局部支撑 → C² 连续(位置/切率/曲率连续)。
后处理: 按弧长等距重采样，并细采样检测样条与禁飞圆的碰撞，碰撞则回退。
特点: 平滑度最好(曲率连续 → 高速飞行无突变过载)，直线段几乎不变形。
缺点: 拐角处轻微"过切"，必须配碰撞检测兜底。""",

"minjerk": """【最小加加速度轨迹 (五次多项式 min-jerk)】
原理: 相邻航点间用五次多项式 p(t)=a0+…+a5·t⁵ 连接，两端固定位置/速度/
      加速度(6 个边界条件)，解 KKT 线性方程组最小化 ∫jerk²dt；
      段间自动 C² 连续。
特点: 动态学最优(加加速度最小 → 电机/桨载荷平顺)，无人机轨迹生成标准做法。
缺点: 拐点处"切内弯"偏离折线，需碰撞检测。""",

"pso": """【粒子群优化 PSO】
原理: 把路径表示为直线上 N 个航点的横向偏移向量 δ，每个粒子是一组 δ；
      适应度 J = w1·航程 + w2·Σ|二阶差分(δ)| + w3·碰撞惩罚 + w4·Σδ²，
      即"走得直 + 不打弯 + 不压障碍 + 别偏离直线"。
      迭代 v = w·v + c1·r1(pbest−x) + c2·r2(gbest−x)，w 线性递减。
特点: 不依赖网格，直接对"平滑+直线"做全局数值优化，固定 seed 可复现。
缺点: 计算量最大；罚函数权重需调(过罚=绕远，欠罚=撞障)。""",
}

METRIC_HELP = """【指标定义】(所有指标在 1m 等弧长重采样后的路径上计算)

· 直线度 straightness = 直线距离 / 实际航程 ∈(0,1]，越接近 1 越"直"
· 多余转弯 excess_turn = Σ|Δ航向| − |净航向变化|(度)
  航向先做窗口 5 低通平滑。理想直线为 0°；绕障、锯齿、打转都会让它增大
· 平滑度 smoothness = exp(−多余转弯 / 45°) ∈(0,1]，越接近 1 越平滑
· 综合评分 score = 直线度 × 平滑度，越接近 1 越接近"平滑直线"
· 最小转弯半径 = 1 / 最大曲率，判断飞机能不能飞
· 最近障碍 = 路径到禁飞圆边界的最小净距(负值 = 已侵入)

排序规则: 先按可行性 可行 > 警告 > 失败；同级按综合评分降序。
"警告"= 轨迹压到禁飞区(如直线/Dubins 这类不避障的算法)。"""


# ================================================================ 主界面
class FlightGUI(tk.Tk):
    GROW_TICK_MS = 25          # 路径生长动画帧间隔
    GROW_STEPS = 45            # 每条路径约 45 帧生成完
    FLIGHT_TICK_MS = 30        # 飞行回放帧间隔
    FLIGHT_BASE_SEC = 12.0     # 1x 速度下走完全程的基准秒数

    # ---------------------------------------------------------- 初始化
    def __init__(self):
        super().__init__()
        self.title("GPS 路径规划仿真 — 垂起四旋翼 FPV")
        self.geometry("1520x920")
        self.minsize(1180, 740)

        self.start = list(DEFAULT_START)
        self.goal = list(DEFAULT_GOAL)
        self.obstacles: list[Obstacle] = []
        self.results: dict = {}
        self.metrics: dict = {}
        self.metrics_sorted: list[dict] = []
        self.ordered: list = []
        self.frame: LocalFrame | None = None

        self._path_lines: dict[str, Line2D] = {}
        self._ac: list = []
        self._trail: Line2D | None = None
        self._label2algo: dict[str, str] = {}

        self._job = None
        self._anim = "idle"        # idle / growth / ready / flight / done
        self._paused = False
        self._q: queue.Queue = queue.Queue()
        self._running = False
        self._stop_evt = threading.Event()
        self._rows: list = []
        self._sort_rev: dict = {}

        self._colors = {name: COLORS[i % len(COLORS)]
                        for i, name in enumerate(REGISTRY)}

        self._build_ui()
        # 鼠标滚轮滚动左侧设置面板(只在指针位于左面板内时生效)
        self.bind_all("<MouseWheel>", self._on_wheel, "+")
        self._load_demo_obstacles(silent=True)
        self._scene_changed()
        self._set_status("就绪 —— 已载入示例禁飞区，点击「运行规划」开始")
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind("<Control-r>", lambda e: self.run())
        self.bind("<space>", self._on_space)

    # ---------------------------------------------------------- UI 搭建
    def _build_ui(self):
        self.option_add("*Font", ("Microsoft YaHei", 10))
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        # ---- 底部状态栏(先打包, 保证在最下层) ----
        self.status_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN,
                  anchor="w", padding=(6, 2)).pack(side=tk.BOTTOM, fill=tk.X)

        # ---- 底部动画控制 ----
        anim = ttk.Frame(self, padding=(6, 4))
        anim.pack(side=tk.BOTTOM, fill=tk.X)
        self.phase_var = tk.StringVar(value="待机")
        self.progress = ttk.Progressbar(anim, maximum=100, length=260)
        self.progress.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(anim, textvariable=self.phase_var, width=8).pack(side=tk.LEFT)
        ttk.Label(anim, text="速度").pack(side=tk.LEFT, padx=(10, 2))
        self.speed = tk.Scale(anim, from_=0.25, to=4.0, resolution=0.25,
                              orient=tk.HORIZONTAL, length=170, showvalue=True,
                              highlightthickness=0)
        self.speed.set(1.0)
        self.speed.pack(side=tk.LEFT)
        ttk.Label(anim, text="飞行算法").pack(side=tk.LEFT, padx=(14, 4))
        self.flight_var = tk.StringVar()
        self.flight_combo = ttk.Combobox(anim, textvariable=self.flight_var,
                                         state="readonly", width=26)
        self.flight_combo.pack(side=tk.LEFT)
        self.auto_fly = tk.BooleanVar(value=True)
        ttk.Checkbutton(anim, text="生成后自动飞行",
                        variable=self.auto_fly).pack(side=tk.LEFT, padx=12)

        # ---- 顶部工具栏 ----
        bar = ttk.Frame(self, padding=(6, 5))
        bar.pack(side=tk.TOP, fill=tk.X)
        self.btn_run = ttk.Button(bar, text="▶ 运行规划", command=self.run)
        self.btn_run.pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="▶ 播放仿真",
                   command=self._on_play).pack(side=tk.LEFT, padx=3)
        self.btn_pause = ttk.Button(bar, text="⏸ 暂停", command=self._toggle_pause)
        self.btn_pause.pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="⏹ 停止", command=self._on_stop).pack(side=tk.LEFT, padx=3)
        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Button(bar, text="📂 导出结果",
                   command=self._on_export).pack(side=tk.LEFT, padx=3)
        ttk.Button(bar, text="❓ 算法说明", command=self._on_help).pack(side=tk.LEFT, padx=3)
        self.v_rank = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.v_rank, foreground="#0a78a0",
                  font=("Microsoft YaHei", 10, "bold")).pack(side=tk.RIGHT, padx=8)

        # ---- 指标表(整宽, 位于画布下方) ----
        tf = ttk.Frame(self, padding=(6, 0))
        tf.pack(side=tk.BOTTOM, fill=tk.X)
        self._build_table(tf)

        # ---- 主体: 左侧控制面板 + 中间画布 ----
        body = ttk.Frame(self, padding=(6, 4))
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        self._build_left(ttk.Frame(body))
        self._build_center(ttk.Frame(body))
        self._lf.grid(row=0, column=0, sticky="nsw", padx=(0, 6))
        self._cf.grid(row=0, column=1, sticky="nsew")

    # -------- 左侧：输入 / 算法 / 禁飞区 / 参数 --------
    def _build_left(self, panel):
        self._lf = panel
        panel.grid_propagate(False)
        panel.configure(width=300)

        # 内容较多(禁飞区 + 参数), 做成可滚动面板, 低分屏下不会被裁掉
        cv = tk.Canvas(panel, highlightthickness=0, bd=0, background="#ececec")
        sb = ttk.Scrollbar(panel, orient=tk.VERTICAL, command=cv.yview)
        cv.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        lf = ttk.Frame(cv)
        win = cv.create_window((0, 0), window=lf, anchor="nw")
        lf.bind("<Configure>", lambda e: cv.configure(scrollregion=cv.bbox("all")))
        cv.bind("<Configure>", lambda e: cv.itemconfigure(win, width=e.width))
        self._left_canvas = cv

        # 起终点
        f = ttk.LabelFrame(lf, text="起终点 GPS", padding=6)
        f.pack(fill=tk.X, pady=(0, 6))
        self.v_slat = tk.StringVar(value=str(DEFAULT_START[0]))
        self.v_slon = tk.StringVar(value=str(DEFAULT_START[1]))
        self.v_glat = tk.StringVar(value=str(DEFAULT_GOAL[0]))
        self.v_glon = tk.StringVar(value=str(DEFAULT_GOAL[1]))
        ttk.Label(f, text="").grid(row=0, column=0)
        ttk.Label(f, text="纬度", font=("Microsoft YaHei", 8),
                  foreground="#666").grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="经度", font=("Microsoft YaHei", 8),
                  foreground="#666").grid(row=0, column=2, sticky="w")
        for r, (lbl, la, lo) in enumerate([("起点", self.v_slat, self.v_slon),
                                           ("目标", self.v_glat, self.v_glon)], 1):
            ttk.Label(f, text=lbl, width=4).grid(row=r, column=0, sticky="w", pady=1)
            for c, var in ((1, la), (2, lo)):
                e = ttk.Entry(f, textvariable=var, width=13)
                e.grid(row=r, column=c, sticky="ew", padx=(0, 3), pady=1)
                e.bind("<Return>", lambda _e: self._apply_gps())
        ttk.Button(f, text="应用坐标", command=self._apply_gps).grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(5, 0))

        # 点击模式
        f = ttk.LabelFrame(lf, text="画布点击模式", padding=6)
        f.pack(fill=tk.X, pady=(0, 6))
        self.click_mode = tk.StringVar(value="选择")
        cb = ttk.Combobox(f, textvariable=self.click_mode, state="readonly",
                          values=["选择", "设为起点", "设为目标", "添加禁飞区"])
        cb.pack(fill=tk.X)
        ttk.Label(f, text="提示: 在画布上单击即可操作；\n导航工具栏可缩放/平移。",
                  foreground="#666", font=("Microsoft YaHei", 8)).pack(anchor="w", pady=(4, 0))

        # 算法
        f = ttk.LabelFrame(lf, text="算法选择", padding=6)
        f.pack(fill=tk.X, pady=(0, 6))
        self.algo_vars: dict[str, tk.BooleanVar] = {}
        for i, (name, label) in enumerate(available()):
            var = tk.BooleanVar(value=True)
            self.algo_vars[name] = var
            ttk.Checkbutton(f, text=label, variable=var).grid(
                row=i, column=0, sticky="w", pady=1)
        rowb = ttk.Frame(f)
        rowb.grid(row=len(self.algo_vars), column=0, sticky="ew", pady=(5, 0))
        ttk.Button(rowb, text="全选", width=6,
                   command=lambda: [v.set(True) for v in self.algo_vars.values()]
                   ).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(rowb, text="清空", width=6,
                   command=lambda: [v.set(False) for v in self.algo_vars.values()]
                   ).pack(side=tk.LEFT)

        # 禁飞区
        f = ttk.LabelFrame(lf, text="禁飞区", padding=6)
        f.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        cols = ("label", "lat", "lon", "r")
        self.obs_tree = ttk.Treeview(f, columns=cols, show="headings", height=6)
        for c, t, w in (("label", "名称", 62), ("lat", "纬度", 78),
                        ("lon", "经度", 78), ("r", "半径m", 46)):
            self.obs_tree.heading(c, text=t)
            self.obs_tree.column(c, width=w, anchor="center")
        self.obs_tree.pack(fill=tk.X)
        self.obs_tree.bind("<Double-1>", self._on_obs_dblclick)
        btns = ttk.Frame(f)
        btns.pack(fill=tk.X, pady=(5, 0))
        for i, (t, cmd) in enumerate([("添加", self._obs_add), ("编辑", self._obs_edit),
                                      ("删除", self._obs_del), ("清空", self._obs_clear)]):
            ttk.Button(btns, text=t, width=6, command=cmd).grid(
                row=i // 4, column=i % 4, padx=1, pady=1)
        for i, (t, cmd) in enumerate([("示例", self._obs_demo),
                                      ("导入JSON", self._obs_import),
                                      ("导出JSON", self._obs_export)]):
            ttk.Button(btns, text=t, width=7, command=cmd).grid(
                row=1, column=i, padx=1, pady=1)

        # 参数
        f = ttk.LabelFrame(lf, text="规划参数", padding=6)
        f.pack(fill=tk.X)
        self.p_head = tk.StringVar()
        self.p_ghead = tk.StringVar()
        self.p_turn = tk.StringVar(value="30")
        self.p_safety = tk.StringVar(value="0")
        self.p_seed = tk.StringVar(value="42")
        for r, (lbl, var, w) in enumerate([("初始航向°(空=自动)", self.p_head, 16),
                                           ("末端航向°(空=自动)", self.p_ghead, 16),
                                           ("最小转弯半径m", self.p_turn, 16),
                                           ("安全余量m", self.p_safety, 16),
                                           ("随机种子", self.p_seed, 16)]):
            ttk.Label(f, text=lbl).grid(row=r, column=0, sticky="w", pady=1)
            ttk.Entry(f, textvariable=var, width=w).grid(row=r, column=1, sticky="ew", pady=1)
        f.columnconfigure(1, weight=1)

    # -------- 中间：Matplotlib 画布 --------
    def _build_center(self, cf):
        self._cf = cf
        self.fig = Figure(figsize=(7, 6), dpi=100, facecolor="white")
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=cf)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.draw()
        self.toolbar = NavigationToolbar2Tk(self.canvas, cf)
        self.toolbar.update()
        self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.mpl_connect("button_press_event", self._on_mpl_click)

    # -------- 指标表(整宽, 画布下方) --------
    def _build_table(self, tf):
        ttk.Label(tf, text="规划结果指标（先按可行性排序，评分越接近 1 越接近『平滑直线』；"
                           "单击高亮，双击直接飞行回放）",
                  font=("Microsoft YaHei", 9, "bold")).pack(anchor="w", pady=(4, 2))
        box = ttk.Frame(tf)
        box.pack(fill=tk.X)
        cols = ("rank", "label", "status", "length", "str", "sm", "excess",
                "rmin", "clear", "ms", "score")
        heads = ("排名", "算法", "状态", "长度m", "直线度", "平滑度", "多余转弯°",
                 "最小转弯m", "最近障碍m", "耗时ms", "评分")
        widths = (46, 168, 54, 78, 66, 66, 84, 86, 92, 66, 72)
        self.tree = ttk.Treeview(box, columns=cols, show="headings", height=7)
        for c, h, w in zip(cols, heads, widths):
            self.tree.heading(c, text=h,
                              command=lambda cc=c: self._sort_tree(cc))
            self.tree.column(c, width=w, anchor="center")
        vsb = ttk.Scrollbar(box, orient=tk.VERTICAL, command=self.tree.yview)
        hsb = ttk.Scrollbar(box, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(row=0, column=0, sticky="ew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        box.columnconfigure(0, weight=1)
        self.tree.bind("<Button-1>", self._on_tree_click)
        self.tree.bind("<Double-1>", self._on_tree_dblclick)

    # ============================================================ 场景
    def _make_frame(self):
        pts = [tuple(self.start), tuple(self.goal)] + \
              [(o.lat, o.lon) for o in self.obstacles]
        self.frame = LocalFrame.from_points(pts)
        for o in self.obstacles:
            o.bind_frame(self.frame)
        return self.frame

    def rebuild_scene(self):
        frame = self._make_frame()
        ax = self.ax
        ax.clear()
        self._path_lines = {}
        self._ac = []
        self._trail = None

        for i, ob in enumerate(self.obstacles):
            ax.add_patch(Circle(ob.xy, ob.radius_m, fc="#ffcccc", ec="#cc0000",
                                lw=1.3, alpha=0.8, zorder=2,
                                label="禁飞区" if i == 0 else "_nolegend_"))
            if ob.label:
                ax.text(ob.xy[0], ob.xy[1], ob.label, ha="center", va="center",
                        fontsize=8, color="#990000", zorder=3)

        sx, sy = frame.to_xy(*self.start)
        gx, gy = frame.to_xy(*self.goal)
        ax.plot([sx, gx], [sy, gy], color="#999999", lw=1.2, ls="--",
                zorder=1, label="直线基准")
        ax.plot(sx, sy, "s", color="#111111", ms=9, zorder=6, label="起点")
        ax.plot(gx, gy, "*", color="gold", ms=17, mec="black", mew=1.2,
                zorder=6, label="目标")

        # 视野
        pts = np.array([[sx, sy], [gx, gy]] +
                       [list(o.xy) for o in self.obstacles])
        lo, hi = pts.min(0), pts.max(0)
        for o in self.obstacles:
            lo = np.minimum(lo, o.center - o.radius_m)
            hi = np.maximum(hi, o.center + o.radius_m)
        c = (lo + hi) / 2
        r = float((hi - lo).max()) / 2 * 1.18 + 30
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)

        ax.set_aspect("equal")
        ax.grid(True, ls=":", alpha=0.5)
        ax.set_xlabel("东向 x (m)")
        ax.set_ylabel("北向 y (m)")
        ax.set_title("路径规划仿真 (局部坐标, 北向上)")
        ax.legend(loc="best", fontsize=8, ncol=2, framealpha=0.9)
        self.canvas.draw_idle()

    def _scene_changed(self):
        self._stop_anim()
        self.results = {}
        self.metrics = {}
        self.metrics_sorted = []
        self.ordered = []
        self._clear_tree()
        self._fill_flight_combo([])
        self.v_rank.set("")
        self.rebuild_scene()
        self._set_progress(0, "待机")

    # ============================================================ 输入
    def _read_gps(self):
        try:
            s = (float(self.v_slat.get()), float(self.v_slon.get()))
            g = (float(self.v_glat.get()), float(self.v_glon.get()))
            for la, lo in (s, g):
                if not (-90.0 <= la <= 90.0 and -180.0 <= lo <= 180.0):
                    raise ValueError("纬度需在 [-90,90]、经度需在 [-180,180]")
            return s, g
        except Exception as e:
            self._set_status(f"坐标无效: {e}")
            return None

    def _apply_gps(self):
        gp = self._read_gps()
        if gp is None:
            return False
        s, g = gp
        if list(s) == self.start and list(g) == self.goal:
            return True
        self.start, self.goal = list(s), list(g)
        d = math.dist(s, g)
        self._scene_changed()
        self._set_status(f"已更新起终点，直线距离 {d:,.1f} m")
        return True

    # ============================================================ 禁飞区
    def _refresh_obs_tree(self):
        t = self.obs_tree
        t.delete(*t.get_children())
        for i, o in enumerate(self.obstacles):
            t.insert("", "end", iid=str(i),
                     values=(o.label or f"禁飞{i + 1}", f"{o.lat:.6f}",
                             f"{o.lon:.6f}", f"{o.radius_m:.0f}"))

    def _obs_selected(self):
        sel = self.obs_tree.selection()
        return int(sel[0]) if sel else None

    def _obs_add(self):
        d = self._obstacle_dialog(None)
        if d:
            self.obstacles.append(Obstacle(**d))
            self._refresh_obs_tree()
            self._scene_changed()

    def _obs_edit(self):
        i = self._obs_selected()
        if i is None:
            self._set_status("请先在禁飞区列表中选中一行")
            return
        o = self.obstacles[i]
        d = self._obstacle_dialog(dict(label=o.label, lat=o.lat, lon=o.lon,
                                       radius_m=o.radius_m))
        if d:
            self.obstacles[i] = Obstacle(**d)
            self._refresh_obs_tree()
            self._scene_changed()

    def _on_obs_dblclick(self, _):
        self._obs_edit()

    def _obs_del(self):
        i = self._obs_selected()
        if i is None:
            return
        del self.obstacles[i]
        self._refresh_obs_tree()
        self._scene_changed()

    def _obs_clear(self):
        if self.obstacles and not messagebox.askyesno("清空", "删除全部禁飞区?"):
            return
        self.obstacles = []
        self._refresh_obs_tree()
        self._scene_changed()

    def _obs_demo(self):
        self._load_demo_obstacles()
        self._scene_changed()
        self._set_status("已载入示例禁飞区(压在直连线上)")

    def _load_demo_obstacles(self, silent=False):
        fr = LocalFrame.from_points([tuple(self.start), tuple(self.goal)])
        self.obstacles = demo_obstacles(tuple(self.start), tuple(self.goal), fr)
        self._refresh_obs_tree()

    def _obs_import(self):
        p = filedialog.askopenfilename(
            title="导入禁飞区 JSON", filetypes=[("JSON", "*.json"), ("全部", "*.*")])
        if not p:
            return
        try:
            with open(p, encoding="utf-8") as f:
                data = json.load(f)
            obs = [Obstacle(lat=float(d["lat"]), lon=float(d["lon"]),
                            radius_m=float(d["radius_m"]),
                            label=d.get("label", f"禁飞{i + 1}"))
                   for i, d in enumerate(data)]
            self.obstacles = obs
            self._refresh_obs_tree()
            self._scene_changed()
            self._set_status(f"已导入 {len(obs)} 个禁飞区: {p}")
        except Exception as e:
            messagebox.showerror("导入失败", str(e))

    def _obs_export(self):
        if not self.obstacles:
            messagebox.showinfo("导出", "当前没有禁飞区")
            return
        p = filedialog.asksaveasfilename(title="导出禁飞区 JSON",
                                         defaultextension=".json",
                                         filetypes=[("JSON", "*.json")])
        if not p:
            return
        data = [{"lat": o.lat, "lon": o.lon, "radius_m": o.radius_m,
                 "label": o.label} for o in self.obstacles]
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self._set_status(f"已导出 {len(data)} 个禁飞区: {p}")

    def _obstacle_dialog(self, initial):
        dlg = tk.Toplevel(self)
        dlg.title("禁飞区" if initial else "添加禁飞区")
        dlg.transient(self)
        dlg.grab_set()
        dlg.resizable(False, False)
        frm = ttk.Frame(dlg, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)
        vars_ = {}
        fields = [("label", "名称", str, initial.get("label") if initial else f"禁飞{len(self.obstacles) + 1}"),
                  ("lat", "纬度", float, initial.get("lat") if initial else self.start[0]),
                  ("lon", "经度", float, initial.get("lon") if initial else self.start[1]),
                  ("radius_m", "半径 (米)", float, initial.get("radius_m") if initial else 200.0)]
        for r, (key, lbl, typ, val) in enumerate(fields):
            ttk.Label(frm, text=lbl, width=10).grid(row=r, column=0, sticky="w", pady=3)
            v = tk.StringVar(value=str(val))
            vars_[key] = (v, typ)
            ttk.Entry(frm, textvariable=v, width=22).grid(row=r, column=1, sticky="ew", pady=3)
        out = {}

        def ok():
            try:
                d = {}
                for k, (v, typ) in vars_.items():
                    s = v.get().strip()
                    d[k] = s if typ is str and k == "label" else float(s)
                if not (-90 <= d["lat"] <= 90 and -180 <= d["lon"] <= 180):
                    raise ValueError("经纬度超出范围")
                if d["radius_m"] <= 0:
                    raise ValueError("半径必须 > 0")
                if not d["label"]:
                    raise ValueError("名称不能为空")
                out.update(d)
            except Exception as e:
                messagebox.showerror("输入无效", str(e), parent=dlg)
                return
            dlg.destroy()

        def cancel():
            out.clear()
            dlg.destroy()

        bf = ttk.Frame(frm)
        bf.grid(row=len(fields), column=0, columnspan=2, pady=(10, 0))
        ttk.Button(bf, text="确定", command=ok).pack(side=tk.LEFT, padx=4)
        ttk.Button(bf, text="取消", command=cancel).pack(side=tk.LEFT, padx=4)
        dlg.bind("<Return>", lambda e: ok())
        dlg.bind("<Escape>", lambda e: cancel())
        dlg.wait_window()
        return out or None

    # ============================================================ 运行
    def run(self):
        if self._running:
            return
        gp = self._read_gps()
        if gp is None:
            return
        s, g = gp
        self.start, self.goal = list(s), list(g)

        algo_list = [n for n, v in self.algo_vars.items() if v.get()]
        if not algo_list:
            messagebox.showinfo("运行", "请至少勾选一个算法")
            return
        try:
            turn_r = float(self.p_turn.get() or 30)
            safety = float(self.p_safety.get() or 0)
            seed = int(float(self.p_seed.get()))
            hs, hg = self.p_head.get().strip(), self.p_ghead.get().strip()
            start_heading = float(hs) if hs else None
            goal_heading = float(hg) if hg else None
        except Exception as e:
            messagebox.showerror("参数无效", str(e))
            return

        self._stop_anim()
        self.results, self.metrics, self.metrics_sorted, self.ordered = {}, {}, [], []
        self._clear_tree()
        self._fill_flight_combo([])
        self.v_rank.set("")
        self.rebuild_scene()

        frame = self.frame
        obstacles = list(self.obstacles)
        start_xy = frame.to_xy(*self.start)
        goal_xy = frame.to_xy(*self.goal)

        self._running = True
        self._stop_evt.clear()
        self.btn_run.state(["disabled"])
        self._set_progress(0, "规划中")
        self._set_status(f"开始规划：{len(algo_list)} 个算法 × {len(obstacles)} 个禁飞区 …")
        self._q = queue.Queue()
        threading.Thread(target=self._worker,
                         args=(frame, obstacles, start_xy, goal_xy, algo_list,
                               turn_r, safety, seed, start_heading, goal_heading),
                         daemon=True).start()
        self.after(60, self._poll)

    def _worker(self, frame, obstacles, start_xy, goal_xy, algo_list,
                turn_r, safety, seed, start_heading, goal_heading):
        try:
            results = {}
            n = len(algo_list)
            for i, name in enumerate(algo_list):
                if self._stop_evt.is_set():
                    self._q.put(("cancelled", None))
                    return
                cls = REGISTRY[name]
                params = {"radius": turn_r} if cls.need_heading else {}
                planner = cls(frame, obstacles, safety_m=safety, seed=seed, **params)
                res = planner.plan(start_xy, goal_xy,
                                   start_heading=start_heading,
                                   goal_heading=goal_heading)
                results[name] = res
                self._q.put(("one", (i + 1, n, name, res)))
            self._q.put(("done", results))
        except Exception:
            self._q.put(("error", traceback.format_exc()))

    def _poll(self):
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if kind == "one":
                    i, n, name, res = payload
                    self.results[name] = res
                    note = f"  [{res.message}]" if res.message else ""
                    self._set_status(
                        f"✓ [{i}/{n}] {res.label}  {res.compute_ms:.0f} ms  "
                        f"{STATUS_CN.get(res.status, res.status)}{note}")
                elif kind == "done":
                    self._on_planned(payload)
                elif kind == "error":
                    messagebox.showerror("规划异常", payload)
                    self._finish_run()
                    return
                elif kind == "cancelled":
                    self._finish_run()
                    return
        except queue.Empty:
            pass
        if self._running:
            self.after(60, self._poll)

    def _finish_run(self):
        self._running = False
        self.btn_run.state(["!disabled"])

    def _on_planned(self, results):
        self._finish_run()
        self.results = results
        metrics = {}
        for name, res in results.items():
            m = evaluate(res.points, self.obstacles)
            metrics[name] = {"label": res.label, "algo": name, **m,
                             "status": res.status, "message": res.message,
                             "compute_ms": res.compute_ms}
        self.metrics = metrics
        self.ordered = rank(metrics)
        self.metrics_sorted = [metrics[k] for k, _ in self.ordered]
        self._fill_tree()
        self._fill_flight_combo([k for k, _ in self.ordered])
        best = self.metrics_sorted[0]
        self.v_rank.set(f"最趋向『平滑直线』: {best['label']}  "
                        f"评分 {best['score']:.4f}")
        self._init_path_artists()
        self._start_growth()

    # ============================================================ 路径画笔
    def _init_path_artists(self):
        for ln in self._path_lines.values():
            ln.remove()
        self._path_lines = {}
        for name, res in self.results.items():
            ln, = self.ax.plot([], [], color=self._colors[name], lw=2.0,
                               zorder=5, solid_capstyle="round", label=res.label)
            self._path_lines[name] = ln
        self.ax.legend(loc="best", fontsize=8, ncol=2, framealpha=0.9)
        self.canvas.draw_idle()

    @staticmethod
    def _decimate(pts, n=700):
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(pts) <= n:
            return pts
        idx = np.linspace(0, len(pts) - 1, n).astype(int)
        return pts[idx]

    # ============================================================ 动画
    def _schedule(self, ms=None):
        ms = max(15, int(ms if ms is not None else self.GROW_TICK_MS))
        if self._job is not None:
            try:
                self.after_cancel(self._job)
            except Exception:
                pass
        self._job = self.after(ms, self._loop)

    def _loop(self):
        self._job = None
        if self._anim == "growth":
            self._growth_step()
        elif self._anim == "flight":
            self._flight_step()

    def _stop_anim(self):
        if self._job is not None:
            try:
                self.after_cancel(self._job)
            except Exception:
                pass
            self._job = None
        self._anim = "idle"
        self._paused = False
        self._clear_aircraft()
        if self._trail is not None:
            try:
                self._trail.remove()
            except Exception:
                pass
            self._trail = None
        for ln in self._path_lines.values():
            ln.set_alpha(1.0)
            ln.set_linewidth(2.0)
        self.btn_pause.configure(text="⏸ 暂停")

    def _start_growth(self):
        if not self.results or not self._path_lines:
            return
        order = [k for k, _ in self.ordered if k in self.results] or list(self.results)
        self._grow_order = order
        self._grow_pts = {k: self._decimate(self.results[k].points, 700)
                          for k in order}
        self._gi, self._gacc = 0, 0.0
        self._paused = False
        self._anim = "growth"
        for ln in self._path_lines.values():
            ln.set_data([], [])
            ln.set_alpha(1.0)
            ln.set_linewidth(2.0)
        self.btn_pause.configure(text="⏸ 暂停")
        self._set_progress(0, "路径生成")
        self._set_status("路径生成动画 …")
        self.canvas.draw_idle()
        self._schedule()

    def _growth_step(self):
        if self._paused:
            self._schedule()
            return
        if self._gi >= len(self._grow_order):
            self._growth_done()
            return

        key = self._grow_order[self._gi]
        pts = self._grow_pts[key]
        sp = max(0.05, float(self.speed.get()))
        self._gacc += len(pts) / float(self.GROW_STEPS) * sp
        k = int(self._gacc)
        finished = k >= len(pts)
        if finished:
            k = len(pts)

        for i, kk in enumerate(self._grow_order):
            if i < self._gi:
                p = self._grow_pts[kk]
            elif i == self._gi:
                p = pts[:k]
            else:
                p = np.empty((0, 2))
            ln = self._path_lines[kk]
            ln.set_data(p[:, 0], p[:, 1])
            ln.set_linewidth(3.2 if i == self._gi else 2.0)

        done = sum(len(self._grow_pts[kk]) for kk in self._grow_order[:self._gi])
        if not finished:
            done += k
        total = sum(len(v) for v in self._grow_pts.values())
        self._set_progress(100.0 * done / max(1, total), "路径生成")
        # 显式绘制: 动画帧用 draw_idle() 时, 空闲回调可能被连续到期的定时器饿死,
        # 导致整段动画攒到最后一次性画出(用户看不到过程)。
        self.canvas.draw()

        if finished:
            self._gi += 1
            self._gacc = 0.0
        if self._gi >= len(self._grow_order):
            self._growth_done()
            return
        self._schedule()

    def _growth_done(self):
        for name, res in self.results.items():
            p = np.asarray(res.points, dtype=float)
            self._path_lines[name].set_data(p[:, 0], p[:, 1])
            self._path_lines[name].set_linewidth(2.0)
        self._set_progress(100, "路径生成")
        self.canvas.draw_idle()
        self._anim = "ready"
        n = len(self.results)
        self._set_status(f"路径生成完成（{n} 条）")
        if self.auto_fly.get():
            self.after(400, self._auto_fly_cb)

    def _auto_fly_cb(self):
        if self._anim == "ready":
            self._start_flight()

    def _start_flight(self, algo=None):
        if not self.results:
            return
        if self._anim not in ("ready", "done", "idle", "flight"):
            return
        if algo is None:
            algo = self._label2algo.get(self.flight_var.get())
        if algo not in self.results:
            algo = next(iter(self.results))

        self._anim = "flight"
        self._paused = False
        self._f_algo = algo
        pts = np.asarray(self.results[algo].points, dtype=float)
        if len(pts) < 2:
            self._anim = "done"
            return
        self._f_pts = pts
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self._f_cum = np.concatenate([[0.0], np.cumsum(seg)])
        self._f_len = float(self._f_cum[-1])
        if self._f_len <= 1e-9:
            self._anim = "done"
            return
        self._f_s = 0.0

        if self._trail is not None:
            try:
                self._trail.remove()
            except Exception:
                pass
        self._trail, = self.ax.plot(
            [], [], color=self._colors[algo], lw=3.6, alpha=0.95, zorder=6,
            solid_capstyle="round")
        self._make_aircraft()

        for k, ln in self._path_lines.items():
            ln.set_alpha(1.0 if k == algo else 0.32)
            ln.set_linewidth(3.0 if k == algo else 1.4)
        self.ax.legend(loc="best", fontsize=8, ncol=2, framealpha=0.9)
        self.btn_pause.configure(text="⏸ 暂停")
        self._set_progress(0, "飞行仿真")
        self._set_status(f"飞行仿真: {self.results[algo].label}  "
                         f"航程 {self._f_len:,.0f} m")
        self.canvas.draw_idle()
        self._schedule(self.FLIGHT_TICK_MS)

    def _flight_step(self):
        if self._paused:
            self._schedule(self.FLIGHT_TICK_MS)
            return
        sp = max(0.05, float(self.speed.get()))
        v = self._f_len / self.FLIGHT_BASE_SEC * sp
        self._f_s += v * self.FLIGHT_TICK_MS / 1000.0
        finished = self._f_s >= self._f_len
        if finished:
            self._f_s = self._f_len

        i = int(np.searchsorted(self._f_cum, self._f_s, side="right") - 1)
        i = max(0, min(i, len(self._f_pts) - 2))
        s0, s1 = self._f_cum[i], self._f_cum[i + 1]
        t = 0.0 if s1 <= s0 else (self._f_s - s0) / (s1 - s0)
        p = self._f_pts[i] + t * (self._f_pts[i + 1] - self._f_pts[i])
        d = self._f_pts[i + 1] - self._f_pts[i]
        theta = math.atan2(float(d[1]), float(d[0]))

        trail = np.vstack([self._f_pts[:i + 1], p])
        self._trail.set_data(trail[:, 0], trail[:, 1])
        self._place_aircraft(p, theta)

        lat, lon = self.frame.to_latlon(float(p[0]), float(p[1]))
        hdg = math.degrees(theta) % 360.0
        self._set_progress(100.0 * self._f_s / self._f_len, "飞行仿真")
        self._set_status(
            f"✈ {self.results[self._f_algo].label}   "
            f"已飞 {self._f_s:,.0f} / {self._f_len:,.0f} m   "
            f"航向 {hdg:5.1f}°   GPS {lat:.6f}, {lon:.6f}")
        self.canvas.draw()

        if finished:
            self._anim = "done"
            self._set_progress(100, "完成")
            self._set_status(f"仿真完成 —— {self.results[self._f_algo].label}")
            return
        self._schedule(self.FLIGHT_TICK_MS)

    # -------- 四旋翼图形 --------
    def _make_aircraft(self):
        self._clear_aircraft()
        col = self._colors.get(self._f_algo, "#111111")
        ax = self.ax
        parts = []
        # 机臂(对角十字)
        parts.append(ax.add_line(Line2D([-0.8, 0.8], [-0.8, 0.8], color="#333333",
                                        lw=2.4, zorder=13)))
        parts.append(ax.add_line(Line2D([-0.8, 0.8], [0.8, -0.8], color="#333333",
                                        lw=2.4, zorder=13)))
        # 旋翼
        for dx, dy in ((-0.8, -0.8), (-0.8, 0.8), (0.8, -0.8), (0.8, 0.8)):
            parts.append(ax.add_patch(
                Circle((dx, dy), 0.34, fc="none", ec=col, lw=1.5, alpha=0.9,
                       zorder=13)))
        # 机身(机头朝 +x)
        parts.append(ax.add_patch(Polygon(
            [(-1.25, -0.34), (0.55, -0.34), (1.05, 0.0), (0.55, 0.34),
             (-1.25, 0.34), (-1.55, 0.0)],
            closed=True, fc="#1f1f1f", ec="white", lw=1.1, zorder=15)))
        parts.append(ax.add_patch(Polygon(
            [(0.5, -0.2), (1.15, 0.0), (0.5, 0.2)],
            closed=True, fc=col, ec="none", zorder=16)))
        self._ac = parts

    def _clear_aircraft(self):
        for a in self._ac:
            try:
                a.remove()
            except Exception:
                pass
        self._ac = []

    def _place_aircraft(self, p, theta):
        if not self._ac:
            return
        x0, x1 = self.ax.get_xlim()
        sc = (x1 - x0) * 0.016
        tr = (Affine2D().scale(sc).rotate(theta)
              .translate(float(p[0]), float(p[1])) + self.ax.transData)
        for a in self._ac:
            a.set_transform(tr)

    # -------- 动画按钮 --------
    def _on_play(self):
        if not self.results:
            self._set_status("尚无规划结果，请先点击「运行规划」")
            return
        self._start_growth()

    def _toggle_pause(self):
        if self._anim not in ("growth", "flight"):
            return
        self._paused = not self._paused
        self.btn_pause.configure(text="▶ 继续" if self._paused else "⏸ 暂停")
        self._set_status("已暂停" if self._paused else "继续仿真")

    def _on_stop(self):
        self._stop_evt.set()
        self._stop_anim()
        for name, res in self.results.items():
            if name in self._path_lines:
                p = np.asarray(res.points, dtype=float)
                self._path_lines[name].set_data(p[:, 0], p[:, 1])
                self._path_lines[name].set_alpha(1.0)
                self._path_lines[name].set_linewidth(2.0)
        if self.results:
            self.ax.legend(loc="best", fontsize=8, ncol=2, framealpha=0.9)
        self._set_progress(0, "待机")
        self._set_status("已停止动画，显示全部完整路径")
        self.canvas.draw_idle()

    # ============================================================ 指标表
    def _clear_tree(self):
        self.tree.delete(*self.tree.get_children())
        self._rows = []

    def _fill_tree(self):
        self._clear_tree()
        for i, m in enumerate(self.metrics_sorted, 1):
            st = m.get("status", "ok")
            rmin = m.get("min_turn_radius_m", float("inf"))
            clr = m.get("clearance_m", float("inf"))
            clear = ("—" if not self.obstacles or math.isinf(clr) else f"{clr:.1f}")
            raw = dict(rank=i, label=m["label"], status=STATUS_CN.get(st, st),
                       length=m["length_m"], str=m["straightness"],
                       sm=m["smoothness"], excess=m["excess_turn_deg"],
                       rmin=(1e18 if math.isinf(rmin) else rmin),
                       clear=(1e18 if math.isinf(clr) else clr),
                       ms=m["compute_ms"], score=m["score"])
            disp = (i, m["label"], STATUS_CN.get(st, st),
                    f"{m['length_m']:.1f}", f"{m['straightness']:.4f}",
                    f"{m['smoothness']:.4f}", f"{m['excess_turn_deg']:.1f}",
                    ("∞" if math.isinf(rmin) else f"{rmin:.1f}"),
                    clear, f"{m['compute_ms']:.0f}", f"{m['score']:.4f}")
            iid = m["algo"]
            self.tree.insert("", "end", iid=iid, values=disp, tags=(st,))
            self._rows.append((iid, raw, disp, st))
        for tag, bg in (("ok", "#e9f7ef"), ("warn", "#fff4e5"),
                        ("failed", "#fdecea"), ("partial", "#fff4e5")):
            self.tree.tag_configure(tag, background=bg)

    def _sort_tree(self, col):
        if not self._rows:
            return
        rev = self._sort_rev.get(col, False)
        rows = sorted(self._rows, key=lambda r: r[1][col], reverse=rev)
        self._sort_rev[col] = not rev
        self.tree.delete(*self.tree.get_children())
        for n, (iid, raw, disp, st) in enumerate(rows, 1):
            v = list(disp)
            if col == "rank":
                v[0] = n
            self.tree.insert("", "end", iid=iid, values=v, tags=(st,))

    def _on_tree_click(self, event):
        iid = self.tree.identify_row(event.y)
        if not iid or iid not in self.results:
            return
        label = self.results[iid].label
        if label in self._label2algo:
            self.flight_var.set(label)
        self._highlight(iid)

    def _on_tree_dblclick(self, event):
        iid = self.tree.identify_row(event.y)
        if iid and iid in self.results:
            self.flight_var.set(self.results[iid].label)
            self._start_flight(iid)

    def _highlight(self, algo):
        if not self._path_lines or self._anim in ("growth", "flight"):
            return
        for k, ln in self._path_lines.items():
            ln.set_alpha(1.0 if k == algo else 0.3)
            ln.set_linewidth(3.0 if k == algo else 1.5)
        self.canvas.draw_idle()

    def _fill_flight_combo(self, algo_order):
        labels = [self.results[k].label for k in algo_order if k in self.results]
        self._label2algo = {self.results[k].label: k
                            for k in algo_order if k in self.results}
        self.flight_combo.configure(values=labels)
        if labels:
            self.flight_var.set(labels[0])
        else:
            self.flight_var.set("")

    # ============================================================ 画布点击
    def _on_mpl_click(self, ev):
        if ev.inaxes != self.ax or ev.xdata is None:
            return
        mode = self.click_mode.get()
        if mode == "选择":
            return
        lat, lon = self.frame.to_latlon(float(ev.xdata), float(ev.ydata))
        if mode == "设为起点":
            self.v_slat.set(f"{lat:.6f}"); self.v_slon.set(f"{lon:.6f}")
            self._apply_gps()
        elif mode == "设为目标":
            self.v_glat.set(f"{lat:.6f}"); self.v_glon.set(f"{lon:.6f}")
            self._apply_gps()
        elif mode == "添加禁飞区":
            d = self._obstacle_dialog(dict(label=f"禁飞{len(self.obstacles) + 1}",
                                           lat=lat, lon=lon, radius_m=200.0))
            if d:
                self.obstacles.append(Obstacle(**d))
                self._refresh_obs_tree()
                self._scene_changed()
                self._set_status(f"已添加 {d['label']} (r={d['radius_m']:.0f} m)")

    # ============================================================ 导出
    def _on_export(self):
        if not self.results:
            messagebox.showinfo("导出", "尚无规划结果，请先运行规划")
            return
        d = filedialog.askdirectory(title="选择导出目录", initialdir=os.getcwd())
        if not d:
            return
        try:
            config = {
                "start": list(self.start), "goal": list(self.goal),
                "safety_m": float(self.p_safety.get() or 0),
                "seed": int(float(self.p_seed.get())),
                "turn_radius_m": float(self.p_turn.get() or 30),
                "algos": list(self.results),
                "obstacles": [{"lat": o.lat, "lon": o.lon, "radius_m": o.radius_m,
                               "label": o.label} for o in self.obstacles],
            }
            save_outputs(d, self.results, self.metrics_sorted, self.frame,
                         self.obstacles, config)
            sx, sy = self.frame.to_xy(*self.start)
            gx, gy = self.frame.to_xy(*self.goal)
            res = {m["algo"]: self.results[m["algo"]] for m in self.metrics_sorted}
            viz.plot_paths(res, self.obstacles, (sx, sy), (gx, gy),
                           os.path.join(d, "paths.png"))
            viz.plot_metrics(self.metrics_sorted, os.path.join(d, "metrics.png"))
            viz.plot_detail(res, self.obstacles, (sx, sy), (gx, gy),
                            os.path.join(d, "detail.png"))
            messagebox.showinfo("导出完成",
                                f"GPS 航点 / 指标 / JSON / PNG 已保存到:\n{d}")
            self._set_status(f"已导出到 {d}")
        except Exception as e:
            messagebox.showerror("导出失败", f"{e}\n\n{traceback.format_exc()}")

    # ============================================================ 帮助
    def _on_help(self):
        dlg = tk.Toplevel(self)
        dlg.title("算法说明与指标定义")
        dlg.geometry("760x680")
        dlg.transient(self)
        txt = ScrolledText(dlg, wrap=tk.WORD, font=("Microsoft YaHei", 10))
        txt.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        txt.tag_configure("h", font=("Microsoft YaHei", 11, "bold"),
                          foreground="#0a58ca", spacing1=6, spacing3=2)
        txt.tag_configure("m", background="#f2f7ff", lmargin1=10, lmargin2=10,
                          spacing1=3, spacing3=8)

        def head(s):
            txt.insert(tk.END, s + "\n", "h")

        def body(s):
            txt.insert(tk.END, s.strip() + "\n\n", "m")

        head("一、八种路径规划算法")
        for _, (name, label) in enumerate(available()):
            head(f"{_ + 1}. {label}   (key: {name})")
            body(ALGO_HELP.get(name, "暂无说明"))
        head("二、指标定义与排序规则")
        body(METRIC_HELP)
        txt.configure(state=tk.DISABLED)

    # ============================================================ 杂项
    def _set_status(self, text):
        self.status_var.set(text)

    def _on_space(self, _event=None):
        # 焦点在输入框时不劫持空格
        w = self.focus_get()
        if isinstance(w, (tk.Entry, ttk.Entry, tk.Text, ScrolledText)):
            return
        self._toggle_pause()

    def _on_wheel(self, event):
        """鼠标滚轮: 仅当指针落在左侧设置面板内时滚动它。"""
        cv = getattr(self, "_left_canvas", None)
        if cv is None:
            return
        w = event.widget
        while w is not None:
            if w is self._lf:
                delta = event.delta or 0
                if not delta and getattr(event, "num", None) in (4, 5):
                    delta = 120 if event.num == 4 else -120
                if delta:
                    cv.yview_scroll(-int(delta / 120), "units")
                return
            w = getattr(w, "master", None)

    def _set_progress(self, pct, phase=None):
        self.progress["value"] = max(0.0, min(100.0, float(pct)))
        if phase:
            self.phase_var.set(phase)

    def _on_close(self):
        self._stop_evt.set()
        if self._job is not None:
            try:
                self.after_cancel(self._job)
            except Exception:
                pass
        self.destroy()


def main():
    app = FlightGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
