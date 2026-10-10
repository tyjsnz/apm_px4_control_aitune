# -*- coding: utf-8 -*-
"""资料目录插图生成脚本(纯 QPainter 离屏绘制, 不依赖 matplotlib).

用法:
    cd guidance_sim/references
    python make_figs.py        # 重新生成 figures/ 下全部 PNG

图形与对应文档:
    fig01_phases.png         01 全流程总览      交战时间轴与信息流
    fig02_collision.png      06 中段与拦截点    碰撞三角形与前置角
    fig03_acquisition.png    06 中段与拦截点    截获窗口(R_acq + FOV)
    fig04_navigation.png     05 组合导航        组合导航/修正回路框图
    fig05_guidance_loops.png 02 地面引导        指令/半主动/主动三种回路
    fig06_pn_pp.png          08 经典导引律      本项目实测 PN vs PP 弹道与过载
    fig07_layers.png         03 路径规划        四层概念分层
    fig08_zem.png            06 中段与拦截点    ZEM(零 effort 脱靶量)概念
    fig09_miss.png           10 工程实践        脱靶量 / CEP / 杀伤半径
"""
from __future__ import annotations

import math
import os
import sys

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QApplication

# ------------------------------------------------------------------ 样式 ---
FONT = "Microsoft YaHei"
BG     = "#fbfcfe"
INK    = "#12233d"
SUB    = "#57606a"
LINE   = "#c9d1d9"
BLUE   = "#1f6feb"
RED    = "#d1242f"
GREEN  = "#1a7f37"
AMBER  = "#bf8700"
PURPLE = "#8250df"
TEAL   = "#0d7d8d"
ORANGE = "#e8660c"
PINK   = "#d6338c"

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(HERE, "figures")


# ------------------------------------------------------------------ 工具 ---
def canvas(w: int, h: int):
    img = QImage(w, h, QImage.Format_ARGB32)
    img.fill(QColor(BG))
    p = QPainter(img)
    p.setRenderHint(QPainter.Antialiasing, True)
    p.setRenderHint(QPainter.TextAntialiasing, True)
    return img, p


def txt(p, x, y, s, size=12, color=INK, bold=False, align=None, w=None, h=None):
    """写文字. 指定 w/h 时用矩形居中(支持换行), 否则用点(基线)."""
    f = QFont(FONT, int(round(size)))
    f.setBold(bold)
    p.setFont(f)
    p.setPen(QColor(color))
    if w is None:
        p.drawText(QPointF(x, y), s)
    else:
        flags = Qt.AlignCenter | Qt.TextWordWrap if align is None \
            else (align | Qt.TextWordWrap)
        p.drawText(QRectF(x, y, w, h), flags, s)


def box(p, x, y, w, h, s, border=BLUE, fill="#ffffff", tcolor=INK,
        size=11, bold=False, radius=7, dash=False, lw=1.4):
    pen = QPen(QColor(border), lw)
    if dash:
        pen.setStyle(Qt.DashLine)
    p.setPen(pen)
    p.setBrush(QColor(fill))
    p.drawRoundedRect(QRectF(x, y, w, h), radius, radius)
    if s:
        txt(p, x + 5, y + 4, s, size=size, color=tcolor, bold=bold,
            w=w - 10, h=h - 8)


def arrow(p, x1, y1, x2, y2, color=INK, lw=1.6, dash=False, head=True,
          hs=10.0):
    pen = QPen(QColor(color), lw)
    if dash:
        pen.setStyle(Qt.DashLine)
    p.setPen(pen)
    p.setBrush(Qt.NoBrush)
    p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
    if head:
        ang = math.atan2(y2 - y1, x2 - x1)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(color))
        p.drawPolygon(QPolygonF([
            QPointF(x2, y2),
            QPointF(x2 - hs * math.cos(ang - 0.45),
                    y2 - hs * math.sin(ang - 0.45)),
            QPointF(x2 - hs * math.cos(ang + 0.45),
                    y2 - hs * math.sin(ang + 0.45))]))


def dot(p, x, y, r=5.0, color=BLUE):
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(color))
    p.drawEllipse(QPointF(x, y), r, r)


def arc(p, cx, cy, r, a0, a1, color=BLUE, lw=1.4, dash=False):
    """按数学角(屏幕坐标系, y 向下)画弧线: 用折线采样, 避免 drawArc 角度约定歧义."""
    pen = QPen(QColor(color), lw)
    if dash:
        pen.setStyle(Qt.DashLine)
    p.setPen(pen)
    p.setBrush(Qt.NoBrush)
    n = 26
    pts = [QPointF(cx + r * math.cos(a0 + (a1 - a0) * i / n),
                   cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    for a, b in zip(pts, pts[1:]):
        p.drawLine(a, b)


def title(p, w, s, sub=None):
    txt(p, 30, 40, s, size=17, bold=True)
    if sub:
        txt(p, 30, 66, sub, size=11, color=SUB)
    p.setPen(QColor(LINE))
    p.drawLine(QPointF(30, 76), QPointF(w - 30, 76))


def save(img, name):
    path = os.path.join(FIG_DIR, name)
    if not img.save(path, "PNG"):
        raise RuntimeError(f"保存失败: {path}")
    print("  ✓", name)


def missile_icon(p, x, y, ang, color=ORANGE, s=13.0):
    """简单导弹箭头, ang 为屏幕角(弧度)."""
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(color))
    tip = QPointF(x + s * math.cos(ang), y + s * math.sin(ang))
    b1 = QPointF(x - s * 0.75 * math.cos(ang) + s * 0.55 * math.sin(ang),
                 y - s * 0.75 * math.sin(ang) - s * 0.55 * math.cos(ang))
    b2 = QPointF(x - s * 0.75 * math.cos(ang) - s * 0.55 * math.sin(ang),
                 y - s * 0.75 * math.sin(ang) + s * 0.55 * math.cos(ang))
    p.drawPolygon(QPolygonF([tip, b1, b2]))


def target_icon(p, x, y, ang, color=TEAL, s=15.0):
    """简易飞机(机身 + 机翼), ang 为屏幕角(弧度)."""
    ca, sa = math.cos(ang), math.sin(ang)
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(color))
    body = [QPointF(x + s * ca, y + s * sa),
            QPointF(x - s * 0.8 * ca + s * 0.25 * sa,
                    y - s * 0.8 * sa - s * 0.25 * ca),
            QPointF(x - s * 0.8 * ca - s * 0.25 * sa,
                    y - s * 0.8 * sa + s * 0.25 * ca)]
    wing = [QPointF(x + s * 0.15 * ca, y + s * 0.15 * sa),
            QPointF(x - s * 0.25 * ca + s * 0.95 * sa,
                    y - s * 0.25 * sa - s * 0.95 * ca),
            QPointF(x - s * 0.55 * ca, y - s * 0.55 * sa),
            QPointF(x - s * 0.25 * ca - s * 0.95 * sa,
                    y - s * 0.25 * sa + s * 0.95 * ca)]
    p.drawPolygon(QPolygonF(wing))
    p.drawPolygon(QPolygonF(body))


# ============================================================ 图 1: 时间轴 ==
def fig01():
    W, H = 1280, 660
    img, p = canvas(W, H)
    title(p, W, "导弹拦截全流程：从发射到命中（时间轴 × 信息流 × 制导律）",
          "上排=信息来源(靠什么知道目标/自己在哪)   柱条=飞行阶段   下排=该阶段执行的算法")

    x0, x1 = 60.0, 1220.0
    total = x1 - x0
    segs = [("发射 / 助推", 0.07, BLUE), ("中段（导引头未截获）", 0.47, GREEN),
            ("截获(交接)", 0.05, AMBER), ("末段（导引头寻的）", 0.37, RED),
            ("命中", 0.04, PURPLE)]
    by, bh = 250.0, 74.0
    seg_times = ["t = 0", "t ≈ 0.1 ~ 6 s", "t_acq", "t_acq → 命中", "t_go"]
    xs, cur = [], x0
    for _, f, _ in segs:
        xs.append(cur)
        cur += total * f
    xs.append(x1)

    for i, (name, f, color) in enumerate(segs):
        x, w = xs[i], xs[i + 1] - xs[i]
        p.setPen(QPen(QColor(color), 1.6))
        c = QColor(color)
        c.setAlpha(46)
        p.setBrush(c)
        p.drawRoundedRect(QRectF(x, by, w, bh), 6, 6)
        txt(p, x + 3, by + 16, name, size=12, color=color, bold=True,
            w=w - 6, h=bh - 20)
        p.setPen(QColor(color))
        p.setFont(QFont(FONT, 10))
        p.drawText(QPointF(x + 4, by + bh + 18), seg_times[i])

    # 顶排: 信息来源
    srcs = [("地面(发射前)\n初始对准 · 位置/航向装订\n发射包线与发射时机解算", 0),
            ("地面/平台雷达 + 数据链(上行)\n周期性发送目标状态与修正指令", 1),
            ("卫星导航 GNSS(北斗/GPS)\n位置速度修正: 误差不随时间累积", 2),
            ("弹上导引头(雷达/红外/电视)\n搜索 → 截获 → 跟踪 → 测 λ̇", 3)]
    for s, i in srcs:
        x = xs[i] if i < 3 else xs[4]
        w = 250
        cx = min(max(xs[i] + (xs[i + 1] - xs[i]) / 2 - w / 2, 40), W - w - 40)
        box(p, cx, 96, w, 96, s, border=SUB, fill="#ffffff", size=10.5)
        arrow(p, cx + w / 2, 196, cx + w / 2, by - 6, color=SUB, dash=True)

    # 底排: 制导律（显式布局, 避免相互重叠）
    lows = [("程序初制导\n(按装订航向飞行)", 60, 230, 100, 10.5),
            ("中段制导: 惯导递推 + 修正\n飞向预测拦截点 PIP / 指令修正", 300, 300, 413, 10.5),
            ("截获判据:\nR ≤ R_acq 且 |λ−ψ| ≤ FOV/2", 620, 230, 715, 10.5),
            ("末段导引律:\nPN / PP / APN / OGL / 落角约束", 870, 240, 959, 10.5),
            ("引信+战斗部\n脱靶量≤杀伤半径", 1120, 150, 1195, 9.5)]
    for s, x, w, ax, fs in lows:
        box(p, x, 372, w, 84, s, border=SUB, fill="#ffffff", size=fs)
        arrow(p, ax, by + bh + 4, ax, 368, color=SUB, dash=True, head=False)

    # 说明
    box(p, 60, 486, 1160, 132,
        "读法(怎么做 / 为什么 / 结果)：\n"
        "① 发射前 —— 地面把“我在哪、朝哪飞、目标在哪”装订进弹上计算机（怎么做）；不装订则中段无从修正（为什么）；\n"
        "    装订与对准误差直接进入截获误差预算（结果：σ 增大 → 截获概率下降）。\n"
        "② 中段 —— 看不见目标，用 惯导+卫星+数据链 保持自身状态并把导弹导向“截获窗口”；本程序中段方式 = pip/straight。\n"
        "③ 截获 —— 一次性交接判据（距离门限 ∧ 视场锥）；未截获则末段无测量输入，通常直接失败。\n"
        "④ 末段 —— 导引头给出 λ、λ̇，执行 PN/PP 等导引律，把脱靶量压到杀伤半径以内。",
        border=LINE, fill="#f6f8fa", size=10.5)
    txt(p, 60, 645, "参考：本目录 01/02/04/06/07/08 章；配套仿真 guidance_sim/docs/05",
        size=10, color=SUB)
    p.end()
    save(img, "fig01_phases.png")


# ==================================================== 图 2: 碰撞三角形 ======
def fig02():
    W, H = 1150, 780
    img, p = canvas(W, H)
    title(p, W, "碰撞三角形与前置角 —— 拦截点(P)是怎么解出来的",
          "已知 M、T 位置与 Vm、Vt，求相遇时刻 TOF 与相遇点 P（所有中段解算的共同几何基础）")

    # 侧向交会几何: 让前置角足够大, 三角形清晰可见
    M = QPointF(200, 600)
    T = QPointF(640, 300)
    P = QPointF(860, 470)
    ta = math.atan2(P.y() - T.y(), P.x() - T.x())      # 目标航向(屏幕角)
    ma = math.atan2(P.y() - M.y(), P.x() - M.x())      # 导弹航向(屏幕角)
    los = math.atan2(T.y() - M.y(), T.x() - M.x())     # 视线角

    # 参考水平线 + 视线 + 两条速度方向
    p.setPen(QPen(QColor(LINE), 1.2, Qt.DashLine))
    p.drawLine(QPointF(M.x(), M.y()), QPointF(M.x() + 520, M.y()))
    txt(p, M.x() + 528, M.y() + 4, "0° 基准", size=10, color=SUB)

    p.setPen(QPen(QColor(TEAL), 2.0, Qt.DashLine))
    p.drawLine(M, T)
    arrow(p, M.x(), M.y(), P.x(), P.y(), color=ORANGE, lw=2.6, hs=13)
    arrow(p, T.x(), T.y(), P.x(), P.y(), color=TEAL, lw=2.6, hs=13)

    txt(p, M.x() + 210, M.y() - 30, "Vm (导弹碰撞方向)", size=12, color=ORANGE,
        bold=True)
    txt(p, T.x() + 96, T.y() + 120, "Vt (目标航向)", size=12, color=TEAL,
        bold=True)
    txt(p, (M.x() + T.x()) / 2 - 96, (M.y() + T.y()) / 2 - 10,
        "R = |P_t − P_m|  (视线 LOS)", size=12, color=TEAL)

    # 角度
    arc(p, M.x(), M.y(), 150, los, 0.0, color=SUB, lw=1.4)
    txt(p, M.x() + 156, M.y() - 24, "λ 视线角", size=11, color=SUB)
    arc(p, M.x(), M.y(), 105, los, ma, color=RED, lw=2.4)
    txt(p, M.x() + 112, M.y() - 44, "φ 前置角 ≈ 24°", size=12, color=RED,
        bold=True)

    ext = QPointF(T.x() + 320 * math.cos(los), T.y() + 320 * math.sin(los))
    p.setPen(QPen(QColor(TEAL), 1.2, Qt.DashLine))
    p.drawLine(T, ext)
    arc(p, T.x(), T.y(), 78, ta, los, color=AMBER, lw=2.2)
    txt(p, T.x() + 92, T.y() - 4, "qt", size=12, color=AMBER, bold=True)

    # 标记与标注
    missile_icon(p, M.x(), M.y(), ma)
    target_icon(p, T.x(), T.y(), ta)
    p.setPen(QPen(QColor(PURPLE), 2.4))
    p.drawLine(QPointF(P.x() - 10, P.y() - 10), QPointF(P.x() + 10, P.y() + 10))
    p.drawLine(QPointF(P.x() - 10, P.y() + 10), QPointF(P.x() + 10, P.y() - 10))
    txt(p, P.x() + 16, P.y() - 12, "拦截点 P = P_t + Vt·TOF", size=12,
        color=PURPLE, bold=True)
    txt(p, M.x() - 12, M.y() + 34, "M 导弹(现在)", size=11, bold=True)
    txt(p, T.x() - 46, T.y() - 16, "T 目标(现在)", size=11, bold=True)
    txt(p, M.x() + 60, M.y() + 58,
        "注意: 不是朝目标“现在”的位置飞(那必然落后), 而是朝相遇点飞",
        size=10.5, color=SUB)

    box(p, 60, 84, 520, 116,
        "① 几何条件（同一时刻到达同一点）\n"
        "     | P_t − P_m + Vt·t | = Vm · t\n"
        "② 展开成关于 t 的一元二次方程\n"
        "     ( |Vt|² − Vm² )·t² + 2·(ΔP·Vt)·t + |ΔP|² = 0",
        border=BLUE, fill="#eef4ff", size=11)
    box(p, 610, 84, 520, 116,
        "③ 等价的前置角写法（碰撞三角形）\n"
        "     Vm · sin φ  =  Vt · sin qt      φ = asin(Vt·sin qt / Vm)\n"
        "④ 解出 t 后：P = P_t + Vt·t，瞄准方向 û = (P − P_m)/(Vm·t)\n"
        "     Vm ≤ Vt 或判别式 < 0 ⇒ 目标不可达（退回指向当前位置）",
        border=GREEN, fill="#eaf7ee", size=11)

    box(p, 60, 648, 1070, 100,
        "怎么做：解上面的二次方程取最小正根（常速直线目标的闭式解）；目标会转弯时改用"
        "“迭代 TOF”：t ← |P_t(t) − P_m| / Vm 反复迭代 3~5 次（P_t(t) 用圆弧外推）。\n"
        "为什么：前置角就是“提前量”—— 目标在动，朝它现在的位置飞必然落后；提前到相遇点，"
        "末段才不需要急转（过载需求最小，脱靶量最小）。\n"
        "结果怎么样：本图几何（Vm=600、Vt=250 m/s）算得 TOF ≈ 1.1 s、前置角 ≈ 24°；"
        "默认场景 pip 中段据此把弹轴转到碰撞方向，5.84 s 即满足截获条件（见图 3）。[R01][R03][R06]",
        border=LINE, fill="#f6f8fa", size=10.5)
    p.end()
    save(img, "fig02_collision.png")


# ================================================ 图 3: 截获窗口 ============
def fig03():
    W, H = 1280, 640
    img, p = canvas(W, H)
    title(p, W, "导引头截获窗口：距离门限 R_acq × 视场 FOV（中段 → 末段的交接判据）",
          "截获 ⇔ R ≤ R_acq 且 |wrap(λ − ψ)| ≤ FOV/2；两个条件缺一不可")

    def scene(x0, ok, note):
        """场景: M 在面板中部, R_acq 与视场锥同尺度, 目标距离 < R_acq."""
        M = QPointF(x0 + 300, 360)
        psi = math.radians(-38.0)
        R_acq, L = 118.0, 178.0
        half = math.radians(45.0)
        diff_deg = 8.0 if ok else 58.0              # |λ−ψ|
        dist = 108.0                                 # < R_acq, 两图都满足距离条件
        bear = psi - math.radians(diff_deg)
        T = QPointF(M.x() + dist * math.cos(bear), M.y() + dist * math.sin(bear))
        c_main = GREEN if ok else AMBER

        # 截获距离圆
        p.setPen(QPen(QColor(BLUE), 1.5, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(M, R_acq, R_acq)
        txt(p, M.x() - R_acq - 172, M.y() - 6,
            "R_acq = 4.0 km\n(截获距离门限, 圈内满足)", size=10, color=BLUE,
            w=164, h=36)

        # 视场锥
        p1 = QPointF(M.x() + L * math.cos(psi - half), M.y() + L * math.sin(psi - half))
        p2 = QPointF(M.x() + L * math.cos(psi + half), M.y() + L * math.sin(psi + half))
        p.setPen(Qt.NoPen)
        c = QColor(c_main)
        c.setAlpha(46)
        p.setBrush(c)
        p.drawPolygon(QPolygonF([M, p1, p2]))
        p.setPen(QPen(QColor(c_main), 1.4, Qt.DotLine))
        p.setBrush(Qt.NoBrush)
        p.drawLine(M, p1)
        p.drawLine(M, p2)
        # FOV 标注放在下边缘外侧, 避免与其它标签重叠
        fx = M.x() + 104 * math.cos(psi + half)
        fy = M.y() + 104 * math.sin(psi + half)
        txt(p, fx + 10, fy + 12, "FOV/2 = 45°", size=10.5, color=c_main, bold=True)

        missile_icon(p, M.x(), M.y(), psi)
        txt(p, M.x() - 78, M.y() + 30, "M 弹轴 ψ", size=10.5, bold=True)

        # 目标与视线
        target_icon(p, T.x(), T.y(), math.radians(-20))
        p.setPen(QPen(QColor(TEAL), 1.6, Qt.DashLine))
        p.drawLine(M, T)
        los = math.atan2(T.y() - M.y(), T.x() - M.x())
        diff = abs((los - psi + math.pi) % (2 * math.pi) - math.pi)
        txt(p, (M.x() + T.x()) / 2 - 74, (M.y() + T.y()) / 2 - 26,
            f"λ−ψ = {math.degrees(diff):.0f}°", size=10.5, color=TEAL, bold=True)
        txt(p, T.x() + 16, T.y() - 8, "T 目标", size=10.5, color=TEAL)
        txt(p, M.x() + 6, M.y() - 132, f"R = {dist * 10:.0f} m ≤ R_acq ✓",
            size=10.5, color=BLUE)

        # 两条件 ✓/✗ 芯片行
        box(p, x0 + 20, 168, 540, 34,
            ("条件① R ≤ R_acq  ✓      条件② 角差 ≤ FOV/2  "
             + ("✓  →  两条件同时满足 ⇒ 截获" if ok else "✗  →  目标在视场外 ⇒ 不截获")),
            border=c_main, fill="#ffffff" if ok else "#fff8e6", size=10.5,
            bold=True)
        head = ("②  R ≤ R_acq  且  角差 ≤ FOV/2  →  截获成功，转入末段 PN/PP"
                if ok else
                "①  R ≤ R_acq 但角差 > FOV/2  →  目标在视场外，仍未截获")
        box(p, x0, 108, 580, 46, head,
            border=GREEN if ok else AMBER, fill="#eaf7ee" if ok else "#fff8e6",
            size=11.5, bold=True)
        box(p, x0, 496, 580, 116, note, border=LINE, fill="#f6f8fa", size=10.5)

    scene(40, False,
          "为什么两个条件要“与”：\n"
          "· 距离条件 R ≤ R_acq —— 超出探测距离，开机也看不见（能量/灵敏度门限）；\n"
          "· 视场条件 —— 导引头视场随弹轴，机头没指向目标就看不见（本程序不建模万向架）。\n"
          "结果：程序直飞 + FOV=10° 时全程未截获，脱靶 597.6 m，结果串标注“导引头未截获”。")
    scene(660, True,
          "怎么让②更容易成立（工程做法）：\n"
          "· 中段把弹轴指向预测拦截点(本程序 pip)，目标自然落在视场中央；\n"
          "· 或给导引头加万向架/搜索扫掠，把“可覆盖角”扩大到 ±30~60°；\n"
          "· 结果：默认场景 pip 中段 t=5.84 s、R=3.99 km 截获，末段 PN 脱靶 1.0 m。")
    p.end()
    save(img, "fig03_acquisition.png")


# ================================================ 图 4: 组合导航回路 ========
def fig04():
    W, H = 1300, 820
    img, p = canvas(W, H)
    title(p, W, "位置修正与组合导航：地面修正 + 卫星修正 + 数据链的融合结构",
          "怎么做：多信息源 → 卡尔曼滤波 → 有界误差状态；为什么：单一信息源都有致命短板")

    # 左列信息源
    src = [("IMU 陀螺 + 加速度计\n(捷联惯导解算)", 96, BLUE,
            "纯惯导: 位置误差随时间发散\n(随机漂移 → ~t, 舒勒振荡)"),
           ("卫星 GNSS 接收机\n伪距 + 载波相位(北斗/GPS)", 236, GREEN,
            "误差不随时间增长\n但更新率低、易被干扰/欺骗"),
           ("地形 / 景象匹配\nTERCOM · SITAN(地形辅助)", 376, PURPLE,
            "无对外辐射、抗干扰\n需预存数字高程/图像"),
           ("地面/平台雷达 + 数据链\n上行: 目标状态与修正指令", 516, AMBER,
            "唯一能提供“目标在哪”的外部信息\n受链路带宽/延迟/干扰制约")]
    for s, y, c, note in src:
        box(p, 40, y, 300, 84, s, border=c, fill="#ffffff", size=11, bold=True)
        txt(p, 44, y + 88, note, size=9.5, color=SUB, w=296, h=40)

    # 中列
    box(p, 400, 96, 250, 92, "INS 递推\nx̂⁻ = f(x̂, dt) 预测步", border=BLUE,
        fill="#eef4ff", size=11)
    box(p, 400, 226, 300, 176,
        "组合导航卡尔曼滤波\n(EKF / UKF / 紧耦合)\n\n"
        "预测: x̂⁻ = f(x̂,dt),  P⁻ = FPFᵀ+Q\n"
        "更新: K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹\n"
        "        x̂ = x̂⁻ + K(z − Hx̂⁻)",
        border=TEAL, fill="#e9f6f8", size=10.5, bold=False)
    box(p, 400, 516, 300, 92, "目标状态估计\n(CV 常速 / CT 匀转弯模型 + 滤波)",
        border=AMBER, fill="#fff8e6", size=11)

    # 右列
    box(p, 770, 96, 300, 92, "导弹位置/速度/姿态估计\n(修正后: 误差有界、不发散)",
        border=GREEN, fill="#eaf7ee", size=11)
    box(p, 770, 230, 300, 92, "拦截点解算 PIP + TOF\n(碰撞三角形 / 迭代 TOF)",
        border=PURPLE, fill="#f3ecfd", size=11)
    box(p, 770, 364, 300, 92, "中段制导指令\n飞向预测拦截点 PIP / 指令修正",
        border=GREEN, fill="#eaf7ee", size=11)
    box(p, 770, 498, 300, 92, "末段导引律 PN / PP / APN\n(导引头截获后, 测 λ̇)",
        border=RED, fill="#fdeeee", size=11)
    box(p, 400, 690, 300, 76, "执行机构(舵) → 弹体运动学\n实际位置更新", border=INK,
        fill="#ffffff", size=10.5)

    # 箭头: 左列(传感器/信息源) → 中列(处理)
    arrow(p, 340, 142, 400, 142, color=SUB)                # IMU → INS 递推
    arrow(p, 340, 282, 400, 282, color=SUB)                # GNSS → 卡尔曼
    arrow(p, 340, 422, 400, 380, color=SUB)                # 地形匹配 → 卡尔曼
    arrow(p, 340, 562, 400, 562, color=SUB)                # 数据链 → 目标状态估计
    arrow(p, 525, 188, 525, 226, color=BLUE)               # INS 递推 → 卡尔曼
    # 中列 → 右列(估计/解算/指令)
    arrow(p, 700, 264, 766, 150, color=GREEN, dash=True)   # 卡尔曼 → 导弹状态估计
    arrow(p, 700, 562, 766, 300, color=AMBER, dash=True)   # 目标状态估计 → 拦截点
    arrow(p, 920, 188, 920, 230, color=GREEN)              # 导弹状态 → 拦截点
    arrow(p, 920, 322, 920, 364, color=PURPLE)             # 拦截点 → 中段指令
    # 指令 → 执行 → 闭环
    arrow(p, 770, 440, 696, 686, color=GREEN, dash=True)   # 中段指令 → 执行机构
    arrow(p, 860, 590, 710, 690, color=RED)                # 末段导引律 → 执行机构
    arrow(p, 450, 766, 250, 700, color=INK, dash=True)     # 回到传感器(闭环)
    txt(p, 60, 694, "闭环: 实际运动\n→ 传感器再次测量", size=10, color=SUB,
        w=180, h=44, align=Qt.AlignLeft | Qt.AlignTop)

    box(p, 735, 676, 560, 130,
        "结果怎么样：\n"
        "· 纯 INS：误差随时间累积，短时高精度、长时失效；\n"
        "· INS+GNSS 组合：位置误差变为“有界小量”，并能在线估计\n"
        "  陀螺/加速度计零偏（可校准）；\n"
        "· 数据链刷新目标状态 → 拦截点误差 σ 减小 → 截获概率提高。",
        border=LINE, fill="#f6f8fa", size=10.5)
    p.end()
    save(img, "fig04_navigation.png")


# ================================================ 图 5: 三种遥控制导回路 ====
def fig05():
    W, H = 1300, 700
    img, p = canvas(W, H)
    title(p, W, "地面引导的三种基本回路：指令制导 / 半主动寻的 / 主动寻的",
          "差别只在一件事：谁在“看”目标、信息怎么传到弹上")

    panels = [
        (24, "(a) 指令制导 Command Guidance", BLUE,
         ["地面雷达\n同时测量导弹与目标（双目标）",
          "地面指令计算机\n解算偏差 → 修正指令",
          "上行无线电链\n（发给导弹）"],
         "特点: 弹上最简单、成本低；精度受地面雷达测角与链路延迟限制，越远越差，"
         "且地面站需全程跟踪导弹与目标（跟踪/照射负担重）。"),
        (448, "(b) 半主动寻的 Semi-Active", TEAL,
         ["照射源（地面/平台雷达）\n连续照射目标",
          "目标反射回波\n（能量来自照射源）",
          "弹上接收机\n测 λ、λ̇ → 导引律"],
         "特点: 弹上不需发射机，成本/体积适中；但发射平台必须全程照射，"
         "存在“照射-接收”几何限制与照射通道数（多目标）限制。"),
        (872, "(c) 主动寻的 Active（自带雷达/红外）", RED,
         ["弹上导引头发射\n并接收回波 / 红外成像",
          "测距 + 测角\n→ 目标状态",
          "弹上制导计算机\nPN / PP / OGL"],
         "特点: “发射后不管”、可同时打多目标；弹上最复杂，导引头体积/功率/成本最高，"
         "作用距离受天线孔径限制（探测方程 r ∝ 天线面积^¼）。"),
    ]

    PW = 404
    for px, head, color, items, note in panels:
        box(p, px, 96, PW, 470, "", border=LINE, fill="#f6f8fa")
        txt(p, px + 16, 128, head, size=12.5, bold=True, color=color,
            w=PW - 32, h=24, align=Qt.AlignLeft | Qt.AlignTop)
        bx, bw, bh = px + 22, PW - 44, 70
        for i, s in enumerate(items):
            y = 162 + i * 104
            box(p, bx, y, bw, bh, s, border=color, fill="#ffffff", size=10.5)
            if i < len(items) - 1:
                arrow(p, bx + bw / 2, y + bh + 4, bx + bw / 2, y + 100,
                      color=color, dash=True, hs=9)
        txt(p, px + 16, 486, note, size=10.5, color=SUB, w=PW - 32, h=72,
            align=Qt.AlignLeft | Qt.AlignTop)

    box(p, 24, 584, 1252, 92,
        "怎么做：三种回路都只解决“把 λ(和 λ̇) 送到制导律”这一件事；区别是测量设备放在哪里、"
        "信息用什么链路传递。   为什么：弹上资源(体积/功率/算力)有限，把“看目标”的任务外置可以简化弹上系统，"
        "代价是依赖平台与链路。   结果：指令制导适合近程低空，半主动适合中程，主动实现“发射后不管”与多目标；"
        "现代中远程防空普遍采用“中段指令/数据链 + 末段主动”的组合（对应本资料 02、06、07 章）。",
        border=LINE, fill="#ffffff", size=11)
    p.end()
    save(img, "fig05_guidance_loops.png")


# ================================================ 图 6: 实测 PN vs PP =======
def fig06():
    sys.path.insert(0, os.path.dirname(HERE))
    from simulation import SimConfig, Simulation

    cfg = SimConfig()
    sim = Simulation(cfg)
    sim.run(cfg.t_max)
    pn, pp, tgt = sim.missiles["pn"], sim.missiles["pp"], sim.target

    W, H = 1300, 760
    img, p = canvas(W, H)
    title(p, W, "两种末段导引律的实测对比（本项目仿真直接出图, 默认参数）",
          "左：弹道(轨迹)   右上：相对距离 R(t)   右下：法向过载 a(t)")

    # ---------------- 左: 弹道 ----------------
    L = QRectF(50, 100, 700, 600)
    p.setPen(QPen(QColor(LINE), 1.2))
    p.setBrush(QColor("#ffffff"))
    p.drawRect(L)
    size = cfg.field_size

    def w2s(x, y):
        return (L.left() + x / size * L.width(),
                L.bottom() - y / size * L.height())

    for i in range(6):
        gx = L.left() + i * L.width() / 5
        gy = L.top() + i * L.height() / 5
        p.setPen(QPen(QColor("#eaeef2"), 1))
        p.drawLine(QPointF(gx, L.top()), QPointF(gx, L.bottom()))
        p.drawLine(QPointF(L.left(), gy), QPointF(L.right(), gy))
        p.setFont(QFont(FONT, 9))
        p.setPen(QColor(SUB))
        p.drawText(QPointF(gx - 8, L.bottom() + 16), f"{i*2} km")
        p.drawText(QPointF(L.left() - 30, gy + 4), f"{10-i*2}")

    def draw_trail(trail, color, w=2.2, dash=False):
        if len(trail) < 2:
            return
        pen = QPen(QColor(color), w)
        if dash:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        pts = [QPointF(*w2s(x, y)) for x, y in trail]
        for a, b in zip(pts, pts[1:]):
            p.drawLine(a, b)

    draw_trail(tgt.trail, TEAL, 1.6, dash=True)
    draw_trail(pn.trail, ORANGE, 2.4)
    draw_trail(pp.trail, PINK, 2.4)

    for m, c in ((pn, ORANGE), (pp, PINK)):
        x, y = w2s(m.x0, m.y0)
        dot(p, x, y, 4.5, c)
    sx, sy = w2s(pn.x0, pn.y0)
    txt(p, sx + 10, sy + 16, "发射点", size=10, color=SUB)
    tx, ty = w2s(tgt.x, tgt.y)
    t_trail = tgt.trail
    if len(t_trail) >= 2:
        (ax, ay), (bx, by2) = t_trail[-2], t_trail[-1]
        t_ang = math.atan2(-(by2 - ay), bx - ax)      # 世界角 → 屏幕角
    else:
        t_ang = 0.0
    target_icon(p, tx, ty, t_ang)
    txt(p, tx - 70, ty - 20, "目标 TGT", size=10.5, color=TEAL, bold=True)

    # 图例
    ly = L.top() + 10
    for i, (name, c, dash) in enumerate(
            [("目标航线", TEAL, True), ("比例导引 PN", ORANGE, False),
             ("纯跟踪 PP", PINK, False)]):
        y = ly + i * 20
        pen = QPen(QColor(c), 2.4)
        if dash:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.drawLine(QPointF(L.left() + 12, y), QPointF(L.left() + 46, y))
        txt(p, L.left() + 54, y + 4, name, size=10.5, color=INK)

    # ---------------- 右上: R(t) ----------------
    R = QRectF(790, 100, 460, 250)
    p.setPen(QPen(QColor(LINE), 1.2)); p.setBrush(QColor("#ffffff")); p.drawRect(R)
    Rin = QRectF(R.left() + 52, R.top() + 34, R.width() - 62, R.height() - 46)
    r0 = max(m.hist_r[0] for m in (pn, pp) if m.hist_r)
    tmax = max(pn.hist_t[-1], pp.hist_t[-1]) if pn.hist_t else 1.0

    def draw_curve(m, color, field, rect=Rin):
        if not m.hist_t:
            return
        p.setPen(QPen(QColor(color), 2.2))
        pts = [QPointF(rect.left() + t / tmax * rect.width(),
                       rect.bottom() - v / r0 * rect.height())
               for t, v in zip(m.hist_t, getattr(m, field))]
        for a, b in zip(pts, pts[1:]):
            p.drawLine(a, b)
        return pts[-1]

    txt(p, R.left() + 10, R.top() + 20, "相对距离 R(t)  [m]", size=11, bold=True)
    p.setPen(QPen(QColor("#eaeef2"), 1))
    p.drawLine(QPointF(Rin.left(), Rin.bottom()), QPointF(Rin.right(), Rin.bottom()))
    draw_curve(pn, ORANGE, "hist_r")
    draw_curve(pp, PINK, "hist_r")
    p.setPen(QColor(SUB)); p.setFont(QFont(FONT, 9))
    p.drawText(QPointF(Rin.left() - 44, Rin.top() + 8), f"{r0:,.0f}")
    p.drawText(QPointF(Rin.left() - 18, Rin.bottom() + 4), "0")
    p.drawText(QPointF(Rin.left() - 6, Rin.bottom() + 18), "0")
    p.drawText(QPointF(Rin.right() - 40, Rin.bottom() + 18), f"{tmax:.1f} s")
    txt(p, Rin.left() + 10, Rin.top() + 20,
        "PN 与 PP 两条线几乎重合：相对距离都稳定单调逼近 0", size=9.5, color=SUB)

    # ---------------- 右下: a(t) ----------------
    A = QRectF(790, 390, 460, 310)
    p.setPen(QPen(QColor(LINE), 1.2)); p.setBrush(QColor("#ffffff")); p.drawRect(A)
    Ain = QRectF(A.left() + 52, A.top() + 30, A.width() - 62, A.height() - 56)
    amax = cfg.max_g * 9.80665
    for m in (pn, pp):
        if m.hist_a:
            amax = max(amax, max(abs(v) for v in m.hist_a) * 1.08)
    mid = (Ain.top() + Ain.bottom()) / 2
    half_h = Ain.height() / 2

    def draw_curve_a(m, color):
        if not m.hist_t:
            return
        p.setPen(QPen(QColor(color), 2.2))
        pts = [QPointF(Ain.left() + t / tmax * Ain.width(),
                       mid - v / amax * half_h)
               for t, v in zip(m.hist_t, m.hist_a)]
        for a, b in zip(pts, pts[1:]):
            p.drawLine(a, b)
        return pts[-1]

    p.setPen(QPen(QColor(RED), 1.2, Qt.DashLine))
    for sgn, lab in ((+1, "+Gmax"), (-1, "−Gmax")):
        y = mid - sgn * half_h
        p.drawLine(QPointF(Ain.left(), y), QPointF(Ain.right(), y))
        txt(p, Ain.left() - 46, y + (13 if sgn > 0 else 4), lab, size=9.5,
            color=RED)
    p.setPen(QPen(QColor(LINE), 1)); p.drawLine(QPointF(Ain.left(), mid),
                                                QPointF(Ain.right(), mid))
    txt(p, Ain.left() - 24, mid + 4, "0", size=9.5, color=SUB)
    draw_curve_a(pn, ORANGE)
    ea = draw_curve_a(pp, PINK)
    txt(p, A.left() + 10, A.top() + 20, "导弹法向过载 a(t)  [m/s²]", size=11,
        bold=True)
    p.setPen(QColor(SUB)); p.setFont(QFont(FONT, 9))
    p.drawText(QPointF(Ain.left() - 6, Ain.bottom() + 16), "0")
    p.drawText(QPointF(Ain.right() - 40, Ain.bottom() + 16), f"{tmax:.1f} s")

    # 曲线特征标注
    txt(p, Ain.left() + 10, Ain.top() + 18, "PN: 前段小尖峰 → 中段平缓(近 0)",
        size=9.5, color=ORANGE)
    txt(p, Ain.left() + 190, Ain.bottom() - 12, "PP: 末端发散并顶满 ±Gmax",
        size=9.5, color=PINK)

    # 结果注记
    box(p, 50, 716, 1200, 40, "", border=LINE, fill="#f6f8fa")
    txt(p, 64, 742,
        f"默认场景实测：比例导引 脱靶 {pn.min_range:.1f} m（过载平稳、很少贴 ±Gmax）；"
        f"纯跟踪 脱靶 {pp.min_range:.1f} m（末端过载发散并饱和）。",
        size=11, color=INK)
    p.end()
    save(img, "fig06_pn_pp.png")


# ================================================ 图 7: 概念分层 ============
def fig07():
    W, H = 1240, 680
    img, p = canvas(W, H)
    title(p, W, "四层概念分层：任务规划 ≠ 拦截点解算 ≠ 制导律 ≠ 执行",
          "越往上越“离线/全局”，越往下越“在线/实时”——三者常被混用，其实回答不同问题")

    layers = [
        ("任务层 · 路径规划", BLUE, 96,
         "问题：这条“路”怎么走？（避障/禁飞区/威胁/地形）",
         "输入：地图与先验环境   输出：一串航路点/航向序列   时间：通常与“何时到达”解耦",
         "算法：Dijkstra · A* · Voronoi · RRT/RRT* · 人工势场 · 伪谱法/凸优化",
         "本项目：战场为空，无路径规划（仅作概念对照）"),
        ("解算层 · 拦截点计算", PURPLE, 236,
         "问题：目标会在哪一点、哪一刻出现？",
         "输入：双方位置与速度(含滤波估计)   输出：拦截点 P + 飞行时间 TOF   时间：核心！必须解出 t",
         "算法：碰撞三角形闭式解 · 迭代 TOF · ZEM 反解 · 卡尔曼预测外推",
         "本项目：predict_intercept()（pip 中段，见 06 章 / docs/05）"),
        ("制导层 · 导引律", RED, 376,
         "问题：当前偏差多大、要产生多少过载去修正？",
         "输入：λ、λ̇、R、Vc、目标估计   输出：法向过载指令 a_cmd   时间：每步(毫秒级)重算",
         "算法：PN/TPN/APN · 纯跟踪 · 迭代制导 IPN · 最优 OGL · 落角约束/偏置 PN · 滑模 · 微分对策",
         "本项目：guidance_command()（PN 与 PP，见 08 章）"),
        ("执行层 · 控制与动力", AMBER, 516,
         "问题：这条指令能不能打出来？",
         "输入：a_cmd   输出：实际过载/舵偏 → 弹体真实运动   时间：毫秒级 + 一阶/二阶滞后",
         "内容：舵机与执行机构 · 弹体动力学 · 过载/舵偏限幅 · 速率限制 · 姿态控制",
         "本项目：一阶滞后 τ + ±Gmax 限幅（见 08/10 章）"),
    ]
    for name, c, y, q, io, alg, note in layers:
        box(p, 50, y, 1140, 126, "", border=c, fill="#ffffff", lw=1.8)
        p.setPen(QColor(c))
        p.setBrush(QColor(c))
        p.drawRoundedRect(QRectF(50, y, 8, 126), 4, 4)
        txt(p, 70, y + 26, name, size=13, bold=True, color=c)
        txt(p, 70, y + 56, q, size=11, color=INK)
        txt(p, 70, y + 80, io, size=10.5, color=SUB)
        txt(p, 70, y + 102, alg, size=10.5, color=SUB)
        txt(p, 760, y + 26, note, size=10.5, color=c, w=410, h=70)

    for y in (96 + 126, 236 + 126, 376 + 126):
        arrow(p, 620, y - 4, 620, y + 26, color=SUB, dash=True, hs=9)
    txt(p, 50, 672,
        "典型误区：把“算出拦截点”当成“规划好弹道”——拦截点只是几何解，能不能飞到由制导层+执行层决定，"
        "所以 PIP 必须每步重算。", size=10.5, color=SUB)
    p.end()
    save(img, "fig07_layers.png")


# ================================================ 图 8: ZEM =================
def fig08():
    W, H = 1180, 720
    img, p = canvas(W, H)
    title(p, W, "ZEM（零 effort 脱靶量）：把“未来会差多远”直接换算成“现在要多大过载”",
          "ZEM = 从现在起不再施加任何控制时的预计脱靶量；ZEM = 0 ⇔ 已在碰撞航线上")

    M = QPointF(190, 540)
    T0 = QPointF(640, 300)
    ta = math.radians(-18.0)                      # 目标航向
    T1 = QPointF(T0.x() + 300 * math.cos(ta), T0.y() + 300 * math.sin(ta))

    # 目标航线
    p.setPen(QPen(QColor(TEAL), 2.0, Qt.DashLine))
    p.drawLine(T0, T1)
    target_icon(p, T0.x(), T0.y(), ta)
    target_icon(p, T1.x(), T1.y(), ta)
    txt(p, T0.x() - 40, T0.y() + 30, "T(现在)", size=11, color=TEAL, bold=True)
    txt(p, T1.x() + 16, T1.y() + 6, "T′(TOF 后)\n= 预测拦截点", size=11,
        color=PURPLE, bold=True)

    # 不加控制的直线外推(失效方向)
    ve = math.radians(-4.0)
    E = QPointF(M.x() + 760 * math.cos(ve), M.y() + 760 * math.sin(ve))
    p.setPen(QPen(QColor(RED), 2.0, Qt.DashLine))
    p.drawLine(M, E)
    txt(p, M.x() + 250, M.y() + 30, "若从现在起不再修正 → 沿当前速度直线外推",
        size=11, color=RED)

    # ZEM: T′ 到外推直线的垂距
    dx, dy = E.x() - M.x(), E.y() - M.y()
    L2 = dx * dx + dy * dy
    tt = ((T1.x() - M.x()) * dx + (T1.y() - M.y()) * dy) / L2
    F = QPointF(M.x() + tt * dx, M.y() + tt * dy)
    p.setPen(QPen(QColor(RED), 2.6))
    p.drawLine(T1, F)
    ang = math.atan2(F.y() - T1.y(), F.x() - T1.x())
    for sgn in (1, -1):
        a2 = ang + sgn * 0.5
        p.drawLine(T1, QPointF(T1.x() + 12 * math.cos(a2),
                               T1.y() + 12 * math.sin(a2)))
    txt(p, (T1.x() + F.x()) / 2 + 8, (T1.y() + F.y()) / 2 - 8,
        "ZEM (零 effort 脱靶量)", size=12, color=RED, bold=True)

    # 修正后的命中轨迹(二次曲线)
    ctrl = QPointF(M.x() + 330, M.y() - 30)
    path_pts = []
    for i in range(41):
        u = i / 40.0
        bx = (1 - u) ** 2 * M.x() + 2 * (1 - u) * u * ctrl.x() + u * u * T1.x()
        by = (1 - u) ** 2 * M.y() + 2 * (1 - u) * u * ctrl.y() + u * u * T1.y()
        path_pts.append(QPointF(bx, by))
    p.setPen(QPen(QColor(ORANGE), 2.6))
    for a, b in zip(path_pts, path_pts[1:]):
        p.drawLine(a, b)
    missile_icon(p, path_pts[-6].x(), path_pts[-6].y(),
                 math.atan2(path_pts[-1].y() - path_pts[-6].y(),
                            path_pts[-1].x() - path_pts[-6].x()))
    txt(p, M.x() + 300, M.y() - 96, "施加过载修正 → 命中", size=12,
        color=ORANGE, bold=True)

    p.setPen(QPen(QColor(LINE), 1.2, Qt.DashLine))
    p.drawLine(QPointF(M.x(), M.y()), QPointF(M.x() + 470, M.y()))
    los_a = math.atan2(T0.y() - M.y(), T0.x() - M.x())
    arc(p, M.x(), M.y(), 110, los_a, 0.0, color=SUB)
    txt(p, M.x() + 118, M.y() - 12, "λ", size=11, color=SUB)
    arc(p, M.x(), M.y(), 70, los_a, ve, color=RED, lw=1.8)
    txt(p, M.x() + 74, M.y() - 30, "ψ", size=11, color=RED)
    missile_icon(p, M.x(), M.y(), ve)
    txt(p, M.x() - 40, M.y() + 32, "M 导弹", size=11, bold=True)

    box(p, 50, 96, 520, 130,
        "怎么做：\n"
        "  ① 用目标状态(含滤波速度)外推 TOF 后的位置 T′；\n"
        "  ② 计算当前速度直线外推与 T′ 的垂直距离 = ZEM；\n"
        "  ③ 令 a_cmd ≈ N · ZEM / TOF²，一步换算成过载指令。",
        border=PURPLE, fill="#f3ecfd", size=11)
    box(p, 610, 96, 520, 130,
        "为什么有效：\n"
        "  · ZEM 是“预测量”，不需要等误差真的出现再修；\n"
        "  · 分母 TOF² 体现“越晚修正所需过载越大”的物理约束；\n"
        "  · ZEM=0 与 λ̇=0 等价 —— 与比例导引同一思想的两种度量。",
        border=BLUE, fill="#eef4ff", size=11)
    box(p, 50, 636, 1080, 66,
        "结果怎么样：本程序中段 pip 模式每步都在“把 ZEM 压到 0”（等价于飞向预测拦截点），"
        "默认场景 5.84 s 就把目标送进截获窗口；末段 PN 再用 λ̇ 收尾，最终脱靶 1.0 m。"
        "参考文献：[R01][R03][R06]（ZEM/OGL 推导）。",
        border=LINE, fill="#f6f8fa", size=10.5)
    p.end()
    save(img, "fig08_zem.png")


# ================================================ 图 9: 脱靶量/CEP =========
def fig09():
    W, H = 1240, 700
    img, p = canvas(W, H)
    title(p, W, "从“脱靶量”到“杀伤概率”：命中判定、CEP 与毁伤半径",
          "导引律最终只回答一个问题：能否把脱靶量压到杀伤半径以内")

    # 左: 单次交会
    box(p, 40, 96, 560, 470, "", border=LINE, fill="#ffffff")
    txt(p, 56, 126, "(a) 单次交会：脱靶量 vs 杀伤半径", size=13, bold=True,
        color=BLUE)
    C = QPointF(320, 330)
    R_screen = 120.0
    p.setPen(QPen(QColor(RED), 1.8, Qt.DashLine))
    p.setBrush(QColor(QColor(RED).red(), QColor(RED).green(),
                      QColor(RED).blue(), 28))
    p.drawEllipse(C, R_screen, R_screen)
    txt(p, C.x() - 66, C.y() + R_screen + 22, "杀伤半径 R_hit = 25 m", size=11,
        color=RED, bold=True)
    target_icon(p, C.x(), C.y(), math.radians(-20), s=17)
    # 弹道经过(从圆下方掠过: 与圆心距离 67px ≈ 14 m, 位于杀伤半径内)
    p.setPen(QPen(QColor(ORANGE), 2.6))
    p.drawLine(QPointF(70, 470), QPointF(570, 330))
    # 最近点与脱靶量: C 到直线的垂足 (直线 0.28x + y − 489.6 = 0)
    foot = QPointF(338, 395)
    p.setPen(QPen(QColor(PURPLE), 2.0, Qt.DashLine))
    p.drawLine(C, foot)
    p.setPen(QPen(QColor(PURPLE), 1.6))
    for sgn in (1, -1):
        p.drawLine(foot, QPointF(foot.x() - 8, foot.y() + sgn * 8))
    txt(p, foot.x() + 8, foot.y() - 6, "脱靶量 d ≈ 14 m", size=11,
        color=PURPLE, bold=True)
    txt(p, 76, 458, "弹道(橙, 与目标最近距离 d)", size=10.5, color=ORANGE)
    box(p, 56, 496, 528, 60,
        "判据：整条弹道上目标到弹道折线的最小距离 d ≤ R_hit → 命中。\n"
        "本程序：R_hit 默认 25 m，脱靶量按“点到线段”精确计算（非采样点）。",
        border=LINE, fill="#f6f8fa", size=10.5)

    # 右: CEP 散布
    box(p, 640, 96, 560, 470, "", border=LINE, fill="#ffffff")
    txt(p, 656, 126, "(b) 多次射击：落点散布与 CEP", size=13, bold=True,
        color=PURPLE)
    S = QPointF(920, 320)
    import random
    random.seed(7)
    pts = []
    for _ in range(60):
        r = abs(random.gauss(0, 62))      # 二维瑞利分布的半径
        a = random.uniform(0, 2 * math.pi)
        pts.append((S.x() + r * math.cos(a), S.y() + r * math.sin(a)))
    for x, y in pts:
        dot(p, x, y, 3.0, BLUE)
    cep = 73.0
    p.setPen(QPen(QColor(PURPLE), 2.0, Qt.DashLine))
    p.setBrush(Qt.NoBrush)
    p.drawEllipse(S, cep, cep)
    txt(p, S.x() - 60, S.y() + cep + 22, "CEP(50% 落点在圆内)", size=11,
        color=PURPLE, bold=True)
    p.setPen(QPen(QColor(RED), 1.4, Qt.DotLine))
    p.setBrush(Qt.NoBrush)
    p.drawEllipse(S, 40, 40)
    txt(p, S.x() + 46, S.y() - 40, "R_hit", size=10.5, color=RED)
    box(p, 656, 470, 528, 86,
        "σ ≈ CEP / 1.1774（二维正态/瑞利分布）\n"
        "单发杀伤概率  P = 1 − exp( −R_hit² / (2σ²) )\n"
        "提高 P 的两条路：减小 σ(更好的导引律/更小脱靶量) 或 增大 R_hit(战斗部/引信)。",
        border=LINE, fill="#f6f8fa", size=10.5)

    box(p, 40, 586, 1160, 84,
        "怎么做：① 每次仿真/打靶记录最小距离 d → 统计均值与散布 σ → 得 CEP；② 给定 R_hit 用上式算杀伤概率。\n"
        "为什么：单次“命中”不能说明武器系统好坏，必须看统计散布；导引律越优 → σ 越小 → 同样战斗部下杀伤概率越高。\n"
        "结果怎么样：本程序默认场景 PN 脱靶 0.2 m、PP 67.8 m（PP 已超出 25 m 杀伤半径 ⇒ 判定脱靶）；详见 10 章指标表。",
        border=LINE, fill="#ffffff", size=10.5)
    p.end()
    save(img, "fig09_miss.png")


# ================================================================ 主函数 ====
def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    app = QApplication(sys.argv)          # 字体/文本绘制需要
    print("生成资料插图 →", FIG_DIR)
    fig01(); fig02(); fig03(); fig04(); fig05()
    fig06(); fig07(); fig08(); fig09()
    print("完成。")


if __name__ == "__main__":
    main()
