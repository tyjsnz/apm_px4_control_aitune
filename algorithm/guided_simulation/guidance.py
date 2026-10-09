# -*- coding: utf-8 -*-
"""导弹导引仿真 —— 导引律与质点运动学模型.

坐标系约定
----------
* 二维平面直角坐标系, 单位: 米(m), x 轴向右, y 轴向上(数学惯例, 屏幕绘制时翻转).
* 角度: 弧度(rad), 从 +x 轴起逆时针为正.
* 导弹/目标均当作"质点 + 速度矢量"处理(圆运动学), 侧向过载 a 改变速度矢量方向:
      d(航向)/dt = a / V

包含
----
* :class:`MissileParams`  导弹制导参数
* :class:`Target`         目标飞机(可直线 / 转弯 / 正弦机动)
* :class:`Missile`        导弹(含比例导引、纯跟踪两种导引律)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

G0 = 9.80665  # 重力加速度, 用于把 m/s^2 换算为过载 g

MID_GAIN = 2.5  # 中段制导的航向收敛增益 (1/s), 类似纯跟踪但更平缓


# --------------------------------------------------------------------------
# 通用小工具
# --------------------------------------------------------------------------
def wrap_pi(a: float) -> float:
    """把角度归一化到 (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def closest_on_segment(px, py, ax, ay, bx, by):
    """点 p 到线段 ab 的距离与最近点(用于精确计算脱靶量)."""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 <= 1e-12 else clamp((wx * vx + wy * vy) / l2, 0.0, 1.0)
    cx, cy = ax + t * vx, ay + t * vy
    return math.hypot(px - cx, py - cy), cx, cy


# --------------------------------------------------------------------------
# 参数
# --------------------------------------------------------------------------
@dataclass
class MissileParams:
    """导弹制导参数(两种导弹共享一份)."""

    nav_ratio: float = 4.0    # N  比例导引导航比
    pp_gain: float = 4.0      # K  纯跟踪航向修正增益 (1/s)
    max_g: float = 9.0        # 最大可用法向过载 (g)
    tau: float = 0.3          # 执行机构/弹体 一阶时间常数 (s)
    use_vc: bool = True       # 比例导引系数中用接近速度 Vc(否则用弹速 Vm)

    # ---- 分段制导: 中段(导引头未截获) -> 末段(已截获) ----
    phased: bool = False      # 是否启用分段制导
    acq_range: float = 4000.0  # 导引头截获距离 R_acq (m): R 小于它才有机会截获
    acq_fov: float = 90.0     # 导引头截获视场 FOV (全角, deg): 目标须落在视场内
    mid_mode: str = "pip"     # 中段方式: "pip" 指向预测拦截点 | "straight" 程序直飞


# --------------------------------------------------------------------------
# 目标飞机
# --------------------------------------------------------------------------
class Target:
    """目标飞机质点模型.

    机动模式 mode:
        * ``straight`` 匀速直线
        * ``left``     持续左转(逆时针), 转弯率 = +turn_rate
        * ``right``    持续右转(顺时针), 转弯率 = -turn_rate
        * ``sine``     正弦机动, 转弯率 = turn_rate * sin(2*pi*t/period)
    """

    MODES = ("straight", "left", "right", "sine")

    def __init__(self, x, y, heading_deg, speed,
                 mode="straight", turn_rate_deg=6.0, period=4.0):
        self.x0, self.y0 = float(x), float(y)
        self.h0 = float(heading_deg)
        self.speed = float(speed)
        self.mode = mode
        self.turn_rate = float(turn_rate_deg)
        self.period = float(period)
        self.reset()

    def reset(self):
        self.x, self.y = self.x0, self.y0
        self.heading = math.radians(self.h0)
        self.t = 0.0
        self.omega = 0.0          # 当前转弯角速度 rad/s
        self.trail = [(self.x, self.y)]

    @property
    def vx(self):
        return self.speed * math.cos(self.heading)

    @property
    def vy(self):
        return self.speed * math.sin(self.heading)

    @property
    def heading_deg(self):
        return math.degrees(self.heading)

    def step(self, dt: float):
        """推进一个仿真步长."""
        w = math.radians(self.turn_rate)
        if self.mode == "straight":
            self.omega = 0.0
        elif self.mode == "left":
            self.omega = w
        elif self.mode == "right":
            self.omega = -w
        else:  # sine
            self.omega = w * math.sin(2.0 * math.pi * self.t / max(self.period, 1e-3))

        self.heading = wrap_pi(self.heading + self.omega * dt)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.t += dt
        self.trail.append((self.x, self.y))

    def position_at(self, t_ahead: float) -> tuple:
        """外推 t_ahead 秒后的位置(按当前速度与当前转弯率, 中段预测用).

        直线时: P = p + v·t
        转弯时: 恒角速度圆弧, ∫V·(cos,sin)(h0+ωt)dt 的解析解
        """
        if t_ahead <= 0.0:
            return self.x, self.y
        w = self.omega
        if abs(w) < 1e-9:
            return self.x + self.vx * t_ahead, self.y + self.vy * t_ahead
        ang = w * t_ahead
        k = self.speed / w
        dx = k * (math.sin(self.heading + ang) - math.sin(self.heading))
        dy = k * (math.cos(self.heading) - math.cos(self.heading + ang))
        return self.x + dx, self.y + dy


# --------------------------------------------------------------------------
# 拦截点预测
# --------------------------------------------------------------------------
def predict_intercept(target: Target, mx: float, my: float, vm: float,
                      iters: int = 5, t_max: float = 60.0) -> tuple:
    """迭代求解**预测拦截点 (PIP, Predicted Intercept Point)** 与飞行时间 TOF.

    要解的问题: 导弹以速度 vm 沿直线飞去, 何时何地能与目标相遇?

        | target.position_at(t) − (mx, my) | = vm · t

    这是一个 t 的标量方程, 用不动点迭代求解(经典"迭代 TOF")::

        t0 = R / vm
        t_{k+1} = | target.position_at(t_k) − P_m | / vm      (k = 0..iters-1)

    当 vm > Vt 时该迭代是压缩映射, 通常 3~5 次即收敛。
    目标转弯时由 :meth:`Target.position_at` 给出圆弧外推, 因此对机动目标也适用。

    Returns
    -------
    (tof, x, y) : 拦截时刻(s) 与 预测拦截点坐标(m)
    """
    t = math.hypot(target.x - mx, target.y - my) / max(vm, 1.0)
    for _ in range(max(iters, 1)):
        ax, ay = target.position_at(t)
        t = clamp(math.hypot(ax - mx, ay - my) / max(vm, 1.0), 0.0, t_max)
    ax, ay = target.position_at(t)
    return t, ax, ay


# --------------------------------------------------------------------------
# 导弹
# --------------------------------------------------------------------------
class Missile:
    """导弹质点模型 + 两种导引律.

    kind:
        * ``"pn"`` 比例导引 Proportional Navigation
        * ``"pp"`` 纯跟踪(追踪法) Pure Pursuit

    每个 step 的执行顺序::

        1) 测量: 视线角 lambda、视线转率 lambda_dot、接近速度 Vc
        2) 导引律: 由测量量算出法向过载指令 a_cmd, 并按最大过载限幅
        3) 执行机构: 一阶滞后 tau, 得到实际过载 a
        4) 运动学积分: 航向 += a/V*dt ; 位置 += V*dt
        5) 记录: 弹道、距离、过载等历史 + 最小距离(脱靶量)
    """

    def __init__(self, kind, name, color, x, y, heading_deg, speed, params: MissileParams):
        self.kind = kind          # "pn" / "pp"
        self.name = name          # 中文显示名
        self.color = color
        self.x0, self.y0 = float(x), float(y)
        self.h0 = float(heading_deg)
        self.v0 = float(speed)
        self.params = params
        self.reset()

    # ------------------------------ 状态复位 ------------------------------
    def reset(self):
        self.x, self.y = self.x0, self.y0
        self.heading = math.radians(self.h0)
        self.speed = self.v0
        self.a = 0.0              # 当前实际法向过载 (m/s^2)
        self.a_cmd = 0.0          # 过载指令 (m/s^2)
        self.t = 0.0
        # 观测量
        self.range = 0.0
        self.los = 0.0
        self.los_dot = 0.0
        self.vc = 0.0
        # 脱靶量
        self.min_range = float("inf")
        self.min_point = (self.x, self.y)
        self.min_t = 0.0
        self.closed_once = False  # 是否曾经处于"接近"状态
        self.open_t = 0.0          # 处于"远离"状态的累计时间
        # 结果
        self.done = False
        self.hit = False
        self.result = "飞行中"
        # 分段制导状态(中段 = 导引头未截获)
        self.acquired = not self.params.phased   # 不分段时视为"开机即已截获"
        self.acq_t = 0.0                         # 截获时刻 (s)
        self.acq_r = float("inf")                # 截获瞬间的相对距离 (m)
        self.acq_pos = None                      # 截获点坐标 (用于画布标记)
        self.trail_split = 0                     # 弹道分段下标: [0:split]=中段
        self.phase = "末段" if self.acquired else "中段"
        # 历史
        self.trail = [(self.x, self.y)]
        self.hist_t, self.hist_r = [], []
        self.hist_a, self.hist_vc, self.hist_losdot = [], [], []

    # ------------------------------ 观测 ------------------------------
    def sense(self, tgt: Target):
        """计算视线角 lambda、视线转率 lambda_dot、接近速度 Vc."""
        dx = tgt.x - self.x
        dy = tgt.y - self.y
        r2 = dx * dx + dy * dy
        r = math.sqrt(r2)
        vx = self.speed * math.cos(self.heading)
        vy = self.speed * math.sin(self.heading)
        rvx = tgt.vx - vx           # 相对速度
        rvy = tgt.vy - vy

        los = math.atan2(dy, dx)
        # lambda_dot = (r x v_rel) / |r|^2   (二维叉积)
        los_dot = (dx * rvy - dy * rvx) / max(r2, 1.0)
        # Vc = -d(r)/dt = -(r . v_rel)/|r|
        vc = -(dx * rvx + dy * rvy) / r if r > 1e-6 else 0.0
        return r, los, los_dot, vc

    # ------------------------------ 导引律 ------------------------------
    def guidance_command(self, r, los, los_dot, vc):
        """导引律: 计算法向过载指令 (m/s^2).

        比例导引 (PN):
            a_cmd = N * V_ref * lambda_dot
            其中 V_ref = Vc (接近速度, 工程常用) 或 Vm (经典形式).
            等价于: 导弹速度矢量转率 = N * 视线转率.

        纯跟踪 (PP):
            a_cmd = Vm * K * wrap(lambda - psi)
            即: 航向以增益 K 收敛到视线方向 -> 速度矢量始终指向目标.
        """
        p = self.params
        if self.kind == "pn":
            v_ref = vc if (p.use_vc and vc > 0.0) else float(self.speed)
            return p.nav_ratio * v_ref * los_dot
        # 纯跟踪
        return self.speed * p.pp_gain * wrap_pi(los - self.heading)

    def midcourse_command(self, tgt: Target) -> float:
        """中段(导引头未截获)的过载指令 (m/s^2).

        * ``pip``     指向**预测拦截点 PIP**: 先迭代解出 TOF 与拦截点,
                      再像"纯跟踪 PIP"那样把机头转过去 —— 相当于
                      INS 递推 + 火控解算的指令修正。
        * ``straight`` 程序直飞: 保持发射航向不变(纯惯性/程序飞行,
                      不做任何目标反馈), 用来演示"没有修正会怎样"。
        """
        p = self.params
        if p.mid_mode == "straight":
            return 0.0
        _, ax, ay = predict_intercept(tgt, self.x, self.y, self.speed)
        theta = math.atan2(ay - self.y, ax - self.x)
        return self.speed * MID_GAIN * wrap_pi(theta - self.heading)

    def _check_acquire(self, r: float, los: float):
        """导引头截获判据: R ≤ R_acq **且** 目标落在视场 FOV 内(相对弹轴)."""
        p = self.params
        if r > p.acq_range:
            return
        fov_half = 0.5 * math.radians(max(p.acq_fov, 0.1))
        if abs(wrap_pi(los - self.heading)) <= fov_half:
            self.acquired = True
            self.acq_t = self.t
            self.acq_r = r
            self.acq_pos = (self.x, self.y)
            self.trail_split = len(self.trail)
            self.phase = "末段"

    # ------------------------------ 单步推进 ------------------------------
    def step(self, tgt: Target, dt: float):
        if self.done:
            return
        p = self.params
        r, los, los_dot, vc = self.sense(tgt)
        self.range, self.los, self.los_dot, self.vc = r, los, los_dot, vc

        # 0) 分段制导: 先判导引头是否截获(截获后即进入末段)
        if p.phased and not self.acquired:
            self._check_acquire(r, los)
        self.phase = "末段" if (not p.phased or self.acquired) else "中段"

        # 1) 导引律 -> 过载指令(带饱和)
        a_max = p.max_g * G0
        if self.phase == "中段":
            cmd = self.midcourse_command(tgt)      # 中段: PIP/程序直飞
        else:
            cmd = self.guidance_command(r, los, los_dot, vc)   # 末段: PN / PP
        self.a_cmd = clamp(cmd, -a_max, a_max)

        # 2) 执行机构一阶滞后
        alpha = 1.0 - math.exp(-dt / max(p.tau, 1e-3))
        self.a += (self.a_cmd - self.a) * alpha
        self.a = clamp(self.a, -a_max, a_max)

        # 3) 运动学积分(半隐式欧拉: 先转航向再平移)
        px, py = self.x, self.y
        self.heading = wrap_pi(self.heading + self.a / max(self.speed, 1.0) * dt)
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt
        self.t += dt

        # 4) 目标到本段弹道的最小距离(精确脱靶量)
        d, cx, cy = closest_on_segment(tgt.x, tgt.y, px, py, self.x, self.y)
        if d < self.min_range:
            self.min_range = d
            self.min_point = (cx, cy)
            self.min_t = self.t
        self.range = math.hypot(tgt.x - self.x, tgt.y - self.y)

        # 5) 历史记录
        self.hist_t.append(self.t)
        self.hist_r.append(self.range)
        self.hist_a.append(self.a)
        self.hist_vc.append(vc)
        self.hist_losdot.append(los_dot)
        self.trail.append((self.x, self.y))

    # ------------------------------ 显示辅助 ------------------------------
    @property
    def heading_deg(self):
        return math.degrees(self.heading)

    @property
    def overload_g(self):
        """当前过载(单位 g)."""
        return self.a / G0

    @property
    def los_deg(self):
        return math.degrees(self.los)

    @property
    def los_dot_dps(self):
        """视线转率 (°/s)."""
        return math.degrees(self.los_dot)
