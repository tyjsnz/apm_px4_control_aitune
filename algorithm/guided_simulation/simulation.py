# -*- coding: utf-8 -*-
"""仿真引擎: 装配目标与导弹、按固定步长推进、判定命中/脱靶."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from guidance import Missile, MissileParams, Target


@dataclass
class SimConfig:
    """仿真配置(界面所有可调参数都在这里, 修改后整体 reset)."""

    # ---- 仿真总体 ----
    mode: str = "both"              # "pn" | "pp" | "both"
    field_size: float = 10000.0     # 战场边长 (m), 正方形
    t_max: float = 60.0             # 最长仿真时间 (s)
    flyover_range: float = 2000.0   # 判定"飞越目标"的距离门限 (m)
    flyover_open_time: float = 0.3  # 持续远离超过该时间即判定飞越 (s)

    # ---- 导弹初始状态 ----
    mx: float = 800.0
    my: float = 1500.0
    m_heading: float = 40.0         # 初始航向 (deg)
    m_speed: float = 600.0          # 导弹速度 (m/s, 假定常值)

    # ---- 导弹制导参数 ----
    nav_ratio: float = 4.0          # N 比例导引导航比
    pp_gain: float = 4.0            # K 纯跟踪增益 (1/s)
    max_g: float = 9.0              # 最大可用过载 (g)
    tau: float = 0.3                # 执行机构时间常数 (s)
    use_vc: bool = True             # PN 系数中使用接近速度 Vc

    # ---- 目标初始状态 ----
    tx: float = 8500.0
    ty: float = 6000.0
    t_heading: float = 200.0        # 初始航向 (deg)
    t_speed: float = 250.0          # 目标速度 (m/s)

    # ---- 目标机动 ----
    t_mode: str = "straight"        # straight | left | right | sine
    t_turn: float = 6.0             # 转弯率 / 正弦幅度 (deg/s)
    t_period: float = 4.0           # 正弦机动周期 (s)

    # ---- 命中判定 ----
    hit_radius: float = 25.0        # 战斗部有效杀伤半径 (m)


class Simulation:
    """一次交战仿真: 一个目标 + 一到两枚导弹(比例导引 / 纯跟踪)."""

    DT = 0.02  # 仿真步长 (s)

    def __init__(self, cfg: SimConfig | None = None):
        self.cfg = cfg if cfg is not None else SimConfig()
        self.t = 0.0
        self.finished = False
        self.target: Target | None = None
        self.missiles: dict[str, Missile] = {}
        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        """按当前 cfg 重新构建目标与导弹."""
        c = self.cfg
        self.t = 0.0
        self.finished = False
        self.target = Target(c.tx, c.ty, c.t_heading, c.t_speed,
                             c.t_mode, c.t_turn, c.t_period)
        params = MissileParams(nav_ratio=c.nav_ratio, pp_gain=c.pp_gain,
                               max_g=c.max_g, tau=c.tau, use_vc=c.use_vc)
        self.missiles = {
            "pn": Missile("pn", "比例导引", "#ff9f43",
                          c.mx, c.my, c.m_heading, c.m_speed, params),
            "pp": Missile("pp", "纯跟踪", "#ff5ad1",
                          c.mx, c.my, c.m_heading, c.m_speed, params),
        }
        # 预置初始观测量, 保证未开始时界面数据显示正确
        for m in self.missiles.values():
            r, los, los_dot, vc = m.sense(self.target)
            m.range, m.los, m.los_dot, m.vc = r, los, los_dot, vc

    # ------------------------------------------------------------------
    @property
    def active_keys(self) -> list[str]:
        if self.cfg.mode == "pn":
            return ["pn"]
        if self.cfg.mode == "pp":
            return ["pp"]
        return ["pn", "pp"]

    @property
    def active(self) -> list[Missile]:
        return [self.missiles[k] for k in self.active_keys]

    # ------------------------------------------------------------------
    def step(self) -> bool:
        """推进一个 DT. 返回是否仍在运行."""
        if self.finished:
            return False
        c = self.cfg
        self.target.step(self.DT)

        for m in self.active:
            if m.done:
                continue
            m.step(self.target, self.DT)

            # 接近/远离状态
            if m.vc > 0.0:
                m.closed_once = True
                m.open_t = 0.0
            else:
                m.open_t += self.DT

            # 命中只打标记: 继续积分到"飞越目标"为止,
            # 这样 min_range 才是整条弹道上的真实最小距离(脱靶量).
            if m.min_range <= c.hit_radius:
                m.hit = True

            passed = (m.closed_once
                      and m.open_t >= c.flyover_open_time
                      and m.range <= max(c.flyover_range, 1.5 * m.min_range))
            if passed:
                m.done = True
                m.result = self._result(m, "飞越目标")
            elif m.t >= c.t_max:
                m.done = True
                m.result = self._result(m, "超时")

        self.t += self.DT
        if all(m.done for m in self.active):
            self.finished = True
        return not self.finished

    def run(self, seconds: float):
        """原地推进 seconds 秒(或直到仿真结束), 用于无界面/自动化测试."""
        t_end = self.t + seconds
        while not self.finished and self.t < t_end:
            self.step()

    # ------------------------------------------------------------------
    @staticmethod
    def _result(m, why: str) -> str:
        if m.hit:
            return f"命中 (脱靶量 {m.min_range:.1f} m)"
        return f"脱靶 {m.min_range:.1f} m ({why})"

    @property
    def running(self) -> bool:
        """是否处于交战进行中(已开始且未结束)."""
        return (not self.finished) and any(m.t > 0.0 for m in self.active)

    def summary(self) -> str:
        parts = []
        for m in self.active:
            parts.append(f"{m.name}: {m.result}")
        return "   |   ".join(parts)
