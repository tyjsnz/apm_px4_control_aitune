# -*- coding: utf-8 -*-
"""无界面数值自检: 跑若干组交战场景, 打印两种导引律的脱靶量.

用法: python selftest.py
"""

from __future__ import annotations

import sys

from simulation import SimConfig, Simulation


def run_case(title: str, **kw) -> tuple:
    cfg = SimConfig(**kw)
    sim = Simulation(cfg)
    sim.run(cfg.t_max)
    pn, pp = sim.missiles["pn"], sim.missiles["pp"]
    return title, pn, pp, sim


def main() -> int:
    cases = [
        ("默认: 交叉目标(直线)", dict()),
        ("目标左转 8°/s", dict(t_mode="left", t_turn=8.0)),
        ("目标右转 8°/s", dict(t_mode="right", t_turn=8.0)),
        ("目标正弦机动", dict(t_mode="sine", t_turn=10.0, t_period=3.0)),
        ("大离轴: 目标航向 260°", dict(t_heading=260.0)),
        ("追击: 目标同向逃逸", dict(t_heading=40.0, tx=6000.0, ty=1500.0)),
        ("头对头迎击", dict(t_heading=220.0, tx=9000.0, ty=1500.0)),
        ("高速目标 400 m/s", dict(t_speed=400.0)),
        ("小场 6 km", dict(field_size=6000.0, tx=5000.0, ty=4000.0)),
        # ---- 分段制导: 中段(未截获) -> 末段(PN/PP) ----
        ("分段制导: PIP中段", dict(phased=True)),
        ("分段制导: 程序直飞+窄视场", dict(phased=True, mid_mode="straight",
                                        acq_fov=20.0)),
        ("分段制导: 截获距离 1.5 km", dict(phased=True, acq_range=1500.0)),
    ]

    print(f"{'场景':<26}{'导引律':<10}{'脱靶量':>10}{'峰值过载':>10}"
          f"{'飞行时间':>10}{'截获':>18}  结果")
    print("-" * 112)

    bad = []
    for title, kw in cases:
        _, pn, pp, sim = run_case(title, **kw)
        for m in (pn, pp):
            peak = max((abs(a) for a in m.hist_a), default=0.0)
            miss = m.min_range
            hit = miss <= sim.cfg.hit_radius
            if not m.params.phased:
                acq = "-"
            elif m.acquired:
                acq = f"t={m.acq_t:.2f}s R={m.acq_r / 1000:.2f}km"
            else:
                acq = "未截获"
            print(f"{title:<26}{m.name:<10}{miss:9.1f} m{peak / 9.80665:9.1f} g"
                  f"{m.t:9.1f} s {acq:>17}  {m.result}")
            if m.done and miss != miss:      # NaN 检查
                bad.append((title, m.name, "NaN"))
            if m.done and miss > 1e6:
                bad.append((title, m.name, "min_range 未更新"))
            if m.params.phased and m.done and m.phase not in ("中段", "末段"):
                bad.append((title, m.name, "阶段状态异常"))
        print("-" * 112)

    if bad:
        print("异常:", bad)
        return 1
    print("自检通过: 所有场景均正常收敛, 无 NaN/未初始化状态.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
