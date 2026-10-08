#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""比例导引(PN)三维拦截仿真：通过位置坐标解算比例导引并输出航迹与报告。

场景：地面端发射拦截机，拦截空中目标（经纬度 + 高度，三维 ENU）。
     每个制导周期只依赖“位置坐标”：由相邻周期位置差解算视线角速率与接近速度，
     按比例导引律产生横向过载指令，积分得到拦截航迹；
     再对输出轨迹用中心差分独立反解，验证 N'（验证“由位置坐标解算”闭环）。

比例导引律（三维矢量式，等价于标量式 a = N'·Vc·λ̇）：
    r  = 目标位置 − 自机位置          （视线矢量, ENU 3D）
    ω  = (r × ṙ) / |r|²              （视线角速率矢量, rad/s）
    Vc = −ṙ · r̂                      （接近速度, m/s）
    ac = N' · Vc · (ω × r̂)           （过载指令, m/s²，垂直于视线）
    反解：N' = a⊥ / (Vc · ω)          （a⊥ 由轨迹曲率中心差分得到）

用法：
    python pn_guidance_sim.py                                （交互/默认想定）
    python pn_guidance_sim.py --n 5 --target-turn 3          （改导航比/目标机动）
    python pn_guidance_sim.py --track pn_track.csv           （反解已有位置轨迹）

参数：
    --launch        地面端发射点 "纬度,经度,高度m"，默认 39.905,116.409,0
    --target        空中目标初始点 "纬度,经度,高度m"，默认 39.970,116.480,3000
    --own-speed     拦截机速度 m/s，默认 400
    --target-speed  目标速度 m/s，默认 150
    --target-hdg    目标航向 度(0=正北)，默认 120
    --target-climb  目标爬升角 度，默认 0
    --target-turn   目标转弯率 度/秒(正=右转)，默认 0
    --n             导航比 N'，默认 4（常用 3~5）
    --nmax          过载限幅，单位 g，默认 9
    --dt            制导周期 秒，默认 0.05 (20 Hz)
    --tmax          最大仿真时长 秒，默认 120
    --tau           自驾驶仪一阶时间常数 秒，0=理想，默认 0
    --init-hdg      拦截机初始方位 度，缺省=对准初始视线
    --track         反解模式：读取位置轨迹 CSV（t,own_lat,...,target_alt）
    --outdir        输出目录，缺省为脚本所在目录
    --show-steps    控制台打印周期采样表

输出（默认写到脚本所在目录）：
    pn_guidance_report.html   仿真报告：算法说明、轨迹图、时间历程、反解验证
    pn_track.csv              逐周期数据：双方位置 + 解算出的视线/过载/N'
    pn_result.json            参数 + 结果摘要 + 全部周期数据

仅依赖 Python 标准库，不依赖本项目其它文件。
命中判据：最近会遇距离（脱靶量）≤ 10 m。
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sys
import unicodedata
from datetime import datetime
from pathlib import Path

EARTH_R = 6371008.8   # 平均地球半径(米)
G0 = 9.80665          # 重力加速度(米/秒²)
HIT_R = 10.0          # 命中判据：脱靶量 <= 10 m


class PNError(RuntimeError):
    """仿真或解算失败。"""


# ========================= 地理坐标工具 =========================
def haversine(lat1, lon1, lat2, lon2):
    """两点大圆距离(米)。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


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


# ========================= 三维矢量工具 =========================
def v3_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v3_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v3_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def v3_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v3_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def v3_norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def v3_unit(a):
    n = v3_norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def vel_from_hdg(speed, hdg_deg, fpa_deg):
    """速度大小 + 航向(度,0=北,顺时针) + 航迹倾角(度,正=爬升) -> ENU 速度矢量。"""
    h = math.radians(hdg_deg)
    f = math.radians(fpa_deg)
    vh = speed * math.cos(f)
    return (vh * math.sin(h), vh * math.cos(h), speed * math.sin(f))


def hdg_fpa_from_vel(v):
    """ENU 速度矢量 -> (航向度, 航迹倾角度)。"""
    h = math.hypot(v[0], v[1])
    return ((math.degrees(math.atan2(v[0], v[1])) + 360.0) % 360.0,
            math.degrees(math.atan2(v[2], h)))


# ========================= 比例导引解算 =========================
def solve_pn(r, r_prev, dt, nav):
    """由相邻两周期的视线矢量(位置坐标差)解算比例导引。

    r      当前视线矢量 (目标 - 自机)
    r_prev 上一周期视线矢量
    返回 dict: R, az, el(度), los_rate(rad/s), vc(m/s), a_cmd(m/s² 矢量与模值)
    """
    R = v3_norm(r)
    if R < 1e-6:
        raise PNError("目标与自机重合，无法解算视线")
    u = v3_mul(r, 1.0 / R)
    rdot = v3_mul(v3_sub(r, r_prev), 1.0 / dt)      # 视线矢量变化率(由位置差得到)
    om = v3_mul(v3_cross(r, rdot), 1.0 / (R * R))   # 视线角速率矢量 ω
    vc = -v3_dot(rdot, u)                           # 接近速度
    lr = v3_norm(om)
    az = (math.degrees(math.atan2(r[0], r[1])) + 360.0) % 360.0
    el = math.degrees(math.asin(max(-1.0, min(1.0, u[2]))))
    if vc <= 0.0:
        a_cmd = (0.0, 0.0, 0.0)                     # 远离目标时不制导
    else:
        a_cmd = v3_mul(v3_cross(om, u), nav * vc)   # ac = N'·Vc·(ω × r̂)
    return {"R": R, "az": az, "el": el, "los": u, "los_rate": lr,
            "vc": vc, "a_cmd": a_cmd, "a_mag": v3_norm(a_cmd)}


def simulate(c):
    """前向仿真：位置 -> 视线解算 -> 过载指令 -> 积分位置。

    c: 参数 dict。返回 (steps, summary)。
    steps 每周期一行 dict（经纬高 + 几何/指令量）。
    """
    lat0, lon0 = c["launch"][0], c["launch"][1]
    lx, ly = geo_to_enu(lat0, lon0, lat0, lon0)
    p_o = (lx, ly, c["launch"][2])                                   # 自机(地面端发射)
    tx, ty = geo_to_enu(c["target"][0], c["target"][1], lat0, lon0)
    p_t = (tx, ty, c["target"][2])                                   # 空中目标

    r0 = v3_sub(p_t, p_o)
    horiz0 = math.hypot(r0[0], r0[1])
    if horiz0 < 1.0:
        raise PNError("发射点与目标水平投影过近(<1 m)，无法构成拦截几何")
    az0 = (math.degrees(math.atan2(r0[0], r0[1])) + 360.0) % 360.0
    el0 = math.degrees(math.atan2(r0[2], horiz0))
    hdg0 = az0 if c["init_hdg"] is None else c["init_hdg"]
    fpa0 = el0  # 俯仰默认取初始视线仰角；方位可由 --init-hdg 覆盖

    v_o = vel_from_hdg(c["own_speed"], hdg0, fpa0)
    v_t = vel_from_hdg(c["tgt_speed"], c["tgt_hdg"], c["tgt_climb"])

    # 上一周期视线种子：用相对速度回推一步，使首个周期即可由位置差解算
    r_prev = v3_sub(r0, v3_mul(v3_sub(v_t, v_o), c["dt"]))
    a_lag = [0.0, 0.0, 0.0]                     # 自驾驶仪输出(一阶延迟状态)

    steps = []
    t = 0.0
    k_max = int(round(c["tmax"] / c["dt"]))
    min_R, min_t, min_row = float("inf"), 0.0, 0
    closing_seen, passed = False, 0
    sat_steps, guide_steps = 0, 0
    a_cmd_max, a_used_max = 0.0, 0.0
    hit_seen = False

    for k in range(k_max + 1):
        r = v3_sub(p_t, p_o)
        R = v3_norm(r)
        if R > 1e-9:
            az = (math.degrees(math.atan2(r[0], r[1])) + 360.0) % 360.0
            el = math.degrees(math.asin(max(-1.0, min(1.0, r[2] / R))))
        else:  # 与目标几何重合（极少见）
            az = steps[-1]["az"] if steps else 0.0
            el = steps[-1]["el"] if steps else 0.0

        if hit_seen:
            # 命中已达成 -> 惯性滑飞：仅用于插值精化脱靶量，不再制导、不计统计
            a_cur = (0.0, 0.0, 0.0)
            lr = vc = a_cmd_mag = a_used = None
            sat_flag = None
        else:
            sol = solve_pn(r, r_prev, c["dt"], c["n"])
            # 指令：方向投影到 ⊥速度平面、大小保持教科书标量式 N'·Vc·λ̇
            vh = v3_unit(v_o)
            cmd = sol["a_cmd"]
            d = v3_sub(cmd, v3_mul(vh, v3_dot(cmd, vh)))
            dn = v3_norm(d)
            if dn > 1e-9:
                a_perp = v3_mul(d, sol["a_mag"] / dn)   # |a| = N'·Vc·|ω|，方向 ⊥ 速度
                a_mag = sol["a_mag"]
            else:
                a_perp = (0.0, 0.0, 0.0)
                a_mag = 0.0
            # 过载限幅
            sat = a_mag > c["amax"]
            if sat:
                a_perp = v3_mul(a_perp, c["amax"] / a_mag)
                a_used = c["amax"]
                sat_steps += 1
            else:
                a_used = a_mag
            # 自驾驶仪一阶延迟（可选）
            if c["tau"] > 0.0:
                alpha = c["dt"] / c["tau"]
                for i in range(3):
                    a_lag[i] += (a_perp[i] - a_lag[i]) * alpha
                a_cur = (a_lag[0], a_lag[1], a_lag[2])
                a_used = v3_norm(a_cur)
            else:
                a_cur = a_perp
            guide_steps += 1
            a_cmd_max = max(a_cmd_max, sol["a_mag"])
            a_used_max = max(a_used_max, a_used)
            lr = sol["los_rate"] * 180.0 / math.pi
            vc = sol["vc"]
            a_cmd_mag = sol["a_mag"]
            sat_flag = 1 if sat else 0

        # 记录本周期状态（先记录、后判定、再积分）
        og = enu_to_geo(p_o[0], p_o[1], lat0, lon0)
        tg = enu_to_geo(p_t[0], p_t[1], lat0, lon0)
        steps.append({
            "t": t,
            "own_lat": og[0], "own_lon": og[1], "own_alt": p_o[2],
            "target_lat": tg[0], "target_lon": tg[1], "target_alt": p_t[2],
            "x": p_o[0], "y": p_o[1], "tx": p_t[0], "ty": p_t[1],
            "R": R, "az": az, "el": el,
            "vc": vc, "lr": lr,
            "a_cmd": a_cmd_mag, "a_used": a_used,
            "sat": sat_flag,
            "own_hdg": hdg_fpa_from_vel(v_o)[0],
            "own_fpa": hdg_fpa_from_vel(v_o)[1],
        })

        # 最近会遇跟踪
        if R < min_R:
            min_R, min_t, min_row = R, t, k
            closing_seen, passed = True, 0
        elif closing_seen and R > min_R + 1e-6:
            passed += 1
        if not hit_seen and min_R <= HIT_R:
            hit_seen = True               # 命中达成 -> 后续周期惯性滑飞

        if passed >= 10 and t > 1.0:      # 距离连续增大 -> 已过最近会遇点
            break
        if k == k_max:
            break

        # 积分：自机(等速 + 横向过载) / 目标(航向转弯率)
        p_o = v3_add(p_o, v3_add(v3_mul(v_o, c["dt"]),
                                 v3_mul(a_cur, 0.5 * c["dt"] * c["dt"])))
        v_new = v3_add(v_o, v3_mul(a_cur, c["dt"]))
        spd = v3_norm(v_new)
        if spd > 1e-9:
            v_o = v3_mul(v_new, c["own_speed"] / spd)   # 速度矢量重新归一到指定速率
        hdg_t = c["tgt_hdg"] + c["tgt_turn"] * (t + 0.5 * c["dt"])
        v_t = vel_from_hdg(c["tgt_speed"], hdg_t, c["tgt_climb"])
        p_t = v3_add(p_t, v3_mul(v_t, c["dt"]))
        r_prev = r
        t += c["dt"]

    if min_row >= len(steps):
        min_row = len(steps) - 1
    # 脱靶量精化：采样步长≈Vc·dt(可达十余米)会高估最近距离，
    # 用最邻近三点做抛物线插值取顶点（R(t) 在最近会遇附近近似二次）
    if 0 < min_row < len(steps) - 1:
        ra = steps[min_row - 1]["R"]
        rb = steps[min_row]["R"]
        rc = steps[min_row + 1]["R"]
        A = 0.5 * (ra - 2.0 * rb + rc)
        B = 0.5 * (rc - ra)
        if A > 1e-9:
            tau = -B / (2.0 * A)
            if abs(tau) <= 1.0:
                miss = rb - B * B / (4.0 * A)
                if 0.0 <= miss <= rb:
                    min_R = miss
                    min_t = steps[min_row]["t"] + tau * c["dt"]
    # 交会量避开末端数值噪声区：
    #  Vc = 交会前 ≥0.2 s 的真实距离差分（末端视线快速旋转时单侧差分不可靠）
    #  λ̇ = 最近会遇前 R≥100 m 的最后一个状态行
    i0 = min_row
    while i0 > 0 and steps[min_row]["t"] - steps[i0]["t"] < 0.2:
        i0 -= 1
    win = steps[min_row]["t"] - steps[i0]["t"]
    vc_close = -(steps[min_row]["R"] - steps[i0]["R"]) / win if win > 0 else 0.0
    close_row = 0
    for i in range(min_row, -1, -1):
        if steps[i]["R"] >= 100.0:
            close_row = i
            break
    lr_close = steps[close_row]["lr"]
    summary = {
        "intercepted": bool(min_R <= HIT_R),
        "miss_m": min_R,
        "t_close": min_t,
        "t_sim": steps[-1]["t"],
        "steps": len(steps),
        "vc_close": vc_close,
        "lr_close": lr_close,
        "a_cmd_max": a_cmd_max,
        "a_used_max": a_used_max,
        "sat_ratio": sat_steps / max(1, guide_steps),
        "sat_steps": sat_steps,
        "init_az": az0, "init_el": el0,
        "init_slant": v3_norm(r0), "init_horiz": horiz0,
        "init_dalt": c["target"][2] - c["launch"][2],
    }
    return steps, summary


def solve_track(times, own, tgt, den_min=0.05, r_min=20.0):
    """反解：由纯位置序列(中心差分)解算比例导引量。

    返回与输入等长的列表，每项 dict：
      R/az/el（全部点）、los_rate(deg/s)/vc/a_lat(横向过载)/n_est（可差分点）。
    N' 估计 = a⊥ / (Vc·ω)，仅在 Vc·ω 与距离满足阈值时给出。
    """
    n = len(times)
    if n < 5:
        raise PNError(f"轨迹点太少({n} 点)，至少需要 5 点才能中心差分")

    def dtc(k):
        return times[k + 1] - times[k - 1]

    # 速度（中心差分）
    v_o, v_t = [None] * n, [None] * n
    for k in range(1, n - 1):
        d = dtc(k)
        if d > 0:
            v_o[k] = v3_mul(v3_sub(own[k + 1], own[k - 1]), 1.0 / d)
            v_t[k] = v3_mul(v3_sub(tgt[k + 1], tgt[k - 1]), 1.0 / d)

    # 自机加速度（对速度再中心差分）
    a_o = [None] * n
    for k in range(2, n - 2):
        d = dtc(k)
        if d > 0 and v_o[k + 1] is not None and v_o[k - 1] is not None:
            a_o[k] = v3_mul(v3_sub(v_o[k + 1], v_o[k - 1]), 1.0 / d)

    res = [None] * n
    for k in range(n):
        r = v3_sub(tgt[k], own[k])
        R = v3_norm(r)
        if R < 1e-6:
            continue
        u = v3_mul(r, 1.0 / R)
        az = (math.degrees(math.atan2(r[0], r[1])) + 360.0) % 360.0
        el = math.degrees(math.asin(max(-1.0, min(1.0, u[2]))))
        item = {"R": R, "az": az, "el": el,
                "lr": None, "vc": None, "a_lat": None, "n_est": None}
        if 1 <= k <= n - 2:
            d = dtc(k)
            if d > 0:
                rdot = v3_mul(v3_sub(v3_sub(tgt[k + 1], own[k + 1]),
                                     v3_sub(tgt[k - 1], own[k - 1])), 1.0 / d)
                om = v3_mul(v3_cross(r, rdot), 1.0 / (R * R))
                lr = v3_norm(om)
                vc = -v3_dot(rdot, u)
                item["lr"] = lr * 180.0 / math.pi
                item["vc"] = vc
                if a_o[k] is not None and v_o[k] is not None:
                    vh = v3_unit(v_o[k])
                    a_lat = v3_sub(a_o[k], v3_mul(vh, v3_dot(a_o[k], vh)))
                    am = v3_norm(a_lat)
                    item["a_lat"] = am
                    den = vc * lr
                    if den >= den_min and vc > 0.0 and R >= r_min:
                        item["n_est"] = am / den
        res[k] = item
    return res


def n_stats(values, configured=None):
    """N' 估计序列 -> 统计 dict。"""
    xs = sorted(v for v in values if v is not None and math.isfinite(v))
    if not xs:
        return {"count": 0}
    m = len(xs)
    med = xs[m // 2] if m % 2 else 0.5 * (xs[m // 2 - 1] + xs[m // 2])
    mean = sum(xs) / m
    var = sum((x - mean) ** 2 for x in xs) / m
    q1 = xs[m // 4]
    q3 = xs[(3 * m) // 4]
    out = {"count": m, "median": med, "mean": mean, "std": math.sqrt(var),
           "min": xs[0], "max": xs[-1], "q1": q1, "q3": q3}
    if configured:
        out["cfg"] = configured
        out["bias_pct"] = (med - configured) / configured * 100.0
        out["ok"] = abs(out["bias_pct"]) <= 5.0
    return out


def attach_inverse(steps, own, tgt, times):
    """把反解结果并入 steps 行（供图表与 CSV 使用）。own/tgt 须为 ENU 米制。
    仅回填制导周期行（命中后滑飞行 lr 为 None，不参与图表/统计）。"""
    res = solve_track(times, own, tgt)
    for row, d in zip(steps, res):
        if not d:
            continue
        row["R"] = d["R"]
        row["az"], row["el"] = d["az"], d["el"]
        if d["lr"] is not None and row.get("lr") is not None:
            row["lr"] = d["lr"]
            row["vc"] = d["vc"]
        if d["a_lat"] is not None and row.get("lr") is not None:
            row["a_lat"] = d["a_lat"]
        if d["n_est"] is not None and row.get("lr") is not None:
            row["n_est"] = d["n_est"]
    if steps and steps[-1].get("lr") is not None:
        # 末点为中心差分不可用处，单侧差分在 R≈数米时因视线快速旋转而失真 -> 置空
        steps[-1]["lr"] = None
        steps[-1]["vc"] = None
    return steps


def load_track(path):
    """读取位置轨迹 CSV（反解模式）。必需列：t, own_lat, own_lon, target_lat, target_lon。
    高度列 own_alt/target_alt 可选（缺省 0）。"""
    p = Path(path)
    if not p.exists():
        raise PNError(f"轨迹文件不存在: {p}")
    times, own, tgt = [], [], []
    with p.open("r", encoding="utf-8-sig", newline="") as f:
        rd = csv.DictReader(f)
        if not rd.fieldnames:
            raise PNError("轨迹文件为空")
        cols = {c.strip().lower(): c for c in rd.fieldnames}
        def col(*names):
            for nm in names:
                if nm in cols:
                    return cols[nm]
            return None
        c_t = col("t", "time")
        c_ol, c_oo = col("own_lat"), col("own_lon")
        c_tl, c_to = col("target_lat", "tgt_lat"), col("target_lon", "tgt_lon")
        if not all((c_t, c_ol, c_oo, c_tl, c_to)):
            raise PNError("轨迹 CSV 需要列: t, own_lat, own_lon, target_lat, target_lon")
        c_oa, c_ta = col("own_alt"), col("target_alt", "tgt_alt")
        for i, row in enumerate(rd):
            try:
                t = float(row[c_t])
                o = (float(row[c_ol]), float(row[c_oo]),
                     float(row[c_oa]) if c_oa and row.get(c_oa) not in (None, "") else 0.0)
                g = (float(row[c_tl]), float(row[c_to]),
                     float(row[c_ta]) if c_ta and row.get(c_ta) not in (None, "") else 0.0)
            except (TypeError, ValueError):
                raise PNError(f"轨迹文件第 {i + 2} 行数值解析失败")
            times.append(t)
            own.append(o)
            tgt.append(g)
    if len(times) >= 2 and times[0] > times[-1]:
        order = sorted(range(len(times)), key=lambda i: times[i])
        times = [times[i] for i in order]
        own = [own[i] for i in order]
        tgt = [tgt[i] for i in order]
    for a, b in zip(times, times[1:]):
        if b - a <= 0:
            raise PNError("轨迹时间列必须严格递增")
    return times, own, tgt


def track_to_rows(times, own, tgt, res):
    """反解模式：由位置序列构造与仿真一致的行（xy 用首点为原点）。"""
    lat0, lon0 = own[0][0], own[0][1]
    rows = []
    for k, t in enumerate(times):
        ox, oy = geo_to_enu(own[k][0], own[k][1], lat0, lon0)
        gx, gy = geo_to_enu(tgt[k][0], tgt[k][1], lat0, lon0)
        d = res[k] or {}
        rows.append({
            "t": t,
            "own_lat": own[k][0], "own_lon": own[k][1], "own_alt": own[k][2],
            "target_lat": tgt[k][0], "target_lon": tgt[k][1], "target_alt": tgt[k][2],
            "x": ox, "y": oy, "tx": gx, "ty": gy,
            "R": d.get("R"), "az": d.get("az"), "el": d.get("el"),
            "lr": d.get("lr"), "vc": d.get("vc"),
            "a_cmd": None, "a_used": None, "sat": None,
            "a_lat": d.get("a_lat"), "n_est": d.get("n_est"),
        })
    return rows


# ========================= SVG 图表 =========================
def nice_step(span):
    """1/2/5×10^k 刻度步长。"""
    if span <= 0:
        return 1.0
    raw = span / 6.0
    mag = 10.0 ** math.floor(math.log10(raw))
    for m in (1.0, 2.0, 5.0, 10.0):
        if raw <= m * mag:
            return m * mag
    return 10.0 * mag


def _decimals(step):
    if step >= 1:
        return 0
    if step >= 0.1:
        return 1
    if step >= 0.01:
        return 2
    if step >= 0.001:
        return 3
    return 4


def downsample(pts, nmax=600):
    if len(pts) <= nmax:
        return pts
    stride = int(math.ceil(len(pts) / float(nmax)))
    out = pts[::stride]
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def _legend(series, x, y, bw=None):
    if len(series) < 2:
        return []
    if bw is None:
        bw = max(_text_w(s["name"]) for s in series) + 52
    bh = 10 + 18 * len(series)
    out = [f'<rect x="{x}" y="{y}" width="{bw:.0f}" height="{bh:.0f}" '
           f'fill="#ffffff" fill-opacity="0.9" stroke="#e2e8f0" rx="4"/>']
    for i, s in enumerate(series):
        yy = y + 14 + i * 18
        dash = ' stroke-dasharray="6 4"' if s.get("dash") else ""
        out.append(f'<line x1="{x + 8}" y1="{yy}" x2="{x + 34}" y2="{yy}" '
                   f'stroke="{s["color"]}" stroke-width="3"{dash}/>')
        out.append(f'<text x="{x + 40}" y="{yy + 4}" font-size="12" '
                   f'fill="#334155">{html.escape(s["name"])}</text>')
    return out


def _text_w(s, size=12.0):
    """粗略文本宽度(CJK 记 2)。"""
    return sum(2.0 if unicodedata.east_asian_width(ch) in "WFA" else 1.0
               for ch in s) * size


def svg_plot(title, xlabel, ylabel, series, hlines=(), width=880, height=330):
    """通用折线图（时程曲线）。hlines: [(值, '标签')] 参考水平线。"""
    pts_all = [p for s in series for p in s["pts"] if p[1] is not None]
    if not pts_all:
        return "<p class='muted'>(无数据)</p>"
    xs = [p[0] for p in pts_all]
    ys = [p[1] for p in pts_all]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    for v, _ in hlines:
        miny, maxy = min(miny, v), max(maxy, v)
    if maxx - minx < 1e-9:
        minx, maxx = minx - 0.5, maxx + 0.5
    if maxy - miny < 1e-9:
        miny, maxy = miny - 1.0, maxy + 1.0
    pad_y = (maxy - miny) * 0.06
    miny -= pad_y
    maxy += pad_y

    px0, px1 = 66.0, width - 14.0
    py0, py1 = 26.0, height - 40.0

    def X(x):
        return px0 + (x - minx) / (maxx - minx) * (px1 - px0)

    def Y(y):
        return py1 - (y - miny) / (maxy - miny) * (py1 - py0)

    out = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
           f'class="chart"><rect width="{width}" height="{height}" fill="#ffffff" '
           f'stroke="#e2e8f0"/>']
    sx = nice_step(maxx - minx)
    dx = _decimals(sx)
    gx = math.ceil(minx / sx) * sx
    while gx <= maxx + 1e-9:
        out.append(f'<line x1="{X(gx):.1f}" y1="{py0}" x2="{X(gx):.1f}" '
                   f'y2="{py1}" stroke="#eef2f7"/>')
        out.append(f'<text x="{X(gx):.1f}" y="{py1 + 16}" font-size="10" '
                   f'fill="#94a3b8" text-anchor="middle">{gx:.{dx}f}</text>')
        gx += sx
    sy = nice_step(maxy - miny)
    dy = _decimals(sy)
    gy = math.ceil(miny / sy) * sy
    while gy <= maxy + 1e-9:
        out.append(f'<line x1="{px0}" y1="{Y(gy):.1f}" x2="{px1}" y2="{Y(gy):.1f}" '
                   f'stroke="#eef2f7"/>')
        out.append(f'<text x="{px0 - 6}" y="{Y(gy) + 3.5:.1f}" font-size="10" '
                   f'fill="#94a3b8" text-anchor="end">{gy:.{dy}f}</text>')
        gy += sy
    out.append(f'<line x1="{px0}" y1="{py1}" x2="{px1}" y2="{py1}" stroke="#cbd5e1"/>')
    out.append(f'<line x1="{px0}" y1="{py0}" x2="{px0}" y2="{py1}" stroke="#cbd5e1"/>')
    for v, lab in hlines:
        out.append(f'<line x1="{px0}" y1="{Y(v):.1f}" x2="{px1}" y2="{Y(v):.1f}" '
                   f'stroke="#f59e0b" stroke-dasharray="6 4" stroke-width="1.4"/>')
        out.append(f'<text x="{px1 - 4}" y="{Y(v) - 5:.1f}" font-size="10" '
                   f'fill="#b45309" text-anchor="end">{html.escape(lab)}</text>')
    for s in series:
        pts = [(x, y) for x, y in s["pts"] if y is not None]
        if not pts:
            continue
        poly = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in downsample(pts))
        dash = ' stroke-dasharray="7 5"' if s.get("dash") else ""
        out.append(f'<polyline points="{poly}" fill="none" stroke="{s["color"]}" '
                   f'stroke-width="2"{dash} stroke-linejoin="round"/>')
    out.append(f'<text x="{(px0 + px1) / 2:.0f}" y="{height - 8}" font-size="11" '
               f'fill="#64748b" text-anchor="middle">{html.escape(xlabel)}</text>')
    out.append(f'<text x="14" y="{(py0 + py1) / 2:.0f}" font-size="11" fill="#64748b" '
               f'text-anchor="middle" transform="rotate(-90 14 '
               f'{(py0 + py1) / 2:.0f})">{html.escape(ylabel)}</text>')
    out.extend(_legend(series, px1 - _legend_w(series) - 6, py0 + 4))
    out.append(f'<text x="{width - 10}" y="17" text-anchor="end" font-size="12" '
               f'fill="#64748b">{html.escape(title)}</text>')
    out.append("</svg>")
    return "".join(out)


def _legend_w(series):
    if len(series) < 2:
        return 0
    return max(_text_w(s["name"]) for s in series) + 52


def svg_track(rows, launch_xy, miss_xy, miss_m, hit, width=880, height=520):
    """拦截轨迹俯视图（北向朝上）。"""
    pts = []
    for r in rows:
        pts.append((r["x"], r["y"]))
        pts.append((r["tx"], r["ty"]))
    if not pts:
        return "<p class='muted'>(无轨迹)</p>"
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    if maxx - minx < 1:
        minx -= 1.0
        maxx += 1.0
    if maxy - miny < 1:
        miny -= 1.0
        maxy += 1.0
    pad = 56.0
    s = min((width - 2 * pad) / (maxx - minx), (height - 2 * pad) / (maxy - miny))
    ox = pad + ((width - 2 * pad) - (maxx - minx) * s) / 2
    oy = pad + ((height - 2 * pad) - (maxy - miny) * s) / 2

    def X(x):
        return ox + (x - minx) * s

    def Y(y):
        return height - (oy + (y - miny) * s)

    out = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
           f'class="chart"><rect width="{width}" height="{height}" fill="#ffffff" '
           f'stroke="#e2e8f0"/>']
    step = nice_step(maxx - minx)
    gx = math.ceil(minx / step) * step
    while gx <= maxx:
        out.append(f'<line x1="{X(gx):.1f}" y1="{Y(miny):.1f}" x2="{X(gx):.1f}" '
                   f'y2="{Y(maxy):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{X(gx):.1f}" y="{height - 12}" font-size="10" '
                   f'fill="#94a3b8" text-anchor="middle">{gx / 1000:.1f}km</text>')
        gx += step
    step_y = nice_step(maxy - miny)
    gy = math.ceil(miny / step_y) * step_y
    while gy <= maxy:
        out.append(f'<line x1="{X(minx):.1f}" y1="{Y(gy):.1f}" x2="{X(maxx):.1f}" '
                   f'y2="{Y(gy):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="6" y="{Y(gy) + 3:.1f}" font-size="10" '
                   f'fill="#94a3b8">{gy / 1000:.1f}km</text>')
        gy += step_y

    own = downsample([(r["x"], r["y"]) for r in rows])
    tgt = downsample([(r["tx"], r["ty"]) for r in rows])
    poly_t = " ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in tgt)
    out.append(f'<polyline points="{poly_t}" fill="none" stroke="#dc2626" '
               f'stroke-width="2" stroke-dasharray="7 5" stroke-linejoin="round"/>')
    poly_o = " ".join(f"{X(p[0]):.1f},{Y(p[1]):.1f}" for p in own)
    out.append(f'<polyline points="{poly_o}" fill="none" stroke="#2563eb" '
               f'stroke-width="2.4" stroke-linejoin="round"/>')
    # 发射点 / 目标初始点 / 脱靶点
    lx, ly = launch_xy
    out.append(f'<circle cx="{X(lx):.1f}" cy="{Y(ly):.1f}" r="6" fill="#16a34a" '
               f'stroke="#fff" stroke-width="2"/>')
    out.append(f'<text x="{X(lx) + 9:.1f}" y="{Y(ly) + 4:.1f}" font-size="12" '
               f'font-weight="bold" fill="#15803d">地面端(发射点)</text>')
    tx0, ty0 = tgt[0]
    out.append(f'<circle cx="{X(tx0):.1f}" cy="{Y(ty0):.1f}" r="6" fill="#dc2626" '
               f'stroke="#fff" stroke-width="2"/>')
    out.append(f'<text x="{X(tx0) + 9:.1f}" y="{Y(ty0) + 4:.1f}" font-size="12" '
               f'font-weight="bold" fill="#b91c1c">空中目标(初始)</text>')
    mx, my = miss_xy
    out.append(f'<circle cx="{X(mx):.1f}" cy="{Y(my):.1f}" r="4.5" fill="none" '
               f'stroke="#f59e0b" stroke-width="2.5"/>')
    lab = f"最近会遇 {miss_m:.1f} m" + ("(命中)" if hit else "(未命中)")
    out.append(f'<text x="{X(mx) + 8:.1f}" y="{Y(my) - 8:.1f}" font-size="12" '
               f'font-weight="bold" fill="#b45309">{lab}</text>')
    series = [{"name": "拦截机航迹", "color": "#2563eb"},
              {"name": "目标航迹", "color": "#dc2626", "dash": True}]
    out.extend(_legend(series, 10, 10))
    out.append(f'<text x="{width - 10}" y="20" text-anchor="end" font-size="12" '
               f'fill="#64748b">拦截轨迹俯视图（北向朝上, 平面投影）</text>')
    out.append("</svg>")
    return "".join(out)


# ========================= HTML 报告 =========================
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
.note{background:#fffbeb;border:1px solid #fde68a;border-radius:8px;padding:10px 14px;
 font-size:13px;line-height:1.8}
.ok{background:#ecfdf5;border:1px solid #6ee7b7;border-radius:8px;padding:12px 16px;
 font-size:14px;line-height:1.8}
.bad{background:#fef2f2;border:1px solid #fecaca;border-radius:8px;padding:12px 16px;
 font-size:14px;line-height:1.8}
table{border-collapse:collapse;width:100%;background:#fff;font-size:13px}
th,td{border:1px solid #e2e8f0;padding:7px 9px;text-align:right}
th{background:#f8fafc;text-align:center;font-weight:600}
td:first-child,th:first-child{text-align:left}
tr:nth-child(even) td{background:#fafcff}
.chart{width:100%;height:auto;display:block;margin:8px 0 4px;
 border-radius:8px;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.formula{background:#0f172a;color:#e2e8f0;border-radius:8px;padding:12px 16px;
 font-family:Consolas,Monaco,monospace;font-size:14px;line-height:2;
 white-space:pre-wrap;overflow-x:auto}
ol,ul{line-height:1.9;font-size:14px}
code{background:#f1f5f9;border-radius:3px;padding:1px 5px;font-size:12px}
"""


def _fmt(v, nd=2):
    return "—" if v is None else f"{v:.{nd}f}"


def build_report(cfg, mode, rows, summary, stats, stamp):
    """生成 HTML 报告字符串。mode: 'sim' | 'track'。"""
    esc = html.escape
    ts = [r["t"] for r in rows]

    # 图1：轨迹俯视
    if mode == "sim":
        o0 = rows[0]
        miss_row = rows[summary.get("min_row", len(rows) - 1)]
        chart_track = svg_track(rows, (o0["x"], o0["y"]),
                                (miss_row["x"], miss_row["y"]),
                                summary["miss_m"], summary["intercepted"])
    else:
        o0 = rows[0]
        chart_track = svg_track(rows, (o0["x"], o0["y"]),
                                (rows[-1]["x"], rows[-1]["y"]), 0.0, False)

    # 图2..6 时程曲线
    chart_alt = svg_plot("高度-时间", "t (s)", "高度 (m)", [
        {"name": "拦截机", "color": "#2563eb",
         "pts": [(r["t"], r["own_alt"]) for r in rows]},
        {"name": "目标", "color": "#dc2626", "dash": True,
         "pts": [(r["t"], r["target_alt"]) for r in rows]},
    ])
    chart_rng = svg_plot("距离-时间（最近会遇=脱靶量）", "t (s)", "斜距 (m)", [
        {"name": "斜距", "color": "#7c3aed",
         "pts": [(r["t"], r["R"]) for r in rows]},
    ], hlines=[(HIT_R, f"命中判据 {HIT_R:.0f} m")] if mode == "sim" else [])
    chart_lr = svg_plot("视线角速率-时间", "t (s)", "λ̇ (°/s)", [
        {"name": "视线角速率", "color": "#0891b2",
         "pts": [(r["t"], r.get("lr")) for r in rows]},
    ])
    a_series = []
    if mode == "sim":
        a_series.append({"name": "过载指令 ac", "color": "#2563eb",
                         "pts": [(r["t"], r.get("a_cmd")) for r in rows]})
        a_series.append({"name": "实际过载(限幅后)", "color": "#16a34a", "dash": True,
                         "pts": [(r["t"], r.get("a_used")) for r in rows]})
        a_series.append({"name": "反解需用过载 a⊥", "color": "#9333ea",
                         "pts": [(r["t"], r.get("a_lat")) for r in rows]})
        a_lines = [(cfg["amax"], f"限幅 {cfg['nmax']:g}g = {cfg['amax']:.1f} m/s²")]
    else:
        a_series.append({"name": "反解需用过载 a⊥", "color": "#9333ea",
                         "pts": [(r["t"], r.get("a_lat")) for r in rows]})
        a_lines = []
    chart_a = svg_plot("横向过载-时间", "t (s)", "a (m/s²)", a_series, hlines=a_lines)
    n_series = [{"name": "N' 估计 = a⊥/(Vc·λ̇)", "color": "#ea580c",
                 "pts": [(r["t"], r.get("n_est")) for r in rows]}]
    n_lines = [(cfg["n"], f"配置 N' = {cfg['n']}")] if mode == "sim" else []
    chart_n = svg_plot("由位置坐标反解的导航比 N'", "t (s)", "N'", n_series,
                       hlines=n_lines)

    # 汇总表
    esc_ = esc
    if mode == "sim":
        verdict_cls = "ok" if summary["intercepted"] else "bad"
        verdict = ("命中（脱靶量 ≤ 10 m）" if summary["intercepted"]
                   else "未命中（脱靶量 &gt; 10 m）")
        rows_tbl = [
            ("最近会遇(脱靶量)", f"{summary['miss_m']:.2f} m", f"判据 ≤ {HIT_R:.0f} m"),
            ("拦截用时", f"{summary['t_close']:.2f} s", f"仿真 {summary['t_sim']:.1f} s / {summary['steps']} 周期"),
            ("末段接近速度 Vc", f"{summary['vc_close']:.1f} m/s",
             "交会前 ≥0.2 s 的真实距离差分"),
            ("交会前视线角速率", f"{summary['lr_close']:.4f} °/s",
             "R≥100 m 处解算；越小越接近纯碰撞航向"),
            ("初始斜距 / 水平 / 高差",
             f"{summary['init_slant'] / 1000:.3f} / {summary['init_horiz'] / 1000:.3f} / "
             f"{summary['init_dalt']:.0f} m",
             f"初始视线方位 {summary['init_az']:.1f}° 仰角 {summary['init_el']:.1f}°"),
            ("过载指令峰值", f"{summary['a_cmd_max']:.1f} m/s² "
                            f"({summary['a_cmd_max'] / G0:.1f} g)", "限幅前"),
            ("实际过载峰值", f"{summary['a_used_max']:.1f} m/s² "
                            f"({summary['a_used_max'] / G0:.1f} g)",
             f"限幅 {cfg['nmax']:g}g；饱和 {summary['sat_steps']} 周期 "
             f"({summary['sat_ratio'] * 100:.1f}%)"),
        ]
    else:
        verdict_cls = "ok"
        verdict = "轨迹反解完成（未做命中判定，如需请提供交会段轨迹）"
        rows_tbl = [
            ("轨迹点数 / 时长", f"{len(rows)} 点 / {ts[-1] - ts[0]:.1f} s",
             f"采样周期 ≈ {(ts[-1] - ts[0]) / max(1, len(rows) - 1):.3f} s"),
            ("斜距范围", f"{min(r['R'] for r in rows if r['R']):.0f} ~ "
                        f"{max(r['R'] for r in rows if r['R']):.0f} m", "由位置直接计算"),
        ]

    # N' 统计行
    if stats.get("count"):
        cfgv = stats.get("cfg")
        bias = (f"配置值 {cfgv} → 偏差 {stats['bias_pct']:+.2f}%"
                if cfgv else "外部轨迹（无配置值可比对）")
        n_tbl = [
            ("有效样本", f"{stats['count']}", "满足 Vc·λ̇ 与距离阈值的周期数"),
            ("N' 中位数", f"{stats['median']:.3f}", bias),
            ("均值 ± 标准差", f"{stats['mean']:.3f} ± {stats['std']:.3f}",
             f"四分位 [{stats['q1']:.3f}, {stats['q3']:.3f}]"),
            ("最小 ~ 最大", f"{stats['min']:.3f} ~ {stats['max']:.3f}", "端点差分噪声会拉大尾部"),
        ]
    else:
        n_tbl = [("有效样本", "0", "差分阈值过滤后无有效样本")]

    parts = [
        "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'>",
        "<meta name='viewport' content='width=device-width,initial-scale=1'>",
        f"<title>比例导引仿真报告 - {esc(stamp)}</title>",
        f"<style>{_CSS}</style></head><body><div class='wrap'>",
        "<h1>比例导引(PN)三维拦截仿真报告</h1>",
        f"<div class='muted'>生成时间 {esc(stamp)} ｜ 模式 "
        f"{'前向仿真 + 反解验证' if mode == 'sim' else '外部轨迹反解'}</div>",
        "<div class='meta'>",
    ]
    if mode == "sim":
        parts.append(
            f"<b>地面端发射点</b> ({cfg['launch'][0]:.6f}, {cfg['launch'][1]:.6f}, "
            f"{cfg['launch'][2]:.0f} m) ｜ <b>空中目标初始点</b> "
            f"({cfg['target'][0]:.6f}, {cfg['target'][1]:.6f}, {cfg['target'][2]:.0f} m)<br>"
            f"<b>自机速度</b> {cfg['own_speed']:.0f} m/s ｜ <b>目标</b> "
            f"{cfg['tgt_speed']:.0f} m/s、航向 {cfg['tgt_hdg']:.0f}°、"
            f"爬升 {cfg['tgt_climb']:.0f}°、转弯率 {cfg['tgt_turn']:.1f} °/s<br>"
            f"<b>导航比 N'</b> {cfg['n']:.1f} ｜ <b>过载限幅</b> {cfg['nmax']:g}g "
            f"({cfg['amax']:.1f} m/s²) ｜ <b>制导周期</b> {cfg['dt'] * 1000:.0f} ms ｜ "
            f"<b>自动驾驶仪 τ</b> {cfg['tau']:.2f} s ｜ "
            f"<b>初始方位</b> {cfg['init_hdg'] if cfg['init_hdg'] is not None else '对准视线'}<br>")
    else:
        parts.append(
            f"<b>轨迹文件</b> {esc(cfg['track_file'])} ｜ <b>点数</b> {len(rows)}<br>")
    parts.append(
        f"<b>场景</b> 地面端 → 空中目标（三维：经纬度 + 高度；质点运动学，"
        f"忽略重力与气动延迟；每个制导周期仅由位置坐标解算视线）｜ "
        f"<b>命中判据</b> 脱靶量 ≤ {HIT_R:.0f} m</div>")

    # 1. 算法说明
    parts.append(
        "<h2>1. 比例导引算法（功能与描述）</h2>"
        "<div class='meta'><b>功能：</b>依据与目标的相对位置（视线）角速率，"
        "按比例关系产生横向过载指令，使视线停止旋转、逐步构成碰撞航向，"
        "最终以小脱靶量交会目标。<br>"
        "<b>描述：</b>比例导引(Proportional Navigation, PN)是拦截/交会制导的经典"
        "制导律。本仿真按<b>三维矢量式</b>实现，水平与垂直通道自动耦合，"
        "无需单独设计俯仰/偏航通道；每个制导周期只取双方<b>位置坐标</b>："
        "相邻周期位置差得到视线变化率，进而解算视线角速率与接近速度，"
        "再按导航比 N' 放大为过载指令，经限幅与（可选）自动驾驶仪延迟后"
        "积分位置进入下一周期。</div>")
    parts.append(
        "<div class='formula'>"
        "r   = P_target − P_own                视线矢量 (ENU 3D, 由位置坐标直接得到)\n"
        "ω   = (r × ṙ) / |r|²                视线角速率矢量 (rad/s, ṙ 由相邻周期位置差得到)\n"
        "Vc  = −ṙ · r̂                        接近速度 (m/s)\n"
        "ac  = N' · Vc · (ω × r̂)             过载指令 (m/s², 垂直视线; 等价标量式 a = N'·Vc·λ̇)\n"
        "限幅: |ac| ≤ n_max · g       反解验证: N' = a⊥ / (Vc · |ω|)  (a⊥ 由自身轨迹曲率差分得到)\n"
        "</div>")

    # 2. 结果摘要
    parts.append("<h2>2. 结果摘要</h2>")
    parts.append(f"<div class='{verdict_cls}'><b>结论：</b>{verdict}</div>")
    parts.append("<table><tr><th>指标</th><th>数值</th><th>说明</th></tr>")
    for k, v, note in rows_tbl:
        parts.append(f"<tr><td>{esc_(k)}</td><td><b>{esc_(v)}</b></td>"
                     f"<td style='text-align:left'>{esc_(note)}</td></tr>")
    parts.append("</table>")

    # 3. 图表
    parts.append("<h2>3. 轨迹与时间历程</h2>")
    parts.append(chart_track)
    parts.append(chart_alt)
    parts.append(chart_rng)
    parts.append(chart_lr)
    parts.append(chart_a)

    # 4. 反解验证
    parts.append("<h2>4. 由位置坐标反解比例导引（验证）</h2>")
    parts.append(
        "<div class='note'>对输出轨迹<b>不再使用</b>仿真内部状态，仅取双方位置序列做"
        "中心差分：由视线序列反解 ω、Vc，由自机轨迹曲率反解横向过载 a⊥，"
        "逐周期计算 N' = a⊥/(Vc·ω)。若轨迹确实由比例导引生成，"
        "反解中位数应落在配置值 ±5% 以内（限幅饱和段会系统性偏低，已计入统计）。</div>")
    parts.append("<table><tr><th>统计项</th><th>数值</th><th>说明</th></tr>")
    for k, v, note in n_tbl:
        parts.append(f"<tr><td>{esc_(k)}</td><td><b>{esc_(v)}</b></td>"
                     f"<td style='text-align:left'>{esc_(note)}</td></tr>")
    parts.append("</table>")
    parts.append(chart_n)

    # 采样表
    parts.append("<h3>解算样例（等间隔抽样）</h3>")
    parts.append("<table><tr><th>t(s)</th><th>斜距(m)</th><th>λ̇(°/s)</th>"
                 "<th>Vc(m/s)</th><th>a⊥(m/s²)</th><th>N'估计</th></tr>")
    n = len(rows)
    picks = sorted(set(min(n - 1, int(round(i * (n - 1) / 9.0))) for i in range(10)))
    for k in picks:
        r = rows[k]
        parts.append(
            f"<tr><td>{r['t']:.2f}</td><td>{_fmt(r.get('R'), 1)}</td>"
            f"<td>{_fmt(r.get('lr'), 4)}</td><td>{_fmt(r.get('vc'), 1)}</td>"
            f"<td>{_fmt(r.get('a_lat'), 2)}</td><td>{_fmt(r.get('n_est'), 3)}</td></tr>")
    parts.append("</table>")

    # 5. 讨论
    if mode == "sim":
        n_txt = (f"反解 N' 中位数 <b>{stats['median']:.2f}</b> vs 配置 {cfg['n']}"
                 f"（偏差 {stats['bias_pct']:+.1f}%）"
                 + ("，验证通过。" if stats.get("ok") else "，偏差偏大（见下）。")
                 if stats.get("count") else "（无有效反解样本）")
        parts.append(
            "<h2>5. 参数影响与讨论</h2><div class='note'>"
            f"<b>导航比 N'：</b>常用 3~5。N' 越大收敛越快、脱靶越小，"
            f"但需用过载峰值越高（本例峰值 {summary['a_cmd_max']:.1f} m/s² = "
            f"{summary['a_cmd_max'] / G0:.1f} g）；N' 过小则轨迹平缓、"
            f"末端易脱靶。<br>"
            f"<b>过载限幅与延迟：</b>限幅 {cfg['nmax']:g}g、自动驾驶仪延迟 "
            f"τ={cfg['tau']:.2f}s 是脱靶量的主要来源；本例饱和 "
            f"{summary['sat_ratio'] * 100:.1f}%，脱靶量 {summary['miss_m']:.2f} m。<br>"
            f"<b>目标机动：</b>--target-turn 给定转弯率后目标做协调转弯，"
            f"视线角速率不再收敛于零，需用过载增大，脱靶随之增大。<br>"
            f"<b>反解验证：</b>{n_txt}</div>")
    else:
        parts.append(
            "<h2>5. 说明</h2><div class='note'>外部轨迹反解不进行命中判定。"
            "若需要复现前向仿真，去掉 --track 即可；"
            "把仿真输出的 pn_track.csv 再用 --track 读回，可验证反解闭环。</div>")

    parts.append(
        "<h2>6. 输出文件</h2><ul>"
        "<li><code>pn_guidance_report.html</code>（本报告）</li>"
        "<li><code>pn_track.csv</code> — 逐周期数据：位置坐标 + 视线/过载/N' 解算值"
        "（lat/lon 保留 9 位小数，可直接用 --track 读回反解）</li>"
        "<li><code>pn_result.json</code> — 参数、结果摘要与全部周期数据</li></ul>")
    parts.append("</div></body></html>")
    return "".join(parts)


# ========================= 输出 =========================
CSV_COLS = ["t", "own_lat", "own_lon", "own_alt",
            "target_lat", "target_lon", "target_alt",
            "range_m", "los_az_deg", "los_el_deg", "los_rate_dps", "vc_mps",
            "a_cmd_mps2", "a_used_mps2", "sat", "a_lat_mps2", "n_est"]


def _r(v, nd):
    return "" if v is None else f"{v:.{nd}f}"


def write_csv(rows, path):
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(CSV_COLS)
        for r in rows:
            wr.writerow([
                f"{r['t']:.3f}",
                f"{r['own_lat']:.9f}", f"{r['own_lon']:.9f}", f"{r['own_alt']:.4f}",
                f"{r['target_lat']:.9f}", f"{r['target_lon']:.9f}",
                f"{r['target_alt']:.4f}",
                _r(r.get("R"), 2), _r(r.get("az"), 3), _r(r.get("el"), 3),
                _r(r.get("lr"), 5), _r(r.get("vc"), 3),
                _r(r.get("a_cmd"), 3), _r(r.get("a_used"), 3),
                "" if r.get("sat") is None else int(r["sat"]),
                _r(r.get("a_lat"), 3), _r(r.get("n_est"), 4),
            ])


def write_json(cfg, mode, rows, summary, stats, path):
    def clean(v):
        if isinstance(v, float):
            return round(v, 6)
        return v

    data = {
        "mode": mode,
        "params": {k: v for k, v in cfg.items() if k != "amax"},
        "summary": {k: clean(v) for k, v in summary.items()} if summary else None,
        "n_est_stats": stats,
        "steps": [{k: clean(v) for k, v in r.items()} for r in rows],
    }
    path.write_text(json.dumps(data, ensure_ascii=False, indent=1),
                    encoding="utf-8")


# ========================= 控制台 =========================
def _dw(s):
    return sum(2 if unicodedata.east_asian_width(ch) in "WFA" else 1 for ch in s)


def _pad(s, w):
    return s + " " * max(0, w - _dw(s))


def print_console(cfg, mode, rows, summary, stats, paths, show_steps=False):
    line = "=" * 78
    print(line)
    print("比例导引仿真  pn_guidance_sim.py  (三维: 经纬 + 高度)")
    print(line)
    if mode == "sim":
        print(f"地面端发射点: ({cfg['launch'][0]:.6f}, {cfg['launch'][1]:.6f}, "
              f"{cfg['launch'][2]:.0f} m)   空中目标: ({cfg['target'][0]:.6f}, "
              f"{cfg['target'][1]:.6f}, {cfg['target'][2]:.0f} m)")
        print(f"初始距离: 斜距 {summary['init_slant'] / 1000:.3f} km / 水平 "
              f"{summary['init_horiz'] / 1000:.3f} km / 高差 {summary['init_dalt']:.0f} m   "
              f"初始视线: 方位 {summary['init_az']:.1f}° 仰角 {summary['init_el']:.1f}°")
        print(f"自机速度 {cfg['own_speed']:.0f} m/s   目标速度 {cfg['tgt_speed']:.0f} m/s "
              f"航向 {cfg['tgt_hdg']:.0f}° 爬升 {cfg['tgt_climb']:.0f}° "
              f"转弯率 {cfg['tgt_turn']:.1f} °/s")
        print(f"导航比 N'={cfg['n']:.1f}   过载限幅 {cfg['nmax']:g}g "
              f"({cfg['amax']:.1f} m/s²)   制导周期 {cfg['dt'] * 1000:.0f} ms   "
              f"自动驾驶仪 τ={cfg['tau']:.2f} s")
        print()
        print("【拦截结果】")
        hit = "命中" if summary["intercepted"] else "未命中"
        print(f"  {hit}: 脱靶量 {summary['miss_m']:.2f} m (判据 ≤ {HIT_R:.0f} m)   "
              f"最近会遇 t={summary['t_close']:.2f} s   "
              f"仿真 {summary['t_sim']:.1f} s / {summary['steps']} 周期")
        print(f"  末段: Vc={summary['vc_close']:.1f} m/s (交会前0.2s距离差分)  "
              f"视线角速率={summary['lr_close']:.4f} °/s (R≥100 m)")
        print(f"  过载: 指令峰值 {summary['a_cmd_max']:.1f} m/s² "
              f"({summary['a_cmd_max'] / G0:.1f} g) | 实际峰值 "
              f"{summary['a_used_max']:.1f} m/s² | 饱和 {summary['sat_steps']} 周期 "
              f"({summary['sat_ratio'] * 100:.1f}%)")
    else:
        print(f"轨迹文件: {cfg['track_file']}   点数: {len(rows)}   "
              f"时长: {rows[-1]['t'] - rows[0]['t']:.1f} s")
        print()
        print("【轨迹反解统计】（未做命中判定）")

    print()
    print("【由位置坐标反解比例导引】")
    if stats.get("count"):
        print(f"  有效样本 {stats['count']}/{len(rows)}   "
              f"N' 中位数 {stats['median']:.3f}   均值 {stats['mean']:.3f} ± "
              f"{stats['std']:.3f}   四分位 [{stats['q1']:.3f}, {stats['q3']:.3f}]")
        if stats.get("cfg"):
            ok = "验证通过: 轨迹确实按比例导引生成" if stats.get("ok") \
                else "偏差超过 5%（可能处于限幅饱和/大延迟/轨迹点过稀）"
            print(f"  配置值 {stats['cfg']} → 偏差 {stats['bias_pct']:+.2f}%   {ok}")
    else:
        print("  无有效反解样本（差分阈值过滤）")

    if show_steps:
        print()
        print("【周期采样】")
        hdr = ("t(s)", "斜距(m)", "λ̇(°/s)", "Vc(m/s)", "ac(m/s²)", "N'估计", "自机高度")
        wds = (8, 10, 9, 10, 11, 8, 9)
        print("  " + "  ".join(_pad(h, w) for h, w in zip(hdr, wds)))
        stride = max(1, len(rows) // 40)
        for i in range(0, len(rows), stride):
            r = rows[i]
            vals = (f"{r['t']:.2f}", _fmt(r.get("R"), 1), _fmt(r.get("lr"), 4),
                    _fmt(r.get("vc"), 1), _fmt(r.get("a_cmd"), 2),
                    _fmt(r.get("n_est"), 3), f"{r['own_alt']:.0f}")
            print("  " + "  ".join(_pad(v, w) for v, w in zip(vals, wds)))

    print()
    print("【输出文件】")
    print(f"  报告: {paths[0]}")
    print(f"  航迹: {paths[1]}")
    print(f"  数据: {paths[2]}")


# ========================= 参数与入口 =========================
def parse_llh(s, label, default_alt=0.0):
    parts = [p.strip() for p in s.split(",")]
    if len(parts) not in (2, 3):
        raise ValueError(f'{label} 应为 "纬度,经度" 或 "纬度,经度,高度m"，收到: {s!r}')
    try:
        lat, lon = float(parts[0]), float(parts[1])
        alt = float(parts[2]) if len(parts) == 3 else default_alt
    except ValueError:
        raise ValueError(f'{label} 含非数字项: {s!r}')
    if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
        raise ValueError(f"{label} 经纬度超出范围: {s!r}")
    return (lat, lon, alt)


def _ask(prompt, default):
    try:
        s = input(prompt).strip()
    except EOFError:
        return default
    return s or default


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="比例导引三维拦截仿真：通过位置坐标解算比例导引（地面端→空中目标）")
    ap.add_argument("--launch", help='地面端发射点 "纬度,经度,高度m"，如 39.905,116.409,0')
    ap.add_argument("--target", help='空中目标初始点 "纬度,经度,高度m"，如 39.970,116.480,3000')
    ap.add_argument("--own-speed", type=float, default=400.0,
                    help="拦截机速度 m/s，默认 400")
    ap.add_argument("--target-speed", type=float, default=150.0,
                    help="目标速度 m/s，默认 150")
    ap.add_argument("--target-hdg", type=float, default=120.0,
                    help="目标航向 度(0=正北)，默认 120")
    ap.add_argument("--target-climb", type=float, default=0.0,
                    help="目标爬升角 度，默认 0")
    ap.add_argument("--target-turn", type=float, default=0.0,
                    help="目标转弯率 度/秒(正=右转)，默认 0")
    ap.add_argument("--n", type=float, default=4.0,
                    help="导航比 N'，默认 4（常用 3~5）")
    ap.add_argument("--nmax", type=float, default=9.0,
                    help="过载限幅，单位 g，默认 9")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="制导周期 秒，默认 0.05 (20 Hz)")
    ap.add_argument("--tmax", type=float, default=120.0,
                    help="最大仿真时长 秒，默认 120")
    ap.add_argument("--tau", type=float, default=0.0,
                    help="自动驾驶仪一阶时间常数 秒，0=理想，默认 0")
    ap.add_argument("--init-hdg", type=float, default=None,
                    help="拦截机初始方位 度，缺省=对准初始视线")
    ap.add_argument("--track", default="",
                    help="反解模式：读取位置轨迹 CSV(t,own_lat,...,target_alt)")
    ap.add_argument("--outdir", default="", help="输出目录(默认脚本所在目录)")
    ap.add_argument("--show-steps", action="store_true",
                    help="控制台打印周期采样表")
    args = ap.parse_args(argv)

    if args.n <= 0:
        print("参数错误: --n 必须 > 0", file=sys.stderr)
        return 2
    if args.nmax <= 0:
        print("参数错误: --nmax 必须 > 0", file=sys.stderr)
        return 2
    if args.dt <= 0:
        print("参数错误: --dt 必须 > 0", file=sys.stderr)
        return 2

    mode = "track" if args.track else "sim"
    try:
        if mode == "track":
            cfg = {"track_file": args.track}
            times, own, tgt = load_track(args.track)
            # 反解在局部 ENU（米）中进行：经纬度是角度，不能直接与高度混算
            lat0, lon0 = own[0][0], own[0][1]
            own_enu, tgt_enu = [], []
            for (la, lo, al), (tl, to, ta) in zip(own, tgt):
                ox, oy = geo_to_enu(la, lo, lat0, lon0)
                gx, gy = geo_to_enu(tl, to, lat0, lon0)
                own_enu.append((ox, oy, al))
                tgt_enu.append((gx, gy, ta))
            res = solve_track(times, own_enu, tgt_enu)
            rows = track_to_rows(times, own, tgt, res)
            summary = None
        else:
            launch_s = args.launch or _ask(
                '请输入地面端发射点(纬度,经度,高度m) [39.905,116.409,0]: ',
                "39.905,116.409,0")
            target_s = args.target or _ask(
                '请输入空中目标初始点(纬度,经度,高度m) [39.970,116.480,3000]: ',
                "39.970,116.480,3000")
            launch = parse_llh(launch_s, "地面端发射点")
            target = parse_llh(target_s, "空中目标")
            cfg = {
                "launch": launch, "target": target,
                "own_speed": args.own_speed, "tgt_speed": args.target_speed,
                "tgt_hdg": args.target_hdg, "tgt_climb": args.target_climb,
                "tgt_turn": args.target_turn,
                "n": args.n, "nmax": args.nmax, "amax": args.nmax * G0,
                "dt": args.dt, "tmax": args.tmax, "tau": args.tau,
                "init_hdg": args.init_hdg,
            }
            if cfg["own_speed"] <= 0 or cfg["tgt_speed"] <= 0:
                raise PNError("速度必须 > 0")
            rows, summary = simulate(cfg)
            # 反解闭环：只用位置坐标（ENU 米制）重新解算 N'
            attach_inverse(rows,
                           [(r["x"], r["y"], r["own_alt"]) for r in rows],
                           [(r["tx"], r["ty"], r["target_alt"]) for r in rows],
                           [r["t"] for r in rows])
            # 记录最近会遇行号(供报告标注)
            min_row = min(range(len(rows)), key=lambda i: rows[i]["R"])
            summary["min_row"] = min_row
        vals = [r.get("n_est") for r in rows]
        stats = n_stats(vals, cfg.get("n") if mode == "sim" else None)
    except (ValueError, PNError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2

    outdir = Path(args.outdir) if args.outdir else Path(__file__).resolve().parent
    outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / "pn_track.csv"
    json_path = outdir / "pn_result.json"
    html_path = outdir / "pn_guidance_report.html"
    write_csv(rows, csv_path)
    write_json(cfg, mode, rows, summary, stats, json_path)
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    html_path.write_text(build_report(cfg, mode, rows, summary, stats, stamp),
                         encoding="utf-8")

    print_console(cfg, mode, rows, summary, stats,
                  (str(html_path), str(csv_path), str(json_path)),
                  show_steps=args.show_steps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
