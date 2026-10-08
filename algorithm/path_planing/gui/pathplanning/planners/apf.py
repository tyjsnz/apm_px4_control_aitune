"""算法4: 人工势场法 (APF, Artificial Potential Field)。

原理
----
把飞机看成在"引力场 + 斥力场"中运动的质点：
    F_att = k_att * (目标 - 当前位置)                    越远引力越大
    F_rep = k_rep * (1/d - 1/d0) / d² * 远离障碍方向      距障碍 d < d0 时生效, d→0 斥力→∞
每步沿"滤波后的合力方向"前进固定步长(对势能做梯度下降)，直到抵达目标。

本实现的四个工程处理(标准 APF 的常见缺陷及对策)：
1) 方向低通滤波 v = βv + (1-β)F̂：纯梯度在引力/斥力平衡带附近会逐步左右
   横跳(chattering)，滤波后输出平滑曲线；滤波向量模长过小时回退瞬时合力，
   防止"两点振荡"极限环；
2) 单步转角上限(默认90°)：彻底禁止一步反向的锯齿运动；
3) 近平衡检测：合力远小于引力(局部极小前兆)时当步立即触发逃逸，不在
   平衡点附近抖动；
4) 局部极小逃逸：施加一个绕障切向的"侧向力"并按5%/步渐变接入/撤出，
   方向连续、无硬拐角；多次逃逸仍失败则明确报告失败。
另: 斥力按"边界距离-安全缓冲"计算，保证数值上不穿透禁飞圆。
"""
from __future__ import annotations

import math

import numpy as np

from .base import PlanningError, Planner


class APFPlanner(Planner):
    algo = "apf"
    label = "人工势场法(APF)"

    def plan_raw(self, start, goal, start_heading=None, goal_heading=None):
        p = self.params
        k_att = float(p.get("k_att", 1.0))
        k_rep = float(p.get("k_rep", 2.0e6))     # 斥力强度(调参: 过小会贴障碍太近)
        d0 = float(p.get("d0", 80.0))            # 斥力影响半径(米)
        step = float(p.get("step", 1.0))         # 步长(米)
        tol = float(p.get("tol", 1.0))           # 到达判定(米)
        beta = float(p.get("beta", 0.6))         # 方向低通滤波系数
        buffer = float(p.get("buffer_m", 8.0))   # 斥力安全缓冲(米)
        max_turn = math.radians(float(p.get("max_turn_deg", 90.0)))   # 单步最大转角
        eq_ratio = float(p.get("eq_ratio", 0.3))     # 合力/引力 < 该比例 => 判定近平衡
        max_steps = int(p.get("max_steps", 60000))

        pos = start.astype(float).copy()
        path = [pos.copy()]
        v = np.zeros(2)                        # 滤波后的行进方向
        prev_dir = (goal - start) / max(float(np.linalg.norm(goal - start)), 1e-9)
        side = np.zeros(2)                     # 逃逸侧向力(实际生效值)
        side_target = np.zeros(2)              # 逃逸侧向力(目标值, 渐变接入)
        side_gap = 0.0                         # 施加侧向力时的剩余距离
        best_gap = float(np.linalg.norm(goal - pos))
        stuck = 0
        escapes = 0

        for k in range(max_steps):
            gap = float(np.linalg.norm(goal - pos))
            if gap <= tol:
                break

            side = side + 0.05 * (side_target - side)   # 侧向力渐变, 避免硬拐角
            F = k_att * (goal - pos) + side
            for ob in self.obstacles:
                dv = pos - ob.center
                db = float(np.linalg.norm(dv)) - ob.radius_m   # 到圆边界真实距离
                d = db - buffer                                 # 含缓冲的斥力距离
                if 0.0 < d < d0:
                    unit = dv / max(np.linalg.norm(dv), 1e-9)
                    F = F + k_rep * (1.0 / d - 1.0 / d0) / (d * d) * unit
                elif db <= 0.0:                                 # 已在圆内: 强制推出
                    unit = dv / max(np.linalg.norm(dv), 1e-9)
                    F = F + unit * 1e6
            nF = float(np.linalg.norm(F))
            if nF < 1e-9:
                F, nF = goal - pos, gap                            # 合力为零: 朝目标硬走

            # 近平衡(引力≈斥力)是局部极小的前兆: 立即触发逃逸,
            # 避免在平衡带附近左右抖动产生锯齿(逃逸进行中不重复触发)
            if not side_target.any() and nF < eq_ratio * k_att * max(gap, 1.0):
                stuck = max(stuck, 151)

            # 方向低通滤波, 抑制平衡带附近的逐步横跳。
            # 滤波向量模长过小说明正处剧烈变向(易形成两点振荡), 改用瞬时合力方向
            v = beta * v + (1.0 - beta) * (F / nF)
            nv = float(np.linalg.norm(v))
            direction = v / nv if nv > 0.5 else F / nF

            # 转角限制: 禁止单步反向(两点振荡), 也等效一个很宽松的最小转弯半径
            dot = float(np.clip(prev_dir @ direction, -1.0, 1.0))
            ang = math.atan2(prev_dir[0] * direction[1] - prev_dir[1] * direction[0], dot)
            ang = max(-max_turn, min(max_turn, ang))
            ca, sa = math.cos(ang), math.sin(ang)
            direction = np.array([prev_dir[0] * ca - prev_dir[1] * sa,
                                  prev_dir[0] * sa + prev_dir[1] * ca])
            prev_dir = direction

            pos = pos + direction * step
            path.append(pos.copy())

            # --- 局部极小检测与连续逃逸 ---
            gap_new = float(np.linalg.norm(goal - pos))
            if gap_new < best_gap - 0.5 * step:    # 有意义的进展才算脱离卡住
                if side_target.any() and gap_new < side_gap - 30.0:
                    side_target = np.zeros(2)          # 逃逸已绕开, 撤去侧向力
                best_gap, stuck = gap_new, 0
            else:
                stuck += 1

            if stuck > 150:
                escapes += 1
                if escapes > 5:
                    raise PlanningError("APF 陷入局部极小(多次逃逸失败)")
                # 绕障切向: 目标方向中垂直于"障碍->当前位置"的分量, 平滑绕行
                if self.obstacles:
                    nearest = min(self.obstacles,
                                  key=lambda o: np.linalg.norm(pos - o.center))
                    u = (pos - nearest.center) / max(
                        np.linalg.norm(pos - nearest.center), 1e-9)
                    g = goal - pos
                    t = g - float(g @ u) * u                       # 切向分量
                    if np.linalg.norm(t) < 1e-6:
                        t = np.array([-u[1], u[0]])
                    side_dir = t / np.linalg.norm(t)
                else:
                    g = goal - pos
                    side_dir = np.array([-g[1], g[0]]) / max(np.linalg.norm(g), 1e-9)
                side_target = side_dir * (k_att * max(best_gap, 50.0) * 0.9)
                side_gap = best_gap
                stuck = 0
        else:
            raise PlanningError(f"APF 超过 {max_steps} 步未到达目标")

        if np.linalg.norm(np.asarray(path[-1]) - goal) > tol:
            path.append(goal.copy())
        self._info = {"steps": len(path) - 1, "escapes": escapes}
        return np.asarray(path, dtype=float)

    def last_extra(self) -> dict:
        return getattr(self, "_info", {})
