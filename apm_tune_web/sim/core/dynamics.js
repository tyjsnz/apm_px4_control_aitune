// ============================================================
// 结构强度 + 飞控动力学 / PID 整定 / 阶跃响应
// ============================================================
import { clamp } from './parts.js';
import { bemt, operatingPoint } from './bemt.js';
import { T } from './tuning.js';

const G = 9.80665;

// --------------------------- 结构 ---------------------------
export function structure({ massParts, arm_m, frame_g, prop: P, motor: M, maxThrustN, hoverThrustN, n, kind, maxRPM = 30000 }) {
  const tMotor = maxThrustN / n;          // 单电机最大推力
  const L = arm_m;

  // 机臂碳板（强度 + 刚度双重控制）
  const plateW_mm = clamp(40 + arm_m * 120, 35, 70);
  const tm = T().carbon;
  const carbon = { E: tm.E_GPa * 1e9, sigmaAll: tm.sigmaAll_MPa * 1e6, density: tm.density };  // 层合板设计值（可调）
  const w = plateW_mm / 1000;
  const M_arm = tMotor * L;                                   // 悬臂根部弯矩 N·m
  const t_stress = Math.sqrt(6 * M_arm / (w * carbon.sigmaAll));
  const dLim = Math.min(0.005, Math.max(0.0015, L * 0.04));   // 允许挠度
  const t_stiff = Math.cbrt((4 * M_arm * L * L) / (carbon.E * w * dLim));
  const t_req = Math.max(t_stress, t_stiff);
  const t_default = clamp(Math.ceil(t_req * 1000), 2, 14);    // mm（向上取整）
  const I_arm = w * Math.pow(t_default / 1000, 3) / 12;
  const sigma_arm = M_arm / (w * Math.pow(t_default / 1000, 2) / 6);
  const SF_arm = carbon.sigmaAll / Math.max(sigma_arm, 1);
  const delta_arm = (tMotor * Math.pow(L, 3)) / (3 * carbon.E * I_arm);   // 端部挠度 m
  // 机臂一阶固有频率（等截面悬臂近似）
  const mUnit = (frame_g / 1000) * 0.55 / Math.max(L, 0.02) + P.density * P.c * 0.11 * P.c;
  const f_arm = (Math.pow(1.875, 2) / (2 * Math.PI)) * Math.sqrt(carbon.E * I_arm / (Math.max(mUnit, 1e-5) * Math.pow(L, 4)));

  // 桨叶离心应力（绕根部旋转杆） + 材料许用
  const vTip = P.tipSpeedAt(maxRPM);
  const matTensile = { plastic: 52e6, carbon: 320e6, wood: 58e6, folded: 55e6 }[P.material.id] || 52e6;
  const sigma_prop = P.density * Math.pow(vTip, 2) / 2;
  const SF_prop = matTensile / Math.max(sigma_prop, 1);

  // 电机座螺钉剪切（2×M3）
  const screwArea = 2 * Math.PI * Math.pow(0.0015, 2);   // 2 颗 M3 有效剪切面
  const tau = tMotor / Math.max(screwArea, 1e-9);
  const SF_screw = T().screw.tauAll_MPa * 1e6 / Math.max(tau, 1);

  // 起落架/整机
  const massTotal = Object.values(massParts).reduce((a, b) => a + (typeof b === 'number' ? b : 0), 0);

  return {
    armPlate: { width_mm: +plateW_mm.toFixed(0), thickness_mm: t_default, sigma_MPa: +(sigma_arm / 1e6).toFixed(1), safety: +SF_arm.toFixed(1), deflection_mm: +(delta_arm * 1000).toFixed(2), required_mm: +(t_req * 1000).toFixed(2), E_GPa: carbon.E / 1e9 },
    armFreq_Hz: +f_arm.toFixed(1),
    prop: { tipStress_MPa: +(sigma_prop / 1e6).toFixed(1), safety: +SF_prop.toFixed(1), vTip_ms: +vTip.toFixed(0), material: P.material.label },
    screw: { tau_MPa: +(tau / 1e6).toFixed(1), safety: +SF_screw.toFixed(1) },
    massParts, massTotal_g: massTotal,
  };
}

// --------------------------- 转动惯量 ---------------------------
export function inertia({ mass_kg, arm_m, motorMass_g, propMass_g, frame_g, battery_g, n }) {
  const mm = motorMass_g / 1000 + propMass_g / 1000;
  // X 构型：电机位于距中心 arm 处，y 分量 = arm/√2
  const y2 = Math.pow(arm_m / Math.SQRT2, 2);
  const I_motors = n * mm * y2 * 2;                       // 每侧两对
  const I_frame = 0.45 * (frame_g / 1000) * arm_m * arm_m;
  const I_batt = (battery_g / 1000) * (Math.pow(0.05, 2) + Math.pow(0.03, 2)) / 12;
  const Ixx = I_motors + I_frame + I_batt;
  const Izz = 2 * n * mm * y2 + 0.5 * (frame_g / 1000) * arm_m * arm_m;
  return { Ixx: +Ixx.toFixed(6), Izz: +Izz.toFixed(6), Iyy: +Ixx.toFixed(6), unit: 'kg·m²' };
}

// --------------------------- 执行机构时间常数 ---------------------------
export function actuatorLag({ prop: P, motor: M, hoverOmega }) {
  // 桨叶惯量 + 3.5 倍（含轮毂与空气质量附加惯量）
  const J_blade = P.B * P.density * (0.11 * P.c) * P.c * Math.pow(P.R, 3) / 3;
  const J_eff = J_blade * 3.5 + 2e-7;
  // 固定占空比下线性化：dTorque/dω = Kt/(KVsi·R_m)
  const slope = M.Kt / (M.KVsi * M.R_m);
  const tau = J_eff / slope;
  const passing = (P.B * hoverOmega) / (2 * Math.PI);   // 桨叶通过频率 Hz
  return {
    J_blade: +J_blade.toExponential(2), J_eff: +J_eff.toExponential(2),
    tau_ms: +(tau * 1000).toFixed(1),
    elecTau_ms: +((15e-6 / M.R_m) * 1000).toFixed(2),
    passing_Hz: +passing.toFixed(0),
    bandwidth_Hz: +(1 / (2 * Math.PI * 2 * tau)).toFixed(1),
  };
}

// --------------------------- 姿态环 PID ---------------------------
/**
 * 速率环极点配置整定。
 * 被控对象（占空比不平衡 u → 角速率 x）：G(s) = K / (s(τs+1))
 *   K  = 最大不平衡推力矩 / 转动惯量  [rad/s² per unit u]
 *   τ  = 电机+桨执行机构一阶滞后
 * 闭环特征方程（u = P·err + I·∫err + D·dMeas）：
 *   τs² + (1+K·D)s + K·P = 0
 *   → ωn² = K·P/τ  ⇒  P = ωn²·τ/K
 *   → ζ = (1+K·D)/(2τωn)  ⇒  D = (2ζ·τ·ωn − 1)/K   （欠阻尼才补 D）
 * 带宽 p（主导闭环极点）受两个上限约束：
 *   ① 执行机构相位：p ≤ 1/τ
 *   ② 杆量阶跃 Δ≈172°/s 的权限：u(0)=pΔ/K ≤ 0.8 ⇒ p ≤ 0.8K/Δ
 * 极点配置到 {−p, −1/τ}（均为实极点 → 无超调，调节时间 ≈ 4/p）：
 *   τs² + (1+K·D)s + K·P = τ(s+p)(s+1/τ)
 *   ⇒ P = p/K,  D = pτ/K
 * p = 1/τ 时与 ζ=1 的二阶配置完全一致；权限受限（p<1/τ）时避免过阻尼病态慢极点。
 * 积分在 p/150 处与比例交叉（对象含 s⁻¹ 已无静差，积分仅抗扰，放远不侵蚀裕度），
 * 积分权限 iAuth=0.1（占空比的 10%）：足以消 8~10% 稳态扰动，又限制绕回。
 * 参数经 K∈[80,2e4]、τ∈[4,120]ms 全域扫描：最坏超调 ≤6%、典型 1.2%。
 */
export function attitudeControl({ Ixx, arm_m, maxThrustN, hoverThrustN, actuatorTau_s, n, kind }) {
  // 执行机构增益：占空比差 → 角加速度
  const dT_dDuty = clamp((maxThrustN - hoverThrustN) / n / 0.6, 0.5, 1e6);
  const lever = arm_m / Math.SQRT2;
  const K = (1.414 * dT_dDuty * lever) / Ixx;   // rad/s² per 占空比

  const tau = Math.max(0.004, actuatorTau_s);
  const DELTA = 3;                               // 典型杆量阶跃 ~172°/s
  const p = Math.min(1 / tau, (0.8 * K) / DELTA, 150);   // 主导极点
  const P = p / K;
  const D = (p * tau) / K;
  const wn = Math.sqrt((K * P) / tau);           // 等效二阶自然频率（报告用）
  const zeta = (1 + K * D) / (2 * tau * wn);     // 等效阻尼比（可 >1，实极点）
  const Ii = (P * p) / 150;                      // 积分交叉频率 = p/150
  const iAuth = 0.1;                             // 积分权限（归一化占空比）
  const iiClamp = Ii > 0 ? iAuth / Ii : Infinity;
  const dfilt = clamp(tau / 2.5, 0.001, 0.02);

  // 阶跃响应：速率环（D 作用于测量值，避免设定值微分踢击）
  const dt = 0.0004, N = 12500;                  // 5s 窗口（覆盖慢积分调节）
  let x = 0, xm = 0, ii = 0, filt = 0, xPrev = 0;
  const resp = [];
  const setpoint = 10;   // rad/s 阶跃
  let peak = 0, riseT = null;
  for (let k = 0; k < N; k++) {
    const err = setpoint - x;
    const dMeas = -(x - xPrev) / dt; xPrev = x;
    filt = filt + (dt / (dfilt + dt)) * (dMeas - filt);
    ii = clamp(ii + err * dt, -iiClamp, iiClamp);
    const u = clamp(P * err + Ii * ii + D * filt, -1, 1);
    xm = xm + ((K * u - xm) / tau) * dt;      // 执行机构一阶滞后 → 角加速度
    x = x + xm * dt;
    peak = Math.max(peak, x);
    if (riseT === null && x >= setpoint * 0.9) riseT = k * dt;
    if (k % 4 === 0) resp.push({ t: +(k * dt).toFixed(3), v: +x.toFixed(3) });
  }
  const overshoot = Math.max(0, (peak - setpoint) / setpoint * 100);
  let settle = null;
  for (let i = 0; i < resp.length; i++) {
    if (resp.slice(i).every(r => Math.abs(r.v - setpoint) <= 0.02 * setpoint)) { settle = resp[i].t; break; }
  }
  const final = resp[resp.length - 1]?.v ?? 0;

  // 稳态抗扰：8% 满权限常值扰动（如重心偏置/单侧风），30s 残差
  let y = 0, ym = 0, ii2 = 0, f2 = 0, yp = 0;
  const dt2 = 0.001;
  for (let k = 0; k < 30000; k++) {
    const err = -y;
    const dM = -(y - yp) / dt2; yp = y;
    f2 = f2 + (dt2 / (dfilt + dt2)) * (dM - f2);
    ii2 = clamp(ii2 + err * dt2, -iiClamp, iiClamp);
    const u = clamp(P * err + Ii * ii2 + D * f2, -1, 1);
    ym = ym + ((K * u - 0.08 * K - ym) / tau) * dt2;
    y = y + ym * dt2;
  }

  return {
    gain: { K: +K.toFixed(1), dT_dDuty_N: +dT_dDuty.toFixed(2), lever_m: +lever.toFixed(3) },
    plant: {
      tau_ms: +(tau * 1000).toFixed(1), wn_hz: +(wn / (2 * Math.PI)).toFixed(2),
      zeta: +zeta.toFixed(2), wn: +wn.toFixed(1),
    },
    pid: {
      P: +P.toFixed(5), I: +Ii.toFixed(5), D: +D.toFixed(5),
      P_255: Math.round(P * 255), I_255: Math.round(Ii * 255), D_255: Math.round(D * 255),
      iAuth, iiClamp: iiClamp === Infinity ? null : +iiClamp.toFixed(0),
      dFilter_ms: +(dfilt * 1000).toFixed(1),
      iTime_ms: +(Ii > 0 ? 1000 / Ii : 0).toFixed(0),
    },
    response: {
      data: resp, overshoot: +overshoot.toFixed(1),
      settling_ms: settle !== null ? settle * 1000 : null,
      rise_ms: riseT !== null ? +(riseT * 1000).toFixed(0) : null,
      final: +final.toFixed(2),
      distResidual: +y.toFixed(2),
      distRejected: Math.abs(y) <= 0.2,
    },
    raw: { setpoint },
  };
}

// --------------------------- 悬停到满油门的推力响应曲线 ---------------------------
export function thrustCurve(ctx, n, steps = 21) {
  const out = [];
  for (let i = 0; i < steps; i++) {
    const duty = i / (steps - 1);
    const op = operatingPoint({ ...ctx, duty });
    out.push({ duty: +duty.toFixed(2), thrust_g: +(op.thrust / G * 1000).toFixed(0), current: +op.current.toFixed(2), rpm: Math.round(op.rpm), power: Math.round(op.pIn), gPerW: +op.gPerW.toFixed(2) });
  }
  return out;
}
