// ============================================================
// 空气动力学：机身阻力、翼型/极曲线、噪音、地面效应
// ============================================================
import { clamp } from './parts.js';
import { reynolds } from './air.js';
import { T } from './tuning.js';

// ---------- 多旋翼机身（含桨盘）阻力 ----------
export function bodyDrag(rho, V, area_m2, kind = 'quad') {
  const ta = T().aero;
  const cd = { quad: ta.cdQuad, hex: ta.cdHex, octo: ta.cdOcto, whoop: ta.cdWhoop, rocket: ta.cdRocket, fixed: ta.cdFixed }[kind] ?? ta.cdQuad;
  const D = 0.5 * rho * V * V * area_m2 * cd;
  return { D, cd, area: area_m2, q: 0.5 * rho * V * V };
}

// ---------- 固定翼 ----------
export function wingGeom({ span, area, cd0 = 0.028, clmax = 1.4, e = 0.8 }) {
  const AR = span * span / area;
  return { span, area, AR, cd0, clmax, e };
}

export function wingPolar(wing, rho, mu, V, W) {
  const S = wing.area;
  const q = 0.5 * rho * V * V * S;
  const CL = W / Math.max(q, 1e-6);
  const CDi = CL * CL / (Math.PI * wing.AR * wing.e);
  const CD = wing.cd0 + CDi;
  const D = CD * q;
  const LD = CL / Math.max(CD, 1e-9);
  const Re = reynolds(V, wing.span / 12, mu / rho);
  return { q, CL, CD, CDi, D, L_D: LD, Re, eff: (CL / CD) };
}

export function stallSpeed(W, rho, wing) {
  return Math.sqrt(2 * W / (rho * wing.area * wing.clmax));
}

// ---------- 垂起火箭（Tailsitter Rocket）巡航气动 ----------
// 水平巡航时：尾翼（低展弦比）+ 细长机身（升力体/cross-flow）共同产生升力。
// 参考面积 Sref = 尾翼总面积 + bodyFactor·机身侧投影（升力体折算）。
export function rocketGeom(rk) {
  const D = rk.bodyD_mm / 1000;
  const Lb = rk.bodyLength_m;
  const Sside = Lb * D;                       // 机身侧投影面积
  const Sfin = rk.finArea_m2;                 // 尾翼总面积
  const bfin = rk.finSpan_m * 2;              // 尾翼全展
  const ARfin = (bfin * bfin) / Math.max(Sfin, 1e-6);
  const bodyFactor = T().aero.bodyFactor;     // 升力体贡献折算系数
  const Sref = Sfin + bodyFactor * Sside;
  const AR = (bfin * bfin) / Sref;
  // 尾翼升力线斜率（Helmbold 低展弦比修正）与失速攻角
  const aFin = (Math.PI * ARfin) / (1 + Math.sqrt(1 + (ARfin / 2) ** 2));
  const alphaStallDeg = (rk.finClmax / aFin) * 180 / Math.PI;
  const aTotal = aFin + 2.0 * bodyFactor * Sside / Sref;   // 尾翼 + 升力体合计升力线斜率
  return {
    D, Lb, Sside, Sfin, bfin, ARfin, Sref, AR, bodyFactor, aFin, aTotal, alphaStallDeg,
    cd0: rk.cd0, clmaxFin: rk.finClmax, finCount: rk.finCount,
    area: Sref, span: bfin, clmax: rk.finClmax, e: 0.62,   // 兼容 wingGeom 字段
  };
}

/** 火箭水平巡航极曲线：含尾翼失速判定、机身升力体、横向绕流阻力 */
export function rocketPolar(g, rho, mu, V, W) {
  const q = 0.5 * rho * V * V;
  const S = g.Sref;
  const CL = W / Math.max(q * S, 1e-9);
  // 尾翼升力线斜率（Helmbold 低展弦比修正），per rad
  const aFin = (Math.PI * g.ARfin) / (1 + Math.sqrt(1 + (g.ARfin / 2) ** 2));
  // 机身法向力（细长体 cross-flow，CN_α≈2 相对侧投影），折算到 Sref
  const aBody = 2.0 * g.bodyFactor * g.Sside / S;
  const aTotal = aFin + aBody;
  const alpha = CL / Math.max(aTotal, 0.2);          // 所需攻角 rad
  const alphaStall = g.clmaxFin / aFin;              // 尾翼失速攻角
  const stalled = Math.abs(alpha) > alphaStall;
  // 阻力：CD0 + 诱导 + 机身横向绕流
  const eEff = stalled ? 0.40 : g.e;
  const CDi = CL * CL / (Math.PI * g.AR * eEff);
  const CDx = 0.6 * Math.pow(Math.sin(alpha), 2) * (g.Sside / S) * (g.Lb / g.D) * 0.05;
  const CD = g.cd0 + CDi + Math.abs(CDx);
  const D = CD * q * S;
  const Re = reynolds(V, g.Lb / 6, mu / rho);
  return {
    q, CL, CD, CDi, CDx, D, L_D: CL / Math.max(CD, 1e-9), Re,
    alphaDeg: alpha * 180 / Math.PI, alphaStallDeg: alphaStall * 180 / Math.PI,
    stalled, aTotal, aFin, aBody,
  };
}

/**
 * 静稳定裕度（火箭 & 固定翼通用）：
 * x_NP 由尾翼/尾容积决定；x_CG 由质量分布估计。
 * SM = (x_NP − x_CG) / c̄。SM>0 静稳定，目标 5%~15%。
 */
export function staticStability({ rho = 1.225, cbar_m, x_NP_m, x_CG_m, V_mps, W_N, S_m2, Iyy, mass_kg, label = '' }) {
  const SM = (x_NP_m - x_CG_m) / cbar_m;                      // 无量纲（c̄ 的百分比）
  const q = 0.5 * rho * V_mps * V_mps;
  // 俯仰阻尼 Mq ≈ −(a·q·S·c̄²)/(2V)·(1 + 尾臂项)
  const Cl_alpha = 2 * Math.PI;
  const Mq = -(Cl_alpha * q * S_m2 * cbar_m * cbar_m) / (2 * Math.max(V_mps, 1));
  // 短周期近似：ωn = sqrt(−Mα/Iyy)，Mα = −q·S·c̄·SM
  const Malpha = -q * S_m2 * cbar_m * SM;
  const wn = Math.sqrt(Math.max(1e-6, Math.abs(Malpha) / Math.max(Iyy, 1e-6)));
  const zeta = Math.abs(Mq) / (2 * Math.max(wn, 1e-3) * Math.max(Iyy, 1e-6));
  return {
    label, SM_pct: +(SM * 100).toFixed(1),
    x_NP_pct: +((x_NP_m / cbar_m) * 100).toFixed(0),
    x_CG_pct: +((x_CG_m / cbar_m) * 100).toFixed(0),
    shortPeriod_Hz: +(wn / (2 * Math.PI)).toFixed(2),
    zeta: +Math.min(zeta, 3).toFixed(2),
    phugoid_s: +((Math.SQRT2 * V_mps) / 9.80665).toFixed(1),
  };
}

export function levelPowerRequired(wing, rho, mu, V, W, etaProp) {
  const pol = wingPolar(wing, rho, mu, V, W);
  return { ...pol, P: pol.D * V / Math.max(etaProp, 0.05) };
}

// ---------- 螺旋桨前进效率（基于 BEMT 的推进功率/轴功率）----------
export function propEfficiency(bemtResult) {
  if (bemtResult.P_aero <= 1e-6) return 0;
  return clamp((bemtResult.T * Math.abs(bemtResult.omega > 0 ? bemtResult.omega : 0) * 0 + (bemtResult.T * (bemtResult.Va ?? 0))) / bemtResult.P_aero, 0, 1);
}

// ---------- 气动噪音（半经验）----------
export function noiseDb({ vTip, blades, D, dist_m = 1 }) {
  if (vTip <= 1) return 0;
  const ta = T().aero;
  const L1 = ta.noiseVelExp * Math.log10(vTip) + ta.noiseSizeExp * Math.log10(Math.max(0.01, blades * D)) + ta.noiseOffset_dB;
  return Math.max(0, L1 - 20 * Math.log10(Math.max(0.1, dist_m)));
}

// ---------- 地面效应 ----------
export function groundEffect(height_m, D) {
  const h = height_m;
  const ratio = h / Math.max(D, 0.01);
  if (ratio >= 1) return { factor: 1, note: '地面效应可忽略 (h/D≥1)' };
  const f = 1 + T().aero.groundK * (1 - ratio) * (1 - ratio);
  return { factor: f, note: `h/D=${ratio.toFixed(2)}，诱导功率下降约 ${((1 - 1 / f) * 100).toFixed(0)}%` };
}

// ---------- 桨叶雷诺数 / 失速裕度 ----------
export function bladeRe(aeroStation, P, air) {
  return reynolds(aeroStation.W, P.c, air.nu);
}
