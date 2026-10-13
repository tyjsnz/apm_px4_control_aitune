// ============================================================
// 桨叶动量-叶素理论 (BEMT) + 电机/电调耦合求解
// 返回单位：SI（N, N·m, W, rad/s）
// ============================================================
import { clamp, clamp01 } from './parts.js';
import { T as tuning } from './tuning.js';   // 别名：本文件内 T 表示推力（行业惯例）

/**
 * 桨叶气动：给定角速度 omega 与轴向来流 Va，求推力/扭矩
 * @param {object} P   prop() 结果
 * @param {object} air airModel() 结果
 */
export function bemt(P, air, omega, Va = 0, opts = {}) {
  const { rho, nu } = air;
  const R = P.R, A = P.A, B = P.B, c0 = P.c, PITCH = P.P;
  const NST = opts.stations ?? 5;
  const INNER = opts.inner ?? 14;
  const r0 = 0.20;
  const dr = (1 - r0) / NST;

  // 调参表（半经验系数）—— 提到循环外读一次
  const tg = tuning().bemt;
  const CAMBER = tg.camber_deg * Math.PI / 180;   // 叶型弯度等效攻角偏移（真实桨叶非平板）
  const K_IND = tg.kInduced, K_VIS = tg.kViscous;
  const K_SEP = tg.kSeparation, SEP_ONSET = tg.sepOnset;
  const K_MACH = tg.kMach, MACH_ONSET = tg.machOnset;
  const VI_RELAX = tg.viRelax;

  const stations = [];
  let T = 0, Q = 0, viAvg = 0;

  for (let i = 0; i < NST; i++) {
    const rbar = r0 + (i + 0.5) * dr;
    const r = rbar * R;
    const drl = dr * R;

    // 几何桨距角：θ(r)=atan(P/2πr)（等桨距螺旋面），根部限幅
    const theta = Math.min(Math.atan2(PITCH, 2 * Math.PI * r), 0.72);
    // 叶尖收尖
    const chord = c0 * (rbar > 0.85 ? 1 - 0.35 * (rbar - 0.85) / 0.15 : 1);

    let vi = opts.vi0 ?? 0;
    let W = 1, phi = 0.1, Cl = 0.4, Cd = P.cd0, dT = 0, dQ = 0;

    for (let k = 0; k < INNER; k++) {
      const ur = omega * r;
      const va = Va + vi;
      phi = Math.atan2(va, ur);
      // 有效攻角：真实桨叶有弯度，零升力角为负
      const alpha = theta - phi;
      const alphaEff = alpha + CAMBER;
      W = Math.hypot(ur, va);
      const Re = Math.max(5e3, W * chord / nu);
      const a = clSlope(Re);
      const clMax = P.material.eff_stall * clMaxOf(Re);
      const alphaStall = clMax / a;
      const Clv = softClamp(a * alphaEff, clMax);
      // 失速后阻力急剧上升 + 高速可压缩性阻力（叶尖马赫数升高）
      const sep = Math.max(0, Math.abs(alphaEff) / alphaStall - SEP_ONSET);
      const mach = W / air.sound;
      const comp = K_MACH * Math.pow(Math.max(0, mach - MACH_ONSET), 2);
      const Cdv = P.cd0 + K_IND * Clv * Clv + K_VIS * Math.abs(Clv) + K_SEP * sep * sep + comp;
      Cl = Clv; Cd = Cdv;

      const q = 0.5 * rho * W * W * chord;
      dT = q * (Cl * Math.cos(phi) - Cd * Math.sin(phi)) * drl;

      // 普朗特叶尖损失
      const sinp = Math.max(0.05, Math.sin(phi));
      const ex = Math.exp(-B * (R - r) / (2 * r * sinp));
      const F = clamp((2 / Math.PI) * Math.acos(clamp(ex, 0, 0.9999)), 0.05, 1);

      // 动量理论反解诱导速度
      const disc = Math.max(1e-9, Math.PI * r * rho * F * drl);
      let viNew;
      if (dT > 0) {
        viNew = 0.5 * (-Va + Math.sqrt(Va * Va + dT / disc));
      } else {
        viNew = Math.min(0, vi * 0.5);
      }
      if (!Number.isFinite(viNew)) viNew = vi;
      vi = (1 - VI_RELAX) * vi + VI_RELAX * clamp(viNew, -20, 60);

      dQ = q * (Cl * Math.sin(phi) + Cd * Math.cos(phi)) * r * drl;
      if (!Number.isFinite(vi) || !Number.isFinite(dT)) { vi = 0; dT = 0; dQ = 0; }
    }

    T += dT;
    Q += dQ;
    viAvg += vi;
    stations.push({
      rbar: +rbar.toFixed(3), r: +r.toFixed(4), thetaDeg: deg(theta), phiDeg: deg(phi),
      alphaDeg: deg(theta - phi + CAMBER), Cl: +Cl.toFixed(3), Cd: +Cd.toFixed(4), W: +W.toFixed(2),
      dT: +(dT * 1000).toFixed(2), re: Math.round(W * chord / nu),
    });
  }

  const P_aero = Q * Math.abs(omega);
  const vi = viAvg / NST;
  const P_ideal = T > 0 ? Math.pow(T, 1.5) / Math.sqrt(2 * rho * A) : 0;
  const FM = P_aero > 1e-6 ? clamp01(P_ideal / P_aero) : 0;
  const vTip = Math.abs(omega) * R;
  return {
    T, Q, P_aero, vi, FM,
    eta: P_aero > 1e-6 ? (T * Math.abs(Va)) / P_aero : 0,
    advanceRatio: Math.abs(omega) > 1e-6 ? Math.PI * Math.abs(Va) / (Math.abs(omega) * R) : 0,
    vTip, machTip: vTip / air.sound,
    stations, omega, Va,
  };
}

// 桨叶截面升力线斜率随雷诺数变化（低 Re 明显掉）
function clSlope(Re) {
  if (Re < 3e4) return 5.0;
  if (Re < 6e4) return 5.4;
  if (Re < 2e5) return 5.7;
  return 5.9;
}
function clMaxOf(Re) {
  if (Re < 3e4) return 1.10;
  if (Re < 6e4) return 1.35;
  if (Re < 2e5) return 1.55;
  return 1.65;
}
function softClamp(x, lim) {
  const a = Math.abs(x);
  if (a <= lim * 0.8) return x;
  const over = (a - lim * 0.8) / lim;
  const sat = lim * 0.8 + lim * 0.2 * (1 - Math.exp(-over * 1.6)) / (1 + over * 0.6);
  return Math.sign(x) * Math.min(sat, lim * 1.02);
}
const deg = (x) => +(x * 180 / Math.PI).toFixed(1);
export { clamp01 };

// ============================================================
// 电机 + 电调 + 电池耦合工作点（以 ω 为主变量的直接解法）
// 稳态：I = Q_aero/Kt + I0 ; V_applied = ω/KVsi + I*R_m
//       duty = V_applied / (V_bus - I*R_escwire)
// ============================================================
const RPM_PER_RAD = 60 / (2 * Math.PI);

/** 直接由 ω 求工作点（单次 BEMT，无嵌套迭代） */
export function pointFromOmega({ omega, battV, rBatt, motor, esc, prop: P, air, Va = 0, protect = true }) {
  let aero = bemt(P, air, omega, Va);
  let throttled = false;
  let I = aero.Q / motor.Kt + motor.I0;

  // 热保护：电流超过允许值 → 扭矩被限制
  if (protect && omega > 1 && I > motor.iBurst) {
    throttled = true;
    const Qmax = motor.Kt * Math.max(0, motor.iBurst - motor.I0);
    omega = solveOmegaForTorque(Qmax, P, air, Va);
    aero = bemt(P, air, omega, Va);
    I = aero.Q / motor.Kt + motor.I0;
  }

  const R_escwire = esc.r + esc.rWire_mOhm / 1000;
  const V_applied = omega / motor.KVsi + I * motor.R_m;   // 反电动势 = ω/KVsi

  const pMech = Math.max(0, aero.Q * omega);
  const pCopper = I * I * motor.R_m;
  const pIron = omega * motor.I0 * motor.Kt;
  const pMotorIn = pMech + pCopper + pIron;
  const pEscWire = I * I * R_escwire;
  const pBus = pMotorIn + pEscWire;
  const pIn = pBus / Math.max(0.05, 1 - esc.swLoss);

  // 电池侧平均电流（PWM 斩波后 pack 电流 = 占空比 × 电机电流）+ 内阻压降
  // 解 I(battV - I·rBatt) = pIn
  let I_pack = pIn / battV;
  const disc = battV * battV - 4 * rBatt * pIn;
  if (rBatt > 0 && disc > 0) I_pack = (battV - Math.sqrt(disc)) / (2 * rBatt);
  const V_bus = Math.max(0, battV - I_pack * rBatt);
  const V_terminal = Math.max(0, V_bus - I * R_escwire);
  const duty = V_terminal > 1e-6 ? V_applied / V_terminal : (V_applied > 1e-9 ? 99 : 0);

  return {
    omega, rpm: omega * RPM_PER_RAD, duty,
    current: I_pack, currentMotor: I, currentPack: I_pack,
    vBus: V_bus, vTerminal: V_terminal, vApplied: V_applied, vBatt: battV,
    thrust: aero.T, torque: aero.Q, aero,
    pMech, pCopper, pIron, pMotorIn, pEscWire, pBus, pIn,
    pCell: battV * I_pack,
    pEscLoss: pEscWire + pBus * esc.swLoss,
    etaEsc: pIn > 1e-6 ? pBus / pIn : 0,
    etaMotor: pMotorIn > 1e-6 ? pMech / pMotorIn : 0,
    etaSystem: pIn > 1e-6 ? pMech / pIn : 0,
    gPerW: pIn > 1e-6 ? aero.T / 9.80665 * 1000 / pIn : 0,
    throttled, thermalLoad: pCopper,
    saturated: duty > 1.001,
  };
}

function solveOmegaForTorque(Qtarget, P, air, Va = 0) {
  let lo = 0, hi = 4000;
  if (bemt(P, air, 0, Va).Q >= Qtarget) return 0;
  for (let k = 0; k < 16; k++) {
    const mid = (lo + hi) / 2;
    if (bemt(P, air, mid, Va).Q < Qtarget) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}

/** 给定油门占空比 0..1 求工作点 */
export function operatingPoint(ctx) {
  const duty = clamp01(ctx.duty ?? 1);
  if (duty <= 0.002) return pointFromOmega({ ...ctx, omega: 0, protect: false });
  const wMax = ctx.motor.KVsi * ctx.battV + 1;
  let lo = 0, hi = wMax;
  // 探测点关闭热保护：二分只要求 duty(ω) 单调，热限幅由最终解给出。
  //（否则每个超流探测点都会触发 solveOmegaForTorque 的 16 步嵌套 BEMT）
  for (let k = 0; k < 15; k++) {
    const mid = (lo + hi) / 2;
    const d = pointFromOmega({ ...ctx, omega: mid, protect: false }).duty;
    if (d < duty) lo = mid; else hi = mid;
  }
  return pointFromOmega({ ...ctx, omega: (lo + hi) / 2 });
}

/** 反解：给定所需单桨推力与轴向来流，求工作点 */
export function dutyForThrust({ thrust, Va = 0, ...ctx }) {
  const top = operatingPoint({ ...ctx, duty: 1, Va, protect: true });
  if (top.thrust < thrust * 0.999) {
    return { duty: top.duty, reachable: false, op: top, needOmega: Infinity };
  }
  let lo = 0, hiW = ctx.motor.KVsi * ctx.battV;
  for (let k = 0; k < 16; k++) {
    const mid = (lo + hiW) / 2;
    const T = pointFromOmega({ ...ctx, omega: mid, Va, protect: false }).thrust;
    if (T < thrust) lo = mid; else hiW = mid;
  }
  const op = pointFromOmega({ ...ctx, omega: (lo + hiW) / 2, Va });
  return { duty: op.duty, reachable: op.duty <= 1.0001, op, needOmega: (lo + hiW) / 2 };
}

/** 满油门工作点：占空比到顶或电池总电流到限，取先到者（更接近真实可用满推力） */
export function fullPoint(ctx, maxPackCurrent_A = Infinity, motors = 1) {
  const op1 = operatingPoint({ ...ctx, duty: 1 });
  if (!(maxPackCurrent_A < Infinity) || maxPackCurrent_A <= 0) return op1;
  const packI = (w) => pointFromOmega({ ...ctx, omega: w, protect: false }).current * motors;
  if (packI(op1.omega) <= maxPackCurrent_A) return op1;
  let lo = 0, hi = op1.omega;
  for (let k = 0; k < 22; k++) {
    const mid = (lo + hi) / 2;
    if (packI(mid) < maxPackCurrent_A) lo = mid; else hi = mid;
  }
  const op = pointFromOmega({ ...ctx, omega: (lo + hi) / 2 });
  return { ...op, batteryLimited: true, maxPackCurrent_A, uncappedPackCurrent_A: op1.current * motors };
}
