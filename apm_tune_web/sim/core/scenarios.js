// ============================================================
// 飞行工况仿真：悬停 / 巡航 / 最高速 / 续航 / 加速（多旋翼 & 固定翼）
// ============================================================
import { G, clamp } from './parts.js';
import { pointFromOmega, dutyForThrust, operatingPoint, fullPoint, bemt } from './bemt.js';
import { bodyDrag, wingPolar, stallSpeed, wingGeom, rocketGeom, rocketPolar } from './aero.js';

const OCV_STEP = 200;   // 续航积分步长（无 BEMT，纯代数）

/** 电池放电积分：给定单桨所需 ω 与来流，算到电压/容量耗尽 */
function discharge({ bat, mot, esc, pr, air, n }, omega, Va, tMax = 5400) {
  const Ah = bat.capacity_mAh / 1000;
  const rBatt = bat.rPack_mOhm / 1000;
  let soc = 1, t = 0;
  const dt = 1;
  let limitedByC = false, cutByVoltage = false;
  let last = null;
  while (t < tMax) {
    const op = pointFromOmega({ omega, battV: bat.vCellAt(soc), rBatt, motor: mot, esc, prop: pr, air, Va });
    const Ipack = op.current * n;
    const vTerm = bat.vCellAt(soc) - Ipack * rBatt;
    if (vTerm <= bat.vCut + 0.02) { cutByVoltage = true; break; }
    if (Ipack > bat.maxCurrent_A) limitedByC = true;
    if (op.duty > 0.999 && t > 1) break;         // 电压不足无法维持
    last = { op, Ipack, vTerm, soc };
    soc -= (Ipack * dt) / (Ah * 3600);
    t += dt;
    if (soc <= 0.03) break;
  }
  return {
    seconds: t, minutes: t / 60,
    endSoc: soc, limitedByC, cutByVoltage,
    energy_Wh: last ? (last.op.pIn * n * t / 3600) : 0,
    last,
  };
}

// ------------------------------------------------------------
// 多旋翼
// ------------------------------------------------------------
export function multirotorCases({ bat, mot, esc, pr, air, n, frame, mass_kg, target_kmh, kind = 'quad' }) {
  const W = mass_kg * G;
  const rBatt = bat.rPack_mOhm / 1000;
  const ctx = { battV: bat.vCellAt(0.9), rBatt, motor: mot, esc, prop: pr, air };
  const massg = mass_kg * 1000;

  // ---- 悬停 ----
  const hoverSol = dutyForThrust({ ...ctx, thrust: W / n });
  const hover = hoverSol.op;
  const hoverDuty = hoverSol.duty;
  const hoverReachable = hoverSol.reachable;

  // ---- 满油门（满电 & 80% 电量）受电池放电能力限制 ----
  const full = fullPoint({ ...ctx, Va: 0 }, bat.maxCurrent_A, n);
  const maxThrust = full.thrust * n;
  const twr = maxThrust / W;

  // ---- 前向飞行 ----
  const dragAt = (V) => bodyDrag(air.rho, V, frame.frontal_m2, kind).D;
  const flightAt = (V, soc = 0.9, dutyCap = 1) => {
    const D = dragAt(V);
    const theta = Math.atan2(D, W);
    const Treq = Math.sqrt(W * W + D * D) / n;
    const Va = V * Math.sin(theta);
    const sol = dutyForThrust({ ...ctx, battV: bat.vCellAt(soc), thrust: Treq, Va });
    return { V, D, theta, tiltDeg: theta * 180 / Math.PI, Treq, Va, ...sol, gPerW: sol.op.pIn > 0 ? (Treq * n / G * 1000) / (sol.op.pIn * n) : 0 };
  };

  // ---- 最高速度 ----
  let lo = 0, hi = 400, vmax = 0;
  const atFull = (V) => {
    const D = dragAt(V);
    const theta = Math.atan2(D, W);
    const Treq = Math.sqrt(W * W + D * D) / n;
    const Va = V * Math.sin(theta);
    const op = fullPoint({ ...ctx, Va, battV: bat.vCellAt(0.9) }, bat.maxCurrent_A, n);
    return op.thrust - Treq;
  };
  if (atFull(0) > 0) {
    for (let k = 0; k < 22; k++) {
      const mid = (lo + hi) / 2;
      if (atFull(mid) > 0) lo = mid; else hi = mid;
    }
    vmax = (lo + hi) / 2;
  }

  // ---- 目标速度 ----
  const target = target_kmh > 0 ? flightAt(target_kmh / 3.6) : null;
  const targetReachable = target ? (target.reachable && target.duty <= 1.0001 && vmax >= target_kmh / 3.6) : false;

  // ---- 速度-功率扫描 ----
  const sweep = [];
  const vTop = Math.max(vmax, target_kmh / 3.6, 5);
  for (let i = 0; i <= 24; i++) {
    const V = (vTop * i) / 24;
    if (V < 0.5) { sweep.push({ V: 0, kmh: 0, power: hover.pIn * n, gPerW: hover.gPerW, current: hover.current * n, duty: hoverDuty, tiltDeg: 0, reachable: true }); continue; }
    const f = flightAt(V);
    sweep.push({
      V: +V.toFixed(2), kmh: +(V * 3.6).toFixed(1),
      power: f.op.pIn * n, gPerW: f.gPerW, current: f.op.current * n,
      duty: f.duty, tiltDeg: f.tiltDeg, reachable: f.reachable && f.duty <= 1.0001,
    });
  }
  const reach = sweep.filter(s => s.reachable && s.power > 0);
  const bestEndurance = reach.length ? reach.reduce((a, b) => (b.gPerW > a.gPerW ? b : a)) : null;
  const bestRange = reach.length && reach.length > 1
    ? reach.filter(s => s.kmh > 0).reduce((a, b) => (b.kmh / b.power > a.kmh / a.power ? b : a)) : null;

  // ---- 续航 ----
  const hoverDis = hoverReachable ? discharge({ bat, mot, esc, pr, air, n }, hover.omega, 0) : null;
  const cruiseOmega = bestEndurance ? flightAt(bestEndurance.V).needOmega : null;
  const cruiseDis = (cruiseOmega && Number.isFinite(cruiseOmega))
    ? discharge({ bat, mot, esc, pr, air, n }, cruiseOmega, bestEndurance.V * Math.sin(0.05)) : null;
  const targetDis = (targetReachable && target && Number.isFinite(target.needOmega))
    ? discharge({ bat, mot, esc, pr, air, n }, target.needOmega, target.Va) : null;

  // ---- 满油门电流 / 电池 C 数 ----
  const fullCurrent = full.current * n;

  // ---- 加速度（最大倾角受推重比限制，含阻力）----
  const maxTilt = clamp(Math.acos(clamp(W / Math.max(maxThrust, 1e-6), -1, 1)), 0.02, 60 * Math.PI / 180);
  const accel = (V) => Math.max(0, (maxThrust * Math.sin(maxTilt) - dragAt(V)) / mass_kg);
  // 数值积分 0 → 100 km/h
  let vAcc = 0, tAcc = 0;
  const vEnd = targetReachable && target ? Math.min(target.V, Math.max(vmax * 0.9, 1)) : 100 / 3.6;
  while (vAcc < vEnd && tAcc < 30) {
    const a = accel(vAcc);
    if (a <= 0.01) { tAcc = Infinity; break; }
    vAcc += a * 0.05; tAcc += 0.05;
  }

  return {
    W, massg, twr, maxThrust, hoverReachable,
    hover: { duty: hoverDuty, current: hover.current * n, currentMotor: hover.currentMotor * n, power: hover.pIn * n, rpm: hover.rpm, gPerW: hover.gPerW, etaMotor: hover.etaMotor, pMech: hover.pMech * n, vTip: hover.aero.vTip, FM: hover.aero.FM, vi: hover.aero.vi },
    full: { thrust: full.thrust * n, current: fullCurrent, rpm: full.rpm, power: full.pIn * n, gPerW: full.gPerW, vTip: full.aero.vTip, machTip: full.aero.machTip, FM: full.aero.FM, duty: full.duty, thermal: full.thermalLoad, throttled: full.throttled, batteryLimited: !!full.batteryLimited, currentDemand: full.uncappedPackCurrent_A != null ? full.uncappedPackCurrent_A : fullCurrent },
    vmax: vmax * 3.6,
    target: target ? {
      kmh: target_kmh, reachable: targetReachable, V: target.V,
      duty: target.duty, tiltDeg: target.tiltDeg, power: target.op.pIn * n,
      current: target.op.current * n, gPerW: target.gPerW, rpm: target.op.rpm,
      drag: target.D, Va: target.Va, Treq: target.Treq,
      minutes: targetDis ? targetDis.minutes : null,
    } : null,
    bestEndurance, bestRange,
    endurance: { hover: hoverDis ? hoverDis.minutes : null, cruise: cruiseDis ? cruiseDis.minutes : null },
    sweep,
    accelToTarget: Number.isFinite(tAcc) ? tAcc : null,
    hoverPoint: hover,
    maxTiltDeg: maxTilt * 180 / Math.PI,
    cRatingLimited: !!full.batteryLimited,
    batteryMaxCurrent: bat.maxCurrent_A,
  };
}

// ------------------------------------------------------------
// 垂起火箭（Tailsitter VTOL）：悬停（多旋翼模态）+ 转换 + 巡航（固定翼模态）
// ------------------------------------------------------------
export function vtolCases({ bat, mot, esc, pr, air, n = 4, rocket, mass_kg, target_kmh }) {
  const W = mass_kg * G;
  const rBatt = bat.rPack_mOhm / 1000;
  const ctx = { battV: bat.vCellAt(0.9), rBatt, motor: mot, esc, prop: pr, air };
  const rg = rocketGeom(rocket);
  const massg = mass_kg * 1000;

  // ================= 悬停模态（机身竖直，推力向上）=================
  const hoverSol = dutyForThrust({ ...ctx, thrust: W / n });
  const hover = hoverSol.op;
  const hoverDuty = hoverSol.duty;
  const hoverReachable = hoverSol.reachable;
  const full = fullPoint({ ...ctx, Va: 0 }, bat.maxCurrent_A, n);
  const maxThrust = full.thrust * n;
  const twr = maxThrust / W;

  // 垂直爬升（剩余推力全部用于爬升，忽略阻力变化）
  const vertClimb = (() => {
    if (!hoverReachable) return { vs_ms: 0, vs_fpm: 0, thrust: 0, margin: 0 };
    const excess = maxThrust - W;
    // 垂直上升阻力 = 机身侧向阻力（bodyD 截面 × cd）+ 桨本身
    const dragAtV = (V) => bodyDrag(air.rho, V, rocket.frontal_m2, 'rocket').D;
    let lo = 0, hi = 60;
    for (let k = 0; k < 20; k++) {
      const mid = (lo + hi) / 2;
      const op = fullPoint({ ...ctx, Va: mid }, bat.maxCurrent_A, n);
      if (op.thrust * n - W - dragAtV(mid) > 0) lo = mid; else hi = mid;
    }
    const vs = (lo + hi) / 2;
    return { vs_ms: vs, vs_fpm: vs * 196.85, thrust: maxThrust, margin: excess / W };
  })();

  // ================= 巡航模态（机身水平，尾翼+升力体）=================
  const polar = (V) => rocketPolar(rg, air.rho, air.mu, V, W);
  const dragBody = (V) => bodyDrag(air.rho, V, rocket.frontal_m2, 'rocket').D;
  // 巡航阻力 = 气动阻力 + 小机身附加阻力（尾翼已计入 Sref）
  const cruiseDrag = (V) => polar(V).D * 0.94 + dragBody(V) * 0.35;

  const flightAt = (V, soc = 0.9) => {
    const p = polar(V);
    const D = cruiseDrag(V);
    const sol = dutyForThrust({ ...ctx, battV: bat.vCellAt(soc), thrust: D, Va: V });
    return { V, polar: p, D, ...sol, gPerW: sol.op.pIn > 0 ? mass_kg * 1000 / (sol.op.pIn * n) : 0 };
  };

  const vStall = stallSpeed(W, air.rho, rg);
  let lo = vStall * 0.9, hi = 400, vmax = vStall;
  const atFull = (V) => {
    const op = fullPoint({ ...ctx, Va: V }, bat.maxCurrent_A, n);
    return op.thrust * n - cruiseDrag(V);
  };
  if (atFull(hi) > 0) {
    for (let k = 0; k < 22; k++) { const mid = (lo + hi) / 2; if (atFull(mid) > 0) lo = mid; else hi = mid; }
    vmax = (lo + hi) / 2;
  } else if (atFull(lo) > 0) {
    for (let k = 0; k < 22; k++) { const mid = (lo + hi) / 2; if (atFull(mid) > 0) lo = mid; else hi = mid; }
    vmax = (lo + hi) / 2;
  }

  // 巡航速度扫描
  const sweep = [];
  const vLo = Math.max(0.7 * vStall, 4);
  const vHi = Math.max(vmax, vLo * 1.2);
  for (let i = 0; i <= 24; i++) {
    const V = vLo + (vHi - vLo) * i / 24;
    const f = flightAt(V);
    sweep.push({
      V: +V.toFixed(2), kmh: +(V * 3.6).toFixed(1),
      CL: f.polar.CL, CD: f.polar.CD, L_D: f.polar.L_D, alphaDeg: f.polar.alphaDeg,
      power: f.op.pIn * n, current: f.op.current * n, duty: f.duty,
      reachable: f.reachable && f.duty <= 1.0001, rpm: f.op.rpm, gPerW: f.gPerW,
    });
  }
  const ok = sweep.filter(s => s.reachable && !(s.CL > rg.clmax));
  const bestLD = ok.length ? ok.reduce((a, b) => (b.L_D > a.L_D ? b : a)) : null;
  const bestEnd = ok.length ? ok.reduce((a, b) => (b.gPerW > a.gPerW ? b : a)) : null;

  const target = target_kmh > 0 ? flightAt(target_kmh / 3.6) : null;
  const targetOk = !!(target && target.reachable && target.duty <= 1.0001 && !target.polar.stalled);

  return {
    W, massg, n, rocketGeom: rg,
    twr, maxThrust, hoverReachable,
    hover: { duty: hoverDuty, current: hover.current * n, power: hover.pIn * n, rpm: hover.rpm, gPerW: hover.gPerW, vTip: hover.aero.vTip, FM: hover.aero.FM, vi: hover.aero.vi, machTip: hover.aero.machTip },
    full: { thrust: full.thrust * n, current: full.current * n, rpm: full.rpm, power: full.pIn * n, gPerW: full.gPerW, vTip: full.aero.vTip, machTip: full.aero.machTip, thermal: full.thermalLoad, duty: full.duty, throttled: full.throttled, batteryLimited: !!full.batteryLimited, currentDemand: full.uncappedPackCurrent_A != null ? full.uncappedPackCurrent_A : full.current * n },
    vStall: vStall * 3.6, vmax: vmax * 3.6,
    pitchSpeed_kmh: ((mot.KVsi * bat.vCellAt(0.9) * 60) / (2 * Math.PI) / 60) * pr.P * 3.6,
    maxLD: bestLD ? bestLD.L_D : null, maxLDSpeed: bestLD ? bestLD.kmh : null,
    minSinkSpeed: bestEnd ? bestEnd.kmh : null,
    bestLD, bestEndurance: bestEnd,
    vertClimb, sweep,
    target: target ? {
      kmh: target_kmh, reachable: targetOk, V: target.V, CL: target.polar.CL, CD: target.polar.CD,
      L_D: target.polar.L_D, duty: target.duty, power: target.op.pIn * n, current: target.op.current * n,
      rpm: target.op.rpm, stalled: target.polar.stalled, alphaDeg: target.polar.alphaDeg,
    } : null,
    endurance: { hover: hoverReachable ? dischargeHover({ bat, mot, esc, pr, air, n }, hover.omega) : null, cruise: null },
    cRatingLimited: !!full.batteryLimited,
    batteryMaxCurrent: bat.maxCurrent_A,
    hoverPoint: hover,
  };
}

/** 悬停续航积分（纯代数，快） */
function dischargeHover({ bat, mot, esc, pr, air, n }, omega) {
  const Ah = bat.capacity_mAh / 1000;
  const rBatt = bat.rPack_mOhm / 1000;
  let soc = 1, t = 0;
  while (t < 5400) {
    const op = pointFromOmega({ omega, battV: bat.vCellAt(soc), rBatt, motor: mot, esc, prop: pr, air, Va: 0 });
    const Ipack = op.current * n;
    if (bat.vCellAt(soc) - Ipack * rBatt <= bat.vCut + 0.02) break;
    if (op.duty > 0.999 && t > 1) break;
    soc -= (Ipack / (Ah * 3600));
    t += 1;
    if (soc <= 0.03) break;
  }
  return t / 60;
}

// ------------------------------------------------------------
// 固定翼
// ------------------------------------------------------------
export function fixedWingCases({ bat, mot, esc, pr, air, n = 1, wing, mass_kg, target_kmh, payload_kg = 0 }) {
  const W = mass_kg * G;
  const rBatt = bat.rPack_mOhm / 1000;
  const ctx = { battV: bat.vCellAt(0.9), rBatt, motor: mot, esc, prop: pr, air };
  const wg = wingGeom(wing);

  const vStall = stallSpeed(W, air.rho, wg);
  const polar = (V) => wingPolar(wg, air.rho, air.mu, V, W);

  // 静态最大推力（地面拉力，用于结构校核与起飞评估），受电池放电能力限制
  const staticMax = fullPoint({ ...ctx, Va: 0 }, bat.maxCurrent_A, n).thrust * n;

  // 平飞：需要推力 = 阻力，螺旋桨带前进来流
  const flightAt = (V, soc = 0.9) => {
    const p = polar(V);
    const sol = dutyForThrust({ ...ctx, battV: bat.vCellAt(soc), thrust: p.D, Va: V });
    return { V, polar: p, ...sol, gPerW: sol.op.pIn > 0 ? mass_kg * 1000 / (sol.op.pIn * n) : 0 };
  };

  // 最高速度：满油门可用推力 = 阻力
  let lo = vStall * 0.9, hi = 400, vmax = vStall;
  const atFull = (V) => {
    const p = polar(V);
    const op = fullPoint({ ...ctx, Va: V }, bat.maxCurrent_A, n);
    return op.thrust * n - p.D;
  };
  if (atFull(hi) > 0) {
    for (let k = 0; k < 22; k++) { const mid = (lo + hi) / 2; if (atFull(mid) > 0) lo = mid; else hi = mid; }
    vmax = (lo + hi) / 2;
  } else if (atFull(lo) > 0) {
    for (let k = 0; k < 22; k++) { const mid = (lo + hi) / 2; if (atFull(mid) > 0) lo = mid; else hi = mid; }
    vmax = (lo + hi) / 2;
  }

  // 极曲线扫描
  const sweep = [];
  const cache = new Map();
  const vLo = Math.max(0.6 * vStall, 3);
  const vHi = Math.max(vmax, vLo * 1.2);
  for (let i = 0; i <= 24; i++) {
    const V = vLo + (vHi - vLo) * i / 24;
    const f = flightAt(V);
    cache.set(+V.toFixed(3), f);
    sweep.push({
      V: +V.toFixed(2), kmh: +(V * 3.6).toFixed(1), CL: f.polar.CL, CD: f.polar.CD,
      L_D: f.polar.L_D, power: f.op.pIn * n, current: f.op.current * n,
      duty: f.duty, reachable: f.reachable && f.duty <= 1.0001,
      rpm: f.op.rpm, etaProp: f.op.aero.eta, gPerW: f.gPerW,
    });
  }
  const ok = sweep.filter(s => s.reachable && s.CL <= wg.clmax);
  const bestLD = ok.length ? ok.reduce((a, b) => (b.L_D > a.L_D ? b : a)) : null;
  const bestEnd = ok.length ? ok.reduce((a, b) => (b.power < a.power ? b : a)) : null;
  const bestEndF = bestEnd ? cache.get(bestEnd.V) : null;
  const bestLDF = bestLD ? cache.get(bestLD.V) : null;

  const target = target_kmh > 0 ? flightAt(target_kmh / 3.6) : null;
  const targetOk = !!(target && target.reachable && target.duty <= 1.0001 && target.polar.CL <= wg.clmax);

  // 续航 / 航程
  const dis = (f) => (f && Number.isFinite(f.needOmega) && f.reachable)
    ? discharge({ bat, mot, esc, pr, air, n }, f.needOmega, f.V) : null;
  const targetDis = dis(target);
  const endDis = bestEnd ? dis(flightAt(bestEnd.V)) : null;

  // 爬升率（75% 油门）
  const climbInfo = (() => {
    const duty = 0.9;
    const V = Math.max(vStall * 1.2, 15);
    const p = polar(V);
    const op = operatingPoint({ ...ctx, duty, Va: V });
    const excess = op.thrust * n - p.D;
    const vs = excess > 0 ? (excess * V) / W : 0;
    // 垂直爬升（悬停式拉起，仅看推重）
    const vert = (op.thrust * n - W) / W;
    return { duty, V, thrust: op.thrust * n, drag: p.D, excess, vs_ms: vs, vs_kmh: vs * 3.6, vs_fpm: vs * 196.85, thrustMargin: vert };
  })();

  const climb80 = (() => {
    const V = Math.max(vStall * 1.4, 20);
    const p = polar(V);
    const op = fullPoint({ ...ctx, Va: V }, bat.maxCurrent_A, n);
    const excess = op.thrust * n - p.D;
    return { vs_ms: excess > 0 ? (excess * V) / W : 0, thrust: op.thrust * n, drag: p.D };
  })();

  // 桨距速度上限（无载几何螺距速度）：固定翼极速的物理天花板之一
  const omegaNL = mot.KVsi * bat.vCellAt(0.9);
  const rpmNL = (omegaNL * 60) / (2 * Math.PI);
  const pitchSpeed = (rpmNL / 60) * pr.P;

  return {
    W, wing: wg, vStall: vStall * 3.6, vmax: vmax * 3.6,
    pitchSpeed_kmh: pitchSpeed * 3.6,
    maxThrust: staticMax, twr: staticMax / W,
    wingArea: wg.area, AR: wg.AR,
    minSinkSpeed: bestEnd ? bestEnd.kmh : null,
    maxLDSpeed: bestLD ? bestLD.kmh : null,
    maxLD: bestLD ? bestLD.L_D : null,
    bestEndOp: bestEndF ? bestEndF.op : null,
    bestEndPoint: bestEndF || null,
    target: target ? {
      kmh: target_kmh, reachable: targetOk, V: target.V, CL: target.polar.CL,
      CD: target.polar.CD, L_D: target.polar.L_D, duty: target.duty,
      power: target.op.pIn * n, current: target.op.current * n, rpm: target.op.rpm,
      etaProp: target.op.aero.eta, minutes: targetDis ? targetDis.minutes : null,
      range_km: targetDis && targetDis.minutes ? target_kmh * targetDis.minutes / 60 : null,
      stalled: target.polar.CL > wg.clmax,
    } : null,
    bestEndurance: bestEnd, bestLD, sweep,
    endurance: { max: endDis ? endDis.minutes : null },
    maxRange_km: null,
    climb: climbInfo, climb80,
    hoverLike: null,
  };
}
