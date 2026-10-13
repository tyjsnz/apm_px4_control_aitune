// ============================================================
// 仿真总入口：参数解析 → 部件建模 → 工况求解 → 结构/控制 → 报告
// ============================================================
import { airModel } from './air.js';
import { battery, motor, esc, prop, FRAMES, WINGS, MOTORS, ESCS, CHEMISTRIES, PROP_MATERIALS, ROCKETS, G, clamp } from './parts.js';
import { multirotorCases, fixedWingCases, vtolCases } from './scenarios.js';
import { structure, inertia, actuatorLag, attitudeControl, thrustCurve } from './dynamics.js';
import { noiseDb, groundEffect, wingPolar, rocketGeom, rocketPolar } from './aero.js';
import { transitionCorridor, hoverStability } from './vtol.js';
import { stabilityAnalysis, navigationCases } from './navigation.js';
import { optimizeHover } from './optimize.js';
import { diagnose, buildReport, flightPlan, linkSolutionRefs } from './analysis.js';
import { solveForTargets } from './solver.js';
import { setTuning, tuningDiff, T } from './tuning.js';

export const PRESETS = {
  frames: FRAMES, wings: WINGS, rockets: ROCKETS, motors: MOTORS, escs: ESCS,
  chemistries: Object.values(CHEMISTRIES), materials: Object.values(PROP_MATERIALS),
  rotors: [
    { id: 'quad', label: '四旋翼', n: 4 },
    { id: 'hex', label: '六旋翼', n: 6 },
    { id: 'octo', label: '八旋翼', n: 8 },
  ],
};

export function normalize(input = {}) {
  const rawType = input.type;
  const type = rawType === 'fixedwing' ? 'fixedwing' : rawType === 'vtol' ? 'vtol' : 'multirotor';
  const frameId = FRAMES.some(f => f.id === input.frameId) ? input.frameId : 'fpv5';
  const frame = FRAMES.find(f => f.id === frameId);
  const wingId = WINGS.some(w => w.id === input.wingId) ? input.wingId : 'wing1000';
  const wing = WINGS.find(w => w.id === wingId);
  const rocketId = ROCKETS.some(r => r.id === input.rocketId) ? input.rocketId : 'rocket8';
  const rocket = ROCKETS.find(r => r.id === rocketId);
  const motorId = MOTORS.some(m => m.id === input.motorType)
    ? input.motorType : (type === 'multirotor' ? '2207' : type === 'vtol' ? '28065' : '4114');
  const escId = ESCS.some(e => e.id === input.escType) ? input.escType : 'shott32';
  const nMotors = type !== 'fixedwing'
    ? (PRESETS.rotors.some(r => r.n === input.rotorCount) ? input.rotorCount : (type === 'vtol' ? rocket.motors : (frame.motors || 4)))
    : clamp(Math.round(input.nMotors || 1), 1, 2);

  // 各类型的默认电池/桨（火箭 8 寸 6S 为基准）
  const batDefault = type === 'vtol' ? rocket.batteryDefault : frame.batteryDefault;
  const kvDefault = MOTORS.find(m => m.id === motorId)?.KV0 ?? (type === 'vtol' ? 1300 : 1750);

  return {
    type,
    frameId, wingId, rocketId, motorType: motorId, escType: escId, nMotors,
    kv: clamp(Number(input.kv) || kvDefault, 80, 40000),
    cells: clamp(Math.round(Number(input.cells) || batDefault?.cells || 6), 1, 24),
    capacity_mAh: clamp(Math.round(Number(input.capacity_mAh) || batDefault?.capacity_mAh || 1500), 100, 100000),
    chemistry: CHEMISTRIES[input.chemistry] ? input.chemistry : 'lipo',
    cRating: Number(input.cRating) > 0 ? Number(input.cRating) : null,
    propD: clamp(Number(input.propD) || (type === 'fixedwing' ? 203 : type === 'vtol' ? 203 : 127), 30, 700),
    propP: clamp(Number(input.propP) || (type === 'fixedwing' ? 152 : type === 'vtol' ? 114 : 109), 20, 500),
    propB: clamp(Math.round(Number(input.propB) || 3), 2, 14),
    propMaterial: PROP_MATERIALS[input.propMaterial] ? input.propMaterial : 'plastic',
    wireAwg: [12, 14, 16, 18, 20, 22].includes(Number(input.wireAwg)) ? Number(input.wireAwg) : (type === 'vtol' ? 14 : 16),
    wireLen_cm: clamp(Number(input.wireLen_cm) || 10, 2, 60),
    target_kmh: clamp(Number(input.target_kmh) || 0, 0, 600),
    payload_g: clamp(Number(input.payload_g) || 0, 0, 50000),
    massOverride_g: Number(input.massOverride_g) > 0 ? Number(input.massOverride_g) : null,
    altitude: clamp(Number(input.altitude) || 0, 0, 8000),
    tempC: clamp(Number(input.tempC) || 25, -30, 55),
    arm_m: Number(input.arm_m) > 0 ? Number(input.arm_m) : null,
    wingSpan: Number(input.wingSpan) > 0 ? Number(input.wingSpan) : null,
    wingArea: Number(input.wingArea) > 0 ? Number(input.wingArea) : null,
    // 调参表（半经验系数/判据阈值），原样透传；normalize 不做范围校验，由 setTuning 负责
    tuning: (input.tuning && typeof input.tuning === 'object') ? input.tuning : null,
  };
}

export function simulate(raw) {
  // 求解器标志（normalize 会丢弃未知字段，须在归一化前读取）：
  //   _skipSolver：不再触发反向求解（避免递归）
  //   _light     ：轻量评估模式（跳过优化引擎/推力曲线/报告生成，仅保留物理指标）
  const skipSolver = !!(raw && (raw._skipSolver || raw._light));
  const light = !!(raw && raw._light);
  const cfg = normalize(raw);
  // 应用调参表（半经验系数 + 判据阈值）。必须在任何物理计算之前。
  // 求解器递归评估时 baseInput 携带 tuning，因此子调用会重新应用同一张表，保持一致。
  setTuning(cfg.tuning);
  const air = airModel({ altitude: cfg.altitude, tempC: cfg.tempC });

  const bat = battery({ cells: cfg.cells, capacity_mAh: cfg.capacity_mAh, chemistry: cfg.chemistry, cRating: cfg.cRating });
  const mot = motor({ type: cfg.motorType, kv: cfg.kv });
  const e = esc({ type: cfg.escType, wireAwg: cfg.wireAwg, wireLen_cm: cfg.wireLen_cm });
  const pr = prop({ d_mm: cfg.propD, p_mm: cfg.propP, blades: cfg.propB, material: cfg.propMaterial });

  const isMR = cfg.type === 'multirotor';
  const isVTOL = cfg.type === 'vtol';
  const isMRish = isMR || isVTOL;          // 有悬停模态（需速率环 PID）
  const frame = FRAMES.find(f => f.id === cfg.frameId);
  const wing = WINGS.find(w => w.id === cfg.wingId);
  const rocket = ROCKETS.find(r => r.id === cfg.rocketId);
  const arm_m = cfg.arm_m ?? (isMR ? frame.arm_m : isVTOL ? rocket.arm_m : 0.5);
  const n = cfg.nMotors;
  const airframe_g = isMR ? frame.frame_g : isVTOL ? rocket.frame_g : wing.mass_g;

  // ---------- 质量分解 ----------
  const mass = {
    frame_g: airframe_g,
    motors_g: n * mot.mass_g,
    props_g: n * pr.mass_g,
    battery_g: bat.mass_g,
    esc_g: isMRish ? (10 + n * 5 + cfg.wireLen_cm * 0.12) : (6 + cfg.wireLen_cm * 0.12),
    avionics_g: isMRish ? 46 : 40,       // 飞控+图传+摄像头+天线
    payload_g: cfg.payload_g,
  };
  mass.total_g = Object.values(mass).reduce((a, b) => a + b, 0);
  const totalMass = cfg.massOverride_g ?? mass.total_g;
  const mass_kg = totalMass / 1000;

  // ---------- 工况 ----------
  const ctx = { bat, mot, esc: e, pr, air, n };
  let cases;
  if (isMR) {
    cases = multirotorCases({ ...ctx, frame, mass_kg, target_kmh: cfg.target_kmh, kind: frame.kind });
  } else if (isVTOL) {
    cases = vtolCases({ ...ctx, rocket, mass_kg, target_kmh: cfg.target_kmh });
  } else {
    const wg = { span: cfg.wingSpan ?? wing.span, area: cfg.wingArea ?? wing.area, cd0: wing.cd0, clmax: wing.clmax, e: wing.e };
    cases = fixedWingCases({ ...ctx, wing: wg, mass_kg, target_kmh: cfg.target_kmh });
  }

  // ---------- 悬停/基准工作点 ----------
  const basePoint = isMRish ? cases.hoverPoint : null;
  const hoverOmega = isMRish ? cases.hoverPoint.omega : 0;

  // ---------- 结构 / 控制 ----------
  const struct = structure({
    massParts: mass, arm_m, frame_g: mass.frame_g, prop: pr, motor: mot,
    maxThrustN: cases.maxThrust ?? (cases.climb?.thrust ?? mass_kg * G * 2),
    hoverThrustN: mass_kg * G, n, kind: isMR ? frame.kind : isVTOL ? 'quad' : 'fixed',
    maxRPM: Math.max(1000, isMRish ? cases.full.rpm : (cases.sweep.at(-1)?.rpm ?? 20000)),
  });
  const iner = inertia({ mass_kg, arm_m, motorMass_g: mot.mass_g, propMass_g: pr.mass_g, frame_g: mass.frame_g, battery_g: bat.mass_g, n: isMRish ? n : 1 });
  const lag = actuatorLag({ prop: pr, motor: mot, hoverOmega: hoverOmega || 600 });

  let pid = null, thrustCurveData = null;
  if (isMRish && !light) {
    const hoverThrustTotal = mass_kg * G;
    const maxT = cases.maxThrust;
    pid = attitudeControl({ Ixx: iner.Ixx, arm_m, maxThrustN: maxT, hoverThrustN: hoverThrustTotal, actuatorTau_s: lag.tau_ms / 1000, n, kind: isMR ? frame.kind : 'quad' });
    thrustCurveData = thrustCurve({ battV: bat.vCellAt(0.9), rBatt: bat.rPack_mOhm / 1000, motor: mot, esc: e, prop: pr, air }, n, 21);
  }

  // ---------- 垂起火箭：转换走廊 + 悬停稳定 ----------
  let vtolInfo = null;
  if (isVTOL) {
    const rg = rocketGeom(rocket);
    const vStall_ms = cases.vStall / 3.6;
    vtolInfo = {
      transition: transitionCorridor({
        bat, mot, esc: e, pr, air, n, rg, mass_kg, vStall_ms,
        frontal_m2: rocket.frontal_m2, arm_m, Ixx: iner.Ixx, pitchSteps: 13,
      }),
      hoverStability: hoverStability({
        rocket, Ixx: iner.Ixx, arm_m,
        maxThrustN: cases.maxThrust, hoverThrustN: mass_kg * G, mass_kg,
      }),
      rocketGeom: rg,
    };
  }

  // ---------- 固定翼 / 火箭巡航：增稳 + 导航 ----------
  let stability = null, navigation = null;
  if (!isMR) {
    const aerog = isVTOL
      ? { span: vtolInfo.rocketGeom.bfin, area: vtolInfo.rocketGeom.Sref, AR: vtolInfo.rocketGeom.AR, cd0: rocket.cd0, clmax: rocket.finClmax, e: 0.62 }
      : (() => {
          const span = cfg.wingSpan ?? wing.span;
          const area = cfg.wingArea ?? wing.area;
          return { span, area, AR: wing.AR ?? (area > 0 ? (span * span) / area : 6), cd0: wing.cd0, clmax: wing.clmax, e: wing.e };
        })();
    stability = stabilityAnalysis({
      wg: aerog, rho: air.rho, mu: air.mu, W_N: cases.W,
      Ixx: iner.Ixx, Iyy: iner.Iyy, Izz: iner.Izz, mass_kg,
    });
    const Vcruise = (cases.maxLDSpeed || cases.minSinkSpeed || cases.vStall * 1.5) / 3.6;
    const cnBetaVTOL = isVTOL
      ? (() => {
          const rg = vtolInfo.rocketGeom;
          const lf = Math.max(rocket.bodyLength_m * 0.5, 0.05);
          return clamp(1.0 * (rg.Sfin * lf) / Math.max(rg.Sref * rg.bfin, 1e-6) * rg.aFin, 0.03, 0.35);
        })()
      : null;
    navigation = navigationCases({
      wg: aerog, rho: air.rho, W_N: cases.W, mass_kg,
      vStall_ms: cases.vStall / 3.6,
      maxLDSpeed_ms: (cases.maxLDSpeed || cases.vStall * 1.6) / 3.6,
      minSinkSpeed_ms: (cases.minSinkSpeed || cases.vStall * 1.4) / 3.6,
      polarAt: isVTOL
        ? (V) => rocketPolar(vtolInfo.rocketGeom, air.rho, air.mu, V, cases.W)
        : (V) => wingPolar(aerog, air.rho, air.mu, V, cases.W),
      powerAt: (V) => {
        const f = cases.sweep.reduce((best, s) => (Math.abs(s.V - V) < Math.abs(best.V - V) ? s : best), cases.sweep[0]);
        return { pIn: f.power, current: f.current, duty: f.duty };
      },
      maxThrust_N: cases.maxThrust,
      endurance_max_min: cases.endurance?.max ?? cases.endurance?.hover ?? null,
      usableEnergy_Wh: bat.energy_Wh * 0.8,
      Cnbeta: cnBetaVTOL,
    });
  }

  // ---------- 流体 ----------
  const fullRPM = isMRish ? cases.full.rpm : (cases.sweep.at(-1)?.rpm ?? 0);
  const vTip = pr.tipSpeedAt(fullRPM || 10000);
  const hoverStation = basePoint?.aero?.stations?.[2];
  const reBlade = hoverStation?.re ?? Math.round((vTip * pr.c) / air.nu);
  const vRef = isMRish ? Math.max(5, (cases.vmax ?? 30) * 0.5) : (cases.maxLDSpeed ? cases.maxLDSpeed / 3.6 : 20);
  const bodyL = isMRish ? (isVTOL ? rocket.bodyLength_m : 0.30) : (cfg.wingSpan ?? wing.span) / 8;
  const reBody = Math.round(vRef * bodyL / air.nu);
  const ge = isMRish ? groundEffect(pr.D * 0.5, pr.D) : { factor: 1, note: '固定翼无地面效应影响（起降阶段除外）', dropPct: 0 };
  const fluid = {
    rho: air.rho, altitude: air.altitude, tempC: air.tempC,
    reBlade, reBody,
    regime: reBlade < 5e4 ? '低雷诺数：层流分离、升力线斜率下降' : reBlade < 2e5 ? '过渡区：分离泡与转捩主导' : '充分湍流：附着良好',
    noise1m: noiseDb({ vTip, blades: pr.B, D: pr.D, dist_m: 1 }),
    noise10m: noiseDb({ vTip, blades: pr.B, D: pr.D, dist_m: 10 }),
    tipSpeed: vTip, mach: vTip / air.sound,
    groundEffect: ge, dropPct: ge.dropPct ?? Math.max(0, (ge.factor - 1) / ge.factor * 100),
    transITION: reBlade < 5e4 ? 65 : reBlade < 2e5 ? 45 : 25,
    downwash: basePoint ? basePoint.aero.vi : null,
  };

  // ---------- 能量链（悬停/最佳巡航）----------
  const energy = isMRish ? {
    pBatt: cases.hover.power, pEsc: cases.hover.power * e.swLoss + (cases.hover.current / n) ** 2 * (e.r + e.rWire_mOhm / 1000) * n,
    pCopper: basePoint.pCopper * n, pIron: basePoint.pIron * n, pMech: basePoint.pMech * n,
    etaEsc: basePoint.etaEsc, etaMotor: basePoint.etaMotor,
  } : (() => {
    const op = cases.bestEndPoint?.op;
    if (!op) return { pBatt: 0, pEsc: 0, pCopper: 0, pIron: 0, pMech: 0, etaEsc: 1, etaMotor: 0.8 };
    return {
      pBatt: op.pIn * n, pEsc: op.pIn * n * e.swLoss + op.pEscWire * n,
      pCopper: op.pCopper * n, pIron: op.pIron * n, pMech: op.pMech * n,
      etaEsc: op.etaEsc, etaMotor: op.etaMotor,
    };
  })();

  // ---------- 悬停效率优化建议引擎（悬停型机型）----------
  let optimize = null;
  if (isMRish && !light) {
    // 桨位约束：X 布局相邻电机间距 = arm·√2（留 5% 余量）
    const maxProp_mm = arm_m * Math.SQRT2 * 0.95 * 1000;
    optimize = optimizeHover({ base: cfg, air, mass_kg, n, maxProp_mm });
  }

  // ---------- 汇总 ----------
  const R = {
    cfg, type: cfg.type,
    typeLabel: isMR ? `${frame.label}` : isVTOL ? `${rocket.label}` : `${wing.label}`,
    air, battery: { ...bat, enduranceHoverMin: cases.endurance?.hover ?? null },
    motor: mot, esc: e, prop: pr, frame, wing, rocket, nMotors: n, arm_m,
    mass, totalMass_g: totalMass,
    cases, fluid, energy,
    structure: struct,
    dynamics: { inertia: iner, actuator: lag, pid, thrustCurve: thrustCurveData },
    vtol: vtolInfo, stability, navigation, optimize,
    batteryInfo: { label: bat.label, energy_Wh: bat.energy_Wh, maxCurrent_A: bat.maxCurrent_A, rPack_mOhm: bat.rPack_mOhm, vFull: bat.vFull, vCut: bat.vCut, cells: bat.cells },
    // 本次实际生效的调参覆盖（相对默认值的差异），供前端高亮"已改"项
    tuningApplied: tuningDiff(),
    // 本次实际生效的判据阈值：前端 KPI 徽章/着色必须用同一套，避免"诊断变了颜色没变"
    thresholds: (() => {
      const d = T().diag;
      return {
        twrError: d.twrError, twrWarn: d.twrWarn, twrHighMR: d.twrHighMR, twrHighVTOL: d.twrHighVTOL,
        hoverDutyError: d.hoverDutyError, hoverDutyWarn: d.hoverDutyWarn, hoverDutyMin: d.hoverDutyMin,
        tipMach: d.tipMach, noise10m: d.noise10m, enduranceMin_min: d.enduranceMin_min,
        armSFError: d.armSFError, armSFWarn: d.armSFWarn, ldMin: d.ldMin, vtolClimbMin: d.vtolClimbMin,
      };
    })(),
  };
  R.battery = { ...R.battery, ...R.batteryInfo };
  R.issues = diagnose(R);
  // 目标驱动反向求解：仅当存在 error/warn issue 时触发（求解器自身的评估调用带 _skipSolver 防递归）
  const hasFailure = R.issues.some(i => i.level === 'error' || i.level === 'warn');
  R.solutions = (!skipSolver && hasFailure)
    ? solveForTargets(R, cfg)
    : { solutions: [], sensitivity: {}, pareto: [], scanned: 0, baseMetrics: null, hasFailures: false };
  linkSolutionRefs(R.issues, R.solutions);   // diagnose 先于求解执行，补挂 solutionRef
  if (light) { R.report = []; R.plan = []; }
  else {
    R.report = buildReport(R);
    R.plan = flightPlan(R);
  }
  return R;
}

export { wingPolar };
