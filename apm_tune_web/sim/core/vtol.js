// ============================================================
// 垂起火箭（Tailsitter）转换段分析
// 机身从竖直（φ=90°，悬停）转到接近水平（φ=5°，巡航）的整个走廊：
//   水平航迹下机头仰角 φ 同时是机身攻角 α，对每个 φ 求解
//     竖直：T·sinφ + L·cosφ = W
//     水平：T·cosφ − L·sinφ − D = 0
//   得到该姿态的平衡速度 V*、需用推力 Treq，与满油门可用推力比较，
//   并检查差动推力控制权限（火箭靠差动推力控姿态，无舵面）。
// 转换段是垂起火箭最危险的阶段：速度低尾翼失速无升力、速度高则
// 动力被巡航阻力吃掉，必须整条走廊都有正裕度。
// ============================================================
import { G, clamp } from './parts.js';
import { fullPoint } from './bemt.js';
import { rocketPolar, bodyDrag } from './aero.js';

// 给定攻角的升力系数（线性段 → 尾翼失速 → 平板大攻角衰减）
function clAtAlpha(rg, alpha) {
  const as = rg.alphaStallDeg * Math.PI / 180;
  const a = Math.abs(alpha);
  if (a <= as) return Math.sign(alpha) * rg.aTotal * a;
  const s = Math.sin(2 * a) / Math.sin(2 * as);
  return Math.sign(alpha) * Math.min(rg.clmax * s, rg.clmax * 1.05);
}

/**
 * @param {object} p bat,mot,esc,pr,air,n,rg(rocketGeom),mass_kg,vStall_ms,
 *                   frontal_m2,arm_m,Ixx,pitchSteps
 */
export function transitionCorridor({ bat, mot, esc, pr, air, n = 4, rg, mass_kg, vStall_ms,
  frontal_m2 = 0.0064, arm_m = 0.118, Ixx = 0.0033, pitchSteps = 17 }) {
  const W = mass_kg * G;
  const rBatt = bat.rPack_mOhm / 1000;
  const ctx = { battV: bat.vCellAt(0.9), rBatt, motor: mot, esc, prop: pr, air };
  const rho = air.rho;
  const S = rg.Sref;

  // 气动合力（与 vtolCases 巡航口径一致）
  const liftDrag = (V, phi) => {
    const q = 0.5 * rho * V * V;
    const CL = clAtAlpha(rg, phi);
    const stalled = Math.abs(phi) > rg.alphaStallDeg * Math.PI / 180;
    const e = stalled ? 0.40 : rg.e;
    const CDi = CL * CL / (Math.PI * rg.AR * e);
    const CDx = 0.6 * Math.pow(Math.sin(phi), 2) * (rg.Sside / S) * (rg.Lb / rg.D) * 0.05;
    const D = (rg.cd0 + CDi + Math.abs(CDx)) * q * S * 0.94
      + bodyDrag(rho, V, frontal_m2, 'rocket').D * 0.35;
    return { L: CL * q * S, D };
  };

  // 满油门可用推力（随来流变化，受电池放电能力限制）
  const thrustFull = (Va) => fullPoint({ ...ctx, Va }, bat.maxCurrent_A, n).thrust;

  const rows = [];
  for (let i = 0; i <= pitchSteps; i++) {
    // φ = 90（竖直悬停）→ 5（近水平巡航）
    const phiDeg = 90 - (85 * i) / pitchSteps;
    const phi = phiDeg * Math.PI / 180;
    const sinP = Math.sin(phi), cosP = Math.cos(phi);

    let V, Treq, okSol = true;
    if (sinP > 0.999) {
      // 端点：垂直悬停，T = W + 无来流阻力
      V = 0; Treq = W;
    } else {
      // 求 g(V) = (W − L·cosφ)·cotφ − L·sinφ − D 的根
      const g = (Vv) => {
        const { L, D } = liftDrag(Vv, phi);
        return (W - L * cosP) * (cosP / sinP) - L * sinP - D;
      };
      let lo = 0.01, hi = 0.01, found = false;
      let gPrev = g(lo);
      for (let Vt = 0.5; Vt <= 60; Vt += 0.5) {
        const gNow = g(Vt);
        if (gPrev > 0 && gNow <= 0) { lo = Vt - 0.5; hi = Vt; found = true; break; }
        gPrev = gNow;
      }
      if (!found) { okSol = false; V = NaN; Treq = Infinity; }
      else {
        for (let k = 0; k < 24; k++) {
          const mid = (lo + hi) / 2;
          if (g(mid) > 0) lo = mid; else hi = mid;
        }
        V = (lo + hi) / 2;
        const { L } = liftDrag(V, phi);
        Treq = (W - L * cosP) / sinP;
      }
    }

    const Vref = okSol ? V : Math.max(vStall_ms * 1.25, 8);
    const Tav = thrustFull(Vref);
    const margin = Number.isFinite(Treq) && Treq > 0 ? (Tav - Treq) / Treq : -1;

    // 差动推力控制权限（X 布局，一对增推一对减推）
    const TperReq = Treq / n, TperMax = Tav / n;
    const dT = clamp(TperMax - TperReq, 0, TperMax);
    const rollMoment = dT * arm_m * Math.SQRT2 * 2;   // 两组电机差动
    const maxAlpha = rollMoment / Math.max(Ixx, 1e-6);

    // 安全判据
    //  硬性失败：无解 / 推力无正裕度 / 控制权限不足
    //  区域判断：推力承载（φ≥75°，机翼不参与）或 机翼完全有效（未失速且 V≥失速速度）
    //  中间"尾翼失速过渡带"：仍有正裕度，但机翼在失速后被依赖 → caution
    //  （真实 tailsitter 靠差动推力 + 桨流快速穿越该带，故不判为不可行）
    const alphaStalled = phi > rg.alphaStallDeg * Math.PI / 180;
    const thrustBorne = phiDeg >= 75;
    const wingAttached = !alphaStalled && okSol && V >= vStall_ms * 0.98;
    const hardOK = okSol && margin > 0 && maxAlpha > 2;
    const caution = hardOK && !thrustBorne && !wingAttached;
    const safe = hardOK && (thrustBorne || wingAttached);
    const mode = thrustBorne ? 'thrust' : (wingAttached ? 'wing' : 'transition');

    rows.push({
      phi_deg: +phiDeg.toFixed(0),
      V_ms: okSol ? +V.toFixed(1) : null,
      V_kmh: okSol ? +(V * 3.6).toFixed(0) : null,
      thrust_avail_N: +Tav.toFixed(2),
      thrust_req_N: Number.isFinite(Treq) ? +Treq.toFixed(2) : null,
      margin_pct: +(margin * 100).toFixed(0),
      controlAlpha: +maxAlpha.toFixed(1),
      alphaDeg: okSol ? +(phiDeg).toFixed(0) : null,
      stalled: alphaStalled,
      mode, hardOK, safe, caution,
    });
  }

  const hardRows = rows.filter(r => !r.hardOK);
  const safeRows = rows.filter(r => r.safe);
  const cautionRows = rows.filter(r => r.caution);
  const finiteMargins = rows.filter(r => r.thrust_req_N != null).map(r => r.margin_pct);
  const minMargin = finiteMargins.length ? Math.min(...finiteMargins) : 0;
  const minCtrl = Math.min(...rows.map(r => r.controlAlpha));
  // 推荐转换点：安全行中推力裕度最大的姿态
  const recommended = safeRows.length
    ? safeRows.reduce((a, b) => (b.margin_pct > a.margin_pct ? b : a))
    : null;

  return {
    rows,
    feasible: hardRows.length === 0,
    unsafeCount: hardRows.length,
    cautionCount: cautionRows.length,
    cautionBand_deg: cautionRows.length
      ? [Math.max(...cautionRows.map(r => r.phi_deg)), Math.min(...cautionRows.map(r => r.phi_deg))]
      : null,
    minMargin_pct: minMargin,
    minControlAlpha: minCtrl,
    recommendedPhi: recommended ? recommended.phi_deg : null,
    recommendedV_kmh: recommended ? recommended.V_kmh : Math.round(vStall_ms * 1.25 * 3.6),
    vEntry_kmh: +(Math.max(vStall_ms * 1.15, 8) * 3.6).toFixed(0),
    vExit_kmh: +(Math.max(vStall_ms * 1.5, 12) * 3.6).toFixed(0),
  };
}

/**
 * 悬停稳定性（垂起火箭竖直姿态）：
 * 重心在推力面下方 → 单摆稳定（回复频率）；上方 → 倒立摆（需飞控持续修正）。
 * 差动推力权限：总推力余量 E 分配到两侧 → τroll = E·arm/√2。
 */
export function hoverStability({ rocket, Ixx, arm_m, maxThrustN, hoverThrustN, mass_kg }) {
  const cg = (rocket.cgBelowThrust_mm ?? 20) / 1000;
  const pendulum_Hz = cg > 0 ? Math.sqrt(9.80665 / cg) / (2 * Math.PI) : null;
  const inverted = cg < 0;
  const tau_inverted = inverted ? Math.sqrt(Math.abs(cg) / 9.80665) : null;
  const requiredBW_Hz = inverted ? 3 / (2 * Math.PI * tau_inverted) : null;
  // 推力余量 E → 最大差动力矩
  const E = Math.max(0, maxThrustN - hoverThrustN);
  const rollMoment = E * arm_m * 0.707;
  const alphaRoll = rollMoment / Math.max(Ixx, 1e-6);
  const yawInertia = 0.7 * mass_kg * arm_m * arm_m;
  const alphaYaw = (E * arm_m * 0.35) / Math.max(yawInertia, 1e-6);
  return {
    cgBelowThrust_mm: rocket.cgBelowThrust_mm ?? 20,
    stable: !inverted,
    pendulum_Hz: pendulum_Hz ? +pendulum_Hz.toFixed(2) : null,
    inverted,
    requiredBW_Hz: requiredBW_Hz ? +requiredBW_Hz.toFixed(1) : null,
    rollAlpha: +alphaRoll.toFixed(1),
    yawAlpha: +alphaYaw.toFixed(1),
    minLoopHz: pendulum_Hz ? +(pendulum_Hz * 10).toFixed(0)
      : (requiredBW_Hz ? +(requiredBW_Hz * 10).toFixed(0) : null),
  };
}
