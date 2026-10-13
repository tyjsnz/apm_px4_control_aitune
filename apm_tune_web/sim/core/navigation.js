// ============================================================
// 固定翼增稳 & 导航工况
//   增稳：静稳定裕度、控制权限（舵面能否在失速前配平/拉杆）、
//         转弯时的滚转-偏航耦合、失速保护需求
//   导航：转弯性能（坡度/半径/速率）、抗风漂移、航点巡航功率、
//         定高盘旋续航、最远转场距离
// ============================================================
import { G, clamp } from './parts.js';
import { reynolds } from './air.js';

/**
 * 固定翼静/动稳定性 + 增稳控制权限
 * @param {object} p
 *   wg       wingGeom() 结果
 *   rho, mu  大气
 *   W_N      重量
 *   Ixx,Iyy,Izz 转动惯量
 *   mass_kg  质量
 *   cControl 舵面弦长占比（有效控制面）
 *   sControl 舵面面积占比
 */
export function stabilityAnalysis({ wg, rho, mu, W_N, Ixx, Iyy, Izz, mass_kg, cControl = 0.22, sControl = 0.12, Cnbeta = null }) {
  const S = wg.area, b = wg.span;
  const cbar = S / b;                                  // 平均几何弦（矩形近似）
  const AR = wg.AR;

  // 升力线斜率（有限展弦比 + 低 Re 修正）
  const a0 = 2 * Math.PI;
  const clAlpha = a0 / (1 + a0 / (Math.PI * wg.e * AR));   // per rad
  // 低 Re 削减（典型 5~15%）
  const ReRef = reynolds(25, cbar, mu / rho);
  const reFactor = ReRef < 1e5 ? 0.88 : ReRef < 3e5 ? 0.95 : 1.0;
  const clAlphaEff = clAlpha * reFactor;

  // ---- 静稳定裕度：重心在 25%MAC、气动中心约 30%MAC（常规布局经验值）----
  // x_np = (尾臂 × 尾面贡献) 迭代；这里用经验：x_np ≈ 0.35c̄（无尾飞翼）~ 0.45c̄（常规）
  // 飞翼/箭式布局靠尾翼稳定，x_np 偏后
  const xNP = 0.42 * cbar;      // 中性点（相对前缘）
  const xCG = 0.25 * cbar;      // 重心（推荐位置）
  const SM = (xNP - xCG) / cbar;

  // 短周期 & 螺旋
  const qDyn = 0.5 * rho * Math.pow(30, 2);           // 参考动压 @30m/s
  const Malpha = -qDyn * S * cbar * SM * clAlphaEff;  // 俯仰刚度 N·m/rad
  const wn_sp = Math.sqrt(Math.abs(Malpha) / Math.max(Iyy, 1e-6));
  const Mq = -clAlphaEff * qDyn * S * cbar * cbar / (2 * 30) * 1.6;   // 俯仰阻尼
  const zeta_sp = Math.abs(Mq) / (2 * wn_sp * Math.max(Iyy, 1e-6));
  const T_phugoid = Math.SQRT2 * 30 / G;

  // ---- 舵面控制权限 ----
  // 升降舵能产生的最大俯仰力矩 vs 稳定性恢复力矩
  const cTau = 0.9;                                  // 舵面效率因子
  const elevatorAuth = (clAlphaEff * cTau * sControl * cControl * S * cbar * 0.5 * 1e4) / Math.max(Malpha, 1);
  // 副翼滚转权限：p_max = 2·V·Clda·δa / (|Clp|·b)（滚转阻尼与速压抵消）
  const Clda = 0.12;                                 // 每弧度副翼效率
  const delta_a = 30 * Math.PI / 180;                // 满副翼行程
  const ClpBar = Math.max(0.5, clAlphaEff * AR / 8); // 滚转阻尼导数
  const rollRateMax = (2 * 30 * Clda * delta_a) / (ClpBar * Math.max(b, 0.1));   // rad/s（满副翼 @30m/s）
  // 方向舵/尾翼航向静稳定：Cnβ ≈ 尾容比 × 升力线斜率（经验 0.05~0.15）
  const CnbetaVal = Number.isFinite(Cnbeta) ? Cnbeta : 0.09;
  const yawStiff = Math.sqrt(Math.abs(CnbetaVal * qDyn * S * b) / Math.max(Izz, 1e-6));   // rad/s 偏航刚度
  const yawAuth = yawStiff / (2 * Math.PI);                                              // Hz 偏航固有频率

  return {
    cbar_m: +cbar.toFixed(4), AR: +AR.toFixed(2),
    clAlpha: +clAlphaEff.toFixed(2),           // per rad
    SM_pct: +(SM * 100).toFixed(1),
    xNP_pct: +(xNP / cbar * 100).toFixed(0),
    xCG_pct: 25,
    wn_sp_Hz: +(wn_sp / (2 * Math.PI)).toFixed(2),
    zeta_sp: +clamp(zeta_sp, 0, 3).toFixed(2),
    phugoid_s: +T_phugoid.toFixed(1),
    elevatorAuth,                             // >1 表示舵面能压过静稳定裕度
    rollRateMax_dps: +(rollRateMax * 180 / Math.PI).toFixed(0),
    Cnbeta: +CnbetaVal.toFixed(3),
    yawStiff_radps: +yawStiff.toFixed(2),
    yawAuth_Hz: +yawAuth.toFixed(2),
    reRef: Math.round(ReRef),
    // 增稳需求判定
    needsAugStab: SM > 0.25 || SM < 0,
  };
}

/**
 * 导航工况：转弯性能 / 抗风 / 航点巡航 / 盘旋续航
 * @param {object} p
 *   wg, rho, W_N, mass_kg, vStall_ms, maxLDSpeed_ms, minSinkSpeed_ms,
 *   polarAt(V) → {CL, CD, D}, powerAt(V) → {pIn (总), current},   （总功率 W）
 *   maxThrust_N, endurance_max_min
 */
export function navigationCases({ wg, rho, W_N, mass_kg, vStall_ms, maxLDSpeed_ms, minSinkSpeed_ms,
  polarAt, powerAt, maxThrust_N, endurance_max_min, usableEnergy_Wh = null, wind_kmh = 20 }) {
  const S = wg.area, b = wg.span;
  const wind = wind_kmh / 3.6;
  const Vcruise = maxLDSpeed_ms || Math.max(vStall_ms * 1.5, 20);

  // ---- 转弯性能：坡度 20/30/45° ----
  // 滚转速率用阻尼平衡式：p = 2·V·Clda·δa / (|Clp|·b)，Clp ≈ Clα·AR/8
  const Clda = 0.12;                       // 每弧度副翼效率
  const delta_a = 30 * Math.PI / 180;      // 满副翼 30°
  const ClpBar = Math.max(0.5, (wg.CL_alpha ?? 4.5) * wg.AR / 8);
  const turns = [20, 30, 45].map(bankDeg => {
    const phi = bankDeg * Math.PI / 180;
    const nLoad = 1 / Math.cos(phi);                          // 过载
    const vTurnMin = vStall_ms * Math.sqrt(nLoad);            // 失速速度随过载升高
    const V = Math.max(Vcruise, vTurnMin * 1.05);
    const radius = (V * V) / (G * Math.tan(phi));             // 转弯半径
    const rate = (G * Math.tan(phi)) / V * 180 / Math.PI;     // 转弯速率 deg/s
    const p = polarAt(V);
    const pow = powerAt(V);
    const clNeed = p.CL * nLoad;
    return {
      bankDeg, nLoad: +nLoad.toFixed(2),
      V_kmh: +(V * 3.6).toFixed(0), vStallTurn_kmh: +(vTurnMin * 3.6).toFixed(0),
      radius_m: +radius.toFixed(0), rate_dps: +rate.toFixed(1),
      power_W: pow.pIn, current_A: pow.current,
      clMargin: +(wg.clmax - clNeed).toFixed(2),
      feasible: clNeed < wg.clmax && pow.duty <= 1.0001,
    };
  });

  // ---- 抗风漂移：巡航速度 vs 风速 ----
  const groundHead = Vcruise - wind;         // 逆风地速
  const groundTail = Vcruise + wind;         // 顺风地速
  const headPenalty = groundHead > 0 ? (1 - groundHead / Vcruise) : 1;
  // 等时往返的平均地速（逆风+顺风谐波平均）
  const avgRoundTrip = (2 * groundHead * groundTail) / Math.max(groundHead + groundTail, 1e-6);

  // ---- 航点巡航：在 Vcruise 下的功率 / 续航 / 航程（按可用电量估算）----
  const cruisePow = powerAt(Vcruise);
  const cruisePolar = polarAt(Vcruise);
  const Euse = (usableEnergy_Wh != null && usableEnergy_Wh > 0) ? usableEnergy_Wh : null;
  const cruiseEndurance_min = Euse && cruisePow.pIn > 0 ? Euse * 60 / cruisePow.pIn : null;
  const range_km = cruiseEndurance_min != null ? Vcruise * 3.6 * cruiseEndurance_min / 60 : null;

  // ---- 盘旋（定高 360°）续航：以 minSinkSpeed 盘旋 ----
  const loiterV = minSinkSpeed_ms || Vcruise;
  const loiterPow = powerAt(loiterV);
  const loiterEndurance_min = Euse && loiterPow.pIn > 0 ? Euse * 60 / loiterPow.pIn : null;
  const loiterRadius = (loiterV * loiterV) / (G * Math.tan(20 * Math.PI / 180));   // 20° 坡度盘旋

  return {
    Vcruise_kmh: +(Vcruise * 3.6).toFixed(0),
    turns,
    wind: {
      wind_kmh,
      groundHead_kmh: +(groundHead * 3.6).toFixed(0),
      groundTail_kmh: +(groundTail * 3.6).toFixed(0),
      headPenalty_pct: +(headPenalty * 100).toFixed(0),
      avgRoundTrip_kmh: +(avgRoundTrip * 3.6).toFixed(0),
      driftAngle_deg: +(Math.asin(clamp(wind / Vcruise, -1, 1)) * 180 / Math.PI).toFixed(1),
      feasible: groundHead > 2,
    },
    cruise: {
      V_kmh: +(Vcruise * 3.6).toFixed(0),
      power_W: cruisePow.pIn, current_A: cruisePow.current,
      CL: +cruisePolar.CL.toFixed(3), CD: +cruisePolar.CD.toFixed(4),
      L_D: +cruisePolar.L_D.toFixed(1),
      endurance_min: cruiseEndurance_min != null ? +cruiseEndurance_min.toFixed(1) : null,
    },
    loiter: {
      V_kmh: +(loiterV * 3.6).toFixed(0),
      radius_m: +loiterRadius.toFixed(0),
      power_W: loiterPow.pIn, current_A: loiterPow.current,
      endurance_min: loiterEndurance_min != null ? +loiterEndurance_min.toFixed(1) : null,
    },
    range_km: range_km != null ? +range_km.toFixed(1) : null,
    usableEnergy_Wh: Euse,
    maxLDSpeed_kmh: +(Vcruise * 3.6).toFixed(0),
  };
}
