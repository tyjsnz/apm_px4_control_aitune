// ============================================================
// 悬停效率优化建议引擎
// 在机型/重量/电机/电池约束下，扫描「桨径 × 桨距 × KV × 电芯数 × 容量 × 叶数」，
// 用 BEMT 精确求解悬停工作点（g/W），坐标下降 + 桨-KV 网格细化，
// 输出：基准 → 单项最优建议（每项带量化收益）→ 组合最优方案。
// 每条建议均带硬约束校验：推重比 / 悬停油门 / 电池 C 数 / 桨尖马赫 / 电机热 / 桨位。
// ============================================================
import { G, clamp, clamp01, battery, motor, esc, prop } from './parts.js';
import { dutyForThrust, pointFromOmega, fullPoint } from './bemt.js';

// 候选空间（贴近真实市场规格，不给无效组合）
const D_GRID = [51, 76, 89, 102, 127, 152, 178, 203, 229, 254, 305, 381, 432];
const P_GRID = [51, 64, 76, 89, 102, 114, 119, 127, 140, 152, 178, 203];
const KV_SCALE = [0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.25];
const CAP_SCALE = [0.6, 0.75, 0.9, 1.0];
const BLADES = [2, 3, 4];
const CELLS_OPT = [0, 2];            // 相对基准：不变 / +2S

/**
 * @param {object} p
 *   base      normalize() 结果（含 type/frameId/rocketId 等）
 *   air       airModel() 结果
 *   mass_kg   基准起飞重量（含电池）
 *   n         电机数
 *   maxProp_mm 桨位允许的最大桨径（机架几何约束，可为 null 表示不限）
 * @returns {{baseline, suggestions, combined, scanned}}
 */
export function optimizeHover({ base, air, mass_kg, n = 4, maxProp_mm = null, topN = 6 }) {
  const esc0 = esc({ type: base.escType, wireAwg: base.wireAwg, wireLen_cm: base.wireLen_cm });
  const baseBat = battery({ cells: base.cells, capacity_mAh: base.capacity_mAh, chemistry: base.chemistry, cRating: base.cRating });
  const baseMot = motor({ type: base.motorType, kv: base.kv });
  const basePr = prop({ d_mm: base.propD, p_mm: base.propP, blades: base.propB, material: base.propMaterial });

  const Dmin = Math.max(40, Math.round(base.propD / 1.35 / 6) * 6);
  const Dmax = Math.min(maxProp_mm ?? 9999, Math.round(base.propD * 1.5 / 6) * 6);

  let scanned = 0;

  // 单次评估：over 可覆盖 cells/capacity/kv/propD/propP/propB
  // withEndurance=false 时跳过放电积分（网格扫描只按 g/W 排序，无需续航），
  // 胜出配置再补一次带续航的评估，总体省掉上百次 BEMT 积分。
  const evaluate = (over = {}, withEndurance = true) => {
    if (++scanned > 4000) return null;
    const cfg = { ...base, ...over };
    const cells = clamp(Math.round(cfg.cells), 1, 24);
    const dmm = cfg.propD, pmm = cfg.propP, pb = Math.round(cfg.propB);
    const ratio = pmm / dmm;
    if (ratio < 0.40 || ratio > 1.15) return null;
    if (dmm < Dmin || dmm > Dmax) return null;

    const bat = battery({ cells, capacity_mAh: cfg.capacity_mAh, chemistry: cfg.chemistry, cRating: cfg.cRating });
    const mot = motor({ type: cfg.motorType, kv: cfg.kv });
    const pr = prop({ d_mm: dmm, p_mm: pmm, blades: pb, material: cfg.propMaterial });

    // 质量随电池/电机/桨变化
    const m_kg = mass_kg
      - baseBat.mass_g / 1000 + bat.mass_g / 1000
      - n * (baseMot.mass_g - mot.mass_g) / 1000
      - n * (basePr.mass_g - pr.mass_g) / 1000;
    const W = m_kg * G;

    const ctx = { battV: bat.vCellAt(0.9), rBatt: bat.rPack_mOhm / 1000, motor: mot, esc: esc0, prop: pr, air };
    const sol = dutyForThrust({ ...ctx, thrust: W / n, Va: 0 });
    if (!sol.reachable) return null;
    const op = sol.op;
    const full = fullPoint({ ...ctx, Va: 0 }, bat.maxCurrent_A, n);
    const maxThrust = full.thrust * n;
    const twr = maxThrust / W;
    const current = op.current * n;

    // ---- 硬约束 ----
    if (sol.duty > 0.66 || sol.duty < 0.12) return null;
    if (twr < 1.5) return null;
    if (current > bat.maxCurrent_A) return null;
    if (op.throttled) return null;
    if (op.aero.machTip > 0.72) return null;          // 悬停桨尖马赫（效率与噪声）
    if (full.aero.machTip > 0.92) return null;        // 满油门桨尖马赫（激波/效率骤降）
    if (full.thermalLoad > mot.PlossBurst * 0.9) return null;

    // ---- 续航（自适应步长代数积分）----
    // 恒转速点下电流随 SOC 变化平缓，用固定 1s 步长纯属浪费 BEMT 调用：
    // 先按首步电流估计总时长选步长（全程 ≈40 步），截止电压处线性插值定位，
    // 精度与 1s 步长几乎一致（误差 ≤ 半步长），耗时降一个数量级。
    let endurance_min = null;
    if (withEndurance) {
      const Ah = bat.capacity_mAh / 1000;
      const rBatt = bat.rPack_mOhm / 1000;
      const vCutNow = bat.vCut + 0.02;
      let soc = 1, t = 0, dt = 1;
      {
        const o0 = pointFromOmega({ omega: op.omega, battV: bat.vCellAt(1), rBatt, motor: mot, esc: esc0, prop: pr, air, Va: 0 });
        const est_s = (Ah * 3600 * 0.95) / Math.max(o0.current * n, 1e-3);
        dt = clamp(Math.round(est_s / 40), 1, 25);
      }
      while (t < 5400) {
        const o = pointFromOmega({ omega: op.omega, battV: bat.vCellAt(soc), rBatt, motor: mot, esc: esc0, prop: pr, air, Va: 0 });
        const Ip = o.current * n;
        const vNow = bat.vCellAt(soc) - Ip * rBatt;
        if (vNow <= vCutNow) break;
        if (o.duty > 0.999 && t > 1) break;
        const soc2 = clamp01(soc - (Ip * dt) / (Ah * 3600));
        const vNext = bat.vCellAt(soc2) - Ip * rBatt;
        if (vNext <= vCutNow) {   // 步内线性插值定位截止时刻
          t += dt * (vNow - vCutNow) / Math.max(vNow - vNext, 1e-9);
          break;
        }
        soc = soc2; t += dt;
        if (soc <= 0.03) break;
      }
      endurance_min = +(t / 60).toFixed(1);
    }

    return {
      cfg: { cells, capacity_mAh: cfg.capacity_mAh, kv: cfg.kv, propD: dmm, propP: pmm, propB: pb },
      mass_kg: +m_kg.toFixed(3),
      gPerW: +op.gPerW.toFixed(3),
      duty: +sol.duty.toFixed(3),
      current: +current.toFixed(1),
      rpm: Math.round(op.rpm),
      power_W: Math.round(op.pIn * n),
      twr: +twr.toFixed(2),
      endurance_min,
      machTip: +full.aero.machTip.toFixed(2),
      massDelta_g: +((m_kg - mass_kg) * 1000).toFixed(0),
    };
  };

  const baseline = evaluate({});
  if (!baseline) {
    return { baseline: null, suggestions: [], combined: null, scanned, note: '当前配置不满足悬停硬约束（推重比/油门/电池/桨尖），无法作为优化基准。' };
  }
  const bg = baseline.gPerW;

  const dLabel = (a, b, unit, name) => (a !== b ? `${name} ${a}→${b}${unit}` : null);

  // ---------- 1) 坐标下降：逐轴找单项最优 ----------
  const axes = [
    { name: 'propD', values: D_GRID.filter(d => d >= Dmin && d <= Dmax) },
    { name: 'propP', values: P_GRID },
    { name: 'kv', values: KV_SCALE.map(s => Math.round(base.kv * s / 10) * 10).filter(v => v >= 100 && v <= 30000) },
    { name: 'capacity_mAh', values: CAP_SCALE.map(s => Math.round(base.capacity_mAh * s / 50) * 50).filter(v => v >= 300) },
    { name: 'cells', values: CELLS_OPT.map(d => base.cells + d) },
    { name: 'propB', values: BLADES },
  ];

  const singles = [];
  for (const axis of axes) {
    let best = null;
    for (const v of axis.values) {
      const r = evaluate({ [axis.name]: v });
      if (r && (!best || r.gPerW > best.gPerW)) best = r;
    }
    if (best) singles.push({ axis: axis.name, ...best, d_gPerW: +(best.gPerW - bg).toFixed(3), d_pct: +(((best.gPerW - bg) / bg) * 100).toFixed(1) });
  }

  // ---------- 2) 桨径 × KV 网格细化（强耦合，值得联合扫）----------
  const combos = [];
  for (const d of D_GRID.filter(x => x >= Dmin && x <= Dmax)) {
    for (const ks of [0.7, 0.8, 0.9, 1.0, 1.1]) {
      const kv = Math.round(base.kv * ks / 10) * 10;
      for (const p of P_GRID) {
        const r = evaluate({ propD: d, propP: p, kv }, false);
        if (r) combos.push(r);
      }
    }
  }

  // ---------- 3) 组合最优：从 D×KV 最优点出发再收敛各轴 ----------
  combos.sort((a, b) => b.gPerW - a.gPerW);
  let combined = combos[0] || null;
  if (combined) {
    let cur = combined, improved = true, guard = 0;
    while (improved && guard++ < 3) {
      improved = false;
      for (const axis of axes) {
        let best = cur;
        for (const v of axis.values) {
          const r = evaluate({ ...cur.cfg, [axis.name]: v }, false);
          if (r && r.gPerW > best.gPerW) best = r;
        }
        if (best.gPerW > cur.gPerW) { cur = best; improved = true; }
      }
      combined = cur;
    }
    combined = evaluate(combined.cfg, true) || combined;   // 网格免积分，最后补算续航
  }

  // ---------- 4) 生成建议（相对基准） ----------
  const mkSug = (r) => {
    const changes = [
      dLabel(baseline.cfg.propD, r.cfg.propD, 'mm', '桨径'),
      dLabel(baseline.cfg.propP, r.cfg.propP, 'mm', '桨距'),
      dLabel(baseline.cfg.propB, r.cfg.propB, ' 叶', '叶数'),
      dLabel(baseline.cfg.kv, r.cfg.kv, 'KV', 'KV'),
      r.cfg.cells !== baseline.cfg.cells ? `${baseline.cfg.cells}S→${r.cfg.cells}S` : null,
      dLabel(baseline.cfg.capacity_mAh, r.cfg.capacity_mAh, 'mAh', '容量'),
    ].filter(Boolean);
    const dEnd = (r.endurance_min != null && baseline.endurance_min != null)
      ? +(r.endurance_min - baseline.endurance_min).toFixed(1) : null;
    const dTwr = +(r.twr - baseline.twr).toFixed(2);
    return {
      ...r,
      d_gPerW: +(r.gPerW - bg).toFixed(3),
      d_pct: +(((r.gPerW - bg) / bg) * 100).toFixed(1),
      d_endurance_min: dEnd,
      d_twr: dTwr,
      changes,
      changeLabel: changes.join('，') || '保持当前配置',
      gains: [
        `悬停效率 ${bg.toFixed(2)}→${r.gPerW.toFixed(2)} g/W（${(((r.gPerW - bg) / bg) * 100).toFixed(1)}%）`,
        dEnd != null && dEnd > 0.2 ? `续航 +${dEnd}min` : (dEnd != null && dEnd < -0.2 ? `续航 ${dEnd}min` : null),
        r.duty > baseline.duty ? `悬停油门 +${((r.duty - baseline.duty) * 100).toFixed(0)}pp` : (r.duty < baseline.duty ? `悬停油门 −${((baseline.duty - r.duty) * 100).toFixed(0)}pp` : null),
        r.massDelta_g !== 0 ? `重量 ${r.massDelta_g > 0 ? '+' : ''}${r.massDelta_g}g` : null,
      ].filter(Boolean),
      costs: [
        dTwr < -0.1 ? `推重比 −${Math.abs(dTwr)}` : null,
        r.machTip > 0.68 ? `桨尖马赫 ${r.machTip} 偏高` : null,
        r.current > baseline.current * 1.05 ? `电流 +${(r.current - baseline.current).toFixed(0)}A` : null,
      ].filter(Boolean),
    };
  };

  const suggestions = [];
  const pushSug = (r) => {
    if (!r || r.gPerW <= bg + 0.005) return;
    if (suggestions.some(s => s.cfg.propD === r.cfg.propD && s.cfg.propP === r.cfg.propP && s.cfg.kv === r.cfg.kv && s.cfg.cells === r.cfg.cells && s.cfg.propB === r.cfg.propB && s.cfg.capacity_mAh === r.cfg.capacity_mAh)) return;
    suggestions.push(mkSug(r));
  };

  // 单项改进按收益排序
  singles.filter(s => s.d_gPerW > 0.005).sort((a, b) => b.gPerW - a.gPerW).slice(0, topN).forEach(pushSug);
  if (combined) pushSug(combined);

  suggestions.sort((a, b) => b.gPerW - a.gPerW);

  return {
    baseline,
    suggestions: suggestions.slice(0, topN + 1),
    combined: combined ? mkSug(combined) : null,
    scanned,
    maxProp_mm: maxProp_mm ?? null,
    note: suggestions.length ? null : '当前配置已在该约束空间的高效区（或受机架桨位限制无法再改）。',
  };
}
