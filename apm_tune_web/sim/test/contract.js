// 校验所有 demo 配置可完整跑通且关键字段非空（供前端渲染的数据契约）
import { simulate } from '../core/simulate.js';
import { extractMetrics } from '../core/solver.js';
import { tuningMeta } from '../core/tuning.js';

const DEMOS = {
  default: { type: 'multirotor', frameId: 'fpv5', cells: 4, capacity_mAh: 1300, cRating: 100, motorType: '2207', kv: 1750, propD: 127, propP: 109, propB: 3, target_kmh: 120 },
  longrange: { type: 'multirotor', frameId: 'longrange', cells: 4, capacity_mAh: 3000, motorType: '3110', kv: 950, propD: 178, propP: 102, target_kmh: 80 },
  heavy: { type: 'multirotor', frameId: 'x8heavy', rotorCount: 8, cells: 12, capacity_mAh: 10000, motorType: '8120', kv: 150, propD: 432, propP: 152, propB: 2, escType: 'hv', target_kmh: 60, payload_g: 5000 },
  whoop: { type: 'multirotor', frameId: 'whoop65', cells: 1, capacity_mAh: 300, motorType: '0804', kv: 5000, propD: 51, propP: 76, target_kmh: 80 },
  wing: { type: 'fixedwing', wingId: 'wing1000', cells: 4, capacity_mAh: 2200, motorType: '4114', kv: 650, propD: 203, propP: 152, propB: 2, target_kmh: 100 },
  glider: { type: 'fixedwing', wingId: 'glider2m', cells: 4, capacity_mAh: 5000, motorType: '4114', kv: 650, propD: 254, propP: 152, propB: 2, target_kmh: 80 },
  rocket8: { type: 'vtol', rocketId: 'rocket8', cells: 6, capacity_mAh: 3000, motorType: '28065', kv: 1300, propD: 203, propP: 114, propB: 3, target_kmh: 100 },
  rocket10: { type: 'vtol', rocketId: 'rocket10', cells: 6, capacity_mAh: 5000, motorType: '4114', kv: 650, propD: 254, propP: 114, propB: 3, target_kmh: 90 },
};

let fail = 0;
const bad = [];
const DEMO_RESULTS = {};   // 每个 demo 的结果，供后续调参契约复用
const need = (R, path) => {
  const v = path.split('.').reduce((a, k) => (a == null ? a : a[k]), R);
  if (v === undefined || v === null || (typeof v === 'number' && !Number.isFinite(v))) bad.push(path);
  return v;
};

for (const [k, cfg] of Object.entries(DEMOS)) {
  const R = simulate(cfg);
  DEMO_RESULTS[k] = R;
  need(R, 'issues'); need(R, 'report'); need(R, 'plan');
  need(R, 'mass.total_g'); need(R, 'cases.vmax'); need(R, 'cases.sweep');
  need(R, 'fluid.reBlade'); need(R, 'energy.pBatt'); need(R, 'battery.energy_Wh');
  need(R, 'structure.armPlate.safety'); need(R, 'dynamics.actuator.passing_Hz');
  if (R.type === 'multirotor') {
    need(R, 'cases.hover.duty'); need(R, 'cases.hover.gPerW'); need(R, 'cases.twr');
    if (R.cases.hoverReachable) need(R, 'cases.endurance.hover');
    else if (R.cases.endurance?.hover != null) bad.push(k + ': 不可悬停却给出续航');
    need(R, 'dynamics.thrustCurve');
    need(R, 'dynamics.pid.response.overshoot'); need(R, 'dynamics.pid.response.settling_ms');
    need(R, 'dynamics.pid.pid.P'); need(R, 'dynamics.pid.pid.D');
    need(R, 'dynamics.pid.response.distResidual');
  } else if (R.type === 'vtol') {
    need(R, 'cases.vStall'); need(R, 'cases.maxLD'); need(R, 'cases.vertClimb.vs_fpm');
    need(R, 'cases.pitchSpeed_kmh'); need(R, 'cases.hover.duty'); need(R, 'cases.twr');
    need(R, 'cases.maxThrust');
    need(R, 'vtol.transition.feasible'); need(R, 'vtol.transition.rows');
    need(R, 'vtol.transition.minMargin_pct'); need(R, 'vtol.transition.minControlAlpha');
    need(R, 'vtol.hoverStability.rollAlpha'); need(R, 'vtol.hoverStability.yawAlpha');
    need(R, 'vtol.rocketGeom.alphaStallDeg');
    need(R, 'stability.SM_pct'); need(R, 'stability.rollRateMax_dps'); need(R, 'stability.Cnbeta');
    need(R, 'navigation.cruise.power_W'); need(R, 'navigation.turns');
    need(R, 'navigation.range_km'); need(R, 'navigation.loiter.radius_m');
    need(R, 'optimize.baseline.gPerW'); need(R, 'optimize.maxProp_mm');
    if (!R.cases.hoverReachable) bad.push(k + ': vtol hover unreachable');
    if (R.dynamics.pid == null) bad.push(k + ': vtol should have attitude pid');
    if (R.vtol.transition.rows.length < 5) bad.push(k + ': transition rows too few');
  } else {
    need(R, 'cases.vStall'); need(R, 'cases.maxLD'); need(R, 'cases.climb.vs_fpm');
    need(R, 'cases.pitchSpeed_kmh');
    if (R.dynamics.pid) bad.push(k + ': fixedwing should not have pid');
  }
  const over = R.dynamics.pid?.response.overshoot;
  if (over != null && over > 10) bad.push(`${k}: overshoot ${over}% > 10%`);
  if (R.type === 'multirotor' && R.cases.twr > 1 && (R.cases.endurance.hover == null || R.cases.endurance.hover <= 0))
    bad.push(`${k}: hover endurance invalid`);

  // ---- 目标驱动反向求解：R.solutions 结构契约 ----
  const S = R.solutions;
  if (!S || !Array.isArray(S.solutions) || !Array.isArray(S.pareto)
    || typeof S.scanned !== 'number' || typeof S.hasFailures !== 'boolean')
    bad.push(k + ': R.solutions 结构缺失');
  else {
    if (S.solutions.length > 5 + S.pareto.length)
      bad.push(k + `: 方案数 ${S.solutions.length} 超出 Top5+Pareto 上限`);
    if (S.scanned > 110) bad.push(k + `: 扫描数 ${S.scanned} 超出预算`);
    if (S.hasFailures && !S.baseMetrics) bad.push(k + ': hasFailures 但缺 baseMetrics');
    if (!S.hasFailures && S.solutions.length) bad.push(k + ': 无失败却给出方案');
    for (const s of S.solutions) {
      if (!s.label || !Array.isArray(s.changes) || !s.changes.length)
        bad.push(k + ': solution 缺 label/changes');
      if (typeof s.reached !== 'boolean' || !s.confidence || typeof s.margin !== 'number')
        bad.push(k + ': solution 缺 reached/confidence/margin');
      if (!s.pareto && s.before?.[s.metric] == null) bad.push(k + ': solution 缺 before.' + s.metric);
      if (!s.after || typeof s.massDelta_g !== 'number') bad.push(k + ': solution 缺 after/massDelta_g');
      if (s.changes.some(c => !c.param || c.from === undefined || c.to === undefined))
        bad.push(k + ': solution.change 字段不全');
      if (!s.pareto && s.cfg && typeof s.cfg !== 'object') bad.push(k + ': solution.cfg 非对象');
      if (s.sideEffects && !Array.isArray(s.sideEffects)) bad.push(k + ': sideEffects 非数组');
    }
    // 已达标且可回填的方案：回代 simulate 验证其达标声明（确定性一致性检查）
    for (const s of S.solutions.filter(x => x.reached && x.cfg && !x.pareto && !x.analytic).slice(0, 2)) {
      let vm;
      try {
        vm = extractMetrics(simulate({ ...cfg, ...s.cfg, _skipSolver: 1, _light: 1 }));
      } catch (e) { bad.push(`${k}: 方案回代异常 ${e.message}`); continue; }
      const v = vm[s.metric];
      const ok = s.direction === '>=' ? v >= s.goal
        : s.direction === '<=' ? v <= s.goal
          : (v >= (s.goalLo ?? 5) && v <= (s.goalHi ?? 15));   // range 上下限随调参表
      if (!ok) bad.push(`${k}: 方案「${s.label}」回代未达标 ${s.metric}=${v} goal=${s.goal}`);
    }
  }
}

// ---- 调参表契约：参数表结构 + R.tuningApplied/R.thresholds + 覆盖生效 + 状态隔离 ----
{
  const meta = tuningMeta();
  const keys = new Set();
  if (!Array.isArray(meta.params) || meta.params.length < 30) bad.push('tuning: params 数量不足');
  if (!Array.isArray(meta.groups) || meta.groups.length < 3) bad.push('tuning: groups 缺失');
  const gids = new Set(meta.groups.map(g => g.id));
  for (const p of meta.params) {
    if (keys.has(p.key)) bad.push('tuning: 重复 key ' + p.key);
    keys.add(p.key);
    if (!gids.has(p.group)) bad.push('tuning: 未知分组 ' + p.group + ' (' + p.key + ')');
    if (!p.label || !p.desc) bad.push('tuning: 缺 label/desc ' + p.key);
    if (!Number.isFinite(p.def) || p.def < p.min || p.def > p.max)
      bad.push(`tuning: 默认值越界 ${p.key}=${p.def} ∉ [${p.min},${p.max}]`);
    if (!Number.isFinite(p.step) || p.step <= 0) bad.push('tuning: 非法 step ' + p.key);
  }
  // 默认值必须与历史硬编码一致——这是校准基准不被破坏的守门条件
  const expect = { 'bemt.kInduced': 0.0075, 'bemt.camber_deg': 4, 'bemt.viRelax': 0.55,
    'carbon.E_GPa': 45, 'carbon.sigmaAll_MPa': 420, 'aero.cdQuad': 1.15, 'aero.bodyFactor': 0.55,
    'aero.noiseOffset_dB': -33, 'aero.groundK': 0.18, 'diag.hoverDutyWarn': 0.55,
    'diag.tipMach': 0.72, 'diag.noise10m': 75, 'solver.speedMargin': 1.15, 'solver.maxCalls': 110 };
  for (const [k, v] of Object.entries(expect)) {
    if (meta.defaults?.[k.split('.')[0]]?.[k.split('.')[1]] !== v)
      bad.push(`tuning: 默认值 ${k} 漂移 → ${meta.defaults?.[k.split('.')[0]]?.[k.split('.')[1]]}（期望 ${v}）`);
  }

  const TH = ['twrError', 'twrWarn', 'twrHighMR', 'twrHighVTOL', 'hoverDutyError', 'hoverDutyWarn',
    'hoverDutyMin', 'tipMach', 'noise10m', 'enduranceMin_min', 'armSFError', 'armSFWarn', 'ldMin', 'vtolClimbMin'];
  for (const k of Object.keys(DEMOS)) {
    if (!Array.isArray(R_of(k)?.tuningApplied)) bad.push(k + ': R.tuningApplied 缺失');
    const th = R_of(k)?.thresholds;
    if (!th) { bad.push(k + ': R.thresholds 缺失'); continue; }
    for (const f of TH) if (!Number.isFinite(th[f])) bad.push(`${k}: R.thresholds.${f} 缺失`);
    if (Math.abs(th.hoverDutyWarn - 0.55) > 1e-9 || Math.abs(th.noise10m - 75) > 1e-9)
      bad.push(`${k}: 默认阈值漂移 warn=${th.hoverDutyWarn} noise=${th.noise10m}`);
  }

  // 覆盖必须真的生效，且必须被回传报告
  const cfg = DEMOS.default;
  const R0 = simulate(cfg);
  const R1 = simulate({ ...cfg, tuning: { bemt: { kInduced: 0.03 }, carbon: { sigmaAll_MPa: 300 },
    aero: { cdQuad: 2.0 } } });
  if (R1.tuningApplied.length !== 3)
    bad.push(`tuning: tuningApplied=${R1.tuningApplied.length} 期望 3`);
  if (!(R1.cases.hover.current > R0.cases.hover.current)) bad.push('tuning: kInduced 覆盖未改变电流');
  if (!(R1.structure.armPlate.safety < R0.structure.armPlate.safety)) bad.push('tuning: σ 覆盖未改变安全系数');
  if (!(R1.cases.vmax < R0.cases.vmax)) bad.push('tuning: cdQuad 覆盖未改变最高速');
  // 判据阈值必须单独对照：只改 diag/* 时物理读数逐位不变（否则会把物理改动误当成阈值效果）
  const Rth = simulate({ ...cfg, tuning: { diag: { noise10m: 60, hoverDutyWarn: 0.30, twrWarn: 2.0 } } });
  if (Rth.tuningApplied.length !== 3) bad.push(`tuning: diag 覆盖未全被报告（${Rth.tuningApplied.length}/3）`);
  if (Rth.thresholds.noise10m !== 60 || Math.abs(Rth.thresholds.hoverDutyWarn - 0.30) > 1e-9)
    bad.push(`tuning: thresholds 未回传 diag 覆盖 (${Rth.thresholds.noise10m}, ${Rth.thresholds.hoverDutyWarn})`);
  const phys = (R) => [R.cases.hover.duty, R.cases.twr, R.cases.full.current, R.cases.vmax,
    R.fluid.noise10m, R.structure.armPlate.safety].map(x => Number(x).toFixed(6)).join('|');
  if (phys(Rth) !== phys(R0)) bad.push('tuning: 判据阈值不应改变物理读数');
  // 越界/非法值必须被夹紧或忽略，不能污染结果
  const R2 = simulate({ ...cfg, tuning: { bemt: { kInduced: 999 }, bogus: { x: 1 }, carbon: { E_GPa: 'zz' } } });
  if (!Number.isFinite(R2.cases.twr)) bad.push('tuning: 非法 tuning 导致结果非法');
  // 状态隔离：带覆盖的请求之后，不带覆盖的请求必须与首次逐位一致
  const sig = (R) => [R.cases.hover.duty, R.cases.twr, R.cases.full.current,
    R.structure.armPlate.safety, R.fluid.noise10m].map(x => Number(x).toFixed(6)).join('|');
  if (sig(simulate(cfg)) !== sig(R0)) bad.push('tuning: 调参后状态未隔离（结果被污染）');
}

function R_of(k) { return DEMO_RESULTS[k]; }
if (bad.length) { console.log('✗ 数据契约失败:'); bad.forEach(b => console.log('  - ' + b)); process.exit(1); }
console.log('✓ 全部 demo 数据契约通过');
