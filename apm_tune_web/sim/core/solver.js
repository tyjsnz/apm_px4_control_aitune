// ============================================================
// 目标驱动反向求解引擎
//   当 diagnose() 报告某项指标不达标时，自动搜索参数空间，
//   输出"把 KV 1750→2300、电芯 4S→6S 即可让目标速度 120km/h
//   从不可达变为可达（裕度 18%）"这类可执行的量化改造方案。
//
//   - 算法 A：目标可达性反向搜索（速度/推重比/悬停油门/续航/电池/热/噪音）
//     阶段 1 单项改动直搜（优先"最小改动"）→ 阶段 2 坐标下降累积
//   - 算法 B：解析反解（结构安全系数、静稳定裕度，闭式公式）
//   - 算法 C：灵敏度分析（各参数对目标的杠杆排序，有限差分）
//   - 算法 D：Pareto 多目标前沿（速度 vs 续航冲突时的权衡方案）
//   - 阶段 3：kv×cells×propD 联合网格（同时满足多目标的组合最优）
//
//   求解器只调 simulate() 作为唯一物理真值评估器，不重写物理；
//   评估调用带 _light 标志（跳过优化引擎/推力曲线/报告），并共享
//   结果缓存（多个目标搜索同一配置只算一次）。
// ============================================================
import { simulate } from './simulate.js';
import { MOTORS, PROP_PRESETS, clamp } from './parts.js';
import { T } from './tuning.js';

const TOP_N = 5;            // 解排序后保留条数（Pareto 另计）
const MAX_SEARCH_TARGETS = 4;
// 预算上限（maxCalls/deadline_ms）已收进 core/tuning.js 的 solver 组，可在线调整。

let callCount = 0, deadline = 0;
let baseErrorCodes = new Set();
const evalCache = new Map();

// ---------------------------- 指标提取 ----------------------------
export const METRIC_META = {
  vmax: { label: '最大速度', unit: 'km/h', fmt: 0 },
  twr: { label: '推重比', unit: '', fmt: 2 },
  hoverDuty: { label: '悬停油门', unit: '%', fmt: 0, scale: 100 },
  endurance: { label: '续航时间', unit: 'min', fmt: 1 },
  structureSafety: { label: '机臂安全系数', unit: '', fmt: 1 },
  SM_pct: { label: '静稳定裕度', unit: '%', fmt: 1 },
  fullCurrentDemand: { label: '满油门需求电流', unit: 'A', fmt: 0 },
  motorHeatRatio: { label: '电机热负荷比', unit: '', fmt: 2 },
  noise10m: { label: '10m 噪音', unit: 'dB', fmt: 0 },
  machTip: { label: '满油门桨尖马赫', unit: '', fmt: 2 },
  vertClimb: { label: '垂直爬升率', unit: 'ft/min', fmt: 0 },
};

export function extractMetrics(R) {
  const c = R.cases;
  const endurance = R.type === 'fixedwing'
    ? (c.endurance?.max ?? 0)
    : (R.battery?.enduranceHoverMin ?? c.endurance?.hover ?? 0);
  return {
    vmax: c.vmax ?? 0,
    twr: c.twr ?? 0,
    hoverDuty: c.hover?.duty ?? 0,
    hoverGPerW: c.hover?.gPerW ?? 0,
    endurance,
    structureSafety: R.structure?.armPlate?.safety ?? 0,
    armThickness_mm: R.structure?.armPlate?.thickness_mm ?? 0,
    armRequired_mm: R.structure?.armPlate?.required_mm ?? 0,
    armWidth_mm: R.structure?.armPlate?.width_mm ?? 45,
    SM_pct: R.stability?.SM_pct ?? null,
    maxLD: c.maxLD ?? 0,
    targetReachable: c.target?.reachable ?? false,
    targetKmh: c.target?.kmh ?? 0,
    targetDuty: c.target?.duty ?? 0,
    mass_g: R.totalMass_g ?? 0,
    fullCurrent: c.full?.current ?? 0,
    fullCurrentDemand: c.full?.currentDemand ?? c.full?.current ?? 0,
    batMaxCurrent_A: R.battery?.maxCurrent_A ?? Infinity,
    cRatingLimited: c.cRatingLimited ?? false,
    machTip: c.full?.machTip ?? 0,
    motorHeatRatio: (c.full?.thermal ?? 0) / Math.max(R.motor?.PlossBurst ?? 1, 1),
    noise10m: R.fluid?.noise10m ?? 0,
    vertClimb: c.vertClimb?.vs_fpm ?? 0,
    type: R.type,
    nMotors: R.nMotors,
    arm_m: R.arm_m,
  };
}

// ---------------------------- 诊断 → 目标映射 ----------------------------
// 覆盖 diagnose() 中可由"换桨/换电机/换电池"解决的 error/warn 项。
function issuesToTargets(R) {
  const m = extractMetrics(R);
  const cfg = R.cfg;
  const targets = [];
  const seen = new Set();
  const td = T().diag;   // 与 diagnose() 同一套判据，避免诊断与求解器口径不一致

  const push = (t) => {
    t.severity = t.level === 'error' ? 2 : 1;
    t.metricLabel = METRIC_META[t.metric]?.label || t.metric;
    t.metricUnit = METRIC_META[t.metric]?.unit || '';
    if (!seen.has(t.id)) { seen.add(t.id); targets.push(t); }
  };

  for (const issue of R.issues) {
    if (issue.level === 'ok' || issue.level === 'info') continue;
    switch (issue.code) {
      case 'target':
        if (cfg.target_kmh > 0 && !m.targetReachable) {
          push({
            id: 'target_speed', metric: 'vmax', current: m.vmax,
            goal: cfg.target_kmh * T().solver.speedMargin, direction: '>=',
            label: `目标速度 ${cfg.target_kmh} km/h`, issueCode: 'target',
            level: issue.level, search: true, axes: ['cells', 'kv', 'propP', 'motorType'],
          });
        }
        break;
      case 'twr':
        if (m.twr < (R.type === 'vtol' ? T().solver.twrGoalVTOL : T().solver.twrGoalMR)) {
          const goal = R.type === 'vtol' ? T().solver.twrGoalVTOL : T().solver.twrGoalMR;
          push({
            id: 'twr', metric: 'twr', current: m.twr, goal, direction: '>=',
            label: '推重比', issueCode: 'twr', level: issue.level,
            search: true, axes: ['propD', 'kv', 'cells', 'motorType'],
          });
        }
        break;
      case 'climb':
        if (R.type === 'vtol' && m.vertClimb < 250) {
          push({
            id: 'vert_climb', metric: 'vertClimb', current: m.vertClimb, goal: 250,
            direction: '>=', label: '垂直爬升率', issueCode: 'climb', level: issue.level,
            search: true, axes: ['propD', 'kv', 'cells', 'motorType'],
          });
        }
        break;
      case 'hover':
        if (m.hoverDuty > td.hoverDutyWarn) {
          push({
            id: 'hover_duty', metric: 'hoverDuty', current: m.hoverDuty, goal: 0.5,
            direction: '<=', label: '悬停油门', issueCode: 'hover', level: issue.level,
            search: true, axes: ['propD', 'propP', 'kv'],
          });
        } else if (m.hoverDuty < 0.18) {
          push({
            id: 'hover_duty_low', metric: 'hoverDuty', current: m.hoverDuty, goal: 0.20,
            direction: '>=', label: '悬停油门（动力过剩）', issueCode: 'hover', level: issue.level,
            search: true, axes: ['propD', 'propP', 'kv'],
          });
        }
        break;
      case 'time':
        if (m.endurance > 0 && m.endurance < td.enduranceMin_min) {
          push({
            id: 'endurance', metric: 'endurance', current: m.endurance,
            goal: Math.max(2, m.endurance * 1.8), direction: '>=',
            label: '续航时间', issueCode: 'time', level: issue.level,
            search: true, axes: ['capacity_mAh', 'propD', 'kv'],
          });
        }
        break;
      case 'batteryC':
        push({
          id: 'battery_c', metric: 'fullCurrentDemand', current: m.fullCurrentDemand,
          goal: m.batMaxCurrent_A, direction: '<=',
          label: '电池持续放电能力', issueCode: 'batteryC', level: issue.level,
          search: true, axes: ['capacity_mAh', 'cRating', 'kv', 'propD', 'propP'],
        });
        break;
      case 'motorHeat':
        push({
          id: 'motor_heat', metric: 'motorHeatRatio', current: m.motorHeatRatio,
          goal: issue.level === 'error' ? 0.9 : td.motorHeatWarn, direction: '<=',
          label: '电机热负荷', issueCode: 'motorHeat', level: issue.level,
          search: true, axes: ['kv', 'propD', 'propP', 'cells'],
        });
        break;
      case 'tipmach':
        push({
          id: 'tip_mach', metric: 'machTip', current: m.machTip, goal: td.tipMach,
          direction: '<=', label: '桨尖马赫', issueCode: 'tipmach', level: issue.level,
          search: true, axes: ['kv', 'propD', 'cells', 'propMaterial'],
        });
        break;
      case 'noise':
        push({
          id: 'noise', metric: 'noise10m', current: m.noise10m, goal: td.noise10m,
          direction: '<=', label: '噪音（10m）', issueCode: 'noise', level: issue.level,
          search: true, axes: ['kv', 'propD', 'cells', 'propMaterial'],
        });
        break;
      case 'arm':
        if (m.structureSafety < td.armSFError) {
          push({
            id: 'structure', metric: 'structureSafety', current: m.structureSafety,
            goal: T().solver.sfGoal, direction: '>=', label: '机臂安全系数', issueCode: 'arm',
            level: issue.level, analytic: true,
          });
        }
        break;
      case 'sm':
        if (m.SM_pct != null && (m.SM_pct < td.smMin || m.SM_pct > td.smMax)) {
          push({
            id: 'sm', metric: 'SM_pct', current: m.SM_pct, goal: `${td.smMin}~${td.smMax}%`,
            goalLo: td.smMin, goalHi: td.smMax,
            direction: 'range', label: '静稳定裕度', issueCode: 'sm',
            level: issue.level, analytic: true,
          });
        }
        break;
    }
  }
  // error 优先，其次按"当前值偏离目标的比例"排序，取前 MAX_SEARCH_TARGETS 个搜索目标
  targets.sort((a, b) => b.severity - a.severity);
  return targets;
}

// ---------------------------- 搜索轴定义 ----------------------------
const KV_SCALE = [0.6, 0.7, 0.85, 1.0, 1.2, 1.4];
const CAP_SCALE = [0.6, 0.8, 1.0, 1.3, 1.6, 2.0];
const CR_SCALE = [1, 1.5, 2, 3];

function motorFilter(type, motorId) {
  const byType = {
    multirotor: ['0804', '1103', '1204', '1404', '1507', '1804', '2205', '2207', '2306', '2507', '28065', '3110', '4114', '5215', '8120'],
    vtol: ['2205', '2207', '2306', '2507', '28065', '3110', '4114'],
    fixedwing: ['1507', '1804', '2205', '2207', '2306', '3110', '4114', '5215'],
  };
  return (byType[type] || byType.multirotor).includes(motorId);
}

// 电机耐压：从部件库标签解析推荐电芯数上限（如 "2207 (4-6S 5寸)" → 6S）
export function maxCellsOf(motorId) {
  const m = MOTORS.find(x => x.id === motorId);
  if (!m) return 24;
  const range = m.label.match(/\((\d+)-(\d+)S/);
  if (range) return +range[2];
  const single = m.label.match(/\((\d+)S/);
  return single ? +single[1] : 24;
}

function axisValues(axis, cfg, m) {
  switch (axis) {
    case 'kv':
      return KV_SCALE.map(x => Math.round(cfg.kv * x / 10) * 10)
        .filter(v => v >= 80 && v <= 30000);
    case 'cells': {
      const cap = maxCellsOf(cfg.motorType);
      return [cfg.cells - 4, cfg.cells - 2, cfg.cells, cfg.cells + 2, cfg.cells + 4]
        .filter(c => c >= 1 && c <= Math.min(24, cap));
    }
    case 'propD': {
      const maxD = m.type === 'multirotor' ? Math.round(m.arm_m * Math.SQRT2 * 0.95 * 1000) : 600;
      const all = [...new Set(PROP_PRESETS.map(p => p.d))]
        .filter(d => d >= 40 && d <= maxD).sort((a, b) => a - b);
      const idx = all.findIndex(d => d >= cfg.propD);
      const start = Math.max(0, (idx < 0 ? all.length - 3 : idx) - 2);
      return all.slice(start, start + 6);
    }
    case 'propP': {
      const all = [...new Set(PROP_PRESETS.map(p => p.p))].sort((a, b) => a - b);
      const idx = all.findIndex(p => p >= cfg.propP);
      const start = Math.max(0, (idx < 0 ? all.length - 3 : idx) - 2);
      return all.slice(start, start + 6);
    }
    case 'motorType':
      return MOTORS.filter(mt => motorFilter(m.type, mt.id) && maxCellsOf(mt.id) >= cfg.cells)
        .map(mt => mt.id);
    case 'capacity_mAh':
      return CAP_SCALE.map(x => Math.round(cfg.capacity_mAh * x / 50) * 50)
        .filter(v => v >= 100);
    case 'cRating':
      return CR_SCALE.map(x => Math.round(cfg.cRating * x))
        .filter(v => v > cfg.cRating && v <= 300);
    case 'propMaterial':
      return ['plastic', 'carbon'].filter(x => x !== cfg.propMaterial);
    case 'propB':
      return [2, 3, 4].filter(b => b !== cfg.propB);
    default:
      return [];
  }
}

// ---------------------------- 评估函数（共享缓存 + 预算） ----------------------------
function evalConfig(baseInput, over) {
  const keys = Object.keys(over).sort();
  const key = JSON.stringify(keys.map(k => [k, over[k]]));
  const hit = evalCache.get(key);
  if (hit !== undefined) return hit;
  if (callCount >= T().solver.maxCalls || Date.now() > deadline) return null;
  callCount++;
  let res = null;
  try {
    const R = simulate({ ...baseInput, ...over, _skipSolver: true, _light: 1 });
    res = { metrics: extractMetrics(R), R, over };
  } catch { res = null; }
  evalCache.set(key, res);
  return res;
}

// 硬约束剪枝：不接受明显不可行/明显劣化的配置
function feasible(m, baseMetrics, target) {
  // 基准本身不可飞（twr<0.8）时放宽到"不低于基准"，否则反例机型永远无解；
  // 未达标的方案会照常标 reached=false，且新 error 记入 sideEffects。
  const twrFloor = baseMetrics.twr < 0.8 ? Math.max(baseMetrics.twr, 0.1) : 0.8;
  if (!(m.twr >= twrFloor && m.twr <= 15)) return false;
  if (m.machTip > 0.95) return false;
  // 基准已被电池封顶时，拒绝"更吃电池"的劣化方向（目标本身是电池/电流时除外）
  const demandingMoreBattery = baseMetrics.cRatingLimited && m.cRatingLimited
    && m.fullCurrentDemand > baseMetrics.fullCurrentDemand * 1.05
    && target.metric !== 'fullCurrentDemand';
  if (demandingMoreBattery) return false;
  return true;
}

function meetsGoal(value, goal, direction, goalLo = 5, goalHi = 15) {
  if (value == null || !Number.isFinite(value)) return false;
  if (direction === '>=') return value >= goal;
  if (direction === '<=') return value <= goal;
  if (direction === 'range') return value >= goalLo && value <= goalHi;
  return false;
}

// 达标裕度（正=超出目标，负=不足）
function margin(value, goal, direction, goalLo = 5, goalHi = 15) {
  if (value == null || !Number.isFinite(value)) return -Infinity;
  if (direction === '>=') return (value - goal) / Math.max(Math.abs(goal), 1e-6);
  if (direction === '<=') return (goal - value) / Math.max(Math.abs(goal), 1e-6);
  if (direction === 'range') {
    if (value < goalLo) return (value - goalLo) / Math.max(Math.abs(goalLo), 1e-6);
    if (value > goalHi) return (goalHi - value) / Math.max(Math.abs(goalHi), 1e-6);
    return 0;
  }
  return 0;
}

// 绑定到 target 的便捷包装（自动透传 range 方向的上下限）
const goalOk = (v, t) => meetsGoal(v, t.goal, t.direction, t.goalLo, t.goalHi);
const goalMargin = (v, t) => margin(v, t.goal, t.direction, t.goalLo, t.goalHi);

// ---------------------------- 解构造 ----------------------------
function paramUnit(k) {
  return { kv: 'KV', cells: 'S', propD: 'mm', propP: 'mm', propB: '叶',
    capacity_mAh: 'mAh', cRating: 'C', motorType: '', propMaterial: '' }[k] || '';
}

function fieldLabel(k) {
  return {
    kv: 'KV', cells: '电芯', propD: '桨径', propP: '桨距', propB: '叶数',
    capacity_mAh: '容量', cRating: '电池C数', motorType: '电机', propMaterial: '桨材料',
  }[k] || k;
}

function paramLabel(k, from, to) {
  if (k === 'motorType') {
    const m0 = MOTORS.find(m => m.id === from);
    const m1 = MOTORS.find(m => m.id === to);
    return `${m0?.label || from} → ${m1?.label || to}`;
  }
  if (k === 'propMaterial') {
    const nm = { plastic: '塑料', carbon: '碳纤', wood: '木质', folded: '折叠' };
    return `桨材料 ${nm[from] || from} → ${nm[to] || to}`;
  }
  const u = paramUnit(k);
  return `${fieldLabel(k)} ${from}${u} → ${to}${u}`;
}

function fmtMetric(metric, v) {
  if (v == null || !Number.isFinite(v)) return '—';
  const meta = METRIC_META[metric] || { fmt: 2, scale: 1 };
  const s = meta.scale || 1;
  return (v * s).toFixed(meta.fmt ?? 2) + (meta.unit || '');
}

function confidenceOf(reached, marginPct, changes) {
  if (!reached) return 'low';
  if (changes.length <= 2 && marginPct >= 10) return 'high';
  return 'medium';
}

function mkSolution(target, baseInput, baseMetrics, over, verify, { reached = null, note = null } = {}) {
  const val = verify.metrics[target.metric];
  const mg = goalMargin(val, target);
  const changes = [];
  for (const [k, v] of Object.entries(over)) {
    if (v !== baseInput[k] && v != null) {
      changes.push({ param: k, from: baseInput[k], to: v, unit: paramUnit(k), label: paramLabel(k, baseInput[k], v) });
    }
  }
  if (!changes.length) return null;
  if (reached == null) reached = goalOk(val, target);
  const marginPct = +(mg * 100).toFixed(1);
  // 副作用检查：新配置是否引入了基准没有的 error 级问题
  const sideEffects = (verify.R?.issues ?? [])
    .filter(i => i.level === 'error' && !baseErrorCodes.has(i.code))
    .map(i => i.title);
  const after = {
    [target.metric]: typeof val === 'number' ? +val.toFixed(3) : val,
    twr: +verify.metrics.twr.toFixed(2),
    mass_g: Math.round(verify.metrics.mass_g),
  };
  if (target.metric !== 'vmax') after.vmax = Math.round(verify.metrics.vmax);
  if (target.metric !== 'endurance') after.endurance = +verify.metrics.endurance.toFixed(1);
  return {
    targetId: target.id,
    label: target.label,
    metric: target.metric,
    metricLabel: target.metricLabel,
    metricUnit: target.metricUnit,
    direction: target.direction,
    goal: target.goal,
    goalText: target.direction === 'range' ? target.goal
      : `${target.direction === '>=' ? '≥' : '≤'} ${fmtMetric(target.metric, target.goal)}`,
    before: { [target.metric]: +target.current.toFixed(3), mass_g: Math.round(baseMetrics.mass_g) },
    after,
    changes,
    changeLabel: changes.map(c => c.label).join('，'),
    margin: marginPct,
    reached,
    confidence: sideEffects.length ? 'low' : confidenceOf(reached, marginPct, changes),
    sideEffects,
    massDelta_g: Math.round(verify.metrics.mass_g - baseMetrics.mass_g),
    cfg: over,
    note: [note, sideEffects.length ? `⚠ 副作用：该方案会引入「${sideEffects.join('」「')}」` : null]
      .filter(Boolean).join('；') || null,
    analytic: false,
  };
}

// ---------------------------- 算法 A：非线性目标反向搜索 ----------------------------
// 阶段 1：单项改动直搜（最小改动优先）——任何"只改一个参数"即可达标的方案直接返回
function stage1MinimalChange(target, baseInput, baseMetrics) {
  const cands = [];
  for (const axis of target.axes) {
    for (const v of axisValues(axis, baseInput, baseMetrics)) {
      if (v === baseInput[axis]) continue;
      const ev = evalConfig(baseInput, { [axis]: v });
      if (!ev || !feasible(ev.metrics, baseMetrics, target)) continue;
      const val = ev.metrics[target.metric];
      if (!goalOk(val, target)) continue;
      cands.push({ axis, v, ev, margin: goalMargin(val, target) });
    }
  }
  if (!cands.length) return null;
  // 排序：质量代价最小优先，其次裕度最大
  cands.sort((a, b) =>
    Math.abs(a.ev.metrics.mass_g - baseMetrics.mass_g) - Math.abs(b.ev.metrics.mass_g - baseMetrics.mass_g)
    || b.margin - a.margin);
  const best = cands[0];
  return mkSolution(target, baseInput, baseMetrics, { [best.axis]: best.v }, best.ev);
}

// 阶段 2：坐标下降（单项不够时沿多轴累积改进，逐轴收敛即停）
function stage2Descent(target, baseInput, baseMetrics) {
  let over = {};
  let cur = target.current;
  let curMargin = goalMargin(cur, target);
  let improved = false;

  for (const axis of target.axes) {
    let axisBest = null;
    for (const v of axisValues(axis, baseInput, baseMetrics)) {
      if (v === baseInput[axis]) continue;
      const ov = { ...over, [axis]: v };
      const ev = evalConfig(baseInput, ov);
      if (!ev || !feasible(ev.metrics, baseMetrics, target)) continue;
      const val = ev.metrics[target.metric];
      const mg = goalMargin(val, target);
      if (mg > curMargin + 1e-9 && (!axisBest || mg > axisBest.margin)) {
        axisBest = { ov, val, margin: mg, ev };
      }
    }
    if (axisBest) {
      over = axisBest.ov; cur = axisBest.val; curMargin = axisBest.margin; improved = true;
      if (goalOk(cur, target)) break;
    }
  }
  if (!improved) return null;
  const verify = evalConfig(baseInput, over);
  if (!verify) return null;
  const sol = mkSolution(target, baseInput, baseMetrics, over, verify);
  if (sol && !sol.reached) sol.note = `仍差 ${fmtMetric(target.metric, Math.abs(target.goal - cur))}（已扫描 ${callCount} 个配置点，受预算限制未继续搜索）`;
  return sol;
}

// ---------------------------- 算法 B：解析反解 ----------------------------
function solveStructureAnalytic(R, target, baseMetrics) {
  const a = R.structure.armPlate;
  const tm = T().carbon;
  const carbon = { E: tm.E_GPa * 1e9, sigmaAll: tm.sigmaAll_MPa * 1e6 };
  const w = a.width_mm / 1000;
  const L = R.arm_m;
  const tMotor = (R.cases.maxThrust ?? 0) / R.nMotors;
  const M = tMotor * L;
  const dLim = Math.min(0.005, Math.max(0.0015, L * 0.04));
  const SF = target.goal;
  // 与 dynamics.structure() 同源：强度 t∝√SF、刚度 t∝∛SF，取大者
  const t_stress = Math.sqrt((6 * M * SF) / (w * carbon.sigmaAll));
  const t_stiff = Math.cbrt((4 * M * L * L * SF) / (carbon.E * w * dLim));
  const t_req_mm = Math.ceil(Math.max(t_stress, t_stiff) * 1000 * 10) / 10;   // 向上取整 0.1mm

  if (t_req_mm <= a.thickness_mm) return null;   // 已足够

  const dMass = Math.round((t_req_mm - a.thickness_mm) * a.width_mm * 1.6);  // 碳板密度≈1.6 g/cm³ 折算
  return {
    targetId: 'structure',
    label: target.label,
    metric: 'structureSafety',
    metricLabel: '机臂安全系数',
    metricUnit: '',
    direction: '>=',
    goal: SF,
    goalText: `≥ ${SF}`,
    before: { structureSafety: a.safety, thickness_mm: a.thickness_mm, mass_g: baseMetrics.mass_g },
    after: { structureSafety: SF, thickness_mm: t_req_mm, mass_g: baseMetrics.mass_g + dMass },
    changes: [{
      param: 'armPlateThickness', from: a.thickness_mm, to: t_req_mm, unit: 'mm',
      label: `机臂碳板厚 ${a.thickness_mm}mm → ${t_req_mm}mm（${a.width_mm}mm 宽）`,
    }],
    changeLabel: `机臂碳板厚 ${a.thickness_mm}mm → ${t_req_mm}mm`,
    margin: 0,
    reached: true,
    confidence: 'high',
    massDelta_g: dMass,
    cfg: null,
    note: `按悬臂根部弯矩 ${M.toFixed(2)} N·m 反解：强度需 ${Math.ceil(t_stress * 10000) / 10}mm、刚度（挠度 ≤${(dLim * 1000).toFixed(1)}mm）需 ${Math.ceil(t_stiff * 10000) / 10}mm，取大者 ${t_req_mm}mm。当前 ${a.thickness_mm}mm、SF=${a.safety}。`,
    analytic: true,
  };
}

function solveStabilityAnalytic(R, target, baseMetrics) {
  const st = R.stability;
  if (!st) return null;
  const td = T().diag;
  const SM_target = (td.smMin + td.smMax) / 2;      // 目标取允许区间中点
  const xNP = st.xNP_pct;
  const xCG_current = st.xCG_pct;
  const xCG_target = +(xNP - SM_target).toFixed(0);
  const delta = xCG_target - xCG_current;

  if (Math.abs(delta) < 1) return null;

  return {
    targetId: 'sm',
    label: target.label,
    metric: 'SM_pct',
    metricLabel: '静稳定裕度',
    metricUnit: '%',
    direction: 'range',
    goal: `${td.smMin}~${td.smMax}%`,
    goalText: `${td.smMin}~${td.smMax}%`,
    before: { SM_pct: st.SM_pct, xCG_pct: xCG_current, mass_g: baseMetrics.mass_g },
    after: { SM_pct: SM_target, xCG_pct: xCG_target, mass_g: baseMetrics.mass_g },
    changes: [{
      param: 'CG', from: xCG_current, to: xCG_target, unit: '% c̄',
      label: `重心 ${xCG_current}% → ${xCG_target}% c̄（${delta > 0 ? '后移' : '前移'} ${Math.abs(delta)}% c̄）`,
    }],
    changeLabel: `重心 ${xCG_current}% → ${xCG_target}% c̄`,
    margin: 0,
    reached: true,
    confidence: 'high',
    massDelta_g: 0,
    cfg: null,
    note: `中性点 ${xNP}% c̄，重心调至 ${xCG_target}% c̄ → SM=${SM_target}%。调整方式：${delta > 0 ? '电池/载荷后移' : '电池/载荷前移'}。`,
    analytic: true,
  };
}

// ---------------------------- 阶段 3：联合网格（kv × cells × propD） ----------------------------
function jointSearch(targets, baseInput, baseMetrics) {
  const sTargets = targets.filter(t => t.search && t.axes);
  if (sTargets.length < 2) return null;

  const baseSat = sTargets.filter(t => goalOk(t.current, t)).length;
  const score = (m) => {
    let sat = 0, sum = 0;
    for (const t of sTargets) {
      const v = m[t.metric];
      if (goalOk(v, t)) { sat++; sum += 1; }
      else sum += goalMargin(v, t);
    }
    return { sat, sum };
  };

  const kvVals = [...new Set([0.8, 1.0, 1.25].map(x => Math.round(baseInput.kv * x / 10) * 10))]
    .filter(v => v >= 80 && v <= 30000);
  const cellCap = Math.min(24, maxCellsOf(baseInput.motorType));
  const cellVals = [...new Set([-2, 0, 2].map(d => baseInput.cells + d))].filter(c => c >= 1 && c <= cellCap);
  const dAll = axisValues('propD', baseInput, baseMetrics);
  const dVals = [...new Set([dAll[0], dAll[Math.floor(dAll.length / 2)], dAll[dAll.length - 1]])]
    .filter(d => d != null);

  let best = null;
  for (const kv of kvVals) {
    for (const cells of cellVals) {
      for (const propD of dVals) {
        const over = { kv, cells, propD };
        const unchanged = Object.entries(over).every(([k, v]) => v === baseInput[k]);
        if (unchanged) continue;
        const ev = evalConfig(baseInput, over);
        if (!ev) continue;
        const ok = sTargets.every(t => feasible(ev.metrics, baseMetrics, t));
        if (!ok) continue;
        const sc = score(ev.metrics);
        const massDelta = Math.abs(ev.metrics.mass_g - baseMetrics.mass_g);
        if (!best
          || sc.sat > best.sc.sat
          || (sc.sat === best.sc.sat && sc.sum > best.sc.sum)
          || (sc.sat === best.sc.sat && sc.sum === best.sc.sum && massDelta < best.massDelta)) {
          best = { ev, sc, over, massDelta };
        }
      }
    }
  }
  if (!best || best.sc.sat <= baseSat) return null;

  const main = sTargets
    .filter(t => goalOk(best.ev.metrics[t.metric], t))
    .sort((a, b) => b.severity - a.severity)[0] || sTargets[0];
  const sol = mkSolution(main, baseInput, baseMetrics, best.over, best.ev, { reached: false });
  if (!sol) return null;
  const satList = sTargets.map(t => ({
    label: t.metricLabel,
    ok: goalOk(best.ev.metrics[t.metric], t),
  }));
  sol.targetId = 'joint';
  sol.label = `组合方案（同时优化 ${sTargets.length} 项指标）`;
  sol.joint = { satisfied: best.sc.sat, total: sTargets.length, goals: satList };
  sol.reached = best.sc.sat >= sTargets.length;
  sol.confidence = confidenceOf(sol.reached, sol.margin, sol.changes);
  sol.note = `同时满足 ${best.sc.sat}/${sTargets.length} 项目标：`
    + satList.map(s => `${s.ok ? '✓' : '✗'}${s.label}`).join('、');
  return sol;
}

// ---------------------------- 算法 C：灵敏度分析 ----------------------------
const SENS_UNIT = { kv: 100, cells: 1, propD: 10, propP: 10, capacity_mAh: 500, cRating: 50 };

function sensitivityAnalysis(target, baseInput, baseMetrics) {
  const results = [];
  for (const axis of target.axes) {
    const values = axisValues(axis, baseInput, baseMetrics);
    if (values.length < 2) continue;
    const v0 = baseInput[axis];
    const v1 = values.find(v => v > v0) ?? values.find(v => v < v0);
    if (v1 == null || v1 === v0) continue;

    const ev = evalConfig(baseInput, { [axis]: v1 });
    if (!ev) continue;
    const val0 = target.current;
    const val1 = ev.metrics[target.metric];
    if (val1 == null || !Number.isFinite(val1)) continue;

    const unit = SENS_UNIT[axis] ?? 0;
    const leverage = unit ? (val1 - val0) / (v1 - v0) * unit : (val1 - val0);
    const meta = METRIC_META[target.metric] || { fmt: 2, scale: 1, unit: '' };
    results.push({
      param: axis,
      paramLabel: fieldLabel(axis),
      leverage: +leverage.toFixed(3),
      unit: unit ? `${meta.unit || ''}/${fieldLabel(axis)}+${unit}${paramUnit(axis)}` : `${meta.unit || ''}/换型`,
      direction: val1 > val0 ? '↑' : val1 < val0 ? '↓' : '→',
      sample: `${paramLabel(axis, v0, v1)} → ${fmtMetric(target.metric, val1)}`,
    });
  }
  results.sort((a, b) => Math.abs(b.leverage) - Math.abs(a.leverage));
  return results;
}

// ---------------------------- 算法 D：Pareto 多目标前沿 ----------------------------
function paretoFront(baseInput, baseMetrics) {
  const samples = [];
  const kvVals = [...new Set([0.8, 1.0, 1.25].map(x => Math.round(baseInput.kv * x / 10) * 10))].filter(v => v >= 80);
  const cellCap = Math.min(24, maxCellsOf(baseInput.motorType));
  const cellVals = [-2, 0, 2].map(d => baseInput.cells + d).filter(c => c >= 1 && c <= cellCap);
  const dAll = axisValues('propD', baseInput, baseMetrics);
  const dVals = [...new Set([dAll[0], dAll[Math.floor(dAll.length / 2)], dAll[dAll.length - 1]])].filter(d => d != null);

  for (const kv of kvVals) {
    for (const cells of cellVals) {
      for (const propD of dVals) {
        const over = { kv, cells, propD };
        if (Object.entries(over).every(([k, v]) => v === baseInput[k])) continue;
        const ev = evalConfig(baseInput, over);
        if (!ev) continue;
        if (ev.metrics.twr < (baseMetrics.twr < 0.9 ? Math.max(baseMetrics.twr, 0.1) : 0.9)
          || ev.metrics.machTip > 0.9) continue;
        samples.push({
          vmax: ev.metrics.vmax, endurance: ev.metrics.endurance,
          twr: ev.metrics.twr, mass_g: ev.metrics.mass_g,
          over,
        });
      }
    }
  }
  if (samples.length < 3) return [];

  const dominated = (s) => samples.some(o =>
    o !== s && o.vmax >= s.vmax && o.endurance >= s.endurance
    && (o.vmax > s.vmax || o.endurance > s.endurance));
  const front = samples.filter(s => !dominated(s)).sort((a, b) => a.vmax - b.vmax);
  if (!front.length) return [];

  const pick = [];
  const push = (p, tag) => { if (p && !pick.some(x => x.p === p)) pick.push({ p, tag }); };
  push(front[0], '续航优先');
  if (front.length >= 3) push(front[Math.floor(front.length / 2)], '均衡');
  push(front[front.length - 1], '极速优先');

  return pick.map(({ p, tag }) => ({
    targetId: 'pareto',
    label: `速度-续航权衡（${tag}）`,
    metric: 'pareto',
    metricLabel: 'Pareto 前沿',
    metricUnit: '',
    direction: 'frontier',
    goal: '速度 × 续航 帕累托最优',
    goalText: '速度 × 续航 帕累托最优',
    before: { vmax: Math.round(baseMetrics.vmax), endurance: +baseMetrics.endurance.toFixed(1), mass_g: Math.round(baseMetrics.mass_g) },
    after: { vmax: Math.round(p.vmax), endurance: +p.endurance.toFixed(1), twr: +p.twr.toFixed(2), mass_g: Math.round(p.mass_g) },
    changes: Object.entries(p.over).filter(([k, v]) => v !== baseInput[k])
      .map(([k, v]) => ({ param: k, from: baseInput[k], to: v, unit: paramUnit(k), label: paramLabel(k, baseInput[k], v) })),
    changeLabel: Object.entries(p.over).filter(([k, v]) => v !== baseInput[k])
      .map(([k, v]) => paramLabel(k, baseInput[k], v)).join('，'),
    margin: +(((p.vmax - baseMetrics.vmax) / Math.max(baseMetrics.vmax, 1)) * 100).toFixed(1),
    reached: true,
    confidence: 'medium',
    massDelta_g: Math.round(p.mass_g - baseMetrics.mass_g),
    cfg: p.over,
    note: `${tag}：${Math.round(p.vmax)} km/h / ${p.endurance.toFixed(1)} min（基准 ${Math.round(baseMetrics.vmax)} km/h / ${baseMetrics.endurance.toFixed(1)} min）`,
    analytic: false,
    pareto: true,
  }));
}

// ---------------------------- 主入口 ----------------------------
export function solveForTargets(R, baseInput) {
  callCount = 0;
  deadline = Date.now() + T().solver.deadline_ms;
  evalCache.clear();

  // normalize() 读的是 rotorCount，而 cfg 里是 nMotors——透传回去，
  // 保证六/八轴配置在评估调用中不被重置为机架默认电机数。
  baseInput = { ...baseInput, rotorCount: baseInput.nMotors ?? baseInput.rotorCount };

  const baseMetrics = extractMetrics(R);
  baseErrorCodes = new Set(R.issues.filter(i => i.level === 'error').map(i => i.code));
  const allTargets = issuesToTargets(R);
  if (!allTargets.length) {
    return { solutions: [], sensitivity: {}, pareto: [], scanned: 0, baseMetrics, hasFailures: false };
  }
  const searchTargets = allTargets.filter(t => t.search).slice(0, MAX_SEARCH_TARGETS);
  const analyticTargets = allTargets.filter(t => t.analytic);

  const solutions = [];
  const sensitivityMap = {};

  // 算法 B：解析反解（毫秒级，优先）
  for (const target of analyticTargets) {
    let sol = null;
    if (target.id === 'structure') sol = solveStructureAnalytic(R, target, baseMetrics);
    else if (target.id === 'sm') sol = solveStabilityAnalytic(R, target, baseMetrics);
    if (sol) { sol.sensitivity = []; solutions.push(sol); }
  }

  // 算法 A：非线性搜索（阶段 1 单项直搜 → 阶段 2 坐标下降）
  for (const target of searchTargets) {
    let sol = stage1MinimalChange(target, baseInput, baseMetrics)
      || stage2Descent(target, baseInput, baseMetrics);
    if (sol) {
      sol.sensitivity = sensitivityAnalysis(target, baseInput, baseMetrics);
      sensitivityMap[target.metric] = sol.sensitivity;
      solutions.push(sol);
    }
  }

  // 阶段 3：联合网格（多目标同时优化）
  let joint = null;
  if (searchTargets.length >= 2 && callCount < T().solver.maxCalls - 30) {
    joint = jointSearch(searchTargets, baseInput, baseMetrics);
    if (joint) solutions.push(joint);
  }

  // 算法 D：Pareto 前沿（速度 vs 续航，仅当两者之一有目标且预算充足）
  let paretoSolutions = [];
  const hasSpeed = searchTargets.some(t => t.metric === 'vmax');
  const hasEndurance = searchTargets.some(t => t.metric === 'endurance');
  if ((hasSpeed || hasEndurance) && callCount < T().solver.maxCalls - 25 && Date.now() < deadline - 1500) {
    paretoSolutions = paretoFront(baseInput, baseMetrics);
  }

  // 解排序：①达标 ②无副作用 ③联合满足目标数 ④改动参数数 ⑤质量代价 ⑥裕度
  solutions.sort((a, b) => {
    if (a.reached !== b.reached) return a.reached ? -1 : 1;
    const ea = a.sideEffects?.length ?? 0, eb = b.sideEffects?.length ?? 0;
    if (ea !== eb) return ea - eb;
    const sa = a.joint?.satisfied ?? 0, sb = b.joint?.satisfied ?? 0;
    if (sa !== sb) return sb - sa;
    if (a.changes.length !== b.changes.length) return a.changes.length - b.changes.length;
    if (Math.abs(a.massDelta_g) !== Math.abs(b.massDelta_g)) return Math.abs(a.massDelta_g) - Math.abs(b.massDelta_g);
    return (b.margin ?? 0) - (a.margin ?? 0);
  });

  const all = [...solutions.slice(0, TOP_N), ...paretoSolutions];

  return {
    solutions: all,
    sensitivity: sensitivityMap,
    pareto: paretoSolutions,
    scanned: callCount,
    baseMetrics,
    hasFailures: true,
  };
}
