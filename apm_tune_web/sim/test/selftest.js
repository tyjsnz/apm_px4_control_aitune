import { airModel } from '../core/air.js';
import { battery, motor, esc, prop } from '../core/parts.js';
import { bemt, operatingPoint, dutyForThrust, pointFromOmega } from '../core/bemt.js';
import { simulate } from '../core/simulate.js';
import { extractMetrics } from '../core/solver.js';
import { tuningMeta, T, resetTuning, tuningDiff } from '../core/tuning.js';

const air = airModel({ altitude: 0, tempC: 25 });
console.log('rho =', air.rho.toFixed(4), 'kg/m3');
const fail = (m) => { console.error('✗ 自检失败:', m); process.exit(1); };

function setup({ cells = 4, cap = 1300, motorId = '2207', kv = 1750, d = 127, p = 109, b = 3, escId = 'shott32' }) {
  const bat = battery({ cells, capacity_mAh: cap, chemistry: 'lipo' });
  const mot = motor({ type: motorId, kv });
  const e = esc({ type: escId });
  const pr = prop({ d_mm: d, p_mm: p, blades: b, material: 'plastic' });
  return { bat, mot, e, pr, ctx: { battV: bat.ocv, rBatt: bat.rPack_mOhm / 1000, motor: mot, esc: e, prop: pr, air } };
}

// ---- 场景1：经典 5寸 4S 1750KV 5043 ----
{
  const s = setup({});
  console.log('\n=== 5" 4S 1750KV 5043 ===');
  console.log('电机 R=', (s.mot.R_m * 1000).toFixed(1), 'mΩ  Icont=', s.mot.iCont.toFixed(1), 'A  Iburst=', s.mot.iBurst.toFixed(1), 'A');
  const full = operatingPoint({ ...s.ctx, duty: 1 });
  console.log(`满油门: ${(full.thrust / 9.80665 * 1000).toFixed(0)}g  ${full.current.toFixed(1)}A  ${full.rpm.toFixed(0)}rpm  ${(full.pIn).toFixed(0)}W  占空比=${full.duty.toFixed(3)}  节流=${full.throttled}`);
  const hov = dutyForThrust({ ...s.ctx, thrust: 1.6 });
  console.log(`160g  : ${(hov.op.thrust / 9.80665 * 1000).toFixed(0)}g  ${hov.op.current.toFixed(1)}A  ${hov.op.rpm.toFixed(0)}rpm  ${(hov.op.pIn).toFixed(1)}W  duty=${hov.duty.toFixed(3)}  ${hov.op.gPerW.toFixed(2)} g/W`);
  const b = bemt(s.pr, air, hov.op.omega, 0);
  console.log(`BEMT  : vi=${b.vi.toFixed(2)}m/s  FM=${b.FM.toFixed(2)}  P_aero=${b.P_aero.toFixed(1)}W  vTip=${b.vTip.toFixed(1)}m/s`);
}

// ---- 场景2：7寸 6S 长航时 ----
{
  const s = setup({ cells: 6, cap: 3000, motorId: '3110', kv: 950, d: 178, p: 102, b: 3 });
  console.log('\n=== 7" 6S 950KV 7040 ===');
  const full = operatingPoint({ ...s.ctx, duty: 1 });
  console.log(`满油门: ${(full.thrust / 9.80665 * 1000).toFixed(0)}g  ${full.current.toFixed(1)}A  ${full.rpm.toFixed(0)}rpm  ${full.pIn.toFixed(0)}W  duty=${full.duty.toFixed(3)}`);
}

// ---- 场景3：12S 大载重八轴 ----
{
  const s = setup({ cells: 12, cap: 10000, motorId: '8120', kv: 150, d: 432, p: 152, b: 2, escId: 'hv' });
  console.log('\n=== 17" 12S 150KV ===');
  const full = operatingPoint({ ...s.ctx, duty: 1 });
  console.log(`满油门: ${(full.thrust / 9.80665).toFixed(1)}kg  ${full.current.toFixed(1)}A  ${full.rpm.toFixed(0)}rpm  ${(full.pIn / 1000).toFixed(2)}kW  duty=${full.duty.toFixed(3)}`);
}

// ---- 前进飞行的推力衰减 ----
{
  const s = setup({});
  console.log('\n=== 前进来流对 5" 桨的影响 (满油门) ===');
  for (const Va of [0, 5, 10, 15]) {
    const op = pointFromOmega({ ...s.ctx, omega: s.mot.KVsi * s.bat.ocv, Va, protect: false });
    console.log(`Va=${Va}m/s  T=${(op.thrust / 9.80665 * 1000).toFixed(0)}g  η=${op.aero.eta.toFixed(2)}  J=${op.aero.advanceRatio.toFixed(2)}`);
  }
}

// ---- 性能：单点耗时 ----
{
  const s = setup({});
  const t0 = performance.now();
  for (let i = 0; i < 200; i++) operatingPoint({ ...s.ctx, duty: 0.5 + (i % 50) / 100 });
  console.log('\n耗时: 200 个工作点 =', (performance.now() - t0).toFixed(1), 'ms');
}

// ---- 目标驱动反向求解引擎：5寸目标 200km/h（基准不可达）----
{
  const cfg = {
    type: 'multirotor', frameId: 'fpv5', cells: 4, capacity_mAh: 1300, cRating: 100,
    motorType: '2207', kv: 1750, propD: 127, propP: 109, propB: 3, target_kmh: 200,
  };
  console.log('\n=== 目标驱动反向求解：5寸 目标 200km/h（基准不可达）===');
  const t0 = performance.now();
  const R = simulate(cfg);
  const ms = performance.now() - t0;
  const S = R.solutions;

  if (R.cases.target?.reachable) fail('基准 5寸 4S 不应直接可达 200km/h');
  if (!S?.solutions?.length) fail('应生成改造方案');
  if (!(S.scanned > 0 && S.scanned <= 110)) fail(`扫描数 ${S.scanned} 超出预算 [1,110]`);
  const s0 = S.solutions[0];
  if (!s0.label || !s0.changes?.length) fail('方案缺少 label/changes');
  if (s0.before?.[s0.metric] == null || s0.after == null) fail('方案缺少 before/after');
  if (typeof s0.reached !== 'boolean' || !s0.confidence) fail('方案缺少 reached/confidence');
  const solsOk = S.solutions.filter(s => s.pareto || s.analytic || (s.sensitivity !== undefined));
  if (!solsOk.length) fail('方案结构不完整');

  // 回代验证：声称达标的方案（非 Pareto、有 cfg）必须真能达标
  const chk = (v, s) => s.direction === '>=' ? v >= s.goal
    : s.direction === '<=' ? v <= s.goal
      : (v >= (s.goalLo ?? 5) && v <= (s.goalHi ?? 15));   // range 上下限随调参表
  for (const s of S.solutions.filter(x => x.reached && x.cfg && !x.pareto && !x.analytic).slice(0, 3)) {
    const vm = extractMetrics(simulate({ ...cfg, ...s.cfg, _skipSolver: 1, _light: 1 }));
    if (!chk(vm[s.metric], s)) {
      fail(`方案「${s.label}」声称达标但回代 ${s.metric}=${vm[s.metric]} 未达 goal ${s.goal}`);
    }
  }
  // 改进方向：速度型方案不应劣于基准
  const sp = S.solutions.find(s => s.metric === 'vmax');
  if (sp && sp.after.vmax < sp.before.vmax - 0.5) fail(`速度方案劣化: ${sp.before.vmax} → ${sp.after.vmax}`);
  // 灵敏度应按杠杆降序
  const sens = S.sensitivity?.vmax;
  if (sens) {
    for (let i = 1; i < sens.length; i++) {
      if (Math.abs(sens[i - 1].leverage) < Math.abs(sens[i].leverage)) fail('灵敏度未按杠杆降序');
    }
  }
  console.log(`求解耗时 ${ms.toFixed(0)}ms | 扫描 ${S.scanned} 点 | 方案 ${S.solutions.length} 个（Pareto ${S.pareto.length}）`);
  for (const s of S.solutions.slice(0, 3)) {
    console.log(`  ${s.reached ? '✓' : '⚠'} ${s.label}: ${s.changeLabel}（裕度 ${s.margin}%，${s.confidence}置信）`);
  }
  if (ms > 15000) console.log(`  ⚠ 求解耗时 ${ms.toFixed(0)}ms 偏长（预算上限约束仍在）`);
}

// ---- 调参表：半经验系数 + 判据阈值的可调性 ----
{
  console.log('\n=== 调参表（半经验系数 / 判据阈值）===');
  const meta = tuningMeta();
  const cfg = { type: 'multirotor', frameId: 'fpv5', cells: 4, capacity_mAh: 1300, cRating: 100,
    motorType: '2207', kv: 1750, propD: 127, propP: 109, propB: 3, target_kmh: 120 };

  // ① 元数据完整性
  if (meta.params.length < 30) fail(`参数数 ${meta.params.length} 过少`);
  const uniq = new Set(meta.params.map(p => p.key));
  if (uniq.size !== meta.params.length) fail('参数 key 重复');
  for (const p of meta.params) {
    if (!(p.def >= p.min && p.def <= p.max)) fail(`默认值越界 ${p.key}=${p.def}`);
    if (!p.label || !p.desc || !(p.step > 0)) fail(`参数元数据不全 ${p.key}`);
  }
  console.log(`  参数 ${meta.params.length} 个 / 分组 ${meta.groups.length}: ${meta.groups.map(g => g.id).join(' ')}`);

  // ② 默认路径零改动：tuningApplied 为空，且校准基准不变（852g 满油门推力是上文已验的锚点）
  const R0 = simulate(cfg);
  if (R0.tuningApplied.length !== 0) fail(`默认运行不应有调参覆盖，实得 ${R0.tuningApplied.length}`);
  if (Math.abs(R0.thresholds.hoverDutyWarn - 0.55) > 1e-12 || R0.thresholds.noise10m !== 75)
    fail('默认阈值漂移');
  const sig = (R) => [R.cases.hover.duty, R.cases.twr, R.cases.full.current, R.cases.vmax,
    R.structure.armPlate.safety, R.fluid.noise10m].map(x => Number(x).toFixed(6)).join('|');
  const s0 = sig(R0);

  // ③ 半经验系数必须真的改变物理
  const rInd = simulate({ ...cfg, tuning: { bemt: { kInduced: 0.02 } } });
  if (!(rInd.cases.hover.current > R0.cases.hover.current)) fail('kInduced 提高未增加悬停电流');
  if (!(rInd.cases.hover.gPerW < R0.cases.hover.gPerW)) fail('kInduced 提高未降低悬停效率');
  const rSoft = simulate({ ...cfg, tuning: { carbon: { sigmaAll_MPa: 200 } } });
  if (!(rSoft.structure.armPlate.safety < R0.structure.armPlate.safety)) fail('σ 降低未降低安全系数');
  if (Math.abs(rSoft.structure.armPlate.safety / R0.structure.armPlate.safety - 200 / 420) > 0.01)
    fail('SF 与 σ 未保持正比');
  const rStiff = simulate({ ...cfg, tuning: { carbon: { E_GPa: 90 } } });
  if (!(rStiff.structure.armPlate.required_mm < R0.structure.armPlate.required_mm)) fail('E 提高未降低所需板厚');
  const rDrag = simulate({ ...cfg, tuning: { aero: { cdQuad: 2.0 } } });
  if (!(rDrag.cases.vmax < R0.cases.vmax)) fail('cdQuad 提高未降低最高速');
  const rQuiet = simulate({ ...cfg, tuning: { aero: { noiseOffset_dB: -45 } } });
  if (Math.abs((rQuiet.fluid.noise10m - R0.fluid.noise10m) - (-12)) > 0.01) fail('噪音偏置未线性平移读数');

  // ④ 判据阈值只改判定、不改物理
  const rTh = simulate({ ...cfg, tuning: { diag: { noise10m: 100, hoverDutyWarn: 0.30, twrWarn: 2.0 } } });
  if (sig(rTh) !== s0) fail('判据阈值改变了物理读数');
  if (rTh.issues.some(i => i.code === 'noise')) fail('噪音阈值抬到 100 后超标不成立，issue 应消失');
  if (!R0.issues.some(i => i.code === 'noise')) fail('基线 77.8dB>75 应有噪音 issue');
  if (rTh.thresholds.noise10m !== 100) fail(`thresholds 未回传覆盖 ${rTh.thresholds.noise10m}`);
  if (rTh.tuningApplied.length !== 3) fail(`tuningApplied 计数 ${rTh.tuningApplied.length} ≠ 3`);

  // ⑤ 归一化：越界/非法值必须被夹紧或忽略
  simulate({ ...cfg, tuning: { bemt: { kInduced: 999, viRelax: -5 }, carbon: { E_GPa: 'zz' }, bogus: { q: 1 } } });
  const t = T();
  if (t.bemt.kInduced !== 0.05) fail(`kInduced=999 未夹到上限（${t.bemt.kInduced}）`);
  if (t.bemt.viRelax !== 0.05) fail(`viRelax=-5 未夹到下限（${t.bemt.viRelax}）`);
  if (t.carbon.E_GPa !== 45) fail(`E_GPa 非法未回退默认（${t.carbon.E_GPa}）`);

  // ⑥ 求解器达标线随 solver.speedMargin 走（用不可达的 200km/h 保证必然产出速度型方案）
  const cfgSpeed = { ...cfg, target_kmh: 200 };
  const rSol = simulate({ ...cfgSpeed, tuning: { solver: { speedMargin: 1.4 } } });
  const sv = rSol.solutions?.solutions?.find(x => x.metric === 'vmax' && !x.pareto);
  if (!sv) fail('200km/h case 未产出速度型方案，无法校验达标线联动');
  else if (Math.abs(sv.goal - 200 * 1.4) > 1e-9) fail(`速度达标线未随 speedMargin: ${sv.goal}`);
  else console.log(`  目标速度达标线随 speedMargin: ${sv.goal} = 200 × 1.4`);

  // ⑦ 状态隔离：带覆盖跑完之后，纯默认必须逐位回到初始
  if (sig(simulate(cfg)) !== s0) fail('调参后状态未隔离，纯默认结果被污染');
  resetTuning();
  if (tuningDiff().length !== 0) fail('resetTuning 后 tuningDiff 未归零');
  if (sig(simulate(cfg)) !== s0) fail('reset 后结果未回到初始');
  console.log('  ✓ 半经验系数改变物理 / 判据只改判定 / 越界夹紧 / 求解达标线联动 / 状态隔离');
}
