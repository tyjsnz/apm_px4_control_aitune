// 网页版数值内核与 Python 桌面版对拍 (12 场景, golden 来源: guide/selftest.py --json)
//   node test/guide_core.test.js
import { readFileSync } from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { SimConfig, Simulation } from '../guide/web/core/simulation.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const golden = JSON.parse(
  readFileSync(path.join(__dirname, 'golden', 'guide_selftest.json'), 'utf8'));

// 与 guide/selftest.py CASES 一一对应 (顺序/参数必须一致)
const CASES = [
  ['默认: 交叉目标(直线)', {}],
  ['目标左转 8°/s', { t_mode: 'left', t_turn: 8.0 }],
  ['目标右转 8°/s', { t_mode: 'right', t_turn: 8.0 }],
  ['目标正弦机动', { t_mode: 'sine', t_turn: 10.0, t_period: 3.0 }],
  ['大离轴: 目标航向 260°', { t_heading: 260.0 }],
  ['追击: 目标同向逃逸', { t_heading: 40.0, tx: 6000.0, ty: 1500.0 }],
  ['头对头迎击', { t_heading: 220.0, tx: 9000.0, ty: 1500.0 }],
  ['高速目标 400 m/s', { t_speed: 400.0 }],
  ['小场 6 km', { field_size: 6000.0, tx: 5000.0, ty: 4000.0 }],
  ['分段制导: PIP中段', { phased: true }],
  ['分段制导: 程序直飞+窄视场', { phased: true, mid_mode: 'straight', acq_fov: 20.0 }],
  ['分段制导: 截获距离 1.5 km', { phased: true, acq_range: 1500.0 }],
];

const REL = 1e-6;          // 相对容差 (数值内核逐行同序, 差异仅来自 libm ulp)
const REL_ACQ = 1e-6;      // 截获时刻/距离同样相对容差
const DT = 0.02;
const T_EPS = DT + 1e-9;   // 结束时刻允许跨一个步长 (阈值判定量化)

let failed = 0;
let checked = 0;
const worst = { miss: 0, peak: 0, t: 0, acq: 0 };

const relErr = (a, b) => {
  const denom = Math.max(Math.abs(a), Math.abs(b));
  if (denom < 1e-12) return Math.abs(a - b);
  return Math.abs(a - b) / denom;
};

function fail(msg) {
  failed++;
  console.error('  [不一致] ' + msg);
}

if (golden.length !== CASES.length * 2) {
  fail(`golden 记录数 ${golden.length} != ${CASES.length * 2}`);
}

for (const [title, kw] of CASES) {
  const cfg = new SimConfig(kw);
  const sim = new Simulation(cfg);
  sim.run(cfg.t_max);

  for (const law of ['pn', 'pp']) {
    const m = sim.missiles[law];
    const peak = m.hist_a.reduce((mx, v) => Math.max(mx, Math.abs(v)), 0.0);
    const g = golden.find((r) => r.case === title && r.law === law);
    const tag = `${title} / ${law}`;
    if (!g) { fail(`golden 缺记录: ${tag}`); continue; }
    checked++;

    // 1) 脱靶量
    const eMiss = relErr(m.min_range, g.miss);
    worst.miss = Math.max(worst.miss, eMiss);
    if (!(eMiss <= REL)) fail(`${tag} miss: js=${m.min_range} py=${g.miss} rel=${eMiss.toExponential(2)}`);

    // 2) 峰值过载
    const ePeak = relErr(peak / 9.80665, g.peak_g);
    worst.peak = Math.max(worst.peak, ePeak);
    if (!(ePeak <= REL)) fail(`${tag} peak_g: js=${peak / 9.80665} py=${g.peak_g} rel=${ePeak.toExponential(2)}`);

    // 3) 结束时刻
    const eT = Math.abs(m.t - g.t);
    worst.t = Math.max(worst.t, eT);
    if (!(eT <= T_EPS)) fail(`${tag} t: js=${m.t} py=${g.t} diff=${eT}`);

    // 4) 离散状态
    if (m.done !== g.done) fail(`${tag} done: js=${m.done} py=${g.done}`);
    const jsHit = Number.isFinite(m.min_range) && m.min_range <= cfg.hit_radius;
    if (jsHit !== g.hit) fail(`${tag} hit: js=${jsHit} py=${g.hit}`);
    if (m.phase !== g.phase) fail(`${tag} phase: js=${m.phase} py=${g.phase}`);

    // 5) 分段制导专有状态 (非分段场景: golden 中 acq_* 恒为 null, acquired 无意义)
    if (cfg.phased) {
      if (!!m.acquired !== g.acquired) fail(`${tag} acquired: js=${m.acquired} py=${g.acquired}`);
      if (g.acq_t !== null && m.acquired) {
        const eAcqT = relErr(m.acq_t, g.acq_t);
        const eAcqR = relErr(m.acq_r, g.acq_r);
        worst.acq = Math.max(worst.acq, eAcqT, eAcqR);
        if (!(eAcqT <= REL_ACQ)) fail(`${tag} acq_t: js=${m.acq_t} py=${g.acq_t}`);
        if (!(eAcqR <= REL_ACQ)) fail(`${tag} acq_r: js=${m.acq_r} py=${g.acq_r}`);
      } else if (g.acq_t !== null) {
        fail(`${tag} Python 已截获但 JS 未截获`);
      }
    } else if (g.acq_t !== null) {
      fail(`${tag} 非分段场景不应有截获记录`);
    }
  }
}

console.log(`  对拍记录 ${checked}/${golden.length}`
  + `  最大相对误差: miss=${worst.miss.toExponential(2)}`
  + ` peak=${worst.peak.toExponential(2)} acq=${worst.acq.toExponential(2)}`
  + `  |t|max=${worst.t.toExponential(2)}s`);

if (failed === 0) {
  console.log('  网页版与 Python 版 12 场景数值一致 (容差 rel<=1e-6, t<=1 DT)');
  process.exit(0);
}
console.error(`  ${failed} 项不一致`);
process.exit(1);
