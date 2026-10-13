/* ============================================================
 * FPV 仿真台 前端逻辑
 *   配置采集 → POST /api/simulate → 渲染 KPI/诊断/报告/数据表/图表/首飞流程
 *   图表：原生 Canvas 折线/柱状，无外部依赖
 * ============================================================ */
'use strict';

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];

const state = { presets: null, demoConfigs: null, type: 'multirotor', R: null, baseline: null, loading: false };

const DEMO_LABELS = {
  default: '5寸竞速(默认)', longrange: '7寸长航时', heavy: '12S八轴载重',
  whoop: '1S牙签(反例)', wing: '1000mm飞翼', glider: '2m滑翔机',
  rocket8: '8寸垂起火箭', rocket10: '10寸垂起火箭',
};

// 表单字段（与 normalize() 对应）
const FIELDS = ['frameId', 'wingId', 'rocketId', 'motorType', 'escType', 'rotorCount', 'kv', 'cells',
  'capacity_mAh', 'chemistry', 'cRating', 'propD', 'propP', 'propB', 'propMaterial',
  'wireAwg', 'wireLen_cm', 'target_kmh', 'payload_g', 'altitude', 'tempC',
  'massOverride_g', 'arm_m', 'wingSpan', 'wingArea', 'bodyLength_m', 'bodyD_mm',
  'finSpan_m', 'finArea_m2'];

/* ---------------- 初始化 ---------------- */
async function init() {
  try {
    const r = await fetch('api/presets') // 相对路径：兼容独立运行(/)与主应用挂载(/sim/);
    const j = await r.json();
    state.presets = j.presets;
    state.demoConfigs = j.demoConfigs || {};
    fillSelect('#frameId', j.presets.frames);
    fillSelect('#wingId', j.presets.wings);
    fillSelect('#rocketId', j.presets.rockets);
    fillSelect('#motorType', j.presets.motors);
    fillSelect('#escType', j.presets.escs);
    fillSelect('#chemistry', j.presets.chemistries);
    fillSelect('#propMaterial', j.presets.materials);
    buildDemos(j.demos || []);
    loadTuning();
    setStatus('就绪 · 选择参数或演示配置');
    // 自动化冒烟测试钩子：?autotest=demo名 → 自动加载并运行
    const at = new URLSearchParams(location.search).get('autotest');
    if (at) setTimeout(() => loadDemo(at), 50);
  } catch (e) {
    setStatus('无法加载部件库：' + e.message, 'fail');
  }

  // 默认值
  setVal('frameId', 'fpv5'); setVal('wingId', 'wing1000');
  setVal('motorType', '2207'); setVal('escType', 'shott32');
  setVal('chemistry', 'lipo'); setVal('propMaterial', 'plastic');
  setVal('propD', 127); setVal('propP', 109); setVal('propB', 3);
  setVal('cells', 4); setVal('capacity_mAh', 1300); setVal('target_kmh', 120);
  setVal('kv', 1750);

  bind();
}

function fillSelect(sel, list) {
  const el = $(sel);
  if (!el) return;
  el.innerHTML = list.map(x => `<option value="${x.id}">${x.label}</option>`).join('');
}
function setVal(id, v) { const el = $('#' + id); if (el) el.value = v; }
function getVal(id) {
  const el = $('#' + id);
  if (!el) return undefined;
  const v = el.value.trim();
  if (v === '') return undefined;
  if (el.type === 'number') { const n = Number(v); return Number.isFinite(n) ? n : undefined; }
  return v;
}

function buildDemos(keys) {
  $('#demos').innerHTML = keys.map(k =>
    `<button class="btn" data-demo="${k}">${DEMO_LABELS[k] || k}</button>`).join('');
  $$('#demos .btn').forEach(b => b.onclick = () => loadDemo(b.dataset.demo));
}

/* ---------------- 事件绑定 ---------------- */
function bind() {
  $('#runBtn').onclick = run;
  $('#tuningReset').onclick = () => { resetTuningUI(); if (state.R) run(); };
  $('#tuningCompare').onclick = runCompare;
  $$('#typeSeg button').forEach(b => b.onclick = () => switchType(b.dataset.type));
  $$('#tabs button').forEach(b => b.onclick = () => {
    $$('#tabs button').forEach(x => x.classList.toggle('on', x === b));
    $$('.panel').forEach(p => p.classList.toggle('on', p.id === 'p-' + b.dataset.p));
    if (b.dataset.p === 'charts' && state.R) drawCharts(state.R);
    if (b.dataset.p === 'solutions' && state.R) drawSolutionCharts(state.R);
  });
  document.addEventListener('keydown', (e) => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') run();
  });
}

function switchType(t) {
  state.type = t;
  $$('#typeSeg button').forEach(x => x.classList.toggle('on', x.dataset.type === t));
  $('#mrCfg').style.display = t === 'multirotor' ? '' : 'none';
  $('#fwCfg').style.display = t === 'fixedwing' ? '' : 'none';
  $('#vtolCfg').style.display = t === 'vtol' ? '' : 'none';
  // 手动切类型时，把"属于另一类型默认值"的动力参数换成本类型合理值
  const num = id => { const v = Number(getVal(id)); return Number.isFinite(v) && getVal(id) !== undefined ? v : null; };
  if (t === 'fixedwing') {
    const m = getVal('motorType');
    if (!m || ['2207', '2205', '2306', '28065'].includes(m)) setVal('motorType', '4114');
    const kv = num('kv');
    if (kv == null || [1750, 2300, 1900, 1700, 1300].includes(kv)) setVal('kv', 650);
    const d = num('propD');
    if (d == null || d <= 152) { setVal('propD', 203); setVal('propP', 152); setVal('propB', 2); }
    const c = num('cells');
    if (c == null || c <= 6) setVal('cells', 4);
    const cap = num('capacity_mAh');
    if (cap == null || cap <= 1500) setVal('capacity_mAh', 2200);
  } else if (t === 'vtol') {
    // 垂起火箭：默认 6S、2806.5、8寸桨
    const m = getVal('motorType');
    if (!m || ['2207', '4114'].includes(m)) setVal('motorType', '28065');
    const kv = num('kv');
    if (kv == null || [1750, 650].includes(kv)) setVal('kv', 1300);
    const d = num('propD');
    if (d == null || d < 178) { setVal('propD', 203); setVal('propP', 114); setVal('propB', 3); }
    const c = num('cells');
    if (c == null || c < 4) setVal('cells', 6);
    const cap = num('capacity_mAh');
    if (cap == null || cap < 3000) setVal('capacity_mAh', 3000);
  } else {
    const m = getVal('motorType');
    if (!m || m === '4114') setVal('motorType', '2207');
    const kv = num('kv');
    if (kv == null || kv === 650) setVal('kv', 1750);
    const d = num('propD');
    if (d == null || d >= 203) { setVal('propD', 127); setVal('propP', 109); setVal('propB', 3); }
    const c = num('cells');
    if (c == null) setVal('cells', 4);
    const cap = num('capacity_mAh');
    if (cap == null || cap >= 2200) setVal('capacity_mAh', 1300);
  }
}

async function loadDemo(key) {
  const cfg = state.demoConfigs?.[key];
  if (!cfg) { setStatus(`未知演示 ${key}`, 'fail'); return; }
  // 先按类型归位，再清空、再套用演示配置（避免上次手工修改残留）
  switchType(cfg.type === 'fixedwing' ? 'fixedwing' : cfg.type === 'vtol' ? 'vtol' : 'multirotor');
  for (const f of FIELDS) { const el = $('#' + f); if (el) el.value = ''; }
  setVal('frameId', 'fpv5'); setVal('wingId', 'wing1000'); setVal('rocketId', 'rocket8');
  setVal('escType', 'shott32'); setVal('chemistry', 'lipo'); setVal('propMaterial', 'plastic');
  for (const [k, v] of Object.entries(cfg)) {
    const el = $('#' + k);
    if (el && v !== undefined && v !== null) el.value = v;
  }
  await run();
}

/* ---------------- 调参表（半经验系数 / 判据阈值） ---------------- */
const TUNING = { meta: null, dirty: new Map() };   // dirty: key → 数值（只记用户改过的）

async function loadTuning() {
  try {
    const r = await fetch('api/tuning');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    TUNING.meta = await r.json();
    buildTuning();
  } catch (e) {
    const g = $('#tuningGroups');
    if (g) g.innerHTML = `<div class="hint">调参表加载失败：${e.message}</div>`;
  }
}

function tuningVal(key, p) { return TUNING.dirty.has(key) ? TUNING.dirty.get(key) : p.def; }

function buildTuning() {
  const wrap = $('#tuningGroups');
  if (!wrap || !TUNING.meta) return;
  const { groups, params } = TUNING.meta;
  wrap.innerHTML = '';
  for (const g of groups) {
    const list = params.filter(p => p.group === g.id);
    if (!list.length) continue;
    const fs = document.createElement('fieldset');
    fs.className = 'tgroup';
    fs.innerHTML = `<legend>${g.label}</legend><div class="hint">${g.hint}</div>`;
    for (const p of list) {
      const row = document.createElement('div');
      row.className = 'field trow';
      row.dataset.key = p.key;
      const id = 't_' + p.key.replace(/\./g, '_');
      row.innerHTML =
        `<label for="${id}" title="${esc(p.desc)}">${p.label}</label>` +
        `<input id="${id}" type="number" min="${p.min}" max="${p.max}" step="${p.step}" value="${p.def}" placeholder="${p.def}">` +
        (p.unit ? `<span class="unit">${p.unit}</span>` : '');
      row.querySelector('input').addEventListener('input', (e) => {
        const raw = e.target.value;
        if (raw === '' || raw === String(p.def)) { TUNING.dirty.delete(p.key); row.classList.remove('on'); }
        else {
          let x = Number(raw);
          if (!Number.isFinite(x)) return;
          x = Math.min(p.max, Math.max(p.min, x));
          TUNING.dirty.set(p.key, x);
          row.classList.add('on');
        }
        state.baseline = null;          // 系数一改，旧的"默认基准"就作废
        updateTuningBadge();
      });
      fs.appendChild(row);
    }
    wrap.appendChild(fs);
  }
  updateTuningBadge();
}

function updateTuningBadge() {
  const b = $('#tuningBadge');
  if (b) b.textContent = TUNING.dirty.size ? ` · 已改 ${TUNING.dirty.size} 项` : '';
}

function resetTuningUI() {
  TUNING.dirty.clear();
  if (!TUNING.meta) return;
  for (const p of TUNING.meta.params) {
    const el = $('#t_' + p.key.replace(/\./g, '_'));
    if (el) { el.value = p.def; el.closest('.trow')?.classList.remove('on'); }
  }
  updateTuningBadge();
}

// 只把改过的参数发给服务端：未动 = 服务端默认，保证默认路径与改动前逐位一致
function tuningPayload() {
  if (!TUNING.dirty.size) return undefined;
  const out = {};
  for (const [k, v] of TUNING.dirty) {
    const [g, n] = k.split('.');
    (out[g] ||= {})[n] = v;
  }
  return out;
}

/* ---------------- 运行 ---------------- */
function collect() {
  const cfg = { type: state.type };
  for (const f of FIELDS) {
    const v = getVal(f);
    if (v !== undefined) cfg[f] = v;
  }
  const t = tuningPayload();
  if (t) cfg.tuning = t;
  return cfg;
}

// kind: '' 正常 | 'diag' 仿真成功但诊断有致命问题 | 'fail' 运行失败（冒烟测试只认 FAIL）
function setStatus(msg, kind = '') {
  const s = $('#status');
  s.textContent = msg;
  s.className = kind || '';
  document.title = (kind === 'fail' ? 'FAIL: ' : '') + msg + ' | FPV 仿真台';
}

async function run() {
  if (state.loading) return;
  state.loading = true;
  const btn = $('#runBtn');
  btn.disabled = true;
  btn.innerHTML = '<span class="spin"></span> 计算中';
  setStatus('BEMT / 工况 / 结构 / 控制 求解中…');
  const t0 = performance.now();
  try {
    const res = await fetch('api/simulate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(collect()),
    });
    if (!res.ok) {
      const e = await res.json().catch(() => ({}));
      throw new Error(e.error || `HTTP ${res.status}`);
    }
    const R = await res.json();
    const ms = Math.round(performance.now() - t0);
    state.R = R;
    render(R);
    const errs = R.issues.filter(i => i.level === 'error').length;
    setStatus(`完成 ${ms}ms · ${errs ? errs + ' 个致命问题' : '无致命问题'}`, errs > 0 ? 'diag' : '');
  } catch (e) {
    setStatus('失败：' + e.message, 'fail');
  } finally {
    state.loading = false;
    btn.disabled = false;
    btn.innerHTML = '▶ 运行仿真';
  }
}

// 用默认调参重跑一次，量化"你改的这些系数到底带来了多大差别"
async function runCompare() {
  if (state.loading || !state.R || !TUNING.dirty.size) return;
  state.loading = true;
  const btn = $('#tuningCompare');
  const old = btn.textContent;
  btn.disabled = true; btn.textContent = '对比中…';
  try {
    const cfg = collect();
    delete cfg.tuning;                       // 基准 = 全部默认系数
    const res = await fetch('api/simulate', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(cfg),
    });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    state.baseline = await res.json();
    renderTuneCmp(state.R, state.baseline);
  } catch (e) {
    setStatus('对比失败：' + e.message, 'fail');
  } finally {
    state.loading = false; btn.disabled = false; btn.textContent = old;
  }
}

// 统一可比的指标行（跨机型取空值安全的 getter）
const CMP_ROWS = [
  ['起飞重量', R => R.totalMass_g, 'g', 0],
  ['推重比', R => R.cases.twr, '', 2],
  ['悬停油门', R => R.cases.hover?.duty != null ? R.cases.hover.duty * 100 : null, '%', 1],
  ['悬停电流', R => R.cases.hover?.current, 'A', 1],
  ['悬停效率', R => R.cases.hover?.gPerW, 'g/W', 2],
  ['最高速度', R => R.cases.vmax, 'km/h', 1],
  ['机臂安全系数', R => R.structure?.armPlate?.safety, '', 2],
  ['10m 噪音', R => R.fluid?.noise10m, 'dB', 1],
  ['最大升阻比', R => R.cases.maxLD, '', 2],
  ['续航', R => R.cases.endurance?.hover ?? R.cases.endurance?.max, 'min', 1],
];

function renderTuneCmp(R, base) {
  const el = $('#tuneCmp');
  if (!el) return;
  const applied = R?.tuningApplied || [];
  if (!applied.length && !base) { el.style.display = 'none'; el.innerHTML = ''; return; }
  el.style.display = '';
  const rows = [];
  if (applied.length) {
    rows.push(`<div class="tc-head">已应用 <b>${applied.length}</b> 项调参：` +
      applied.map(a => `<span class="chip" title="默认 ${a.def}${a.unit ? ' ' + a.unit : ''}">${esc(a.label)} <b>${a.value}${a.unit ? ' ' + a.unit : ''}</b></span>`).join('') + `</div>`);
  }
  if (base) {
    const tbl = CMP_ROWS.map(([label, get, unit, d]) => {
      const a = get(R), b = get(base);
      if (a == null || b == null || !Number.isFinite(a) || !Number.isFinite(b)) return '';
      const dv = a - b;
      const pct = Math.abs(b) > 1e-9 ? dv / Math.abs(b) * 100 : null;
      const same = Math.abs(dv) < 1e-9;
      return `<tr><td>${label}</td><td>${fnum(a, d)}${unit}</td><td>${fnum(b, d)}${unit}</td>` +
        `<td class="${same ? 'same' : 'diff'}">${same ? '不变' : (dv > 0 ? '+' : '') + fnum(dv, d) + unit +
          (pct != null && Math.abs(pct) > 0.05 ? ` (${dv > 0 ? '+' : ''}${pct.toFixed(1)}%)` : '')}</td></tr>`;
    }).filter(Boolean).join('');
    rows.push(`<div class="tc-head">与<b>默认系数</b>对比（左＝当前，右＝默认）</div>
      <table class="tc"><thead><tr><th>指标</th><th>当前</th><th>默认</th><th>Δ</th></tr></thead><tbody>${tbl}</tbody></table>`);
  } else if (applied.length) {
    rows.push(`<div class="hint">点击「对比默认结果」量化这些系数带来的差别。</div>`);
  }
  el.innerHTML = `<div class="card tune-cmp">${rows.join('')}</div>`;
}
const fnum = (x, d) => Number(x).toFixed(d);

/* ---------------- 渲染 ---------------- */
function render(R) {
  $('#empty').style.display = 'none';
  for (const id of ['kpis', 'issues', 'tabs']) $('#' + id).style.display = '';
  renderKPIs(R);
  renderTuneCmp(R, state.baseline);
  renderIssues(R);
  renderOverview(R);
  renderTables(R);
  renderPlan(R);
  renderSolutions(R);
  // 图表切到该页时再画（canvas 需要可见尺寸），但先画一次保证默认页也能看
  $$('.panel').forEach(p => p.classList.toggle('on', p.id === 'p-overview'));
  $$('#tabs button').forEach(x => x.classList.toggle('on', x.dataset.p === 'overview'));
  drawCharts(R);
}

function kpi(k, v, unit, cls = '') {
  return `<div class="kpi ${cls}"><div class="k">${k}</div><div class="v">${v}${unit ? `<small>${unit}</small>` : ''}</div></div>`;
}
const f1 = (x, d = 1) => (x == null || !Number.isFinite(x)) ? '—' : Number(x).toFixed(d);

function renderKPIs(R) {
  const c = R.cases, out = [];
  // 判据阈值由后端随结果回传（与 diagnose() 同源），调参后 KPI 着色跟着变
  const th = R.thresholds || { twrError: 1, twrWarn: 1.6, twrHighMR: 6, twrHighVTOL: 9,
    hoverDutyError: 0.68, hoverDutyWarn: 0.55, hoverDutyMin: 0.18, ldMin: 8, vtolClimbMin: 200 };
  const dutyCls = (d) => d > th.hoverDutyError ? 'bad' : d > th.hoverDutyWarn ? 'warn' : d < th.hoverDutyMin ? 'warn' : 'good';
  out.push(kpi('起飞重量', f1(R.totalMass_g / 1000, 2), 'kg'));
  if (R.type === 'multirotor') {
    out.push(kpi('推重比', f1(c.twr, 2), '', c.twr < th.twrError ? 'bad' : c.twr > th.twrHighMR ? 'warn' : 'good'));
    out.push(kpi('悬停油门', f1(c.hover.duty * 100, 0), '%', dutyCls(c.hover.duty)));
    out.push(kpi('悬停效率', f1(c.hover.gPerW, 2), 'g/W', c.hover.gPerW > 4 ? 'good' : ''));
    out.push(kpi('最大速度', f1(c.vmax, 0), 'km/h'));
    out.push(kpi('悬停续航', f1(c.endurance?.hover, 1), 'min'));
    out.push(kpi('满油门电流', f1(c.full.current, 0), 'A',
      c.cRatingLimited ? 'bad' : ''));
    if (c.target) out.push(kpi(`目标 ${c.target.kmh}km/h`, c.target.reachable ? '可达' : '不可达', '',
      c.target.reachable ? 'good' : 'bad'));
    if (R.dynamics.pid) out.push(kpi('PID 超调', f1(R.dynamics.pid.response.overshoot, 1), '%',
      R.dynamics.pid.response.overshoot < 10 ? 'good' : 'bad'));
  } else if (R.type === 'vtol') {
    out.push(kpi('推重比', f1(c.twr, 2), '', c.twr < th.twrError ? 'bad' : c.twr > th.twrHighVTOL ? 'warn' : 'good'));
    out.push(kpi('悬停油门', f1(c.hover.duty * 100, 0), '%', dutyCls(c.hover.duty)));
    out.push(kpi('悬停效率', f1(c.hover.gPerW, 2), 'g/W', c.hover.gPerW > 4 ? 'good' : ''));
    out.push(kpi('失速速度', f1(c.vStall, 0), 'km/h'));
    out.push(kpi('最大速度', f1(c.vmax, 0), 'km/h'));
    out.push(kpi('最大升阻比', f1(c.maxLD, 1), '', c.maxLD >= th.ldMin ? 'good' : 'warn'));
    out.push(kpi('最省电速度', f1(c.minSinkSpeed, 0), 'km/h'));
    out.push(kpi('垂直爬升', f1(c.vertClimb?.vs_fpm, 0), 'ft/min', c.vertClimb?.vs_fpm > th.vtolClimbMin ? 'good' : 'warn'));
    out.push(kpi('满油门电流', f1(c.full.current, 0), 'A',
      c.cRatingLimited ? 'bad' : ''));
    if (c.target) out.push(kpi(`目标 ${c.target.kmh}km/h`, c.target.reachable ? '可达' : '不可达', '',
      c.target.reachable ? 'good' : 'bad'));
    if (R.vtol?.transition?.feasible !== undefined) out.push(kpi('转换走廊', R.vtol.transition.feasible ? '可达' : '不可达', '',
      R.vtol.transition.feasible ? 'good' : 'bad'));
    if (R.dynamics.pid) out.push(kpi('PID 超调', f1(R.dynamics.pid.response.overshoot, 1), '%',
      R.dynamics.pid.response.overshoot < 10 ? 'good' : 'bad'));
  } else {
    out.push(kpi('失速速度', f1(c.vStall, 0), 'km/h'));
    out.push(kpi('最大速度', f1(c.vmax, 0), 'km/h'));
    out.push(kpi('最大升阻比', f1(c.maxLD, 1), '', c.maxLD >= th.ldMin ? 'good' : 'warn'));
    out.push(kpi('最省电速度', f1(c.minSinkSpeed, 0), 'km/h'));
    out.push(kpi('桨距速度', f1(c.pitchSpeed_kmh, 0), 'km/h'));
    out.push(kpi('巡航续航', f1(c.endurance?.max, 1), 'min'));
    out.push(kpi('爬升率', f1(c.climb?.vs_fpm, 0), 'ft/min'));
    if (c.target) out.push(kpi(`目标 ${c.target.kmh}km/h`, c.target.reachable ? '可达' : '不可达', '',
      c.target.reachable ? 'good' : 'bad'));
  }
  $('#kpis').innerHTML = out.join('');
}

function renderIssues(R) {
  $('#issues').innerHTML = R.issues.map(i => `
    <div class="issue ${i.level}">
      <div class="t">${esc(i.title)}</div>
      <div class="m">${esc(i.msg)}</div>
      ${i.fix ? `<div class="f">${esc(i.fix)}</div>` : ''}
      ${i.solutionRef ? `<div class="sol-link" data-sol="1">💡 已生成改造方案「${esc(i.solutionRef.label)}」${i.solutionRef.reached ? '（可达标）' : ''} → <a href="#" class="jump-sol">查看「改造方案」页</a></div>` : ''}
    </div>`).join('');
  $$('#issues .jump-sol').forEach(a => a.onclick = (e) => {
    e.preventDefault();
    const btn = $$('#tabs button').find(b => b.dataset.p === 'solutions');
    if (btn) btn.click();
  });
}

function renderOverview(R) {
  const html = R.report.map(s => `
    <div class="card"><h3>${esc(s.title)}</h3><p>${esc(s.body)}</p></div>`).join('');
  // 附电池/部件速览
  const b = R.battery;
  const extra = `
    <div class="card"><h3>核心部件</h3>
      <p>电机 ${esc(R.motor.label)}（R₀=${f1(R.motor.R_m * 1000, 1)}mΩ，持续 ${f1(R.motor.iCont, 1)}A / 爆发 ${f1(R.motor.iBurst, 1)}A）</p>
      <p>桨叶 ${esc(R.prop.label)}（${esc(R.prop.material.label)}，桨尖负荷 ${f1(R.prop.mass_g, 1)}g）</p>
      <p>电调 ${esc(R.esc.label)}（R=${f1(R.esc.r * 1000, 1)}mΩ，开关损耗 ${f1(R.esc.swLoss * 100, 1)}%，响应 ${f1(R.esc.resp_ms, 1)}ms）</p>
      <p>电池 ${esc(b.label)}（${f1(b.energy_Wh, 1)}Wh，内阻 ${f1(b.rPack_mOhm, 1)}mΩ，${f1(b.maxCurrent_A, 0)}A 上限，满 ${f1(b.vFull, 1)}V → 空 ${f1(b.vCut, 1)}V）</p>
      <p>大气 ρ=${f1(R.air.rho, 3)} kg/m³ @ ${R.air.altitude}m / ${R.air.tempC}℃</p>
    </div>`;
  $('#p-overview').innerHTML = html + extra;
}

/* ---------------- 数据表 ---------------- */
function tbl(headers, rows) {
  return `<div class="tbl-wrap"><table><thead><tr>${headers.map(h => `<th>${h}</th>`).join('')}</tr></thead>
    <tbody>${rows.map(r => `<tr>${r.map(c => `<td class="${c?.n !== false ? 'num' : ''}">${c?.v ?? c ?? '—'}</td>`).join('')}</tr>`).join('')}</tbody></table></div>`;
}

function renderTables(R) {
  const c = R.cases;
  const parts = [];
  // 质量分解（占比分母用各部件实际质量之和，避免 massOverride 覆盖后占比爆炸）
  const m = R.mass;
  const partsTotal = m.total_g || (m.frame_g + m.motors_g + m.props_g + m.battery_g + m.esc_g + m.avionics_g + m.payload_g);
  const mt = R.totalMass_g;
  const hasOverride = Math.abs(mt - partsTotal) > 0.5;  // 用户覆盖了整机重量
  const massRows = [
    ['机架/机身', m.frame_g], ['电机 ×' + R.nMotors, m.motors_g], ['桨叶 ×' + R.nMotors, m.props_g],
    ['电池', m.battery_g], ['电调/线材', m.esc_g], ['航电(飞控/图传)', m.avionics_g],
    ['载荷', m.payload_g],
  ].filter(r => r[1] > 0);
  const pct = v => f1(v / partsTotal * 100, 1) + '%';
  const bar = (k, v) => `<div style="background:${barColor(k)};height:9px;width:${Math.max(2, Math.min(100, v / partsTotal * 100))}%;border-radius:3px"></div>`;
  const rows = massRows.map(([k, v]) => [k, f1(v, 0), pct(v), bar(k, v)])
    .concat([['<b>合计</b>', `<b>${f1(partsTotal, 0)}</b>`, '100%', '']]);
  if (hasOverride) {
    rows.push(['<b>用户覆盖</b>', `<b>${f1(mt, 0)}</b>`, `${f1((mt - partsTotal) / partsTotal * 100, 1)}%`, '']);
  }
  parts.push(`<div class="card"><h3>质量分解（合计 ${f1(partsTotal, 0)} g${hasOverride ? `，覆盖为 ${f1(mt, 0)} g` : ''}）</h3>` +
    tbl(['部件', '质量 g', '占比', '条'], rows) + '</div>');

  // 工况表
  if (R.type === 'multirotor') {
    const rows = [];
    rows.push(['悬停', f1(c.hover.duty * 100, 1) + '%', f1(c.hover.current, 1), f1(c.hover.power, 0),
      Math.round(c.hover.rpm), f1(c.hover.gPerW, 2), f1(c.hover.vTip, 0), '—']);
    rows.push(['满油门', '100%', f1(c.full.current, 1), f1(c.full.power, 0), Math.round(c.full.rpm),
      f1(c.full.gPerW, 2), f1(c.full.vTip, 0), f1(c.full.machTip, 2)]);
    if (c.target) rows.push([`目标 ${c.target.kmh}km/h`, f1(c.target.duty * 100, 1) + '%',
      f1(c.target.current, 1), f1(c.target.power, 0), Math.round(c.target.rpm), f1(c.target.gPerW, 2),
      f1(c.target.Va, 1) + ' m/s', c.target.reachable ? '✓ 可达' : '✗ 不可达']);
    parts.push(`<div class="card"><h3>关键工况</h3>` +
      tbl(['工况', '油门', '电流 A', '功率 W', '转速 rpm', 'g/W', '桨尖/前向 m/s', '马赫/状态'], rows) + '</div>');
  } else if (R.type === 'vtol') {
    const rows = [];
    rows.push(['悬停(多旋翼)', f1(c.hover.duty * 100, 1) + '%', f1(c.hover.current, 1), f1(c.hover.power, 0),
      Math.round(c.hover.rpm), f1(c.hover.gPerW, 2), f1(c.hover.vTip, 0), '—']);
    rows.push(['满油门(竖直)', '100%', f1(c.full.current, 1), f1(c.full.power, 0), Math.round(c.full.rpm),
      f1(c.full.gPerW, 2), f1(c.full.vTip, 0), f1(c.full.machTip, 2)]);
    rows.push(['垂直爬升', '100%', '—', '—', '—', '—', f1(c.vertClimb?.vs_fpm, 0) + ' ft/min', '—']);
    if (c.target) rows.push([`目标 ${c.target.kmh}km/h(巡航)`, f1(c.target.duty * 100, 1) + '%',
      f1(c.target.current, 1), f1(c.target.power, 0), Math.round(c.target.rpm), f1(c.target.gPerW, 2),
      f1(c.target.L_D, 1) + ' L/D', c.target.reachable ? '✓ 可达' : '✗ 不可达']);
    parts.push(`<div class="card"><h3>关键工况（双模）</h3>` +
      tbl(['工况', '油门', '电流 A', '功率 W', '转速 rpm', 'g/W', '桨尖/爬升/LD', '马赫/状态'], rows) + '</div>');
    // 转换走廊表
    if (R.vtol?.transition?.rows) {
      const tr = R.vtol.transition.rows;
      parts.push(`<div class="card"><h3>转换走廊（俯仰 90°→0°）</h3>` +
        tbl(['机头角°', '速度 km/h', '需推力 N', '可用 N', '裕度%', '差动α rad/s²', '状态', '区域'],
        tr.map(r => [r.phi_deg, r.V_kmh ?? '—', r.thrust_req_N ?? '—', r.thrust_avail_N,
          r.margin_pct, r.controlAlpha, r.safe ? '✓ 安全' : r.caution ? '⚠ 失速过渡带' : '✗ 不安全', r.mode]))
        + '</div>');
    }
  } else {
    const rows = [];
    const push = (name, s) => s && rows.push([name, f1(s.kmh ?? s.V * 3.6, 1), f1(s.CL, 3), f1(s.CD, 4),
      f1(s.L_D, 1), f1(s.power, 0), f1(s.current, 1), f1(s.duty * 100, 0) + '%',
      s.reachable === false ? '✗' : '✓']);
    push('最佳升阻比', c.bestLD);
    push('最省电点', c.bestEndurance);
    if (c.target) rows.push([`目标 ${c.target.kmh}`, f1(c.target.kmh, 1), f1(c.target.CL, 3),
      f1(c.target.CD, 4), f1(c.target.L_D, 1), f1(c.target.power, 0), f1(c.target.current, 1),
      f1(c.target.duty * 100, 0) + '%', c.target.reachable ? '✓' : '✗']);
    parts.push(`<div class="card"><h3>固定翼工况</h3>` +
      tbl(['工况', '速度 km/h', 'CL', 'CD', 'L/D', '功率 W', '电流 A', '油门', '可达'], rows) + '</div>');
  }

  // 速度扫描
  const sw = c.sweep || [];
  if (sw.length) {
    const isFW = R.type === 'fixedwing';
    const isVTOL = R.type === 'vtol';
    const hdr = isFW || isVTOL
      ? ['速度 km/h', 'CL', 'CD', 'L/D', '功率 W', '电流 A', '油门', '转速', '可达', '失速', '攻角°']
      : ['速度 km/h', '功率 W', '电流 A', '油门', '倾角°', 'g/W', '可达'];
    const row = isFW || isVTOL
      ? s => [f1(s.kmh, 1), f1(s.CL, 3), f1(s.CD, 4), f1(s.L_D, 1), f1(s.power, 0), f1(s.current, 1),
           f1(s.duty * 100, 0) + '%', Math.round(s.rpm || 0), s.reachable ? '✓' : '✗',
           s.stalled ? '✗' : '✓', f1(s.alphaDeg, 1)]
      : s => [f1(s.kmh, 1), f1(s.power, 0), f1(s.current, 1), f1((s.duty ?? 0) * 100, 0) + '%',
           f1(s.tiltDeg, 1), f1(s.gPerW, 2), s.reachable ? '✓' : '✗'];
    parts.push(`<div class="card"><h3>速度扫描（${sw.length} 点）</h3>` +
      tbl(hdr, sw.map(row)) + '</div>');
  }

  // 结构
  const a = R.structure.armPlate;
  parts.push(`<div class="card"><h3>结构强度</h3>` +
    tbl(['项目', '值', '判据'], [
      ['机臂碳板', `${a.width_mm} × <b>${a.thickness_mm}</b> mm`, `需 ≥${f1(a.required_mm, 2)}mm`],
      ['根部应力', `${f1(a.sigma_MPa, 1)} MPa`, `安全系数 ${a.safety}`],
      ['端部挠度', `${f1(a.deflection_mm, 2)} mm`, '越小越好'],
      ['机臂一阶频率', `${f1(R.structure.armFreq_Hz, 1)} Hz`, `桨叶通过频率 ${R.dynamics.actuator.passing_Hz} Hz`],
      ['桨叶离心应力', `${f1(R.structure.prop.tipStress_MPa, 1)} MPa`, `安全系数 ${R.structure.prop.safety}（${R.structure.prop.material}）`],
      ['电机座螺钉', `${f1(R.structure.screw.tau_MPa, 1)} MPa 剪切`, `安全系数 ${R.structure.screw.safety}`],
      ['转动惯量 Ixx', `${R.dynamics.inertia.Ixx} kg·m²`, `Izz=${R.dynamics.inertia.Izz}`],
    ]) + '</div>');

  // 能量链
  const e = R.energy;
  parts.push(`<div class="card"><h3>能量链（悬停/巡航工作点）</h3>` +
    tbl(['环节', '功率 W', '损耗 W', '效率'], [
      ['电池输出', f1(e.pBatt, 0), '—', '—'],
      ['电调 → 电机', f1(e.pBatt - e.pEsc, 0), f1(e.pEsc, 1), f1(e.etaEsc * 100, 1) + '%'],
      ['电机铜损', f1(e.pBatt - e.pEsc - e.pCopper, 0), f1(e.pCopper, 1), '—'],
      ['铁损+摩擦', f1(e.pMech, 0), f1(e.pIron, 1), f1(e.etaMotor * 100, 1) + '%'],
      ['气动功率', f1(e.pMech, 0), '—', f1(e.pMech / Math.max(e.pBatt, 1e-6) * 100, 1) + '% 全链'],
    ]) + '</div>');

  // 控制
  if (R.dynamics.pid) {
    const p = R.dynamics.pid, resp = p.response;
    parts.push(`<div class="card"><h3>PID 整定（速率环极点配置）</h3>` +
      tbl(['项', '值', '说明'], [
        ['执行器增益 K', f1(p.gain.K, 1) + ' rad/s²/duty', `单机推力差 ${f1(p.gain.dT_dDuty_N, 2)}N/duty，力臂 ${f1(p.gain.lever_m, 3)}m`],
        ['执行器滞后 τ', f1(p.plant.tau_ms, 1) + ' ms', `电机+桨一阶时间常数`],
        ['闭环带宽', f1(p.plant.wn_hz, 2) + ' Hz', `ζ=${f1(p.plant.zeta, 2)}`],
        ['P / I / D', `<b>${p.pid.P}</b> / ${p.pid.I} / ${p.pid.D}`, `0~255 量纲 ${p.pid.P_255} / ${p.pid.I_255} / ${p.pid.D_255}`],
        ['D 项低通', f1(p.pid.dFilter_ms, 1) + ' ms', `截止 ${f1(1000 / (2 * Math.PI * p.pid.dFilter_ms), 1)}Hz`],
        ['积分权限', f1(p.pid.iAuth * 100, 0) + '%', `限幅 ±${p.pid.iiClamp}`],
        ['阶跃超调', f1(resp.overshoot, 1) + '%', `目标 <10%`],
        ['上升 / 调节时间', `${f1(resp.rise_ms, 0)} / ${f1(resp.settling_ms, 0)} ms`, `终值 ${resp.final} rad/s（设定 10）`],
        ['抗扰残差', f1(resp.distResidual, 2) + ' rad/s', resp.distRejected ? '8% 常值扰动已消除 ✓' : '扰动消除不足 ✗'],
      ]) + '</div>');
  }

  $('#p-tables').innerHTML = parts.join('');
}

function barColor(k) {
  if (k.includes('电池')) return '#37b6ff';
  if (k.includes('电机')) return '#52e0a4';
  if (k.includes('机架')) return '#ffb020';
  if (k.includes('桨')) return '#c77dff';
  if (k.includes('航电')) return '#ff5d7d';
  return '#5f7a99';
}

/* ---------------- 首飞流程 ---------------- */
function renderPlan(R) {
  $('#p-plan').innerHTML = R.plan.map(ph => `
    <div class="phase">
      <div class="ph">${esc(ph.phase)}</div>
      <ul>${ph.items.map(it => `<li>${esc(it)}</li>`).join('')}</ul>
    </div>`).join('');
}

/* ---------------- 改造方案（目标驱动反向求解） ---------------- */
const CONF_LABEL = { high: '高置信', medium: '中置信', low: '低置信' };

function goalTextOf(s) {
  if (s.goalText) return s.goalText;
  return String(s.goal);
}

function fmtBeforeAfter(s) {
  const u = s.metricUnit || '';
  const fmt = v => (v == null || !Number.isFinite(v)) ? '—' : `${v}${u}`;
  const b = s.before?.[s.metric], a = s.after?.[s.metric];
  return `${fmt(b)} → ${fmt(a)}`;
}

function solCard(s, idx) {
  const badge = s.pareto
    ? '<span class="badge pareto">Pareto 权衡</span>'
    : (s.reached ? '<span class="badge ok">✓ 可达标</span>' : '<span class="badge warn">⚠ 未达标</span>');
  const conf = s.confidence ? `<span class="badge conf-${s.confidence}">${CONF_LABEL[s.confidence] || s.confidence}</span>` : '';
  const joint = s.joint ? `<span class="badge joint">联合满足 ${s.joint.satisfied}/${s.joint.total}</span>` : '';
  const mass = s.massDelta_g
    ? `重量 ${s.massDelta_g > 0 ? '+' : ''}${s.massDelta_g}g`
    : '重量不变';
  const marginTxt = (s.margin != null && s.direction !== 'frontier')
    ? `裕度 ${s.margin >= 0 ? '+' : ''}${s.margin}%` : '';
  const canApply = s.cfg && Object.keys(s.cfg).length;

  const rows = [];
  if (!s.pareto) rows.push(['目标', `${esc(s.label)}：${fmtBeforeAfter(s)}（目标 ${goalTextOf(s)}）`]);
  if (s.after) {
    const others = Object.entries(s.after)
      .filter(([k]) => k !== s.metric && !['twr', 'mass_g', 'vmax', 'endurance'].includes(k));
    const brief = [
      s.after.twr != null ? `推重比 ${s.after.twr}` : null,
      s.after.vmax != null ? `最高速 ${s.after.vmax}km/h` : null,
      s.after.endurance != null ? `续航 ${s.after.endurance}min` : null,
    ].filter(Boolean).join('、');
    if (brief) rows.push(['改造后', brief + `（整机 ${s.after.mass_g ?? Math.round((s.before?.mass_g ?? 0) + (s.massDelta_g || 0))}g）`]);
    if (others.length) rows.push(['其余指标', others.map(([k, v]) => `${k}=${v}`).join('、')]);
  }
  if (s.joint?.goals) {
    rows.push(['目标达成', s.joint.goals.map(g => `${g.ok ? '✓' : '✗'}${esc(g.label)}`).join('；')]);
  }
  if (s.note) rows.push(['备注', esc(s.note)]);

  return `<div class="card sol-card">
    <h3>#${idx + 1} ${esc(s.label)} ${badge}${conf}${joint}</h3>
    <div class="sol-meta">${esc(marginTxt)} · ${esc(mass)} · 改动 ${s.changes?.length ?? 0} 项</div>
    <ul class="sol-changes">${(s.changes || []).map(c => `<li>${esc(c.label)}</li>`).join('')}</ul>
    ${tbl(['项目', '内容'], rows)}
    ${s.sensitivity?.length ? `<div class="sol-sens">灵敏度（对「${esc(s.metricLabel || s.metric)}」的杠杆）：
      ${s.sensitivity.map(x => `<span title="${esc(x.sample)}">${esc(x.paramLabel || x.param)} ${x.leverage >= 0 ? '+' : ''}${x.leverage} ${esc(x.unit || '')}</span>`).join(' · ')}
    </div>` : ''}
    ${canApply ? `<button class="sol-apply" data-i="${idx}">应用此方案（回填表单并重跑）</button>` : ''}
  </div>`;
}

function renderSolutions(R) {
  const S = R.solutions;
  const host = $('#p-solutions');
  if (!S || !S.solutions?.length) {
    host.innerHTML = `<div class="card"><h3>改造方案</h3>
      <p>${S?.baseMetrics ? '当前配置没有 error/warn 级失败目标——所有关键指标均已达标，无需改造。'
        : '未触发反向求解（无 error/warn 级诊断）。修正失败配置后自动求解。'}</p></div>`;
    return;
  }
  const reached = S.solutions.filter(s => s.reached).length;
  const head = `<div class="card"><h3>目标驱动反向求解</h3>
    <p>针对当前 ${S.baseMetrics ? '失败指标' : '指标'} 自动生成可执行改造方案：共 ${S.solutions.length} 个方案（${reached} 个可达标${S.pareto?.length ? `，含 ${S.pareto.length} 个速度-续航权衡点` : ''}），
    扫描 ${S.scanned} 个配置点。每个方案均经完整物理仿真验证。</p></div>`;

  const cards = S.solutions.map((s, i) => solCard(s, i)).join('');

  // 图表：灵敏度杠杆（首个带灵敏度的方案）+ Pareto 前沿
  const solSens = S.solutions.find(s => s.sensitivity?.length);
  const charts = [];
  if (solSens) {
    charts.push(chartCard('参数灵敏度', `改哪个参数最有效（对「${esc(solSens.metricLabel || solSens.metric)}」的单位杠杆）`, 'ch-sens', ''));
  }
  if (S.pareto?.length >= 2) {
    charts.push(chartCard('速度-续航 Pareto 前沿', '左上为更优：更高速度与更长续航不可兼得时的权衡曲线', 'ch-pareto',
      legend([{ name: '候选配置', color: PALETTE[2] }, { name: '当前基准', color: PALETTE[3] }])));
  }
  host.innerHTML = head + `<div class="sol-list">${cards}</div>`
    + (charts.length ? `<div class="charts">${charts.join('')}</div>` : '');

  drawSolutionCharts(R);
  $$('#p-solutions .sol-apply').forEach(btn => {
    btn.onclick = () => applySolution(R, +btn.dataset.i);
  });
}

function drawSolutionCharts(R) {
  const S = R.solutions;
  if (!S) return;
  const $c = id => document.getElementById(id);
  if ($c('ch-sens')) {
    const sol = S.solutions.find(s => s.sensitivity?.length);
    if (sol) {
      barChart($c('ch-sens'), {
        items: sol.sensitivity.slice(0, 6).map(x => ({
          label: `${x.paramLabel || x.param} ${x.direction}`,
          v: Math.abs(x.leverage),
          color: x.leverage >= 0 ? PALETTE[1] : PALETTE[3],
        })),
        unit: '',
      });
    }
  }
  if ($c('ch-pareto') && S.pareto?.length >= 2) {
    const pts = [...S.pareto].sort((a, b) => a.after.vmax - b.after.vmax);
    lineChart($c('ch-pareto'), {
      series: [
        { name: '前沿', color: PALETTE[2], data: pts.map(p => ({ x: p.after.vmax, y: p.after.endurance })) },
        { name: '基准', color: PALETTE[3], data: [{ x: S.baseMetrics?.vmax ?? 0, y: S.baseMetrics?.endurance ?? 0 }, { x: S.baseMetrics?.vmax ?? 0, y: S.baseMetrics?.endurance ?? 0 }] },
      ],
      xLabel: '最高速度 km/h', yLabel: '续航 min',
    });
  }
}

/** 一键应用方案：把 cfg 回填表单并重跑 */
function applySolution(R, idx) {
  const s = R.solutions?.solutions?.[idx];
  if (!s?.cfg) return;
  for (const [k, v] of Object.entries(s.cfg)) {
    const el = $('#' + k);
    if (el && v !== undefined && v !== null) el.value = v;
  }
  setStatus(`已应用方案：${s.changeLabel}，正在重跑…`, 'diag');
  run();
}

/* ============================================================
 * Canvas 图表（折线 + 柱状，双 Y 轴，无依赖）
 * ============================================================ */
const PALETTE = ['#37b6ff', '#52e0a4', '#ffb020', '#ff5d7d', '#c77dff', '#7aa2ff'];

function cssVar(n) { return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }

/** 折线图：series = [{name, color, data:[{x,y}], axis:'l'|'r'}] */
function lineChart(canvas, { series, xLabel, yLabel, y2Label, refY }) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth || 460, H = canvas.clientHeight || 240;
  canvas.width = W * dpr; canvas.height = H * dpr;
  const g = canvas.getContext('2d');
  g.scale(dpr, dpr);
  g.clearRect(0, 0, W, H);
  g.font = '11px Cascadia Mono, Consolas, monospace';

  const padL = 52, padR = series.some(s => s.axis === 'r') ? 52 : 14, padT = 12, padB = 34;
  const iw = W - padL - padR, ih = H - padT - padB;

  const xs = series.flatMap(s => s.data.map(p => p.x));
  if (!xs.length) return;
  let x0 = Math.min(...xs), x1 = Math.max(...xs);
  if (x1 === x0) { x1 = x0 + 1; }
  const range = (arr, includeZero) => {
    let lo = Math.min(...arr), hi = Math.max(...arr);
    if (includeZero && lo > 0) lo = 0;
    if (hi === lo) { hi = lo + 1; }
    const m = (hi - lo) * 0.08;
    return [lo - (lo > 0 ? m : 0), hi + m];
  };
  const lSeries = series.filter(s => s.axis !== 'r');
  const rSeries = series.filter(s => s.axis === 'r');
  const [ly0, ly1] = range(lSeries.flatMap(s => s.data.map(p => p.y)), true);
  const [ry0, ry1] = rSeries.length ? range(rSeries.flatMap(s => s.data.map(p => p.y)), true) : [0, 1];

  const X = v => padL + (v - x0) / (x1 - x0) * iw;
  const YL = v => padT + ih - (v - ly0) / (ly1 - ly0) * ih;
  const YR = v => padT + ih - (v - ry0) / (ry1 - ry0) * ih;

  // 网格 + 左轴
  g.strokeStyle = cssVar('--line') || '#23303f';
  g.fillStyle = cssVar('--dim') || '#8a9bb0';
  g.lineWidth = 1;
  const ticks = 5;
  for (let i = 0; i <= ticks; i++) {
    const yv = ly0 + (ly1 - ly0) * i / ticks;
    const y = YL(yv);
    g.globalAlpha = 0.5;
    g.beginPath(); g.moveTo(padL, y); g.lineTo(padL + iw, y); g.stroke();
    g.globalAlpha = 1;
    g.textAlign = 'right'; g.textBaseline = 'middle';
    g.fillText(fmtNum(yv), padL - 6, y);
    if (rSeries.length) { g.textAlign = 'left'; g.fillText(fmtNum(ry0 + (ry1 - ry0) * i / ticks), padL + iw + 6, y); }
  }
  // X 轴
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (let i = 0; i <= 5; i++) {
    const xv = x0 + (x1 - x0) * i / 5;
    g.fillText(fmtNum(xv), X(xv), padT + ih + 6);
  }
  g.strokeStyle = cssVar('--line');
  g.beginPath(); g.moveTo(padL, padT); g.lineTo(padL, padT + ih); g.lineTo(padL + iw, padT + ih); g.stroke();

  // 参考线
  if (refY != null && refY >= ly0 && refY <= ly1) {
    g.strokeStyle = '#ffffff55'; g.setLineDash([5, 4]);
    g.beginPath(); g.moveTo(padL, YL(refY)); g.lineTo(padL + iw, YL(refY)); g.stroke();
    g.setLineDash([]);
  }

  // 曲线
  series.forEach((s, si) => {
    g.strokeStyle = s.color || PALETTE[si % PALETTE.length];
    g.lineWidth = 2;
    g.beginPath();
    s.data.forEach((p, i) => {
      const px = X(p.x), py = (s.axis === 'r' ? YR : YL)(p.y);
      i ? g.lineTo(px, py) : g.moveTo(px, py);
    });
    g.stroke();
  });

  // 轴标题
  g.fillStyle = cssVar('--dim');
  g.textAlign = 'left'; g.textBaseline = 'bottom';
  if (yLabel) g.fillText(yLabel, padL, padT - 2);
  if (y2Label && rSeries.length) { g.textAlign = 'right'; g.fillText(y2Label, padL + iw, padT - 2); }
  g.textAlign = 'right'; g.textBaseline = 'top';
  if (xLabel) g.fillText(xLabel, padL + iw, H - 12);
}

function fmtNum(v) {
  if (!Number.isFinite(v)) return '';
  const a = Math.abs(v);
  if (a >= 1000) return (v / 1000).toFixed(1) + 'k';
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  if (a >= 1) return v.toFixed(2);
  return v.toFixed(3);
}

/** 柱状图 */
function barChart(canvas, { items, unit }) {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth || 460, H = canvas.clientHeight || 240;
  canvas.width = W * dpr; canvas.height = H * dpr;
  const g = canvas.getContext('2d');
  g.scale(dpr, dpr);
  g.clearRect(0, 0, W, H);
  g.font = '11px Cascadia Mono, Consolas, monospace';
  if (!items.length) return;

  const padL = 96, padR = 14, padT = 10, padB = 24;
  const iw = W - padL - padR, ih = H - padT - padB;
  const max = Math.max(...items.map(i => i.v)) || 1;
  const bh = Math.min(24, ih / items.length - 4);

  g.textBaseline = 'middle';
  items.forEach((it, i) => {
    const y = padT + (ih / items.length) * i + (ih / items.length - bh) / 2;
    const w = Math.max(2, it.v / max * iw);
    g.fillStyle = it.color || PALETTE[i % PALETTE.length];
    g.fillRect(padL, y, w, bh);
    g.fillStyle = cssVar('--txt') || '#d7e1ec';
    g.textAlign = 'right';
    g.fillText(it.label, padL - 6, y + bh / 2);
    g.textAlign = 'left';
    g.fillText(`${fmtNum(it.v)}${unit || ''}`, padL + w + 5, y + bh / 2);
  });
}

function legend(items) {
  return `<div class="legend">${items.map(i => `<span><i style="background:${i.color}"></i>${i.name}</span>`).join('')}</div>`;
}

/* ---------------- 图表装配 ---------------- */
function chartCard(title, desc, canvasId, legendHtml) {
  return `<div class="chart-card"><h4>${title}</h4><div class="desc">${desc}</div>
    <canvas id="${canvasId}"></canvas>${legendHtml || ''}</div>`;
}

function drawCharts(R) {
  const host = $('#p-charts');
  const c = R.cases;
  const cards = [];
  const isVTOL = R.type === 'vtol';

  // 1) 推力-油门曲线（多旋翼）或 极曲线（固定翼/火箭）
  if (R.type === 'multirotor' && R.dynamics.thrustCurve?.length) {
    const tc = R.dynamics.thrustCurve;
    cards.push(chartCard('推力特性曲线', '油门 → 单电机推力(g) / 电流(A)，看线性度与动力余量', 'ch-thrust',
      legend([{ name: '推力 g', color: PALETTE[0] }, { name: '电流 A', color: PALETTE[1] }])));
  } else if ((R.type === 'fixedwing' || isVTOL) && c.sweep?.length) {
    cards.push(chartCard('升力-阻力极曲线', 'CL vs CD：曲线越靠左下，气动效率越高', 'ch-polar',
      legend([{ name: 'CL-CD 极曲线', color: PALETTE[0] }])));
  }

  // 2) 速度扫描：功率 + 效率
  if (c.sweep?.length) {
    if (isVTOL) {
      // 双模：悬停曲线（左）+ 巡航曲线（右）
      cards.push(chartCard('悬停/巡航双模特性', '左：悬停功率曲线；右：巡航功率与 L/D', 'ch-sweep',
        legend([{ name: '悬停功率 W', color: PALETTE[0] }, { name: '巡航功率 W', color: PALETTE[1] }, { name: 'L/D', color: PALETTE[2], r: 1 }])));
    } else if (R.type === 'fixedwing') {
      cards.push(chartCard('速度 vs 功率 / 升阻比', '找最省电巡航点与最大升阻比转场速度', 'ch-sweep',
        legend([{ name: '功率 W', color: PALETTE[0] }, { name: 'L/D', color: PALETTE[1], r: 1 }])));
    } else {
      cards.push(chartCard('速度 vs 功率 / 效率', '前向飞行功率与克每瓦效率，用于定巡航速度', 'ch-sweep',
        legend([{ name: '功率 W', color: PALETTE[0] }, { name: 'g/W', color: PALETTE[1], r: 1 }])));
    }
  }

  // 3) VTOL 转换走廊
  if (isVTOL && R.vtol?.transition?.rows?.length) {
    const tr = R.vtol.transition.rows;
    cards.push(chartCard('转换走廊', '机头 90°→0°：所需推力/可用推力/裕度/差动角加速度', 'ch-transition',
      legend([{ name: '需推力 N', color: PALETTE[0] }, { name: '可用 N', color: PALETTE[1] }, { name: '裕度 %', color: PALETTE[2], r: 1 }, { name: 'α rad/s²', color: PALETTE[3], r: 1 }])));
  }

  // 4) PID 阶跃响应
  if (R.dynamics.pid?.response?.data?.length) {
    cards.push(chartCard('速率环阶跃响应', '10 rad/s 设定值阶跃：目标超调 <10%、无持续振荡', 'ch-pid',
      legend([{ name: '角速率 rad/s', color: PALETTE[1] }])));
  }

  // 5) 能量链瀑布
  const e = R.energy;
  if (e?.pBatt > 0) {
    cards.push(chartCard('能量链损耗分解', '从电池到气动的逐级损耗（W）', 'ch-energy', ''));
  }

  host.innerHTML = `<div class="charts">${cards.join('')}</div>`;

  // 绘制
  const $c = id => document.getElementById(id);
  if ($c('ch-thrust')) {
    const tc = R.dynamics.thrustCurve;
    lineChart($c('ch-thrust'), {
      series: [
        { name: '推力', color: PALETTE[0], data: tc.map(p => ({ x: p.duty * 100, y: p.thrust_g })) },
        { name: '电流', color: PALETTE[1], axis: 'r', data: tc.map(p => ({ x: p.duty * 100, y: p.current })) },
      ],
      xLabel: '油门 %', yLabel: '推力 g', y2Label: '电流 A',
      refY: R.type === 'multirotor' ? (R.cases.W / R.nMotors / 9.80665 * 1000) : undefined,
    });
  }
  if ($c('ch-polar')) {
    lineChart($c('ch-polar'), {
      series: [{ name: 'CL-CD', color: PALETTE[0], data: c.sweep.map(s => ({ x: s.CD, y: s.CL })) }],
      xLabel: 'CD', yLabel: 'CL',
    });
  }
  if ($c('ch-sweep')) {
    const isVTOL = R.type === 'vtol';
    const fw = R.type === 'fixedwing';
    if (isVTOL) {
      // 双模：左轴悬停功率，右轴巡航功率+L/D
      // 悬停扫描点在 sweep 里有 tiltDeg 字段
      const hoverSw = c.sweep.filter(s => s.tiltDeg != null && s.tiltDeg < 10); // 小倾角≈悬停
      const cruiseSw = c.sweep.filter(s => s.CL != null && s.L_D != null);
      lineChart($c('ch-sweep'), {
        series: [
          { name: '悬停功率', color: PALETTE[0], data: hoverSw.map(s => ({ x: s.kmh, y: s.power })) },
          { name: '巡航功率', color: PALETTE[1], data: cruiseSw.map(s => ({ x: s.kmh, y: s.power })) },
          { name: 'L/D', color: PALETTE[2], axis: 'r', data: cruiseSw.map(s => ({ x: s.kmh, y: s.L_D })) },
        ],
        xLabel: '速度 km/h', yLabel: '功率 W', y2Label: 'L/D',
      });
    } else {
      lineChart($c('ch-sweep'), {
        series: [
          { name: '功率', color: PALETTE[0], data: c.sweep.filter(s => s.kmh > 0).map(s => ({ x: s.kmh, y: s.power })) },
          { name: fw ? 'L/D' : 'g/W', color: PALETTE[1], axis: 'r',
            data: c.sweep.filter(s => s.kmh > 0).map(s => ({ x: s.kmh, y: fw ? s.L_D : s.gPerW })) },
        ],
        xLabel: '速度 km/h', yLabel: '功率 W', y2Label: fw ? 'L/D' : 'g/W',
      });
    }
  }
  if ($c('ch-transition')) {
    const tr = R.vtol.transition.rows;
    lineChart($c('ch-transition'), {
      series: [
        { name: '需推力', color: PALETTE[0], data: tr.map(r => ({ x: r.phi_deg, y: r.thrust_req_N })) },
        { name: '可用推力', color: PALETTE[1], data: tr.map(r => ({ x: r.phi_deg, y: r.thrust_avail_N })) },
        { name: '裕度', color: PALETTE[2], axis: 'r', data: tr.map(r => ({ x: r.phi_deg, y: r.margin_pct })) },
        { name: '差动α', color: PALETTE[3], axis: 'r', data: tr.map(r => ({ x: r.phi_deg, y: r.controlAlpha })) },
      ],
      xLabel: '机头俯仰角 °', yLabel: '推力 N', y2Label: '裕度% / α rad/s²',
    });
  }
  if ($c('ch-pid')) {
    const d = R.dynamics.pid.response.data;
    lineChart($c('ch-pid'), {
      series: [{ name: '角速率', color: PALETTE[1], data: d.map(p => ({ x: p.t, y: p.v })) }],
      xLabel: '时间 s', yLabel: 'rad/s', refY: 10,
    });
  }
  if ($c('ch-energy')) {
    const items = [
      { label: '电调损耗', v: Math.max(0, e.pEsc), color: '#ff5d7d' },
      { label: '电机铜损', v: Math.max(0, e.pCopper), color: '#ffb020' },
      { label: '铁损+摩擦', v: Math.max(0, e.pIron), color: '#c77dff' },
      { label: '气动功率', v: Math.max(0, e.pMech), color: '#52e0a4' },
    ];
    barChart($c('ch-energy'), { items, unit: 'W' });
  }
}

/* ---------------- 工具 ---------------- */
function esc(s) {
  return String(s ?? '').replace(/[&<>"]/g, m => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[m]));
}

let resizeTimer;
window.addEventListener('resize', () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => { if (state.R && $('#p-charts').classList.contains('on')) drawCharts(state.R); }, 150);
});

// 未捕获异常也要暴露给冒烟测试
window.addEventListener('error', (e) => { document.title = 'FAIL: ' + (e.message || 'runtime error'); });

init();
