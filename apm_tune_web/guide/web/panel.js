// 右侧控制面板 (由 guide/panel.py 的 ControlPanel 移植)
// 控件只负责: 写 cfg / 回调, 真正的重置交给 app.js。
import { clamp } from './core/guidance.js';

// ---------------------------------------------------------------- DOM 工具 --
function el(tag, cls, html) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (html !== undefined) e.innerHTML = html;
  return e;
}

function group(title) {
  const g = el('fieldset', 'grp');
  const l = el('legend', null, title);
  g.appendChild(l);
  return g;
}

/** 数值输入 (对应 QDoubleSpinBox: min/max/step/decimals/后缀) */
function spin(o) {
  const input = el('input', 'spin');
  input.type = 'number';
  input.min = o.min; input.max = o.max;
  input.step = o.step ?? 1;
  input.value = o.val;
  if (o.dec !== undefined) input.dataset.dec = o.dec;
  const wrap = el('label', 'spin-wrap');
  wrap.appendChild(input);
  if (o.suffix) wrap.appendChild(el('span', 'spin-suf', o.suffix));
  if (o.onChange) {
    // 对应 setKeyboardTracking(False): 失焦/回车才提交, 打字不触发重置
    // (syncFromCfg 程序化赋值不触发 change, 天然不会循环)
    input._last = String(input.value);      // syncFromCfg 更新它, 保持去重一致
    const commit = () => {
      let v = parseFloat(input.value);
      if (Number.isNaN(v)) return;
      v = clamp(v, o.min, o.max);
      input.value = o.dec ? v.toFixed(o.dec) : String(Math.round(v));
      if (input.value === input._last) return;   // change+blur 连发时只提交一次
      input._last = input.value;
      o.onChange(v);
    };
    input.addEventListener('change', commit);
    input.addEventListener('blur', commit);
    input.addEventListener('keydown', (e) => { if (e.key === 'Enter') { commit(); input.blur(); } });
  }
  return { wrap, input };
}

function row(labelText, widget) {
  const r = el('div', 'frow');
  r.appendChild(el('span', 'flab', labelText));
  r.appendChild(widget);
  return r;
}

function select(options, onChange) {
  // options: [[value, label], ...]
  const s = el('select', 'sel');
  for (const [v, lab] of options) {
    const o = el('option', null, lab);
    o.value = v;
    s.appendChild(o);
  }
  if (onChange) s.addEventListener('change', () => onChange(s.value));
  return s;
}

function checkbox(label, onChange) {
  const lab = el('label', 'ck');
  const i = el('input');
  i.type = 'checkbox';
  lab.appendChild(i);
  lab.appendChild(el('span', null, label));
  if (onChange) i.addEventListener('change', () => onChange(i.checked));
  return { wrap: lab, input: i };
}

const ROWS = [
  ['时间 t (s)', 't'],
  ['相对距离 R (m)', 'r'],
  ['接近速度 Vc (m/s)', 'vc'],
  ['视线角 λ (°)', 'los'],
  ['视线角速率 λ̇ (°/s)', 'losdot'],
  ['过载 a (m/s²)', 'a'],
  ['过载 n (g)', 'g'],
  ['弹速 Vm (m/s)', 'v'],
  ['制导阶段', 'phase'],
  ['截获时刻 (s)', 'acq'],
  ['最小距离 (m)', 'min'],
  ['结果', 'res'],
];

function fmtNum(v, d) {                 // Python {v:+.Nf} 的符号格式
  return (v >= 0 ? '+' : '') + v.toFixed(d);
}
function fmtInt(v) {                    // Python {:,.0f}
  return Math.round(v).toLocaleString('en-US');
}
function fmt1(v) {                      // Python {:,.1f}
  return v.toLocaleString('en-US', { minimumFractionDigits: 1, maximumFractionDigits: 1 });
}

// ================================================================ 面板 =====
export class Panel {
  /**
   * @param sim   Simulation
   * @param root  挂载容器
   * @param cb    {params, position(kind,x,y), run(on), step, reset, defaults, help}
   */
  constructor(sim, root, cb) {
    this.sim = sim;
    this.root = root;
    this.cb = cb;
    this._building = true;
    this._build();
    this._building = false;
    this.syncFromCfg();
  }

  _emit(fn, ...args) {
    if (this._building) return;
    this.cb[fn](...args);
  }

  _build() {
    const R = this.root;
    R.innerHTML = '';

    // ---------------- 帮助 ----------------
    this.btnHelp = el('button', 'btn-help', '📖&nbsp; 帮助文档 (算法与参数详解)&nbsp; F1');
    this.btnHelp.title = '打开 Markdown 帮助文档: 导引算法原理、每个仿真参数的含义与调参建议';
    this.btnHelp.addEventListener('click', () => this._emit('help'));
    R.appendChild(this.btnHelp);

    // ---------------- 导引方法 ----------------
    let g = group('导引方法');
    this.modeBox = select([
      ['both', '双弹对比 (PN vs PP)'],
      ['pn', '比例导引 (PN)'],
      ['pp', '纯跟踪 (PP)'],
    ], () => this._emit('params'));
    g.appendChild(this.modeBox);
    g.appendChild(el('p', 'tip', '对比模式下两枚导弹同起点同时发射，<br>分别按两种导引律飞行，便于比较。'));
    R.appendChild(g);

    // ---------------- 初始位置 ----------------
    g = group('初始位置 (可直接在画布上拖拽)');
    this.spMx = spin({ min: 0, max: 10000, val: 0, step: 100, suffix: ' m', onChange: (v) => this._pos('m', v, this.spMy.input.valueAsNumber) });
    this.spMy = spin({ min: 0, max: 10000, val: 0, step: 100, suffix: ' m', onChange: (v) => this._pos('m', this.spMx.input.valueAsNumber, v) });
    this.spTx = spin({ min: 0, max: 10000, val: 0, step: 100, suffix: ' m', onChange: (v) => this._pos('t', v, this.spTy.input.valueAsNumber) });
    this.spTy = spin({ min: 0, max: 10000, val: 0, step: 100, suffix: ' m', onChange: (v) => this._pos('t', this.spTx.input.valueAsNumber, v) });
    g.appendChild(row('导弹 X', this.spMx.wrap));
    g.appendChild(row('导弹 Y', this.spMy.wrap));
    g.appendChild(row('目标 X', this.spTx.wrap));
    g.appendChild(row('目标 Y', this.spTy.wrap));
    R.appendChild(g);

    // ---------------- 导弹参数 ----------------
    g = group('导弹参数');
    this.spVms = spin({ min: 100, max: 1500, val: 600, step: 10, suffix: ' m/s', onChange: () => this._emit('params') });
    this.spG = spin({ min: 1, max: 40, val: 9, step: 0.5, dec: 1, suffix: ' g', onChange: () => this._emit('params') });
    this.spTau = spin({ min: 0.02, max: 2.0, val: 0.3, step: 0.05, dec: 2, suffix: ' s', onChange: () => this._emit('params') });
    this.spN = spin({ min: 1, max: 8, val: 4, step: 0.5, dec: 1, onChange: () => this._emit('params') });
    this.spK = spin({ min: 0.5, max: 12, val: 4, step: 0.5, dec: 1, onChange: () => this._emit('params') });
    this.cbVc = checkbox('PN 系数用接近速度 Vc<br>(否则用弹速 Vm)', () => this._emit('params'));
    g.appendChild(row('速度 Vm', this.spVms.wrap));
    g.appendChild(row('最大过载 Gmax', this.spG.wrap));
    g.appendChild(row('执行机构 τ', this.spTau.wrap));
    g.appendChild(row('导航比 N (PN)', this.spN.wrap));
    g.appendChild(row('跟踪增益 K (PP)', this.spK.wrap));
    g.appendChild(this.cbVc.wrap);
    R.appendChild(g);

    // ---------------- 导引头 / 分段制导 ----------------
    g = group('导引头 / 分段制导 (中段→末段)');
    this.cbPhased = checkbox('启用分段制导<br>(未截获=中段飞行, 截获后=末段寻的)', () => {
      this._applyPhasedUi();
      this._emit('params');
    });
    g.appendChild(this.cbPhased.wrap);
    this.modeMid = select([
      ['pip', '指向预测拦截点 PIP'],
      ['straight', '程序直飞 (惯性)'],
    ], () => this._emit('params'));
    this.spAcq = spin({ min: 500, max: 15000, val: 4000, step: 100, suffix: ' m', onChange: () => this._emit('params') });
    this.spFov = spin({ min: 10, max: 180, val: 90, step: 5, suffix: ' °', onChange: () => this._emit('params') });
    g.appendChild(row('中段方式', this.modeMid));
    g.appendChild(row('截获距离 R_acq', this.spAcq.wrap));
    g.appendChild(row('截获视场 FOV', this.spFov.wrap));
    g.appendChild(el('p', 'tip', '截获条件: R ≤ R_acq 且目标落在视场内(相对弹轴);<br>未截获时按中段方式飞行, 截获后才转入 PN/PP。'));
    R.appendChild(g);

    // ---------------- 目标飞机 ----------------
    g = group('目标飞机');
    this.spVt = spin({ min: 0, max: 800, val: 250, step: 10, suffix: ' m/s', onChange: () => this._emit('params') });
    this.spTh = spin({ min: -360, max: 360, val: 200, step: 5, suffix: '°', onChange: () => this._emit('params') });
    this.modeT = select([
      ['straight', '匀速直线'],
      ['left', '持续左转'],
      ['right', '持续右转'],
      ['sine', '正弦机动'],
    ], () => this._emit('params'));
    this.spTurn = spin({ min: 0, max: 40, val: 6, step: 0.5, dec: 1, suffix: ' °/s', onChange: () => this._emit('params') });
    this.spPer = spin({ min: 1, max: 30, val: 4, step: 0.5, dec: 1, suffix: ' s', onChange: () => this._emit('params') });
    g.appendChild(row('速度 Vt', this.spVt.wrap));
    g.appendChild(row('初始航向', this.spTh.wrap));
    g.appendChild(row('机动模式', this.modeT));
    g.appendChild(row('转弯率/幅度', this.spTurn.wrap));
    g.appendChild(row('机动周期', this.spPer.wrap));
    R.appendChild(g);

    // ---------------- 场景 ----------------
    g = group('场景');
    this.spField = spin({ min: 4000, max: 20000, val: 10000, step: 500, suffix: ' m', onChange: () => this._emit('params') });
    this.spHit = spin({ min: 1, max: 200, val: 25, step: 5, suffix: ' m', onChange: () => this._emit('params') });
    g.appendChild(row('战场边长', this.spField.wrap));
    g.appendChild(row('命中判定半径', this.spHit.wrap));
    R.appendChild(g);

    // ---------------- 仿真控制 ----------------
    g = group('仿真控制');
    const btns = el('div', 'btnrow');
    this.btnRun = el('button', 'btn-run', '▶ 开始');
    this.btnStep = el('button', 'btn', '单步');
    this.btnReset = el('button', 'btn', '重置');
    this.btnDefault = el('button', 'btn', '默认参数');
    this.btnRun.addEventListener('click', () => this._emit('run', !this._running));
    this.btnStep.addEventListener('click', () => this._emit('step'));
    this.btnReset.addEventListener('click', () => this._emit('reset'));
    this.btnDefault.addEventListener('click', () => this._emit('defaults'));
    for (const b of [this.btnRun, this.btnStep, this.btnReset, this.btnDefault]) btns.appendChild(b);
    g.appendChild(btns);

    const spd = el('div', 'speedrow');
    spd.appendChild(el('span', 'flab', '倍速'));
    this.speed = el('input');
    this.speed.type = 'range';
    this.speed.min = 5; this.speed.max = 80; this.speed.value = 10;  // 0.5x ~ 8.0x
    this.speed.classList.add('slider');
    this.labSpeed = el('span', 'lab-speed', '1.0×');
    this.speed.addEventListener('input', () => {
      this.labSpeed.textContent = `${(this.speed.value / 10).toFixed(1)}×`;
    });
    spd.appendChild(this.speed);
    spd.appendChild(this.labSpeed);
    g.appendChild(spd);
    R.appendChild(g);

    // ---------------- 实时数据 ----------------
    g = group('实时数据');
    const tbl = el('table', 'dtable');
    const thead = el('thead');
    const trh = el('tr');
    for (const h of ['参数', '比例导引', '纯跟踪']) trh.appendChild(el('th', null, h));
    thead.appendChild(trh);
    tbl.appendChild(thead);
    const tbody = el('tbody');
    this.cells = [[], [], []];
    for (let r = 0; r < ROWS.length; r++) {
      const tr = el('tr');
      const td0 = el('td', 'rlab', ROWS[r][0]);
      tr.appendChild(td0);
      for (let c = 1; c <= 2; c++) {
        const td = el('td', 'rval', '-');
        tr.appendChild(td);
        this.cells[c].push(td);
      }
      tbody.appendChild(tr);
    }
    tbl.appendChild(tbody);
    g.appendChild(tbl);
    R.appendChild(g);

    R.appendChild(el('p', 'tip', '提示: 修改任意参数或拖拽位置后<br>仿真自动重置并暂停。'));
  }

  _pos(kind, x, y) {
    this._emit('position', kind, x, y);
  }

  _applyPhasedUi() {
    const on = this.cbPhased.input.checked;
    for (const w of [this.spAcq.wrap, this.spFov.wrap, this.modeMid]) {
      w.classList.toggle('dis', !on);
      const inp = w.querySelector ? w.querySelector('input,select') : null;
      if (inp) inp.disabled = !on;
      if (w.tagName === 'SELECT') w.disabled = !on;
    }
  }

  // ---------------------------------------------------------- 信号/状态 --
  get mode() { return this.modeBox.value; }

  get running() { return this._running; }

  speedFactor() { return this.speed.value / 10.0; }

  setRunning(on) {
    if (this._running !== on) {
      this._running = on;
      this.btnRun.textContent = on ? '⏸ 暂停' : '▶ 开始';
      this.btnRun.classList.toggle('on', on);
    }
  }

  // ------------------------------------------------------------ 同步 cfg --
  syncFromCfg() {
    const c = this.sim.cfg;
    this._building = true;
    const set = (input, v) => { input.value = v; input._last = v; };
    set(this.spMx.input, String(Math.round(c.mx)));
    set(this.spMy.input, String(Math.round(c.my)));
    set(this.spTx.input, String(Math.round(c.tx)));
    set(this.spTy.input, String(Math.round(c.ty)));
    set(this.spVms.input, String(c.m_speed));
    set(this.spG.input, c.max_g.toFixed(1));
    set(this.spTau.input, c.tau.toFixed(2));
    set(this.spN.input, c.nav_ratio.toFixed(1));
    set(this.spK.input, c.pp_gain.toFixed(1));
    this.cbVc.input.checked = c.use_vc;
    set(this.spVt.input, String(c.t_speed));
    set(this.spTh.input, String(c.t_heading));
    this.modeT.value = c.t_mode;
    set(this.spTurn.input, c.t_turn.toFixed(1));
    set(this.spPer.input, c.t_period.toFixed(1));
    set(this.spField.input, String(c.field_size));
    set(this.spHit.input, String(c.hit_radius));
    this.modeBox.value = c.mode;
    this.cbPhased.input.checked = c.phased;
    set(this.spAcq.input, String(c.acq_range));
    set(this.spFov.input, String(c.acq_fov));
    this.modeMid.value = c.mid_mode;
    this._building = false;
    this._applyPhasedUi();
  }

  writeCfg() {
    if (this._building) return;
    const c = this.sim.cfg;
    const num = (i) => i.valueAsNumber;
    c.mode = this.modeBox.value;
    c.m_speed = num(this.spVms.input);
    c.max_g = num(this.spG.input);
    c.tau = num(this.spTau.input);
    c.nav_ratio = num(this.spN.input);
    c.pp_gain = num(this.spK.input);
    c.use_vc = this.cbVc.input.checked;
    c.t_speed = num(this.spVt.input);
    c.t_heading = num(this.spTh.input);
    c.t_mode = this.modeT.value;
    c.t_turn = num(this.spTurn.input);
    c.t_period = num(this.spPer.input);
    c.field_size = num(this.spField.input);
    c.hit_radius = num(this.spHit.input);
    c.phased = this.cbPhased.input.checked;
    c.acq_range = num(this.spAcq.input);
    c.acq_fov = num(this.spFov.input);
    c.mid_mode = this.modeMid.value;
  }

  /** 外部(画布拖拽)更新位置输入框 */
  setPositions(kind, x, y) {
    this._building = true;
    const set = (input, v) => { input.value = v; input._last = v; };
    if (kind === 'm') {
      set(this.spMx.input, String(Math.round(x)));
      set(this.spMy.input, String(Math.round(y)));
    } else {
      set(this.spTx.input, String(Math.round(x)));
      set(this.spTy.input, String(Math.round(y)));
    }
    this._building = false;
  }

  // ------------------------------------------------------------- 实时表 --
  updateData() {
    const c = this.sim.cfg;
    const showPn = c.mode === 'pn' || c.mode === 'both';
    const showPp = c.mode === 'pp' || c.mode === 'both';
    this.cells[1].forEach((td) => { td.style.display = showPn ? '' : 'none'; });
    this.cells[2].forEach((td) => { td.style.display = showPp ? '' : 'none'; });

    for (const [col, key] of [[1, 'pn'], [2, 'pp']]) {
      const m = this.sim.missiles[key];
      const vals = Panel.values(m);
      const active = this.sim.active_keys.includes(key);
      for (let r = 0; r < ROWS.length; r++) {
        const td = this.cells[col][r];
        const txt = vals[ROWS[r][1]] ?? '-';
        if (td.textContent !== txt) {
          td.textContent = txt;
          td.style.color = active ? m.color : '#5b6b85';
        }
      }
    }
  }

  static values(m) {
    const finite = (v) => Number.isFinite(v);
    const r = finite(m.range) ? m.range : 0.0;
    const mn = finite(m.min_range) ? m.min_range : 0.0;
    return {
      t: m.t.toFixed(2),
      r: fmtInt(r),
      vc: m.vc.toFixed(0),
      los: m.los_deg.toFixed(1),
      losdot: fmtNum(m.los_dot_dps, 3),
      a: fmtNum(m.a, 1),
      g: fmtNum(m.overload_g, 2),
      v: m.speed.toFixed(0),
      phase: m.phase,
      acq: (m.params.phased && m.acquired) ? m.acq_t.toFixed(2) : '-',
      min: finite(m.min_range) ? fmt1(mn) : '-',
      res: m.result,
    };
  }
}
