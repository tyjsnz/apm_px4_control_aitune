// 仿真引擎: 装配目标与导弹、按固定步长推进、判定命中/脱靶 (由 guide/simulation.py 移植)
import { Missile, MissileParams, Target } from './guidance.js';

// --------------------------------------------------------------- 配置 --
export class SimConfig {
  constructor(o = {}) {
    // ---- 仿真总体 ----
    this.mode = o.mode ?? 'both';                // "pn" | "pp" | "both"
    this.field_size = o.field_size ?? 10000.0;   // 战场边长 (m), 正方形
    this.t_max = o.t_max ?? 60.0;                // 最长仿真时间 (s)
    this.flyover_range = o.flyover_range ?? 2000.0;
    this.flyover_open_time = o.flyover_open_time ?? 0.3;
    // ---- 导弹初始状态 ----
    this.mx = o.mx ?? 800.0;
    this.my = o.my ?? 1500.0;
    this.m_heading = o.m_heading ?? 40.0;        // 初始航向 (deg)
    this.m_speed = o.m_speed ?? 600.0;           // 导弹速度 (m/s)
    // ---- 导弹制导参数 ----
    this.nav_ratio = o.nav_ratio ?? 4.0;         // N
    this.pp_gain = o.pp_gain ?? 4.0;             // K
    this.max_g = o.max_g ?? 9.0;
    this.tau = o.tau ?? 0.3;
    this.use_vc = o.use_vc ?? true;
    // ---- 目标初始状态 ----
    this.tx = o.tx ?? 8500.0;
    this.ty = o.ty ?? 6000.0;
    this.t_heading = o.t_heading ?? 200.0;       // 初始航向 (deg)
    this.t_speed = o.t_speed ?? 250.0;           // 目标速度 (m/s)
    // ---- 目标机动 ----
    this.t_mode = o.t_mode ?? 'straight';        // straight | left | right | sine
    this.t_turn = o.t_turn ?? 6.0;               // 转弯率 / 正弦幅度 (deg/s)
    this.t_period = o.t_period ?? 4.0;           // 正弦机动周期 (s)
    // ---- 分段制导 ----
    this.phased = o.phased ?? false;
    this.acq_range = o.acq_range ?? 4000.0;
    this.acq_fov = o.acq_fov ?? 90.0;
    this.mid_mode = o.mid_mode ?? 'pip';
    // ---- 命中判定 ----
    this.hit_radius = o.hit_radius ?? 25.0;
  }
}

// -------------------------------------------------------------- 仿真 --
export class Simulation {
  constructor(cfg = null) {
    this.DT = 0.02;                 // 仿真步长 (s)
    this.cfg = cfg ?? new SimConfig();
    this.t = 0.0;
    this.finished = false;
    this.target = null;
    this.missiles = {};
    this.reset();
  }

  reset() {
    // 按当前 cfg 重新构建目标与导弹
    const c = this.cfg;
    this.t = 0.0;
    this.finished = false;
    this.target = new Target(c.tx, c.ty, c.t_heading, c.t_speed,
      c.t_mode, c.t_turn, c.t_period);
    const params = new MissileParams({
      nav_ratio: c.nav_ratio, pp_gain: c.pp_gain, max_g: c.max_g, tau: c.tau,
      use_vc: c.use_vc, phased: c.phased, acq_range: c.acq_range,
      acq_fov: c.acq_fov, mid_mode: c.mid_mode,
    });
    this.missiles = {
      pn: new Missile('pn', '比例导引', '#ff9f43', c.mx, c.my, c.m_heading, c.m_speed, params),
      pp: new Missile('pp', '纯跟踪', '#ff5ad1', c.mx, c.my, c.m_heading, c.m_speed, params),
    };
    // 预置初始观测量, 保证未开始时界面数据显示正确
    for (const m of Object.values(this.missiles)) {
      const { r, los, los_dot, vc } = m.sense(this.target);
      m.range = r; m.los = los; m.los_dot = los_dot; m.vc = vc;
    }
  }

  get active_keys() {
    if (this.cfg.mode === 'pn') return ['pn'];
    if (this.cfg.mode === 'pp') return ['pp'];
    return ['pn', 'pp'];
  }

  get active() {
    return this.active_keys.map((k) => this.missiles[k]);
  }

  step() {
    // 推进一个 DT. 返回是否仍在运行
    if (this.finished) return false;
    const c = this.cfg;
    this.target.step(this.DT);

    for (const m of this.active) {
      if (m.done) continue;
      m.step(this.target, this.DT);

      // 接近/远离状态
      if (m.vc > 0.0) {
        m.closed_once = true;
        m.open_t = 0.0;
      } else {
        m.open_t += this.DT;
      }

      // 命中只打标记: 继续积分到"飞越目标"为止, 保证 min_range 是整条弹道真实最小距离
      if (m.min_range <= c.hit_radius) m.hit = true;

      const passed = m.closed_once
        && m.open_t >= c.flyover_open_time
        && m.range <= Math.max(c.flyover_range, 1.5 * m.min_range);
      if (passed) {
        m.done = true;
        m.result = Simulation._result(m, '飞越目标', c);
      } else if (m.t >= c.t_max) {
        m.done = true;
        m.result = Simulation._result(m, '超时', c);
      }
    }

    this.t += this.DT;
    if (this.active.every((m) => m.done)) this.finished = true;
    return !this.finished;
  }

  run(seconds) {
    // 原地推进 seconds 秒 (或直到仿真结束)
    const tEnd = this.t + seconds;
    while (!this.finished && this.t < tEnd) this.step();
  }

  static _result(m, why, cfg) {
    let s = m.hit ? `命中 (脱靶量 ${m.min_range.toFixed(1)} m)`
      : `脱靶 ${m.min_range.toFixed(1)} m (${why})`;
    if (m.params.phased && !m.acquired) s += ' · 导引头未截获';
    return s;
  }

  get running() {
    // 是否处于交战进行中 (已开始且未结束)
    return (!this.finished) && this.active.some((m) => m.t > 0.0);
  }

  summary() {
    return this.active.map((m) => `${m.name}: ${m.result}`).join('   |   ');
  }
}
