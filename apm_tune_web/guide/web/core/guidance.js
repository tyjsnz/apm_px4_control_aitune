// 导弹导引仿真 —— 导引律与质点运动学模型 (浏览器版)
// 由 guide/guidance.py 逐行移植, 运算顺序与 Python 版保持一致, 用于数值对拍。
// 坐标系: 二维平面, 米(m), x 右 y 上, 角度弧度从 +x 轴逆时针为正。

export const G0 = 9.80665;        // 重力加速度 (m/s^2), 用于 m/s^2 -> g
export const MID_GAIN = 2.5;      // 中段制导航向收敛增益 (1/s)

// ------------------------------------------------------------------ 工具 --
export function wrapPi(a) {
  // 把角度归一化到 (-pi, pi]; JS 的 % 即 C fmod (符号随被除数), 与 math.fmod 一致
  a = (a + Math.PI) % (2.0 * Math.PI);
  if (a < 0.0) a += 2.0 * Math.PI;
  return a - Math.PI;
}

export { wrapPi as wrapPiExact };   // 兼容别名

export function clamp(v, lo, hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

export function closestOnSegment(px, py, ax, ay, bx, by) {
  // 点 p 到线段 ab 的距离与最近点 (精确脱靶量)
  const vx = bx - ax, vy = by - ay;
  const wx = px - ax, wy = py - ay;
  const l2 = vx * vx + vy * vy;
  const t = l2 <= 1e-12 ? 0.0 : clamp((wx * vx + wy * vy) / l2, 0.0, 1.0);
  const cx = ax + t * vx, cy = ay + t * vy;
  return { d: Math.hypot(px - cx, py - cy), cx, cy };
}

// ------------------------------------------------------- 导弹制导参数 ----
export class MissileParams {
  constructor(o = {}) {
    this.nav_ratio = o.nav_ratio ?? 4.0;      // N  比例导引导航比
    this.pp_gain = o.pp_gain ?? 4.0;          // K  纯跟踪航向修正增益 (1/s)
    this.max_g = o.max_g ?? 9.0;              // 最大可用法向过载 (g)
    this.tau = o.tau ?? 0.3;                  // 执行机构一阶时间常数 (s)
    this.use_vc = o.use_vc ?? true;           // PN 系数用 Vc (否则用 Vm)
    // 分段制导: 中段(导引头未截获) -> 末段(已截获)
    this.phased = o.phased ?? false;
    this.acq_range = o.acq_range ?? 4000.0;   // 导引头截获距离 R_acq (m)
    this.acq_fov = o.acq_fov ?? 90.0;         // 导引头视场 FOV (全角, deg)
    this.mid_mode = o.mid_mode ?? 'pip';      // "pip" 预测拦截点 | "straight" 程序直飞
  }
}

// -------------------------------------------------------------- 目标飞机 --
export class Target {
  constructor(x, y, headingDeg, speed, mode = 'straight', turnRateDeg = 6.0, period = 4.0) {
    this.x0 = +x; this.y0 = +y;
    this.h0 = +headingDeg;
    this.speed = +speed;
    this.mode = mode;
    this.turn_rate = +turnRateDeg;
    this.period = +period;
    this.reset();
  }

  reset() {
    this.x = this.x0; this.y = this.y0;
    this.heading = (this.h0 * Math.PI) / 180.0;
    this.t = 0.0;
    this.omega = 0.0;                 // 当前转弯角速度 rad/s
    this.trail = [[this.x, this.y]];
  }

  get vx() { return this.speed * Math.cos(this.heading); }
  get vy() { return this.speed * Math.sin(this.heading); }
  get heading_deg() { return (this.heading * 180.0) / Math.PI; }

  step(dt) {
    const w = (this.turn_rate * Math.PI) / 180.0;
    if (this.mode === 'straight') this.omega = 0.0;
    else if (this.mode === 'left') this.omega = w;
    else if (this.mode === 'right') this.omega = -w;
    else this.omega = w * Math.sin((2.0 * Math.PI * this.t) / Math.max(this.period, 1e-3));

    this.heading = wrapPi(this.heading + this.omega * dt);
    this.x += this.vx * dt;
    this.y += this.vy * dt;
    this.t += dt;
    this.trail.push([this.x, this.y]);
  }

  position_at(tAhead) {
    // 外推 t_ahead 秒后的位置 (直线: p+v·t; 转弯: 恒角速度圆弧解析解)
    if (tAhead <= 0.0) return { x: this.x, y: this.y };
    const w = this.omega;
    if (Math.abs(w) < 1e-9) {
      return { x: this.x + this.vx * tAhead, y: this.y + this.vy * tAhead };
    }
    const ang = w * tAhead;
    const k = this.speed / w;
    const dx = k * (Math.sin(this.heading + ang) - Math.sin(this.heading));
    const dy = k * (Math.cos(this.heading) - Math.cos(this.heading + ang));
    return { x: this.x + dx, y: this.y + dy };
  }
}

// ---------------------------------------------------------- 拦截点预测 --
export function predictIntercept(target, mx, my, vm, iters = 5, tMax = 60.0) {
  // 迭代求解预测拦截点 (PIP) 与飞行时间 TOF
  let t = Math.hypot(target.x - mx, target.y - my) / Math.max(vm, 1.0);
  let ax = 0, ay = 0;
  for (let i = 0; i < Math.max(iters, 1); i++) {
    const p = target.position_at(t);
    ax = p.x; ay = p.y;
    t = clamp(Math.hypot(ax - mx, ay - my) / Math.max(vm, 1.0), 0.0, tMax);
  }
  const p = target.position_at(t);
  return { tof: t, x: p.x, y: p.y };
}

// -------------------------------------------------------------- 导弹 ----
export class Missile {
  constructor(kind, name, color, x, y, headingDeg, speed, params) {
    this.kind = kind;                 // "pn" / "pp"
    this.name = name;                 // 中文显示名
    this.color = color;
    this.x0 = +x; this.y0 = +y;
    this.h0 = +headingDeg;
    this.v0 = +speed;
    this.params = params;
    this.reset();
  }

  reset() {
    this.x = this.x0; this.y = this.y0;
    this.heading = (this.h0 * Math.PI) / 180.0;
    this.speed = this.v0;
    this.a = 0.0;                     // 当前实际法向过载 (m/s^2)
    this.a_cmd = 0.0;                 // 过载指令 (m/s^2)
    this.t = 0.0;
    // 观测量
    this.range = 0.0;
    this.los = 0.0;
    this.los_dot = 0.0;
    this.vc = 0.0;
    // 脱靶量
    this.min_range = Infinity;
    this.min_point = [this.x, this.y];
    this.min_t = 0.0;
    this.closed_once = false;
    this.open_t = 0.0;
    // 结果
    this.done = false;
    this.hit = false;
    this.result = '飞行中';
    // 分段制导状态
    this.acquired = !this.params.phased;
    this.acq_t = 0.0;
    this.acq_r = Infinity;
    this.acq_pos = null;
    this.trail_split = 0;
    this.phase = this.acquired ? '末段' : '中段';
    // 历史
    this.trail = [[this.x, this.y]];
    this.hist_t = []; this.hist_r = [];
    this.hist_a = []; this.hist_vc = []; this.hist_losdot = [];
  }

  sense(tgt) {
    // 视线角 lambda、视线转率 lambda_dot、接近速度 Vc
    const dx = tgt.x - this.x;
    const dy = tgt.y - this.y;
    const r2 = dx * dx + dy * dy;
    const r = Math.sqrt(r2);
    const vx = this.speed * Math.cos(this.heading);
    const vy = this.speed * Math.sin(this.heading);
    const rvx = tgt.vx - vx;
    const rvy = tgt.vy - vy;

    const los = Math.atan2(dy, dx);
    const los_dot = (dx * rvy - dy * rvx) / Math.max(r2, 1.0);
    const vc = r > 1e-6 ? -(dx * rvx + dy * rvy) / r : 0.0;
    return { r, los, los_dot, vc };
  }

  guidance_command(r, los, losDot, vc) {
    const p = this.params;
    if (this.kind === 'pn') {
      const vRef = (p.use_vc && vc > 0.0) ? vc : this.speed;
      return p.nav_ratio * vRef * losDot;
    }
    return this.speed * p.pp_gain * wrapPi(los - this.heading);
  }

  midcourse_command(tgt) {
    // 中段 (导引头未截获): pip 指向预测拦截点 | straight 程序直飞
    const p = this.params;
    if (p.mid_mode === 'straight') return 0.0;
    const pip = predictIntercept(tgt, this.x, this.y, this.speed);
    const theta = Math.atan2(pip.y - this.y, pip.x - this.x);
    return this.speed * MID_GAIN * wrapPi(theta - this.heading);
  }

  _check_acquire(r, los) {
    // 导引头截获判据: R ≤ R_acq 且目标落在视场内 (相对弹轴)
    const p = this.params;
    if (r > p.acq_range) return;
    const fovHalf = 0.5 * ((Math.max(p.acq_fov, 0.1) * Math.PI) / 180.0);
    if (Math.abs(wrapPi(los - this.heading)) <= fovHalf) {
      this.acquired = true;
      this.acq_t = this.t;
      this.acq_r = r;
      this.acq_pos = [this.x, this.y];
      this.trail_split = this.trail.length;
      this.phase = '末段';
    }
  }

  step(tgt, dt) {
    if (this.done) return;
    const p = this.params;
    const { r, los, los_dot, vc } = this.sense(tgt);
    this.range = r; this.los = los; this.los_dot = los_dot; this.vc = vc;

    // 0) 分段制导: 先判导引头是否截获
    if (p.phased && !this.acquired) this._check_acquire(r, los);
    this.phase = (!p.phased || this.acquired) ? '末段' : '中段';

    // 1) 导引律 -> 过载指令 (带饱和)
    const aMax = p.max_g * G0;
    let cmd;
    if (this.phase === '中段') cmd = this.midcourse_command(tgt);
    else cmd = this.guidance_command(r, los, los_dot, vc);
    this.a_cmd = clamp(cmd, -aMax, aMax);

    // 2) 执行机构一阶滞后
    const alpha = 1.0 - Math.exp(-dt / Math.max(p.tau, 1e-3));
    this.a += (this.a_cmd - this.a) * alpha;
    this.a = clamp(this.a, -aMax, aMax);

    // 3) 运动学积分 (半隐式欧拉: 先转航向再平移)
    const px = this.x, py = this.y;
    this.heading = wrapPi(this.heading + (this.a / Math.max(this.speed, 1.0)) * dt);
    this.x += this.speed * Math.cos(this.heading) * dt;
    this.y += this.speed * Math.sin(this.heading) * dt;
    this.t += dt;

    // 4) 目标到本段弹道的最小距离 (精确脱靶量)
    const { d, cx, cy } = closestOnSegment(tgt.x, tgt.y, px, py, this.x, this.y);
    if (d < this.min_range) {
      this.min_range = d;
      this.min_point = [cx, cy];
      this.min_t = this.t;
    }
    this.range = Math.hypot(tgt.x - this.x, tgt.y - this.y);

    // 5) 历史记录
    this.hist_t.push(this.t);
    this.hist_r.push(this.range);
    this.hist_a.push(this.a);
    this.hist_vc.push(vc);
    this.hist_losdot.push(los_dot);
    this.trail.push([this.x, this.y]);
  }

  // 显示辅助
  get heading_deg() { return (this.heading * 180.0) / Math.PI; }
  get overload_g() { return this.a / G0; }
  get los_deg() { return (this.los * 180.0) / Math.PI; }
  get los_dot_dps() { return (this.los_dot * 180.0) / Math.PI; }
}
