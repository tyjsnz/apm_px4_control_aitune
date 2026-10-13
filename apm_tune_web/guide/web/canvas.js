// 战场画布 (由 guide/canvas.py 的 Canvas 移植, 纯 Canvas2D 绘制)
import { G0, clamp } from './core/guidance.js';

// ---------------------------------------------------------------- 配色 --
export const C = {
  BG: '#0a1020',
  GRID: '#152238',
  GRID_MAJOR: '#1e3050',
  BORDER: '#2b4468',
  TXT: '#8fa8cc',
  TXT_HI: '#e8f1ff',
  C_TARGET: '#3fd2ff',
  C_PN: '#ff9f43',
  C_PP: '#ff5ad1',
};

export function rgba(hex, alpha) {
  const h = hex.replace('#', '');
  const r = parseInt(h.slice(0, 2), 16);
  const g = parseInt(h.slice(2, 4), 16);
  const b = parseInt(h.slice(4, 6), 16);
  return `rgba(${r},${g},${b},${alpha / 255})`;
}

export function sgn(v, d) {          // 对应 Python 的 {v:+.Nf} 符号格式
  return (v >= 0 ? '+' : '') + v.toFixed(d);
}

export function niceStep(span, targetTicks = 6) {
  if (span <= 0) return 1.0;
  const raw = span / Math.max(targetTicks, 1);
  const mag = 10.0 ** Math.floor(Math.log10(raw));
  for (const m of [1.0, 2.0, 2.5, 5.0, 10.0]) {
    if (raw <= m * mag) return m * mag;
  }
  return 10.0 * mag;
}

export class Canvas {
  constructor(canvasEl, sim) {
    this.cv = canvasEl;
    this.ctx = canvasEl.getContext('2d');
    this.sim = sim;
    this.PICK_R = 30;                 // 鼠标拾取半径 (像素)
    this._drag = null;                // 当前拖拽对象 "m"/"t"
    this._hover = null;
    this.w = 0; this.h = 0; this.dpr = 1;
    this._bind();
  }

  // ------------------------------------------------------------ 尺寸 --
  resize() {
    const rect = this.cv.getBoundingClientRect();
    this.dpr = window.devicePixelRatio || 1;
    this.w = rect.width;
    this.h = rect.height;
    this.cv.width = Math.max(1, Math.round(rect.width * this.dpr));
    this.cv.height = Math.max(1, Math.round(rect.height * this.dpr));
  }

  // ------------------------------------------------------ 坐标变换 --
  _view() {
    const size = this.sim.cfg.field_size;
    const w = this.w, h = this.h;
    const margin = 16.0;
    const s = Math.min((w - 2 * margin) / size, (h - 2 * margin) / size);
    const ox = (w - size * s) / 2.0;
    const oy = (h - size * s) / 2.0;
    return { s, ox, oy, size };
  }

  w2s(x, y) {
    const { s, ox, oy, size } = this._view();
    return [ox + x * s, oy + (size - y) * s];
  }

  s2w(px, py) {
    const { s, ox, oy, size } = this._view();
    if (s <= 0) return [0.0, 0.0];
    return [(px - ox) / s, size - (py - oy) / s];
  }

  // -------------------------------------------------------- 鼠标事件 --
  _bind() {
    const pos = (ev) => {
      const r = this.cv.getBoundingClientRect();
      return [ev.clientX - r.left, ev.clientY - r.top];
    };
    const pick = (px, py) => {
      const c = this.sim.cfg;
      const cand = [['m', c.mx, c.my]];
      if (c.mode === 'pn' || c.mode === 'both') {
        const m = this.sim.missiles.pn;
        cand.push(['m', m.x, m.y]);
      }
      cand.push(['t', c.tx, c.ty]);
      let best = null, bestD = this.PICK_R;
      for (const [kind, x, y] of cand) {
        const [sx, sy] = this.w2s(x, y);
        const d = Math.hypot(sx - px, sy - py);
        if (d <= bestD) { best = kind; bestD = d; }
      }
      return best;
    };

    this.cv.addEventListener('mousedown', (ev) => {
      if (ev.button !== 0) return;
      const [px, py] = pos(ev);
      this._drag = pick(px, py);
      if (this._drag) {
        this.cv.style.cursor = 'grabbing';
        ev.preventDefault();
      }
    });
    window.addEventListener('mousemove', (ev) => {
      const [px, py] = pos(ev);
      if (this._drag) {
        const [x, y] = this.s2w(px, py);
        const size = this.sim.cfg.field_size;
        if (this.onDrag) this.onDrag(this._drag, clamp(x, 0.0, size), clamp(y, 0.0, size));
      } else {
        if (this.cv.width <= 0) return;
        const h = pick(px, py);
        if (h !== this._hover) {
          this._hover = h;
          this.cv.style.cursor = h ? 'grab' : 'crosshair';
        }
      }
    });
    window.addEventListener('mouseup', () => {
      this._drag = null;
      this.cv.style.cursor = this._hover ? 'grab' : 'crosshair';
    });
    // 触屏支持 (桌面版无, 网页版增量): 单指拖拽
    this.cv.addEventListener('touchstart', (ev) => {
      const t = ev.touches[0];
      const r = this.cv.getBoundingClientRect();
      this._drag = pick(t.clientX - r.left, t.clientY - r.top);
      if (this._drag) ev.preventDefault();
    }, { passive: false });
    this.cv.addEventListener('touchmove', (ev) => {
      if (!this._drag) return;
      const t = ev.touches[0];
      const r = this.cv.getBoundingClientRect();
      const [x, y] = this.s2w(t.clientX - r.left, t.clientY - r.top);
      const size = this.sim.cfg.field_size;
      if (this.onDrag) this.onDrag(this._drag, clamp(x, 0.0, size), clamp(y, 0.0, size));
      ev.preventDefault();
    }, { passive: false });
    this.cv.addEventListener('touchend', () => { this._drag = null; });
  }

  // -------------------------------------------------------- 绘制入口 --
  draw() {
    const p = this.ctx;
    p.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    p.clearRect(0, 0, this.w, this.h);
    p.fillStyle = C.BG;
    p.fillRect(0, 0, this.w, this.h);

    this._drawField(p);
    this._drawTrails(p);
    this._drawStartMarks(p);
    this._drawLos(p);
    for (const key of this.sim.active_keys) this._drawSeeker(p, this.sim.missiles[key]);
    this._drawTarget(p);
    for (const key of this.sim.active_keys) this._drawMissile(p, this.sim.missiles[key]);
    this._drawLegend(p);
    this._drawOverlay(p);
  }

  // -------------------------------------------------------------- 网格 --
  _drawField(p) {
    const { s, ox, oy, size } = this._view();
    let step = 1000.0;
    while (step * s < 55.0) step *= 2.0;

    p.strokeStyle = C.GRID;
    p.lineWidth = 0.8;
    const n = Math.floor(size / step);
    for (let i = 0; i <= n; i++) {
      const v = i * step;
      let a = this.w2s(v, 0), b = this.w2s(v, size);
      p.beginPath(); p.moveTo(a[0], a[1]); p.lineTo(b[0], b[1]); p.stroke();
      a = this.w2s(0, v); b = this.w2s(size, v);
      p.beginPath(); p.moveTo(a[0], a[1]); p.lineTo(b[0], b[1]); p.stroke();
    }

    // 主刻度 + 坐标标注 (每 2km)
    p.strokeStyle = C.GRID_MAJOR;
    p.lineWidth = 1.2;
    p.font = '10px "Segoe UI", sans-serif';
    p.fillStyle = rgba(C.TXT, 200);
    const maj = step * 2;
    let i = 0;
    while (i * maj <= size + 1e-6) {
      const v = i * maj;
      let a = this.w2s(v, 0), b = this.w2s(v, size);
      p.beginPath(); p.moveTo(a[0], a[1]); p.lineTo(b[0], b[1]); p.stroke();
      a = this.w2s(0, v); b = this.w2s(size, v);
      p.beginPath(); p.moveTo(a[0], a[1]); p.lineTo(b[0], b[1]); p.stroke();
      const sp = this.w2s(v, 0);
      p.fillText((v / 1000).toFixed(0), sp[0] + 3, Math.min(this.h - 4, sp[1] + 14));
      const sp2 = this.w2s(0, v);
      p.fillText((v / 1000).toFixed(0), Math.max(2.0, sp2[0] - 30), sp2[1] - 3);
      i++;
      if (maj * s < 1) break;
    }

    // 轴单位
    p.fillStyle = rgba(C.TXT, 160);
    p.font = 'bold 10px "Segoe UI", sans-serif';
    p.fillText('x / km', this.w - 46, this.h - 6);
    p.fillText('y / km', 6, 14);

    // 战场边界
    p.strokeStyle = C.BORDER;
    p.lineWidth = 1.6;
    const tl = this.w2s(0, size);
    p.strokeRect(tl[0], tl[1], size * s, size * s);
  }

  // -------------------------------------------------- 弹道辅助 --
  _path(p, points) {
    p.beginPath();
    let first = true;
    for (const [x, y] of points) {
      const pt = this.w2s(x, y);
      if (first) { p.moveTo(pt[0], pt[1]); first = false; } else p.lineTo(pt[0], pt[1]);
    }
  }

  _drawTrails(p) {
    // 目标航线: 宽淡 + 细亮虚线
    this._path(p, this.sim.target.trail);
    p.lineCap = 'round'; p.lineJoin = 'round';
    p.strokeStyle = rgba(C.C_TARGET, 70); p.lineWidth = 4.0;
    p.setLineDash([]); p.stroke();
    p.strokeStyle = rgba(C.C_TARGET, 210); p.lineWidth = 1.4;
    p.setLineDash([6, 5]); p.stroke();

    // 导弹弹道 (发光: 粗淡 + 细亮); 分段制导: 截获前暗色虚线, 截获后亮色实线
    const phased = this.sim.cfg.phased;
    for (const key of this.sim.active_keys) {
      const m = this.sim.missiles[key];
      const split = phased ? m.trail_split : 0;
      let pts;
      if (split >= 2) {
        p.strokeStyle = rgba(m.color, 110);
        p.lineWidth = 1.7;
        p.setLineDash([5, 4]);
        this._path(p, m.trail.slice(0, split));
        p.stroke();
        pts = m.trail.slice(split);
      } else {
        pts = m.trail;
      }
      if (pts.length < 2) { p.setLineDash([]); continue; }
      p.setLineDash([]);
      this._path(p, pts);
      p.strokeStyle = rgba(m.color, 60); p.lineWidth = 6.0;
      p.stroke();
      p.strokeStyle = rgba(m.color, 235); p.lineWidth = 2.0;
      p.stroke();
    }
    p.setLineDash([]);
  }

  _drawStartMarks(p) {
    // 初始位置标记 ×
    for (const [kind, x, y, color] of [
      ['m', this.sim.cfg.mx, this.sim.cfg.my, C.C_PN],
      ['t', this.sim.cfg.tx, this.sim.cfg.ty, C.C_TARGET],
    ]) {
      const c = this.w2s(x, y);
      p.strokeStyle = rgba(color, 150);
      p.lineWidth = 1.2;
      p.beginPath();
      p.moveTo(c[0] - 5, c[1] - 5); p.lineTo(c[0] + 5, c[1] + 5);
      p.moveTo(c[0] - 5, c[1] + 5); p.lineTo(c[0] + 5, c[1] - 5);
      p.stroke();
    }
  }

  // -------------------------------------------------------------- 视线 --
  _drawLos(p) {
    const tgt = this.sim.target;
    for (const key of this.sim.active_keys) {
      const m = this.sim.missiles[key];
      const a = this.w2s(m.x, m.y);
      const b = this.w2s(tgt.x, tgt.y);
      p.strokeStyle = rgba(m.color, 130);
      p.lineWidth = 1.1;
      p.setLineDash([4, 4]);
      p.beginPath(); p.moveTo(a[0], a[1]); p.lineTo(b[0], b[1]); p.stroke();
      p.setLineDash([]);
      if (this.sim.t > 0 && Math.hypot(b[0] - a[0], b[1] - a[1]) >= 90) {
        const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
        const dx = b[0] - a[0], dy = b[1] - a[1];
        const L = Math.hypot(dx, dy) || 1.0;
        const nx = (-dy / L) * 15.0, ny = (dx / L) * 15.0;
        this._text(p, [mx + nx, my + ny], `R=${(m.range / 1000).toFixed(2)} km`,
          rgba(m.color, 230), 11);
      }
    }
  }

  // -------------------------------------------------------------- 导引头 --
  _drawSeeker(p, m) {
    const cfg = this.sim.cfg;
    if (!cfg.phased) return;
    const c = this.w2s(m.x, m.y);
    const { s } = this._view();

    if (!m.acquired) {
      // (a) 截获距离圆 R_acq
      const rr = cfg.acq_range * s;
      if (rr >= 3.0) {
        p.strokeStyle = rgba(m.color, 70);
        p.lineWidth = 1.0;
        p.setLineDash([4, 4]);
        p.beginPath(); p.arc(c[0], c[1], rr, 0, Math.PI * 2); p.stroke();
        p.setLineDash([]);
      }
      // (b) 视场锥: 相对弹轴 ±FOV/2 (屏幕 y 向下 -> 角度取负)
      const half = 0.5 * ((cfg.acq_fov * Math.PI) / 180.0);
      const L = 78.0;
      const p1 = [c[0] + L * Math.cos(-m.heading - half), c[1] + L * Math.sin(-m.heading - half)];
      const p2 = [c[0] + L * Math.cos(-m.heading + half), c[1] + L * Math.sin(-m.heading + half)];
      const tip = [c[0] + L * Math.cos(-m.heading), c[1] + L * Math.sin(-m.heading)];
      p.fillStyle = rgba(m.color, 34);
      p.beginPath(); p.moveTo(c[0], c[1]); p.lineTo(p1[0], p1[1]); p.lineTo(p2[0], p2[1]);
      p.closePath(); p.fill();
      p.strokeStyle = rgba(m.color, 150);
      p.lineWidth = 1.0;
      p.setLineDash([1.5, 3]);
      p.beginPath();
      p.moveTo(c[0], c[1]); p.lineTo(p1[0], p1[1]);
      p.moveTo(c[0], c[1]); p.lineTo(p2[0], p2[1]);
      p.stroke();
      p.setLineDash([]);
      this._text(p, [tip[0] - 24, tip[1] + 16], `FOV ${cfg.acq_fov.toFixed(0)}°`,
        rgba(m.color, 140), 10);
    } else if (m.acq_pos) {
      // (c) 截获点标记: × + 时刻/距离
      const a = this.w2s(m.acq_pos[0], m.acq_pos[1]);
      p.strokeStyle = rgba(m.color, 210);
      p.lineWidth = 1.5;
      p.beginPath();
      p.moveTo(a[0] - 6, a[1] - 6); p.lineTo(a[0] + 6, a[1] + 6);
      p.moveTo(a[0] - 6, a[1] + 6); p.lineTo(a[0] + 6, a[1] - 6);
      p.stroke();
      this._text(p, [a[0] + 9, a[1] + 4],
        `截获 t=${m.acq_t.toFixed(1)}s R=${(m.acq_r / 1000).toFixed(2)}km`,
        rgba(m.color, 195), 10);
    }
  }

  // -------------------------------------------------------------- 目标 ----
  _drawTarget(p) {
    const tgt = this.sim.target;
    const c = this.w2s(tgt.x, tgt.y);

    // 命中半径
    const r = this.sim.cfg.hit_radius * this._view().s;
    if (r >= 2.0) {
      p.strokeStyle = rgba(C.C_TARGET, 90);
      p.lineWidth = 1.0;
      p.setLineDash([1, 3]);
      p.beginPath(); p.arc(c[0], c[1], r, 0, Math.PI * 2); p.stroke();
      p.setLineDash([]);
    }

    this._drawPlane(p, c, tgt.heading);

    let label = `目标 TGT  ${tgt.speed.toFixed(0)} m/s`;
    if (tgt.mode !== 'straight') label += `  ω=${sgn((tgt.omega * 180) / Math.PI, 1)}°/s`;
    this._text(p, [c[0] + 28, c[1] - 30], label, rgba(C.C_TARGET, 245), 12, true);
    if (this.sim.active[0].range > 1200.0) {
      this._text(p, [c[0] + 28, c[1] - 15],
        `(${(tgt.x / 1000).toFixed(2)}, ${(tgt.y / 1000).toFixed(2)}) km`,
        rgba(C.C_TARGET, 160), 10);
    }
  }

  _drawPlane(p, c, heading) {
    // 飞机俯视图标 (机头指向 +x, 绘制时旋转)
    p.save();
    p.translate(c[0], c[1]);
    p.rotate(-heading);

    const k = 0.82;
    const wing = [[6, 2.6], [-8, 20], [-15, 20], [-12, 2.6],
      [-12, -2.6], [-15, -20], [-8, -20], [6, -2.6]];
    const tail = [[-14, 1.8], [-21, 9], [-25, 9], [-22, 1.8],
      [-22, -1.8], [-25, -9], [-21, -9], [-14, -1.8]];
    const body = [[26, 0], [10, -3.6], [-14, -3.6], [-20, -2.6],
      [-20, 2.6], [-14, 3.6], [10, 3.6]];
    const poly = (pts) => {
      p.beginPath();
      pts.forEach(([x, y], i) => (i ? p.lineTo(x * k, -y * k) : p.moveTo(x * k, -y * k)));
      p.closePath();
    };

    p.fillStyle = rgba(C.C_TARGET, 70);
    poly(wing); p.fill();
    poly(tail); p.fill();
    p.fillStyle = rgba(C.C_TARGET, 210);
    poly(body); p.fill();

    p.strokeStyle = rgba(C.C_TARGET, 255);
    p.lineWidth = 1.4;
    poly(body); p.stroke();
    poly(wing); p.stroke();
    p.restore();
  }

  // -------------------------------------------------------------- 导弹 ----
  _drawMissile(p, m) {
    const c = this.w2s(m.x, m.y);

    // 速度矢量箭头
    const L = 46.0;
    const tip = [c[0] + L * Math.cos(-m.heading), c[1] + L * Math.sin(-m.heading)];
    p.strokeStyle = rgba(m.color, 200);
    p.lineWidth = 1.6;
    p.beginPath(); p.moveTo(c[0], c[1]); p.lineTo(tip[0], tip[1]); p.stroke();
    p.save();
    p.translate(tip[0], tip[1]);
    p.rotate(-((m.heading * 180) / Math.PI));
    p.fillStyle = rgba(m.color, 220);
    p.beginPath();
    p.moveTo(-8, -4); p.lineTo(0, 0); p.lineTo(-8, 4);
    p.closePath(); p.fill();
    p.restore();

    this._drawDart(p, c, m.heading, m.color, this.sim.running);

    // 对比模式: PN 标签在上, PP 标签在下, 避免重叠
    const both = this.sim.cfg.mode === 'both';
    let y0;
    if (both && m.kind === 'pp') y0 = 18.0;
    else if (both) y0 = -26.0;
    else y0 = -22.0;
    const x0 = m.range < 900.0 ? -112.0 : 24.0;
    const tag = this.sim.cfg.phased ? `  [${m.phase}]` : '';
    this._text(p, [c[0] + x0, c[1] + y0],
      `${m.name} ${m.speed.toFixed(0)} m/s${tag}`, rgba(m.color, 250), 12, true);
    this._text(p, [c[0] + x0, c[1] + y0 + 15],
      `n=${sgn(m.overload_g, 1)}g  λ̇=${sgn(m.los_dot_dps, 2)}°/s`,
      rgba(m.color, 170), 10);
  }

  _drawDart(p, c, heading, color, burning) {
    p.save();
    p.translate(c[0], c[1]);
    p.rotate(-((heading * 180) / Math.PI));

    const poly = (pts) => {
      p.beginPath();
      pts.forEach(([x, y], i) => (i ? p.lineTo(x, -y) : p.moveTo(x, -y)));
      p.closePath();
    };
    const flame = [[-14, -1.8], [-14, 1.8], [-26, 0]];
    p.fillStyle = rgba('#ffd166', burning ? 230 : 90);
    poly(flame); p.fill();

    const body = [[20, 0], [7, -3.0], [-11, -3.0], [-15, -1.6], [-15, 1.6], [-11, 3.0], [7, 3.0]];
    const fin1 = [[-7, 2.6], [-15, 9], [-18, 9], [-12, 2.6]];
    const fin2 = [[-7, -2.6], [-15, -9], [-18, -9], [-12, -2.6]];

    p.fillStyle = rgba(color, 80);
    poly(fin1); p.fill();
    poly(fin2); p.fill();
    p.fillStyle = rgba('#0a1020', 240);
    poly(body); p.fill();
    p.strokeStyle = rgba(color, 255);
    p.lineWidth = 1.5;
    poly(body); p.stroke();
    p.restore();
  }

  // -------------------------------------------------------------- 图例 ---
  _drawLegend(p) {
    if (this.sim.finished) return;   // 结束后用结果横幅代替图例
    const items = [['目标航线', C.C_TARGET, true]];
    if (this.sim.active_keys.includes('pn')) items.push(['比例导引(PN)弹道', C.C_PN, false]);
    if (this.sim.active_keys.includes('pp')) items.push(['纯跟踪(PP)弹道', C.C_PP, false]);
    if (this.sim.cfg.phased) items.push(['中段弹道(虚线)·视场锥·截获圆', C.TXT, true]);

    let x = 14.0, y = 26.0;
    p.font = '11px "Segoe UI", sans-serif';
    for (const [text, color, dashed] of items) {
      p.strokeStyle = rgba(color, 235);
      p.lineWidth = 2.4;
      p.setLineDash(dashed ? [5, 4] : []);
      p.beginPath(); p.moveTo(x, y - 4); p.lineTo(x + 22, y - 4); p.stroke();
      p.setLineDash([]);
      this._text(p, [x + 28, y], text, rgba(C.TXT_HI, 235), 11);
      y += 17;
    }
  }

  // -------------------------------------------------------------- 提示 ----
  _drawOverlay(p) {
    const sim = this.sim;
    const state = sim.finished ? '仿真结束' : (sim.running ? '运行中' : '就绪');
    this._text(p, [14, this.h - 34],
      `t = ${sim.t.toFixed(2).padStart(6)} s    ${state}`, rgba(C.TXT, 240), 12, true);

    if (!sim.running && !sim.finished) {
      this._text(p, [14, this.h - 16],
        '拖拽导弹/目标调整初始位置   空格: 开始/暂停   R: 重置   S: 单步   F1: 帮助文档',
        rgba(C.TXT, 170), 11);
    }

    if (sim.finished) this._banner(p, sim.summary());
  }

  _banner(p, text) {
    const rect = { x: 0, y: 10, w: this.w, h: 40 };
    p.fillStyle = rgba('#050a16', 215);
    roundRect(p, rect.x, rect.y, rect.w, rect.h, 8);
    p.fill();
    p.strokeStyle = '#3f5f95';
    p.lineWidth = 1.2;
    roundRect(p, rect.x, rect.y, rect.w, rect.h, 8);
    p.stroke();
    p.font = 'bold 13px "Segoe UI", sans-serif';
    const br = p.measureText(text).width;
    p.fillStyle = C.TXT_HI;
    p.fillText(text, (this.w - br) / 2, rect.y + rect.h / 2 + 5);
  }

  _text(p, pos, s, color, size = 11, bold = false) {
    p.font = `${bold ? 'bold ' : ''}${size}px "Segoe UI", sans-serif`;
    p.fillStyle = rgba('#000000', 165);
    p.fillText(s, pos[0] + 1, pos[1] + 1);
    p.fillStyle = color;
    p.fillText(s, pos[0], pos[1]);
  }
}

export function roundRect(p, x, y, w, h, r) {
  p.beginPath();
  p.moveTo(x + r, y);
  p.arcTo(x + w, y, x + w, y + h, r);
  p.arcTo(x + w, y + h, x, y + h, r);
  p.arcTo(x, y + h, x, y, r);
  p.arcTo(x, y, x + w, y, r);
  p.closePath();
}

export { G0 };
