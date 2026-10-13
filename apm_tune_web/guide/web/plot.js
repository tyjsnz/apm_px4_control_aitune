// 双面板曲线图 (由 guide/canvas.py 的 PlotWidget 移植): 上=R(t) 相对距离, 下=a(t) 过载
import { C, rgba, niceStep } from './canvas.js';

export class Plot {
  constructor(canvasEl, sim) {
    this.cv = canvasEl;
    this.ctx = canvasEl.getContext('2d');
    this.sim = sim;
    this.w = 0; this.h = 0; this.dpr = 1;
  }

  resize() {
    const rect = this.cv.getBoundingClientRect();
    this.dpr = window.devicePixelRatio || 1;
    this.w = rect.width;
    this.h = rect.height;
    this.cv.width = Math.max(1, Math.round(rect.width * this.dpr));
    this.cv.height = Math.max(1, Math.round(rect.height * this.dpr));
  }

  static fmtM(v) {
    return Math.abs(v) >= 1000 ? `${(v / 1000).toFixed(1)}k` : v.toFixed(0);
  }

  draw() {
    const p = this.ctx;
    p.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    p.clearRect(0, 0, this.w, this.h);
    p.fillStyle = C.BG;
    p.fillRect(0, 0, this.w, this.h);

    const w = this.w, h = this.h;
    const ml = 58.0, mr = 14.0, mt = 14.0, mb = 24.0, gap = 14.0;
    const pw = w - ml - mr;
    const ph = (h - mt - mb - gap) / 2.0;
    if (pw < 60 || ph < 40) return;
    const top = { x: ml, y: mt, w: pw, h: ph };
    const bot = { x: ml, y: mt + ph + gap, w: pw, h: ph };

    const acts = this.sim.active;
    const G0 = 9.80665;

    // ---------- 上: 相对距离 ----------
    let rmax = 0.0;
    for (const m of acts) if (m.hist_r.length) rmax = Math.max(rmax, Math.max(...m.hist_r));
    rmax = rmax > 0 ? rmax : this.sim.cfg.field_size;
    const curvesR = acts.map((m) => ({
      color: m.color, label: m.name, xs: m.hist_t, ys: m.hist_r, cur: m.range,
    }));
    const vlines = acts
      .filter((m) => this.sim.cfg.phased && m.acquired)
      .map((m) => ({ t: m.acq_t, label: m.name, color: m.color }));
    this._panel(p, top, '相对距离 R(t)', 'm', curvesR, 0.0, rmax,
      Plot.fmtM, [], false, vlines);

    // ---------- 下: 过载 ----------
    let amax = this.sim.cfg.max_g * G0 * 1.15;
    for (const m of acts) {
      if (m.hist_a.length) amax = Math.max(amax, Math.max(...m.hist_a.map((v) => Math.abs(v))) * 1.05);
    }
    const curvesA = acts.map((m) => ({
      color: m.color, label: m.name, xs: m.hist_t, ys: m.hist_a, cur: m.a,
    }));
    this._panel(p, bot, '导弹法向过载 a(t)', 'm/s²', curvesA, -amax, amax,
      (v) => v.toFixed(0),
      [
        { v: -this.sim.cfg.max_g * G0, label: '−Gmax' },
        { v: 0.0, label: '0' },
        { v: this.sim.cfg.max_g * G0, label: '+Gmax' },
      ], true, vlines);
  }

  _panel(p, rect, title, unit, curves, ymin, ymax, fmt, hlines, showX, vlines) {
    // 面板底
    p.fillStyle = '#0d1526';
    p.beginPath();
    if (p.roundRect) p.roundRect(rect.x, rect.y, rect.w, rect.h, 6);
    else p.rect(rect.x, rect.y, rect.w, rect.h);
    p.fill();
    p.strokeStyle = '#1d2c47';
    p.lineWidth = 1.0;
    p.beginPath();
    if (p.roundRect) p.roundRect(rect.x, rect.y, rect.w, rect.h, 6);
    else p.rect(rect.x, rect.y, rect.w, rect.h);
    p.stroke();

    let tmax = 5.0;
    for (const c of curves) if (c.xs.length) tmax = Math.max(tmax, Math.max(...c.xs));
    if (ymax <= ymin) ymax = ymin + 1.0;

    const left = rect.x, right = rect.x + rect.w;
    const topY = rect.y, botY = rect.y + rect.h;
    const px = (t) => left + (t / tmax) * rect.w;
    const py = (v) => botY - ((v - ymin) / (ymax - ymin)) * rect.h;

    // 横向网格
    p.font = '10px "Segoe UI", sans-serif';
    for (let i = 0; i < 5; i++) {
      const v = ymin + ((ymax - ymin) * i) / 4.0;
      const y = py(v);
      p.strokeStyle = C.GRID;
      p.lineWidth = 0.8;
      if (Math.abs(v) < 1e-9 && ymin < 0 && 0 < ymax) {
        p.strokeStyle = C.GRID_MAJOR;
        p.lineWidth = 1.2;
      }
      p.beginPath(); p.moveTo(left, y); p.lineTo(right, y); p.stroke();
      p.fillStyle = rgba(C.TXT, 210);
      p.textAlign = 'right';
      p.textBaseline = 'middle';
      p.fillText(fmt(v), left - 6, y);
      p.textAlign = 'left';
      p.textBaseline = 'alphabetic';
    }

    // 指定水平参考线 (±Gmax)
    for (const hl of hlines || []) {
      if (hl.v >= ymin && hl.v <= ymax) {
        p.strokeStyle = rgba('#ff5a5a', 90);
        p.lineWidth = 1.0;
        p.setLineDash([5, 4]);
        const y = py(hl.v);
        p.beginPath(); p.moveTo(left, y); p.lineTo(right, y); p.stroke();
        p.setLineDash([]);
        p.fillStyle = rgba('#ff8080', 200);
        p.fillText(hl.label, right - 42, y - 3);
      }
    }

    // 指定时刻竖线 (导引头截获时刻)
    (vlines || []).forEach((vl, i) => {
      if (vl.t >= 0.0 && vl.t <= tmax) {
        const x = px(vl.t);
        p.strokeStyle = rgba(vl.color, 120);
        p.lineWidth = 1.0;
        p.setLineDash([5, 4]);
        p.beginPath(); p.moveTo(x, topY); p.lineTo(x, botY); p.stroke();
        p.setLineDash([]);
        p.font = '9px "Segoe UI", sans-serif';
        p.fillStyle = rgba(vl.color, 215);
        p.fillText(`${vl.label}截获`, x + 3, topY + 11 + 11 * (i % 2));
        p.font = '10px "Segoe UI", sans-serif';
      }
    });

    // 时间刻度
    const tstep = niceStep(tmax, 6);
    for (let t = 0.0; t <= tmax + 1e-6; t += tstep) {
      const x = px(t);
      p.strokeStyle = C.GRID;
      p.lineWidth = 0.8;
      p.beginPath(); p.moveTo(x, topY); p.lineTo(x, botY); p.stroke();
      if (showX) {
        p.fillStyle = rgba(C.TXT, 210);
        p.fillText(t.toFixed(0), x - 8, botY + 15);
      }
    }
    if (showX) {
      p.fillStyle = rgba(C.TXT, 170);
      p.fillText('t / s', right - 44, botY + 15);
    }

    // 曲线
    for (const c of curves) {
      if (c.xs.length >= 2) {
        p.beginPath();
        p.moveTo(px(c.xs[0]), py(c.ys[0]));
        for (let i = 1; i < c.xs.length; i++) p.lineTo(px(c.xs[i]), py(c.ys[i]));
        p.lineCap = 'round'; p.lineJoin = 'round';
        p.strokeStyle = rgba(c.color, 55);
        p.lineWidth = 5.0;
        p.stroke();
        p.strokeStyle = rgba(c.color, 240);
        p.lineWidth = 1.8;
        p.stroke();
        // 当前值端点
        p.fillStyle = rgba(c.color, 250);
        p.beginPath();
        p.arc(px(c.xs[c.xs.length - 1]), py(c.ys[c.ys.length - 1]), 3.0, 0, Math.PI * 2);
        p.fill();
      }
    }

    // 标题(含单位) / 图例
    p.font = 'bold 11px "Segoe UI", sans-serif';
    p.fillStyle = C.TXT_HI;
    p.fillText(`${title}   [${unit}]`, left + 8, topY + 15);

    if (curves.length >= 1) {
      let lx = right - (76 * curves.length + 4);
      p.font = '10px "Segoe UI", sans-serif';
      for (const c of curves) {
        p.fillStyle = rgba(c.color, 245);
        p.fillText(`${c.label} ${fmt(c.cur)}`, lx, topY + 15);
        lx += 74;
      }
    }
  }
}
