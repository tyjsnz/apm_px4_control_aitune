// 轻量 SVG 折线图渲染(无第三方依赖, 离线可用, 便于导出)
(function () {
  const FALLBACK_PALETTE = ['#58a6ff', '#f0883e', '#3fb950', '#f85149', '#d29922',
    '#a371f7', '#39c5cf', '#ff7b72', '#7ee787', '#ffa657'];

  function cssVar(name, fallback) {
    try {
      const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
      return v || fallback;
    } catch (e) { return fallback; }
  }

  // 随主题变化: 读 --chart-N (8 色) + --accent/--info 补足 10 槽
  function palette() {
    const p = [];
    for (let i = 1; i <= 8; i++) p.push(cssVar('--chart-' + i, FALLBACK_PALETTE[i - 1]));
    p.push(cssVar('--accent', FALLBACK_PALETTE[8]));
    p.push(cssVar('--info', FALLBACK_PALETTE[9]));
    return p;
  }

  const gridColor = () => cssVar('--border', '#30363d');
  const axisTextColor = () => cssVar('--text-tertiary', '#8b949e');
  const axisColor = () => cssVar('--text-muted', '#6e7681');

  const W = 860;
  const H = 240;
  const PAD = { l: 52, r: 14, t: 14, b: 26 };

  const num = (v) => {
    if (!isFinite(v)) return '';
    if (v === 0) return '0';
    const a = Math.abs(v);
    if (a >= 1000) return v.toFixed(0);
    if (a >= 100) return v.toFixed(1);
    if (a >= 1) return v.toFixed(2);
    return v.toFixed(3);
  };

  function svgEl(name, attrs) {
    const el = document.createElementNS('http://www.w3.org/2000/svg', name);
    for (const k in attrs) el.setAttribute(k, attrs[k]);
    return el;
  }

  function makeChart(g) {
    const card = document.createElement('div');
    card.className = 'chart-card';
    const h = document.createElement('h5');
    h.textContent = g.title;
    card.appendChild(h);

    const series = (g.series || []).filter((s) => s && s.t && s.t.length > 1);
    const wrap = document.createElement('div');
    wrap.className = 'chart-svg-wrap';
    card.appendChild(wrap);
    if (!series.length) {
      wrap.innerHTML = '<div class="chart-empty">无数据</div>';
      return card;
    }

    let xmin = Infinity; let xmax = -Infinity; let ymin = Infinity; let ymax = -Infinity;
    for (const s of series) {
      for (let i = 0; i < s.t.length; i++) {
        const x = s.t[i]; const y = s.v[i];
        if (typeof x === 'number') { if (x < xmin) xmin = x; if (x > xmax) xmax = x; }
        if (typeof y === 'number' && isFinite(y)) { if (y < ymin) ymin = y; if (y > ymax) ymax = y; }
      }
    }
    if (!isFinite(xmin) || !isFinite(ymin)) { wrap.innerHTML = '<div class="chart-empty">无数据</div>'; return card; }
    if (xmax === xmin) xmax = xmin + 1;
    if (ymax === ymin) { ymax = ymin + 1; ymin -= 1; }
    else {
      const padY = (ymax - ymin) * 0.08;
      ymin -= padY; ymax += padY;
    }

    const plotW = W - PAD.l - PAD.r;
    const plotH = H - PAD.t - PAD.b;
    const X = (t) => PAD.l + ((t - xmin) / (xmax - xmin)) * plotW;
    const Y = (v) => PAD.t + plotH - ((v - ymin) / (ymax - ymin)) * plotH;

    const svg = svgEl('svg', { viewBox: `0 0 ${W} ${H}`, width: '100%', class: 'chart-svg' });

    // 网格 + y 轴刻度
    const PALETTE = palette();
    for (let i = 0; i <= 4; i++) {
      const yy = PAD.t + (plotH * i) / 4;
      svg.appendChild(svgEl('line', {
        x1: PAD.l, y1: yy, x2: W - PAD.r, y2: yy,
        stroke: gridColor(), 'stroke-width': 0.6,
      }));
      const val = ymax - ((ymax - ymin) * i) / 4;
      const txt = svgEl('text', { x: PAD.l - 6, y: yy + 3, 'text-anchor': 'end', fill: axisTextColor(), 'font-size': 10 });
      txt.textContent = num(val);
      svg.appendChild(txt);
    }
    // x 轴刻度
    for (let i = 0; i <= 4; i++) {
      const xx = PAD.l + (plotW * i) / 4;
      svg.appendChild(svgEl('line', {
        x1: xx, y1: PAD.t, x2: xx, y2: PAD.t + plotH, stroke: gridColor(), 'stroke-width': 0.4,
      }));
      const tv = xmin + ((xmax - xmin) * i) / 4;
      const txt = svgEl('text', { x: xx, y: H - 8, 'text-anchor': 'middle', fill: axisTextColor(), 'font-size': 10 });
      txt.textContent = num(tv) + 's';
      svg.appendChild(txt);
    }
    // 轴
    svg.appendChild(svgEl('line', { x1: PAD.l, y1: PAD.t, x2: PAD.l, y2: PAD.t + plotH, stroke: axisColor(), 'stroke-width': 0.8 }));
    svg.appendChild(svgEl('line', { x1: PAD.l, y1: PAD.t + plotH, x2: W - PAD.r, y2: PAD.t + plotH, stroke: axisColor(), 'stroke-width': 0.8 }));

    const usedUnits = new Set();
    series.forEach((s, idx) => {
      const color = PALETTE[idx % PALETTE.length];
      const pts = [];
      for (let i = 0; i < s.t.length; i++) {
        const x = s.t[i]; const y = s.v[i];
        if (typeof x !== 'number' || typeof y !== 'number' || !isFinite(y)) continue;
        pts.push(`${X(x).toFixed(1)},${Y(y).toFixed(1)}`);
      }
      if (!pts.length) return;
      svg.appendChild(svgEl('polyline', {
        points: pts.join(' '), fill: 'none', stroke: color, 'stroke-width': 1.4,
        'stroke-linejoin': 'round',
      }));
      if (s.unit) usedUnits.add(s.unit);
    });
    wrap.appendChild(svg);

    // 图例
    const legend = document.createElement('div');
    legend.className = 'chart-legend';
    series.forEach((s, idx) => {
      const span = document.createElement('span');
      span.className = 'chart-legend-item';
      const sw = document.createElement('i');
      sw.style.background = PALETTE[idx % PALETTE.length];
      span.appendChild(sw);
      span.appendChild(document.createTextNode(s.name + (s.unit ? ` (${s.unit})` : '')));
      legend.appendChild(span);
    });
    card.appendChild(legend);
    return card;
  }

  let last = null;
  function render(container, groups) {
    if (!container) return;
    last = { container, groups };
    container.innerHTML = '';
    if (!groups || !groups.length) {
      container.innerHTML = '<div class="chart-empty">该日志未包含可绘制的传感器数据</div>';
      return;
    }
    for (const g of groups) container.appendChild(makeChart(g));
  }

  // 主题切换后按当前数据重绘 (SVG 静态, 不会像 canvas 那样自动刷新)
  if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('fy:theme', () => { if (last) render(last.container, last.groups); });
  }

  window.Charts = { render };
})();
