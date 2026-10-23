// 扶摇 FuyaoTune — 运行时监视模块
//   仪表盘 (姿态仪 + 遥测卡片 + 飞控消息流)
//   实时数据 (ATTITUDE / VFR_HUD / GLOBAL_POSITION_INT / SYS_STATUS 曲线)
//   消息控制台 (MAVLink 消息速率 + 最近消息样本 + STATUSTEXT 文本流)
// 数据全部只在本机浏览器内存中, 不落盘、不上传。
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const ws = window.ws;

  const CHART_COLORS = ['#3B82F6', '#22C55E', '#EF4444', '#F59E0B', '#A855F7', '#06B6D4'];
  let themeVars = {};
  function cvar(name, fb) {
    if (name in themeVars) return themeVars[name];
    let v = fb;
    try { v = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb; } catch (e) { /* 读取失败走兜底 */ }
    themeVars[name] = v;
    return v;
  }
  // 系列颜色随主题: s.color 存的是夜间色, 按 CHART_COLORS 下标换算 --chart-N
  function colorOf(s) {
    const i = CHART_COLORS.indexOf(s.color);
    return i >= 0 ? cvar('--chart-' + (i + 1), s.color) : s.color;
  }
  const HIST_MAX = 6000;
  const EVENT_MAX = 200;

  const state = {
    view: 'dashboard',
    connected: false,
    arm: false,
    mode: '--',
    fw: 'apm',
    att: null,              // { roll, pitch, yaw } (deg)
    gps: { sats: null, fix: null },
    chartsPaused: false,
    consolePaused: false,
    sampleTimer: null,
    eventsDirty: false,
    chartSamples: 0,
    sampleRate: 0,
    params: new Set(),
    lastParamCount: -1,
    samples: [],            // 消息样本 (前端缓存)
    rateData: null,         // { counts, cum, total, windowMs }
  };

  // ---------- 实时曲线样本缓冲 ----------
  const hist = {};

  const CHARTS = [
    {
      canvas: 'rtAtt', leg: 'rtAttLeg', unit: '°',
      series: [
        { key: 'roll', label: '横滚', color: CHART_COLORS[0], fmt: (v) => v.toFixed(1) },
        { key: 'pitch', label: '俯仰', color: CHART_COLORS[1], fmt: (v) => v.toFixed(1) },
        { key: 'yaw', label: '航向', color: CHART_COLORS[2], fmt: (v) => v.toFixed(1) },
      ],
    },
    {
      canvas: 'rtRates', leg: 'rtRatesLeg', unit: '°/s',
      series: [
        { key: 'rollrate', label: '横滚角速度', color: CHART_COLORS[0], fmt: (v) => v.toFixed(1) },
        { key: 'pitchrate', label: '俯仰角速度', color: CHART_COLORS[1], fmt: (v) => v.toFixed(1) },
        { key: 'yawrate', label: '偏航角速度', color: CHART_COLORS[3], fmt: (v) => v.toFixed(1) },
      ],
    },
    {
      canvas: 'rtMotion', leg: 'rtMotionLeg', unit: '',
      series: [
        { key: 'alt', label: '相对高度 m', color: CHART_COLORS[4], fmt: (v) => v.toFixed(1) },
        { key: 'gspeed', label: '地速 m/s', color: CHART_COLORS[1], fmt: (v) => v.toFixed(2) },
        { key: 'climb', label: '垂速 m/s', color: CHART_COLORS[5], fmt: (v) => v.toFixed(2) },
      ],
    },
    {
      canvas: 'rtPower', leg: 'rtPowerLeg', unit: '',
      series: [
        { key: 'volt', label: '电压 V', color: CHART_COLORS[3], fmt: (v) => v.toFixed(2) },
        { key: 'curr', label: '电流 A', color: CHART_COLORS[2], fmt: (v) => v.toFixed(2) },
        { key: 'load', label: '负载 %', color: CHART_COLORS[0], fmt: (v) => v.toFixed(0) },
        { key: 'throttle', label: '油门 %', color: CHART_COLORS[5], fmt: (v) => v.toFixed(0) },
      ],
    },
  ];

  const FIX_LABEL = { 0: '无数据', 1: '无定位', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK 固定', 6: 'RTK 浮动' };

  // ==================== 通用小工具 ====================
  function setMetric(id, val, unit) {
    const el = $(id);
    if (!el) return;
    const html = val + (unit ? `<span class="metric-unit">${unit}</span>` : '');
    if (el.dataset.h !== html) {
      el.dataset.h = html;
      el.innerHTML = html;
    }
  }

  function metricClass(id, cls) {
    const el = $(id);
    if (!el) return;
    const want = 'metric-value' + (cls ? ' ' + cls : '');
    if (el.className !== want) el.className = want;
  }

  function setText(id, text) {
    const el = $(id);
    if (el && el.textContent !== text) el.textContent = text;
  }

  function timeStr(t) {
    const d = new Date(t);
    const p = (n, w) => String(n).padStart(w || 2, '0');
    return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`;
  }

  function fitCanvas(cv) {
    if (!cv) return 0;
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth || cv.width;
    const h = cv.clientHeight || cv.height || w;
    const bw = Math.max(1, Math.round(w * dpr));
    const bh = Math.max(1, Math.round(h * dpr));
    if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return w;
  }

  function pushSample(key, v) {
    if (v === null || v === undefined || !isFinite(v)) return;
    let a = hist[key];
    if (!a) a = hist[key] = [];
    a.push({ t: Date.now(), v });
    state.chartSamples++;
    if (a.length > HIST_MAX) a.splice(0, a.length - HIST_MAX);
  }

  function windowSecs() {
    const sel = $('rtWindow');
    return (sel && Number(sel.value)) || 30;
  }

  function pruneHist() {
    const cutoff = Date.now() - (windowSecs() + 2) * 1000;
    for (const k of Object.keys(hist)) {
      const a = hist[k];
      let i = 0;
      while (i < a.length && a[i].t < cutoff) i++;
      if (i > 0) a.splice(0, i);
    }
  }

  // ==================== 姿态仪 ====================
  function drawAtt() {
    const cv = $('attCanvas');
    if (!cv) return;
    const W = fitCanvas(cv);
    if (!W) return;
    const ctx = cv.getContext('2d');
    const cx = W / 2;
    const cy = W / 2;
    const R = W / 2 - 4;

    ctx.clearRect(0, 0, W, W);
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.clip();

    const att = state.att;
    if (!att) {
      ctx.fillStyle = cvar('--bg-primary', '#101018');
      ctx.fillRect(0, 0, W, W);
      ctx.fillStyle = cvar('--text-tertiary', '#555566');
      ctx.font = '12px ' + getComputedStyle(document.body).fontFamily;
      ctx.textAlign = 'center';
      ctx.fillText('等待 ATTITUDE 数据', cx, cy);
      ctx.restore();
      return;
    }

    const roll = att.roll;
    const pitch = att.pitch;
    const pxPerDeg = R / 40;

    ctx.translate(cx, cy);
    ctx.rotate(-roll * Math.PI / 180);
    const yH = pitch * pxPerDeg;   // 俯仰抬头 -> 地平线下移

    // 天空 / 地面
    ctx.fillStyle = '#17375E';
    ctx.fillRect(-2 * W, -2 * W, 4 * W, 2 * W + yH);
    ctx.fillStyle = '#4A3A26';
    ctx.fillRect(-2 * W, yH, 4 * W, 2 * W);

    // 地平线
    ctx.strokeStyle = '#E8E8F0';
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(-W, yH);
    ctx.lineTo(W, yH);
    ctx.stroke();

    // 俯仰梯线
    ctx.lineWidth = 1;
    ctx.font = '10px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'center';
    for (let d = -30; d <= 30; d += 10) {
      if (d === 0) continue;
      const y = yH - d * pxPerDeg;
      if (Math.abs(y) > R) continue;
      const halfW = (Math.abs(d) % 20 === 0) ? R * 0.28 : R * 0.16;
      ctx.strokeStyle = d > 0 ? 'rgba(240,240,245,.85)' : 'rgba(240,240,245,.55)';
      if (d < 0) ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(-halfW, y);
      ctx.lineTo(halfW, y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(240,240,245,.75)';
      ctx.fillText(String(Math.abs(d)), -halfW - 12, y + 3);
      ctx.fillText(String(Math.abs(d)), halfW + 12, y + 3);
    }
    ctx.restore();

    // 倾斜刻度弧 (机身坐标系, 不随姿态旋转)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = 'rgba(136,136,160,.75)';
    ctx.lineWidth = 1;
    ctx.font = '9px ' + getComputedStyle(document.body).fontFamily;
    ctx.fillStyle = 'rgba(136,136,160,.9)';
    ctx.textAlign = 'center';
    for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const rad = (-90 + a) * Math.PI / 180;
      const long = (a % 30 === 0) ? 8 : 4;
      const x1 = Math.cos(rad) * (R - 2);
      const y1 = Math.sin(rad) * (R - 2);
      const x2 = Math.cos(rad) * (R - 2 - long);
      const y2 = Math.sin(rad) * (R - 2 - long);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
      if (a % 30 === 0 && a !== 0) {
        ctx.fillText(String(Math.abs(a)), Math.cos(rad) * (R - 16), Math.sin(rad) * (R - 16) + 3);
      }
    }

    // 当前横滚指针 (顶部三角, 随姿态反向)
    ctx.save();
    ctx.rotate(-roll * Math.PI / 180);
    ctx.fillStyle = cvar('--chart-1', '#3B82F6');
    ctx.beginPath();
    ctx.moveTo(0, -R + 3);
    ctx.lineTo(-6, -R + 13);
    ctx.lineTo(6, -R + 13);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // 固定飞机标识
    ctx.strokeStyle = cvar('--chart-4', '#F59E0B');
    ctx.lineWidth = 2.4;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(-R * 0.46, 0); ctx.lineTo(-R * 0.16, 0);
    ctx.lineTo(-R * 0.06, R * 0.08);
    ctx.moveTo(R * 0.46, 0); ctx.lineTo(R * 0.16, 0);
    ctx.lineTo(R * 0.06, R * 0.08);
    ctx.stroke();
    ctx.fillStyle = cvar('--chart-4', '#F59E0B');
    ctx.beginPath();
    ctx.arc(0, 0, 2.6, 0, Math.PI * 2);
    ctx.fill();

    // 航向读数
    ctx.fillStyle = 'rgba(10,10,15,.75)';
    ctx.fillRect(-34, R - 26, 68, 18);
    ctx.fillStyle = '#F0F0F5';
    ctx.font = '600 11px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'center';
    const hdg = ((Math.round(att.yaw) % 360) + 360) % 360;
    ctx.fillText('HDG ' + hdg + '°', 0, R - 13);
    ctx.restore();
  }

  // ==================== 实时曲线 ====================
  function drawChart(def) {
    const cv = $(def.canvas);
    if (!cv) return;
    const w = fitCanvas(cv);
    const h = cv.clientHeight || 190;
    if (!w || !h) return;
    const ctx = cv.getContext('2d');
    ctx.clearRect(0, 0, w, h);

    const pad = { l: 42, r: 10, t: 8, b: 18 };
    const iw = w - pad.l - pad.r;
    const ih = h - pad.t - pad.b;
    if (iw <= 0 || ih <= 0) return;

    const now = Date.now();
    const win = windowSecs() * 1000;
    const t0 = now - win;

    let min = Infinity;
    let max = -Infinity;
    let any = false;
    for (const s of def.series) {
      const a = hist[s.key] || [];
      for (let i = a.length - 1; i >= 0; i--) {
        if (a[i].t < t0) break;
        if (a[i].v < min) min = a[i].v;
        if (a[i].v > max) max = a[i].v;
        any = true;
      }
    }
    if (!any) {
      ctx.fillStyle = cvar('--text-tertiary', '#555566');
      ctx.font = '12px ' + getComputedStyle(document.body).fontFamily;
      ctx.textAlign = 'center';
      ctx.fillText(state.connected ? '采样中…' : '连接飞控后开始采集', w / 2, h / 2);
      return;
    }
    if (max - min < 1e-6) { max += 0.5; min -= 0.5; }
    const span = max - min;
    min -= span * 0.08;
    max += span * 0.08;

    const X = (t) => pad.l + ((t - t0) / win) * iw;
    const Y = (v) => pad.t + (1 - (v - min) / (max - min)) * ih;

    // 网格
    ctx.strokeStyle = cvar('--border', 'rgba(255,255,255,.06)');
    ctx.lineWidth = 1;
    ctx.fillStyle = cvar('--text-tertiary', '#555566');
    ctx.font = '10px ' + getComputedStyle(document.body).fontFamily;
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
      const v = min + (max - min) * (i / 4);
      const y = Math.round(Y(v)) + .5;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
      ctx.fillText(fmtNum(v), pad.l - 5, y + 3);
    }
    ctx.textAlign = 'center';
    for (let i = 0; i <= 4; i++) {
      const t = t0 + win * (i / 4);
      const x = Math.round(X(t)) + .5;
      ctx.strokeStyle = cvar('--line-soft', 'rgba(255,255,255,.04)');
      ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.r); ctx.stroke();
      ctx.fillStyle = cvar('--text-tertiary', '#555566');
      ctx.fillText(i === 4 ? '现在' : '-' + Math.round((win / 1000) * (1 - i / 4)) + 's', x, h - 5);
    }

    // 数据曲线
    for (const s of def.series) {
      const a = hist[s.key] || [];
      if (a.length < 2) continue;
      ctx.strokeStyle = colorOf(s);
      ctx.lineWidth = 1.6;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < a.length; i++) {
        const p = a[i];
        if (p.t < t0) continue;
        const x = X(p.t);
        const y = Y(p.v);
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  }

  function fmtNum(v) {
    const a = Math.abs(v);
    if (a >= 1000) return v.toFixed(0);
    if (a >= 100) return v.toFixed(1);
    if (a >= 1) return v.toFixed(2);
    return v.toFixed(3);
  }

  function renderLegend(def) {
    const box = $(def.leg);
    if (!box) return;
    const html = def.series.map((s) => {
      const a = hist[s.key] || [];
      const last = a.length ? a[a.length - 1].v : null;
      const val = last === null ? '--' : s.fmt(last);
      return `<span><i style="background:${colorOf(s)}"></i>${s.label} <b>${val}</b></span>`;
    }).join('');
    if (box.dataset.h !== html) { box.dataset.h = html; box.innerHTML = html; }
  }

  function renderCharts() {
    for (const def of CHARTS) {
      drawChart(def);
      renderLegend(def);
    }
  }

  // ==================== 仪表盘 ====================
  function drawDashboard() {
    drawAtt();
  }

  function pushEvent(sev, text) {
    if (!text) return;
    state.events.unshift({ t: Date.now(), sev, text });
    if (state.events.length > EVENT_MAX) state.events.length = EVENT_MAX;
    state.eventsDirty = true;
  }

  function renderEvents() {
    const html = state.events.map((e) => {
      const cls = e.sev >= 6 ? ' sev' + e.sev : (e.sev >= 4 ? ' sev' + e.sev : '');
      const esc = String(e.text).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
      return `<div class="ev-row${cls}"><span class="ev-time">${timeStr(e.t)}</span><span class="ev-text">${esc}</span></div>`;
    }).join('');

    const dash = $('dashEvents');
    if (dash && dash.dataset.h !== html) {
      dash.dataset.h = html;
      dash.innerHTML = html || '<div class="ev-row"><span class="ev-text" style="color:var(--text-muted)">暂无飞控消息</span></div>';
    }
    const cn = $('cnTextList');
    if (cn && cn.dataset.h !== html) {
      cn.dataset.h = html;
      cn.innerHTML = html || '<div class="cn-empty">暂无飞控消息</div>';
    }
    setText('cnTextCount', state.events.length + ' 条');
  }

  // ==================== 消息控制台 ====================
  function renderRates() {
    const d = state.rateData;
    const body = $('cnRateBody');
    if (!body) return;
    const empty = $('cnRateEmpty');
    if (!d || !Object.keys(d.cum).length) {
      body.innerHTML = '';
      if (empty) empty.style.display = 'flex';
      setText('cnTotal', '总计 0');
      return;
    }
    if (empty) empty.style.display = 'none';

    const rows = Object.keys(d.cum).map((name) => ({
      name,
      period: d.counts[name] || 0,
      cum: d.cum[name],
    })).sort((a, b) => (b.period - a.period) || (b.cum - a.cum));

    const html = rows.map((r) => `
      <tr>
        <td class="dt-mono" style="color:var(--accent)">${r.name}</td>
        <td style="text-align:right;font-family:var(--font-mono);color:${r.period ? 'var(--text-primary)' : 'var(--text-disabled)'}">${r.period}</td>
        <td style="text-align:right;font-family:var(--font-mono);color:var(--text-secondary)">${r.cum}</td>
      </tr>`).join('');
    if (body.dataset.h !== html) { body.dataset.h = html; body.innerHTML = html; }
    setText('cnTotal', `总计 ${d.total} / ${(d.windowMs / 1000).toFixed(1)}s`);
  }

  function summarize(fields) {
    if (!fields || typeof fields !== 'object') return '';
    const keys = Object.keys(fields);
    const parts = [];
    for (const k of keys.slice(0, 4)) {
      let v = fields[k];
      if (Array.isArray(v)) v = '[' + v.slice(0, 4).join(',') + (v.length > 4 ? '…' : '') + ']';
      else if (typeof v === 'number') v = Number(v.toFixed(4));
      parts.push(k + '=' + v);
    }
    if (keys.length > 4) parts.push('…');
    return parts.join('  ');
  }

  function renderSamples() {
    const body = $('cnSampleBody');
    if (!body) return;
    const empty = $('cnSampleEmpty');
    const list = state.samples;
    if (!list.length) {
      body.innerHTML = '';
      if (empty) empty.style.display = 'flex';
      return;
    }
    if (empty) empty.style.display = 'none';
    const html = list.map((m, i) => `
      <tr class="cn-sample-row" data-i="${i}">
        <td class="dt-mono" style="color:var(--text-muted);white-space:nowrap">${timeStr(m.t)}</td>
        <td class="dt-mono" style="color:var(--accent);white-space:nowrap">${m.name}</td>
        <td class="dt-mono" style="color:var(--text-secondary);font-size:11px">${summarize(m.fields)}</td>
      </tr>`).join('');
    if (body.dataset.h !== html) { body.dataset.h = html; body.innerHTML = html; }
  }

  function requestSamples() {
    if (!state.connected || !ws || ws.readyState !== 1) return;
    ws.send(JSON.stringify({ type: 'mav_recent', max: 60 }));
  }

  function startSamplePolling() {
    if (state.sampleTimer) return;
    requestSamples();
    state.sampleTimer = setInterval(() => {
      if (state.view !== 'console' || state.consolePaused) return;
      requestSamples();
    }, 1000);
  }

  function stopSamplePolling() {
    if (state.sampleTimer) { clearInterval(state.sampleTimer); state.sampleTimer = null; }
  }

  // ==================== 消息入口 ====================
  function onMsg(msg) {
    switch (msg.type) {
      case 'connected': {
        state.connected = true;
        resetData();
        setText('cnStatus', '已连接 ' + (msg.path || ''));
        setText('dashConnChip', '已连接');
        setText('dashFwChip', '识别中…');
        pushEvent(6, `已连接 ${msg.path || ''} @ ${msg.baud || ''}`);
        break;
      }
      case 'disconnected': {
        state.connected = false;
        setText('cnStatus', '未连接');
        setText('dashConnChip', '未连接');
        setText('dashFwChip', '未连接');
        setText('gpsCount', '--');
        pushEvent(4, '链路已断开');
        renderEvents();
        break;
      }
      case 'heartbeat': {
        if (msg.flightMode) {
          state.mode = msg.flightMode;
          setText('dashMode', msg.flightMode);
        }
        if (msg.armed !== undefined && msg.armed !== state.arm) {
          state.arm = !!msg.armed;
          const el = $('dashArm');
          if (el) {
            el.textContent = state.arm ? '已解锁' : '上锁';
            el.className = 'pill ' + (state.arm ? 'pill--danger' : '');
          }
          pushEvent(state.arm ? 6 : 5, state.arm ? '飞控已解锁 (ARMED)' : '飞控已上锁 (DISARMED)');
        }
        if (msg.firmware) {
          state.fw = msg.firmware;
          setText('dashFwChip', msg.firmware === 'px4' ? 'PX4' : 'ArduPilot');
        }
        break;
      }
      case 'attitude': {
        const roll = msg.roll * 180 / Math.PI;
        const pitch = msg.pitch * 180 / Math.PI;
        const yaw = msg.yaw * 180 / Math.PI;
        state.att = { roll, pitch, yaw };
        setText('attRoll', roll.toFixed(1));
        setText('attPitch', pitch.toFixed(1));
        setText('attYaw', ((Math.round(yaw) % 360) + 360) % 360);
        pushSample('roll', roll);
        pushSample('pitch', pitch);
        // 航向环绕处理: 展开到连续角度, 避免曲线在 ±180° 处跳变
        pushSample('yaw', unwrap(yaw, 'yaw'));
        pushSample('rollrate', msg.rollspeed * 180 / Math.PI);
        pushSample('pitchrate', msg.pitchspeed * 180 / Math.PI);
        pushSample('yawrate', msg.yawspeed * 180 / Math.PI);
        if (state.view === 'dashboard') drawAtt();
        break;
      }
      case 'global_position': {
        setMetric('mAlt', (msg.relativeAlt / 100).toFixed(1), 'm');
        if (state.gps.sats === null && isFinite(msg.lat)) {
          state.gps.fix = state.gps.fix || 3;
        }
        updateGpsChip();
        break;
      }
      case 'vfr_hud': {
        const gs = Number(msg.groundspeed);
        const climb = Number(msg.climb);
        setMetric('mGspeed', gs.toFixed(1), 'm/s');
        setMetric('mClimb', climb.toFixed(1), 'm/s');
        setMetric('mThrottle', Math.round(Number(msg.throttle)), '%');
        pushSample('gspeed', gs);
        pushSample('climb', climb);
        pushSample('throttle', Number(msg.throttle));
        break;
      }
      case 'sys_status': {
        if (msg.voltageBattery) {
          const v = msg.voltageBattery / 1000;
          setMetric('mVolt', v.toFixed(2), 'V');
          setText('dashVolt', '电池: ' + v.toFixed(1) + 'V');
          pushSample('volt', v);
        }
        if (msg.currentBattery !== undefined && msg.currentBattery !== null && msg.currentBattery >= 0) {
          setMetric('mCurr', (msg.currentBattery / 100).toFixed(2), 'A');
          pushSample('curr', msg.currentBattery / 100);
        }
        if (msg.batteryRemaining !== undefined && msg.batteryRemaining >= 0) {
          setMetric('mBatt', Math.round(msg.batteryRemaining), '%');
          metricClass('mBatt', msg.batteryRemaining <= 15 ? 'danger' : (msg.batteryRemaining <= 30 ? 'warn' : 'ok'));
        }
        if (msg.load !== undefined) {
          const load = msg.load / 10;
          setMetric('mLoad', load.toFixed(0), '%');
          setText('dashLoad', '负载: ' + load.toFixed(0) + '%');
          pushSample('load', load);
        }
        break;
      }
      case 'gps_status': {
        state.gps.sats = (msg.satellites === 255) ? null : msg.satellites;
        state.gps.fix = msg.fixType;
        updateGpsChip();
        break;
      }
      case 'esc_telemetry': {
        if (msg.rpm > 0) {
          setMetric('mRpm', Math.round(msg.rpm), 'rpm');
          metricClass('mRpm', 'accent');
        }
        break;
      }
      case 'statustext': {
        pushEvent(msg.severity || 6, msg.text);
        break;
      }
      case 'mav_stats': {
        state.rateData = { counts: msg.counts || {}, cum: msg.cum || {}, total: msg.total || 0, windowMs: msg.windowMs || 1000 };
        if (state.view === 'console') renderRates();
        break;
      }
      case 'mav_recent': {
        state.samples = (msg.list || []).slice().reverse();
        setText('cnSampleHint', state.samples.length + ' 条, 点击展开字段');
        if (state.view === 'console') renderSamples();
        break;
      }
      case 'param_value': {
        state.params.add(msg.paramId);
        if (state.params.size !== state.lastParamCount) {
          state.lastParamCount = state.params.size;
          setText('paramCount', state.params.size + ' 项');
        }
        break;
      }
      default:
        break;
    }
  }

  const unwraps = {};
  function unwrap(v, key) {
    const prev = unwraps[key];
    if (prev === undefined) { unwraps[key] = v; return v; }
    let d = v - prev;
    if (d > 180) { unwraps[key] = v - 360; }
    else if (d < -180) { unwraps[key] = v + 360; }
    return unwraps[key];
  }

  function updateGpsChip() {
    const fix = state.gps.fix;
    const label = fix === null || fix === undefined ? '--' : (FIX_LABEL[fix] || ('fix=' + fix));
    const sats = state.gps.sats === null || state.gps.sats === undefined ? '--' : state.gps.sats;
    setText('dashGps', 'GPS: ' + label);
    setText('gpsCount', sats + ' 星 · ' + label);
    setMetric('mSats', String(sats), '');
    setMetric('mFix', label, '');
    metricClass('mFix', fix >= 3 ? 'ok' : (fix ? 'warn' : ''));
  }

  function resetData() {
    for (const k of Object.keys(hist)) delete hist[k];
    unwraps.yaw = undefined;
    state.att = null;
    state.gps = { sats: null, fix: null };
    state.events = [];
    state.eventsDirty = true;
    state.rateData = null;
    state.samples = [];
    state.params = new Set();
    state.lastParamCount = -1;
    state.chartSamples = 0;
    state.arm = false;
    state.mode = '--';
    // 指标复位
    ['mAlt', 'mGspeed', 'mClimb', 'mThrottle', 'mVolt', 'mCurr', 'mBatt', 'mLoad', 'mSats', 'mRpm', 'mMsgRate'].forEach((id) => setMetric(id, '--', ''));
    setMetric('mFix', '--', '');
    ['mBatt', 'mRpm', 'mFix'].forEach((id) => metricClass(id, ''));
    setText('attRoll', '--');
    setText('attPitch', '--');
    setText('attYaw', '--');
    setText('dashArm', '上锁');
    const arm = $('dashArm'); if (arm) arm.className = 'pill';
    setText('dashMode', '--');
    setText('dashGps', 'GPS: --');
    setText('dashVolt', '电池: --');
    setText('dashLoad', '负载: --');
    setText('paramCount', '--');
    renderEvents();
    renderRates();
    renderSamples();
    setText('cnSampleHint', '点击展开字段');
    const d = $('dashEvents'); if (d) { d.dataset.h = ''; d.innerHTML = ''; }
  }

  // ==================== 视图切换 ====================
  function onView(name) {
    state.view = name;
    if (name === 'dashboard') drawDashboard();
    if (name === 'charts') { pruneHist(); renderCharts(); }
    if (name === 'console') {
      renderRates();
      renderSamples();
      renderEvents();
      startSamplePolling();
    } else {
      stopSamplePolling();
    }
  }

  // ==================== 控件 ====================
  function bindControls() {
    const pauseBtn = $('rtPause');
    if (pauseBtn) {
      pauseBtn.onclick = () => {
        state.chartsPaused = !state.chartsPaused;
        pauseBtn.textContent = state.chartsPaused ? '▶ 继续绘制' : '⏸ 暂停绘制';
      };
    }
    const clearBtn = $('rtClear');
    if (clearBtn) {
      clearBtn.onclick = () => {
        for (const k of Object.keys(hist)) delete hist[k];
        state.chartSamples = 0;
        renderCharts();
      };
    }
    const winSel = $('rtWindow');
    if (winSel) winSel.onchange = () => { pruneHist(); renderCharts(); };

    const cnPause = $('cnPause');
    if (cnPause) {
      cnPause.onclick = () => {
        state.consolePaused = !state.consolePaused;
        cnPause.textContent = state.consolePaused ? '继续刷新' : '暂停刷新';
        if (state.consolePaused) stopSamplePolling();
        else if (state.view === 'console') startSamplePolling();
      };
    }
    const cnClear = $('cnClear');
    if (cnClear) {
      cnClear.onclick = () => {
        state.events = [];
        state.eventsDirty = true;
        const d = $('dashEvents'); if (d) d.dataset.h = '';
        const c = $('cnTextList'); if (c) c.dataset.h = '';
        renderEvents();
      };
    }

    // 消息样本行: 点击展开完整字段
    const sampleBody = $('cnSampleBody');
    if (sampleBody) {
      sampleBody.addEventListener('click', (e) => {
        const tr = e.target.closest('tr.cn-sample-row');
        if (!tr) return;
        const next = tr.nextElementSibling;
        if (next && next.classList.contains('cn-sample-fields')) { next.remove(); return; }
        sampleBody.querySelectorAll('.cn-sample-fields').forEach((el) => el.remove());
        const m = state.samples[Number(tr.dataset.i)];
        if (!m) return;
        const td = document.createElement('td');
        td.colSpan = 3;
        td.className = 'cn-sample-fields';
        const lines = Object.entries(m.fields || {}).map(([k, v]) => {
          const val = Array.isArray(v) ? '[' + v.join(', ') + ']' : String(v);
          return `${k}: ${val}`;
        });
        td.textContent = lines.join('\n') || '(无字段)';
        const row = document.createElement('tr');
        row.className = 'cn-sample-fields-row';
        row.appendChild(td);
        tr.after(row);
      });
    }

    window.addEventListener('resize', () => {
      if (state.view === 'dashboard') drawAtt();
      if (state.view === 'charts') renderCharts();
    });
  }

  // ==================== 周期任务 ====================
  function tick() {
    if (state.eventsDirty && (state.view === 'dashboard' || state.view === 'console')) {
      state.eventsDirty = false;
      renderEvents();
    }
    if (state.view === 'charts') {
      if (!state.chartsPaused) { pruneHist(); renderCharts(); }
      setText('rtRate', state.sampleRate + ' 样本/秒');
    }
    if (state.view === 'console') renderRates();
  }

  function rateTick() {
    state.sampleRate = state.chartSamples;
    state.chartSamples = 0;
    setMetric('mMsgRate', state.rateData ? String(state.rateData.total) : '--', '/s');
  }

  // 主题切换: 清 CSS 变量缓存并立即重绘 (曲线图例/姿态仪按新主题取色)
  if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('fy:theme', () => {
      themeVars = {};
      try { drawAtt(); renderCharts(); } catch (e) { /* 视图未就绪忽略 */ }
    });
  }

  // ==================== 初始化 ====================
  function init() {
    state.events = [];
    bindControls();
    renderEvents();
    renderRates();
    renderSamples();
    drawAtt();
    setInterval(tick, 200);
    setInterval(rateTick, 1000);
    console.log('监视模块 (仪表盘/实时数据/消息台) 已初始化');
  }

  window.FcMonitor = {
    onMsg,
    onView,
    init,
  };

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
