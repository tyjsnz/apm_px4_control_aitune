// 飞行日志分析: 从 .bin 提取统计/关键指标/数据片段, 并生成 AI 报告提示词
import { parseBin } from './binlog.js';

const TRACK_TYPES = new Set([
  'ATT', 'IMU', 'VIBE', 'GPS', 'GPA', 'BARO', 'BAT', 'BAT2', 'CURR', 'POWR',
  'RCIN', 'RCOU', 'MODE', 'ERR', 'EV', 'NKF1', 'NKF2', 'NKF3', 'NKF4', 'NKF5',
  'XKF1', 'XKF2', 'XKF3', 'XKF4', 'XKF5', 'ESC', 'POS', 'MOTB', 'MAG', 'PM', 'OF',
]);

const EXCERPT_TYPES = ['ATT', 'GPS', 'VIBE', 'BAT', 'RCOU', 'MODE'];

const COPTER_MODES = [
  'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE',
  'LAND', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW',
  'AVOID_ADSB', 'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG',
  'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL',
];

const fmt = (v, d = 3) => {
  if (typeof v !== 'number' || !isFinite(v)) return String(v);
  if (Number.isInteger(v)) return String(v);
  return String(Number(v.toFixed(d)));
};
const modeName = (m) => COPTER_MODES[m] || ('MODE' + m);

export function analyzeLog(buf, opts = {}) {
  const excerptRows = opts.excerptRows || 30;
  const stats = {};
  const modes = [];
  const errs = [];
  const evs = [];
  const excerpts = {};
  const acc = {
    dur: null, firstUS: null, lastUS: null,
    attRollErr: 0, attPitchErr: 0, attYawErr: 0,
    maxVibe: 0, maxClip: 0, maxSpeed: 0, minSats: Infinity,
    voltMin: Infinity, voltMax: -Infinity, currMax: -Infinity, remMin: Infinity,
    maxRelAlt: null, homeAlt: null, maxRcout: 0, maxEscRpm: 0,
  };
  let total = 0;

  const track = (st, f) => {
    let s = st.fields[f];
    if (!s) { s = st.fields[f] = { min: Infinity, max: -Infinity, sum: 0, n: 0, first: null, last: null }; }
    return s;
  };
  const upd = (s, v) => {
    if (typeof v !== 'number' || !isFinite(v)) return;
    s.min = Math.min(s.min, v); s.max = Math.max(s.max, v);
    s.sum += v; s.n++;
    if (s.first === null) s.first = v;
    s.last = v;
  };

  parseBin(buf, (name, fields) => {
    total++;
    const us = fields.TimeUS;
    if (typeof us === 'number') {
      if (acc.firstUS === null || us < acc.firstUS) acc.firstUS = us;
      if (acc.lastUS === null || us > acc.lastUS) acc.lastUS = us;
    }

    let st = stats[name];
    if (!st) { st = stats[name] = { count: 0, firstUS: null, lastUS: null, fields: {} }; }
    st.count++;
    if (typeof us === 'number') {
      if (st.firstUS === null || us < st.firstUS) st.firstUS = us;
      if (st.lastUS === null || us > st.lastUS) st.lastUS = us;
    }

    if (TRACK_TYPES.has(name)) {
      let n = 0;
      for (const [k, v] of Object.entries(fields)) {
        if (n >= 40) break;
        if (typeof v === 'number') { upd(track(st, k), v); n++; }
      }
    }

    // 关键指标
    if (name === 'ATT') {
      if (typeof fields.Roll === 'number' && typeof fields.DesRoll === 'number') {
        acc.attRollErr = Math.max(acc.attRollErr, Math.abs(fields.Roll - fields.DesRoll));
      }
      if (typeof fields.Pitch === 'number' && typeof fields.DesPitch === 'number') {
        acc.attPitchErr = Math.max(acc.attPitchErr, Math.abs(fields.Pitch - fields.DesPitch));
      }
      if (typeof fields.Yaw === 'number' && typeof fields.DesYaw === 'number') {
        let d = Math.abs(fields.Yaw - fields.DesYaw);
        if (d > Math.PI) d = Math.abs(d - 2 * Math.PI);
        acc.attYawErr = Math.max(acc.attYawErr, d);
      }
    }
    if (name === 'VIBE') {
      for (const k of ['VibeX', 'VibeY', 'VibeZ']) {
        if (typeof fields[k] === 'number') acc.maxVibe = Math.max(acc.maxVibe, fields[k]);
      }
      for (const k of ['Clip0', 'Clip1', 'Clip2']) {
        if (typeof fields[k] === 'number') acc.maxClip = Math.max(acc.maxClip, fields[k]);
      }
    }
    if (name === 'GPS') {
      if (typeof fields.Spd === 'number') acc.maxSpeed = Math.max(acc.maxSpeed, fields.Spd);
      if (typeof fields.NSats === 'number') acc.minSats = Math.min(acc.minSats, fields.NSats);
    }
    if (name === 'BAT' || name === 'BAT2') {
      if (typeof fields.Volt === 'number') { acc.voltMin = Math.min(acc.voltMin, fields.Volt); acc.voltMax = Math.max(acc.voltMax, fields.Volt); }
      if (typeof fields.Curr === 'number') acc.currMax = Math.max(acc.currMax, fields.Curr);
      if (typeof fields.RemPct === 'number') acc.remMin = Math.min(acc.remMin, fields.RemPct);
    }
    if (name === 'POS') {
      const rel = (typeof fields.RelHomeAlt === 'number') ? fields.RelHomeAlt
        : (typeof fields.RelOriginAlt === 'number') ? fields.RelOriginAlt : null;
      if (rel !== null) acc.maxRelAlt = acc.maxRelAlt === null ? rel : Math.max(acc.maxRelAlt, rel);
    }
    if (name === 'RCOU') {
      for (const k of ['C1', 'C2', 'C3', 'C4']) {
        if (typeof fields[k] === 'number') acc.maxRcout = Math.max(acc.maxRcout, fields[k]);
      }
    }
    if (name === 'ESC') {
      for (const k of Object.keys(fields)) {
        if (/^RPM/i.test(k) && typeof fields[k] === 'number') acc.maxEscRpm = Math.max(acc.maxEscRpm, fields[k]);
      }
    }
    if (name === 'MODE' && typeof fields.Mode === 'number') {
      if (!modes.length || modes[modes.length - 1].mode !== fields.Mode) {
        modes.push({ us, mode: fields.Mode });
      }
    }
    if (name === 'ERR' && errs.length < 40) errs.push({ us, Subsys: fields.Subsys, ECode: fields.ECode });
    if (name === 'EV' && evs.length < 40) evs.push({ us, Id: fields.Id });
    if (EXCERPT_TYPES.includes(name)) {
      if (!excerpts[name]) excerpts[name] = [];
      if (excerpts[name].length < excerptRows) excerpts[name].push(fields);
    }
  });

  const dur = (acc.firstUS !== null && acc.lastUS !== null)
    ? (acc.lastUS - acc.firstUS) / 1e6 : 0;
  const t0 = acc.firstUS || 0;

  const L = [];
  L.push('# 飞行日志摘要');
  L.push(`- 消息记录条数: ${total}`);
  L.push(`- 日志时长: ${fmt(dur, 1)} s`);
  L.push(`- 消息类型数: ${Object.keys(stats).length}`);
  if (modes.length) {
    L.push(`- 模式切换(${modes.length}次): `
      + modes.slice(0, 30).map((m) => `${fmt((m.us - t0) / 1e6, 1)}s=${modeName(m.mode)}`).join(', '));
  }

  L.push('\n## 关键指标');
  if (acc.maxRelAlt !== null) L.push(`- 最大相对高度: ${fmt(acc.maxRelAlt, 2)} m`);
  if (acc.maxSpeed > 0) L.push(`- 最大地速: ${fmt(acc.maxSpeed, 2)} m/s`);
  if (acc.minSats !== Infinity) L.push(`- 最少卫星数: ${acc.minSats}`);
  if (acc.maxVibe > 0) L.push(`- 最大振动(VibeX/Y/Z): ${fmt(acc.maxVibe, 1)} m/s²  (clipping 峰值 ${fmt(acc.maxClip, 0)})`);
  if (acc.attRollErr || acc.attPitchErr) {
    L.push(`- 姿态跟踪最大误差: Roll ${fmt(acc.attRollErr, 1)}° / Pitch ${fmt(acc.attPitchErr, 1)}°`);
  }
  if (acc.voltMin !== Infinity) L.push(`- 电池电压: ${fmt(acc.voltMin, 2)} ~ ${fmt(acc.voltMax, 2)} V，最大电流 ${fmt(acc.currMax, 1)} A，最低剩余 ${acc.remMin === Infinity ? '?' : acc.remMin}%`);
  if (acc.maxRcout > 0) L.push(`- 电机输出(RCOU)峰值: ${fmt(acc.maxRcout, 0)}`);
  if (acc.maxEscRpm > 0) L.push(`- ESC 转速峰值: ${fmt(acc.maxEscRpm, 0)} RPM`);

  if (errs.length) {
    L.push(`\n## 错误记录 ERR (${errs.length} 条)`);
    for (const e of errs.slice(0, 20)) L.push(`- t=${fmt((e.us - t0) / 1e6, 1)}s Subsys=${e.Subsys} ECode=${e.ECode}`);
  }
  if (evs.length) {
    L.push(`\n## 事件 EV (${evs.length} 条)`);
    for (const e of evs.slice(0, 20)) L.push(`- t=${fmt((e.us - t0) / 1e6, 1)}s Id=${e.Id}`);
  }

  L.push('\n## 各消息类型字段统计 (min / max / avg / last)');
  const typeNames = Object.keys(stats).sort();
  for (const name of typeNames) {
    const st = stats[name];
    const fields = Object.keys(st.fields);
    if (!fields.length) { L.push(`- ${name}: ${st.count} 条`); continue; }
    const span = (st.firstUS !== null && st.lastUS !== null) ? ((st.lastUS - st.firstUS) / 1e6) : 0;
    const parts = fields.slice(0, 20).map((f) => {
      const s = st.fields[f];
      return `${f}[${fmt(s.min, 3)}/${fmt(s.max, 3)}/${fmt(s.sum / s.n, 3)}/${fmt(s.last, 3)}]`;
    });
    L.push(`- ${name} (${st.count}条, ${fmt(span, 1)}s): ${parts.join(' ')}`);
  }

  if (Object.keys(excerpts).length) {
    L.push('\n## 关键数据片段 (前若干行)');
    for (const name of Object.keys(excerpts)) {
      const rows = excerpts[name];
      if (!rows.length) continue;
      const cols = Object.keys(rows[0]);
      L.push(`\n### ${name}`);
      L.push(cols.join(','));
      for (const row of rows) {
        L.push(cols.map((c) => (typeof row[c] === 'number' ? fmt(row[c], 4) : row[c])).join(','));
      }
    }
  }

  const text = L.join('\n');
  const metrics = { total, duration: dur, messageTypes: typeNames.length, modes: modes.length, errors: errs.length };
  return { text, metrics };
}

// ==================== AI 报告提示词 ====================
export const LOG_SYSTEM_PROMPT = `你是资深 ArduPilot/PX4 多旋翼试飞与日志分析专家。
根据提供的飞行日志统计与数据片段，输出一份完整、专业、可执行的《飞行报告》。
要求:
1. 使用中文 Markdown，包含以下章节:
   ## 一、飞行概况  (时长、模式、起降、是否异常)
   ## 二、姿态与振动  (姿态跟踪误差、振动/削波、滤波是否合理)
   ## 三、动力与电池  (电压/电流/电机输出、是否过载或电压跌落)
   ## 四、定位与导航  (GPS/卫星/EKF、位置漂移、模式切换是否合理)
   ## 五、问题与风险  (按严重程度排序，指出证据)
   ## 六、调参建议  (给出具体参数名 + 当前值/建议值 + 理由，仅限 ArduPilot 参数)
   ## 七、下一步试飞建议  (安全、可执行的步骤)
2. 只依据给定数据推断，数据不足处明确写"数据不足，需补充 XX 日志"。
3. 不要编造未出现的数值。参数建议给出单位与取值范围。
4. 结尾附一行安全提示。`;

export function buildLogUserPrompt(fileName, summaryText) {
  return `日志文件: ${fileName}\n\n${summaryText}\n\n请基于以上数据生成完整飞行报告。`;
}

// ==================== 传感器时序曲线(供前端画图) ====================
const CHART_GROUPS = [
  { id: 'att', title: '姿态 (°)', msg: 'ATT', series: [['Roll', '°'], ['Pitch', '°'], ['Yaw', '°'], ['DesRoll', '°'], ['DesPitch', '°'], ['DesYaw', '°']] },
  { id: 'gyro', title: '陀螺仪 (rad/s)', msg: 'IMU', series: [['GyrX', 'rad/s'], ['GyrY', 'rad/s'], ['GyrZ', 'rad/s']] },
  { id: 'accel', title: '加速度计 (m/s²)', msg: 'IMU', series: [['AccX', 'm/s²'], ['AccY', 'm/s²'], ['AccZ', 'm/s²']] },
  { id: 'vibe', title: '振动 (m/s²)', msg: 'VIBE', series: [['VibeX', 'm/s²'], ['VibeY', 'm/s²'], ['VibeZ', 'm/s²']] },
  { id: 'clip', title: '振动削波 (次)', msg: 'VIBE', series: [['Clip0', '次'], ['Clip1', '次'], ['Clip2', '次']] },
  { id: 'gps', title: 'GPS', msg: 'GPS', series: [['Spd', 'm/s'], ['NSats', '颗'], ['HDop', ''], ['Status', '']] },
  { id: 'altbaro', title: '气压高度 (m)', msg: 'BARO', series: [['Alt', 'm']] },
  { id: 'altpos', title: '相对高度 (m)', msg: 'POS', series: [['RelHomeAlt', 'm'], ['RelOriginAlt', 'm']] },
  { id: 'battery', title: '电池', msg: 'BAT', series: [['Volt', 'V'], ['Curr', 'A']] },
  { id: 'batpct', title: '电量 (%)', msg: 'BAT', series: [['RemPct', '%']] },
  { id: 'rcou', title: '电机输出 RCOU (PWM)', msg: 'RCOU', series: [['C1', 'PWM'], ['C2', 'PWM'], ['C3', 'PWM'], ['C4', 'PWM']] },
  { id: 'rcin', title: '遥控输入 RCIN (PWM)', msg: 'RCIN', series: [['C1', 'PWM'], ['C2', 'PWM'], ['C3', 'PWM'], ['C4', 'PWM']] },
  { id: 'ekfpos', title: 'EKF 位置 (m)', msg: 'NKF1', series: [['PN', 'm'], ['PE', 'm'], ['PD', 'm']] },
  { id: 'ekfvel', title: 'EKF 速度 (m/s)', msg: 'NKF1', series: [['VN', 'm/s'], ['VE', 'm/s'], ['VD', 'm/s']] },
  { id: 'ekfpos2', title: 'EKF 位置 (m)', msg: 'XKF1', series: [['PN', 'm'], ['PE', 'm'], ['PD', 'm']] },
  { id: 'ekfvel2', title: 'EKF 速度 (m/s)', msg: 'XKF1', series: [['VN', 'm/s'], ['VE', 'm/s'], ['VD', 'm/s']] },
];

function downsample(series, maxPoints) {
  const n = series.t.length;
  if (n <= maxPoints) return series;
  const stride = Math.ceil(n / maxPoints);
  const t = []; const v = [];
  for (let i = 0; i < n; i += stride) { t.push(series.t[i]); v.push(series.v[i]); }
  return { name: series.name, unit: series.unit, t, v };
}

/**
 * 从 .bin 抽取各传感器时序, 返回 [{id,title,series:[{name,unit,t:[],v:[]}]}]
 * t 为相对日志起点的秒数
 */
export function extractCharts(buf, opts = {}) {
  const maxPoints = opts.maxPoints || 400;
  const byMsg = new Map();
  for (const g of CHART_GROUPS) {
    if (!byMsg.has(g.msg)) byMsg.set(g.msg, []);
    byMsg.get(g.msg).push(g);
  }
  const groups = new Map(); // id -> {id,title, series:Map}
  const esc = { rpm: new Map(), volt: { t: [], v: [] }, curr: { t: [], v: [] }, temp: { t: [], v: [] } };
  let t0 = null;

  const ensure = (g) => {
    let e = groups.get(g.id);
    if (!e) { e = { id: g.id, title: g.title, order: CHART_GROUPS.indexOf(g), series: new Map() }; groups.set(g.id, e); }
    return e;
  };
  const push = (e, name, unit, t, v) => {
    let s = e.series.get(name);
    if (!s) { s = { name, unit, t: [], v: [] }; e.series.set(name, s); }
    s.t.push(t); s.v.push(v);
  };

  parseBin(buf, (name, fields) => {
    const us = fields.TimeUS;
    if (typeof us !== 'number') return;
    const sec = us / 1e6;
    if (t0 === null || sec < t0) t0 = sec;

    const gs = byMsg.get(name);
    if (gs) {
      for (const g of gs) {
        const e = ensure(g);
        for (const [f, unit] of g.series) {
          if (typeof fields[f] === 'number') push(e, f, unit, sec, fields[f]);
        }
      }
    }
    if (name === 'ESC') {
      const inst = (typeof fields.Instance === 'number') ? fields.Instance : 0;
      if (typeof fields.RPM === 'number') {
        if (!esc.rpm.has(inst)) esc.rpm.set(inst, { t: [], v: [] });
        const s = esc.rpm.get(inst); s.t.push(sec); s.v.push(fields.RPM);
      }
      if (inst === 0) {
        if (typeof fields.Volt === 'number') { esc.volt.t.push(sec); esc.volt.v.push(fields.Volt); }
        if (typeof fields.Curr === 'number') { esc.curr.t.push(sec); esc.curr.v.push(fields.Curr); }
        if (typeof fields.Temp === 'number') { esc.temp.t.push(sec); esc.temp.v.push(fields.Temp); }
      }
    }
  });

  const out = [];
  for (const e of [...groups.values()].sort((a, b) => a.order - b.order)) {
    const series = [];
    for (const s of e.series.values()) {
      if (s.t.length < 2) continue;
      const ds = downsample(s, maxPoints);
      ds.t = ds.t.map((x) => Number((x - (t0 || 0)).toFixed(3)));
      series.push(ds);
    }
    if (series.length) out.push({ id: e.id, title: e.title, series });
  }

  // ESC(电机/电调)
  if (esc.rpm.size || esc.volt.t.length) {
    const series = [];
    for (const [inst, s] of [...esc.rpm.entries()].sort((a, b) => a[0] - b[0])) {
      if (s.t.length < 2) continue;
      const ds = downsample({ name: 'RPM' + inst, unit: 'RPM', t: s.t, v: s.v }, maxPoints);
      ds.t = ds.t.map((x) => Number((x - (t0 || 0)).toFixed(3)));
      series.push(ds);
    }
    for (const [key, unit, s] of [['Volt', 'V', esc.volt], ['Curr', 'A', esc.curr], ['Temp', '°C', esc.temp]]) {
      if (s.t.length < 2) continue;
      const ds = downsample({ name: key, unit, t: s.t, v: s.v }, maxPoints);
      ds.t = ds.t.map((x) => Number((x - (t0 || 0)).toFixed(3)));
      series.push(ds);
    }
    if (series.length) out.push({ id: 'esc', title: '电调 ESC (电机转速/电压/电流/温度)', series });
  }

  return out;
}
