// 使用统计存储 (匿名): 按天分桶的 JSON 文件, 无数据库依赖
// 隐私约定:
//   - 客户端只上报随机匿名 ID (localStorage UUID), 不含 IP/账号/飞控数据
//   - UV 用 加盐SHA256(匿名ID) 当天去重统计, 无法反推原始 ID
// 独立于 server.js 以便单元测试
import fs from 'fs';
import path from 'path';
import crypto from 'crypto';

// 客户端可上报的事件 (其余一律丢弃)
export const STATS_EVENTS = ['page_view', 'fc_connect', 'onetouch'];
// 服务端内部计数的功能 (AI 分析/备份等在服务端处理, 无需经过客户端)
export const STATS_FEATURES = ['onetouch', 'log_analyze', 'fault_ai', 'thrust_ai', 'backup'];

const EMPTY_FEATURES = () => {
  const o = {};
  for (const f of STATS_FEATURES) o[f] = 0;
  return o;
};

function emptyDay() {
  return { pv: 0, uv: [], connects: 0, fw: { apm: 0, px4: 0 }, features: EMPTY_FEATURES() };
}

/** 补齐/纠正从磁盘加载的天桶 (向前兼容结构变更) */
function normalizeDay(d) {
  const out = emptyDay();
  if (!d || typeof d !== 'object') return out;
  out.pv = Number.isFinite(d.pv) ? d.pv : 0;
  out.connects = Number.isFinite(d.connects) ? d.connects : 0;
  out.uv = Array.isArray(d.uv) ? d.uv.filter((s) => typeof s === 'string') : [];
  if (d.fw && typeof d.fw === 'object') {
    out.fw.apm = Number.isFinite(d.fw.apm) ? d.fw.apm : 0;
    out.fw.px4 = Number.isFinite(d.fw.px4) ? d.fw.px4 : 0;
  }
  if (d.features && typeof d.features === 'object') {
    for (const f of STATS_FEATURES) {
      out.features[f] = Number.isFinite(d.features[f]) ? d.features[f] : 0;
    }
  }
  return out;
}

export function dayKey(d = new Date()) {
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}`;
}

/**
 * @param {string} file  统计 JSON 文件路径 (父目录自动创建)
 * @param {{ now?: () => Date, keepDays?: number, maxUvPerDay?: number }} [opts]
 */
export function createStatsStore(file, opts = {}) {
  const now = opts.now || (() => new Date());
  const keepDays = opts.keepDays || 400;
  const maxUvPerDay = opts.maxUvPerDay || 20000;
  fs.mkdirSync(path.dirname(file), { recursive: true });

  let data = { version: 1, salt: '', days: {} };
  if (fs.existsSync(file)) {
    try {
      const raw = JSON.parse(fs.readFileSync(file, 'utf-8'));
      if (raw && typeof raw === 'object' && raw.days) {
        data = { version: 1, salt: String(raw.salt || ''), days: raw.days };
      }
    } catch (e) { /* 损坏文件按新建处理, 不阻塞服务启动 */ }
  }
  if (!data.salt) data.salt = crypto.randomBytes(16).toString('hex');

  function dayKeyNow() { return dayKey(now()); }

  function prune() {
    const keys = Object.keys(data.days);
    if (keys.length <= keepDays) return false;
    const cutoff = new Date(now());
    cutoff.setDate(cutoff.getDate() - (keepDays - 1));
    const cut = dayKey(cutoff);
    let changed = false;
    for (const k of keys) {
      if (k < cut) { delete data.days[k]; changed = true; }
    }
    return changed;
  }

  function save() {
    const tmp = file + '.tmp';
    fs.writeFileSync(tmp, JSON.stringify(data), 'utf-8');
    fs.renameSync(tmp, file);   // 原子替换, 避免写一半被读
  }

  function getDay(key) {
    if (!data.days[key]) data.days[key] = emptyDay();
    else data.days[key] = normalizeDay(data.days[key]);
    return data.days[key];
  }

  function hashAnon(anonId) {
    return crypto.createHash('sha256').update(data.salt + ':' + anonId).digest('hex');
  }

  /** 记录客户端事件: page_view / fc_connect / onetouch; 返回是否受理 */
  function record(event, meta = {}) {
    if (!STATS_EVENTS.includes(event)) return false;
    const key = dayKeyNow();
    const day = getDay(key);
    let changed = false;

    if (event === 'page_view') {
      day.pv++;
      const anonId = String(meta.anonId || '');
      if (/^[\w-]{8,64}$/.test(anonId)) {
        const h = hashAnon(anonId);
        if (!day.uv.includes(h) && day.uv.length < maxUvPerDay) {
          day.uv.push(h);
        }
      }
      changed = true;
    } else if (event === 'fc_connect') {
      day.connects++;
      if (meta.fw === 'apm' || meta.fw === 'px4') day.fw[meta.fw]++;
      changed = true;
    } else if (event === 'onetouch') {
      day.features.onetouch++;
      changed = true;
    }

    if (changed) {
      if (prune()) { /* 顺带清理过期天 */ }
      save();
    }
    return changed;
  }

  /** 服务端内部功能计数 */
  function feature(name) {
    if (!STATS_FEATURES.includes(name)) return false;
    const day = getDay(dayKeyNow());
    day.features[name]++;
    save();
    return true;
  }

  function dayStats(key) {
    const d = data.days[key] ? normalizeDay(data.days[key]) : emptyDay();
    return { pv: d.pv, uv: d.uv.length, connects: d.connects, fw: d.fw, features: d.features };
  }

  function windowStats(fromDate) {
    const from = dayKey(fromDate);
    let pv = 0;
    let connects = 0;
    const fw = { apm: 0, px4: 0 };
    const features = EMPTY_FEATURES();
    const uvSet = new Set();
    for (const k of Object.keys(data.days)) {
      if (k < from || k > dayKeyNow()) continue;
      const d = normalizeDay(data.days[k]);
      pv += d.pv;
      connects += d.connects;
      fw.apm += d.fw.apm;
      fw.px4 += d.fw.px4;
      for (const f of STATS_FEATURES) features[f] += d.features[f];
      for (const h of d.uv) uvSet.add(h);   // 窗口 UV = 跨天并集
    }
    return { pv, uv: uvSet.size, connects, fw, features };
  }

  function daysAgo(n) {
    const d = new Date(now());
    d.setDate(d.getDate() - n);
    return d;
  }

  /** 聚合: 今日 / 近7天 / 近30天 / 累计 + 近30天日序列 */
  function summary() {
    prune();
    const today = dayKeyNow();
    const series = [];
    for (let i = 29; i >= 0; i--) {
      const k = dayKey(daysAgo(i));
      const s = dayStats(k);
      series.push({ d: k, pv: s.pv, uv: s.uv, connects: s.connects });
    }
    return {
      generatedAt: now().toISOString(),
      today: Object.assign({ d: today }, dayStats(today)),
      last7: windowStats(daysAgo(6)),
      last30: windowStats(daysAgo(29)),
      total: Object.assign({ days: Object.keys(data.days).length }, windowStats(new Date(0))),
      series,
    };
  }

  return { file, record, feature, summary };
}

function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

/** 统计仪表盘 HTML (无外部资源, 直接由 /stats 路由输出) */
export function renderStatsHtml(s) {
  const card = (label, w) => `
    <div class="card"><div class="cl">${esc(label)}</div>
      <div class="cv">${w.pv} <span class="cs">PV</span> / ${w.uv} <span class="cs">UV</span></div>
      <div class="cx">连接 ${w.connects} · AP ${w.fw.apm} · PX4 ${w.fw.px4}</div></div>`;
  const featRow = (label, key) => `<tr><td>${esc(label)}</td>`
    + `<td>${s.today.features[key]}</td><td>${s.last7.features[key]}</td>`
    + `<td>${s.last30.features[key]}</td><td>${s.total.features[key]}</td></tr>`;
  const seriesRows = s.series.map((r) => `<tr><td>${esc(r.d)}</td><td>${r.pv}</td><td>${r.uv}</td><td>${r.connects}</td></tr>`).join('\n');

  return `<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>扶摇 FuyaoTune · 使用统计</title>
<style>
  body { background:#0f1419; color:#d7dde4; font:14px/1.6 -apple-system,"PingFang SC",sans-serif; margin:0; padding:24px; }
  h1 { font-size:18px; margin:0 0 4px; } .sub { color:#7a8794; font-size:12px; margin-bottom:20px; }
  .cards { display:flex; flex-wrap:wrap; gap:12px; margin-bottom:24px; }
  .card { background:#171d24; border:1px solid #2a3440; border-radius:10px; padding:14px 18px; min-width:200px; }
  .cl { color:#7a8794; font-size:12px; } .cv { font-size:26px; font-weight:700; color:#4da3ff; }
  .cs { font-size:12px; color:#7a8794; font-weight:400; } .cx { color:#9aa7b4; font-size:12px; margin-top:4px; }
  h2 { font-size:14px; margin:24px 0 8px; color:#9aa7b4; }
  table { border-collapse:collapse; width:100%; max-width:760px; background:#171d24; border:1px solid #2a3440; border-radius:8px; overflow:hidden; }
  th,td { padding:6px 12px; text-align:left; font-size:13px; border-bottom:1px solid #222b35; }
  th { background:#1c242d; color:#9aa7b4; font-weight:600; }
  tr:last-child td { border-bottom:none; }
  .note { color:#7a8794; font-size:12px; margin-top:24px; max-width:760px; }
</style></head><body>
<h1>扶摇 FuyaoTune · 使用统计</h1>
<div class="sub">生成于 ${esc(s.generatedAt)} · 数据保留 ${s.total.days} 天</div>

<div class="cards">
${card('今日', s.today)}
${card('近 7 天', s.last7)}
${card('近 30 天', s.last30)}
${card('累计', s.total)}
</div>

<h2>功能使用 (今日 / 近7天 / 近30天 / 累计)</h2>
<table><tr><th>功能</th><th>今日</th><th>近7天</th><th>近30天</th><th>累计</th></tr>
${featRow('一键首飞写入', 'onetouch')}
${featRow('日志分析', 'log_analyze')}
${featRow('故障速查 AI', 'fault_ai')}
${featRow('动力体检 AI', 'thrust_ai')}
${featRow('参数备份', 'backup')}
</table>

<h2>最近 30 天</h2>
<table><tr><th>日期</th><th>PV</th><th>UV</th><th>飞控连接</th></tr>
${seriesRows}
</table>

<div class="note">
  隐私说明: 仅统计匿名随机标识的访问与功能使用次数, 不收集 IP、账号或飞控数据;
  UV 为按天加盐哈希去重结果。关闭统计: 服务端设置 <code>STATS_DISABLED=1</code>;
  浏览器端可在控制台执行 <code>localStorage.setItem('fy_stats_off','1')</code> 后刷新。
  本页需 <code>/stats?token=你的 STATS_TOKEN</code> 访问, 未配置 token 时返回 404。
</div>
</body></html>`;
}
