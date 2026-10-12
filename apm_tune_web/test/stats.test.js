// 使用统计存储单测 (纯逻辑 + 落盘, 不依赖飞控/网络)
//   node test/stats.test.js
import fs from 'fs';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';
import { createStatsStore, dayKey, renderStatsHtml, STATS_EVENTS, STATS_FEATURES } from '../lib/stats.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'stats-'));
const FILE = path.join(TMP, 'usage.json');

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

// 可控时钟: 测试中可推进日期
let fakeNow = new Date(2026, 9, 12, 10, 0, 0);   // 2026-10-12 本地时间
const now = () => new Date(fakeNow);

// ---------- 1. 事件受理与白名单 ----------
{
  const st = createStatsStore(FILE, { now });
  check('白名单常量', STATS_EVENTS.length === 3 && STATS_FEATURES.length === 5);
  check('page_view 受理', st.record('page_view', { anonId: 'user-a-0001' }) === true);
  check('fc_connect 受理', st.record('fc_connect', { fw: 'apm' }) === true);
  check('onetouch 受理', st.record('onetouch') === true);
  check('未知事件拒绝', st.record('evil_event') === false);
  check('非法 fw 不计入固件分布', st.record('fc_connect', { fw: 'evil' }) === true);

  const s = st.summary();
  check('今日 PV=1', s.today.pv === 1, JSON.stringify(s.today));
  check('今日 UV=1', s.today.uv === 1);
  check('连接计数=2', s.today.connects === 2);
  check('一键首飞计数=1', s.today.features.onetouch === 1, JSON.stringify(s.today.features));
  check('固件分布 AP=1 PX4=0', s.today.fw.apm === 1 && s.today.fw.px4 === 0);
  check('非法 fw 不进 apm/px4', s.today.fw.apm + s.today.fw.px4 === 1);
}

// ---------- 2. UV 按天去重 / 跨天重新计 ----------
{
  const st = createStatsStore(FILE, { now });
  st.record('page_view', { anonId: 'user-a-0001' });   // 同日重复访问
  st.record('page_view', { anonId: 'user-b-0002' });
  st.record('page_view', { anonId: '' });               // 无匿名 ID: 只计 PV
  let s = st.summary();
  check('同日同 ID 去重 (PV=4 含前节1次, UV=2)', s.today.pv === 4 && s.today.uv === 2, JSON.stringify(s.today));

  fakeNow = new Date(2026, 9, 13, 9, 0, 0);            // 次日
  st.record('page_view', { anonId: 'user-a-0001' });
  s = st.summary();
  check('次日 PV 独立', s.today.d === '2026-10-13' && s.today.pv === 1 && s.today.uv === 1);
  check('近7天 UV 为跨天并集=2', s.last7.uv === 2, JSON.stringify(s.last7));
  check('近7天 PV=5', s.last7.pv === 5, JSON.stringify(s.last7));
}

// ---------- 3. 服务端功能计数 ----------
{
  const st = createStatsStore(FILE, { now });
  st.feature('log_analyze');
  st.feature('fault_ai');
  st.feature('thrust_ai');
  st.feature('backup');
  st.feature('backup');
  check('未知功能拒绝', st.feature('not_exist') === false);
  const s = st.summary();
  check('功能计数累加', s.today.features.log_analyze === 1 && s.today.features.fault_ai === 1
    && s.today.features.thrust_ai === 1 && s.today.features.backup === 2,
  JSON.stringify(s.today.features));
}

// ---------- 4. 持久化与损坏恢复 ----------
{
  const st = createStatsStore(FILE, { now });
  const raw = JSON.parse(fs.readFileSync(FILE, 'utf-8'));
  check('落盘结构含 salt/days', typeof raw.salt === 'string' && raw.salt.length >= 16 && !!raw.days);
  check('落盘数据可读', raw.days['2026-10-13'].pv === 1);

  const st2 = createStatsStore(FILE, { now });   // 重新加载
  check('重载后累计不丢', st2.summary().last7.pv === 5, JSON.stringify(st2.summary().last7));
  check('salt 稳定 (UV 可跨进程去重)', JSON.parse(fs.readFileSync(FILE, 'utf-8')).salt === raw.salt);

  const BROKEN = path.join(TMP, 'broken.json');
  fs.writeFileSync(BROKEN, '{oops', 'utf-8');
  const st3 = createStatsStore(BROKEN, { now });
  check('损坏文件按新建恢复', st3.record('page_view', { anonId: 'x-user-9999' }) === true);
  check('无 tmp 残留 (原子写)', !fs.existsSync(FILE + '.tmp'));
}

// ---------- 5. 过期天清理 ----------
{
  const FILE2 = path.join(TMP, 'prune.json');
  const st = createStatsStore(FILE2, { now, keepDays: 2 });
  fakeNow = new Date(2026, 9, 13, 10, 0, 0);
  st.record('page_view', { anonId: 'u1' });            // 10-13
  fakeNow = new Date(2026, 9, 14, 10, 0, 0);
  st.record('page_view', { anonId: 'u2' });            // 10-14
  fakeNow = new Date(2026, 9, 20, 10, 0, 0);           // 10-20
  st.record('page_view', { anonId: 'u3' });            // 触发清理: 只留 10-19..10-20
  const raw = JSON.parse(fs.readFileSync(FILE2, 'utf-8'));
  const keys = Object.keys(raw.days).sort();
  check('过期天被清理', keys.length === 1 && keys[0] === '2026-10-20', JSON.stringify(keys));
  check('总累计只含保留天', st.summary().total.pv === 1, String(st.summary().total.pv));
}

// ---------- 6. 窗口聚合与序列 ----------
{
  const FILE3 = path.join(TMP, 'win.json');
  fakeNow = new Date(2026, 9, 12, 10, 0, 0);
  const st = createStatsStore(FILE3, { now });
  // 10-12: pv=1; 10-10 (2天前): pv=1 → last7 含, 昨日数据不进 today
  st.record('page_view', { anonId: 'a' });
  fakeNow = new Date(2026, 9, 10, 8, 0, 0);
  st.record('page_view', { anonId: 'b' });
  fakeNow = new Date(2026, 9, 12, 10, 0, 0);

  const s = st.summary();
  check('序列 30 项且按日升序', s.series.length === 30
    && s.series[0].d === '2026-09-13' && s.series[29].d === '2026-10-12',
  s.series[0].d + '..' + s.series[29].d);
  check('序列含历史 PV', s.series.find((x) => x.d === '2026-10-10').pv === 1);
  check('today/last7/total 分层正确',
    s.today.pv === 1 && s.last7.pv === 2 && s.last30.pv === 2 && s.total.pv === 2
    && s.total.days === 2, JSON.stringify([s.today.pv, s.last7.pv, s.total]));
}

// ---------- 7. 仪表盘 HTML ----------
{
  const FILE4 = path.join(TMP, 'dash.json');
  const st = createStatsStore(FILE4, { now });
  st.record('page_view', { anonId: 'a' });
  st.record('fc_connect', { fw: 'px4' });
  st.feature('thrust_ai');
  const html = renderStatsHtml(st.summary());
  check('HTML 含标题与卡片', /扶摇 FuyaoTune · 使用统计/.test(html) && /今日/.test(html));
  check('HTML 含功能表与隐私说明', /一键首飞写入/.test(html) && /隐私说明/.test(html));
  check('HTML 含序列日期', /2026-10-12/.test(html));
}

fs.rmSync(TMP, { recursive: true, force: true });
console.log(failures === 0 ? '\n使用统计单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
