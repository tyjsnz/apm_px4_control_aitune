// 使用统计 WS / HTTP 集成测试 (不依赖飞控/网络)
//   node test/stats_ws.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';
import WebSocket from 'ws';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const PORT = 3987;
const TOKEN = 'test-token-0123456789';

const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'statsws-'));
const ENV = {
  ...process.env,
  PORT: String(PORT),
  STATS_TOKEN: TOKEN,
  STATS_FILE: path.join(TMP, 'usage.json'),
  FAULT_DIR: path.join(TMP, 'faults'),
  AI_CONFIG_FILE: path.join(TMP, 'config', 'ai_config.json'),
  BACKUP_DIR: path.join(TMP, 'backups'),
  LOG_DIR: path.join(TMP, 'logs'),
};

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function httpGet(p) {
  return new Promise((resolve, reject) => {
    fetch(`http://127.0.0.1:${PORT}${p}`).then(async (r) => {
      const text = await r.text();
      resolve({ status: r.status, text });
    }).catch(reject);
  });
}

function connect() {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(`ws://127.0.0.1:${PORT}`);
    ws.on('open', () => resolve(ws));
    ws.on('error', reject);
  });
}

const srv = spawn(process.execPath, ['server.js'], { cwd: ROOT, env: ENV });

async function main() {
  try {
    await sleep(1800);

    // ---------- 1. 仪表盘 token 门禁 ----------
    let r = await httpGet('/api/stats');
    check('无 token 返回 404', r.status === 404, String(r.status));
    r = await httpGet('/api/stats?token=wrong-token-xxx');
    check('错误 token 返回 404', r.status === 404, String(r.status));
    r = await httpGet('/stats?token=wrong-token-xxx');
    check('/stats 错误 token 返回 404', r.status === 404);
    r = await httpGet(`/api/stats?token=${TOKEN}`);
    let s = r.status === 200 ? JSON.parse(r.text) : {};
    check('正确 token 返回 JSON 汇总', r.status === 200
      && s.today && s.last7 && s.last30 && s.total && Array.isArray(s.series) && s.series.length === 30,
    r.status + ' ' + r.text.slice(0, 80));
    r = await httpGet(`/stats?token=${TOKEN}`);
    check('正确 token 返回仪表盘 HTML', r.status === 200 && /扶摇 FuyaoTune · 使用统计/.test(r.text));

    // ---------- 2. WS 匿名事件 ----------
    const ws = await connect();
    const send = (o) => ws.send(JSON.stringify(o));

    send({ type: 'stats_event', event: 'page_view', anonId: 'int-user-0001' });
    send({ type: 'stats_event', event: 'page_view', anonId: 'int-user-0001' });   // 同日重复
    send({ type: 'stats_event', event: 'page_view', anonId: 'int-user-0002' });
    send({ type: 'stats_event', event: 'fc_connect', anonId: 'int-user-0001', fw: 'px4' });
    send({ type: 'stats_event', event: 'fc_connect', anonId: 'int-user-0001', fw: 'apm' });
    send({ type: 'stats_event', event: 'onetouch', anonId: 'int-user-0001' });
    send({ type: 'stats_event', event: 'evil_event', anonId: 'x' });               // 白名单外
    await sleep(600);

    r = await httpGet(`/api/stats?token=${TOKEN}`);
    s = JSON.parse(r.text);
    check('PV=3 (白名单外事件不计)', s.today.pv === 3, JSON.stringify(s.today));
    check('UV=2 (同 ID 当天去重)', s.today.uv === 2, JSON.stringify(s.today));
    check('固件分布 PX4=1 AP=1', s.today.fw.px4 === 1 && s.today.fw.apm === 1, JSON.stringify(s.today.fw));
    check('一键首飞计数=1', s.today.features.onetouch === 1, JSON.stringify(s.today.features));
    check('统计文件落盘', fs.existsSync(ENV.STATS_FILE));

    // ---------- 3. 服务端功能计数 (备份上传) ----------
    const bres = await fetch(`http://127.0.0.1:${PORT}/api/backups`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name: 'stats_ws_test.json', content: '{"params":{"A":1}}' }),
    });
    check('备份上传成功', bres.status === 200);
    await sleep(300);
    r = await httpGet(`/api/stats?token=${TOKEN}`);
    s = JSON.parse(r.text);
    check('服务端计数: 备份=1', s.today.features.backup === 1, JSON.stringify(s.today.features));

    // ---------- 4. 限流: 超过 60 条/分钟静默丢弃 ----------
    // 前面已发 7 条(含 1 条白名单外), 同一限流窗口内再发 80 条 → 至多再受理 53 条 page_view
    for (let i = 0; i < 80; i++) send({ type: 'stats_event', event: 'page_view', anonId: 'spammer-01' });
    await sleep(700);
    r = await httpGet(`/api/stats?token=${TOKEN}`);
    s = JSON.parse(r.text);
    check('限流生效 (PV 落在 56~63)', s.today.pv >= 56 && s.today.pv <= 63, 'pv=' + s.today.pv);

    ws.close();

    // ---------- 5. STATS_DISABLED 总开关 (独立实例) ----------
    srv.kill('SIGKILL');
    await sleep(400);
    const ENV2 = { ...ENV, PORT: '3986', STATS_DISABLED: '1' };
    const srv2 = spawn(process.execPath, ['server.js'], { cwd: ROOT, env: ENV2 });
    await sleep(1800);
    const r404 = await fetch('http://127.0.0.1:3986/api/stats?token=' + TOKEN);
    check('关闭后 /api/stats 仍 404', r404.status === 404, String(r404.status));
    const ws2 = await new Promise((resolve, reject) => {
      const w = new WebSocket('ws://127.0.0.1:3986');
      w.on('open', () => resolve(w));
      w.on('error', reject);
    });
    const pvBefore = (() => {
      const d = JSON.parse(fs.readFileSync(ENV.STATS_FILE, 'utf-8')).days || {};
      return Object.values(d).reduce((a, x) => a + (x.pv || 0), 0);
    })();
    ws2.send(JSON.stringify({ type: 'stats_event', event: 'page_view', anonId: 'u-off' }));
    await sleep(500);
    const pvAfter = (() => {
      const d = JSON.parse(fs.readFileSync(ENV.STATS_FILE, 'utf-8')).days || {};
      return Object.values(d).reduce((a, x) => a + (x.pv || 0), 0);
    })();
    check('关闭后事件不写入 (PV 不变)', pvAfter === pvBefore, `${pvBefore} -> ${pvAfter}`);
    ws2.close();
    srv2.kill('SIGKILL');
    await sleep(200);

    console.log(failures === 0 ? '\n使用统计 WS 集成测试通过' : `\n${failures} 项失败`);
  } catch (err) {
    console.error('测试异常:', err.message);
    failures++;
  } finally {
    try { srv.kill('SIGKILL'); } catch (e) { /* ignore */ }
    await sleep(200);
    try { fs.rmSync(TMP, { recursive: true, force: true }); } catch (e) { /* ignore */ }
    process.exit(failures === 0 ? 0 : 1);
  }
}

main();
