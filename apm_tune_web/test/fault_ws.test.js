// 故障速查 + AI 配置(本机 Key) WebSocket 集成测试 (不依赖飞控/PTY/pymavlink)
//   node test/fault_ws.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';
import WebSocket from 'ws';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const PORT = 3988;

// 隔离的数据目录(避免污染真实 faults/ 等)
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'faultws-'));
const FAULT_DIR = path.join(TMP, 'faults');
const ENV = {
  ...process.env,
  PORT: String(PORT),
  FAULT_DIR,
  BACKUP_DIR: path.join(TMP, 'backups'),
  LOG_DIR: path.join(TMP, 'logs'),
  STATS_DISABLED: '1',   // 不测统计: 关闭以免污染真实统计
};

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function connect() {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(`ws://127.0.0.1:${PORT}`);
    ws.__msgs = [];
    ws.on('message', (d) => ws.__msgs.push(JSON.parse(d.toString())));
    ws.on('open', () => resolve(ws));
    ws.on('error', reject);
  });
}

function waitFor(ws, pred, ms, label) {
  return new Promise((resolve, reject) => {
    const take = () => {
      const i = ws.__msgs.findIndex(pred);
      return i < 0 ? null : ws.__msgs.splice(i, 1)[0];
    };
    const hit = take();
    if (hit) { resolve(hit); return; }
    const t0 = Date.now();
    const iv = setInterval(() => {
      const h = take();
      if (h) { clearInterval(iv); resolve(h); }
      else if (Date.now() - t0 > ms) { clearInterval(iv); reject(new Error('等待 ' + label + ' 超时')); }
    }, 30);
  });
}

const srv = spawn(process.execPath, ['server.js'], { cwd: ROOT, env: ENV });
let ws;

async function main() {
  try {
    await sleep(1800);
    ws = await connect();
    const send = (o) => ws.send(JSON.stringify(o));

    // ---------- 故障记录 ----------
    send({ type: 'fault_list' });
    const l0 = await waitFor(ws, (m) => m.type === 'fault_list', 5000, 'fault_list');
    check('初始 fault_list 返回数组', Array.isArray(l0.faults));

    send({ type: 'fault_save', record: { title: 'WS 测试故障', symptom: '现象', severity: 'high', tags: 'a,b' } });
    const saved = await waitFor(ws, (m) => m.type === 'fault_saved', 5000, 'fault_saved');
    check('fault_saved 合法文件名', /^fault_[\w.-]+\.json$/.test(saved.file), saved.file);
    const l1 = await waitFor(ws, (m) => m.type === 'fault_list' && m.faults.length === l0.faults.length + 1, 5000, 'fault_list(+)');
    check('记录持久化且标签解析', !!l1.faults.find((f) => f.file === saved.file && f.tags.length === 2));
    check('记录文件落盘', fs.existsSync(path.join(FAULT_DIR, saved.file)));

    // 未配置 AI 时分析应被明确拒绝(不发起网络请求)
    send({ type: 'fault_analyze', record: {} });
    const an0 = await waitFor(ws, (m) => m.type === 'fault_analyze_done', 5000, 'fault_analyze_done(空)');
    check('空记录被拒绝', an0.ok === false && /标题|现象/.test(an0.error || ''), JSON.stringify(an0));
    send({ type: 'fault_analyze', record: { title: 'x' } });
    const an1 = await waitFor(ws, (m) => m.type === 'fault_analyze_done', 5000, 'fault_analyze_done(未配置)');
    check('未配置 AI 时拒绝分析并指引「AI 设置」', an1.ok === false && /尚未配置可用的 AI|AI 设置/.test(an1.error || ''), JSON.stringify(an1));
    check('服务端从未落盘 Key 文件', !fs.existsSync(path.join(ROOT, 'config', 'ai_config.json')));

    send({ type: 'fault_delete', file: saved.file });
    const del = await waitFor(ws, (m) => m.type === 'fault_deleted', 5000, 'fault_deleted');
    check('fault_deleted ok', del.ok === true);

    // ---------- AI 配置 (B 方案: Key 归属浏览器, 服务端只提供目录与 env 默认) ----------
    send({ type: 'ai_providers' });
    const ap = await waitFor(ws, (m) => m.type === 'ai_providers', 5000, 'ai_providers');
    check('ai_providers 非空', Array.isArray(ap.providers) && ap.providers.length > 3);

    send({ type: 'ai_config_get' });
    const c0 = await waitFor(ws, (m) => m.type === 'ai_config', 5000, 'ai_config(get)');
    check('初始 ai_config 为空且标记 server 来源', c0.config.provider === '' && c0.config.hasKey === false
      && c0.config.source === 'server', JSON.stringify(c0.config));
    check('ai_config 不下发明文 Key', !('apiKey' in c0.config));

    // 服务端不再接收 Key: 旧协议消息给出明确提示 (而非静默)
    send({ type: 'ai_config_set', config: { provider: 'deepseek', model: 'deepseek-flash', remember: true }, apiKey: 'sk-1' });
    const dep1 = await waitFor(ws, (m) => m.type === 'error', 5000, 'ai_config_set(deprecated)');
    check('ai_config_set 被拒绝并提示本机保存', /浏览器/.test(dep1.message || ''), JSON.stringify(dep1));
    check('服务端不落盘任何 AI 配置', !fs.existsSync(path.join(ROOT, 'config', 'ai_config.json')));

    send({ type: 'ai_config_clear_key' });
    const dep2 = await waitFor(ws, (m) => m.type === 'error', 5000, 'ai_config_clear_key(deprecated)');
    check('ai_config_clear_key 提示去「AI 设置」', /AI 设置/.test(dep2.message || ''), JSON.stringify(dep2));
    check('服务端仍不落盘', !fs.existsSync(path.join(ROOT, 'config', 'ai_config.json')));

    // 新连接拿到的只是"服务端默认"(不含 Key), 各用户配置互不共享
    const ws2 = await connect();
    const push = await waitFor(ws2, (m) => m.type === 'ai_config', 5000, 'ai_config(push)');
    check('新连接只拿到服务端默认 (用户 Key 不互通)', push.config.source === 'server'
      && push.config.hasKey === false, JSON.stringify(push.config));
    ws2.close();

    console.log(failures === 0 ? '\n故障速查 / AI 配置 WS 集成测试通过' : `\n${failures} 项失败`);
  } catch (err) {
    console.error('测试异常:', err.message);
    failures++;
  } finally {
    try { if (ws) ws.close(); } catch (e) { /* ignore */ }
    srv.kill('SIGKILL');
    await sleep(200);
    try { fs.rmSync(TMP, { recursive: true, force: true }); } catch (e) { /* ignore */ }
    process.exit(failures === 0 ? 0 : 1);
  }
}

main();
