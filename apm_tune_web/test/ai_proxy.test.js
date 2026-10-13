// AI 代理集成测试: 本机 Key 端到端送达 + 服务端 Key 护栏 (防盗取)
//   场景 A: 无服务端默认 → 请求用客户端随消息携带的 Key/BaseURL (B 方案主链路)
//   场景 B: 有服务端默认 (env AI_*) → 忽略客户端 BaseURL, 服务端 Key 只发服务端地址
//   node test/ai_proxy.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import http from 'http';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';
import WebSocket from 'ws';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'aiproxy-'));

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/** 假 AI 服务端: 记录收到的认证头, 返回 OpenAI 风格响应 */
function mockServer() {
  const hits = [];
  const srv = http.createServer((req, res) => {
    req.on('data', () => {});
    req.on('end', () => {
      hits.push({ url: req.url, auth: req.headers.authorization || '' });
      res.setHeader('Content-Type', 'application/json');
      res.end(JSON.stringify({ choices: [{ message: { role: 'assistant', content: 'MOCK-OK' } }] }));
    });
  });
  return new Promise((resolve) => {
    srv.listen(0, '127.0.0.1', () => {
      resolve({ hits, url: `http://127.0.0.1:${srv.address().port}`, close: () => srv.close() });
    });
  });
}

function startServer(port, env) {
  const child = spawn(process.execPath, ['server.js'], {
    cwd: ROOT,
    env: {
      ...process.env,
      PORT: String(port),
      BACKUP_DIR: path.join(TMP, 'backups'),
      LOG_DIR: path.join(TMP, 'logs'),
      FAULT_DIR: path.join(TMP, 'faults'),
      STATS_DISABLED: '1',
      ...env,
    },
  });
  return new Promise((resolve, reject) => {
    let buf = '';
    const timer = setTimeout(() => reject(new Error('服务启动超时')), 8000);
    child.stdout.on('data', (d) => {
      buf += d.toString();
      if (buf.includes('扶摇 FuyaoTune 已启动')) { clearTimeout(timer); resolve(child); }
    });
    child.stderr.on('data', (d) => { if (process.env.E2E_VERBOSE) process.stderr.write('[srv] ' + d); });
    child.on('error', reject);
  });
}

function connect(port) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(`ws://127.0.0.1:${port}`);
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

async function run() {
  const mockA = await mockServer();
  const mockB = await mockServer();
  const decoy = await mockServer();
  let srvA; let srvB; let ws;
  try {
    // ---------- 场景 A: 无服务端默认, 用客户端自带 Key ----------
    srvA = await startServer(3986, {});
    ws = await connect(3986);
    const r1 = await new Promise((resolve) => {
      ws.on('message', (d) => { const m = JSON.parse(d.toString()); if (m.type === 'fault_analyze_done') resolve(m); });
      ws.send(JSON.stringify({
        provider: 'custom', model: 'mock-model', baseUrl: mockA.url, apiKey: 'sk-client',
        type: 'fault_analyze',
        record: { title: '代理链路测试', symptom: 'x' },
      }));
    });
    check('场景A 分析成功 (客户端配置端到端生效)', r1.ok === true && r1.report === 'MOCK-OK', JSON.stringify(r1).slice(0, 200));
    check('场景A 服务端用客户端 Key 请求', mockA.hits.length > 0 && mockA.hits[0].auth === 'Bearer sk-client',
      JSON.stringify(mockA.hits[0] || null));
    check('场景A 不落盘任何 AI 配置', !fs.existsSync(path.join(ROOT, 'config', 'ai_config.json')));
    ws.close();

    // ---------- 场景 B: 服务端有默认, 客户端试图改道 (不带 Key) ----------
    srvB = await startServer(3985, {
      AI_PROVIDER: 'openai',
      AI_MODEL: 'mock-model',
      AI_BASE_URL: mockB.url,
      AI_API_KEY: 'sk-server-default',
    });
    ws = await connect(3985);
    const r2 = await new Promise((resolve) => {
      ws.on('message', (d) => { const m = JSON.parse(d.toString()); if (m.type === 'fault_analyze_done') resolve(m); });
      ws.send(JSON.stringify({
        provider: 'custom', model: 'x', baseUrl: decoy.url,   // 攻击面: 想把服务端 Key 送到自己地址
        type: 'fault_analyze',
        record: { title: '护栏测试', symptom: 'y' },
      }));
    });
    check('场景B 分析成功 (服务端默认兜底)', r2.ok === true && r2.report === 'MOCK-OK', JSON.stringify(r2).slice(0, 200));
    check('场景B 服务端 Key 发到服务端指定地址', mockB.hits.length > 0 && mockB.hits[0].auth === 'Bearer sk-server-default',
      JSON.stringify(mockB.hits[0] || null));
    check('场景B 护栏: 攻击者地址零请求', decoy.hits.length === 0, JSON.stringify(decoy.hits));
    ws.close();
  } catch (err) {
    console.error('测试异常:', err.message);
    failures++;
  } finally {
    try { if (ws) ws.close(); } catch (e) { /* ignore */ }
    try { if (srvA) srvA.kill('SIGKILL'); } catch (e) { /* ignore */ }
    try { if (srvB) srvB.kill('SIGKILL'); } catch (e) { /* ignore */ }
    mockA.close(); mockB.close(); decoy.close();
    await sleep(200);
    try { fs.rmSync(TMP, { recursive: true, force: true }); } catch (e) { /* ignore */ }
    console.log(failures === 0 ? '\nAI 代理集成测试通过' : `\n${failures} 项失败`);
    process.exit(failures === 0 ? 0 : 1);
  }
}

run();
