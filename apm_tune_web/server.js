// APM/PX4 调参 Web 后端 (服务器部署版)
// 职责边界 (Web Serial 架构):
//   - 飞控串口链路在浏览器端 (Web Serial) 直连, MAVLink/参数/日志操作由浏览器完成
//   - 本服务端只负责: 静态页面 / AI 代理(日志·故障分析, Key 留在服务端) / 故障记录 / 备份·日志文件存储
// 客户端消息协议与旧版保持一致; 串口类消息 (connect/set_param/ot_*/log_* 等) 已迁移至浏览器端。
import express from 'express';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import fs from 'fs';
import path from 'path';
import { exec } from 'child_process';
import { fileURLToPath } from 'url';

import { analyzeLog, buildLogUserPrompt, LOG_SYSTEM_PROMPT, extractCharts } from './lib/loganalysis.js';
import { FAULT_SYSTEM_PROMPT, buildFaultUserPrompt } from './lib/faultai.js';
import { createFaultStore } from './lib/faultstore.js';
import { createAiConfigStore } from './lib/aiconfigstore.js';
import { aiChat, providerList, providerInfo } from './lib/ai.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
// 数据目录 (可用环境变量覆盖, 便于测试隔离)
const BACKUP_DIR = process.env.BACKUP_DIR || path.join(__dirname, 'backups');
const LOG_DIR = process.env.LOG_DIR || path.join(__dirname, 'logs');
const FAULT_DIR = process.env.FAULT_DIR || path.join(__dirname, 'faults');
const AI_CONFIG_FILE = process.env.AI_CONFIG_FILE || path.join(__dirname, 'config', 'ai_config.json');

const PORT = Number(process.env.PORT) || 3000;
const HOST = process.env.HOST || '0.0.0.0';

fs.mkdirSync(BACKUP_DIR, { recursive: true });
fs.mkdirSync(LOG_DIR, { recursive: true });
fs.mkdirSync(FAULT_DIR, { recursive: true });

const app = express();
const server = createServer(app);

// 可选 HTTP Basic 认证: 设置环境变量 AUTH_USER / AUTH_PASS 即启用(公网部署强烈建议)
const AUTH_USER = process.env.AUTH_USER || '';
const AUTH_PASS = process.env.AUTH_PASS || '';
const AUTH_ON = !!(AUTH_USER || AUTH_PASS);

function checkAuth(req) {
  if (!AUTH_ON) return true;
  const h = req.headers.authorization || '';
  if (!h.startsWith('Basic ')) return false;
  let dec = '';
  try { dec = Buffer.from(h.slice(6), 'base64').toString('utf8'); } catch (e) { return false; }
  const idx = dec.indexOf(':');
  if (idx < 0) return false;
  return dec.slice(0, idx) === AUTH_USER && dec.slice(idx + 1) === AUTH_PASS;
}

app.use((req, res, next) => {
  if (checkAuth(req)) return next();
  res.set('WWW-Authenticate', 'Basic realm="apm-tune"');
  return res.status(401).send('需要认证');
});

app.use(express.static(path.join(__dirname, 'public')));
// 协议层模块以 ES Module 形式提供给浏览器 (lib/fcops.js + lib/paramlink.js + lib/mavlink.js)
app.use('/lib', express.static(path.join(__dirname, 'lib')));

// WebSocket: 同样做认证(HTTP 中间件不覆盖 upgrade 请求)
const wss = new WebSocketServer({ noServer: true });
server.on('upgrade', (req, socket, head) => {
  if (!checkAuth(req)) {
    socket.write('HTTP/1.1 401 Unauthorized\r\n'
      + 'WWW-Authenticate: Basic realm="apm-tune"\r\nConnection: close\r\n\r\n');
    socket.destroy();
    return;
  }
  wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws, req));
});

// ==================== 本地文件下载 ====================
function serveFile(res, dir, name, contentType) {
  const safe = path.basename(String(name || ''));
  const full = path.join(dir, safe);
  if (!safe || !full.startsWith(dir) || !fs.existsSync(full)) {
    res.status(404).send('文件不存在');
    return;
  }
  res.setHeader('Content-Type', contentType || 'application/octet-stream');
  res.setHeader('Content-Disposition', `attachment; filename="${safe}"`);
  fs.createReadStream(full).pipe(res);
}

app.get('/logs/:name', (req, res) => serveFile(res, LOG_DIR, req.params.name, 'application/octet-stream'));
app.get('/backups/:name', (req, res) => serveFile(res, BACKUP_DIR, req.params.name, 'application/json'));
app.get('/faults/:name', (req, res) => serveFile(res, FAULT_DIR, req.params.name, 'application/json'));

// ==================== 浏览器端上传 API (Web Serial 架构) ====================
const safeName = (name, re) => {
  const raw = String(name || '');
  const s = path.basename(raw);
  // 含路径分隔符 / .. 的一律拒绝 (不只依赖 basename 洗涤)
  if (!s || s !== raw) return null;
  return re.test(s) ? s : null;
};

// 保存参数备份 (浏览器端 FcOps.store.saveText → POST /api/backups)
app.post('/api/backups', express.json({ limit: '20mb' }), (req, res) => {
  const body = req.body || {};
  const file = safeName(body.name, /^[\w.-]+\.json$/);
  if (!file) return res.status(400).json({ error: '非法备份文件名' });
  if (typeof body.content !== 'string' || !body.content) {
    return res.status(400).json({ error: '缺少备份内容' });
  }
  fs.writeFileSync(path.join(BACKUP_DIR, file), body.content, 'utf-8');
  res.json({ ok: true, file });
});

// 读取参数备份 JSON (浏览器端恢复备份用)
app.get('/api/backups/:name', (req, res) => {
  const file = safeName(req.params.name, /^[\w.-]+\.json$/);
  const full = file && path.join(BACKUP_DIR, file);
  if (!full || !fs.existsSync(full)) return res.status(404).json({ error: '备份文件不存在' });
  res.setHeader('Content-Type', 'application/json');
  fs.createReadStream(full).pipe(res);
});

// 保存下载的飞行日志 (浏览器端 FcOps.store.saveBin → POST /api/logs?name=xxx.bin)
app.post('/api/logs', express.raw({ type: 'application/octet-stream', limit: '500mb' }), (req, res) => {
  const file = safeName(req.query.name, /^[\w.-]+\.bin$/);
  if (!file) return res.status(400).json({ error: '非法日志文件名' });
  const data = req.body;
  if (!data || !data.length) return res.status(400).json({ error: '空日志数据' });
  fs.writeFileSync(path.join(LOG_DIR, file), data);
  res.json({ ok: true, file, size: data.length });
});

// ==================== WebSocket ====================
const clients = new Set();
const broadcast = (obj) => {
  const msg = JSON.stringify(obj);
  for (const c of clients) {
    if (c.readyState === 1) {
      try { c.send(msg); } catch (e) { /* ignore */ }
    }
  }
};
const send = (ws, obj) => {
  if (ws && ws.readyState === 1) {
    try { ws.send(JSON.stringify(obj)); } catch (e) { /* ignore */ }
  }
};

// 已迁移至浏览器端的串口类消息 (给出明确提示而不是"未知消息类型")
const MIGRATED_TYPES = [
  'list_ports', 'connect', 'disconnect', 'mavlink_send',
  'request_params', 'set_param', 'reboot', 'send_heartbeat',
  'ot_read', 'ot_write', 'ot_cal', 'ot_accel_cal_start',
  'ot_accel_cal_confirm', 'ot_accel_cal_abort', 'ot_spin_start',
  'backup_restore', 'param_backup', 'param_restore',
  'log_list', 'log_download', 'log_erase',
];

wss.on('connection', (ws) => {
  clients.add(ws);
  console.log('WebSocket 客户端已连接');
  // 连接时推送当前状态
  send(ws, { type: 'heartbeat', flightMode: '--', armed: false });
  send(ws, { type: 'ai_config', config: aiConfigStore.getPublic() });

  ws.on('close', () => {
    clients.delete(ws);
    console.log('WebSocket 客户端已断开');
  });

  ws.on('message', async (raw) => {
    let msg;
    try { msg = JSON.parse(raw.toString()); } catch (e) {
      send(ws, { type: 'error', message: '消息格式错误' });
      return;
    }
    try {
      await handleClientMessage(ws, msg);
    } catch (err) {
      console.error('处理消息出错:', err);
      send(ws, { type: 'error', message: err.message });
    }
  });
});

// ==================== 日志 AI 分析 ====================
async function logAnalyze(ws, msg) {
  let files = [];
  if (Array.isArray(msg.files) && msg.files.length) files = msg.files;
  else if (msg.file) files = [msg.file];
  if (!files.length) { send(ws, { type: 'log_analyze_done', ok: false, error: '未指定日志' }); return; }

  const summaries = [];
  const metricsList = [];
  const usedFiles = [];
  const missing = [];
  for (const f of files) {
    const safe = path.basename(String(f || ''));
    const full = path.join(LOG_DIR, safe);
    if (!safe || !full.startsWith(LOG_DIR) || !fs.existsSync(full)) { missing.push(safe || String(f)); continue; }
    try {
      const buf = fs.readFileSync(full);
      const s = analyzeLog(buf, { excerptRows: 30 });
      summaries.push(`########## 日志 ${safe} ##########\n${s.text}`);
      metricsList.push({ file: safe, metrics: s.metrics });
      usedFiles.push(safe);
      // 传感器时序曲线
      try {
        const charts = extractCharts(buf, { maxPoints: 400 });
        send(ws, { type: 'log_charts', file: safe, charts });
      } catch (e) {
        send(ws, { type: 'log_charts', file: safe, charts: [], error: e.message });
      }
    } catch (err) {
      send(ws, { type: 'log_analyze_error', file: safe, error: '解析失败: ' + err.message });
    }
  }
  if (!summaries.length) {
    send(ws, { type: 'log_analyze_done', ok: false, error: '无可分析日志' + (missing.length ? ' (不存在: ' + missing.join(', ') + ')' : '') });
    return;
  }
  let combined = summaries.join('\n\n');
  const LIMIT = 150000;
  if (combined.length > LIMIT) combined = combined.slice(0, LIMIT) + '\n... (内容过长已截断)';
  const title = usedFiles.length === 1 ? usedFiles[0] : `${usedFiles.length} 个日志`;
  send(ws, {
    type: 'log_analyze_summary', file: title, files: usedFiles,
    summary: combined, metrics: metricsList,
  });

  // 使用全局 AI 配置 (请求中显式指定的字段可覆盖)
  const ai = aiConfigStore.resolve(msg);
  if (!aiReady(ai)) {
    // 未配置可用的 AI -> 仅返回数据摘要
    send(ws, { type: 'log_analyze_done', ok: true, ai: false, file: title, files: usedFiles, report: combined });
    return;
  }
  send(ws, { type: 'log_analyze_progress', stage: 'ai', file: title, files: usedFiles });
  const r = await aiChat({
    provider: ai.provider,
    apiKey: ai.apiKey,
    model: ai.model,
    baseUrl: ai.baseUrl,
    system: LOG_SYSTEM_PROMPT,
    user: buildLogUserPrompt(title, combined),
    timeoutMs: 180000,
  });
  if (r.error) {
    send(ws, { type: 'log_analyze_done', ok: false, ai: true, file: title, files: usedFiles, error: r.error, report: combined });
    return;
  }
  send(ws, { type: 'log_analyze_done', ok: true, ai: true, file: title, files: usedFiles, report: r.text });
}

function listBackups() {
  if (!fs.existsSync(BACKUP_DIR)) return [];
  const files = fs.readdirSync(BACKUP_DIR).filter(
    (f) => (f.startsWith('backup_') || f.startsWith('param_')) && f.endsWith('.json'));
  const out = [];
  for (const f of files) {
    try {
      const data = JSON.parse(fs.readFileSync(path.join(BACKUP_DIR, f), 'utf-8'));
      out.push({
        file: f,
        timestamp: data.timestamp || '',
        fcType: data.fc_type || '',
        count: Object.keys(data.params || {}).length,
        type: f.startsWith('param_') ? '全量参数' : '一键首飞',
      });
    } catch (e) { /* skip bad file */ }
  }
  out.sort((a, b) => (a.timestamp < b.timestamp ? 1 : -1));
  return out;
}

function listLocalLogs() {
  if (!fs.existsSync(LOG_DIR)) return [];
  const out = [];
  for (const f of fs.readdirSync(LOG_DIR)) {
    if (!f.endsWith('.bin')) continue;
    const st = fs.statSync(path.join(LOG_DIR, f));
    out.push({ file: f, size: st.size, mtime: st.mtimeMs });
  }
  out.sort((a, b) => b.mtime - a.mtime);
  return out;
}

// ==================== 故障速查: 记录存储 ====================
const faultStore = createFaultStore(FAULT_DIR);

// ==================== 全局 AI 配置 ====================
const aiConfigStore = createAiConfigStore(AI_CONFIG_FILE);

/** 全局配置是否可用于发起 AI 请求(可免 Key 的服务商除外) */
function aiReady(cfg) {
  if (!cfg || !cfg.provider || !cfg.model) return false;
  const info = providerInfo(cfg.provider);
  return !!cfg.apiKey || !!(info && info.keyOptional);
}

async function faultAnalyze(ws, msg) {
  const rec = msg.record || {};
  if (!String(rec.title || '').trim() && !String(rec.symptom || '').trim()) {
    send(ws, { type: 'fault_analyze_done', ok: false, error: '请先填写故障标题或现象描述' });
    return;
  }
  const ai = aiConfigStore.resolve(msg);
  if (!aiReady(ai)) {
    send(ws, { type: 'fault_analyze_done', ok: false, error: '尚未配置可用的全局 AI，请先在「AI 设置」中完成配置' });
    return;
  }
  send(ws, { type: 'fault_analyze_start' });
  const r = await aiChat({
    provider: ai.provider,
    apiKey: ai.apiKey,
    model: ai.model,
    baseUrl: ai.baseUrl,
    system: FAULT_SYSTEM_PROMPT,
    user: buildFaultUserPrompt(rec),
    timeoutMs: 180000,
  });
  if (r.error) {
    send(ws, { type: 'fault_analyze_done', ok: false, error: r.error });
    return;
  }
  send(ws, { type: 'fault_analyze_done', ok: true, report: r.text, at: new Date().toISOString() });
}

async function handleClientMessage(ws, msg) {
  switch (msg.type) {
    // ---------- 日志本地管理 (文件存于服务器) ----------
    case 'log_local_list': {
      send(ws, { type: 'log_local_list', logs: listLocalLogs() });
      return;
    }
    case 'log_local_delete': {
      const safe = path.basename(String(msg.file || ''));
      const full = path.join(LOG_DIR, safe);
      if (safe && full.startsWith(LOG_DIR) && fs.existsSync(full)) fs.unlinkSync(full);
      send(ws, { type: 'log_local_list', logs: listLocalLogs() });
      return;
    }
    case 'log_analyze': {
      await logAnalyze(ws, msg);
      return;
    }
    // ---------- 备份列表管理 (文件存于服务器) ----------
    case 'backup_list': {
      send(ws, { type: 'backup_list', backups: listBackups() });
      return;
    }
    case 'backup_delete': {
      const safe = path.basename(String(msg.file || ''));
      const full = path.join(BACKUP_DIR, safe);
      if (safe && full.startsWith(BACKUP_DIR) && fs.existsSync(full)) fs.unlinkSync(full);
      send(ws, { type: 'backup_list', backups: listBackups() });
      return;
    }
    // ---------- AI 配置 ----------
    case 'ai_providers': {
      send(ws, { type: 'ai_providers', providers: providerList() });
      return;
    }
    case 'ai_config_get': {
      send(ws, { type: 'ai_config', config: aiConfigStore.getPublic() });
      return;
    }
    case 'ai_config_set': {
      const cfg = Object.assign({}, msg.config || {});
      if (typeof msg.apiKey === 'string' && msg.apiKey.trim()) cfg.apiKey = msg.apiKey;
      const pub = aiConfigStore.set(cfg);
      broadcast({ type: 'ai_config', config: pub });  // 广播给所有标签页/客户端
      return;
    }
    case 'ai_config_clear_key': {
      const pub = aiConfigStore.clearKey();
      broadcast({ type: 'ai_config', config: pub });
      return;
    }
    // ---------- 故障速查 ----------
    case 'fault_list': {
      send(ws, { type: 'fault_list', faults: faultStore.list() });
      return;
    }
    case 'fault_save': {
      const r = faultStore.save(msg.record || {});
      send(ws, { type: 'fault_saved', file: r.file, id: r.id, updatedAt: r.updatedAt });
      send(ws, { type: 'fault_list', faults: faultStore.list() });
      return;
    }
    case 'fault_delete': {
      const ok = faultStore.remove(msg.file);
      send(ws, { type: 'fault_deleted', file: msg.file, ok });
      send(ws, { type: 'fault_list', faults: faultStore.list() });
      return;
    }
    case 'fault_analyze': {
      await faultAnalyze(ws, msg);
      return;
    }
    default:
      if (MIGRATED_TYPES.includes(msg.type)) {
        send(ws, {
          type: 'error',
          message: `消息 ${msg.type} 已迁移至浏览器端(Web Serial 直连飞控), 请刷新页面使用新版前端`,
        });
        return;
      }
      send(ws, { type: 'error', message: '未知消息类型: ' + msg.type });
  }
}

// ==================== 启动 ====================
server.listen(PORT, HOST, () => {
  console.log(`AP Tune 已启动: http://localhost:${PORT} (监听 ${HOST})`);
  console.log('串口连接由浏览器 Web Serial 直连 (需 Chrome/Edge + HTTPS 或 localhost)');
  if (AUTH_ON) console.log('已启用 HTTP Basic 认证');
  else console.log('⚠ 未启用认证 (公网部署请设置 AUTH_USER / AUTH_PASS)');
  const wantOpen = process.argv.includes('--open') || (process.pkg && !process.argv.includes('--no-open'));
  if (wantOpen) openBrowser(`http://localhost:${PORT}`);
});

function openBrowser(url) {
  const platform = process.platform;
  let cmd;
  if (platform === 'darwin') cmd = `open "${url}"`;
  else if (platform === 'win32') cmd = `cmd /c start "" "${url}"`;
  else cmd = `xdg-open "${url}"`;
  exec(cmd, (err) => { if (err) console.log('请手动打开浏览器: ' + url); });
}
