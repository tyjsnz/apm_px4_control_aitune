// ============================================================
// 零依赖 HTTP 服务：静态文件 + 仿真 API
//   GET  /api/presets        部件库（机架/电机/电调/电池/桨/机翼）
//   POST /api/simulate       运行仿真，返回完整结果（body.tuning 可覆盖半经验系数/判据阈值）
//   GET  /api/tuning         可调参数表：定义 + 分组 + 默认值
//   GET  /api/demo?preset=x  预设整机快速演示
//   *    静态文件 public/
// ============================================================
import http from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';
import { simulate, PRESETS, normalize } from '../core/simulate.js';
import { tuningMeta } from '../core/tuning.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC = path.join(__dirname, '..', 'public');
const PORT = Number(process.env.PORT) || 8080;

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.ico': 'image/x-icon',
  '.woff2': 'font/woff2',
};

// 安全预设（演示用整机配置）
const DEMOS = {
  default: { type: 'multirotor', frameId: 'fpv5', cells: 4, capacity_mAh: 1300, cRating: 100, motorType: '2207', kv: 1750, propD: 127, propP: 109, propB: 3, target_kmh: 120 },
  longrange: { type: 'multirotor', frameId: 'longrange', cells: 4, capacity_mAh: 3000, motorType: '3110', kv: 950, propD: 178, propP: 102, target_kmh: 80 },
  heavy: { type: 'multirotor', frameId: 'x8heavy', rotorCount: 8, cells: 12, capacity_mAh: 10000, motorType: '8120', kv: 150, propD: 432, propP: 152, propB: 2, escType: 'hv', target_kmh: 60, payload_g: 5000 },
  whoop: { type: 'multirotor', frameId: 'whoop65', cells: 1, capacity_mAh: 300, motorType: '0804', kv: 5000, propD: 51, propP: 76, target_kmh: 80 },
  wing: { type: 'fixedwing', wingId: 'wing1000', cells: 4, capacity_mAh: 2200, motorType: '4114', kv: 650, propD: 203, propP: 152, propB: 2, target_kmh: 100 },
  glider: { type: 'fixedwing', wingId: 'glider2m', cells: 4, capacity_mAh: 5000, motorType: '4114', kv: 650, propD: 254, propP: 152, propB: 2, target_kmh: 80 },
  rocket8: { type: 'vtol', rocketId: 'rocket8', cells: 6, capacity_mAh: 3000, motorType: '28065', kv: 1300, propD: 203, propP: 114, propB: 3, target_kmh: 100 },
  rocket10: { type: 'vtol', rocketId: 'rocket10', cells: 6, capacity_mAh: 5000, motorType: '4114', kv: 650, propD: 254, propP: 114, propB: 3, target_kmh: 90 },
};

function json(res, code, obj) {
  const body = JSON.stringify(obj);
  res.writeHead(code, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' });
  res.end(body);
}

function readBody(req, limit = 512 * 1024) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', (c) => {
      size += c.length;
      if (size > limit) { reject(new Error('payload too large')); req.destroy(); return; }
      chunks.push(c);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });
}

async function serveStatic(req, res, urlPath) {
  let rel = decodeURIComponent(urlPath);
  if (rel === '/' || rel === '') rel = '/index.html';
  // 防目录穿越
  const full = path.normalize(path.join(PUBLIC, rel));
  if (!full.startsWith(PUBLIC)) { res.writeHead(403); res.end('Forbidden'); return; }
  try {
    const data = await readFile(full);
    const ext = path.extname(full).toLowerCase();
    res.writeHead(200, {
      'Content-Type': MIME[ext] || 'application/octet-stream',
      'Cache-Control': ext === '.html' ? 'no-cache' : 'public, max-age=300',
    });
    res.end(data);
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('404 Not Found');
  }
}

/**
 * 仿真请求处理器 (导出供主应用挂载: app.use('/sim', simHandler);
 * Express 挂载时会自动剥离 /sim 前缀, 独立运行时 req.url 即完整路径)。
 */
export async function simHandler(req, res) {
  const t0 = Date.now();
  try {
    const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
    const p = url.pathname;

    if (p === '/api/presets' && req.method === 'GET') {
      return json(res, 200, { presets: PRESETS, demos: Object.keys(DEMOS), demoConfigs: DEMOS });
    }

    if (p === '/api/tuning' && req.method === 'GET') {
      return json(res, 200, tuningMeta());
    }

    if (p === '/api/simulate' && req.method === 'POST') {
      const raw = await readBody(req);
      let input;
      try { input = raw ? JSON.parse(raw) : {}; }
      catch { return json(res, 400, { error: '请求体必须是合法 JSON' }); }
      if (typeof input !== 'object' || input === null || Array.isArray(input)) {
        return json(res, 400, { error: '请求体必须是 JSON 对象' });
      }
      const R = simulate(input);
      res.setHeader('X-Sim-Ms', String(Date.now() - t0));
      return json(res, 200, R);
    }

    if (p === '/api/demo' && req.method === 'GET') {
      const key = url.searchParams.get('preset') || 'default';
      const cfg = DEMOS[key];
      if (!cfg) return json(res, 404, { error: `未知预设 ${key}，可用：${Object.keys(DEMOS).join(', ')}` });
      return json(res, 200, simulate(cfg));
    }

    if (p === '/api/health') return json(res, 200, { ok: true, uptime: process.uptime() });

    if (p.startsWith('/api/')) return json(res, 404, { error: 'Not Found' });

    return await serveStatic(req, res, p);
  } catch (err) {
    console.error(`[error] ${req.method} ${req.url}:`, err.message);
    if (!res.headersSent) json(res, 500, { error: err.message || 'Internal Server Error' });
    else res.end();
  }
}

// ---------- 独立运行 (node sim/server/index.js 或 npm start); 被主应用 import 时不起监听 ----------
const isMain = !!process.argv[1] && pathToFileURL(process.argv[1]).href === import.meta.url;

if (isMain) {
  const server = http.createServer(simHandler);
  server.on('error', (err) => {
    if (err.code === 'EADDRINUSE') {
      console.error(`端口 ${PORT} 已被占用，可用 PORT=8081 npm start 指定其他端口`);
      process.exit(1);
    }
    throw err;
  });
  server.listen(PORT, () => {
    console.log(`FPV 仿真服务已启动: http://localhost:${PORT}`);
    console.log(`  API: POST /api/simulate | GET /api/presets | GET /api/tuning | GET /api/demo?preset=default`);
  });
}
