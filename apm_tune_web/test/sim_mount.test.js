// FPV 仿真台挂载集成测试: /sim/ 与主应用共进程互不干扰 + sim 独立运行不回归
//   node test/sim_mount.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const PORT = 3985;
const SIM_PORT = 3984;

const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'simmount-'));
const ENV = {
  ...process.env,
  PORT: String(PORT),
  STATS_DISABLED: '1',
  AUTH_USER: '',
  AUTH_PASS: '',
  FAULT_DIR: path.join(TMP, 'faults'),
  BACKUP_DIR: path.join(TMP, 'backups'),
  LOG_DIR: path.join(TMP, 'logs'),
};

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function httpGet(base, p) {
  const r = await fetch(`http://127.0.0.1:${base}${p}`, { redirect: 'manual' });   // 不跟随重定向, 便于断言 302
  const text = await r.text();
  return { status: r.status, text, headers: r.headers };
}

const srv = spawn(process.execPath, ['server.js'], { cwd: ROOT, env: ENV });
let simSrv = null;

async function main() {
  try {
    await sleep(1800);

    // ---------- 1. 挂载: 路径与静态资源 ----------
    let r = await httpGet(PORT, '/sim');
    check('/sim 重定向到 /sim/', r.status === 302 && r.headers.get('location') === '/sim/',
      `${r.status} loc=${r.headers.get('location')}`);

    r = await httpGet(PORT, '/sim/');
    check('/sim/ 返回仿真台页面', r.status === 200 && /app\.js/.test(r.text) && /扶摇调参/.test(r.text),
      String(r.status));

    r = await httpGet(PORT, '/sim/style.css');
    check('/sim/style.css 返回样式', r.status === 200 && /backlink/.test(r.text), String(r.status));

    r = await httpGet(PORT, '/sim/app.js');
    check('sim 前端 API 引用为相对路径 (挂载态可用)', r.status === 200
      && /fetch\('api\//.test(r.text) && !/fetch\('\/api\//.test(r.text),
    String(r.status));

    // ---------- 2. 挂载: API (含穿过挂载点的实际计算) ----------
    r = await httpGet(PORT, '/sim/api/health');
    check('/sim/api/health', r.status === 200 && JSON.parse(r.text).ok === true, String(r.status));

    r = await httpGet(PORT, '/sim/api/presets');
    const presets = r.status === 200 ? JSON.parse(r.text) : {};
    check('/sim/api/presets 返回部件库', r.status === 200 && presets.presets && Array.isArray(presets.demos)
      && presets.demos.length > 0, r.status + ' ' + r.text.slice(0, 80));

    const demoKey = (presets.demos && presets.demos[0]) || 'default';
    const t0 = Date.now();
    r = await httpGet(PORT, `/sim/api/demo?preset=${demoKey}`);   // 实际仿真计算穿过挂载点
    let demo = {};
    try { demo = JSON.parse(r.text); } catch { /* ignore */ }
    check(`/sim/api/demo?preset=${demoKey} 完成仿真计算`, r.status === 200
      && typeof demo === 'object' && Object.keys(demo).length > 0,
    `${r.status} ${Date.now() - t0}ms ${r.text.slice(0, 80)}`);

    r = await httpGet(PORT, '/sim/no-such-page');
    check('sim 子树内未知路径 404 (不回落主站页面)', r.status === 404, String(r.status));

    // ---------- 3. 主应用不受影响, API 前缀互不串 ----------
    r = await httpGet(PORT, '/');
    check('主站首页仍正常', r.status === 200 && /扶摇 FuyaoTune/.test(r.text), String(r.status));

    r = await httpGet(PORT, '/lib/fcops.js');
    check('主站 /lib 静态资源仍正常', r.status === 200, String(r.status));

    r = await httpGet(PORT, '/api/presets');
    check('sim API 未泄漏到根路径 /api/*', r.status === 404, String(r.status));

    // ---------- 4. sim 独立运行 (node sim/server/index.js) 不回归 ----------
    simSrv = spawn(process.execPath, ['server/index.js'], {
      cwd: path.join(ROOT, 'sim'),
      env: { ...process.env, PORT: String(SIM_PORT) },
    });
    await sleep(1200);

    r = await httpGet(SIM_PORT, '/');
    check('独立运行: 首页 200', r.status === 200 && /app\.js/.test(r.text), String(r.status));

    r = await httpGet(SIM_PORT, '/style.css');
    check('独立运行: 相对静态资源 200', r.status === 200, String(r.status));

    r = await httpGet(SIM_PORT, '/api/presets');
    check('独立运行: /api/presets 200', r.status === 200 && !!JSON.parse(r.text).presets, String(r.status));

    console.log(failures === 0 ? '\nFPV 仿真台挂载集成测试通过' : `\n${failures} 项失败`);
  } catch (err) {
    console.error('测试异常:', err.message);
    failures++;
  } finally {
    try { srv.kill('SIGKILL'); } catch (e) { /* ignore */ }
    try { if (simSrv) simSrv.kill('SIGKILL'); } catch (e) { /* ignore */ }
    await sleep(200);
    try { fs.rmSync(TMP, { recursive: true, force: true }); } catch (e) { /* ignore */ }
    process.exit(failures === 0 ? 0 : 1);
  }
}

main();
