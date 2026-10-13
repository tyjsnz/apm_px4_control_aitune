// 导引算法实验室挂载集成测试: /guide 三处静态挂载 + 与既有子系统互不干扰
//   node test/guide_mount.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const PORT = 3986;

const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'guidemount-'));
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

async function get(p) {
  const r = await fetch(`http://127.0.0.1:${PORT}${p}`, { redirect: 'manual' });
  const text = await r.text();
  return { status: r.status, text, headers: r.headers };
}

const srv = spawn(process.execPath, ['server.js'], { cwd: ROOT, env: ENV });

async function main() {
  try {
    await sleep(1800);

    // ---------- 1. /guide (交互仿真) ----------
    let r = await get('/guide');
    check('/guide 302 到 /guide/', r.status === 302 && r.headers.get('location') === '/guide/',
      `${r.status} loc=${r.headers.get('location')}`);

    r = await get('/guide/');
    check('/guide/ 返回仿真页面', r.status === 200 && /id="field"/.test(r.text)
      && /导引算法实验室/.test(r.text) && /app\.js/.test(r.text), String(r.status));

    r = await get('/guide/guide.css');
    check('/guide/guide.css 样式可取', r.status === 200 && /\.sidebar/.test(r.text), String(r.status));

    r = await get('/guide/app.js');
    check('/guide/app.js 主逻辑可取 (ESM)', r.status === 200 && /from '.\/core\/simulation.js'/.test(r.text),
      String(r.status));

    r = await get('/guide/core/guidance.js');
    check('/guide/core/guidance.js 数值内核可取', r.status === 200 && /export function wrapPi/.test(r.text),
      String(r.status));

    r = await get('/guide/canvas.js');
    check('/guide/canvas.js 画布模块可取', r.status === 200 && /class Canvas/.test(r.text), String(r.status));

    r = await get('/guide/no-such-file.js');
    check('guide 子树未知路径 404', r.status === 404, String(r.status));

    // ---------- 2. /guide/docs (理论手册) ----------
    r = await get('/guide/docs');
    check('/guide/docs 302 到 /guide/docs/', r.status === 302 && r.headers.get('location') === '/guide/docs/',
      `${r.status} loc=${r.headers.get('location')}`);

    r = await get('/guide/docs/' + encodeURIComponent('01_比例导引法PN.md'));
    check('理论手册 .md 可取 (中文文件名)', r.status === 200 && /比例导引/.test(r.text), String(r.status));

    r = await get('/guide/docs/' + encodeURIComponent('README.md'));
    check('理论手册总目录可取', r.status === 200 && /帮助文档总目录/.test(r.text), String(r.status));

    // ---------- 3. /guide/refs (参考资料 + 插图) ----------
    r = await get('/guide/refs');
    check('/guide/refs 302 到 /guide/refs/', r.status === 302 && r.headers.get('location') === '/guide/refs/',
      `${r.status} loc=${r.headers.get('location')}`);

    r = await get('/guide/refs/' + encodeURIComponent('06_中段制导与拦截点计算.md'));
    check('参考资料 .md 可取', r.status === 200 && /拦截点/.test(r.text), String(r.status));

    r = await get('/guide/refs/figures/fig01_phases.png');
    check('参考资料插图可取 (相对路径解析目标)', r.status === 200
      && (r.headers.get('content-type') || '').includes('image'), `${r.status} ${r.headers.get('content-type')}`);

    // ---------- 4. 文档库页面 ----------
    r = await get('/guide/docs.html');
    check('/guide/docs.html 返回文档库', r.status === 200 && /id="docsBody"/.test(r.text)
      && /docs\.js/.test(r.text), String(r.status));

    r = await get('/guide/docs.js');
    check('/guide/docs.js 含两库路由基址', r.status === 200
      && r.text.includes("/guide/docs/") && r.text.includes("/guide/refs/"), String(r.status));

    r = await get('/guide/md.js');
    check('/guide/md.js 渲染器可取', r.status === 200 && /renderMarkdown/.test(r.text), String(r.status));

    // ---------- 5. 既有子系统不受影响 ----------
    r = await get('/');
    check('入口页含第三张导引实验室卡片', r.status === 200 && /href="\/guide\/"/.test(r.text)
      && /导引算法实验室/.test(r.text), String(r.status));

    r = await get('/app/');
    check('/app/ 调参工作台不受影响', r.status === 200 && /btnConnect/.test(r.text), String(r.status));

    r = await get('/sim/');
    check('/sim/ FPV 仿真台不受影响', r.status === 200 && /app\.js/.test(r.text), String(r.status));

    r = await get('/sim/api/health');
    check('sim API 不受 guide 挂载影响', r.status === 200 && JSON.parse(r.text).ok === true, String(r.status));

    r = await get('/lib/fcops.js');
    check('主站 /lib 静态资源仍正常', r.status === 200, String(r.status));

    console.log(failures === 0 ? '\n导引算法实验室挂载集成测试通过' : `\n${failures} 项失败`);
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
