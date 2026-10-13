// 导引算法实验室静态结构检查: import/资源引用闭环、文档清单与磁盘一致、渲染冒烟
//   node test/guide_dom.test.js
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { renderMarkdown } from '../guide/web/md.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const WEB = path.join(ROOT, 'guide', 'web');

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const read = (p) => fs.readFileSync(p, 'utf8');
const exists = (p) => fs.existsSync(p);

// ---------- 1. ES Module import 闭环 ----------
const webFiles = fs.readdirSync(WEB).filter((f) => f.endsWith('.js'))
  .map((f) => path.join(WEB, f))
  .concat(fs.readdirSync(path.join(WEB, 'core')).map((f) => path.join(WEB, 'core', f)));

let importChecked = 0;
let brokenImport = [];
for (const file of webFiles) {
  const src = read(file);
  const re = /from\s+['"](\.[^'"]+)['"]/g;
  let m;
  while ((m = re.exec(src))) {
    importChecked++;
    const target = path.resolve(path.dirname(file), m[1]);
    if (!exists(target)) brokenImport.push(`${path.basename(file)} -> ${m[1]}`);
  }
}
check(`guide/web 模块 import 全部可解析 (${importChecked} 处)`, importChecked >= 9 && brokenImport.length === 0,
  brokenImport.join(', '));

// ---------- 2. 页面资源引用与 DOM 挂点 ----------
const idx = read(path.join(WEB, 'index.html'));
check('index.html 引用 guide.css/app.js', /href="guide\.css"/.test(idx) && /src="app\.js"/.test(idx));
check('index.html 画布/曲线/面板/状态/抽屉挂点齐全',
  ['id="field"', 'id="plot"', 'id="panel"', 'id="statusbar"', 'id="panelToggle"'].every((s) => idx.includes(s)));

const docsp = read(path.join(WEB, 'docs.html'));
check('docs.html 引用 guide.css/docs.js', /href="guide\.css"/.test(docsp) && /src="docs\.js"/.test(docsp));
check('docs.html 目录/正文/元信息/翻页/抽屉挂点齐全',
  ['id="docsSidebar"', 'id="docsBody"', 'id="docsMeta"', 'id="docsPager"', 'id="libToggle"'].every((s) => docsp.includes(s)));

// 页面里出现的相对资源必须真实存在
for (const [name, html] of [['index.html', idx], ['docs.html', docsp]]) {
  const refs = [...html.matchAll(/(?:href|src)="([^":][^"]*)"/g)].map((m) => m[1])
    .filter((u) => !u.startsWith('/') && !u.startsWith('#') && !u.startsWith('data:'));
  const missing = refs.filter((u) => !exists(path.join(WEB, u.split('?')[0])));
  check(`${name} 相对资源引用闭环 (${refs.length} 处)`, refs.length > 0 && missing.length === 0, missing.join(', '));
}

// ---------- 3. docs.js 文档清单与磁盘一致 ----------
const docsJs = read(path.join(WEB, 'docs.js'));
const libDirs = { docs: path.join(ROOT, 'guide', 'docs'), refs: path.join(ROOT, 'guide', 'references') };
const libBlocks = [...docsJs.matchAll(/id:\s*'(docs|refs)'[\s\S]*?files:\s*\[([\s\S]*?)\]/g)];
check('docs.js 含两个库清单', libBlocks.length === 2, `found ${libBlocks.length}`);

let manifestFiles = 0;
const manifestMissing = [];
for (const [, libId, block] of libBlocks) {
  const files = [...block.matchAll(/'([^']+\.md)'/g)].map((m) => m[1]);
  manifestFiles += files.length;
  for (const f of files) {
    if (!exists(path.join(libDirs[libId], f))) manifestMissing.push(`${libId}/${f}`);
  }
}
check(`清单 ${manifestFiles} 篇文档均存在于磁盘`, manifestFiles === 17 && manifestMissing.length === 0,
  manifestMissing.join(', '));

// 磁盘上 docs/references 的正文文档也必须都登记进清单 (防新增文档漏挂)
const listed = new Set([...docsJs.matchAll(/'([^']+\.md)'/g)].map((m) => m[1]));
const unlisted = [];
for (const [libId, dir] of Object.entries(libDirs)) {
  for (const f of fs.readdirSync(dir)) {
    if (f.endsWith('.md') && !listed.has(f)) unlisted.push(`${libId}/${f}`);
  }
}
check('磁盘 .md 与清单双向一致', unlisted.length === 0, unlisted.join(', '));

// ---------- 4. Markdown 渲染冒烟 (真实文档) ----------
const resolve = { link: (h) => h, img: (h) => h };
for (const f of [
  path.join(libDirs.docs, 'README.md'),
  path.join(libDirs.docs, '01_比例导引法PN.md'),
  path.join(libDirs.refs, '06_中段制导与拦截点计算.md'),
  path.join(libDirs.refs, 'README.md'),
]) {
  const html = renderMarkdown(read(f), resolve);
  const ok = /<h1/.test(html) && /<table>/.test(html) && !/\*\*/.test(html)
    && !/\|---/.test(html) && !/</.test(html.replace(/<[^>]+>/g, ''));
  check(`渲染 ${path.basename(f)}`, ok && html.length > 1000, `${html.length}B`);
}

// 安全: 文本中的 HTML 被转义
const evil = renderMarkdown('段落 <script>alert(1)</script> **粗**', resolve);
check('HTML 被转义 (无注入面)', !/<script>/.test(evil) && /&lt;script&gt;/.test(evil) && /<strong>粗<\/strong>/.test(evil));

console.log(failures === 0 ? '\n导引算法实验室静态结构检查通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
