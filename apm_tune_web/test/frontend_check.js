// 前端静态检查: HTML 中引用的资源与 JS 里的 getElementById/$ 目标是否存在
//   node test/frontend_check.js
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUB = path.join(__dirname, '..', 'public');

const html = fs.readFileSync(path.join(PUB, 'index.html'), 'utf8');
const htmlIds = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map((m) => m[1]));

// 动态创建的 id (由 JS 运行时生成)
const DYNAMIC = new Set(['otAccelModal', 'otAccelText', 'otAccelConfirm', 'otAccelAbort', 'otSpinCk', 'otSpinGo', 'otSpinCancel']);

const files = ['app.js', 'onetouch.js', 'datatools.js', 'charts.js', 'fault.js', 'aiconfig.js'];
const errs = [];
let checked = 0;

for (const f of files) {
  const src = fs.readFileSync(path.join(PUB, f), 'utf8');
  const refs = new Set();
  for (const m of src.matchAll(/getElementById\(\s*'([^']+)'\s*\)/g)) refs.add(m[1]);
  for (const m of src.matchAll(/\$\(\s*'([^']+)'\s*\)/g)) refs.add(m[1]);
  for (const id of refs) {
    checked++;
    if (!htmlIds.has(id) && !DYNAMIC.has(id)) errs.push(`${f}: 引用了不存在的元素 id="${id}"`);
  }
}

// 检查 HTML 引用的脚本/样式文件存在
for (const m of html.matchAll(/(?:src|href)="([^"]+)"/g)) {
  const ref = m[1];
  if (/^https?:/.test(ref)) continue;
  const p = path.join(PUB, ref);
  if (!fs.existsSync(p)) errs.push(`index.html: 引用文件不存在 ${ref}`);
}

if (errs.length) {
  console.error('前端静态检查失败:\n' + errs.join('\n'));
  process.exit(1);
}
console.log(`前端静态检查通过: ${checked} 个元素引用, 资源文件齐全`);
