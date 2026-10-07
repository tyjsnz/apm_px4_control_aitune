// charts.js 渲染逻辑单测 (最小 DOM 打桩, 不依赖浏览器)
//   node test/charts.test.js
import fs from 'fs';
import path from 'path';
import vm from 'vm';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');

function makeEl(ns, tag) {
  return {
    ns, tag,
    attrs: {}, children: [], _html: '',
    setAttribute(k, v) { this.attrs[k] = v; },
    getAttribute(k) { return this.attrs[k]; },
    appendChild(c) { this.children.push(c); return c; },
    set innerHTML(v) { this._html = v; if (v === '') this.children = []; },
    get innerHTML() { return this._html; },
    set textContent(v) { this._text = v; },
    get textContent() { return this._text || ''; },
    set className(v) { this.attrs.class = v; },
    get className() { return this.attrs.class || ''; },
    set style(v) { this.attrs.style = v; },
    get style() { return this.attrs.style || {}; },
  };
}

const document = {
  createElement: (t) => makeEl(null, t),
  createElementNS: (ns, t) => makeEl(ns, t),
  createTextNode: (t) => ({ tag: '#text', _text: String(t), textContent: String(t), children: [] }),
};
const sandbox = { window: {}, document, console };
sandbox.window.document = document;
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', 'charts.js'), 'utf8'), sandbox);

const groups = [
  {
    id: 'imu', title: '陀螺仪',
    series: [
      { name: 'GyrX', unit: 'rad/s', t: [0, 1, 2, 3], v: [0, 0.1, -0.1, 0.05] },
      { name: 'GyrY', unit: 'rad/s', t: [0, 1, 2, 3], v: [0.2, 0.1, 0, -0.2] },
    ],
  },
  {
    id: 'gps', title: 'GPS',
    series: [{ name: 'Spd', unit: 'm/s', t: [0, 0.5, 1], v: [0, 2, 4] }],
  },
];

const container = makeEl(null, 'div');
sandbox.window.Charts.render(container, groups);

let failures = 0;
const check = (cond, msg) => { if (cond) console.log('  ok  ' + msg); else { console.log('  FAIL ' + msg); failures++; } };

const cards = container.children.filter((c) => c.className === 'chart-card');
check(cards.length === 2, '两个分组生成两张图: ' + cards.length);

let svgCount = 0; let polyCount = 0; let legendCount = 0;
for (const card of cards) {
  const stack = [...card.children];
  while (stack.length) {
    const el = stack.pop();
    if (el.tag === 'svg') svgCount++;
    if (el.tag === 'polyline') polyCount++;
    if (el.className === 'chart-legend-item') legendCount++;
    if (el.children) stack.push(...el.children);
  }
}
check(svgCount === 2, '生成 2 个 <svg>: ' + svgCount);
check(polyCount === 3, '折线数=3 (2+1): ' + polyCount);
check(legendCount === 3, '图例项=3: ' + legendCount);

// 空数据不崩
const empty = makeEl(null, 'div');
sandbox.window.Charts.render(empty, []);
check(empty._html.includes('chart-empty') || empty._html.includes('未包含'), '空数据给出提示');

console.log(`\n${failures === 0 ? '图表渲染单测通过' : failures + ' 项失败'}`);
process.exit(failures === 0 ? 0 : 1);
