// Golden 比对: Python ai_pid_tune._ot_stages 生成的方案 vs 前端 onetouch.js 移植实现
//   node test/golden_compare.js
import fs from 'fs';
import path from 'path';
import vm from 'vm';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');

const sandbox = { window: {}, console };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', 'onetouch_data.js'), 'utf8'), sandbox);
vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', 'onetouch.js'), 'utf8'), sandbox);
const core = sandbox.window.OneTouchCore;
if (!core) { console.error('未能加载 OneTouchCore'); process.exit(1); }

const golden = JSON.parse(fs.readFileSync(path.join(__dirname, 'golden', 'plans.json'), 'utf8'));

let errs = [];
let compared = 0;

const numEq = (a, b) => {
  if (a === null && b === null) return true;
  if (a === null || b === null) return false;
  return Math.abs(Number(a) - Number(b)) <= Math.max(1e-6, Math.abs(Number(a)) * 1e-9);
};
const arrEq = (a, b) => {
  if (a === null && b === null) return true;
  if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return false;
  return a.every((v, i) => (Array.isArray(v) && Array.isArray(b[i]))
    ? (v.length === b[i].length && v.every((x, j) => (typeof x === 'number' && typeof b[i][j] === 'number')
      ? numEq(x, b[i][j]) : x === b[i][j]))
    : (typeof v === 'number' && typeof b[i] === 'number' ? numEq(v, b[i]) : v === b[i]));
};

for (const caseDef of golden) {
  const ctxP = caseDef.ctx;
  const ctx = {
    size: ctxP.size, cells: ctxP.cells, hover: ctxP.hover,
    escLinear: ctxP.esc_linear, purpose: ctxP.purpose, profile: ctxP.profile,
  };
  if (ctxP.gyro !== undefined && ctxP.gyro !== null) ctx.gyro = ctxP.gyro;
  const got = core.otStages(ctx);
  const want = caseDef.stages;
  const tag = `[${ctx.purpose}/${ctx.profile}/${ctx.size}寸/${ctx.cells}S/hover${ctx.hover}${ctx.escLinear ? '/esc' : ''}${ctx.gyro ? '/gyro' + ctx.gyro : ''}]`;

  if (got.length !== want.length) {
    errs.push(`${tag} 阶段数不符: ${got.length} vs ${want.length}`);
    continue;
  }
  for (let i = 0; i < want.length; i++) {
    if (got[i].id !== want[i].id || got[i].label !== want[i].label) {
      errs.push(`${tag} 阶段${i} 不符: ${got[i].id}/${got[i].label} vs ${want[i].id}/${want[i].label}`);
      continue;
    }
    if (got[i].items.length !== want[i].items.length) {
      errs.push(`${tag} 阶段${want[i].id} 项数不符: ${got[i].items.length} vs ${want[i].items.length}`);
      continue;
    }
    for (let j = 0; j < want[i].items.length; j++) {
      const g = got[i].items[j];
      const w = want[i].items[j];
      const it = `${tag} ${want[i].id}.${w.name}`;
      if (g.name !== w.name) { errs.push(`${it} 名称不符: ${g.name}`); continue; }
      if (!numEq(g.value, w.value)) errs.push(`${it}.value: ${g.value} vs ${w.value}`);
      if ((g.unit || '') !== (w.unit || '')) errs.push(`${it}.unit: ${g.unit} vs ${w.unit}`);
      if (!arrEq(g.rng, w.rng)) errs.push(`${it}.rng: ${JSON.stringify(g.rng)} vs ${JSON.stringify(w.rng)}`);
      if (!!g.check !== !!w.check) errs.push(`${it}.check: ${g.check} vs ${w.check}`);
      if ((g.desc || '') !== (w.desc || '')) errs.push(`${it}.desc 不符`);
      const norm = (s) => String(s || '').replace(/(\d)\.0(?=\D|$)/g, '$1');
      const gh = norm((g.help || '').split('\n')[0]);
      const wh = norm((w.help || '').split('\n')[0]);
      if (gh !== wh) errs.push(`${it}.help首行: ${gh} vs ${wh}`);
      if (!arrEq(g.aliases, w.aliases)) {
        errs.push(`${it}.aliases: ${JSON.stringify(g.aliases)} vs ${JSON.stringify(w.aliases)}`);
      }
      compared++;
    }
  }

  // 交叉校验规则冒烟
  const rows = got.flatMap((st) => st.items.map((e) => ({ entry: e, value: e.value })));
  const issues = core.otValidate(rows);
  if (!Array.isArray(issues)) errs.push(`${tag} otValidate 未返回数组`);
}

if (errs.length) {
  console.error('Golden 比对失败 (' + errs.length + ' 项):\n' + errs.slice(0, 60).join('\n')
    + (errs.length > 60 ? `\n... 其余 ${errs.length - 60} 项` : ''));
  process.exit(1);
}
console.log(`Golden 比对通过: ${golden.length} 组方案, ${compared} 项全部一致`);
