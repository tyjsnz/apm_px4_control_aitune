// .bin 解析器对拍: Node parseBin vs pymavlink DFReader (期望值由 make_sample_bin.py 生成)
//   node test/binlog.test.js
import fs from 'fs';
import path from 'path';
import { spawnSync } from 'child_process';
import { fileURLToPath } from 'url';
import { parseBin } from '../lib/binlog.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const GOLD = path.join(__dirname, 'golden');

const gen = spawnSync('python3', [path.join(__dirname, 'make_sample_bin.py')], { cwd: ROOT, encoding: 'utf8' });
if (gen.status !== 0) {
  console.error('生成样例失败:\n' + gen.stderr);
  process.exit(1);
}

const expected = JSON.parse(fs.readFileSync(path.join(GOLD, 'sample_expected.json'), 'utf8'));
const buf = fs.readFileSync(path.join(GOLD, 'sample.bin'));

const got = [];
const stats = parseBin(buf, (name, fields) => got.push({ name, fields }));

let failures = 0;
const check = (cond, msg) => { if (cond) console.log('  ok  ' + msg); else { console.log('  FAIL ' + msg); failures++; } };

check(stats.records === expected.length, `记录数 ${stats.records} == ${expected.length}`);

const numEq = (a, b) => {
  if (typeof a === 'number' && typeof b === 'number') return Math.abs(a - b) <= Math.max(1e-4, Math.abs(b) * 1e-4);
  return a === b;
};

for (let i = 0; i < Math.min(got.length, expected.length); i++) {
  const g = got[i]; const w = expected[i];
  check(g.name === w.name, `记录${i} 名称 ${g.name} == ${w.name}`);
  for (const k of Object.keys(w.fields)) {
    check(numEq(g.fields[k], w.fields[k]), `记录${i} ${w.name}.${k}: ${g.fields[k]} == ${w.fields[k]}`);
  }
}

// 截断缓冲区不应抛异常
try {
  parseBin(buf.subarray(0, 40), () => {});
  check(true, '截断日志不抛异常');
} catch (e) {
  check(false, '截断日志不抛异常: ' + e.message);
}

console.log(`\n${failures === 0 ? '.bin 解析对拍通过' : failures + ' 项失败'}`);
process.exit(failures === 0 ? 0 : 1);
