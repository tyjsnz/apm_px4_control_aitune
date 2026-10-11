// 推力线性化体检单测: public/thrustcheck.js (vm 沙箱) + lib/thrustai.js prompt
//   node test/thrust.test.js
import fs from 'fs';
import path from 'path';
import vm from 'vm';
import { fileURLToPath } from 'url';
import { THRUST_SYSTEM_PROMPT, buildThrustUserPrompt } from '../lib/thrustai.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');

let failures = 0;
function check(cond, msg) {
  if (cond) { console.log('  ok  ' + msg); } else { console.log('  FAIL ' + msg); failures++; }
}
const approx = (a, b, eps = 1e-6) => Math.abs(a - b) <= eps;

// ---------- vm 沙箱加载前端纯逻辑 ----------
const sandbox = { window: {}, console, Math, Number, isFinite };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', 'thrustcheck.js'), 'utf8'), sandbox);
const TC = sandbox.window.ThrustCheck;
check(!!TC && typeof TC.buildRows === 'function', '加载 window.ThrustCheck');

// ---------- expoToActuator: AP 官方公式 ----------
check(approx(TC.expoToActuator(0.5, 0), 0.5), 'e=0 时 a=T (线性)');
check(approx(TC.expoToActuator(0.5, 0, 0.9), 0.45), 'e=0 时 a=liftMax*T');
check(approx(TC.expoToActuator(0, 0.55), 0), 'e=0.55 时 a(0)=0');
check(approx(TC.expoToActuator(1, 0.55), 1), 'e=0.55 时 a(1)=1');
const a055 = TC.expoToActuator(0.5, 0.55);
check(a055 > 0.62 && a055 < 0.64, `e=0.55, T=0.5 -> a≈0.628 (实际 ${a055.toFixed(4)})`);
// 反演检验: AP 假设电机模型 plant(a) = (1-e)*a + e*a^2, expoToActuator 应为其逆
let roundOk = true;
for (const e of [0.1, 0.55, 0.75, -0.5]) {
  for (let i = 1; i < 10; i++) {
    const t = i / 10;
    const a = TC.expoToActuator(t, e);
    const plant = (1 - e) * a + e * a * a;
    if (!approx(plant, t, 1e-6)) { roundOk = false; break; }
  }
}
check(roundOk, 'plant(expoToActuator(T, e)) == T (曲线与 AP 电机模型互逆)');
// 单调性
let mono = true;
for (const e of [0, 0.55, -1]) {
  let prev = -1;
  for (let i = 0; i <= 40; i++) {
    const a = TC.expoToActuator(i / 40, e);
    if (a < prev - 1e-12) mono = false;
    prev = a;
  }
}
check(mono, '补偿曲线单调不减');
check(TC.expoToActuator(1.5, 0.55) <= 1 && TC.expoToActuator(-1, 0.55) >= 0, '输入越界被钳位到 [0,1]');

// ---------- curvePoints / actuatorToExpoThrust ----------
const pts = TC.curvePoints(0.55, { steps: 40 });
check(pts.length === 41, '曲线采样 steps+1 个点');
check(approx(pts[0].a, 0) && approx(pts[40].a, 1), '曲线端点 0 -> 1');
check(approx(TC.actuatorToExpoThrust(pts[20].a, 0.55), pts[20].t, 1e-4), 'actuatorToExpoThrust 为曲线逆映射');

// ---------- buildRows: 健康参数集 -> 0 warn ----------
const healthy = {
  MOT_PWM_TYPE: 6, MOT_THST_EXPO: 0.55, MOT_THST_HOVER: 0.22,
  MOT_HOVER_LEARN: 2, MOT_SPIN_ARM: 0.10, MOT_SPIN_MIN: 0.15,
  MOT_SPIN_MAX: 0.95, MOT_PWM_MIN: 1000, MOT_PWM_MAX: 2000,
};
let rows = TC.buildRows(healthy, {});
check(rows.length === 8, `健康集 8 行 (实际 ${rows.length})`);
check(rows.filter((r) => r.level === 'warn').length === 0, '健康集无 warn');
check(rows.some((r) => r.name === 'MOT_PWM_TYPE' && r.level === 'ok'), 'DShot600 判为 ok');
check(rows.some((r) => r.name === 'HOVER ↔ SPIN_MIN' && r.level === 'ok'), '悬停/最小输出交叉 ok');
check(rows.some((r) => r.name === 'MOT_PWM_MIN~MAX' && r.level === 'ok'), '标准 1000~2000 行程 ok');

// ---------- buildRows: 各类告警 ----------
let w = TC.buildRows({ ...healthy, MOT_SPIN_ARM: 0.10, MOT_SPIN_MIN: 0.11 }, {})
  .filter((r) => r.level === 'warn');
check(w.some((r) => r.name === 'SPIN_MIN ↔ SPIN_ARM'), 'SPIN_MIN < ARM+0.03 判 warn');
w = TC.buildRows({ ...healthy, MOT_THST_HOVER: 0.04 }, {}).filter((r) => r.level === 'warn');
check(w.some((r) => r.name === 'MOT_THST_HOVER'), '悬停油门 0.04 判 warn');
w = TC.buildRows({ ...healthy, MOT_PWM_MIN: 1100, MOT_PWM_MAX: 1250 }, {}).filter((r) => r.level === 'warn');
check(w.some((r) => r.name === 'MOT_PWM_MIN~MAX'), '行程 150us 判 warn');
w = TC.buildRows(healthy, { escLinear: true }).filter((r) => r.level === 'warn');
check(w.some((r) => r.name === 'MOT_THST_EXPO'), '勾选电调已线性化 + EXPO=0.55 判 warn');

// ---------- buildRows: 缺参 -> na ----------
rows = TC.buildRows({}, {});
check(rows.every((r) => r.level === 'na'), '空值集全部 na');
check(rows.some((r) => r.name === 'MOT_THST_EXPO' && r.level === 'na'), '缺失 EXPO 标 na');

// ---------- verdict ----------
let v = TC.verdict(TC.buildRows(healthy, {}), healthy);
check(v.includes('未发现需要关注的异常项'), '健康集结论无异常');
check(v.includes('官方按桨径') && v.includes('油门阶梯实测'), 'EXPO=0.55 结论含官方参考与实测提示');
v = TC.verdict(TC.buildRows({ ...healthy, MOT_THST_HOVER: 0.8 }, {}),
  { ...healthy, MOT_THST_HOVER: 0.8 });
check(/发现 1 项需要关注/.test(v), 'warn 计数进入结论');
v = TC.verdict([], {});
check(v.includes('未读到 MOT_THST_EXPO'), '缺 EXPO 结论明示无法判定');

// ---------- lib/thrustai.js prompt ----------
check(typeof THRUST_SYSTEM_PROMPT === 'string' && THRUST_SYSTEM_PROMPT.length > 100, 'system prompt 非空');
check(/结论速览/.test(THRUST_SYSTEM_PROMPT) && /建议修改/.test(THRUST_SYSTEM_PROMPT)
  && /地面验证步骤/.test(THRUST_SYSTEM_PROMPT), 'system prompt 含结构化章节');
check(/MOT_THST_EXPO/.test(THRUST_SYSTEM_PROMPT), 'system prompt 含核心参数');

const prompt = buildThrustUserPrompt({
  firmware: 'ArduPilot',
  manual: true,
  values: { MOT_THST_EXPO: 0.55, MOT_SPIN_MIN: 0.11 },
  rows: [{ name: 'SPIN_MIN ↔ SPIN_ARM', value: '0.11 / 0.1', level: 'warn', text: '裕度不足' }],
  verdictText: '发现 1 项需要关注。',
  inputs: { note: '5寸 6S, 低油门段有突跳' },
});
check(prompt.includes('# 推力线性化体检数据'), 'prompt 含标题');
check(prompt.includes('MOT_THST_EXPO = 0.55'), 'prompt 含参数值');
check(prompt.includes('手工输入假设值'), 'manual 标记进入 prompt');
check(prompt.includes('[warn] SPIN_MIN ↔ SPIN_ARM') && prompt.includes('裕度不足'), 'prompt 含体检行');
check(prompt.includes('发现 1 项需要关注'), 'prompt 含文字结论');
check(prompt.includes('5寸 6S, 低油门段有突跳'), 'prompt 含用户备注');
check(!prompt.includes('手工输入假设值 (非实机读取)') || prompt.includes('数据来源'), '数据来源字段存在');

const empty = buildThrustUserPrompt({});
check(empty.includes('(无)'), '空 payload 参数章节为 (无)');

console.log(failures === 0 ? '\n推力线性化体检单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
