// 故障 AI prompt 单测: lib/faultai.js
//   node test/faultai.test.js
import { FAULT_SYSTEM_PROMPT, buildFaultUserPrompt } from '../lib/faultai.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

check('system prompt 非空', typeof FAULT_SYSTEM_PROMPT === 'string' && FAULT_SYSTEM_PROMPT.length > 100);
check('system prompt 含结构化章节', /可能原因/.test(FAULT_SYSTEM_PROMPT) && /修复方案/.test(FAULT_SYSTEM_PROMPT));

const full = buildFaultUserPrompt({
  title: '解锁后电机抖动',
  aircraft: '5寸穿越机',
  firmware: 'ArduCopter 4.6.0',
  severity: 'high',
  status: 'open',
  tags: ['电调', '解锁'],
  symptom: '解锁瞬间电机快速抖动并停转',
  context: 'MOT_SPIN_ARM=0.10',
  cause: '怀疑电调协议不匹配',
  fix: '尚未修复',
});
check('prompt 含标题', full.includes('解锁后电机抖动'));
check('prompt 含机型', full.includes('5寸穿越机'));
check('prompt 含标签', full.includes('电调, 解锁'));
check('prompt 含现象章节', /## 故障现象/.test(full) && full.includes('解锁瞬间电机快速抖动'));
check('prompt 含上下文章节', /## 上下文/.test(full) && full.includes('MOT_SPIN_ARM=0.10'));
check('prompt 含结尾指令', full.includes('请基于以上信息给出故障分析报告'));

const sparse = buildFaultUserPrompt({ title: '只有标题' });
check('空字段不生成章节', !/## 故障现象/.test(sparse) && !/## 已尝试/.test(sparse));
check('空标题仍可生成', buildFaultUserPrompt({}).includes('# 飞控故障记录'));
check('tags 字符串也支持', buildFaultUserPrompt({ tags: 'a,b' }).includes('a,b'));

console.log(failures === 0 ? '\n故障 AI Prompt 单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
