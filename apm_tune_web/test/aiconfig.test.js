// 全局 AI 配置客户端单测: public/aiconfig.js (用假 ws / 最小环境在 node 中执行)
//   node test/aiconfig.test.js
import fs from 'fs';
import path from 'path';
import vm from 'vm';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const code = fs.readFileSync(path.join(__dirname, '..', 'public', 'aiconfig.js'), 'utf8');

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

function load() {
  const sent = [];
  const sandbox = {
    window: {},
    ws: { readyState: 1, send: (s) => sent.push(JSON.parse(s)) },
    console: { log() {}, warn() {}, error() {} },
    document: {},
  };
  vm.createContext(sandbox);
  vm.runInContext(code, sandbox);
  return { ai: sandbox.window.AIConfig, sent };
}

const PROVIDERS = [
  { id: 'deepseek', label: 'DeepSeek', models: ['deepseek-flash'], keyOptional: false },
  { id: 'ollama', label: 'Ollama 本地', models: ['qwen3:8b'], keyOptional: true },
];

// 1. 初始
const { ai, sent } = load();
check('初始未配置', ai.isConfigured() === false);
check('初始状态文案含"读取中"', /读取中/.test(ai.statusText()), ai.statusText());

// 2. onOpen 请求服务商 + 配置
ai.onOpen();
check('onOpen 请求 ai_providers', sent.some((m) => m.type === 'ai_providers'));
check('onOpen 请求 ai_config_get', sent.some((m) => m.type === 'ai_config_get'));

// 3. 收到服务商与配置(来自服务端, 无明文 Key)
ai.handleMessage({ type: 'ai_providers', providers: PROVIDERS });
ai.handleMessage({ type: 'ai_config', config: { provider: 'deepseek', model: 'deepseek-flash', baseUrl: '', remember: false, hasKey: true } });
check('服务端配置 -> 已配置', ai.isConfigured() === true);
check('hasKey 反映真实存在', ai.get().hasKey === true);
check('状态文案显示 DeepSeek', /DeepSeek \/ deepseek-flash/.test(ai.statusText()), ai.statusText());
check('状态文案标记 Key 已设置', /Key 已设置/.test(ai.statusText()), ai.statusText());

// 4. 未配 Key 且服务商需要 Key -> 未配置
ai.handleMessage({ type: 'ai_config', config: { provider: 'deepseek', model: 'deepseek-flash', hasKey: false } });
check('缺 Key 视为未配置', ai.isConfigured() === false);
check('状态提示未填 Key', /未填 Key/.test(ai.statusText()), ai.statusText());

// 5. 可免 Key 服务商 -> 已配置
ai.handleMessage({ type: 'ai_config', config: { provider: 'ollama', model: 'qwen3:8b', hasKey: false } });
check('可免 Key 服务商已配置', ai.isConfigured() === true);

// 6. 空配置 -> 未配置
ai.handleMessage({ type: 'ai_config', config: { provider: '', model: '', hasKey: false } });
check('空配置未配置', ai.isConfigured() === false);
check('空配置文案', /未配置/.test(ai.statusText()), ai.statusText());

// 7. toRequest 不再携带任何 AI 字段(配置在服务端)
check('toRequest 返回空对象', Object.keys(ai.toRequest()).length === 0);

// 8. save 发送 ai_config_set, Key 非空才带 apiKey
const { ai: ai2, sent: sent2 } = load();
ai2.save({ config: { provider: 'deepseek', model: 'm', baseUrl: '', remember: false }, apiKey: '' });
const setMsg = sent2.find((m) => m.type === 'ai_config_set');
check('save 发送 ai_config_set', !!setMsg);
check('空 Key 时不带 apiKey 字段', setMsg && !('apiKey' in setMsg), JSON.stringify(setMsg));
ai2.save({ config: { provider: 'deepseek', model: 'm', remember: true }, apiKey: 'sk-1' });
const setMsg2 = sent2.filter((m) => m.type === 'ai_config_set').pop();
check('填写 Key 时携带 apiKey', setMsg2.apiKey === 'sk-1', JSON.stringify(setMsg2));

// 9. clearKey 发送清除消息
ai2.clearKey();
check('clearKey 发送 ai_config_clear_key', sent2.some((m) => m.type === 'ai_config_clear_key'));

// 10. subscribe 立即触发并随配置变化触发
let hits = 0;
ai2.subscribe(() => { hits++; });
ai2.handleMessage({ type: 'ai_config', config: { provider: 'deepseek', model: 'm', hasKey: true } });
check('subscribe 被触发', hits >= 2, String(hits));

console.log(failures === 0 ? '\n全局 AI 配置客户端单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
