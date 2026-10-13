// AI 配置客户端单测: public/aiconfig.js (B 方案: Key 归属浏览器 localStorage)
//   用假 ws / 假 localStorage 在 node vm 中执行
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

/** store: Map<string,string> 模拟 localStorage (跨"重载"共享) */
function load(store = new Map()) {
  const sent = [];
  const sandbox = {
    window: {},
    ws: { readyState: 1, send: (s) => sent.push(JSON.parse(s)) },
    console: { log() {}, warn() {}, error() {} },
    document: {},
    localStorage: {
      getItem: (k) => (store.has(k) ? store.get(k) : null),
      setItem: (k, v) => { store.set(k, String(v)); },
      removeItem: (k) => { store.delete(k); },
    },
  };
  vm.createContext(sandbox);
  vm.runInContext(code, sandbox);
  return { ai: sandbox.window.AIConfig, sent, store };
}

const PROVIDERS = [
  { id: 'deepseek', label: 'DeepSeek', models: ['deepseek-flash'], keyOptional: false },
  { id: 'ollama', label: 'Ollama 本地', models: ['qwen3:8b'], keyOptional: true },
];

// 1. 初始 (本机无任何存储)
const { ai, sent, store } = load();
check('初始未配置', ai.isConfigured() === false);
check('初始状态文案含"读取中"', /读取中/.test(ai.statusText()), ai.statusText());
check('初始 toRequest 为空', Object.keys(ai.toRequest()).length === 0);

// 2. onOpen 只拉取服务商与服务端默认, 不上传任何配置
ai.onOpen();
check('onOpen 请求 ai_providers', sent.some((m) => m.type === 'ai_providers'));
check('onOpen 请求 ai_config_get', sent.some((m) => m.type === 'ai_config_get'));
check('从不发送 ai_config_set', !sent.some((m) => m.type === 'ai_config_set'));
check('从不发送 ai_config_clear_key', !sent.some((m) => m.type === 'ai_config_clear_key'));

// 3. 服务端默认 (env AI_*) 可用 → 用它兜底
ai.handleMessage({ type: 'ai_providers', providers: PROVIDERS });
ai.handleMessage({ type: 'ai_config', config: { provider: 'deepseek', model: 'srv-model', baseUrl: '', hasKey: true, source: 'server' } });
check('服务端默认 → 已配置', ai.isConfigured() === true);
check('来源标记为 server', ai.get().source === 'server');
check('状态文案标注服务器默认', /服务器默认/.test(ai.statusText()), ai.statusText());
check('服务端默认时不携带本机字段 (交给服务端)', Object.keys(ai.toRequest()).length === 0);

// 4. 服务端默认不可用 → 未配置
ai.handleMessage({ type: 'ai_config', config: { provider: 'deepseek', model: '', baseUrl: '', hasKey: false } });
check('服务端无 Key 且本机无 Key → 未配置', ai.isConfigured() === false);
check('空配置文案含未配置', /未配置/.test(ai.statusText()), ai.statusText());

// 5. 本机保存 Key (记住) → 本机独立可用, 不经服务端
ai.save({ config: { provider: 'deepseek', model: 'deepseek-flash', baseUrl: '', remember: true }, apiKey: 'sk-1' });
check('保存后已配置', ai.isConfigured() === true);
check('来源标记为 client', ai.get().source === 'client');
check('状态文案含本机 Key', /本机 Key/.test(ai.statusText()), ai.statusText());
check('toRequest 携带 Key', ai.toRequest().apiKey === 'sk-1', JSON.stringify(ai.toRequest()));
check('Key 写入 localStorage', store.get('fy_ai_key') === 'sk-1');
check('配置写入 localStorage', /deepseek-flash/.test(store.get('fy_ai_cfg') || ''));

// 6. 本机 Key 优先于服务端默认
ai.handleMessage({ type: 'ai_config', config: { provider: 'deepseek', model: 'srv-model', baseUrl: '', hasKey: true, source: 'server' } });
check('本机 Key 优先于服务端默认', ai.get().source === 'client' && ai.toRequest().apiKey === 'sk-1');

// 7. 不记住 → 仅内存, 浏览器不落 Key (刷新即失)
const store2 = new Map();
const { ai: aiB, store: stB } = load(store2);
aiB.save({ config: { provider: 'deepseek', model: 'm', remember: false }, apiKey: 'sk-temp' });
check('不记住时内存可用', aiB.toRequest().apiKey === 'sk-temp');
check('不记住时不写 Key 到 localStorage', !stB.has('fy_ai_key'), String(stB.get('fy_ai_key')));
check('不记住时配置仍写入 (便于下次显示)', /"model":"m"/.test(stB.get('fy_ai_cfg') || ''));
// 模拟刷新: 同一 store 重新载入
const { ai: aiB2 } = load(stB);
aiB2.handleMessage({ type: 'ai_providers', providers: PROVIDERS });
aiB2.handleMessage({ type: 'ai_config', config: { provider: '', model: '', hasKey: false } });
check('刷新后 Key 失效', aiB2.isConfigured() === false && aiB2.get().hasKey === false);

// 8. 记住 → 刷新后仍在
const { ai: aiC, store: stC } = load();
aiC.save({ config: { provider: 'deepseek', model: 'm', remember: true }, apiKey: 'sk-persist' });
const { ai: aiC2, store: stC2 } = load(stC);
check('记住时刷新后 Key 保留', aiC2.toRequest().apiKey === 'sk-persist', JSON.stringify(aiC2.toRequest()));
aiC2.handleMessage({ type: 'ai_providers', providers: PROVIDERS });
aiC2.handleMessage({ type: 'ai_config', config: { provider: '', hasKey: false } });
check('记住时刷新后已配置', aiC2.isConfigured() === true);

// 9. 空 Key 保存不覆盖已有 Key
aiC2.save({ config: { provider: 'deepseek', model: 'm2', remember: true }, apiKey: '' });
check('空 Key 保留原 Key', aiC2.toRequest().apiKey === 'sk-persist');
check('空 Key 更新了 model', aiC2.get().model === 'm2');

// 10. 清除 Key → 内存与 localStorage 都清
aiC2.clearKey();
check('清除后内存无 Key', aiC2.get().hasKey === false && !('apiKey' in aiC2.toRequest()));
check('清除后 localStorage 无 Key', !stC2.has('fy_ai_key'));
aiC2.handleMessage({ type: 'ai_config', config: { provider: '', hasKey: false } });
check('清除后未配置', aiC2.isConfigured() === false);

// 11. 本机配置不完整 → 未填 Key 提示
aiC2.save({ config: { provider: 'deepseek', model: 'm', remember: false }, apiKey: '' });
check('有服务商无 Key → 未配置', aiC2.isConfigured() === false);
check('状态提示未填 Key', /未填 Key/.test(aiC2.statusText()), aiC2.statusText());

// 12. 可免 Key 服务商 → 无需 Key 即可用
aiC2.save({ config: { provider: 'ollama', model: 'qwen3:8b', baseUrl: 'http://127.0.0.1:11434/v1', remember: false }, apiKey: '' });
check('免 Key 服务商已配置', aiC2.isConfigured() === true);
check('免 Key 时不带 apiKey 字段', !('apiKey' in aiC2.toRequest()));
check('免 Key 状态文案', /免 Key/.test(aiC2.statusText()), aiC2.statusText());

// 13. subscribe 立即触发并随保存触发
let hits = 0;
aiC2.subscribe(() => { hits++; });
aiC2.save({ config: { provider: 'deepseek', model: 'm', remember: true }, apiKey: 'sk-s' });
check('subscribe 被触发', hits >= 2, String(hits));

console.log(failures === 0 ? '\nAI 配置客户端单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
