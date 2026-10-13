// AI 参数解析单测: lib/ai.js resolveAi / isKeyOptional
//   覆盖 B 方案核心规则: 用户自带 Key 优先 → 服务端 env 默认兜底 → 护栏(服务端 Key 不改道)
//   node test/airesolve.test.js
import { resolveAi, isKeyOptional } from '../lib/ai.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

// ---------- isKeyOptional ----------
check('deepseek 需要 Key', isKeyOptional('deepseek') === false);
check('ollama 可免 Key', isKeyOptional('ollama') === true);
check('未知服务商按需 Key 处理', isKeyOptional('nope') === false);
check('空服务商不算免 Key', isKeyOptional('') === false);

const SRV = { provider: 'deepseek', model: 'srv-model', baseUrl: 'https://srv.example/v1', apiKey: 'sk-server' };

// 1. 完全没配置 → 空结果 (由 aiReady 拦截提示)
let r = resolveAi({}, {});
check('双空返回空配置', r.provider === '' && r.apiKey === '' && r.source === '', JSON.stringify(r));

// 2. 客户端自带 Key → 完全用客户端 (含自定义 baseUrl)
r = resolveAi(
  { provider: 'openai', model: 'gpt-x', baseUrl: 'https://my-proxy/v1', apiKey: 'sk-mine' },
  SRV,
);
check('自带 Key 时客户端优先', r.source === 'client' && r.apiKey === 'sk-mine' && r.provider === 'openai',
  JSON.stringify(r));
check('自带 Key 时使用客户端 baseUrl', r.baseUrl === 'https://my-proxy/v1');

// 3. 客户端为免 Key 服务商 → 无需 Key 也用客户端
r = resolveAi({ provider: 'ollama', model: 'qwen3:8b', baseUrl: 'http://127.0.0.1:11434/v1' }, SRV);
check('免 Key 服务商走客户端', r.source === 'client' && r.apiKey === '' && r.provider === 'ollama',
  JSON.stringify(r));

// 4. 客户端只填了服务商没填 Key → 服务端默认兜底
r = resolveAi({ provider: 'deepseek', model: 'local-model', baseUrl: 'https://attacker/v1' }, SRV);
check('缺 Key 时回退服务端默认', r.source === 'server' && r.apiKey === 'sk-server', JSON.stringify(r));
check('护栏: 忽略客户端 baseUrl', r.baseUrl === SRV.baseUrl, r.baseUrl);
check('护栏: 忽略客户端 model', r.model === SRV.model, r.model);

// 5. 完全空请求 + 服务端有默认 → 服务端兜底
r = resolveAi({}, SRV);
check('空请求使用服务端默认', r.source === 'server' && r.provider === 'deepseek', JSON.stringify(r));

// 6. 服务端默认不完整 (只填 provider 没 key) → 不可用, 回退客户端
r = resolveAi({ provider: 'deepseek', model: 'm' }, { provider: 'deepseek', model: 'm', apiKey: '' });
check('服务端默认缺 Key 不启用', r.source === 'client' && r.apiKey === '', JSON.stringify(r));

// 7. 服务端默认为免 Key 服务商 (无 key 也可用)
r = resolveAi({}, { provider: 'ollama', model: 'qwen3:8b', baseUrl: 'http://server:11434/v1', apiKey: '' });
check('服务端免 Key 服务商可兜底', r.source === 'server' && r.provider === 'ollama', JSON.stringify(r));

// 8. 客户端与服务端都不可用 → 空 (走"未配置"提示)
r = resolveAi({ provider: 'deepseek', model: 'm' }, { provider: 'deepseek', model: '', apiKey: 'sk-x' });
check('双方均不可用时返回客户端空 Key', r.source === 'client' && r.apiKey === '', JSON.stringify(r));

// 9. 无 Key 且非免 Key 服务商, 但服务端可用 → 服务端 Key 绝不外泄到客户端指定地址
r = resolveAi({ provider: 'deepseek', model: 'm', baseUrl: 'https://evil.example/collect', apiKey: '' },
  { provider: 'deepseek', model: 'm', baseUrl: 'https://api.deepseek.com', apiKey: 'sk-server' });
check('防盗取: 服务端 Key 只发服务端地址', r.baseUrl === 'https://api.deepseek.com' && r.source === 'server',
  JSON.stringify(r));

console.log(failures === 0 ? '\nAI 参数解析单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
