// 全局 AI 配置存储单测: lib/aiconfigstore.js
//   node test/aiconfigstore.test.js
import fs from 'fs';
import os from 'os';
import path from 'path';
import { createAiConfigStore } from '../lib/aiconfigstore.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'aiconfig-'));
const file = path.join(dir, 'ai_config.json');

// 1. 初始
let s = createAiConfigStore(file);
check('初始为空', s.getPublic().provider === '' && s.getPublic().hasKey === false);
check('初始文件未创建(读取不报错)', !fs.existsSync(file));

// 2. 不记住 Key -> provider/model 落盘, Key 不落盘
s.set({ provider: 'deepseek', model: 'deepseek-flash', baseUrl: 'https://x/v1', remember: false, apiKey: 'sk-secret' });
check('内存持有 Key', s.get().apiKey === 'sk-secret');
check('对外 hasKey=true', s.getPublic().hasKey === true);
check('对外不含明文 Key', !('apiKey' in s.getPublic()));
const raw1 = JSON.parse(fs.readFileSync(file, 'utf-8'));
check('文件已写 provider/model/baseUrl', raw1.provider === 'deepseek' && raw1.model === 'deepseek-flash' && raw1.baseUrl === 'https://x/v1');
check('不记住时文件不含 Key', !('apiKey' in raw1), JSON.stringify(raw1));

// 3. 重启(重新加载) -> Key 丢失, 其它保留
s = createAiConfigStore(file);
check('重载后 Key 丢失', s.get().apiKey === '' && s.getPublic().hasKey === false);
check('重载后 provider/model 保留', s.getPublic().provider === 'deepseek' && s.getPublic().model === 'deepseek-flash');

// 4. 记住 Key -> 落盘, 重载保留
s.set({ provider: 'deepseek', model: 'deepseek-flash', remember: true, apiKey: 'sk-saved' });
const raw2 = JSON.parse(fs.readFileSync(file, 'utf-8'));
check('记住时文件含 Key', raw2.apiKey === 'sk-saved' && raw2.remember === true, JSON.stringify(raw2));
s = createAiConfigStore(file);
check('重载后 Key 保留', s.get().apiKey === 'sk-saved');
check('重载后 hasKey=true', s.getPublic().hasKey === true);
check('resolve 返回全局配置', s.resolve({}).provider === 'deepseek' && s.resolve({}).apiKey === 'sk-saved');
check('resolve 可被请求覆盖', s.resolve({ provider: 'openai', model: 'x' }).provider === 'openai'
  && s.resolve({ provider: 'openai' }).apiKey === 'sk-saved');

// 5. 清除 Key
s.clearKey();
check('清除后内存无 Key', s.get().apiKey === '' && s.getPublic().hasKey === false);
const raw3 = JSON.parse(fs.readFileSync(file, 'utf-8'));
check('清除后文件无 Key', !('apiKey' in raw3));
s = createAiConfigStore(file);
check('重载后仍无 Key', s.get().apiKey === '');

// 6. 未提供 apiKey 的 set 不应清空已有 Key
s.set({ provider: 'deepseek', model: 'deepseek-flash', remember: true, apiKey: 'sk-keep' });
s.set({ model: 'deepseek-v4-pro' }); // 不含 apiKey
check('未提供 Key 时保留原 Key', s.get().apiKey === 'sk-keep', s.get().apiKey);
check('set 更新了 model', s.getPublic().model === 'deepseek-v4-pro');

// 7. 从记住 -> 不记住: 文件移除 Key, 内存保留
s.set({ remember: false });
check('转不记住后文件无 Key', !('apiKey' in JSON.parse(fs.readFileSync(file, 'utf-8'))));
check('转不记住后内存仍保留(本次会话可用)', s.get().apiKey === 'sk-keep');

// 8. 损坏文件容错
fs.writeFileSync(file, '{ not json');
const s2 = createAiConfigStore(file);
check('损坏文件不抛错且为空', s2.getPublic().provider === '');

fs.rmSync(dir, { recursive: true, force: true });
console.log(failures === 0 ? '\n全局 AI 配置存储单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
