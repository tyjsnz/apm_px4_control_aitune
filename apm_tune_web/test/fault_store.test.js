// 故障记录存储单测: lib/faultstore.js
//   node test/fault_store.test.js
import fs from 'fs';
import os from 'os';
import path from 'path';
import {
  createFaultStore, normalizeFault, safeFile, FAULT_SEVERITIES, FAULT_STATUSES,
} from '../lib/faultstore.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'faultstore-'));
const store = createFaultStore(dir);

// 1. 空目录
check('空目录列表为空', store.list().length === 0);

// 2. 新建
const r1 = store.save({ title: '解锁时电机不转', symptom: '推油门无反应', severity: 'high', status: 'open', tags: '电调, 解锁' });
check('保存返回文件名', /^fault_[\w.-]+\.json$/.test(r1.file), r1.file);
check('文件名通过 safeFile', safeFile(r1.file) === r1.file);
const list1 = store.list();
check('列表含 1 条', list1.length === 1 && list1[0].file === r1.file, JSON.stringify(list1.map((x) => x.file)));
check('标签被拆分', Array.isArray(list1[0].tags) && list1[0].tags.length === 2, JSON.stringify(list1[0].tags));
check('严重度合法', FAULT_SEVERITIES.includes(list1[0].severity));
check('状态合法', FAULT_STATUSES.includes(list1[0].status));
check('含时间戳', !!list1[0].createdAt && !!list1[0].updatedAt);

// 3. 更新同一条(不应新增)
const r2 = store.save({ file: r1.file, title: '解锁时电机不转(已定位)', severity: 'mid', status: 'fixed', aiAnalysis: 'AI 报告内容' });
const list2 = store.list();
check('更新不新增记录', list2.length === 1 && r2.file === r1.file, JSON.stringify(list2.map((x) => x.file)));
const rec2 = store.get(r1.file);
check('更新写入新标题', rec2.title === '解锁时电机不转(已定位)', rec2.title);
check('更新保留 createdAt', rec2.createdAt === list1[0].createdAt);
check('AI 报告已存储', rec2.aiAnalysis === 'AI 报告内容');
check('id 稳定', rec2.id === list1[0].id);

// 4. 非法字段清理
const r3 = store.save({ title: 'x', severity: 'bogus', status: 'bogus' });
const rec3 = store.get(r3.file);
check('非法严重度回退 mid', rec3.severity === 'mid', rec3.severity);
check('非法状态回退 open', rec3.status === 'open', rec3.status);

// 5. 排序(按 updatedAt 倒序)
check('列表按更新时间倒序', store.list()[0].file === r3.file, JSON.stringify(store.list().map((x) => x.file)));

// 6. 路径穿越防护
check('safeFile 拒绝穿越', safeFile('../evil.json') === null && safeFile('/etc/passwd') === null);
check('get 拒绝穿越', store.get('../evil.json') === null);
check('remove 拒绝穿越', store.remove('../../x.json') === false);

// 7. 删除
check('删除成功', store.remove(r3.file) === true);
check('删除后仅剩 1 条', store.list().length === 1, String(store.list().length));
check('重复删除返回 false', store.remove(r3.file) === false);

// 8. normalizeFault 独立可用
const n = normalizeFault({ title: '', tags: ['a', 'b'] });
check('空标题用默认名', n.title === '未命名故障');
check('超长字段被截断', normalizeFault({ title: 'x'.repeat(300) }).title.length === 200);

fs.rmSync(dir, { recursive: true, force: true });

console.log(failures === 0 ? '\n故障存储单测通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
