// 顺序执行全部测试并汇总
//   node test/run_tests.js
import { spawnSync } from 'child_process';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');

const STEPS = [
  ['语法检查 (server/lib/public)', 'node', ['-e', `
    const {execFileSync}=require('child_process');
    const files=['server.js','lib/mavlink.js','lib/paramlink.js','lib/mavlink_defs.js','lib/fcops.js',
                 'lib/ai.js','lib/faultai.js','lib/thrustai.js','lib/faultstore.js','lib/aiconfigstore.js','lib/ulog.js','lib/loganalysis.js',
                 'public/app.js','public/thrustcheck.js','public/onetouch.js','public/onetouch_data.js','public/onetouch_px4.js','public/datatools.js','public/charts.js','public/fault.js','public/aiconfig.js',
                 'public/fclink.js','public/vendor/buffer-shim.js'];
    for(const f of files) execFileSync(process.execPath,['--check',f],{stdio:'inherit'});
    console.log('  语法 OK');
  `]],
  ['前端静态检查', 'node', ['test/frontend_check.js']],
  ['Buffer Shim 单测', 'node', ['test/buffershim.test.js']],
  ['Golden 方案比对', 'node', ['test/golden_compare.js']],
  ['.bin 解析对拍', 'node', ['test/binlog.test.js']],
  ['.ulg 解析单测', 'node', ['test/ulog.test.js']],
  ['PX4 方案派生单测', 'node', ['test/px4_onetouch.test.js']],
  ['图表渲染单测', 'node', ['test/charts.test.js']],
  ['故障存储单测', 'node', ['test/fault_store.test.js']],
  ['故障 AI Prompt 单测', 'node', ['test/faultai.test.js']],
  ['推力线性化体检单测', 'node', ['test/thrust.test.js']],
  ['AI 配置存储单测', 'node', ['test/aiconfigstore.test.js']],
  ['AI 配置客户端单测', 'node', ['test/aiconfig.test.js']],
  ['故障速查 / AI 配置 WS 集成', 'node', ['test/fault_ws.test.js']],
  ['MAVLink 双向对拍', 'bash', ['-c',
    'node test/mavlink_roundtrip.js gen && python3 test/pymavlink_roundtrip.py gen && python3 test/pymavlink_roundtrip.py check && node test/mavlink_roundtrip.js check']],
  ['ParamLink 单测', 'node', ['test/paramlink.test.js']],
  ['PX4 集成 (进程内模拟器)', 'node', ['test/px4.test.js']],
  ['端到端集成 (pty 模拟飞控)', 'node', ['test/e2e.test.js']],
];

let failed = 0;
for (const [name, cmd, args] of STEPS) {
  process.stdout.write(`\n=== ${name} ===\n`);
  const res = spawnSync(cmd, args, { cwd: ROOT, stdio: 'inherit', env: process.env, shell: false });
  if (res.status !== 0) {
    failed++;
    console.log(`[失败] ${name} (exit=${res.status})`);
  }
}

console.log(`\n${failed === 0 ? '全部测试通过' : failed + ' 个测试步骤失败'}`);
process.exit(failed === 0 ? 0 : 1);
