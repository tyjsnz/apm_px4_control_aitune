// 顺序执行全部测试并汇总
//   node test/run_tests.js
import { spawnSync } from 'child_process';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const SIM = path.join(ROOT, 'sim');

const STEPS = [
  ['语法检查 (server/lib/public)', 'node', ['-e', `
    const {execFileSync}=require('child_process');
    const files=['server.js','lib/mavlink.js','lib/paramlink.js','lib/mavlink_defs.js','lib/fcops.js',
                 'lib/ai.js','lib/faultai.js','lib/thrustai.js','lib/faultstore.js','lib/ulog.js','lib/loganalysis.js','lib/stats.js',
                 'public/app.js','public/monitor.js','public/sensors.js','public/thrustcheck.js','public/onetouch.js','public/onetouch_data.js','public/onetouch_px4.js','public/motortest.js','public/datatools.js','public/charts.js','public/fault.js','public/faultdb.js','public/aiconfig.js','public/stats.js','public/takeoffcheck.js',
                 'public/fclink.js','public/vendor/buffer-shim.js','public/theme.js',
                 'guide/web/app.js','guide/web/canvas.js','guide/web/plot.js','guide/web/panel.js','guide/web/md.js','guide/web/docs.js',
                 'guide/web/core/guidance.js','guide/web/core/simulation.js'];
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
  ['警告速查库单测', 'node', ['test/faultdb.test.js']],
  ['故障 AI Prompt 单测', 'node', ['test/faultai.test.js']],
  ['推力线性化体检单测', 'node', ['test/thrust.test.js']],
  ['传感器体检单测', 'node', ['test/sensors.test.js']],
  ['起飞就绪检查单测', 'node', ['test/takeoffcheck.test.js']],
  ['AI 参数解析单测 (Key 归属)', 'node', ['test/airesolve.test.js']],
  ['AI 配置客户端单测', 'node', ['test/aiconfig.test.js']],
  ['故障速查 / AI 配置 WS 集成', 'node', ['test/fault_ws.test.js']],
  ['AI 代理集成 (本机 Key / 服务端护栏)', 'node', ['test/ai_proxy.test.js']],
  ['使用统计单测', 'node', ['test/stats.test.js']],
  ['使用统计 WS 集成', 'node', ['test/stats_ws.test.js']],
  ['MAVLink 双向对拍', 'node', ['-e', `
    const {spawnSync}=require('child_process');
    const ok=(e)=>{try{return spawnSync(e,['--version'],{stdio:'ignore'}).status===0}catch{return false}};
    const PY=(process.platform==='win32'?['python','python3']:['python3','python']).find(ok);
    if(!PY){console.error('找不到可用的 python/python3');process.exit(1)}
    const steps=[
      ['node',['test/mavlink_roundtrip.js','gen']],
      [PY,['test/pymavlink_roundtrip.py','gen']],
      [PY,['test/pymavlink_roundtrip.py','check']],
      ['node',['test/mavlink_roundtrip.js','check']],
    ];
    for(const [cmd,args] of steps){
      const r=spawnSync(cmd,args,{stdio:'inherit'});
      if(r.status!==0) process.exit(r.status===null?1:r.status);
    }
  `]],
  ['ParamLink 单测', 'node', ['test/paramlink.test.js']],
  ['PX4 集成 (进程内模拟器)', 'node', ['test/px4.test.js']],
  ['FPV 仿真台单测', 'node', ['-e', `
    const {spawnSync}=require('child_process');
    const cwd=${JSON.stringify(SIM)};
    for(const f of ['test/selftest.js','test/simtest.js','test/contract.js']){
      const r=spawnSync(process.execPath,[f],{stdio:'inherit',cwd});
      if(r.status!==0) process.exit(r.status===null?1:r.status);
    }
  `]],
  ['FPV 仿真台挂载集成', 'node', ['test/sim_mount.test.js']],
  ['导引仿真数值对拍 (JS↔Python)', 'node', ['test/guide_core.test.js']],
  ['导引实验室静态检查', 'node', ['test/guide_dom.test.js']],
  ['导引实验室挂载集成', 'node', ['test/guide_mount.test.js']],
  ['端到端集成 (pty 模拟飞控)', 'node', ['test/e2e.test.js']],
];

let failed = 0;
for (const [name, cmd, args, cwd] of STEPS) {
  process.stdout.write(`\n=== ${name} ===\n`);
  const res = spawnSync(cmd, args, { cwd: cwd || ROOT, stdio: 'inherit', env: process.env, shell: false });
  if (res.status !== 0) {
    failed++;
    console.log(`[失败] ${name} (exit=${res.status})`);
  }
}

console.log(`\n${failed === 0 ? '全部测试通过' : failed + ' 个测试步骤失败'}`);
process.exit(failed === 0 ? 0 : 1);
