// 起飞就绪检查单测: public/takeoffcheck.js (vm 沙箱) onMsg + evaluate
//   node test/takeoffcheck.test.js
import fs from 'fs';
import path from 'path';
import vm from 'vm';
import { fileURLToPath } from 'url';
import { FcOps } from '../lib/fcops.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');

let failures = 0;
function check(cond, msg) {
  if (cond) { console.log('  ok  ' + msg); } else { console.log('  FAIL ' + msg); failures++; }
}
function hasReason(r, frag) {
  return r.reasons.some((x) => x.text.indexOf(frag) >= 0);
}
function hasNote(r, frag) {
  return r.notes.some((x) => x.indexOf(frag) >= 0);
}

// ---------- vm 沙箱加载 ----------
const sandbox = {
  window: {}, console, Math, Number, isFinite, Date, JSON, setTimeout, clearTimeout,
  document: { readyState: 'loading', addEventListener() {}, getElementById() { return null; } },
};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', 'takeoffcheck.js'), 'utf8'), sandbox);
const TC = sandbox.window.TakeoffCheck;
check(!!TC && typeof TC.onMsg === 'function', '加载 window.TakeoffCheck');
check(typeof TC.evaluate === 'function', 'evaluate 存在');

const connected = () => TC.onMsg({ type: 'connected', path: 'COM1', baud: 115200 });
const hb = (o = {}) => TC.onMsg({ type: 'heartbeat', flightMode: o.mode || 'STABILIZE', armed: !!o.armed, firmware: o.fw || 'apm', autopilot: 3, baseMode: 0, customMode: 0 });
const gps = (fixType, satellites, eph) => TC.onMsg({ type: 'gps_status', fixType, satellites, eph });
const sys = (present, enabled, health, extra = {}) => TC.onMsg({
  type: 'sys_status', sensorsPresent: present, sensorsEnabled: enabled, sensorsHealth: health,
  voltageBattery: extra.v !== undefined ? extra.v : 15200,
  batteryRemaining: extra.rem === undefined ? 80 : extra.rem, load: 25,
});
const ekf = (flags, posHorizVariance) => TC.onMsg({ type: 'ekf_status', flags, posHorizVariance, velocityVariance: 0.1 });
const statustext = (text) => TC.onMsg({ type: 'statustext', severity: 6, text });

// ---------- 1. 初始 offline ----------
let r = TC.evaluate();
check(r.state === 'offline', `初始 state=offline (实际 ${r.state})`);

// ---------- 2. 已连接但无数据 → 阻断 ----------
connected();
r = TC.evaluate();
check(r.state === 'blocked', `连接无数据 → blocked (实际 ${r.state})`);
check(hasReason(r, '心跳'), '缺心跳原因');
check(hasReason(r, 'GPS'), '缺 GPS 原因');

// ---------- 3. 心跳 + GPS 无三维定位 ----------
hb();
gps(1, 4, 9999);
r = TC.evaluate();
check(r.state === 'blocked', `fix=1 → blocked (实际 ${r.state})`);
check(hasReason(r, '未完成三维定位'), 'fix=1 原因文案');
check(hasReason(r, '4 星'), '卫星数并入文案');

// ---------- 4. GPS 三维定位, 无 sys/ekf → pending (不误报) ----------
gps(3, 14, 120);
r = TC.evaluate();
check(r.state === 'pending', `fix3/14星 无 prearm → pending (实际 ${r.state})`);
check(hasNote(r, '尚未执行主动 PreArm'), '提示建议主动检测');

// ---------- 5. 主动检测通过 → ready ----------
TC.onMsg({ type: 'prearm_check_done', ok: true, result: 0, timeout: false, texts: ['PreArm: all checks passing'] });
r = TC.evaluate();
check(r.state === 'ready', `检测通过 → ready (实际 ${r.state})`);
check(hasNote(r, '通过'), 'notes 含通过结论');

// ---------- 6. 被动 PreArm 失败 → blocked + FaultDB 对照 ----------
sandbox.window.FaultDB = {
  search(q) {
    if (/compass/i.test(q)) return [{ zh: '罗盘不健康', en: 'Compass not healthy', fix: '重新校准罗盘\n室外重试' }];
    return [];
  },
};
statustext('PreArm: Compass not healthy');
r = TC.evaluate();
check(r.state === 'blocked', `PreArm 失败 → blocked (实际 ${r.state})`);
check(hasReason(r, '飞控预检未通过: Compass not healthy'), '失败原因含原文');
const hintR = r.reasons.find((x) => x.text.indexOf('Compass') >= 0);
check(!!hintR && /罗盘不健康/.test(hintR.hint || ''), 'FaultDB 中文对照注入 hint');

// ---------- 7. PreArm 通过文本清失败缓存 ----------
statustext('PreArm: all checks passing');
r = TC.evaluate();
check(r.state === 'ready', `通过文本清除失败 → ready (实际 ${r.state})`);

// ---------- 8. sys_status 传感器不健康 ----------
sys(47, 47, 43);  // 磁罗盘 bit4 未通过
r = TC.evaluate();
check(r.state === 'blocked', `磁罗盘不健康 → blocked (实际 ${r.state})`);
check(hasReason(r, '三轴磁罗盘'), '磁罗盘原因');
sys(47, 47, 47);

// ---------- 9. PREARM_CHECK 状态位 ----------
sys(47 | 268435456, 47 | 268435456, 47);   // present/enabled 位开, healthy 位关
r = TC.evaluate();
check(hasReason(r, 'PreArm 状态位'), 'SYS_STATUS PREARM_CHECK 位阻断');
sys(47, 47, 47);

// ---------- 10. 电池电压未上报 → note ----------
sys(47, 47, 47, { v: 0 });
r = TC.evaluate();
check(hasNote(r, '电池电压未上报'), '电压 0 → 提示 note');
sys(47, 47, 47);

// ---------- 11. EKF 无水平位置 / 方差过大 ----------
ekf(1, 0.5);       // flags 无 8|16|256|512
r = TC.evaluate();
check(hasReason(r, 'EKF 无水平位置估计'), 'EKF flags 阻断');
ekf(8, 9.5);
r = TC.evaluate();
check(hasReason(r, '方差过大'), 'EKF 方差阻断');
ekf(8, 0.5);

// ---------- 12. 主动检测: 超时/不支持 → 降级 unsupported ----------
r = TC.evaluate();
check(r.state === 'ready', `快照全绿+已通过 → ready (实际 ${r.state})`);
TC.onMsg({ type: 'prearm_check_done', ok: false, result: null, timeout: true, texts: [] });
r = TC.evaluate();
check(r.state === 'ready', `超时降级不阻断 → ready (实际 ${r.state})`);
check(hasNote(r, '未收到检测 ACK'), '超时降级提示');
TC.onMsg({ type: 'prearm_check_done', ok: false, result: 3, timeout: false, texts: [] });
r = TC.evaluate();
check(hasNote(r, '不支持该命令'), 'UNSUPPORTED 降级提示');
check(r.state === 'ready', `UNSUPPORTED 仍 ready (实际 ${r.state})`);

// ---------- 13. 主动检测: 飞控拒绝 → blocked ----------
TC.onMsg({ type: 'prearm_check_done', ok: false, result: 2, timeout: false, texts: [] });
r = TC.evaluate();
check(r.state === 'blocked', `拒绝 result=2 → blocked (实际 ${r.state})`);
check(hasReason(r, '被飞控拒绝'), '拒绝原因文案');

// ---------- 14. 主动检测: ok + texts 失败 → blocked ----------
TC.onMsg({ type: 'prearm_check_done', ok: true, result: 0, timeout: false, texts: ['PreArm: Throttle below failsafe'] });
r = TC.evaluate();
check(r.state === 'blocked', `ok+失败文本 → blocked (实际 ${r.state})`);
check(hasReason(r, 'Throttle below failsafe'), '失败文本进原因');

// ---------- 15. 主动检测: ok + 仅通过文本 → ready ----------
TC.onMsg({ type: 'prearm_check_done', ok: true, result: 0, timeout: false, texts: ['PreArm: all checks passing'] });
r = TC.evaluate();
check(r.state === 'ready', `ok+通过文本 → ready (实际 ${r.state})`);

// ---------- 16. 已解锁 → armed (不判起飞) ----------
hb({ armed: true, mode: 'GUIDED' });
r = TC.evaluate();
check(r.state === 'armed' && r.mode === 'GUIDED', `armed 态 (实际 ${r.state}/${r.mode})`);
check(r.reasons.length === 0, 'armed 无原因列表');

// ---------- 17. PX4 → px4 态 ----------
TC.reset();
connected();
hb({ fw: 'px4' });
r = TC.evaluate();
check(r.state === 'px4', `px4 → px4 (实际 ${r.state})`);
check(hasNote(r, 'QGroundControl'), 'px4 提示 QGC');

// ---------- 18. 断连 → offline 复位 ----------
TC.onMsg({ type: 'disconnected' });
r = TC.evaluate();
check(r.state === 'offline', `断连复位 → offline (实际 ${r.state})`);
check(TC._state.prearmFail === null && TC._state.gps === null, '断连清快照');

// ---------- 19. 检测窗 busy 提示 ----------
connected(); hb(); gps(3, 14, 120); sys(47, 47, 47);
TC.onMsg({ type: 'prearm_check_done', ok: false, busy: true, reason: '上一次检测尚未结束' });
r = TC.evaluate();
check(hasNote(r, '上一次检测尚未结束'), 'busy 提示进 notes');

// ---------- 20. evaluate 无 DOM 依赖 ----------
check(typeof sandbox.document.getElementById === 'function' && TC.evaluate().state !== undefined, 'evaluate 可在无渲染环境运行');

// ========== FcOps prearm_check 后端 (lib/fcops.js) ==========
// 21. PX4 拒绝执行 (链路已连, 但固件为 px4)
{
  const em = [];
  const fc = new FcOps({ emit: (m) => em.push(m) });
  fc.telemetry.firmware = 'px4';
  fc.attached = true;
  fc.link = { sendCommand: async () => { throw new Error('PX4 不应发送 CMD401'); } };
  await fc.handle({ type: 'prearm_check' });
  const done = em.find((m) => m.type === 'prearm_check_done');
  check(done && done.unsupported === true, 'PX4 → prearm_check_done.unsupported');
}

// 22. 未连接守卫
{
  const em = [];
  const fc = new FcOps({ emit: (m) => em.push(m) });
  await fc.handle({ type: 'prearm_check' });
  check(em.some((m) => m.type === 'error' && /未连接/.test(m.message || '')), '未连接 → error 提示');
}

// 23. 窗口流程: CMD401 → STATUSTEXT 收集 → 4.5s 后统一 emit + busy 防重入
{
  const em = [];
  const fc = new FcOps({ emit: (m) => em.push(m) });
  fc.telemetry.firmware = 'apm';
  fc.attached = true;
  let sentCmd = null;
  fc.link = {
    sendCommand: async (cmd, params, opts) => {
      sentCmd = { cmd, params, opts };
      return { ok: true, result: 0, text: [] };
    },
  };
  await fc.handle({ type: 'prearm_check' });
  check(sentCmd && sentCmd.cmd === 401, `发送 CMD=401 (实际 ${sentCmd && sentCmd.cmd})`);
  check(sentCmd && sentCmd.opts.timeout > 3000, 'sendCommand 超时在窗口内');
  // 模拟飞控预检期间打印失败文本 (走真实 handleMavlink 路径)
  fc.handleMavlink({ name: 'STATUSTEXT', fields: { severity: 4, text: 'PreArm: Compass not healthy' } });
  // 重入 → busy
  await fc.handle({ type: 'prearm_check' });
  const busy = em.filter((m) => m.type === 'prearm_check_done').pop();
  check(busy && busy.busy === true, '检测中重入 → busy done');
  await new Promise((r) => setTimeout(r, 4700));
  const done = em.filter((m) => m.type === 'prearm_check_done').pop();
  check(done && done.ok === true && done.result === 0, `窗口结束 emit ok (实际 ok=${done && done.ok} result=${done && done.result})`);
  check(done && done.texts.some((t) => /PreArm: Compass/.test(t)), 'texts 收集到 PreArm 文本');
  check(fc.prearmWin === null, '窗口已清理');
}

console.log(failures === 0 ? '\n起飞就绪单测: 全部通过' : `\n起飞就绪单测: ${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
