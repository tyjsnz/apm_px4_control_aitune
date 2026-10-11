// PX4 ULog 解析器单测: 程序合成 ULog (新旧 P/Q 格式 / 截断尾 / 丢包 / 错误字符串)
//   node test/ulog.test.js
import {
  isUlog, isDataFlash, parseUlog, analyzeUlog, extractUlogCharts, px4NavStateName,
} from '../lib/ulog.js';
import {
  analyzeLogAuto, extractChartsAuto, pickLogSystemPrompt,
} from '../lib/loganalysis.js';

let failures = 0;
function check(cond, msg) {
  if (cond) { console.log('  ok  ' + msg); } else { console.log('  FAIL ' + msg); failures++; }
}

// ================= 合成 ULog 构建器 =================
const parts = [];
function msg(type, payload) {
  const head = Buffer.alloc(3);
  head.writeUInt16LE(payload.length, 0);
  head.writeUInt8(type.charCodeAt(0), 2);
  parts.push(head, payload);
}
const str = (s) => Buffer.from(String(s), 'utf8');

// 'I': key_len + key + value(字符串)
function uInfo(key, value) {
  const k = str(key); const v = str(value);
  const p = Buffer.alloc(1 + k.length + v.length);
  p.writeUInt8(k.length, 0);
  k.copy(p, 1); v.copy(p, 1 + k.length);
  msg('I', p);
}
// 'P' 新格式: key_len + key + f32
function uParamNew(key, value) {
  const k = str(key);
  const p = Buffer.alloc(1 + k.length + 4);
  p.writeUInt8(k.length, 0);
  k.copy(p, 1);
  p.writeFloatLE(value, 1 + k.length);
  msg('P', p);
}
// 'P' 旧格式: char[16] + f32 (消息体恰为 20 字节)
function uParamOld(key, value) {
  const p = Buffer.alloc(20);
  str(key).copy(p, 0, 0, 16);
  p.writeFloatLE(value, 16);
  msg('P', p);
}
// 'Q' 新格式: default_types(1B) + key_len + key + f32
function uDefaultNew(key, value) {
  const k = str(key);
  const p = Buffer.alloc(1 + 1 + k.length + 4);
  p.writeUInt8(2, 0);
  p.writeUInt8(k.length, 1);
  k.copy(p, 2);
  p.writeFloatLE(value, 2 + k.length);
  msg('Q', p);
}
// 'Q' 旧格式: default_types(1B) + char[16] + f32 (消息体恰为 21 字节)
function uDefaultOld(key, value) {
  const p = Buffer.alloc(21);
  p.writeUInt8(2, 0);
  str(key).copy(p, 1, 0, 16);
  p.writeFloatLE(value, 17);
  msg('Q', p);
}
const uFormat = (s) => msg('F', str(s));
function uSub(multiId, msgId, name) {
  const n = str(name);
  const p = Buffer.alloc(3 + n.length);
  p.writeUInt8(multiId, 0);
  p.writeUInt16LE(msgId, 1);
  n.copy(p, 3);
  msg('A', p);
}
function uData(msgId, fieldsBuf) {
  const p = Buffer.alloc(2 + fieldsBuf.length);
  p.writeUInt16LE(msgId, 0);
  fieldsBuf.copy(p, 2);
  msg('D', p);
}
// 'L': level(1B) + timestamp(u64) + text
function uStrMsg(level, us, text) {
  const t = str(text);
  const p = Buffer.alloc(9 + t.length);
  p.writeUInt8(level, 0);
  p.writeBigUInt64LE(BigInt(us), 1);
  t.copy(p, 9);
  msg('L', p);
}
function uDropout(ms) {
  const p = Buffer.alloc(2);
  p.writeUInt16LE(ms, 0);
  msg('O', p);
}

// ---------- 组装文件 ----------
const header = Buffer.alloc(16);
header.write('ULog', 0, 'ascii');
header.writeUInt8(0x01, 4);
header.writeUInt8(0x12, 5);
header.writeUInt8(0x35, 6);
header.writeUInt8(1, 7); // version
header.writeBigUInt64LE(1000000n, 8);

uInfo('ver_hw', 'Pixhawk1');
uInfo('ver_sw', 'v1.14.3');
uParamNew('MPC_XY_CRUISE', 5.0);   // 新格式参数
uParamOld('MC_YAW_P', 2.8);        // 旧格式参数 (size==20)
uDefaultNew('MPC_TKO_SPEED', 1.5);
uDefaultOld('RTL_RETURN_ALT', 60.0);

uFormat('vehicle_status:uint64 timestamp;uint8 nav_state;uint8 arming_state;bool failsafe');
uFormat('vehicle_attitude:uint64 timestamp;float[4] q');
uFormat('vehicle_attitude_setpoint:uint64 timestamp;float roll_body;float pitch_body;float yaw_body');
uFormat('vehicle_imu:uint64 timestamp;float[3] mean_accel');
uFormat('vehicle_angular_velocity:uint64 timestamp;float[3] xyz');
uFormat('vehicle_local_position:uint64 timestamp;float z;float x;float y;float[2] vel');
uFormat('vehicle_gps_position:uint64 timestamp;uint8 satellites_used;uint8 fix_type;float eph;float vel_north;float vel_east');
uFormat('battery_status:uint64 timestamp;float voltage_v;float current_a;float remaining');
uFormat('actuator_motors:uint64 timestamp;float[8] control');
uFormat('estimator_innovations:uint64 timestamp;float[6] vel_pos_innov');

uSub(0, 1, 'vehicle_status');
uSub(0, 2, 'vehicle_attitude');
uSub(0, 3, 'vehicle_attitude_setpoint');
uSub(0, 4, 'vehicle_imu');
uSub(0, 5, 'vehicle_angular_velocity');
uSub(0, 6, 'vehicle_local_position');
uSub(0, 7, 'vehicle_gps_position');
uSub(0, 8, 'battery_status');
uSub(0, 9, 'actuator_motors');
uSub(0, 10, 'estimator_innovations');

const N = 30; // 10Hz × 30 帧 → 2.9s
for (let i = 0; i < N; i++) {
  const us = BigInt(1e6 + i * 100000);

  { // MANUAL → AUTO_MISSION → AUTO_RTL; 解锁一次; 第 25 帧一次 failsafe
    const p = Buffer.alloc(11);
    p.writeBigUInt64LE(us, 0);
    p.writeUInt8(i < 10 ? 0 : (i < 22 ? 3 : 5), 8);
    p.writeUInt8(i < 5 ? 1 : 2, 9);
    p.writeUInt8(i === 25 ? 1 : 0, 10);
    uData(1, p);
  }
  { // 小姿态: q≈[1, 0.02, 0.01, 0] → roll≈2.3°
    const p = Buffer.alloc(24);
    p.writeBigUInt64LE(us, 0);
    p.writeFloatLE(1, 8); p.writeFloatLE(0.02, 12); p.writeFloatLE(0.01, 16); p.writeFloatLE(0, 20);
    uData(2, p);
  }
  { // 设定姿态 roll_body=0.03 → 跟踪误差存在
    const p = Buffer.alloc(20);
    p.writeBigUInt64LE(us, 0);
    p.writeFloatLE(0.03, 8); p.writeFloatLE(0.02, 12); p.writeFloatLE(0.5, 16);
    uData(3, p);
  }
  { // 振动: |a|≈11.3 → 偏离 1g ≈1.5 m/s²
    const p = Buffer.alloc(20);
    p.writeBigUInt64LE(us, 0);
    p.writeFloatLE(0.2, 8); p.writeFloatLE(0.1, 12); p.writeFloatLE(11.3, 16);
    uData(4, p);
  }
  {
    const p = Buffer.alloc(20);
    p.writeBigUInt64LE(us, 0);
    p.writeFloatLE(0.1, 8); p.writeFloatLE(0.2, 12); p.writeFloatLE(0.05, 16);
    uData(5, p);
  }
  { // 爬升: z=-0.2i → 最大相对高度 5.8m; vel=[3,4] → 地速 5
    const p = Buffer.alloc(28);
    p.writeBigUInt64LE(us, 0);
    p.writeFloatLE(-0.2 * i, 8);
    p.writeFloatLE(0.05 * i, 12);
    p.writeFloatLE(0, 16);
    p.writeFloatLE(3, 20); p.writeFloatLE(4, 24);
    uData(6, p);
  }
  {
    const p = Buffer.alloc(23);
    p.writeBigUInt64LE(us, 0);
    p.writeUInt8(12, 8); p.writeUInt8(6, 9);
    p.writeFloatLE(0.8, 10); p.writeFloatLE(3, 14); p.writeFloatLE(4, 18);
    uData(7, p);
  }
  { // 电压 25.1→24.81, 电流升至 ~18.7A
    const p = Buffer.alloc(20);
    p.writeBigUInt64LE(us, 0);
    p.writeFloatLE(25.1 - 0.01 * i, 8);
    p.writeFloatLE(10 + 0.3 * i, 12);
    p.writeFloatLE(0.9 - 0.005 * i, 16);
    uData(8, p);
  }
  { // 电机 0.45
    const p = Buffer.alloc(40);
    p.writeBigUInt64LE(us, 0);
    const ctrl = [0.45, 0.46, 0.44, 0.45, 0.1, 0.1, 0, 0];
    ctrl.forEach((v, k) => p.writeFloatLE(v, 8 + k * 4));
    uData(9, p);
  }
  { // EKF 新息
    const p = Buffer.alloc(32);
    p.writeBigUInt64LE(us, 0);
    const inn = [0.1, -0.05, 0.2, 0.3, -0.1, 0.05];
    inn.forEach((v, k) => p.writeFloatLE(v, 8 + k * 4));
    uData(10, p);
  }
}

uStrMsg(3, 1200000, 'preflight fail: denied'); // 错误字符串 (level=3)
uDropout(50);

const buf = Buffer.concat([header, ...parts]);

// ================= 解析测试 =================
check(isUlog(buf) === true, 'isUlog 识别合成日志');
check(isUlog(Buffer.from('NOTAULOG1234')) === false, 'isUlog 拒绝非 ULog');
check(isDataFlash(buf) === false, 'isDataFlash 不误判 ULog');
check(isDataFlash(Buffer.from([0xa3, 0x95, 0x00])) === true, 'isDataFlash 识别 DataFlash 头');

const defs = parseUlog(buf, null);
check(defs.version === 1, '版本号 = 1');
check(defs.info.ver_hw === 'Pixhawk1', "info 'ver_hw' = Pixhawk1 -> " + defs.info.ver_hw);
check(defs.info.ver_sw === 'v1.14.3', "info 'ver_sw' = v1.14.3 -> " + defs.info.ver_sw);
check(Math.abs(defs.params.MPC_XY_CRUISE - 5.0) < 1e-6, "参数 新格式 MPC_XY_CRUISE=5 -> " + defs.params.MPC_XY_CRUISE);
check(Math.abs(defs.params.MC_YAW_P - 2.8) < 1e-6, "参数 旧格式 MC_YAW_P=2.8 -> " + defs.params.MC_YAW_P);
check(Math.abs(defs.defaults.MPC_TKO_SPEED - 1.5) < 1e-6, "默认参数 新格式 MPC_TKO_SPEED=1.5 -> " + defs.defaults.MPC_TKO_SPEED);
check(Math.abs(defs.defaults.RTL_RETURN_ALT - 60.0) < 1e-6, "默认参数 旧格式 RTL_RETURN_ALT=60 -> " + defs.defaults.RTL_RETURN_ALT);
check(defs.formats.vehicle_status && defs.formats.vehicle_status.size === 11, 'F 格式 vehicle_status 解析');
check(defs.subs.get(7).name === 'vehicle_gps_position', 'A 订阅 msg_id=7 -> vehicle_gps_position');
check(defs.strings.length === 1 && /preflight fail/.test(defs.strings[0].text), 'L 字符串记录');
check(defs.dropouts === 1 && defs.dropMs === 50, 'O 丢包记录 (1 段 / 50ms)');

// 尾部截断 (崩溃日志): 宣称 200 字节但只剩 10 字节
const truncated = Buffer.concat([buf, (() => {
  const h = Buffer.alloc(3);
  h.writeUInt16LE(200, 0);
  h.writeUInt8('D'.charCodeAt(0), 2);
  return Buffer.concat([h, Buffer.alloc(10, 0xaa)]);
})()]);
let truncFlag = null;
try {
  const r = parseUlog(truncated, null);
  truncFlag = r.truncated;
  check(true, '截断尾不抛异常');
} catch (e) {
  check(false, '截断尾不抛异常: ' + e.message);
}
check(truncFlag === true, '截断尾标记 truncated=true -> ' + truncFlag);
try {
  parseUlog(buf.subarray(0, 10), null);
  check(true, '文件头截断不抛异常');
} catch (e) {
  check(false, '文件头截断不抛异常: ' + e.message);
}

// ================= 指标摘要 =================
const { text, metrics } = analyzeUlog(buf);
check(/PX4 ULog/.test(text), '摘要标题为 PX4 ULog');
check(/AUTO_MISSION/.test(text) && /AUTO_RTL/.test(text), '模式切换解码 (AUTO_MISSION/AUTO_RTL)');
check(/ARMED/.test(text) && /DISARMED/.test(text), '解锁/上锁边沿');
check(/最大相对高度: 5\.8 m/.test(text), '最大相对高度 5.8m -> ' + (text.match(/最大相对高度: [\d.]+ m/) || [])[0]);
check(/最大地速: 5 m\/s/.test(text), '最大地速 5 m/s');
check(/最少卫星数: 12/.test(text), '最少卫星数 12');
check(/最大振动/.test(text), '振动指标输出');
check(/姿态跟踪最大误差/.test(text), '姿态跟踪误差输出');
check(/电池电压/.test(text), '电池指标输出');
check(/EKF 位置\/速度新息 RMS/.test(text), 'EKF 新息 RMS 输出');
check(/记录丢包: 1 段 \/ 共 50 ms/.test(text), '丢包统计进入摘要');
check(/failsafe 1 次/.test(text), 'failsafe 边沿计 1 次');
check(metrics.total === N * 10, `数据条数 ${metrics.total} == ${N * 10}`);
check(metrics.duration > 2.8 && metrics.duration < 3.0, `时长 ${metrics.duration.toFixed(2)}s ≈ 2.9s`);
check(metrics.errors === 2, `异常数 ${metrics.errors} == 2 (failsafe 1 + 错误串 1)`);

// ================= 曲线抽取 =================
const charts = extractUlogCharts(buf);
const ids = charts.map((c) => c.id);
for (const want of ['att', 'attsp', 'gyro', 'accel', 'altpos', 'gps', 'battery', 'rcou', 'ekf']) {
  check(ids.includes(want), `曲线组包含 ${want}`);
}
const att = charts.find((c) => c.id === 'att');
check(att.series.length === 3 && att.series[0].t.length === N, '姿态曲线 3 序列 × 30 点');
check(att.series[0].t[0] === 0, '曲线时间轴归零');
const bat = charts.find((c) => c.id === 'battery');
check(bat.series.length === 3, '电池曲线 Volt/Curr/Rem');
const volt = bat.series.find((s) => s.name === 'Volt');
check(volt.v[0] > volt.v[volt.v.length - 1], '电压单调下降');

// ================= 自动分流 =================
const auto = analyzeLogAuto(buf);
check(auto.format === 'ulg', "analyzeLogAuto 标记 format='ulg'");
check(/PX4 ULog/.test(auto.text), 'analyzeLogAuto 走 ULog 分支');
check(extractChartsAuto(buf).length > 0, 'extractChartsAuto 走 ULog 分支');
const ulgPrompt = pickLogSystemPrompt('ulg');
const binPrompt = pickLogSystemPrompt('bin');
check(ulgPrompt !== binPrompt, 'ulg/bin 提示词不同');
check(/PX4/.test(ulgPrompt) && /MC_ROLLRATE|v1\.14/.test(ulgPrompt), 'ULG 提示词为 PX4 调参语境');

// px4NavStateName 工具
check(px4NavStateName(3) === 'AUTO_MISSION', 'px4NavStateName(3)=AUTO_MISSION');
check(px4NavStateName(14) === 'OFFBOARD', 'px4NavStateName(14)=OFFBOARD');
check(px4NavStateName(99) === 'NAV99', 'px4NavStateName 未知值兜底');

console.log(`\n${failures === 0 ? 'ULog 解析单测通过' : failures + ' 项失败'}`);
process.exit(failures === 0 ? 0 : 1);
