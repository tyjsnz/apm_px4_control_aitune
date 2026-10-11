// PX4 进程内模拟飞控集成测试 (不依赖串口/pty):
//   固件识别 / custom_mode 解码 / 参数表回读(带丢包重试) / PARAM_SET 类型协商
//   / 校准映射 (PX4 分流) / 日志下载扩展名 (.ulg vs .bin)
//   node test/px4.test.js
import { FcOps, decodePx4CustomMode, MAV_AUTOPILOT_PX4 } from '../lib/fcops.js';
import { MavlinkLink } from '../lib/mavlink.js';
import { isUlog } from '../lib/ulog.js';

let failures = 0;
function check(cond, msg) {
  if (cond) { console.log('  ok  ' + msg); } else { console.log('  FAIL ' + msg); failures++; }
}

// ================= 内存存储 (FcStore) =================
const files = new Map();
const store = {
  saveText: async (n, t) => { files.set(n, t); },
  loadText: async (n) => { if (!files.has(n)) throw new Error('not found: ' + n); return files.get(n); },
  saveBin: async (n, d) => { files.set(n, Buffer.from(d)); },
};

// ================= PX4 模拟器 =================
// 参数表: t=9 REAL32 / t=6 INT32 (PX4 严格校验 PARAM_SET.param_type, 类型不符静默忽略)
const fcParams = {
  MPC_XY_CRUISE: { v: 5.0, t: 9 },
  MPC_TKO_SPEED: { v: 1.5, t: 9 },
  MPC_LAND_SPEED: { v: 0.7, t: 9 },
  MC_ROLL_P: { v: 6.5, t: 9 },
  MC_ROLLRATE_P: { v: 0.15, t: 9 },
  COM_DISARM_LAND: { v: 2.0, t: 9 },
  BAT_LOW_THR: { v: 0.15, t: 9 },
  IMU_GYRO_CUTOFF: { v: 40.0, t: 9 },
  RTL_RETURN_ALT: { v: 60.0, t: 9 },
  GF_ACTION: { v: 3, t: 6 },       // INT32
  NAV_RCL_ACT: { v: 2, t: 6 },     // INT32
};

// 日志: 1 号为 ULog(大量 0 字节, 压测 MAVLink2 零截断+数组补零), 2 号为任意二进制
const ulogContent = Buffer.alloc(300, 0);
ulogContent.write('ULog', 0, 'ascii');
ulogContent.writeUInt8(0x01, 4);
ulogContent.writeUInt8(0x12, 5);
ulogContent.writeUInt8(0x35, 6);
ulogContent.writeUInt8(1, 7);
ulogContent[120] = 0x50;
ulogContent[121] = 0x11;
const binContent = Buffer.alloc(120, 0x5a);
const fcLogs = { 1: ulogContent, 2: binContent };

let ignoreListRequests = 1;      // 模拟 PX4 建链初期丢弃首个 PARAM_REQUEST_LIST
const commands = [];             // 收到的 COMMAND_LONG 记录

let ops;
const fc = new MavlinkLink({
  write: (b) => ops.feed(b),
  srcSys: 1, srcComp: 1,
  onMessage: (m) => {
    const f = m.fields || {};
    const nParams = Object.keys(fcParams).length;
    if (m.name === 'PARAM_REQUEST_LIST') {
      if (ignoreListRequests > 0) { ignoreListRequests--; return; }
      Object.keys(fcParams).forEach((name, i) => {
        fc.send('PARAM_VALUE', {
          param_id: name, param_value: fcParams[name].v, param_type: fcParams[name].t,
          param_count: nParams, param_index: i,
        });
      });
    } else if (m.name === 'PARAM_REQUEST_READ') {
      const name = String(f.param_id || '').toUpperCase();
      const p = fcParams[name];
      // PX4 对未知参数不回 (模拟)
      if (p) {
        fc.send('PARAM_VALUE', {
          param_id: name, param_value: p.v, param_type: p.t,
          param_count: nParams, param_index: 0,
        });
      }
    } else if (m.name === 'PARAM_SET') {
      const name = String(f.param_id || '').toUpperCase();
      const p = fcParams[name];
      // PX4 严格校验类型: 不符则静默忽略 (不回显) → 逼出 setParamSmart 的换类型重试
      if (p && f.param_type === p.t) {
        p.v = f.param_value;
        fc.send('PARAM_VALUE', {
          param_id: name, param_value: p.v, param_type: p.t,
          param_count: nParams, param_index: 0,
        });
      }
    } else if (m.name === 'COMMAND_LONG') {
      commands.push(f);
      fc.send('COMMAND_ACK', {
        command: f.command, result: 0, progress: 100, result_param2: 0,
        target_system: 255, target_component: 190,
      });
    } else if (m.name === 'LOG_REQUEST_LIST') {
      const start = f.start || 0;
      const end = f.end || 0xffff;
      const ids = Object.keys(fcLogs).map(Number)
        .filter((id) => id >= start && id <= end);
      const last = Math.max(...Object.keys(fcLogs).map(Number));
      ids.forEach((id) => {
        fc.send('LOG_ENTRY', {
          id, num_logs: ids.length, last_log_num: last,
          time_utc: 1700000000 + id, size: fcLogs[id].length,
        });
      });
    } else if (m.name === 'LOG_REQUEST_DATA') {
      const log = fcLogs[f.id];
      if (!log) return;
      let off = f.ofs || 0;
      const end = Math.min(off + (f.count || 90), log.length);
      while (off < end) {
        const n = Math.min(90, end - off);
        fc.send('LOG_DATA', {
          id: f.id, ofs: off, count: n,
          data: Array.from(log.subarray(off, off + n)),
        });
        off += n;
      }
    }
  },
});

// ================= 工具 =================
const emitted = [];
function lastOf(type) {
  for (let i = emitted.length - 1; i >= 0; i--) if (emitted[i].type === type) return emitted[i];
  return null;
}
function fcHeartbeat(autopilot, customMode, armed = false) {
  fc.send('HEARTBEAT', {
    custom_mode: customMode, type: 6 /* MAV_TYPE_QUADROTOR */, autopilot,
    base_mode: armed ? 0x80 : 0, system_status: 4, mavlink_version: 3,
  });
}
const PX4 = MAV_AUTOPILOT_PX4;      // 12
const AP_COPTER = 3;
const px4Mode = (main, sub = 0) => (main | (sub << 8)) << 16; // main bits16-23, sub bits24-31

async function main() {
  ops = new FcOps({
    write: (b) => fc.feed(b),
    emit: (m) => emitted.push(m),
    store,
    log: () => {},
  });
  ops.attach({ path: 'sim', baud: 57600 });
  check(emitted.some((m) => m.type === 'connected'), 'attach 发出 connected');

  // ---------- 1. 固件识别 + 模式解码 ----------
  fcHeartbeat(PX4, px4Mode(3)); // POSCTL
  check(lastOf('heartbeat').firmware === 'px4', "HEARTBEAT autopilot=12 → firmware='px4'");
  check(lastOf('heartbeat').flightMode === 'POSCTL', 'POSCTL 模式解码');
  check(ops.telemetry.firmware === 'px4' && ops.fcTypeName() === 'PX4', 'telemetry/fcTypeName 为 PX4');

  fcHeartbeat(PX4, px4Mode(4, 4), true); // AUTO/MISSION, armed
  check(lastOf('heartbeat').flightMode === 'AUTO MISSION', 'AUTO MISSION 模式解码');
  check(lastOf('heartbeat').armed === true, '解锁位识别');

  check(decodePx4CustomMode(px4Mode(4, 5)) === 'AUTO RTL', 'AUTO RTL 解码');
  check(decodePx4CustomMode(px4Mode(4, 2)) === 'AUTO TAKEOFF', 'AUTO TAKEOFF 解码');
  check(decodePx4CustomMode(px4Mode(6)) === 'OFFBOARD', 'OFFBOARD 解码');
  check(decodePx4CustomMode(px4Mode(2)) === 'ALTCTL', 'ALTCTL 解码');
  check(decodePx4CustomMode(px4Mode(7)) === 'STABILIZED', 'STABILIZED 解码');

  // ---------- 2. 参数表回读 (首个请求被丢 → 静默重发) ----------
  const dump = await ops.dumpParamList({ timeoutMs: 8000, quietMs: 300 });
  check(Object.keys(dump.params).length === Object.keys(fcParams).length,
    `参数表回读 ${Object.keys(dump.params).length}/${Object.keys(fcParams).length} (含丢包重试)`);
  check(dump.expect === Object.keys(fcParams).length, 'param_count 正确');
  check(Math.abs(dump.params.GF_ACTION - 3) < 1e-6, 'INT32 参数值正确 GF_ACTION=3');
  check(Math.abs(dump.params.MC_ROLL_P - 6.5) < 1e-6, 'REAL32 参数值正确 MC_ROLL_P=6.5');
  check(ops.paramCacheAll.MC_ROLL_P.type === 9 && ops.paramCacheAll.GF_ACTION.type === 6,
    '参数缓存携带类型 (9/6)');

  // ---------- 3. set_param: 缓存类型优先 ----------
  await ops.handle({ type: 'set_param', paramId: 'MPC_XY_CRUISE', paramValue: 7.5, paramType: 9 });
  const s1 = lastOf('param_set_sent');
  check(s1 && s1.paramId === 'MPC_XY_CRUISE', '带缓存类型写入成功 (param_set_sent)');
  check(Math.abs(fcParams.MPC_XY_CRUISE.v - 7.5) < 1e-3, '飞控侧值已更新 7.5');

  // ---------- 4. set_param 类型协商 (无缓存 + 错误 hint → 换类型重试) ----------
  // 4a. INT32 参数被错误地按 REAL32(hint=9) 发 → 静默忽略 → 重试 6 成功
  delete ops.paramCacheAll.GF_ACTION;
  await ops.handle({ type: 'set_param', paramId: 'GF_ACTION', paramValue: 0, paramType: 9 });
  check(lastOf('param_set_sent') && lastOf('param_set_sent').paramId === 'GF_ACTION'
    && lastOf('param_set_sent').paramValue === 0,
  'REAL32 失败后协商为 INT32 写入 GF_ACTION=0');
  check(fcParams.GF_ACTION.v === 0, '飞控侧 INT32 值 = 0');
  // 4b. REAL32 参数被错误地按 INT32(hint=6) 发 → 重试 9 成功
  delete ops.paramCacheAll.RTL_RETURN_ALT;
  await ops.handle({ type: 'set_param', paramId: 'RTL_RETURN_ALT', paramValue: 30, paramType: 6 });
  check(lastOf('param_set_sent') && lastOf('param_set_sent').paramId === 'RTL_RETURN_ALT',
    'INT32 失败后协商为 REAL32 写入 RTL_RETURN_ALT');
  check(Math.abs(fcParams.RTL_RETURN_ALT.v - 30) < 1e-3, '飞控侧 REAL32 值 = 30');

  // ---------- 5. 校准分流 (PX4 映射: gyro=p1, accel=p5; level 禁用) ----------
  fc.send('SYS_STATUS', {
    voltage_battery: 24800, current_battery: 1500, battery_remaining: 80, load: 300,
  });
  await new Promise((r) => setTimeout(r, 50));

  commands.length = 0;
  await ops.handle({ type: 'ot_cal', kind: 'level' });
  const calLevel = lastOf('ot_cal_done');
  check(calLevel && calLevel.ok === false && /QGC|QGroundControl/.test(calLevel.text),
    'PX4 level 校准被禁用并提示 QGC');
  check(commands.length === 0, 'level 未发出校准命令');

  commands.length = 0;
  await ops.handle({ type: 'ot_cal', kind: 'gyro' });
  check(commands.length === 1 && commands[0].command === 241
    && commands[0].param1 === 1 && commands[0].param3 === 0 && commands[0].param5 === 0,
  'gyro 校准 → PREFLIGHT_CALIBRATION param1=1');
  check(lastOf('ot_cal_done') && lastOf('ot_cal_done').ok === true, 'gyro 校准收到 ACK');

  commands.length = 0;
  await ops.handle({ type: 'ot_cal', kind: 'accel' });
  check(commands.length === 1 && commands[0].param5 === 1,
    'accel 校准 → PREFLIGHT_CALIBRATION param5=1 (QGC 映射)');

  // ---------- 6. 六面向导 / 拆桨实测对 PX4 禁用 ----------
  await ops.handle({ type: 'ot_accel_cal_start' });
  const accelErr = [...emitted].reverse().find((m) => m.type === 'error');
  check(accelErr && /QGroundControl|QGC/.test(accelErr.message), '六面向导 PX4 禁用并提示 QGC');
  await ops.handle({ type: 'ot_spin_start' });
  const spin = lastOf('ot_spin_done');
  check(spin && spin.ok === false && /QGC|QGroundControl/.test(spin.text), '拆桨实测 PX4 禁用并提示 QGC');

  // ---------- 7. 一键首飞写入 (自动备份 + 写入) ----------
  await ops.handle({ type: 'ot_write', items: [
    { name: 'MPC_TKO_SPEED', value: 2.0 },
    { name: 'NAV_RCL_ACT', value: 3 },
  ] });
  const wdone = lastOf('ot_write_done');
  check(wdone && wdone.success === 2 && wdone.failed.length === 0,
    `ot_write 成功 ${wdone && wdone.success}/2 失败 ${JSON.stringify(wdone && wdone.failed)}`);
  check(wdone && /^backup_.*_onetouch\.json$/.test(wdone.backup || ''), '写入前自动备份文件名 ' + (wdone && wdone.backup));
  const bk = wdone && wdone.backup ? JSON.parse(files.get(wdone.backup)) : null;
  check(bk && Math.abs(bk.params.MPC_TKO_SPEED - 1.5) < 1e-3 && bk.params.NAV_RCL_ACT === 2,
    '备份保存了写入前的旧值 (1.5 / 2)');
  check(bk && bk.fc_type === 'PX4', '备份标记 fc_type=PX4');
  check(Math.abs(fcParams.MPC_TKO_SPEED.v - 2.0) < 1e-3, '飞控侧 MPC_TKO_SPEED=2.0');
  check(fcParams.NAV_RCL_ACT.v === 3, '飞控侧 NAV_RCL_ACT=3 (INT32 经缓存类型写入)');

  // ---------- 8. 日志: 列表 + .ulg 下载 ----------
  await ops.handle({ type: 'log_list' });
  const ll = lastOf('log_list');
  check(ll && ll.logs.length === 2 && ll.logs[0].size === ulogContent.length,
    '日志列表 2 条且大小正确 (PX4 支持 LOG_REQUEST_LIST)');

  emitted.length = 0;
  await ops.handle({ type: 'log_download', ids: [1] });
  const saved1 = lastOf('log_download_saved');
  check(saved1 && /\.ulg$/.test(saved1.file), 'PX4 日志保存为 .ulg → ' + (saved1 && saved1.file));
  const got = saved1 ? files.get(saved1.file) : null;
  check(got && got.length === ulogContent.length && got.equals(ulogContent),
    '下载内容与模拟日志逐字节一致 (零截断/补零往返无损)');
  check(got && isUlog(got) === true, '下载的日志是合法 ULog');
  check(lastOf('log_download_done') && lastOf('log_download_done').failed.length === 0, '下载无失败项');

  // ---------- 9. ArduPilot 固件下扩展名仍为 .bin ----------
  fcHeartbeat(AP_COPTER, 4); // GUIDED
  check(lastOf('heartbeat').firmware === 'apm' && lastOf('heartbeat').flightMode === 'GUIDED',
    'AP 心跳 → firmware=apm + GUIDED 模式');
  emitted.length = 0;
  await ops.handle({ type: 'log_download', ids: [2] });
  const saved2 = lastOf('log_download_saved');
  check(saved2 && /\.bin$/.test(saved2.file), 'AP 日志保存为 .bin → ' + (saved2 && saved2.file));

  ops.detach();
  check(lastOf('disconnected') !== null, 'detach 发出 disconnected');

  console.log(`\n${failures === 0 ? 'PX4 集成测试通过' : failures + ' 项失败'}`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('测试异常:', e);
  process.exit(1);
});
