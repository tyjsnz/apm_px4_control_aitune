// MAVLink 对拍测试 (Node 侧)
//   node test/mavlink_roundtrip.js gen     -> 写 test/out/js_frames.json (并自检回环)
//   node test/mavlink_roundtrip.js check   -> 解析 test/out/py_frames.json 并比对
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import { MavlinkLink, Parser, MAVLINK_DEFS } from '../lib/mavlink.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const OUT = path.join(__dirname, 'out');

export const CASES = [
  ['HEARTBEAT', { custom_mode: 0, type: 2, autopilot: 3, base_mode: 81, system_status: 4, mavlink_version: 3 }],
  ['PARAM_REQUEST_READ', { param_index: -1, target_system: 1, target_component: 1, param_id: 'MOT_THST_HOVER' }],
  ['PARAM_REQUEST_LIST', { target_system: 1, target_component: 1 }],
  ['PARAM_VALUE', { param_value: 0.25, param_count: 1200, param_index: 5, param_id: 'MOT_THST_HOVER', param_type: 9 }],
  ['PARAM_SET', { param_value: 0.3, target_system: 1, target_component: 1, param_id: 'MOT_THST_HOVER', param_type: 9 }],
  ['COMMAND_LONG', { param1: 1, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0, command: 246, target_system: 1, target_component: 1, confirmation: 0 }],
  ['COMMAND_ACK', { command: 246, result: 0, progress: 0, result_param2: 0, target_system: 1, target_component: 1 }],
  ['STATUSTEXT', { severity: 6, text: 'PreArm: check', id: 0, chunk_seq: 0 }],
  ['ATTITUDE', { time_boot_ms: 123456, roll: 0.1, pitch: -0.2, yaw: 1.5, rollspeed: 0.01, pitchspeed: -0.02, yawspeed: 0.003 }],
  ['GLOBAL_POSITION_INT', { time_boot_ms: 1000, lat: 399000000, lon: 1164000000, alt: 50000, relative_alt: 3000, vx: 10, vy: -20, vz: 5, hdg: 9000 }],
  ['VFR_HUD', { airspeed: 5, groundspeed: 6, alt: 30, climb: 1.2, heading: 90, throttle: 55 }],
  ['SYS_STATUS', { onboard_control_sensors_present: 1, onboard_control_sensors_enabled: 1, onboard_control_sensors_health: 1, load: 250, voltage_battery: 24000, current_battery: 100, drop_rate_comm: 0, errors_comm: 0, errors_count1: 0, errors_count2: 0, errors_count3: 0, errors_count4: 0, battery_remaining: 80, onboard_control_sensors_present_extended: 0, onboard_control_sensors_enabled_extended: 0, onboard_control_sensors_health_extended: 0 }],
  ['ESC_TELEMETRY_1_TO_4', { voltage: [2400, 2401, 2402, 2403], current: [100, 101, 102, 103], totalcurrent: [10, 11, 12, 13], rpm: [0, 1500, 1600, 1700], count: [1, 2, 3, 4], temperature: [30, 31, 32, 33] }],
];

function numEq(a, b) {
  if (Array.isArray(a) || Array.isArray(b)) {
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return false;
    return a.every((v, i) => numEq(v, b[i]));
  }
  if (typeof a === 'string' || typeof b === 'string') return String(a) === String(b);
  if (typeof a === 'number' && typeof b === 'number') {
    return Math.abs(a - b) <= Math.max(1e-4, Math.abs(a) * 1e-4);
  }
  return a === b;
}

function compare(name, got, want, prefix, errs, version) {
  const base = version === 1 ? new Set(MAVLINK_DEFS[name].v1Fields) : null;
  for (const k of Object.keys(want)) {
    if (base && !base.has(k)) continue; // v1.0 不含 MAVLink2 扩展字段
    if (!numEq(got[k], want[k])) {
      errs.push(`${prefix} ${name}.${k}: got=${JSON.stringify(got[k])} want=${JSON.stringify(want[k])}`);
    }
  }
}

function gen() {
  fs.mkdirSync(OUT, { recursive: true });
  const frames = [];
  const errs = [];
  for (const [name, fields] of CASES) {
    for (const version of [1, 2]) {
      if (version === 1 && MAVLINK_DEFS[name].id > 255) continue;
      const chunks = [];
      const link = new MavlinkLink({
        write: (b) => chunks.push(Buffer.from(b)),
        srcSys: 255, srcComp: 190,
      });
      link.version = version;
      link.send(name, fields);
      const frame = Buffer.concat(chunks);
      // 自检: 用 Parser 解析回环
      const p = new Parser();
      const msgs = p.feed(frame);
      if (msgs.length !== 1) {
        errs.push(`${name} v${version}: 回环解析得到 ${msgs.length} 条`);
      } else {
        compare(name, msgs[0].fields, fields, `v${version} self`, errs, version);
        if (msgs[0].version !== version) errs.push(`${name}: 版本不符`);
        if (msgs[0].msgId !== MAVLINK_DEFS[name].id) errs.push(`${name}: msgId 不符`);
      }
      frames.push({ name, version, hex: frame.toString('hex'), fields, msgId: MAVLINK_DEFS[name].id, sysid: 255, compid: 190 });
    }
  }
  fs.writeFileSync(path.join(OUT, 'js_frames.json'), JSON.stringify(frames, null, 1));
  if (errs.length) {
    console.error('JS 自检失败:\n' + errs.join('\n'));
    process.exit(1);
  }
  console.log(`JS 编码 ${frames.length} 帧, 自检通过 -> test/out/js_frames.json`);
}

function check() {
  const file = path.join(OUT, 'py_frames.json');
  if (!fs.existsSync(file)) {
    console.error('缺少 ' + file + ' (先跑 python3 test/pymavlink_roundtrip.py gen)');
    process.exit(1);
  }
  const frames = JSON.parse(fs.readFileSync(file, 'utf8'));
  const errs = [];
  let n = 0;
  for (const f of frames) {
    const p = new Parser();
    const msgs = p.feed(Buffer.from(f.hex, 'hex'));
    if (msgs.length !== 1) { errs.push(`${f.name} v${f.version}: 解析到 ${msgs.length} 条, bad=${p.badFrames}`); continue; }
    const m = msgs[0];
    if (m.name !== f.name) errs.push(`${f.name}: name=${m.name}`);
    if (m.msgId !== f.msgId) errs.push(`${f.name}: msgId=${m.msgId} want=${f.msgId}`);
    if (m.version !== f.version) errs.push(`${f.name}: version=${m.version} want=${f.version}`);
    if (m.sysid !== f.sysid) errs.push(`${f.name}: sysid=${m.sysid} want=${f.sysid}`);
    if (m.compid !== f.compid) errs.push(`${f.name}: compid=${m.compid} want=${f.compid}`);
    compare(f.name, m.fields, f.fields, `v${f.version} py->js`, errs, f.version);
    n++;
  }
  if (errs.length) { console.error('对拍失败:\n' + errs.join('\n')); process.exit(1); }
  console.log(`解析 pymavlink 帧 ${n} 条, 全部一致`);
}

const mode = process.argv[2] || 'gen';
if (mode === 'gen') gen();
else if (mode === 'check') check();
else { console.error('用法: node test/mavlink_roundtrip.js [gen|check]'); process.exit(1); }
