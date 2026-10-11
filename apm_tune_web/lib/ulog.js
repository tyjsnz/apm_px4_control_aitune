// PX4 ULog 解析器 (规格: https://docs.px4.io/main/en/dev_log/ulog_file_format)
// 与 lib/binlog.js (ArduPilot DataFlash) 平行; 输出结构与 lib/loganalysis.js 对齐:
//   analyzeUlog(buf)        -> { text, metrics }   (文本摘要, 供 AI 报告)
//   extractUlogCharts(buf)  -> [{id,title,series:[{name,unit,t:[],v:[]}]}]
// 约束: 只依赖 Buffer 全局 (浏览器用 public/vendor/buffer-shim.js), 不用任何 Node 专有 API
//
// 文件结构:
//   16 字节文件头: "ULog" + 0x01 0x12 0x35 + version(1B) + timestamp(8B LE)
//   之后为消息流: uint16 msg_size (LE, 不含 3 字节头) + uint8 msg_type + payload(msg_size)
//   msg_type: 'B'标志位 'F'格式 'I'信息 'M'多信息 'P'参数 'Q'默认参数
//             'A'订阅 'R'退订 'D'数据 'L'字符串 'C'带标签字符串 'S'同步 'O'丢包

const TYPE_SIZES = {
  int8: 1, uint8: 1, bool: 1, char: 1,
  int16: 2, uint16: 2,
  int32: 4, uint32: 4, float: 4,
  int64: 8, uint64: 8, double: 8,
};

// PX4 vehicle_status.nav_state (v1.13+ 重排后的编号; v1.14/v1.15/main 通用)
export const PX4_NAV_STATES = {
  0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER',
  5: 'AUTO_RTL', 6: 'POSITION_SLOW', 7: 'GUIDED_COURSE', 8: 'ALTITUDE_CRUISE',
  9: 'MANUAL_PARKING', 10: 'ACRO', 11: 'FREE', 12: 'DESCEND', 13: 'TERMINATION',
  14: 'OFFBOARD', 15: 'STABILIZED', 16: 'FREE', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND',
  19: 'AUTO_FOLLOW', 20: 'AUTO_PRECLAND', 21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF',
};
export const px4NavStateName = (n) => PX4_NAV_STATES[n] || `NAV${n}`;

const fmtNum = (v, d = 3) => {
  if (typeof v !== 'number' || !isFinite(v)) return String(v);
  if (Number.isInteger(v)) return String(v);
  return String(Number(v.toFixed(d)));
};

/** 是否 ULog (文件头 "ULog") */
export function isUlog(buf) {
  return !!(buf && buf.length >= 8
    && buf[0] === 0x55 && buf[1] === 0x4c && buf[2] === 0x6f && buf[3] === 0x67);
}

/** 是否 ArduPilot DataFlash (magic 0x95 0xA3) */
export function isDataFlash(buf) {
  return !!(buf && buf.length >= 2 && buf[0] === 0xa3 && buf[1] === 0x95);
}

// ---------- 内部: 消息体解析 ----------
function readScalar(buf, off, type) {
  switch (type) {
    case 'int8': return buf.readInt8(off);
    case 'uint8': case 'bool': case 'char': return buf.readUInt8(off);
    case 'int16': return buf.readInt16LE(off);
    case 'uint16': return buf.readUInt16LE(off);
    case 'int32': return buf.readInt32LE(off);
    case 'uint32': return buf.readUInt32LE(off);
    case 'float': return buf.readFloatLE(off);
    case 'int64': return Number(buf.readBigInt64LE(off));
    case 'uint64': return Number(buf.readBigUInt64LE(off));
    case 'double': return buf.readDoubleLE(off);
    default: return null;
  }
}

/** 解析 'F' 格式串: "msg_name:type0 f0;type1 f1;..." -> {name, fields, size} 或 null */
function parseFormat(str) {
  const i = str.indexOf(':');
  if (i <= 0) return null;
  const name = str.slice(0, i).trim();
  const fields = [];
  for (const part of str.slice(i + 1).split(';')) {
    if (!part) continue;
    const sp = part.indexOf(' ');
    if (sp <= 0) return null; // 嵌套消息类型暂不支持
    const typeStr = part.slice(0, sp);
    const fname = part.slice(sp + 1).trim();
    const m = /^(\w+)(?:\[(\d+)\])?$/.exec(typeStr);
    if (!m || !fname) return null;
    const base = m[1];
    const size = TYPE_SIZES[base];
    if (size === undefined) return null; // 嵌套类型
    const arr = m[2] ? parseInt(m[2], 10) : 0;
    fields.push({ type: base, name: fname, size: size * (arr || 1), array: arr });
  }
  if (!fields.length) return null;
  return { name, fields, size: fields.reduce((a, f) => a + f.size, 0) };
}

/** 按格式解码一条数据(末尾 _padding 可能未记录, 超长截断) */
function decodeData(fmt, data) {
  const out = {};
  let off = 0;
  for (const f of fmt.fields) {
    if (off + f.size > data.length) break;
    if (!/^_padding/.test(f.name)) {
      if (f.array) {
        const arr = [];
        for (let i = 0; i < f.array; i++) arr.push(readScalar(data, off + i * TYPE_SIZES[f.type], f.type));
        out[f.name] = arr;
      } else {
        out[f.name] = readScalar(data, off, f.type);
      }
    }
    off += f.size;
  }
  return out;
}

/**
 * 遍历 ULog。定义/参数/信息/字符串在返回值里, 数据消息经 onData 流式回调 (避免整文件驻留内存)。
 * @param {Buffer} buf
 * @param {(topic:string, fields:object, us:number)=>void} [onData]
 */
export function parseUlog(buf, onData) {
  if (!isUlog(buf)) throw new Error('不是 ULog 文件 (文件头不匹配)');
  const out = {
    version: buf[7],
    info: {},            // 'I' key -> value
    params: {},          // 'P' name -> float
    defaults: {},        // 'Q' name -> float
    formats: {},         // msg_name -> {name, fields, size}
    subs: new Map(),     // msg_id -> {name, multiId}
    strings: [],         // {us, level, text} (上限 500 条)
    stringCount: 0,
    dropouts: 0, dropMs: 0,
    dataCount: 0,
    truncated: false,
  };
  let off = 16;
  while (off + 3 <= buf.length) {
    const size = buf.readUInt16LE(off);
    const type = String.fromCharCode(buf[off + 2]);
    const body = off + 3;
    if (body + size > buf.length) { out.truncated = true; break; } // 尾部截断(崩溃日志)
    const payload = buf.subarray(body, body + size);
    try {
      switch (type) {
        case 'I':
        case 'P':
        case 'Q': {
          // 'I': key_len + key + value;  'P': 同 'I' (新版) 或 char[16]+f32 (旧版);
          // 'Q': default_types + key_len + key + value 或 1B + char[16] + f32 (旧版)
          const oldFixed = (type === 'P' && size === 20)
            || (type === 'Q' && size === 21);
          if (oldFixed) {
            const nameStart = type === 'Q' ? 1 : 0;
            let end = nameStart;
            while (end < nameStart + 16 && payload[end] !== 0) end++;
            const key = payload.toString('utf8', nameStart, end);
            const value = payload.readFloatLE(type === 'Q' ? 17 : 16);
            if (type === 'P') out.params[key] = value;
            else if (type === 'Q') out.defaults[key] = value;
            else out.info[key] = value;
          } else {
            const keyLenOff = type === 'Q' ? 1 : 0;
            const keyLen = payload[keyLenOff];
            const keyStart = keyLenOff + 1;
            if (keyLen <= size - keyStart - 4) {
              const key = payload.toString('utf8', keyStart, keyStart + keyLen);
              const value = payload.readFloatLE(size - 4);
              if (type === 'P') out.params[key] = value;
              else if (type === 'Q') out.defaults[key] = value;
              else out.info[key] = payload.toString('utf8', keyStart + keyLen, size);
            } else if (type === 'I' && keyLen <= size - keyStart) {
              const key = payload.toString('utf8', keyStart, keyStart + keyLen);
              out.info[key] = payload.toString('utf8', keyStart + keyLen, size);
            }
          }
          break;
        }
        case 'M': {
          const keyLen = payload[1];
          const keyStart = 2;
          if (keyLen <= size - keyStart) {
            const key = payload.toString('utf8', keyStart, keyStart + keyLen);
            const value = payload.toString('utf8', keyStart + keyLen, size);
            if (out.info[key] === undefined) out.info[key] = value;
          }
          break;
        }
        case 'F': {
          const f = parseFormat(payload.toString('utf8', 0, size));
          if (f) out.formats[f.name] = f;
          break;
        }
        case 'A': {
          const multiId = payload[0];
          const msgId = payload.readUInt16LE(1);
          const name = payload.toString('utf8', 3, size);
          out.subs.set(msgId, { name, multiId });
          break;
        }
        case 'R': {
          out.subs.delete(payload.readUInt16LE(0));
          break;
        }
        case 'D': {
          const msgId = payload.readUInt16LE(0);
          const sub = out.subs.get(msgId);
          if (sub && onData) {
            const fmt = out.formats[sub.name];
            if (fmt) {
              const fields = decodeData(fmt, payload.subarray(2, size));
              const us = typeof fields.timestamp === 'number' ? fields.timestamp : null;
              const topic = sub.multiId ? `${sub.name}:${sub.multiId}` : sub.name;
              out.dataCount++;
              onData(topic, fields, us);
            }
          }
          break;
        }
        case 'L': {
          const level = payload[0];
          const us = Number(payload.readBigUInt64LE(1));
          const text = payload.toString('utf8', 9, size);
          out.stringCount++;
          if (out.strings.length < 500) out.strings.push({ us, level, text });
          break;
        }
        case 'C': {
          const level = payload[0];
          const us = Number(payload.readBigUInt64LE(3));
          const text = payload.toString('utf8', 11, size);
          out.stringCount++;
          if (out.strings.length < 500) out.strings.push({ us, level, text });
          break;
        }
        case 'O': {
          out.dropouts++;
          out.dropMs += payload.readUInt16LE(0);
          break;
        }
        default: break; // 'B' 'S' 及未知消息: 跳过
      }
    } catch (e) {
      // 单条消息解析失败不终止整个文件
    }
    off = body + size;
  }
  return out;
}

// ---------- 关键指标采集 ----------
const TRACK_TOPICS = new Set([
  'vehicle_status', 'vehicle_status_flags', 'vehicle_control_mode',
  'vehicle_attitude', 'vehicle_attitude_setpoint', 'vehicle_rates_setpoint',
  'vehicle_local_position', 'vehicle_local_position_setpoint',
  'vehicle_global_position', 'vehicle_gps_position', 'vehicle_gps0',
  'sensor_combined', 'vehicle_imu', 'vehicle_imu_status', 'vehicle_air_data',
  'battery_status', 'actuator_outputs', 'actuator_motors', 'actuator_servos',
  'manual_control_setpoint', 'vehicle_land_detected',
  'estimator_status', 'estimator_innovations', 'estimator_sensor_bias',
  'estimator_status_flags', 'wind_estimate', 'takeoff_status',
]);

const FIELD_CAP = 24; // 每个 topic 最多统计的字段数

const RAD2DEG = 180 / Math.PI;

function quatToEuler(q) {
  // q = [w, x, y, z] (PX4 vehicle_attitude.q)
  const [w, x, y, z] = q;
  const roll = Math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const s = 2.0 * (w * y - z * x);
  const pitch = Math.abs(s) >= 1 ? (Math.sign(s) * Math.PI / 2) : Math.asin(s);
  const yaw = Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return { roll, pitch, yaw };
}

/**
 * ULog 文本摘要 (结构与 ArduPilot analyzeLog 输出对齐, 供同一套 AI 报告流程复用)
 */
export function analyzeUlog(buf, opts = {}) {
  const excerptRows = opts.excerptRows || 30;
  const defs = parseUlog(buf, null); // 只拿定义, 数据另遍历
  const stats = {};
  const modes = [];        // {us, nav}
  const arms = [];         // {us, armed}
  const failsafes = [];    // {us}
  const acc = {
    firstUS: null, lastUS: null,
    maxVib: 0,               // 加速度模长偏离 1g 峰值 (vehicle_imu)
    attRollErr: 0, attPitchErr: 0,
    maxSpeed: 0, maxRelAlt: null,
    minSats: Infinity,
    voltMin: Infinity, voltMax: -Infinity, currMax: -Infinity, remMin: Infinity,
    maxMotors: 0,            // actuator_motors control 峰值 (归一化 0~1)
    maxRcout: 0,             // actuator_outputs outputs 峰值 (PWM)
    ekfVelPosInnov: null,    // RMS
  };
  let lastAtt = null;        // {us, roll, pitch}
  let lastFS = false;        // failsafe 上一拍状态 (bool 解码为 0/1, 需按边沿计次)
  let innovSum = 0; let innovN = 0;
  const excerpts = {};
  let total = 0;

  const track = (st, f) => {
    let s = st.fields[f];
    if (!s) { s = st.fields[f] = { min: Infinity, max: -Infinity, sum: 0, n: 0, first: null, last: null }; }
    return s;
  };
  const upd = (s, v) => {
    if (typeof v !== 'number' || !isFinite(v)) return;
    s.min = Math.min(s.min, v); s.max = Math.max(s.max, v);
    s.sum += v; s.n++;
    if (s.first === null) s.first = v;
    s.last = v;
  };

  const EXCERPT_TOPICS = ['battery_status', 'vehicle_gps_position', 'vehicle_gps0', 'vehicle_status'];

  // 二次遍历数据 (流式)
  parseUlog(buf, (topic, fields, us) => {
    total++;
    const base = topic.split(':')[0];
    if (typeof us === 'number') {
      if (acc.firstUS === null || us < acc.firstUS) acc.firstUS = us;
      if (acc.lastUS === null || us > acc.lastUS) acc.lastUS = us;
    }

    let st = stats[topic];
    if (!st) { st = stats[topic] = { count: 0, firstUS: null, lastUS: null, fields: {} }; }
    st.count++;
    if (typeof us === 'number') {
      if (st.firstUS === null || us < st.firstUS) st.firstUS = us;
      if (st.lastUS === null || us > st.lastUS) st.lastUS = us;
    }
    if (TRACK_TOPICS.has(base)) {
      let n = 0;
      for (const [k, v] of Object.entries(fields)) {
        if (n >= FIELD_CAP) break;
        if (k === 'timestamp') continue;
        if (typeof v === 'number') { upd(track(st, k), v); n++; }
      }
    }

    if (base === 'vehicle_status') {
      if (typeof fields.nav_state === 'number') {
        if (!modes.length || modes[modes.length - 1].nav !== fields.nav_state) {
          modes.push({ us, nav: fields.nav_state });
        }
      }
      if (typeof fields.arming_state === 'number') {
        const armed = fields.arming_state === 2;
        if (!arms.length || arms[arms.length - 1].armed !== armed) arms.push({ us, armed });
      }
      // bool 字段解码为 0/1 数字, 按"进入 failsafe"的边沿计次
      const fs = fields.failsafe === true || fields.failsafe === 1;
      if (fs && !lastFS) failsafes.push({ us });
      lastFS = fs;
    }

    if (base === 'vehicle_imu' && Array.isArray(fields.mean_accel)) {
      const a = fields.mean_accel;
      const mag = Math.sqrt((a[0] || 0) ** 2 + (a[1] || 0) ** 2 + (a[2] || 0) ** 2);
      acc.maxVib = Math.max(acc.maxVib, Math.abs(mag - 9.80665));
    }

    if (base === 'vehicle_attitude' && Array.isArray(fields.q)) {
      const e = quatToEuler(fields.q);
      lastAtt = { us, roll: e.roll, pitch: e.pitch };
    }
    if (base === 'vehicle_attitude_setpoint' && lastAtt
        && typeof fields.roll_body === 'number' && typeof fields.pitch_body === 'number') {
      // 两个 topic 记录频率接近, 用最近一条姿态近似对比
      acc.attRollErr = Math.max(acc.attRollErr, Math.abs(lastAtt.roll - fields.roll_body));
      acc.attPitchErr = Math.max(acc.attPitchErr, Math.abs(lastAtt.pitch - fields.pitch_body));
    }

    if (base === 'vehicle_local_position') {
      if (typeof fields.z === 'number') acc.maxRelAlt = acc.maxRelAlt === null ? -fields.z : Math.max(acc.maxRelAlt, -fields.z);
      if (Array.isArray(fields.vel)) {
        const sp = Math.hypot(fields.vel[0] || 0, fields.vel[1] || 0);
        acc.maxSpeed = Math.max(acc.maxSpeed, sp);
      }
    }

    if (base === 'vehicle_gps_position' || base === 'vehicle_gps0') {
      for (const k of ['satellites_used', 'sats_used', 'satellites_visible']) {
        if (typeof fields[k] === 'number') acc.minSats = Math.min(acc.minSats, fields[k]);
      }
      const vn = fields.vel_north !== undefined ? fields.vel_north : fields.vel_n;
      const ve = fields.vel_east !== undefined ? fields.vel_east : fields.vel_e;
      if (typeof vn === 'number' && typeof ve === 'number') {
        acc.maxSpeed = Math.max(acc.maxSpeed, Math.hypot(vn, ve));
      }
    }

    if (base === 'battery_status') {
      const volt = fields.voltage_v !== undefined ? fields.voltage_v : fields.voltage;
      if (typeof volt === 'number' && volt > 0) {
        acc.voltMin = Math.min(acc.voltMin, volt); acc.voltMax = Math.max(acc.voltMax, volt);
      }
      if (typeof fields.current_a === 'number') acc.currMax = Math.max(acc.currMax, fields.current_a);
      if (typeof fields.remaining === 'number') acc.remMin = Math.min(acc.remMin, fields.remaining);
    }

    if (base === 'actuator_motors' && Array.isArray(fields.control)) {
      for (const v of fields.control) if (typeof v === 'number') acc.maxMotors = Math.max(acc.maxMotors, v);
    }
    if (base === 'actuator_outputs' && Array.isArray(fields.outputs)) {
      for (const v of fields.outputs) if (typeof v === 'number') acc.maxRcout = Math.max(acc.maxRcout, v);
    }

    if (base === 'estimator_innovations' && Array.isArray(fields.vel_pos_innov)) {
      for (const v of fields.vel_pos_innov) {
        if (typeof v === 'number' && isFinite(v)) { innovSum += v * v; innovN++; }
      }
    }

    if (EXCERPT_TOPICS.includes(base)) {
      if (!excerpts[base]) excerpts[base] = [];
      if (excerpts[base].length < excerptRows) excerpts[base].push(fields);
    }
  });

  const dur = (acc.firstUS !== null && acc.lastUS !== null)
    ? (acc.lastUS - acc.firstUS) / 1e6 : 0;
  const t0 = acc.firstUS || 0;
  if (innovN) acc.ekfVelPosInnov = Math.sqrt(innovSum / innovN);

  const L = [];
  L.push('# 飞行日志摘要 (PX4 ULog)');
  const hw = defs.info.ver_hw || defs.info['char ver_hw'] || '';
  const sw = defs.info.ver_sw || '';
  if (hw || sw) L.push(`- 硬件/软件: ${hw} ${sw}`.trim());
  L.push(`- 消息记录条数: ${total}`);
  L.push(`- 日志时长: ${fmtNum(dur, 1)} s`);
  L.push(`- 话题(topic)数: ${Object.keys(stats).length}`);
  L.push(`- 记录参数: ${Object.keys(defs.params).length} 项`);
  if (modes.length) {
    L.push(`- 模式切换(${modes.length}次): `
      + modes.slice(0, 30).map((m) => `${fmtNum((m.us - t0) / 1e6, 1)}s=${px4NavStateName(m.nav)}`).join(', '));
  }
  if (arms.length) {
    L.push(`- 解锁/上锁(${arms.length}次): `
      + arms.slice(0, 20).map((a) => `${fmtNum((a.us - t0) / 1e6, 1)}s=${a.armed ? 'ARMED' : 'DISARMED'}`).join(', '));
  }

  L.push('\n## 关键指标');
  if (acc.maxRelAlt !== null) L.push(`- 最大相对高度: ${fmtNum(acc.maxRelAlt, 2)} m`);
  if (acc.maxSpeed > 0) L.push(`- 最大地速: ${fmtNum(acc.maxSpeed, 2)} m/s`);
  if (acc.minSats !== Infinity) L.push(`- 最少卫星数: ${acc.minSats}`);
  if (acc.maxVib > 0) L.push(`- 最大振动(加速度模长偏离 1g): ${fmtNum(acc.maxVib, 2)} m/s²`);
  if (acc.attRollErr || acc.attPitchErr) {
    L.push(`- 姿态跟踪最大误差: Roll ${fmtNum(acc.attRollErr * RAD2DEG, 1)}° / Pitch ${fmtNum(acc.attPitchErr * RAD2DEG, 1)}°`);
  }
  if (acc.voltMin !== Infinity) {
    L.push(`- 电池电压: ${fmtNum(acc.voltMin, 2)} ~ ${fmtNum(acc.voltMax, 2)} V，最大电流 ${fmtNum(acc.currMax, 1)} A，最低剩余 `
      + (acc.remMin === Infinity ? '?' : fmtNum(acc.remMin * (acc.remMin <= 1.5 ? 100 : 1), 0)) + '%');
  }
  if (acc.maxMotors > 0) L.push(`- 电机输出(actuator_motors)峰值: ${fmtNum(acc.maxMotors, 2)} (归一化 0~1)`);
  if (acc.maxRcout > 0) L.push(`- 输出(actuator_outputs)峰值: ${fmtNum(acc.maxRcout, 0)} (PWM)`);
  if (acc.ekfVelPosInnov !== null) L.push(`- EKF 位置/速度新息 RMS: ${fmtNum(acc.ekfVelPosInnov, 3)}`);
  if (defs.dropouts) L.push(`- 记录丢包: ${defs.dropouts} 段 / 共 ${defs.dropMs} ms`);

  const errStrings = defs.strings.filter((s) => s.level <= 53 /* '5' */ && /[Ee]rr|fail|denied|reject|critical/i.test(s.text));
  if (failsafes.length || errStrings.length) {
    L.push(`\n## 异常记录 (failsafe ${failsafes.length} 次 / 疑似错误字符串 ${errStrings.length} 条)`);
    for (const f of failsafes.slice(0, 10)) L.push(`- t=${fmtNum((f.us - t0) / 1e6, 1)}s 进入 failsafe`);
    for (const s of errStrings.slice(0, 20)) L.push(`- t=${fmtNum((s.us - t0) / 1e6, 1)}s [${s.level}] ${s.text}`);
  }

  L.push('\n## 各 topic 字段统计 (min / max / avg / last)');
  const topicNames = Object.keys(stats).sort();
  for (const name of topicNames) {
    const st = stats[name];
    const fields = Object.keys(st.fields);
    if (!fields.length) { L.push(`- ${name}: ${st.count} 条`); continue; }
    const span = (st.firstUS !== null && st.lastUS !== null) ? ((st.lastUS - st.firstUS) / 1e6) : 0;
    const parts = fields.slice(0, 20).map((f) => {
      const s = st.fields[f];
      return `${f}[${fmtNum(s.min, 3)}/${fmtNum(s.max, 3)}/${fmtNum(s.sum / s.n, 3)}/${fmtNum(s.last, 3)}]`;
    });
    L.push(`- ${name} (${st.count}条, ${fmtNum(span, 1)}s): ${parts.join(' ')}`);
  }

  // 关键参数快照 (供 AI 给出具体调参建议)
  const KEY_PARAM_RE = /^(MC_|MPC_|MIS_|NAV_|RTL_|COM_|BAT_|GF_|EKF2_|IMU_|SENS_|CA_)/;
  const keyParams = Object.keys(defs.params).filter((k) => KEY_PARAM_RE.test(k)).sort().slice(0, 60);
  if (keyParams.length) {
    L.push('\n## 关键参数快照 (节选)');
    L.push(keyParams.map((k) => `${k}=${fmtNum(defs.params[k], 4)}`).join(' '));
  }

  if (Object.keys(excerpts).length) {
    L.push('\n## 关键数据片段 (前若干行)');
    for (const name of Object.keys(excerpts)) {
      const rows = excerpts[name];
      if (!rows.length) continue;
      const cols = Object.keys(rows[0]).slice(0, 14);
      L.push(`\n### ${name}`);
      L.push(cols.join(','));
      for (const row of rows) {
        L.push(cols.map((c) => {
          const v = row[c];
          if (typeof v === 'number') return fmtNum(v, 4);
          if (Array.isArray(v)) return '[' + v.map((x) => fmtNum(x, 3)).join(' ') + ']';
          return v;
        }).join(','));
      }
    }
  }

  const text = L.join('\n');
  const metrics = {
    total, duration: dur, messageTypes: topicNames.length,
    modes: modes.length, errors: failsafes.length + errStrings.length,
  };
  return { text, metrics };
}

// ==================== 传感器时序曲线 (供前端画图) ====================
// 输出结构与 loganalysis.extractCharts 一致: [{id,title,series:[{name,unit,t,v}]}]
const CHART_GROUPS = [
  {
    id: 'att', title: '姿态 (°)', topics: ['vehicle_attitude'],
    from: (f, push) => {
      if (!Array.isArray(f.q)) return;
      const e = quatToEuler(f.q);
      push('Roll', '°', e.roll * RAD2DEG);
      push('Pitch', '°', e.pitch * RAD2DEG);
      push('Yaw', '°', e.yaw * RAD2DEG);
    },
  },
  {
    id: 'attsp', title: '姿态设定 (°)', topics: ['vehicle_attitude_setpoint'],
    from: (f, push) => {
      push('DesRoll', '°', f.roll_body);
      push('DesPitch', '°', f.pitch_body);
      push('DesYaw', '°', f.yaw_body);
    },
  },
  {
    id: 'gyro', title: '角速度 (rad/s)', topics: ['vehicle_angular_velocity'],
    from: (f, push) => {
      const v = f.xyz;
      if (Array.isArray(v)) { push('RollRate', 'rad/s', v[0]); push('PitchRate', 'rad/s', v[1]); push('YawRate', 'rad/s', v[2]); }
    },
  },
  {
    id: 'accel', title: '加速度计 (m/s²)', topics: ['vehicle_imu'],
    from: (f, push) => {
      const v = f.mean_accel;
      if (Array.isArray(v)) { push('AccX', 'm/s²', v[0]); push('AccY', 'm/s²', v[1]); push('AccZ', 'm/s²', v[2]); }
    },
  },
  {
    id: 'altbaro', title: '气压高度 (m)', topics: ['vehicle_air_data'],
    from: (f, push) => push('BaroAlt', 'm', f.baro_alt_meter),
  },
  {
    id: 'altpos', title: '相对高度 / 位置 (m)', topics: ['vehicle_local_position'],
    from: (f, push) => {
      push('RelAlt(-z)', 'm', typeof f.z === 'number' ? -f.z : undefined);
      push('X', 'm', f.x);
      push('Y', 'm', f.y);
    },
  },
  {
    id: 'gps', title: 'GPS', topics: ['vehicle_gps_position', 'vehicle_gps0'],
    from: (f, push) => {
      const vn = f.vel_north !== undefined ? f.vel_north : f.vel_n;
      const ve = f.vel_east !== undefined ? f.vel_east : f.vel_e;
      if (typeof vn === 'number' && typeof ve === 'number') push('Spd', 'm/s', Math.hypot(vn, ve));
      const sats = f.satellites_used !== undefined ? f.satellites_used : f.sats_used;
      push('Sats', '颗', sats);
      push('Fix', '', f.fix_type);
      push('EPH', 'm', f.eph);
    },
  },
  {
    id: 'battery', title: '电池', topics: ['battery_status'],
    from: (f, push) => {
      push('Volt', 'V', f.voltage_v !== undefined ? f.voltage_v : f.voltage);
      push('Curr', 'A', f.current_a);
      if (typeof f.remaining === 'number') push('Rem', '%', f.remaining <= 1.5 ? f.remaining * 100 : f.remaining);
    },
  },
  {
    id: 'rcou', title: '电机输出 (归一化)', topics: ['actuator_motors'],
    from: (f, push) => {
      if (!Array.isArray(f.control)) return;
      f.control.forEach((v, i) => { if (i < 6) push('M' + (i + 1), '', v); });
    },
  },
  {
    id: 'rcou_pwm', title: '输出 PWM', topics: ['actuator_outputs'],
    from: (f, push) => {
      if (!Array.isArray(f.outputs)) return;
      f.outputs.forEach((v, i) => { if (i < 6) push('O' + (i + 1), 'PWM', v); });
    },
  },
  {
    id: 'rcin', title: '遥控输入', topics: ['manual_control_setpoint'],
    from: (f, push) => {
      push('Roll', '', f.roll);
      push('Pitch', '', f.pitch);
      push('Throttle', '', f.throttle);
      push('Yaw', '', f.yaw);
    },
  },
  {
    id: 'ekf', title: 'EKF 新息', topics: ['estimator_innovations'],
    from: (f, push) => {
      const v = f.vel_pos_innov;
      if (Array.isArray(v)) v.forEach((x, i) => { if (i < 6) push('VP' + i, '', x); });
    },
  },
];

function downsample(series, maxPoints) {
  const n = series.t.length;
  if (n <= maxPoints) return series;
  const stride = Math.ceil(n / maxPoints);
  const t = []; const v = [];
  for (let i = 0; i < n; i += stride) { t.push(series.t[i]); v.push(series.v[i]); }
  return { name: series.name, unit: series.unit, t, v };
}

/**
 * 从 .ulg 抽取各传感器时序, 输出结构与 loganalysis.extractCharts 相同
 */
export function extractUlogCharts(buf, opts = {}) {
  const maxPoints = opts.maxPoints || 400;
  const byTopic = new Map();
  CHART_GROUPS.forEach((g, gi) => {
    for (const tp of g.topics) {
      if (!byTopic.has(tp)) byTopic.set(tp, []);
      byTopic.get(tp).push({ g, gi });
    }
  });
  const groups = new Map(); // id -> {id,title,order, series:Map}
  let t0 = null;

  const ensure = (g, gi) => {
    let e = groups.get(g.id);
    if (!e) { e = { id: g.id, title: g.title, order: gi, series: new Map() }; groups.set(g.id, e); }
    return e;
  };

  parseUlog(buf, (topic, fields, us) => {
    if (typeof us !== 'number') return;
    const sec = us / 1e6;
    if (t0 === null || sec < t0) t0 = sec;
    const base = topic.split(':')[0];
    const gs = byTopic.get(base);
    if (!gs) return;
    for (const { g, gi } of gs) {
      const e = ensure(g, gi);
      g.from(fields, (name, unit, v) => {
        if (typeof v !== 'number' || !isFinite(v)) return;
        let s = e.series.get(name);
        if (!s) { s = { name, unit, t: [], v: [] }; e.series.set(name, s); }
        s.t.push(sec); s.v.push(v);
      });
    }
  });

  const out = [];
  for (const e of [...groups.values()].sort((a, b) => a.order - b.order)) {
    const series = [];
    for (const s of e.series.values()) {
      if (s.t.length < 2) continue;
      const ds = downsample(s, maxPoints);
      ds.t = ds.t.map((x) => Number((x - (t0 || 0)).toFixed(3)));
      series.push(ds);
    }
    if (series.length) out.push({ id: e.id, title: e.title, series });
  }
  return out;
}
