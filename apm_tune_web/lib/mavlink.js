// MAVLink v1.0 / v2.0 编解码与流式解析 (Node, ESM)
// - CRC-16/X.25, 计入消息 crc_extra
// - 支持 v1(0xFE) 与 v2(0xFD), v2 签名帧(10+len+2+13)可跳过
// - 字段布局来自 lib/mavlink_defs.js (由 pymavlink 生成)
import { MAVLINK_DEFS, MSG_NAME_BY_ID } from './mavlink_defs.js';

export { MAVLINK_DEFS, MSG_NAME_BY_ID } from './mavlink_defs.js';

export const STX_V1 = 0xFE;
export const STX_V2 = 0xFD;
export const MAVLINK_IFLAG_SIGNED = 0x01;

const TYPE_SIZE = {
  u8: 1, i8: 1, u16: 2, i16: 2, u32: 4, i32: 4,
  f32: 4, f64: 8, i64: 8, u64: 8, bool: 1,
};

// ---------- CRC-16/X.25 ----------
export function crcAccumulate(byte, crc) {
  let tmp = (byte ^ (crc & 0xff)) & 0xff;
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

export function crcCalculate(bytes, extra) {
  let crc = 0xFFFF;
  for (let i = 0; i < bytes.length; i++) crc = crcAccumulate(bytes[i], crc);
  if (extra !== undefined && extra !== null) crc = crcAccumulate(extra & 0xff, crc);
  return crc;
}

// ---------- 字段尺寸 ----------
export function payloadSize(def) {
  let n = 0;
  for (const f of def.fields) {
    if (f.type === 'char') n += f.len || 0;
    else if (f.len > 0) n += TYPE_SIZE[f.type] * f.len;
    else n += TYPE_SIZE[f.type];
  }
  return n;
}

// ---------- payload 解码 -> 字段对象 ----------
// 短载荷(MAVLink2 尾零裁剪 / v1.0 缺扩展字段)按全 0 补齐
export function decodePayload(msgId, payload) {
  const name = MSG_NAME_BY_ID[msgId];
  const def = name ? MAVLINK_DEFS[name] : null;
  if (!def) return null;
  const full = payloadSize(def);
  if (payload.length < full) {
    const padded = Buffer.alloc(full);
    payload.copy(padded);
    payload = padded;
  }
  const out = {};
  let off = 0;
  for (const f of def.fields) {
    if (f.type === 'char') {
      const len = f.len || 0;
      let end = off;
      while (end < off + len && end < payload.length && payload[end] !== 0) end++;
      out[f.name] = payload.toString('ascii', off, end);
      off += len;
      continue;
    }
    if (f.type === 'bool') {
      out[f.name] = payload.readUInt8(off) !== 0;
      off += 1;
      continue;
    }
    const sz = TYPE_SIZE[f.type];
    if (f.len > 0) {
      const arr = [];
      for (let i = 0; i < f.len; i++) {
        arr.push(readScalar(payload, off + i * sz, f.type));
      }
      out[f.name] = arr;
      off += sz * f.len;
    } else {
      out[f.name] = readScalar(payload, off, f.type);
      off += sz;
    }
  }
  return out;
}

function readScalar(buf, off, type) {
  switch (type) {
    case 'u8': return buf.readUInt8(off);
    case 'i8': return buf.readInt8(off);
    case 'u16': return buf.readUInt16LE(off);
    case 'i16': return buf.readInt16LE(off);
    case 'u32': return buf.readUInt32LE(off);
    case 'i32': return buf.readInt32LE(off);
    case 'f32': return buf.readFloatLE(off);
    case 'f64': return buf.readDoubleLE(off);
    case 'u64': return Number(buf.readBigUInt64LE(off));
    case 'i64': return Number(buf.readBigInt64LE(off));
    default: throw new Error('未知字段类型: ' + type);
  }
}

// ---------- 字段对象 -> payload 编码 ----------
// version=1 时截断到 v1.0 基准字段长度(去掉 MAVLink2 扩展字段)
export function encodePayload(name, fields, version = 2) {
  const def = MAVLINK_DEFS[name];
  if (!def) throw new Error('未知消息: ' + name);
  const size = payloadSize(def);
  const buf = Buffer.alloc(size);
  let off = 0;
  for (const f of def.fields) {
    const v = fields[f.name];
    if (f.type === 'char') {
      const len = f.len || 0;
      writeChar(buf, off, len, v);
      off += len;
      continue;
    }
    if (f.type === 'bool') {
      buf.writeUInt8(v ? 1 : 0, off);
      off += 1;
      continue;
    }
    const sz = TYPE_SIZE[f.type];
    if (f.len > 0) {
      const arr = Array.isArray(v) ? v : [];
      for (let i = 0; i < f.len; i++) {
        writeScalar(buf, off + i * sz, f.type, arr[i] === undefined ? 0 : arr[i]);
      }
      off += sz * f.len;
    } else {
      writeScalar(buf, off, f.type, v === undefined ? 0 : v);
      off += sz;
    }
  }
  if (version === 1) {
    if (def.v1Size === null || def.v1Size === undefined) {
      throw new Error(name + ' 不支持 MAVLink v1.0');
    }
    return buf.subarray(0, def.v1Size);
  }
  return buf;
}

function writeChar(buf, off, len, v) {
  buf.fill(0, off, off + len);
  if (v === undefined || v === null) return;
  const src = Buffer.isBuffer(v) ? v : Buffer.from(String(v), 'ascii');
  src.copy(buf, off, 0, Math.min(src.length, len));
}

function writeScalar(buf, off, type, v) {
  const num = typeof v === 'number' ? v : Number(v) || 0;
  switch (type) {
    case 'u8': buf.writeUInt8(num & 0xff, off); break;
    case 'i8': buf.writeInt8(num, off); break;
    case 'u16': buf.writeUInt16LE(num & 0xffff, off); break;
    case 'i16': buf.writeInt16LE(num, off); break;
    case 'u32': buf.writeUInt32LE(num >>> 0, off); break;
    case 'i32': buf.writeInt32LE(num | 0, off); break;
    case 'f32': buf.writeFloatLE(num, off); break;
    case 'f64': buf.writeDoubleLE(num, off); break;
    case 'u64': buf.writeBigUInt64LE(BigInt(Math.trunc(num)), off); break;
    case 'i64': buf.writeBigInt64LE(BigInt(Math.trunc(num)), off); break;
    default: throw new Error('未知字段类型: ' + type);
  }
}

// ---------- 组帧 ----------
export function encodeFrame(msgId, crcExtra, payload, opts = {}) {
  const version = opts.version === 1 ? 1 : 2;
  const seq = opts.seq & 0xff;
  const sysid = opts.sysid === undefined ? 255 : opts.sysid;
  const compid = opts.compid === undefined ? 190 : opts.compid;
  const payloadLen = payload.length;

  if (version === 1) {
    if (msgId > 255) throw new Error('v1 不支持 msgid>255: ' + msgId);
    const buf = Buffer.alloc(6 + payloadLen + 2);
    buf[0] = STX_V1;
    buf[1] = payloadLen;
    buf[2] = seq;
    buf[3] = sysid;
    buf[4] = compid;
    buf[5] = msgId & 0xff;
    payload.copy(buf, 6);
    let crc = 0xFFFF;
    for (let i = 1; i <= 5 + payloadLen; i++) crc = crcAccumulate(buf[i], crc);
    crc = crcAccumulate(crcExtra & 0xff, crc);
    buf[6 + payloadLen] = crc & 0xff;
    buf[6 + payloadLen + 1] = (crc >> 8) & 0xff;
    return buf;
  }

  const buf = Buffer.alloc(10 + payloadLen + 2);
  buf[0] = STX_V2;
  buf[1] = payloadLen;          // 未做尾零裁剪, 全量发送
  buf[2] = 0;                   // incompat_flags
  buf[3] = 0;                   // compat_flags
  buf[4] = seq;
  buf[5] = sysid;
  buf[6] = compid;
  buf[7] = msgId & 0xff;
  buf[8] = (msgId >> 8) & 0xff;
  buf[9] = (msgId >> 16) & 0xff;
  payload.copy(buf, 10);
  let crc = 0xFFFF;
  for (let i = 1; i <= 9 + payloadLen; i++) crc = crcAccumulate(buf[i], crc);
  crc = crcAccumulate(crcExtra & 0xff, crc);
  buf[10 + payloadLen] = crc & 0xff;
  buf[10 + payloadLen + 1] = (crc >> 8) & 0xff;
  return buf;
}

// ---------- 流式解析 ----------
// 事件: { version, msgId, name, sysid, compid, seq, payload, fields, complete }
// fields 为解码后的对象; 未收录的消息 fields=null, 仍可转发原始 payload
export class Parser {
  constructor() {
    this.buf = Buffer.alloc(0);
    this.badFrames = 0;
    this.goodFrames = 0;
    this.lastVersion = null;
    this.lastSysId = null;
    this.lastCompId = null;
    this.messages = 0;
  }

  feed(chunk) {
    if (!chunk || chunk.length === 0) return [];
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : Buffer.from(chunk);
    const out = [];
    let i = 0;
    const buf = this.buf;

    while (i < buf.length) {
      const stx = buf[i];
      if (stx !== STX_V1 && stx !== STX_V2) {
        i++;
        continue;
      }
      const version = stx === STX_V2 ? 2 : 1;
      const headerLen = version === 2 ? 10 : 6;
      if (i + headerLen > buf.length) break; // 头不完整

      const payloadLen = buf[i + 1];
      let incompat = 0;
      if (version === 2) incompat = buf[i + 2];
      const signed = version === 2 && (incompat & MAVLINK_IFLAG_SIGNED);
      const sigLen = signed ? 13 : 0;
      const total = headerLen + payloadLen + 2 + sigLen;
      if (i + total > buf.length) break; // 帧不完整, 等更多数据

      let msgId, sysid, compid, seq, payloadStart;
      if (version === 2) {
        seq = buf[i + 4];
        sysid = buf[i + 5];
        compid = buf[i + 6];
        msgId = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16);
        payloadStart = i + 10;
      } else {
        seq = buf[i + 2];
        sysid = buf[i + 3];
        compid = buf[i + 4];
        msgId = buf[i + 5];
        payloadStart = i + 6;
      }

      const name = MSG_NAME_BY_ID[msgId];
      const def = name ? MAVLINK_DEFS[name] : null;
      const crcByteOff = payloadStart + payloadLen;
      const crcGot = buf[crcByteOff] | (buf[crcByteOff + 1] << 8);

      let valid;
      if (def) {
        let crc = 0xFFFF;
        for (let k = i + 1; k < crcByteOff; k++) crc = crcAccumulate(buf[k], crc);
        crc = crcAccumulate(def.crcExtra & 0xff, crc);
        valid = crc === crcGot;
      } else {
        // 未收录消息: 无法校验(缺 crc_extra), 直接放行原始 payload
        valid = true;
      }

      if (!valid) {
        this.badFrames++;
        i += 1; // 重新同步
        continue;
      }

      const raw = buf.subarray(payloadStart, payloadStart + payloadLen);
      const fields = def ? decodePayload(msgId, raw) : null;
      // 统一输出补齐后的完整载荷, 便于消费端按固定布局解析
      let payload;
      if (def && raw.length < payloadSize(def)) {
        payload = Buffer.alloc(payloadSize(def));
        raw.copy(payload);
      } else {
        payload = Buffer.from(raw);
      }
      this.goodFrames++;
      this.messages++;
      this.lastVersion = version;
      this.lastSysId = sysid;
      this.lastCompId = compid;
      out.push({
        version, msgId, name: name || null, sysid, compid, seq,
        payload, fields, signed,
      });
      i += total;
    }

    this.buf = i >= buf.length ? Buffer.alloc(0) : Buffer.from(buf.subarray(i));
    // 防御: 缓冲区过大(无有效帧)时截断
    if (this.buf.length > 8192) {
      const tail = this.buf.subarray(this.buf.length - 2048);
      this.badFrames++;
      this.buf = Buffer.from(tail);
    }
    return out;
  }

  reset() {
    this.buf = Buffer.alloc(0);
  }
}

// ---------- 链路封装: 发送 + 解析 + 自适应版本 ----------
export class MavlinkLink {
  /**
   * @param {object} opts
   * @param {(buf:Buffer)=>void} opts.write   底层写函数(串口/WS)
   * @param {(msg:object)=>void} [opts.onMessage] 每收到一条消息回调
   */
  constructor(opts = {}) {
    this.write = opts.write || (() => {});
    this.onMessage = opts.onMessage || null;
    this.srcSys = opts.srcSys === undefined ? 255 : opts.srcSys;
    this.srcComp = opts.srcComp === undefined ? 190 : opts.srcComp;
    this.targetSys = opts.targetSys || 1;
    this.targetComp = opts.targetComp || 1;
    this.seq = 0;
    // 未收到任何帧前默认用 v2 (ArduPilot 现代固件均支持)
    this.version = 2;
    this.parser = new Parser();
  }

  setTarget(sysid, compid) {
    if (sysid) this.targetSys = sysid;
    if (compid) this.targetComp = compid;
  }

  feed(chunk) {
    const msgs = this.parser.feed(chunk);
    for (const m of msgs) {
      if (m.sysid && m.sysid !== 0 && m.sysid !== this.srcSys) {
        this.targetSys = m.sysid;
        this.targetComp = m.compid || this.targetComp;
      }
      this.version = m.version;
      if (this.onMessage) this.onMessage(m);
    }
    return msgs;
  }

  /** 发送已收录消息 */
  send(name, fields, opts = {}) {
    const def = MAVLINK_DEFS[name];
    if (!def) throw new Error('未知消息: ' + name);
    let version = opts.version || this.version || 2;
    if (def.id > 255 || def.v1Size === null) version = 2;
    const payload = encodePayload(name, fields, version);
    return this.sendId(def.id, def.crcExtra, payload, { ...opts, version });
  }

  /** 发送任意 payload */
  sendId(msgId, crcExtra, payload, opts = {}) {
    let version = opts.version || this.version || 2;
    if (msgId > 255) version = 2; // v1 无法承载
    const frame = encodeFrame(msgId, crcExtra, payload, {
      version,
      seq: this.seq++,
      sysid: opts.sysid === undefined ? this.srcSys : opts.sysid,
      compid: opts.compid === undefined ? this.srcComp : opts.compid,
    });
    this.write(frame);
    return frame;
  }

  reset() {
    this.parser.reset();
  }
}

export function findDef(name) {
  return MAVLINK_DEFS[name] || null;
}
