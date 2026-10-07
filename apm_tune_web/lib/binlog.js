// ArduPilot DataFlash (.bin) 日志解析器 (Node, 无依赖)
// 结构: 每条记录 = 0xA3 0x95 <type> + FMT 定义的字段
// 类型/尺寸映射对齐 pymavlink DFReader.FORMAT_TO_STRUCT

export const HEAD1 = 0xA3;
export const HEAD2 = 0x95;
export const FMT_TYPE = 0x80;

// code: [kind, size, multiplier]
// kind: i=int, u=uint, f=float, s=string, h=half
export const FORMAT_TO_STRUCT = {
  a: { kind: 's', size: 64 },
  b: { kind: 'i', size: 1 },
  B: { kind: 'u', size: 1 },
  g: { kind: 'h', size: 2 },
  h: { kind: 'i', size: 2 },
  H: { kind: 'u', size: 2 },
  i: { kind: 'i', size: 4 },
  I: { kind: 'u', size: 4 },
  f: { kind: 'f', size: 4 },
  n: { kind: 's', size: 4 },
  N: { kind: 's', size: 16 },
  Z: { kind: 's', size: 64 },
  c: { kind: 'i', size: 2, mul: 0.01 },
  C: { kind: 'u', size: 2, mul: 0.01 },
  e: { kind: 'i', size: 4, mul: 0.01 },
  E: { kind: 'u', size: 4, mul: 0.01 },
  L: { kind: 'i', size: 4, mul: 1.0e-7 },
  d: { kind: 'f', size: 8 },
  M: { kind: 'i', size: 1 },
  q: { kind: 'i', size: 8, big: true },
  Q: { kind: 'u', size: 8, big: true },
};

function halfToFloat(h) {
  const s = (h & 0x8000) >> 15;
  const e = (h & 0x7C00) >> 10;
  const f = h & 0x03FF;
  if (e === 0) return (s ? -1 : 1) * Math.pow(2, -14) * (f / 1024);
  if (e === 31) return f ? NaN : (s ? -Infinity : Infinity);
  return (s ? -1 : 1) * Math.pow(2, e - 15) * (1 + f / 1024);
}

function readField(buf, off, code) {
  const t = FORMAT_TO_STRUCT[code];
  if (!t) throw new Error('未知格式字符: ' + code);
  switch (t.kind) {
    case 'i':
      if (t.size === 1) return buf.readInt8(off);
      if (t.size === 2) return buf.readInt16LE(off);
      if (t.size === 8) return Number(buf.readBigInt64LE(off));
      return buf.readInt32LE(off);
    case 'u':
      if (t.size === 1) return buf.readUInt8(off);
      if (t.size === 2) return buf.readUInt16LE(off);
      if (t.size === 8) return Number(buf.readBigUInt64LE(off));
      return buf.readUInt32LE(off);
    case 'f':
      return t.size === 8 ? buf.readDoubleLE(off) : buf.readFloatLE(off);
    case 'h':
      return halfToFloat(buf.readUInt16LE(off));
    case 's': {
      let end = off;
      const limit = off + t.size;
      while (end < limit && end < buf.length && buf[end] !== 0) end++;
      return buf.toString('ascii', off, end);
    }
    default:
      throw new Error('未知字段类型: ' + t.kind);
  }
}

function buildFormat(type, name, length, format, columns) {
  const cols = (columns || '').split(',').filter((c) => c !== '');
  const items = [];
  let off = 0;
  for (const c of format) {
    const t = FORMAT_TO_STRUCT[c];
    if (!t) throw new Error(`消息 ${name} 含不支持的格式字符 '${c}'`);
    items.push({ code: c, t, off });
    off += t.size;
  }
  return { type, name, length, format, columns: cols, items, payloadSize: off };
}

const FMT_FORMAT = buildFormat(FMT_TYPE, 'FMT', 89, 'BBnNZ', 'Type,Length,Name,Format,Columns');

function decodeRecord(buf, fmt, start) {
  const out = {};
  for (let i = 0; i < fmt.items.length; i++) {
    const it = fmt.items[i];
    let v = readField(buf, start + it.off, it.code);
    if (it.t.mul !== undefined && typeof v === 'number') v *= it.t.mul;
    const col = fmt.columns[i] !== undefined ? fmt.columns[i] : ('C' + i);
    out[col] = v;
  }
  return out;
}

/**
 * 解析 DataFlash 日志。
 * @param {Buffer} buf
 * @param {(name:string, fields:object, type:number)=>void} [onRecord]
 * @returns {{records:number, resync:number, reconnects:number, formats:number, errors:string[]}}
 */
export function parseBin(buf, onRecord) {
  const formats = new Map();
  formats.set(FMT_TYPE, FMT_FORMAT);
  const stats = { records: 0, resync: 0, reconnects: 0, formats: 1, errors: [] };
  let i = 0;
  const n = buf.length;

  while (i + 3 <= n) {
    if (buf[i] !== HEAD1 || buf[i + 1] !== HEAD2) {
      i++;
      stats.resync++;
      continue;
    }
    const type = buf[i + 2];
    const fmt = formats.get(type);
    if (!fmt) {
      i++;
      continue;
    }
    const len = fmt.length;
    if (len < 3) { i++; continue; }
    if (i + len > n) break; // 截断

    let fields;
    try {
      fields = decodeRecord(buf, fmt, i + 3);
    } catch (err) {
      stats.errors.push(err.message);
      i += 3;
      continue;
    }

    if (type === FMT_TYPE) {
      try {
        const ftype = fields.Type;
        const fname = fields.Name;
        const flen = fields.Length;
        const fformat = fields.Format;
        const fcols = fields.Columns;
        if (typeof ftype === 'number' && fformat) {
          const old = formats.get(ftype);
          const nf = buildFormat(ftype, fname, flen, fformat, fcols);
          if (old && old.payloadSize < nf.payloadSize) stats.reconnects++;
          formats.set(ftype, nf);
          if (old) stats.formats++;
        }
      } catch (err) {
        stats.errors.push('FMT: ' + err.message);
      }
    } else {
      stats.records++;
      if (onRecord) onRecord(fmt.name, fields, type);
    }
    i += len;
  }

  return stats;
}

export { buildFormat, decodeRecord, FMT_FORMAT };
