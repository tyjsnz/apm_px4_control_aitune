// 浏览器端 Buffer 最小实现 (仅覆盖 lib/mavlink.js / lib/paramlink.js / lib/fcops.js 用到的 API)
// 基于 Uint8Array + DataView, 无任何依赖。Node 环境下勿加载(用原生 Buffer)。
// 用法: <script src="vendor/buffer-shim.js"></script>  →  window.Buffer
(function (global) {
  'use strict';

  const textEncoder = new TextEncoder();
  const textDecoder = new TextDecoder('utf-8');

  class BufferShim extends Uint8Array {
    // ---------- 构造 ----------
    static alloc(size) {
      return new BufferShim(new Uint8Array(size | 0));
    }

    static from(value, encodingOrOffset, length) {
      if (typeof value === 'string') {
        const enc = (encodingOrOffset || 'utf8').toLowerCase();
        if (enc === 'ascii' || enc === 'latin1' || enc === 'binary') {
          const out = new Uint8Array(value.length);
          for (let i = 0; i < value.length; i++) out[i] = value.charCodeAt(i) & 0x7f;
          return new BufferShim(out);
        }
        return new BufferShim(textEncoder.encode(value)); // utf8 / utf-8
      }
      if (value instanceof ArrayBuffer) {
        return new BufferShim(new Uint8Array(value, encodingOrOffset || 0, length));
      }
      if (ArrayBuffer.isView(value)) {
        // TypedArray / DataView / BufferShim → 拷贝 (与 Node Buffer.from 语义一致)
        return new BufferShim(new Uint8Array(value.buffer, value.byteOffset, value.byteLength));
      }
      if (Array.isArray(value)) {
        return new BufferShim(new Uint8Array(value));
      }
      throw new TypeError('Buffer.from: 不支持的类型 ' + typeof value);
    }

    static concat(list, totalLength) {
      if (!Array.isArray(list)) throw new TypeError('Buffer.concat: 需要数组');
      const len = totalLength === undefined
        ? list.reduce((s, b) => s + b.length, 0)
        : totalLength;
      const out = new Uint8Array(len);
      let off = 0;
      for (const b of list) {
        if (off + b.length > len) { out.set(b.subarray ? b.subarray(0, len - off) : b, off); break; }
        out.set(b, off);
        off += b.length;
      }
      return new BufferShim(out);
    }

    static isBuffer(b) { return b instanceof BufferShim; }

    // ---------- 视图 / 拷贝 ----------
    subarray(start, end) {
      // Node Buffer.subarray 共享内存; 此处返回独立拷贝 (本项目用法安全)
      return new BufferShim(super.subarray(start, end));
    }

    copy(target, targetStart, sourceStart, sourceEnd) {
      const s = sourceStart === undefined ? 0 : sourceStart;
      const e = sourceEnd === undefined ? this.length : sourceEnd;
      const n = Math.max(0, Math.min(e, this.length) - s);
      if (n > 0) target.set(this.subarray(s, s + n), targetStart || 0);
      return n;
    }

    fill(value, start, end) {
      super.fill(value, start === undefined ? 0 : start, end === undefined ? this.length : end);
      return this;
    }

    toString(encoding, start, end) {
      const enc = (encoding || 'utf8').toLowerCase();
      const s = start === undefined ? 0 : start;
      const e = end === undefined ? this.length : Math.min(end, this.length);
      if (e <= s) return '';
      if (enc === 'ascii' || enc === 'latin1' || enc === 'binary') {
        let out = '';
        for (let i = s; i < e; i++) out += String.fromCharCode(this[i] & 0x7f);
        return out;
      }
      return textDecoder.decode(this.subarray(s, e));
    }

    // ---------- 读 (小端) ----------
    _dv() { return new DataView(this.buffer, this.byteOffset, this.byteLength); }

    readUInt8(offset) { return this._dv().getUint8(offset); }
    readInt8(offset) { return this._dv().getInt8(offset); }
    readUInt16LE(offset) { return this._dv().getUint16(offset, true); }
    readInt16LE(offset) { return this._dv().getInt16(offset, true); }
    readUInt32LE(offset) { return this._dv().getUint32(offset, true); }
    readInt32LE(offset) { return this._dv().getInt32(offset, true); }
    readFloatLE(offset) { return this._dv().getFloat32(offset, true); }
    readDoubleLE(offset) { return this._dv().getFloat64(offset, true); }
    readBigUInt64LE(offset) { return this._dv().getBigUint64(offset, true); }
    readBigInt64LE(offset) { return this._dv().getBigInt64(offset, true); }

    // ---------- 写 (小端) ----------
    writeUInt8(value, offset) { this._dv().setUint8(offset, value); return offset + 1; }
    writeInt8(value, offset) { this._dv().setInt8(offset, value); return offset + 1; }
    writeUInt16LE(value, offset) { this._dv().setUint16(offset, value, true); return offset + 2; }
    writeInt16LE(value, offset) { this._dv().setInt16(offset, value, true); return offset + 2; }
    writeUInt32LE(value, offset) { this._dv().setUint32(offset, value, true); return offset + 4; }
    writeInt32LE(value, offset) { this._dv().setInt32(offset, value, true); return offset + 4; }
    writeFloatLE(value, offset) { this._dv().setFloat32(offset, value, true); return offset + 4; }
    writeDoubleLE(value, offset) { this._dv().setFloat64(offset, value, true); return offset + 8; }
    writeBigUInt64LE(value, offset) { this._dv().setBigUint64(offset, BigInt(value), true); return offset + 8; }
    writeBigInt64LE(value, offset) { this._dv().setBigInt64(offset, BigInt(value), true); return offset + 8; }
  }

  // Node 兼容: Buffer.isBuffer / Buffer.alloc / Buffer.from / Buffer.concat 已覆盖主要入口
  global.Buffer = BufferShim;
  global.__BufferShim = BufferShim;
})(typeof window !== 'undefined' ? window : globalThis);
