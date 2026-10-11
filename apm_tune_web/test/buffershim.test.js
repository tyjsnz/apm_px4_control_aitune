// Buffer Shim 单测: public/vendor/buffer-shim.js 与 Node 原生 Buffer 行为对拍
//   node test/buffershim.test.js
// 覆盖: 构造/拷贝/切片/编解码 + lib/mavlink.js 在 shim 下的组帧/解析往返
import { Buffer as RealBuffer } from 'buffer';
import '../public/vendor/buffer-shim.js';

const Shim = globalThis.__BufferShim;
let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}
const eq = (a, b) => a.length === b.length && a.every((v, i) => v === b[i]);

function parityTests() {
  // alloc / from / concat
  const a = Shim.alloc(8);
  const ra = RealBuffer.alloc(8);
  check('alloc 内容一致', eq([...a], [...ra]) && a.length === 8);
  check('alloc 是 Uint8Array', a instanceof Uint8Array);

  const s = 'ATC_RAT_RLL_P';
  check('from(string, ascii) 一致',
    eq([...Shim.from(s, 'ascii')], [...RealBuffer.from(s, 'ascii')]));
  const u8 = new Uint8Array([0, 1, 127, 128, 255]);
  check('from(Uint8Array) 为拷贝', eq([...Shim.from(u8)], [...RealBuffer.from(u8)]));
  const src = Shim.from([1, 2, 3]);
  src[0] = 9;
  check('from(Uint8Array) 拷贝语义', u8[0] === 0, String(u8[0]));

  const c = Shim.concat([Shim.from([1, 2]), Shim.from([3, 4])]);
  check('concat 一致', eq([...c], [1, 2, 3, 4]) && c.length === 4);

  // subarray / copy / fill / toString
  const sub = Shim.from([10, 20, 30, 40]).subarray(1, 3);
  check('subarray 一致', eq([...sub], [20, 30]));
  const tgt = Shim.alloc(4);
  Shim.from([7, 8, 9]).copy(tgt, 1);
  check('copy 一致', eq([...tgt], [0, 7, 8, 9]));
  const fl = Shim.alloc(4);
  fl.fill(5, 1, 3);
  check('fill 一致', eq([...fl], [0, 5, 5, 0]));
  const utf = Shim.from('调参✓', 'utf8');
  check('utf8 编解码往返', utf.toString('utf8') === '调参✓', utf.toString('utf8'));
  check('isBuffer', Shim.isBuffer(utf) && !Shim.isBuffer(u8));

  // 读写对拍 (各类偏移)
  const buf = Shim.alloc(32);
  const rbuf = RealBuffer.alloc(32);
  const cases = [
    ['UInt8', 0xfe, 1], ['Int8', -5, 3], ['UInt16LE', 0xbeef, 4], ['Int16LE', -1234, 7],
    ['UInt32LE', 0xdeadbeef, 9], ['Int32LE', -1234567, 13], ['FloatLE', 3.14159, 17],
    ['DoubleLE', -2.718281828, 21],
  ];
  for (const [name, val, off] of cases) {
    buf['write' + name](val, off);
    rbuf['write' + name](val, off);
    const got = buf['read' + name](off);
    const ref = rbuf['read' + name](off);
    check(`write/read ${name}`, got === ref && eq([...buf], [...rbuf]), `${got} vs ${ref}`);
  }
  buf.writeBigUInt64LE(123456789012345n, 1);
  rbuf.writeBigUInt64LE(123456789012345n, 1);
  check('readBigUInt64LE 一致', buf.readBigUInt64LE(1) === rbuf.readBigUInt64LE(1)
    && eq([...buf], [...rbuf]));
  buf.writeBigInt64LE(-42n, 9);
  rbuf.writeBigInt64LE(-42n, 9);
  check('readBigInt64LE 一致', buf.readBigInt64LE(9) === rbuf.readBigInt64LE(9));
}

async function mavlinkTests() {
  const mav = await import('../lib/mavlink.js');
  const fields = {
    param_value: 0.15, target_system: 1, target_component: 1,
    param_id: 'MOT_SPIN_ARM', param_type: 9,
  };

  // 真 Buffer 组帧作为参考
  globalThis.Buffer = RealBuffer;
  const refFrame = mav.MavlinkLink.prototype.send.call(
    new mav.MavlinkLink({ write: () => {} }), 'PARAM_SET', fields);

  // shim Buffer 重新组帧并比对字节
  globalThis.Buffer = Shim;
  const shimFrame = mav.MavlinkLink.prototype.send.call(
    new mav.MavlinkLink({ write: () => {} }), 'PARAM_SET', fields);
  check('shim 组帧与原生 Buffer 字节一致', eq([...shimFrame], [...refFrame]),
    `${shimFrame.length} vs ${refFrame.length}`);

  // shim 解析原生帧
  const parser = new mav.Parser();
  const msgs = parser.feed(RealBuffer.from([...refFrame]));
  check('shim 解析出 1 条消息', msgs.length === 1, String(msgs.length));
  const m = msgs[0];
  check('shim 解析 CRC 有效', parser.badFrames === 0 && parser.goodFrames === 1,
    `bad=${parser.badFrames} good=${parser.goodFrames}`);
  check('字段往返一致',
    m.name === 'PARAM_SET'
    && m.fields.param_id === 'MOT_SPIN_ARM'
    && Math.abs(m.fields.param_value - 0.15) < 1e-6
    && m.fields.param_type === 9,
    JSON.stringify(m.fields));

  // 分片喂入 (模拟串口字节流)
  const bytes = RealBuffer.from([...refFrame]);
  const p2 = new mav.Parser();
  let all = [];
  for (let i = 0; i < bytes.length; i += 3) all = all.concat(p2.feed(Shim.from(bytes.subarray(i, i + 3))));
  check('分片喂入完整解析', all.length === 1 && p2.badFrames === 0, String(all.length));

  globalThis.Buffer = RealBuffer;
}

parityTests();
mavlinkTests().then(() => {
  console.log(failures === 0 ? 'Buffer Shim 单测通过' : failures + ' 项失败');
  process.exit(failures === 0 ? 0 : 1);
}).catch((e) => {
  console.error('Buffer Shim 单测异常:', e);
  globalThis.Buffer = RealBuffer;
  process.exit(1);
});
