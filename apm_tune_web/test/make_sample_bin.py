#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成样例 DataFlash .bin 并用 pymavlink 读取为期望值, 供 Node 解析器对拍。
  python3 test/make_sample_bin.py
输出: test/golden/sample.bin, test/golden/sample_expected.json
"""
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
GOLD = os.path.join(HERE, 'golden')

# 与 pymavlink DFReader.FORMAT_TO_STRUCT 对应的 struct 码
STRUCT = {
    'a': '64s', 'b': 'b', 'B': 'B', 'g': 'e', 'h': 'h', 'H': 'H',
    'i': 'i', 'I': 'I', 'f': 'f', 'n': '4s', 'N': '16s', 'Z': '64s',
    'c': 'h', 'C': 'H', 'e': 'i', 'E': 'I', 'L': 'i', 'd': 'd',
    'M': 'b', 'q': 'q', 'Q': 'Q',
}


def enc_str(s, size):
    b = s.encode('ascii')[:size - 1]
    return b + b'\x00' * (size - len(b))


def enc_name(s):
    # 'n' = 4s, DataFlash 名称用满 4 字节, 不预留终止符
    b = s.encode('ascii')[:4]
    return b + b'\x00' * (4 - len(b))


def fmt_record(ftype, length, name, fmt, columns):
    rec = struct.pack('<BB', ftype, length)
    rec += enc_name(name)
    rec += enc_str(fmt, 16)
    rec += enc_str(columns, 64)
    return struct.pack('<BBB', 0xA3, 0x95, 0x80) + rec


def msg_record(ftype, fmt, values):
    sfmt = '<' + ''.join(STRUCT[c] for c in fmt)
    return struct.pack('<BBB', 0xA3, 0x95, ftype) + struct.pack(sfmt, *values)


def main():
    os.makedirs(GOLD, exist_ok=True)
    out = bytearray()

    # TEST: 基础数值类型 (含 half)
    fmt_test = 'QBbhiIfdg'
    cols_test = 'TimeUS,u8,i8,i16,i32,u32,f32,f64,half'
    length_test = 3 + struct.calcsize('<' + ''.join(STRUCT[c] for c in fmt_test))
    out += fmt_record(200, length_test, 'TEST', fmt_test, cols_test)

    # MULT: 带倍率类型
    fmt_mult = 'QcCeEL'
    cols_mult = 'TimeUS,c,C,e,E,L'
    length_mult = 3 + struct.calcsize('<' + ''.join(STRUCT[c] for c in fmt_mult))
    out += fmt_record(201, length_mult, 'MULT', fmt_mult, cols_mult)

    # STR: 字符串类型
    fmt_str = 'QnNZ'
    cols_str = 'TimeUS,n,N,Z'
    length_str = 3 + struct.calcsize('<' + ''.join(STRUCT[c] for c in fmt_str))
    out += fmt_record(202, length_str, 'STR', fmt_str, cols_str)

    out += msg_record(200, fmt_test, [1000000, 200, -5, -1234, -100000, 4000000000, 1.5, -2.25, 3.5])
    out += msg_record(201, fmt_mult, [1001000, 123, 456, 789, 101112, 399000000])
    out += msg_record(202, fmt_str, [1002000, b'ab', b'hello', b'world'])
    out += msg_record(200, fmt_test, [2000000, 1, 2, 3, 4, 5, 7.5, 8.25, -0.5])

    bin_path = os.path.join(GOLD, 'sample.bin')
    with open(bin_path, 'wb') as f:
        f.write(out)

    # 用 pymavlink 读取作为期望
    sys.path.insert(0, ROOT)
    from pymavlink import DFReader
    recs = []
    reader = DFReader.DFReader_binary(bin_path)
    while True:
        m = reader.recv_msg()
        if m is None:
            break
        if m.get_type() == 'FMT':
            continue
        d = m.to_dict()
        d.pop('mavpackettype', None)
        recs.append({'name': m.get_type(), 'fields': d})

    exp_path = os.path.join(GOLD, 'sample_expected.json')
    with open(exp_path, 'w', encoding='utf-8') as f:
        json.dump(recs, f, ensure_ascii=False, indent=1)

    print('已写出 %s (%d 字节) 与 %s (%d 条记录)'
          % (bin_path, len(out), exp_path, len(recs)))


if __name__ == '__main__':
    main()
