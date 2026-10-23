#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 pymavlink 导出 MAVLink 消息定义 -> lib/mavlink_defs.js

用法: python3 scripts/gen_mavlink_defs.py

输出内容:
  - 消息 id / 名称 / crc_extra
  - wire 顺序的字段表: 名称 / 结构码 / 数组长度(0=标量)
数据来源为 pymavlink 生成方言(common + ardupilotmega), 与 ArduPilot 固件一致。
"""
import os
import re
import json

from pymavlink.dialects.v20 import common as common
from pymavlink.dialects.v20 import ardupilotmega as apm

try:
    from pymavlink.dialects.v10 import common as common1
except Exception:
    common1 = None

# 需要的消息 (name -> 方言优先顺序)
WANTED = [
    'HEARTBEAT',
    'PARAM_REQUEST_READ',
    'PARAM_REQUEST_LIST',
    'PARAM_VALUE',
    'PARAM_SET',
    'COMMAND_LONG',
    'COMMAND_ACK',
    'STATUSTEXT',
    'ATTITUDE',
    'GLOBAL_POSITION_INT',
    'VFR_HUD',
    'SYS_STATUS',
    'GPS_RAW_INT',
    'AUTOPILOT_VERSION',
    'ESC_TELEMETRY_1_TO_4',
    'LOG_REQUEST_LIST',
    'LOG_ENTRY',
    'LOG_REQUEST_DATA',
    'LOG_DATA',
    'LOG_ERASE',
    'LOG_REQUEST_END',
    # 传感器体检
    'VIBRATION',
    'EKF_STATUS_REPORT',
    'SCALED_IMU2',
    'SCALED_PRESSURE',
    'RAW_IMU',
]

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib', 'mavlink_defs.js')

# 本机 pymavlink 可能落后于 MAVLink 公共方言(common.xml)。
# 若方言里缺少下列字段, 按公共方言定义补进 v2 字段表, 保证与真实飞控载荷一致。
EXTRA_V2_FIELDS = {
    'SYS_STATUS': [
        ('onboard_control_sensors_present_extended', 'u32'),
        ('onboard_control_sensors_enabled_extended', 'u32'),
        ('onboard_control_sensors_health_extended', 'u32'),
    ],
}


def find_msg(dialects, name):
    for mod in dialects:
        cls = getattr(mod, 'MAVLink_%s_message' % name.lower(), None)
        if cls is not None:
            return cls
    return None


def parse_format(fmt):
    """把 '<I4H16sB' 解析成 [(count, code), ...]"""
    if fmt and fmt[0] in '<>!@=':
        fmt = fmt[1:]
    tokens = []
    i = 0
    while i < len(fmt):
        m = re.match(r'(\d+)?([A-Za-z?])', fmt[i:])
        if not m:
            raise ValueError('bad format %r at %d' % (fmt, i))
        count = int(m.group(1)) if m.group(1) else 1
        code = m.group(2)
        tokens.append((count, code))
        i += m.end()
    return tokens


def build_def(name):
    cls = find_msg((common, apm), name)
    if cls is None:
        # 退而求其次用 v1.0 方言(字段布局相同)
        cls = find_msg((common1,), name) if common1 else None
    if cls is None:
        raise SystemExit('无法在 pymavlink 方言中找到消息 %s' % name)

    fmt = cls.unpacker.format
    tokens = parse_format(fmt)
    fieldnames = list(getattr(cls, 'ordered_fieldnames', []))
    arrlens = list(getattr(cls, 'array_lengths', []))
    if len(tokens) != len(fieldnames):
        raise SystemExit('%s: format 字段数(%d) 与 ordered_fieldnames(%d) 不一致'
                         % (name, len(tokens), len(fieldnames)))

    fields = []
    for (count, code), fname, alen in zip(tokens, fieldnames, arrlens):
        fields.append({
            'name': fname,
            'type': code,
            'len': int(alen or (count if count > 1 else 0)),
        })

    # v1.0 载荷长度(基准字段, 不含 MAVLink2 扩展字段); 无 v1 定义则为 None
    v1_size = None
    v1_fields = []
    cls1 = find_msg((common1,), name) if common1 else None
    if cls1 is not None:
        v1_size = cls1.unpacker.size
        v1_fields = list(getattr(cls1, 'ordered_fieldnames', []))

    # 补齐方言版本缺失的 MAVLink2 扩展字段 (仅当 v2 字段表中确实没有时)
    existing = {f['name'] for f in fields}
    base_size = int(cls.unpacker.size)
    type_size = {'b': 1, 'B': 1, 'h': 2, 'H': 2, 'i': 4, 'I': 4, 'f': 4, 'd': 8,
                 'q': 8, 'Q': 8, '?': 1,
                 'i8': 1, 'u8': 1, 'i16': 2, 'u16': 2, 'i32': 4, 'u32': 4,
                 'f32': 4, 'f64': 8, 'i64': 8, 'u64': 8, 'bool': 1}
    for fname, code in EXTRA_V2_FIELDS.get(name, []):
        if fname not in existing:
            fields.append({'name': fname, 'type': code, 'len': 0})
            existing.add(fname)
            base_size += type_size[code]

    return {
        'id': int(cls.id),
        'name': cls.msgname if hasattr(cls, 'msgname') else name,
        'crcExtra': int(cls.crc_extra),
        'format': fmt,
        'size': base_size,
        'v1Size': v1_size,
        'v1Fields': v1_fields,
        'fields': fields,
    }


def js_type(code):
    mapping = {
        'b': 'i8', 'B': 'u8', 'h': 'i16', 'H': 'u16',
        'i': 'i32', 'I': 'u32', 'f': 'f32', 'd': 'f64',
        's': 'char', '?': 'bool', 'q': 'i64', 'Q': 'u64',
    }
    return mapping.get(code, code)


def main():
    defs = {}
    order = []
    for name in WANTED:
        d = build_def(name)
        d['fields'] = [
            {'name': f['name'], 'type': js_type(f['type']), 'len': f['len']}
            for f in d['fields']
        ]
        defs[name] = d
        order.append(name)

    lines = []
    lines.append('// 本文件由 scripts/gen_mavlink_defs.py 自动生成, 请勿手工编辑。')
    lines.append('// 数据来源: pymavlink common + ardupilotmega 方言')
    lines.append('')
    lines.append('export const MAVLINK_DEFS = {')
    for name in order:
        d = defs[name]
        lines.append('  %s: {' % name)
        lines.append('    id: %d,' % d['id'])
        lines.append('    crcExtra: %d,' % d['crcExtra'])
        lines.append('    v1Size: %s,' % ('null' if d['v1Size'] is None else d['v1Size']))
        lines.append('    v1Fields: [%s],' % ', '.join("'%s'" % x for x in d['v1Fields']))
        lines.append('    size: %d,' % d['size'])
        lines.append('    fields: [')
        for f in d['fields']:
            lines.append("      { name: '%s', type: '%s', len: %d },"
                         % (f['name'], f['type'], f['len']))
        lines.append('    ],')
        lines.append('  },')
    lines.append('};')
    lines.append('')
    lines.append('export const MSG_NAME_BY_ID = {')
    for name in order:
        d = defs[name]
        lines.append('  %d: %s,' % (d['id'], json.dumps(d['name'])))
    lines.append('};')
    lines.append('')
    lines.append('export const MSG_ID = {')
    for name in order:
        lines.append('  %s: %d,' % (name, defs[name]['id']))
    lines.append('};')
    lines.append('')

    with open(OUT, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('\n'.join(lines))
    print('已写出 %s (%d 条消息)' % (os.path.normpath(OUT), len(order)))
    for name in order:
        print('  %-24s id=%-6d crc=%-4d fields=%d'
              % (name, defs[name]['id'], defs[name]['crcExtra'],
                 len(defs[name]['fields'])))


if __name__ == '__main__':
    main()
