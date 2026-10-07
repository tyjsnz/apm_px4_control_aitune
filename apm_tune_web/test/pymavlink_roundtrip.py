#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MAVLink 对拍测试 (pymavlink 侧)
  python3 test/pymavlink_roundtrip.py gen     -> 写 test/out/py_frames.json
  python3 test/pymavlink_roundtrip.py check   -> 解析 test/out/js_frames.json 并比对

JS 侧: node test/mavlink_roundtrip.js gen|check
"""
import json
import os
import sys

from pymavlink.dialects.v10 import common as c1
from pymavlink.dialects.v20 import common as c2
from pymavlink.dialects.v20 import ardupilotmega as a2

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')

CASES = [
    ('HEARTBEAT', dict(custom_mode=0, type=2, autopilot=3, base_mode=81,
                       system_status=4, mavlink_version=3)),
    ('PARAM_REQUEST_READ', dict(param_index=-1, target_system=1,
                                target_component=1, param_id='MOT_THST_HOVER')),
    ('PARAM_REQUEST_LIST', dict(target_system=1, target_component=1)),
    ('PARAM_VALUE', dict(param_value=0.25, param_count=1200, param_index=5,
                         param_id='MOT_THST_HOVER', param_type=9)),
    ('PARAM_SET', dict(param_value=0.3, target_system=1, target_component=1,
                       param_id='MOT_THST_HOVER', param_type=9)),
    ('COMMAND_LONG', dict(param1=1, param2=0, param3=0, param4=0, param5=0,
                          param6=0, param7=0, command=246, target_system=1,
                          target_component=1, confirmation=0)),
    ('COMMAND_ACK', dict(command=246, result=0, progress=0, result_param2=0,
                         target_system=1, target_component=1)),
    ('STATUSTEXT', dict(severity=6, text='PreArm: check', id=0, chunk_seq=0)),
    ('ATTITUDE', dict(time_boot_ms=123456, roll=0.1, pitch=-0.2, yaw=1.5,
                      rollspeed=0.01, pitchspeed=-0.02, yawspeed=0.003)),
    ('GLOBAL_POSITION_INT', dict(time_boot_ms=1000, lat=399000000,
                                 lon=1164000000, alt=50000, relative_alt=3000,
                                 vx=10, vy=-20, vz=5, hdg=9000)),
    ('VFR_HUD', dict(airspeed=5, groundspeed=6, alt=30, climb=1.2, heading=90,
                     throttle=55)),
    ('SYS_STATUS', dict(onboard_control_sensors_present=1,
                        onboard_control_sensors_enabled=1,
                        onboard_control_sensors_health=1, load=250,
                        voltage_battery=24000, current_battery=100,
                        drop_rate_comm=0, errors_comm=0, errors_count1=0,
                        errors_count2=0, errors_count3=0, errors_count4=0,
                        battery_remaining=80,
                        onboard_control_sensors_present_extended=0,
                        onboard_control_sensors_enabled_extended=0,
                        onboard_control_sensors_health_extended=0)),
    ('ESC_TELEMETRY_1_TO_4', dict(voltage=[2400, 2401, 2402, 2403],
                                  current=[100, 101, 102, 103],
                                  totalcurrent=[10, 11, 12, 13],
                                  rpm=[0, 1500, 1600, 1700],
                                  count=[1, 2, 3, 4],
                                  temperature=[30, 31, 32, 33])),
]

SYSID, COMPID = 255, 190


def dialect_for(name, version):
    if name == 'ESC_TELEMETRY_1_TO_4':
        return a2
    return c2 if version == 2 else c1


def msg_class(dialect, name):
    return getattr(dialect, 'MAVLink_%s_message' % name.lower())


def make_mav(dialect, seq):
    mav = dialect.MAVLink(bytearray(), srcSystem=SYSID, srcComponent=COMPID)
    mav.seq = seq
    return mav


def num_eq(a, b):
    if isinstance(a, float) or isinstance(b, float):
        return abs(float(a) - float(b)) <= max(1e-4, abs(float(a)) * 1e-4)
    if isinstance(a, (list, tuple)) or isinstance(b, (list, tuple)):
        return list(a) == list(b) or (
            isinstance(a, (list, tuple)) and isinstance(b, (list, tuple))
            and len(a) == len(b)
            and all(num_eq(x, y) for x, y in zip(a, b)))
    return a == b


def parse_one(dialect, data):
    mav = dialect.MAVLink(bytearray())
    msg = None
    for i in range(len(data)):
        got = mav.parse_char(data[i:i + 1])
        if got is not None:
            msg = got
    return msg


def gen():
    os.makedirs(OUT, exist_ok=True)
    frames = []
    seq = 0
    for name, fields in CASES:
        for version in (1, 2):
            dialect = dialect_for(name, version)
            cls = msg_class(dialect, name)
            if cls.id > 255 and version == 1:
                continue
            args = [fields[f].encode('ascii') if isinstance(fields[f], str) else fields[f]
                    for f in cls.fieldnames]
            mav = make_mav(dialect, seq)
            seq += 1
            frame = bytes(cls(*args).pack(mav))
            frames.append(dict(name=name, version=version, hex=frame.hex(),
                               fields=fields, msgId=cls.id,
                               sysid=SYSID, compid=COMPID))
    dest = os.path.join(OUT, 'py_frames.json')
    with open(dest, 'w', encoding='utf-8') as fh:
        json.dump(frames, fh, indent=1)
    print('pymavlink 编码 %d 帧 -> %s' % (len(frames), dest))


def norm_v1(dialect, name, version, fields):
    """v1.0 只比对基准字段(不含 MAVLink2 扩展字段)"""
    if version != 1:
        return fields
    cls = msg_class(dialect, name)
    base = set(cls.fieldnames)
    return {k: v for k, v in fields.items() if k in base}


def check():
    src = os.path.join(OUT, 'js_frames.json')
    if not os.path.exists(src):
        print('缺少 %s (先跑 node test/mavlink_roundtrip.js gen)' % src)
        sys.exit(1)
    frames = json.load(open(src, encoding='utf-8'))
    errs = []
    n = 0
    for f in frames:
        dialect = dialect_for(f['name'], f['version'])
        msg = parse_one(dialect, bytes.fromhex(f['hex']))
        if msg is None:
            errs.append('%s v%d: pymavlink 无法解析' % (f['name'], f['version']))
            continue
        if msg.get_type() != f['name']:
            errs.append('%s: 类型=%s' % (f['name'], msg.get_type()))
        if getattr(msg, 'get_srcSystem', lambda: None)() != f.get('sysid'):
            errs.append('%s: sysid=%s' % (f['name'], msg.get_srcSystem()))
        if getattr(msg, 'get_srcComponent', lambda: None)() != f.get('compid'):
            errs.append('%s: compid=%s' % (f['name'], msg.get_srcComponent()))
        for k, want in norm_v1(dialect, f['name'], f['version'], f['fields']).items():
            got = getattr(msg, k, None)
            if not num_eq(got, want):
                errs.append('v%d %s.%s: got=%r want=%r'
                            % (f['version'], f['name'], k, got, want))
        n += 1
    if errs:
        print('对拍失败:\n' + '\n'.join(errs))
        sys.exit(1)
    print('pymavlink 解析 JS 帧 %d 条, 全部一致' % n)


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'gen'
    if mode == 'gen':
        gen()
    elif mode == 'check':
        check()
    else:
        print('用法: python3 test/pymavlink_roundtrip.py [gen|check]')
        sys.exit(1)
