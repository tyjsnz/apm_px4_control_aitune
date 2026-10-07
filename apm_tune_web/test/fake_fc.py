#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟 ArduCopter 飞控 (pty 从设备), 用于 server.js 端到端集成测试。

用法: python3 test/fake_fc.py
输出: 第一行打印 'PTY <从设备路径>', 第二行 'READY'
"""
import os
import select
import sys
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_sample_bin as msb  # noqa: E402

from pymavlink.dialects.v20 import ardupilotmega as a  # noqa: E402

SYSID = 1
COMPID = 1

# 初始参数表
PARAMS = {
    'MOT_THST_HOVER': 0.25,
    'MOT_SPIN_ARM': 0.10,
    'MOT_SPIN_MIN': 0.15,
    'MOT_SPIN_MAX': 0.95,
    'WPNAV_SPEED': 1500.0,
    'WPNAV_ACCEL': 250.0,
    'WPNAV_JERK': 1.0,
    'WPNAV_RADIUS': 200.0,
    'PSC_POSXY_P': 1.0,
    'PSC_VELXY_P': 2.0,
    'FENCE_ENABLE': 0.0,
    'RTL_ALT': 1500.0,
    'BATT_LOW_VOLT': 10.5,
    'BATT_CRT_VOLT': 9.0,
    'INS_GYRO_FILTER': 20.0,
    'PSC_ACCZ_P': 0.25,
    'PSC_ACCZ_I': 0.5,
}
PARAM_COUNT = len(PARAMS)

_LOG = lambda *args: print(*args, file=sys.stderr, flush=True)


def build_log(n_records):
    """构造一个 DataFlash .bin: TEST 基础类型 + 真实传感器消息(姿态/IMU/振动/GPS/电池/电机/电调/气压)"""
    import math
    out = bytearray()

    def fmt_msg(ftype, name, fmt, cols):
        length = 3 + msb.struct.calcsize('<' + ''.join(msb.STRUCT[c] for c in fmt))
        return msb.fmt_record(ftype, length, name, fmt, cols)

    fmt = 'QBbhiIfdg'
    cols = 'TimeUS,u8,i8,i16,i32,u32,f32,f64,half'
    length = 3 + msb.struct.calcsize('<' + ''.join(msb.STRUCT[c] for c in fmt))
    out += msb.fmt_record(200, length, 'TEST', fmt, cols)
    for i in range(n_records):
        out += msb.msg_record(200, fmt, [
            1000000 + i * 100000, i % 256, -5, -1234, -100000 + i,
            4000000000, 1.5 + i * 0.1, -2.25, 3.5,
        ])

    # 传感器消息定义
    out += fmt_msg(100, 'ATT', 'Qffffff', 'TimeUS,Roll,Pitch,Yaw,DesRoll,DesPitch,DesYaw')
    out += fmt_msg(101, 'IMU', 'QBffffff', 'TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ')
    out += fmt_msg(102, 'VIBE', 'QfffBBB', 'TimeUS,VibeX,VibeY,VibeZ,Clip0,Clip1,Clip2')
    out += fmt_msg(103, 'GPS', 'QBffH', 'TimeUS,Status,Spd,HDop,NSats')
    out += fmt_msg(104, 'BAT', 'Qfff', 'TimeUS,Volt,Curr,RemPct')
    out += fmt_msg(105, 'RCOU', 'QHHHH', 'TimeUS,C1,C2,C3,C4')
    out += fmt_msg(106, 'ESC', 'QBffff', 'TimeUS,Instance,RPM,Volt,Curr,Temp')
    out += fmt_msg(107, 'BARO', 'Qff', 'TimeUS,Alt,Press')

    for i in range(n_records):
        t = 1000000 + i * 20000
        ph = i * 0.15
        out += msb.msg_record(100, 'Qffffff', [
            t, 10 * math.sin(ph), 5 * math.cos(ph), 90 + 20 * math.sin(ph * 0.5),
            10 * math.sin(ph), 5 * math.cos(ph), 90 + 20 * math.sin(ph * 0.5)])
        out += msb.msg_record(101, 'QBffffff', [
            t, 0, 0.02 * math.sin(ph), 0.02 * math.cos(ph), 0.01 * math.sin(ph * 2),
            0.1 * math.sin(ph), 0.1 * math.cos(ph), 9.8 + 0.3 * math.sin(ph * 3)])
        out += msb.msg_record(102, 'QfffBBB', [
            t, 8 + 4 * math.sin(ph), 7 + 3 * math.cos(ph), 12 + 6 * math.sin(ph * 1.5),
            i % 3, 0, 0])
        out += msb.msg_record(103, 'QBffH', [
            t, 3, 5 + 3 * math.sin(ph * 0.3), 0.8 + 0.2 * math.cos(ph), 14])
        out += msb.msg_record(104, 'Qfff', [
            t, 24.5 - 0.002 * i, 12 + 6 * abs(math.sin(ph)), 95 - i * 0.1])
        base = 1500 + 200 * math.sin(ph)
        out += msb.msg_record(105, 'QHHHH', [t, int(base), int(base + 30), int(base - 20), int(base + 10)])
        for inst in range(4):
            out += msb.msg_record(106, 'QBffff', [
                t, inst, 1500 + 400 * math.sin(ph + inst), 24.4, 3 + inst * 0.2, 35 + inst])
        out += msb.msg_record(107, 'Qff', [t, 10 + 8 * math.sin(ph * 0.2), 1013.0])
    return bytes(out)


class FakeFC:
    def __init__(self, master):
        self.master = master
        self.mav = a.MAVLink(bytearray(), srcSystem=SYSID, srcComponent=COMPID)
        self.rx = a.MAVLink(bytearray())
        self.motor = None          # (motor, pct)
        self.accel_pos = None
        self.last_hb = 0.0
        self.logs = {0: build_log(200), 1: build_log(60)}

    def emit(self, cls, **kw):
        args = [kw[f] for f in cls.fieldnames]
        self.mav.seq = (self.mav.seq + 1) & 0xff
        data = bytes(cls(*args).pack(self.mav))
        os.write(self.master, data)

    def send_param(self, name, value, index):
        self.emit(a.MAVLink_param_value_message,
                  param_id=name.encode('ascii'),
                  param_value=float(value),
                  param_type=9, param_count=PARAM_COUNT, param_index=index)

    def send_heartbeat(self):
        self.emit(a.MAVLink_heartbeat_message,
                  type=2, autopilot=3, base_mode=0, custom_mode=0,
                  system_status=4, mavlink_version=3)

    def send_ack(self, command, result=0):
        self.emit(a.MAVLink_command_ack_message,
                  command=command, result=result, progress=0,
                  result_param2=0, target_system=255, target_component=190)

    def send_statustext(self, text, severity=6):
        self.emit(a.MAVLink_statustext_message,
                  severity=severity, text=text.encode('ascii'), id=0, chunk_seq=0)

    def send_telemetry(self):
        self.emit(a.MAVLink_attitude_message,
                  time_boot_ms=int(time.time() * 1000) & 0xffffffff,
                  roll=0.01, pitch=-0.02, yaw=1.0,
                  rollspeed=0, pitchspeed=0, yawspeed=0)
        self.emit(a.MAVLink_global_position_int_message,
                  time_boot_ms=int(time.time() * 1000) & 0xffffffff,
                  lat=399000000, lon=1164000000, alt=50000, relative_alt=3000,
                  vx=0, vy=0, vz=0, hdg=9000)
        self.emit(a.MAVLink_vfr_hud_message,
                  airspeed=0.0, groundspeed=0.0, alt=30.0, climb=0.0,
                  heading=90, throttle=0)
        self.emit(a.MAVLink_sys_status_message,
                  onboard_control_sensors_present=1,
                  onboard_control_sensors_enabled=1,
                  onboard_control_sensors_health=1,
                  load=250, voltage_battery=24000, current_battery=100,
                  drop_rate_comm=0, errors_comm=0, errors_count1=0,
                  errors_count2=0, errors_count3=0, errors_count4=0,
                  battery_remaining=80,
                  onboard_control_sensors_present_extended=0,
                  onboard_control_sensors_enabled_extended=0,
                  onboard_control_sensors_health_extended=0)

    def send_esc(self):
        if self.motor and self.motor[1] >= 10:
            rpm = [1500, 1500, 1500, 1500]
        else:
            rpm = [0, 0, 0, 0]
        self.emit(a.MAVLink_esc_telemetry_1_to_4_message,
                  voltage=[2400] * 4, current=[100] * 4,
                  totalcurrent=[10] * 4, rpm=rpm,
                  count=[1] * 4, temperature=[30] * 4)

    def handle(self, m):
        t = m.get_type()
        if t == 'HEARTBEAT':
            if getattr(m, 'autopilot', None) == 8:
                self.send_heartbeat()
        elif t == 'PARAM_REQUEST_LIST':
            for i, (k, v) in enumerate(PARAMS.items()):
                self.send_param(k, v, i)
        elif t == 'PARAM_REQUEST_READ':
            name = m.param_id
            name = name.decode() if isinstance(name, (bytes, bytearray)) else str(name)
            name = name.split('\x00')[0]
            if name in PARAMS:
                self.send_param(name, PARAMS[name], list(PARAMS).index(name))
        elif t == 'PARAM_SET':
            name = m.param_id
            name = name.decode() if isinstance(name, (bytes, bytearray)) else str(name)
            name = name.split('\x00')[0]
            PARAMS[name] = m.param_value
            self.send_param(name, m.param_value, list(PARAMS).index(name) if name in PARAMS else 0)
        elif t == 'LOG_REQUEST_LIST':
            start = int(m.start)
            end = int(m.end)
            if end == 0xFFFF:
                end = max(self.logs) if self.logs else 0
            num = len(self.logs)
            last = max(self.logs) if self.logs else 0
            for i in sorted(self.logs):
                if start <= i <= end:
                    self.emit(a.MAVLink_log_entry_message, id=i, num_logs=num,
                              last_log_num=last, time_utc=1700000000 + i * 3600,
                              size=len(self.logs[i]))
        elif t == 'LOG_REQUEST_DATA':
            lid = int(m.id)
            ofs = int(m.ofs)
            count = int(m.count)
            data = self.logs.get(lid, b'')
            end = min(ofs + count, len(data))
            pos = ofs
            while pos < end:
                chunk = data[pos:min(pos + 90, end)]
                arr = list(chunk) + [0] * (90 - len(chunk))
                self.emit(a.MAVLink_log_data_message, id=lid, ofs=pos,
                          count=len(chunk), data=arr)
                pos += len(chunk)
        elif t == 'LOG_ERASE':
            self.logs = {}
            self.send_statustext('Erasing logs')
            self.emit(a.MAVLink_log_entry_message, id=0, num_logs=0,
                      last_log_num=0, time_utc=0, size=0)
        elif t == 'COMMAND_LONG':
            cmd = int(m.command)
            if cmd == 241:  # PREFLIGHT_CALIBRATION
                if float(m.param5) == 1.0:  # 加速度计
                    self.accel_pos = 1
                    self.emit(a.MAVLink_command_long_message,
                              param1=1, param2=0, param3=0, param4=0,
                              param5=0, param6=0, param7=0, command=42429,
                              target_system=255, target_component=190,
                              confirmation=0)
                    self.send_ack(cmd)
                else:
                    self.send_ack(cmd)
                    self.send_statustext('calibration successful')
            elif cmd == 42429:  # ACCELCAL_VEHICLE_POS
                pos = int(m.param1)
                nxt = pos + 1
                if nxt > 6:
                    self.emit(a.MAVLink_command_long_message,
                              param1=16777215, param2=0, param3=0, param4=0,
                              param5=0, param6=0, param7=0, command=42429,
                              target_system=255, target_component=190,
                              confirmation=0)
                else:
                    self.emit(a.MAVLink_command_long_message,
                              param1=nxt, param2=0, param3=0, param4=0,
                              param5=0, param6=0, param7=0, command=42429,
                              target_system=255, target_component=190,
                              confirmation=0)
            elif cmd == 209:  # DO_MOTOR_TEST
                self.motor = (int(m.param1), int(m.param3))
                self.send_ack(cmd)
            elif cmd == 246:  # reboot
                self.send_ack(cmd)
            else:
                self.send_ack(cmd)
        elif t == 'COMMAND_ACK':
            pass


def main():
    master, slave = os.openpty()
    # 关闭回显/线路规则, 两端都设为 raw, 避免二进制帧被破坏
    tty.setraw(master)
    tty.setraw(slave)
    print('PTY', os.ttyname(slave), flush=True)
    fc = FakeFC(master)
    print('READY', flush=True)
    last_tele = 0.0
    while True:
        r, _, _ = select.select([master], [], [], 0.05)
        if r:
            try:
                data = os.read(master, 8192)
            except OSError:
                break
            if not data:
                break
            try:
                msgs = fc.rx.parse_buffer(data) or []
            except Exception:
                msgs = []
            for got in msgs:
                try:
                    fc.handle(got)
                except Exception as e:  # noqa
                    _LOG('handle error:', e)
        now = time.time()
        if now - fc.last_hb >= 1.0:
            fc.last_hb = now
            fc.send_heartbeat()
        fc.send_esc()
        if now - last_tele >= 1.0:
            last_tele = now
            fc.send_telemetry()


if __name__ == '__main__':
    main()
