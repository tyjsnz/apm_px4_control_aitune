# -*- coding: utf-8 -*-
"""简易 PX4 假飞控 (TCP), 用于 px4_control / px4 API 无硬件冒烟测试.

与 APM 版 fake_fc 的关键差异 (按 PX4 官方行为模拟):
  - 心跳 MAV_AUTOPILOT_PX4, custom_mode = (main_mode<<16)|(sub_mode<<24)
  - 模式切换走 MAV_CMD_DO_SET_MODE(param2=main, param3=sub) / SET_MODE
  - 不发 EKF_STATUS_REPORT (PX4 无此消息), 位置就绪看 GPS+GLOBAL_POSITION
  - 发 EXTENDED_SYS_STATE(landed_state) 供 wait_landed 判定
  - 任务无 home 槽位: count 项即 count 项
  - 参数: PARAM_SET 写入即回显 PARAM_VALUE (MIS_TAKEOFF_ALT 等真实参数名)

用法: python -m px4.tests.fake_fc_px4 [--port 15761]
API 侧连接串: tcp:127.0.0.1:15761
"""
import argparse
import math
import os
import sys
import time

from pymavlink import mavutil

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from px4_control import pack_custom_mode, custom_mode_parts
except ImportError:                      # 作为包内模块导入时
    from px4.px4_control import pack_custom_mode, custom_mode_parts

MAIN_NAMES = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO',
              5: 'ACRO', 6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE'}
AUTO_SUBS = {1: 'READY', 2: 'TAKEOFF', 3: 'LOITER', 4: 'MISSION',
             5: 'RTL', 6: 'LAND', 7: 'RTGS', 8: 'FOLLOW_TARGET'}
MODE_NAME = {v: k for k, v in MAIN_NAMES.items()}
MODE_NAME.update({('AUTO_' + s): (4, i) for i, s in AUTO_SUBS.items()})

HOME = (31.2304, 121.4737, 10.0)
M_PER_DEG = 111319.49
ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR

DEFAULT_PARAMS = {
    'MIS_TAKEOFF_ALT': 10.0,
    'COM_DISARM_PRFLT': 10.0,
    'COM_DISARM_LAND': 2.0,
    'COM_RCL_EXCEPT': 0.0,
    'COM_LOW_BAT_ACT': 0.0,
    'NAV_DLL_ACT': 0.0,
    'COM_OBL_RC_ACT': 5.0,
    'MPC_XY_VEL_MAX': 12.0,
    'MPC_TKO_SPEED': 1.5,
    'MPC_TAKEOFF_ALT': 2.5,
    'MPC_Z_VEL_MAX_UP': 3.0,
    'MPC_Z_VEL_MAX_DN': 1.5,
    'MPC_ACC_RAD': 1.5,
    'BAT_LOW_THR': 15.0,
    'RTL_RETURN_ALT': 60.0,
    'RC_MAP_MODE_SW': 5.0,
}


class FakeFCpx4(object):
    def __init__(self, conn_str):
        self.conn = mavutil.mavlink_connection(conn_str, source_system=1,
                                               source_component=1)
        self.armed = False
        self.main, self.sub = 3, 0          # 默认 POSCTL
        self.lat, self.lon, self.rel = HOME[0], HOME[1], 0.0
        self.vn = self.ve = self.vz = 0.0
        self.heading = 0.0
        self.pos_target = None              # OFFBOARD 位置设定值
        self.vel_target = None              # OFFBOARD 速度设定值
        self.takeoff_alt = None
        self.mission = []
        self.mission_seq = 0
        self.mission_upload = None
        self.params = dict(DEFAULT_PARAMS)
        self.received = {'pos_target': 0, 'vel_target': 0, 'arm': 0,
                         'mode': 0, 'takeoff': 0, 'do_set_servo': 0}
        self.ground_t = None                # 触地时刻 (COM_DISARM_LAND)
        self._t0 = time.time()
        self._next = {'hb': 0, 'gps': 0, 'pos': 0, 'att': 0, 'vfr': 0,
                      'sys': 0, 'mc': 0, 'home': 0, 'ext': 0, 'rc': 0}

    # ---------------- 遥测 ----------------

    def send_telemetry(self, now):
        n = self
        if now >= n._next['hb']:
            n._next['hb'] = now + 1.0
            base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if n.armed:
                base |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            base |= mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            n.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_PX4,
                base, pack_custom_mode(n.main, n.sub), 4)
        if now >= n._next['gps']:
            n._next['gps'] = now + 0.5
            n.conn.mav.gps_raw_int_send(int((now - n._t0) * 1e6), 3,
                                        int(n.lat * 1e7), int(n.lon * 1e7),
                                        int((HOME[2] + n.rel) * 1000),
                                        90, 150, 0, int(n.heading * 100) % 36000,
                                        14)
        if now >= n._next['pos']:
            n._next['pos'] = now + 0.2
            n.conn.mav.global_position_int_send(
                int((now - n._t0) * 1000),
                int(n.lat * 1e7), int(n.lon * 1e7),
                int((HOME[2] + n.rel) * 1000),
                int(n.rel * 1000),
                int(n.vn * 100), int(n.ve * 100), int(-n.vz * 100),
                int(n.heading * 100) % 36000)
        if now >= n._next['ext']:
            n._next['ext'] = now + 0.5
            state = ON_GROUND if n.rel <= 0.05 else IN_AIR
            n.conn.mav.extended_sys_state_send(state, 0)
        if now >= n._next['att']:
            n._next['att'] = now + 0.2
            n.conn.mav.attitude_send(int((now - n._t0) * 1000),
                                     0.02, -0.01,
                                     math.radians(n.heading), 0, 0, 0)
        if now >= n._next['vfr']:
            n._next['vfr'] = now + 0.5
            sp = math.hypot(n.vn, n.ve)
            n.conn.mav.vfr_hud_send(sp, sp, int(n.heading) % 360,
                                    55 if n.armed else 0,
                                    n.rel, 0.0)
        if now >= n._next['sys']:
            n._next['sys'] = now + 1.0
            n.conn.mav.sys_status_send(0x7FFFFFF, 0x7FFFFFF, 0x7FFFFFF,
                                       0, 15200, 1250,
                                       88 if n.armed else 100, 0, 0, 0, 0, 0, 0)
        if now >= n._next['rc']:
            n._next['rc'] = now + 0.5
            n.conn.mav.rc_channels_raw_send(
                int((now - n._t0) * 1000), 0,
                1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 255)
        if now >= n._next['mc']:
            n._next['mc'] = now + 0.5
            n.conn.mav.mission_current_send(n.mission_seq)
        if now >= n._next['home']:
            n._next['home'] = now + 5.0
            n.conn.mav.home_position_send(int(HOME[0] * 1e7),
                                          int(HOME[1] * 1e7),
                                          int(HOME[2] * 1000),
                                          0, 0, 0, [1, 0, 0, 0],
                                          0, 0, 0)

    # ---------------- 消息处理 ----------------

    @property
    def mode_name(self):
        if self.main == 4:
            return 'AUTO_' + AUTO_SUBS.get(self.sub, str(self.sub))
        return MAIN_NAMES.get(self.main, str(self.main))

    def handle(self, msg):
        t = msg.get_type()
        if t == 'HEARTBEAT':
            return
        if t == 'SET_MODE':
            self.set_mode(int(msg.custom_mode))
            return
        if t == 'SET_POSITION_TARGET_GLOBAL_INT':
            if (msg.type_mask & 0b11) == 0 and (msg.lat_int or msg.lon_int):
                self.pos_target = (msg.lat_int / 1e7, msg.lon_int / 1e7,
                                   float(msg.alt or self.rel))
                self.vel_target = None
                self.received['pos_target'] += 1
            return
        if t == 'SET_POSITION_TARGET_LOCAL_NED':
            if (msg.type_mask & 0x38) == 0:   # 速度位(vx/vy/vz)可用
                self.vel_target = (float(msg.vx), float(msg.vy),
                                   float(msg.vz))
                self.pos_target = None
                self.received['vel_target'] += 1
            return
        if t == 'MISSION_COUNT':
            self.mission_upload = {'count': int(msg.count), 'items': {}}
            self.conn.mav.mission_request_int_send(
                msg.get_srcSystem(), msg.get_srcComponent(), 0)
            return
        if t in ('MISSION_ITEM_INT', 'MISSION_ITEM'):
            if self.mission_upload is None:
                return
            self.mission_upload['items'][int(msg.seq)] = msg
            nxt = int(msg.seq) + 1
            if nxt < self.mission_upload['count']:
                self.conn.mav.mission_request_int_send(
                    msg.get_srcSystem(), msg.get_srcComponent(), nxt)
            else:
                self.mission = [self.mission_upload['items'][k]
                                for k in sorted(self.mission_upload['items'])]
                self.mission_seq = 0
                self.mission_upload = None
                self.conn.mav.mission_ack_send(
                    msg.get_srcSystem(), msg.get_srcComponent(),
                    mavutil.mavlink.MAV_MISSION_ACCEPTED)
                self.status(6, 'mission accepted (%d)' % len(self.mission))
            return
        if t == 'MISSION_SET_CURRENT':
            self.mission_seq = int(msg.seq)
            self.conn.mav.mission_current_send(self.mission_seq)
            return
        if t == 'COMMAND_LONG':
            self.command(msg)
            return
        if t == 'PARAM_REQUEST_READ':
            name = self._param_name(msg.param_id)
            val = float(self.params.get(name, -1.0))
            self.conn.mav.param_value_send(
                name.encode('ascii', 'replace'), val,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
            return
        if t == 'PARAM_SET':
            name = self._param_name(msg.param_id)
            self.params[name] = float(msg.param_value)
            self.conn.mav.param_value_send(
                name.encode('ascii', 'replace'), float(msg.param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
            return

    @staticmethod
    def _param_name(param_id):
        name = param_id
        if isinstance(name, (bytes, bytearray)):
            name = name.decode('ascii', 'replace')
        return str(name).split('\x00')[0]

    def command(self, msg):
        cmd = int(msg.command)
        ack = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if float(msg.param1) >= 1.0:
                self.armed = True
                self.received['arm'] += 1
            else:
                self.armed = False
                self.ground_t = None
            self.status(6, 'ARMED' if self.armed else 'DISARMED')
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            alt = float(msg.param7)
            if alt != alt:                       # NaN -> MIS_TAKEOFF_ALT
                alt = float(self.params.get('MIS_TAKEOFF_ALT', 10.0))
            self.takeoff_alt = alt or 10.0
            self.received['takeoff'] += 1
            self.pos_target = None
            self.vel_target = None
            # PX4: NAV_TAKEOFF 被接受即切 Takeoff 子模式
            if self.armed:
                self.set_mode(pack_custom_mode(4, 2))
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            main = int(msg.param2) & 0xFF
            sub = int(msg.param3) & 0xFF
            self.set_mode(pack_custom_mode(main, sub))
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
            self.mission_seq = int(msg.param1)
            self.conn.mav.mission_current_send(self.mission_seq)
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            self.received['do_set_servo'] += 1
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode(pack_custom_mode(4, 5))
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
            self.set_mode(pack_custom_mode(4, 6))
        elif cmd == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
            pass
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
            pass
        elif cmd == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            mid = int(msg.param1)
            if mid == 193:
                pass            # PX4 不发 EKF_STATUS_REPORT
            elif mid == 245:    # EXTENDED_SYS_STATE
                self.conn.mav.extended_sys_state_send(
                    ON_GROUND if self.rel <= 0.05 else IN_AIR, 0)
            elif mid == 36:
                self.conn.mav.servo_output_raw_send(0, 0, 1000, 1000, 1000,
                                                    1000, 0, 0, 0, 0, 0)
        else:
            ack = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        self.conn.mav.command_ack_send(cmd, ack)

    def set_mode(self, custom_mode):
        self.main, self.sub = custom_mode_parts(custom_mode)
        self.received['mode'] += 1
        self.status(6, 'MODE -> %s' % self.mode_name)
        if self.main == 4 and self.sub == 2 and self.armed:
            # 自动 Takeoff: 用 MIS_TAKEOFF_ALT 兜底
            if self.takeoff_alt is None:
                self.takeoff_alt = float(self.params.get('MIS_TAKEOFF_ALT',
                                                         10.0))

    def status(self, sev, text):
        self.conn.mav.statustext_send(sev, text.encode('utf-8', 'replace'))

    # ---------------- 简单飞行动力学 ----------------

    def step(self, dt):
        if not self.armed:
            self.rel = 0.0
            self.vn = self.ve = self.vz = 0.0
            return
        name = self.mode_name

        if self.takeoff_alt is not None and self.rel < self.takeoff_alt:
            goal = (self.lat, self.lon, self.takeoff_alt)
        elif name in ('AUTO_LAND', 'LAND'):
            goal = (self.lat, self.lon, 0.0)
        elif name in ('AUTO_RTL', 'RTL'):
            dh = math.hypot((HOME[0] - self.lat) * M_PER_DEG,
                            (HOME[1] - self.lon) * M_PER_DEG *
                            math.cos(math.radians(self.lat)))
            goal = (HOME[0], HOME[1], 20.0 if dh > 3.0 else 0.0)
        elif name in ('AUTO_MISSION', 'MISSION'):
            goal = self.mission_goal()
        elif name == 'OFFBOARD':
            goal = self.pos_target
            if goal is None and self.vel_target is not None:
                vx, vy, vz = self.vel_target
                self.vn, self.ve, self.vz = vx, vy, vz
                self.lat += vx * dt / M_PER_DEG
                self.lon += vy * dt / (M_PER_DEG *
                                       math.cos(math.radians(self.lat)))
                self.rel = max(0.0, self.rel - vz * dt)
                self._update_heading()
                return
        else:
            goal = None       # POSCTL/LOITER/ALTCTL 等: 保持原地悬停

        if goal is None:
            self.vn *= 0.9
            self.ve *= 0.9
            self.vz = 0.0
            return
        dlat = goal[0] - self.lat
        dlon = goal[1] - self.lon
        north = dlat * M_PER_DEG
        east = dlon * M_PER_DEG * math.cos(math.radians(self.lat))
        dist = math.hypot(north, east)
        if dist > 0.05:
            sp = min(8.0, max(1.0, dist))
            self.vn = north / dist * sp
            self.ve = east / dist * sp
            self.lat += self.vn * dt / M_PER_DEG
            self.lon += self.ve * dt / (M_PER_DEG *
                                        math.cos(math.radians(self.lat)))
            self._update_heading()
        else:
            self.vn = self.ve = 0.0
        dz = goal[2] - self.rel
        if abs(dz) > 0.05:
            self.rel += max(-3.0, min(3.0, dz)) * dt
            self.vz = -max(-3.0, min(3.0, dz))
        else:
            self.rel = goal[2]
            self.vz = 0.0
            self.takeoff_alt = None

        landed = self.rel <= 0.01
        if landed and name in ('AUTO_LAND', 'LAND', 'AUTO_RTL', 'RTL'):
            if self.ground_t is None:
                self.ground_t = time.time()
            # COM_DISARM_LAND: 触地 2s 后自动上锁
            if time.time() - self.ground_t >= \
                    float(self.params.get('COM_DISARM_LAND', 2.0)):
                self.rel = 0.0
                self.armed = False
                self.takeoff_alt = None
                self.pos_target = None
                self.vel_target = None
                self.status(6, 'Landed, disarmed')

    def _update_heading(self):
        if abs(self.vn) > 0.05 or abs(self.ve) > 0.05:
            self.heading = math.degrees(
                math.atan2(self.ve, self.vn)) % 360

    def mission_goal(self):
        if not self.mission:
            return None
        if self.mission_seq >= len(self.mission):
            return None
        it = self.mission[self.mission_seq]
        cmd = int(it.command)
        if cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            goal = (it.x / 1e7, it.y / 1e7, float(it.z))
            d = math.hypot((goal[0] - self.lat) * M_PER_DEG,
                           (goal[1] - self.lon) * M_PER_DEG *
                           math.cos(math.radians(self.lat)))
            if d < 1.5 and abs(goal[2] - self.rel) < 1.0:
                self.mission_seq += 1
                self.conn.mav.mission_current_send(self.mission_seq)
                return self.mission_goal()
            return goal
        if cmd == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode(pack_custom_mode(4, 5))
            return (HOME[0], HOME[1], max(0.0, self.rel))
        if cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
            self.set_mode(pack_custom_mode(4, 6))
            return (self.lat, self.lon, 0.0)
        # DO_SET_SERVO / NAV_DELAY 等直接跳过
        self.mission_seq += 1
        self.conn.mav.mission_current_send(self.mission_seq)
        return self.mission_goal()

    def run(self):
        print('[fake_fc_px4] simulator running')
        last = time.time()
        while True:
            now = time.time()
            dt = min(0.2, now - last)
            last = now
            for _ in range(50):
                msg = self.conn.recv_match(blocking=False)
                if msg is None:
                    break
                try:
                    self.handle(msg)
                except Exception as e:
                    print('[fake_fc_px4] handle error: %r' % e)
            self.step(dt)
            self.send_telemetry(now)
            time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=15761)
    ap.add_argument('--host', default='127.0.0.1')
    args = ap.parse_args()
    conn = 'tcpin:%s:%d' % (args.host, args.port)
    print('[fake_fc_px4] %s' % conn)
    FakeFCpx4(conn).run()


if __name__ == '__main__':
    main()
