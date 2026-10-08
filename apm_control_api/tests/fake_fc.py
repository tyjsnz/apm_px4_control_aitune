# -*- coding: utf-8 -*-
"""简易假飞控 (TCP), 用于 apm_control_api 无硬件冒烟测试.

用法: python -m apm_control_api.tests.fake_fc [--port 15760]
API 侧连接串: tcp:127.0.0.1:15760
"""
import argparse
import math
import time

from pymavlink import mavutil

COPTER_MODE = {
    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
    'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'LAND': 9,
}
MODE_NAME = {v: k for k, v in COPTER_MODE.items()}

EKF_FLAGS_READY = 0x78
HOME = (31.2304, 121.4737, 10.0)
M_PER_DEG = 111319.49


class FakeFC(object):
    def __init__(self, conn_str):
        self.conn = mavutil.mavlink_connection(conn_str, source_system=1,
                                               source_component=1)
        self.armed = False
        self.mode = COPTER_MODE['LOITER']
        self.lat, self.lon, self.rel = HOME[0], HOME[1], 0.0
        self.vn = self.ve = 0.0
        self.heading = 0.0
        self.target = None
        self.takeoff_alt = None
        self.mission = []
        self.mission_seq = 0
        self.mission_upload = None
        self.rc_override = [0] * 8
        self.received = {'pos_target': 0, 'arm': 0, 'mode': 0}
        self.last_pos_target = None
        self.ground = True
        self._t0 = time.time()
        self._next = {'hb': 0, 'gps': 0, 'pos': 0, 'att': 0, 'vfr': 0,
                      'sys': 0, 'rc': 0, 'ekf': 0, 'mc': 0, 'home': 0}

    # ---------------- 发送遥测 ----------------

    def send_telemetry(self, now):
        n = self
        if now >= n._next['hb']:
            n._next['hb'] = now + 1.0
            n.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                 | (mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    if n.armed else 0)),
                n.mode, 4)
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
                int(n.vn * 100), int(n.ve * 100), 0,
                int(n.heading * 100) % 36000)
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
            ch3 = n.rc_override[2] or 1000
            n.conn.mav.rc_channels_raw_send(
                int((now - n._t0) * 1000), 0,
                1500, 1500, ch3, 1500, 1500, 1500, 1500, 1500, 255)
        if now >= n._next['ekf']:
            n._next['ekf'] = now + 1.0
            n.conn.mav.ekf_status_report_send(EKF_FLAGS_READY, 0.01,
                                              0.05, 0.02, 0.03, 0.01)
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

    # ---------------- 命令处理 ----------------

    def handle(self, msg):
        t = msg.get_type()
        if t == 'HEARTBEAT':
            return
        if t == 'SET_MODE':
            self.set_mode(int(msg.custom_mode))
            return
        if t == 'SET_POSITION_TARGET_GLOBAL_INT':
            if (msg.type_mask & 0b11) == 0 and (msg.lat_int or msg.lon_int):
                self.target = (msg.lat_int / 1e7, msg.lon_int / 1e7,
                               float(msg.alt or self.rel))
                self.received['pos_target'] += 1
                self.last_pos_target = {
                    'lat': msg.lat_int / 1e7, 'lon': msg.lon_int / 1e7,
                    'alt': float(msg.alt), 'mask': int(msg.type_mask),
                    'vx': float(msg.vx), 'vy': float(msg.vy),
                    'vz': float(msg.vz), 'frame': int(msg.coordinate_frame),
                    'at': time.time()}
            return
        if t == 'RC_CHANNELS_OVERRIDE':
            self.rc_override = [int(getattr(msg, 'chan%d_raw' % (i + 1)) or 0)
                                for i in range(8)]
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
                items = [self.mission_upload['items'][k]
                         for k in sorted(self.mission_upload['items'])]
                self.mission = items[1:] if len(items) > 1 else items
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
            name = msg.param_id
            if isinstance(name, (bytes, bytearray)):
                name = name.decode('ascii', 'replace')
            name = str(name).split('\x00')[0]
            self.conn.mav.param_value_send(
                name.encode('ascii', 'replace'), -1.0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
            return

    def command(self, msg):
        cmd = int(msg.command)
        ack = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if float(msg.param1) >= 1.0:
                self.armed = True
                self.received['arm'] += 1
            else:
                self.armed = False
            self.status(6, 'ARMED' if self.armed else 'DISARMED')
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            self.takeoff_alt = float(msg.param7) or 10.0
            self.target = None
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            self.set_mode(int(msg.param2))
            ack = mavutil.mavlink.MAV_RESULT_ACCEPTED
        elif cmd == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
            pass
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
            self.mission_seq = 0
            self.conn.mav.mission_current_send(0)
        elif cmd == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            mid = int(msg.param1)
            if mid == 193:
                self.conn.mav.ekf_status_report_send(EKF_FLAGS_READY,
                                                     0.01, 0.05, 0.02, 0.03,
                                                     0.01)
            elif mid == 36:
                self.conn.mav.servo_output_raw_send(0, 0, 1000, 1000, 1000,
                                                    1000, 0, 0, 0, 0, 0)
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            pass
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode(COPTER_MODE['RTL'])
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
            self.set_mode(COPTER_MODE['LAND'])
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
            pass
        else:
            ack = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        self.conn.mav.command_ack_send(cmd, ack)

    def set_mode(self, mode):
        self.mode = int(mode)
        self.received['mode'] += 1
        self.status(6, 'MODE -> %s' % MODE_NAME.get(self.mode, self.mode))

    def status(self, sev, text):
        self.conn.mav.statustext_send(sev, text.encode('utf-8', 'replace'))

    # ---------------- 简单飞行动力学 ----------------

    def step(self, dt):
        if not self.armed:
            self.rel = 0.0
            self.vn = self.ve = 0.0
            return
        if (self.takeoff_alt is not None
                and self.rel >= self.takeoff_alt - 0.05):
            self.takeoff_alt = None
        name = MODE_NAME.get(self.mode, '')
        goal = None
        if self.takeoff_alt is not None and self.rel < self.takeoff_alt:
            goal = (self.lat, self.lon, self.takeoff_alt)
        elif name == 'LAND':
            goal = (self.lat, self.lon, 0.0)
        elif name == 'RTL':
            dh = math.hypot((HOME[0] - self.lat) * M_PER_DEG,
                            (HOME[1] - self.lon) * M_PER_DEG *
                            math.cos(math.radians(self.lat)))
            goal = (HOME[0], HOME[1], 20.0 if dh > 3.0 else 0.0)
        elif name == 'ALT_HOLD':
            self.vn *= 0.8
            self.ve *= 0.8
            if self.rc_override[2]:
                self.vz = (float(self.rc_override[2]) - 1500.0) / 400.0
                self.rel = max(0.0, self.rel + self.vz * dt)
            else:
                self.vz = 0.0
            return
        elif name == 'AUTO':
            goal = self.mission_goal()
        elif self.target is not None:
            goal = self.target
        if goal is None:
            self.vn *= 0.9
            self.ve *= 0.9
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
            self.heading = math.degrees(math.atan2(self.ve, self.vn)) % 360
        else:
            self.vn = self.ve = 0.0
        dz = goal[2] - self.rel
        if abs(dz) > 0.05:
            self.rel += max(-3.0, min(3.0, dz)) * dt
        else:
            self.rel = goal[2]
        if self.rel <= 0.01 and name == 'LAND':
            self.rel = 0.0
            self.armed = False
            self.takeoff_alt = None
            self.target = None
        if name == 'RTL' and dist < 1.0 and self.rel <= 0.05:
            self.rel = 0.0
            self.armed = False

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
            self.set_mode(COPTER_MODE['RTL'])
            return (HOME[0], HOME[1], max(0.0, self.rel))
        if cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
            self.set_mode(COPTER_MODE['LAND'])
            return (self.lat, self.lon, 0.0)
        if cmd == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            self.mission_seq += 1
            self.conn.mav.mission_current_send(self.mission_seq)
            return self.mission_goal()
        self.mission_seq += 1
        return self.mission_goal()

    def run(self):
        print('[fake_fc] autopilot simulator running')
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
                    print('[fake_fc] handle error: %r' % e)
            self.step(dt)
            self.send_telemetry(now)
            time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=15760)
    ap.add_argument('--host', default='127.0.0.1')
    args = ap.parse_args()
    conn = 'tcpin:%s:%d' % (args.host, args.port)
    print('[fake_fc] %s' % conn)
    FakeFC(conn).run()


if __name__ == '__main__':
    main()
