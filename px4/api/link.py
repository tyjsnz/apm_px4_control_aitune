# -*- coding: utf-8 -*-
"""单个地面站(数传)链路: 连接、读线程、遥测快照、指令代理.

设计与 px4_control_ui.py 一致:
  - 唯一读线程 recv 所有 MAVLink 消息 -> 更新共享遥测 + 灌入消息队列
  - MasterProxy.recv_match 从消息队列取, 控制函数(px4_control/px4_control_test)
    与读线程互不抢消息; motors_armed() 直接读共享遥测, 不阻塞等心跳
  - MavSendProxy 统计控制指令频率与 RC 覆盖状态
"""
import math
import os
import queue
import sys
import threading
import time

from pymavlink import mavutil

_PX4 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PX4 not in sys.path:
    sys.path.insert(0, _PX4)

import px4_control as apc
import px4_control_test as tct

from . import config

try:
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

FIX_NAMES = {0: '无定位', 1: '无GPS', 2: '2D', 3: '3D',
             4: 'DGPS', 5: 'RTK浮', 6: 'RTK固'}
SYS_NAMES = {0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING', 3: 'STANDBY',
             4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY', 7: 'POWEROFF',
             8: 'TERM'}

STREAM_IDS = {
    'ALL': 0, 'RAW_SENSORS': 1, 'EXTENDED_STATUS': 2, 'RC_CHANNELS': 3,
    'RAW_CONTROLLER': 4, 'POSITION': 5, 'EXTRA1': 6, 'EXTRA2': 7,
    'EXTRA3': 8,
}


def haversine(lat1, lon1, lat2, lon2):
    """两点球面距离(米)"""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2 +
         math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def get_available_ports():
    """可用串口列表 [(device, description)]"""
    if not HAVE_SERIAL:
        return []
    out = []
    for p in list_ports.comports():
        desc = '%s - %s' % (p.device, p.description)
        if p.hwid:
            desc += ' (%s)' % p.hwid[:30]
        out.append((p.device, desc))
    return out


def is_serial_conn(conn):
    low = str(conn).lower()
    return not any(low.startswith(p) for p in (
        'tcp:', 'tcpin:', 'tcpout:', 'tcps:', 'udp:', 'udpin:', 'udpout:',
        'udpcast:', 'mcast:', 'file:', 'serial:'))


class MavSendProxy:
    """包一层 *.send, 记录控制指令名称/频率/RC覆盖状态"""

    def __init__(self, mav, stats):
        object.__setattr__(self, '_mav', mav)
        object.__setattr__(self, '_stats', stats)

    def __getattr__(self, name):
        attr = getattr(object.__getattribute__(self, '_mav'), name)
        if not (name.endswith('_send') and callable(attr)):
            return attr
        stats = object.__getattribute__(self, '_stats')

        def wrapper(*args, **kwargs):
            with stats['lock']:
                stats['counts'][name] = stats['counts'].get(name, 0) + 1
                if name != 'heartbeat_send':
                    stats['last'] = name
                    stats['last_t'] = time.time()
                if name == 'rc_channels_override_send' and len(args) >= 10:
                    chans = tuple(int(c) for c in args[2:10]
                                  if isinstance(c, (int, float)))
                    zero = all(c in (0, 65535) for c in chans)
                    neutral = tuple(int(c) for c in args[2:6]) in (
                        tct.RC_NEUTRAL_GROUND, tct.RC_NEUTRAL_AIR)
                    stats['rc_override'] = bool(chans) and not zero \
                        and not neutral
            return attr(*args, **kwargs)

        return wrapper


class MasterProxy:
    """recv_match 改从消息队列读, 其余属性透传真实 master"""

    def __init__(self, raw, msg_q, link):
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(self, '_q', msg_q)
        object.__setattr__(self, '_link', link)

    def motors_armed(self):
        link = object.__getattribute__(self, '_link')
        with link.tel_lock:
            return bool(link.tel.get('armed'))

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, '_raw'), name)

    def __setattr__(self, name, value):
        setattr(object.__getattribute__(self, '_raw'), name, value)

    @property
    def mav(self):
        return object.__getattribute__(self, '_raw').mav

    def recv_match(self, condition=None, blocking=False, timeout=None,
                   type=None, *args, **kwargs):
        q = object.__getattribute__(self, '_q')
        types = None
        if type:
            types = (type,) if isinstance(type, str) else tuple(type)

        def ok(m):
            if m is None:
                return False
            if types and m.get_type() not in types:
                return False
            if condition and not condition(m):
                return False
            return True

        if not blocking:
            try:
                m = q.get_nowait()
            except queue.Empty:
                return None
            return m if ok(m) else self.recv_match(
                condition=condition, blocking=False, type=type)

        if timeout is None:
            while True:
                m = q.get()
                if ok(m):
                    return m
        deadline = time.time() + float(timeout)
        while True:
            remain = deadline - time.time()
            if remain <= 0:
                return None
            try:
                m = q.get(timeout=min(remain, 0.25))
            except queue.Empty:
                continue
            if ok(m):
                return m


class Link(object):
    """一路数传地面站 + 其配对飞控"""

    def __init__(self, link_id, port, baud=None, name=None):
        self.id = link_id
        self.port = port
        self.baud = int(baud or config.DEFAULT_BAUD)
        self.name = name or port
        self.created_at = time.time()

        self.raw = None
        self.proxy = None
        self.reader_thread = None
        self.reader_running = False
        self.connected = False
        self.last_error = None

        self.msg_q = queue.Queue(maxsize=2000)
        self.tel_lock = threading.RLock()
        self.tel = {
            'hb_t': 0.0, 'mode': '?', 'armed': None, 'sys_status': None,
            'fix': None, 'sats': None, 'hdop': None,
            'lat': None, 'lon': None, 'rel_alt': None, 'abs_alt': None,
            'vx': None, 'vy': None, 'vz': None, 'heading': None,
            'gs': None, 'as': None, 'thr': None, 'climb': None,
            'roll': None, 'pitch': None, 'yaw': None,
            'volt': None, 'curt': None, 'remain': None, 'load': None,
            'rc': [None] * 8, 'rssi': None,
            'pos_flags': None, 'status_text': '--', 'msgs': 0,
            'pos_flags_t': 0.0, 'gps_t': 0.0, 'pos_t': 0.0, 'link_ok': False,
            'vib': None, 'mission_seq': None,
        }
        self.home = None
        self.home_alt_amsl = None

        self.cmd_stats = {'lock': threading.Lock(), 'counts': {},
                          'last': '--', 'last_t': 0.0, 'rc_override': False}
        self._acc_counts = {}
        self._rate_t = time.time()
        self.msg_lock = threading.Lock()
        self.msg_acc = {}
        self.msg_rates = {}
        self._msg_rate_t = time.time()

        self.task_lock = threading.Lock()
        self.tasks = None
        self.guidance = None

    # ---------------- 连接 ----------------

    def connect(self, timeout=None):
        timeout = float(timeout or config.CONNECT_TIMEOUT)
        conn = self.port
        if is_serial_conn(conn):
            try:
                import serial
                ser = serial.Serial(conn, self.baud, timeout=0.5,
                                    write_timeout=0.5)
                ser.close()
            except Exception as e:
                self.last_error = '串口打开失败: %s' % e
                raise RuntimeError(self.last_error)

        raw = mavutil.mavlink_connection(conn, baud=self.baud)
        ok = False
        try:
            hb = None
            t0 = time.time()
            while time.time() - t0 < timeout:
                hb = raw.recv_match(type='HEARTBEAT', blocking=False)
                if hb is not None:
                    break
                time.sleep(0.02)
            if hb is None:
                self.last_error = '未收到飞控心跳 (%s)' % conn
                raise RuntimeError(self.last_error)
            if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
                ap = mavutil.mavlink.enums['MAV_AUTOPILOT'].get(hb.autopilot)
                ap_name = ap.name if ap is not None else str(hb.autopilot)
                self.last_error = '对端不是 PX4 固件 (%s)' % ap_name
                raise RuntimeError(self.last_error)
            raw.target_system = hb.get_srcSystem()
            raw.target_component = hb.get_srcComponent()
            try:
                raw.vehicle_type = 'copter'
            except AttributeError:
                pass
            self.raw = raw
            self.proxy = MasterProxy(raw, self.msg_q, self)
            raw.mav = MavSendProxy(raw.mav, self.cmd_stats)
            self._request_streams()
            self.reader_running = True
            th = threading.Thread(target=self._reader_loop,
                                  name='reader-%s' % self.id, daemon=True)
            self.reader_thread = th
            th.start()
            self.connected = True
            self.last_error = None
            ok = True
            return self.info()
        finally:
            if not ok:
                try:
                    raw.close()
                except Exception:
                    pass
                self.raw = None
                self.proxy = None
                self.reader_running = False

    def _request_streams(self):
        try:
            for name, rate in config.TELEMETRY_STREAMS:
                sid = STREAM_IDS.get(name)
                if sid is None:
                    continue
                self.raw.mav.request_data_stream_send(
                    self.raw.target_system, self.raw.target_component,
                    sid, rate, 1)
            self.raw.mav.command_long_send(
                self.raw.target_system, self.raw.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                0, 0, 0, 0, 0, 0)
        except Exception:
            pass

    def close(self):
        if self.guidance is not None:
            self.guidance.stop()
        if self.tasks is not None:
            self.tasks.cancel()
        self.stop_offboard_stream()
        self.reader_running = False
        th = self.reader_thread
        if th is not None and th.is_alive() \
                and th is not threading.current_thread():
            th.join(timeout=1.5)
        raw = self.raw
        self.raw = None
        self.proxy = None
        self.connected = False
        if raw is not None:
            try:
                raw.close()
            except Exception:
                pass
        with self.tel_lock:
            self.tel['link_ok'] = False
            self.tel['armed'] = None
            self.tel['mode'] = '?'
        self._drain_queue()

    def stop_offboard_stream(self):
        """停止本链路的 Offboard 设定值流 (退出/急停/引导接管时调用)"""
        proxy = self.proxy
        if proxy is None:
            return
        try:
            apc.stop_offboard_stream(proxy)
        except Exception:
            pass

    def _drain_queue(self):
        while True:
            try:
                self.msg_q.get_nowait()
            except queue.Empty:
                break

    # ---------------- 读线程 ----------------

    def _reader_loop(self):
        me = threading.current_thread()
        last_hb_send = 0.0
        rate_count = 0
        rate_t = time.time()
        while self.reader_running and self.raw is not None \
                and self.reader_thread is me:
            raw = self.raw
            processed = 0
            for _ in range(300):
                try:
                    msg = raw.recv_match(blocking=False)
                except Exception:
                    time.sleep(0.5)
                    break
                if msg is None:
                    break
                processed += 1
                rate_count += 1
                now = time.time()
                mtype = msg.get_type()
                if mtype != 'BAD_DATA':
                    with self.msg_lock:
                        self.msg_acc[mtype] = self.msg_acc.get(mtype, 0) + 1
                        if now - self._msg_rate_t >= 1.0:
                            dt = now - self._msg_rate_t
                            self.msg_rates = {k: round(v / dt, 2)
                                              for k, v in self.msg_acc.items()}
                            self.msg_acc.clear()
                            self._msg_rate_t = now
                if now - rate_t >= 1.0:
                    with self.tel_lock:
                        self.tel['msgs'] = rate_count
                        self.tel['link_ok'] = True
                    rate_count = 0
                    rate_t = now
                self._update_tel(msg)
                if now - last_hb_send >= 1.0:
                    last_hb_send = now
                    try:
                        raw.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    except Exception:
                        pass
                try:
                    self.msg_q.put_nowait(msg)
                except queue.Full:
                    try:
                        self.msg_q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self.msg_q.put_nowait(msg)
                    except queue.Full:
                        pass
            if processed == 0:
                time.sleep(0.004)

    def _update_tel(self, msg):
        mtype = msg.get_type()
        with self.tel_lock:
            t = self.tel
            if mtype == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    return
                t['hb_t'] = time.time()
                t['mode'] = apc.custom_mode_name(msg.custom_mode)
                t['armed'] = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                t['sys_status'] = msg.system_status
            elif mtype == 'GPS_RAW_INT':
                t['fix'] = msg.fix_type
                t['sats'] = msg.satellites_visible
                t['hdop'] = None if msg.eph == 65535 else msg.eph * 0.01
                t['gps_t'] = time.time()
            elif mtype == 'GLOBAL_POSITION_INT':
                t['lat'] = msg.lat / 1e7
                t['lon'] = msg.lon / 1e7
                t['rel_alt'] = msg.relative_alt / 1000.0
                t['abs_alt'] = msg.alt / 1000.0
                t['vx'] = msg.vx / 100.0
                t['vy'] = msg.vy / 100.0
                t['vz'] = msg.vz / 100.0
                t['heading'] = None if msg.hdg == 65535 else msg.hdg / 100.0
                t['pos_t'] = time.time()
                flags = int(t.get('pos_flags') or 0)
                if msg.lat and msg.lon:
                    flags |= tct.POS_FLAG_GPS_GLOBAL
                else:
                    flags &= ~tct.POS_FLAG_GPS_GLOBAL
                t['pos_flags'] = flags
                t['pos_flags_t'] = time.time()
                if self.home_alt_amsl is None:
                    self.home_alt_amsl = t['abs_alt'] - t['rel_alt']
            elif mtype == 'LOCAL_POSITION_NED':
                flags = int(t.get('pos_flags') or 0) | tct.POS_FLAG_LOCAL
                t['pos_flags'] = flags
                t['pos_flags_t'] = time.time()
            elif mtype == 'VFR_HUD':
                t['gs'] = msg.groundspeed
                t['as'] = msg.airspeed
                t['thr'] = msg.throttle
                t['climb'] = msg.climb
            elif mtype == 'ATTITUDE':
                t['roll'] = math.degrees(msg.roll)
                t['pitch'] = math.degrees(msg.pitch)
                t['yaw'] = math.degrees(msg.yaw) % 360.0
            elif mtype == 'SYS_STATUS':
                t['volt'] = msg.voltage_battery / 1000.0
                t['curt'] = (msg.current_battery / 100.0
                             if msg.current_battery >= 0 else None)
                t['remain'] = (msg.battery_remaining
                               if msg.battery_remaining >= 0 else None)
                t['load'] = msg.load / 10.0
            elif mtype == 'BATTERY_STATUS':
                v = [x for x in msg.voltages if x != 65535]
                if v:
                    t['volt'] = sum(v) / 1000.0
                br = getattr(msg, 'battery_remaining', -1)
                if br is not None and br >= 0:
                    t['remain'] = br
                cur = getattr(msg, 'current_battery', -1)
                if cur is not None and cur >= 0:
                    t['curt'] = cur / 100.0
            elif mtype in ('RC_CHANNELS', 'RC_CHANNELS_RAW'):
                for i in range(8):
                    try:
                        t['rc'][i] = int(getattr(msg, 'chan%d_raw' % (i + 1)))
                    except Exception:
                        pass
                rssi = getattr(msg, 'rssi', None)
                t['rssi'] = None if rssi == 255 else rssi
            elif mtype == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, (bytes, bytearray)):
                    text = text.decode('utf-8', 'replace')
                t['status_text'] = str(text)
            elif mtype == 'HOME_POSITION':
                if msg.latitude and msg.longitude:
                    self.home = {'lat': msg.latitude / 1e7,
                                 'lon': msg.longitude / 1e7}
                    if msg.altitude:
                        self.home_alt_amsl = msg.altitude / 1000.0
            elif mtype == 'VIBRATION':
                t['vib'] = (msg.vibration_x, msg.vibration_y, msg.vibration_z)
            elif mtype == 'MISSION_CURRENT':
                t['mission_seq'] = int(msg.seq)

    # ---------------- 状态 ----------------

    def tel_copy(self):
        with self.tel_lock:
            t = dict(self.tel)
            t['rc'] = list(t['rc'])
            if t.get('vib'):
                t['vib'] = tuple(t['vib'])
        return t

    def position_ready(self):
        t = self.tel_copy()
        return tct.position_ready_state(t.get('fix'), t.get('sats'),
                                        t.get('pos_flags'))

    def fresh_position(self, timeout=3.0):
        """从共享遥测取 timeout 秒内的位置 (lat, lon, rel_alt), 过旧返回 None"""
        with self.tel_lock:
            lat, lon = self.tel.get('lat'), self.tel.get('lon')
            rel = self.tel.get('rel_alt')
            pos_t = self.tel.get('pos_t', 0.0)
        if lat is None or lon is None:
            return None
        if pos_t and time.time() - pos_t > timeout:
            return None
        return lat, lon, rel

    def origin_alt_amsl(self):
        if self.home_alt_amsl is not None:
            return self.home_alt_amsl
        with self.tel_lock:
            a, r = self.tel.get('abs_alt'), self.tel.get('rel_alt')
        if a is None or r is None:
            return None
        return a - r

    def info(self):
        t = self.tel_copy()
        return {
            'id': self.id,
            'name': self.name,
            'port': self.port,
            'baud': self.baud,
            'connected': self.connected,
            'link_ok': bool(t.get('link_ok')),
            'sysid': getattr(self.raw, 'target_system', None) if self.raw else None,
            'compid': getattr(self.raw, 'target_component', None) if self.raw else None,
            'mode': t.get('mode'),
            'armed': t.get('armed'),
            'heartbeat_age': round(time.time() - t['hb_t'], 1) if t.get('hb_t') else None,
            'task': self.tasks.status() if self.tasks else None,
            'guidance': self.guidance.status() if self.guidance else None,
            'created_at': self.created_at,
            'last_error': self.last_error,
        }

    def _fmt(self, v, unit='', d=1):
        if v is None:
            return '--'
        return ('%.*f%s' % (d, v, unit)) if unit else ('%.*f' % (d, v))

    def telemetry(self):
        """对应 px4_control_ui 左栏: 飞行状态/GPS/速度姿态/电池系统/控制信号RC"""
        t = self.tel_copy()
        now = time.time()
        ready, why = self.position_ready()

        with self.cmd_stats['lock']:
            counts = dict(self.cmd_stats['counts'])
            self.cmd_stats['counts'] = {}
            last_cmd = self.cmd_stats['last']
            rc_ov = self.cmd_stats['rc_override']
        with self.msg_lock:
            rates = dict(self.msg_rates)
            dt = max(1e-3, now - self._rate_t)
            cmd_rates = {k: round(c / max(dt, 1e-3), 1)
                         for k, c in counts.items()}
            self._rate_t = now

        dist_home = None
        if self.home and t.get('lat') is not None:
            dist_home = haversine(self.home['lat'], self.home['lon'],
                                  t['lat'], t['lon'])
        fix_txt = FIX_NAMES.get(t['fix'], str(t['fix']))
        if isinstance(t['fix'], int) and t['fix'] < 3 and t['armed'] is True:
            fix_txt += '·惯导继续'
        if t['armed'] is True:
            armed_txt = '已解锁 ARMED'
        elif t['armed'] is False:
            armed_txt = '未解锁 DISARMED'
        else:
            armed_txt = '--'
        hb_txt = ('%.1fs 前' % (now - t['hb_t'])) if t['hb_t'] else '--'
        if ready:
            posrdy_txt, posrdy_state = '✅ 就绪 (可POSCTL解锁)', 'ready'
        elif t.get('pos_flags') is None and t.get('fix') is None:
            posrdy_txt, posrdy_state = '⏳ 等待飞控数据', 'waiting'
        else:
            posrdy_txt, posrdy_state = '⏳ ' + why, 'not_ready'
        if t.get('pos_flags') is not None:
            flags_txt = '0x%02X' % int(t['pos_flags'])
        else:
            flags_txt = '--'
        rc = t['rc']
        rc_txt = ' '.join('--' if x is None else str(x) for x in rc[:4])

        labels = {
            'mode': t['mode'] or '--',
            'armed': armed_txt,
            'sys': SYS_NAMES.get(t['sys_status'], str(t['sys_status'])),
            'hb': hb_txt,
            'gps': '%s  卫星 %s  HDOP %s' % (
                fix_txt, self._fmt(t['sats'], '', 0), self._fmt(t['hdop'], '', 1)),
            'pos': ('%.6f, %.6f' % (t['lat'], t['lon']))
                   if t['lat'] is not None else '--',
            'alt': self._fmt(t['rel_alt'], ' m', 2),
            'absalt': self._fmt(t['abs_alt'], ' m', 2),
            'home': ('%.6f, %.6f' % (self.home['lat'], self.home['lon']))
                    if self.home else '--',
            'dist': ('%.1f m' % dist_home) if dist_home is not None else '--',
            'vel': 'N %s  E %s  D %s' % (
                self._fmt(t['vx'], '', 2), self._fmt(t['vy'], '', 2),
                self._fmt(t['vz'], '', 2)),
            'gs': '地 %s  空 %s  油门 %s' % (
                self._fmt(t['gs'], '', 1), self._fmt(t['as'], '', 1),
                self._fmt(t['thr'], '%', 0)),
            'hdg': '航向 %s  爬升 %s' % (
                self._fmt(t['heading'], '°', 0), self._fmt(t['climb'], '', 1)),
            'rpy': '%s / %s / %s' % (
                self._fmt(t['roll'], '', 1), self._fmt(t['pitch'], '', 1),
                self._fmt(t['yaw'], '', 1)),
            'bat': 'V %s  A %s  %s%%' % (
                self._fmt(t['volt'], '', 2), self._fmt(t['curt'], '', 1),
                self._fmt(t['remain'], '', 0)),
            'load': self._fmt(t['load'], '%', 1),
            'ekf': '无EKF报文(PX4)',
            'posflags': flags_txt,
            'posrdy': posrdy_txt,
            'st': (t['status_text'] or '--')[:28],
            'rcov': '杆值覆盖中' if rc_ov else '未覆盖',
            'rc': rc_txt,
            'rssi': self._fmt(t['rssi'], '', 0),
            'cmd': last_cmd,
            'link': '%d' % t['msgs'],
        }

        return {
            'link_id': self.id,
            'connected': self.connected,
            'link_ok': bool(t['link_ok']),
            'updated_at': now,
            'flight_status': {
                'mode': t['mode'],
                'armed': t['armed'],
                'armed_text': armed_txt,
                'system_status': t['sys_status'],
                'system_status_name': SYS_NAMES.get(t['sys_status'],
                                                    str(t['sys_status'])),
                'heartbeat_age_s': round(now - t['hb_t'], 2) if t['hb_t'] else None,
                'last_status_text': t['status_text'],
            },
            'gps': {
                'fix': t['fix'],
                'fix_name': FIX_NAMES.get(t['fix'], str(t['fix'])),
                'satellites': t['sats'],
                'hdop': t['hdop'],
                'lat': t['lat'],
                'lon': t['lon'],
                'rel_alt': t['rel_alt'],
                'abs_alt': t['abs_alt'],
                'home': self.home,
                'home_alt_amsl': self.home_alt_amsl,
                'dist_home': round(dist_home, 1) if dist_home is not None else None,
                'heading': t['heading'],
                'age_s': round(now - t['gps_t'], 2) if t['gps_t'] else None,
            },
            'velocity_attitude': {
                'vx_north': t['vx'], 'vy_east': t['vy'], 'vz_down': t['vz'],
                'groundspeed': t['gs'], 'airspeed': t['as'],
                'throttle': t['thr'], 'climb': t['climb'],
                'roll': t['roll'], 'pitch': t['pitch'], 'yaw': t['yaw'],
            },
            'battery_system': {
                'voltage': t['volt'], 'current': t['curt'],
                'remaining': t['remain'], 'load': t['load'],
                'pos_flags': t['pos_flags'],
                'position_estimate': {'ready': ready, 'why': why,
                                      'state': posrdy_state},
                'vibration': list(t['vib']) if t['vib'] else None,
            },
            'control_signals': {
                'rc_override': rc_ov,
                'rc_channels': rc,
                'rc_rssi': t['rssi'],
                'last_command': last_cmd,
                'command_rates_hz': cmd_rates,
                'msg_per_sec': t['msgs'],
                'message_rates_hz': rates,
                'mission_seq': t['mission_seq'],
            },
            'labels': labels,
            'task': self.tasks.status() if self.tasks else None,
            'guidance': self.guidance.status() if self.guidance else None,
        }

    # ---------------- 立即动作(急停类, 直发不排队) ----------------

    def immediate_mode(self, mode):
        """直发一次模式切换 (不排队/不读心跳, 引导与急停线程专用).
        PX4: MAV_CMD_DO_SET_MODE(param1=flags, param2=main, param3=sub),
        同时补发 SET_MODE 消息兜底(不等确认); 返回 PX4 规范模式名."""
        if not self.connected or self.raw is None:
            raise RuntimeError('未连接地面站 %s' % self.id)
        name = apc._resolve_mode(str(mode).upper())
        if name is None:
            raise ValueError('未知飞行模式: %s (PX4: %s)' % (
                mode, ','.join(sorted(apc.PX4_MODES))))
        flags, main_mode, sub_mode = apc.PX4_MODES[name]
        raw = self.raw
        raw.mav.command_long_send(
            raw.target_system, raw.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            flags, float(main_mode), float(sub_mode), 0, 0, 0, 0)
        try:
            raw.mav.set_mode_send(
                raw.target_system, flags,
                apc.pack_custom_mode(main_mode, sub_mode))
        except Exception:
            pass
        return name
