# -*- coding: utf-8 -*-
"""外部修正坐标 -> OFFBOARD 持续引导流.

参考 docs/外部坐标引导飞行.md:
  - 坐标系 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (相对起飞点高度)
  - 有速度时用 位置+速度 掩码(0x7C0)做前馈, 否则纯位置(0x7F8)
  - 下发频率默认 10Hz (建议 5~20Hz); 首次切 OFFBOARD 前先按
    apc.OFFBOARD_RATE_HZ 预热 apc.OFFBOARD_WARMUP_SEC 秒 (PX4 官方要求)
  - 高度给绝对高程(AMSL)时按 home 高程换算成相对高度
  - 持续刷新目标 => 飞机跟踪移动目标; 刷新停止后仍按最后目标点继续飞到位
"""
import threading
import time

from pymavlink import mavutil

from . import config
from .link import haversine
from .tasks import arm_and_takeoff

import px4_control as apc

MASK_POSITION = 0b0000011111111000   # 0x7F8 只控位置 (同 apc.POSITION_TYPE_MASK)
MASK_POSITION_VEL = 0b0000011111000000   # 0x7C0 位置+速度前馈 (0x7F8 开出速度位)


def to_ned(vx, vy, vz, frame):
    if frame == 'enu':
        return vy, vx, -vz
    return vx, vy, vz


class GuidanceStreamer(object):
    def __init__(self, link):
        self.link = link
        self.lock = threading.Lock()
        self.thread = None
        self.running = False
        self.state = 'idle'
        self.error = None
        self.target = None
        self.rate_hz = config.GUIDANCE_RATE_HZ
        self.sent = 0
        self.received = 0
        self.last_sent_at = 0.0
        self.started_at = 0.0
        self.expires_at = None
        self.stop_reason = None
        self.ensure_offboard = True
        self._evt = threading.Event()
        self._takeoff_alt = None
        self._auto_takeoff = False

    # ---------------- 目标解析 ----------------

    def _resolve(self, req):
        lat = float(req.lat)
        lon = float(req.lon)
        alt = float(req.alt)
        alt_frame = getattr(req, 'alt_frame', 'relative')
        if alt_frame == 'amsl':
            origin = self.link.origin_alt_amsl()
            if origin is None:
                raise ValueError('无法换算绝对高程: 尚未取得 HOME/起飞点高程')
            alt = alt - origin
        vx, vy, vz = to_ned(float(getattr(req, 'vx', 0.0) or 0.0),
                            float(getattr(req, 'vy', 0.0) or 0.0),
                            float(getattr(req, 'vz', 0.0) or 0.0),
                            getattr(req, 'vel_frame', 'ned'))
        has_vel = bool(abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(vz) > 1e-6)
        return {
            'lat': lat, 'lon': lon, 'alt': max(0.1, alt),
            'vx': vx, 'vy': vy, 'vz': vz, 'has_vel': has_vel,
            'id': getattr(req, 'id', None),
            'type': int(getattr(req, 'type', 0) or 0),
            'track': bool(getattr(req, 'track', True)),
            'confidence': getattr(req, 'confidence', None),
            'received_at': time.time(),
        }

    def _min_confidence(self, req):
        v = getattr(req, 'min_confidence', None)
        if v is None:
            return config.GUIDANCE_MIN_CONFIDENCE
        return int(v)

    # ---------------- 对外接口 ----------------

    def submit(self, req):
        """接收一条外部修正数据; 返回 (accepted, detail)"""
        conf = getattr(req, 'confidence', None)
        lo = self._min_confidence(req)
        if conf is not None and conf < lo:
            return False, {'reason': 'confidence %s < min %s' % (conf, lo)}
        auto_takeoff = bool(getattr(req, 'auto_takeoff', False))
        armed = self.link.tel_copy().get('armed')
        if armed is None:
            return False, {'reason': '未收到飞控心跳, 链路不可用'}
        if not armed and not auto_takeoff:
            return False, {'reason': '飞机未解锁且未启用 auto_takeoff, '
                                     '引导目标无法生效'}
        with self.lock:
            tgt = self._resolve(req)
            self.target = tgt
            self.received += 1
            track = tgt['track']
            rate = getattr(req, 'rate_hz', None) or config.GUIDANCE_RATE_HZ
            self.rate_hz = max(config.GUIDANCE_MIN_RATE_HZ,
                               min(config.GUIDANCE_MAX_RATE_HZ, float(rate)))
            self.ensure_offboard = bool(getattr(req, 'ensure_offboard', True))
            self._auto_takeoff = auto_takeoff
            ta = getattr(req, 'takeoff_alt', None)
            self._takeoff_alt = float(ta) if ta else max(
                tgt['alt'], config.MISSION_TAKEOFF_ALT)
            ttl = getattr(req, 'ttl', None)
            ttl = config.GUIDANCE_DEFAULT_TTL if ttl is None else float(ttl)
            if ttl > 0:
                self.expires_at = time.time() + ttl
            else:
                self.expires_at = None
            if track:
                if not self.running:
                    self._start_locked()
            elif self.running:
                pass
        if not track:
            self._send_once(tgt)
            return True, self.status()
        return True, self.status()

    def _start_locked(self):
        self._evt = threading.Event()
        self.running = True
        self.state = 'starting'
        self.error = None
        self.stop_reason = None
        self.sent = 0
        self.started_at = time.time()
        th = threading.Thread(target=self._run, name='guidance-%s' % self.link.id,
                              daemon=True)
        self.thread = th
        th.start()

    def stop(self, reason='stopped'):
        with self.lock:
            if not self.running:
                self.state = 'idle'
                self.stop_reason = reason
                return False
            self.running = False
            self.state = 'stopped'
            self.stop_reason = reason
            evt = self._evt
            evt.set()
            th = self.thread
        if th is not None and th.is_alive() \
                and th is not threading.current_thread():
            th.join(timeout=2.0)
        return True

    def status(self):
        with self.lock:
            tgt = dict(self.target) if self.target else None
            st = {
                'active': self.running,
                'state': self.state,
                'rate_hz': self.rate_hz,
                'sent': self.sent,
                'received': self.received,
                'last_sent_at': self.last_sent_at or None,
                'started_at': self.started_at or None,
                'expires_at': self.expires_at,
                'stop_reason': self.stop_reason,
                'error': self.error,
                'ensure_offboard': self.ensure_offboard,
                'auto_takeoff': self._auto_takeoff,
            }
            if tgt:
                st['target'] = {
                    'id': tgt['id'], 'type': tgt['type'],
                    'lat': tgt['lat'], 'lon': tgt['lon'], 'alt': tgt['alt'],
                    'vx': tgt['vx'], 'vy': tgt['vy'], 'vz': tgt['vz'],
                    'has_vel': tgt['has_vel'], 'track': tgt['track'],
                    'confidence': tgt['confidence'],
                    'received_at': tgt['received_at'],
                }
                now = time.time()
                st['target_age_s'] = round(now - tgt['received_at'], 2)
                st['expires_in_s'] = (round(self.expires_at - now, 1)
                                      if self.expires_at else None)
                pos = self.link.fresh_position(timeout=5.0)
                if pos:
                    d = haversine(pos[0], pos[1], tgt['lat'], tgt['lon'])
                    st['distance_m'] = round(d, 1)
                    st['arrived'] = d <= config.GOTO_DEFAULT_THRESHOLD
                    st['rel_alt'] = pos[2]
                else:
                    st['distance_m'] = None
                    st['arrived'] = None
            t = self.link.tel_copy()
            st['armed'] = t.get('armed')
            st['mode'] = t.get('mode')
            return st

    # ---------------- 发送 ----------------

    def _send_target(self, tgt):
        raw = self.link.raw
        if raw is None:
            raise RuntimeError('链路已断开')
        mask = MASK_POSITION_VEL if tgt['has_vel'] else MASK_POSITION
        raw.mav.set_position_target_global_int_send(
            0,
            raw.target_system,
            raw.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mask,
            int(round(tgt['lat'] * 1e7)),
            int(round(tgt['lon'] * 1e7)),
            float(tgt['alt']),
            float(tgt['vx']), float(tgt['vy']), float(tgt['vz']),
            0, 0, 0,
            0, 0)

    def _send_once(self, tgt):
        for i in range(5):
            try:
                self._send_target(tgt)
                with self.lock:
                    self.sent += 1
                    self.last_sent_at = time.time()
                    if i == 0:
                        self.state = 'sent_once'
                        self.stop_reason = 'track=false'
            except Exception as e:
                with self.lock:
                    self.error = str(e)
                return
            time.sleep(0.1)

    def _prepare(self):
        link = self.link
        with link.tel_lock:
            armed = link.tel.get('armed')
        if armed is None:
            raise RuntimeError('未收到飞控心跳, 无法引导')
        if not armed:
            if not self._auto_takeoff:
                raise RuntimeError('飞机未解锁且未启用 auto_takeoff, 目标无法生效')
            with link.task_lock:
                if not arm_and_takeoff(link.proxy, self._takeoff_alt,
                                       mode='POSCTL', arm_timeout=10,
                                       takeoff_timeout=45, set_delay=True):
                    raise RuntimeError('自动解锁起飞失败')

    def _run(self):
        evt = self._evt
        try:
            with self.lock:
                self.state = 'preparing'
            self._prepare()
            with self.lock:
                self.state = 'running'
            warmup_t0 = time.time()
            last_mode_fix = 0.0
            while not evt.is_set():
                with self.lock:
                    tgt = dict(self.target) if self.target else None
                    rate = self.rate_hz
                    expires = self.expires_at
                    ensure = self.ensure_offboard
                if tgt is None:
                    break
                if expires and time.time() > expires:
                    self.stop('ttl_expired')
                    break
                now = time.time()
                in_warmup = now - warmup_t0 < apc.OFFBOARD_WARMUP_SEC
                try:
                    self._send_target(tgt)
                    with self.lock:
                        self.sent += 1
                        self.last_sent_at = time.time()
                        self.error = None
                except Exception as e:
                    with self.lock:
                        self.error = str(e)
                        self.state = 'error'
                    if not self.link.connected:
                        break
                if ensure and not in_warmup and now - last_mode_fix >= 2.0:
                    last_mode_fix = now
                    t = self.link.tel_copy()
                    if t.get('mode') != 'OFFBOARD' and t.get('armed') is True:
                        try:
                            self.link.immediate_mode('OFFBOARD')
                        except Exception as e:
                            with self.lock:
                                self.error = str(e)
                period = 1.0 / float(rate)
                if in_warmup:
                    period = min(period, 1.0 / apc.OFFBOARD_RATE_HZ)
                evt.wait(period)
        except Exception as e:
            with self.lock:
                self.error = str(e)
                self.state = 'error'
                self.running = False
                self.stop_reason = 'error'
