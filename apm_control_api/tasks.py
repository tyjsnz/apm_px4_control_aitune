# -*- coding: utf-8 -*-
"""链路任务管理 + 单点飞行 / 航点任务 / 解锁起飞等控制任务.

控制逻辑复用 apm_control.py 与 apm_control_test.py (与 apm_control_ui 相同),
所有控制函数在后台线程执行, stdout 由 tasklog 路由进任务日志.
"""
import os
import sys
import threading
import time
import traceback

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import apm_control as apc
import apm_control_test as tct

from . import config
from .link import haversine
from .tasklog import TaskLogBuffer, bind, unbind


class TaskError(Exception):
    pass


class BusyError(Exception):
    pass


def _run(fn, *args, **kwargs):
    """调用会 sys.exit 的控制函数, 把 SystemExit 转成 TaskError"""
    try:
        return fn(*args, **kwargs)
    except SystemExit as e:
        raise TaskError(str(e) or '控制函数中止')


class TaskCtx(object):
    def __init__(self, manager, record):
        self.manager = manager
        self.link = manager.link
        self.record = record
        self.cancel = record.cancel_event

    @property
    def proxy(self):
        return self.link.proxy

    def check_cancel(self):
        if self.cancel.is_set():
            raise TaskError('任务已被取消')

    def set_update(self, data):
        self.manager.set_update(data)


class TaskRecord(object):
    def __init__(self, kind, params):
        self.id = '%s-%d' % (kind, int(time.time() * 1000) % 100000000)
        self.kind = kind
        self.params = params
        self.status = 'pending'
        self.started_at = None
        self.finished_at = None
        self.result = None
        self.error = None
        self.log = TaskLogBuffer(config.TASK_LOG_MAX_LINES)
        self.lock = threading.Lock()
        self.cancel_event = threading.Event()

    def to_dict(self, log_tail=None):
        with self.lock:
            d = {
                'id': self.id,
                'kind': self.kind,
                'status': self.status,
                'params': self.params,
                'started_at': self.started_at,
                'finished_at': self.finished_at,
                'duration_s': (round(self.finished_at - self.started_at, 1)
                               if self.started_at and self.finished_at else None),
                'result': self.result,
                'error': self.error,
            }
        if log_tail:
            d['log'] = self.log.snapshot(tail=log_tail)
        return d


class TaskManager(object):
    def __init__(self, link):
        self.link = link
        self.lock = threading.Lock()
        self.active = []
        self.current = None
        self.history = []
        self._update_epoch = 0
        self._update_data = None
        self._update_lock = threading.Lock()

    # ---------------- 状态 ----------------

    def _live(self):
        with self.lock:
            return [r for r in self.active if r.status in ('pending', 'running')]

    def status(self):
        live = self._live()
        with self.lock:
            cur = self.current
            total = len(self.history) + len(self.active)
        if not live:
            return {'current': None, 'total': total, 'busy': False}
        return {'current': (cur or live[0]).to_dict(), 'total': total,
                'busy': True}

    def list(self, log_tail=200):
        with self.lock:
            items = list(self.history) + list(self.active)
        out = [r.to_dict(log_tail=log_tail) for r in items]
        out.reverse()
        return out

    def get(self, task_id, log_tail=2000):
        with self.lock:
            items = list(self.history) + list(self.active)
        for r in items:
            if r.id == task_id:
                return r.to_dict(log_tail=log_tail)
        return None

    # ---------------- 执行 ----------------

    def start(self, kind, params, fn, exclusive=True):
        """启动任务; exclusive=True 时已有任务在跑会抛 BusyError"""
        if exclusive:
            self.preempt('settle_land')
            live = self._live()
            if live:
                raise BusyError('链路 %s 正在执行任务 %s(%s)'
                                % (self.link.id, live[0].kind, live[0].id))
        rec = TaskRecord(kind, params)
        with self.lock:
            self.active.append(rec)
            self.current = rec
            self._update_epoch = 0
            self._update_data = None
        th = threading.Thread(target=self._run, args=(rec, fn),
                              name='task-%s' % rec.id, daemon=True)
        rec.thread = th
        th.start()
        return rec

    def _run(self, rec, fn):
        bind(rec.log)
        rec.started_at = time.time()
        with rec.lock:
            rec.status = 'running'
        ctx = TaskCtx(self, rec)
        try:
            result = fn(ctx)
            rec.result = result
            if rec.cancel_event.is_set():
                rec.status = 'cancelled'
            elif result is False:
                rec.status = 'failed'
            else:
                rec.status = 'success'
        except TaskError as e:
            rec.error = str(e)
            rec.status = 'cancelled' if rec.cancel_event.is_set() else 'failed'
        except Exception as e:
            rec.error = '%s: %s' % (type(e).__name__, e)
            rec.traceback = traceback.format_exc()
            rec.status = 'failed'
            print('❌ 任务异常: %s' % rec.error)
        finally:
            rec.finished_at = time.time()
            unbind()
            with self.lock:
                if rec in self.active:
                    self.active.remove(rec)
                self.history.append(rec)
                if len(self.history) > config.TASK_HISTORY:
                    del self.history[:len(self.history) - config.TASK_HISTORY]
                if self.current is rec:
                    live = [r for r in self.active
                            if r.status in ('pending', 'running')]
                    self.current = live[-1] if live else None

    def cancel(self, kind=None):
        """取消所有(或指定 kind 的)在跑任务; 返回取消条数"""
        with self.lock:
            targets = [r for r in self.active
                       if r.status in ('pending', 'running')
                       and (kind is None or r.kind == kind)]
        for rec in targets:
            rec.cancel_event.set()
        return len(targets)

    def preempt(self, kind, timeout=3.0):
        """取消并等待指定 kind 的任务退出; 返回 (取消数, 是否已全部退出)"""
        n = self.cancel(kind)
        if not n:
            return 0, True
        t_end = time.time() + timeout
        while time.time() < t_end:
            with self.lock:
                live = [r for r in self.active
                        if r.status in ('pending', 'running')
                        and r.kind == kind]
            if not live:
                return n, True
            time.sleep(0.1)
        return n, False

    def set_update(self, data):
        with self._update_lock:
            self._update_epoch += 1
            self._update_data = data
            return self._update_epoch

    def pop_update(self, epoch):
        with self._update_lock:
            if self._update_epoch != epoch:
                return self._update_epoch, self._update_data
            return epoch, None


# ---------------- 具体任务 ----------------

def do_goto(ctx, req):
    """单点飞行: 解锁起飞(可选) -> GUIDED 反复 goto -> 到达判定"""
    link = ctx.link
    p = ctx.proxy
    lat, lon, alt = float(req.lat), float(req.lon), float(req.alt)
    threshold = float(req.threshold or config.GOTO_DEFAULT_THRESHOLD)
    speed = req.speed

    with link.tel_lock:
        armed = link.tel.get('armed')
    if not armed:
        if not req.auto_takeoff:
            raise TaskError('飞机未解锁, 且 auto_takeoff=false')
        to_alt = max(alt, config.MISSION_TAKEOFF_ALT)
        if not _run(tct.arm_and_takeoff, p, to_alt, mode='GUIDED',
                    arm_timeout=10, takeoff_timeout=45, set_delay=True):
            raise TaskError('自动解锁起飞失败, 单点飞行取消')
        print('  ✅ 起飞完成, 飞往目的地')
    else:
        _run(apc.set_mode, p, 'GUIDED')
        print('  已确保 GUIDED, 飞往目的地...')

    if speed:
        if not tct.send_change_speed(p, float(speed), timeout=2.5):
            print('  ⚠️ 速度指令未被接受, 按飞控现有 WPNAV_SPEED 飞行')

    pos0 = link.fresh_position() or (lat, lon, alt)
    d0 = haversine(pos0[0], pos0[1], lat, lon)
    timeout = float(req.timeout or max(60.0, 30.0 + d0 / 2.0))
    print('  距目的地 %.1fm, 超时 %.0fs ...' % (d0, timeout))

    t_end = time.time() + timeout
    last_goto = 0.0
    epoch = 0
    while time.time() < t_end:
        ctx.check_cancel()
        epoch, upd = ctx.manager.pop_update(epoch)
        if upd:
            lat, lon, alt = float(upd['lat']), float(upd['lon']), float(upd['alt'])
            pos0 = link.fresh_position() or (lat, lon, alt)
            d0 = haversine(pos0[0], pos0[1], lat, lon)
            t_end = time.time() + max(60.0, 30.0 + d0 / 2.0)
            print('  ✈ 改点 → %.6f, %.6f 高 %.1fm' % (lat, lon, alt))
        now = time.time()
        if now - last_goto >= 1.0:
            apc.goto(p, lat, lon, alt)
            last_goto = now
        m = p.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        if m is None:
            continue
        d = haversine(m.lat / 1e7, m.lon / 1e7, lat, lon)
        rela = m.relative_alt / 1000.0
        if d < threshold and abs(rela - alt) < config.GOTO_ARRIVAL_ALT_TOL:
            print('  ✅ 已到达目的地 (水平差 %.1fm)' % d)
            return {'arrived': True, 'distance_m': round(d, 1),
                    'lat': lat, 'lon': lon, 'alt': alt}
    print('  ❌ 到达超时 (%.0fs), 飞机保持 GUIDED 悬停' % timeout)
    return {'arrived': False, 'reason': 'timeout', 'timeout_s': timeout,
            'lat': lat, 'lon': lon, 'alt': alt}


def do_mission(ctx, req):
    """航点任务: 上传任务 -> 解锁起飞 -> AUTO 执行 -> 结束动作"""
    link = ctx.link
    p = ctx.proxy
    items = []
    n_wp = 0
    for w in req.waypoints:
        if w.drop is not None:
            items.append(('DROP', w.drop.channel, w.drop.pwm_on,
                          w.drop.pwm_off, w.drop.hold_ms))
        else:
            items.append((float(w.lat), float(w.lon), float(w.alt)))
            n_wp += 1
    if n_wp == 0:
        raise TaskError('任务里没有有效航点(只有抛投项)')
    end_action = req.end_action
    print('开始航点任务: %d 航点, 结束动作 %s' % (n_wp, end_action.upper()))
    for i, it in enumerate(items):
        print('  [%d] %s' % (i + 1, it))

    if not _run(apc.upload_mission, p, items, end_action=end_action):
        raise TaskError('航点任务上传失败')
    print('  ✅ 任务上传成功')

    takeoff_alt = float(req.takeoff_alt or max(
        [config.MISSION_TAKEOFF_ALT] + [w.alt for w in req.waypoints
                                        if w.drop is None]))
    with link.tel_lock:
        armed = link.tel.get('armed')
    if not armed:
        if not req.auto_takeoff:
            raise TaskError('飞机未解锁, 且 auto_takeoff=false')
        if not _run(tct.arm_and_takeoff, p, takeoff_alt, mode='GUIDED',
                    arm_timeout=10, takeoff_timeout=45, set_delay=True):
            raise TaskError('起飞失败, 任务取消')
    else:
        _run(apc.set_mode, p, 'GUIDED')

    _run(apc.reset_mission_to_start, p)
    _run(apc.set_mode, p, 'AUTO')

    wps = [it for it in items if not (isinstance(it, tuple)
                                      and str(it[0]).upper() == 'DROP')]
    dist = sum(haversine(wps[i][0], wps[i][1], wps[i + 1][0], wps[i + 1][1])
               for i in range(len(wps) - 1)) if len(wps) > 1 else 0.0
    timeout = float(req.timeout or min(
        config.MISSION_TOTAL_TIMEOUT, 120 + 30 * len(wps) + dist))
    print('  任务超时预算 %ds (航线 %.0f m)' % (int(timeout), dist))

    ok = _run(apc.wait_auto_mission, p, len(wps), end_action, timeout)
    result = {'completed': bool(ok), 'waypoints': n_wp,
              'end_action': end_action, 'distance_m': round(dist, 1)}
    if not ok:
        raise TaskError('航点任务未确认完成(已离开 AUTO 或超时), '
                        '请检查飞机状态')
    if end_action in ('rtl', 'land'):
        print('  等待降落...')
        landed = _run(apc.wait_landed, p, timeout=300)
        result['landed'] = bool(landed)
        if landed and p.motors_armed():
            _run(tct.disarm_vehicle, p, force=False, timeout=6)
    return result


def do_arm(ctx, req):
    ok = _run(tct.arm_vehicle, ctx.proxy, mode=req.mode, timeout=10)
    return {'armed': bool(ok)}


def do_disarm(ctx, req):
    ok = _run(tct.disarm_vehicle, ctx.proxy, force=bool(req.force), timeout=6)
    if ok:
        try:
            tct.release_rc_override(ctx.proxy)
        except Exception:
            pass
    return {'disarmed': bool(ok)}


def do_takeoff(ctx, req):
    link = ctx.link
    p = ctx.proxy
    if req.mode == 'ALT_HOLD':
        ok = _run(tct.arm_vehicle, p, mode='ALT_HOLD', timeout=10)
        if not ok:
            raise TaskError('ALT_HOLD 解锁失败')
        ok = _run(tct.takeoff_althold, p, float(req.alt), timeout=90)
        if not ok:
            raise TaskError('定高起飞失败')
        return {'alt': req.alt, 'mode': 'ALT_HOLD'}
    if not _run(tct.arm_and_takeoff, p, float(req.alt), mode='GUIDED',
                arm_timeout=10, takeoff_timeout=60, set_delay=True):
        raise TaskError('解锁起飞失败')
    return {'alt': req.alt, 'mode': 'GUIDED'}


def do_set_mode(ctx, req):
    mode = str(req.mode).strip().upper()
    _run(apc.set_mode, ctx.proxy, mode)
    return {'mode': mode}


def do_settle_land(ctx):
    """急停(RTL/LAND)后的后台跟进: 等到低于接管高度后软着陆并上锁"""
    p = ctx.proxy
    t_end = time.time() + config.LANDING_TIMEOUT
    print('  等待降落: 低于 1m 后接管软着陆 (最长 %ds)...'
          % int(config.LANDING_TIMEOUT))
    while True:
        ctx.check_cancel()
        t = ctx.link.tel_copy()
        if t.get('armed') is False:
            return {'landed': True, 'disarmed': True}
        if t.get('rel_alt') is not None and t.get('rel_alt') <= 1.0:
            break
        if time.time() > t_end:
            raise TaskError('等待降落超时 (%ss), 请人工处置'
                            % int(config.LANDING_TIMEOUT))
        time.sleep(0.2)
    ctx.check_cancel()
    landed = _run(apc.wait_landed, p, timeout=60)
    if not landed and p.motors_armed():
        raise TaskError('软着陆未确认触地, 请人工处置')
    return {'landed': bool(landed), 'disarmed': not p.motors_armed()}


def do_rc_handover(ctx, req):
    tct.release_rc_override(ctx.proxy, handover=True)
    return {'handed_over': True}
