#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QuadPlane 测试 UI: 解锁 / 起飞 / 降落 / 锁定 + 实时状态.

依赖: pip install pymavlink
启动: python qp_ui.py            (默认 udp:127.0.0.1:14550, 适合 SITL)
      python qp_ui.py COM3 --baud 115200
"""

import argparse
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from pymavlink import mavutil

import qp_fly as qf

_FLAG_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

MODE_HINT = {
    'QSTABILIZE': '四轴自稳', 'QHOVER': '四轴定高', 'QLOITER': '四轴定点',
    'QLAND': '四轴降落', 'QRTL': '四轴返航',
    'FBWA': '固定翼A', 'FBWB': '固定翼B', 'CRUISE': '巡航',
    'LOITER': '固定翼盘旋', 'GUIDED': '引导', 'AUTO': '任务',
    'MANUAL': '手动', 'RTL': '返航', 'TAKEOFF': '固定翼起飞',
}

EKF_BITS = ((0x01, '姿态'), (0x02, '水平位置'), (0x04, '垂直位置'),
            (0x08, 'IMU'), (0x10, 'GPS'), (0x20, '地面控制'))


def fmt_ekf(flags):
    if flags is None:
        return '-'
    bad = [n for b, n in EKF_BITS if not (flags & b)]
    return 'OK' if not bad else ('缺:' + '/'.join(bad))


class QuadUI(object):
    POLL_MS = 150

    def __init__(self, root, conn, baud):
        self.root = root
        self.conn = conn
        self.baud = baud
        self.master = None
        self.reader = None
        self.reader_on = False
        self.worker = None
        self.mode_map = {}
        self.msg_q = queue.Queue()
        self.last_hb = 0.0

        self.v = {k: tk.StringVar(value='-') for k in (
            'link', 'mode', 'armed', 'alt', 'gs', 'vs', 'thr',
            'gps', 'batt', 'att', 'ekf', 'pos', 'time')}

        self._build()
        self.root.after(200, self._poll)
        if conn:
            self.root.after(300, self.on_connect)

    # ------------------------------------------------------------------ UI
    def _build(self):
        self.root.title('QuadPlane 垂起测试台')
        self.root.geometry('760x600')
        self.root.minsize(700, 540)

        top = ttk.Frame(self.root, padding=6)
        top.pack(fill='x')
        ttk.Label(top, text='连接:').pack(side='left')
        self.e_conn = ttk.Entry(top, width=26)
        self.e_conn.insert(0, self.conn)
        self.e_conn.pack(side='left', padx=4)
        ttk.Label(top, text='波特率:').pack(side='left')
        self.cb_baud = ttk.Combobox(top, width=8, values=(
            '115200', '57600', '38400', '19200', '921600'), state='normal')
        self.cb_baud.set(str(self.baud))
        self.cb_baud.pack(side='left', padx=4)
        self.b_conn = ttk.Button(top, text='连接', command=self.on_connect)
        self.b_conn.pack(side='left', padx=6)

        bar = ttk.LabelFrame(self.root, text='控制', padding=6)
        bar.pack(fill='x', padx=6, pady=(4, 2))
        row0 = ttk.Frame(bar)
        row0.pack(fill='x')
        self.b_arm = ttk.Button(row0, text='解锁 ARM',
                                command=lambda: self._cmd('arm'))
        self.b_arm.pack(side='left', padx=3)
        self.b_dis = ttk.Button(row0, text='锁定 DISARM',
                                command=lambda: self._cmd('disarm'))
        self.b_dis.pack(side='left', padx=3)
        ttk.Label(row0, text='高度m:').pack(side='left', padx=(12, 2))
        self.e_alt = ttk.Entry(row0, width=6)
        self.e_alt.insert(0, '10')
        self.e_alt.pack(side='left')
        self.b_to = ttk.Button(row0, text='起飞',
                               command=lambda: self._cmd('takeoff'))
        self.b_to.pack(side='left', padx=3)
        self.b_land = ttk.Button(row0, text='降落',
                                 command=lambda: self._cmd('land'))
        self.b_land.pack(side='left', padx=3)
        self.b_flow = ttk.Button(row0, text='完整流程',
                                 command=lambda: self._cmd('fly'))
        self.b_flow.pack(side='left', padx=(12, 3))
        self.b_stop = ttk.Button(row0, text='中断', command=self.on_stop,
                                 state='disabled')
        self.b_stop.pack(side='left', padx=3)

        row1 = ttk.Frame(bar)
        row1.pack(fill='x', pady=(6, 0))
        ttk.Label(row1, text='起飞模态:').pack(side='left')
        self.cb_mode = ttk.Combobox(row1, width=10, state='readonly',
                                    values=('QLOITER', 'QHOVER'))
        self.cb_mode.set('QLOITER')
        self.cb_mode.pack(side='left', padx=(2, 10))
        ttk.Label(row1, text='降落模态:').pack(side='left')
        self.cb_land = ttk.Combobox(row1, width=10, state='readonly',
                                    values=('QLAND', 'QRTL'))
        self.cb_land.set('QLAND')
        self.cb_land.pack(side='left', padx=(2, 10))
        self.v_rc = tk.BooleanVar(value=True)
        ttk.Checkbutton(row1, text='解锁覆盖油门',
                        variable=self.v_rc).pack(side='left', padx=8)

        st = ttk.LabelFrame(self.root, text='飞行状态', padding=6)
        st.pack(fill='x', padx=6, pady=2)
        st.columnconfigure(1, weight=1)
        st.columnconfigure(3, weight=1)
        grid = ttk.Frame(st)
        grid.pack(fill='x')
        pairs = (
            ('链路', 'link', '模式', 'mode'),
            ('解锁', 'armed', '相对高度', 'alt'),
            ('地速', 'gs', '爬升率', 'vs'),
            ('油门', 'thr', '电压/电流', 'batt'),
            ('GPS', 'gps', 'EKF', 'ekf'),
            ('GPS位置', 'pos', '飞控UTC', 'time'),
        )
        for i, (l1, k1, l2, k2) in enumerate(pairs):
            ttk.Label(grid, text=l1 + ':').grid(
                row=i, column=0, sticky='e', padx=(4, 2), pady=1)
            e1 = ttk.Label(grid, textvariable=self.v[k1], width=26,
                           relief='groove', padding=1)
            e1.grid(row=i, column=1, sticky='w', padx=(0, 12), pady=1)
            ttk.Label(grid, text=l2 + ':').grid(
                row=i, column=2, sticky='e', padx=(4, 2), pady=1)
            e2 = ttk.Label(grid, textvariable=self.v[k2], width=26,
                           relief='groove', padding=1)
            e2.grid(row=i, column=3, sticky='w', pady=1)
        # 姿态单独一行(需要更宽)
        fr = ttk.Frame(st)
        fr.pack(fill='x', pady=(4, 0))
        ttk.Label(fr, text='姿态 R/P/Y(°):').pack(side='left')
        ttk.Label(fr, textvariable=self.v['att'], width=30,
                  relief='groove', padding=1).pack(side='left', padx=4)

        lf = ttk.LabelFrame(self.root, text='日志', padding=4)
        lf.pack(fill='both', expand=True, padx=6, pady=(2, 6))
        self.txt = tk.Text(lf, height=14, state='disabled',
                           font=('Consolas', 9), wrap='word')
        sb = ttk.Scrollbar(lf, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        self.txt.pack(fill='both', expand=True)
        self.txt.tag_configure('ok', foreground='#0a7a0a')
        self.txt.tag_configure('err', foreground='#b00000')
        self.txt.tag_configure('fc', foreground='#1a4f8a')
        self.txt.tag_configure('warn', foreground='#a06000')

    # ------------------------------------------------------------- helpers
    def _log(self, msg, tag=None):
        def put():
            self.txt.configure(state='normal')
            self.txt.insert('end', '[%s] %s\n' % (
                time.strftime('%H:%M:%S'), msg))
            self.txt.see('end')
            self.txt.configure(state='disabled')
        self.root.after(0, put)

    def _set(self, key, val):
        self.root.after(0, lambda: self.v[key].set(val))

    def _btn_state(self, busy):
        st = 'disabled' if busy else 'normal'
        for b in (self.b_arm, self.b_dis, self.b_to, self.b_land, self.b_flow):
            b.configure(state=st)
        self.b_stop.configure(state='disabled' if not busy else 'normal')
        self.b_conn.configure(state=st)

    def _require_link(self):
        if self.master is None:
            self._log('未连接, 请先点[连接]', 'err')
            return False
        return True

    def _read_args(self):
        try:
            alt = float(self.e_alt.get())
        except ValueError:
            raise qf.QPError('高度必须是数字')
        if alt <= 0:
            raise qf.QPError('高度必须大于 0')
        return alt

    # ------------------------------------------------------------ commands
    def on_connect(self):
        if self.master is not None:
            self._log('已连接, 如需重连请重启程序')
            return
        conn = self.e_conn.get().strip()
        baud = self.cb_baud.get().strip() or '115200'
        try:
            baud = int(baud)
        except ValueError:
            self._log('波特率无效: %s' % baud, 'err')
            return
        self.e_conn.configure(state='disabled')
        self.cb_baud.configure(state='disabled')
        self.b_conn.configure(state='disabled')
        self._log('正在连接 %s ...' % conn)
        threading.Thread(target=self._connect_worker,
                         args=(conn, baud), daemon=True).start()

    def _connect_worker(self, conn, baud):
        qf.set_logger(lambda m: self.msg_q.put(('log', str(m))))
        try:
            master = qf.connect(conn, baud)
        except qf.QPError as e:
            self.msg_q.put(('err', str(e)))
            self.msg_q.put(('ui', 'reconnect'))
            return
        except Exception as e:
            self.msg_q.put(('err', '连接异常: %s: %s' % (
                type(e).__name__, e)))
            self.msg_q.put(('ui', 'reconnect'))
            return
        self.master = master
        try:
            self.mode_map = master.mode_mapping() or dict(qf.PLANE_MODE)
        except Exception:
            self.mode_map = dict(qf.PLANE_MODE)
        self.reader_on = True
        self.reader = threading.Thread(target=self._reader, daemon=True)
        self.reader.start()
        self.msg_q.put(('ok', '连接成功, 可以操作'))

    def _run_cmd(self, name, fn):
        """工作线程: 执行一个控制函数"""

        def work():
            self.msg_q.put(('ui', 'busy'))
            qf.set_logger(lambda m: self.msg_q.put(('log', str(m))))
            try:
                fn()
                self.msg_q.put(('ok', '%s 完成' % name))
            except qf.QPError as e:
                self.msg_q.put(('err', '%s 失败: %s' % (name, e)))
            except Exception as e:
                self.msg_q.put(('err', '%s 异常: %s: %s' % (
                    name, type(e).__name__, e)))
            finally:
                qf.set_logger(None)
                self.msg_q.put(('ui', 'idle'))
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def _cmd(self, name):
        if not self._require_link():
            return
        if self.worker is not None and self.worker.is_alive():
            self._log('上一个操作还在进行中', 'warn')
            return
        try:
            alt = self._read_args()
            mode = self.cb_mode.get()
            land_mode = self.cb_land.get()
            m = self.master
            if name == 'arm':
                self._run_cmd('解锁', lambda: (
                    qf.set_mode(m, mode),
                    qf.arm(m, use_rc_override=self.v_rc.get())))
            elif name == 'disarm':
                self._run_cmd('锁定', lambda: qf.disarm(m))
            elif name == 'takeoff':
                self._run_cmd('起飞', lambda: qf.takeoff(
                    m, alt, mode=mode, timeout=max(60.0, alt * 15)))
            elif name == 'land':
                self._run_cmd('降落', lambda: qf.land(m, mode=land_mode))
            elif name == 'fly':
                self._run_cmd('完整流程', lambda: self._flow(m, alt, mode,
                                                             land_mode))
        except qf.QPError as e:
            self._log(str(e), 'err')

    def _flow(self, m, alt, mode, land_mode):
        qf.wait_gps(m, timeout=60)
        qf.set_mode(m, mode)
        qf.arm(m, use_rc_override=self.v_rc.get())
        qf.takeoff(m, alt, mode=mode, timeout=max(60.0, alt * 15))
        self._log('悬停 5 秒...')
        time.sleep(5)
        qf.land(m, mode=land_mode)
        if qf.is_armed(m):
            qf.disarm(m)

    def on_stop(self):
        """中断: 后台尝试强制上锁"""
        m = self.master
        if m is None:
            return
        self._log('中断请求: 强制上锁...', 'warn')

        def work():
            try:
                qf.set_logger(lambda msg: self.msg_q.put(('log', str(msg))))
                qf.disarm(m, force=True)
                self.msg_q.put(('ok', '强制上锁完成'))
            except Exception as e:
                self.msg_q.put(('err', '强制上锁失败: %s' % e))
            finally:
                qf.set_logger(None)
                self.msg_q.put(('ui', 'idle'))
        threading.Thread(target=work, daemon=True).start()

    # -------------------------------------------------------------- reader
    def _reader(self):
        m = self.master
        while self.reader_on and m is not None:
            try:
                msg = m.recv_match(blocking=True, timeout=0.4)
            except Exception:
                break
            if msg is None or not qf._from_fc(m, msg):
                continue
            try:
                self._handle(msg)
            except Exception:
                pass

    def _handle(self, msg):
        t = msg.get_type()
        if t == 'HEARTBEAT':
            self.last_hb = time.time()
            armed = bool(msg.base_mode & _FLAG_ARMED)
            name = self.mode_map.get(msg.custom_mode,
                                     'MODE(%d)' % msg.custom_mode)
            hint = MODE_HINT.get(name, '')
            self._set('mode', '%s %s' % (name, hint) if hint else name)
            self._set('armed', '已解锁 ARMED' if armed else '上锁 DISARMED')
        elif t == 'GLOBAL_POSITION_INT':
            rel = msg.relative_alt / 1000.0
            gs = ((msg.vx ** 2 + msg.vy ** 2) ** 0.5) / 100.0
            vs = -msg.vz / 100.0
            hdg = msg.hdg / 100.0 if msg.hdg != 65535 else float('nan')
            self._set('alt', '%.1f m' % rel)
            self._set('gs', '%.1f m/s' % gs)
            self._set('vs', '%+.1f m/s' % vs)
            self._set('pos', '%.6f, %.6f  航向%.0f°' % (
                msg.lat / 1e7, msg.lon / 1e7, hdg))
        elif t == 'GPS_RAW_INT':
            self._set('gps', 'fix=%d 星=%d%s' % (
                msg.fix_type, msg.satellites_visible,
                '' if msg.eph == 65535 else ' 精度%.1fm' % (msg.eph / 100.0)))
        elif t == 'SYS_STATUS':
            v = msg.voltage_battery / 1000.0
            c = msg.current_battery / 100.0
            self._set('batt', '%.1fV %.1fA' % (v, c) if c >= 0
                      else '%.1fV' % v)
        elif t == 'VFR_HUD':
            self._set('thr', '%d%%' % msg.throttle)
        elif t == 'ATTITUDE':
            self._set('att', '%.1f / %.1f / %.1f' % (
                math.degrees(msg.roll), math.degrees(msg.pitch),
                math.degrees(msg.yaw)))
        elif t == 'EKF_STATUS_REPORT':
            self._set('ekf', fmt_ekf(msg.flags or 0))
        elif t == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            self._log('[飞控] %s' % str(text), 'fc')
        elif t == 'SYSTEM_TIME':
            try:
                utc = msg.time_unix_usec / 1e6
                self._set('time', time.strftime(
                    '%Y-%m-%d %H:%M:%S', time.gmtime(utc)) + ' UTC')
            except Exception:
                pass

    # ---------------------------------------------------------------- poll
    def _poll(self):
        try:
            while True:
                kind, payload = self.msg_q.get_nowait()
                if kind == 'log':
                    self._log(payload)
                elif kind == 'ok':
                    self._log(payload, 'ok')
                elif kind == 'err':
                    self._log(payload, 'err')
                elif kind == 'ui':
                    if payload == 'busy':
                        self._btn_state(True)
                    elif payload == 'idle':
                        self._btn_state(False)
                    elif payload == 'reconnect':
                        self.e_conn.configure(state='normal')
                        self.cb_baud.configure(state='normal')
                        self.b_conn.configure(state='normal')
        except queue.Empty:
            pass

        if self.master is not None:
            age = time.time() - self.last_hb
            self.v['link'].set('已连接 %s' % self.conn
                               if age < 3 else '心跳丢失 %.0fs!' % age)
        self.root.after(self.POLL_MS, self._poll)

    def on_close(self):
        self.reader_on = False
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description='QuadPlane 测试 UI')
    ap.add_argument('conn', nargs='?', default='udp:127.0.0.1:14550',
                    help='连接串 (默认 udp:127.0.0.1:14550)')
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()
    root = tk.Tk()
    app = QuadUI(root, args.conn, args.baud)
    root.protocol('WM_DELETE_WINDOW', app.on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
