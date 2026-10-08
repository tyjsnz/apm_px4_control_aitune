#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
APM 功能测试 UI 界面 (apm_control_ui.py)

功能与 apm_control_test.py 完全一致(测试1~14 + 运行全部), 另外提供:
  基础:
  - 实时遥测: 模式/解锁/GPS/位置/速度/姿态/电池/RC/EKF/链路状态
  - 日志窗口: 测试与飞控 STATUSTEXT 输出实时显示
  - 控制信号: 最近 MAVLink 指令及发送频率
  - 快速控制: 模式切换/解锁/起飞/降落/RTL/锁定/速度微动/抛投
    (通道/触发PWM/回位PWM/保持ms, 二次确认; 空闲入队带ACK, 任务中直发)
  - 参数面板: 连接串/波特率/起飞高度等, 与 apm_control_test 共享
  安全:
  - 急停: 一键 LOITER 悬停(快捷键F9) + 立即 RTL/LAND, 直发不排队
  - 地理围栏: 离HOME距离/相对高度限制, 越界告警可选自动RTL
  - 低电量保护: 告警电压/触发电压/电量阈值, 可选 LAND/RTL
  - 心跳丢失监控: 超时红字告警(阈值可调)
  - 飞行日志录制: 遥测 CSV (5Hz)
  调试/效率:
  - 小地图: 多底图源(默认 Esri卫星影像 WGS-84 免密钥; 天地图矢量/影像
    WGS-84 需免费tk; 高德 GCJ-02 备选), WGS-84 无偏移叠加,
    HOME/航迹/当前位置, 滚轮缩放/拖拽平移/回中HOME/跟随飞机,
    瓦片磁盘缓存(离线可复用), 无 GPS 也能立即加载(默认北京)
  - 姿态仪表: 地平仪(roll/pitch)
  - 目标跟踪曲线: 高度/地速 实际 vs 目标
  - 参数读写面板: 飞控参数读取/写入 + 历史
  - RC覆盖滑条: 4通道覆盖(仅地面调试, 任务运行时自动释放)
  - 航点任务: 解析/上传/一键执行(复用 apm_control.py), 支持 DROP 行
    自动抛投 (到航点后 DO_SET_SERVO + NAV_DELAY 保持 + 回位),
    航点与单点目的地均由用户自行填写 GPS 数据
  - 测试报告导出
  - 遥测消息速率表

依赖: pip install pymavlink   (tkinter 为 Python 自带)
运行: python apm_control_ui.py
"""
import json
import collections
import csv
import math
import os
import queue
import sys
import threading
import time
import traceback
import tkinter as tk
import urllib.request
from tkinter import filedialog, messagebox, ttk

try:
    import io as _io
    from PIL import Image as _PILImage
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False

from pymavlink import mavutil

import apm_control as apc
import apm_control_test as tct

try:
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

# ============ 默认配置 ============
DEFAULT_CONN = 'COM23'
DEFAULT_BAUD = 57600

# ============ 串口配置持久化 ============
CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'apm_ui_config.json')
LAST_PORT_KEY = 'last_port'
TELEM_PERIOD_MS = 200
LOG_MAX_LINES = 3000
ALT_HISTORY = 150          # 高度曲线点数
CURVE_HISTORY = 300        # 目标跟踪曲线点数
TRACK_HISTORY = 800        # 地图航迹点数
ALARM_COOLDOWN = 10.0      # 同类告警最短间隔(秒)
CSV_DIR = 'flight_logs'    # 飞行日志目录
# 软遥控方向: 实机方向相反时把对应项改为 -1 (键位: W前/S后 A左/D右 Q左/E右)
SOFT_RC_INVERT = {'pitch': 1, 'roll': 1, 'yaw': 1}

FIX_NAMES = {0: '无定位', 1: '无GPS', 2: '2D', 3: '3D',
             4: 'DGPS', 5: 'RTK浮', 6: 'RTK固'}
SYS_NAMES = {0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING', 3: 'STANDBY',
             4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY', 7: 'POWEROFF',
             8: 'TERM'}

CSV_HEADER = ['time', 'mode', 'armed', 'lat', 'lon', 'rel_alt', 'abs_alt',
              'gs', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'hdg',
              'volt', 'curt', 'remain', 'thr', 'sats', 'fix',
              'rc_override', 'last_cmd', 'dist_home', 'target_alt',
              'target_speed', 'status_text']

# ============ 发射箱参数说明 (v1.32: 点[写入发射箱参数]先弹说明确认页) ============
# 键=参数名(对应 tct.LAUNCH_PARAMS 与 DISARM_DELAY), 值=写入前给操作员看的说明
LAUNCH_PARAM_DOC = {
    'FS_THR_ENABLE': '遥控器(油门)失控保护动作. 0=关闭, 信号丢失后保持当前模式'
                     ' 不自动切模式/不自动降落; 1=RTL 返航, 2=Land 降落',
    'FS_GCS_ENABLE': '地面站(GCS)心跳断链保护. 0=关闭, 断链后飞控不返航不降落,'
                     ' 继续保持当前模式(由地面站/发射箱接管); 1=RTL, 2=SmartRTL',
    'FS_DR_ENABLE': '惯导死亡推算(DR)失败保护. 0=关闭, 位置估计丢失时不自动'
                    ' 返航/降落; 1=Land, 2=RTL, 3=SmartRTL或RTL',
    'FS_DR_TIMEOUT': '惯导可用秒数: 超过后转入EKF失败保护(会降落). '
                     '0=关闭该超时计时, 不因惯导超时结束当前动作',
    'FS_EKF_ACTION': 'EKF(位置估计)失败动作. 0=不降落/不切模式, 只在地面站'
                     ' 发告警; 非0则会自动降落或返航',
    'FENCE_ENABLE': '飞控地理围栏失效保护开关. 0=围栏触发后飞控不自动动作;'
                    ' 1=越界自动执行围栏动作(默认RTL/降落)',
    'BRD_SAFETYENABLE': '硬件安全开关(机身红色安全按钮)是否必须按下才允许解锁.'
                        ' 0=不要求, 发射箱/台架无安全开关时必须为0, '
                        '否则 PreArm: Safety Switch',
    'BATT_FS_LOW_ACT': '电池低电压故障保护动作. 0=仅蜂鸣/告警, 不自动降落;'
                       ' 1=RTL, 2=Land',
    'BATT_FS_CRT_ACT': '电池电量危急故障保护动作. 0=仅告警, 不自动降落; '
                       '1=RTL, 2=Land',
    'RC_OVERRIDE_TIME': '地面站RC覆盖(软遥控摇杆)超时秒数. -1=永不过期'
                        '(无接收机的发射箱必须为-1, 否则停发摇杆几秒后覆盖失效);'
                        ' 0=禁用RC覆盖; >0=超时后自动回退接收机',
    'RC_OPTIONS': 'RC选项位. bit1(值2)=忽略地面站发来的RC覆盖 → 软遥控摇杆'
                  '完全无效; 本组写0=不忽略, 使屏幕摇杆/软遥控生效',
    'DISARM_DELAY': '解锁后一直未起飞自动上锁的秒数. 60=解锁后60秒未起飞'
                    '自动DISARM(防止测试解锁后遗忘); 0=不自动上锁',
}


def _haversine(lat1, lon1, lat2, lon2):
    """两点球面距离(米)"""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2 +
         math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _fnum(sv, default=0.0):
    """从 StringVar 安全取浮点"""
    try:
        return float(sv.get())
    except Exception:
        return default


# ============ 地图底图源 (默认 Esri 卫星影像: WGS-84 免密钥) ============
TILE_HOSTS = ('webrd01.is.autonavi.com', 'webrd02.is.autonavi.com',
              'webrd03.is.autonavi.com', 'webrd04.is.autonavi.com')
TILE_DIR = 'map_tiles'          # 瓦片磁盘缓存(用过一次后离线也能显示)
TILE_RETRY_SEC = 60.0           # 下载失败后冷却重试时间
MAP_MIN_ZOOM = 3
MAP_MAX_ZOOM = 17
DEFAULT_MAP_CENTER = (39.9042, 116.4074)   # 北京天安门 (WGS-84)
MAP_IMG_CACHE = 300             # 内存中保留的瓦片数
TIANDITU_KEY_FILE = 'tianditu_key.txt'     # 天地图密钥持久化
TIANDITU_HELP = '注册免费密钥: console.tianditu.gov.cn/api/key'

MAP_SOURCES = {
    'esri': {
        'label': 'Esri卫星影像 · WGS-84 · 免密钥',
        'gcj': False, 'key': False, 'ext': 'jpg',
        'url': ('https://server.arcgisonline.com/ArcGIS/rest/services/'
                'World_Imagery/MapServer/tile/{z}/{y}/{x}'),
    },
    'tdt_vec': {
        'label': '天地图矢量 · WGS-84 · 需密钥',
        'gcj': False, 'key': True, 'ext': 'png',
        'url': ('https://t{s}.tianditu.gov.cn/vec_w/wmts?SERVICE=WMTS'
                '&REQUEST=GetTile&VERSION=1.0.0&LAYER=vec&STYLE=default'
                '&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}'
                '&TILEROW={y}&TILECOL={x}&tk={tk}'),
    },
    'tdt_img': {
        'label': '天地图影像 · WGS-84 · 需密钥',
        'gcj': False, 'key': True, 'ext': 'png',
        'url': ('https://t{s}.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS'
                '&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default'
                '&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}'
                '&TILEROW={y}&TILECOL={x}&tk={tk}'),
    },
    'gaode': {
        'label': '高德道路 · GCJ-02有偏移 · 免密钥',
        'gcj': True, 'key': False, 'ext': 'png',
        'url': ('https://{host}/appmaptile?lang=zh_cn&size=1&scale=1'
                '&style=8&x={x}&y={y}&z={z}'),
    },
}

# --- WGS-84 -> GCJ-02 (火星坐标, 高德底图用) ---
_GCJ_A = 6378245.0
_GCJ_EE = 0.00669342162296594323


def _in_china(lat, lon):
    return 72.004 <= lon <= 137.8347 and 0.8293 <= lat <= 55.8271


def _gcj_tlat(x, y):
    ret = (-100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y +
           0.1 * x * y + 0.2 * math.sqrt(abs(x)))
    ret += (20.0 * math.sin(6.0 * x * math.pi) +
            20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) +
            40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) +
            320.0 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def _gcj_tlon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + \
        0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) +
            20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) +
            40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) +
            300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lat, lon):
    """GPS(WGS-84) 坐标转高德底图坐标(GCJ-02); 中国境外原样返回"""
    if lat is None or lon is None or not _in_china(lat, lon):
        return lat, lon
    dlat = _gcj_tlat(lon - 105.0, lat - 35.0)
    dlon = _gcj_tlon(lon - 105.0, lat - 35.0)
    rlat = math.radians(lat)
    magic = 1.0 - _GCJ_EE * math.sin(rlat) ** 2
    sqrtm = math.sqrt(magic)
    dlat = dlat * 180.0 / ((_GCJ_A * (1 - _GCJ_EE)) /
                           (magic * sqrtm) * math.pi)
    dlon = dlon * 180.0 / (_GCJ_A / sqrtm * math.cos(rlat) * math.pi)
    return lat + dlat, lon + dlon


# --- WebMercator 瓦片数学 ---
def _latlon_to_tilef(lat, lon, z):
    """经纬度 -> 浮点瓦片坐标 (可带小数)"""
    n = 1 << z
    x = (lon + 180.0) / 360.0 * n
    lat = max(min(lat, 85.05112878), -85.05112878)
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def _tilef_to_latlon(fx, fy, z):
    """浮点瓦片坐标 -> 经纬度"""
    n = 1 << z
    lon = fx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * fy / n))))
    return lat, lon


class TileStore(object):
    """瓦片下载: 后台单线程 + 磁盘缓存 + 失败冷却; 按底图源分目录"""

    def __init__(self, src_key, token=''):
        self.src = src_key
        self.cfg = MAP_SOURCES[src_key]
        self.token = token
        self.root = os.path.join(TILE_DIR, src_key)
        self.lock = threading.Lock()
        self.pending = set()
        self.failed = {}          # key -> 上次失败时间
        self.q = queue.Queue()
        self.new_ok = threading.Event()   # 新瓦片到货标志
        self.stop_ev = threading.Event()
        self.stats = {'ok': 0, 'fail': 0}
        threading.Thread(target=self._loop, daemon=True).start()

    @property
    def need_key(self):
        return bool(self.cfg.get('key')) and not self.token

    def _path(self, z, x, y):
        return os.path.join(self.root, str(z), str(x),
                            '%d.%s' % (y, self.cfg['ext']))

    def _url(self, z, x, y):
        return self.cfg['url'].format(
            s=(x + y) % 8,
            host=TILE_HOSTS[(x + y) % len(TILE_HOSTS)],
            z=z, x=x, y=y, tk=self.token or '')

    def request(self, z, x, y):
        """请求瓦片; 磁盘已有则直接返回路径, 否则入队下载返回 None"""
        if self.need_key:
            return None
        path = self._path(z, x, y)
        if os.path.isfile(path):
            return path
        key = (z, x, y)
        with self.lock:
            if key in self.pending:
                return None
            t_fail = self.failed.get(key)
            if t_fail and time.time() - t_fail < TILE_RETRY_SEC:
                return None
            self.pending.add(key)
        self.q.put(key)
        return None

    def _loop(self):
        magic = (b'\x89PNG\r\n\x1a\n' if self.cfg['ext'] == 'png'
                 else b'\xff\xd8\xff')
        while not self.stop_ev.is_set():
            try:
                z, x, y = self.q.get(timeout=0.3)
            except queue.Empty:
                continue
            path = self._path(z, x, y)
            ok = False
            try:
                req = urllib.request.Request(
                    self._url(z, x, y),
                    headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; '
                                           'Win64; x64)'})
                data = urllib.request.urlopen(req, timeout=8).read()
                if data[:len(magic)] == magic:
                    d = os.path.dirname(path)
                    if not os.path.isdir(d):
                        os.makedirs(d)
                    with open(path, 'wb') as f:
                        f.write(data)
                    ok = True
            except Exception:
                ok = False
            with self.lock:
                self.pending.discard((z, x, y))
                if ok:
                    self.stats['ok'] += 1
                else:
                    self.stats['fail'] += 1
                    self.failed[(z, x, y)] = time.time()
            if ok:
                self.new_ok.set()

    @staticmethod
    def load_photo(path, ext):
        """磁盘瓦片 -> tk.PhotoImage; jpg 经 Pillow 转 PNG. 失败返回 None"""
        try:
            with open(path, 'rb') as fp:
                raw = fp.read()
        except OSError:
            return None
        if ext == 'png' or raw[:8] == b'\x89PNG\r\n\x1a\n':
            data = raw
        elif HAVE_PIL:
            try:
                buf = _io.BytesIO()
                _PILImage.open(_io.BytesIO(raw)).save(buf, 'PNG')
                data = buf.getvalue()
            except Exception:
                return None
        else:
            return None
        try:
            return tk.PhotoImage(data=data)
        except Exception:
            return None


# ============================================================
#  日志重定向 (任意线程 -> 队列 -> GUI)
# ============================================================

class TeeWriter:
    """把 print 输出截获到日志队列, 同时保留原 stdout"""

    def __init__(self, log_q, orig):
        self._q = log_q
        self._orig = orig
        self._buf = ''

    def write(self, s):
        if not s:
            return 0
        try:
            self._orig.write(s)
        except Exception:
            pass
        self._buf += s
        while '\n' in self._buf or '\r' in self._buf:
            i_n = self._buf.find('\n')
            i_r = self._buf.find('\r')
            idx = min(x for x in (i_n, i_r) if x >= 0)
            line = self._buf[:idx]
            self._buf = self._buf[idx + 1:]
            if line.strip():
                self._q.put(line)
        return len(s)

    def flush(self):
        try:
            self._orig.flush()
        except Exception:
            pass
        if self._buf.strip():
            self._q.put(self._buf)
            self._buf = ''


# ============================================================
#  MAVLink 包装: 单读线程 + 指令监听 + 测试用 recv_match 队列
# ============================================================

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
                stats['last'] = name
                stats['last_t'] = time.time()
                if name == 'rc_channels_override_send' and len(args) >= 10:
                    chans = tuple(int(c) for c in args[2:10]
                                  if isinstance(c, (int, float)))
                    # v1.23: 全0=已释放, 中立帧=收尾保持(不算杆值激活),
                    # 否则中立收尾后状态栏会永久显示"覆盖中"
                    zero = all(c in (0, 65535) for c in chans)
                    neutral = tuple(int(c) for c in args[2:6]) in (
                        tct.RC_NEUTRAL_GROUND, tct.RC_NEUTRAL_AIR)
                    stats['rc_override'] = bool(chans) and not zero \
                        and not neutral
                elif name in ('heartbeat_send',):
                    pass
            return attr(*args, **kwargs)

        return wrapper


class MasterProxy:
    """recv_match 改从捕获队列读, 其余属性透传给真实 master.
    队列由唯一读线程灌入, 保证测试函数与遥测显示互不抢消息.
    新增 motors_armed() 读取共享遥测, 避免与读线程抢 recv_match 导致死锁."""

    def __init__(self, raw, msg_q, capture_evt, ui=None):
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(self, '_q', msg_q)
        object.__setattr__(self, '_cap', capture_evt)
        object.__setattr__(self, '_ui', ui)

    def motors_armed(self):
        """从共享遥测读取解锁状态, 不调用 raw.motors_armed() (会阻塞等待心跳)."""
        ui = object.__getattribute__(self, '_ui')
        if ui is not None:
            with ui.tel_lock:
                return bool(ui.tel.get('armed'))
        # 兜底: 调用 raw 方法 (可能阻塞)
        return object.__getattribute__(self, '_raw').motors_armed()

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


def load_last_port():
    """读取上次使用的串口号"""
    try:
        with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        return cfg.get(LAST_PORT_KEY, '')
    except Exception:
        return ''


def save_last_port(port):
    """保存上次使用的串口号"""
    try:
        cfg = {}
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        cfg[LAST_PORT_KEY] = port
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception:
        pass


def get_available_ports():
    """获取可用串口列表"""
    if not HAVE_SERIAL:
        return []
    ports = []
    for p in list_ports.comports():
        desc = f"{p.device} - {p.description}"
        if p.hwid:
            desc += f" ({p.hwid[:30]})"
        ports.append((p.device, desc))
    return ports


# ============================================================
#  主界面
# ============================================================

class AmpUI(object):
    def __init__(self, root):
        self.root = root
        root.title('APM 功能测试 UI  (apm_control_test 图形版)')
        root.geometry('1360x980')
        root.minsize(1100, 700)

        # ---- 线程间状态 ----
        self.log_q = queue.Queue()
        self.msg_q = queue.Queue(maxsize=2000)
        self.capture = threading.Event()
        self.action_q = collections.deque()
        self.action_lock = threading.Lock()
        self.action_busy = threading.Event()

        self.raw = None
        self.proxy = None
        self.reader_thread = None
        self.worker_thread = None
        self.reader_running = False
        self.connected = False
        # v1.30: 独立 EKF 监控线程 (检测 + 结论推给安全条)
        self.ekf_mon_thread = None
        self.ekf_mon_running = False
        self.ekf_verdict = {'text': 'EKF: --', 'style': 'Gray.TLabel',
                            'ready': False, 'why': '--'}
        self._ekf_mon_state = None      # 上一轮判定 (变化才打日志)
        self._ekf_mon_log_t = 0.0       # 判定日志节流

        self.cmd_stats = {'lock': threading.Lock(), 'counts': {},
                          'last': '--', 'last_t': 0.0,
                          'rc_override': False}
        self._acc_counts = {}
        self._rate_prev_t = time.time()
        self.task_name = '空闲'

        # 遥测 (读线程写, GUI 读)
        # 用 RLock: _update_tel 等可能嵌套获取, 普通 Lock 会自锁死
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
            'ekf': None, 'ekf_flags': None, 'status_text': '--', 'msgs': 0,
            # v1.30: 最近一次收到 GPS/EKF 状态的时间 (监控线程判数据是否陈旧)
            'ekf_t': 0.0, 'gps_t': 0.0,
            'link_ok': False,
        }
        self.alt_hist = collections.deque(maxlen=ALT_HISTORY)

        # 目标跟踪曲线 / 地图航迹
        self.curve_alt = collections.deque(maxlen=CURVE_HISTORY)
        self.curve_spd = collections.deque(maxlen=CURVE_HISTORY)
        self.track = collections.deque(maxlen=TRACK_HISTORY)
        self.target_alt = None
        self.target_speed = None
        self._track_last_t = 0.0
        self._track_last_pos = None

        # HOME / 安全保护
        self.home = None
        self._dist_home = None
        self.fence_done = False
        self.bat_done = False
        self.hb_lost = False
        self._gps_lost_t = 0.0          # v1.21: GPS失联提示节流时间戳
        self._alarm_last = {}
        # v1.22: 重试解锁人工确认 (工作线程等待, 主线程弹模态框)
        self.arm_confirm_evt = threading.Event()
        self.arm_confirm_result = False
        self.arm_confirm_gen = 0
        self.arm_confirm_auto = False   # True=跳过人工确认(冒烟/自动化)
        self._arm_dlg = None
        tct.ARM_CONFIRM_HOOK = self._arm_confirm
        # v1.32: [写入发射箱参数] 说明确认页 (弹窗 + 回读当前值上下文)
        self._launch_dlg = None
        self._launch_dlg_ctx = None

        # 抛投
        self._drop_after_id = None

        # CSV 录制
        self.rec_fp = None
        self.rec_wr = None
        self.rec_path = None
        self.rec_last = 0.0
        self.recording = False

        # 测试报告
        self.last_results = collections.OrderedDict()

        # 遥测消息速率 (读线程写, GUI 读)
        self.msg_lock = threading.Lock()
        self.msg_acc = {}
        self.msg_rates = {}
        self._msg_rate_t = time.time()
        self._msg_tree_t = 0.0

        # RC 覆盖滑条 / 软遥控 (v1.23)
        self.rc_scale = []
        self._rc_last_send = 0.0
        self._rc_was_on = False
        self._rc_gen = 0              # 启用/停用代际, 作废未完成的参数准备
        self._rc_grace_t = 0.0        # 启用后等模式切到手动的宽限截止时刻
        self._soft_rc_job = None      # 20Hz 发送循环 root.after id
        self.var_kb_on = tk.BooleanVar(value=False)   # 键盘接管开关
        self._key_state = set()       # 当前按住的键 (虚拟摇杆状态由滑条承载)
        self._stick = {'lx': 0.0, 'ly': -1.0,
                       'rx': 0.0, 'ry': 0.0}          # 左=偏航/油门 右=横滚/俯仰
        self._stick_drag = None

        # 解锁状态迁移 (用于清除目标值)
        self._was_armed = False

        # stdout 重定向
        self._orig_stdout = sys.stdout
        sys.stdout = TeeWriter(self.log_q, self._orig_stdout)
        sys.stderr = TeeWriter(self.log_q, sys.stderr)

        # 主线程卡死看门狗状态 (v1.29.2): _tick 每拍自增, 看门狗检测停摆后转储堆栈
        self._tick_seq = 0
        self._hang_dump_t = 0.0

        self._build_ui()
        self._tick()
        self._start_hang_watchdog()
        root.protocol('WM_DELETE_WINDOW', self._on_close)
        root.bind('<F9>', lambda e: self.on_panic_loiter())
        # v1.23 键盘接管 (开关在软遥控页; 焦点于输入框时自动忽略)
        root.bind('<KeyPress>', self.on_soft_rc_key_press)
        root.bind('<KeyRelease>', self.on_soft_rc_key_release)

    def _start_hang_watchdog(self):
        """后台看门狗: 主线程 _tick 停摆 >3s 时, 把所有线程堆栈写入
        ui_hang_dump.txt, 用于定位界面卡死点 (GIL 被顶住 vs 主线程阻塞)."""
        def watch():
            last = self._tick_seq
            stall_from = None
            while True:
                time.sleep(1.0)
                if self._tick_seq != last:
                    last = self._tick_seq
                    stall_from = None
                    continue
                if stall_from is None:
                    stall_from = time.time()
                    continue
                # 未连接时不判定(空闲 tick 停摆无意义)
                if not self.connected:
                    stall_from = time.time()
                    continue
                if time.time() - stall_from < 3.0:
                    continue
                stall_from = time.time()
                if time.time() - self._hang_dump_t < 5.0:
                    continue
                self._hang_dump_t = time.time()
                self._write_hang_dump()
        threading.Thread(target=watch, daemon=True).start()

    def _write_hang_dump(self):
        try:
            frames = sys._current_frames()
            buf = ['==== HANG DUMP %s (主线程 _tick 停摆 >3s) ====' %
                   time.strftime('%Y-%m-%d %H:%M:%S')]
            for tid, frame in frames.items():
                th = None
                for t in threading.enumerate():
                    if t.ident == tid:
                        th = t
                        break
                name = th.name if th else '?'
                buf.append('\n--- thread %s (ident=%s, daemon=%s) ---' % (
                    name, tid, getattr(th, 'daemon', '?')))
                buf.append(''.join(traceback.format_stack(frame)))
            text = '\n'.join(buf)
        except Exception as e:
            text = 'dump failed: %s' % e
        path = os.path.join(os.getcwd(), 'ui_hang_dump.txt')
        try:
            with open(path, 'a', encoding='utf-8') as f:
                f.write(text + '\n')
        except Exception:
            pass
        # 同时转成日志行 (取主线程那段的前几行, 便于直接看)
        try:
            self.log_q.put('🐞 界面卡死堆栈已写入 %s' % path)
        except Exception:
            pass

    # ---------------- UI 构建 ----------------

    def _build_ui(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use('vista')
        except Exception:
            try:
                style.theme_use('clam')
            except Exception:
                pass
        style.configure('Green.TLabel', foreground='#0a7a0a')
        style.configure('Red.TLabel', foreground='#b00000')
        style.configure('Gray.TLabel', foreground='#666666')
        style.configure('Big.TButton', padding=4)
        style.configure('Bad.TLabel', foreground='#d00000',
                        font=('Microsoft YaHei', 9, 'bold'))
        style.configure('Warn.TLabel', foreground='#d06a00',
                        font=('Microsoft YaHei', 9, 'bold'))
        style.configure('Ok.TLabel', foreground='#0a7a0a')

        # ===== 顶部: 连接与参数 =====
        top = ttk.Frame(self.root, padding=4)
        top.pack(fill='x')

        ttk.Label(top, text='连接:').pack(side='left', padx=(2, 2))
        self.var_conn = tk.StringVar(value=load_last_port() or DEFAULT_CONN)
        self.cmb_conn = ttk.Combobox(top, textvariable=self.var_conn, width=22)
        self.cmb_conn.pack(side='left')
        self.btn_refresh_ports = ttk.Button(top, text='刷新串口',
                                            command=self.on_refresh_ports, width=10)
        self.btn_refresh_ports.pack(side='left', padx=(2, 0))
        self.on_refresh_ports()  # 初始填充串口列表

        ttk.Label(top, text='波特率:').pack(side='left', padx=(6, 2))
        self.var_baud = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Combobox(top, textvariable=self.var_baud, width=8,
                     values=('57600', '115200', '921600', '38400')
                     ).pack(side='left')
        self.btn_conn = ttk.Button(top, text='连接', command=self.on_connect)
        self.btn_conn.pack(side='left', padx=6)
        self.btn_disc = ttk.Button(top, text='断开', command=self.on_disconnect,
                                   state='disabled')
        self.btn_disc.pack(side='left')
        # v1.24.4: 飞控重启 (维护类, 需已连接+已上锁+二次确认)
        self.btn_reboot = ttk.Button(top, text='重启飞控',
                                     command=self.on_reboot_fc,
                                     state='disabled')
        self.btn_reboot.pack(side='left', padx=(6, 0))

        self.var_link = tk.StringVar(value='未连接')
        ttk.Label(top, textvariable=self.var_link,
                  style='Red.TLabel').pack(side='left', padx=14)

        ttk.Separator(top, orient='vertical').pack(side='left', fill='y',
                                                   padx=6, pady=2)
        ttk.Label(top, text='起飞高度m:').pack(side='left')
        self.var_toff = tk.StringVar(value=str(tct.TAKEOFF_ALT))
        ttk.Entry(top, textvariable=self.var_toff, width=5).pack(side='left')
        # v1.31: 爬升杆位不再由用户填写(闭环自适应, 见 tct.takeoff_althold)
        ttk.Label(top, text='低空起降m:').pack(side='left', padx=(6, 0))
        self.var_bounce = tk.StringVar(value=str(tct.BOUNCE_ALT))
        ttk.Entry(top, textvariable=self.var_bounce, width=5).pack(side='left')
        ttk.Label(top, text='测试速度:').pack(side='left', padx=(6, 0))
        self.var_speed = tk.StringVar(value=str(tct.TEST_SPEED))
        ttk.Entry(top, textvariable=self.var_speed, width=5).pack(side='left')
        ttk.Label(top, text='时长s:').pack(side='left', padx=(6, 0))
        self.var_dur = tk.StringVar(value=str(tct.TEST_DURATION))
        ttk.Entry(top, textvariable=self.var_dur, width=4).pack(side='left')
        # v1.30: 参数写入唯一入口 — 系统起飞/解锁不再自动写任何飞控参数
        # v1.32: 点击先弹[参数说明+确认]页, 页底确认后才真正写入
        ttk.Button(top, text='写入发射箱参数…',
                   command=self.on_write_launch_params).pack(side='left', padx=(8, 0))
        ttk.Label(top, text='(参数只由本按钮/参数页手动写, 系统不自动写)',
                  style='Gray.TLabel').pack(side='left', padx=(4, 0))

        # ===== 安全条: 急停 + 围栏/电池/心跳状态 =====
        bar = ttk.Frame(self.root, padding=(4, 2))
        bar.pack(fill='x')

        tk.Button(bar, text='⛔ 急停: 悬停 LOITER (F9)',
                  bg='#c00000', fg='white', activebackground='#e00000',
                  activeforeground='white',
                  font=('Microsoft YaHei', 10, 'bold'),
                  relief='raised', bd=3,
                  command=self.on_panic_loiter).pack(side='left', padx=(0, 6))
        tk.Button(bar, text='立即 RTL', bg='#b45309', fg='white',
                  font=('Microsoft YaHei', 9, 'bold'), bd=2,
                  command=lambda: self.on_immediate_mode('RTL')
                  ).pack(side='left', padx=3)
        tk.Button(bar, text='立即 LAND', bg='#92400e', fg='white',
                  font=('Microsoft YaHei', 9, 'bold'), bd=2,
                  command=lambda: self.on_immediate_mode('LAND')
                  ).pack(side='left', padx=3)
        # v1.24: 全0帧撤销GCS覆盖 → 物理遥控器摇杆即刻生效
        tk.Button(bar, text='交还遥控器', bg='#1d4ed8', fg='white',
                  font=('Microsoft YaHei', 9, 'bold'), bd=2,
                  command=self.on_handover_rc).pack(side='left', padx=3)
        ttk.Separator(bar, orient='vertical').pack(side='left', fill='y',
                                                   padx=8, pady=2)

        self.var_fence_st = tk.StringVar(value='围栏: 关闭')
        self.lbl_fence = ttk.Label(bar, textvariable=self.var_fence_st,
                                   style='Gray.TLabel')
        self.lbl_fence.pack(side='left', padx=8)
        self.var_bat_st = tk.StringVar(value='电池: --')
        self.lbl_bat = ttk.Label(bar, textvariable=self.var_bat_st,
                                 style='Gray.TLabel')
        self.lbl_bat.pack(side='left', padx=8)
        self.var_hb_st = tk.StringVar(value='心跳: --')
        self.lbl_hb = ttk.Label(bar, textvariable=self.var_hb_st,
                                style='Gray.TLabel')
        self.lbl_hb.pack(side='left', padx=8)
        # v1.30: EKF 解锁判定 — 独立监控线程结论 (与解锁预检门禁同一判据)
        self.var_ekf_st = tk.StringVar(value='EKF: --')
        self.lbl_ekf_st = ttk.Label(bar, textvariable=self.var_ekf_st,
                                    style='Gray.TLabel')
        self.lbl_ekf_st.pack(side='left', padx=8)
        self.var_prot_st = tk.StringVar(value='')
        ttk.Label(bar, textvariable=self.var_prot_st,
                  style='Bad.TLabel').pack(side='left', padx=8)

        # ===== 主体: 左遥测 / 右控制 =====
        body = ttk.Frame(self.root, padding=4)
        body.pack(fill='both', expand=True)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, weight=1)
        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        # ---- 左: 实时遥测 (跨满两行, 底部区域留给姿态仪表) ----
        left = ttk.LabelFrame(body, text='实时状态 / 遥测', padding=4)
        left.grid(row=0, column=0, rowspan=2, sticky='nsw', padx=(0, 4))

        self.tel_labels = {}
        self._goto_spec = None
        self._goto_epoch = 0
        self._goto_active = threading.Event()

        def add_row(r, title):
            ttk.Label(left, text=title, style='Gray.TLabel').grid(
                row=r, column=0, sticky='w', pady=(3, 0))

        def add_pair(r, key, label, width=22):
            ttk.Label(left, text=label).grid(row=r, column=1, sticky='w',
                                             padx=6)
            v = tk.StringVar(value='--')
            ttk.Label(left, textvariable=v, width=width,
                      anchor='w').grid(row=r, column=2, sticky='w')
            self.tel_labels[key] = v

        add_row(0, '◆ 飞行状态')
        add_pair(1, 'mode', '飞行模式', 14)
        ttk.Label(left, text='解锁状态').grid(row=2, column=1, sticky='w',
                                              padx=6)
        self.var_armed = tk.StringVar(value='--')
        self.lbl_armed = ttk.Label(left, textvariable=self.var_armed,
                                   width=14, anchor='w')
        self.lbl_armed.grid(row=2, column=2, sticky='w')
        self.tel_labels['armed'] = self.var_armed
        add_pair(3, 'sys', '系统状态', 14)
        add_pair(4, 'hb', '心跳', 14)

        add_row(5, '◆ GPS / 位置')
        add_pair(6, 'gps', 'GPS', 24)
        add_pair(7, 'pos', '经纬度', 24)
        add_pair(8, 'alt', '相对高度', 14)
        add_pair(9, 'absalt', '绝对高度AMSL', 14)
        add_pair(10, 'home', 'HOME 经纬度', 24)
        add_pair(11, 'dist', '离HOME距离', 14)

        add_row(12, '◆ 速度 / 姿态')
        add_pair(13, 'vel', '速度NED m/s', 24)
        add_pair(14, 'gs', '地速/空速/油门', 24)
        add_pair(15, 'hdg', '航向/爬升', 24)
        add_pair(16, 'rpy', '姿态 R/P/Y °', 24)

        add_row(17, '◆ 电池 / 系统')
        add_pair(18, 'bat', '电池 V/A/%', 24)
        add_pair(19, 'load', 'CPU负载', 14)
        add_pair(20, 'ekf', 'EKF方差 水/速/罗', 24)
        # v1.29.3: 位置估计就绪状态 — 解锁预检门禁的可视化 (未就绪不解锁)
        ttk.Label(left, text='位置估计').grid(row=21, column=1, sticky='w',
                                              padx=6)
        self.var_posrdy = tk.StringVar(value='--')
        self.lbl_posrdy = ttk.Label(left, textvariable=self.var_posrdy,
                                    width=24, anchor='w',
                                    style='Gray.TLabel')
        self.lbl_posrdy.grid(row=21, column=2, sticky='w')
        self.tel_labels['posrdy'] = self.var_posrdy
        add_pair(22, 'st', '最近飞控消息', 24)

        add_row(23, '◆ 控制信号 / RC')
        add_pair(24, 'rcov', 'RC覆盖', 14)
        add_pair(25, 'rc', 'RC ch1-4', 24)
        add_pair(26, 'rssi', 'RC RSSI', 14)
        add_pair(27, 'cmd', '最近指令', 24)
        add_pair(28, 'link', '链路 msg/s', 14)

        # 高度曲线
        ttk.Label(left, text='◆ 高度曲线', style='Gray.TLabel').grid(
            row=29, column=0, columnspan=3, sticky='w', pady=(4, 0))
        self.alt_canvas = tk.Canvas(left, width=270, height=70,
                                    bg='#101820', highlightthickness=1,
                                    highlightbackground='#444')
        self.alt_canvas.grid(row=30, column=0, columnspan=3, sticky='w',
                             pady=2)

        # 姿态仪表
        ttk.Label(left, text='◆ 姿态仪表 (地平仪)', style='Gray.TLabel'
                  ).grid(row=31, column=0, columnspan=3, sticky='w',
                         pady=(4, 0))
        self.att_canvas = tk.Canvas(left, width=270, height=120,
                                    bg='#000', highlightthickness=1,
                                    highlightbackground='#444')
        self.att_canvas.grid(row=32, column=0, columnspan=3, sticky='w',
                             pady=2)

        # ---- 中: 快速控制 + 控制信号 ----
        mid = ttk.Frame(body)
        mid.grid(row=0, column=1, sticky='nsew', padx=(0, 4))
        mid.columnconfigure(0, weight=1)
        mid.rowconfigure(1, weight=1)

        qc = ttk.LabelFrame(mid, text='快速控制 (测试运行中自动禁用)', padding=6)
        qc.grid(row=0, column=0, sticky='ew')
        qc.columnconfigure(5, weight=1)

        self.btn_vars = []
        b_arm = ttk.Button(qc, text='解锁 ARM', command=self.on_arm)
        b_arm.grid(row=0, column=0, padx=3, pady=3, sticky='ew')
        b_dis = ttk.Button(qc, text='锁定 DISARM', command=self.on_disarm)
        b_dis.grid(row=0, column=1, padx=3, pady=3, sticky='ew')
        # v1.24: 定高(ALT_HOLD)解锁/上锁一键切换 (不依赖水平位置估计, 室内试用)
        b_alt = ttk.Button(qc, text='定高解锁/上锁',
                           command=self.on_arm_alt_hold)
        b_alt.grid(row=0, column=2, padx=3, pady=3, sticky='ew')
        b_to = ttk.Button(qc, text='起飞(按高度)', command=self.on_takeoff)
        b_to.grid(row=0, column=3, padx=3, pady=3, sticky='ew')
        b_land = ttk.Button(qc, text='降落 LAND', command=self.on_land)
        b_land.grid(row=0, column=4, padx=3, pady=3, sticky='ew')
        b_rtl = ttk.Button(qc, text='返航 RTL', command=self.on_rtl)
        b_rtl.grid(row=0, column=5, padx=3, pady=3, sticky='ew')

        ttk.Label(qc, text='模式:').grid(row=1, column=0, sticky='e', padx=3)
        self.var_mode = tk.StringVar(value='GUIDED')
        self.cmb_mode = ttk.Combobox(qc, textvariable=self.var_mode, width=12,
                                      values=('GUIDED', 'LOITER', 'ALT_HOLD',
                                              'STABILIZE', 'RTL', 'LAND',
                                              'POSHOLD', 'AUTOTUNE', 'AUTO'),
                                      state='disabled')
        self.cmb_mode.grid(row=1, column=1, sticky='w', padx=3)
        ttk.Button(qc, text='切换模式',
                   command=self.on_set_mode).grid(row=1, column=2, padx=3)
        ttk.Label(qc, text='(RTL/LAND 不可解锁)',
                  style='Gray.TLabel').grid(
            row=1, column=3, columnspan=2, sticky='w')

        ttk.Label(qc, text='速度微动 m/s:').grid(row=2, column=0,
                                                 sticky='e', padx=3, pady=3)
        self.var_vspd = tk.StringVar(value='1.0')
        ttk.Entry(qc, textvariable=self.var_vspd, width=5).grid(
            row=2, column=1, sticky='w', padx=3)
        vbtns = ttk.Frame(qc)
        vbtns.grid(row=2, column=2, columnspan=3, sticky='w')
        for txt, vx, vy, vz in (
                ('↑前', 1, 0, 0), ('↓后', -1, 0, 0), ('←左', 0, -1, 0),
                ('→右', 0, 1, 0), ('↑升', 0, 0, -1), ('↓降', 0, 0, 1),
                ('停止', 0, 0, 0)):
            ttk.Button(vbtns, text=txt, width=4,
                       command=lambda a=(vx, vy, vz): self.on_velocity(*a)
                       ).pack(side='left', padx=2)

        # 抛投: 通道/触发/回位/保持 (前提 SERVO<通道>_FUNCTION=0)
        drop_row = ttk.Frame(qc)
        drop_row.grid(row=3, column=0, columnspan=5, sticky='w',
                      padx=3, pady=(4, 0))
        ttk.Label(drop_row, text='抛投 通道:').pack(side='left')
        self.var_drop_no = tk.StringVar(value='10')
        ttk.Entry(drop_row, textvariable=self.var_drop_no, width=4).pack(
            side='left', padx=(2, 6))
        ttk.Label(drop_row, text='触发µs:').pack(side='left')
        self.var_drop_on = tk.StringVar(value='2000')
        ttk.Entry(drop_row, textvariable=self.var_drop_on, width=5).pack(
            side='left', padx=(2, 6))
        ttk.Label(drop_row, text='回位µs:').pack(side='left')
        self.var_drop_off = tk.StringVar(value='1000')
        ttk.Entry(drop_row, textvariable=self.var_drop_off, width=5).pack(
            side='left', padx=(2, 6))
        ttk.Label(drop_row, text='保持ms:').pack(side='left')
        self.var_drop_hold = tk.StringVar(value='1500')
        ttk.Entry(drop_row, textvariable=self.var_drop_hold, width=6).pack(
            side='left', padx=(2, 6))
        ttk.Button(drop_row, text='⚠ 抛投', command=self.on_drop).pack(
            side='left', padx=(4, 0))

        # 功能页: 安全设置 / 目标曲线 / 小地图 / 参数 / 航点 / RC / 信号
        self.nb = ttk.Notebook(mid)
        self.nb.grid(row=1, column=0, sticky='nsew', pady=(4, 0))
        self._build_tab_safety()
        self._build_tab_curves()
        self._build_tab_map()
        self._build_tab_param()
        self._build_tab_guided()
        self._build_tab_mission()
        self._build_tab_rc()
        self._build_tab_signals()

        # ---- 右: 测试项 (跨满两行) ----
        right = ttk.LabelFrame(body, text='功能测试 (同 apm_control_test)',
                               padding=6)
        right.grid(row=0, column=2, rowspan=2, sticky='nsw', padx=(4, 0))

        self.test_buttons = {}
        r = 0
        for tid, (name, _) in tct.TESTS.items():
            b = ttk.Button(right, text='%s. %s' % (tid, name), width=18,
                           style='Big.TButton', state='disabled',
                           command=lambda i=tid: self.on_test(i))
            b.grid(row=r // 2, column=r % 2, padx=2, pady=2, sticky='ew')
            self.test_buttons[tid] = b
            r += 1
        self.btn_all = ttk.Button(right, text='a. 运行全部测试',
                                  state='disabled',
                                  command=self.on_test_all)
        self.btn_all.grid(row=r // 2 + 1, column=0, columnspan=2,
                          padx=2, pady=(8, 2), sticky='ew')
        self.btn_stop = ttk.Button(right, text='停止(取消后续队列)',
                                   command=self.on_stop)
        self.btn_stop.grid(row=r // 2 + 2, column=0, columnspan=2,
                           padx=2, pady=2, sticky='ew')
        self.btn_clear = ttk.Button(right, text='清空日志',
                                    command=self.on_clear_log)
        self.btn_clear.grid(row=r // 2 + 3, column=0, columnspan=2,
                            padx=2, pady=2, sticky='ew')
        self.btn_rec = ttk.Button(right, text='⏺ 录制飞行日志(CSV)',
                                  command=self.on_toggle_record)
        self.btn_rec.grid(row=r // 2 + 4, column=0, columnspan=2,
                          padx=2, pady=2, sticky='ew')
        self.btn_log_last = ttk.Button(
            right, text='⬇ 下载最后飞行日志',
            command=lambda: self.on_log_fetch(True))
        self.btn_log_last.grid(row=r // 2 + 5, column=0, columnspan=2,
                               padx=2, pady=2, sticky='ew')
        self.btn_log_all = ttk.Button(
            right, text='⬇ 下载全部飞控日志',
            command=lambda: self.on_log_fetch(False))
        self.btn_log_all.grid(row=r // 2 + 6, column=0, columnspan=2,
                              padx=2, pady=2, sticky='ew')
        self.btn_report = ttk.Button(right, text='导出测试报告',
                                     command=self.on_export_report)
        self.btn_report.grid(row=r // 2 + 7, column=0, columnspan=2,
                             padx=2, pady=2, sticky='ew')

        self.var_task = tk.StringVar(value='空闲')
        ttk.Label(right, textvariable=self.var_task, wraplength=170,
                  justify='left').grid(row=r // 2 + 8, column=0,
                                       columnspan=2, pady=(8, 0))

        # ---- 中列底部: 日志 (左右两栏通到底, 日志居中在下) ----
        bottom = ttk.LabelFrame(body, text='日志 / 飞控反馈', padding=2)
        bottom.grid(row=1, column=1, sticky='nsew', pady=(4, 0),
                    padx=(0, 4))
        body.rowconfigure(1, weight=1)
        bottom.rowconfigure(0, weight=1)
        bottom.columnconfigure(0, weight=1)

        # ---- 右列底部: 震动监测 (Mission Planner Vibration 类似) ----
        vib_frame = ttk.LabelFrame(body, text='震动监测 (Vibration)', padding=4)
        vib_frame.grid(row=1, column=2, sticky='nsew', pady=(4, 0),
                       padx=(0, 4))
        vib_frame.rowconfigure(0, weight=1)
        vib_frame.columnconfigure(0, weight=1)

        # 震动数据存储: 每轴保留最近 300 个点 (约 5 分钟 @ 1Hz)
        self.vib_data = {'x': [], 'y': [], 'z': []}
        self.vib_max_points = 300

        # 画布
        self.vib_canvas = tk.Canvas(vib_frame, bg='#0d1117',
                                     highlightthickness=1,
                                     highlightbackground='#30363d')
        self.vib_canvas.grid(row=0, column=0, sticky='nsew')
        self.vib_canvas.bind('<Configure>', self._on_vib_resize)

        # 当前值标签
        vib_info = ttk.Frame(vib_frame)
        vib_info.grid(row=1, column=0, sticky='ew', pady=(4, 0))
        self.vib_labels = {}
        for i, (axis, color) in enumerate([('X', '#ff6b6b'), ('Y', '#7bd87b'), ('Z', '#8ecae6')]):
            f = ttk.Frame(vib_info)
            f.pack(side='left', expand=True, fill='x', padx=4)
            ttk.Label(f, text=f'{axis}:', style='Gray.TLabel').pack(side='left')
            v = tk.StringVar(value='0.0')
            lbl = ttk.Label(f, textvariable=v, foreground=color,
                            font=('Consolas', 10, 'bold'))
            lbl.pack(side='left', padx=(4, 0))
            self.vib_labels[axis.lower()] = v

        # 心跳计数器用于 1Hz 采样
        self._vib_tick = 0

        self.txt_log = tk.Text(bottom, height=14, wrap='none',
                               font=('Consolas', 9), state='disabled',
                               bg='#111', fg='#d8d8d8',
                               insertbackground='#fff')
        self.txt_log.tag_configure('err', foreground='#ff6b6b')
        self.txt_log.tag_configure('ok', foreground='#7bd87b')
        self.txt_log.tag_configure('warn', foreground='#ffd166')
        self.txt_log.tag_configure('fc', foreground='#8ecae6')
        sb1 = ttk.Scrollbar(bottom, orient='vertical',
                            command=self.txt_log.yview)
        sb2 = ttk.Scrollbar(bottom, orient='horizontal',
                            command=self.txt_log.xview)
        self.txt_log.configure(yscrollcommand=sb1.set,
                               xscrollcommand=sb2.set)
        self.txt_log.grid(row=0, column=0, sticky='nsew')
        sb1.grid(row=0, column=1, sticky='ns')
        sb2.grid(row=1, column=0, sticky='ew')

    # ---------------- 日志 ----------------

    def _log_line(self, text):
        # 任意线程可调用: 非主线程转入队列, 由 _tick 统一渲染
        if threading.current_thread() is not threading.main_thread():
            self.log_q.put(text)
            return
        t = time.strftime('%H:%M:%S')
        tag = None
        up = text
        if '[飞控]' in text or '[调参]' in text:
            tag = 'fc'
        elif '❌' in text or '错误' in text or '失败' in text:
            tag = 'err'
        elif '✅' in text or '通过' in text:
            tag = 'ok'
        elif '⚠️' in text or '警告' in text:
            tag = 'warn'
        self.txt_log.configure(state='normal')
        self.txt_log.insert('end', '[%s] %s\n' % (t, text), tag)
        lines = int(self.txt_log.index('end-1c').split('.')[0])
        if lines > LOG_MAX_LINES:
            self.txt_log.delete('1.0', '%d.0' % (lines - LOG_MAX_LINES))
        self.txt_log.see('end')
        self.txt_log.configure(state='disabled')

    def on_clear_log(self):
        self.txt_log.configure(state='normal')
        self.txt_log.delete('1.0', 'end')
        self.txt_log.configure(state='disabled')

    # ---------------- 动作队列 ----------------

    def _enqueue(self, name, fn):
        if not self.connected:
            self._log_line('❌ 未连接飞控, 请先连接')
            return
        if self.action_busy.is_set():
            self._log_line('⚠️ 当前有任务运行中, 已加入队列: %s' % name)
        with self.action_lock:
            self.action_q.append((name, fn))
        self._log_line('▶ 已排队: %s' % name)

    def on_stop(self):
        with self.action_lock:
            n = len(self.action_q)
            self.action_q.clear()
        self._log_line('⏹ 已取消 %d 个待运行任务(当前任务不可中断)' % n)

    def _worker_loop(self):
        while True:
            item = None
            with self.action_lock:
                if self.action_q:
                    item = self.action_q.popleft()
            if item is None:
                # 常驻: 断开后不退出, 避免"入队连接任务但 worker 已死"的孤儿竞态
                time.sleep(0.1)
                continue
            name, fn = item
            self.action_busy.set()
            self.task_name = '运行: %s' % name
            # 开启捕获, 清空旧消息
            while True:
                try:
                    self.msg_q.get_nowait()
                except queue.Empty:
                    break
            self.capture.set()
            ok = False
            try:
                ok = bool(fn())
            except SystemExit as e:
                self._log_line('❌ %s: %s' % (name, e))
            except Exception as e:
                self._log_line('❌ %s 异常: %s: %s' % (
                    name, type(e).__name__, e))
            finally:
                self.capture.clear()
                while True:
                    try:
                        self.msg_q.get_nowait()
                    except queue.Empty:
                        break
                self.action_busy.clear()
                self.task_name = '空闲'
            self._log_line('%s: %s' % (name, '✅ 通过' if ok else '❌ 失败'))

    # ---------------- 连接 ----------------

    def _apply_params(self):
        try:
            tct.TAKEOFF_ALT = float(self.var_toff.get())
            tct.BOUNCE_ALT = float(self.var_bounce.get())
            tct.TEST_SPEED = float(self.var_speed.get())
            tct.TEST_DURATION = float(self.var_dur.get())
        except ValueError:
            self._log_line('⚠️ 参数格式错误, 保持原值')
            return False
        return True

    def on_refresh_ports(self):
        """刷新可用串口列表"""
        ports = get_available_ports()
        if ports:
            self.cmb_conn['values'] = [desc for _, desc in ports]
            # 保持当前选择，如果不在列表中则选第一个
            current = self.var_conn.get()
            if current not in [d for _, d in ports]:
                self.var_conn.set(ports[0][1])
        else:
            self.cmb_conn['values'] = ['(无可用串口)']
            self.var_conn.set('(无可用串口)')
        # 日志组件可能尚未创建，安全检查
        if hasattr(self, 'txt_log') and self.txt_log:
            self._log_line('🔍 串口列表已刷新，共 %d 个' % len(ports))

    def on_connect(self):
        if self.connected:
            return
        desc = self.var_conn.get().strip()
        # 从 "COM22 - 设备描述" 中提取设备名 COM22
        if ' - ' in desc:
            conn = desc.split(' - ')[0].strip()
        else:
            conn = desc
        try:
            baud = int(self.var_baud.get().strip())
        except ValueError:
            self._log_line('❌ 波特率无效')
            return
        self.btn_conn.configure(state='disabled')
        self.var_link.set('连接中...')
        self.task_name = '连接中...'

        def do():
            self._log_line('🔌 正在连接 %s @ %d ...' % (conn, baud))
            # 串口连接先做可用性预检(端口被占用/不存在/驱动问题会立即报错),
            # 网络连接(tcp:/udp:)跳过预检
            low = conn.lower()
            is_serial = not any(low.startswith(p) for p in (
                'tcp:', 'udp:', 'udpin:', 'udpout:', 'udpcast:', 'mcast:',
                'file:', 'serial:'))
            if is_serial:
                import serial
                try:
                    self._log_line('  🔧 尝试打开串口...')
                    ser = serial.Serial(conn, baud, timeout=0.5,
                                        write_timeout=0.5)
                    self._log_line('  ✅ 串口打开成功')
                    ser.close()
                except serial.SerialException as e:
                    self._log_line('❌ 串口打开失败: %s '
                                   '(端口被占用/不存在/驱动问题)' % e)
                    return False
                except Exception as e:
                    self._log_line('❌ 串口异常: %s: %s' % (type(e).__name__, e))
                    return False

            raw = None
            ok = False
            try:
                self._log_line('  🔧 建立 MAVLink 连接...')
                raw = mavutil.mavlink_connection(conn, baud=baud)
                # 非阻塞轮询等待心跳(避免 Windows 串口 select 忙等顶死主线程),
                # 每 3 秒打印一次进度
                hb = None
                t0 = time.time()
                last_log = 0
                while time.time() - t0 < 30:
                    hb = raw.recv_match(type='HEARTBEAT', blocking=False)
                    if hb is not None:
                        break
                    elapsed = int(time.time() - t0)
                    if elapsed >= last_log + 3:
                        last_log = elapsed
                        self._log_line('  ⏳ 等待心跳中... (%ds/30s)' % elapsed)
                    time.sleep(0.02)
                if hb is None:
                    self._log_line('❌ 未收到心跳, 请检查串口/波特率/飞控供电/接线')
                    return False
                raw.target_system = hb.get_srcSystem()
                raw.target_component = hb.get_srcComponent()
                try:
                    raw.vehicle_type = 'copter'
                except AttributeError:
                    pass
                # 等旧读线程退出(断开时 reader_running 已置 False), 防双 reader 抢消息
                old = self.reader_thread
                if old is not None and old.is_alive() \
                        and old is not threading.current_thread():
                    old.join(timeout=1.5)
                self.raw = raw
                self.proxy = MasterProxy(raw, self.msg_q, self.capture,
                                         ui=self)
                # 包装 send 以统计指令(测试经 proxy.mav 访问)
                raw.mav = MavSendProxy(raw.mav, self.cmd_stats)
                # v1.29.3: 请求必要数据流, 保证解锁预检门禁能拿到 GPS/EKF
                # 判据 (依赖飞控默认速率时可能收不到 EKF_STATUS_REPORT)
                self._request_streams(raw)
                self.reader_running = True
                self.reader_thread = threading.Thread(
                    target=self._reader_loop, daemon=True)
                self.reader_thread.start()
                self._log_line('✅ 已连接 sysid=%s compid=%s' % (
                    raw.target_system, raw.target_component))
                ok = True
                return True
            finally:
                if not ok:
                    # 连接失败: 必须释放串口, 否则重连/Mission Planner 均打不开
                    try:
                        raw.close()
                    except Exception:
                        pass
                    self.raw = None
                    self.proxy = None
                    self.reader_running = False

        def wrap():
            ok = False
            try:
                ok = do()
            except Exception as e:
                self._log_line('❌ 连接异常: %s: %s' % (type(e).__name__, e))
                ok = False
            finally:
                self.root.after(0, lambda: self._after_connect(ok))
            return ok

        with self.action_lock:
            self.action_q.append(('连接', wrap))
        if not self.worker_thread or not self.worker_thread.is_alive():
            self.worker_thread = threading.Thread(
                target=self._worker_loop, daemon=True)
            self.worker_thread.start()

    @staticmethod
    def _request_streams(raw):
        """v1.29.3: 连接后请求必要数据流 (各 2Hz, 不加重链路) —
        EXTENDED_STATUS(GPS_RAW_INT/EKF 相关) + POSITION(位置) +
        主动要一条 EKF_STATUS_REPORT, 供解锁预检门禁判定位置估计是否就绪.
        失败静默(部分固件不支持 REQUEST_MESSAGE 属正常)"""
        if raw is None:
            return
        try:
            for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                        mavutil.mavlink.MAV_DATA_STREAM_POSITION):
                raw.mav.request_data_stream_send(
                    raw.target_system, raw.target_component, sid, 2, 1)
            raw.mav.command_long_send(
                raw.target_system, raw.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                0, 0, 0, 0, 0, 0)
        except Exception:
            pass

    def _after_connect(self, ok):
        if ok:
            self.connected = True
            self.btn_disc.configure(state='normal')
            self.btn_reboot.configure(state='normal')
            self.cmb_mode.configure(state='readonly')
            self.var_link.set('● 已连接 (实时)')
            for tid in self.test_buttons:
                self.test_buttons[tid].configure(state='normal')
            self.btn_all.configure(state='normal')
            tct.reset_launch_params_flag()   # v1.24.5: 连接时重置写入标记
            # 保存成功连接的串口
            self._save_selected_port()
            self._start_ekf_monitor()        # v1.30: EKF 解锁判定线程
        else:
            self.connected = False
            self.btn_conn.configure(state='normal')
            self.var_link.set('连接失败')

    def _save_selected_port(self):
        """从下拉框描述中提取设备名并保存"""
        desc = self.var_conn.get()
        if desc and ' - ' in desc:
            device = desc.split(' - ')[0].strip()
            if device:
                save_last_port(device)

    def on_disconnect(self):
        if self.action_busy.is_set():
            self._log_line('⚠️ 任务运行中, 请先停止并等待完成再断开')
            return
        self._cancel_pending_drop()
        self._stop_record()
        self._cancel_soft_rc_loop()
        self.var_rc_on.set(False)
        self.var_kb_on.set(False)
        self._release_rc_override()
        self.reader_running = False
        self.connected = False
        self._stop_ekf_monitor()             # v1.30: 停 EKF 监控线程
        if getattr(self, 'lbl_ekf_st', None) is not None:
            self.ekf_verdict = {'text': 'EKF: --', 'style': 'Gray.TLabel',
                                'ready': False, 'why': '--'}
            self.var_ekf_st.set('EKF: --')
            self.lbl_ekf_st.configure(style='Gray.TLabel')
        raw = self.raw
        self.raw = None
        self.proxy = None
        # 先等读线程退出(≤0.3s 一个 recv 超时), 再关串口, 避免关着读
        t = self.reader_thread
        if t is not None and t.is_alive() \
                and t is not threading.current_thread():
            t.join(timeout=1.5)
        if raw is not None:
            try:
                raw.close()
            except Exception as e:
                self._log_line('⚠️ 关闭串口异常: %s' % e)
        self.btn_conn.configure(state='normal')
        self.btn_disc.configure(state='disabled')
        self.btn_reboot.configure(state='disabled')
        self.cmb_mode.configure(state='disabled')
        for tid in self.test_buttons:
            self.test_buttons[tid].configure(state='disabled')
        self.btn_all.configure(state='disabled')
        self.var_link.set('未连接')
        self.task_name = '空闲'
        with self.tel_lock:
            self.tel['link_ok'] = False
            self.tel['mode'] = '?'
            self.tel['armed'] = None
        tct.reset_launch_params_flag()   # v1.24.5: 断开时重置写入标记
        self._log_line('🔌 已断开连接')

    # ---------------- 飞控重启 (v1.24.4) ----------------

    def _confirm_reboot(self):
        """重启二次确认; 冒烟通过替换本方法打桩"""
        return messagebox.askyesno(
            '重启飞控',
            '确认重启飞控?\n\n'
            '· 必须处于上锁状态 (已解锁会直接拒绝)\n'
            '· 重启约 10~30s, USB 虚拟串口可能需要重新连接\n'
            '· 正在执行的任务会被中断')

    def on_reboot_fc(self):
        """v1.24.4: 重启飞控 — 已解锁/未连接拒绝, 二次确认后入队"""
        p = self._need_proxy()
        if not p:
            return
        if p.motors_armed():
            self._log_line('❌ 已解锁, 禁止重启飞控; 请先上锁再重启')
            return
        if not self._confirm_reboot():
            self._log_line('已取消重启飞控')
            return

        def fn():
            # v1.24.4: 不在 worker 线程碰 tk 变量(var_link.set 曾致异常
            # → 任务显示❌失败但重启实际成功); 进度由 tct 内 print 输出
            return tct.reboot_flight_controller(p)
        self._enqueue('重启飞控', fn)

    # ---------------- 读线程 (唯一 recv 源) ----------------

    def _reader_loop(self):
        me = threading.current_thread()
        last_hb_send = 0.0
        rate_count = 0
        rate_t = time.time()
        # 身份校验: 即使 reader_running 被新一轮连接置回 True, 旧线程也必须退出
        while self.reader_running and self.raw is not None \
                and self.reader_thread is me:
            raw = self.raw
            processed = 0
            # 关键: Windows 串口上 pymavlink 的 select() 对非 socket 句柄会
            # 立即抛错并忙等, 导致 recv_match(blocking=True) 空转、读线程
            # 100% 占用 CPU 并顶着 GIL, 把 tkinter 主线程饿死 -> 界面卡死。
            # 这里改为非阻塞轮询 + 空闲时 sleep 主动让出 GIL。
            for _ in range(300):
                try:
                    msg = raw.recv_match(blocking=False)
                except Exception as e:
                    if self.reader_running:
                        self._log_line('⚠️ 读消息异常: %s' % e)
                        time.sleep(0.5)
                    break
                if msg is None:
                    break
                processed += 1
                now = time.time()
                rate_count += 1

                # 遥测消息速率统计
                mtype = msg.get_type()
                if mtype != 'BAD_DATA':
                    with self.msg_lock:
                        self.msg_acc[mtype] = self.msg_acc.get(mtype, 0) + 1
                        if now - self._msg_rate_t >= 1.0:
                            dt = now - self._msg_rate_t
                            self.msg_rates = {k: v / dt
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

                if self.capture.is_set():
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
                # 空闲: 小睡让出 GIL, 避免空转顶死主线程
                time.sleep(0.004)

    def _update_tel(self, msg):
        mtype = msg.get_type()
        with self.tel_lock:
            t = self.tel
            if mtype == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    return
                t['hb_t'] = time.time()
                mm = {}
                try:
                    mm = self.raw.mode_mapping() or {}
                except Exception:
                    mm = {}
                t['mode'] = next((n for n, i in mm.items()
                                  if i == msg.custom_mode),
                                 'MODE(%s)' % msg.custom_mode)
                t['armed'] = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                t['sys_status'] = msg.system_status
            elif mtype == 'GPS_RAW_INT':
                t['fix'] = msg.fix_type
                t['sats'] = msg.satellites_visible
                t['hdop'] = None if msg.eph == 65535 else msg.eph * 0.01
                t['gps_t'] = time.time()
                # v1.21: 空中GPS失联提示(节流10s, 仅提示不动作, 惯导继续)
                if (isinstance(msg.fix_type, int) and msg.fix_type < 3
                        and t.get('armed')
                        and time.time() - self._gps_lost_t >= 10.0):
                    self._gps_lost_t = time.time()
                    print('⚠️ GPS失联(fix=%d), 惯导继续飞向目标 '
                          '(仅提示, 不影响飞行)' % msg.fix_type)
            elif mtype == 'GLOBAL_POSITION_INT':
                t['lat'] = msg.lat / 1e7
                t['lon'] = msg.lon / 1e7
                t['rel_alt'] = msg.relative_alt / 1000.0
                t['abs_alt'] = msg.alt / 1000.0
                t['vx'] = msg.vx / 100.0
                t['vy'] = msg.vy / 100.0
                t['vz'] = msg.vz / 100.0
                t['heading'] = None if msg.hdg == 65535 else msg.hdg / 100.0
                self.alt_hist.append((time.time(), t['rel_alt']))
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
                t['rssi'] = getattr(msg, 'rssi', None)
                if t['rssi'] == 255:
                    t['rssi'] = None
            elif mtype == 'EKF_STATUS_REPORT':
                try:
                    t['ekf'] = (msg.pos_horiz_variance,
                                msg.velocity_variance,
                                msg.compass_variance)
                except AttributeError:
                    pass
                # v1.29.3: 存 flags 供"位置估计就绪"判定/显示 (非阻塞)
                t['ekf_flags'] = int(getattr(msg, 'flags', 0))
                t['ekf_t'] = time.time()   # v1.30: 监控线程据其判数据陈旧
            elif mtype == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, (bytes, bytearray)):
                    text = text.decode('utf-8', 'replace')
                t['status_text'] = str(text)
            elif mtype == 'HOME_POSITION':
                if msg.latitude and msg.longitude:
                    if self.home is None or self.home.get('src') == '飞控':
                        self.home = {'lat': msg.latitude / 1e7,
                                     'lon': msg.longitude / 1e7,
                                     'src': '飞控'}
            elif mtype == 'VIBRATION':
                # VIBRATION: vibration_x, vibration_y, vibration_z (m/s^2)
                # 存储最新值，_tick 中按 1Hz 采样存入历史
                # 注意: 本方法开头已持有 tel_lock, 此处不可再次获取(非重入锁会死锁)
                t['vib'] = (msg.vibration_x, msg.vibration_y, msg.vibration_z)

    # ---------------- 快速控制 ----------------

    def _need_proxy(self):
        if not self.connected or self.proxy is None:
            self._log_line('❌ 未连接飞控')
            return None
        return self.proxy

    # ---------------- 重试解锁人工确认 (v1.22) ----------------
    def _arm_confirm(self, reason=''):
        """tct 确认钩子(工作线程调用): 醒目提示+等现场人员确认后再解锁.
        防止重试就绪时机身突然自解锁; arm_confirm_auto=True 时跳过(自动化)"""
        if self.arm_confirm_auto:
            print('  (自动确认模式, 跳过人工确认)')
            return True
        self.arm_confirm_gen += 1
        gen = self.arm_confirm_gen
        self.arm_confirm_result = False
        self.arm_confirm_evt.clear()
        self.root.after(0, self._arm_confirm_dialog, reason, gen)
        ok = self.arm_confirm_evt.wait(timeout=tct.ARM_CONFIRM_TIMEOUT)
        if not ok:
            print('  ⛔ 确认超时(%ds), 已取消解锁'
                  % tct.ARM_CONFIRM_TIMEOUT)
            self.root.after(0, self._close_arm_dlg)
            return False
        return bool(self.arm_confirm_result)

    def _arm_confirm_dialog(self, reason, gen):
        """主线程: 置顶模态确认框 (检测一切正常 -> 提示 -> 确认方可解锁)"""
        if gen != self.arm_confirm_gen:
            return
        win = tk.Toplevel(self.root)
        self._arm_dlg = win
        win.title('⚠️ 确认解锁')
        try:
            win.attributes('-topmost', True)
        except Exception:
            pass
        win.geometry('480x240')
        frm = ttk.Frame(win, padding=16)
        frm.pack(fill='both', expand=True)
        # v1.24: 位置估计未就绪时改警示文案, 不再谎报"检测一切正常"
        # v1.29.4: 事由含"改用"= 预检未就绪走 ALT_HOLD 降级, 标题说清降级意图
        if bool(getattr(tct, 'ARM_CONFIRM_ALL_READY', True)):
            headline = '🚨 检测一切正常, 即将解锁电机!'
        elif '改用' in str(reason or ''):
            headline = '⚠️ 位置估计未就绪 → 改用 ALT_HOLD 解锁!'
        else:
            headline = '⚠️ 位置估计未就绪, 解锁可能被拒!'
        ttk.Label(frm, text=headline,
                  font=('Microsoft YaHei', 14, 'bold'),
                  foreground='#b00020').pack(pady=(0, 10))
        ttk.Label(frm, text='⚠️ 请所有人员远离飞机到安全位置,\n'
                            '确认现场安全后再点"确认解锁"',
                  font=('Microsoft YaHei', 11), justify='center').pack()
        if reason:
            ttk.Label(frm, text='事由: %s' % reason,
                      foreground='#666').pack(pady=(8, 0))
        btns = ttk.Frame(frm)
        btns.pack(pady=(18, 0))

        def done(v):
            if gen != self.arm_confirm_gen:
                try:
                    win.destroy()
                except Exception:
                    pass
                return
            self.arm_confirm_result = bool(v)
            self.arm_confirm_evt.set()
            try:
                win.grab_release()
            except Exception:
                pass
            if self._arm_dlg is win:
                self._arm_dlg = None
            try:
                win.destroy()
            except Exception:
                pass
            print('  ✅ 测试人员已确认解锁' if v else
                  '  ⛔ 测试人员取消解锁')

        ttk.Button(btns, text='✅ 确认,可以解锁',
                   command=lambda: done(True)).pack(side='left', padx=12)
        ttk.Button(btns, text='⛔ 取消',
                   command=lambda: done(False)).pack(side='left', padx=12)
        win.protocol('WM_DELETE_WINDOW', lambda: done(False))
        try:
            win.update_idletasks()
            win.geometry('+%d+%d' % (
                max(0, (win.winfo_screenwidth() - 480) // 2),
                max(0, (win.winfo_screenheight() - 240) // 2)))
            win.grab_set()
        except Exception:
            pass

    def _close_arm_dlg(self):
        """主线程: 确认超时后关闭未决确认框, 避免残留误导"""
        win = self._arm_dlg
        if win is None:
            return
        self._arm_dlg = None
        try:
            win.grab_release()
        except Exception:
            pass
        try:
            win.destroy()
        except Exception:
            pass

    def on_arm(self):
        p = self._need_proxy()
        if not p:
            return

        def fn():
            tct.ensure_disarm_delay(p)
            return tct.arm_vehicle(p, mode='GUIDED', timeout=10)
        self._enqueue('解锁 ARM', fn)

    def on_disarm(self):
        p = self._need_proxy()
        if not p:
            return

        def fn():
            ok = tct.disarm_vehicle(p, force=False, timeout=6)
            if ok:
                # v1.24: 上锁后交还遥控器(RC_HANDOVER=1 时全0帧), TX可直接再解锁
                tct.release_rc_override(p)
            return ok
        self._enqueue('锁定 DISARM', fn)

    def on_arm_alt_hold(self):
        """v1.24: 定高(ALT_HOLD)解锁/上锁一键切换 — 未解锁→切ALT_HOLD并解锁,
        已解锁→上锁. ALT_HOLD 不要求水平位置估计, 室内(GPS多径)可用"""
        p = self._need_proxy()
        if not p:
            return

        def fn():
            if p.motors_armed():
                print('已解锁 → 上锁 (定高按钮)')
                ok = tct.disarm_vehicle(p, force=False, timeout=6)
                if ok:
                    tct.release_rc_override(p)
                return ok
            print('定高(ALT_HOLD)解锁: 不依赖水平位置估计')
            tct.ensure_disarm_delay(p)
            return tct.arm_vehicle(p, mode='ALT_HOLD', timeout=10)
        self._enqueue('定高解锁/上锁', fn)

    def on_takeoff(self):
        p = self._need_proxy()
        if not p:
            return
        if not self._apply_params():
            return
        alt = tct.TAKEOFF_ALT
        self.target_alt = alt

        def fn():
            ok = tct.arm_and_takeoff(p, alt, mode='GUIDED', arm_timeout=10,
                                     takeoff_timeout=45, set_delay=True)
            if not ok:
                self.target_alt = None
            else:
                self._hint_takeover()
            return ok
        self._enqueue('起飞到 %.1fm' % alt, fn)

    def on_land(self):
        p = self._need_proxy()
        if not p:
            return

        def fn():
            tct.send_zero_velocity(p, count=5)
            p.set_mode_apm('LAND')
            self._log_line('  已切 LAND (≤1m 油门微调软着陆)...')
            return tct.land_soft_final(p, phase0_timeout=90)
        self._enqueue('降落 LAND', fn)

    def on_rtl(self):
        p = self._need_proxy()
        if not p:
            return

        def fn():
            p.set_mode_apm('RTL')
            self._log_line('  已切 RTL 返航 (≤1m 油门微调软着陆)...')
            return tct.land_soft_final(p, phase0_timeout=180)
        self._enqueue('返航 RTL', fn)

    def on_set_mode(self):
        p = self._need_proxy()
        if not p:
            return
        mode = self.var_mode.get().strip().upper()

        def fn():
            apc.set_mode(p, mode)
            self._log_line('  ✅ 已切换 %s' % mode)
            return True
        self._enqueue('切换模式 %s' % mode, fn)

    def on_velocity(self, vx, vy, vz):
        p = self._need_proxy()
        if not p:
            return
        if not self._apply_params():
            return
        try:
            spd = float(self.var_vspd.get())
        except ValueError:
            self._log_line('❌ 速度无效')
            return
        if vx or vy or vz:
            n = math.sqrt(vx * vx + vy * vy + vz * vz)
            vx, vy, vz = vx / n * spd, vy / n * spd, vz / n * spd
            self.target_speed = spd
        else:
            self.target_speed = 0.0
        type_mask = 0b0000110111000111

        def fn():
            if vx or vy or vz:
                with self.tel_lock:
                    cur = self.tel.get('mode')
                if cur and cur != 'GUIDED' and not str(cur).startswith('MODE('):
                    print('  ⚠️ 当前模式 %s, 速度指令仅 GUIDED 生效, '
                          '飞控会忽略本条 (先[切换模式]GUIDED)' % cur)
            t = time.time() + 1.0
            while time.time() < t and self.action_busy.is_set():
                p.mav.set_position_target_local_ned_send(
                    0, p.target_system, p.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    type_mask, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
                time.sleep(0.1)
            if not (vx == 0 and vy == 0 and vz == 0):
                tct.send_zero_velocity(p, count=5)
            self._log_line('  ✅ 速度指令完成 (%.1f, %.1f, %.1f)' % (
                vx, vy, vz))
            return True
        label = ('停止' if (not vx and not vy and not vz)
                 else '速度 %.1f m/s' % spd)
        self._enqueue(label, fn)

    # ---------------- 抛投 ----------------

    def on_drop(self):
        """抛投触发: 空闲时入队(完整ACK校验); 任务运行中GUI直发不排队
        (排队会导致任务结束落地后才触发, 时机错误)"""
        if not self.connected or self.raw is None:
            self._log_line('❌ 未连接, 无法抛投')
            return
        try:
            sno = int(self.var_drop_no.get())
            pwm_on = int(self.var_drop_on.get())
            off_raw = self.var_drop_off.get().strip()
            pwm_off = int(off_raw) if off_raw else None
            hold_ms = int(self.var_drop_hold.get())
        except ValueError:
            self._log_line('❌ 抛投参数格式错误 (需整数)')
            return
        if not (1 <= sno <= 16):
            self._log_line('❌ 抛投通道超范围(1~16): %d' % sno)
            return
        for tag, v in (('触发', pwm_on), ('回位', pwm_off)):
            if v is not None and not (500 <= v <= 2500):
                self._log_line('❌ %s PWM 超范围(500~2500): %d' % (tag, v))
                return
        if not (0 <= hold_ms <= 60000):
            self._log_line('❌ 保持时间范围 0~60000 ms')
            return
        off_txt = '不回位' if pwm_off is None else '回位 %d µs' % pwm_off
        if not messagebox.askyesno(
                '确认抛投',
                '通道 %d: 触发 %d µs → 保持 %d ms → %s\n\n确认触发抛投?'
                % (sno, pwm_on, hold_ms, off_txt)):
            self._log_line('⚠️ 已取消抛投')
            return

        if not self.action_busy.is_set():
            self._enqueue('抛投 通道%d' % sno,
                          lambda: apc.trigger_drop(
                              self.proxy, sno, pwm_on, pwm_off, hold_ms))
            return

        # 任务运行中: 直发不排队 (同急停先例); STATUSTEXT 拒收信息
        # 经 _update_tel 显示在遥测状态行
        self._log_line('⚠️ 任务运行中, 直发抛投 (通道%d)' % sno)
        self._cancel_pending_drop()
        try:
            self.raw.mav.command_long_send(
                self.raw.target_system, self.raw.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                float(sno), float(pwm_on), 0, 0, 0, 0, 0)
            self._log_line('▶ 抛投已发送: 通道%d %d µs (回位 %d ms 后)' % (
                sno, pwm_on, hold_ms))
        except Exception as e:
            self._log_line('❌ 抛投发送失败: %s' % e)
            return
        if pwm_off is not None:
            if hold_ms > 0:
                self._drop_after_id = self.root.after(
                    hold_ms, lambda: self._drop_return(sno, pwm_off))
            else:
                self._drop_return(sno, pwm_off)

    def _drop_return(self, sno, pwm_off):
        """任务运行直发路径的回位定时回调"""
        self._drop_after_id = None
        if not self.connected or self.raw is None:
            return
        try:
            self.raw.mav.command_long_send(
                self.raw.target_system, self.raw.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                float(sno), float(pwm_off), 0, 0, 0, 0, 0)
            self._log_line('✅ 抛投回位: 通道%d → %d µs' % (sno, pwm_off))
        except Exception as e:
            self._log_line('❌ 抛投回位失败: %s' % e)

    def _on_vib_resize(self, event):
        """画布大小改变时重绘"""
        self._draw_vibration()

    def _draw_vibration(self):
        """绘制震动波形图 (类似 Mission Planner Vibration)"""
        c = self.vib_canvas
        if not c.winfo_exists():
            return
        c.delete('all')
        w = int(c['width'])
        h = int(c['height'])
        if w < 50 or h < 50:
            return

        # 背景网格
        for i in range(5):
            y = h * i / 4
            c.create_line(0, y, w, y, fill='#21262d', width=1)
        for i in range(10):
            x = w * i / 9
            c.create_line(x, 0, x, h, fill='#21262d', width=1)

        # 坐标轴标签
        c.create_text(4, 4, anchor='nw', text='m/s²',
                      fill='#666', font=('Consolas', 8))

        # 三轴颜色
        colors = {'x': '#ff6b6b', 'y': '#7bd87b', 'z': '#8ecae6'}
        labels = {'x': 'X', 'y': 'Y', 'z': 'Z'}

        # 计算最大振幅用于缩放
        all_vals = []
        for axis in ('x', 'y', 'z'):
            arr = self.vib_data[axis]
            if arr:
                all_vals.extend(arr)
        if not all_vals:
            c.create_text(w // 2, h // 2, text='等待震动数据...',
                          fill='#666', font=('Microsoft YaHei', 9))
            return

        max_val = max(max(v) for v in (self.vib_data['x'], self.vib_data['y'], self.vib_data['z']) if v)
        min_val = min(min(v) for v in (self.vib_data['x'], self.vib_data['y'], self.vib_data['z']) if v)
        # 对称范围
        max_abs = max(abs(max_val), abs(min_val), 0.5)
        scale = (h - 20) / (2 * max_abs)

        pad_left = 40
        pad_right = 10
        pad_top = 10
        pad_bottom = 10
        plot_w = w - pad_left - pad_right
        plot_h = h - pad_top - pad_bottom
        center_y = pad_top + plot_h // 2

        for axis in ('x', 'y', 'z'):
            arr = self.vib_data[axis]
            if not arr:
                continue
            color = colors[axis]
            n = len(arr)
            if n < 2:
                continue
            line = []
            for i, val in enumerate(arr):
                x = pad_left + plot_w * i / max(n - 1, 1)
                y = center_y - val * scale
                line.extend((x, y))
            if len(line) >= 4:
                c.create_line(*line, fill=color, width=1.5, tags='vib_line')

            # 当前值标记
            last_val = arr[-1]
            x_last = pad_left + plot_w
            y_last = center_y - last_val * scale
            c.create_oval(x_last - 3, y_last - 3, x_last + 3, y_last + 3,
                          fill=color, outline='', tags='vib_dot')
            c.create_text(x_last + 18, y_last - 4, anchor='sw',
                          text='%s: %.2f' % (labels[axis], last_val),
                          fill=color, font=('Consolas', 8, 'bold'))

        # 零线
        c.create_line(pad_left, center_y, w - pad_right, center_y,
                      fill='#30363d', width=1, dash=(4, 4))

    def _cancel_pending_drop(self):
        if self._drop_after_id is not None:
            try:
                self.root.after_cancel(self._drop_after_id)
            except Exception:
                pass
            self._drop_after_id = None

    # ---------------- 测试 ----------------

    def on_test(self, tid):
        p = self._need_proxy()
        if not p:
            return
        if not self._apply_params():
            return
        name, func = tct.TESTS[tid]
        key = '%s. %s' % (tid, name)

        def fn():
            try:
                ok = bool(func(p))
            except SystemExit as e:
                self._log_line('❌ %s: %s' % (name, e))
                ok = False
            self.last_results[key] = (ok, 'PASS' if ok else 'FAIL')
            return ok
        self._enqueue(key, fn)

    def on_test_all(self):
        p = self._need_proxy()
        if not p:
            return
        if not self._apply_params():
            return

        def fn():
            results = {}
            for tid, (name, func) in tct.TESTS.items():
                if not self.reader_running:
                    break
                try:
                    ok = bool(func(p))
                except SystemExit as e:
                    self._log_line('❌ %s: %s' % (name, e))
                    ok = False
                except Exception as e:
                    self._log_line('❌ %s 异常: %s' % (name, e))
                    ok = False
                results[name] = ok
                self.last_results['%s. %s' % (tid, name)] = (
                    ok, 'PASS' if ok else 'FAIL')
            self._log_line('=' * 40)
            passed = 0
            for name, ok in results.items():
                passed += 1 if ok else 0
                self._log_line('%s  %s' % ('✅' if ok else '❌', name))
            self._log_line('总计: %d/%d 通过' % (passed, len(results)))
            return passed == len(results) and len(results) > 0
        self._enqueue('运行全部测试', fn)

    # ---------------- 功能页构建 ----------------

    def _build_tab_safety(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 安全设置 ')

        ttk.Label(f, text='地理围栏 (仅在已解锁时判定, 0=关闭该项)',
                  style='Gray.TLabel').grid(row=0, column=0, columnspan=4,
                                            sticky='w')
        ttk.Label(f, text='最大距离m:').grid(row=1, column=0, sticky='w',
                                             pady=2)
        self.var_fence_d = tk.StringVar(value='0')
        ttk.Entry(f, textvariable=self.var_fence_d, width=7).grid(
            row=1, column=1, sticky='w', padx=4)
        ttk.Label(f, text='最大高度m:').grid(row=1, column=2, sticky='w',
                                             padx=(8, 0))
        self.var_fence_a = tk.StringVar(value='0')
        ttk.Entry(f, textvariable=self.var_fence_a, width=7).grid(
            row=1, column=3, sticky='w', padx=4)
        ttk.Label(f, text='越界动作:').grid(row=2, column=0, sticky='w',
                                            pady=2)
        self.var_fence_act = tk.StringVar(value='仅告警')
        ttk.Combobox(f, textvariable=self.var_fence_act, width=8,
                     state='readonly',
                     values=('仅告警', '自动RTL')).grid(
            row=2, column=1, sticky='w', padx=4)

        ttk.Separator(f, orient='horizontal').grid(row=3, column=0,
                                                   columnspan=4,
                                                   sticky='ew', pady=8)
        ttk.Label(f, text='低电量保护', style='Gray.TLabel').grid(
            row=4, column=0, columnspan=4, sticky='w')
        ttk.Label(f, text='触发电压V:').grid(row=5, column=0, sticky='w',
                                             pady=2)
        self.var_bat_v = tk.StringVar(value='0')
        ttk.Entry(f, textvariable=self.var_bat_v, width=7).grid(
            row=5, column=1, sticky='w', padx=4)
        ttk.Label(f, text='触发电量%:').grid(row=5, column=2, sticky='w',
                                             padx=(8, 0))
        self.var_bat_p = tk.StringVar(value='0')
        ttk.Entry(f, textvariable=self.var_bat_p, width=7).grid(
            row=5, column=3, sticky='w', padx=4)
        ttk.Label(f, text='告警电压V:').grid(row=6, column=0, sticky='w',
                                             pady=2)
        self.var_bat_wv = tk.StringVar(value='10.5')
        ttk.Entry(f, textvariable=self.var_bat_wv, width=7).grid(
            row=6, column=1, sticky='w', padx=4)
        ttk.Label(f, text='低电动作:').grid(row=6, column=2, sticky='w',
                                            padx=(8, 0))
        self.var_bat_act = tk.StringVar(value='仅告警')
        ttk.Combobox(f, textvariable=self.var_bat_act, width=8,
                     state='readonly',
                     values=('仅告警', '自动LAND', '自动RTL')).grid(
            row=6, column=3, sticky='w', padx=4)

        ttk.Separator(f, orient='horizontal').grid(row=7, column=0,
                                                   columnspan=4,
                                                   sticky='ew', pady=8)
        ttk.Label(f, text='心跳超时s:').grid(row=8, column=0, sticky='w',
                                             pady=2)
        self.var_hb_to = tk.StringVar(value='3.0')
        ttk.Entry(f, textvariable=self.var_hb_to, width=7).grid(
            row=8, column=1, sticky='w', padx=4)
        ttk.Button(f, text='HOME=当前坐标', width=15,
                   command=self.on_set_home).grid(row=8, column=2,
                                                  padx=4, sticky='w')
        ttk.Button(f, text='清除HOME/航迹', width=15,
                   command=self.on_clear_home).grid(row=9, column=2,
                                                    pady=4, sticky='w')
        ttk.Label(f, text='提示: 急停按钮/F9 随时可用, 不受队列影响',
                  style='Gray.TLabel').grid(row=10, column=0, columnspan=4,
                                            sticky='w', pady=(8, 0))

    def _build_tab_curves(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 目标跟踪曲线 ')
        ttk.Label(f, text='青色=实际   红虚线=目标 (起飞/速度微动时记录)',
                  style='Gray.TLabel').pack(anchor='w')
        self.crv_alt_canvas = tk.Canvas(f, height=160, bg='#101820',
                                        highlightthickness=1,
                                        highlightbackground='#444')
        self.crv_alt_canvas.pack(fill='x', pady=(4, 8))
        ttk.Label(f, text='地速跟踪', style='Gray.TLabel').pack(anchor='w')
        self.crv_spd_canvas = tk.Canvas(f, height=160, bg='#101820',
                                        highlightthickness=1,
                                        highlightbackground='#444')
        self.crv_spd_canvas.pack(fill='x', pady=4)

    def _build_tab_map(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 小地图 ')

        hd = ttk.Frame(f)
        hd.pack(fill='x')
        ttk.Label(hd, text='HOME=黄框 当前=红点 航迹=蓝线',
                  style='Gray.TLabel').pack(side='left')
        self.var_map_follow = tk.BooleanVar(value=True)
        ttk.Checkbutton(hd, text='跟随飞机',
                        variable=self.var_map_follow).pack(side='right',
                                                           padx=4)
        ttk.Button(hd, text='回中HOME', width=10,
                   command=self._map_home).pack(side='right', padx=4)
        ttk.Button(hd, text='－', width=3,
                   command=lambda: self._map_zoom(-1)
                   ).pack(side='right', padx=1)
        ttk.Button(hd, text='＋', width=3,
                   command=lambda: self._map_zoom(1)
                   ).pack(side='right', padx=1)
        self.var_map_z = tk.StringVar(value='z16')
        ttk.Label(hd, textvariable=self.var_map_z, width=5,
                  anchor='e').pack(side='right', padx=4)

        # 底图源选择 + 天地图密钥
        r2 = ttk.Frame(f)
        r2.pack(fill='x', pady=(3, 0))
        ttk.Label(r2, text='底图:').pack(side='left')
        self.var_map_src = tk.StringVar(value=MAP_SOURCES['esri']['label'])
        cb = ttk.Combobox(r2, textvariable=self.var_map_src,
                          values=[c['label'] for c in MAP_SOURCES.values()],
                          width=30, state='readonly')
        cb.pack(side='left', padx=4)
        cb.bind('<<ComboboxSelected>>', lambda e: self._map_source_changed())
        ttk.Label(r2, text='天地图tk:').pack(side='left', padx=(10, 2))
        token = ''
        try:
            if os.path.isfile(TIANDITU_KEY_FILE):
                with open(TIANDITU_KEY_FILE, 'r', encoding='utf-8') as fp:
                    token = fp.read().strip()
        except OSError:
            pass
        self.var_map_tk = tk.StringVar(value=token)
        ttk.Entry(r2, textvariable=self.var_map_tk, width=32).pack(
            side='left')
        ttk.Button(r2, text='应用', width=5,
                   command=self._map_apply_token).pack(side='left', padx=4)
        ttk.Label(r2, text=TIANDITU_HELP, style='Gray.TLabel').pack(
            side='left', padx=6)

        self.map_canvas = tk.Canvas(f, bg='#0b1a12', highlightthickness=1,
                                    highlightbackground='#444',
                                    cursor='fleur')
        self.map_canvas.pack(fill='both', expand=True, pady=6)
        self.map_canvas.bind('<MouseWheel>', self._on_map_wheel)
        self.map_canvas.bind('<Button-4>', lambda e: self._map_zoom(1))
        self.map_canvas.bind('<Button-5>', lambda e: self._map_zoom(-1))
        self.map_canvas.bind('<ButtonPress-1>', self._on_map_down)
        self.map_canvas.bind('<B1-Motion>', self._on_map_move)
        self.map_canvas.bind('<Double-Button-1>',
                             lambda e: self._map_home())

        self.var_map_st = tk.StringVar(value='地图加载中 ...')
        ttk.Label(f, textvariable=self.var_map_st, style='Gray.TLabel'
                  ).pack(anchor='w')

        # 地图状态
        self.map_zoom = 16
        self.map_src = 'esri'
        self.map_center = None          # 底图坐标系下的中心; None=未初始化
        self._map_imgs = collections.OrderedDict()
        self._map_drag = None
        self._map_last_draw = 0.0
        self._tile_store = TileStore(self.map_src, token=token)

    def _build_tab_param(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 参数读写 ')
        row = ttk.Frame(f)
        row.pack(fill='x')
        ttk.Label(row, text='参数名:').pack(side='left')
        self.var_param_name = tk.StringVar(value='PILOT_SPEED_UP')
        ttk.Combobox(row, textvariable=self.var_param_name, width=26,
                     values=('PILOT_SPEED_UP', 'WPNAV_SPEED',
                             'WPNAV_SPEED_UP', 'WPNAV_SPEED_DN',
                             'ANGLE_MAX', 'FENCE_RADIUS', 'FENCE_ALT_MAX',
                             'FENCE_ACTION', 'BATT_LOW_VOLT', 'BATT_LOW_MAH',
                             'DISARM_DELAY', 'RTL_ALT', 'LAND_SPEED',
                             'THROTTLE_HOVER', 'SERVO10_FUNCTION')).pack(
            side='left', padx=4)
        ttk.Label(row, text='值:').pack(side='left', padx=(8, 0))
        self.var_param_val = tk.StringVar(value='0')
        ttk.Entry(row, textvariable=self.var_param_val, width=10).pack(
            side='left', padx=4)
        ttk.Button(row, text='读取', command=self.on_param_read).pack(
            side='left', padx=4)
        ttk.Button(row, text='写入', command=self.on_param_write).pack(
            side='left', padx=2)
        cols = ('name', 'value', 'time')
        self.param_tree = ttk.Treeview(f, columns=cols, show='headings',
                                       height=12)
        for c, w, t in (('name', 200, '参数名'), ('value', 120, '值'),
                        ('time', 140, '时间')):
            self.param_tree.heading(c, text=t)
            self.param_tree.column(c, width=w, anchor='w')
        self.param_tree.pack(fill='both', expand=True, pady=6)
        ttk.Label(f, text='提示: 写入立即生效并存入飞控EEPROM, 注意参数取值范围; '
                          '抛投前提 SERVO10_FUNCTION=0 (Disabled)',
                  style='Gray.TLabel').pack(anchor='w')

    def _build_tab_guided(self):
        """GUIDED 单点飞行: 目的地可随时改点, 到达后悬停"""
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 单点飞行(GUIDED) ')
        ttk.Label(f, text='GUIDED 动态改点 · 到达后悬停 · 飞行中可随时改目的地(立即生效)',
                  style='Gray.TLabel').pack(anchor='w', pady=(0, 4))
        gp = ttk.Frame(f)
        gp.pack(fill='x', pady=(6, 0))
        ttk.Label(gp, text='单点飞行:').pack(side='left')
        self.var_dest = tk.StringVar()
        ttk.Entry(gp, textvariable=self.var_dest, width=24,
                  font=('Consolas', 10)).pack(side='left', padx=4)
        ttk.Label(gp, text='高度m:').pack(side='left')
        self.var_dest_alt = tk.StringVar(value='20')
        ttk.Entry(gp, textvariable=self.var_dest_alt, width=6).pack(
            side='left', padx=2)
        ttk.Label(gp, text='巡航m/s:').pack(side='left', padx=(8, 0))
        self.var_cruise = tk.StringVar()
        ttk.Entry(gp, textvariable=self.var_cruise, width=5).pack(
            side='left', padx=2)
        ttk.Button(gp, text='起飞并飞往(GUIDED)',
                   command=self.on_goto_point).pack(side='left', padx=(8, 2))
        ttk.Button(gp, text='仅飞往(GUIDED)',
                   command=lambda: self.on_goto_point(need_takeoff=False)
                   ).pack(side='left', padx=2)
        ttk.Label(f, text='巡航m/s 留空=按飞控现有 WPNAV_SPEED; 填写=发 '
                          'DO_CHANGE_SPEED 指令调速(只对本次飞行生效, 不写参数; '
                          '范围 0.2~20)',
                  style='Gray.TLabel').pack(anchor='w', pady=(3, 0))
        ttk.Label(f, text='目的地由用户自行填写 GPS 数据: '
                          '纬度,经度 或 @东x,北y(相对飞机GPS), 高度填在右侧高度m',
                  style='Gray.TLabel').pack(anchor='w', pady=(3, 0))
        spr = ttk.Frame(f)
        spr.pack(fill='x', pady=(2, 0))
        ttk.Button(spr, text='读飞控速度参数', width=14,
                   command=self.on_speed_params_read).pack(side='left')
        self.var_speed_params = tk.StringVar(
            value='飞控当前: WPNAV_SPEED=未读取  THROTTLE_HOVER=未读取 '
                  '(悬停油门是全速潜力基准)')
        ttk.Label(spr, textvariable=self.var_speed_params,
                  style='Gray.TLabel').pack(side='left', padx=8)
        # 当前模式指示
        mode_row = ttk.Frame(f)
        mode_row.pack(fill='x', pady=(4, 0))
        ttk.Label(mode_row, text='当前模式:').pack(side='left')
        ttk.Label(mode_row, textvariable=self.tel_labels['mode'],
                  font=('Consolas', 10, 'bold')).pack(side='left', padx=4)

    def _build_tab_mission(self):
        """AUTO 航点任务: 固定航线按序飞+抛投"""
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 航点任务(AUTO) ')
        btnrow = ttk.Frame(f)
        btnrow.pack(fill='x')
        ttk.Button(btnrow, text='校验', width=6,
                   command=self.on_wp_check).pack(side='right', padx=2)
        ttk.Button(btnrow, text='仅上传', width=7,
                   command=self.on_wp_upload).pack(side='right', padx=2)
        ttk.Button(btnrow, text='执行(解锁+起飞→AUTO)', width=18,
                   command=self.on_wp_run).pack(side='right', padx=2)
        ttk.Label(btnrow, text='航点任务操作',
                  style='Gray.TLabel').pack(side='left', padx=2)
        ttk.Label(f, text='AUTO 固定航线 · 执行后切 AUTO 按序飞+抛投 · 中途不可改点',
                  style='Gray.TLabel').pack(anchor='w', pady=(3, 0))
        ttk.Label(f, text='每行: 纬度,经度[,高度] | @东x,北y[,高度]'
                          '(按飞机GPS换算) | DROP[,通道,触发PWM,回位PWM,ms]'
                          '  # 注释',
                  style='Gray.TLabel').pack(anchor='w', pady=(3, 0))

        ttk.Label(f, text='航点由用户自行填写 GPS 数据 (直接编辑下方文本框, 或点 '
                          '[加载 waypoints.txt]), 校验无误后再 '
                          '[仅上传]/[执行]',
                  style='Gray.TLabel').pack(anchor='w', pady=(6, 0))

        self.txt_wp = tk.Text(f, height=9, font=('Consolas', 10),
                              bg='#141414', fg='#d8d8d8', wrap='none')
        self.txt_wp.pack(fill='both', expand=True, pady=4)
        self.var_wp_st = tk.StringVar(value='未加载航点')
        ttk.Label(f, textvariable=self.var_wp_st, style='Gray.TLabel'
                  ).pack(anchor='w')
        ttk.Button(f, text='加载 waypoints.txt (当前目录)',
                   command=self.on_wp_load).pack(anchor='w', pady=(4, 0))
        # 当前模式指示
        mode_row = ttk.Frame(f)
        mode_row.pack(fill='x', pady=(4, 0))
        ttk.Label(mode_row, text='当前模式:').pack(side='left')
        ttk.Label(mode_row, textvariable=self.tel_labels['mode'],
                  font=('Consolas', 10, 'bold')).pack(side='left', padx=4)

    def _build_tab_rc(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 软遥控 ')
        ttk.Label(f, text='软遥控 = 屏幕摇杆/键盘经 MAVLink RC 覆盖打杆; '
                          '仅手动模式生效, 任务运行中自动停用',
                  style='Gray.TLabel').pack(anchor='w')
        hdr = ttk.Frame(f)
        hdr.pack(fill='x', pady=4)
        self.var_rc_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(hdr, text='启用软遥控 (CH1-4, 1000~2000)',
                        variable=self.var_rc_on,
                        command=self.on_rc_toggle).pack(side='left')
        ttk.Button(hdr, text='一键接管(LOITER+回中)',
                   command=self.on_rc_takeover).pack(side='left', padx=6)
        ttk.Button(hdr, text='回中', width=6,
                   command=self.on_rc_center).pack(side='left', padx=6)
        ttk.Checkbutton(hdr, text='键盘接管',
                        variable=self.var_kb_on,
                        command=self.on_soft_rc_kb_toggle).pack(
            side='left', padx=6)
        self.var_rc_st = tk.StringVar(value='未启用')
        ttk.Label(hdr, textvariable=self.var_rc_st, style='Gray.TLabel'
                  ).pack(side='left', padx=8)

        self.var_rc_mode = tk.StringVar(value='模式: --')
        ttk.Label(f, textvariable=self.var_rc_mode,
                  style='Gray.TLabel').pack(anchor='w')
        ttk.Label(f, text='键位: WASD=俯仰/横滚  Q/E=偏航  '
                          'Shift/Ctrl=油门升降  R=回中  (输入框内不响应); '
                          '限幅 横滚/俯仰 ±%dus(1250~1750) 偏航 ±%dus(1200~1800)'
                          % (tct.SOFT_RC_RP_US, tct.SOFT_RC_YAW_US),
                  style='Gray.TLabel').pack(anchor='w')

        names = (('CH1 横滚', 1500), ('CH2 俯仰', 1500),
                 ('CH3 油门', 1000), ('CH4 偏航', 1500))
        self.rc_scale = []
        cols = ttk.Frame(f)
        cols.pack(fill='x', pady=4)
        for nm, init in names:
            cell = ttk.Frame(cols)
            cell.pack(side='left', padx=6)
            ttk.Label(cell, text=nm).pack(anchor='w')
            sc = tk.Scale(cell, from_=1000, to=2000, orient='horizontal',
                          length=150, resolution=10, showvalue=True,
                          font=('Consolas', 9))
            sc.set(init)
            sc.pack(anchor='w')
            self.rc_scale.append(sc)

        # 虚拟摇杆: 左=偏航/油门 右=横滚/俯仰 (拖动, 右杆松手回中)
        sticks = ttk.Frame(f)
        sticks.pack(fill='x', pady=(8, 0))
        self._build_stick(sticks, '左摇杆 (上=油门+ / 右=偏航+)', 'l')
        self._build_stick(sticks, '右摇杆 (上=俯仰前 / 右=横滚+)', 'r')
        ttk.Label(sticks, text='拖动虚拟摇杆即时打杆;\n'
                               '左杆松手: 油门保持+偏航回中\n'
                               '右杆松手: 俯仰/横滚回中\n'
                               '应急接管: [一键接管] 或 F9',
                  style='Gray.TLabel').pack(side='left', padx=14)

    def _build_tab_signals(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 信号速率 ')
        ttk.Label(f, text='控制指令 (GUI发送计数, 近1秒)',
                  style='Gray.TLabel').pack(anchor='w')
        cols = ('cmd', 'hz')
        self.tree_sig = ttk.Treeview(f, columns=cols, show='headings',
                                     height=7)
        self.tree_sig.heading('cmd', text='MAVLink 指令')
        self.tree_sig.heading('hz', text='Hz')
        self.tree_sig.column('cmd', width=280, anchor='w')
        self.tree_sig.column('hz', width=70, anchor='e')
        self.tree_sig.pack(fill='x', pady=(2, 8))

        ttk.Label(f, text='遥测消息 (读线程计数, 近1秒)',
                  style='Gray.TLabel').pack(anchor='w')
        cols2 = ('msg', 'hz')
        self.tree_msg = ttk.Treeview(f, columns=cols2, show='headings',
                                     height=10)
        self.tree_msg.heading('msg', text='消息类型')
        self.tree_msg.heading('hz', text='Hz')
        self.tree_msg.column('msg', width=280, anchor='w')
        self.tree_msg.column('hz', width=70, anchor='e')
        sb = ttk.Scrollbar(f, orient='vertical',
                           command=self.tree_msg.yview)
        self.tree_msg.configure(yscrollcommand=sb.set)
        self.tree_msg.pack(side='left', fill='both', expand=True, pady=2)
        sb.pack(side='right', fill='y')

    # ---------------- 安全: 告警 / 急停 / HOME ----------------

    def _alarm(self, kind, text, level='bad'):
        now = time.time()
        if now - self._alarm_last.get(kind, 0) < ALARM_COOLDOWN:
            return
        self._alarm_last[kind] = now
        self._log_line('%s [%s] %s' % (
            '❌' if level == 'bad' else '⚠️', kind, text))

    def on_panic_loiter(self):
        self.on_immediate_mode('LOITER', '⛔ 急停')

    def on_immediate_mode(self, mode, prefix='立即'):
        """GUI 线程直发(非阻塞), 不进队列, 不等 ACK"""
        if not self.connected or self.raw is None:
            self._log_line('❌ 未连接, 无法%s %s' % (prefix, mode))
            return
        if self.action_busy.is_set():
            self._log_line('⚠️ 有任务运行中, 仍强制%s %s' % (prefix, mode))
        try:
            self.raw.set_mode_apm(mode)
            self._log_line('%s已发送模式 %s (F9=LOITER 急停)' % (prefix, mode))
        except Exception as e:
            self._log_line('❌ %s发送失败: %s' % (prefix, e))

    def on_handover_rc(self):
        """v1.24: 立即交还遥控器 — 发4通道全0帧撤销GCS RC覆盖,
        物理遥控器摇杆即刻生效 (RC_OVERRIDE_TIME=-1 下覆盖永不过期,
        不交还则接收机输入一直被屏蔽). 顺带停用软遥控防止20Hz回发覆盖.
        注意: 摇杆生效还要求模式在手动档(GUIDED下摇杆无效→F9切LOITER)"""
        if not self.connected or self.raw is None:
            self._log_line('❌ 未连接, 无法交还遥控器')
            return
        if self.var_rc_on.get():
            self._rc_gen += 1                 # 作废未完成的启用回调
            self._cancel_soft_rc_loop()
            self._key_state.clear()
            self.var_rc_on.set(False)
            self.var_rc_st.set('已交还物理遥控器')
            self._log_line('  (软遥控已同时停用)')
        self._rc_was_on = False
        try:
            tct.release_rc_override(self.raw, handover=True)
            self.var_rc_st.set('已交还物理遥控器')
            self._log_line('✅ RC覆盖已交还: 物理遥控器摇杆即刻生效 '
                           '(手动模式下有效; GUIDED/自动档摇杆无效, '
                           '可按 F9 切 LOITER 接管)')
        except Exception as e:
            self._log_line('❌ 交还遥控器失败: %s' % e)

    def on_write_launch_params(self):
        """v1.30: 唯一的参数写入口; v1.32: 先弹[参数说明页], 操作员在页底点
        [✅ 确认写入] 才真正写飞控参数 — 不确认则一个参数都不写"""
        p = self._need_proxy()
        if not p:
            return
        self._open_launch_param_dialog(p)

    def _launch_param_rows(self):
        """说明页展示行 = tct.LAUNCH_PARAMS + DISARM_DELAY (顺序=写入顺序)"""
        rows = [(str(n), float(v), str(lab))
                for n, v, lab in getattr(tct, 'LAUNCH_PARAMS', ())]
        dsec = float(getattr(tct, 'DISARM_DELAY_SEC', 0) or 0)
        if dsec > 0:
            rows.append(('DISARM_DELAY', dsec, '解锁后未起飞自动上锁秒数'))
        return rows

    def _open_launch_param_dialog(self, p):
        """主线程: 打开说明确认页(置顶, 非模态), 并排队回读飞控当前值"""
        win = self._launch_dlg
        if win is not None:
            try:
                win.lift()
                win.focus_force()
            except Exception:
                pass
            return
        rows = self._launch_param_rows()
        if not rows:
            self._log_line('⚠️ 没有可写入的发射箱参数(LAUNCH_PRESET=0?)')
            return

        win = tk.Toplevel(self.root)
        self._launch_dlg = win
        win.title('📋 写入发射箱参数 — 参数说明与确认')
        try:
            win.attributes('-topmost', True)
        except Exception:
            pass
        win.geometry('940x600')
        frm = ttk.Frame(win, padding=12)
        frm.pack(fill='both', expand=True)

        ttk.Label(frm, text='即将写入飞控 %d 个参数 — 请逐项核对'
                  % len(rows), font=('Microsoft YaHei', 13, 'bold')).pack(
            anchor='w')
        info = ('• 只有点下方 [✅ 确认写入] 才会写入; 直接关闭本页 = 不改任何参数。\n'
                '• 写入时只写"飞控当前值 ≠ 目标值"的参数, 已一致的自动跳过。\n'
                '• 参数写入立即生效并保存到飞控 EEPROM(掉电保留), 改回需手动重写。\n'
                '• 本组参数面向发射箱/台架: 关闭机上自动保护(失联返航/降落/围栏/'
                '低电降落), 改由地面站接管 — 请勿在带接收机的实飞机上使用。\n'
                '• ⚠️ 风险: 关闭后飞机失去机上自动保护, 必须有人盯守地面站, '
                '随时可用 [⛔ 急停] 悬停/降落。\n'
                '• 打开本页会自动回读飞控当前值; 读不到的行显示 "--"(不影响确认)。')
        ttk.Label(frm, text=info, foreground='#444', justify='left',
                  wraplength=890).pack(anchor='w', pady=(6, 8))

        cols = ('param', 'cur', 'target', 'act', 'desc')
        box = ttk.Frame(frm)
        box.pack(fill='both', expand=True)
        tree = ttk.Treeview(box, columns=cols, show='headings', height=13)
        for c, t, w in (('param', '参数名', 138), ('cur', '飞控当前值', 96),
                        ('target', '目标值', 74), ('act', '动作', 148),
                        ('desc', '说明', 440)):
            tree.heading(c, text=t)
            tree.column(c, width=w, anchor='w')
        tree.tag_configure('need', foreground='#b00020')
        tree.tag_configure('same', foreground='#0a7a28')
        tree.tag_configure('wait', foreground='#888888')
        vs = ttk.Scrollbar(box, orient='vertical', command=tree.yview)
        tree.configure(yscrollcommand=vs.set)
        vs.pack(side='right', fill='y')
        tree.pack(side='left', fill='both', expand=True)

        ctx = {'win': win, 'tree': tree, 'rows': {}, 'read': 0,
               'total': len(rows), 'proxy': p}
        for name, target, label in rows:
            iid = tree.insert('', 'end', tags=('wait',), values=(
                name, '读取中…', '%g' % target, '⏳ 回读中',
                LAUNCH_PARAM_DOC.get(name, label)))
            ctx['rows'][name] = (iid, target)
        self._launch_dlg_ctx = ctx

        bot = ttk.Frame(frm)
        bot.pack(fill='x', pady=(8, 0))
        status = ttk.Label(bot, text='⏳ 正在回读飞控当前值…', foreground='#666')
        status.pack(side='left')
        ctx['status'] = status

        def close():
            if self._launch_dlg is win:
                self._launch_dlg = None
                self._launch_dlg_ctx = None
            try:
                win.destroy()
            except Exception:
                pass

        def confirm():
            if not self.connected or self.proxy is None:
                self._log_line('❌ 已断开飞控, 取消写入参数')
                close()
                return
            proxy = self.proxy
            close()
            self._log_line('✅ 已确认写入发射箱参数 (%d 项)' % len(rows))
            self._enqueue('写入发射箱参数',
                          lambda: self._do_write_launch_params(proxy))

        btns = ttk.Frame(bot)
        btns.pack(side='right')
        ttk.Button(btns, text='⛔ 取消(不写入)',
                   command=close).pack(side='right', padx=(8, 0))
        ttk.Button(btns, text='✅ 确认写入',
                   command=confirm).pack(side='right')
        win.protocol('WM_DELETE_WINDOW', close)
        try:
            win.update_idletasks()
            win.geometry('+%d+%d' % (
                max(0, (win.winfo_screenwidth() - 940) // 2),
                max(0, (win.winfo_screenheight() - 600) // 2)))
        except Exception:
            pass
        # 回读走任务队列(与参数页一致), 不阻塞主线程
        self._enqueue('回读发射箱参数当前值',
                      lambda: self._read_launch_params(ctx, rows))

    def _read_launch_params(self, ctx, rows):
        """工作线程: 逐项回读飞控当前值; 说明页已关闭则提前停"""
        for name, target, label in rows:
            if self._launch_dlg_ctx is not ctx:
                return None
            try:
                if not ctx['win'].winfo_exists():
                    return None
            except Exception:
                return None
            try:
                cur = tct.get_param(ctx['proxy'], name, timeout=1.5)
            except Exception:
                cur = None
            self.root.after(0, self._launch_param_apply, name, cur)
            time.sleep(0.05)
        return True

    def _launch_param_apply(self, name, cur):
        """主线程: 回读结果刷新说明页某一行 + 进度状态"""
        ctx = self._launch_dlg_ctx
        if ctx is None or name not in ctx['rows']:
            return
        iid, target = ctx['rows'][name]
        if cur is None:
            cur_txt, action, tag = '--', '固件无此参数/未响应(跳过)', 'wait'
        elif abs(float(cur) - target) < 0.01:
            cur_txt, action, tag = ('%g' % float(cur),
                                    '✅ 已一致, 跳过不写', 'same')
        else:
            cur_txt, action, tag = '%g' % float(cur), '✏️ 将写入', 'need'
        try:
            desc = ctx['tree'].set(iid, 'desc')
            ctx['tree'].item(iid, tags=(tag,), values=(
                name, cur_txt, '%g' % target, action, desc))
        except Exception:
            return
        ctx['read'] += 1
        try:
            if ctx['read'] >= ctx['total']:
                ctx['status'].config(
                    text='✅ 回读完成 %d/%d — 点 [✅ 确认写入] 开始写入'
                         % (ctx['read'], ctx['total']),
                    foreground='#0a7a28')
            else:
                ctx['status'].config(
                    text='⏳ 正在回读飞控当前值 %d/%d …'
                         % (ctx['read'], ctx['total']),
                    foreground='#666')
        except Exception:
            pass

    def _do_write_launch_params(self, master):
        """实际执行写入：每次点击都写(只写与期望值不符的项)"""
        print("  🔧 手动写入发射箱参数 + DISARM_DELAY...")
        tct.write_launch_params(master)
        tct.write_disarm_delay(master)
        print("  ✅ 发射箱参数与 DISARM_DELAY 手动写入完成; "
              "系统后续只做飞行控制, 不再自动写任何参数")

    def on_set_home(self):
        with self.tel_lock:
            lat, lon = self.tel['lat'], self.tel['lon']
        if lat is None or lon is None:
            self._log_line('❌ 无 GPS 定位, 无法设置 HOME')
            return
        self.home = {'lat': lat, 'lon': lon, 'src': '手动(GPS)'}
        self.track.clear()
        self._log_line('✅ HOME = %.7f, %.7f' % (lat, lon))

    def on_clear_home(self):
        self.home = None
        self.track.clear()
        self._dist_home = None
        self._log_line('已清除 HOME / 航迹')

    def _pos_ready_ui(self, t):
        """v1.29.3: 位置估计就绪状态文案+样式 (判据与 arm_vehicle 预检门禁
        共用 tct.position_ready_state, 避免"界面说就绪但解锁仍被拒").
        返回 (文案, ttk style 名)"""
        if not self.connected:
            return '--', 'Gray.TLabel'
        ready, why = tct.position_ready_state(
            t.get('fix'), t.get('sats'), t.get('ekf_flags'))
        if ready:
            return '✅ 就绪 (可GUIDED解锁)', 'Ok.TLabel'
        if t.get('ekf_flags') is None and t.get('fix') is None:
            return '⏳ 等待飞控数据', 'Gray.TLabel'
        # 等待中(上电收敛/搜星) 用黄色, 永不满足(如 EKF 失效)也用黄提示
        return '⏳ ' + why, 'Warn.TLabel'

    # ---------------- EKF 解锁判定监控线程 (v1.30) ----------------

    def _start_ekf_monitor(self):
        """连接成功后启动独立 EKF 监控线程 (主线程调用).
        与读线程分离: 只读共享遥测 + 只发请求, 不抢 recv_match"""
        self.ekf_mon_running = False
        old = self.ekf_mon_thread
        if old is not None and old.is_alive() \
                and old is not threading.current_thread():
            old.join(timeout=1.5)
        self._ekf_mon_state = None
        self._ekf_mon_log_t = 0.0
        self.ekf_verdict = {'text': 'EKF: ⏳ 检测中...', 'style': 'Gray.TLabel',
                            'ready': False, 'why': '检测中'}
        self.ekf_mon_running = True
        self.ekf_mon_thread = threading.Thread(target=self._ekf_monitor_loop,
                                               daemon=True)
        self.ekf_mon_thread.start()
        self._log_line('🔍 EKF 监控线程已启动 (判据与解锁预检门禁一致)')

    def _stop_ekf_monitor(self):
        """停监控线程 (断开/退出时调用)"""
        self.ekf_mon_running = False
        t = self.ekf_mon_thread
        if t is not None and t.is_alive() \
                and t is not threading.current_thread():
            t.join(timeout=1.5)
        self.ekf_mon_thread = None

    def _ekf_monitor_loop(self):
        """独立线程: 每秒判定位置估计是否就绪, 结论推给安全条, 状态变化打日志
        (✅ EKF正常可GUIDED解锁 / ⏳ 未就绪原因).
        数据源 = 读线程灌的共享遥测, 全程不调 recv_match (不与读线程/测试任务
        抢消息); 仅当 EKF 状态报文陈旧(>6s)才主动 REQUEST 一次 (只发不收,
        零竞争), 保证固件不流式发送时也有判据."""
        me = threading.current_thread()
        last_req = 0.0
        while self.ekf_mon_running and self.ekf_mon_thread is me \
                and self.connected:
            now = time.time()
            with self.tel_lock:
                fix = self.tel.get('fix')
                sats = self.tel.get('sats')
                flags = self.tel.get('ekf_flags')
                ekf_t = self.tel.get('ekf_t', 0.0)
                hb_t = self.tel.get('hb_t', 0.0)
            raw = self.raw
            if raw is not None and now - ekf_t > 6.0 \
                    and now - last_req >= 3.0:
                last_req = now
                try:
                    raw.mav.command_long_send(
                        raw.target_system, raw.target_component,
                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                        0, 0, 0, 0, 0, 0)
                except Exception:
                    pass

            if hb_t and now - hb_t > 5.0:
                text, style, ready, why = (
                    'EKF: ⚠️ 心跳丢失', 'Bad.TLabel', False, '心跳丢失')
            elif fix is None and flags is None:
                text, style, ready, why = (
                    'EKF: ⏳ 等待飞控数据', 'Gray.TLabel', False, '等待数据')
            else:
                ready, why = tct.position_ready_state(fix, sats, flags)
                if ready:
                    text, style = 'EKF: ✅ 正常 可GUIDED解锁', 'Ok.TLabel'
                else:
                    text, style = 'EKF: ⏳ 未就绪: %s' % why, 'Warn.TLabel'
            self.ekf_verdict = {'text': text, 'style': style,
                                'ready': ready, 'why': why}
            self._ekf_log_state(ready, why, flags)
            time.sleep(1.0)

    def _ekf_log_state(self, ready, why, flags):
        """判定(就绪+原因)变化才打日志, 5s 节流防止每秒刷屏;
        节流期间不更新状态, 下一轮仍未变则补打 (延迟不丢失)"""
        key = (bool(ready), str(why))
        now = time.time()
        if key == self._ekf_mon_state:
            return
        if now - self._ekf_mon_log_t < 5.0:
            return
        self._ekf_mon_state = key
        self._ekf_mon_log_t = now
        if ready:
            self._log_line('✅ EKF 正常, 可 GUIDED 解锁%s' % (
                '' if flags is None else ' (flags=0x%04x)' % int(flags)))
        else:
            self._log_line('⏳ EKF 未就绪(%s), 暂不可 GUIDED 解锁' % why)

    def _check_safety(self, now):
        """围栏/低电/心跳监控 (GUI 线程, _tick 内调用)"""
        with self.tel_lock:
            t = dict(self.tel)
        connected = self.connected
        hb_to = _fnum(self.var_hb_to, 3.0)

        # --- 心跳 ---
        if connected and t['hb_t'] > 0:
            age = now - t['hb_t']
            if age > hb_to:
                self.var_hb_st.set('心跳: 丢失 %.1fs' % age)
                self.lbl_hb.configure(style='Bad.TLabel')
                if not self.hb_lost:
                    self.hb_lost = True
                    self._alarm('hb', '心跳丢失 %.1fs (阈值%.1fs), 链路可能断开'
                                % (age, hb_to))
            else:
                self.var_hb_st.set('心跳: 正常 %.2fs' % age)
                self.lbl_hb.configure(style='Ok.TLabel')
                self.hb_lost = False
        else:
            self.var_hb_st.set('心跳: --')
            self.lbl_hb.configure(style='Gray.TLabel')
            self.hb_lost = False

        armed = bool(t['armed'])
        if not armed:
            self.fence_done = False
            self.bat_done = False

        # --- 围栏 ---
        fd = _fnum(self.var_fence_d, 0)
        fa = _fnum(self.var_fence_a, 0)
        rel = t['rel_alt']
        dist = self._dist_home
        breach_d = fd > 0 and dist is not None and dist > fd
        breach_a = fa > 0 and rel is not None and rel > fa
        if breach_d or breach_a:
            why = []
            if breach_d:
                why.append('距离%.0fm>%.0fm' % (dist, fd))
            if breach_a:
                why.append('高度%.0fm>%.0fm' % (rel, fa))
            self.var_fence_st.set('围栏: 越界 %s' % ' '.join(why))
            self.lbl_fence.configure(style='Bad.TLabel')
            self._alarm('fence', '围栏越界(%s)' % ' '.join(why))
            if armed and not self.fence_done and \
                    self.var_fence_act.get() == '自动RTL':
                self.fence_done = True
                self._enqueue('围栏自动RTL', self._act_rtl)
        elif fd > 0 or fa > 0:
            inside_d = fd <= 0 or (dist is not None and dist < fd * 0.9)
            inside_a = fa <= 0 or (rel is not None and rel < fa * 0.9)
            if inside_d and inside_a:
                self.fence_done = False
            self.var_fence_st.set('围栏: 正常 (d%.0f/a%.0f)' % (fd, fa))
            self.lbl_fence.configure(style='Ok.TLabel')
        else:
            self.var_fence_st.set('围栏: 关闭')
            self.lbl_fence.configure(style='Gray.TLabel')

        # --- 电池 ---
        bv = _fnum(self.var_bat_v, 0)
        bp = _fnum(self.var_bat_p, 0)
        wv = _fnum(self.var_bat_wv, 10.5)
        volt, remain = t['volt'], t['remain']
        low_v = bv > 0 and volt is not None and volt < bv
        low_p = bp > 0 and remain is not None and 0 <= remain < bp
        warn_v = volt is not None and volt < wv
        if low_v or low_p:
            self.var_bat_st.set('电池: 低电 V%s %s%%' % (
                ('%.1f' % volt) if volt is not None else '--', remain))
            self.lbl_bat.configure(style='Bad.TLabel')
            self._alarm('bat', '低电量! V=%s 剩余%s%%' % (volt, remain))
            if armed and not self.bat_done:
                act = self.var_bat_act.get()
                if act == '自动LAND':
                    self.bat_done = True
                    self._enqueue('低电自动LAND', self._act_land)
                elif act == '自动RTL':
                    self.bat_done = True
                    self._enqueue('低电自动RTL', self._act_rtl)
        elif warn_v:
            self.var_bat_st.set('电池: 偏低 V%.1f %s%%' % (volt, remain))
            self.lbl_bat.configure(style='Warn.TLabel')
            self._alarm('batw', '电压偏低 V=%.1f (告警线%.1f)' % (volt, wv),
                        level='warn')
        elif volt is not None:
            self.var_bat_st.set('电池: 正常 V%.1f %s%%' % (volt, remain))
            self.lbl_bat.configure(style='Ok.TLabel')
        else:
            self.var_bat_st.set('电池: --')
            self.lbl_bat.configure(style='Gray.TLabel')

        self.var_prot_st.set('保护已触发' if (self.fence_done or
                                            self.bat_done) else '')

    def _act_rtl(self):
        if self.raw is None:
            return False
        self.raw.set_mode_apm('RTL')
        self._log_line('  保护动作: 已切 RTL')
        return True

    def _act_land(self):
        if self.raw is None:
            return False
        self.raw.set_mode_apm('LAND')
        self._log_line('  保护动作: 已切 LAND')
        return True

    # ---------------- 软遥控 / RC 覆盖 (v1.23) ----------------

    def _release_rc_override(self):
        """断开/退出/停用前收尾 (v1.23 中立帧): 无接收机下全0=交还遥控器
        =无效PWM(空中掉高); 按 armed 自动选地面/空中中立帧, 连发3帧"""
        raw = self.raw
        if raw is None:
            return
        try:
            tct.release_rc_override(raw)
        except Exception:
            pass

    def on_rc_toggle(self):
        """启用 = 只读校验软遥控所需参数(RC_OVERRIDE_TIME=-1/RC_OPTIONS=0),
        worker 完成回调后才真正打开 (gen 作废防止停用后被旧任务重开).
        v1.30: 系统不写参数 — 不符时提示用户先点[写入发射箱参数]再启用"""
        if self.var_rc_on.get():
            self.var_rc_on.set(False)
            if not self.connected:
                self.var_rc_st.set('未连接')
                self._log_line('❌ 未连接, 无法启用软遥控')
                return
            if self.action_busy.is_set():
                self.var_rc_st.set('任务运行中, 拒绝启用')
                self._log_line('⚠️ 任务运行中, 不能启用软遥控')
                return
            self._rc_gen += 1
            gen = self._rc_gen
            p = self.proxy

            def fn():
                v1 = tct.get_param(p, 'RC_OVERRIDE_TIME', timeout=2.0)
                v2 = tct.get_param(p, 'RC_OPTIONS', timeout=2.0)
                # RC_OVERRIDE_TIME: 0=禁用GCS覆盖(软遥控无效), -1=永不过期
                # RC_OPTIONS bit1(值2)=忽略GCS覆盖
                ok1 = v1 is not None and abs(v1) > 0.01
                ok2 = v2 is not None and (int(v2) & 2) == 0

                def fin():
                    if gen != self._rc_gen:
                        self._log_line('软遥控启用已取消(开关已关)')
                        return
                    if self.action_busy.is_set():
                        # 等 worker finally 释放 busy, 否则下一拍 _send_rc
                        # 会把刚启用的软遥控当"任务中"自动停用
                        self.root.after(50, fin)
                        return
                    if not (ok1 and ok2):
                        self.var_rc_st.set('参数未就绪, 未启用')
                        self.var_rc_on.set(False)
                        self._log_line(
                            '❌ 软遥控参数不符(系统不自动写参数): '
                            'RC_OVERRIDE_TIME=%s(需≠0, 建议-1) '
                            'RC_OPTIONS=%s(bit1=2需为0); '
                            '请先点[写入发射箱参数]再启用软遥控'
                            % (('读不到' if v1 is None else '%g' % v1),
                               ('读不到' if v2 is None else '%g' % v2)))
                        return
                    if v1 is not None and abs(v1 - (-1.0)) >= 0.01:
                        self._log_line(
                            '⚠️ RC_OVERRIDE_TIME=%g (非-1), 覆盖会在超时后'
                            '回落遥控器; 建议点[写入发射箱参数]改为 -1' % v1)
                    self.var_rc_on.set(True)
                    self._rc_grace_t = time.time() + 5
                    self.var_rc_st.set('启用中 (%dHz)' % tct.SOFT_RC_RATE_HZ)
                    self._log_line(
                        '⚠️ 软遥控已启用: 摇杆/键盘生效(仅手动模式), %dHz 发送; '
                        '5s 内模式未切手动将自动停用' % tct.SOFT_RC_RATE_HZ)
                    self._start_soft_rc_loop()
                self.root.after(0, fin)
                return ok1 and ok2
            self._enqueue('软遥控参数校验(只读)', fn)
            self.var_rc_st.set('校验中(参数)...')
            self._log_line('▶ 只读校验软遥控参数, 通过后自动启用软遥控')
        else:
            self._rc_gen += 1          # 作废未完成的参数准备
            self._cancel_soft_rc_loop()
            self._key_state.clear()
            self.var_rc_st.set('已关闭')
            if self._rc_was_on:
                self._release_rc_override()
                self._rc_was_on = False
                self._log_line('软遥控已停用(RC覆盖已收尾, '
                               + ('全0交还遥控器' if tct.RC_HANDOVER
                                  else '中立帧') + ')')
            else:
                self._log_line('软遥控已关闭')

    def on_rc_takeover(self):
        """一键应急接管: 摇杆回中(油门1500=空中安全) + 切 LOITER + 启用.
        F9=纯切LOITER; 本按钮额外保证启用时杆值全回中"""
        if not self.connected:
            self._log_line('❌ 未连接, 无法接管')
            return
        for i, v in enumerate(tct.RC_NEUTRAL_AIR):
            self.rc_scale[i].set(v)
        self._stick.update(lx=0.0, ly=0.0, rx=0.0, ry=0.0)
        self._sync_sticks()
        self._rc_grace_t = time.time() + 5
        self.on_immediate_mode('LOITER', '⛔ 软遥控接管')
        if self.var_rc_on.get():
            self._log_line('软遥控已启用, 摇杆已回中(油门1500), 模式切 LOITER')
            self._start_soft_rc_loop()
        else:
            self.var_rc_on.set(True)
            self.on_rc_toggle()         # 走参数准备流程, 完成后自动启用

    def on_rc_center(self):
        self.rc_scale[0].set(1500)
        self.rc_scale[1].set(1500)
        self.rc_scale[2].set(1000)
        self.rc_scale[3].set(1500)
        self._stick.update(lx=0.0, ly=-1.0, rx=0.0, ry=0.0)
        self._sync_sticks()
        if self.var_rc_on.get():
            self._send_rc(force=True)

    def _send_rc(self, force=False, min_interval=0.2):
        """软遥控发送 (v1.23): 仅手动模式下生效; 任务运行/非手动模式
        自动中立帧停用; 启用期间由专职 20Hz 循环调用(min_interval=0)"""
        on = bool(self.connected and self.var_rc_on.get())
        with self.tel_lock:
            mode = str(self.tel.get('mode') or '?').upper()
        manual = mode in tct.SOFT_RC_MANUAL_MODES
        if on and self.action_busy.is_set():
            on = False
            self.var_rc_on.set(False)
            self.var_rc_st.set('任务运行中已自动停用')
            self._log_line('⚠️ 任务运行中, 软遥控已自动停用(中立帧收尾)')
        if on and not manual:
            if mode == '?' or time.time() < self._rc_grace_t:
                return                  # 等模式信息/等模式切到手动, 本拍不发
            on = False
            self.var_rc_on.set(False)
            self.var_rc_st.set('非手动模式已停用')
            self._log_line('⚠️ 当前 %s 模式摇杆无效, 软遥控已停用 '
                           '(应急接管: F9切LOITER 或 点[一键接管])' % mode)
        now = time.time()
        if not force and now - self._rc_last_send < min_interval:
            return
        if not on:
            if self._rc_was_on:
                self._release_rc_override()   # 中立帧收尾(3帧)
                self._rc_was_on = False
            return
        if not self.connected or self.raw is None:
            return
        self._rc_last_send = now
        vals = [int(self.rc_scale[i].get()) for i in range(4)]
        try:
            self.raw.mav.rc_channels_override_send(
                self.raw.target_system, self.raw.target_component,
                vals[0], vals[1], vals[2], vals[3], 0, 0, 0, 0)
            self._rc_was_on = True
        except Exception as e:
            self._log_line('❌ 软遥控发送失败: %s' % e)
            self.var_rc_on.set(False)
            self.var_rc_st.set('发送失败已关闭')

    # ---- 20Hz 发送循环 ----

    def _start_soft_rc_loop(self):
        if self._soft_rc_job is not None:
            return
        self._soft_rc_tick()

    def _soft_rc_tick(self):
        self._soft_rc_job = None
        if not (self.connected and self.var_rc_on.get()):
            return                      # 停用/断开 → 循环自然结束
        self._send_rc(min_interval=0)
        self._soft_rc_job = self.root.after(
            max(20, int(1000 / tct.SOFT_RC_RATE_HZ)), self._soft_rc_tick)

    def _cancel_soft_rc_loop(self):
        if self._soft_rc_job is not None:
            try:
                self.root.after_cancel(self._soft_rc_job)
            except Exception:
                pass
            self._soft_rc_job = None

    # ---- 键盘接管 (FPS式 WASD, v1.23) ----
    #   W/S=俯仰  A/D=横滚  Q/E=偏航  Shift/Ctrl=油门升降  R=回中
    #   实机方向若相反, 改 SOFT_RC_INVERT 对应项符号

    def _soft_rc_focus_ok(self):
        """焦点在输入框/文本时不拦截按键 (防打字误触)"""
        try:
            w = self.root.focus_get()
            if w is None:
                return True
            return w.winfo_class() not in ('Entry', 'TEntry', 'Text')
        except Exception:
            return True

    def _soft_rc_key(self, keysym):
        """按当前按键集计算滑条目标值并立即发一帧.
        油门为锁存式(Shift/Ctrl 逐步增减), 俯仰/横滚/偏航按住偏转松手回中"""
        rp = tct.SOFT_RC_RP_US                       # 1500±250=1250~1750
        yw = tct.SOFT_RC_YAW_US                      # 1500±300=1200~1800
        ks = self._key_state
        inv = SOFT_RC_INVERT

        def axis(pos_key, neg_key, span, sign=1):
            v = (1 if pos_key in ks else 0) - (1 if neg_key in ks else 0)
            return int(1500 + sign * v * span)

        roll = axis('d', 'a', rp, inv.get('roll', 1))       # D右=1750
        pitch = axis('s', 'w', rp, inv.get('pitch', 1))     # S后仰=1750, W前=1250
        yaw = axis('e', 'q', yw, inv.get('yaw', 1))         # E右=1800
        if keysym in ('Shift_L', 'Shift_R'):
            self.rc_scale[2].set(
                min(2000, int(self.rc_scale[2].get()) + 40))
        elif keysym in ('Control_L', 'Control_R'):
            self.rc_scale[2].set(
                max(1000, int(self.rc_scale[2].get()) - 40))
        elif keysym == 'r':
            self.on_rc_center()
            return
        self.rc_scale[0].set(max(1000, min(2000, roll)))
        self.rc_scale[1].set(max(1000, min(2000, pitch)))
        self.rc_scale[3].set(max(1000, min(2000, yaw)))
        if self.var_rc_on.get():
            self._send_rc(force=True)

    def on_soft_rc_key_press(self, event):
        if not self.var_kb_on.get() or not self.connected:
            return
        if not self._soft_rc_focus_ok():
            return
        ks = str(event.keysym)
        if ks in ('w', 's', 'a', 'd', 'q', 'e', 'r',
                  'Shift_L', 'Shift_R', 'Control_L', 'Control_R',
                  'W', 'S', 'A', 'D', 'Q', 'E', 'R'):
            self._key_state.add(ks.lower() if len(ks) == 1 else ks)
            self._soft_rc_key(ks.lower() if len(ks) == 1 else ks)
        return 'break'

    def on_soft_rc_key_release(self, event):
        if not self._key_state:
            return
        ks = str(event.keysym)
        ks = ks.lower() if len(ks) == 1 else ks
        if ks in self._key_state:
            self._key_state.discard(ks)
            if self.var_kb_on.get() and self.connected \
                    and ks not in ('Shift_L', 'Shift_R',
                                   'Control_L', 'Control_R', 'r'):
                self._soft_rc_key(ks)

    # ---- 虚拟摇杆 (v1.23) ----

    def _build_stick(self, parent, title, canvas_id):
        cell = ttk.Frame(parent)
        cell.pack(side='left', padx=10)
        ttk.Label(cell, text=title).pack(anchor='w')
        cv = tk.Canvas(cell, width=140, height=140, bg='#1b1b1b',
                       highlightthickness=1, highlightbackground='#555')
        cv.pack()
        cv.create_line(70, 8, 70, 132, fill='#444')
        cv.create_line(8, 70, 132, 70, fill='#444')
        cv.create_oval(20, 20, 120, 120, outline='#333')
        knob = cv.create_oval(62, 62, 78, 78, fill='#3aa0ff', outline='')
        setattr(self, '_knob_' + canvas_id, knob)
        setattr(self, '_cv_' + canvas_id, cv)

        def press(ev):
            self._stick_drag = canvas_id
            self._stick_move(ev)

        def drag(ev):
            if self._stick_drag == canvas_id:
                self._stick_move(ev)

        def release(ev):
            if self._stick_drag != canvas_id:
                return
            self._stick_drag = None
            if canvas_id == 'r':       # 右摇杆松手: 横滚/俯仰回中
                self._stick['rx'] = 0.0
                self._stick['ry'] = 0.0
            else:                      # 左摇杆松手: 偏航回中, 油门保持
                self._stick['lx'] = 0.0
            self._stick_apply()

        cv.bind('<ButtonPress-1>', press)
        cv.bind('<B1-Motion>', drag)
        cv.bind('<ButtonRelease-1>', release)
        return cell

    def _stick_move(self, ev):
        canvas_id = self._stick_drag
        x = max(-1.0, min(1.0, (ev.x - 70) / 50.0))
        y = max(-1.0, min(1.0, (ev.y - 70) / 50.0))
        if canvas_id == 'l':
            self._stick['lx'] = x          # 偏航 (右=+)
            self._stick['ly'] = -y         # 油门 (画布上=+)
        else:
            self._stick['rx'] = x          # 横滚 (右=+)
            self._stick['ry'] = y          # 俯仰 (画布下=后仰=+, 与PWM同号)
        self._stick_apply()

    def _stick_apply(self):
        """摇杆值(-1..1) -> 滑条 PWM 并立即发帧; 限幅 SOFT_RC_RP_US/YAW_US"""
        rp = tct.SOFT_RC_RP_US
        yw = tct.SOFT_RC_YAW_US
        inv = SOFT_RC_INVERT
        st = self._stick
        self.rc_scale[0].set(int(1500 + inv.get('roll', 1)
                                 * st['rx'] * rp))
        self.rc_scale[1].set(int(1500 + inv.get('pitch', 1)
                                 * st['ry'] * rp))
        self.rc_scale[3].set(int(1500 + inv.get('yaw', 1)
                                 * st['lx'] * yw))
        self.rc_scale[2].set(int(1500 + st['ly'] * 500))   # 油门全量
        self._sync_sticks()
        if self.var_rc_on.get():
            self._send_rc(force=True)

    def _sync_sticks(self):
        """把摇杆画布旋钮同步到当前滑条值 (仅俯仰/横滚/偏航/油门分解显示)"""
        try:
            rp = tct.SOFT_RC_RP_US or 1
            yw = tct.SOFT_RC_YAW_US or 1
            roll = (int(self.rc_scale[0].get()) - 1500) / float(rp)
            pitch = (int(self.rc_scale[1].get()) - 1500) / float(rp)
            yaw = (int(self.rc_scale[3].get()) - 1500) / float(yw)
            thr = (int(self.rc_scale[2].get()) - 1500) / 500.0
            for cid, (x, y) in (('r', (roll, pitch)),
                                ('l', (yaw, -thr))):
                cv = getattr(self, '_cv_' + cid)
                knob = getattr(self, '_knob_' + cid)
                cx = 70 + max(-1.0, min(1.0, x)) * 50
                cy = 70 + max(-1.0, min(1.0, y)) * 50
                cv.coords(knob, cx - 8, cy - 8, cx + 8, cy + 8)
        except Exception:
            pass

    def on_soft_rc_kb_toggle(self):
        if self.var_kb_on.get():
            if not self.connected:
                self.var_kb_on.set(False)
                self._log_line('❌ 未连接, 键盘接管不可用')
                return
            self._key_state.clear()
            self._log_line('⌨️ 键盘接管已开: WASD俯仰/横滚  Q/E偏航  '
                           'Shift/Ctrl油门  R回中 (输入框内不响应)')
        else:
            self._key_state.clear()

    # ---------------- 飞控日志下载 / CSV 录制 / 报告 ----------------

    def on_log_fetch(self, only_last):
        """从飞控下载 DataFlash 日志: True=只下最后一条, False=全部"""
        p = self._need_proxy()
        if not p:
            return

        def fn():
            paths = tct.download_flash_logs(p, only_last=only_last,
                                            out_dir=CSV_DIR)
            if paths:
                self._log_line('✅ 飞控日志已下载 %d 个 → %s/'
                               % (len(paths), CSV_DIR))
            else:
                self._log_line('⚠️ 飞控日志下载失败或无日志')
            return bool(paths)
        self._enqueue('下载飞控日志(%s)' % ('最后' if only_last else '全部'),
                      fn)

    def on_toggle_record(self):
        if self.recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        if not os.path.isdir(CSV_DIR):
            try:
                os.makedirs(CSV_DIR)
            except OSError as e:
                self._log_line('❌ 创建日志目录失败: %s' % e)
                return
        self.rec_path = os.path.join(
            CSV_DIR, time.strftime('flight_%Y%m%d_%H%M%S.csv'))
        try:
            self.rec_fp = open(self.rec_path, 'w', newline='',
                               encoding='utf-8-sig')
            self.rec_wr = csv.writer(self.rec_fp)
            self.rec_wr.writerow(CSV_HEADER)
        except OSError as e:
            self._log_line('❌ 录制启动失败: %s' % e)
            self.rec_fp = None
            self.rec_wr = None
            return
        self.recording = True
        self.rec_last = 0.0
        self.btn_rec.configure(text='⏹ 停止录制')
        self._log_line('⏺ 开始录制飞行日志 %s (5Hz)' % self.rec_path)

    def _stop_record(self):
        if not self.recording and not self.rec_fp:
            return
        if self.rec_fp:
            try:
                self.rec_fp.close()
            except Exception:
                pass
        self._log_line('⏹ 录制结束 %s' % (self.rec_path or ''))
        self.rec_fp = None
        self.rec_wr = None
        self.recording = False
        try:
            self.btn_rec.configure(text='⏺ 录制飞行日志(CSV)')
        except Exception:
            pass

    def _record_row(self, now):
        if not self.recording or self.rec_wr is None:
            return
        if now - self.rec_last < 0.2:
            return
        self.rec_last = now
        with self.tel_lock:
            g = dict(self.tel)
            g['rc'] = list(self.tel['rc'])
        try:
            self.rec_wr.writerow([
                time.strftime('%Y-%m-%d %H:%M:%S'),
                g['mode'], g['armed'], g['lat'], g['lon'], g['rel_alt'],
                g['abs_alt'], g['gs'], g['vx'], g['vy'], g['vz'],
                g['roll'], g['pitch'], g['yaw'], g['heading'],
                g['volt'], g['curt'], g['remain'], g['thr'],
                g['sats'], g['fix'],
                1 if self.var_rc_on.get() else 0,
                self.cmd_stats['last'],
                round(self._dist_home, 1) if self._dist_home else '',
                self.target_alt, self.target_speed,
                g['status_text'],
            ])
        except Exception as e:
            self._log_line('❌ 录制写入失败: %s' % e)
            self._stop_record()

    def on_export_report(self):
        if not self.last_results:
            messagebox.showinfo('导出报告', '还没有测试结果, 请先运行测试。')
            return
        path = filedialog.asksaveasfilename(
            title='导出测试报告', defaultextension='.md',
            filetypes=(('Markdown', '*.md'), ('文本', '*.txt')),
            initialfile=time.strftime('apm_report_%Y%m%d_%H%M%S.md'))
        if not path:
            return
        with self.tel_lock:
            g = dict(self.tel)
        try:
            with open(path, 'w', encoding='utf-8') as fp:
                fp.write('# APM 测试报告 %s\n\n' % time.strftime(
                    '%Y-%m-%d %H:%M:%S'))
                fp.write('## 测试结果\n\n| 项目 | 结果 | 说明 |\n')
                fp.write('|---|---|---|\n')
                for k, (ok, info) in self.last_results.items():
                    fp.write('| %s | %s | %s |\n' % (
                        k, 'PASS' if ok else 'FAIL',
                        str(info).replace('|', '/')))
                fp.write('\n## 遥测快照\n\n')
                for k in ('mode', 'armed', 'lat', 'lon', 'rel_alt', 'volt',
                          'remain', 'sats', 'fix', 'status_text'):
                    fp.write('- %s: %s\n' % (k, g.get(k)))
            self._log_line('✅ 报告已导出: %s' % path)
        except OSError as e:
            self._log_line('❌ 导出失败: %s' % e)

    # ---------------- 参数读写 ----------------

    def on_speed_params_read(self):
        """读回 WPNAV_SPEED(巡航上限) + THROTTLE_HOVER(悬停油门)
        显示当前飞控侧的真实巡航能力基准, 供填巡航m/s参考"""
        p = self._need_proxy()
        if not p:
            return

        def _read(name):
            p.mav.param_request_read_send(
                p.target_system, p.target_component,
                name.encode('ascii', 'replace'), -1)
            msg = p.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg is None:
                return None
            pid = msg.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode('ascii', 'replace')
            if str(pid).split('\x00')[0] != name:
                return None
            return float(msg.param_value)

        def fn():
            spd = _read('WPNAV_SPEED')
            hov = _read('THROTTLE_HOVER')

            def show():
                parts = []
                if spd is not None:
                    parts.append('巡航上限 WPNAV_SPEED=%.0f cm/s '
                                 '(%.1f m/s)' % (spd, spd / 100.0))
                else:
                    parts.append('WPNAV_SPEED 无响应')
                if hov is not None:
                    parts.append('悬停油门 THROTTLE_HOVER=%s' % (
                        ('%.0f%%' % hov) if hov <= 100
                        else ('PWM %.0f' % hov)))
                else:
                    parts.append('THROTTLE_HOVER 无响应')
                txt = '  |  '.join(parts)
                self.var_speed_params.set(txt)
                self._log_line('  ' + txt)

            self.root.after(0, show)
            return spd is not None or hov is not None
        self._enqueue('读速度参数', fn)

    def on_param_read(self):
        p = self._need_proxy()
        if not p:
            return
        name = self.var_param_name.get().strip().upper()

        def fn():
            p.mav.param_request_read_send(
                p.target_system, p.target_component,
                name.encode('ascii', 'replace'), -1)
            msg = p.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg is None:
                print('  ❌ 参数 %s 无响应' % name)
                return False
            pid = msg.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode('ascii', 'replace')
            pid = str(pid).split('\x00')[0]
            val = msg.param_value
            print('  ✅ %s = %s (type=%s)' % (pid, val, msg.param_type))
            self.root.after(0, lambda: self._param_hist(pid, val))
            return True
        self._enqueue('读参数 %s' % name, fn)

    def on_param_write(self):
        p = self._need_proxy()
        if not p:
            return
        name = self.var_param_name.get().strip().upper()
        try:
            val = float(self.var_param_val.get())
        except ValueError:
            self._log_line('❌ 参数值无效: %r' % self.var_param_val.get())
            return

        def fn():
            ok = tct.set_param(p, name, val)
            if ok:
                self.root.after(0, lambda: self._param_hist(name, val))
            else:
                print('  ❌ 参数 %s 写入无确认' % name)
            return ok
        self._enqueue('写参数 %s=%s' % (name, val), fn)

    def _param_hist(self, name, val):
        self.param_tree.insert('', 0, values=(
            name, val, time.strftime('%H:%M:%S')))

    # ---------------- 航点任务 ----------------

    def _parse_wp_lines(self, raw):
        """解析航点文本(多行), 返回混合列表:
        绝对点 (lat, lon, alt) / 相对点 ('REL', 东米, 北米, alt) /
        抛投 ('DROP', 通道, 触发PWM, 回位PWM, 保持ms); 出错抛 ValueError.
        相对行: @东x,北y[,高], 以飞机GPS为原点, 上传/执行时换算为绝对坐标"""
        items = []
        for i, line in enumerate(raw.splitlines(), 1):
            s = line.split('#', 1)[0].strip()
            if not s:
                continue
            if s.upper().startswith('DROP') and (
                    len(s) == 4 or s[4:5] in (',', ';')):
                if not items:
                    raise ValueError('第%d行: DROP 前必须先有航点' % i)
                vals = [x.strip() for x in s.replace(';', ',').split(',')[1:]]
                if len(vals) > 4:
                    raise ValueError('第%d行: DROP 最多 4 个参数' % i)
                try:
                    sno = (int(vals[0]) if len(vals) > 0 and vals[0]
                           else int(self.var_drop_no.get()))
                    s_on = (int(vals[1]) if len(vals) > 1 and vals[1]
                            else int(self.var_drop_on.get()))
                    off_raw = (vals[2] if len(vals) > 2
                               else self.var_drop_off.get().strip())
                    s_off = int(off_raw) if str(off_raw).strip() else None
                    hold_raw = (vals[3] if len(vals) > 3
                                else self.var_drop_hold.get().strip())
                    hold_ms = int(hold_raw) if str(hold_raw).strip() else 0
                except ValueError:
                    raise ValueError(
                        '第%d行: DROP 参数需为整数 %r' % (i, line))
                if not (1 <= sno <= 16):
                    raise ValueError('第%d行: 通道超范围(1~16): %d'
                                     % (i, sno))
                for tag, v in (('触发', s_on), ('回位', s_off)):
                    if v is not None and not (500 <= v <= 2500):
                        raise ValueError(
                            '第%d行: %s PWM 超范围(500~2500): %d'
                            % (i, tag, v))
                if not (0 <= hold_ms <= 60000):
                    raise ValueError(
                        '第%d行: 保持时间范围 0~60000 ms: %d'
                        % (i, hold_ms))
                items.append(('DROP', sno, s_on, s_off, hold_ms))
                continue
            if s.startswith('@'):
                parts = [x.strip() for x in s[1:].replace(';', ',').split(',')]
                if len(parts) < 2 or len(parts) > 3:
                    raise ValueError('第%d行: 相对行格式 @东米,北米[,高]'
                                     % i)
                try:
                    dx = float(parts[0])
                    dy = float(parts[1])
                    alt = float(parts[2]) if len(parts) > 2 else 20.0
                except ValueError:
                    raise ValueError('第%d行: 数值无效 %r' % (i, line))
                if abs(dx) > 100000 or abs(dy) > 100000:
                    raise ValueError('第%d行: 偏移超范围(±100000m)'
                                     % i)
                if alt <= 0:
                    raise ValueError('第%d行: 高度必须>0' % i)
                items.append(('REL', dx, dy, alt))
                continue
            parts = [x.strip() for x in s.replace(';', ',').split(',')]
            if len(parts) < 2:
                raise ValueError('第%d行: 需要 纬度,经度[,高度]' % i)
            try:
                lat = float(parts[0])
                lon = float(parts[1])
                alt = float(parts[2]) if len(parts) > 2 else 20.0
            except ValueError:
                raise ValueError('第%d行: 数值无效 %r' % (i, line))
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError('第%d行: 经纬度超范围' % i)
            if alt <= 0:
                raise ValueError('第%d行: 高度必须>0' % i)
            items.append((lat, lon, alt))
        if not [x for x in items if not apc._is_drop(x)]:
            raise ValueError('没有航点')
        return items

    def _parse_wp_text(self):
        """解析文本框航点(语法见 _parse_wp_lines)"""
        return self._parse_wp_lines(self.txt_wp.get('1.0', 'end'))

    @staticmethod
    def _resolve_rel(items, olat, olon):
        """相对点 ('REL',...) 换算为绝对 (lat, lon, alt); olat/olon=原点
        (飞机GPS), 无效抛 ValueError; 其余条目原样保留"""
        if olat is None or olon is None:
            raise ValueError('取不到飞机GPS位置, 无法换算相对航点')
        out = []
        for it in items:
            if isinstance(it, (tuple, list)) and it and it[0] == 'REL':
                _, dx, dy, alt = it
                lat = olat + dy / 111319.49
                lon = olon + dx / (111319.49 *
                                   math.cos(math.radians(olat)))
                out.append((lat, lon, alt))
            else:
                out.append(it)
        return out

    def _gps_origin(self):
        """遥测里的飞机经纬度; 未连接/无GPS时为 (None, None)"""
        with self.tel_lock:
            return self.tel.get('lat'), self.tel.get('lon')

    @staticmethod
    def _fresh_pos(p, timeout=3.0):
        """取一帧新鲜 GLOBAL_POSITION_INT → (lat, lon, rel_alt) 或 None"""
        m = p.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                         timeout=timeout)
        if m is None:
            return None
        return m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0

    def _resolve_items(self, p, items):
        """执行前把相对点换算为绝对点(取新鲜飞机GPS).
        无相对点原样返回; 失败记日志并返回 None"""
        if not any(isinstance(x, (tuple, list)) and x and x[0] == 'REL'
                   for x in items):
            return items
        pos = self._fresh_pos(p)
        if pos is None:
            self._log_line('❌ 取不到飞机GPS, 相对航点无法换算')
            return None
        try:
            return self._resolve_rel(items, pos[0], pos[1])
        except ValueError as e:
            self._log_line('❌ 相对航点换算失败: %s' % e)
            return None

    def on_wp_check(self):
        try:
            items = self._parse_wp_text()
        except ValueError as e:
            self.var_wp_st.set('校验失败: %s' % e)
            self._log_line('❌ 航点校验失败: %s' % e)
            return
        wps = [x for x in items if not apc._is_drop(x)]
        n_drop = len(items) - len(wps)
        n_rel = sum(1 for x in wps if x[0] == 'REL')
        d = None
        if n_rel:
            # 相对点: 有飞机GPS就换算并算航线长, 否则留到执行时算
            try:
                olat, olon = self._gps_origin()
                rw = [x for x in self._resolve_rel(items, olat, olon)
                      if not apc._is_drop(x)]
                d = sum(_haversine(rw[i][0], rw[i][1],
                                   rw[i + 1][0], rw[i + 1][1])
                        for i in range(len(rw) - 1))
            except ValueError:
                pass
        else:
            d = sum(_haversine(wps[i][0], wps[i][1], wps[i + 1][0],
                               wps[i + 1][1]) for i in range(len(wps) - 1))
        if d is not None:
            self.var_wp_st.set('校验通过: %d 个航点%s, %d 个抛投点, '
                               '航线总长 %.0f m'
                               % (len(wps),
                                  ' (含%d相对,按飞机GPS换算)' % n_rel
                                  if n_rel else '', n_drop, d))
        else:
            self.var_wp_st.set('校验通过: %d 个航点(含%d相对,执行时按飞机'
                               'GPS换算), %d 个抛投点'
                               % (len(wps), n_rel, n_drop))
        self._log_line('✅ ' + self.var_wp_st.get())

    def on_wp_load(self):
        for cand in ('waypoints.txt', 'mission.txt', 'waypoints.csv'):
            if os.path.isfile(cand):
                try:
                    with open(cand, 'r', encoding='utf-8') as fp:
                        data = fp.read()
                except OSError as e:
                    self._log_line('❌ 读取 %s 失败: %s' % (cand, e))
                    continue
                self.txt_wp.delete('1.0', 'end')
                self.txt_wp.insert('1.0', data)
                self.var_wp_st.set('已加载 %s' % cand)
                self.on_wp_check()
                return
        self.var_wp_st.set('当前目录未找到 waypoints.txt / mission.txt')

    def on_wp_upload(self):
        p = self._need_proxy()
        if not p:
            return
        try:
            items = self._parse_wp_text()
        except ValueError as e:
            self._log_line('❌ 航点无效: %s' % e)
            return
        n_wp = sum(1 for x in items if not apc._is_drop(x))
        n_drop = len(items) - n_wp

        def fn():
            items2 = self._resolve_items(p, items)
            if items2 is None:
                return False
            return apc.upload_mission(p, items2)
        self._enqueue('上传任务(%d航点, %d 抛投)' % (n_wp, n_drop), fn)

    def on_wp_run(self):
        p = self._need_proxy()
        if not p:
            return
        if not self._apply_params():
            return
        try:
            items = self._parse_wp_text()
        except ValueError as e:
            self._log_line('❌ 航点无效: %s' % e)
            return
        wps = [x for x in items if not apc._is_drop(x)]
        n_drop = len(items) - len(wps)
        # REL 元组 ('REL',dx,dy,alt) 高度在 [3], 绝对点在 [2]; 精确值在
        # fn 内换算后重算
        max_alt = max((w[3] if w[0] == 'REL' else w[2]) for w in wps)
        takeoff_alt = max(tct.TAKEOFF_ALT, max_alt)

        def fn():
            items2 = self._resolve_items(p, items)
            if items2 is None:
                return False
            wps2 = [x for x in items2 if not apc._is_drop(x)]
            takeoff_alt2 = max(tct.TAKEOFF_ALT, max(w[2] for w in wps2))
            dist = sum(_haversine(wps2[i][0], wps2[i][1],
                                  wps2[i + 1][0], wps2[i + 1][1])
                       for i in range(len(wps2) - 1))
            n_items = apc.upload_mission(p, items2)
            if not n_items:
                print('  ❌ 任务上传失败')
                return False
            if not tct.arm_and_takeoff(p, takeoff_alt2):
                print('  ❌ 起飞失败, 任务取消')
                return False
            self.target_alt = takeoff_alt2
            apc.reset_mission_to_start(p)
            try:
                apc.set_mode(p, 'AUTO')
            except SystemExit as e:
                print('  ❌ 切 AUTO 失败: %s' % e)
                return False
            timeout = 120 + 30 * len(wps2) + dist
            print('  任务超时预算 %d 秒 (航线 %.0f m + 每航点 30s)'
                  % (int(timeout), dist))
            return apc.wait_auto_mission(
                p, int(n_items), 'rtl', timeout=timeout)
        self._enqueue('执行任务(%d航点, %d 抛投, 起飞%.0fm)'
                      % (len(wps), n_drop, takeoff_alt),
                      fn)

    def _hint_takeover(self):
        self._log_line('ℹ️ GUIDED 下摇杆无效属正常; 接管: 按 F9 切 LOITER, '
                       '或软遥控页[一键接管], 或拨 TX 模式开关')

    def on_goto_point(self, need_takeoff=True):
        """单点飞行: 目的地(纬度,经度 或 @东x,北y 相对飞机GPS) -> [解锁起飞]
        -> GUIDED 飞往 -> 悬停. 反复发 goto(1Hz)直到水平<2.5m/垂直<1.5m
        支持飞行中立即改点: 再次点击按钮直接更新目标, 不排队"""
        p = self._need_proxy()
        if not p:
            return
        dest = self.var_dest.get().strip()
        if not dest:
            self._log_line('❌ 请输入目的地: 纬度,经度 或 @东x,北y')
            return
        try:
            alt = float(self.var_dest_alt.get().strip())
        except ValueError:
            self._log_line('❌ 目的地高度无效')
            return
        if alt <= 0:
            self._log_line('❌ 高度必须>0')
            return
        rel = dest.startswith('@')
        body = dest[1:] if rel else dest
        parts = [x.strip() for x in body.replace(';', ',').split(',')]
        if len(parts) != 2:
            self._log_line('❌ 目的地格式: 纬度,经度 或 @东x,北y '
                           '(高度用右侧框)')
            return
        try:
            a = float(parts[0])
            b = float(parts[1])
        except ValueError:
            self._log_line('❌ 目的地数值无效: %r' % dest)
            return
        if rel:
            if abs(a) > 100000 or abs(b) > 100000:
                self._log_line('❌ 偏移超范围(±100000m)')
                return
        elif not (-90 <= a <= 90 and -180 <= b <= 180):
            self._log_line('❌ 经纬度超范围')
            return
        cruise = self.var_cruise.get().strip()
        if cruise:
            try:
                cruise_v = float(cruise)
            except ValueError:
                self._log_line('❌ 巡航速度无效: %r' % cruise)
                return
            if not (0.2 <= cruise_v <= 20.0):
                self._log_line('❌ 巡航速度范围 0.2~20 m/s '
                               '(对应 WPNAV_SPEED 20~2000 cm/s)')
                return
        else:
            cruise_v = 0.0

        # 构造 spec 原子写入 (含 epoch 用于 fn 内比对)
        self._goto_epoch += 1
        epoch = self._goto_epoch
        self._goto_spec = {
            'e': epoch,
            'rel': rel, 'a': a, 'b': b, 'alt': alt,
            'cruise_v': cruise_v, 'need_takeoff': need_takeoff
        }
        # 若已有 goto 任务在跑 -> 立即改点, 不入队
        if self._goto_active.is_set():
            self._log_line('✈ 立即改点 → %s (epoch=%d)'
                           % ('相对' if rel else '绝对', epoch))
            return

        def resolve_spec(spec, p):
            """按当前 spec 换算出绝对 lat/lon/alt; rel 坐标每次改点重算一次"""
            if spec['rel']:
                pos = self._fresh_pos(p)
                if pos is None:
                    return None, None, None
                lat = pos[0] + spec['b'] / 111319.49
                lon = pos[1] + spec['a'] / (111319.49 *
                                            math.cos(math.radians(pos[0])))
            else:
                lat, lon = spec['a'], spec['b']
            return lat, lon, spec['alt']

        def fn():
            self._goto_active.set()
            my_epoch = -1
            lat = lon = None
            try:
                # 初始解析
                spec = self._goto_spec
                my_epoch = spec['e']
                lat, lon, alt = resolve_spec(spec, p)
                if lat is None:
                    print('  ❌ 取不到飞机GPS, 相对目的地无法换算')
                    return False
                print('  目的地: %.6f, %.6f  高 %.1fm (%s)' % (
                    lat, lon, alt,
                    '按飞机GPS换算' if spec['rel'] else '绝对坐标'))
                if not p.motors_armed():
                    if not spec['need_takeoff']:
                        print('  ❌ 飞机未解锁, 请用"起飞并飞往"')
                        return False
                    to_alt = max(alt, tct.TAKEOFF_ALT)
                    if not tct.arm_and_takeoff(p, to_alt, mode='GUIDED',
                                               arm_timeout=10,
                                               takeoff_timeout=45,
                                               set_delay=True):
                        print('  ❌ 起飞失败, 单点飞行取消')
                        return False
                    self.target_alt = to_alt
                    self._hint_takeover()
                else:
                    p.set_mode_apm('GUIDED')
                    self._log_line('  已确保 GUIDED, 飞往目的地...')
                self.target_alt = max(self.target_alt or 0.0, alt)
                if spec['cruise_v'] > 0:
                    # v1.30: 不写 WPNAV_SPEED 参数, 改发 DO_CHANGE_SPEED 指令
                    # (只对本次飞行生效, 重启/上锁后回到飞控参数值)
                    print('  巡航速度 %.1f m/s: 发 DO_CHANGE_SPEED 指令 '
                          '(不写飞控参数)...' % spec['cruise_v'])
                    if not tct.send_change_speed(p, spec['cruise_v'],
                                                 timeout=2.5):
                        print('  ⚠️ 速度指令未被接受, 按飞控现有 WPNAV_SPEED '
                              '速度飞行; 需永久生效请在参数页手动写入')
                # 进入 goto 循环
                pos0 = self._fresh_pos(p) or (lat, lon, alt)
                d0 = _haversine(pos0[0], pos0[1], lat, lon)
                timeout = max(60.0, 30.0 + d0 / 2.0)
                print('  飞往 %.1fm 外, 超时 %.0fs ...' % (d0, timeout))
                t_end = time.time() + timeout
                last_goto = 0.0
                while time.time() < t_end:
                    now = time.time()
                    # 检查是否有新的改点
                    if self._goto_epoch != my_epoch:
                        spec = self._goto_spec
                        my_epoch = spec['e']
                        lat2, lon2, alt2 = resolve_spec(spec, p)
                        if lat2 is not None:
                            lat, lon, alt = lat2, lon2, alt2
                            pos0 = self._fresh_pos(p) or (lat, lon, alt)
                            d0 = _haversine(pos0[0], pos0[1], lat, lon)
                            timeout = max(60.0, 30.0 + d0 / 2.0)
                            t_end = time.time() + timeout
                            print('✈ 改点 → %.6f, %.6f  高 %.1fm (新超时 %.0fs)'
                                  % (lat, lon, alt, timeout))
                    if now - last_goto >= 1.0:
                        apc.goto(p, lat, lon, alt)
                        last_goto = now
                    m = p.recv_match(type='GLOBAL_POSITION_INT',
                                     blocking=True, timeout=0.5)
                    if m is None:
                        continue
                    clat, clon = m.lat / 1e7, m.lon / 1e7
                    rela = m.relative_alt / 1000.0
                    d = _haversine(clat, clon, lat, lon)
                    if d < 2.5 and abs(rela - alt) < 1.5:
                        print('  ✅ 已到达目的地 (水平差 %.1fm)' % d)
                        self._hint_takeover()
                        # 到达前再查一次 epoch, 避免"刚到旧点就丢新点"
                        if self._goto_epoch != my_epoch:
                            continue
                        return True
                print('  ❌ 到达超时 (%.0fs), 飞机保持 GUIDED 悬停' % timeout)
                self._hint_takeover()
                return False
            finally:
                self._goto_active.clear()
        self._enqueue('单点飞行(GUIDED, %s)' % ('相对' if rel else '绝对'), fn)

    # ---------------- 绘制 ----------------

    def _draw_attitude(self):
        c = self.att_canvas
        c.delete('all')
        w = int(c['width'])
        h = int(c['height'])
        with self.tel_lock:
            roll, pitch = self.tel['roll'], self.tel['pitch']
        if roll is None or pitch is None:
            c.create_text(w // 2, h // 2, text='ATTITUDE 无数据',
                          fill='#666', font=('Microsoft YaHei', 9))
            return
        cx, cy, r = w / 2.0, h / 2.0, min(w, h) / 2.0 - 8
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      outline='#555', fill='#111')
        # 地平线: 按 pitch 上下移, 绕中心按 roll 旋转
        dy = max(-r, min(r, pitch * 2.4))
        rad = math.radians(-roll)
        def rot(x, y):
            dx, dyy = x - cx, y - cy
            return (cx + dx * math.cos(rad) - dyy * math.sin(rad),
                    cy + dx * math.sin(rad) + dyy * math.cos(rad))
        p1 = rot(cx - r * 3, cy + dy)
        p2 = rot(cx + r * 3, cy + dy)
        c.create_line(p1[0], p1[1], p2[0], p2[1], fill='#ffd166', width=2)
        c.create_line(cx - 20, cy, cx - 7, cy, fill='#0f0', width=2)
        c.create_line(cx + 7, cy, cx + 20, cy, fill='#0f0', width=2)
        c.create_text(cx, 10, text='R %+.0f  P %+.0f' % (roll, pitch),
                      fill='#0f0', font=('Consolas', 9, 'bold'))

    def _draw_curve(self, cv, data, unit):
        cv.delete('all')
        w = cv.winfo_width() or 500
        h = cv.winfo_height() or 160
        if w < 20 or h < 20:
            return
        pts = list(data)
        if len(pts) < 2:
            cv.create_text(w // 2, h // 2, text='等待数据 ...',
                           fill='#666', font=('Microsoft YaHei', 9))
            return
        allv = [v for _, a, b in pts for v in (a, b) if v is not None]
        if not allv:
            return
        lo, hi = min(allv), max(allv)
        if hi - lo < 1.0:
            hi = lo + 1.0
        def xy(i, v):
            return (4 + (w - 8) * i / (len(pts) - 1),
                    h - 6 - (h - 12) * (v - lo) / (hi - lo))
        def line_of(idx):
            return [c for i, row in enumerate(pts)
                    for c in xy(i, row[idx]) if row[idx] is not None]
        a = line_of(1)
        if len(a) >= 4:
            cv.create_line(*a, fill='#4ecdc4', width=2)
        b = line_of(2)
        if len(b) >= 4:
            cv.create_line(*b, fill='#ff6b6b', width=2, dash=(6, 4))
        act = pts[-1][1]
        tgt = pts[-1][2]
        cv.create_text(6, 8, anchor='w', fill='#4ecdc4',
                       font=('Consolas', 9, 'bold'),
                       text='实际 %s%s' % (('%.1f' % act) if act is not None
                                           else '--', unit))
        cv.create_text(w - 6, 8, anchor='e', fill='#ff6b6b',
                       font=('Consolas', 9, 'bold'),
                       text='目标 %s%s' % (('%.1f' % tgt) if tgt is not None
                                           else '--', unit))
        cv.create_text(6, h - 8, anchor='w', fill='#888',
                       font=('Consolas', 8),
                       text='范围 %.1f ~ %.1f%s' % (lo, hi, unit))

    # ---------------- 小地图 (高德瓦片) ----------------

    def _map_zoom(self, delta):
        z = max(MAP_MIN_ZOOM, min(MAP_MAX_ZOOM, self.map_zoom + delta))
        if z != self.map_zoom:
            self.map_zoom = z
        self.var_map_z.set('z%d' % self.map_zoom)
        self._draw_map(force=True)

    def _map_home(self):
        """中心回到 HOME; 无 HOME 则回到当前位置"""
        with self.tel_lock:
            lat, lon = self.tel['lat'], self.tel['lon']
        if self.home:
            self.map_center = self._to_map(self.home['lat'],
                                           self.home['lon'])
            self.var_map_follow.set(False)   # 看HOME, 不被飞机拽走
        elif lat is not None:
            self.map_center = self._to_map(lat, lon)
            self.var_map_follow.set(True)
        else:
            self.map_center = self._to_map(*DEFAULT_MAP_CENTER)
            self.var_map_follow.set(False)
        self._draw_map(force=True)

    def _to_map(self, lat, lon):
        """WGS-84(GPS) -> 当前底图坐标系; 仅高德需转 GCJ-02, 其余恒等"""
        if lat is None or lon is None:
            return None
        if MAP_SOURCES[self.map_src]['gcj']:
            return wgs84_to_gcj02(lat, lon)
        return lat, lon

    def _map_current_key(self):
        lbl = self.var_map_src.get()
        for k, cfg in MAP_SOURCES.items():
            if cfg['label'] == lbl:
                return k
        return 'esri'

    def _map_source_changed(self):
        key = self._map_current_key()
        self.map_src = key
        self._map_imgs.clear()
        self.map_center = None
        self._tile_store = TileStore(key,
                                     token=self.var_map_tk.get().strip())
        self.var_map_st.set('底图切换: %s' % MAP_SOURCES[key]['label'])
        self._log_line('底图切换为 %s' % MAP_SOURCES[key]['label'])
        self._draw_map(force=True)

    def _map_apply_token(self):
        token = self.var_map_tk.get().strip()
        try:
            with open(TIANDITU_KEY_FILE, 'w', encoding='utf-8') as fp:
                fp.write(token)
        except OSError as e:
            self._log_line('❌ 密钥保存失败: %s' % e)
        self._map_imgs.clear()
        self._tile_store = TileStore(self.map_src, token=token)
        self.var_map_st.set('天地图密钥已%s' % ('更新' if token else '清空'))
        self._log_line('天地图密钥已%s (%s)' % (
            '更新' if token else '清空', TIANDITU_HELP))
        self._draw_map(force=True)

    def _on_map_wheel(self, event):
        d = event.delta
        if d:
            self._map_zoom(1 if d > 0 else -1)

    def _on_map_down(self, event):
        self._map_drag = (event.x, event.y)

    def _on_map_move(self, event):
        if self._map_drag is None:
            return
        dx = event.x - self._map_drag[0]
        dy = event.y - self._map_drag[1]
        self._map_drag = (event.x, event.y)
        if not dx and not dy:
            return
        # 拖动地图 -> 中心反向移动, 同时退出跟随
        self.var_map_follow.set(False)
        clat, clon = self.map_center or DEFAULT_MAP_CENTER
        fx, fy = _latlon_to_tilef(clat, clon, self.map_zoom)
        fx -= dx / 256.0
        fy -= dy / 256.0
        n = 1 << self.map_zoom
        fy = max(0.0, min(n - 1e-6, fy))
        self.map_center = _tilef_to_latlon(fx, fy, self.map_zoom)
        self._draw_map(force=True)

    def _draw_map(self, force=False):
        """高德瓦片底图 + HOME/航迹/当前位置叠加; 无 GPS 也会立即出图"""
        now = time.time()
        tile_new = self._tile_store.new_ok.is_set()
        if tile_new:
            self._tile_store.new_ok.clear()
        if not force and not tile_new and \
                now - self._map_last_draw < 0.4:
            return
        c = self.map_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 40 or h < 40:
            return
        self._map_last_draw = now
        c.delete('all')

        if self._tile_store.need_key:
            c.create_text(w // 2, h // 2, justify='center', fill='#ffd166',
                          font=('Microsoft YaHei', 11),
                          text='当前底图需要天地图密钥\n\n'
                               '请输入 tk 并点 [应用]\n' + TIANDITU_HELP)
            self.var_map_st.set('天地图密钥未设置 — ' + TIANDITU_HELP)
            return

        with self.tel_lock:
            lat, lon = self.tel['lat'], self.tel['lon']
        mpos = self._to_map(lat, lon) if lat is not None else None

        # --- 中心: 跟随飞机 > HOME > 当前默认 > 北京 ---
        if mpos and self.var_map_follow.get():
            self.map_center = mpos
        elif self.map_center is None:
            if self.home:
                self.map_center = self._to_map(self.home['lat'],
                                               self.home['lon'])
            else:
                self.map_center = self._to_map(*DEFAULT_MAP_CENTER)

        z = self.map_zoom
        clat, clon = self.map_center
        fx0, fy0 = _latlon_to_tilef(clat, clon, z)
        # 中心对齐到画布正中
        fx0 -= (w / 2.0) / 256.0
        fy0 -= (h / 2.0) / 256.0
        x_first = max(0, int(math.floor(fx0)))
        y_first = max(0, int(math.floor(fy0)))
        x_last = min((1 << z) - 1, int(math.floor(fx0 + w / 256.0)))
        y_last = min((1 << z) - 1, int(math.floor(fy0 + h / 256.0)))

        # --- 铺瓦片 ---
        loaded = 0
        pending = 0
        for tx in range(x_first, x_last + 1):
            for ty in range(y_first, y_last + 1):
                sx = (tx - fx0) * 256.0
                sy = (ty - fy0) * 256.0
                key = (z, tx, ty)
                img = self._map_imgs.get(key)
                if img is None:
                    path = self._tile_store.request(z, tx, ty)
                    if path:
                        img = TileStore.load_photo(
                            path, self._tile_store.cfg['ext'])
                        if img is not None:
                            self._map_imgs[key] = img
                            self._map_imgs.move_to_end(key)
                            loaded += 1
                    else:
                        pending += 1
                if img is not None:
                    c.create_image(sx, sy, anchor='nw', image=img)
                else:
                    c.create_rectangle(sx, sy, sx + 255, sy + 255,
                                       outline='#12301c', fill='#0b1a12')
        # 内存裁剪
        while len(self._map_imgs) > MAP_IMG_CACHE:
            self._map_imgs.popitem(last=False)

        def proj(lat_, lon_):
            fx, fy = _latlon_to_tilef(*self._to_map(lat_, lon_), z=z)
            return (fx - fx0) * 256.0, (fy - fy0) * 256.0

        # --- 航迹 ---
        if len(self.track) >= 2:
            flat = []
            for la_, lo_ in self.track:
                x_, y_ = proj(la_, lo_)
                flat.extend((x_, y_))
            c.create_line(*flat, fill='#3388ff', width=2)

        # --- HOME ---
        if self.home:
            hx, hy = proj(self.home['lat'], self.home['lon'])
            c.create_rectangle(hx - 7, hy - 7, hx + 7, hy + 7,
                               outline='#ffd166', width=2)
            c.create_text(hx + 10, hy - 8, anchor='w', text='HOME',
                          fill='#ffd166', font=('Consolas', 10, 'bold'))

        # --- 当前位置 ---
        if lat is not None:
            px, py = proj(lat, lon)
            c.create_oval(px - 6, py - 6, px + 6, py + 6,
                          outline='#ffffff', fill='#ff4d4d', width=2)
            c.create_text(px + 9, py + 8, anchor='w', text='机',
                          fill='#ff8080',
                          font=('Microsoft YaHei', 10, 'bold'))
            c.create_line(px, py - 14, px, py + 14, fill='#ff4d4d')
            c.create_line(px - 14, py, px + 14, py, fill='#ff4d4d')

        # --- 比例尺 ---
        mpp = 156543.03392 * math.cos(math.radians(clat)) / (2 ** z)
        bar_m = mpp * 80.0
        k = 10 ** math.floor(math.log10(max(bar_m, 1e-9)))
        nice = min((m * k for m in (1, 2, 5, 10)), key=lambda v:
                   abs(v - bar_m))
        px_len = nice / mpp
        c.create_rectangle(8, h - 22, 8 + px_len, h - 10,
                           fill='#000000', outline='#ffffff')
        c.create_text(12 + px_len, h - 16, anchor='w',
                      text='%d m' % nice, fill='#ffffff',
                      font=('Consolas', 9))

        # --- 状态栏 ---
        crs = 'GCJ02' if MAP_SOURCES[self.map_src]['gcj'] else 'WGS84'
        pos = ('%.6f, %.6f (WGS84)' % (lat, lon)) if lat is not None \
            else '无GPS'
        hint = ''
        if self.map_src == 'esri' and not HAVE_PIL:
            hint = '  ⚠缺Pillow: pip install pillow'
        self.var_map_st.set(
            '%s  z%d  中心 %.6f,%.6f(%s)  %s  瓦片 新载%d/待下%d  '
            '缓存%d  下载OK%d 失败%d%s' % (
                MAP_SOURCES[self.map_src]['label'], z, clat, clon, crs,
                pos, loaded, pending,
                len(self._map_imgs),
                self._tile_store.stats['ok'],
                self._tile_store.stats['fail'], hint))

    def _update_msg_tree(self, now):
        if now - self._msg_tree_t < 1.0:
            return
        self._msg_tree_t = now
        with self.msg_lock:
            rates = dict(self.msg_rates)
        try:
            kids = self.tree_msg.get_children()
            if kids:
                self.tree_msg.delete(*kids)
        except Exception:
            return
        for name, hz in sorted(rates.items(), key=lambda kv: -kv[1])[:40]:
            self.tree_msg.insert('', 'end', values=(name, '%.1f' % hz))

    # ---------------- GUI 定时刷新 ----------------

    def _fmt(self, v, unit='', d=1):
        if v is None:
            return '--'
        if isinstance(v, float):
            return ('%.*f%s' % (d, v, unit))
        return '%s%s' % (v, unit)

    def _tick(self):
        """主线程刷新循环外层: 任何异常都不能中断 after 链, 否则界面会假死."""
        self._tick_seq += 1
        try:
            self._tick_body()
        except Exception as e:
            try:
                self._log_line('❌ UI刷新异常(已忽略): %s: %s'
                               % (type(e).__name__, e))
            except Exception:
                pass
            traceback.print_exc()
        finally:
            self.root.after(TELEM_PERIOD_MS, self._tick)

    def _tick_body(self):
        # 日志
        n = 0
        while n < 400:
            try:
                line = self.log_q.get_nowait()
            except queue.Empty:
                break
            self._log_line(line)
            n += 1

        # 遥测
        with self.tel_lock:
            t = dict(self.tel)
            t['rc'] = list(t['rc'])
        with self.cmd_stats['lock']:
            counts = self.cmd_stats['counts']
            for k, c in counts.items():
                self._acc_counts[k] = self._acc_counts.get(k, 0) + c
            self.cmd_stats['counts'] = {}
            last_cmd = self.cmd_stats['last']
            rc_ov = self.cmd_stats['rc_override']
        self.var_task.set(self.task_name)

        L = self.tel_labels
        L['mode'].set(t['mode'] or '--')
        if t['armed'] is True:
            L['armed'].set('已解锁 ARMED')
        elif t['armed'] is False:
            L['armed'].set('未解锁 DISARMED')
        else:
            L['armed'].set('--')
        L['sys'].set(SYS_NAMES.get(t['sys_status'], str(t['sys_status'])))
        if t['hb_t'] > 0:
            age = time.time() - t['hb_t']
            L['hb'].set('%.1fs 前' % age)
        else:
            L['hb'].set('--')

        fix_txt = FIX_NAMES.get(t['fix'], str(t['fix']))
        if isinstance(t['fix'], int) and t['fix'] < 3 and t['armed'] is True:
            fix_txt += '·惯导继续'      # v1.21: 空中GPS失联仅提示, 不动作
        L['gps'].set('%s  卫星 %s  HDOP %s' % (
            fix_txt,
            self._fmt(t['sats'], '', 0),
            self._fmt(t['hdop'], '', 1)))
        if t['lat'] is not None:
            L['pos'].set('%.6f, %.6f' % (t['lat'], t['lon']))
        else:
            L['pos'].set('--')
        L['alt'].set(self._fmt(t['rel_alt'], ' m', 2))
        L['absalt'].set(self._fmt(t['abs_alt'], ' m', 2))
        L['vel'].set('N %s  E %s  D %s' % (
            self._fmt(t['vx'], '', 2), self._fmt(t['vy'], '', 2),
            self._fmt(t['vz'], '', 2)))
        L['gs'].set('地 %s  空 %s  油门 %s' % (
            self._fmt(t['gs'], '', 1), self._fmt(t['as'], '', 1),
            self._fmt(t['thr'], '%', 0)))
        L['hdg'].set('航向 %s  爬升 %s' % (
            self._fmt(t['heading'], '°', 0), self._fmt(t['climb'], '', 1)))
        L['rpy'].set('%s / %s / %s' % (
            self._fmt(t['roll'], '', 1), self._fmt(t['pitch'], '', 1),
            self._fmt(t['yaw'], '', 1)))
        bat = 'V %s  A %s  %s%%' % (
            self._fmt(t['volt'], '', 2),
            self._fmt(t['curt'], '', 1),
            self._fmt(t['remain'], '', 0))
        L['bat'].set(bat)
        L['load'].set(self._fmt(t['load'], '%', 1))
        if t['ekf']:
            L['ekf'].set('%.3f / %.3f / %.3f' % t['ekf'])
        else:
            L['ekf'].set('--')
        # v1.29.3: 位置估计就绪状态 (与解锁预检门禁共用同一判据)
        pr_txt, pr_style = self._pos_ready_ui(t)
        if L.get('posrdy') is not None:
            L['posrdy'].set(pr_txt)
        lbl_pr = getattr(self, 'lbl_posrdy', None)
        if lbl_pr is not None:
            lbl_pr.configure(style=pr_style)
        # v1.30: EKF 监控线程结论 (线程只判定, 安全条由主线程统一渲染)
        ev = self.ekf_verdict
        if ev is not None and getattr(self, 'lbl_ekf_st', None) is not None:
            if self.var_ekf_st.get() != ev['text']:
                self.var_ekf_st.set(ev['text'])
            if self.lbl_ekf_st.cget('style') != ev['style']:
                self.lbl_ekf_st.configure(style=ev['style'])
        L['st'].set(t['status_text'][:28])
        L['rcov'].set('杆值覆盖中' if rc_ov else '未覆盖')
        if self.var_rc_mode is not None:
            m = (t.get('mode') or '--').upper()
            manual = m in tct.SOFT_RC_MANUAL_MODES
            self.var_rc_mode.set('模式: %s %s' % (
                m, '✓手动(摇杆有效)' if manual else '✗非手动(摇杆无效)'))
        rc = t['rc']
        L['rc'].set(' '.join('--' if x is None else str(x) for x in rc[:4]))
        L['rssi'].set(self._fmt(t['rssi'], '', 0))
        L['cmd'].set(last_cmd)
        L['link'].set('%d' % t['msgs'])

        # HOME / 离HOME距离
        if self.home:
            L['home'].set('%.6f, %.6f' % (self.home['lat'],
                                          self.home['lon']))
        else:
            L['home'].set('--')
        if self.home and t['lat'] is not None:
            self._dist_home = _haversine(self.home['lat'],
                                         self.home['lon'],
                                         t['lat'], t['lon'])
            L['dist'].set('%.1f m' % self._dist_home)
        else:
            self._dist_home = None
            L['dist'].set('--')

        now = time.time()

        # 目标跟踪采样 (仅飞行中)
        if t['armed'] and t['rel_alt'] is not None:
            self.curve_alt.append((now, t['rel_alt'], self.target_alt))
        if t['armed'] and t['gs'] is not None:
            self.curve_spd.append((now, t['gs'], self.target_speed))

        # 解锁->上锁 时清除目标
        if self._was_armed and not t['armed']:
            self.target_alt = None
            self.target_speed = None
        self._was_armed = bool(t['armed'])

        # 震动数据采样 (1Hz)
        self._vib_tick += 1
        if self._vib_tick >= 1000 // TELEM_PERIOD_MS:  # ~1Hz
            self._vib_tick = 0
            vib = t.get('vib')
            if vib is not None:
                vx, vy, vz = vib
                for axis, val in zip(('x', 'y', 'z'), (vx, vy, vz)):
                    arr = self.vib_data[axis]
                    arr.append(val)
                    if len(arr) > self.vib_max_points:
                        arr.pop(0)
                # 更新当前值标签
                self.vib_labels['x'].set('%.2f' % vx)
                self.vib_labels['y'].set('%.2f' % vy)
                self.vib_labels['z'].set('%.2f' % vz)
                self._draw_vibration()

        # 航迹采样 (0.5m 或 1s 一个点)
        if t['armed'] and t['lat'] is not None:
            moved = True
            if self._track_last_pos is not None:
                d = _haversine(self._track_last_pos[0],
                               self._track_last_pos[1],
                               t['lat'], t['lon'])
                moved = d > 0.5 or now - self._track_last_t > 1.0
            if moved:
                self.track.append((t['lat'], t['lon']))
                self._track_last_pos = (t['lat'], t['lon'])
                self._track_last_t = now

        # 安全监控 / RC覆盖 / 录制
        self._check_safety(now)
        self._send_rc()
        self._record_row(now)

        # 链路指示
        if self.connected and t['link_ok']:
            self.var_link.set('● 已连接 (实时)')
        elif self.connected:
            self.var_link.set('○ 已连接 (无数据)')
        if t['armed']:
            self.lbl_armed.configure(style='Red.TLabel')
        else:
            self.lbl_armed.configure(style='Green.TLabel')

        # 指令频率表 (按 ~1s 窗口)
        now = time.time()
        if now - self._rate_prev_t >= 0.95:
            dt = now - self._rate_prev_t
            rows = sorted(((k, c / dt) for k, c in self._acc_counts.items()),
                          key=lambda x: -x[1])
            self._acc_counts = {}
            self._rate_prev_t = now
            old = {self.tree_sig.item(i)['values'][0]: i
                   for i in self.tree_sig.get_children()}
            seen = set()
            for k, hz in rows[:15]:
                seen.add(k)
                vals = [k, '%.1f' % hz]
                if k in old:
                    self.tree_sig.item(old[k], values=vals)
                else:
                    self.tree_sig.insert('', 0, values=vals)
            for k, i in old.items():
                if k not in seen:
                    self.tree_sig.delete(i)

        self._draw_alt()
        self._draw_attitude()
        self._draw_curve(self.crv_alt_canvas, self.curve_alt, ' m')
        self._draw_curve(self.crv_spd_canvas, self.curve_spd, ' m/s')
        self._draw_map()
        self._update_msg_tree(now)

    def _draw_alt(self):
        c = self.alt_canvas
        c.delete('all')
        w = int(c['width'])
        h = int(c['height'])
        pts = list(self.alt_hist)
        if len(pts) < 2:
            c.create_text(w // 2, h // 2, text='等待高度数据...',
                          fill='#666', font=('Microsoft YaHei', 9))
            return
        alts = [a for _, a in pts]
        lo, hi = min(alts), max(alts)
        if hi - lo < 1.0:
            hi = lo + 1.0
        pad = 6
        line = []
        n = len(pts)
        for i, (_, a) in enumerate(pts):
            x = pad + (w - 2 * pad) * i / max(n - 1, 1)
            y = h - pad - (h - 2 * pad) * (a - lo) / (hi - lo)
            line.extend((x, y))
        if len(line) >= 4:
            c.create_line(*line, fill='#4ec9b0', width=2)
        c.create_text(4, 8, anchor='w', text='%.1fm' % alts[-1],
                      fill='#d8d8d8', font=('Consolas', 9))
        c.create_text(w - 4, 8, anchor='e',
                      text='峰值 %.1f' % hi, fill='#888',
                      font=('Consolas', 9))
        c.create_text(4, h - 6, anchor='sw', text='%.1f' % lo,
                      fill='#888', font=('Consolas', 9))

    # ---------------- 关闭 ----------------

    def _on_close(self):
        if self.connected:
            with self.tel_lock:
                armed = self.tel.get('armed')
            if armed:
                if not messagebox.askyesno(
                        '飞机仍处于解锁状态',
                        '飞控当前已解锁(ARMED), 确定要退出 UI?\n'
                        '退出不会自动上锁, 请确保遥控器可接管!'):
                    return
            elif not messagebox.askyesno('退出', '确定退出测试 UI?'):
                return
        self._cancel_soft_rc_loop()
        self.var_rc_on.set(False)
        self._release_rc_override()
        self.reader_running = False
        self.connected = False
        self._stop_ekf_monitor()             # v1.30: 退出前停 EKF 监控线程
        raw = self.raw
        self.raw = None
        self.proxy = None
        with self.action_lock:
            self.action_q.clear()
        t = self.reader_thread
        if t is not None and t.is_alive() \
                and t is not threading.current_thread():
            t.join(timeout=1.5)
        if raw is not None:
            try:
                raw.close()
            except Exception:
                pass
        self._cancel_pending_drop()
        self._stop_record()
        try:
            sys.stdout = self._orig_stdout
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        root.option_add('*Font', '{Microsoft YaHei} 9')
    except Exception:
        pass
    AmpUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
