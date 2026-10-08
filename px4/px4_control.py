#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 飞控 MAVLink 控制脚本 (对应 APM 版 apm_control.py, 按 PX4 官方文档实现)

官方文档依据:
  - 飞行模式 / custom_mode 编码:  https://docs.px4.io/main/en/flight_modes/
    custom_mode = (main_mode << 16) | (sub_mode << 24)
  - 模式切换: MAV_CMD_DO_SET_MODE (param1=MAV_MODE_FLAG_*, param2=main_mode,
    param3=sub_mode) / SET_MODE 消息
    https://docs.px4.io/main/en/mavlink/standard_modes
  - Offboard 控制(位置/速度设定值): SET_POSITION_TARGET_LOCAL_NED /
    SET_POSITION_TARGET_GLOBAL_INT, 需 >2Hz 连续流且切换前至少流 1 秒
    https://docs.px4.io/main/en/flight_modes/offboard
  - 起飞/降落/返航: MAV_CMD_NAV_TAKEOFF / LAND / RETURN_TO_LAUNCH,
    也可直接切 Takeoff / Land / Return(AUTO) 子模式
  - 解锁/上锁与自动上锁: MAV_CMD_COMPONENT_ARM_DISARM;
    COM_DISARM_PRFLT(起飞太慢自动上锁, 默认10s), COM_DISARM_LAND(降落后2s)
    https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm
  - 故障保护参数: NAV_DLL_ACT / COM_LOW_BAT_ACT / COM_OBL_RC_ACT / COM_RCL_EXCEPT
    https://docs.px4.io/main/en/config/safety
  - 任务: MAVLink 任务协议 (MISSION_COUNT/MISSION_REQUEST_INT/MISSION_ITEM_INT),
    与 ArduPilot 通用; 结束项支持 NAV_RETURN_TO_LAUNCH / NAV_LAND

功能流程 (与 APM 版一致):
  1. 连接飞控(串口 / UDP / TCP), 校验确为 PX4 (MAV_AUTOPILOT_PX4)
  2. 等待 GPS 锁星 + 位置估计就绪
  3. 逐个测试飞行模式切换(默认 POSCTL/LOITER/RTL), 每项确认成功后继续
  4. 解锁 (ARM)
  5. 起飞到指定高度 (MAV_CMD_NAV_TAKEOFF -> Takeoff 模式)
  6. 飞往指定 GPS 坐标; 飞行途中可用地面站持续修正目标点
     --nav-mode position: OFFBOARD 位置设定值 (GLOBAL_RELATIVE_ALT_INT)
     --nav-mode speed:    OFFBOARD 速度设定值 (LOCAL_NED, 低速巡航最小油门)
  7. 到达后悬停等待指令 (ON_TIMEOUT='hold' 无限悬停等软件指令,
     自动返航已禁用; 'rtl' 恢复旧行为: --hover 秒内无新指令则返航上锁)

WAYPOINT 航点任务 (--test 4 / mission):
  将多个 GPS 组成航点任务上传飞控, 起飞后切 MISSION 按序飞行,
  末项动作 --mission-end none(默认,停末航点保持)/rtl/land.
  任务条目支持抛投项 ('DROP', 通道, 触发PWM, 回位PWM, 保持ms), 展开为
  DO_SET_SERVO + NAV_DELAY(保持) + DO_SET_SERVO(回位); 单次触发用
  trigger_drop() (实时 COMMAND_LONG, 每步校验 ACK).
  前提: 对应输出通道未被占用 (PX4 该通道输出 function=0/Manual),
  否则 DO_SET_SERVO 被拒.

目标修正/指令来源:
  A. 其他地面站经 MAVLink 下发 SET_POSITION_TARGET_GLOBAL_INT(同链路可见)
  B. 本脚本运行中在终端直接输入指令(回车确认):
       31.2304,121.4737        修正目标(纬度,经度, 高度不变)
       31.2304,121.4737,15     修正目标(纬度,经度,相对高度)
       hover 30                更新悬停等待时间(秒)
       rtl                     立即返航

PID 自动调参: PX4 无 MAVLink AUTOTUNE, 请用 QGroundControl 的 Autotune /
  MC_AT_* 参数; 参数读写校验见 px4_control_test.py 测试11.

依赖: pip install pymavlink

示例:
  串口(PX4/Pixhawk):
    python px4_control.py COM3 --latitude 31.2304 --longitude 121.4737 --altitude 20
  串口指定波特率(图传一般 57600):
    python px4_control.py /dev/ttyUSB0 --baud 57600 --latitude 31.2304 --longitude 121.4737 --altitude 20
  UDP(数传/局域网):
    python px4_control.py udp:127.0.0.1:14550 --latitude 31.2304 --longitude 121.4737 --altitude 20
  PX4 SITL 仿真测试:
    python px4_control.py tcp:127.0.0.1:5760 --latitude 31.2304 --longitude 121.4737 --altitude 20
  多航点任务(上传后起飞按序飞行, 结束动作默认停末航点):
    python px4_control.py COM3 --test 4 --waypoints "31.2304,121.4737,20;31.2310,121.4745,25;31.2320,121.4750,20"
    python px4_control.py COM3 --test 4 --waypoints-file route.txt --mission-end rtl
"""

import argparse
import math
import os
import queue
import sys
import threading
import time

from pymavlink import mavutil

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)  # 允许 python px4/px4_control.py 直接运行
import px4_control_test as tct

# ============================================================
#  PX4 模式定义 (custom_mode = main_mode<<16 | sub_mode<<24)
#  参见 PX4-Autopilot src/modules/commander/px4_custom_mode.h
# ============================================================

PX4_MAIN_MODE = {
    'MANUAL': 1, 'ALTCTL': 2, 'POSCTL': 3, 'AUTO': 4,
    'ACRO': 5, 'OFFBOARD': 6, 'STABILIZED': 7, 'RATTITUDE': 8,
}

PX4_SUB_MODE_AUTO = {
    'READY': 1, 'TAKEOFF': 2, 'LOITER': 3, 'MISSION': 4,
    'RTL': 5, 'LAND': 6, 'RTGS': 7, 'FOLLOW_TARGET': 8,
}

_M = mavutil.mavlink
_FLAG_CUSTOM = _M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
_FLAG_MANUAL = _M.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
_FLAG_STABILIZE = _M.MAV_MODE_FLAG_STABILIZE_ENABLED
_FLAG_AUTO = _M.MAV_MODE_FLAG_AUTO_ENABLED
_FLAG_GUIDED = _M.MAV_MODE_FLAG_GUIDED_ENABLED
_AUTO_FLAGS = _FLAG_CUSTOM | _FLAG_AUTO | _FLAG_GUIDED | _FLAG_STABILIZE

# 模式名 -> (MAV_MODE_FLAG 组合, main_mode, sub_mode)
PX4_MODES = {
    'MANUAL':     (_FLAG_CUSTOM | _FLAG_MANUAL | _FLAG_STABILIZE,
                   PX4_MAIN_MODE['MANUAL'], 0),
    'STABILIZED': (_FLAG_CUSTOM | _FLAG_MANUAL | _FLAG_STABILIZE,
                   PX4_MAIN_MODE['STABILIZED'], 0),
    'ACRO':       (_FLAG_CUSTOM | _FLAG_MANUAL,
                   PX4_MAIN_MODE['ACRO'], 0),
    'RATTITUDE':  (_FLAG_CUSTOM | _FLAG_MANUAL,
                   PX4_MAIN_MODE['RATTITUDE'], 0),
    'ALTCTL':     (_FLAG_CUSTOM | _FLAG_MANUAL | _FLAG_STABILIZE,
                   PX4_MAIN_MODE['ALTCTL'], 0),
    'POSCTL':     (_FLAG_CUSTOM | _FLAG_MANUAL | _FLAG_STABILIZE,
                   PX4_MAIN_MODE['POSCTL'], 0),
    'READY':      (_AUTO_FLAGS, PX4_MAIN_MODE['AUTO'],
                   PX4_SUB_MODE_AUTO['READY']),
    'TAKEOFF':    (_AUTO_FLAGS, PX4_MAIN_MODE['AUTO'],
                   PX4_SUB_MODE_AUTO['TAKEOFF']),
    'LOITER':     (_AUTO_FLAGS, PX4_MAIN_MODE['AUTO'],
                   PX4_SUB_MODE_AUTO['LOITER']),
    'MISSION':    (_AUTO_FLAGS, PX4_MAIN_MODE['AUTO'],
                   PX4_SUB_MODE_AUTO['MISSION']),
    'RTL':        (_AUTO_FLAGS, PX4_MAIN_MODE['AUTO'],
                   PX4_SUB_MODE_AUTO['RTL']),
    'LAND':       (_AUTO_FLAGS, PX4_MAIN_MODE['AUTO'],
                   PX4_SUB_MODE_AUTO['LAND']),
    'OFFBOARD':   (_AUTO_FLAGS, PX4_MAIN_MODE['OFFBOARD'], 0),
}

# ArduPilot/通用模式名 -> PX4 模式名 (兼容旧脚本/UI/API 传参)
APM_MODE_ALIASES = {
    'GUIDED': 'POSCTL', 'GUIDED_NOGPS': 'POSCTL',
    'ALT_HOLD': 'ALTCTL', 'STABILIZE': 'STABILIZED',
    'POSHOLD': 'LOITER', 'BRAKE': 'LOITER', 'CIRCLE': 'LOITER',
    'SMARTRTL': 'RTL', 'AUTO': 'MISSION', 'AUTOTUNE': 'POSCTL',
    'DRIFT': 'POSCTL', 'SPORT': 'ALTCTL', 'HOLD': 'LOITER',
}


def pack_custom_mode(main_mode, sub_mode=0):
    """PX4 custom_mode 打包: (main_mode << 16) | (sub_mode << 24)"""
    return ((int(main_mode) & 0xFF) << 16) | ((int(sub_mode) & 0xFF) << 24)


def custom_mode_parts(custom_mode):
    """PX4 custom_mode 解包 -> (main_mode, sub_mode)"""
    cm = int(custom_mode) & 0xFFFFFFFF
    return (cm >> 16) & 0xFF, (cm >> 24) & 0xFF


def custom_mode_name(custom_mode):
    """PX4 custom_mode -> 人读模式名 (供 UI/日志显示)"""
    main, sub = custom_mode_parts(custom_mode)
    for name, (_f, m, s) in PX4_MODES.items():
        if m == main and s == sub:
            return name
    main_names = {v: k for k, v in PX4_MAIN_MODE.items()}
    if main in main_names:
        if main == PX4_MAIN_MODE['AUTO'] and sub:
            sname = {v: k for k, v in PX4_SUB_MODE_AUTO.items()}.get(sub)
            if sname:
                return 'AUTO_%s' % sname
        return main_names[main]
    return 'UNKNOWN(%s)' % custom_mode


DEG2RAD = math.pi / 180.0
EARTH_RADIUS = 6371000.0

# v1.21 发射箱: "到不了目标/悬停无指令"时的策略
#   'hold' = 悬停保持等软件指令 (禁自动返航, 符合"只有软件能命令动作")
#   'rtl'  = 超时自动返航降落 (旧行为, 地面交互兜底)
ON_TIMEOUT = 'hold'

# PX4 解锁后起飞太慢会自动上锁: COM_DISARM_PRFLT (秒), 默认 10s
# 0=不改飞控参数; 起飞前写入, 避免 arm 后未及时 takeoff 被 DISARM
DISARM_DELAY_SEC = 60

# Offboard 设定值流 (PX4 官方要求 >2Hz 且切换前至少流 1 秒)
OFFBOARD_RATE_HZ = 10.0        # 本地发送线程频率
OFFBOARD_WARMUP_SEC = 1.5      # 切 OFFBOARD 前预热流时长

# SET_POSITION_TARGET_* type_mask:
#   bit0-2: x/y/z  bit3-5: vx/vy/vz  bit6-8: afx/afy/afz
#   bit9: yaw      bit10: yaw_rate   (置1=忽略该组)
POSITION_TYPE_MASK = 0b0000011111111000   # 0x7F8 只控位置
VELOCITY_TYPE_MASK = 0b0000011111000111   # 0x7C7 只控速度(NED)


# ============================================================
#  地理计算
# ============================================================

def haversine(lat1, lon1, lat2, lon2):
    """计算两点球面距离, 返回米"""
    dlat = (lat2 - lat1) * DEG2RAD
    dlon = (lon2 - lon1) * DEG2RAD
    a = (math.sin(dlat / 2.0) ** 2 +
         math.cos(lat1 * DEG2RAD) * math.cos(lat2 * DEG2RAD) *
         math.sin(dlon / 2.0) ** 2)
    return EARTH_RADIUS * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def bearing_to(lat1, lon1, lat2, lon2):
    """计算从点1到点2的方位角(北为0度,顺时针), 返回度数"""
    dlon = (lon2 - lon1) * DEG2RAD
    y = math.sin(dlon) * math.cos(lat2 * DEG2RAD)
    x = (math.cos(lat1 * DEG2RAD) * math.sin(lat2 * DEG2RAD) -
         math.sin(lat1 * DEG2RAD) * math.cos(lat2 * DEG2RAD) * math.cos(dlon))
    return math.degrees(math.atan2(y, x)) % 360.0


# ============================================================
#  连接 / 预检
# ============================================================

def mavconnect(conn_str, baud):
    """建立 MAVLink 连接并等待心跳, 校验对端确为 PX4"""
    master = mavutil.mavlink_connection(conn_str, baud=baud)
    print('正在连接 %s , 等待飞控心跳...' % conn_str)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        sys.exit('错误: 无法获取飞控心跳, 请检查连接/波特率!')
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent()
    try:
        master.vehicle_type = 'copter'
    except AttributeError:
        pass
    type_name = mavutil.mavlink.enums['MAV_TYPE'][hb.type].name
    ap_enum = mavutil.mavlink.enums['MAV_AUTOPILOT'].get(hb.autopilot)
    ap_name = ap_enum.name if ap_enum is not None else str(hb.autopilot)
    print('[OK] 已连接: sysid=%u compid=%u 机型=%s 飞控=%s' % (
        hb.get_srcSystem(), hb.get_srcComponent(), type_name, ap_name))
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        sys.exit('错误: 对端不是 PX4 固件 (%s)! '
                 'APM/ArduPilot 请使用 apm_control.py' % ap_name)
    print('  当前模式: %s' % custom_mode_name(hb.custom_mode))
    return master


def wait_gps_lock(master, min_sats=6, timeout=30):
    """等待 GPS 3D 定位且卫星数足够"""
    print('等待 GPS 锁星...')
    t_end = time.time() + timeout
    while time.time() < t_end:
        g = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if g and g.fix_type >= 3 and g.satellites_visible >= min_sats:
            print('[OK] GPS 已定位: fix_type=%d 卫星=%d颗' % (
                g.fix_type, g.satellites_visible))
            return
    sys.exit('错误: GPS 定位超时, 请确认室外/有足够卫星信号')


def wait_position_ready(master, timeout=30):
    """等待位置估计就绪: GPS 3D + 收到 GLOBAL_POSITION_INT(经纬度非0).
    PX4 不发 EKF_STATUS_REPORT, 位置有效性以全局位置流为准.
    每 5s 打印进度; 超时 sys.exit (与 APM 版行为一致)."""
    print('等待位置估计就绪 (GPS+EKF2 收敛)...')
    t_end = time.time() + timeout
    last_report = 0.0
    while time.time() < t_end:
        m = master.recv_match(type=['GPS_RAW_INT', 'GLOBAL_POSITION_INT'],
                              blocking=True, timeout=1)
        now = time.time()
        if m is None:
            if now - last_report >= 5.0:
                print('  等待中... 剩余 %.0f 秒' % max(t_end - now, 0))
                last_report = now
            continue
        if m.get_type() == 'GPS_RAW_INT':
            if m.fix_type < 3:
                continue
        elif m.lat == 0 and m.lon == 0:
            continue
        print('[OK] 位置估计就绪')
        return
    sys.exit('错误: 位置估计未就绪 (超时 %gs), 无法安全起飞' % timeout)


# ============================================================
#  模式切换 (PX4: MAV_CMD_DO_SET_MODE + 心跳 custom_mode 校验)
# ============================================================

def _resolve_mode(mode_name):
    """模式名(含 ArduPilot 别名) -> PX4 规范名; 未知返回 None"""
    name = str(mode_name).strip().upper()
    name = APM_MODE_ALIASES.get(name, name)
    return name if name in PX4_MODES else None


def set_mode(master, mode_name):
    """切换 PX4 飞行模式并等待心跳确认生效 (失败 sys.exit).

    发送顺序:
      1. COMMAND_LONG MAV_CMD_DO_SET_MODE(param1=flags, param2=main, param3=sub)
      2. 3 秒未确认再补发 SET_MODE(base_mode, custom_mode) 消息兜底
    成功判据: HEARTBEAT.custom_mode 等于打包后的目标值.
    """
    name = _resolve_mode(mode_name)
    if name is None:
        sys.exit('错误: 未知飞行模式 %s (PX4 支持: %s)' % (
            mode_name, ','.join(sorted(PX4_MODES))))
    flags, main_mode, sub_mode = PX4_MODES[name]
    target_cm = pack_custom_mode(main_mode, sub_mode)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        flags, float(main_mode), float(sub_mode), 0, 0, 0, 0)

    t_start = time.time()
    fallback_sent = False
    deadline = t_start + 10.0
    while time.time() < deadline:
        if not fallback_sent and time.time() - t_start >= 3.0:
            # 兜底: 部分链路对 DO_SET_MODE 无确认, 用 SET_MODE 消息重试
            fallback_sent = True
            try:
                master.mav.set_mode_send(
                    master.target_system, flags, target_cm)
            except Exception:
                pass
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        if (hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                and int(hb.custom_mode) == target_cm):
            print('[OK] 飞行模式: %s' % name)
            return
    sys.exit('错误: 切换到 %s 模式失败 (未解锁/位置无效/飞控配置不支持)'
             % name)


def get_mode(master, timeout=1.0):
    """读取当前模式名 (等待一个心跳); 超时返回 None"""
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
    if hb is None:
        return None
    return custom_mode_name(hb.custom_mode)


# ============================================================
#  参数
# ============================================================

def _param_id_eq(param_id, name):
    pn = param_id
    if isinstance(pn, (bytes, bytearray)):
        pn = pn.decode('ascii', 'replace')
    return str(pn).split('\x00')[0] == name


def set_param(master, name, value, timeout=3.0):
    """PARAM_SET 写参数, 等 PARAM_VALUE 回读. 返回 True/False"""
    name = str(name)
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('ascii', 'replace'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            return False
        if _param_id_eq(msg.param_id, name):
            print('[OK] 参数 %s = %g' % (name, float(msg.param_value)))
            return True
    return False


def get_param(master, name, timeout=1.5):
    """PARAM_REQUEST_READ -> PARAM_VALUE, 返回 float 或 None"""
    name = str(name)
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii', 'replace'), -1)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            return None
        if _param_id_eq(msg.param_id, name):
            return float(msg.param_value)
    return None


def ensure_disarm_delay(master):
    """起飞前延长 PX4 '起飞太慢自动上锁' 超时 (COM_DISARM_PRFLT, 默认10s),
    防止解锁后未起飞被自动上锁 (对应 APM 的 DISARM_DELAY).
    DISARM_DELAY_SEC<=0 则跳过. 失败仅警告不退出."""
    if not DISARM_DELAY_SEC or DISARM_DELAY_SEC <= 0:
        return
    print('设置 COM_DISARM_PRFLT=%ds (防止解锁后未起飞自动上锁)...'
          % DISARM_DELAY_SEC)
    if not set_param(master, 'COM_DISARM_PRFLT', float(DISARM_DELAY_SEC),
                     timeout=2.5):
        print('警告: 写入 COM_DISARM_PRFLT 失败, 仍继续; '
              '若仍自动上锁请在 QGroundControl 改该参数')


def ensure_launch_params(master):
    """发射箱参数预设委托给测试模块 (PX4 故障保护参数组)"""
    return tct.ensure_launch_params(master)


def collect_status_text(master, duration=2.0):
    """收集并打印 STATUSTEXT(预检/解锁拒绝原因等), 返回最后一条"""
    last = ''
    t_end = time.time() + duration
    while time.time() < t_end:
        msg = master.recv_match(type='STATUSTEXT', blocking=True,
                                timeout=max(0.1, t_end - time.time()))
        if msg is None:
            continue
        text = msg.text
        if isinstance(text, (bytes, bytearray)):
            text = text.decode('utf-8', 'replace')
        last = str(text)
        print('  [飞控] %s' % last)
    return last


# ============================================================
#  Offboard 设定值流
# ============================================================

class OffboardStream(object):
    """后台线程以 OFFBOARD_RATE_HZ 连续发送当前设定值(PX4 要求 >2Hz).
    kind='pos' 发 GLOBAL_RELATIVE_ALT_INT 位置, 'vel' 发 LOCAL_NED 速度."""

    def __init__(self, master, rate=OFFBOARD_RATE_HZ):
        self.master = master
        self.period = 1.0 / float(rate)
        self._lock = threading.Lock()
        self._kind = 'vel'
        self._vx = self._vy = self._vz = 0.0
        self._lat = self._lon = self._alt = 0.0
        self._run = False
        self._thread = None
        self.started_at = 0.0     # 流启动时刻(切 OFFBOARD 前需预热)
        self.ob_confirmed_at = 0.0  # 最近一次确认处于 OFFBOARD 的时刻

    @property
    def running(self):
        with self._lock:
            return self._run

    def set_velocity(self, vx, vy, vz=0.0):
        with self._lock:
            self._kind = 'vel'
            self._vx, self._vy, self._vz = float(vx), float(vy), float(vz)

    def set_position(self, lat, lon, alt):
        with self._lock:
            self._kind = 'pos'
            self._lat, self._lon = float(lat), float(lon)
            self._alt = float(alt)

    def start(self):
        """启动发送线程(已启动则直接返回), 返回是否是本次新启动"""
        with self._lock:
            if self._run:
                return False
            self._run = True
            self.started_at = time.time()
            self._thread = threading.Thread(target=self._loop,
                                            name='px4-offboard', daemon=True)
            self._thread.start()
        return True

    def stop(self):
        with self._lock:
            self._run = False
            th = self._thread
            self._thread = None
        if th is not None and th is not threading.current_thread():
            th.join(timeout=1.0)

    def _sample(self):
        with self._lock:
            if self._kind == 'pos':
                return 'pos', self._lat, self._lon, self._alt
            return 'vel', self._vx, self._vy, self._vz

    def _loop(self):
        while True:
            with self._lock:
                if not self._run:
                    return
            kind, a, b, c = self._sample()
            try:
                if kind == 'pos':
                    self.master.mav.set_position_target_global_int_send(
                        0, self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        POSITION_TYPE_MASK,
                        int(round(a * 1e7)), int(round(b * 1e7)),
                        float(c), 0, 0, 0, 0, 0, 0, 0, 0)
                else:
                    self.master.mav.set_position_target_local_ned_send(
                        0, self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        VELOCITY_TYPE_MASK,
                        0, 0, 0, float(a), float(b), float(c),
                        0, 0, 0, 0, 0)
            except Exception:
                return  # 链路已关闭
            time.sleep(self.period)


_streams = {}


def _stream_for(master):
    """按 master 取(或建)其 OffboardStream 单例"""
    key = id(master)
    s = _streams.get(key)
    if s is None or s.master is not master:
        s = OffboardStream(master)
        _streams[key] = s
    return s


def stop_offboard_stream(master=None):
    """停止设定值流(master=None 停全部), 断开/退出前调用防线程残留"""
    if master is None:
        for s in list(_streams.values()):
            s.stop()
        _streams.clear()
        return
    s = _streams.get(id(master))
    if s is not None:
        s.stop()
        _streams.pop(id(master), None)


def ensure_offboard(master, timeout=6.0, force_check=False):
    """启动设定值流(必要时预热 OFFBOARD_WARMUP_SEC)并确保处于 OFFBOARD.
    未解锁时只流不切(解锁后下次调用再切); 已确认 ≤8s 内不重复读心跳.
    返回 True/False"""
    s = _stream_for(master)
    fresh = s.start()
    if fresh:
        time.sleep(OFFBOARD_WARMUP_SEC)
    now = time.time()
    if not force_check and now - s.ob_confirmed_at < 8.0:
        return True
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
    if hb is None:
        return False
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if int(hb.custom_mode) == pack_custom_mode(
            PX4_MAIN_MODE['OFFBOARD'], 0):
        s.ob_confirmed_at = time.time()
        return True
    if not armed:
        return True  # 解锁后调用才会真正切入
    print('  切入 OFFBOARD 模式 (需设定值流 ≥%gs)...'
          % OFFBOARD_WARMUP_SEC)
    if tct.set_flight_mode(master, 'OFFBOARD', timeout=timeout):
        s.ob_confirmed_at = time.time()
        return True
    print('  错误: 切 OFFBOARD 失败 (流未就绪/被其他模式占用)')
    return False


# ============================================================
#  解锁 / 起飞 / 上锁
# ============================================================

def arm(master):
    """解锁 (ARM): 提示发射箱/DISARM_DELAY 参数 -> 发 ARM 命令
    (PX4 无油门高位 PreArm, 不需要 RC 覆盖) -> 校验 ACK + ARMED,
    被拒收集 STATUSTEXT 原因后退出 (与 APM 版行为一致)"""
    ensure_disarm_delay(master)
    tct.ensure_launch_params(master)

    print('发送 ARM (PX4: 无需 RC 油门覆盖)...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    ack = None
    last_text = ''
    t_end = time.time() + 10.0
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            if master.motors_armed():
                break
            continue
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            last_text = str(text)
            print('  [飞控] %s' % last_text)
        elif mtype == 'COMMAND_ACK' and \
                msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print('  ARM 命令已接受, 等待 ARMED...')
            else:
                ack = msg
                break
        elif mtype == 'HEARTBEAT' and (
                msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            ack = None
            break

    if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        more = collect_status_text(master, duration=1.5)
        result_names = {
            1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
            4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
        }
        rn = result_names.get(ack.result, str(ack.result))
        sys.exit('错误: 解锁失败 COMMAND_ACK=%s%s' % (
            rn, (', ' + more) if more else ''))

    if not master.motors_armed():
        t_end = time.time() + 5.0
        while time.time() < t_end:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb and (hb.base_mode &
                       mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                break
            if not last_text:
                last_text = collect_status_text(master, duration=0.3)
        if not last_text:
            collect_status_text(master, duration=0.5)

    time.sleep(0.2)

    if master.motors_armed():
        print('[OK] 已解锁 (ARMED)')
        return

    if not last_text:
        last_text = collect_status_text(master, duration=1.0)
    sys.exit('错误: 解锁后未检测到 ARMED%s' % (
        ('; 飞控: ' + last_text) if last_text else ''))


def takeoff(master, alt, timeout=45.0):
    """起飞到指定高度(解锁后尽快调用, 避免 COM_DISARM_PRFLT 自动上锁).
    PX4: 写 MIS_TAKEOFF_ALT=alt + MAV_CMD_NAV_TAKEOFF(param5/6=NaN 用当前
    位置, param7=alt); 被拒/未见爬升兜底切 AUTO/TAKEOFF 子模式;
    等 GLOBAL_POSITION_INT.relative_alt 到位."""
    print('正在起飞至 %.1f 米...' % alt)
    set_param(master, 'MIS_TAKEOFF_ALT', float(alt), timeout=2.0)
    nan = float('nan')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, nan, nan, float(alt))

    rejected = False
    t_end = time.time() + 10.0
    while time.time() < t_end:
        a = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if a and a.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if a.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                collect_status_text(master, duration=1.0)
                rejected = True
                break
            break

    if rejected:
        print('  起飞命令被拒, 兜底切 TAKEOFF 模式 (按 MIS_TAKEOFF_ALT 爬升)...')
        if not tct.set_flight_mode(master, 'TAKEOFF', timeout=5.0,
                                   quiet=True):
            sys.exit('错误: NAV_TAKEOFF 被拒且切 TAKEOFF 模式失败')

    t_end = time.time() + timeout
    rel = 0.0
    last_report = 0.0
    while time.time() < t_end:
        p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1)
        if p:
            rel = p.relative_alt / 1000.0
            if rel >= alt - 1.0:
                print('[OK] 已到达目标高度 %.1f 米' % rel)
                print('提示: OFFBOARD/自动模式下摇杆无效属正常, '
                      '拨遥控器模式开关到 LOITER 可接管')
                return
            now = time.time()
            if now - last_report >= 5.0:
                print('  爬升中: 当前 %.1f / %.1f m' % (rel, alt))
                last_report = now
        if not master.motors_armed():
            sys.exit('错误: 起飞过程中电机已被上锁(DISARM), '
                     '可能触发 COM_DISARM_PRFLT')
    sys.exit('错误: 起飞超时, 最后高度仅 %.1f 米' % rel)


def arm_and_takeoff(master, alt, mode='POSCTL'):
    """解锁+立即起飞一体(测试3/4): 避免 arm 后未及时 takeoff 被
    COM_DISARM_PRFLT 上锁. mode 也接受 APM 名(GUIDED 等)自动翻译."""
    name = _resolve_mode(mode)
    if name is None:
        sys.exit('错误: 未知起飞模式 %s' % mode)
    if name != 'AUTO':
        set_mode(master, name)
    arm(master)
    print('[起飞] 解锁后立即 NAV_TAKEOFF -> %.1f 米' % alt)
    takeoff(master, alt)


def goto(master, lat, lon, alt):
    """设定 OFFBOARD 目标点并确保处于 OFFBOARD(位置设定值流).
    与 APM 版一致可反复调用(1Hz)刷新目标; 返回 True/False"""
    s = _stream_for(master)
    s.set_position(lat, lon, alt)
    return ensure_offboard(master)


def set_velocity(master, vx, vy, vz=0.0):
    """发送速度指令(LOCAL NED, 仅控速度, 位置/朝向由飞控保持),
    用于低速巡航最小油门; 到达后传 (0,0,0) 即悬停. 返回 True/False"""
    s = _stream_for(master)
    s.set_velocity(vx, vy, vz)
    return ensure_offboard(master)


# ============================================================
#  航点解析 / 任务上传执行
# ============================================================

def parse_waypoints(spec, default_alt):
    """解析航点串: 'lat,lon[,alt];lat,lon[,alt]' (分号或换行分隔),
    alt 缺省用 default_alt; 返回 [(lat, lon, alt), ...]"""
    if not spec or not str(spec).strip():
        return []
    wps = []
    text = str(spec).replace('\r\n', '\n').replace('\r', '\n')
    parts = []
    for chunk in text.replace('\n', ';').split(';'):
        chunk = chunk.strip()
        if chunk:
            parts.append(chunk)
    for part in parts:
        coords = [c.strip() for c in part.split(',')]
        if len(coords) < 2:
            sys.exit('错误: 航点格式应为 lat,lon[,alt]: %r' % part)
        try:
            lat = float(coords[0])
            lon = float(coords[1])
            alt = float(coords[2]) if len(coords) >= 3 else float(default_alt)
        except ValueError:
            sys.exit('错误: 航点数值无效: %r' % part)
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            sys.exit('错误: 航点经纬度超出有效范围: %r' % part)
        if alt <= 0:
            sys.exit('错误: 航点高度必须大于 0: %r' % part)
        wps.append((lat, lon, alt))
    return wps


def load_waypoints_file(path, default_alt):
    """从文件读取航点: 每行 lat,lon[,alt], 支持 # 注释"""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            raw = f.read()
    except OSError as e:
        sys.exit('错误: 无法读取航点文件 %s: %s' % (path, e))
    lines = []
    for line in raw.splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            lines.append(line)
    return parse_waypoints(';'.join(lines), default_alt)


def _is_drop(entry):
    """条目是否为抛投项 ('DROP', 通道, 触发PWM, 回位PWM, 保持ms)"""
    return (isinstance(entry, (tuple, list)) and len(entry) >= 1
            and str(entry[0]).upper() == 'DROP')


MISSION_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'ERROR', 2: 'UNSUPPORTED_FRAME', 3: 'UNSUPPORTED',
    4: 'NO_SPACE(任务超容量)', 5: 'INVALID', 6: 'INVALID_PARAM1',
    7: 'INVALID_PARAM2', 8: 'INVALID_PARAM3', 9: 'INVALID_PARAM4',
    10: 'INVALID_PARAM5_X', 11: 'INVALID_PARAM6_Y', 12: 'INVALID_PARAM7',
    13: 'INVALID_SEQUENCE', 14: 'DENIED', 15: 'OPERATION_CANCELLED',
}


def upload_mission(master, wps, end_action='rtl', timeout=30.0):
    """通过 MAVLink 任务协议上传航点任务到 PX4.
    wps: [(lat, lon, alt), ...] 与 ('DROP', 通道, 触发PWM, 回位PWM, 保持ms)
    混合; DROP 展开为 DO_SET_SERVO(触发) [+NAV_DELAY 保持] +
    DO_SET_SERVO(回位). end_action: rtl/land/none 追加结束指令.
    成功返回任务项数, 失败 False. (PX4 无 home 槽位, 按真实项数上传)"""
    items = []
    n_wp = 0
    n_drop = 0
    last_ll = None

    def _drop_items(sno, s_on, s_off, hold_ms):
        def _do(pwm):
            return {
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                'command': mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                'current': 0,
                'param1': float(sno), 'param2': float(pwm),
                'param3': 0.0, 'param4': 0.0,
                'x': 0, 'y': 0, 'z': 0.0,
            }
        seq = [_do(s_on)]
        if hold_ms and float(hold_ms) > 0:
            seq.append({
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                'command': mavutil.mavlink.MAV_CMD_NAV_DELAY,
                'current': 0,
                'param1': float(max(1, int(math.ceil(
                    float(hold_ms) / 1000.0)))),
                'param2': -1.0, 'param3': -1.0, 'param4': -1.0,
                'x': 0, 'y': 0, 'z': 0.0,
            })
        if s_off is not None:
            seq.append(_do(s_off))
        return seq

    for entry in wps:
        if _is_drop(entry):
            sno = entry[1]
            s_on = entry[2]
            s_off = entry[3] if len(entry) > 3 else None
            hold_ms = entry[4] if len(entry) > 4 else 0
            items.extend(_drop_items(sno, s_on, s_off, hold_ms))
            n_drop += 1
            continue
        lat, lon, alt = entry
        last_ll = (lat, lon, alt)
        items.append({
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 1 if n_wp == 0 else 0,
            'param1': 0.0,   # 到点悬停秒数(0=按飞控默认)
            'param2': 2.0,   # 到点半径(米)
            'param3': 0.0,
            # PX4: yaw=NaN 表示保持当前朝向(不触发旋转)
            'param4': float('nan'),
            'x': int(round(lat * 1e7)),
            'y': int(round(lon * 1e7)),
            'z': float(alt),
        })
        n_wp += 1
    if end_action == 'rtl':
        items.append({
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            'current': 0,
            'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
            'x': 0, 'y': 0, 'z': 0.0,
        })
    elif end_action == 'land':
        lat, lon, alt = last_ll if last_ll else (0.0, 0.0, 0.0)
        items.append({
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            'command': mavutil.mavlink.MAV_CMD_NAV_LAND,
            'current': 0,
            'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
            'x': int(round(lat * 1e7)),
            'y': int(round(lon * 1e7)),
            'z': 0.0,
        })

    count = len(items)
    print('开始上传任务: %d 航点, %d 抛投, 结束动作 %s ...' % (
        n_wp, n_drop, end_action.upper()))

    def _send_item(seq):
        it = items[seq]
        master.mav.mission_item_int_send(
            master.target_system, master.target_component,
            seq, it['frame'], it['command'], it['current'], 1,
            it['param1'], it['param2'], it['param3'], it['param4'],
            it['x'], it['y'], it['z'])

    master.mav.mission_count_send(
        master.target_system, master.target_component, count)

    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
            blocking=True, timeout=1)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
            seq = msg.seq
            if 0 <= seq < count:
                _send_item(seq)
                print('  发送任务项 %d/%d' % (seq + 1, count))
        elif mtype == 'MISSION_ACK':
            ack = msg.type
            if ack == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print('[OK] 任务上传成功 (%d 项)' % count)
                return count
            print('错误: 任务上传被拒绝, MAV_MISSION_RESULT=%s %s'
                  % (ack, MISSION_RESULT_NAMES.get(int(ack), '未知')))
            return False
    print('错误: 任务上传超时')
    return False


def reset_mission_to_start(master, timeout=2.0):
    """把任务指针复位到第 1 项(进 MISSION 前调用).
    MAV_CMD_DO_SET_MISSION_CURRENT(param1=0) 取 ACK, 被拒/无 ACK 再回退
    MISSION_SET_CURRENT(0) 消息; best-effort, 失败仅告警. 返回 True/False"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0,
        0, 0, 0, 0, 0, 0, 0)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            break
        if msg.command != mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT:
            continue
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print('[OK] 任务指针已复位到第 1 项')
            return True
        break
    print('  改用 MISSION_SET_CURRENT(0) 复位任务指针...')
    master.mav.mission_set_current_send(
        master.target_system, master.target_component, 0)
    t_end = time.time() + 1.5
    while time.time() < t_end:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            break
        if master.target_system and \
                msg.get_srcSystem() != master.target_system:
            continue
        if msg.seq == 0:
            print('[OK] 任务指针已复位到第 1 项')
            return True
        break
    print('  任务指针复位未确认, 按当前指针执行')
    return False


def wait_auto_mission(master, n_wp, end_action, timeout=600.0):
    """MISSION 模式下等待航点任务完成.
    监视 MISSION_CURRENT 与心跳 custom_mode; 末项 rtl/land 由飞控切入
    时判定成功; end_action=none 时以到达最后航点为准. 返回 True/False"""
    print('任务执行中 (共 %d 航点, 结束=%s), 超时 %d 秒...' % (
        n_wp, end_action.upper(), int(timeout)))
    stop_offboard_stream(master)   # 任务由 MISSION 模式接管
    t_end = time.time() + timeout
    last_seq = -1
    hold_since = None
    last_report = 0.0
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'MISSION_CURRENT':
            last_seq = msg.seq
            now = time.time()
            if now - last_report >= 5.0:
                shown = min(last_seq + 1, n_wp)
                print('  任务进度: 航点 %d/%d (seq=%d)' % (
                    shown, n_wp, last_seq))
                last_report = now
        elif mtype == 'HEARTBEAT':
            if master.target_system and \
                    msg.get_srcSystem() != master.target_system:
                continue
            name = custom_mode_name(msg.custom_mode)
            if name == 'RTL':
                print('[OK] 任务结束, 飞控已进入 RTL 返航')
                return True
            if name == 'LAND' and end_action == 'land':
                print('[OK] 任务结束, 飞控已进入 LAND 降落')
                return True
            main_mode, sub_mode = custom_mode_parts(msg.custom_mode)
            in_auto = (main_mode == PX4_MAIN_MODE['AUTO']
                       and sub_mode != PX4_SUB_MODE_AUTO['LOITER'])
            if not in_auto and last_seq >= 0:
                if end_action == 'none' and last_seq >= n_wp - 1:
                    print('[OK] 任务完成, 已离开 MISSION (当前 %s)' % name)
                    return True
                print('警告: 任务中离开自动档模式 -> %s' % name)
                return False
        elif mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            text = str(text)
            low = text.lower()
            if 'mission' in low or 'command' in low or 'takeoff' in low:
                print('  [任务] %s' % text)
                if 'reject' in low or 'fail' in low or 'error' in low:
                    return False

        if end_action == 'none' and last_seq >= n_wp - 1:
            now = time.time()
            if hold_since is None:
                hold_since = now
            elif now - hold_since >= 3.0:
                print('[OK] 已到达最后航点, 任务完成 (停留在 MISSION)')
                return True
        else:
            hold_since = None
    print('错误: 任务执行超时 (最后 seq=%d)' % last_seq)
    return False


# ============================================================
#  目标修正 / 终端指令 / 导航
# ============================================================

class MissionState(object):
    """任务状态: 当前目标点/高度/悬停等待时间; dirty=目标刚被修正"""
    def __init__(self, lat, lon, alt, hover):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.hover = hover
        self.dirty = False


def start_stdin_listener():
    """后台线程读取终端指令, 返回 queue; 无交互输入时自动结束"""
    q = queue.Queue()

    def _reader():
        try:
            while True:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
                if line:
                    q.put(line)
        except Exception:
            pass

    threading.Thread(target=_reader, daemon=True).start()
    return q


def parse_stdin_command(text):
    """解析终端指令, 返回:
    ('rtl',) | ('hover', 秒) | ('target', lat, lon, alt或None) | None"""
    t = text.strip()
    if not t:
        return None
    low = t.lower()
    if low in ('rtl', 'return', 'home', '返航'):
        return ('rtl',)
    parts = t.split()
    if len(parts) == 2 and parts[0].lower() in ('hover', 'time', 't', '悬停'):
        try:
            sec = float(parts[1])
        except ValueError:
            return None
        if sec > 0:
            return ('hover', sec)
        return None
    coords = t.split(',')
    if len(coords) in (2, 3):
        try:
            lat = float(coords[0])
            lon = float(coords[1])
            alt = float(coords[2]) if len(coords) == 3 else None
        except ValueError:
            return None
        if (-90 <= lat <= 90) and (-180 <= lon <= 180):
            if alt is not None and alt <= 0:
                return None
            return ('target', lat, lon, alt)
    return None


def drain_stdin(st, cmd_q):
    """处理终端指令队列, 返回 'rtl' | 'hover' | 'target' | None"""
    if cmd_q is None:
        return None
    result = None
    saw_rtl = False
    while True:
        try:
            line = cmd_q.get_nowait()
        except queue.Empty:
            break
        cmd = parse_stdin_command(line)
        if cmd is None:
            print('未知指令: %s  (支持: lat,lon | lat,lon,alt | hover 秒 | rtl)'
                  % line)
            continue
        if cmd[0] == 'rtl':
            saw_rtl = True
            continue
        if cmd[0] == 'hover':
            st.hover = cmd[1]
            result = 'hover'
            print('[指令] 悬停等待时间: %.1f 秒' % st.hover)
            continue
        st.lat = cmd[1]
        st.lon = cmd[2]
        if cmd[3] is not None:
            st.alt = cmd[3]
        st.dirty = True
        result = 'target'
        print('[指令] 目标点修正: %.6f, %.6f  高度 %.1f m' % (
            st.lat, st.lon, st.alt))
    if saw_rtl:
        return 'rtl'
    return result


def apply_external_target(st, msg):
    """处理地面站下发的 SET_POSITION_TARGET_GLOBAL_INT, 修正目标;
    返回是否有变化"""
    if (msg.type_mask & 0b11) != 0:
        return False  # 未使用经纬度(如纯速度指令)
    if msg.lat_int == 0 and msg.lon_int == 0:
        return False
    lat = msg.lat_int / 1e7
    lon = msg.lon_int / 1e7
    if abs(lat - st.lat) < 1e-7 and abs(lon - st.lon) < 1e-7:
        return False  # 与当前目标一致(含自身回显), 忽略
    alt = None
    if (msg.type_mask & 0b100) == 0 and msg.alt:
        if msg.coordinate_frame in (
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT):
            alt = float(msg.alt)
    st.lat = lat
    st.lon = lon
    if alt is not None:
        st.alt = alt
    st.dirty = True
    print('[指令] 地面站目标修正: %.6f, %.6f  高度 %.1f m' % (
        st.lat, st.lon, st.alt))
    return True


def fly_to_target(master, st, speed, threshold=3.0, timeout=180, cmd_q=None):
    """以指定巡航速度(speed, 米/秒)低速飞往 st 目标点(OFFBOARD 速度流).
    途中监听 stdin 指令与地面站 SET_POSITION_TARGET_GLOBAL_INT, 可随时
    修正目标. 返回 'arrived' | 'rtl' | 'hold'(超时且 ON_TIMEOUT='hold')"""
    print('以 %.1f m/s 巡航速度飞往目标点 (%.6f, %.6f)...' % (
        speed, st.lat, st.lon))
    t_end = time.time() + timeout
    last_report = 0.0
    d = float('inf')
    while time.time() < t_end:
        act = drain_stdin(st, cmd_q)
        if act == 'rtl':
            set_velocity(master, 0.0, 0.0, 0.0)
            print('收到 rtl 指令, 中止巡航')
            return 'rtl'
        if st.dirty:
            st.dirty = False
            t_end = time.time() + timeout
            print('航向更新 -> (%.6f, %.6f)' % (st.lat, st.lon))
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'SET_POSITION_TARGET_GLOBAL_INT':
            apply_external_target(st, msg)
            continue
        if mtype != 'GLOBAL_POSITION_INT':
            continue
        cur_lat = msg.lat / 1e7
        cur_lon = msg.lon / 1e7
        d = haversine(cur_lat, cur_lon, st.lat, st.lon)
        if d <= threshold:
            set_velocity(master, 0.0, 0.0, 0.0)
            print('[OK] 已到达目标点 (误差 %.1f m)' % d)
            return 'arrived'
        brg = math.radians(bearing_to(cur_lat, cur_lon, st.lat, st.lon))
        # NED: 北分量 = speed*cos(方位角), 东分量 = speed*sin(方位角)
        vx = speed * math.cos(brg)
        vy = speed * math.sin(brg)
        set_velocity(master, vx, vy, 0.0)
        now = time.time()
        if now - last_report >= 5.0:
            print('  距目标 %.1f m, 高度 %.1f m' % (
                d, msg.relative_alt / 1000.0))
            last_report = now
    set_velocity(master, 0.0, 0.0, 0.0)
    if ON_TIMEOUT == 'rtl':
        print('警告: 飞往目标点超时(最后距目标 %.1f m), 转为返航' % d)
        return 'rtl'
    print('警告: 飞往目标点超时(最后距目标 %.1f m), 悬停保持等待软件指令 '
          '(ON_TIMEOUT=%s)' % (d, ON_TIMEOUT))
    return 'hold'


def wait_to_reach(master, st, threshold=3.0, timeout=180, cmd_q=None):
    """OFFBOARD 位置设定值方式飞往 st 目标点, 速度由飞控自主规划.
    途中监听指令修正目标; 返回 'arrived' | 'rtl' | 'hold'(超时保持)"""
    print('正在飞往目标点 (%.6f, %.6f), 速度由飞控自主规划...' % (
        st.lat, st.lon))
    t_end = time.time() + timeout
    last_report = 0.0
    last_goto = 0.0
    d = float('inf')
    while time.time() < t_end:
        act = drain_stdin(st, cmd_q)
        if act == 'rtl':
            print('收到 rtl 指令, 中止导航')
            return 'rtl'
        now = time.time()
        if st.dirty:
            st.dirty = False
            t_end = time.time() + timeout
            print('目标修正 -> (%.6f, %.6f)' % (st.lat, st.lon))
        if now - last_goto >= 1.0:
            goto(master, st.lat, st.lon, st.alt)
            last_goto = now
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'SET_POSITION_TARGET_GLOBAL_INT':
            apply_external_target(st, msg)
            continue
        if mtype != 'GLOBAL_POSITION_INT':
            continue
        d = haversine(msg.lat / 1e7, msg.lon / 1e7, st.lat, st.lon)
        if now - last_report >= 5.0:
            print('  距目标 %.1f m, 高度 %.1f m' % (
                d, msg.relative_alt / 1000.0))
            last_report = now
        if d <= threshold:
            print('[OK] 已到达目标点 (误差 %.1f m)' % d)
            return 'arrived'
    if ON_TIMEOUT == 'rtl':
        print('警告: 飞往目标点超时(最后距目标 %.1f m), 转为返航' % d)
        return 'rtl'
    print('警告: 飞往目标点超时(最后距目标 %.1f m), 悬停保持等待软件指令 '
          '(ON_TIMEOUT=%s)' % (d, ON_TIMEOUT))
    return 'hold'


def wait_landed(master, timeout=300):
    """等待降落并上锁 (PX4: EXTENDED_SYS_STATE.landed_state==ON_GROUND
    + 降落后 COM_DISARM_LAND(2s) 自动上锁; 超时兜底补发 DISARM).
    返回 True=已落地并上锁"""
    print('正在等待降落...')
    t_end = time.time() + timeout
    on_ground = False
    last_report = 0.0
    while time.time() < t_end:
        msg = master.recv_match(type=['EXTENDED_SYS_STATE', 'HEARTBEAT'],
                                blocking=True, timeout=1)
        now = time.time()
        if msg is None:
            if now - last_report >= 10.0:
                print('  降落中... 剩余 %d 秒' % max(int(t_end - now), 0))
                last_report = now
            continue
        if msg.get_type() == 'EXTENDED_SYS_STATE':
            if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                on_ground = True
                print('  已触地 (ON_GROUND), 等待自动上锁 '
                      '(COM_DISARM_LAND=2s)...')
                break
        elif not (msg.base_mode &
                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('[OK] 已上锁 (DISARMED)')
            return True

    t_wait = time.time() + (15.0 if on_ground else 0.0)
    while time.time() < t_wait:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and not (hb.base_mode &
                       mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('[OK] 已落地并上锁')
            return True

    if not master.motors_armed():
        print('[OK] 已落地并上锁')
        return True
    if on_ground:
        print('  已触地但未自动上锁, 补发 DISARM...')
        if disarm(master):
            return True
        print('警告: 补发 DISARM 未确认, 请人工确认电机状态')
        return False
    print('警告: 等待降落超时(%ds), 仍尝试上锁' % timeout)
    if master.motors_armed():
        disarm(master)
    return False


def disarm(master, force=False, timeout=5.0):
    """上锁 (DISARM); 校验 ACK/心跳, 被拒则强制上锁(21196, PX4 同样
    识别该 magic). 返回 True/False"""
    if not master.motors_armed():
        print('[OK] 已是上锁状态 (DISARMED)')
        return True

    param2 = 21196 if force else 0
    print('发送 DISARM%s...' % (' (强制)' if force else ''))
    while master.recv_match(blocking=False):
        pass
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, param2, 0, 0, 0, 0, 0)

    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            if not master.motors_armed():
                print('[OK] 已上锁 (DISARMED)')
                return True
            continue
        mtype = msg.get_type()
        if mtype == 'COMMAND_ACK' and \
                msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                pass
            elif not force:
                print('警告: DISARM 被拒(result=%s), 改用强制上锁...'
                      % msg.result)
                return disarm(master, force=True, timeout=timeout)
            else:
                print('错误: 强制 DISARM 仍失败(result=%s)' % msg.result)
                return False
        elif mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            print('  [飞控] %s' % text)
        elif mtype == 'HEARTBEAT' and not (
                msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('[OK] 已上锁 (DISARMED)')
            return True

    if not master.motors_armed():
        print('[OK] 已上锁 (DISARMED)')
        return True
    print('警告: 上锁超时(%gs), 请人工确认' % timeout)
    return False


def trigger_drop(master, servo_no=10, pwm_on=2000, pwm_off=1000,
                 hold_ms=1500, timeout=2.0):
    """触发抛投器 (MAV_CMD_DO_SET_SERVO): 发触发PWM -> 保持 hold_ms ->
    发回位PWM. 每步校验 COMMAND_ACK; pwm_off=None 表示不回位.
    返回 True/False. 前提: 该通道输出 function=0/未被占用"""
    sno = int(servo_no)
    if not (1 <= sno <= 16):
        raise ValueError('通道号超范围(1~16): %r' % servo_no)
    hold_ms = int(hold_ms)
    if hold_ms < 0:
        raise ValueError('保持时间不能为负: %r' % hold_ms)
    for tag, pwm in (('触发', pwm_on), ('回位', pwm_off)):
        if pwm is None:
            continue
        if not (500 <= int(pwm) <= 2500):
            raise ValueError('%s PWM 超范围(500~2500): %r' % (tag, pwm))

    def _send(pwm):
        while master.recv_match(blocking=False):
            pass
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            float(sno), float(pwm), 0, 0, 0, 0, 0)
        t_end = time.time() + timeout
        while time.time() < t_end:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            mtype = msg.get_type()
            if (mtype == 'COMMAND_ACK'
                    and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO):
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                print('错误: DO_SET_SERVO 被拒(result=%s), '
                      '检查该通道是否被占用(PX4: CA_SV/输出 function)'
                      % msg.result)
                return False
            if mtype == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, (bytes, bytearray)):
                    text = text.decode('utf-8', 'replace')
                print('  [飞控] %s' % text)
        print('错误: DO_SET_SERVO 等待 ACK 超时(%gs)' % timeout)
        return False

    print('抛投: 通道%d -> %d us' % (sno, int(pwm_on)))
    if not _send(pwm_on):
        return False
    if hold_ms > 0:
        time.sleep(hold_ms / 1000.0)
    if pwm_off is None:
        print('[OK] 抛投已触发 (保持 %d us 不回位)' % int(pwm_on))
        return True
    if not _send(pwm_off):
        print('警告: 触发成功但回位失败 (通道%d -> %d us), '
              '请检查抛投器状态' % (sno, int(pwm_off)))
        return False
    print('[OK] 抛投完成: 通道%d %d us -> 保持 %d ms -> 回位 %d us' % (
        sno, int(pwm_on), hold_ms, int(pwm_off)))
    return True


# ============================================================
#  测试流程 / 入口
# ============================================================

def test_flight_modes(master, modes):
    """连接后逐个切换飞行模式, 每一项都确认切换成功后才继续"""
    names = []
    for m in modes:
        up = _resolve_mode(m.strip())
        if up is None:
            sys.exit('错误: 未知飞行模式 %s (PX4: %s)'
                     % (m.strip(), ','.join(sorted(PX4_MODES))))
        names.append(up)

    print('开始飞行模式切换测试: %s' % ' -> '.join(names))
    idx = 1
    for name in names:
        print('  [%d/%d] 切换为 %s ...' % (idx, len(names), name))
        set_mode(master, name)  # 内部会验证模式已生效, 失败即退出
        time.sleep(1)
        idx += 1
    if names[-1] != 'POSCTL':
        print('  最后切回 POSCTL, 准备解锁...')
        set_mode(master, 'POSCTL')
        time.sleep(1)
    print('[OK] 所有飞行模式切换测试通过, 可安全继续')


def fly_to_and_hover(master, args):
    """按 --nav-mode 飞往目标点 -> 悬停等待指令 (ON_TIMEOUT='hold':
    悬停期间收到新目标则继续飞往新点; rtl 立即返航; 'hold' 无限悬停等
    软件指令(自动返航已禁用), 'rtl' 保持旧行为: 等待窗内无新指令则返航)"""
    cmd_q = start_stdin_listener()
    st = MissionState(args.latitude, args.longitude, args.altitude,
                      args.hover)

    while True:
        if args.nav_mode == 'position':
            goto(master, st.lat, st.lon, st.alt)
            leg = wait_to_reach(master, st, threshold=args.threshold,
                                cmd_q=cmd_q)
        else:
            leg = fly_to_target(master, st, speed=args.speed,
                                threshold=args.threshold, cmd_q=cmd_q)
        if leg == 'rtl':
            break

        if ON_TIMEOUT == 'hold':
            print('悬停保持, 自动返航已禁用, 等待软件指令 '
                  '(rtl=立即返航, 新目标=继续飞)')
            deadline = None
        else:
            print('已到达, 悬停等待指令 %.1f 秒, 无新指令则返航...'
                  % st.hover)
            deadline = time.time() + st.hover
        last_goto = 0.0
        last_report = 0.0
        reroute = False
        while deadline is None or time.time() < deadline:
            act = drain_stdin(st, cmd_q)
            if act == 'rtl':
                break
            if act == 'hover':
                if deadline is None:
                    print('  悬停保持中(自动返航已禁用), 无需重置等待窗')
                else:
                    deadline = time.time() + st.hover
                    print('  等待窗重置, 剩余悬停等待 %.1f 秒' % st.hover)
            now = time.time()
            if st.dirty:
                st.dirty = False
                reroute = True
                break
            if now - last_goto >= 1.0:
                goto(master, st.lat, st.lon, st.alt)
                last_goto = now
            if now - last_report >= 5.0:
                if deadline is None:
                    print('  悬停中, 自动返航已禁用, 等待软件指令')
                else:
                    left = deadline - now
                    print('  悬停中, 距自动返航还有 %.0f 秒' % max(left, 0))
                last_report = now
            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg and msg.get_type() == 'SET_POSITION_TARGET_GLOBAL_INT':
                apply_external_target(st, msg)
        if reroute:
            print('收到新目标, 继续飞行...')
            continue
        break

    print('进入返航 (RTL)...')
    set_mode(master, 'RTL')      # 先离开 OFFBOARD
    stop_offboard_stream(master)  # 再停设定值流
    if wait_landed(master):
        if master.motors_armed():
            disarm(master)
    else:
        if master.motors_armed():
            print('警告: 降落未确认, 仍尝试上锁(失败则请遥控器处理)')
            disarm(master)


def run_test_modes(master, args, test_modes):
    """测试1: 仅测试飞行模式切换"""
    print('\n===== 测试1: 飞行模式切换测试 =====')
    if not args.no_gps:
        wait_gps_lock(master)
    test_flight_modes(master, test_modes)
    print('[OK] 测试1完成: 飞行模式切换全部成功')


def run_test_arm(master, args):
    """测试2: 测试解锁(延长自动上锁超时后立即验证, 并尽快上锁)"""
    print('\n===== 测试2: 解锁测试 =====')
    if not args.no_gps:
        wait_gps_lock(master)
    set_mode(master, 'POSCTL')
    arm(master)
    print('[OK] 测试2完成: 解锁成功(ARMED)')
    if not disarm(master):
        print('     上锁未确认, 请检查电机状态')
    else:
        print('     已重新上锁, 测试结束')


def run_test_full(master, args, test_modes):
    """测试3: 完整任务(模式切换 + 解锁起飞一体 + 飞往GPS可修正 +
    悬停等指令, ON_TIMEOUT='hold' 无限悬停等软件指令, 不自动返航)"""
    print('\n===== 测试3: 完整飞行任务测试 =====')
    if not args.no_gps:
        wait_gps_lock(master)
    test_flight_modes(master, test_modes)
    arm_and_takeoff(master, args.altitude, mode='POSCTL')
    fly_to_and_hover(master, args)
    print('[OK] 测试3完成: 完整任务执行成功')


def run_test_mission(master, args):
    """测试4/mission: 多GPS航点任务(上传WAYPOINT -> 解锁起飞 ->
    MISSION按序飞行 -> 结束RTL/LAND)"""
    print('\n===== 测试4: WAYPOINT 航点任务 =====')
    wps = args.waypoints_list
    print('航点数: %d, 结束动作: %s' % (len(wps), args.mission_end.upper()))
    for i, (lat, lon, alt) in enumerate(wps):
        print('  [%d] %.6f, %.6f  高度 %.1f m' % (i + 1, lat, lon, alt))

    if not args.no_gps:
        wait_gps_lock(master)

    print('[1/5] 上传航点任务到飞控...')
    if not upload_mission(master, wps, end_action=args.mission_end):
        sys.exit('错误: 航点任务上传失败, 已中止')

    print('[2/5] 切换 POSCTL 并解锁起飞(一体)...')
    take_alt = max([args.altitude] + [w[2] for w in wps])
    arm_and_takeoff(master, take_alt, mode='POSCTL')

    print('[4/5] 切换 MISSION 开始执行航点任务...')
    reset_mission_to_start(master)
    set_mode(master, 'MISSION')

    ok = wait_auto_mission(master, n_wp=len(wps),
                           end_action=args.mission_end,
                           timeout=args.mission_timeout)

    if ok and args.mission_end in ('rtl', 'land'):
        print('[5/5] 等待降落...')
        if wait_landed(master):
            if master.motors_armed():
                disarm(master)
        elif master.motors_armed():
            print('警告: 降落未确认, 仍尝试上锁(失败则请遥控器处理)')
            disarm(master)
    elif ok and args.mission_end == 'none':
        print('[5/5] 任务结束, 手动返航...')
        set_mode(master, 'RTL')
        if wait_landed(master):
            if master.motors_armed():
                disarm(master)
        elif master.motors_armed():
            print('警告: 降落未确认, 仍尝试上锁(失败则请遥控器处理)')
            disarm(master)
    else:
        print('警告: 航点任务未确认完成, 请检查日志/遥控器接管')
        return
    if ok:
        print('[OK] 测试4完成: 航点任务执行成功')


def main():
    parser = argparse.ArgumentParser(
        description='PX4 MAVLink 控制脚本, 可按参数选择测试项目')
    parser.add_argument('connection', nargs='?', default='COM23',
                        help='连接方式: 串口(如 COM3 / /dev/ttyUSB0) 或 '
                             'UDP/TCP(如 udp:127.0.0.1:14550, '
                             'tcp:127.0.0.1:5760, SITL 默认5760)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='串口波特率(默认 115200, 图传一般 57600)')
    parser.add_argument('--test', default='3',
                        help='要执行的测试: 1=飞行模式切换, 2=解锁, '
                             '3=完整任务(单点GPS可修正->悬停等指令), '
                             '4=航点任务(多GPS上传MISSION飞行), '
                             '默认 3; 参数读写校验见 '
                             'px4_control_test.py 菜单11')
    parser.add_argument('--latitude', type=float,
                        help='目标点纬度(度), 仅测试3需要')
    parser.add_argument('--longitude', type=float,
                        help='目标点经度(度), 仅测试3需要')
    parser.add_argument('--waypoints', default='',
                        help='航点串,仅测试4: "lat,lon[,alt];..." '
                             '(分号/换行分隔, alt缺省用--altitude)')
    parser.add_argument('--waypoints-file', default='',
                        help='航点文件路径,仅测试4: 每行 lat,lon[,alt], '
                             '支持#注释')
    parser.add_argument('--mission-end', choices=['rtl', 'land', 'none'],
                        default='none',
                        help='航点任务结束动作: none=停在末航点保持(默认), '
                             'rtl=返航降落, land=末点降落')
    parser.add_argument('--mission-timeout', type=float, default=600.0,
                        help='航点任务等待超时,秒(默认 600)')
    parser.add_argument('--altitude', type=float, default=20.0,
                        help='飞行高度,米(默认 20)')
    parser.add_argument('--hover', type=float, default=10.0,
                        help='到达目标后的悬停等待时间,秒(默认 10); '
                             'ON_TIMEOUT=rtl 时生效')
    parser.add_argument('--threshold', type=float, default=3.0,
                        help='判定到达目标的水平距离误差,米(默认 3)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='巡航速度,米/秒(默认1.0, 越小油门越小), '
                             '仅 --nav-mode speed 生效')
    parser.add_argument('--nav-mode', choices=['speed', 'position'],
                        default='speed',
                        help='飞往目标的导航方式: speed=OFFBOARD速度流'
                             '低速巡航最小油门(默认), position=OFFBOARD'
                             '位置设定值+飞控自主速度规划')
    parser.add_argument('--test-modes', default='POSCTL,LOITER,RTL',
                        help='模式切换测试的飞行模式列表,逗号分隔'
                             '(默认 POSCTL,LOITER,RTL), 每项都会确认切换'
                             '成功, 测试后自动停在 POSCTL')
    parser.add_argument('--no-gps', action='store_true',
                        help='跳过 GPS 锁星等待(测试1/2在室内等场景使用)')
    args = parser.parse_args()

    test_map = {
        '1': 'modes', '2': 'arm', '3': 'full', '4': 'mission',
        'modes': 'modes', 'arm': 'arm', 'full': 'full',
        'mission': 'mission',
    }
    if args.test.strip().lower() not in test_map:
        sys.exit('错误: 无效的 --test 值 %r, 可选 1(modes)/2(arm)/3(full)'
                 '/4(mission)' % args.test)
    test_choice = test_map[args.test.strip().lower()]

    if test_choice == 'full':
        if args.latitude is None or args.longitude is None:
            sys.exit('错误: 完整任务(测试3)必须提供 --latitude 和 '
                     '--longitude')
        if not (-90 <= args.latitude <= 90) or \
                not (-180 <= args.longitude <= 180):
            sys.exit('错误: 纬度/经度超出有效范围')
        if args.speed <= 0:
            sys.exit('错误: --speed 必须大于 0')
        if args.hover <= 0:
            sys.exit('错误: --hover 必须大于 0')

    args.waypoints_list = []
    if test_choice == 'mission':
        if args.waypoints_file:
            args.waypoints_list.extend(
                load_waypoints_file(args.waypoints_file, args.altitude))
        if args.waypoints:
            args.waypoints_list.extend(
                parse_waypoints(args.waypoints, args.altitude))
        if not args.waypoints_list:
            sys.exit('错误: 航点任务(测试4)必须提供 --waypoints 或 '
                     '--waypoints-file')
        if args.mission_timeout <= 0:
            sys.exit('错误: --mission-timeout 必须大于 0')

    test_modes = [m.strip() for m in args.test_modes.split(',') if m.strip()]
    if not test_modes:
        sys.exit('错误: --test-modes 不能为空')

    master = mavconnect(args.connection, args.baud)

    if test_choice == 'modes':
        run_test_modes(master, args, test_modes)
    elif test_choice == 'arm':
        run_test_arm(master, args)
    elif test_choice == 'mission':
        run_test_mission(master, args)
    else:
        run_test_full(master, args, test_modes)
    stop_offboard_stream(master)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n已中止, 请立即用遥控器接管飞机!')
        stop_offboard_stream()
        sys.exit(1)
