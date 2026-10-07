#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
APM / ArduPilot 飞控 MAVLink 控制脚本
功能流程:
  1. 连接飞控(串口 / UDP / TCP)
  2. 等待 GPS 锁星
  3. 逐个测试飞行模式切换(默认 GUIDED/Loiter/RTL), 每项确认成功后继续
  4. 解锁 (ARM)
  5. 起飞到指定高度
  6. 飞往指定 GPS 坐标; 飞行途中可用地面站持续修正目标点
  7. 到达后悬停等待指令 (v1.21: ON_TIMEOUT='hold' 无限悬停等软件指令,
     自动返航已禁用; 'rtl' 恢复旧行为: --hover 秒内无新指令则返航上锁)

WAYPOINT 航点任务 (--test 4 / mission):
  将多个 GPS 组成航点任务上传飞控, 起飞后切 AUTO 按序飞行,
  末项动作 --mission-end none(默认,停末航点保持)/rtl/land.
  任务条目支持抛投项 ('DROP', 通道, 触发PWM, 回位PWM, 保持ms), 展开为
  DO_SET_SERVO + NAV_DELAY(保持) + DO_SET_SERVO(回位); 单次触发用
  trigger_drop() (实时 COMMAND_LONG, 每步校验 ACK).
  前提: 飞控参数 SERVO<通道>_FUNCTION = 0 (Disabled).

目标修正/指令来源:
  A. 其他地面站经 MAVLink 下发 SET_POSITION_TARGET_GLOBAL_INT(同链路可见)
  B. 本脚本运行中在终端直接输入指令(回车确认):
       31.2304,121.4737        修正目标(纬度,经度, 高度不变)
       31.2304,121.4737,15     修正目标(纬度,经度,相对高度)
       hover 30                更新悬停等待时间(秒)
       rtl                     立即返航

PID 自动调参请使用: apm_control_test.py 菜单项 12

依赖: pip install pymavlink

示例:
  串口(APM/Pixhawk):
    python apm_control.py COM3 --latitude 31.2304 --longitude 121.4737 --altitude 20
  串口指定波特率(图传一般 57600):
    python apm_control.py /dev/ttyUSB0 --baud 57600 --latitude 31.2304 --longitude 121.4737 --altitude 20
  UDP(数传/局域网):
    python apm_control.py udp:127.0.0.1:14550 --latitude 31.2304 --longitude 121.4737 --altitude 20
  SITL 仿真测试:
    python apm_control.py tcp:127.0.0.1:5760 --latitude 31.2304 --longitude 121.4737 --altitude 20
  多航点任务(上传后起飞按序飞行, 结束动作默认停末航点):
    python apm_control.py COM3 --test 4 --waypoints "31.2304,121.4737,20;31.2310,121.4745,25;31.2320,121.4750,20"
    python apm_control.py COM3 --test 4 --waypoints-file route.txt --mission-end rtl
"""

import argparse
import math
import queue
import sys
import threading
import time

from pymavlink import mavutil

import apm_control_test as tct

# ArduCopter 模式编号(新老固件通用)
COPTER_MODE = {
    'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
    'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'LAND': 9,
}

DEG2RAD = math.pi / 180.0
EARTH_RADIUS = 6371000.0

# 解锁后未起飞被自动上锁的参数: DISARM_DELAY(秒), 默认约10s
# 0=不涉及; v1.30 起系统不自动写该参数, 仅提示, 由界面[写入发射箱参数]手动写
DISARM_DELAY_SEC = 60

# v1.21 发射箱: "到不了目标/悬停无指令"时的策略
#   'hold' = 悬停保持等软件指令 (禁自动返航, 符合"只有软件能命令动作")
#   'rtl'  = 超时自动返航降落 (旧行为, 地面交互兜底)
ON_TIMEOUT = 'hold'


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


def mavconnect(conn_str, baud):
    """建立 MAVLink 连接并等待心跳"""
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
    print('[OK] 已连接: sysid=%u compid=%u 机型=%s' % (
        hb.get_srcSystem(), hb.get_srcComponent(), type_name))
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


def set_mode(master, mode_name):
    """切换飞行模式并等待心跳确认生效"""
    mode_name = mode_name.upper()
    mode_map = None
    if hasattr(master, 'mode_mapping'):
        try:
            mode_map = master.mode_mapping()
        except Exception:
            mode_map = None
    mode_id = (mode_map or {}).get(mode_name, COPTER_MODE.get(mode_name))
    if mode_id is None:
        sys.exit('错误: 未知飞行模式 %s' % mode_name)

    sent = False
    if mode_map and mode_name in mode_map and hasattr(master, 'set_mode_apm'):
        try:
            master.set_mode_apm(mode_name)
            sent = True
        except Exception:
            sent = False
    if not sent:
        # 兜底: 使用 DO_SET_MODE 命令切换
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)

    deadline = time.time() + 10.0
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        if master.target_system and hb.get_srcSystem() != master.target_system:
            continue
        if hb.custom_mode == mode_id:
            print('[OK] 飞行模式: %s' % mode_name)
            return
    sys.exit('错误: 切换到 %s 模式失败(未解锁或飞控配置不支持)' % mode_name)


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
        pn = msg.param_id
        if isinstance(pn, (bytes, bytearray)):
            pn = pn.decode('ascii', 'replace')
        if str(pn).split('\x00')[0] == name:
            print('[OK] 参数 %s = %g' % (name, float(msg.param_value)))
            return True
    return False


def ensure_disarm_delay(master):
    """解锁/起飞前不再写 DISARM_DELAY (v1.30: 系统只做飞行控制, 参数写入
    由用户在界面点[写入发射箱参数]手动触发). 这里只打印提示, 不发参数指令."""
    if not DISARM_DELAY_SEC or DISARM_DELAY_SEC <= 0:
        return
    print('提示: DISARM_DELAY=%ds 未写入(系统不自动写飞控参数), '
          '需要时请在界面点[写入发射箱参数]' % DISARM_DELAY_SEC)


def collect_status_text(master, duration=2.0):
    """收集并打印 STATUSTEXT(PreArm 等), 返回最后一条"""
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


def arm(master):
    """解锁 (ARM): 发射箱参数/DISARM_DELAY 只提示不写入(v1.30, 写入由用户
    手动触发) -> RC覆盖油门最低 -> 校验 ACK + ARMED + PreArm 文本"""
    ensure_disarm_delay(master)
    tct.ensure_launch_params(master)

    # 油门摇杆常在中位 -> PreArm: Throttle high; 解锁期间强制最低
    print('RC 覆盖油门=1000, 发送 ARM...')
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1000, 1500, 0, 0, 0, 0)
    time.sleep(0.4)

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
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                1500, 1500, 1000, 1500, 0, 0, 0, 0)
            continue
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            last_text = str(text)
            print('  [飞控] %s' % last_text)
        elif mtype == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print('  ARM 命令已接受, 等待 ARMED...')
            else:
                ack = msg
                break
        elif mtype == 'HEARTBEAT' and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            ack = None
            break

    if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        more = collect_status_text(master, duration=1.5)
        tct.release_rc_override(master)
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
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                break
            if not last_text:
                last_text = collect_status_text(master, duration=0.3)
        if not last_text:
            collect_status_text(master, duration=0.5)

    tct.release_rc_override(master)
    time.sleep(0.2)

    if master.motors_armed():
        print('[OK] 已解锁 (ARMED)')
        return

    if not last_text:
        last_text = collect_status_text(master, duration=1.0)
    sys.exit('错误: 解锁后未检测到 ARMED%s' % (
        ('; 飞控: ' + last_text) if last_text else ''))


def takeoff(master, alt):
    """起飞到指定高度(GUIDED); 解锁后应尽快调用, 避免 DISARM_DELAY 自动上锁"""
    print('正在起飞至 %.1f 米...' % alt)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, float(alt))

    t_end = time.time() + 10.0
    while time.time() < t_end:
        a = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if a and a.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if a.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                collect_status_text(master, duration=1.0)
                sys.exit('错误: 起飞命令被拒绝 (result=%s)' % a.result)
            break

    t_end = time.time() + 30.0
    rel = 0.0
    while time.time() < t_end:
        p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if p:
            rel = p.relative_alt / 1000.0
            if rel >= alt - 1.0:
                print('[OK] 已到达目标高度 %.1f 米' % rel)
                print('提示: GUIDED 下摇杆无效属正常, '
                      '拨遥控器模式开关到 LOITER 可接管')
                return
        if not master.motors_armed():
            sys.exit('错误: 起飞过程中电机已被上锁(DISARM), 可能触发 DISARM_DELAY')
    sys.exit('错误: 起飞超时, 最后高度仅 %.1f 米' % rel)


def arm_and_takeoff(master, alt, mode='GUIDED'):
    """解锁+立即起飞一体(测试3/4): 避免 arm 后未及时 takeoff 被 DISARM_DELAY 上锁"""
    if mode.upper() != 'AUTO':
        set_mode(master, mode)
    arm(master)
    print('[起飞] 解锁后立即 NAV_TAKEOFF -> %.1f 米' % alt)
    takeoff(master, alt)


def goto(master, lat, lon, alt):
    """发送位置指令(GUIDED 模式下调用), 仅指定位置, 速度/朝向由飞控自行规划"""
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # type_mask: 忽略速度/加速度/偏航, 只控制位置
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        int(alt),
        0, 0, 0,  # vx, vy, vz
        0, 0, 0,  # afx, afy, afz
        0, 0)    # yaw, yaw_rate


def set_velocity(master, vx, vy, vz=0.0):
    """发送速度指令(仅控制速度, 位置/偏航由飞控自行保持),
    用于以低速度/低油门巡航, 到达后传 (0,0,0) 悬停"""
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111000111,  # type_mask: 忽略位置/加速度/偏航, 只使用 vx/vy/vz
        0, 0, 0,          # lat/lon/alt(忽略, 填0)
        vx, vy, vz,       # 北东地速度分量
        0, 0, 0,          # afx, afy, afz
        0, 0)             # yaw, yaw_rate


def parse_waypoints(spec, default_alt):
    """解析航点串: 'lat,lon[,alt];lat,lon[,alt]' (分号或换行分隔), alt 缺省用 default_alt
    返回 [(lat, lon, alt), ...]"""
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


# MAV_MISSION_RESULT 枚举名(common.xml), 用于把上传拒绝码翻译成人话
MISSION_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'ERROR', 2: 'UNSUPPORTED_FRAME', 3: 'UNSUPPORTED',
    4: 'NO_SPACE(任务超容量)', 5: 'INVALID', 6: 'INVALID_PARAM1',
    7: 'INVALID_PARAM2', 8: 'INVALID_PARAM3', 9: 'INVALID_PARAM4',
    10: 'INVALID_PARAM5_X', 11: 'INVALID_PARAM6_Y', 12: 'INVALID_PARAM7',
    13: 'INVALID_SEQUENCE', 14: 'DENIED', 15: 'OPERATION_CANCELLED',
}


def upload_mission(master, wps, end_action='rtl', timeout=30.0):
    """通过 MAVLink 任务协议上传航点任务到飞控.
    wps: [(lat, lon, alt), ...] 与 ('DROP', 通道, 触发PWM, 回位PWM, 保持ms) 混合;
    DROP 展开为 DO_SET_SERVO(触发) [+NAV_DELAY 保持] + DO_SET_SERVO(回位).
    end_action: rtl/land/none 追加的结束指令. 成功返回任务项数, 失败 False.
    ArduPilot 的 seq0 专供 home(mavlink.io: ArduPilot's first mission seq is
    the home position), 因此线上多发一项 count+1, seq0 复用首项; 少发则
    首个航点被 home 槽静默吞掉. 返回值仍是真实任务项数(不含 home)."""
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
                # param4 必须有限值: ArduPilot 逐项 NaN 校验, NaN 会被
                # MAV_MISSION_INVALID_PARAM4 拒绝整份任务
                'param3': 0.0, 'param4': 0.0,
                'x': 0, 'y': 0, 'z': 0.0,
            }
        seq = [_do(s_on)]
        if hold_ms and float(hold_ms) > 0:
            seq.append({
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                'command': mavutil.mavlink.MAV_CMD_NAV_DELAY,
                'current': 0,
                # param1=保持秒(向上取整); param2~4=-1 表示不按时刻触发
                'param1': float(max(1, int(math.ceil(float(hold_ms) / 1000.0)))),
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
            'param1': 0.0,   # 到点悬停秒数(0=不停留, 由飞控默认)
            'param2': 2.0,   # 到点半径(米)
            'param3': 0.0,
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
            # RTL 曾用 param4=NaN 被 ArduPilot 拒绝(=9 INVALID_PARAM4,
            # 固件早于 ardupilot#31658 无 RTL 的 nan 豁免), 改有限 0
            'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
            'x': 0, 'y': 0, 'z': 0.0,
        })
    elif end_action == 'land':
        # 末航点处降落
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
    wire_count = count + 1
    print('开始上传任务: %d 航点, %d 抛投, 结束动作 %s ...' % (
        n_wp, n_drop, end_action.upper()))

    def _send_item(seq):
        it = items[0] if seq == 0 else items[min(seq - 1, count - 1)]
        master.mav.mission_item_int_send(
            master.target_system, master.target_component,
            seq, it['frame'], it['command'], it['current'], 1,
            it['param1'], it['param2'], it['param3'], it['param4'],
            it['x'], it['y'], it['z'])

    master.mav.mission_count_send(
        master.target_system, master.target_component, wire_count)

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
            if 0 <= seq < wire_count:
                _send_item(seq)
                print('  发送任务项 %d/%d' % (seq, wire_count))
        elif mtype == 'MISSION_ACK':
            ack = msg.type
            if ack == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print('[OK] 任务上传成功 (%d 项)' % count)
                return count
            print('错误: 任务上传被拒绝, MAV_MISSION_RESULT=%s %s'
                  % (ack, MISSION_RESULT_NAMES.get(int(ack), '未知')) +
                  (' (param4无效: 该项 param4 不可为 NaN)'
                   if int(ack) == 9 else ''))
            return False
    print('错误: 任务上传超时')
    return False


def reset_mission_to_start(master, timeout=2.0):
    """把任务指针复位到第 1 项(进 AUTO 前调用).
    先发 MAV_CMD_DO_SET_MISSION_CURRENT(param1=0, param2=0)取 COMMAND_ACK;
    被拒/无 ACK 再回退 MISSION_SET_CURRENT(0) 消息 — ArduPilot 只在
    set_current_cmd 成功时才回显 MISSION_CURRENT(seq=0), 故回显即确认.
    best-effort: 两条都不确认只告警, 仍按当前指针继续执行. 返回 True/False"""
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
    print('  ⏳ 改用 MISSION_SET_CURRENT(0) 复位任务指针...')
    master.mav.mission_set_current_send(
        master.target_system, master.target_component, 0)
    t_end = time.time() + 1.5
    while time.time() < t_end:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            break
        if master.target_system and msg.get_srcSystem() != master.target_system:
            continue
        if msg.seq == 0:
            print('[OK] 任务指针已复位到第 1 项')
            return True
        break
    print('  ⚠️ 任务指针复位未确认(固件较旧?), 按当前指针执行')
    return False


def wait_auto_mission(master, n_wp, end_action, timeout=600.0):
    """AUTO 模式下等待航点任务完成.
    监视 MISSION_CURRENT 与心跳模式; 末项 rtl/land 由飞控切入时判定成功.
    end_action=none 时以到达最后航点为准. 返回 True/False"""
    print('AUTO 任务执行中 (共 %d 航点, 结束=%s), 超时 %d 秒...' % (
        n_wp, end_action.upper(), int(timeout)))
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
                shown = min(last_seq, n_wp)
                print('  任务进度: 航点 %d/%d (seq=%d)' % (
                    shown, n_wp, last_seq))
                last_report = now
        elif mtype == 'HEARTBEAT':
            if master.target_system and msg.get_srcSystem() != master.target_system:
                continue
            cm = msg.custom_mode
            if cm == COPTER_MODE['RTL']:
                print('[OK] 任务结束, 飞控已进入 RTL 返航')
                return True
            if cm == COPTER_MODE['LAND'] and end_action == 'land':
                print('[OK] 任务结束, 飞控已进入 LAND 降落')
                return True
            if cm != COPTER_MODE['AUTO'] and last_seq >= 0:
                name = _mode_name_from_id(cm)
                if end_action == 'none' and last_seq >= n_wp:
                    print('[OK] 任务完成, 已离开 AUTO (当前 %s)' % name)
                    return True
                print('警告: 任务中离开 AUTO 模式 -> %s' % name)
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

        if end_action == 'none' and last_seq >= n_wp:
            now = time.time()
            if hold_since is None:
                hold_since = now
            elif now - hold_since >= 3.0:
                print('[OK] 已到达最后航点, 任务完成 (停留在 AUTO)')
                return True
        else:
            hold_since = None
    print('错误: 任务执行超时 (最后 seq=%d)' % last_seq)
    return False


def _mode_name_from_id(mode_id):
    for name, mid in COPTER_MODE.items():
        if mid == mode_id:
            return name
    return 'UNKNOWN(%s)' % mode_id


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
            print('未知指令: %s  (支持: lat,lon | lat,lon,alt | hover 秒 | rtl)' % line)
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
    """处理地面站下发的 SET_POSITION_TARGET_GLOBAL_INT, 修正目标; 返回是否有变化"""
    if (msg.type_mask & 0b11) != 0:
        return False  # 未使用经纬度(如纯速度指令)
    if msg.lat == 0 and msg.lon == 0:
        return False
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    if abs(lat - st.lat) < 1e-7 and abs(lon - st.lon) < 1e-7:
        return False  # 与当前目标一致(含自身回显), 忽略
    alt = None
    if (msg.type_mask & 0b100) == 0 and msg.alt:
        if msg.frame in (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
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
    """以指定巡航速度(speed, 米/秒)低速飞往 st 目标点.
    途中监听 stdin 指令与地面站 SET_POSITION_TARGET_GLOBAL_INT, 可随时修正目标.
    返回 'arrived' | 'rtl'(显式指令) | 'hold'(超时且 ON_TIMEOUT='hold')"""
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
        vx = speed * math.sin(brg)
        vy = speed * math.cos(brg)
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
    print('警告: 飞往目标点超时(最后距目标 %.1f m), 悬停保持等待软件指令'
          ' (ON_TIMEOUT=%s)' % (d, ON_TIMEOUT))
    return 'hold'


def wait_to_reach(master, st, threshold=3.0, timeout=180, cmd_q=None):
    """位置规划方式飞往 st 目标点, 速度由飞控自主规划.
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
    print('警告: 飞往目标点超时(最后距目标 %.1f m), 悬停保持等待软件指令'
          ' (ON_TIMEOUT=%s)' % (d, ON_TIMEOUT))
    return 'hold'


def wait_landed(master, timeout=300):
    """等待降落: ≤1m 时由 tct.land_soft_final 接管 (切 ALT_HOLD 油门
    微调缓降触地并上锁); timeout 为等待降至接管高度的超时"""
    print('正在返航降落 (≤1m 油门微调软着陆)...')
    if tct.land_soft_final(master, phase0_timeout=timeout):
        print('[OK] 已软着陆并上锁')
        return True
    print('警告: 等待降落超时, 请人工确认飞机状态')
    return False


def disarm(master, force=False, timeout=5.0):
    """上锁 (DISARM); 校验 ACK/心跳, 被拒则强制上锁(21196). 返回 True/False"""
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
        if mtype == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                pass
            elif not force:
                print('警告: DISARM 被拒(result=%s), 改用强制上锁...' % msg.result)
                return disarm(master, force=True, timeout=timeout)
            else:
                print('错误: 强制 DISARM 仍失败(result=%s)' % msg.result)
                return False
        elif mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            print('  [飞控] %s' % text)
        elif mtype == 'HEARTBEAT' and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('[OK] 已上锁 (DISARMED)')
            return True

    if not master.motors_armed():
        print('[OK] 已上锁 (DISARMED)')
        return True
    print('警告: 上锁超时(%gs), 请人工确认' % timeout)
    return False


def trigger_drop(master, servo_no=10, pwm_on=2000, pwm_off=1000,
                 hold_ms=1500, timeout=2.0):
    """触发抛投器 (MAV_CMD_DO_SET_SERVO): 发触发PWM -> 保持 hold_ms -> 发回位PWM.
    每步校验 COMMAND_ACK; pwm_off=None 表示不回位. 返回 True/False.
    前提: 飞控参数 SERVO<通道>_FUNCTION = 0 (Disabled)"""
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
                      '检查 SERVO%d_FUNCTION=0' % (msg.result, sno))
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


def test_flight_modes(master, modes):
    """连接后逐个切换飞行模式, 每一项都确认切换成功后才继续"""
    names = []
    for m in modes:
        up = m.strip().upper()
        if up not in COPTER_MODE:
            sys.exit('错误: 未知飞行模式 %s' % up)
        names.append(up)

    print('开始飞行模式切换测试: %s' % ' -> '.join(names))
    idx = 1
    for name in names:
        print('  [%d/%d] 切换为 %s ...' % (idx, len(names), name))
        set_mode(master, name)  # 内部会验证模式已生效, 失败即退出
        time.sleep(1)
        idx += 1
    if names[-1] != 'GUIDED':
        print('  最后切回 GUIDED, 准备解锁...')
        set_mode(master, 'GUIDED')
        time.sleep(1)
    print('[OK] 所有飞行模式切换测试通过, 可安全继续')


def fly_to_and_hover(master, args):
    """按 --nav-mode 飞往目标点 -> 悬停等待指令 (v1.21 ON_TIMEOUT='hold'):
    悬停期间收到新目标则继续飞往新点; hover 指令重置等待窗(仅 rtl 策略);
    rtl 立即返航; ON_TIMEOUT='hold' 时无限悬停等软件指令(自动返航已禁用),
    'rtl' 时保持旧行为: 等待窗(--hover, 默认10秒)内无新指令 -> 返航"""
    cmd_q = start_stdin_listener()
    st = MissionState(args.latitude, args.longitude, args.altitude, args.hover)

    while True:
        if args.nav_mode == 'position':
            goto(master, st.lat, st.lon, st.alt)
            leg = wait_to_reach(master, st, threshold=args.threshold, cmd_q=cmd_q)
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
            print('已到达, 悬停等待指令 %.1f 秒, 无新指令则返航...' % st.hover)
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
    set_mode(master, 'RTL')
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
    """测试2: 测试解锁(延长 DISARM_DELAY 后立即验证, 并尽快上锁)"""
    print('\n===== 测试2: 解锁测试 =====')
    if not args.no_gps:
        wait_gps_lock(master)
    set_mode(master, 'GUIDED')
    arm(master)
    print('[OK] 测试2完成: 解锁成功(ARMED)')
    if not disarm(master):
        print('     上锁未确认, 请检查电机状态')
    else:
        print('     已重新上锁, 测试结束')


def run_test_full(master, args, test_modes):
    """测试3: 完整任务(模式切换 + 解锁起飞一体 + 飞往GPS可修正 + 悬停等指令,
    ON_TIMEOUT='hold' 时无限悬停等软件指令, 不自动返航)"""
    print('\n===== 测试3: 完整飞行任务测试 =====')
    if not args.no_gps:
        wait_gps_lock(master)
    test_flight_modes(master, test_modes)
    arm_and_takeoff(master, args.altitude, mode='GUIDED')
    fly_to_and_hover(master, args)
    print('[OK] 测试3完成: 完整任务执行成功')


def run_test_mission(master, args):
    """测试4/mission: 多GPS航点任务(上传WAYPOINT -> 解锁起飞 -> AUTO按序飞行 -> 结束RTL/LAND)"""
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

    print('[2/5] 切换 GUIDED 并解锁起飞(一体)...')
    take_alt = max([args.altitude] + [w[2] for w in wps])
    arm_and_takeoff(master, take_alt, mode='GUIDED')

    print('[4/5] 切换 AUTO 开始执行航点任务...')
    reset_mission_to_start(master)
    set_mode(master, 'AUTO')

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
        description='APM/ArduPilot MAVLink 控制脚本, 可按参数选择测试项目')
    parser.add_argument('connection', nargs='?', default='COM23',
                        help='连接方式: 串口(如 COM3 / /dev/ttyUSB0) 或 '
                             'UDP/TCP(如 udp:127.0.0.1:14550, tcp:127.0.0.1:5760)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='串口波特率(默认 115200)')
    parser.add_argument('--test', default='3',
                        help='要执行的测试: 1=飞行模式切换, 2=解锁, '
                             '3=完整任务(单点GPS可修正->悬停等指令), '
                             '4=航点任务(多GPS上传AUTO飞行), '
                             '默认 3; PID调参见 apm_control_test.py 菜单11')
    parser.add_argument('--latitude', type=float,
                        help='目标点纬度(度), 仅测试3需要')
    parser.add_argument('--longitude', type=float,
                        help='目标点经度(度), 仅测试3需要')
    parser.add_argument('--waypoints', default='',
                        help='航点串,仅测试4: "lat,lon[,alt];lat,lon[,alt];..." '
                             '(分号/换行分隔, alt缺省用--altitude)')
    parser.add_argument('--waypoints-file', default='',
                        help='航点文件路径,仅测试4: 每行 lat,lon[,alt], 支持#注释')
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
                             '期间收到新指令则继续, 无指令超时则RTL; '
                             '运行中可用 "hover N" 指令修改')
    parser.add_argument('--threshold', type=float, default=3.0,
                        help='判定到达目标的水平距离误差,米(默认 3)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='巡航速度,米/秒(默认1.0, 越小油门越小; '
                             '最小可飞速度约为0.5~1.0), 仅 --nav-mode speed 生效')
    parser.add_argument('--nav-mode', choices=['speed', 'position'], default='speed',
                        help='飞往目标的导航方式: speed=低速巡航最小油门(默认), '
                             'position=位置+飞控自主速度规划')
    parser.add_argument('--test-modes', default='GUIDED,LOITER,RTL',
                        help='模式切换测试的飞行模式列表,逗号分隔(默认 GUIDED,LOITER,RTL),'
                             '每项都会确认切换成功, 测试后自动停在 GUIDED')
    parser.add_argument('--no-gps', action='store_true',
                        help='跳过 GPS 锁星等待(测试1/2在室内等场景使用)')
    args = parser.parse_args()

    test_map = {
        '1': 'modes', '2': 'arm', '3': 'full', '4': 'mission',
        'modes': 'modes', 'arm': 'arm', 'full': 'full', 'mission': 'mission',
    }
    if args.test.strip().lower() not in test_map:
        sys.exit('错误: 无效的 --test 值 %r, 可选 1(modes)/2(arm)/3(full)/4(mission)'
                 % args.test)
    test_choice = test_map[args.test.strip().lower()]

    if test_choice == 'full':
        if args.latitude is None or args.longitude is None:
            sys.exit('错误: 完整任务(测试3)必须提供 --latitude 和 --longitude')
        if not (-90 <= args.latitude <= 90) or not (-180 <= args.longitude <= 180):
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
            sys.exit('错误: 航点任务(测试4)必须提供 --waypoints 或 --waypoints-file')
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


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n已中止, 请立即用遥控器接管飞机!')
        sys.exit(1)
