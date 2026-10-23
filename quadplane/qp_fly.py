#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QuadPlane (垂起固定翼) 基础流程控制: 解锁 / 起飞到指定高度 / 降落 / 锁定.

流程按四旋翼模态执行 (Q 模式), 不做固定翼转换:
  连接 -> 等 GPS -> 切 QLOITER -> ARM -> NAV_TAKEOFF 到 --alt
       -> 悬停 --hover 秒 -> 切 QLAND 垂直降落(落地自动上锁) -> 兜底 DISARM

用法:
  python qp_fly.py <连接串> fly --alt 10          # 完整流程
  python qp_fly.py <连接串> arm                    # 仅解锁
  python qp_fly.py <连接串> takeoff --alt 10       # 仅起飞(需已解锁)
  python qp_fly.py <连接串> land                  # 仅降落(落地自动上锁)
  python qp_fly.py <连接串> disarm [--force]      # 仅锁定

连接串示例:
  COM3                          串口(默认 115200, 用 --baud 改)
  udp:127.0.0.1:14550           数传/局域网
  tcp:127.0.0.1:5760            SITL

依赖: pip install pymavlink
"""

import argparse
import sys
import time

from pymavlink import mavutil

# ArduPlane 模式编号(与 pymavlink 对 MAV_TYPE=21 的映射一致, 作兜底)
PLANE_MODE = {
    'MANUAL': 0, 'CIRCLE': 1, 'STABILIZE': 2, 'TRAINING': 3, 'ACRO': 4,
    'FBWA': 5, 'FBWB': 6, 'CRUISE': 7, 'AUTOTUNE': 8, 'AUTO': 10,
    'RTL': 11, 'LOITER': 12, 'TAKEOFF': 13, 'AVOID_ADSB': 14,
    'GUIDED': 15, 'INITIALISING': 16,
    'QSTABILIZE': 17, 'QHOVER': 18, 'QLOITER': 19, 'QLAND': 20, 'QRTL': 21,
    'QAUTOTUNE': 22, 'QACRO': 23, 'THERMAL': 24,
    'LOITERALTQLAND': 25, 'AUTOLAND': 26,
}

DISARM_MAGIC = 2198474837  # 强制上锁 magic value

MAV_TYPE_NAMES = {
    1: 'FIXED_WING', 13: 'HEXAROTOR', 14: 'OCTOROTOR',
    19: 'COAXIALROTOR', 20: 'VTOL_DUOTOR', 21: 'VTOL_QUADPLANE',
}


def die(msg):
    raise QPError(msg)


class QPError(Exception):
    """受控错误: CLI 捕获后退出, UI 捕获后显示"""


_log_hook = [None]


def set_logger(fn):
    """设置日志回调(None=恢复 print), 供 UI 复用"""
    _log_hook[0] = fn


def log(msg):
    fn = _log_hook[0]
    if fn:
        fn(str(msg))
    else:
        print(msg)


def connect(conn, baud):
    master = mavutil.mavlink_connection(conn, baud=baud)
    log('正在连接 %s , 等待飞控心跳...' % conn)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        die('无法获取飞控心跳, 请检查连接/波特率/供电')
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent()
    mtype = MAV_TYPE_NAMES.get(hb.type, 'MAV_TYPE_%d' % hb.type)
    log('[OK] 已连接: sysid=%u compid=%u 机型=%s' % (
        hb.get_srcSystem(), hb.get_srcComponent(), mtype))
    if hb.type not in (1, 21):
        log('警告: 心跳机型为 %s, 本脚本按 QuadPlane/ArduPlane 处理, '
              '请确认固件是 ArduPlane 且 Q_ENABLE=1' % mtype)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    return master


def _from_fc(master, msg):
    """心跳/消息是否来自目标飞控"""
    return (not master.target_system
            or msg.get_srcSystem() == master.target_system)


def mode_name(master, custom_mode):
    mm = {}
    try:
        mm = master.mode_mapping() or {}
    except Exception:
        mm = {}
    if not mm:
        mm = PLANE_MODE
    for name, mid in mm.items():
        if mid == custom_mode:
            return name
    return 'MODE(%d)' % custom_mode


def current_mode_id(master):
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb and _from_fc(master, hb):
        return hb.custom_mode
    return None


def set_mode(master, mode_name_, timeout=10.0):
    name = mode_name_.upper()
    mm = None
    try:
        mm = master.mode_mapping()
    except Exception:
        mm = None
    mode_id = (mm or {}).get(name, PLANE_MODE.get(name))
    if mode_id is None:
        die('未知飞行模式 %s (可用: %s)' % (
            name, ','.join(sorted((mm or PLANE_MODE).keys()))))

    sent = False
    if mm and name in mm and hasattr(master, 'set_mode_apm'):
        try:
            master.set_mode_apm(name)
            sent = True
        except Exception:
            sent = False
    if not sent:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0)

    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None or not _from_fc(master, hb):
            continue
        if hb.custom_mode == mode_id:
            log('[OK] 飞行模式: %s' % name)
            return
    die('切换到 %s 失败(超时未在心跳中确认)' % name)


def wait_gps(master, min_sats=6, timeout=60):
    log('等待 GPS 3D 定位 (>= %d 星)...' % min_sats)
    t_end = time.time() + timeout
    while time.time() < t_end:
        g = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if g and _from_fc(master, g) and g.fix_type >= 3 \
                and g.satellites_visible >= min_sats:
            log('[OK] GPS 已定位: fix=%d 卫星=%d' % (
                g.fix_type, g.satellites_visible))
            return
    die('GPS 定位超时, 请到室外开阔处重试')


def collect_status(master, duration=1.0):
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
        log('  [飞控] %s' % last)
    return last


def is_armed(master):
    try:
        return bool(master.motors_armed())
    except Exception:
        return False


def wait_armed(master, want, timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None or not _from_fc(master, hb):
            continue
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed == want:
            return True
    return False


def rc_throttle_low(master):
    """解锁期间覆盖油门=最低, 避免 Throttle-high 拒解锁"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1000, 1500, 0, 0, 0, 0)


def release_rc(master):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0)


def arm(master, timeout=15.0, use_rc_override=True):
    if is_armed(master):
        log('[OK] 已处于解锁状态')
        return
    if use_rc_override:
        log('RC 覆盖油门=1000, 发送 ARM...')
        rc_throttle_low(master)
        time.sleep(0.4)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    ack = None
    last_text = ''
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            if use_rc_override:
                rc_throttle_low(master)
            continue
        if not _from_fc(master, msg):
            continue
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            last_text = str(text)
            log('  [飞控] %s' % last_text)
        elif mtype == 'COMMAND_ACK' \
                and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            ack = msg
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                log('  ARM 命令已接受, 等待 ARMED...')
            break
        elif mtype == 'HEARTBEAT' and \
                (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break

    if use_rc_override:
        release_rc(master)

    if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        more = collect_status(master, 1.5)
        names = {1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
                 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}
        die('解锁被拒绝 ACK=%s%s' % (
            names.get(ack.result, ack.result), ('; ' + more) if more else ''))

    if not wait_armed(master, True, 5.0):
        if not last_text:
            last_text = collect_status(master, 1.0)
        die('解锁后未检测到 ARMED%s' % (
            ('; 飞控: ' + last_text) if last_text else ''))
    log('[OK] 已解锁 (ARMED)')


def takeoff(master, alt, mode='QLOITER', timeout=90.0):
    if not is_armed(master):
        die('未解锁, 无法起飞 (先执行 arm)')
    cur = current_mode_id(master)
    want = (master.mode_mapping() or PLANE_MODE).get(mode.upper())
    if want is not None and cur != want:
        set_mode(master, mode)

    log('发送 NAV_TAKEOFF -> %.1f 米 (模态 %s)...' % (alt, mode.upper()))
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, float(alt))

    ack_t_end = time.time() + 8.0
    while time.time() < ack_t_end:
        a = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if a and _from_fc(master, a) \
                and a.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if a.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                more = collect_status(master, 1.5)
                die('起飞命令被拒绝 result=%s%s' % (
                    a.result, ('; ' + more) if more else ''))
            break

    t_end = time.time() + timeout
    last_print = 0.0
    rel = 0.0
    while time.time() < t_end:
        p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1)
        if p and _from_fc(master, p):
            rel = p.relative_alt / 1000.0
            now = time.time()
            if now - last_print >= 2.0:
                last_print = now
                log('  高度 %.1f / %.1f 米' % (rel, alt))
            if rel >= alt - 0.5:
                log('[OK] 已到达目标高度 %.1f 米' % rel)
                return
        if not is_armed(master):
            die('起飞过程中被上锁 (可能触发自动上锁/PreArm 失败)')
    die('起飞超时, 最后高度仅 %.1f 米' % rel)


def land(master, mode='QLAND', timeout=180.0):
    if not is_armed(master):
        log('[OK] 已处于上锁状态, 无需降落')
        return
    log('切换 %s 垂直降落 (落地后飞控自动上锁)...' % mode.upper())
    set_mode(master, mode)

    t_end = time.time() + timeout
    last_print = 0.0
    while time.time() < t_end:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None or not _from_fc(master, hb):
            continue
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        now = time.time()
        if now - last_print >= 2.0:
            last_print = now
            p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            alt = (p.relative_alt / 1000.0) if p else float('nan')
            log('  模式 %s 高度 %.1f 米 %s' % (
                mode_name(master, hb.custom_mode), alt,
                '降落中' if armed else '已上锁'))
        if not armed:
            log('[OK] 已落地并上锁 (DISARMED)')
            return
    log('警告: 降落超时仍为解锁状态, 建议人工接管或执行 disarm --force')


def disarm(master, force=False, timeout=8.0):
    if not is_armed(master):
        log('[OK] 已处于上锁状态')
        return
    value = DISARM_MAGIC if force else 1
    log('发送 DISARM%s...' % (' (强制)' if force else ''))
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        value, 0, 0, 0, 0, 0, 0)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None or not _from_fc(master, msg):
            continue
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            collect_status(master, 0.2)
        elif mtype == 'COMMAND_ACK' \
                and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                  mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                more = collect_status(master, 1.0)
                die('上锁被拒绝 result=%s%s' % (
                    msg.result, ('; ' + more) if more else ''))
        elif mtype == 'HEARTBEAT' and not \
                (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log('[OK] 已上锁 (DISARMED)')
            return
    die('上锁确认超时 (仍为 ARMED)')


def flow(master, args):
    wait_gps(master, timeout=args.gps_timeout)
    set_mode(master, args.mode)
    arm(master, use_rc_override=not args.no_rc_override)
    takeoff(master, args.alt, mode=args.mode,
            timeout=max(60.0, args.alt * 15))
    log('悬停 %.0f 秒...' % args.hover)
    time.sleep(args.hover)
    land(master, mode=args.land_mode)
    if is_armed(master):
        disarm(master)
    log('[OK] 完整流程结束: 解锁 -> 起飞 %.1f 米 -> 降落 -> 上锁' % args.alt)


def main():
    ap = argparse.ArgumentParser(
        description='QuadPlane 基础流程: 解锁/起飞/降落/锁定')
    ap.add_argument('conn', help='连接串: COM3 / udp:127.0.0.1:14550 / '
                                 'tcp:127.0.0.1:5760')
    ap.add_argument('--baud', type=int, default=115200,
                    help='串口波特率 (默认 115200)')
    ap.add_argument('cmd', choices=['arm', 'takeoff', 'land', 'disarm',
                                    'fly'],
                    help='arm=解锁 takeoff=起飞 land=降落 '
                         'disarm=锁定 fly=完整流程')
    ap.add_argument('--alt', type=float, default=10.0,
                    help='起飞高度/米 (默认 10)')
    ap.add_argument('--mode', default='QLOITER',
                    help='起飞模态: QLOITER(默认)/QHOVER')
    ap.add_argument('--land-mode', default='QLAND',
                    help='降落模态: QLAND(默认)/QRTL')
    ap.add_argument('--hover', type=float, default=5.0,
                    help='起飞后悬停秒数 (fly 流程, 默认 5)')
    ap.add_argument('--gps-timeout', type=float, default=60.0,
                    help='等 GPS 秒数 (默认 60)')
    ap.add_argument('--no-rc-override', action='store_true',
                    help='解锁时不覆盖油门通道')
    ap.add_argument('--force', action='store_true',
                    help='disarm 时使用强制上锁')
    args = ap.parse_args()

    if args.alt <= 0:
        die('--alt 必须大于 0')

    master = connect(args.conn, args.baud)
    try:
        if args.cmd == 'arm':
            wait_gps(master, timeout=args.gps_timeout)
            set_mode(master, args.mode)
            arm(master, use_rc_override=not args.no_rc_override)
        elif args.cmd == 'takeoff':
            takeoff(master, args.alt, mode=args.mode,
                    timeout=max(60.0, args.alt * 15))
        elif args.cmd == 'land':
            land(master, mode=args.land_mode)
        elif args.cmd == 'disarm':
            disarm(master, force=args.force)
        elif args.cmd == 'fly':
            flow(master, args)
    except KeyboardInterrupt:
        log('\n用户中断, 执行紧急上锁...')
        try:
            disarm(master, force=True)
        except (QPError, SystemExit) as e:
            log(e)
        sys.exit(130)
    except QPError as e:
        sys.exit('错误: %s' % e)


if __name__ == '__main__':
    main()
