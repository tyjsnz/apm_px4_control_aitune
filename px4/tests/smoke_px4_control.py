# -*- coding: utf-8 -*-
"""px4_control 核心链路冒烟测试 (对 fake_fc_px4 假飞控, 无需硬件).

覆盖: 连接/PX4 校验 -> GPS锁星 -> 模式切换 -> 解锁+起飞 ->
      OFFBOARD 位置 goto -> OFFBOARD 速度巡航 -> 任务上传/执行(MISSION)
      -> RTL 降落自动上锁 -> 参数回读.

用法: python px4/tests/smoke_px4_control.py   (在仓库根目录运行)
"""
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PX4 = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_PX4)
for _p in (_PX4, _HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pymavlink import mavutil                # noqa: E402

from fake_fc_px4 import FakeFCpx4            # noqa: E402

import px4_control as apc                    # noqa: E402
import px4_control_test as tct               # noqa: E402

PORT = 15791
RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok)))
    print('[%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                         (' - ' + detail) if detail else ''))
    return bool(ok)


def run():
    srv = threading.Thread(
        target=FakeFCpx4('tcpin:127.0.0.1:%d' % PORT).run, daemon=True)
    srv.start()
    time.sleep(0.6)

    master = apc.mavconnect('tcp:127.0.0.1:%d' % PORT, 57600)
    apc.wait_gps_lock(master)

    # 1) 模式切换
    apc.test_flight_modes(master, ['POSCTL', 'LOITER', 'RTL'])
    check('模式切换 POSCTL/LOITER/RTL', True)

    # 2) 解锁 + 起飞
    apc.arm_and_takeoff(master, 12.0, mode='POSCTL')
    check('解锁+起飞到12m', master.motors_armed())

    # 3) OFFBOARD 位置 goto
    lat0 = 31.2304 + 0.0006
    lon0 = 121.4737 + 0.0006
    t_end = time.time() + 60
    arrived = False
    while time.time() < t_end:
        apc.goto(master, lat0, lon0, 15.0)
        m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1)
        if m is None:
            continue
        d = apc.haversine(m.lat / 1e7, m.lon / 1e7, lat0, lon0)
        if d <= 3.0:
            arrived = True
            break
        time.sleep(0.5)
    check('OFFBOARD 位置 goto 到达(<3m)', arrived,
          '最后距离 %.1f m' % d)

    # 4) OFFBOARD 速度巡航 (fly_to_target -> set_velocity)
    st = apc.MissionState(lat0 - 0.0004, lon0 - 0.0004, 15.0, 10.0)
    act = apc.fly_to_target(master, st, speed=3.0, threshold=3.0,
                            timeout=60, cmd_q=None)
    check('速度巡航 fly_to_target 到达', act == 'arrived', '返回 %r' % act)

    # 5) 任务上传 + MISSION 执行 (末项 RTL)
    wps = [(31.2310, 121.4745, 18.0), (31.2315, 121.4750, 18.0)]
    n = apc.upload_mission(master, wps, end_action='rtl')
    check('任务上传', n == 3, '返回 %r (3=2航点+RTL项)' % n)
    apc.reset_mission_to_start(master)
    # set_mode 失败会 sys.exit, 由外层捕获
    apc.set_mode(master, 'MISSION')
    ok = apc.wait_auto_mission(master, n_wp=len(wps), end_action='rtl',
                               timeout=150)
    check('MISSION 任务执行至 RTL', ok)

    # 6) 降落自动上锁
    ok = apc.wait_landed(master, timeout=180)
    check('RTL 降落并上锁', ok and not master.motors_armed())

    # 7) 参数回读
    v = apc.get_param(master, 'MIS_TAKEOFF_ALT')
    check('参数回读 MIS_TAKEOFF_ALT', v is not None and abs(v - 12.0) < 0.01,
          '读到 %r' % v)

    # 8) tct 链路 (UI 走这条): 切模式 + 解锁 + NAV_TAKEOFF 起飞
    ok = tct.set_flight_mode(master, 'POSCTL', timeout=6)
    check('tct.set_flight_mode(POSCTL)', ok)
    ok = tct.arm_and_takeoff(master, 10.0, mode='TAKEOFF',
                             arm_timeout=10, takeoff_timeout=45,
                             set_delay=True)
    check('tct.arm_and_takeoff (send_guided_takeoff)', ok
          and master.motors_armed())

    # 9) 收尾: 切 LAND, 落地后 COM_DISARM_LAND 自动上锁
    tct.set_flight_mode(master, 'LAND', timeout=6)
    t_end = time.time() + 60
    disarmed = False
    while time.time() < t_end:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        if not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            disarmed = True
            break
    check('tct 收尾 LAND 落地自动上锁', disarmed)

    apc.stop_offboard_stream()


def main():
    try:
        run()
    except SystemExit as e:
        check('流程未中断', False, 'sys.exit: %s' % e)
    except Exception as e:
        check('流程未中断', False, '%s: %s' % (type(e).__name__, e))
    finally:
        apc.stop_offboard_stream()
    failed = [n for n, ok in RESULTS if not ok]
    print('\n==== 结果: %d 通过 / %d 失败 ====' % (
        len(RESULTS) - len(failed), len(failed)))
    for n in failed:
        print('  FAIL: %s' % n)
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
