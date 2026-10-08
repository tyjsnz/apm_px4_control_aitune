# -*- coding: utf-8 -*-
"""对 fake_fc_px4 顺序运行 px4_control_test 的 1~14 项测试 (冒烟回归).

用法: python px4/tests/run_tct_suite.py [编号 ...]   # 默认 1..14
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

from fake_fc_px4 import FakeFCpx4            # noqa: E402

import px4_control as apc                    # noqa: E402
import px4_control_test as tct               # noqa: E402

PORT = 15793


def main():
    ids = sys.argv[1:] or sorted(tct.TESTS, key=int)

    srv = threading.Thread(
        target=FakeFCpx4('tcpin:127.0.0.1:%d' % PORT).run, daemon=True)
    srv.start()
    time.sleep(0.6)

    master = apc.mavconnect('tcp:127.0.0.1:%d' % PORT, 57600)
    apc.wait_gps_lock(master)

    results = []
    for tid in ids:
        ok = tct.run_single_test(master, tid)
        results.append((tid, bool(ok)))

    apc.stop_offboard_stream()
    try:
        master.close()
    except Exception:
        pass

    print('\n' + '=' * 50)
    print('📋 tct 14 项测试汇总 (对 fake_fc_px4)')
    print('=' * 50)
    for tid, ok in results:
        name = tct.TESTS.get(tid, ('?', None))[0]
        print('  %s  %s. %s' % ('✅' if ok else '❌', tid, name))
    passed = sum(1 for _, ok in results if ok)
    print('\n  总计: %d/%d 通过' % (passed, len(results)))
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
