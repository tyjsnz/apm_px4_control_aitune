# -*- coding: utf-8 -*-
"""简易 PX4 假飞控 (TCP), 用于 px4.api 无硬件冒烟测试.

基于 px4/tests/fake_fc_px4.FakeFCpx4 (心跳/模式/任务/参数/EXTENDED_SYS_STATE
均按 PX4 行为模拟), 本模块额外补记最近一帧 SET_POSITION_TARGET_GLOBAL_INT
(frame/mask/alt), 供冒烟测试断言引导帧格式.

用法: python -m px4.api.tests.fake_fc [--port 15761]
      python px4/api/tests/fake_fc.py [--port 15761]
API 侧连接串: tcp:127.0.0.1:15761
"""
import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PX4 = os.path.dirname(os.path.dirname(_HERE))
_ROOT = os.path.dirname(_PX4)
for _p in (_PX4, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from px4.tests.fake_fc_px4 import FakeFCpx4            # noqa: E402


class FakeFC(FakeFCpx4):
    """FakeFCpx4 + 最近一帧全局位置设定值记录"""

    def __init__(self, conn_str):
        super().__init__(conn_str)
        self.last_pos_target = None

    def handle(self, msg):
        if msg.get_type() == 'SET_POSITION_TARGET_GLOBAL_INT':
            self.last_pos_target = {
                'lat': msg.lat_int / 1e7, 'lon': msg.lon_int / 1e7,
                'alt': float(msg.alt), 'mask': int(msg.type_mask),
                'vx': float(msg.vx), 'vy': float(msg.vy),
                'vz': float(msg.vz), 'frame': int(msg.coordinate_frame),
                'at': time.time()}
        super().handle(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=15761)
    ap.add_argument('--host', default='127.0.0.1')
    args = ap.parse_args()
    conn = 'tcpin:%s:%d' % (args.host, args.port)
    print('[fake_fc] %s' % conn)
    FakeFC(conn).run()


if __name__ == '__main__':
    main()
