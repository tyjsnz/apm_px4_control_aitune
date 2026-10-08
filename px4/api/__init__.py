# -*- coding: utf-8 -*-
"""PX4 多地面站无人机控制服务 (FastAPI).

对外只暴露 HTTP 接口, 飞控控制(OFFBOARD 持续引导/起飞/任务/急停)全部在服务端内部完成.
"""

__version__ = '1.0.0'
