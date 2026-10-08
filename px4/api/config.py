# -*- coding: utf-8 -*-
"""运行配置: 全部可用环境变量覆盖 (PX4_API_ 前缀)."""
import os


def _env(name, default, cast=str):
    raw = os.environ.get(name)
    if raw is None or raw == '':
        return default
    try:
        return cast(raw)
    except (TypeError, ValueError):
        return default


HOST = _env('PX4_API_HOST', '0.0.0.0')
PORT = _env('PX4_API_PORT', 8000, int)

DEFAULT_BAUD = _env('PX4_API_DEFAULT_BAUD', 57600, int)
CONNECT_TIMEOUT = _env('PX4_API_CONNECT_TIMEOUT', 30.0, float)

TELEMETRY_STREAMS = (
    ('EXTENDED_STATUS', 2),
    ('POSITION', 2),
    ('RC_CHANNELS', 2),
    ('EXTRA1', 5),
    ('EXTRA2', 2),
)

GUIDANCE_RATE_HZ = _env('PX4_API_GUIDANCE_RATE', 10.0, float)
GUIDANCE_MIN_RATE_HZ = 1.0
GUIDANCE_MAX_RATE_HZ = 30.0
GUIDANCE_DEFAULT_TTL = _env('PX4_API_GUIDANCE_TTL', 0.0, float)
GUIDANCE_ALT_FRAME = _env('PX4_API_ALT_FRAME', 'relative')
GUIDANCE_MIN_CONFIDENCE = _env('PX4_API_MIN_CONFIDENCE', 0, int)

GOTO_DEFAULT_THRESHOLD = _env('PX4_API_GOTO_THRESHOLD', 2.5, float)
GOTO_DEFAULT_ALT = _env('PX4_API_GOTO_ALT', 20.0, float)
GOTO_ARRIVAL_ALT_TOL = 1.5
GOTO_SPEED_MIN = 0.2
GOTO_SPEED_MAX = 20.0

MISSION_TAKEOFF_ALT = _env('PX4_API_TAKEOFF_ALT', 5.0, float)
MISSION_POINT_TIMEOUT = 30.0
MISSION_TOTAL_TIMEOUT = _env('PX4_API_MISSION_TIMEOUT', 900.0, float)
LANDING_TIMEOUT = _env('PX4_API_LANDING_TIMEOUT', 300.0, float)

TASK_HISTORY = 50
TASK_LOG_MAX_LINES = 4000

AUTO_CONFIRM_ARM = _env('PX4_API_AUTO_CONFIRM', '1') not in ('0', 'false', 'False')
