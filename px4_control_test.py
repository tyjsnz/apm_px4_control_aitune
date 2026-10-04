# PX4 control test module
from pymavlink import mavutil
import time
import math
import sys
import os

CONNECTION_STRING = 'COM25'
BAUD_RATE = 115200
CONTROL_RATE = 20

TAKEOFF_ALT = 5.0
TEST_SPEED = 2.0
TEST_DURATION = 5

# PX4 mode names
MODE_OFFBOARD = 'OFFBOARD'
MODE_POSCTL = 'POSCTL'
MODE_ALTCTL = 'ALTCTL'
MODE_MANUAL = 'MANUAL'
MODE_STABILIZED = 'STABILIZED'
MODE_ACRO = 'ACRO'
MODE_MISSION = 'MISSION'
MODE_RTL = 'RTL'
MODE_LAND = 'LAND'
MODE_TAKEOFF = 'TAKEOFF'
MODE_LOITER = 'LOITER'

DISARM_DELAY_SEC = 60

SOFT_RC_MANUAL_MODES = ('MANUAL', 'STABILIZED', 'ACRO', 'ALTCTL', 'POSCTL')
RC_NEUTRAL_GROUND = (1500, 1500, 1000, 1500)
RC_NEUTRAL_AIR = (1500, 1500, 1500, 1500)
RC_HANDOVER = 1

SOFT_LAND_ALT = 1.0
SOFT_LAND_RATE = 0.25
SOFT_LAND_THR_START = 1480
SOFT_LAND_THR_MIN = 1300
SOFT_LAND_THR_MAX = 1520
SOFT_LAND_GROUND_ALT = 0.15
SOFT_LAND_SETTLE_T = 45

STATUS_LISTEN = 3.0
BATTERY_LISTEN = 3.0

BOUNCE_ALT = 2.0
BOUNCE_HOVER_SEC = 2.0
BOUNCE_TAKEOFF_TIMEOUT = 30
BOUNCE_LAND_TIMEOUT = 60
BOUNCE_LAND_ALT = 0.3

ARM_CONFIRM_HOOK = None
ARM_CONFIRM_ALL_READY = True

LAUNCH_PARAMS_WRITTEN = False
DISARM_DELAY_WRITTEN = False

TAKEOFF_CLIMB_THR = 1800
AUTOTUNE_CLIMB_THR = 1700
AUTOTUNE_SETTLE_THR = 1500
AUTOTUNE_MIN_SATS = 6
AUTOTUNE_LOITER_SETTLE = 3
AUTOTUNE_ALT = 5.0
AUTOTUNE_TIMEOUT = 1800
LAUNCH_PARAMS = ()
LAUNCH_PRESET = 0
ARM_RETRY = 2
ARM_RETRY_GPS_WAIT = 30
ARM_RETRY_KEYS = ()
EKF_POS_HORIZ_FLAGS = 0x18
EKF_POS_VERT_FLAGS = 0x60
EKF_CONST_POS_FLAG = 0x80
ARM_CONFIRM_TIMEOUT = 120
SOFT_RC_RP_US = 250
SOFT_RC_YAW_US = 300
SOFT_RC_RATE_HZ = 20

def reset_launch_params_flag():
    global LAUNCH_PARAMS_WRITTEN, DISARM_DELAY_WRITTEN
    LAUNCH_PARAMS_WRITTEN = False
    DISARM_DELAY_WRITTEN = False

def connect():
    print(f'\n连接 {CONNECTION_STRING} ...')
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print('未收到心跳')
        sys.exit(1)
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent()
    try:
        master.vehicle_type = 'copter'
    except Exception:
        pass
    print(f'PX4 已连接 (sysid={master.target_system})')
    return master

def drain_rx(master):
    while master.recv_match(blocking=False):
        pass

def _mode_name(master, cm):
    try:
        mode_map = master.mode_mapping() or {}
        return next((n for n,i in mode_map.items() if i==cm), str(cm))
    except Exception:
        return str(cm)

def set_param(*a,**k): return True
def get_param(*a,**k): return None
def ensure_launch_params(*a,**k): return True
def write_launch_params(*a,**k): return True
def ensure_disarm_delay(*a,**k): return True
def write_disarm_delay(*a,**k): return True
def arm_vehicle(*a,**k): return False
def disarm_vehicle(*a,**k): return True
def release_rc_override(*a,**k): pass
def takeoff_althold(*a,**k): return True
def land_soft_final(*a,**k): return True
def reboot_flight_controller(*a,**k): return True
def get_current_alt(*a,**k): return 0.0
def get_current_pos(*a,**k): return None
def wait_for_alt(*a,**k): return True
def send_zero_velocity(*a,**k): pass
def wait_gps_lock(*a,**k): return True
def wait_ekf_position(*a,**k): return True
def collect_status_text(*a,**k): return ''
def confirm_arm(*a,**k): return True
def print_motor_outputs(*a,**k): pass
def send_guided_takeoff(*a,**k): return True
def arm_and_takeoff(*a,**k): return False
def wait_autotune(*a,**k): return True
def download_flash_logs(*a,**k): return []

# TESTS for UI compatibility
TESTS = {}
def run_single_test(*a,**k): return True
def run_all_tests(*a,**k): return []
def show_menu(): pass
