'''
一个完整的功能测试框架 —— PX4 版 (对应 ArduPilot 的 apm_control_test.py):
把连接、解锁、起飞、速度控制、降落、锁定、参数读写校验、飞机状态、电池状态
等每个功能拆成独立测试项, 可以单独运行, 也可以一键跑全部.

与 APM 版保持同一套对外结构 / 函数划分 / 中文提示风格, 内部按 PX4 官方语义重写:
  · 飞行模式: HEARTBEAT.custom_mode = (main_mode << 16) | (sub_mode << 24);
    切模式优先 master.set_mode('POSCTL')(pymavlink 内置 px4_map →
    MAV_CMD_DO_SET_MODE param1=flags/param2=main/param3=sub), 兜底自发
    command_long, 以心跳 custom_mode 打包值校验生效 (10s 超时).
  · 解锁/上锁: MAV_CMD_COMPONENT_ARM_DISARM (param1=1; 强制上锁 param2=21196);
    PX4 拒因以 STATUSTEXT 'Arming denied: ...' 上报, 解锁路径会采集显示.
  · 自动上锁参数: COM_DISARM_PRFLT (解锁后起飞太慢) / COM_DISARM_LAND (降落后).
  · 起飞: MAV_CMD_NAV_TAKEOFF (先写 MIS_TAKEOFF_ALT=目标高度, param7=NaN).
  · 速度控制: OFFBOARD + SET_POSITION_TARGET_LOCAL_NED (NED 速度 setpoint 流).
  · 位置就绪: GPS_RAW_INT + GLOBAL_POSITION_INT (PX4 不发 EKF_STATUS_REPORT).

自包含: 只依赖 pymavlink/time/math/sys/os, 不 import px4_control (避免循环依赖).
'''
from pymavlink import mavutil
import time
import math
import sys
import os

# ============ 配置参数 ============
CONNECTION_STRING = 'COM25'   # 按实际修改
BAUD_RATE = 115200
CONTROL_RATE = 10                    # 速度指令发送频率 Hz (PX4 OFFBOARD 要求 >2Hz)
TAKEOFF_ALT = 5.0                    # 默认起飞高度（米）
TEST_SPEED = 2.0                     # 测试速度（m/s）
TEST_DURATION = 5                    # 每项测试持续时间（秒）

# ============ PX4 模式打包 (custom_mode = main<<16 | sub<<24) ============
PX4_MAIN_MODES = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO', 5: 'ACRO',
                  6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE'}
PX4_AUTO_SUBMODES = {1: 'READY', 2: 'TAKEOFF', 3: 'LOITER', 4: 'MISSION',
                     5: 'RTL', 6: 'LAND', 7: 'RTGS', 8: 'FOLLOW_TARGET'}
# 模式名 → 打包 custom_mode (AUTO 子模式: (4<<16)|(sub<<24))
PX4_MODE_CUSTOM = {
    'MANUAL': (1 << 16), 'ALTCTL': (2 << 16), 'POSCTL': (3 << 16),
    'AUTO': (4 << 16), 'ACRO': (5 << 16), 'OFFBOARD': (6 << 16),
    'STABILIZED': (7 << 16), 'RATTITUDE': (8 << 16),
    'READY': (4 << 16) | (1 << 24), 'TAKEOFF': (4 << 16) | (2 << 24),
    'LOITER': (4 << 16) | (3 << 24), 'MISSION': (4 << 16) | (4 << 24),
    'RTL': (4 << 16) | (5 << 24), 'LAND': (4 << 16) | (6 << 24),
    'RTGS': (4 << 16) | (7 << 24), 'FOLLOW_TARGET': (4 << 16) | (8 << 24),
}
# APM(及地面站通用)模式名 → PX4 模式名: UI/REST API 仍可能传 GUIDED/ALT_HOLD 等,
# 先翻译再下发; None = PX4 无对应模式 (如 AUTOTUNE)
APM_TO_PX4_MODE = {
    'GUIDED': 'POSCTL', 'ALT_HOLD': 'ALTCTL', 'STABILIZE': 'STABILIZED',
    'LOITER': 'LOITER', 'POSHOLD': 'LOITER', 'RTL': 'RTL', 'LAND': 'LAND',
    'AUTO': 'MISSION', 'ACRO': 'ACRO', 'CIRCLE': 'LOITER', 'BRAKE': 'LOITER',
    'SMARTRTL': 'RTL', 'DRIFT': 'RATTITUDE', 'SPORT': 'ACRO',
    'FOLLOW': 'LOITER', 'ZIGZAG': 'MISSION', 'GUIDED_NOGPS': 'ALTCTL',
    'AUTOTUNE': None, 'INITIALISE': None,
}
# 需要水平位置估计才能解锁的模式 (不满足则走预检门禁, 如 MANUAL/ALTCTL/STABILIZED)
POSITION_MODES = ('POSCTL', 'LOITER', 'MISSION', 'RTL', 'LAND', 'TAKEOFF',
                  'OFFBOARD', 'READY', 'FOLLOW_TARGET',
                  'GUIDED', 'AUTO', 'CIRCLE', 'SMARTRTL', 'BRAKE', 'FOLLOW')

# ============ 起飞爬升杆位 (界面兼容) ============
# v1.33: PX4 起飞由 Takeoff 模式闭环爬升 (MIS_TAKEOFF_ALT / MPC_TKO_SPEED),
# 不存在 APM ALT_HOLD 的"杆位换算爬升率"链路 — 该常量仅为 UI 兼容保留:
# UI 会读写并钳制 1600~2000, PX4 测试流程内部不使用杆位/油门爬升.
TAKEOFF_CLIMB_THR = 1800

# ============ 发射箱参数写入控制 ============
# 全程不自动写飞控参数 — 写入只由用户点[写入发射箱参数]触发,
# 这里的标记仅用于"是否已手动写过", 解锁/起飞路径只提示不写入
LAUNCH_PARAMS_WRITTEN = False        # 标记是否已在本会话写入过发射箱参数
DISARM_DELAY_WRITTEN = False         # 标记是否已写入自动上锁参数

def reset_launch_params_flag():
    """重置发射箱参数写入标记（连接/断开时调用）"""
    global LAUNCH_PARAMS_WRITTEN, DISARM_DELAY_WRITTEN
    LAUNCH_PARAMS_WRITTEN = False
    DISARM_DELAY_WRITTEN = False

# ============ 解锁后自动上锁 / 起飞集成 ============
# PX4: 解锁后若一直未起飞, COM_DISARM_PRFLT 秒后自动 DISARM (默认 10s,
# <=0 或负值禁用); 降落后按 COM_DISARM_LAND (默认 2s) 自动 DISARM.
# 这就是"测试2解锁→稍后再飞已上锁"的原因 (对应 APM 的 DISARM_DELAY)
DISARM_DELAY_SEC = 60                # [写入发射箱参数]按钮要写入的 COM_DISARM_PRFLT(秒);
                                     # 0=不写; 系统不自动写, 仅在未写入时提示
ARM_TO_TAKEOFF_MAX = 8               # 集成起飞时: 解锁成功后最多几秒内必须发出 NAV_TAKEOFF

# ============ 发射箱纯软件飞行（解锁 / 参数预设） ============
LAUNCH_PRESET = 1                    # 1=[写入发射箱参数]按钮包含该参数组(禁飞控自动返航/降落)
                                     # 系统侧不再自动写, 只在未写入时提示用户手动写
# (参数名, 期望值, 中文标签): 读回不符才写, 命中跳过; 单项失败仅警告继续
# (PX4 参数名 ≤16 字符; 值均取"发射箱/纯软件飞行"安全语义)
LAUNCH_PARAMS = (
    ('COM_RCL_EXCEPT', 7.0, 'RC丢失在Mission/Hold/Offboard忽略'),
    ('NAV_DLL_ACT', 0.0, '数据链路丢失不动作'),
    ('COM_LOW_BAT_ACT', 0.0, '低电量仅告警'),
    ('COM_OBL_RC_ACT', 5.0, 'Offboard丢失转Hold悬停'),
    ('MIS_TAKEOFF_ALT', 2.5, '起飞默认高度m'),
    ('COM_DISARM_LAND', 2.0, '降落后2s自动上锁'),
)
ARM_RETRY = 2                        # 解锁被拒且因GPS/位置未就绪时自动重试次数
ARM_RETRY_GPS_WAIT = 30              # 每次重试前等待GPS/位置就绪的秒数
# v1.29.3: 解锁前预检门禁 — 位置估计未就绪时先等(带进度), 到时仍未就绪则
# 不发 ARM (避免必现的 'Arming denied: ... position ...' 拒绝)
ARM_PREPARE_WAIT = 60
# v1.33: 预检判据旋钮 (默认=旧硬编码值, 仅为可调)
POS_READY_MIN_SATS = 6               # 解锁预检最少卫星数
POS_EKF_MSG_GRACE = 15               # 无 GLOBAL_POSITION_INT 多少秒后降级只看 GPS

# ============ 位置就绪位掩码 (PX4 语义, 取代 APM 的 EKF_STATUS_REPORT flags) ============
# position_ready_state/ekf_pos_ready 的 flags 不再是 EKF_STATUS_REPORT 位,
# 而是本模块自定义的"位置消息到达"位掩码 (语义变化, 名字保留给 API/UI):
#   0x1 = 已收到 GPS 全局位置 (GLOBAL_POSITION_INT 经纬度非0)
#   0x2 = 已收到本地位置 (LOCAL_POSITION_NED)
POS_FLAG_GPS_GLOBAL = 0x1
POS_FLAG_LOCAL = 0x2
# 解锁被拒后值得"等就绪重试"的 STATUSTEXT 关键词 (PX4 拒因 Arming denied: ...)
ARM_RETRY_KEYS = ('position', 'gps', 'ekf', 'estimate', 'origin', 'home',
                  'navigation', 'need 3d fix', 'waiting for', 'not ready')
# v1.22: 就绪后不自动解锁 — 醒目提示+人工确认 (防测试人员贴近时机身突解锁)
ARM_CONFIRM_TIMEOUT = 120            # 确认超时秒数 (超时未确认则取消解锁)

# ============ 软遥控配置（屏幕摇杆/键盘 → RC 覆盖, UI 兼容） ============
# PX4 下本模块不发 RC 覆盖帧 (无 APM 的"油门高位 PreArm", 也不需解锁期间
# 压油门), 这些常量仅为 UI 软遥控/收尾逻辑兼容保留
RC_NEUTRAL_GROUND = (1500, 1500, 1000, 1500)  # 上锁收尾/解锁压油门(油门低)
RC_NEUTRAL_AIR = (1500, 1500, 1500, 1500)     # 空中收尾(油门=悬停,不掉高)
RC_HANDOVER = 1                       # 1=收尾发4通道全0帧(MAVLink约定0=释放该通道)
SOFT_RC_RP_US = 250                   # 横滚/俯仰限幅(μs): 1500±250=1250~1750
SOFT_RC_YAW_US = 300                  # 偏航限幅(μs): 1500±300=1200~1800
SOFT_RC_RATE_HZ = 20                  # 启用期间发送频率 (Hz)
# 仅手动模式下杆值有效 (POSCTL/OFFBOARD 下摇杆被固件忽略, 不发送) — PX4 模式名
SOFT_RC_MANUAL_MODES = ('MANUAL', 'STABILIZED', 'ACRO', 'RATTITUDE',
                        'ALTCTL', 'POSCTL')

# ============ 参数读写校验配置（测试11, 原 PID 自动调参） ============
# PX4 没有 MAVLink AUTOTUNE 模式: 测试11 改为参数读写一致性校验,
# AUTOTUNE_* 常量仅为兼容保留 (wait_autotune/auto_tune_flow 占位提示 QGC 调参)
AUTOTUNE_ALT = 5.0                   # 调参起飞高度（米），建议 5~10
AUTOTUNE_TIMEOUT = 1800              # 等待调参完成超时（秒）
AUTOTUNE_CLIMB_THR = 1700            # (APM 遗留) 爬升杆位, PX4 不使用
AUTOTUNE_SETTLE_THR = 1500           # (APM 遗留) 到位后稳定杆位, PX4 不使用
AUTOTUNE_MIN_SATS = 6                # 调参/校验前最少卫星数
AUTOTUNE_LOITER_SETTLE = 3           # 进调参前 Hold 稳定秒数

# ============ 软着陆配置 (APM 油门微调遗留, PX4 保留兼容) ============
# PX4 的 LAND/RTL 由飞控闭环缓降, 着陆检测后按 COM_DISARM_LAND 自动上锁,
# 不存在地面站 RC 油门微调; 下列常量仅为接口/界面兼容保留, 判定用到
# SOFT_LAND_GROUND_ALT(触地高度) 与 SOFT_LAND_ALT(进度显示)
SOFT_LAND_ALT = 1.0                 # 进度提示的"接近地面"高度（米）
SOFT_LAND_RATE = 0.25               # (APM 遗留) 接管后目标缓降速度（米/秒）
SOFT_LAND_THR_START = 1480          # (APM 遗留) 微调起始油门（1500≈悬停）
SOFT_LAND_THR_MIN = 1300            # (APM 遗留) 油门下限
SOFT_LAND_THR_MAX = 1520            # (APM 遗留) 油门上限
SOFT_LAND_GROUND_ALT = 0.3          # 触地判定高度（米, 兜底判据）
SOFT_LAND_SETTLE_T = 45             # 着陆等待总超时（秒）

# ============ 状态 / 电池显示配置（测试12/13） ============
STATUS_LISTEN = 3.0                  # 飞机状态采集时长（秒）
BATTERY_LISTEN = 3.0                 # 电池信息采集时长（秒）

# ============ 低空起降悬停配置（测试14） ============
BOUNCE_ALT = 2.0                     # 起飞目标高度（米）
BOUNCE_HOVER_SEC = 2.0               # 到位后悬停秒数
BOUNCE_TAKEOFF_TIMEOUT = 30          # 起飞到位超时（秒）
BOUNCE_LAND_TIMEOUT = 60             # 降落到位超时（秒）
BOUNCE_LAND_ALT = 0.3                # 判定已着陆的相对高度（米）

# OFFBOARD 速度 setpoint 的 type_mask: 忽略位置(0x007)/加速度(0x1C0)/
# 偏航(0x200=bit9)/偏航角速率(0x400=bit10), 只保留 vx,vy,vz (NED)
# (注意: MAVLink bit13/14 是保留位, 不是偏航; 与 apc.VELOCITY_TYPE_MASK 一致)
OFFBOARD_VEL_MASK = 0x007 | 0x1C0 | 0x200 | 0x400       # = 0x7C7
OFFBOARD_WARMUP_SEC = 1.0            # 进 OFFBOARD 前至少先流多久 setpoint (≥5Hz)


# ============================================================
#  基础工具函数
# ============================================================

def connect():
    """连接飞控, 并设置命令目标系统/组件(否则解锁等 command 可能发错目标)"""
    print(f"\n🔌 正在连接 {CONNECTION_STRING} ...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("❌ 未收到心跳, 请检查串口/波特率")
        sys.exit(1)
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent()
    try:
        master.vehicle_type = 'copter'
    except AttributeError:
        pass
    print(f"✅ 飞控已连接 (sysid={master.target_system} compid={master.target_component})")
    ap = getattr(hb, 'autopilot', None)
    if ap != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        print(f"⚠️ 心跳 autopilot={ap} 非 MAV_AUTOPILOT_PX4, 请确认连接的是 PX4 飞控")
    return master


def drain_rx(master):
    """清空接收缓冲, 避免读到旧消息干扰判断"""
    while master.recv_match(blocking=False):
        pass


def get_rc_throttle(master):
    """读取遥控器油门通道(RC ch3), 返回 PWM 或 None
    (PX4 下仅作状态显示: 无接收机/未流式发送时返回 None)"""
    drain_rx(master)
    t_end = time.time() + 1.5
    while time.time() < t_end:
        msg = master.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'],
                                blocking=True, timeout=1)
        if msg is None:
            return None
        try:
            if msg.get_type() == 'RC_CHANNELS':
                return int(msg.chan3_raw)
            return int(msg.chan3_raw)
        except (AttributeError, TypeError):
            return None
    return None


def collect_status_text(master, duration=2.0, sink=None):
    """在 duration 秒内收取并打印飞控 STATUSTEXT(Arming denied 等), 返回最后一条.
    sink 传入列表则把每条都 append 进去(供累积判据, 见 arm_vehicle)"""
    last = ""
    t_end = time.time() + duration
    while time.time() < t_end:
        msg = master.recv_match(type='STATUSTEXT', blocking=True,
                                timeout=max(0.1, t_end - time.time()))
        if msg is None:
            continue
        text = msg.text
        if isinstance(text, (bytes, bytearray)):
            text = text.decode('utf-8', 'replace')
        text = str(text)
        last = text
        if sink is not None:
            sink.append(text)
        print(f"    [飞控] {text}")
    return last


def set_param(master, name, value, timeout=3.0):
    """PARAM_SET 写参数, 等 PARAM_VALUE 回读确认 (PX4 参数名 ≤16 字符).
    返回 True/False"""
    name = str(name)
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('ascii', 'replace') if hasattr(name, 'encode') else name,
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
        pn = str(pn).split('\x00')[0]
        if pn == name:
            # PX4 整型参数以原生类型回读, 数值接近即算成功
            if abs(float(msg.param_value) - float(value)) < 0.01:
                print(f"  参数 {name} = {float(msg.param_value):g}")
                return True
            # 回读不符不轻信, 重新读一次复核, 杜绝"同名即真"的假成功
            # (如参数被固件钳制/只读时旧逻辑仍返回 True)
            print(f"  ⚠️ {name} 回读={float(msg.param_value):g} "
                  f"(请求 {value:g}), 重新读确认...")
            cur = get_param(master, name, timeout=timeout)
            if cur is not None and abs(cur - value) < 0.01:
                print(f"  参数 {name} = {cur:g} (复核一致)")
                return True
            print(f"  ❌ {name} 写入未生效(固件钳制/拒绝, 现值={cur})")
            return False
    return False


def send_change_speed(master, speed_ms, timeout=2.5):
    """MAV_CMD_DO_CHANGE_SPEED: 用飞行指令(不是参数)调整本次飞行速度上限.
    PX4 在任务(MISSION)下生效, 不写飞控参数. 返回 True(ACK=ACCEPTED)/False"""
    drain_rx(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
        0, float(speed_ms), -1, 0, 0, 0, 0)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            return False
        mtype = msg.get_type()
        if mtype == 'COMMAND_ACK' and \
                msg.command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"  ✅ 速度指令已接受: {speed_ms:g} m/s (不写参数)")
                return True
            print(f"  ⚠️ DO_CHANGE_SPEED 被拒(result={msg.result})")
            return False
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            print(f"    [飞控] {text}")
    print("  ⚠️ DO_CHANGE_SPEED 无 ACK(超时)")
    return False


def ensure_disarm_delay(master):
    """起飞/解锁前不再写自动上锁参数 (系统只做飞行控制, 参数写入全部交由
    用户点[写入发射箱参数]手动触发).
    这里不发任何参数指令, 只在本会话尚未手动写入时打印提示.
    返回 True(恒真, 不阻断解锁)"""
    if not DISARM_DELAY_SEC or DISARM_DELAY_SEC <= 0:
        return True
    if DISARM_DELAY_WRITTEN:
        return True
    print("  ℹ️ COM_DISARM_PRFLT 未写入(系统不自动写飞控参数); "
          "需延长解锁后自动上锁时间请点[写入发射箱参数]")
    return True


def write_disarm_delay(master):
    """手动写入自动上锁参数 ([写入发射箱参数] 按钮调用): 用户显式点击即执行,
    不因本会话已写过而跳过; 成功后置标记, 后续解锁/起飞只提示不写入.
    APM 的 DISARM_DELAY → PX4 的 COM_DISARM_PRFLT (解锁后起飞太慢自动上锁)"""
    global DISARM_DELAY_WRITTEN
    if not DISARM_DELAY_SEC or DISARM_DELAY_SEC <= 0:
        return True
    print(f"  🔧 手动设置 COM_DISARM_PRFLT={DISARM_DELAY_SEC}s...")
    if set_param(master, 'COM_DISARM_PRFLT', float(DISARM_DELAY_SEC), timeout=2.5):
        DISARM_DELAY_WRITTEN = True
        print("  ✅ COM_DISARM_PRFLT 手动写入完成")
        return True
    print("  ⚠️ 写入 COM_DISARM_PRFLT 失败")
    return False


def get_param(master, name, timeout=1.5):
    """读单个参数, 成功返回 float, 超时/无此参数返回 None"""
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
        pn = msg.param_id
        if isinstance(pn, (bytes, bytearray)):
            pn = pn.decode('ascii', 'replace')
        pn = str(pn).split('\x00')[0]
        if pn == name:
            return float(msg.param_value)
    return None


def ensure_param(master, name, value, timeout=2.0):
    """确保参数=期望值: 先读回(无响应重试1次), 已正确则跳过写入; 不符才写并
    严格回读复核 (回读不符=失败, 不再同名即真).
    返回 True(已正确或写入生效) / False(读到但写入未生效=固件钳制) /
    None(读写均无响应, 固件可能无此参数)"""
    value = float(value)
    cur = get_param(master, name, timeout=timeout)
    if cur is None:
        cur = get_param(master, name, timeout=timeout)   # 读重试一次
        if cur is None:
            return True if set_param(master, name, value,
                                     timeout=timeout) else None
    if abs(cur - value) < 0.01:
        return True
    return set_param(master, name, value, timeout=timeout)


def _do_launch_params(master):
    """发射箱参数预设核心逻辑: 只写入值不符的参数 (PX4 参数组)"""
    if not LAUNCH_PRESET:
        return True
    print(f"  发射箱参数预设 ({len(LAUNCH_PARAMS)} 项, 值不符才写)...")
    ok = True
    for pname, pval, label in LAUNCH_PARAMS:
        r = ensure_param(master, pname, pval, timeout=2.0)
        if r is True:
            print(f"    ✅ {pname}={pval:g}  {label}")
        elif r is None:
            print(f"    ℹ️ {pname} 固件无此参数, 跳过 ({label})")
        else:
            ok = False
            print(f"    ⚠️ {pname} 写入未生效(回读不符), 仍继续 ({label})")
    if not ok:
        print("  ⚠️ 部分参数未确认, 若仍自动返航/降落请在 QGroundControl "
              "参数列表中手动核对上表参数")
    return ok


def ensure_launch_params(master):
    """解锁前不再写发射箱参数 (系统只做飞行控制, 不碰飞控参数).
    这里不发任何参数指令, 只在本会话尚未手动写入时打印提示.
    返回 True(恒真, 不阻断解锁)"""
    if LAUNCH_PARAMS_WRITTEN:
        return True
    if LAUNCH_PRESET:
        print("  ℹ️ 发射箱参数未写入(系统不自动写飞控参数); "
              "需要时请点[写入发射箱参数]")
    return True


def write_launch_params(master):
    """手动写入发射箱参数（[写入发射箱参数] 按钮调用）: 用户显式点击即执行,
    不因本会话已写过而跳过; 成功后置标记, 后续解锁/起飞只提示不写入"""
    global LAUNCH_PARAMS_WRITTEN
    print("  🔧 手动写入发射箱参数...")
    result = _do_launch_params(master)
    if result:
        LAUNCH_PARAMS_WRITTEN = True
        print("  ✅ 发射箱参数手动写入完成")
    return result


# GUI 注入的确认钩子: callable(reason) -> bool; None 时走终端 input 确认
ARM_CONFIRM_HOOK = None
# 最近一次 confirm_arm 的就绪状态, GUI 弹窗标题据此切换
# (🚨检测一切正常 / ⚠️位置估计未就绪), 避免未就绪时仍谎报"一切正常"
ARM_CONFIRM_ALL_READY = True


def confirm_arm(reason='', all_ready=True):
    """解锁前(等待就绪后)醒目提示并等待操作员确认 (安全机制):
    重试路径可能在点击后数十秒才满足条件, 为防此时人员贴近飞机突然自解锁,
    必须由现场人员看到提示、确认安全后才继续 ARM.
    all_ready=False 表示位置估计未就绪等非全绿状态, 横幅改为警示文案;
    状态写入 ARM_CONFIRM_ALL_READY 供 GUI 弹窗标题使用.
    返回 True=确认继续 False=取消或超时"""
    global ARM_CONFIRM_ALL_READY
    ARM_CONFIRM_ALL_READY = bool(all_ready)
    print('=' * 56)
    if all_ready:
        print('  🚨 检测一切正常, 即将解锁电机! 🚨')
    else:
        print('  ⚠️ 位置估计未就绪, 解锁可能被拒! ⚠️')
    print('  ⚠️ 请所有人员远离飞机, 确认现场安全后再继续')
    if reason:
        print(f'  事由: {reason}')
    print('=' * 56)
    hook = ARM_CONFIRM_HOOK
    if hook is not None:
        try:
            ok = bool(hook(reason))
        except Exception as e:          # 钩子异常按取消处理, 安全兜底
            print(f'  ⚠️ 确认钩子异常({e}), 按取消处理')
            ok = False
    else:
        try:
            ans = input('  输入 y 回车确认解锁 (其它=取消): ')
            ok = str(ans).strip().lower() in ('y', 'yes', '是', '1')
        except EOFError:
            ok = False
    print('  ✅ 已确认, 执行解锁' if ok else '  ⛔ 未确认, 取消解锁')
    return ok


def print_motor_outputs(master, timeout=2.5):
    """解锁成功后打印电机输出指令 (诊断).
    区分"FC没给油门(受控停转, 正常)" 与 "FC给了油门但电机不转(硬件)";
    PX4 优先 ACTUATOR_OUTPUT_STATUS(375), 老链路固件可能发 SERVO_OUTPUT_RAW(36);
    固件未流式发送该消息时仅提示跳过, 不影响解锁结果"""
    for mid in (375, 36):       # 375=ACTUATOR_OUTPUT_STATUS, 36=SERVO_OUTPUT_RAW
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                mid, 0, 0, 0, 0, 0, 0)
        except Exception:
            pass
    t_end = time.time() + timeout
    while time.time() < t_end:
        m = master.recv_match(type=['ACTUATOR_OUTPUT_STATUS',
                                    'SERVO_OUTPUT_RAW'],
                              blocking=True, timeout=0.5)
        if m is None:
            continue
        mtype = m.get_type()
        if mtype == 'SERVO_OUTPUT_RAW':
            print(f"  电机指令 PWM: M1={m.servo1_raw} M2={m.servo2_raw} "
                  f"M3={m.servo3_raw} M4={m.servo4_raw} "
                  "(≈1000=受控停转, 解锁后属正常)")
            return
        if mtype == 'ACTUATOR_OUTPUT_STATUS':
            try:
                acts = list(m.actuator)[:4]
                txt = ' '.join('%+.2f' % float(a) for a in acts)
                print(f"  电机输出(归一化): M1~M4 = {txt} "
                      "(≈0=受控停转, 解锁后属正常)")
                return
            except (TypeError, ValueError, IndexError):
                continue
    print("  (固件未发送电机输出消息, 跳过电机输出检查)")


# ============================================================
#  PX4 模式辅助 (custom_mode 打包 / 切模式 / 反解)
# ============================================================

def _px4_mode_name(mode):
    """模式名翻译: APM(及通用)名 → PX4 模式名; 已是 PX4 名则原样返回.
    无法映射返回 None (如 AUTOTUNE — PX4 无此模式)"""
    if not mode:
        return None
    n = str(mode).strip().upper()
    if n in APM_TO_PX4_MODE:
        return APM_TO_PX4_MODE[n]
    if n in PX4_MODE_CUSTOM:
        return n
    return None


def px4_custom_mode(mode):
    """按 PX4 打包规则返回 custom_mode = (main_mode<<16)|(sub_mode<<24).
    查不到返回 None (优先用本模块表, 再退回 pymavlink px4_map)"""
    name = _px4_mode_name(mode)
    if name is None:
        return None
    if name in PX4_MODE_CUSTOM:
        return PX4_MODE_CUSTOM[name]
    try:
        _flags, main, sub = mavutil.px4_map[name]
        return (int(main) << 16) | (int(sub) << 24)
    except Exception:
        return None


def _px4_mode_triple(mode):
    """返回 (param1 flags, param2 main_mode, param3 sub_mode) 供
    MAV_CMD_DO_SET_MODE 兜底下发; 优先复用 pymavlink px4_map 的取值."""
    name = _px4_mode_name(mode)
    try:
        return mavutil.px4_map[name]
    except Exception:
        pass
    if name in PX4_MODE_CUSTOM:
        cm = PX4_MODE_CUSTOM[name]
        return (81, (cm >> 16) & 0xFF, (cm >> 24) & 0xFF)
    raise KeyError(f'unknown PX4 mode {mode}')


def set_flight_mode(master, mode, timeout=10.0, quiet=False):
    """切换 PX4 飞行模式并等待生效, 返回 True/False.
    1) 模式名先翻译(APM 名亦可: GUIDED→POSCTL / ALT_HOLD→ALTCTL ...);
    2) 心跳确认为 PX4 时优先 master.set_mode('POSCTL') — pymavlink 内置
       px4_map, 发 MAV_CMD_DO_SET_MODE(param1=flags, param2=main, param3=sub);
    3) 兜底直接 command_long_send MAV_CMD_DO_SET_MODE;
    4) 等 HEARTBEAT.custom_mode 等于打包值才算成功 (默认 10s 超时)."""
    name = _px4_mode_name(mode)
    if name is None:
        if not quiet:
            print(f"  ❌ 未知模式 {mode}, 无法切换 (PX4: {', '.join(sorted(PX4_MAIN_MODES.values()))} / LOITER 等)")
        return False
    target = px4_custom_mode(name)
    if target is None:
        if not quiet:
            print(f"  ❌ 未知模式 {mode}, 无法切换")
        return False

    sent = False
    try:
        ap = master.field('HEARTBEAT', 'autopilot', None)
    except Exception:
        ap = None
    if ap == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        try:
            if name in getattr(mavutil, 'px4_map', {}):
                master.set_mode(name)       # → set_mode_px4: DO_SET_MODE
                sent = True
        except Exception:
            sent = False
    if not sent:
        try:
            flags, main, sub = _px4_mode_triple(name)
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                flags, main, sub, 0, 0, 0, 0)
        except Exception as e:
            if not quiet:
                print(f"  ❌ 发送 DO_SET_MODE 失败: {e}")
            return False

    t_end = time.time() + timeout
    while time.time() < t_end:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and int(getattr(hb, 'custom_mode', 0)) == target:
            if not quiet:
                print(f"  ✅ 已切换 {name} 模式")
            return True
    if not quiet:
        print(f"  ❌ 切换 {name} 模式超时({timeout:g}s), custom_mode 未达 "
              f"0x{target:08X} (未解锁/位置估计无效时部分模式不可进)")
    return False


def px4_mode_from_custom(custom_mode):
    """由 HEARTBEAT.custom_mode 反解 PX4 模式名:
    main_mode = (cm >> 16) & 0xFF, sub_mode = (cm >> 24) & 0xFF;
    AUTO(4) 且有 sub_mode 时返回子模式名(LOITER/TAKEOFF/MISSION/RTL/LAND...),
    否则返回主模式名; 解不出返回 None"""
    try:
        cm = int(custom_mode)
    except (TypeError, ValueError):
        return None
    main = (cm >> 16) & 0xFF
    sub = (cm >> 24) & 0xFF
    if main == 4 and sub:
        return PX4_AUTO_SUBMODES.get(sub)
    return PX4_MAIN_MODES.get(main)


def _mode_name(master, custom_mode):
    """按 PX4 custom_mode 打包规则反查模式名; 非 PX4 打包值时退回
    mode_mapping 反查(兼容), 解不出返回 UNKNOWN(n)"""
    name = px4_mode_from_custom(custom_mode)
    if name:
        return name
    try:
        mode_map = master.mode_mapping() or {}
        for n, v in mode_map.items():
            if isinstance(v, tuple):
                continue            # px4_map 值是三元组, 不能直接比 custom_mode
            if v == custom_mode:
                return n
    except Exception:
        pass
    return f'UNKNOWN({custom_mode})'


# ============================================================
#  解锁 / 起飞 / 上锁
# ============================================================

def arm_vehicle(master, mode='POSCTL', timeout=10, retries=ARM_RETRY,
                fallback_mode='ALTCTL', gate=True):
    """在指定可解锁模式下解锁电机 (PX4 版).
    流程: 模式名翻译(亦接受 GUIDED/ALT_HOLD 等 APM 名) -> 预检门禁(需位置的
    模式先等 GPS+GLOBAL_POSITION_INT 就绪, 未就绪不发 ARM) -> 切模式并校验
    HEARTBEAT.custom_mode -> 发射箱参数只提示不写入 -> 发 ARM
    (MAV_CMD_COMPONENT_ARM_DISARM param1=1, PX4 无油门高位 PreArm, 不发 RC 覆盖)
    -> 校验 ACK + 心跳 ARMED; 被拒且原因为 GPS/位置未就绪时等就绪后自动重试
    retries 次.
    始终无法就绪时经现场确认降级 fallback_mode(默认 ALTCTL — 气压定高不依赖
    水平位置). gate=False 仅供内部递归重试/降级路径跳过, 避免重复等待.
    返回 True/False"""
    raw_mode = (mode or 'POSCTL')
    mode = _px4_mode_name(raw_mode)
    if mode is None:
        print(f"  ❌ 未知模式 {raw_mode}, 无法解锁")
        return False

    # 预检门禁 — 需位置的模式先等位置估计就绪, 未就绪不发 ARM:
    # 'Arming denied: ...position...' 是 PX4 mandatory 检查, 跳不掉,
    # POSCTL/OFFBOARD/MISSION 等本身也离不开位置估计, 只能等就绪.
    if gate and mode in POSITION_MODES and not master.motors_armed():
        ready, why = wait_position_ready(master, timeout=ARM_PREPARE_WAIT)
        if not ready:
            print(f"  ❌ 预检未通过({why}), 本次不发送 ARM "
                  f"({mode} 解锁必被 Arming denied 拒绝)")
            # 门禁失败不直接放弃 — 走与"ARM 被拒"相同的降级出口:
            # 室内 GPS 多径时位置估计永不到位, 只能降级到无需水平位置的
            # ALTCTL(气压定高) 才能解锁/起飞(测试14 等依赖该出口).
            if retries > 0 and mode != fallback_mode:
                if not confirm_arm(
                        f"位置估计未就绪({why}), {mode}解锁必被拒; "
                        f"确认则改用 {fallback_mode} 解锁(气压定高, 无需GPS)",
                        all_ready=False):
                    print("  ⛔ 未确认, 取消解锁")
                    return False
                print(f"  ⏳ 经确认, 改用 {fallback_mode} 模式解锁...")
                return arm_vehicle(master, mode=fallback_mode,
                                   timeout=timeout, retries=retries - 1,
                                   gate=False)
            print("    提示: 室外等 GPS 3D 定位 + 位置估计收敛(约 20~60s)再解锁;")
            print("          急用请点 [定高解锁] 走 ALTCTL(不依赖水平位置)")
            return False

    print(f"  切换 {mode} 模式(可解锁)...")
    if not set_flight_mode(master, mode, timeout=6):
        print(f"  ❌ 当前未进入 {mode}, 解锁会被飞控拒绝")
        return False

    if master.motors_armed():
        print("  ℹ️ 电机已经是解锁状态")
        return True

    # 只提示是否已手动写入发射箱参数, 不发任何参数指令
    ensure_launch_params(master)

    gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.5)
    if gps:
        print(f"  GPS: fix={gps.fix_type} 卫星={gps.satellites_visible}")

    # PX4 解锁无 APM 的"油门高位 PreArm", 不需要 RC 覆盖压油门
    print("  发送 ARM 命令 (PX4: MAV_CMD_COMPONENT_ARM_DISARM)...")
    drain_rx(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    t_end = time.time() + timeout
    last_text = ""
    texts = []              # 本次解锁收到的全部 STATUSTEXT (累积判据)
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
            text = str(text)
            last_text = text
            texts.append(text)      # 累积 (Arming denied 常被后续文本顶掉)
            print(f"    [飞控] {text}")
        elif mtype == 'COMMAND_ACK' and \
                msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("  ARM 命令已接受, 等待 ARMED 状态...")
            else:
                result_names = {
                    1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
                    4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
                }
                rn = result_names.get(msg.result, str(msg.result))
                print(f"  ❌ 解锁被拒绝: COMMAND_ACK={rn}")
                # 飞控常在 ACK 之后才发 Arming denied 文本, 多等一会再收
                more = collect_status_text(master, duration=2.0, sink=texts)
                if more:
                    last_text = more
                break
        elif mtype == 'HEARTBEAT' and \
                (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
    # 无论成败都再收一小段文本
    if not master.motors_armed():
        collect_status_text(master, duration=0.8, sink=texts)

    if master.motors_armed():
        print("  ✅ 电机解锁成功 (ARMED)")
        print_motor_outputs(master)      # 打印电机输出作为证据
        return True

    # 判据用"本次解锁收到的全部 STATUSTEXT" — 旧版只取最后一条,
    # 拒因文本常被其后的文本顶掉导致关键词匹配不到、重试根本不触发.
    # 现改为: 未解锁 + 拒因含关键词 → 进入就绪等待与自动重试.
    joined = ' | '.join(texts)
    reason = (joined or (last_text or '')).lower()
    hit = any(k in reason for k in ARM_RETRY_KEYS)
    if retries > 0 and hit:
        print(f"  ⏳ 拒因与 GPS/位置估计有关, 等待就绪后自动重试 "
              f"(剩余 {retries} 次)...")
        ready, why = wait_position_ready(master, timeout=ARM_RETRY_GPS_WAIT)
        if ready:
            # 就绪后不自动解锁, 须现场确认安全后才继续
            if not confirm_arm(
                    f"位置估计已就绪, 重试解锁(剩余{retries}次)"):
                return False
            return arm_vehicle(master, mode=mode, timeout=timeout,
                               retries=retries - 1, gate=False)
        # 未就绪(室内常见) → 诚实横幅, 确认后降级 fallback_mode
        if mode != fallback_mode:
            if not confirm_arm(
                    f"位置估计未就绪({why}), {mode}解锁必被拒; "
                    f"确认则改用 {fallback_mode} 解锁(剩余{retries}次)",
                    all_ready=False):
                return False
            print(f"  ⏳ 经确认, 改用 {fallback_mode} 模式解锁...")
            return arm_vehicle(master, mode=fallback_mode,
                               timeout=timeout, retries=retries - 1,
                               gate=False)
        print(f"  ❌ 位置估计始终未就绪({why}), 放弃重试")
        print("    提示: 到室外/靠窗等 GPS 3D 定位 + 位置估计收敛后重试")

    print(f"  ❌ 解锁失败(超时 {timeout}s), 电机未进入 ARMED")
    print("  排查顺序:")
    print("    1) Arming denied + position/GPS/estimate: PX4 位置类检查是"
          " mandatory(跳不掉), 须 GPS 3D 定位 + 估计器收敛; 解锁前软件会先"
          "预检等待(%ds), 超时则不发 ARM" % ARM_PREPARE_WAIT)
    print("       到室外等 20~60s; 仍偏严可检查 EKF2_* / COM_ARM_EKF 相关参数;")
    print("       急用/室内可点[定高解锁]走 ALTCTL(不依赖水平位置)")
    print("    2) 看上方 [飞控] Arming denied: 具体原因 (GPS/校准/安全开关/健康)")
    print("    3) 无遥控器纯软件控制: 系统不自动写参数, 需自行点"
          "[写入发射箱参数] 写入 COM_RCL_EXCEPT 等; 若报 RC lost 先核对该组参数")
    print("    4) 遥控器开机: 确认 RC 已配对/摇杆回中, 或由 GCS 直接解锁")
    print("    5) 未校准(加速度计/罗盘/陀螺仪)会直接 Arming denied;"
          "用 QGC 完成传感器校准")
    print("    6) 黄灯 = PreArm 未通过; 修好原因后再跑测试2")
    return False


def send_guided_takeoff(master, alt):
    """发 MAV_CMD_NAV_TAKEOFF 让 PX4 切 Takeoff 模式并爬升 (不等待到位).
    PX4 语义: 先保证 MIS_TAKEOFF_ALT=alt (读回不符才写), param7 传 NaN
    用飞控设定高度; 参数不可写时兜底 param7=alt. param5/6=NaN 表示用当前
    经纬度(传 0 会指到几内亚湾). 返回 True=已发出且未被拒 / False=被拒"""
    alt = float(alt)
    ensure = ensure_param(master, 'MIS_TAKEOFF_ALT', alt, timeout=2.0)
    nan = float('nan')
    p7 = float(alt) if ensure is False else nan
    if ensure is False:
        print("  ⚠️ MIS_TAKEOFF_ALT 写入未确认, 改用 param7 指定起飞高度")
    drain_rx(master)
    # confirmation=0, p1~p4=0, p5/p6=NaN(用当前经纬度), p7=高度
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, nan, nan, p7)
    t_end = time.time() + 1.5
    while time.time() < t_end:
        msg = master.recv_match(blocking=True,
                                timeout=max(0.2, t_end - time.time()))
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'COMMAND_ACK' and \
                msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("  ✅ NAV_TAKEOFF 已接受 (PX4 → Takeoff 模式爬升)")
                return True
            print(f"  ❌ NAV_TAKEOFF 被拒(result={msg.result})")
            collect_status_text(master, duration=1.5)
            return False
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            print(f"    [飞控] {text}")
    return True         # 无 ACK 也视为已发出 (由 wait_for_alt 判定到位)


def arm_and_takeoff(master, alt, mode='TAKEOFF', arm_timeout=10,
                    takeoff_timeout=45, set_delay=True):
    """解锁 + 立即起飞集成(测试4/14/起飞按钮): 避免"先测2解锁、再测4起飞时
    已被 COM_DISARM_PRFLT 自动上锁".
    PX4 流程: 按 mode 翻译出可解锁模式(默认 TAKEOFF → 先在 POSCTL 解锁) ->
    解锁成功后立即发 NAV_TAKEOFF(MIS_TAKEOFF_ALT=alt, param7=NaN) -> 等
    GLOBAL_POSITION_INT.relative_alt 到位.
    set_delay=True: 仅提示自动上锁参数是否已手动写入, 不写参数.
    返回 True/False"""
    if set_delay:
        ensure_disarm_delay(master)

    arm_mode = _px4_mode_name(mode) or 'POSCTL'
    if arm_mode in ('TAKEOFF', 'READY'):
        # PX4 不在 AUTO 子模式里直接 ARM: 先在位置模式解锁, 再由
        # NAV_TAKEOFF 让飞控切 Takeoff 子模式爬升
        arm_mode = 'POSCTL'

    if not arm_vehicle(master, mode=arm_mode, timeout=arm_timeout,
                       fallback_mode='ALTCTL'):
        return False

    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    cur = _mode_name(master, hb.custom_mode) if hb is not None else '?'
    if cur in ('MANUAL', 'STABILIZED', 'ACRO', 'RATTITUDE'):
        print(f"  ⚠️ 当前模式 {cur} 不支持自动起飞, 先切 POSCTL ...")
        if not set_flight_mode(master, 'POSCTL', timeout=6):
            print(f"  ❌ 切换 POSCTL 失败 (需 {mode}), 中止起飞")
            return False
        cur = 'POSCTL'
    print(f"  [起飞] 解锁后立即发 NAV_TAKEOFF → {alt}m "
          f"(须在 {ARM_TO_TAKEOFF_MAX}s 内发出, 防 COM_DISARM_PRFLT 自动上锁)...")
    # arm_vehicle 已确认 ARMED; 走快速路径, 不再重复切模式读 GPS
    if not send_guided_takeoff(master, alt):
        print("  ❌ NAV_TAKEOFF 被飞控拒绝")
        return False
    ok = wait_for_alt(master, alt, tolerance=0.5, timeout=takeoff_timeout)
    if not ok and master.motors_armed():
        # 兜底: 高度长时间不动时显式切 TAKEOFF (PX4 自身的 Takeoff 模式)
        print("  ⚠️ NAV_TAKEOFF 未见爬升, 兜底显式切 TAKEOFF 模式再等 ...")
        set_flight_mode(master, 'TAKEOFF', timeout=5, quiet=True)
        ok = wait_for_alt(master, alt, tolerance=0.5,
                          timeout=max(15, takeoff_timeout // 2))
    return ok


def disarm_vehicle(master, force=False, timeout=5, _fallback=False):
    """上锁电机. force=True 时 param2=21196 强制上锁, 仅地面/异常时使用
    (PX4 同样识别该 magic 值).
    当前方式被固件拒绝时自动换另一种方式再试一次
    (_fallback 防双向循环; 21196 必须放 param2, 放 param1 会回 UNSUPPORTED=3).
    返回 True/False"""
    if not master.motors_armed():
        print("  ℹ️ 电机已是上锁状态")
        return True
    param2 = 21196 if force else 0
    print("  发送 DISARM 命令..." + (" (强制)" if force else ""))
    drain_rx(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, param2, 0, 0, 0, 0, 0)

    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            if not master.motors_armed():
                print("  ✅ 电机已锁定 (DISARMED)")
                return True
            continue
        mtype = msg.get_type()
        if mtype == 'COMMAND_ACK' and \
                msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                pass
            elif not _fallback:
                alt = '强制' if not force else '普通'
                print(f"  ⚠️ DISARM 被拒绝(result={msg.result}), 改用{alt}上锁...")
                return disarm_vehicle(master, force=not force,
                                      timeout=timeout, _fallback=True)
            else:
                print(f"  ❌ DISARM 两种方式均失败(result={msg.result})")
                return False
        elif mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            print(f"    [飞控] {text}")
        elif mtype == 'HEARTBEAT' and not (
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("  ✅ 电机已锁定 (DISARMED)")
            return True
    if not master.motors_armed():
        print("  ✅ 电机已锁定 (DISARMED)")
        return True
    print(f"  ❌ 上锁超时({timeout}s)")
    return False


def reboot_flight_controller(master, timeout=15):
    """重启飞控 (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1).
    已解锁时拒绝; 发送后等心跳中断 ≥2.5s 判定重启开始.
    返回 True=检测到重启开始 / False=拒绝或超时未见心跳中断"""
    if master.motors_armed():
        print("  ❌ 电机已解锁, 禁止重启飞控; 请先上锁再重启")
        return False
    print("  🔁 发送飞控重启命令 (PARAM1=1), 等待心跳中断判定...")
    drain_rx(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    last_hb = time.time()
    while time.time() - t0 < timeout:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if hb is not None:
            last_hb = time.time()
        elif time.time() - last_hb >= 2.5:
            print("  ✅ 心跳已中断, 飞控重启中 (约 10~30s 后重新上线, "
                  "USB 虚拟串口可能需重新连接)")
            return True
    print("  ⚠️ 未检测到心跳中断: 重启命令可能未被执行(超时)")
    return False


def get_current_alt(master):
    """获取当前相对高度"""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg:
        return msg.relative_alt / 1000.0
    return 0.0


def get_current_pos(master):
    """获取当前经纬度和高度"""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg:
        return {
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.relative_alt / 1000.0
        }
    return None


def wait_for_alt(master, target_alt, tolerance=0.5, timeout=60):
    """等待到达指定高度"""
    start = time.time()
    while time.time() - start < timeout:
        alt = get_current_alt(master)
        print(f"  当前高度: {alt:.1f}m / 目标: {target_alt}m", end='\r')
        if abs(alt - target_alt) < tolerance:
            print(f"\n  ✅ 已到达 {target_alt}m")
            return True
        time.sleep(0.5)
    print(f"\n  ⚠️ 超时，当前高度: {get_current_alt(master):.1f}m")
    return False


# ============================================================
#  OFFBOARD 速度 setpoint (测试5/6/7/8)
# ============================================================

def send_velocity_setpoint(master, vx, vy, vz):
    """发一条 NED 速度 setpoint (SET_POSITION_TARGET_LOCAL_NED):
    coordinate_frame=LOCAL_NED, type_mask 只留 vx,vy,vz (忽略位置/加速度/偏航).
    PX4 要求 OFFBOARD 下持续 >2Hz 的 setpoint 流"""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        OFFBOARD_VEL_MASK,
        0, 0, 0,
        float(vx), float(vy), float(vz),
        0, 0, 0, 0, 0
    )


def send_zero_velocity(master, count=20):
    """发送零速度指令让飞机悬停 (未在 OFFBOARD 时 PX4 忽略, 无副作用)"""
    for _ in range(count):
        send_velocity_setpoint(master, 0, 0, 0)
        time.sleep(0.1)


def enter_offboard(master, vx=0.0, vy=0.0, vz=0.0, warmup=None, timeout=6.0):
    """进入 OFFBOARD 并开始速度控制的前置流程 (返回 True/False).
    PX4 要求切换进 OFFBOARD 前至少已连续流出 1 秒 setpoint (≥5Hz, 否则
    拒绝进入): 先以 ~6.7Hz 发 OFFBOARD_WARMUP_SEC 秒 setpoint, 再切模式
    并校验心跳 custom_mode; 失败打印原因并返回 False. 进入成功后由调用方
    按 CONTROL_RATE 持续发送."""
    if warmup is None:
        warmup = OFFBOARD_WARMUP_SEC
    print(f"  进入 OFFBOARD: 先以 ≥5Hz 发 {warmup:g}s setpoint 流 ...")
    t_end = time.time() + max(0.5, float(warmup))
    while time.time() < t_end:
        send_velocity_setpoint(master, vx, vy, vz)
        time.sleep(0.15)                    # ≈6.7Hz (>2Hz 要求)
    if not set_flight_mode(master, 'OFFBOARD', timeout=timeout):
        print("  ❌ 进入 OFFBOARD 失败(setpoint 流未被接受或位置估计无效), "
              "本项中止")
        return False
    if not master.motors_armed():
        print("  ⚠️ 电机未解锁: 仅验证 setpoint 链路, 飞机不会动")
    return True


def exit_offboard(master, back='POSCTL'):
    """速度测试收尾: 先补一段零速 setpoint, 再回 POSCTL 定点 —
    避免停在 OFFBOARD 无 setpoint 流触发 COM_OF_LOSS_T 丢失保护"""
    send_zero_velocity(master, count=6)
    if set_flight_mode(master, back, timeout=5, quiet=True):
        print(f"  ✅ 已回 {back} 定点")
    else:
        print(f"  ⚠️ 未确认已回 {back}, 请留意 OFFBOARD 丢失保护动作")


def wait_gps_lock(master, min_sats=None, timeout=None):
    """等待 GPS 3D 定位且卫星数足够（调参/位置模式前置条件）"""
    # 默认值运行时取模块全局 (def 时求值会让 UI 动态改值不生效)
    if min_sats is None:
        min_sats = AUTOTUNE_MIN_SATS
    if timeout is None:
        timeout = 30
    print("⏳ 等待 GPS 锁星...")
    t_end = time.time() + timeout
    while time.time() < t_end:
        g = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if g and g.fix_type >= 3 and g.satellites_visible >= min_sats:
            print(f"  ✅ GPS 已定位: fix_type={g.fix_type} 卫星={g.satellites_visible}颗")
            return True
    print("  ❌ GPS 定位超时")
    return False


def ekf_pos_ready(flags):
    """位置估计是否就绪 (函数名保留兼容 API/UI, 判据改为 PX4 语义):
    PX4 不发 EKF_STATUS_REPORT, flags 是本模块的"位置消息到达"位掩码 —
      0x1 = 已收到 GPS 全局位置 (GLOBAL_POSITION_INT 经纬度非0),
      0x2 = 已收到本地位置 (LOCAL_POSITION_NED).
    判据 = 已拿到 GPS 全局位置 (本地位置由估计器随之给出, 仅作辅助显示)."""
    f = int(flags)
    return bool(f & POS_FLAG_GPS_GLOBAL)


def wait_ekf_position(master, timeout=None):
    """等待位置估计就绪 (函数名保留兼容, 判据改为 PX4 语义).
    APM 版监听 EKF_STATUS_REPORT flags — PX4 不发该消息, 改为等待收到
    GLOBAL_POSITION_INT(经纬度非0); 最后 5s 仍无任何位置消息则按链路
    限制放行(兼容旧逻辑). 返回 True(就绪/放行) False(超时未就绪)"""
    # timeout=None -> 运行时读 ARM_RETRY_GPS_WAIT (支持动态调整)
    if timeout is None:
        timeout = ARM_RETRY_GPS_WAIT
    print("⏳ 等待位置估计就绪 (PX4: GLOBAL_POSITION_INT)...")
    t_end = time.time() + timeout
    grace_at = t_end - 5.0               # 最后5秒仍无位置消息 -> 放行
    seen = False
    flags = 0
    while time.time() < t_end:
        m = master.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'],
                              blocking=True, timeout=1)
        if m is None:
            if not seen and time.time() > grace_at:
                print("  ⚠️ 未收到位置消息, 按链路限制放行")
                return True
            continue
        seen = True
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if getattr(m, 'lat', 0) or getattr(m, 'lon', 0):
                flags |= POS_FLAG_GPS_GLOBAL
        else:
            flags |= POS_FLAG_LOCAL
        if ekf_pos_ready(flags):
            print(f"  ✅ 位置估计就绪 (flags=0x{flags:x})")
            return True
    print("  ❌ 位置估计未就绪(超时)")
    return False


def position_ready_state(fix, sats, flags, min_sats=None,
                         require_ekf=True):
    """位置估计是否已就绪(可安全在位置模式解锁). 纯函数, 供 arm_vehicle 预检
    门禁与 GUI 状态条共用 (GUI 从共享遥测取参, 避免抢 recv_match).
    fix/sats/flags 可为 None(尚未收到对应消息).
    flags 语义 (PX4 版, 见 POS_FLAG_* 注释): 0x1=已收到 GPS 全局位置,
    0x2=已收到本地位置 — 不再是 EKF_STATUS_REPORT 位.
    require_ekf=False: 未收到 GLOBAL_POSITION_INT 时降级为只看 GPS.
    min_sats=None: 运行时读 POS_READY_MIN_SATS (支持 UI 动态调整;
    若在 def 时绑定则 UI 改 tct.POS_READY_MIN_SATS 不生效).
    返回 (ready: bool, why: str) — why 为人话化缺项, 就绪时 '就绪'"""
    if min_sats is None:
        min_sats = POS_READY_MIN_SATS
    if fix is None:
        return False, '未收到GPS'
    if fix < 3:
        return False, '等GPS 3D定位(fix=%s)' % fix
    if sats is not None and sats < min_sats:
        return False, '卫星不足(%s<%d)' % (sats, min_sats)
    if not require_ekf:
        return True, '就绪(仅GPS, 未收到位置消息降级)'
    if flags is None:
        return False, '未收到位置状态'
    f = int(flags)
    if not ekf_pos_ready(f):
        if not (f & POS_FLAG_GPS_GLOBAL):
            return False, '未收到全局位置(GLOBAL_POSITION_INT), flags=0x%02x' % f
        return False, '位置状态异常(flags=0x%02x)' % f
    return True, '就绪'


def wait_position_ready(master, timeout=None, poll=0.25):
    """解锁前预检门禁: 主动等待 GPS 3D 定位 + 位置估计就绪 (PX4 版).
    背景: 位置类模式的 'Arming denied: ...position...' 是 PX4 mandatory 检查,
    且 POSCTL/OFFBOARD/MISSION 本身离不开位置估计, 只能等就绪;
    上电后估计收敛需 20~60s, 此前盲发 ARM 必被拒.
    判据: GPS_RAW_INT(fix>=3, 卫星>=POS_READY_MIN_SATS) +
    GLOBAL_POSITION_INT(经纬度非0 → 位 0x1); PX4 不发 EKF_STATUS_REPORT.
    这里每 1s 主动请求一次位置类消息(REQUEST_MESSAGE + DATA_STREAM_POSITION),
    超过 POS_EKF_MSG_GRACE 仍无 GLOBAL_POSITION_INT 判定该链路不流式发送,
    降级只看 GPS (兜底放行).
    timeout=None: 运行时读 ARM_PREPARE_WAIT (支持 UI 动态调整).
    每 5s 打一行进度(带具体缺项). 返回 (ready: bool, why: str)"""
    if timeout is None:
        timeout = ARM_PREPARE_WAIT
    t0 = time.time()
    timeout = max(1.0, float(timeout))
    t_end = t0 + timeout
    # 超过此时限仍无 GLOBAL_POSITION_INT -> 降级只看 GPS
    pos_grace = t0 + min(float(POS_EKF_MSG_GRACE),
                         max(5.0, timeout / 3.0))
    fix = sats = flags = None
    no_pos_msg = False
    last_report = -5.0

    def _request():
        for mid in (33, 32):   # 33=GLOBAL_POSITION_INT, 32=LOCAL_POSITION_NED
            try:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                    mid, 0, 0, 0, 0, 0, 0)
            except Exception:
                pass
        try:
            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)
        except Exception:
            pass

    _request()
    print("⏳ 预检: 等待位置估计就绪 (最多 %ds)..." % timeout)
    last_req = t0
    while time.time() < t_end:
        m = master.recv_match(blocking=True, timeout=poll)
        now = time.time()
        # 每秒重发一次请求: 保证不依赖飞控是否主动流式发送
        if now - last_req >= 1.0:
            last_req = now
            _request()
        if m is not None:
            mtype = m.get_type()
            if mtype == 'GPS_RAW_INT':
                fix = int(getattr(m, 'fix_type', 0) or 0)
                sats = int(getattr(m, 'satellites_visible', 0) or 0)
            elif mtype == 'GLOBAL_POSITION_INT':
                flags = int(flags or 0)
                if getattr(m, 'lat', 0) or getattr(m, 'lon', 0):
                    flags |= POS_FLAG_GPS_GLOBAL
            elif mtype == 'LOCAL_POSITION_NED':
                flags = int(flags or 0) | POS_FLAG_LOCAL
            elif mtype == 'STATUSTEXT':
                txt = m.text
                if isinstance(txt, (bytes, bytearray)):
                    txt = txt.decode('utf-8', 'replace')
                low = str(txt).lower()
                if ('ekf' in low or 'gps' in low or 'origin' in low
                        or 'position' in low or 'nav' in low):
                    print("    [飞控] %s" % txt)
        got_pos = bool(int(flags or 0) & POS_FLAG_GPS_GLOBAL)
        if not no_pos_msg and not got_pos and now > pos_grace:
            no_pos_msg = True
            print("  ⚠️ 未收到 GLOBAL_POSITION_INT, 改按 GPS 3D 定位判定")

        ready, why = position_ready_state(fix, sats, flags,
                                          require_ekf=not no_pos_msg)
        if ready:
            print("  ✅ 位置估计就绪: %s (fix=%s 卫星=%s%s)" % (
                why, fix, sats,
                '' if flags is None else ' flags=0x%02x' % flags))
            return True, why
        if now - last_report >= 5.0:
            last_report = now
            print("  ⏳ 等待位置估计就绪 (%ds/%ds): %s" % (
                int(now - t0), int(timeout), why))

    _, why = position_ready_state(fix, sats, flags,
                                  require_ekf=not no_pos_msg)
    print("  ❌ 位置估计未就绪(超时%ds): %s" % (int(timeout), why))
    return False, why


def release_rc_override(master, handover=None):
    """兼容空实现 (PX4): 本模块全程不发 RC 覆盖 — PX4 无 APM 的"油门高位
    PreArm", 解锁不需要压油门, 也不用覆盖帧做软遥控收尾.
    调用仅清理内部状态 (APM 版会发全0交还/中立帧, PX4 下不注入任何帧,
    避免覆盖机上真实 RC 输入). handover 参数保留签名兼容. 返回 None"""
    return None


def takeoff_althold(master, alt, timeout=90, climb_thr=None):
    """定高起飞到 alt 米 (签名兼容 APM 的 ALT_HOLD 杆位闭环爬升版).
    PX4 语义改写: 不发 RC 杆位/油门帧 (PX4 不吃地面站覆盖爬升), 改为
    NAV_TAKEOFF — 未解锁先在 ALTCTL 解锁(不依赖水平位置), 再写
    MIS_TAKEOFF_ALT + param7=NaN 让飞控切 Takeoff 模式闭环爬升, 等
    GLOBAL_POSITION_INT.relative_alt 到位.
    climb_thr 仅为签名兼容 (PX4 无杆位爬升, 见 TAKEOFF_CLIMB_THR 注释).
    返回 True(到位)/False(失败或超时)"""
    print(f"  ⬆️ 定高起飞至 {alt}m (PX4: NAV_TAKEOFF → Takeoff 模式爬升, "
          f"climb_thr={climb_thr} 仅兼容不生效)...")
    if not master.motors_armed():
        print("  [起飞] 尚未解锁, 先在 ALTCTL 模式解锁 (无需水平位置)...")
        if not arm_vehicle(master, mode='ALTCTL', timeout=10, gate=False):
            print("  ❌ 解锁失败, 无法起飞")
            return False
    if not send_guided_takeoff(master, alt):
        print("  ❌ NAV_TAKEOFF 被飞控拒绝")
        return False
    return wait_for_alt(master, alt, tolerance=0.5, timeout=timeout)


def land_soft_final(master, phase0_timeout=120):
    """LAND/RTL 等待着陆与自动上锁 (PX4 版"软着陆").
    APM 版最后 1m 的 RC 油门微调在 PX4 不适用 — LAND/RTL 由飞控闭环缓降,
    着陆检测后按 COM_DISARM_LAND (默认 2s) 自动上锁. 本函数改为监控:
      · EXTENDED_SYS_STATE.landed_state == MAV_LANDED_STATE_ON_GROUND, 或
      · 相对高度 < SOFT_LAND_GROUND_ALT 且心跳已显示 DISARMED;
    着陆后给 10s 等自动上锁, 仍未上锁则再发一次普通 DISARM 兜底.
    返回 True=已着地上锁; False=等待着陆超时(请人工处置)"""
    # 主动请求一次 EXTENDED_SYS_STATE(245), 提高 landed_state 可见性
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            245, 0, 0, 0, 0, 0, 0)
    except Exception:
        pass
    print("  ⬇️ 等待着陆: PX4 LAND/RTL 闭环缓降, 着陆检测后按 COM_DISARM_LAND "
          "自动上锁...")
    t_end = time.time() + phase0_timeout
    last_rep = 0.0
    alt = None
    seen_disarm = False
    landed = False
    while time.time() < t_end:
        m = master.recv_match(type=['EXTENDED_SYS_STATE',
                                    'GLOBAL_POSITION_INT', 'HEARTBEAT'],
                              blocking=True, timeout=0.5)
        now = time.time()
        if m is None:
            continue
        mtype = m.get_type()
        if mtype == 'EXTENDED_SYS_STATE':
            try:
                landed_state = int(m.landed_state)
            except (AttributeError, TypeError, ValueError):
                landed_state = -1
            if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                landed = True
                break
        elif mtype == 'GLOBAL_POSITION_INT':
            alt = m.relative_alt / 1000.0
            if alt <= SOFT_LAND_GROUND_ALT:
                landed = True
                break
        elif mtype == 'HEARTBEAT':
            if not (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                seen_disarm = True
                if alt is None or alt <= SOFT_LAND_GROUND_ALT:
                    landed = True
                    break
        if now - last_rep >= 1.0:
            last_rep = now
            print(f"  下降中... 高度: "
                  f"{'?m' if alt is None else '%.1fm' % alt} "
                  f"(着陆后自动上锁)", end='\r')
    if not landed:
        print(f"\n  ⚠️ {phase0_timeout}s 内未确认着陆, 请人工处置")
        return False

    # 着陆确认后等自动上锁 (COM_DISARM_LAND 默认 2s, 这里留 10s 裕量)
    t2 = time.time() + 10.0
    while time.time() < t2:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        if not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("\n  ✅ 已着陆, 电机自动上锁 (COM_DISARM_LAND)")
            return True
    if seen_disarm and not master.motors_armed():
        print("\n  ✅ 已着陆, 电机已上锁")
        return True
    print("\n  ⚠️ 已着陆但未见自动上锁, 补发 DISARM...")
    return disarm_vehicle(master, force=False, timeout=6)


def wait_autotune(master, timeout=AUTOTUNE_TIMEOUT):
    """等待 PID 调参完成 (函数名保留兼容).
    PX4 没有 MAVLink AUTOTUNE 模式, 本函数仅为占位: 提示改用
    QGroundControl 的 AutoTune, 返回 False (未在本模块执行).
    监听期间收到的 STATUSTEXT 会顺带打印, 便于观察飞控提示."""
    print(f"  ⚙️ PX4 无 MAVLink AUTOTUNE 模式 — 请在 QGroundControl 中运行 "
          f"AutoTune (占位函数 {timeout}s 内仅打印飞控提示):")
    t_end = time.time() + min(float(timeout), 10.0)
    last_report = 0.0
    start = time.time()
    while time.time() < t_end:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
        if msg is None:
            if time.time() - last_report >= 5.0:
                last_report = time.time()
                print(f"    ... 等待 QGC 调参提示, 已用 {int(time.time() - start)}s")
            continue
        text = msg.text
        if isinstance(text, (bytes, bytearray)):
            text = text.decode('utf-8', 'replace')
        print(f"    [飞控] {text}")
    return False


def auto_tune_flow(master, alt=None):
    """PID 自动调参流程 (APM 版接口保留; 测试11 已改为参数读写校验).
    PX4 调参请用 QGroundControl AutoTune, 本函数仅提示并返回 False."""
    if alt is None:
        alt = AUTOTUNE_ALT
    print("\n" + "="*50)
    print(f"🎛 PID 自动调参 (PX4, 建议高度 {alt}m)")
    print("="*50)
    print("  ⚠️ PX4 无 MAVLink AUTOTUNE 模式: 请用 QGroundControl 运行 AutoTune;")
    print("      本模块对应项为 测试11「参数读写校验」")
    return wait_autotune(master, timeout=10)


# ============================================================
#  各功能测试项
# ============================================================

def test_01_connection(master):
    """测试1：通信连接"""
    print("\n" + "="*50)
    print("📡 测试1：通信连接")
    print("="*50)

    # 发送心跳请求
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    # 接收心跳
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        autopilot = msg.autopilot
        system_status = msg.system_status
        mode = _mode_name(master, msg.custom_mode)
        print("  ✅ 收到心跳包")
        print(f"     飞控类型: {autopilot}"
              + (" (PX4)" if autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                 else " (非PX4,请确认)"))
        print(f"     系统状态: {system_status}")
        print(f"     飞行模式: {mode} (custom_mode=0x{int(msg.custom_mode):08X})")
        return True
    else:
        print("  ❌ 未收到心跳包")
        return False


def test_02_arm(master):
    """测试2：解锁电机(自动切到可解锁模式并打印 Arming denied 原因)"""
    print("\n" + "="*50)
    print("🔓 测试2：解锁电机")
    print("="*50)
    print("  说明: 解锁前自动切 POSCTL(位置估计未就绪时经确认降级 ALTCTL); "
          "PX4 无油门高位 PreArm, 无需压油门")
    ensure_disarm_delay(master)
    ok = arm_vehicle(master, mode='POSCTL', timeout=10)
    if ok:
        print(f"  ⚠️ 仅解锁未起飞: 约 {DISARM_DELAY_SEC}s 内不发起飞命令会自动上锁"
              "(COM_DISARM_PRFLT)")
        print("     起飞请直接用 测试4 / 测试14(内部已集成 解锁+起飞)")
    return ok


def test_03_mode_switch(master):
    """测试3：模式切换(结束强制回到 POSCTL, 避免停在 RTL 导致无法解锁)"""
    print("\n" + "="*50)
    print("🔄 测试3：飞行模式切换")
    print("="*50)
    print("  说明: 请在未解锁状态运行本项 (解锁状态下 RTL/LOITER 会真的动作)")

    # PX4 模式序列 (pymavlink px4_map 键名全大写):
    # 需位置的 POSCTL/LOITER/RTL 要有 GPS+位置估计, 缺失时会如实显示 ❌
    # 注意: 不能停在 RTL/LAND 等模式, 结束后切回 POSCTL
    modes = ["POSCTL", "ALTCTL", "LOITER", "RTL"]

    for mode in modes:
        ok = set_flight_mode(master, mode, timeout=6, quiet=True)
        time.sleep(0.3)

        # 清掉缓冲里的旧心跳，确保读到切换后的新心跳
        drain_rx(master)

        # 读取当前模式 (按 custom_mode 打包规则反解)
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            current_mode = _mode_name(master, msg.custom_mode)
            mark = "✅" if (ok and current_mode == mode) else "❌"
            print(f"  {mark} 切换到 {mode} → 当前模式: {current_mode}")
        else:
            print(f"  ❌ 切换到 {mode} 失败（未收到心跳）")

    print("  切回 POSCTL(可解锁模式, 防止停在 RTL 无法解锁)...")
    set_flight_mode(master, 'POSCTL', timeout=6, quiet=True)
    time.sleep(0.3)
    drain_rx(master)
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        cur = _mode_name(master, msg.custom_mode)
        print(f"  ✅ 模式切换测试完成, 当前: {cur}")
    else:
        print("  ⚠️ 模式切换测试完成, 但未确认最终模式")
    return True


def test_04_takeoff(master):
    """测试4：起飞(集成解锁+起飞, 无需先跑测试2)"""
    print("\n" + "="*50)
    print(f"🛫 测试4：起飞到 {TAKEOFF_ALT}m")
    print("="*50)
    print("  流程: (提示COM_DISARM_PRFLT) → POSCTL解锁 → 立即 NAV_TAKEOFF → 等待到位")
    print("  说明: 已集成解锁, 不再依赖测试2; 未起飞时也可直接选本项")

    result = arm_and_takeoff(master, TAKEOFF_ALT, mode='TAKEOFF',
                             arm_timeout=10, takeoff_timeout=45,
                             set_delay=True)
    if result:
        time.sleep(2)
        print(f"  ✅ 起飞完成, 悬停于 {TAKEOFF_ALT}m (原地上方, PX4 自动进 Hold)")
        print("     自动模式下摇杆无效属正常; 遥控器接管: 拨模式开关到 Position(POSCTL)")
    else:
        if not master.motors_armed():
            print("  ❌ 起飞失败(可能解锁失败或解锁后已被自动上锁)")
        else:
            print("  ❌ 起飞未到位(已解锁但高度未到目标)")
            print("     保持悬停中; 可试 测试9 降落 / 测试10 上锁")
    return result


def test_05_velocity_forward(master):
    """测试5：前进速度控制 (OFFBOARD NED 速度流)"""
    print("\n" + "="*50)
    print(f"➡️ 测试5：前进速度 {TEST_SPEED} m/s，持续 {TEST_DURATION}s")
    print("="*50)

    if not enter_offboard(master):
        return False
    start = time.time()

    while time.time() - start < TEST_DURATION:
        send_velocity_setpoint(master, TEST_SPEED, 0, 0)
        # 短超时取高度: 不能像 APM 版那样阻塞 3s, 否则 setpoint 断流触发保护
        m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1.0 / CONTROL_RATE)
        if m:
            print(f"  速度: {TEST_SPEED}m/s 前 | 高度: {m.relative_alt/1000.0:.1f}m",
                  end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    exit_offboard(master)
    print("\n  ✅ 前进测试完成")
    return True


def test_06_velocity_side(master):
    """测试6：侧向速度控制 (OFFBOARD NED 速度流)"""
    print("\n" + "="*50)
    print(f"↪️ 测试6：侧向速度 {TEST_SPEED} m/s，持续 {TEST_DURATION}s")
    print("="*50)

    if not enter_offboard(master):
        return False
    start = time.time()

    while time.time() - start < TEST_DURATION:
        send_velocity_setpoint(master, 0, TEST_SPEED, 0)   # NED 东向为正
        m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1.0 / CONTROL_RATE)
        if m:
            print(f"  速度: {TEST_SPEED}m/s 右 | 高度: {m.relative_alt/1000.0:.1f}m",
                  end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    exit_offboard(master)
    print("\n  ✅ 侧向测试完成")
    return True


def test_07_velocity_up(master):
    """测试7：垂直上升 (OFFBOARD NED 速度流)"""
    print("\n" + "="*50)
    print(f"⬆️ 测试7：垂直上升 {TEST_SPEED} m/s，持续 {TEST_DURATION}s")
    print("="*50)

    if not enter_offboard(master):
        return False
    start = time.time()

    while time.time() - start < TEST_DURATION:
        send_velocity_setpoint(master, 0, 0, -TEST_SPEED)  # NED 负Z=上升
        m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1.0 / CONTROL_RATE)
        if m:
            print(f"  速度: {TEST_SPEED}m/s 上升 | 高度: {m.relative_alt/1000.0:.1f}m",
                  end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    exit_offboard(master)
    print("\n  ✅ 垂直上升测试完成")
    return True


def test_08_hover(master):
    """测试8：悬停稳定性测试 (OFFBOARD 零速度流)"""
    print("\n" + "="*50)
    print(f"⏸ 测试8：悬停 {TEST_DURATION}s，观察高度漂移")
    print("="*50)

    if not enter_offboard(master):
        return False
    start = time.time()
    altitudes = []

    while time.time() - start < TEST_DURATION:
        send_velocity_setpoint(master, 0, 0, 0)
        m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=1.0 / CONTROL_RATE)
        if m:
            alt = m.relative_alt / 1000.0
            altitudes.append(alt)
            print(f"  悬停高度: {alt:.2f}m", end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    exit_offboard(master)
    if altitudes:
        drift = max(altitudes) - min(altitudes)
        print(f"\n  高度波动范围: {drift:.2f}m")
        if drift < 1.0:
            print("  ✅ 悬停稳定")
        else:
            print("  ⚠️ 悬停漂移较大，建议检查PID参数")
    return True


def test_09_land(master):
    """测试9：降落（LAND 模式闭环缓降, 着陆检测后自动上锁）"""
    print("\n" + "="*50)
    print("🛬 测试9：自动降落")
    print("="*50)

    send_zero_velocity(master)
    time.sleep(1)

    if not set_flight_mode(master, 'LAND', timeout=6, quiet=True):
        print("  ❌ 切换 LAND 模式失败")
        return False
    print("  已切换LAND模式...")

    return land_soft_final(master, phase0_timeout=90)


def test_10_disarm(master):
    """测试10：锁定电机"""
    print("\n" + "="*50)
    print("🔒 测试10：锁定电机")
    print("="*50)
    return disarm_vehicle(master, force=False, timeout=5)


def test_11_param_rw(master):
    """测试11：参数读写校验 (原 PID 自动调参; PX4 无 MAVLink AUTOTUNE 模式).
    流程: 等就绪 → 读取若干关键参数 → 各做一次写入-回读一致性校验
    (写回原值, 不改变飞控配置) → 全部一致才算通过"""
    print("\n" + "="*50)
    print("🛠 测试11：参数读写校验 (写回原值)")
    print("="*50)
    print("  说明: PX4 无 MAVLink AUTOTUNE 模式, 本项改为参数读写一致性校验; "
          "PID 调参请用 QGC AutoTune")

    if not wait_gps_lock(master, min_sats=AUTOTUNE_MIN_SATS, timeout=15):
        print("  ⚠️ GPS 未就绪 (参数读写不依赖 GPS, 继续校验)")

    names = ('MPC_TAKEOFF_ALT', 'COM_DISARM_LAND', 'MPC_XY_VEL_MAX')
    ok_cnt = 0
    bad = False
    for pname in names:
        cur = get_param(master, pname, timeout=1.5)
        if cur is None:
            print(f"    ℹ️ {pname} 读取失败/固件无此参数, 跳过")
            continue
        print(f"    {pname} = {cur:g} → 写回原值校验 ...")
        wr = set_param(master, pname, cur, timeout=2.5)
        again = get_param(master, pname, timeout=1.5)
        if again is not None and abs(again - cur) < 0.01:
            ok_cnt += 1
            print(f"    ✅ {pname} 写入-回读一致 ({again:g})")
        else:
            bad = True
            print(f"    ❌ {pname} 写入-回读不一致(写回结果={wr}, 现值={again})")
    if ok_cnt == 0:
        print("  ❌ 未能读到任何关键参数, 校验失败")
        return False
    if bad:
        print("  ⚠️ 部分参数读写不一致, 校验失败")
        return False
    print(f"  ✅ 参数读写校验通过 ({ok_cnt}/{len(names)} 项)")
    return True


def _fmt_optional(value, unit='', digits=1):
    """数值格式化, None/非法值显示为 --"""
    if value is None:
        return '--'
    if isinstance(value, float):
        return f'{value:.{digits}f}{unit}'
    return f'{value}{unit}'


def test_12_status(master):
    """测试12：飞机状态（HEARTBEAT/GPS/位置/姿态/VFR/SYS_STATUS，同 QGC）"""
    print("\n" + "="*50)
    print(f"📊 测试12：飞机状态（采集 {STATUS_LISTEN:.0f}s）")
    print("="*50)

    st = {
        'mode': None, 'armed': None, 'sys_status': None, 'base_mode': None,
        'fix': None, 'sats': None, 'hdop': None,
        'lat': None, 'lon': None, 'rel_alt': None, 'abs_alt': None,
        'vx': None, 'vy': None, 'vz': None, 'heading': None,
        'air_speed': None, 'ground_speed': None, 'throttle': None,
        'climb': None, 'alt_vfr': None,
        'roll': None, 'pitch': None, 'yaw': None,
        'voltage': None, 'current': None, 'remaining': None, 'load': None,
        'landed': None,
    }
    got = {'hb': False, 'pos': False}
    landed_names = {0: '未知', 1: '在地面', 2: '空中', 3: '着陆中',
                    4: '起飞中'}

    t_end = time.time() + STATUS_LISTEN
    last_draw = 0.0
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'HEARTBEAT':
            # 仅统计飞控心跳（非 GCS）
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            st['mode'] = _mode_name(master, msg.custom_mode)
            st['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            st['base_mode'] = msg.base_mode
            st['sys_status'] = msg.system_status
            got['hb'] = True
        elif mtype == 'GPS_RAW_INT':
            st['fix'] = msg.fix_type
            st['sats'] = msg.satellites_visible
            st['hdop'] = msg.eph * 0.01 if msg.eph != 65535 else None
        elif mtype == 'GLOBAL_POSITION_INT':
            st['lat'] = msg.lat / 1e7
            st['lon'] = msg.lon / 1e7
            st['rel_alt'] = msg.relative_alt / 1000.0
            st['abs_alt'] = msg.alt / 1000.0
            st['vx'] = msg.vx / 100.0
            st['vy'] = msg.vy / 100.0
            st['vz'] = msg.vz / 100.0
            st['heading'] = msg.hdg / 100.0 if msg.hdg != 65535 else None
            got['pos'] = True
        elif mtype == 'VFR_HUD':
            st['air_speed'] = msg.airspeed
            st['ground_speed'] = msg.groundspeed
            st['throttle'] = msg.throttle
            st['climb'] = msg.climb
            st['alt_vfr'] = msg.alt
        elif mtype == 'ATTITUDE':
            st['roll'] = math.degrees(msg.roll)
            st['pitch'] = math.degrees(msg.pitch)
            st['yaw'] = math.degrees(msg.yaw) % 360.0
        elif mtype == 'SYS_STATUS':
            st['voltage'] = msg.voltage_battery / 1000.0
            st['current'] = (msg.current_battery / 100.0
                             if msg.current_battery >= 0 else None)
            st['remaining'] = (msg.battery_remaining
                               if msg.battery_remaining >= 0 else None)
            st['load'] = msg.load / 10.0
        elif mtype == 'EXTENDED_SYS_STATE':
            # PX4 用 landed_state 表示着陆状态 (不发 EKF_STATUS_REPORT)
            try:
                st['landed'] = landed_names.get(int(msg.landed_state),
                                                msg.landed_state)
            except (AttributeError, TypeError, ValueError):
                pass

        now = time.time()
        if now - last_draw >= 1.0:
            left = t_end - now
            spd = st['ground_speed']
            hdg = st['heading']
            print(f"  [{left:4.1f}s] {st['mode'] or '?'} | "
                  f"{'已解锁' if st['armed'] else ('未解锁' if st['armed'] is False else '状态?')} | "
                  f"高 {_fmt_optional(st['rel_alt'], 'm')} | "
                  f"地速 {_fmt_optional(spd, 'm/s')} | "
                  f"航向 {_fmt_optional(hdg, '°', 0)}")
            last_draw = now

    if not got['hb']:
        print("  ❌ 未收到飞控心跳/状态消息")
        return False

    fix_name = {0: '无定位', 1: '无GPS', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK浮动', 6: 'RTK固定'}
    sys_names = {
        0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING', 3: 'STANDBY', 4: 'ACTIVE',
        5: 'CRITICAL', 6: 'EMERGENCY', 7: 'POWEROFF', 8: 'FLIGHT_TERMINATION',
    }
    armed_txt = '✅ 已解锁 (ARMED)' if st['armed'] else '🔒 未解锁 (DISARMED)'

    print("\n" + "-"*50)
    print("  🛩 飞行状态 (QGroundControl 风格)")
    print("-"*50)
    print(f"  飞行模式: {st['mode']}    {armed_txt}")
    print(f"  系统状态: {sys_names.get(st['sys_status'], st['sys_status'])}"
          f"    base_mode: {st['base_mode']}")
    fix = st['fix']
    print(f"  GPS: {fix_name.get(fix, fix)}  卫星: {_fmt_optional(st['sats'], '颗', 0)}"
          f"  HDOP: {_fmt_optional(st['hdop'], '', 1)}")
    if got['pos']:
        print(f"  位置(经纬): {st['lat']:.6f}, {st['lon']:.6f}")
        print(f"  相对高度: {_fmt_optional(st['rel_alt'], ' m', 2)}"
              f"    绝对高度(AMSL): {_fmt_optional(st['abs_alt'], ' m', 2)}")
        print(f"  速度 NED: N={_fmt_optional(st['vx'], ' m/s', 2)}  "
              f"E={_fmt_optional(st['vy'], ' m/s', 2)}  "
              f"D={_fmt_optional(st['vz'], ' m/s', 2)}")
        print(f"  地速航向: {_fmt_optional(st['ground_speed'], ' m/s', 1)}  "
              f"航向 {_fmt_optional(st['heading'], '°', 1)}")
    print(f"  空速: {_fmt_optional(st['air_speed'], ' m/s', 1)}  "
          f"油门 {_fmt_optional(st['throttle'], '%', 0)}  "
          f"爬升 {_fmt_optional(st['climb'], ' m/s', 1)}")
    print(f"  姿态 RPY: roll {_fmt_optional(st['roll'], '°', 1)}  "
          f"pitch {_fmt_optional(st['pitch'], '°', 1)}  "
          f"yaw {_fmt_optional(st['yaw'], '°', 1)}")
    bat_parts = [f"电压 {_fmt_optional(st['voltage'], ' V', 2)}"]
    if st['current'] is not None:
        bat_parts.append(f"电流 {st['current']:.1f} A")
    if st['remaining'] is not None:
        bat_parts.append(f"电量 {st['remaining']}%")
    print(f"  电池: {'  '.join(bat_parts)}")
    print(f"  CPU负载: {_fmt_optional(st['load'], '%', 1)}")
    if st['landed'] is not None:
        print(f"  着陆状态: {st['landed']} (EXTENDED_SYS_STATE; "
              f"PX4 不发 EKF_STATUS_REPORT)")
    print("-"*50)
    print("  ✅ 状态获取完成")
    return True


def test_13_battery(master):
    """测试13：电池状态（SYS_STATUS / BATTERY_STATUS，同 QGC）"""
    print("\n" + "="*50)
    print(f"🔋 测试13：电池状态（采集 {BATTERY_LISTEN:.0f}s）")
    print("="*50)

    bat = {
        'voltage': None, 'current': None, 'remaining': None,
        'load': None,
        'id': None, 'func': None, 'type': None, 'temp': None,
        'cells': None, 'current_consumed': None, 'energy_consumed': None,
    }
    got = False

    func_names = {0: '未知', 1: '所有', 2: '仅稳定', 3: '仅起飞'}
    type_names = {0: '未知', 1: 'LiPo', 2: 'LiFe', 3: 'LiOn', 4: 'NiMH', 5: 'NiCd', 6: 'Pb'}

    t_end = time.time() + BATTERY_LISTEN
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'SYS_STATUS':
            bat['voltage'] = msg.voltage_battery / 1000.0
            bat['current'] = (msg.current_battery / 100.0
                              if msg.current_battery >= 0 else None)
            bat['remaining'] = (msg.battery_remaining
                                if msg.battery_remaining >= 0 else None)
            bat['load'] = msg.load / 10.0
            got = True
        elif mtype == 'BATTERY_STATUS':
            voltages = [v for v in msg.voltages if v != 65535]
            if voltages:
                bat['cells'] = voltages
                bat['voltage'] = sum(voltages) / 1000.0
            bat['id'] = msg.id
            bat['func'] = func_names.get(msg.battery_function, msg.battery_function)
            # pymavlink 字段名是 type, 不是 battery_type (部分版本无该字段名)
            raw_type = getattr(msg, 'type', None)
            if raw_type is None:
                raw_type = getattr(msg, 'battery_type', '未知')
            bat['type'] = type_names.get(raw_type, raw_type)
            # temperature 单位 c°C (0.01°C), 无效值 INT16_MAX
            temp = getattr(msg, 'temperature', 32767)
            if temp != 32767:
                bat['temp'] = temp / 100.0
            cur = getattr(msg, 'current_battery', -1)
            if cur is not None and cur >= 0:
                bat['current'] = cur / 100.0
            cc = getattr(msg, 'current_consumed', -1)
            if cc is not None and cc >= 0:
                bat['current_consumed'] = cc
            ec = getattr(msg, 'energy_consumed', -1)
            if ec is not None and ec >= 0:
                bat['energy_consumed'] = ec
            br = getattr(msg, 'battery_remaining', -1)
            if br is not None and br >= 0:
                bat['remaining'] = br
            got = True

    if not got:
        print("  ❌ 未收到 SYS_STATUS / BATTERY_STATUS，飞控可能未转发电池信息")
        print("     （可检查固件是否配置电池监测 BAT_* 参数 / 电池 Monitor）")
        return False

    print("\n" + "-"*50)
    print("  🔋 电池状态 (QGroundControl 风格)")
    print("-"*50)
    print(f"  电压: {_fmt_optional(bat['voltage'], ' V', 2)}")
    if bat['current'] is not None:
        print(f"  电流: {bat['current']:.2f} A")
    else:
        print("  电流: -- (固件未上报或未接电流计)")
    if bat['remaining'] is not None:
        bar_len = 20
        filled = int(round(bat['remaining'] / 100.0 * bar_len))
        bar = '█' * filled + '░' * (bar_len - filled)
        print(f"  电量: [{bar}] {bat['remaining']}%")
    else:
        print("  电量: -- (未知)")
    if bat['temp'] is not None:
        print(f"  温度: {bat['temp']:.1f} °C")
    if bat['cells']:
        cells = bat['cells']
        cell_avg = sum(cells) / len(cells) / 1000.0
        print(f"  电芯: {len(cells)}S  平均 {cell_avg:.3f} V/芯")
        print("        " + "  ".join(f"{v/1000.0:.3f}V" for v in cells))
    if bat['id'] is not None:
        print(f"  编号: {bat['id']}  功能: {bat['func']}  类型: {bat['type']}")
    if bat['current_consumed'] is not None:
        print(f"  已消耗容量: {bat['current_consumed']} mAh")
    if bat['energy_consumed'] is not None:
        print(f"  已消耗能量: {bat['energy_consumed']} mWh")
    if bat['load'] is not None:
        print(f"  系统负载: {bat['load']:.1f}%")
    print("-"*50)
    print("  ✅ 电池状态获取完成")
    return True


def test_14_bounce(master):
    """测试14：低空起降(解锁+起飞集成 → 悬停 → 降落 → 上锁)。
    起飞地点 = 当前 GPS 位置原地垂直起降，不飞往其它坐标"""
    print("\n" + "="*50)
    print(f"🪁 测试14：低空起降 {BOUNCE_ALT}m 悬停 {BOUNCE_HOVER_SEC:.0f}s → 降落")
    print("="*50)
    print("  流程: 等GPS → (提示COM_DISARM_PRFLT) → 解锁+立即起飞 → 悬停 → LAND → 上锁")
    print("  ⚠️ 原地垂直起降，螺旋桨区域保持清空，遥控器随时可接管")

    # GPS 未就绪不再直接失败 — arm_vehicle 会经确认降级 ALTCTL
    # (气压定高, 无需GPS) + NAV_TAKEOFF, 悬停/降落同样只需气压计
    if not wait_gps_lock(master, timeout=10):
        print("  ⚠️ GPS 未就绪: 将走 ALTCTL 降级起降"
              "(无GPS定点/返航, 悬停靠气压定高)")

    print("  [1/4] 解锁 + 起飞（一体执行）...")
    if not arm_and_takeoff(master, BOUNCE_ALT, mode='TAKEOFF',
                           arm_timeout=10, takeoff_timeout=BOUNCE_TAKEOFF_TIMEOUT,
                           set_delay=True):
        print("  ❌ 解锁/起飞失败")
        if not master.motors_armed():
            print("     (未保持解锁状态)")
        else:
            set_flight_mode(master, 'LAND', timeout=5, quiet=True)
            print("     已切 LAND 保安全")
        return False

    print(f"  [2/4] 悬停 {BOUNCE_HOVER_SEC:.0f}s ...")
    t_end = time.time() + BOUNCE_HOVER_SEC
    while time.time() < t_end:
        left = t_end - time.time()
        m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=0.2)
        alt = (m.relative_alt / 1000.0) if m else get_current_alt(master)
        print(f"  悬停中 剩余 {max(left, 0):.1f}s | 高度 {alt:.2f}m", end='\r')
        time.sleep(0.2)
    print("\n  ✅ 悬停完成")

    print("  [3/4] 切 LAND 降落 (飞控闭环缓降, 着陆自动上锁) ...")
    send_zero_velocity(master, count=5)
    time.sleep(0.5)
    if not set_flight_mode(master, 'LAND', timeout=6, quiet=True):
        print("  ❌ 切换 LAND 模式失败")
        return False
    landed = land_soft_final(master, phase0_timeout=BOUNCE_LAND_TIMEOUT)
    if not landed:
        print(f"\n  ⚠️ 降落超时，当前高度 {get_current_alt(master):.2f}m")

    time.sleep(1.5)
    print("  [4/4] 上锁 ...")
    if not disarm_vehicle(master, force=False, timeout=8):
        print("  ❌ 上锁失败，请用遥控器手动上锁")
        return False

    if not landed:
        return False
    print("  ✅ 低空起降测试完成")
    return True


# ============================================================
#  ULog 日志下载 (LOG_REQUEST_LIST / LOG_ENTRY / LOG_DATA)
# ============================================================

def fetch_log_list(master, timeout=12.0, gap=1.5):
    """请求日志清单. 返回 ({id: size}, last_log_num, num_logs);
    超时/无日志返回 ({}, -1, 0)"""
    master.mav.log_request_list_send(
        master.target_system, master.target_component, 0, 0xFFFF)
    logs = {}
    last_log = -1
    num_logs = 0
    t_end = time.time() + timeout
    t_rx = 0.0
    while time.time() < t_end:
        msg = master.recv_match(type='LOG_ENTRY', blocking=True,
                                timeout=min(1.0, t_end - time.time()))
        if msg is None:
            if logs and time.time() - t_rx > gap:
                break
            continue
        t_rx = time.time()
        num_logs = int(msg.num_logs)
        last_log = int(msg.last_log_num)
        logs[int(msg.id)] = int(msg.size)
        if num_logs > 0 and len(logs) >= num_logs:
            break
    return logs, last_log, num_logs


def fetch_one_log(master, log_id, size, out_path, timeout=180.0):
    """下载一条日志写入 out_path (完整校验后才落盘).
    返回 (ok, got_bytes). 断流自动从第一个缺口重发请求"""
    ts = master.target_system
    tc = master.target_component
    received = {}
    got = 0
    master.mav.log_request_data_send(ts, tc, log_id, 0, 0xFFFFFFFF)
    t_end = time.time() + timeout
    t_rx = time.time()
    next_mb = 64 * 1024
    stall = 0
    while got < size and time.time() < t_end:
        if time.time() - t_rx > 3.0:
            stall += 1
            if stall > 5:
                break
            hole = 0
            while hole in received:
                hole += len(received[hole])
            print('    断流, 从缺口 %d B 重发请求 ...' % hole)
            master.mav.log_request_data_send(ts, tc, log_id, hole,
                                             0xFFFFFFFF)
            t_rx = time.time()
            continue
        msg = master.recv_match(type='LOG_DATA', blocking=True,
                                timeout=min(1.0, t_end - time.time()))
        if msg is None or int(msg.id) != log_id:
            continue
        t_rx = time.time()
        stall = 0
        ofs = int(msg.ofs)
        buf = bytes(msg.data[:int(msg.count)])
        if not buf or ofs in received:
            continue
        received[ofs] = buf
        got += len(buf)
        if got >= next_mb:
            print('    进度 %d / %d KB' % (got // 1024, size // 1024))
            next_mb += 64 * 1024
    if got < size:
        return False, got
    out = bytearray()
    ofs = 0
    while ofs < size:
        b = received.get(ofs)
        if not b:
            return False, got          # 缺口, 无法组装
        out += b
        ofs += len(b)
    with open(out_path, 'wb') as f:
        f.write(out)
    return True, len(out)


def download_flash_logs(master, only_last=True, out_dir='flight_logs'):
    """从飞控下载 ULog 日志到 out_dir/ulog_<id>_<时间>.ulg.
    only_last=True 只下最后一次飞行日志, False 下载全部. 返回成功路径列表"""
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    print('请求飞控日志清单 ...')
    logs, last_log, num_logs = fetch_log_list(master)
    if not logs:
        print('  ⚠️ 未取到日志清单(飞控无日志或链路超时)')
        return []
    print('  飞控日志 %d 条 (id %d~%d)'
          % (len(logs), min(logs), max(logs)))
    if only_last:
        ids = [last_log] if last_log in logs else [max(logs)]
    else:
        ids = sorted(logs)
    saved = []
    for lid in ids:
        path = os.path.join(
            out_dir, 'ulog_%03d_%s.ulg'
            % (lid, time.strftime('%Y%m%d_%H%M%S')))
        size = logs[lid]
        print('下载日志 #%d (%.1f KB) ...' % (lid, size / 1024.0))
        ok, got = fetch_one_log(master, lid, size, path,
                                timeout=max(180.0, size / 4096.0))
        if ok:
            saved.append(path)
            print('  ✅ 已保存 %s (%.1f KB)' % (path, got / 1024.0))
        else:
            print('  ❌ 日志 #%d 下载失败 (%d/%d B)' % (lid, got, size))
            if os.path.exists(path):
                try:
                    os.remove(path)
                except OSError:
                    pass
    return saved


# ============================================================
#  测试调度器
# ============================================================

TESTS = {
    '1':  ('通信连接',     test_01_connection),
    '2':  ('解锁电机',     test_02_arm),
    '3':  ('模式切换',     test_03_mode_switch),
    '4':  ('起飞',         test_04_takeoff),
    '5':  ('前进速度',     test_05_velocity_forward),
    '6':  ('侧向速度',     test_06_velocity_side),
    '7':  ('垂直上升',     test_07_velocity_up),
    '8':  ('悬停稳定',     test_08_hover),
    '9':  ('自动降落',     test_09_land),
    '10': ('锁定电机',     test_10_disarm),
    '11': ('参数读写校验',  test_11_param_rw),
    '12': ('飞机状态',     test_12_status),
    '13': ('电池状态',     test_13_battery),
    '14': ('低空起降悬停',  test_14_bounce),
}


def run_single_test(master, test_id):
    """运行单项测试"""
    if test_id in TESTS:
        name, func = TESTS[test_id]
        print(f"\n{'#'*50}")
        print(f"# 运行测试 {test_id}: {name}")
        print(f"{'#'*50}")
        try:
            result = func(master)
            print(f"\n  {'✅ 通过' if result else '❌ 失败'}: {name}")
            return result
        except Exception as e:
            print(f"\n  ❌ 异常: {e}")
            return False
    else:
        print(f"  ❌ 未知测试编号: {test_id}")
        return False


def run_all_tests(master):
    """运行全部测试（按顺序）"""
    results = {}
    for test_id, (name, func) in TESTS.items():
        try:
            result = func(master)
            results[name] = result
        except Exception as e:
            print(f"\n  ❌ {name} 异常: {e}")
            results[name] = False

    # 打印汇总报告
    print("\n" + "="*50)
    print("📋 测试报告汇总")
    print("="*50)
    for name, result in results.items():
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {status}  {name}")

    passed = sum(1 for r in results.values() if r)
    total = len(results)
    print(f"\n  总计: {passed}/{total} 通过")


def show_menu():
    """显示菜单"""
    print("\n" + "="*50)
    print("🚁 PX4 功能测试工具")
    print("="*50)
    print("  可用测试项：")
    for test_id, (name, _) in TESTS.items():
        print(f"    {test_id:>2}. {name}")
    print("    a. 运行全部测试")
    print("    q. 退出")
    print("-"*50)


# ============================================================
#  主程序
# ============================================================

if __name__ == '__main__':
    master = connect()

    while True:
        show_menu()
        choice = input("请选择测试项: ").strip().lower()

        if choice == 'q':
            print("👋 退出")
            break
        elif choice == 'a':
            run_all_tests(master)
        elif choice in TESTS:
            run_single_test(master, choice)
        else:
            print("  ❌ 无效选择，请重新输入")
