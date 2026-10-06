'''
一个完整的功能测试框架，把连接、解锁、起飞、速度控制、降落、锁定、PID自动调参、飞机状态、电池状态等每个功能拆成独立测试项，可以单独运行，也可以一键跑全部
'''
from pymavlink import mavutil
import time
import math
import sys
import os

# ============ 配置参数 ============
CONNECTION_STRING = 'COM25'   # 按实际修改
BAUD_RATE = 115200
CONTROL_RATE = 10                    # 速度指令发送频率 Hz
TAKEOFF_ALT = 5.0                    # 默认起飞高度（米）
TAKEOFF_CLIMB_THR = 1800             # 起飞爬升油门(覆盖帧ch3): 1500=悬停,
                                     # ±100死区内=零爬升需求(1600=死区边缘),
                                     # 默认1800≈1.25m/s需求; UI可调 1600~2000
TEST_SPEED = 2.0                     # 测试速度（m/s）
TEST_DURATION = 5                    # 每项测试持续时间（秒）

# ============ 发射箱参数写入控制 ============
# v1.24.5: 用户可通过按钮一次性写入发射箱参数，后续操作自动跳过
LAUNCH_PARAMS_WRITTEN = False        # 标记是否已在本会话写入过发射箱参数
DISARM_DELAY_WRITTEN = False         # 标记是否已写入 DISARM_DELAY

def reset_launch_params_flag():
    """重置发射箱参数写入标记（连接/断开时调用）"""
    global LAUNCH_PARAMS_WRITTEN, DISARM_DELAY_WRITTEN
    LAUNCH_PARAMS_WRITTEN = False
    DISARM_DELAY_WRITTEN = False

# ============ 解锁后自动上锁 / 起飞集成 ============
# ArduCopter: 解锁后若一直低油门未起飞, DISARM_DELAY 秒后自动 DISARM
# 这就是"测试2解锁→稍后再飞已上锁"、"RC解锁不推油门自己停"的原因
DISARM_DELAY_SEC = 60                # 起飞前写入的 DISARM_DELAY(秒); 0=不改飞控参数
ARM_TO_TAKEOFF_MAX = 8               # 集成起飞时: 解锁成功后最多几秒内必须发出 NAV_TAKEOFF

# ============ 发射箱纯软件飞行（v1.20 解锁 / v1.21 参数预设） ============
LAUNCH_PRESET = 1                    # 1=解锁前自动写"发射箱参数组"(禁飞控自动返航/降落)
# (参数名, 期望值, 中文标签): 读回不符才写, 命中跳过; 单项失败仅警告继续
LAUNCH_PARAMS = (
    ('FS_THR_ENABLE', 0.0, '无遥控器RC失效保护'),
    ('FS_GCS_ENABLE', 0.0, 'GCS断链不动作'),
    ('FS_DR_ENABLE', 0.0, 'GPS失联不返航(惯导继续)'),
    ('FS_DR_TIMEOUT', 0.0, '惯导超时计时关闭'),
    ('FS_EKF_ACTION', 0.0, 'EKF失效仅报告不降落'),
    ('FENCE_ENABLE', 0.0, '飞控围栏不自动动作'),
    ('BRD_SAFETYENABLE', 0.0, '无安全开关(发射箱)'),
    ('BATT_FS_LOW_ACT', 0.0, '低电仅告警'),
    ('BATT_FS_CRT_ACT', 0.0, '电量危急仅告警'),
    ('RC_OVERRIDE_TIME', -1.0, 'RC覆盖永不过期(无接收机)'),
    ('RC_OPTIONS', 0.0, '忽略GCS覆盖位关闭'),
)
ARM_RETRY = 2                        # 解锁被拒且因GPS/EKF未就绪时自动重试次数
ARM_RETRY_GPS_WAIT = 30              # 每次重试前等待GPS/EKF就绪的秒数
# v1.29.3: 解锁前预检门禁 — 位置估计未就绪时先等(带进度), 到时仍未就绪则
# 不发 ARM (避免必现的 PreArm: Need Position Estimate 弹窗)
ARM_PREPARE_WAIT = 60
# 需要水平位置估计才能解锁的模式 (不满足则跳过预检门禁, 如 ALT_HOLD/STABILIZE)
POSITION_MODES = ('GUIDED', 'AUTO', 'LOITER', 'RTL', 'CIRCLE', 'SMARTRTL',
                  'LAND', 'BRAKE', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE')

ARM_RETRY_KEYS = ('position estimate', 'need 3d fix', 'gps', 'ekf',
                  'home', 'hdop', 'origin', 'waiting for')
# v1.24: EKF_STATUS_FLAGS 官方位定义 (pymavlink ardupilotmega dialect):
#  1=ATTITUDE 2=VELOCITY_HORIZ 4=VELOCITY_VERT 8=POS_HORIZ_REL 16=POS_HORIZ_ABS
#  32=POS_VERT_ABS 64=POS_VERT_AGL 128=CONST_POS_MODE(恒定位置=无有效位置)
# 旧 0x0A 实为"速度水平|水平位置相对", 语义错误; 实机室内 flags=0x00A7:
#  有姿态/速度/垂直位置, 但无水平位置 + CONST_POS_MODE → 正确判为未就绪
EKF_POS_HORIZ_FLAGS = 0x18           # 水平位置: 相对(8) | 绝对(16) 任一
EKF_POS_VERT_FLAGS = 0x60            # 垂直位置: 绝对(32) | 对地(64) 任一
EKF_CONST_POS_FLAG = 0x80            # 恒定位置模式 → 位置估计无效
# v1.22: 重试就绪后不自动解锁 — 醒目提示+人工确认 (防测试人员贴近时机身突解锁)
ARM_CONFIRM_TIMEOUT = 120            # 确认超时秒数 (超时未确认则取消解锁)

# ============ 软遥控配置（v1.23 屏幕摇杆/键盘 → RC 覆盖） ============
# 中立帧两种: 无接收机 + RC_OVERRIDE_TIME=-1 下, 停发不回退遥控器,
# "释放"必须发中立帧收尾; 全0=交还不存在的接收机=无效PWM(空中会掉高)
RC_NEUTRAL_GROUND = (1500, 1500, 1000, 1500)  # 上锁收尾/解锁压油门(油门低)
RC_NEUTRAL_AIR = (1500, 1500, 1500, 1500)     # 空中收尾(油门=悬停,不掉高)
# v1.24: 收尾是否"交还遥控器" — 1=发4通道全0帧(MAVLink约定0=释放该通道),
# GCS覆盖立即撤销, 机上接收机摇杆立刻生效(物理遥控器随时接管);
# 0=v1.23中立帧(纯软件发射箱无接收机时: 全0=无效PWM空中掉高)
RC_HANDOVER = 1
SOFT_RC_RP_US = 250                   # 横滚/俯仰限幅(μs): 1500±250=1250~1750
SOFT_RC_YAW_US = 300                  # 偏航限幅(μs): 1500±300=1200~1800
SOFT_RC_RATE_HZ = 20                  # 启用期间发送频率 (Hz)
# 仅手动模式下杆值有效 (GUIDED/AUTO 下摇杆被固件忽略, 不发送)
SOFT_RC_MANUAL_MODES = ('STABILIZE', 'ACRO', 'ALT_HOLD', 'LOITER',
                        'POSHOLD', 'DRIFT', 'SPORT')

# ============ PID 自动调参配置（测试11） ============
AUTOTUNE_ALT = 5.0                   # 调参起飞高度（米），建议 5~10
AUTOTUNE_TIMEOUT = 1800              # 等待调参完成超时（秒）
AUTOTUNE_CLIMB_THR = 1700            # AltHold 爬升油门（1500=悬停）
AUTOTUNE_SETTLE_THR = 1500           # 到位后稳定油门
AUTOTUNE_MIN_SATS = 6                # 调参前最少卫星数
AUTOTUNE_LOITER_SETTLE = 3           # 进 AUTOTUNE 前 LOITER 稳定秒数

# ============ 软着陆配置（LAND/RTL 最后1m 油门微调） ============
SOFT_LAND_ALT = 1.0                 # ≤此高度接管油门微调（米）
SOFT_LAND_RATE = 0.25               # 接管后目标缓降速度（米/秒）
SOFT_LAND_THR_START = 1480          # 微调起始油门（1500≈悬停）
SOFT_LAND_THR_MIN = 1300            # 油门下限（防止掉高过快）
SOFT_LAND_THR_MAX = 1520            # 油门上限（略高于悬停，仅兜底减速）
SOFT_LAND_GROUND_ALT = 0.15         # 触地判定高度（米）
SOFT_LAND_SETTLE_T = 45             # 缓降阶段总超时（秒）

# ============ 状态 / 电池显示配置（测试12/13） ============
STATUS_LISTEN = 3.0                  # 飞机状态采集时长（秒）
BATTERY_LISTEN = 3.0                 # 电池信息采集时长（秒）

# ============ 低空起降悬停配置（测试14） ============
BOUNCE_ALT = 2.0                     # 起飞目标高度（米）
BOUNCE_HOVER_SEC = 2.0               # 到位后悬停秒数
BOUNCE_TAKEOFF_TIMEOUT = 30          # 起飞到位超时（秒）
BOUNCE_LAND_TIMEOUT = 60             # 降落到位超时（秒）
BOUNCE_LAND_ALT = 0.3                # 判定已着陆的相对高度（米）


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
    return master


def drain_rx(master):
    """清空接收缓冲, 避免读到旧消息干扰判断"""
    while master.recv_match(blocking=False):
        pass


def get_rc_throttle(master):
    """读取遥控器油门通道(RC ch3), 返回 PWM 或 None"""
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
    """在 duration 秒内收取并打印飞控 STATUSTEXT(PreArm 等), 返回最后一条.
    v1.29.3: sink 传入列表则把每条都 append 进去(供累积判据, 见 arm_vehicle)"""
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
    """PARAM_SET 写参数, 等 PARAM_VALUE 回读确认. 返回 True/False"""
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
            # 有的固件回读值可能已是 int 编码, 接近即算成功
            if abs(float(msg.param_value) - float(value)) < 0.01:
                print(f"  参数 {name} = {float(msg.param_value):g}")
                return True
            # v1.24: 回读不符不轻信, 重新读一次复核, 杜绝"同名即真"的假成功
            # (如 RC_OVERRIDE_TIME 被固件钳制 -1→0 时旧逻辑仍返回 True)
            print(f"  ⚠️ {name} 回读={float(msg.param_value):g} "
                  f"(请求 {value:g}), 重新读确认...")
            cur = get_param(master, name, timeout=timeout)
            if cur is not None and abs(cur - value) < 0.01:
                print(f"  参数 {name} = {cur:g} (复核一致)")
                return True
            print(f"  ❌ {name} 写入未生效(固件钳制/拒绝, 现值={cur})")
            return False
    return False


def ensure_disarm_delay(master):
    """起飞相关测试前延长 DISARM_DELAY, 避免解锁后未及时起飞被自动上锁.
    DISARM_DELAY_SEC=0 则跳过. v1.24.5: 用户已手动写入则跳过.
    返回 True(已设置或跳过)/False(设置失败仍继续)"""
    global DISARM_DELAY_WRITTEN
    if not DISARM_DELAY_SEC or DISARM_DELAY_SEC <= 0:
        return True
    if DISARM_DELAY_WRITTEN:
        print("  ℹ️ DISARM_DELAY 已手动写入，跳过")
        return True
    print(f"  设置 DISARM_DELAY={DISARM_DELAY_SEC}s (防止解锁后未起飞自动上锁)...")
    if set_param(master, 'DISARM_DELAY', float(DISARM_DELAY_SEC), timeout=2.5):
        return True
    print("  ⚠️ 写入 DISARM_DELAY 失败(旧固件可能参数名不同), 仍继续; "
          "若仍自动上锁请在 Mission Planner 全部参数中改 DISARM_DELAY")
    return False


def write_disarm_delay(master):
    """强制写入 DISARM_DELAY（按钮调用）：若已手动写入则提示跳过"""
    global DISARM_DELAY_WRITTEN
    if not DISARM_DELAY_SEC or DISARM_DELAY_SEC <= 0:
        return True
    if DISARM_DELAY_WRITTEN:
        print("  ℹ️ DISARM_DELAY 已手动写入，跳过")
        return True
    print(f"  🔧 手动设置 DISARM_DELAY={DISARM_DELAY_SEC}s...")
    if set_param(master, 'DISARM_DELAY', float(DISARM_DELAY_SEC), timeout=2.5):
        DISARM_DELAY_WRITTEN = True
        print("  ✅ DISARM_DELAY 手动写入完成")
        return True
    print("  ⚠️ 写入 DISARM_DELAY 失败")
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
    严格回读复核 (v1.24: 回读不符=失败, 不再同名即真).
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
    """发射箱参数预设核心逻辑: 只写入值不符的参数"""
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
        print("  ⚠️ 部分参数未确认, 若仍自动返航/降落请在 Mission Planner "
              "全部参数中手动将上表参数设为 0")
    try:
        gcs_id = get_param(master, 'MAV_GCS_SYSID', timeout=1.5)
        src = int(getattr(master, 'source_system', 255))
        if gcs_id is not None and int(gcs_id) != src:
            print(f"  ⚠️ MAV_GCS_SYSID={int(gcs_id)} 与本端 source_system={src} "
                  f"不一致, RC覆盖/软遥控可能被飞控忽略")
    except Exception:
        pass
    return ok


def ensure_launch_params(master):
    """发射箱参数预设 - 若用户已手动写入则跳过 (v1.24.5)"""
    global LAUNCH_PARAMS_WRITTEN
    if LAUNCH_PARAMS_WRITTEN:
        print("  ℹ️ 发射箱参数已手动写入，跳过")
        return True
    return _do_launch_params(master)


def write_launch_params(master):
    """强制写入发射箱参数（按钮调用）：若已手动写入则提示跳过"""
    global LAUNCH_PARAMS_WRITTEN
    if LAUNCH_PARAMS_WRITTEN:
        print("  ℹ️ 发射箱参数已手动写入，跳过")
        return True
    print("  🔧 手动写入发射箱参数...")
    result = _do_launch_params(master)
    if result:
        LAUNCH_PARAMS_WRITTEN = True
        print("  ✅ 发射箱参数手动写入完成")
    return result


# GUI 注入的确认钩子: callable(reason) -> bool; None 时走终端 input 确认
ARM_CONFIRM_HOOK = None
# v1.24: 最近一次 confirm_arm 的就绪状态, GUI 弹窗标题据此切换
# (🚨检测一切正常 / ⚠️位置估计未就绪), 避免未就绪时仍谎报"一切正常"
ARM_CONFIRM_ALL_READY = True


def confirm_arm(reason='', all_ready=True):
    """解锁前(等待就绪后)醒目提示并等待操作员确认 (v1.22 安全机制):
    重试路径可能在点击后数十秒才满足条件, 为防此时人员贴近飞机突然自解锁,
    必须由现场人员看到提示、确认安全后才继续 ARM.
    v1.24: all_ready=False 表示位置估计未就绪等非全绿状态, 横幅改为警示文案;
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
    """解锁成功后打印 SERVO_OUTPUT_RAW 电机1~4指令PWM (v1.24 诊断).
    区分"FC没给油门(受控停转, 正常)" 与 "FC给了油门但电机不转(硬件)";
    固件未流式发送该消息时仅提示跳过, 不影响解锁结果"""
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 4, 1)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            36, 0, 0, 0, 0, 0, 0)        # 36=SERVO_OUTPUT_RAW
    except Exception:
        pass
    t_end = time.time() + timeout
    while time.time() < t_end:
        m = master.recv_match(type='SERVO_OUTPUT_RAW',
                              blocking=True, timeout=0.5)
        if m is None or m.get_type() != 'SERVO_OUTPUT_RAW':
            continue
        print(f"  电机指令 PWM: M1={m.servo1_raw} M2={m.servo2_raw} "
              f"M3={m.servo3_raw} M4={m.servo4_raw} "
              "(≈1000=受控停转, GUIDED解锁后属正常)")
        return
    print("  (固件未发送 SERVO_OUTPUT_RAW, 跳过电机输出检查)")


def arm_vehicle(master, mode='GUIDED', timeout=10, retries=ARM_RETRY,
                fallback_mode='ALT_HOLD', gate=True):
    """在指定可解锁模式下解锁电机.
    流程: 切模式 -> 发射箱参数预设(禁自动返航/降落, v1.21) -> RC覆盖强制
    油门最低(解决脚本解锁时摇杆不在低位被拒) -> 发 ARM -> 校验 ACK +
    心跳 ARMED; 被拒且原因为 GPS/EKF 未就绪时等就绪后自动重试 retries 次.
    v1.24.3: 始终无法就绪时经现场确认降级 fallback_mode(默认 ALT_HOLD —
    气压定高不依赖位置估计, 且可油门起飞, 比 STABILIZE 更安全; 起飞流程
    会以油门爬升替代 NAV_TAKEOFF). v1.29.4: 该降级出口对"预检门禁超时"
    与"ARM 被拒"两条路径统一生效. 返回 True/False"""
    mode = (mode or 'GUIDED').upper()
    mode_map = master.mode_mapping() or {}
    if mode not in mode_map:
        print(f"  ❌ 未知模式 {mode}, 无法解锁")
        return False

    # v1.29.3: 预检门禁 — 需位置的模式先等位置估计就绪, 未就绪不发 ARM.
    # 消灭"刚开机就点解锁必现 PreArm: Need Position Estimate"的体验:
    # 该检查是飞控 mandatory 项(跳不掉), GUIDED 也离不开位置估计.
    # gate=False 仅供内部递归重试/降级路径跳过, 避免重复等待.
    if gate and mode in POSITION_MODES and not master.motors_armed():
        ready, why = wait_position_ready(master, timeout=ARM_PREPARE_WAIT)
        if not ready:
            print(f"  ❌ 预检未通过({why}), 本次不发送 ARM "
                  f"({mode} 解锁必被 PreArm 拒绝)")
            # v1.29.4: 门禁失败不再直接放弃 — 走与"ARM 被拒"相同的降级出口.
            # 旧版在这一步 return False, 而 v1.24.3 的 ALT_HOLD 降级分支要
            # "ARM 命令被飞控拒绝"才触发: 室内 GPS 多径 / EKF 恒定位时位置
            # 估计永不到位, 门禁永远先拦在 ARM 之前 → 降级分支永不触发,
            # 起飞(按高度)/测试14 只能干等 60s 后失败(实机反馈的问题).
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
            print("    提示: 室外等 GPS 3D 定位 + EKF 收敛(约 20~60s)再解锁;")
            print("          急用请点 [定高解锁] 走 ALT_HOLD(不依赖位置估计)")
            return False

    print(f"  切换 {mode} 模式(可解锁)...")
    master.set_mode_apm(mode)
    time.sleep(1.5)
    drain_rx(master)
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if hb and hb.custom_mode != mode_map[mode]:
        cur = next((n for n, i in mode_map.items() if i == hb.custom_mode), hb.custom_mode)
        print(f"  ❌ 当前模式仍为 {cur}, 非 {mode}, 解锁会被飞控拒绝")
        return False

    if master.motors_armed():
        print("  ℹ️ 电机已经是解锁状态")
        return True

    ensure_launch_params(master)

    thr = get_rc_throttle(master)
    if thr is not None:
        print(f"  当前 RC 油门 PWM: {thr}"
              + (" (偏高, 将用覆盖强制拉低)" if thr > 1200 else ""))
    gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.5)
    if gps:
        print(f"  GPS: fix={gps.fix_type} 卫星={gps.satellites_visible}")

    # 解锁期间 RC 覆盖: 油门 1000=最低, 横滚/俯仰/偏航居中
    # 遥控器解锁时人手会压低油门; 脚本解锁时摇杆常在中位 -> PreArm: Throttle high
    print("  RC 覆盖油门=1000 并发送 ARM 命令...")
    released = False
    try:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            1500, 1500, 1000, 1500, 0, 0, 0, 0)
        time.sleep(0.4)

        drain_rx(master)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)

        t_end = time.time() + timeout
        last_text = ""
        texts = []          # v1.29.3: 本次解锁收到的全部 STATUSTEXT (累积判据)
        while time.time() < t_end:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                if master.motors_armed():
                    break
                # 保持覆盖有效(部分飞控 RC_OVERRIDE 会超时)
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500, 1500, 1000, 1500, 0, 0, 0, 0)
                continue
            mtype = msg.get_type()
            if mtype == 'STATUSTEXT':
                text = msg.text
                if isinstance(text, (bytes, bytearray)):
                    text = text.decode('utf-8', 'replace')
                text = str(text)
                last_text = text
                texts.append(text)      # v1.29.3: 累积(PreArm 常被后续文本顶掉)
                print(f"    [飞控] {text}")
            elif mtype == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("  ARM 命令已接受, 等待 ARMED 状态...")
                else:
                    result_names = {
                        1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
                        4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
                    }
                    rn = result_names.get(msg.result, str(msg.result))
                    print(f"  ❌ 解锁被拒绝: COMMAND_ACK={rn}")
                    # 飞控常在 ACK 之后才发 PreArm 文本, 多等一会再收
                    more = collect_status_text(master, duration=2.0,
                                               sink=texts)
                    if more:
                        last_text = more
                    break
            elif mtype == 'HEARTBEAT' and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                break
        # 无论成败都再收一小段文本
        if not master.motors_armed():
            collect_status_text(master, duration=0.8, sink=texts)
    finally:
        if not released:
            release_rc_override(master)
            released = True
            time.sleep(0.2)

    if master.motors_armed():
        print("  ✅ 电机解锁成功 (ARMED)")
        print_motor_outputs(master)      # v1.24: 打印电机指令PWM作为证据
        return True

    # v1.29.3: 判据改用"本次解锁收到的全部 STATUSTEXT" — 旧版只取最后一条,
    # PreArm 文本常被其后的文本顶掉导致关键词匹配不到、重试根本不触发; 且旧版
    # 要求 rejected 才重试, 超时无 ACK 的同类失败直接放弃. 现改为: 未解锁 +
    # 拒因含关键词 → 进入就绪等待与自动重试.
    joined = ' | '.join(texts)
    reason = (joined or (last_text or '')).lower()
    hit = any(k in reason for k in ARM_RETRY_KEYS)
    if retries > 0 and hit:
        print(f"  ⏳ 拒因与 GPS/EKF 位置估计有关, 等待就绪后自动重试 "
              f"(剩余 {retries} 次)...")
        ready, why = wait_position_ready(master, timeout=ARM_RETRY_GPS_WAIT)
        if ready:
            # v1.22: 就绪后不自动解锁, 须现场确认安全后才继续
            if not confirm_arm(
                    f"位置估计已就绪, 重试解锁(剩余{retries}次)"):
                return False
            return arm_vehicle(master, mode=mode, timeout=timeout,
                               retries=retries - 1, gate=False)
        # v1.24.3: 未就绪(室内常见) → 诚实横幅, 确认后降级 fallback_mode
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
        print("    提示: 到室外/靠窗等 GPS 3D 定位 + EKF 收敛后重试")

    print(f"  ❌ 解锁失败(超时 {timeout}s), 电机未进入 ARMED")
    thr2 = get_rc_throttle(master)
    if thr2 is not None:
        print(f"  释放覆盖后 RC 油门 PWM: {thr2}")
    print("  排查顺序:")
    print("    1) Need Position Estimate: 这是飞控 mandatory 检查(跳不掉), "
          "须 GPS 3D 定位 + EKF 收敛; 解锁前软件会先预检等待(%ds), "
          "超时则不发 ARM" % ARM_PREPARE_WAIT)
    print("       到室外等 20~60s; 仍偏严可把 FS_EKF_THRESH 调到 1.0(Relaxed);")
    print("       急用/室内可点[定高解锁]走 ALT_HOLD(不依赖位置估计)")
    print("    2) 看上方 [飞控] PreArm: 具体原因 (GPS/EKF/罗盘/加速度计/安全开关)")
    print("    3) 无遥控器纯软件控制: 解锁时已自动写 FS_THR_ENABLE=0 关闭"
          "RC失效保护; 若报 Radio failsafe 需确认该参数为 0")
    print("    4) 遥控器开机: 油门摇杆压到最低(约 1000)后再解锁")
    print("    5) Pixhawk 安全开关: 按一下物理按钮; 部分固件无"
          "BRD_SAFETYENABLE 参数属正常(跳过)")
    print("    6) 黄灯 = PreArm 未通过; 修好原因后再跑测试2")
    return False


def send_guided_takeoff(master, alt):
    """GUIDED 下发 NAV_TAKEOFF. 返回是否发出(不等待到位)"""
    drain_rx(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, float(alt))
    return True


def arm_and_takeoff(master, alt, mode='GUIDED', arm_timeout=10,
                    takeoff_timeout=45, set_delay=True):
    """解锁 + 立即起飞集成(测试4/14/起飞按钮): 避免"先测2解锁、再测4起飞时
    已被 DISARM_DELAY 上锁".
    v1.24.3: 按解锁后的实际模式分派起飞方式 —
      GUIDED/AUTO → NAV_TAKEOFF (原路径);
      ALT_HOLD(位置估计未就绪经确认降级) → 油门覆盖爬升(气压定高, 无需GPS),
        旧版无条件发 NAV_TAKEOFF 会被 STABILIZE/ALT_HOLD 忽略导致不解锁;
      其它(如 STABILIZE) → 中止并提示(手动模式不支持自动起飞).
    返回 True/False"""
    if set_delay:
        ensure_disarm_delay(master)

    if not arm_vehicle(master, mode=mode, timeout=arm_timeout,
                       fallback_mode='ALT_HOLD'):
        return False

    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    cur = _mode_name(master, hb.custom_mode) if hb is not None else '?'
    if cur in ('GUIDED', 'AUTO'):
        print(f"  [起飞] 解锁后立即发 NAV_TAKEOFF → {alt}m "
              f"(须在 {ARM_TO_TAKEOFF_MAX}s 内发出, 防自动上锁)...")
        # arm_vehicle 已确认 ARMED; 走快速路径, 不再重复切模式读 GPS
        send_guided_takeoff(master, alt)
        return wait_for_alt(master, alt, tolerance=0.5,
                            timeout=takeoff_timeout)
    if cur == 'ALT_HOLD':
        print(f"  [起飞] 实际模式 ALT_HOLD(位置估计未就绪已降级) → "
              f"油门覆盖爬升至 {alt}m (气压定高, 无需GPS)...")
        ok = takeoff_althold(master, alt, timeout=max(takeoff_timeout, 90))
        if ok:
            print(f"  ✅ 已爬升至 {alt}m, 覆盖已交还(遥控器可直接接管)")
        return ok
    print(f"  ❌ 当前模式 {cur} 不支持自动起飞 (需 {mode} 或 ALT_HOLD)")
    return False


def disarm_vehicle(master, force=False, timeout=5, _fallback=False):
    """上锁电机. force=True 时 param2=21196 强制上锁, 仅地面/异常时使用.
    v1.24: 当前方式被固件拒绝时自动换另一种方式再试一次
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
        if mtype == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
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
        elif mtype == 'HEARTBEAT' and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
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
    v1.24.4. 返回 True=检测到重启开始 / False=拒绝或超时未见心跳中断"""
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


def send_zero_velocity(master, count=20):
    """发送零速度指令让飞机悬停"""
    type_mask = 0b0000110111000111
    for _ in range(count):
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)


def wait_gps_lock(master, min_sats=AUTOTUNE_MIN_SATS, timeout=30):
    """等待 GPS 3D 定位且卫星数足够（调参/Loiter 前置条件）"""
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
    """v1.24: 按官方位定义判断 EKF 位置估计就绪 — 水平(相对8|绝对16)+
    垂直(绝对32|对地64)均有效, 且不在恒定位置模式(128)"""
    f = int(flags)
    return bool(f & EKF_POS_HORIZ_FLAGS) and bool(f & EKF_POS_VERT_FLAGS) \
        and not (f & EKF_CONST_POS_FLAG)


def wait_ekf_position(master, timeout=ARM_RETRY_GPS_WAIT):
    """等待 EKF 水平+垂直位置估计就绪 (EKF_STATUS_REPORT flags, v1.21/v1.24 修正).
    GPS 锁星后 EKF 设 origin/收敛仍需数秒~数十秒, 此前 ARM 报
    'Need Position Estimate'. 固件未流式发送该消息时末尾 5s 放行(兼容).
    返回 True(就绪/放行) False(超时未就绪)"""
    print("⏳ 等待 EKF 位置估计就绪...")
    t_end = time.time() + timeout
    grace_at = t_end - 5.0               # 最后5秒仍无该消息 -> 按旧固件放行
    seen = False
    while time.time() < t_end:
        m = master.recv_match(type='EKF_STATUS_REPORT',
                              blocking=True, timeout=1)
        if m is None:
            if not seen and time.time() > grace_at:
                print("  ⚠️ 未收到 EKF_STATUS_REPORT, 按旧固件放行")
                return True
            continue
        seen = True
        flags = int(getattr(m, 'flags', 0))
        if ekf_pos_ready(flags):
            print(f"  ✅ EKF 位置估计就绪 (flags=0x{flags:04x})")
            return True
    print("  ❌ EKF 位置估计未就绪(超时)")
    return False


def position_ready_state(fix, sats, flags, min_sats=AUTOTUNE_MIN_SATS,
                         require_ekf=True):
    """位置估计是否已就绪(可安全 GUIDED 解锁). 纯函数, 供 arm_vehicle 预检
    门禁与 GUI 状态条共用 (GUI 从共享遥测取参, 避免抢 recv_match).
    fix/sats/flags 可为 None(尚未收到对应消息).
    require_ekf=False: 固件未流式发送 EKF_STATUS_REPORT 时降级为只看 GPS.
    返回 (ready: bool, why: str) — why 为人话化缺项, 就绪时 '就绪'"""
    if fix is None:
        return False, '未收到GPS'
    if fix < 3:
        return False, '等GPS 3D定位(fix=%s)' % fix
    if sats is not None and sats < min_sats:
        return False, '卫星不足(%s<%d)' % (sats, min_sats)
    if flags is None:
        if require_ekf:
            return False, '未收到EKF状态'
        return True, '就绪(仅GPS)'
    if not ekf_pos_ready(int(flags)):
        f = int(flags)
        # 室内实测 0x00A7: 有姿态/速度/垂直位置 + CONST_POS_MODE, 无水平位置
        # — 旧文案只说"无有效位置"易被误读成姿态问题, 这里点明真正缺项
        if (f & EKF_CONST_POS_FLAG) and not (f & EKF_POS_HORIZ_FLAGS):
            return False, 'EKF无水平位置(恒定位,未融合GPS,flags=0x%04x)' % f
        return False, 'EKF无有效位置(flags=0x%04x)' % f
    return True, '就绪'


def wait_position_ready(master, timeout=ARM_PREPARE_WAIT, poll=0.25):
    """解锁前预检门禁 (v1.29.3): 主动等待 GPS 3D 定位 + EKF 位置估计就绪.
    背景: 'PreArm: Need Position Estimate' 是飞控 mandatory 检查
    (AP_Arming_Copter::mandatory_position_checks -> copter.position_ok()),
    ARMING_SKIPCHK 跳不掉, 且 GUIDED 本身 requires_position, 只能等就绪;
    上电后 EKF 收敛需 20~60s, 此前盲发 ARM 必被拒.
    每 5s 打一行进度(带具体缺项). 返回 (ready: bool, why: str)"""
    t0 = time.time()
    timeout = max(1.0, float(timeout))
    t_end = t0 + timeout
    # 超过此时限仍无 EKF_STATUS_REPORT -> 判定该固件不流式发送, 改只看 GPS
    ekf_grace = t0 + min(15.0, max(5.0, timeout / 3.0))
    fix = sats = flags = None
    no_ekf_msg = False
    last_report = -5.0

    # 主动请求一次 EKF_STATUS_REPORT(193), 降低"等不到该消息"概率
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
            0, 0, 0, 0, 0, 0)
    except Exception:
        pass

    print("⏳ 预检: 等待位置估计就绪 (最多 %ds)..." % timeout)
    last_req = t0
    while time.time() < t_end:
        m = master.recv_match(blocking=True, timeout=poll)
        now = time.time()
        # 每秒重发一次请求: 保证不依赖飞控是否主动流式发送
        if now - last_req >= 1.0:
            last_req = now
            try:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                    0, 0, 0, 0, 0, 0)
                master.mav.request_data_stream_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)
            except Exception:
                pass
        if m is not None:
            mtype = m.get_type()
            if mtype == 'GPS_RAW_INT':
                fix = int(getattr(m, 'fix_type', 0) or 0)
                sats = int(getattr(m, 'satellites_visible', 0) or 0)
            elif mtype == 'EKF_STATUS_REPORT':
                flags = int(getattr(m, 'flags', 0))
            elif mtype == 'STATUSTEXT':
                txt = m.text
                if isinstance(txt, (bytes, bytearray)):
                    txt = txt.decode('utf-8', 'replace')
                low = str(txt).lower()
                if 'ekf' in low or 'gps' in low or 'origin' in low:
                    print("    [飞控] %s" % txt)
        if not no_ekf_msg and flags is None and now > ekf_grace:
            no_ekf_msg = True
            print("  ⚠️ 未收到 EKF_STATUS_REPORT, 改按 GPS 3D 定位判定")

        ready, why = position_ready_state(fix, sats, flags,
                                          require_ekf=not no_ekf_msg)
        if ready:
            print("  ✅ 位置估计就绪: %s (fix=%s 卫星=%s%s)" % (
                why, fix, sats,
                '' if flags is None else ' flags=0x%04x' % flags))
            return True, why
        if now - last_report >= 5.0:
            last_report = now
            print("  ⏳ 等待位置估计就绪 (%ds/%ds): %s" % (
                int(now - t0), int(timeout), why))

    _, why = position_ready_state(fix, sats, flags,
                                  require_ekf=not no_ekf_msg)
    print("  ❌ 位置估计未就绪(超时%ds): %s" % (int(timeout), why))
    return False, why


def release_rc_override(master, handover=None):
    """释放/交还 RC 覆盖, 连发3帧防丢包 (v1.24).
    handover=None → 取全局 RC_HANDOVER; True → 全0帧交还遥控器
    (MAVLink约定: 通道值=0 即释放该通道, 覆盖撤销, 接收机摇杆立即生效
    — 物理遥控器"立刻接管"的唯一出口, 因 RC_OVERRIDE_TIME=-1 覆盖永不过期).
    handover=False → v1.23 中立帧: 无接收机时全0=无效PWM空中掉高,
    按 armed 选地面/空中中立.
    兜底: 取不到 armed 状态时按地面处理"""
    if handover is None:
        handover = bool(RC_HANDOVER)
    try:
        armed = bool(master.motors_armed())
    except Exception:
        armed = False
    if handover:
        frame = (0, 0, 0, 0)
        print("  RC覆盖收尾: 全0交还遥控器(物理摇杆立即生效)")
    else:
        frame = RC_NEUTRAL_AIR if armed else RC_NEUTRAL_GROUND
        print("  RC覆盖收尾: 中立帧(不交还, 无接收机模式)")
    for _ in range(3):
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            frame[0], frame[1], frame[2], frame[3], 0, 0, 0, 0
        )
        time.sleep(0.05)


def takeoff_althold(master, alt, timeout=90, climb_thr=None):
    """ALT_HOLD 下用 RC 油门覆盖爬升到 alt 米（默认通道: 1横滚/2俯仰/3油门/4偏航）。
    到达后中油门稳定 2 秒再释放；无论成败退出前必定释放覆盖.
    v1.24.4: climb_thr 缺省取 TAKEOFF_CLIMB_THR(UI 可调, 默认1800); 爬升段
    全程发 climb_thr 直到到位 — 旧版到位前降 1600 正好是 ±100 死区边缘
    (=零爬升需求): 低空目标(BOUNCE_ALT=2)从地面就发 1600 → 根本不离地,
    5m 目标爬到 3m 后也卡死; 1500=悬停(稳定段用, 需求为零=保持高度)"""
    if climb_thr is None:
        climb_thr = TAKEOFF_CLIMB_THR
    print(f"  ⬆️ ALT_HOLD 爬升至 {alt}m (油门 {climb_thr})...")
    t_end = time.time() + timeout
    rel = 0.0
    reached = False
    try:
        while time.time() < t_end:
            p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.2)
            if p:
                rel = p.relative_alt / 1000.0
                if rel >= alt - 0.5:
                    reached = True
                    break
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                1500, 1500, climb_thr, 1500, 0, 0, 0, 0
            )
        if reached:
            for _ in range(10):
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500, 1500, AUTOTUNE_SETTLE_THR, 1500, 0, 0, 0, 0
                )
                time.sleep(0.2)
            print(f"  ✅ 已到达 {rel:.1f}m，已释放遥控器控制权")
    finally:
        release_rc_override(master)
    return reached


def land_soft_final(master, phase0_timeout=120):
    """LAND/RTL 最后 1m 软着陆接管:
    高度 > SOFT_LAND_ALT 时维持原模式等飞控缓降; ≤SOFT_LAND_ALT 切 ALT_HOLD,
    按 0.2s 节拍用 RC 油门小步微调把降速压向 SOFT_LAND_RATE (限幅
    [SOFT_LAND_THR_MIN, SOFT_LAND_THR_MAX]: 超速→升油门, 不降→降油门),
    触地判定 = 高度≤SOFT_LAND_GROUND_ALT 且降速平稳(连续2拍)后:
    锁最低油门 → DISARM(被拒自动强制) → 释放 RC 覆盖。
    返回 True=已触地上锁; False=等待接管超时或缓降超时(已释放控制权)"""
    # 阶段0: 维持原模式(LAND/RTL), 等飞控降到接管高度
    t_end = time.time() + phase0_timeout
    last_rep = 0.0
    alt = None
    while time.time() < t_end:
        p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                              timeout=0.5)
        if p is None:
            continue
        alt = p.relative_alt / 1000.0
        if alt <= SOFT_LAND_ALT:
            break
        if time.time() - last_rep >= 1.0:
            print(f"  下降中... 高度: {alt:.1f}m "
                  f"(≤{SOFT_LAND_ALT}m 后接管微调)", end='\r')
            last_rep = time.time()
    else:
        print(f"\n  ⚠️ {phase0_timeout}s 内未降至 {SOFT_LAND_ALT}m, 本次不接管")
        return False

    # 阶段1: 接管 — 切 ALT_HOLD, 油门微调缓降
    print(f"\n  ⬇️ 已至 {alt:.2f}m, 切 ALT_HOLD 接管, "
          f"油门微调缓降 (目标 {SOFT_LAND_RATE}m/s)")
    master.set_mode_apm("ALT_HOLD")
    time.sleep(0.3)

    p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    prev_alt = p.relative_alt / 1000.0 if p else alt
    t_prev = time.time()
    thr = SOFT_LAND_THR_START
    stable = 0
    landed = False
    last_rep = 0.0
    t_end = time.time() + SOFT_LAND_SETTLE_T
    try:
        while time.time() < t_end:
            time.sleep(0.2)
            p = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                                  timeout=0.5)
            if p is None:
                continue
            now = time.time()
            alt = p.relative_alt / 1000.0
            dt = max(now - t_prev, 1e-3)
            rate = (prev_alt - alt) / dt      # 降速(米/秒, 向下为正)
            prev_alt, t_prev = alt, now
            if alt <= SOFT_LAND_GROUND_ALT and rate <= 0.3:
                stable += 1
                if stable >= 2:
                    landed = True
                    break
            else:
                stable = 0
            # 油门微调: 小步限幅, 超速升油门 / 不降降油门
            if rate > 0.6:
                thr += 15
            elif rate > SOFT_LAND_RATE * 1.4:
                thr += 8
            elif rate < 0.05:
                thr -= 10
            thr = max(SOFT_LAND_THR_MIN, min(SOFT_LAND_THR_MAX, thr))
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                1500, 1500, thr, 1500, 0, 0, 0, 0)
            if now - last_rep >= 1.0:
                print(f"  缓降: 高度 {alt:.2f}m 降速 {max(rate, 0.0):.2f}m/s "
                      f"油门 {thr}", end='\r')
                last_rep = now
    finally:
        release_rc_override(master)

    if not landed:
        print(f"\n  ⚠️ 缓降 {SOFT_LAND_SETTLE_T}s 未触地, 已释放控制权, "
              f"请人工处置")
        return False

    # 阶段2: 触地 — 锁最低油门立即上锁, 再释放覆盖
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, SOFT_LAND_THR_MIN, 1500, 0, 0, 0, 0)
    print("\n  ✅ 已着陆 (油门微调缓降完成), 锁最低油门并上锁...")
    disarm_vehicle(master, force=False, timeout=6)
    release_rc_override(master)
    print("  ✅ 软着陆完成: 电机已上锁, RC 覆盖已释放")
    return True


def wait_autotune(master, timeout=AUTOTUNE_TIMEOUT):
    """AUTOTUNE 模式下等待 PID 调参完成。
    监听 STATUSTEXT 打印进度并识别完成/失败；心跳离开 AUTOTUNE 则提前结束。
    返回 True=完成信号, False=超时/失败/模式被切走"""
    print(f"  ⚙️ PID 自动调参进行中（超时 {timeout}s），请勿打杆...")
    mode_map = master.mode_mapping() or {}
    at_id = mode_map.get('AUTOTUNE', 15)
    t_end = time.time() + timeout
    start = time.time()
    last_report = 0.0
    while time.time() < t_end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, (bytes, bytearray)):
                text = text.decode('utf-8', 'replace')
            text = str(text)
            low = text.lower()
            if 'autotun' in low:
                print(f"    [调参] {text}")
                if (('complete' in low or 'success' in low or 'finish' in low)
                        and 'incomplete' not in low):
                    print(f"  ✅ 收到调参完成信号（用时 {int(time.time() - start)}s）")
                    return True
                if 'fail' in low or 'error' in low or 'abort' in low:
                    print(f"  ⚠️ 调参报告失败（用时 {int(time.time() - start)}s）")
                    return False
            elif msg.severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                print(f"    [飞控] {text}")
        elif mtype == 'HEARTBEAT' and msg.custom_mode != at_id:
            print(f"  ⚠️ 已离开 AUTOTUNE 模式 (custom_mode={msg.custom_mode})，停止等待")
            return False
        now = time.time()
        if now - last_report >= 30.0:
            print(f"    ... 调参进行中，已用 {int(now - start)}s")
            last_report = now
    print(f"  ⚠️ 等待调参完成超时 ({timeout}s)")
    return False


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
        print(f"  ✅ 收到心跳包")
        print(f"     飞控类型: {autopilot}")
        print(f"     系统状态: {system_status}")
        return True
    else:
        print("  ❌ 未收到心跳包")
        return False


def test_02_arm(master):
    """测试2：解锁电机(自动切到可解锁模式并打印 PreArm 原因)"""
    print("\n" + "="*50)
    print("🔓 测试2：解锁电机")
    print("="*50)
    print("  说明: 解锁前自动切 GUIDED(位置估计未就绪时经确认降级ALT_HOLD);"
          " 油门摇杆请打到最低")
    ensure_disarm_delay(master)
    ok = arm_vehicle(master, mode='GUIDED', timeout=10)
    if ok:
        print(f"  ⚠️ 仅解锁未起飞: 约 {DISARM_DELAY_SEC}s 内不发起飞命令会自动上锁")
        print("     起飞请直接用 测试4 / 测试14(内部已集成 解锁+起飞)")
    return ok


def test_03_mode_switch(master):
    """测试3：模式切换(结束强制回到 GUIDED, 避免停在 RTL 导致无法解锁)"""
    print("\n" + "="*50)
    print("🔄 测试3：飞行模式切换")
    print("="*50)

    # pymavlink mode_mapping() 的键必须全大写，且 AltHold 实际键名是 ALT_HOLD
    # 注意: 不能停在 RTL/LAND 等不可解锁模式, 结束后切回 GUIDED
    modes = ["GUIDED", "STABILIZE", "ALT_HOLD", "LOITER", "RTL"]

    for mode in modes:
        master.set_mode_apm(mode)
        time.sleep(1)

        # 清掉缓冲里的旧心跳，确保读到切换后的新心跳
        while master.recv_match(blocking=False):
            pass

        # 读取当前模式
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            mode_map = master.mode_mapping() or {}
            # mode_mapping() 是 {名字: 编号}，需反查编号→名字
            current_mode = next(
                (name for name, num in mode_map.items() if num == msg.custom_mode),
                'UNKNOWN'
            )
            ok = "✅" if current_mode.upper() == mode else "❌"
            print(f"  {ok} 切换到 {mode} → 当前模式: {current_mode}")
        else:
            print(f"  ❌ 切换到 {mode} 失败（未收到心跳）")

    print("  切回 GUIDED(可解锁模式, 防止停在 RTL 无法解锁)...")
    master.set_mode_apm("GUIDED")
    time.sleep(1)
    while master.recv_match(blocking=False):
        pass
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        mode_map = master.mode_mapping() or {}
        cur = next((n for n, i in mode_map.items() if i == msg.custom_mode), '?')
        print(f"  ✅ 模式切换测试完成, 当前: {cur}")
    else:
        print("  ⚠️ 模式切换测试完成, 但未确认最终模式")
    return True


def test_04_takeoff(master):
    """测试4：起飞(集成解锁+起飞, 无需先跑测试2)"""
    print("\n" + "="*50)
    print(f"🛫 测试4：起飞到 {TAKEOFF_ALT}m")
    print("="*50)
    print("  流程: (延长DISARM_DELAY) → GUIDED解锁 → 立即 NAV_TAKEOFF → 等待到位")
    print("  说明: 已集成解锁, 不再依赖测试2; 未起飞时也可直接选本项")

    result = arm_and_takeoff(master, TAKEOFF_ALT, mode='GUIDED',
                             arm_timeout=10, takeoff_timeout=45,
                             set_delay=True)
    if result:
        time.sleep(2)
        print(f"  ✅ 起飞完成, 悬停于 {TAKEOFF_ALT}m (原地上方)")
        print("     GUIDED 下摇杆无效属正常; 遥控器接管: 拨模式开关到 LOITER")
    else:
        if not master.motors_armed():
            print("  ❌ 起飞失败(可能解锁失败或解锁后已被自动上锁)")
        else:
            print("  ❌ 起飞未到位(已解锁但高度未到目标)")
            print("     保持 GUIDED 悬停中; 可试 测试9 降落 / 测试10 上锁")
    return result


def test_05_velocity_forward(master):
    """测试5：前进速度控制"""
    print("\n" + "="*50)
    print(f"➡️ 测试5：前进速度 {TEST_SPEED} m/s，持续 {TEST_DURATION}s")
    print("="*50)

    type_mask = 0b0000110111000111
    start = time.time()

    while time.time() - start < TEST_DURATION:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            TEST_SPEED, 0, 0,
            0, 0, 0, 0, 0
        )
        pos = get_current_pos(master)
        if pos:
            print(f"  速度: {TEST_SPEED}m/s 前 | 高度: {pos['alt']:.1f}m", end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    send_zero_velocity(master)
    print(f"\n  ✅ 前进测试完成")
    return True


def test_06_velocity_side(master):
    """测试6：侧向速度控制"""
    print("\n" + "="*50)
    print(f"↪️ 测试6：侧向速度 {TEST_SPEED} m/s，持续 {TEST_DURATION}s")
    print("="*50)

    type_mask = 0b0000110111000111
    start = time.time()

    while time.time() - start < TEST_DURATION:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            0, TEST_SPEED, 0,
            0, 0, 0, 0, 0
        )
        pos = get_current_pos(master)
        if pos:
            print(f"  速度: {TEST_SPEED}m/s 右 | 高度: {pos['alt']:.1f}m", end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    send_zero_velocity(master)
    print(f"\n  ✅ 侧向测试完成")
    return True


def test_07_velocity_up(master):
    """测试7：垂直上升"""
    print("\n" + "="*50)
    print(f"⬆️ 测试7：垂直上升 {TEST_SPEED} m/s，持续 {TEST_DURATION}s")
    print("="*50)

    type_mask = 0b0000110111000111
    start = time.time()

    while time.time() - start < TEST_DURATION:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            0, 0, -TEST_SPEED,   # NED坐标系，负Z=上升
            0, 0, 0, 0, 0
        )
        pos = get_current_pos(master)
        if pos:
            print(f"  速度: {TEST_SPEED}m/s 上升 | 高度: {pos['alt']:.1f}m", end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    send_zero_velocity(master)
    print(f"\n  ✅ 垂直上升测试完成")
    return True


def test_08_hover(master):
    """测试8：悬停稳定性测试"""
    print("\n" + "="*50)
    print(f"⏸ 测试8：悬停 {TEST_DURATION}s，观察高度漂移")
    print("="*50)

    type_mask = 0b0000110111000111
    start = time.time()
    altitudes = []

    while time.time() - start < TEST_DURATION:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0
        )
        alt = get_current_alt(master)
        altitudes.append(alt)
        print(f"  悬停高度: {alt:.2f}m", end='\r')
        time.sleep(1.0 / CONTROL_RATE)

    if altitudes:
        drift = max(altitudes) - min(altitudes)
        print(f"\n  高度波动范围: {drift:.2f}m")
        if drift < 1.0:
            print("  ✅ 悬停稳定")
        else:
            print("  ⚠️ 悬停漂移较大，建议检查PID参数")
    return True


def test_09_land(master):
    """测试9：降落（LAND 降至1m 后油门微调软着陆并上锁）"""
    print("\n" + "="*50)
    print("🛬 测试9：自动降落")
    print("="*50)

    send_zero_velocity(master)
    time.sleep(1)

    master.set_mode_apm("LAND")
    print("  已切换LAND模式...")

    return land_soft_final(master, phase0_timeout=90)


def test_10_disarm(master):
    """测试10：锁定电机"""
    print("\n" + "="*50)
    print("🔒 测试10：锁定电机")
    print("="*50)
    return disarm_vehicle(master, force=False, timeout=5)


def auto_tune_flow(master, alt=None):
    """PID 自动调参流程 (测试11 调用): GPS检查 → ALT_HOLD解锁 → RC爬升 → LOITER稳定 →
    AUTOTUNE调参 → LOITER保存参数 → RTL返航降落 → 上锁.
    返回 True=收到调参完成信号且新参数已保存"""
    if alt is None:
        alt = AUTOTUNE_ALT
    print("\n" + "="*50)
    print(f"🎛 PID 自动调参（高度 {alt}m）")
    print("="*50)
    print("  流程: GPS检查 → ALT_HOLD解锁 → 爬升 → LOITER → AUTOTUNE → LOITER保存 → RTL降落")
    print("  ⚠️ 遥控器请开机备用，调参期间请勿打杆")

    if not wait_gps_lock(master):
        print("  ❌ GPS 未就绪，无法定点/调参")
        return False

    print("  [1/6] 切换 ALT_HOLD ...")
    master.set_mode_apm("ALT_HOLD")
    time.sleep(1)

    print("  [2/6] 解锁电机 ...")
    if not arm_vehicle(master, mode='ALT_HOLD', timeout=10):
        print("  ❌ 解锁失败")
        return False

    print(f"  [3/6] AltHold 爬升至 {alt}m ...")
    if not takeoff_althold(master, alt, climb_thr=AUTOTUNE_CLIMB_THR):
        print("  ❌ 爬升超时")
        return False

    print("  [4/6] 切换 LOITER 定点稳定 ...")
    master.set_mode_apm("LOITER")
    time.sleep(AUTOTUNE_LOITER_SETTLE)

    print("  [5/6] 切换 AUTOTUNE，开始 PID 自动调参 ...")
    master.set_mode_apm("AUTOTUNE")
    time.sleep(1)
    while master.recv_match(blocking=False):
        pass
    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    mode_map = master.mode_mapping() or {}
    at_id = mode_map.get('AUTOTUNE', 15)
    if not hb or hb.custom_mode != at_id:
        print("  ❌ 进入 AUTOTUNE 模式失败（固件不支持或未解锁）")
        return False
    print("  ✅ 已进入 AUTOTUNE")

    ok = wait_autotune(master, timeout=AUTOTUNE_TIMEOUT)

    print("  [6/6] 切回 LOITER，飞控将自动保存调参结果 ...")
    master.set_mode_apm("LOITER")
    time.sleep(2)

    print("  🏠 切换 RTL 返航 (≤1m 油门微调软着陆) ...")
    master.set_mode_apm("RTL")
    landed = land_soft_final(master, phase0_timeout=120)
    if not landed:
        print("  ⚠️ 等待降落超时，请人工确认")
        if master.motors_armed():
            disarm_vehicle(master, force=False, timeout=8)

    if ok:
        print("  ✅ PID 自动调参完成，新参数已保存")
    else:
        print("  ⚠️ 调参未确认全部完成，请检查上方日志")
    return ok


def test_11_autotune(master):
    """测试11：PID 自动调参（AltHold起飞 → Loiter → Autotune → Loiter保存 → RTL）"""
    return auto_tune_flow(master)


def _mode_name(master, custom_mode):
    """按 mode_mapping 反查模式名"""
    mode_map = master.mode_mapping() or {}
    return next((n for n, i in mode_map.items() if i == custom_mode),
                f'UNKNOWN({custom_mode})')


def _fmt_optional(value, unit='', digits=1):
    """数值格式化, None/非法值显示为 --"""
    if value is None:
        return '--'
    if isinstance(value, float):
        return f'{value:.{digits}f}{unit}'
    return f'{value}{unit}'


def test_12_status(master):
    """测试12：飞机状态（HEARTBEAT/GPS/位置/姿态/VFR/SYS_STATUS，同 Mission Planner）"""
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
        'ekf': None,
    }
    got = {'hb': False, 'pos': False}

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
        elif mtype == 'EKF_STATUS_REPORT':
            # 与 MP 类似: 位置/速度融合方差越小越好, health 标志
            try:
                st['ekf'] = {
                    'vel_var': msg.velocity_variance,
                    'pos_horiz_var': msg.pos_horiz_variance,
                    'pos_vert_var': msg.pos_vert_variance,
                    'compass_var': msg.compass_variance,
                    'terrain_alt': msg.terrain_alt,
                    'flags': msg.flags,
                }
            except AttributeError:
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
    print("  🛩 飞行状态 (Mission Planner 风格)")
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
    if st['ekf']:
        e = st['ekf']
        print(f"  EKF: 水平位置方差 {_fmt_optional(e['pos_horiz_var'], '', 3)}  "
              f"速度方差 {_fmt_optional(e['vel_var'], '', 3)}  "
              f"罗盘方差 {_fmt_optional(e['compass_var'], '', 3)}")
        print(f"       flags=0x{e['flags']:04X}  "
              f"地形高度 {_fmt_optional(e['terrain_alt'], ' m', 1)}")
    print("-"*50)
    print("  ✅ 状态获取完成")
    return True


def test_13_battery(master):
    """测试13：电池状态（SYS_STATUS / BATTERY_STATUS，同 Mission Planner）"""
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
        print("     （可检查固件是否配置电池监测 BATT_* 参数 / 电池Monitor）")
        return False

    print("\n" + "-"*50)
    print("  🔋 电池状态 (Mission Planner 风格)")
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
    print("  流程: 等GPS → (延长DISARM_DELAY) → 解锁+立即起飞 → 悬停 → LAND → 上锁")
    print("  ⚠️ 原地垂直起降，螺旋桨区域保持清空，遥控器随时可接管")

    # v1.24.3: GPS 未就绪不再直接失败 — arm_vehicle 会经确认降级 ALT_HOLD
    # (气压定高, 无需GPS) + 油门爬升, 悬停/降落同样只需气压计
    if not wait_gps_lock(master, timeout=10):
        print("  ⚠️ GPS 未就绪: 将走 ALT_HOLD 降级起降"
              "(无GPS定点/返航, 悬停靠气压定高)")

    print("  [1/4] 解锁 + 起飞（一体执行）...")
    if not arm_and_takeoff(master, BOUNCE_ALT, mode='GUIDED',
                           arm_timeout=10, takeoff_timeout=BOUNCE_TAKEOFF_TIMEOUT,
                           set_delay=True):
        print("  ❌ 解锁/起飞失败")
        if not master.motors_armed():
            print("     (未保持解锁状态)")
        else:
            master.set_mode_apm("LAND")
            print("     已切 LAND 保安全")
        return False

    print(f"  [2/4] 悬停 {BOUNCE_HOVER_SEC:.0f}s ...")
    t_end = time.time() + BOUNCE_HOVER_SEC
    while time.time() < t_end:
        send_zero_velocity(master, count=2)
        left = t_end - time.time()
        alt = get_current_alt(master)
        print(f"  悬停中 剩余 {max(left, 0):.1f}s | 高度 {alt:.2f}m", end='\r')
        time.sleep(0.2)
    print(f"\n  ✅ 悬停完成")

    print("  [3/4] 切 LAND 降落 (≤1m 油门微调软着陆) ...")
    send_zero_velocity(master)
    time.sleep(0.5)
    master.set_mode_apm("LAND")
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
#  DataFlash 日志下载 (LOG_REQUEST_LIST / LOG_REQUEST_DATA)
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
    """从飞控下载 DataFlash 日志到 out_dir/flash_log_<id>_<时间>.bin.
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
            out_dir, 'flash_log_%03d_%s.bin'
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
    '11': ('PID自动调参',  test_11_autotune),
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
    print("🚁 APM 功能测试工具")
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
