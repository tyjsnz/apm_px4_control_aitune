# AI PID Tuning GUI (Standalone)
from ai_tune_core import *
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import json, os, time, threading, queue

_BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(_BASE, 'ai_tune_config.json')
BACKUP_DIR = os.path.join(_BASE, 'ai_tune_backups')

# ===== 首飞/起飞参数预设 (rng=固件合法范围, aliases=4.7 起改名+换单位, scale=换算系数) =====
def _apm_takeoff_params(speed_up, accel_z, pilot_speed_up):
    return [
        dict(name='MOT_SPIN_ARM', value=0.10, unit='', rng=(0.0, 0.2),
             desc='解锁后电机最低转速',
             help='拆桨后用地面站电机测试逐步升油门，记录电机"刚好稳定起转"的油门；实测约 12% 就填 0.14。\n'
                  '必须小于 MOT_SPIN_MIN（至少低 0.03），否则解锁后可能不转或转速跳变。'),
        dict(name='MOT_SPIN_MIN', value=0.15, unit='', rng=(0.0, 0.25),
             desc='飞行最低油门下限',
             help='必须大于 MOT_SPIN_ARM，建议至少高 0.03；\n'
                  '它是油门线性区下限，过低会导致起飞段输出不线性、离地瞬间抖动。'),
        dict(name='MOT_SPIN_MAX', value=0.95, unit='', rng=(0.9, 1.0),
             desc='最大输出上限(推力线性化顶点)',
             help='固件默认 0.95：顶部约 5% 油门几乎不产生额外推力，留作余量。\n'
                  '不要为了"更有力"设到 1.0，那会让油门在顶端饱和。'),
        dict(name='MOT_THST_HOVER', value=0.25, unit='', rng=(0.125, 0.6875),
             desc='悬停油门初始估计(0~1)',
             help='首飞保守给 0.25，交给 MOT_HOVER_LEARN=2 自己学。\n'
                  '估高 → Guided 起飞瞬间过猛(火箭机尤其危险)；估低 → 离地慢、反复贴地。\n'
                  '经验区间 0.25~0.50；>0.65 动力不足，<0.15 推重比过大、降落困难。'),
        dict(name='MOT_HOVER_LEARN', value=2, unit='', rng=(0, 2),
             desc='悬停油门学习: 0=关 1=学 2=学并保存',
             help='保持 2：在 AltHold/Loiter 悬停 ≥30s，落地并上锁后才写回 MOT_THST_HOVER。\n'
                  '频繁更换载荷(相机/电池重量变化)时用 1，只学不存，重启回原值。'),
        dict(name='TKOFF_THR_MAX', value=1.0, unit='', rng=(0.0, 1.0),
             desc='起飞阶段允许的最大油门',
             help='允许起飞段用满油门；真正"多快离地"由 WPNAV_SPEED_UP / WPNAV_ACCEL_Z 决定。\n'
                  '动力严重不足时才需要下调，一般保持 1.0。'),
        dict(name='WPNAV_SPEED_UP', value=speed_up, unit='cm/s', rng=(10, 1000),
             aliases=[('WP_SPD_UP', 0.01)],
             desc='Guided/AUTO 起飞上升速度',
             help='离地手感最关键的参数(固件默认 250)。\n'
                  '离地太猛 → 降到 80；离地犹豫、反复贴地 → 升到 120~150。\n'
                  '单位 cm/s；4.7 起改名 WP_SPD_UP 且单位换 m/s，本工具自动识别换算。'),
        dict(name='WPNAV_ACCEL_Z', value=accel_z, unit='cm/s/s', rng=(50, 500),
             aliases=[('WP_ACC_Z', 0.01)],
             desc='垂直加速度上限',
             help='越小越"绵"(固件默认 100，范围 50~500)；首飞 100~200 合适。\n'
                  '过大时到顶会冲高/回荡，过小则爬升像"电梯迟滞"。'),
        dict(name='PILOT_SPEED_UP', value=pilot_speed_up, unit='cm/s', rng=(50, 500),
             aliases=[('PILOT_SPD_UP', 0.01)],
             desc='手动模式上升速度上限',
             help='AltHold/Loiter 油门杆到底的最大爬升率(固件默认 250)。\n'
                  '首飞先 100，低空手感确认后再回 250。'),
        dict(name='PILOT_ACCEL_Z', value=200, unit='cm/s/s', rng=(50, 500),
             aliases=[('PILOT_ACC_Z', 0.01)],
             desc='手动模式垂直加速度',
             help='固件默认 250，范围 50~500；调小更平滑。\n'
                  '振动大的机器设过高会让高度出现"抖动"。'),
    ]


TAKEOFF_PRESETS = [
    dict(id='apm1', fc='APM_COPTER',
         label='A 保守首飞起点 (10项 · 推荐首飞)',
         note='ArduCopter 垂直起飞的保守起点：先拆桨核对起转油门，再低空 2~3m 试飞。'
              '离地快慢由 WPNAV_SPEED_UP / WPNAV_ACCEL_Z 决定，其余多数与固件默认一致。',
         params=_apm_takeoff_params(100, 150, 100)),
    dict(id='apm2', fc='APM_COPTER',
         label='B 离地太猛 → 更慢 (推重比大/火箭机)',
         note='A 的降速档：起飞上升 0.8 m/s、垂直加速度 1.2 m/s²。'
              '首飞宁可慢一点离地，也不要为了响应快把悬停油门设高。',
         params=_apm_takeoff_params(80, 120, 80)),
    dict(id='apm3', fc='APM_COPTER',
         label='C 离地犹豫/贴地 → 更快',
         note='A 的提速档：起飞上升 1.5 m/s。'
              '若升到 150 仍反复贴地，先核对 MOT_THST_HOVER 是否估低(用 MOT_HOVER_LEARN=2 学 30s)。',
         params=_apm_takeoff_params(150, 200, 150)),
    dict(id='apm_fence', fc='APM_COPTER',
         label='D 地理围栏 (首飞安全兜底)',
         note='首飞强烈建议一起写入：半径 50m + 高度 20m 的圆柱围栏，越界自动 RTL。'
              'RTL_ALT 必须 ≤ FENCE_ALT_MAX，否则返航爬升会再次越界形成循环。',
         params=[
             dict(name='FENCE_ENABLE', value=1, unit='', rng=(0, 1),
                  desc='启用地理围栏',
                  help='0=关 1=开。配合 FENCE_ACTION=1 越界自动返航。\n'
                       '注意：改动不会被持久保存的例外是 MAVLink/RC 临时开关，本项写入后断电仍有效。'),
             dict(name='FENCE_TYPE', value=3, unit='', rng=(0, 7),
                  desc='围栏类型位掩码',
                  help='bit0=最高高度, bit1=圆形(圆心=home), bit2=多边形, bit3=最低高度。\n'
                       '3 = 高度+圆形；多边形需先从 GCS 导入顶点，首飞用不到。固件默认 7。'),
             dict(name='FENCE_ACTION', value=1, unit='', rng=(0, 5),
                  desc='越界动作',
                  help='0=仅报告, 1=RTL或LAND, 2=强制降落, 3=SmartRTL或RTL, 4=Brake或LAND。\n'
                       '首飞用 1。设定为 0 等于没有围栏。'),
             dict(name='FENCE_RADIUS', value=50, unit='m', rng=(30, 10000),
                  desc='圆形围栏半径(米)',
                  help='固件范围 30~10000m，20m 会被拒绝或夹到 30m。\n'
                       '首飞建议 50m，给返航转弯留余量。'),
             dict(name='FENCE_ALT_MAX', value=20, unit='m', rng=(10, 1000),
                  desc='允许最大相对高度(米)',
                  help='必须 ≥ RTL_ALT。想用 10m 高度围栏，先把 RTL_ALT 降到 800(8m)。'),
             dict(name='FENCE_MARGIN', value=5, unit='m', rng=(1, 10),
                  desc='距围栏的预留余量(米)',
                  help='到达围栏前提前减速/刹停的缓冲距离，太小容易来不及刹住。'),
             dict(name='RTL_ALT', value=1000, unit='cm', rng=(0, 10000),
                  desc='返航爬升高度(厘米)',
                  help='固件默认 1500(15m) > 20m 围栏的爬升安全余量，首飞调到 1000(10m) 更稳妥。\n'
                       '4.7 起可能改名(按 m 计)，本工具自动识别。'),
         ]),
    dict(id='px4', fc='PX4_MC',
         label='E PX4 保守起飞 (对照实现)',
         note='PX4 多旋翼的保守起飞对照项(单位为 m / m/s，与 APM 预设不同)：'
              '起飞速度与上升加速度均低于默认值，首飞目标 2m 悬停。',
         params=[
             dict(name='MPC_TKO_SPEED', value=1.0, unit='m/s', rng=(0.5, 8.0),
                  desc='Takeoff 模式上升速度',
                  help='固件默认 1.5 m/s；首飞取 1.0，离地更平顺。'),
             dict(name='MPC_Z_V_AUTO_UP', value=1.0, unit='m/s', rng=(0.5, 8.0),
                  desc='AUTO/任务模式上升速度上限',
                  help='固件默认 3.0 m/s；用于 AUTO 起飞与任务爬升，首飞压到 1.0。'),
             dict(name='MPC_Z_VEL_MAX_UP', value=1.5, unit='m/s', rng=(0.5, 8.0),
                  desc='手动模式上升速度上限',
                  help='固件默认 3.0 m/s；手动杆到底的最大爬升率，首飞 1.5。'),
             dict(name='MPC_ACC_UP_MAX', value=3.0, unit='m/s²', rng=(2.0, 15.0),
                  desc='上升加速度上限',
                  help='固件默认 4.0，范围 2~15；越小越平滑，过小则爬升迟钝。'),
             dict(name='MIS_TAKEOFF_ALT', value=2.0, unit='m', rng=None,
                  desc='Takeoff 指令目标高度',
                  help='固件默认 2.5m；首飞 2m 起步，稳定后再加高。'),
         ]),
]

TAKEOFF_HELP = """起飞调参 · 帮助说明
========================================================
一、这是什么
  一套"保守可实验"的垂直起飞参数起点，先低空起飞测试，不要一上来就追求快起飞。
  预设 D(地理围栏) 建议和 A 一起写入，作为首飞的最后安全网。

二、推荐首飞参数起点 (与上表一致)
  MOT_SPIN_ARM   0.10  解锁后电机最低转速
  MOT_SPIN_MIN   0.15  飞行最低油门下限(必须 > SPIN_ARM 至少 0.03)
  MOT_SPIN_MAX   0.95  最大输出上限
  MOT_THST_HOVER 0.25  悬停油门初始估计
  MOT_HOVER_LEARN  2   让飞控学习并保存真实悬停油门
  TKOFF_THR_MAX  1.0   起飞阶段允许最大油门
  WPNAV_SPEED_UP 100   起飞上升速度 (cm/s)
  WPNAV_ACCEL_Z  150   垂直加速度 (cm/s/s)
  PILOT_SPEED_UP 100   手动模式上升速度上限 (cm/s)
  PILOT_ACCEL_Z  200   手动模式垂直加速度 (cm/s/s)

三、这些值怎么用
  · MOT_SPIN_ARM=0.10：先拆桨，在地面站电机测试里逐步提高油门，确认电机刚好稳定起转。
    如果实际起转点在 12%，改成 0.14 左右。
  · MOT_SPIN_MIN=0.15：必须大于 MOT_SPIN_ARM，至少高 0.03。
  · MOT_THST_HOVER=0.25：先保守估计，别一上来就设高。推重比大的机器真实悬停油门可能更低，
    保留 0.25 让飞控学习。
  · TKOFF_THR_MAX=1.0：允许起飞用满油门，但真正起飞速度由 WPNAV_SPEED_UP/WPNAV_ACCEL_Z 控制。
  · WPNAV_SPEED_UP=100、WPNAV_ACCEL_Z=150：Guided 起飞最关键的两项。默认上升速度通常偏快，
    首飞建议降到 100 cm/s，让它慢慢离地。

四、MAVLink Guided 起飞建议流程
  1. 先切 STABILIZE，解锁，低油门确认姿态稳定。
  2. 切 GUIDED。
  3. 发送 MAV_CMD_NAV_TAKEOFF，目标高度先设 2~3 米。
  4. 起飞后观察：离地太猛 → WPNAV_SPEED_UP 降到 80；离地犹豫、反复贴地 → 升到 120~150。
  5. 首次不要飞高，2~3 米悬停几秒后切 RTL 或手动降落。

五、特别注意
  · 推力大的机器若 MOT_THST_HOVER 估高，Guided 起飞瞬间会过猛；估低则离地慢。
    首飞宁可慢一点离地，也不要为了响应快把悬停油门设高。
  · Guided 起飞前确认 GPS/EKF 定位可靠，并设置低高度地理围栏(建议半径 50m、高度 20m)。

--------------------------------------------------------
六、本工具补充 (务必先读)
  1) 固件版本差异：ArduPilot 4.7 起大量参数改名并换成 SI 单位
       PILOT_SPEED_UP(cm/s) → PILOT_SPD_UP(m/s)
       PILOT_ACCEL_Z(cm/s/s) → PILOT_ACC_Z(m/s/s)
       WPNAV_SPEED_UP/WPNAV_ACCEL_Z → WP_SPD_UP/WP_ACC_Z
     本页"读取当前值"会自动探测实际存在的参数名，并按单位换算后再显示/写入，
     所以 4.6 与 4.7 固件都能用同一套推荐值(按 cm/s 填)。
  2) 写入前自动备份：点"应用勾选"会先把选中参数的当前值存进 ai_tune_backups/，
     出问题可到「备份管理」页一键回滚。
  3) 只写不同的项：勾选后，当前值已等于推荐值的参数会被跳过，减少无谓写入。
  4) 一致性校验：应用前会检查 MOT_SPIN_MIN > MOT_SPIN_ARM+0.03、
     FENCE_ALT_MAX >= RTL_ALT、数值是否落在固件合法范围，被拒绝时请按提示改。
  5) 参数生效时机：本页参数多数立即生效，无需重启；但 MOT_PWM_TYPE、遥控协议等
     需要重启才生效。改完建议重启飞控再试飞。
  6) 拆桨地面验证顺序：
     写入参数 → 拆桨 → 电机测试核对 MOT_SPIN_ARM 起转点 → 装桨 →
     STABILIZE 低油门看姿态 → GUIDED 起飞 2~3m → RTL/降落 → 落地保持 Loiter ≥30s
     (让 MOT_HOVER_LEARN=2 学习并保存悬停油门) → 上锁。
  7) 起飞前检查清单：
     GPS 3D 定位且卫星 ≥10、EKF/地磁就绪、ARMING_CHECK 全开、遥控失控保护已设、
     低电压保护已设、罗盘已校准、桨叶方向与型号正确、电池电压足够、场地空旷无人。
  8) 症状对照：
     离地太猛/窜天      → 降 WPNAV_SPEED_UP、核对 MOT_THST_HOVER 是否偏大
     离地犹豫/反复贴地  → 升 WPNAV_SPEED_UP、核对 MOT_THST_HOVER 是否偏小
     到顶冲高回荡        → 降 WPNAV_ACCEL_Z
     高度抖动            → 检查振动(桨/机架)、降 PILOT_ACCEL_Z
     RTL 后又越界        → FENCE_ALT_MAX 必须 ≥ RTL_ALT
  9) 建议开着日志(LOG_BITMASK 保留基础项)飞行，事后用「日志管理」回看。
"""

def _is_checked(value):
    """Treeview 的值读回来是字符串，'False' 也是非空字符串，必须显式判断。"""
    return str(value).strip().lower() in ('✓', 'true', '1', 'y', 'yes')

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AI PID Tuning Tool - Multirotor APM/PX4')
        self.geometry('1400x900')
        self.mav_master = None
        self.fc_type = None
        self.connected = False
        self.config = self.load_config()
        self.param_names = []
        self.history = []
        self.round = 0
        
        # Threading
        self.action_q = queue.Queue()
        self.log_q = queue.Queue()
        self.worker_thread = None
        self.worker_running = True
        self.action_busy = threading.Event()
        
        # Flight test
        self.flight_recorder = None
        self.recording = False
        self.step_abort = threading.Event()
        
        self.build_ui()
        self.refresh_ports()
        self.start_worker()
        self.poll_log()
        self.log('Ready')

    def load_config(self):
        try:
            if os.path.exists(ROOT):
                with open(ROOT, 'r', encoding='utf-8') as f:
                    return json.load(f)
        except Exception:
            pass
        return {'provider': 'deepseek', 'api_key': '', 'model': 'deepseek-flash', 'base_url': '',
                'providers': {}, 'last_port': '', 'baud': 115200, 'last_spec': {}}

    def save_config(self):
        self.store_provider_state()
        cfg = {
            'provider': provider_id_by_label(self.provider_var.get()),
            'api_key': self.api_key_var.get(),
            'model': self.model_var.get(),
            'base_url': self.base_url_var.get(),
            'providers': getattr(self, 'provider_state', {}),
            'last_port': self.port_var.get(),
            'baud': int(self.baud_var.get()),
            'last_spec': self.get_spec_dict()
        }
        try:
            with open(ROOT, 'w', encoding='utf-8') as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def get_spec_dict(self):
        return {
            'fc_type': self.fc_type_var.get(),
            'frame_type': self.frame_type_var.get(),
            'size_inch': self.size_var.get(),
            'weight_g': self.weight_var.get(),
            'motor_kv': self.kv_var.get(),
            'motor_type': self.motor_type_var.get(),
            'motor_max_current': self.motor_max_current_var.get(),
            'motor_resistance': self.motor_resistance_var.get(),
            'motor_pole': self.motor_pole_var.get(),
            'esc_protocol': self.esc_protocol_var.get(),
            'esc_max_current': self.esc_max_current_var.get(),
            'esc_bec': self.esc_bec_var.get(),
            'prop_size': self.prop_size_var.get(),
            'prop_pitch': self.prop_pitch_var.get(),
            'prop_blades': self.prop_blades_var.get(),
            'prop_material': self.prop_material_var.get(),
            'battery_s': self.battery_var.get(),
            'battery_capacity': self.battery_capacity_var.get(),
            'battery_c_rating': self.battery_c_rating_var.get(),
            'notes': self.notes_text.get('1.0', 'end-1c')
        }

    def set_spec_from_dict(self, d):
        self.fc_type_var.set(d.get('fc_type', '自动识别'))
        self.frame_type_var.set(d.get('frame_type', '四旋翼'))
        self.size_var.set(d.get('size_inch', '5寸'))
        self.weight_var.set(d.get('weight_g', ''))
        self.kv_var.set(d.get('motor_kv', ''))
        self.motor_type_var.set(d.get('motor_type', ''))
        self.motor_max_current_var.set(d.get('motor_max_current', ''))
        self.motor_resistance_var.set(d.get('motor_resistance', ''))
        self.motor_pole_var.set(d.get('motor_pole', ''))
        self.esc_protocol_var.set(d.get('esc_protocol', ''))
        self.esc_max_current_var.set(d.get('esc_max_current', ''))
        self.esc_bec_var.set(d.get('esc_bec', ''))
        self.prop_size_var.set(d.get('prop_size', ''))
        self.prop_pitch_var.set(d.get('prop_pitch', ''))
        self.prop_blades_var.set(d.get('prop_blades', ''))
        self.prop_material_var.set(d.get('prop_material', ''))
        self.battery_var.set(d.get('battery_s', ''))
        self.battery_capacity_var.set(d.get('battery_capacity', ''))
        self.battery_c_rating_var.set(d.get('battery_c_rating', ''))
        self.notes_text.delete('1.0', 'end')
        self.notes_text.insert('1.0', d.get('notes', ''))

    def store_provider_state(self):
        pid = provider_id_by_label(self.provider_var.get())
        self._cur_provider = pid
        self.provider_state[pid] = {
            'key': self.api_key_var.get().strip(),
            'model': self.model_var.get().strip(),
            'base_url': self.base_url_var.get().strip()
        }

    def on_provider_change(self, event=None):
        old = getattr(self, '_cur_provider', None)
        if old and hasattr(self, 'api_key_var'):
            self.provider_state[old] = {
                'key': self.api_key_var.get().strip(),
                'model': self.model_var.get().strip(),
                'base_url': self.base_url_var.get().strip()
            }
        pid = provider_id_by_label(self.provider_var.get())
        self._cur_provider = pid
        info = provider_info(pid)
        st = self.provider_state.get(pid, {})
        self.api_key_var.set(st.get('key', ''))
        self.base_url_var.set(st.get('base_url') or info['base_url'])
        self.model_var.set(st.get('model') or (info['models'][0] if info['models'] else ''))
        self.model_cb['values'] = info['models']
        self.api_key_label.configure(text=info['label'] + ' API Key:')
        self.log(f'已切换 AI 服务商: {info["label"]}')

    def get_ai_settings(self):
        """返回 (provider_id, api_key, base_url, model)，在主线程调用后传入工作线程。"""
        pid = provider_id_by_label(self.provider_var.get())
        info = provider_info(pid)
        return (pid, self.api_key_var.get().strip(),
                self.base_url_var.get().strip() or info['base_url'],
                self.model_var.get().strip())

    def check_ai_settings(self):
        pid, key, base_url, model = self.get_ai_settings()
        info = provider_info(pid)
        if not key and not info.get('key_optional'):
            messagebox.showwarning('提示', f'请先填写 {info["label"]} API Key')
            return None
        if not base_url:
            messagebox.showwarning('提示', '请先填写 Base URL')
            return None
        if not model:
            messagebox.showwarning('提示', '请先选择或输入模型名')
            return None
        return (pid, key, base_url, model)

    def build_ui(self):
        # Top bar
        top = ttk.Frame(self, padding=5)
        top.pack(fill='x')
        ttk.Label(top, text='串口:').pack(side='left')
        self.port_var = tk.StringVar()
        self.port = ttk.Combobox(top, textvariable=self.port_var, width=20)
        self.port.pack(side='left', padx=2)
        ttk.Label(top, text='波特率:').pack(side='left')
        self.baud_var = tk.StringVar(value=str(self.config.get('baud', 115200)))
        self.baud = ttk.Combobox(top, textvariable=self.baud_var, width=8, values=['57600', '115200', '921600'])
        self.baud.pack(side='left', padx=2)
        ttk.Button(top, text='刷新', command=self.refresh_ports).pack(side='left', padx=2)
        self.btn_conn = ttk.Button(top, text='连接', command=self.on_connect)
        self.btn_conn.pack(side='left', padx=2)
        self.btn_disc = ttk.Button(top, text='断开', command=self.on_disconnect, state='disabled')
        self.btn_disc.pack(side='left', padx=2)
        self.fc_lbl = ttk.Label(top, text='FC: AUTO')
        self.fc_lbl.pack(side='left', padx=10)
        self.link_lbl = ttk.Label(top, text='○ 未连接', foreground='gray')
        self.link_lbl.pack(side='left', padx=5)

        # Main body
        body = ttk.Frame(self)
        body.pack(fill='both', expand=True, padx=5, pady=5)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # Left panel: Spec form + API
        left = ttk.LabelFrame(body, text='机型规格与API', padding=8)
        left.grid(row=0, column=0, sticky='nsew', padx=(0,5))
        
        # Spec form
        row = 0
        
        # Tooltip class
        class ToolTip:
            def __init__(self, widget, text):
                self.widget = widget
                self.text = text
                self.tip = None
                widget.bind('<Enter>', self.show)
                widget.bind('<Leave>', self.hide)
            def show(self, event=None):
                if self.tip: return
                x = self.widget.winfo_rootx() + 20
                y = self.widget.winfo_rooty() + self.widget.winfo_height() + 5
                self.tip = tk.Toplevel(self.widget)
                self.tip.wm_overrideredirect(True)
                self.tip.wm_geometry(f'+{x}+{y}')
                label = tk.Label(self.tip, text=self.text, justify='left',
                               background='#ffffe0', relief='solid', borderwidth=1,
                               font=('微软雅黑', 8), wraplength=300)
                label.pack()
            def hide(self, event=None):
                if self.tip:
                    self.tip.destroy()
                    self.tip = None
        
        def add_field(label, var, values=None, width=18, tooltip=None):
            nonlocal row
            lbl = ttk.Label(left, text=label)
            lbl.grid(row=row, column=0, sticky='w', pady=2)
            if values:
                w = ttk.Combobox(left, textvariable=var, values=values, width=width)
            else:
                w = ttk.Entry(left, textvariable=var, width=width)
            w.grid(row=row, column=1, sticky='ew', pady=2)
            if tooltip:
                ToolTip(lbl, tooltip)
                ToolTip(w, tooltip)
            row += 1
            return w

        self.frame_type_var = tk.StringVar()
        self.size_var = tk.StringVar()
        self.weight_var = tk.StringVar()
        self.kv_var = tk.StringVar()
        self.motor_type_var = tk.StringVar()
        self.motor_max_current_var = tk.StringVar()
        self.motor_resistance_var = tk.StringVar()
        self.motor_pole_var = tk.StringVar()
        self.esc_protocol_var = tk.StringVar()
        self.esc_max_current_var = tk.StringVar()
        self.esc_bec_var = tk.StringVar()
        self.prop_pitch_var = tk.StringVar()
        self.prop_blades_var = tk.StringVar()
        self.prop_material_var = tk.StringVar()
        self.prop_size_var = tk.StringVar()
        self.battery_var = tk.StringVar()
        self.battery_capacity_var = tk.StringVar()
        self.battery_c_rating_var = tk.StringVar()
        self.fc_type_var = tk.StringVar(value='自动识别')

        add_field('飞控类型', self.fc_type_var, ['自动识别', 'APM Copter', 'PX4 MC'],
            tooltip='飞控类型选择。自动识别：通过心跳包autopilot字段判断(APM=3, PX4=12)。手动选择可强制指定参数集。')
        add_field('机架类型', self.frame_type_var, ['四旋翼', '六旋翼', '八旋翼'],
            tooltip='机架构型。影响推力分配矩阵、混控逻辑。四旋翼最常见，六/八旋翼冗余性更好。')
        add_field('尺寸档', self.size_var, ['5寸', '7寸', '8寸', '10寸', '12寸', '13寸', '15寸'],
            tooltip='机架尺寸档位(桨叶直径对应)。影响惯性矩、推力杠杆臂。飞控无法获取，需人工测量机臂长度确定。')
        add_field('机身重量(g)', self.weight_var,
            tooltip='机身空重(不含电池)。影响悬停油门、推重比计算。飞控无法获取，需电子秤称量。')
        ttk.Separator(left, orient='horizontal').grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        ttk.Label(left, text='电机参数', font=('微软雅黑', 9, 'bold')).grid(row=row, column=0, columnspan=2, sticky='w', pady=2)
        row += 1
        add_field('电机KV', self.kv_var,
            tooltip='电机KV值(RPM/V)。飞控无法直接获取。\n其他获取方式：\n1. 空载测试：测空载电压+转速计算\n2. 电机铭牌/说明书\n3. 厂家数据库查询\n⚠ KV值直接影响推力曲线和PID初始值')
        add_field('电机类型', self.motor_type_var, ['外转子', '内转子'],
            tooltip='电机结构类型。外转子转矩大适合大桨，内转子转速高适合小桨。飞控无法获取，需查看电机外观/说明书。')
        add_field('最大电流(A)', self.motor_max_current_var,
            tooltip='电机最大连续电流。飞控无法获取。\n获取方式：电机说明书/厂家规格书。用于计算最大推力、电调选型、电池C数校验。')
        add_field('内阻(mΩ)', self.motor_resistance_var,
            tooltip='电机相内阻。飞控无法获取。\n获取方式：\n1. 专用电机测试仪(相电阻测试)\n2. 万用表测量(需换算)\n3. 厂家数据表\n⚠ 内阻影响电压跌落、效率、最大功率')
        add_field('极对数', self.motor_pole_var, ['12N14P', '12N16P', '14N12P', '其他'],
            tooltip='电机定子槽数/转子磁钢数(如12N14P=12槽14极)。\nPX4可通过参数MOT_POLE_COUNT获取极对数；APM无此参数。\n说明书通常标注"14P"或"7对极"。影响电调换相时序和转速计算。')
        ttk.Separator(left, orient='horizontal').grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        ttk.Label(left, text='电调参数', font=('微软雅黑', 9, 'bold')).grid(row=row, column=0, columnspan=2, sticky='w', pady=2)
        row += 1
        add_field('电调协议', self.esc_protocol_var, ['DShot1200', 'DShot600', 'DShot300', 'DShot150', 'Multishot', 'Oneshot125', 'PWM'],
            tooltip='电调通信协议。飞控通过参数设置(MOT_PWM_TYPE等)，但无法反向读取电调实际协议。\n需查看电调说明书或配置软件(如BLHeliSuite/AM32)。\n⚠ 协议不匹配会导致电调不工作或抖动。\nDShot1200: 1.2Mbps, 最新最快协议，需电调/飞控固件支持。')
        add_field('最大电流(A)', self.esc_max_current_var,
            tooltip='电调最大连续电流额定值。飞控无法获取。\n需查看电调铭牌/说明书。用于验证电调是否匹配电机最大电流(建议留20%余量)。')
        add_field('BEC输出', self.esc_bec_var, ['5V/3A', '5V/5A', '无BEC', '可调'],
            tooltip='电调内置BEC输出规格。飞控无法获取。\n查看电调说明书。无BEC时需外接UBEC供电飞控/舵机。可调BEC需注意设置正确电压。')
        ttk.Separator(left, orient='horizontal').grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        ttk.Label(left, text='桨叶参数', font=('微软雅黑', 9, 'bold')).grid(row=row, column=0, columnspan=2, sticky='w', pady=2)
        row += 1
        add_field('桨叶尺寸', self.prop_size_var, ['5寸', '5.1寸', '6寸', '7寸', '8寸', '9寸', '10寸', '11寸', '12寸', '13寸', '15寸'],
            tooltip='桨叶直径。飞控无法获取。\n桨叶上通常印有规格(如"1045"=10寸4.5距)。\n⚠ 尺寸直接决定推力杠杆臂、盘面载荷、推力系数。')
        add_field('桨距(英寸)', self.prop_pitch_var,
            tooltip='桨叶螺距。桨叶规格中第二个数字(如1045中4.5)。\n桨叶上印刷或说明书。影响推力/功率曲线、螺旋桨效率。')
        add_field('桨叶数', self.prop_blades_var, ['2', '3', '4'],
            tooltip='桨叶片数。2叶效率最高，3/4叶振动小、响应快。\n目视确认。影响推力系数Kt、功率系数Kq、噪音。')
        add_field('桨叶材质', self.prop_material_var, ['碳纤维', '尼龙', 'PC', '木制', '复合材料'],
            tooltip='桨叶材质。碳纤维刚性好变形小、效率高；尼龙/PC韧性好抗摔；木制经典。\n目视/手感/说明书确认。影响共振频率、振动传递、断裂模式。')
        ttk.Separator(left, orient='horizontal').grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        ttk.Label(left, text='电池参数', font=('微软雅黑', 9, 'bold')).grid(row=row, column=0, columnspan=2, sticky='w', pady=2)
        row += 1
        add_field('电池S数', self.battery_var, ['4S', '6S', '8S', '12S'],
            tooltip='电池串数。飞控可通过电压估算：BATTERY_STATUS.voltage / 单节电压(~3.7V)。\n电池标签上标注(如6S)。决定系统电压、KV匹配、最大功率。')
        add_field('电池容量(mAh)', self.battery_capacity_var,
            tooltip='电池额定容量。飞控只能获取"已消耗容量"(mAh_consumed)，无法获取额定容量。\n电池标签标注(如5000mAh)。用于计算续航、C数换算、消耗百分比。')
        add_field('电池C数', self.battery_c_rating_var,
            tooltip='电池放电倍率。飞控无法获取。\n电池标签标注(如100C)。\n最大连续电流 = 容量(Ah) × C数。用于验证电池能否满足最大电流需求。\n⚠ 标称C数常虚标，建议留50%余量。')
        
        # 从飞控获取按钮
        btn_fc_fetch = ttk.Button(left, text='🔄 从飞控获取可用参数', command=self.on_fetch_from_fc)
        btn_fc_fetch.grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        ToolTip(btn_fc_fetch, '尝试从飞控读取可自动获取的参数：\n- 电池电压/电流 → 推算S数\n- ESC遥测 → 电机转速、电调温度、电流\n- 电池状态 → 电压、电流、已消耗容量\n- PX4参数 → 极对数(MOT_POLE_COUNT)\n⚠ 大部分配置参数(KV、桨距、C数等)飞控无法获取')
        row += 1
        
        ttk.Label(left, text='备注:').grid(row=row, column=0, sticky='nw', pady=2)
        self.notes_text = tk.Text(left, width=22, height=4)
        self.notes_text.grid(row=row, column=1, sticky='ew', pady=2)
        row += 1
        
        ttk.Separator(left, orient='horizontal').grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        
        # AI Provider / API Key
        if 'providers' in self.config:
            self.provider_state = self.config.get('providers') or {}
        else:
            # 旧配置只有单个 DeepSeek api_key/model
            self.provider_state = {}
            if self.config.get('api_key'):
                self.provider_state['deepseek'] = {
                    'key': self.config.get('api_key', ''),
                    'model': self.config.get('model', ''),
                    'base_url': self.config.get('base_url', '')
                }
        prov_pid = self.config.get('provider', 'deepseek')
        if prov_pid not in AI_PROVIDERS:
            prov_pid = 'deepseek'
        self._cur_provider = prov_pid
        prov = provider_info(prov_pid)
        prov_state = self.provider_state.get(prov_pid, {})

        ttk.Label(left, text='AI 服务商:').grid(row=row, column=0, sticky='w', pady=2)
        self.provider_var = tk.StringVar(value=prov['label'])
        self.provider_cb = ttk.Combobox(left, textvariable=self.provider_var, values=provider_labels(),
                                        width=22, state='readonly')
        self.provider_cb.grid(row=row, column=1, sticky='ew', pady=2)
        self.provider_cb.bind('<<ComboboxSelected>>', self.on_provider_change)
        row += 1

        self.api_key_label = ttk.Label(left, text=prov['label'] + ' API Key:')
        self.api_key_label.grid(row=row, column=0, sticky='w', pady=2)
        self.api_key_var = tk.StringVar(value=prov_state.get('key', ''))
        self.api_key_entry = ttk.Entry(left, textvariable=self.api_key_var, width=22, show='*')
        self.api_key_entry.grid(row=row, column=1, sticky='ew', pady=2)
        row += 1

        self.base_url_label = ttk.Label(left, text='Base URL:')
        self.base_url_label.grid(row=row, column=0, sticky='w', pady=2)
        self.base_url_var = tk.StringVar(value=prov_state.get('base_url') or prov['base_url'])
        self.base_url_entry = ttk.Entry(left, textvariable=self.base_url_var, width=22)
        self.base_url_entry.grid(row=row, column=1, sticky='ew', pady=2)
        ToolTip(self.base_url_label, '接口地址(不含 /chat/completions)。\n留空使用该服务商默认地址。\n可用于中转站、代理或本地 Ollama(如 http://localhost:11434/v1)。')
        ToolTip(self.base_url_entry, '接口地址(不含 /chat/completions)。\n留空使用该服务商默认地址。\n可用于中转站、代理或本地 Ollama(如 http://localhost:11434/v1)。')
        row += 1

        self.model_label = ttk.Label(left, text='模型:')
        self.model_label.grid(row=row, column=0, sticky='w', pady=2)
        self.model_var = tk.StringVar(value=prov_state.get('model') or (prov['models'][0] if prov['models'] else ''))
        self.model_cb = ttk.Combobox(left, textvariable=self.model_var, values=prov['models'], width=22)
        self.model_cb.grid(row=row, column=1, sticky='ew', pady=2)
        ToolTip(self.model_cb, '下拉选择常用模型，也可直接输入该平台的任意模型名。')
        row += 1

        ttk.Button(left, text='保存配置', command=self.save_config).grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)
        row += 1
        
        # Load last spec
        self.set_spec_from_dict(self.config.get('last_spec', {}))

        # Center: Notebook
        center = ttk.Frame(body)
        center.grid(row=0, column=1, sticky='nsew')
        center.columnconfigure(0, weight=1)
        center.rowconfigure(0, weight=1)
        
        self.nb = ttk.Notebook(center)
        self.nb.grid(row=0, column=0, sticky='nsew')
        
        self.build_tab_ai()
        self.build_tab_params()
        self.build_tab_takeoff()
        self.build_tab_history()
        self.build_tab_backup()
        self.build_tab_logs()

        # Right panel: Log
        right = ttk.LabelFrame(body, text='日志', padding=5)
        right.grid(row=0, column=2, sticky='nsew', padx=(5,0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        
        self.txt_log = tk.Text(right, state='disabled', wrap='none', bg='#1e1e1e', fg='#d4d4d4', font=('Consolas', 9))
        self.txt_log.grid(row=0, column=0, sticky='nsew')
        scroll_y = ttk.Scrollbar(right, orient='vertical', command=self.txt_log.yview)
        scroll_y.grid(row=0, column=1, sticky='ns')
        scroll_x = ttk.Scrollbar(right, orient='horizontal', command=self.txt_log.xview)
        scroll_x.grid(row=1, column=0, sticky='ew')
        self.txt_log.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        self.txt_log.tag_config('ok', foreground='#4ec9b0')
        self.txt_log.tag_config('err', foreground='#f44747')
        self.txt_log.tag_config('warn', foreground='#dcdcaa')
        self.txt_log.tag_config('info', foreground='#9cdcfe')

        # Restore last port
        if self.config.get('last_port'):
            self.port_var.set(self.config['last_port'])

    def build_tab_ai(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' AI建议 ')
        
        # Toolbar
        tb = ttk.Frame(f)
        tb.pack(fill='x', pady=(0,5))
        ttk.Button(tb, text='读取当前参数', command=self.on_read_params).pack(side='left', padx=2)
        ttk.Button(tb, text='AI 生成建议', command=self.on_ai_generate).pack(side='left', padx=2)
        ttk.Button(tb, text='应用勾选', command=self.on_apply_checked).pack(side='left', padx=2)
        
        # Treeview
        cols = ('name', 'current', 'ai', 'delta', 'check', 'desc')
        self.ai_tree = ttk.Treeview(f, columns=cols, show='headings', height=18)
        for c, w, t in (('name', 150, '参数名'), ('current', 80, '当前值'), ('ai', 80, 'AI建议'), ('delta', 70, '变化%'), ('check', 50, '勾选(点击)'), ('desc', 250, '中文说明')):
            self.ai_tree.heading(c, text=t)
            self.ai_tree.column(c, width=w, anchor='center' if c not in ('name', 'desc') else 'w')
        self.ai_tree.pack(fill='both', expand=True, pady=5)
        self.ai_tree.tag_configure('changed', background='#3a3a2a')
        self.ai_tree.bind('<Button-1>', lambda e: self.on_check_click(self.ai_tree, e, 5))
        
        # AI output
        ttk.Label(f, text='AI 分析:').pack(anchor='w')
        self.ai_text = tk.Text(f, height=6, wrap='word', bg='#1e1e1e', fg='#d4d4d4')
        self.ai_text.pack(fill='x', pady=2)

    def build_tab_params(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 当前参数 ')
        
        # Group checkboxes
        gf = ttk.Frame(f)
        gf.pack(fill='x', pady=(0,5))
        self.group_vars = {}
        for g in ('attitude', 'rate', 'filter'):
            v = tk.BooleanVar(value=True)
            self.group_vars[g] = v
            ttk.Checkbutton(gf, text=PARAM_GROUPS[g]['label'], variable=v).pack(side='left', padx=5)
        ttk.Button(gf, text='批量读取', command=self.on_read_all_params).pack(side='left', padx=10)
        
        cols = ('name', 'value', 'desc')
        self.param_tree = ttk.Treeview(f, columns=cols, show='headings', height=22)
        for c, w, t in (('name', 200, '参数名'), ('value', 100, '当前值'), ('desc', 300, '中文说明')):
            self.param_tree.heading(c, text=t)
            self.param_tree.column(c, width=w, anchor='center' if c != 'name' else 'w')
        self.param_tree.pack(fill='both', expand=True, pady=5)

    def build_tab_takeoff(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 起飞调参 ')

        tb = ttk.Frame(f)
        tb.pack(fill='x', pady=(0, 3))
        ttk.Label(tb, text='预设:').pack(side='left')
        self.preset_var = tk.StringVar()
        self.preset_cb = ttk.Combobox(
            tb, textvariable=self.preset_var,
            values=[p['label'] for p in TAKEOFF_PRESETS], state='readonly', width=42)
        self.preset_cb.pack(side='left', padx=5)
        self.preset_cb.bind('<<ComboboxSelected>>', self.on_preset_change)
        ttk.Button(tb, text='📖 完整帮助说明', command=self.on_takeoff_help).pack(side='left', padx=5)
        ttk.Button(tb, text='读取当前值', command=self.on_takeoff_read).pack(side='left', padx=5)
        self.takeoff_status = ttk.Label(tb, text='未连接', foreground='gray')
        self.takeoff_status.pack(side='right', padx=5)

        tb2 = ttk.Frame(f)
        tb2.pack(fill='x', pady=(0, 3))
        ttk.Button(tb2, text='全选', command=lambda: self._takeoff_check_all(True)).pack(side='left', padx=3)
        ttk.Button(tb2, text='全不选', command=lambda: self._takeoff_check_all(False)).pack(side='left', padx=3)
        ttk.Button(tb2, text='只勾选与推荐值不同的项', command=self._takeoff_check_diff).pack(side='left', padx=3)
        self.takeoff_only_diff = tk.BooleanVar(value=True)
        ttk.Checkbutton(tb2, text='应用时跳过已一致的项',
                        variable=self.takeoff_only_diff).pack(side='left', padx=8)
        ttk.Button(tb2, text='应用勾选（先自动备份）',
                   command=self.on_takeoff_apply).pack(side='right', padx=3)

        cols = ('check', 'name', 'rec', 'cur', 'delta', 'unit', 'desc')
        self.takeoff_tree = ttk.Treeview(f, columns=cols, show='headings', height=14)
        for c, w, t in (('check', 56, '勾选'), ('name', 150, '参数名'),
                        ('rec', 110, '推荐值(双击改)'), ('cur', 90, '当前值'),
                        ('delta', 90, '差异'), ('unit', 70, '单位'),
                        ('desc', 330, '作用')):
            self.takeoff_tree.heading(c, text=t)
            self.takeoff_tree.column(c, width=w, anchor='w' if c in ('name', 'desc') else 'center')
        self.takeoff_tree.pack(fill='both', expand=True, pady=3)
        self.takeoff_tree.tag_configure('same', background='#1e3a2a')
        self.takeoff_tree.tag_configure('diff', background='#3a3a2a')
        self.takeoff_tree.tag_configure('missing', background='#3a1e1e')
        self.takeoff_tree.bind('<Button-1>', lambda e: self.on_check_click(self.takeoff_tree, e, 1))
        self.takeoff_tree.bind('<Double-1>', self.on_takeoff_edit)
        self.takeoff_tree.bind('<<TreeviewSelect>>', self.on_takeoff_select)

        ttk.Label(f, text='帮助说明（选中参数显示其用法；右上"📖 完整帮助说明"看全流程）:').pack(anchor='w')
        self.takeoff_help = tk.Text(f, height=7, wrap='word', bg='#1e1e1e', fg='#d4d4d4')
        self.takeoff_help.pack(fill='x', pady=2)

        self.takeoff_rows = []
        self.takeoff_preset = None
        self.preset_var.set(TAKEOFF_PRESETS[0]['label'])
        self._fill_takeoff_tree()

    def _fill_takeoff_tree(self):
        label = self.preset_var.get()
        preset = next((p for p in TAKEOFF_PRESETS if p['label'] == label),
                      TAKEOFF_PRESETS[0])
        self.takeoff_preset = preset
        self.takeoff_tree.delete(*self.takeoff_tree.get_children())
        self.takeoff_rows = []
        for e in preset['params']:
            iid = self.takeoff_tree.insert(
                '', 'end',
                values=('', e['name'], self._fmt_toff(e['value']), '—', '—',
                        e.get('unit') or '', e['desc']))
            self.takeoff_rows.append(dict(iid=iid, entry=e, name=e['name'],
                                          scale=1.0, current=None))
        self._set_takeoff_help('%s\n\n%s' % (preset['note'],
                                             '双击"推荐值"单元格可直接改值；'
                                             '点"读取当前值"后会自动算出差异并按需勾选。'))
        self._update_takeoff_status()

    def _update_takeoff_status(self):
        if not getattr(self, 'takeoff_status', None):
            return
        if not self.connected:
            self.takeoff_status.configure(text='未连接', foreground='gray')
            return
        fc = self.fc_type or '未识别'
        want = self.takeoff_preset['fc'] if self.takeoff_preset else ''
        ok = (want == fc)
        self.takeoff_status.configure(
            text=f'飞控 {fc} / 预设 {want}' + ('  ✓' if ok else '  ⚠ 可能不匹配'),
            foreground=('green' if ok else 'orange'))

    def _fmt_toff(self, v):
        try:
            v = float(v)
        except (TypeError, ValueError):
            return str(v)
        if v == int(v) and abs(v) < 1e6:
            return str(int(v))
        return f'{v:.4g}'

    def _set_takeoff_help(self, text):
        self.takeoff_help.configure(state='normal')
        self.takeoff_help.delete('1.0', 'end')
        self.takeoff_help.insert('1.0', text)
        self.takeoff_help.configure(state='disabled')

    def on_preset_change(self, event=None):
        self._fill_takeoff_tree()

    def on_takeoff_select(self, event=None):
        sel = self.takeoff_tree.selection()
        if not sel or not self.takeoff_preset:
            return
        iid = sel[0]
        row = next((r for r in self.takeoff_rows if r['iid'] == iid), None)
        if not row:
            return
        e = row['entry']
        rng = e.get('rng')
        lines = [f"【{e['name']}】{e['desc']}",
                 f"推荐值 {self._fmt_toff(e['value'])} {e.get('unit') or ''}   "
                 f"固件范围 {('%s ~ %s' % (rng[0], rng[1])) if rng else '未标注'}"]
        if row.get('current') is not None:
            lines.append(f"当前值 {self._fmt_toff(row['current'])}"
                         f"（实际写入名 {row['name']}，系数 {row['scale']:g}）")
        elif row.get('resolved'):
            lines.append(f"当前值 读取失败（尝试名 {row['name']}）")
        lines.append('')
        lines.append(e['help'])
        self._set_takeoff_help('\n'.join(lines))

    def on_takeoff_edit(self, event=None):
        col = self.takeoff_tree.identify_column(event.x)
        if col != '#3':
            return
        iid = self.takeoff_tree.identify_row(event.y)
        if not iid:
            return
        row = next((r for r in self.takeoff_rows if r['iid'] == iid), None)
        if not row:
            return
        e = row['entry']
        rng = e.get('rng')
        try:
            cur = float(self.takeoff_tree.set(iid, 'rec'))
        except (TypeError, ValueError):
            cur = e['value']
        hint = f"{e['name']} （单位 {e.get('unit') or '无'}）"
        if rng:
            hint += f"\n固件合法范围: {rng[0]} ~ {rng[1]}"
        hint += f"\n\n{e['help']}"
        v = simpledialog.askfloat('编辑推荐值', hint, initialvalue=cur,
                                  minvalue=(rng[0] if rng else -1e9),
                                  maxvalue=(rng[1] if rng else 1e9), parent=self)
        if v is None:
            return
        self.takeoff_tree.set(iid, 'rec', self._fmt_toff(v))
        self._update_takeoff_delta(iid)

    def _takeoff_parse_rec(self, iid):
        try:
            return float(self.takeoff_tree.set(iid, 'rec'))
        except (TypeError, ValueError):
            return None

    def _update_takeoff_delta(self, iid, cur=None):
        row = next((r for r in self.takeoff_rows if r['iid'] == iid), None)
        if not row:
            return
        rec = self._takeoff_parse_rec(iid)
        if rec is None:
            self.takeoff_tree.set(iid, 'delta', '—')
            return
        if cur is None:
            cur = row.get('current')
        if cur is None:
            self.takeoff_tree.set(iid, 'delta', '未读取')
            self.takeoff_tree.item(iid, tags=('missing',))
            return
        if abs(cur) < 1e-9:
            self.takeoff_tree.set(iid, 'delta', '—')
            self.takeoff_tree.item(iid, tags=('diff',))
            return
        if abs(rec - cur) <= max(1e-6, abs(rec) * 0.005):
            self.takeoff_tree.set(iid, 'delta', '一致')
            self.takeoff_tree.item(iid, tags=('same',))
            return
        self.takeoff_tree.set(iid, 'delta', f'{(rec - cur) / cur * 100:+.1f}%')
        self.takeoff_tree.item(iid, tags=('diff',))

    def _takeoff_check_all(self, flag):
        for r in self.takeoff_rows:
            self.takeoff_tree.set(r['iid'], 'check', '✓' if flag else '')

    def _takeoff_check_diff(self):
        n = 0
        for r in self.takeoff_rows:
            rec = self._takeoff_parse_rec(r['iid'])
            cur = r.get('current')
            if rec is None:
                continue
            diff = (cur is None or abs(cur) < 1e-9 or
                    abs(rec - cur) > max(1e-6, abs(rec) * 0.005))
            self.takeoff_tree.set(r['iid'], 'check', '✓' if diff else '')
            n += 1 if diff else 0
        self.log(f'已勾选 {n} 项与推荐值不同的参数')

    def on_takeoff_help(self):
        win = tk.Toplevel(self)
        win.title('起飞调参 · 完整帮助说明')
        win.geometry('760x720')
        txt = tk.Text(win, wrap='word', bg='#1e1e1e', fg='#d4d4d4',
                      font=('Consolas', 10))
        sb = ttk.Scrollbar(win, orient='vertical', command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        txt.pack(side='left', fill='both', expand=True, padx=5, pady=5)
        txt.insert('1.0', TAKEOFF_HELP)
        txt.configure(state='disabled')
        ttk.Button(win, text='关闭', command=win.destroy).pack(side='bottom', pady=5)

    # ===== 起飞调参：读取 / 应用 =====
    def _takeoff_check_fc(self):
        if not self.takeoff_preset:
            return False
        want = self.takeoff_preset['fc']
        if self.fc_type and self.fc_type != want:
            return messagebox.askyesno(
                '飞控类型不匹配',
                f'当前预设「{self.takeoff_preset["label"]}」适用于 {want}，'
                f'而连接的是 {self.fc_type}。\n参数名与单位都不同，写入多半会失败。仍要继续？')
        return True

    def _resolve_param(self, entry, timeout=0.9):
        """按 原名→4.7新名 顺序探测，返回归一化(旧单位)的当前值。"""
        cands = [(entry['name'], 1.0)] + [(n, s) for n, s in entry.get('aliases', [])]
        first = None
        for name, scale in cands:
            v = get_param(self.mav_master, name, timeout=timeout)
            if v is None:
                continue
            if first is None:
                first = (name, v, scale)
            if abs(v) > 1e-9:
                return dict(name=name, scale=scale, value=v / scale)
        if first:
            name, v, scale = first
            return dict(name=name, scale=scale, value=v / scale)
        return dict(name=entry['name'], scale=1.0, value=None)

    def on_takeoff_read(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if not self._takeoff_check_fc():
            return
        n = len(self.takeoff_rows)
        self.log(f'读取起飞预设参数 {n} 项...')
        self.enqueue('读取起飞参数', self._do_takeoff_read)

    def _do_takeoff_read(self):
        out = []
        for i, r in enumerate(self.takeoff_rows):
            res = self._resolve_param(r['entry'])
            out.append(res)
            if res['value'] is None:
                self.log(f'  [{i+1}/{len(self.takeoff_rows)}] {r["entry"]["name"]} 读取失败')
            else:
                self.log(f'  [{i+1}/{len(self.takeoff_rows)}] {r["name"]} = '
                         f'{self._fmt_toff(res["value"])} {r["entry"].get("unit") or ""}')
        self.after(0, lambda: self._show_takeoff_read(out))

    def _show_takeoff_read(self, out):
        for r, res in zip(self.takeoff_rows, out):
            r['name'] = res['name']
            r['scale'] = res['scale']
            r['current'] = res['value']
            cur_txt = '—' if res['value'] is None else self._fmt_toff(res['value'])
            if res['value'] is not None and res['name'] != r['entry']['name']:
                cur_txt += f' ({res["name"]})'
            self.takeoff_tree.set(r['iid'], 'cur', cur_txt)
            self._update_takeoff_delta(r['iid'])
            if res['value'] is None:
                self.takeoff_tree.set(r['iid'], 'check', '')
        self._takeoff_check_diff()
        self.log('✅ 起飞预设参数读取完成')

    def on_takeoff_apply(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if not self._takeoff_check_fc():
            return
        items = []
        for r in self.takeoff_rows:
            if not _is_checked(self.takeoff_tree.set(r['iid'], 'check')):
                continue
            rec = self._takeoff_parse_rec(r['iid'])
            if rec is None:
                messagebox.showwarning('提示', f'{r["entry"]["name"]} 的推荐值不是有效数字')
                return
            items.append((r, rec))
        if not items:
            messagebox.showinfo('提示', '没有勾选任何参数')
            return
        issues = []
        for r, rec in items:
            rng = r['entry'].get('rng')
            if rng and not (rng[0] <= rec <= rng[1]):
                issues.append(f'{r["entry"]["name"]}={rec} 超出固件范围 {rng[0]}~{rng[1]}')
        values = {}
        for r, rec in items:
            values[r['entry']['name']] = rec
            if r['entry']['name'] != r['name']:
                values[r['name']] = rec * r['scale']
        arm, mn = values.get('MOT_SPIN_ARM'), values.get('MOT_SPIN_MIN')
        if arm is not None and mn is not None and mn < arm + 0.03 - 1e-9:
            issues.append(f'MOT_SPIN_MIN({mn}) 必须 ≥ MOT_SPIN_ARM({arm}) + 0.03')
        alt, rtl = values.get('FENCE_ALT_MAX'), values.get('RTL_ALT')
        if alt is not None and rtl is not None and alt < rtl / 100.0 - 1e-9:
            issues.append(f'FENCE_ALT_MAX({alt}m) 必须 ≥ RTL_ALT({rtl}cm = {rtl/100:.2f}m)，'
                          '否则返航爬升会再次越界')
        if issues:
            if not messagebox.askyesno('参数校验未通过', '\n'.join(issues) +
                                       '\n\n仍要写入吗？（不推荐）'):
                return
        skip = self.takeoff_only_diff.get()
        pending = []
        for r, rec in items:
            cur = r.get('current')
            if (skip and cur is not None and rec is not None
                    and abs(rec - cur) <= max(1e-6, abs(rec) * 0.005)):
                continue
            pending.append((r, rec))
        if not pending:
            messagebox.showinfo('提示', '所有勾选项当前值已与推荐值一致，无需写入')
            return
        label = self.takeoff_preset['label']
        listing = '\n'.join(
            f'  {r["entry"]["name"]} → {self._fmt_toff(rec)} {r["entry"].get("unit") or ""}'
            for r, rec in pending)
        if not messagebox.askyesno(
                '确认写入起飞预设',
                f'预设: {label}\n将写入 {len(pending)} 个参数（写入前自动备份）:\n{listing}\n\n继续？'):
            return
        self.log(f'开始应用起飞预设「{label}」，共 {len(pending)} 项...')
        self.enqueue('应用起飞预设',
                     lambda: self._do_takeoff_apply(pending, label))

    def _do_takeoff_apply(self, pending, label):
        names = [r['name'] for r, _ in pending]
        backup_path = backup_params(self.mav_master, names, self.get_spec_dict(),
                                    tag='takeoff')
        self.log(f'📦 已备份: {os.path.basename(backup_path)}')
        success, failed = 0, []
        for r, rec in pending:
            e = r['entry']
            val = rec * r['scale']
            if set_param(self.mav_master, r['name'], val):
                success += 1
                extra = '' if abs(val - rec) < 1e-9 else f' (原 {self._fmt_toff(rec)}{e.get("unit") or ""})'
                self.log(f'  ✅ {r["name"]} = {self._fmt_toff(val)}{extra}')
            else:
                failed.append(r['name'])
                self.log(f'  ❌ {r["name"]} 写入失败')
        self.after(0, lambda: self._after_takeoff_apply(success, failed,
                                                        backup_path, label))

    def _after_takeoff_apply(self, success, failed, backup_path, label):
        if failed:
            messagebox.showwarning('结果',
                                   f'成功 {success} 个，失败 {len(failed)} 个: {", ".join(failed)}')
        else:
            messagebox.showinfo('结果', f'起飞预设「{label}」全部 {success} 个参数写入成功')
        self.round += 1
        self.history.append({
            'round': self.round,
            'time': time.strftime('%H:%M:%S'),
            'written': success,
            'summary': f'起飞预设: {label}',
            'backup': os.path.basename(backup_path)
        })
        self.refresh_history()
        if not failed:
            self.on_takeoff_read()

    def build_tab_history(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 试飞采集 ')
        
        # Recording controls
        rf = ttk.LabelFrame(f, text='数据采集', padding=5)
        rf.pack(fill='x', pady=5)
        self.btn_start_rec = ttk.Button(rf, text='开始记录', command=self.on_start_recording)
        self.btn_start_rec.pack(side='left', padx=5)
        self.btn_stop_rec = ttk.Button(rf, text='停止记录', command=self.on_stop_recording, state='disabled')
        self.btn_stop_rec.pack(side='left', padx=5)
        self.rec_status = ttk.Label(rf, text='未记录', foreground='gray')
        self.rec_status.pack(side='left', padx=10)
        
        # Metrics display
        mf = ttk.LabelFrame(f, text='分析指标', padding=5)
        mf.pack(fill='x', pady=5)
        self.metrics_text = tk.Text(mf, height=8, wrap='word', bg='#1e1e1e', fg='#d4d4d4', font=('Consolas', 9))
        self.metrics_text.pack(fill='x', padx=5, pady=5)
        
        # Step excitation
        sf = ttk.LabelFrame(f, text='自动阶跃激励 (需解锁+定点+高度≥2m+GPS良好)', padding=5)
        sf.pack(fill='x', pady=5)
        ttk.Label(sf, text='轴:').pack(side='left')
        self.step_axis_var = tk.StringVar(value='roll')
        ttk.Combobox(sf, textvariable=self.step_axis_var, values=['roll', 'pitch'], width=8, state='readonly').pack(side='left', padx=5)
        ttk.Label(sf, text='幅度(度):').pack(side='left')
        self.step_amp_var = tk.StringVar(value='10')
        ttk.Entry(sf, textvariable=self.step_amp_var, width=6).pack(side='left', padx=5)
        ttk.Label(sf, text='周期:').pack(side='left')
        self.step_cycles_var = tk.StringVar(value='3')
        ttk.Entry(sf, textvariable=self.step_cycles_var, width=4).pack(side='left', padx=5)
        self.btn_step = ttk.Button(sf, text='执行阶跃激励', command=self.on_step_excitation)
        self.btn_step.pack(side='left', padx=10)
        self.step_status = ttk.Label(sf, text='', foreground='gray')
        self.step_status.pack(side='left', padx=5)
        
        # Feedback text
        tf = ttk.Frame(f)
        tf.pack(fill='x', pady=5)
        ttk.Label(tf, text='试飞现象描述:').pack(side='left')
        self.feedback_text = tk.Text(tf, width=60, height=3)
        self.feedback_text.pack(side='left', padx=5, fill='x', expand=True)
        ttk.Button(tf, text='发起下一轮 AI 迭代', command=self.on_next_iteration).pack(side='left', padx=5)
        
        # Iteration history
        cols = ('round', 'time', 'written', 'summary', 'backup')
        self.hist_tree = ttk.Treeview(f, columns=cols, show='headings', height=10)
        for c, w, t in (('round', 60, '轮次'), ('time', 120, '时间'), ('written', 80, '写入数'), ('summary', 300, 'AI摘要'), ('backup', 200, '备份文件')):
            self.hist_tree.heading(c, text=t)
            self.hist_tree.column(c, width=w, anchor='center' if c != 'summary' else 'w')
        self.hist_tree.pack(fill='both', expand=True, pady=5)

    def build_tab_backup(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 备份管理 ')
        
        ttk.Button(f, text='刷新列表', command=self.refresh_backups).pack(anchor='w', pady=5)
        ttk.Button(f, text='恢复选中', command=self.on_restore_backup).pack(anchor='w', pady=5)
        
        cols = ('file', 'time', 'fc', 'count')
        self.backup_tree = ttk.Treeview(f, columns=cols, show='headings', height=20)
        for c, w, t in (('file', 250, '文件名'), ('time', 120, '时间'), ('fc', 80, '飞控类型'), ('count', 80, '参数数')):
            self.backup_tree.heading(c, text=t)
            self.backup_tree.column(c, width=w, anchor='center' if c != 'file' else 'w')
        self.backup_tree.pack(fill='both', expand=True, pady=5)
        self.refresh_backups()

    def build_tab_logs(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 日志管理 ')
        
        # Toolbar
        tb = ttk.Frame(f)
        tb.pack(fill='x', pady=5)
        ttk.Button(tb, text='刷新日志列表', command=self.on_refresh_logs).pack(side='left', padx=5)
        ttk.Button(tb, text='下载选中', command=self.on_download_logs).pack(side='left', padx=5)
        ttk.Button(tb, text='清除所有日志', command=self.on_erase_logs).pack(side='left', padx=5)
        self.logs_status = ttk.Label(tb, text='未获取', foreground='gray')
        self.logs_status.pack(side='left', padx=10)
        
        # Progress
        pf = ttk.Frame(f)
        pf.pack(fill='x', pady=5)
        self.log_progress = ttk.Progressbar(pf, mode='determinate')
        self.log_progress.pack(fill='x', padx=5, pady=5)
        self.log_progress_label = ttk.Label(pf, text='')
        self.log_progress_label.pack()
        
        # Log list
        cols = ('id', 'size', 'time_utc', 'select')
        self.logs_tree = ttk.Treeview(f, columns=cols, show='headings', height=18)
        for c, w, t in (('id', 80, '日志ID'), ('size', 100, '大小'), ('time_utc', 180, '时间(UTC)'), ('select', 60, '选择(点击)')):
            self.logs_tree.heading(c, text=t)
            self.logs_tree.column(c, width=w, anchor='center')
        self.logs_tree.pack(fill='both', expand=True, pady=5)
        self.logs_tree.bind('<Button-1>', lambda e: self.on_check_click(self.logs_tree, e, 4))
        
        # Download options
        df = ttk.LabelFrame(f, text='下载选项', padding=5)
        df.pack(fill='x', pady=5)
        ttk.Label(df, text='保存目录:').pack(side='left')
        self.log_dir_var = tk.StringVar(value=os.path.join(os.path.dirname(__file__), 'flight_logs'))
        ttk.Entry(df, textvariable=self.log_dir_var, width=50).pack(side='left', padx=5)
        ttk.Button(df, text='浏览', command=self.on_browse_log_dir).pack(side='left', padx=5)
        self.auto_parse_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(df, text='下载后自动解析(CSV)', variable=self.auto_parse_var).pack(side='left', padx=10)
        self.ai_analyze_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(df, text='解析后自动AI分析', variable=self.ai_analyze_var).pack(side='left', padx=10)
        
        # AI Analysis output
        af = ttk.LabelFrame(f, text='AI 分析结果', padding=5)
        af.pack(fill='both', expand=True, pady=5)
        self.log_ai_text = tk.Text(af, height=8, wrap='word', bg='#1e1e1e', fg='#d4d4d4', font=('Consolas', 9))
        self.log_ai_text.pack(fill='both', expand=True, padx=5, pady=5)

    # ===== Port =====
    def refresh_ports(self):
        try:
            import serial.tools.list_ports as lp
            ports = [p.device for p in lp.comports()]
            self.port['values'] = ports
            if ports:
                last = self.config.get('last_port', '')
                if last in ports:
                    self.port_var.set(last)
                else:
                    self.port.current(0)
            self.log(f'串口列表已刷新，共 {len(ports)} 个')
        except Exception as e:
            self.log(f'刷新串口失败: {e}')

    # ===== Connect / Disconnect =====
    def on_connect(self):
        if self.connected:
            return
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showwarning('提示', '请选择串口')
            return
        self.btn_conn.configure(state='disabled')
        self.link_lbl.configure(text='连接中...', foreground='orange')
        self.log(f'正在连接 {port} @ {baud}...')
        
        def worker():
            master, fc_type = connect_port(port, baud)
            self.after(0, lambda: self._after_connect(master, fc_type))
        
        threading.Thread(target=worker, daemon=True).start()

    def _after_connect(self, master, fc_type):
        if master:
            self.mav_master = master
            # Respect user's FC type selection if not "自动识别"
            user_fc = self.fc_type_var.get()
            if user_fc == 'APM Copter':
                self.fc_type = 'APM_COPTER'
            elif user_fc == 'PX4 MC':
                self.fc_type = 'PX4_MC'
            else:
                self.fc_type = fc_type
            self.connected = True
            self.btn_conn.configure(state='disabled')
            self.btn_disc.configure(state='normal')
            self.fc_lbl.configure(text=f'FC: {self.fc_type}')
            self.link_lbl.configure(text='● 已连接', foreground='green')
            self.log(f'✅ 已连接 {self.fc_type} (sysid={master.target_system})')
            self._update_takeoff_status()
            self.config['last_port'] = self.port_var.get()
            self.config['baud'] = int(self.baud_var.get())
            self.save_config()
            # Auto-read params
            self.on_read_params()
        else:
            self.connected = False
            self.btn_conn.configure(state='normal')
            self.link_lbl.configure(text='连接失败', foreground='red')
            self.log('❌ 连接失败：未收到心跳，请检查串口/波特率/飞控供电')

    def on_disconnect(self):
        if not self.connected:
            return
        self.connected = False
        self.btn_conn.configure(state='normal')
        self.btn_disc.configure(state='disabled')
        self.fc_lbl.configure(text='FC: AUTO')
        self.link_lbl.configure(text='○ 未连接', foreground='gray')
        if self.mav_master:
            disconnect(self.mav_master)
            self.mav_master = None
            self.fc_type = None
        self._update_takeoff_status()
        self.log('已断开连接')

    # ===== Parameter operations =====
    def on_read_params(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        names = get_param_names_for_groups(self.fc_type, [g for g, v in self.group_vars.items() if v.get()])
        self.param_names = names
        self.log(f'开始读取 {len(names)} 个参数...')
        self.enqueue('读取参数', lambda: self._do_read_params(names))

    def on_fetch_from_fc(self):
        """从飞控自动获取可读取的参数并填充表单"""
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        self.log('正在从飞控读取可用参数...')
        self.enqueue('从飞控获取参数', self._do_fetch_from_fc)

    def _do_fetch_from_fc(self):
        """在工作线程中从飞控读取各种状态数据"""
        try:
            master = self.master
            fetched = {}
            
            # 1. 读取电池状态 (BATTERY_STATUS / SYS_STATUS)
            bat = master.recv_match(type=['BATTERY_STATUS', 'SYS_STATUS'], blocking=True, timeout=3)
            if bat:
                if bat.get_type() == 'BATTERY_STATUS':
                    volt = bat.voltages[0] / 1000.0 if bat.voltages and bat.voltages[0] > 0 else bat.voltage_battery / 1000.0
                    curr = bat.current_battery / 100.0 if bat.current_battery != -1 else 0
                    consumed = bat.charge_consumed / 1000.0 if bat.charge_consumed != -1 else 0
                else:  # SYS_STATUS
                    volt = bat.voltage_battery / 1000.0
                    curr = bat.current_battery / 100.0
                    consumed = 0
                
                if volt > 0:
                    fetched['battery_voltage'] = volt
                    # 估算 S 数: 每节 ~3.7V (满电4.2V, 空3.0V)
                    s_est = round(volt / 3.7)
                    if 3 <= s_est <= 14:
                        fetched['battery_s'] = f'{s_est}S'
                    self.log(f'  电池电压: {volt:.2f}V → 推算 {s_est}S')
                
                if curr > 0:
                    fetched['battery_current'] = curr
                if consumed > 0:
                    fetched['battery_consumed_mah'] = consumed
            
            # 2. 读取 ESC 遥测 (ESC_TELEMETRY_1_TO_4)
            esc_data = []
            for i in range(4):
                esc = master.recv_match(type='ESC_TELEMETRY_1_TO_4', blocking=True, timeout=1)
                if esc and esc.rpm[i] > 0:
                    esc_data.append({
                        'rpm': esc.rpm[i],
                        'voltage': esc.voltage[i] / 100.0 if esc.voltage[i] > 0 else 0,
                        'current': esc.current[i] / 100.0 if esc.current[i] > 0 else 0,
                        'temp': esc.temperature[i] / 100.0 if esc.temperature[i] > 0 else 0,
                    })
            if esc_data:
                fetched['esc_telemetry'] = esc_data
                rpm_avg = sum(d['rpm'] for d in esc_data) / len(esc_data)
                self.log(f'  ESC遥测: {len(esc_data)}个电调, 平均RPM={rpm_avg:.0f}')
            
            # 3. PX4 特有参数: 极对数
            if self.fc_type == 'PX4_MC':
                try:
                    pole = get_param(master, 'MOT_POLE_COUNT')
                    if pole:
                        fetched['motor_pole_pairs'] = int(pole)
                        self.log(f'  PX4极对数: {int(pole)}')
                except Exception:
                    pass
            
            # 4. GPS 状态
            gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
            if gps and gps.fix_type >= 3:
                fetched['gps_fix'] = gps.fix_type
                self.log(f'  GPS定位: {gps.fix_type}D')
            
            # 5. 飞行模式
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                fetched['flight_mode'] = hb.custom_mode
            
            # 更新 UI (主线程)
            self.after(0, lambda: self._apply_fetched_params(fetched))
            
        except Exception as e:
            self.after(0, lambda: self.log(f'❌ 从飞控获取参数异常: {e}'))

    def _apply_fetched_params(self, fetched):
        """将获取到的参数应用到表单"""
        updated = []
        if 'battery_s' in fetched:
            self.battery_var.set(fetched['battery_s'])
            updated.append(f'电池S数: {fetched["battery_s"]}')
        if 'battery_voltage' in fetched:
            updated.append(f'电池电压: {fetched["battery_voltage"]:.2f}V')
        if 'battery_current' in fetched:
            updated.append(f'电池电流: {fetched["battery_current"]:.2f}A')
        if 'battery_consumed_mah' in fetched:
            updated.append(f'已消耗: {fetched["battery_consumed_mah"]:.0f}mAh')
        if 'esc_telemetry' in fetched:
            esc = fetched['esc_telemetry']
            rpm = [d['rpm'] for d in esc if d['rpm'] > 0]
            if rpm:
                updated.append(f'电机RPM: {min(rpm)}-{max(rpm)} (平均{sum(rpm)/len(rpm):.0f})')
        if 'motor_pole_pairs' in fetched:
            # 极对数转极数描述
            pairs = fetched['motor_pole_pairs']
            self.motor_pole_var.set(f'{pairs*2}N{pairs*2+2}P' if pairs == 7 else f'{pairs*2}极')
            updated.append(f'电机极对数: {pairs}对 ({pairs*2}极)')
        if 'gps_fix' in fetched:
            updated.append(f'GPS: {fetched["gps_fix"]}D定位')
        
        if updated:
            self.log('✅ 从飞控获取参数: ' + '; '.join(updated))
            messagebox.showinfo('获取完成', '已填充可自动获取的参数:\n' + '\n'.join(updated))
        else:
            self.log('⚠️ 未获取到可用参数 (可能飞控不支持相关消息或未启用遥测)')
            messagebox.showinfo('提示', '未获取到可用参数\n可能原因: 飞控不支持相关消息、未启用ESC遥测、电池监测未配置')

    def _do_read_params(self, names):
        result = read_params(self.mav_master, names, on_progress=lambda i, n, name, val: self.log(f'  [{i}/{n}] {name}={val}'))
        self.after(0, lambda: self._show_params(result))

    def _show_params(self, params):
        self.param_tree.delete(*self.param_tree.get_children())
        self.ai_tree.delete(*self.ai_tree.get_children())
        param_set = PARAM_SETS.get(self.fc_type, {})
        for name, val in params.items():
            desc = param_set.get(name, {}).get('desc', '')
            self.param_tree.insert('', 'end', values=(name, f'{val:.4f}', desc))
            self.ai_tree.insert('', 'end', values=(name, f'{val:.4f}', '', '', '', desc))
        self.log(f'✅ 已读取 {len(params)} 个参数')

    def on_read_all_params(self):
        self.on_read_params()

    def on_ai_generate(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        settings = self.check_ai_settings()
        if not settings:
            return
        if not self.param_names:
            messagebox.showwarning('提示', '请先读取当前参数')
            return
        self.log('正在生成 AI 调参建议...')
        self.enqueue('AI生成', lambda: self._do_ai_generate(settings))

    def _do_ai_generate(self, settings):
        # Build current params dict
        current = {}
        for item in self.ai_tree.get_children():
            vals = self.ai_tree.item(item)['values']
            if vals:
                current[vals[0]] = float(vals[1])
        
        spec = self.get_spec_dict()
        selected_groups = [g for g, v in self.group_vars.items() if v.get()]
        feedback = self.feedback_text.get('1.0', 'end-1c').strip()
        
        prompt = build_prompt(spec, current, self.fc_type, selected_groups, feedback, self.history)
        pid, api_key, base_url, model = settings
        self.log(f'调用 {provider_info(pid)["label"]} / {model} ...')
        content, err = ai_chat(api_key, SYSTEM_PROMPT, prompt, model=model,
                               provider=pid, base_url=base_url)
        
        if err:
            self.after(0, lambda: self.log(f'❌ AI 调用失败: {err}'))
            return
        
        parsed = extract_json(content)
        if not parsed:
            self.after(0, lambda: self.log('❌ AI 返回格式错误，无法解析 JSON'))
            self.after(0, lambda: self.ai_text.delete('1.0', 'end'))
            self.after(0, lambda: self.ai_text.insert('1.0', content))
            return
        
        ai_params = parsed.get('params', {})
        valid_params, warnings = validate_ai_params(ai_params, self.fc_type, selected_groups)
        
        # Update AI tree
        self.after(0, lambda: self._update_ai_tree(valid_params, warnings))
        self.after(0, lambda: self._show_ai_result(parsed))

    def _update_ai_tree(self, valid_params, warnings):
        for item in self.ai_tree.get_children():
            vals = list(self.ai_tree.item(item)['values'])
            name = vals[0]
            if name in valid_params:
                cur = float(vals[1])
                new = valid_params[name]
                delta = ((new - cur) / cur * 100) if cur != 0 else 0
                vals[2] = f'{new:.4f}'
                vals[3] = f'{delta:+.1f}%'
                vals[4] = '✓'
                self.ai_tree.item(item, values=vals, tags=('changed',))
            else:
                vals[2] = ''
                vals[3] = ''
                vals[4] = ''
                self.ai_tree.item(item, values=vals, tags=())
        for w in warnings:
            self.log(f'⚠️ {w}')

    def _show_ai_result(self, parsed):
        self.ai_text.delete('1.0', 'end')
        self.ai_text.insert('1.0', f"摘要: {parsed.get('summary', '')}\n\n")
        self.ai_text.insert('end', f"理由: {parsed.get('reasoning', '')}\n\n")
        if parsed.get('warnings'):
            self.ai_text.insert('end', "风险提示:\n" + '\n'.join(f'  - {w}' for w in parsed['warnings']))

    def on_apply_checked(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        checked = []
        for item in self.ai_tree.get_children():
            vals = self.ai_tree.item(item)['values']
            if vals and _is_checked(vals[4]) and vals[2]:
                try:
                    checked.append((vals[0], float(vals[2])))
                except (ValueError, TypeError):
                    pass
        if not checked:
            messagebox.showinfo('提示', '没有勾选任何参数，或勾选项缺少 AI 建议值')
            return
        if not messagebox.askyesno('确认', f'将写入 {len(checked)} 个参数，并自动备份当前值。\n继续？'):
            return
        self.log(f'开始应用 {len(checked)} 个参数...')
        self.enqueue('应用参数', lambda: self._do_apply(checked))

    def _do_apply(self, checked):
        # Backup first
        names = [n for n, _ in checked]
        spec = self.get_spec_dict()
        backup_path = backup_params(self.mav_master, names, spec, tag=f'round{self.round+1}')
        self.log(f'📦 已备份: {os.path.basename(backup_path)}')
        
        success = 0
        failed = []
        for name, val in checked:
            if set_param(self.mav_master, name, val):
                success += 1
                self.log(f'  ✅ {name} = {val:.4f}')
            else:
                failed.append(name)
                self.log(f'  ❌ {name} 写入失败')
        
        self.after(0, lambda: self._after_apply(success, failed, backup_path, checked))

    def _after_apply(self, success, failed, backup_path, checked):
        if failed:
            messagebox.showwarning('结果', f'成功 {success} 个，失败 {len(failed)} 个: {", ".join(failed)}')
        else:
            messagebox.showinfo('结果', f'全部 {success} 个参数写入成功')
        self.round += 1
        hist_entry = {
            'round': self.round,
            'time': time.strftime('%H:%M:%S'),
            'written': success,
            'summary': '应用参数',
            'backup': os.path.basename(backup_path)
        }
        self.history.append(hist_entry)
        self.refresh_history()
        # Refresh current values
        self.on_read_params()

    # ===== Iteration =====
    def on_next_iteration(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if self.round == 0:
            messagebox.showinfo('提示', '首轮请先点击"AI 生成建议"并应用')
            return
        settings = self.check_ai_settings()
        if not settings:
            return
        self.log('开始下一轮 AI 迭代...')
        self.enqueue('AI迭代', lambda: self._do_next_iteration(settings))

    # ===== Backup =====
    def refresh_backups(self):
        self.backup_tree.delete(*self.backup_tree.get_children())
        for b in list_backups():
            self.backup_tree.insert('', 'end', values=(b['file'], b['timestamp'], b['fc_type'], b['count']))

    def on_restore_backup(self):
        sel = self.backup_tree.selection()
        if not sel:
            messagebox.showinfo('提示', '请先选择要恢复的备份')
            return
        item = self.backup_tree.item(sel[0])
        fname = item['values'][0]
        if not messagebox.askyesno('确认', f'将恢复备份 {fname}，这将覆盖当前参数。\n继续？'):
            return
        fpath = os.path.join(BACKUP_DIR, fname)
        self.log(f'正在恢复备份 {fname}...')
        self.enqueue('恢复备份', lambda: self._do_restore(fpath))

    def _do_restore(self, fpath):
        success, failed = restore_backup(self.mav_master, fpath)
        if isinstance(success, bool):
            self.after(0, lambda: self.log(f'❌ 恢复失败: {failed}'))
        else:
            self.after(0, lambda: self.log(f'✅ 恢复完成: 成功 {success} 个, 失败 {len(failed)} 个'))
            self.after(0, self.on_read_params)

    def refresh_history(self):
        self.hist_tree.delete(*self.hist_tree.get_children())
        for h in self.history:
            self.hist_tree.insert('', 0, values=(h['round'], h['time'], h['written'], h['summary'], h['backup']))

    # ===== Log Management =====
    def on_refresh_logs(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        self.logs_status.configure(text='获取中...', foreground='orange')
        self.log('正在获取飞行日志列表...')
        self.enqueue('刷新日志列表', self._do_refresh_logs)

    def _do_refresh_logs(self):
        logs = request_log_list(self.mav_master)
        self.after(0, lambda: self._show_logs(logs))

    def on_check_click(self, tree, event, col_index):
        """点击指定列时切换 ✓ 勾选状态（col_index 从 1 开始）。"""
        row = tree.identify_row(event.y)
        if not row:
            return
        if tree.identify_column(event.x) != f'#{col_index}':
            return
        vals = tree.item(row)['values']
        if not vals or len(vals) < col_index:
            return
        col_name = tree['columns'][col_index - 1]
        tree.set(row, col_name, '' if _is_checked(vals[col_index - 1]) else '✓')

    def _show_logs(self, logs):
        self.logs_tree.delete(*self.logs_tree.get_children())
        for log in logs:
            t = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(log['time_utc'])) if log['time_utc'] else 'Unknown'
            size_str = f"{log['size']/1024:.1f} KB" if log['size'] < 1024*1024 else f"{log['size']/1024/1024:.1f} MB"
            self.logs_tree.insert('', 'end', values=(log['id'], size_str, t, ''), tags=(str(log['id']),))
        self.logs_status.configure(text=f'共 {len(logs)} 个日志', foreground='green')
        self.log(f'✅ 获取到 {len(logs)} 个飞行日志')

    def on_browse_log_dir(self):
        from tkinter import filedialog
        d = filedialog.askdirectory(initialdir=self.log_dir_var.get())
        if d:
            self.log_dir_var.set(d)

    def on_download_logs(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        checked = []
        highlighted = []
        for item in self.logs_tree.get_children():
            vals = self.logs_tree.item(item)['values']
            if not vals:
                continue
            try:
                log_id = int(vals[0])
            except (ValueError, TypeError):
                continue
            if _is_checked(vals[3]):
                checked.append(log_id)
            elif item in self.logs_tree.selection():
                highlighted.append(log_id)
        selected = checked or highlighted
        if not selected:
            messagebox.showinfo('提示', '请先勾选要下载的日志\n(点击行首"选择"列打 ✓，或点击选中行)')
            return
        if not checked and highlighted:
            self.log(f'按高亮选中的 {len(highlighted)} 个日志下载（未勾选"选择"列）')
        if not messagebox.askyesno('确认', f'将下载 {len(selected)} 个日志文件，可能需要较长时间。\n继续？'):
            return
        self.log(f'开始下载 {len(selected)} 个日志: {selected}')
        self.log_progress['maximum'] = len(selected)
        self.log_progress['value'] = 0
        self.enqueue('下载日志', lambda: self._do_download_logs(selected))

    def _do_download_logs(self, log_ids):
        save_dir = self.log_dir_var.get()
        os.makedirs(save_dir, exist_ok=True)
        
        results = download_logs(self.mav_master, log_ids)
        
        saved = []
        failed = []
        for i, log_id in enumerate(log_ids):
            data, err = results.get(log_id, (None, "Unknown error"))
            self.after(0, lambda v=i+1: self.log_progress.configure(value=v))
            if err:
                failed.append((log_id, err))
            elif data:
                fpath = save_log_to_file(data, log_id, save_dir)
                saved.append((log_id, fpath))
        
        self.after(0, lambda: self._after_download_logs(saved, failed))

    def _after_download_logs(self, saved, failed):
        self.log_progress['value'] = self.log_progress['maximum']
        if saved:
            for log_id, fpath in saved:
                self.log(f'✅ 日志 {log_id} 已保存: {fpath}')
        if failed:
            for log_id, err in failed:
                self.log(f'❌ 日志 {log_id} 下载失败: {err}')
        
        msg = f'下载完成: 成功 {len(saved)} 个, 失败 {len(failed)} 个'
        if self.auto_parse_var.get() and saved:
            msg += '\n开始解析...'
            self.log(msg)
            self.enqueue('解析日志', lambda: self._do_parse_logs(saved))
        else:
            self.log(msg)
            messagebox.showinfo('完成', msg)

    def _do_parse_logs(self, saved):
        for log_id, fpath in saved:
            self.log(f'正在解析 {fpath}...')
            csv_dir, err = parse_log_with_mavlogdump(fpath)
            if err:
                self.log(f'❌ 解析失败: {err}')
                continue
            self.log(f'✅ 解析完成: {csv_dir}')
            if self.ai_analyze_var.get():
                self.log('正在进行 AI 分析...')
                self.enqueue('AI日志分析', lambda: self._do_ai_log_analysis(csv_dir, fpath))
                break

    def _do_ai_log_analysis(self, csv_dir, bin_path):
        pid, api_key, base_url, model = self.get_ai_settings()
        info = provider_info(pid)
        if not api_key and not info.get('key_optional'):
            self.log('❌ 未填写 API Key，跳过 AI 分析')
            return
        if not model:
            self.log('❌ 未选择模型，跳过 AI 分析')
            return
        self.log(f'调用 {info["label"]} / {model} 分析日志...')
        analysis, err = analyze_log_with_ai(csv_dir, api_key, model,
                                            provider=pid, base_url=base_url)
        if err:
            self.after(0, lambda: self.log(f'❌ AI 分析失败: {err}'))
            return
        self.after(0, lambda: self._show_log_ai_analysis(analysis, bin_path))

    def _show_log_ai_analysis(self, analysis, bin_path):
        self.log_ai_text.delete('1.0', 'end')
        self.log_ai_text.insert('1.0', f"日志文件: {bin_path}\n\n")
        self.log_ai_text.insert('end', analysis)
        self.log('✅ AI 日志分析完成，结果已显示')

    def on_erase_logs(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if not messagebox.askyesno('确认', '将清除飞控上所有数据闪存日志，此操作不可恢复！\n确认？'):
            return
        self.log('正在清除飞控日志...')
        self.enqueue('清除日志', self._do_erase_logs)

    def _do_erase_logs(self):
        success, msg = erase_logs(self.mav_master)
        if success:
            self.after(0, lambda: self.log(f'✅ {msg}'))
            self.after(0, self.on_refresh_logs)
        else:
            self.after(0, lambda: self.log(f'❌ {msg}'))

    # ===== Flight Test =====
    def on_start_recording(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        self.flight_recorder = FlightRecorder(self.mav_master)
        self.flight_recorder.start()
        self.recording = True
        self.btn_start_rec.configure(state='disabled')
        self.btn_stop_rec.configure(state='normal')
        self.rec_status.configure(text='记录中...', foreground='green')
        self.metrics_text.delete('1.0', 'end')
        self.log('📹 开始记录飞行数据...')

    def on_stop_recording(self):
        if not self.recording or not self.flight_recorder:
            return
        records = self.flight_recorder.stop()
        self.recording = False
        self.btn_start_rec.configure(state='normal')
        self.btn_stop_rec.configure(state='disabled')
        self.rec_status.configure(text=f'已停止 ({len(records)} 条)', foreground='blue')
        self.log(f'🛑 停止记录，共 {len(records)} 条数据')
        # Analyze
        self.enqueue('分析飞行数据', lambda: self._do_analyze_flight())

    def _do_analyze_flight(self):
        if not self.flight_recorder:
            return
        metrics = self.flight_recorder.analyze()
        self.after(0, lambda: self._show_metrics(metrics))

    def _show_metrics(self, metrics):
        self.metrics_text.delete('1.0', 'end')
        if not metrics:
            self.metrics_text.insert('end', '无数据')
            return
        lines = [f"记录时长: {metrics.get('duration', 0):.1f}s"]
        lines.append(f"采样数: 姿态={metrics['sample_counts'].get('attitude',0)}, 振动={metrics['sample_counts'].get('vibration',0)}, GPS={metrics['sample_counts'].get('gps',0)}, RC={metrics['sample_counts'].get('rc',0)}")
        if 'attitude' in metrics:
            a = metrics['attitude']
            lines.append(f"横滚: 均值={a['roll']['mean']:.3f} RMS={a['roll']['rms']:.3f} 极差=[{a['roll']['min']:.3f},{a['roll']['max']:.3f}] 零穿越={a['roll']['zero_cross']}")
            lines.append(f"俯仰: 均值={a['pitch']['mean']:.3f} RMS={a['pitch']['rms']:.3f} 极差=[{a['pitch']['min']:.3f},{a['pitch']['max']:.3f}] 零穿越={a['pitch']['zero_cross']}")
            lines.append(f"航向: 均值={a['yaw']['mean']:.3f} RMS={a['yaw']['rms']:.3f} 零穿越={a['yaw']['zero_cross']}")
            lines.append(f"角速度RMS: 横滚={a['roll_rate_rms']:.3f} 俯仰={a['pitch_rate_rms']:.3f} 偏航={a['yaw_rate_rms']:.3f}")
        if 'vibration' in metrics:
            v = metrics['vibration']
            lines.append(f"振动 X: RMS={v['x']['rms']:.2f} 峰值={v['x']['peak']:.2f} 裁剪={v['x']['clip_count']}")
            lines.append(f"振动 Y: RMS={v['y']['rms']:.2f} 峰值={v['y']['peak']:.2f} 裁剪={v['y']['clip_count']}")
            lines.append(f"振动 Z: RMS={v['z']['rms']:.2f} 峰值={v['z']['peak']:.2f} 裁剪={v['z']['clip_count']}")
        if 'altitude' in metrics:
            alt = metrics['altitude']
            lines.append(f"高度: 均值={alt['alt_mean']:.2f}m RMS={alt['alt_rms']:.3f}m 竖向速度RMS={alt['vz_rms']:.3f}m/s")
        if 'rc' in metrics:
            rc = metrics['rc']
            for k, v in rc.items():
                lines.append(f"RC {k}: 均值={v['mean']:.0f} 范围=[{v['min']},{v['max']}]")
        self.metrics_text.insert('end', '\n'.join(lines))

    def on_step_excitation(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        # Safety check
        ok, msg = check_safety_for_step(self.mav_master)
        if not ok:
            messagebox.showerror('安全检查失败', f'无法执行阶跃激励:\n{msg}\n\n请确保:\n1. 已解锁\n2. 处于定点/悬停模式\n3. 相对高度 ≥ 2米\n4. GPS 3D定位良好')
            return
        # Double confirmation
        if not messagebox.askyesno('双重确认', f'即将执行自动阶跃激励:\n- 轴: {self.step_axis_var.get()}\n- 幅度: {self.step_amp_var.get()}度\n- 周期: {self.step_cycles_var.get()}\n\n飞机将自动进行姿态阶跃，过程可随时点击"中止"。\n确认执行？'):
            return
        self.step_abort.clear()
        self.step_status.configure(text='执行中...', foreground='orange')
        self.btn_step.configure(state='disabled')
        self.log(f'🚀 开始自动阶跃激励: 轴={self.step_axis_var.get()}, 幅度={self.step_amp_var.get()}°')
        self.enqueue('阶跃激励', lambda: self._do_step_excitation())

    def _do_step_excitation(self):
        try:
            axis = self.step_axis_var.get()
            amp = float(self.step_amp_var.get())
            cycles = int(self.step_cycles_var.get())
            success, msg = auto_step_excitation(self.mav_master, axis=axis, amp_deg=amp, cycles=cycles, abort_flag=self.step_abort)
            self.after(0, lambda: self._after_step(success, msg))
        except Exception as e:
            self.after(0, lambda: self._after_step(False, f'异常: {e}'))

    def _after_step(self, success, msg):
        self.btn_step.configure(state='normal')
        self.step_status.configure(text=msg, foreground='green' if success else 'red')
        if success:
            self.log(f'✅ {msg}')
        else:
            self.log(f'❌ {msg}')

    def _do_next_iteration(self, settings):
        pid, api_key, base_url, model = settings
        current = {}
        for item in self.ai_tree.get_children():
            vals = self.ai_tree.item(item)['values']
            if vals:
                current[vals[0]] = float(vals[1])
        spec = self.get_spec_dict()
        selected_groups = [g for g, v in self.group_vars.items() if v.get()]
        # Include flight metrics in feedback if available
        feedback = self.feedback_text.get('1.0', 'end-1c').strip()
        if self.flight_recorder:
            metrics = self.flight_recorder.analyze()
            if metrics:
                fb = []
                if 'attitude' in metrics:
                    a = metrics['attitude']
                    fb.append(f"横滚RMS={a['roll']['rms']:.3f} 零穿越={a['roll']['zero_cross']}")
                    fb.append(f"俯仰RMS={a['pitch']['rms']:.3f} 零穿越={a['pitch']['zero_cross']}")
                if 'vibration' in metrics:
                    v = metrics['vibration']
                    fb.append(f"振动X RMS={v['x']['rms']:.2f} Y={v['y']['rms']:.2f} Z={v['z']['rms']:.2f}")
                if 'altitude' in metrics:
                    alt = metrics['altitude']
                    fb.append(f"高度RMS={alt['alt_rms']:.3f}m VZ={alt['vz_rms']:.3f}m/s")
                if fb:
                    feedback += "\n[飞行指标] " + "; ".join(fb)
        
        prompt = build_prompt(spec, current, self.fc_type, selected_groups, feedback, self.history)
        self.log(f'调用 {provider_info(pid)["label"]} / {model} ...')
        content, err = ai_chat(api_key, SYSTEM_PROMPT, prompt, model=model,
                               provider=pid, base_url=base_url)
        
        if err:
            self.after(0, lambda: self.log(f'❌ AI 调用失败: {err}'))
            return
        parsed = extract_json(content)
        if not parsed:
            self.after(0, lambda: self.log('❌ AI 返回格式错误'))
            self.after(0, lambda: self.ai_text.delete('1.0', 'end'))
            self.after(0, lambda: self.ai_text.insert('1.0', content))
            return
        
        ai_params = parsed.get('params', {})
        valid_params, warnings = validate_ai_params(ai_params, self.fc_type, selected_groups)
        
        self.after(0, lambda: self._update_ai_tree(valid_params, warnings))
        self.after(0, lambda: self._show_ai_result(parsed))

    def start_worker(self):
        def loop():
            while self.worker_running:
                try:
                    name, fn = self.action_q.get(timeout=0.5)
                except queue.Empty:
                    continue
                self.action_busy.set()
                try:
                    fn()
                except Exception as e:
                    self.log(f'❌ 任务异常: {e}')
                finally:
                    self.action_busy.clear()
                    self.action_q.task_done()
        self.worker_thread = threading.Thread(target=loop, daemon=True)
        self.worker_thread.start()

    def enqueue(self, name, fn):
        if not self.connected:
            self.log('❌ 未连接飞控')
            return
        if self.action_busy.is_set():
            self.log(f'⚠️ 当前有任务运行中，已加入队列: {name}')
        self.action_q.put((name, fn))
        self.log(f'▶ 已排队: {name}')

    def poll_log(self):
        while True:
            try:
                msg = self.log_q.get_nowait()
                self._log_line(msg)
            except queue.Empty:
                break
        self.after(200, self.poll_log)

    def log(self, msg):
        self.log_q.put(msg)

    def _log_line(self, msg):
        tag = None
        if '✅' in msg or '通过' in msg:
            tag = 'ok'
        elif '❌' in msg or '失败' in msg or '错误' in msg:
            tag = 'err'
        elif '⚠️' in msg or '警告' in msg:
            tag = 'warn'
        elif '▶' in msg or '已排队' in msg:
            tag = 'info'
        t = time.strftime('%H:%M:%S')
        self.txt_log.configure(state='normal')
        self.txt_log.insert('end', f'[{t}] {msg}\n', tag)
        self.txt_log.see('end')
        self.txt_log.configure(state='disabled')
        # Limit lines
        lines = int(self.txt_log.index('end-1c').split('.')[0])
        if lines > 3000:
            self.txt_log.configure(state='normal')
            self.txt_log.delete('1.0', f'{lines - 3000}.0')
            self.txt_log.configure(state='disabled')

    def on_closing(self):
        self.worker_running = False
        if self.connected:
            self.on_disconnect()
        self.destroy()

if __name__ == '__main__':
    app = App()
    app.protocol('WM_DELETE_WINDOW', app.on_closing)
    app.mainloop()