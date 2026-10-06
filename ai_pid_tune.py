# AI PID Tuning GUI (Standalone)
from ai_tune_core import *
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import json, os, time, threading, queue

_BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(_BASE, 'ai_tune_config.json')
BACKUP_DIR = os.path.join(_BASE, 'ai_tune_backups')

# ===== 一键首飞：按《docs/apm建议调参顺序.md》的调参顺序自动派生全量方案 =====
# 数据来源：ArduPilot 官方 "Setting the Aircraft Up for Tuning" / "Initial Tuning Flight"
#           / "Parameter Name Changes (4.6→4.7)" / "Throttle Based Dynamic Notch Setup"
_SIZE_PROFILES = {
    5:  dict(gyro=80, expo=0.55, acc=1100, accy=300, slew=1.0),
    7:  dict(gyro=60, expo=0.60, acc=1100, accy=250, slew=1.0),
    10: dict(gyro=40, expo=0.65, acc=1100, accy=200, slew=1.5),
    13: dict(gyro=30, expo=0.68, acc=900,  accy=150, slew=2.0),
    15: dict(gyro=25, expo=0.70, acc=700,  accy=120, slew=2.0),
    20: dict(gyro=20, expo=0.75, acc=500,  accy=100, slew=2.0),
    30: dict(gyro=20, expo=0.80, acc=200,  accy=90,  slew=3.0),
}
_OT_PROFILES = {
    'soft': dict(name='更保守', speed_up=80,  accel_z=120, pilot_speed=80,
                 pilot_accel=150, slew_adj=0.5, jerk=6),
    'std':  dict(name='标准',   speed_up=100, accel_z=150, pilot_speed=100,
                 pilot_accel=200, slew_adj=0.0, jerk=8),
    'hot':  dict(name='更激进', speed_up=150, accel_z=200, pilot_speed=150,
                 pilot_accel=250, slew_adj=-0.3, jerk=12),
}
_OT_STAGES = [
    ('battery',  '1 电池与推力曲线'),
    ('spin',     '2 电机怠速与输出'),
    ('takeoff',  '3 起飞'),
    ('track',    '4 水平导航 WPNAV (GUIDED/AUTO)'),
    ('vert',     '5 垂直与高度'),
    ('filter',   '6 滤波与姿态'),
    ('ekf',      '7 定位与 EKF (可选)'),
    ('safety',   '8 安全与保护'),
]
_OT_PURPOSES = {
    'first': '首飞(安全保守)',
    'guided': 'GUIDED 外部引导追踪',
    'auto': 'AUTO 任务航点',
}
# 第 4 阶段取值：用途 × 档位。数值全部 ≥ 固件默认，否则"推荐"反而更肉。
# 固件默认(4.6): WPNAV_SPEED=1000 ACCEL=100~250 JERK=1 RADIUS=200
#                PSC_POSXY_P=1.0 VELXY_P=2.0 VELXY_I=1.0 VELXY_FF=0 JERK_XY=5
_TRACK_PROFILES = {
    'soft': dict(speed=1200, accel=300, jerk=8,   radius=300,
                 pos=1.0, velp=2.0, veli=0.7, velff=0.2, jxy=8,
                 speed_dn=150, vzff=0.2),
    'std':  dict(speed=1500, accel=400, jerk=12,  radius=200,
                 pos=1.2, velp=2.2, veli=1.0, velff=0.3, jxy=12,
                 speed_dn=200, vzff=0.3),
    'hot':  dict(speed=2000, accel=500, jerk=20,  radius=100,
                 pos=1.5, velp=2.5, veli=1.0, velff=0.5, jxy=20,
                 speed_dn=300, vzff=0.5),
}
_TRACK_FIRST = dict(speed=800, accel=150, jerk=5, radius=300,
                    pos=1.0, velp=1.5, veli=0.7, velff=0.1, jxy=5,
                    speed_dn=150, vzff=0.1)
# AUTO 任务(固定航线/转场/测绘)取值，来自《docs/AUTO模式专用速查表.md》并按固件范围封顶:
#   SPEED 文档 1000~1500 / 2000~2500 → 固件上限 2000; ACCEL 文档 800~1500 → 上限 500
#   JERK 文档 15~30 → 上限 20; ACCEL_Z 文档 500~800 → 上限 500; POSXY_P 文档 1.2~1.8
#   VELXY_P 1.5~2.2; VELXY_I 0.5~1.0; SPEED_UP 300~500; SPEED_DN 200~400; JERK_Z 10~20
_AUTO_PROFILES = {
    'soft': dict(speed=1200, accel=300, jerk=12,  radius=300,
                 pos=1.2, velp=1.5, veli=0.7, velff=0.1, jxy=8,
                 speed_up=300, speed_dn=200, accel_z=300, vzff=0.1, jerk_z=10),
    'std':  dict(speed=1500, accel=400, jerk=15,  radius=200,
                 pos=1.4, velp=2.0, veli=1.0, velff=0.1, jxy=12,
                 speed_up=400, speed_dn=300, accel_z=400, vzff=0.2, jerk_z=14),
    'hot':  dict(speed=2000, accel=500, jerk=20,  radius=100,
                 pos=1.8, velp=2.2, veli=1.0, velff=0.2, jxy=20,
                 speed_up=500, speed_dn=400, accel_z=500, vzff=0.3, jerk_z=20),
}


def _first_int(text):
    """从 '5寸' / '6S' / '5x4.3' 里取出第一个整数，取不到返回 None。"""
    digits = ''
    for ch in str(text or ''):
        if ch.isdigit():
            digits += ch
        elif digits:
            break
    try:
        return int(digits) if digits else None
    except ValueError:
        return None


def _size_profile(size):
    try:
        size = int(round(float(size)))
    except (TypeError, ValueError):
        size = 5
    best = min(_SIZE_PROFILES, key=lambda k: (abs(k - size), k))
    return _SIZE_PROFILES[best]


def _mk(name, value, unit, rng, desc, h, aliases=None, check=True):
    d = dict(name=name, value=value, unit=unit, rng=rng, desc=desc, help=h)
    if aliases:
        d['aliases'] = aliases
    d['check'] = check
    return d


def _ot_stages(ctx):
    """按调参顺序生成 8 个阶段的参数方案。
    ctx: size/cells/hover/esc_linear/profile/gyro/purpose(first|guided|auto)"""
    size = int(ctx.get('size') or 5)
    cells = int(ctx.get('cells') or 6)
    hover = float(ctx.get('hover') or 0.25)
    esc_linear = bool(ctx.get('esc_linear'))
    purpose = ctx.get('purpose') if ctx.get('purpose') in _OT_PURPOSES else 'first'
    prof_key = ctx.get('profile') or 'std'
    q = _OT_PROFILES.get(prof_key, _OT_PROFILES['std'])
    if purpose == 'auto':
        tk_ = _AUTO_PROFILES.get(prof_key, _AUTO_PROFILES['std'])
    elif purpose == 'guided':
        tk_ = _TRACK_PROFILES.get(prof_key, _TRACK_PROFILES['std'])
    else:
        tk_ = _TRACK_FIRST
    auto = (purpose == 'auto')
    v_up, v_accz, v_jerk = ((tk_['speed_up'], tk_['accel_z'], tk_['jerk_z'])
                            if auto else (q['speed_up'], q['accel_z'], q['jerk']))
    p = _size_profile(size)
    expo = 0.10 if esc_linear else p['expo']
    slew = max(0.25, min(5.0, p['slew'] + q['slew_adj']))
    gyro = int(round(float(ctx.get('gyro') or p['gyro'])))
    gyro_src = ctx.get('gyro') and '读取值' or '按尺寸推荐值'
    half = max(5, min(100, int(round(gyro / 2.0))))
    st = {}

    st['battery'] = [
        _mk('MOT_BAT_VOLT_MAX', round(4.2 * cells, 2), 'V', (0, 60),
            '满电电压(电压补偿上限)',
            f'官方: 满电电压 = 4.2V × 串数 = 4.2 × {cells} = {4.2*cells:.1f}V。\n'
            '决定油门/PID 电压补偿的上限，不设则不同电量下响应不一致。'),
        _mk('MOT_BAT_VOLT_MIN', round(3.3 * cells, 2), 'V', (0, 60),
            '最低工作电压(电压补偿下限)',
            f'官方: 最低电压 = 3.3V × 串数 = 3.3 × {cells} = {3.3*cells:.1f}V。\n'
            '与 BATT_CRT_VOLT 一致较合理；再往下飞控不再放大增益。'),
        _mk('MOT_THST_EXPO', expo, '', (-1.0, 1.0),
            '推力曲线线性化指数',
            (f'电调已内置线性化 → 0~0.2(取 0.10)。\n' if esc_linear else
             f'官方按桨径: 5寸=0.55 / 10寸=0.65 / 20寸=0.75，当前 {size}寸 → {expo}。\n') +
             '注: 文档"大桨低KV降到0.2~0.3"只适用于电调已线性化的情况；\n'
             '桨越大 EXPO 应当越高，用错会让低油门段发冲、刹车姿态不稳。'),
        _mk('MOT_THST_HOVER', hover, '', (0.125, 0.6875),
            '悬停油门初始估计',
            '首飞保守取 0.20~0.25；推重比大的火箭机宁可低估。\n'
            '它同时是下面 PSC_ACCZ_P/I 的派生基准，写完起飞 30s 落地会自动校正。'),
        _mk('MOT_HOVER_LEARN', 2, '', (0, 2),
            '悬停油门学习: 0=关 1=学 2=学并保存',
            '保持 2：悬停 ≥30s 落地上锁后写回真实悬停油门。'),
    ]
    st['spin'] = [
        _mk('MOT_SPIN_ARM', 0.10, '', (0.0, 0.2),
            '解锁怠速(电机起转余量)',
            '先用「拆桨实测起转」按钮自动测：从 6% 逐档升油门，\n'
            '用 ESC 遥测 RPM>0 判定起转点，再 +1% 作为本值。\n'
            '没有 ESC 遥测就拆桨人工测。'),
        _mk('MOT_SPIN_MIN', 0.15, '', (0.0, 0.25),
            '飞行最低油门下限',
            '必须 ≥ MOT_SPIN_ARM + 0.03(应用前会自动校验)。\n'
            '过低会让起飞段输出不线性、离地瞬间抖动。'),
        _mk('MOT_SPIN_MAX', 0.95, '', (0.9, 1.0),
            '最大输出上限',
            '默认 0.95，顶部留 5% 余量；不要设 1.0 以免油门饱和。'),
    ]
    st['takeoff'] = [
        _mk('TKOFF_THR_MAX', 1.0, '', (0.0, 1.0),
            '起飞阶段允许的最大油门',
            '保持 1.0，离地快慢由 WPNAV_SPEED_UP/WPNAV_ACCEL_Z 决定。'),
        _mk('TKOFF_SLEW_TIME', slew, 's', (0.25, 5.0),
            '起飞油门爬升时间',
            f'文档建议 0.5~1.0s(更柔 2~3s)，固件默认 2.0s。\n'
            f'{size}寸 → {slew:g}s；离地"弹一下"就加大它。'),
    ]
    purpose_cn = _OT_PURPOSES[purpose]
    st['track'] = [
        _mk('WPNAV_SPEED', tk_['speed'], 'cm/s', (10, 2000),
            '水平导航最大速度',
            f'用途「{purpose_cn}」→ {tk_["speed"]} cm/s (固件默认 1000)。\n'
            + ('AUTO: 航点间最大水平速度，常规 1000~1500、快速转场再往上；\n'
               '只提高它不提 WPNAV_ACCEL/JERK 会"跑不满速度"。任务中可用\n'
               'DO_CHANGE_SPEED 命令在航段间动态改速。\n' if auto else
               'GUIDED: 追移动目标时它是"追不追得上"的第一限制。\n')
            + '文档建议 1500~2500，⚠ 固件上限 2000，本工具按 2000 封顶。\n'
            '超调/摆动就降到 1200。4.7 起为 WP_SPD(m/s) 自动换算。',
            aliases=[('WP_SPD', 0.01)]),
        _mk('WPNAV_ACCEL', tk_['accel'], 'cm/s/s', (50, 500),
            '水平导航加速度上限',
            f'用途「{purpose_cn}」→ {tk_["accel"]} cm/s² (固件默认 100~250)。\n'
            '⚠ 固件声明范围只有 50~500，两份文档里的 800~1500 已超出上限，\n'
            '本工具按 500 封顶；速度高时必须同步提高它，否则"跑不满速度"。\n'
            '追不上/跑不动先加它，来回摆就降它。4.7 起为 WP_ACC(m/s²)。',
            aliases=[('WP_ACC', 0.01)]),
        _mk('WPNAV_JERK', tk_['jerk'], 'm/s³', (1, 20),
            '水平加加速度(响应猛不猛)',
            f'用途「{purpose_cn}」→ {tk_["jerk"]} m/s³ (固件默认 1，范围 1~20)。\n'
            'GUIDED 文档写 10~30、AUTO 文档写 15~30，⚠ 上限只有 20；\n'
            '越大越"跟手"，超过 15 容易抖。4.7 起为 WP_JERK(不换算)。',
            aliases=[('WP_JERK', 1.0)]),
        _mk('WPNAV_RADIUS', tk_['radius'], 'cm', (5, 1000),
            '到达容差半径',
            f'用途「{purpose_cn}」→ {tk_["radius"]} cm (固件默认 200)。\n'
            '首飞/追踪/任务初期 200~300：太小会在航点附近反复修正、任务时间变长，\n'
            '看起来像"到不了"；测绘/拍照需要精准过点时再逐次收到 100。\n'
            '4.7 起为 WP_RADIUS_M(m) 自动换算。',
            aliases=[('WP_RADIUS_M', 0.01)]),
        _mk('PSC_POSXY_P', tk_['pos'], '', (0.5, 2.0),
            '水平位置环 P',
            f'用途「{purpose_cn}」→ {tk_["pos"]} (固件默认 1.0，范围 0.5~2.0)。\n'
            '位置误差→目标速度的换算系数。超调/来回摆先降它，追不到再升。\n'
            '文档 1.2~1.8 已在合法区间内。4.7 起为 PSC_NE_POS_P(不换算)。',
            aliases=[('PSC_NE_POS_P', 1.0)]),
        _mk('PSC_VELXY_P', tk_['velp'], '', (0.1, 6.0),
            '水平速度环 P',
            f'用途「{purpose_cn}」→ {tk_["velp"]} (固件默认 2.0)。\n'
            '决定对速度误差的响应力度；加大后配合 PSC_VELXY_FF 更跟手。\n'
            '出现水平抖动先降到 2.0。4.7 起为 PSC_NE_VEL_P(不换算)。',
            aliases=[('PSC_NE_VEL_P', 1.0)]),
        _mk('PSC_VELXY_I', tk_['veli'], '', (0.02, 1.0),
            '水平速度环 I',
            f'用途「{purpose_cn}」→ {tk_["veli"]} (固件默认 1.0，范围上限就是 1.0)。\n'
            '消除稳态误差(顶风漂移)；逆风跟随出现偏移时升到 1.0。\n'
            '4.7 起为 PSC_NE_VEL_I；同名 IMAX 在 4.7 缩小 100 倍，本工具不碰。',
            aliases=[('PSC_NE_VEL_I', 1.0)]),
        _mk('PSC_VELXY_FF', tk_['velff'], '', (0, 6),
            '水平速度前馈',
            f'用途「{purpose_cn}」→ {tk_["velff"]} (固件默认 0)。\n'
            '★ 外部系统已提供目标速度(vx,vy)时，这一项能明显减小滞后：\n'
            '  地面站发位置+速度前馈 + 这里给 0.3~0.5。\n'
            '只发纯位置指令时给 0.1~0.3 即可。4.7 起为 PSC_NE_VEL_FF。',
            aliases=[('PSC_NE_VEL_FF', 1.0)]),
        _mk('PSC_JERK_XY', tk_['jxy'], 'm/s³', (1, 20),
            '水平轨迹整形 Jerk',
            f'用途「{purpose_cn}」→ {tk_["jxy"]} m/s³ (固件默认 5，范围 1~20)。\n'
            '控制加速度变化率：越大起步/转向越干脆，过大则机身猛抖。\n'
            '与 WPNAV_JERK 不同，它作用在位置控制器的轨迹整形上。4.7 起为 PSC_NE_JERK。',
            aliases=[('PSC_NE_JERK', 1.0)]),
    ]
    for e in st['track']:
        e['check'] = (purpose in ('guided', 'auto'))
    if auto:
        st['track'] += [
            _mk('WP_YAW_BEHAVIOR', 2, '', (0, 3),
                'AUTO 航向行为',
                '0=不改航向; 1=指向下个航点; 2=指向下个航点(RTL 除外,固件默认);\n'
                '3=沿 GPS 航迹方向。\n'
                '光电/雷达要始终对准任务区 → 提前规划(或用 ROI/固定航向);\n'
                '沿航线自然飞、看航迹方向 → 3。仅 AUTO 任务与 RTL 使用，GUIDED 不看它。'),
            _mk('TUNE', 10, '', None,
                '遥控器旋钮调参目标 (可选)',
                '10 = WP Speed(实时调 WPNAV_SPEED)。\n'
                '要用它必须把某个遥控通道的 RCx_OPTION 设为 219(发射机调参)，\n'
                '通道号按实际接线(常用 RC6_OPTION)；同时配套下面 TUNE_MIN/MAX。\n'
                '⚠ 默认不勾选：没配通道就写入也没用，配了又不想被覆盖请保持不勾。',
                check=False),
            _mk('TUNE_MIN', 500, 'cm/s', None,
                '旋钮最低端对应值 (可选)',
                'TUNE=10 时单位是 cm/s：500 = 5 m/s。\n'
                '固件默认 0(旋钮拧到底会把速度写成 0)，建议给 500 左右。默认不勾选。',
                check=False),
            _mk('TUNE_MAX', 2000, 'cm/s', None,
                '旋钮最高端对应值 (可选)',
                'TUNE=10 时单位是 cm/s：2000 = 20 m/s(固件速度上限)。\n'
                '与 TUNE_MIN 一起保证旋钮全程落在合法区间。默认不勾选。',
                check=False),
        ]
    st['vert'] = [
        _mk('WPNAV_SPEED_UP', v_up, 'cm/s', (10, 1000),
            'Guided/AUTO 起飞上升速度',
            ('AUTO: 文档建议 300~500 cm/s，档位「%s」→ %d cm/s；'
             '快速爬升的航线可再提高。' % (q['name'], v_up) if auto else
             '首飞最敏感的一项：离地太猛降、反复贴地升。\n'
             '档位「%s」→ %d cm/s。' % (q['name'], v_up))
            + '4.7 起为 WP_SPD_UP(m/s) 自动换算。',
            aliases=[('WP_SPD_UP', 0.01)]),
        _mk('WPNAV_SPEED_DN', tk_['speed_dn'], 'cm/s', (10, 500),
            'Guided/AUTO 下降速度',
            f'用途「{purpose_cn}」→ {tk_["speed_dn"]} cm/s (固件默认 150)。\n'
            + ('AUTO 文档建议 200~400 cm/s：下降过快容易失稳。\n' if auto else '')
            + '目标高度往下走时的下降率上限；降落/掉高太快就调小。\n'
            '4.7 起为 WP_SPD_DN(m/s) 自动换算。',
            aliases=[('WP_SPD_DN', 0.01)]),
        _mk('WPNAV_ACCEL_Z', v_accz, 'cm/s/s', (50, 500),
            '垂直加速度上限',
            (f'AUTO 文档建议 500~800，⚠ 固件上限 500 → 取 {v_accz}；先稳后快。\n'
             if auto else
             f'档位「{q["name"]}」→ {v_accz}；\n')
            + '到顶冲高回荡就减小；过小则爬升迟钝。4.7 起为 WP_ACC_Z(m/s²)。',
            aliases=[('WP_ACC_Z', 0.01)]),
        _mk('PILOT_SPEED_UP', q['pilot_speed'], 'cm/s', (50, 500),
            '手动模式上升速度上限',
            f'首飞压到 {q["pilot_speed"]}，确认手感后可回默认 250。',
            aliases=[('PILOT_SPD_UP', 0.01)]),
        _mk('PILOT_ACCEL_Z', q['pilot_accel'], 'cm/s/s', (50, 500),
            '手动模式垂直加速度',
            f'档位「{q["name"]}」→ {q["pilot_accel"]}；高度抖动时调小。',
            aliases=[('PILOT_ACC_Z', 0.01)]),
        _mk('PSC_ACCZ_P', round(hover, 3), '', (0.2, 1.5),
            '垂直加速度 P(由悬停油门派生)',
            '官方 Initial Tuning Flight:\n'
            '  ≤4.6: PSC_ACCZ_P = MOT_THST_HOVER\n'
            '  4.7+: PSC_D_ACC_P = 0.1 × MOT_THST_HOVER(数值缩小 10 倍)\n'
            '本工具按固件实际存在的参数名自动换算，避免写错 10 倍。',
            aliases=[('PSC_D_ACC_P', 0.1)]),
        _mk('PSC_ACCZ_I', round(2 * hover, 3), '', (0.0, 3.0),
            '垂直加速度 I(由悬停油门派生)',
            '官方: ≤4.6 为 2×MOT_THST_HOVER；4.7+ 为 0.2×MOT_THST_HOVER。\n'
            '过大会出现缓慢的高度振荡。',
            aliases=[('PSC_D_ACC_I', 0.1)]),
        _mk('PSC_JERK_Z', v_jerk, '', (5, 50),
            '垂直加加速度(油门变化剧烈程度)',
            (f'AUTO 文档建议 10~20，档位「{q["name"]}」→ {v_jerk}；高度变化不再突兀。\n'
             if auto else
             f'文档 5~10，档位「{q["name"]}」取 {v_jerk}；\n')
            + '离地瞬间"弹一下" → 加大本值或降 WPNAV_ACCEL_Z。\n'
            '4.7 起改名 PSC_D_JERK(不换算)。',
            aliases=[('PSC_D_JERK', 1.0)]),
        _mk('PSC_VELZ_FF', tk_['vzff'], '', (0, 1),
            '垂直速度前馈',
            f'用途「{purpose_cn}」→ {tk_["vzff"]} (固件默认 0)。\n'
            '目标高度连续变化时给一点前馈，高度跟踪会更贴合、滞后更小。\n'
            '高度保持不变的场景给 0 即可。4.7 起为 PSC_D_VEL_FF(不换算)。',
            aliases=[('PSC_D_VEL_FF', 1.0)]),
    ]
    filt = [
        _mk('INS_GYRO_FILTER', p['gyro'], 'Hz', (0, 256),
            '陀螺低通滤波频率',
            f'官方按桨径: 5寸=80 / 10寸=40 / 20寸=20 Hz，{size}寸 → {p["gyro"]} Hz。\n'
            '⚠ 该项默认不勾选：若机器振动大，贸然拉高会把振动引进 PID。\n'
            '先飞一段看日志振动，再决定是否改。',
            check=False),
        _mk('INS_ACCEL_FILTER', 10, 'Hz', (0, 256),
            '加速度计低通滤波频率',
            '官方初始值 10Hz。'),
        _mk('INS_HNTCH_ENABLE', 1, '', (0, 1),
            '谐波陷波滤波器使能',
            '官方建议首飞就开；⚠ 需重启后 INS_HNTCH_FREQ/REF 等参数才出现。\n'
            '首飞后用「日志管理」看 FFT 再补 FREQ/BW/REF(REF≈悬停油门)。'),
        _mk('INS_HNTCH_MODE', 1, '', (0, 3),
            '陷波中心频率控制方式',
            '0=固定频率, 1=油门跟随(默认,无需传感器), 2/3=转速/FFT。\n'
            '有双向 DShot 遥测可改 3。'),
        _mk('ATC_RAT_RLL_FLTD', half, 'Hz', (5, 100),
            '横滚速率环 D 滤波',
            f'官方规则 = INS_GYRO_FILTER / 2(当前按 {gyro_src} {gyro} → {half} Hz)。'),
        _mk('ATC_RAT_RLL_FLTT', half, 'Hz', (5, 100),
            '横滚速率环前馈滤波',
            f'官方规则 = INS_GYRO_FILTER / 2 → {half} Hz。'),
        _mk('ATC_RAT_PIT_FLTD', half, 'Hz', (5, 100),
            '俯仰速率环 D 滤波',
            f'官方规则 = INS_GYRO_FILTER / 2 → {half} Hz。'),
        _mk('ATC_RAT_PIT_FLTT', half, 'Hz', (5, 100),
            '俯仰速率环前馈滤波',
            f'官方规则 = INS_GYRO_FILTER / 2 → {half} Hz。'),
        _mk('ATC_RAT_YAW_FLTT', half, 'Hz', (5, 100),
            '偏航速率环前馈滤波',
            f'官方规则 = INS_GYRO_FILTER / 2 → {half} Hz。'),
        _mk('ATC_RAT_YAW_FLTE', 2, 'Hz', None,
            '偏航速率环误差滤波',
            '官方初始值 2Hz(偏航 D 滤波一般不开)。'),
        _mk('ATC_ACCEL_P_MAX', p['acc'], 'deg/s/s', (0, 1800),
            '俯仰角加速度上限',
            f'官方按桨径: 10寸=1100 / 20寸=500 / 30寸=200，{size}寸 → {p["acc"]}。\n'
            '大桨必须调低，否则姿态过猛。4.7 起改名 ATC_ACC_P_MAX(不换算)。',
            aliases=[('ATC_ACC_P_MAX', 1.0)]),
        _mk('ATC_ACCEL_R_MAX', p['acc'], 'deg/s/s', (0, 1800),
            '横滚角加速度上限',
            f'同俯仰，{size}寸 → {p["acc"]}。4.7 起改名 ATC_ACC_R_MAX。',
            aliases=[('ATC_ACC_R_MAX', 1.0)]),
        _mk('ATC_ACCEL_Y_MAX', p['accy'], 'deg/s/s', (0, 1800),
            '偏航角加速度上限',
            f'官方偏航最保守: 10寸=200 / 20寸=100，{size}寸 → {p["accy"]}。\n'
            '偏航过快在首飞容易抖；要敏捷可手动改回默认值。4.7 起改名 ATC_ACC_Y_MAX。',
            aliases=[('ATC_ACC_Y_MAX', 1.0)]),
    ]
    st['filter'] = filt
    st['ekf'] = [
        _mk('EK3_POSNE_M_NSE', 0.5, 'm', (0.1, 10.0),
            'GPS 水平位置观测噪声',
            '固件默认 0.5m。\n'
            '⚠ 关键：外部系统给的是"目标点"，不是飞机自身位置，\n'
            '  与本项无关。只有飞机自己装 RTK/双天线(定位 <0.1m)才往下调，\n'
            '  每次减 10%~20%；GPS 普通就保持默认，调小会 EKF 发散、位置漂移。',
            check=False),
        _mk('EK3_VELNE_M_NSE', 0.5, 'm/s', (0.05, 5.0),
            '水平速度观测噪声下限',
            '固件默认 0.3~0.5 m/s。与上一条同理，只在定位源本身很稳时才收紧。',
            check=False),
        _mk('EK3_VELD_M_NSE', 0.5, 'm/s', (0.05, 5.0),
            '垂直速度观测噪声下限',
            '固件默认 0.5~0.7 m/s。调小会让高度更"信"速度观测，但对噪声更敏感。',
            check=False),
        _mk('EK3_ALT_M_NSE', 2.0, 'm', (0.1, 100.0),
            '气压高度观测噪声',
            '固件默认 2.0m。有气压计防护罩/风扰大时反而应调大，不要盲目调小。',
            check=False),
    ]
    if auto:
        st['ekf'] += [
            _mk('AHRS_EKF_TYPE', 3, '', (0, 3),
                '姿态估计类型 (体检确认)',
                '保持 3 = 使用 EKF3(Auto 模式必须有位置估计)。\n'
                '⚠ AUTO 同样受 EKF 影响：外部定位/GPS/气压不稳会出现航点偏移、\n'
                '高度跳动、转弯不顺 —— 不要只靠提高 P 增益解决。默认不勾选。',
                check=False),
            _mk('EK3_ENABLE', 1, '', (0, 1),
                '启用 EKF3 (体检确认)',
                '保持 1；改 0/1 需重启飞控才生效。默认不勾选。',
                check=False),
            _mk('EK3_SRC1_POSXY', 3, '', (0, 6),
                '水平位置数据源 (体检确认)',
                '固件默认 3=GPS。0=无 3=GPS 4=信标 6=外部导航。\n'
                '室外 GPS 用 3；用了外部定位(动捕/RTK 基站解算)才改。默认不勾选。',
                check=False),
            _mk('EK3_SRC1_VELXY', 3, '', (0, 7),
                '水平速度数据源 (体检确认)',
                '固件默认 3=GPS。0=无 3=GPS 4=信标 5=光流 6=外部导航 7=轮速计。\n'
                '有外部速度源时才改。默认不勾选。',
                check=False),
            _mk('EK3_SRC1_POSZ', 1, '', (0, 6),
                '高度数据源 (体检确认)',
                '固件默认 1=气压计。0=无 1=Baro 2=测距 3=GPS 4=信标 6=外部导航。\n'
                '默认不勾选。',
                check=False),
            _mk('EK3_SRC1_VELZ', 3, '', (0, 6),
                '垂直速度数据源 (体检确认)',
                '固件默认 3=GPS。也可用气压计衍生速度。默认不勾选。',
                check=False),
            _mk('EK3_SRC1_YAW', 1, '', (0, 8),
                '航向数据源 (体检确认)',
                '固件默认 1=罗盘。0=无 1=罗盘 2=GPS 3=GPS+罗盘回退 6=外部导航 8=GSF。\n'
                '双天线 GPS 才用 2/3。默认不勾选。',
                check=False),
        ]
    st['safety'] = [
        _mk('FS_GCS_ENABLE', 1, '', (0, 7),
            '地面站失联保护',
            'Guided 控制必须开: 1=RTL(GPS 不可用时自动改 LAND)。\n'
            '地面站卡死/MAVLink 中断时按此动作。'),
        _mk('BATT_FS_LOW_ACT', 2, '', (0, 5),
            '一级低电压动作',
            '2=RTL；低空调试也可选 1=LAND。'),
        _mk('BATT_FS_CRT_ACT', 1, '', (0, 5),
            '严重低电压动作',
            '1=LAND(强制降落，别再返航)。'),
        _mk('BATT_LOW_VOLT', round(3.5 * cells, 2), 'V', (0, 100),
            '低电压阈值',
            f'3.5V × {cells}S = {3.5*cells:.1f}V。\n'
            '⚠ 固件默认 10.5V，6S 机器永远不会触发，等于保护没开。'),
        _mk('BATT_CRT_VOLT', round(3.3 * cells, 2), 'V', (0, 100),
            '严重低电压阈值',
            f'3.3V × {cells}S = {3.3*cells:.1f}V，须 ≤ BATT_LOW_VOLT。'),
        _mk('FENCE_ENABLE', 1, '', (0, 1), '启用地理围栏', '首飞必须开。'),
        _mk('FENCE_TYPE', 3, '', (0, 7), '围栏类型位掩码',
            'bit0=高度, bit1=圆形(圆心 HOME), bit2=多边形; 首飞用 3。'),
        _mk('FENCE_ACTION', 1, '', (0, 5), '越界动作', '1=RTL 或 LAND。'),
        _mk('FENCE_RADIUS', 50, 'm', (30, 10000), '圆形围栏半径',
            '固件范围 30~10000m(文档写 20m 会被夹到 30m)，首飞 50m。'),
        _mk('FENCE_ALT_MAX', 20, 'm', (10, 1000), '允许最大相对高度',
            '必须 ≥ RTL_ALT，否则返航爬升会再次越界形成循环。'),
        _mk('FENCE_MARGIN', 5, 'm', (1, 10), '距围栏的预留余量',
            '到达围栏前提前减速的缓冲距离。'),
        _mk('RTL_ALT', 1000, 'cm', (0, 10000), '返航爬升高度',
            '默认 1500(15m) > 20m 围栏的爬升安全余量，降到 1000(10m)。\n'
            '★ AUTO 任务必须高于航线中最高障碍物。\n'
            '4.7 起改名 RTL_ALT_M 且按 m 计，自动换算。',
            aliases=[('RTL_ALT_M', 0.01)]),
    ]
    if auto:
        st['safety'] += [
            _mk('RTL_ALT_FINAL', 0, 'cm', (0, 1000),
                '任务/返航结束的最终高度',
                '0 = 到家后自动降落(推荐，任务跑完自己落地)。\n'
                '给正数则悬停在该高度等待指令(要人接手时用)。\n'
                '必须 ≤ RTL_ALT。4.7 起改名 RTL_ALT_FINAL_M 且按 m 计，自动换算。',
                aliases=[('RTL_ALT_FINAL_M', 0.01)]),
            _mk('FS_GCS_TIMEOUT', 5, 's', (2, 120),
                '地面站失联多久触发保护 (体检确认)',
                '固件默认 5s；文档建议 3~10s：太长延误保护、太短易误触发。\n'
                'AUTO 任务依赖外部系统时尤其要设。默认不勾选(等于固件默认)。',
                check=False),
            _mk('FS_EKF_ACTION', 1, '', (1, 3),
                'EKF 异常保护动作 (体检确认)',
                '1=Land(默认，要求位置的模式下转降落) 2=AltHold 3=任何模式都 Land。\n'
                '4.7 起额外支持 0=Report only(只记录不动作)。\n'
                '复杂任务里确认这个动作是否合适。默认不勾选。',
                check=False),
            _mk('DISARM_DELAY', 10, 's', (0, 127),
                '落地后自动上锁延时 (体检确认)',
                '固件默认 10s；0 = 关闭自动上锁。\n'
                '太短会在落地判定瞬间就上锁，螺旋桨还在转动时意外停机。\n'
                '默认不勾选(等于固件默认)。',
                check=False),
        ]
    return [dict(id=sid, label=label, items=st.get(sid, []))
            for sid, label in _OT_STAGES]


OT_HELP = """一键首飞 · 帮助说明 (GUIDED 追踪 / AUTO 任务)
========================================================
一、这个页解决什么
  输入 桨径 / 电池串数 / 悬停油门 / 档位 / 用途 → 自动派生全量参数 →
  体检读取 → 交叉校验 → 自动备份 → 一键写入 → 报告。
  共 8 个阶段，每行都有"依据"列，可勾选、可双击改值。

  用途三选一(左上角):
  · 首飞(安全保守)          —— 先把机器安全飞起来。第 4 阶段(水平导航)、
                                第 7 阶段(EKF)只展示不勾选，避免一次改太多。
                                57 项。
  · GUIDED 外部引导追踪      —— 给"地面站 10~20Hz 发目标坐标追移动目标"用，
                                第 4 阶段按移动目标推荐值勾选(速度/加速度/Jerk/
                                位置速度环/速度前馈放开)。57 项。
  · AUTO 任务航点            —— 给"预先上传航线、按航点执行"用(转场/测绘/定点作业)。
                                第 4 阶段取值改走 AUTO 文档区间，垂直上升/下降也按
                                AUTO 建议放开，并追加 15 项: 航向行为、遥控器调参、
                                EKF 数据源体检、任务安全(RTL_ALT_FINAL 等)。72 项。
  GUIDED 与 AUTO 底层共用 WPNAV 导航层，所以第 1/2/3/5/6/8 阶段完全通用。

二、推荐顺序
  0) 校准: 气压 / 水平 / 六面加速度计 / 罗盘(右侧按钮，六面有向导)。
  1) 用途=首飞，填输入 → ① 生成方案 → ② 读取体检 → ③ 一键写入。
  2) 重启飞控 → 装桨 → STABILIZE 看姿态 → MAV_CMD_NAV_TAKEOFF 起飞 2~3m
     → 悬停 ≥30s 落地上锁(学习悬停油门 MOT_HOVER_LEARN=2)。
  3) 起飞手感 OK 后二选一:
     · 外部系统实时发目标 → 用途切「GUIDED 外部引导追踪」→ 重新 ①②③。
     · 上传固定航线执行   → 用途切「AUTO 任务航点」→ 重新 ①②③。
  4) GUIDED 试飞: 先只发静止目标 → 再开位置+速度前馈 → 最后才追移动目标。
     AUTO 试跑: 先 1 条短航线 → 确认航向/围栏/RTL → 再跑全程任务。

三、第 4/5 阶段: 水平导航 + 垂直 参数怎么派生 (固件默认来自 4.6 参数表)
  参数             固件默认   首飞   GUIDED 追踪(软/标/激)   AUTO 任务(软/标/激)   说明
  WPNAV_SPEED      1000      800    1200 / 1500 / 2000     1200 / 1500 / 2000  cm/s  上限 2000
  WPNAV_ACCEL      100~250   150    300 / 400 / 500        300 / 400 / 500     cm/s² 上限 500
  WPNAV_JERK       1         5      8 / 12 / 20            12 / 15 / 20        m/s³  范围 1~20
  WPNAV_RADIUS     200       300    300 / 200 / 100        300 / 200 / 100     cm    到达容差
  PSC_POSXY_P      1.0       1.0    1.0 / 1.2 / 1.5        1.2 / 1.4 / 1.8           AUTO 文档 1.2~1.8
  PSC_VELXY_P      2.0       1.5    2.0 / 2.2 / 2.5        1.5 / 2.0 / 2.2           AUTO 文档 1.5~2.2
  PSC_VELXY_I      1.0       0.7    0.7 / 1.0 / 1.0        0.7 / 1.0 / 1.0           AUTO 文档 0.5~1.0
  PSC_VELXY_FF     0         0.1    0.2 / 0.3 / 0.5        0.1 / 0.1 / 0.2           ★追踪必开，AUTO 只给少量
  PSC_JERK_XY      5         5      8 / 12 / 20            8 / 12 / 20               范围 1~20
  WPNAV_SPEED_UP   250       100    同首飞档位             300 / 400 / 500     cm/s   AUTO 文档 300~500
  WPNAV_SPEED_DN   150       150    150 / 200 / 300        200 / 300 / 400     cm/s   AUTO 文档 200~400
  WPNAV_ACCEL_Z    —         150    同首飞档位             300 / 400 / 500     cm/s²  AUTO 文档 500~800→封顶 500
  PSC_JERK_Z       —         8      同首飞档位             10 / 14 / 20               AUTO 文档 10~20
  PSC_VELZ_FF      0         0.1    0.2 / 0.3 / 0.5        0.1 / 0.2 / 0.3            高度连续变化时给前馈
  (档位 = 左上「更保守 / 标准 / 更激进」；首飞的 WPNAV_SPEED_UP/ACCEL_Z/PSC_JERK_Z
   走档位 80·100·150 / 120·150·200 / 6·8·12，AUTO 起改走 AUTO 文档区间。)

  调参口诀(GUIDED 追踪与 AUTO 任务通用，两者底层同一套 WPNAV):
  · 追不上/跑不动       → 先升 WPNAV_ACCEL、WPNAV_JERK，再升 WPNAV_SPEED、PSC_POSXY_P
  · 超调、来回摆       → 降 WPNAV_ACCEL、WPNAV_JERK、PSC_POSXY_P
  · 顶风有偏移         → 升 PSC_VELXY_I(顶到 1.0)
  · 反复"到不了"       → 升 WPNAV_RADIUS(先 200~300，稳了再收到 100)
  · 已发速度前馈仍滞后 → 升 PSC_VELXY_FF(GUIDED 0.3~0.5)
  · AUTO 航点附近来回修正 → 降 PSC_POSXY_P(1.2 以上在 AUTO 容易过冲)、升 WPNAV_RADIUS
  · 任务时间太长       → WPNAV_SPEED + WPNAV_ACCEL 一起提；接近目标区再用
                         DO_CHANGE_SPEED 命令降速

四、与文档不同的地方(以 ArduPilot 官方参数表/源码为准，已按官方实现)
  1) WPNAV_ACCEL 固件声明范围只有 50~500 cm/s²，文档写 800~1500 是超范围的；
     本工具按 500 封顶。真要更猛，靠 WPNAV_SPEED + 倾角(ANGLE_MAX)而不是它。
  2) WPNAV_JERK 范围 1~20 m/s³(默认 1)，文档"10~30"的上限无效。
  3) MOT_THST_EXPO: 桨越大应越高(5寸0.55/10寸0.65/20寸0.75)；
     "大桨降到0.2~0.3"只适用于电调已内置线性化(勾选该开关才取 0.10)。
  4) PSC_ACCZ_P/I 在 4.7 起改名 PSC_D_ACC_P/I 且数值缩小 10 倍；
     照抄"P=悬停油门"会大 10 倍，本工具按固件实际参数名自动换算。
     同理 WP_SPD / WP_ACC / WP_RADIUS_M / PSC_NE_* / PSC_D_VEL_FF 全部自动探测。
  5) EKF 噪声与"外部目标坐标精度"无关: 外部系统给的是目标点，不是飞机自身位置。
     飞机自己不是 RTK/双天线就别动 EK3_* (第 7 阶段默认不勾选)。
  6) FENCE_RADIUS 固件范围 30~10000m(文档写 20m 会被夹到 30m)；
     BATT_LOW_VOLT 固件默认 10.5V，6S 机器永远不触发，本工具按 3.5V×串数 写入。
  7) 《AUTO模式专用速查表》按官方参数表核对后的 4 处:
     · WPNAV_SPEED 写的 2000~2500 → 固件上限 2000 cm/s，本工具按 2000 封顶；
     · WPNAV_ACCEL 写的 800~1500 → 固件上限 500 cm/s²，按 500 封顶；
     · WPNAV_ACCEL_Z 写的 500~800 → 固件上限 500 cm/s²，按 500 封顶；
     · WPNAV_JERK 写的 15~30 → 范围 1~20，上限按 20。
     另: 文档的 EK3_POS_M_NSE 实际参数名是 EK3_POSNE_M_NSE(本工具用真名)。
  8) FS_GCS_TIMEOUT 范围 2~120s(固件默认 5，文档 3~10)；DISARM_DELAY 范围 0~127s
     (默认 10)；FS_EKF_ACTION 在 4.6 只有 1/2/3，4.7 起才多一个 0=Report only；
     WP_YAW_BEHAVIOR 多旋翼固件默认是 2(指下个航点、RTL 除外)而不是 1。

五、地面站侧(GCS)检查清单 —— 参数调对了也追不上，多半是这里的问题
  · 坐标系固定用 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT；绝对高程要先减起飞点高程。
  · 消息 SET_POSITION_TARGET_GLOBAL_INT: lat_int/lon_int = 度×1e7，
    alt = 相对高度，vx/vy/vz 单位是 m/s。
  · 目标静止: type_mask = 0b0000101111111000 (只用位置)
    目标移动: type_mask = 0b0000100111111000 (位置+速度) —— 只发位置必然滞后。
  · 发送频率 ≥5Hz，推荐 10~20Hz；低于 2~3Hz 会一顿一顿。
  · 起飞阶段先用 MAV_CMD_NAV_TAKEOFF(param7=3m)，等它确认后再连发目标。
  · GUID_OPTIONS 保持默认 0：bit6 会让位置目标走航点平滑，追移动目标更滞后。
  · 数传带宽不足时先降下行遥测，别让上行目标指令被堵。
  · 紧急退出: 随时切 LOITER/ALT_HOLD/LAND，并保证 FS_GCS_ENABLE=1(第 8 阶段)。

六、安全约束(写入前自动校验)与验证顺序
  · MOT_SPIN_MIN ≥ MOT_SPIN_ARM + 0.03
  · FENCE_ALT_MAX ≥ RTL_ALT；BATT_CRT_VOLT ≤ BATT_LOW_VOLT；数值落在固件范围
  · 高速 + 小围栏半径会给提示(目标点可能"到不了")
  · 写入前自动备份到 ai_tune_backups/，出问题到「备份管理」一键回滚
  · 拆桨验证顺序: 写参数 → 拆桨 → 「拆桨实测起转」定 MOT_SPIN_ARM → 装桨 →
    STABILIZE 低油门看姿态 → GUIDED 2~3m → RTL/降落 → 悬停 30s 学悬停油门 → 上锁
  · 「拆桨实测起转」有两次确认(说明弹窗 + 必须勾选「已拆桨」才能点开始)，
    未勾选不发任何电机指令；顶栏「重启飞控」需已连接 + 已上锁 + 二次确认

七、症状对照
  离地太猛/窜天      → 降 WPNAV_SPEED_UP、核对 MOT_THST_HOVER 是否偏大
  离地犹豫/反复贴地  → 升 WPNAV_SPEED_UP、核对 MOT_THST_HOVER 是否偏小
  到顶冲高回荡        → 降 WPNAV_ACCEL_Z；高度抖动 → 查振动、降 PILOT_ACCEL_Z
  追不上移动目标      → 升 WPNAV_ACCEL / WPNAV_JERK / PSC_POSXY_P
  任务跑不满速度      → 同时升 WPNAV_ACCEL + WPNAV_JERK(只升 SPEED 没用)
  AUTO 航点附近来回修正 → 降 PSC_POSXY_P、升 WPNAV_RADIUS
  水平来回摆/抖       → 降 WPNAV_ACCEL / PSC_POSXY_P / PSC_VELXY_P
  水平顶风偏移        → 升 PSC_VELXY_I
  到达目标点附近晃    → 升 WPNAV_RADIUS
  机头指向不对        → WP_YAW_BEHAVIOR(0/1/2/3)，或任务里用 ROI / 固定航向
  任务跑完不降落      → RTL_ALT_FINAL=0(要悬停接手就给正数，且 ≤ RTL_ALT)
  RTL 后又越界        → FENCE_ALT_MAX 必须 ≥ RTL_ALT

八、工具替你做不了的
  · 罗盘/六面加速度计需要人工摆位(六面有向导弹你)
  · 首飞前: GPS≥10 星、EKF 就绪、遥控失控保护已设、桨叶方向正确、场地空旷
  · 推重比特别大的火箭机: MOT_THST_HOVER 千万别估高、WPNAV_SPEED_UP 先压低
  · 追踪作业半径较大时，把 FENCE_RADIUS 双击改到 ≥ 作业半径
  · 上传航线、编辑航点、设置 ROI 属于任务规划，本工具只负责把参数调对

九、AUTO 任务模式专属(用途=AUTO 任务航点 时多出来的 15 项)
  第 4 阶段 +4
  · WP_YAW_BEHAVIOR=2  任务中的机头航向: 0=不改 1=指下个航点(默认 HELI)
                        2=指下个航点但 RTL 除外(多旋翼默认) 3=沿 GPS 航迹方向。
                        光电/雷达要持续对准任务区 → 提前规划，别等到航线跑起来再改。
  · TUNE=10 / TUNE_MIN=500 / TUNE_MAX=2000   机上旋钮实时调 WPNAV_SPEED(默认不勾选)。
                        前提: 某个遥控通道 RCx_OPTION=219(发射机调参)，通道号自选。
                        TUNE_MIN/MAX 单位 = 被调参数单位，TUNE=10 时是 cm/s。
  第 7 阶段 +7(全部只展示不勾选，纯体检)
  · AHRS_EKF_TYPE=3 / EK3_ENABLE=1
  · EK3_SRC1_POSXY=3(GPS) / VELXY=3(GPS) / POSZ=1(气压) / VELZ=3(GPS) / YAW=1(罗盘)
    —— 这是"飞机自己用什么定位"，与外部系统给的目标坐标无关。AUTO 出现航点偏移、
       高度跳动、转弯不顺，先看这里和振动，不要只加 P。
  第 8 阶段 +4
  · RTL_ALT_FINAL=0    任务跑完/返航到家后的最终高度: 0=自动降落(推荐)，正数=悬停等指令
  · FS_GCS_TIMEOUT=5   地面站失联几秒触发保护(范围 2~120，默认 5) —— 不勾选
  · FS_EKF_ACTION=1    EKF 异常动作 1=Land 2=AltHold 3=任何模式都 Land(4.7 起 0=仅记录)
  · DISARM_DELAY=10    落地后几秒自动上锁(0=关闭) —— 不勾选
  任务命令(不是参数，要在任务规划器里加)
  · SPLINE 航点   平滑转弯，减少折线；直线航点更适合测绘
  · DO_CHANGE_SPEED 航段间动态改速: 转场快、进目标区慢，持续生效直到被覆盖
  · LOITER_TIME   到点后悬停 N 秒(拍照/对准/等外部系统确认)

十、GUIDED vs AUTO 怎么选
  对比项      GUIDED                          AUTO
  目标来源    外部实时 MAVLink 指令           预先上传的航点任务
  速度控制    MAVLink 指令 + WPNAV 参数        主要由 WPNAV_SPEED / DO_CHANGE_SPEED 决定
  路径形态    持续追目标点                     按航点顺序执行，可直线或 SPLINE
  调参重点    位置环、速度前馈、更新频率        航点速度、加速度、航向行为、任务安全
  适用场景    移动目标跟踪、外部闭环控制        固定航线、转场、测绘、定点作业
  实时性      高(10~20Hz 连发)                 低(改航点要重传任务)
  需要外部系统持续修正目标位置时选 GUIDED；需要"按任务路径执行 + 阶段性修正"时选 AUTO。
  两者都要紧急退出: 随时切 LOITER/RTL/LAND，并保证 FS_GCS_ENABLE=1。
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
        self.btn_reboot = ttk.Button(top, text='重启飞控', command=self.on_reboot_fc,
                                     state='disabled')
        self.btn_reboot.pack(side='left', padx=2)
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

        # Bottom row: 报告/体检结果 + 日志 同行(右侧面板挪到底部，腾出调参界面宽度)
        bottom = ttk.Frame(center)
        bottom.grid(row=1, column=0, sticky='ew', pady=(5, 0))
        bottom.columnconfigure(0, weight=3)
        bottom.columnconfigure(1, weight=2)

        rep_box = ttk.LabelFrame(bottom, text='报告 / 体检结果', padding=3)
        rep_box.grid(row=0, column=0, sticky='nsew', padx=(0, 5))
        self.ot_report = tk.Text(rep_box, height=8, wrap='word',
                                 bg='#1e1e1e', fg='#d4d4d4')
        self.ot_report.pack(fill='both', expand=True)

        log_box = ttk.LabelFrame(bottom, text='日志', padding=3)
        log_box.grid(row=0, column=1, sticky='nsew')
        log_box.columnconfigure(0, weight=1)
        log_box.rowconfigure(0, weight=1)
        self.txt_log = tk.Text(log_box, state='disabled', wrap='none', height=10,
                               bg='#1e1e1e', fg='#d4d4d4', font=('Consolas', 9))
        self.txt_log.grid(row=0, column=0, sticky='nsew')
        scroll_y = ttk.Scrollbar(log_box, orient='vertical', command=self.txt_log.yview)
        scroll_y.grid(row=0, column=1, sticky='ns')
        scroll_x = ttk.Scrollbar(log_box, orient='horizontal', command=self.txt_log.xview)
        scroll_x.grid(row=1, column=0, sticky='ew')
        self.txt_log.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        self.txt_log.tag_config('ok', foreground='#4ec9b0')
        self.txt_log.tag_config('err', foreground='#f44747')
        self.txt_log.tag_config('warn', foreground='#dcdcaa')
        self.txt_log.tag_config('info', foreground='#9cdcfe')

        self.build_tab_ai()
        self.build_tab_params()
        self.build_tab_onetouch()
        self.build_tab_history()
        self.build_tab_backup()
        self.build_tab_logs()

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

    def _fmt_toff(self, v):
        try:
            v = float(v)
        except (TypeError, ValueError):
            return str(v)
        if v == int(v) and abs(v) < 1e6:
            return str(int(v))
        return f'{v:.4g}'

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


    def build_tab_onetouch(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text=' 一键首飞 ')

        r1 = ttk.Frame(f)
        r1.pack(fill='x', pady=(0, 3))
        ttk.Label(r1, text='用途:').pack(side='left')
        self.ot_purpose_var = tk.StringVar(value=_OT_PURPOSES['first'])
        cmb_purpose = ttk.Combobox(r1, textvariable=self.ot_purpose_var, width=18,
                                   state='readonly', values=list(_OT_PURPOSES.values()))
        cmb_purpose.pack(side='left', padx=(2, 8))
        cmb_purpose.bind('<<ComboboxSelected>>', self.on_ot_purpose)
        ttk.Label(r1, text='桨径:').pack(side='left')
        self.ot_size_var = tk.StringVar(value='5寸')
        ttk.Combobox(r1, textvariable=self.ot_size_var, width=6, state='readonly',
                     values=['5寸', '7寸', '10寸', '13寸', '15寸', '20寸', '30寸']
                     ).pack(side='left', padx=(2, 8))
        ttk.Label(r1, text='电池:').pack(side='left')
        self.ot_cells_var = tk.StringVar(value='6S')
        ttk.Combobox(r1, textvariable=self.ot_cells_var, width=5, state='readonly',
                     values=['4S', '5S', '6S', '8S', '10S', '12S']
                     ).pack(side='left', padx=(2, 8))
        ttk.Label(r1, text='悬停油门:').pack(side='left')
        self.ot_hover_var = tk.StringVar(value='0.25')
        ttk.Entry(r1, textvariable=self.ot_hover_var, width=6).pack(side='left', padx=(2, 8))
        ttk.Label(r1, text='档位:').pack(side='left')
        self.ot_profile_var = tk.StringVar(value='标准')
        ttk.Combobox(r1, textvariable=self.ot_profile_var, width=7, state='readonly',
                     values=['更保守', '标准', '更激进']).pack(side='left', padx=(2, 8))
        self.ot_esc_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r1, text='电调已内置推力线性化',
                        variable=self.ot_esc_var).pack(side='left', padx=6)
        ttk.Button(r1, text='📖 帮助', command=self.on_ot_help).pack(side='right', padx=3)
        self.ot_status = ttk.Label(r1, text='未连接', foreground='gray')
        self.ot_status.pack(side='right', padx=6)

        r2 = ttk.Frame(f)
        r2.pack(fill='x', pady=(0, 3))
        ttk.Label(r2, text='阶段:').pack(side='left')
        self.ot_stage_vars = {}
        for sid, label in _OT_STAGES:
            v = tk.BooleanVar(value=True)
            self.ot_stage_vars[sid] = v
            ttk.Checkbutton(r2, text=label, variable=v).pack(side='left', padx=3)

        r3 = ttk.Frame(f)
        r3.pack(fill='x', pady=(0, 3))
        ttk.Button(r3, text='① 生成方案', command=self.on_ot_plan).pack(side='left', padx=3)
        ttk.Button(r3, text='② 读取当前值(体检)',
                   command=self.on_ot_read).pack(side='left', padx=3)
        ttk.Button(r3, text='③ 一键写入(先自动备份)',
                   command=self.on_ot_apply).pack(side='left', padx=3)

        r4 = ttk.Frame(f)
        r4.pack(fill='x', pady=(0, 3))
        ttk.Label(r4, text='辅助:').pack(side='left')
        for txt, kind in (('陀螺校准', 'gyro'), ('气压校准', 'baro'),
                          ('水平校准', 'level'), ('六面加速度计(向导)', 'accel')):
            ttk.Button(r4, text=txt,
                       command=lambda k=kind: self.on_ot_cal(k)).pack(side='left', padx=3)
        ttk.Button(r4, text='⚠ 拆桨实测起转(自动定 SPIN_ARM)',
                   command=self.on_ot_spin_test).pack(side='left', padx=10)
        ttk.Button(r4, text='清空报告',
                   command=lambda: self._ot_report('', clear=True)).pack(side='right', padx=3)

        cols = ('check', 'stage', 'name', 'cur', 'new', 'unit', 'basis')
        self.ot_tree = ttk.Treeview(f, columns=cols, show='headings', height=13)
        for c, w, t in (('check', 46, '写入'), ('stage', 130, '阶段'),
                        ('name', 150, '参数名'), ('cur', 95, '当前值'),
                        ('new', 95, '推荐值(双击改)'), ('unit', 66, '单位'),
                        ('basis', 470, '依据 / 说明')):
            self.ot_tree.heading(c, text=t)
            self.ot_tree.column(c, width=w, anchor='w' if c in ('name', 'basis', 'stage') else 'center')
        self.ot_tree.pack(fill='both', expand=True, pady=3)
        self.ot_tree.tag_configure('same', background='#1e3a2a')
        self.ot_tree.tag_configure('diff', background='#3a3a2a')
        self.ot_tree.tag_configure('missing', background='#3a1e1e')
        self.ot_tree.bind('<Button-1>', lambda e: self.on_check_click(self.ot_tree, e, 1))
        self.ot_tree.bind('<Double-1>', self.on_ot_edit)

        self.ot_rows = []
        self.ot_gyro_cache = None
        self.ot_cal_win = None
        self.ot_cal_event = None
        self.ot_cal_pos = None
        self.accel_cal_active = False
        self._ot_prefill_from_spec()

    def _ot_prefill_from_spec(self):
        try:
            spec = self.get_spec_dict()
        except Exception:
            return
        size = _first_int(spec.get('size_inch') or spec.get('prop_size'))
        if size:
            self.ot_size_var.set(f'{size}寸')
        cells = _first_int(spec.get('battery_s'))
        if cells:
            self.ot_cells_var.set(f'{cells}S')

    def _ot_ctx(self):
        cn = self.ot_purpose_var.get()
        purpose = next((k for k, v in _OT_PURPOSES.items() if v == cn), 'first')
        ctx = dict(size=_first_int(self.ot_size_var.get()) or 5,
                   cells=_first_int(self.ot_cells_var.get()) or 6,
                   esc_linear=bool(self.ot_esc_var.get()),
                   purpose=purpose,
                   profile={'更保守': 'soft', '标准': 'std',
                            '更激进': 'hot'}.get(self.ot_profile_var.get(), 'std'))
        try:
            ctx['hover'] = max(0.125, min(0.6875, float(self.ot_hover_var.get())))
        except (TypeError, ValueError):
            ctx['hover'] = 0.25
        if self.ot_gyro_cache:
            ctx['gyro'] = self.ot_gyro_cache
        return ctx

    def on_ot_purpose(self, event=None):
        ctx = self._ot_ctx()
        p = ctx['purpose']
        if p == 'guided':
            self.log('一键首飞: 用途=GUIDED 外部引导追踪 → 第 4 阶段水平导航参数已按推荐值勾选')
        elif p == 'auto':
            self.log('一键首飞: 用途=AUTO 任务航点 → 第 4 阶段按任务取值勾选，'
                     '并追加航向行为/遥控调参/EKF 数据源/任务安全共 15 项')
        else:
            self.log('一键首飞: 用途=首飞(安全保守) → 第 4 阶段仅展示、默认不勾选')

    def on_ot_help(self):
        win = tk.Toplevel(self)
        win.title('一键首飞 · 完整帮助说明')
        win.geometry('780x720')
        txt = tk.Text(win, wrap='word', bg='#1e1e1e', fg='#d4d4d4', font=('Consolas', 10))
        sb = ttk.Scrollbar(win, orient='vertical', command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        txt.pack(side='left', fill='both', expand=True, padx=5, pady=5)
        txt.insert('1.0', OT_HELP)
        txt.configure(state='disabled')
        ttk.Button(win, text='关闭', command=win.destroy).pack(side='bottom', pady=5)

    def _ot_report(self, text, clear=False):
        self.ot_report.configure(state='normal')
        if clear:
            self.ot_report.delete('1.0', 'end')
        if text:
            self.ot_report.insert('end', text if text.endswith('\n') else text + '\n')
            self.ot_report.see('end')
        self.ot_report.configure(state='disabled')

    def on_ot_plan(self):
        if self.fc_type and self.fc_type != 'APM_COPTER':
            messagebox.showwarning(
                '飞控类型不匹配',
                '「一键首飞」按 ArduCopter 文档实现；当前连接的是 %s。\n'
                '参数名与单位都不同，写入多半会失败。' % self.fc_type)
            return
        ctx = self._ot_ctx()
        rows = []
        for st in _ot_stages(ctx):
            if not self.ot_stage_vars[st['id']].get():
                continue
            for e in st['items']:
                rows.append(dict(stage=st['label'], entry=e, value=e['value'],
                                 basis=e['help'].split('\n')[0][:80],
                                 name=e['name'], scale=1.0, current=None,
                                 check=bool(e.get('check', True))))
        self.ot_rows = rows
        self._fill_ot_tree()
        ctx_txt = (f'用途 {_OT_PURPOSES[ctx["purpose"]]} / 桨径 {ctx["size"]}寸 / '
                   f'{ctx["cells"]}S / 悬停油门 {ctx["hover"]:.3f} / '
                   f'档位 {_OT_PROFILES[ctx["profile"]]["name"]}' +
                   (' / 电调已线性化' if ctx['esc_linear'] else ''))
        nchk = sum(1 for r in rows if r['check'])
        lines = [f'已生成 {len(rows)} 项方案 (勾选 {nchk} 项) — {ctx_txt}']
        if ctx['purpose'] == 'first':
            lines.append('  ▸ 首飞模式: 第 4 阶段(水平导航)与第 7 阶段(EKF)只展示、默认不勾选。')
            lines.append('  ▸ 首飞悬停稳定后，把"用途"切到 '
                         '「GUIDED 外部引导追踪」或「AUTO 任务航点」重新生成方案。')
        elif ctx['purpose'] == 'guided':
            lines.append('  ▸ GUIDED 追踪: 第 4 阶段已按移动目标推荐值勾选(含速度前馈 PSC_VELXY_FF)。')
            lines.append('  ▸ 追不上 → 升 WPNAV_ACCEL / WPNAV_JERK；来回摆 → 降这两项或 PSC_POSXY_P。')
            lines.append('  ▸ 围栏: FENCE_RADIUS(50m) 必须 ≥ 实际追踪作业半径，不够就双击该行改大。')
        else:
            lines.append('  ▸ AUTO 任务: 第 4 阶段按任务取值(速度/加速度/Jerk 走 AUTO 文档区间)，'
                         '并追加航向行为/遥控调参/EKF 数据源/任务安全项。')
            lines.append('  ▸ 跑不动 → 升 WPNAV_ACCEL / WPNAV_JERK；航点"到不了" → 升 WPNAV_RADIUS。')
            lines.append('  ▸ 机上调参: TUNE=10 + 某通道 RCx_OPTION=219，可旋钮实时改 WPNAV_SPEED。')
            lines.append('  ▸ 第 7 阶段数据源 7 项仅体检确认(GPS=3/气压=1/罗盘=1)，默认不勾选。')
        lines.append('  下一步: ② 读取当前值体检 → ③ 一键写入(自动备份)')
        self._ot_report('\n'.join(lines), clear=True)
        self.log(f'一键首飞: 生成 {len(rows)} 项方案 ({ctx_txt})')

    def _fill_ot_tree(self):
        self.ot_tree.delete(*self.ot_tree.get_children())
        for r in self.ot_rows:
            iid = self.ot_tree.insert(
                '', 'end',
                values=('✓' if r['check'] else '', r['stage'], r['name'],
                        '—', self._fmt_toff(r['value']), r['entry'].get('unit') or '',
                        r['basis']))
            r['iid'] = iid
            self.ot_tree.item(iid, tags=('missing',))

    def on_ot_edit(self, event=None):
        if self.ot_tree.identify_column(event.x) != '#6':
            return
        iid = self.ot_tree.identify_row(event.y)
        row = next((r for r in self.ot_rows if r.get('iid') == iid), None)
        if not row:
            return
        e = row['entry']
        rng = e.get('rng')
        try:
            cur = float(self.ot_tree.set(iid, 'new'))
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
        row['value'] = v
        self.ot_tree.set(iid, 'new', self._fmt_toff(v))
        self._ot_update_delta(row)

    def _ot_update_delta(self, row):
        iid = row.get('iid')
        if not iid:
            return
        cur, new = row.get('current'), row.get('value')
        if cur is None:
            self.ot_tree.set(iid, 'cur', '—')
            self.ot_tree.item(iid, tags=('missing',))
            return
        same = abs(new - cur) <= max(1e-6, abs(new) * 0.005)
        self.ot_tree.set(iid, 'cur',
                         f'{self._fmt_toff(cur)}' + (' ✓' if same else ''))
        self.ot_tree.item(iid, tags=('same' if same else 'diff',))
        if same and self.ot_tree.set(iid, 'check') == '✓' and row.get('auto_uncheck', True):
            self.ot_tree.set(iid, 'check', '')
            row['check'] = False

    def on_ot_read(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if not self.ot_rows:
            self.on_ot_plan()
        if not self.ot_rows:
            return
        n = len(self.ot_rows)
        self.log(f'一键首飞: 体检读取 {n} 项...')
        self.enqueue('一键首飞体检', self._do_ot_read)

    def _do_ot_read(self):
        master = self.mav_master
        hint = {}
        try:
            bat = master.recv_match(type=['BATTERY_STATUS', 'SYS_STATUS'],
                                    blocking=True, timeout=2)
            if bat:
                volt = (bat.voltage_battery / 1000.0
                        if getattr(bat, 'voltage_battery', 0) else 0)
                if bat.get_type() == 'BATTERY_STATUS' and bat.voltages and bat.voltages[0] > 0:
                    volt = bat.voltages[0] / 1000.0
                if volt > 0:
                    hint['volt'] = volt
                    hint['cells'] = round(volt / 3.7)
        except Exception:
            pass
        out = []
        for i, r in enumerate(self.ot_rows):
            res = self._resolve_param(r['entry'])
            out.append(res)
            if r['entry']['name'] == 'INS_GYRO_FILTER' and res['value']:
                self.ot_gyro_cache = int(round(res['value']))
            if i % 8 == 7:
                self.log(f'  体检进度 {i+1}/{len(self.ot_rows)}')
        self.after(0, lambda: self._show_ot_read(out, hint))

    def _show_ot_read(self, out, hint):
        verify = bool(getattr(self, 'ot_verify_after_write', False))
        self.ot_verify_after_write = False
        same = diff = miss = 0
        for r, res in zip(self.ot_rows, out):
            r['name'] = res['name']
            r['scale'] = res['scale']
            r['current'] = res['value']
            if res['value'] is None:
                miss += 1
            elif abs(res['value'] - r['value']) <= max(1e-6, abs(r['value']) * 0.005):
                same += 1
            else:
                diff += 1
            self._ot_update_delta(r)
        lines = [f'{"复读校验" if verify else "体检完成"}: '
                 f'已读取 {len(out) - miss}/{len(out)}，'
                 f'已一致 {same}，需改 {diff}，读不到 {miss}。']
        if hint.get('volt'):
            cells = hint.get('cells')
            lines.append(f'  实测电压 {hint["volt"]:.2f}V → 推算 {cells}S'
                         + ('' if cells == _first_int(self.ot_cells_var.get())
                            else f'  (与当前输入 {_first_int(self.ot_cells_var.get())}S 不一致，请核对)'))
        if self.ot_gyro_cache:
            lines.append(f'  INS_GYRO_FILTER = {self.ot_gyro_cache} Hz '
                         '(FLTD/FLTT 按它的一半计算；想换滤波频率请先改桨径再重新生成方案)')
        if miss:
            lines.append('  ⚠ 读不到的项多为 4.7 改名或该机未启用，已自动尝试新旧两套参数名')
        lines.append('  复读一致 → 重启飞控 → 装桨 → 按上方"下一步"试飞'
                     if verify else
                     '  下一步: 拆桨实测起转 → ③ 一键写入 → 重启飞控 → 装桨试飞')
        self._ot_report('\n'.join(lines), clear=not verify)
        self.log(('✅ 写入后复读校验' if verify else '✅ 一键首飞体检完成')
                 + ': %d 项需修改' % diff)

    def on_ot_apply(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if self.fc_type and self.fc_type != 'APM_COPTER':
            messagebox.showwarning('提示', '「一键首飞」仅支持 ArduCopter')
            return
        if not self.ot_rows:
            self.on_ot_plan()
        pending = []
        for r in self.ot_rows:
            if not _is_checked(self.ot_tree.set(r.get('iid', ''), 'check')):
                continue
            cur = r.get('current')
            if cur is not None and abs(r['value'] - cur) <= max(1e-6, abs(r['value']) * 0.005):
                continue
            pending.append(r)
        if not pending:
            messagebox.showinfo('提示', '没有需要写入的参数（未勾选或已与推荐值一致）')
            return
        issues = self._ot_validate(pending)
        if issues:
            if not messagebox.askyesno('参数校验未通过',
                                       '\n'.join(issues) + '\n\n仍要写入吗？（不推荐）'):
                return
        by_stage = {}
        for r in pending:
            by_stage.setdefault(r['stage'], []).append(r)
        listing = []
        for st in [l for _, l in _OT_STAGES]:
            if st in by_stage:
                listing.append(f'【{st}】')
                for r in by_stage[st]:
                    listing.append(f'  {r["name"]} → {self._fmt_toff(r["value"])}'
                                   f'{r["entry"].get("unit") or ""}')
        if not messagebox.askyesno(
                '确认一键写入',
                f'将写入 {len(pending)} 个参数（写入前自动备份）:\n\n' +
                '\n'.join(listing) + '\n\n继续？'):
            return
        self.log(f'一键首飞: 开始写入 {len(pending)} 项...')
        self.enqueue('一键首飞写入',
                     lambda: self._do_ot_apply(list(pending),
                                               f'一键首飞({len(pending)}项)'))

    def _ot_validate(self, rows):
        issues = []
        values = {r['entry']['name']: r['value'] for r in rows}
        for r in rows:
            rng = r['entry'].get('rng')
            v = r['value']
            if rng and not (rng[0] <= v <= rng[1]):
                issues.append(f'{r["entry"]["name"]}={v} 超出固件范围 {rng[0]}~{rng[1]}')
        arm, mn = values.get('MOT_SPIN_ARM'), values.get('MOT_SPIN_MIN')
        if arm is not None and mn is not None and mn < arm + 0.03 - 1e-9:
            issues.append(f'MOT_SPIN_MIN({mn}) 必须 ≥ MOT_SPIN_ARM({arm}) + 0.03')
        alt, rtl = values.get('FENCE_ALT_MAX'), values.get('RTL_ALT')
        if alt is not None and rtl is not None and alt < rtl / 100.0 - 1e-9:
            issues.append(f'FENCE_ALT_MAX({alt}m) 必须 ≥ RTL_ALT({rtl}cm = {rtl/100:.2f}m)')
        low, crt = values.get('BATT_LOW_VOLT'), values.get('BATT_CRT_VOLT')
        if low is not None and crt is not None and crt > low + 1e-9:
            issues.append(f'BATT_CRT_VOLT({crt}) 必须 ≤ BATT_LOW_VOLT({low})')
        p, i, hover = (values.get('PSC_ACCZ_P'), values.get('PSC_ACCZ_I'),
                       values.get('MOT_THST_HOVER'))
        if p is not None and hover is not None and p < 0.2:
            issues.append(f'PSC_ACCZ_P({p}) 低于固件下限 0.2 — 悬停油门 {hover} 过低，'
                          '请提高"悬停油门"输入(0.2 起)')
        if i is not None and hover is not None and abs(i - 2 * hover) > 1e-6:
            issues.append(f'PSC_ACCZ_I({i}) ≠ 2×悬停油门({2*hover}) — 派生值被手动改过，请确认')
        speed, radius = values.get('WPNAV_SPEED'), values.get('WPNAV_RADIUS')
        if speed is not None and radius is not None and speed >= 1500 and radius < 100:
            issues.append(f'WPNAV_RADIUS({radius}cm) 太小而 WPNAV_SPEED({speed}cm/s) 很快，'
                          '目标点附近可能一直"到不了"；建议 ≥100')
        dn, up = values.get('WPNAV_SPEED_DN'), values.get('WPNAV_SPEED_UP')
        if dn is not None and up is not None and dn > up * 3:
            issues.append(f'WPNAV_SPEED_DN({dn}) 远大于 WPNAV_SPEED_UP({up})，下降会比爬升快很多')
        tmin, tmax = values.get('TUNE_MIN'), values.get('TUNE_MAX')
        if tmin is not None and tmax is not None and tmin >= tmax - 1e-9:
            issues.append(f'TUNE_MIN({tmin}) 必须 < TUNE_MAX({tmax})，'
                          '否则旋钮一拧就把参数写成反向/超范围值')
        fin, rtl2 = values.get('RTL_ALT_FINAL'), values.get('RTL_ALT')
        if fin is not None and rtl2 is not None and fin > rtl2 + 1e-9:
            issues.append(f'RTL_ALT_FINAL({fin}cm) 必须 ≤ RTL_ALT({rtl2}cm)，'
                          '最终高度不能高过返航爬升高度')
        return issues

    def _do_ot_apply(self, pending, label):
        resolved = []
        for r in pending:
            if r.get('current') is not None:
                res = dict(name=r['name'], scale=r['scale'], value=r['current'])
            else:
                res = self._resolve_param(r['entry'])
            resolved.append((r, res))
        names = [res['name'] for _, res in resolved]
        backup_path = backup_params(self.mav_master, names, self.get_spec_dict(),
                                    tag='onetouch')
        self.log(f'📦 已备份: {os.path.basename(backup_path)}')
        success, failed = 0, []
        for r, res in resolved:
            if res['value'] is None and r.get('current') is None:
                self.log(f'  ⚠ {r["entry"]["name"]} 读不到，仍按 {res["name"]} 写入')
            val = r['value'] * res['scale']
            if set_param(self.mav_master, res['name'], val):
                success += 1
                extra = '' if abs(val - r['value']) < 1e-9 else \
                    f' (原 {self._fmt_toff(r["value"])}{r["entry"].get("unit") or ""})'
                self.log(f'  ✅ {res["name"]} = {self._fmt_toff(val)}{extra}')
            else:
                failed.append(res['name'])
                self.log(f'  ❌ {res["name"]} 写入失败')
        self.after(0, lambda: self._after_ot_apply(success, failed, backup_path, label))

    def _after_ot_apply(self, success, failed, backup_path, label):
        if failed:
            messagebox.showwarning('结果',
                                   f'成功 {success} 个，失败 {len(failed)} 个: {", ".join(failed)}')
        else:
            messagebox.showinfo('结果', f'{label} 全部 {success} 个参数写入成功')
        self.round += 1
        self.history.append({
            'round': self.round,
            'time': time.strftime('%H:%M:%S'),
            'written': success,
            'summary': f'一键首飞: {label}',
            'backup': os.path.basename(backup_path)
        })
        self.refresh_history()
        purpose = self._ot_ctx()['purpose']
        if purpose == 'guided':
            nxt = ('  追踪验证: 重启飞控 → 起飞 3m 切 GUIDED → 先发静止目标(纯位置) →\n'
                   '  再开位置+速度前馈(10~20Hz) → 确认无超调后再追移动目标 →\n'
                   '  追不上升 WPNAV_ACCEL/JERK，来回摆降它们或 PSC_POSXY_P')
        elif purpose == 'auto':
            nxt = ('  任务验证: 重启飞控 → 上传航线(必要时 SPLINE 航点平滑转弯) →\n'
                   '  任务里可加 DO_CHANGE_SPEED 转场加速/进区减速、LOITER_TIME 等待 →\n'
                   '  试跑 1 条短航线，确认航向行为(WP_YAW_BEHAVIOR)符合传感器指向需求 →\n'
                   '  ★ 全程保持 LOITER/RTL 随时可切；跑不满速度升 WPNAV_ACCEL/JERK')
        else:
            nxt = ('  建议: 重启飞控 → 装桨 → STABILIZE 确认姿态方向 → GUIDED 起飞 2~3m\n'
                   '  → 悬停 30s 落地上锁(学习悬停油门) → 看日志振动/EKF → 再飞高\n'
                   '  → 稳定后把"用途"切到「GUIDED 外部引导追踪」或「AUTO 任务航点」')
        self._ot_report(f'写入完成: 成功 {success}，失败 {len(failed)}\n{nxt}', clear=True)
        self.ot_verify_after_write = True
        if not failed:
            self.on_ot_read()

    # ===== 辅助校准 =====
    def on_ot_cal(self, kind):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if kind == 'accel':
            self._start_accel_cal()
            return
        tips = {'gyro': '陀螺校准: 请把飞机静置在平稳处，校准期间不要移动。',
                'baro': '气压校准: 请保持静止、避免风吹与遮挡。',
                'level': '水平校准: 请把飞机按真实水平飞行姿态摆好(装好桨后的水平)。'}
        if not messagebox.askyesno('校准确认', tips[kind] + '\n\n开始？'):
            return
        self.log(f'开始校准: {kind}')
        self.enqueue(f'校准({kind})', lambda: self._do_ot_cal(kind))

    def _do_ot_cal(self, kind):
        from pymavlink import mavutil
        master = self.mav_master
        p1 = 1.0 if kind == 'gyro' else 0.0
        p3 = 1.0 if kind == 'baro' else 0.0
        p5 = 2.0 if kind == 'level' else 0.0
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
            p1, 0, p3, 0, p5, 0, 0)
        ok, text = self._wait_ack(master, 8.0, extra=('STATUSTEXT',))
        self.log(f'校准 {kind}: ' + ('✅ 已完成 ' if ok else '⚠ ') + (text or '无返回'))

    def _wait_ack(self, master, timeout=5.0, extra=()):
        from pymavlink import mavutil
        t_end = time.time() + timeout
        texts = []
        while time.time() < t_end:
            m = master.recv_match(type=['COMMAND_ACK'] + list(extra),
                                  blocking=True, timeout=0.5)
            if m is None:
                continue
            if m.get_type() == 'COMMAND_ACK':
                if m.command in (mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                                 mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                 getattr(mavutil.mavlink, 'MAV_CMD_ACCELCAL_VEHICLE_POS', -1)):
                    res = getattr(m, 'result', 0)
                    return (res == mavutil.mavlink.MAV_RESULT_ACCEPTED,
                            f'result={res}')
            elif m.get_type() == 'STATUSTEXT':
                txt = m.text.decode('utf-8', 'ignore') if isinstance(m.text, bytes) else str(m.text)
                texts.append(txt)
                if any(k in txt for k in ('calibrat', 'Calibrat', 'success', 'Success',
                                          'failed', 'Failed', 'complete')):
                    return ('failed' not in txt.lower(), txt)
        return (None, texts[-1] if texts else '')

    def _start_accel_cal(self):
        if self.accel_cal_active:
            return
        if not messagebox.askyesno(
                '六面加速度计校准',
                '需要逐面摆放飞机(水平/左/右/机头下/机头上/背面)，'
                '每面摆好静止后点「该面已摆好」。\n\n'
                '⚠ 校准期间不要执行其它操作；如需中断请重启飞控。\n开始？'):
            return
        self.accel_cal_active = True
        self._open_accel_cal_win()
        threading.Thread(target=self._do_accel_cal, daemon=True).start()

    def _open_accel_cal_win(self):
        win = tk.Toplevel(self)
        win.title('六面加速度计校准')
        win.geometry('460x210')
        self.ot_cal_win = win
        self.ot_cal_event = threading.Event()
        self.ot_cal_pos = tk.StringVar(value='正在启动校准...')
        ttk.Label(win, textvariable=self.ot_cal_pos, wraplength=420,
                  font=('微软雅黑', 11)).pack(padx=12, pady=14, fill='x')
        btn = ttk.Button(win, text='该面已摆好(发送确认)', state='disabled')
        btn.pack(pady=6)
        self.ot_cal_btn = btn
        ttk.Label(win, text='提示: LEVEL 最重要，务必按真实水平飞行姿态摆好；'
                            '其余各面 ±20° 可接受，静止最关键。',
                  wraplength=420, foreground='gray').pack(padx=12, pady=6)
        ttk.Label(win, text='如需中断: 重启飞控',
                  foreground='gray').pack(side='bottom', pady=6)

    def _do_accel_cal(self):
        from pymavlink import mavutil
        master = self.mav_master
        pos_name = {1: 'LEVEL 水平', 2: 'LEFT 左侧', 3: 'RIGHT 右侧',
                    4: 'NOSEDOWN 机头朝下', 5: 'NOSEUP 机头朝上', 6: 'BACK 背面朝上'}
        CMD = mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
            0, 0, 0, 0, 1, 0, 0)
        self.log('六面校准已启动，按提示逐面摆放')
        t_end = time.time() + 300
        done = None
        while time.time() < t_end and done is None:
            m = master.recv_match(type=['COMMAND_LONG', 'STATUSTEXT', 'COMMAND_ACK'],
                                  blocking=True, timeout=0.5)
            if m is None:
                continue
            if m.get_type() == 'COMMAND_LONG' and int(m.command) == CMD:
                pos = int(m.param1)
                if pos >= 16777215:
                    done = (pos == 16777215)
                    break
                if pos in pos_name:
                    self.after(0, lambda p=pos: self._ot_cal_prompt(p))
                    if not self.ot_cal_event.wait(timeout=180):
                        break
                    self.ot_cal_event.clear()
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        CMD, 0, float(pos), 0, 0, 0, 0, 0, 0)
            elif m.get_type() == 'STATUSTEXT':
                txt = (m.text.decode('utf-8', 'ignore')
                       if isinstance(m.text, bytes) else str(m.text))
                if txt.lower().startswith('place'):
                    self.after(0, lambda t=txt: self._ot_cal_prompt_text(t))
        self.accel_cal_active = False
        msg = '✅ 六面加速度计校准完成' if done else \
              '⚠ 六面校准未收到完成信号(可重试，或重启飞控后改用地面站)'
        self.log(msg)
        self.after(0, lambda: self._ot_cal_finish(msg))

    def _ot_cal_prompt(self, pos):
        names = {1: 'LEVEL 水平', 2: 'LEFT 左侧', 3: 'RIGHT 右侧',
                 4: 'NOSEDOWN 机头朝下', 5: 'NOSEUP 机头朝上', 6: 'BACK 背面朝上'}
        self.ot_cal_pos.set(f'请摆放: {names.get(pos, pos)}  —  摆好并静止后点下面按钮')
        self.ot_cal_btn.configure(state='normal',
                                  command=lambda: self._ot_cal_confirm(pos))

    def _ot_cal_prompt_text(self, text):
        self.ot_cal_pos.set(text)

    def _ot_cal_confirm(self, pos):
        self.ot_cal_btn.configure(state='disabled', text='已确认，等待下一面...')
        self.ot_cal_event.set()
        self.after(4000, lambda: self.ot_cal_btn.configure(
            state='disabled', text='该面已摆好(发送确认)'))

    def _ot_cal_finish(self, msg):
        if self.ot_cal_win and self.ot_cal_win.winfo_exists():
            self.ot_cal_pos.set(msg)
        self._ot_report(msg)

    # ===== 拆桨实测 MOT_SPIN_ARM (ESC 遥测判定起转点) =====
    def on_ot_spin_test(self):
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if self.fc_type and self.fc_type != 'APM_COPTER':
            messagebox.showwarning('提示', '电机起转实测仅支持 ArduCopter')
            return
        if not messagebox.askyesno(
                '⚠ 拆桨实测',
                '本功能会自动逐档升油门让电机转动，用来测定起转点。\n\n'
                '⚠⚠ 必须先拆除所有桨叶！\n'
                '需要 ESC 遥测(DShot + 遥测线/双向 DShot)；没有遥测会中止。\n'
                '飞控需处于落地、未解锁、安全开关未锁状态。\n\n'
                '确认已拆桨并开始？(下一步还有一次二次确认)'):
            return
        if not self._ot_spin_confirm():
            self.log('⚠️ 已取消拆桨实测(未通过二次确认，未发送任何电机指令)')
            return
        self.log('拆桨实测: 从 6% 逐档升油门寻找起转点...')
        self.enqueue('拆桨实测起转', self._do_ot_spin_test)

    def _ot_spin_confirm(self):
        """拆桨二次确认: 必须勾选「已拆桨」才能点开始; 测试/冒烟可整体打桩替换"""
        result = {'ok': False}
        win = tk.Toplevel(self)
        win.title('⚠ 二次确认 · 拆桨')
        win.transient(self)
        win.resizable(False, False)
        pad = {'padx': 12}
        ttk.Label(win, text='⚠⚠ 第二次确认：桨叶是否已全部拆除？',
                  font=('微软雅黑', 11, 'bold'),
                  foreground='#dcdcaa').grid(row=0, column=0, columnspan=2,
                                             sticky='w', pady=(12, 6), **pad)
        ttk.Label(win, justify='left', text=(
            '未拆桨起转 = 桨叶高速甩出，可能割伤人员或打坏设备。\n'
            '· 本测试从 6% 起步逐档升油门，最高到 40%(约 1500~2000 RPM)\n'
            '· 需要 ESC 遥测(DShot + 遥测线/双向 DShot)，无遥测会中止\n'
            '· 飞控落地放置并固定牢靠，人员与手指全程远离电机\n'
            '· 若有 ESC 遥测以外的情况(未落地/安全开关锁住)飞控会拒绝'
        )).grid(row=1, column=0, columnspan=2, sticky='w', **pad)
        confirmed = tk.BooleanVar(value=False)
        bar = ttk.Frame(win)
        bar.grid(row=3, column=0, columnspan=2, sticky='e', pady=12, **pad)
        btn_start = ttk.Button(bar, text='✓ 已拆桨，开始实测',
                               state='disabled',
                               command=lambda: (result.update(ok=True), win.destroy()))
        ttk.Button(bar, text='取消', command=win.destroy).pack(side='right', padx=4)
        btn_start.pack(side='right', padx=4)
        ttk.Checkbutton(
            win, text='我已拆下全部桨叶，确认人员与手指远离电机、飞控已固定',
            variable=confirmed,
            command=lambda: btn_start.configure(
                state='normal' if confirmed.get() else 'disabled')
        ).grid(row=2, column=0, columnspan=2, sticky='w', pady=(8, 0), **pad)
        win.protocol('WM_DELETE_WINDOW', win.destroy)
        win.update_idletasks()
        x = self.winfo_rootx() + max((self.winfo_width() - win.winfo_width()) // 2, 0)
        y = self.winfo_rooty() + max((self.winfo_height() - win.winfo_height()) // 2, 0)
        win.geometry(f'+{x}+{y}')
        win.grab_set()
        win.focus_set()
        self.wait_window(win)
        return bool(result['ok'])

    def _do_ot_spin_test(self):
        from pymavlink import mavutil
        master = self.mav_master

        def motor_test(seq, pct, timeout=2.0):
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                float(seq), float(mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT),
                float(pct), float(timeout), 1.0, 0.0, 0.0)
            txts = []
            t_end = time.time() + 1.2
            while time.time() < t_end:
                m = master.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'],
                                      blocking=True, timeout=0.3)
                if m is None:
                    continue
                if m.get_type() == 'COMMAND_ACK' and \
                        m.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
                    return getattr(m, 'result', 0), txts
                if m.get_type() == 'STATUSTEXT':
                    t = (m.text.decode('utf-8', 'ignore')
                         if isinstance(m.text, bytes) else str(m.text))
                    txts.append(t)
                    if 'fail' in t.lower() or 'cancel' in t.lower():
                        return -1, txts
            return 0, txts

        def read_rpm(idx, span=0.8):
            best = 0
            t_end = time.time() + span
            while time.time() < t_end:
                m = master.recv_match(type='ESC_TELEMETRY_1_TO_4',
                                      blocking=True, timeout=0.3)
                if m is None:
                    continue
                try:
                    best = max(best, int(m.rpm[idx]))
                except Exception:
                    pass
            return best

        spin = None
        for pct in range(6, 41, 2):
            res, txts = motor_test(1, pct)
            if res not in (0, mavutil.mavlink.MAV_RESULT_ACCEPTED):
                self.log('❌ 电机测试被飞控拒绝: ' + ('; '.join(txts) or f'result={res}'))
                self.after(0, lambda: self._ot_report(
                    '❌ 电机测试被拒(未落地/安全开关/未校准遥控)，改用人工方法:\n'
                    '   拆桨 → 地面站电机测试逐档升油门 → 记录刚好起转的百分比 → '
                    '填进 MOT_SPIN_ARM(+1%)', clear=True))
                return
            rpm = read_rpm(0)
            self.log(f'  电机1 @ {pct}%: RPM={rpm}')
            if rpm > 0:
                spin = pct
                break
        if spin is None:
            self.after(0, lambda: self._ot_report(
                '❌ 40% 以内电机1 仍未起转，或未收到 ESC 遥测(RPM=0)。\n'
                '   需要 DShot + 遥测线；否则请拆桨人工测定 MOT_SPIN_ARM。', clear=True))
            return
        for seq in (2, 3, 4):
            pct = spin
            while pct <= spin + 8:
                res, _ = motor_test(seq, pct)
                if read_rpm(seq - 1) > 0:
                    break
                pct += 2
            if pct > spin + 8:
                self.log(f'  ⚠ 电机{seq} 在 {spin}~{spin+8}% 未检测到转速，取上限')
                spin = max(spin, spin + 8)
            else:
                spin = max(spin, pct)
        time.sleep(2.5)
        arm = round(min(0.2, spin / 100.0 + 0.01), 2)
        mn = round(min(0.25, arm + 0.03), 2)
        self.ot_spin_result = dict(arm=arm, min=mn, spin=spin)
        for r in self.ot_rows:
            if r['entry']['name'] == 'MOT_SPIN_ARM':
                r['value'] = arm
                if r.get('iid'):
                    self.ot_tree.set(r['iid'], 'new', self._fmt_toff(arm))
            elif r['entry']['name'] == 'MOT_SPIN_MIN':
                r['value'] = mn
                if r.get('iid'):
                    self.ot_tree.set(r['iid'], 'new', self._fmt_toff(mn))
        msg = (f'✅ 起转点实测 ≈ {spin}% → 建议 MOT_SPIN_ARM={arm}、MOT_SPIN_MIN={mn}\n'
               '  已自动填入方案表；确认无误后点「③ 一键写入」。')
        self.log(f'拆桨实测完成: 起转 {spin}% → ARM={arm} MIN={mn}')
        self.after(0, lambda: self._ot_report(msg, clear=True))

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
            self.btn_reboot.configure(state='normal')
            self.fc_lbl.configure(text=f'FC: {self.fc_type}')
            self.link_lbl.configure(text='● 已连接', foreground='green')
            self.log(f'✅ 已连接 {self.fc_type} (sysid={master.target_system})')
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
        self.btn_reboot.configure(state='disabled')
        self.fc_lbl.configure(text='FC: AUTO')
        self.link_lbl.configure(text='○ 未连接', foreground='gray')
        if self.mav_master:
            disconnect(self.mav_master)
            self.mav_master = None
            self.fc_type = None
        self.log('已断开连接')

    # ===== 飞控重启 (顶栏按钮: 需已连接 + 已上锁 + 二次确认) =====
    def _fc_armed(self, timeout=1.0):
        """读一条心跳判断是否已解锁; 读不到心跳返回 None(未知)"""
        master = self.mav_master
        if not master:
            return None
        try:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        except Exception:
            return None
        if hb is None:
            return None
        from pymavlink import mavutil
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _confirm_reboot(self):
        """重启二次确认; 测试/冒烟可整体打桩替换"""
        return messagebox.askyesno(
            '重启飞控',
            '确认重启飞控？\n\n'
            '· 已解锁会被直接拒绝(需先上锁)\n'
            '· 重启约 10~30s，期间心跳中断，'
            'USB 虚拟串口可能需要重新连接\n'
            '· 正在执行的读参数/写参数等任务会被中断\n\n'
            '重启后请点「断开」再「连接」刷新链路。')

    def on_reboot_fc(self):
        """顶栏「重启飞控」: 未连接/任务运行中/已解锁 → 拒绝, 二次确认后入队"""
        if not self.connected:
            messagebox.showwarning('提示', '请先连接飞控')
            return
        if self.action_busy.is_set():
            messagebox.showwarning('提示', '当前有任务运行中，请先等待完成再重启')
            return
        if self._fc_armed():
            messagebox.showerror('重启飞控', '❌ 飞控已解锁，请先上锁再重启')
            return
        if not self._confirm_reboot():
            self.log('已取消重启飞控')
            return
        self.enqueue('重启飞控', self._do_reboot_fc)

    def _do_reboot_fc(self):
        """MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(param1=1); 心跳中断≥2.5s 判定重启开始"""
        from pymavlink import mavutil
        master = self.mav_master
        if self._fc_armed(timeout=2.0):
            self.log('❌ 已解锁，禁止重启飞控；请先上锁再重启')
            return
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
            1, 0, 0, 0, 0, 0, 0)
        self.log('🔁 已发送重启命令(PARAM1=1)，等待心跳中断判定...')
        t0 = time.time()
        last = time.time()
        while time.time() - t0 < 15:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if hb is not None:
                last = time.time()
            elif time.time() - last >= 2.5:
                self.log('✅ 心跳已中断，飞控重启中(约 10~30s 后重新上线，'
                         'USB 虚拟串口可能需重新连接)')
                self.after(0, lambda: self.link_lbl.configure(
                    text='○ 重启中(请断开后重新连接)', foreground='orange'))
                return
        self.log('⚠️ 未检测到心跳中断：重启命令可能未被执行(超时 15s)')

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
            master = self.mav_master
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
        if getattr(self, 'accel_cal_active', False):
            self.log(f'⚠️ 六面校准进行中，已忽略: {name}')
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