// 本文件由 scripts/gen_ot_data.py 自动生成, 请勿手工编辑。
// 数据来源: docs/ai_pid_tune.py
window.OT_DATA = {
 "help": "一键首飞 · 帮助说明 (GUIDED 追踪 / AUTO 任务)\n========================================================\n一、这个页解决什么\n  输入 桨径 / 电池串数 / 悬停油门 / 档位 / 用途 → 自动派生全量参数 →\n  体检读取 → 交叉校验 → 自动备份 → 一键写入 → 报告。\n  共 8 个阶段，每行都有\"依据\"列，可勾选、可双击改值。\n\n  用途三选一(左上角):\n  · 首飞(安全保守)          —— 先把机器安全飞起来。第 4 阶段(水平导航)、\n                                第 7 阶段(EKF)只展示不勾选，避免一次改太多。\n                                57 项。\n  · GUIDED 外部引导追踪      —— 给\"地面站 10~20Hz 发目标坐标追移动目标\"用，\n                                第 4 阶段按移动目标推荐值勾选(速度/加速度/Jerk/\n                                位置速度环/速度前馈放开)。57 项。\n  · AUTO 任务航点            —— 给\"预先上传航线、按航点执行\"用(转场/测绘/定点作业)。\n                                第 4 阶段取值改走 AUTO 文档区间，垂直上升/下降也按\n                                AUTO 建议放开，并追加 15 项: 航向行为、遥控器调参、\n                                EKF 数据源体检、任务安全(RTL_ALT_FINAL 等)。72 项。\n  GUIDED 与 AUTO 底层共用 WPNAV 导航层，所以第 1/2/3/5/6/8 阶段完全通用。\n\n二、推荐顺序\n  0) 校准: 气压 / 水平 / 六面加速度计 / 罗盘(右侧按钮，六面有向导)。\n  1) 用途=首飞，填输入 → ① 生成方案 → ② 读取体检 → ③ 一键写入。\n  2) 重启飞控 → 装桨 → STABILIZE 看姿态 → MAV_CMD_NAV_TAKEOFF 起飞 2~3m\n     → 悬停 ≥30s 落地上锁(学习悬停油门 MOT_HOVER_LEARN=2)。\n  3) 起飞手感 OK 后二选一:\n     · 外部系统实时发目标 → 用途切「GUIDED 外部引导追踪」→ 重新 ①②③。\n     · 上传固定航线执行   → 用途切「AUTO 任务航点」→ 重新 ①②③。\n  4) GUIDED 试飞: 先只发静止目标 → 再开位置+速度前馈 → 最后才追移动目标。\n     AUTO 试跑: 先 1 条短航线 → 确认航向/围栏/RTL → 再跑全程任务。\n\n三、第 4/5 阶段: 水平导航 + 垂直 参数怎么派生 (固件默认来自 4.6 参数表)\n  参数             固件默认   首飞   GUIDED 追踪(软/标/激)   AUTO 任务(软/标/激)   说明\n  WPNAV_SPEED      1000      800    1200 / 1500 / 2000     1200 / 1500 / 2000  cm/s  上限 2000\n  WPNAV_ACCEL      100~250   150    300 / 400 / 500        300 / 400 / 500     cm/s² 上限 500\n  WPNAV_JERK       1         5      8 / 12 / 20            12 / 15 / 20        m/s³  范围 1~20\n  WPNAV_RADIUS     200       300    300 / 200 / 100        300 / 200 / 100     cm    到达容差\n  PSC_POSXY_P      1.0       1.0    1.0 / 1.2 / 1.5        1.2 / 1.4 / 1.8           AUTO 文档 1.2~1.8\n  PSC_VELXY_P      2.0       1.5    2.0 / 2.2 / 2.5        1.5 / 2.0 / 2.2           AUTO 文档 1.5~2.2\n  PSC_VELXY_I      1.0       0.7    0.7 / 1.0 / 1.0        0.7 / 1.0 / 1.0           AUTO 文档 0.5~1.0\n  PSC_VELXY_FF     0         0.1    0.2 / 0.3 / 0.5        0.1 / 0.1 / 0.2           ★追踪必开，AUTO 只给少量\n  PSC_JERK_XY      5         5      8 / 12 / 20            8 / 12 / 20               范围 1~20\n  WPNAV_SPEED_UP   250       100    同首飞档位             300 / 400 / 500     cm/s   AUTO 文档 300~500\n  WPNAV_SPEED_DN   150       150    150 / 200 / 300        200 / 300 / 400     cm/s   AUTO 文档 200~400\n  WPNAV_ACCEL_Z    —         150    同首飞档位             300 / 400 / 500     cm/s²  AUTO 文档 500~800→封顶 500\n  PSC_JERK_Z       —         8      同首飞档位             10 / 14 / 20               AUTO 文档 10~20\n  PSC_VELZ_FF      0         0.1    0.2 / 0.3 / 0.5        0.1 / 0.2 / 0.3            高度连续变化时给前馈\n  (档位 = 左上「更保守 / 标准 / 更激进」；首飞的 WPNAV_SPEED_UP/ACCEL_Z/PSC_JERK_Z\n   走档位 80·100·150 / 120·150·200 / 6·8·12，AUTO 起改走 AUTO 文档区间。)\n\n  调参口诀(GUIDED 追踪与 AUTO 任务通用，两者底层同一套 WPNAV):\n  · 追不上/跑不动       → 先升 WPNAV_ACCEL、WPNAV_JERK，再升 WPNAV_SPEED、PSC_POSXY_P\n  · 超调、来回摆       → 降 WPNAV_ACCEL、WPNAV_JERK、PSC_POSXY_P\n  · 顶风有偏移         → 升 PSC_VELXY_I(顶到 1.0)\n  · 反复\"到不了\"       → 升 WPNAV_RADIUS(先 200~300，稳了再收到 100)\n  · 已发速度前馈仍滞后 → 升 PSC_VELXY_FF(GUIDED 0.3~0.5)\n  · AUTO 航点附近来回修正 → 降 PSC_POSXY_P(1.2 以上在 AUTO 容易过冲)、升 WPNAV_RADIUS\n  · 任务时间太长       → WPNAV_SPEED + WPNAV_ACCEL 一起提；接近目标区再用\n                         DO_CHANGE_SPEED 命令降速\n\n四、与文档不同的地方(以 ArduPilot 官方参数表/源码为准，已按官方实现)\n  1) WPNAV_ACCEL 固件声明范围只有 50~500 cm/s²，文档写 800~1500 是超范围的；\n     本工具按 500 封顶。真要更猛，靠 WPNAV_SPEED + 倾角(ANGLE_MAX)而不是它。\n  2) WPNAV_JERK 范围 1~20 m/s³(默认 1)，文档\"10~30\"的上限无效。\n  3) MOT_THST_EXPO: 桨越大应越高(5寸0.55/10寸0.65/20寸0.75)；\n     \"大桨降到0.2~0.3\"只适用于电调已内置线性化(勾选该开关才取 0.10)。\n  4) PSC_ACCZ_P/I 在 4.7 起改名 PSC_D_ACC_P/I 且数值缩小 10 倍；\n     照抄\"P=悬停油门\"会大 10 倍，本工具按固件实际参数名自动换算。\n     同理 WP_SPD / WP_ACC / WP_RADIUS_M / PSC_NE_* / PSC_D_VEL_FF 全部自动探测。\n  5) EKF 噪声与\"外部目标坐标精度\"无关: 外部系统给的是目标点，不是飞机自身位置。\n     飞机自己不是 RTK/双天线就别动 EK3_* (第 7 阶段默认不勾选)。\n  6) FENCE_RADIUS 固件范围 30~10000m(文档写 20m 会被夹到 30m)；\n     BATT_LOW_VOLT 固件默认 10.5V，6S 机器永远不触发，本工具按 3.5V×串数 写入。\n  7) 《AUTO模式专用速查表》按官方参数表核对后的 4 处:\n     · WPNAV_SPEED 写的 2000~2500 → 固件上限 2000 cm/s，本工具按 2000 封顶；\n     · WPNAV_ACCEL 写的 800~1500 → 固件上限 500 cm/s²，按 500 封顶；\n     · WPNAV_ACCEL_Z 写的 500~800 → 固件上限 500 cm/s²，按 500 封顶；\n     · WPNAV_JERK 写的 15~30 → 范围 1~20，上限按 20。\n     另: 文档的 EK3_POS_M_NSE 实际参数名是 EK3_POSNE_M_NSE(本工具用真名)。\n  8) FS_GCS_TIMEOUT 范围 2~120s(固件默认 5，文档 3~10)；DISARM_DELAY 范围 0~127s\n     (默认 10)；FS_EKF_ACTION 在 4.6 只有 1/2/3，4.7 起才多一个 0=Report only；\n     WP_YAW_BEHAVIOR 多旋翼固件默认是 2(指下个航点、RTL 除外)而不是 1。\n\n五、地面站侧(GCS)检查清单 —— 参数调对了也追不上，多半是这里的问题\n  · 坐标系固定用 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT；绝对高程要先减起飞点高程。\n  · 消息 SET_POSITION_TARGET_GLOBAL_INT: lat_int/lon_int = 度×1e7，\n    alt = 相对高度，vx/vy/vz 单位是 m/s。\n  · 目标静止: type_mask = 0b0000101111111000 (只用位置)\n    目标移动: type_mask = 0b0000100111111000 (位置+速度) —— 只发位置必然滞后。\n  · 发送频率 ≥5Hz，推荐 10~20Hz；低于 2~3Hz 会一顿一顿。\n  · 起飞阶段先用 MAV_CMD_NAV_TAKEOFF(param7=3m)，等它确认后再连发目标。\n  · GUID_OPTIONS 保持默认 0：bit6 会让位置目标走航点平滑，追移动目标更滞后。\n  · 数传带宽不足时先降下行遥测，别让上行目标指令被堵。\n  · 紧急退出: 随时切 LOITER/ALT_HOLD/LAND，并保证 FS_GCS_ENABLE=1(第 8 阶段)。\n\n六、安全约束(写入前自动校验)与验证顺序\n  · MOT_SPIN_MIN ≥ MOT_SPIN_ARM + 0.03\n  · FENCE_ALT_MAX ≥ RTL_ALT；BATT_CRT_VOLT ≤ BATT_LOW_VOLT；数值落在固件范围\n  · 高速 + 小围栏半径会给提示(目标点可能\"到不了\")\n  · 写入前自动备份到 ai_tune_backups/，出问题到「备份管理」一键回滚\n  · 拆桨验证顺序: 写参数 → 拆桨 → 「拆桨实测起转」定 MOT_SPIN_ARM → 装桨 →\n    STABILIZE 低油门看姿态 → GUIDED 2~3m → RTL/降落 → 悬停 30s 学悬停油门 → 上锁\n\n七、症状对照\n  离地太猛/窜天      → 降 WPNAV_SPEED_UP、核对 MOT_THST_HOVER 是否偏大\n  离地犹豫/反复贴地  → 升 WPNAV_SPEED_UP、核对 MOT_THST_HOVER 是否偏小\n  到顶冲高回荡        → 降 WPNAV_ACCEL_Z；高度抖动 → 查振动、降 PILOT_ACCEL_Z\n  追不上移动目标      → 升 WPNAV_ACCEL / WPNAV_JERK / PSC_POSXY_P\n  任务跑不满速度      → 同时升 WPNAV_ACCEL + WPNAV_JERK(只升 SPEED 没用)\n  AUTO 航点附近来回修正 → 降 PSC_POSXY_P、升 WPNAV_RADIUS\n  水平来回摆/抖       → 降 WPNAV_ACCEL / PSC_POSXY_P / PSC_VELXY_P\n  水平顶风偏移        → 升 PSC_VELXY_I\n  到达目标点附近晃    → 升 WPNAV_RADIUS\n  机头指向不对        → WP_YAW_BEHAVIOR(0/1/2/3)，或任务里用 ROI / 固定航向\n  任务跑完不降落      → RTL_ALT_FINAL=0(要悬停接手就给正数，且 ≤ RTL_ALT)\n  RTL 后又越界        → FENCE_ALT_MAX 必须 ≥ RTL_ALT\n\n八、工具替你做不了的\n  · 罗盘/六面加速度计需要人工摆位(六面有向导弹你)\n  · 首飞前: GPS≥10 星、EKF 就绪、遥控失控保护已设、桨叶方向正确、场地空旷\n  · 推重比特别大的火箭机: MOT_THST_HOVER 千万别估高、WPNAV_SPEED_UP 先压低\n  · 追踪作业半径较大时，把 FENCE_RADIUS 双击改到 ≥ 作业半径\n  · 上传航线、编辑航点、设置 ROI 属于任务规划，本工具只负责把参数调对\n\n九、AUTO 任务模式专属(用途=AUTO 任务航点 时多出来的 15 项)\n  第 4 阶段 +4\n  · WP_YAW_BEHAVIOR=2  任务中的机头航向: 0=不改 1=指下个航点(默认 HELI)\n                        2=指下个航点但 RTL 除外(多旋翼默认) 3=沿 GPS 航迹方向。\n                        光电/雷达要持续对准任务区 → 提前规划，别等到航线跑起来再改。\n  · TUNE=10 / TUNE_MIN=500 / TUNE_MAX=2000   机上旋钮实时调 WPNAV_SPEED(默认不勾选)。\n                        前提: 某个遥控通道 RCx_OPTION=219(发射机调参)，通道号自选。\n                        TUNE_MIN/MAX 单位 = 被调参数单位，TUNE=10 时是 cm/s。\n  第 7 阶段 +7(全部只展示不勾选，纯体检)\n  · AHRS_EKF_TYPE=3 / EK3_ENABLE=1\n  · EK3_SRC1_POSXY=3(GPS) / VELXY=3(GPS) / POSZ=1(气压) / VELZ=3(GPS) / YAW=1(罗盘)\n    —— 这是\"飞机自己用什么定位\"，与外部系统给的目标坐标无关。AUTO 出现航点偏移、\n       高度跳动、转弯不顺，先看这里和振动，不要只加 P。\n  第 8 阶段 +4\n  · RTL_ALT_FINAL=0    任务跑完/返航到家后的最终高度: 0=自动降落(推荐)，正数=悬停等指令\n  · FS_GCS_TIMEOUT=5   地面站失联几秒触发保护(范围 2~120，默认 5) —— 不勾选\n  · FS_EKF_ACTION=1    EKF 异常动作 1=Land 2=AltHold 3=任何模式都 Land(4.7 起 0=仅记录)\n  · DISARM_DELAY=10    落地后几秒自动上锁(0=关闭) —— 不勾选\n  任务命令(不是参数，要在任务规划器里加)\n  · SPLINE 航点   平滑转弯，减少折线；直线航点更适合测绘\n  · DO_CHANGE_SPEED 航段间动态改速: 转场快、进目标区慢，持续生效直到被覆盖\n  · LOITER_TIME   到点后悬停 N 秒(拍照/对准/等外部系统确认)\n\n十、GUIDED vs AUTO 怎么选\n  对比项      GUIDED                          AUTO\n  目标来源    外部实时 MAVLink 指令           预先上传的航点任务\n  速度控制    MAVLink 指令 + WPNAV 参数        主要由 WPNAV_SPEED / DO_CHANGE_SPEED 决定\n  路径形态    持续追目标点                     按航点顺序执行，可直线或 SPLINE\n  调参重点    位置环、速度前馈、更新频率        航点速度、加速度、航向行为、任务安全\n  适用场景    移动目标跟踪、外部闭环控制        固定航线、转场、测绘、定点作业\n  实时性      高(10~20Hz 连发)                 低(改航点要重传任务)\n  需要外部系统持续修正目标位置时选 GUIDED；需要\"按任务路径执行 + 阶段性修正\"时选 AUTO。\n  两者都要紧急退出: 随时切 LOITER/RTL/LAND，并保证 FS_GCS_ENABLE=1。\n",
 "purposes": {
  "first": "首飞(安全保守)",
  "guided": "GUIDED 外部引导追踪",
  "auto": "AUTO 任务航点"
 },
 "stageOrder": [
  [
   "battery",
   "1 电池与推力曲线"
  ],
  [
   "spin",
   "2 电机怠速与输出"
  ],
  [
   "takeoff",
   "3 起飞"
  ],
  [
   "track",
   "4 水平导航 WPNAV (GUIDED/AUTO)"
  ],
  [
   "vert",
   "5 垂直与高度"
  ],
  [
   "filter",
   "6 滤波与姿态"
  ],
  [
   "ekf",
   "7 定位与 EKF (可选)"
  ],
  [
   "safety",
   "8 安全与保护"
  ]
 ],
 "sizes": {
  "5": {
   "gyro": 80,
   "expo": 0.55,
   "acc": 1100,
   "accy": 300,
   "slew": 1.0
  },
  "7": {
   "gyro": 60,
   "expo": 0.6,
   "acc": 1100,
   "accy": 250,
   "slew": 1.0
  },
  "10": {
   "gyro": 40,
   "expo": 0.65,
   "acc": 1100,
   "accy": 200,
   "slew": 1.5
  },
  "13": {
   "gyro": 30,
   "expo": 0.68,
   "acc": 900,
   "accy": 150,
   "slew": 2.0
  },
  "15": {
   "gyro": 25,
   "expo": 0.7,
   "acc": 700,
   "accy": 120,
   "slew": 2.0
  },
  "20": {
   "gyro": 20,
   "expo": 0.75,
   "acc": 500,
   "accy": 100,
   "slew": 2.0
  },
  "30": {
   "gyro": 20,
   "expo": 0.8,
   "acc": 200,
   "accy": 90,
   "slew": 3.0
  }
 },
 "profiles": {
  "soft": {
   "name": "更保守",
   "speed_up": 80,
   "accel_z": 120,
   "pilot_speed": 80,
   "pilot_accel": 150,
   "slew_adj": 0.5,
   "jerk": 6
  },
  "std": {
   "name": "标准",
   "speed_up": 100,
   "accel_z": 150,
   "pilot_speed": 100,
   "pilot_accel": 200,
   "slew_adj": 0.0,
   "jerk": 8
  },
  "hot": {
   "name": "更激进",
   "speed_up": 150,
   "accel_z": 200,
   "pilot_speed": 150,
   "pilot_accel": 250,
   "slew_adj": -0.3,
   "jerk": 12
  }
 },
 "trackProfiles": {
  "soft": {
   "speed": 1200,
   "accel": 300,
   "jerk": 8,
   "radius": 300,
   "pos": 1.0,
   "velp": 2.0,
   "veli": 0.7,
   "velff": 0.2,
   "jxy": 8,
   "speed_dn": 150,
   "vzff": 0.2
  },
  "std": {
   "speed": 1500,
   "accel": 400,
   "jerk": 12,
   "radius": 200,
   "pos": 1.2,
   "velp": 2.2,
   "veli": 1.0,
   "velff": 0.3,
   "jxy": 12,
   "speed_dn": 200,
   "vzff": 0.3
  },
  "hot": {
   "speed": 2000,
   "accel": 500,
   "jerk": 20,
   "radius": 100,
   "pos": 1.5,
   "velp": 2.5,
   "veli": 1.0,
   "velff": 0.5,
   "jxy": 20,
   "speed_dn": 300,
   "vzff": 0.5
  }
 },
 "trackFirst": {
  "speed": 800,
  "accel": 150,
  "jerk": 5,
  "radius": 300,
  "pos": 1.0,
  "velp": 1.5,
  "veli": 0.7,
  "velff": 0.1,
  "jxy": 5,
  "speed_dn": 150,
  "vzff": 0.1
 },
 "autoProfiles": {
  "soft": {
   "speed": 1200,
   "accel": 300,
   "jerk": 12,
   "radius": 300,
   "pos": 1.2,
   "velp": 1.5,
   "veli": 0.7,
   "velff": 0.1,
   "jxy": 8,
   "speed_up": 300,
   "speed_dn": 200,
   "accel_z": 300,
   "vzff": 0.1,
   "jerk_z": 10
  },
  "std": {
   "speed": 1500,
   "accel": 400,
   "jerk": 15,
   "radius": 200,
   "pos": 1.4,
   "velp": 2.0,
   "veli": 1.0,
   "velff": 0.1,
   "jxy": 12,
   "speed_up": 400,
   "speed_dn": 300,
   "accel_z": 400,
   "vzff": 0.2,
   "jerk_z": 14
  },
  "hot": {
   "speed": 2000,
   "accel": 500,
   "jerk": 20,
   "radius": 100,
   "pos": 1.8,
   "velp": 2.2,
   "veli": 1.0,
   "velff": 0.2,
   "jxy": 20,
   "speed_up": 500,
   "speed_dn": 400,
   "accel_z": 500,
   "vzff": 0.3,
   "jerk_z": 20
  }
 }
};
