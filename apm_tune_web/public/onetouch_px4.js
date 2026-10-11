// PX4 一键首飞: 数据与方案派生 (由 onetouch.js 的固件派发调用)
// 参数名/取值均按 PX4 v1.14 参数手册核对; 结构参照 onetouch_data.js
// window.PX4OneTouch.otStages(ctx) / otValidate(rows) 与 OneTouchCore 同签名
(function () {
  const PX4_OT_DATA = {
    help: `一键首飞 · 帮助说明 (PX4)
========================================================
输入 桨径 / 电池串数 / 悬停油门 / 档位 / 用途 → 生成 PX4 参数方案 →
体检读取 → 交叉校验 → 自动备份 → 一键写入 → 报告。
共 7 个阶段, 每行都有"依据"列, 可勾选、可双击改值。
参数按 PX4 v1.14 命名(向下兼容 v1.12+), 与 ArduPilot 方案互不通用,
切换固件后本页会自动重建(见左上角"固件"徽章)。

一、这个页解决什么
  用途三选一(左上角):
  · 首飞(安全保守)          —— 只写安全/起降/围栏类参数, 水平导航(MPC_XY_*)
                                与 EKF2 只展示不勾选, 避免一次改太多。
  · Offboard 外部引导追踪    —— 地面站 10~20Hz 发目标位姿追移动目标,
                                速度/加速度/Jerk 按追踪放开并勾选。
  · AUTO 任务航点            —— 预先上传航线按航点执行, 取值走任务区间,
                                并启用任务垂直速度 MPC_Z_V_AUTO_*。

  阶段说明:
  1 电池与保护  BAT1_CAPACITY / 低电三阈值 / COM_LOW_BAT_ACT
  2 起飞与降落  MPC_TKO_* / MPC_LAND_SPEED / MIS_TAKEOFF_ALT
  3 水平导航    MPC_XY_CRUISE/VEL_MAX/ACC_HOR/JERK_MAX + NAV_ACC_RAD
  4 垂直与高度  MPC_Z_VEL_MAX_*/MPC_ACC_*MAX (AUTO 另有 MPC_Z_V_AUTO_*)
  5 滤波与速率环 IMU_* 滤波 + MC_*_P/I/D (全部只体检不勾选)
  6 定位与 EKF2 EKF2_* 噪声参数 (体检确认, 默认即最佳起点)
  7 安全与保护  RC/数据链失联、落地自锁、低电、地理围栏、返航

二、推荐顺序
  0) 校准: 在 QGroundControl 完成 加速度计(六面)/罗盘/气压/遥控器 校准
     (本工具对 PX4 禁用六面向导与拆桨实测, 请用 QGC 的 Motor 测试)。
  1) 用途=首飞, 填输入 → ① 生成方案 → ② 读取体检 → ③ 一键写入。
  2) 重启飞控 → 装桨 → Position Hold(Hold 模式) 看姿态方向是否正确。
  3) 低空悬停 30s 落地上锁, 看"日志管理"页解析 .ulg: 振动/EKF 新息/输出。
  4) 悬停正常后再飞高/飞远; 稳定后把"用途"切到 Offboard 或 AUTO 重新生成。

三、校准与安全
  · 拆桨实测 / 六面加速度计 / 水平校准按钮对 PX4 禁用 → 用 QGC 完成。
  · 围栏: GF_MAX_HOR_DIST=50m(水平) / GF_MAX_VER_DIST=50m(垂直),
    动作 GF_ACTION=3(返航); 0=禁用, 4=终止(切电, 慎用)。
  · RC 失联 NAV_RCL_ACT=2(返航); 数据链 NAV_DLL_ACT=0(禁用)起步。
  · 低电 COM_LOW_BAT_ACT=3: 低电返航, 紧急电平原地降落。
  · 返航高度 RTL_RETURN_ALT 必须 ≤ 垂直围栏, 否则返航爬升会再次越界。

四、调参与验证
  · 速率环 v1.14 命名 MC_ROLLRATE_P/I/D (旧 MC_ROLLRATE_PR/PRR 已废弃),
    角度环 MC_ROLL_P/PITCH_P/YAW_P; 首飞不改它们(第 5 阶段只体检)。
  · 振动大 → 降 IMU_GYRO_CUTOFF(40Hz 默认, 可 20~30); 大桨再配动性陷波。
  · 跑不满速度 → 升 MPC_ACC_HOR 再升 MPC_XY_CRUISE; 来回摆 → 降这两项。
  · EKF2_* 噪声是滤波权重: 定位源差(普通 GPS)时调小会漂移, 保持默认。
  · 写入后务必重启飞控使参数生效(部分参数需 Reboot, 已在依据列注明)。

五、安全提示
  首飞必须拆桨确认转向 → 低空系留/空旷场地 → 随时准备切 Hold/RTL。
  参数异常时用「备份恢复」页回滚; 本工具写入前会自动全量备份。`,

    purposes: {
      first: '首飞(安全保守)',
      guided: 'Offboard 外部引导追踪',
      auto: 'AUTO 任务航点',
    },
    // 阶段 ID 复用 ArduPilot 的 stageOrder (少一个 spin: PX4 无 SPIN 参数)
    stageOrder: [
      ['battery', '1 电池与保护'],
      ['takeoff', '2 起飞与降落'],
      ['track', '3 水平导航 MPC'],
      ['vert', '4 垂直与高度'],
      ['filter', '5 滤波与速率环'],
      ['ekf', '6 定位与 EKF2 (可选)'],
      ['safety', '7 安全与保护'],
    ],
  };
  window.PX4_OT_DATA = PX4_OT_DATA;

  // ---- 工具 ----
  const r = (x, n) => {
    const f = Math.pow(10, n);
    return Math.round((Number(x) + Number.EPSILON) * f) / f;
  };
  const mk = (name, value, unit, rng, desc, help, check) => ({
    name, value, unit, rng, desc,
    help: help || desc,
    aliases: [],
    check: check === undefined ? true : check,
  });

  // 各用途基准值 (再按档位 soft/std/hot × 0.7/1.0/1.3 缩放速度与加速度)
  const T = {
    first: { cruise: 5, xyvel: 12, acc: 3, accmax: 5, jerk: 8, jerkAuto: 4, up: 3, dn: 1.5, accUp: 4, accDn: 3, navrad: 10 },
    guided: { cruise: 8, xyvel: 15, acc: 6, accmax: 8, jerk: 12, jerkAuto: 8, up: 4, dn: 2, accUp: 6, accDn: 4, navrad: 10 },
    auto: { cruise: 6, xyvel: 15, acc: 5, accmax: 8, jerk: 10, jerkAuto: 6, up: 4, dn: 2, accUp: 5, accDn: 4, navrad: 8 },
  };
  const PROF_MUL = { soft: 0.7, std: 1, hot: 1.3 };

  function otStages(ctx) {
    const D = window.PX4_OT_DATA;
    const purpose = Object.prototype.hasOwnProperty.call(D.purposes, ctx.purpose) ? ctx.purpose : 'first';
    const mul = PROF_MUL[ctx.profile] || 1;
    const t = T[purpose] || T.first;
    const purposeCn = D.purposes[purpose];
    const size = parseInt(ctx.size, 10) || 5;
    const spd = (v) => r(v * mul, 1);
    const st = {};

    st.battery = [
      mk('BAT1_CAPACITY', -1, 'mAh', [-1, 100000],
        '电池容量(-1=自动识别)',
        '-1=按电压自动判定电量(首飞推荐)。\n填实测容量(mAh)可让电量百分比更准。改后需重启。'),
      mk('BAT_LOW_THR', 0.15, '', [0.12, 0.5],
        '低电阈值(0~1 比例, 0.15=默认)',
        '低于 15% 报"电量低"并按 COM_LOW_BAT_ACT 执行。\n⚠ 单位是 0~1 比例不是百分数。'),
      mk('BAT_CRIT_THR', 0.07, '', [0.05, 0.25],
        '严重低电阈值(0.07=默认)',
        '低于 7% 视为严重低电(通常触发返航)。必须 < BAT_LOW_THR。'),
      mk('BAT_EMERGEN_THR', 0.05, '', [0.03, 0.1],
        '紧急低电阈值(0.05=默认)',
        '低于 5% 视为紧急(原地降落)。必须 < BAT_CRIT_THR。'),
      mk('COM_LOW_BAT_ACT', 3, '', [0, 3],
        '低电动作: 0=警告 2=降落 3=返航+紧急降落',
        '推荐 3: 低于 BAT_CRIT_THR 返航, 低于 BAT_EMERGEN_THR 原地降落。\n0=仅警告(保护等于没开)。'),
    ];

    st.takeoff = [
      mk('MPC_TKO_SPEED', 1.5, 'm/s', [1, 5],
        '起飞爬升速度(1.5=默认)',
        '离地阶段的爬升率上限。首飞保持 1.5; 弹起飞再加大到 2~2.5。'),
      mk('MPC_TKO_RAMP_T', 3, 's', [0, 5],
        '起飞油门斜坡时间常数(3=默认,0=关闭)',
        '越大起飞越柔; 太大可能"蹭地"。默认 3s。'),
      mk('MPC_LAND_SPEED', 0.7, 'm/s', [0.6, 5],
        '降落下降速度(0.7=默认)',
        '最后阶段降落率。草地/不平地面可降到 0.6。'),
      mk('MIS_TAKEOFF_ALT', 2.5, 'm', [0, 80],
        '任务起飞高度(2.5=默认)',
        'AUTO 任务起飞段爬升到该高度再飞航点。'),
    ];

    st.track = [
      mk('MPC_XY_CRUISE', spd(t.cruise), 'm/s', [3, 20],
        '任务/RTL 水平巡航速度',
        `用途「${purposeCn}」→ ${spd(t.cruise)} m/s (固件默认 5)。\n只升它不升 MPC_ACC_HOR 会"跑不满速度"。`),
      mk('MPC_XY_VEL_MAX', spd(t.xyvel), 'm/s', [0, 20],
        'AUTO 模式水平速度上限',
        `用途「${purposeCn}」→ ${spd(t.xyvel)} m/s (固件默认 12)。\n任务中命令更快的速度会被它封顶。`),
      mk('MPC_ACC_HOR', spd(t.acc), 'm/s²', [2, 15],
        '水平加速度(手动/自动)',
        `用途「${purposeCn}」→ ${spd(t.acc)} m/s² (固件默认 3)。\n追不上→升它; 来回摆→降它 (通常 2~6)。`),
      mk('MPC_ACC_HOR_MAX', spd(t.accmax), 'm/s²', [2, 15],
        '最大水平加速度',
        `用途「${purposeCn}」→ ${spd(t.accmax)} m/s² (固件默认 5)。\n上限保护, 一般 ≥ MPC_ACC_HOR。`),
      mk('MPC_JERK_MAX', spd(t.jerk), 'm/s³', [0.5, 500],
        '全局限制加加速度(越大越跟手)',
        `用途「${purposeCn}」→ ${spd(t.jerk)} m/s³ (固件默认 8)。\n越大起步/转向越干脆, 过大机身猛抖。`),
      mk('MPC_JERK_AUTO', spd(t.jerkAuto), 'm/s³', [1, 80],
        'AUTO 模式加加速度限制',
        `用途「${purposeCn}」→ ${spd(t.jerkAuto)} m/s³ (固件默认 4)。`),
      mk('NAV_ACC_RAD', t.navrad, 'm', [0.05, 200],
        '航点到达判定半径(10=默认)',
        '任务航点视为"到达"的水平半径。测绘/精准过点逐次收小(≥0.5)。\n太小会在航点附近反复修正, 看起来像"到不了"。'),
      mk('MIS_DIST_1WP', 900, 'm', [-1, 10000],
        '首航点最大距离(≤0=禁用)',
        '当前位置到首航点超过该距离则拒绝启动任务(防旧任务异地误飞)。\n仅 AUTO 任务相关。',
        purpose === 'auto'),
    ];
    // 首飞: 水平导航参数只展示不勾选 (与 ArduPilot 行为一致)
    for (const e of st.track) e.check = (purpose === 'guided' || purpose === 'auto');

    st.vert = [
      mk('MPC_Z_VEL_MAX_UP', spd(t.up), 'm/s', [0.5, 8],
        '手动/Offboard 最大上升速度',
        `用途「${purposeCn}」→ ${spd(t.up)} m/s (固件默认 3)。\n首飞最敏感: 离地太猛降、反复贴地升。`),
      mk('MPC_Z_VEL_MAX_DN', spd(t.dn), 'm/s', [0.5, 4],
        '手动/Offboard 最大下降速度',
        `用途「${purposeCn}」→ ${spd(t.dn)} m/s (固件默认 1.5)。\n掉高太快/降不下来先升它。`),
      mk('MPC_ACC_UP_MAX', spd(t.accUp), 'm/s²', [2, 15],
        '最大上升加速度',
        `用途「${purposeCn}」→ ${spd(t.accUp)} m/s² (固件默认 4)。`),
      mk('MPC_ACC_DOWN_MAX', spd(t.accDn), 'm/s²', [2, 15],
        '最大下降加速度',
        `用途「${purposeCn}」→ ${spd(t.accDn)} m/s² (固件默认 3)。`),
      mk('MPC_Z_V_AUTO_UP', spd(t.up), 'm/s', [0.5, 8],
        'AUTO/任务 最大上升速度',
        `AUTO 任务单独的上升率限制 → ${spd(t.up)} m/s (默认 3)。`,
        purpose === 'auto'),
      mk('MPC_Z_V_AUTO_DN', spd(t.dn), 'm/s', [0.5, 4],
        'AUTO/任务 最大下降速度',
        `AUTO 任务单独的下降率限制 → ${spd(t.dn)} m/s (默认 1.5)。`,
        purpose === 'auto'),
      mk('MPC_THR_HOVER', 0.5, '', [0.1, 0.8],
        '悬停油门(0~1, 体检确认)',
        '悬停所需油门, 手动油门中点映射用它。\n默认 0.5; MPC_USE_HTE=1 时起飞后由估计器自动学习, 无需手调。',
        false),
    ];

    // 滤波与速率环: 全部只体检不勾选 —— PX4 开箱即飞, 首飞不动 PID
    const gyroCutoff = size >= 20 ? 20 : size >= 13 ? 30 : 40;
    st.filter = [
      mk('IMU_GYRO_CUTOFF', gyroCutoff, 'Hz', [0, 1000],
        '陀螺低通滤波(v1.14 默认 40, 0=关闭)',
        `${size}寸 → 建议 ${gyroCutoff} Hz。振动大先降它; 大桨升到 80 会把振动引进 PID。\n体检确认: 与建议差异大时先看日志振动再决定。`,
        false),
      mk('IMU_ACCEL_CUTOFF', 30, 'Hz', [0, 1000],
        '加计低通滤波(默认 30, 0=关闭)',
        '体检确认。高度抖动可略降(20~30)。',
        false),
      mk('IMU_GYRO_RATEMAX', 400, 'Hz', [100, 2000],
        '速率环数据频率(默认 400)',
        '可选 100/250/400/800/1000/2000。升高会增加 CPU 负载, 首飞保持 400。改后需重启。',
        false),
      mk('MC_ROLL_P', 6.5, '', [0, 12],
        '横滚角度环 P(6.5=默认)',
        '体检确认。外场调参: 超调降 0.5~1, 跟手慢升 0.5。', false),
      mk('MC_PITCH_P', 6.5, '', [0, 12],
        '俯仰角度环 P(6.5=默认)',
        '体检确认。同横滚。', false),
      mk('MC_YAW_P', 2.8, '', [0, 5],
        '偏航角度环 P(2.8=默认)',
        '体检确认。偏航抖先降。', false),
      mk('MC_ROLLRATE_P', 0.15, '', [0.01, 0.5],
        '横滚速率环 P(0.15=默认)',
        '体检确认。高频振荡降 P/D, 低频晃升 P。', false),
      mk('MC_ROLLRATE_I', 0.2, '', [0, 5],
        '横滚速率环 I(0.2=默认)',
        '体检确认。抗常值干扰(顶风漂/重心偏)。', false),
      mk('MC_ROLLRATE_D', 0.003, '', [0, 0.01],
        '横滚速率环 D(0.003=默认)',
        '体检确认。过大会抖/发热。', false),
      mk('MC_PITCHRATE_P', 0.15, '', [0.01, 0.6],
        '俯仰速率环 P(0.15=默认)',
        '体检确认。', false),
      mk('MC_PITCHRATE_I', 0.2, '', [0, 5],
        '俯仰速率环 I(0.2=默认)',
        '体检确认。', false),
      mk('MC_PITCHRATE_D', 0.003, '', [0, 0.01],
        '俯仰速率环 D(0.003=默认)',
        '体检确认。', false),
      mk('MC_YAWRATE_P', 0.2, '', [0, 0.6],
        '偏航速率环 P(0.2=默认)',
        '体检确认。', false),
      mk('MC_YAWRATE_I', 0.1, '', [0, 5],
        '偏航速率环 I(0.1=默认)',
        '体检确认。', false),
      mk('MC_YAWRATE_D', 0, '', [0, 5],
        '偏航速率环 D(0=默认)',
        '体检确认。默认不开偏航 D。', false),
    ];

    st.ekf = [
      mk('EKF2_GYR_NOISE', 0.015, 'rad/s', [0.0001, 0.1],
        '陀螺噪声(0.015=默认)',
        '体检确认。普通机保持默认; 调小对噪声更敏感, 调大发散。',
        false),
      mk('EKF2_ACC_NOISE', 0.35, 'm/s²', [0.01, 1],
        '加计噪声(0.35=默认)',
        '体检确认。振动大时反而应调大, 不要盲目调小。', false),
      mk('EKF2_MAG_NOISE', 0.05, 'Gauss', [0.001, 1],
        '磁力计噪声(0.05=默认)',
        '体检确认。罗盘受扰(电机线近)先排除干扰源。', false),
      mk('EKF2_GPS_P_NOISE', 0.5, 'm', [0.01, 10],
        'GPS 位置噪声(0.5=默认)',
        '体检确认。普通 GPS 保持 0.5; 只有 RTK(<0.1m) 才收紧, 每次减 10~20%。',
        false),
      mk('EKF2_GPS_V_NOISE', 0.3, 'm/s', [0.01, 5],
        'GPS 速度噪声(0.3=默认)',
        '体检确认。', false),
      mk('EKF2_BARO_NOISE', 3.5, 'm', [0.01, 15],
        '气压高度噪声(3.5=默认)',
        '体检确认。风扰/罩子气流大时调大, 不要调小。', false),
    ];

    st.safety = [
      mk('COM_RC_LOSS_T', 0.5, 's', [0, 35],
        'RC 丢失判定时间(0.5=默认)',
        '连续无 RC 数据超过该时长视为丢失。太短易误触发(接收机刷新率低时加大)。'),
      mk('NAV_RCL_ACT', 2, '', [1, 6],
        'RC 失联动作: 1=悬停 2=返航 3=降落 5=终止 6=上锁',
        '推荐 2=返航(固件默认)。场地很小/室内可改 1=悬停。\n5=终止会直接切电, 慎用。'),
      mk('NAV_DLL_ACT', 0, '', [0, 6],
        '数据链丢失动作: 0=禁用 1=悬停 2=返航 3=降落 5=终止 6=上锁',
        '首飞保持 0(禁用) —— 没有数传时避免误触发。有数传再设 1/2。\n超时时间 COM_DL_LOSS_T(默认 10s)。', false),
      mk('COM_DISARM_LAND', 2, 's', [0, 120],
        '落地自动上锁延时(2=默认,0=关闭)',
        '落地检测后 2s 自动上锁, 螺旋桨停转。0=关闭(手动上锁)。'),
      mk('GF_ACTION', 3, '', [0, 5],
        '围栏动作: 0=无 1=警告 2=悬停 3=返航 4=终止 5=降落',
        '推荐 3=返航(固件默认 2=悬停)。4=终止会切电, 只在竞速/室内慎用。'),
      mk('GF_MAX_HOR_DIST', 50, 'm', [0, 10000],
        '水平围栏最大距离(0=禁用, 首飞 50m)',
        '离 Home 超过该距离触发 GF_ACTION。按作业半径放大, 但要留返航余量。'),
      mk('GF_MAX_VER_DIST', 50, 'm', [0, 10000],
        '垂直围栏最大距离(0=禁用, 首飞 50m)',
        '离 Home 超过该高度触发 GF_ACTION。\n⚠ 必须 ≥ RTL_RETURN_ALT, 否则返航爬升会再次越界形成循环。'),
      mk('RTL_RETURN_ALT', 30, 'm', [0, 8000],
        '返航爬升高度(60=默认)',
        '★ 必须高于航线中最高障碍物; 同时必须 ≤ GF_MAX_VER_DIST。\n场地空旷首飞 30m 足够。'),
    ];

    return D.stageOrder.map(([sid, label]) => ({ id: sid, label, items: st[sid] || [] }));
  }

  // ---- 交叉校验 (PX4 专属) ----
  function otValidate(rows) {
    const issues = [];
    const values = {};
    for (const row of rows) values[row.entry.name] = row.value;
    // 通用: 固件范围
    for (const row of rows) {
      const rng = row.entry.rng;
      const v = row.value;
      if (rng && !(v >= rng[0] && v <= rng[1])) {
        issues.push(`${row.entry.name}=${v} 超出固件范围 ${rng[0]}~${rng[1]}`);
      }
    }
    const lo = values.BAT_LOW_THR; const cr = values.BAT_CRIT_THR; const em = values.BAT_EMERGEN_THR;
    if (cr !== undefined && lo !== undefined && cr >= lo - 1e-9) {
      issues.push(`BAT_CRIT_THR(${cr}) 必须 < BAT_LOW_THR(${lo})`);
    }
    if (em !== undefined && cr !== undefined && em >= cr - 1e-9) {
      issues.push(`BAT_EMERGEN_THR(${em}) 必须 < BAT_CRIT_THR(${cr})`);
    }
    const up = values.MPC_Z_VEL_MAX_UP; const dn = values.MPC_Z_VEL_MAX_DN;
    if (up !== undefined && dn !== undefined && dn > up + 1e-9) {
      issues.push(`MPC_Z_VEL_MAX_DN(${dn}) 大于 MPC_Z_VEL_MAX_UP(${up}), 下降比爬升快很多`);
    }
    const rtl = values.RTL_RETURN_ALT; const fenceV = values.GF_MAX_VER_DIST;
    if (rtl !== undefined && fenceV !== undefined && fenceV > 0 && fenceV < rtl - 1e-9) {
      issues.push(`GF_MAX_VER_DIST(${fenceV}m) 必须 ≥ RTL_RETURN_ALT(${rtl}m), 否则返航爬升会再次越界`);
    }
    const crui = values.MPC_XY_CRUISE; const acc = values.MPC_ACC_HOR;
    if (crui !== undefined && acc !== undefined && crui >= 10 && acc < 3) {
      issues.push(`MPC_XY_CRUISE(${crui}m/s) 较快而 MPC_ACC_HOR(${acc}m/s²) 偏小, 可能"跑不满速度"`);
    }
    return issues;
  }

  window.PX4OneTouch = { otStages, otValidate };
})();
