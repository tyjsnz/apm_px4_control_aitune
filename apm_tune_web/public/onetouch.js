// 一键首飞前端: 方案生成 / 体检读取 / 一键写入 / 校准 / 拆桨实测 / 帮助
// 方案派生逻辑移植自 docs/ai_pid_tune.py::_ot_stages (由 test/golden 比对保证一致)
(function () {
  const D = window.OT_DATA || {};
  const STAGE_ORDER = D.stageOrder || [];

  // ---------- 数值工具 (对齐 Python) ----------
  const r = (x, n) => {
    const f = Math.pow(10, n);
    return Math.round((Number(x) + Number.EPSILON) * f) / f;
  };
  // 模仿 Python %g (去掉多余尾零)
  const gz = (v) => {
    const n = Number(v);
    if (Number.isInteger(n)) return String(n);
    return String(parseFloat(n.toFixed(6)));
  };

  const PROF_NAME = { soft: '更保守', std: '标准', hot: '更激进' };

  function sizeProfile(size) {
    let s = Math.round(Number(size));
    if (!isFinite(s)) s = 5;
    const keys = Object.keys(D.sizes).map(Number);
    let best = keys[0];
    for (const k of keys) {
      if (Math.abs(k - s) < Math.abs(best - s) || (Math.abs(k - s) === Math.abs(best - s) && k < best)) {
        best = k;
      }
    }
    return D.sizes[String(best)];
  }

  function mk(name, value, unit, rng, desc, help, aliases, check) {
    const d = { name, value, unit, rng, desc, help, aliases: aliases || [], check: check === undefined ? true : check };
    return d;
  }

  // ---------- 核心: 8 阶段方案派生 ----------
  function otStages(ctx) {
    const size = parseInt(ctx.size, 10) || 5;
    const cells = parseInt(ctx.cells, 10) || 6;
    const hover = parseFloat(ctx.hover);
    const hoverV = isFinite(hover) ? hover : 0.25;
    const escLinear = !!ctx.escLinear;
    const purpose = Object.prototype.hasOwnProperty.call(D.purposes, ctx.purpose) ? ctx.purpose : 'first';
    const profKey = ctx.profile || 'std';
    const q = D.profiles[profKey] || D.profiles.std;
    let tk;
    if (purpose === 'auto') tk = D.autoProfiles[profKey] || D.autoProfiles.std;
    else if (purpose === 'guided') tk = D.trackProfiles[profKey] || D.trackProfiles.std;
    else tk = D.trackFirst;
    const auto = purpose === 'auto';
    const v_up = auto ? tk.speed_up : q.speed_up;
    const v_accz = auto ? tk.accel_z : q.accel_z;
    const v_jerk = auto ? tk.jerk_z : q.jerk;
    const p = sizeProfile(size);
    const expo = escLinear ? 0.10 : p.expo;
    const slew = Math.max(0.25, Math.min(5.0, p.slew + q.slew_adj));
    const gyro = Math.round(parseFloat(ctx.gyro || p.gyro));
    const gyroSrc = ctx.gyro ? '读取值' : '按尺寸推荐值';
    const half = Math.max(5, Math.min(100, Math.round(gyro / 2.0)));
    const purposeCn = D.purposes[purpose];
    const st = {};

    st.battery = [
      mk('MOT_BAT_VOLT_MAX', r(4.2 * cells, 2), 'V', [0, 60],
        '满电电压(电压补偿上限)',
        `官方: 满电电压 = 4.2V × 串数 = 4.2 × ${cells} = ${(4.2 * cells).toFixed(1)}V。\n决定油门/PID 电压补偿的上限，不设则不同电量下响应不一致。`),
      mk('MOT_BAT_VOLT_MIN', r(3.3 * cells, 2), 'V', [0, 60],
        '最低工作电压(电压补偿下限)',
        `官方: 最低电压 = 3.3V × 串数 = 3.3 × ${cells} = ${(3.3 * cells).toFixed(1)}V。\n与 BATT_CRT_VOLT 一致较合理；再往下飞控不再放大增益。`),
      mk('MOT_THST_EXPO', expo, '', [-1.0, 1.0],
        '推力曲线线性化指数',
        (escLinear
          ? '电调已内置线性化 → 0~0.2(取 0.10)。\n'
          : `官方按桨径: 5寸=0.55 / 10寸=0.65 / 20寸=0.75，当前 ${size}寸 → ${expo}。\n`)
        + '注: 文档"大桨低KV降到0.2~0.3"只适用于电调已线性化的情况；\n桨越大 EXPO 应当越高，用错会让低油门段发冲、刹车姿态不稳。'),
      mk('MOT_THST_HOVER', hoverV, '', [0.125, 0.6875],
        '悬停油门初始估计',
        '首飞保守取 0.20~0.25；推重比大的火箭机宁可低估。\n它同时是下面 PSC_ACCZ_P/I 的派生基准，写完起飞 30s 落地会自动校正。'),
      mk('MOT_HOVER_LEARN', 2, '', [0, 2],
        '悬停油门学习: 0=关 1=学 2=学并保存',
        '保持 2：悬停 ≥30s 落地上锁后写回真实悬停油门。'),
    ];

    st.spin = [
      mk('MOT_SPIN_ARM', 0.10, '', [0.0, 0.2],
        '解锁怠速(电机起转余量)',
        '先用「拆桨实测起转」按钮自动测：从 6% 逐档升油门，\n用 ESC 遥测 RPM>0 判定起转点，再 +1% 作为本值。\n没有 ESC 遥测就拆桨人工测。'),
      mk('MOT_SPIN_MIN', 0.15, '', [0.0, 0.25],
        '飞行最低油门下限',
        '必须 ≥ MOT_SPIN_ARM + 0.03(应用前会自动校验)。\n过低会让起飞段输出不线性、离地瞬间抖动。'),
      mk('MOT_SPIN_MAX', 0.95, '', [0.9, 1.0],
        '最大输出上限',
        '默认 0.95，顶部留 5% 余量；不要设 1.0 以免油门饱和。'),
    ];

    st.takeoff = [
      mk('TKOFF_THR_MAX', 1.0, '', [0.0, 1.0],
        '起飞阶段允许的最大油门',
        '保持 1.0，离地快慢由 WPNAV_SPEED_UP/WPNAV_ACCEL_Z 决定。'),
      mk('TKOFF_SLEW_TIME', slew, 's', [0.25, 5.0],
        '起飞油门爬升时间',
        `文档建议 0.5~1.0s(更柔 2~3s)，固件默认 2.0s。\n${gz(size)}寸 → ${gz(slew)}s；离地"弹一下"就加大它。`),
    ];

    st.track = [
      mk('WPNAV_SPEED', tk.speed, 'cm/s', [10, 2000],
        '水平导航最大速度',
        `用途「${purposeCn}」→ ${tk.speed} cm/s (固件默认 1000)。\n`
        + (auto
          ? 'AUTO: 航点间最大水平速度，常规 1000~1500、快速转场再往上；\n只提高它不提 WPNAV_ACCEL/JERK 会"跑不满速度"。任务中可用\nDO_CHANGE_SPEED 命令在航段间动态改速。\n'
          : 'GUIDED: 追移动目标时它是"追不追得上"的第一限制。\n')
        + '文档建议 1500~2500，⚠ 固件上限 2000，本工具按 2000 封顶。\n超调/摆动就降到 1200。4.7 起为 WP_SPD(m/s) 自动换算。',
        [['WP_SPD', 0.01]]),
      mk('WPNAV_ACCEL', tk.accel, 'cm/s/s', [50, 500],
        '水平导航加速度上限',
        `用途「${purposeCn}」→ ${tk.accel} cm/s² (固件默认 100~250)。\n⚠ 固件声明范围只有 50~500，两份文档里的 800~1500 已超出上限，\n本工具按 500 封顶；速度高时必须同步提高它，否则"跑不满速度"。\n追不上/跑不动先加它，来回摆就降它。4.7 起为 WP_ACC(m/s²)。`,
        [['WP_ACC', 0.01]]),
      mk('WPNAV_JERK', tk.jerk, 'm/s³', [1, 20],
        '水平加加速度(响应猛不猛)',
        `用途「${purposeCn}」→ ${tk.jerk} m/s³ (固件默认 1，范围 1~20)。\nGUIDED 文档写 10~30、AUTO 文档写 15~30，⚠ 上限只有 20；\n越大越"跟手"，超过 15 容易抖。4.7 起为 WP_JERK(不换算)。`,
        [['WP_JERK', 1.0]]),
      mk('WPNAV_RADIUS', tk.radius, 'cm', [5, 1000],
        '到达容差半径',
        `用途「${purposeCn}」→ ${tk.radius} cm (固件默认 200)。\n首飞/追踪/任务初期 200~300：太小会在航点附近反复修正、任务时间变长，\n看起来像"到不了"；测绘/拍照需要精准过点时再逐次收到 100。\n4.7 起为 WP_RADIUS_M(m) 自动换算。`,
        [['WP_RADIUS_M', 0.01]]),
      mk('PSC_POSXY_P', tk.pos, '', [0.5, 2.0],
        '水平位置环 P',
        `用途「${purposeCn}」→ ${tk.pos} (固件默认 1.0，范围 0.5~2.0)。\n位置误差→目标速度的换算系数。超调/来回摆先降它，追不到再升。\n文档 1.2~1.8 已在合法区间内。4.7 起为 PSC_NE_POS_P(不换算)。`,
        [['PSC_NE_POS_P', 1.0]]),
      mk('PSC_VELXY_P', tk.velp, '', [0.1, 6.0],
        '水平速度环 P',
        `用途「${purposeCn}」→ ${tk.velp} (固件默认 2.0)。\n决定对速度误差的响应力度；加大后配合 PSC_VELXY_FF 更跟手。\n出现水平抖动先降到 2.0。4.7 起为 PSC_NE_VEL_P(不换算)。`,
        [['PSC_NE_VEL_P', 1.0]]),
      mk('PSC_VELXY_I', tk.veli, '', [0.02, 1.0],
        '水平速度环 I',
        `用途「${purposeCn}」→ ${tk.veli} (固件默认 1.0，范围上限就是 1.0)。\n消除稳态误差(顶风漂移)；逆风跟随出现偏移时升到 1.0。\n4.7 起为 PSC_NE_VEL_I；同名 IMAX 在 4.7 缩小 100 倍，本工具不碰。`,
        [['PSC_NE_VEL_I', 1.0]]),
      mk('PSC_VELXY_FF', tk.velff, '', [0, 6],
        '水平速度前馈',
        `用途「${purposeCn}」→ ${tk.velff} (固件默认 0)。\n★ 外部系统已提供目标速度(vx,vy)时，这一项能明显减小滞后：\n  地面站发位置+速度前馈 + 这里给 0.3~0.5。\n只发纯位置指令时给 0.1~0.3 即可。4.7 起为 PSC_NE_VEL_FF。`,
        [['PSC_NE_VEL_FF', 1.0]]),
      mk('PSC_JERK_XY', tk.jxy, 'm/s³', [1, 20],
        '水平轨迹整形 Jerk',
        `用途「${purposeCn}」→ ${tk.jxy} m/s³ (固件默认 5，范围 1~20)。\n控制加速度变化率：越大起步/转向越干脆，过大则机身猛抖。\n与 WPNAV_JERK 不同，它作用在位置控制器的轨迹整形上。4.7 起为 PSC_NE_JERK。`,
        [['PSC_NE_JERK', 1.0]]),
    ];
    for (const e of st.track) e.check = (purpose === 'guided' || purpose === 'auto');
    if (auto) {
      st.track = st.track.concat([
        mk('WP_YAW_BEHAVIOR', 2, '', [0, 3],
          'AUTO 航向行为',
          '0=不改航向; 1=指向下个航点; 2=指向下个航点(RTL 除外,固件默认);\n3=沿 GPS 航迹方向。\n光电/雷达要始终对准任务区 → 提前规划(或用 ROI/固定航向);\n沿航线自然飞、看航迹方向 → 3。仅 AUTO 任务与 RTL 使用，GUIDED 不看它。'),
        mk('TUNE', 10, '', null,
          '遥控器旋钮调参目标 (可选)',
          '10 = WP Speed(实时调 WPNAV_SPEED)。\n要用它必须把某个遥控通道的 RCx_OPTION 设为 219(发射机调参)，\n通道号按实际接线(常用 RC6_OPTION)；同时配套下面 TUNE_MIN/MAX。\n⚠ 默认不勾选：没配通道就写入也没用，配了又不想被覆盖请保持不勾。',
          null, false),
        mk('TUNE_MIN', 500, 'cm/s', null,
          '旋钮最低端对应值 (可选)',
          'TUNE=10 时单位是 cm/s：500 = 5 m/s。\n固件默认 0(旋钮拧到底会把速度写成 0)，建议给 500 左右。默认不勾选。',
          null, false),
        mk('TUNE_MAX', 2000, 'cm/s', null,
          '旋钮最高端对应值 (可选)',
          'TUNE=10 时单位是 cm/s：2000 = 20 m/s(固件速度上限)。\n与 TUNE_MIN 一起保证旋钮全程落在合法区间。默认不勾选。',
          null, false),
      ]);
    }

    st.vert = [
      mk('WPNAV_SPEED_UP', v_up, 'cm/s', [10, 1000],
        'Guided/AUTO 起飞上升速度',
        (auto
          ? `AUTO: 文档建议 300~500 cm/s，档位「${q.name}」→ ${v_up} cm/s；快速爬升的航线可再提高。`
          : `首飞最敏感的一项：离地太猛降、反复贴地升。\n档位「${q.name}」→ ${v_up} cm/s。`)
        + '4.7 起为 WP_SPD_UP(m/s) 自动换算。',
        [['WP_SPD_UP', 0.01]]),
      mk('WPNAV_SPEED_DN', tk.speed_dn, 'cm/s', [10, 500],
        'Guided/AUTO 下降速度',
        `用途「${purposeCn}」→ ${tk.speed_dn} cm/s (固件默认 150)。\n`
        + (auto ? 'AUTO 文档建议 200~400 cm/s：下降过快容易失稳。\n' : '')
        + '目标高度往下走时的下降率上限；降落/掉高太快就调小。\n4.7 起为 WP_SPD_DN(m/s) 自动换算。',
        [['WP_SPD_DN', 0.01]]),
      mk('WPNAV_ACCEL_Z', v_accz, 'cm/s/s', [50, 500],
        '垂直加速度上限',
        (auto
          ? `AUTO 文档建议 500~800，⚠ 固件上限 500 → 取 ${v_accz}；先稳后快。\n`
          : `档位「${q.name}」→ ${v_accz}；\n`)
        + '到顶冲高回荡就减小；过小则爬升迟钝。4.7 起为 WP_ACC_Z(m/s²)。',
        [['WP_ACC_Z', 0.01]]),
      mk('PILOT_SPEED_UP', q.pilot_speed, 'cm/s', [50, 500],
        '手动模式上升速度上限',
        `首飞压到 ${q.pilot_speed}，确认手感后可回默认 250。`,
        [['PILOT_SPD_UP', 0.01]]),
      mk('PILOT_ACCEL_Z', q.pilot_accel, 'cm/s/s', [50, 500],
        '手动模式垂直加速度',
        `档位「${q.name}」→ ${q.pilot_accel}；高度抖动时调小。`,
        [['PILOT_ACC_Z', 0.01]]),
      mk('PSC_ACCZ_P', r(hoverV, 3), '', [0.2, 1.5],
        '垂直加速度 P(由悬停油门派生)',
        '官方 Initial Tuning Flight:\n  ≤4.6: PSC_ACCZ_P = MOT_THST_HOVER\n  4.7+: PSC_D_ACC_P = 0.1 × MOT_THST_HOVER(数值缩小 10 倍)\n本工具按固件实际存在的参数名自动换算，避免写错 10 倍。',
        [['PSC_D_ACC_P', 0.1]]),
      mk('PSC_ACCZ_I', r(2 * hoverV, 3), '', [0.0, 3.0],
        '垂直加速度 I(由悬停油门派生)',
        '官方: ≤4.6 为 2×MOT_THST_HOVER；4.7+ 为 0.2×MOT_THST_HOVER。\n过大会出现缓慢的高度振荡。',
        [['PSC_D_ACC_I', 0.1]]),
      mk('PSC_JERK_Z', v_jerk, '', [5, 50],
        '垂直加加速度(油门变化剧烈程度)',
        (auto
          ? `AUTO 文档建议 10~20，档位「${q.name}」→ ${v_jerk}；高度变化不再突兀。\n`
          : `文档 5~10，档位「${q.name}」取 ${v_jerk}；\n`)
        + '离地瞬间"弹一下" → 加大本值或降 WPNAV_ACCEL_Z。\n4.7 起改名 PSC_D_JERK(不换算)。',
        [['PSC_D_JERK', 1.0]]),
      mk('PSC_VELZ_FF', tk.vzff, '', [0, 1],
        '垂直速度前馈',
        `用途「${purposeCn}」→ ${tk.vzff} (固件默认 0)。\n目标高度连续变化时给一点前馈，高度跟踪会更贴合、滞后更小。\n高度保持不变的场景给 0 即可。4.7 起为 PSC_D_VEL_FF(不换算)。`,
        [['PSC_D_VEL_FF', 1.0]]),
    ];

    st.filter = [
      mk('INS_GYRO_FILTER', p.gyro, 'Hz', [0, 256],
        '陀螺低通滤波频率',
        `官方按桨径: 5寸=80 / 10寸=40 / 20寸=20 Hz，${size}寸 → ${p.gyro} Hz。\n⚠ 该项默认不勾选：若机器振动大，贸然拉高会把振动引进 PID。\n先飞一段看日志振动，再决定是否改。`,
        null, false),
      mk('INS_ACCEL_FILTER', 10, 'Hz', [0, 256],
        '加速度计低通滤波频率',
        '官方初始值 10Hz。'),
      mk('INS_HNTCH_ENABLE', 1, '', [0, 1],
        '谐波陷波滤波器使能',
        '官方建议首飞就开；⚠ 需重启后 INS_HNTCH_FREQ/REF 等参数才出现。\n首飞后用「日志管理」看 FFT 再补 FREQ/BW/REF(REF≈悬停油门)。'),
      mk('INS_HNTCH_MODE', 1, '', [0, 3],
        '陷波中心频率控制方式',
        '0=固定频率, 1=油门跟随(默认,无需传感器), 2/3=转速/FFT。\n有双向 DShot 遥测可改 3。'),
      mk('ATC_RAT_RLL_FLTD', half, 'Hz', [5, 100],
        '横滚速率环 D 滤波',
        `官方规则 = INS_GYRO_FILTER / 2(当前按 ${gyroSrc} ${gyro} → ${half} Hz)。`),
      mk('ATC_RAT_RLL_FLTT', half, 'Hz', [5, 100],
        '横滚速率环前馈滤波',
        `官方规则 = INS_GYRO_FILTER / 2 → ${half} Hz。`),
      mk('ATC_RAT_PIT_FLTD', half, 'Hz', [5, 100],
        '俯仰速率环 D 滤波',
        `官方规则 = INS_GYRO_FILTER / 2 → ${half} Hz。`),
      mk('ATC_RAT_PIT_FLTT', half, 'Hz', [5, 100],
        '俯仰速率环前馈滤波',
        `官方规则 = INS_GYRO_FILTER / 2 → ${half} Hz。`),
      mk('ATC_RAT_YAW_FLTT', half, 'Hz', [5, 100],
        '偏航速率环前馈滤波',
        `官方规则 = INS_GYRO_FILTER / 2 → ${half} Hz。`),
      mk('ATC_RAT_YAW_FLTE', 2, 'Hz', null,
        '偏航速率环误差滤波',
        '官方初始值 2Hz(偏航 D 滤波一般不开)。'),
      mk('ATC_ACCEL_P_MAX', p.acc, 'deg/s/s', [0, 1800],
        '俯仰角加速度上限',
        `官方按桨径: 10寸=1100 / 20寸=500 / 30寸=200，${size}寸 → ${p.acc}。\n大桨必须调低，否则姿态过猛。4.7 起改名 ATC_ACC_P_MAX(不换算)。`,
        [['ATC_ACC_P_MAX', 1.0]]),
      mk('ATC_ACCEL_R_MAX', p.acc, 'deg/s/s', [0, 1800],
        '横滚角加速度上限',
        `同俯仰，${size}寸 → ${p.acc}。4.7 起改名 ATC_ACC_R_MAX。`,
        [['ATC_ACC_R_MAX', 1.0]]),
      mk('ATC_ACCEL_Y_MAX', p.accy, 'deg/s/s', [0, 1800],
        '偏航角加速度上限',
        `官方偏航最保守: 10寸=200 / 20寸=100，${size}寸 → ${p.accy}。\n偏航过快在首飞容易抖；要敏捷可手动改回默认值。4.7 起改名 ATC_ACC_Y_MAX。`,
        [['ATC_ACC_Y_MAX', 1.0]]),
    ];

    st.ekf = [
      mk('EK3_POSNE_M_NSE', 0.5, 'm', [0.1, 10.0],
        'GPS 水平位置观测噪声',
        '固件默认 0.5m。\n⚠ 关键：外部系统给的是"目标点"，不是飞机自身位置，\n  与本项无关。只有飞机自己装 RTK/双天线(定位 <0.1m)才往下调，\n  每次减 10%~20%；GPS 普通就保持默认，调小会 EKF 发散、位置漂移。',
        null, false),
      mk('EK3_VELNE_M_NSE', 0.5, 'm/s', [0.05, 5.0],
        '水平速度观测噪声下限',
        '固件默认 0.3~0.5 m/s。与上一条同理，只在定位源本身很稳时才收紧。',
        null, false),
      mk('EK3_VELD_M_NSE', 0.5, 'm/s', [0.05, 5.0],
        '垂直速度观测噪声下限',
        '固件默认 0.5~0.7 m/s。调小会让高度更"信"速度观测，但对噪声更敏感。',
        null, false),
      mk('EK3_ALT_M_NSE', 2.0, 'm', [0.1, 100.0],
        '气压高度观测噪声',
        '固件默认 2.0m。有气压计防护罩/风扰大时反而应调大，不要盲目调小。',
        null, false),
    ];
    if (auto) {
      st.ekf = st.ekf.concat([
        mk('AHRS_EKF_TYPE', 3, '', [0, 3],
          '姿态估计类型 (体检确认)',
          '保持 3 = 使用 EKF3(Auto 模式必须有位置估计)。\n⚠ AUTO 同样受 EKF 影响：外部定位/GPS/气压不稳会出现航点偏移、\n高度跳动、转弯不顺 —— 不要只靠提高 P 增益解决。默认不勾选。',
          null, false),
        mk('EK3_ENABLE', 1, '', [0, 1],
          '启用 EKF3 (体检确认)',
          '保持 1；改 0/1 需重启飞控才生效。默认不勾选。',
          null, false),
        mk('EK3_SRC1_POSXY', 3, '', [0, 6],
          '水平位置数据源 (体检确认)',
          '固件默认 3=GPS。0=无 3=GPS 4=信标 6=外部导航。\n室外 GPS 用 3；用了外部定位(动捕/RTK 基站解算)才改。默认不勾选。',
          null, false),
        mk('EK3_SRC1_VELXY', 3, '', [0, 7],
          '水平速度数据源 (体检确认)',
          '固件默认 3=GPS。0=无 3=GPS 4=信标 5=光流 6=外部导航 7=轮速计。\n有外部速度源时才改。默认不勾选。',
          null, false),
        mk('EK3_SRC1_POSZ', 1, '', [0, 6],
          '高度数据源 (体检确认)',
          '固件默认 1=气压计。0=无 1=Baro 2=测距 3=GPS 4=信标 6=外部导航。\n默认不勾选。',
          null, false),
        mk('EK3_SRC1_VELZ', 3, '', [0, 6],
          '垂直速度数据源 (体检确认)',
          '固件默认 3=GPS。也可用气压计衍生速度。默认不勾选。',
          null, false),
        mk('EK3_SRC1_YAW', 1, '', [0, 8],
          '航向数据源 (体检确认)',
          '固件默认 1=罗盘。0=无 1=罗盘 2=GPS 3=GPS+罗盘回退 6=外部导航 8=GSF。\n双天线 GPS 才用 2/3。默认不勾选。',
          null, false),
      ]);
    }

    st.safety = [
      mk('FS_GCS_ENABLE', 1, '', [0, 7],
        '地面站失联保护',
        'Guided 控制必须开: 1=RTL(GPS 不可用时自动改 LAND)。\n地面站卡死/MAVLink 中断时按此动作。'),
      mk('BATT_FS_LOW_ACT', 2, '', [0, 5],
        '一级低电压动作',
        '2=RTL；低空调试也可选 1=LAND。'),
      mk('BATT_FS_CRT_ACT', 1, '', [0, 5],
        '严重低电压动作',
        '1=LAND(强制降落，别再返航)。'),
      mk('BATT_LOW_VOLT', r(3.5 * cells, 2), 'V', [0, 100],
        '低电压阈值',
        `3.5V × ${cells}S = ${(3.5 * cells).toFixed(1)}V。\n⚠ 固件默认 10.5V，6S 机器永远不会触发，等于保护没开。`),
      mk('BATT_CRT_VOLT', r(3.3 * cells, 2), 'V', [0, 100],
        '严重低电压阈值',
        `3.3V × ${cells}S = ${(3.3 * cells).toFixed(1)}V，须 ≤ BATT_LOW_VOLT。`),
      mk('FENCE_ENABLE', 1, '', [0, 1], '启用地理围栏', '首飞必须开。'),
      mk('FENCE_TYPE', 3, '', [0, 7], '围栏类型位掩码',
        'bit0=高度, bit1=圆形(圆心 HOME), bit2=多边形; 首飞用 3。'),
      mk('FENCE_ACTION', 1, '', [0, 5], '越界动作', '1=RTL 或 LAND。'),
      mk('FENCE_RADIUS', 50, 'm', [30, 10000], '圆形围栏半径',
        '固件范围 30~10000m(文档写 20m 会被夹到 30m)，首飞 50m。'),
      mk('FENCE_ALT_MAX', 20, 'm', [10, 1000], '允许最大相对高度',
        '必须 ≥ RTL_ALT，否则返航爬升会再次越界形成循环。'),
      mk('FENCE_MARGIN', 5, 'm', [1, 10], '距围栏的预留余量',
        '到达围栏前提前减速的缓冲距离。'),
      mk('RTL_ALT', 1000, 'cm', [0, 10000], '返航爬升高度',
        '默认 1500(15m) > 20m 围栏的爬升安全余量，降到 1000(10m)。\n★ AUTO 任务必须高于航线中最高障碍物。\n4.7 起改名 RTL_ALT_M 且按 m 计，自动换算。',
        [['RTL_ALT_M', 0.01]]),
    ];
    if (auto) {
      st.safety = st.safety.concat([
        mk('RTL_ALT_FINAL', 0, 'cm', [0, 1000],
          '任务/返航结束的最终高度',
          '0 = 到家后自动降落(推荐，任务跑完自己落地)。\n给正数则悬停在该高度等待指令(要人接手时用)。\n必须 ≤ RTL_ALT。4.7 起改名 RTL_ALT_FINAL_M 且按 m 计，自动换算。',
          [['RTL_ALT_FINAL_M', 0.01]]),
        mk('FS_GCS_TIMEOUT', 5, 's', [2, 120],
          '地面站失联多久触发保护 (体检确认)',
          '固件默认 5s；文档建议 3~10s：太长延误保护、太短易误触发。\nAUTO 任务依赖外部系统时尤其要设。默认不勾选(等于固件默认)。',
          null, false),
        mk('FS_EKF_ACTION', 1, '', [1, 3],
          'EKF 异常保护动作 (体检确认)',
          '1=Land(默认，要求位置的模式下转降落) 2=AltHold 3=任何模式都 Land。\n4.7 起额外支持 0=Report only(只记录不动作)。\n复杂任务里确认这个动作是否合适。默认不勾选。',
          null, false),
        mk('DISARM_DELAY', 10, 's', [0, 127],
          '落地后自动上锁延时 (体检确认)',
          '固件默认 10s；0 = 关闭自动上锁。\n太短会在落地判定瞬间就上锁，螺旋桨还在转动时意外停机。\n默认不勾选(等于固件默认)。',
          null, false),
      ]);
    }

    return STAGE_ORDER.map(([sid, label]) => ({ id: sid, label, items: st[sid] || [] }));
  }

  // ---------- 交叉校验 (移植 _ot_validate) ----------
  function otValidate(rows) {
    const issues = [];
    const values = {};
    for (const row of rows) values[row.entry.name] = row.value;
    for (const row of rows) {
      const rng = row.entry.rng;
      const v = row.value;
      if (rng && !(v >= rng[0] && v <= rng[1])) {
        issues.push(`${row.entry.name}=${v} 超出固件范围 ${rng[0]}~${rng[1]}`);
      }
    }
    const arm = values.MOT_SPIN_ARM; const mn = values.MOT_SPIN_MIN;
    if (arm !== undefined && mn !== undefined && mn < arm + 0.03 - 1e-9) {
      issues.push(`MOT_SPIN_MIN(${mn}) 必须 ≥ MOT_SPIN_ARM(${arm}) + 0.03`);
    }
    const alt = values.FENCE_ALT_MAX; const rtl = values.RTL_ALT;
    if (alt !== undefined && rtl !== undefined && alt < rtl / 100.0 - 1e-9) {
      issues.push(`FENCE_ALT_MAX(${alt}m) 必须 ≥ RTL_ALT(${rtl}cm = ${(rtl / 100).toFixed(2)}m)`);
    }
    const low = values.BATT_LOW_VOLT; const crt = values.BATT_CRT_VOLT;
    if (low !== undefined && crt !== undefined && crt > low + 1e-9) {
      issues.push(`BATT_CRT_VOLT(${crt}) 必须 ≤ BATT_LOW_VOLT(${low})`);
    }
    const pa = values.PSC_ACCZ_P; const pi = values.PSC_ACCZ_I; const hover = values.MOT_THST_HOVER;
    if (pa !== undefined && hover !== undefined && pa < 0.2) {
      issues.push(`PSC_ACCZ_P(${pa}) 低于固件下限 0.2 — 悬停油门 ${hover} 过低，请提高"悬停油门"输入(0.2 起)`);
    }
    if (pi !== undefined && hover !== undefined && Math.abs(pi - 2 * hover) > 1e-6) {
      issues.push(`PSC_ACCZ_I(${pi}) ≠ 2×悬停油门(${(2 * hover)}) — 派生值被手动改过，请确认`);
    }
    const speed = values.WPNAV_SPEED; const radius = values.WPNAV_RADIUS;
    if (speed !== undefined && radius !== undefined && speed >= 1500 && radius < 100) {
      issues.push(`WPNAV_RADIUS(${radius}cm) 太小而 WPNAV_SPEED(${speed}cm/s) 很快，目标点附近可能一直"到不了"；建议 ≥100`);
    }
    const dn = values.WPNAV_SPEED_DN; const up = values.WPNAV_SPEED_UP;
    if (dn !== undefined && up !== undefined && dn > up * 3) {
      issues.push(`WPNAV_SPEED_DN(${dn}) 远大于 WPNAV_SPEED_UP(${up})，下降会比爬升快很多`);
    }
    const tmin = values.TUNE_MIN; const tmax = values.TUNE_MAX;
    if (tmin !== undefined && tmax !== undefined && tmin >= tmax - 1e-9) {
      issues.push(`TUNE_MIN(${tmin}) 必须 < TUNE_MAX(${tmax})，否则旋钮一拧就把参数写成反向/超范围值`);
    }
    const fin = values.RTL_ALT_FINAL; const rtl2 = values.RTL_ALT;
    if (fin !== undefined && rtl2 !== undefined && fin > rtl2 + 1e-9) {
      issues.push(`RTL_ALT_FINAL(${fin}cm) 必须 ≤ RTL_ALT(${rtl2}cm)，最终高度不能高过返航爬升高度`);
    }
    return issues;
  }

  window.OneTouchCore = { otStages, otValidate, sizeProfile, r, gz };

  // ================= UI =================
  const $ = (id) => document.getElementById(id);
  const fmt = (v) => {
    if (v === null || v === undefined || v === '') return '—';
    const n = Number(v);
    if (!isFinite(n)) return String(v);
    if (n === Math.trunc(n) && Math.abs(n) < 1e6) return String(Math.trunc(n));
    return String(parseFloat(n.toFixed(4)));
  };

  const state = {
    rows: [],
    connected: false,
    reading: false,
    writing: false,
    accelActive: false,
    accelPos: 0,
    calActive: null,    // 'gyro' | 'baro' | 'level'
    spinActive: false,
    nextPurpose: null,
  };

  const CAL_INFO = {
    gyro: { btn: 'otCalGyro', label: '陀螺校准' },
    baro: { btn: 'otCalBaro', label: '气压校准' },
    level: { btn: 'otCalLevel', label: '水平校准' },
  };
  const ACCEL_BTN = 'otCalAccel';
  const SPIN_BTN = 'otSpin';

  let calTimer = null;
  let accelStartTimer = null;
  let spinTimer = null;

  /** 是否有独占链路的操作正在进行(体检/写入/校准/六面/拆桨) */
  function busyOp() {
    return !!(state.reading || state.writing || state.calActive
      || state.spinActive || state.accelActive);
  }

  function busyText() {
    if (state.calActive) return `${CAL_INFO[state.calActive].label}中...`;
    if (state.spinActive) return '拆桨实测中...';
    if (state.accelActive) return '六面校准中...';
    if (state.reading) return '体检读取中...';
    if (state.writing) return '写入中...';
    return '';
  }

  /** 按钮进行中: 改文案+禁用; 结束后恢复 */
  function setBtnBusy(id, busy, text) {
    const btn = $(id);
    if (!btn) return;
    if (busy) {
      if (!btn.dataset.normalText) btn.dataset.normalText = btn.textContent;
      if (text) btn.textContent = text;
    } else if (btn.dataset.normalText) {
      btn.textContent = btn.dataset.normalText;
      delete btn.dataset.normalText;
    }
  }

  function clearAllTimers() {
    clearTimeout(calTimer);
    clearTimeout(accelStartTimer);
    clearTimeout(spinTimer);
  }

  /** 复位所有"进行中"状态(断连或异常时调用) */
  function resetBusy() {
    clearAllTimers();
    state.calActive = null;
    state.spinActive = false;
    state.accelActive = false;
    state.accelPos = 0;
    state.reading = false;
    state.writing = false;
    for (const k of Object.keys(CAL_INFO)) setBtnBusy(CAL_INFO[k].btn, false);
    setBtnBusy(ACCEL_BTN, false);
    setBtnBusy(SPIN_BTN, false);
  }

  function getCtx() {
    const sizeText = $('otSize') ? $('otSize').value : '5寸';
    const cellsText = $('otCells') ? $('otCells').value : '6S';
    const size = parseInt(String(sizeText).replace(/[^0-9]/g, ''), 10) || 5;
    const cells = parseInt(String(cellsText).replace(/[^0-9]/g, ''), 10) || 6;
    let hover = parseFloat($('otHover').value);
    if (!isFinite(hover)) hover = 0.25;
    hover = Math.max(0.125, Math.min(0.6875, hover));
    const purposeCn = $('otPurpose').value;
    let purpose = 'first';
    for (const [k, v] of Object.entries(D.purposes)) if (v === purposeCn) purpose = k;
    const profileCn = $('otProfile').value;
    const profile = { '更保守': 'soft', '标准': 'std', '更激进': 'hot' }[profileCn] || 'std';
    const gyroParam = (typeof paramCache !== 'undefined' && paramCache)
      ? paramCache.INS_GYRO_FILTER : undefined;
    const ctx = { size, cells, hover, escLinear: $('otEsc').checked, purpose, profile };
    if (gyroParam !== undefined) ctx.gyro = gyroParam;
    return ctx;
  }

  function report(text, clear) {
    const el = $('otReport');
    if (!el) return;
    if (clear) el.textContent = '';
    if (text) {
      el.textContent += text.endsWith('\n') ? text : text + '\n';
      el.scrollTop = el.scrollHeight;
    }
  }

  function stageEnabled(sid) {
    const cb = document.querySelector(`.ot-stage-cb[data-stage="${sid}"]`);
    return cb ? cb.checked : true;
  }

  function buildStagesUI() {
    const box = $('otStages');
    box.innerHTML = '';
    for (const [sid, label] of STAGE_ORDER) {
      const lab = document.createElement('label');
      lab.className = 'ot-stage';
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = true;
      cb.className = 'ot-stage-cb';
      cb.dataset.stage = sid;
      lab.appendChild(cb);
      lab.appendChild(document.createTextNode(' ' + label));
      box.appendChild(lab);
    }
  }

  function generatePlan() {
    const ctx = getCtx();
    const rows = [];
    for (const st of otStages(ctx)) {
      if (!stageEnabled(st.id)) continue;
      for (const e of st.items) {
        rows.push({
          stage: st.label, entry: e, value: e.value, name: e.name,
          scale: 1.0, current: null, check: e.check !== false,
          resolvedName: e.name, autoUncheck: true,
        });
      }
    }
    state.rows = rows;
    state.readDone = false;
    renderTable();
    const nchk = rows.filter((r) => r.check).length;
    const ctxTxt = `用途 ${D.purposes[ctx.purpose]} / 桨径 ${ctx.size}寸 / ${ctx.cells}S / `
      + `悬停油门 ${Number(ctx.hover).toFixed(3)} / 档位 ${PROF_NAME[ctx.profile]}`
      + (ctx.escLinear ? ' / 电调已线性化' : '');
    const lines = [`已生成 ${rows.length} 项方案 (勾选 ${nchk} 项) — ${ctxTxt}`];
    if (ctx.purpose === 'first') {
      lines.push('  ▸ 首飞模式: 第 4 阶段(水平导航)与第 7 阶段(EKF)只展示、默认不勾选。');
      lines.push('  ▸ 首飞悬停稳定后，把"用途"切到「GUIDED 外部引导追踪」或「AUTO 任务航点」重新生成方案。');
    } else if (ctx.purpose === 'guided') {
      lines.push('  ▸ GUIDED 追踪: 第 4 阶段已按移动目标推荐值勾选(含速度前馈 PSC_VELXY_FF)。');
      lines.push('  ▸ 追不上 → 升 WPNAV_ACCEL / WPNAV_JERK；来回摆 → 降这两项或 PSC_POSXY_P。');
      lines.push('  ▸ 围栏: FENCE_RADIUS(50m) 必须 ≥ 实际追踪作业半径，不够就双击该行改大。');
    } else {
      lines.push('  ▸ AUTO 任务: 第 4 阶段按任务取值(速度/加速度/Jerk 走 AUTO 文档区间)，并追加航向行为/遥控调参/EKF 数据源/任务安全项。');
      lines.push('  ▸ 跑不动 → 升 WPNAV_ACCEL / WPNAV_JERK；航点"到不了" → 升 WPNAV_RADIUS。');
      lines.push('  ▸ 机上调参: TUNE=10 + 某通道 RCx_OPTION=219，可旋钮实时改 WPNAV_SPEED。');
      lines.push('  ▸ 第 7 阶段数据源 7 项仅体检确认(GPS=3/气压=1/罗盘=1)，默认不勾选。');
    }
    lines.push('  下一步: ② 读取当前值体检 → ③ 一键写入(自动备份)');
    report(lines.join('\n'), true);
    updateButtons();
  }

  function renderTable() {
    const tbody = $('otTableBody');
    tbody.innerHTML = '';
    state.rows.forEach((row, idx) => {
      row.idx = idx;
      const tr = document.createElement('tr');
      tr.dataset.idx = String(idx);
      tr.className = 'ot-row-missing';

      // 写入勾选
      const tdChk = document.createElement('td');
      tdChk.className = 'ot-col-chk';
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = !!row.check;
      cb.addEventListener('change', () => { row.check = cb.checked; updateButtons(); });
      tdChk.appendChild(cb);

      const tdStage = document.createElement('td');
      tdStage.textContent = row.stage;
      tdStage.className = 'ot-col-stage';

      const tdName = document.createElement('td');
      tdName.textContent = row.entry.name;
      tdName.className = 'ot-col-name';

      const tdCur = document.createElement('td');
      tdCur.className = 'ot-col-cur';
      tdCur.textContent = '—';

      const tdNew = document.createElement('td');
      tdNew.className = 'ot-col-new';
      const inp = document.createElement('input');
      inp.type = 'number';
      inp.value = row.value;
      inp.step = 'any';
      if (row.entry.rng) { inp.min = row.entry.rng[0]; inp.max = row.entry.rng[1]; }
      inp.title = row.entry.help || '';
      inp.addEventListener('change', () => {
        let v = parseFloat(inp.value);
        if (!isFinite(v)) { inp.value = row.value; return; }
        row.value = v;
        inp.value = v;
        updateRowDelta(row);
      });
      tdNew.appendChild(inp);

      const tdUnit = document.createElement('td');
      tdUnit.textContent = row.entry.unit || '';
      tdUnit.className = 'ot-col-unit';

      const tdBasis = document.createElement('td');
      tdBasis.textContent = (row.entry.help || '').split('\n')[0].slice(0, 90);
      tdBasis.className = 'ot-col-basis';
      tdBasis.title = row.entry.desc || '';

      tr.appendChild(tdChk);
      tr.appendChild(tdStage);
      tr.appendChild(tdName);
      tr.appendChild(tdCur);
      tr.appendChild(tdNew);
      tr.appendChild(tdUnit);
      tr.appendChild(tdBasis);
      tbody.appendChild(tr);
      row.tr = tr;
    });
  }

  function updateRowDelta(row) {
    const tdCur = row.tr.querySelector('.ot-col-cur');
    const cb = row.tr.querySelector('input[type="checkbox"]');
    row.tr.classList.remove('ot-row-same', 'ot-row-diff', 'ot-row-missing');
    if (row.current === null || row.current === undefined) {
      tdCur.textContent = '—';
      row.tr.classList.add('ot-row-missing');
      return;
    }
    const tol = Math.max(1e-6, Math.abs(row.value) * 0.005);
    const same = Math.abs(row.value - row.current) <= tol;
    tdCur.textContent = fmt(row.current) + (same ? ' ✓' : '');
    row.tr.classList.add(same ? 'ot-row-same' : 'ot-row-diff');
    if (same && cb.checked && row.autoUncheck) { cb.checked = false; row.check = false; }
  }

  function updateButtons() {
    const connected = state.connected;
    const hasRows = state.rows.length > 0;
    const busy = busyOp();
    $('otBtnPlan').disabled = busy;
    $('otBtnRead').disabled = !connected || !hasRows || busy;
    $('otBtnWrite').disabled = !connected || !hasRows || busy;
    for (const k of Object.keys(CAL_INFO)) $(CAL_INFO[k].btn).disabled = !connected || busy;
    $(ACCEL_BTN).disabled = !connected || busy;
    $(SPIN_BTN).disabled = !connected || busy;

    const label = busyText();
    $('otStatus').textContent = connected ? (label || '已连接') : '未连接';
    $('otStatus').className = 'ot-status '
      + (!connected ? 'bad' : (busy ? 'ok busy' : 'ok'));
  }

  // ---------- 体检 ----------
  function doRead() {
    if (!state.connected) { alert('请先连接飞控'); return; }
    if (busyOp()) { alert('已有操作正在进行，请等待完成后再试'); return; }
    if (!state.rows.length) generatePlan();
    if (!state.rows.length) return;
    state.reading = true;
    updateButtons();
    const params = state.rows.map((row) => ({
      name: row.entry.name,
      aliases: row.entry.aliases || [],
    }));
    report(`体检读取 ${params.length} 项...`, true);
    wsSend({ type: 'ot_read', params });
  }

  function onReadResult(msg) {
    state.reading = false;
    const byIdx = {};
    msg.results.forEach((res, i) => { byIdx[i] = res; });
    let same = 0; let diff = 0; let miss = 0;
    state.rows.forEach((row, i) => {
      const res = msg.results[i];
      if (!res) return;
      row.resolvedName = res.name;
      row.scale = res.scale;
      row.current = res.value;
      if (res.value === null) miss++;
      else if (Math.abs(res.value - row.value) <= Math.max(1e-6, Math.abs(row.value) * 0.005)) same++;
      else diff++;
      updateRowDelta(row);
    });
    const lines = [`体检完成: 已读取 ${state.rows.length - miss}/${state.rows.length}，已一致 ${same}，需改 ${diff}，读不到 ${miss}。`];
    if (msg.volt) {
      const cells = msg.cells;
      const inputCells = parseInt(String($('otCells').value).replace(/[^0-9]/g, ''), 10);
      lines.push(`  实测电压 ${Number(msg.volt).toFixed(2)}V → 推算 ${cells}S`
        + (cells === inputCells ? '' : `  (与当前输入 ${inputCells}S 不一致，请核对)`));
    }
    if (miss) lines.push('  ⚠ 读不到的项多为 4.7 改名或该机未启用，已自动尝试新旧两套参数名');
    lines.push('  下一步: 拆桨实测起转 → ③ 一键写入 → 重启飞控 → 装桨试飞');
    report(lines.join('\n'), true);
    updateButtons();
  }

  // ---------- 写入 ----------
  function doWrite() {
    if (!state.connected) { alert('请先连接飞控'); return; }
    if (busyOp()) { alert('已有操作正在进行，请等待完成后再试'); return; }
    if (!state.rows.length) { alert('请先生成方案'); return; }
    if (!state.readDone) {
      if (!confirm('尚未体检读取当前值，将直接按推荐值写入。建议先「② 读取当前值(体检)」。\n仍要写入？')) return;
    }
    const pending = state.rows.filter((row) => {
      if (!row.check) return false;
      if (row.current !== null && row.current !== undefined
        && Math.abs(row.value - row.current) <= Math.max(1e-6, Math.abs(row.value) * 0.005)) return false;
      return true;
    });
    if (!pending.length) { alert('没有需要写入的参数（未勾选或已与推荐值一致）'); return; }
    const issues = otValidate(pending);
    if (issues.length) {
      if (!confirm('参数校验未通过:\n' + issues.join('\n') + '\n\n仍要写入吗？（不推荐）')) return;
    }
    const byStage = {};
    for (const row of pending) (byStage[row.stage] = byStage[row.stage] || []).push(row);
    const listing = [];
    for (const [, label] of STAGE_ORDER) {
      if (!byStage[label]) continue;
      listing.push(`【${label}】`);
      for (const row of byStage[label]) {
        listing.push(`  ${row.entry.name} → ${fmt(row.value)}${row.entry.unit || ''}`);
      }
    }
    if (!confirm(`将写入 ${pending.length} 个参数（写入前自动备份）:\n\n${listing.join('\n')}\n\n继续？`)) return;

    state.writing = true;
    updateButtons();
    state.pendingWrite = pending;
    report(`一键写入: 开始写入 ${pending.length} 项...`, true);
    const items = pending.map((row) => {
      const fwValue = row.value * (row.scale || 1.0);
      return { key: row.entry.name, name: row.resolvedName || row.entry.name, value: fwValue };
    });
    state.nextPurpose = getCtx().purpose;
    state.pendingItems = items;
    wsSend({ type: 'ot_write', items });
  }

  function onWriteDone(msg) {
    state.writing = false;
    if (msg.aborted) {
      const go = confirm(
        `备份失败, 已暂停写入(未修改任何参数)。\n\n原因: ${msg.reason || '参数读取无响应'}\n\n`
        + '仍要跳过备份直接写入吗？\n(不推荐: 出问题将无法恢复原参数)\n\n'
        + '点"取消"则放弃本次写入。');
      if (go && state.pendingItems && state.pendingItems.length) {
        state.writing = true;
        updateButtons();
        report(`⚠ 已按你的确认跳过备份, 直接写入 ${state.pendingItems.length} 项...`, true);
        wsSend({ type: 'ot_write', items: state.pendingItems, skipBackup: true });
        return;
      }
      report(`已取消写入(未修改任何参数)。备份失败原因: ${msg.reason || '参数读取无响应'}\n`
        + '  建议: 到「备份恢复」页用「备份全部参数」测试链路, 或重新连接后再试。', true);
      updateButtons();
      return;
    }
    const nxt = state.nextPurpose === 'guided'
      ? ('  追踪验证: 重启飞控 → 起飞 3m 切 GUIDED → 先发静止目标(纯位置) →\n'
        + '  再开位置+速度前馈(10~20Hz) → 确认无超调后再追移动目标 →\n'
        + '  追不上升 WPNAV_ACCEL/JERK，来回摆降它们或 PSC_POSXY_P')
      : state.nextPurpose === 'auto'
        ? ('  任务验证: 重启飞控 → 上传航线(必要时 SPLINE 航点平滑转弯) →\n'
          + '  任务里可加 DO_CHANGE_SPEED 转场加速/进区减速、LOITER_TIME 等待 →\n'
          + '  试跑 1 条短航线，确认航向行为(WP_YAW_BEHAVIOR)符合传感器指向需求 →\n'
          + '  ★ 全程保持 LOITER/RTL 随时可切；跑不满速度升 WPNAV_ACCEL/JERK')
        : ('  建议: 重启飞控 → 装桨 → STABILIZE 确认姿态方向 → GUIDED 起飞 2~3m\n'
          + '  → 悬停 30s 落地上锁(学习悬停油门) → 看日志振动/EKF → 再飞高\n'
          + '  → 稳定后把"用途"切到「GUIDED 外部引导追踪」或「AUTO 任务航点」');
    const head = msg.failed && msg.failed.length
      ? `写入完成: 成功 ${msg.success}，失败 ${msg.failed.length} (${msg.failed.join(', ')})`
      : `写入完成: 成功 ${msg.success}`;
    const bk = msg.skippedBackup
      ? '  备份: 已跳过(经你确认, 本次写入无备份)'
      : (msg.backup
        ? `  已自动备份: ${msg.backup} (${msg.backupCount}/${msg.backupTotal || msg.backupCount} 项)`
        : '  已自动备份: (无)');
    report(`${head}\n${nxt}\n${bk}`, true);
    updateButtons();
  }

  // ---------- WS ----------
  function wsSend(obj) {
    if (typeof ws !== 'undefined' && ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
    else alert('WebSocket 未连接');
  }

  function onReadProgress(msg) {
    report(`  体检进度 ${msg.done}/${msg.total} ${msg.name}` + (msg.resolved !== msg.name ? ` (→${msg.resolved})` : ''));
  }

  function handleMessage(msg) {
    switch (msg.type) {
      case 'ot_read_progress': onReadProgress(msg); break;
      case 'ot_read_result': state.readDone = true; onReadResult(msg); break;
      case 'ot_backup': {
        const parts = [];
        parts.push(msg.total && msg.total !== msg.count ? `${msg.count}/${msg.total} 项` : `${msg.count} 项`);
        if (msg.fromCache) parts.push(`其中 ${msg.fromCache} 项取自参数表缓存`);
        if (msg.missing) parts.push(`⚠ ${msg.missing} 项读取失败未入备份`);
        report(`📦 已备份: ${msg.backup} (${parts.join(', ')})`);
        break;
      }
      case 'ot_write_progress': report(`  ${msg.ok ? '✅' : '❌'} ${msg.name} (${msg.done}/${msg.total})`); break;
      case 'ot_write_done': onWriteDone(msg); break;
      case 'ot_cal_done': finishCal(msg.kind, !!msg.ok, msg.text); break;
      case 'ot_accel_cal_prompt': onAccelPrompt(msg); break;
      case 'ot_accel_cal_done': onAccelDone(msg); break;
      case 'ot_spin_progress': report(`  电机${msg.motor} @ ${msg.pct}%: RPM=${msg.rpm}`); break;
      case 'ot_spin_done': onSpinDone(msg); break;
      case 'error':
        // 启动六面校准被拒/失败时复位, 避免按钮卡在"进行中"
        if (state.accelActive && /六面|加速度计/.test(msg.message || '')) {
          clearTimeout(accelStartTimer);
          state.accelActive = false;
          state.accelPos = 0;
          setBtnBusy(ACCEL_BTN, false);
          updateButtons();
        }
        break;
      default: break;
    }
  }
  // ---------- 校准 ----------
  function doCal(kind) {
    if (!state.connected) { alert('请先连接飞控'); return; }
    if (busyOp()) { alert('已有操作正在进行，请等待完成后再试'); return; }
    const info = CAL_INFO[kind];
    state.calActive = kind;
    setBtnBusy(info.btn, true, `⏳ ${info.label}中...`);
    updateButtons();
    report(`开始${info.label}... (等待飞控回执，期间请勿重复点击或执行其它操作)`, true);
    wsSend({ type: 'ot_cal', kind });
    clearTimeout(calTimer);
    // 飞控/链路无回执时兜底复位, 避免按钮永久卡住
    calTimer = setTimeout(() => {
      if (state.calActive === kind) finishCal(kind, null, '未收到完成回执(可能已完成，或链路无响应)');
    }, 12000);
  }

  function finishCal(kind, ok, text) {
    clearTimeout(calTimer);
    const info = CAL_INFO[kind];
    if (!info) return;
    if (state.calActive === kind) {
      state.calActive = null;
      setBtnBusy(info.btn, false);
    }
    updateButtons();
    if (typeof ok === 'boolean') {
      report(`${ok ? '✅' : '⚠'} ${info.label}: ${ok ? (text || '完成') : (text || '失败')}`);
    } else if (text) {
      report(`⚠ ${info.label}: ${text}`);
    }
  }

  function onAccelPrompt(msg) {
    if (msg.pos >= 1) {
      clearTimeout(accelStartTimer);
      state.accelPos = msg.pos;
    }
    let modal = document.getElementById('otAccelModal');
    if (!modal) modal = createAccelModal();
    modal.style.display = 'flex';
    state.accelActive = true;
    setBtnBusy(ACCEL_BTN, true, '⏳ 六面校准中...');
    updateButtons();
    $('otAccelText').textContent = msg.pos >= 1
      ? `请摆放: ${msg.text}  —  摆好并静止后点下面按钮`
      : (msg.text || '正在启动校准...');
    const btn = $('otAccelConfirm');
    btn.disabled = !(msg.pos >= 1);
    btn.onclick = () => {
      btn.disabled = true;
      btn.textContent = '已确认，等待下一面...';
      wsSend({ type: 'ot_accel_cal_confirm', pos: msg.pos });
      setTimeout(() => { btn.textContent = '该面已摆好(发送确认)'; }, 3000);
    };
  }

  function createAccelModal() {
    const div = document.createElement('div');
    div.id = 'otAccelModal';
    div.className = 'ot-modal';
    div.innerHTML = `
      <div class="ot-modal-box">
        <h3>六面加速度计校准</h3>
        <p id="otAccelText">正在启动校准...</p>
        <button id="otAccelConfirm" disabled>该面已摆好(发送确认)</button>
        <p class="ot-modal-hint">提示: LEVEL 最重要，务必按真实水平飞行姿态摆好；其余各面 ±20° 可接受，静止最关键。</p>
        <p class="ot-modal-hint">如需中断: 重启飞控</p>
        <button id="otAccelAbort" class="ot-modal-close">关闭</button>
      </div>`;
    document.body.appendChild(div);
    div.querySelector('#otAccelAbort').onclick = () => {
      div.style.display = 'none';
      wsSend({ type: 'ot_accel_cal_abort' });
    };
    return div;
  }

  function onAccelDone(msg) {
    clearTimeout(accelStartTimer);
    state.accelActive = false;
    state.accelPos = 0;
    setBtnBusy(ACCEL_BTN, false);
    updateButtons();
    report((msg.ok ? '✅ ' : '⚠ ') + (msg.text || (msg.ok ? '六面校准完成' : '六面校准未完成')));
    const modal = document.getElementById('otAccelModal');
    if (modal) {
      $('otAccelText').textContent = (msg.text || '')
        + (msg.ok ? '' : '。可关闭此窗口后重新开始');
      const btn = $('otAccelConfirm');
      btn.disabled = true;
      btn.textContent = '该面已摆好(发送确认)';
      const abortBtn = modal.querySelector('#otAccelAbort');
      if (abortBtn) {
        if (msg.ok) {
          // 已完成: 关闭不应再发中止, 否则会被误报为"未完成"
          abortBtn.textContent = '完成(关闭)';
          abortBtn.onclick = () => { modal.style.display = 'none'; };
        } else {
          abortBtn.textContent = '关闭';
          abortBtn.onclick = () => {
            modal.style.display = 'none';
            wsSend({ type: 'ot_accel_cal_abort' });
          };
        }
      }
    }
  }

  function doSpinTest() {
    if (!state.connected) { alert('请先连接飞控'); return; }
    if (busyOp()) { alert('已有操作正在进行，请等待完成后再试'); return; }
    const modal = document.createElement('div');
    modal.className = 'ot-modal';
    modal.innerHTML = `
      <div class="ot-modal-box">
        <h3>⚠ 拆桨实测 · 二次确认</h3>
        <p>本功能会逐档升油门让电机转动，用来测定起转点。</p>
        <p class="ot-danger">⚠⚠ 必须先拆除所有桨叶！</p>
        <p>需要 ESC 遥测(DShot + 遥测线/双向 DShot)；没有遥测会中止。<br>
           飞控需处于落地、未解锁、安全开关未锁状态。</p>
        <label class="ot-confirm-lab">
          <input type="checkbox" id="otSpinCk">
          我已拆下全部桨叶，确认人员与手指远离电机、飞控已固定
        </label>
        <div class="ot-modal-actions">
          <button id="otSpinGo" disabled>✓ 已拆桨，开始实测</button>
          <button id="otSpinCancel">取消</button>
        </div>
      </div>`;
    document.body.appendChild(modal);
    modal.querySelector('#otSpinCk').addEventListener('change', (e) => {
      modal.querySelector('#otSpinGo').disabled = !e.target.checked;
    });
    modal.querySelector('#otSpinCancel').onclick = () => modal.remove();
    modal.querySelector('#otSpinGo').onclick = () => {
      modal.remove();
      state.spinActive = true;
      setBtnBusy(SPIN_BTN, true, '⏳ 拆桨实测中...');
      updateButtons();
      report('拆桨实测: 从 6% 逐档升油门寻找起转点...(进行中请勿重复点击其它操作)', true);
      wsSend({ type: 'ot_spin_start' });
      clearTimeout(spinTimer);
      // 兜底: 长时间未回执则复位(正常应由 ot_spin_done 结束)
      spinTimer = setTimeout(() => {
        if (state.spinActive) {
          state.spinActive = false;
          setBtnBusy(SPIN_BTN, false);
          updateButtons();
          report('⚠ 拆桨实测等待超时(未收到完成回执)，已复位按钮。请检查链路。', true);
        }
      }, 180000);
    };
  }

  function onSpinDone(msg) {
    clearTimeout(spinTimer);
    state.spinActive = false;
    setBtnBusy(SPIN_BTN, false);
    updateButtons();
    if (msg.ok) {
      // 自动填入方案表
      for (const row of state.rows) {
        if (row.entry.name === 'MOT_SPIN_ARM') { row.value = msg.arm; refreshRowValue(row); }
        else if (row.entry.name === 'MOT_SPIN_MIN') { row.value = msg.min; refreshRowValue(row); }
      }
      report(`✅ 起转点实测 ≈ ${msg.spin}% → 建议 MOT_SPIN_ARM=${msg.arm}、MOT_SPIN_MIN=${msg.min}\n`
        + '  已自动填入方案表；确认无误后点「③ 一键写入」。', true);
    } else {
      report('❌ ' + (msg.text || '拆桨实测失败'), true);
    }
  }

  function refreshRowValue(row) {
    if (!row.tr) return;
    const inp = row.tr.querySelector('.ot-col-new input');
    if (inp) inp.value = row.value;
    updateRowDelta(row);
  }

  // ---------- 帮助 ----------
  function parseHelp(raw) {
    const lines = String(raw || '').split('\n');
    const sectionRe = /^[一二三四五六七八九十]+、/;
    const sections = [];
    const intro = [];
    let title = '帮助说明';
    let cur = null;
    let first = true;
    for (const line of lines) {
      if (/^=+$/.test(line) || /^-{3,}$/.test(line)) continue;
      if (first && line.trim()) { title = line.trim(); first = false; continue; }
      first = false;
      if (sectionRe.test(line)) {
        cur = { title: line.trim(), body: [] };
        sections.push(cur);
      } else if (cur) {
        cur.body.push(line);
      } else if (line.trim()) {
        intro.push(line);
      }
    }
    for (const s of sections) s.body = s.body.join('\n').replace(/^\n+/, '').replace(/\s+$/, '');
    return { title, intro: intro.join('\n').trim(), sections };
  }

  function escHtml(s) {
    return String(s === undefined || s === null ? '' : s)
      .replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  }

  function showHelp() {
    const data = parseHelp(D.help || '');
    const modal = document.createElement('div');
    modal.className = 'ot-modal ot-help-modal';
    modal.innerHTML = `
      <div class="ot-modal-box ot-help-box">
        <div class="ot-help-head">
          <h3>一键首飞 · 帮助说明</h3>
          <span class="ot-help-count"></span>
          <input class="ot-help-search" type="text" placeholder="搜索帮助内容..." />
          <button class="ot-modal-close ot-help-close">关闭</button>
        </div>
        <div class="ot-help-main">
          <nav class="ot-help-toc"></nav>
          <div class="ot-help-content"></div>
        </div>
      </div>`;

    const tocEl = modal.querySelector('.ot-help-toc');
    const contentEl = modal.querySelector('.ot-help-content');
    const searchEl = modal.querySelector('.ot-help-search');
    const countEl = modal.querySelector('.ot-help-count');

    const items = [];
    if (data.intro) items.push({ title: '概述', body: data.intro });
    for (const s of data.sections) items.push({ title: s.title, body: s.body });

    // 目录
    items.forEach((it, i) => {
      const a = document.createElement('a');
      a.className = 'ot-help-toc-item';
      a.textContent = it.title;
      a.onclick = () => {
        const sec = contentEl.querySelector('#otHelpSec' + i);
        if (sec) sec.scrollIntoView({ behavior: 'smooth', block: 'start' });
      };
      tocEl.appendChild(a);
    });

    // 正文
    items.forEach((it, i) => {
      const sec = document.createElement('section');
      sec.className = 'ot-help-sec';
      sec.id = 'otHelpSec' + i;
      const h = document.createElement('h4');
      h.textContent = it.title;
      const pre = document.createElement('pre');
      pre.textContent = it.body;
      sec.appendChild(h);
      sec.appendChild(pre);
      contentEl.appendChild(sec);
    });

    // 搜索过滤 + 高亮
    const applyFilter = () => {
      const q = searchEl.value.trim().toLowerCase();
      let shown = 0;
      const secs = contentEl.querySelectorAll('.ot-help-sec');
      const tocs = tocEl.querySelectorAll('.ot-help-toc-item');
      items.forEach((it, i) => {
        const sec = secs[i];
        const toc = tocs[i];
        if (!sec) return;
        const match = !q || (it.title + '\n' + it.body).toLowerCase().includes(q);
        sec.classList.toggle('hidden', !match);
        if (toc) toc.classList.toggle('hidden', !match);
        if (match) {
          shown++;
          const pre = sec.querySelector('pre');
          if (q) {
            const re = new RegExp(q.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'), 'gi');
            pre.innerHTML = escHtml(it.body).replace(re, (m) => `<mark>${m}</mark>`);
            sec.querySelector('h4').innerHTML = escHtml(it.title).replace(re, (m) => `<mark>${m}</mark>`);
          } else {
            pre.textContent = it.body;
            sec.querySelector('h4').textContent = it.title;
          }
        }
      });
      countEl.textContent = q ? `匹配 ${shown}/${items.length} 节` : `${items.length} 节`;
    };
    searchEl.addEventListener('input', applyFilter);
    applyFilter();

    // 滚动时高亮目录
    contentEl.addEventListener('scroll', () => {
      const top = contentEl.getBoundingClientRect().top;
      let active = 0;
      const secs = contentEl.querySelectorAll('.ot-help-sec');
      secs.forEach((s, i) => { if (s.getBoundingClientRect().top - top <= 4) active = i; });
      tocEl.querySelectorAll('.ot-help-toc-item').forEach((a, i) => a.classList.toggle('active', i === active));
    });

    modal.querySelector('.ot-help-close').onclick = () => modal.remove();
    modal.addEventListener('click', (e) => { if (e.target === modal) modal.remove(); });
    document.addEventListener('keydown', function onEsc(e) {
      if (e.key === 'Escape') { modal.remove(); document.removeEventListener('keydown', onEsc); }
    });
    document.body.appendChild(modal);
    const firstToc = tocEl.querySelector('.ot-help-toc-item');
    if (firstToc) firstToc.classList.add('active');
  }

  function init() {
    buildStagesUI();
    $('otBtnPlan').onclick = generatePlan;
    $('otBtnRead').onclick = doRead;
    $('otBtnWrite').onclick = doWrite;
    $('otBtnHelp').onclick = showHelp;
    $('otBtnClear').onclick = () => report('', true);
    $('otCalGyro').onclick = () => doCal('gyro');
    $('otCalBaro').onclick = () => doCal('baro');
    $('otCalLevel').onclick = () => doCal('level');
    $('otCalAccel').onclick = () => {
      if (!state.connected) { alert('请先连接飞控'); return; }
      if (busyOp()) { alert('已有操作正在进行，请等待完成后再试'); return; }
      if (!confirm('六面加速度计校准需要逐面摆放飞机(水平/左/右/机头下/机头上/背面)，每面摆好静止后点「该面已摆好」。\n\n⚠ 校准期间不要执行其它操作；如需中断请重启飞控。\n开始？')) return;
      state.accelActive = true;
      state.accelPos = 0;
      setBtnBusy(ACCEL_BTN, true, '⏳ 六面校准中...');
      updateButtons();
      report('六面加速度计校准: 正在启动, 请等待飞控给出第一面提示...', true);
      wsSend({ type: 'ot_accel_cal_start' });
      clearTimeout(accelStartTimer);
      // 启动阶段无 prompt 回执则复位(避免卡住)
      accelStartTimer = setTimeout(() => {
        if (state.accelActive && state.accelPos === 0) {
          state.accelActive = false;
          setBtnBusy(ACCEL_BTN, false);
          updateButtons();
          report('⚠ 六面校准启动未收到飞控响应, 已复位。请检查链路后重试。', true);
        }
      }, 8000);
    };
    $('otSpin').onclick = doSpinTest;
    $('otPurpose').addEventListener('change', () => {
      const ctx = getCtx();
      if (ctx.purpose === 'guided') report('用途=GUIDED 外部引导追踪 → 第 4 阶段水平导航参数将按推荐值勾选');
      else if (ctx.purpose === 'auto') report('用途=AUTO 任务航点 → 第 4 阶段按任务取值勾选，并追加 15 项');
      else report('用途=首飞(安全保守) → 第 4 阶段仅展示、默认不勾选');
    });
    updateButtons();
    generatePlan();
  }

  function setConnected(v) {
    state.connected = !!v;
    if (!state.connected) resetBusy();  // 断连时复位所有"进行中"状态
    updateButtons();
  }

  window.OneTouch = { init, handleMessage, setConnected };
  if (typeof document !== 'undefined') window.OneTouch.init();
})();
