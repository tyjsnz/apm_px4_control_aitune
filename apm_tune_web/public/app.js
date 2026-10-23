
// ==================== 参数定义 ====================
const PARAM_GROUPS = [
  {
    title: 'GUIDED/AUTO 导航控制',
    items: [
      { name: '*WPNAV_SPEED', desc: '水平最大导航速度', unit: 'cm/s', min: 1000, max: 2500, step: 100 },
      { name: '*WPNAV_ACCEL', desc: '水平加速度上限', unit: 'cm/s²', min: 500, max: 1500, step: 50 },
      { name: '*WPNAV_JERK', desc: '水平加加速度', unit: 'm/s³', min: 10, max: 30, step: 1 },
      { name: '*WPNAV_RADIUS', desc: '到达容差半径', unit: 'cm', min: 10, max: 500, step: 10 },
      { name: 'WPNAV_SPEED_UP', desc: '最大上升速度', unit: 'cm/s', min: 100, max: 500, step: 10 },
      { name: 'WPNAV_SPEED_DN', desc: '最大下降速度', unit: 'cm/s', min: 100, max: 400, step: 10 },
      { name: 'WPNAV_ACCEL_Z', desc: '垂直加速度上限', unit: 'cm/s²', min: 200, max: 800, step: 50 },
      { name: 'PSC_JERK_Z', desc: '垂直加加速度', unit: 'm/s³', min: 5, max: 20, step: 1 },
    ]
  },
  {
    title: '位置/速度控制环',
    items: [
      { name: '*PSC_POSXY_P', desc: '水平位置环增益', unit: '', min: 0.5, max: 3.0, step: 0.1 },
      { name: '*PSC_VELXY_P', desc: '水平速度环比例增益', unit: '', min: 1.0, max: 3.0, step: 0.1 },
      { name: 'PSC_VELXY_I', desc: '水平速度环积分增益', unit: '', min: 0.1, max: 2.0, step: 0.1 },
      { name: '*PSC_VELXY_FF', desc: '水平速度前馈', unit: '', min: 0.0, max: 1.0, step: 0.05 },
      { name: '*PSC_POSZ_P', desc: '高度位置环P(高度迟缓/过冲先看)', unit: '', min: 0.1, max: 10.0, step: 0.1 },
      { name: '*PSC_VELZ_P', desc: '高度速度环P', unit: '', min: 0.1, max: 3.0, step: 0.05 },
      { name: 'PSC_VELZ_FF', desc: '垂直速度前馈', unit: '', min: 0.0, max: 1.0, step: 0.05 },
      { name: 'PSC_ACCZ_P', desc: '垂直加速度环比例', unit: '', min: 0, max: 1.0, step: 0.01 },
      { name: 'PSC_ACCZ_I', desc: '垂直加速度环积分', unit: '', min: 0, max: 2.0, step: 0.01 },
      { name: 'PSC_ACCZ_FF', desc: '垂直加速度前馈', unit: '', min: 0, max: 1.0, step: 0.05 },
    ]
  },
  {
    title: '电机与悬停基础',
    items: [
      { name: '*MOT_THST_HOVER', desc: '悬停油门估计值', unit: '', min: 0.05, max: 0.5, step: 0.01 },
      { name: 'MOT_HOVER_LEARN', desc: '悬停油门自学习(0/1/2)', unit: '', min: 0, max: 2, step: 1 },
      { name: 'MOT_SPIN_ARM', desc: '解锁怠速输出', unit: '', min: 0.0, max: 0.3, step: 0.01 },
      { name: 'MOT_SPIN_MIN', desc: '飞行最小输出', unit: '', min: 0.0, max: 0.3, step: 0.01 },
      { name: 'MOT_SPIN_MAX', desc: '最大电机输出', unit: '', min: 0.5, max: 1.0, step: 0.01 },
      { name: 'MOT_THST_EXPO', desc: '推力曲线指数', unit: '', min: 0.0, max: 1.0, step: 0.01 },
      { name: 'MOT_PWM_TYPE', desc: '输出协议(0:PWM/1-2:OneShot/3+:DShot)', unit: '', min: 0, max: 7, step: 1 },
      { name: 'MOT_PWM_MIN', desc: '电机输出最小PWM', unit: 'us', min: 750, max: 1250, step: 10 },
      { name: 'MOT_PWM_MAX', desc: '电机输出最大PWM', unit: 'us', min: 1750, max: 2250, step: 10 },
      { name: 'MOT_YAW_HEADROOM', desc: '偏航控制预留油门', unit: '', min: 0.0, max: 0.3, step: 0.01 },
    ]
  },
  {
    title: '姿态与速率环 ATC (调参主体)',
    items: [
      { name: '*ATC_ANG_RLL_P', desc: '横滚角度环P(角度误差→目标角速度)', unit: '', min: 3, max: 12, step: 0.1 },
      { name: '*ATC_ANG_PIT_P', desc: '俯仰角度环P', unit: '', min: 3, max: 12, step: 0.1 },
      { name: 'ATC_ANG_YAW_P', desc: '偏航角度环P', unit: '', min: 3, max: 12, step: 1 },
      { name: '*ATC_RAT_RLL_P', desc: '横滚速率环P(全局振荡时下调)', unit: '', min: 0.01, max: 0.5, step: 0.005 },
      { name: 'ATC_RAT_RLL_I', desc: '横滚速率环I', unit: '', min: 0.01, max: 2.0, step: 0.01 },
      { name: 'ATC_RAT_RLL_D', desc: '横滚速率环D(高频抖动先降滤波)', unit: '', min: 0, max: 0.05, step: 0.001 },
      { name: 'ATC_RAT_RLL_FF', desc: '横滚速率环前馈(跟随性差优先加)', unit: '', min: 0, max: 0.5, step: 0.001 },
      { name: 'ATC_RAT_RLL_FLTT', desc: '横滚目标角速度滤波频率', unit: 'Hz', min: 5, max: 100, step: 1 },
      { name: 'ATC_RAT_RLL_FLTD', desc: '横滚D项滤波频率', unit: 'Hz', min: 5, max: 100, step: 1 },
      { name: '*ATC_RAT_PIT_P', desc: '俯仰速率环P', unit: '', min: 0.01, max: 0.5, step: 0.005 },
      { name: 'ATC_RAT_PIT_I', desc: '俯仰速率环I', unit: '', min: 0.01, max: 2.0, step: 0.01 },
      { name: 'ATC_RAT_PIT_D', desc: '俯仰速率环D', unit: '', min: 0, max: 0.05, step: 0.001 },
      { name: 'ATC_RAT_PIT_FF', desc: '俯仰速率环前馈', unit: '', min: 0, max: 0.5, step: 0.001 },
      { name: 'ATC_RAT_PIT_FLTT', desc: '俯仰目标角速度滤波频率', unit: 'Hz', min: 5, max: 100, step: 1 },
      { name: 'ATC_RAT_PIT_FLTD', desc: '俯仰D项滤波频率', unit: 'Hz', min: 5, max: 100, step: 1 },
      { name: '*ATC_RAT_YAW_P', desc: '偏航速率环P(转向无力/打滑优先加)', unit: '', min: 0.1, max: 2.5, step: 0.005 },
      { name: 'ATC_RAT_YAW_I', desc: '偏航速率环I', unit: '', min: 0.01, max: 1.0, step: 0.01 },
      { name: 'ATC_RAT_YAW_D', desc: '偏航速率环D(通常保持0)', unit: '', min: 0, max: 0.02, step: 0.001 },
      { name: 'ATC_RAT_YAW_FF', desc: '偏航速率环前馈', unit: '', min: 0, max: 0.5, step: 0.001 },
      { name: 'ATC_RAT_YAW_FLTT', desc: '偏航目标角速度滤波频率', unit: 'Hz', min: 1, max: 50, step: 1 },
      { name: 'ATC_RAT_YAW_FLTE', desc: '偏航误差滤波频率(偏航抖动可下调)', unit: 'Hz', min: 0, max: 20, step: 1 },
      { name: '*ATC_RAT_RLL_MAX', desc: '横滚最大角速度', unit: 'deg/s', min: 0, max: 500, step: 5 },
      { name: '*ATC_RAT_PIT_MAX', desc: '俯仰最大角速度', unit: 'deg/s', min: 0, max: 500, step: 5 },
      { name: '*ATC_RAT_YAW_MAX', desc: '偏航最大角速度', unit: 'deg/s', min: 0, max: 500, step: 5 },
      { name: 'ATC_ACCEL_RLL_PIT_MAX', desc: '倾转角加速度上限(过小动作肉)', unit: 'deg/s²', min: 10, max: 1000, step: 10 },
      { name: 'ATC_ACCEL_Y_MAX', desc: '偏航角加速度上限', unit: 'deg/s²', min: 10, max: 1000, step: 10 },
      { name: 'ATC_THR_MIX_MIN', desc: '油门优先下限(姿态让位油门的程度)', unit: '', min: 0.1, max: 0.25, step: 0.01 },
      { name: 'ATC_THR_MIX_MAX', desc: '油门优先上限', unit: '', min: 0.5, max: 0.9, step: 0.01 },
    ]
  },
  {
    title: '起飞参数',
    items: [
      { name: '*TKOFF_THR_MAX', desc: '起飞阶段最大油门', unit: '', min: 0.5, max: 1.0, step: 0.01 },
      { name: '*TKOFF_SLEW_TIME', desc: '起飞油门爬升时间', unit: 's', min: 0.1, max: 3.0, step: 0.1 },
    ]
  },
  {
    title: 'EKF3 状态估计',
    items: [
      { name: 'AHRS_EKF_TYPE', desc: 'EKF类型(保持3)', unit: '', min: 3, max: 3, step: 1 },
      { name: 'EK3_ENABLE', desc: '启用EKF3', unit: '', min: 1, max: 1, step: 1 },
      { name: 'EK3_SRC1_POSXY', desc: '水平位置主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_VELXY', desc: '水平速度主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_POSZ', desc: '高度主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_VELZ', desc: '垂直速度主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_YAW', desc: '航向主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_POS_M_NSE', desc: '位置测量噪声', unit: '', min: 0.01, max: 10.0, step: 0.01 },
      { name: 'EK3_VELD_M_NSE', desc: '速度测量噪声', unit: '', min: 0.01, max: 10.0, step: 0.01 },
      { name: 'EK3_ALT_M_NSE', desc: '高度测量噪声', unit: '', min: 0.01, max: 10.0, step: 0.01 },
    ]
  },
  {
    title: '电池计量与压降补偿',
    items: [
      { name: 'BATT_CAPACITY', desc: '电池容量(-1=自动识别)', unit: 'mAh', min: -1, max: 100000, step: 50 },
      { name: 'BATT_AMP_PERVLT', desc: '电流计比例(电压读数→安培)', unit: 'A/V', min: 1, max: 1000, step: 1 },
      { name: 'BATT_VOLT_MULT', desc: '电压计倍率(分压校准)', unit: '', min: 0, max: 10, step: 0.05 },
      { name: 'MOT_BAT_VOLT_MAX', desc: '压降补偿满电电压(0=关闭补偿)', unit: 'V', min: 6, max: 53, step: 0.1 },
      { name: 'MOT_BAT_VOLT_MIN', desc: '压降补偿低电电压', unit: 'V', min: 6, max: 42, step: 0.1 },
      { name: 'MOT_BATT_CURR_MAX', desc: '最大电流限制(0=不限)', unit: 'A', min: 0, max: 300, step: 1 },
    ]
  },
  {
    title: '安全保护',
    items: [
      { name: '*FS_GCS_ENABLE', desc: '地面站失联保护', unit: '', min: 0, max: 2, step: 1 },
      { name: 'FS_GCS_TIMEOUT', desc: 'GCS失联超时', unit: 's', min: 1, max: 30, step: 1 },
      { name: '*FS_THR_ENABLE', desc: '油门失控保护(0禁用/1返航/2降落/3智能返航/4智能返航或降落)', unit: '', min: 0, max: 4, step: 1 },
      { name: 'FS_THR_VALUE', desc: '油门失联保护阈值', unit: '', min: 880, max: 975, step: 1 },
      { name: '*FENCE_ENABLE', desc: '地理围栏使能', unit: '', min: 0, max: 1, step: 1 },
      { name: 'FENCE_TYPE', desc: '围栏类型', unit: '', min: 0, max: 3, step: 1 },
      { name: '*FENCE_ALT_MAX', desc: '地理围栏最大高度', unit: 'm', min: 5, max: 120, step: 5 },
      { name: '*FENCE_RADIUS', desc: '地理围栏水平半径', unit: 'm', min: 10, max: 500, step: 10 },
      { name: '*RTL_ALT', desc: '返航高度', unit: 'm', min: 10, max: 120, step: 5 },
      { name: 'RTL_ALT_FINAL', desc: '返航后最终高度', unit: 'm', min: 0, max: 50, step: 1 },
      { name: 'DISARM_DELAY', desc: '降落后自动上锁延时', unit: 's', min: 0, max: 10, step: 1 },
      { name: 'FS_EKF_ACTION', desc: 'EKF异常保护动作', unit: '', min: 0, max: 3, step: 1 },
    ]
  },
  {
    title: 'AUTO 模式任务',
    items: [
      { name: 'WP_YAW_BEHAVIOR', desc: 'AUTO机头航向行为', unit: '', min: 0, max: 3, step: 1 },
      { name: 'LOITER_TIME', desc: '航点悬停时间', unit: 's', min: 0, max: 300, step: 1 },
      { name: 'TUNE', desc: '遥控器调参目标参数', unit: '', min: 0, max: 100, step: 1 },
      { name: 'TUNE_MIN', desc: '调参通道最小值', unit: 'cm/s', min: 100, max: 3000, step: 100 },
      { name: 'TUNE_MAX', desc: '调参通道最大值', unit: 'cm/s', min: 100, max: 5000, step: 100 },
    ]
  },
  {
    title: '传感器与滤波',
    items: [
      { name: 'AHRS_ORIENTATION', desc: '飞控安装方向', unit: '', min: 0, max: 31, step: 1 },
      { name: 'INS_GYRO_FILTER', desc: '陀螺仪低通滤波', unit: 'Hz', min: 10, max: 200, step: 5 },
      { name: 'INS_ACCEL_FILTER', desc: '加速度计低通滤波', unit: 'Hz', min: 5, max: 50, step: 1 },
      { name: 'INS_HNTCH_ENABLE', desc: '谐波陷波使能', unit: '', min: 0, max: 1, step: 1 },
      { name: 'INS_HNTCH_MODE', desc: '陷波模式(0静态/1节流/2陀螺动态/3FFT)', unit: '', min: 0, max: 3, step: 1 },
      { name: 'INS_HNTCH_FREQ', desc: '陷波中心频率(静态模式必填)', unit: 'Hz', min: 10, max: 495, step: 1 },
      { name: 'INS_HNTCH_REF', desc: '节流参考值(节流模式,自学习后读取)', unit: '', min: 0, max: 1, step: 0.01 },
      { name: 'INS_HNTCH_HARMICS', desc: '谐波集合(位掩码:1/2/4/8阶,1+2阶=3)', unit: '', min: 1, max: 15, step: 1 },
      { name: 'INS_HNTCH_BW', desc: '陷波带宽', unit: 'Hz', min: 5, max: 250, step: 1 },
    ]
  },
];

// ==================== PX4 参数定义 (v1.14 参数手册核对, 参数名向下兼容 v1.12+) ====================
const PX4_PARAM_GROUPS = [
  {
    title: '位置/速度控制 MPC',
    items: [
      { name: 'MPC_XY_CRUISE', desc: '任务/RTL 水平巡航速度', unit: 'm/s', min: 3, max: 20, step: 0.5 },
      { name: 'MPC_XY_VEL_MAX', desc: 'AUTO 模式水平速度上限', unit: 'm/s', min: 0, max: 20, step: 0.5 },
      { name: 'MPC_Z_VEL_MAX_UP', desc: '手动/Offboard 最大上升速度', unit: 'm/s', min: 0.5, max: 8, step: 0.1 },
      { name: 'MPC_Z_VEL_MAX_DN', desc: '手动/Offboard 最大下降速度', unit: 'm/s', min: 0.5, max: 4, step: 0.1 },
      { name: 'MPC_Z_V_AUTO_UP', desc: 'AUTO/任务 最大上升速度', unit: 'm/s', min: 0.5, max: 8, step: 0.1 },
      { name: 'MPC_Z_V_AUTO_DN', desc: 'AUTO/任务 最大下降速度', unit: 'm/s', min: 0.5, max: 4, step: 0.1 },
      { name: 'MPC_ACC_HOR', desc: '水平加速度(手动/自动)', unit: 'm/s²', min: 2, max: 15, step: 0.5 },
      { name: 'MPC_ACC_HOR_MAX', desc: '最大水平加速度', unit: 'm/s²', min: 2, max: 15, step: 0.5 },
      { name: 'MPC_ACC_UP_MAX', desc: '最大上升加速度', unit: 'm/s²', min: 2, max: 15, step: 0.5 },
      { name: 'MPC_ACC_DOWN_MAX', desc: '最大下降加速度', unit: 'm/s²', min: 2, max: 15, step: 0.5 },
      { name: 'MPC_JERK_MAX', desc: '全局限制加加速度(越大越灵敏)', unit: 'm/s³', min: 0.5, max: 500, step: 0.5 },
      { name: 'MPC_JERK_AUTO', desc: 'AUTO 模式加加速度限制', unit: 'm/s³', min: 1, max: 80, step: 0.5 },
    ]
  },
  {
    title: '起飞与降落',
    items: [
      { name: '*MPC_TKO_SPEED', desc: '起飞爬升速度', unit: 'm/s', min: 1, max: 5, step: 0.1 },
      { name: 'MPC_TKO_RAMP_T', desc: '起飞油门斜坡时间常数(0=关闭斜坡)', unit: 's', min: 0, max: 5, step: 0.5 },
      { name: '*MPC_LAND_SPEED', desc: '降落下降速度', unit: 'm/s', min: 0.6, max: 5, step: 0.1 },
      { name: 'MPC_THR_HOVER', desc: '悬停油门(0~1,按实测/输入)', unit: '', min: 0.1, max: 0.8, step: 0.01 },
      { name: '*MIS_TAKEOFF_ALT', desc: '任务起飞高度', unit: 'm', min: 0, max: 80, step: 0.5 },
    ]
  },
  {
    title: '姿态角环 MC (角度 P)',
    items: [
      { name: '*MC_ROLL_P', desc: '横滚角度环 P(6.5=默认)', unit: '', min: 0, max: 12, step: 0.1 },
      { name: '*MC_PITCH_P', desc: '俯仰角度环 P(6.5=默认)', unit: '', min: 0, max: 12, step: 0.1 },
      { name: 'MC_YAW_P', desc: '偏航角度环 P(2.8=默认)', unit: '', min: 0, max: 5, step: 0.1 },
      { name: 'MC_ROLLRATE_MAX', desc: '横滚最大角速度(220=默认)', unit: 'deg/s', min: 0, max: 1800, step: 5 },
      { name: 'MC_PITCHRATE_MAX', desc: '俯仰最大角速度(220=默认)', unit: 'deg/s', min: 0, max: 1800, step: 5 },
      { name: 'MC_YAWRATE_MAX', desc: '偏航最大角速度(200=默认)', unit: 'deg/s', min: 0, max: 1800, step: 5 },
    ]
  },
  {
    title: '速率环 MC (RATE P/I/D/K, v1.14 命名)',
    items: [
      { name: '*MC_ROLLRATE_P', desc: '横滚速率环 P(0.15=默认)', unit: '', min: 0.01, max: 0.5, step: 0.01 },
      { name: '*MC_ROLLRATE_I', desc: '横滚速率环 I(0.2=默认)', unit: '', min: 0, max: 5, step: 0.01 },
      { name: '*MC_ROLLRATE_D', desc: '横滚速率环 D(0.003=默认)', unit: '', min: 0, max: 0.01, step: 0.0005 },
      { name: 'MC_ROLLRATE_K', desc: '横滚速率环总增益(1.0=默认)', unit: '', min: 0.01, max: 5, step: 0.05 },
      { name: '*MC_PITCHRATE_P', desc: '俯仰速率环 P(0.15=默认)', unit: '', min: 0.01, max: 0.6, step: 0.01 },
      { name: '*MC_PITCHRATE_I', desc: '俯仰速率环 I(0.2=默认)', unit: '', min: 0, max: 5, step: 0.01 },
      { name: '*MC_PITCHRATE_D', desc: '俯仰速率环 D(0.003=默认)', unit: '', min: 0, max: 0.01, step: 0.0005 },
      { name: 'MC_PITCHRATE_K', desc: '俯仰速率环总增益(1.0=默认)', unit: '', min: 0.01, max: 5, step: 0.05 },
      { name: 'MC_YAWRATE_P', desc: '偏航速率环 P(0.2=默认)', unit: '', min: 0, max: 0.6, step: 0.01 },
      { name: 'MC_YAWRATE_I', desc: '偏航速率环 I(0.1=默认)', unit: '', min: 0, max: 5, step: 0.01 },
      { name: 'MC_YAWRATE_D', desc: '偏航速率环 D(0=默认)', unit: '', min: 0, max: 5, step: 0.0005 },
      { name: 'MC_YAWRATE_K', desc: '偏航速率环总增益(1.0=默认)', unit: '', min: 0, max: 5, step: 0.05 },
    ]
  },
  {
    title: '位置环 MPC (XY/Z P-I-D, v1.14)',
    items: [
      { name: '*MPC_XY_P', desc: '水平位置环P(0.95=默认)', unit: '', min: 0, max: 2.0, step: 0.05 },
      { name: '*MPC_XY_VEL_P_ACC', desc: '水平速度环P(1.8=默认)', unit: '', min: 1.2, max: 5.0, step: 0.05 },
      { name: 'MPC_XY_VEL_I_ACC', desc: '水平速度环I(0.4=默认,抗风稳态)', unit: '', min: 0, max: 60, step: 0.05 },
      { name: 'MPC_XY_VEL_D_ACC', desc: '水平速度环D(0.2=默认)', unit: '', min: 0.1, max: 2.0, step: 0.05 },
      { name: '*MPC_Z_P', desc: '垂直位置环P(1.0=默认)', unit: '', min: 0, max: 1.5, step: 0.05 },
      { name: '*MPC_Z_VEL_P_ACC', desc: '垂直速度环P(4.0=默认)', unit: '', min: 2.0, max: 15.0, step: 0.1 },
      { name: 'MPC_Z_VEL_I_ACC', desc: '垂直速度环I(2.0=默认,兼悬停油门学习)', unit: '', min: 0.2, max: 3.0, step: 0.05 },
      { name: 'MPC_Z_VEL_D_ACC', desc: '垂直速度环D(0.0=默认)', unit: '', min: 0, max: 2.0, step: 0.05 },
    ]
  },
  {
    title: '状态估计 EKF2',
    items: [
      { name: 'EKF2_GYR_NOISE', desc: '陀螺噪声(0.015=默认)', unit: 'rad/s', min: 0.0001, max: 0.1, step: 0.001 },
      { name: 'EKF2_ACC_NOISE', desc: '加计噪声(0.35=默认)', unit: 'm/s²', min: 0.01, max: 1, step: 0.01 },
      { name: 'EKF2_MAG_NOISE', desc: '磁力计噪声(0.05=默认)', unit: 'Gauss', min: 0.001, max: 1, step: 0.001 },
      { name: 'EKF2_GPS_P_NOISE', desc: 'GPS 位置噪声(0.5=默认)', unit: 'm', min: 0.01, max: 10, step: 0.01 },
      { name: 'EKF2_GPS_V_NOISE', desc: 'GPS 速度噪声(0.3=默认)', unit: 'm/s', min: 0.01, max: 5, step: 0.01 },
      { name: 'EKF2_BARO_NOISE', desc: '气压高度噪声(3.5=默认)', unit: 'm', min: 0.01, max: 15, step: 0.1 },
    ]
  },
  {
    title: '传感器与输出',
    items: [
      { name: '*IMU_GYRO_CUTOFF', desc: '陀螺低通滤波(40=默认,0=关闭)', unit: 'Hz', min: 0, max: 1000, step: 5 },
      { name: 'IMU_DGYRO_CUTOFF', desc: 'D项滤波(30=默认,0=关闭)', unit: 'Hz', min: 0, max: 1000, step: 5 },
      { name: 'IMU_GYRO_NF0_FRQ', desc: '陷波中心频率(0=禁用)', unit: 'Hz', min: 0, max: 1000, step: 1 },
      { name: 'IMU_GYRO_NF0_BW', desc: '陷波带宽(20=默认)', unit: 'Hz', min: 0, max: 100, step: 1 },
      { name: 'IMU_ACCEL_CUTOFF', desc: '加计低通滤波(30=默认,0=关闭)', unit: 'Hz', min: 0, max: 1000, step: 5 },
      { name: 'IMU_GYRO_RATEMAX', desc: '速率环数据频率(400=默认)', unit: 'Hz', min: 100, max: 2000, step: 100 },
    ]
  },
  {
    title: '任务与返航 NAV/RTL',
    items: [
      { name: '*NAV_ACC_RAD', desc: '航点到达判定半径(10=默认)', unit: 'm', min: 0.05, max: 200, step: 0.5 },
      { name: 'MIS_DIST_1WP', desc: '首航点最大距离(≤0=禁用)', unit: 'm', min: -1, max: 10000, step: 100 },
      { name: '*RTL_RETURN_ALT', desc: '返航爬升高度(60=默认)', unit: 'm', min: 0, max: 8000, step: 0.5 },
      { name: 'RTL_DESCEND_ALT', desc: '到家后开始下降高度(30=默认)', unit: 'm', min: 0, max: 1000, step: 0.5 },
      { name: 'RTL_CONE_ANG', desc: '返航锥角(0/25/45/65/80/90,45=默认)', unit: '°', min: 0, max: 90, step: 1 },
      { name: 'RTL_LAND_DELAY', desc: '到家降落前等待(-1=不降落仅悬停)', unit: 's', min: -1, max: 120, step: 0.5 },
    ]
  },
  {
    title: '安全与保护',
    items: [
      { name: '*COM_RC_LOSS_T', desc: 'RC 丢失判定时间(0.5=默认)', unit: 's', min: 0, max: 35, step: 0.1 },
      { name: '*NAV_RCL_ACT', desc: 'RC 失联动作(1=悬停 2=返航 3=降落 5=终止 6=上锁)', unit: '', min: 1, max: 6, step: 1 },
      { name: 'NAV_DLL_ACT', desc: '数据链丢失(0=禁用 1=悬停 2=返航 3=降落 5=终止 6=上锁)', unit: '', min: 0, max: 6, step: 1 },
      { name: '*COM_DISARM_LAND', desc: '落地自动上锁延时(0=关闭)', unit: 's', min: 0, max: 120, step: 0.1 },
      { name: '*BAT_LOW_THR', desc: '低电阈值(0~1 比例,0.15=默认)', unit: '', min: 0.12, max: 0.5, step: 0.01 },
      { name: 'BAT_CRIT_THR', desc: '严重低电阈值(0.07=默认)', unit: '', min: 0.05, max: 0.25, step: 0.01 },
      { name: 'BAT_EMERGEN_THR', desc: '紧急低电阈值(0.05=默认)', unit: '', min: 0.03, max: 0.1, step: 0.01 },
      { name: '*COM_LOW_BAT_ACT', desc: '低电动作(0=警告 2=降落 3=返航+紧急降落)', unit: '', min: 0, max: 3, step: 1 },
      { name: 'BAT1_CAPACITY', desc: '电池容量(-1=自动识别)', unit: 'mAh', min: -1, max: 100000, step: 50 },
      { name: '*GF_ACTION', desc: '围栏动作(0=无 1=警告 2=悬停 3=返航 4=终止 5=降落)', unit: '', min: 0, max: 5, step: 1 },
      { name: 'GF_MAX_HOR_DIST', desc: '水平围栏最大距离(0=禁用)', unit: 'm', min: 0, max: 10000, step: 1 },
      { name: 'GF_MAX_VER_DIST', desc: '垂直围栏最大距离(0=禁用)', unit: 'm', min: 0, max: 10000, step: 1 },
    ]
  },
];

// ==================== 状态管理 ====================
// ws 由 fclink.js 提供: Web Serial 直连飞控 + 服务器 WS (AI/故障/文件) 的统一传输层
const ws = window.ws;
let paramCache = {};       // { paramId: value }
let currentCategory = 0;
let connected = false;
let connectedPort = null;
let firmware = 'apm';      // 'apm' | 'px4' (由飞控 HEARTBEAT 自动识别)

function activeGroups() {
  return firmware === 'px4' ? PX4_PARAM_GROUPS : PARAM_GROUPS;
}

// 暴露给 onetouch.js 等模块查询当前固件类型
window.getFirmwareType = () => firmware;

/** 固件类型切换: 重渲染参数表 / 推力体检 / 一键首飞方案 */
function setFirmware(fw) {
  if (fw !== 'apm' && fw !== 'px4') return;
  if (firmware === fw) return;
  firmware = fw;
  // 固件切换后官方参数元数据需重新拉取 (AP/PX4 是两份数据)
  if (fwParamMeta.key !== paramMetaFileKey()) {
    fwParamMeta.map = null;
    fwParamMeta.loading = null;
    fwParamMeta.tried = false;
  }
  updateFirmwareBadge();
  if (connected) {
    renderCategories();
    renderThrustCheck();
    if (typeof OneTouch !== 'undefined' && OneTouch.onFirmwareChange) OneTouch.onFirmwareChange();
  }
  console.log('固件类型切换为: ' + fw);
}

function updateFirmwareBadge() {
  const el = document.getElementById('fwVersion');
  if (el) el.textContent = '固件: ' + (firmware === 'px4' ? 'PX4' : 'ArduPilot');
}

// ==================== 串口连接 ====================
function refreshPorts() {
  if (ws.readyState !== 1) return;   // 传输层就绪即可 (串口列表为浏览器本地处理)
  ws.send(JSON.stringify({ type: 'list_ports' }));
}

let portListSignature = '';

function updatePortList(ports) {
  const sel = document.getElementById('portSelect');
  const keep = connectedPort || sel.value;
  const signature = ports.map((p) => `${p.path}|${p.manufacturer || ''}|${p.serialNumber || ''}`).join(',') + '|' + (connectedPort || '');
  if (signature === portListSignature && [...sel.options].some((o) => o.value === keep)) return;
  portListSignature = signature;
  sel.innerHTML = '<option value="">请选择串口</option>';
  for (const p of ports) {
    const opt = document.createElement('option');
    opt.value = p.path;
    if (p.path === 'webserial:pick') {
      // 选择新串口: 点击后弹出系统授权对话框
      opt.textContent = p.manufacturer || '选择新串口…';
      sel.appendChild(opt);
      continue;
    }
    let label = p.path;
    if (p.manufacturer) label += ` (${p.manufacturer})`;
    if (p.serialNumber) label += ` [${p.serialNumber}]`;
    if (p.path === connectedPort) label = `${p.path} (已连接)`;
    opt.textContent = label;
    sel.appendChild(opt);
  }
  if (keep) {
    if (![...sel.options].some((o) => o.value === keep)) {
      const opt = document.createElement('option');
      opt.value = keep;
      opt.textContent = keep + (keep === connectedPort ? ' (已连接)' : '');
      sel.appendChild(opt);
    }
    sel.value = keep;
  }
}

async function connect() {
  const sel = document.getElementById('portSelect').value;
  const baud = parseInt(document.getElementById('baudInput').value);
  if (!sel) return alert('请选择串口');
  let path = sel;
  if (sel === 'webserial:pick') {
    // Web Serial: 必须在用户手势内同步弹出授权对话框 (由 fclink.js 处理)
    if (!FcSerial.supported()) {
      return alert('当前页面无法使用串口: 浏览器不支持 Web Serial。\n'
        + '请用 Chrome/Edge 打开; 若正在用 http://IP 访问, 请改用 localhost 或启用 HTTPS\n'
        + '(详见页面顶部红色提示条)');
    }
  }
  ws.send(JSON.stringify({ type: 'connect', path, baud }));
}

function disconnect() {
  ws.send(JSON.stringify({ type: 'disconnect' }));
}

function requestParams() {
  ws.send(JSON.stringify({ type: 'request_params' }));
}

function setParam(name, value) {
  // 去掉开头的 * 号获取实际参数名
  const paramName = name.replace(/^\*/, '');
  // 判断参数类型 (MAV_PARAM_TYPE: 6=INT32, 9=REAL32)
  // 提示仅为初值: 服务端 setParamSmart 优先用飞控上报类型, 失败时会换类型重试
  let paramType = 9; // REAL32 (绝大多数调参参数为浮点)
  const item = findParamItem(name);
  if (item) {
    const range = getItemRange(item);
    if (range.step >= 1 && range.min === Math.floor(range.min)) {
      paramType = 6; // 整数步进的参数按 INT32 发送
      value = Math.round(value);
    }
  } else if (Number.isInteger(Number(value))) {
    paramType = 6; // 未知参数: 值为整数时先按 INT32 尝试
  }
  recordSessionChange(paramName, parseFloat(value));   // 会话改动记录 (供 diff/回滚)
  ws.send(JSON.stringify({
    type: 'set_param',
    paramId: paramName,
    paramValue: parseFloat(value),
    paramType
  }));
}

function findParamItem(name) {
  for (const group of activeGroups()) {
    for (const item of group.items) {
      if (item.name === name) return item;
    }
  }
  return null;
}

function getItemRange(item) {
  return { min: item.min || 0, max: item.max || 9999, step: item.step || 1 };
}

// ==================== 会话改动记录 (手动写入 diff + 一键回滚) ====================
// 只记录本会话内通过界面手动写入的参数 (一键首飞走 ot_write 通道, 自带备份不重复记录)
const sessionChanges = new Map();   // paramName -> { old, new, time }

function recordSessionChange(name, value) {
  if (!connected || isNaN(value)) return;
  const old = paramCache[name];
  if (old === undefined || old === value) return;   // 未读到原值(无法回滚) / 值未变
  const rec = sessionChanges.get(name);
  if (rec) {
    if (value === rec.old) return;      // 会话中来回切换, 保留最初原值
    rec.new = value;
  } else {
    sessionChanges.set(name, { old, new: value, time: Date.now() });
  }
  renderSessionDiff();
}

function syncSessionChangeOnSet(name, value) {
  const rec = sessionChanges.get(name);
  if (!rec) return;
  if (value === rec.old) sessionChanges.delete(name);   // 已恢复原值, 移出列表
  else rec.new = value;
  renderSessionDiff();
}

function rollbackParam(name) {
  const rec = sessionChanges.get(name);
  if (!rec) return;
  setParam(name, rec.old);   // param_set_sent 确认后自动移出改动列表
}

function rollbackAllParams() {
  if (!sessionChanges.size) return;
  if (!confirm(`将 ${sessionChanges.size} 项参数写回本会话开始时的原值, 继续?`)) return;
  for (const [name, rec] of [...sessionChanges]) setParam(name, rec.old);
}

function renderSessionDiff() {
  const box = document.getElementById('sessionDiff');
  if (!box) return;
  const n = sessionChanges.size;
  box.style.display = n ? 'block' : 'none';
  document.getElementById('sdCount').textContent = n;

  const list = document.getElementById('sdList');
  list.innerHTML = '';
  for (const [name, rec] of sessionChanges) {
    const row = document.createElement('div');
    row.className = 'sd-row';

    const label = document.createElement('span');
    label.className = 'sd-name';
    label.textContent = name;

    const arrow = document.createElement('span');
    arrow.className = 'sd-values';
    arrow.textContent = `${rec.old} → ${rec.new}`;

    const btn = document.createElement('button');
    btn.className = 'btn-mini';
    btn.textContent = '撤销';
    btn.onclick = () => rollbackParam(name);

    row.appendChild(label);
    row.appendChild(arrow);
    row.appendChild(btn);
    list.appendChild(row);
  }
}

function rebootFC() {
  if (confirm('确认重启飞控？重启后需要重新连接。')) {
    ws.send(JSON.stringify({ type: 'reboot' }));
  }
}

// ==================== WebSocket 消息处理 ====================
ws.onopen = () => {
  console.log('WebSocket connected');
  refreshPorts();
  if (window.Stats) Stats.event('page_view');   // 匿名使用统计 (仅 PV/UV, 见 stats.js)
  if (typeof AIConfig !== 'undefined') AIConfig.onOpen();
  if (typeof DataTools !== 'undefined') DataTools.onOpen();
  if (typeof Fault !== 'undefined') Fault.onOpen();
};

ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);

  if (msg.type === 'ports') {
    updatePortList(msg.ports);
  }

  if (msg.type === 'connected') {
    connected = true;
    connectedPort = msg.path;
    const portSel = document.getElementById('portSelect');
    if (![...portSel.options].some((o) => o.value === msg.path)) {
      const opt = document.createElement('option');
      opt.value = msg.path;
      opt.textContent = `${msg.path} (已连接)`;
      portSel.appendChild(opt);
    }
    portSel.value = msg.path;
    portSel.disabled = true;
    document.getElementById('baudInput').disabled = true;
    document.getElementById('status').textContent = `已连接 ${msg.path}`;
    document.getElementById('status').className = 'status-badge connected';
    document.getElementById('btnConnect').disabled = true;
    document.getElementById('btnDisconnect').disabled = false;
    document.getElementById('btnReboot').disabled = false;
    document.getElementById('btnRefreshParams').disabled = false;
    document.getElementById('paramSearch').disabled = false;
    document.getElementById('categoryFilter').disabled = false;
    document.getElementById('telemetryPanel').style.display = 'flex';
    renderCategories();
    if (typeof OneTouch !== 'undefined') OneTouch.setConnected(true);
    if (typeof DataTools !== 'undefined') DataTools.setConnected(true);
    if (typeof Fault !== 'undefined') Fault.setConnected(true);
    scheduleThrustCheck(600);
    requestParams();
  }

  if (msg.type === 'disconnected') {
    connected = false;
    connectedPort = null;
    firmware = 'apm';   // 固件识别状态复位
    updateFirmwareBadge();
    document.getElementById('portSelect').disabled = false;
    document.getElementById('baudInput').disabled = false;
    document.getElementById('status').textContent = '未连接';
    document.getElementById('status').className = 'status-badge disconnected';
    document.getElementById('btnConnect').disabled = false;
    document.getElementById('btnDisconnect').disabled = true;
    document.getElementById('btnReboot').disabled = true;
    document.getElementById('btnRefreshParams').disabled = true;
    document.getElementById('paramSearch').disabled = true;
    document.getElementById('categoryFilter').disabled = true;
    document.getElementById('telemetryPanel').style.display = 'none';
    sessionChanges.clear();      // 断开后丢弃会话改动记录 (原值快照不再可信)
    renderSessionDiff();
    if (typeof OneTouch !== 'undefined') OneTouch.setConnected(false);
    if (typeof DataTools !== 'undefined') DataTools.setConnected(false);
    document.getElementById('paramPanel').innerHTML = `
      <div class="empty-state">
        <p>连接已断开</p>
        <p class="hint">请重新选择串口并连接</p>
      </div>`;
    renderThrustCheck();
  }

  if (msg.type === 'error') {
    alert('错误: ' + msg.message);
  }

  if (msg.type === 'thrust_analyze_start') {
    document.getElementById('btnTcAi').disabled = true;
    document.getElementById('tcAiStatus').textContent = 'AI 分析中...';
  }
  if (msg.type === 'thrust_analyze_done') {
    document.getElementById('btnTcAi').disabled = false;
    const st = document.getElementById('tcAiStatus');
    if (msg.ok) {
      document.getElementById('tcAiReport').value = msg.report || '';
      st.textContent = '完成 ' + new Date().toLocaleTimeString();
    } else {
      st.textContent = '失败: ' + (msg.error || '未知错误');
    }
  }

  // 参数值更新
  if (msg.type === 'param_value') {
    paramCache[msg.paramId] = msg.paramValue;
    updateParamDisplay(msg.paramId, msg.paramValue);
    if (fullParamsActive()) appendFullParamRow(msg.paramId);   // 全量模式: 流式读取时增量插入
    if (TC_PARAMS.includes(msg.paramId)) scheduleThrustCheck();
  }

  // 参数写入确认
  if (msg.type === 'param_set_sent') {
    paramCache[msg.paramId] = msg.paramValue;
    updateParamDisplay(msg.paramId, msg.paramValue);
    syncSessionChangeOnSet(msg.paramId, msg.paramValue);   // 写回原值后自动移出改动列表
    if (TC_PARAMS.includes(msg.paramId)) scheduleThrustCheck();
    // 闪烁提示
    const el = document.getElementById('val_' + msg.paramId);
    if (el) {
      el.classList.add('saved');
      setTimeout(() => el.classList.remove('saved'), 1000);
    }
  }

  // 遥测数据 (顶部遥测条; 单位在 HTML 中单独的 tbar__unit 上)
  if (msg.type === 'global_position') {
    const lat = msg.lat / 1e7;
    const lon = msg.lon / 1e7;
    document.getElementById('telLat').textContent = lat.toFixed(7);
    document.getElementById('telLon').textContent = lon.toFixed(7);
    document.getElementById('telAlt').textContent = (msg.relativeAlt / 100).toFixed(1);
  }

  if (msg.type === 'vfr_hud') {
    document.getElementById('telSpeed').textContent = Number(msg.groundspeed).toFixed(1);
  }

  if (msg.type === 'sys_status') {
    if (msg.voltageBattery) {
      document.getElementById('telBattery').textContent = (msg.voltageBattery / 1000).toFixed(1);
    }
    if (msg.load !== undefined) {
      document.getElementById('loadPercent').textContent = (msg.load / 100).toFixed(0);
    }
  }

  if (msg.type === 'attitude') {
    // 可以在这里显示姿态数据
  }

  if (msg.type === 'heartbeat') {
    if (msg.flightMode) {
      document.getElementById('flightMode').textContent = msg.flightMode + (msg.armed ? ' (已解锁)' : '');
    }
    if (msg.firmware) {
      setFirmware(msg.firmware); // apm / px4 自动切换
      if (window.Stats) Stats.event('fc_connect', { fw: msg.firmware });  // 页内自动去重
    }
  }

  if (msg.type === 'reboot_sent') {
    console.log('重启指令已发送');
  }

  // 一键首飞相关消息
  if (typeof OneTouch !== 'undefined') OneTouch.handleMessage(msg);
  // 电机测试页 (拆桨实测起转)
  if (window.MotorTest) MotorTest.onMsg(msg);
  // AI 配置 (服务商列表 + 服务端默认; Key 在浏览器本地)
  if (typeof AIConfig !== 'undefined') AIConfig.handleMessage(msg);
  // 备份恢复 / 日志分析相关消息
  if (typeof DataTools !== 'undefined') DataTools.handleMessage(msg);
  // 故障速查相关消息
  if (typeof Fault !== 'undefined') Fault.handleMessage(msg);
  // 仪表盘 / 实时曲线 / 消息控制台 (monitor.js)
  if (window.FcMonitor) FcMonitor.onMsg(msg);
  // 传感器体检 (sensors.js)
  if (window.SensorCheck) SensorCheck.onMsg(msg);
  // 起飞就绪检查 (takeoffcheck.js)
  if (window.TakeoffCheck) TakeoffCheck.onMsg(msg);
};

// ==================== UI 渲染 ====================
function renderCategories() {
  const list = document.getElementById('categoryList');
  const filter = document.getElementById('categoryFilter');

  // 更新分类下拉
  filter.innerHTML = '<option value="all">全部分类</option>';
  list.innerHTML = '';

  activeGroups().forEach((g, gi) => {
    // 下拉选项
    const opt = document.createElement('option');
    opt.value = gi;
    opt.textContent = g.title;
    filter.appendChild(opt);

    // 侧边栏
    const cat = document.createElement('div');
    cat.className = 'category';
    cat.dataset.index = gi;
    cat.innerHTML = `<h3>${g.title}</h3>`;
    cat.onclick = () => scrollToCategory(gi);
    list.appendChild(cat);
  });

  // 全量参数浏览器入口 (飞控实读参数全集, 未收录参数也可查可写)
  const fullOpt = document.createElement('option');
  fullOpt.value = '__full__';
  fullOpt.textContent = '全量参数 (FC 实读)';
  filter.appendChild(fullOpt);

  const fullCat = document.createElement('div');
  fullCat.className = 'category';
  fullCat.dataset.index = 'full';
  fullCat.innerHTML = '<h3>全量参数 (FC 实读)</h3>';
  fullCat.onclick = () => scrollToCategory('full');
  list.appendChild(fullCat);

  // 默认选中第一个
  if (activeGroups().length) {
    list.querySelector('.category').classList.add('active');
    currentCategory = 0;
    renderParams();
  }
}

function setActiveCategory(gi) {
  document.querySelectorAll('.category').forEach((c) => {
    c.classList.toggle('active', String(c.dataset.index) === String(gi));
  });
  currentCategory = gi;
}

function highlightGroup(el) {
  if (!el) return;
  el.scrollIntoView({ behavior: 'smooth', block: 'start' });
  el.classList.remove('highlight');
  void el.offsetWidth; // 重启动画
  el.classList.add('highlight');
  setTimeout(() => el.classList.remove('highlight'), 1200);
}

function scrollToCategory(gi) {
  setActiveCategory(gi);

  // 全量参数浏览器: 同步下拉框并重渲染 (分组 id 为 group-full)
  if (String(gi) === 'full') {
    document.getElementById('categoryFilter').value = '__full__';
    renderParams();
    highlightGroup(document.getElementById('group-full'));
    return;
  }

  let el = document.getElementById('group-' + gi);
  if (!el) {
    // 目标分组被下拉框/搜索过滤掉了，重置筛选后重新渲染
    document.getElementById('categoryFilter').value = 'all';
    document.getElementById('paramSearch').value = '';
    renderParams();
    el = document.getElementById('group-' + gi);
  }
  highlightGroup(el);
}

// 滚动时同步左侧选中项
function syncActiveCategory() {
  const panel = document.getElementById('paramPanel');
  const groups = panel.querySelectorAll('.param-group');
  if (!groups.length) return;

  const top = panel.getBoundingClientRect().top;
  let active = groups[0];

  // 已滚到底部：最后一个分组不满一屏，始终选中它
  if (panel.scrollTop + panel.clientHeight >= panel.scrollHeight - 4) {
    active = groups[groups.length - 1];
  } else {
    for (const g of groups) {
      if (g.getBoundingClientRect().top - top <= 24) active = g;
      else break;
    }
  }
  setActiveCategory(active.dataset.index);
}

function renderParams() {
  const panel = document.getElementById('paramPanel');
  const search = document.getElementById('paramSearch').value.toLowerCase();
  const catFilter = document.getElementById('categoryFilter').value;
  const starOnly = document.getElementById('starFilter').checked;

  panel.innerHTML = '';

  // 全量参数浏览器模式: 枚举飞控实读参数全集
  if (catFilter === '__full__') {
    renderFullParams(search, starOnly);
    return;
  }

  let totalCount = 0;
  let shownCount = 0;

  activeGroups().forEach((group, gi) => {
    if (catFilter !== 'all' && parseInt(catFilter, 10) !== gi) return;

    let groupHasVisible = false;
    const groupDiv = document.createElement('div');
    groupDiv.className = 'param-group';
    groupDiv.id = 'group-' + gi;
    groupDiv.dataset.index = gi;

    const groupHeader = document.createElement('div');
    groupHeader.className = 'param-group-header';
    groupHeader.textContent = group.title;
    groupDiv.appendChild(groupHeader);

    for (const item of group.items) {
      totalCount++;
      const name = item.name;
      const namePlain = name.replace(/^\*/, '');
      const isStar = name.startsWith('*');

      // 过滤
      if (starOnly && !isStar) continue;
      if (search) {
        const matchName = namePlain.toLowerCase().includes(search) || name.toLowerCase().includes(search);
        const matchDesc = item.desc.toLowerCase().includes(search);
        if (!matchName && !matchDesc) continue;
      }

      shownCount++;
      groupHasVisible = true;

      groupDiv.appendChild(buildParamRow({
        name,
        desc: item.desc,
        unit: item.unit,
        range: getItemRange(item),
      }));
    }

    if (groupHasVisible) {
      panel.appendChild(groupDiv);
    }
  });

  if (shownCount === 0) {
    panel.innerHTML = '<div class="empty-state"><p>没有找到匹配的参数</p></div>';
  }
}

// ---------- 参数行构建 (参数表/全量浏览器共用) ----------
function buildParamRow(opt) {
  const name = opt.name;
  const namePlain = name.replace(/^\*/, '');
  const isStar = name.startsWith('*');
  const noRange = !!opt.noRange;   // 全量模式的未收录参数: 无推荐范围, 不做范围校验
  const range = noRange ? null : (opt.range || getItemRange({ min: 0, max: 9999, step: 1 }));

  const row = document.createElement('div');
  row.className = 'param-row';
  row.dataset.paramName = namePlain;

  // 参数名
  const label = document.createElement('div');
  label.className = 'param-name';
  const starHtml = isStar ? '<span class="star">*</span>' : '';
  label.innerHTML = `${starHtml}${name}`;

  // 当前值
  const currentVal = paramCache[namePlain] !== undefined ? paramCache[namePlain] : '';
  const valInput = document.createElement('input');
  valInput.type = 'number';
  valInput.className = 'param-value';
  valInput.id = 'val_' + namePlain;
  if (noRange) {
    valInput.step = 'any';
  } else {
    valInput.step = range.step || 1;
    valInput.min = range.min;
    valInput.max = range.max;
  }
  valInput.value = currentVal !== '' ? currentVal : (noRange ? '' : (range.min || ''));
  valInput.placeholder = '读取中...';
  valInput.disabled = !connected;

  // 单位
  const unitSpan = document.createElement('span');
  unitSpan.className = 'param-unit';
  unitSpan.textContent = opt.unit || '';

  // 描述
  const desc = document.createElement('div');
  desc.className = 'param-hint';
  desc.textContent = opt.desc || '';

  // 写入按钮
  const btn = document.createElement('button');
  btn.className = 'btn-write';
  btn.textContent = '写入';
  btn.onclick = () => {
    const v = parseFloat(valInput.value);
    if (isNaN(v)) return alert('请输入有效数值');
    if (!noRange && (v < range.min || v > range.max)) {
      if (!confirm(`值 ${v} 超出推荐范围 [${range.min}, ${range.max}]，确定写入？`)) return;
    }
    setParam(name, v);
  };

  row.appendChild(label);
  row.appendChild(valInput);
  row.appendChild(unitSpan);
  row.appendChild(desc);
  row.appendChild(btn);
  return row;
}

// ---------- 全量参数浏览器: 飞控实读参数全集 ----------
/** 参数表索引: { 参数名 -> PARAM_GROUPS 中的定义项 } */
function fullParamMeta() {
  const map = new Map();
  for (const g of activeGroups()) {
    for (const it of g.items) {
      const plain = it.name.replace(/^\*/, '');
      if (!map.has(plain)) map.set(plain, it);
    }
  }
  return map;
}

// ---------- 固件官方参数元数据 (懒加载, 补全量视图的描述/单位/范围) ----------
// 数据由 scripts/gen_param_meta.py 生成 (ArduPilot autotest pdef + PX4 artifacts)
const fwParamMeta = { key: null, map: null, loading: null, tried: false };

function paramMetaFileKey() {
  return firmware === 'px4' ? 'px4' : 'apm_copter';
}

function loadParamMeta() {
  const key = paramMetaFileKey();
  if (fwParamMeta.map && fwParamMeta.key === key) return Promise.resolve(fwParamMeta.map);
  if (fwParamMeta.loading && fwParamMeta.key === key) return fwParamMeta.loading;
  fwParamMeta.key = key;
  fwParamMeta.loading = fetch('parammeta/' + key + '.json')
    .then((r) => (r.ok ? r.json() : null))
    .then((j) => {
      fwParamMeta.map = (j && typeof j === 'object') ? j : null;
      fwParamMeta.loading = null;
      fwParamMeta.tried = true;
      return fwParamMeta.map;
    })
    .catch(() => {
      fwParamMeta.loading = null;
      fwParamMeta.tried = true;
      return null;
    });
  return fwParamMeta.loading;
}

function ensureParamMetaLoaded() {
  // tried 防止加载失败后 renderParams↔ensure 互相触发死循环
  if (fwParamMeta.map || fwParamMeta.loading || fwParamMeta.tried) return;
  loadParamMeta().then(() => {
    // 元数据到达后若仍在全量视图则刷新, 让占位行补上官方描述
    if (fullParamsActive()) renderParams();
  });
}

function firmwareParamEntry(namePlain) {
  const m = fwParamMeta.map;
  const e = m && m[namePlain];
  return (e && typeof e === 'object') ? e : null;
}

/** 全量视图中"内置表未收录"参数的行: 官方元数据优先, 否则占位文案 */
function buildUnlistedParamRow(namePlain) {
  const e = firmwareParamEntry(namePlain);
  if (!e) {
    return buildParamRow({ name: namePlain, desc: '固件实读参数 (参数表未收录)', unit: '', noRange: true });
  }
  const parts = [];
  if (e.n) parts.push(e.n);
  if (e.lo !== undefined && e.hi !== undefined) parts.push('范围 ' + e.lo + '~' + e.hi);
  const row = buildParamRow({ name: namePlain, desc: parts.join(' · '), unit: e.u || '', noRange: true });
  if (e.d) {
    const hint = row.querySelector('.param-hint');
    if (hint) hint.title = e.d;   // 完整官方说明做悬停提示
  }
  return row;
}

function renderFullParams(search, starOnly) {
  const panel = document.getElementById('paramPanel');
  const names = Object.keys(paramCache).sort();
  if (!names.length) {
    panel.innerHTML = '<div class="empty-state"><p>尚未读取到飞控参数</p><p class="hint">连接后自动拉取, 也可点击「刷新参数」</p></div>';
    return;
  }
  ensureParamMetaLoaded();   // 描述缺失时后台拉官方元数据, 到达后自动刷新本视图
  const metaMap = fullParamMeta();

  const groupDiv = document.createElement('div');
  groupDiv.className = 'param-group';
  groupDiv.id = 'group-full';
  groupDiv.dataset.index = 'full';

  const header = document.createElement('div');
  header.className = 'param-group-header';
  header.id = 'fullHeader';
  header.textContent = `全量参数 (飞控实读 ${names.length} 项)`;
  groupDiv.appendChild(header);

  let shown = 0;
  for (const plain of names) {
    const known = metaMap.get(plain);
    if (starOnly && !(known && known.name.startsWith('*'))) continue;
    if (search) {
      const hit = plain.toLowerCase().includes(search) ||
        (known && known.desc.toLowerCase().includes(search));
      if (!hit) continue;
    }
    shown++;
    groupDiv.appendChild(known
      ? buildParamRow({ name: known.name, desc: known.desc, unit: known.unit, range: getItemRange(known) })
      : buildUnlistedParamRow(plain));
  }

  if (!shown) {
    panel.innerHTML = '<div class="empty-state"><p>没有找到匹配的参数</p></div>';
    return;
  }
  panel.appendChild(groupDiv);
}

function fullParamsActive() {
  return document.getElementById('categoryFilter').value === '__full__';
}

/** 参数流式读取过程中, 增量插入新到达的行 (避免整页重渲染丢滚动位置/输入) */
function appendFullParamRow(namePlain) {
  let group = document.getElementById('group-full');
  if (!group) { renderParams(); return; }   // 面板尚无全量分组 (如从空状态恢复)
  if (document.getElementById('val_' + namePlain)) return;   // 行已存在

  const metaMap = fullParamMeta();
  const known = metaMap.get(namePlain);
  const row = known
    ? buildParamRow({ name: known.name, desc: known.desc, unit: known.unit, range: getItemRange(known) })
    : buildUnlistedParamRow(namePlain);

  // 按参数名排序插入
  const rows = [...group.querySelectorAll('.param-row')];
  const after = rows.find((r) => r.dataset.paramName > namePlain);
  if (after) group.insertBefore(row, after);
  else group.appendChild(row);

  const header = document.getElementById('fullHeader');
  if (header) header.textContent = `全量参数 (飞控实读 ${Object.keys(paramCache).length} 项)`;
}

function updateParamDisplay(paramId, value) {
  const el = document.getElementById('val_' + paramId);
  if (el) {
    el.value = value;
  }
}

// ==================== 推力线性化体检 (独立 tab, 纯逻辑在 thrustcheck.js) ====================
const TC_PARAMS = window.ThrustCheck.TC_PARAMS;
let tcTimer = null;
let tcOverrides = {};   // 手工输入值 {参数名: 数值}, 优先于 paramCache 实读值

function scheduleThrustCheck(delay) {
  clearTimeout(tcTimer);
  tcTimer = setTimeout(renderThrustCheck, delay === undefined ? 300 : delay);
}

/** 当前生效的体检参数值: 手填覆盖 > 飞控实读 */
function tcValues() {
  const out = {};
  for (const n of TC_PARAMS) {
    const v = tcOverrides[n] !== undefined ? tcOverrides[n] : paramCache[n];
    if (v !== undefined && v !== null && v !== '' && isFinite(Number(v))) out[n] = Number(v);
  }
  return out;
}

function tcEscLinear() {
  return !!(document.getElementById('tcEscLinear')?.checked
    || document.getElementById('otEsc')?.checked);
}

function renderThrustCheck() {
  const rowsEl = document.getElementById('tcRows');
  const sumEl = document.getElementById('tcSummary');
  const verdictEl = document.getElementById('tcVerdict');
  if (!rowsEl) return;

  const setEmpty = (sumText, sumCls, verdict) => {
    rowsEl.innerHTML = '';
    sumEl.textContent = sumText;
    sumEl.className = 'tc-summary' + (sumCls ? ' ' + sumCls : '');
    verdictEl.textContent = verdict;
    const cv = document.getElementById('tcCurve');
    if (cv && cv.getContext) cv.getContext('2d').clearRect(0, 0, cv.width, cv.height);
  };

  // PX4: 对应概念是 THR_MDL_FAC(默认 0 即可), 给指引而非隐藏
  if (connected && firmware === 'px4') {
    setEmpty('—', '', '推力线性化体检为 ArduPilot 专属 (MOT_* 参数体系); PX4 对应 THR_MDL_FAC (默认 0 一般无需配置), 电调配置请用 QGC。');
    return;
  }
  const hasOverride = Object.keys(tcOverrides).length > 0;
  if (!connected && !hasOverride) {
    setEmpty('未连接', '', '连接飞控并读取参数后自动评估; 也可在下方手工填写关键参数直接体检。');
    return;
  }
  const values = tcValues();
  if (!Object.keys(values).length) {
    if (hasOverride) setEmpty('—', '', '手填值无效，请检查输入');
    else setEmpty('读取中...', '', '正在读取推力相关参数...');
    return;
  }

  const rows = window.ThrustCheck.buildRows(values, { escLinear: tcEscLinear() });
  const warns = rows.filter((r) => r.level === 'warn').length;
  const oks = rows.filter((r) => r.level === 'ok').length;
  sumEl.textContent = (hasOverride ? '手填值 · ' : '')
    + (warns ? `${warns} 项需关注 / 共 ${rows.length} 项` : `正常 ${oks}/${rows.length}`);
  sumEl.className = 'tc-summary ' + (warns ? 'bad' : 'good');
  verdictEl.textContent = window.ThrustCheck.verdict(rows, values);

  rowsEl.innerHTML = rows.map((r) => `
    <div class="tc-row ${r.level}">
      <span class="tc-name">${r.name}</span>
      <span class="tc-val">${r.value}</span>
      <span class="tc-text">${r.text}</span>
    </div>`).join('');

  drawTcCurve(values);
  tcSyncInputs(values);
}

/** 曲线预览: 期望推力 -> 油门指令 (AP_Motors_Thrust_Linearization 公式) */
function drawTcCurve(values) {
  const cv = document.getElementById('tcCurve');
  if (!cv || !cv.getContext) return;
  const ctx = cv.getContext('2d');
  const W = cv.width; const H = cv.height;
  const pad = { l: 44, r: 12, t: 18, b: 26 };
  const iw = W - pad.l - pad.r; const ih = H - pad.t - pad.b;
  ctx.clearRect(0, 0, W, H);
  const css = getComputedStyle(document.body);
  const cs = (name, dft) => (css.getPropertyValue(name) || dft).trim();
  const cText = cs('--text-secondary', '#8b949e');
  const cGrid = cs('--border-color', '#30363d');
  const cIdeal = cs('--text-muted', '#6e7681');
  const cExpo = cs('--accent', '#58a6ff');
  const cHover = cs('--success', '#3fb950');
  const X = (t) => pad.l + t * iw;
  const Y = (a) => pad.t + (1 - a) * ih;

  // 网格 + 坐标
  ctx.strokeStyle = cGrid; ctx.lineWidth = 1;
  ctx.fillStyle = cText; ctx.font = '10px sans-serif';
  for (let i = 0; i <= 4; i++) {
    const g = i / 4;
    ctx.beginPath(); ctx.moveTo(X(g), Y(0)); ctx.lineTo(X(g), Y(1)); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(X(0), Y(g)); ctx.lineTo(X(1), Y(g)); ctx.stroke();
    ctx.fillText((g * 100).toFixed(0) + '%', X(g) - 10, H - 9);
    ctx.fillText((g * 100).toFixed(0) + '%', 6, Y(g) + 3);
  }

  // 理想线性 a=T
  ctx.strokeStyle = cIdeal; ctx.setLineDash([4, 4]); ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(1), Y(1)); ctx.stroke();
  ctx.setLineDash([]);

  // 当前 EXPO 补偿曲线
  const expo = Number(values.MOT_THST_EXPO) || 0;
  const liftMax = values.MOT_THR_MAX !== undefined ? Number(values.MOT_THR_MAX) : 1;
  const pts = window.ThrustCheck.curvePoints(expo, { steps: 60, liftMax });
  ctx.strokeStyle = cExpo; ctx.lineWidth = 2;
  ctx.beginPath();
  pts.forEach((p, i) => { if (i === 0) ctx.moveTo(X(p.t), Y(p.a)); else ctx.lineTo(X(p.t), Y(p.a)); });
  ctx.stroke();

  // 悬停点: 油门指令 = MOT_THST_HOVER 处的期望推力
  const hover = Number(values.MOT_THST_HOVER);
  if (isFinite(hover) && hover >= 0 && hover <= 1) {
    const tHover = window.ThrustCheck.actuatorToExpoThrust(hover, expo, liftMax);
    ctx.fillStyle = cHover;
    ctx.beginPath(); ctx.arc(X(tHover), Y(hover), 4, 0, Math.PI * 2); ctx.fill();
    ctx.fillText('悬停', X(tHover) + 6, Y(hover) - 6);
  }
  ctx.fillStyle = cText;
  ctx.fillText(`期望推力 → 油门指令 (EXPO=${expo})`, pad.l - 30, 11);
}

// 主题切换: 推力曲线已逐次读 CSS 变量, 触发一次重绘即可
if (typeof document !== 'undefined' && document.addEventListener) {
  document.addEventListener('fy:theme', () => {
    try { renderThrustCheck(); } catch (e) { /* 忽略 */ }
  });
}

/** 手工输入框: 仅回填无手填覆盖的参数 */
function tcSyncInputs(values) {
  for (const n of window.ThrustCheck.TC_EDITABLE) {
    const inp = document.querySelector(`#tcInputs input[data-p="${n}"]`);
    if (!inp || tcOverrides[n] !== undefined) continue;
    inp.value = values[n] !== undefined ? values[n] : '';
  }
}

function buildTcInputs() {
  const box = document.getElementById('tcInputs');
  if (!box) return;
  box.innerHTML = window.ThrustCheck.TC_EDITABLE.map((n) => `
    <label class="tc-input-item" title="${n}">
      <span>${n.replace('MOT_', '')}</span>
      <input type="number" step="any" data-p="${n}" placeholder="--" />
    </label>`).join('');
}

// 手动体检: 重新从飞控全量拉取 (param_value 事件会触发重渲染)
document.getElementById('btnTcRead')?.addEventListener('click', () => {
  if (!connected) { alert('请先连接飞控'); return; }
  tcOverrides = {};
  document.querySelectorAll('#tcInputs input').forEach((i) => { i.value = ''; });
  requestParams();
  scheduleThrustCheck(1500); // 兜底重渲染
});

// 用手工填写的假设值评估 (不写入飞控)
document.getElementById('btnTcReval')?.addEventListener('click', () => {
  const ov = {};
  document.querySelectorAll('#tcInputs input').forEach((i) => {
    const v = i.value.trim();
    if (v !== '' && isFinite(Number(v))) ov[i.dataset.p] = Number(v);
  });
  tcOverrides = ov;
  renderThrustCheck();
});

// 清除手填值, 回到飞控实读值
document.getElementById('btnTcReset')?.addEventListener('click', () => {
  tcOverrides = {};
  document.querySelectorAll('#tcInputs input').forEach((i) => { i.value = ''; });
  renderThrustCheck();
});

['tcEscLinear', 'otEsc'].forEach((id) => {
  document.getElementById(id)?.addEventListener('change', () => scheduleThrustCheck(0));
});

// AI 分析: 把体检数据发给服务器 AI 代理
function tcAiAnalyze() {
  const statusEl = document.getElementById('tcAiStatus');
  const values = tcValues();
  if (!Object.keys(values).length) {
    if (statusEl) statusEl.textContent = '暂无体检数据: 请先读取参数或手工填写';
    return;
  }
  if (typeof AIConfig !== 'undefined' && !AIConfig.isConfigured()) {
    if (confirm('尚未配置可用的 AI。\n\n打开「AI 设置」填入你自己的 API Key（仅保存在本机浏览器）？')) AIConfig.openSettings();
    return;
  }
  const rows = window.ThrustCheck.buildRows(values, { escLinear: tcEscLinear() });
  document.getElementById('btnTcAi').disabled = true;
  if (statusEl) statusEl.textContent = '分析中...';
  ws.send(JSON.stringify({
    // AI 配置随请求携带 (本机 Key), 服务端仅代理且不保存
    ...(typeof AIConfig !== 'undefined' ? AIConfig.toRequest() : {}),
    type: 'thrust_analyze',
    firmware: firmware === 'px4' ? 'PX4' : 'ArduPilot',
    manual: Object.keys(tcOverrides).length > 0,
    values,
    rows: rows.map((r) => ({ name: r.name, value: r.value, level: r.level, text: r.text })),
    verdictText: window.ThrustCheck.verdict(rows, values),
    inputs: { note: document.getElementById('tcAiNote')?.value || '' },
  }));
}
document.getElementById('btnTcAi')?.addEventListener('click', tcAiAnalyze);

// ==================== 搜索和过滤 ====================
document.getElementById('paramSearch').addEventListener('input', renderParams);
document.getElementById('starFilter').addEventListener('change', renderParams);
document.getElementById('categoryFilter').addEventListener('change', (e) => {
  const v = e.target.value;
  if (v === 'all') { renderParams(); return; }
  scrollToCategory(v === '__full__' ? 'full' : parseInt(v, 10));
});

// 右侧滚动时同步左侧选中项
(function bindScrollSpy() {
  const panel = document.getElementById('paramPanel');
  let ticking = false;
  panel.addEventListener('scroll', () => {
    if (ticking) return;
    ticking = true;
    requestAnimationFrame(() => {
      ticking = false;
      syncActiveCategory();
    });
  });
})();

// ==================== 按钮事件 ====================
document.getElementById('btnConnect').onclick = connect;
document.getElementById('btnDisconnect').onclick = disconnect;
document.getElementById('btnReboot').onclick = rebootFC;
document.getElementById('btnRefreshParams').onclick = requestParams;
document.getElementById('btnRollbackAll').onclick = rollbackAllParams;
document.getElementById('btnBackup').onclick = () => switchView('backup');
document.getElementById('btnRestore').onclick = () => switchView('backup');

// 自动刷新串口列表
setInterval(refreshPorts, 3000);

// ==================== 视图切换 ====================
function switchView(name) {
  document.querySelectorAll('.view-tab').forEach((t) => {
    t.classList.toggle('active', t.dataset.view === name);
  });
  const map = {
    dashboard: 'viewDashboard',
    params: 'viewParams',
    thrust: 'viewThrust',
    sensors: 'viewSensors',
    onetouch: 'viewOnetouch',
    motortest: 'viewMotorTest',
    charts: 'viewCharts',
    console: 'viewConsole',
    backup: 'viewBackup',
    logs: 'viewLogs',
    fault: 'viewFault',
  };
  for (const [k, id] of Object.entries(map)) {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('hidden', k !== name);
  }
  if (name === 'backup' && typeof DataTools !== 'undefined') DataTools.handleMessage({ type: 'noop' });
  if (name === 'fault' && typeof Fault !== 'undefined') Fault.onShow();
  if (window.FcMonitor) FcMonitor.onView(name);   // 仪表盘重绘 / 曲线与消息台启停轮询
  if (window.SensorCheck) SensorCheck.onView(name); // 传感器体检: 进入重绘 / 离开停止采样
  if (window.MotorTest) MotorTest.onView(name);  // 电机测试: 进入刷新按钮状态
  if (window.TakeoffCheck) TakeoffCheck.onView(name); // 起飞就绪: 进入仪表盘时刷新判定
}
for (const t of document.querySelectorAll('.view-tab')) {
  t.onclick = () => switchView(t.dataset.view);
}

// ==================== 初始化 ====================
buildTcInputs();
renderThrustCheck();
renderCategories();
console.log('扶摇 FuyaoTune 前端已初始化');

// Web Serial 支持检测 (Firefox/Safari 不支持; 非安全上下文下 navigator.serial 也不存在)
(function webSerialCheck() {
  if (typeof FcSerial === 'undefined' || FcSerial.supported()) return;   // 正常情况不打扰
  const insecure = !window.isSecureContext;   // http://IP 访问时为 false
  document.getElementById('status').textContent = insecure ? '页面非 HTTPS, 串口不可用' : '浏览器不支持 Web Serial';
  document.getElementById('status').className = 'status-badge disconnected';
  document.getElementById('btnConnect').disabled = true;
  const banner = document.createElement('div');
  banner.id = 'webSerialBanner';
  banner.style.cssText = 'background:#7a1f1f;color:#ffd9d9;padding:10px 16px;font-size:13px;'
    + 'line-height:1.7;border-bottom:1px solid #a33;';
  banner.innerHTML = insecure
    ? '<b>⚠ 串口功能被浏览器禁用: 当前页面不是安全上下文</b>(正在通过 <code>http://IP</code> 访问, 浏览器隐藏了 Web Serial)。<br>'
      + '解决: ① 飞控插在本机时请改用 <b>http://localhost:3000</b> 打开本页; '
      + '② 局域网/远程访问请启用 HTTPS: <code>APM_DOMAIN=你的IP docker compose --profile tls up -d</code>, '
      + '首次访问在证书警告页点「继续前往(不安全)」后刷新本页。<br>'
      + '(WebSocket 已连接不受影响, 但串口直连必须安全上下文)'
    : '<b>⚠ 当前浏览器不支持 Web Serial</b>: 请改用 Chrome / Edge (101+) 打开本页面 (Firefox / Safari 不支持)。';
  document.body.insertBefore(banner, document.body.firstChild);
  console.warn(insecure
    ? '页面非安全上下文 (http://IP 访问): navigator.serial 不可用。请改用 localhost 访问, 或部署 HTTPS (docker compose --profile tls)'
    : '当前浏览器不支持 Web Serial, 请使用 Chrome / Edge');
})();