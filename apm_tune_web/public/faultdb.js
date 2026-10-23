// 警告速查知识库 — 由 scripts/build_faultdb.js 自动生成, 勿直接编辑
// 数据源: docs/(通用警告，多轴警告，固定翼警告).csv (制表 2026-03-16, Copter/Plane4.6.3) + docs/(PreArm补充-官方文档).csv (https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)
// 构建: node scripts/build_faultdb.js   共 260 条 (csv 249 / wiki 11), 剔除错误行 1 条, 去重 0 条
(function (g) {
  'use strict';
  const META = {
 "csvFile": "docs/(通用警告，多轴警告，固定翼警告).csv",
 "csvDate": "2026-03-16",
 "firmware": "Copter/Plane4.6.3",
 "note": "Ardupilot固件多轴与固定翼消息警告提示、产生原因、解决方法汇总表 / 制表于：20260316\n对应固件版本-Copter/Plane4.6.3",
 "wikiFile": "docs/(PreArm补充-官方文档).csv",
 "wikiUrl": "https://ardupilot.org/copter/docs/common-prearm-safety-checks.html",
 "builtAt": "2026-10-22",
 "counts": {
  "total": 260,
  "csv": 249,
  "wiki": 11
 }
};
  const CATS = ["通用警告","仅多轴或直升机的警告","仅固定翼的警告"];
  const CAT_SHORT = {
    '通用警告': '通用',
    '仅多轴或直升机的警告': '多轴·直升机',
    '仅固定翼的警告': '固定翼'
  };
  const ENTRIES = [
 {
  "cat": "通用警告",
  "en": "3D Accel calibration needed",
  "zh": "需进行 3D 加速度计校准",
  "cause": "尚未进行加速度计校准",
  "fix": "完成加速度计校准",
  "src": "csv",
  "id": 0
 },
 {
  "cat": "通用警告",
  "en": "Accels calibrated requires reboot",
  "zh": "加速度计已校准，需重启",
  "cause": "加速度计校准后需重启自动驾驶仪",
  "fix": "重启自动驾驶仪",
  "src": "csv",
  "id": 1
 },
 {
  "cat": "通用警告",
  "en": "Accels inconsistent",
  "zh": "加速度计数据不一致",
  "cause": "两个加速度计数据偏差 0.75 米 / 秒 ²",
  "fix": "重新进行加速度计校准；让自动驾驶仪预热后重启；\n若问题持续，更换自动驾驶仪；若内燃机启动后再解锁，可参考 ARMING_OPTIONS 参数第 2 位作为备选解决方案",
  "src": "csv",
  "id": 2
 },
 {
  "cat": "通用警告",
  "en": "ADSB out of memory",
  "zh": "ADSB 系统内存不足",
  "cause": "自动驾驶仪内存耗尽",
  "fix": "禁用部分功能，或更换性能更强的自动驾驶仪",
  "src": "csv",
  "id": 3
 },
 {
  "cat": "通用警告",
  "en": "Accels not healthy",
  "zh": "加速度计状态异常",
  "cause": "至少一个加速度计未输出数据",
  "fix": "重启自动驾驶仪；\n若问题持续，更换自动驾驶仪",
  "src": "csv",
  "id": 4
 },
 {
  "cat": "通用警告",
  "en": "AHRS: not using configured AHRS type",
  "zh": "AHRS：未使用配置的姿态参考系统类型",
  "cause": "扩展卡尔曼滤波器 3（EKF3）未就绪，设备当前使用 DCM（方向余弦矩阵）",
  "fix": "若在室内，移至室外；确保 GPS 信号良好；检查 EKF 配置是否错误（参考 AHRS_EKF_TYPE 参数）",
  "src": "csv",
  "id": 5
 },
 {
  "cat": "通用警告",
  "en": "AHRS: waiting for home",
  "zh": "AHRS：等待 Home 点（返航点）设置",
  "cause": "GPS 未完成定位（未获取固定解）",
  "fix": "若在室内，移至室外；确保指南针和加速度计已完成校准；排除可能干扰 GPS 的射频源",
  "src": "csv",
  "id": 6
 },
 {
  "cat": "通用警告",
  "en": "Airspeed 1 not healthy",
  "zh": "空速传感器 1 状态异常",
  "cause": "自动驾驶仪无法从传感器获取数据",
  "fix": "检查传感器物理连接及配置",
  "src": "csv",
  "id": 7
 },
 {
  "cat": "通用警告",
  "en": "AP_Relay not available",
  "zh": "AP_Relay（自动驾驶仪继电器）功能不可用",
  "cause": "降落伞通过继电器控制，但继电器功能缺失（可能使用了自定义编译服务器，需重新编译并启用继电器功能）",
  "fix": "重新编译固件并启用继电器功能",
  "src": "csv",
  "id": 8
 },
 {
  "cat": "通用警告",
  "en": "Auxiliary authorisation refused",
  "zh": "辅助授权被拒绝",
  "cause": "外部系统拒绝授权",
  "fix": "检查外部授权系统",
  "src": "csv",
  "id": 9
 },
 {
  "cat": "通用警告",
  "en": "Batch sampling requires reboot",
  "zh": "批量采样需重启",
  "cause": "批量采样功能需重启自动驾驶仪才能生效",
  "fix": "重启自动驾驶仪，或检查批量采样配置",
  "src": "csv",
  "id": 10
 },
 {
  "cat": "通用警告",
  "en": "Battery below minimum arming capacity",
  "zh": "电池容量低于解锁最低要求",
  "cause": "电池容量低于 BATT_ARM_MAH（电池解锁最小容量）参数值",
  "fix": "更换电池，或调整 BATT_ARM_MAH 参数。\nBATT_ARM_MAH参数默认=0！\n用户设置后，解锁时飞控就会检查当前的电量是否比该参数的值要低，低的话飞控就会拒绝解锁！",
  "src": "csv",
  "id": 11
 },
 {
  "cat": "通用警告",
  "en": "Battery below minimum arming voltage",
  "zh": "电池电压低于解锁最低要求",
  "cause": "电池电压低于 BATT_ARM_VOLT（电池解锁最低电压）参数值",
  "fix": "更换电池，或调整 BATT_ARM_VOLT 参数\nBATT_ARM_VOLT 参数默认=0！\n用户设置后，解锁时飞控就会检查当前的电量是否比该参数的值要低，低的话飞控就会拒绝解锁！",
  "src": "csv",
  "id": 12
 },
 {
  "cat": "通用警告",
  "en": "Battery capacity failsafe critical >= low",
  "zh": "电池容量失效保护：紧急阈值≥低阈值",
  "cause": "电池容量失效保护参数配置错误",
  "fix": "确保 BATT_LOW_MAH（电池低容量阈值）高于 BATT_CRT_MAH（电池紧急容量阈值）",
  "src": "csv",
  "id": 13
 },
 {
  "cat": "通用警告",
  "en": "Battery critical capacity failsafe",
  "zh": "电池容量紧急失效保护触发",
  "cause": "电池容量低于 BATT_CRT_MAH（电池紧急容量阈值）参数值",
  "fix": "更换电池，或调整 BATT_CRT_MAH 参数",
  "src": "csv",
  "id": 14
 },
 {
  "cat": "通用警告",
  "en": "Battery critical voltage failsafe",
  "zh": "电池电压紧急失效保护触发",
  "cause": "电池电压低于 BATT_CRT_VOLT（电池紧急电压阈值）参数值",
  "fix": "更换电池，或调整 BATT_CRT_VOLT 参数",
  "src": "csv",
  "id": 15
 },
 {
  "cat": "通用警告",
  "en": "Battery low capacity failsafe",
  "zh": "电池容量低失效保护触发",
  "cause": "电池容量低于 BATT_LOW_MAH（电池低容量阈值）参数值",
  "fix": "更换电池，或调整 BATT_LOW_MAH 参数",
  "src": "csv",
  "id": 16
 },
 {
  "cat": "通用警告",
  "en": "Battery low voltage failsafe",
  "zh": "电池电压低失效保护触发",
  "cause": "电池电压低于 BATT_LOW_VOLT（电池低电压阈值）参数值",
  "fix": "更换电池，或调整 BATT_LOW_VOLT 参数",
  "src": "csv",
  "id": 17
 },
 {
  "cat": "通用警告",
  "en": "Battery unhealthy",
  "zh": "电池状态异常",
  "cause": "电池未输出数据（无法检测电池状态）",
  "fix": "检查电池监测器的物理连接及配置",
  "src": "csv",
  "id": 18
 },
 {
  "cat": "通用警告",
  "en": "Battery voltage failsafe critical >= low",
  "zh": "电池电压失效保护：紧急阈值≥低阈值",
  "cause": "电池电压失效保护参数配置错误",
  "fix": "确保 BATT_LOW_VOLT（电池低电压阈值）高于 BATT_CRT_VOLT（电池紧急电压阈值）",
  "src": "csv",
  "id": 19
 },
 {
  "cat": "通用警告",
  "en": "Baro not healthy",
  "zh": "气压计不健康",
  "cause": "气压计传感器报告其不健康，这通常是硬件故障的迹象。",
  "fix": "重启自动驾驶仪；\n若问题持续，更换自动驾驶仪",
  "src": "csv",
  "id": 20
 },
 {
  "cat": "通用警告",
  "en": "BendyRuler OA requires reboot",
  "zh": "BendyRuler 避障需重启",
  "cause": "避障（OA）配置修改后需重启才能生效",
  "fix": "重启自动驾驶仪；参考避障配置说明",
  "src": "csv",
  "id": 21
 },
 {
  "cat": "通用警告",
  "en": "Board (Xv) out of range 4.3-5.8v",
  "zh": "主板电压（Xv）超出 4.3-5.8V 范围",
  "cause": "主板电压低于 BRD_VBUS_MIN（主板最小供电电压）或过高",
  "fix": "检查电源；若通过 USB 供电，需接入电池或更换 USB 线",
  "src": "csv",
  "id": 22
 },
 {
  "cat": "通用警告",
  "en": "BTN_PINx=y invalid",
  "zh": "按钮引脚 BTN_PINx=y 配置无效",
  "cause": "按钮配置错误",
  "fix": "BTN_PINx 参数被设为无效值；参考按钮设置说明",
  "src": "csv",
  "id": 23
 },
 {
  "cat": "通用警告",
  "en": "BTN_PINx=y, set SERVOz_FUNCTION=-1",
  "zh": "按钮引脚 BTN_PINx=y 需设置 SERVOz_FUNCTION=-1",
  "cause": "按钮配置错误",
  "fix": "将 SERVOz_FUNCTION（伺服通道功能）参数设为 - 1（禁用该通道功能）",
  "src": "csv",
  "id": 24
 },
 {
  "cat": "通用警告",
  "en": "Can't check rally without position",
  "zh": "无位置信息，无法检查拉力点",
  "cause": "扩展卡尔曼滤波器（EKF）尚未获取位置估算值",
  "fix": "等待，或移至 GPS 信号更好的位置",
  "src": "csv",
  "id": 25
 },
 {
  "cat": "通用警告",
  "en": "Check fence",
  "zh": "请检查地理围栏",
  "cause": "地理围栏功能初始化失败",
  "fix": "重启自动驾驶仪",
  "src": "csv",
  "id": 26
 },
 {
  "cat": "通用警告",
  "en": "Check mag field (xy diff:x>875)",
  "zh": "检查磁场（XY 轴偏差：x>875）",
  "cause": "指南针水平磁场强度过大或过小",
  "fix": "将设备移至远离环境中金属物体的位置；将指南针移至远离机架金属部件处；重新校准指南针；禁用内置指南针",
  "src": "csv",
  "id": 27
 },
 {
  "cat": "通用警告",
  "en": "Check mag field (z diff:x>875)",
  "zh": "检查磁场（Z 轴偏差：x>875）",
  "cause": "指南针垂直磁场强度过大或过小",
  "fix": "将设备移至远离环境中金属物体的位置；将指南针移至远离机架金属部件处；重新校准指南针；禁用内置指南针",
  "src": "csv",
  "id": 28
 },
 {
  "cat": "通用警告",
  "en": "Check mag field: x, max y, min z",
  "zh": "检查磁场：x（当前值）、y（最大值）、z（最小值）",
  "cause": "指南针磁场强度过大或过小",
  "fix": "将设备移至远离环境中金属物体的位置；将指南针移至远离机架金属部件处；重新校准指南针；禁用内置指南针",
  "src": "csv",
  "id": 29
 },
 {
  "cat": "通用警告",
  "en": "Chute has no channel",
  "zh": "降落伞无控制通道",
  "cause": "降落伞配置错误",
  "fix": "降落伞通过 PWM（脉冲宽度调制）控制，但未配置伺服通道功能（例：需将 SERVOx_FUNCTION 设为 27）；参考降落伞设置说明",
  "src": "csv",
  "id": 30
 },
 {
  "cat": "通用警告",
  "en": "Chute has no relay",
  "zh": "降落伞无继电器控制",
  "cause": "降落伞配置错误",
  "fix": "降落伞通过继电器控制，但未配置继电器输出；参考降落伞设置说明",
  "src": "csv",
  "id": 31
 },
 {
  "cat": "通用警告",
  "en": "Chute is released",
  "zh": "降落伞已释放",
  "cause": "降落伞已触发释放",
  "fix": "重启自动驾驶仪",
  "src": "csv",
  "id": 32
 },
 {
  "cat": "通用警告",
  "en": "Compass calibrated requires reboot",
  "zh": "指南针已校准，需重启",
  "cause": "指南针校准后需重启自动驾驶仪",
  "fix": "重启自动驾驶仪",
  "src": "csv",
  "id": 33
 },
 {
  "cat": "通用警告",
  "en": "Compass calibration running",
  "zh": "指南针校准中",
  "cause": "指南针校准流程正在进行",
  "fix": "完成或取消指南针校准流程",
  "src": "csv",
  "id": 34
 },
 {
  "cat": "通用警告",
  "en": "Compass not healthy",
  "zh": "指南针状态异常",
  "cause": "至少一个指南针未输出数据。\n指南针传感器报告其不正常，这是硬件故障的迹象。",
  "fix": "检查指南针与自动驾驶仪的连接及配置",
  "src": "csv",
  "id": 35
 },
 {
  "cat": "通用警告",
  "en": "Compass offsets too high",
  "zh": "指南针偏移量过高",
  "cause": "指南针偏移参数值过大",
  "fix": "将指南针移至远离机架金属部件处；重新校准指南针；禁用内置指南针；增大 COMPASS_OFFS_MAX（指南针最大偏移量）参数",
  "src": "csv",
  "id": 36
 },
 {
  "cat": "通用警告",
  "en": "Compasses inconsistent",
  "zh": "指南针数据不一致",
  "cause": "两个指南针的角度或磁场强度数据不匹配",
  "fix": "检查指南针安装方向（参考 COMPASS_ORIENT 参数）；将指南针移至远离机架金属部件处；重新校准指南针；禁用内置指南针",
  "src": "csv",
  "id": 37
 },
 {
  "cat": "通用警告",
  "en": "CrashDump data detected",
  "zh": "检测到崩溃日志（CrashDump）",
  "cause": "已记录崩溃日志数据（表明 CPU 曾崩溃）",
  "fix": "CPU 曾发生崩溃并记录数据，设备可能无法安全飞行！参考 “看门狗 / 崩溃日志上报” 说明",
  "src": "csv",
  "id": 38
 },
 {
  "cat": "通用警告",
  "en": "Dijkstra OA requires reboot",
  "zh": "Dijkstra 避障需重启",
  "cause": "避障（OA）配置修改后需重启才能生效",
  "fix": "重启自动驾驶仪；参考避障配置说明",
  "src": "csv",
  "id": 39
 },
 {
  "cat": "通用警告",
  "en": "Disarm Switch on",
  "zh": "解锁开关处于开启状态",
  "cause": "解锁开关处于高电平位置（未关闭）",
  "fix": "将解锁开关拨至低电平位置，或检查辅助功能配置",
  "src": "csv",
  "id": 40
 },
 {
  "cat": "通用警告",
  "en": "Downloading logs",
  "zh": "日志下载中",
  "cause": "日志下载期间无法解锁设备",
  "fix": "等待日志下载完成，或取消下载，或重启自动驾驶仪",
  "src": "csv",
  "id": 41
 },
 {
  "cat": "通用警告",
  "en": "DroneCAN: Duplicate Node x../y!",
  "zh": "DroneCAN：节点 x../y 重复",
  "cause": "DroneCAN 检测到两个设备使用相同节点 ID",
  "fix": "设置 CAN_D1_UC_OPTION=1 以清除 DroneCAN DNS 服务器，然后重启",
  "src": "csv",
  "id": 42
 },
 {
  "cat": "通用警告",
  "en": "DroneCAN: Failed to access storage!",
  "zh": "DroneCAN：存储访问失败",
  "cause": "可能存在硬件问题",
  "fix": "重启自动驾驶仪",
  "src": "csv",
  "id": 43
 },
 {
  "cat": "通用警告",
  "en": "DroneCAN: Failed to add Node x!",
  "zh": "DroneCAN：无法添加节点 x",
  "cause": "DroneCAN 无法初始化与某设备的连接",
  "fix": "检查传感器的物理连接及供电",
  "src": "csv",
  "id": 44
 },
 {
  "cat": "通用警告",
  "en": "DroneCAN: Node x unhealthy!",
  "zh": "DroneCAN：节点 x 状态异常",
  "cause": "某 DroneCAN 设备未输出数据",
  "fix": "检查传感器的物理连接及供电",
  "src": "csv",
  "id": 45
 },
 {
  "cat": "通用警告",
  "en": "Duplicate Aux Switch Options",
  "zh": "辅助开关功能重复",
  "cause": "两个辅助开关配置了相同功能",
  "fix": "检查辅助功能配置；查看是否有 RCx_OPTION（遥控器通道功能）参数值重复",
  "src": "csv",
  "id": 46
 },
 {
  "cat": "通用警告",
  "en": "EKF3 Roll/Pitch inconsistent by x degs",
  "zh": "EKF3：横滚 / 俯仰偏差 x 度",
  "cause": "横滚或俯仰倾斜角估算值不一致",
  "fix": "通常因 EKF3 未获取足够精度的 GPS 数据，也可能因其他传感器出错；移至室外，或等待，或重启自动驾驶仪",
  "src": "csv",
  "id": 47
 },
 {
  "cat": "通用警告",
  "en": "EKF3x vel error y",
  "zh": "EKF3x：速度误差 y",
  "cause": "EKF3 的速度新息（innovation）为 “y”（速度估算偏差）",
  "fix": "EKF3 位置速度新息过高；检查 GPS，或等待，或重启",
  "src": "csv",
  "id": 48
 },
 {
  "cat": "通用警告",
  "en": "EKF3 waiting for GPS config data",
  "zh": "EKF3：等待 GPS 配置数据",
  "cause": "GPS 自动配置未完成",
  "fix": "检查 GPS 连接及配置（使用 DroneCAN GPS 时需特别检查）",
  "src": "csv",
  "id": 49
 },
 {
  "cat": "通用警告",
  "en": "EKF3 Yaw inconsistent by x degs",
  "zh": "EKF3：偏航偏差 x 度",
  "cause": "偏航角估算值不一致",
  "fix": "等待，或重启自动驾驶仪",
  "src": "csv",
  "id": 50
 },
 {
  "cat": "通用警告",
  "en": "Failed to open mission.stg",
  "zh": "无法打开 mission.stg（任务文件）",
  "cause": "无法从 SD 卡加载任务",
  "fix": "检查 SD 卡；尝试重新将任务保存到 SD 卡",
  "src": "csv",
  "id": 51
 },
 {
  "cat": "通用警告",
  "en": "Failed to create log directory /APM/LOGS: ENOMEM",
  "zh": "无法创建日志目录 / APM/LOGS：内存不足",
  "cause": "无法在 SD 卡创建日志目录（内存不足）",
  "fix": "减小 LOG_FILE_BUFSIZE（日志文件缓冲区大小）参数，或禁用其他功能以降低内存占用",
  "src": "csv",
  "id": 52
 },
 {
  "cat": "通用警告",
  "en": "Fence requires position",
  "zh": "地理围栏需位置信息",
  "cause": "若启用地理围栏，需获取位置估算值",
  "fix": "等待，或移至天空视野开阔的位置；减少射频干扰源",
  "src": "csv",
  "id": 53
 },
 {
  "cat": "通用警告",
  "en": "FENCE_ALT_MAX < FENCE_ALT_MIN",
  "zh": "地理围栏最大高度 < 最小高度",
  "cause": "FENCE_ALT_MAX（围栏最大高度）必须大于 FENCE_ALT_MIN（围栏最小高度）",
  "fix": "增大 FENCE_ALT_MAX，或降低 FENCE_ALT_MIN",
  "src": "csv",
  "id": 54
 },
 {
  "cat": "通用警告",
  "en": "FENCE_MARGIN is less than FENCE_RADIUS",
  "zh": "地理围栏边际 < 半径",
  "cause": "FENCE_MARGIN（围栏边际）必须大于 FENCE_RADIUS（围栏半径）",
  "fix": "增大 FENCE_RADIUS，或减小 FENCE_MARGIN",
  "src": "csv",
  "id": 55
 },
 {
  "cat": "通用警告",
  "en": "FENCE_MARGIN too big",
  "zh": "地理围栏边际过大",
  "cause": "围栏最大高度与最小高度之差 < 2 倍围栏边际",
  "fix": "减小 FENCE_MARGIN，或增大 FENCE_ALT_MAX 与 FENCE_ALT_MIN 的差值",
  "src": "csv",
  "id": 56
 },
 {
  "cat": "通用警告",
  "en": "Fences enabled, but none selected",
  "zh": "地理围栏已启用，但未选择围栏类型",
  "cause": "地理围栏已启用，但未定义任何围栏类型",
  "fix": "通过 FENCE_ENABLE（围栏启用）或 FENCE_TYPE（围栏类型）禁用部分 / 全部围栏，或定义缺失的围栏",
  "src": "csv",
  "id": 57
 },
 {
  "cat": "通用警告",
  "en": "Fences invalid",
  "zh": "地理围栏无效",
  "cause": "多边形围栏配置无效",
  "fix": "检查多边形围栏是否存在线条重叠",
  "src": "csv",
  "id": 58
 },
 {
  "cat": "通用警告",
  "en": "FETtec: Invalid motor mask",
  "zh": "FETtec：电机掩码无效",
  "cause": "FETtec（电调品牌）配置错误",
  "fix": "参考 FETtec 配置说明",
  "src": "csv",
  "id": 59
 },
 {
  "cat": "通用警告",
  "en": "FETtec: Invalid pole count x",
  "zh": "FETtec：磁极数 x 无效",
  "cause": "FETtec 配置错误",
  "fix": "参考 FETtec 配置说明",
  "src": "csv",
  "id": 60
 },
 {
  "cat": "通用警告",
  "en": "FETtec: No uart",
  "zh": "FETtec：无 UART（串行通信）",
  "cause": "FETtec 配置错误",
  "fix": "参考 FETtec 配置说明",
  "src": "csv",
  "id": 61
 },
 {
  "cat": "通用警告",
  "en": "FETtec: Not initialised",
  "zh": "FETtec：未初始化",
  "cause": "FETtec 电调未与自动驾驶仪通信",
  "fix": "参考 FETtec 配置说明",
  "src": "csv",
  "id": 62
 },
 {
  "cat": "通用警告",
  "en": "FETtec: x of y ESCs are not running",
  "zh": "FETtec：y 个电调中有 x 个未运行",
  "cause": "FETtec 电调未旋转（未工作）",
  "fix": "参考 FETtec 配置说明",
  "src": "csv",
  "id": 63
 },
 {
  "cat": "通用警告",
  "en": "FETtec: x of y ESCs are not sending telem",
  "zh": "FETtec：y 个电调中有 x 个未发送遥测",
  "cause": "FETtec 电调未与自动驾驶仪通信",
  "fix": "参考 FETtec 配置说明",
  "src": "csv",
  "id": 64
 },
 {
  "cat": "通用警告",
  "en": "FFT calibrating noise",
  "zh": "FFT（快速傅里叶变换）噪声校准中",
  "cause": "FFT 谐波陷波分析未完成",
  "fix": "等待飞行中 FFT 分析完成",
  "src": "csv",
  "id": 65
 },
 {
  "cat": "通用警告",
  "en": "FFT config MAXHZ xHz > yHz",
  "zh": "FFT 配置：最大频率 xHz>yHz",
  "cause": "FFT 谐波陷波配置错误",
  "fix": "参考 “飞行中 FFT 谐波陷波设置” 说明",
  "src": "csv",
  "id": 66
 },
 {
  "cat": "通用警告",
  "en": "FFT self-test failed, max error Hz",
  "zh": "FFT 自检失败，最大误差 Hz",
  "cause": "FFT 谐波陷波功能失败",
  "fix": "参考 “飞行中 FFT 谐波陷波设置” 说明",
  "src": "csv",
  "id": 67
 },
 {
  "cat": "通用警告",
  "en": "FFT still analyzing",
  "zh": "FFT 仍在分析中",
  "cause": "FFT 谐波陷波分析未完成",
  "fix": "等待飞行中 FFT 分析完成",
  "src": "csv",
  "id": 68
 },
 {
  "cat": "通用警告",
  "en": "FFT: calibrated xHz/xHz/xHz",
  "zh": "FFT：已校准 xHz/xHz/xHz",
  "cause": "FFT 谐波陷波相关问题",
  "fix": "参考 “飞行中 FFT 谐波陷波设置” 说明",
  "src": "csv",
  "id": 69
 },
 {
  "cat": "通用警告",
  "en": "FFT: resolution is xHz, increase length",
  "zh": "FFT：分辨率 xHz，需增加长度",
  "cause": "FFT 谐波陷波配置错误",
  "fix": "参考 “飞行中 FFT 谐波陷波设置” 说明",
  "src": "csv",
  "id": 70
 },
 {
  "cat": "通用警告",
  "en": "Generator: Not healthy",
  "zh": "发电机状态异常",
  "cause": "发电机未与自动驾驶仪通信",
  "fix": "检查发电机配置",
  "src": "csv",
  "id": 71
 },
 {
  "cat": "通用警告",
  "en": "Generator: No backend driver",
  "zh": "发电机：无后端驱动",
  "cause": "固件未包含所选发电机的驱动",
  "fix": "通过custom.ardupilot.org（自定义编译网站）编译包含所需发电机驱动的固件",
  "src": "csv",
  "id": 72
 },
 {
  "cat": "通用警告",
  "en": "GPS alt error xm (see BARO_ALTERR_MAX)",
  "zh": "GPS 高度误差 xm（参考 BARO_ALTERR_MAX）",
  "cause": "GPS 与气压计（BARO）高度偏差过大",
  "fix": "查看 BARO_ALTERR_MAX（GPS 与气压计高度最大偏差）参数说明",
  "src": "csv",
  "id": 73
 },
 {
  "cat": "通用警告",
  "en": "GPS and AHRS differ by Xm",
  "zh": "GPS 与 AHRS 位置偏差 X 米",
  "cause": "GPS 与 EKF 位置偏差至少 10 米",
  "fix": "等待 GPS 信号质量提升；移至天空视野开阔的位置；减少射频干扰源",
  "src": "csv",
  "id": 74
 },
 {
  "cat": "通用警告",
  "en": "GPS blending unhealthy",
  "zh": "GPS 融合状态异常",
  "cause": "至少一个 GPS 未提供有效数据",
  "fix": "移至天空视野开阔的位置；减少射频干扰源；检查 GPS 融合配置",
  "src": "csv",
  "id": 75
 },
 {
  "cat": "通用警告",
  "en": "GPS Node x not set as instance y",
  "zh": "GPS 节点 x 未设为实例 y",
  "cause": "DroneCAN GPS 配置错误",
  "fix": "检查 GPS1_CAN_NODEID（GPS1 节点 ID）和 GPS2_CAN_NODEID（GPS2 节点 ID）参数",
  "src": "csv",
  "id": 76
 },
 {
  "cat": "通用警告",
  "en": "GPS positions differ by Xm",
  "zh": "GPS 位置偏差 X 米",
  "cause": "两个 GPS 报告的位置偏差≥50 米",
  "fix": "等待 GPS 信号质量提升；移至天空视野开阔的位置；减少射频干扰源",
  "src": "csv",
  "id": 77
 },
 {
  "cat": "通用警告",
  "en": "GPS x still configuring this GPS",
  "zh": "GPS x：仍在配置中",
  "cause": "GPS 自动配置未完成",
  "fix": "等待配置完成；检查 GPS 连接及配置（使用 DroneCAN GPS 时需特别检查）",
  "src": "csv",
  "id": 78
 },
 {
  "cat": "通用警告",
  "en": "GPS x: Bad fix",
  "zh": "GPS x：定位质量差",
  "cause": "GPS 未获取良好定位（无固定解）",
  "fix": "移至天空视野开阔的位置；减少射频干扰源",
  "src": "csv",
  "id": 79
 },
 {
  "cat": "通用警告",
  "en": "GPS x: not healthy",
  "zh": "GPS x 状态异常",
  "cause": "GPS 未输出数据",
  "fix": "检查 GPS 与自动驾驶仪的物理连接及配置",
  "src": "csv",
  "id": 80
 },
 {
  "cat": "通用警告",
  "en": "GPS x: primary but TYPE 0",
  "zh": "GPS x：为主 GPS 但类型为 0",
  "cause": "主 GPS 未配置类型",
  "fix": "检查 GPS_PRIMARY（主 GPS 选择）参数，确保对应 GPS1_TYPE 或 GPS2_TYPE（GPS 类型）与所用 GPS 匹配",
  "src": "csv",
  "id": 81
 },
 {
  "cat": "通用警告",
  "en": "GPS x: was not found",
  "zh": "GPS x：未找到",
  "cause": "GPS 断开连接或配置错误",
  "fix": "检查 GPS 与自动驾驶仪的物理连接及配置",
  "src": "csv",
  "id": 82
 },
 {
  "cat": "通用警告",
  "en": "GPSx yaw not available",
  "zh": "GPSx：偏航信息不可用",
  "cause": "已配置 GPS 偏航功能但未生效",
  "fix": "移至 GPS 信号更好的位置；检查 GPS 偏航配置",
  "src": "csv",
  "id": 83
 },
 {
  "cat": "通用警告",
  "en": "Gyro x rate yHz < loop rate z Hz",
  "zh": "陀螺仪 x 速率 yHz < 循环速率 zHz",
  "cause": "循环速率快于陀螺仪数据输出速率",
  "fix": "降低 SCHED_LOOP_RATE（调度循环速率）参数",
  "src": "csv",
  "id": 84
 },
 {
  "cat": "通用警告",
  "en": "Gyros inconsistent",
  "zh": "陀螺仪数据不一致",
  "cause": "两个陀螺仪数据偏差≥5 度 / 秒",
  "fix": "重启自动驾驶仪并保持设备静止，直至陀螺仪校准完成；让自动驾驶仪预热后重启；若问题持续，更换自动驾驶仪；若内燃机启动后再解锁，可参考 ARMING_OPTIONS 参数第 2 位作为备选解决方案",
  "src": "csv",
  "id": 85
 },
 {
  "cat": "通用警告",
  "en": "Gyros not calibrated",
  "zh": "陀螺仪未校准",
  "cause": "启动时默认执行的陀螺仪校准失败",
  "fix": "重启并保持设备静止，直至陀螺仪校准完成",
  "src": "csv",
  "id": 86
 },
 {
  "cat": "通用警告",
  "en": "Gyros not healthy",
  "zh": "陀螺仪状态异常",
  "cause": "至少一个陀螺仪未输出数据",
  "fix": "重启自动驾驶仪；若问题持续，更换自动驾驶仪",
  "src": "csv",
  "id": 87
 },
 {
  "cat": "通用警告",
  "en": "Hardware safety switch",
  "zh": "硬件安全开关未触发",
  "cause": "硬件安全开关（通常在 GPS 顶部）未按下",
  "fix": "按下硬件安全开关（通常在 GPS 顶部），或设置 BRD_SAFETY_DEFLT=0（禁用硬件安全开关）并重启自动驾驶仪",
  "src": "csv",
  "id": 88
 },
 {
  "cat": "通用警告",
  "en": "heater temp low (x < 45)",
  "zh": "加热器温度低（x < 45℃）",
  "cause": "主板加热器温度低于 BRD_HEAT_TARG（主板加热目标温度）",
  "fix": "等待主板加热至目标温度；可通过 BRD_HEAT_TARG 调整目标温度",
  "src": "csv",
  "id": 89
 },
 {
  "cat": "通用警告",
  "en": "In OSD menu",
  "zh": "处于 OSD（屏幕显示）菜单中",
  "cause": "正在配置 OSD",
  "fix": "完成 OSD 配置；检查 OSD 配置",
  "src": "csv",
  "id": 90
 },
 {
  "cat": "通用警告",
  "en": "Internal errors 0x%x l:%u %s",
  "zh": "内部错误 0x% x l:% u % s",
  "cause": "发生内部错误",
  "fix": "重启自动驾驶仪；向开发团队上报错误信息",
  "src": "csv",
  "id": 91
 },
 {
  "cat": "通用警告",
  "en": "Invalid FENCE_ALT_MAX value",
  "zh": "FENCE_ALT_MAX 值无效",
  "cause": "FENCE_ALT_MAX 必须为正值",
  "fix": "增大 FENCE_ALT_MAX 参数",
  "src": "csv",
  "id": 92
 },
 {
  "cat": "通用警告",
  "en": "Invalid FENCE_ALT_MIN value",
  "zh": "FENCE_ALT_MIN 值无效",
  "cause": "FENCE_ALT_MIN 必须大于 - 100",
  "fix": "增大 FENCE_ALT_MIN 参数",
  "src": "csv",
  "id": 93
 },
 {
  "cat": "通用警告",
  "en": "Invalid FENCE_MARGIN value",
  "zh": "FENCE_MARGIN 值无效",
  "cause": "FENCE_MARGIN 必须为正值",
  "fix": "增大 FENCE_MARGIN 参数",
  "src": "csv",
  "id": 94
 },
 {
  "cat": "通用警告",
  "en": "Invalid FENCE_RADIUS value",
  "zh": "FENCE_RADIUS 值无效",
  "cause": "FENCE_RADIUS 必须为正值",
  "fix": "增大 FENCE_RADIUS 参数",
  "src": "csv",
  "id": 95
 },
 {
  "cat": "通用警告",
  "en": "Logging failed",
  "zh": "日志记录失败",
  "cause": "无法写入日志，可能为硬件故障",
  "fix": "重启自动驾驶仪；更换自动驾驶仪",
  "src": "csv",
  "id": 96
 },
 {
  "cat": "通用警告",
  "en": "Logging not started",
  "zh": "日志记录未启动",
  "cause": "无法写入日志，可能为硬件故障",
  "fix": "重启自动驾驶仪；更换自动驾驶仪",
  "src": "csv",
  "id": 97
 },
 {
  "cat": "通用警告",
  "en": "Main loop slow (xHz < 400Hz)",
  "zh": "主循环缓慢（xHz < 400Hz）",
  "cause": "自动驾驶仪 CPU 过载",
  "fix": "观察错误是否为暂时性；禁用部分功能或更换性能更强的自动驾驶仪；降低 SCHED_LOOP_RATE 参数",
  "src": "csv",
  "id": 98
 },
 {
  "cat": "通用警告",
  "en": "Margin is less than inclusion circle radius",
  "zh": "边际小于包含圆半径",
  "cause": "圆形围栏半径小于 FENCE_MARGIN（围栏边际）",
  "fix": "增大相关圆形围栏的半径，或减小 FENCE_MARGIN 参数",
  "src": "csv",
  "id": 99
 },
 {
  "cat": "通用警告",
  "en": "memory low for auxiliary authorisation",
  "zh": "辅助授权内存不足",
  "cause": "自动驾驶仪内存耗尽",
  "fix": "禁用部分功能，或更换性能更强的自动驾驶仪",
  "src": "csv",
  "id": 100
 },
 {
  "cat": "通用警告",
  "en": "Missing mission item: do land start",
  "zh": "缺失任务项：DO_LAND_START（着陆启动）",
  "cause": "自动任务需包含 DO_LAND_START 指令",
  "fix": "在任务中添加 DO_LAND_START 指令，或调整 ARMING_MIS_ITEMS（解锁任务项要求）参数",
  "src": "csv",
  "id": 101
 },
 {
  "cat": "通用警告",
  "en": "Missing mission item: land",
  "zh": "缺失任务项：LAND（着陆）",
  "cause": "自动任务需包含 LAND 指令",
  "fix": "在任务中添加 LAND 指令，或调整 ARMING_MIS_ITEMS 参数",
  "src": "csv",
  "id": 102
 },
 {
  "cat": "通用警告",
  "en": "Missing mission item: RTL",
  "zh": "缺失任务项：RTL（返航）",
  "cause": "自动任务需包含 RTL 指令",
  "fix": "在任务中添加 RTL 指令，或调整 ARMING_MIS_ITEMS 参数",
  "src": "csv",
  "id": 103
 },
 {
  "cat": "通用警告",
  "en": "Missing mission item: takeoff",
  "zh": "缺失任务项：TAKEOFF（起飞）",
  "cause": "自动任务需包含 TAKEOFF 指令",
  "fix": "在任务中添加 TAKEOFF 指令，或调整 ARMING_MIS_ITEMS 参数",
  "src": "csv",
  "id": 104
 },
 {
  "cat": "通用警告",
  "en": "Missing mission item: vtol land",
  "zh": "缺失任务项：VTOL_LAND（垂直起降着陆）",
  "cause": "自动任务需包含 VTOL_LAND 指令",
  "fix": "在任务中添加 VTOL_LAND 指令，或调整 ARMING_MIS_ITEMS 参数",
  "src": "csv",
  "id": 105
 },
 {
  "cat": "通用警告",
  "en": "Missing mission item: vtol takeoff",
  "zh": "缺失任务项：VTOL_TAKEOFF（垂直起降起飞）",
  "cause": "自动任务需包含 VTOL_TAKEOFF 指令",
  "fix": "在任务中添加 VTOL_TAKEOFF 指令，或调整 ARMING_MIS_ITEMS 参数",
  "src": "csv",
  "id": 106
 },
 {
  "cat": "通用警告",
  "en": "Mode channel and RCx_OPTION conflict",
  "zh": "模式通道与 RCx_OPTION 冲突",
  "cause": "遥控器飞行模式开关同时被用作辅助功能",
  "fix": "修改 FLTMODE_CH（飞行模式通道， Rover 车型为 MODE_CH）或 RCx_OPTION 参数以消除冲突",
  "src": "csv",
  "id": 107
 },
 {
  "cat": "通用警告",
  "en": "Mode requires mission",
  "zh": "该模式需任务支持",
  "cause": "尝试在自动（Auto）模式下解锁，但无任务",
  "fix": "在其他模式下解锁，或创建并上传自动任务",
  "src": "csv",
  "id": 108
 },
 {
  "cat": "通用警告",
  "en": "Motors Emergency Stopped",
  "zh": "电机紧急停止",
  "cause": "电机已触发紧急停止",
  "fix": "解除紧急停止；参考辅助功能说明",
  "src": "csv",
  "id": 109
 },
 {
  "cat": "通用警告",
  "en": "Mount: check TYPE",
  "zh": "云台：检查类型",
  "cause": "云台（相机云台）配置错误",
  "fix": "检查 MNT1_TYPE（云台 1 类型）是否有效；参考云台配置说明",
  "src": "csv",
  "id": 110
 },
 {
  "cat": "通用警告",
  "en": "Mount: not healthy",
  "zh": "云台状态异常",
  "cause": "云台未与自动驾驶仪通信",
  "fix": "检查自动驾驶仪与云台的物理连接；参考云台配置说明",
  "src": "csv",
  "id": 111
 },
 {
  "cat": "通用警告",
  "en": "Multiple SERIAL ports configured for RC input",
  "zh": "多个串口配置为遥控器输入",
  "cause": "遥控器配置错误",
  "fix": "参考 “多遥控器接收器” 说明",
  "src": "csv",
  "id": 112
 },
 {
  "cat": "通用警告",
  "en": "No mission library present",
  "zh": "无任务库",
  "cause": "自动任务功能已被禁用",
  "fix": "固件可能通过自定义编译服务器生成（未包含自动任务功能）；重新编译并启用自动任务",
  "src": "csv",
  "id": 113
 },
 {
  "cat": "通用警告",
  "en": "No rally library present",
  "zh": "无拉力点库",
  "cause": "拉力点功能已被禁用",
  "fix": "固件可能通过自定义编译服务器生成（未包含拉力点功能）；重新编译并包含拉力点功能",
  "src": "csv",
  "id": 114
 },
 {
  "cat": "通用警告",
  "en": "No SD card",
  "zh": "无 SD 卡",
  "cause": "SD 卡损坏或缺失",
  "fix": "格式化 SD 卡，或更换 SD 卡",
  "src": "csv",
  "id": 115
 },
 {
  "cat": "通用警告",
  "en": "No sufficiently close rally point located",
  "zh": "未找到足够近的拉力点",
  "cause": "拉力点距离超过 RALLY_LIMIT_KM（拉力点最大距离限制）",
  "fix": "将拉力点移至更靠近设备当前位置处，或增大 RALLY_LIMIT_KM 参数",
  "src": "csv",
  "id": 116
 },
 {
  "cat": "通用警告",
  "en": "OA requires reboot",
  "zh": "避障（OA）需重启",
  "cause": "避障配置修改后需重启才能生效",
  "fix": "重启自动驾驶仪；参考避障配置说明",
  "src": "csv",
  "id": 117
 },
 {
  "cat": "通用警告",
  "en": "OpenDroneID: ARM_STATUS not available",
  "zh": "OpenDroneID：解锁状态不可用",
  "cause": "OpenDroneID（无人机远程识别）配置错误",
  "fix": "参考 “远程识别（Remote ID）” 配置说明",
  "src": "csv",
  "id": 118
 },
 {
  "cat": "通用警告",
  "en": "OpenDroneID: operator location must be set",
  "zh": "OpenDroneID：需设置操作员位置",
  "cause": "操作员位置未设置（不可用）",
  "fix": "参考 “远程识别（Remote ID）” 配置说明",
  "src": "csv",
  "id": 119
 },
 {
  "cat": "通用警告",
  "en": "OpenDroneID: SYSTEM not available",
  "zh": "OpenDroneID：系统信息不可用",
  "cause": "OpenDroneID 配置错误",
  "fix": "参考 “远程识别（Remote ID）” 配置说明",
  "src": "csv",
  "id": 120
 },
 {
  "cat": "通用警告",
  "en": "OpenDroneID: UA_TYPE required in BasicID",
  "zh": "OpenDroneID：BasicID 中需包含 UA_TYPE（无人机类型）",
  "cause": "OpenDroneID 配置错误",
  "fix": "参考 “远程识别（Remote ID）” 配置说明",
  "src": "csv",
  "id": 121
 },
 {
  "cat": "通用警告",
  "en": "OSD_TYPE2 not compatible with first OSD",
  "zh": "OSD_TYPE2 与第一个 OSD 不兼容",
  "cause": "第一个 OSD 与第二个 OSD 配置不兼容",
  "fix": "禁用第二个 OSD（设置 OSD_TYPE2=0），或检查 OSD 配置",
  "src": "csv",
  "id": 122
 },
 {
  "cat": "通用警告",
  "en": "Param storage failed",
  "zh": "参数存储失败",
  "cause": "EEPROM（电可擦除存储器）硬件故障",
  "fix": "检查电源，或更换自动驾驶仪",
  "src": "csv",
  "id": 123
 },
 {
  "cat": "通用警告",
  "en": "parameter storage full",
  "zh": "参数存储已满",
  "cause": "EEPROM 存储已满",
  "fix": "保存当前参数；恢复出厂默认参数；重新加载保存的参数",
  "src": "csv",
  "id": 124
 },
 {
  "cat": "通用警告",
  "en": "PiccoloCAN: Servo x not detected",
  "zh": "PiccoloCAN：未检测到伺服 x",
  "cause": "PiccoloCAN 配置错误或伺服故障",
  "fix": "参考 Currawong Velocity ESC（电调品牌）设置说明",
  "src": "csv",
  "id": 125
 },
 {
  "cat": "通用警告",
  "en": "Pin x disabled (ISR flood)",
  "zh": "引脚 x 已禁用（中断请求 ISR 溢出）",
  "cause": "GPIO（通用输入输出）引脚连接的传感器信号频繁变化",
  "fix": "检查指定引脚连接的传感器",
  "src": "csv",
  "id": 126
 },
 {
  "cat": "通用警告",
  "en": "Pitch (RCx) is not neutral",
  "zh": "俯仰（RCx 通道）未回中",
  "cause": "遥控器俯仰摇杆未回中",
  "fix": "将遥控器俯仰摇杆回中，或重新进行遥控器校准",
  "src": "csv",
  "id": 127
 },
 {
  "cat": "通用警告",
  "en": "Pitch radio max too low",
  "zh": "俯仰遥控器最大行程过低",
  "cause": "遥控器俯仰通道最大行程低于 1700（PWM 值）",
  "fix": "重新进行遥控器校准，或增大 RC2_MAX（俯仰通道最大行程）至 1700 以上",
  "src": "csv",
  "id": 128
 },
 {
  "cat": "通用警告",
  "en": "Pitch radio min too high",
  "zh": "俯仰遥控器最小行程过高",
  "cause": "遥控器俯仰通道最小行程高于 1300（PWM 值）",
  "fix": "重新进行遥控器校准，或减小 RC2_MIN（俯仰通道最小行程）至 1300 以下",
  "src": "csv",
  "id": 129
 },
 {
  "cat": "通用警告",
  "en": "PRXx: No Data",
  "zh": "PRXx（接近传感器 x）：无数据",
  "cause": "接近传感器未输出数据",
  "fix": "检查接近传感器的物理连接及配置",
  "src": "csv",
  "id": 130
 },
 {
  "cat": "通用警告",
  "en": "PRXx: Not Connected",
  "zh": "PRXx（接近传感器 x）：未连接",
  "cause": "接近传感器未输出数据（未连接）",
  "fix": "检查接近传感器的物理连接及配置",
  "src": "csv",
  "id": 131
 },
 {
  "cat": "通用警告",
  "en": "Radio failsafe on",
  "zh": "遥控器失效保护已触发",
  "cause": "遥控器失效保护被触发",
  "fix": "打开遥控器，或检查遥控器失效保护配置",
  "src": "csv",
  "id": 132
 },
 {
  "cat": "通用警告",
  "en": "Rangefinder x: Not Connected",
  "zh": "测距仪 x：未连接",
  "cause": "测距仪未输出数据（未连接）",
  "fix": "检查测距仪与自动驾驶仪的物理连接及配置；同时确认固件是否包含该测距仪驱动（多数测距仪需通过自定义编译服务器添加驱动）",
  "src": "csv",
  "id": 133
 },
 {
  "cat": "通用警告",
  "en": "RC calibrating",
  "zh": "遥控器校准中",
  "cause": "遥控器校准流程正在进行",
  "fix": "完成遥控器校准流程",
  "src": "csv",
  "id": 134
 },
 {
  "cat": "通用警告",
  "en": "RC not calibrated",
  "zh": "遥控器未校准",
  "cause": "尚未进行遥控器校准",
  "fix": "完成遥控器校准；RC3_MIN（油门通道最小行程）和 RC3_MAX（油门通道最大行程）需偏离默认值 1100 和 1900；且 1-4 通道的 MIN 值需≤1300、MAX 值需≥1700",
  "src": "csv",
  "id": 135
 },
 {
  "cat": "通用警告",
  "en": "RC not found",
  "zh": "未检测到遥控器",
  "cause": "已启用遥控器失效保护，但未检测到遥控器信号",
  "fix": "打开遥控器，或检查遥控器与自动驾驶仪的连接；若仅通过地面站（GCS）操作，参考 “仅使用地面站操作” 说明",
  "src": "csv",
  "id": 136
 },
 {
  "cat": "通用警告",
  "en": "RCx_MAX is less than RCx_TRIM",
  "zh": "RCx 通道最大行程 < 微调值",
  "cause": "遥控器配置错误",
  "fix": "调整 RCx_TRIM（通道 x 微调）至低于 RCx_MAX（通道 x 最大行程），或重新进行遥控器校准",
  "src": "csv",
  "id": 137
 },
 {
  "cat": "通用警告",
  "en": "RCx_MIN is greater than RCx_TRIM",
  "zh": "RCx 通道最小行程 > 微调值",
  "cause": "遥控器配置错误",
  "fix": "调整 RCx_TRIM（通道 x 微调）至高于 RCx_MIN（通道 x 最小行程），或重新进行遥控器校准",
  "src": "csv",
  "id": 138
 },
 {
  "cat": "通用警告",
  "en": "RELAYx_PIN=y invalid",
  "zh": "继电器引脚 RELAYx_PIN=y 配置无效",
  "cause": "继电器配置错误",
  "fix": "RELAYx_PIN 参数被设为无效值；参考继电器设置说明",
  "src": "csv",
  "id": 139
 },
 {
  "cat": "通用警告",
  "en": "RELAYx_PIN=y, set SERVOx_FUNCTION=-1",
  "zh": "继电器引脚 RELAYx_PIN=y 需设置 SERVOx_FUNCTION=-1",
  "cause": "继电器配置错误",
  "fix": "将 SERVOx_FUNCTION（伺服通道功能）参数设为 - 1（禁用该通道功能）",
  "src": "csv",
  "id": 140
 },
 {
  "cat": "通用警告",
  "en": "RNGFNDDx_PIN not set",
  "zh": "测距仪引脚 RNGFNDDx_PIN 未设置",
  "cause": "测距仪配置错误",
  "fix": "将 RNGFNDDx_PIN 参数设为非零值；参考测距仪配置说明",
  "src": "csv",
  "id": 141
 },
 {
  "cat": "通用警告",
  "en": "RNGFNDDx_PIN=y invalid",
  "zh": "测距仪引脚 RNGFNDDx_PIN=y 配置无效",
  "cause": "测距仪配置错误",
  "fix": "RNGFNDDx_PIN 参数被设为无效值；参考测距仪配置说明",
  "src": "csv",
  "id": 142
 },
 {
  "cat": "通用警告",
  "en": "RNGFNDDx_PIN=y, set SERVOx_FUNCTION=-1",
  "zh": "测距仪引脚 RNGFNDDx_PIN=y 需设置 SERVOx_FUNCTION=-1",
  "cause": "测距仪配置错误",
  "fix": "将 SERVOx_FUNCTION（伺服通道功能）参数设为 - 1（禁用该通道功能）",
  "src": "csv",
  "id": 143
 },
 {
  "cat": "通用警告",
  "en": "Roll (RCx) is not neutral",
  "zh": "横滚（RCx 通道）未回中",
  "cause": "遥控器横滚摇杆未回中",
  "fix": "将遥控器横滚摇杆回中，或重新进行遥控器校准",
  "src": "csv",
  "id": 144
 },
 {
  "cat": "通用警告",
  "en": "Roll radio max too low",
  "zh": "横滚遥控器最大行程过低",
  "cause": "遥控器横滚通道最大行程低于 1700（PWM 值）",
  "fix": "重新进行遥控器校准，或增大 RC1_MAX（横滚通道最大行程）至 1700 以上",
  "src": "csv",
  "id": 145
 },
 {
  "cat": "通用警告",
  "en": "Roll radio min too high",
  "zh": "横滚遥控器最小行程过高",
  "cause": "遥控器横滚通道最小行程高于 1300（PWM 值）",
  "fix": "重新进行遥控器校准，或减小 RC1_MIN（横滚通道最小行程）至 1300 以下",
  "src": "csv",
  "id": 146
 },
 {
  "cat": "通用警告",
  "en": "RPMx_PIN not set",
  "zh": "RPM（转速）传感器引脚 RPMx_PIN 未设置",
  "cause": "RPM 传感器配置错误",
  "fix": "检查 RPMx_PIN 参数值；参考 RPM 传感器设置说明",
  "src": "csv",
  "id": 147
 },
 {
  "cat": "通用警告",
  "en": "RPMx_PIN=y invalid",
  "zh": "RPM 传感器引脚 RPMx_PIN=y 配置无效",
  "cause": "RPM 传感器配置错误",
  "fix": "RPMx_PIN 参数被设为无效值；参考 RPM 传感器设置说明",
  "src": "csv",
  "id": 148
 },
 {
  "cat": "通用警告",
  "en": "RPMx_PIN=y, set SERVOx_FUNCTION=-1",
  "zh": "RPM 传感器引脚 RPMx_PIN=y 需设置 SERVOx_FUNCTION=-1",
  "cause": "RPM 传感器配置错误",
  "fix": "将 SERVOx_FUNCTION（伺服通道功能）参数设为 - 1（禁用该通道功能）",
  "src": "csv",
  "id": 149
 },
 {
  "cat": "通用警告",
  "en": "Same Node Id x set for multiple GPS",
  "zh": "多个 GPS 设置相同节点 ID x",
  "cause": "DroneCAN GPS 配置错误",
  "fix": "确保 GPS1_CAN_NODEID 和 GPS2_CAN_NODEID 参数值不同；将其中一个设为 0 并重启自动驾驶仪",
  "src": "csv",
  "id": 150
 },
 {
  "cat": "通用警告",
  "en": "Same rnd on different CAN ports",
  "zh": "不同 CAN 端口出现相同 RND（测距仪）",
  "cause": "两个测距仪出现在不同 CAN 端口（冲突）",
  "fix": "参考 USD1、TOFSensP、NanoRadar 或 Benewake（均为测距仪品牌）设置说明",
  "src": "csv",
  "id": 151
 },
 {
  "cat": "通用警告",
  "en": "Scripting: loaded CRC incorrect want: x",
  "zh": "脚本：加载的 CRC（循环冗余校验）错误，期望 x",
  "cause": "脚本 CRC 校验错误",
  "fix": "替换为预期版本的 Lua 脚本",
  "src": "csv",
  "id": 152
 },
 {
  "cat": "通用警告",
  "en": "Scripting: running CRC incorrect want: x",
  "zh": "脚本：运行中 CRC 错误，期望 x",
  "cause": "脚本 CRC 校验错误",
  "fix": "替换为预期版本的 Lua 脚本",
  "src": "csv",
  "id": 153
 },
 {
  "cat": "通用警告",
  "en": "Scripting: xxx failed to start",
  "zh": "脚本：xxx 启动失败",
  "cause": "Lua 脚本启动失败",
  "fix": "自动驾驶仪内存不足或 Lua 脚本配置错误；参考 Lua 脚本说明",
  "src": "csv",
  "id": 154
 },
 {
  "cat": "通用警告",
  "en": "Scripting: xxx out of memory",
  "zh": "脚本：xxx 内存不足",
  "cause": "Lua 脚本内存耗尽",
  "fix": "增大 SCR_HEAP_SIZE（脚本堆内存大小）参数，或检查 Lua 脚本配置",
  "src": "csv",
  "id": 155
 },
 {
  "cat": "通用警告",
  "en": "Servo voltage to low (Xv < 4.3v)",
  "zh": "伺服电压过低（Xv < 4.3V）",
  "cause": "伺服电源轨电压低于 4.3V",
  "fix": "检查后部伺服电源轨的供电",
  "src": "csv",
  "id": 156
 },
 {
  "cat": "通用警告",
  "en": "SERVOx_FUNCTION=y on disabled channel",
  "zh": "禁用通道上配置了 SERVOx_FUNCTION=y",
  "cause": "PWM（脉冲宽度调制）输出配置错误",
  "fix": "SERVOx_FUNCTION 参数被设置在已禁用的伺服通道上；参考 BLHeli（电调固件）设置说明",
  "src": "csv",
  "id": 157
 },
 {
  "cat": "通用警告",
  "en": "SERVOx_MAX is less than SERVOx_TRIM",
  "zh": "SERVOx 通道最大行程 < 微调值",
  "cause": "PWM 输出配置错误",
  "fix": "将 SERVOx_TRIM（伺服 x 微调）设为低于 SERVOx_MAX（伺服 x 最大行程）",
  "src": "csv",
  "id": 158
 },
 {
  "cat": "通用警告",
  "en": "SERVOx_MIN is greater than SERVOx_TRIM",
  "zh": "SERVOx 通道最小行程 > 微调值",
  "cause": "PWM 输出配置错误",
  "fix": "将 SERVOx_TRIM（伺服 x 微调）设为高于 SERVOx_MIN（伺服 x 最小行程）",
  "src": "csv",
  "id": 159
 },
 {
  "cat": "通用警告",
  "en": "System not initialized",
  "zh": "系统未初始化",
  "cause": "系统仍在启动中",
  "fix": "等待；若短时间内未解决，可能存在其他诊断未检测到的传感器问题",
  "src": "csv",
  "id": 160
 },
 {
  "cat": "通用警告",
  "en": "temperature cal running",
  "zh": "温度校准中",
  "cause": "温度校准流程正在进行",
  "fix": "等待温度校准完成，或重启自动驾驶仪",
  "src": "csv",
  "id": 161
 },
 {
  "cat": "通用警告",
  "en": "terrain disabled",
  "zh": "地形功能已禁用",
  "cause": "自动任务需地形数据，但地形功能已禁用",
  "fix": "启用地形数据库（设置 TERRAIN_ENABLE=1），或移除自动任务中使用地形高度的任务项；多旋翼机型还需检查 RTL_ALT_TYPE（返航高度类型）参数",
  "src": "csv",
  "id": 162
 },
 {
  "cat": "通用警告",
  "en": "Terrain out of memory",
  "zh": "地形功能内存不足",
  "cause": "自动驾驶仪内存耗尽",
  "fix": "禁用部分功能，或更换性能更强的自动驾驶仪",
  "src": "csv",
  "id": 163
 },
 {
  "cat": "通用警告",
  "en": "terrain required but disabled",
  "zh": "需地形功能但已禁用",
  "cause": "自动任务需地形数据，但固件未包含地形功能",
  "fix": "通过自定义编译服务器编译包含地形数据库的固件，或移除自动任务中使用地形高度的任务项；多旋翼机型还需检查 RTL_ALT_TYPE 参数",
  "src": "csv",
  "id": 164
 },
 {
  "cat": "通用警告",
  "en": "Throttle (RCx) is not neutral",
  "zh": "油门（RCx 通道）未回中",
  "cause": "遥控器油门摇杆过高（未回中至最低位）",
  "fix": "降低油门摇杆至最低位，或重新进行遥控器校准",
  "src": "csv",
  "id": 165
 },
 {
  "cat": "通用警告",
  "en": "Throttle radio max too low",
  "zh": "油门遥控器最大行程过低",
  "cause": "遥控器油门通道最大行程低于 1700（PWM 值）",
  "fix": "重新进行遥控器校准，或增大 RC2_MAX（油门通道最大行程）至 1700 以上",
  "src": "csv",
  "id": 166
 },
 {
  "cat": "通用警告",
  "en": "Throttle radio min too high",
  "zh": "油门遥控器最小行程过高",
  "cause": "遥控器油门通道最小行程高于 1300（PWM 值）",
  "fix": "重新进行遥控器校准，或减小 RC1_MIN（油门通道最小行程）至 1300 以下",
  "src": "csv",
  "id": 167
 },
 {
  "cat": "通用警告",
  "en": "Too many auxiliary authorisers",
  "zh": "辅助授权器过多",
  "cause": "超过 3 个外部系统控制解锁权限",
  "fix": "检查外部授权系统",
  "src": "csv",
  "id": 168
 },
 {
  "cat": "通用警告",
  "en": "vehicle outside fence",
  "zh": "设备在地理围栏外",
  "cause": "设备当前位置在地理围栏范围外",
  "fix": "将设备移至地理围栏范围内",
  "src": "csv",
  "id": 169
 },
 {
  "cat": "通用警告",
  "en": "VisOdom: not healthy",
  "zh": "视觉里程计（VisOdom）状态异常",
  "cause": "视觉里程计传感器未输出数据",
  "fix": "检查视觉里程计的物理连接及配置",
  "src": "csv",
  "id": 170
 },
 {
  "cat": "通用警告",
  "en": "VisOdom: out of memory",
  "zh": "视觉里程计（VisOdom）内存不足",
  "cause": "自动驾驶仪内存耗尽",
  "fix": "禁用部分功能，或更换性能更强的自动驾驶仪",
  "src": "csv",
  "id": 171
 },
 {
  "cat": "通用警告",
  "en": "VTOL Fwd Throttle iz not zero",
  "zh": "垂直起降（VTOL）前向油门非零",
  "cause": "遥控器垂直起降前向油门摇杆过高",
  "fix": "降低垂直起降前向油门摇杆，或重新进行遥控器校准",
  "src": "csv",
  "id": 172
 },
 {
  "cat": "通用警告",
  "en": "waiting for terrain data",
  "zh": "等待地形数据",
  "cause": "等待地面站（GCS）提供所需地形数据",
  "fix": "等待，或移至 GPS 信号更好的位置",
  "src": "csv",
  "id": 173
 },
 {
  "cat": "通用警告",
  "en": "Yaw (RCx) is not neutral",
  "zh": "偏航（RCx 通道）未回中",
  "cause": "遥控器偏航摇杆未回中",
  "fix": "将遥控器偏航摇杆回中，或重新进行遥控器校准",
  "src": "csv",
  "id": 174
 },
 {
  "cat": "通用警告",
  "en": "Yaw radio max too low",
  "zh": "偏航遥控器最大行程过低",
  "cause": "遥控器偏航通道最大行程低于 1700（PWM 值）",
  "fix": "重新进行遥控器校准，或增大 RC2_MAX（偏航通道最大行程）至 1700 以上",
  "src": "csv",
  "id": 175
 },
 {
  "cat": "通用警告",
  "en": "Yaw radio min too high",
  "zh": "偏航遥控器最小行程过高",
  "cause": "遥控器偏航通道最小行程高于 1300（PWM 值）",
  "fix": "重新进行遥控器校准，或减小 RC1_MIN（偏航通道最小行程）至 1300 以下",
  "src": "csv",
  "id": 176
 },
 {
  "cat": "通用警告",
  "en": "Check EK3_SRCx_POSXY/VELXY/POSZ/VELZ/YAW",
  "zh": "EKF3 数据源参数配置值不受支持",
  "cause": "EKF3_SRCx_POSXY/VELXY/POSZ/VELZ/YAW 数据源参数取了不受支持的值",
  "fix": "复查对应的 EK3_SRC 参数组，选择受支持的数据源",
  "src": "wiki",
  "id": 177
 },
 {
  "cat": "通用警告",
  "en": "Compass not calibrated",
  "zh": "指南针未校准",
  "cause": "指南针偏移量为零，或传感器数量发生变化",
  "fix": "完成指南针校准",
  "src": "wiki",
  "id": 178
 },
 {
  "cat": "通用警告",
  "en": "Compass X not healthy",
  "zh": "指南针 X 状态异常",
  "cause": "第 X 个指南针未输出数据",
  "fix": "检查该指南针与自动驾驶仪的连接及配置",
  "src": "wiki",
  "id": 179
 },
 {
  "cat": "通用警告",
  "en": "EKF3 sources require x",
  "zh": "EKF3 数据源 x 不可用",
  "cause": "所选的 EKF3 数据源当前不可用",
  "fix": "连接或配置所提示的传感器，或在对应的 EK3_SRC 参数组中改用可用的数据源",
  "src": "wiki",
  "id": 180
 },
 {
  "cat": "通用警告",
  "en": "RELAYx_PIN=y, set SERVx_FUNCTION=-1",
  "zh": "继电器引脚 RELAYx_PIN=y 需设置 SERVx_FUNCTION=-1",
  "cause": "继电器（Relay）配置错误",
  "fix": "将对应 SERVOx_FUNCTION（伺服通道功能）参数设为 -1（禁用该通道功能）",
  "src": "wiki",
  "id": 181
 },
 {
  "cat": "通用警告",
  "en": "RNGFNDx_PIN not set",
  "zh": "测距仪引脚 RNGFNDx_PIN 未设置",
  "cause": "测距仪（Rangefinder）配置错误（引脚未设置）",
  "fix": "将 RNGFNDx_PIN 设为非零值，参考测距仪配置说明",
  "src": "wiki",
  "id": 182
 },
 {
  "cat": "通用警告",
  "en": "RNGFNDx_PIN=y invalid",
  "zh": "测距仪引脚 RNGFNDx_PIN=y 配置无效",
  "cause": "测距仪配置错误（引脚值无效）",
  "fix": "RNGFNDx_PIN 被设为无效值，参考测距仪配置说明",
  "src": "wiki",
  "id": 183
 },
 {
  "cat": "通用警告",
  "en": "RNGFNDx_PIN=y, set SERVOx_FUNCTION=-1",
  "zh": "测距仪引脚 RNGFNDx_PIN=y 需设置 SERVOx_FUNCTION=-1",
  "cause": "测距仪配置错误",
  "fix": "将对应 SERVOx_FUNCTION（伺服通道功能）参数设为 -1（禁用该通道功能）",
  "src": "wiki",
  "id": 184
 },
 {
  "cat": "通用警告",
  "en": "Same rfnd on different CAN ports",
  "zh": "不同 CAN 端口上配置了相同测距仪",
  "cause": "两个测距仪出现在不同 CAN 端口上",
  "fix": "参考 USD1、TOFSensP、NanoRadar 或 Benewake 等测距仪的配置说明，检查 CAN 端口分配",
  "src": "wiki",
  "id": 185
 },
 {
  "cat": "通用警告",
  "en": "terrain data expired, possible errors",
  "zh": "地形数据过期（可能存在误差）",
  "cause": "SD 卡上的地形数据已过期",
  "fix": "更新 SD 卡上的地形数据",
  "src": "wiki",
  "id": 186
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "ADSB threat detected",
  "zh": "检测到ADSB威胁",
  "cause": "ADSB故障保护。附近有载人飞行器",
  "fix": "参见ADSB配置 (前提是要加装了ADSB装置)",
  "src": "csv",
  "id": 187
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "AHRS not healthy",
  "zh": "AHRS不健康",
  "cause": "AHRS/EKF尚未准备就绪",
  "fix": "等待。重启自动驾驶仪",
  "src": "csv",
  "id": 188
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Altitude disparity",
  "zh": "高度差异",
  "cause": "气压计和EKF高度相差至少1米",
  "fix": "等待EKF高度稳定。重启自动驾驶仪",
  "src": "csv",
  "id": 189
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Auto mode not armable",
  "zh": "自动模式无法解锁",
  "cause": "飞行器在自动模式下无法解锁",
  "fix": "切换到其他模式（如悬停）或设置 RTL_OPTIONS = 3。参见自动模式",
  "src": "csv",
  "id": 190
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Bad parameter: ATC_ANG_PIT_P must be > 0",
  "zh": "参数错误：ATC_ANG_PIT_P 必须 > 0",
  "cause": "姿态控制器配置错误",
  "fix": "增加指定参数值至零以上。参见调参过程说明",
  "src": "csv",
  "id": 191
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Bad parameter: PSC_POSXY_P must be > 0",
  "zh": "参数错误：PSC_POSXY_P 必须 > 0",
  "cause": "位置控制器配置错误",
  "fix": "增加指定参数值至零以上。参见调参过程说明",
  "src": "csv",
  "id": 192
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Battery failsafe",
  "zh": "电池故障保护",
  "cause": "电池故障保护已触发",
  "fix": "连接电池并检查其电压和容量。参见：电池故障保护配置",
  "src": "csv",
  "id": 193
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Check ACRO_BAL_ROLL/PITCH",
  "zh": "检查 ACRO_BAL_ROLL/PITCH",
  "cause": "ACRO_BAL_ROLL 或 ACRO_BAL_PITCH 过高",
  "fix": "降低 ACRO_BAL_ROLL 至 ATC_ANG_P 以下，和/或降低 ACRO_BAL_PITCH 至 ATC_ANG_P 以下。参见特技模式",
  "src": "csv",
  "id": 194
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Check ANGLE_MAX",
  "zh": "检查 ANGLE_MAX",
  "cause": "ANGLE_MAX 设置过高",
  "fix": "将 ANGLE_MAX 降低至 8000（例如 80 度）或更低",
  "src": "csv",
  "id": 195
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Check FS_THR_VALUE",
  "zh": "检查 FS_THR_VALUE",
  "cause": "遥控器故障保护配置错误",
  "fix": "将 FS_THR_VALUE 设置在 910 和遥控器油门最小值之间（例如 RC3_MIN）。参见：电池故障保护配置",
  "src": "csv",
  "id": 196
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Check PILOT_SPEED_UP",
  "zh": "检查 PILOT_SPEED_UP",
  "cause": "PILOT_SPEED_UP 设置过低",
  "fix": "将 PILOT_SPEED_UP 增加到正数（例如 100 = 1m/s）。参见定高模式",
  "src": "csv",
  "id": 197
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Collective below failsafe (TradHeli only)",
  "zh": "总距低于故障保护值（仅限传统直升机）",
  "cause": "遥控器总距输入低于 FS_THR_VALUE",
  "fix": "打开遥控器发射机或检查 FS_THR_VALUE。检查遥控器故障保护设置",
  "src": "csv",
  "id": 198
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "EKF attitude is bad",
  "zh": "EKF 姿态不佳",
  "cause": "EKF 没有良好的姿态估计",
  "fix": "等待EKF姿态稳定。重启自动驾驶仪。更换自动驾驶仪",
  "src": "csv",
  "id": 199
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "EKF compass variance",
  "zh": "EKF 罗盘方差",
  "cause": "罗盘方向似乎不正确",
  "fix": "将飞行器移离环境中的金属。将罗盘移离机架中的金属。重新进行罗盘校准。禁用内部罗盘。",
  "src": "csv",
  "id": 200
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "EKF height variance",
  "zh": "EKF 高度方差",
  "cause": "气压计值不稳定或振动过大",
  "fix": "等待。测量振动并添加减震装置",
  "src": "csv",
  "id": 201
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "EKF position variance",
  "zh": "EKF 位置方差",
  "cause": "GPS 位置不稳定",
  "fix": "等待。如果在室内，请到室外。消除可能干扰GPS的射频源",
  "src": "csv",
  "id": 202
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "EKF velocity variance",
  "zh": "EKF 速度方差",
  "cause": "GPS 或光流速度不稳定",
  "fix": "等待。如果在室内，请到室外。消除可能干扰GPS的射频源。检查光流校准",
  "src": "csv",
  "id": 203
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Fence enabled, need position estimate",
  "zh": "电子围栏已启用，需要位置估计",
  "cause": "电子围栏已启用，因此需要位置估计",
  "fix": "等待。如果在室内，请到室外。确保罗盘和加速度计校准已完成。消除可能干扰GPS的射频源。参见电子围栏配置",
  "src": "csv",
  "id": 204
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "FS_GCS_ENABLE=2 removed, see FS_OPTIONS",
  "zh": "FS_GCS_ENABLE=2 已移除，参见 FS_OPTIONS",
  "cause": "地面站故障保护配置错误",
  "fix": "设置 FS_GCS_ENABLE = 1 并检查 FS_OPTIONS 参数。参见：地面站故障保护配置",
  "src": "csv",
  "id": 205
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "GCS failsafe on",
  "zh": "地面站故障保护开启",
  "cause": "地面站故障保护已触发",
  "fix": "检查数传连接。参见：地面站故障保护配置",
  "src": "csv",
  "id": 206
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "GPS glitching",
  "zh": "GPS 信号跳动",
  "cause": "GPS 位置不稳定",
  "fix": "等待。如果在室内，请到室外。消除可能干扰GPS的射频源",
  "src": "csv",
  "id": 207
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "High GPS HDOP",
  "zh": "GPS HDOP 过高",
  "cause": "GPS 水平质量太低",
  "fix": "等待或移动到GPS接收更好的位置。您可以提高 GPS_HDOP_GOOD，但这通常不是个好主意",
  "src": "csv",
  "id": 208
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Home too far from EKF origin",
  "zh": "返航点距离EKF原点太远",
  "cause": "返航点距离EKF原点超过50公里",
  "fix": "重启自动驾驶仪以将EKF原点重置为当前位置",
  "src": "csv",
  "id": 209
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Interlock/E-Stop Conflict (TradHeli only)",
  "zh": "互锁/急停冲突（仅限传统直升机）",
  "cause": "配置了不兼容的辅助功能开关",
  "fix": "从辅助功能设置中移除互锁、急停或紧急停止",
  "src": "csv",
  "id": 210
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Invalid MultiCopter FRAME_CLASS",
  "zh": "无效的多旋翼 FRAME_CLASS",
  "cause": "FRAME_CLASS 参数配置错误",
  "fix": "加载了多旋翼固件但 FRAME_CLASS 设置为直升机。加载直升机固件或更改 FRAME_CLASS",
  "src": "csv",
  "id": 211
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Inverted flight option not supported",
  "zh": "不支持倒飞选项",
  "cause": "不支持倒飞辅助功能",
  "fix": "移除倒飞辅助功能开关",
  "src": "csv",
  "id": 212
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Leaning",
  "zh": "倾斜",
  "cause": "飞行器倾斜角度超过 ANGLE_MAX",
  "fix": "将飞行器调平",
  "src": "csv",
  "id": 213
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motor Interlock Enabled",
  "zh": "电机互锁已启用",
  "cause": "电机互锁处于中间或高位",
  "fix": "将电机互锁辅助功能开关移到低位",
  "src": "csv",
  "id": 214
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motor Interlock not configured",
  "zh": "未配置电机互锁",
  "cause": "直升机需要配置电机互锁",
  "fix": "启用电机互锁辅助功能开关",
  "src": "csv",
  "id": 215
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motors: Check frame class and type",
  "zh": "电机：检查机架类别和类型",
  "cause": "机架类别或类型未知或配置错误",
  "fix": "输入有效的机架类别和/或类型",
  "src": "csv",
  "id": 216
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motors: Check MOT_PWM_MIN and MOT_PWM_MAX",
  "zh": "电机：检查 MOT_PWM_MIN 和 MOT_PWM_MAX",
  "cause": "MOT_PWM_MIN 或 MOT_PWM_MAX 配置错误",
  "fix": "设置 MOT_PWM_MIN = 1000 和 MOT_PWM_MAX = 2000 并重新进行电调校准",
  "src": "csv",
  "id": 217
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motors: MOT_SPIN_ARM > MOT_SPIN_MIN",
  "zh": "电机：MOT_SPIN_ARM > MOT_SPIN_MIN",
  "cause": "MOT_SPIN_ARM 过高或 MOT_SPIN_MIN 过低",
  "fix": "降低 MOT_SPIN_ARM 至 MOT_SPIN_MIN 以下。参见设置电机范围",
  "src": "csv",
  "id": 218
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motors: MOT_SPIN_MIN too high x > 0.3",
  "zh": "电机：MOT_SPIN_MIN 过高 x > 0.3",
  "cause": "MOT_SPIN_MIN 参数值过高",
  "fix": "降低 MOT_SPIN_MIN 至 0.3 以下。参见设置电机范围",
  "src": "csv",
  "id": 219
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Motors: no SERVOx_FUNCTION set to MotorX",
  "zh": "电机：没有 SERVOx_FUNCTION 设置为 MotorX",
  "cause": "至少有一个电机输出未配置",
  "fix": "检查 SERVOx_FUNCTION 值是否为 \"Motor1\", \"Motor2\" 等。检查电调和电机配置",
  "src": "csv",
  "id": 220
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Need Alt Estimate",
  "zh": "需要高度估计",
  "cause": "EKF 尚未计算出高度",
  "fix": "等待。允许自动驾驶仪预热。确保加速度计校准已完成。",
  "src": "csv",
  "id": 221
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Need Position Estimate",
  "zh": "需要位置估计",
  "cause": "EKF 没有位置估计",
  "fix": "等待。如果在室内，请到室外。确保罗盘和加速度计校准已完成。消除可能干扰GPS的射频源",
  "src": "csv",
  "id": 222
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Proximity x deg, ym (want > Zm)",
  "zh": "接近障碍物 x 度, y米 (需要 > Z米)",
  "cause": "障碍物离飞行器太近",
  "fix": "将障碍物移开或检查传感器。参见接近传感器配置",
  "src": "csv",
  "id": 223
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "RTL mode not armable",
  "zh": "RTL 模式无法解锁",
  "cause": "飞行器在RTL模式下无法解锁",
  "fix": "切换到其他飞行模式",
  "src": "csv",
  "id": 224
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "RTL_ALT_TYPE is above- terrain but no rangefinder unavailable",
  "zh": "RTL_ALT_TYPE 是高于地形但无测距仪可用",
  "cause": "RTL 使用测距仪但测距仪不可用",
  "fix": "检查测距仪配置，包括 RNGFNDx_ORIENT=251",
  "src": "csv",
  "id": 225
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "RTL_ALT_TYPE is above- terrain but no terrain data",
  "zh": "RTL_ALT_TYPE 是高于地形但无地形数据",
  "cause": "RTL 使用地形但地形数据库不可用",
  "fix": "设置 TERRAIN_ENABLE = 1。参见地形跟随",
  "src": "csv",
  "id": 226
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "RTL_ALT_TYPE is above- terrain but RTL_ALT>RNGFND_MAX",
  "zh": "RTL_ALT_TYPE 是高于地形但 RTL_ALT>RNGFND_MAX",
  "cause": "RTL 返航高度超过测距仪范围",
  "fix": "降低 RTL_ALT 至小于 RNGFNDx_MAX。参见地形跟随",
  "src": "csv",
  "id": 227
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Safety Switch",
  "zh": "安全开关",
  "cause": "硬件安全开关未被按下",
  "fix": "按下安全开关（通常在GPS顶部）或通过设置 BRD_SAFETY_DEFLT 为零并重启自动驾驶仪来禁用",
  "src": "csv",
  "id": 228
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Throttle below failsafe",
  "zh": "油门低于故障保护值",
  "cause": "遥控器油门输入低于 FS_THR_VALUE",
  "fix": "打开遥控器发射机或检查 FS_THR_VALUE。检查遥控器故障保护设置",
  "src": "csv",
  "id": 229
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "Vehicle too far from EKF origin",
  "zh": "飞行器距离EKF原点太远",
  "cause": "飞行器距离EKF原点超过50公里",
  "fix": "重启自动驾驶仪以将EKF原点重置为当前位置",
  "src": "csv",
  "id": 230
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "winch unhealthy",
  "zh": "绞车不健康",
  "cause": "绞车未与自动驾驶仪通信",
  "fix": "检查绞车的物理连接和配置",
  "src": "csv",
  "id": 231
 },
 {
  "cat": "仅多轴或直升机的警告",
  "en": "RTL_ALT_TYPE is above-terrain but no rangefinder",
  "zh": "RTL_ALT_TYPE 设为基于地形但无测距仪",
  "cause": "RTL 使用基于地形的高度（above-terrain），但没有可用的测距仪",
  "fix": "检查测距仪配置（含 RNGFNDx_ORIENT=251）",
  "src": "wiki",
  "id": 232
 },
 {
  "cat": "仅固定翼的警告",
  "en": "ADSB threat detected",
  "zh": "检测到 ADSB 威胁",
  "cause": "ADSB 故障保护；附近存在载人飞行器",
  "fix": "参阅 ADSB 配置",
  "src": "csv",
  "id": 233
 },
 {
  "cat": "仅固定翼的警告",
  "en": "AHRS not healthy",
  "zh": "航姿参考系统（AHRS）异常",
  "cause": "航姿参考系统 / 扩展卡尔曼滤波器尚未就绪",
  "fix": "等待；重启自动驾驶仪",
  "src": "csv",
  "id": 234
 },
 {
  "cat": "仅固定翼的警告",
  "en": "AIRSPEED_MIN too low x<5 m/s",
  "zh": "空速最小值过低（<5 米 / 秒）",
  "cause": "参数设置过低，低于 5 米 / 秒",
  "fix": "调至失速速度的 120% 及以上",
  "src": "csv",
  "id": 235
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Bad parameter: ATC_ANG_PIT_P must be > 0",
  "zh": "参数错误：姿态角俯仰控制器参数（ATC_ANG_PIT_P）必须＞0",
  "cause": "姿态控制器配置错误",
  "fix": "将该参数值调至大于 0；参阅调试流程说明",
  "src": "csv",
  "id": 236
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Bad parameter: PSC_POSXY_P must be > 0",
  "zh": "参数错误：位置控制器参数（PSC_POSXY_P）必须＞0",
  "cause": "位置控制器配置错误",
  "fix": "将该参数值调至大于 0；参阅调试流程说明",
  "src": "csv",
  "id": 237
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Check Q_ANGLE_MAX",
  "zh": "检查 Q_ANGLE_MAX 参数",
  "cause": "设置值超过 80 度",
  "fix": "将 Q_ANGLE_MAX 调至 80 度以下；30 度为推荐值",
  "src": "csv",
  "id": 238
 },
 {
  "cat": "仅固定翼的警告",
  "en": "In landing sequence",
  "zh": "处于着陆序列中",
  "cause": "在着陆序列中尝试解锁",
  "fix": "重置任务；切换至非着陆序列的任务项",
  "src": "csv",
  "id": 239
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Invalid THR_FS_VALUE for reversed throttle input",
  "zh": "油门反向输入对应的 THR_FS_VALUE 参数无效",
  "cause": "THR_FS_VALUE 的脉冲宽度值未高于最大油门脉冲宽度",
  "fix": "将 THR_FS_VALUE 调至最大油门脉冲宽度值之上",
  "src": "csv",
  "id": 240
 },
 {
  "cat": "仅固定翼的警告",
  "en": "ROLL_LIMIT_DEG too small x",
  "zh": "横滚限制角度（ROLL_LIMIT_DEG）过小",
  "cause": "参数设置低于 3 度",
  "fix": "调大参数；45 度为保证控制效果的推荐最小值",
  "src": "csv",
  "id": 241
 },
 {
  "cat": "仅固定翼的警告",
  "en": "PTCH_LIM_MAX_DEG too small x",
  "zh": "俯仰最大限制角度（PTCH_LIM_MAX_DEG）过小",
  "cause": "参数设置低于 3 度",
  "fix": "调大参数；45 度为保证控制效果的推荐最小值",
  "src": "csv",
  "id": 242
 },
 {
  "cat": "仅固定翼的警告",
  "en": "PTCH_LIM_MIN_DEG too large x",
  "zh": "俯仰最小限制角度（PTCH_LIM_MIN_DEG）过大",
  "cause": "参数设置高于 3 度",
  "fix": "调大参数；45 度为保证控制效果的推荐最小值",
  "src": "csv",
  "id": 243
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Mode not armable",
  "zh": "该模式无法解锁",
  "cause": "此模式下不支持解锁",
  "fix": "切换模式",
  "src": "csv",
  "id": 244
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Mode not QMODE",
  "zh": "非 Q 模式（QMODE）",
  "cause": "Q_OPTION 设置为仅在 Q 模式 / 自动模式下允许解锁",
  "fix": "切换模式或重置 Q_OPTIONS 第 18 位",
  "src": "csv",
  "id": 245
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Motors: Check frame class and type",
  "zh": "电机：检查机架类型与分类",
  "cause": "机架类型 / 分类未知或配置错误",
  "fix": "输入有效的机架分类和 / 或类型",
  "src": "csv",
  "id": 246
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Motors: Check MOT_PWM_MIN and MOT_PWM_MAX",
  "zh": "电机：检查 MOT_PWM_MIN/MOT_PWM_MAX 参数",
  "cause": "MOT_PWM_MIN 或 MOT_PWM_MAX 配置错误",
  "fix": "设置 MOT_PWM_MIN=1000、MOT_PWM_MAX=2000，重新执行电子调速器（ESC）校准",
  "src": "csv",
  "id": 247
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Motors: MOT_SPIN_ARM > MOT_SPIN_MIN",
  "zh": "电机：解锁自旋值（MOT_SPIN_ARM）大于最小自旋值（MOT_SPIN_MIN）",
  "cause": "解锁自旋值过高或最小自旋值过低",
  "fix": "将解锁自旋值调至低于最小自旋值；参阅电机范围设置说明",
  "src": "csv",
  "id": 248
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Motors: MOT_SPIN_MIN too high x > 0.3",
  "zh": "电机：最小自旋值（MOT_SPIN_MIN）过高（＞0.3）",
  "cause": "MOT_SPIN_MIN 参数值过高",
  "fix": "将最小自旋值调至 0.3 以下；参阅电机范围设置说明",
  "src": "csv",
  "id": 249
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Motors: no SERVOx_FUNCTION set to MotorX",
  "zh": "电机：未将任意伺服通道功能（SERVOx_FUNCTION）设为电机 X",
  "cause": "至少一个电机输出未配置",
  "fix": "检查 “Motor1”“Motor2” 等对应的 SERVOx_FUNCTION 参数；检查电子调速器（ESC）和电机配置",
  "src": "csv",
  "id": 250
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Q_ASSIST_SPEED is not set",
  "zh": "未设置 Q 辅助速度（Q_ASSIST_SPEED）",
  "cause": "Q_ASSIST_SPEED 参数未配置",
  "fix": "设置 Q_ASSIST_SPEED；参阅固定翼辅助飞行说明",
  "src": "csv",
  "id": 251
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Quadplane enabled but not running",
  "zh": "四旋翼机型已启用但未运行",
  "cause": "已设置 Q_ENABLE 参数，但四旋翼机型代码未启动",
  "fix": "重启",
  "src": "csv",
  "id": 252
 },
 {
  "cat": "仅固定翼的警告",
  "en": "quadplane needs SCHED_LOOP_RATE >= 100",
  "zh": "四旋翼机型要求调度循环频率（SCHED_LOOP_RATE）≥100",
  "cause": "四旋翼机型需更高循环频率保证性能",
  "fix": "调高 SCHED_LOOP_RATE；300 为推荐值",
  "src": "csv",
  "id": 253
 },
 {
  "cat": "仅固定翼的警告",
  "en": "set Q_FWD_THR_USE to 0",
  "zh": "将 Q 前向油门使用参数（Q_FWD_THR_USE）设为 0",
  "cause": "尾座式机型尝试使用前向油门",
  "fix": "尾座式机型需将 Q_FWD_THR_USE 设为 0",
  "src": "csv",
  "id": 254
 },
 {
  "cat": "仅固定翼的警告",
  "en": "set TAILSIT_ENABLE 0 or TILT_ENABLE 0",
  "zh": "将尾座模式使能（TAILSIT_ENABLE）设为 0 或 将倾转使能（TILT_ENABLE）设为 0",
  "cause": "倾转旋翼与尾座模式无法同时启用",
  "fix": "选择与机型匹配的一项并重启",
  "src": "csv",
  "id": 255
 },
 {
  "cat": "仅固定翼的警告",
  "en": "tiltsitter setup not complete, reboot",
  "zh": "尾座式机型设置未完成，请重启",
  "cause": "已启用倾转旋翼但未重启",
  "fix": "重启",
  "src": "csv",
  "id": 256
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Throttle trim not near center stick %x",
  "zh": "油门微调未接近摇杆中心位置",
  "cause": "油门摇杆居中对应的遥控器（RC）微调设置错误",
  "fix": "若已设置 FLIGHT_OPTIONS 第 10 位，将油门通道的遥控器微调设至中心位置（怠速）",
  "src": "csv",
  "id": 257
 },
 {
  "cat": "仅固定翼的警告",
  "en": "unset one of RTL_AUTOLAND or Q_RTL_MODE",
  "zh": "仅保留自动返航着陆（RTL_AUTOLAND）或 Q 模式自动返航（Q_RTL_MODE）其中一项",
  "cause": "两项参数均已设置",
  "fix": "垂直起降飞行器（VTOL）中仅用其一设置自动返航（RTL）模式行为",
  "src": "csv",
  "id": 258
 },
 {
  "cat": "仅固定翼的警告",
  "en": "Waiting for RC",
  "zh": "等待遥控器（RC）信号",
  "cause": "已启用遥控器故障保护但无信号",
  "fix": "开启遥控器发射机；检查遥控器与自动驾驶仪的连接；若仅用地面控制站（GCS），参阅《仅使用 GCS 操作》说明",
  "src": "csv",
  "id": 259
 }
];

  function norm(s) {
    return String(s === undefined || s === null ? '' : s)
      .toLowerCase()
      .replace(/[^a-z0-9\u4e00-\u9fff]+/g, '');
  }

  // 单词命中: 原文包含 或 归一化包含 (容忍标点/大小写差异)
  function tokenHit(tok, raw, nrm) {
    if (raw.includes(tok)) return true;
    const t = norm(tok);
    if (!t) return true;
    return nrm.includes(t);
  }

  function score(e, q, qn) {
    const enN = norm(e.en);
    if (qn && enN === qn) return 100;
    if (qn && enN.startsWith(qn)) return 80;
    if (q && e.en.toLowerCase().includes(q)) return 60;
    if (q && e.zh.includes(q)) return 50;
    if (q && (e.cause.includes(q) || e.fix.includes(q))) return 20;
    return 10;
  }

  // 搜索: 多词 AND (中英文均可), cat 为空表示全部分类; 空查询返回该分类全部
  function search(query, cat) {
    let list = ENTRIES;
    if (cat && CATS.indexOf(cat) >= 0) list = list.filter((e) => e.cat === cat);
    const q = String(query === undefined || query === null ? '' : query).trim().toLowerCase();
    if (!q) return list.slice();
    // 剥离 GCS/日志装饰前缀 (PreArm: / failed to ...), 剥空的 token 弃用; 全被剥空时退回原 token
    let tokens = q.split(/\s+/).filter(Boolean)
      .map((t) => t.replace(/^(prearmfailure|prearm|failedto|failed|warning)[\s:：_\-]*/, ''))
      .filter(Boolean);
    if (!tokens.length) tokens = q.split(/\s+/).filter(Boolean);
    const qn = norm(q);
    const hits = [];
    for (const e of list) {
      const raw = (e.en + ' ' + e.zh + ' ' + e.cause + ' ' + e.fix).toLowerCase();
      const nrm = norm(raw);
      if (tokens.every((t) => tokenHit(t, raw, nrm))) hits.push([score(e, q, qn), e]);
    }
    hits.sort((a, b) => b[0] - a[0]);
    return hits.map((x) => x[1]);
  }

  function byId(id) {
    return ENTRIES[id] && ENTRIES[id].id === id ? ENTRIES[id] : ENTRIES.find((e) => e.id === id) || null;
  }

  g.FaultDB = {
    META, CATS, CAT_SHORT, ENTRIES,
    count: ENTRIES.length,
    search, byId
  };
})(typeof window !== 'undefined' ? window : globalThis);
