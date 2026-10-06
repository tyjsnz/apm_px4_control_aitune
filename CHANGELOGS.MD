# CHANGELOG

APM/ArduPilot MAVLink 控制脚本开发日志。

## v1.10 - 2026-09-24

### 新增 — `amp_control_ui.py` 图形测试界面
与 `apm_control_test.py` 功能完全一致(测试1~15 + 运行全部), tkinter 实现:

- **实时遥测面板**: 模式/解锁/系统状态/心跳、GPS(fix/卫星/HDOP)、经纬度、相对/绝对高度、速度 NED、地速空速油门、航向爬升、姿态 RPY、电池 V/A/%、CPU 负载、EKF 方差、RC ch1-4/RSSI、链路 msg/s、最近飞控 STATUSTEXT。
- **高度曲线**: 最近 150 点相对高度走势。
- **控制信号表**: 近 1 秒各 MAVLink 指令发送频率 + 最近指令 + RC 覆盖状态。
- **快速控制**: 解锁/锁定/起飞/降落/RTL、模式切换、速度微动(前后左右升降停止)。
- **日志窗口**: 截获 print 输出(含 `[飞控]` PreArm 等), 按级别着色。
- **参数面板**: 连接串/波特率/起飞高度/低空起降高度/测试速度/时长, 写入 `apm_control_test` 同名常量。
- 架构: 单读线程收包 → 遥测显示 + 捕获队列(测试 `recv_match` 消费), 指令动作单工作线程排队, GUI 与飞控线程隔离。
- 退出时若仍解锁则弹窗警告。

## v1.9 - 2026-09-24

### 修复 — `apm_control.py` 解锁/起飞/降落与测试脚本同类问题
对照 `apm_control_test.py` 已修项, 检查并加固 `apm_control.py`:

| 问题 | 原实现 | 现实现 |
|------|--------|--------|
| 油门中位解锁被拒 | `arm()` 直接发 ARM, 无 RC 覆盖 | 解锁期间 **RC 油门=1000**, 结束释放 |
| PreArm 看不见 | 只看 ACK result | 同时打印 **STATUSTEXT/PreArm** |
| 解锁后未起飞自动上锁 | 未设 `DISARM_DELAY` | `ensure_disarm_delay` 写入默认 **60s**(`DISARM_DELAY_SEC`) |
| 解锁→起飞分离 | 测试3/4 先 `arm` 再 `takeoff` | 新增 **`arm_and_takeoff()`** 一体调用 |
| 上锁假成功 | `disarm` 只 sleep(2) 打印 OK | 校验 **ACK/心跳**, 失败自动 **强制上锁** |
| 起飞 ACK 不校验 | 收到 ACK 不管 result | 非 ACCEPTED 报错退出; 起飞中超时/失锁提示 |
| 着陆未确认仍上锁 | `wait_landed` 超时也继续 `disarm` | 着陆后等自动 DISARM, 仍解锁才发 DISARM |

- 配置: 顶部 `DISARM_DELAY_SEC = 60`(`0` 不改飞控参数)。
- 覆盖: `arm` / `takeoff` / `arm_and_takeoff` / `disarm` / `wait_landed` / 测试2/3/4 流程。

## v1.8.2 - 2026-09-24

### 修复 — 测试14 电池状态异常
- `MAVLink_battery_status_message` **无 `battery_type` 属性** → `AttributeError`。
- pymavlink 正确字段名为 **`msg.type`**; 已改 `getattr(msg, 'type', ...)` 并对 temperature/current 等字段做 `getattr` 防御, 避免不同 pymavlink 版本缺字段再炸。

## v1.8.1 - 2026-09-24

### 修复 — 解锁后自动上锁 / 测试4 与测试2 分离来不及
- **根因**: ArduCopter 参数 **`DISARM_DELAY`**(默认约 10s): 解锁后一直低油门未起飞 → 自动 DISARM。
  - RC 解锁后不推油门「自己停并锁定」= 同一机制;
  - 测试2 解锁成功 → 再选测试4 时可能已被自动上锁 → 提示「先解锁」。
- **修复** (`apm_control_test.py`):
  1. 新增 `set_param()` / `ensure_disarm_delay()`: 起飞类测试前写 **`DISARM_DELAY = DISARM_DELAY_SEC`(默认 60)**, `0` 表示不改飞控;
  2. 新增 `arm_and_takeoff()`: **解锁+立即 NAV_TAKEOFF 一体**, 解锁后立刻发起飞, 不再依赖先跑测试2;
  3. **测试4** 改为集成起飞(未解锁会自动解锁); **测试15** 第1步改为集成; 测试2 成功后提示须在时限内起飞;
  4. 测试2 也会先延长 DISARM_DELAY, 单独解锁测试时窗口更宽。
- 可调: 顶部 `DISARM_DELAY_SEC`、`ARM_TO_TAKEOFF_MAX`; Mission Planner 全部参数里也可改 `DISARM_DELAY`。

## v1.8 - 2026-09-24

### 新增 — 菜单15 低空起降悬停 (`apm_control_test.py`)
- **流程**: 等GPS锁星 → GUIDED解锁(`arm_vehicle`) → `MAV_CMD_NAV_TAKEOFF` 到 **BOUNCE_ALT(默认2m)** → 悬停 **BOUNCE_HOVER_SEC(默认2s)** → LAND → 高度 &lt; 0.3m 视为着陆 → `disarm_vehicle` 上锁。
- **原地垂直起飞**: 不飞往其它坐标, 起点=当前 GPS 位置。
- 配置: `BOUNCE_ALT / BOUNCE_HOVER_SEC / BOUNCE_TAKEOFF_TIMEOUT / BOUNCE_LAND_TIMEOUT / BOUNCE_LAND_ALT`。
- 说明: 测试4「起飞」仅 NAV_TAKEOFF 到 `TAKEOFF_ALT=5m` 原地悬停, **不飞往地点**; 要去某点用 `apm_control.py`。

## v1.7.1 - 2026-09-23

### 修复 — 解锁 `COMMAND_ACK=FAILED` 且无 PreArm 文本
- **根因**: 遥控器解锁时人手把油门压到最低; **脚本解锁时油门摇杆常在中位(1500)**, ArduCopter PreArm 报 Throttle high 直接 FAILED。
- **修复** (`arm_vehicle`):
  1. 解锁期间 **RC 覆盖强制油门=1000**, 横滚/俯仰/偏航居中, 成功或失败后 `finally` 释放覆盖;
  2. 被拒后 **再收 2s STATUSTEXT** (飞控常在 ACK 之后才发 PreArm 文本);
  3. 解锁前后打印 **RC 油门 PWM**、**GPS fix/卫星数** 便于对照黄灯;
  4. 覆盖循环中定时重发, 防止 `RC_OVERRIDE` 超时失效。

## v1.7 - 2026-09-23

### 修复 — `apm_control_test.py` 电机解锁无效 / 飞控闪黄灯
- **根因1**: 测试3 模式切换列表以 `RTL` 结尾且不切回, ArduCopter **RTL 模式禁止解锁**, 后续解锁被拒。
  - 测试3 结束强制切回 **GUIDED**。
- **根因2**: `test_02` 不先切可解锁模式, 直接 `arducopter_arm()`, 且不检查 `COMMAND_ACK`。
- **根因3**: `motors_armed_wait()` 无超时, 解锁失败时**死等**, 界面像“没有作用”。
- **根因4**: `connect()` 未设置 `target_system/target_component`。
- 黄灯 = ArduPilot **PreArm 自检未通过**指示; 原脚本不读 `STATUSTEXT`, 看不到原因。
- **修复**:
  - 新增 `arm_vehicle()`: 切 GUIDED/指定模式 → 发 ARM → 校验 ACK + 心跳 ARMED → **打印全部 PreArm/STATUSTEXT** → 超时返回。
  - 新增 `disarm_vehicle()`: 校验 ACK, 拒绝时自动强制上锁(21196)。
  - 新增 `drain_rx()`; `connect()` 按心跳设置目标系统/组件。
  - 测试2/11/12、起飞前均改用上述函数; 起飞前检查是否已解锁。
- 遥控器可正常解锁/上锁, 说明硬件与 PreArm 可通过; 脚本侧需油门摇杆最低 + GUIDED 模式。

## v1.6 - 2026-09-23

### 新增 — WAYPOINT 航点任务 (`apm_control.py --test 4`)
- **WAYPOINT = 航点** (`MAV_CMD_NAV_WAYPOINT=16`); 此前脚本无任务协议, 仅单点 GUIDED。
- 新参数:
  - `--waypoints "lat,lon[,alt];lat,lon[,alt];..."` 多航点串(分号/换行分隔, alt 缺省用 `--altitude`)
  - `--waypoints-file route.txt` 从文件读取(每行 `lat,lon[,alt]`, 支持 `#` 注释), 可与 `--waypoints` 叠加
  - `--mission-end rtl|land|none` 任务结束动作(默认 `rtl`: 末项 `MAV_CMD_NAV_RETURN_TO_LAUNCH`)
  - `--mission-timeout` 任务等待超时(默认 600 秒)
- 新函数: `parse_waypoints()`、`load_waypoints_file()`、`upload_mission()`(MISSION_COUNT/REQUEST/ITEM_INT/ACK 协议)、`wait_auto_mission()`(监视 MISSION_CURRENT/心跳)、`run_test_mission()`。
- 流程: 上传任务 → GUIDED 解锁 → 起飞 → **AUTO** 按序飞航点 → 末项 RTL/LAND → 等待降落上锁。

## v1.5 - 2026-09-23

### 职责划分
- **`apm_control_test.py`**（已验证的测试框架）: 承载全部测试项与 **PID 自动调参配置/流程**。
  - 配置区新增: `AUTOTUNE_ALT` / `AUTOTUNE_TIMEOUT` / `AUTOTUNE_CLIMB_THR` / `AUTOTUNE_SETTLE_THR` / `AUTOTUNE_MIN_SATS` / `AUTOTUNE_LOITER_SETTLE`。
  - 新增菜单项 **12 = PID自动调参**（`test_12_autotune`）: GPS锁星 → ALT_HOLD解锁 → RC油门覆盖爬升 → LOITER稳定 → AUTOTUNE调参（STATUSTEXT进度+完成/失败识别）→ 切回LOITER保存参数 → RTL降落上锁。
  - 新增辅助函数: `wait_gps_lock()`、`release_rc_override()`、`takeoff_althold()`、`wait_autotune()`。
  - 新增菜单项 **13 = 飞机状态**: 采集 HEARTBEAT/GPS_RAW_INT/GLOBAL_POSITION_INT/VFR_HUD/ATTITUDE/SYS_STATUS/EKF_STATUS_REPORT，按 Mission Planner 风格打印模式、解锁、GPS、位置、速度、姿态、电池、CPU、EKF 等（`STATUS_LISTEN` 可配时长）。
  - 新增菜单项 **14 = 电池状态**: 采集 SYS_STATUS/BATTERY_STATUS，打印电压、电流、电量条、温度、电芯电压、已消耗容量等（`BATTERY_LISTEN` 可配时长）。

- **`apm_control.py`**: 专注 GPS 任务飞行，**已移除** PID 调参相关代码（原 --test 4）。
  - 飞往指定 GPS（`--latitude/--longitude`，`--nav-mode speed|position` 保持可选）。
  - **途中目标修正**: 监听同链路地面站 `SET_POSITION_TARGET_GLOBAL_INT`，或终端 stdin 指令:
    - `lat,lon` / `lat,lon,alt` — 修正目标点
    - `hover N` — 更新悬停等待时间
    - `rtl` — 立即返航
  - **到达后悬停**: 默认 `--hover 10` 秒等待窗；期间收到新指令则继续执行（新目标继续飞/时间指令重置窗口）；**超时无任何新指令 → RTL 返航降落上锁**。
  - 巡航/导航超时不再直接退出，改为自动转 RTL，避免飞机滞空。
  - 保留 `--test 1/2/3`（模式切换/解锁/完整任务）。

### 说明
- 遥控器请开机备用；调参时勿打杆。
- 调参轴以固件 `AUTOTUNE_AXES` 为准；`AUTOTUNE_ALT` 建议 5~10 米。

## v1.4 - 2026-09-21

### 新增
- 新增 `--nav-mode` 参数,导航方式二选一:
  - `speed`(默认): 低速巡航最小油门飞行(v1.3 方案)。
  - `position`: 恢复 v1.2 的原"位置指令 + 飞控自主速度规划"方式(`goto()` + `wait_to_reach()`)。
- 恢复 `wait_to_reach()` 函数用于位置规划方式。

## v1.3 - 2026-09-21

### 新增
- 低速(最小油门)巡航飞行:
  - 飞往目标点改为速度指令模式(`SET_POSITION_TARGET_GLOBAL_INT` 仅下发 vx/vy/vz),取代原先的位置+自主速度规划。
  - 新增 `--speed` 参数指定巡航速度,默认 1.0 m/s(接近悬停油门对应的最小可飞速度);速度越小,推进所需油门越小。
  - 新增 `bearing_to()`(方位角计算)、`set_velocity()`(速度指令)、`fly_to_target()`(低速巡航+到达检测),替换原 `wait_to_reach()`。
  - 到达后发送零速度悬停,悬停阶段维持目标点位置。
- 参数校验: `--speed` 必须大于 0 。

### 说明
- GUIDED 模式下油门由飞控自动控制,无法直接用 MAVLink 设定油门百分比;本版本通过"低速巡航"实现等效的最小油门飞行,悬停时即为悬停维持油门。

## v1.2 - 2026-09-21

### 新增
- 新增 `--test` 参数,可单独选择要执行的测试项:
  - `1` = 飞行模式切换测试(连接 + 逐模式切换校验,不解锁)
  - `2` = 解锁测试(切 GUIDED -> 解锁确认 ARMED -> 重新上锁)
  - `3` = 完整任务测试(模式切换 -> 解锁 -> 起飞 -> 飞往 GPS 坐标悬停 -> RTL 返航)[默认]
- 新增 `--no-gps` 参数,可跳过 GPS 锁星等待(用于测试1/2室内快速验证)。
- `--latitude/--longitude` 改为仅测试3必填,测试1/2 无需经纬度,值非法会直接报错。
- 新增辅助函数 `fly_to_and_hover()`、`run_test_modes()`、`run_test_arm()`、`run_test_full()`,按测试项分发执行。

### 变更
- 完整任务测试中,模式切换测试(测试1)在执行解锁前自动完成,全部通过才继续。

## v1.1 - 2026-09-21

### 新增
- 新增 `test_flight_modes()`,连接后逐个切换飞行模式(默认 GUIDED/LOITER/RTL),每项通过心跳 `custom_mode` 校验切换成功,任一失败立即退出,不再继续解锁/起飞。
- 新增 `--test-modes` 参数,逗号分隔自定义测试模式列表,测试结束后自动停在 GUIDED 模式。

### 修复
- 修复起飞失败时 `rel`、飞往目标超时时 `d` 可能未定义导致的崩溃。
- 中文输出在 Windows 控制台 GBK 编码下显示乱码,不影响功能(建议 `chcp 65001` 或 PyCharm 运行)。

## v1.0 - 2026-09-21

### 新增
- 初版脚本,基于 pymavlink 2.4.x 实现 APM/ArduPilot 控制。
- 完整任务流程:
  1. 连接飞控(串口 / UDP / TCP)
  2. 等待 GPS 锁星(3D 定位且 >= 6 颗卫星)
  3. 切换 GUIDED 模式
  4. 解锁 (ARM) 并校验 ARMED 状态
  5. 起飞到指定高度 (`MAV_CMD_NAV_TAKEOFF`)
  6. 发送位置指令 (`SET_POSITION_TARGET_GLOBAL_INT`) 飞往目标 GPS 坐标
  7. 到达后悬停(默认 10 秒,持续发送目标点保持位置)
  8. `MAV_CMD` / `SET_MODE` 切换 RTL 自动返航,等待降落并上锁
- 模式编号对照表 `COPTER_MODE`(ArduCopter 新旧固件通用)。
- 命令均通过 COMMAND_ACK / 心跳校验确认成功,失败即报错退出。
- `Ctrl+C` 中断提示立即用遥控器接管。

## 参数说明(当前)

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `connection` | COM3 | 串口(COM3 等)或 UDP/TCP(tcp:127.0.0.1:5760 等) |
| `--baud` | 115200 | 串口波特率 |
| `--test` | 3 | 测试项: 1=模式切换 / 2=解锁 / 3=单点任务 / 4=航点任务(PID调参见 apm_control_test.py 菜单12) |
| `--latitude` | - | 目标点纬度(测试3必填) |
| `--longitude` | - | 目标点经度(测试3必填) |
| `--waypoints` | - | 航点串 `lat,lon[,alt];...`(测试4) |
| `--waypoints-file` | - | 航点文件路径(测试4, 每行一个点) |
| `--mission-end` | rtl | 任务结束: rtl / land / none |
| `--mission-timeout` | 600 | 任务等待超时(秒) |
| `--altitude` | 20.0 | 飞行高度(米) |
| `--hover` | 10.0 | 到达后悬停等待指令时间(秒), 无新指令则RTL; 运行中可 `hover N` 修改 |
| `--threshold` | 3.0 | 判定到达目标的水平距离误差(米) |
| `--speed` | 1.0 | 巡航速度(米/秒),越小油门越小,仅 speed 导航方式生效 |
| `--nav-mode` | speed | 导航方式: speed=低速巡航最小油门 / position=位置+飞控自主规划 |
| `--test-modes` | GUIDED,LOITER,RTL | 模式切换测试的模式列表 |
| `--no-gps` | False | 跳过 GPS 锁星等待 |

## v1.11 - 2026-09-25

### 新增 13 项功能 · 图形界面 `apm_control_ui.py` (v1.10 增强)
按"做全部"要求一次性加入; 已通过 `py_compile` + GUI 实例化 + 逻辑冒烟测试(围栏/低电/心跳/CSV/航点/报告导出 全部通过)。

**安全**
- **急停 LOITER(快捷键 F9)** + **立即 RTL / 立即 LAND**: GUI 线程直发不排队(`set_mode_apm` 已验证非阻塞), 任务运行中也可用
- **地理围栏**: 离 HOME 距离 / 相对高度双阈值(0=关闭), 越界红字告警, 可选 "仅告警/自动RTL"; 单次触发放行一次, 回到 90% 阈值内自动复位; 仅在已解锁时判定; HOME 支持 HOME_POSITION 自动获取 + 手动设置
- **低电量保护**: 触发电压/电量双阈值 + 独立告警电压(橙色提醒), 可选 "仅告警/自动LAND/自动RTL"; 落地解锁后自动复位
- **心跳丢失监控**: 超时阈值可调(默认 3s), 状态栏红字 + 日志告警, 同类告警 10s 冷却防刷屏
- **飞行日志 CSV 录制**: 5Hz 写入 `flight_logs/flight_时间戳.csv`(27 列: 时间/模式/解锁/位置/速度/姿态/电池/GPS/RC覆盖/最近指令/HOME距离/目标值/飞控消息)

**控制调试**
- **小地图航迹**: HOME(黄框)/当前位置(红点)/航迹(蓝线), 自动比例 + 比例尺; 航迹按 0.5m 或 1s 采样(上限 800 点), 解锁期间记录
- **姿态仪表**: 地平仪(roll 旋转地平线 + pitch 上下平移 + 绿色十字基准)
- **目标跟踪曲线**: 高度/地速 实际(青) vs 目标(红虚线); 起飞记录 target_alt、速度微动记录 target_speed, 解锁→上锁迁移时自动清除
- **参数读写面板**: 常用参数下拉 + PARAM_VALUE 回读 + PARAM_SET 写入(复用 `tct.set_param`) + 写入历史表
- **RC 覆盖滑条**: CH1-4 1000~2000, 5Hz 发送 + 回中按钮; 任务/测试运行中自动释放覆盖并提示(仅地面调试)

**效率**
- **航点任务页**: 文本框逐行解析 `纬度,经度[,高度]`(# 注释) → 校验(航点数+航线总长)/仅上传(`apc.upload_mission`)/一键执行(上传 + 解锁起飞到最高航点 + `apc.wait_auto_mission` 等待 RTL 收尾); 支持加载当前目录 waypoints.txt
- **测试报告导出**: 15 项测试结果(PASS/FAIL/说明) + 遥测快照 → Markdown/文本(filedialog 选路径)
- **遥测消息速率表**: 读线程按 1s 窗口统计各消息类型 Hz; 与控制指令频率表并列于"信号速率"页

- 界面: 顶部新增**安全条**(急停/立即RTL/立即LAND + 围栏/电池/心跳状态灯); 左侧遥测新增 HOME 经纬度、离 HOME 距离、姿态仪表; 中部改为 Notebook 7 页(安全设置/目标曲线/小地图/参数读写/航点任务/RC覆盖/信号速率); 右侧新增 录制CSV/导出报告 按钮
- 更名: `amp_control_ui.py` → **`apm_control_ui.py`**; 复用 `apm_control.py`(任务上传/等待) 与 `apm_control_test.py`(解锁/参数/测试) 不变


## v1.11.1 - 2026-09-25

### 变更 2 小地图升级为高德瓦片底图 (国内可用)
- **原实现**: 纯自绘示意图(网格+点位), 无真实底图, 且必须等 GPS 有坐标才显示内容
- **现实现**:
  - **高德地图瓦片** (`webrd01~04.is.autonavi.com/style=8`): 国内 CDN 直连、无需 API 密钥, 本机实测 0.3s/张
  - **无 GPS 也立即加载**: 默认中心北京天安门, 有 HOME 先显示 HOME, 有 GPS 后可"跟随飞机"
  - **坐标对齐**: GPS 为 WGS-84, 高德底图为 GCJ-02(火星坐标), 内置 `wgs84_to_gcj02` 转换(实测北京偏移约555m, 不转换会明显错位)
  - **瓦片缓存**: 下载后存 `map_tiles/z/x/y.png`(磁盘) + 内存 PhotoImage(300张上限), 用过一次离线也能显示; 下载失败 60s 冷却重试, 不刷屏
  - **交互**: 滚轮/＋－ 缩放(z3~17)、拖拽平移(自动退出跟随)、双击或"回中HOME"回 HOME、"跟随飞机"开关
  - **叠加层**: HOME(黄框)、航迹(蓝线, 复用原采样)、当前位置(红点+十字)、自动比例尺(1/2/5×10^k 取整)
  - 后台单线程下载队列, 不阻塞 GUI; 状态栏显示中心坐标/瓦片进度/下载统计
- 依赖: 仅标准库 `urllib.request`(无新增 pip 依赖)


## v1.12 - 2026-09-25

### 变更 3 小地图底图改为 WGS-84 无偏移方案 (多源可选)
- **背景**: 高德底图为 GCJ-02 火星坐标(固有加密偏移, 实测北京约555m), 按需求更换为 WGS-84 地图
- **新底图源** (小地图页"底图"下拉切换, 磁盘缓存按源分目录 `map_tiles/<源>/`):
  | 源 | 坐标系 | 密钥 | 说明 |
  |---|---|---|---|
  | **Esri卫星影像 (默认)** | WGS-84 | 免 | 卫星图, 本机实测国内直连 1.2s/张; JPEG 瓦片经 **Pillow** 转 PNG 后显示 |
  | 天地图矢量 | WGS-84/CGCS2000 | 免费tk | 国家官方电子地图, 注册 console.tianditu.gov.cn, 密钥填入界面[应用]并自动存 `tianditu_key.txt` |
  | 天地图影像 | WGS-84/CGCS2000 | 免费tk | 国家官方卫星图 |
  | 高德道路 (备选) | GCJ-02 有偏移 | 免 | 保留旧源, 标签已注明有偏移 |
- **坐标对齐**: 仅高德源做 WGS-84→GCJ 转换, Esri/天地图恒等变换(**零偏移**); 状态栏实时显示当前源/坐标系/中心坐标
- **无密钥处理**: 选天地图未填tk时画布中央提示注册地址, 不发无效请求
- **新依赖**: `pip install pillow` (JPEG→PNG; 未安装时状态栏给出提示, 其余源不受影响)
- 实测: Esri 跟随模式中心与 GPS 完全一致(偏移0m); 高德源切换后正确转出554.9m偏移; 密钥持久化/源切换/缩放拖拽回归通过


## v1.13 - 2026-09-26

### 新增 PWM10 抛投器触发 (手动按钮 + 航点任务自动)
- **飞控前提**: `SERVO<通道>_FUNCTION = 0 (Disabled)`, 否则指令被拒(`ServoRelayEvent: Channel x is already in use`); PWM10 = Pixhawk **AUX OUT 2**; 指令 = `MAV_CMD_DO_SET_SERVO(183)` (param1=通道, param2=PWM)
- **`apc.trigger_drop(master, servo_no=10, pwm_on=2000, pwm_off=1000, hold_ms=1500)`**: 实时 COMMAND_LONG 触发 → 保持 hold_ms → 自动回位, 每步校验 COMMAND_ACK + 打印飞控 STATUSTEXT, 被拒提示检查 `SERVO10_FUNCTION=0`; `pwm_off=None` 不回位; 参数校验 通道1~16 / PWM 500~2500
- **`apc.upload_mission()` 混合任务**: 条目支持 `('DROP', 通道, 触发PWM, 回位PWM, 保持ms)`, 展开为 `DO_SET_SERVO(触发)` → `NAV_DELAY(保持秒, 向上取整, 期间悬停)` → `DO_SET_SERVO(回位)`; 纯航点列表行为不变(向后兼容); 成功返回任务项数(供进度对齐), `land` 末点自动取最后一个航点; 新辅助 `_is_drop()`
- **UI 快速控制抛投行**: 通道[10]/触发µs[2000]/回位µs[1000]/保持ms[1500] 可调 + **⚠抛投按钮** (`messagebox` 二次确认)
  - 空闲时走 `_enqueue` 队列(capture 开启, 完整 ACK 校验, 适合地面测试)
  - **任务运行中 GUI 直发不排队**(同 F9 急停先例): 排队会任务结束落地后才触发(时机错误); 回位用 `root.after` 定时, 重复点击先 `after_cancel`, 断开连接自动取消
  - 拒收 STATUSTEXT 经遥测状态行实时显示
- **航点任务页 DROP 行**: `DROP[,通道,触发PWM,回位PWM,保持ms]`, 缺省项用界面抛投配置(回位留空=不回位); 校验: 首行不能 DROP / 通道 1~16 / PWM 500~2500 / 保持 0~60000ms; 校验/上传/执行均按"航点数+抛投点"分类计数, 进度按展开后总项数对齐
- **参数页**: 下拉新增 `SERVO10_FUNCTION`, 提示"抛投前提 SERVO10_FUNCTION=0"
- **不加入 TESTS**: 抛投不进"运行全部测试", 避免连带误触发
- **已知取舍**: 任务内保持为整秒(NAV_DELAY 限制, 向上取整); 手动触发保持精确 ms
- **验证**: `py_compile` + `pyflakes`(仅既有 `up` 警告) + 新增 `smoke_drop.py`(trigger_drop ACK/被拒/参数校验、纯航点回归、DROP 展开序列 SERVO→DELAY(2s)→SERVO、UI 解析/校验/确认框/入队/直发+回位, 全过) + 回归 `smoke_ui2`/`smoke_map2` 全过


## v1.14 - 2026-09-27

### 修复 — UI 测试3无输出 + 断开后无法重连 (两处根因)

| 问题 | 根因 | 修复 |
|------|------|------|
| 测试3(模式切换)无输出/无效果, 命令行与 UI 日志均无切换结果 | `MasterProxy.__getattr__`/`__setattr__`/`recv_match` 三处误写 **`object.__getattr__`**(Python 无此 API), 一切透传属性访问(`set_mode_apm`/`mode_mapping`/`target_system`...)与 `recv_match` 均抛 `AttributeError: type object 'object' has no attribute '__getattr__'` | 三处改为 **`object.__getattribute__`**; 此前 `smoke_drop` 用独立 MockMaster 绕过 MasterProxy, 故未检出 |
| 断开后无法重连, Mission Planner 也连不上, 必须拔插 USB | `on_disconnect`/`_on_close`/**连接失败路径** 均未调用 `raw.close()`, COM 句柄一直被占用 | 断开/退出: 先停读线程(`join` ≤1.5s)再 **`raw.close()`**; `do()` 失败路径 `finally` 关闭并清 `raw/proxy` |
| 断开后立即重连卡在"连接中...", 无任何输出 | `_worker_loop` 空队列 0.1s 后若 `reader_running=False` 直接退出; 此时"连接"任务入队但 worker 已(将)死 → **孤儿任务永无人执行** | worker 改为**常驻不退出**, `on_connect` 保留首次启动/兜底重启 |
| 快速重连后可能出现双读线程抢同一路消息 | 旧 reader 线程若在 `recv_match` 0.3s 超时内, 重连把 `reader_running` 置回 True 后它会继续活下来 | 读循环加**线程身份校验** `self.reader_thread is me`; `do()` 启新 reader 前 `join` 旧线程 |
| 连接成功但日志打印"连接: ❌ 失败" | `wrap()` 无返回值, `_worker_loop` 记 False | 补 `return ok` |

- **验证**: 假飞控全链路复现/回归(真实 UI 管线: reader 线程 + MasterProxy + MavSendProxy + 工作线程): 测试3 六次 `DO_SET_MODE` 全部 ✅ 并回显切换结果; 断开→立即重连 ×5 全通(0 孤儿任务); 断开后 0 句柄泄漏; 回归 `py_compile` + `pyflakes`(仅既有 `up` 警告) + `smoke_ui2`/`smoke_drop`/`smoke_map2` 全过
## v1.15 - 2026-09-28

### 新增 航点页相对航点 + 单点飞行 + 遥控器接管保障 (apm_control_ui.py / apm_control_test.py / apm_control.py)

| 项 | 说明 |
|----|------|
| 航点页 `@` 相对行 | 每行支持 `@东米,北米[,高度]`（缺省 20m，限 ±100000m），以飞机 GPS 为原点；解析重构为公共 `_parse_wp_lines` + 文本框包装 `_parse_wp_text`；页头语法提示同步更新 |
| 校验/上传/执行换算 | 校验: 有 GPS 时按原点换算并显示航线总长，无 GPS 提示"执行时按飞机GPS换算"；上传/执行: worker 内 `_fresh_pos`(GLOBAL_POSITION_INT, 3s 超时) 取新鲜原点，经 `_resolve_items`/`_resolve_rel` 换算成绝对坐标，取不到 GPS 则 ❌ 失败（相对数值绝不误传飞控） |
| 单点飞行面板 | 航点页顶部: 目的地 `纬度,经度` 或 `@东x,北y` + 高度框 + [起飞并飞往] / [仅飞往(已解锁)]；`on_goto_point`: 未解锁 → (延长 DISARM_DELAY → GUIDED 解锁 → NAV_TAKEOFF → 到位)，已解锁 → 确保 GUIDED → 1Hz 重发 `apc.goto` → 轮询 GLOBAL_POSITION_INT 直到水平<2.5m 且垂直<1.5m，超时 max(60, 30+d0/2)s，到达后保持 GUIDED 悬停 |
| 遥控器接管保障 | 实测"只打摇杆不拨模式开关"飞机不响应 = GUIDED 下摇杆无效（ArduPilot 设计）；① 断开/退出前 `_release_rc_override()` 发 `rc_channels_override` 8 通道全 0（MAVLink 规范 0=交还遥控器，且在 `raw.close()` 之前），防残留覆盖挡住接管；② `_hint_takeover()` 在起飞完成/单点到达后提示"拨 TX 模式开关到 LOITER(或按 F9 急停)"；③ CLI 同步提示: 测试4 成功分支、`apm_control.py` `takeoff()` 到位后打印同样接管说明 |
| 修复(顺手) | ① `on_goto_point` 解锁起飞路径原重复调用 `ensure_disarm_delay`（2×2.5s 参数写超时），改为只走 `arm_and_takeoff(set_delay=True)` 一次；② `on_wp_run` 起飞高度对 `@` 行错取北向偏移 dy 当高度（REL 元组 [3] 才是高度）→ GUI 估算按类型取 [3]/[2]，fn 内换算后按绝对点重算 takeoff_alt `ensure_disarm_delay`（2×2.5s 参数写超时），改为只走 `arm_and_takeoff(set_delay=True)` 一次 |

- **验证**: 新增冒烟 `smoke_wp_rel.py`（FakeFC 全链路，%TEMP%\opencode）51 项断言全绿: 解析/数值换算/校验两分支/单点绝对+相对+未解锁自动起飞/任务上传 `@` 行换算(6 项 ACK)/断开 RC 释放帧+句柄关闭；回归 `repro_test3`(test3 六次 DO_SET_MODE 全✅、断开立即重连×5 全通 0 孤儿、0 句柄泄漏) + `smoke_ui2` + `smoke_drop` + `smoke_map2` 全绿；`py_compile` 三文件 + pyflakes 无新增警告；测试残留已清理（tianditu_key.txt、flight_20260928_*.csv）
## v1.15.1 - 2026-09-28

### 布局调整: 姿态仪表恢复可见, 日志移到中列底部 (apm_control_ui.py)

| 项 | 说明 |
|----|------|
| 问题 | 左栏内容需求高 1007px, 窗口仅容 766~960px → 姿态仪表(最后一行)整块落在窗口外"被遮挡"; 全宽日志行又与左栏抢高度 |
| 左栏通底 | `实时状态/遥测` 框 `rowspan=2` 跨满两行, 底部腾出的左下区域正好放姿态仪表; 右侧 `功能测试` 框同样 `rowspan=2` |
| 日志居中底部 | `日志/飞控反馈` 从"跨全宽 row1"改为 `row=1, column=1` (仅中列), 位于快速控制/功能页正下方, 即"中间底部" |
| 左栏瘦身 ~123px | 遥测行 pady 1→0、段标题 pady 6→3、曲线/姿态标题 pady 8→4、高度曲线 90→70px、姿态仪 150→120px、框 padding 6→4 → 需求高 1007→884 |
| 默认窗口 | 1360x840 → **1360x980** (姿态仪底部 y≈954, 默认余量 26px; 1080p 最大化余量 ≈80px) |

- 绘制自适应: `_draw_attitude`/`_draw_alt` 均读画布 width/height 配置, 无需改绘制代码
- **验证**: 截图目检(左下地平仪 R+18/P-7 完整、曲线/日志/右栏正常) + 几何测量(980/1010/1035 窗口姿态仪可见, 840/900 窗口裁剪) + `py_compile`/`pyflakes`(仅既有 `up`) + `smoke_ui2` + `smoke_wp_rel` 全绿
## v1.15.2 - 2026-09-29

### 修复 航点任务上传被拒 MAV_MISSION_RESULT=9 (apm_control.py)

| 项 | 说明 |
|----|------|
| 现象 | 实机 [仅上传/执行] → "任务上传被拒绝, MAV_MISSION_RESULT=9"; 4 项全部发完(1/4~4/4)才被拒 |
| 根因 | 9 = **MAV_MISSION_INVALID_PARAM4**; ArduPilot 对每个任务项做 NaN 参数豁免校验(AP_Mission sanity check), 结束项 **RTL 的 param4=NaN 不在豁免内**(固件早于 ardupilot#31658) → 整份任务被拒; 航点项 param4=NaN(保持偏航)被接受, 所以 3 个航点全部收到后才在 RTL 项报错 |
| 修复 | RTL / LAND / DO_SET_SERVO(抛投) 条目 param4 `NaN` → **0.0** (有限值任何固件版本都通过校验); NAV_WAYPOINT param4 保留 NaN(语义=不改变偏航, 已被实机接受) |
| 诊断改进 | 上传拒绝时打印错误码名称: `MAV_MISSION_RESULT=9 INVALID_PARAM4 (param4无效: 该项 param4 不可为 NaN)` |

- **验证**: `smoke_wp_rel` P5 新增 2 项断言(非航点条目 param4 必须有限值 / RTL 末项 param4=0.0、cmd=20) 全绿; `py_compile` + `pyflakes` 无警告
## v1.16 - 2026-09-29

### 单点飞行新增"巡航m/s" (apm_control_ui.py)

| 项 | 说明 |
|----|------|
| 背景 | "速度微动 m/s" 只管手动微动按钮的速度指令; 单点飞行 goto 是位置指令, 速度由飞控参数 WPNAV_SPEED(cm/s) 决定, 想全速飞只能去参数页手改 |
| 新增 | 单点飞行行新增 **巡航m/s** 输入框: 点[起飞并飞往/仅飞往]时若已填, 自动 `PARAM_SET WPNAV_SPEED = m/s × 100` 再开始飞 (写入即生效, 0.2~20 m/s = 20~2000 cm/s); **留空 = 不动飞控参数** |
| 失败降级 | 写入失败(超时/旧固件)仅提示 ⚠️, 按飞控现有 WPNAV_SPEED 继续飞行, 不中断任务 |
| 界面微调 | 航点任务页顶部改为两行: 按钮行(校验/仅上传/执行) + 格式说明行; 修复默认 1360 宽度下三个任务按钮被格式长文本顶出可视区、格式行被截断的问题; 单点行目的地框 32→24 字符 |

- **验证**: `smoke_wp_rel` 新增 3 项断言 (巡航10 → PARAM_SET WPNAV_SPEED=1000 ✓; 留空 → 不发 PARAM_SET ✓; FakeFC 新增 param_set→PARAM_VALUE 回显) 全绿; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; 截图+几何测量: 单点行 628px ≤ 可用 641px, 三按钮/格式行/巡航框全部可见; `py_compile` + `pyflakes` 无新警告
## v1.16.1 - 2026-09-29

### 读取飞控速度参数: 巡航上限 + 悬停油门 (轻量版)

| 项 | 说明 |
|----|------|
| 新增按钮 | 航点任务页 **[读飞控速度参数]**: 一次读回 `WPNAV_SPEED`(当前巡航指令上限) + `THROTTLE_HOVER`(悬停油门, 全速潜力基准), 显示在按钮右侧, 同时写入日志 |
| 显示格式 | `巡航上限 WPNAV_SPEED=1200 cm/s (12.0 m/s) | 悬停油门 THROTTLE_HOVER=45%` (THROTTLE_HOVER ≤100 按 % 显示, 否则按 PWM) |
| 钳制 | 巡航m/s 输入已在客户端校验 0.2~20 m/s (=2000cm/s 固件上限); 超范围直接拒绝并提示 |
| 说明 | 参数范围编译在固件里, MAVLink 参数协议不传 min/max, 故按 Copter 公开范围钳制; WPNAV_SPEED 只是"指令上限", 飞机达不到时油门饱和速度卡住 —— 真实上限需实测(中/重量版待做) |

- **验证**: `smoke_wp_rel` 新增 P4c 3 断言 (读回 WPNAV_SPEED=1000cm/s=10.0m/s ✓, THROTTLE_HOVER=45% ✓, 写入后参数持久可读 ✓) 全绿; FakeFC 新增 `param_request_read` 回显 + `params` 字典; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; 截图目检新行完整; `py_compile` + `pyflakes` 无新警告
## v1.16.2 - 2026-09-29

### 取消"9. 航点飞行"测试项, 测试序列重编为 1~14

| 项 | 说明 |
|----|------|
| 移除 | 功能测试区 **9. 航点飞行** 按钮 + CLI 菜单项 + "运行全部测试" 不再执行该项. 原因: 航点任务页(校验/仅上传/执行 + 单点飞行)已完整覆盖, 且该按钮不便设置 GPS 位置 |
| 删除 | `test_09_waypoint` 函数(向北偏移10m 的固定逻辑)整体移除 |
| 重编号 | 原 10~15 顺延为 **9~14**: 9自动降落 / 10锁定电机 / 11PID自动调参 / 12飞机状态 / 13电池状态 / 14低空起降悬停; 函数名同步 `test_09_land`~`test_14_bounce`, 所有打印/注释/交叉提示(如"可试 测试9 降落 / 测试10 上锁")一并更新 |
| 同步 | `apm_control_ui.py` 头注释 "测试1~15" → "测试1~14"; 测试框架头说明移除"航点飞行"; 面板 14 按钮刚好 7 整行 |

- **验证**: `TESTS` 导入 = 1~14 共 14 项, 无残留 `test_09_waypoint`/旧函数名引用; `py_compile` + `pyflakes` 无新警告(删除函数还消除了 1 条旧 f-string 警告); 回归 `smoke_ui2`/`repro_test3`/`smoke_drop`/`smoke_wp_rel` 全绿; 截图目检按钮区 1~14 连续编号、航点飞行已消失
## v1.17 - 2026-09-29

### 飞控日志下载: 最后一次飞行日志 / 全部日志

| 项 | 说明 |
|----|------|
| 新增按钮 | 功能测试区 **[⬇ 下载最后飞行日志] / [⬇ 下载全部飞控日志]** (录制飞行日志(CSV) 下方): 从飞控 DataFlash 下载原始日志 |
| 协议 | MAVLink 经典日志协议(兼容旧固件): `LOG_REQUEST_LIST(0~0xFFFF)` → 收 `LOG_ENTRY` 清单(id/num_logs/last_log_num/size) → 逐条 `LOG_REQUEST_DATA` → `LOG_DATA` 90B 分块组包; 断流 3s 自动从第一个缺口重发请求(最多5次) |
| 完整性 | 全部字节到齐且**连续组装校验通过才落盘**, 不完整直接丢弃并报 `❌ 日志 #N 下载失败 (got/size)` |
| 保存 | `flight_logs/flash_log_<id>_<时间戳>.bin` (DataFlash 原始格式); 单条超时 = max(180s, 大小/4096); "最后一条"取 last_log_num |
| 进度 | 日志窗实时显示: 清单条数、每条大小、每 64KB 进度、`✅ 已保存 ...`; 作为 worker 任务入队, 与其它功能排队一致 |
| 实现 | `apm_control_test.py`: `fetch_log_list` / `fetch_one_log` / `download_flash_logs`; `apm_control_ui.py`: `on_log_fetch(only_last)` + 两按钮 |

- **验证**: `smoke_wp_rel` 新增 P7 12 断言 (清单请求 ✓ / 只下最后一条 008 且内容逐字节一致 1505B ✓ / 全部 007+008 一致 ✓ / 3 次数据请求 ✓ / 日志窗显示"已保存/已下载1个" ✓ / 临时 bin 清理 ✓) 全绿; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; 几何: 右面板 reqh=544 ≤ body 906 (22 控件)、单点行 628≤641; `py_compile`+`pyflakes` 无新警告

## v1.18 - 2026-09-30

### PID 自动调参 (测试11) 内部重构 + 冒烟覆盖

| 项 | 说明 |
|----|------|
| 实现 | `apm_control_test.py`: 测试11 主体抽出为 `auto_tune_flow(master, alt=None)` (流程/打印不变, 高度参数化), `test_11_autotune` 改薄包装直接调用; 用户可见入口唯一: **功能测试 11. PID自动调参** |
| 去重 | 曾在快速控制新增"🎛 一键自动调参"按钮, 经确认与测试11 调同一 `auto_tune_flow` 完全重复, 已移除按钮与 `on_auto_tune()`, 保留测试11 为唯一入口 (测试编号 1~14 不变) |
| 流程 (测试11) | GPS 锁定检查(≥6星/fix3) → 切 ALT_HOLD → 解锁 → RC 油门爬升至 5m → LOITER 稳定 3s → AUTOTUNE(hb 校验) → 等 STATUSTEXT 完成信号(1800s 超时) → LOITER 保存参数 → RTL 返航降落(≤120s) → 自动上锁 |

- **验证**: 冒烟 FakeFC 扩展: `MODES` 增 AUTOTUNE:15、`GPS_RAW_INT` 1Hz(fix3/12星)、AUTOTUNE 期间每 hb 推 `STATUSTEXT "AutoTune: complete"`、ALT_HOLD 下按 ch3 驱动爬升(≥1600→climb 否则保持)、RTL/LAND 进入时下降动画(补齐只升不降)、DISARM 0 命令回 ACK; `smoke_wp_rel` 新增 **P8** (经功能测试11入口) 8 断言全绿: 完成信号/已着陆/已上锁/任务标签 `11. PID自动调参: ✅ 通过`/fc 已解锁/模式序列 `[2,2,5,15,5,6]` (ALT_HOLD→ALT_HOLD→LOITER→AUTOTUNE→LOITER→RTL)/终点在地面 rel=0.0, 全流程 16s; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; `py_compile`+`pyflakes` 无新增警告

## v1.19 - 2026-09-30

### 软着陆: LAND/RTL 最后 1m 油门微调缓降 (防触地过快损坏飞机)

| 项 | 说明 |
|----|------|
| 新函数 | `apm_control_test.py` **`land_soft_final(master, phase0_timeout)`**: ① 高度 >1m 维持原模式(LAND/RTL)等飞控缓降 ② ≤1m 切 ALT_HOLD, 0.2s 节拍发 RC 油门**小步微调** (降速>0.6→+15, >0.35→+8, <0.05→-10; 限幅 1300~1520) 把降速压向 **0.25m/s** ③ 触地判定 (高度≤0.15m 且降速平稳, 连续2拍) 后: 锁最低油门 → DISARM (被拒自动强制) → 释放 RC 覆盖; 等待接管/缓降超时则释放覆盖交还控制权且不上锁 |
| 接入 | 测试9 自动降落 / 测试14 低空起降落段 / 测试11 RTL 返航末段 / 快速控制 **降落 LAND**、**返航 RTL** 按钮 / `apm_control.py` **`wait_landed`** (单点飞行超时返航、航点任务 rtl/land/none 收尾统一走软着陆); 降落按钮落地后现在会自动上锁 (原实现落地后电机保持解锁) |
| 配置 | 常量可调: `SOFT_LAND_ALT=1.0` `SOFT_LAND_RATE=0.25` `SOFT_LAND_THR_START=1480` `SOFT_LAND_THR_MIN/MAX=1300/1520` `SOFT_LAND_GROUND_ALT=0.15` `SOFT_LAND_SETTLE_T=45` |
| 防误判 | 接管/触地判定只信有消息的读数 (recv None 跳过), 通信中断不会误判触地锁桨 |

- **验证**: 冒烟 FakeFC 扩展 (RTL/LAND 缓降速率 0.5m/s; ALT_HOLD 油门分档 ≥1600 爬升 / 1400~1599 保持 / <1400 缓降); `smoke_wp_rel` **P8 升级** (RTL→ALT_HOLD 接管模式序列尾 `[6,2]`、接管后微调帧 ≥3) + 新增 **P9** 降落按钮软着陆 7 断言 (完成日志/任务标签/已解锁/模式 `[6,2,9,2]` LAND→ALT_HOLD 接管/微调帧 x39/终点在地面) 全绿, P9 实录缓降 0.25m/s; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; `pyflakes` 8 条均既有 (测试9 一处 f-string 无占位符随重写消除, 无新增)

## v1.20 - 2026-09-30

### 无遥控器纯软件控制解锁 (没有遥控器也能 ARM)

| 项 | 说明 |
|----|------|
| 症状 | 无遥控器时解锁被拒: `COMMAND_ACK=FAILED` + `Arm: Need Position Estimate` (EKF 无位置估计) + `Radio Failsafe - Disarming` (RC 失效保护触发, 信号丢失即拦解锁/断桨) |
| 新函数 | `apm_control_test.py` **`ensure_rc_failsafe_off(master)`**: 写 `FS_THR_ENABLE=0` 关闭遥控器失效保护, 无信号时不再拦截解锁; 写失败仅警告仍继续 (旧固件参数名可能是 THR_FS_ENABLE, 提示手动设置) |
| 解锁重试 | **`arm_vehicle`** 新增 `retries=ARM_RETRY(2)`: 被拒且拒因为 GPS/EKF 未就绪 (position estimate/gps/ekf/home/hdop…) 时, `wait_gps_lock` 等 3D 定位 (`ARM_RETRY_GPS_WAIT=30`s) 后自动重发 ARM, 最多 2 次; 解锁失败的"排查顺序"扩为 6 条 (Need Position Estimate 到室外等 20~60s/FS_THR_ENABLE=0/无遥控器专项) |
| 接入点 | 自动关失效保护发生在 `arm_vehicle` 内部, 故 GUI 快速控件解锁 ARM、测试2/4/11/14、`arm_and_takeoff`、`apm_control.py` **`arm()`** (命令行/航点任务) 全覆盖; 已解锁时早退不重复写参数 |
| 配置 | `NO_RC_MODE=1` (0=解锁前不改飞控参数) / `ARM_RETRY=2` / `ARM_RETRY_GPS_WAIT=30` / `ARM_RETRY_KEYS` (可重试拒因关键词) |

- **验证**: `smoke_wp_rel` P8 新增断言 (经流程解锁写 `FS_THR_ENABLE=0`) + 新增 **P10 解锁重试**: FakeFC 首次 ARM 拒绝并回 `Arm: Need Position Estimate` → 软件等 GPS 就绪自动重试成功 (首拒/自动重试日志/ARMED/拒因消费/任务标签 `2. 解锁电机: ✅ 通过`/参数回读/测试10 收尾上锁, 12.2s) 全绿; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; `pyflakes` 8→6 条 (last_text/rejected 随重试逻辑变为有用变量, 余 6 条均既有) 无新增。
- **实机指引**: ① 到室外等 GPS 3D 定位+EKF 就绪 20~60s 再解锁 (软件会自动等待重试); ② `FS_THR_ENABLE=0` 由软件解锁时自动写入; ③ 若仍报 `Radio Failsafe` → 参数没生效, 在 Mission Planner 全部参数搜 FS_THR_ENABLE 手动设 0; ④ 室内无 GPS 无法建立位置估计, GUIDED/航点飞行必须有定位。

## v1.21 - 2026-09-30

### 发射箱纯软件飞行 (无遥控器, 禁飞控自动返航/降落, GPS失联惯导继续)

| 项 | 说明 |
|----|------|
| 参数预设 | `apm_control_test.py` 新增 **`get_param`/`ensure_param`/`ensure_launch_params`**: 解锁前自动确保 9 项参数=0 — `FS_THR_ENABLE`(无遥控器RC失效)/`FS_GCS_ENABLE`(GCS断链)/`FS_DR_ENABLE`+`FS_DR_TIMEOUT`(**GPS失联不返航**,惯导继续)/`FS_EKF_ACTION`(**EKF失效仅报告**,默认1=自动降落)/`FENCE_ENABLE`(飞控围栏)/`BRD_SAFETYENABLE`(发射箱无安全开关)/`BATT_FS_LOW_ACT`+`BATT_FS_CRT_ACT`(低电仅告警). **先读回、值相符跳过写入**, 单项失败仅警告继续; 配置 `LAUNCH_PRESET=1` + `LAUNCH_PARAMS` 表 (v1.20 的 `ensure_rc_failsafe_off`/`NO_RC_MODE` 并入删除, `apm_control.py arm()` 同步改) |
| 超时策略 | `apm_control.py` 新增 **`ON_TIMEOUT='hold'`**: `fly_to_target`/`wait_to_reach` 超时由自动返航改为**悬停保持等软件指令**; `fly_to_and_hover` 悬停环 hold 模式无限悬停(5s报告'自动返航已禁用'), 显式 `rtl` 指令照常立即返航; `'rtl'` 恢复旧行为. `--mission-end` 默认 `rtl`→**`none`**(停末航点保持, 可显式选 rtl/land) |
| 解锁重试 | 新增 **`wait_ekf_position`**: 重试前除 GPS 定位外再等 `EKF_STATUS_REPORT` 水平(0x02)+垂直(0x08)位置标志就绪 (5s无该消息按旧固件放行), 解决实机 GPS 已锁但 EKF 未就绪导致 `Need Position Estimate` 重试秒发打光次数的问题 |
| GPS失联显示 | GUI 空中 fix<3 时状态栏显示 **'GPS失联·惯导继续'** 并节流10s日志 ⚠️ (仅提示不动作); 单点飞行继续 1Hz 发目标由飞控惯导飞向目标 (原行为, 不中止) |
| 文档 | `apm_control.py` 模块头/`--test`/`run_test_full`/`fly_to_and_hover` docstring 与新策略同步 |

- **验证**: `smoke_wp_rel` FakeFC 预置 9 项**不安全默认值**(1/2/10) 走通"读→差异→写"完整路径; P8 断言升级为**9 项参数全写出=0** (+9) + 预设日志入窗断言; FakeMav 新增 `EKF_STATUS_REPORT` 1Hz (flags=0x0A); P10 断言新增 **'EKF 位置估计就绪'** (重试走 EKF 就绪快路径, 12.2s) — 全绿; 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; `pyflakes` 基线 6 条均既有无新增; 临时文件清理 0 残留。
- **实机注意**: ① 惯导飞行时长受固件物理限制 — 默认约 7~15s, 配风阻估计 `EK3_DRAG_BCOEF_X/Y/MCOEF`(按机架调) 可到约 45s, 发射箱场景需实测允许的 GPS 失联距离; ② 老固件缺 `FS_DR_TIMEOUT`/`BATT_FS_*` 等参数时预设按单项警告继续, 需在 Mission Planner 手动设 0; ③ `BRD_SAFETYENABLE` 写入后安全开关灯应即灭, 若仍亮重启飞控一次。

## v1.22 - 2026-09-30

### 重试解锁人工确认 (安全: 防测试人员贴近时机身突然自解锁)

| 项 | 说明 |
|----|------|
| 背景 | 解锁被拒(位置估计未就绪)后软件会等待 GPS/EKF 就绪再重发 ARM — 重试可能在点击后数十秒才满足条件, 此时人员可能已走近飞机, 自动解锁有危险 |
| 新函数 | `apm_control_test.py` **`confirm_arm(reason)`**: 就绪后醒目横幅 (🚨 检测一切正常, 即将解锁电机! / ⚠️ 请所有人员远离飞机) + **等待操作员确认**才继续 ARM; 取消/超时/钩子异常一律不解锁 (安全兜底); 终端(无GUI)走 `input('y 回车确认')` |
| 钩子 | 模块级 **`ARM_CONFIRM_HOOK`**(callable(reason)->bool) 由 GUI 注入; 配置 **`ARM_CONFIRM_TIMEOUT=120`** 秒 |
| 接入 | `arm_vehicle` 重试路径: `wait_gps_lock` → `wait_ekf_position` → **`confirm_arm`** → 确认后才递归重发 ARM; 首次解锁(操作员刚点击)不弹确认; 已解锁早退不弹 |
| GUI | `apm_control_ui.py` **`_arm_confirm`** (工作线程) 经 `root.after(0,...)` 在主线程弹**置顶模态确认框** (`_arm_confirm_dialog`): 红色大字告警 + 远离提示 + 事由, 【✅确认,可以解锁】/【⛔取消】, 关窗=取消; 代际计数防过期弹窗误确认; **超时自动关框取消** (`_close_arm_dlg`); 日志同步横幅. `arm_confirm_auto=True` 可跳过人工确认 (冒烟/自动化专用, 默认 False) |
| 日志 | 确认/取消/超时均有明确日志 (`✅ 测试人员已确认解锁` / `⛔ 测试人员取消解锁` / `⛔ 确认超时(120s)`), 任务以失败标签结束不解锁 |

- **验证**: `smoke_wp_rel` P10 新增断言 (确认横幅 `即将解锁电机` + `自动确认模式` 日志) + 新增 **P11 确认被拒**: 首拒→就绪→钩子返回 False → `未确认, 取消解锁` + **电机保持上锁** + 任务标签 `2. 解锁电机: ❌ 失败`, 恢复钩子; P1~P11 + P6 全绿 (`SMOKE_WP_REL_OK`); 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; `pyflakes` 基线 6 条无新增; 临时文件清理 0 残留。
- **实机流程**: 解锁被拒 → 日志提示等待就绪 → GPS+EKF 就绪 → **屏幕弹置顶确认框** → 测试人员确认现场无人靠近 → 点【确认解锁】→ 解锁+后续任务继续; 120s 未响应自动取消。

## v1.23 - 2026-09-30

### 软遥控 (无遥控器: 屏幕虚拟摇杆/键盘经 MAVLink RC 覆盖打杆, 仅手动模式应急接管)

| 项 | 说明 |
|----|------|
| 协议 | MAVLink `RC_CHANNELS_OVERRIDE`(#70) 逐通道 PWM (既有滑条/解锁压油门即走此协议); 输入源新增 **屏幕虚拟摇杆**(左=油门/偏航 右=俯仰/横滚, 拖动即时打杆, 右杆松手回中/左杆油门锁存) + **键盘 FPS式 WASD** (W/S俯仰 A/D横滚 Q/E偏航 Shift/Ctrl油门升降 R回中, 焦点在输入框时不拦截); 限幅 横滚/俯仰 ±250us(1250~1750) 偏航 ±300us(1200~1800), 方向可经 `SOFT_RC_INVERT` 翻转 |
| 中立帧收尾 | 无接收机 + `RC_OVERRIDE_TIME=-1` 下全0(交还遥控器)=无效PWM会空中掉高 → `tct.release_rc_override()` 改为按 `motors_armed()` 自动选 **`RC_NEUTRAL_GROUND`(1500,1500,1000,1500)** / **`RC_NEUTRAL_AIR`(1500,1500,1500,1500=油门悬停)**, 连发3帧防丢包; apu 断开/退出/停用路径与 `apm_control.py arm()` 内联全0 全部改走该 helper |
| 参数预设 | `LAUNCH_PARAMS` 9→**11 项** (+`RC_OVERRIDE_TIME=-1` 覆盖永不过期 / +`RC_OPTIONS=0` 忽略GCS覆盖位关闭); `ensure_launch_params` 尾部只读校验 `MAV_GCS_SYSID` 与本端 source_system 不一致仅警告(不写入) |
| 启用流程 | 勾选启用 → 排队 worker 任务 `软遥控准备(参数)`(ensure 两参数) → 完成回调才真正打开: 代际计数防"停用后被未完成任务重开", 回调等 action_busy 释放防刚启用就被"任务中自动停用"误杀; 启用期间 20Hz 专职 `root.after` 发送循环 + 摇杆/按键事件立即发 (`SOFT_RC_RATE_HZ`) |
| 模式门控 | 仅 `SOFT_RC_MANUAL_MODES`(STABILIZE/ACRO/ALT_HOLD/LOITER/POSHOLD/DRIFT/SPORT) 下发送; 启用后 5s 宽限等模式切手动; 非手动→自动中立帧停用+日志(含接管提示); 任务运行中(action_busy)自动停用; 面板实时显示 `模式: XX ✓手动(摇杆有效)/✗非手动(摇杆无效)` |
| 一键接管 | 软遥控页 `[一键接管(LOITER+回中)]`: 四杆回中(油门1500=空中安全) + 直发切 LOITER + 启用; F9 急停提示文案同步更新 (F9切LOITER / 一键接管 / TX模式开关) |
| 状态显示 | `MavSendProxy` 活性判定排除全0与两中立帧 (否则中立收尾后状态栏永久"覆盖中"), 文案 `杆值覆盖中/未覆盖`; 标签页 `RC覆盖` 更名 `软遥控` |

- **验证**: `smoke_wp_rel` UNSAFE_PARAMS 扩 11 项走"读→差异→写"路径, P8 断言升级 11 项按期望值写出 (`RC_OVERRIDE_TIME=-1` 其余=0, FakeFC 预置 `MAV_GCS_SYSID=255`); 新增 **P12 软遥控**: 参数准备任务写出两参数 → 键盘打杆 D→横滚 1750 帧/松手回中 1500 → 切 GUIDED 门控自动停用 + **空中中立帧×3**(armed) + 中立帧不计为激活覆盖 → 上锁态 `_release_rc_override` **地面中立帧×3**; **P6 断言改中立帧收尾**(不再全0); P1~P12+P6 全绿 (`SMOKE_WP_REL_OK`); 回归 `smoke_ui2`/`smoke_drop`/`repro_test3`/`smoke_map2` 全绿; `pyflakes` 基线 6 条均既有无新增; 临时文件清理 0 残留
- **实机注意**: 打杆方向按 ArduPilot 常规约定 (W前=1250 / D右=1750 / E偏航右=1800); 若实机方向相反改 `apm_control_ui.py` 顶部 `SOFT_RC_INVERT` 对应项=-1; 软遥控仅手动模式生效, 应急接管路径=F9(切LOITER) 或 [一键接管]; `RC_OVERRIDE_TIME=-1` 下停发不回退遥控器, 故停用/断开/退出一律中立帧收尾, 软件异常退出时飞控保持最后帧 (启用前摇杆/滑条位置即安全基准)

## v1.24 - 2026-09-30

### 测试2 室内解锁失败根因修复 (诚实横幅 + GUIDED→STABILIZE 降级 + 参数/EKF 判定修正)

| 项 | 说明 |
|----|------|
| 根因确证 | 实机 (WFG100 固件, 桨已拆) 室内测试2失败 = `Arm: Need Position Estimate`: GUIDED 解锁需 EKF 位置, 室内 `EKF_STATUS_REPORT flags=0x00A7` 恒定 (有姿态/速度/垂直位置, 但**无水平位置 + CONST_POS_MODE**, GPS fix=3 eph=83 多径不融合); 遥控器可解锁因非位置模式不查 EKF; 电机/遥控/写参硬件均正常; `BRD_SAFETYENABLE` 固件**无此参数**(假警报); `RC_OVERRIDE_TIME=-1` 实测真实生效 |
| EKF 掩码修正 | `EKF_POS_READY_FLAGS=0x0A` 语义全错 → 按官方 `EKF_STATUS_FLAGS` 改为 `EKF_POS_HORIZ_FLAGS=0x18`(相对8\|绝对16) + `EKF_POS_VERT_FLAGS=0x60`(绝对32\|对地64) + 排除 `EKF_CONST_POS_FLAG=0x80`(恒定位置=无效), 新 helper **`ekf_pos_ready(flags)`**; 实机 0xA7 正确判未就绪, FakeFC 默认 `0x2F` 就绪 |
| 重试路径 | `arm_vehicle` 重试前**检查 `wait_ekf_position` 返回值** (旧代码忽略→未就绪仍弹🚨): 就绪→原模式确认后重试; **未就绪且 GUIDED→`confirm_arm(all_ready=False)` 诚实横幅, 确认后自动降级 `mode='STABILIZE'` 解锁**(台架/室内, 室外EKF就绪仍走原GUIDED路径); 未就绪非GUIDED→放弃并指引 |
| 诚实横幅 | `confirm_arm(reason='', all_ready=True)`: 就绪=🚨检测一切正常 (P10/P11 断言不变), 未就绪=**⚠️位置估计未就绪, 解锁可能被拒!**; 状态存模块级 **`ARM_CONFIRM_ALL_READY`**, GUI `_arm_confirm_dialog` 标题据此切换 (不再谎报); hook 保持单参 `hook(reason)` 兼容 |
| 参数判定 | `set_param` 回读不符**不再"同名即真"**: 重读复核仍不符→`❌ 固件钳制/写入未生效` 返回 False; `ensure_param` 三态 True/False(钳制,计失败)/**None(读×2+写均无响应=固件无此参数)**; `ensure_launch_params` 对 None 打 `ℹ️ …固件无此参数, 跳过` **不计失败** (修 BRD_SAFETYENABLE 假警报), 钳制才计入"部分参数未确认" |
| 解锁证据 | `arm_vehicle` 成功后新增 **`print_motor_outputs`**: 请求 RAW_CONTROLLER 流 + `MAV_CMD_REQUEST_MESSAGE(36)` 读 `SERVO_OUTPUT_RAW`, 打印 M1~M4 指令PWM (`≈1000=受控停转, GUIDED解锁后属正常`), 区分"FC没给油门"与"FC给油门但电机不转"; 无消息仅提示跳过不判失败 |
| 上锁回退 | `disarm_vehicle` 增加 `_fallback` 守卫: DISARM ACK 被拒时**普通↔强制自动切换再试一次** (实机遇过 force=21196 返回 `UNSUPPORTED=3`), 双向防循环 |
| 文案 | 测试2 说明补"室内EKF未就绪经确认降级STABILIZE"; 失败排查顺序 1) 补降级路径、5) 改"安全开关按物理按钮, 部分固件无 BRD_SAFETYENABLE 属正常" |

- **验证**: `smoke_wp_rel` FakeFC 升级 (`ekf_flags` 属性 + armed 发 `servo_output_raw` + param 读写对未知参数不响应模拟真实固件 + 预置 `DISARM_DELAY`), P10 加 `电机指令 PWM` 断言; **新增 P13**: `flags=0xA7`+`Need Position Estimate` → 诚实横幅/无假横幅/STABILIZE(0) 模式命令/降级解锁/`ARM_CONFIRM_ALL_READY=False`; **新增 P14**: pop BRD → `ℹ️ 固件无此参数, 跳过` 且不计失败+任务✅; P1~P14+P6 全绿 (`SMOKE_WP_REL_OK`); 回归 `repro_test3`/`smoke_ui2`/`smoke_drop`/`smoke_map2` 全绿; `pyflakes` 基线 6 条均既有无新增; 临时文件清理 0 残留
- **实机复测** (COM19@57600, 桨已拆, GUI 断开, `%TEMP%\\opencode\\test2_bench.py`): **TEST2_BENCH_OK 12/12** — GUIDED 拒绝→EKF 诚实超时→⚠️横幅(自动确认)→STABILIZE 降级解锁→电机指令 PWM 打印→上锁, 无🚨假横幅; 注意测试后 FC `DISARM_DELAY=60` (测试2自动写入属预期)

## v1.24.1 - 2026-09-30

### 定高解锁/上锁按钮 (快速控制, 室内试用)

| 项 | 说明 |
|----|------|
| 按钮 | 快速控制第一行新增 **`定高解锁/上锁`** (ARM/DISARM 右侧, 起飞前): 一键切换 — 未解锁→切 **ALT_HOLD(定高)** 并 `arm_vehicle` 解锁(含 DISARM_DELAY 延长/发射箱参数预设/解锁后打印电机指令PWM), 已解锁→`disarm_vehicle` 上锁+RC中立收尾; `apu.on_arm_alt_hold()` 走 `_enqueue` worker 任务, 任务标签 `定高解锁/上锁` |
| 适用 | ALT_HOLD **不依赖水平位置估计**, 室内GPS多径(EKF无水平位置, GUIDED/测试2被拒)下仍可解锁; 首次点击即直接解锁, 无位置等待/确认弹窗; 60s 内不发起飞命令仍按 DISARM_DELAY 自动上锁 |

- **验证**: 冒烟新增 **P15**: 未锁→切 ALT_HOLD(2) 解锁+电机PWM+任务✅, 再点→上锁; `SMOKE_WP_REL_OK`(P1~P15+P6); `pyflakes` 基线 6 条无新增; **实机 COM19@57600** `alt_hold_bench.py` → **`ALT_HOLD_ARM_OK`**(直接 ARM 接受, 电机指令 PWM=1000 打印, 收尾正常上锁), `BRD_SAFETYENABLE` 显示 `ℹ️ 固件无此参数, 跳过`
- **注意**: 需**重启 GUI** 加载新代码; "解锁 ARM" 按钮仍走 GUIDED(位置估计依赖), 室内请用「定高解锁/上锁」

## v1.24.2 - 2026-09-30

### 交还遥控器 (物理遥控器立刻接管)

| 项 | 说明 |
|----|------|
| 问题 | `RC_OVERRIDE_TIME=-1` 下 GCS 覆盖**永不过期**, 软件解锁/软遥控收尾发的是**中立帧**(继续钉住覆盖) → 机上接收机输入被屏蔽, **物理遥控器摇杆一直无效**; 实机证据: 上锁后 ch3 仍钉在 1500(空中收尾值), 手动发地面中立帧才变 1000 — 覆盖跨解锁持久 |
| 机制 | MAVLink `RC_CHANNELS_OVERRIDE` 约定**通道值=0 即释放该通道**: 发 4 通道全0帧 → GCS 覆盖撤销 → 接收机摇杆立即生效 |
| 配置 | `tct.RC_HANDOVER = 1` (默认): 收尾走全0交还; `0` = v1.23 中立帧(纯软件发射箱无接收机时: 全0=无效PWM空中掉高) |
| 自动交还 | `release_rc_override(master, handover=None)` 按 `RC_HANDOVER` 选帧; **解锁成功/锁定成功/软遥控停用/断开/退出/CLI arm** 全部收尾自动交还 → 平时覆盖即处于释放态, 摇杆随时有效 |
| 一键交还 | 安全条新增 **「交还遥控器」** 按钮 (`on_handover_rc`): 强制全0帧×3 + 自动停用软遥控(防20Hz回发覆盖) + 状态`已交还物理遥控器`; 未连接仅提示 |
| 接管条件 | ①覆盖已交还(本功能) ②模式在**手动档**(GUIDED/自动档下固件忽略摇杆 → 按 F9 切 LOITER 或接收机模式开关) ③软遥控未启用(其20Hz覆盖优先); 三者齐备 = 推杆即接管 |

- **验证**: 冒烟新增 **P16**: `RC_HANDOVER=1` → 全0帧×3+日志+状态标签; `RC_HANDOVER=0` → 回退地面中立帧×3 (P6/P12 中立断言靠启动时 `RC_HANDOVER=0` 保持); `SMOKE_WP_REL_OK`(P1~P16+P6); `pyflakes` 基线 6 条无新增; 回归全绿
- **实机验证** (已装桨, 全程不解锁, `rc_safe_bench.py`): 钉住态读回接收机活值(ch1=1515等, 采样内实时变化) -> 发覆盖 ch1=1400 立即生效 -> **全0交还后 ch1 回到 1515 实时值 = 物理遥控链路恢复** -> 中立帧钉住对照精确 1500/1500/1000/1500 -> 最终交还; SAFE_BENCH_OK, motors_armed=False 全程
- **实机注意**: 需**重启 GUI**; 有接收机(本机已实测遥控可解锁)保持默认 `RC_HANDOVER=1`; 若将来改成纯软件无接收机发射箱, 改 tct 顶部 `RC_HANDOVER=0`; 验证流程 = 定高解锁(日志见"全0交还") → F9 切 LOITER → 拨物理摇杆有响应

## v1.24.3 - 2026-09-30

### 起飞链路兼容降级模式 (STABILIZE 不起飞根因修复 + ALT_HOLD 油门爬升)

| 项 | 说明 |
|----|------|
| 问题 | 实机复现: 重启飞控后 EKF 未就绪 → GUIDED 解锁被拒 → 经确认降级 **STABILIZE** 后能解锁, 但 `arm_and_takeoff` **无条件发 NAV_TAKEOFF**, 手动模式下该指令被固件忽略 → 解锁后不起飞(等高度超时)。"是否油门未逐步增加": 本质是**模式与起飞方式不匹配** — STABILIZE 纯手动不自动加油, NAV_TAKEOFF 只在 GUIDED/AUTO 生效 |
| 降级改 ALT_HOLD | `arm_vehicle(..., fallback_mode='ALT_HOLD')` (默认): GPS 全无/EKF 持续未就绪时经现场确认降级 ALT_HOLD 而非 STABILIZE — 气压定高不依赖位置估计(室内实测可解锁), 且比 STABILIZE 更安全(摇杆释放自动定高); 两条拒因路径(GPS 未3D定位/EKF 未就绪)均给降级选项 |
| 起飞分派 | `arm_and_takeoff` 解锁后读心跳判定实际模式: **GUIDED/AUTO → NAV_TAKEOFF**(原路径) / **ALT_HOLD → `takeoff_althold` 油门覆盖爬升**(AUTOTUNE_CLIMB_THR=1700 → 中油门稳 2s → 自动释放覆盖, 测试4/测试14/起飞按钮自动生效) / 其它模式(如 STABILIZE) → 中止并提示(不再盲发 NAV_TAKEOFF 空等) |
| 测试14 | GPS 门控放宽: 未就绪仅 ⚠️ 提示并继续(走 ALT_HOLD 降级起降, 悬停/降落只需气压计), 不再直接 ❌ 失败 |
| 文案 | 解锁失败排查顺序 1) 与测试2 说明改为 ALT_HOLD 降级路径 |
| 待确诊 | 当日 16:43 测试2 "解锁失败(超时 10s)" 全程无 ACK/STATUSTEXT(rejected=False 未走重试), 疑与飞控刚重启有关, 未复现; 若再出现请保存完整日志 |

- **验证**: `smoke_wp_rel`: P13 断言改 ALT_HOLD(2) 降级; **新增 P17**: EKF 未就绪 → 起飞按钮 → 降级 ALT_HOLD + 油门爬升到位(≥4.5m) + 最终模式 ALT_HOLD(2) + 任务✅ + 上锁清理; `SMOKE_WP_REL_OK`(P1~P17+P6); `py_compile` OK; `pyflakes` 基线 6 条无新增; 回归 `repro_test3`/`smoke_ui2`/`smoke_drop`/`smoke_map2` 全绿
- **实机注意**: 桨已装, 本轮未做任何实机解锁测试; **重启 GUI** 后生效; 室内/无位置估计时: 起飞按钮或测试4/14 自动走"降级确认 → ALT_HOLD 油门爬升"(需现场确认放行), 室外 EKF 就绪仍走原 GUIDED+NAV_TAKEOFF 路径; "定高解锁/上锁"按钮不受影响

## v1.24.4 - 2026-10-02

### 起飞油门死区修复 + 起飞油门设置 + 飞控重启

| 项 | 说明 |
|----|------|
| 根因 | 实机反馈"解锁成功但起飞时油门不够飞不起来": `takeoff_althold` 到位前把爬升油门降到 **1600** — ArduCopter 油门以 1500=悬停、±100 死区换算爬升率, **1600=死区边缘(零爬升需求)**; 低空目标(BOUNCE_ALT=2) `rel < alt-2` 从地面即判假 → **从头就发 1600 → 根本不离地**; 5m 目标爬到 3m 也卡死。且 1700 仅≈0.6m/s 需求, 本就偏慢 |
| 修复 | `takeoff_althold(master, alt, timeout, climb_thr=None)`: 爬升段**全程发 climb_thr 直到到位**(删除 1600 死区段), 到位后仍 1500 稳 2s→释放(不变); 自动调参调用点显式传 `climb_thr=AUTOTUNE_CLIMB_THR` 保持原行为 |
| 新配置 | `tct.TAKEOFF_CLIMB_THR = 1800` (≈1.25m/s 需求); GUI 参数行新增 **「起飞油门」** 输入框, `_apply_params` 钳制 **1600~2000**(越界⚠️+自动限制)后写入, 起飞按钮/测试4/14 自动生效 |
| 飞控重启 | 新函数 `reboot_flight_controller(master, timeout=15)`: 已解锁直接拒绝; 发 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=1`; 等**心跳中断 ≥2.5s** 判定重启开始(超时则⚠️提示未执行); GUI 连接行新增 **「重启飞控」** 按钮(断开旁, 连接时才可用) → `on_reboot_fc`: 未连接/已解锁拒绝, `messagebox` 二次确认(重启窗口/USB 可能需重连/任务中断提示), 入队执行 |
| 顺手修复 | `on_reboot_fc` 的 fn 曾在 worker 线程 `var_link.set()` → 抛异常致任务显示 ❌ 失败(实际重启已成功); 改为只用 tct 内 print 输出进度, **不在线程内碰 tk 变量**(冒烟抓出的真 bug) |

- **验证**: 冒烟 **P17 强化**: 起飞高度=2m + 起飞油门=1750 → 断言 `_apply_params` 写入 1750、爬升帧 ch3=1750 且通道正确、**全程零 1600 帧**(死区回归)、到位≥1.5m、任务✅, 结束恢复 5m/1800; **新增 P18**: ①已解锁→拒绝重启+零命令帧 ②上锁→确认→命令 param1=1 单发→心跳中断→任务✅(FakeFC 加 cmd246 应答+心跳挂起 4s); `SMOKE_WP_REL_OK`(P1~P18+P6); `py_compile` OK; `pyflakes` 基线 6 条无新增; 回归 `repro_test3`/`smoke_ui2`/`smoke_drop`/`smoke_map2` 全绿
- **实机注意**: **重启 GUI** 后生效; 起飞仍嫌慢就把「起飞油门」调到 1900/2000(上限 2000, ALT_HOLD 下该值=爬升率需求, 实际电机由飞控高度环给出); 「重启飞控」须已上锁, 重启约 10~30s 后重新连接(USB 虚拟串口可能重新枚举)。本轮未做实机解锁/重启(桨已装), 待现场验证

## v1.24.5 - 2026-10-03

### 发射箱参数一次性写入按钮 (避免重复写入)

| 项 | 说明 |
|----|------|
| 背景 | 用户反馈：每次解锁/起飞都会重复写入发射箱参数，若飞控参数未变实属多余。原实现中 `ensure_launch_params` / `ensure_disarm_delay` 虽然只写值不符的参数，但每次仍需读取/对比，耗时且日志冗余 |
| 新功能 | GUI 参数行新增 **"写入发射箱参数"** 按钮（起飞油门后）。点击后一次性写入全部发射箱预设参数 + DISARM_DELAY，并设置会话标记。后续自动调用 `ensure_launch_params` / `ensure_disarm_delay` 时直接跳过并打印 `ℹ️ 已手动写入，跳过` |
| 实现细节 | - 模块级标记 `LAUNCH_PARAMS_WRITTEN` / `DISARM_DELAY_WRITTEN` (v1.24.5)
- `ensure_launch_params` / `ensure_disarm_delay`: 检查标记，已写入则跳过
- `write_launch_params` / `write_disarm_delay`: 手动写入函数，也检查标记并打印跳过提示
- `reset_launch_params_flag()`: 连接/断开时自动重置（连接时重置，断开时重置）
- GUI 按钮 `on_write_launch_params` 入队执行，不阻塞 UI
- `on_takeoff` / `on_arm_alt_hold` / CLI `arm` 等自动流程受益：首次手动写入后，后续完全跳过读写 |
| 验证 | 冒烟 **P19 新增**: 手动写入 -> 日志含 "发射箱参数预设" / "DISARM_DELAY" -> 二次调用直接跳过并打印 "已手动写入，跳过" / "发射箱参数预设" 不再出现 -> 断开重连后标记重置；`SMOKE_WP_REL_OK`(P1~P19+P6)；`py_compile` OK；`pyflakes` 基线 6 条无新增；回归 `repro_test3`/`smoke_ui2`/`smoke_drop`/`smoke_map2` 全绿 |
| 实机建议 | 重启 GUI 生效；首次连接新飞控时点击"写入发射箱参数"一次；后续所有解锁/起飞/测试将自动跳过参数写入，仅打印跳过提示。若飞控固件升级/参数被外部改动，重新连接后按钮可再次点击。本轮未做实机验证（桨已装），请现场验证 |


## v1.24.6 - 2026-10-03

### 串口自动枚举与默认选择

| 项 | 说明 |
|----|------|
| 背景 | 连接框原为手动输入 COM 口号，每次需查看设备管理器或试错，易输错；重启软件后不记忆上次端口 |
| 新功能 | - **连接栏改为下拉框**：自动枚举系统所有串口设备（COMx + 设备描述 + 硬件ID）显示
- **刷新串口按钮**：点击即时刷新列表，支持热插拔
- **上次连接记忆**：连接成功后自动保存该串口（设备名如 COM3）到 `apm_ui_config.json`，下次启动自动选中（若该端口仍存在） |
| 实现细节 | - 依赖 `pyserial` 的 `serial.tools.list_ports`（已作为可选依赖，无则降级为手动输入）
- 配置文件 `apm_ui_config.json` 存放于脚本同级目录，仅存 `{"last_port": "COM3"}`
- 连接 UI：`Entry` -> `Combobox` + `刷新串口` 按钮，显示格式 `COM3 - USB Serial Port (VID:PID)`
- 连接成功时 `_after_connect` 调用 `_save_selected_port` 提取设备名保存
- 启动时 `load_last_port()` 读取上次端口，若端口仍在列表则默认选中 |
| 验证 | `py_compile` OK；`pyflakes` 基线 6 条无新增；冒烟 `SMOKE_WP_REL_OK`(P1~P19+P6)；回归 4 套全绿 |
| 实机建议 | 重启 GUI 生效；首次打开会自动枚举串口；连接一次后下次打开自动选中该端口；拔插 USB 后点击「刷新串口」更新列表 |


## v1.24.7 - 2026-10-03

### 震动监测 (Mission Planner Vibration 类似)

| 项 | 说明 |
|----|------|
| 功能 | 右侧底部新增「震动监测 (Vibration)」面板，实时显示 X/Y/Z 三轴加速度震动波形 (m/s²) |
| 数据源 | 解析 MAVLink `VIBRATION` 消息 (`vibration_x/y/z`，单位 m/s²)，按 1Hz 采样存入历史 (最多 300 点，约 5 分钟) |
| 显示 | - Canvas 绘制三色波形图：X=红、Y=绿、Z=蓝，含零线、网格、实时数值标记
- 底部实时数值标签：X/Y/Z 当前值 (m/s²，保留 2 位小数)
- 自动缩放：按历史最大振幅对称缩放，保留零线居中
- 空数据时显示「等待震动数据...」提示 |
| 采样 | `_tick` 中按 1Hz 从 `tel['vib']` (reader 线程解析 VIBRATION 消息写入) 采样，最多保留 300 点 |
| 布局 | 主体网格扩展为 3 列：左=遥测、中=测试/日志、右=震动监测 (与日志同行，等宽) |
| 依赖 | 仅使用 tkinter Canvas 绘制，无需 matplotlib 等额外依赖，轻量实时 |
| 验证 | `py_compile` OK；`pyflakes` 基线 6 条无新增；冒烟 `SMOKE_WP_REL_OK`(P1~P19+P6)；回归 4 套全绿 |


## v1.25.0 - 2025-10-04

### Added
- **AI PID Tuning Tool (Standalone)**: 新增独立工具 i_pid_tune.py + 核心 i_tune_core.py
  - 支持多旋翼（APM Copter + PX4 MC），规格推荐 → 参数写入 → 试飞采集 → 闭环迭代
  - DeepSeek API 接入（deepseek-chat），界面填写并保存 API Key
  - 参数白名单、范围钳制、写入前自动备份、备份恢复
  - 试飞采集（被动记录 + 可选自动阶跃激励，双重确认，安全闸）
- **Smoke 测试**：smoke_ai_tune.py 基本冒烟

### Notes
- 独立工具，不修改任何 pm_control*.py / px4_control*.py 脚本


## v1.25.0 - 2025-10-04

### Added
- **AI PID Tuning Tool (Standalone)**: 新增独立工具 ai_pid_tune.py + 核心 ai_tune_core.py
  - 支持多旋翼（APM Copter + PX4 MC），规格推荐 -> 参数写入 -> 手动描述迭代闭环（第一期）
  - DeepSeek API 接入（deepseek-chat/deepseek-reasoner），界面填写并保存 API Key
  - 参数白名单（姿态环/速率环/滤波/响应）、范围钳制、写入前自动备份、备份恢复
  - 自动识别飞控类型（APM=3/PX4=12，心跳 autopilot 字段）
  - 标准 MAVLink PARAM 协议读写（get_param/set_param/read_params），APM/PX4 通用
  - 配置持久化 ai_tune_config.json（API Key/模型/端口/波特率/机型规格）
  - GUI: 顶栏连接/FC类型/状态、左栏规格表单+API Key、Notebook四页（AI建议/当前参数/迭代历史/备份管理）
  - 线程安全：worker 线程串行执行耗时操作，主线程 after() 轮询日志/UI
- **Smoke 测试**：smoke_ai_tune.py 基本冒烟（参数集、验证、JSON提取）

### Notes
- 独立工具，不修改任何 apm_control*.py / px4_control*.py 脚本
- 第二期计划：试飞被动采集（ATTITUDE/VIBRATION） -> 自动阶跃激励（安全闸） -> 采集指标喂入迭代


## v1.25.1 - 2025-10-04

### Added (Phase 2 - Flight Test Closed Loop)
- **FlightRecorder 类** (i_tune_core.py): 被动采集 ATTITUDE/VIBRATION/GLOBAL_POSITION_INT/RC_CHANNELS
  - 后台线程记录，线程安全锁保护
  - nalyze() 多维指标：姿态零穿越/极差/RMS、振动 RMS/峰值/裁剪、高度稳定性、RC 输入统计
- **安全闸检查** check_safety_for_step(): 解锁+定点模式+高度≥2m+GPS 3D定位
- **自动阶跃激励** uto_step_excitation(): SET_ATTITUDE_TARGET 发送姿态阶跃，可中止
- **GUI 试飞采集页**: 开始/停止记录、实时指标显示、阶跃激励配置与执行、双重确认、中止按钮
- **AI 迭代集成**: 飞行指标自动注入下一轮提示词，AI 基于实测数据优化

### Updated
- i_pid_tune.py: 新增飞行记录器、指标显示、阶跃激励控制、中止标志
- 迭代历史自动包含飞行指标摘要


## v1.26.0 - 2025-10-04

### Added - DataFlash Log Management & AI Analysis
- **Log Management Tab** (i_pid_tune.py): 完整的日志管理界面
  - 刷新日志列表：通过 MAVLink LOG_REQUEST_LIST/LOG_ENTRY 获取飞控上所有数据闪存日志
  - 列表显示：日志ID、大小、UTC时间、勾选下载
  - 批量下载：支持多选日志并行下载（LOG_REQUEST_DATA 分块传输），进度条显示
  - 保存目录可配置，自动保存为 .bin 文件
  - 清除日志：一键清除飞控所有数据闪存（需双重确认）

- **自动解析**：集成 pymavlink.tools.mavlogdump 将 .bin 转为 CSV（ATTITUDE/VIBRATION/GPS/RCIN/RCOU/EKF/IMU 等）

- **AI 日志分析**：将关键 CSV 数据发送 DeepSeek，输出结构化 JSON：
  - 飞行状态总结、问题列表、具体参数调整建议（含当前值/建议值/理由）、安全提示
  - 结果直接显示在日志管理页，可直接指导调参

- **核心库扩展** (i_tune_core.py)：
  - 
equest_log_list() / download_log() / download_logs() / erase_logs() - 标准 MAVLink 日志协议
  - parse_log_with_mavlogdump() - 调用 mavlogdump 生成 CSV
  - nalyze_log_with_ai() - 读取关键 CSV，发送 DeepSeek 分析
  - save_log_to_file() - 保存 .bin 文件

- **GUI 集成**：新增「日志管理」标签页（第5页），包含日志列表、进度条、下载选项、AI分析结果区
- 与调参闭环集成：AI分析结果可直接指导下一轮参数调整

### Updated
- i_tune_core.py: 新增日志管理、解析、AI分析完整链路
- i_pid_tune.py: 新增「日志管理」标签页（第5页），完整下载/解析/AI分析工作流
- i_pid_tune.py: 日志列表支持勾选、进度条、自动解析/分析开关、目录选择


## v1.27.0 - 2025-10-04

### Added - 参数自动获取与帮助提示
- **完整工具提示系统**: 为所有规格参数字段添加悬停工具提示，详细说明：
  - 飞控是否可获取（通过何种 MAVLink 消息/参数）
  - 其他自动获取途径（空载测试、厂家数据库、万用表测量等）
  - 手工填写回落方案
  - 关键性提示（⚠ 标记关键参数）

- **详细字段扩展** (v1.26.0 基础上):
  - 尺寸档新增 13寸/15寸
  - 电池 S 数下拉框: 4S/6S/8S/12S
  - 电机: 类型/最大电流/内阻/极对数
  - 电调: 协议/DShot版本/最大电流/BEC输出
  - 桨叶: 尺寸/桨距/片数/材质
  - 电池: S数/容量/C数

- **一键从飞控获取参数** (i_pid_tune.py + i_tune_core.py):
  - 新增「🔄 从飞控获取可用参数」按钮
  - 自动读取并填充:
    - 电池电压/电流 → 推算 S 数
    - ESC 遥测 → 电机 RPM、电调电压/电流/温度
    - 电池状态 → 电压、电流、已消耗容量
    - PX4 参数 → 极对数 (MOT_POLE_COUNT)
    - GPS 定位状态、飞行模式
  - 结果弹窗显示获取到的参数，自动填充对应表单字段
  - 核心库新增 etch_fc_autofill_params() 统一接口

- **完善帮助体系**: 每字段悬停提示包含：
  - ✅ 飞控可获取: 具体 MAVLink 消息/参数名
  - 🔧 其他自动途径: 测试方法、工具、数据库
  - 📝 手工回落: 说明书、铭牌、目视确认
  - ⚠ 关键性警示: 对调参影响大的参数特别标注


## v1.28.0 - 2025-10-04

### Changed - DeepSeek 模型名更新
- 根据官方最新文档 (https://api-docs.deepseek.com/zh-cn/) 更新模型名称:
  - **deepseek-flash** (默认/推荐) - 替代旧的 deepseek-chat / deepseek-v4-flash
  - **deepseek-v4-pro** - 替代旧的 deepseek-reasoner
- 旧模型名 (deepseek-chat, deepseek-reasoner, deepseek-v4-flash) 仍可调用但已下线/重定向
- 所有默认配置、下拉选项、函数默认参数、配置文件统一更新

## v1.29.0 - 2026-10-04

### Fixed - apm_control_ui.py: on_wp_run 缺失切 AUTO 模式
- `on_wp_run` 内 `arm_and_takeoff` 成功后新增 `apc.set_mode(p, 'AUTO')`，`try/except SystemExit` 兜底
- 修复了起飞后悬停在 GUIDED、任务永不执行、wait_auto_mission 超时/误报"离开 AUTO"的 Bug
- 参考实现 `apm_control.py:run_test_mission` 已有该步骤

### Added - 模式下拉框新增 'AUTO'
- 快速控制面板模式下拉 (`var_mode`) 补全 'AUTO'，可手动切入 AUTO

### Changed - "航点任务" 页拆分为两个 Notebook 页，明确区分 GUIDED 单点飞行 与 AUTO 航点任务
- **页一「单点飞行(GUIDED)」**：目的地/高度/巡航 + `起飞并飞往(GUIDED)` + `仅飞往(GUIDED)` + 读速度参数 + 当前模式指示
  - 说明栏: "GUIDED 动态改点 · 到达后悬停 · 飞行中可随时改目的地(立即生效)"
- **页二「航点任务(AUTO)」**：校验/仅上传/`执行(解锁+起飞→AUTO)` + 航点文本框 + 雷达填充 + 加载 waypoints.txt + 当前模式指示
  - 说明栏: "AUTO 固定航线 · 执行后切 AUTO 按序飞+抛投 · 中途不可改点"
- 两页各加 "当前模式: XX" 指示 (共享 `tel_labels['mode']`，遥测每拍更新)

### Added - 单点飞行 支持飞行中立即改点
- 新增 `_goto_spec`(原子 dict 含 epoch) + `_goto_active`(Event) 状态
- `on_goto_point` 校验通过后总是先写 spec 并 bump epoch：
  - 若 `_goto_active` 已置位(已有 goto 任务在跑) → 日志 `✈ 立即改点 → ...`，直接返回，**不入队**
  - 否则照旧 `_enqueue` 启动新任务
- `fn()` goto 循环每圈比对 epoch：
  - epoch 变化 → 重新解析相对/绝对坐标(仅在改点时刻按当前飞机 GPS 换算一次) → 按新距离重置超时 `max(60, 30+d/2)` → 继续飞
  - 到达判定前再查一次 epoch，有待处理改点则继续循环不返回 (防"刚到旧点就丢新点")
- 巡航速度 WPNAV_SPEED 仍在任务开始时写一次，改点不重写

### Verified
- `python -m py_compile apm_control_ui.py` 通过
- GUI 创建/销毁冒烟通过 (两页标签: "单点飞行(GUIDED)" / "航点任务(AUTO)")

## v1.29.1 - 2026-10-04

### Fixed - 串口(COM)连接成功后界面卡死 / 无遥测数据 (Windows)
- **根因**: Windows 上 pymavlink 对**串口句柄**调用 `select.select()` 会立即抛
  `OSError`(WinError 10093/10038, 非 socket 句柄), pymavlink 内部 `except
  select.error` 吞掉后**立即返回**。于是 `recv_match(blocking=True, timeout=...)`
  在无数据时变成**纯 Python 忙等**——读线程 100% 占用 CPU 且持续持有 GIL,
  把 tkinter 主线程饿死 → 连接成功后界面卡死、无任何遥测刷新。
  - TCP/UDP 连接 select 正常, 故之前网络连接无此问题。
- **修复** (`apm_control_ui.py:_reader_loop`):
  1. 改为 `recv_match(blocking=False)` **非阻塞轮询**, 每轮最多排空 300 条消息;
  2. 空闲(本轮无消息)时 `time.sleep(0.004)` **主动让出 GIL**, 轮询率约 250Hz,
     不再空转;
  3. 连接阶段等待心跳同样改为非阻塞轮询 + 20ms 小睡(原 `wait_heartbeat`
     在 Windows 串口上也会忙等)。
- **验证**: 空闲 0.3s 内 `recv_match` 空闲调用 ~65 次(旧实现达数十万次);
  有消息时按序排空处理正确; `AmpUI` 完整初始化 8 个页签正常。
- 说明: `px4_control_ui.py` 的 `_reader_loop` 为同款结构, 存在相同隐患,
  本次按约定未改动(待需要时同步修复)。

### Added - 串口连接预检与进度日志 (apm_control_ui.py:on_connect)
- 串口连接(COMx/`serial:`)先经 pyserial 以 0.5s 超时试开一次, 端口被占用/
  不存在/驱动异常时立即报错, 不再干等 30s; 网络连接(tcp:/udp:)跳过预检。
- 心跳等待期间每 3s 打印 `⏳ 等待心跳中... (Ns/30s)`, 避免无反馈显得卡死。

## v1.29.2 - 2026-10-04

### Fixed - 连接成功即卡死(真凶): VIBRATION 分支 tel_lock 自锁死 (**关键修复**)
- **根因**: `apm_control_ui.py` 的 `_update_tel()` 开头已 `with self.tel_lock:`
  (普通 `threading.Lock`, **非重入**), 但 VIBRATION 分支内又写了一次
  `with self.tel_lock: self.tel['vib'] = ...`。**同线程重复获取非重入锁
  → 永久死锁**:
  - 读线程 + 主线程的 `_tick` 都在等这把锁 → 界面彻底无响应;
  - 恰好 **真实 APM 固件会持续发送 `VIBRATION` 消息**, 而之前的网络假流测试
    未包含该消息, 故只在接真飞控(COM19/COM23)时必现;
  - 现象与"连接成功但无数据、界面卡死"完全吻合。
- **修复**:
  1. 删除 VIBRATION 分支内多余的 `with self.tel_lock:`(外层已持有, 直接用 `t`);
  2. `self.tel_lock` 由 `threading.Lock()` 改为 `threading.RLock()`
     (可重入), 从根本上杜绝此类嵌套自锁死。
- 同步修复 `px4_control_ui.py` 中**完全相同**的 VIBRATION 自锁死与 `tel_lock`。
- **验证**:
  - 复现测试: 假 MAVLink 服务器持续发 HEARTBEAT+GLOBAL_POSITION_INT+
    **VIBRATION**, 连接后主线程 tick 稳定推进(~200ms/拍), 未卡死;
  - 锁语义验证: 普通 Lock 同线程重复获取 2s 未完成(死锁), RLock 正常完成。

### Changed - 主线程刷新循环防假死 (apm_control_ui.py)
- `_tick` 拆为外层包装 + `_tick_body`: 外层 `try/except/finally` 保证**任何
  一次刷新异常都不会中断 `root.after` 重排链**(否则界面会假死), 异常写入日志。

### Added - 界面卡死看门狗诊断 (apm_control_ui.py)
- 后台看门狗线程: 已连接状态下主线程 `_tick` 停摆 >3s 时, 把所有线程堆栈
  转储到工作目录 `ui_hang_dump.txt`, 便于后续定位任何卡死点。
- 验证: 人为阻塞主线程 6s, 看门狗成功转储含阻塞帧的堆栈。

## v1.29.4 - 2026-10-05

### Fixed - 预检门禁拦在 ARM 之前, 室内"起飞(按高度)/14低空起降"干等60s后失败

- **现象(实机)**: 点 **起飞(按高度)** 或 **14 低空起降悬停**, 反复只打
  `⏳ 等待位置估计就绪 (Ns/60s): EKF无有效位置(flags=0x00a7)`, 60s 后
  `❌ 预检未通过(...) 本次不发送 ARM` 直接失败; **遥控器 STABILIZE 却能起飞**。
- **不是姿态问题**: `flags=0x00a7` 解码 = `ATTITUDE|VELOCITY_HORIZ|VELOCITY_VERT|POS_VERT_ABS|CONST_POS_MODE`,
  **缺 `POS_HORIZ_REL/ABS`** → 飞控 `PreArm: Need Position Estimate`(mandatory, 跳不掉)。
  室内 GPS 多径(fix=3 但 EKF 不融合水平位置 → 恒定位模式); 遥控器手动模式不查位置估计, 故可飞。
- **根因(代码 bug)**: v1.29.3 预检门禁 `wait_position_ready` 超时后**直接 `return False`**,
  而 v1.24.3 的 **ALT_HOLD 降级分支要"ARM 命令被飞控拒绝"才触发** → 门禁永远先拦在 ARM 之前,
  降级分支永不执行, 两条起飞路径只能失败。
- **修复** (`apm_control_test.py::arm_vehicle`): 预检未通过改走与"ARM 被拒"**同一降级出口** —
  `confirm_arm(all_ready=False)` 现场确认后 `arm_vehicle(fallback_mode='ALT_HOLD', gate=False)`;
  `arm_and_takeoff` 按实际模式用**油门覆盖爬升**(气压定高, 无需GPS)替代 NAV_TAKEOFF(原有分派逻辑, 无需改动)。
- **文案**: `position_ready_state` 恒定位时返回 `EKF无水平位置(恒定位,未融合GPS,flags=0x00a7)`;
  GUI 确认框事由含"改用"时标题改为 `⚠️ 位置估计未就绪 → 改用 ALT_HOLD 解锁!`(不再含糊)。
- **验证**: `py_compile` OK; `pyflakes` 基线 8 条无新增; 门禁三路径脚本
  (确认→降级 ALT_HOLD / 取消→不解锁 / 就绪→继续 GUIDED) + `position_ready_state` 判据用例全绿。

## v1.30.0 - 2026-10-05

### Added - 独立 EKF 状态监控线程 + 安全条解锁判定 (apm_control_ui.py)

- **需求**: 连接后随时可见"EKF 是否正常 / 能否 GUIDED 解锁", 而不是点起飞
  才在日志里干等 60s 预检才知道结果。
- **线程**: `AmpUI._ekf_monitor_loop` (1s 周期), `_after_connect` 启动,
  断开/退出时 `_stop_ekf_monitor` 停止 (带线程身份校验, 防重连出双线程)。
- **判据同源**: 直接调 `tct.position_ready_state(fix, sats, flags)` — 与
  `arm_vehicle` 预检门禁完全一致, **界面结论 = 解锁行为**, 不会出现
  "界面说行但解锁被拒"。
- **不抢消息**: 线程只读读线程灌的共享遥测 (`tel_lock`), 全程不调
  `recv_match`; 仅当 `EKF_STATUS_REPORT` 陈旧 >6s 才主动 `REQUEST_MESSAGE`
  一次 (只发不收, 零竞争), 覆盖固件不流式发送该报文的情况。
- **安全条新增 `EKF:` 标签**, 结论四态:
  | 文案 | 样式 | 条件 |
  |------|------|------|
  | `✅ 正常 可GUIDED解锁` | Ok 绿 | 判据全绿 |
  | `⏳ 未就绪: <原因>` | Warn 黄 | 如 `EKF无水平位置(恒定位,未融合GPS,flags=0x00a7)` |
  | `⏳ 等待飞控数据` | Gray 灰 | 未收到 GPS/EKF 报文 |
  | `⚠️ 心跳丢失` | Bad 红 | 心跳 >5s 未更新 |
- **日志**: 判定(就绪+原因)变化才打一行, 5s 节流防刷屏 —
  `✅ EKF 正常, 可 GUIDED 解锁 (flags=0x...)` / `⏳ EKF 未就绪(...), 暂不可 GUIDED 解锁`。
- **数据新鲜度**: `_update_tel` 记录 `gps_t`/`ekf_t`, 监控线程据其判报文陈旧,
  并区分"心跳丢失"与"EKF 未就绪"两种失败。
- **渲染线程安全**: 线程只写 `self.ekf_verdict`, 由主线程 `_tick_body`
  统一渲染到标签 (与遥测面板同一约定, 不从后台线程碰 Tk 变量)。
- **验证**: `py_compile` OK; `pyflakes` 基线 8 条无新增; 监控线程 5 用例
  (就绪 / 室内 0xA7 / 无数据 / 陈旧触发 REQUEST / 心跳丢失) +
  真实 `AmpUI` 构造→启动线程→灌遥测→`_tick_body` 渲染→停线程 UI 冒烟全绿。
- 说明: 仅 `apm_control_ui.py` (px4 侧无 `position_ready_state`, 未同步)。
