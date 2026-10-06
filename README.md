# APM / PX4 地面站控制台 + AI 调参工具

基于 **MAVLink + pymavlink + tkinter** 的 ArduCopter(APM) / PX4 飞控地面端工具集，
包含图形控制台、自动化飞行测试脚本、航点任务/抛投，以及接入大模型的 PID 调参助手。

> **免责声明**：本项目仅供学习与研究。飞行有风险，请务必在空旷场地、拆桨完成地面联调，
> 首飞前逐条检查参数与安全设置，责任自负。

---

## 功能概览

| 入口 | 类型 | 说明 |
|------|------|------|
| `apm_control_ui.py` | GUI | ArduCopter 图形控制台（主程序） |
| `apm_control_test.py` | 库 | APM 自动化测试与控制函数（测试 1~14） |
| `apm_control.py` | CLI | APM 命令行控制脚本，可解析航点任务文件 |
| `px4_control_ui.py` | GUI | PX4 图形控制台 |
| `px4_control_test.py` | 库 | PX4 测试与控制函数 |
| `px4_control.py` | CLI | PX4 命令行控制脚本 |
| `ai_pid_tune.py` | GUI | AI PID 调参独立界面 |
| `ai_tune_core.py` | 库 | 调参核心引擎（参数表、飞行试验、日志分析、DeepSeek 调用） |
| `CHANGELOGS.MD` | 文档 | 逐版本开发日志 |

---

## 图形控制台

两个 UI 共用同一套架构：**单读线程收包 → 遥测显示 + 捕获队列**，
指令动作在独立工作线程排队执行，GUI 线程与 MAVLink 线程完全隔离。

### 页签（8 页）

| 页签 | 用途 |
|------|------|
| **安全设置** | 地理围栏（离 HOME 距离/相对高度）、低电量保护、心跳丢失监控、**位置估计就绪状态**、飞行日志录制、急停 |
| **目标跟踪曲线** | 高度 / 地速 的实际值 vs 目标值实时曲线 |
| **小地图** | 多底图源、WGS-84 无偏移叠加、HOME / 航迹 / 当前位置、滚轮缩放 / 拖拽 / 回中 / 跟随、瓦片磁盘缓存（离线可复用） |
| **参数读写** | 飞控参数读取 / 写入 + 历史记录 |
| **单点飞行(GUIDED)** | 单点目的地解锁起飞，支持就地改点、**立即生效** |
| **航点任务(AUTO)** | 解析 / 上传 / 一键执行航点任务，支持 `DROP` 抛投行 |
| **软遥控** | 4 通道 RC 覆盖滑条（仅地面调试，任务运行时自动释放） |
| **信号速率** | 各 MAVLink 消息频率统计、最近指令表 |

### 实时遥测

模式 / 解锁 / GPS(fix、卫星、HDOP) / 经纬度 / 相对与绝对高度 / 速度 NED /
地速 / 姿态 RPY / 电池 V·A·% / CPU 负载 / EKF 方差 / RC / 链路 msg·s⁻¹ /
飞控 STATUSTEXT、**EKF 状态标志**（位置估计是否就绪）。

### 安全能力

- **急停**：一键 `LOITER` 悬停（快捷键 `F9`）+ 立即 `RTL` / `LAND`，直发不排队
- **地理围栏**：离 HOME 距离与相对高度限制，越界告警，可选自动 RTL
- **低电量保护**：告警电压 / 触发电压 / 电量阈值，可选 `LAND` / `RTL`
- **心跳丢失监控**：超时红字告警（阈值可调）
- **解锁预检门禁**：`GUIDED` / `LOITER` / `AUTO` 等位置模式解锁前，先等待
  GPS 3D 定位 + EKF 位置估计就绪；未就绪**不发送 ARM**，并把飞控返回的
  `PreArm` 文本显示在安全页；解锁被拒时按关键词（如 `Need Position Estimate`）
  自动等待就绪后重试
- **界面看门狗**：主线程卡死时生成 `ui_hang_dump.txt` 记录阻塞调用栈，便于诊断

### 航点任务与抛投

- 任务文件为纯文本，每行一个航点：`纬度,经度,高度`
- `DROP` 行展开为 `DO_SET_SERVO`(触发) + `NAV_DELAY`(保持) + `DO_SET_SERVO`(回位)；
  也可用实时 `COMMAND_LONG` 单次触发（每步校验 ACK）
- 前提：飞控参数 `SERVO<通道>_FUNCTION = 0 (Disabled)`
- 上传后切 `AUTO` 按序飞行；支持末项动作 `none` / `rtl` / `land`
- 任务协议按 ArduPilot 约定：`seq0` 专供 home，故线上多发一项，首个航点不会被吞
- 进 `AUTO` 前自动把任务指针复位到第 1 项（`MAV_CMD_DO_SET_MISSION_CURRENT`，失败回退
  `MISSION_SET_CURRENT`），重复执行任务总是从起点开始

### 小地图底图

| 底图 | 坐标系 | 密钥 |
|------|--------|------|
| Esri 卫星影像（默认） | WGS-84 | 无需 |
| 天地图 矢量 / 影像 | WGS-84 | 需免费 `tk`，写入 `tianditu_key.txt` |
| 高德 | GCJ-02 | 备选 |

瓦片落盘到 `map_tiles/`，断网后仍可复用；无 GPS 时默认定位北京。

---

## 自动化测试（`apm_control_test.py`）

| # | 测试项 | # | 测试项 |
|---|--------|---|--------|
| 1 | 连接 | 8 | 悬停 |
| 2 | 解锁 | 9 | 降落 |
| 3 | 模式切换 | 10 | 上锁 |
| 4 | 起飞 | 11 | AutoTune |
| 5 | 前进速度 | 12 | 状态巡检 |
| 6 | 侧移速度 | 13 | 电池状态 |
| 7 | 升降速度 | 14 | 弹跳测试 |

关键设计：

- **解锁**：解锁期间强制 RC 油门 `1000`，结束自动释放；同时读取 `STATUSTEXT`
  打印 `PreArm` 原因；解锁后写入 `DISARM_DELAY`（默认 60s）防失控自锁
- **上锁**：校验 ACK 与心跳，失败自动重发强制上锁
- **起飞**：`arm_and_takeoff()` 一体调用，ACK 非 `ACCEPTED` 直接报错
- **着陆**：等自动 `DISARM`，仍解锁才发 `DISARM`

---

## 快速开始

### 环境要求

- Python **3.8+**（含 `tkinter`，Windows 官方安装包自带）
- 串口驱动（CH340 / CP210x / FTDI 等，按你的数据线而定）

### 安装

```bash
git clone https://github.com/tyjsnz/apm_px4_control_aitune.git
cd apm_px4_control_aitune
pip install pymavlink
```

可选（仅小地图需要）：

```bash
pip install pillow requests
```

### 运行

```bash
python apm_control_ui.py        # APM 图形控制台
python px4_control_ui.py        # PX4 图形控制台
python ai_pid_tune.py           # AI PID 调参
```

### 连接飞控

1. USB 连接飞控，端口选 `COMxx`（Windows），波特率常用 `57600`
2. 勾选「自动识别」或直接指定 APM / PX4
3. 点击 **连接**，成功后遥测面板开始刷新、日志窗口出现心跳

> 已连接端口会被占用，若连接失败先确认 Mission Planner / QGroundControl
> 等其他地面站已断开。

---

## AI 调参工具

`ai_pid_tune.py` 通过 **DeepSeek Chat API** 生成参数建议，闭环流程：

1. 填入 API Key（本地保存在 `ai_tune_config.json`，**已被 gitignore，不会上传**）
2. 填写机架规格（尺寸、重量、KV、桨距、电池、电调限流等）
3. 试飞录波形 → CSV 落盘到 `flight_logs/`
4. 点 **AI 生成** 得到第一组参数 → 应用 → 再录一段 → **AI 迭代**
5. 每轮参数写入 `ai_tune_backups/`，可随时回滚

参数范围表在 `ai_tune_core.py` 顶部（`APM_COPTER` / `PX4_MC`），
AI 输出会被**夹紧到安全范围**内再应用。

支持模型下拉：`deepseek-flash`、`deepseek-v4-pro`。

---

## 常见问题

### `PreArm: Need Position Estimate`

飞控 EKF 尚未完成位置估计，**这是强制解锁检查，无法跳过**。处理方式：

1. **室外**等 GPS 3D 定位 + EKF 收敛（通常 20~60s），看安全页的「位置就绪」行
2. 确认 `AHRS_EKF_TYPE = 3`、`EK3_ENABLE = 1`（EKF3 已启用）
3. 需要放宽时把 `FS_EKF_THRESH` 从 `0.6`（严格）调到 `0.8`（默认）或 `1.0`（宽松）
   ——注意这会降低对位置估计质量的把关，请谨慎
4. **急用可点「定高解锁」**走 `ALT_HOLD`，该模式不依赖位置估计（无法定点）

> `ANGLE_MAX` / `ATC_ANG_LIM_TC` / `WPNAV_ACCEL` / `WPNAV_JERK` 等姿态与
> 导航调参**不参与解锁自检**，调它们无效。

### 界面卡死后没有反应

程序会写 `ui_hang_dump.txt` 记录阻塞时的调用栈，附带该文件提 issue 即可定位。

### 地图不显示

首次加载需要联网下载瓦片；下载后缓存在 `map_tiles/`，之后可离线显示。

### 串口 `PermissionError` / 连接超时

端口被其他程序占用，或波特率不符。先关掉其他地面站，确认飞控波特率。

---

## 目录结构

```
.
├── apm_control_ui.py        # APM 图形控制台（主程序）
├── apm_control_test.py      # APM 测试与控制函数库
├── apm_control.py           # APM 命令行脚本
├── px4_control_ui.py        # PX4 图形控制台
├── px4_control_test.py      # PX4 测试与控制函数库
├── px4_control.py           # PX4 命令行脚本
├── ai_pid_tune.py           # AI 调参 GUI
├── ai_tune_core.py          # 调参核心引擎
├── CHANGELOGS.MD            # 开发日志
├── README.md
├── LICENSE                   # MIT
└── .gitignore
```

运行时生成（不入库）：`ai_tune_config.json`、`apm_ui_config.json`、
`tianditu_key.txt`、`flight_logs/`、`map_tiles/`、`ai_tune_backups/`、`ui_hang_dump.txt`

---

## 技术要点

- **MAVLink 消费模型**：单读线程持续 `recv_match`，普通数据进遥测字典，
  测试/等待函数通过"捕获队列"消费自己关心的消息，避免多线程抢包
- **指令排队**：所有用户点击动作 post 到工作线程队列串行执行，
  急停类指令绕过队列直发
- **GIL / 阻塞隔离**：读线程有超时轮询，不会永久阻塞主线程
- **UI 看门狗**：周期性检测主线程心跳，超时导出堆栈

---

## 贡献

Issue 与 PR 均欢迎。提交前请至少跑通：

```bash
python -m py_compile apm_control_ui.py apm_control_test.py px4_control_ui.py
```

---

## 许可证

[MIT](LICENSE) © tyjsnz
