# px4/api

多地面站(数传)无人机控制服务。基于 FastAPI + pymavlink，功能对齐
`px4_control_ui.py`(PX4 读取与手动控制) 与 `px4_control.py`/`px4_control_test.py`
(自动化控制)：

- 同时连接多个地面站(串口 / TCP / UDP 数传)，每条链路独立读线程与遥测快照
- 查询飞控遥测：飞行状态 / GPS / 速度姿态 / 电池系统 / 控制信号 RC(左栏数据)
- 接收外部系统持续推送的修正坐标，向指定(或全部)地面站下发 OFFBOARD 位置指令
- 单点飞行(goto)、航点任务(mission，支持抛投)、起飞/解锁/切模式/RC 交还
- 急停(LOITER)、立即 RTL、立即 LAND，均直发 MAVLink 不排队

与 `apm_control_api` 的差异见文末「PX4 语义」。

## 启动

```bash
# 在仓库根目录下启动 (px4.api 是包, app.py 使用了相对引用)
python -m px4.api                # 默认 0.0.0.0:8000
python -m px4.api --port 8080
python -m px4.api --reload       # 开发热更新
```

依赖：`pip install -r px4/api/requirements.txt`
(本仓库已装 fastapi / uvicorn / pymavlink / pyserial / httpx)

启动后打开 `http://127.0.0.1:8000/docs` 可交互调试全部接口。

连接校验：对端心跳 `autopilot` 必须是 `MAV_AUTOPILOT_PX4`，否则 502 拒绝连接
(与 `px4_control.mavconnect` 一致)。

## 配置(环境变量)

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `PX4_API_HOST` / `PX4_API_PORT` | `0.0.0.0` / `8000` | 监听地址(也可用 `--host/--port`) |
| `PX4_API_DEFAULT_BAUD` | `57600` | 串口默认波特率 |
| `PX4_API_CONNECT_TIMEOUT` | `30.0` | 连接地面站超时(秒) |
| `PX4_API_AUTO_CONFIRM` | `1` | 自动确认解锁(服务端无交互输入) |
| `PX4_API_GUIDANCE_RATE` | `10.0` | 引导流默认下发频率 Hz(1–30) |
| `PX4_API_GUIDANCE_TTL` | `0.0` | 引导流存活秒数，0=不限，到期自动停 |
| `PX4_API_ALT_FRAME` | `relative` | 引导数据默认高度帧 relative/amsl |
| `PX4_API_MIN_CONFIDENCE` | `0` | 低于该置信度的数据直接丢弃 |
| `PX4_API_GOTO_THRESHOLD` | `2.5` | goto 到达判定水平距离(米) |
| `PX4_API_GOTO_ALT` | `20.0` | goto 缺省相对高度(米) |
| `PX4_API_TAKEOFF_ALT` | `5.0` | 任务自动起飞缺省高度(米) |
| `PX4_API_MISSION_TIMEOUT` | `900.0` | 航点任务总超时(秒) |
| `PX4_API_LANDING_TIMEOUT` | `300.0` | RTL/LAND 后台等待降落超时(秒) |

## 互斥规则

- 每条链路同一时刻只跑一个 `goto/mission/takeoff/arm/...` 任务，冲突返回 **409**
- 启动引导流会取消正在执行的任务并停掉 Offboard 设定值流；`goto/mission/takeoff`
  启动会停止引导流
- 同一 `goto` 重复调用走“改点”路径(返回 `retarget: true`)，不报忙
- `stop/rtl/land` 取消任务、停引导、停设定值流后，才直发模式切换

## 接口

### 链路与遥测

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/health` | 服务状态 |
| GET | `/api/v1/ports` | 枚举本机可用串口 |
| POST | `/api/v1/links` | 连接地面站，`{id?, port, baud?, name?}` → 201 |
| GET | `/api/v1/links` | 已连接链路列表 |
| GET | `/api/v1/links/{id}` | 单条链路详情 |
| DELETE | `/api/v1/links/{id}` | 断开(取消任务、停引导、停设定值流) |
| GET | `/api/v1/links/{id}/telemetry` | 全部遥测(左栏各段) |

`telemetry` 返回：`flight_status`(mode/armed/心跳/状态文本)、
`gps`(fix/卫星/经纬高/HOME)、`velocity_attitude`(地速/空速/俯仰滚转偏航)、
`battery_system`(电压电流剩余 + `pos_flags` + `position_estimate` 就绪判定)、
`control_signals`(RC 1–8 通道、指令与消息速率)、`labels`(格式化字符串)、
`link_ok`、`task`、`guidance`。

位置就绪判据：`fix>=3` + 卫星数 >= `POS_READY_MIN_SATS` + 已收到
`GLOBAL_POSITION_INT`(位 `0x1`)，即 `tct.position_ready_state()`。

### 外部修正坐标持续引导

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | `/api/v1/links/{id}/guidance` | 向该地面站推送修正数据 |
| POST | `/api/v1/guidance` | 同上，`link_id` 可为 `all`(全部地面站) |
| GET | `/api/v1/links/{id}/guidance` | 引导流状态(目标、已发帧数、状态) |
| DELETE | `/api/v1/links/{id}/guidance` | 手动停止引导流 |

请求体即外部系统样例：

```json
{"id":916,"lat":35.279691711186814,"lon":105.2478624923401,"alt":800,
 "vx":30,"vy":-10,"vz":-2,"type":32,"track":true,"confidence":88}
```

语义与约束：

- `track=true` 持续流(默认 10Hz)；`track=false` 只发一次最终点(重复 5 帧)
- 坐标帧 `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`；纯位置掩码 `0x7F8`，
  位置+速度前馈掩码 `0x7C0`(PX4 type_mask: 置 1 = 忽略该组)
- 引导线程自己发设定值流，进入前先按 `px4_control.OFFBOARD_RATE_HZ`(10Hz)
  预热 `OFFBOARD_WARMUP_SEC`(1.5s)，再直发 `MAV_CMD_DO_SET_MODE` 切
  **OFFBOARD**；`ensure_offboard=true`(默认) 每 2s 检查一次被切走就切回来
- `ensure_offboard=true` 时请保持 `rate_hz >= 3`：PX4 OFFBOARD 要求设定值
  流 >2Hz，`rate_hz=1` 只在预热期提速，之后会被飞控判为失流
- `alt_frame`: `relative` 默认按相对高度；`amsl` 用 home 高程换算成相对高度下发
- `vel_frame`: `ned`(默认，与 MAVLink 一致) / `enu`
- `confidence < min_confidence` → **422**
- 飞机未解锁且 `auto_takeoff=false` → **422**；`auto_takeoff=true` 先解锁起飞到
  `takeoff_alt` 再进入引导
- 引导流必须覆盖上层需求时建议频率 ≥ 5Hz(官方要求持续流由上层维护)

### 单点飞行 / 航点任务

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | `/api/v1/links/{id}/goto` | 单点飞行 → 202 `{task}` |
| POST | `/api/v1/links/{id}/mission` | 航点任务 → 202 `{task}` |
| GET | `/api/v1/links/{id}/tasks` | 任务列表(含日志尾部) |
| GET | `/api/v1/links/{id}/tasks/{tid}` | 单个任务详情与日志 |

`goto`：`{lat, lon, alt, speed?, auto_takeoff=true, threshold?, timeout?}`
未解锁先自动起飞，否则先切 `POSCTL`；随后走 `px4_control.goto()`(OFFBOARD
位置设定值流) 反复刷新目标点直到到达判定。

`mission`：`{waypoints:[{lat,lon,alt,drop?}], end_action: none|rtl|land,
auto_takeoff=true, takeoff_alt?, timeout?}`，`drop` 支持抛投
(`{channel, pwm_on, pwm_off, hold_ms}`)。执行顺序：上传任务 → (必要时)起飞 →
`reset_mission_to_start` → 切 **MISSION** → `wait_auto_mission` 监控到完成。

### 急停与立即动作

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | `/api/v1/links/{id}/stop` | 急停 → LOITER(默认) |
| POST | `/api/v1/links/{id}/rtl` | 立即返航 |
| POST | `/api/v1/links/{id}/land` | 立即降落 |

三者均先取消任务/停引导/停设定值流，再**直发**模式切换(不排队，不读心跳)。
RTL/LAND 若飞机仍在空中，会自动启动后台 `settle_land` 任务：等高度降到 1m 后
确认触地、等 `COM_DISARM_LAND` 自动上锁(可在响应 `settle_task` 中查看)。

### 基础控制

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | `/api/v1/links/{id}/arm` | 解锁 `{mode: POSCTL\|ALTCTL}` |
| POST | `/api/v1/links/{id}/disarm` | 上锁 `{force?}` |
| POST | `/api/v1/links/{id}/takeoff` | 起飞 `{alt, mode: POSCTL\|ALTCTL}` |
| POST | `/api/v1/links/{id}/mode` | 切模式 `{mode}`(PX4 名或 APM 别名) |
| POST | `/api/v1/links/{id}/rc/handover` | RC 覆盖收尾，交还物理遥控器 |

切模式可用：`MANUAL/STABILIZED/ACRO/RATTITUDE/ALTCTL/POSCTL/OFFBOARD/
READY/TAKEOFF/LOITER/MISSION/RTL/LAND`，也接受别名
(`GUIDED`→`POSCTL`, `ALT_HOLD`→`ALTCTL`, `AUTO`→`MISSION`,
`STABILIZE`→`STABILIZED`, `LOITER/RTL/LAND` 同名)。

## curl 示例

```bash
# 连接串口地面站
curl -X POST localhost:8000/api/v1/links -H 'Content-Type: application/json' \
  -d '{"id":"gcs1","port":"/dev/ttyUSB0","baud":57600}'
# 连接 TCP/UDP 地面站
curl -X POST localhost:8000/api/v1/links -H 'Content-Type: application/json' \
  -d '{"id":"gcs2","port":"udp:127.0.0.1:14550"}'

# 遥测
curl localhost:8000/api/v1/links/gcs1/telemetry

# 外部修正坐标持续引导(未解锁则自动起飞)
curl -X POST localhost:8000/api/v1/links/gcs1/guidance -H 'Content-Type: application/json' \
  -d '{"lat":35.27969,"lon":105.24786,"alt":15,"vx":30,"vy":-10,"vz":-2,
       "track":true,"confidence":88,"auto_takeoff":true,"takeoff_alt":15}'
curl localhost:8000/api/v1/links/gcs1/guidance     # 查看引导状态
curl -X DELETE localhost:8000/api/v1/links/gcs1/guidance

# 单点飞行
curl -X POST localhost:8000/api/v1/links/gcs1/goto -H 'Content-Type: application/json' \
  -d '{"lat":31.2309,"lon":121.4742,"alt":20,"speed":8}'

# 航点任务
curl -X POST localhost:8000/api/v1/links/gcs1/mission -H 'Content-Type: application/json' \
  -d '{"waypoints":[{"lat":31.2309,"lon":121.4742,"alt":20},
                    {"lat":31.2312,"lon":121.4747,"alt":20}],
       "end_action":"rtl"}'

# 查看任务日志
curl localhost:8000/api/v1/links/gcs1/tasks
curl localhost:8000/api/v1/links/gcs1/tasks/goto-12345678

# 急停 / RTL / LAND
curl -X POST localhost:8000/api/v1/links/gcs1/stop
curl -X POST localhost:8000/api/v1/links/gcs1/rtl
curl -X POST localhost:8000/api/v1/links/gcs1/land

# 基础控制
curl -X POST localhost:8000/api/v1/links/gcs1/takeoff -H 'Content-Type: application/json' \
  -d '{"alt":8,"mode":"ALTCTL"}'
```

## 冒烟测试(无硬件)

内置假飞控(单进程 TCP，PX4 心跳/模式/任务/参数行为)，覆盖连接/遥测/引导/
置信度过滤/急停/单点/航点/RTL 落地/基础控制/断开：

```bash
python -m px4.api.tests.smoke_test        # 或
python px4/api/tests/smoke_test.py
```

单独起假飞控：`python -m px4.api.tests.fake_fc --port 15761`。
全部通过时输出 `✅ 冒烟测试通过:` 及覆盖项列表。

## 目录结构

```
px4/api/
├── app.py        # FastAPI 路由、链路注册、互斥规则
├── link.py       # 链路(唯一读线程 + 遥测快照 + 急停直发)
├── guidance.py   # 外部修正坐标持续引导流(OFFBOARD)
├── tasks.py      # 任务管理 + goto/mission/arm/软着陆等任务函数
├── models.py     # pydantic 请求模型
├── config.py     # 环境变量配置
├── tasklog.py    # 任务线程 print 输出路由(收集到各任务日志缓冲)
└── tests/        # fake_fc.py 假飞控 + smoke_test.py 冒烟测试
```

## PX4 语义(相对 apm_control_api 的差异)

- 模式名：`GUIDED`→`POSCTL`(引导/goto 走 `OFFBOARD`)、`ALT_HOLD`→`ALTCTL`、
  `AUTO`→`MISSION`、`STABILIZE`→`STABILIZED`；`/mode` 仍接受 APM 别名
- 引导请求字段与状态字段 `ensure_guided` 更名为 `ensure_offboard`
- PX4 不发 `EKF_STATUS_REPORT`：`battery_system.ekf_variance/ekf_flags`
  更名为 `pos_flags`(本模块的“位置消息到达”位掩码：`0x1`=已收到
  `GLOBAL_POSITION_INT`，`0x2`=已收到 `LOCAL_POSITION_NED`)；
  `labels.ekf` 恒为提示文案，`labels.posflags` 给出 `pos_flags` 十六进制
- 解锁起飞走 `tct.arm_vehicle`(预检门禁+现场确认) + `px4_control.takeoff`
  (`MIS_TAKEOFF_ALT` + `NAV_TAKEOFF`)：`tct.arm_and_takeoff`/
  `tct.takeoff_althold` 内部 `send_guided_takeoff` 调 `command_long_send`
  少传 `param7` 会 TypeError(`px4_control_test.py:822-825`，已报告待修)
- `track=false` 只发 5 帧一次性目标点，不负责把飞机切进 OFFBOARD；
  需要持续跟踪请用 `track=true`
