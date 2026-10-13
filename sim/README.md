# FPV 无人机仿真台

从装配参数到稳定高效飞行的**全栈物理仿真**：结构强度、空气动力、流体、能量链、PID 控制、诊断报告、首飞流程、目标反向求解，一键输出。零 npm 依赖（Node 内置模块 + 原生 Canvas），单次仿真约 0.4~3s（含优化建议与反向求解）。

## 快速开始

```bash
npm start            # 默认 http://localhost:8080 （PORT=xxxx 可改）
npm test             # 单元 + 端到端 + 数据契约 回归
npm run scan         # PID 参数扫描（调参工具）
```

浏览器打开 `http://localhost:8080`：选类型（**多旋翼 / 垂起火箭 VTOL / 固定翼**）→ 填电机/桨/电池/目标速度 → 运行仿真。左侧"整机演示"可一键加载 8 套预设（5寸竞速 / 7寸长航时 / 12S八轴载重 / 1S牙签反例 / 1000mm飞翼 / 2m滑翔机 / **8寸垂起火箭 / 10寸垂起火箭**）。

## API

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/simulate` | 传 JSON 配置，返回完整仿真结果（可带 `tuning` 覆盖半经验系数/判据阈值） |
| GET | `/api/presets` | 部件库（机架/机翼/电机/电调/化学/桨材料）+ 演示配置 |
| GET | `/api/tuning` | **可调参数表**：参数定义（key/分组/范围/步进/中文说明）+ 默认值 |
| GET | `/api/demo?preset=default` | 直接返回某预设的仿真结果 |
| GET | `/api/health` | 健康检查 |

示例：

```bash
curl -X POST http://localhost:8080/api/simulate \
  -H "Content-Type: application/json" \
  -d '{"type":"multirotor","frameId":"fpv5","cells":4,"capacity_mAh":1300,
       "cRating":100,"motorType":"2207","kv":1750,"propD":127,"propP":109,
       "propB":3,"target_kmh":120}'
```

主要输入字段：`type`（multirotor/vtol/fixedwing）、`frameId`/`wingId`/`rocketId`、`rotorCount`、`motorType`、`kv`、`escType`、`propD/propP/propB/propMaterial`、`cells`、`capacity_mAh`、`chemistry`（lipo/lihv/liion）、`cRating`、`wireAwg/wireLen_cm`、`target_kmh`、`payload_g`、`altitude`、`tempC`、`massOverride_g`、`arm_m`、`wingSpan/wingArea`、`bodyLength_m/bodyD_mm/finSpan_m/finArea_m2`（火箭机型）。留空即用物理默认值。另可选 `tuning`（对象，见下文「调参表」）。

返回结果关键字段：`cases`（悬停/满油门/最高速/目标/速度扫描/续航/转换走廊/垂直爬升）、`issues`（分级诊断 + 修复建议，命中方案时带 `solutionRef` 跳转）、`solutions`（**目标驱动反向求解**：方案列表/Pareto 前沿/灵敏度/扫描点数）、`report`（中文分析报告）、`plan`（首飞流程）、`structure`（机臂应力/挠度/固有频率/桨离心/螺钉）、`dynamics`（转动惯量/执行器/PID整定/推力曲线）、`energy`（电调-铜损-铁损-气动能量链）、`fluid`（雷诺数/噪音/地面效应）、`mass`、`battery`、`vtol`（转换走廊/悬停稳定/火箭几何）、`stability`（静稳定/短周期/浮沉/舵面权限/滚转权限/偏航刚度）、`navigation`（巡航/转弯/抗风/盘旋/航程）、`optimize`（悬停效率基准/单项建议/组合最优/扫描点数）、`tuningApplied`（本次相对默认值被改动的系数清单）、`thresholds`（本次实际生效的判据阈值，前端 KPI 着色与诊断同源）。

## 调参表（半经验系数 / 判据阈值）

仿真里的公式**并非同一性质**，按可调性分五层。前两层不可调，调了就错：

| 层 | 性质 | 例子 | 可调？ |
|---|---|---|---|
| 1 | 物理定律 | 动量理论反解诱导速度、翼素 `dT=q(Cl·cosφ−Cd·sinφ)dr`、悬臂弯矩 `M=t·L`、ISA 大气、基尔霍夫 | ❌ |
| 2 | 教科书标准式 | 普朗特叶尖损失、Helmbold 低展弦比升力线斜率 | ❌（形式固定） |
| 3 | 半经验系数 | 阻力极曲线 `0.0075Cl²+0.004|Cl|+0.35sep²`、压缩性 `1.3(M−0.28)²`、弯度偏移 4°、诱导速度松弛 0.55、噪音 `…−33dB`、地面效应 `0.18`、机身阻力系数表、升力体 `bodyFactor 0.55` | ✅ |
| 4 | 工程判据阈值 | 悬停油门 0.55/0.68/0.18、推重比 1.0/1.6、电机热 0.55、桨尖马赫 0.72、噪音 75dB、机臂 SF 1.5/3、静稳定 5%~25%、升阻比 8 | ✅ |
| 5 | 数据库 | 电机库（2207：R₀=45mΩ、PlossCont=95W）、机翼库（cd0/e）、火箭库 | ✅（改数据） |

以上第 3/4 层共 **47 个参数**集中在 `core/tuning.js`，每项带分组、单位、取值范围、步进与中文说明。**默认值与历史硬编码值逐字相同**——`test/selftest.js` 的校准锚点（5寸 4S 1750KV 5043 = 852g/21.5A/27199rpm、悬停 4.34 g/W）就是它的守门条件。

用法（扁平 key 与嵌套写法等价）：

```bash
curl -X POST http://localhost:8080/api/simulate \
  -H "Content-Type: application/json" \
  -d '{"type":"multirotor","frameId":"fpv5","cells":4,"capacity_mAh":1300,
       "motorType":"2207","kv":1750,"propD":127,"propP":109,"propB":3,
       "tuning":{"bemt":{"kInduced":0.02},"carbon":{"sigmaAll_MPa":300},"diag":{"noise10m":60}}}'
```

分组与语义：

- **`blade` 桨叶气动（BEMT 半经验）** — `kInduced`/`kViscous`/`kSeparation`/`sepOnset`/`kMach`/`machOnset`/`camber_deg`/`viRelax`。决定推力-电流-效率曲线形状。
- **`material` 结构材料** — `carbon.E_GPa`/`carbon.sigmaAll_MPa`/`carbon.density`/`screw.tauAll_MPa`。换碳板牌号（T700 / M40J / 玻纤）时改这里；结构反解（`solver.js`）与判定（`dynamics.js`）**共用同一张表**，不会出现反解厚度与实测判定不一致。
- **`aero` 整机气动** — 六种机型阻力系数、`bodyFactor`、`groundK`、噪音标定 `noiseOffset_dB`/`noiseVelExp`/`noiseSizeExp`。**噪音式是半经验的，`-33dB` 就是标定常数：拿实测数据拟合时改这一项。**
- **`diag` 诊断判据阈值** — 只决定「什么算合格」，**不改变任何物理读数**（`test/contract.js` 逐位断言这一点）。
- **`solver` 反向求解器** — `speedMargin`（默认 1.15 倍目标速度）、推重比/安全系数目标、评估预算 `maxCalls=110` 与墙钟上限 `deadline_ms=9000`。

约束与安全：

- 越界值被**夹紧**到 `[min,max]`，非数值回退默认，未知 key 被忽略——非法 `tuning` 不会让请求失败。
- `simulate()` 是纯同步的（无 `await`），调参表用模块级单例穿透到深层物理函数，**一次仿真内不会被交错改写**，并发请求安全。
- 每次请求结束都会把调参表重置为默认再应用覆盖，因此**带 `tuning` 的请求之后，不带 `tuning` 的请求结果与首次逐位一致**（有回归断言）。
- 前端「调参表」折叠面板动态渲染全部 47 项，只把**用户改过的**参数放进请求体——未动的项走服务端默认，保证默认路径与改动前零差异；「对比默认结果」按钮会再跑一次全默认配置，逐指标给出 Δ 与百分比。


## 物理模型

- **桨叶 BEMT**：叶素动量理论，带前前进来流（固定翼）、诱导速度、Figure of Merit、桨尖马赫数。
- **电机/电调/电池耦合**：以 ω 为主变量直接求解工作点（单点约 0.5ms）；电机 R₀ 温度、铜损/铁损/摩擦分账；电调导通电阻 + 开关损耗；电池内阻 + OCV(soc) + C 数上限校验。
- **结构**：机臂碳板弯曲应力/挠度/一阶固有频率（对比桨叶通过频率避振）、桨叶离心应力、螺钉剪切。
- **控制**：执行器一阶滞后 τ → 速率环 PID **主导实极点配置**（P=p/K, D=pτ/K, I=Pp/150，积分权限 10%），输出阶跃超调/调节时间与 8% 常值扰动抗扰残差。
- **固定翼 / 火箭巡航**：极曲线（cd0 + 诱导阻力）、失速、升阻比、螺距速度天花板、爬升率；火箭 = 尾翼（Helmbold 低展弦比）+ 细长机身升力体（cross-flow，bodyFactor 0.55）。
- **垂起火箭（Tailsitter VTOL）**：双模飞行。悬停 = 多旋翼模态（推力向上，机身竖直）；巡航 = 固定翼模态（机身水平，尾翼+机身产生升力）。
  - **转换走廊**：机头 90°→0° 连续姿态，解竖直/水平力平衡（T·sinφ + L·cosφ = W；T·cosφ − L·sinφ − D = 0）得每姿态平衡速度 V*、需用推力、推力裕度、差动推力控制权限。
  - 安全判据：推力承载段（φ≥75°）、机翼有效段（未失速且 V≥失速速度）、尾翼失速过渡带（仅靠差动推力控姿，建议大推力快速穿越）。
  - **悬停稳定性**：重心在推力面下 → 单摆稳定（回复频率）；上方 → 倒立摆（需飞控带宽 ≥3Hz）。差动推力滚转/偏航角加速度。
- **增稳 & 导航（固定翼 & 火箭）**：静稳定裕度 SM、短周期/浮沉、升降舵配平权限、满副翼滚转速率、Cnβ 偏航刚度；转弯性能（20/30/45° 坡度半径/速率/失速余量）、抗风漂移、巡航功率/续航/航程、盘旋半径/续航。
- **悬停效率优化引擎**：坐标下降 + 桨径×KV 网格细化（约百次 BEMT），硬约束（推重比/悬停油门/电池C数/桨尖马赫/电机热/桨位），输出基准/单项建议/组合最优，量化收益与代价。
- **目标驱动反向求解引擎**（`core/solver.js`）：诊断出现 error/warn 时自动反向搜索参数空间，输出可一键回填的量化改造方案：
  - **算法 A 非线性反向搜索**：先单项改动直搜（最小改动优先），不足再坐标下降累积；阶段 3 用 kv×cells×propD 联合网格同时优化多项指标。搜索轴全部取市售规格（电机库/桨叶预设），电芯数受电机耐压标签约束。
  - **算法 B 解析反解**：结构安全系数按悬臂弯矩闭式反解碳板厚度（强度 t∝√SF、刚度 t∝∛SF 取大）；静稳定裕度按中性点/重心闭式反解重心位置。
  - **算法 C 灵敏度杠杆**：对每个搜索轴做有限差分，标注 `km/h/电芯+1S`、`dB/KV+100KV` 这类单位化杠杆并降序排列。
  - **算法 D Pareto 前沿**：速度 vs 续航冲突时给出续航优先/均衡/极速优先 3 个权衡点。
  - 方案含：目标当前值→目标值、改造清单、预测 after、质量代价、裕度、置信度（高/中/低）、副作用告警（新配置若引入基准没有的 error 会明确标注）；排序按 ①达标 ②无副作用 ③联合满足数 ④改动参数数 ⑤质量代价 ⑥裕度，取 Top5 + Pareto。
  - 评估预算受控（≤110 次 `simulate()` 轻量调用 + 墙钟上限），轻量评估共享缓存且带 `_skipSolver` 防递归；UI「改造方案」页渲染卡片与灵敏度/Pareto 图表，「应用此方案」一键回填表单重跑。
- **集中可调参数表**（`core/tuning.js` + `GET /api/tuning` + 侧栏「调参表」面板）：把上文所有半经验系数与判据阈值收编为 47 项可在线调整的参数，详见上文「调参表」章节。诊断与求解器**共用同一套 `diag` 阈值**，前端 KPI 着色读取结果里回传的 `thresholds`，三者不会出现口径不一致。
- 数据以真实 FPV 数据锚定（5寸 4S 1750KV 5043：满油门 ~850g、悬停 4.34 g/W @39% 油门；7寸 6S 950KV 7040：满油门 ~1750g）。

## 目录结构

```
core/      物理核心（纯 ESM，无 DOM/Node 依赖，可直接移植浏览器）
  air.js       ISA 大气 / 黏度 / 雷诺数
  parts.js     电池/电机/电调/机架/机翼/火箭/桨叶数据库
  bemt.js      BEMT + 工作点求解（含电池受限满油门 fullPoint）
  aero.js      机身阻力 / 翼型极曲线 / 火箭气动 / 噪音 / 地面效应
  scenarios.js 多旋翼 & 固定翼 & VTOL 工况求解（悬停/满油门/速度扫描/续航/转换/垂直爬升）
  dynamics.js  结构/惯量/执行器/PID 整定
  navigation.js 固定翼/火箭增稳 + 导航工况（转弯/抗风/航程/盘旋）
  optimize.js  悬停效率优化建议引擎（坐标下降 + D×KV 网格）
  solver.js    目标驱动反向求解（单轴直搜/坐标下降/联合网格/解析反解/灵敏度/Pareto）
  analysis.js  诊断规则引擎 + 中文报告 + 首飞流程 + issue↔方案关联
  tuning.js    集中可调参数表（47 项半经验系数 + 判据阈值，含范围/步进/说明 + 越界夹紧）
  simulate.js  总入口 simulate(input) → 完整结果
  vtol.js      垂起火箭转换走廊 + 悬停稳定性
server/    零依赖 http 静态服务 + JSON API
public/    前端（原生 JS + Canvas 图表，无 CDN）
test/      selftest（单元）/ simtest（端到端）/ contract（数据契约）/ pidscan（调参）
```

## 部署到服务器

无构建步骤、无 node_modules，部署即复制目录。

**pm2（推荐）**

```bash
scp -r sim user@host:/opt/fpv-sim
ssh user@host 'cd /opt/fpv-sim && npm i -g pm2 && pm2 start server/index.js --name fpv-sim && pm2 save && pm2 startup'
```

**systemd**

```ini
# /etc/systemd/system/fpv-sim.service
[Unit]
Description=FPV Simulator
After=network.target
[Service]
WorkingDirectory=/opt/fpv-sim
ExecStart=/usr/bin/node server/index.js
Environment=PORT=8080
Restart=always
User=www-data
[Install]
WantedBy=multi-user.target
```

**Docker**

```dockerfile
FROM node:20-alpine
WORKDIR /app
COPY . .
ENV PORT=8080
EXPOSE 8080
CMD ["node", "server/index.js"]
```

**Nginx 反代（公网 HTTPS）**

```nginx
server {
  listen 443 ssl;
  server_name sim.example.com;
  location / { proxy_pass http://127.0.0.1:8080; proxy_set_header X-Real-IP $remote_addr; }
}
```

## 已知口径

- 速度统一使用 **km/h**。
- "最高速"对多旋翼是推力-阻力平衡点，对固定翼还需满足 ≥ 失速速度；不可达目标时诊断会给出螺距速度/推力/阻力哪一项是瓶颈。
- 电池 C 数默认：LiPo 50 / LiHV 70 / Li-ion 10；高推力配置请在表单里显式填 `cRating`（如竞速锂电 100C）。
- **满油门** = 占空比 100% **或** 电池总电流达放电上限（二者先到者，通过 `fullPoint()` 实现）。满油门电流/推力/功率均受此限，诊断会同时给出"需求电流"与"受限后实际值"。
