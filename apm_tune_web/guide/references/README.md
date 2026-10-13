# 导弹制导与拦截 全流程参考资料

> 一套面向**学习与工程查阅**的专题资料：从发射前地面准备，到中段修正、导引头截获，再到末段导引与杀伤判定。
> 每个专题统一按 **怎么做 / 为什么 / 结果怎么样** 三问展开，并给出对应文献编号（`[Rxx]`，总表见下）。
>
> 配套可运行仿真：本目录位于 `guidance_sim/references/`，仿真程序在 `../`，
> 帮助文档在 `../docs/`（`docs/05_中段制导与拦截点计算.md` 与本资料 `06` 互为详略）。

---

## 一、目录与阅读路线

| # | 文件 | 主题 | 关键图 |
|---|------|------|--------|
| 01 | [全流程总览：从发射到命中](01_全流程总览_从发射到命中.md) | 阶段划分 × 信息来源 × 各段算法；总纲 | [fig01](figures/fig01_phases.png) |
| 02 | [发射前准备与地面引导](02_发射前准备与地面引导.md) | 初始对准、参数装订、指令/驾束/照射三种地面引导回路 | [fig05](figures/fig05_guidance_loops.png) |
| 03 | [路径规划与航迹生成](03_路径规划与航迹生成.md) | A*/RRT/势场/凸优化，及与**拦截点解算**的本质区别 | [fig07](figures/fig07_layers.png) |
| 04 | [位置修正：地面与卫星导航](04_位置修正_地面与卫星导航.md) | INS 漂移、雷达/数据链修正、GNSS、地形匹配 | [fig04](figures/fig04_navigation.png) |
| 05 | [组合导航与状态滤波](05_组合导航与状态滤波.md) | 卡尔曼/EKF/UKF、误差状态模型、目标状态估计 | — |
| 06 | [中段制导与拦截点计算](06_中段制导与拦截点计算.md) | 碰撞三角形、迭代 TOF、ZEM、截获窗口（**核心章**） | [fig02](figures/fig02_collision.png) · [fig03](figures/fig03_acquisition.png) · [fig08](figures/fig08_zem.png) |
| 07 | [导引头与目标探测](07_导引头与目标探测.md) | 雷达/红外/可见光成像导引头、搜索-截获-跟踪、抗干扰 | [fig03](figures/fig03_acquisition.png) |
| 08 | [末段导引策略·经典方法](08_末段导引策略_经典方法.md) | PN / TPN / APN / PP / OGL / 落角约束（**核心章**） | [fig06](figures/fig06_pn_pp.png) |
| 09 | [末段导引策略·现代与智能方法](09_末段导引策略_现代与智能方法.md) | 微分对策、滑模、轨迹整形、协同、强化学习 | — |
| 10 | [工程实践：场景·指标·案例](10_工程实践_场景指标与案例.md) | 交会几何、脱靶量/CEP/杀伤概率、调参与排错 | [fig09](figures/fig09_miss.png) |

### 三条阅读路线

1. **先建全局（推荐首读）**：01 → 06 → 08 → 10。
   先知道全程有哪几段，再深入“看不见目标时怎么飞”（中段）与“看见之后怎么打”（末段），最后落到指标。
2. **补导航功底**：04 → 05 → 06。
   位置修正是中段一切计算的前提：不知道自己在哪，算出的拦截点毫无意义。
3. **做仿真/写代码**：01 → 06 → 08 → [../README.md](../README.md)。
   每章末尾都有「动手实验」，可直接在 `guidance_sim` 里复现。

---

## 二、全文统一的“三问”模板

每个专题都尽量按下面的格式写，方便横向比较：

| 分栏 | 回答的问题 | 落到什么内容 |
|------|-----------|--------------|
| **怎么做** | 具体步骤/公式/伪代码是什么？ | 数学式、算法流程、参数含义 |
| **为什么** | 为什么必须这么做？不做会怎样？ | 物理直觉 + 数学依据 + 工程约束 |
| **结果怎么样** | 效果如何、代价是什么？ | 定量结论、典型数据、失效模式、文献结论 |

**符号约定**（全文通用，与仿真代码一致）：

- `λ` 视线角（line-of-sight angle），`λ̇` 视线转率；`ψ` 导弹弹轴/速度方向角
- `R` 弹目距离，`Vc = Ṙ < 0` 接近速度，`Vm`/`Vt` 弹/目速度
- `N` 导航比（navigation ratio），`a_cmd` 过载指令，`Gmax` 可用过载上限
- `TOF` 飞行时间（time of flight），`PIP` 预测拦截点（predicted intercept point）
- `ZEM` 零 effort 脱靶量（zero-effort miss），`R_acq` 截获距离门限，`FOV` 导引头视场（全角）
- `R_hit` 杀伤半径，`CEP` 圆概率误差，`σ` 标准差

---

## 三、文献总表 `[Rxx]`

> **核验状态说明**（诚实标注，避免引用错误）：
> ✅ = 已通过公开检索核实到书名/版本/出处；📖 = 公认经典文献，条目信息按通行说法给出，
> **出版年/卷期建议在正式引用前再核对一次**；🔍 = 建议以你自己的检索结果为准（该方向文献更新快）。

### A. 制导与控制（导弹总体）

| 编号 | 文献 | 状态 | 主要用于 |
|------|------|------|----------|
| R01 | Zarchan P. *Tactical and Strategic Missile Guidance*, 7th ed.（分卷1引言卷/卷2提高卷）, AIAA Progress in Astronautics and Aeronautics, 2019 | ✅ | 01/06/07/08/09/10，PN、OGL、落角、微分对策的权威主线 |
| R02 | Siouris G.M. *Missile Guidance and Control Systems*. Springer, 2004 | ✅ | 02/07，导引头与系统级综述 |
| R03 | Palumbo N.E., Blauwkamp R.A., Lloyd J.M. *Basic Principles of Homing Guidance*. Johns Hopkins APL Technical Digest, 2010, 29(1): 25–41（开放获取 PDF） | ✅ | 06/08，寻的制导原理速读首选 |
| R04 | Garnell P., East D.J. *Guided Weapon Control Systems*, 2nd ed. Pergamon Press, 1980 | ✅ | 02，遥控制导与控制系统经典教材 |
| R05 | Yanushevsky R. *Modern Missile Guidance*, 2nd ed. CRC Press, 2016（1st ed. 2007） | ✅ | 09，现代导引律综述视角 |
| R06 | Guelman M. *A Qualitative Study of Proportional Navigation*. IEEE Trans. AES, 1971, AES-7(4): 637–643 | ✅ | 06/08，PN 定性/闭式分析经典 |
| R09 | 雷虎民 等《导弹制导与控制原理》（第2版），国防工业出版社，2018 | ✅ | 01/02/07，中文系统教材 |
| R10 | 《导弹制导控制原理》，北京航空航天大学出版社，2021（ISBN 9787512432086） | ✅ | 01/07，中文教材 |
| R11 | 张友安《角度控制与时间控制导引律》，电子工业出版社，2017 | ✅ | 09，落角/时间约束导引 |
| R12 | 王辉、王伟、林德福、唐道光《战术导弹制导律设计理论与方法：多约束视角》，北京理工大学出版社 | ✅（出版年待核） | 08/09，多约束导引律 |
| R13 | 空军工程大学《导弹制导与控制原理》在线课程（学堂在线） | ✅ | 入门串联课程 |

### B. 导航、滤波与估计

| 编号 | 文献 | 状态 | 主要用于 |
|------|------|------|----------|
| R07 | Titterton D.H., Weston J.L. *Strapdown Inertial Navigation Technology*, 2nd ed. IEE(Peter Peregrinus), 2004 | ✅ | 04/05，捷联惯导“圣经” |
| R08 | Grewal M.S., Weill L.R., Andrews A.P. *Global Positioning Systems, Inertial Navigation, and Integration*, 2nd ed. Wiley, 2007 | ✅ | 04/05，GNSS/INS 组合 |
| R14 | Kalman R.E. *A New Approach to Linear Filtering and Prediction Problems*. ASME J. Basic Engineering, 1960 | 📖 | 05，卡尔曼滤波原始论文 |
| R15 | Bar-Shalom Y., Li X.R., Kirubarajan T. *Estimation with Applications to Tracking and Navigation*. Wiley, 2001 | 📖 | 05/07，跟踪滤波与数据关联 |
| R16 | Kaplan E.D., Hegarty C.J.（eds.）*Understanding GPS/GNSS: Principles and Applications*, 3rd ed. Artech House, 2017 | 📖 | 04，GNSS 原理与干扰 |
| R17 | Maybeck P.S. *Stochastic Models, Estimation, and Control*（卷1）, Academic Press | 📖 | 05，工程滤波实现与调参 |

### C. 路径规划与最优控制

| 编号 | 文献 | 状态 | 主要用于 |
|------|------|------|----------|
| R18 | Dijkstra E.W. *A Note on Two Problems in Connexion with Graphs*. Numerische Mathematik, 1959, 1: 269–271 | 📖 | 03 |
| R19 | Hart P.E., Nilsson N.J., Raphael B. *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*（A*）. IEEE Trans. SSC, 1968 | 📖 | 03 |
| R20 | LaValle S.M. *Rapidly-Exploring Random Trees: A New Tool for Path Planning*（技术报告）, 1998 | 📖 | 03 |
| R21 | Khatib O. *Real-Time Obstacle Avoidance for Manipulators and Mobile Robots*. Int. J. Robotics Research, 1986, 5(1): 90–98 | 📖 | 03 |
| R22 | Betts J.T. *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*, 2nd ed. SIAM, 2010 | 📖 | 03/09，伪谱法/最优轨迹 |
| R23 | Bryson A.E., Ho Y.-C. *Applied Optimal Control: Optimization, Estimation and Control*. Blaisdell, 1968（修订版 1975） | 📖 | 08/09，OGL 的方法论源头 |

### D. 博弈、鲁棒与智能方法

| 编号 | 文献 | 状态 | 主要用于 |
|------|------|------|----------|
| R24 | Isaacs R. *Differential Games*. Wiley, 1965 | 📖 | 09，微分对策源头 |
| R25 | Slotine J.-J.E., Li W. *Applied Nonlinear Control*. Prentice Hall, 1991 | 📖 | 09，滑模控制标准教材 |
| R26 | Lewis F.L., Vrabie D., Syrmos V.L. *Optimal Control*, 3rd ed. Wiley, 2012 | 📖 | 08/09 |
| R27 | 强化学习/深度学习制导方向：近五年 AIAA JGCD、IEEE TNNLS、Neurocomputing 上的综述与应用论文 | 🔍 | 09（方向更新快，请以最新检索为准） |

### 使用文献的三条建议

1. **优先读 R01/R03**：一本 Zarchan + 一篇 JHU APL Technical Digest，能覆盖 06/08 章 80% 的主线内容，
   且都是“推导 + 数值结论 + 工程注解”的写法。
2. **中文优先 R09/R11/R12**：公式符号与国内教材一致，读起来最省力。
3. **写论文/报告时**：以 📖 条目为线索反查原文，核对卷期页码后再引用。

---

## 四、插图与再生成

所有插图为脚本生成的 PNG（浅色底，适合插入文档/打印）：

```powershell
cd guidance_sim\references
python make_figs.py      # 重新生成 figures\ 下全部 9 张图
```

脚本只依赖项目已有的 PyQt5（QPainter 离屏绘制），不使用 matplotlib。
图与章的对应关系见第一节表格；`fig06_pn_pp.png` 是**直接调用仿真代码**画出的真实弹道与过载曲线。

---

## 五、与仿真程序的分工

| 你想做的事 | 用哪里 |
|-----------|--------|
| 读懂算法原理与出处 | **本目录 `references/`** |
| 在程序里对照公式看实现 | `../docs/`（应用内“帮助”可打开） |
| 动手改参数、复现实验 | `../` 主程序（空格运行、`S` 快照，参数面板） |
| 改完帮助文档自动生效 | `../docs/*.md` 编辑后在帮助窗口点「重新加载」 |
| 重新生成本资料插图 | `python make_figs.py` |

> 本资料中出现的性能数字（如“PN 脱靶 0.2 m、PP 脱靶 67.8 m、pip 中段 5.84 s 截获”）均来自
> 默认参数下的仿真复现，可用 `python selftest.py` 或界面操作验证；工程实际数字请以具体型号公开资料为准。
