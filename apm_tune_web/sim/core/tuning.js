// ============================================================
// 集中可调参数表（tuning）
//
//   仿真里的公式分两类：
//     ① 物理定律（动量理论、翼素受力、悬臂梁弯矩、基尔霍夫…）——不可调，调了就错
//     ② 半经验系数 / 工程判据阈值 —— 本文件集中管理，可调
//
//   用法：
//     const { T, setTuning, tuningDiff } = tuning;
//     simulate(input) 内部会 setTuning(input.tuning)；
//     深层物理函数通过 T().bemt.kInduced 读取（模块级单例，属性访问零开销）。
//
//   注意：simulate() 是纯同步的（无 await），单例在一次仿真内不会被交错改写，
//   因此并发请求也是安全的。默认值必须与历史硬编码值逐字相同，
//   否则会破坏 test/selftest.js 的校准基准（5寸 4S 1750KV 5043 = 852g/21.5A/4.34 g/W）。
// ============================================================

/** 参数定义：key / 分组 / 中文名 / 单位 / 范围 / 步进 / 说明 / 默认值 */
export const PARAMS = [
  // ---------------- 桨叶气动（BEMT 半经验修正） ----------------
  { key: 'bemt.kInduced', group: 'blade', label: '诱导阻力系数', unit: '', min: 0, max: 0.05, step: 0.0005, def: 0.0075,
    desc: '桨叶阻力中与 Cl² 成正比的项（cd = k·Cl²）。越大高负载时效率越差。' },
  { key: 'bemt.kViscous', group: 'blade', label: '粘性阻力系数', unit: '', min: 0, max: 0.05, step: 0.0005, def: 0.004,
    desc: '桨叶阻力中与 |Cl| 成正比的项（剖面摩阻+压差阻力）。' },
  { key: 'bemt.kSeparation', group: 'blade', label: '失速分离阻力', unit: '', min: 0, max: 2, step: 0.01, def: 0.35,
    desc: '攻角超过分离起点后阻力二次项系数。越大失速惩罚越重。' },
  { key: 'bemt.sepOnset', group: 'blade', label: '分离起始攻角比', unit: '', min: 0.3, max: 0.95, step: 0.01, def: 0.60,
    desc: '攻角达失速攻角的此比例时开始出现分离阻力。' },
  { key: 'bemt.kMach', group: 'blade', label: '可压缩性阻力系数', unit: '', min: 0, max: 5, step: 0.05, def: 1.3,
    desc: '高速桨尖的压缩性阻力 k·(M−M0)²。影响高 KV/大桨尖马赫的惩罚。' },
  { key: 'bemt.machOnset', group: 'blade', label: '压缩性起始马赫', unit: '', min: 0.1, max: 0.8, step: 0.01, def: 0.28,
    desc: '桨叶剖面开始出现压缩性阻力的来流马赫数。' },
  { key: 'bemt.camber_deg', group: 'blade', label: '叶型弯度偏移', unit: '°', min: -5, max: 12, step: 0.1, def: 4,
    desc: '真实桨叶有弯度，零升力角为负，等效攻角整体偏移此值。' },
  { key: 'bemt.viRelax', group: 'blade', label: '诱导速度松弛', unit: '', min: 0.05, max: 1, step: 0.01, def: 0.55,
    desc: 'BEMT 诱导速度迭代的松弛因子（数值稳定性，非物理量）。调大会收敛快但可能振荡。' },

  // ---------------- 结构材料 ----------------
  { key: 'carbon.E_GPa', group: 'material', label: '碳板弹性模量', unit: 'GPa', min: 5, max: 200, step: 1, def: 45,
    desc: '准各向同性碳纤维层合板设计值。T700 约 50~60，玻纤约 20。影响挠度与固有频率。' },
  { key: 'carbon.sigmaAll_MPa', group: 'material', label: '碳板许用应力', unit: 'MPa', min: 50, max: 2000, step: 10, def: 420,
    desc: '层合板许用拉应力。影响机臂安全系数与反解所需板厚。' },
  { key: 'carbon.density', group: 'material', label: '碳板密度', unit: 'kg/m³', min: 800, max: 2600, step: 10, def: 1600,
    desc: '碳板密度。当前仅用于板厚反解时估算增重。' },
  { key: 'screw.tauAll_MPa', group: 'material', label: '螺钉许用剪应力', unit: 'MPa', min: 50, max: 800, step: 10, def: 240,
    desc: '电机座 M3 螺钉的许用剪应力（2 颗有效剪切面）。' },

  // ---------------- 整机气动 ----------------
  { key: 'aero.cdQuad', group: 'aero', label: '四轴阻力系数', unit: '', min: 0.3, max: 3, step: 0.05, def: 1.15, desc: '四轴整机（含桨盘）钝体阻力系数。' },
  { key: 'aero.cdHex', group: 'aero', label: '六轴阻力系数', unit: '', min: 0.3, max: 3, step: 0.05, def: 1.05, desc: '六轴整机阻力系数。' },
  { key: 'aero.cdOcto', group: 'aero', label: '八轴阻力系数', unit: '', min: 0.3, max: 3, step: 0.05, def: 0.95, desc: '八轴整机阻力系数。' },
  { key: 'aero.cdWhoop', group: 'aero', label: '穿越机阻力系数', unit: '', min: 0.3, max: 3, step: 0.05, def: 1.35, desc: '带桨保框的 whoop 阻力系数。' },
  { key: 'aero.cdRocket', group: 'aero', label: '火箭机身阻力系数', unit: '', min: 0.1, max: 3, step: 0.05, def: 0.55, desc: '垂起火箭细长机身阻力系数。' },
  { key: 'aero.cdFixed', group: 'aero', label: '固定翼机身阻力系数', unit: '', min: 0.2, max: 3, step: 0.05, def: 0.9, desc: '固定翼机身/吊舱阻力系数。' },
  { key: 'aero.bodyFactor', group: 'aero', label: '机身升力体折算', unit: '', min: 0, max: 1.5, step: 0.01, def: 0.55,
    desc: '细长机身侧投影产生升力的折算系数（cross-flow 升力体）。越大火箭/飞翼巡航升阻比越高。' },
  { key: 'aero.groundK', group: 'aero', label: '地面效应强度', unit: '', min: 0, max: 0.6, step: 0.01, def: 0.18,
    desc: '地面效应诱导功率下降系数 1 + k·(1−h/D)²。' },
  { key: 'aero.noiseOffset_dB', group: 'aero', label: '噪音标定偏置', unit: 'dB', min: -60, max: 0, step: 0.5, def: -33,
    desc: '半经验噪音式的标定常数 L1 = 60·log10(vTip) + 10·log10(b·D) + 偏置。用实测数据拟合时改这里。' },
  { key: 'aero.noiseVelExp', group: 'aero', label: '噪音速度指数', unit: '', min: 20, max: 100, step: 1, def: 60,
    desc: '噪音随桨尖速度的幂次（60 表示 L ∝ vTip^6，即声功率∝vTip^8 的常见标度）。' },
  { key: 'aero.noiseSizeExp', group: 'aero', label: '噪音桨径指数', unit: '', min: 0, max: 30, step: 1, def: 10,
    desc: '噪音随桨叶尺寸（b·D）的幂次。' },

  // ---------------- 诊断判据阈值 ----------------
  { key: 'diag.hoverDutyError', group: 'diag', label: '悬停油门 error 线', unit: '', min: 0.3, max: 0.99, step: 0.01, def: 0.68, desc: '悬停油门超过此值判为无动力余量（error）。' },
  { key: 'diag.hoverDutyWarn', group: 'diag', label: '悬停油门 warn 线', unit: '', min: 0.2, max: 0.9, step: 0.01, def: 0.55, desc: '悬停油门超过此值告警偏高。' },
  { key: 'diag.hoverDutyMin', group: 'diag', label: '悬停油门下限', unit: '', min: 0.02, max: 0.4, step: 0.01, def: 0.18, desc: '悬停油门低于此值判为动力过剩（分辨率差）。' },
  { key: 'diag.twrError', group: 'diag', label: '推重比 error 线', unit: '', min: 0.5, max: 2, step: 0.05, def: 1.0, desc: '推重比低于此值无法起飞（error）。' },
  { key: 'diag.twrWarn', group: 'diag', label: '推重比 warn 线', unit: '', min: 1, max: 4, step: 0.05, def: 1.6, desc: '推重比低于此值告警抗风/机动能力差。' },
  { key: 'diag.twrHighMR', group: 'diag', label: '多旋翼推重比上限', unit: '', min: 2, max: 30, step: 0.5, def: 6, desc: '多旋翼推重比超过此值告警油门过于灵敏。' },
  { key: 'diag.twrHighVTOL', group: 'diag', label: 'VTOL 推重比上限', unit: '', min: 2, max: 30, step: 0.5, def: 9, desc: 'VTOL 推重比超过此值告警油门过于灵敏。' },
  { key: 'diag.motorHeatWarn', group: 'diag', label: '电机热负荷 warn 线', unit: '', min: 0.2, max: 1, step: 0.01, def: 0.55, desc: '满油门铜损/允许值超过此比值告警过热。' },
  { key: 'diag.tipMach', group: 'diag', label: '桨尖马赫 warn 线', unit: '', min: 0.3, max: 1.2, step: 0.01, def: 0.72, desc: '满油门桨尖马赫超过此值告警噪音剧增、桨叶负荷大。' },
  { key: 'diag.noise10m', group: 'diag', label: '10m 噪音 warn 线', unit: 'dB', min: 50, max: 110, step: 1, def: 75, desc: '10m 处噪音超过此值告警扰民。' },
  { key: 'diag.armSFError', group: 'diag', label: '机臂安全系数 error 线', unit: '', min: 1, max: 5, step: 0.1, def: 1.5, desc: '机臂安全系数低于此值判为炸机风险（error）。' },
  { key: 'diag.armSFWarn', group: 'diag', label: '机臂安全系数目标', unit: '', min: 1.5, max: 10, step: 0.1, def: 3, desc: '机臂安全系数低于此值提示可优化（info）。' },
  { key: 'diag.smMin', group: 'diag', label: '静稳定裕度下限', unit: '%', min: 0, max: 20, step: 0.5, def: 5, desc: 'SM 低于此值告警抗扰能力弱。' },
  { key: 'diag.smMax', group: 'diag', label: '静稳定裕度上限', unit: '%', min: 10, max: 50, step: 0.5, def: 25, desc: 'SM 高于此值告警舵面可能压不过。' },
  { key: 'diag.vStallRatio', group: 'diag', label: '包线过窄判据', unit: '', min: 1.05, max: 2.5, step: 0.05, def: 1.3,
    desc: '失速速度×此系数 > 最高速时告警飞行包线过窄。' },
  { key: 'diag.enduranceMin_min', group: 'diag', label: '最短续航 warn 线', unit: 'min', min: 0.5, max: 20, step: 0.5, def: 2, desc: '续航低于此值告警过短。' },
  { key: 'diag.vtolClimbMin', group: 'diag', label: 'VTOL 垂直爬升 warn 线', unit: 'ft/min', min: 50, max: 1000, step: 10, def: 200, desc: 'VTOL 垂直爬升率低于此值告警无动力余量。' },
  { key: 'diag.ldMin', group: 'diag', label: '最小升阻比 warn 线', unit: '', min: 3, max: 20, step: 0.5, def: 8, desc: '固定翼最大升阻比低于此值告警巡航效率差。' },

  // ---------------- 反向求解器 ----------------
  { key: 'solver.speedMargin', group: 'solver', label: '目标速度裕度', unit: '', min: 1, max: 1.5, step: 0.01, def: 1.15,
    desc: '求解时把目标速度放大此倍作为达标线（留机动余量）。' },
  { key: 'solver.twrGoalMR', group: 'solver', label: '多旋翼推重比目标', unit: '', min: 1.2, max: 5, step: 0.1, def: 1.8, desc: '求解器把推重比抬到此值算达标。' },
  { key: 'solver.twrGoalVTOL', group: 'solver', label: 'VTOL 推重比目标', unit: '', min: 1.5, max: 6, step: 0.1, def: 2.5, desc: 'VTOL 转换段需要更大裕度。' },
  { key: 'solver.sfGoal', group: 'solver', label: '结构安全系数目标', unit: '', min: 1.5, max: 8, step: 0.1, def: 3, desc: '结构解析反解把安全系数抬到此值。' },
  { key: 'solver.maxCalls', group: 'solver', label: '求解评估预算', unit: '次', min: 20, max: 500, step: 10, def: 110, desc: '反向求解允许的 simulate() 调用次数上限。' },
  { key: 'solver.deadline_ms', group: 'solver', label: '求解墙钟上限', unit: 'ms', min: 1000, max: 30000, step: 500, def: 9000, desc: '反向求解的墙钟时间上限，防慢机器失控。' },
];

export const GROUPS = [
  { id: 'blade', label: '桨叶气动（BEMT 半经验）', hint: '影响推力/电流/效率曲线形状' },
  { id: 'material', label: '结构材料', hint: '影响机臂安全系数/挠度/固有频率' },
  { id: 'aero', label: '整机气动', hint: '影响阻力/升阻比/噪音/地面效应' },
  { id: 'diag', label: '诊断判据阈值', hint: '只影响"什么算合格"，不影响物理结果' },
  { id: 'solver', label: '反向求解器', hint: '影响改造方案的达标线与搜索预算' },
];

const BY_KEY = new Map(PARAMS.map(p => [p.key, p]));
const DEFAULTS = {};
for (const p of PARAMS) {
  const [g, n] = p.key.split('.');
  (DEFAULTS[g] ||= {})[n] = p.def;
}

/** 参数表快照（深拷贝，供 API/UI 展示默认值） */
export function tuningMeta() {
  return {
    params: PARAMS.map(p => ({ ...p })),
    groups: GROUPS.map(g => ({ ...g })),
    defaults: JSON.parse(JSON.stringify(DEFAULTS)),
  };
}

// ---------------- 当前生效值（模块级单例） ----------------
let CUR = JSON.parse(JSON.stringify(DEFAULTS));
let APPLIED = {};   // 本次显式覆盖的 {key: value}

/** 读取当前调参表：T().bemt.kInduced */
export function T() { return CUR; }

function clampNum(v, p) {
  const x = Number(v);
  if (!Number.isFinite(x)) return null;
  return Math.min(p.max, Math.max(p.min, x));
}

/**
 * 设置调参表（覆盖默认值）。
 * @param {object} overrides 扁平 key（'bemt.kInduced'）或嵌套 {bemt:{kInduced:…}} 均可
 * @returns {{applied:object, errors:string[]}} 实际生效的覆盖项与被拒绝的项
 */
export function setTuning(overrides) {
  CUR = JSON.parse(JSON.stringify(DEFAULTS));
  APPLIED = {};
  const errors = [];
  if (!overrides || typeof overrides !== 'object') return { applied: {}, errors };

  // 归一化成扁平 {key: value}
  const flat = {};
  for (const [k, v] of Object.entries(overrides)) {
    if (v === null || v === undefined) continue;
    if (k.includes('.')) { flat[k] = v; continue; }
    if (typeof v === 'object') {
      for (const [n, val] of Object.entries(v)) flat[`${k}.${n}`] = val;
    }
  }
  for (const [key, v] of Object.entries(flat)) {
    const p = BY_KEY.get(key);
    if (!p) { errors.push(`未知参数 ${key}`); continue; }
    if (typeof v === 'string' && v.trim() === '') { errors.push(`${key} 为空`); continue; }
    const x = clampNum(v, p);
    if (x === null) { errors.push(`${key} 非数值: ${JSON.stringify(v)}`); continue; }
    const [g, n] = key.split('.');
    CUR[g][n] = x;
    if (x !== p.def) APPLIED[key] = x;   // 与默认值相同的覆盖不记为"已改"
  }
  return { applied: { ...APPLIED }, errors };
}

/** 当前相对默认值的差异（用于报告与 UI 高亮） */
export function tuningDiff() {
  const out = [];
  for (const p of PARAMS) {
    const [g, n] = p.key.split('.');
    const cur = CUR[g][n];
    if (cur !== p.def) out.push({ key: p.key, label: p.label, unit: p.unit, def: p.def, value: cur });
  }
  return out;
}

/** 恢复默认值 */
export function resetTuning() { return setTuning(null); }
