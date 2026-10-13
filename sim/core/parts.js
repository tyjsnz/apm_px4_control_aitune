// ============================================================
// 部件数据库：电池 / 电机 / 电调 / 机架 / 桨叶 / 翼型
// ============================================================

export const CHEMISTRIES = {
  lipo:  { id:'lipo',  label:'LiPo 标准锂聚合物', vFull:4.20, vNom:3.70, vCut:3.30, rFac:1.00, cDefault:50 },
  lihv:  { id:'lihv',  label:'LiHV 高压锂电 (4.35V)', vFull:4.35, vNom:3.80, vCut:3.40, rFac:1.15, cDefault:70 },
  liion: { id:'liion', label:'Li-ion 18650/21700', vFull:4.20, vNom:3.60, vCut:3.00, rFac:2.40, cDefault:10 },
};

// 内阻基准：2.9mΩ @1000mAh 每片（含极耳/引线的等效值）
export function battery({ cells = 6, capacity_mAh = 1500, chemistry = 'lipo', cRating = null, soc = 1 }) {
  const ch = CHEMISTRIES[chemistry] || CHEMISTRIES.lipo;
  const cap = Math.max(100, capacity_mAh);
  const Ah = cap / 1000;
  const rCell_mOhm = 2.9 * (1000 / cap) * ch.rFac;
  const rPack_mOhm = rCell_mOhm * cells;
  const C = cRating || ch.cDefault;
  const vFull = cells * ch.vFull;
  const vNom = cells * ch.vNom;
  const vCut = cells * ch.vCut;
  const energy_Wh = Ah * vNom;
  const maxCurrent_A = Ah * C;
  const vCell = (s) => ch.vCut + (ch.vFull - ch.vCut) * Math.pow(clamp01(s), 0.8);
  return {
    cells, capacity_mAh, chemistry: ch.id, chemistryLabel: ch.label, cRating: C,
    rCell_mOhm, rPack_mOhm, vFull, vNom, vCut, vEmpty: vCut,
    energy_Wh, maxCurrent_A, mass_g: estimatePackMass(cap, cells, chemistry),
    ocv: cells * vCell(soc),
    vCellAt: (s) => cells * vCell(s),
    label: `${cells}S ${cap}mAh ${ch.label}`,
  };
}

function estimatePackMass(cap, cells, chemistry) {
  // LiPo ≈ 9.5 g/Wh，Li-ion ≈ 5.8 g/Wh（含保护板/引线）
  const ch = CHEMISTRIES[chemistry] || CHEMISTRIES.lipo;
  const wh = (cap / 1000) * cells * ch.vNom;
  const gPerWh = chemistry === 'liion' ? 5.8 : 9.5;
  return wh * gPerWh;
}

// ---------------------------- 电机 ----------------------------
// R0 / I0 / PcontLoss 为该型号在 KV0 下的基准值
export const MOTORS = [
  { id:'0804',  label:'0804 (1S 穿越机)',    mass_g:10,  KV0:25000, R0:0.190, I0:0.45, PlossCont:16,  PlossBurst:55,   stator:'08' },
  { id:'1103',  label:'1103 (1-2S 微型)',    mass_g:7,   KV0:10000, R0:0.120, I0:0.50, PlossCont:16,  PlossBurst:50,   stator:'11' },
  { id:'1204',  label:'1204 (2-3S 微型)',    mass_g:11,  KV0:6000,  R0:0.090, I0:0.60, PlossCont:22,  PlossBurst:70,   stator:'12' },
  { id:'1404',  label:'1404 (2-3S 牙签)',    mass_g:16,  KV0:4000, R0:0.075, I0:0.70, PlossCont:32,  PlossBurst:100,  stator:'14' },
  { id:'1507',  label:'1507 (3-4S 4寸)',     mass_g:22,  KV0:2800, R0:0.062, I0:0.80, PlossCont:45,  PlossBurst:150,  stator:'15' },
  { id:'1804',  label:'1804 (4S 4-5寸)',     mass_g:30,  KV0:2500, R0:0.055, I0:0.90, PlossCont:60,  PlossBurst:200,  stator:'18' },
  { id:'2205',  label:'2205 (4S 5寸)',       mass_g:28,  KV0:2300, R0:0.058, I0:1.00, PlossCont:70,  PlossBurst:230,  stator:'22' },
  { id:'2207',  label:'2207 (4-6S 5寸)',     mass_g:34,  KV0:1750, R0:0.045, I0:1.20, PlossCont:95,  PlossBurst:320,  stator:'22' },
  { id:'2306',  label:'2306 (4-6S 5寸)',     mass_g:36,  KV0:1900, R0:0.048, I0:1.20, PlossCont:100, PlossBurst:340,  stator:'23' },
  { id:'2507',  label:'2507 (5-6S 5寸)',     mass_g:46,  KV0:1700, R0:0.038, I0:1.30, PlossCont:125, PlossBurst:420,  stator:'25' },
  { id:'28065', label:'2806.5 (6S 7-8寸)',  mass_g:58,  KV0:1300, R0:0.030, I0:1.40, PlossCont:160, PlossBurst:540,  stator:'28' },
  { id:'3110',  label:'3110 (6S 7寸)',       mass_g:95,  KV0:950,  R0:0.022, I0:1.60, PlossCont:240, PlossBurst:800,  stator:'31' },
  { id:'4114',  label:'4114 (6-12S 9-10寸)', mass_g:170, KV0:650,  R0:0.014, I0:2.00, PlossCont:400, PlossBurst:1350, stator:'41' },
  { id:'5215',  label:'5215 (12S 大载重)',   mass_g:300, KV0:380,  R0:0.008, I0:2.60, PlossCont:650, PlossBurst:2200, stator:'52' },
  { id:'8120',  label:'8120 (12S 重载八轴)', mass_g:520, KV0:150,  R0:0.005, I0:3.50, PlossCont:1100, PlossBurst:3800, stator:'81' },
];

export function motor({ type = '2207', kv = 1750 } = {}) {
  const m = MOTORS.find(x => x.id === type) || MOTORS[4];
  const kvv = clamp(kv, 100, 30000);
  const R_m = m.R0 * Math.pow(m.KV0 / kvv, 1.2);       // 匝数/线径随 KV 变化
  const I0 = m.I0 * Math.pow(kvv / m.KV0, 0.7);
  const Kt = 9.5493 / kvv;                              // N·m/A
  const KVsi = kvv * 2 * Math.PI / 60;                  // rad/s per V
  const iBurst = Math.sqrt(m.PlossBurst / R_m);
  const iCont = Math.sqrt(m.PlossCont / R_m);
  return {
    ...m, kv: kvv, R_m, I0, Kt, KVsi,
    iCont, iBurst,
    torqueCont: Kt * iCont,
    label: `${m.id} ${kvv}KV`,
  };
}

// ---------------------------- 电调 ----------------------------
export const ESCS = [
  { id:'pwm',    label:'模拟 PWM 48kHz (老式)',  r:0.0060, swLoss:0.050, resp_ms:8.0, note:'损耗大、线性差，仅适合保留老设备' },
  { id:'shott_s',label:'BLHeli_S + DShot600',    r:0.0042, swLoss:0.036, resp_ms:5.0, note:'入门主流，嗡鸣略大、启动稍钝' },
  { id:'shott32',label:'BLHeli_32 / AM32',       r:0.0032, swLoss:0.026, resp_ms:3.2, note:'主流推荐，发热与响应均衡' },
  { id:'race',   label:'竞速级 8MHz PWM / 模拟',  r:0.0024, swLoss:0.020, resp_ms:2.4, note:'损耗最低，适合高频调参' },
  { id:'hv',     label:'高压大电流电调 (12S/60A+)', r:0.0018, swLoss:0.017, resp_ms:2.8, note:'低内阻，服务大 KV/大电流系统' },
];

export function esc({ type = 'shott32', wireAwg = 16, wireLen_cm = 10 } = {}) {
  const e = ESCS.find(x => x.id === type) || ESCS[2];
  const awg = { 12:5.210, 14:8.286, 16:13.17, 18:20.95, 20:33.31, 22:52.96 }; // Ω/km
  const rWire_mOhm = (awg[wireAwg] || 13.17) * (wireLen_cm / 100000) * 1000 * 2; // 往返两根（mΩ）
  return { ...e, wireAwg, wireLen_cm, rWire_mOhm, rTotal_mOhm: e.r * 1000 + rWire_mOhm };
}

// ---------------------------- 机架 ----------------------------
export const FRAMES = [
  { id:'whoop65',  label:'65mm 穿越机 (1S)',      arm_m:0.038, frame_g:20,  frontal_m2:0.0042, batteryDefault:{cells:1, capacity_mAh:300}, motors:4, kind:'quad' },
  { id:'cinewhoop',label:'75mm 穿越机 (2-4S)',    arm_m:0.050, frame_g:55,  frontal_m2:0.0068, batteryDefault:{cells:2, capacity_mAh:450}, motors:4, kind:'quad' },
  { id:'toothpick',label:'4寸 牙签机 (3-4S)',     arm_m:0.100, frame_g:70,  frontal_m2:0.0100, batteryDefault:{cells:4, capacity_mAh:850}, motors:4, kind:'quad' },
  { id:'fpv5',     label:'5寸 FPV 竞速机 (4-6S)', arm_m:0.125, frame_g:95,  frontal_m2:0.0140, batteryDefault:{cells:4, capacity_mAh:1300}, motors:4, kind:'quad' },
  { id:'longrange',label:'7寸 长航时机 (4-6S)',   arm_m:0.160, frame_g:140, frontal_m2:0.0200, batteryDefault:{cells:4, capacity_mAh:3000}, motors:4, kind:'quad' },
  { id:'hex6',     label:'六旋翼 航拍机 (6S)',     arm_m:0.150, frame_g:300, frontal_m2:0.0480, batteryDefault:{cells:6, capacity_mAh:5000}, motors:6, kind:'hex' },
  { id:'cinelift', label:'9寸 电影机 (6S)',        arm_m:0.200, frame_g:260, frontal_m2:0.0300, batteryDefault:{cells:6, capacity_mAh:4000}, motors:4, kind:'quad' },
  { id:'x8heavy',  label:'大型八轴 载重 (12S)',    arm_m:0.350, frame_g:950, frontal_m2:0.0900, batteryDefault:{cells:12, capacity_mAh:10000}, motors:8, kind:'x8' },
];

export const ROTOR_COUNTS = [
  { id:'quad',  label:'四旋翼', n:4, layout:'X' },
  { id:'hex',   label:'六旋翼', n:6, layout:'X' },
  { id:'octo',  label:'八旋翼', n:8, layout:'X' },
];

// ---------------------------- 垂起火箭（Tailsitter Rocket VTOL）----------------------------
// 双模：垂直悬停（电机向上，机身竖直）→ 巡航（机身水平，尾翼+机身升力体产生升力）。
// bodyLength/bodyD 为细长体主尺寸；finSpan 为单侧尾翼半展；cgBelowThrust 为重心在推力面下方距离
//（电池前置/下置 → 稳定摆；为负表示倒立摆，需飞控持续修正）。
export const ROCKETS = [
  { id:'rocket6',  label:'6寸 垂起火箭 (4S)',        bodyLength_m:0.46, bodyD_mm:62,  finCount:4, finSpan_m:0.21, finArea_m2:0.024, finClmax:1.00, arm_m:0.140, frame_g:210, frontal_m2:0.0052, motors:4, cd0:0.032, cgBelowThrust_mm:18, batteryDefault:{cells:4, capacity_mAh:1800} },
  { id:'rocket8',  label:'8寸 FPV 垂起火箭 (4-6S)',  bodyLength_m:0.56, bodyD_mm:74,  finCount:4, finSpan_m:0.27, finArea_m2:0.036, finClmax:1.10, arm_m:0.190, frame_g:285, frontal_m2:0.0064, motors:4, cd0:0.030, cgBelowThrust_mm:22, batteryDefault:{cells:6, capacity_mAh:3000} },
  { id:'rocket10', label:'10寸 垂起火箭 (6S)',       bodyLength_m:0.68, bodyD_mm:88,  finCount:4, finSpan_m:0.33, finArea_m2:0.055, finClmax:1.12, arm_m:0.245, frame_g:380, frontal_m2:0.0086, motors:4, cd0:0.028, cgBelowThrust_mm:26, batteryDefault:{cells:6, capacity_mAh:5000} },
];

// ---------------------------- 固定翼机身 ----------------------------
// area 为参考机翼面积（含飞翼的中央段）；AR = span²/area，飞翼 2.7~3.7、常规 4.5~9
export const WINGS = [
  { id:'wing700',  label:'700mm FPV 飞翼',        span:0.70, area:0.18, mass_g:450,  cd0:0.026, clmax:1.15, e:0.80 },
  { id:'wing1000', label:'1000mm FPV 飞翼',       span:1.00, area:0.27, mass_g:750,  cd0:0.025, clmax:1.20, e:0.80 },
  { id:'trainer',  label:'1400mm 上单翼教练机',    span:1.40, area:0.42, mass_g:1100, cd0:0.031, clmax:1.55, e:0.78 },
  { id:'glider2m', label:'2m 滑翔机 (长航时)',     span:2.00, area:0.45, mass_g:1250, cd0:0.017, clmax:1.45, e:0.85 },
  { id:'acro900',  label:'900mm 3D 特技机',       span:0.90, area:0.30, mass_g:950,  cd0:0.038, clmax:1.85, e:0.70 },
  { id:'cine1200', label:'1200mm FPV 巡航机',      span:1.20, area:0.32, mass_g:1050, cd0:0.027, clmax:1.35, e:0.80 },
];

// ---------------------------- 桨叶 ----------------------------
export const PROP_MATERIALS = {
  plastic: { id:'plastic', label:'POM/PC 塑料', density:1200, cd0:0.028, flex:0.055, eff_stall:1.00 },
  carbon:  { id:'carbon',  label:'碳纤维',       density:1550, cd0:0.022, flex:0.010, eff_stall:1.02 },
  wood:    { id:'wood',    label:'木质',         density:700,  cd0:0.025, flex:0.020, eff_stall:0.96 },
  folded:  { id:'folded',  label:'折叠桨 (长航时)', density:1250, cd0:0.032, flex:0.035, eff_stall:0.92 },
};

export const PROP_PRESETS = [
  { label:'2寸 2030', d:51,  p:76,  b:3 },
  { label:'3寸 3020', d:76,  p:51,  b:3 },
  { label:'3.5寸 3520', d:89, p:51, b:3 },
  { label:'4寸 4025', d:102, p:64,  b:3 },
  { label:'5寸 5030', d:127, p:76,  b:3 },
  { label:'5寸 5043 竞速', d:127, p:109, b:3 },
  { label:'5寸 5050', d:127, p:127, b:3 },
  { label:'6寸 6030', d:152, p:76,  b:3 },
  { label:'7寸 7040', d:178, p:102, b:3 },
  { label:'8寸 8045', d:203, p:114, b:3 },
  { label:'9寸 9047', d:229, p:119, b:3 },
  { label:'10寸 1045', d:254, p:114, b:2 },
  { label:'12寸 1260', d:305, p:152, b:2 },
  { label:'15寸 1555', d:381, p:140, b:2 },
  { label:'17寸 1760', d:432, p:152, b:2 },
  { label:'21寸 2170', d:533, p:178, b:2 },
  { label:'固定翼 5×5', d:127, p:127, b:2 },
  { label:'固定翼 8×6', d:203, p:152, b:2 },
  { label:'固定翼 10×6', d:254, p:152, b:2 },
  { label:'固定翼 12×8', d:305, p:203, b:2 },
  { label:'固定翼 13×8', d:330, p:203, b:2 },
];

export function prop({ d_mm = 127, p_mm = 109, blades = 3, material = 'plastic', chordRatio = null, dir = 'cw' } = {}) {
  const mat = PROP_MATERIALS[material] || PROP_MATERIALS.plastic;
  const D = d_mm / 1000;
  const P = p_mm / 1000;
  const R = D / 2;
  const c = (chordRatio ?? defaultChord(d_mm)) * D;
  const A = Math.PI * D * D / 4;
  const B = Math.max(2, Math.round(blades));
  // 桨叶惯量（近似为沿展向的矩形杆）
  const J = B * mat.density * c * Math.pow(R, 4) / 4 * 0.75;
  const tipSpeedAt = (rpm) => rpm * 2 * Math.PI / 60 * R;
  const theta75 = Math.atan2(P, 2 * Math.PI * 0.75 * R);
  const volume = B * c * R * (0.11 * c);   // 叶片实体体积近似
  return {
    d_mm, p_mm, blades: B, material: mat, chordRatio: c / D,
    D, P, R, c, A, B, J, density: mat.density, cd0: mat.cd0, flex: mat.flex,
    theta75, pitchRatio: P / D, tipSpeedAt, mass_g: mat.density * volume * 1000,
    label: `${d_mm}×${p_mm}×${B} ${mat.label}`,
  };
}

function defaultChord(d_mm) {
  // 小桨弦径比更大（三叶厚实），大桨更细长
  if (d_mm <= 76) return 0.150;
  if (d_mm <= 127) return 0.135;
  if (d_mm <= 203) return 0.115;
  if (d_mm <= 305) return 0.095;
  return 0.080;
}

// ---------------------------- 工具 ----------------------------
export function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
export function clamp01(v) { return clamp(v, 0, 1); }
export function lerp(a, b, t) { return a + (b - a) * t; }
export const G = 9.80665;
