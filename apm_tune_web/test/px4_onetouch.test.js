// PX4 一键首飞方案派生单测: onetouch_px4.js (window.PX4OneTouch) + onetouch.js 固件派发
//   node test/px4_onetouch.test.js
import fs from 'fs';
import path from 'path';
import vm from 'vm';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');

let failures = 0;
function check(cond, msg) {
  if (cond) { console.log('  ok  ' + msg); } else { console.log('  FAIL ' + msg); failures++; }
}

// ---------- 加载前端脚本 (与 golden_compare 相同的 vm 沙箱) ----------
let fw = 'apm';
const sandbox = { window: { getFirmwareType: () => fw }, console };
vm.createContext(sandbox);
for (const f of ['onetouch_data.js', 'onetouch_px4.js', 'onetouch.js']) {
  vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', f), 'utf8'), sandbox);
}
const core = sandbox.window.OneTouchCore;
const px4d = sandbox.window.PX4_OT_DATA;
check(!!core && !!px4d, '加载 OneTouchCore / PX4_OT_DATA');
check(typeof sandbox.window.PX4OneTouch.otStages === 'function'
  && typeof sandbox.window.PX4OneTouch.otValidate === 'function', 'PX4OneTouch 导出 otStages/otValidate');

const ctx = (over = {}) => ({ size: 5, cells: 6, hover: 0.25, purpose: 'first', profile: 'std', ...over });
const flat = (stages) => stages.flatMap((s) => s.items);
const byName = (stages) => Object.fromEntries(flat(stages).map((e) => [e.name, e]));

// ---------- AP 路径不受影响 ----------
let ap = core.stagesFor(ctx());
check(core.isPx4() === false, "firmware=apm 时 isPx4() 为 false");
check(ap.length === 8 && ap.some((s) => s.id === 'spin'), 'AP 路径 8 阶段含 spin');

// ---------- PX4 路径 ----------
fw = 'px4';
check(core.isPx4() === true, "firmware=px4 时 isPx4() 为 true");

const stages = core.stagesFor(ctx());
check(stages.length === px4d.stageOrder.length, `PX4 阶段数 ${stages.length} == stageOrder ${px4d.stageOrder.length}`);
check(stages.length === 7 && !stages.some((s) => s.id === 'spin'), 'PX4 7 阶段且无 spin (PX4 无 SPIN 参数)');
const all = flat(stages);
check(all.length > 25, `首飞方案项数 ${all.length} > 25`);
check(all.every((e) => e.name && typeof e.value === 'number' && Array.isArray(e.rng) && e.desc),
  '每项含 name/value/rng/desc');
check(all.every((e) => e.rng[0] < e.rng[1]), 'rng 区间合法');

const p = byName(stages);
check(Math.abs(p.MPC_XY_CRUISE.value - 5) < 1e-6, '首飞 MPC_XY_CRUISE=5 m/s (固件默认)');
check(p.MPC_XY_CRUISE.check === false, '首飞不勾选水平导航 (仅体检展示)');
check(Math.abs(p.MPC_TKO_SPEED.value - 1.5) < 1e-6, 'MPC_TKO_SPEED=1.5');
check(Math.abs(p.MPC_LAND_SPEED.value - 0.7) < 1e-6, 'MPC_LAND_SPEED=0.7');
check(p.BAT1_CAPACITY.value === -1, 'BAT1_CAPACITY=-1 (自动识别)');
check(p.COM_LOW_BAT_ACT.value === 3, 'COM_LOW_BAT_ACT=3 (低电返航+紧急降落)');
check(p.NAV_RCL_ACT.value === 2 && p.NAV_DLL_ACT.value === 0, 'RC 失联返航 / 数据链禁用');
check(p.GF_MAX_HOR_DIST.value === 50 && p.GF_MAX_VER_DIST.value === 50, '围栏 50m');
check(p.RTL_RETURN_ALT.value <= p.GF_MAX_VER_DIST.value, `RTL 高度 ${p.RTL_RETURN_ALT.value} ≤ 垂直围栏 ${p.GF_MAX_VER_DIST.value}`);
check(p.IMU_GYRO_CUTOFF.value === 40, '5寸 陀螺滤波 40Hz (v1.14 默认)');
check(p.MC_ROLL_P.check === false && p.MC_ROLLRATE_P.check === false, '速率环/角度环只体检不勾选');
check(p.EKF2_GYR_NOISE.check === false && p.MPC_THR_HOVER.check === false, 'EKF2/悬停油门只体检不勾选');
check(p.MPC_THR_HOVER.value === 0.5, 'MPC_THR_HOVER=0.5 (体检参考值, 起飞自动学习)');

// ---------- 交叉校验 ----------
const rows = all.map((e) => ({ entry: e, value: e.value }));
const okIssues = core.validateFor(rows);
check(Array.isArray(okIssues) && okIssues.length === 0,
  `默认方案交叉校验通过 (${JSON.stringify(okIssues)})`);

const mutate = (name, value) => rows.map((r) => (r.entry.name === name ? { ...r, value } : r));
let iss = core.validateFor(mutate('BAT_CRIT_THR', 0.2)); // > BAT_LOW_THR 0.15
check(iss.some((x) => /BAT_CRIT_THR/.test(x)), '电池阈值顺序校验 (CRIT < LOW)');
iss = core.validateFor(mutate('RTL_RETURN_ALT', 60)); // > GF_MAX_VER_DIST 50
check(iss.some((x) => /RTL_RETURN_ALT/.test(x)), '返航高度 ≤ 垂直围栏校验');
iss = core.validateFor(mutate('MPC_Z_VEL_MAX_DN', 5)); // > UP 3
check(iss.some((x) => /MPC_Z_VEL_MAX_DN/.test(x)), '下降速度 ≤ 上升速度校验');
iss = core.validateFor(mutate('MPC_TKO_SPEED', 99)); // 超 rng [1,5]
check(iss.some((x) => /MPC_TKO_SPEED.*超出固件范围/.test(x)), '固件范围校验');

// ---------- 尺寸/用途/档位派生 ----------
const st20 = byName(core.stagesFor(ctx({ size: 20 })));
check(st20.IMU_GYRO_CUTOFF.value === 20, '20寸 → 陀螺滤波 20Hz');
const st13 = byName(core.stagesFor(ctx({ size: 13 })));
check(st13.IMU_GYRO_CUTOFF.value === 30, '13寸 → 陀螺滤波 30Hz');

const guided = byName(core.stagesFor(ctx({ purpose: 'guided' })));
check(Math.abs(guided.MPC_XY_CRUISE.value - 8) < 1e-6, 'Offboard 巡航 8 m/s');
check(guided.MPC_XY_CRUISE.check === true && guided.MPC_ACC_HOR.check === true,
  'Offboard 用途勾选水平导航参数');
check(Math.abs(guided.MPC_ACC_HOR.value - 6) < 1e-6, 'Offboard MPC_ACC_HOR=6');

const autoSoft = byName(core.stagesFor(ctx({ purpose: 'auto', profile: 'soft' })));
check(Math.abs(autoSoft.MPC_XY_CRUISE.value - 4.2) < 1e-6, 'AUTO+更保守 → 巡航 6*0.7=4.2 m/s');
check(autoSoft.MPC_Z_V_AUTO_UP.check === true && autoSoft.MIS_DIST_1WP.check === true,
  'AUTO 用途勾选任务垂直速度/首航点距离');

const firstHot = byName(core.stagesFor(ctx({ profile: 'hot' })));
check(Math.abs(firstHot.MPC_XY_CRUISE.value - 6.5) < 1e-6, '首飞+更激进 → 5*1.3=6.5 m/s');
check(firstHot.MPC_XY_CRUISE.check === false, '首飞即使更激进也不自动勾选水平导航');

// PX4 数据缺 sizes 键 → sizeProfile 回退 AP 数据
check(core.sizeProfile(5) && core.sizeProfile(5).gyro > 0, 'PX4 下 sizeProfile 回退 AP 数据');

console.log(`\n${failures === 0 ? 'PX4 方案派生单测通过' : failures + ' 项失败'}`);
process.exit(failures === 0 ? 0 : 1);
