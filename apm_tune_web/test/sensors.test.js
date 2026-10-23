// 传感器体检单测: public/sensors.js (vm 沙箱) evaluateSensors 纯函数
//   node test/sensors.test.js
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

// ---------- vm 沙箱加载前端纯逻辑 ----------
const sandbox = {
  window: {}, console, Math, Number, isFinite,
  document: { readyState: 'loading', addEventListener() {}, getElementById() { return null; } },
  navigator: {}, setInterval() {}, setTimeout() {}, clearTimeout() {},
};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(ROOT, 'public', 'sensors.js'), 'utf8'), sandbox);
const SC = sandbox.window.SensorCheck;
check(!!SC && typeof SC.evaluateSensors === 'function', '加载 window.SensorCheck');

const T = SC.THRESHOLDS;

// ---------- SYS_STATUS 位掩码 ----------
// 陀螺(1)+加计(2)+磁(4)+气压(8)+GPS(32) = 47, 全健康
const healthySys = {
  sensorsPresent: 47,
  sensorsEnabled: 47,
  sensorsHealth: 47,
  sensorsPresentExt: 0,
  sensorsEnabledExt: 0,
  sensorsHealthExt: 0,
};
// 振动/EKF/IMU/气压/GPS 正常输入
function healthyInput(extra = {}) {
  return {
    sysStatus: healthySys,
    vibration: Array.from({ length: 30 }, () => ({ vibX: 3, vibY: 2, vibZ: 4, clipping: [0, 0, 0] })),
    ekf: Array.from({ length: 10 }, () => ({
      velocityVariance: 0.1, posHorizVariance: 0.5, posVertVariance: 0.3,
      compassVariance: 0.05, flags: 47,
    })),
    imu: Array.from({ length: 50 }, (_, i) => ({
      ax: 0.1 + (i % 2) * 0.05, ay: 0.2 + (i % 3) * 0.04, az: 9.8 + (i % 3) * 0.1,
      temperature: 35,
    })),
    baro: Array.from({ length: 10 }, () => ({ pressAbs: 1013.25, temperature: 25 })),
    gps: { fixType: 3, satellites: 14, eph: 120 },
    prearm: [],
    ...extra,
  };
}

// ---------- 健康全绿 ----------
let r = SC.evaluateSensors(healthyInput());
check(r.verdict === 'ok', `健康输入 verdict=ok (实际 ${r.verdict})`);
const byId = Object.fromEntries(r.sensors.map((s) => [s.id, s]));
check(byId.gyro.status === 'ok', '陀螺仪 ok');
check(byId.accel.status === 'ok', '加速度计 ok');
check(byId.mag.status === 'ok', '磁力计 ok');
check(byId.baro.status === 'ok', '气压计 ok');
check(byId.gps.status === 'ok', 'GPS ok (fix3/14星)');
check(byId.vibration.status === 'ok', '振动 ok');
check(byId.ekf.status === 'ok', 'EKF ok');
check(byId.imu.status === 'ok', 'IMU ok');
check(byId.baro_read.status === 'ok', '气压读数 ok');
check(byId.prearm.status === 'ok', 'PreArm ok (无失败)');
check(r.prearm.length === 0, 'prearm 列表空');

// ---------- 位掩码: 不健康 ----------
r = SC.evaluateSensors(healthyInput({
  sysStatus: { ...healthySys, sensorsHealth: 47 & ~4 },  // 磁力计 health=0
}));
check(r.verdict === 'err', `磁力计不健康 → verdict=err (实际 ${r.verdict})`);
byId.gyro && 0;
const m = r.sensors.find((s) => s.id === 'mag');
check(m.status === 'err', '磁力计 err');

// ---------- 位掩码: 未装 (present=0) 不拉低总评 ----------
r = SC.evaluateSensors(healthyInput({
  sysStatus: { sensorsPresent: 1 | 2 | 8 | 32, sensorsEnabled: 1 | 2 | 8 | 32, sensorsHealth: 1 | 2 | 8 | 32 }, // 无磁
}));
check(r.verdict === 'ok', `磁计未装不拉低总评 (实际 ${r.verdict})`);
check(r.sensors.find((s) => s.id === 'mag').status === 'na', '磁力计 na');

// ---------- 无 SYS_STATUS ----------
r = SC.evaluateSensors({ ...healthyInput(), sysStatus: null });
check(r.sensors.find((s) => s.id === 'gyro').status === 'na', '无 sys_status → 陀螺 na');
check(r.sensors.find((s) => s.id === 'prearm').status === 'na', '无 sys_status → PreArm na');

// ---------- 振动阈值 ----------
// 峰值 >= vibWarn → err
r = SC.evaluateSensors(healthyInput({
  vibration: Array.from({ length: 20 }, () => ({ vibX: 35, vibY: 5, vibZ: 5, clipping: [0, 0, 0] })),
}));
check(r.sensors.find((s) => s.id === 'vibration').status === 'err', `振动峰值 35 → err (>=${T.vibWarn})`);

// 均值在 [vibOk, vibWarn) → warn
r = SC.evaluateSensors(healthyInput({
  vibration: Array.from({ length: 20 }, () => ({ vibX: 18, vibY: 5, vibZ: 5, clipping: [0, 0, 0] })),
}));
check(r.sensors.find((s) => s.id === 'vibration').status === 'warn', `振动均值 18 → warn (>=${T.vibOk})`);

// clipping > 0 → warn (即使振动低)
r = SC.evaluateSensors(healthyInput({
  vibration: Array.from({ length: 20 }, () => ({ vibX: 2, vibY: 2, vibZ: 2, clipping: [3, 0, 0] })),
}));
check(r.sensors.find((s) => s.id === 'vibration').status === 'warn', '振动裁剪>0 → warn');

// ---------- EKF 阈值 ----------
r = SC.evaluateSensors(healthyInput({
  ekf: [{ velocityVariance: 3.0, posHorizVariance: 0.5, posVertVariance: 0.3, compassVariance: 0.05, flags: 47 }],
}));
check(r.sensors.find((s) => s.id === 'ekf').status === 'err', `EKF 速度方差 3 → err (>=${T.ekfVelWarn})`);

r = SC.evaluateSensors(healthyInput({
  ekf: [{ velocityVariance: 1.0, posHorizVariance: 0.5, posVertVariance: 0.3, compassVariance: 0.05, flags: 47 }],
}));
check(r.sensors.find((s) => s.id === 'ekf').status === 'warn', `EKF 速度方差 1 → warn (>=${T.ekfVelOk})`);

// flags 缺姿态/速度 → 至少 warn
r = SC.evaluateSensors(healthyInput({
  ekf: [{ velocityVariance: 0.1, posHorizVariance: 0.5, posVertVariance: 0.3, compassVariance: 0.05, flags: 0 }],
}));
check(r.sensors.find((s) => s.id === 'ekf').status === 'warn', 'EKF flags=0 → warn');

// ---------- IMU 卡死 (加计冻结) ----------
r = SC.evaluateSensors(healthyInput({
  imu: Array.from({ length: 30 }, () => ({ ax: 1.0, ay: 2.0, az: 9.8, temperature: 35 })),
}));
check(r.sensors.find((s) => s.id === 'imu').status === 'err', 'IMU 加计冻结 → err (卡死)');

// IMU 温度超界 → warn
r = SC.evaluateSensors(healthyInput({
  imu: Array.from({ length: 30 }, (_, i) => ({ ax: 0.1 + (i % 2) * 0.1, ay: 0.2, az: 9.8, temperature: 85 })),
}));
check(r.sensors.find((s) => s.id === 'imu').status === 'warn', 'IMU 温度 85°C → warn');

// ---------- 气压读数 ----------
r = SC.evaluateSensors(healthyInput({
  baro: [{ pressAbs: 50, temperature: 25 }],  // 气压异常低
}));
check(r.sensors.find((s) => s.id === 'baro_read').status === 'warn', '气压 50hPa → warn');

// ---------- GPS ----------
r = SC.evaluateSensors(healthyInput({ gps: { fixType: 1, satellites: 3, eph: 999 } }));
check(r.sensors.find((s) => s.id === 'gps_qual').status === 'err', 'GPS fix=1 → err');

r = SC.evaluateSensors(healthyInput({ gps: { fixType: 3, satellites: 6, eph: 120 } }));
check(r.sensors.find((s) => s.id === 'gps_qual').status === 'warn', 'GPS 星数 6 → warn (<10)');

// ---------- PreArm ----------
r = SC.evaluateSensors(healthyInput({ prearm: ['PreArm: Compass not healthy', 'PreArm: Gyro calibration failed'] }));
check(r.verdict === 'err', `PreArm 失败 → verdict=err (实际 ${r.verdict})`);
check(r.prearm.length === 2, 'prearm 列表 2 条');
check(r.prearm[0] === 'Compass not healthy', 'prearm 前缀已剥离');
check(r.sensors.find((s) => s.id === 'prearm').status === 'err', 'PreArm 卡片 err');

// ---------- 无振动/EKF/IMU 数据 (PX4 场景) ----------
r = SC.evaluateSensors({
  sysStatus: healthySys,
  vibration: [], ekf: [], imu: [], baro: [],
  gps: { fixType: 4, satellites: 16, eph: 80 },
  prearm: [],
});
check(r.sensors.find((s) => s.id === 'vibration').status === 'na', '无 VIBRATION → na');
check(r.sensors.find((s) => s.id === 'ekf').status === 'na', '无 EKF → na');
check(r.verdict === 'ok', `数据缺失不拉低总评 (实际 ${r.verdict})`);

// ---------- buildReport ----------
const rep = SC.buildReport(SC.evaluateSensors(healthyInput()), { firmware: 'ArduPilot', durationMs: 30000, sampled: { vib: 30, ekf: 10, imu: 50, baro: 10 } });
check(typeof rep === 'string' && rep.includes('传感器体检报告'), '报告含标题');
check(rep.includes('结论: ✅ 通过'), '报告结论通过');
check(rep.includes('陀螺仪'), '报告含传感器名');

const repBad = SC.buildReport(SC.evaluateSensors(healthyInput({ prearm: ['PreArm: fail'] })), { firmware: 'ArduPilot', durationMs: 10000 });
check(repBad.includes('❌'), '故障报告含 ❌');

// ---------- 常量 ----------
check(SC.SENSOR_BITS.some((s) => s.id === 'gyro' && s.bit === 1), 'SENSOR_BITS 陀螺=1');
check(SC.SENSOR_BITS.some((s) => s.id === 'mag' && s.bit === 4), 'SENSOR_BITS 磁=4');
check(SC.SENSOR_BITS.some((s) => s.id === 'gyro3' && s.ext && s.bit === 1), 'SENSOR_BITS 第3陀螺 扩展位');
check(SC.SENSOR_BITS.some((s) => s.id === 'accel3' && s.ext && s.bit === 2), 'SENSOR_BITS 第3加计 扩展位');
check(SC.SENSOR_BITS.some((s) => s.id === 'mag3' && s.ext && s.bit === 4), 'SENSOR_BITS 第3磁计 扩展位');
check(SC.SENSOR_BITS.some((s) => s.id === 'battery' && s.bit === 33554432), 'SENSOR_BITS 电池监测');
check(SC.EKF_FLAG_NAMES[1] === '姿态', 'EKF_FLAG_NAMES 姿态');

// ---------- 扩展位 (第3组 IMU) ----------
// 扩展掩码 gyro3(bit0)+accel3(bit1)+mag3(bit2) = 7, 全健康
r = SC.evaluateSensors(healthyInput({
  sysStatus: {
    ...healthySys,
    sensorsPresentExt: 7, sensorsEnabledExt: 7, sensorsHealthExt: 7,
  },
}));
check(r.sensors.find((s) => s.id === 'gyro3').status === 'ok', '扩展位 第3陀螺 ok');
check(r.sensors.find((s) => s.id === 'accel3').status === 'ok', '扩展位 第3加计 ok');
check(r.sensors.find((s) => s.id === 'mag3').status === 'ok', '扩展位 第3磁计 ok');

// 扩展位 present 但 health=0 → err
r = SC.evaluateSensors(healthyInput({
  sysStatus: {
    ...healthySys,
    sensorsPresentExt: 7, sensorsEnabledExt: 7, sensorsHealthExt: 7 & ~1,  // gyro3 不健康
  },
}));
check(r.sensors.find((s) => s.id === 'gyro3').status === 'err', '扩展位 第3陀螺 不健康 → err');
check(r.sensors.find((s) => s.id === 'accel3').status === 'ok', '扩展位 第3加计 仍 ok');

// 无扩展字段 (老固件) → 扩展位 na 不拉低总评
r = SC.evaluateSensors(healthyInput());
check(r.sensors.find((s) => s.id === 'gyro3').status === 'na', '无扩展字段 → 第3陀螺 na');
check(r.verdict === 'ok', `扩展位缺失不拉低总评 (实际 ${r.verdict})`);

console.log(failures === 0 ? '\n传感器体检单测全部通过' : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
