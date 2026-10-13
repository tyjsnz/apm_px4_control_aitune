import { simulate } from '../core/simulate.js';

function show(name, input) {
  const t0 = performance.now();
  const R = simulate(input);
  const ms = (performance.now() - t0).toFixed(1);
  console.log(`\n######## ${name}  (${ms}ms) ########`);
  console.log('型别:', R.typeLabel, '| 重量:', (R.totalMass_g / 1000).toFixed(2), 'kg | 电机:', R.motor.label, '| 桨:', R.prop.label, '|', R.battery.label);
  const c = R.cases;
  if (R.type === 'multirotor') {
    console.log(`推重比 ${c.twr.toFixed(2)}  最大推力 ${(c.maxThrust / 9.80665).toFixed(2)}kgf  悬停油门 ${(c.hover.duty * 100).toFixed(1)}%  悬停电流 ${c.hover.current.toFixed(1)}A  ${c.hover.gPerW.toFixed(2)}g/W  ${Math.round(c.hover.rpm)}rpm`);
    console.log(`满油门 ${(c.full.thrust / 9.80665).toFixed(2)}kgf  ${c.full.current.toFixed(1)}A  ${Math.round(c.full.power)}W  桨尖 ${c.full.vTip.toFixed(0)}m/s  M${c.full.machTip.toFixed(2)}`);
    console.log(`最高速 ${c.vmax.toFixed(0)}km/h  目标 ${c.target?.kmh}km/h 可达=${c.target?.reachable} 油门=${(c.target?.duty * 100).toFixed(0)}% 倾角=${c.target?.tiltDeg?.toFixed(1)}  续航悬停=${c.endurance.hover?.toFixed(1)}min`);
    console.log(`加速到目标: ${c.accelToTarget?.toFixed(2)}s`);
  } else {
    console.log(`失速 ${c.vStall.toFixed(0)}km/h  最高 ${c.vmax.toFixed(0)}km/h  最佳升阻比 ${c.maxLD?.toFixed(1)} @ ${c.maxLDSpeed?.toFixed(0)}km/h  最省电 ${c.minSinkSpeed?.toFixed(0)}km/h`);
    console.log(`目标 ${c.target?.kmh}km/h 可达=${c.target?.reachable} 功率${c.target?.power?.toFixed(0)}W 续航${c.target?.minutes != null ? c.target.minutes.toFixed(1) + 'min' : '-'} 升阻比${c.target?.L_D?.toFixed(1)} 失速=${c.target?.stalled}`);
    console.log(`爬升 ${c.climb?.vs_fpm?.toFixed(0)}ft/min @ ${c.climb?.V}m/s`);
  }
  console.log('结构: 板厚', R.structure.armPlate.thickness_mm, 'mm, 应力', R.structure.armPlate.sigma_MPa, 'MPa, SF', R.structure.armPlate.safety, '| 桨离心 SF', R.structure.prop.safety);
  if (R.dynamics.pid) console.log('控制: τ=', R.dynamics.actuator.tau_ms, 'ms  PID P=', R.dynamics.pid.pid.P, 'D=', R.dynamics.pid.pid.D, ' 超调', R.dynamics.pid.response.overshoot + '%', '调节', R.dynamics.pid.response.settling_ms + 'ms');
  console.log('流体: Re桨', R.fluid.reBlade.toExponential(2), '| 噪音', R.fluid.noise1m.toFixed(0), 'dB@1m |', R.fluid.regime);
  console.log('诊断:');
  for (const i of R.issues) console.log(`  [${i.level}] ${i.title} — ${i.msg}`);
  const S = R.solutions;
  console.log(`反向求解: 方案 ${S.solutions.length} 个（Pareto ${S.pareto.length}）| 扫描 ${S.scanned} 点 | 达标 ${S.solutions.filter(s => s.reached).length}`);
  for (const s of S.solutions.slice(0, 2)) console.log(`  ${s.reached ? '✓' : '⚠'} ${s.label}: ${s.changeLabel}`);
  return R;
}

show('5寸 FPV 竞速 (默认)', { type: 'multirotor', frameId: 'fpv5', cells: 4, capacity_mAh: 1300, motorType: '2207', kv: 1750, propD: 127, propP: 109, propB: 3, target_kmh: 120 });
show('7寸 长航时', { type: 'multirotor', frameId: 'longrange', cells: 4, capacity_mAh: 3000, motorType: '3110', kv: 950, propD: 178, propP: 102, target_kmh: 80 });
show('12S 大载重八轴', { type: 'multirotor', frameId: 'x8heavy', rotorCount: 8, cells: 12, capacity_mAh: 10000, motorType: '8120', kv: 150, propD: 432, propP: 152, propB: 2, escType: 'hv', target_kmh: 60, payload_g: 5000 });
show('1S 65mm 穿越机 (不良配置)', { type: 'multirotor', frameId: 'whoop65', cells: 1, capacity_mAh: 300, motorType: '0804', kv: 5000, propD: 51, propP: 76, target_kmh: 80 });
show('1000mm FPV 飞翼', { type: 'fixedwing', wingId: 'wing1000', cells: 4, capacity_mAh: 2200, motorType: '4114', kv: 650, propD: 203, propP: 152, propB: 2, target_kmh: 100 });
show('2m 滑翔机 长航时', { type: 'fixedwing', wingId: 'glider2m', cells: 4, capacity_mAh: 5000, motorType: '4114', kv: 650, propD: 254, propP: 152, propB: 2, target_kmh: 80 });
