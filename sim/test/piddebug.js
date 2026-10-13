// 调试 12S 八轴 settle=null 与 1S whoop 慢响应
import { simulate } from '../core/simulate.js';

const R = simulate({ type: 'multirotor', frameId: 'x8heavy', rotorCount: 8, cells: 12, capacity_mAh: 10000, motorType: '8120', kv: 150, propD: 432, propP: 152, propB: 2, escType: 'hv', target_kmh: 60, payload_g: 5000 });
const pid = R.dynamics.pid;
console.log('gain', pid.gain, 'plant', pid.plant);
console.log('pid', pid.pid);
console.log('resp: over', pid.response.overshoot, 'settle', pid.response.settling_ms, 'rise', pid.response.rise_ms, 'final', pid.response.final, 'distResid', pid.response.distResidual, 'distOk', pid.response.distRejected);
const d = pid.response.data;
console.log('trace tail:');
for (const p of d.slice(-12)) console.log(' ', p.t, p.v);
console.log('trace mid (around settle):');
for (const p of d) if (p.t > 0.02 && p.t < 0.3 && Math.round(p.t * 1000) % 20 === 0) console.log(' ', p.t, p.v);
