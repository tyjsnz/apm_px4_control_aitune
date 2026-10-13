// 全域稳健性验证：wn=1/τ, iDiv=150, auth=0.1
function design(K, tau, iDiv = 150, auth = 0.1) {
  const wn = Math.min(1 / tau, (0.8 * K) / 3, 150);
  const P = (wn * wn * tau) / K;
  const D = Math.max(0, (2 * tau * wn - 1) / K);
  const Ii = (P * wn) / iDiv;
  const clamp = auth / Ii;
  const dfilt = Math.min(0.02, Math.max(0.001, tau / 2.5));
  const dt = 0.0004, N = 12500;
  let x = 0, xm = 0, ii = 0, filt = 0, xPrev = 0;
  const sp = 10; let peak = 0; const resp = [];
  for (let k = 0; k < N; k++) {
    const err = sp - x;
    const dMeas = -(x - xPrev) / dt; xPrev = x;
    filt += (dt / (dfilt + dt)) * (dMeas - filt);
    ii = Math.min(clamp, Math.max(-clamp, ii + err * dt));
    const u = Math.min(1, Math.max(-1, P * err + Ii * ii + D * filt));
    xm += ((K * u - xm) / tau) * dt;
    x += xm * dt;
    peak = Math.max(peak, x);
    if (k % 4 === 0) resp.push({ t: k * dt, v: x });
  }
  const over = Math.max(0, (peak - sp) / sp * 100);
  let settle = null;
  for (let i = 0; i < resp.length; i++) {
    if (resp.slice(i).every(r => Math.abs(r.v - sp) <= 0.02 * sp)) { settle = resp[i].t * 1000; break; }
  }
  // 抗扰 8%
  let y = 0, ym = 0, ii2 = 0, f2 = 0, yp = 0;
  const dt2 = 0.001;
  for (let k = 0; k < 30000; k++) {
    const err = -y;
    const dM = -(y - yp) / dt2; yp = y;
    f2 += (dt2 / (dfilt + dt2)) * (dM - f2);
    ii2 = Math.min(clamp, Math.max(-clamp, ii2 + err * dt2));
    const u2 = Math.min(1, Math.max(-1, P * err + Ii * ii2 + D * f2));
    ym += ((K * u2 - 0.08 * K - ym) / tau) * dt2;
    y += ym * dt2;
  }
  return { wn: +wn.toFixed(1), over: +over.toFixed(1), settle: settle, resid: +y.toFixed(2), final: +x.toFixed(3) };
}

let n = 0, bad = 0, wOver = 0, wSet = 0;
for (const tau of [0.004, 0.006, 0.01, 0.02, 0.035, 0.053, 0.08, 0.12]) {
  for (const K of [80, 200, 500, 1200, 3000, 8000, 20000]) {
    const r = design(K, tau);
    n++;
    wOver = Math.max(wOver, r.over);
    wSet = Math.max(wSet, r.settle ?? 99999);
    const ok = r.over <= 10 && (r.settle ?? 99999) <= 500 && Math.abs(r.resid) <= 0.5;
    if (!ok) { bad++; console.log(`BAD tau=${tau * 1000}ms K=${K}: wn=${r.wn} over=${r.over}% set=${r.settle} resid=${r.resid}`); }
  }
}
console.log(`样本 ${n} 超差 ${bad} | 最坏超调 ${wOver}% 最坏调节 ${wSet}ms`);

// 单位阶跃跟踪精度检查（iTerm 权限 0.1 是否限制大误差积分——改为检查 setpoint 变化时响应）
const r = design(1899.7, 0.02);
console.log('5寸样例:', r);
console.log('大机样例:', design(517, 0.0529));
