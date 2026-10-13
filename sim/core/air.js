// 大气模型：ISA 简化模型 + Sutherland 黏度公式
export function airModel({ altitude = 0, tempC = 25 } = {}) {
  const alt = Math.max(0, Math.min(altClamp(altitude), 11000));
  const T = tempC + 273.15;
  const p = 101325 * Math.pow(1 - 2.25577e-5 * alt, 5.25588);
  const rho = p / (287.0538 * T);
  const mu = 1.458e-6 * Math.pow(T, 1.5) / (T + 110.4);
  const nu = mu / rho;
  const sound = Math.sqrt(1.4 * 287.0538 * T);
  return {
    altitude: alt, tempC, T, p, rho, mu, nu, sound,
    densityRatio: rho / 1.225,
  };
}

function altClamp(a) {
  return Number.isFinite(a) ? a : 0;
}

// 雷诺数
export function reynolds(V, L, nu) {
  if (!V || !L) return 0;
  return Math.abs(V) * L / nu;
}
