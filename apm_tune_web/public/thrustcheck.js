// 推力线性化体检: 纯逻辑层 (浏览器 window.ThrustCheck / Node vm 单测共用, 无 DOM 依赖)
// 曲线公式取自 ArduPilot 源码 libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:
//   apply_thrust_curve: a = ((e-1) + sqrt((1-e)^2 + 4*e*lift_max*T)) / (2*e)   (e=0 时 a = lift_max*T)
//   AP 假设的电机模型: 推力 = (1-e)*a + e*a^2 (e=0 线性 / e=1 平方), 控制器先反演该模型再下发油门
(function () {
  const TC_PARAMS = [
    'MOT_THST_EXPO', 'MOT_THST_HOVER', 'MOT_HOVER_LEARN',
    'MOT_SPIN_ARM', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX',
    'MOT_PWM_MIN', 'MOT_PWM_MAX', 'MOT_PWM_TYPE',
  ];
  // 手工输入优先开放的核心参数 (其余只读展示)
  const TC_EDITABLE = ['MOT_THST_EXPO', 'MOT_THST_HOVER', 'MOT_SPIN_ARM', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX'];
  const PWM_TYPE_LABELS = [
    'PWM(50/100Hz)', 'OneShot', 'OneShot125', 'DShot1200',
    'DShot900', 'DShot600', 'DShot300', 'DShot150',
  ];

  // 控制器期望推力 T(0~1) -> 线性化后的油门指令 a(0~1)
  function expoToActuator(t, expo, liftMax) {
    const e = Math.max(-1, Math.min(1, Number(expo) || 0));
    const lm = (liftMax === undefined || liftMax === null || !isFinite(Number(liftMax))) ? 1 : Number(liftMax);
    t = Math.max(0, Math.min(1, Number(t) || 0));
    if (Math.abs(e) < 1e-9) return Math.max(0, Math.min(1, lm * t));
    const disc = (1 - e) * (1 - e) + 4 * e * lm * t;
    const a = ((e - 1) + Math.sqrt(Math.max(0, disc))) / (2 * e);
    return Math.max(0, Math.min(1, a));
  }

  // 补偿曲线采样 [{t, a}], 供画图使用
  function curvePoints(expo, opts) {
    const steps = (opts && opts.steps) || 40;
    const liftMax = opts && opts.liftMax;
    const pts = [];
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      pts.push({ t, a: expoToActuator(t, expo, liftMax) });
    }
    return pts;
  }

  // 由油门指令 a 反查期望推力 T (二分, 曲线单调; 用于在曲线上标注悬停点)
  function actuatorToExpoThrust(a, expo, liftMax) {
    let lo = 0; let hi = 1;
    for (let i = 0; i < 40; i++) {
      const mid = (lo + hi) / 2;
      if (expoToActuator(mid, expo, liftMax) < a) lo = mid; else hi = mid;
    }
    return (lo + hi) / 2;
  }

  /**
   * 依据参数值生成体检行。
   * @param values {Object} {参数名: 数值}; 缺失的参数视为"未读到"
   * @param opts {escLinear: 电调已内置线性化}
   * @returns [{name, value, level(ok|info|warn|na), text}]
   */
  function buildRows(values, opts) {
    values = values || {};
    const escLinear = !!(opts && opts.escLinear);
    const has = (n) => values[n] !== undefined && values[n] !== null && isFinite(Number(values[n]));
    const num = (n) => Number(values[n]);
    const rows = [];
    const add = (name, value, level, text) => rows.push({ name, value, level, text });

    // 输出协议（对应"先看电调协议"）
    if (!has('MOT_PWM_TYPE')) {
      add('MOT_PWM_TYPE', '--', 'na', '未读到该参数，跳过协议判定');
    } else {
      const t = num('MOT_PWM_TYPE');
      const label = PWM_TYPE_LABELS[t] !== undefined ? PWM_TYPE_LABELS[t] : `type=${t}`;
      if (t === 0) add('MOT_PWM_TYPE', label, 'info', '传统 PWM 时序：需要电调行程校准，油门-推力非线性靠 MOT_THST_EXPO 补偿');
      else if (t <= 2) add('MOT_PWM_TYPE', label, 'info', 'OneShot 类：仍属模拟 PWM 时序，建议配合 EXPO 补偿并做地面实测');
      else add('MOT_PWM_TYPE', label, 'ok', 'DShot 系列：电调端通常已接近线性，一般无需传统 PWM 行程校准');
    }

    // MOT_THST_EXPO（核心判断）
    if (!has('MOT_THST_EXPO')) {
      add('MOT_THST_EXPO', '--', 'na', '未读到该参数');
    } else {
      const v = num('MOT_THST_EXPO');
      let level = 'ok';
      let text = '';
      if (v <= 0.05) text = '≈0，飞控几乎不做线性化补偿 → 动力链已较线性，无需再调';
      else if (v <= 0.2) text = '低补偿(0~0.2)，与"电调已内置线性化"匹配，无需再调';
      else if (v < 0.45) { level = 'info'; text = '中等补偿，按桨径属偏低区间，建议地面油门阶梯实测确认低段无死区'; }
      else { level = 'info'; text = '明显补偿（官方按桨径 5寸0.55/10寸0.65/20寸0.75），属正常配置，不等于电调硬件不线性'; }
      if (escLinear && v > 0.3) {
        level = 'warn';
        text = `已勾选"电调已内置推力线性化"，但 EXPO=${v} 偏大 → 建议改为 0.10 左右`;
      }
      add('MOT_THST_EXPO', v, level, text);
    }

    // MOT_THST_HOVER
    if (!has('MOT_THST_HOVER')) {
      add('MOT_THST_HOVER', '--', 'na', '未读到该参数');
    } else {
      const v = num('MOT_THST_HOVER');
      if (v < 0.05) add('MOT_THST_HOVER', v, 'warn', '低于 0.05，接近电机死区，低油门段控制余量不足');
      else if (v > 0.6) add('MOT_THST_HOVER', v, 'warn', '高于 0.6，动力余量偏小，上升与抗风能力受限');
      else add('MOT_THST_HOVER', v, 'ok', '在正常区间 0.05~0.5；多次飞行保持稳定说明动力链路较健康');
    }

    // 悬停自学习
    if (!has('MOT_HOVER_LEARN')) {
      add('MOT_HOVER_LEARN', '--', 'na', '未读到该参数');
    } else {
      const v = num('MOT_HOVER_LEARN');
      if (v === 0) add('MOT_HOVER_LEARN', v, 'info', '悬停油门自学习已关闭，MOT_THST_HOVER 不会随飞行更新（建议 1 或 2）');
      else add('MOT_HOVER_LEARN', v, 'ok', '自学习开启，悬停油门会在飞行中自动更新');
    }

    // 交叉：悬停油门 vs 飞行最小输出
    if (has('MOT_THST_HOVER') && has('MOT_SPIN_MIN')) {
      const h = num('MOT_THST_HOVER');
      const s = num('MOT_SPIN_MIN');
      if (h <= s + 0.03) add('HOVER ↔ SPIN_MIN', `${h} / ${s}`, 'warn', '悬停油门过于接近飞行最小输出，低油门修正几乎没有余量');
      else add('HOVER ↔ SPIN_MIN', `${h} / ${s}`, 'ok', '悬停油门高于最小输出，低油门段仍有控制余量');
    } else add('HOVER ↔ SPIN_MIN', '--', 'na', '缺 MOT_THST_HOVER 或 MOT_SPIN_MIN，跳过');

    // 交叉：飞行最小输出 vs 解锁怠速
    if (has('MOT_SPIN_MIN') && has('MOT_SPIN_ARM')) {
      const mn = num('MOT_SPIN_MIN');
      const ar = num('MOT_SPIN_ARM');
      if (mn < ar + 0.03) add('SPIN_MIN ↔ SPIN_ARM', `${mn} / ${ar}`, 'warn', 'MOT_SPIN_MIN 必须 ≥ MOT_SPIN_ARM + 0.03，否则起飞段输出不线性、离地瞬间易抖动');
      else add('SPIN_MIN ↔ SPIN_ARM', `${mn} / ${ar}`, 'ok', '怠速与飞行最小输出之间留有 0.03 以上裕度');
    } else add('SPIN_MIN ↔ SPIN_ARM', '--', 'na', '缺 MOT_SPIN_MIN 或 MOT_SPIN_ARM，跳过');

    // PWM 行程
    if (has('MOT_PWM_MIN') && has('MOT_PWM_MAX')) {
      const lo = num('MOT_PWM_MIN');
      const hi = num('MOT_PWM_MAX');
      const span = hi - lo;
      if (span < 300) add('MOT_PWM_MIN~MAX', `${lo}~${hi}us`, 'warn', `行程仅 ${span}us，过窄，油门分辨率与线性度都会变差`);
      else if (lo === 1000 && hi === 2000) add('MOT_PWM_MIN~MAX', `${lo}~${hi}us`, 'ok', '标准行程，与电调行程校准一致');
      else add('MOT_PWM_MIN~MAX', `${lo}~${hi}us`, 'info', '非标准行程，请确认与电调校准后的实际行程一致');
    } else add('MOT_PWM_MIN~MAX', '--', 'na', '未读到（部分固件用 SERVO1_MIN/MAX 表示输出行程）');

    // 最大输出
    if (!has('MOT_SPIN_MAX')) {
      add('MOT_SPIN_MAX', '--', 'na', '未读到该参数');
    } else {
      const v = num('MOT_SPIN_MAX');
      if (v > 1.0 || v < 0.5) add('MOT_SPIN_MAX', v, 'warn', '取值超出常规 0.5~1.0');
      else if (v < 0.8) add('MOT_SPIN_MAX', v, 'info', `最大输出被限制为 ${v}，满油门以上无余量`);
      else add('MOT_SPIN_MAX', v, 'ok', '取值正常');
    }

    return rows;
  }

  // 文字结论 (rows 为 buildRows 输出, values 为原始参数值)
  function verdict(rows, values) {
    values = values || {};
    const warns = rows.filter((r) => r.level === 'warn').length;
    const head = warns ? `发现 ${warns} 项需要关注（见上表）。` : '未发现需要关注的异常项。';
    const evOk = values.MOT_THST_EXPO !== undefined && values.MOT_THST_EXPO !== null
      && isFinite(Number(values.MOT_THST_EXPO));
    if (!evOk) return head + ' 未读到 MOT_THST_EXPO，无法给出线性化结论。';
    const v = Number(values.MOT_THST_EXPO);
    let tail;
    if (v <= 0.2) {
      tail = `EXPO=${v} 很小，飞控几乎不做线性化补偿 → 可认为当前电调+电机+桨已较线性，MOT_THST_EXPO 无需再调。`;
    } else if (v < 0.45) {
      tail = `EXPO=${v} 属中等补偿 → 参数本身可保持，但建议做一次地面油门阶梯实测，确认低油门段无死区、无突跳。`;
    } else {
      tail = `EXPO=${v} 明显（官方按桨径 5寸0.55/10寸0.65/20寸0.75，属正常配置）→ 不能据此判断电调硬件不线性；若悬停附近易振荡、小油门修正不跟手、自动调参反复变化，再考虑调整 EXPO。`;
    }
    return head + tail + ' 本结论基于参数体检，真实推力线性度仍需油门阶梯实测确认。';
  }

  window.ThrustCheck = {
    TC_PARAMS, TC_EDITABLE, PWM_TYPE_LABELS,
    expoToActuator, curvePoints, actuatorToExpoThrust,
    buildRows, verdict,
  };
})();
