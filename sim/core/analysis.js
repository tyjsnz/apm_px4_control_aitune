// ============================================================
// 诊断规则 + 中文分析报告生成（结构 / 空气动力 / 流体 / 能量 / 控制）
// ============================================================
import { G } from './parts.js';
import { T } from './tuning.js';

const pct = (x) => `${(x * 100).toFixed(1)}%`;
const n1 = (x, d = 1) => Number(x).toFixed(d);

// ---------------------------- 诊断规则 ----------------------------
export function diagnose(R) {
  const out = [];
  const add = (level, code, title, msg, fix, solutionRef) => out.push({ level, code, title, msg, fix, solutionRef });

  const isMR = R.type === 'multirotor';
  const isVTOL = R.type === 'vtol';
  const hoverMode = isMR || isVTOL;
  const td = T().diag;   // 判据阈值（可在线调整）

  if (hoverMode) {
    const h = R.cases.hover, twr = R.cases.twr;
    const tilt = isMR && R.cases.maxTiltDeg != null ? `，最大倾角 ${n1(R.cases.maxTiltDeg, 0)}°` : '';
    if (twr < td.twrError) add('error', 'twr', '推重比 < 1，无法起飞', `最大推力 ${n1(R.cases.maxThrust / 9.80665)}kgf < 起飞重量 ${n1(R.cases.W / 9.80665)}kgf，推重比 ${n1(twr, 2)}。`, '降低整机重量 / 加大桨径 / 提高 KV / 增加电芯数，直到推重比 > 1.5。');
    else if (twr < td.twrWarn) add('warn', 'twr', '推重比偏低，抗风与机动能力差', `推重比 ${n1(twr, 2)}${tilt}，阵风下容易掉高、飞控饱和。` + (isVTOL ? '垂起火箭在转换段还需额外推力裕度，此值明显不足。' : ''), '目标：巡航机 ≥1.8，航拍机 ≥2.0，穿越机 ≥5；垂起机型建议 ≥2.5 以保证转换安全。');
    else if (twr > (isVTOL ? td.twrHighVTOL : td.twrHighMR)) add('warn', 'twr', '推重比过高', `推重比 ${n1(twr, 2)}，油门极度灵敏，悬停处于油门前段，能耗与噪音都偏高。`, '可换小一号桨或降 KV，让悬停油门落在 35%~50%。');
    else add('ok', 'twr', '推重比合适', `推重比 ${n1(twr, 2)}${tilt}，兼顾机动与效率。`, '');

    const hd = h.duty;
    if (hd > td.hoverDutyError) add('error', 'hover', '悬停油门过高，无动力余量', `悬停需 ${pct(hd)} 油门、${n1(h.current, 1)}A，遇障碍/机动即满油门。`, '换大一号桨、降低 KV，或减少电池容量/重量。');
    else if (hd > td.hoverDutyWarn) add('warn', 'hover', '悬停油门偏高', `悬停油门 ${pct(hd)}，剩余动力余量约 ${pct(1 - hd)}。`, '优化方向：增大桨径或增加桨距，让悬停油门降到 35%~50%。');
    else if (hd < td.hoverDutyMin) add('warn', 'hover', '悬停油门过低', `悬停仅需 ${pct(hd)}，动力严重过剩，油门分辨率差、起步易抖。`, '换小一号桨或降 KV；也可减小电池容量减重。');
    else add('ok', 'hover', '悬停油门健康', `悬停油门 ${pct(hd)}，单机效率 ${n1(h.gPerW, 2)} g/W，处于高效区间。`, '');

    if (R.cases.cRatingLimited) add('error', 'batteryC', '电池持续放电能力不足',
      `满油门需求总电流约 ${n1(R.cases.full.currentDemand ?? R.cases.full.current, 0)}A，超过 ${R.battery.label} 的 ${R.cases.batteryMaxCurrent}A 上限，实际推力被电压跌落限制在 ${n1(R.cases.maxThrust / 9.80665, 2)}kgf（推重比 ${n1(R.cases.twr, 2)}），强行满油门会严重压降、发热甚至鼓包。`,
      '提高电池 C 数 / 增大容量（mAh），或限制油门（设置电机输出上限），并按"电池受限"下的推重比重新评估。');
    if (R.cases.full.thermal > 0) {
      const ratio = R.cases.full.thermal / R.motor.PlossBurst;
      if (ratio > 1) add('error', 'motorHeat', '电机持续过流', `满油门铜损已达允许上限的 ${pct(ratio)}，电机会迅速退磁/烧毁。`, '降低 KV、加大桨（降低转速与电流）、或换更大机座电机。');
      else if (ratio > td.motorHeatWarn) add('warn', 'motorHeat', '电机热负荷较高', `满油门铜损为允许值的 ${pct(ratio)}，长时间满油门会过热。`, '正常飞行（非持续满油门）可接受；巡航应控制在 60% 油门内。');
    }
    if (R.cases.full.machTip > td.tipMach) add('warn', 'tipmach', '桨尖速度过高',
      `桨尖马赫数 ${n1(R.cases.full.machTip, 2)}（${n1(R.cases.full.vTip, 0)} m/s），气动噪音剧增、效率下降、桨叶负荷大。`,
      '降 KV 或换低桨距桨，把桨尖马赫控制在 0.6 以下。');
    if (R.fluid.noise10m > td.noise10m) add('warn', 'noise', '噪音偏高', `${n1(R.fluid.noise10m, 0)} dB(A)@10m，城市飞行扰民风险高。`, '换低转速大桨方案（降 KV + 加桨径）。');

    if (R.cases.target && R.cases.target.kmh > 0 && !R.cases.target.reachable) {
      const ps = R.cases.pitchSpeed_kmh;
      add('error', 'target', `目标速度 ${R.cases.target.kmh} km/h 不可达`,
        `该配置最高速度约 ${n1(R.cases.vmax, 0)} km/h。` +
        (isVTOL
          ? (R.cases.target.stalled ? `且此速度下尾翼已失速（超出机身升力体失速边界）。` : (ps ? `桨几何螺距速度（无载）${n1(ps, 0)} km/h。` : ''))
          : `按 ${R.cases.target.kmh} km/h 计算需要单桨 ${n1(R.cases.target.Treq / 9.80665 * 1000, 0)}g 推力，超出可用范围。`),
        isVTOL ? '加大桨距/提高 KV/增加电芯数，或增大尾翼面积提高巡航升阻比；转换段速度需 ≥ 失速速度 ×1.25。'
          : '提高 KV / 增加电芯数 / 减小迎风面积 / 减重，或下调目标速度。');
    } else if (R.cases.target && R.cases.target.kmh > 0) {
      const load = R.cases.target.duty;
      if (load > 0.95) add('warn', 'target', `目标速度余量极小`, `${R.cases.target.kmh} km/h 需要 ${pct(load)} 油门，无法再机动。`, '为该速度预留 ≥15% 油门余量。');
      else if (load > 0.7) add('info', 'target', `目标速度可达，但负荷较高`, `${R.cases.target.kmh} km/h 需 ${pct(load)} 油门、${n1(R.cases.target.current, 0)}A，功率 ${n1(R.cases.target.power, 0)}W。`, '若需长期保持，建议把目标下调到 70% 油门以内以延长续航。');
      else add('ok', 'target', '目标速度轻松可达', `${R.cases.target.kmh} km/h 仅需 ${pct(load)} 油门，还有 ${pct(1 - load)} 动力余量` + (R.cases.target.minutes != null ? `，续航 ${n1(R.cases.target.minutes, 1)} 分钟。` : '。'), '');
    }

    const arm = R.structure.armPlate;
    if (arm.safety < td.armSFError) add('error', 'arm', '机臂强度不足', `${arm.thickness_mm}mm 碳板应力 ${arm.sigma_MPa}MPa，安全系数仅 ${arm.safety}，炸机风险高。`, `机臂板厚需 ≥${n1(arm.required_mm, 1)}mm，或加宽机臂/加三角支撑。`);
    else if (arm.safety < td.armSFWarn) add('info', 'arm', '机臂强度可优化', `安全系数 ${arm.safety}，最大挠度 ${arm.deflection_mm}mm。`, `按 ${n1(arm.required_mm, 1)}mm 设计即可满足，更多余量会增重。`);
    else add('ok', 'arm', '结构强度充足', `机臂安全系数 ${arm.safety}，挠度 ${arm.deflection_mm}mm，固有频率 ${R.structure.armFreq_Hz}Hz。`, '');

    const pf = R.dynamics.actuator.passing_Hz;
    const af = R.structure.armFreq_Hz;
    if (pf > 0 && Math.abs(af - pf) / pf < 0.18) add('warn', 'vibration', '机臂固有频率接近桨叶通过频率',
      `机臂 ${af}Hz vs 桨叶通过频率 ${pf}Hz，可能引发共振、图像果冻、D 项噪声。`,
      '增加机臂刚度（加厚/加三角支撑）或换桨叶数改变通过频率，并降低 D 项低通频率。');

    if (isVTOL) {
      const tr = R.vtol?.transition;
      const hs = R.vtol?.hoverStability;
      if (tr) {
        const band = tr.cautionBand_deg ? `尾翼失速过渡带约 ${tr.cautionBand_deg[0]}°→${tr.cautionBand_deg[1]}° 机头角` : '';
        if (!tr.feasible) add('error', 'transition', '转换走廊不安全（转换段可能失控）',
          `俯仰 90°→0° 共 ${tr.rows.length} 个采样点中 ${tr.unsafeCount} 个硬性失败（无解/推力无裕度/控制权限不足），最小推力裕度 ${tr.minMargin_pct}%，最小差动角加速度 ${tr.minControlAlpha} rad/s²。`,
          `提高推重比（目标 ≥2.5）、降低失速速度（加大尾翼面积/减轻重量）、或收窄转换速度窗口（${tr.vEntry_kmh}→${tr.vExit_kmh} km/h）。`);
        else add('ok', 'transition', '转换走廊可达（含失速过渡带）', `全部俯仰角均有正推力裕度（最小 ${tr.minMargin_pct}%），推荐转换速度 ${tr.recommendedV_kmh} km/h${tr.recommendedPhi != null ? `（约 ${tr.recommendedPhi}° 机头角）` : ''}${band ? `；${band}需快速穿越` : ''}。`, '');
        if (tr.cautionCount > 0 && tr.feasible) add('info', 'transitionBand', '存在尾翼失速过渡带',
          `共 ${tr.cautionCount} 个采样点处于「机翼失速但依赖推力的过渡带」，此时尾翼无舵效、仅靠差动推力控姿，遇到阵风姿态会短暂发散。`,
          '方法一：转换时保持大推力、以最短时间穿过该带（推荐）。方法二：加大尾翼/增加机身升力体，降低失速速度以缩窄该带。');
        if (tr.minControlAlpha < 3) add('warn', 'transitionCtrl', '转换段差动推力权限偏低',
          `最小可用差动角加速度仅 ${tr.minControlAlpha} rad/s²，阵风下姿态响应迟缓。`,
          '提高电机/桨推力上限（降 KV 或加大桨）、减小机身转动惯量，或缩短力臂。');
      }
      if (hs) {
        if (hs.inverted) add('warn', 'hoverstab', '悬停为倒立摆（重心在推力面之上）',
          `重心高于推力中心 ${Math.abs(hs.cgBelowThrust_mm)}mm，无自恢复性，需飞控姿态环带宽 ≥${hs.requiredBW_Hz}Hz 持续修正。`,
          '调整电池/载荷位置使重心位于推力面下方（稳定摆），或确认姿态环带宽与采样率满足要求。');
        else if (hs.pendulum_Hz < 1.2) add('info', 'hoverstab', '悬停恢复摆周期较长', `重心低于推力中心 ${hs.cgBelowThrust_mm}mm，单摆频率 ${hs.pendulum_Hz}Hz，恢复较慢。`, '可将电池进一步前移/下移以提高单摆频率，或提高姿态环带宽。');
        else add('ok', 'hoverstab', '悬停稳定性良好', `重心位于推力面下方 ${hs.cgBelowThrust_mm}mm，单摆频率 ${hs.pendulum_Hz}Hz；差动横滚 ${hs.rollAlpha} rad/s²、偏航 ${hs.yawAlpha} rad/s²。`, '');
      }
      if (R.cases.vStall * td.vStallRatio > R.cases.vmax) add('warn', 'fw', '巡航包线过窄', `失速速度 ${n1(R.cases.vStall, 0)}km/h，最高速仅 ${n1(R.cases.vmax, 0)}km/h。`, '增大动力或减轻重量、加大尾翼面积。');
      const vc = R.cases.vertClimb;
      if (vc && vc.vs_fpm < td.vtolClimbMin) add('warn', 'climb', '垂直爬升率不足', `垂直爬升仅 ${n1(vc.vs_fpm, 0)} ft/min，无动力余量应对下降气流。`, '提高推重比到 ≥2.5（转换与抗风都需要）。');
      else if (vc) add('ok', 'climb', '垂直爬升能力充足', `垂直爬升 ${n1(vc.vs_fpm, 0)} ft/min（推力余量 ${pct(vc.margin)}）。`, '');
    }
  } else {
    // 固定翼
    const t = R.cases.target;
    if (R.cases.vStall * td.vStallRatio > R.cases.vmax) add('warn', 'fw', '飞行包线过窄', `失速速度 ${n1(R.cases.vStall, 0)}km/h，最高速仅 ${n1(R.cases.vmax, 0)}km/h。`, '增大动力或减轻重量、增大翼面积。');
    if (t && t.kmh > 0 && !t.reachable) {
      const ps = R.cases.pitchSpeed_kmh;
      const pitchBound = ps && t.kmh > ps * 0.9;
      add('error', 'target', `目标速度 ${t.kmh} km/h 不可达`,
        `该机最高速约 ${n1(R.cases.vmax, 0)} km/h。` +
        (ps ? `当前桨几何螺距速度（无载）${n1(ps, 0)} km/h` + (pitchBound ? `，目标已超过螺距速度——纯螺距限制。` : `，距离螺距天花板仅 ${n1(ps - R.cases.vmax, 0)} km/h。`) : ''),
        pitchBound ? '提高桨距（如 8×6→10×8）或提高 KV / 增加电芯数，使螺距速度 ≥ 目标 ×1.3。'
          : '增推力（更大桨距/更高KV）/ 减阻 / 减重，或下调目标速度。');
    }
    if (t && t.kmh > 0 && t.stalled) add('error', 'stall', '目标速度下已失速', `所需 CL 超过翼型 CLmax=${R.cases.wing.clmax}。`, '降低目标速度、减轻重量或增大翼面积。');
    if (R.cases.maxLD < td.ldMin) add('warn', 'ld', '升阻比偏低', `最大升阻比 ${n1(R.cases.maxLD, 1)}，巡航效率差。`, '增大展弦比、减小迎风面积、表面光洁化。');
    else add('ok', 'ld', '气动效率良好', `最大升阻比 ${n1(R.cases.maxLD, 1)}，出现在 ${n1(R.cases.maxLDSpeed, 0)}km/h。`, '');
    if (R.cases.climb && R.cases.climb.vs_fpm < td.vtolClimbMin) add('warn', 'climb', '爬升率不足', `${n1(R.cases.climb.vs_fpm, 0)} ft/min，抗下降气流能力弱。`, '提高动力或减重。');
  }

  // ---------- 静稳定裕度与舵面权限（固定翼 & 火箭巡航）----------
  const st = R.stability;
  if (st) {
    if (st.SM_pct < 0) add('error', 'sm', '静不稳定（重心在中性点之后）', `静稳定裕度 ${st.SM_pct}%（CG ${st.xCG_pct}% / NP ${st.xNP_pct}% c̄），受扰后姿态发散。`, '前移重心至 25% c̄，或增大尾翼面积/力臂使中性点后移。');
    else if (st.SM_pct > td.smMax) add('warn', 'sm', '静稳定裕度过大', `SM ${st.SM_pct}%，回正力矩过强，舵面可能压不过（配平权限 ${st.elevatorAuth.toFixed(2)}）。`, '后移重心至 25%~30% c̄，或减小尾翼力臂。');
    else if (st.SM_pct < td.smMin) add('warn', 'sm', '静稳定裕度偏小', `SM ${st.SM_pct}%，操纵灵敏但抗扰能力弱、易进入飘摆。`, '重心前移，目标 SM 5%~15%。');
    else add('ok', 'sm', '静稳定裕度合适', `SM ${st.SM_pct}%（CG ${st.xCG_pct}% / NP ${st.xNP_pct}% c̄），短周期 ${st.wn_sp_Hz}Hz、阻尼比 ${st.zeta_sp}。`, '');
    if (st.elevatorAuth < 1) add('warn', 'elev', '舵面配平权限不足', `升降舵权限系数 ${st.elevatorAuth.toFixed(2)}<1，全行程拉杆仍压不过静稳定力矩，会失速掉高度。`, '增大舵面面积/弦长，或后移重心减小 SM。');
    if (st.rollRateMax_dps < 45) add('info', 'roll', '滚转速率偏低', `满副翼滚转仅 ${st.rollRateMax_dps}°/s，转弯与抗风修正迟缓。`, '增大副翼面积/行程，或适当提高巡航速度。');
    else add('ok', 'roll', '滚转控制权限充足', `满副翼滚转 ${st.rollRateMax_dps}°/s，满足导航转弯需求。`, '');
  }

  // ---------- 导航工况 ----------
  const nv = R.navigation;
  if (nv) {
    if (nv.wind && !nv.wind.feasible) add('warn', 'wind', '无法逆风巡航', `${nv.wind.wind_kmh} km/h 风下逆风地速仅 ${nv.wind.groundHead_kmh} km/h，几乎无法前进。`, '提高巡航速度，或选择风速 < 60% 巡航速度的时间窗飞行。');
    const b45 = nv.turns?.find(t => t.bankDeg === 45);
    if (b45 && !b45.feasible) add('info', 'turn', '45° 坡度转弯有失速风险', `45° 坡度过载 ${b45.nLoad}g，失速速度升至 ${b45.vStallTurn_kmh} km/h，或所需功率超出动力。`, `转弯前加油门，保持 ≥${Math.round(b45.vStallTurn_kmh * 1.2)} km/h。`);
    else if (b45) add('ok', 'turn', '转弯性能正常', `45° 坡度：半径 ${b45.radius_m}m、速率 ${b45.rate_dps}°/s、过载 ${b45.nLoad}g，CL 余量 ${n1(b45.clMargin, 2)}。`, '');
    if (nv.range_km && nv.range_km < 1) add('info', 'range', '转场航程很短', `估算最远转场仅 ${nv.range_km} km。`, '按最大升阻比速度飞行、减重或加大电池。');
  }

  // ---------- 悬停效率优化建议 ----------
  const opt = R.optimize;
  if (opt && opt.baseline && opt.suggestions.length) {
    const top = opt.suggestions[0];
    add('info', 'optimize', '存在更省电的悬停配置',
      `基准 ${opt.baseline.gPerW} g/W；最优改进「${top.changeLabel}」→ ${top.gPerW} g/W（${top.d_pct > 0 ? '+' : ''}${top.d_pct}%），${top.gains.slice(0, 2).join('、')}。`,
      '详见报告"悬停效率优化建议"一节，按收益与代价（推重比/续航/重量）权衡选择。');
  }

  if (R.battery.enduranceHoverMin !== null && R.battery.enduranceHoverMin < td.enduranceMin_min)
    add('warn', 'time', '续航过短', `满油门/悬停估算续航仅 ${n1(R.battery.enduranceHoverMin, 1)} 分钟。`, '加大电池容量（注意增重）、提高悬停效率。');

  if (!out.some(o => o.level === 'error')) add('ok', 'noshowstop', '无致命配置问题', '当前配置可以进入首飞阶段。', '');

  // 关联 solutionRef（若 solveForTargets 已在本次运行中先执行）
  linkSolutionRefs(out, R.solutions);

  return out;
}

// ---------------------------- issue → 方案 关联 ----------------------------
// diagnose() 在 R.solutions 生成之前执行，因此 simulate() 求解完成后需再调用一次本函数。
export function linkSolutionRefs(issues, solutions) {
  if (!solutions?.solutions?.length) return issues;
  const solByTarget = {};
  for (const s of solutions.solutions) {
    if (s.targetId && !solByTarget[s.targetId]) solByTarget[s.targetId] = s;
  }
  const codeToTargetIds = {
    'target': ['target_speed'],
    'twr': ['twr'],
    'hover': ['hover_duty', 'hover_duty_low'],
    'time': ['endurance'],
    'arm': ['structure'],
    'sm': ['sm'],
    'batteryC': ['battery_c'],
    'motorHeat': ['motor_heat'],
    'tipmach': ['tip_mach'],
    'noise': ['noise'],
    'climb': ['vert_climb'],
  };
  for (const issue of issues) {
    issue.solutionRef = undefined;
    for (const targetId of (codeToTargetIds[issue.code] || [])) {
      if (solByTarget[targetId]) { issue.solutionRef = solByTarget[targetId]; break; }
    }
  }
  return issues;
}

// ---------------------------- 报告 ----------------------------
export function buildReport(R) {
  const secs = [];
  const h = R.cases.hover;
  const mass = R.mass;

  secs.push({
    id: 'overview', title: '总览与动力匹配',
    body: `${R.typeLabel}，起飞重量 ${n1(mass.total_g / 1000, 2)} kg（含 ${R.battery.label} ${n1(mass.battery_g, 0)} g），共 ${R.nMotors} 组动力单元：${R.motor.label} + ${R.prop.label}。`
      + `最大静推力 ${n1(R.cases.maxThrust / 9.80665, 2)} kgf，推重比 ${n1(R.cases.twr, 2)}。`
      + (R.type === 'multirotor'
        ? `悬停需 ${pct(h.duty)} 油门、${n1(h.current, 1)}A、${n1(h.power, 0)}W，单瓦拉起 ${n1(h.gPerW, 2)} g；满油门单电机 ${n1(R.cases.full.thrust / R.nMotors / 9.80665 * 1000, 0)} g、${n1(R.cases.full.current, 1)}A（总）。`
        : R.type === 'vtol'
          ? `双模飞行：悬停需 ${pct(h.duty)} 油门、${n1(h.current, 1)}A、${n1(h.power, 0)}W，单瓦拉起 ${n1(h.gPerW, 2)} g；巡航失速 ${n1(R.cases.vStall, 0)} km/h、最高 ${n1(R.cases.vmax, 0)} km/h、最大升阻比 ${n1(R.cases.maxLD, 1)}，垂直爬升 ${n1(R.cases.vertClimb?.vs_fpm ?? 0, 0)} ft/min。`
          : `巡航最省电点 ${n1(R.cases.bestEndurance?.kmh ?? 0, 0)} km/h，最大升阻比 ${n1(R.cases.maxLD, 1)}。`)
  });

  secs.push({
    id: 'structure', title: '结构与力学',
    body: `机臂按 ${R.structure.armPlate.width_mm}×${R.structure.armPlate.thickness_mm}mm 碳板（E=${R.structure.armPlate.E_GPa}GPa）校核：根部弯矩 ${n1(R.cases.maxThrust / R.nMotors * R.arm_m, 3)} N·m，应力 ${R.structure.armPlate.sigma_MPa} MPa，安全系数 ${R.structure.armPlate.safety}，端部挠度 ${R.structure.armPlate.deflection_mm} mm，一阶固有频率 ${R.structure.armFreq_Hz} Hz。`
      + `桨叶离心应力（30000rpm 峰值）约 ${R.structure.prop.tipStress_MPa} MPa（${R.structure.prop.material}），安全系数 ${R.structure.prop.safety}。`
      + `转动惯量 Ixx≈${R.dynamics.inertia.Ixx} kg·m²，电机座螺钉剪切安全系数 ${R.structure.screw.safety}。`
  });

  secs.push({
    id: 'aero', title: '空气动力',
    body: (R.type === 'multirotor'
      ? `悬停时盘面诱导速度 ${n1(h.vi, 2)} m/s（下洗风速），桨盘载荷 ${n1((R.cases.W / 9.80665 * 1000) / (R.prop.A * 100), 1)} g/dm²，图默曼系数 FM=${n1(h.FM, 2)}（1.0 为理想动量理论）。`
        + `满油门桨尖 ${n1(R.cases.full.vTip, 0)} m/s、马赫 ${n1(R.cases.full.machTip, 2)}。前向飞行时倾角随速度增大，${R.cases.target ? `${R.cases.target.kmh} km/h 需倾角 ${n1(R.cases.target.tiltDeg, 1)}°、克服阻力 ${n1(R.cases.target.drag, 1)}N。` : ''}`
        + `最高平飞 ${n1(R.cases.vmax, 0)} km/h，此时阻力与推力在倾角 ${n1(R.cases.maxTiltDeg, 0)}° 内平衡。`
      : R.type === 'vtol'
        ? `火箭机身 ${n1(R.rocket.bodyLength_m, 2)}m×${R.rocket.bodyD_mm}mm（细长体升力体），尾翼 ${R.rocket.finCount} 片：全展 ${n1(R.vtol.rocketGeom.bfin, 2)}m、总面积 ${n1(R.rocket.finArea_m2, 3)}m²，参考面积 ${n1(R.vtol.rocketGeom.Sref, 3)}m²、展弦比 ${n1(R.vtol.rocketGeom.AR, 1)}。`
          + `巡航失速 ${n1(R.cases.vStall, 0)} km/h（尾翼失速攻角 ${n1(R.vtol.rocketGeom.alphaStallDeg, 0)}°），最大升阻比 ${n1(R.cases.maxLD, 1)} @ ${n1(R.cases.maxLDSpeed, 0)} km/h，最高平飞 ${n1(R.cases.vmax, 0)} km/h。`
          + `悬停时盘面诱导速度 ${n1(h.vi, 2)} m/s、FM=${n1(h.FM, 2)}；垂直上升机身阻力由 ${R.rocket.frontal_m2 * 1e4}cm² 迎风面承担。`
        : `翼展 ${n1(R.cases.wing.span, 2)} m，翼面积 ${n1(R.cases.wingArea, 2)} m²，展弦比 ${n1(R.cases.AR, 1)}；失速速度 ${n1(R.cases.vStall, 0)} km/h。`
          + `最佳升阻比 ${n1(R.cases.maxLD, 1)} @ ${n1(R.cases.maxLDSpeed, 0)} km/h，最小功率速度 ${n1(R.cases.minSinkSpeed, 0)} km/h。`)
  });

  secs.push({
    id: 'fluid', title: '流体与环境',
    body: `空气密度 ρ=${n1(R.air.rho, 3)} kg/m³（海拔 ${R.air.altitude}m / ${R.air.tempC}℃）。桨叶弦长雷诺数 Re≈${R.fluid.reBlade.toExponential(2)}（${R.fluid.regime}），`
      + `机/机身 Re≈${R.fluid.reBody.toExponential(2)}。噪音 ${n1(R.fluid.noise1m, 0)} dB(A)@1m、${n1(R.fluid.noise10m, 0)} dB(A)@10m。`
      + (R.type === 'multirotor'
        ? `离地 0.5D 悬停时地面效应使诱导功率下降约 ${n1(R.fluid.groundEffect.dropPct, 0)}%，起飞阶段会有"吸附感"。`
        : R.type === 'vtol'
          ? `悬停离地 0.5D 地面效应降低诱导功率约 ${n1(R.fluid.groundEffect.dropPct, 0)}%；巡航状态层流-湍流转捩位于弦长 ${R.fluid.transITION.toFixed(0)}% 处，机身/尾翼前缘光洁度决定 CD0。`
          : `层流-湍流转捩位于弦长 ${R.fluid.transITION.toFixed(0)}% 处，机头/翼面前缘保持光洁可降低 CD0。`)
  });

  secs.push({
    id: 'energy', title: '能量链与效率',
    body: `链路：电池 ${n1(R.energy.pBatt, 0)}W → 电调损耗 ${n1(R.energy.pEsc, 0)}W（η=${pct(R.energy.etaEsc)}）→ 电机铜损 ${n1(R.energy.pCopper, 0)}W + 铁损/摩擦 ${n1(R.energy.pIron, 0)}W（η_motor=${pct(R.energy.etaMotor)}）→ 气动 ${n1(R.energy.pMech, 0)}W。`
      + `电池 ${R.battery.label} 存储 ${n1(R.battery.energy_Wh, 1)} Wh，满电 ${n1(R.battery.vFull, 1)}V → 空电 ${n1(R.battery.vCut, 1)}V，内阻 ${n1(R.battery.rPack_mOhm, 1)}mΩ，${n1(R.battery.maxCurrent_A, 0)}A 上限。`
      + (R.battery.enduranceHoverMin ? `估算悬停续航 ${n1(R.battery.enduranceHoverMin, 1)} min。` : '')
  });

  if (R.dynamics.pid) {
    secs.push({
      id: 'control', title: '飞控与稳定性',
      body: `执行机构（电机+桨）机械时间常数 τ=${n1(R.dynamics.actuator.tau_ms, 1)} ms，电气 τ=${n1(R.dynamics.actuator.elecTau_ms, 2)} ms，理论速率环带宽约 ${n1(R.dynamics.actuator.bandwidth_Hz, 1)} Hz，桨叶通过频率 ${R.dynamics.actuator.passing_Hz} Hz。`
        + `基于惯量 Ixx=${R.dynamics.inertia.Ixx} kg·m² 与执行器增益 ${R.dynamics.pid.gain.K} rad/s²/duty，推荐速率环 P=${n1(R.dynamics.pid.pid.P, 3)} / I=${n1(R.dynamics.pid.pid.I, 4)} / D=${n1(R.dynamics.pid.pid.D, 3)}（0~255 量纲：${R.dynamics.pid.pid.P_255} / ${R.dynamics.pid.pid.I_255} / ${R.dynamics.pid.pid.D_255}），D 项低通截止 ${n1(1000 / (2 * Math.PI * R.dynamics.pid.pid.dFilter_ms), 1)}Hz（时间常数 ${n1(R.dynamics.pid.pid.dFilter_ms, 1)}ms）。`
        + `阶跃响应：超调 ${n1(R.dynamics.pid.response.overshoot, 1)}%，上升时间 ${n1(R.dynamics.pid.response.rise_ms, 0)} ms，调节时间 ${n1(R.dynamics.pid.response.settling_ms, 0)} ms。`
    });
  }

  // ---------- 垂起火箭：转换走廊 & 悬停稳定 ----------
  if (R.type === 'vtol' && R.vtol) {
    const tr = R.vtol.transition, hs = R.vtol.hoverStability;
    if (tr) {
      const unsafeTxt = tr.feasible ? '全部俯仰角可达' : `${tr.unsafeCount}/${tr.rows.length} 个俯仰角硬性失败`;
      secs.push({
        id: 'transition', title: '转换走廊（垂直→巡航）',
        body: `机身俯仰 90°→0° 共 ${tr.rows.length} 个采样点：${unsafeTxt}，最小推力裕度 ${tr.minMargin_pct}%，转换段最小差动角加速度 ${tr.minControlAlpha} rad/s²。`
          + `推荐转换速度 ${tr.recommendedV_kmh ?? '—'} km/h${tr.recommendedPhi != null ? `（机头下压至约 ${tr.recommendedPhi}° 时裕度最大）` : ''}，进入转换 ${tr.vEntry_kmh} km/h、完成转换 ${tr.vExit_kmh} km/h。`
          + (tr.cautionCount ? `其中 ${tr.cautionCount} 个采样点处于尾翼失速过渡带（${tr.cautionBand_deg?.[0]}°→${tr.cautionBand_deg?.[1]}°），机翼失速、仅靠差动推力控姿，应保持大推力快速穿越。` : '')
          + (hs ? `悬停姿态：重心位于推力面${hs.inverted ? '之上（倒立摆，需飞控持续修正）' : `下方 ${hs.cgBelowThrust_mm}mm（单摆 ${hs.pendulum_Hz}Hz）`}，差动横滚权限 ${hs.rollAlpha} rad/s²、偏航 ${hs.yawAlpha} rad/s²。` : '')
      });
    }
  }

  // ---------- 固定翼 / 火箭：增稳 & 导航 ----------
  if (R.stability) {
    const st = R.stability;
    secs.push({
      id: 'stability', title: '静稳定与增稳（固定翼模态）',
      body: `平均气动弦 c̄=${n1(st.cbar_m * 1000, 0)}mm，重心位于 ${st.xCG_pct}% c̄、中性点 ${st.xNP_pct}% c̄ → 静稳定裕度 SM=${st.SM_pct}%（目标 5%~15%）。`
        + `升力线斜率 ${st.clAlpha}/rad（Re=${st.reRef.toExponential(1)} 修正后）。`
        + `短周期 ${st.wn_sp_Hz}Hz、阻尼比 ${st.zeta_sp}；长周期（浮沉）约 ${st.phugoid_s}s。`
        + `舵面配平权限系数 ${st.elevatorAuth.toFixed(2)}（<1 表示压不过静稳定力矩）；满副翼滚转 ${st.rollRateMax_dps}°/s；航向静稳定导数 Cnβ≈${st.Cnbeta}、偏航固有频率 ${st.yawAuth_Hz}Hz。`
        + (st.needsAugStab ? ' 当前裕度需飞控增稳介入（或重心/舵面调整）才能稳定飞行。' : ' 可由气动布局自稳定，飞控仅需阻尼增稳。')
    });
  }
  if (R.navigation) {
    const nv = R.navigation;
    const t45 = nv.turns.find(t => t.bankDeg === 45);
    secs.push({
      id: 'navigation', title: '导航工况（巡航/转弯/抗风）',
      body: `巡航参考速度 ${nv.Vcruise_kmh} km/h（L/D ${nv.cruise.L_D}、CL=${nv.cruise.CL}、功率 ${n1(nv.cruise.power_W, 0)}W）。`
        + `转弯性能：20°/30°/45° 坡度半径 ${nv.turns.map(t => `${t.radius_m}m`).join(' / ')}，转弯速率 ${nv.turns.map(t => `${t.rate_dps}°/s`).join(' / ')}（45° 时过载 ${t45 ? t45.nLoad : '—'}g、失速速度升至 ${t45 ? t45.vStallTurn_kmh : '—'} km/h）。`
        + `抗风：${nv.wind.wind_kmh} km/h 风下逆风地速 ${nv.wind.groundHead_kmh} km/h、顺风 ${nv.wind.groundTail_kmh} km/h，等时往返平均 ${nv.wind.avgRoundTrip_kmh} km/h，偏航修正角 ${nv.wind.driftAngle_deg}°。`
        + `盘旋（20° 坡度）半径 ${nv.loiter.radius_m}m @ ${nv.loiter.V_kmh} km/h${nv.loiter.endurance_min != null ? `、续航约 ${nv.loiter.endurance_min}min` : ''}；巡航续航约 ${nv.cruise.endurance_min ?? '—'}min，转场航程参考 ${nv.range_km ?? '—'} km（按可用电量 ${n1(nv.usableEnergy_Wh ?? 0, 0)}Wh 估算）。`
    });
  }

  // ---------- 悬停效率优化建议 ----------
  if (R.optimize && R.optimize.baseline) {
    const o = R.optimize;
    const top = o.suggestions[0];
    secs.push({
      id: 'optimize', title: '悬停效率优化建议',
      body: `基准配置 ${o.baseline.gPerW} g/W @ ${pct(o.baseline.duty)} 油门、${o.baseline.current}A、续航 ${o.baseline.endurance_min}min（扫描 ${o.scanned} 个工作点${o.maxProp_mm ? `，桨径上限 ${Math.round(o.maxProp_mm)}mm` : ''}）。`
        + (top
          ? `最优改进「${top.changeLabel}」→ ${top.gPerW} g/W（${top.d_pct > 0 ? '+' : ''}${top.d_pct}%）：${top.gains.join('、')}${top.costs.length ? `；代价：${top.costs.join('、')}` : ''}。`
          : (o.note || '当前配置已处于该约束空间的高效区。'))
        + (o.suggestions.length > 1 ? ` 其余候选：${o.suggestions.slice(1, 3).map(s => `${s.changeLabel}（${s.d_pct > 0 ? '+' : ''}${s.d_pct}%）`).join('；')}。` : '')
    });
  }

  // ---------- 改造方案建议（目标驱动反向求解） ----------
  if (R.solutions?.solutions?.length) {
    const sols = R.solutions.solutions;
    const reached = sols.filter(s => s.reached);
    const bodyParts = [
      `共生成 ${sols.length} 个改造方案，其中 ${reached.length} 个可达标。扫描 ${R.solutions.scanned} 个配置点。`,
    ];
    for (const s of sols.slice(0, 3)) {
      const changes = (s.changes || []).map(c => c.label || `${c.param} ${c.from}→${c.to}`).join('，');
      const afterVals = Object.entries(s.after).map(([k, v]) => `${k}=${v}`).join('、');
      const status = s.pareto ? '[权衡]' : (s.reached ? '✓达标' : '⚠未达标');
      bodyParts.push(`${status} ${s.label}：${changes} → ${afterVals}（裕度 ${s.margin >= 0 ? '+' : ''}${s.margin}%，重量 ${s.massDelta_g >= 0 ? '+' : ''}${s.massDelta_g}g）${s.note ? `；${s.note}` : ''}`);
    }
    if (R.solutions.pareto?.length) {
      bodyParts.push(`速度-续航 Pareto 前沿：${R.solutions.pareto.map(p => `${p.tag}(vmax=${p.after.vmax}km/h,end=${p.after.endurance}min)`).join('；')}`);
    }
    if (sols.length > 3) bodyParts.push(`... 及其余 ${sols.length - 3} 个方案详见"改造方案"标签页。`);
    secs.push({
      id: 'solutions',
      title: '改造方案建议',
      body: bodyParts.join('\n\n'),
    });
  }

  return secs;
}

// ---------------------------- 首飞流程 ----------------------------
export function flightPlan(R) {
  const h = R.cases.hover;
  if (R.type === 'vtol') return vtolPlan(R);
  if (R.type !== 'multirotor') return fixedWingPlan(R);
  const canHover = R.cases.hoverReachable;
  const steps = [
    { phase: '0 · 地面检查', items: [
      `核对桨向与电机转向（${R.nMotors} 电机），螺钉上胶`,
      `电池固定牢靠，重心位于机架几何中心（偏差 <5mm）`,
      `校准加速度计与陀螺，设置失控保护（油门降落）`,
      `设置电调校准、电机进角；确认电流/电压回传`,
    ] },
    { phase: '1 · 首次通电与怠速', items: [
      `不装桨试转：确认电机转向、振动、发热正常`,
      `装桨后低油门（10%）检查振动：机架不应有明显抖动`,
      `设置怠速与 Air Mode 前先确认怠速可靠`,
    ] },
    ...(canHover ? [
      { phase: '2 · 悬停首飞（系留/空旷场地）', items: [
        `目标悬停油门 ${pct(h.duty)}，若 >65% 立即降落换桨/换 KV`,
        `油门快速推至 50% 离地 0.5m，观察偏航与自旋（地效会放大响应）`,
        `保持 1~2m 悬停 30s：观察漂移（需重新校准陀螺/重心）`,
        `确认 ${n1(h.current, 1)}A 电流与回传电压一致（校准电流计）`,
      ] },
    ] : [
      { phase: '2 · 悬停首飞（无法悬停）', items: [
        `当前推重比仅 ${n1(R.cases.twr, 2)}、悬停需 ${pct(h.duty)} 油门，本配置不具备稳定悬停能力`,
        `必须先解决动力问题：换大桨/降 KV/减重/增加电芯数，使悬停油门落在 35%~50%`,
        `在解决前，仅可做系留或短距滑跑测试`,
      ] },
    ]),
    { phase: '3 · 姿态环整定（Betaflight/PX4 通用）', items: [
      `先整定角速度环：P=${n1(R.dynamics.pid?.pid.P ?? 0, 3)} 起步，手动抖动杆量观察跟随`,
      `D 项从 ${n1((R.dynamics.pid?.pid.D ?? 0) * 0.6, 3)} 起加，直到出现高频嗡鸣即回退 20%`,
      `I 项控制长时漂移与外力恢复，过大导致低频晃动`,
      `再整定角度环：P ≈ 角速度环 P × 0.4，观察是否过冲`,
      `阶跃响应目标：超调 <10%，调节时间 <${n1(R.dynamics.pid?.response.settling_ms ?? 100, 0)}ms`,
    ] },
    { phase: '4 · 动态科目', items: [
      `直线加速到 ${R.cases.target?.kmh ?? Math.min(60, Math.round(R.cases.vmax))} km/h，记录电流与油门（应 <70%）`,
      `急停/急转：检查回中是否震荡（D 过大或滤波过低）`,
      `低空低速通场：验证低油门控制权（悬停油门越低，低速越钝）`,
    ] },
    { phase: '5 · 高效巡航调优', items: [
      `用本系统的"效率-速度"曲线找最佳巡航点：${R.cases.bestEndurance ? `${n1(R.cases.bestEndurance.kmh, 0)} km/h 时 ${n1(R.cases.bestEndurance.gPerW, 2)} g/W` : ''}`,
      `长航时按最大续航点飞；转场按 ${R.cases.bestRange ? `${n1(R.cases.bestRange.kmh, 0)} km/h（最大航程速度）` : '最大升阻比点'} 飞`,
      `监控电池：单片压降 >0.1V/cell 即下降功率；空电电压不低于 ${n1(R.battery.vCut / R.battery.cells, 2)}V/cell`,
      `每次飞行记录：悬停油门、悬停电流、温度，作为下次调参基准`,
    ] },
  ];
  return steps;
}

function fixedWingPlan(R) {
  const c = R.cases;
  const st = R.stability, nv = R.navigation;
  return [
    { phase: '0 · 装机检查', items: [
      `翼展 ${n1(c.wing.span, 0)}mm、翼面积 ${n1(c.wingArea, 2)}m²、展弦比 ${n1(c.AR, 1)}，舵面中立校准`,
      `重心位于平均气动弦 25%~33% 处（本机建议 ${n1((c.wing.span * c.wing.span / c.wingArea) * 0.28 * 100, 0)}mm 之后）`,
      st ? `静稳定裕度校验：SM=${st.SM_pct}%（中性点 ${st.xNP_pct}% c̄），重心每前移 1% c̄ SM 约 +1%` : '',
      `确认推力线与机身轴线夹角（上/下倾角），电调/接收机/图传固定`,
    ].filter(Boolean) },
    { phase: '1 · 无动力滑翔（手抛）', items: [
      `先不给动力，高处手抛：应平稳滑翔 5~10m，不坠机头`,
      `左右倾斜用配重或舵面微调修正好后再上动力`,
      `确认失控保护（舵面回中、推力降落）`,
    ] },
    { phase: '2 · 动力首飞', items: [
      `半油门（50%）弹射/手抛起飞，保持 ${n1(c.vStall * 1.3, 0)} km/h 以上（失速速度 ${n1(c.vStall, 0)} km/h 的 1.3 倍）`,
      `爬升到 50~100m 后松杆：检查直线性与滑翔比`,
      `记录平飞电流/功率，与本表 ${c.bestEndurance ? `${n1(c.bestEndurance.power, 0)}W @ ${n1(c.bestEndurance.kmh, 0)}km/h` : ''} 对比`,
    ] },
    { phase: '3 · 自动驾驶/增稳整定', items: [
      `陀螺/加速度校准，罗盘（如有）校准`,
      `固定翼以姿态环为主：先角速度环，再角度环；巡航高度环 P 从小值起`,
      st ? `短周期 ${st.wn_sp_Hz}Hz/阻尼比 ${st.zeta_sp}：俯仰 P 使阻尼比落在 0.3~0.5，过冲则加 D` : '',
      st && st.elevatorAuth < 1 ? `⚠ 舵面配平权限 ${st.elevatorAuth.toFixed(2)}<1：优先增大舵面或后移重心，不要用飞控硬拉` : '',
      st ? `满副翼滚转 ${st.rollRateMax_dps}°/s：滚转不足先加大副翼行程/面积，不要过调 P` : '',
      `油门-高度耦合：油门映射到 ${n1(c.climb?.V ?? 20, 0)} m/s 巡航速度附近`,
      `检查失速保护：接近 ${n1(c.vStall, 0)} km/h 时应有告警/自动推杆`,
    ].filter(Boolean) },
    { phase: '4 · 巡航效率与导航科目', items: [
      `最佳升阻比 ${n1(c.maxLD, 1)} 出现在 ${n1(c.maxLDSpeed, 0)} km/h —— 转场按此速度飞最远`,
      `最省电点 ${n1(c.minSinkSpeed, 0)} km/h —— 长航时按此速度飞`,
      nv ? `转弯科目：45° 坡度半径 ${nv.turns.find(t => t.bankDeg === 45)?.radius_m}m、速率 ${nv.turns.find(t => t.bankDeg === 45)?.rate_dps}°/s，进入前加油门至 ≥1.3×坡度失速速度（${nv.turns.find(t => t.bankDeg === 45)?.vStallTurn_kmh} km/h）` : '',
      nv ? `抗风：${nv.wind.wind_kmh} km/h 风下逆风地速 ${nv.wind.groundHead_kmh} km/h，航线需偏航修正 ${nv.wind.driftAngle_deg}°；逆风地速 <5 km/h 时取消任务` : '',
      nv ? `盘旋/返航：20° 坡度盘旋半径 ${nv.loiter.radius_m}m，转场航程参考 ${nv.range_km} km（含 20% 余量外另行保守）` : '',
      `爬升率 ${n1(c.climb?.vs_fpm ?? 0, 0)} ft/min：留足越障余量，避免在低功率点遇到下降气流`,
    ].filter(Boolean) },
    { phase: '5 · 降落', items: [
      `逆风进近，速度 ${n1(c.vStall * 1.3, 0)} km/h 左右（1.2~1.3 倍失速）`,
      `最后 2m 拉平，收油至怠速滑降`,
    ] },
  ];
}

function vtolPlan(R) {
  const c = R.cases;
  const tr = R.vtol?.transition;
  const hs = R.vtol?.hoverStability;
  const recV = tr?.recommendedV_kmh ?? Math.round(c.vStall * 1.25);
  return [
    { phase: '0 · 装机检查（垂起特有）', items: [
      `机身 ${n1(R.rocket.bodyLength_m, 2)}m×${R.rocket.bodyD_mm}mm，尾翼 ${R.rocket.finCount} 片对称、无松动，螺钉上胶`,
      `重心位于推力面${hs && hs.inverted ? `上方 ${Math.abs(hs.cgBelowThrust_mm)}mm（倒立摆，首飞必须系留）` : `下方 ${hs ? hs.cgBelowThrust_mm : R.rocket.cgBelowThrust_mm}mm（单摆频率 ${hs?.pendulum_Hz ?? '—'}Hz）`}`,
      `校准加速度计/陀螺，设置失控保护（油门降落 / 自动返航）`,
      `确认电机转向与桨向（差动推力是唯一控制来源，单侧失效即失控）`,
    ].filter(Boolean) },
    { phase: '1 · 垂直悬停首飞（系留/空旷）', items: [
      `目标悬停油门 ${pct(c.hover.duty)}，若 >65% 立即降落换桨/换 KV`,
      `油门推至离地 0.5m，观察自旋与漂移：差动横滚权限 ${hs?.rollAlpha ?? '—'} rad/s²、偏航 ${hs?.yawAlpha ?? '—'} rad/s²`,
      `保持 1~2m 悬停 30s：机身摆动周期应接近 ${hs?.pendulum_Hz ?? '—'}Hz；快速晃动 = 姿态环 P 过高`,
      `确认 ${n1(c.hover.current, 1)}A 电流回传一致；垂直爬升 ${n1(c.vertClimb?.vs_fpm ?? 0, 0)} ft/min 能力`,
    ].filter(Boolean) },
    { phase: '2 · 转换首飞（本机型关键科目）', items: [
      `在 ${recV} km/h 附近开转换（走廊 ${tr ? `${tr.vEntry_kmh}→${tr.vExit_kmh}` : '—'} km/h），全程保留 30% 油门余量`,
      `机头下压必须缓慢：每个俯仰段保持推力裕度，最小裕度 ${tr?.minMargin_pct ?? '—'}% 出现在中段`,
      `转换中速度不得低于 ${Math.round(c.vStall * 1.15)} km/h（尾翼失速 ${n1(c.vStall, 0)} km/h 以下无舵效/升力）`,
      `失败处置：立即加大油门+回拉机头退回垂直悬停，不要继续前推`,
    ].filter(Boolean) },
    { phase: '3 · 巡航与导航', items: [
      `巡航最省电 ${n1(c.minSinkSpeed ?? 0, 0)} km/h，最大升阻比 ${n1(c.maxLD, 1)} @ ${n1(c.maxLDSpeed ?? 0, 0)} km/h`,
      `转弯：45° 坡度半径 ${R.navigation?.turns.find(t => t.bankDeg === 45)?.radius_m ?? '—'}m，进弯加油门`,
      `抗风：${R.navigation?.wind.wind_kmh ?? '—'} km/h 风下逆风地速 ${R.navigation?.wind.groundHead_kmh ?? '—'} km/h`,
      `目标 ${c.target?.kmh ?? '—'} km/h 巡航油门 ${c.target ? pct(c.target.duty) : '—'}，长期保持建议 ≤70% 油门`,
    ].filter(Boolean) },
    { phase: '4 · 垂直降落', items: [
      `返场后减速到 ${Math.round(c.vStall * 1.2)} km/h 以下再拉起机头（先减速后转姿，顺序不可反）`,
      `垂直段用 ${pct(c.hover.duty)} 附近油门缓降，注意地面效应吸附`,
      `触地前 0.5m 保持油门稳定，切忌高油门砸地`,
    ].filter(Boolean) },
    { phase: '5 · 调优记录', items: [
      `记录悬停油门/电流/温度、转换速度窗口、巡航电流，作为下次基准`,
      R.optimize?.suggestions?.length ? `效率优化：可试「${R.optimize.suggestions[0].changeLabel}」（悬停效率 ${R.optimize.baseline.gPerW}→${R.optimize.suggestions[0].gPerW} g/W，${R.optimize.suggestions[0].d_pct > 0 ? '+' : ''}${R.optimize.suggestions[0].d_pct}%）` : '当前悬停配置已接近该约束空间的高效区',
    ].filter(Boolean) },
  ];
}
