// 扶摇 FuyaoTune — 传感器体检模块
//   SYS_STATUS 位掩码 (陀螺/加计/磁/气压/GPS/光流…) + 振动/EKF/IMU/气压限时采样
//   体检结论矩阵 + 图表 + PreArm 聚合 + 可复制报告
// 数据仅存浏览器内存; 体检期间临时提升消息速率, 结束自动恢复默认。
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);

  // ==================== 判定阈值 (导出供单测) ====================
  const THRESHOLDS = {
    vibOk: 15, vibWarn: 30,            // m/s² (ArduPilot 建议 < 30)
    ekfVelOk: 0.5, ekfVelWarn: 2.0,
    ekfPosHOk: 4, ekfPosHWarn: 25,
    ekfPosVOk: 4, ekfPosVWarn: 25,
    ekfMagOk: 0.2, ekfMagWarn: 0.6,
    imuTempMin: 0, imuTempMax: 70,     // °C 正常工作区
    baroTempMin: -40, baroTempMax: 85,
    baroPressMin: 300, baroPressMax: 1100, // hPa
    gpsSatsGood: 10, gpsEphGoodCm: 250,
  };

  // MAV_SYS_STATUS_SENSOR 位 (主掩码 32 位 + 扩展掩码 32 位, 扩展位用 ext 标记)
  // 参见 MAVLink common.xml: 主掩码 bit0-31, 扩展掩码 bit32-63 (ext.bit 为扩展字段内位偏移)
  const SENSOR_BITS = [
    { bit: 1, id: 'gyro', name: '陀螺仪' },
    { bit: 2, id: 'accel', name: '加速度计' },
    { bit: 4, id: 'mag', name: '磁力计' },
    { bit: 8, id: 'baro', name: '气压计' },
    { bit: 16, id: 'diffpress', name: '空速计' },
    { bit: 32, id: 'gps', name: 'GPS' },
    { bit: 64, id: 'flow', name: '光流' },
    { bit: 128, id: 'vision', name: '视觉定位' },
    { bit: 256, id: 'laser', name: '激光测距' },
    { bit: 65536, id: 'rc', name: '遥控接收' },
    { bit: 131072, id: 'gyro2', name: '副陀螺' },
    { bit: 262144, id: 'accel2', name: '副加计' },
    { bit: 524288, id: 'mag2', name: '副磁计' },
    { bit: 33554432, id: 'battery', name: '电池监测' },
    { bit: 67108864, id: 'proximity', name: '接近传感器' },
    { bit: 134217728, id: 'satcom', name: '卫星通信' },
    // 扩展位 (present_extended/enabled_extended/health_extended; bit 为扩展字段内位)
    { bit: 1, ext: true, id: 'gyro3', name: '第3陀螺' },
    { bit: 2, ext: true, id: 'accel3', name: '第3加计' },
    { bit: 4, ext: true, id: 'mag3', name: '第3磁计' },
  ];

  // EKF_STATUS_FLAGS
  const EKF_FLAG_NAMES = {
    1: '姿态', 2: '水平速度', 4: '垂直速度', 8: '相对水平位置',
    16: '绝对水平位置', 32: '绝对垂直位置', 64: '预测相对位置', 128: '预测绝对位置',
  };

  const STATUS_LABEL = { ok: '正常', warn: '警告', err: '故障', na: '无数据' };
  const STATUS_ICON = { ok: '✅', warn: '⚠️', err: '❌', na: '⚪' };

  // ==================== 纯计算 (导出供单测) ====================
  function mean(arr) {
    if (!arr || !arr.length) return null;
    let s = 0;
    for (const v of arr) s += v;
    return s / arr.length;
  }

  function stddev(arr) {
    if (!arr || arr.length < 2) return 0;
    const m = mean(arr);
    let s = 0;
    for (const v of arr) s += (v - m) * (v - m);
    return Math.sqrt(s / (arr.length - 1));
  }

  function worst(a, b) {
    const rank = { ok: 0, na: 1, warn: 2, err: 3 };
    return rank[a] >= rank[b] ? a : b;
  }

  /**
   * 传感器体检评估 (纯函数, 无 DOM)。
   * @param {object} input
   *   sysStatus  {sensorsPresent, sensorsEnabled, sensorsHealth, ...Ext} | null
   *   vibration  [{vibX,vibY,vibZ,clipping}] 采样
   *   ekf        [{velocityVariance,posHorizVariance,posVertVariance,compassVariance,flags}]
   *   imu        [{ax,ay,az,gx,gy,gz,temperature}]
   *   baro       [{pressAbs,temperature}]
   *   gps        {fixType,satellites,eph} | null   (eph 单位 cm)
   *   prearm     [text,...]
   * @returns {{sensors:Array, verdict:string, prearm:Array, stats:object}}
   */
  function evaluateSensors(input) {
    const T = THRESHOLDS;
    const sensors = [];
    const stats = {};
    const sys = input.sysStatus || null;

    // ---- 位掩码传感器 ----
    for (const s of SENSOR_BITS) {
      if (!sys) {
        sensors.push({ id: s.id, name: s.name, status: 'na', value: '--', detail: '未收到 SYS_STATUS' });
        continue;
      }
      // ext 位从 *_extended 掩码取; 主位从基础掩码取
      const presentMask = s.ext ? (sys.sensorsPresentExt || 0) : sys.sensorsPresent;
      const enabledMask = s.ext ? (sys.sensorsEnabledExt || 0) : sys.sensorsEnabled;
      const healthMask = s.ext ? (sys.sensorsHealthExt || 0) : sys.sensorsHealth;
      const present = (presentMask & s.bit) !== 0;
      const enabled = (enabledMask & s.bit) !== 0;
      const healthy = (healthMask & s.bit) !== 0;
      if (!present) {
        sensors.push({ id: s.id, name: s.name, status: 'na', value: '未装', detail: 'SYS_STATUS 未置 present' });
      } else if (!healthy) {
        sensors.push({ id: s.id, name: s.name, status: 'err', value: '不健康', detail: enabled ? 'present 但 health=0' : 'present 但未使能且不健康' });
      } else {
        sensors.push({ id: s.id, name: s.name, status: 'ok', value: '健康', detail: enabled ? 'present+enabled+health' : 'present+health' });
      }
    }

    // ---- 振动 ----
    const vib = input.vibration || [];
    let vibStatus = 'na';
    let vibValue = '无数据';
    let vibDetail = '未收到 VIBRATION 消息 (该消息依赖 ArduPilot)';
    if (vib.length) {
      const xs = vib.map((v) => v.vibX);
      const ys = vib.map((v) => v.vibY);
      const zs = vib.map((v) => v.vibZ);
      const mx = Math.max(mean(xs), mean(ys), mean(zs));
      const peak = Math.max(Math.max(...xs), Math.max(...ys), Math.max(...zs));
      const clip = vib.reduce((a, v) => a + (v.clipping ? v.clipping[0] + v.clipping[1] + v.clipping[2] : 0), 0);
      stats.vibMean = mx;
      stats.vibPeak = peak;
      stats.clipping = clip;
      vibValue = `均值 ${mx.toFixed(1)} / 峰值 ${peak.toFixed(1)} m/s²`;
      vibDetail = `裁剪计数 ${clip}; 样本 ${vib.length}`;
      vibStatus = clip > 0 ? 'warn' : (peak >= T.vibWarn ? 'err' : (mx >= T.vibOk ? 'warn' : 'ok'));
      if (clip > 0) vibDetail += ' (加速度计饱和, 检查减振)';
    }
    sensors.push({ id: 'vibration', name: '振动', status: vibStatus, value: vibValue, detail: vibDetail });

    // ---- EKF ----
    const ekf = input.ekf || [];
    let ekfStatus = 'na';
    let ekfValue = '无数据';
    let ekfDetail = '未收到 EKF_STATUS_REPORT';
    if (ekf.length) {
      const last = ekf[ekf.length - 1];
      const flags = last.flags || 0;
      const active = [];
      for (const [bit, nm] of Object.entries(EKF_FLAG_NAMES)) {
        if (flags & Number(bit)) active.push(nm);
      }
      stats.ekfFlags = flags;
      stats.ekfVel = last.velocityVariance;
      stats.ekfPosH = last.posHorizVariance;
      stats.ekfPosV = last.posVertVariance;
      stats.ekfMag = last.compassVariance;
      ekfValue = `速度 ${last.velocityVariance.toFixed(2)} / 水平 ${last.posHorizVariance.toFixed(2)} / 罗盘 ${last.compassVariance.toFixed(2)}`;
      ekfDetail = `融合中: ${active.length ? active.join('、') : '无'}; 样本 ${ekf.length}`;
      let st = 'ok';
      if (last.velocityVariance >= T.ekfVelWarn || last.posHorizVariance >= T.ekfPosHWarn
        || last.compassVariance >= T.ekfMagWarn) st = 'err';
      else if (last.velocityVariance >= T.ekfVelOk || last.posHorizVariance >= T.ekfPosHOk
        || last.compassVariance >= T.ekfMagOk) st = 'warn';
      // 基本融合标志缺失 → 至少警告
      if (!(flags & 1) || !(flags & 2)) st = worst(st, 'warn');
      ekfStatus = st;
    }
    sensors.push({ id: 'ekf', name: 'EKF 融合', status: ekfStatus, value: ekfValue, detail: ekfDetail });

    // ---- IMU 温度 / 卡死 ----
    const imu = input.imu || [];
    let imuStatus = 'na';
    let imuValue = '无数据';
    let imuDetail = '未收到 SCALED_IMU2';
    if (imu.length) {
      const temps = imu.map((s) => s.temperature).filter((v) => isFinite(v) && v !== 0);
      const temp = temps.length ? mean(temps) : null;
      const axStd = stddev(imu.map((s) => s.ax));
      const ayStd = stddev(imu.map((s) => s.ay));
      const azStd = stddev(imu.map((s) => s.az));
      // 三轴全部冻结才算卡死: 静止台架上单轴因量化可能恒定, 不能误判
      const stuck = imu.length >= 20 && axStd < 0.001 && ayStd < 0.001 && azStd < 0.001;
      stats.imuTemp = temp;
      stats.imuNoise = Math.max(axStd, ayStd, azStd);
      imuValue = temp !== null ? `温度 ${temp.toFixed(1)}°C` : '温度未知';
      imuDetail = `加计噪声σ ${Math.max(axStd, ayStd, azStd).toFixed(3)} m/s²; 样本 ${imu.length}`;
      let st = 'ok';
      if (stuck) st = 'err';
      if (temp !== null && (temp < T.imuTempMin || temp > T.imuTempMax)) st = worst(st, 'warn');
      imuStatus = st;
      if (st === 'err') imuDetail += ' (疑似卡死: 加计读数无变化)';
    }
    sensors.push({ id: 'imu', name: 'IMU 温度/卡死', status: imuStatus, value: imuValue, detail: imuDetail });

    // ---- 气压读数合理性 ----
    const baro = input.baro || [];
    let baroStatus = 'na';
    let baroValue = '无数据';
    let baroDetail = '未收到 SCALED_PRESSURE';
    if (baro.length) {
      const last = baro[baro.length - 1];
      stats.baroPress = last.pressAbs;
      stats.baroTemp = last.temperature;
      baroValue = `${last.pressAbs.toFixed(1)} hPa / ${last.temperature.toFixed(1)}°C`;
      baroDetail = `样本 ${baro.length}`;
      let st = 'ok';
      if (last.pressAbs < T.baroPressMin || last.pressAbs > T.baroPressMax) st = worst(st, 'warn');
      if (last.temperature < T.baroTempMin || last.temperature > T.baroTempMax) st = worst(st, 'warn');
      baroStatus = st;
    }
    sensors.push({ id: 'baro_read', name: '气压读数', status: baroStatus, value: baroValue, detail: baroDetail });

    // ---- GPS 数据质量 (与位掩码 gps 卡分开; 位掩码看健康, 这里看 fix/星数/精度) ----
    const gps = input.gps || null;
    let gpsStatus = 'na';
    let gpsValue = '无数据';
    let gpsDetail = '未收到 GPS_RAW_INT';
    if (gps) {
      const fix = gps.fixType || 0;
      const sats = gps.satellites || 0;
      const eph = gps.eph; // cm
      gpsValue = `fix=${fix} 星=${sats}` + (eph ? ` EPH=${(eph / 100).toFixed(1)}m` : '');
      gpsDetail = fix >= 3 ? '3D 及以上定位' : (fix ? '定位级不足' : '未定位');
      let st = 'ok';
      if (fix < 2) st = 'err';
      else if (fix < 3) st = 'warn';
      if (fix >= 3 && sats < T.gpsSatsGood) st = worst(st, 'warn');
      if (eph && eph > T.gpsEphGoodCm * 2) st = worst(st, 'warn');
      gpsStatus = st;
    }
    sensors.push({ id: 'gps_qual', name: 'GPS 数据', status: gpsStatus, value: gpsValue, detail: gpsDetail });

    // ---- PreArm ----
    const prearm = (input.prearm || []).map((t) => String(t).replace(/^PreArm:\s*/i, '').trim()).filter(Boolean);
    const prearmStatus = prearm.length ? 'err' : (sys ? 'ok' : 'na');
    sensors.push({
      id: 'prearm', name: '预解锁检查',
      status: prearmStatus,
      value: prearm.length ? `${prearm.length} 项失败` : (sys ? '无失败项' : '无数据'),
      detail: prearm.length ? prearm.join('; ') : (sys ? '采样窗口内未见 PreArm 失败消息' : '未连接'),
    });

    // ---- 总评 ----
    let verdict = 'ok';
    for (const s of sensors) verdict = worst(verdict, s.status);
    // na 不拉低总评: 未装的传感器不该让整体变警告
    const real = sensors.filter((s) => s.status !== 'na');
    verdict = 'ok';
    for (const s of real) verdict = worst(verdict, s.status);

    return { sensors, verdict, prearm, stats };
  }

  function buildReport(result, meta) {
    const lines = [];
    lines.push(`传感器体检报告 ${new Date().toLocaleString('zh-CN')}`);
    if (meta) {
      lines.push(`飞控: ${meta.firmware || '未知'}  采样时长: ${Math.round((meta.durationMs || 0) / 1000)}s`
        + (meta.sampled ? `  样本: 振动${meta.sampled.vib || 0}/EKF${meta.sampled.ekf || 0}/IMU${meta.sampled.imu || 0}/气压${meta.sampled.baro || 0}` : ''));
    }
    const vLabel = { ok: '✅ 通过', warn: '⚠ 有警告', err: '❌ 有故障', na: '⚪ 数据不足' };
    lines.push(`结论: ${vLabel[result.verdict] || result.verdict}`);
    lines.push('');
    lines.push('传感器矩阵:');
    for (const s of result.sensors) {
      lines.push(`  ${STATUS_ICON[s.status] || '⚪'} ${s.name.padEnd(10, '　')} ${STATUS_LABEL[s.status] || s.status}  ${s.value}`);
      if (s.detail && s.status !== 'ok') lines.push(`      ${s.detail}`);
    }
    if (result.prearm.length) {
      lines.push('');
      lines.push('预解锁失败项:');
      for (const p of result.prearm) lines.push(`  - ${p}`);
    }
    return lines.join('\n');
  }

  // ==================== 运行时状态 ====================
  const state = {
    view: 'sensors',
    connected: false,
    firmware: 'apm',
    running: false,
    durationMs: 30000,
    startedAt: 0,
    timer: null,
    buffers: { vibration: [], ekf: [], imu: [], baro: [], prearm: [] },
    lastSys: null,
    lastGps: null,
    result: null,
  };

  const BUF_MAX = 3000;

  function pushBuf(key, item) {
    const a = state.buffers[key];
    a.push(item);
    if (a.length > BUF_MAX) a.splice(0, a.length - BUF_MAX);
  }

  function clearBuffers() {
    state.buffers = { vibration: [], ekf: [], imu: [], baro: [], prearm: [] };
  }

  function elapsedMs() {
    return state.running ? Date.now() - state.startedAt : 0;
  }

  // ==================== 消息入口 ====================
  function onMsg(msg) {
    switch (msg.type) {
      case 'connected':
        state.connected = true;
        renderStatus();
        break;
      case 'disconnected':
        state.connected = false;
        if (state.running) stopCheck(true);
        renderStatus();
        break;
      case 'heartbeat':
        state.firmware = msg.firmware || 'apm';
        break;
      case 'sys_status':
        state.lastSys = msg;
        if (state.view === 'dashboard' || state.view === 'sensors') renderBadge();
        if (state.view === 'charts') renderLiveSensors();
        break;
      case 'gps_status':
        state.lastGps = { fixType: msg.fixType, satellites: msg.satellites, eph: msg.eph };
        if (state.view === 'charts') renderLiveSensors();
        break;
      case 'vibration':
        if (state.running) {
          pushBuf('vibration', { vibX: msg.vibX, vibY: msg.vibY, vibZ: msg.vibZ, clipping: msg.clipping });
          renderLiveCounts();
        }
        break;
      case 'ekf_status':
        if (state.running) {
          pushBuf('ekf', {
            velocityVariance: msg.velocityVariance,
            posHorizVariance: msg.posHorizVariance,
            posVertVariance: msg.posVertVariance,
            compassVariance: msg.compassVariance,
            flags: msg.flags,
          });
          renderLiveCounts();
        }
        break;
      case 'imu_sample':
        if (state.running) {
          pushBuf('imu', { ax: msg.ax, ay: msg.ay, az: msg.az, temperature: msg.temperature });
          renderLiveCounts();
        }
        break;
      case 'scaled_pressure':
        if (state.running) {
          pushBuf('baro', { pressAbs: msg.pressAbs, temperature: msg.temperature });
          renderLiveCounts();
        }
        break;
      case 'statustext':
        if (state.running && /^PreArm:/i.test(msg.text || '')) {
          pushBuf('prearm', msg.text);
          renderLiveCounts();
        }
        break;
      case 'sensor_check_started':
        renderStatus();
        break;
      case 'sensor_check_stopped':
        renderStatus();
        break;
      default:
        break;
    }
  }

  // ==================== 体检控制 ====================
  function startCheck() {
    if (state.running || !state.connected) return;
    clearBuffers();
    state.result = null;
    state.startedAt = Date.now();
    state.running = true;
    const durSel = $('scDuration');
    state.durationMs = (durSel && Number(durSel.value)) || 30000;
    try { window.ws.send(JSON.stringify({ type: 'sensor_check_start' })); } catch (e) { /* 串口未就绪 */ }
    state.timer = setTimeout(() => finishCheck(), state.durationMs + 500); // +500 等最后几帧
    renderStatus();
    renderReportArea();
    renderCharts();
  }

  function stopCheck(silent) {
    if (!state.running) return;
    state.running = false;
    if (state.timer) { clearTimeout(state.timer); state.timer = null; }
    try { window.ws.send(JSON.stringify({ type: 'sensor_check_stop' })); } catch (e) { /* ignore */ }
    if (!silent) finishCheck();
    else renderStatus();
  }

  function finishCheck() {
    if (!state.running && !state.startedAt) return;
    state.running = false;
    if (state.timer) { clearTimeout(state.timer); state.timer = null; }
    try { window.ws.send(JSON.stringify({ type: 'sensor_check_stop' })); } catch (e) { /* ignore */ }
    const durationMs = Math.min(elapsedMs() || state.durationMs, state.durationMs + 1000);
    state.result = evaluateSensors({
      sysStatus: state.lastSys,
      vibration: state.buffers.vibration,
      ekf: state.buffers.ekf,
      imu: state.buffers.imu,
      baro: state.buffers.baro,
      gps: state.lastGps,
      prearm: state.buffers.prearm,
    });
    state.lastDurationMs = durationMs;
    renderStatus();
    renderCards();
    renderCharts();
    renderPrearmList();
    renderReportArea();
    state.startedAt = 0;
  }

  // ==================== 渲染 ====================
  function setText(id, text) {
    const el = $(id);
    if (el && el.textContent !== text) el.textContent = text;
  }

  function renderStatus() {
    const el = $('scSummary');
    if (!el) return;
    let text;
    let cls = 'sensor-summary';
    if (!state.connected) { text = '未连接'; }
    else if (state.running) {
      const left = Math.max(0, Math.ceil((state.durationMs - elapsedMs()) / 1000));
      text = `采样中… 剩余 ${left}s`;
      cls += ' running';
    } else if (state.result) {
      const v = { ok: '✅ 体检通过', warn: '⚠ 有警告', err: '❌ 有故障', na: '⚪ 数据不足' };
      text = v[state.result.verdict] || state.result.verdict;
      cls += ' ' + state.result.verdict;
    } else {
      text = '待检';
    }
    if (el.textContent !== text) { el.textContent = text; }
    el.className = cls;
    const startBtn = $('btnScStart');
    const stopBtn = $('btnScStop');
    if (startBtn) startBtn.disabled = !state.connected || state.running;
    if (stopBtn) stopBtn.disabled = !state.running;
  }

  function renderLiveCounts() {
    setText('scLiveCounts',
      `振动 ${state.buffers.vibration.length} · EKF ${state.buffers.ekf.length}`
      + ` · IMU ${state.buffers.imu.length} · 气压 ${state.buffers.baro.length}`
      + (state.buffers.prearm.length ? ` · PreArm失败 ${state.buffers.prearm.length}` : ''));
  }

  function renderCards() {
    const box = $('scCards');
    if (!box) return;
    if (!state.result) {
      box.innerHTML = '<div class="sensor-cards-empty">点击「开始体检」后, 这里显示各传感器健康矩阵。</div>';
      return;
    }
    const html = state.result.sensors.map((s) => `
      <div class="sensor-card ${s.status}">
        <div class="sensor-card-head">
          <span class="sensor-card-name">${STATUS_ICON[s.status]} ${s.name}</span>
          <span class="sensor-card-status">${STATUS_LABEL[s.status] || s.status}</span>
        </div>
        <div class="sensor-card-value">${s.value}</div>
        <div class="sensor-card-detail">${s.detail}</div>
      </div>`).join('');
    box.innerHTML = html;
  }

  function renderPrearmList() {
    const box = $('scPrearmList');
    if (!box) return;
    if (!state.result) { box.innerHTML = ''; return; }
    if (!state.result.prearm.length) {
      box.innerHTML = '<div class="sensor-prearm-ok">采样窗口内未捕获到 PreArm 失败消息。</div>';
      return;
    }
    box.innerHTML = state.result.prearm.map((p) => `<div class="sensor-prearm-item">❌ ${p}</div>`).join('');
  }

  function renderReportArea() {
    const pre = $('scReport');
    if (!pre) return;
    if (state.running) {
      pre.textContent = '采样进行中… 结束后自动生成报告。';
      return;
    }
    if (!state.result) {
      pre.textContent = '点「开始体检」后自动采样并生成报告。';
      return;
    }
    pre.textContent = buildReport(state.result, {
      firmware: state.firmware === 'px4' ? 'PX4' : 'ArduPilot',
      durationMs: state.lastDurationMs || state.durationMs,
      sampled: {
        vib: state.buffers.vibration.length,
        ekf: state.buffers.ekf.length,
        imu: state.buffers.imu.length,
        baro: state.buffers.baro.length,
      },
    });
  }

  // ==================== 图表 ====================
  const CHART_DEFS = [
    {
      canvas: 'scVibChart',
      series: [
        { key: 'x', label: 'X', color: '#3B82F6', get: (s) => s.vibX },
        { key: 'y', label: 'Y', color: '#22C55E', get: (s) => s.vibY },
        { key: 'z', label: 'Z', color: '#F59E0B', get: (s) => s.vibZ },
      ],
      unit: 'm/s²',
    },
    {
      canvas: 'scEkfChart',
      series: [
        { key: 'vel', label: '速度方差', color: '#3B82F6', get: (s) => s.velocityVariance },
        { key: 'posH', label: '水平位置', color: '#22C55E', get: (s) => s.posHorizVariance },
        { key: 'mag', label: '罗盘', color: '#EF4444', get: (s) => s.compassVariance },
      ],
      unit: '',
    },
  ];

  let themeVars = {};
  function cvar(name, fb) {
    if (name in themeVars) return themeVars[name];
    let v = fb;
    try { v = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb; } catch (e) { /* 读取失败走兜底 */ }
    themeVars[name] = v;
    return v;
  }
  // 系列颜色随主题: ser.color 为夜间色, 换算 --chart-N
  const SER_CHART_N = { '#3B82F6': 1, '#22C55E': 2, '#EF4444': 3, '#F59E0B': 4 };
  function serColor(ser) {
    const n = SER_CHART_N[ser.color];
    return n ? cvar('--chart-' + n, ser.color) : ser.color;
  }

  function fitCanvas(cv) {
    if (!cv) return 0;
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth || cv.width;
    const h = cv.clientHeight || cv.height || 180;
    const bw = Math.max(1, Math.round(w * dpr));
    const bh = Math.max(1, Math.round(h * dpr));
    if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return w;
  }

  function fmtAxis(v) {
    const a = Math.abs(v);
    if (a >= 100) return v.toFixed(0);
    if (a >= 10) return v.toFixed(1);
    return v.toFixed(2);
  }

  function renderCharts() {
    for (const def of CHART_DEFS) drawCheckChart(def);
  }

  function drawCheckChart(def) {
    const cv = $(def.canvas);
    if (!cv) return;
    const w = fitCanvas(cv);
    const h = cv.clientHeight || 180;
    if (!w || !h) return;
    const ctx = cv.getContext('2d');
    ctx.clearRect(0, 0, w, h);

    const pad = { l: 44, r: 10, t: 8, b: 18 };
    const iw = w - pad.l - pad.r;
    const ih = h - pad.t - pad.b;
    if (iw <= 0 || ih <= 0) return;

    // 序列取自对应缓冲 (振动图取 vibration, EKF 图取 ekf)
    const bufKey = def.canvas === 'scVibChart' ? 'vibration' : 'ekf';
    const arr = state.buffers[bufKey] || [];
    if (!arr.length) {
      ctx.fillStyle = cvar('--text-tertiary', '#555566');
      ctx.font = '12px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(state.running ? '采样中…' : (state.result ? '本次无数据' : '开始体检后显示曲线'), w / 2, h / 2);
      return;
    }

    let min = Infinity;
    let max = -Infinity;
    for (const s of arr) {
      for (const ser of def.series) {
        const v = ser.get(s);
        if (!isFinite(v)) continue;
        if (v < min) min = v;
        if (v > max) max = v;
      }
    }
    if (!isFinite(min)) { min = 0; max = 1; }
    if (max - min < 1e-6) { max += 0.5; min -= 0.5; }
    const span = max - min;
    min -= span * 0.08;
    max += span * 0.08;

    const n = arr.length;
    const X = (i) => pad.l + (n <= 1 ? iw / 2 : (i / (n - 1)) * iw);
    const Y = (v) => pad.t + (1 - (v - min) / (max - min)) * ih;

    // 网格 + Y 轴
    ctx.strokeStyle = cvar('--border', 'rgba(255,255,255,.06)');
    ctx.fillStyle = cvar('--text-tertiary', '#555566');
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
      const v = min + (max - min) * (i / 4);
      const y = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
      ctx.fillText(fmtAxis(v), pad.l - 5, y + 3);
    }
    ctx.textAlign = 'center';
    ctx.fillText(`样本 0..${n - 1}${def.unit ? ' (' + def.unit + ')' : ''}`, w / 2, h - 5);

    // 曲线
    for (const ser of def.series) {
      ctx.strokeStyle = serColor(ser);
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < n; i++) {
        const v = ser.get(arr[i]);
        if (!isFinite(v)) continue;
        const x = X(i);
        const y = Y(v);
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // 图例
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'left';
    let lx = pad.l + 4;
    for (const ser of def.series) {
      ctx.fillStyle = serColor(ser);
      ctx.fillRect(lx, pad.t + 2, 8, 8);
      ctx.fillStyle = cvar('--text-secondary', '#aaa');
      ctx.fillText(ser.label, lx + 12, pad.t + 10);
      lx += 12 + ctx.measureText(ser.label).width + 12;
    }
  }

  // ==================== 仪表盘徽章 ====================
  function renderBadge() {
    const el = $('dashSensors');
    if (!el) return;
    const sys = state.lastSys;
    if (!sys || !sys.sensorsPresent) {
      if (el.textContent !== '传感器: --') el.textContent = '传感器: --';
      el.className = 'pill';
      return;
    }
    // 全部位掩码传感器 (主+扩展) present/health 统计
    let present = 0;
    let bad = 0;
    for (const s of SENSOR_BITS) {
      const presentMask = s.ext ? (sys.sensorsPresentExt || 0) : sys.sensorsPresent;
      const healthMask = s.ext ? (sys.sensorsHealthExt || 0) : sys.sensorsHealth;
      if (presentMask & s.bit) {
        present++;
        if (!(healthMask & s.bit)) bad++;
      }
    }
    let cls = 'pill pill--ok';
    let text;
    if (bad > 0) { cls = 'pill pill--danger'; text = `传感器 ${present - bad}/${present} 异常`; }
    else { text = `传感器 ${present} 正常`; }
    if (el.textContent !== text) el.textContent = text;
    if (el.className !== cls) el.className = cls;
  }

  // ==================== 实时数据页传感器状态条 ====================
  let lastLiveSig = '';
  function renderLiveSensors() {
    const list = $('rtSensorsList');
    if (!list) return;
    const sys = state.lastSys;
    if (!sys || !sys.sensorsPresent) {
      const html = '<span class="rt-sensor-pill na">等待 SYS_STATUS…</span>';
      if (lastLiveSig !== html) { lastLiveSig = html; list.innerHTML = html; }
      return;
    }
    // 只显示 present 的传感器 (未装的不占位), 每项带健康色
    const parts = [];
    for (const s of SENSOR_BITS) {
      const presentMask = s.ext ? (sys.sensorsPresentExt || 0) : sys.sensorsPresent;
      const healthMask = s.ext ? (sys.sensorsHealthExt || 0) : sys.sensorsHealth;
      const enabledMask = s.ext ? (sys.sensorsEnabledExt || 0) : sys.sensorsEnabled;
      if (!(presentMask & s.bit)) continue;
      const healthy = (healthMask & s.bit) !== 0;
      const enabled = (enabledMask & s.bit) !== 0;
      let cls = 'ok';
      let title = s.name + ': 健康';
      if (!healthy) { cls = 'err'; title = s.name + ': 不健康 (health=0)'; }
      else if (!enabled) { cls = 'warn'; title = s.name + ': 健康但未使能'; }
      parts.push(`<span class="rt-sensor-pill ${cls}" title="${title}">${s.name}</span>`);
    }
    // GPS 数据质量附加到 GPS 位掩码项旁
    if (state.lastGps && (sys.sensorsPresent & 32)) {
      const g = state.lastGps;
      const fixLbl = { 0: '无', 1: '无定位', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK固', 6: 'RTK浮' }[g.fixType] || ('fix' + g.fixType);
      const gcls = g.fixType >= 3 ? 'ok' : (g.fixType >= 2 ? 'warn' : 'err');
      parts.push(`<span class="rt-sensor-pill ${gcls}" title="GPS 定位质量">${fixLbl}${g.satellites ? '·' + g.satellites + '星' : ''}</span>`);
    }
    const html = parts.join('') || '<span class="rt-sensor-pill na">SYS_STATUS 无 present 位置位</span>';
    if (lastLiveSig !== html) { lastLiveSig = html; list.innerHTML = html; }
  }

  // ==================== 控件 ====================
  function bindControls() {
    const startBtn = $('btnScStart');
    if (startBtn) startBtn.onclick = () => startCheck();
    const stopBtn = $('btnScStop');
    if (stopBtn) stopBtn.onclick = () => stopCheck(false);
    const copyBtn = $('btnScCopy');
    if (copyBtn) {
      copyBtn.onclick = async () => {
        const pre = $('scReport');
        const text = pre ? pre.textContent : '';
        if (!text) return;
        try {
          await navigator.clipboard.writeText(text);
          copyBtn.textContent = '✅ 已复制';
          setTimeout(() => { copyBtn.textContent = '复制报告'; }, 1500);
        } catch (e) {
          // 剪贴板不可用时选中文本提示手动复制
          const range = document.createRange();
          range.selectNodeContents(pre);
          const sel = window.getSelection();
          sel.removeAllRanges();
          sel.addRange(range);
          copyBtn.textContent = '请 Ctrl+C';
          setTimeout(() => { copyBtn.textContent = '复制报告'; }, 2000);
        }
      };
    }
    const badge = $('dashSensors');
    if (badge) {
      badge.style.cursor = 'pointer';
      badge.onclick = () => {
        const tab = document.querySelector('.view-tab[data-view="sensors"]');
        if (tab) tab.click();
      };
    }
    const moreBtn = $('rtSensorsMore');
    if (moreBtn) {
      moreBtn.onclick = () => {
        const tab = document.querySelector('.view-tab[data-view="sensors"]');
        if (tab) tab.click();
      };
    }
    window.addEventListener('resize', () => {
      if (state.view === 'sensors') renderCharts();
    });
  }

  // ==================== 视图切换 ====================
  function onView(name) {
    state.view = name;
    if (name === 'sensors') {
      renderStatus();
      renderCards();
      renderCharts();
      renderPrearmList();
      renderReportArea();
      renderLiveCounts();
    } else if (name === 'dashboard') {
      renderBadge();
    } else if (name === 'charts') {
      renderLiveSensors();   // 实时数据页进入即刷一次状态条
    } else if (state.running) {
      // 离开体检页不停止 (允许切去别处继续采样), 但用户显式停止更符合直觉 → 停止
      stopCheck(true);
    }
  }

  // ==================== 初始化 ====================
  function init() {
    bindControls();
    renderStatus();
    renderCards();
    renderBadge();
    setInterval(() => {
      if (state.running && state.view === 'sensors') {
        renderStatus();
        renderCharts();
      }
    }, 500);
    console.log('传感器体检模块已初始化');
  }

  // 主题切换: 清 CSS 变量缓存并重绘体检曲线 (仅本页可见时)
  if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('fy:theme', () => {
      themeVars = {};
      if (state.view === 'sensors') renderCharts();
    });
  }

  window.SensorCheck = {
    init,
    onMsg,
    onView,
    // 纯函数导出 (单测)
    evaluateSensors,
    buildReport,
    THRESHOLDS,
    SENSOR_BITS,
    EKF_FLAG_NAMES,
  };

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
