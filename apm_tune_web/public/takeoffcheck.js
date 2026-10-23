/* 起飞就绪检查 (仪表盘): 快照判定 + MAV_CMD_RUN_PREARM_CHECKS(401) 主动检测
 * 判定 Auto/Guided 起飞条件: PreArm 结论 / GPS 定位 / 传感器健康 / EKF 就绪;
 * 不满足时列出原因, 可用 FaultDB 匹配中文对照与处理方法。 */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);

  const PREARM_TTL = 90000;     // 被动 PreArm 失败缓存有效期(ms)
  const PASS_RE = /all checks passing|everything ok|^ok$|checks? pass/i;
  const EKF_HORIZ_BITS = 8 | 16 | 256 | 512;  // POS_HORIZ_REL/ABS + PRED_REL/ABS

  // MAV_SYS_STATUS_SENSOR 位 (pymavlink 核定)
  const SENSOR_BITS = [
    [1, '三轴陀螺仪'],
    [2, '三轴加速度计'],
    [4, '三轴磁罗盘'],
    [8, '气压计'],
    [32, 'GPS'],
    [65536, '遥控接收机'],
    [33554432, '电池监测'],
  ];
  const SYS_PREARM_BIT = 268435456;   // MAV_SYS_STATUS_PREARM_CHECK
  const ACK_NAMES = { 1: '暂时拒绝', 2: '被飞控拒绝', 3: '固件不支持该命令', 4: '执行失败' };

  const state = {
    view: '',
    connected: false,
    firmware: null,
    armed: false,
    flightMode: '--',
    hb: false,
    gps: null,
    sys: null,
    ekf: null,
    prearmFail: null,             // {text, at} 最近 PreArm 失败(被动+主动共用)
    prearmMode: 'none',           // none | pass | unsupported
    checkNote: '',
    checking: false,
    lastCheckAt: 0,
  };

  function stripPreArm(t) {
    return String(t || '').replace(/^prearm\s*(failure)?\s*[:：]\s*/i, '').trim();
  }

  function isPassText(t) {
    const body = stripPreArm(t);
    return !body || PASS_RE.test(body);
  }

  // ---------- FaultDB 对照 (可选, 未加载时静默跳过) ----------
  function hintFor(text) {
    try {
      if (typeof window.FaultDB === 'undefined' || typeof window.FaultDB.search !== 'function') return '';
      const hits = window.FaultDB.search(stripPreArm(text));
      if (!hits.length) return '';
      const e = hits[0];
      let fix = String(e.fix || '').split('\n')[0].trim();
      if (fix.length > 90) fix = fix.slice(0, 90) + '…';
      return (e.zh || e.en) + (fix ? ' → ' + fix : '');
    } catch (err) {
      return '';
    }
  }

  function reset() {
    state.connected = false;
    state.firmware = null;
    state.armed = false;
    state.flightMode = '--';
    state.hb = false;
    state.gps = null;
    state.sys = null;
    state.ekf = null;
    state.prearmFail = null;
    state.prearmMode = 'none';
    state.checkNote = '';
    state.checking = false;
    state.lastCheckAt = 0;
  }

  // ---------- 消息 ----------
  function onMsg(msg) {
    if (!msg || !msg.type) return;
    switch (msg.type) {
      case 'connected':
        state.connected = true;
        break;
      case 'disconnect':
      case 'disconnected':
        reset();
        break;
      case 'heartbeat':
        state.connected = true;
        state.hb = true;
        state.firmware = msg.firmware || state.firmware;
        state.armed = !!msg.armed;
        state.flightMode = msg.flightMode || '--';
        break;
      case 'gps_status':
        state.gps = { fixType: msg.fixType, satellites: msg.satellites, eph: msg.eph };
        break;
      case 'sys_status':
        state.sys = {
          voltage: msg.voltageBattery,
          remaining: msg.batteryRemaining,
          load: msg.load,
          present: (msg.sensorsPresent || 0) >>> 0,
          enabled: (msg.sensorsEnabled || 0) >>> 0,
          health: (msg.sensorsHealth || 0) >>> 0,
        };
        break;
      case 'ekf_status':
        state.ekf = {
          flags: (msg.flags || 0) >>> 0,
          posHorizVariance: msg.posHorizVariance,
        };
        break;
      case 'statustext': {
        const t = String(msg.text || '');
        if (/^prearm/i.test(t)) {
          if (isPassText(t)) {
            state.prearmFail = null;
          } else {
            state.prearmFail = { text: stripPreArm(t) || t, at: Date.now() };
          }
        }
        break;
      }
      case 'prearm_check_done':
        onCheckDone(msg);
        break;
      default:
        return;
    }
    if (state.view === 'dashboard') render();
  }

  function onCheckDone(m) {
    state.checking = false;
    state.lastCheckAt = Date.now();
    if (m.busy) {
      state.checkNote = m.reason || '上一次检测尚未结束';
      return;
    }
    if (m.unsupported) {
      state.prearmMode = 'unsupported';
      state.checkNote = m.reason || '飞控不支持主动 PreArm 检测, 结果基于被动快照';
      return;
    }
    if (m.timeout || m.result === null || m.result === undefined) {
      state.prearmMode = 'unsupported';
      state.checkNote = '未收到检测 ACK(固件可能不支持 CMD401), 已用被动数据判定';
      return;
    }
    if (m.result === 3) {
      state.prearmMode = 'unsupported';
      state.checkNote = '飞控返回「不支持该命令」, 已用被动数据判定';
      return;
    }
    if (!m.ok) {
      const reason = 'PreArm 检测被飞控拒绝: ' + (ACK_NAMES[m.result] || ('result=' + m.result));
      state.prearmMode = 'none';
      state.prearmFail = { text: reason, at: Date.now() };
      return;
    }
    const fails = (m.texts || []).filter((t) => /prearm/i.test(t) && !isPassText(t));
    if (fails.length) {
      state.prearmMode = 'none';
      state.prearmFail = { text: stripPreArm(fails[fails.length - 1]), at: Date.now() };
    } else {
      state.prearmMode = 'pass';
      state.prearmFail = null;
      state.checkNote = '';
    }
  }

  // ---------- 判定 (纯函数, 不碰 DOM) ----------
  function evaluate() {
    const now = Date.now();
    const reasons = [];
    const notes = [];

    if (!state.connected) return { state: 'offline', reasons, notes };
    if (state.firmware === 'px4') {
      notes.push('PX4 的预检请在 QGroundControl 中查看');
      return { state: 'px4', reasons, notes };
    }
    if (state.armed) return { state: 'armed', mode: state.flightMode, reasons, notes };

    if (!state.hb) reasons.push({ text: '尚未收到飞控心跳(HEARTBEAT), 数据不足' });

    const pf = state.prearmFail;
    if (pf && now - pf.at < PREARM_TTL) {
      reasons.push({ text: '飞控预检未通过: ' + pf.text, hint: hintFor(pf.text) });
    }

    if (!state.gps) {
      reasons.push({ text: '尚未收到 GPS 状态(GPS_RAW_INT), 无法判定定位' });
    } else {
      const fix = state.gps.fixType;
      const sats = state.gps.satellites;
      const satsTxt = (sats !== undefined && sats !== null && sats !== 255) ? ' · ' + sats + ' 星' : '';
      if (fix < 3) {
        reasons.push({ text: 'GPS 未完成三维定位 (fix=' + fix + satsTxt + ')' });
      } else {
        if (sats !== undefined && sats !== null && sats !== 255 && sats < 6) {
          notes.push('GPS 卫星仅 ' + sats + ' 颗, 建议 ≥6 颗再起飞');
        }
        if (state.gps.eph && state.gps.eph > 0 && state.gps.eph !== 65535 && state.gps.eph > 500) {
          notes.push('GPS 精度较差 (HDOP=' + (state.gps.eph / 100).toFixed(1) + ')');
        }
      }
    }

    if (state.sys) {
      const s = state.sys;
      for (const [bit, name] of SENSOR_BITS) {
        if ((s.present & bit) && (s.enabled & bit) && !(s.health & bit)) {
          reasons.push({ text: name + '未通过健康检查 (传感器异常)' });
        }
      }
      if ((s.present & SYS_PREARM_BIT) && (s.enabled & SYS_PREARM_BIT)
          && !(s.health & SYS_PREARM_BIT) && reasons.every((r) => r.text.indexOf('预检') < 0)) {
        reasons.push({ text: '飞控 PreArm 状态位: 未通过' });
      }
      const v = s.voltage;
      if (!v || v === 65535) notes.push('电池电压未上报, 无法核对剩余电量');
      else if (s.remaining === 0) notes.push('电量显示 0%, 起飞前请确认电池');
    }

    if (state.ekf) {
      const e = state.ekf;
      if (!(e.flags & EKF_HORIZ_BITS)) reasons.push({ text: 'EKF 无水平位置估计 (位置融合未就绪)' });
      if (e.posHorizVariance != null && e.posHorizVariance > 4) {
        reasons.push({ text: 'EKF 水平位置方差过大 (' + Number(e.posHorizVariance).toFixed(1) + ' > 4)' });
      }
    }

    if (state.checkNote) notes.push(state.checkNote);
    if (state.prearmMode === 'pass' && state.lastCheckAt && now - state.lastCheckAt < 60000) {
      notes.push('主动 PreArm 检测: 通过 (' + hhmmss(state.lastCheckAt) + ')');
    } else if (state.prearmMode === 'pass') {
      notes.push('PreArm 检测结果已过期, 建议重新检测');
    } else if (state.prearmMode !== 'unsupported' && !reasons.length) {
      notes.push('尚未执行主动 PreArm 检测, 建议点「立即检测」跑完整预检');
    }
    if (state.flightMode === 'Auto') notes.push('Auto 起飞需任务包含起飞(TAKEOFF)航点并已上传');
    if (state.flightMode === 'Guided') notes.push('Guided 起飞由地面站/脚本给定航点');

    if (reasons.length) return { state: 'blocked', reasons, notes };
    const fresh = state.lastCheckAt && now - state.lastCheckAt < 60000;
    if (state.prearmMode === 'pass' && fresh) return { state: 'ready', reasons, notes };
    if (state.prearmMode === 'unsupported') return { state: 'ready', reasons, notes };  // 降级: 被动快照全绿即通过
    return { state: 'pending', reasons, notes };
  }

  // ---------- 交互 ----------
  function runCheck() {
    if (state.checking || !state.connected || state.armed || state.firmware === 'px4') return;
    state.checking = true;
    state.checkNote = '';
    render();
    try {
      if (window.ws && window.ws.readyState === 1) {
        window.ws.send(JSON.stringify({ type: 'prearm_check' }));
      } else {
        state.checking = false;
        state.checkNote = 'WebSocket 未连接, 无法发送检测指令';
        render();
        return;
      }
    } catch (e) {
      state.checking = false;
      state.checkNote = '检测指令发送失败: ' + ((e && e.message) || e);
      render();
      return;
    }
    clearTimeout(runCheck._t);
    runCheck._t = setTimeout(() => {          // FcOps 4.5s 必回, 8s 兜底恢复按钮
      if (state.checking) {
        state.checking = false;
        state.checkNote = '检测超时无响应(链路可能中断)';
        render();
      }
    }, 8000);
  }

  // ---------- 渲染 ----------
  function esc(s) {
    return String(s == null ? '' : s)
      .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  function hhmmss(t) {
    const d = new Date(t);
    const p = (n) => (n < 10 ? '0' : '') + n;
    return p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds());
  }

  const VERDICT = {
    offline: ['idle', '未连接飞控'],
    px4: ['idle', 'PX4 固件 — 请用 QGroundControl 预检'],
    armed: ['idle', ''],
    blocked: ['blocked', ''],
    ready: ['ready', '✔ 可以起飞 (Auto / Guided)'],
    pending: ['pending', '◐ 快照检查通过, 尚未执行 PreArm 检测'],
  };

  function render() {
    const el = $('tcVerdict');
    if (!el) return;
    const v = evaluate();

    if (state.checking) {
      el.className = 'tc-verdict tc-verdict--pending';
      el.textContent = '检测中… 正在让飞控执行预检';
    } else {
      const pair = VERDICT[v.state] || ['idle', ''];
      const cls = pair[0];
      let head = pair[1];
      if (v.state === 'armed') head = '已解锁 (' + (v.mode || '--') + '), 起飞检查不适用';
      if (v.state === 'blocked') head = '✖ 不可起飞 — ' + v.reasons.length + ' 项未满足';
      el.className = 'tc-verdict tc-verdict--' + cls;
      el.textContent = head;
    }

    const reasons = $('tcReasons');
    if (reasons) {
      reasons.innerHTML = v.reasons.map((r) =>
        '<li>' + esc(r.text)
        + (r.hint ? '<span class="tc-hint">' + esc(r.hint) + '</span>' : '')
        + '</li>'
      ).join('');
    }
    const notes = $('tcNotes');
    if (notes) {
      notes.innerHTML = v.notes.map((n) => '<div>· ' + esc(n) + '</div>').join('');
    }

    const checked = $('tcChecked');
    if (checked) {
      if (state.checking) checked.textContent = '检测中…';
      else if (state.lastCheckAt) checked.textContent = '最近检测 ' + hhmmss(state.lastCheckAt);
      else checked.textContent = '未检测';
    }
    const btn = $('tcRun');
    if (btn) {
      btn.disabled = state.checking || !state.connected || state.armed || state.firmware === 'px4';
      btn.textContent = state.checking ? '检测中…' : '立即检测';
    }
  }

  function onView(name) {
    state.view = name || '';
    if (state.view === 'dashboard') render();
  }

  function init() {
    const btn = $('tcRun');
    if (btn) btn.onclick = runCheck;
    render();
    console.log('起飞就绪检查已初始化');
  }

  window.TakeoffCheck = { onMsg, onView, init, evaluate, reset, runCheck, _state: state };

  if (typeof document !== 'undefined' && document.addEventListener) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
  }
})();
