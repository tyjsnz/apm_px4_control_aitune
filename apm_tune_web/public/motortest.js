// 电机测试页: 拆桨实测起转 (自动定 MOT_SPIN_ARM / MOT_SPIN_MIN)
// 后端: ot_spin_start → 逐档升油门(DO_MOTOR_TEST) + ESC 遥测 RPM 判起转点 → ot_spin_progress / ot_spin_done
(function () {
  const $ = (id) => document.getElementById(id);

  const state = {
    view: 'motortest',
    connected: false,
    running: false,    // 拆桨实测进行中
    results: null,     // 最近一次成功结果 { spin, arm, min }
    // 单电机点动测试
    singleActive: false,
    singleMotor: 0,
    singleTimer: null,
  };

  function isPx4() {
    return typeof window.getFirmwareType === 'function' && window.getFirmwareType() === 'px4';
  }

  function wsSend(obj) {
    if (window.ws && window.ws.readyState === 1) window.ws.send(JSON.stringify(obj));
  }

  function log(text, clear) {
    const el = $('mtLog');
    if (!el) return;
    if (clear) el.textContent = '';
    if (text) {
      el.textContent += (el.textContent && !el.textContent.endsWith('\n') ? '\n' : '') + text + '\n';
      el.scrollTop = el.scrollHeight;
    }
  }

  function renderStatus() {
    const st = $('mtStatus');
    if (st) {
      st.textContent = !state.connected ? '未连接'
        : (state.running ? '实测中...' : (isPx4() ? 'PX4 请用 QGC' : '就绪'));
      st.className = 'mt-status ' + (!state.connected ? 'bad' : (state.running ? 'busy' : 'ok'));
    }
    const btn = $('mtStart');
    if (btn) {
      btn.disabled = !state.connected || state.running || state.singleActive || isPx4();
      btn.title = isPx4() ? 'PX4 拆桨实测请使用 QGC 电机测试' : '';
      if (!state.running && !btn.dataset.normalText) { /* 初始文案保留在 HTML */ }
    }
    if (btn && !state.running && btn.dataset.busy === '1') {
      btn.textContent = '⚠ 开始拆桨实测(自动定 SPIN_ARM)';
      delete btn.dataset.busy;
    }
    if (btn && state.running && btn.dataset.busy !== '1') {
      btn.textContent = '⏳ 拆桨实测中...';
      btn.dataset.busy = '1';
    }
    // 单电机点动控件
    const canSingle = state.connected && !state.running && !state.singleActive && !isPx4();
    document.querySelectorAll('.mt-motor-btn').forEach((b) => {
      b.disabled = !canSingle;
      b.title = isPx4() ? 'PX4 请使用 QGC 电机测试' : '';
    });
    ['mtThrPct', 'mtDurSec'].forEach((id) => {
      const el = $(id);
      if (el) el.disabled = !canSingle;
    });
    const stopBtn = $('mtStop');
    if (stopBtn) stopBtn.disabled = !state.singleActive;
  }

  function renderResult() {
    const box = $('mtResult');
    if (!box) return;
    const r = state.results;
    if (!r) { box.classList.add('hidden'); box.innerHTML = ''; return; }
    box.classList.remove('hidden');
    box.innerHTML = `
      <div class="mt-result-title">✅ 起转点实测 ≈ ${r.spin}%</div>
      <div class="mt-result-rows">
        <div><span>MOT_SPIN_ARM</span><code>${r.arm}</code></div>
        <div><span>MOT_SPIN_MIN</span><code>${r.min}</code></div>
      </div>
      <div class="mt-result-actions">
        <button id="mtCopy">复制数值</button>
        <span class="mt-result-hint">${r.filled
          ? '已自动回填一键首飞方案表 (如已生成)'
          : '一键首飞方案表未生成 — 请切换到「一键首飞」页生成方案后重新实测，或手动记录'}</span>
      </div>`;
    const copyBtn = $('mtCopy');
    if (copyBtn) {
      copyBtn.onclick = () => {
        const txt = `MOT_SPIN_ARM=${r.arm}\nMOT_SPIN_MIN=${r.min}`;
        if (navigator.clipboard && navigator.clipboard.writeText) {
          navigator.clipboard.writeText(txt).then(() => { copyBtn.textContent = '已复制'; setTimeout(() => { copyBtn.textContent = '复制数值'; }, 2000); });
        } else {
          alert(txt);
        }
      };
    }
  }

  // ---------- 二次确认 + 启动 ----------
  function startTest() {
    if (!state.connected) { alert('请先连接飞控'); return; }
    if (state.running) return;
    if (window.OneTouch && typeof window.OneTouch.isBusy === 'function' && window.OneTouch.isBusy()) {
      alert('一键首飞页有操作正在进行(读取/写入/校准)，请等待完成后再试');
      return;
    }
    const modal = document.createElement('div');
    modal.className = 'ot-modal';
    modal.innerHTML = `
      <div class="ot-modal-box">
        <h3>⚠ 拆桨实测 · 二次确认</h3>
        <p>本功能会逐档升油门让电机转动，用来测定起转点。</p>
        <p class="ot-danger">⚠⚠ 必须先拆除所有桨叶！</p>
        <p>需要 ESC 遥测(DShot + 遥测线/双向 DShot)；没有遥测会中止。<br>
           飞控需处于落地、未解锁、安全开关未锁状态。</p>
        <label class="ot-confirm-lab">
          <input type="checkbox" id="otSpinCk">
          我已拆下全部桨叶，确认人员与手指远离电机、飞控已固定
        </label>
        <div class="ot-modal-actions">
          <button id="otSpinGo" disabled>✓ 已拆桨，开始实测</button>
          <button id="otSpinCancel">取消</button>
        </div>
      </div>`;
    document.body.appendChild(modal);
    modal.querySelector('#otSpinCk').addEventListener('change', (e) => {
      modal.querySelector('#otSpinGo').disabled = !e.target.checked;
    });
    modal.querySelector('#otSpinCancel').onclick = () => modal.remove();
    modal.querySelector('#otSpinGo').onclick = () => {
      modal.remove();
      state.running = true;
      window.__motorTestRunning = true;
      renderStatus();
      log('拆桨实测: 从 6% 逐档升油门寻找起转点...(进行中请勿操作其它功能)', true);
      wsSend({ type: 'ot_spin_start' });
      clearTimeout(state.timer);
      // 兜底: 长时间未回执则复位 (正常应由 ot_spin_done 结束)
      state.timer = setTimeout(() => {
        if (state.running) {
          state.running = false;
          window.__motorTestRunning = false;
          renderStatus();
          log('⚠ 拆桨实测等待超时(未收到完成回执)，已复位。请检查链路。');
        }
      }, 180000);
    };
  }

  function onSpinDone(msg) {
    clearTimeout(state.timer);
    state.running = false;
    window.__motorTestRunning = false;
    renderStatus();
    if (msg.ok) {
      // 回填一键首飞方案表 (若已生成)
      let filled = false;
      if (window.OneTouch && typeof window.OneTouch.applySpinResult === 'function') {
        filled = !!window.OneTouch.applySpinResult(msg);
      }
      state.results = { spin: msg.spin, arm: msg.arm, min: msg.min, filled };
      renderResult();
      log(`✅ 起转点 ≈ ${msg.spin}% → MOT_SPIN_ARM=${msg.arm}、MOT_SPIN_MIN=${msg.min}` + (filled ? ' (已回填方案表)' : ''));
    } else {
      log('❌ ' + (msg.text || '拆桨实测失败'));
    }
  }

  // ---------- 单电机点动测试 (检查电机/桨叶安装, 类 Mission Planner) ----------
  function readSingleInputs() {
    const tIn = $('mtThrPct');
    const sIn = $('mtDurSec');
    let pct = Math.round(Number(tIn && tIn.value));
    let sec = Math.round(Number(sIn && sIn.value));
    if (!isFinite(pct)) pct = 10;
    if (!isFinite(sec)) sec = 5;
    pct = Math.min(30, Math.max(1, pct));
    sec = Math.min(10, Math.max(1, sec));
    if (tIn) tIn.value = pct;
    if (sIn) sIn.value = sec;
    return { pct, sec };
  }

  function finishSingle(reason) {
    clearTimeout(state.singleTimer);
    state.singleActive = false;
    state.singleMotor = 0;
    window.__motorTestRunning = false;
    renderStatus();
    if (reason) log(reason);
  }

  function startSingle(motor) {
    if (!state.connected) { alert('请先连接飞控'); return; }
    if (state.running || state.singleActive) return;
    if (window.OneTouch && typeof window.OneTouch.isBusy === 'function' && window.OneTouch.isBusy()) {
      alert('一键首飞页有操作正在进行(读取/写入/校准)，请等待完成后再试');
      return;
    }
    const ck = $('mtSingleCk');
    if (!ck || !ck.checked) {
      alert('请先勾选安全确认: 带桨测试仅限低油门、人员手指远离桨叶、飞控已固定');
      return;
    }
    const { pct, sec } = readSingleInputs();
    state.singleActive = true;          // 占位防双击; 失败回执复位
    state.singleMotor = motor;
    window.__motorTestRunning = true;
    renderStatus();
    wsSend({ type: 'ot_motor_test', motor, pct, seconds: sec });
    clearTimeout(state.singleTimer);
    state.singleTimer = setTimeout(() => {
      if (state.singleActive) {
        finishSingle('⚠ 未收到点动回执, 已复位界面。若电机仍在转请点「立即停止」');
      }
    }, sec * 1000 + 4000);
  }

  function stopSingle() {
    if (!state.singleActive) return;
    wsSend({ type: 'ot_motor_stop', motor: state.singleMotor || 1 });
    clearTimeout(state.singleTimer);
    state.singleTimer = setTimeout(() => {
      if (state.singleActive) {
        finishSingle('⚠ 停止回执超时, 已复位界面 (电机应已由飞控按时自动停止)');
      }
    }, 3500);
  }

  // ==================== 模块入口 ====================
  function onMsg(msg) {
    switch (msg.type) {
      case 'connected':
        state.connected = true;
        renderStatus();
        break;
      case 'disconnected':
        state.connected = false;
        if (state.running) {
          clearTimeout(state.timer);
          state.running = false;
          window.__motorTestRunning = false;
          log('⚠ 连接断开, 实测已中止');
        }
        if (state.singleActive) finishSingle('⚠ 连接断开, 单电机测试状态已复位');
        renderStatus();
        break;
      case 'heartbeat':
        break;
      case 'ot_spin_progress':
        if (state.view === 'motortest') log(`  电机${msg.motor} @ ${msg.pct}%: RPM=${msg.rpm}`);
        break;
      case 'ot_spin_done':
        onSpinDone(msg);
        break;
      case 'ot_motor_test_done': {
        if (msg.ok) {
          clearTimeout(state.singleTimer);
          log(`▶ 电机${msg.motor} @ ${msg.pct}% 点动 ${msg.seconds}s (到时飞控自动停止)` + (msg.note ? ` — ${msg.note}` : ''));
          state.singleTimer = setTimeout(() => {
            finishSingle(`✓ 电机${state.singleMotor} 点动结束 (飞控已自动停止输出)`);
          }, msg.seconds * 1000 + 1500);
        } else {
          finishSingle('❌ 单电机测试失败: ' + (msg.text || '未知错误'));
        }
        break;
      }
      case 'ot_motor_stop_done':
        finishSingle(msg.ok
          ? `■ 已停止电机${msg.motor} (输出已复位)`
          : '❌ 停止失败: ' + (msg.text || '未知错误'));
        break;
      default:
        break;
    }
  }

  function onView(name) {
    state.view = name;
    if (name === 'motortest') renderStatus();
  }

  function init() {
    const btn = $('mtStart');
    if (btn) btn.onclick = startTest;
    document.querySelectorAll('.mt-motor-btn').forEach((b) => {
      b.onclick = () => startSingle(Number(b.dataset.motor));
    });
    const stopBtn = $('mtStop');
    if (stopBtn) stopBtn.onclick = stopSingle;
    renderStatus();
  }

  window.MotorTest = { init, onMsg, onView };
  if (typeof document !== 'undefined') window.MotorTest.init();
})();
