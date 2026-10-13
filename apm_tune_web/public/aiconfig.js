// AI 配置 (B 方案: Key 归属浏览器, 服务端不保存)
// - 配置与 API Key 只属于当前浏览器: 默认仅存内存(刷新即失), 勾选「记住在本浏览器」
//   才写入 localStorage (fy_ai_cfg / fy_ai_key), 任何时候都不会上传/落盘到服务器
// - 发起 AI 分析时把配置随消息携带 (toRequest()), 服务端只做代理调用, 不记日志
// - 若管理员在服务端配了默认 (env AI_*), 本机未配置时用其兜底, 状态栏标注"服务器默认"
// - 任意页面通过 AIConfig.isConfigured()/statusText() 判断与展示
(function () {
  const LS_CFG = 'fy_ai_cfg';   // { provider, model, baseUrl, remember }
  const LS_KEY = 'fy_ai_key';   // 明文 Key (仅 remember=true 时写入)

  const state = {
    providers: [],
    provider: '',
    model: '',
    baseUrl: '',
    remember: false,
    key: '',
    server: null,     // 服务端默认 (env AI_*, 不含明文 Key)
    loaded: false,
    modal: null,
  };

  const subscribers = [];
  function notify() {
    for (const fn of subscribers) {
      try { fn(); } catch (e) { /* ignore */ }
    }
  }

  function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
  function lsSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* 隐私模式等忽略 */ } }
  function lsDel(k) { try { localStorage.removeItem(k); } catch (e) { /* ignore */ } }

  function loadLocal() {
    try {
      const raw = lsGet(LS_CFG);
      if (raw) {
        const d = JSON.parse(raw) || {};
        state.provider = String(d.provider || '').trim();
        state.model = String(d.model || '').trim();
        state.baseUrl = String(d.baseUrl || '').trim();
        state.remember = !!d.remember;
        if (state.remember) state.key = String(lsGet(LS_KEY) || '');
        if (state.provider || state.model) state.loaded = true;
      }
    } catch (e) { /* 损坏则按未配置处理 */ }
  }

  /** 写回本机: 记住 → 存 Key; 不记住 → 只留内存(并清掉历史存的 Key) */
  function persistLocal() {
    lsSet(LS_CFG, JSON.stringify({
      provider: state.provider, model: state.model, baseUrl: state.baseUrl, remember: state.remember,
    }));
    if (state.remember && state.key) lsSet(LS_KEY, state.key);
    else lsDel(LS_KEY);
  }
  loadLocal();

  function wsSend(obj) {
    if (typeof ws !== 'undefined' && ws && ws.readyState === 1) {
      ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  function providerInfo(id) {
    return state.providers.find((p) => p.id === id) || null;
  }

  function providerLabel(id) {
    const p = providerInfo(id);
    return p ? p.label : (id || '');
  }

  function isKeyOptional(id) {
    const p = providerInfo(id);
    return !!(p && p.keyOptional);
  }

  /** 本机配置可独立发起调用 (有 Key, 或该服务商免 Key) */
  function localReady() {
    if (!state.provider || !state.model) return false;
    return !!state.key || isKeyOptional(state.provider);
  }

  /** 服务端默认可发起调用 (本机没配时的兜底) */
  function serverReady() {
    const s = state.server;
    if (!s || !s.provider || !s.model) return false;
    return !!s.hasKey || isKeyOptional(s.provider);
  }

  function isConfigured() {
    if (!state.loaded && !localReady()) return false;
    return localReady() || serverReady();
  }

  /** AI 最终会从哪来: 本机 / 服务端 / 未配置 */
  function source() {
    if (localReady()) return 'client';
    if (serverReady()) return 'server';
    return '';
  }

  function statusText() {
    if (!state.loaded) return 'AI: 读取中...';
    if (localReady()) {
      return `AI: ${providerLabel(state.provider)} / ${state.model} · ${state.key ? '本机 Key' : '免 Key'}`;
    }
    if (serverReady()) {
      const s = state.server;
      return `AI: ${providerLabel(s.provider)} / ${s.model} · 服务器默认`;
    }
    if (state.provider && state.model) {
      return `AI: ${providerLabel(state.provider)} / ${state.model} · 未填 Key`;
    }
    return 'AI: 未配置（仅生成摘要）';
  }

  function get() {
    return {
      providers: state.providers,
      provider: state.provider,
      model: state.model,
      baseUrl: state.baseUrl,
      remember: state.remember,
      hasKey: !!state.key,
      loaded: state.loaded,
      configured: isConfigured(),
      source: source(),
      server: state.server,
    };
  }

  function subscribe(fn) {
    if (typeof fn === 'function' && !subscribers.includes(fn)) {
      subscribers.push(fn);
      try { fn(); } catch (e) { /* ignore */ }
    }
  }

  /** 随 AI 分析请求携带的配置; 本机不可用时返回 {} (交给服务端默认或提示未配置) */
  function toRequest() {
    if (!localReady()) return {};
    const o = { provider: state.provider, model: state.model, baseUrl: state.baseUrl };
    if (state.key) o.apiKey = state.key;
    return o;
  }

  // ---------- WS (仅拉取服务商目录与服务端默认, 不再上传任何配置) ----------
  function requestProviders() { wsSend({ type: 'ai_providers' }); }
  function requestConfig() { wsSend({ type: 'ai_config_get' }); }

  function onOpen() {
    requestProviders();
    requestConfig();
  }

  function handleMessage(msg) {
    if (msg.type === 'ai_providers') {
      state.providers = msg.providers || [];
      if (state.modal) renderModal();
      notify();
      return;
    }
    if (msg.type === 'ai_config') {
      const c = msg.config || {};
      state.server = {
        provider: String(c.provider || '').trim(),
        model: String(c.model || '').trim(),
        baseUrl: String(c.baseUrl || '').trim(),
        hasKey: !!c.hasKey,
      };
      state.loaded = true;
      if (state.modal) renderModal();
      notify();
    }
  }

  /** 保存到本机浏览器 (apiKey 为空字符串表示保持原 Key 不变) */
  function save(cfg) {
    const config = (cfg && cfg.config) || {};
    if ('provider' in config) state.provider = String(config.provider || '').trim();
    if ('model' in config) state.model = String(config.model || '').trim();
    if ('baseUrl' in config) state.baseUrl = String(config.baseUrl || '').trim();
    if ('remember' in config) state.remember = !!config.remember;
    const k = cfg && typeof cfg.apiKey === 'string' ? cfg.apiKey.trim() : '';
    if (k) state.key = k;
    state.loaded = true;
    persistLocal();
    notify();
    return true;
  }

  /** 清除本机浏览器里保存的 Key */
  function clearKey() {
    state.key = '';
    lsDel(LS_KEY);
    notify();
    return true;
  }

  // ---------- 设置弹窗 ----------
  function openSettings() {
    if (!state.modal) {
      const modal = document.createElement('div');
      modal.className = 'ot-modal';
      modal.innerHTML = `
        <div class="ot-modal-box ai-modal-box">
          <h3>AI 设置 <span class="ai-modal-hint">（仅保存在本机浏览器，服务器不留存）</span></h3>
          <div class="ai-form">
            <label>服务商
              <select class="ai-provider"></select>
            </label>
            <label>模型
              <input class="ai-model" list="aiModelListGlobal" placeholder="模型名" />
              <datalist id="aiModelListGlobal"></datalist>
            </label>
            <label>API Key
              <input class="ai-key" type="password" autocomplete="off" />
              <span class="ai-key-state"></span>
            </label>
            <label>Base URL(可选)
              <input class="ai-baseurl" placeholder="留空用服务商默认" />
            </label>
            <label class="ai-remember">
              <input type="checkbox" class="ai-remember-ck" />
              记住在本浏览器（写入 localStorage，不上传服务器）
            </label>
            <p class="dt-note ai-note">🔒 Key 只存在你自己的浏览器：发起分析时经 HTTPS 随请求交给服务端代理调用，服务端不保存、不写日志；不勾选「记住」则刷新即失效，「清除 Key」可随时移除。</p>
            <p class="dt-note ai-server-hint" style="display:none"></p>
          </div>
          <div class="ot-modal-actions">
            <button class="ai-save">保存</button>
            <button class="ai-clear" title="清除本机浏览器保存的 API Key">清除 Key</button>
            <button class="ot-modal-close ai-close">关闭</button>
          </div>
        </div>`;
      modal.querySelector('.ai-provider').addEventListener('change', () => fillModelAndBase(modal, true));
      modal.querySelector('.ai-key').addEventListener('input', (e) => { e.target.dataset.dirty = '1'; });
      modal.querySelector('.ai-save').onclick = () => {
        const cfg = {
          provider: modal.querySelector('.ai-provider').value,
          model: modal.querySelector('.ai-model').value,
          baseUrl: modal.querySelector('.ai-baseurl').value,
          remember: modal.querySelector('.ai-remember-ck').checked,
        };
        save({ config: cfg, apiKey: modal.querySelector('.ai-key').value });
        closeSettings();
      };
      modal.querySelector('.ai-clear').onclick = () => {
        if (!confirm('清除本机浏览器里保存的 API Key？')) return;
        clearKey();
        modal.querySelector('.ai-key').value = '';
        modal.querySelector('.ai-key').dataset.dirty = '';
      };
      modal.querySelector('.ai-close').onclick = closeSettings;
      modal.addEventListener('click', (e) => { if (e.target === modal) closeSettings(); });
      state.modal = modal;
    }
    renderModal();
    document.body.appendChild(state.modal);
  }

  function closeSettings() {
    if (state.modal && state.modal.parentNode) state.modal.parentNode.removeChild(state.modal);
  }

  function fillModelAndBase(modal, resetModel) {
    const sel = modal.querySelector('.ai-provider');
    const info = providerInfo(sel.value);
    const dl = modal.querySelector('#aiModelListGlobal');
    dl.innerHTML = '';
    if (info) {
      for (const m of info.models || []) {
        const o = document.createElement('option');
        o.value = m;
        dl.appendChild(o);
      }
    }
    if (resetModel) modal.querySelector('.ai-model').value = (info && info.models && info.models[0]) || '';
    const base = modal.querySelector('.ai-baseurl');
    base.placeholder = info && info.baseUrl ? info.baseUrl : '留空用服务商默认';
    const keyState = modal.querySelector('.ai-key-state');
    const keyEl = modal.querySelector('.ai-key');
    const optional = info && info.keyOptional;
    keyEl.placeholder = state.key ? '已保存，留空保持不变' : (optional ? '可免 Key' : '请输入 API Key');
    keyState.textContent = state.key ? '（本机已存 Key）' : '';
    keyState.className = 'ai-key-state' + (state.key ? ' ok' : '');
  }

  function renderModal() {
    const modal = state.modal;
    if (!modal) return;
    const sel = modal.querySelector('.ai-provider');
    sel.innerHTML = '<option value="">（不使用 AI，仅生成摘要）</option>';
    for (const p of state.providers) {
      const o = document.createElement('option');
      o.value = p.id;
      o.textContent = p.label + (p.keyOptional ? ' (可免 Key)' : '');
      sel.appendChild(o);
    }
    sel.value = state.provider || '';
    const modelEl = modal.querySelector('.ai-model');
    if (!modelEl.dataset.touched) modelEl.value = state.model || '';
    modal.querySelector('.ai-baseurl').value = state.baseUrl || '';
    modal.querySelector('.ai-remember-ck').checked = !!state.remember;
    fillModelAndBase(modal, false);

    // 服务端默认提示 (本机没配时告知会用它)
    const hint = modal.querySelector('.ai-server-hint');
    if (hint) {
      if (!localReady() && serverReady()) {
        const s = state.server;
        hint.style.display = '';
        hint.textContent = `服务器已提供默认 AI：${providerLabel(s.provider)} / ${s.model}，本机未填 Key 时自动使用它。`;
      } else {
        hint.style.display = 'none';
        hint.textContent = '';
      }
    }
  }

  window.AIConfig = {
    init: onOpen,
    onOpen,
    handleMessage,
    openSettings,
    clearStoredKey: clearKey,  // 兼容旧命名
    clearKey,
    requestProviders,
    requestConfig,
    subscribe,
    toRequest,
    statusText,
    isConfigured,
    source,
    get,
    save,
  };
})();
