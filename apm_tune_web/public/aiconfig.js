// 全局 AI 配置 (多页共用, 服务端权威)
// - 配置保存在服务器 (config/ai_config.json): provider/model/baseUrl 始终保存;
//   API Key 仅在勾选「记住」时保存到服务器, 否则只在本次服务进程内存中有效。
// - 任意页面通过 AIConfig.isConfigured()/statusText() 判断与展示;
//   发起 AI 请求时无需再传配置, 后端自动使用全局配置。
(function () {
  const state = {
    providers: [],
    provider: '',
    model: '',
    baseUrl: '',
    remember: false,
    hasKey: false,
    loaded: false,
    modal: null,
  };

  const subscribers = [];
  function notify() {
    for (const fn of subscribers) {
      try { fn(); } catch (e) { /* ignore */ }
    }
  }

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

  function isConfigured() {
    if (!state.provider || !state.model) return false;
    const p = providerInfo(state.provider);
    return !!state.hasKey || !!(p && p.keyOptional);
  }

  function statusText() {
    if (!state.loaded) return 'AI: 读取中...';
    if (!state.provider || !state.model) return 'AI: 未配置（仅生成摘要）';
    const keyOk = !!state.hasKey;
    return `AI: ${providerLabel(state.provider)} / ${state.model}` + (keyOk ? ' · Key 已设置' : ' · 未填 Key');
  }

  function get() {
    return {
      providers: state.providers,
      provider: state.provider,
      model: state.model,
      baseUrl: state.baseUrl,
      remember: state.remember,
      hasKey: state.hasKey,
      loaded: state.loaded,
      configured: isConfigured(),
    };
  }

  function subscribe(fn) {
    if (typeof fn === 'function' && !subscribers.includes(fn)) {
      subscribers.push(fn);
      try { fn(); } catch (e) { /* ignore */ }
    }
  }

  /** 兼容旧接口: 现在配置由服务端持有, 请求无需携带 AI 字段 */
  function toRequest() {
    return {};
  }

  // ---------- WS ----------
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
      state.provider = c.provider || '';
      state.model = c.model || '';
      state.baseUrl = c.baseUrl || '';
      state.remember = !!c.remember;
      state.hasKey = !!c.hasKey;
      state.loaded = true;
      if (state.modal) renderModal();
      notify();
    }
  }

  /** 保存到服务端 (apiKey 为空字符串时表示保持原 Key 不变) */
  function save(cfg) {
    const payload = { type: 'ai_config_set', config: cfg.config || {} };
    if (typeof cfg.apiKey === 'string' && cfg.apiKey.trim()) payload.apiKey = cfg.apiKey.trim();
    return wsSend(payload);
  }

  function clearKey() {
    return wsSend({ type: 'ai_config_clear_key' });
  }

  // ---------- 设置弹窗 ----------
  function openSettings() {
    if (!state.modal) {
      const modal = document.createElement('div');
      modal.className = 'ot-modal';
      modal.innerHTML = `
        <div class="ot-modal-box ai-modal-box">
          <h3>全局 AI 设置 <span class="ai-modal-hint">（日志分析 / 故障速查 共用，设置一次即可）</span></h3>
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
              记住 API Key（保存到本机服务器 config/ai_config.json，仅建议本地部署使用）
            </label>
            <p class="dt-note ai-note">🔒 不勾选「记住」时，Key 仅本次服务运行期间在内存有效，重启即丢；「清除 Key」可随时移除。</p>
          </div>
          <div class="ot-modal-actions">
            <button class="ai-save">保存</button>
            <button class="ai-clear" title="清除服务器保存/内存中的 API Key">清除 Key</button>
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
        const keyEl = modal.querySelector('.ai-key');
        if (!save({ config: cfg, apiKey: keyEl.value })) { alert('WebSocket 未连接，无法保存'); return; }
        closeSettings();
      };
      modal.querySelector('.ai-clear').onclick = () => {
        if (!confirm('清除服务器上保存/内存中的 API Key？')) return;
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
    keyEl.placeholder = state.hasKey ? '已保存，留空保持不变' : (optional ? '可免 Key' : '请输入 API Key');
    keyState.textContent = state.hasKey ? '（当前已配置 Key）' : '';
    keyState.className = 'ai-key-state' + (state.hasKey ? ' ok' : '');
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
    get,
    save,
  };
})();
