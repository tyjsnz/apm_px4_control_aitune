// 飞控故障速查 (前端)
// 记录故障/修复, 服务器 faults/ 永久存储; 可调用 AI 协助分析
// AI 配置为全局共享 (见 aiconfig.js)
(function () {
  const $ = (id) => document.getElementById(id);
  const state = {
    faults: [],
    currentFile: null,
    aiAt: '',
    dirty: false,
    analyzing: false,
    loaded: false,
  };

  const SEV_LABEL = { info: '提示', low: '轻微', mid: '中等', high: '严重' };
  const STATUS_LABEL = { open: '未解决', monitor: '观察中', fixed: '已解决' };

  const wsSend = (obj) => {
    if (typeof ws !== 'undefined' && ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
    else alert('WebSocket 未连接');
  };

  const esc = (s) => String(s === undefined || s === null ? '' : s)
    .replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  const fmtTime = (s) => {
    if (!s) return '--';
    const d = new Date(s);
    if (isNaN(d.getTime())) return s;
    const p = (n) => String(n).padStart(2, '0');
    return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
  };

  function setStatus(text, cls) {
    const el = $('ftStatus');
    if (!el) return;
    el.textContent = text;
    el.className = 'dt-status' + (cls ? ' ' + cls : '');
  }

  // ---------- 表单 ----------
  function formRecord() {
    return {
      file: state.currentFile || undefined,
      title: $('ftTitle').value,
      aircraft: $('ftAircraft').value,
      firmware: $('ftFirmware').value,
      severity: $('ftSeverity').value,
      status: $('ftStatus').value,
      tags: $('ftTags').value,
      symptom: $('ftSymptom').value,
      context: $('ftContext').value,
      cause: $('ftCause').value,
      fix: $('ftFix').value,
      aiAnalysis: $('ftAiReport').value,
      aiAt: state.aiAt || '',
    };
  }

  function markDirty(v) {
    state.dirty = !!v;
  }

  function loadRecord(rec) {
    state.currentFile = rec.file || null;
    state.aiAt = rec.aiAt || '';
    $('ftTitle').value = rec.title || '';
    $('ftAircraft').value = rec.aircraft || '';
    $('ftFirmware').value = rec.firmware || '';
    $('ftSeverity').value = rec.severity || 'mid';
    $('ftStatus').value = rec.status || 'open';
    $('ftTags').value = Array.isArray(rec.tags) ? rec.tags.join(', ') : (rec.tags || '');
    $('ftSymptom').value = rec.symptom || '';
    $('ftContext').value = rec.context || '';
    $('ftCause').value = rec.cause || '';
    $('ftFix').value = rec.fix || '';
    $('ftAiReport').value = rec.aiAnalysis || '';
    markDirty(false);
    highlightRow();
    setStatus('已载入: ' + (rec.title || rec.file));
  }

  function clearForm() {
    state.currentFile = null;
    state.aiAt = '';
    for (const id of ['ftTitle', 'ftAircraft', 'ftFirmware', 'ftTags', 'ftSymptom',
      'ftContext', 'ftCause', 'ftFix', 'ftAiReport']) $(id).value = '';
    $('ftSeverity').value = 'mid';
    $('ftStatus').value = 'open';
    markDirty(false);
    highlightRow();
    setStatus('新建记录 (未保存)');
  }

  function highlightRow() {
    for (const tr of document.querySelectorAll('#ftTableBody tr')) {
      tr.classList.toggle('active', tr.dataset.file === state.currentFile);
    }
  }

  // ---------- 列表 ----------
  function renderList() {
    const tb = $('ftTableBody');
    const q = ($('ftSearch').value || '').trim().toLowerCase();
    const list = state.faults.filter((f) => {
      if (!q) return true;
      const hay = [f.title, f.symptom, f.context, f.cause, f.fix,
        (f.tags || []).join(' '), f.aircraft, f.firmware].join(' ').toLowerCase();
      return hay.includes(q);
    });
    tb.innerHTML = '';
    if (!list.length) {
      tb.innerHTML = `<tr><td colspan="4" class="dt-empty">${state.faults.length ? '没有匹配的记录' : '暂无记录，点「＋ 新建记录」开始'}</td></tr>`;
    }
    for (const f of list) {
      const tr = document.createElement('tr');
      tr.dataset.file = f.file;
      if (f.file === state.currentFile) tr.classList.add('active');
      tr.innerHTML = `<td class="dt-mono">${esc(fmtTime(f.updatedAt))}</td>`
        + `<td>${esc(f.title || '未命名故障')}</td>`
        + `<td><span class="ft-sev ft-sev-${esc(f.severity || 'mid')}">${esc(SEV_LABEL[f.severity] || f.severity || '中等')}</span></td>`
        + `<td><span class="ft-st ft-st-${esc(f.status || 'open')}">${esc(STATUS_LABEL[f.status] || f.status || '未解决')}</span></td>`;
      tr.onclick = () => {
        if (state.dirty && !confirm('当前记录有未保存的修改，切换将丢失。继续？')) return;
        loadRecord(f);
      };
      tb.appendChild(tr);
    }
    $('ftListMeta').textContent = `共 ${state.faults.length} 条` + (q ? `, 匹配 ${list.length} 条` : '');
  }

  // ---------- 保存 / 删除 / 导出 ----------
  function saveRecord(silent) {
    const rec = formRecord();
    if (!rec.title.trim() && !rec.symptom.trim()) {
      if (!silent) alert('请至少填写「标题」或「故障现象」');
      return false;
    }
    wsSend({ type: 'fault_save', record: rec });
    return true;
  }

  function deleteCurrent() {
    if (!state.currentFile) { alert('当前没有已保存的记录'); return; }
    if (!confirm('确定删除当前记录？此操作不可恢复。')) return;
    wsSend({ type: 'fault_delete', file: state.currentFile });
    state.currentFile = null;
  }

  function toMarkdown(rec) {
    const L = [];
    L.push(`# ${rec.title || '未命名故障'}`);
    const meta = [];
    if (rec.aircraft) meta.push(`机型: ${rec.aircraft}`);
    if (rec.firmware) meta.push(`固件: ${rec.firmware}`);
    meta.push(`严重: ${SEV_LABEL[rec.severity] || rec.severity || '中等'}`);
    meta.push(`状态: ${STATUS_LABEL[rec.status] || rec.status || '未解决'}`);
    if (rec.tags && rec.tags.length) meta.push(`标签: ${Array.isArray(rec.tags) ? rec.tags.join(', ') : rec.tags}`);
    if (rec.createdAt) meta.push(`创建: ${rec.createdAt}`);
    if (rec.updatedAt) meta.push(`更新: ${rec.updatedAt}`);
    L.push('', meta.join('  |  '), '');
    const sec = (t, v) => { if (v && String(v).trim()) L.push(`## ${t}`, '', String(v).trim(), ''); };
    sec('故障现象', rec.symptom);
    sec('上下文 / 日志 / 已做操作', rec.context);
    sec('原因判断', rec.cause);
    sec('修复方案与结果', rec.fix);
    sec('AI 分析', rec.aiAnalysis);
    return L.join('\n');
  }

  function exportMarkdown() {
    const rec = formRecord();
    if (!rec.title && !rec.symptom) { alert('没有可导出的内容'); return; }
    const md = toMarkdown(rec);
    const name = (rec.title || 'fault').replace(/[\\/:*?"<>|]/g, '_').slice(0, 40);
    const blob = new Blob([md], { type: 'text/markdown;charset=utf-8' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${name}.md`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  }

  // ---------- AI ----------
  function updateAiStatus() {
    const el = $('ftAiStatus');
    if (!el) return;
    if (typeof AIConfig === 'undefined') { el.textContent = 'AI: 未配置'; return; }
    el.textContent = AIConfig.statusText();
    el.classList.toggle('configured', AIConfig.isConfigured());
  }

  function runAi() {
    if (state.analyzing) { alert('AI 正在分析中，请稍候...'); return; }
    const rec = formRecord();
    if (!String(rec.title || '').trim() && !String(rec.symptom || '').trim()) {
      alert('请先填写「标题」或「故障现象」');
      return;
    }
    if (typeof AIConfig === 'undefined' || !AIConfig.isConfigured()) {
      if (typeof AIConfig !== 'undefined'
        && confirm('尚未配置可用的全局 AI（服务商/模型/API Key 不完整）。\n现在打开「AI 设置」？设置一次全局共用。')) {
        AIConfig.openSettings();
      }
      return;
    }
    state.analyzing = true;
    updateButtons();
    setStatus('AI 分析中... (可能需数十秒)');
    // AI 配置由服务端全局持有, 此处无需携带
    wsSend({ type: 'fault_analyze', record: rec });
  }

  function onAnalyzeDone(msg) {
    state.analyzing = false;
    updateButtons();
    if (msg.ok) {
      $('ftAiReport').value = msg.report || '';
      state.aiAt = msg.at || new Date().toISOString();
      saveRecord(true);
      setStatus('AI 分析完成并已保存');
    } else {
      setStatus('AI 分析失败: ' + (msg.error || '未知错误'));
      alert('AI 分析失败: ' + (msg.error || '未知错误'));
    }
  }

  // ---------- 按钮状态 ----------
  function updateButtons() {
    if ($('ftAiRun')) $('ftAiRun').disabled = state.analyzing;
    if ($('ftSave')) $('ftSave').disabled = state.analyzing;
  }

  // ---------- 消息 ----------
  function handleMessage(msg) {
    switch (msg.type) {
      case 'fault_list':
        state.faults = msg.faults || [];
        if (state.currentFile && !state.faults.some((f) => f.file === state.currentFile)) {
          state.currentFile = null;
        }
        renderList();
        if (!state.loaded) { state.loaded = true; setStatus('已连接, 记录已加载'); }
        break;
      case 'fault_saved':
        state.currentFile = msg.file;
        state.dirty = false;
        state.aiAt = msg.updatedAt || state.aiAt;
        setStatus('已保存: ' + fmtTime(msg.updatedAt));
        break;
      case 'fault_deleted':
        clearForm();
        setStatus(msg.ok ? '记录已删除' : '删除失败');
        break;
      case 'fault_analyze_start':
        setStatus('AI 分析中... (可能需数十秒)');
        break;
      case 'fault_analyze_done':
        onAnalyzeDone(msg);
        break;
      default:
        break;
    }
  }

  function onOpen() {
    wsSend({ type: 'fault_list' });
  }

  function onShow() {
    // 切到本页时刷新列表
    if (typeof ws !== 'undefined' && ws && ws.readyState === 1) wsSend({ type: 'fault_list' });
  }

  function setConnected(v) {
    // 故障记录不依赖飞控串口连接, 仅提示服务器(WebSocket)状态
    if (!state.loaded) setStatus(v ? '已连接' : '未连接飞控(记录功能仍可用)');
  }

  function init() {
    if (!$('viewFault')) return;
    $('ftNew').onclick = () => {
      if (state.dirty && !confirm('当前记录有未保存的修改，新建将丢失。继续？')) return;
      clearForm();
    };
    $('ftRefresh').onclick = () => wsSend({ type: 'fault_list' });
    $('ftSave').onclick = () => { saveRecord(false); };
    $('ftDelete').onclick = deleteCurrent;
    $('ftExport').onclick = exportMarkdown;
    $('ftSearch').addEventListener('input', renderList);
    $('ftAiRun').onclick = runAi;
    $('ftAiSettingsBtn').onclick = () => { if (typeof AIConfig !== 'undefined') AIConfig.openSettings(); };
    if (typeof AIConfig !== 'undefined') AIConfig.subscribe(updateAiStatus);
    // 未保存提醒
    for (const id of ['ftTitle', 'ftAircraft', 'ftFirmware', 'ftTags', 'ftSymptom',
      'ftContext', 'ftCause', 'ftFix', 'ftAiReport']) {
      $(id).addEventListener('input', () => markDirty(true));
    }
    $('ftSeverity').addEventListener('change', () => markDirty(true));
    $('ftStatus').addEventListener('change', () => markDirty(true));
    updateButtons();
    if (typeof window !== 'undefined' && window.addEventListener) {
      window.addEventListener('beforeunload', (e) => {
        if (state.dirty) { e.preventDefault(); e.returnValue = ''; }
      });
    }
  }

  window.Fault = { init, handleMessage, onOpen, onShow, setConnected };
  if (typeof document !== 'undefined') window.Fault.init();
})();
