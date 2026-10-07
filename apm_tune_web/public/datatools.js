// 备份恢复 + 日志分析 (前端)
// AI 配置为全局共享 (见 aiconfig.js); 此处仅负责日志分析与备份恢复
(function () {
  const $ = (id) => document.getElementById(id);
  const state = {
    connected: false,
    fcLogs: [],
    localLogs: [],
    lastReport: '',
    analyzing: false,
    busy: false,
    pendingAnalyze: false,
    chartsByFile: {},
    chartOrder: [],
    activeChartFile: null,
    backups: [],
  };

  const fmtSize = (n) => {
    if (n === undefined || n === null) return '--';
    if (n < 1024) return n + ' B';
    if (n < 1024 * 1024) return (n / 1024).toFixed(1) + ' KB';
    return (n / 1024 / 1024).toFixed(1) + ' MB';
  };
  const fmtTime = (s) => {
    if (!s) return '--';
    const m = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/.exec(s);
    if (m) return `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}:${m[6]}`;
    return s;
  };
  const fmtUtc = (t) => (t ? new Date(t * 1000).toISOString().replace('T', ' ').slice(0, 19) : 'Unknown');

  const wsSend = (obj) => {
    if (typeof ws !== 'undefined' && ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
    else alert('WebSocket 未连接');
  };

  function report(el, text, clear) {
    const n = $(el);
    if (!n) return;
    if (clear) n.textContent = '';
    if (text) {
      n.textContent += text.endsWith('\n') ? text : text + '\n';
      n.scrollTop = n.scrollHeight;
    }
  }

  function updateButtons() {
    const c = state.connected;
    const b = state.busy;
    if ($('btnParamBackup')) $('btnParamBackup').disabled = !c || b;
    if ($('btnLogRefresh')) $('btnLogRefresh').disabled = !c || b;
    if ($('btnLogDownload')) $('btnLogDownload').disabled = !c || b;
    if ($('btnLogDownloadAnalyze')) $('btnLogDownloadAnalyze').disabled = !c || b || state.analyzing;
    if ($('btnLogErase')) $('btnLogErase').disabled = !c || b;
    if ($('btnLogAnalyze')) $('btnLogAnalyze').disabled = state.analyzing;
    if ($('backupStatus')) $('backupStatus').textContent = c ? '已连接' : '未连接';
  }

  // ==================== 备份恢复 ====================
  function renderBackups(list) {
    state.backups = list || [];
    list = state.backups;
    const tb = $('backupTableBody');
    tb.innerHTML = '';
    if (!list || !list.length) {
      tb.innerHTML = '<tr><td colspan="6" class="dt-empty">暂无备份</td></tr>';
      $('backupStatus').textContent = '0 个备份';
      return;
    }
    for (const b of list) {
      const tr = document.createElement('tr');
      const countCell = b.count ? String(b.count) : '<span class="dt-zero" title="该备份不含参数，无法恢复">0</span>';
      tr.innerHTML = `<td>${b.type || ''}</td><td>${fmtTime(b.timestamp)}</td>`
        + `<td>${countCell}</td><td>${b.fcType || ''}</td><td class="dt-mono">${b.file}</td>`;
      const td = document.createElement('td');

      const a = document.createElement('a');
      a.href = '/backups/' + encodeURIComponent(b.file);
      a.download = b.file;
      a.textContent = '下载';
      a.className = 'dt-link';
      td.appendChild(a);

      const btnR = document.createElement('button');
      btnR.textContent = '恢复到飞控';
      btnR.disabled = !state.connected || !b.count;
      if (!b.count) btnR.title = '该备份不含参数(0 项)，无法恢复';
      btnR.onclick = () => {
        if (!b.count) { alert(`备份 ${b.file} 不含任何参数(0 项)，无法恢复。\n可能原因: 备份时参数读取失败。`); return; }
        if (!confirm(`将把备份 ${b.file} 中的 ${b.count} 个参数写回飞控，覆盖当前值。\n继续？`)) return;
        state.busy = true; updateButtons();
        report('backupReport', `开始恢复 ${b.file} ...`, true);
        wsSend({ type: 'param_restore', file: b.file });
      };
      td.appendChild(btnR);

      const btnD = document.createElement('button');
      btnD.textContent = '删除';
      btnD.onclick = () => {
        if (!confirm('删除备份 ' + b.file + ' ?')) return;
        wsSend({ type: 'backup_delete', file: b.file });
      };
      td.appendChild(btnD);

      tr.appendChild(td);
      tb.appendChild(tr);
    }
    const emptyCount = list.filter((b) => !b.count).length;
    $('backupStatus').textContent = `${list.length} 个备份`
      + (emptyCount ? `, 其中 ${emptyCount} 个为空(不可恢复)` : '');
  }

  // ==================== 日志 ====================
  function renderFcLogs(logs) {
    state.fcLogs = logs || [];
    const tb = $('fcLogTableBody');
    tb.innerHTML = '';
    if (!state.fcLogs.length) {
      tb.innerHTML = '<tr><td colspan="4" class="dt-empty">飞控上暂无日志</td></tr>';
      $('logStatus').textContent = '0 条';
      return;
    }
    for (const l of state.fcLogs) {
      const tr = document.createElement('tr');
      const tdC = document.createElement('td');
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.dataset.id = l.id;
      tdC.appendChild(cb);
      tr.appendChild(tdC);
      tr.insertAdjacentHTML('beforeend',
        `<td>${l.id}</td><td>${fmtSize(l.size)}</td><td>${fmtUtc(l.timeUtc)}</td>`);
      tb.appendChild(tr);
    }
    $('logStatus').textContent = `${state.fcLogs.length} 条`;
  }

  function renderLocalLogs(logs) {
    state.localLogs = logs || [];
    const tb = $('localLogTableBody');
    tb.innerHTML = '';
    if (!state.localLogs.length) {
      tb.innerHTML = '<tr><td colspan="3" class="dt-empty">暂无本地日志</td></tr>';
      return;
    }
    for (const l of state.localLogs) {
      const tr = document.createElement('tr');
      const tdR = document.createElement('td');
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.dataset.file = l.file;
      cb.addEventListener('change', updateButtons);
      tdR.appendChild(cb);
      tdR.appendChild(document.createTextNode(' '));
      const nameSpan = document.createElement('span');
      nameSpan.className = 'dt-mono';
      nameSpan.textContent = l.file;
      tdR.appendChild(nameSpan);
      // 点击行也可切换选中
      tr.onclick = (e) => {
        if (e.target.tagName === 'BUTTON' || e.target.tagName === 'A' || e.target === cb) return;
        cb.checked = !cb.checked;
      };
      tr.appendChild(tdR);

      tr.insertAdjacentHTML('beforeend', `<td>${fmtSize(l.size)}</td>`);
      const td = document.createElement('td');
      const a = document.createElement('a');
      a.href = '/logs/' + encodeURIComponent(l.file);
      a.download = l.file;
      a.textContent = '下载到电脑';
      a.className = 'dt-link';
      td.appendChild(a);
      const btnA = document.createElement('button');
      btnA.textContent = '分析';
      btnA.onclick = () => analyzeFiles([l.file]);
      td.appendChild(btnA);
      const btnD = document.createElement('button');
      btnD.textContent = '删除';
      btnD.onclick = () => {
        if (!confirm('删除本地日志 ' + l.file + ' ?')) return;
        wsSend({ type: 'log_local_delete', file: l.file });
      };
      td.appendChild(btnD);
      tr.appendChild(td);
      tb.appendChild(tr);
    }
  }

  function selectedLocalFiles() {
    return [...document.querySelectorAll('#localLogTableBody input[type="checkbox"]:checked')]
      .map((c) => c.dataset.file);
  }

  function checkedFcIds() {
    return [...document.querySelectorAll('#fcLogTableBody input[type="checkbox"]:checked')]
      .map((c) => Number(c.dataset.id));
  }

  function downloadSelectedFcLogs(analyzeAfter) {
    const ids = checkedFcIds();
    if (!ids.length) { alert('请先勾选要下载的日志'); return; }
    const tip = analyzeAfter
      ? `将下载 ${ids.length} 个日志并自动进行 AI 分析。\n继续？`
      : `将下载 ${ids.length} 个日志，可能需要较长时间。\n继续？`;
    if (!confirm(tip)) return;
    state.pendingAnalyze = !!analyzeAfter;
    state.busy = true; updateButtons();
    report('logReport', `开始下载 ${ids.length} 个日志: ${ids.join(', ')}`, true);
    wsSend({ type: 'log_download', ids });
  }

  function analyzeFiles(files) {
    if (!files || !files.length) { alert('请先选择要分析的日志'); return; }
    // 未配置全局 AI 时先提示配置(可直接去设置, 或仅生成摘要)
    if (typeof AIConfig !== 'undefined' && !AIConfig.isConfigured()) {
      const go = confirm('尚未配置可用的全局 AI。\n\n点「确定」打开「AI 设置」（设置一次全局共用）；\n点「取消」则只生成数据摘要，不做 AI 分析。');
      if (go) { AIConfig.openSettings(); return; }
    }
    state.analyzing = true; updateButtons();
    state.chartsByFile = {}; state.chartOrder = []; state.activeChartFile = null;
    if ($('logCharts')) $('logCharts').innerHTML = '';
    if ($('btnChartExportHtml')) $('btnChartExportHtml').disabled = true;
    $('logAnalyzeStatus').textContent = '解析中...';
    report('logReport', `分析 ${files.length} 个日志: ${files.join(', ')}`, true);
    // AI 配置由服务端全局持有, 此处无需携带
    wsSend({ type: 'log_analyze', files });
  }

  // 统一入口: 优先分析本地勾选; 否则把飞控勾选的日志"下载并分析"
  function analyzeSelected() {
    const local = selectedLocalFiles();
    if (local.length) { analyzeFiles(local); return; }
    if (checkedFcIds().length) { downloadSelectedFcLogs(true); return; }
    alert('请先勾选飞控日志(将自动下载并分析)，或勾选已下载的本地日志');
  }

  // ==================== AI 设置 (全局) ====================
  function updateAiStatus() {
    const el = $('aiStatus');
    if (!el) return;
    if (typeof AIConfig === 'undefined') { el.textContent = 'AI: 未配置'; return; }
    el.textContent = AIConfig.statusText();
    el.classList.toggle('configured', AIConfig.isConfigured());
  }

  // ==================== 消息 ====================
  function handleMessage(msg) {
    switch (msg.type) {
      case 'log_charts':
        state.chartsByFile[msg.file] = msg.charts || [];
        if (!state.chartOrder.includes(msg.file)) state.chartOrder.push(msg.file);
        renderChartsUI();
        break;

      // 备份
      case 'backup_list':
        renderBackups(msg.backups);
        break;
      case 'param_backup_progress':
        $('backupStatus').textContent = `备份中 ${msg.count}/${msg.total || '?'}`;
        break;
      case 'param_backup_done':
        state.busy = false; updateButtons();
        if (msg.ok) report('backupReport', `✅ 备份完成: ${msg.file} (${msg.count}/${msg.total || '?'} 项)`
          + (msg.fromCache ? ' [取自参数表缓存]' : ''), false);
        else report('backupReport', `❌ 备份失败: ${msg.error || ''}`, false);
        wsSend({ type: 'backup_list' });
        break;
      case 'param_restore_progress':
        $('backupStatus').textContent = `恢复中 ${msg.done}/${msg.total}`;
        if (!msg.ok) report('backupReport', `  ❌ ${msg.name} 写入失败`);
        break;
      case 'param_restore_done':
        state.busy = false; updateButtons();
        if (msg.empty) report('backupReport', `❌ 取消恢复: 备份 ${msg.file} 不含任何参数(0 项)`, false);
        else report('backupReport', `恢复完成: 成功 ${msg.success}` +
          (msg.failed.length ? `，失败 ${msg.failed.length} (${msg.failed.join(', ')})` : ''), false);
        break;

      // 日志
      case 'log_list':
        renderFcLogs(msg.logs);
        break;
      case 'log_download_start':
        report('logReport', `下载日志 ${msg.id} ...`);
        break;
      case 'log_download_progress':
        $('logStatus').textContent = `日志${msg.id}: ${fmtSize(msg.current)}/${fmtSize(msg.total)}`;
        break;
      case 'log_download_saved':
        report('logReport', `  ✅ 日志 ${msg.id} 已保存: ${msg.file} (${fmtSize(msg.size)})`);
        break;
      case 'log_download_error':
        report('logReport', `  ❌ 日志 ${msg.id} 下载失败: ${msg.error}`);
        break;
      case 'log_download_done':
        state.busy = false; updateButtons();
        report('logReport', `下载完成: 成功 ${msg.saved.length}，失败 ${msg.failed.length}`, false);
        wsSend({ type: 'log_local_list' });
        if (state.pendingAnalyze) {
          state.pendingAnalyze = false;
          const files = msg.saved.map((s) => s.file);
          if (files.length) analyzeFiles(files);
          else report('logReport', '⚠ 没有成功下载的日志，无法分析', false);
        }
        break;
      case 'log_local_list':
        renderLocalLogs(msg.logs);
        break;
      case 'log_analyze_error':
        report('logReport', `  ❌ ${msg.file}: ${msg.error}`, false);
        break;
      case 'log_erase_done':
        report('logReport', (msg.ok ? '✅ ' : '⚠ ') + (msg.text || ''), false);
        wsSend({ type: 'log_list' });
        break;
      case 'log_analyze_summary':
        state.lastReport = msg.summary || '';
        report('logReport', msg.summary || '', false);
        $('logAnalyzeStatus').textContent = '已生成摘要，等待 AI...';
        $('btnLogReportExport').disabled = false;
        break;      case 'log_analyze_progress':
        $('logAnalyzeStatus').textContent = 'AI 分析中...';
        report('logReport', '\n=== AI 飞行报告 ===\n');
        break;
      case 'log_analyze_done':
        state.analyzing = false; updateButtons();
        if (msg.report) {
          if (msg.ai) {
            // 摘要已输出, 这里追加 AI 报告
            report('logReport', msg.report, false);
          }
          state.lastReport = (state.lastReport ? state.lastReport + '\n\n' : '') + (msg.ai ? msg.report : '');
        }
        $('btnLogReportExport').disabled = false;
        if (msg.error) {
          $('logAnalyzeStatus').textContent = 'AI 分析失败';
          report('logReport', '\n⚠ AI 分析失败: ' + msg.error + ' (已保留数据摘要)', false);
        } else {
          $('logAnalyzeStatus').textContent = msg.ai ? '✅ 报告完成' : '✅ 摘要完成(未启用AI)';
        }
        break;
      default:
        break;
    }
  }

  // ==================== 传感器图表 ====================
  function renderChartsUI() {
    const box = $('logCharts');
    if (!box || typeof Charts === 'undefined') return;
    box.innerHTML = '';
    if (!state.chartOrder.length) return;
    let active = state.activeChartFile;
    if (!active || !state.chartsByFile[active]) { active = state.chartOrder[0]; state.activeChartFile = active; }

    if (state.chartOrder.length > 1) {
      const tabs = document.createElement('div');
      tabs.className = 'chart-tabs';
      for (const f of state.chartOrder) {
        const b = document.createElement('button');
        b.textContent = f;
        b.className = 'chart-tab' + (f === active ? ' active' : '');
        b.onclick = () => { state.activeChartFile = f; renderChartsUI(); };
        tabs.appendChild(b);
      }
      box.appendChild(tabs);
    }
    const holder = document.createElement('div');
    box.appendChild(holder);
    Charts.render(holder, state.chartsByFile[active] || []);
    if ($('btnChartExportHtml')) $('btnChartExportHtml').disabled = false;
  }

  function escapeHtml(s) {
    return String(s || '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  }

  const CHART_EXPORT_CSS = `
    body{background:#0d1117;color:#e6edf3;font-family:-apple-system,Segoe UI,Helvetica,Arial,sans-serif;padding:24px;max-width:1100px;margin:0 auto;}
    h1{color:#58a6ff;font-size:22px;} h2{color:#58a6ff;font-size:18px;margin-top:28px;}
    pre{white-space:pre-wrap;background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;font-size:12.5px;line-height:1.6;}
    .chart-card{border:1px solid #30363d;border-radius:8px;margin:14px 0;padding:10px 12px;background:#161b22;}
    .chart-card h5{margin:0 0 6px;font-size:13px;color:#8b949e;font-weight:600;}
    .chart-svg-wrap{width:100%;}
    .chart-svg{display:block;width:100%;height:auto;}
    .chart-legend{display:flex;flex-wrap:wrap;gap:12px;font-size:12px;color:#8b949e;margin-top:6px;}
    .chart-legend-item{display:inline-flex;align-items:center;gap:5px;}
    .chart-legend-item i{display:inline-block;width:10px;height:10px;border-radius:2px;}
    .chart-empty{color:#6e7681;padding:14px;text-align:center;}`;

  function exportHtml() {
    const report = $('logReport') ? $('logReport').textContent : '';
    // 汇总所有日志的图表
    const tmp = document.createElement('div');
    let chartsHtml = '';
    for (const f of state.chartOrder) {
      const h = document.createElement('h2');
      h.textContent = '传感器图表 · ' + f;
      tmp.innerHTML = '';
      tmp.appendChild(h);
      const holder = document.createElement('div');
      tmp.appendChild(holder);
      if (typeof Charts !== 'undefined') Charts.render(holder, state.chartsByFile[f] || []);
      chartsHtml += tmp.innerHTML;
    }
    const html = '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
      + '<meta name="viewport" content="width=device-width, initial-scale=1.0">'
      + '<title>AP Tune 飞行报告</title><style>' + CHART_EXPORT_CSS + '</style></head><body>'
      + '<h1>AP Tune 飞行报告</h1>'
      + '<pre>' + escapeHtml(report) + '</pre>'
      + chartsHtml
      + '</body></html>';
    const blob = new Blob([html], { type: 'text/html' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `flight_report_${Date.now()}.html`;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  function exportReport() {
    if (!state.lastReport) return;
    const blob = new Blob([state.lastReport], { type: 'text/markdown' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `flight_report_${Date.now()}.md`;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  function init() {
    if ($('btnParamBackup')) {
      $('btnParamBackup').onclick = () => {
        if (!state.connected) { alert('请先连接飞控'); return; }
        if (!confirm('读取飞控全部参数并备份？(约需数秒)')) return;
        state.busy = true; updateButtons();
        report('backupReport', '开始读取并备份全部参数...', true);
        wsSend({ type: 'param_backup' });
      };
    }
    if ($('btnBackupRefresh')) $('btnBackupRefresh').onclick = () => wsSend({ type: 'backup_list' });
    if ($('btnLogRefresh')) $('btnLogRefresh').onclick = () => {
      if (!state.connected) { alert('请先连接飞控'); return; }
      $('logStatus').textContent = '读取中...';
      wsSend({ type: 'log_list' });
    };
    if ($('btnLogDownload')) $('btnLogDownload').onclick = () => downloadSelectedFcLogs(false);
    if ($('btnLogDownloadAnalyze')) $('btnLogDownloadAnalyze').onclick = () => downloadSelectedFcLogs(true);
    if ($('btnLogErase')) $('btnLogErase').onclick = () => {
      if (!state.connected) { alert('请先连接飞控'); return; }
      if (!confirm('将清除飞控上所有数据闪存日志，此操作不可恢复！\n确认？')) return;
      state.busy = true; updateButtons();
      report('logReport', '正在清除飞控日志...', true);
      wsSend({ type: 'log_erase' });
    };
    if ($('btnLogLocalRefresh')) $('btnLogLocalRefresh').onclick = () => wsSend({ type: 'log_local_list' });
    if ($('btnLogAnalyze')) $('btnLogAnalyze').onclick = analyzeSelected;
    if ($('btnLogReportExport')) $('btnLogReportExport').onclick = exportReport;
    if ($('btnChartExportHtml')) $('btnChartExportHtml').onclick = exportHtml;
    if ($('aiSettingsBtn')) $('aiSettingsBtn').onclick = () => { if (typeof AIConfig !== 'undefined') AIConfig.openSettings(); };
    if ($('aiClearStoredKey')) $('aiClearStoredKey').onclick = () => {
      if (typeof AIConfig === 'undefined') return;
      if (!confirm('清除本机浏览器中保存的 API Key？')) return;
      AIConfig.clearStoredKey();
    };
    if (typeof AIConfig !== 'undefined') AIConfig.subscribe(updateAiStatus);

    updateButtons();
  }

  function onOpen() {
    wsSend({ type: 'backup_list' });
    wsSend({ type: 'log_local_list' });
  }

  function setConnected(v) {
    state.connected = !!v;
    updateButtons();
    // 连接状态变化后重新渲染备份表(恢复按钮的可用状态依赖它)
    if (state.backups.length) renderBackups(state.backups);
  }

  window.DataTools = { init, handleMessage, setConnected, onOpen };
  if (typeof document !== 'undefined') window.DataTools.init();
})();
