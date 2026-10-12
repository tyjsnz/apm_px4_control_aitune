// 使用统计客户端 (匿名): 只上报随机 UUID 事件, 不含任何个人信息/飞控数据
//   - 匿名 ID 存 localStorage('fy_anon_id'), 仅用于按天 UV 去重 (服务端加盐哈希, 不可反推)
//   - 用户可关闭: localStorage.setItem('fy_stats_off','1') 后刷新页面
//   - 服务端另有 STATS_DISABLED=1 总开关; 未收到响应不影响业务
(function () {
  const ID_KEY = 'fy_anon_id';
  const OFF_KEY = 'fy_stats_off';

  let enabled = true;
  let anonId = '';
  try { enabled = localStorage.getItem(OFF_KEY) !== '1'; } catch (e) { enabled = true; }

  if (enabled) {
    try {
      anonId = localStorage.getItem(ID_KEY) || '';
      if (!/^[\w-]{8,64}$/.test(anonId)) {
        anonId = (window.crypto && crypto.randomUUID)
          ? crypto.randomUUID()
          : 'anon-' + Date.now().toString(36) + '-' + Math.random().toString(36).slice(2, 12);
        localStorage.setItem(ID_KEY, anonId);
      }
    } catch (e) { anonId = ''; }   // 隐私模式等读写受限: 无匿名 ID, 服务端只计 PV
  }

  // 页内只发一次的事件 (page_view / fc_connect); onetouch 每次写入都计
  const once = {};

  window.Stats = {
    /** @param {string} name  page_view | fc_connect | onetouch */
    event(name, extra) {
      if (!enabled) return;
      try {
        const w = window.ws;
        if (!w || w.readyState !== 1) return;   // WS 未就绪: 静默丢弃, 不重试
        if (name !== 'onetouch') {
          if (once[name]) return;
          once[name] = true;
        }
        const m = { type: 'stats_event', event: name, anonId };
        if (extra && typeof extra === 'object') Object.assign(m, extra);
        w.send(JSON.stringify(m));
      } catch (e) { /* 统计失败绝不影响业务 */ }
    },
  };
})();
