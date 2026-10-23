// 浏览器端传输层: Web Serial 直连飞控 + 服务器 WS (AI/故障/文件存储)
// 对外暴露 window.ws —— 与原生 WebSocket 接口兼容 (readyState / onopen / onmessage / send)
// 消息协议与原 server.js 完全一致:
//   - 串口/飞控类消息 (connect/set_param/ot_*/log_* 等) → 浏览器内 FcOps 本地处理
//   - AI/故障速查/备份列表等消息 → 转发给远端服务器 WebSocket
(function () {
  'use strict';

  const WS_PROTO = location.protocol === 'https:' ? 'wss:' : 'ws:';

  // 本地处理的消息类型 (其余转发服务器)
  const LOCAL_TYPES = new Set([
    'list_ports', 'connect', 'disconnect', 'mavlink_send',
    'request_params', 'set_param', 'reboot', 'send_heartbeat',
    'ot_read', 'ot_write', 'ot_cal', 'ot_accel_cal_start',
    'ot_accel_cal_confirm', 'ot_accel_cal_abort', 'ot_spin_start',
    'ot_motor_test', 'ot_motor_stop',
    'prearm_check',
    'backup_restore', 'param_backup', 'param_restore',
    'log_list', 'log_download', 'log_erase',
    'mav_recent',
    'sensor_check_start', 'sensor_check_stop',
  ]);

  function describePort(port) {
    const info = port.getInfo ? port.getInfo() : {};
    const vid = info.usbVendorId !== undefined ? info.usbVendorId.toString(16).padStart(4, '0') : '????';
    const pid = info.usbProductId !== undefined ? info.usbProductId.toString(16).padStart(4, '0') : '????';
    return `usb:${vid}:${pid}`;
  }

  // ==================== 服务器文件存储 (备份/日志落在服务器磁盘) ====================
  const httpStore = {
    async saveText(name, text) {
      const r = await fetch('/api/backups', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ name, content: text }),
      });
      if (!r.ok) throw new Error('备份上传服务器失败 (HTTP ' + r.status + ')');
    },
    async saveBin(name, data) {
      const r = await fetch('/api/logs?name=' + encodeURIComponent(name), {
        method: 'POST',
        headers: { 'content-type': 'application/octet-stream' },
        body: data,
      });
      if (!r.ok) throw new Error('日志上传服务器失败 (HTTP ' + r.status + ')');
    },
    async loadText(name) {
      const r = await fetch('/api/backups/' + encodeURIComponent(name));
      if (!r.ok) throw new Error('读取服务器备份失败 (HTTP ' + r.status + ')');
      return r.text();
    },
  };

  // ==================== 传输层 ====================
  class FcTransport {
    constructor() {
      this.readyState = 0;          // 0=CONNECTING 1=OPEN (与 WebSocket 常量一致)
      this.onopen = null;
      this.onmessage = null;
      this.onclose = null;

      this._ops = null;             // FcOps 实例 (模块加载完成后可用)
      this._opsReady = false;
      this._opsQueue = [];          // 模块就绪前排队的本地消息
      this._pending = [];           // onmessage 绑定前收到的服务器消息

      this._server = null;          // 到远端服务器的 WebSocket
      this._outQueue = [];          // 服务器 WS 未就绪时排队的转发消息

      this._port = null;            // 已打开的 Web Serial 端口
      this._writer = null;
      this._reader = null;
      this._granted = [];           // 已授权端口 (navigator.serial.getPorts())
      this._pickedPort = null;      // requestPort() 刚选择的端口

      this._initModules();
      this._connectServer();
    }

    async _initModules() {
      try {
        const { FcOps } = await import('/lib/fcops.js');
        this._ops = new FcOps({
          emit: (obj) => this._dispatch(obj),
          store: httpStore,
          log: (...a) => console.log('[fc]', ...a),
        });
        this._opsReady = true;
        const q = this._opsQueue.splice(0);
        for (const m of q) this._handleLocal(m);
      } catch (err) {
        console.error('飞控操作模块加载失败:', err);
        this._dispatch({ type: 'error', message: '飞控操作模块加载失败: ' + err.message });
      }
    }

    _connectServer() {
      try {
        this._server = new WebSocket(`${WS_PROTO}//${location.host}`);
      } catch (err) {
        return this._retryServer();
      }
      this._server.onopen = () => {
        this.readyState = 1;
        const q = this._outQueue.splice(0);
        for (const m of q) this._server.send(JSON.stringify(m));
        if (this.onopen) this.onopen({});
      };
      this._server.onmessage = (e) => this._dispatch(JSON.parse(e.data));
      this._server.onclose = () => {
        this.readyState = 0;
        this._retryServer();
      };
      this._server.onerror = () => { try { this._server.close(); } catch (e) { /* ignore */ } };
    }

    _retryServer() {
      setTimeout(() => this._connectServer(), 2000);
    }

    /** 输出消息到页面 (等价原服务器 broadcast/send 到本页) */
    _dispatch(obj) {
      const data = JSON.stringify(obj);
      if (this.onmessage) this.onmessage({ data });
      else this._pending.push(data);
    }

    // ---------- WebSocket 兼容接口 ----------
    send(raw) {
      let msg;
      try { msg = JSON.parse(raw); } catch (e) { return; }

      // 选择新串口必须在用户手势内同步调用 requestPort()
      if (msg.type === 'connect' && msg.path === 'webserial:pick') {
        this._pickAndConnect(Number(msg.baud) || 115200);
        return;
      }
      if (LOCAL_TYPES.has(msg.type)) {
        if (!this._opsReady) { this._opsQueue.push(msg); return; }
        this._handleLocal(msg);
        return;
      }
      // 其余转发服务器
      if (this._server && this.readyState === 1) this._server.send(raw);
      else this._outQueue.push(msg);
    }

    close() {
      this.readyState = 3;
      this._disconnect();
      if (this._server) { try { this._server.close(); } catch (e) { /* ignore */ } }
    }

    // ---------- 本地消息处理 ----------
    _pickAndConnect(baud) {
      if (!('serial' in navigator)) {
        this._dispatch({
          type: 'error',
          message: '无法使用串口: 浏览器不支持或已隐藏 Web Serial。'
            + '请用 Chrome/Edge 并通过 https:// 或 http://localhost 打开本页 (http://IP 访问时浏览器会禁用串口 API)',
        });
        return;
      }
      let promise;
      try {
        promise = navigator.serial.requestPort();  // 必须在用户手势内同步调用
      } catch (err) {
        this._dispatch({ type: 'error', message: '选择串口失败: ' + err.message });
        return;
      }
      promise.then((port) => {
        this._pickedPort = port;
        this._openPort(port, 'webserial:pick', baud);
      }).catch((err) => {
        this._dispatch({ type: 'error', message: '选择串口失败: ' + (err && err.message || err) });
      });
    }

    async _handleLocal(msg) {
      switch (msg.type) {
        case 'list_ports': {
          let list = [];
          if ('serial' in navigator) {
            try { this._granted = await navigator.serial.getPorts(); } catch (e) { this._granted = []; }
            list = this._granted.map((p, i) => ({
              path: 'webserial:' + i,
              manufacturer: 'WebSerial',
              serialNumber: describePort(p),
            }));
          }
          list.push({ path: 'webserial:pick', manufacturer: '选择新串口…(弹出系统对话框)', serialNumber: '' });
          this._dispatch({ type: 'ports', ports: list });
          return;
        }
        case 'connect': {
          const token = String(msg.path || '');
          const baud = Number(msg.baud) || 115200;
          if (this._port) { this._dispatch({ type: 'error', message: '已连接，请先断开' }); return; }
          if (token === 'webserial:pick') { this._pickAndConnect(baud); return; }
          const m = /^webserial:(\d+)$/.exec(token);
          const port = m ? this._granted[Number(m[1])] : null;
          if (!port) { this._dispatch({ type: 'error', message: '串口无效，请点击"连接"重新选择' }); return; }
          await this._openPort(port, token, baud);
          return;
        }
        case 'disconnect': {
          await this._disconnect();
          return;
        }
        case 'mavlink_send': {
          if (this._port && Array.isArray(msg.data)) this._serialWrite(Uint8Array.from(msg.data));
          return;
        }
        default: {
          // 其余串口类消息交给 FcOps (与原服务器 handleClientMessage 协议一致)
          try { await this._ops.handle(msg); } catch (err) {
            this._dispatch({ type: 'error', message: err.message });
          }
          return;
        }
      }
    }

    // ---------- Web Serial 端口管理 ----------
    async _openPort(port, token, baud) {
      const label = token === 'webserial:pick'
        ? describePort(port)
        : (token + ' (' + describePort(port) + ')');
      try {
        await port.open({ baudRate: baud });
      } catch (err) {
        this._dispatch({ type: 'error', message: '打开串口失败: ' + (err && err.message || err) });
        return;
      }
      this._port = port;
      this._writer = port.writable.getWriter();
      this._ops.setWrite((buf) => this._serialWrite(buf));
      this._ops.attach({ path: label, baud });

      // 拔出检测
      this._onPortDisconnect = () => {
        this._dispatch({ type: 'error', message: '串口设备已断开' });
        this._cleanupPort().then(() => { if (this._ops) this._ops.detach(); });
      };
      if (port.addEventListener) port.addEventListener('disconnect', this._onPortDisconnect);

      this._readLoop(port);
    }

    async _readLoop(port) {
      while (port.readable && this._port === port) {
        const reader = port.readable.getReader();
        this._reader = reader;
        try {
          for (;;) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value && value.length && this._ops) this._ops.feed(value);
          }
        } catch (err) {
          if (this._port === port) {
            this._dispatch({ type: 'error', message: '串口读取出错: ' + (err && err.message || err) });
            this._cleanupPort().then(() => { if (this._ops) this._ops.detach(); });
          }
        } finally {
          try { reader.releaseLock(); } catch (e) { /* ignore */ }
          if (this._reader === reader) this._reader = null;
        }
      }
    }

    _serialWrite(data) {
      if (!this._writer) return;
      const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
      this._writer.write(bytes).catch(() => { /* 设备已拔出时由 disconnect 事件处理 */ });
    }

    async _cleanupPort() {
      const port = this._port;
      this._port = null;
      if (this._reader) { try { await this._reader.cancel(); } catch (e) { /* ignore */ } }
      if (this._writer) { try { this._writer.releaseLock(); } catch (e) { /* ignore */ } this._writer = null; }
      if (port) {
        try { if (port.removeEventListener && this._onPortDisconnect) port.removeEventListener('disconnect', this._onPortDisconnect); } catch (e) { /* ignore */ }
        try { await port.close(); } catch (e) { /* ignore */ }
      }
    }

    async _disconnect() {
      if (!this._port) return;
      await this._cleanupPort();
      if (this._ops) this._ops.detach();
    }
  }

  // ---------- onmessage 属性支持绑定前排队 ----------
  const proto = FcTransport.prototype;
  Object.defineProperty(proto, 'onmessage', {
    get() { return this._onmessage || null; },
    set(fn) {
      this._onmessage = fn;
      if (fn && this._pending.length) {
        const q = this._pending.splice(0);
        for (const data of q) fn({ data });
      }
    },
    configurable: true,
  });

  window.FcSerial = {
    supported() { return typeof navigator !== 'undefined' && 'serial' in navigator; },
  };
  window.ws = new FcTransport();
})();
