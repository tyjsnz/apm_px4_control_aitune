
// ==================== 参数定义 ====================
const PARAM_GROUPS = [
  {
    title: 'GUIDED/AUTO 导航控制',
    items: [
      { name: '*WPNAV_SPEED', desc: '水平最大导航速度', unit: 'cm/s', min: 1000, max: 2500, step: 100 },
      { name: '*WPNAV_ACCEL', desc: '水平加速度上限', unit: 'cm/s²', min: 500, max: 1500, step: 50 },
      { name: '*WPNAV_JERK', desc: '水平加加速度', unit: 'm/s³', min: 10, max: 30, step: 1 },
      { name: '*WPNAV_RADIUS', desc: '到达容差半径', unit: 'cm', min: 10, max: 500, step: 10 },
      { name: 'WPNAV_SPEED_UP', desc: '最大上升速度', unit: 'cm/s', min: 100, max: 500, step: 10 },
      { name: 'WPNAV_SPEED_DN', desc: '最大下降速度', unit: 'cm/s', min: 100, max: 400, step: 10 },
      { name: 'WPNAV_ACCEL_Z', desc: '垂直加速度上限', unit: 'cm/s²', min: 200, max: 800, step: 50 },
      { name: 'PSC_JERK_Z', desc: '垂直加加速度', unit: 'm/s³', min: 5, max: 20, step: 1 },
    ]
  },
  {
    title: '位置/速度控制环',
    items: [
      { name: '*PSC_POSXY_P', desc: '水平位置环增益', unit: '', min: 0.5, max: 3.0, step: 0.1 },
      { name: '*PSC_VELXY_P', desc: '水平速度环比例增益', unit: '', min: 1.0, max: 3.0, step: 0.1 },
      { name: 'PSC_VELXY_I', desc: '水平速度环积分增益', unit: '', min: 0.1, max: 2.0, step: 0.1 },
      { name: '*PSC_VELXY_FF', desc: '水平速度前馈', unit: '', min: 0.0, max: 1.0, step: 0.05 },
      { name: 'PSC_VELZ_FF', desc: '垂直速度前馈', unit: '', min: 0.0, max: 1.0, step: 0.05 },
      { name: 'PSC_ACCZ_P', desc: '垂直加速度环比例', unit: '', min: 0, max: 1.0, step: 0.01 },
      { name: 'PSC_ACCZ_I', desc: '垂直加速度环积分', unit: '', min: 0, max: 2.0, step: 0.01 },
    ]
  },
  {
    title: '电机与悬停基础',
    items: [
      { name: '*MOT_THST_HOVER', desc: '悬停油门估计值', unit: '', min: 0.05, max: 0.5, step: 0.01 },
      { name: 'MOT_HOVER_LEARN', desc: '悬停油门自学习(0/1/2)', unit: '', min: 0, max: 2, step: 1 },
      { name: 'MOT_SPIN_ARM', desc: '解锁怠速输出', unit: '', min: 0.0, max: 0.3, step: 0.01 },
      { name: 'MOT_SPIN_MIN', desc: '飞行最小输出', unit: '', min: 0.0, max: 0.3, step: 0.01 },
      { name: 'MOT_SPIN_MAX', desc: '最大电机输出', unit: '', min: 0.5, max: 1.0, step: 0.01 },
      { name: 'MOT_THST_EXPO', desc: '推力曲线指数', unit: '', min: 0.0, max: 1.0, step: 0.01 },
      { name: 'MOT_PWM_TYPE', desc: '输出协议(0:PWM/1-2:OneShot/3+:DShot)', unit: '', min: 0, max: 7, step: 1 },
      { name: 'MOT_PWM_MIN', desc: '电机输出最小PWM', unit: 'us', min: 750, max: 1250, step: 10 },
      { name: 'MOT_PWM_MAX', desc: '电机输出最大PWM', unit: 'us', min: 1750, max: 2250, step: 10 },
      { name: 'MOT_YAW_HEADROOM', desc: '偏航控制预留油门', unit: '', min: 0.0, max: 0.3, step: 0.01 },
    ]
  },
  {
    title: '起飞参数',
    items: [
      { name: '*TKOFF_THR_MAX', desc: '起飞阶段最大油门', unit: '', min: 0.5, max: 1.0, step: 0.01 },
      { name: '*TKOFF_SLEW_TIME', desc: '起飞油门爬升时间', unit: 's', min: 0.1, max: 3.0, step: 0.1 },
    ]
  },
  {
    title: 'EKF3 状态估计',
    items: [
      { name: 'AHRS_EKF_TYPE', desc: 'EKF类型(保持3)', unit: '', min: 3, max: 3, step: 1 },
      { name: 'EK3_ENABLE', desc: '启用EKF3', unit: '', min: 1, max: 1, step: 1 },
      { name: 'EK3_SRC1_POSXY', desc: '水平位置主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_VELXY', desc: '水平速度主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_POSZ', desc: '高度主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_VELZ', desc: '垂直速度主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_SRC1_YAW', desc: '航向主数据源', unit: '', min: 0, max: 5, step: 1 },
      { name: 'EK3_POS_M_NSE', desc: '位置测量噪声', unit: '', min: 0.01, max: 10.0, step: 0.01 },
      { name: 'EK3_VELD_M_NSE', desc: '速度测量噪声', unit: '', min: 0.01, max: 10.0, step: 0.01 },
      { name: 'EK3_ALT_M_NSE', desc: '高度测量噪声', unit: '', min: 0.01, max: 10.0, step: 0.01 },
    ]
  },
  {
    title: '安全保护',
    items: [
      { name: '*FS_GCS_ENABLE', desc: '地面站失联保护', unit: '', min: 0, max: 2, step: 1 },
      { name: 'FS_GCS_TIMEOUT', desc: 'GCS失联超时', unit: 's', min: 1, max: 30, step: 1 },
      { name: 'FS_THR_VALUE', desc: '油门失联保护阈值', unit: '', min: 0, max: 1000, step: 10 },
      { name: '*FENCE_ENABLE', desc: '地理围栏使能', unit: '', min: 0, max: 1, step: 1 },
      { name: 'FENCE_TYPE', desc: '围栏类型', unit: '', min: 0, max: 3, step: 1 },
      { name: '*FENCE_ALT_MAX', desc: '地理围栏最大高度', unit: 'm', min: 5, max: 120, step: 5 },
      { name: '*FENCE_RADIUS', desc: '地理围栏水平半径', unit: 'm', min: 10, max: 500, step: 10 },
      { name: '*RTL_ALT', desc: '返航高度', unit: 'm', min: 10, max: 120, step: 5 },
      { name: 'RTL_ALT_FINAL', desc: '返航后最终高度', unit: 'm', min: 0, max: 50, step: 1 },
      { name: 'DISARM_DELAY', desc: '降落后自动上锁延时', unit: 's', min: 0, max: 10, step: 1 },
      { name: 'FS_EKF_ACTION', desc: 'EKF异常保护动作', unit: '', min: 0, max: 3, step: 1 },
    ]
  },
  {
    title: 'AUTO 模式任务',
    items: [
      { name: 'WP_YAW_BEHAVIOR', desc: 'AUTO机头航向行为', unit: '', min: 0, max: 3, step: 1 },
      { name: 'LOITER_TIME', desc: '航点悬停时间', unit: 's', min: 0, max: 300, step: 1 },
      { name: 'TUNE', desc: '遥控器调参目标参数', unit: '', min: 0, max: 100, step: 1 },
      { name: 'TUNE_MIN', desc: '调参通道最小值', unit: 'cm/s', min: 100, max: 3000, step: 100 },
      { name: 'TUNE_MAX', desc: '调参通道最大值', unit: 'cm/s', min: 100, max: 5000, step: 100 },
    ]
  },
  {
    title: '传感器与滤波',
    items: [
      { name: 'AHRS_ORIENTATION', desc: '飞控安装方向', unit: '', min: 0, max: 31, step: 1 },
      { name: 'INS_GYRO_FILTER', desc: '陀螺仪低通滤波', unit: 'Hz', min: 10, max: 200, step: 5 },
      { name: 'INS_ACCEL_FILTER', desc: '加速度计低通滤波', unit: 'Hz', min: 5, max: 50, step: 1 },
      { name: 'INS_HNTCH_ENABLE', desc: '谐波陷波使能', unit: '', min: 0, max: 1, step: 1 },
      { name: 'INS_HNTCH_MODE', desc: '陷波工作模式', unit: '', min: 0, max: 3, step: 1 },
    ]
  },
];

// ==================== 状态管理 ====================
const WS_PROTO = (typeof location !== 'undefined' && location.protocol === 'https:') ? 'wss:' : 'ws:';
const ws = new WebSocket(`${WS_PROTO}//${location.host}`);
let paramCache = {};       // { paramId: value }
let currentCategory = 0;
let connected = false;
let connectedPort = null;

// ==================== 串口连接 ====================
function refreshPorts() {
  if (ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: 'list_ports' }));
}

let portListSignature = '';

function updatePortList(ports) {
  const sel = document.getElementById('portSelect');
  const keep = connectedPort || sel.value;
  const signature = ports.map((p) => `${p.path}|${p.manufacturer || ''}|${p.serialNumber || ''}`).join(',') + '|' + (connectedPort || '');
  if (signature === portListSignature && [...sel.options].some((o) => o.value === keep)) return;
  portListSignature = signature;
  sel.innerHTML = '<option value="">请选择串口</option>';
  for (const p of ports) {
    const opt = document.createElement('option');
    opt.value = p.path;
    let label = p.path;
    if (p.manufacturer) label += ` (${p.manufacturer})`;
    if (p.serialNumber) label += ` [${p.serialNumber}]`;
    if (p.path === connectedPort) label = `${p.path} (已连接)`;
    opt.textContent = label;
    sel.appendChild(opt);
  }
  if (keep) {
    if (![...sel.options].some((o) => o.value === keep)) {
      const opt = document.createElement('option');
      opt.value = keep;
      opt.textContent = keep + (keep === connectedPort ? ' (已连接)' : '');
      sel.appendChild(opt);
    }
    sel.value = keep;
  }
}

function connect() {
  const path = document.getElementById('portSelect').value;
  const baud = document.getElementById('baudInput').value;
  if (!path) return alert('请选择串口');
  ws.send(JSON.stringify({ type: 'connect', path, baud: parseInt(baud) }));
}

function disconnect() {
  ws.send(JSON.stringify({ type: 'disconnect' }));
}

function requestParams() {
  ws.send(JSON.stringify({ type: 'request_params' }));
}

function setParam(name, value) {
  // 去掉开头的 * 号获取实际参数名
  const paramName = name.replace(/^\*/, '');
  // 判断参数类型
  let paramType = 6; // 默认 FLOAT
  const item = findParamItem(name);
  if (item) {
    const range = getItemRange(item);
    if (range.step && range.step >= 1 && range.min === Math.floor(range.min)) {
      paramType = 5; // INT32
      value = Math.round(value);
    }
  }
  ws.send(JSON.stringify({
    type: 'set_param',
    paramId: paramName,
    paramValue: parseFloat(value),
    paramType
  }));
}

function findParamItem(name) {
  for (const group of PARAM_GROUPS) {
    for (const item of group.items) {
      if (item.name === name) return item;
    }
  }
  return null;
}

function getItemRange(item) {
  return { min: item.min || 0, max: item.max || 9999, step: item.step || 1 };
}

function rebootFC() {
  if (confirm('确认重启飞控？重启后需要重新连接。')) {
    ws.send(JSON.stringify({ type: 'reboot' }));
  }
}

// ==================== WebSocket 消息处理 ====================
ws.onopen = () => {
  console.log('WebSocket connected');
  refreshPorts();
  if (typeof AIConfig !== 'undefined') AIConfig.onOpen();
  if (typeof DataTools !== 'undefined') DataTools.onOpen();
  if (typeof Fault !== 'undefined') Fault.onOpen();
};

ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);

  if (msg.type === 'ports') {
    updatePortList(msg.ports);
  }

  if (msg.type === 'connected') {
    connected = true;
    connectedPort = msg.path;
    const portSel = document.getElementById('portSelect');
    if (![...portSel.options].some((o) => o.value === msg.path)) {
      const opt = document.createElement('option');
      opt.value = msg.path;
      opt.textContent = `${msg.path} (已连接)`;
      portSel.appendChild(opt);
    }
    portSel.value = msg.path;
    portSel.disabled = true;
    document.getElementById('baudInput').disabled = true;
    document.getElementById('status').textContent = `已连接 ${msg.path}`;
    document.getElementById('status').className = 'status-badge connected';
    document.getElementById('btnConnect').disabled = true;
    document.getElementById('btnDisconnect').disabled = false;
    document.getElementById('btnReboot').disabled = false;
    document.getElementById('btnRefreshParams').disabled = false;
    document.getElementById('paramSearch').disabled = false;
    document.getElementById('categoryFilter').disabled = false;
    document.getElementById('telemetryPanel').style.display = 'flex';
    renderCategories();
    if (typeof OneTouch !== 'undefined') OneTouch.setConnected(true);
    if (typeof DataTools !== 'undefined') DataTools.setConnected(true);
    if (typeof Fault !== 'undefined') Fault.setConnected(true);
    scheduleThrustCheck(600);
    requestParams();
  }

  if (msg.type === 'disconnected') {
    connected = false;
    connectedPort = null;
    document.getElementById('portSelect').disabled = false;
    document.getElementById('baudInput').disabled = false;
    document.getElementById('status').textContent = '未连接';
    document.getElementById('status').className = 'status-badge disconnected';
    document.getElementById('btnConnect').disabled = false;
    document.getElementById('btnDisconnect').disabled = true;
    document.getElementById('btnReboot').disabled = true;
    document.getElementById('btnRefreshParams').disabled = true;
    document.getElementById('paramSearch').disabled = true;
    document.getElementById('categoryFilter').disabled = true;
    document.getElementById('telemetryPanel').style.display = 'none';
    if (typeof OneTouch !== 'undefined') OneTouch.setConnected(false);
    if (typeof DataTools !== 'undefined') DataTools.setConnected(false);
    document.getElementById('paramPanel').innerHTML = `
      <div class="empty-state">
        <p>连接已断开</p>
        <p class="hint">请重新选择串口并连接</p>
      </div>`;
    renderThrustCheck();
  }

  if (msg.type === 'error') {
    alert('错误: ' + msg.message);
  }

  // 参数值更新
  if (msg.type === 'param_value') {
    paramCache[msg.paramId] = msg.paramValue;
    updateParamDisplay(msg.paramId, msg.paramValue);
    if (TC_PARAMS.includes(msg.paramId)) scheduleThrustCheck();
  }

  // 参数写入确认
  if (msg.type === 'param_set_sent') {
    paramCache[msg.paramId] = msg.paramValue;
    updateParamDisplay(msg.paramId, msg.paramValue);
    if (TC_PARAMS.includes(msg.paramId)) scheduleThrustCheck();
    // 闪烁提示
    const el = document.getElementById('val_' + msg.paramId);
    if (el) {
      el.classList.add('saved');
      setTimeout(() => el.classList.remove('saved'), 1000);
    }
  }

  // 遥测数据
  if (msg.type === 'global_position') {
    const lat = msg.lat / 1e7;
    const lon = msg.lon / 1e7;
    document.getElementById('telLat').textContent = `纬度: ${lat.toFixed(7)}`;
    document.getElementById('telLon').textContent = `经度: ${lon.toFixed(7)}`;
    document.getElementById('telAlt').textContent = `高度: ${msg.relativeAlt / 100}m`;
  }

  if (msg.type === 'vfr_hud') {
    document.getElementById('telSpeed').textContent = `速度: ${msg.groundspeed}m/s`;
  }

  if (msg.type === 'sys_status') {
    if (msg.voltageBattery) {
      document.getElementById('telBattery').textContent = `电池: ${(msg.voltageBattery / 1000).toFixed(1)}V`;
    }
    if (msg.load !== undefined) {
      document.getElementById('loadPercent').textContent = (msg.load / 100).toFixed(0);
    }
  }

  if (msg.type === 'attitude') {
    // 可以在这里显示姿态数据
  }

  if (msg.type === 'heartbeat') {
    if (msg.flightMode) {
      document.getElementById('flightMode').textContent = msg.flightMode + (msg.armed ? ' (已解锁)' : '');
    }
  }

  if (msg.type === 'reboot_sent') {
    console.log('重启指令已发送');
  }

  // 一键首飞相关消息
  if (typeof OneTouch !== 'undefined') OneTouch.handleMessage(msg);
  // 全局 AI 配置 (服务商列表)
  if (typeof AIConfig !== 'undefined') AIConfig.handleMessage(msg);
  // 备份恢复 / 日志分析相关消息
  if (typeof DataTools !== 'undefined') DataTools.handleMessage(msg);
  // 故障速查相关消息
  if (typeof Fault !== 'undefined') Fault.handleMessage(msg);
};

// ==================== UI 渲染 ====================
function renderCategories() {
  const list = document.getElementById('categoryList');
  const filter = document.getElementById('categoryFilter');

  // 更新分类下拉
  filter.innerHTML = '<option value="all">全部分类</option>';
  list.innerHTML = '';

  PARAM_GROUPS.forEach((g, gi) => {
    // 下拉选项
    const opt = document.createElement('option');
    opt.value = gi;
    opt.textContent = g.title;
    filter.appendChild(opt);

    // 侧边栏
    const cat = document.createElement('div');
    cat.className = 'category';
    cat.dataset.index = gi;
    cat.innerHTML = `<h3>${g.title}</h3>`;
    cat.onclick = () => scrollToCategory(gi);
    list.appendChild(cat);
  });

  // 默认选中第一个
  if (PARAM_GROUPS.length) {
    list.querySelector('.category').classList.add('active');
    currentCategory = 0;
    renderParams();
  }
}

function setActiveCategory(gi) {
  document.querySelectorAll('.category').forEach((c) => {
    c.classList.toggle('active', Number(c.dataset.index) === gi);
  });
  currentCategory = gi;
}

function scrollToCategory(gi) {
  setActiveCategory(gi);

  let el = document.getElementById('group-' + gi);
  if (!el) {
    // 目标分组被下拉框/搜索过滤掉了，重置筛选后重新渲染
    document.getElementById('categoryFilter').value = 'all';
    document.getElementById('paramSearch').value = '';
    renderParams();
    el = document.getElementById('group-' + gi);
  }
  if (!el) return;

  el.scrollIntoView({ behavior: 'smooth', block: 'start' });
  el.classList.remove('highlight');
  void el.offsetWidth; // 重启动画
  el.classList.add('highlight');
  setTimeout(() => el.classList.remove('highlight'), 1200);
}

// 滚动时同步左侧选中项
function syncActiveCategory() {
  const panel = document.getElementById('paramPanel');
  const groups = panel.querySelectorAll('.param-group');
  if (!groups.length) return;

  const top = panel.getBoundingClientRect().top;
  let active = groups[0];

  // 已滚到底部：最后一个分组不满一屏，始终选中它
  if (panel.scrollTop + panel.clientHeight >= panel.scrollHeight - 4) {
    active = groups[groups.length - 1];
  } else {
    for (const g of groups) {
      if (g.getBoundingClientRect().top - top <= 24) active = g;
      else break;
    }
  }
  setActiveCategory(Number(active.dataset.index));
}

function renderParams() {
  const panel = document.getElementById('paramPanel');
  const search = document.getElementById('paramSearch').value.toLowerCase();
  const catFilter = document.getElementById('categoryFilter').value;
  const starOnly = document.getElementById('starFilter').checked;

  panel.innerHTML = '';

  let totalCount = 0;
  let shownCount = 0;

  PARAM_GROUPS.forEach((group, gi) => {
    if (catFilter !== 'all' && parseInt(catFilter, 10) !== gi) return;

    let groupHasVisible = false;
    const groupDiv = document.createElement('div');
    groupDiv.className = 'param-group';
    groupDiv.id = 'group-' + gi;
    groupDiv.dataset.index = gi;

    const groupHeader = document.createElement('div');
    groupHeader.className = 'param-group-header';
    groupHeader.textContent = group.title;
    groupDiv.appendChild(groupHeader);

    for (const item of group.items) {
      totalCount++;
      const name = item.name;
      const namePlain = name.replace(/^\*/, '');
      const isStar = name.startsWith('*');

      // 过滤
      if (starOnly && !isStar) continue;
      if (search) {
        const matchName = namePlain.toLowerCase().includes(search) || name.toLowerCase().includes(search);
        const matchDesc = item.desc.toLowerCase().includes(search);
        if (!matchName && !matchDesc) continue;
      }

      shownCount++;
      groupHasVisible = true;

      const range = getItemRange(item);

      const row = document.createElement('div');
      row.className = 'param-row';
      row.dataset.paramName = namePlain;

      // 参数名
      const label = document.createElement('div');
      label.className = 'param-name';
      const starHtml = isStar ? '<span class="star">*</span>' : '';
      label.innerHTML = `${starHtml}${name}`;

      // 当前值
      const currentVal = paramCache[namePlain] !== undefined ? paramCache[namePlain] : '';
      const valInput = document.createElement('input');
      valInput.type = 'number';
      valInput.className = 'param-value';
      valInput.id = 'val_' + namePlain;
      valInput.step = range.step || 1;
      valInput.min = range.min;
      valInput.max = range.max;
      valInput.value = currentVal !== '' ? currentVal : (range.min || '');
      valInput.placeholder = '读取中...';
      valInput.disabled = !connected;

      // 单位
      const unitSpan = document.createElement('span');
      unitSpan.className = 'param-unit';
      unitSpan.textContent = item.unit || '';

      // 描述
      const desc = document.createElement('div');
      desc.className = 'param-hint';
      desc.textContent = item.desc;

      // 写入按钮
      const btn = document.createElement('button');
      btn.className = 'btn-write';
      btn.textContent = '写入';
      btn.onclick = () => {
        const v = parseFloat(valInput.value);
        if (isNaN(v)) return alert('请输入有效数值');
        if (v < range.min || v > range.max) {
          if (!confirm(`值 ${v} 超出推荐范围 [${range.min}, ${range.max}]，确定写入？`)) return;
        }
        setParam(name, v);
      };

      row.appendChild(label);
      row.appendChild(valInput);
      row.appendChild(unitSpan);
      row.appendChild(desc);
      row.appendChild(btn);
      groupDiv.appendChild(row);
    }

    if (groupHasVisible) {
      panel.appendChild(groupDiv);
    }
  });

  if (shownCount === 0) {
    panel.innerHTML = '<div class="empty-state"><p>没有找到匹配的参数</p></div>';
  }
}

function updateParamDisplay(paramId, value) {
  const el = document.getElementById('val_' + paramId);
  if (el) {
    el.value = value;
  }
}

// ==================== 推力线性化体检 ====================
const TC_PARAMS = [
  'MOT_THST_EXPO', 'MOT_THST_HOVER', 'MOT_HOVER_LEARN',
  'MOT_SPIN_ARM', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX',
  'MOT_PWM_MIN', 'MOT_PWM_MAX', 'MOT_PWM_TYPE',
];
const TC_PWM_TYPE = ['PWM(50/100Hz)', 'OneShot', 'OneShot125', 'DShot1200', 'DShot900', 'DShot600', 'DShot300', 'DShot150'];
let tcTimer = null;

const tcHas = (n) => paramCache[n] !== undefined;
const tcNum = (n) => Number(paramCache[n]);

function scheduleThrustCheck(delay) {
  clearTimeout(tcTimer);
  tcTimer = setTimeout(renderThrustCheck, delay === undefined ? 300 : delay);
}

function tcVerdict(rows) {
  const warns = rows.filter((r) => r.level === 'warn').length;
  const head = warns ? `发现 ${warns} 项需要关注（见上表）。` : '未发现需要关注的异常项。';
  if (!tcHas('MOT_THST_EXPO')) return head + ' 未读到 MOT_THST_EXPO，无法给出线性化结论。';
  const v = tcNum('MOT_THST_EXPO');
  let tail;
  if (v <= 0.2) {
    tail = `EXPO=${v} 很小，飞控几乎不做线性化补偿 → 可认为当前电调+电机+桨已较线性，MOT_THST_EXPO 无需再调。`;
  } else if (v < 0.45) {
    tail = `EXPO=${v} 属中等补偿 → 参数本身可保持，但建议做一次地面油门阶梯实测，确认低油门段无死区、无突跳。`;
  } else {
    tail = `EXPO=${v} 明显（官方按桨径 5寸0.55/10寸0.65/20寸0.75，属正常配置）→ 不能据此判断电调硬件不线性；若悬停附近易振荡、小油门修正不跟手、自动调参反复变化，再考虑调整 EXPO。`;
  }
  return head + tail + ' 本结论基于参数体检，真实推力线性度仍需油门阶梯实测确认。';
}

function renderThrustCheck() {
  const rowsEl = document.getElementById('tcRows');
  const sumEl = document.getElementById('tcSummary');
  const verdictEl = document.getElementById('tcVerdict');
  if (!rowsEl) return;

  if (!connected) {
    rowsEl.innerHTML = '';
    sumEl.textContent = '未连接';
    sumEl.className = 'tc-summary';
    verdictEl.textContent = '连接飞控并读取参数后自动评估。';
    return;
  }

  const rows = [];
  const add = (name, value, level, text) => rows.push({ name, value, level, text });
  const escLinear = !!document.getElementById('otEsc')?.checked;

  if (!TC_PARAMS.some(tcHas)) {
    sumEl.textContent = '读取中...';
    sumEl.className = 'tc-summary';
    verdictEl.textContent = '正在读取推力相关参数...';
    return;
  }

  // 输出协议（对应"先看电调协议"）
  if (!tcHas('MOT_PWM_TYPE')) {
    add('MOT_PWM_TYPE', '--', 'na', '未读到该参数，跳过协议判定');
  } else {
    const t = tcNum('MOT_PWM_TYPE');
    const label = TC_PWM_TYPE[t] !== undefined ? TC_PWM_TYPE[t] : `type=${t}`;
    if (t === 0) add('MOT_PWM_TYPE', label, 'info', '传统 PWM 时序：需要电调行程校准，油门-推力非线性靠 MOT_THST_EXPO 补偿');
    else if (t <= 2) add('MOT_PWM_TYPE', label, 'info', 'OneShot 类：仍属模拟 PWM 时序，建议配合 EXPO 补偿并做地面实测');
    else add('MOT_PWM_TYPE', label, 'ok', 'DShot 系列：电调端通常已接近线性，一般无需传统 PWM 行程校准');
  }

  // MOT_THST_EXPO（核心判断）
  if (!tcHas('MOT_THST_EXPO')) {
    add('MOT_THST_EXPO', '--', 'na', '未读到该参数');
  } else {
    const v = tcNum('MOT_THST_EXPO');
    let level = 'ok';
    let text = '';
    if (v <= 0.05) text = '≈0，飞控几乎不做线性化补偿 → 动力链已较线性，无需再调';
    else if (v <= 0.2) text = '低补偿(0~0.2)，与"电调已内置线性化"匹配，无需再调';
    else if (v < 0.45) { level = 'info'; text = '中等补偿，按桨径属偏低区间，建议地面油门阶梯实测确认低段无死区'; }
    else { level = 'info'; text = '明显补偿（官方按桨径 5寸0.55/10寸0.65/20寸0.75），属正常配置，不等于电调硬件不线性'; }
    if (escLinear && v > 0.3) {
      level = 'warn';
      text = `已勾选"电调已内置推力线性化"，但 EXPO=${v} 偏大 → 建议改为 0.10 左右`;
    }
    add('MOT_THST_EXPO', v, level, text);
  }

  // MOT_THST_HOVER
  if (!tcHas('MOT_THST_HOVER')) {
    add('MOT_THST_HOVER', '--', 'na', '未读到该参数');
  } else {
    const v = tcNum('MOT_THST_HOVER');
    if (v < 0.05) add('MOT_THST_HOVER', v, 'warn', '低于 0.05，接近电机死区，低油门段控制余量不足');
    else if (v > 0.6) add('MOT_THST_HOVER', v, 'warn', '高于 0.6，动力余量偏小，上升与抗风能力受限');
    else add('MOT_THST_HOVER', v, 'ok', '在正常区间 0.05~0.5；多次飞行保持稳定说明动力链路较健康');
  }

  // 悬停自学习
  if (!tcHas('MOT_HOVER_LEARN')) {
    add('MOT_HOVER_LEARN', '--', 'na', '未读到该参数');
  } else {
    const v = tcNum('MOT_HOVER_LEARN');
    if (v === 0) add('MOT_HOVER_LEARN', v, 'info', '悬停油门自学习已关闭，MOT_THST_HOVER 不会随飞行更新（建议 1 或 2）');
    else add('MOT_HOVER_LEARN', v, 'ok', '自学习开启，悬停油门会在飞行中自动更新');
  }

  // 交叉：悬停油门 vs 飞行最小输出
  if (tcHas('MOT_THST_HOVER') && tcHas('MOT_SPIN_MIN')) {
    const h = tcNum('MOT_THST_HOVER');
    const s = tcNum('MOT_SPIN_MIN');
    if (h <= s + 0.03) add('HOVER ↔ SPIN_MIN', `${h} / ${s}`, 'warn', '悬停油门过于接近飞行最小输出，低油门修正几乎没有余量');
    else add('HOVER ↔ SPIN_MIN', `${h} / ${s}`, 'ok', '悬停油门高于最小输出，低油门段仍有控制余量');
  } else add('HOVER ↔ SPIN_MIN', '--', 'na', '缺 MOT_THST_HOVER 或 MOT_SPIN_MIN，跳过');

  // 交叉：飞行最小输出 vs 解锁怠速
  if (tcHas('MOT_SPIN_MIN') && tcHas('MOT_SPIN_ARM')) {
    const mn = tcNum('MOT_SPIN_MIN');
    const ar = tcNum('MOT_SPIN_ARM');
    if (mn < ar + 0.03) add('SPIN_MIN ↔ SPIN_ARM', `${mn} / ${ar}`, 'warn', 'MOT_SPIN_MIN 必须 ≥ MOT_SPIN_ARM + 0.03，否则起飞段输出不线性、离地瞬间易抖动');
    else add('SPIN_MIN ↔ SPIN_ARM', `${mn} / ${ar}`, 'ok', '怠速与飞行最小输出之间留有 0.03 以上裕度');
  } else add('SPIN_MIN ↔ SPIN_ARM', '--', 'na', '缺 MOT_SPIN_MIN 或 MOT_SPIN_ARM，跳过');

  // PWM 行程
  if (tcHas('MOT_PWM_MIN') && tcHas('MOT_PWM_MAX')) {
    const lo = tcNum('MOT_PWM_MIN');
    const hi = tcNum('MOT_PWM_MAX');
    const span = hi - lo;
    if (span < 300) add('MOT_PWM_MIN~MAX', `${lo}~${hi}us`, 'warn', `行程仅 ${span}us，过窄，油门分辨率与线性度都会变差`);
    else if (lo === 1000 && hi === 2000) add('MOT_PWM_MIN~MAX', `${lo}~${hi}us`, 'ok', '标准行程，与电调行程校准一致');
    else add('MOT_PWM_MIN~MAX', `${lo}~${hi}us`, 'info', '非标准行程，请确认与电调校准后的实际行程一致');
  } else add('MOT_PWM_MIN~MAX', '--', 'na', '未读到（部分固件用 SERVO1_MIN/MAX 表示输出行程）');

  // 最大输出
  if (!tcHas('MOT_SPIN_MAX')) {
    add('MOT_SPIN_MAX', '--', 'na', '未读到该参数');
  } else {
    const v = tcNum('MOT_SPIN_MAX');
    if (v > 1.0 || v < 0.5) add('MOT_SPIN_MAX', v, 'warn', '取值超出常规 0.5~1.0');
    else if (v < 0.8) add('MOT_SPIN_MAX', v, 'info', `最大输出被限制为 ${v}，满油门以上无余量`);
    else add('MOT_SPIN_MAX', v, 'ok', '取值正常');
  }

  const warns = rows.filter((r) => r.level === 'warn').length;
  const oks = rows.filter((r) => r.level === 'ok').length;
  sumEl.textContent = warns ? `${warns} 项需关注 / 共 ${rows.length} 项` : `正常 ${oks}/${rows.length}`;
  sumEl.className = 'tc-summary ' + (warns ? 'bad' : 'good');
  verdictEl.textContent = tcVerdict(rows);

  rowsEl.innerHTML = rows.map((r) => `
    <div class="tc-row ${r.level}">
      <span class="tc-name">${r.name}</span>
      <span class="tc-val">${r.value}</span>
      <span class="tc-text">${r.text}</span>
    </div>`).join('');
}

function setThrustCheckCollapsed(collapsed) {
  document.getElementById('thrustCheck')?.classList.toggle('collapsed', collapsed);
  const t = document.getElementById('tcToggle');
  if (t) t.textContent = collapsed ? '展开 ▼' : '收起 ▲';
  try { localStorage.setItem('tcCollapsed', collapsed ? '1' : '0'); } catch (e) { /* 忽略 */ }
}

// ==================== 搜索和过滤 ====================
document.getElementById('paramSearch').addEventListener('input', renderParams);
document.getElementById('starFilter').addEventListener('change', renderParams);
document.getElementById('categoryFilter').addEventListener('change', (e) => {
  renderParams();
  const v = e.target.value;
  if (v !== 'all') scrollToCategory(parseInt(v, 10));
});

// 右侧滚动时同步左侧选中项
(function bindScrollSpy() {
  const panel = document.getElementById('paramPanel');
  let ticking = false;
  panel.addEventListener('scroll', () => {
    if (ticking) return;
    ticking = true;
    requestAnimationFrame(() => {
      ticking = false;
      syncActiveCategory();
    });
  });
})();

// ==================== 按钮事件 ====================
document.getElementById('btnConnect').onclick = connect;
document.getElementById('btnDisconnect').onclick = disconnect;
document.getElementById('btnReboot').onclick = rebootFC;
document.getElementById('btnRefreshParams').onclick = requestParams;
document.getElementById('btnBackup').onclick = () => switchView('backup');
document.getElementById('btnRestore').onclick = () => switchView('backup');

// 自动刷新串口列表
setInterval(refreshPorts, 3000);

// ==================== 视图切换 ====================
function switchView(name) {
  document.querySelectorAll('.view-tab').forEach((t) => {
    t.classList.toggle('active', t.dataset.view === name);
  });
  const map = {
    params: 'viewParams',
    onetouch: 'viewOnetouch',
    backup: 'viewBackup',
    logs: 'viewLogs',
    fault: 'viewFault',
  };
  for (const [k, id] of Object.entries(map)) {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('hidden', k !== name);
  }
  if (name === 'backup' && typeof DataTools !== 'undefined') DataTools.handleMessage({ type: 'noop' });
  if (name === 'fault' && typeof Fault !== 'undefined') Fault.onShow();
}
for (const t of document.querySelectorAll('.view-tab')) {
  t.onclick = () => switchView(t.dataset.view);
}

// ==================== 初始化 ====================
const tcHeadEl = document.getElementById('tcHead');
if (tcHeadEl) {
  tcHeadEl.onclick = () => {
    setThrustCheckCollapsed(!document.getElementById('thrustCheck').classList.contains('collapsed'));
  };
}
document.getElementById('otEsc')?.addEventListener('change', () => scheduleThrustCheck(0));
let tcCollapsed = false;
try { tcCollapsed = localStorage.getItem('tcCollapsed') === '1'; } catch (e) { /* 忽略 */ }
setThrustCheckCollapsed(tcCollapsed);
renderThrustCheck();
renderCategories();
console.log('AP Tune 前端已初始化');