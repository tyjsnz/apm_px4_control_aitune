// APM/PX4 调参 Web 后端
// - 正确的 MAVLink v1/v2 协议层 (lib/mavlink.js) + 参数读写 (lib/paramlink.js)
// - WebSocket: 遥测广播 / 参数读写 / 一键首飞(方案体检·写入·校准·拆桨实测·备份)
import express from 'express';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import { SerialPort } from 'serialport';
import fs from 'fs';
import path from 'path';
import { exec } from 'child_process';
import { fileURLToPath } from 'url';

import { MAVLINK_DEFS } from './lib/mavlink.js';
import { ParamLink, MAV_CMD, MAV_RESULT, MOTOR_TEST_THROTTLE_PERCENT } from './lib/paramlink.js';
import { aiChat, providerList, providerInfo } from './lib/ai.js';
import { analyzeLog, buildLogUserPrompt, LOG_SYSTEM_PROMPT, extractCharts } from './lib/loganalysis.js';
import { FAULT_SYSTEM_PROMPT, buildFaultUserPrompt } from './lib/faultai.js';
import { createFaultStore } from './lib/faultstore.js';
import { createAiConfigStore } from './lib/aiconfigstore.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
// 数据目录 (可用环境变量覆盖, 便于测试隔离)
const BACKUP_DIR = process.env.BACKUP_DIR || path.join(__dirname, 'backups');
const LOG_DIR = process.env.LOG_DIR || path.join(__dirname, 'logs');
const FAULT_DIR = process.env.FAULT_DIR || path.join(__dirname, 'faults');
const AI_CONFIG_FILE = process.env.AI_CONFIG_FILE || path.join(__dirname, 'config', 'ai_config.json');

const PORT = Number(process.env.PORT) || 3000;
const HOST = process.env.HOST || '0.0.0.0';

// ArduCopter 飞行模式表 (custom_mode)
const COPTER_MODES = [
  'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL',
  'CIRCLE', 'LAND', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD',
  'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD',
  'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL',
];

const ACCEL_POS_NAMES = {
  1: 'LEVEL 水平', 2: 'LEFT 左侧', 3: 'RIGHT 右侧',
  4: 'NOSEDOWN 机头朝下', 5: 'NOSEUP 机头朝上', 6: 'BACK 背面朝上',
};

fs.mkdirSync(BACKUP_DIR, { recursive: true });
fs.mkdirSync(LOG_DIR, { recursive: true });
fs.mkdirSync(FAULT_DIR, { recursive: true });

const app = express();
const server = createServer(app);

// 可选 HTTP Basic 认证: 设置环境变量 AUTH_USER / AUTH_PASS 即启用(公网部署强烈建议)
const AUTH_USER = process.env.AUTH_USER || '';
const AUTH_PASS = process.env.AUTH_PASS || '';
const AUTH_ON = !!(AUTH_USER || AUTH_PASS);

function checkAuth(req) {
  if (!AUTH_ON) return true;
  const h = req.headers.authorization || '';
  if (!h.startsWith('Basic ')) return false;
  let dec = '';
  try { dec = Buffer.from(h.slice(6), 'base64').toString('utf8'); } catch (e) { return false; }
  const idx = dec.indexOf(':');
  if (idx < 0) return false;
  return dec.slice(0, idx) === AUTH_USER && dec.slice(idx + 1) === AUTH_PASS;
}

app.use((req, res, next) => {
  if (checkAuth(req)) return next();
  res.set('WWW-Authenticate', 'Basic realm="apm-tune"');
  return res.status(401).send('需要认证');
});

app.use(express.static(path.join(__dirname, 'public')));

// WebSocket: 同样做认证(HTTP 中间件不覆盖 upgrade 请求)
const wss = new WebSocketServer({ noServer: true });
server.on('upgrade', (req, socket, head) => {
  if (!checkAuth(req)) {
    socket.write('HTTP/1.1 401 Unauthorized\r\n'
      + 'WWW-Authenticate: Basic realm="apm-tune"\r\nConnection: close\r\n\r\n');
    socket.destroy();
    return;
  }
  wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws, req));
});

// ==================== 本地文件下载 ====================
function serveFile(res, dir, name, contentType) {
  const safe = path.basename(String(name || ''));
  const full = path.join(dir, safe);
  if (!safe || !full.startsWith(dir) || !fs.existsSync(full)) {
    res.status(404).send('文件不存在');
    return;
  }
  res.setHeader('Content-Type', contentType || 'application/octet-stream');
  res.setHeader('Content-Disposition', `attachment; filename="${safe}"`);
  fs.createReadStream(full).pipe(res);
}

app.get('/logs/:name', (req, res) => serveFile(res, LOG_DIR, req.params.name, 'application/octet-stream'));
app.get('/backups/:name', (req, res) => serveFile(res, BACKUP_DIR, req.params.name, 'application/json'));
app.get('/faults/:name', (req, res) => serveFile(res, FAULT_DIR, req.params.name, 'application/json'));

// ==================== 状态 ====================
let serialPort = null;
let link = null;              // ParamLink 实例
let heartbeatTimer = null;
const clients = new Set();

const telemetry = {
  volt: null,        // V
  cells: null,
  load: null,
  armed: false,
  mode: '--',
  autopilot: null,
  sysid: 1,
  compid: 1,
};

let lastEsc = { rpm: null, t: 0 };  // 最新 ESC 遥测
let accelCal = null;                // 六面校准状态机
let opLock = Promise.resolve();     // 串行化一键首飞操作
let paramCacheAll = {};             // name -> { value, type } 最近一次 PARAM_VALUE(含列表全量回读)

// 突发消息队列 (PARAM_VALUE / LOG_ENTRY / LOG_DATA / STATUSTEXT):
// 同一批到达的多条消息不能靠"单次等待"逐条取, 否则会漏包
const msgQueue = [];
const MSG_QUEUE_MAX = 50000;

function enqueueMsg(name, fields) {
  msgQueue.push({ name, fields });
  if (msgQueue.length > MSG_QUEUE_MAX) msgQueue.splice(0, msgQueue.length - MSG_QUEUE_MAX);
}

function takeMsg(name, pred) {
  const i = msgQueue.findIndex((x) => x.name === name && (!pred || pred(x.fields)));
  if (i < 0) return null;
  return msgQueue.splice(i, 1)[0].fields;
}

function clearMsg(name) {
  for (let i = msgQueue.length - 1; i >= 0; i--) if (msgQueue[i].name === name) msgQueue.splice(i, 1);
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ==================== 工具 ====================
const broadcast = (obj) => {
  const msg = JSON.stringify(obj);
  for (const c of clients) {
    if (c.readyState === 1) {
      try { c.send(msg); } catch (e) { /* ignore */ }
    }
  }
};

const send = (ws, obj) => {
  if (ws && ws.readyState === 1) {
    try { ws.send(JSON.stringify(obj)); } catch (e) { /* ignore */ }
  }
};

function withLock(fn) {
  const run = opLock.then(fn, fn);
  opLock = run.then(() => {}, () => {});
  return run;
}

function timestamp() {
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

// ==================== MAVLink 消息处理 ====================
function handleMavlink(m) {
  if (m.sysid && m.sysid !== link.link.srcSys) {
    telemetry.sysid = m.sysid;
    telemetry.compid = m.compid || telemetry.compid;
  }

  switch (m.name) {
    case 'HEARTBEAT': {
      const f = m.fields;
      if (f.autopilot === 8) break; // 忽略其它 GCS
      telemetry.autopilot = f.autopilot;
      telemetry.armed = !!(f.base_mode & 0x80);
      if (f.autopilot === 3 || f.autopilot === 0) {
        telemetry.mode = COPTER_MODES[f.custom_mode] || String(f.custom_mode);
      } else {
        telemetry.mode = String(f.custom_mode);
      }
      broadcast({
        type: 'heartbeat',
        flightMode: telemetry.mode,
        armed: telemetry.armed,
        autopilot: f.autopilot,
        mavType: f.type,
        baseMode: f.base_mode,
        customMode: f.custom_mode,
      });
      break;
    }
    case 'PARAM_VALUE': {
      const f = m.fields;
      enqueueMsg('PARAM_VALUE', f);
      const pname = (f.param_id || '').replace(/\0.*$/, '');
      if (pname) paramCacheAll[pname] = { value: f.param_value, type: f.param_type };
      broadcast({
        type: 'param_value',
        paramId: pname,
        paramValue: f.param_value,
        paramType: f.param_type,
        paramCount: f.param_count,
        paramIndex: f.param_index,
      });
      break;
    }
    case 'ATTITUDE': {
      broadcast({
        type: 'attitude',
        timestamp: m.fields.time_boot_ms,
        roll: m.fields.roll, pitch: m.fields.pitch, yaw: m.fields.yaw,
        rollspeed: m.fields.rollspeed, pitchspeed: m.fields.pitchspeed,
        yawspeed: m.fields.yawspeed,
      });
      break;
    }
    case 'GLOBAL_POSITION_INT': {
      broadcast({
        type: 'global_position',
        timeBootMs: m.fields.time_boot_ms,
        lat: m.fields.lat, lon: m.fields.lon,
        alt: m.fields.alt, relativeAlt: m.fields.relative_alt,
        vx: m.fields.vx, vy: m.fields.vy, vz: m.fields.vz, hdg: m.fields.hdg,
      });
      break;
    }
    case 'VFR_HUD': {
      broadcast({
        type: 'vfr_hud',
        airspeed: m.fields.airspeed, groundspeed: m.fields.groundspeed,
        heading: m.fields.heading, throttle: m.fields.throttle,
        altitude: m.fields.alt, clambrate: m.fields.climb,
      });
      break;
    }
    case 'SYS_STATUS': {
      const volt = m.fields.voltage_battery > 0 ? m.fields.voltage_battery / 1000 : null;
      if (volt) { telemetry.volt = volt; telemetry.cells = Math.round(volt / 3.7); }
      telemetry.load = m.fields.load / 100;
      broadcast({
        type: 'sys_status',
        voltageBattery: m.fields.voltage_battery,
        currentBattery: m.fields.current_battery,
        batteryRemaining: m.fields.battery_remaining,
        load: m.fields.load,
      });
      break;
    }
    case 'ESC_TELEMETRY_1_TO_4': {
      lastEsc = { rpm: m.fields.rpm, t: Date.now() };
      break;
    }
    case 'STATUSTEXT': {
      enqueueMsg('STATUSTEXT', m.fields);
      broadcast({
        type: 'statustext',
        severity: m.fields.severity,
        text: (m.fields.text || '').replace(/\0.*$/, ''),
      });
      break;
    }
    case 'LOG_ENTRY': {
      enqueueMsg('LOG_ENTRY', m.fields);
      break;
    }
    case 'LOG_DATA': {
      enqueueMsg('LOG_DATA', m.fields);
      break;
    }
    case 'COMMAND_LONG': {
      // 六面加速度计校准: 飞控回显 ACCELCAL_VEHICLE_POS
      const f = m.fields;
      if (Number(f.command) === MAV_CMD.ACCELCAL_VEHICLE_POS && accelCal) {
        const pos = Number(f.param1);
        if (pos >= 16777215) {
          const ok = pos === 16777215;
          accelCal = null;
          broadcast({ type: 'ot_accel_cal_done', ok, text: '六面加速度计校准完成' });
        } else if (accelCal) {
          accelCal.pos = pos;
          broadcast({
            type: 'ot_accel_cal_prompt',
            pos,
            text: ACCEL_POS_NAMES[pos] || String(pos),
          });
        }
      }
      break;
    }
    default:
      break;
  }
}

// ==================== 串口 ====================
function startHeartbeat() {
  stopHeartbeat();
  heartbeatTimer = setInterval(() => {
    if (link && serialPort && serialPort.isOpen) {
      try { link.sendHeartbeat(); } catch (e) { /* ignore */ }
    }
  }, 1000);
}

function stopHeartbeat() {
  if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
}

function openSerial(pathName, baud) {
  if (serialPort && serialPort.isOpen) {
    throw new Error('已连接，请先断开');
  }
  serialPort = new SerialPort({ path: pathName, baudRate: baud, autoOpen: false });

  paramCacheAll = {};
  link = new ParamLink({
    write: (buf) => { if (serialPort && serialPort.isOpen) serialPort.write(buf); },
    onMessage: handleMavlink,
  });

  serialPort.on('data', (data) => {
    try { link.feed(data); } catch (err) { console.error('MAVLink 解析错误:', err.message); }
  });

  serialPort.on('open', () => {
    console.log(`串口已打开: ${pathName} @ ${baud}`);
    broadcast({ type: 'connected', path: pathName, baud });
    startHeartbeat();
    link.sendHeartbeat();
    setTimeout(() => { try { link.requestParamList(); } catch (e) { /* ignore */ } }, 500);
  });

  serialPort.on('close', () => {
    console.log('串口已关闭');
    stopHeartbeat();
    if (accelCal) { accelCal = null; broadcast({ type: 'ot_accel_cal_done', ok: false, text: '连接断开，校准中止' }); }
    serialPort = null;
    if (link) { link.close(); link = null; }
    paramCacheAll = {};
    broadcast({ type: 'disconnected' });
  });

  serialPort.on('error', (err) => {
    console.error('串口错误:', err.message);
    const wasOpen = serialPort && serialPort.isOpen;
    broadcast({ type: 'error', message: err.message });
    // 不在 error 处理器里调用 close()，否则会触发 'Port is not open' 二次错误死循环
    if (!wasOpen) {
      stopHeartbeat();
      if (link) { link.close(); link = null; }
      serialPort = null;
    }
  });

  serialPort.open();
}

function requireLink(ws) {
  if (!link || !serialPort || !serialPort.isOpen) {
    send(ws, { type: 'error', message: '未连接飞控' });
    return null;
  }
  return link;
}

// ==================== 一键首飞: 体检读取 ====================
async function resolveParam(entry, timeout = 900) {
  const cands = [[entry.name, 1.0]].concat(entry.aliases || []);
  let first = null;
  for (const [name, scale] of cands) {
    const v = await link.readParam(name, timeout, 0);
    if (v === null) continue;
    if (first === null) first = { name, scale, value: v / scale };
    if (Math.abs(v) > 1e-9) return { name, scale, value: v / scale };
  }
  if (first) return first;
  return { name: entry.name, scale: 1.0, value: null };
}

async function otRead(ws, params) {
  // 电压/串数提示依赖 SYS_STATUS, 若尚未收到则短暂等待
  if (telemetry.volt === null) {
    const t0 = Date.now();
    while (telemetry.volt === null && Date.now() - t0 < 1500) {
      await new Promise((r) => setTimeout(r, 100));
    }
  }
  const total = params.length;
  const results = new Array(total);
  let done = 0;
  let cursor = 0;
  const concurrency = 4;
  const worker = async () => {
    while (true) {
      const i = cursor++;
      if (i >= total) return;
      const p = params[i];
      const res = await resolveParam(p);
      results[i] = {
        key: p.name, name: res.name, scale: res.scale, value: res.value,
      };
      done++;
      send(ws, {
        type: 'ot_read_progress', done, total,
        name: p.name, value: res.value, resolved: res.name,
      });
    }
  };
  await Promise.all(Array.from({ length: Math.min(concurrency, total) }, worker));
  send(ws, {
    type: 'ot_read_result',
    results,
    volt: telemetry.volt,
    cells: telemetry.cells,
  });
}

// ==================== 一键首飞: 备份 + 写入 ====================
function fcTypeName() {
  return telemetry.autopilot === 3 ? 'APM_COPTER' : 'UNKNOWN';
}

/**
 * 备份指定参数。逐个 PARAM_REQUEST_READ 读取, 读不到的回退到参数表缓存
 * (PARAM_REQUEST_LIST 全量回读 / 单条回写的最近值)。
 * 一项都拿不到时抛错, 绝不写出"0 项的空备份"。
 */
async function saveBackup(names, tag) {
  const total = names.length;
  if (!total) throw new Error('没有需要备份的参数');
  const vals = await link.readMany(names, { concurrency: 4, timeout: 1500, retries: 2 });

  // 逐名读取 → 会话参数缓存, 组装出可备份的参数集
  const collect = (src) => {
    const out = {};
    const missing = [];
    let fromCache = 0;
    for (const n of names) {
      let v = src[n];
      if ((v === null || v === undefined) && paramCacheAll[n]) {
        v = paramCacheAll[n].value;
        fromCache++;
      }
      if (v !== null && v !== undefined && !Number.isNaN(Number(v))) out[n] = v;
      else missing.push(n);
    }
    return { out, missing, fromCache };
  };

  let got = collect(vals);
  if (!Object.keys(got.out).length) {
    // 逐个读全部失败 → 再做一次全量参数列表回读兜底
    const dump = await dumpParamList({ timeoutMs: 8000 });
    for (const n of names) {
      if ((vals[n] === null || vals[n] === undefined) && dump.params[n] !== undefined) vals[n] = dump.params[n];
    }
    got = collect(vals);
  }

  const params = got.out;
  const count = Object.keys(params).length;
  if (!count) {
    const noHeartbeat = telemetry.autopilot === null || telemetry.autopilot === undefined;
    throw new Error(`${total} 项参数全部读取失败`
      + (noHeartbeat ? '(且未收到飞控心跳, 请检查串口/波特率或固件类型)' : '(飞控未响应参数读取)'));
  }
  const ts = timestamp();
  const file = `backup_${ts}${tag ? '_' + tag : ''}.json`;
  const data = {
    timestamp: ts,
    fc_type: fcTypeName(),
    spec: {},
    params,
  };
  fs.writeFileSync(path.join(BACKUP_DIR, file), JSON.stringify(data, null, 2), 'utf-8');
  console.log(`备份 ${file}: ${count}/${total} 项 (缓存补齐 ${got.fromCache}, 读取失败 ${got.missing.length})`);
  return { file, count, total, fromCache: got.fromCache, missing: got.missing.length };
}

async function otWrite(ws, items, opts = {}) {
  const names = items.map((it) => it.name);
  let backup = null;
  if (!opts.skipBackup) {
    try {
      backup = await saveBackup(names, 'onetouch');
      send(ws, {
        type: 'ot_backup', backup: backup.file, count: backup.count,
        total: backup.total, fromCache: backup.fromCache, missing: backup.missing.length,
      });
    } catch (err) {
      // 不发 error 弹窗: 由前端用确认框呈现"是否跳过备份继续写入"
      send(ws, {
        type: 'ot_write_done', success: 0, failed: names,
        backup: null, aborted: true, reason: err.message,
      });
      return;
    }
  }

  let success = 0;
  const failed = [];
  const total = items.length;
  for (let i = 0; i < total; i++) {
    const it = items[i];
    let ok = false;
    // 优先用飞控上报的参数类型, 其次前端指定, 最后默认 REAL32
    const ptype = it.paramType
      || (paramCacheAll[it.name] && paramCacheAll[it.name].type)
      || 9;
    try {
      ok = await link.setParam(it.name, it.value, ptype);
    } catch (err) {
      ok = false;
    }
    if (ok) success++; else failed.push(it.name);
    send(ws, {
      type: 'ot_write_progress', done: i + 1, total,
      name: it.name, ok,
    });
  }
  send(ws, {
    type: 'ot_write_done', success, failed,
    backup: backup ? backup.file : null,
    backupCount: backup ? backup.count : 0,
    backupTotal: backup ? backup.total : 0,
    skippedBackup: !!opts.skipBackup,
  });
}

// ==================== 一键首飞: 校准 ====================
async function otCal(ws, kind) {
  const p1 = kind === 'gyro' ? 1 : 0;
  const p3 = kind === 'baro' ? 1 : 0;
  const p5 = kind === 'level' ? 2 : 0;
  const ack = await link.sendCommand(
    MAV_CMD.PREFLIGHT_CALIBRATION, [p1, 0, p3, 0, p5, 0, 0], { timeout: 8000 });
  send(ws, {
    type: 'ot_cal_done', kind, ok: ack.ok,
    text: (ack.text && ack.text.length ? ack.text[ack.text.length - 1] : '')
      || (ack.timeout ? '无返回(超时)' : `result=${ack.result}`),
  });
}

async function startAccelCal(ws) {
  if (accelCal) { send(ws, { type: 'error', message: '六面校准已在进行中' }); return; }
  accelCal = { pos: null, startedAt: Date.now() };
  send(ws, { type: 'ot_accel_cal_prompt', pos: 0, text: '正在启动校准...' });
  const ack = await link.sendCommand(
    MAV_CMD.PREFLIGHT_CALIBRATION, [0, 0, 0, 0, 1, 0, 0], { timeout: 6000 });
  if (!ack.ok && ack.result !== MAV_RESULT.IN_PROGRESS && accelCal) {
    send(ws, { type: 'error', message: '启动六面校准失败: ' + JSON.stringify(ack) });
  }
}

function accelCalConfirm(ws, pos) {
  if (!accelCal) { send(ws, { type: 'error', message: '没有进行中的六面校准' }); return; }
  link.send('COMMAND_LONG', {
    param1: Number(pos), param2: 0, param3: 0, param4: 0,
    param5: 0, param6: 0, param7: 0,
    command: MAV_CMD.ACCELCAL_VEHICLE_POS,
    target_system: link.targetSys, target_component: link.targetComp,
    confirmation: 0,
  });
}

// ==================== 一键首飞: 拆桨实测起转 ====================
function waitEscRpm(index, spanMs) {
  return new Promise((resolve) => {
    let best = 0;
    const t0 = Date.now();
    const tick = () => {
      if (lastEsc.rpm && lastEsc.rpm[index] !== undefined) {
        best = Math.max(best, Number(lastEsc.rpm[index]) || 0);
      }
      if (Date.now() - t0 >= spanMs) { resolve(best); return; }
      setTimeout(tick, 60);
    };
    tick();
  });
}

async function otSpinTest(ws) {
  if (telemetry.armed) {
    send(ws, { type: 'ot_spin_done', ok: false, text: '飞控已解锁，禁止电机测试' });
    return;
  }
  let escEverSeen = false;
  const spinOf = async (motor) => {
    for (let pct = 6; pct <= 40; pct += 2) {
      const ack = await link.sendCommand(
        MAV_CMD.DO_MOTOR_TEST,
        [motor, MOTOR_TEST_THROTTLE_PERCENT, pct, 2, 1, 0, 0],
        { timeout: 2500 });
      if (ack.result !== null && ack.result !== MAV_RESULT.ACCEPTED
          && ack.result !== MAV_RESULT.IN_PROGRESS) {
        throw new Error('电机测试被拒 (result=' + ack.result + ') '
          + (ack.text && ack.text.join('; ') || ''));
      }
      const rpm = await waitEscRpm(motor - 1, 800);
      if (lastEsc.rpm) escEverSeen = true;
      send(ws, { type: 'ot_spin_progress', motor, pct, rpm });
      if (rpm > 0) return pct;
    }
    return null;
  };

  try {
    const spin1 = await spinOf(1);
    if (spin1 === null && !escEverSeen) {
      send(ws, {
        type: 'ot_spin_done', ok: false,
        text: '未收到 ESC 遥测(RPM=0)。需要 DShot + 遥测线；否则请拆桨人工测定 MOT_SPIN_ARM。',
      });
      return;
    }
    if (spin1 === null) {
      send(ws, { type: 'ot_spin_done', ok: false, text: '40% 以内电机1 仍未起转。' });
      return;
    }
    let spin = spin1;
    for (const motor of [2, 3, 4]) {
      let pct = spin;
      let found = false;
      while (pct <= spin + 8) {
        await link.sendCommand(
          MAV_CMD.DO_MOTOR_TEST,
          [motor, MOTOR_TEST_THROTTLE_PERCENT, pct, 2, 1, 0, 0],
          { timeout: 2500 });
        const rpm = await waitEscRpm(motor - 1, 700);
        send(ws, { type: 'ot_spin_progress', motor, pct, rpm });
        if (rpm > 0) { found = true; break; }
        pct += 2;
      }
      if (found) spin = Math.max(spin, pct);
      else spin = Math.max(spin, spin + 8);
    }
    const arm = Math.round(Math.min(0.2, spin / 100 + 0.01) * 100) / 100;
    const min = Math.round(Math.min(0.25, arm + 0.03) * 100) / 100;
    send(ws, { type: 'ot_spin_done', ok: true, spin, arm, min });
  } catch (err) {
    send(ws, { type: 'ot_spin_done', ok: false, text: err.message });
  }
}

// ==================== 备份管理 ====================
// ==================== 参数全量备份 / 恢复 ====================
/** 全量参数回读(PARAM_REQUEST_LIST + 消息队列), 返回 {params, expect} */
async function dumpParamList(opts = {}) {
  const timeoutMs = opts.timeoutMs || 30000;
  const quietMs = opts.quietMs || 3000;
  const onProgress = opts.onProgress || null;
  const params = {};
  let expect = null;
  clearMsg('PARAM_VALUE');
  const t0 = Date.now();
  link.requestParamList();
  while (Date.now() - t0 < timeoutMs) {
    const f = takeMsg('PARAM_VALUE');
    if (!f) {
      if (expect !== null && Object.keys(params).length >= expect) break;
      if (Date.now() - t0 > quietMs && Object.keys(params).length === 0) break;
      await sleep(30);
      continue;
    }
    const name = (f.param_id || '').replace(/\0.*$/, '');
    if (f.param_count) expect = f.param_count;
    params[name] = f.param_value;
    if (onProgress) onProgress(Object.keys(params).length, expect);
    if (expect !== null && Object.keys(params).length >= expect) break;
  }
  return { params, expect };
}

async function paramBackupAll(ws, tag) {
  const dumped = await dumpParamList({
    timeoutMs: 30000,
    onProgress: (count, total) => {
      if (count % 25 === 0) send(ws, { type: 'param_backup_progress', count, total });
    },
  });
  const params = dumped.params;
  let expect = dumped.expect;
  let count = Object.keys(params).length;
  let fromCache = false;
  if (!count) {
    // 全量回读没拿到(链路繁忙/超时)则退回会话内参数缓存, 避免空备份
    for (const [k, v] of Object.entries(paramCacheAll)) params[k] = v.value;
    count = Object.keys(params).length;
    fromCache = count > 0;
    if (count) expect = count;
  }
  if (!count) { send(ws, { type: 'param_backup_done', ok: false, error: '未读取到参数' }); return; }
  const ts = timestamp();
  const file = `param_${ts}${tag ? '_' + tag : ''}.json`;
  const data = {
    timestamp: ts,
    fc_type: fcTypeName(),
    total: expect,
    params,
  };
  fs.writeFileSync(path.join(BACKUP_DIR, file), JSON.stringify(data, null, 2), 'utf-8');
  send(ws, { type: 'param_backup_done', ok: true, file, count, total: expect, fromCache });
}

async function paramRestore(ws, file) {
  const safe = path.basename(String(file || ''));
  const full = path.join(BACKUP_DIR, safe);
  if (!safe || !full.startsWith(BACKUP_DIR) || !fs.existsSync(full)) {
    send(ws, { type: 'error', message: '备份文件不存在' });
    return;
  }
  const data = JSON.parse(fs.readFileSync(full, 'utf-8'));
  const params = data.params || {};
  const entries = Object.entries(params);
  if (!entries.length) {
    send(ws, { type: 'param_restore_done', success: 0, failed: [], file: safe, empty: true });
    return;
  }
  let success = 0;
  const failed = [];
  for (let i = 0; i < entries.length; i++) {
    const [name, value] = entries[i];
    let ok = false;
    // 用飞控已知的参数类型写回, 避免非浮点参数因类型不符被忽略
    const ptype = paramCacheAll[name] && paramCacheAll[name].type ? paramCacheAll[name].type : 9;
    try { ok = await link.setParam(name, value, ptype); } catch (e) { ok = false; }
    if (ok) success++; else failed.push(name);
    send(ws, { type: 'param_restore_progress', done: i + 1, total: entries.length, name, ok });
  }
  send(ws, { type: 'param_restore_done', success, failed, file: safe });
}

// ==================== 日志下载 / 清除 ====================
async function logList(timeoutMs = 8000) {
  const logs = [];
  const seen = new Set();
  let num = null;
  clearMsg('LOG_ENTRY');
  const t0 = Date.now();
  link.send('LOG_REQUEST_LIST', {
    target_system: link.targetSys, target_component: link.targetComp,
    start: 0, end: 0xFFFF,
  });
  while (Date.now() - t0 < timeoutMs) {
    const f = takeMsg('LOG_ENTRY');
    if (!f) {
      if (num !== null && logs.length >= num) break;
      if (Date.now() - t0 > 2500 && logs.length === 0) break;
      await sleep(30);
      continue;
    }
    const id = f.id;
    num = f.num_logs;
    if (!seen.has(id)) {
      seen.add(id);
      logs.push({
        id, size: f.size, timeUtc: f.time_utc,
        lastLogNum: f.last_log_num, numLogs: f.num_logs,
      });
    }
    if (num !== null && logs.length >= num) break;
  }
  logs.sort((a, b) => a.id - b.id);
  return logs;
}

async function requestLogBlock(id, offset, length, deadline) {
  const end = offset + length;
  let next = offset;
  const pending = new Map();
  const chunks = [];
  const request = () => link.send('LOG_REQUEST_DATA', {
    target_system: link.targetSys, target_component: link.targetComp,
    id, ofs: next, count: end - next,
  });
  request();
  let lastReq = Date.now();
  while (next < end) {
    if (Date.now() > deadline) return null;
    const f = takeMsg('LOG_DATA', (x) => x.id === id);
    if (!f) {
      if (Date.now() - lastReq > 3000) request();
      await sleep(20);
      continue;
    }
    const ofs = f.ofs;
    let d = Buffer.from(f.data).subarray(0, f.count);
    if (ofs >= end || ofs + d.length <= next) continue;
    let start = ofs;
    if (start < next) { d = d.subarray(next - start); start = next; }
    if (start + d.length > end) d = d.subarray(0, end - start);
    if (!pending.has(start)) pending.set(start, d);
    while (pending.has(next)) {
      const b = pending.get(next);
      pending.delete(next);
      chunks.push(b);
      next += b.length;
    }
  }
  return Buffer.concat(chunks);
}

async function downloadLogById(id, onProgress) {
  // 取大小
  clearMsg('LOG_ENTRY');
  link.send('LOG_REQUEST_LIST', {
    target_system: link.targetSys, target_component: link.targetComp,
    start: id, end: id,
  });
  let size = null;
  const t0 = Date.now();
  while (Date.now() - t0 < 5000) {
    const f = takeMsg('LOG_ENTRY', (x) => x.id === id);
    if (f) { size = f.size; break; }
    await sleep(30);
  }
  if (size === null) return { error: '无法获取日志大小' };
  if (size === 0) return { error: '日志为空' };

  clearMsg('LOG_DATA');
  const deadline = Date.now() + Math.max(60000, (size / 8000) * 1000 + 30000);
  const out = Buffer.alloc(size);
  let offset = 0;
  const blockSize = 900;
  while (offset < size) {
    if (Date.now() > deadline) {
      return { error: `下载超时: ${offset}/${size} 字节` };
    }
    const reqLen = Math.min(blockSize, size - offset);
    const block = await requestLogBlock(id, offset, reqLen, deadline);
    if (block === null) return { error: `下载超时: ${offset}/${size} 字节` };
    block.copy(out, offset);
    offset += block.length;
    if (onProgress) onProgress(offset, size, id);
  }
  return { data: out };
}

async function logDownloadMany(ws, ids) {
  const saved = [];
  const failed = [];
  for (const id of ids) {
    send(ws, { type: 'log_download_start', id });
    const res = await downloadLogById(id, (cur, total) => {
      send(ws, { type: 'log_download_progress', id, current: cur, total });
    });
    if (res.error) { failed.push({ id, error: res.error }); send(ws, { type: 'log_download_error', id, error: res.error }); continue; }
    const ts = timestamp();
    const name = `log_${id}_${ts}.bin`;
    fs.writeFileSync(path.join(LOG_DIR, name), res.data);
    saved.push({ id, file: name, size: res.data.length });
    send(ws, { type: 'log_download_saved', id, file: name, size: res.data.length });
  }
  send(ws, { type: 'log_download_done', saved, failed });
}

async function logErase(ws) {
  clearMsg('STATUSTEXT');
  clearMsg('LOG_ENTRY');
  link.send('LOG_ERASE', { target_system: link.targetSys, target_component: link.targetComp });
  const t0 = Date.now();
  let erased = false;
  while (Date.now() - t0 < 20000) {
    const st = takeMsg('STATUSTEXT', (f) => /eras/i.test(f.text || ''));
    if (st) { erased = true; break; }
    const le = takeMsg('LOG_ENTRY', (f) => f.num_logs === 0);
    if (le) { erased = true; break; }
    await sleep(50);
  }
  send(ws, { type: 'log_erase_done', ok: erased, text: erased ? '日志已清除' : '清除超时或飞控未确认' });
}

function listLocalLogs() {
  if (!fs.existsSync(LOG_DIR)) return [];
  const out = [];
  for (const f of fs.readdirSync(LOG_DIR)) {
    if (!f.endsWith('.bin')) continue;
    const st = fs.statSync(path.join(LOG_DIR, f));
    out.push({ file: f, size: st.size, mtime: st.mtimeMs });
  }
  out.sort((a, b) => b.mtime - a.mtime);
  return out;
}

// ==================== 日志 AI 分析 ====================
async function logAnalyze(ws, msg) {
  let files = [];
  if (Array.isArray(msg.files) && msg.files.length) files = msg.files;
  else if (msg.file) files = [msg.file];
  if (!files.length) { send(ws, { type: 'log_analyze_done', ok: false, error: '未指定日志' }); return; }

  const summaries = [];
  const metricsList = [];
  const usedFiles = [];
  const missing = [];
  for (const f of files) {
    const safe = path.basename(String(f || ''));
    const full = path.join(LOG_DIR, safe);
    if (!safe || !full.startsWith(LOG_DIR) || !fs.existsSync(full)) { missing.push(safe || String(f)); continue; }
    try {
      const buf = fs.readFileSync(full);
      const s = analyzeLog(buf, { excerptRows: 30 });
      summaries.push(`########## 日志 ${safe} ##########\n${s.text}`);
      metricsList.push({ file: safe, metrics: s.metrics });
      usedFiles.push(safe);
      // 传感器时序曲线
      try {
        const charts = extractCharts(buf, { maxPoints: 400 });
        send(ws, { type: 'log_charts', file: safe, charts });
      } catch (e) {
        send(ws, { type: 'log_charts', file: safe, charts: [], error: e.message });
      }
    } catch (err) {
      send(ws, { type: 'log_analyze_error', file: safe, error: '解析失败: ' + err.message });
    }
  }
  if (!summaries.length) {
    send(ws, { type: 'log_analyze_done', ok: false, error: '无可分析日志' + (missing.length ? ' (不存在: ' + missing.join(', ') + ')' : '') });
    return;
  }
  let combined = summaries.join('\n\n');
  const LIMIT = 150000;
  if (combined.length > LIMIT) combined = combined.slice(0, LIMIT) + '\n... (内容过长已截断)';
  const title = usedFiles.length === 1 ? usedFiles[0] : `${usedFiles.length} 个日志`;
  send(ws, {
    type: 'log_analyze_summary', file: title, files: usedFiles,
    summary: combined, metrics: metricsList,
  });

  // 使用全局 AI 配置 (请求中显式指定的字段可覆盖)
  const ai = aiConfigStore.resolve(msg);
  if (!aiReady(ai)) {
    // 未配置可用的 AI -> 仅返回数据摘要
    send(ws, { type: 'log_analyze_done', ok: true, ai: false, file: title, files: usedFiles, report: combined });
    return;
  }
  send(ws, { type: 'log_analyze_progress', stage: 'ai', file: title, files: usedFiles });
  const r = await aiChat({
    provider: ai.provider,
    apiKey: ai.apiKey,
    model: ai.model,
    baseUrl: ai.baseUrl,
    system: LOG_SYSTEM_PROMPT,
    user: buildLogUserPrompt(title, combined),
    timeoutMs: 180000,
  });
  if (r.error) {
    send(ws, { type: 'log_analyze_done', ok: false, ai: true, file: title, files: usedFiles, error: r.error, report: combined });
    return;
  }
  send(ws, { type: 'log_analyze_done', ok: true, ai: true, file: title, files: usedFiles, report: r.text });
}

function listBackups() {
  if (!fs.existsSync(BACKUP_DIR)) return [];
  const files = fs.readdirSync(BACKUP_DIR).filter(
    (f) => (f.startsWith('backup_') || f.startsWith('param_')) && f.endsWith('.json'));
  const out = [];
  for (const f of files) {
    try {
      const data = JSON.parse(fs.readFileSync(path.join(BACKUP_DIR, f), 'utf-8'));
      out.push({
        file: f,
        timestamp: data.timestamp || '',
        fcType: data.fc_type || '',
        count: Object.keys(data.params || {}).length,
        type: f.startsWith('param_') ? '全量参数' : '一键首飞',
      });
    } catch (e) { /* skip bad file */ }
  }
  out.sort((a, b) => (a.timestamp < b.timestamp ? 1 : -1));
  return out;
}

// ==================== 故障速查: 记录存储 ====================
const faultStore = createFaultStore(FAULT_DIR);

// ==================== 全局 AI 配置 ====================
const aiConfigStore = createAiConfigStore(AI_CONFIG_FILE);

/** 全局配置是否可用于发起 AI 请求(可免 Key 的服务商除外) */
function aiReady(cfg) {
  if (!cfg || !cfg.provider || !cfg.model) return false;
  const info = providerInfo(cfg.provider);
  return !!cfg.apiKey || !!(info && info.keyOptional);
}

async function faultAnalyze(ws, msg) {
  const rec = msg.record || {};
  if (!String(rec.title || '').trim() && !String(rec.symptom || '').trim()) {
    send(ws, { type: 'fault_analyze_done', ok: false, error: '请先填写故障标题或现象描述' });
    return;
  }
  const ai = aiConfigStore.resolve(msg);
  if (!aiReady(ai)) {
    send(ws, { type: 'fault_analyze_done', ok: false, error: '尚未配置可用的全局 AI，请先在「AI 设置」中完成配置' });
    return;
  }
  send(ws, { type: 'fault_analyze_start' });
  const r = await aiChat({
    provider: ai.provider,
    apiKey: ai.apiKey,
    model: ai.model,
    baseUrl: ai.baseUrl,
    system: FAULT_SYSTEM_PROMPT,
    user: buildFaultUserPrompt(rec),
    timeoutMs: 180000,
  });
  if (r.error) {
    send(ws, { type: 'fault_analyze_done', ok: false, error: r.error });
    return;
  }
  send(ws, { type: 'fault_analyze_done', ok: true, report: r.text, at: new Date().toISOString() });
}

async function restoreBackup(ws, file) {
  const safe = path.basename(file);
  const full = path.join(BACKUP_DIR, safe);
  if (!fs.existsSync(full)) { send(ws, { type: 'error', message: '备份文件不存在' }); return; }
  const data = JSON.parse(fs.readFileSync(full, 'utf-8'));
  const params = data.params || {};
  let success = 0;
  const failed = [];
  const entries = Object.entries(params);
  if (!entries.length) {
    send(ws, { type: 'error', message: `备份 ${safe} 不含任何参数(0 项), 已取消恢复` });
    send(ws, { type: 'backup_restore_done', success: 0, failed: [], file: safe, empty: true });
    return;
  }
  for (let i = 0; i < entries.length; i++) {
    const [name, value] = entries[i];
    const ptype = paramCacheAll[name] && paramCacheAll[name].type ? paramCacheAll[name].type : 9;
    const ok = await link.setParam(name, value, ptype);
    if (ok) success++; else failed.push(name);
    send(ws, { type: 'backup_restore_progress', done: i + 1, total: entries.length, name, ok });
  }
  send(ws, { type: 'backup_restore_done', success, failed, file: safe });
}

// ==================== WebSocket ====================
wss.on('connection', (ws) => {
  clients.add(ws);
  console.log('WebSocket 客户端已连接');
  // 连接时推送当前状态
  send(ws, { type: 'heartbeat', flightMode: telemetry.mode, armed: telemetry.armed });
  send(ws, { type: 'ai_config', config: aiConfigStore.getPublic() });

  ws.on('close', () => {
    clients.delete(ws);
    console.log('WebSocket 客户端已断开');
  });

  ws.on('message', async (raw) => {
    let msg;
    try { msg = JSON.parse(raw.toString()); } catch (e) {
      send(ws, { type: 'error', message: '消息格式错误' });
      return;
    }
    try {
      await handleClientMessage(ws, msg);
    } catch (err) {
      console.error('处理消息出错:', err);
      send(ws, { type: 'error', message: err.message });
    }
  });
});

async function handleClientMessage(ws, msg) {
  switch (msg.type) {
    case 'list_ports': {
      const ports = await SerialPort.list();
      send(ws, { type: 'ports', ports });
      return;
    }
    case 'connect': {
      openSerial(msg.path, Number(msg.baud) || 115200);
      return;
    }
    case 'disconnect': {
      if (serialPort) serialPort.close();
      return;
    }
    case 'request_params': {
      const l = requireLink(ws); if (!l) return;
      l.requestParamList();
      return;
    }
    case 'set_param': {
      const l = requireLink(ws); if (!l) return;
      await withLock(async () => {
        const ok = await l.setParam(msg.paramId, msg.paramValue, msg.paramType || 9);
        if (ok) {
          broadcast({ type: 'param_set_sent', paramId: msg.paramId, paramValue: msg.paramValue });
        } else {
          send(ws, { type: 'error', message: `参数 ${msg.paramId} 写入失败或未确认` });
        }
      });
      return;
    }
    case 'reboot': {
      const l = requireLink(ws); if (!l) return;
      await l.sendCommand(MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN, [1, 0, 0, 0, 0, 0, 0], { timeout: 3000 });
      send(ws, { type: 'reboot_sent' });
      return;
    }
    case 'send_heartbeat': {
      const l = requireLink(ws); if (!l) return;
      l.sendHeartbeat();
      return;
    }
    case 'mavlink_send': {
      if (serialPort && serialPort.isOpen && Array.isArray(msg.data)) {
        serialPort.write(Buffer.from(msg.data));
      }
      return;
    }

    // ---------- 一键首飞 ----------
    case 'ot_read': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => otRead(ws, msg.params || []));
      return;
    }
    case 'ot_write': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => otWrite(ws, msg.items || [], { skipBackup: !!msg.skipBackup }));
      return;
    }
    case 'ot_cal': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => otCal(ws, msg.kind));
      return;
    }
    case 'ot_accel_cal_start': {
      const l = requireLink(ws); if (!l) return;
      await startAccelCal(ws);
      return;
    }
    case 'ot_accel_cal_confirm': {
      const l = requireLink(ws); if (!l) return;
      accelCalConfirm(ws, msg.pos);
      return;
    }
    case 'ot_accel_cal_abort': {
      if (!accelCal) return;  // 已经结束, 不重复报告失败
      accelCal = null;
      send(ws, { type: 'ot_accel_cal_done', ok: false, text: '已中止(需重启飞控才能彻底退出校准)' });
      return;
    }
    case 'ot_spin_start': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => otSpinTest(ws));
      return;
    }
    case 'backup_list': {
      send(ws, { type: 'backup_list', backups: listBackups() });
      return;
    }
    case 'backup_restore': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => restoreBackup(ws, msg.file));
      return;
    }
    case 'backup_delete': {
      const safe = path.basename(String(msg.file || ''));
      const full = path.join(BACKUP_DIR, safe);
      if (safe && full.startsWith(BACKUP_DIR) && fs.existsSync(full)) fs.unlinkSync(full);
      send(ws, { type: 'backup_list', backups: listBackups() });
      return;
    }
    // ---------- 全量参数备份/恢复 ----------
    case 'param_backup': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => paramBackupAll(ws, msg.tag));
      return;
    }
    case 'param_restore': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => paramRestore(ws, msg.file));
      return;
    }
    // ---------- 日志 ----------
    case 'log_list': {
      const l = requireLink(ws); if (!l) return;
      await withLock(async () => {
        const logs = await logList();
        send(ws, { type: 'log_list', logs });
      });
      return;
    }
    case 'log_download': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => logDownloadMany(ws, msg.ids || []));
      return;
    }
    case 'log_erase': {
      const l = requireLink(ws); if (!l) return;
      await withLock(() => logErase(ws));
      return;
    }
    case 'log_local_list': {
      send(ws, { type: 'log_local_list', logs: listLocalLogs() });
      return;
    }
    case 'log_local_delete': {
      const safe = path.basename(String(msg.file || ''));
      const full = path.join(LOG_DIR, safe);
      if (safe && full.startsWith(LOG_DIR) && fs.existsSync(full)) fs.unlinkSync(full);
      send(ws, { type: 'log_local_list', logs: listLocalLogs() });
      return;
    }
    case 'log_analyze': {
      await withLock(() => logAnalyze(ws, msg));
      return;
    }
    case 'ai_providers': {
      send(ws, { type: 'ai_providers', providers: providerList() });
      return;
    }
    // ---------- 全局 AI 配置 ----------
    case 'ai_config_get': {
      send(ws, { type: 'ai_config', config: aiConfigStore.getPublic() });
      return;
    }
    case 'ai_config_set': {
      const cfg = Object.assign({}, msg.config || {});
      if (typeof msg.apiKey === 'string' && msg.apiKey.trim()) cfg.apiKey = msg.apiKey;
      const pub = aiConfigStore.set(cfg);
      broadcast({ type: 'ai_config', config: pub });  // 广播给所有标签页/客户端
      return;
    }
    case 'ai_config_clear_key': {
      const pub = aiConfigStore.clearKey();
      broadcast({ type: 'ai_config', config: pub });
      return;
    }
    // ---------- 故障速查 ----------
    case 'fault_list': {
      send(ws, { type: 'fault_list', faults: faultStore.list() });
      return;
    }
    case 'fault_save': {
      // 不占用串口操作锁: 仅读写本地记录文件
      const r = faultStore.save(msg.record || {});
      send(ws, { type: 'fault_saved', file: r.file, id: r.id, updatedAt: r.updatedAt });
      send(ws, { type: 'fault_list', faults: faultStore.list() });
      return;
    }
    case 'fault_delete': {
      const ok = faultStore.remove(msg.file);
      send(ws, { type: 'fault_deleted', file: msg.file, ok });
      send(ws, { type: 'fault_list', faults: faultStore.list() });
      return;
    }
    case 'fault_analyze': {
      // AI 分析与飞控串口无关, 不占锁, 避免长时间阻塞参数读写
      await faultAnalyze(ws, msg);
      return;
    }
    default:
      send(ws, { type: 'error', message: '未知消息类型: ' + msg.type });
  }
}

// ==================== 启动 ====================
server.listen(PORT, HOST, () => {
  console.log(`AP Tune 已启动: http://localhost:${PORT}`);
  if (AUTH_ON) console.log('已启用 HTTP Basic 认证');
  else console.log('⚠ 未启用认证 (公网部署请设置 AUTH_USER / AUTH_PASS)');
  const wantOpen = process.argv.includes('--open') || (process.pkg && !process.argv.includes('--no-open'));
  if (wantOpen) openBrowser(`http://localhost:${PORT}`);
});

function openBrowser(url) {
  const platform = process.platform;
  let cmd;
  if (platform === 'darwin') cmd = `open "${url}"`;
  else if (platform === 'win32') cmd = `cmd /c start "" "${url}"`;
  else cmd = `xdg-open "${url}"`;
  exec(cmd, (err) => { if (err) console.log('请手动打开浏览器: ' + url); });
}
