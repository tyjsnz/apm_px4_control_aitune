// 飞控操作层 (浏览器 Web Serial / Node 测试通用)
// 从原 server.js 抽取: MAVLink 消息处理 + 参数读写 + 备份恢复 + 日志下载 + 校准 + 电机测试
// 约束: 只依赖 Buffer 全局 (浏览器用 public/vendor/buffer-shim.js 提供), 不用任何 Node 专有 API
import { ParamLink, MAV_CMD, MAV_RESULT, MOTOR_TEST_THROTTLE_PERCENT } from './paramlink.js';

// ArduCopter 飞行模式表 (custom_mode)
export const COPTER_MODES = [
  'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL',
  'CIRCLE', 'LAND', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD',
  'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD',
  'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL',
];

export const ACCEL_POS_NAMES = {
  1: 'LEVEL 水平', 2: 'LEFT 左侧', 3: 'RIGHT 右侧',
  4: 'NOSEDOWN 机头朝下', 5: 'NOSEUP 机头朝上', 6: 'BACK 背面朝上',
};

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * @typedef {object} FcStore  备份/日志的持久化后端
 * @property {(name:string, text:string)=>Promise<void>} saveText  保存 JSON 备份
 * @property {(name:string, data:Uint8Array)=>Promise<void>} saveBin  保存二进制日志
 * @property {(name:string)=>Promise<string>} loadText  读取 JSON 备份内容
 */

export class FcOps {
  /**
   * @param {object} opts
   * @param {(buf:Uint8Array)=>void} opts.write  底层串口写函数 (attach 前可为空实现)
   * @param {(msg:object)=>void} opts.emit       消息输出 (原服务器 broadcast/send)
   * @param {FcStore} opts.store                 备份/日志存储
   * @param {()=>void} [opts.log]               日志输出 (默认 console.log)
   */
  constructor(opts = {}) {
    this.writeFn = opts.write || (() => {});
    this.emit = opts.emit || (() => {});
    this.store = opts.store || null;
    this.log = opts.log || ((...a) => console.log(...a));

    this.link = null;             // ParamLink 实例 (attach 后可用)
    this.heartbeatTimer = null;
    this.attached = false;

    this.telemetry = {
      volt: null, cells: null, load: null,
      armed: false, mode: '--', autopilot: null,
      sysid: 1, compid: 1,
    };
    this.lastEsc = { rpm: null, t: 0 };
    this.accelCal = null;
    this.opLock = Promise.resolve();
    this.paramCacheAll = {};       // name -> { value, type }

    // 突发消息队列 (PARAM_VALUE / LOG_ENTRY / LOG_DATA / STATUSTEXT)
    this.msgQueue = [];
    this.MSG_QUEUE_MAX = 50000;
  }

  // ==================== 基础 ====================
  setWrite(fn) { this.writeFn = fn; }

  /** 串口打开后调用: 建链 + 心跳 + 首次参数拉取 */
  attach({ path = '', baud = 0 } = {}) {
    if (this.link) this.detach();
    this.paramCacheAll = {};
    this.telemetry.volt = null;
    this.telemetry.cells = null;
    this.telemetry.autopilot = null;
    this.link = new ParamLink({
      write: (buf) => { try { this.writeFn(buf); } catch (e) { /* ignore */ } },
      onMessage: (m) => this.handleMavlink(m),
    });
    this.attached = true;
    this.log(`飞控链路已建立: ${path} @ ${baud}`);
    this.emit({ type: 'connected', path, baud });
    this.startHeartbeat();
    this.link.sendHeartbeat();
    setTimeout(() => { try { this.link.requestParamList(); } catch (e) { /* ignore */ } }, 500);
  }

  /** 串口关闭后调用 */
  detach() {
    this.stopHeartbeat();
    if (this.accelCal) {
      this.accelCal = null;
      this.emit({ type: 'ot_accel_cal_done', ok: false, text: '连接断开，校准中止' });
    }
    if (this.link) { this.link.close(); this.link = null; }
    this.paramCacheAll = {};
    this.attached = false;
    this.telemetry.volt = null;
    this.telemetry.armed = false;
    this.telemetry.autopilot = null;
    this.emit({ type: 'disconnected' });
  }

  feed(chunk) {
    if (!this.link) return;
    try { this.link.feed(chunk); } catch (err) { this.log('MAVLink 解析错误: ' + err.message); }
  }

  requireLink() {
    if (!this.link || !this.attached) {
      this.emit({ type: 'error', message: '未连接飞控' });
      return null;
    }
    return this.link;
  }

  startHeartbeat() {
    this.stopHeartbeat();
    this.heartbeatTimer = setInterval(() => {
      if (this.link && this.attached) {
        try { this.link.sendHeartbeat(); } catch (e) { /* ignore */ }
      }
    }, 1000);
  }

  stopHeartbeat() {
    if (this.heartbeatTimer) { clearInterval(this.heartbeatTimer); this.heartbeatTimer = null; }
  }

  withLock(fn) {
    const run = this.opLock.then(fn, fn);
    this.opLock = run.then(() => {}, () => {});
    return run;
  }

  enqueueMsg(name, fields) {
    this.msgQueue.push({ name, fields });
    if (this.msgQueue.length > this.MSG_QUEUE_MAX) {
      this.msgQueue.splice(0, this.msgQueue.length - this.MSG_QUEUE_MAX);
    }
  }

  takeMsg(name, pred) {
    const i = this.msgQueue.findIndex((x) => x.name === name && (!pred || pred(x.fields)));
    if (i < 0) return null;
    return this.msgQueue.splice(i, 1)[0].fields;
  }

  clearMsg(name) {
    for (let i = this.msgQueue.length - 1; i >= 0; i--) {
      if (this.msgQueue[i].name === name) this.msgQueue.splice(i, 1);
    }
  }

  timestamp() {
    const d = new Date();
    const p = (n) => String(n).padStart(2, '0');
    return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
  }

  fcTypeName() {
    return this.telemetry.autopilot === 3 ? 'APM_COPTER' : 'UNKNOWN';
  }

  // ==================== MAVLink 消息处理 ====================
  handleMavlink(m) {
    const t = this.telemetry;
    if (m.sysid && m.sysid !== this.link.link.srcSys) {
      t.sysid = m.sysid;
      t.compid = m.compid || t.compid;
    }

    switch (m.name) {
      case 'HEARTBEAT': {
        const f = m.fields;
        if (f.autopilot === 8) break; // 忽略其它 GCS
        t.autopilot = f.autopilot;
        t.armed = !!(f.base_mode & 0x80);
        if (f.autopilot === 3 || f.autopilot === 0) {
          t.mode = COPTER_MODES[f.custom_mode] || String(f.custom_mode);
        } else {
          t.mode = String(f.custom_mode);
        }
        this.emit({
          type: 'heartbeat',
          flightMode: t.mode,
          armed: t.armed,
          autopilot: f.autopilot,
          mavType: f.type,
          baseMode: f.base_mode,
          customMode: f.custom_mode,
        });
        break;
      }
      case 'PARAM_VALUE': {
        const f = m.fields;
        this.enqueueMsg('PARAM_VALUE', f);
        const pname = (f.param_id || '').replace(/\0.*$/, '');
        if (pname) this.paramCacheAll[pname] = { value: f.param_value, type: f.param_type };
        this.emit({
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
        this.emit({
          type: 'attitude',
          timestamp: m.fields.time_boot_ms,
          roll: m.fields.roll, pitch: m.fields.pitch, yaw: m.fields.yaw,
          rollspeed: m.fields.rollspeed, pitchspeed: m.fields.pitchspeed,
          yawspeed: m.fields.yawspeed,
        });
        break;
      }
      case 'GLOBAL_POSITION_INT': {
        this.emit({
          type: 'global_position',
          timeBootMs: m.fields.time_boot_ms,
          lat: m.fields.lat, lon: m.fields.lon,
          alt: m.fields.alt, relativeAlt: m.fields.relative_alt,
          vx: m.fields.vx, vy: m.fields.vy, vz: m.fields.vz, hdg: m.fields.hdg,
        });
        break;
      }
      case 'VFR_HUD': {
        this.emit({
          type: 'vfr_hud',
          airspeed: m.fields.airspeed, groundspeed: m.fields.groundspeed,
          heading: m.fields.heading, throttle: m.fields.throttle,
          altitude: m.fields.alt, clambrate: m.fields.climb,
        });
        break;
      }
      case 'SYS_STATUS': {
        const volt = m.fields.voltage_battery > 0 ? m.fields.voltage_battery / 1000 : null;
        if (volt) { t.volt = volt; t.cells = Math.round(volt / 3.7); }
        t.load = m.fields.load / 100;
        this.emit({
          type: 'sys_status',
          voltageBattery: m.fields.voltage_battery,
          currentBattery: m.fields.current_battery,
          batteryRemaining: m.fields.battery_remaining,
          load: m.fields.load,
        });
        break;
      }
      case 'ESC_TELEMETRY_1_TO_4': {
        this.lastEsc = { rpm: m.fields.rpm, t: Date.now() };
        break;
      }
      case 'STATUSTEXT': {
        this.enqueueMsg('STATUSTEXT', m.fields);
        this.emit({
          type: 'statustext',
          severity: m.fields.severity,
          text: (m.fields.text || '').replace(/\0.*$/, ''),
        });
        break;
      }
      case 'LOG_ENTRY': {
        this.enqueueMsg('LOG_ENTRY', m.fields);
        break;
      }
      case 'LOG_DATA': {
        this.enqueueMsg('LOG_DATA', m.fields);
        break;
      }
      case 'COMMAND_LONG': {
        // 六面加速度计校准: 飞控回显 ACCELCAL_VEHICLE_POS
        const f = m.fields;
        if (Number(f.command) === MAV_CMD.ACCELCAL_VEHICLE_POS && this.accelCal) {
          const pos = Number(f.param1);
          if (pos >= 16777215) {
            const ok = pos === 16777215;
            this.accelCal = null;
            this.emit({ type: 'ot_accel_cal_done', ok, text: '六面加速度计校准完成' });
          } else if (this.accelCal) {
            this.accelCal.pos = pos;
            this.emit({
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

  // ==================== 一键首飞: 体检读取 ====================
  async resolveParam(entry, timeout = 900) {
    const cands = [[entry.name, 1.0]].concat(entry.aliases || []);
    let first = null;
    for (const [name, scale] of cands) {
      const v = await this.link.readParam(name, timeout, 0);
      if (v === null) continue;
      if (first === null) first = { name, scale, value: v / scale };
      if (Math.abs(v) > 1e-9) return { name, scale, value: v / scale };
    }
    if (first) return first;
    return { name: entry.name, scale: 1.0, value: null };
  }

  async otRead(params) {
    // 电压/串数提示依赖 SYS_STATUS, 若尚未收到则短暂等待
    if (this.telemetry.volt === null) {
      const t0 = Date.now();
      while (this.telemetry.volt === null && Date.now() - t0 < 1500) {
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
        const res = await this.resolveParam(p);
        results[i] = { key: p.name, name: res.name, scale: res.scale, value: res.value };
        done++;
        this.emit({
          type: 'ot_read_progress', done, total,
          name: p.name, value: res.value, resolved: res.name,
        });
      }
    };
    await Promise.all(Array.from({ length: Math.min(concurrency, total) }, worker));
    this.emit({
      type: 'ot_read_result',
      results,
      volt: this.telemetry.volt,
      cells: this.telemetry.cells,
    });
  }

  // ==================== 一键首飞: 备份 + 写入 ====================
  /**
   * 备份指定参数。逐个 PARAM_REQUEST_READ 读取, 读不到的回退到参数表缓存
   * (PARAM_REQUEST_LIST 全量回读 / 单条回写的最近值)。
   * 一项都拿不到时抛错, 绝不写出"0 项的空备份"。
   */
  async saveBackup(names, tag) {
    if (!this.store) throw new Error('未配置备份存储');
    const total = names.length;
    if (!total) throw new Error('没有需要备份的参数');
    const vals = await this.link.readMany(names, { concurrency: 4, timeout: 1500, retries: 2 });

    // 逐名读取 → 会话参数缓存, 组装出可备份的参数集
    const collect = (src) => {
      const out = {};
      const missing = [];
      let fromCache = 0;
      for (const n of names) {
        let v = src[n];
        if ((v === null || v === undefined) && this.paramCacheAll[n]) {
          v = this.paramCacheAll[n].value;
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
      const dump = await this.dumpParamList({ timeoutMs: 8000 });
      for (const n of names) {
        if ((vals[n] === null || vals[n] === undefined) && dump.params[n] !== undefined) vals[n] = dump.params[n];
      }
      got = collect(vals);
    }

    const params = got.out;
    const count = Object.keys(params).length;
    if (!count) {
      const noHeartbeat = this.telemetry.autopilot === null || this.telemetry.autopilot === undefined;
      throw new Error(`${total} 项参数全部读取失败`
        + (noHeartbeat ? '(且未收到飞控心跳, 请检查串口/波特率或固件类型)' : '(飞控未响应参数读取)'));
    }
    const ts = this.timestamp();
    const file = `backup_${ts}${tag ? '_' + tag : ''}.json`;
    const data = { timestamp: ts, fc_type: this.fcTypeName(), spec: {}, params };
    await this.store.saveText(file, JSON.stringify(data, null, 2));
    this.log(`备份 ${file}: ${count}/${total} 项 (缓存补齐 ${got.fromCache}, 读取失败 ${got.missing.length})`);
    return { file, count, total, fromCache: got.fromCache, missing: got.missing.length };
  }

  async otWrite(items, opts = {}) {
    const names = items.map((it) => it.name);
    let backup = null;
    if (!opts.skipBackup) {
      try {
        backup = await this.saveBackup(names, 'onetouch');
        this.emit({
          type: 'ot_backup', backup: backup.file, count: backup.count,
          total: backup.total, fromCache: backup.fromCache, missing: backup.missing.length,
        });
      } catch (err) {
        // 不发 error 弹窗: 由前端用确认框呈现"是否跳过备份继续写入"
        this.emit({
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
        || (this.paramCacheAll[it.name] && this.paramCacheAll[it.name].type)
        || 9;
      try {
        ok = await this.link.setParam(it.name, it.value, ptype);
      } catch (err) {
        ok = false;
      }
      if (ok) success++; else failed.push(it.name);
      this.emit({ type: 'ot_write_progress', done: i + 1, total, name: it.name, ok });
    }
    this.emit({
      type: 'ot_write_done', success, failed,
      backup: backup ? backup.file : null,
      backupCount: backup ? backup.count : 0,
      backupTotal: backup ? backup.total : 0,
      skippedBackup: !!opts.skipBackup,
    });
  }

  // ==================== 一键首飞: 校准 ====================
  async otCal(kind) {
    const p1 = kind === 'gyro' ? 1 : 0;
    const p3 = kind === 'baro' ? 1 : 0;
    const p5 = kind === 'level' ? 2 : 0;
    const ack = await this.link.sendCommand(
      MAV_CMD.PREFLIGHT_CALIBRATION, [p1, 0, p3, 0, p5, 0, 0], { timeout: 8000 });
    this.emit({
      type: 'ot_cal_done', kind, ok: ack.ok,
      text: (ack.text && ack.text.length ? ack.text[ack.text.length - 1] : '')
        || (ack.timeout ? '无返回(超时)' : `result=${ack.result}`),
    });
  }

  async startAccelCal() {
    if (this.accelCal) { this.emit({ type: 'error', message: '六面校准已在进行中' }); return; }
    this.accelCal = { pos: null, startedAt: Date.now() };
    this.emit({ type: 'ot_accel_cal_prompt', pos: 0, text: '正在启动校准...' });
    const ack = await this.link.sendCommand(
      MAV_CMD.PREFLIGHT_CALIBRATION, [0, 0, 0, 0, 1, 0, 0], { timeout: 6000 });
    if (!ack.ok && ack.result !== MAV_RESULT.IN_PROGRESS && this.accelCal) {
      this.emit({ type: 'error', message: '启动六面校准失败: ' + JSON.stringify(ack) });
    }
  }

  accelCalConfirm(pos) {
    if (!this.accelCal) { this.emit({ type: 'error', message: '没有进行中的六面校准' }); return; }
    this.link.send('COMMAND_LONG', {
      param1: Number(pos), param2: 0, param3: 0, param4: 0,
      param5: 0, param6: 0, param7: 0,
      command: MAV_CMD.ACCELCAL_VEHICLE_POS,
      target_system: this.link.targetSys, target_component: this.link.targetComp,
      confirmation: 0,
    });
  }

  // ==================== 一键首飞: 拆桨实测起转 ====================
  waitEscRpm(index, spanMs) {
    return new Promise((resolve) => {
      let best = 0;
      const t0 = Date.now();
      const tick = () => {
        if (this.lastEsc.rpm && this.lastEsc.rpm[index] !== undefined) {
          best = Math.max(best, Number(this.lastEsc.rpm[index]) || 0);
        }
        if (Date.now() - t0 >= spanMs) { resolve(best); return; }
        setTimeout(tick, 60);
      };
      tick();
    });
  }

  async otSpinTest() {
    if (this.telemetry.armed) {
      this.emit({ type: 'ot_spin_done', ok: false, text: '飞控已解锁，禁止电机测试' });
      return;
    }
    let escEverSeen = false;
    const spinOf = async (motor) => {
      for (let pct = 6; pct <= 40; pct += 2) {
        const ack = await this.link.sendCommand(
          MAV_CMD.DO_MOTOR_TEST,
          [motor, MOTOR_TEST_THROTTLE_PERCENT, pct, 2, 1, 0, 0],
          { timeout: 2500 });
        if (ack.result !== null && ack.result !== MAV_RESULT.ACCEPTED
            && ack.result !== MAV_RESULT.IN_PROGRESS) {
          throw new Error('电机测试被拒 (result=' + ack.result + ') '
            + (ack.text && ack.text.join('; ') || ''));
        }
        const rpm = await this.waitEscRpm(motor - 1, 800);
        if (this.lastEsc.rpm) escEverSeen = true;
        this.emit({ type: 'ot_spin_progress', motor, pct, rpm });
        if (rpm > 0) return pct;
      }
      return null;
    };

    try {
      const spin1 = await spinOf(1);
      if (spin1 === null && !escEverSeen) {
        this.emit({
          type: 'ot_spin_done', ok: false,
          text: '未收到 ESC 遥测(RPM=0)。需要 DShot + 遥测线；否则请拆桨人工测定 MOT_SPIN_ARM。',
        });
        return;
      }
      if (spin1 === null) {
        this.emit({ type: 'ot_spin_done', ok: false, text: '40% 以内电机1 仍未起转。' });
        return;
      }
      let spin = spin1;
      for (const motor of [2, 3, 4]) {
        let pct = spin;
        let found = false;
        while (pct <= spin + 8) {
          await this.link.sendCommand(
            MAV_CMD.DO_MOTOR_TEST,
            [motor, MOTOR_TEST_THROTTLE_PERCENT, pct, 2, 1, 0, 0],
            { timeout: 2500 });
          const rpm = await this.waitEscRpm(motor - 1, 700);
          this.emit({ type: 'ot_spin_progress', motor, pct, rpm });
          if (rpm > 0) { found = true; break; }
          pct += 2;
        }
        if (found) spin = Math.max(spin, pct);
        else spin = Math.max(spin, spin + 8);
      }
      const arm = Math.round(Math.min(0.2, spin / 100 + 0.01) * 100) / 100;
      const min = Math.round(Math.min(0.25, arm + 0.03) * 100) / 100;
      this.emit({ type: 'ot_spin_done', ok: true, spin, arm, min });
    } catch (err) {
      this.emit({ type: 'ot_spin_done', ok: false, text: err.message });
    }
  }

  // ==================== 参数全量备份 / 恢复 ====================
  /** 全量参数回读(PARAM_REQUEST_LIST + 消息队列), 返回 {params, expect} */
  async dumpParamList(opts = {}) {
    const timeoutMs = opts.timeoutMs || 30000;
    const quietMs = opts.quietMs || 3000;
    const onProgress = opts.onProgress || null;
    const params = {};
    let expect = null;
    this.clearMsg('PARAM_VALUE');
    const t0 = Date.now();
    this.link.requestParamList();
    while (Date.now() - t0 < timeoutMs) {
      const f = this.takeMsg('PARAM_VALUE');
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

  async paramBackupAll(tag) {
    if (!this.store) throw new Error('未配置备份存储');
    const dumped = await this.dumpParamList({
      timeoutMs: 30000,
      onProgress: (count, total) => {
        if (count % 25 === 0) this.emit({ type: 'param_backup_progress', count, total });
      },
    });
    const params = dumped.params;
    let expect = dumped.expect;
    let count = Object.keys(params).length;
    let fromCache = false;
    if (!count) {
      // 全量回读没拿到(链路繁忙/超时)则退回会话内参数缓存, 避免空备份
      for (const [k, v] of Object.entries(this.paramCacheAll)) params[k] = v.value;
      count = Object.keys(params).length;
      fromCache = count > 0;
      if (count) expect = count;
    }
    if (!count) { this.emit({ type: 'param_backup_done', ok: false, error: '未读取到参数' }); return; }
    const ts = this.timestamp();
    const file = `param_${ts}${tag ? '_' + tag : ''}.json`;
    const data = { timestamp: ts, fc_type: this.fcTypeName(), total: expect, params };
    await this.store.saveText(file, JSON.stringify(data, null, 2));
    this.emit({ type: 'param_backup_done', ok: true, file, count, total: expect, fromCache });
  }

  async paramRestore(file) {
    const safe = String(file || '').split(/[\\/]/).pop();
    let data;
    try {
      data = JSON.parse(await this.store.loadText(safe));
    } catch (e) {
      this.emit({ type: 'error', message: '备份文件不存在' });
      return;
    }
    const params = data.params || {};
    const entries = Object.entries(params);
    if (!entries.length) {
      this.emit({ type: 'param_restore_done', success: 0, failed: [], file: safe, empty: true });
      return;
    }
    let success = 0;
    const failed = [];
    for (let i = 0; i < entries.length; i++) {
      const [name, value] = entries[i];
      let ok = false;
      // 用飞控已知的参数类型写回, 避免非浮点参数因类型不符被忽略
      const ptype = this.paramCacheAll[name] && this.paramCacheAll[name].type
        ? this.paramCacheAll[name].type : 9;
      try { ok = await this.link.setParam(name, value, ptype); } catch (e) { ok = false; }
      if (ok) success++; else failed.push(name);
      this.emit({ type: 'param_restore_progress', done: i + 1, total: entries.length, name, ok });
    }
    this.emit({ type: 'param_restore_done', success, failed, file: safe });
  }

  /** 一键首飞页的备份恢复(消息类型 backup_restore, 语义同 param_restore) */
  async restoreBackup(file) {
    const safe = String(file || '').split(/[\\/]/).pop();
    let data;
    try {
      data = JSON.parse(await this.store.loadText(safe));
    } catch (e) {
      this.emit({ type: 'error', message: '备份文件不存在' });
      return;
    }
    const params = data.params || {};
    let success = 0;
    const failed = [];
    const entries = Object.entries(params);
    if (!entries.length) {
      this.emit({ type: 'error', message: `备份 ${safe} 不含任何参数(0 项), 已取消恢复` });
      this.emit({ type: 'backup_restore_done', success: 0, failed: [], file: safe, empty: true });
      return;
    }
    for (let i = 0; i < entries.length; i++) {
      const [name, value] = entries[i];
      const ptype = this.paramCacheAll[name] && this.paramCacheAll[name].type
        ? this.paramCacheAll[name].type : 9;
      const ok = await this.link.setParam(name, value, ptype);
      if (ok) success++; else failed.push(name);
      this.emit({ type: 'backup_restore_progress', done: i + 1, total: entries.length, name, ok });
    }
    this.emit({ type: 'backup_restore_done', success, failed, file: safe });
  }

  // ==================== 日志下载 / 清除 ====================
  async logList(timeoutMs = 8000) {
    const logs = [];
    const seen = new Set();
    let num = null;
    this.clearMsg('LOG_ENTRY');
    const t0 = Date.now();
    this.link.send('LOG_REQUEST_LIST', {
      target_system: this.link.targetSys, target_component: this.link.targetComp,
      start: 0, end: 0xFFFF,
    });
    while (Date.now() - t0 < timeoutMs) {
      const f = this.takeMsg('LOG_ENTRY');
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

  async requestLogBlock(id, offset, length, deadline) {
    const end = offset + length;
    let next = offset;
    const pending = new Map();
    const chunks = [];
    const request = () => this.link.send('LOG_REQUEST_DATA', {
      target_system: this.link.targetSys, target_component: this.link.targetComp,
      id, ofs: next, count: end - next,
    });
    request();
    let lastReq = Date.now();
    while (next < end) {
      if (Date.now() > deadline) return null;
      const f = this.takeMsg('LOG_DATA', (x) => x.id === id);
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

  async downloadLogById(id, onProgress) {
    // 取大小
    this.clearMsg('LOG_ENTRY');
    this.link.send('LOG_REQUEST_LIST', {
      target_system: this.link.targetSys, target_component: this.link.targetComp,
      start: id, end: id,
    });
    let size = null;
    const t0 = Date.now();
    while (Date.now() - t0 < 5000) {
      const f = this.takeMsg('LOG_ENTRY', (x) => x.id === id);
      if (f) { size = f.size; break; }
      await sleep(30);
    }
    if (size === null) return { error: '无法获取日志大小' };
    if (size === 0) return { error: '日志为空' };

    this.clearMsg('LOG_DATA');
    const deadline = Date.now() + Math.max(60000, (size / 8000) * 1000 + 30000);
    const out = Buffer.alloc(size);
    let offset = 0;
    const blockSize = 900;
    while (offset < size) {
      if (Date.now() > deadline) {
        return { error: `下载超时: ${offset}/${size} 字节` };
      }
      const reqLen = Math.min(blockSize, size - offset);
      const block = await this.requestLogBlock(id, offset, reqLen, deadline);
      if (block === null) return { error: `下载超时: ${offset}/${size} 字节` };
      block.copy(out, offset);
      offset += block.length;
      if (onProgress) onProgress(offset, size, id);
    }
    return { data: out };
  }

  async logDownloadMany(ids) {
    if (!this.store) throw new Error('未配置日志存储');
    const saved = [];
    const failed = [];
    for (const id of ids) {
      this.emit({ type: 'log_download_start', id });
      const res = await this.downloadLogById(id, (cur, total) => {
        this.emit({ type: 'log_download_progress', id, current: cur, total });
      });
      if (res.error) {
        failed.push({ id, error: res.error });
        this.emit({ type: 'log_download_error', id, error: res.error });
        continue;
      }
      const ts = this.timestamp();
      const name = `log_${id}_${ts}.bin`;
      await this.store.saveBin(name, res.data);
      saved.push({ id, file: name, size: res.data.length });
      this.emit({ type: 'log_download_saved', id, file: name, size: res.data.length });
    }
    this.emit({ type: 'log_download_done', saved, failed });
  }

  async logErase() {
    this.clearMsg('STATUSTEXT');
    this.clearMsg('LOG_ENTRY');
    this.link.send('LOG_ERASE', {
      target_system: this.link.targetSys, target_component: this.link.targetComp,
    });
    const t0 = Date.now();
    let erased = false;
    while (Date.now() - t0 < 20000) {
      const st = this.takeMsg('STATUSTEXT', (f) => /eras/i.test(f.text || ''));
      if (st) { erased = true; break; }
      const le = this.takeMsg('LOG_ENTRY', (f) => f.num_logs === 0);
      if (le) { erased = true; break; }
      await sleep(50);
    }
    this.emit({ type: 'log_erase_done', ok: erased, text: erased ? '日志已清除' : '清除超时或飞控未确认' });
  }

  // ==================== 客户端消息入口 ====================
  /**
   * 处理一条前端消息(串口相关子集, 协议与原服务器完全一致)。
   * 非串口类消息(AI/故障/备份列表等)由调用方转发给服务器。
   */
  async handle(msg) {
    switch (msg.type) {
      case 'request_params': {
        const l = this.requireLink(); if (!l) return;
        l.requestParamList();
        return;
      }
      case 'set_param': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(async () => {
          const ok = await l.setParam(msg.paramId, msg.paramValue, msg.paramType || 9);
          if (ok) {
            this.emit({ type: 'param_set_sent', paramId: msg.paramId, paramValue: msg.paramValue });
          } else {
            this.emit({ type: 'error', message: `参数 ${msg.paramId} 写入失败或未确认` });
          }
        });
        return;
      }
      case 'reboot': {
        const l = this.requireLink(); if (!l) return;
        await l.sendCommand(MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN, [1, 0, 0, 0, 0, 0, 0], { timeout: 3000 });
        this.emit({ type: 'reboot_sent' });
        return;
      }
      case 'send_heartbeat': {
        const l = this.requireLink(); if (!l) return;
        l.sendHeartbeat();
        return;
      }
      // ---------- 一键首飞 ----------
      case 'ot_read': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.otRead(msg.params || []));
        return;
      }
      case 'ot_write': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.otWrite(msg.items || [], { skipBackup: !!msg.skipBackup }));
        return;
      }
      case 'ot_cal': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.otCal(msg.kind));
        return;
      }
      case 'ot_accel_cal_start': {
        const l = this.requireLink(); if (!l) return;
        await this.startAccelCal();
        return;
      }
      case 'ot_accel_cal_confirm': {
        const l = this.requireLink(); if (!l) return;
        this.accelCalConfirm(msg.pos);
        return;
      }
      case 'ot_accel_cal_abort': {
        if (!this.accelCal) return;  // 已经结束, 不重复报告失败
        this.accelCal = null;
        this.emit({ type: 'ot_accel_cal_done', ok: false, text: '已中止(需重启飞控才能彻底退出校准)' });
        return;
      }
      case 'ot_spin_start': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.otSpinTest());
        return;
      }
      // ---------- 备份恢复 ----------
      case 'backup_restore': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.restoreBackup(msg.file));
        return;
      }
      // ---------- 全量参数备份/恢复 ----------
      case 'param_backup': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.paramBackupAll(msg.tag));
        return;
      }
      case 'param_restore': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.paramRestore(msg.file));
        return;
      }
      // ---------- 日志 ----------
      case 'log_list': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(async () => {
          const logs = await this.logList();
          this.emit({ type: 'log_list', logs });
        });
        return;
      }
      case 'log_download': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.logDownloadMany(msg.ids || []));
        return;
      }
      case 'log_erase': {
        const l = this.requireLink(); if (!l) return;
        await this.withLock(() => this.logErase());
        return;
      }
      default:
        this.emit({ type: 'error', message: '浏览器端不处理的消息类型: ' + msg.type });
    }
  }
}

export { MAV_CMD, MAV_RESULT };
