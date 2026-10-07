// 参数读写 / 命令请求-响应层 (基于 lib/mavlink.js)
// 语义对齐 pymavlink: get_param / set_param / command_long + COMMAND_ACK
import { MavlinkLink, MAVLINK_DEFS } from './mavlink.js';

export const MAV_PARAM_TYPE_REAL32 = 9;
export const MAV_RESULT = {
  ACCEPTED: 0, TEMPORARILY_REJECTED: 1, DENIED: 2, UNSUPPORTED: 3,
  FAILED: 4, IN_PROGRESS: 5, CANCELLED: 6,
};
export const MAV_CMD = {
  NAV_TAKEOFF: 22,
  DO_SET_MODE: 176,
  DO_MOTOR_TEST: 209,
  PREFLIGHT_CALIBRATION: 241,
  PREFLIGHT_REBOOT_SHUTDOWN: 246,
  COMPONENT_ARM_DISARM: 400,
  ACCELCAL_VEHICLE_POS: 42429,
};
export const MOTOR_TEST_THROTTLE_PERCENT = 0;

export class ParamLink {
  /**
   * @param {object} opts
   * @param {(buf:Buffer)=>void} opts.write
   * @param {(msg:object)=>void} [opts.onMessage]  消息旁路回调(非请求响应)
   */
  constructor(opts = {}) {
    this.onMessage = opts.onMessage || null;
    this.link = new MavlinkLink({
      write: opts.write,
      srcSys: opts.srcSys,
      srcComp: opts.srcComp,
      onMessage: (m) => this._dispatch(m),
    });
    this._paramWaiters = []; // {name, resolve, timer}
    this._cmdWaiters = [];   // {command, resolve, text, timer}
    this._msgWaiters = [];   // {name, pred, resolve, timer} 通用单次等待
    this.stats = { paramsRead: 0, paramsWritten: 0, commands: 0, timeouts: 0 };
  }

  feed(chunk) { return this.link.feed(chunk); }

  /** 透传底层发送(如自定义 COMMAND_LONG) */
  send(name, fields, opts) { return this.link.send(name, fields, opts); }

  get version() { return this.link.version; }
  get targetSys() { return this.link.targetSys; }
  get targetComp() { return this.link.targetComp; }

  parmName(pid) {
    return typeof pid === 'string' ? pid.replace(/\0.*$/, '') : '';
  }

  _dispatch(m) {
    if (m.name === 'PARAM_VALUE' && m.fields) {
      const pid = (this.parmName(m.fields.param_id) || '').toUpperCase();
      const idx = this._paramWaiters.findIndex(
        (w) => w.name === pid || (w.index !== undefined && w.index === m.fields.param_index));
      if (idx >= 0) {
        const w = this._paramWaiters.splice(idx, 1)[0];
        clearTimeout(w.timer);
        w.resolve(m.fields.param_value);
        this.stats.paramsRead++;
      }
    } else if (m.name === 'COMMAND_ACK' && m.fields) {
      const cmd = m.fields.command;
      const w = this._cmdWaiters.find((x) => x.command === cmd);
      if (w) {
        w.result = m.fields.result;
        w.progress = m.fields.progress;
        // ACCEPTED / IN_PROGRESS 时若已有 → 立即结束; IN_PROGRESS 继续等
        if (w.result !== MAV_RESULT.IN_PROGRESS) {
          this._finishCmd(w);
        }
      }
    } else if (m.name === 'STATUSTEXT' && m.fields) {
      const text = m.fields.text || '';
      for (const w of this._cmdWaiters) w.text.push(text);
    }
    // 通用单次等待(日志/校准等)
    for (let i = this._msgWaiters.length - 1; i >= 0; i--) {
      const w = this._msgWaiters[i];
      if (w.name === m.name && (!w.pred || w.pred(m))) {
        this._msgWaiters.splice(i, 1);
        clearTimeout(w.timer);
        w.resolve(m);
      }
    }
    if (this.onMessage) this.onMessage(m);
  }

  /** 等待一条指定类型消息; 超时返回 null */
  waitFor(name, pred, timeout = 3000) {
    return new Promise((resolve) => {
      const w = {
        name, pred, resolve,
        timer: setTimeout(() => {
          const i = this._msgWaiters.indexOf(w);
          if (i >= 0) this._msgWaiters.splice(i, 1);
          resolve(null);
        }, timeout),
      };
      this._msgWaiters.push(w);
    });
  }

  _finishCmd(w) {
    const i = this._cmdWaiters.indexOf(w);
    if (i >= 0) this._cmdWaiters.splice(i, 1);
    clearTimeout(w.timer);
    this.stats.commands++;
    w.resolve({ ok: w.result === MAV_RESULT.ACCEPTED, result: w.result, text: w.text });
  }

  /** 请求单个参数(按名), 返回数值或 null。等价 pymavlink get_param */
  readParam(name, timeout = 1500, retries = 1) {
    const upper = String(name).toUpperCase();
    return new Promise((resolve) => {
      let attempt = 0;
      const tryOnce = () => {
        attempt++;
        const waiter = {
          name: upper,
          resolve,
          timer: setTimeout(() => {
            const i = this._paramWaiters.indexOf(waiter);
            if (i >= 0) this._paramWaiters.splice(i, 1);
            if (attempt <= retries) tryOnce();
            else { this.stats.timeouts++; resolve(null); }
          }, timeout),
        };
        this._paramWaiters.push(waiter);
        this.link.send('PARAM_REQUEST_READ', {
          param_index: -1,
          target_system: this.link.targetSys,
          target_component: this.link.targetComp,
          param_id: upper,
        });
      };
      tryOnce();
    });
  }

  /** 按索引请求参数 */
  readParamByIndex(index, timeout = 1500) {
    return new Promise((resolve) => {
      const waiter = {
        name: null, index,
        resolve,
        timer: setTimeout(() => {
          const i = this._paramWaiters.indexOf(waiter);
          if (i >= 0) this._paramWaiters.splice(i, 1);
          this.stats.timeouts++;
          resolve(null);
        }, timeout),
      };
      this._paramWaiters.push(waiter);
      this.link.send('PARAM_REQUEST_READ', {
        param_index: index,
        target_system: this.link.targetSys,
        target_component: this.link.targetComp,
        param_id: '',
      });
    });
  }

  /**
   * 写参数: 发 PARAM_SET, 等同名 PARAM_VALUE 回读比对。
   * 返回 true/false。等价 pymavlink set_param
   */
  async setParam(name, value, type = MAV_PARAM_TYPE_REAL32, timeout = 3000) {
    const upper = String(name).toUpperCase();
    const val = Number(value);
    const echoed = await new Promise((resolve) => {
      const waiter = {
        name: upper,
        resolve: (v) => resolve(v),
        timer: setTimeout(() => {
          const i = this._paramWaiters.indexOf(waiter);
          if (i >= 0) this._paramWaiters.splice(i, 1);
          resolve(null);
        }, timeout),
      };
      this._paramWaiters.push(waiter);
      this.link.send('PARAM_SET', {
        param_value: val,
        target_system: this.link.targetSys,
        target_component: this.link.targetComp,
        param_id: upper,
        param_type: type,
      });
    });
    if (echoed !== null && Math.abs(echoed - val) < 0.01) {
      this.stats.paramsWritten++;
      return true;
    }
    // 回读失败再主动核对一次
    const got = await this.readParam(upper, Math.min(1500, timeout));
    const ok = got !== null && Math.abs(got - val) < 0.01;
    if (ok) this.stats.paramsWritten++;
    return ok;
  }

  /** 批量读取(带并发与进度回调)。返回 {name: value|null} */
  async readMany(names, opts = {}) {
    const concurrency = Math.max(1, opts.concurrency || 4);
    const timeout = opts.timeout || 1200;
    const retries = opts.retries === undefined ? 1 : opts.retries;
    const onProgress = opts.onProgress || null;
    const result = {};
    let done = 0;
    let cursor = 0;
    const total = names.length;
    const worker = async () => {
      while (true) {
        const i = cursor++;
        if (i >= total) return;
        const name = names[i];
        const v = await this.readParam(name, timeout, retries);
        result[name] = v;
        done++;
        if (onProgress) onProgress(done, total, name, v);
      }
    };
    await Promise.all(Array.from({ length: Math.min(concurrency, total) }, worker));
    return result;
  }

  /** 请求全部参数列表(FC 会连续回 PARAM_VALUE) */
  requestParamList() {
    return this.link.send('PARAM_REQUEST_LIST', {
      target_system: this.link.targetSys,
      target_component: this.link.targetComp,
    });
  }

  /**
   * 发送 COMMAND_LONG 并等 COMMAND_ACK。
   * 返回 {ok, result, text[]}
   */
  sendCommand(command, params = [], opts = {}) {
    const p = [0, 0, 0, 0, 0, 0, 0];
    for (let i = 0; i < 7; i++) p[i] = params[i] === undefined ? 0 : params[i];
    const timeout = opts.timeout || 5000;
    return new Promise((resolve) => {
      const waiter = {
        command,
        text: [],
        result: undefined,
        resolve: (r) => resolve(r),
        timer: setTimeout(() => {
          const i = this._cmdWaiters.indexOf(waiter);
          if (i >= 0) this._cmdWaiters.splice(i, 1);
          this.stats.timeouts++;
          resolve({ ok: false, result: null, text: waiter.text, timeout: true });
        }, timeout),
      };
      this._cmdWaiters.push(waiter);
      this.link.send('COMMAND_LONG', {
        param1: p[0], param2: p[1], param3: p[2], param4: p[3],
        param5: p[4], param6: p[5], param7: p[6],
        command,
        target_system: this.link.targetSys,
        target_component: this.link.targetComp,
        confirmation: opts.confirmation || 0,
      });
    });
  }

  sendHeartbeat() {
    return this.link.send('HEARTBEAT', {
      custom_mode: 0,
      type: 6,          // MAV_TYPE_GCS
      autopilot: 8,     // MAV_AUTOPILOT_INVALID
      base_mode: 0,
      system_status: 4, // MAV_STATE_ACTIVE
      mavlink_version: 3,
    });
  }

  close() {
    for (const w of this._paramWaiters) clearTimeout(w.timer);
    for (const w of this._cmdWaiters) clearTimeout(w.timer);
    for (const w of this._msgWaiters) clearTimeout(w.timer);
    this._paramWaiters = [];
    this._cmdWaiters = [];
    this._msgWaiters = [];
  }
}

export { MAVLINK_DEFS };
