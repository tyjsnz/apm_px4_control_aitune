// 端到端集成测试 (Web Serial 架构)
//   Part 1: lib/fcops.js <- pty 模拟飞控   (等价浏览器端 Web Serial 链路的全部飞控操作)
//   Part 2: server.js HTTP API + WS        (备份/日志存储、日志解析、迁移消息提示)
//   node test/e2e.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import http from 'http';
import path from 'path';
import { fileURLToPath } from 'url';
import WebSocket from 'ws';
import { SerialPort } from 'serialport';
import { FcOps } from '../lib/fcops.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const PORT = 3999;
const OUT = path.join(ROOT, 'test', 'out');
const STORE_DIR = path.join(OUT, 'e2e_store');
const SRV_BACKUP_DIR = path.join(OUT, 'e2e_server_backups');
const SRV_LOG_DIR = path.join(OUT, 'e2e_server_logs');
const SRV_FAULT_DIR = path.join(OUT, 'e2e_server_faults');
const SRV_AI_CONFIG = path.join(OUT, 'e2e_server_ai_config.json');

let failures = 0;
const results = [];
function check(name, cond, detail = '') {
  results.push({ name, cond, detail });
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function lineReader(proc) {
  let buf = '';
  const lines = [];
  const waiters = [];
  proc.stdout.on('data', (d) => {
    buf += d.toString();
    let i;
    while ((i = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, i).replace(/\r$/, '');
      buf = buf.slice(i + 1);
      lines.push(line);
      for (let j = waiters.length - 1; j >= 0; j--) {
        if (line.startsWith(waiters[j].prefix)) {
          const w = waiters.splice(j, 1)[0];
          clearTimeout(w.timer);
          w.resolve(line.slice(w.prefix.length).trim());
        }
      }
    }
  });
  return {
    wait(prefix, timeout = 8000) {
      const hit = lines.find((l) => l.startsWith(prefix));
      if (hit) return Promise.resolve(hit.slice(prefix.length).trim());
      return new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error('等待 ' + prefix + ' 超时')), timeout);
        waiters.push({ prefix, resolve, timer });
      });
    },
  };
}

/** 消息收集 + 条件等待 (与旧版 WS 客户端 helper 语义一致) */
function makeSink() {
  const messages = [];
  const waiters = [];
  const emit = (m) => {
    messages.push(m);
    for (let i = waiters.length - 1; i >= 0; i--) {
      if (waiters[i].pred(m)) {
        const w = waiters.splice(i, 1)[0];
        clearTimeout(w.timer);
        w.resolve(m);
      }
    }
  };
  const wait = (pred, timeout = 15000, label = '消息') => new Promise((resolve, reject) => {
    const hit = messages.find(pred);
    if (hit) { resolve(hit); return; }
    const timer = setTimeout(() => {
      const idx = waiters.findIndex((x) => x.timer === timer);
      if (idx >= 0) waiters.splice(idx, 1);
      const counts = {};
      for (const m of messages) counts[m.type] = (counts[m.type] || 0) + 1;
      const lastErr = messages.filter((m) => m.type === 'error').slice(-3).map((m) => m.message);
      reject(new Error('等待 ' + label + ' 超时; 类型统计: ' + JSON.stringify(counts)
        + ' 最近错误: ' + JSON.stringify(lastErr)));
    }, timeout);
    waiters.push({ pred, resolve, timer });
  });
  return { messages, wait, emit };
}

/** 文件系统备份/日志存储 (Node 环境下的 FcStore 实现) */
const fsStore = {
  async saveText(name, text) { fs.writeFileSync(path.join(STORE_DIR, name), text, 'utf-8'); },
  async saveBin(name, data) { fs.writeFileSync(path.join(STORE_DIR, name), data); },
  async loadText(name) {
    const p = path.join(STORE_DIR, name);
    if (!fs.existsSync(p)) throw new Error('备份文件不存在: ' + name);
    return fs.readFileSync(p, 'utf-8');
  },
};

// ==================== Part 1: FcOps <-> pty 模拟飞控 ====================
async function part1FcOps() {
  fs.rmSync(STORE_DIR, { recursive: true, force: true });
  fs.mkdirSync(STORE_DIR, { recursive: true });

  // 1. 模拟飞控
  const fc = spawn('python3', [path.join(__dirname, 'fake_fc.py')], { cwd: ROOT });
  fc.stderr.on('data', (d) => process.env.E2E_VERBOSE && process.stderr.write('[fc] ' + d));
  const fcLines = lineReader(fc);
  const ptyPath = await fcLines.wait('PTY');
  await fcLines.wait('READY');
  console.log('模拟飞控 PTY:', ptyPath);

  // 2. 串口 (开发依赖 serialport; 生产环境浏览器用 Web Serial)
  const port = new SerialPort({ path: ptyPath, baudRate: 115200 });
  await new Promise((resolve, reject) => {
    port.on('open', resolve);
    port.on('error', reject);
  });

  const sink = makeSink();
  const ops = new FcOps({ emit: sink.emit, store: fsStore, log: (...a) => process.env.E2E_VERBOSE && console.log('[ops]', ...a) });
  port.on('data', (d) => ops.feed(d));
  ops.setWrite((buf) => { if (port.isOpen) port.write(buf); });
  ops.attach({ path: ptyPath, baud: 115200 });

  try {
    await sink.wait((m) => m.type === 'connected', 5000, 'connected');
    check('链路建立 (connected)', true);
    await sink.wait((m) => m.type === 'heartbeat' && m.flightMode, 8000, 'heartbeat');
    check('收到心跳/飞行模式', true);

    // 3. 体检读取(含别名回退)
    await ops.handle({
      type: 'ot_read',
      params: [
        { name: 'MOT_THST_HOVER', aliases: [] },
        { name: 'WPNAV_SPEED', aliases: [] },
        { name: 'NO_SUCH_PARAM', aliases: [] },
      ],
    });
    const readRes = await sink.wait((m) => m.type === 'ot_read_result', 15000, 'ot_read_result');
    const byKey = Object.fromEntries(readRes.results.map((r) => [r.key, r]));
    check('体检读到 MOT_THST_HOVER=0.25', Math.abs(byKey.MOT_THST_HOVER.value - 0.25) < 1e-6,
      JSON.stringify(byKey.MOT_THST_HOVER));
    check('体检读到 WPNAV_SPEED=1500', Math.abs(byKey.WPNAV_SPEED.value - 1500) < 1e-6,
      JSON.stringify(byKey.WPNAV_SPEED));
    check('不存在的参数 value=null', byKey.NO_SUCH_PARAM.value === null,
      JSON.stringify(byKey.NO_SUCH_PARAM));
    check('体检电压提示存在', typeof readRes.volt === 'number' && readRes.volt > 0, String(readRes.volt));

    // 4. 一键写入(自动备份 → 存储层)
    await ops.handle({ type: 'ot_write', items: [
      { name: 'WPNAV_SPEED', value: 1800 },
      { name: 'MOT_SPIN_ARM', value: 0.12 },
    ] });
    const bkMsg = await sink.wait((m) => m.type === 'ot_backup', 15000, 'ot_backup');
    check('自动备份数量 > 0', bkMsg.count > 0, JSON.stringify(bkMsg));
    const writeDone = await sink.wait((m) => m.type === 'ot_write_done', 20000, 'ot_write_done');
    check('一键写入全部成功', writeDone.success === 2 && writeDone.failed.length === 0,
      JSON.stringify(writeDone));
    check('写入前已自动备份', !!writeDone.backup && writeDone.backupCount > 0,
      JSON.stringify({ backup: writeDone.backup, count: writeDone.backupCount }));
    const backupOnDisk = fs.existsSync(path.join(STORE_DIR, writeDone.backup || ''));
    check('备份文件已写入存储层', backupOnDisk, String(writeDone.backup));

    // 5. 单轴校准
    await ops.handle({ type: 'ot_cal', kind: 'gyro' });
    const cal = await sink.wait((m) => m.type === 'ot_cal_done' && m.kind === 'gyro', 12000, 'ot_cal_done');
    check('陀螺校准返回 ACK', cal.ok === true, JSON.stringify(cal));

    // 6. 六面加速度计向导
    ops.handle({ type: 'ot_accel_cal_start' });
    let pos = await sink.wait((m) => m.type === 'ot_accel_cal_prompt' && m.pos >= 1, 10000, 'accel prompt');
    check('六面向导收到第 1 面提示', pos.pos === 1, JSON.stringify(pos));
    for (let i = 1; i <= 6; i++) {
      ops.handle({ type: 'ot_accel_cal_confirm', pos: i });
      if (i < 6) {
        const next = await sink.wait(
          (m) => m.type === 'ot_accel_cal_prompt' && m.pos === i + 1, 8000, 'prompt ' + (i + 1));
        check(`六面向导第 ${i + 1} 面提示`, next.pos === i + 1);
      }
    }
    const accelDone = await sink.wait((m) => m.type === 'ot_accel_cal_done', 10000, 'accel done');
    check('六面校准完成', accelDone.ok === true, JSON.stringify(accelDone));

    // 7. 拆桨实测起转
    await ops.handle({ type: 'ot_spin_start' });
    const spinDone = await sink.wait((m) => m.type === 'ot_spin_done', 40000, 'ot_spin_done');
    check('拆桨实测完成', spinDone.ok === true, JSON.stringify(spinDone));
    check('起转点=10%', spinDone.spin === 10, String(spinDone.spin));
    check('ARM=0.11 MIN=0.14', spinDone.arm === 0.11 && spinDone.min === 0.14,
      `arm=${spinDone.arm} min=${spinDone.min}`);

    // 8. 备份列表 (存储层文件)
    const backupFiles = fs.readdirSync(STORE_DIR).filter((f) => /^backup_.*\.json$/.test(f));
    check('备份列表非空', backupFiles.length >= 1, JSON.stringify(backupFiles.slice(0, 2)));

    // 9. 全量参数备份 + 恢复
    await ops.handle({ type: 'ot_read', params: [{ name: 'WPNAV_SPEED', aliases: [] }] });
    const before = await sink.wait(
      (m) => m.type === 'ot_read_result' && m.results[0] && m.results[0].value !== null,
      15000, 'ot_read(before)');
    const v0 = before.results[0].value;

    await ops.handle({ type: 'param_backup' });
    const bk = await sink.wait((m) => m.type === 'param_backup_done', 40000, 'param_backup_done');
    check('全量参数备份成功', bk.ok === true && bk.count > 0 && !!bk.file, JSON.stringify(bk));

    await ops.handle({ type: 'ot_write', items: [{ name: 'WPNAV_SPEED', value: 1999 }], skipBackup: true });
    await sink.wait((m) => m.type === 'ot_write_done', 20000, 'ot_write(changed)');
    await ops.handle({ type: 'ot_read', params: [{ name: 'WPNAV_SPEED', aliases: [] }] });
    const changed = await sink.wait(
      (m) => m.type === 'ot_read_result' && m.results[0] && m.results[0].value === 1999,
      15000, 'ot_read(changed)');
    check('参数已改为 1999', Math.abs(changed.results[0].value - 1999) < 1e-6);

    await ops.handle({ type: 'param_restore', file: bk.file });
    const rd = await sink.wait((m) => m.type === 'param_restore_done', 60000, 'param_restore_done');
    check('参数恢复完成', rd.success > 0 && rd.failed.length === 0,
      JSON.stringify({ success: rd.success, failed: rd.failed.length }));
    await ops.handle({ type: 'ot_read', params: [{ name: 'WPNAV_SPEED', aliases: [] }] });
    const restored = await sink.wait(
      (m) => m.type === 'ot_read_result' && m.results[0] && m.results[0].value !== null,
      15000, 'ot_read(restored)');
    check('参数已恢复为备份值', Math.abs(restored.results[0].value - v0) < 1e-6,
      `${restored.results[0].value} vs ${v0}`);

    // 9b. 空备份恢复守卫
    const emptyName = 'backup_e2e_empty.json';
    fs.writeFileSync(path.join(STORE_DIR, emptyName),
      JSON.stringify({ timestamp: 'test', fc_type: 'APM_COPTER', params: {} }), 'utf-8');
    await ops.handle({ type: 'param_restore', file: emptyName });
    const emptyRes = await sink.wait(
      (m) => m.type === 'param_restore_done' && m.file === emptyName, 8000, 'empty restore');
    check('空备份被拒绝恢复', emptyRes.empty === true && emptyRes.success === 0,
      JSON.stringify(emptyRes));

    // 10. 日志列表 + 下载 (写入存储层)
    await ops.handle({ type: 'log_list' });
    const ll = await sink.wait((m) => m.type === 'log_list', 20000, 'log_list');
    check('获取到 2 条日志', ll.logs.length === 2 && ll.logs.every((x) => x.size > 0),
      JSON.stringify(ll.logs));
    const sizeExpected = Object.fromEntries(ll.logs.map((x) => [x.id, x.size]));

    await ops.handle({ type: 'log_download', ids: [0, 1] });
    const dl = await sink.wait((m) => m.type === 'log_download_done', 90000, 'log_download_done');
    check('日志下载成功 2 个', dl.saved.length === 2 && dl.failed.length === 0,
      JSON.stringify({ saved: dl.saved, failed: dl.failed }));
    check('下载大小与飞控一致',
      dl.saved.every((s) => s.size === sizeExpected[s.id]),
      JSON.stringify(dl.saved.map((s) => [s.id, s.size, sizeExpected[s.id]])));
    check('日志文件已写入存储层',
      dl.saved.every((s) => fs.existsSync(path.join(STORE_DIR, s.file))),
      JSON.stringify(dl.saved.map((s) => s.file)));

    // 11. 清除飞控日志
    await ops.handle({ type: 'log_erase' });
    const er = await sink.wait((m) => m.type === 'log_erase_done', 30000, 'log_erase_done');
    check('清除飞控日志成功', er.ok === true, JSON.stringify(er));

    return dl.saved;
  } finally {
    try { port.close(); } catch (e) { /* ignore */ }
    fc.kill('SIGKILL');
    await sleep(200);
  }
}

// ==================== Part 2: server HTTP API + WS ====================
function httpPost(urlPath, { json, raw, contentType } = {}) {
  return new Promise((resolve, reject) => {
    const body = json !== undefined ? JSON.stringify(json) : raw;
    const req = http.request({
      host: '127.0.0.1', port: PORT, path: urlPath, method: 'POST',
      headers: { 'content-type': contentType || (json !== undefined ? 'application/json' : 'application/octet-stream') },
    }, (res) => {
      let buf = '';
      res.on('data', (d) => { buf += d; });
      res.on('end', () => resolve({ status: res.statusCode, body: buf }));
    });
    req.on('error', reject);
    req.end(body);
  });
}

function httpGet(urlPath) {
  return new Promise((resolve, reject) => {
    http.get({ host: '127.0.0.1', port: PORT, path: urlPath }, (res) => {
      const chunks = [];
      res.on('data', (d) => chunks.push(d));
      res.on('end', () => resolve({ status: res.statusCode, body: Buffer.concat(chunks) }));
    }).on('error', reject);
  });
}

async function part2Server(logFiles) {
  fs.rmSync(SRV_BACKUP_DIR, { recursive: true, force: true });
  fs.rmSync(SRV_LOG_DIR, { recursive: true, force: true });
  fs.rmSync(SRV_FAULT_DIR, { recursive: true, force: true });
  fs.mkdirSync(SRV_BACKUP_DIR, { recursive: true });
  fs.mkdirSync(SRV_LOG_DIR, { recursive: true });
  fs.mkdirSync(SRV_FAULT_DIR, { recursive: true });

  // 把 Part 1 下载的真实日志放进服务器日志目录, 供解析测试
  const realLog = logFiles[0];
  fs.copyFileSync(path.join(STORE_DIR, realLog.file), path.join(SRV_LOG_DIR, realLog.file));

  const srv = spawn(process.execPath, ['server.js'], {
    cwd: ROOT,
    env: {
      ...process.env,
      PORT: String(PORT),
      BACKUP_DIR: SRV_BACKUP_DIR,
      LOG_DIR: SRV_LOG_DIR,
      FAULT_DIR: SRV_FAULT_DIR,
      AI_CONFIG_FILE: SRV_AI_CONFIG,
      AUTH_USER: '', AUTH_PASS: '',
    },
  });
  srv.stdout.on('data', (d) => process.env.E2E_VERBOSE && process.stdout.write('[srv] ' + d));
  srv.stderr.on('data', (d) => process.env.E2E_VERBOSE && process.stderr.write('[srv] ' + d));
  await lineReader(srv).wait('AP Tune 已启动', 8000);

  const ws = new WebSocket(`ws://127.0.0.1:${PORT}`);
  const messages = [];
  const waiters = [];
  ws.on('message', (raw) => {
    let m;
    try { m = JSON.parse(raw.toString()); } catch (e) { return; }
    messages.push(m);
    for (let i = waiters.length - 1; i >= 0; i--) {
      if (waiters[i].pred(m)) {
        const w = waiters.splice(i, 1)[0];
        clearTimeout(w.timer);
        w.resolve(m);
      }
    }
  });
  const wait = (pred, timeout = 15000, label = '消息') => new Promise((resolve, reject) => {
    const hit = messages.find(pred);
    if (hit) { resolve(hit); return; }
    const timer = setTimeout(() => {
      const idx = waiters.findIndex((x) => x.timer === timer);
      if (idx >= 0) waiters.splice(idx, 1);
      reject(new Error('等待 ' + label + ' 超时; 收到: ' + JSON.stringify(messages.slice(-5))));
    }, timeout);
    waiters.push({ pred, resolve, timer });
  });
  await new Promise((resolve, reject) => { ws.on('open', resolve); ws.on('error', reject); });
  const send = (o) => ws.send(JSON.stringify(o));

  try {
    // 1. 备份上传/读取 API (浏览器端 FcOps.store.saveText / loadText 走这里)
    const backupJson = JSON.stringify({ timestamp: 'api_test', fc_type: 'APM_COPTER', params: { WPNAV_SPEED: 1500 } });
    const up = await httpPost('/api/backups', { json: { name: 'backup_api_test.json', content: backupJson } });
    check('POST /api/backups 成功', up.status === 200 && JSON.parse(up.body).ok === true, up.body);
    const got = await httpGet('/api/backups/backup_api_test.json');
    check('GET /api/backups/:name 内容一致', got.status === 200 && got.body.toString('utf-8') === backupJson);
    const badName = await httpPost('/api/backups', { json: { name: '../evil.json', content: backupJson } });
    check('非法备份文件名被拒绝', badName.status === 400, String(badName.status));

    send({ type: 'backup_list' });
    const bl = await wait((m) => m.type === 'backup_list', 8000, 'backup_list');
    check('WS backup_list 含上传的备份',
      Array.isArray(bl.backups) && bl.backups.some((b) => b.file === 'backup_api_test.json' && b.count === 1),
      JSON.stringify(bl.backups));

    // 2. 日志上传 API (浏览器端 FcOps.store.saveBin 走这里)
    const logBuf = fs.readFileSync(path.join(STORE_DIR, realLog.file));
    const lup = await httpPost('/api/logs?name=log_api_upload.bin', { raw: logBuf });
    check('POST /api/logs 成功', lup.status === 200 && JSON.parse(lup.body).ok === true, lup.body);
    const badLog = await httpPost('/api/logs?name=../evil.bin', { raw: logBuf });
    check('非法日志文件名被拒绝', badLog.status === 400, String(badLog.status));

    send({ type: 'log_local_list' });
    const ll = await wait((m) => m.type === 'log_local_list', 8000, 'log_local_list');
    check('WS log_local_list 含上传的日志',
      ll.logs.some((l) => l.file === 'log_api_upload.bin' && l.size === logBuf.length),
      JSON.stringify(ll.logs));

    // 3. 日志解析 (无 AI, 服务器端解析 + 图表)
    send({ type: 'log_analyze', file: realLog.file });
    const ch = await wait((m) => m.type === 'log_charts' && m.file === realLog.file, 20000, 'log_charts');
    const chartIds = (ch.charts || []).map((g) => g.id);
    check('传感器图表数据完整',
      chartIds.length >= 6 && ['att', 'gps', 'rcou', 'esc'].every((x) => chartIds.includes(x)),
      JSON.stringify(chartIds));
    const an = await wait((m) => m.type === 'log_analyze_done', 30000, 'log_analyze_done');
    check('日志解析成功(无AI)', an.ok === true && an.ai === false && /飞行日志摘要/.test(an.report || ''),
      JSON.stringify({ ok: an.ok, ai: an.ai, len: (an.report || '').length }));

    // 4. 串口类消息已被迁移 → 明确提示
    send({ type: 'ot_read', params: [] });
    const mig = await wait((m) => m.type === 'error' && /迁移/.test(m.message || ''), 8000, 'migrated');
    check('串口类消息返回迁移提示', !!mig);

    // 5. 故障记录 API 仍可用 (服务器存储)
    send({ type: 'fault_save', record: { title: 'e2e 测试', symptom: '无' } });
    const fsaved = await wait((m) => m.type === 'fault_saved', 8000, 'fault_saved');
    check('故障记录保存成功', !!fsaved.file, JSON.stringify(fsaved));
  } finally {
    try { ws.close(); } catch (e) { /* ignore */ }
    srv.kill('SIGKILL');
    await sleep(200);
  }
}

async function main() {
  try {
    const logFiles = await part1FcOps();
    await part2Server(logFiles);
    console.log(`\n${failures === 0 ? '端到端全部通过' : failures + ' 项失败'}`);
  } catch (err) {
    console.error('测试异常:', err.message);
    failures++;
  } finally {
    if (!process.env.E2E_VERBOSE) {
      const bad = results.filter((r) => !r.cond);
      if (bad.length) console.log('失败项: ' + bad.map((b) => b.name).join('; '));
    }
    process.exit(failures === 0 ? 0 : 1);
  }
}

main();
