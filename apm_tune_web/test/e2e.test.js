// 端到端集成测试: pty 模拟飞控 <- server.js <- WebSocket 客户端
//   node test/e2e.test.js
import { spawn } from 'child_process';
import fs from 'fs';
import http from 'http';
import path from 'path';
import { fileURLToPath } from 'url';
import WebSocket from 'ws';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const PORT = 3999;

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

function makeClient() {
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
      const counts = {};
      for (const m of messages) counts[m.type] = (counts[m.type] || 0) + 1;
      const lastErr = messages.filter((m) => m.type === 'error').slice(-3).map((m) => m.message);
      reject(new Error('等待 ' + label + ' 超时; 类型统计: ' + JSON.stringify(counts)
        + ' 最近错误: ' + JSON.stringify(lastErr)));
    }, timeout);
    waiters.push({ pred, resolve, timer });
  });
  const opened = new Promise((resolve, reject) => {
    ws.on('open', resolve);
    ws.on('error', reject);
  });
  return { ws, messages, wait, opened, send: (o) => ws.send(JSON.stringify(o)) };
}

async function main() {
  // 1. 启动模拟飞控
  const fc = spawn('python3', [path.join(__dirname, 'fake_fc.py')], { cwd: ROOT });
  fc.stderr.on('data', (d) => process.env.E2E_VERBOSE && process.stderr.write('[fc] ' + d));
  const fcLines = lineReader(fc);
  const ptyPath = await fcLines.wait('PTY');
  await fcLines.wait('READY');
  console.log('模拟飞控 PTY:', ptyPath);

  // 2. 启动 server
  const srv = spawn(process.execPath, ['server.js'], {
    cwd: ROOT, env: { ...process.env, PORT: String(PORT) },
  });
  srv.stdout.on('data', (d) => process.env.E2E_VERBOSE && process.stdout.write('[srv] ' + d));
  srv.stderr.on('data', (d) => process.env.E2E_VERBOSE && process.stderr.write('[srv] ' + d));
  await lineReader(srv).wait('AP Tune 已启动', 8000);

  const client = makeClient();
  try {
    await client.opened;
    // 3. 连接串口
    client.send({ type: 'connect', path: ptyPath, baud: 115200 });
    await client.wait((m) => m.type === 'connected', 8000, 'connected');
    check('串口连接成功', true);
    await client.wait((m) => m.type === 'heartbeat' && m.flightMode, 5000, 'heartbeat');
    check('收到心跳/飞行模式', true);

    // 4. 体检读取(含别名回退)
    client.send({
      type: 'ot_read',
      params: [
        { name: 'MOT_THST_HOVER', aliases: [] },
        { name: 'WPNAV_SPEED', aliases: [] },
        { name: 'NO_SUCH_PARAM', aliases: [] },
      ],
    });
    const readRes = await client.wait((m) => m.type === 'ot_read_result', 15000, 'ot_read_result');
    const byKey = Object.fromEntries(readRes.results.map((r) => [r.key, r]));
    check('体检读到 MOT_THST_HOVER=0.25', Math.abs(byKey.MOT_THST_HOVER.value - 0.25) < 1e-6,
      JSON.stringify(byKey.MOT_THST_HOVER));
    check('体检读到 WPNAV_SPEED=1500', Math.abs(byKey.WPNAV_SPEED.value - 1500) < 1e-6,
      JSON.stringify(byKey.WPNAV_SPEED));
    check('不存在的参数 value=null', byKey.NO_SUCH_PARAM.value === null,
      JSON.stringify(byKey.NO_SUCH_PARAM));
    check('体检电压提示存在', typeof readRes.volt === 'number' && readRes.volt > 0, String(readRes.volt));

    // 5. 一键写入(自动备份)
    client.send({ type: 'ot_write', items: [
      { name: 'WPNAV_SPEED', value: 1800 },
      { name: 'MOT_SPIN_ARM', value: 0.12 },
    ] });
    const bkMsg = await client.wait((m) => m.type === 'ot_backup', 15000, 'ot_backup');
    check('自动备份数量 > 0', bkMsg.count > 0, JSON.stringify(bkMsg));
    const writeDone = await client.wait((m) => m.type === 'ot_write_done', 20000, 'ot_write_done');
    check('一键写入全部成功', writeDone.success === 2 && writeDone.failed.length === 0,
      JSON.stringify(writeDone));
    check('写入前已自动备份', !!writeDone.backup && writeDone.backupCount > 0,
      JSON.stringify({ backup: writeDone.backup, count: writeDone.backupCount }));

    // 6. 单轴校准
    client.send({ type: 'ot_cal', kind: 'gyro' });
    const cal = await client.wait((m) => m.type === 'ot_cal_done' && m.kind === 'gyro', 12000, 'ot_cal_done');
    check('陀螺校准返回 ACK', cal.ok === true, JSON.stringify(cal));

    // 7. 六面加速度计向导
    client.send({ type: 'ot_accel_cal_start' });
    let pos = await client.wait((m) => m.type === 'ot_accel_cal_prompt' && m.pos >= 1, 10000, 'accel prompt');
    check('六面向导收到第 1 面提示', pos.pos === 1, JSON.stringify(pos));
    for (let i = 1; i <= 6; i++) {
      client.send({ type: 'ot_accel_cal_confirm', pos: i });
      if (i < 6) {
        const next = await client.wait(
          (m) => m.type === 'ot_accel_cal_prompt' && m.pos === i + 1, 8000, 'prompt ' + (i + 1));
        check(`六面向导第 ${i + 1} 面提示`, next.pos === i + 1);
      }
    }
    const accelDone = await client.wait((m) => m.type === 'ot_accel_cal_done', 10000, 'accel done');
    check('六面校准完成', accelDone.ok === true, JSON.stringify(accelDone));

    // 8. 拆桨实测起转
    client.send({ type: 'ot_spin_start' });
    const spinDone = await client.wait((m) => m.type === 'ot_spin_done', 40000, 'ot_spin_done');
    check('拆桨实测完成', spinDone.ok === true, JSON.stringify(spinDone));
    check('起转点=10%', spinDone.spin === 10, String(spinDone.spin));
    check('ARM=0.11 MIN=0.14', spinDone.arm === 0.11 && spinDone.min === 0.14,
      `arm=${spinDone.arm} min=${spinDone.min}`);

    // 9. 备份列表
    client.send({ type: 'backup_list' });
    const list = await client.wait((m) => m.type === 'backup_list', 8000, 'backup_list');
    check('备份列表非空', Array.isArray(list.backups) && list.backups.length >= 1,
      JSON.stringify(list.backups.slice(0, 2)));

    // 10. 全量参数备份 + 恢复
    client.send({ type: 'ot_read', params: [{ name: 'WPNAV_SPEED', aliases: [] }] });
    const before = await client.wait(
      (m) => m.type === 'ot_read_result' && m.results[0] && m.results[0].value !== null,
      15000, 'ot_read(before)');
    const v0 = before.results[0].value;

    client.send({ type: 'param_backup' });
    const bk = await client.wait((m) => m.type === 'param_backup_done', 40000, 'param_backup_done');
    check('全量参数备份成功', bk.ok === true && bk.count > 0 && !!bk.file, JSON.stringify(bk));

    client.send({ type: 'ot_write', items: [{ name: 'WPNAV_SPEED', value: 1999 }] });
    await client.wait((m) => m.type === 'ot_write_done', 20000, 'ot_write(changed)');
    client.send({ type: 'ot_read', params: [{ name: 'WPNAV_SPEED', aliases: [] }] });
    const changed = await client.wait(
      (m) => m.type === 'ot_read_result' && m.results[0] && m.results[0].value === 1999,
      15000, 'ot_read(changed)');
    check('参数已改为 1999', Math.abs(changed.results[0].value - 1999) < 1e-6);

    client.send({ type: 'param_restore', file: bk.file });
    const rd = await client.wait((m) => m.type === 'param_restore_done', 60000, 'param_restore_done');
    check('参数恢复完成', rd.success > 0 && rd.failed.length === 0,
      JSON.stringify({ success: rd.success, failed: rd.failed.length }));
    client.send({ type: 'ot_read', params: [{ name: 'WPNAV_SPEED', aliases: [] }] });
    const restored = await client.wait(
      (m) => m.type === 'ot_read_result' && m.results[0] && m.results[0].value !== null,
      15000, 'ot_read(restored)');
    check('参数已恢复为备份值', Math.abs(restored.results[0].value - v0) < 1e-6,
      `${restored.results[0].value} vs ${v0}`);

    // 10b. 空备份恢复守卫
    const emptyName = 'backup_e2e_empty.json';
    const emptyPath = path.join(ROOT, 'backups', emptyName);
    fs.mkdirSync(path.dirname(emptyPath), { recursive: true });
    fs.writeFileSync(emptyPath,
      JSON.stringify({ timestamp: 'test', fc_type: 'APM_COPTER', params: {} }), 'utf-8');
    try {
      client.send({ type: 'param_restore', file: emptyName });
      const emptyRes = await client.wait(
        (m) => m.type === 'param_restore_done' && m.file === emptyName, 8000, 'empty restore');
      check('空备份被拒绝恢复', emptyRes.empty === true && emptyRes.success === 0,
        JSON.stringify(emptyRes));
    } finally {
      try { fs.unlinkSync(emptyPath); } catch (e) { /* ignore */ }
    }

    // 11. 日志列表 + 下载
    client.send({ type: 'log_list' });
    const ll = await client.wait((m) => m.type === 'log_list', 20000, 'log_list');
    check('获取到 2 条日志', ll.logs.length === 2 && ll.logs.every((x) => x.size > 0),
      JSON.stringify(ll.logs));
    const sizeExpected = Object.fromEntries(ll.logs.map((x) => [x.id, x.size]));

    client.send({ type: 'log_download', ids: [0, 1] });
    const dl = await client.wait((m) => m.type === 'log_download_done', 90000, 'log_download_done');
    check('日志下载成功 2 个', dl.saved.length === 2 && dl.failed.length === 0,
      JSON.stringify({ saved: dl.saved, failed: dl.failed }));
    check('下载大小与飞控一致',
      dl.saved.every((s) => s.size === sizeExpected[s.id]),
      JSON.stringify(dl.saved.map((s) => [s.id, s.size, sizeExpected[s.id]])));

    client.send({ type: 'log_local_list' });
    const local = await client.wait((m) => m.type === 'log_local_list', 10000, 'log_local_list');
    check('本地日志列表非空', local.logs.length >= 2, String(local.logs.length));

    // 12. 日志解析(无 AI)
    client.send({ type: 'log_analyze', file: dl.saved[0].file });
    const an = await client.wait((m) => m.type === 'log_analyze_done', 30000, 'log_analyze_done');
    check('日志解析成功(无AI)', an.ok === true && an.ai === false && /飞行日志摘要/.test(an.report || ''),
      JSON.stringify({ ok: an.ok, ai: an.ai, len: (an.report || '').length }));

    // 12b. 传感器图表数据
    const ch = await client.wait(
      (m) => m.type === 'log_charts' && m.file === dl.saved[0].file, 15000, 'log_charts');
    const chartIds = (ch.charts || []).map((g) => g.id);
    check('传感器图表数据完整',
      chartIds.length >= 6 && ['att', 'gps', 'rcou', 'esc'].every((x) => chartIds.includes(x)),
      JSON.stringify(chartIds));
    const esc = (ch.charts || []).find((g) => g.id === 'esc');
    check('电调图表含多路 RPM', !!esc && esc.series.filter((s) => /^RPM/.test(s.name)).length === 4,
      esc ? JSON.stringify(esc.series.map((s) => s.name)) : 'no esc');

    // 13. AI 分析(mock 服务商)
    const aiSrv = http.createServer((req, res) => {
      let body = '';
      req.on('data', (d) => { body += d; });
      req.on('end', () => {
        res.setHeader('content-type', 'application/json');
        res.end(JSON.stringify({ choices: [{ message: { content: '# 模拟飞行报告\n\n结论: 飞行正常。' } }] }));
      });
    });
    await new Promise((r) => aiSrv.listen(0, '127.0.0.1', r));
    const aiPort = aiSrv.address().port;
    client.send({
      type: 'log_analyze', file: dl.saved[0].file,
      provider: 'custom', apiKey: 'test-key-123', model: 'mock-1',
      baseUrl: `http://127.0.0.1:${aiPort}/v1`,
    });
    const ai = await client.wait((m) => m.type === 'log_analyze_done' && m.ai === true, 30000, 'log_analyze_done(ai)');
    check('AI 报告返回', ai.ok === true && /模拟飞行报告/.test(ai.report || ''),
      JSON.stringify({ ok: ai.ok, error: ai.error }));
    aiSrv.close();

    // 13b. 多日志合并分析(无 AI)
    client.send({ type: 'log_analyze', files: [dl.saved[0].file, dl.saved[1].file] });
    const multi = await client.wait(
      (m) => m.type === 'log_analyze_done' && Array.isArray(m.files) && m.files.length === 2,
      30000, 'log_analyze_done(multi)');
    check('多日志合并分析', multi.ok === true && multi.files.length === 2
      && (multi.report || '').includes(dl.saved[0].file) && (multi.report || '').includes(dl.saved[1].file),
      JSON.stringify({ ok: multi.ok, files: multi.files }));

    // 14. 清除飞控日志
    client.send({ type: 'log_erase' });
    const er = await client.wait((m) => m.type === 'log_erase_done', 30000, 'log_erase_done');
    check('清除飞控日志成功', er.ok === true, JSON.stringify(er));

    console.log(`\n${failures === 0 ? '端到端全部通过' : failures + ' 项失败'}`);
  } catch (err) {
    console.error('测试异常:', err.message);
    failures++;
  } finally {
    try { client.ws.close(); } catch (e) { /* ignore */ }
    srv.kill('SIGKILL');
    fc.kill('SIGKILL');
    await sleep(300);
    if (!process.env.E2E_VERBOSE) {
      // 汇总
      const bad = results.filter((r) => !r.cond);
      if (bad.length) console.log('失败项: ' + bad.map((b) => b.name).join('; '));
    }
    process.exit(failures === 0 ? 0 : 1);
  }
}

main();
