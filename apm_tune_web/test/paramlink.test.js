// ParamLink 进程内回环单测: 一端当 GCS, 一端当飞控
//   node test/paramlink.test.js
import { ParamLink, MAV_CMD } from '../lib/paramlink.js';
import { MavlinkLink } from '../lib/mavlink.js';

const store = {
  MOT_THST_HOVER: 0.25,
  WPNAV_SPEED: 1500,
  MOT_SPIN_ARM: 0.1,
};

let gcs; // 需要先声明, 供 fc 写回
const fc = new MavlinkLink({
  write: (b) => gcs.feed(b),
  srcSys: 1, srcComp: 1,
  onMessage: (m) => {
    if (m.name === 'PARAM_REQUEST_READ' && m.fields) {
      const name = (m.fields.param_id || '').toUpperCase();
      const val = store[name];
      if (val !== undefined) {
        fc.send('PARAM_VALUE', {
          param_id: name, param_value: val, param_type: 9,
          param_count: Object.keys(store).length, param_index: 0,
        });
      }
    } else if (m.name === 'PARAM_SET' && m.fields) {
      const name = (m.fields.param_id || '').toUpperCase();
      store[name] = m.fields.param_value;
      fc.send('PARAM_VALUE', {
        param_id: name, param_value: m.fields.param_value, param_type: m.fields.param_type || 9,
        param_count: Object.keys(store).length, param_index: 0,
      });
    } else if (m.name === 'COMMAND_LONG' && m.fields) {
      fc.send('COMMAND_ACK', {
        command: m.fields.command, result: 0, progress: 0, result_param2: 0,
        target_system: 255, target_component: 190,
      });
    }
  },
});

gcs = new ParamLink({ write: (b) => fc.feed(b), srcSys: 255, srcComp: 190 });

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) { console.log('  ok  ' + name); }
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

async function main() {
  const v = await gcs.readParam('MOT_THST_HOVER');
  check('readParam MOT_THST_HOVER = 0.25', Math.abs(v - 0.25) < 1e-6, String(v));

  const missing = await gcs.readParam('NOT_EXIST', 300, 0);
  check('readParam 不存在的参数 -> null', missing === null, String(missing));

  const ok = await gcs.setParam('WPNAV_SPEED', 1800);
  check('setParam 回读一致', ok === true && store.WPNAV_SPEED === 1800, JSON.stringify(store));

  const many = await gcs.readMany(Object.keys(store), { concurrency: 3, timeout: 600 });
  check('readMany 全部读到', Object.values(many).every((x) => x !== null), JSON.stringify(many));

  const ack = await gcs.sendCommand(MAV_CMD.PREFLIGHT_CALIBRATION, [0, 0, 1, 0, 0, 0, 0], { timeout: 1500 });
  check('sendCommand 收到 ACK', ack.ok === true && ack.result === 0, JSON.stringify(ack));

  console.log(`\n${failures === 0 ? '全部通过' : failures + ' 项失败'}`);
  gcs.close();
  process.exit(failures === 0 ? 0 : 1);
}

main();
