// 警告速查知识库单测: public/faultdb.js (由 scripts/build_faultdb.js 生成)
//   node test/faultdb.test.js
import '../public/faultdb.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log('  ok  ' + name);
  else { console.log('  FAIL ' + name + (detail ? ' -> ' + detail : '')); failures++; }
}

const DB = globalThis.FaultDB;
check('FaultDB 已加载且含 search/byId', !!DB && typeof DB.search === 'function' && typeof DB.byId === 'function');
if (!DB) { console.log('\n失败'); process.exit(1); }

check('条目数 >= 240', DB.count >= 240, String(DB.count));
check('id 唯一且连续', new Set(DB.ENTRIES.map((e) => e.id)).size === DB.count
  && DB.ENTRIES.every((e, i) => e.id === i));
check('分类全部合法', DB.ENTRIES.every((e) => DB.CATS.includes(e.cat)));
check('四要素字段完整 (en/zh/cause/fix)', DB.ENTRIES.every((e) => e.en && e.zh && e.cause && e.fix));

const csvN = DB.ENTRIES.filter((e) => e.src === 'csv').length;
const wikiN = DB.ENTRIES.filter((e) => e.src === 'wiki').length;
check('CSV 条目 = 249 (源 250 - 剔除错配行 1)', csvN === 249, String(csvN));
check('官方文档补充 = 11', wikiN === 11, String(wikiN));

check('错配行已剔除 (EN=Accels... 且 zh=高度差)',
  !DB.ENTRIES.some((e) => e.en === 'Accels calibrated requires reboot' && e.zh.includes('高度差')));
check('正确行 Altitude disparity 在库', DB.ENTRIES.some((e) => e.en === 'Altitude disparity'));
check('剔除后 Accels calibrated requires reboot 仅 1 条',
  DB.ENTRIES.filter((e) => e.en === 'Accels calibrated requires reboot').length === 1);

// 搜索
check('搜索: 中文 (指南针)', DB.search('指南针').length > 0);
check('搜索: 英文 (baro)', DB.search('baro').some((e) => /Baro/.test(e.en)));
check('搜索: 整句英文 (3D Accel calibration)',
  DB.search('3d accel calibration needed').some((e) => e.en === '3D Accel calibration needed'));
check('搜索: PreArm 前缀容错',
  DB.search('PreArm: check mag').some((e) => /Check mag field/.test(e.en)));
check('搜索: 多词 AND 收紧 (gps unhealthy)',
  DB.search('gps unhealthy').length > 0 && DB.search('gps unhealthy').length < DB.search('gps').length);
check('搜索: 分类过滤 (仅固定翼)',
  DB.search('', '仅固定翼的警告').length > 0
  && DB.search('', '仅固定翼的警告').every((e) => e.cat === '仅固定翼的警告'));
check('搜索: 分类 + 关键词联动 (通用 + baro)',
  DB.search('baro', '通用警告').every((e) => e.cat === '通用警告'));
check('搜索: 无匹配返回空', DB.search('zzz_not_a_warning_xyz').length === 0);
check('搜索: 空查询返回全部', DB.search('').length === DB.count);

// 打分: 精确英文应排最前
const r1 = DB.search('compass not healthy');
check('排序: 精确英文命中排第一', r1.length > 0 && /^Compass not healthy$/i.test(r1[0].en), r1[0] && r1[0].en);

// byId
check('byId 往返', DB.byId(0) === DB.ENTRIES[0] && DB.byId(DB.count - 1) === DB.ENTRIES[DB.count - 1]);
check('byId 越界返回 null', DB.byId(999999) === null);

// META
check('META 含数据源信息',
  !!DB.META && !!DB.META.csvFile && DB.META.csvDate === '2026-03-16'
  && DB.META.firmware === 'Copter/Plane4.6.3'
  && /common-prearm-safety-checks/.test(DB.META.wikiUrl || ''));
check('META.counts 与实际一致',
  DB.META.counts && DB.META.counts.total === DB.count
  && DB.META.counts.csv === csvN && DB.META.counts.wiki === wikiN);
check('CAT_SHORT 覆盖全部分类', DB.CATS.every((c) => !!DB.CAT_SHORT[c]));

console.log(failures === 0 ? `\n警告速查库单测通过 (${DB.count} 条)` : `\n${failures} 项失败`);
process.exit(failures === 0 ? 0 : 1);
