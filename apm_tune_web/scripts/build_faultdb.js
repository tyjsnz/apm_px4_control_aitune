#!/usr/bin/env node
// 警告速查知识库构建: docs/*.csv -> public/faultdb.js
//   node scripts/build_faultdb.js
// 数据源:
//   docs/(通用警告，多轴警告，固定翼警告).csv   主表 (制表 20260316, Copter/Plane 4.6.3)
//   docs/(PreArm补充-官方文档).csv             ardupilot.org Pre-Arm Safety Checks 补充
// public/faultdb.js 为生成物, 勿手改; 修改/新增 CSV 后重新运行本脚本
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const DOCS = path.join(ROOT, 'docs');
const OUT = path.join(ROOT, 'public', 'faultdb.js');
const MAIN_CSV = '(通用警告，多轴警告，固定翼警告).csv';
const WIKI_CSV = '(PreArm补充-官方文档).csv';
const CAT_ORDER = ['通用警告', '仅多轴或直升机的警告', '仅固定翼的警告'];

function parseCSV(text) {
  const rows = [];
  let row = [];
  let field = '';
  let inQ = false;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (inQ) {
      if (c === '"') {
        if (text[i + 1] === '"') { field += '"'; i++; } else inQ = false;
      } else field += c;
    } else if (c === '"') inQ = true;
    else if (c === ',') { row.push(field); field = ''; }
    else if (c === '\n') { row.push(field); rows.push(row); row = []; field = ''; }
    else if (c !== '\r') field += c;
  }
  if (field.length || row.length) { row.push(field); rows.push(row); }
  return rows;
}

function loadRows(file) {
  const raw = fs.readFileSync(path.join(DOCS, file), 'utf8').replace(/^\uFEFF/, '');
  const rows = parseCSV(raw);
  const hi = rows.findIndex((r) => r[0] && r[0].trim() === '警告类型');
  if (hi < 0) throw new Error(`${file}: 找不到表头行 (警告类型,...)`);
  const header = rows[hi];
  const data = rows.slice(hi + 1).filter((r) => r.length >= 5 && r[0].trim() && r[1].trim());
  return { header, rows: data, pre: rows.slice(0, hi) };
}

const main = loadRows(MAIN_CSV);
const wiki = loadRows(WIKI_CSV);

// ---- meta (取自主表表头前的说明行) ----
const noteLines = main.pre.map((r) => String(r[0] || '').trim()).filter(Boolean);
const note = noteLines.join(' / ');
const dateM = note.match(/制表于：(\d{4})(\d{2})(\d{2})/);
const fwM = note.match(/对应固件版本-\s*([^\s/][^\s]*)/);
const META = {
  csvFile: `docs/${MAIN_CSV}`,
  csvDate: dateM ? `${dateM[1]}-${dateM[2]}-${dateM[3]}` : '',
  firmware: fwM ? fwM[1] : '',
  note,
  wikiFile: `docs/${WIKI_CSV}`,
  wikiUrl: 'https://ardupilot.org/copter/docs/common-prearm-safety-checks.html',
};

// ---- 数据修正: 主表个别行英文与内容不符 (英文写成 Accels..., 中文/原因实为高度差;
//      正确的 Altitude disparity 行已存在于多轴类目), 生成时剔除 ----
const shouldDrop = (r) => r[1].trim() === 'Accels calibrated requires reboot' && r[2].includes('高度差');

let entries = [];
let dropped = 0;
for (const [src, fallback] of [[main, 'csv'], [wiki, 'wiki']]) {
  for (const r of src.rows) {
    if (shouldDrop(r)) { dropped++; continue; }
    const col6 = String(r[5] || '').trim();
    entries.push({
      cat: r[0].trim(),
      en: r[1].trim(),
      zh: r[2].trim(),
      cause: String(r[3] || '').trim(),
      fix: String(r[4] || '').trim(),
      src: col6 ? 'wiki' : fallback,
    });
  }
}

// ---- 同类目内同英文去重 (保留首条) ----
const seen = new Set();
let dups = 0;
entries = entries.filter((e) => {
  const k = `${e.cat}|${e.en.toLowerCase()}`;
  if (seen.has(k)) { dups++; return false; }
  seen.add(k);
  return true;
});

// ---- 校验 ----
const errs = [];
entries.forEach((e, i) => {
  for (const k of ['cat', 'en', 'zh', 'cause', 'fix']) {
    if (!e[k]) errs.push(`#${i} [${e.en || '(空EN)'}] 字段 ${k} 为空`);
  }
  if (!CAT_ORDER.includes(e.cat)) errs.push(`#${i} 未知分类: ${e.cat}`);
  if (!['csv', 'wiki'].includes(e.src)) errs.push(`#${i} 未知数据源: ${e.src}`);
});
if (errs.length) {
  console.error(`CSV 校验失败 (${errs.length} 项):\n` + errs.slice(0, 20).join('\n'));
  process.exit(1);
}

// ---- 分类分组稳定排序 (通用 -> 多轴 -> 固定翼, 组内保持源文件顺序) ----
entries.sort((a, b) => CAT_ORDER.indexOf(a.cat) - CAT_ORDER.indexOf(b.cat));
entries.forEach((e, i) => { e.id = i; });

const counts = {
  total: entries.length,
  csv: entries.filter((e) => e.src === 'csv').length,
  wiki: entries.filter((e) => e.src === 'wiki').length,
  byCat: Object.fromEntries(CAT_ORDER.map((c) => [c, entries.filter((e) => e.cat === c).length])),
};
META.builtAt = new Date().toISOString().slice(0, 10);
META.counts = {
  total: entries.length,
  csv: entries.filter((e) => e.src === 'csv').length,
  wiki: entries.filter((e) => e.src === 'wiki').length,
};

const out = `// 警告速查知识库 — 由 scripts/build_faultdb.js 自动生成, 勿直接编辑
// 数据源: ${META.csvFile} (制表 ${META.csvDate}, ${META.firmware}) + ${META.wikiFile} (${META.wikiUrl})
// 构建: node scripts/build_faultdb.js   共 ${counts.total} 条 (csv ${counts.csv} / wiki ${counts.wiki}), 剔除错误行 ${dropped} 条, 去重 ${dups} 条
(function (g) {
  'use strict';
  const META = ${JSON.stringify(META, null, 1)};
  const CATS = ${JSON.stringify(CAT_ORDER)};
  const CAT_SHORT = {
    '通用警告': '通用',
    '仅多轴或直升机的警告': '多轴·直升机',
    '仅固定翼的警告': '固定翼'
  };
  const ENTRIES = ${JSON.stringify(entries, null, 1)};

  function norm(s) {
    return String(s === undefined || s === null ? '' : s)
      .toLowerCase()
      .replace(/[^a-z0-9\\u4e00-\\u9fff]+/g, '');
  }

  // 单词命中: 原文包含 或 归一化包含 (容忍标点/大小写差异)
  function tokenHit(tok, raw, nrm) {
    if (raw.includes(tok)) return true;
    const t = norm(tok);
    if (!t) return true;
    return nrm.includes(t);
  }

  function score(e, q, qn) {
    const enN = norm(e.en);
    if (qn && enN === qn) return 100;
    if (qn && enN.startsWith(qn)) return 80;
    if (q && e.en.toLowerCase().includes(q)) return 60;
    if (q && e.zh.includes(q)) return 50;
    if (q && (e.cause.includes(q) || e.fix.includes(q))) return 20;
    return 10;
  }

  // 搜索: 多词 AND (中英文均可), cat 为空表示全部分类; 空查询返回该分类全部
  function search(query, cat) {
    let list = ENTRIES;
    if (cat && CATS.indexOf(cat) >= 0) list = list.filter((e) => e.cat === cat);
    const q = String(query === undefined || query === null ? '' : query).trim().toLowerCase();
    if (!q) return list.slice();
    // 剥离 GCS/日志装饰前缀 (PreArm: / failed to ...), 剥空的 token 弃用; 全被剥空时退回原 token
    let tokens = q.split(/\\s+/).filter(Boolean)
      .map((t) => t.replace(/^(prearmfailure|prearm|failedto|failed|warning)[\\s:：_\\-]*/, ''))
      .filter(Boolean);
    if (!tokens.length) tokens = q.split(/\\s+/).filter(Boolean);
    const qn = norm(q);
    const hits = [];
    for (const e of list) {
      const raw = (e.en + ' ' + e.zh + ' ' + e.cause + ' ' + e.fix).toLowerCase();
      const nrm = norm(raw);
      if (tokens.every((t) => tokenHit(t, raw, nrm))) hits.push([score(e, q, qn), e]);
    }
    hits.sort((a, b) => b[0] - a[0]);
    return hits.map((x) => x[1]);
  }

  function byId(id) {
    return ENTRIES[id] && ENTRIES[id].id === id ? ENTRIES[id] : ENTRIES.find((e) => e.id === id) || null;
  }

  g.FaultDB = {
    META, CATS, CAT_SHORT, ENTRIES,
    count: ENTRIES.length,
    search, byId
  };
})(typeof window !== 'undefined' ? window : globalThis);
`;

fs.writeFileSync(OUT, out, 'utf8');
console.log(`已生成 ${path.relative(ROOT, OUT)}: ${counts.total} 条 (csv ${counts.csv} / wiki ${counts.wiki}, 剔除 ${dropped}, 去重 ${dups}), ${(out.length / 1024).toFixed(1)} KB`);
