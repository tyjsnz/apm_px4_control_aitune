// 飞控故障速查: 记录存储 (每记录一个 JSON 文件)
// 独立于 server.js 以便单元测试
import fs from 'fs';
import path from 'path';

export const FAULT_SEVERITIES = ['info', 'low', 'mid', 'high'];
export const FAULT_STATUSES = ['open', 'monitor', 'fixed'];

function str(v, max = 20000) {
  const s = v === undefined || v === null ? '' : String(v);
  return s.length > max ? s.slice(0, max) : s;
}

export function safeFile(name) {
  const safe = path.basename(String(name || ''));
  return /^fault_[\w.-]+\.json$/.test(safe) ? safe : null;
}

function safeId(id) {
  const s = str(id, 60).replace(/[^\w.-]/g, '_');
  if (s) return s;
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  const stamp = `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
  // 加随机后缀, 避免同一秒内新建多条记录时文件名冲突被覆盖
  const rand = Math.random().toString(36).slice(2, 7);
  return `${stamp}_${rand}`;
}

export function normalizeFault(rec = {}, prev = null) {
  const now = new Date().toISOString();
  let tags;
  if (Array.isArray(rec.tags)) {
    tags = rec.tags.map((t) => str(t, 40).trim()).filter(Boolean);
  } else {
    tags = str(rec.tags, 200).split(/[,，\s]+/).filter(Boolean);
  }
  return {
    id: (prev && prev.id) || safeId(rec.id),
    createdAt: (prev && prev.createdAt) || now,
    updatedAt: now,
    title: str(rec.title, 200).trim() || '未命名故障',
    aircraft: str(rec.aircraft, 120).trim(),
    firmware: str(rec.firmware, 120).trim(),
    severity: FAULT_SEVERITIES.includes(rec.severity) ? rec.severity : 'mid',
    status: FAULT_STATUSES.includes(rec.status) ? rec.status : 'open',
    tags: tags.slice(0, 20),
    symptom: str(rec.symptom),
    context: str(rec.context),
    cause: str(rec.cause),
    fix: str(rec.fix),
    aiAnalysis: str(rec.aiAnalysis, 60000),
    aiAt: str(rec.aiAt, 40),
  };
}

export function createFaultStore(dir) {
  fs.mkdirSync(dir, { recursive: true });

  function list() {
    if (!fs.existsSync(dir)) return [];
    const files = fs.readdirSync(dir).filter((f) => f.startsWith('fault_') && f.endsWith('.json'));
    const out = [];
    for (const f of files) {
      try {
        const data = JSON.parse(fs.readFileSync(path.join(dir, f), 'utf-8'));
        out.push(Object.assign({ file: f }, data));
      } catch (e) { /* skip bad file */ }
    }
    out.sort((a, b) => String(b.updatedAt || '').localeCompare(String(a.updatedAt || '')));
    return out;
  }

  function get(file) {
    const safe = safeFile(file);
    if (!safe) return null;
    const full = path.join(dir, safe);
    if (!full.startsWith(dir) || !fs.existsSync(full)) return null;
    try { return JSON.parse(fs.readFileSync(full, 'utf-8')); } catch (e) { return null; }
  }

  function save(rec = {}) {
    const safe = rec.file ? safeFile(rec.file) : null;
    const prev = safe ? get(safe) : null;
    const data = normalizeFault(rec, prev);
    const file = safe || `fault_${data.id}.json`;
    fs.writeFileSync(path.join(dir, file), JSON.stringify(data, null, 2), 'utf-8');
    return { file, id: data.id, updatedAt: data.updatedAt };
  }

  function remove(file) {
    const safe = safeFile(file);
    if (!safe) return false;
    const full = path.join(dir, safe);
    if (!full.startsWith(dir) || !fs.existsSync(full)) return false;
    fs.unlinkSync(full);
    return true;
  }

  return { dir, list, get, save, remove };
}
