#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从官方参数元数据生成前端懒加载用的精简 JSON (含 gzip 副本)

数据源 (与 QGC 同源):
  ArduPilot: https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json
  PX4:       https://artifacts.px4.io/Firmware/_general/parameters.json.xz

用法:
  python scripts/gen_param_meta.py          # 下载(如无缓存)+生成
  python scripts/gen_param_meta.py --offline # 只用 scripts/parammeta_src/ 已有缓存

输出:
  public/parammeta/apm_copter.json  (+ .json.gz)
  public/parammeta/px4.json         (+ .json.gz)

精简后的单条格式 (键名极短以控制体积):
  { n: 显示名, d: 说明(截断), u: 单位, lo: 下限, hi: 上限, v: {值: 标签} }
根级 _meta 记录来源/日期/条数, 查询时跳过下划线开头的键。
"""
import gzip
import json
import lzma
import os
import sys
import urllib.request
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
SRC = os.path.join(HERE, 'parammeta_src')
OUT = os.path.join(ROOT, 'public', 'parammeta')

AP_URL = 'https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json'
PX4_URL = 'https://artifacts.px4.io/Firmware/_general/parameters.json.xz'

DESC_CAP = 500
VALUES_CAP = 24   # 可选值表最多保留条数 (超长位掩码表截断)


def fetch(url, dest, offline=False):
    if os.path.exists(dest) and os.path.getsize(dest) > 0:
        print('  使用缓存 %s (%d 字节)' % (os.path.basename(dest), os.path.getsize(dest)))
        return dest
    if offline:
        raise SystemExit('离线模式且缺少缓存: %s' % dest)
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    print('  下载 %s' % url)
    req = urllib.request.Request(url, headers={'User-Agent': 'fuyao-tune-parammeta/1.0'})
    with urllib.request.urlopen(req, timeout=300) as r, open(dest, 'wb') as f:
        f.write(r.read())
    print('  已缓存 %s (%d 字节)' % (os.path.basename(dest), os.path.getsize(dest)))
    return dest


def cap_text(s, cap=DESC_CAP):
    s = ' '.join(str(s).split())
    return s if len(s) <= cap else s[:cap - 1] + '…'


def build_apm(src_path):
    """ArduPilot apm.pdef.json: 顶层按前缀分组, 组内键即完整参数名"""
    d = json.load(open(src_path, encoding='utf-8'))
    out = {}
    for group, items in d.items():
        if group.startswith('_') or not isinstance(items, dict):
            continue
        for name, it in items.items():
            if not isinstance(it, dict):
                continue
            e = {}
            if it.get('DisplayName'):
                e['n'] = cap_text(it['DisplayName'], 80)
            if it.get('Description'):
                e['d'] = cap_text(it['Description'])
            if it.get('Units'):
                e['u'] = str(it['Units'])
            r = it.get('Range')
            if isinstance(r, dict):
                try:
                    e['lo'] = float(r.get('low'))
                    e['hi'] = float(r.get('high'))
                except (TypeError, ValueError):
                    pass
            vals = it.get('Values')
            if isinstance(vals, dict) and vals:
                # 值键是字符串数字, 保留原始映射 (前 VALUES_CAP 条)
                e['v'] = dict(list(vals.items())[:VALUES_CAP])
            if e:
                out[name] = e
    return out


def build_px4(src_path):
    """PX4 parameters.json (xz 解压后): parameters 数组"""
    with lzma.open(src_path, 'rb') as f:
        d = json.loads(f.read().decode('utf-8'))
    out = {}
    for p in d.get('parameters', []):
        name = p.get('name')
        if not name:
            continue
        e = {}
        if p.get('shortDesc'):
            e['n'] = cap_text(p['shortDesc'], 80)
        if p.get('longDesc'):
            e['d'] = cap_text(p['longDesc'])
        if p.get('units'):
            e['u'] = str(p['units'])
        if p.get('min') is not None:
            e['lo'] = p['min']
        if p.get('max') is not None:
            e['hi'] = p['max']
        vals = p.get('values')
        if isinstance(vals, list) and vals:
            m = {}
            for it in vals[:VALUES_CAP]:
                if isinstance(it, dict) and 'value' in it:
                    m[str(it['value'])] = it.get('description', '')
            if m:
                e['v'] = m
        if e:
            out[name] = e
    return out


def emit(key, params, source):
    os.makedirs(OUT, exist_ok=True)
    doc = {
        '_meta': {
            'key': key,
            'source': source,
            'generated': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'count': len(params),
        },
    }
    doc.update(params)
    dest = os.path.join(OUT, key + '.json')
    raw = json.dumps(doc, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    with open(dest, 'wb') as f:
        f.write(raw)
    with open(dest + '.gz', 'wb') as fo, \
            gzip.GzipFile(fileobj=fo, mode='wb', compresslevel=9, mtime=0) as f:
        f.write(raw)
    print('  %s: %d 项, JSON %d 字节, gz %d 字节'
          % (key, len(params), len(raw), os.path.getsize(dest + '.gz')))


def main():
    offline = '--offline' in sys.argv
    print('参数元数据生成 (源: ArduPilot autotest + PX4 artifacts)')

    print('[1/2] ArduCopter')
    ap_src = fetch(AP_URL, os.path.join(SRC, 'apm_copter.pdef.json'), offline)
    emit('apm_copter', build_apm(ap_src), AP_URL)

    print('[2/2] PX4')
    px_src = fetch(PX4_URL, os.path.join(SRC, 'px4_parameters.json.xz'), offline)
    emit('px4', build_px4(px_src), PX4_URL)

    print('完成 -> %s' % os.path.normpath(OUT))


if __name__ == '__main__':
    main()
