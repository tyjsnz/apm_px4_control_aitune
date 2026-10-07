#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用参考实现 ai_pid_tune._ot_stages() 生成 golden 方案, 供 JS 移植比对。

用法: python3 scripts/gen_golden_plan.py
输出: test/golden/plans.json
"""
import json
import os
import sys

BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(BASE)
sys.path.insert(0, os.path.join(ROOT, 'docs'))

import ai_pid_tune as ref  # noqa: E402

CASE_MATRIX = [
    # purpose, profile, size, cells, hover, esc_linear, gyro
    ('first', 'soft', 5, 4, 0.25, False, None),
    ('first', 'std', 5, 6, 0.25, False, None),
    ('first', 'hot', 7, 6, 0.25, False, None),
    ('first', 'std', 10, 6, 0.20, False, None),
    ('first', 'std', 20, 12, 0.25, False, None),
    ('first', 'std', 5, 6, 0.25, True, None),
    ('first', 'std', 5, 6, 0.25, False, 40),
    ('guided', 'soft', 5, 6, 0.25, False, None),
    ('guided', 'std', 10, 6, 0.25, False, None),
    ('guided', 'hot', 20, 12, 0.25, True, 20),
    ('auto', 'soft', 5, 4, 0.25, False, None),
    ('auto', 'std', 10, 6, 0.25, False, None),
    ('auto', 'hot', 20, 12, 0.25, False, 15),
]


def main():
    out = []
    for purpose, profile, size, cells, hover, esc, gyro in CASE_MATRIX:
        ctx = {
            'size': size, 'cells': cells, 'hover': hover,
            'esc_linear': esc, 'purpose': purpose, 'profile': profile,
        }
        if gyro is not None:
            ctx['gyro'] = gyro
        stages = ref._ot_stages(ctx)
        out.append({
            'ctx': ctx,
            'stages': [
                {
                    'id': st['id'],
                    'label': st['label'],
                    'items': [
                        {
                            'name': it['name'],
                            'value': it['value'],
                            'unit': it.get('unit'),
                            'rng': list(it['rng']) if it.get('rng') else None,
                            'check': bool(it.get('check', True)),
                            'aliases': [list(a) for a in it.get('aliases', [])],
                            'desc': it.get('desc'),
                            'help': it.get('help'),
                        }
                        for it in st['items']
                    ],
                }
                for st in stages
            ],
        })

    dest = os.path.join(ROOT, 'test', 'golden', 'plans.json')
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with open(dest, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    total = sum(len(st['items']) for c in out for st in c['stages'])
    print('已写出 %s: %d 组方案, 共 %d 项' % (dest, len(out), total))


if __name__ == '__main__':
    main()
