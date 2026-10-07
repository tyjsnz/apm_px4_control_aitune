#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成前端常量/帮助文本 -> public/onetouch_data.js (避免长文本手工转写)
数据来源: docs/ai_pid_tune.py
用法: python3 scripts/gen_ot_data.py
"""
import json
import os
import sys

BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(BASE)
sys.path.insert(0, os.path.join(ROOT, 'docs'))

import ai_pid_tune as ref  # noqa: E402

OUT = os.path.join(ROOT, 'public', 'onetouch_data.js')

data = {
    'help': ref.OT_HELP,
    'purposes': ref._OT_PURPOSES,
    'stageOrder': [[sid, label] for sid, label in ref._OT_STAGES],
    'sizes': {str(k): v for k, v in ref._SIZE_PROFILES.items()},
    'profiles': ref._OT_PROFILES,
    'trackProfiles': ref._TRACK_PROFILES,
    'trackFirst': ref._TRACK_FIRST,
    'autoProfiles': ref._AUTO_PROFILES,
}

with open(OUT, 'w', encoding='utf-8') as fh:
    fh.write('// 本文件由 scripts/gen_ot_data.py 自动生成, 请勿手工编辑。\n')
    fh.write('// 数据来源: docs/ai_pid_tune.py\n')
    fh.write('window.OT_DATA = ')
    json.dump(data, fh, ensure_ascii=False, indent=1)
    fh.write(';\n')

print('已写出 %s' % os.path.normpath(OUT))
print('  purposes=%d stages=%d sizes=%d profiles=%d help=%d 字'
      % (len(data['purposes']), len(data['stageOrder']), len(data['sizes']),
         len(data['profiles']), len(data['help'])))
