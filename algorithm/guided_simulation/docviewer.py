# -*- coding: utf-8 -*-
"""Markdown 帮助文档查看器.

把 ``docs/`` 目录下的 ``.md`` 文件解析成 HTML, 用 PyQt5 的 QTextBrowser 渲染。
不依赖任何第三方 Markdown 库(自实现常用语法子集), 用户可自行增删改 md 文件后
点"刷新"或重新打开帮助窗口即可加载。

支持的 Markdown 语法
--------------------
* 标题 ``#`` ~ ``######``
* 有序/无序列表(支持多级缩进)、段落、引用 ``>``
* 表格(含对齐)、围栏代码块 ``` ``` 、分隔线 ``---``
* 行内: ``**粗体**`` ``*斜体*`` `` `代码` `` 、链接 ``[文字](文件.md)`` 、图片 ``![alt](x.png)``
"""

from __future__ import annotations

import html as _html
import os
import re
from urllib.parse import unquote

from PyQt5.QtCore import Qt, QUrl, pyqtSignal
from PyQt5.QtGui import QDesktopServices, QFont
from PyQt5.QtWidgets import (
    QDialog, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QPushButton, QTextBrowser, QVBoxLayout, QWidget,
)

LIST_RE = re.compile(r'^(\s*)([-*+]|\d+\.)\s+(.*)$')
HR_RE = re.compile(r'^\s*([-*_])(\s*\1\s*){2,}$')
HEADING_RE = re.compile(r'^(#{1,6})\s+(.*?)\s*#*\s*$')
TABLE_SEP_RE = re.compile(r'^\s*\|?\s*:?-{2,}:?\s*(\|\s*:?-{2,}:?\s*)+\|?\s*$')

CSS = """
body { font-family: "Microsoft YaHei UI", "Segoe UI"; font-size: 13px;
       color: #d7e3f7; background-color: #0d1526; }
h1 { color: #7fb3ff; font-size: 21px; margin-top: 14px; margin-bottom: 8px;
     border-bottom: 1px solid #2b4468; padding-bottom: 5px; }
h2 { color: #8fc1ff; font-size: 17px; margin-top: 18px; margin-bottom: 6px; }
h3 { color: #a9d0ff; font-size: 15px; margin-top: 14px; margin-bottom: 5px; }
h4 { color: #c3deff; font-size: 14px; margin-top: 12px; margin-bottom: 4px; }
p  { margin-top: 7px; margin-bottom: 7px; }
ul, ol { margin-top: 6px; margin-bottom: 6px; }
li { margin-top: 4px; margin-bottom: 4px; }
code { font-family: Consolas, "Courier New"; color: #9fe8c9;
       background-color: #101a2e; }
pre  { font-family: Consolas, "Courier New"; color: #cfe3ff;
       background-color: #101a2e; padding: 8px; border: 1px solid #26364f;
       margin-top: 8px; margin-bottom: 8px; }
blockquote { color: #ffd89b; background-color: #14203a; padding: 7px;
             margin-top: 8px; margin-bottom: 8px; border: 1px solid #3a4c70; }
table { border-collapse: collapse; margin-top: 8px; margin-bottom: 8px; }
th { background-color: #16233a; color: #9fc0f0; padding: 5px;
     border: 1px solid #2b4468; }
td { padding: 5px; border: 1px solid #26364f; color: #d7e3f7; }
hr { color: #2b4468; }
a  { color: #6fb3ff; }
b, strong { color: #ffffff; }
"""


# ============================================================== 转换器 ======
def _inline(text: str) -> str:
    """行内 Markdown -> HTML(先转义, 代码片段用占位符保护)."""
    codes: list[str] = []

    def _code(m):
        codes.append(m.group(1))
        return f"\x00{len(codes) - 1}\x00"

    s = _html.escape(text, quote=False)
    s = re.sub(r'`([^`]+)`', _code, s)
    s = re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', s)
    s = re.sub(r'(?<![*\w])\*([^*\n]+)\*(?!\*)', r'<i>\1</i>', s)
    s = re.sub(r'!\[([^\]]*)\]\(([^)]+)\)', r'<img src="\2" alt="\1">', s)
    s = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2">\1</a>', s)
    s = re.sub(r'\x00(\d+)\x00',
               lambda m: f'<code>{codes[int(m.group(1))]}</code>', s)
    return s


def _split_row(line: str) -> list[str]:
    s = line.strip()
    if s.startswith('|'):
        s = s[1:]
    if s.endswith('|'):
        s = s[:-1]
    return [c.strip() for c in s.split('|')]


def _render_list(items: list, k: int = 0) -> tuple[str, int]:
    """items: [indent, ordered, text] -> (html, 下标)."""
    out: list[str] = []
    while k < len(items):
        level, ordered, _ = items[k]
        tag = 'ol' if ordered else 'ul'
        out.append(f'<{tag}>')
        while k < len(items) and items[k][0] == level and items[k][1] == ordered:
            text = items[k][2]
            k += 1
            out.append(f'<li>{_inline(text)}')
            if k < len(items) and items[k][0] > level:
                sub, k = _render_list(items, k)
                out.append(sub)
            out.append('</li>')
        out.append(f'</{tag}>')
        if k < len(items) and items[k][0] < level:
            break                              # 回到上层, 交给调用方
        if k < len(items) and items[k][0] > level:
            sub, k = _render_list(items, k)    # 防御: 不应到达
            out.append(sub)
    return ''.join(out), k


def _is_structural(line: str) -> bool:
    s = line.strip()
    return bool(
        s.startswith('#') or s.startswith('```') or s.startswith('|')
        or s.startswith('>') or LIST_RE.match(line) or HR_RE.match(s)
    )


def md_to_html(md_text: str) -> str:
    """Markdown 文档 -> HTML 片段(不含 <html> 外壳)."""
    lines = md_text.replace('\r\n', '\n').replace('\r', '\n').split('\n')
    out: list[str] = []
    i, n = 0, len(lines)

    while i < n:
        line = lines[i]
        s = line.strip()

        # ---- 围栏代码块 ----
        if s.startswith('```'):
            buf = []
            i += 1
            while i < n and not lines[i].strip().startswith('```'):
                buf.append(lines[i])
                i += 1
            i += 1
            out.append(f'<pre>{_html.escape(chr(10).join(buf))}</pre>')
            continue

        # ---- 标题 ----
        m = HEADING_RE.match(line)
        if m:
            lvl = len(m.group(1))
            txt = m.group(2)
            aid = f"h{len(out)}"
            out.append(f'<h{lvl} id="{aid}">{_inline(txt)}</h{lvl}>')
            i += 1
            continue

        # ---- 分隔线 ----
        if HR_RE.match(s):
            out.append('<hr>')
            i += 1
            continue

        # ---- 表格 ----
        if s.startswith('|') and i + 1 < n and TABLE_SEP_RE.match(lines[i + 1]):
            header = _split_row(line)
            aligns = []
            for c in _split_row(lines[i + 1]):
                if c.startswith(':') and c.endswith(':'):
                    aligns.append('center')
                elif c.endswith(':'):
                    aligns.append('right')
                else:
                    aligns.append('left')
            i += 2
            rows = []
            while i < n and lines[i].strip().startswith('|'):
                rows.append(_split_row(lines[i]))
                i += 1
            tb = ['<table>','<tr>']
            for j, c in enumerate(header):
                a = aligns[j] if j < len(aligns) else 'left'
                tb.append(f'<th align="{a}">{_inline(c)}</th>')
            tb.append('</tr>')
            for row in rows:
                tb.append('<tr>')
                for j, c in enumerate(row):
                    a = aligns[j] if j < len(aligns) else 'left'
                    tb.append(f'<td align="{a}">{_inline(c)}</td>')
                tb.append('</tr>')
            tb.append('</table>')
            out.append(''.join(tb))
            continue

        # ---- 引用 ----
        if s.startswith('>'):
            buf = []
            while i < n and lines[i].strip().startswith('>'):
                buf.append(lines[i].lstrip()[1:].strip())
                i += 1
            inner = ' '.join(x for x in buf if x)
            out.append(f'<blockquote>{_inline(inner)}</blockquote>')
            continue

        # ---- 列表 ----
        if LIST_RE.match(line):
            items = []
            while i < n:
                m2 = LIST_RE.match(lines[i])
                if m2:
                    indent = len(m2.group(1).replace('\t', '  '))
                    items.append([indent, m2.group(2)[-1] == '.', m2.group(3)])
                    i += 1
                elif items and lines[i].strip() and lines[i][:1] in (' ', '\t'):
                    items[-1][2] += ' ' + lines[i].strip()   # 续行
                    i += 1
                else:
                    break
            html_, _ = _render_list(items)
            out.append(html_)
            continue

        # ---- 空行 ----
        if not s:
            i += 1
            continue

        # ---- 段落 ----
        buf = [line.strip()]
        i += 1
        while i < n and lines[i].strip() and not _is_structural(lines[i]):
            buf.append(lines[i].strip())
            i += 1
        out.append(f'<p>{_inline(" ".join(buf))}</p>')

    return '\n'.join(out)


def wrap_document(body: str) -> str:
    return ('<!DOCTYPE html><html><head><meta charset="utf-8"></head>'
            f'<body style="background-color:#0d1526;"><style>{CSS}</style>'
            f'{body}</body></html>')


# ============================================================== 对话框 ======
class HelpDialog(QDialog):
    """文档浏览器: 左侧文件列表, 右侧渲染内容, 支持刷新/外部编辑/缩放."""

    def __init__(self, docs_dir: str, parent=None):
        super().__init__(parent)
        self.docs_dir = docs_dir
        self._files: list[str] = []
        self.setWindowTitle("帮助文档 — 导引算法与仿真参数详解")
        self.resize(1180, 800)
        self.setAttribute(Qt.WA_DeleteOnClose, False)

        # ---------- 左: 文档列表 ----------
        self.listw = QListWidget()
        self.listw.setMinimumWidth(190)
        self.listw.setMaximumWidth(250)
        self.listw.currentRowChanged.connect(self._on_select)

        # ---------- 右: 工具栏 + 内容 ----------
        self.browser = QTextBrowser()
        self.browser.setOpenLinks(False)        # 全部交给 _on_anchor 处理
        self.browser.setOpenExternalLinks(True)
        self.browser.setFont(QFont("Microsoft YaHei UI", 11))
        self.browser.setStyleSheet(
            "QTextBrowser{background:#0d1526; border:1px solid #26364f;"
            "border-radius:4px; padding:6px;}")
        self.browser.anchorClicked.connect(self._on_anchor)

        bar = QHBoxLayout()
        bar.setSpacing(6)
        self.btn_refresh = QPushButton("🔄 刷新")
        self.btn_edit = QPushButton("✏ 编辑本文档")
        self.btn_dir = QPushButton("📂 打开文档目录")
        self.btn_out = QPushButton("🔍 缩小")
        self.btn_in = QPushButton("🔍 放大")
        self.btn_close = QPushButton("关闭")
        for b in (self.btn_refresh, self.btn_edit, self.btn_dir,
                  self.btn_out, self.btn_in):
            bar.addWidget(b)
        bar.addStretch(1)
        bar.addWidget(self.btn_close)

        self.status = QLabel("")
        self.status.setStyleSheet("color:#8fa8cc; font-size:11px;")

        right = QVBoxLayout()
        right.setSpacing(5)
        right.addLayout(bar)
        right.addWidget(self.browser, 1)
        right.addWidget(self.status)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)
        left_col = QVBoxLayout()
        left_col.setSpacing(6)
        left_col.addWidget(self.listw, 1)
        hint = QLabel("提示\n\n· 在 docs/ 目录下新增或\n  修改 .md 文件后，点\n  「🔄 刷新」即可重新加载\n\n· 点正文里的蓝色链接\n  可跳转到其他文档\n\n· 「✏ 编辑本文档」用系统\n  默认程序打开当前文件")
        hint.setWordWrap(True)
        hint.setAlignment(Qt.AlignTop)
        hint.setStyleSheet("color:#7d93b5; font-size:11px;")
        left_col.addWidget(hint)
        lay.addLayout(left_col)
        lay.addLayout(right, 1)

        # ---------- 信号 ----------
        self.btn_refresh.clicked.connect(self.reload)
        self.btn_close.clicked.connect(self.hide)
        self.btn_in.clicked.connect(lambda: self.browser.zoomIn(1))
        self.btn_out.clicked.connect(lambda: self.browser.zoomOut(1))
        self.btn_dir.clicked.connect(self._open_dir)
        self.btn_edit.clicked.connect(self._edit_current)

        self.reload()

    # ---------------------------------------------------------- 加载 ----
    def reload(self, keep_name: str | None = None):
        """重新扫描 docs 目录并渲染(用户改过 md 后调用)."""
        current = keep_name or self._current_file()
        self.listw.blockSignals(True)
        self.listw.clear()
        self._files = []
        if os.path.isdir(self.docs_dir):
            names = [f for f in sorted(os.listdir(self.docs_dir))
                     if f.lower().endswith('.md')]
            names.sort(key=lambda s: (0 if s.lower() in ('readme.md', 'index.md')
                                      else 1, s))
            self._files = names
            for f in names:
                it = QListWidgetItem(os.path.splitext(f)[0])
                it.setData(Qt.UserRole, f)
                self.listw.addItem(it)
        self.listw.blockSignals(False)

        if not self._files:
            self.browser.setHtml(wrap_document(
                f'<p>未在 <code>{_html.escape(self.docs_dir)}</code> 下找到 .md 文档。</p>'))
            self.status.setText("目录为空")
            return

        target = current if current in self._files else self._files[0]
        idx = self._files.index(target)
        self.listw.setCurrentRow(idx)      # 触发 _on_select

    def _current_file(self) -> str | None:
        it = self.listw.currentItem()
        return it.data(Qt.UserRole) if it else None

    def show_file(self, name: str):
        """按文件名(可带 .md)定位并显示."""
        self.reload(keep_name=name if name.endswith('.md') else name + '.md')

    # ---------------------------------------------------------- 事件 ----
    def hideEvent(self, ev):
        """关闭帮助窗口时把焦点/激活交还父窗口, 保证空格/R/S/F1 等快捷键继续有效."""
        super().hideEvent(ev)
        p = self.parent()
        if isinstance(p, QWidget):
            p.activateWindow()
            p.setFocus()

    def _on_select(self, row: int):
        if row < 0 or row >= len(self._files):
            return
        path = os.path.join(self.docs_dir, self._files[row])
        try:
            with open(path, 'r', encoding='utf-8') as fh:
                text = fh.read()
        except Exception as exc:                      # noqa: BLE001
            text = f'<p>读取失败: {_html.escape(str(exc))}</p>'
            body = text
        else:
            body = md_to_html(text)
        self.browser.setSearchPaths([self.docs_dir,
                                     os.path.dirname(path)])
        self.browser.setHtml(wrap_document(body))
        self.browser.verticalScrollBar().setValue(0)
        size = os.path.getsize(path) if os.path.exists(path) else 0
        self.status.setText(f"docs/{self._files[row]}   {size/1024:.1f} KB   "
                            f"修改文件后点「刷新」即可重新加载")

    def _on_anchor(self, url: QUrl):
        """文档内链接: 指向 .md 则切换文档, 锚点则滚动, 其余交给系统."""
        s = unquote(url.toString())
        base, _, frag = s.partition('#')
        if not base and frag:                   # 同文档锚点 #xxx
            self.browser.scrollToAnchor(frag)
            return
        if base.lower().endswith('.md'):
            self.show_file(base)
            return
        QDesktopServices.openUrl(url)

    def _open_dir(self):
        if os.path.isdir(self.docs_dir):
            os.startfile(self.docs_dir)              # Windows

    def _edit_current(self):
        f = self._current_file()
        if f:
            path = os.path.join(self.docs_dir, f)
            if os.path.exists(path):
                os.startfile(path)                   # 用系统默认程序打开
