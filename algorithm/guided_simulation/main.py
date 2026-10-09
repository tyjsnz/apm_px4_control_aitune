# -*- coding: utf-8 -*-
"""导弹导引仿真 —— 主窗口与程序入口.

用法
----
    python main.py                        # 启动图形界面
    python main.py --snapshot out.png     # 渲染 seconds 秒后的画面并保存(自动化测试)
    python main.py --quit-after 3         # 3 秒后退出(测试真实屏幕绘制路径)
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtCore import Qt, QTimer                       # noqa: E402
from PyQt5.QtGui import QKeySequence                      # noqa: E402
from PyQt5.QtWidgets import (                             # noqa: E402
    QApplication, QHBoxLayout, QMainWindow, QScrollArea,
    QShortcut, QVBoxLayout, QWidget,
)

from canvas import Canvas, PlotWidget                     # noqa: E402
from docviewer import HelpDialog                          # noqa: E402
from guidance import clamp                                # noqa: E402
from panel import ControlPanel                            # noqa: E402
from simulation import SimConfig, Simulation              # noqa: E402

#: 帮助文档目录(可用任意编辑器增删改其中的 .md 文件)
DOCS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs")

STYLE = """
QMainWindow, QWidget { background:#111826; color:#d7e3f7; font-size:12px; }
QGroupBox { border:1px solid #26364f; border-radius:6px; margin-top:9px;
            padding-top:12px; font-weight:bold; }
QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; color:#7fb3ff; }
QComboBox, QDoubleSpinBox { background:#0d1526; border:1px solid #2a3b58;
            border-radius:4px; padding:2px 5px; }
QComboBox QAbstractItemView { background:#0d1526; border:1px solid #2a3b58;
            selection-background-color:#24406a; }
QPushButton { background:#16233a; border:1px solid #2a3b58; border-radius:4px;
              padding:5px 8px; }
QPushButton:hover { background:#1d2f4d; }
QPushButton:pressed { background:#24406a; }
QTableWidget { background:#0d1526; border:1px solid #26364f; gridline-color:#1a2740; }
QHeaderView::section { background:#16233a; color:#9fc0f0; border:none;
              border-right:1px solid #26364f; padding:3px; }
QScrollArea { border:none; }
QScrollBar:vertical { background:#0d1526; width:10px; }
QScrollBar::handle:vertical { background:#2a3b58; border-radius:4px; min-height:24px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
QSlider::groove:horizontal { height:4px; background:#22344f; border-radius:2px; }
QSlider::handle:horizontal { width:14px; margin:-5px 0; background:#4a86d8;
              border-radius:7px; }
QLabel { background:transparent; }
QStatusBar { background:#0d1526; color:#9fb6d9; }
"""


class MainWindow(QMainWindow):
    """主窗口: 左侧画布+曲线, 右侧参数面板."""

    TICK_MS = 16

    def __init__(self):
        super().__init__()
        self.setWindowTitle("导弹导引仿真 —— 比例导引(PN) vs 纯跟踪(PP)")
        self.resize(1400, 880)

        self.sim = Simulation()
        self._running = False
        self.acc = 0.0
        self._dirty = True
        self._help: HelpDialog | None = None

        # ---------------- 左: 画布 + 曲线 ----------------
        self.canvas = Canvas(self.sim)
        self.plot = PlotWidget(self.sim)
        left = QVBoxLayout()
        left.setSpacing(6)
        left.addWidget(self.canvas, 3)
        left.addWidget(self.plot, 1)

        # ---------------- 右: 控制面板 ----------------
        self.panel = ControlPanel(self.sim)
        scroll = QScrollArea()
        scroll.setWidget(self.panel)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(320)
        scroll.setMaximumWidth(400)

        central = QWidget()
        lay = QHBoxLayout(central)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)
        lay.addLayout(left, 1)
        lay.addWidget(scroll)
        self.setCentralWidget(central)

        # ---------------- 信号 ----------------
        self.canvas.posEdited.connect(self._on_position)
        self.panel.positionChanged.connect(self._on_position)
        self.panel.paramsChanged.connect(self._on_params)
        self.panel.runToggled.connect(self._on_run)
        self.panel.stepRequested.connect(self._on_step)
        self.panel.resetRequested.connect(self._on_reset)
        self.panel.defaultsRequested.connect(self._on_defaults)
        self.panel.helpRequested.connect(self.open_help)

        # ---------------- 快捷键 ----------------
        QShortcut(QKeySequence(Qt.Key_Space), self, self._toggle_run)
        QShortcut(QKeySequence("R"), self, self._on_reset)
        QShortcut(QKeySequence("S"), self, self._on_step)
        QShortcut(QKeySequence(Qt.Key_F1), self, self.open_help)

        # ---------------- 定时器 ----------------
        self.timer = QTimer(self)
        self.timer.setInterval(self.TICK_MS)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

        self._refresh()

    # ============================================================== 逻辑 ===
    def _on_params(self):
        self.panel.write_cfg()
        self._reset()

    def _on_position(self, kind: str, x: float, y: float):
        size = self.sim.cfg.field_size
        x, y = clamp(x, 0.0, size), clamp(y, 0.0, size)
        if kind == "m":
            self.sim.cfg.mx, self.sim.cfg.my = x, y
        else:
            self.sim.cfg.tx, self.sim.cfg.ty = x, y
        self._reset()

    def _on_run(self, on: bool):
        if on and self.sim.finished:
            # 已结束后再次点"开始" = 重新开始
            self._reset(keep_run_state=True)
        self._running = on and not self.sim.finished
        self.panel.btn_run.clearFocus()

    def _toggle_run(self):
        self.panel.set_running(not self.panel.btn_run.isChecked())

    def _on_step(self):
        self.panel.set_running(False)
        self._running = False
        if not self.sim.finished:
            self.sim.step()
        self._dirty = True
        self._refresh()

    def _on_reset(self):
        self.panel.set_running(False)
        self._reset()

    def _on_defaults(self):
        self.sim.cfg = SimConfig()
        self.panel.set_running(False)
        self._reset()

    def open_help(self, topic: str | None = None):
        """打开/前置帮助文档窗口(Markdown, 读取 docs/ 目录)."""
        if self._help is None:
            self._help = HelpDialog(DOCS_DIR, self)
        self._help.reload()                 # 重新扫描, 便于用户改过文档后立即生效
        if topic:
            self._help.show_file(topic)
        self._help.show()
        self._help.raise_()
        self._help.activateWindow()

    def _reset(self, keep_run_state: bool = False):
        c = self.sim.cfg
        size = c.field_size
        c.mx = clamp(c.mx, 0.0, size); c.my = clamp(c.my, 0.0, size)
        c.tx = clamp(c.tx, 0.0, size); c.ty = clamp(c.ty, 0.0, size)
        if not keep_run_state:
            self.panel.set_running(False)
            self._running = False
        self.acc = 0.0
        self.sim.reset()
        self.panel.sync_from_cfg()
        self._dirty = True
        self._refresh()

    # ============================================================== 定时 ===
    def _tick(self):
        if self._running and not self.sim.finished:
            self.acc += (self.TICK_MS / 1000.0) * self.panel.speed_factor()
            n = 0
            while self.acc >= Simulation.DT and n < 500:
                self.sim.step()
                self.acc -= Simulation.DT
                n += 1
                if self.sim.finished:
                    self.acc = 0.0
                    self.panel.set_running(False)
                    self._running = False
                    break
            self._refresh()
        elif self._dirty:
            self._dirty = False
            self._refresh()

    def _refresh(self):
        self.canvas.update()
        self.plot.update()
        self.panel.update_data()
        state = "结束" if self.sim.finished else (
            "运行中" if (self._running or self.sim.running) else "就绪")
        self.statusBar().showMessage(
            f"t={self.sim.t:.2f}s  [{state}]   " + self.sim.summary())


# ================================================================ 入口 =====
def main():
    ap = argparse.ArgumentParser(description="导弹导引仿真")
    ap.add_argument("--snapshot", help="运行若干秒后截图保存到该文件并退出")
    ap.add_argument("--seconds", type=float, default=8.0, help="截图前推进的仿真秒数")
    ap.add_argument("--mode", choices=["pn", "pp", "both"], default=None)
    ap.add_argument("--quit-after", type=float, default=0.0,
                    help="N 秒后直接退出(用于测试真实屏幕绘制路径)")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)

    win = MainWindow()
    if args.mode:
        win.sim.cfg.mode = args.mode
        win.panel.sync_from_cfg()
        win.sim.reset()
    win.show()

    if args.snapshot:
        def _snap():
            win.sim.run(args.seconds)
            win._refresh()
            win.grab().save(args.snapshot)
            app.quit()
        QTimer.singleShot(700, _snap)

    if args.quit_after > 0:
        QTimer.singleShot(int(args.quit_after * 1000), app.quit)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
