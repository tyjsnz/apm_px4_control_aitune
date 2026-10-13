# -*- coding: utf-8 -*-
"""右侧控制面板: 导引方法选择、初始位置、导弹/目标参数、仿真控制、实时数据."""

from __future__ import annotations

import math

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QPushButton, QSlider,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from guidance import G0, clamp


def _spin(lo, hi, val, step=1.0, decimals=0, suffix="", prefix=""):
    """创建 QDoubleSpinBox(关闭边输入时的实时回调, 避免打字时频繁重置)."""
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setValue(val)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    s.setSuffix(suffix)
    s.setPrefix(prefix)
    s.setKeyboardTracking(False)
    s.setButtonSymbols(QDoubleSpinBox.PlusMinus)
    return s


class ControlPanel(QWidget):
    """所有控件只负责: 写 cfg / 发信号, 真正的重置交给 MainWindow."""

    paramsChanged = pyqtSignal()                 # 一般参数变化 -> 需 reset
    positionChanged = pyqtSignal(str, float, float)  # (kind, x, y)
    runToggled = pyqtSignal(bool)
    stepRequested = pyqtSignal()
    resetRequested = pyqtSignal()
    defaultsRequested = pyqtSignal()
    helpRequested = pyqtSignal()                 # 打开帮助文档

    ROWS = [
        ("时间 t (s)", "t"),
        ("相对距离 R (m)", "r"),
        ("接近速度 Vc (m/s)", "vc"),
        ("视线角 λ (°)", "los"),
        ("视线角速率 λ̇ (°/s)", "losdot"),
        ("过载 a (m/s²)", "a"),
        ("过载 n (g)", "g"),
        ("弹速 Vm (m/s)", "v"),
        ("制导阶段", "phase"),
        ("截获时刻 (s)", "acq"),
        ("最小距离 (m)", "min"),
        ("结果", "res"),
    ]

    def __init__(self, sim, parent=None):
        super().__init__(parent)
        self.sim = sim
        self._building = True
        self._build_ui()
        self._building = False
        self.sync_from_cfg()

    # ================================================================== UI ==
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---------------- 帮助 ----------------
        self.btn_help = QPushButton("📖  帮助文档 (算法与参数详解)   F1")
        self.btn_help.setFixedHeight(32)
        self.btn_help.setStyleSheet(
            "QPushButton{background:#1b3a63; border:1px solid #3f6ea8;"
            "border-radius:5px; font-weight:bold; color:#cfe4ff; font-size:13px;}"
            "QPushButton:hover{background:#24507f;}"
            "QPushButton:pressed{background:#1b3a63;}")
        self.btn_help.setToolTip(
            "打开 Markdown 帮助文档: 导引算法原理、每个仿真参数的含义与调参建议")
        self.btn_help.clicked.connect(self.helpRequested.emit)
        root.addWidget(self.btn_help)

        # ---------------- 导引方法 ----------------
        g = QGroupBox("导引方法")
        v = QVBoxLayout(g)
        self.mode_box = QComboBox()
        self.mode_box.addItem("双弹对比 (PN vs PP)", "both")
        self.mode_box.addItem("比例导引 (PN)", "pn")
        self.mode_box.addItem("纯跟踪 (PP)", "pp")
        self.mode_box.currentIndexChanged.connect(self._on_mode)
        v.addWidget(self.mode_box)
        tip = QLabel("对比模式下两枚导弹同起点同时发射，\n分别按两种导引律飞行，便于比较。")
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#8fa8cc; font-size:11px;")
        v.addWidget(tip)
        root.addWidget(g)

        # ---------------- 初始位置 ----------------
        g = QGroupBox("初始位置 (可直接在画布上拖拽)")
        grid = QGridLayout(g)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        grid.addWidget(QLabel("导弹 X"), 0, 0); self.sp_mx = _spin(0, 10000, 0, 100, 0, " m")
        grid.addWidget(self.sp_mx, 0, 1)
        grid.addWidget(QLabel("导弹 Y"), 1, 0); self.sp_my = _spin(0, 10000, 0, 100, 0, " m")
        grid.addWidget(self.sp_my, 1, 1)
        grid.addWidget(QLabel("目标 X"), 2, 0); self.sp_tx = _spin(0, 10000, 0, 100, 0, " m")
        grid.addWidget(self.sp_tx, 2, 1)
        grid.addWidget(QLabel("目标 Y"), 3, 0)
        self.sp_ty = _spin(0, 10000, 0, 100, 0, " m")
        grid.addWidget(self.sp_ty, 3, 1)
        self.sp_mx.valueChanged.connect(lambda v: self.positionChanged.emit("m", v, self.sp_my.value()))
        self.sp_my.valueChanged.connect(lambda v: self.positionChanged.emit("m", self.sp_mx.value(), v))
        self.sp_tx.valueChanged.connect(lambda v: self.positionChanged.emit("t", v, self.sp_ty.value()))
        self.sp_ty.valueChanged.connect(lambda v: self.positionChanged.emit("t", self.sp_tx.value(), v))
        root.addWidget(g)

        # ---------------- 导弹参数 ----------------
        g = QGroupBox("导弹参数")
        grid = QGridLayout(g)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        grid.addWidget(QLabel("速度 Vm"), 0, 0)
        self.sp_vms = _spin(100, 1500, 600, 10, 0, " m/s")
        grid.addWidget(self.sp_vms, 0, 1)
        grid.addWidget(QLabel("最大过载 Gmax"), 1, 0)
        self.sp_g = _spin(1, 40, 9, 0.5, 1, " g")
        grid.addWidget(self.sp_g, 1, 1)
        grid.addWidget(QLabel("执行机构 τ"), 2, 0)
        self.sp_tau = _spin(0.02, 2.0, 0.3, 0.05, 2, " s")
        grid.addWidget(self.sp_tau, 2, 1)
        grid.addWidget(QLabel("导航比 N (PN)"), 3, 0)
        self.sp_n = _spin(1, 8, 4, 0.5, 1)
        grid.addWidget(self.sp_n, 3, 1)
        grid.addWidget(QLabel("跟踪增益 K (PP)"), 4, 0)
        self.sp_k = _spin(0.5, 12, 4, 0.5, 1)
        grid.addWidget(self.sp_k, 4, 1)
        self.cb_vc = QCheckBox("PN 系数用接近速度 Vc\n(否则用弹速 Vm)")
        self.cb_vc.setChecked(True)
        grid.addWidget(self.cb_vc, 5, 0, 1, 2)
        for w in (self.sp_vms, self.sp_g, self.sp_tau, self.sp_n, self.sp_k):
            w.valueChanged.connect(self._params)
        self.cb_vc.stateChanged.connect(self._params)
        root.addWidget(g)

        # ---------------- 导引头 / 分段制导 ----------------
        g = QGroupBox("导引头 / 分段制导 (中段→末段)")
        grid = QGridLayout(g)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        self.cb_phased = QCheckBox("启用分段制导\n(未截获=中段飞行, 截获后=末段寻的)")
        grid.addWidget(self.cb_phased, 0, 0, 1, 2)
        grid.addWidget(QLabel("中段方式"), 1, 0)
        self.mode_mid = QComboBox()
        self.mode_mid.addItem("指向预测拦截点 PIP", "pip")
        self.mode_mid.addItem("程序直飞 (惯性)", "straight")
        grid.addWidget(self.mode_mid, 1, 1)
        grid.addWidget(QLabel("截获距离 R_acq"), 2, 0)
        self.sp_acq = _spin(500, 15000, 4000, 100, 0, " m")
        grid.addWidget(self.sp_acq, 2, 1)
        grid.addWidget(QLabel("截获视场 FOV"), 3, 0)
        self.sp_fov = _spin(10, 180, 90, 5, 0, " °")
        grid.addWidget(self.sp_fov, 3, 1)
        tip = QLabel("截获条件: R ≤ R_acq 且目标落在视场内(相对弹轴);\n"
                     "未截获时按中段方式飞行, 截获后才转入 PN/PP。")
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#8fa8cc; font-size:11px;")
        grid.addWidget(tip, 4, 0, 1, 2)
        for w in (self.sp_acq, self.sp_fov):
            w.valueChanged.connect(self._params)
        self.mode_mid.currentIndexChanged.connect(self._params)
        self.cb_phased.stateChanged.connect(self._on_phased)
        self._apply_phased_ui()
        root.addWidget(g)

        # ---------------- 目标参数 ----------------
        g = QGroupBox("目标飞机")
        grid = QGridLayout(g)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        grid.addWidget(QLabel("速度 Vt"), 0, 0)
        self.sp_vt = _spin(0, 800, 250, 10, 0, " m/s")
        grid.addWidget(self.sp_vt, 0, 1)
        grid.addWidget(QLabel("初始航向"), 1, 0)
        self.sp_th = _spin(-360, 360, 200, 5, 0, "°")
        grid.addWidget(self.sp_th, 1, 1)
        grid.addWidget(QLabel("机动模式"), 2, 0)
        self.mode_t = QComboBox()
        self.mode_t.addItem("匀速直线", "straight")
        self.mode_t.addItem("持续左转", "left")
        self.mode_t.addItem("持续右转", "right")
        self.mode_t.addItem("正弦机动", "sine")
        grid.addWidget(self.mode_t, 2, 1)
        grid.addWidget(QLabel("转弯率/幅度"), 3, 0)
        self.sp_turn = _spin(0, 40, 6, 0.5, 1, " °/s")
        grid.addWidget(self.sp_turn, 3, 1)
        grid.addWidget(QLabel("机动周期"), 4, 0)
        self.sp_per = _spin(1, 30, 4, 0.5, 1, " s")
        grid.addWidget(self.sp_per, 4, 1)
        for w in (self.sp_vt, self.sp_th, self.sp_turn, self.sp_per):
            w.valueChanged.connect(self._params)
        self.mode_t.currentIndexChanged.connect(self._params)
        root.addWidget(g)

        # ---------------- 场景 ----------------
        g = QGroupBox("场景")
        grid = QGridLayout(g)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        grid.addWidget(QLabel("战场边长"), 0, 0)
        self.sp_field = _spin(4000, 20000, 10000, 500, 0, " m")
        grid.addWidget(self.sp_field, 0, 1)
        grid.addWidget(QLabel("命中判定半径"), 1, 0)
        self.sp_hit = _spin(1, 200, 25, 5, 0, " m")
        grid.addWidget(self.sp_hit, 1, 1)
        self.sp_field.valueChanged.connect(self._params)
        self.sp_hit.valueChanged.connect(self._params)
        root.addWidget(g)

        # ---------------- 仿真控制 ----------------
        g = QGroupBox("仿真控制")
        v = QVBoxLayout(g)
        row = QHBoxLayout()
        self.btn_run = QPushButton("▶ 开始")
        self.btn_run.setCheckable(True)
        self.btn_run.setStyleSheet(
            "QPushButton{padding:6px; font-weight:bold;}"
            "QPushButton:checked{background:#1f6f3f; color:#eaffef;}")
        self.btn_step = QPushButton("单步")
        self.btn_reset = QPushButton("重置")
        self.btn_default = QPushButton("默认参数")
        for b in (self.btn_run, self.btn_step, self.btn_reset, self.btn_default):
            row.addWidget(b)
        v.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("倍速"))
        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(5, 80)          # 0.5x ~ 8.0x
        self.speed.setValue(10)
        self.speed.setTickPosition(QSlider.TicksBelow)
        self.speed.setTickInterval(15)
        row2.addWidget(self.speed, 1)
        self.lab_speed = QLabel("1.0×")
        self.lab_speed.setMinimumWidth(42)
        row2.addWidget(self.lab_speed)
        v.addLayout(row2)

        self.btn_run.toggled.connect(self._run_toggled)
        self.btn_step.clicked.connect(self.stepRequested.emit)
        self.btn_reset.clicked.connect(self.resetRequested.emit)
        self.btn_default.clicked.connect(self.defaultsRequested.emit)
        self.speed.valueChanged.connect(
            lambda v: self.lab_speed.setText(f"{v / 10:.1f}×"))
        root.addWidget(g)

        # ---------------- 实时数据 ----------------
        g = QGroupBox("实时数据")
        v = QVBoxLayout(g)
        self.table = QTableWidget(len(self.ROWS), 3)
        self.table.setHorizontalHeaderLabels(["参数", "比例导引", "纯跟踪"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.setShowGrid(False)
        self.table.setColumnWidth(0, 118)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(1, QHeaderView.Stretch)
        hh.setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.setFixedHeight(26 + 24 * len(self.ROWS))
        for r, (label, _) in enumerate(self.ROWS):
            it = QTableWidgetItem(label)
            it.setForeground(Qt.gray)
            self.table.setItem(r, 0, it)
            for c in (1, 2):
                self.table.setItem(r, c, QTableWidgetItem("-"))
        v.addWidget(self.table)
        root.addWidget(g)

        hint = QLabel("提示: 修改任意参数或拖拽位置后\n仿真自动重置并暂停。")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#8fa8cc; font-size:11px;")
        root.addWidget(hint)
        root.addStretch(1)

    # ============================================================== 信号 ===
    def _params(self, *args):
        if self._building:
            return
        self.paramsChanged.emit()

    def _on_mode(self, *args):
        if self._building:
            return
        self.paramsChanged.emit()

    def _apply_phased_ui(self):
        """分段制导关闭时禁用其子控件(不发信号)."""
        on = self.cb_phased.isChecked()
        for w in (self.sp_acq, self.sp_fov, self.mode_mid):
            w.setEnabled(on)

    def _on_phased(self, *args):
        self._apply_phased_ui()
        self._params(*args)

    def _run_toggled(self, checked: bool):
        self.btn_run.setText("⏸ 暂停" if checked else "▶ 开始")
        self.runToggled.emit(checked)

    # ============================================================ 同步数据 ==
    @property
    def mode(self) -> str:
        return self.mode_box.currentData()

    def speed_factor(self) -> float:
        return self.speed.value() / 10.0

    def set_running(self, on: bool):
        """由外部(快捷键等)控制运行按钮状态."""
        if self.btn_run.isChecked() != on:
            self.btn_run.setChecked(on)

    def sync_from_cfg(self):
        """把 cfg 写入所有控件(逐个阻断信号, 避免循环触发重置)."""
        c = self.sim.cfg
        boxes = (self.sp_mx, self.sp_my, self.sp_tx, self.sp_ty,
                 self.sp_vms, self.sp_g, self.sp_tau, self.sp_n, self.sp_k,
                 self.sp_vt, self.sp_th, self.sp_turn, self.sp_per,
                 self.sp_field, self.sp_hit, self.mode_box, self.mode_t,
                 self.cb_vc, self.cb_phased, self.mode_mid,
                 self.sp_acq, self.sp_fov)
        for b in boxes:
            b.blockSignals(True)
        self.sp_mx.setValue(c.mx); self.sp_my.setValue(c.my)
        self.sp_tx.setValue(c.tx); self.sp_ty.setValue(c.ty)
        self.sp_vms.setValue(c.m_speed)
        self.sp_g.setValue(c.max_g); self.sp_tau.setValue(c.tau)
        self.sp_n.setValue(c.nav_ratio); self.sp_k.setValue(c.pp_gain)
        self.cb_vc.setChecked(c.use_vc)
        self.sp_vt.setValue(c.t_speed); self.sp_th.setValue(c.t_heading)
        self.mode_t.setCurrentIndex(max(0, self.mode_t.findData(c.t_mode)))
        self.sp_turn.setValue(c.t_turn); self.sp_per.setValue(c.t_period)
        self.sp_field.setValue(c.field_size); self.sp_hit.setValue(c.hit_radius)
        self.mode_box.setCurrentIndex(max(0, self.mode_box.findData(c.mode)))
        self.cb_phased.setChecked(c.phased)
        self.sp_acq.setValue(c.acq_range); self.sp_fov.setValue(c.acq_fov)
        self.mode_mid.setCurrentIndex(max(0, self.mode_mid.findData(c.mid_mode)))
        for b in boxes:
            b.blockSignals(False)
        self._apply_phased_ui()          # 复选框状态变了, 同步子控件可用性

    def write_cfg(self):
        """把控件值写入 cfg(由 MainWindow 在 paramsChanged 时调用)."""
        if self._building:
            return
        c = self.sim.cfg
        c.mode = self.mode
        c.m_speed = self.sp_vms.value()
        c.max_g = self.sp_g.value()
        c.tau = self.sp_tau.value()
        c.nav_ratio = self.sp_n.value()
        c.pp_gain = self.sp_k.value()
        c.use_vc = self.cb_vc.isChecked()
        c.t_speed = self.sp_vt.value()
        c.t_heading = self.sp_th.value()
        c.t_mode = self.mode_t.currentData()
        c.t_turn = self.sp_turn.value()
        c.t_period = self.sp_per.value()
        c.field_size = self.sp_field.value()
        c.hit_radius = self.sp_hit.value()
        c.phased = self.cb_phased.isChecked()
        c.acq_range = self.sp_acq.value()
        c.acq_fov = self.sp_fov.value()
        c.mid_mode = self.mode_mid.currentData()

    def set_positions(self, kind, x, y):
        """外部(画布拖拽)更新位置 spinbox."""
        if kind == "m":
            boxes = (self.sp_mx, self.sp_my)
            vals = (x, y)
        else:
            boxes = (self.sp_tx, self.sp_ty)
            vals = (x, y)
        for b in boxes:
            b.blockSignals(True)
        boxes[0].setValue(vals[0])
        boxes[1].setValue(vals[1])
        for b in boxes:
            b.blockSignals(False)

    # ============================================================ 实时表格 ==
    def update_data(self):
        c = self.sim.cfg
        show_pn = c.mode in ("pn", "both")
        show_pp = c.mode in ("pp", "both")
        self.table.setColumnHidden(1, not show_pn)
        self.table.setColumnHidden(2, not show_pp)

        for col, key in ((1, "pn"), (2, "pp")):
            m = self.sim.missiles[key]
            vals = self._values(m)
            for r, (_, tag) in enumerate(self.ROWS):
                it = self.table.item(r, col)
                txt = vals.get(tag, "-")
                if it.text() != txt:
                    it.setText(txt)
                    active = key in self.sim.active_keys
                    it.setForeground(_color(m.color if active else "#5b6b85"))

    @staticmethod
    def _values(m) -> dict:
        inf = float("inf")
        r = m.range if math.isfinite(m.range) else 0.0
        mn = m.min_range if math.isfinite(m.min_range) else 0.0
        return {
            "t": f"{m.t:.2f}",
            "r": f"{r:,.0f}",
            "vc": f"{m.vc:.0f}",
            "los": f"{m.los_deg:.1f}",
            "losdot": f"{m.los_dot_dps:+.3f}",
            "a": f"{m.a:+.1f}",
            "g": f"{m.overload_g:+.2f}",
            "v": f"{m.speed:.0f}",
            "phase": m.phase,
            "acq": (f"{m.acq_t:.2f}"
                    if (m.params.phased and m.acquired) else "-"),
            "min": f"{mn:,.1f}" if math.isfinite(m.min_range) else "-",
            "res": m.result,
        }


def _color(hexstr: str):
    from PyQt5.QtGui import QColor
    return QColor(hexstr)
