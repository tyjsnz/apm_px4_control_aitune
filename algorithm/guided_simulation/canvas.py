# -*- coding: utf-8 -*-
"""战场画布与曲线图控件(纯 QPainter 绘制, 无第三方绘图库依赖)."""

from __future__ import annotations

import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget

from guidance import G0, clamp

# ---------------------------------------------------------------- 配色 -----
BG          = "#0a1020"
GRID        = "#152238"
GRID_MAJOR  = "#1e3050"
BORDER      = "#2b4468"
TXT         = "#8fa8cc"
TXT_HI      = "#e8f1ff"
C_TARGET    = "#3fd2ff"
C_PN        = "#ff9f43"
C_PP        = "#ff5ad1"
C_MISSILE   = C_PN


# =============================================================== 工具函数 ===
def _qcolor(hexstr: str, alpha: int = 255) -> QColor:
    c = QColor(hexstr)
    c.setAlpha(alpha)
    return c


def _nice_step(span: float, target_ticks: int = 6) -> float:
    """取一个"好看"的刻度步长."""
    if span <= 0:
        return 1.0
    raw = span / max(target_ticks, 1)
    mag = 10.0 ** math.floor(math.log10(raw))
    for m in (1.0, 2.0, 2.5, 5.0, 10.0):
        if raw <= m * mag:
            return m * mag
    return 10.0 * mag


# =============================================================== 画布 ======
class Canvas(QWidget):
    """战场画布: 绘制网格、弹道、目标/导弹图标、视线, 支持鼠标拖拽."""

    #: (kind, x, y) kind: "m"=导弹  "t"=目标, 单位 m
    posEdited = pyqtSignal(str, float, float)

    PICK_R = 30          # 鼠标拾取半径 (像素)

    def __init__(self, sim, parent=None):
        super().__init__(parent)
        self.sim = sim
        self.setMinimumSize(660, 460)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)
        self._drag = None      # 当前拖拽对象 "m"/"t"
        self._hover = None

    # ---------------------------------------------------------- 坐标变换 --
    def _view(self):
        size = self.sim.cfg.field_size
        w, h = self.width(), self.height()
        margin = 16.0
        s = min((w - 2 * margin) / size, (h - 2 * margin) / size)
        ox = (w - size * s) / 2.0
        oy = (h - size * s) / 2.0
        return s, ox, oy, size

    def w2s(self, x, y) -> QPointF:
        s, ox, oy, size = self._view()
        return QPointF(ox + x * s, oy + (size - y) * s)

    def s2w(self, p: QPointF):
        s, ox, oy, size = self._view()
        if s <= 0:
            return 0.0, 0.0
        return (p.x() - ox) / s, size - (p.y() - oy) / s

    # ------------------------------------------------------------ 鼠标事件 -
    def _pick(self, pos: QPointF):
        c = self.sim.cfg
        cand = [("m", c.mx, c.my)]
        if c.mode in ("pn", "both"):
            m = self.sim.missiles["pn"]
            cand.append(("m", m.x, m.y))
        cand.append(("t", c.tx, c.ty))
        best, best_d = None, self.PICK_R
        for kind, x, y in cand:
            s = self.w2s(x, y)
            d = math.hypot(s.x() - pos.x(), s.y() - pos.y())
            if d <= best_d:
                best, best_d = kind, d
        return best

    def mousePressEvent(self, ev):
        if ev.button() != Qt.LeftButton:
            return
        self._drag = self._pick(ev.pos())
        if self._drag:
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, ev):
        pos = ev.pos()
        if self._drag:
            x, y = self.s2w(pos)
            size = self.sim.cfg.field_size
            self.posEdited.emit(self._drag, clamp(x, 0.0, size), clamp(y, 0.0, size))
        else:
            h = self._pick(pos)
            if h != self._hover:
                self._hover = h
                self.setCursor(Qt.OpenHandCursor if h else Qt.CrossCursor)

    def mouseReleaseEvent(self, ev):
        self._drag = None
        self.setCursor(Qt.OpenHandCursor if self._hover else Qt.CrossCursor)

    # ------------------------------------------------------------ 绘制入口 -
    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(BG))

        self._draw_field(p)
        self._draw_trails(p)
        self._draw_start_marks(p)
        self._draw_los(p)
        self._draw_target(p)
        for key in self.sim.active_keys:
            self._draw_missile(p, self.sim.missiles[key])
        self._draw_legend(p)
        self._draw_overlay(p)
        p.end()

    # -------------------------------------------------------------- 网格 ---
    def _draw_field(self, p: QPainter):
        s, ox, oy, size = self._view()
        step = 1000.0
        while step * s < 55.0:
            step *= 2.0

        # 网格线
        pen = QPen(QColor(GRID))
        pen.setWidthF(0.8)
        p.setPen(pen)
        n = int(size / step)
        for i in range(n + 1):
            v = i * step
            a = self.w2s(v, 0)
            b = self.w2s(v, size)
            p.drawLine(QPointF(a.x(), a.y()), QPointF(b.x(), b.y()))
            a = self.w2s(0, v)
            b = self.w2s(size, v)
            p.drawLine(QPointF(a.x(), a.y()), QPointF(b.x(), b.y()))

        # 主刻度 + 坐标标注(每 2km)
        pen = QPen(QColor(GRID_MAJOR))
        pen.setWidthF(1.2)
        p.setPen(pen)
        p.setFont(_font(10))
        lab = _qcolor(TXT, 200)
        maj = step * 2
        i = 0
        while i * maj <= size + 1e-6:
            v = i * maj
            a = self.w2s(v, 0); b = self.w2s(v, size)
            p.setPen(pen)
            p.drawLine(QPointF(a.x(), a.y()), QPointF(b.x(), b.y()))
            a = self.w2s(0, v); b = self.w2s(size, v)
            p.drawLine(QPointF(a.x(), a.y()), QPointF(b.x(), b.y()))
            # 底部 X 标注
            sp = self.w2s(v, 0)
            p.setPen(lab)
            p.drawText(QPointF(sp.x() + 3, min(self.height() - 4, sp.y() + 14)),
                       f"{v / 1000:.0f}")
            # 左侧 Y 标注
            sp = self.w2s(0, v)
            p.drawText(QPointF(max(2.0, sp.x() - 30), sp.y() - 3),
                       f"{v / 1000:.0f}")
            i += 1
            if maj * s < 1:
                break

        # 轴单位
        p.setPen(_qcolor(TXT, 160))
        p.setFont(_font(10, bold=True))
        p.drawText(QPointF(self.width() - 46, self.height() - 6), "x / km")
        p.drawText(QPointF(6, 14), "y / km")

        # 战场边界
        pen = QPen(QColor(BORDER))
        pen.setWidthF(1.6)
        p.setPen(pen)
        tl = self.w2s(0, size)
        p.drawRect(QRectF(tl.x(), tl.y(), size * s, size * s))

    # -------------------------------------------------------------- 弹道 ---
    def _path(self, points) -> QPainterPath:
        path = QPainterPath()
        first = True
        for x, y in points:
            pt = self.w2s(x, y)
            if first:
                path.moveTo(pt)
                first = False
            else:
                path.lineTo(pt)
        return path

    def _draw_trails(self, p: QPainter):
        # 目标航线
        tpath = self._path(self.sim.target.trail)
        pen = QPen(_qcolor(C_TARGET, 70))
        pen.setWidthF(4.0)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        p.setPen(pen)
        p.drawPath(tpath)
        pen = QPen(_qcolor(C_TARGET, 210))
        pen.setWidthF(1.4)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.drawPath(tpath)

        # 导弹弹道(发光效果: 粗淡 + 细亮)
        for key in self.sim.active_keys:
            m = self.sim.missiles[key]
            path = self._path(m.trail)
            pen = QPen(_qcolor(m.color, 60))
            pen.setWidthF(6.0)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen)
            p.drawPath(path)
            pen = QPen(_qcolor(m.color, 235))
            pen.setWidthF(2.0)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen)
            p.drawPath(path)

    def _draw_start_marks(self, p: QPainter):
        """初始位置标记 ×."""
        for kind, x, y, color in (
            ("m", self.sim.cfg.mx, self.sim.cfg.my, C_PN),
            ("t", self.sim.cfg.tx, self.sim.cfg.ty, C_TARGET),
        ):
            c = self.w2s(x, y)
            pen = QPen(_qcolor(color, 150))
            pen.setWidthF(1.2)
            p.setPen(pen)
            p.drawLine(QPointF(c.x() - 5, c.y() - 5), QPointF(c.x() + 5, c.y() + 5))
            p.drawLine(QPointF(c.x() - 5, c.y() + 5), QPointF(c.x() + 5, c.y() - 5))

    # -------------------------------------------------------------- 视线 ---
    def _draw_los(self, p: QPainter):
        tgt = self.sim.target
        for key in self.sim.active_keys:
            m = self.sim.missiles[key]
            a = self.w2s(m.x, m.y)
            b = self.w2s(tgt.x, tgt.y)
            pen = QPen(_qcolor(m.color, 130))
            pen.setWidthF(1.1)
            pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.drawLine(a, b)
            # 视线太短(接近命中)时不画距离标注, 避免与图标重叠
            if self.sim.t > 0 and math.hypot(b.x() - a.x(), b.y() - a.y()) >= 90:
                # 标注放在视线中点、并沿视线法向偏移, 避开弹道与图标
                mx, my = (a.x() + b.x()) / 2, (a.y() + b.y()) / 2
                dx, dy = b.x() - a.x(), b.y() - a.y()
                L = math.hypot(dx, dy) or 1.0
                nx, ny = -dy / L * 15.0, dx / L * 15.0
                self._text(p, QPointF(mx + nx, my + ny),
                           f"R={m.range / 1000:.2f} km",
                           _qcolor(m.color, 230), 11)

    # -------------------------------------------------------------- 目标 ----
    def _draw_target(self, p: QPainter):
        tgt = self.sim.target
        c = self.w2s(tgt.x, tgt.y)

        # 命中半径
        r = self.sim.cfg.hit_radius * self._view()[0]
        if r >= 2.0:
            pen = QPen(_qcolor(C_TARGET, 90))
            pen.setWidthF(1.0)
            pen.setStyle(Qt.DotLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(c, r, r)

        self._draw_plane(p, c, tgt.heading)

        label = f"目标 TGT  {tgt.speed:.0f} m/s"
        if tgt.mode != "straight":
            w = math.degrees(tgt.omega)
            label += f"  ω={w:+.1f}°/s"
        self._text(p, c + QPointF(28, -30), label, _qcolor(C_TARGET, 245), 12, bold=True)
        # 目标坐标仅在距离较远时标注, 避免与导弹图标/标签堆叠
        if self.sim.active[0].range > 1200.0:
            self._text(p, c + QPointF(28, -15),
                       f"({tgt.x/1000:.2f}, {tgt.y/1000:.2f}) km",
                       _qcolor(C_TARGET, 160), 10)

    def _draw_plane(self, p: QPainter, c: QPointF, heading: float):
        """飞机俯视图标(机头指向 +x, 绘制时旋转)."""
        p.save()
        p.translate(c)
        p.rotate(-math.degrees(heading))

        k = 0.82
        wing = _poly([(6, 2.6), (-8, 20), (-15, 20), (-12, 2.6),
                      (-12, -2.6), (-15, -20), (-8, -20), (6, -2.6)], k)
        tail = _poly([(-14, 1.8), (-21, 9), (-25, 9), (-22, 1.8),
                      (-22, -1.8), (-25, -9), (-21, -9), (-14, -1.8)], k)
        body = _poly([(26, 0), (10, -3.6), (-14, -3.6), (-20, -2.6),
                      (-20, 2.6), (-14, 3.6), (10, 3.6)], k)

        p.setPen(Qt.NoPen)
        p.setBrush(_qcolor(C_TARGET, 70))
        p.drawPolygon(wing)
        p.drawPolygon(tail)
        p.setBrush(_qcolor(C_TARGET, 210))
        p.drawPolygon(body)

        pen = QPen(_qcolor(C_TARGET, 255))
        pen.setWidthF(1.4)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(body)
        p.drawPolygon(wing)
        p.restore()

    # -------------------------------------------------------------- 导弹 ----
    def _draw_missile(self, p: QPainter, m):
        c = self.w2s(m.x, m.y)

        # 速度矢量箭头
        L = 46.0
        tip = QPointF(c.x() + L * math.cos(-m.heading),
                      c.y() + L * math.sin(-m.heading))
        pen = QPen(_qcolor(m.color, 200))
        pen.setWidthF(1.6)
        p.setPen(pen)
        p.drawLine(c, tip)
        ang = math.degrees(m.heading)
        p.save(); p.translate(tip); p.rotate(-ang)
        p.setBrush(_qcolor(m.color, 220)); p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([QPointF(-8, -4), QPointF(0, 0), QPointF(-8, 4)]))
        p.restore()

        self._draw_dart(p, c, m.heading, m.color, self.sim.running)

        # 对比模式下: 比例导引标签在上方, 纯跟踪标签在下方, 避免重叠
        both = self.sim.cfg.mode == "both"
        if both and m.kind == "pp":
            y0 = 18.0
        elif both:
            y0 = -26.0
        else:
            y0 = -22.0
        # 接近目标时改到图标左侧, 避免与目标标签叠在一起
        x0 = -112.0 if m.range < 900.0 else 24.0
        self._text(p, c + QPointF(x0, y0), f"{m.name} {m.speed:.0f} m/s",
                   _qcolor(m.color, 250), 12, bold=True)
        self._text(p, c + QPointF(x0, y0 + 15),
                   f"n={m.overload_g:+.1f}g  λ̇={m.los_dot_dps:+.2f}°/s",
                   _qcolor(m.color, 170), 10)

    def _draw_dart(self, p: QPainter, c: QPointF, heading: float, color, burning):
        p.save()
        p.translate(c)
        p.rotate(-math.degrees(heading))

        flame = _poly([(-14, -1.8), (-14, 1.8), (-26, 0)], 1.0)
        p.setPen(Qt.NoPen)
        p.setBrush(_qcolor("#ffd166", 230 if burning else 90))
        p.drawPolygon(flame)

        body = _poly([(20, 0), (7, -3.0), (-11, -3.0), (-15, -1.6),
                      (-15, 1.6), (-11, 3.0), (7, 3.0)], 1.0)
        fin1 = _poly([(-7, 2.6), (-15, 9), (-18, 9), (-12, 2.6)], 1.0)
        fin2 = _poly([(-7, -2.6), (-15, -9), (-18, -9), (-12, -2.6)], 1.0)

        p.setBrush(_qcolor(color, 80))
        p.drawPolygon(fin1)
        p.drawPolygon(fin2)
        p.setBrush(_qcolor("#0a1020", 240))
        p.drawPolygon(body)
        pen = QPen(_qcolor(color, 255))
        pen.setWidthF(1.5)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(body)
        p.restore()

    # -------------------------------------------------------------- 图例 ---
    def _draw_legend(self, p: QPainter):
        if self.sim.finished:      # 结束后用结果横幅代替图例, 避免遮挡
            return
        items = [("目标航线", C_TARGET, True)]
        if "pn" in self.sim.active_keys:
            items.append(("比例导引(PN)弹道", C_PN, False))
        if "pp" in self.sim.active_keys:
            items.append(("纯跟踪(PP)弹道", C_PP, False))

        x, y = 14.0, 26.0
        p.setFont(_font(11))
        for text, color, dashed in items:
            pen = QPen(_qcolor(color, 235))
            pen.setWidthF(2.4)
            if dashed:
                pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.drawLine(QPointF(x, y - 4), QPointF(x + 22, y - 4))
            self._text(p, QPointF(x + 28, y), text, _qcolor(TXT_HI, 235), 11)
            y += 17

    # -------------------------------------------------------------- 提示 ----
    def _draw_overlay(self, p: QPainter):
        # 顶部状态
        sim = self.sim
        state = "仿真结束" if sim.finished else ("运行中" if sim.running else "就绪")
        self._text(p, QPointF(14, self.height() - 34),
                   f"t = {sim.t:6.2f} s    {state}", _qcolor(TXT, 240), 12, bold=True)

        if not sim.running and not sim.finished:
            self._text(p, QPointF(14, self.height() - 16),
                       "拖拽导弹/目标调整初始位置   空格: 开始/暂停   R: 重置   S: 单步   F1: 帮助文档",
                       _qcolor(TXT, 170), 11)

        if sim.finished:
            self._banner(p, sim.summary())

    def _banner(self, p: QPainter, text: str):
        rect = QRectF(0, 10, self.width(), 40)
        p.setPen(Qt.NoPen)
        p.setBrush(_qcolor("#050a16", 215))
        p.drawRoundedRect(rect, 8, 8)
        pen = QPen(_qcolor("#3f5f95"))
        pen.setWidthF(1.2)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(rect, 8, 8)
        p.setFont(_font(13, bold=True))
        br = p.fontMetrics().horizontalAdvance(text)
        p.setPen(QColor(TXT_HI))
        p.drawText(QPointF((self.width() - br) / 2, rect.center().y() + 5), text)

    # -------------------------------------------------------------- 文字 ----
    def _text(self, p: QPainter, pos: QPointF, s, color, size=11, bold=False):
        p.setFont(_font(size, bold))
        p.setPen(_qcolor("#000000", 165))
        p.drawText(QPointF(pos.x() + 1, pos.y() + 1), s)
        p.setPen(color)
        p.drawText(pos, s)


def _font(size: int, bold: bool = False) -> QFont:
    f = QFont("Segoe UI", size)
    f.setPixelSize(size)
    f.setBold(bold)
    return f


def _poly(pts, k=1.0) -> QPolygonF:
    return QPolygonF([QPointF(x * k, -y * k) for x, y in pts])
    # 注意: 屏幕 y 向下, 因此取 -y 使局部坐标与世界坐标(数学方向)一致


# =============================================================== 曲线图 ====
class PlotWidget(QWidget):
    """双面板曲线: 上=相对距离 R(t), 下=导弹过载 a(t)."""

    def __init__(self, sim, parent=None):
        super().__init__(parent)
        self.sim = sim
        self.setMinimumHeight(190)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(BG))

        w, h = self.width(), self.height()
        ml, mr, mt, mb, gap = 58.0, 14.0, 14.0, 24.0, 14.0
        pw = w - ml - mr
        ph = (h - mt - mb - gap) / 2.0
        if pw < 60 or ph < 40:
            p.end()
            return
        top = QRectF(ml, mt, pw, ph)
        bot = QRectF(ml, mt + ph + gap, pw, ph)

        acts = self.sim.active

        # ---------- 上: 相对距离 ----------
        rmax = 0.0
        for m in acts:
            if m.hist_r:
                rmax = max(rmax, max(m.hist_r))
        rmax = rmax if rmax > 0 else self.sim.cfg.field_size
        curves_r = [(m.color, m.name, m.hist_t, m.hist_r, m.range) for m in acts]
        self._panel(p, top, "相对距离 R(t)", "m", curves_r, 0.0, rmax,
                    self._fmt_m, show_x=False)

        # ---------- 下: 过载 ----------
        amax = self.sim.cfg.max_g * G0 * 1.15
        for m in acts:
            if m.hist_a:
                amax = max(amax, max(abs(v) for v in m.hist_a) * 1.05)
        curves_a = [(m.color, m.name, m.hist_t, m.hist_a, m.a) for m in acts]
        self._panel(p, bot, "导弹法向过载 a(t)", "m/s²", curves_a,
                    -amax, amax, lambda v: f"{v:.0f}",
                    hlines=[(-self.sim.cfg.max_g * G0, "−Gmax"),
                            (0.0, "0"),
                            (self.sim.cfg.max_g * G0, "+Gmax")],
                    show_x=True)
        p.end()

    # ------------------------------------------------------------------
    @staticmethod
    def _fmt_m(v):
        return f"{v / 1000:.1f}k" if abs(v) >= 1000 else f"{v:.0f}"

    def _panel(self, p, rect, title, unit, curves, ymin, ymax, fmt,
               hlines=None, show_x=False):
        # 面板底
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#0d1526"))
        p.drawRoundedRect(rect, 6, 6)
        pen = QPen(QColor("#1d2c47")); pen.setWidthF(1.0)
        p.setPen(pen); p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(rect, 6, 6)

        tmax = 5.0
        for _, _, xs, _, _ in curves:
            if xs:
                tmax = max(tmax, max(xs))
        if ymax <= ymin:
            ymax = ymin + 1.0

        def px(t):   return rect.left() + (t / tmax) * rect.width()
        def py(v):   return rect.bottom() - ((v - ymin) / (ymax - ymin)) * rect.height()

        # 横向网格
        p.setFont(_font(10))
        for i in range(5):
            v = ymin + (ymax - ymin) * i / 4.0
            y = py(v)
            gpen = QPen(QColor(GRID)); gpen.setWidthF(0.8)
            if abs(v) < 1e-9 and ymin < 0 < ymax:
                gpen = QPen(QColor(GRID_MAJOR)); gpen.setWidthF(1.2)
            p.setPen(gpen)
            p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            p.setPen(_qcolor(TXT, 210))
            p.drawText(QRectF(rect.left() - 54, y - 8, 48, 16),
                       int(Qt.AlignRight | Qt.AlignVCenter), fmt(v))

        # 指定水平参考线(如 ±Gmax)
        for v, lab in (hlines or []):
            if ymin <= v <= ymax:
                hp = QPen(_qcolor("#ff5a5a", 90)); hp.setWidthF(1.0)
                hp.setStyle(Qt.DashLine)
                p.setPen(hp)
                y = py(v)
                p.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
                p.setPen(_qcolor("#ff8080", 200))
                p.drawText(QPointF(rect.right() - 42, y - 3), lab)

        # 时间刻度
        tstep = _nice_step(tmax, 6)
        t = 0.0
        while t <= tmax + 1e-6:
            x = px(t)
            gpen = QPen(QColor(GRID)); gpen.setWidthF(0.8)
            p.setPen(gpen)
            p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            if show_x:
                p.setPen(_qcolor(TXT, 210))
                p.drawText(QPointF(x - 8, rect.bottom() + 15), f"{t:.0f}")
            t += tstep
        if show_x:
            p.setPen(_qcolor(TXT, 170))
            p.drawText(QPointF(rect.right() - 44, rect.bottom() + 15), "t / s")

        # 曲线
        for color, label, xs, ys, cur in curves:
            if len(xs) >= 2:
                path = QPainterPath()
                path.moveTo(px(xs[0]), py(ys[0]))
                for i in range(1, len(xs)):
                    path.lineTo(px(xs[i]), py(ys[i]))
                gp = QPen(_qcolor(color, 55)); gp.setWidthF(5.0)
                gp.setCapStyle(Qt.RoundCap); gp.setJoinStyle(Qt.RoundJoin)
                p.setPen(gp); p.setBrush(Qt.NoBrush); p.drawPath(path)
                pen = QPen(_qcolor(color, 240)); pen.setWidthF(1.8)
                pen.setCapStyle(Qt.RoundCap); pen.setJoinStyle(Qt.RoundJoin)
                p.setPen(pen); p.drawPath(path)
                # 当前值端点
                p.setBrush(_qcolor(color, 250)); p.setPen(Qt.NoPen)
                p.drawEllipse(QPointF(px(xs[-1]), py(ys[-1])), 3.0, 3.0)

        # 标题(含单位) / 图例
        p.setFont(_font(11, bold=True))
        p.setPen(QColor(TXT_HI))
        p.drawText(QPointF(rect.left() + 8, rect.top() + 15),
                   f"{title}   [{unit}]")

        if len(curves) >= 1:
            lx = rect.right() - (76 * len(curves) + 4)
            p.setFont(_font(10))
            for color, label, xs, ys, cur in curves:
                p.setPen(_qcolor(color, 245))
                p.drawText(QPointF(lx, rect.top() + 15),
                           f"{label} {fmt(cur)}")
                lx += 74
