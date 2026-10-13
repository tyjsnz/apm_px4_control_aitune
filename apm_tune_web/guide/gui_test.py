# -*- coding: utf-8 -*-
"""GUI 集成测试: 验证信号连线、参数联动、拖拽、运行控制与结果输出.

用法: python gui_test.py   (通过打印 PASS/FAIL, 失败返回非0退出码)
"""

from __future__ import annotations

import os
import re
import sys

from PyQt5.QtCore import QEvent, QPoint, Qt
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from main import MainWindow, STYLE
from simulation import SimConfig, Simulation

DT = Simulation.DT

FAILS = []


def check(cond, msg):
    tag = "PASS" if cond else "FAIL"
    print(f"[{tag}] {msg}", flush=True)
    if not cond:
        FAILS.append(msg)


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)

    win = MainWindow()
    win.show()
    QTest.qWait(300)                      # 让首帧绘制完成
    sim, panel, cfg = win.sim, win.panel, win.sim.cfg

    # 1) 初始状态
    check(sim.t == 0.0 and not sim.finished, "初始状态: t=0 且未结束")
    check(abs(cfg.t_heading - 200.0) < 1e-9,
          f"目标初始航向 200° 未被控件限幅 (实际 {cfg.t_heading})")
    check(panel.sp_th.value() == 200.0,
          f"航向控件显示 200° (实际 {panel.sp_th.value()})")

    # 2) 开始运行
    panel.btn_run.setChecked(True)
    QTest.qWait(600)
    check(sim.t > 0.2, f"点击开始后仿真推进 (t={sim.t:.2f}s)")
    check(panel.btn_run.isChecked(), "运行按钮保持选中状态")

    # 3) 暂停
    panel.btn_run.setChecked(False)
    t_pause = sim.t
    QTest.qWait(300)
    check(abs(sim.t - t_pause) < 1e-9, "暂停后仿真停止推进")

    # 4) 修改参数 -> 自动重置
    panel.sp_n.setValue(3.5)
    check(abs(cfg.nav_ratio - 3.5) < 1e-9, f"导航比写入 cfg ({cfg.nav_ratio})")
    check(sim.t == 0.0, "参数修改后自动重置")
    check(abs(sim.missiles["pn"].params.nav_ratio - 3.5) < 1e-9,
          "导弹对象已用新参数重建")
    check(not panel.btn_run.isChecked(), "参数修改后运行按钮复位")

    # 5) 拖拽位置 -> cfg / 控件 / 弹道起点同步
    win.canvas.posEdited.emit("m", 1234.0, 5678.0)
    check(abs(cfg.mx - 1234.0) < 1e-6 and abs(cfg.my - 5678.0) < 1e-6,
          "画布拖拽写入 cfg")
    check(abs(panel.sp_mx.value() - 1234.0) < 1e-6,
          f"拖拽后位置控件同步 ({panel.sp_mx.value()})")
    m0 = sim.missiles["pn"]
    check(abs(m0.x0 - 1234.0) < 1e-6 and abs(m0.y0 - 5678.0) < 1e-6,
          "导弹起始点随拖拽更新")

    # 5b) 真实鼠标事件（完整走 mousePress/Move/Release 处理链）
    QTest.qWait(100)
    p0 = win.canvas.w2s(win.sim.cfg.mx, win.sim.cfg.my).toPoint()
    p1 = p0 + QPoint(70, -45)
    g0, g1 = win.canvas.mapToGlobal(p0), win.canvas.mapToGlobal(p1)
    for etype, pos, buttons in (
            (QEvent.MouseButtonPress, p0, Qt.LeftButton),
            (QEvent.MouseMove, p1, Qt.LeftButton),
            (QEvent.MouseButtonRelease, p1, Qt.NoButton)):
        ev = QMouseEvent(etype, pos, g0 if pos == p0 else g1,
                         Qt.LeftButton, buttons, Qt.NoModifier)
        QApplication.sendEvent(win.canvas, ev)
    wx, wy = win.canvas.s2w(p1)
    size = win.sim.cfg.field_size
    ex = min(max(wx, 0.0), size)
    ey = min(max(wy, 0.0), size)
    check(abs(cfg.mx - ex) < 1e-6 and abs(cfg.my - ey) < 1e-6,
          f"鼠标拖拽改变导弹位置 ({cfg.mx:.0f}, {cfg.my:.0f}) -> 期望 ({ex:.0f}, {ey:.0f})")
    # 位置控件为 0 位小数(取整到米), 因此容差取 1 m
    check(abs(panel.sp_mx.value() - ex) < 1.0 and abs(panel.sp_my.value() - ey) < 1.0,
          f"拖拽后坐标框同步显示 ({panel.sp_mx.value():.0f}, {panel.sp_my.value():.0f})")

    # 反向: 控件改位置 -> 弹道同步
    panel.sp_tx.setValue(4000.0)
    check(abs(sim.cfg.tx - 4000.0) < 1e-6 and abs(sim.target.x0 - 4000.0) < 1e-6,
          "控件修改目标位置生效")

    # 6) 导引方法切换
    panel.mode_box.setCurrentIndex(1)        # PN
    check(cfg.mode == "pn" and sim.active_keys == ["pn"], "切换到仅比例导引")
    check(panel.table.isColumnHidden(2), "纯跟踪数据列被隐藏")
    panel.mode_box.setCurrentIndex(0)        # both
    check(sim.active_keys == ["pn", "pp"], "切换到双弹对比")
    check(not panel.table.isColumnHidden(2), "纯跟踪数据列显示")

    # 7) 目标机动模式
    panel.mode_t.setCurrentIndex(3)          # 正弦机动
    check(cfg.t_mode == "sine", "目标机动模式切换为正弦")
    panel.mode_t.setCurrentIndex(0)

    # 8) 单步
    panel.btn_step.click()
    check(abs(sim.t - DT) < 1e-9, f"单步推进一个步长 (t={sim.t:.4f})")
    QTest.qWait(200)
    check(abs(sim.t - DT) < 1e-9, "单步后不会自动继续")

    # 9) 重置 / 默认参数
    panel.btn_reset.click()
    check(sim.t == 0.0 and not sim.finished, "重置按钮生效")
    panel.sp_n.setValue(2.0)
    panel.btn_default.click()
    d = SimConfig()
    cur = win.sim.cfg                       # 注意: 默认参数会整体替换 cfg 对象
    check(abs(cur.nav_ratio - d.nav_ratio) < 1e-9 and cur.mode == d.mode
          and abs(cur.t_heading - d.t_heading) < 1e-9,
          "恢复默认参数")

    # 10) 跑完整个交战(提高倍速以缩短测试时间)
    panel.speed.setValue(80)                # 8.0x
    panel.btn_run.setChecked(True)
    for _ in range(600):                    # 最多等 12s 实时 = 96s 仿真
        QTest.qWait(20)
        if sim.finished:
            break
    check(sim.finished, f"交战结束 (t={sim.t:.2f}s)")
    summary = sim.summary()
    print("     summary:", summary, flush=True)
    check("比例导引" in summary and "纯跟踪" in summary, "汇总包含两种导引律")
    check(sim.missiles["pn"].done and sim.missiles["pp"].done, "两枚导弹均已出结果")
    check(not panel.btn_run.isChecked(), "结束后运行按钮自动复位")

    # 11) 结束后再按开始 = 重新开始
    panel.btn_run.setChecked(True)
    QTest.qWait(200)
    check(sim.t > 0.0 and not sim.finished, "结束后再按开始可重新交战")
    panel.btn_run.setChecked(False)

    # 12) 数据表有内容
    win._refresh()
    check(panel.table.item(0, 1).text() not in ("", "-"), "数据表已填充")
    win.grab().save("gui_test.png")
    print("截图已保存: gui_test.png", flush=True)

    # 13) 帮助文档窗口
    check(panel.btn_help.isVisible(), "面板顶部帮助按钮可见")
    win.open_help()
    QTest.qWait(250)
    h = win._help
    check(h is not None and h.isVisible(), "点击帮助按钮打开文档窗口")
    n = h.listw.count()
    names = [h.listw.item(i).text() for i in range(n)]
    check(n >= 6, f"docs 目录下 {n} 个 md 文档已载入: {names}")
    check(names and names[0] == "README", f"README 排在列表首位 ({names[:1]})")
    txt = h.browser.toPlainText()
    check("比例导引" in txt and "纯跟踪" in txt, "README 正文渲染成功")
    check("03_仿真参数详解" in h.browser.toHtml() or "仿真参数详解" in txt,
          "README 目录链接指向各文档")

    h.listw.setCurrentRow(3)                      # 03_仿真参数详解
    QTest.qWait(200)
    t3 = h.browser.toPlainText()
    check("Gmax" in t3 and "导航比" in t3 and "跟踪增益" in t3,
          "参数文档渲染出关键参数说明")
    check("03_仿真参数详解" in h.status.text(), f"状态栏显示当前文件 ({h.status.text()[:30]})")
    html = h.browser.toHtml()
    check("<table" in html, "Markdown 表格已转换为 HTML 表格")
    # Qt 把 blockquote 导出为带 margin-left + background-color 的 <p>
    check("background-color:#14203a" in html and "修改任意参数" in t3,
          "引用块已渲染(带缩进与底色)")

    # 文档内链接跳转
    from PyQt5.QtCore import QUrl
    h._on_anchor(QUrl("01_比例导引法PN.md"))
    QTest.qWait(200)
    check("01_比例导引法PN" in h.status.text(), "点击文档链接切换到 01 文档")
    check("<pre" in h.browser.toHtml(), "围栏代码块已转换为 pre")
    check("<h2" in h.browser.toHtml(), "标题已转换为 h1~h6 标签")

    # 新文档 05 的渲染
    h._on_anchor(QUrl("05_中段制导与拦截点计算.md"))
    QTest.qWait(200)
    t5 = h.browser.toPlainText()
    check("05_中段制导与拦截点计算" in h.status.text(), "点击链接跳转到 05 文档")
    check("拦截点" in t5 and "路径规划" in t5 and "截获" in t5,
          "05 文档正文渲染成功")
    html5 = h.browser.toHtml()
    check("<table" in html5 and "<pre" in html5 and "<h2" in html5,
          "05 文档的表格/代码块/标题均已渲染")

    # 用户自行增删改 md -> 刷新后重新加载
    extra = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs",
                         "_tmp_test_doc.md")
    try:
        with open(extra, "w", encoding="utf-8") as fh:
            fh.write("# 临时文档\n\n这是**自定义**文档测试。\n")
        h.reload()
        check(h.listw.count() == n + 1, f"新增 md 后刷新可载入 ({h.listw.count()})")
        h.show_file("_tmp_test_doc")
        QTest.qWait(150)
        check("自定义" in h.browser.toPlainText(), "自定义文档内容渲染成功")
    finally:
        if os.path.exists(extra):
            os.remove(extra)
        h.reload()
    check(h.listw.count() == n, "删除 md 后刷新列表恢复")

    # 文档内所有链接都指向真实存在的 md 文件
    docs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs")
    link_re = re.compile(r"\]\(([^)#]+\.md)\)")
    broken = []
    nlinks = 0
    for fn in sorted(os.listdir(docs_dir)):
        if not fn.endswith(".md"):
            continue
        with open(os.path.join(docs_dir, fn), encoding="utf-8") as fh:
            for t in link_re.findall(fh.read()):
                nlinks += 1
                if not os.path.exists(os.path.join(docs_dir, t)):
                    broken.append((fn, t))
    check(not broken, f"文档内 {nlinks} 个 md 链接全部有效 {broken or ''}")

    # F1 快捷键 + 关闭/重开
    h.hide()
    QTest.qWait(150)                      # 等 hideEvent 把激活交还主窗口
    check(not h.isVisible(), "关闭帮助窗口")
    QTest.keyPress(win, Qt.Key_F1)
    QTest.qWait(200)
    check(h.isVisible(), "F1 打开帮助窗口")
    win.open_help()
    check(h.isVisible(), "重复调用不会重复创建窗口")
    h.grab().save("preview_help.png")
    print("截图已保存: preview_help.png", flush=True)
    h.hide()

    # 14) 分段制导(中段 -> 末段 + 导引头截获判据)
    panel.cb_phased.setChecked(True)
    cfg = win.sim.cfg                         # 注意: 默认参数按钮会整体替换 cfg 对象
    check(cfg.phased, "启用分段制导写入 cfg")
    check(panel.sp_acq.isEnabled() and panel.sp_fov.isEnabled()
          and panel.mode_mid.isEnabled(), "启用后截获参数控件可用")
    check(abs(cfg.acq_range - 4000.0) < 1e-9 and abs(cfg.acq_fov - 90.0) < 1e-9,
          f"截获参数默认值 (R_acq={cfg.acq_range:.0f} m, FOV={cfg.acq_fov:.0f}°)")
    pn = sim.missiles["pn"]
    check(pn.phase == "中段" and not pn.acquired, "初始处于中段(导引头未截获)")
    phase_row = next(i for i, (_, tag) in enumerate(panel.ROWS) if tag == "phase")
    acq_row = next(i for i, (_, tag) in enumerate(panel.ROWS) if tag == "acq")
    win._refresh()
    check(panel.table.item(phase_row, 1).text() == "中段",
          f"数据表显示制导阶段 ({panel.table.item(phase_row, 1).text()})")
    check(panel.table.item(acq_row, 1).text() == "-",
          "未截获时截获时刻显示 -")

    # 中段方式 / 截获参数写入 cfg
    panel.mode_mid.setCurrentIndex(1)
    check(win.sim.cfg.mid_mode == "straight", "中段方式切换为程序直飞")
    panel.mode_mid.setCurrentIndex(0)
    panel.sp_acq.setValue(1500.0)
    check(abs(win.sim.cfg.acq_range - 1500.0) < 1e-9,
          f"截获距离写入 cfg ({win.sim.cfg.acq_range:.0f} m)")
    panel.sp_fov.setValue(45.0)
    check(abs(win.sim.cfg.acq_fov - 45.0) < 1e-9,
          f"截获视场写入 cfg ({win.sim.cfg.acq_fov:.0f}°)")
    panel.sp_acq.setValue(4000.0)
    panel.sp_fov.setValue(90.0)
    check(win.sim.cfg.mid_mode == "pip" and abs(win.sim.cfg.acq_fov - 90.0) < 1e-9,
          "截获参数恢复默认")

    # 跑到截获 -> 转入末段
    pn = sim.missiles["pn"]                   # 每次改参数都会重建导弹对象
    panel.btn_run.setChecked(True)
    for _ in range(600):
        QTest.qWait(20)
        if pn.acquired or sim.finished:
            break
    check(pn.acquired, f"导引头截获 (t={pn.acq_t:.2f} s, R={pn.acq_r:.0f} m)")
    check(pn.phase == "末段" and pn.acq_pos is not None, "截获后转入末段制导")
    check(pn.trail_split >= 2, f"弹道按截获点分段 (split={pn.trail_split})")
    win._refresh()
    check(panel.table.item(phase_row, 1).text() == "末段", "数据表阶段更新为末段")
    check(panel.table.item(acq_row, 1).text() == f"{pn.acq_t:.2f}",
          f"数据表显示截获时刻 ({panel.table.item(acq_row, 1).text()})")

    for _ in range(600):
        QTest.qWait(20)
        if sim.finished:
            break
    check(sim.finished and "未截获" not in sim.summary(),
          f"截获后正常交战结束 ({sim.summary()[:70]})")

    # 截获不了的情形: 程序直飞 + 视场 10°
    panel.mode_mid.setCurrentIndex(1)
    panel.sp_fov.setValue(10.0)
    pn = sim.missiles["pn"]
    check(not pn.acquired and pn.phase == "中段", "程序直飞+窄视场: 开局处于中段")
    panel.btn_run.setChecked(True)
    for _ in range(700):
        QTest.qWait(20)
        if sim.finished:
            break
    check(sim.finished and not pn.acquired, f"全程未截获 (飞行 {pn.t:.1f} s)")
    check("未截获" in pn.result and "未截获" in sim.summary(),
          f"结果附注导引头未截获 ({pn.result})")

    # 关闭分段制导 -> 恢复"开机即末段"
    panel.cb_phased.setChecked(False)
    cfg = win.sim.cfg
    check(not cfg.phased, "关闭分段制导写入 cfg")
    check(not panel.sp_acq.isEnabled() and not panel.mode_mid.isEnabled(),
          "关闭后其子控件置灰")
    pn = sim.missiles["pn"]
    check(pn.phase == "末段" and pn.acquired and pn.trail_split == 0,
          "关闭后直接按末段制导运行")
    win._refresh()
    check(panel.table.item(phase_row, 1).text() == "末段",
          "关闭分段制导后数据表阶段=末段")

    print("-" * 60)
    print("全部通过" if not FAILS else f"失败 {len(FAILS)} 项: {FAILS}",
          flush=True)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
