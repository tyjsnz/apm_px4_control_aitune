// 导引算法实验室 —— 主窗口逻辑 (由 guide/main.py 移植)
import { Simulation, SimConfig } from './core/simulation.js';
import { clamp } from './core/guidance.js';
import { Canvas } from './canvas.js';
import { Plot } from './plot.js';
import { Panel } from './panel.js';

class App {
  constructor() {
    this.sim = new Simulation();
    this.running = false;
    this.acc = 0.0;
    this.dirty = true;
    this.lastFrame = performance.now();

    this.fieldCv = document.getElementById('field');
    this.plotCv = document.getElementById('plot');
    this.statusBar = document.getElementById('statusbar');

    this.canvas = new Canvas(this.fieldCv, this.sim);
    this.plot = new Plot(this.plotCv, this.sim);
    this.canvas.onDrag = (kind, x, y) => this.onPosition(kind, x, y);

    this.panel = new Panel(this.sim, document.getElementById('panel'), {
      params: () => this.onParams(),
      position: (kind, x, y) => this.onPosition(kind, x, y),
      run: (on) => this.onRun(on),
      step: () => this.onStep(),
      reset: () => this.onReset(),
      defaults: () => this.onDefaults(),
      help: () => this.openHelp(),
    });

    this._bindKeys();
    this._bindResize();
    this._bindPanelToggle();

    this._resizeAll();
    this.refresh();
    requestAnimationFrame((t) => this.tick(t));
  }

  // ============================================================== 逻辑 ===
  onParams() {
    this.panel.writeCfg();
    this._reset();
  }

  onPosition(kind, x, y) {
    const size = this.sim.cfg.field_size;
    x = clamp(x, 0.0, size);
    y = clamp(y, 0.0, size);
    if (kind === 'm') {
      this.sim.cfg.mx = x;
      this.sim.cfg.my = y;
    } else {
      this.sim.cfg.tx = x;
      this.sim.cfg.ty = y;
    }
    this._reset();
  }

  onRun(on) {
    if (on && this.sim.finished) {
      // 已结束后再次点"开始" = 重新开始
      this._reset(true);
    }
    this.running = on && !this.sim.finished;
    this.panel.setRunning(this.running);
  }

  toggleRun() {
    this.onRun(!this.panel.running);
  }

  onStep() {
    this.panel.setRunning(false);
    this.running = false;
    if (!this.sim.finished) this.sim.step();
    this.dirty = true;
    this.refresh();
  }

  onReset() {
    this.panel.setRunning(false);
    this.running = false;
    this._reset();
  }

  onDefaults() {
    this.sim.cfg = new SimConfig();
    this.panel.setRunning(false);
    this.running = false;
    this._reset();
  }

  openHelp() {
    window.open('docs.html', '_blank');
  }

  _reset(keepRunState = false) {
    const c = this.sim.cfg;
    const size = c.field_size;
    c.mx = clamp(c.mx, 0.0, size); c.my = clamp(c.my, 0.0, size);
    c.tx = clamp(c.tx, 0.0, size); c.ty = clamp(c.ty, 0.0, size);
    if (!keepRunState) {
      this.panel.setRunning(false);
      this.running = false;
    }
    this.acc = 0.0;
    this.sim.reset();
    this.panel.syncFromCfg();
    this.dirty = true;
    this.refresh();
  }

  // ============================================================== 定时 ===
  tick(now) {
    const dtReal = Math.min((now - this.lastFrame) / 1000.0, 0.25); // 切后台不追帧
    this.lastFrame = now;

    if (this.running && !this.sim.finished) {
      this.acc += dtReal * this.panel.speedFactor();
      let n = 0;
      while (this.acc >= this.sim.DT && n < 500) {
        this.sim.step();
        this.acc -= this.sim.DT;
        n++;
        if (this.sim.finished) {
          this.acc = 0.0;
          this.panel.setRunning(false);
          this.running = false;
          break;
        }
      }
      this.refresh();
    } else if (this.dirty) {
      this.dirty = false;
      this.refresh();
    }
    requestAnimationFrame((t) => this.tick(t));
  }

  refresh() {
    this.canvas.draw();
    this.plot.draw();
    this.panel.updateData();
    const state = this.sim.finished ? '结束'
      : ((this.running || this.sim.running) ? '运行中' : '就绪');
    this.statusBar.textContent =
      `t=${this.sim.t.toFixed(2)}s  [${state}]   ${this.sim.summary()}`;
  }

  // ============================================================== 事件 ===
  _bindKeys() {
    window.addEventListener('keydown', (e) => {
      const tag = (e.target && e.target.tagName) || '';
      const typing = /INPUT|SELECT|TEXTAREA/.test(tag);
      if (e.key === 'F1') {
        e.preventDefault();
        this.openHelp();
        return;
      }
      if (typing || e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.code === 'Space') {
        e.preventDefault();
        this.toggleRun();
      } else if (e.key === 'r' || e.key === 'R') {
        this.onReset();
      } else if (e.key === 's' || e.key === 'S') {
        this.onStep();
      }
    });
  }

  _bindResize() {
    const ro = new ResizeObserver(() => {
      this._resizeAll();
      this.dirty = true;
      this.refresh();
    });
    ro.observe(this.fieldCv.parentElement);
    ro.observe(this.plotCv.parentElement);
    window.addEventListener('orientationchange', () => {
      setTimeout(() => { this._resizeAll(); this.refresh(); }, 120);
    });
  }

  _bindPanelToggle() {
    const btn = document.getElementById('panelToggle');
    if (btn) {
      btn.addEventListener('click', () => {
        document.body.classList.toggle('panel-open');
      });
    }
    // 点击遮罩/面板外区域时收起移动端抽屉
    document.addEventListener('click', (e) => {
      if (!document.body.classList.contains('panel-open')) return;
      if (e.target.closest && e.target.closest('#panel, #panelToggle')) return;
      document.body.classList.remove('panel-open');
    });
  }

  _resizeAll() {
    this.canvas.resize();
    this.plot.resize();
  }
}

function boot() {
  window.__guideApp = new App();   // 便于自动化测试注入/检查
}
if (document.readyState === 'loading') {
  window.addEventListener('DOMContentLoaded', boot);
} else {
  boot();
}

export { App };
