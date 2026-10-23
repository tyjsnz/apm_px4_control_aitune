// 主题切换: 白天(亮色) / 夜间(暗色, 默认)
// 键 fy_theme ('dark'|'light'); 切换后派发 document 事件 'fy:theme' 供图表重绘
window.Theme = (function () {
  const KEY = 'fy_theme';
  const META_COLOR = { dark: '#0A0A0F', light: '#F7F8FA' };

  function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
  function lsSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* 隐私模式忽略 */ } }

  function current() {
    return document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark';
  }

  function apply(t, save) {
    const theme = t === 'light' ? 'light' : 'dark';
    document.documentElement.setAttribute('data-theme', theme);
    const meta = document.querySelector('meta[name="theme-color"]');
    if (meta) {
      // dark 色取该页初始值 (index #0A0A0F / landing #0b0f17), 首次 apply 前捕获
      if (!meta.dataset.fyDark) meta.dataset.fyDark = meta.getAttribute('content') || META_COLOR.dark;
      meta.setAttribute('content', theme === 'light' ? META_COLOR.light : meta.dataset.fyDark);
    }
    const dark = theme === 'dark';
    ['themeToggle', 'landingThemeToggle'].forEach((id) => {
      const b = document.getElementById(id);
      if (!b) return;
      b.textContent = dark ? '☀️ 白天' : '🌙 夜间';
      b.title = dark ? '切换到白天(亮色)主题' : '切换到夜间(暗色)主题';
      b.setAttribute('aria-label', b.title);
    });
    if (save) lsSet(KEY, theme);
    try { document.dispatchEvent(new CustomEvent('fy:theme', { detail: { theme } })); } catch (e) { /* IE 兜底 */ }
  }

  function toggle() { apply(current() === 'dark' ? 'light' : 'dark', true); }

  function init() {
    apply(lsGet(KEY) === 'light' ? 'light' : 'dark', false);
    ['themeToggle', 'landingThemeToggle'].forEach((id) => {
      const b = document.getElementById(id);
      if (b && !b.dataset.themeBound) { b.dataset.themeBound = '1'; b.addEventListener('click', toggle); }
    });
  }

  return { init, toggle, apply, current };
})();

if (typeof document !== 'undefined') {
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', window.Theme.init);
  else window.Theme.init();
}
