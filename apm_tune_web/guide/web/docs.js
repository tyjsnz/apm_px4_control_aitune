// 理论文档库: 目录 + 取文 + Markdown 渲染 (配合 md.js)
// URL: docs.html?lib=docs|refs&f=<文件名>  (缺省 lib=docs, f=该库 README)
import { renderMarkdown } from './md.js';

// ---------------------------------------------------------------- 清单 ----
// 与 guide/docs/、guide/references/ 保持一致 (新增文件时在此登记)
const LIBS = [
  {
    id: 'docs',
    title: '📖 理论手册',
    desc: '应用内帮助: 公式推导 · 参数含义 · 界面操作',
    base: '/guide/docs/',
    home: 'README.md',
    homeTitle: '帮助文档总目录',
    files: [
      'README.md',
      '01_比例导引法PN.md',
      '02_纯跟踪法PP.md',
      '03_仿真参数详解.md',
      '04_界面与操作解读.md',
      '05_中段制导与拦截点计算.md',
    ],
  },
  {
    id: 'refs',
    title: '📚 参考资料',
    desc: '从发射到命中的全流程工程链条 (10 章 + 插图)',
    base: '/guide/refs/',
    home: 'README.md',
    homeTitle: '参考资料总目录',
    files: [
      'README.md',
      '01_全流程总览_从发射到命中.md',
      '02_发射前准备与地面引导.md',
      '03_路径规划与航迹生成.md',
      '04_位置修正_地面与卫星导航.md',
      '05_组合导航与状态滤波.md',
      '06_中段制导与拦截点计算.md',
      '07_导引头与目标探测.md',
      '08_末段导引策略_经典方法.md',
      '09_末段导引策略_现代与智能方法.md',
      '10_工程实践_场景指标与案例.md',
    ],
  },
];

const libById = (id) => LIBS.find((l) => l.id === id) || LIBS[0];

function labelOf(lib, f) {
  if (f === lib.home) return lib.homeTitle;
  return f.replace(/^\d+_/, '').replace(/\.md$/, '').replace(/_/g, ' ');
}

function docUrl(lib, f) {
  return `docs.html?lib=${encodeURIComponent(lib.id)}&f=${encodeURIComponent(f)}`;
}

// -------------------------------------------------------------- 路由改写 --
function makeResolve(lib) {
  return {
    link(href) {
      if (/^[a-z][a-z0-9+.-]*:/i.test(href) || href.startsWith('#')) return href;
      const name = href.split('/').pop();          // ../x.md 与 x.md 同名处理
      if (/\.md$/i.test(name)) {
        const target = LIBS.find((l) => l.files.includes(name));
        if (target) return docUrl(target, name);
        return 'docs.html';                        // 清单外 (如 ../README.md) -> 文档库首页
      }
      if (/\.html?$/i.test(name)) return href;
      return docUrl(lib, lib.home);
    },
    img(src) {
      if (/^[a-z][a-z0-9+.-]*:/i.test(src)) return src;
      // 相对图片按文档库 base 解析 (figures/xxx.png -> /guide/refs/figures/...)
      const clean = src.replace(/^\.\//, '').replace(/^\.\.\//, '');
      return lib.base + clean;
    },
  };
}

// ---------------------------------------------------------------- 侧栏 ----
function buildSidebar(active) {
  const sb = document.getElementById('docsSidebar');
  sb.innerHTML = '';
  for (const lib of LIBS) {
    const sec = document.createElement('div');
    sec.className = 'lib';
    const h = document.createElement('div');
    h.className = 'lib-title';
    h.textContent = lib.title;
    sec.appendChild(h);
    const d = document.createElement('div');
    d.className = 'lib-desc';
    d.textContent = lib.desc;
    sec.appendChild(d);

    const ul = document.createElement('ul');
    ul.className = 'doc-list';
    for (const f of lib.files) {
      const li = document.createElement('li');
      const a = document.createElement('a');
      a.href = docUrl(lib, f);
      a.textContent = labelOf(lib, f);
      if (active && active.lib === lib.id && active.f === f) {
        a.className = 'on';
        li.className = 'on';
      }
      li.appendChild(a);
      ul.appendChild(li);
    }
    sec.appendChild(ul);
    sb.appendChild(sec);
  }
}

// ---------------------------------------------------------------- 取文 ----
async function load() {
  const params = new URLSearchParams(location.search);
  const lib = libById(params.get('lib'));
  const f = params.get('f') || lib.home;
  const file = lib.files.includes(f) ? f : lib.home;

  buildSidebar({ lib: lib.id, f: file });

  const meta = document.getElementById('docsMeta');
  const body = document.getElementById('docsBody');
  const pager = document.getElementById('docsPager');
  document.title = `${labelOf(lib, file)} — 导引算法理论文档`;

  meta.innerHTML = `<span class="crumb">${lib.title}</span>
    <span class="fname">${file}</span>
    <a class="back-sim" href="/guide/">🎯 打开交互仿真 →</a>`;

  let res = null;
  try {
    res = await fetch(lib.base + encodeURIComponent(file), { cache: 'no-cache' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const md = await res.text();

    // 用正文一级标题刷新目录项显示名
    const m = /^#\s+(.+)$/m.exec(md);
    if (m) {
      const want = docUrl(lib, file);
      for (const a of document.querySelectorAll('.doc-list a')) {
        if (a.getAttribute('href') === want) { a.textContent = m[1].trim(); break; }
      }
    }

    body.innerHTML = renderMarkdown(md, makeResolve(lib));
    meta.insertAdjacentHTML('beforeend',
      `<span class="size">${(new Blob([md]).size / 1024).toFixed(1)} KB</span>`);

    // 上一篇 / 下一篇
    const i = lib.files.indexOf(file);
    const prev = i > 0 ? lib.files[i - 1] : null;
    const next = i >= 0 && i < lib.files.length - 1 ? lib.files[i + 1] : null;
    pager.innerHTML = `
      ${prev ? `<a class="pg" href="${docUrl(lib, prev)}">← ${labelOf(lib, prev)}</a>` : '<span></span>'}
      <a class="pg home" href="${docUrl(lib, lib.home)}">目录</a>
      ${next ? `<a class="pg" href="${docUrl(lib, next)}">${labelOf(lib, next)} →</a>` : '<span></span>'}`;

    // 锚点定位
    if (location.hash) {
      const el = document.getElementById(decodeURIComponent(location.hash.slice(1)));
      if (el) { el.scrollIntoView(); return; }
    }
    window.scrollTo(0, 0);
    document.querySelector('.docs-main')?.scrollTo?.(0, 0);
  } catch (err) {
    body.innerHTML = `<div class="doc-error">
      <h3>文档加载失败</h3>
      <p><code>${file}</code> (${res ? `HTTP ${res.status}` : err.message})</p>
      <p><a href="docs.html">← 返回文档库首页</a></p></div>`;
    pager.innerHTML = '';
  }
}

// 移动端目录抽屉 (点遮罩/正文/切换按钮收起)
document.getElementById('libToggle').addEventListener('click', () => {
  document.body.classList.toggle('panel-open');
});
document.addEventListener('click', (e) => {
  if (!document.body.classList.contains('panel-open')) return;
  if (e.target.closest && e.target.closest('#docsSidebar, #libToggle')) return;
  document.body.classList.remove('panel-open');
});

load();
