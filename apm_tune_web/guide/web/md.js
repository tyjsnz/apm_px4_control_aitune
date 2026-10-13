// 轻量 Markdown 渲染器 (针对本项目 15 篇文档的语法子集重写)
// 支持: ATX 标题 / 段落 / 无序·有序列表(嵌套) / 表格 / 代码块 / 行内代码 /
//       引用 / 分隔线 / 粗体 / 斜体 / 删除线 / 链接 / 图片
// 安全性: 文本先转义再做行内替换; 链接/图片经 resolve 回调改写路由。

const escapeHtml = (s) => s
  .replace(/&/g, '&amp;')
  .replace(/</g, '&lt;')
  .replace(/>/g, '&gt;')
  .replace(/"/g, '&quot;');

const RE_HEADING = /^(#{1,6})\s+(.*)$/;
const RE_HR = /^\s*([-*_])\s*(?:\1\s*){2,}$/;
const RE_UL = /^(\s*)[-*+]\s+(.*)$/;
const RE_OL = /^(\s*)\d{1,9}[.)]\s+(.*)$/;
const RE_QUOTE = /^\s*>\s?(.*)$/;
const RE_FENCE = /^\s*```/;

// ---------------------------------------------------------------- 行内 ----
function renderInline(text, resolve) {
  // 1) 保护行内代码
  const codes = [];
  let s = text.replace(/`([^`]+)`/g, (_, c) => {
    codes.push(c);
    return `\u0000${codes.length - 1}\u0000`;
  });

  // 2) 转义 HTML (此时仍保留 markdown 标记)
  s = escapeHtml(s);

  // 3) 图片 (先于链接)
  s = s.replace(/!\[([^\]]*)\]\(([^)\s]+)(?:\s+&quot;[^&]*&quot;)?\)/g,
    (_, alt, src) => `<img src="${resolve.img(src)}" alt="${alt}" loading="lazy">`);

  // 4) 链接
  s = s.replace(/\[([^\]]+)\]\(([^)\s]+)(?:\s+&quot;([^)]*)&quot;)?\)/g,
    (_, txt, href, title) => {
      const u = resolve.link(href);
      const t = title ? ` title="${title}"` : '';
      const ext = /^https?:/i.test(u) ? ' target="_blank" rel="noopener"' : '';
      return `<a href="${u}"${t}${ext}>${txt}</a>`;
    });

  // 5) 强调
  s = s.replace(/\*\*\*([^*]+)\*\*\*/g, '<strong><em>$1</em></strong>');
  s = s.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
  s = s.replace(/(^|[^*\w])\*([^*\n]+)\*(?=[^*\w]|$)/g, '$1<em>$2</em>');
  s = s.replace(/(^|[^_\w])_([^_\n]+)_(?=[^_\w]|$)/g, '$1<em>$2</em>');
  s = s.replace(/~~([^~]+)~~/g, '<del>$1</del>');

  // 6) 还原行内代码
  s = s.replace(/\u0000(\d+)\u0000/g, (_, i) => `<code>${escapeHtml(codes[+i])}</code>`);
  return s;
}

// ---------------------------------------------------------------- 表格 ----
function splitRow(line) {
  let t = line.trim();
  if (t.startsWith('|')) t = t.slice(1);
  if (t.endsWith('|')) t = t.slice(0, -1);
  return t.split('|').map((c) => c.trim());
}

function isTableSep(line) {
  return /^\s*\|?\s*:?-{2,}:?\s*(\|\s*:?-{2,}:?\s*)*\|?\s*$/.test(line);
}

// ---------------------------------------------------------------- 列表 ----
/** 解析同类型列表块; 返回 { html, consumed }。调用方保证顶层类型/缩进一致。 */
function parseList(lines, resolve) {
  const m0 = RE_UL.exec(lines[0]) || RE_OL.exec(lines[0]);
  const ordered = !RE_UL.test(lines[0]);
  const baseIndent = m0[1].length;

  const items = [];        // { text, sub: string[] }
  let i = 0;
  while (i < lines.length) {
    const mU = RE_UL.exec(lines[i]);
    const mO = RE_OL.exec(lines[i]);
    const m = mU || mO;
    if (m) {
      const indent = m[1].length;
      if (indent < baseIndent) break;
      if (indent === baseIndent) {
        if (!!mU === ordered) break;   // 顶层类型切换 -> 交给上层重新分组
        items.push({ text: m[2], sub: [] });
        i++;
        continue;
      }
      // 更深缩进: 归入上一条的子块 (保持原始行, 由 renderBlocks 递归)
      if (items.length) items[items.length - 1].sub.push(lines[i]);
      i++;
      continue;
    }
    if (/^\s+\S/.test(lines[i]) && items.length) {
      // 缩进续行: 若已有子块则并入子块, 否则并入条目文本
      const last = items[items.length - 1];
      if (last.sub.length) last.sub.push(lines[i]);
      else last.text += ' ' + lines[i].trim();
      i++;
      continue;
    }
    break;
  }

  const out = items.map((it) => {
    const inner = it.sub.length ? renderBlocks(it.sub, resolve) : '';
    return `<li>${renderInline(it.text, resolve)}${inner}</li>`;
  }).join('');
  const tag = ordered ? 'ol' : 'ul';
  return { html: `<${tag}>${out}</${tag}>`, consumed: i };
}

// ------------------------------------------------------------ 区块解析 ----
function renderBlocks(lines, resolve) {
  const html = [];
  let i = 0;
  while (i < lines.length) {
    const line = lines[i];

    if (!line.trim()) { i++; continue; }

    // 代码块
    if (RE_FENCE.test(line)) {
      const lang = line.trim().slice(3).trim();
      const buf = [];
      i++;
      while (i < lines.length && !RE_FENCE.test(lines[i])) {
        buf.push(lines[i]);
        i++;
      }
      i++; // 收尾 ```
      const cls = lang ? ` class="lang-${escapeHtml(lang)}"` : '';
      html.push(`<pre><code${cls}>${escapeHtml(buf.join('\n'))}</code></pre>`);
      continue;
    }

    // 标题
    const h = RE_HEADING.exec(line);
    if (h) {
      const lv = h[1].length;
      const txt = h[2].trim();
      const id = txt.replace(/`/g, '').replace(/\s+/g, '-');
      html.push(`<h${lv} id="${escapeHtml(id)}">${renderInline(txt, resolve)}</h${lv}>`);
      i++;
      continue;
    }

    // 分隔线
    if (RE_HR.test(line)) { html.push('<hr>'); i++; continue; }

    // 引用 (连续 > 行 + 引用内普通续行)
    if (RE_QUOTE.test(line)) {
      const buf = [];
      while (i < lines.length) {
        const m = RE_QUOTE.exec(lines[i]);
        if (m) { buf.push(m[1]); i++; continue; }
        if (lines[i].trim() && buf.length && !RE_FENCE.test(lines[i]) &&
            !RE_HEADING.test(lines[i]) && !RE_HR.test(lines[i]) &&
            !RE_UL.test(lines[i]) && !RE_OL.test(lines[i])) {
          buf.push(lines[i]);
          i++;
          continue;
        }
        break;
      }
      html.push(`<blockquote>${renderBlocks(buf, resolve)}</blockquote>`);
      continue;
    }

    // 表格
    if (line.includes('|') && i + 1 < lines.length && isTableSep(lines[i + 1])) {
      const head = splitRow(line);
      const aligns = splitRow(lines[i + 1]).map((c) => {
        if (c.startsWith(':') && c.endsWith(':')) return 'center';
        if (c.endsWith(':')) return 'right';
        return 'left';
      });
      i += 2;
      const rows = [];
      while (i < lines.length && lines[i].includes('|') && lines[i].trim()) {
        rows.push(splitRow(lines[i]));
        i++;
      }
      let t = '<table><thead><tr>';
      head.forEach((c, k) => {
        t += `<th style="text-align:${aligns[k] || 'left'}">${renderInline(c, resolve)}</th>`;
      });
      t += '</tr></thead><tbody>';
      for (const r of rows) {
        t += '<tr>';
        r.forEach((c, k) => {
          t += `<td style="text-align:${aligns[k] || 'left'}">${renderInline(c, resolve)}</td>`;
        });
        t += '</tr>';
      }
      html.push(`${t}</tbody></table>`);
      continue;
    }

    // 列表 (只收集同类型顶层条目, 类型切换时结束本块)
    if (RE_UL.test(line) || RE_OL.test(line)) {
      const firstIsUl = RE_UL.test(line);
      const baseIndent = (RE_UL.exec(line) || RE_OL.exec(line))[1].length;
      const buf = [];
      while (i < lines.length) {
        const mU = RE_UL.exec(lines[i]);
        const mO = RE_OL.exec(lines[i]);
        const m = mU || mO;
        if (m) {
          const indent = m[1].length;
          if (indent < baseIndent) break;
          if (indent === baseIndent) {
            if (!!mU !== firstIsUl) break;   // 同级换类型 -> 新列表块
            buf.push(lines[i]);
            i++;
            continue;
          }
          buf.push(lines[i]);                // 嵌套
          i++;
          continue;
        }
        if (/^\s+\S/.test(lines[i]) && buf.length) { buf.push(lines[i]); i++; continue; }
        break;
      }
      const { html: lh, consumed } = parseList(buf, resolve);
      html.push(lh);
      i -= (buf.length - consumed);         // 未消费的行退回外层循环
      continue;
    }

    // 段落
    const para = [];
    while (i < lines.length && lines[i].trim() &&
      !RE_HEADING.test(lines[i]) && !RE_FENCE.test(lines[i]) &&
      !RE_QUOTE.test(lines[i]) && !RE_HR.test(lines[i]) &&
      !RE_UL.test(lines[i]) && !RE_OL.test(lines[i]) &&
      !(lines[i].includes('|') && i + 1 < lines.length && isTableSep(lines[i + 1]))) {
      para.push(lines[i]);
      i++;
    }
    if (para.length) {
      const body = para.map((l, k) =>
        (k < para.length - 1 && /\s{2,}$/.test(l))
          ? `${renderInline(l.trimEnd(), resolve)}<br>`      // 行尾双空格 = 硬换行
          : renderInline(l.trim(), resolve)).join(' ');
      html.push(`<p>${body}</p>`);
    } else {
      i++; // 防死循环
    }
  }
  return html.join('\n');
}

/**
 * 渲染 Markdown 文本为 HTML。
 * @param {string} md 原文
 * @param {{link:(h:string)=>string, img:(s:string)=>string}} resolve
 *        链接/图片改写回调 (由 docs.html 注入, 负责路由到对应文档)
 */
export function renderMarkdown(md, resolve) {
  const text = md.replace(/\r\n?/g, '\n');
  return renderBlocks(text.split('\n'), resolve || { link: (h) => h, img: (h) => h });
}

export { escapeHtml };
