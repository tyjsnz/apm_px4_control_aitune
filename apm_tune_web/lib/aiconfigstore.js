// 全局 AI 配置存储 (服务端权威)
// - provider / model / baseUrl 始终落盘
// - API Key: 仅当 remember=true 时落盘; 否则只保留在服务进程内存(重启即失)
// - 对外 getPublic() 不返回明文 Key, 只返回 hasKey
import fs from 'fs';
import path from 'path';

export function createAiConfigStore(file) {
  const state = { provider: '', model: '', baseUrl: '', remember: false, apiKey: '' };

  function load() {
    try {
      if (!fs.existsSync(file)) return;
      const d = JSON.parse(fs.readFileSync(file, 'utf-8'));
      state.provider = d.provider || '';
      state.model = d.model || '';
      state.baseUrl = d.baseUrl || '';
      state.remember = !!d.remember;
      state.apiKey = state.remember ? (d.apiKey || '') : '';
    } catch (e) { /* 配置损坏则忽略 */ }
  }

  function persist() {
    fs.mkdirSync(path.dirname(file), { recursive: true });
    const d = {
      provider: state.provider,
      model: state.model,
      baseUrl: state.baseUrl,
      remember: state.remember,
    };
    if (state.remember && state.apiKey) d.apiKey = state.apiKey;
    fs.writeFileSync(file, JSON.stringify(d, null, 2), 'utf-8');
  }

  /** 不含明文 Key, 供前端展示 */
  function getPublic() {
    return {
      provider: state.provider,
      model: state.model,
      baseUrl: state.baseUrl,
      remember: state.remember,
      hasKey: !!state.apiKey,
    };
  }

  /** 含明文 Key, 供服务端发起请求 */
  function get() {
    return {
      provider: state.provider,
      model: state.model,
      baseUrl: state.baseUrl,
      apiKey: state.apiKey,
    };
  }

  /**
   * 更新配置。
   * - cfg.apiKey 为非空字符串时更新内存 Key;
   * - remember 变化时重新落盘(不记住则不写 Key, 但内存保留以便本次会话继续使用)。
   */
  function set(cfg = {}) {
    if ('provider' in cfg) state.provider = String(cfg.provider || '');
    if ('model' in cfg) state.model = String(cfg.model || '').trim();
    if ('baseUrl' in cfg) state.baseUrl = String(cfg.baseUrl || '').trim();
    if ('remember' in cfg) state.remember = !!cfg.remember;
    if (typeof cfg.apiKey === 'string' && cfg.apiKey.trim()) state.apiKey = cfg.apiKey.trim();
    persist();
    return getPublic();
  }

  /** 清除内存与本机保存的 Key */
  function clearKey() {
    state.apiKey = '';
    persist();
    return getPublic();
  }

  /** 合并请求覆盖项与全局配置, 返回真正用于调用 AI 的参数 */
  function resolve(msg = {}) {
    return {
      provider: msg.provider || state.provider,
      model: msg.model || state.model,
      baseUrl: msg.baseUrl || state.baseUrl,
      apiKey: msg.apiKey || state.apiKey,
    };
  }

  load();
  return { file, get, getPublic, set, clearKey, resolve };
}
