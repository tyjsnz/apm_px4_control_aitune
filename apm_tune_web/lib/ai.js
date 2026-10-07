// 多服务商 AI 调用 (Node, 使用全局 fetch)
// 安全: API Key 仅用于当次请求, 不写入磁盘/日志/配置
export const PROVIDERS = {
  deepseek: {
    label: 'DeepSeek', type: 'openai',
    baseUrl: 'https://api.deepseek.com',
    models: ['deepseek-flash', 'deepseek-v4-pro'],
  },
  openai: {
    label: 'OpenAI', type: 'openai',
    baseUrl: 'https://api.openai.com/v1',
    models: ['gpt-5.6', 'gpt-5.6-mini', 'gpt-4.1', 'gpt-4.1-mini', 'gpt-4o', 'gpt-4o-mini'],
  },
  anthropic: {
    label: 'Anthropic Claude', type: 'anthropic',
    baseUrl: 'https://api.anthropic.com',
    models: ['claude-sonnet-5', 'claude-opus-5', 'claude-haiku-4-5'],
  },
  gemini: {
    label: 'Google Gemini', type: 'gemini',
    baseUrl: 'https://generativelanguage.googleapis.com/v1beta',
    models: ['gemini-3.5-flash', 'gemini-3.1-pro-preview', 'gemini-2.5-flash'],
  },
  qwen: {
    label: '通义千问 Qwen', type: 'openai',
    baseUrl: 'https://dashscope.aliyuncs.com/compatible-mode/v1',
    models: ['qwen-max', 'qwen-plus', 'qwen-turbo', 'qwen3-max'],
  },
  glm: {
    label: '智谱 GLM', type: 'openai',
    baseUrl: 'https://open.bigmodel.cn/api/paas/v4',
    models: ['glm-4.6', 'glm-4.5', 'glm-4-flash', 'glm-4-plus'],
  },
  kimi: {
    label: 'Kimi 月之暗面', type: 'openai',
    baseUrl: 'https://api.moonshot.cn/v1',
    models: ['kimi-k2.6', 'moonshot-v1-128k', 'moonshot-v1-8k'],
  },
  doubao: {
    label: '豆包 火山方舟', type: 'openai',
    baseUrl: 'https://ark.cn-beijing.volces.com/api/v3',
    models: ['doubao-1-5-pro-32k', 'doubao-pro-32k', 'doubao-lite-4k'],
  },
  openrouter: {
    label: 'OpenRouter 聚合', type: 'openai',
    baseUrl: 'https://openrouter.ai/api/v1',
    models: ['deepseek/deepseek-chat', 'openai/gpt-5.6', 'anthropic/claude-sonnet-5', 'google/gemini-3.5-flash'],
  },
  groq: {
    label: 'Groq', type: 'openai',
    baseUrl: 'https://api.groq.com/openai/v1',
    models: ['llama-3.3-70b-versatile', 'llama-3.1-8b-instant'],
  },
  mistral: {
    label: 'Mistral', type: 'openai',
    baseUrl: 'https://api.mistral.ai/v1',
    models: ['mistral-large-latest', 'mistral-small-latest'],
  },
  ollama: {
    label: 'Ollama 本地', type: 'openai',
    baseUrl: 'http://localhost:11434/v1',
    models: ['qwen3:8b', 'llama3.1:8b', 'deepseek-r1:8b'],
    keyOptional: true,
  },
  custom: {
    label: '自定义 OpenAI 兼容', type: 'openai',
    baseUrl: '', models: [],
  },
};

export function providerList() {
  return Object.entries(PROVIDERS).map(([id, p]) => ({
    id, label: p.label, type: p.type, baseUrl: p.baseUrl,
    models: p.models, keyOptional: !!p.keyOptional,
  }));
}

export function providerInfo(id) {
  return PROVIDERS[id] || PROVIDERS.deepseek;
}

function chatEndpoint(baseUrl) {
  const b = String(baseUrl || '').trim().replace(/\/+$/, '');
  if (!b) return null;
  if (b.endsWith('/chat/completions')) return b;
  return b + '/chat/completions';
}

async function postJson(url, payload, headers, timeoutMs, retries = 3) {
  for (let attempt = 0; attempt < retries; attempt++) {
    const ac = new AbortController();
    const timer = setTimeout(() => ac.abort(), timeoutMs);
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', ...(headers || {}) },
        body: JSON.stringify(payload),
        signal: ac.signal,
      });
      clearTimeout(timer);
      const text = await res.text();
      if (!res.ok) {
        if (res.status === 401 || res.status === 403) return { error: `API Key 无效或无权限 (${res.status})` };
        if (res.status === 402) return { error: '余额不足 (402)' };
        if (res.status === 404) return { error: `接口或模型不存在 (404): ${text.slice(0, 300)}` };
        if (res.status === 429) {
          if (attempt < retries - 1) { await new Promise((r) => setTimeout(r, 2000)); continue; }
          return { error: '请求过于频繁 (429)' };
        }
        if (res.status >= 500) {
          if (attempt < retries - 1) { await new Promise((r) => setTimeout(r, 1000)); continue; }
          return { error: `服务端错误 ${res.status}: ${text.slice(0, 300)}` };
        }
        return { error: `HTTP ${res.status}: ${text.slice(0, 300)}` };
      }
      try {
        return { data: JSON.parse(text) };
      } catch (e) {
        return { error: '响应不是合法 JSON: ' + text.slice(0, 200) };
      }
    } catch (e) {
      clearTimeout(timer);
      if (attempt < retries - 1) { await new Promise((r) => setTimeout(r, 1000)); continue; }
      return { error: '请求失败: ' + (e.name === 'AbortError' ? '超时' : e.message) };
    }
  }
  return { error: '多次重试失败' };
}

async function openaiChat(apiKey, system, user, model, baseUrl, timeoutMs) {
  const url = chatEndpoint(baseUrl);
  if (!url) return { error: 'Base URL 未填写' };
  const headers = apiKey ? { Authorization: 'Bearer ' + apiKey } : {};
  const payload = {
    model,
    messages: [{ role: 'system', content: system }, { role: 'user', content: user }],
    temperature: 0.3, stream: false,
  };
  const r = await postJson(url, payload, headers, timeoutMs);
  if (r.error) return r;
  try {
    const msg = r.data.choices[0].message;
    const content = msg.content !== null && msg.content !== undefined
      ? msg.content : (msg.reasoning_content || '');
    if (!content) return { error: '响应无文本内容' };
    return { text: content };
  } catch (e) {
    return { error: '响应格式异常: ' + JSON.stringify(r.data).slice(0, 300) };
  }
}

async function anthropicChat(apiKey, system, user, model, baseUrl, timeoutMs) {
  const base = String(baseUrl || 'https://api.anthropic.com').trim().replace(/\/+$/, '');
  const url = base + '/v1/messages';
  const headers = { 'x-api-key': apiKey, 'anthropic-version': '2023-06-01' };
  const payload = {
    model, max_tokens: 4096, temperature: 0.3,
    system, messages: [{ role: 'user', content: user }],
  };
  const r = await postJson(url, payload, headers, timeoutMs);
  if (r.error) return r;
  try {
    const text = (r.data.content || []).filter((p) => p.type === 'text').map((p) => p.text || '').join('');
    if (!text) return { error: '响应无文本内容' };
    return { text };
  } catch (e) {
    return { error: '响应格式异常' };
  }
}

async function geminiChat(apiKey, system, user, model, baseUrl, timeoutMs) {
  const base = String(baseUrl || 'https://generativelanguage.googleapis.com/v1beta').trim().replace(/\/+$/, '');
  const url = `${base}/models/${model}:generateContent`;
  const headers = { 'x-goog-api-key': apiKey };
  const payload = {
    systemInstruction: { parts: [{ text: system }] },
    contents: [{ role: 'user', parts: [{ text: user }] }],
    generationConfig: { temperature: 0.3 },
  };
  const r = await postJson(url, payload, headers, timeoutMs);
  if (r.error) return r;
  try {
    const cand = r.data.candidates || [];
    if (!cand.length) return { error: '无返回内容(可能被安全策略拦截)' };
    const text = (cand[0].content?.parts || []).map((p) => p.text || '').join('');
    if (!text) return { error: '响应无文本内容' };
    return { text };
  } catch (e) {
    return { error: '响应格式异常' };
  }
}

/**
 * 统一调用入口。apiKey 只用于本次请求, 不做任何持久化。
 * @returns {Promise<{text?:string, error?:string}>}
 */
export async function aiChat({ provider = 'deepseek', apiKey, system, user, model, baseUrl, timeoutMs = 120000 } = {}) {
  const info = providerInfo(provider);
  const base = (baseUrl && String(baseUrl).trim()) ? String(baseUrl).trim() : info.baseUrl;
  const mdl = model || (info.models && info.models[0]) || '';
  const key = (apiKey || '').trim();
  if (!mdl) return { error: '模型名未填写' };
  if (!key && !info.keyOptional) return { error: info.label + ' API Key 未填写' };
  if (info.type === 'anthropic') return anthropicChat(key, system, user, mdl, base, timeoutMs);
  if (info.type === 'gemini') return geminiChat(key, system, user, mdl, base, timeoutMs);
  return openaiChat(key, system, user, mdl, base, timeoutMs);
}
