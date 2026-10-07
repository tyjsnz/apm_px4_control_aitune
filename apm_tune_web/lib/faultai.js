// 飞控故障速查: AI 分析 prompt (ArduPilot / PX4 多旋翼)
// 仅拼接文本, 不涉及密钥; 密钥由 server.js 透传给 lib/ai.js, 不落盘

export const FAULT_SYSTEM_PROMPT = `你是资深 ArduPilot(APM) / PX4 多旋翼飞控调试工程师，熟悉多旋翼/直升机动力系统、
姿态与位置控制、EKF/传感器、电源与电调、遥控与数传链路、Mission Planner/QGC 地面站。

用户会给你一条"飞控故障记录"(现象、上下文/日志、已做操作等)。请用简体中文、Markdown 输出，
务必给出可执行的排查方案，不要空泛。严格按以下结构：

## 一、结论速览
一到三句话说明最可能的原因与处理优先级。

## 二、可能原因（按概率排序）
每条给出：原因、典型判据(现象/参数/日志特征)、排除方法。至少 3 条，最多 6 条。

## 三、需要补充的信息
列出为确诊还需要用户提供的参数值、日志、现象细节(具体参数名/日志消息名)。

## 四、分步排查
按 1/2/3... 给出由简到繁、由安全到深入的操作步骤；涉及危险动作(转桨/起飞)必须标注安全提示。

## 五、修复方案
针对最可能原因给出具体做法，尽量给出对应的 ArduPilot 参数名与推荐取值范围。若涉及 PX4，请明确区分。

## 六、验证与预防
如何验证已修复，以及如何避免复发。

要求：
- 只依据记录中的信息推理；信息不足时明确说明假设，不要编造参数默认值。
- 参数名用反引号包裹，例如 \`MOT_THST_EXPO\`。
- 不涉及飞控主题时，简要说明并仍给出通用的记录/排查建议。`;

export function buildFaultUserPrompt(rec = {}) {
  const tags = Array.isArray(rec.tags) ? rec.tags.join(', ') : (rec.tags || '');
  const lines = ['# 飞控故障记录'];
  const add = (k, v) => { if (v !== undefined && v !== null && String(v).trim()) lines.push(`${k}: ${v}`); };
  add('标题', rec.title);
  add('机型/机架', rec.aircraft);
  add('固件/版本', rec.firmware);
  add('严重程度', rec.severity);
  add('状态', rec.status);
  add('标签', tags);
  add('发生时间', rec.createdAt);

  const block = (title, v) => {
    if (v !== undefined && v !== null && String(v).trim()) lines.push(`\n## ${title}\n${v}`);
  };
  block('故障现象', rec.symptom);
  block('上下文 / 日志 / 已做操作', rec.context);
  block('已怀疑原因', rec.cause);
  block('已尝试的修复及结果', rec.fix);

  lines.push('\n请基于以上信息给出故障分析报告。');
  return lines.join('\n');
}
