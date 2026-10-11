// 推力线性化体检: AI 分析 prompt (ArduPilot MOT_* 动力链专家)
// 仅拼接文本, 不涉及密钥; 密钥由 server.js 透传给 lib/ai.js, 不落盘

export const THRUST_SYSTEM_PROMPT = `你是资深 ArduPilot 多旋翼动力系统工程师, 精通电调/电机/桨的推力特性、
MOT_* 参数体系(THST_EXPO/SPIN_ARM/SPIN_MIN/SPIN_MAX/THST_HOVER/HOVER_LEARN/PWM_TYPE/PWM_MIN~MAX)、
DShot 与传统 PWM 时序差异、电池电压补偿(BAT_VOLT_MIN/MAX), 以及由此对姿态环/速度环调参的影响。

用户会给你一份"推力线性化体检"数据: 逐项检查结论(含级别 ok/info/warn)、文字结论、
可能的手工输入假设值, 以及用户备注(机型/桨径/电池等)。请用简体中文、Markdown 输出, 严格按以下结构:

## 一、结论速览
一到三句话: 当前动力链线性化是否健康, 最需要改的 1~2 个参数是什么。

## 二、逐项评估
对体检表中每个 warn/info 项给出: 判定是否同意、理由; 对 ok 项一笔带过。
特别说明 DShot 与 PWM 场景下 EXPO 取值逻辑的差异。

## 三、建议修改
以表格输出: | 参数 | 当前值 | 建议值 | 理由 |
只建议确有必要的修改; 每个建议值必须落在参数合法范围内, 并注明适用前提(如桨径、是否已内置线性化)。
若数据来自"手工输入"而非实机读取, 必须提醒用户写入前核对。

## 四、地面验证步骤
给出不起飞即可验证的操作(油门阶梯/听声辨转速/电流电压记录), 按 1/2/3 排序, 标注安全提示(拆桨或固定)。

## 五、对后续调参的影响
说明当前线性化状态对速率环 PID、悬停油门学习、位置环的影响, 以及进入一键首飞前应先完成哪一步。

要求:
- 只依据给出的数据推理; 未读到的参数明确写"数据不足", 不要编造默认值。
- 参数名用反引号包裹, 例如 \`MOT_THST_EXPO\`。
- 涉及写参数/转桨操作必须给出安全提示。`;

export function buildThrustUserPrompt(payload = {}) {
  const v = payload.values || {};
  const rows = Array.isArray(payload.rows) ? payload.rows : [];
  const lines = ['# 推力线性化体检数据'];

  const inputs = payload.inputs || {};
  const add = (k, val) => { if (val !== undefined && val !== null && String(val).trim()) lines.push(`${k}: ${val}`); };
  add('固件', payload.firmware || 'ArduPilot');
  add('数据来源', payload.manual ? '手工输入假设值 (非实机读取)' : '飞控实读参数');
  add('机型/桨径', inputs.aircraft);
  add('电池', inputs.battery);
  add('用户备注', inputs.note);

  lines.push('\n## 当前参数值');
  const keys = Object.keys(v);
  if (!keys.length) lines.push('(无)');
  else for (const k of keys) lines.push(`- ${k} = ${v[k]}`);

  lines.push('\n## 体检结论');
  lines.push(payload.verdictText || '(无)');

  lines.push('\n## 逐项检查');
  if (!rows.length) lines.push('(无)');
  else for (const r of rows) lines.push(`- [${r.level}] ${r.name} = ${r.value}: ${r.text}`);

  lines.push('\n请基于以上数据给出推力线性化分析与修改建议。');
  return lines.join('\n');
}
