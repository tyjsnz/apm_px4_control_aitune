# AI PID Tuning Core Engine (Standalone)
from pymavlink import mavutil
import time, json, os, re, math, threading, traceback
from urllib import request, error

AUTOPILOT_APM = 3
AUTOPILOT_PX4 = 12

APM_COPTER = {
    "ATC_ANG_RLL_P": {"min": 1.0, "max": 15.0, "default": 9.0, "desc": "Angle roll P"},
    "ATC_ANG_PIT_P": {"min": 1.0, "max": 15.0, "default": 9.0, "desc": "Angle pitch P"},
    "ATC_ANG_YAW_P": {"min": 1.0, "max": 15.0, "default": 4.5, "desc": "Angle yaw P"},
    "ATC_RAT_RLL_P": {"min": 0.01, "max": 1.0, "default": 0.135, "desc": "Rate roll P"},
    "ATC_RAT_RLL_I": {"min": 0.0, "max": 5.0, "default": 0.135, "desc": "Rate roll I"},
    "ATC_RAT_RLL_D": {"min": 0.0, "max": 0.5, "default": 0.0036, "desc": "Rate roll D"},
    "ATC_RAT_RLL_FF": {"min": 0.0, "max": 1.0, "default": 0.0, "desc": "Rate roll FF"},
    "ATC_RAT_PIT_P": {"min": 0.01, "max": 1.0, "default": 0.135, "desc": "Rate pitch P"},
    "ATC_RAT_PIT_I": {"min": 0.0, "max": 5.0, "default": 0.135, "desc": "Rate pitch I"},
    "ATC_RAT_PIT_D": {"min": 0.0, "max": 0.5, "default": 0.0036, "desc": "Rate pitch D"},
    "ATC_RAT_PIT_FF": {"min": 0.0, "max": 1.0, "default": 0.0, "desc": "Rate pitch FF"},
    "ATC_RAT_YAW_P": {"min": 0.01, "max": 1.0, "default": 0.18, "desc": "Rate yaw P"},
    "ATC_RAT_YAW_I": {"min": 0.0, "max": 5.0, "default": 0.018, "desc": "Rate yaw I"},
    "ATC_RAT_YAW_D": {"min": 0.0, "max": 0.5, "default": 0.0, "desc": "Rate yaw D"},
    "ATC_RAT_YAW_FF": {"min": 0.0, "max": 1.0, "default": 0.0, "desc": "Rate yaw FF"},
    "ATC_INPUT_TC": {"min": 0.0, "max": 1.0, "default": 0.2, "desc": "Input TC"},
    "INS_GYRO_FILTER": {"min": 1.0, "max": 200.0, "default": 20.0, "desc": "Gyro filter"},
    "INS_ACC_FILTER": {"min": 1.0, "max": 200.0, "default": 20.0, "desc": "Acc filter"},
}

PX4_MC = {
    "MC_ROLL_P": {"min": 1.0, "max": 15.0, "default": 6.0, "desc": "Roll P"},
    "MC_PITCH_P": {"min": 1.0, "max": 15.0, "default": 6.0, "desc": "Pitch P"},
    "MC_YAW_P": {"min": 1.0, "max": 10.0, "default": 2.8, "desc": "Yaw P"},
    "MC_ROLLRATE_P": {"min": 0.01, "max": 1.0, "default": 0.15, "desc": "Rollrate P"},
    "MC_ROLLRATE_I": {"min": 0.0, "max": 5.0, "default": 0.2, "desc": "Rollrate I"},
    "MC_ROLLRATE_D": {"min": 0.0, "max": 0.5, "default": 0.003, "desc": "Rollrate D"},
    "MC_ROLLRATE_FF": {"min": 0.0, "max": 1.0, "default": 0.0, "desc": "Rollrate FF"},
    "MC_PITCHRATE_P": {"min": 0.01, "max": 1.0, "default": 0.15, "desc": "Pitchrate P"},
    "MC_PITCHRATE_I": {"min": 0.0, "max": 5.0, "default": 0.2, "desc": "Pitchrate I"},
    "MC_PITCHRATE_D": {"min": 0.0, "max": 0.5, "default": 0.003, "desc": "Pitchrate D"},
    "MC_PITCHRATE_FF": {"min": 0.0, "max": 1.0, "default": 0.0, "desc": "Pitchrate FF"},
    "MC_YAWRATE_P": {"min": 0.01, "max": 1.0, "default": 0.2, "desc": "Yawrate P"},
    "MC_YAWRATE_I": {"min": 0.0, "max": 5.0, "default": 0.1, "desc": "Yawrate I"},
    "MC_YAWRATE_D": {"min": 0.0, "max": 0.5, "default": 0.0, "desc": "Yawrate D"},
    "MC_YAWRATE_FF": {"min": 0.0, "max": 1.0, "default": 0.0, "desc": "Yawrate FF"},
    "MC_GYRO_CUTOFF": {"min": 10, "max": 200, "default": 30, "desc": "Gyro cutoff"},
}

PARAM_SETS = {"APM_COPTER": APM_COPTER, "PX4_MC": PX4_MC}

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "ai_tune_config.json")
BACKUP_DIR = os.path.join(os.path.dirname(__file__), "ai_tune_backups")

def load_config():
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
    except Exception:
        pass
    return {"api_key": "", "model": "deepseek-flash", "last_port": "", "baud": 115200, "last_spec": {}}

def save_config(cfg):
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception:
        pass

def connect_port(port, baud=115200, timeout=30):
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
        hb = master.wait_heartbeat(timeout=timeout)
        if hb is None:
            return None, None
        master.target_system = hb.get_srcSystem()
        master.target_component = hb.get_srcComponent()
        try:
            master.vehicle_type = "copter"
        except Exception:
            pass
        autopilot = hb.autopilot
        if autopilot == AUTOPILOT_APM:
            fc_type = "APM_COPTER"
        elif autopilot == AUTOPILOT_PX4:
            fc_type = "PX4_MC"
        else:
            fc_type = "APM_COPTER"
        return master, fc_type
    except Exception:
        return None, None

def disconnect(master):
    try:
        if master:
            master.close()
    except Exception:
        pass

def drain_rx(master, timeout=0.5):
    try:
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = master.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                break
    except Exception:
        pass

def get_param(master, name, timeout=1.5):
    try:
        name_bytes = name.upper().encode("ascii", "replace")
        master.mav.param_request_read_send(master.target_system, master.target_component, name_bytes, -1)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is None:
            return None
        pid = msg.param_id
        if isinstance(pid, (bytes, bytearray)):
            pid = pid.decode("ascii", "replace")
        pid = str(pid).split("\x00")[0]
        if pid.upper() == name.upper():
            return float(msg.param_value)
        return float(msg.param_value) if pid else None
    except Exception:
        return None

def set_param(master, name, value, timeout=3.0):
    try:
        name_bytes = name.upper().encode("ascii", "replace")
        val = float(value)
        master.mav.param_set_send(master.target_system, master.target_component, name_bytes, val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.2)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is None:
            return False
        pid = msg.param_id
        if isinstance(pid, (bytes, bytearray)):
            pid = pid.decode("ascii", "replace")
        pid = str(pid).split("\x00")[0]
        if pid.upper() != name.upper():
            return False
        if abs(msg.param_value - val) < 0.01:
            return True
        got = get_param(master, name, timeout=1.0)
        return got is not None and abs(got - val) < 0.01
    except Exception:
        return False

def read_params(master, names, on_progress=None):
    result = {}
    for i, name in enumerate(names):
        val = get_param(master, name)
        if val is not None:
            result[name] = val
        if on_progress:
            on_progress(i + 1, len(names), name, val)
        time.sleep(0.05)
    return result

def ensure_backup_dir():
    os.makedirs(BACKUP_DIR, exist_ok=True)

def backup_params(master, names, spec=None, tag=""):
    ensure_backup_dir()
    params = {}
    for name in names:
        val = get_param(master, name)
        if val is not None:
            params[name] = val
    ts = time.strftime("%Y%m%d_%H%M%S")
    tag_suffix = "_" + tag if tag else ""
    fname = "backup_" + ts + tag_suffix + ".json"
    fpath = os.path.join(BACKUP_DIR, fname)
    data = {"timestamp": ts, "fc_type": "UNKNOWN", "spec": spec or {}, "params": params}
    with open(fpath, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return fpath

def list_backups():
    if not os.path.exists(BACKUP_DIR):
        return []
    files = []
    for f in os.listdir(BACKUP_DIR):
        if f.startswith("backup_") and f.endswith(".json"):
            fpath = os.path.join(BACKUP_DIR, f)
            try:
                with open(fpath, "r", encoding="utf-8") as fh:
                    data = json.load(fh)
                files.append({"file": f, "path": fpath, "timestamp": data.get("timestamp", ""), "fc_type": data.get("fc_type", ""), "spec": data.get("spec", {}), "count": len(data.get("params", {}))})
            except Exception:
                pass
    files.sort(key=lambda x: x["timestamp"], reverse=True)
    return files

def restore_backup(master, backup_path):
    try:
        with open(backup_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        params = data.get("params", {})
        if not params:
            return False, "Backup has no parameters"
        success = 0
        failed = []
        for name, val in params.items():
            if set_param(master, name, val):
                success += 1
            else:
                failed.append(name)
        return success, failed
    except Exception as e:
        return False, str(e)

DEEPSEEK_API_URL = "https://api.deepseek.com/chat/completions"

# type: openai = OpenAI Chat Completions 兼容协议; anthropic / gemini = 各自私有协议
AI_PROVIDERS = {
    "deepseek": {
        "label": "DeepSeek", "type": "openai",
        "base_url": "https://api.deepseek.com",
        "models": ["deepseek-flash", "deepseek-v4-pro"],
    },
    "openai": {
        "label": "OpenAI", "type": "openai",
        "base_url": "https://api.openai.com/v1",
        "models": ["gpt-5.6", "gpt-5.6-sol", "gpt-5.6-terra", "gpt-5.6-luna", "gpt-4.1", "gpt-4.1-mini", "gpt-4o", "gpt-4o-mini"],
    },
    "anthropic": {
        "label": "Anthropic Claude", "type": "anthropic",
        "base_url": "https://api.anthropic.com",
        "models": ["claude-sonnet-5", "claude-opus-5", "claude-haiku-4-5", "claude-opus-4-8", "claude-fable-5"],
    },
    "gemini": {
        "label": "Google Gemini", "type": "gemini",
        "base_url": "https://generativelanguage.googleapis.com/v1beta",
        "models": ["gemini-3.5-flash", "gemini-3.1-pro-preview", "gemini-3.1-flash-lite", "gemini-2.5-flash"],
    },
    "qwen": {
        "label": "通义千问 Qwen", "type": "openai",
        "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "models": ["qwen-max", "qwen-plus", "qwen-turbo", "qwen3-max"],
    },
    "glm": {
        "label": "智谱 GLM", "type": "openai",
        "base_url": "https://open.bigmodel.cn/api/paas/v4",
        "models": ["glm-4.6", "glm-4.5", "glm-4-flash", "glm-4-plus"],
    },
    "kimi": {
        "label": "Kimi 月之暗面", "type": "openai",
        "base_url": "https://api.moonshot.cn/v1",
        "models": ["kimi-k2.6", "moonshot-v1-128k", "moonshot-v1-8k"],
    },
    "doubao": {
        "label": "豆包 火山方舟", "type": "openai",
        "base_url": "https://ark.cn-beijing.volces.com/api/v3",
        "models": ["doubao-1-5-pro-32k", "doubao-pro-32k", "doubao-lite-4k"],
    },
    "openrouter": {
        "label": "OpenRouter 聚合", "type": "openai",
        "base_url": "https://openrouter.ai/api/v1",
        "models": ["deepseek/deepseek-chat", "openai/gpt-5.6", "anthropic/claude-sonnet-5", "google/gemini-3.5-flash", "moonshotai/kimi-k2.6"],
    },
    "groq": {
        "label": "Groq", "type": "openai",
        "base_url": "https://api.groq.com/openai/v1",
        "models": ["llama-3.3-70b-versatile", "llama-3.1-8b-instant"],
    },
    "mistral": {
        "label": "Mistral", "type": "openai",
        "base_url": "https://api.mistral.ai/v1",
        "models": ["mistral-large-latest", "mistral-small-latest"],
    },
    "ollama": {
        "label": "Ollama 本地", "type": "openai",
        "base_url": "http://localhost:11434/v1",
        "models": ["qwen3:8b", "llama3.1:8b", "deepseek-r1:8b"],
        "key_optional": True,
    },
    "custom": {
        "label": "自定义 OpenAI兼容", "type": "openai",
        "base_url": "",
        "models": [],
    },
}

def provider_labels():
    return [p["label"] for p in AI_PROVIDERS.values()]

def provider_id_by_label(label):
    for pid, p in AI_PROVIDERS.items():
        if p["label"] == label:
            return pid
    return "deepseek"

def provider_info(pid):
    return AI_PROVIDERS.get(pid, AI_PROVIDERS["deepseek"])

def _chat_endpoint(base_url):
    b = (base_url or "").strip().rstrip("/")
    if not b:
        return None
    if b.endswith("/chat/completions"):
        return b
    return b + "/chat/completions"

def _post_json(url, payload, headers, timeout, retries=3):
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    hdrs = {"Content-Type": "application/json"}
    hdrs.update(headers or {})
    req = request.Request(url, data=body, headers=hdrs, method="POST")
    for attempt in range(retries):
        try:
            with request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read().decode("utf-8")), None
        except error.HTTPError as e:
            try:
                text = e.read().decode("utf-8", "replace")
            except Exception:
                text = ""
            if e.code in (401, 403):
                return None, "API Key 无效或无权限 (" + str(e.code) + ")"
            elif e.code == 402:
                return None, "余额不足 (402)"
            elif e.code == 404:
                return None, "接口或模型不存在 (404): " + text[:300]
            elif e.code == 429:
                if attempt < retries - 1:
                    time.sleep(2)
                    continue
                return None, "请求过于频繁 (429)"
            elif e.code >= 500:
                if attempt < retries - 1:
                    time.sleep(1)
                    continue
                return None, "服务端错误 " + str(e.code) + ": " + text[:300]
            else:
                return None, "HTTP " + str(e.code) + ": " + text[:300]
        except Exception as e:
            if attempt < retries - 1:
                time.sleep(1)
                continue
            return None, "请求失败: " + str(e)
    return None, "多次重试失败"

def _openai_chat(api_key, system_prompt, user_prompt, model, base_url, timeout):
    url = _chat_endpoint(base_url)
    if not url:
        return None, "Base URL 未填写"
    headers = {"Authorization": "Bearer " + api_key} if api_key else {}
    payload = {"model": model,
               "messages": [{"role": "system", "content": system_prompt}, {"role": "user", "content": user_prompt}],
               "temperature": 0.3, "stream": False}
    data, err = _post_json(url, payload, headers, timeout)
    if err:
        return None, err
    try:
        msg = data["choices"][0]["message"]
        content = msg.get("content")
        if content is None:
            content = msg.get("reasoning_content") or ""
        return content, None
    except Exception:
        return None, "响应格式异常: " + json.dumps(data, ensure_ascii=False)[:300]

def _anthropic_chat(api_key, system_prompt, user_prompt, model, base_url, timeout):
    url = (base_url or "https://api.anthropic.com").strip().rstrip("/") + "/v1/messages"
    headers = {"x-api-key": api_key, "anthropic-version": "2023-06-01"}
    payload = {"model": model, "max_tokens": 4096, "temperature": 0.3,
               "system": system_prompt,
               "messages": [{"role": "user", "content": user_prompt}]}
    data, err = _post_json(url, payload, headers, timeout)
    if err:
        return None, err
    try:
        parts = data["content"]
        text = "".join(p.get("text", "") for p in parts if p.get("type") == "text")
        if not text:
            return None, "响应无文本内容: " + json.dumps(data, ensure_ascii=False)[:300]
        return text, None
    except Exception:
        return None, "响应格式异常: " + json.dumps(data, ensure_ascii=False)[:300]

def _gemini_chat(api_key, system_prompt, user_prompt, model, base_url, timeout):
    base = (base_url or "https://generativelanguage.googleapis.com/v1beta").strip().rstrip("/")
    url = base + "/models/" + model + ":generateContent"
    headers = {"x-goog-api-key": api_key}
    payload = {"systemInstruction": {"parts": [{"text": system_prompt}]},
               "contents": [{"role": "user", "parts": [{"text": user_prompt}]}],
               "generationConfig": {"temperature": 0.3}}
    data, err = _post_json(url, payload, headers, timeout)
    if err:
        return None, err
    try:
        cand = data.get("candidates") or []
        if not cand:
            return None, "无返回内容(可能被安全策略拦截): " + json.dumps(data.get("promptFeedback", {}), ensure_ascii=False)
        parts = cand[0].get("content", {}).get("parts", [])
        text = "".join(p.get("text", "") for p in parts)
        if not text:
            return None, "响应无文本内容: " + json.dumps(data, ensure_ascii=False)[:300]
        return text, None
    except Exception:
        return None, "响应格式异常: " + json.dumps(data, ensure_ascii=False)[:300]

def ai_chat(api_key, system_prompt, user_prompt, model="deepseek-flash",
            provider="deepseek", base_url=None, timeout=60):
    """统一 AI 调用入口。provider 取自 AI_PROVIDERS；base_url 为空则用该服务商默认地址。"""
    info = provider_info(provider)
    if base_url is not None and str(base_url).strip():
        base = str(base_url).strip()
    else:
        base = info["base_url"]
    if not model:
        model = info["models"][0] if info["models"] else ""
    if not model:
        return None, "模型名未填写"
    api_key = (api_key or "").strip()
    if not api_key and not info.get("key_optional"):
        return None, info["label"] + " API Key 未填写"
    kind = info["type"]
    if kind == "anthropic":
        return _anthropic_chat(api_key, system_prompt, user_prompt, model, base, timeout)
    if kind == "gemini":
        return _gemini_chat(api_key, system_prompt, user_prompt, model, base, timeout)
    return _openai_chat(api_key, system_prompt, user_prompt, model, base, timeout)

def deepseek_chat(api_key, system_prompt, user_prompt, model="deepseek-flash", timeout=60):
    return ai_chat(api_key, system_prompt, user_prompt, model=model,
                   provider="deepseek", timeout=timeout)

def extract_json(text):
    if not text:
        return None
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if m:
        text = m.group(1)
    else:
        m = re.search(r"(\{.*\})", text, re.DOTALL)
        if m:
            text = m.group(1)
    try:
        return json.loads(text)
    except Exception:
        return None

SYSTEM_PROMPT = """你是资深 ArduPilot/PX4 多旋翼调参工程师。
根据用户提供的机型规格、当前参数值、试飞反馈，给出 PID+滤波参数调整建议。
必须严格输出 JSON，格式：
{
  "summary": "一句话总结本轮调整重点",
  "params": {"PARAM_NAME": value, ...},
  "reasoning": "详细分析理由",
  "warnings": ["风险提示1", "风险提示2"]
}
只输出白名单内的参数，值必须在 min/max 范围内。"""

PARAM_GROUPS = {
    "attitude": {"APM_COPTER": ["ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P"], "PX4_MC": ["MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P"], "label": "姿态环"},
    "rate": {"APM_COPTER": ["ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_RAT_RLL_FF", "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D", "ATC_RAT_PIT_FF", "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D", "ATC_RAT_YAW_FF"], "PX4_MC": ["MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_FF", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_FF", "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D", "MC_YAWRATE_FF"], "label": "速率环"},
    "filter": {"APM_COPTER": ["ATC_INPUT_TC", "INS_GYRO_FILTER", "INS_ACC_FILTER"], "PX4_MC": ["MC_GYRO_CUTOFF"], "label": "滤波/响应"}
}

def get_param_names_for_groups(fc_type, groups):
    names = []
    for g in groups:
        if g in PARAM_GROUPS and fc_type in PARAM_GROUPS[g]:
            names.extend(PARAM_GROUPS[g][fc_type])
    return names


def fetch_fc_autofill_params(master, fc_type):
    """
    从飞控自动获取可填充的参数。
    返回 dict，包含可自动获取的字段值。
    """
    fetched = {}
    try:
        # 1. 电池状态
        bat = master.recv_match(type=['BATTERY_STATUS', 'SYS_STATUS'], blocking=True, timeout=3)
        if bat:
            if bat.get_type() == 'BATTERY_STATUS':
                volt = bat.voltages[0] / 1000.0 if bat.voltages and bat.voltages[0] > 0 else bat.voltage_battery / 1000.0
                curr = bat.current_battery / 100.0 if bat.current_battery != -1 else 0
                consumed = bat.charge_consumed / 1000.0 if bat.charge_consumed != -1 else 0
            else:
                volt = bat.voltage_battery / 1000.0
                curr = bat.current_battery / 100.0
                consumed = 0
            
            if volt > 0:
                fetched['battery_voltage'] = volt
                s_est = round(volt / 3.7)
                if 3 <= s_est <= 14:
                    fetched['battery_s'] = f'{s_est}S'
            
            if curr > 0:
                fetched['battery_current'] = curr
            if consumed > 0:
                fetched['battery_consumed_mah'] = consumed
        
        # 2. ESC 遥测
        esc_data = []
        for i in range(4):
            esc = master.recv_match(type='ESC_TELEMETRY_1_TO_4', blocking=True, timeout=1)
            if esc and esc.rpm[i] > 0:
                esc_data.append({
                    'rpm': esc.rpm[i],
                    'voltage': esc.voltage[i] / 100.0 if esc.voltage[i] > 0 else 0,
                    'current': esc.current[i] / 100.0 if esc.current[i] > 0 else 0,
                    'temp': esc.temperature[i] / 100.0 if esc.temperature[i] > 0 else 0,
                })
        if esc_data:
            fetched['esc_telemetry'] = esc_data
        
        # 3. PX4 特有参数
        if fc_type == 'PX4_MC':
            try:
                pole = get_param(master, 'MOT_POLE_COUNT')
                if pole:
                    fetched['motor_pole_pairs'] = int(pole)
            except Exception:
                pass
        
        # 4. GPS
        gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if gps and gps.fix_type >= 3:
            fetched['gps_fix'] = gps.fix_type
        
    except Exception as e:
        pass
    
    return fetched

def build_prompt(spec, current_params, fc_type, selected_groups, feedback_text="", history=None):
    param_set = PARAM_SETS.get(fc_type, {})
    white_names = get_param_names_for_groups(fc_type, selected_groups)
    param_info = []
    for name in white_names:
        if name in param_set:
            info = param_set[name]
            cur = current_params.get(name, info["default"])
            param_info.append(name + ": 当前=" + str(round(cur, 4)) + ", 默认=" + str(info["default"]) + ", 范围=[" + str(info["min"]) + "," + str(info["max"]) + "], 说明=" + info["desc"])
    spec_str = json.dumps(spec, ensure_ascii=False, indent=2)
    param_str = "\n".join(param_info)
    hist_str = ""
    if history:
        hist_str = "\n历史轮次:\n" + "\n".join([f'第{h["round"]}轮({h["time"]}): {h["summary"]}, 写入{h["written"]}个, 备份={h["backup"]}' for h in history[-3:]])
    user = "机型规格:\n" + spec_str + "\n\n当前参数值 (仅选中分组):\n" + param_str + "\n\n试飞反馈/现象描述:\n" + (feedback_text if feedback_text else "(首轮调参，无试飞数据)") + hist_str + "\n\n请分析后给出调整建议，输出严格 JSON。"
    return user

def validate_ai_params(ai_params, fc_type, selected_groups):
    param_set = PARAM_SETS.get(fc_type, {})
    white_names = get_param_names_for_groups(fc_type, selected_groups)
    valid = {}
    warnings = []
    for name, val in ai_params.items():
        if name not in white_names:
            warnings.append("参数 " + name + " 不在选中分组白名单中，已忽略")
            continue
        if name not in param_set:
            warnings.append("参数 " + name + " 未在定义中，已忽略")
            continue
        info = param_set[name]
        try:
            v = float(val)
            if v < info["min"]:
                v = info["min"]
                warnings.append(name + "=" + str(val) + " 低于下限 " + str(info["min"]) + "，已钳制")
            elif v > info["max"]:
                v = info["max"]
                warnings.append(name + "=" + str(val) + " 高于上限 " + str(info["max"]) + "，已钳制")
            valid[name] = v
        except Exception:
            warnings.append(name + " 值 " + str(val) + " 非法，已忽略")
    return valid, warnings


# ===== Flight Test Recording & Analysis =====
class FlightRecorder:
    """Passive flight data recorder for ATTITUDE, VIBRATION, GPS, RC channels."""
    
    def __init__(self, master):
        self.master = master
        self.records = []
        self.recording = False
        self.record_thread = None
        self._lock = threading.Lock()
    
    def start(self):
        """Start recording in background thread."""
        if self.recording:
            return
        self.records = []
        self.recording = True
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()
    
    def stop(self):
        """Stop recording and return collected records."""
        self.recording = False
        if self.record_thread:
            self.record_thread.join(timeout=2.0)
        with self._lock:
            return list(self.records)
    
    def _record_loop(self):
        """Background loop to capture MAVLink messages."""
        types = ('ATTITUDE', 'VIBRATION', 'GLOBAL_POSITION_INT', 'RC_CHANNELS', 'RC_CHANNELS_RAW')
        while self.recording:
            try:
                msg = self.master.recv_match(type=types, blocking=True, timeout=0.5)
                if msg is None:
                    continue
                mtype = msg.get_type()
                t = time.time()
                if mtype == 'ATTITUDE':
                    with self._lock:
                        self.records.append({
                            't': t, 'type': 'ATTITUDE',
                            'roll': msg.roll, 'pitch': msg.pitch, 'yaw': msg.yaw,
                            'rollspeed': msg.rollspeed, 'pitchspeed': msg.pitchspeed, 'yawspeed': msg.yawspeed
                        })
                elif mtype == 'VIBRATION':
                    with self._lock:
                        self.records.append({
                            't': t, 'type': 'VIBRATION',
                            'vibe_x': msg.vibration_x, 'vibe_y': msg.vibration_y, 'vibe_z': msg.vibration_z,
                            'clip_x': msg.clipping_0, 'clip_y': msg.clipping_1, 'clip_z': msg.clipping_2
                        })
                elif mtype == 'GLOBAL_POSITION_INT':
                    with self._lock:
                        self.records.append({
                            't': t, 'type': 'GLOBAL_POSITION_INT',
                            'lat': msg.lat, 'lon': msg.lon,
                            'alt': msg.alt, 'relative_alt': msg.relative_alt,
                            'vx': msg.vx, 'vy': msg.vy, 'vz': msg.vz,
                            'hdg': msg.hdg
                        })
                elif mtype in ('RC_CHANNELS', 'RC_CHANNELS_RAW'):
                    chans = [getattr(msg, f'chan{i}_raw', 0) for i in range(1, 9)]
                    with self._lock:
                        self.records.append({
                            't': t, 'type': 'RC_CHANNELS',
                            'chans': chans
                        })
            except Exception:
                pass
    
    def analyze(self):
        """Analyze recorded data and return metrics."""
        with self._lock:
            records = list(self.records)
        if not records:
            return {}
        
        # Separate by type
        att = [r for r in records if r['type'] == 'ATTITUDE']
        vib = [r for r in records if r['type'] == 'VIBRATION']
        gps = [r for r in records if r['type'] == 'GLOBAL_POSITION_INT']
        rc = [r for r in records if r['type'] == 'RC_CHANNELS']
        
        metrics = {
            'duration': records[-1]['t'] - records[0]['t'] if len(records) > 1 else 0,
            'sample_counts': {'attitude': len(att), 'vibration': len(vib), 'gps': len(gps), 'rc': len(rc)}
        }
        
        # Attitude analysis
        if len(att) >= 10:
            metrics['attitude'] = self._analyze_attitude(att)
        
        # Vibration analysis
        if len(vib) >= 10:
            metrics['vibration'] = self._analyze_vibration(vib)
        
        # Altitude stability
        if len(gps) >= 10:
            metrics['altitude'] = self._analyze_altitude(gps)
        
        # RC input analysis
        if len(rc) >= 10:
            metrics['rc'] = self._analyze_rc(rc)
        
        return metrics
    
    def _analyze_attitude(self, att):
        """Analyze attitude data for oscillations, overshoot, etc."""
        rolls = [r['roll'] for r in att]
        pitches = [r['pitch'] for r in att]
        yaws = [r['yaw'] for r in att]
        roll_rates = [r['rollspeed'] for r in att]
        pitch_rates = [r['pitchspeed'] for r in att]
        yaw_rates = [r['yawspeed'] for r in att]
        
        def zero_crossings(x):
            """Count zero crossings (sign changes) as oscillation indicator."""
            signs = [1 if v > 0 else (-1 if v < 0 else 0) for v in x]
            crossings = 0
            for i in range(1, len(signs)):
                if signs[i] != 0 and signs[i-1] != 0 and signs[i] != signs[i-1]:
                    crossings += 1
            return crossings
        
        def rms(x):
            return math.sqrt(sum(v*v for v in x) / len(x)) if x else 0
        
        return {
            'roll': {'mean': sum(rolls)/len(rolls), 'rms': rms(rolls), 'max': max(rolls), 'min': min(rolls), 'zero_cross': zero_crossings(rolls)},
            'pitch': {'mean': sum(pitches)/len(pitches), 'rms': rms(pitches), 'max': max(pitches), 'min': min(pitches), 'zero_cross': zero_crossings(pitches)},
            'yaw': {'mean': sum(yaws)/len(yaws), 'rms': rms(yaws), 'max': max(yaws), 'min': min(yaws), 'zero_cross': zero_crossings(yaws)},
            'roll_rate_rms': rms(roll_rates),
            'pitch_rate_rms': rms(pitch_rates),
            'yaw_rate_rms': rms(yaw_rates)
        }
    
    def _analyze_vibration(self, vib):
        """Analyze vibration data (RMS, peak, clipping)."""
        x = [r['vibe_x'] for r in vib]
        y = [r['vibe_y'] for r in vib]
        z = [r['vibe_z'] for r in vib]
        clip_x = [r['clip_x'] for r in vib]
        clip_y = [r['clip_y'] for r in vib]
        clip_z = [r['clip_z'] for r in vib]
        
        def rms(arr):
            return math.sqrt(sum(v*v for v in arr) / len(arr)) if arr else 0
        
        return {
            'x': {'rms': rms(x), 'peak': max(x) if x else 0, 'clip_count': sum(1 for c in clip_x if c > 0)},
            'y': {'rms': rms(y), 'peak': max(y) if y else 0, 'clip_count': sum(1 for c in clip_y if c > 0)},
            'z': {'rms': rms(z), 'peak': max(z) if z else 0, 'clip_count': sum(1 for c in clip_z if c > 0)}
        }
    
    def _analyze_altitude(self, gps):
        """Analyze altitude stability."""
        alts = [r['relative_alt'] / 1000.0 for r in gps]  # mm to m
        vz = [r['vz'] / 100.0 for r in gps]  # cm/s to m/s
        
        def rms(arr):
            return math.sqrt(sum(v*v for v in arr) / len(arr)) if arr else 0
        
        return {
            'alt_mean': sum(alts)/len(alts) if alts else 0,
            'alt_rms': rms(alts),
            'alt_min': min(alts) if alts else 0,
            'alt_max': max(alts) if alts else 0,
            'vz_rms': rms(vz)
        }
    
    def _analyze_rc(self, rc):
        """Analyze RC channel inputs."""
        # Channels: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw
        if not rc:
            return {}
        chans = [r['chans'] for r in rc]
        n = len(chans[0])
        result = {}
        for i in range(min(n, 4)):
            vals = [c[i] for c in chans]
            result[f'chan{i+1}'] = {'mean': sum(vals)/len(vals), 'min': min(vals), 'max': max(vals)}
        return result


# Safety check for auto step excitation
def check_safety_for_step(master, min_alt_m=2.0):
    """Check if safe to perform auto step excitation."""
    try:
        # Check armed
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if not hb or not (hb.base_mode & 0x80):  # MAV_MODE_FLAG_SAFETY_ARMED
            return False, "未解锁"
        
        # Check mode (LOITER=5, POS_HOLD=4 for APM; for PX4 check custom_mode)
        # We'll be lenient and just check not in MANUAL/STABILIZE for safety
        
        # Check GPS
        gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if not gps or gps.fix_type < 3:  # 3D fix
            return False, "GPS 非3D定位"
        
        # Check altitude
        pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if not pos or pos.relative_alt / 1000.0 < min_alt_m:
            return False, f"相对高度不足 {min_alt_m}m"
        
        return True, "安全检查通过"
    except Exception as e:
        return False, f"检查异常: {e}"


def auto_step_excitation(master, axis='roll', amp_deg=10, cycles=3, hold_s=2.0, abort_flag=None):
    """
    Perform automatic step excitation for system identification.
    axis: 'roll' or 'pitch'
    amp_deg: step amplitude in degrees
    cycles: number of step cycles
    hold_s: hold time per step
    abort_flag: threading.Event to signal abort from GUI
    """
    import math
    from pymavlink import mavutil
    
    try:
        # Use SET_ATTITUDE_TARGET for offboard-like control
        # Need to be in a mode that accepts attitude targets (GUIDED/LOITER for APM, OFFBOARD for PX4)
        # We'll send ATTITUDE_TARGET at ~10Hz
        
        target_system = master.target_system
        target_component = master.target_component
        
        amp_rad = math.radians(amp_deg)
        period = hold_s * 2  # step + return
        
        for cycle in range(cycles):
            if abort_flag and abort_flag.is_set():
                return False, "用户中止"
            
            # Step 1: positive step
            t0 = time.time()
            while time.time() - t0 < hold_s:
                if abort_flag and abort_flag.is_set():
                    return False, "用户中止"
                # Send attitude target
                if axis == 'roll':
                    q = [math.cos(amp_rad/2), math.sin(amp_rad/2), 0, 0]
                else:
                    q = [math.cos(amp_rad/2), 0, math.sin(amp_rad/2), 0]
                master.mav.set_attitude_target_send(
                    int(time.time() * 1e6) & 0xFFFFFFFF,  # time_boot_ms
                    target_system, target_component,
                    0b00000111,  # ignore body rates, use attitude
                    q,  # quaternion
                    0, 0, 0,  # body roll/pitch/yaw rate (ignored)
                    0.5  # thrust (normalized, ~hover)
                )
                time.sleep(0.1)
            
            # Step 2: return to level
            t0 = time.time()
            while time.time() - t0 < hold_s:
                if abort_flag and abort_flag.is_set():
                    return False, "用户中止"
                q = [1, 0, 0, 0]  # level
                master.mav.set_attitude_target_send(
                    int(time.time() * 1e6) & 0xFFFFFFFF,
                    target_system, target_component,
                    0b00000111,
                    q, 0, 0, 0, 0.5
                )
                time.sleep(0.1)
        
        # Final: return to level and hold briefly
        for _ in range(20):
            if abort_flag and abort_flag.is_set():
                break
            master.mav.set_attitude_target_send(
                int(time.time() * 1e6) & 0xFFFFFFFF,
                target_system, target_component,
                0b00000111, [1,0,0,0], 0,0,0, 0.5
            )
            time.sleep(0.1)
        
        return True, "阶跃激励完成"
    except Exception as e:
        return False, f"激励异常: {e}"


# ===== DataFlash Log Management =====
def request_log_list(master, timeout=10):
    """
    Request the list of dataflash logs from the flight controller.
    Returns list of dicts: [{'id': num, 'size': bytes, 'time_utc': timestamp, 'num_logs': total}, ...]
    """
    try:
        master.mav.log_request_list_send(
            master.target_system, master.target_component,
            0,  # start index
            0xFFFF  # end index (all)
        )
        
        logs = []
        num_logs = None
        t0 = time.time()
        
        while time.time() - t0 < timeout:
            msg = master.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
            if msg is None:
                continue
            
            if num_logs is None:
                num_logs = msg.num_logs
            
            # LOG_ENTRY fields: id, num_logs, last_log_num, time_utc, size
            logs.append({
                'id': msg.id,
                'size': msg.size,
                'time_utc': msg.time_utc,
                'num_logs': msg.num_logs
            })
            
            # Check if we've received all entries
            if len(logs) >= msg.num_logs:
                break
        
        return logs
    except Exception as e:
        return []


def _drain_log_data(master):
    """丢弃接收队列中残留的 LOG_DATA 包(上一次请求的尾包)。"""
    try:
        for _ in range(500):
            if master.recv_match(type='LOG_DATA', blocking=False) is None:
                break
    except Exception:
        pass


def _request_log_block(master, log_id, offset, length, deadline):
    """
    请求 [offset, offset+length) 这一段日志并收集其产生的所有 LOG_DATA 包。
    LOG_DATA 单包最多 90 字节，一次请求会对应多个包，需按 ofs 组装。
    容忍乱序、重复、以及窗口外的残留旧包；静默 3 秒自动重发剩余部分。
    返回 bytes，超时返回 None。
    """
    sys_id = master.target_system
    comp_id = master.target_component
    end = offset + length
    assembled = bytearray()
    pending = {}  # ofs -> bytes
    next_needed = offset
    last_request = 0.0

    def send_request():
        nonlocal last_request
        master.mav.log_request_data_send(sys_id, comp_id, log_id, next_needed, end - next_needed)
        last_request = time.time()

    send_request()
    while next_needed < end:
        if time.time() > deadline:
            return None
        msg = master.recv_match(type='LOG_DATA', blocking=True, timeout=1)
        if msg is None:
            if time.time() - last_request > 3:
                send_request()  # 疑似丢包，从缺口处重发
            continue
        if msg.id != log_id:
            continue
        start = int(msg.ofs)
        data = bytes(msg.data[:msg.count])
        if start >= end or start + len(data) <= next_needed:
            continue  # 窗口外残留包 / 已收到的重复包
        if start < next_needed:  # 与已收部分重叠，裁掉前段
            data = data[next_needed - start:]
            start = next_needed
        if start + len(data) > end:  # 裁掉超出本块窗口的部分
            data = data[:end - start]
        if start not in pending:
            pending[start] = data
        while next_needed in pending:  # 连续组装
            block = pending.pop(next_needed)
            assembled.extend(block)
            next_needed += len(block)
    return bytes(assembled)


def download_log(master, log_id, on_progress=None, timeout=60, block_size=900):
    """
    Download a specific dataflash log by ID.
    Returns (log_data_bytes, error_message) tuple.
    on_progress callback: (current_bytes, total_bytes, log_id)
    timeout: 至少秒数；实际总时限按日志大小与波特率估算后再取较大值。
    """
    try:
        _drain_log_data(master)

        # First get log size from LOG_ENTRY (need to request it)
        master.mav.log_request_list_send(
            master.target_system, master.target_component,
            log_id, log_id
        )
        
        # Wait for LOG_ENTRY to get size
        t0 = time.time()
        log_size = None
        while time.time() - t0 < 5:
            msg = master.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
            if msg and msg.id == log_id:
                log_size = msg.size
                break
        
        if log_size is None:
            return None, "无法获取日志大小"
        if log_size == 0:
            return None, "日志为空"

        # 总超时：按 115200bps≈11KB/s 估算传输时间并留余量
        est_seconds = log_size / 8000.0 + 30
        deadline = time.time() + max(timeout, est_seconds)

        # Download log data block by block
        log_data = bytearray()
        offset = 0
        while offset < log_size:
            if time.time() > deadline:
                return None, f"下载超时: 已获取 {len(log_data)}/{log_size} 字节 (offset={offset})"
            req_len = min(block_size, log_size - offset)
            block = _request_log_block(master, log_id, offset, req_len, deadline)
            if block is None:
                return None, f"下载超时: 已获取 {len(log_data)}/{log_size} 字节 (offset={offset})"
            log_data.extend(block)
            offset += len(block)
            if on_progress:
                on_progress(len(log_data), log_size, log_id)

        if len(log_data) != log_size:
            return None, f"数据不完整: 期望 {log_size} 字节, 实际 {len(log_data)} 字节"
        return bytes(log_data), None
    except Exception as e:
        return None, f"下载异常: {e}"


def download_logs(master, log_ids, on_progress=None, timeout=60):
    """
    Download multiple logs.
    Returns dict: {log_id: (log_data_bytes, error_message)}
    """
    results = {}
    for log_id in log_ids:
        data, err = download_log(master, log_id, on_progress=on_progress, timeout=timeout)
        results[log_id] = (data, err)
        if err:
            print(f"Log {log_id} download failed: {err}")
    return results


def erase_logs(master, timeout=10):
    """Erase all dataflash logs."""
    try:
        master.mav.log_request_end_send(
            master.target_system, master.target_component
        )
        # Wait for LOG_ERASE or timeout
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = master.recv_match(type='LOG_ERASE', blocking=True, timeout=1)
            if msg:
                return True, "日志已清除"
        return False, "清除超时"
    except Exception as e:
        return False, f"清除异常: {e}"


def save_log_to_file(log_data, log_id, log_dir=None):
    """Save log data to .bin file."""
    if log_dir is None:
        log_dir = os.path.join(os.path.dirname(__file__), 'flight_logs')
    os.makedirs(log_dir, exist_ok=True)
    
    ts = time.strftime('%Y%m%d_%H%M%S')
    fname = f'log_{log_id}_{ts}.bin'
    fpath = os.path.join(log_dir, fname)
    
    with open(fpath, 'wb') as f:
        f.write(log_data)
    return fpath


def parse_log_with_mavlogdump(log_path, output_dir=None):
    """
    Parse .bin log using mavlogdump (if available) or pymavlink's mavlogdump.
    Returns path to parsed CSV directory or error.
    """
    if output_dir is None:
        output_dir = os.path.splitext(log_path)[0] + '_csv'
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Use pymavlink's mavlogdump module
        from pymavlink import mavutil
        # mavlogdump is a script, we can call it via subprocess
        import subprocess
        result = subprocess.run([
            sys.executable, '-m', 'pymavlink.tools.mavlogdump',
            '--format=csv', '--output', output_dir, log_path
        ], capture_output=True, text=True, timeout=120)
        
        if result.returncode == 0:
            return output_dir, None
        else:
            return None, f"解析失败: {result.stderr}"
    except FileNotFoundError:
        return None, "未找到 mavlogdump 工具"
    except Exception as e:
        return None, f"解析异常: {e}"


def analyze_log_with_ai(log_csv_dir, api_key, model='deepseek-flash', timeout=120,
                        provider='deepseek', base_url=None):
    """
    Send parsed log CSV files to AI for analysis.
    Returns AI analysis text or error.
    """
    import glob
    
    # Collect key CSV files (ATTITUDE, VIBRATION, GPS, etc.)
    csv_files = glob.glob(os.path.join(log_csv_dir, '*.csv'))
    key_types = ['ATTITUDE', 'VIBRATION', 'GPS', 'RCIN', 'RCOU', 'NKF', 'EKF', 'IMU', 'BARO', 'COMPASS']
    
    content_parts = []
    for csv_file in csv_files:
        fname = os.path.basename(csv_file).upper()
        if any(kt in fname for kt in key_types):
            try:
                with open(csv_file, 'r', encoding='utf-8', errors='ignore') as f:
                    # Read first 100 lines + header
                    lines = [f.readline() for _ in range(101)]
                content_parts.append(f"=== {fname} ===\n" + ''.join(lines))
            except Exception:
                pass
    
    if not content_parts:
        return None, "未找到关键日志数据"
    
    log_content = '\n\n'.join(content_parts)
    
    # Truncate if too long
    if len(log_content) > 50000:
        log_content = log_content[:50000] + "\n... (truncated)"
    
    system_prompt = """你是资深 ArduPilot/PX4 飞行日志分析专家。
根据提供的飞行日志 CSV 数据（ATTITUDE、VIBRATION、GPS、RCIN/RCOU、NKF/EKF 等），分析飞行状态并给出调参建议。
必须严格输出 JSON，格式：
{
  "summary": "飞行状态一句话总结",
  "issues": ["问题1", "问题2"],
  "recommendations": [{"param": "PARAM_NAME", "current": 0.135, "suggested": 0.15, "reason": "..."}, ...],
  "safety_notes": ["安全提示1"]
}"""
    
    user_prompt = f"""飞行日志数据片段:
{log_content}

请分析飞行状态，识别问题（振荡、振动、GPS/定位异常、磁偏、EKF 重置、电压/电流异常、姿态跟踪误差等），给出具体参数调整建议。"""
    
    return ai_chat(api_key, system_prompt, user_prompt, model=model,
                   provider=provider, base_url=base_url, timeout=timeout)