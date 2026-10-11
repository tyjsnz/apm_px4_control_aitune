# AP Tune 服务器部署指南 (Web Serial 版)

本版本把飞控链路移到了**浏览器端**:

```
[飞控 USB] ──串口── [你的电脑 + Chrome/Edge] ──HTTPS/WSS── [服务器上的本应用]
                                                      ├─ AI 代理 (日志/故障分析, Key 在服务端)
                                                      ├─ 故障速查记录存储
                                                      └─ 参数备份 / 飞行日志文件存储
```

- 浏览器通过 **Web Serial API** 直接打开本机 USB 串口,MAVLink/参数/日志操作全部在浏览器内完成
- 服务器不需要连接串口,可以部署在云端/内网任意机器上
- 备份与日志文件保存在服务器磁盘(通过 `/api/backups`、`/api/logs` 上传)

## 浏览器要求 (重要)

| 要求 | 说明 |
|---|---|
| Chrome / Edge (桌面版) | Web Serial API 仅 Chromium 系支持; Firefox / Safari 不可用 |
| HTTPS 或 localhost | Web Serial 只在**安全上下文**可用。用 IP 直接 `http://` 访问时 `navigator.serial` 不存在 → 必须配 HTTPS(见下) |
| 用户授权 | 每次选择串口需在弹出的系统对话框中授权;已授权端口下次可直接选择 |

## 快速部署 (Docker Compose)

```bash
cd apm_tune_web

# 1. (可选但强烈建议) 设置认证与域名
cat > .env <<'EOF'
AUTH_USER=admin
AUTH_PASS=换成强密码
APM_DOMAIN=tune.example.com
EOF

# 2. 启动 (HTTP, 适合先在局域网验证)
docker compose up -d --build

# 3a. 局域网验证后, 启用 HTTPS (Caddy 反代, 自签证书)
APM_DOMAIN=192.168.1.100 docker compose --profile tls up -d
# 浏览器首次访问需在警告页选择"继续访问"(自签证书)

# 3b. 公网域名场景: 删掉 Caddyfile 中的 `tls internal` 一行, Caddy 会自动签发 Let's Encrypt 证书
APM_DOMAIN=tune.example.com docker compose --profile tls up -d
```

访问 `https://<APM_DOMAIN>` (或 `http://服务器IP:3000`),在「连接」处选择串口即可。

## 环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `PORT` | 3000 | 监听端口 |
| `HOST` | 0.0.0.0 | 监听地址 |
| `AUTH_USER` / `AUTH_PASS` | 空 | HTTP Basic 认证;**公网部署必须设置**(本应用可写飞控参数/做电机测试) |
| `BACKUP_DIR` / `LOG_DIR` / `FAULT_DIR` | `./backups` `./logs` `./faults` | 数据目录 (compose 已挂载为宿主机同名目录) |
| `AI_CONFIG_FILE` | `./config/ai_config.json` | 全局 AI 配置(含 API Key) |

## 数据持久化

`docker-compose.yml` 已把以下目录挂载到宿主机(相对 compose 文件所在目录):

- `backups/` — 参数备份 (`backup_*.json` / `param_*.json`)
- `logs/` — 下载的飞行日志 (`log_*.bin`, 可删除以释放空间)
- `faults/` — 故障速查记录
- `config/` — AI 配置(含 Key,建议 `chmod 600 config`)

## 反向代理 / HTTPS 说明

- 用自带 Caddy profile(见快速部署)最省事;也可以用 Nginx/Traefik 替代,只需反代到 `apm-tune:3000`
- 若只在本机浏览器使用,`http://localhost:3000` 本身就是安全上下文,可免 HTTPS
- WebSocket 会自动随反代透传(路径为根路径 `/`)

## 使用流程

1. 电脑用 USB 连接飞控,打开 Chrome/Edge 访问本站点
2. 串口下拉框选择「选择新串口…」→ 在系统弹窗中授权(CH340/CP210x 等常见芯片均支持)
3. 选择波特率(默认 115200)→ 连接 → 参数读写 / 一键首飞 / 日志下载等与旧版完全一致
4. 下载的日志与备份会自动上传服务器保存;AI 分析在服务器端执行(服务器需能访问 AI 服务商)

## 故障排查

| 现象 | 原因与处理 |
|---|---|
| 串口下拉框只有「选择新串口…」 | 正常,尚未授权过任何串口;选择后系统弹窗授权,之后会记住 |
| 点连接没弹授权框 / 提示不支持 Web Serial | 浏览器不是 Chrome/Edge,或页面不是 HTTPS/localhost(检查地址栏是否有 "不安全" 标记) |
| 连接后无心跳 | 波特率不对(常见 115200/57600),或飞控 USB 只供电不通信;拔插后重新选择 |
| 打开串口失败 (Failed to open) | 串口被其它程序占用(如 Mission Planner / QGC),先关闭它们 |
| 提示 "需要认证" | 设置了 `AUTH_USER`/`AUTH_PASS`,浏览器弹出的登录框中输入即可 |
