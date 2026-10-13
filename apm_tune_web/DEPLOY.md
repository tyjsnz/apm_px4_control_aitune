# 扶摇 FuyaoTune 服务器部署指南 (Web Serial 版)

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

# 3b. 公网域名场景 (可选): 申请正式免费证书让浏览器无警告, 见下方「正式 HTTPS 证书」一节
APM_DOMAIN=tune.example.com docker compose --profile tls up -d
```

访问 `https://<APM_DOMAIN>` (或 `http://服务器IP:3000`),在「连接」处选择串口即可。

## 环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `PORT` | 3000 | 容器内监听端口 |
| `HOST` | 0.0.0.0 | 监听地址 |
| `AUTH_USER` / `AUTH_PASS` | 空 | HTTP Basic 认证;**公网部署必须设置**(本应用可写飞控参数/做电机测试) |
| `APM_DOMAIN` | localhost | tls profile 的站点域名,**可带端口**(如 `tune.example.com:8443`) |
| `TLS_PORT` | 443 | tls profile 对外映射的 HTTPS 端口(需与 `APM_DOMAIN` 中的端口一致) |
| `STATS_TOKEN` | 空 | 使用统计仪表盘口令(设置后访问 `/stats?token=口令`) |
| `STATS_DISABLED` | 空 | 设为 `1` 完全关闭匿名使用统计 |
| `BACKUP_DIR` / `LOG_DIR` / `FAULT_DIR` | `./backups` `./logs` `./faults` | 数据目录 (compose 已挂载为宿主机同名目录) |
| `AI_CONFIG_FILE` | `./config/ai_config.json` | 全局 AI 配置(含 API Key) |
| `STATS_FILE` | `./stats/usage.json` | 匿名使用统计数据(按天分桶 JSON) |

## 非标准端口部署 (80/443 被占用, 如宝塔面板)

服务器已跑 nginx/宝塔占用 80/443 时, 用非标准端口直接对外提供 HTTPS(Caddy 仍负责 TLS, 与常规用法一致):

```bash
cat > .env <<'EOF'
AUTH_USER=admin
AUTH_PASS=换成强密码
APM_DOMAIN=tune.example.com:8443
TLS_PORT=8443
EOF
docker compose --profile tls up -d
# 访问 https://tune.example.com:8443
```

要点:

- `APM_DOMAIN` 与 `TLS_PORT` 的端口必须一致(前者决定 Caddy 容器内监听端口, 后者决定宿主机映射)
- 云服务器**安全组**和本机防火墙都要放行该端口; `AUTH_USER`/`AUTH_PASS` 必设
- **不要用浏览器受限端口**: Chrome/Edge/Firefox 对 `6000-6005`、`6665-6669`、`10080` 等端口直接拒绝加载(报 `ERR_UNSAFE_PORT`), 推荐 `8443` / `9443` / `6443`
- 站点按域名匹配: **域名必须能解析到服务器**(用 IP 访问会连接被拒), 先在 DNS 添加 A 记录
- Caddyfile 的 `tls.conf` 默认 `tls internal` 时为自建 CA 证书: 首次访问浏览器显示"不安全", 点「高级 → 继续前往」即可(Web Serial 仍可用); 按「正式 HTTPS 证书」一节切换到 Let's Encrypt 后浏览器无警告(需域名 80 端口可达或用 DNS-01)


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
- TLS 证书策略在 `tls.conf`(被 Caddyfile import):默认 `tls internal` 自建 CA;改一行即可切换到正式证书(见下节)

## 正式 HTTPS 证书 (免浏览器警告, Let's Encrypt 免费)

自建 CA 证书首次访问会弹"不安全"警告,正式对外使用建议换成 Let's Encrypt:

```bash
# 1. 签发 (服务器 80 端口需外网可达; 复用已装的 acme.sh)
/root/.acme.sh/acme.sh --issue -d 你的域名 --webroot /www/server/nginx/html

# 2. 安装到项目 tls/ 目录, 并配置续签后自动 reload Caddy
/root/.acme.sh/acme.sh --install-cert -d 你的域名 \
  --fullchain-file /www/wwwroot/你的目录/tls/fullchain.pem \
  --key-file /www/wwwroot/你的目录/tls/privkey.pem \
  --reloadcmd "docker exec 你的caddy容器名 caddy reload --config /etc/caddy/Caddyfile --adapter caddyfile || docker restart 你的caddy容器名"

# 3. tls.conf 切换为文件证书并重建
echo "tls /etc/caddy/tls/fullchain.pem /etc/caddy/tls/privkey.pem" > tls.conf
docker compose --profile tls up -d
```

- acme.sh 已内置 cron 每天检查,到期前自动续签并执行 reloadcmd,无需手工干预
- 证书有效期 90 天(LE);80 端口不可达的服务器改用 DNS-01:`--issue -d 域名 --dns dns_ali`(需阿里云 AK)
- `tls/` 目录含私钥,已加入 .gitignore,勿入库

## 宝塔/系统 nginx 前置反代 (标准端口 https://域名 直连)

服务器 80/443 被宝塔 nginx 占用时,可让 nginx 做 443 的 TLS 终结并反代到容器,
实现不带端口的 `https://tune.flacn.net` 访问(Caddy 8443 可并存保留):

```nginx
# /www/server/panel/vhost/nginx/你的域名.conf
server {
    listen 80;
    server_name 你的域名;
    location ^~ /.well-known/acme-challenge/ {   # 保留 acme.sh webroot 续签
        root /www/server/nginx/html;
    }
    location / { return 301 https://$host$request_uri; }
}
server {
    listen 443 ssl http2;
    server_name 你的域名;
    ssl_certificate     /path/tls/fullchain.pem;      # acme.sh --install-cert 的文件
    ssl_certificate_key /path/tls/privkey.pem;
    ssl_protocols TLSv1.2 TLSv1.3;
    location / {
        proxy_pass http://127.0.0.1:3000;
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection $connection_upgrade;  # 需已有 map $http_upgrade $connection_upgrade
        proxy_read_timeout 3600s;      # 日志下载 / AI 分析耗时长
        client_max_body_size 0;        # 日志上传不限
        proxy_buffering off;
    }
}
```

要点:

- 80 块里的 **ACME 挑战路径必须排除重定向**,否则续签验证会跟着跳到 443 而失败
- `acme.sh --install-cert` 的 `--reloadcmd` 要同时 reload nginx 与 Caddy(见上节模板)
- 修改后 `nginx -t && nginx -s reload`;此文件为手工创建,宝塔面板 UI 中不显示属正常
- 关键:容器端口 3000 建议限制为本机访问,强制走 nginx 的 TLS 入口:
  `.env` 里设 `APM_BIND=127.0.0.1` 后 `docker compose up -d`,
  外网直连 `http://IP:3000` 即失效(避免误用明文 HTTP 导致串口被浏览器禁用)

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
| 浏览器报 `ERR_UNSAFE_PORT` | 部署端口在浏览器黑名单内(`6000-6005`/`6665-6669`/`10080` 等), 改用 `8443` 等端口并同步改 `APM_DOMAIN`/`TLS_PORT` |
| 用 IP 访问被拒绝 / 连接关闭 | 站点按域名匹配, 需用 `APM_DOMAIN` 中的域名访问(先加 DNS A 记录); 本机联调可用 `curl --resolve 域名:端口:IP` |
| 非 HTTPS 页面连接按钮灰色 + 顶部红条 | 页面不是安全上下文(用 `http://IP` 访问), 改用 `localhost` 或 HTTPS 访问 |
| 容器状态 unhealthy 但网页正常 | 旧版健康检查在启用认证时误判(根路径 401); 新版改用 node 探针仅将 5xx 视为异常, `docker compose up -d --build` 重建即可 |
