# AP Tune 服务器部署镜像 (Web Serial 架构)
# 串口由浏览器 Web Serial 直连, 服务端仅静态页面 + AI 代理 + 文件存储, 无原生依赖
FROM node:20-alpine

WORKDIR /app

# 依赖层缓存 (生产依赖只有 express + ws, 纯 JS 无需编译)
COPY package.json ./
RUN npm install --omit=dev --no-audit --no-fund

COPY server.js ./
COPY lib ./lib
COPY public ./public

# 数据目录 (挂载卷): 参数备份 / 飞行日志 / 故障记录 / AI 配置
RUN mkdir -p backups logs faults config && chown -R node:node /app
USER node

ENV PORT=3000 HOST=0.0.0.0
EXPOSE 3000

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD wget -q -O /dev/null "http://127.0.0.1:${PORT}/" || exit 1

CMD ["node", "server.js"]
