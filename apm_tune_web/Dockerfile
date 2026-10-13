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
# FPV 仿真台 (零 npm 依赖, 由 server.js 挂载到 /sim/; sim/test 由 .dockerignore 排除)
COPY sim ./sim
# 导引算法实验室 (静态页面 + JS 数值内核, 挂载 /guide、/guide/docs、/guide/refs)
COPY guide ./guide

# 数据目录 (挂载卷): 参数备份 / 飞行日志 / 故障记录
RUN mkdir -p backups logs faults && chown -R node:node /app
USER node

ENV PORT=3000 HOST=0.0.0.0
EXPOSE 3000

# 健康检查: 启用 HTTP Basic 认证时根路径返回 401, 属正常; 仅 5xx/连接失败视为不健康
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD node -e "require('http').get({host:'127.0.0.1',port:process.env.PORT||3000,path:'/',timeout:4000},r=>process.exit(r.statusCode<500?0:1)).on('error',()=>process.exit(1)).on('timeout',function(){this.destroy();process.exit(1)})" || exit 1

CMD ["node", "server.js"]
