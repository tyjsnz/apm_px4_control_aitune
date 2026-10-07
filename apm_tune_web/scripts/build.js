#!/usr/bin/env node
// 构建便携包: 把本工具 + 自带 Node 运行时 + 启动器 打包成 zip
//   node scripts/build.js                # 仅当前平台
//   node scripts/build.js --all          # win-x64/macos-x64/macos-arm64/linux-x64
//   node scripts/build.js --targets linux-x64,win-x64
// 产物: dist/apm-tune-<target>/  与  dist/apm-tune-<target>.zip
import fs from 'fs';
import path from 'path';
import { Readable } from 'stream';
import { pipeline } from 'stream/promises';
import { execFileSync } from 'child_process';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..');
const DIST = path.join(ROOT, 'dist');
const CACHE = path.join(ROOT, '.cache');
const NODE_VERSION = process.env.NODE_VERSION || 'v20.18.1';

const PLATFORMS = {
  'win-x64': { node: 'win-x64', ext: 'zip', bin: 'node.exe', kind: 'win' },
  'win-arm64': { node: 'win-arm64', ext: 'zip', bin: 'node.exe', kind: 'win' },
  'macos-x64': { node: 'darwin-x64', ext: 'tar.gz', bin: 'bin/node', kind: 'mac' },
  'macos-arm64': { node: 'darwin-arm64', ext: 'tar.gz', bin: 'bin/node', kind: 'mac' },
  'linux-x64': { node: 'linux-x64', ext: 'tar.gz', bin: 'bin/node', kind: 'linux' },
  'linux-arm64': { node: 'linux-arm64', ext: 'tar.gz', bin: 'bin/node', kind: 'linux' },
};

function currentTarget() {
  const os = process.platform;
  const arch = process.arch;
  if (os === 'win32') return arch === 'arm64' ? 'win-arm64' : 'win-x64';
  if (os === 'darwin') return arch === 'arm64' ? 'macos-arm64' : 'macos-x64';
  if (os === 'linux') return arch === 'arm64' ? 'linux-arm64' : 'linux-x64';
  throw new Error('不支持的平台: ' + os);
}

function parseArgs() {
  const args = process.argv.slice(2);
  if (args.includes('--all')) {
    return ['win-x64', 'macos-x64', 'macos-arm64', 'linux-x64'];
  }
  const i = args.indexOf('--targets');
  if (i >= 0 && args[i + 1]) return args[i + 1].split(',').map((s) => s.trim()).filter(Boolean);
  return [currentTarget()];
}

async function download(url, dest) {
  if (fs.existsSync(dest) && fs.statSync(dest).size > 1024 * 1024) {
    console.log('  使用缓存: ' + path.basename(dest));
    return;
  }
  fs.mkdirSync(path.dirname(dest), { recursive: true });
  console.log('  下载 ' + url);
  const res = await fetch(url);
  if (!res.ok) throw new Error('下载失败 ' + res.status + ' ' + url);
  await pipeline(Readable.fromWeb(res.body), fs.createWriteStream(dest));
}

function extract(archive, outDir) {
  fs.mkdirSync(outDir, { recursive: true });
  execFileSync('tar', ['-xf', archive, '-C', outDir], { stdio: 'inherit' });
}

function copyDir(src, dest) {
  fs.cpSync(src, dest, { recursive: true });
}

function rmrf(p) { fs.rmSync(p, { recursive: true, force: true }); }

function makeLauncher(kind, target) {
  if (kind === 'win') {
    return '@echo off\r\n'
      + 'cd /d "%~dp0"\r\n'
      + 'echo 启动 AP Tune ...\r\n'
      + '"%~dp0runtime\\node.exe" "%~dp0server.js" --open\r\n'
      + 'if errorlevel 1 pause\r\n';
  }
  const sh = kind === 'mac' ? 'start.command' : 'start.sh';
  return `#!/bin/bash\n`
    + `cd "$(dirname "$0")"\n`
    + `echo "启动 AP Tune ..."\n`
    + `"./runtime/bin/node" "./server.js" --open\n`
    + `if [ $? -ne 0 ]; then echo "启动失败，按回车退出"; read; fi\n`;
}

const README = `AP Tune（APM/ArduPilot 调参 & 日志分析 Web 工具）— 便携版
========================================================
1. 把飞控用 USB 连接到本机（运行本程序的电脑）。
2. 运行启动器:
   - Windows: 双击 start.bat
   - macOS:   双击 start.command（如被拦截: 右键→打开，或系统设置里允许）
   - Linux:   ./start.sh
3. 会自动打开浏览器 http://localhost:3000 ；若没自动打开，手动访问该地址。
4. 关闭: 关掉弹出的命令行窗口即可。

说明:
- 已内置 Node 运行时，无需另外安装 Node。
- 数据目录: backups/(参数备份, 需自行备份)   logs/(下载的飞行日志)。
- 端口被占用时会启动失败；可编辑启动器，在命令后加 --port 3001 之类（或在命令前设置 PORT 环境变量）。
- 本程序能写飞控参数/电机测试/清除日志，切勿在联网环境中开放给不受信任的人。
`;

async function buildTarget(target) {
  const plt = PLATFORMS[target];
  if (!plt) throw new Error('未知目标: ' + target);
  const pkgDir = path.join(DIST, `apm-tune-${target}`);
  console.log(`\n=== 构建 ${target} ===`);
  rmrf(pkgDir);
  fs.mkdirSync(pkgDir, { recursive: true });

  // 1. Node 运行时
  const baseName = `node-${NODE_VERSION}-${plt.node}`;
  const archive = path.join(CACHE, `${baseName}.${plt.ext}`);
  await download(`https://nodejs.org/dist/${NODE_VERSION}/${baseName}.${plt.ext}`, archive);
  const extractDir = path.join(CACHE, `extract-${baseName}`);
  if (!fs.existsSync(path.join(extractDir, baseName))) {
    rmrf(extractDir);
    extract(archive, extractDir);
  }
  const runtimeSrc = path.join(extractDir, baseName);
  copyDir(runtimeSrc, path.join(pkgDir, 'runtime'));
  console.log('  Node 运行时已就绪');

  // 2. 生产依赖 (不跑编译脚本, serialport 用自带 prebuilds)
  const staging = path.join(CACHE, 'staging');
  rmrf(staging);
  fs.mkdirSync(staging, { recursive: true });
  fs.copyFileSync(path.join(ROOT, 'package.json'), path.join(staging, 'package.json'));
  if (fs.existsSync(path.join(ROOT, 'package-lock.json'))) {
    fs.copyFileSync(path.join(ROOT, 'package-lock.json'), path.join(staging, 'package-lock.json'));
  }
  console.log('  安装生产依赖 (npm ci --omit=dev --ignore-scripts) ...');
  execFileSync('npm', ['ci', '--omit=dev', '--ignore-scripts', '--no-audit', '--no-fund'],
    { cwd: staging, stdio: 'inherit' });
  copyDir(path.join(staging, 'node_modules'), path.join(pkgDir, 'node_modules'));

  // 3. 应用文件
  fs.copyFileSync(path.join(ROOT, 'server.js'), path.join(pkgDir, 'server.js'));
  fs.copyFileSync(path.join(ROOT, 'package.json'), path.join(pkgDir, 'package.json'));
  copyDir(path.join(ROOT, 'lib'), path.join(pkgDir, 'lib'));
  copyDir(path.join(ROOT, 'public'), path.join(pkgDir, 'public'));
  fs.mkdirSync(path.join(pkgDir, 'backups'), { recursive: true });
  fs.mkdirSync(path.join(pkgDir, 'logs'), { recursive: true });
  fs.writeFileSync(path.join(pkgDir, 'README.txt'), README);

  // 4. 启动器
  if (plt.kind === 'win') {
    fs.writeFileSync(path.join(pkgDir, 'start.bat'), makeLauncher('win', target));
  } else {
    const name = plt.kind === 'mac' ? 'start.command' : 'start.sh';
    const p = path.join(pkgDir, name);
    fs.writeFileSync(p, makeLauncher(plt.kind, target));
    fs.chmodSync(p, 0o755);
  }

  // 5. 打包 zip
  const zip = path.join(DIST, `apm-tune-${target}.zip`);
  rmrf(zip);
  execFileSync('zip', ['-r', '-q', zip, `apm-tune-${target}`], { cwd: DIST, stdio: 'inherit' });
  const size = (fs.statSync(zip).size / 1024 / 1024).toFixed(1);
  console.log(`  ✅ ${path.relative(ROOT, zip)} (${size} MB)`);
}

async function main() {
  if (process.platform !== 'darwin' && process.platform !== 'linux') {
    console.log('提示: 建议在 macOS/Linux 上构建(需要 tar/zip)。');
  }
  fs.mkdirSync(DIST, { recursive: true });
  const targets = parseArgs();
  for (const t of targets) await buildTarget(t);
  console.log('\n全部完成。产物在 dist/ 目录。');
}

main().catch((e) => { console.error('构建失败:', e.message); process.exit(1); });
