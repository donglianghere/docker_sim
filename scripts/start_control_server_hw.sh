#!/usr/bin/env bash
# 一键SSH启动真机Jetson上的 control_server.py（飞机端HTTP控制/UDP心跳
# 进程，8890端口HTTP控制/查状态，8891端口UDP心跳广播供GCS自动发现，
# 见 docker_sim/scripts/control_server.py——这个文件只在Jetson上，这台
# 开发机没有）。这个脚本本身跑在GCS/开发机这一侧，SSH过去操作Jetson。
#
# 幂等：先pgrep检查目标机上是不是已经在跑，跑着了直接退出、不重复启动
# ——DEBUG_JOURNAL.md"探讨：地面站能否远程拉起control_server.py"那条
# 记录里明确提过"盲发容易重复启动、状态判断不准"的坑，这里先查再启动
# 规避。⚠️ 这不是常驻方案：真正的长期方向是在Jetson上把它注册成
# systemd service（Restart=always+开机自启），同一条记录里也提到过，
# 这个脚本只是"systemd还没配好之前"的手动一键启动过渡方案。
#
# 前提：本机 ~/.ssh/config 已经配好 nx01/nx02 别名 + 免密登录（《真机
# 部署操作清单》阶段0要求，不是这个脚本负责配置的）。
#
# 用法: ./scripts/start_control_server_hw.sh [nx01|nx02]
#   不传参数默认 nx01。
#   环境变量可覆盖：
#     REMOTE_PATH   默认 /home/nvidia/ai_uav/docker_sim/scripts/control_server.py
#     REMOTE_LOG    默认 /home/nvidia/ai_uav/docker_sim/logs/control_server.log
set -eo pipefail

SSH_HOST="${1:-nx01}"
REMOTE_PATH="${REMOTE_PATH:-/home/nvidia/ai_uav/docker_sim/scripts/control_server.py}"
REMOTE_LOG="${REMOTE_LOG:-/home/nvidia/ai_uav/docker_sim/logs/control_server.log}"
REMOTE_DIR="$(dirname "$REMOTE_PATH")"
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8)

echo "== [1/4] 检查 ${SSH_HOST} 是不是能免密连上 =="
if ! ssh "${SSH_OPTS[@]}" "$SSH_HOST" true 2>/dev/null; then
    echo "!! SSH连不上 ${SSH_HOST}（免密登录没配好，或者主机不在线/网络不通）" >&2
    echo "   先检查: ssh ${SSH_HOST}（应该不问密码），~/.ssh/config 里的别名是否配好" >&2
    exit 1
fi

echo "== [2/4] 检查 control_server.py 在 ${SSH_HOST} 上是不是已经在跑 =="
if ssh "${SSH_OPTS[@]}" "$SSH_HOST" "pgrep -f '[c]ontrol_server\.py'" >/dev/null 2>&1; then
    echo "-- 已经在跑，不重复启动。要重启的话先手动: ssh ${SSH_HOST} \"pkill -f control_server.py\"，再重新跑这个脚本 --"
    exit 0
fi

echo "== [3/4] SSH后台启动（nohup detach，日志追加写到 ${REMOTE_LOG}）=="
# 实测踩坑：光靠远程这边"nohup...&"+重定向三个fd，不保证ssh客户端本身
# 会返回——远程命令其实已经成功跑起来了，但本地ssh会因为等这次会话的
# 某些fd/channel关闭而卡住不退出，脚本表现为卡在这一步不往下走。真正
# 可靠的做法是让ssh客户端自己在鉴权完成后转后台（-f），不依赖远程那边
# 把fd清干净——加上timeout兜底，双保险，避免脚本被卡死。远程侧nohup+
# 重定向仍然保留（防止真的SIGHUP到进程本身），双重防护。
if ! timeout 15 ssh -f "${SSH_OPTS[@]}" "$SSH_HOST" \
    "mkdir -p '$(dirname "$REMOTE_LOG")' && cd '$REMOTE_DIR' && nohup python3 '$REMOTE_PATH' >>'$REMOTE_LOG' 2>&1 </dev/null &"; then
    echo "!! SSH启动命令15秒内没返回或失败，检查网络/免密登录：ssh ${SSH_HOST} 手动排查" >&2
    exit 1
fi

echo "== [4/4] 验证进程真的起来了（${SSH_HOST}上curl localhost:8890/status，最长等10秒）=="
DEADLINE=$((SECONDS + 10))
until ssh "${SSH_OPTS[@]}" "$SSH_HOST" "curl -sf -o /dev/null http://localhost:8890/status" 2>/dev/null; do
    if (( SECONDS > DEADLINE )); then
        echo "!! 10秒内 ${SSH_HOST} 的8890端口没起响应，检查远程日志: ssh ${SSH_HOST} tail -50 ${REMOTE_LOG}" >&2
        exit 1
    fi
    sleep 1
done
echo "-- control_server.py 已启动，8890/status 响应正常 --"
echo "GCS网页'查状态'/机队发现面板应该能自动发现到这台飞机了（UDP:8891心跳 + HTTP:8890）"
