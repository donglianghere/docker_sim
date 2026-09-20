#!/usr/bin/env bash
# 直连真机 192.168.2.106（nvidia@192.168.2.106）
# 用法：
#   ./ssh_106.sh              # 交互式登录
#   ./ssh_106.sh "cmd ..."    # 远程执行一条命令后退出

set -euo pipefail

HOST="192.168.2.106"
USER="nvidia"

exec ssh "${USER}@${HOST}" "$@"
