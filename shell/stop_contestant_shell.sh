#!/usr/bin/env bash
# 关闭 start_contestant_shell.sh 起的那个常驻选手容器
# （docker stop + docker rm，跟start_contestant_task.sh那种--rm一次性
# 容器没有关系，那种不需要关闭）。
CONTAINER_NAME="contestant-sdk-shell"
set -eo pipefail

# 文件管理器里"右键 -> Run as a program"这种方式，是临时开一个终端窗口跑
# 这个脚本，脚本一结束终端窗口就自动关掉，看不清输出——用trap在退出前
# 停一下等按键，双击运行时才能看清结果再关；命令行直接跑也会多等这一下，
# 代价很小。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

if ! docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "没有找到容器 $CONTAINER_NAME，不需要清理。"
    exit 0
fi

docker stop "$CONTAINER_NAME" >/dev/null
docker rm "$CONTAINER_NAME" >/dev/null
echo "已关闭并删除容器 $CONTAINER_NAME。"
