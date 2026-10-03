#!/usr/bin/env bash
# 清理已经停止(Exited)的容器残余——不动正在运行的容器，只删已经退出的。
#
# 这个脚本不依赖"自己在哪个目录"，也不依赖当前工作目录——纯操作docker
# daemon，跟脚本文件放在哪、从哪个目录双击/执行完全无关。
set -eo pipefail

# 文件管理器"右键 -> Run as a program"是临时开一个终端窗口跑这个脚本，
# 脚本一结束终端窗口就自动关掉，看不清输出——用trap在退出前停一下等
# 按键，双击运行时才能看清结果再关；命令行直接跑也会多等这一下，
# 代价很小。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

echo "当前已停止(Exited)的容器："
echo
docker ps -a --filter "status=exited" --format "table {{.ID}}\t{{.Image}}\t{{.Status}}\t{{.Names}}"
echo

exited_count=$(docker ps -a --filter "status=exited" -q | wc -l | tr -d ' ')
if [[ "$exited_count" -eq 0 ]]; then
    echo "没有已停止的容器，不需要清理。"
    exit 0
fi

read -r -p "确认删除以上 ${exited_count} 个已停止的容器？(y/N) " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "已取消，什么都没有删除。"
    exit 0
fi

echo
docker container prune -f
