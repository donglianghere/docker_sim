#!/usr/bin/env bash
# 清理"构建镜像"产生的残余：build cache（docker build时BuildKit缓存的
# 中间层，反复build flight-stack/sim-world/contestant-sdk这些大镜像最
# 容易堆积）+ dangling镜像（<none>标签、没有任何tag指向的旧镜像层）。
#
# 不删任何有tag的镜像本身（不是`docker system prune -a`那种连镜像一起
# 清的做法）——contestant-sdk:latest/flight-stack:latest等现有镜像不受
# 影响，只清"为了加速下次build而缓存的中间产物"，代价是清完之后下一次
# 全新build会变慢（缓存没了要重新生成），但不影响已经build好的镜像能
# 不能正常跑。
#
# 这个脚本不依赖"自己在哪个目录"，也不依赖当前工作目录——纯操作docker
# daemon，跟脚本文件放在哪、从哪个目录双击/执行完全无关。
set -eo pipefail

# 文件管理器"右键 -> Run as a program"是临时开一个终端窗口跑这个脚本，
# 脚本一结束终端窗口就自动关掉，看不清输出——用trap在退出前停一下等
# 按键，双击运行时才能看清结果再关；命令行直接跑也会多等这一下，
# 代价很小。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

echo "当前docker磁盘占用："
echo
docker system df
echo

read -r -p "确认清理全部build cache + dangling镜像？（不影响现有已build好的镜像）(y/N) " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "已取消，什么都没有删除。"
    exit 0
fi

echo
echo "== 清理dangling镜像（<none>标签的旧镜像层）=="
docker image prune -f

echo
echo "== 清理build cache（-a：连带已有镜像引用过的缓存一起清，最大化释放空间）=="
docker builder prune -af

echo
echo "清理后docker磁盘占用："
docker system df
