#!/usr/bin/env bash
# 一键build仿真镜像：sim-world -> flight-stack-nx01，顺序执行、同一时间
# 只编一个——不能裸跑不带service名的`docker compose build`，那样两个
# 镜像会被并发build，叠加各自内部colcon/make的并行编译，在核数远超内存
# 的机器上容易把内存打爆、系统死机（docker-compose.yml/README里也有
# 同样的警告，是这台机器真实踩过的坑，不是这个脚本自己加的顾虑）。
# flight-stack-nx02复用flight-stack-nx01同一份镜像，不用单独build。
#
# 不包含公共基础镜像mighty-base:humble——那是几乎不变的底座（只有改
# docker/Dockerfile.base本身才需要重建），不属于日常迭代sim-world/
# flight-stack这两个镜像时的常规动作，本机已经build过。这个脚本只检查
# 它存不存在，不存在就报错提示手动build，不会替你重建。
#
# 这个脚本本身不放在docker_sim项目目录里，所以不能靠"脚本自己在哪个目录"
# 反推项目目录（那样脚本被移到/复制到别处就会失效）——直接写死
# docker_sim的绝对路径，不管这个脚本文件放在哪、从哪个当前目录执行，
# 都固定操作同一个项目目录。
PROJECT_DIR="/home/robots/ai_uav/docker_sim"
set -eo pipefail

# 文件管理器里"右键 -> Run as a program"这种方式，是临时开一个终端窗口跑
# 这个脚本，脚本一结束（不管成功还是失败）终端窗口就自动关掉，没时间看
# 输出/报错——"一闪而逝"就是这么来的，跟脚本本身对不对无关。用trap在
# 脚本退出前（EXIT，覆盖正常结束/`exit 1`/`set -e`中途失败三种情况）
# 停下来等一下按键，双击运行时窗口才能停留到你看清结果再关。命令行里
# 直接跑这个脚本时也会等这一下，代价很小，不影响正常使用。
trap 'ec=$?; echo; if [[ $ec -ne 0 ]]; then echo "!! 执行失败（退出码 $ec），请查看上面的错误信息"; else echo "-- 执行完成 --"; fi; read -n 1 -s -r -p "按任意键关闭窗口..."; echo' EXIT

cd "$PROJECT_DIR"

if ! docker image inspect mighty-base:humble >/dev/null 2>&1; then
    echo "!! 找不到 mighty-base:humble 镜像，请先手动build（见README《使用步骤》一节）：" >&2
    echo "     docker build --network=host -f docker/Dockerfile.base -t mighty-base:humble ." >&2
    exit 1
fi

docker compose build sim-world
docker compose build flight-stack-nx01
