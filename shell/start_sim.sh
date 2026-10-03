#!/usr/bin/env bash
# 一键启动仿真系统：xhost对宿主机X server的授权 + docker compose up -d
# 起sim-world/flight-stack-nx01/flight-stack-nx02三个容器。不拉tmux监控
# 面板——监控看地面站(GCS)网页就够了，不需要这个脚本再额外拉一套。
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

# 为什么还需要xhost：容器里Gazebo/RViz2这类GUI窗口要连宿主机X server，
# xhost授权是宿主机X server自己的运行时ACL，只在当前X session存活期间
# 有效（重启图形会话/宿主机后会失效），docker compose本身没有能在`up`
# 之前自动跑宿主机命令的钩子，只能在外面包一层脚本里做。
if [[ -z "${DISPLAY:-}" ]]; then
    echo "警告：当前没有 DISPLAY 环境变量，Gazebo/RViz2 的GUI窗口不会显示（仍会正常仿真）。" >&2
else
    echo "== 授权容器访问宿主机 X server (xhost +local:docker) =="
    xhost +local:docker
fi

# 如果sim-world已经在跑，说明它的gzclient/rviz2这两个GUI子进程是在这次
# xhost授权之前就启动的，那时候大概率没被授权、已经崩溃退出了（容器本身
# 仍显示running）。只补xhost授权、不重启容器的话，已经死掉的GUI进程不会
# 自己复活。sim-world/flight-stack-nx01/flight-stack-nx02三个容器是一体的
# （flight-stack靠depends_on等sim-world、DLIO/mighty/MAVROS都要连
# sim-world里的PX4 SITL+点云+IMU等话题），单独重启sim-world会导致两个
# flight-stack容器缓存的旧连接状态跟新的sim-world脱节，所以三个一起重启。
RUNNING_SERVICES=$(docker compose ps --status running --services 2>/dev/null || true)
if echo "$RUNNING_SERVICES" | grep -qx "sim-world"; then
    echo "== 检测到 sim-world 容器已在运行，三个容器是一体的，一起重启以重新拉起 gzclient/rviz2（应用刚才的X server授权） =="
    docker compose restart sim-world flight-stack-nx01 flight-stack-nx02
fi

echo "== docker compose up -d =="
docker compose up -d
