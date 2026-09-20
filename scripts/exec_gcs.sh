#!/usr/bin/env bash
# 交互式docker exec进gcs-gcs-1（GCS自己的ROS2节点栈：rosbridge_websocket+
# 前端静态文件服务这两个entrypoint.sh起的常驻进程所在的容器，装的是
# gcs_ws/decomp_ws两个overlay，用来跑ros2 CLI/rviz2排查），2026-09-01新增。
#
# 跟直接手敲`docker exec -it gcs-gcs-1 bash`比，多做了一件事：
#   RMW_IMPLEMENTATION——entrypoint.sh按PLANNER环境变量联动切换cyclonedds/
#   FastDDS（见gcs/entrypoint.sh里的注释），但那个export只在entrypoint.sh
#   自己那个进程树里生效，docker exec开的是全新shell看不到，必须在这里
#   重新判断一遍PLANNER、跟entrypoint.sh保持完全一致的逻辑——否则
#   PLANNER=ego_planner时这个交互shell看到的还是默认FastDDS，跟已经切到
#   CycloneDDS的真实rosbridge/ros2节点完全对不上话题，`ros2 topic list`
#   会是空的（看着像连不上，实际只是RMW不一致），跟scripts/ros2_env_setup.sh
#   （flight-stack容器同款问题）是同一类坑。
# CYCLONEDDS_URI/ROS_DOMAIN_ID不用在这里重复处理——两个都是docker-compose
# environment:里设的容器级变量，docker exec开的新shell天然能读到，只有
# RMW_IMPLEMENTATION这种"entrypoint.sh运行时export、不是容器级变量"的
# 才需要这份脚本重新推导一遍。
# ROS2/gcs_ws/decomp_ws这三个overlay的source不用在这里手动写——`bash`交互式
# shell会自动读/root/.bashrc，三行source已经预先写在里面了（见gcs/
# Dockerfile），比照式写法见Dockerfile里"2026-08-19实测确认"那段注释。
#
# 容器名固定为gcs-gcs-1（docker-compose项目名gcs+服务名gcs+序号1，
# gcs/backend/app.py里GCS_CONTAINER_NAME用的是同一个名字，改了要一起改）。
#
# 用法：
#   ./scripts/exec_gcs.sh                 # 交互式登录，进去就是能跑ros2 CLI
#                                          # 的bash
#   ./scripts/exec_gcs.sh "ros2 topic list"   # 免登录跑一条命令后退出
set -euo pipefail

CONTAINER="gcs-gcs-1"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" >/dev/null 2>&1; then
    echo "[exec_gcs] 容器 ${CONTAINER} 没有在跑，先用 scripts/up_gcs.sh 拉起来" >&2
    exit 1
fi

# 跟entrypoint.sh完全一致的判断逻辑，见上面文件头注释。
PLANNER_VAL="$(docker exec "$CONTAINER" printenv PLANNER 2>/dev/null || echo ego_planner)"
RMW_SETUP=""
if [ "$PLANNER_VAL" = "ego_planner" ]; then
    RMW_SETUP='export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; '
fi

if [ "$#" -eq 0 ]; then
    exec docker exec -it "$CONTAINER" bash -c "${RMW_SETUP}exec bash"
else
    exec docker exec -it "$CONTAINER" bash -c "${RMW_SETUP}$1"
fi
