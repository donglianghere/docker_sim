#!/usr/bin/env bash
# 给watch_sim.sh/record_rosbag.sh这些用docker exec现拼现起的辅助脚本
# （dual_goal_input.py/status_monitor.py/attitude_thrust_logger.py/
# ros2 bag record）统一source的RMW环境配置——原来是在每处docker exec的
# bash -c单引号参数里手写同一段if判断，字符串本身要穿过watch_sim.sh自己
# ->tmux起的shell->docker exec的bash -c这三层解析，改一次就要重新验证
# 一次转义关系，出过好几次因为$和"转义层级搞反导致改了等于没改的问题
# （见README 2026-08-08"dual_goal_input.py一直等位置数据"那次排查）。
# 抽成一个独立文件、用source加载，彻底不用再操心这层转义。
#
# entrypoint.sh里PLANNER=ego_planner时export的RMW_IMPLEMENTATION/
# CYCLONEDDS_URI只在它自己的进程树里有效，docker exec起的新shell看不到，
# 只能读到docker-compose environment:里设的真正容器级变量（PLANNER本身
# 是这种）——这里必须重新判断一遍、跟entrypoint.sh保持完全一致的逻辑，
# 否则PLANNER=ego_planner时这些辅助脚本默认还是FastDDS，跟已经切到
# CycloneDDS的真实节点（mavros/ego_planner_node等）完全对不上、发现不了
# 任何话题。
if [ "${PLANNER:-ego_planner}" = ego_planner ]; then
    # 2026-08-08实测踩过的另一个坑：容器刚重启的瞬间（比如start.sh检测到
    # sim-world已经在跑、决定重启三个容器之后，几乎立刻又去起这些
    # docker exec辅助脚本），entrypoint.sh自己还没跑到写这份配置文件的
    # 那一步——`docker inspect`确认过一次容器StartedAt和这份文件的mtime
    # 只差0.8秒，`dual_goal_input.py`当时直接因为"can't open configuration
    # file"崩溃退出。这里等文件真正出现再export，而不是假设它一定已经
    # 在了；最多等30秒（entrypoint.sh正常几秒内就能写到这一步，30秒是
    # 留了充足余量的上限，不是预期的正常等待时长），超时就放弃等待、
    # 直接往下走（不是卡死，只是退回到"文件不存在"这个原来就有的失败
    # 模式，不会比等之前更差）。
    CYCLONEDDS_CONF_FILE=/tmp/docker_sim_cyclonedds.xml
    for _i in $(seq 1 30); do
        [ -f "$CYCLONEDDS_CONF_FILE" ] && break
        sleep 1
    done
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://${CYCLONEDDS_CONF_FILE}"
fi
