#!/usr/bin/env bash
# 论文实验的一次完整仿真run：锁起飞点 -> 开始录包 -> 起飞 -> 长机动 -> 停录 -> 降落。
#
# 用法：  scripts/paper_exp/run_sim_run.sh <run_tag> [side_m] [laps]
# 产物：  runtime_logs/paper_exp/<run_tag>/  （rosbag，宿主机上直接能读）
#
# 只录论文需要的几路轻量话题，不用scripts/record_rosbag.sh那份完整列表——那份
# 带点云/占据栅格，几十MB/秒，对这个实验纯属浪费；这里几路PoseStamped/Odometry
# 加起来不到1MB/秒。
set -uo pipefail
cd "$(dirname "$0")/../.."

TAG="${1:?用法: run_sim_run.sh <run_tag> [side_m] [laps]}"
SIDE="${2:-3.0}"
LAPS="${3:-3}"
NS_LIST=(NX01 NX02)
C_NX01=docker_sim-flight-stack-nx01-1
C_NX02=docker_sim-flight-stack-nx02-1
BAG_DIR_IN_C="/logs/paper_exp/${TAG}"
BAG_DIR_HOST="runtime_logs/paper_exp/${TAG}"

# 容器里跑ros2 CLI必须自己把DDS环境补齐——entrypoint导出的RMW_IMPLEMENTATION/
# CYCLONEDDS_URI只在它自己起的那些节点进程里，docker exec进来是新的shell，
# 不带这些变量的话`ros2 topic list`只能看到/rosout和/parameter_events。
ENVSETUP='source /opt/ros/humble/setup.bash;
          source /opt/px4ctrl_ws/install/setup.bash 2>/dev/null;
          export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
          export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml;'

TOPICS=""
for ns in "${NS_LIST[@]}"; do
  TOPICS="$TOPICS /${ns}/dlio/odom_node/odom /${ns}/uwb/pose_abs /${ns}/uwb/pose_truth"
  TOPICS="$TOPICS /${ns}/origin_setter/yaw_estimate /${ns}/origin_setter/yaw_sample_count"
  TOPICS="$TOPICS /${ns}/origin_setter/residual_mad /${ns}/origin_setter/downweighted"
  TOPICS="$TOPICS /${ns}/term_goal"
done
TOPICS="$TOPICS /tf /tf_static"

echo "=========== run ${TAG} (side=${SIDE}m laps=${LAPS}) ==========="

echo "== 1/6 锁定起飞点（两机静止时做，θ*此时还是0，之后飞起来才在线更新）=="
for ns in "${NS_LIST[@]}"; do
  c="C_${ns}"; c="${!c}"
  out=$(docker exec "$c" bash -c "${ENVSETUP} ros2 service call /${ns}/set_origin_from_uwb std_srvs/srv/Trigger" 2>&1 | tail -2)
  echo "   ${ns}: ${out}"
done

echo "== 2/6 开始录包 -> ${BAG_DIR_HOST} =="
docker exec "$C_NX01" bash -c "rm -rf ${BAG_DIR_IN_C}; mkdir -p /logs/paper_exp"
docker exec -d "$C_NX01" bash -c "${ENVSETUP} ros2 bag record -o ${BAG_DIR_IN_C} ${TOPICS} > /tmp/paper_bag_${TAG}.log 2>&1"
sleep 5

echo "== 3/6 起飞（TakeoffLand话题连发3次，跟launch_control.py同一套）=="
for ns in "${NS_LIST[@]}"; do
  c="C_${ns}"; c="${!c}"
  docker exec "$c" bash -c "${ENVSETUP} for i in 1 2 3; do ros2 topic pub --once /${ns}/takeoff_land quadrotor_msgs/msg/TakeoffLand '{takeoff_land_cmd: 1}' >/dev/null 2>&1; sleep 0.2; done"
  echo "   ${ns}: 起飞口令已下达"
done

echo "== 4/6 长机动（两机并行，各自局部系里绕${SIDE}米方形飞${LAPS}圈）=="
for ns in "${NS_LIST[@]}"; do
  c="C_${ns}"; c="${!c}"
  docker cp scripts/paper_exp/paper_flight.py "$c":/tmp/paper_flight.py >/dev/null
  docker exec -d "$c" bash -c "${ENVSETUP} python3 /tmp/paper_flight.py ${ns} --side ${SIDE} --laps ${LAPS} > /tmp/paper_flight_${ns}.log 2>&1"
done
# 等两边的机动进程结束（有RTF因子，墙钟时间比仿真时间长，给足超时）
deadline=$(( $(date +%s) + 900 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  n=0
  for ns in "${NS_LIST[@]}"; do
    c="C_${ns}"; c="${!c}"
    docker exec "$c" pgrep -f "paper_flight.py ${ns}" >/dev/null 2>&1 && n=$((n+1))
  done
  [ "$n" -eq 0 ] && break
  sleep 10
done
echo "   机动结束（或超时），各机日志末尾："
for ns in "${NS_LIST[@]}"; do
  c="C_${ns}"; c="${!c}"
  echo "   --- ${ns} ---"; docker exec "$c" tail -3 /tmp/paper_flight_${ns}.log 2>/dev/null | sed 's/^/   /'
done

echo "== 5/6 停止录包 =="
docker exec "$C_NX01" bash -c 'pkill -INT -f "ros2 bag record" || true'
sleep 6

echo "== 6/6 降落 =="
for ns in "${NS_LIST[@]}"; do
  c="C_${ns}"; c="${!c}"
  docker exec "$c" bash -c "${ENVSETUP} for i in 1 2 3; do ros2 topic pub --once /${ns}/takeoff_land quadrotor_msgs/msg/TakeoffLand '{takeoff_land_cmd: 2}' >/dev/null 2>&1; sleep 0.2; done"
done
sleep 10
du -sh "${BAG_DIR_HOST}" 2>/dev/null || echo "   ⚠️ 没生成bag，检查 /tmp/paper_bag_${TAG}.log"
echo "=========== run ${TAG} 结束 ==========="
