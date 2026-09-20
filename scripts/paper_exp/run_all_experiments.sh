#!/usr/bin/env bash
# 论文仿真侧实验总调度：E11收敛精度重复实验 / E13目标点重变换前后对照 /
# E15观测污染闭环 / E12真值角度扫描。每组之间按需重建容器（换环境变量）。
# 用法： nohup scripts/paper_exp/run_all_experiments.sh > runtime_logs/paper_exp/all.log 2>&1 &
set -uo pipefail
cd "$(dirname "$0")/../.."
mkdir -p runtime_logs/paper_exp

C1=docker_sim-flight-stack-nx01-1
C2=docker_sim-flight-stack-nx02-1
ENVSETUP='source /opt/ros/humble/setup.bash;
          source /opt/px4ctrl_ws/install/setup.bash 2>/dev/null;
          export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
          export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml;'

wait_ready() {   # 等两个PX4实例都Ready for takeoff、且DLIO在发里程计
  local deadline=$(( $(date +%s) + 300 ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    n=$(docker exec docker_sim-sim-world-1 bash -c 'grep -l "Ready for takeoff" /tmp/px4_NX0*.log 2>/dev/null | wc -l' 2>/dev/null || echo 0)
    if [ "${n:-0}" -ge 2 ]; then
      if docker exec "$C1" bash -c "${ENVSETUP} timeout 8 ros2 topic echo /NX01/dlio/odom_node/odom --once >/dev/null 2>&1"; then
        echo "   [ready] PX4×2 + DLIO 就绪"; sleep 5; return 0
      fi
    fi
    sleep 10
  done
  echo "   !! 等待就绪超时"; return 1
}

recreate() {     # $1... = 额外环境变量；重建三个容器并等就绪
  echo "== 重建容器: $* =="
  env "$@" LOCALIZATION_SOURCE=uwb_slam PLANNER=ego_planner CONTROLLER=px4ctrl \
      docker compose up -d --force-recreate sim-world flight-stack-nx01 flight-stack-nx02 >/dev/null 2>&1
  sleep 20
  wait_ready
}

goal_test() {    # $1=tag  —— E13：起飞后立刻发世界系目标点，量落点误差
  local tag="$1"
  echo "== E13 goal test: ${tag} =="
  for ns in NX01 NX02; do
    c=$([ "$ns" = NX01 ] && echo "$C1" || echo "$C2")
    docker exec "$c" bash -c "${ENVSETUP} ros2 service call /${ns}/set_origin_from_uwb std_srvs/srv/Trigger" >/dev/null 2>&1
    docker cp scripts/paper_exp/goal_retransform_test.py "$c":/tmp/ >/dev/null
  done
  for ns in NX01 NX02; do
    c=$([ "$ns" = NX01 ] && echo "$C1" || echo "$C2")
    docker exec "$c" bash -c "${ENVSETUP} for i in 1 2 3; do ros2 topic pub --once /${ns}/takeoff_land quadrotor_msgs/msg/TakeoffLand '{takeoff_land_cmd: 1}' >/dev/null 2>&1; sleep 0.2; done"
  done
  # 两机目标点分别朝各自远离对方的方向，约3.2米
  docker exec -d "$C1" bash -c "${ENVSETUP} python3 /tmp/goal_retransform_test.py NX01 --wx 5.0 --wy 2.5 --wait 120 > /tmp/goaltest_NX01.log 2>&1"
  docker exec -d "$C2" bash -c "${ENVSETUP} python3 /tmp/goal_retransform_test.py NX02 --wx 8.0 --wy -2.5 --wait 120 > /tmp/goaltest_NX02.log 2>&1"
  local deadline=$(( $(date +%s) + 400 ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    a=$(docker exec "$C1" pgrep -cf goal_retransform_test.py 2>/dev/null || echo 0)
    b=$(docker exec "$C2" pgrep -cf goal_retransform_test.py 2>/dev/null || echo 0)
    [ "${a:-0}" -eq 0 ] && [ "${b:-0}" -eq 0 ] && break
    sleep 10
  done
  for ns in NX01 NX02; do
    c=$([ "$ns" = NX01 ] && echo "$C1" || echo "$C2")
    line=$(docker exec "$c" grep RESULT /tmp/goaltest_${ns}.log 2>/dev/null | tail -1)
    echo "   ${tag} ${line}"
    echo "${tag} ${line}" >> runtime_logs/paper_exp_e13_goal_results.txt
  done
  for ns in NX01 NX02; do
    c=$([ "$ns" = NX01 ] && echo "$C1" || echo "$C2")
    docker exec "$c" bash -c "${ENVSETUP} for i in 1 2 3; do ros2 topic pub --once /${ns}/takeoff_land quadrotor_msgs/msg/TakeoffLand '{takeoff_land_cmd: 2}' >/dev/null 2>&1; sleep 0.2; done"
  done
  sleep 12
}

echo "############ A. E11 收敛精度重复实验（默认配置，再飞4次）############"
for i in 02 03 04 05; do
  recreate GOAL_RETRANSFORM=true || continue
  ./scripts/paper_exp/run_sim_run.sh "e11_run${i}" 3.0 3
done

echo "############ B. E13 目标点持续重变换：修复前 ############"
for i in 1 2; do
  recreate GOAL_RETRANSFORM=false || continue
  goal_test "before_run${i}"
done
echo "############ B. E13 目标点持续重变换：修复后 ############"
for i in 1 2; do
  recreate GOAL_RETRANSFORM=true || continue
  goal_test "after_run${i}"
done

echo "############ C. E15 观测野值污染闭环（抗差关/开）############"
recreate GOAL_RETRANSFORM=true UWB_OUTLIER_PROB=0.05 UWB_OUTLIER_MAG_M=2.0 ROTATION_ROBUST=false \
  && ./scripts/paper_exp/run_sim_run.sh e15_outlier_robustoff 3.0 3
recreate GOAL_RETRANSFORM=true UWB_OUTLIER_PROB=0.05 UWB_OUTLIER_MAG_M=2.0 ROTATION_ROBUST=true \
  && ./scripts/paper_exp/run_sim_run.sh e15_outlier_robuston 3.0 3

echo "############ D. E15b 恒定偏置闭环（验证式10/11）############"
recreate GOAL_RETRANSFORM=true UWB_BIAS_X=0.5 UWB_BIAS_Y=-0.3 \
  && ./scripts/paper_exp/run_sim_run.sh e15_bias 3.0 3

echo "############ E. E12 真值角度扫描 ############"
for pair in "0 90" "120 180"; do
  set -- $pair
  recreate GOAL_RETRANSFORM=true SPAWN_YAW_DEG_NX01="$1" SPAWN_YAW_DEG_NX02="$2" \
    && ./scripts/paper_exp/run_sim_run.sh "e12_yaw_${1}_${2}" 3.0 2
done

echo "############ 全部实验结束 ############"
