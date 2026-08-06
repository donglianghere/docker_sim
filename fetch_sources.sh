#!/usr/bin/env bash
# ----------------------------------------------------------------------------
# 手动预取脚本 —— 在宿主机(Ubuntu24.04)上运行，不在 docker build 里跑。
# 用宿主机自己的网络/代理，把所有源码 clone 到 staging/ 下，docker build 时只做 COPY。
#
# 用法：
#   export http_proxy  = http://127.0.0.1:7897/
#   export https_proxy=$http_proxy
#   git config --global http.proxy $http_proxy      # 让 git clone 也走代理
#   ./fetch_sources.sh                                # 全部拉一遍
#   ./fetch_sources.sh --only px4,dlio                # 只拉指定的几组（见下面 GROUP 名）
#
# 幂等：目录已存在就跳过 clone，只做 fetch+checkout，可以放心重复执行。
# ----------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")/staging"

ONLY="${1:-}"
if [[ "$ONLY" == "--only" ]]; then ONLY="${2:-}"; fi

want() {
  # 没传 --only 就什么都拉；传了就只拉列表里出现的组
  [[ -z "$ONLY" ]] && return 0
  [[ ",$ONLY," == *",$1,"* ]]
}

clone_pin() {
  # clone_pin <group> <url> <dest_dir> <ref(branch/tag/commit)>
  local group=$1 url=$2 dest=$3 ref=$4
  want "$group" || return 0
  if [[ -d "$dest/.git" ]]; then
    echo "== [$group] $dest 已存在，git fetch 更新 =="
    git -C "$dest" fetch --all --tags
  else
    echo "== [$group] git clone $url -> $dest =="
    git clone "$url" "$dest"
  fi
  echo "   checkout $ref"
  git -C "$dest" checkout "$ref"
}

echo "############################################"
echo "# 1. PX4 固件 + 经典Gazebo仿真插件           #"
echo "############################################"
clone_pin px4 https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot v1.16.0
# PX4-Autopilot 的 .gitmodules 里 Tools/simulation/gazebo-classic/sitl_gazebo-classic
# 本来就注册成指向 PX4-SITL_gazebo-classic.git 的 submodule，这一步 submodule
# update 会把它一起拉到位——不需要（也不能）再单独clone一份出来COPY，
# 否则会跟submodule自己的.git文件打架（Docker COPY报 cannot copy to non-directory）。
if [[ -d PX4-Autopilot/.git ]]; then
  git -C PX4-Autopilot submodule update --init --recursive
fi

echo "############################################"
echo "# 2. DLIO (vectr-ucla, ROS2分支)             #"
echo "############################################"
clone_pin dlio https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git dlio_ws_src/direct_lidar_inertial_odometry feature/ros2

echo "############################################"
echo "# 3. mighty 及 mighty.repos 里的全部依赖      #"
echo "############################################"
clone_pin mighty https://github.com/mit-acl/mighty.git mighty_ws_src/mighty main
clone_pin mighty https://gitlab.com/mit-acl/lab/acl-mapping.git mighty_ws_src/acl-mapping 0d3875daf29d163a35c19d356c4652a749ed501d
clone_pin mighty https://github.com/kotakondo/dynus_interfaces.git mighty_ws_src/dynus_interfaces 628a682c6a12b07fb2be7cffb4782ccdfe5b323d
clone_pin mighty https://github.com/kotakondo/gazebo_ros_pkgs.git mighty_ws_src/gazebo_ros_pkgs ed1f7b886971d7bf898e6e0f59c622df929c0521
clone_pin mighty https://github.com/kotakondo/livox_laser_simulation_ros2.git mighty_ws_src/livox_laser_simulation_ros2 2f0024c1aebae0ad5d54e70672af824e8fd37dba
clone_pin mighty https://github.com/kotakondo/realsense_gazebo_plugin.git mighty_ws_src/realsense_gazebo_plugin 8c357e69da2160f54328db70915bd9b28eb975bf
clone_pin mighty https://github.com/kotakondo/uav_simulator.git mighty_ws_src/uav_simulator d4e953b716789bbb2a687eb4a49fe85d58bc03bc
# decomp_ws 单独一个工作区（README里写的编译顺序：先 decomp_util 再其余）
clone_pin mighty https://github.com/kotakondo/DecompROS2.git decomp_ws_src/DecompROS2 a2bc42ed46bf934323994706f17af8a3883860ea
# livox_ws 单独一个工作区（Livox-SDK2 是 cmake 装的非 ROS 库，livox_ros_driver2 用自己的 build.sh）
clone_pin mighty https://github.com/kotakondo/livox_ros_driver2.git livox_ws_src/livox_ros_driver2 274aad204b5dfce15c648e88dfd375fa5d7b3026
clone_pin mighty https://github.com/Livox-SDK/Livox-SDK2.git livox_ws_src/Livox-SDK2 6a940156dd7151c3ab6a52442d86bc83613bd11b

echo "############################################"
echo "# 4. ros2_px4_stack（kotakondo, dynus分支）  #"
echo "############################################"
clone_pin ros2px4 https://github.com/kotakondo/ros2_px4_stack.git ros2_px4_stack dynus

echo "############################################"
echo "# 5. hospital/office/tunnel 三个world依赖的  #"
echo "#    外部Gazebo模型资产（体积较大，选做）     #"
echo "############################################"
clone_pin gz_assets https://github.com/aws-robotics/aws-robomaker-hospital-world.git gazebo_models_external/aws-robomaker-hospital-world ros1
clone_pin gz_assets https://github.com/osrf/gazebo_models.git gazebo_models_external/osrf-gazebo_models master
clone_pin gz_assets https://github.com/osrf/subt.git gazebo_models_external/osrf-subt master

cat <<'EOF'

============================================================
预取完成。注意三点：
1. hospital.world 那组模型，aws-robomaker-hospital-world 仓库自己还带了一个
   fuel_utility.py，会额外去 Ignition Fuel 拉一批模型 —— 这部分我没有验证过是否
   在纯离线环境下能用，建议单独跑一遍这个脚本、确认能不能成功，成功了再把结果
   一并塞进 staging/gazebo_models_external/ 下。
2. office.world / tunnel.world 里具体要用到 osrf-gazebo_models / osrf-subt 仓库
   下的哪几个子目录（willowgarage / rough_tunnel_tile_* / subt_tunnel_staging_area
   等），需要你实际对照 world 文件里的 model:// 名字，从这两个大仓库里挑出对应的
   模型文件夹，放到 Dockerfile.sim-world 里 GAZEBO_MODEL_PATH 能找到的位置——
   这两个仓库体积很大，我没有把"整仓库塞进镜像"当成最终方案，建议你只挑需要的
   几个模型子目录出来精简一下。
3. 如果这次不打算首批就验证 hospital/office/tunnel 这三个场景，可以先
   `./fetch_sources.sh --only px4,dlio,mighty,ros2px4` 只拉核心链路，
   把 gz_assets 这组放到后面单独处理，不阻塞主线联调。
============================================================
EOF
