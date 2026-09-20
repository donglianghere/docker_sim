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
# livox_ros_driver2 原来clone的是kotakondo个人fork（274aad2，2024-12-17），2026-08-13
# 改回官方Livox-SDK仓库——fork分叉点是官方commit b6ff7d1(2023-11-29)，之后官方还有
# 43bb2f4/6b9356c（README/版本号更新）、13eb05e（新增Ubuntu24.04+ROS2 Jazzy支持，
# 这个项目用不上）、4a1def9（新增Avia2雷达支持+修了一个"配置项全部留空时点云不
# 发布"的bug）四个commit没跟进；fork自己在分叉点之后加的4个commit里，改
# config/MID360_config.json的IP地址是kotakondo自己硬件的私有配置(本来就要改成
# 真实设备IP)，两次README更新无实质内容，唯一有价值的是274aad2里
# InitImuMsg()的4行改动(IMU消息frame_id从硬编码"livox_frame"改成读可配置的
# frame_id_成员变量，跟点云发布已经在用的frame_id_保持一致)——这个改动官方
# master分支(4a1def9)里依然没有修，已单独摘出来做成
# patches/livox_ros_driver2_imu_frame_id.patch。
clone_pin mighty https://github.com/Livox-SDK/livox_ros_driver2.git livox_ws_src/livox_ros_driver2 4a1def929e5b59c7a8122d19fce6efba581ce9f7
# Livox-SDK2原来pin在6a94015（2024-02-18），跟上面livox_ros_driver2换成官方最新
# （4a1def9，2026-07-31，"Support Avia2 Lidar"）之后API对不上——2026-08-13实测
# docker build失败：livox_ros_driver2的src/comm/pub_handler.cpp引用
# kLivoxLidarTypeMid360s/kLivoxLidarDoubleEchoData/LivoxLidarDoubleEchoRawPoint
# 这几个SDK枚举/结构体，6a94015这个SDK版本里根本没有定义（是2026-04-14
# 22d98dc"Support Mid-360s Lidar"之后才加的）。SDK的08f523c commit日期
# （2026-07-31）跟driver的4a1def9完全同一天，明显是配套发布，pin一起跟到
# 官方最新，两边API才对得上（已用mighty-base:humble镜像单独跑colcon build
# 验证过：6a94015+4a1def9组合编译报错，08f523c+4a1def9组合编译通过）。
clone_pin mighty https://github.com/Livox-SDK/Livox-SDK2.git livox_ws_src/Livox-SDK2 08f523c930b2f0ba1e98a6afaa8d7476bf479908

echo "############################################"
echo "# 4. ros2_px4_stack（kotakondo, dynus分支）  #"
echo "############################################"
clone_pin ros2px4 https://github.com/kotakondo/ros2_px4_stack.git ros2_px4_stack dynus

echo "############################################"
echo "# 5. ego-planner-swarm（ZJU-FAST-Lab, ROS2分支）#"
echo "#    只用其中src/planner的规划器核心，Dockerfile #"
echo "#    COPY时只挑src/planner，不带src/uav_simulator#"
echo "#    （跟mighty_ws_src/uav_simulator同源，13个   #"
echo "#    包名重复，规划器核心本身不依赖它）           #"
echo "############################################"
clone_pin egoplanner https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git ego-planner-swarm 23a8d5a191711dd65633df689bd00f55d4dea8f9

echo "############################################"
echo "# 6. nlink_parser2（nooploop官方UWB ROS2驱动，真机用，2026-08-13新增） #"
echo "#    LinkTrack系列真实驱动，跟uwb_sim仿真真值替身是两回事——只在      #"
echo "#    flight-stack镜像编译，sim-world不需要。带3个submodule           #"
echo "#    （extern/asio、src/nlink_utils/nlink_unpack、                  #"
echo "#    src/nlink_utils/protocol_extracter），clone_pin不支持submodule，#"
echo "#    checkout之后单独补一次 submodule update --init --recursive     #"
echo "#    （跟上面PX4-Autopilot的处理方式一样）。                         #"
echo "############################################"
clone_pin nlink https://github.com/nooploop-dev/nlink_parser2.git nlink_ws_src/nlink_parser2 efbd56437c524d95487ed866df1ce93099880eb8
if [[ -d nlink_ws_src/nlink_parser2/.git ]]; then
  git -C nlink_ws_src/nlink_parser2 submodule update --init --recursive
fi

echo "############################################"
echo "# 7. hospital/office/tunnel 三个world依赖的  #"
echo "#    外部Gazebo模型资产（体积较大，选做）     #"
echo "############################################"
clone_pin gz_assets https://github.com/aws-robotics/aws-robomaker-hospital-world.git gazebo_models_external/aws-robomaker-hospital-world ros1
clone_pin gz_assets https://github.com/osrf/gazebo_models.git gazebo_models_external/osrf-gazebo_models master
clone_pin gz_assets https://github.com/osrf/subt.git gazebo_models_external/osrf-subt master

echo "############################################"
echo "# 8. point-lio ROS2移植（dfloreaa个人fork，仅用于SLAM选型评估，#"
echo "#    暂不接入任何镜像的Dockerfile，只clone到staging/供分析代码用）#"
echo "#    官方hku-mars/Point-LIO没有ROS2分支，dfloreaa是目前star数   #"
echo "#    最高(244)的第三方ROS2移植，支持Livox系列+Unitree Unilidar #"
echo "############################################"
clone_pin pointlio https://github.com/dfloreaa/point_lio_ros2.git pointlio_ws_src/point_lio_ros2 a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7

echo "############################################"
echo "# 9. hku-mars/Point-LIO 官方仓库（ROS1，master分支，评估用）#"
echo "#    只用于源码级分析对比，不接入任何镜像构建。默认分支现在  #"
echo "#    是point-lio-with-grid-map（2026-06-13还在更新），这里 #"
echo "#    pin的是master——第三方ROS2移植(dfloreaa)fork自的就是   #"
echo "#    这个分支，作为算法基线跟第三方移植做逐项对比更合适     #"
echo "############################################"
clone_pin pointlio_ros1 https://github.com/hku-mars/Point-LIO.git pointlio_ws_src/Point-LIO_official_ros1 1510c3bbf1743f254d83d0b22fbabb5b2d729f6b
# 带ikd-Tree/IKFoM两个submodule，clone_pin不处理submodule，跟前面PX4/nlink_parser2一样单独补一次
if [[ -d pointlio_ws_src/Point-LIO_official_ros1/.git ]]; then
  git -C pointlio_ws_src/Point-LIO_official_ros1 submodule update --init --recursive
fi

echo "############################################"
echo "# 10. hku-mars/FAST_LIO 官方仓库ROS2分支（评估用）           #"
echo "#     maintainer就是Ericsiii本人，跟此前单独评估过的         #"
echo "#     Ericsii/FAST_LIO_ROS2同一血统/同一套代码，这里改clone  #"
echo "#     官方仓库本身的ROS2分支，只用于源码级分析，不接入        #"
echo "#     任何镜像构建                                           #"
echo "############################################"
clone_pin fastlio2ros2 https://github.com/hku-mars/FAST_LIO.git pointlio_ws_src/FAST_LIO_official_ros2 a4743b095409588842a5b30ddfa27e29d2f99164
# 带ikd-Tree一个submodule，同上单独补一次
if [[ -d pointlio_ws_src/FAST_LIO_official_ros2/.git ]]; then
  git -C pointlio_ws_src/FAST_LIO_official_ros2 submodule update --init --recursive
fi

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
