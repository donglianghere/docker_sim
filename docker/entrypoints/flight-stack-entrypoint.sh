#!/usr/bin/env bash
# flight-stack 容器入口：单机的 DLIO + mighty + ros2_px4_stack 三件套。
# 跑两份容器实例（docker-compose里分别传 NAMESPACE=NX01 / NX02）。
#
# 环境变量：
#   NAMESPACE       必填，如 NX01 / NX02
#   AGENT_INDEX     必填，1-based，用于 ROS_DOMAIN_ID/MAV_SYS_ID等对齐PX4实例编号
#   SIM_WORLD_HOST  sim-world容器的可达地址（host网络模式下用127.0.0.1即可）
#   LOCALIZATION_SOURCE  默认 dlio。可选 gt——跳过真正的DLIO SLAM，改用
#                   gt_odom_bridge节点把Gazebo仿真真值（/plug/model_states_plug）
#                   转发成跟DLIO同名的odom话题，喂给mighty规划器和PX4飞控（经
#                   repub_odom.py的vision_pose）。用来单独验证"规划+板外控制器+
#                   飞控"这条链路，排除DLIO SLAM本身收敛/漂移的影响；三个模块
#                   （DLIO/mighty/ros2_px4_stack）都要测就用默认的dlio。
#                   两种模式二选一，不会同时跑DLIO和gt_odom_bridge——都发布同一个
#                   最终话题名(<namespace>/dlio/odom_node/odom)，同时跑会有两个
#                   发布者抢话题，下游收到谁的数据变得不确定。
#   CONTROL_LAW     默认 trajectory（唯一实测飞过、确认安全的模式）。可选
#                   attitude——2026-08-06一度试过切成默认，第一次真给目标点
#                   就炸机（get_angular()里`m/u1`没加零值保护，body_rate的yaw
#                   分量实测钉在-3.49rad/s，远超配置的限制），已改回
#                   trajectory，attitude在这个bug修好前不要再当默认用，
#                   见README.md。
set -eo pipefail

# ROS2/colcon 生成的 setup.bash 内部会引用一堆没给默认值的变量（比如这里第一个
# 就会炸的 AMENT_TRACE_SETUP_FILES），跟 `set -u` 天生冲突——source 期间关掉
# -u，source完再打开，后面自己写的变量都用了 ${VAR:-default}/`: "${VAR:?...}"`
# 写法，重新开 -u 不会再炸，但仍能兜住后面手滑打错变量名的低级错误。
set +u
source /opt/ros/humble/setup.bash
source /opt/decomp_ws/install/setup.bash 2>/dev/null || true
source /opt/mighty_ws/install/setup.bash
source /opt/dlio_ws/install/setup.bash
source /opt/ros2_px4_stack_ws/install/setup.bash
set -u

: "${NAMESPACE:?必须设置 NAMESPACE，如 NX01}"
: "${AGENT_INDEX:?必须设置 AGENT_INDEX，如 1}"
export VEH_NAME="${NAMESPACE}"

# dynus_mavros.launch.py 里的静态TF（world_mocap -> ${VEH_NAME}/init_pose）要用这几个
# INIT_* 环境变量，之前entrypoint里没设置过，全是None，传进Node(arguments=[...])直接
# 报 "TypeError: 'NoneType' object is not iterable" 崩溃。这里补上，公式跟
# sim-world-entrypoint.sh 里spawn这架飞机用的坐标（x:="$((i*3))" y:="0" z:="0.1" yaw:="0"）
# 保持一致——AGENT_INDEX在两边是同一个1-based编号（NX01=1对应i=1，NX02=2对应i=2），
# 位置对不上TF链会算错。z=0.1跟sim-world那边一致——iris的base_link碰撞箱
# （0.47x0.47x0.11米，以原点为中心）箱底在原点下方0.055米，z=0.1只比零净空高度
# 多留1.5厘米，肉眼几乎看不出下落。之前两边都是z=3（3米悬空），配合重力开着、
# 飞机没解锁，会直接自由落体砸到地面（实测确认过，local_position/pose从z=3掉到
# z=-0.09）；中间试过PX4官方脚本用的z=0.83，对这个具体的iris模型来说依然是
# 悬空的，还是会往下摔一截。
export INIT_X="$((AGENT_INDEX * 3))"
export INIT_Y="0"
export INIT_Z="0.1"
export INIT_ROLL="0"
export INIT_PITCH="0"
export INIT_YAW="0"

# 从 vehicle_profile.yaml 里把 mass/hover_thrust 解析出来，export给
# 打过patch的 ros2_px4_stack（get_thrust()/get_angular() 现在读这两个环境变量），
# 这样只用改 vehicle_profile.yaml 一处，mighty 和 ros2_px4_stack 就都用同一个数字，
# 不会再出现之前专项报告里那种 1.0kg vs 2.906kg 对不上的情况。
# 依赖 python3-yaml；如果基础镜像没装，去 Dockerfile.base 里加 python3-yaml。
export VEHICLE_MASS_KG=$(python3 -c "
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
print(d['offboard_dynus_follower']['ros__parameters']['mass'])
" 2>/dev/null || echo "2.906")
export VEHICLE_HOVER_THRUST=$(python3 -c "
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
print(d['offboard_dynus_follower']['ros__parameters']['hover_thrust'])
" 2>/dev/null || echo "0.50")
echo "== [flight-stack:${NAMESPACE}] VEHICLE_MASS_KG=${VEHICLE_MASS_KG} VEHICLE_HOVER_THRUST=${VEHICLE_HOVER_THRUST} =="

# 规划/避障/控制的重要可调参数，改成从 docker-compose.yml 的环境变量读，不再只能
# 靠改 hw_mighty.yaml 再重新build镜像才能调。复用上面同一套 vehicle_profile 机制
# （mighty_onboard_vehicle_profile.patch 加的 parameters.update()，本来就是"任意
# key都能覆盖"的通用逻辑，不用改launch文件代码）——但静态的
# /opt/config/vehicle_profile.yaml 是build时COPY进镜像的，改不了；这里在容器启动时
# 现算一份新的、多出这些key的yaml，传给 vehicle_profile:= 这个launch参数（见下面
# ros2 launch那一行），而不是直接改静态文件。
#
# 每个变量的默认值都保持跟当前 hw_mighty.yaml 里实际生效的数字一致，
# compose.yml没设这些环境变量时行为完全不变；一旦设了，这里生成的yaml就是
# 唯一生效的值——不会出现"两份数字不同步"的情况（hw_mighty.yaml里这几个字段
# 已经在mighty_planning_params_override_note.patch里加了注释，明确写着"实际生效
# 值以这里为准，改这里的数字不会有任何效果"，避免重蹈vehicle_profile.yaml文件头
# 注释里记录的那次教训——历史上这里出现过两份不同步的数字，只是刚好因为一个patch
# 顺序bug没让第二份意外生效）。
export V_MAX="${V_MAX:-1.0}"
export A_MAX="${A_MAX:-3.0}"
export J_MAX="${J_MAX:-5.0}"
export OMEGA_MAX="${OMEGA_MAX:-0.10472}"
export TIME_WEIGHT="${TIME_WEIGHT:-1.5e+2}"
export GOAL_SEEN_RADIUS="${GOAL_SEEN_RADIUS:-3.0}"
export GOAL_RADIUS="${GOAL_RADIUS:-0.3}"
export DYNAMIC_WEIGHT="${DYNAMIC_WEIGHT:-1e+2}"
export PLANNER_CW="${PLANNER_CW:-3.0}"
export DYN_CONSTR_THRUST_WEIGHT="${DYN_CONSTR_THRUST_WEIGHT:-1e+3}"
echo "== [flight-stack:${NAMESPACE}] 规划/避障/控制参数： v_max=${V_MAX} a_max=${A_MAX} j_max=${J_MAX} omega_max=${OMEGA_MAX} time_weight=${TIME_WEIGHT} goal_seen_radius=${GOAL_SEEN_RADIUS} goal_radius=${GOAL_RADIUS} dynamic_weight=${DYNAMIC_WEIGHT} planner_Cw=${PLANNER_CW} dyn_constr_thrust_weight=${DYN_CONSTR_THRUST_WEIGHT} =="
export VEHICLE_PROFILE_RUNTIME=/tmp/vehicle_profile_runtime.yaml
python3 -c "
import os
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
params = d.setdefault('mighty_node', {}).setdefault('ros__parameters', {})
params['v_max'] = float(os.environ['V_MAX'])
params['a_max'] = float(os.environ['A_MAX'])
params['j_max'] = float(os.environ['J_MAX'])
params['omega_max'] = float(os.environ['OMEGA_MAX'])
params['time_weight'] = float(os.environ['TIME_WEIGHT'])
params['goal_seen_radius'] = float(os.environ['GOAL_SEEN_RADIUS'])
params['goal_radius'] = float(os.environ['GOAL_RADIUS'])
params['dynamic_weight'] = float(os.environ['DYNAMIC_WEIGHT'])
params['planner_Cw'] = float(os.environ['PLANNER_CW'])
params['dyn_constr_thrust_weight'] = float(os.environ['DYN_CONSTR_THRUST_WEIGHT'])
with open(os.environ['VEHICLE_PROFILE_RUNTIME'], 'w') as f:
    yaml.safe_dump(d, f)
"

# MAVROS 之前整个没启动过（之前只跑了 dynus_mavros.launch.py，那里面没有mavros_node，
# 只是声明了一个从没被用到的fcu_url参数）——ros2_px4_stack 那几个节点因此一直连不上
# mavros/cmd/arming、mavros/set_mode、mavros/param/set 这些服务，日志里"not available,
# proceeding without it"就是这个原因，虽然没崩溃，但飞控指令根本发不出去。参照这个仓库
# 自己的 scripts/tmux/dynus_tmux.py 里的真实用法（MAVROS单独一个进程，跟
# dynus_mavros.launch.py是并列关系），补上这一步。
#
# fcu_url端口公式来自PX4官方 ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink：
#   udp_offboard_port_local  = 14580 + instance   （PX4监听，MAVROS要发到这个端口）
#   udp_offboard_port_remote = 14540 + instance   （PX4往外发，MAVROS要绑定这个端口收）
# instance是px4 -i的那个0-based编号，sim-world-entrypoint.sh里是 i-1（i是1-based循环变量），
# 跟这里的AGENT_INDEX是同一套1-based编号，所以 instance = AGENT_INDEX - 1。
# sim-world/flight-stack-nx01/flight-stack-nx02 三个容器全是 network_mode: host，共享同一个
# 网络栈——如果NX01/NX02两边都绑同一个端口（之前就是这样，写死14540/14580）会真的端口冲突，
# 必须按实例号错开。
PX4_INSTANCE=$((AGENT_INDEX - 1))
MAVROS_FCU_URL="udp://:$((14540 + PX4_INSTANCE))@127.0.0.1:$((14580 + PX4_INSTANCE))"
echo "== [flight-stack:${NAMESPACE}] 启动 MAVROS (fcu_url=${MAVROS_FCU_URL} tgt_system=${AGENT_INDEX}) =="
ros2 launch mavros px4.launch \
    namespace:="${NAMESPACE}/mavros" fcu_url:="${MAVROS_FCU_URL}" tgt_system:="${AGENT_INDEX}" &
sleep 3

LOCALIZATION_SOURCE="${LOCALIZATION_SOURCE:-dlio}"
export CONTROL_LAW="${CONTROL_LAW:-trajectory}"
if [ "${LOCALIZATION_SOURCE}" = "gt" ]; then
    echo "== [flight-stack:${NAMESPACE}] 定位模式=gt：跳过DLIO，改用Gazebo仿真真值 (gt_odom_bridge) =="
    ros2 run gt_odom_bridge gt_odom_bridge_node \
        --ros-args -r __ns:="/${NAMESPACE}" -p entity_name:="${NAMESPACE}" &
    sleep 2
else
    echo "== [flight-stack:${NAMESPACE}] 定位模式=dlio：启动 DLIO =="
    # patches/dlio_namespace.patch 让 dlio.launch.py 的 namespace:= 真正生效（Node
    # 会被套上 namespace=NX01/NX02），所以 pointcloud_topic/imu_topic 这两个remap
    # 目标必须传"相对"话题名（不带NX01前缀）——节点自己的namespace会在运行时把它们
    # 解析成 /NX01/mid360_PointCloud2、/NX01/mid360/imu；如果这里还手动拼上
    # "${NAMESPACE}/..."前缀，会被namespace再套一层变成 /NX01/NX01/mid360_PointCloud2
    # 这种双重前缀，订阅不到Gazebo实际发布的 /NX01/mid360_PointCloud2。
    ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
        namespace:="${NAMESPACE}" \
        pointcloud_topic:="mid360_PointCloud2" \
        imu_topic:="mid360/imu" &
    sleep 3
fi

echo "== [flight-stack:${NAMESPACE}] 启动 name_label_node（RViz里头顶跟随的名字标注）=="
# 订阅dlio/odom_node/odom（跟上面LOCALIZATION_SOURCE=dlio/gt两种模式发布的是
# 同一个话题名，不用关心当前用的是哪种定位源），发布一个跟着飞机位置走的
# TEXT_VIEW_FACING marker，文字默认取namespace（NX01/NX02）。
ros2 run gt_odom_bridge name_label_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

echo "== [flight-stack:${NAMESPACE}] 启动 mighty（含 use_frame_alignment/num_agents）=="
# `ros2 launch` 不支持裸的 --params-file（那是 `ros2 run` 的语法）；改用
# vehicle_profile:= 这个 launch 参数——patches/mighty_onboard_vehicle_profile.patch
# 给 onboard_mighty.launch.py 加了这个参数，会把 vehicle_profile.yaml 里
# mighty_node.ros__parameters 那部分覆盖到 mighty.yaml 默认值上面。
#
# use_hardware:=true + use_onboard_localization:=true：之前是 use_hardware:=false，
# 会让 onboard_mighty.launch.py 额外起一个 fake_sim_node，自己在内部积分一个"假状态"
# 发布到 mighty_node 订阅的 state 话题上——DLIO 处理的是 Gazebo 里的真实点云、MAVROS
# 也真的连上了 PX4，但 mighty 规划器根本没在用这些数据，是在拿自己编出来的状态做决策。
# 改成 true 之后走 convert_odom_to_state 节点（remap odom->dlio/odom_node/odom，
# 跟 DLIO 自己发布的话题名对得上），把 DLIO 真实里程计转换成 state 喂给 mighty——
# 这也是为什么前面本来就传了 use_frame_alignment:=true、sim-world也起了uwb_sim：
# use_hardware:=true 之后 map_frame_id 会变成每架飞机自己的 "${NAMESPACE}/map"
# （不再是全局共享的 "map"），要靠 UWB 的 frame_align 机制做跨机对齐，这一整套本来
# 就是配套设计好的，只是 use_hardware 这一个开关之前漏设了。
# lidar_point_cloud_topic:=mid360_PointCloud2 ——⚠️ 更正：这个覆盖参数本身没错，
# 但当初判断"这就是mighty收不到点云的原因"是错的。实测确认mighty_node.cpp压根
# 没有一个叫lidar_cloud_in的订阅（`ros2 node info /NX01/mighty_node`看到的实际
# 订阅列表是occupancy_grid/unknown_grid，不是这个remap目标），onboard_mighty.
# launch.py里mighty_node Node()定义的`remappings=[('lidar_cloud_in', ...)]`是
# 一段死代码，映射到哪个话题名对mighty_node的实际行为都没有任何影响。留着这个
# 参数无害（万一以后mighty_node.cpp重新加回这个订阅名，这里已经准备好了），
# 但它不是obstacle建图链路真正的修复点，见下面 global_mapper_ros 那段。
ros2 launch mighty onboard_mighty.launch.py \
    namespace:="${NAMESPACE}" sim_env:=none use_hardware:=true use_onboard_localization:=true \
    use_frame_alignment:=true num_agents:="${NUM_AGENTS:-2}" \
    lidar_point_cloud_topic:=mid360_PointCloud2 \
    vehicle_profile:="${VEHICLE_PROFILE_RUNTIME}" &
sleep 2

# mighty_node.cpp 无论 use_hardware=true 还是 sim_env=gazebo，实际订阅的都是
# occupancy_grid+unknown_grid这对同步的PointCloud2（占据栅格），不是原始雷达
# 点云——这两个话题要靠 global_mapper_ros 包的 global_mapper_node 节点从原始
# 点云+位姿算出来，但这个节点之前在这整套docker_sim里从来没被启动过，mighty
# 因此从一开始就没收到过任何障碍物数据。global_mapper_node.launch.py默认参数
# 已经跟这套仿真基本对得上：depth_pointcloud_topic默认mid360_PointCloud2、
# pose_topic默认state（跟convert_odom_to_state发布的话题名一致）、hardware:=false
# 时occupancy_grid_topic/unknown_grid_topic默认remap成'occupancy_grid'/
# 'unknown_grid'（正好是mighty_node订阅的裸话题名）。
# hardware:=false（不是true）是刻意的——hardware=true会选用cfg/hw_global_mapper.yaml，
# 那份配置是给地面机器人调的（world_dimensions 15x15、z轴以1.5m为中心），跟咱们
# 20x20米的simple_room room对不上；hardware=false选用的cfg/global_mapper.yaml
# 里world_dimensions=20x20、origin=(0,0,3)，注释写的就是"grid covers x,y∈[-10,+10]"，
# 正好是simple_room的范围。global_frame单独覆盖成"${NAMESPACE}/map"（这个参数
# 不受hardware开关联动，可以独立传）——跟mighty自己在use_hardware:=true时用的
# "${NAMESPACE}/map"保持一致，否则两边TF/坐标系对不上。
# drone_frame:="${NAMESPACE}/lidar" ——global_mapper_ros.cc里drone_frame launch
# 参数为空时，会自己猜一个雷达TF frame名字：`{quad}/{quad}_livox`（比如
# "NX01/NX01_livox"），但DLIO（patches/dlio_namespace.patch）实际发布的雷达外参
# TF是`{namespace}/lidar`（比如"NX01/lidar"）——两边命名约定对不上，显式覆盖成
# DLIO真正发布的那个frame名字。
# ⚠️ 这个覆盖解决的是lidar_frame_成员变量参与的那几处lookupTransform（比如FOV/
# 可见性相关的计算），不是实测报错刷屏的那处。真正刷屏的
# "lookupTransform(NX01/map -> NX01/NX01_livox) failed"来自
# global_mapper_ros.cc的PointCloudCallback，它查的是**点云消息自带的
# header.frame_id**（`cloud_msg->header.frame_id`），根本不读drone_frame/
# lidar_frame_这个参数。往上追到Gazebo雷达插件livox_points_plugin.cpp：
# `cloud.header.frame_id = ns_ + "/" + raySensor->Name();`——SDF里雷达sensor
# 本身就叫`{ns}_livox`（见gen_iris_mid360_sdf.py），拼出来正好是
# "NX01/NX01_livox"，这是Gazebo插件自己的命名习惯，跟DLIO的"NX01/lidar"是两套
# 完全不搭界的名字，中间没有任何TF把它们连起来。
# 补一条恒等静态TF把两个名字接上（物理上是同一个传感器、同一个位置，纯粹是
# 两边代码各自叫法不同）：NX01/lidar -> NX01/NX01_livox，零偏移。这样
# global_mapper_node查"NX01/map -> NX01/NX01_livox"时能沿着
# map->odom->base_link->lidar->NX01_livox这条完整链路解出来。
# 节点名带上${NAMESPACE}前缀——之前这里两架飞机都会起一个叫
# /static_tf_livox_alias的节点，`ros2 node list`实测确认了这个撞名
# （"share an exact name"警告）。发布的TF内容本身两边分别是NX01/...和
# NX02/...（正确加了命名空间），撞名不影响已经发布出去的数据，但节点名冲突
# 本身是个真实的ROS graph卫生问题，顺手一起修掉。
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 "${NAMESPACE}/lidar" "${NAMESPACE}/${NAMESPACE}_livox" \
    --ros-args -r __node:="${NAMESPACE}_static_tf_livox_alias" &

echo "== [flight-stack:${NAMESPACE}] 启动 global_mapper_ros（原始点云->occupancy_grid/unknown_grid）=="
ros2 launch global_mapper_ros global_mapper_node.launch.py \
    quad:="${NAMESPACE}" hardware:=false \
    global_frame:="${NAMESPACE}/map" drone_frame:="${NAMESPACE}/lidar" \
    depth_pointcloud_topic:=mid360_PointCloud2 pose_topic:=state &
sleep 2

echo "== [flight-stack:${NAMESPACE}] 启动 ros2_px4_stack (dynus分支 offboard follower) =="
ros2 launch ros2_px4_stack dynus_mavros.launch.py \
    namespace:="${NAMESPACE}" fcu_url:="${MAVROS_FCU_URL}" &

wait -n
