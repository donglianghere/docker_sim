#!/usr/bin/env bash
# sim-world 容器入口：起 Gazebo(装了双机模型+world) + N个PX4 SITL实例 + UWB真值节点。
# 环境变量：
#   WORLD_ENV       默认 hard_forest（对应 mighty worlds/hard_forest.world）
#   NUM_AGENTS      默认 2
#   USE_GAZEBO_GUI  默认 false（容器里没有桌面就别开GUI，除非你转发了X11）
#   USE_RVIZ        默认 true，起RViz2看点云/建图（NX01/NX02双机命名空间的
#                   multi_mighty.rviz配置），跟gzclient一样靠DISPLAY/X11转发
#   PLANNER         默认 mighty，决定用哪份rviz配置——sim-world容器本身跟
#                   规划器选择无关（PX4 SITL/Gazebo两个规划器都要用），
#                   这个变量纯粹是为了跟flight-stack那边选的PLANNER保持
#                   一致，选对应的rviz视图。=mighty用multi_mighty.rviz
#                   （NX01/NX02两个分组默认展开，看mighty自己的建图/轨迹）；
#                   =ego_planner用multi_ego_planner.rviz（跟multi_mighty.rviz
#                   基本是同一份，只是默认打开的Display不一样：ego_planner
#                   相关的occupancy_inflate/goal_point/global_list/
#                   init_list/optimal_list/a_star_list默认展开，mighty的
#                   NX01/NX02分组默认收起——两边topic不冲突，只是"默认展开
#                   哪些"不一样，两份文件都能看到全部内容，只是初始勾选
#                   状态不同，方便对应场景一打开就是有意义的画面，不用
#                   每次手动勾选）。
set -eo pipefail

# ROS2/colcon 生成的 setup.bash 内部会引用一堆没给默认值的变量（比如这里第一个
# 就会炸的 AMENT_TRACE_SETUP_FILES），跟 `set -u` 天生冲突——source 期间关掉
# -u，source完再打开，后面自己写的变量都用了 ${VAR:-default} 写法，重新开 -u
# 不会再炸，但仍能兜住后面手滑打错变量名的低级错误。
set +u
source /opt/ros/humble/setup.bash
source /opt/decomp_ws/install/setup.bash 2>/dev/null || true
source /opt/livox_ws/install/setup.bash 2>/dev/null || true
source /opt/mighty_ws/install/setup.bash
source /opt/uwb_ws/install/setup.bash
# Gazebo自己的环境脚本——从来没人source过，之前一直靠Dockerfile.sim-world里
# 单独手写的GAZEBO_MODEL_PATH能凑合把world的障碍物模型加载出来，所以一直没
# 发现这个缺口。真正暴露是在查GUI崩溃时：GAZEBO_RESOURCE_PATH（gazebo-11自带
# 的媒体/材质资源）和OGRE_RESOURCE_PATH（OGRE渲染引擎自己的插件目录）全是空的
# ——`ros2 topic echo`/`env`实测确认过。联网查`Assertion 'px != 0' failed`
# 这个具体的Camera/Scene空指针assert，公开报告里能对上号的场景基本都是渲染
# 资源路径/环境没配对导致OGRE渲染系统初始化失败、返回空指针又没做判空保护，
# 不是GPU驱动或world复杂度的问题（之前排查方向：/dev/dri硬件加速、
# LIBGL_ALWAYS_SOFTWARE软件渲染、world障碍物数量、gzclient/gzserver启动
# 时序，全部试过都没解决）。手动测试补上这行source之后，gzserver+gzclient
# 一起并发起（不需要再拆分先后顺序）也不再崩了。
source /usr/share/gazebo/setup.sh
# PX4官方自己的Gazebo环境脚本——同样从来没人source过。libgazebo_mavlink_interface.so
# （提供PX4<->Gazebo的TCP桥接、以及MAVROS用的MAVLink UDP服务）确实编译出来了，
# 但装在PX4自己的构建产物目录`build/px4_sitl_default/build_gazebo-classic`，
# 不在任何标准系统插件路径下，GAZEBO_PLUGIN_PATH原来是空的，Gazebo根本找不到
# 这个插件——iris+mid360融合模型能正常spawn进场景（`spawn_entity.py`不检查
# 插件加载成不成功），但mavlink_interface插件静默加载失败，PX4进程因此永远
# 连不上仿真器，日志卡死在"Waiting for simulator to accept connection on
# TCP port 4560"，MAVROS自然也connected:false——这才是根子，不是模型本身
# 缺插件（模型里确实塞了这个插件定义，问题在Gazebo找不到.so文件本身）。
source /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
    /opt/PX4-Autopilot /opt/PX4-Autopilot/build/px4_sitl_default
set -u

WORLD_ENV="${WORLD_ENV:-hard_forest}"
NUM_AGENTS="${NUM_AGENTS:-2}"
USE_GAZEBO_GUI="${USE_GAZEBO_GUI:-false}"
# USE_RVIZ 默认true——之前雷达在Gazebo里开射线可视化(<visualize>true</visualize>)
# 会拖慢物理循环导致PX4 poll timeout（早就改成false了，见gen_iris_mid360_sdf.py），
# 但"看点云"这个需求本身还在。用户建议：Gazebo里不显示射线，改用RViz2单独看
# 点云/建图——RViz2只管渲染，不在物理仿真的主循环里，不会重蹈之前的覆辙。
# rviz/multi_mighty.rviz这份配置已经按NX01/NX02两机命名空间配好了，双机场景
# 正好能用。跟gzclient共用同一套DISPLAY/X11转发（见docker-compose.yml里
# sim-world服务的DISPLAY环境变量+/tmp/.X11-unix挂载），不需要额外配置。
USE_RVIZ="${USE_RVIZ:-true}"
PLANNER="${PLANNER:-mighty}"
if [ "${PLANNER}" = "ego_planner" ]; then
    RVIZ_CONFIG="multi_ego_planner.rviz"
else
    RVIZ_CONFIG="multi_mighty.rviz"
fi

echo "== [sim-world] 启动Gazebo世界 env=${WORLD_ENV} rviz_config=${RVIZ_CONFIG} =="
ros2 launch mighty base_mighty.launch.py \
    env:="${WORLD_ENV}" use_gazebo_gui:="${USE_GAZEBO_GUI}" \
    use_rviz:="${USE_RVIZ}" rviz_config:="${RVIZ_CONFIG}" &
GAZEBO_PID=$!

# 等Gazebo服务起来再spawn飞机，避免spawn_entity在服务未就绪时报错
sleep 8

# ------------------------------------------------------------------
# 依次为每架飞机: (a) spawn 一个"PX4官方iris(带mavlink_interface插件+电机，真正能
#                    被PX4控制) + Mid-360雷达/IMU"融合模型进Gazebo
#                (b) 起一个PX4 SITL实例连到这个模型（经典Gazebo插件走UDP本地端口）
# 端口/命名规则照抄PX4官方 sitl_multiple_run.sh 的模式：
#   实例i -> MAV_SYS_ID=i+1, 各类端口在官方脚本里按 i 做偏移
#
# 原来这里spawn的是mighty自己的quadrotor.urdf.xacro——只有雷达+IMU的传感器载具，
# 没有libgazebo_mavlink_interface.so插件、没有电机模型，PX4 SITL因此永远连不上
# 任何仿真物理实体（日志卡在"Waiting for simulator to accept connection on TCP
# port 4560"不动，MAVROS一直connected:false）。真正带这个插件的是PX4官方自己的
# iris系列模型，照着PX4官方iris_rplidar那个"iris+外挂传感器"组合模型的先例，用
# docker/scripts/gen_iris_mid360_sdf.py把Mid-360焊到iris上，生成一份纯SDF、
# 每实例独立端口的融合模型，直接用spawn_entity.py的-file参数spawn（不再走
# robot_description/xacro这条路——那条路径现在只在flight-stack容器里给
# robot_state_publisher发布TF用，跟这边spawn进Gazebo的实际物理实体是两回事，
# 互不影响）。
# `gz model --spawn-file`这个Gazebo原生命令实测会莫名挂起几分钟不返回（怀疑是
# 在等待访问外网的models.gazebosim.org超时，这台机器没有稳定外网），改用
# `ros2 run gazebo_ros spawn_entity.py -file`这条路，实测正常（几十秒内完成，
# iris模型本身比mighty原来的纯传感器模型复杂不少，比原来慢一些是正常的）。
#
# 之前spawn和起PX4写在同一个循环里、逐架飞机交替进行：NX01的PX4实例一起来就
# 开始按lockstep协议高频轮询传感器数据，但紧接着立刻又在gzserver主线程里做
# NX02那份重活（渲染jinja、spawn_entity同步RPC调用、LivoxPointsPlugin加载
# 80万行的mid360.csv，实测耗时数秒）——这几秒物理循环被这些重活占住，NX01
# 那边的轮询在这个窗口里连续超时（`ERROR [simulator_mavlink] poll timeout`），
# 而且这个lockstep一旦在启动阶段错位就再也回不来了（哪怕后面几分钟world完全
# 空闲，NX01的日志也只会一直刷poll timeout，永远到不了EKF/Ready for
# takeoff那一步；MAVROS那边`connected`会显示true，因为心跳包走的是低频独立
# 通道，但`system_status`会一直卡在0=UNINIT，飞控实际上没真正起来）。而
# NX02因为是最后一个spawn的，它起PX4轮询的时候后面已经没有别的重活跟它抢，
# 所以从来没复现过这个问题——两架飞机表现不一致，正是这个先后顺序竞态的
# 特征。
# 改成两阶段：先把所有飞机的模型全部spawn完（重活集中在这一阶段做完，此时
# 还没有任何PX4实例在轮询），全部spawn完、留一点时间让物理稳定下来之后，
# 再统一起所有PX4实例——这样任何一个PX4实例开始轮询时，都不会再有别的
# 飞机在做spawn这种重活。
# ------------------------------------------------------------------
declare -a AGENT_NS
for i in $(seq 1 "${NUM_AGENTS}"); do
    NS=$(printf "NX%02d" "$i")
    AGENT_NS[$i]="${NS}"
    echo "== [sim-world] 生成+spawn ${NS} 的 iris+mid360 融合模型 (实例号 $((i-1))) =="
    IRIS_MID360_SDF="/tmp/iris_mid360_${NS}.sdf"
    python3 /opt/docker_scripts/gen_iris_mid360_sdf.py \
        --namespace "${NS}" --instance $((i-1)) --output "${IRIS_MID360_SDF}"
    # z=0.1：之前这里写的是z=3（3米悬空），配合mighty_enable_gravity.patch打开的
    # 重力，飞机在没解锁（电机不转）的情况下直接自由落体砸到地面——实测确认过
    # （mavros/local_position/pose里z从3掉到了-0.09，armed:false）。第一次改成了
    # PX4官方sitl_multiple_run.sh里spawn_model()自己用的0.83米，但0.83米对这个
    # 具体的iris模型来说依然是悬空的、还是会往下摔一截——查了iris.sdf.jinja，
    # base_link_inertia_collision是个以base_link原点为中心的0.47x0.47x0.11米
    # 箱体，箱底在原点下方0.055米，也就是说z=0.055才是箱体刚好贴地、零净空的
    # 高度。改成z=0.1（比这个零净空高度多留1.5厘米，肉眼几乎看不出下落），
    # 让飞机像真机一样"停在地上"等解锁起飞，而不是从半空/近半空开始物理仿真。
    ros2 run gazebo_ros spawn_entity.py \
        -file "${IRIS_MID360_SDF}" -entity "${NS}" \
        -x "$((i*3))" -y "0" -z "0.1" -Y "0"
done

echo "== [sim-world] 全部模型spawn完毕，等物理稳定后再统一起PX4 SITL实例 =="
sleep 3

for i in $(seq 1 "${NUM_AGENTS}"); do
    NS="${AGENT_NS[$i]}"
    echo "== [sim-world] 启动 PX4 SITL 实例 $((i-1)) for ${NS} =="
    (
        cd /opt/PX4-Autopilot
        PX4_SIM_MODEL=gazebo-classic_iris \
        PX4_SIMULATOR=gazebo-classic \
        ./build/px4_sitl_default/bin/px4 \
            -i $((i-1)) \
            -d "$PWD/ROMFS/px4fmu_common" \
            >/tmp/px4_${NS}.log 2>&1
    ) &
    sleep 2
done

echo "== [sim-world] 启动 UWB 真值模拟节点（读取双机ground truth，输出frame_align）=="
ros2 run uwb_sim uwb_ground_truth_node \
    --ros-args -p num_agents:="${NUM_AGENTS}" -p namespace_prefix:=NX \
        -p range_noise_std:=0.05 -p latency_ms:=30.0 &

wait ${GAZEBO_PID}
