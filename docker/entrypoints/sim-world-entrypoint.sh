#!/usr/bin/env bash
# sim-world 容器入口：起 Gazebo(装了双机模型+world) + N个PX4 SITL实例 + UWB真值节点。
# 环境变量：
#   WORLD_ENV       默认 fire_drill_room（大赛任务场景，2026-09-09起改的默认值，
#                   原默认hard_forest仍可用，见下方WORLD_ENV赋值行注释）
#   NUM_AGENTS      默认 2
#   USE_GAZEBO_GUI  默认 false（容器里没有桌面就别开GUI，除非你转发了X11）
#   USE_RVIZ        默认 true，起RViz2看点云/建图（NX01/NX02双机命名空间的
#                   multi_mighty.rviz配置），跟gzclient一样靠DISPLAY/X11转发
#   PLANNER         默认 ego_planner（2026-08-10改，原默认mighty，传
#                   PLANNER=mighty可恢复旧默认），决定用哪份rviz配置——sim-world容器本身跟
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
# contest_mission（2026大赛任务系统专属包，2026-09-08新增）——这个
# 容器只用得到里面的mission_judge_node/scenario_reset_node两个节点
# （阶段7.2/7.3），不source的话`ros2 run contest_mission ...`会报
# "package not found"，参照flight-stack-entrypoint.sh那边
# source_all.sh漏加这个workspace踩过的坑（见DEBUG_JOURNAL.md
# 2026-09-08"阶段3"条目），这次直接在Dockerfile构建阶段COPY时就
# 一起把source加上，不留同样的坑给以后。
source /opt/contest_mission_ws/install/setup.bash
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

# 2026-09-09改默认值hard_forest→fire_drill_room，跟docker-compose.yml里
# WORLD_ENV那行的默认值改动保持一致（原因见那边注释：fire_drill_room是
# 正式比赛场景，不该是"没传就悄悄变成别的场景"）——这里的默认值只有
# 绕过compose直接`docker run`这个镜像、且没显式传WORLD_ENV时才会用到。
WORLD_ENV="${WORLD_ENV:-fire_drill_room}"
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
PLANNER="${PLANNER:-ego_planner}"
if [ "${PLANNER}" = "ego_planner" ]; then
    RVIZ_CONFIG="multi_ego_planner.rviz"
    # ego-planner-swarm自己的Readme.md写明FastDDS(ROS2默认)会导致明显卡顿、
    # 建议换cyclonedds——跟flight-stack-entrypoint.sh联动切换（那边有更完整
    # 的实测依据说明），同一个ROS_DOMAIN_ID下所有参与者必须用同一个RMW实现
    # 才能互相发现，这里必须在下面`ros2 launch`（起Gazebo+PX4 SITL）之前
    # 设置，不能只改flight-stack那两个容器。
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "== [sim-world] PLANNER=ego_planner，切换RMW_IMPLEMENTATION=rmw_cyclonedds_cpp =="

    # 跟flight-stack-entrypoint.sh同一个坑：network_mode:host把宿主机
    # docker0/vmnet1/vmnet8这些跟ROS2无关的虚拟网卡也暴露给了容器，
    # CycloneDDS默认在全部网卡上发组播，日志被"ddsi_udp_conn_write ...
    # failed"刷屏，是mavros/imu/data实测速率上不去的真正原因之一（另一个是
    # PX4固件50Hz限速，已用px4_onboard_imu_rate_firmware_default.patch修过）。
    # 三个容器共享同一个host网络命名空间，锁定成lo环回即可互相发现，两边
    # 必须用同一份配置，否则网卡范围不一致会导致互相发现失败。
    export CYCLONEDDS_URI="file:///tmp/docker_sim_cyclonedds.xml"
    cat > /tmp/docker_sim_cyclonedds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="true"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Internal>
      <!-- 2026-09-22：图像一帧约900KB，默认socket接收缓冲只有208KB，仿真把CPU压满时
           分片来不及取就被内核丢掉（/proc/net/snmp 的 RcvbufErrors 实测10秒涨1731次），
           一帧缺一片就整帧作废，表现为"某一路相机收不到"。用 max 不用 min：
           max 是"尽量申请这么大、内核给不到就用它能给的最大值"，min 是硬性要求、
           给不到DDS直接起不来。真正生效还需要宿主机 net.core.rmem_max 够大。 -->
      <SocketReceiveBufferSize max="16MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
EOF
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
declare -a RESET_NS_LIST RESET_X_LIST RESET_Y_LIST RESET_Z_LIST RESET_YAW_LIST
for i in $(seq 1 "${NUM_AGENTS}"); do
    NS=$(printf "NX%02d" "$i")
    AGENT_NS[$i]="${NS}"
    echo "== [sim-world] 生成+spawn ${NS} 的 iris+mid360 融合模型 (实例号 $((i-1))) =="
    IRIS_MID360_SDF="/tmp/iris_mid360_${NS}.sdf"
    # 2026大赛任务系统阶段2.1：挂前视/下视相机——2026-09-08用户明确要求
    # 不分角色，NX01/NX02（含以后任何NX0N）都统一挂前视+下视两路（之前
    # 按"NX01侦察机/NX02行动机"分工只给NX01挂前视的版本已改掉）。跟上面
    # SPAWN_YAW_DEG_${NS}同一套写法，支持`CAMERAS_${NS}`环境变量覆盖默认值。
    # 2026-09-16临时诊断：把顺序从"front,down"反过来，验证"front每次都
    # 赢渲染竞争"是不是因为它在gen_iris_mid360_sdf.py生成的SDF里排在
    # down前面（Gazebo SensorManager疑似按传感器加载顺序处理共享渲染
    # 队列）——如果赢的变成down，就实锤是排序问题，不是down这个挂载
    # 位置本身有什么问题。验证完记得决定要不要改回来/换成别的最终方案。
    CAMERAS_DEFAULT="down,front"
    _cameras_var="CAMERAS_${NS}"
    # 2026-09-17：`:-`在变量被显式设成空字符串时也会触发默认值替换（bash
    # 这条规则跟"没设置"混为一谈），这里故意改用单杠`-`（只在变量真的
    # 没设置/unset时才用默认值，显式设成空字符串会保留空字符串）——
    # NX02现在要传CAMERAS_NX02=""表示"完全不挂相机"，用`:-`的话这个空
    # 字符串会被悄悄替换回CAMERAS_DEFAULT，等于覆盖失效，见
    # docker-compose.yml里CAMERAS_NX02的完整说明。
    CAMERAS="${!_cameras_var-${CAMERAS_DEFAULT}}"
    # 2026-09-18：默认相机方案改成'switchable'——单个真实相机+可动关节，
    # 在前视/下视两个预设角度间动态切换（见gen_iris_mid360_sdf.py里
    # merge_switchable_camera()的说明+DEBUG_JOURNAL.md 2026-09-18记录）。
    # 彻底避开Gazebo Classic多相机共享渲染队列这个联网核实过的架构限制
    # （不是调参数能缓解的，只能从根上避免同时存在多个真实image传感器）。
    # 旧的'camera'（独立前视/下视传感器）方案保留代码，不再是默认值，
    # 需要时仍可用CAMERA_TYPE_${NS}显式指回去。'logical_camera'方案
    # （纯几何视锥判断，不做图像渲染）2026-09-18排查了一整天，确认
    # `libgazebo_ros_logical_camera.so`插件在gzserver进程内publish()
    # 之后消息完全送不到任何订阅者（跨容器/单容器/单机/多机、1个或2个
    # 传感器都复现，独立进程用同样消息类型+QoS完全正常，"挪到主线程
    # 发布"这个修复也验证过无效），根因未定位到，用户决定放弃这条路、
    # 相关代码已删除，见DEBUG_JOURNAL.md 2026-09-17/09-18记录。
    CAMERA_TYPE_DEFAULT="switchable"
    _camera_type_var="CAMERA_TYPE_${NS}"
    CAMERA_TYPE="${!_camera_type_var-${CAMERA_TYPE_DEFAULT}}"
    # 2026-09-18新增：仅camera_type=switchable时生效——这架飞机spawn出来
    # 那一刻，唯一那个相机关节的初始朝向。用户明确要求NX01初始化为下视、
    # NX02初始化为前视，两个值都硬编码在这里（不是随便挑的默认值，是
    # 明确的产品需求），可以用CAMERA_INITIAL_VIEW_${NS}环境变量覆盖。
    if [ "${NS}" = "NX01" ]; then
        CAMERA_INITIAL_VIEW_DEFAULT="down"
    else
        CAMERA_INITIAL_VIEW_DEFAULT="front"
    fi
    _camera_initial_view_var="CAMERA_INITIAL_VIEW_${NS}"
    CAMERA_INITIAL_VIEW="${!_camera_initial_view_var-${CAMERA_INITIAL_VIEW_DEFAULT}}"
    python3 /opt/docker_scripts/gen_iris_mid360_sdf.py \
        --namespace "${NS}" --instance $((i-1)) --output "${IRIS_MID360_SDF}" \
        --cameras "${CAMERAS}" --camera-type "${CAMERA_TYPE}" \
        --camera-initial-view "${CAMERA_INITIAL_VIEW}"
    # z=0.1：之前这里写的是z=3（3米悬空），配合mighty_enable_gravity.patch打开的
    # 重力，飞机在没解锁（电机不转）的情况下直接自由落体砸到地面——实测确认过
    # （mavros/local_position/pose里z从3掉到了-0.09，armed:false）。第一次改成了
    # PX4官方sitl_multiple_run.sh里spawn_model()自己用的0.83米，但0.83米对这个
    # 具体的iris模型来说依然是悬空的、还是会往下摔一截——查了iris.sdf.jinja，
    # base_link_inertia_collision是个以base_link原点为中心的0.47x0.47x0.11米
    # 箱体，箱底在原点下方0.055米，也就是说z=0.055才是箱体刚好贴地、零净空的
    # 高度。改成z=0.1（比这个零净空高度多留1.5厘米，肉眼几乎看不出下落），
    # 让飞机像真机一样"停在地上"等解锁起飞，而不是从半空/近半空开始物理仿真。
    # 两机各给一个不同的非零spawn yaw——之前两机spawn yaw全是0，局部系跟
    # 全局系数值上重合，情景二(SLAM本地yaw是否需要旋转对齐)这条验证链路
    # 一直没有真实的非零偏移可用来检验，见`docker_sim/多机空间坐标对齐
    # 问题_设计讨论纪要.docx`第七节。2026-08-12用户明确要求把NX01从45度改
    # 成30度、NX02从0度改成-45度——两机现在方向不同（一正一负），且不再
    # 有任何一架维持0度，origin_setter_node的θ*在线估计对两机都是真正
    # 非平凡的检验，不会有"其中一架岁月静好、只测了另一架"这种覆盖不全的
    # 死角。
    # 2026-09-03：默认值不变（NX01=30°、NX02=-45°，但改成可以用环境变量
    # `SPAWN_YAW_DEG_<NS>`按度覆盖——SE(2)标定论文要做"真值角度全域扫描"
    # 实验(E12：0/15/30/60/90/135/180/-45度)，验证估计器在全角域没有符号或
    # 象限错误，写死在这里就没法扫。用度不用弧度是因为实验脚本/命令行里
    # 人读人写的都是度，弧度那串小数抄错一位很难发现。
    # 2026-09-09用户重新设计场景：WORLD_ENV=fire_drill_room时，NX01/
    # NX02起降点合一、固定坐标+机头朝向Y轴正方向(yaw=90°)，跟
    # src/contest_mission/config/fire_drill_room_layout.yaml的
    # takeoff_landing_pads一致（这两处是独立维护的同一份数字，yaml是给
    # ROS节点/文档用的权威定义，这里是shell脚本要用的字面量副本，改
    # 布局时两处都要改）。**只在这个特定场景下生效**，不影响其它
    # WORLD_ENV（尤其是simple_room用到的下面SPAWN_YAW_DEG_<NS>那套
    # SE(2)标定论文专用的偏航角全域扫描机制，两者是完全独立的模块，
    # 不能因为这次场景重新设计就动了那套机制的默认行为）。
    if [ "${WORLD_ENV}" = "fire_drill_room" ]; then
        if [ "${NS}" = "NX01" ]; then
            SPAWN_X="1.5"
        else
            SPAWN_X="-1.5"
        fi
        SPAWN_Y="-10.0"
        SPAWN_Z="0.1"
        SPAWN_YAW_DEG="90"
        SPAWN_YAW=$(awk -v d="${SPAWN_YAW_DEG}" 'BEGIN{printf "%.16f", d*atan2(0,-1)/180}')
    else
        SPAWN_X="$((i*3)).0"
        SPAWN_Y="0.0"
        SPAWN_Z="0.1"
        SPAWN_YAW_DEG_DEFAULT="0"
        if [ "${NS}" = "NX01" ]; then
            SPAWN_YAW_DEG_DEFAULT="30"
        elif [ "${NS}" = "NX02" ]; then
            SPAWN_YAW_DEG_DEFAULT="-45"
        fi
        _yaw_var="SPAWN_YAW_DEG_${NS}"
        SPAWN_YAW_DEG="${!_yaw_var:-${SPAWN_YAW_DEG_DEFAULT}}"
        SPAWN_YAW=$(awk -v d="${SPAWN_YAW_DEG}" 'BEGIN{printf "%.16f", d*atan2(0,-1)/180}')
    fi
    echo "== [sim-world] ${NS} spawn yaw = ${SPAWN_YAW_DEG}° (${SPAWN_YAW} rad) =="
    ros2 run gazebo_ros spawn_entity.py \
        -file "${IRIS_MID360_SDF}" -entity "${NS}" \
        -x "${SPAWN_X}" -y "${SPAWN_Y}" -z "${SPAWN_Z}" -Y "${SPAWN_YAW}"

    # 2026大赛任务系统阶段7.3：把这架机真正spawn时用的坐标记下来，
    # 传给scenario_reset_node当"重置回起飞点"的目标——跟spawn_entity.py
    # 那行用的是完全同一份变量，不是单独维护的一份数字，两处不会drift。
    RESET_NS_LIST+=("${NS}")
    RESET_X_LIST+=("${SPAWN_X}")
    RESET_Y_LIST+=("${SPAWN_Y}")
    RESET_Z_LIST+=("${SPAWN_Z}")
    RESET_YAW_LIST+=("${SPAWN_YAW}")
done

echo "== [sim-world] 全部模型spawn完毕，等物理稳定后再统一起PX4 SITL实例 =="
sleep 3

# 2026大赛任务系统阶段7.2/7.3：自动裁判+场景重置，两个都归属sim-world
# 容器（需要读写Gazebo模型状态的权限，见设计方案5.3容器归属表），不
# 挂在任何一架飞机的命名空间下面——是服务于整个场景的全局节点，不是
# per-drone节点。agent_ns/agent_reset_x/y/z/yaw四个参数直接复用上面
# spawn循环里记录的RESET_*_LIST（同一份坐标，不是重新算一遍）。
_join_by_comma() { local IFS=','; echo "$*"; }
AGENT_NS_CSV=$(_join_by_comma "${RESET_NS_LIST[@]}")
RESET_X_CSV=$(_join_by_comma "${RESET_X_LIST[@]}")
RESET_Y_CSV=$(_join_by_comma "${RESET_Y_LIST[@]}")
RESET_Z_CSV=$(_join_by_comma "${RESET_Z_LIST[@]}")
RESET_YAW_CSV=$(_join_by_comma "${RESET_YAW_LIST[@]}")

echo "== [sim-world] 启动 mission_judge_node（阶段7.2自动裁判，只读Gazebo真值） =="
ros2 run contest_mission mission_judge_node \
    --ros-args -p drone_namespaces:="[${AGENT_NS_CSV}]" &

echo "== [sim-world] 启动 scenario_reset_node（阶段7.3场景重置，agents=${AGENT_NS_CSV}） =="
ros2 run contest_mission scenario_reset_node \
    --ros-args \
    -p agent_ns:="[${AGENT_NS_CSV}]" \
    -p agent_reset_x:="[${RESET_X_CSV}]" \
    -p agent_reset_y:="[${RESET_Y_CSV}]" \
    -p agent_reset_z:="[${RESET_Z_CSV}]" \
    -p agent_reset_yaw:="[${RESET_YAW_CSV}]" &

# 2026-09-09用户要求：容器启动就自动触发一次reset_scenario，不用每次都
# 手动`ros2 service call`——不然重启后立柱永远是world文件里写死的默认
# 朝向（yaw=0，轴对齐），只有手动调用过一次service才会随机转。放后台跑，
# 不阻塞后面PX4 SITL实例的启动。`ros2 service call`本身只会等
# `/scenario_reset_node/reset_scenario`这个service被发现/可调用，等不
# 到它内部`set_state_cli`/`judge_reset_cli`两个client是不是已经连上各自
# 目标服务（`/plug/set_entity_state`、`mission_judge_node/
# reset_checkpoints`）——先sleep 5秒给这两个client留出发现时间，不然
# service调用本身"成功"了，但`_set_model_pose`内部`service_is_ready()`
# 检查为false直接跳过，立柱位置根本没被真的设置。
(
    sleep 5
    echo "== [sim-world] 自动触发一次reset_scenario（随机立柱朝向）=="
    ros2 service call /scenario_reset_node/reset_scenario std_srvs/srv/Trigger "{}"
) &

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

echo "== [sim-world] 启动 UWB 真值模拟节点（读取双机ground truth，输出frame_align+各机/uwb/pose_abs绝对位置，供uwb_origin_bridge起飞点一键锁定使用）=="
# 2026-08-12：publish_rate_hz从默认10Hz显式提到50Hz——LOCALIZATION_SOURCE=
# uwb_imu模式下，uwb_imu_fusion_node喂给PX4的yaw/位置更新率天然被这个UWB
# 话题的发布率摁住(IMU那一路roll/pitch更新快，但yaw/位置只能跟着UWB走)，
# 是整条反馈链路里最原始、最低的一环——实测px4ctrl/so3ctrl在uwb_imu模式下
# 失控炸机(见DEBUG_JOURNAL.md)，先把这个最上游的速率提上去看能不能改善，
# 跟已经做过的MAVLink固件速率patch(px4_onboard_position_rate_firmware_
# default.patch)是同一个方向的尝试，这次是从数据源头这一端提速，不是从
# PX4/MAVLink那一端。
# PUBLISH_FRAME_ALIGN（默认true，环境变量）——设成false关掉这个节点的
# /frame_align/*真值发布，只在专门要验证"不靠仿真真值，光靠
# origin_setter_node的标定结果也能对齐"时才需要设false，这条真值发布留着
# 会跟flight-stack容器里frame_align_bridge_node发的真实计算结果抢同一个
# 话题（见uwb_ground_truth_node.py里publish_frame_align参数的注释）。
# 2026-08-12之前这个开关是从LOCALIZATION_SOURCE=uwb_slam自动推导的，现在
# LOCALIZATION_SOURCE的"dlio"/"uwb_slam"两个值已经合并（两者在flight-stack
# 那边本来就是完全一样的行为，拆开纯粹是历史包袱，见DEBUG_JOURNAL.md
# 2026-08-12"评估LOCALIZATION_SOURCE功能重复"那条），"要不要用仿真真值做
# frame_align"这个正交的维度改成独立环境变量，不再耦合在LOCALIZATION_SOURCE
# 这一个开关上。
# 2026-09-03新增的一批参数（SE(2)标定论文补充实验用，见
# `src/uwb_sim/uwb_sim/uwb_ground_truth_node.py`文件头说明）：默认值全部保持
# 原有行为(不注入污染、不限速、不延迟)，只有做实验时才通过环境变量打开。
#   UWB_NOISE_STD       绝对观测高斯噪声标准差[m]（论文表1的σ）
#   UWB_BIAS_X/Y        恒定偏置[m]，验证论文2.4节的偏置免疫性
#   UWB_BIAS_DRIFT_X/Y  偏置漂移率[m/s]，构造论文6.2节的"时变偏置"场景
#   UWB_OUTLIER_PROB    每帧被多径/NLOS污染的概率，验证2.7节Huber抗差
#   UWB_OUTLIER_MAG_M   野值幅度[m]
#   UWB_ABS_RATE_HZ     绝对观测发布率上限[Hz]，0=不限速（默认，实际跟着
#                       model_states走，约100Hz仿真时间）
#   UWB_ABS_LATENCY_MS  绝对观测时延[ms]，0=不延迟（默认）
#   UWB_PUBLISH_TRUTH   是否发/{ns}/uwb/pose_truth真值旁路（评估用，默认true）
ros2 run uwb_sim uwb_ground_truth_node \
    --ros-args -p num_agents:="${NUM_AGENTS}" -p namespace_prefix:=NX \
        -p range_noise_std:="${UWB_NOISE_STD:-0.05}" \
        -p latency_ms:=30.0 -p publish_rate_hz:=50.0 \
        -p publish_frame_align:="${PUBLISH_FRAME_ALIGN:-true}" \
        -p publish_truth_pose:="${UWB_PUBLISH_TRUTH:-true}" \
        -p abs_bias_xy:="[${UWB_BIAS_X:-0.0}, ${UWB_BIAS_Y:-0.0}]" \
        -p abs_bias_drift_mps:="[${UWB_BIAS_DRIFT_X:-0.0}, ${UWB_BIAS_DRIFT_Y:-0.0}]" \
        -p abs_outlier_prob:="${UWB_OUTLIER_PROB:-0.0}" \
        -p abs_outlier_mag_m:="${UWB_OUTLIER_MAG_M:-0.0}" \
        -p abs_pose_rate_hz:="${UWB_ABS_RATE_HZ:-0.0}" \
        -p abs_pose_latency_ms:="${UWB_ABS_LATENCY_MS:-0.0}" &

wait ${GAZEBO_PID}
