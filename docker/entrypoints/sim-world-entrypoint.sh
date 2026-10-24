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

# 2026-09-09改默认值hard_forest→fire_drill_room，2026-09-30再改成sample_room，
# 都跟docker-compose.yml里WORLD_ENV那行的默认值保持一致（原因见那边注释：
# 默认值必须是当前的正式比赛场景，不该是"没传就悄悄变成别的场景"）——这里的
# 默认值只有绕过compose直接`docker run`这个镜像、且没显式传WORLD_ENV时才用到。
WORLD_ENV="${WORLD_ENV:-sample_room}"
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

# ---- 样题场景：起 Gazebo 之前先把高层着火点抽定、重新生成 world ----
# 为什么在这里抽、而不是等 scenario_reset_node 运行时抽（2026-09-30）：
# 那个节点是在 Gazebo 起来 5 秒后用 SetEntityState 把 tag **热移**过去的，而
# gzclient 对运行中被热改位姿的模型**不会重绘**（DEBUG_JOURNAL 2026-09-15 记过）
# ——界面上 tag 还停在 world 写死的位置，跟飞机相机实际看到的不是一回事。
# 用户据此判断"火情在 2#"，而 Gazebo 查出来的真值是 1#。界面骗人比少一点随机性
# 危险得多，所以改成生成 world 时就抽定：静态 world 里就是随机结果，界面所见
# 即真值；抽中的值同时写回 layout yaml 并关掉随机开关，scenario_reset_node
# 照着同一份 yaml 摆回同一个位置，两边不会再分叉。
# 注意写回的是**容器内**那份（镜像里的副本），每次 compose up 都是新容器、
# 从镜像重新展开，所以下一轮照样会重新抽；只有 `docker compose restart`
# 复用同一个容器文件系统时位置才会保持不变。
if [ "${WORLD_ENV}" = "sample_room" ]; then
    _LAYOUT="$(ros2 pkg prefix contest_mission 2>/dev/null)/share/contest_mission/config/sample_room_layout.yaml"
    _WORLD="/opt/mighty_ws/install/mighty/share/mighty/worlds/sample_room.world"
    # 2026-10-23：场地布置表支持从宿主机挂载覆盖（compose 里的
    # ./src/contest_mission/config:/layout_override:ro）。选手改宿主机那份、
    # 重启容器即生效，不用重建 sim-world 镜像。
    # 为什么是"复制进 install"而不是直接挂到 install 上：
    #   ① 下面的生成器带 --write-resolved-layout，会把随机抽定的火情位置写回
    #      这个路径——挂载是 :ro，直接挂会写失败；挂成可写又会改到选手本地文件。
    #   ② 复制进去之后，scenario_reset_node 等按 ros2 pkg prefix 找表的节点
    #      一个字都不用改。
    #   ③ 每次都从挂载的原始表重新开始，不会让上一轮写回的抽定结果越积越多。
    _OVERRIDE=/layout_override/sample_room_layout.yaml
    if [ -f "${_OVERRIDE}" ] && [ -f "${_LAYOUT}" ]; then
        if cp "${_OVERRIDE}" "${_LAYOUT}" 2>/dev/null; then
            echo "== [sim-world] 场地布置表用挂载那份：${_OVERRIDE} =="
        else
            echo "!! 挂载的场地布置表复制失败，沿用镜像里那份 !!" >&2
        fi
    fi
    if [ -f "${_LAYOUT}" ] && [ -f /opt/docker_scripts/gen_sample_room_world.py ]; then
        echo "== [sim-world] 抽定高层着火点并重新生成 sample_room.world =="
        python3 /opt/docker_scripts/gen_sample_room_world.py \
            --layout "${_LAYOUT}" --output "${_WORLD}" \
            --randomize-fire --write-resolved-layout "${_LAYOUT}" \
            || echo "!! 重新生成 sample_room.world 失败，沿用镜像里那份（火情位置固定） !!" >&2
    else
        echo "!! 找不到 layout 或生成脚本，沿用镜像里的 sample_room.world !!" >&2
    fi
fi

# ---------------------------------------------------------------------------
# 把 gzserver 绑到大核（2026-10-16）
# ---------------------------------------------------------------------------
# 为什么要绑：gzserver 是单线程瓶颈，实时因子直接取决于它那一个线程的速度。
# Intel 混合架构（12 代起的 HX/H 系列）上调度器可能把它甩到 E 核，单线程性能
# 只有 P 核的 55~65%，实时因子就掉下来——而本项目**所有超时都是墙钟**
# （SDK 的 _poll_until 用 time.monotonic），实时因子掉一半等于把所有超时预算
# 砍一半，表现成"卡顿 + 逻辑出错"（goto 判不到点、精降没降完就交棒、事件等不到）。
#
# 核号**不写死**：内核在混合架构上直接给出分组，/sys/devices/cpu_core/cpus
# 就是 P 核列表（i9-14900HX 上是 0-15，i7-14650HX 上会是别的范围），所以这套
# 系统克隆到任何一台机器都能自己找对大核。非混合架构（AMD、老 Intel）没有这个
# 文件，那时不绑——它们本来就没有大小核之分。
bind_gzserver_to_pcores() {
    local pcores pid n=0
    pcores="$(cat /sys/devices/cpu_core/cpus 2>/dev/null || true)"
    if [ -z "$pcores" ]; then
        echo "== [sim-world] 非 Intel 混合架构（没有 cpu_core/cpus），gzserver 不绑核 =="
        return 0
    fi
    # 等 gzserver 起来。用 pgrep -x（精确匹配**进程名**）不是 -f——后者会把
    # 这条命令自己也匹配进去（踩过三次，见 feedback_avoid_self_matching_pkill）。
    while [ "$n" -lt 90 ]; do
        # ⚠️ 必须有 `|| true`：本脚本开头是 `set -eo pipefail`，gzserver 还没起来
        # 时 pgrep 返回 1，赋值语句的退出码就是它 —— **整个后台子 shell 当场静默
        # 退出**，一行日志都不会留（2026-10-16 实测就是这么失效的，手动跑同一条
        # taskset 却成功，白查了一轮）。
        pid="$(pgrep -x gzserver 2>/dev/null | head -1 || true)"
        [ -n "$pid" ] && break
        sleep 1; n=$((n + 1))
    done
    if [ -z "$pid" ]; then
        echo "== [sim-world] 90 秒没等到 gzserver 进程，跳过绑核 =="
        return 0
    fi
    # -a 连它的所有线程一起绑（gzserver 有物理引擎/传感器等多个线程）。
    if taskset -apc "$pcores" "$pid" >/dev/null 2>&1; then
        echo "== [sim-world] gzserver(PID $pid) 已绑到大核 CPU $pcores"
        echo "   （E 核 $(cat /sys/devices/cpu_atom/cpus 2>/dev/null || echo '?') 留给其它容器） =="
    else
        echo "== [sim-world] taskset 失败（缺 util-linux 或权限不足），gzserver 未绑核 =="
    fi
}
bind_gzserver_to_pcores &

echo "== [sim-world] 启动Gazebo世界 env=${WORLD_ENV} rviz_config=${RVIZ_CONFIG} =="
# ⚠️ 2026-10-24 已回退一个失败的尝试，别再想同一个点子：
#   10-23 怀疑"飞机视觉位置跟物理位置对不上、机身和桨叶分家"是 gzclient 起太早
#   造成的——spawn 在 Gazebo 起来 8 秒后才开始，gzclient 若此刻已连上，整个
#   spawn 过程它都在边上接收模型插入消息，赶上哪一步算哪一步。于是改成
#   launch 一律按 use_gazebo_gui:=false 起，等 spawn + reset_scenario 全部走完
#   再用 gzclient.launch.py 单独拉起来。
#   **结果：无效。** 推迟之后照样错位（日志确认推迟真的生效了），纯粹是净退化
#   ——Gazebo 窗口白白晚几十秒出现。10-24 回退回"跟 gzserver 一起起"。
#   这个错位问题至今未解决，但它**只影响 gzclient 画面，不影响任何功能**：
#   model_states_plug/link_states_plug 实测各 link 紧贴机体，pose/info 的位姿流
#   与 SDF 逐位吻合，飞行结果/监视器/日志/照片全部可信。
#   已否证的 9 个假设和唯一没查的方向（<include> 嵌套子模型 gps0/lidar），
#   记在记忆 project_gazebo_visual_desync 里，排查前先看那份，别重复劳动。
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
    # 2026-09-29用户要求：两架飞机在Gazebo里一眼要能分清——NX01黄、NX02红。
    # 只染机身，四个旋翼保持iris原版的Blue/DarkGrey交替（那是"哪两个是前桨"的
    # 方向指示，染了反而看不出机头朝向），雷达整流罩也不动。
    # 材质名取自容器里 /usr/share/gazebo-11/media/materials/scripts/gazebo.material，
    # 可以用BODY_COLOR_${NS}覆盖；设成空字符串就是保持PX4原版的DarkGrey。
    # 配色沿革：黄/红 -> 橙/红（NX01跟当时的立柱同色）-> 红/青 -> 红/黑（本次）。
    # 2026-09-29用户定稿：NX01红、NX02黑。
    if [ "${NS}" = "NX01" ]; then
        BODY_COLOR_DEFAULT="Gazebo/Red"
    else
        BODY_COLOR_DEFAULT="Gazebo/Black"
    fi
    _body_color_var="BODY_COLOR_${NS}"
    BODY_COLOR="${!_body_color_var-${BODY_COLOR_DEFAULT}}"
    python3 /opt/docker_scripts/gen_iris_mid360_sdf.py \
        --namespace "${NS}" --instance $((i-1)) --output "${IRIS_MID360_SDF}" \
        --cameras "${CAMERAS}" --camera-type "${CAMERA_TYPE}" \
        --camera-initial-view "${CAMERA_INITIAL_VIEW}" \
        --body-color "${BODY_COLOR}"
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
    # 2026-09-29 样题版场景（WORLD_ENV=sample_room）：坐标系原点在房间**西南角**，
    # 所以起降点是全正坐标 (8,3)/(12,3)，跟 fire_drill_room 的 (±2,-9.5) 不是一套
    # 数。权威定义在 src/contest_mission/config/sample_room_layout.yaml 的
    # takeoff_landing_pads，这里同样是那份数字的 shell 字面量副本，改布局两处都要改。
    if [ "${WORLD_ENV}" = "sample_room" ]; then
        if [ "${NS}" = "NX01" ]; then
            SPAWN_X="8.0"
        else
            SPAWN_X="12.0"
        fi
        SPAWN_Y="3.0"
        SPAWN_Z="0.1"
        SPAWN_YAW_DEG="90"
        SPAWN_YAW=$(awk -v d="${SPAWN_YAW_DEG}" 'BEGIN{printf "%.16f", d*atan2(0,-1)/180}')
    elif [ "${WORLD_ENV}" = "fire_drill_room" ]; then
        if [ "${NS}" = "NX01" ]; then
            SPAWN_X="2.0"
        else
            SPAWN_X="-2.0"
        fi
        SPAWN_Y="-9.5"
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
# 2026-10-13：从写死的 3 秒改成 SPAWN_SETTLE_S（默认 12）。
# 3 秒不够，而且**最后一个 spawn 的飞机受害最重**（它从落地弹稳到 PX4 开始
# 估加速度计零偏之间的时间最短）。实测 NX02 的 IMU 水平偏置被烤成
# √(0.403²+0.380²)=0.554 m/s²（机体真实姿态是 0.00°，完全水平），而
# EKF2_ABL_LIM=0.4 是偏置修正上限 ⇒ **EKF2 永远修不掉**，残余约 0.15 m/s²
# 持续泄进速度估计，静止时 odom 报 0.29 m/s（NX01 只有 0.025）。
# pt4ctrl 写死 `odom_data.v.norm() > 0.1` 拒绝非静止起飞，于是 NX02 连拒 15 次、
# 永远起不来——"为什么总是 NX02"就是这个先后顺序竞态。
# 跟上面那段注释记录的"上一次"是同一类问题的镜像版本：那次受害者是先 spawn
# 的 NX01（被后面的重活抢物理循环），这次是后 spawn 的 NX02（稳定时间不够）。
# Gazebo 重载下 RTF < 1，所以墙钟 3 秒可能只有一两秒仿真时间。
sleep "${SPAWN_SETTLE_S:-12}"

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
    # （2026-10-23 这里曾经单独拉起 gzclient，10-24 回退——无效，见上面启动
    #   Gazebo 那段的说明。gzclient 现在跟 gzserver 一起由 launch 起。）
) &

# ------------------------------------------------------------------
# 2026-10-18：把 EKF2 的融合窗口开大，并把 vision 样本整体往过去挪 150 ms
# ——治"飞机飞行中途无故降落"。**必须在起 PX4 之前改**，因为飞控启动时才读
# 这个 airframe 文件（`-d "$PWD/ROMFS/px4fmu_common"`）。
#
# 现象与链条（当天实测，两架都掉过）：EKF2 停止融合 vision（cs_ev_pos/hgt/yaw
# 1->0）-> vehicle_local_position 的 xy_valid/v_xy_valid 变 0 -> mode 从
# OFFBOARD 掉成 ALTCTL -> 没有遥控器输入 -> mc_pos_control `invalid setpoints`
# -> `Failsafe: blind land` 把飞机放到地上（NX02 从 2.15 m、NX01 从 1.94 m）。
# 根因不是数据断流：停摆那十几秒 vehicle_visual_odometry 一直 33.3 Hz 在到
# （最大间隔 48 ms、test_ratio 全程 0.00、innovation ±2 cm），而
# `estimator_aid_src_ev_pos` 整段空白——**数据到了 uORB，EKF2 取不到**。
# 上游根因是**两套参考时间源**：PX4 走 Gazebo 步进时钟、ROS 侧走墙钟，实测
# RTF=0.9992、以 -0.83 ms/s 分离，mavros 每 51 s 重置一次时间同步，阶跃瞬间
# vision 时间戳跳 ±100 ms，样本落到融合窗外。
#
# 为什么是这两个数：偏移实测 min -101.8 / p50 -9.5 / p95 +52.4 / max +117.3 ms，
# **跨度 219 ms 超过出厂的 EKF2_DELAY_MAX=200**，单靠 EV_DELAY 平移盖不住两头。
#   DELAY_MAX 400 -> 窗口 [now-400, now]
#   EV_DELAY  150 -> 样本落在 [now-252, now-33]，两个方向都在窗口内
# 🔴 **只改 EV_DELAY 不改 DELAY_MAX 会把飞机摔了**（当天实测）：先只设了
#   EV_DELAY=150，负偏移 -98 叠加成 -248 ms、超出 200 ms buffer，样本因**太旧**
#   被丢，照样停摆 17.8 s、NX01 从 1.94 m 掉到 0.00 m——而同样的 -98 ms 在
#   EV_DELAY=0 下完全无害。两行必须一起生效，所以写在同一处、一次注入。
#
# 为什么不写进 patches/px4_iris_ekf2_vision_aiding.patch：那个 patch 的 COPY 在
# Dockerfile 第 44 行，改它会让后面 500 行的层全部失效、连 PX4 和四个 colcon
# 一起重编（几十分钟）；entrypoint 的 COPY 在倒数第四层，改这里重建是秒级。
# 代价：EKF2 融合时刻更滞后、当前状态更多靠外推，控制带宽略降。
# 回退（不用重建镜像）：SIM_EKF2_EV_WINDOW=off docker compose up -d sim-world
# 判定修没修好的唯一硬指标：跑一轮后看 ulog 的 `estimator_aid_src_ev_pos`
# 还有没有 >1 s 的空档（修之前 NX01 18 分钟 10 次）。
# ------------------------------------------------------------------
AIRFRAME_10015=/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris
if [ "${SIM_EKF2_EV_WINDOW:-on}" = "on" ]; then
    # 幂等：容器重建后文件回到镜像初始状态，这里只在没注入过时追加
    if ! grep -q "EKF2_DELAY_MAX" "$AIRFRAME_10015"; then
        cat >> "$AIRFRAME_10015" <<'EKF2_WINDOW_EOF'

# sim-world-entrypoint.sh 注入（SIM_EKF2_EV_WINDOW=on）：见该脚本里的长注释
param set-default EKF2_DELAY_MAX 400
param set-default EKF2_EV_DELAY 150
EKF2_WINDOW_EOF
    fi
    # 断言：注入失败就当场停，不能让飞控带着出厂窗口起来、再在空中掉下去
    if ! grep -q "^param set-default EKF2_DELAY_MAX 400$" "$AIRFRAME_10015"        || ! grep -q "^param set-default EKF2_EV_DELAY 150$" "$AIRFRAME_10015"; then
        echo "!! [sim-world] EKF2 融合窗口注入失败（$AIRFRAME_10015），拒绝继续启动 !!" >&2
        exit 1
    fi
    echo "== [sim-world] EKF2 融合窗口已注入：EKF2_DELAY_MAX=400 + EKF2_EV_DELAY=150 =="
else
    echo "== [sim-world] SIM_EKF2_EV_WINDOW=off：保持 PX4 出厂窗口（DELAY_MAX=200 / EV_DELAY=0），EV 可能周期性停摆 =="
fi

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

# ---- 2026-10-17：收窄 MAVLink 流速率 ----
# 背景：仿真反复出现"飞着飞着趴地"，根因链是 CPU 峰值打满 → mavros(单进程
# 113%) 被调度延迟 → timesync 把累积偏移一次性清零(实测 offset 0.1277→0.0003，
# stamp 前跳 127ms) → PX4 报 time jump → mc_pos_control 判 setpoint 失效 →
# Failsafe: blind land。见记忆 project_px4_param_relax_servicetype。
#
# 为什么在这里下发、而不是改 px4-rc.mavlink 或固件默认值：
#   · px4-rc.mavlink 里的覆盖跟 configure_streams_to_default() 有竞态，历史
#     注释已记"实测没生效"；等实例完全起来后再发才稳（实测 100→50Hz 生效）。
#   · 改固件默认值(mavlink_main.cpp)要重编译 PX4，而这里只动 entrypoint 末层。
#   · 多实例必须用 `--instance N`，不带它 client 永远连实例 0
#     （`-i N` 会被当成 mavlink 子命令自己的参数，静默打 usage 然后什么都没改）。
#
# 速率依据（静态分析 src/ scripts/ contestant_sim/ 里实际订阅的话题 + 实测对照）：
#   · LOCAL_POSITION_NED 喂 local_position/odom(30处引用)+pose，是最核心的反馈 → 50
#   · HIGHRES_IMU / ATTITUDE_QUATERNION 融合出 imu/data 喂 pt4ctrl → 50
#   · ODOMETRY 和 LOCAL_POSITION_NED_COV 实测是**白发的**：把它们单独降到 1Hz，
#     local_position/odom 频率纹丝不动(99.7→100.2/100.0) → 没有任何消费者 → 10
#   · ATTITUDE 没有订阅者(imu/data 走的是 ATTITUDE_QUATERNION) → 10
#   · DISTANCE_SENSOR 喂 hrlv_ez4_pub，是 line_goto 仿地高度环的唯一输入
#     (见记忆 project_altitude_chain_real)。它原本只有 10Hz(实测到达 9Hz)，
#     2026-10-17 用户要求**提到 50Hz**——这条是提速不是降速：仿地环的 agl
#     更新越快，"地面抬高"判定越跟得上，高度指令抖动越小。
#   · GCS 链路(18570)那 9 条 50Hz：仿真里没有 QGC 连它，地面站走 ROS 话题 → 全 10
# ⚠️ HIGHRES_IMU 一度砍到 50，2026-10-17 **已回滚到 200**：它喂 imu/data_raw，
#    而 uwb_imu_fusion_node 吃这路 IMU 做融合、再发 vision_pose/pose_cov 给 EKF2
#    —— 这是 uwb_imu 模式下 EKF2 唯一的位置输入。砍到 50 之后实测出现
#    **NX02 的 odom 瞬时发散 7 米**（odom 报 x=24.47 在场地外，而真值全程最大
#    只有 17.28），随后位置被判不可信、降级 ALTCTL、invalid setpoints、盲降。
#    这条是整条链上唯一直接作用于定位的流，不要再往下砍。
(
    sleep 25                       # 等两个实例都 boot_complete，避开上面说的竞态
    PX4_MAVLINK_CLI=/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-mavlink
    cd /opt/PX4-Autopilot || exit 0
    for i in $(seq 1 "${NUM_AGENTS}"); do
        inst=$((i-1)); off_port=$((14580+inst)); gcs_port=$((18570+inst))
        # 板外控制链路：核心反馈降到 50，白发的降到 10
        for spec in "LOCAL_POSITION_NED 50" "HIGHRES_IMU 200" "ATTITUDE_QUATERNION 50" \
                    "DISTANCE_SENSOR 50" \
                    "ATTITUDE 10" "ODOMETRY 10"; do
            # 注：LOCAL_POSITION_NED_COV 不在 onboard 链路的流表里，下发会被
            # PX4 回 "stream not found"（无害但刷错误日志），所以不列。
            set -- ${spec}
            "${PX4_MAVLINK_CLI}" --instance "${inst}" stream -r "$2" -s "$1" -u "${off_port}" \
                >/dev/null 2>&1 || true
        done
        # GCS 链路：仿真里没有客户端，全部压到 10
        for sname in POSITION_TARGET_LOCAL_NED LOCAL_POSITION_NED GLOBAL_POSITION_INT \
                     ATTITUDE ATTITUDE_QUATERNION ATTITUDE_TARGET SERVO_OUTPUT_RAW_0 \
                     RC_CHANNELS OPTICAL_FLOW_RAD; do
            "${PX4_MAVLINK_CLI}" --instance "${inst}" stream -r 10 -s "${sname}" -u "${gcs_port}" \
                >/dev/null 2>&1 || true
        done
        echo "== [sim-world] 实例 ${inst} 的 MAVLink 流速率已收窄（核心 50Hz / 其余 10Hz）=="
    done
) &

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
        # 2026-10-13：yaw 噪声原来**根本没下发**，一直在用节点默认 2.0°。
        # uwb_imu 模式下这个 yaw 会进 vision_pose -> EKF2，是真噪声源之一。
        -p yaw_noise_std_deg:="${UWB_YAW_NOISE_STD_DEG:-2.0}" \
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
