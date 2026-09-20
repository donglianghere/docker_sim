#!/usr/bin/env bash
# flight-stack 容器入口：单机的 DLIO + mighty + ros2_px4_stack 三件套。
# 跑两份容器实例（docker-compose里分别传 NAMESPACE=NX01 / NX02）。
#
# 环境变量：
#   NAMESPACE       必填，如 NX01 / NX02
#   AGENT_INDEX     必填，1-based，用于 ROS_DOMAIN_ID/MAV_SYS_ID等对齐PX4实例编号
#   SIM_WORLD_HOST  sim-world容器的可达地址（host网络模式下用127.0.0.1即可）
#   LOCALIZATION_SOURCE  默认 gt（2026-08-10改，原默认dlio/uwb_slam，见下）。
#                   可选 gt——跳过真正的DLIO SLAM，改用gt_odom_bridge节点把
#                   Gazebo仿真真值（/plug/model_states_plug）转发成跟DLIO
#                   同名的odom话题+TF（"${NAMESPACE}/odom" ->
#                   "${NAMESPACE}/base_link"，静态"${NAMESPACE}/base_link" ->
#                   "${NAMESPACE}/lidar"），喂给mighty规划器和PX4飞控（经
#                   repub_odom.py的vision_pose）。用来单独验证"规划+板外控制器+
#                   飞控"这条链路，排除DLIO SLAM本身收敛/漂移的影响。可选
#                   uwb_imu——情景一(SLAM完全失效，见docker_sim/多机空间坐标
#                   对齐问题_设计讨论纪要.docx第8.1节)，同样跳过DLIO，改用
#                   uwb_imu_fusion_node把UWB(位置+yaw) + IMU(roll/pitch)原样
#                   拼装成完整位姿直接发给mavros/vision_pose/pose_cov（跳过
#                   repub_odom这一步，dynus_mavros.launch.py会自动识别这个
#                   环境变量跳过它，避免抢话题），再用local_position_readback_
#                   node把PX4融合后的mavros/local_position/odom读回来改名重发
#                   成跟gt/uwb_slam同名同格式的odom话题+TF，下游规划栈不用
#                   区分。可选uwb_slam（2026-08-10改前叫dlio；2026-08-12把
#                   原来单独存在的"dlio"和"uwb_slam"两个值合并成这一个——
#                   两者在flight-stack这边的实际行为从一开始就完全一样，都是
#                   起真实DLIO SLAM，"dlio"这个名字反而没体现出origin_setter_
#                   node这个节点本来就无条件常驻跑、一直在用滑窗最小二乘
#                   在线估计局部系相对全局系的旋转θ*(飞机一动起来就自动
#                   累积样本，不需要手动触发，诊断话题.../origin_setter/
#                   yaw_estimate能实时看到)，拆成两个值纯粹是历史包袱，见
#                   DEBUG_JOURNAL.md 2026-08-12"评估LOCALIZATION_SOURCE功能
#                   重复"那条）——三种模式互不相同，不会同时跑，都发布同一个
#                   最终话题名(<namespace>/dlio/odom_node/odom)，同时跑会有
#                   多个发布者抢话题，下游收到谁的数据变得不确定。
#                   PLANNER=ego_planner时gt/uwb_imu两种模式下都会额外起
#                   gt_cloud_bridge_node，把原始mid360点云用上面这条TF变换到
#                   odom系再发布，补上DLIO本来提供的dlio/odom_node/pointcloud/
#                   deskewed这个话题（PLANNER/LOCALIZATION_SOURCE可以任意
#                   组合，见下面PLANNER段落）。
#                   2026-08-13新增三个"单机"值：single_slam_only/
#                   single_uwb_imu/single_uwb_slam，是Jetson Orin NX单机真机
#                   移植前的仿真彩排配置（对应真机部署方案里讨论的三种真机
#                   场景：完全没有UWB硬件、UWB+IMU替代SLAM、UWB仅做起飞点
#                   一键定原点），行为分别跟uwb_slam(不启动uwb_origin_bridge，
#                   见下)/uwb_imu/uwb_slam完全一致，只是显式要求
#                   NUM_AGENTS=1（不等于1直接报错退出，避免在双机compose下
#                   手滑用错模式）。single_slam_only是唯一有实质行为差异的：
#                   真机上"完全没有UWB硬件"这种部署形态不应该起
#                   uwb_origin_bridge这个包（它的origin_setter/
#                   frame_align_bridge两个节点在没有UWB发布uwb/pose_abs时
#                   只是安静地等数据、不会报错，但语义上不该在这种场景下
#                   存在），跟uwb_slam的唯一区别就是entrypoint末尾跳过这一步。
#                   uwb_slam/uwb_imu两个非single_*的值继续保留（双机仿真/
#                   联调用），不受这次改动影响。
#   PUBLISH_FRAME_ALIGN  默认 true，只影响sim-world容器（这里列出来方便
#                   查阅，flight-stack这边不读这个变量）——控制
#                   uwb_ground_truth_node要不要发布仿真真值版的
#                   `/frame_align/*`（喂给mighty的use_frame_alignment机间
#                   避让闭环）。设成false时，跨机map系对齐的唯一数据源变成
#                   frame_align_bridge_node（硬件可迁移，靠TF查
#                   origin_setter_node标定出来的world系链路，见
#                   uwb_origin_bridge包），专门用来验证"不靠仿真真值、光靠
#                   真实标定也能对齐"这件事，跟LOCALIZATION_SOURCE是两个
#                   正交的开关——2026-08-12之前这个开关是从
#                   LOCALIZATION_SOURCE=uwb_slam自动推导的，现在拆成独立
#                   变量，任何LOCALIZATION_SOURCE取值下都能单独控制。
#   CONTROLLER      默认 px4ctrl（2026-08-10改，原默认ros2_px4_stack，传
#                   CONTROLLER=ros2_px4_stack可恢复旧默认）。px4ctrl是
#                   2026-08-07完成的ROS2(Humble)移植版，2026-08-13确认：
#                   LOCALIZATION_SOURCE=uwb_slam下PLANNER=mighty/ego_planner
#                   都已实测完整起飞-跟踪-降落流程验证通过，不再是"仅编译
#                   冒烟测试"级别；LOCALIZATION_SOURCE=uwb_imu时仍然禁止
#                   使用（2026-08-12实测炸机，这条限制没有变，见下面
#                   LOCALIZATION_SOURCE段落）；gt定位源下这个组合还没有
#                   专门验证记录。可选 ros2_px4_stack——唯一经过仿真完整
#                   验证的板外控制器（起飞-跟踪-降落全流程实测过）；
#                   可选 so3ctrl——复用px4ctrl的飞行状态机(PX4CtrlFSM，
#                   逐字未改)，内部控制律换成KumarRobotics/kr_mav_control
#                   的SO3几何位置环(P+D+积分+倾角限幅，比px4ctrl自己的
#                   线性近似更完整)，代码在vendor/px4ctrl_ros2/so3ctrl，
#                   2026-08-13确认：跟px4ctrl同样条件（uwb_slam+mighty/
#                   ego_planner）下已实测验证通过，见
#                   so3ctrl/NOTICE.md的许可证归属说明。
#                   可选 pt4ctrl（2026-08-13新增）——复用px4ctrl的飞行状态机
#                   (PX4CtrlFSM，起飞/悬停/降落/RC失控保护逐字未改)，但删掉
#                   了推力模型估计+姿态解算这两步控制律，改成把状态机算出的
#                   位置/速度/加速度/偏航参考量直接打包成
#                   trajectory_msgs/MultiDOFJointTrajectory发到
#                   mavros/setpoint_trajectory/local，交给PX4固件自己的
#                   位置控制环解算姿态/推力——转发方式仿照ros2_px4_stack的
#                   point_to_traj/_pack_into_traj，代码在
#                   vendor/px4ctrl_ros2/pt4ctrl。动机：ros2_px4_stack没有
#                   真正的状态机，起飞降落只支持一次；px4ctrl/so3ctrl能反复
#                   起降但直接发姿态指令，LOCALIZATION_SOURCE=uwb_imu这类
#                   低更新率场景下被禁止（2026-08-12实测炸机）。pt4ctrl理论
#                   上应该同时具备"能反复起降"（继承px4ctrl状态机）和"对
#                   更新率不敏感"（继承ros2_px4_stack的转发方式）两个优点，
#                   但这个组合从未实测过，包括在uwb_imu模式下能不能真的
#                   稳定工作也完全未知，是全新代码、没有任何飞行验证，见
#                   DEBUG_JOURNAL.md 2026-08-13相关记录，首次验证务必单机+
#                   低高度+悬停起步。
#                   四者是"四选一"的关系：ros2_px4_stack模式下正常跑
#                   track_dynus_traj(发setpoint)+repub_odom+
#                   mocap_to_livox_frame等全部职责；px4ctrl/so3ctrl/pt4ctrl
#                   模式下用RUN_OFFBOARD_FOLLOWER=false让
#                   dynus_mavros.launch.py跳过track_dynus_traj（避免多个
#                   节点同时抢着给PX4发setpoint/解锁指令），改起对应的
#                   xxxctrl_node+goal_to_poscmd+px4_param_relax+
#                   takeoff_gate（后三个是px4ctrl_bridge包里跟具体控制器
#                   无关的公共节点，三条路径共用），但repub_odom/
#                   mocap_to_livox_frame/静态TF这些和"发setpoint"无关的
#                   职责继续由dynus_mavros.launch.py提供，不重复实现。
#   PLANNER         默认 ego_planner（2026-08-10改，原默认mighty，传
#                   PLANNER=mighty可恢复旧默认）——
#                   ego-planner-swarm的规划器核心（ego_planner_node+
#                   traj_server），只用其自身建图+B样条优化，前端接
#                   DLIO/Gazebo（跟mighty吃的是同一个dlio/odom_node/odom
#                   和mid360_PointCloud2，gt模式下分别由gt_odom_bridge_node/
#                   gt_cloud_bridge_node顶上），后端发quadrotor_msgs/
#                   PositionCommand（相对话题名position_cmd）。两者是
#                   "二选一"的规划器：mighty模式下正常起mighty_node+
#                   global_mapper_ros；ego_planner模式下改起
#                   ego_planner_bridge这个launch文件，不重复订阅点云/odom。
#                   CONTROLLER三选一都支持：px4ctrl/so3ctrl直接remap
#                   ('cmd','position_cmd')吃这份PositionCommand；
#                   ros2_px4_stack额外起ego_planner_bridge包里的
#                   poscmd_to_goal_node，把PositionCommand转成
#                   dynus_interfaces/Goal发到track_dynus_traj订阅的
#                   /${VEH_NAME}/goal（跟px4ctrl_bridge的goal_to_poscmd
#                   刚好是反方向的同类桥接）。PLANNER/CONTROLLER/
#                   LOCALIZATION_SOURCE三个开关可以任意组合。
#   SLAM_BACKEND    默认 dlio（2026-08-14新增）。只在LOCALIZATION_SOURCE=
#                   uwb_slam/single_uwb_slam/single_slam_only（即"真的起
#                   SLAM"这几个值）下生效，gt/uwb_imu两种模式不跑真正的
#                   SLAM，这个开关对它们没有意义。可选dlio（原有行为不变）/
#                   point_lio（point_lio_ros2，dfloreaa第三方ROS2移植+
#                   本项目补的两个patch：Livox CustomMsg支持、命名空间/
#                   接口映射，详见docker/Dockerfile.flight-stack"7b. point_lio"
#                   一节）/fast_lio（hku-mars/FAST_LIO官方ROS2分支，
#                   2026-09-07新增仿真集成，2026-09-08接通DEPLOY_TARGET=hw
#                   （sim/hw各自独立的launch文件+调参，hw那份密度调参
#                   未验证，见docker/Dockerfile.flight-stack"7c. FAST-LIO2"
#                   一节和launch/fast_lio_hw.launch.py文件头docstring）。
#                   三个后端都发布到同一个最终话题名
#                   (<namespace>/dlio/odom_node/odom、
#                   <namespace>/dlio/odom_node/pointcloud/deskewed)和TF
#                   frame名字(<namespace>/odom -> <namespace>/base_link)，
#                   mighty/ego_planner_bridge/gt_odom_bridge/ros2_px4_stack
#                   这些下游消费者不用关心当前跑的是哪个后端，也不需要为了
#                   切换后端改动一行下游代码。⚠️ point_lio/fast_lio这两条
#                   链路只做过patch可应用性/colcon build层面的验证，还没有
#                   实际跑过仿真起飞验证过闭环（fast_lio是2026-09-06
#                   POINT_LIO_HIGH_FREQ_ODOM=true真机实测确认不可用之后转向
#                   评估的替代候选，见DEBUG_JOURNAL.md同日记录），属于新增
#                   的、待验证的可选项，默认值依然是经过实测的dlio，不影响
#                   现状。
#   POINT_LIO_CPU_CORES  默认4（2026-08-14新增）。只在SLAM_BACKEND=point_lio
#                   时生效。pointlio_mapping这个可执行文件没有自己限制
#                   OpenMP/pthread线程数，2026-08-14双机实测单进程CPU占用
#                   冲到~1150%（11+核），host load average冲到52（32核
#                   机器），点云/里程计速率被拖到个位数Hz——用taskset把这个
#                   进程钉在固定CPU核范围内（按AGENT_INDEX错开，NX01用
#                   0..N-1，NX02用N..2N-1），同时把OMP_NUM_THREADS设成同一个
#                   值封顶OpenMP线程池大小。这个值*双机数量不能超过宿主机
#                   实际核数，机器核数不够或者想留更多余量给Gazebo/其它
#                   节点时调小。
#   DEPLOY_TARGET   默认 sim（2026-08-14新增，真机部署第一批骨架代码）。
#                   可选 hw——切到真机模式，跟sim的差异只在"连什么硬件"这
#                   一层，DLIO/mighty/ego_planner/px4ctrl等算法逻辑完全不变：
#                   1) MAVROS的fcu_url从SITL的UDP端口公式换成真实串口
#                      （FC_SERIAL_DEVICE/FC_BAUD_RATE两个新环境变量）；
#                   2) INIT_X/Y/Z不再按AGENT_INDEX*3算（那是仿真spawn坐标），
#                      固定成0/0/0；
#                   3) PLANNER=ego_planner强制切CycloneDDS时，网卡从锁死的
#                      lo换成真实网卡（HW_NETWORK_INTERFACE），并把GCS加成
#                      静态Peer（GCS_PEER_ADDRESS，留空则不加）；
#                   4) 额外启动真实Mid-360驱动（docker/entrypoints/hw_launch/
#                      mid360_real.launch.py）和真实UWB驱动
#                      （nlink_uwb_bridge.launch.py），这两个仿真模式完全不
#                      启动（用Gazebo插件/uwb_sim顶替）；
#                   5) CONTROLLER=px4ctrl时改用px4ctrl_hw.launch.py
#                      （no_RC=false，需要真实遥控器）而不是
#                      px4ctrl_docker_sim.launch.py（no_RC=true）；
#                      CONTROLLER=pt4ctrl同理用pt4ctrl_hw.launch.py
#                      （2026-08-29补的，见那份文件的docstring——注意
#                      pt4ctrl本身仍是"未实测"状态，补launch文件不等于
#                      这条控制链路已验证）；CONTROLLER=so3ctrl同理用
#                      so3ctrl_hw.launch.py（2026-09-06补的，逐字照抄
#                      px4ctrl_hw.launch.py的模式，同样"未实测"，见该文件
#                      docstring）——三个控制器现在DEPLOY_TARGET=hw下都有
#                      对应的no_RC=false版本了，不会再有CONTROLLER因为缺
#                      hw launch文件被fail fast拦下；
#                   6) px4_param_relax节点（px4ctrl_bridge包）自己读这个
#                      环境变量决定放宽哪些PX4失控保护参数，hw模式下明显
#                      收紧，见该文件顶部docstring。
#                   7) 一处额外的fail fast防呆：LOCALIZATION_SOURCE=gt在hw
#                      模式下直接报错退出（gt模式的存在意义就是"跳过SLAM读
#                      Gazebo真值"，真机没有Gazebo，这个组合没有意义）。
#                      ⚠️ 这条注释之前还提到"SLAM_BACKEND=point_lio在hw
#                      模式下同样直接报错退出"，那是2026-08-19之前的旧
#                      状态，point_lio的hw launch文件早就补齐了（下面
#                      case分支里能看到），这条注释一直没跟着更新，
#                      2026-09-08顺手订正——point_lio/fast_lio现在都能在
#                      DEPLOY_TARGET=hw下跑，只是都还没经过真机验证。
#                   真机场景下这一整层改动只跟"接的是真硬件还是仿真"有关，
#                   跟走mighty/ego_planner、走uwb_slam/uwb_imu等其它开关
#                   完全正交，可以任意组合（gt是唯一的例外，见上面第7点）。
set -eo pipefail

# 2026-09-04：容器stop时PID1(本脚本)收到SIGTERM，此前没有任何trap，bash的
# 默认行为不会把这个信号转发给用`&`起的所有子进程（DLIO/mavros/UWB驱动等
# 一堆ros2进程），子进程要么在docker宽限期结束后被直接SIGKILL，要么根本
# 收不到终止信号——linktrack_node（UWB串口驱动）因此经常来不及走正常的
# 串口close()流程就被强杀，CH340这颗USB转串口芯片被留在卡死状态，下次
# 容器重启即使串口设备节点/权限都正常，open()也会稳定返回Input/output
# error，必须做USB层面复位（拔插/unbind-rebind）才能恢复，光重启容器没用
# （2026-09-04现场排查实测确认：脱离docker直接在host上open()同一个设备
# 节点，同样报I/O error，问题在host内核对这颗物理芯片的状态记录，不是
# 容器/脚本层面能直接绕过的）。
# 现补上：收到SIGTERM/SIGINT时转发给整个进程组（本脚本是这些`&`后台子
# 进程共同的进程组leader，`kill -TERM -$$`一次性覆盖全部，不需要像常见
# 写法那样逐个记录PID数组——这个脚本里几十处`&`launch如果要逐个记录PID
# 改动量太大），等它们各自走完自己的清理逻辑优雅退出，而不是被docker
# 宽限期结束后直接SIGKILL。
_term_handler() {
    trap '' SIGTERM SIGINT  # 避免转发信号又打到自己这个进程触发递归
    echo "== [flight-stack:${NAMESPACE:-?}] 收到终止信号，转发SIGTERM给所有子进程，等待优雅退出... =="
    kill -TERM -- -$$ 2>/dev/null || true
    wait || true
    exit 0
}
trap _term_handler SIGTERM SIGINT

# 2026-08-14：原来这里是10行分散的source（外加livox_ws/point_lio_ws顺序依赖的
# 说明注释），改成统一收进 source_all.sh——entrypoint.sh和手动进容器调试
# （`docker exec -it <container> bash` 之后 `source /opt/source_all.sh`）共用
# 同一份，避免两处顺序/内容不同步。set+u/set-u的成对关系跟原来一样：
# source_all.sh内部只负责`set +u`，`set -u`留在这里手动重新打开。
source /opt/source_all.sh
set -u

# DEPLOY_TARGET（2026-08-14新增，见文件头完整说明）：sim=原有行为，一个字节
# 都不变；hw=真机模式，只影响"连什么硬件"这一层的几处分支（下面陆续出现，
# 每处都有注释指回这里）。必须放在下面CycloneDDS网卡分支之前定义（它是第
# 一个要用到DEPLOY_TARGET的地方），比NAMESPACE/AGENT_INDEX的校验还早。
# 故意在这里就地校验、不认识的值直接退出，避免打错字（比如"HW"大写）被
# 静默当成sim处理——真机部署这类"输错值但没报错"的坑比仿真危险得多。
export DEPLOY_TARGET="${DEPLOY_TARGET:-sim}"
case "${DEPLOY_TARGET}" in
    sim|hw) ;;
    *)
        echo "!! [flight-stack:${NAMESPACE:-?}] DEPLOY_TARGET=${DEPLOY_TARGET} 不是已知值(sim/hw)，退出 !!" >&2
        exit 1
        ;;
esac
echo "== [flight-stack:${NAMESPACE:-?}] DEPLOY_TARGET=${DEPLOY_TARGET} =="

# ego-planner-swarm自己的Readme.md写明"FastDDS(ROS2默认)会导致明显卡顿，
# 原因未知，建议换cyclonedds"——2026-08-08实测复现类似症状（mavros/imu/data、
# mavros/local_position/odom这两个跟ego_planner毫不相关的话题，实测速率只有
# ~12-20Hz，远低于正常水平），跟这条上游已知问题吻合。只在
# PLANNER=ego_planner时切换，mighty这条已经实测验证过的路径继续用默认
# FastDDS不动。必须在这个容器里**任何**ros2节点（包括下面马上要起的MAVROS）
# 启动之前设置——同一个ROS_DOMAIN_ID(=20)下所有参与者（sim-world/
# flight-stack-nx01/flight-stack-nx02）必须用同一个RMW实现才能互相发现，
# 三处（这里+sim-world-entrypoint.sh）都要跟着PLANNER联动切换，不能只改
# 一处。
if [ "${PLANNER:-ego_planner}" = "ego_planner" ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "== [flight-stack:${NAMESPACE:-?}] PLANNER=ego_planner，切换RMW_IMPLEMENTATION=rmw_cyclonedds_cpp（缓解上游文档记录的FastDDS卡顿问题）=="

    # docker-compose.yml三个容器都是network_mode:host，直接暴露宿主机的全部
    # 网卡——实测宿主机上除了真正上网的wlp0s20f3，还挂了docker0(172.17.0.1)、
    # VMware的vmnet1(172.16.202.1)/vmnet8(192.168.19.1)这几个跟ROS2通信完全
    # 无关的虚拟网卡。CycloneDDS默认会枚举并尝试在**全部**网卡上发组播（不像
    # FastDDS那样默认更保守），容器日志被"ddsi_udp_conn_write ... failed with
    # retcode -1"刷屏，且是挂在mavros_node/dlio_odom_node这些发布者线程上的
    # （不是纯发现阶段的一次性开销）——每次发布/发现都要在这几个注定失败的
    # 虚拟网卡上先失败一轮，这才是切到CycloneDDS之后mavros/imu/data实测反而
    # 还是卡在几十Hz、且比之前FastDDS更容易抖动的真正原因，跟PX4固件那次
    # 50Hz限速是两个独立问题（那个已经用px4_onboard_imu_rate_firmware_
    # default.patch修过、也确认生效了）。
    # 三个容器（sim-world+两个flight-stack）本来就靠network_mode:host共享
    # 同一个网络命名空间，互相发现走本地环回(lo)完全够用，不需要真的组播到
    # 物理/虚拟网卡上。显式把CycloneDDS的网卡范围锁定成lo，从源头消掉这些
    # 注定失败的发送。
    #
    # DEPLOY_TARGET=hw：这套"锁lo"的前提（GCS和飞机在同一个网络命名空间）
    # 不成立了——GCS是另一台物理机，要走真实网卡+真实局域网才能发现Jetson
    # 上的节点。HW_NETWORK_INTERFACE（真实网卡名，登上Jetson后用`ip a`确认，
    # 常见是eth0/wlan0，这里给的默认值eth0纯粹是占位，几乎肯定要改）+
    # GCS_PEER_ADDRESS（GCS的静态IP，留空则不加Peers块、退回纯组播——组播
    # 在很多WiFi AP上不可靠，见DEBUG_JOURNAL.md 2026-08-14相关讨论，能填就
    # 尽量填）。这两个值都没有在真实网络环境里验证过，是这批骨架代码里
    # 最需要现场确认的部分。
    export CYCLONEDDS_URI="file:///tmp/docker_sim_cyclonedds.xml"
    if [ "${DEPLOY_TARGET}" = "hw" ]; then
        HW_NETWORK_INTERFACE="${HW_NETWORK_INTERFACE:-eth0}"
        echo "== [flight-stack:${NAMESPACE:-?}] DEPLOY_TARGET=hw：CycloneDDS网卡锁定改成 ${HW_NETWORK_INTERFACE}（占位默认值，需要用 ip a 现场确认是否正确）=="
        if [ -n "${GCS_PEER_ADDRESS:-}" ]; then
            echo "== [flight-stack:${NAMESPACE:-?}] GCS_PEER_ADDRESS=${GCS_PEER_ADDRESS}，加为CycloneDDS静态Peer =="
            cat > /tmp/docker_sim_cyclonedds.xml <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${HW_NETWORK_INTERFACE}" priority="default" multicast="true"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="${GCS_PEER_ADDRESS}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
        else
            echo "== [flight-stack:${NAMESPACE:-?}] 未设置GCS_PEER_ADDRESS，只启用真实网卡上的组播发现（如果WiFi AP屏蔽组播，GCS会发现不了这架飞机，需要补设GCS_PEER_ADDRESS）=="
            cat > /tmp/docker_sim_cyclonedds.xml <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${HW_NETWORK_INTERFACE}" priority="default" multicast="true"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF
        fi
    else
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
  </Domain>
</CycloneDDS>
EOF
    fi
fi

: "${NAMESPACE:?必须设置 NAMESPACE，如 NX01}"
: "${AGENT_INDEX:?必须设置 AGENT_INDEX，如 1}"
export VEH_NAME="${NAMESPACE}"

# dynus_mavros.launch.py 里的静态TF（world_mocap -> ${VEH_NAME}/init_pose）要用这几个
# INIT_* 环境变量，之前entrypoint里没设置过，全是None，传进Node(arguments=[...])直接
# 报 "TypeError: 'NoneType' object is not iterable" 崩溃。仿真下公式跟
# sim-world-entrypoint.sh 里spawn这架飞机用的坐标（x:="$((i*3))" y:="0" z:="0.1" yaw:="0"）
# 保持一致——AGENT_INDEX在两边是同一个1-based编号（NX01=1对应i=1，NX02=2对应i=2），
# 位置对不上TF链会算错。z=0.1跟sim-world那边一致——iris的base_link碰撞箱
# （0.47x0.47x0.11米，以原点为中心）箱底在原点下方0.055米，z=0.1只比零净空高度
# 多留1.5厘米，肉眼几乎看不出下落。之前两边都是z=3（3米悬空），配合重力开着、
# 飞机没解锁，会直接自由落体砸到地面（实测确认过，local_position/pose从z=3掉到
# z=-0.09）；中间试过PX4官方脚本用的z=0.83，对这个具体的iris模型来说依然是
# 悬空的，还是会往下摔一截。
# DEPLOY_TARGET=hw：这几个值只喂给上面那条"world_mocap -> init_pose"静态TF，
# 跟真机物理位置无关（真机没有"spawn点"这回事，起飞点由uwb_origin_bridge的
# 一键定原点机制在飞行时动态确定），固定成0/0/0即可，不用按AGENT_INDEX错开
# （真机部署一台Jetson只对应一架飞机，没有"同机多实例避免坐标重叠"这个仿真
# 特有的需求）。
if [ "${DEPLOY_TARGET}" = "hw" ]; then
    export INIT_X="0"
    export INIT_Y="0"
    export INIT_Z="0"
else
    export INIT_X="$((AGENT_INDEX * 3))"
    export INIT_Y="0"
    export INIT_Z="0.1"
fi
export INIT_ROLL="0"
export INIT_PITCH="0"
export INIT_YAW="0"

# 从 vehicle_profile.yaml 里把 mass/hover_thrust/f_max/f_min 解析出来，export给
# 打过patch的 ros2_px4_stack（get_thrust()/get_angular() 现在读这两个环境变量），
# 这样只用改 vehicle_profile.yaml 一处，mighty 和 ros2_px4_stack 就都用同一个数字，
# 不会再出现之前专项报告里那种 1.0kg vs 2.906kg 对不上的情况。
# 依赖 python3-yaml；如果基础镜像没装，去 Dockerfile.base 里加 python3-yaml。
#
# ⚠️ 2026-08-27修复：这4个变量之前是`export VAR=$(...)`无条件赋值——即使
# docker-compose.hw.yml/.env显式设过这几个环境变量，这里也会原样覆盖回
# vehicle_profile.yaml里的数字，等于外部覆盖入口形同虚设。改成
# `${VAR:-$(...)}`，外部设过就用外部的值，没设才退回vehicle_profile.yaml
# 解析结果这条兜底路径——vehicle_profile.yaml里的mass/f_max/f_min/
# hover_thrust是**仿真机型**(PX4官方iris+仿真挂载)算出来的数字，不是真机
# 物理参数，真机部署必须在.env里显式设VEHICLE_MASS_KG/VEHICLE_HOVER_THRUST/
# VEHICLE_F_MAX_N/VEHICLE_F_MIN_N这4个变量，不能假设默认值能直接飞。
export VEHICLE_MASS_KG="${VEHICLE_MASS_KG:-$(python3 -c "
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
print(d['offboard_dynus_follower']['ros__parameters']['mass'])
" 2>/dev/null || echo "2.906")}"
export VEHICLE_HOVER_THRUST="${VEHICLE_HOVER_THRUST:-$(python3 -c "
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
print(d['offboard_dynus_follower']['ros__parameters']['hover_thrust'])
" 2>/dev/null || echo "0.50")}"
export VEHICLE_F_MAX_N="${VEHICLE_F_MAX_N:-$(python3 -c "
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
print(d['mighty_node']['ros__parameters']['f_max'])
" 2>/dev/null || echo "26.0")}"
export VEHICLE_F_MIN_N="${VEHICLE_F_MIN_N:-$(python3 -c "
import yaml
with open('/opt/config/vehicle_profile.yaml') as f:
    d = yaml.safe_load(f)
print(d['mighty_node']['ros__parameters']['f_min'])
" 2>/dev/null || echo "3.0")}"
echo "== [flight-stack:${NAMESPACE}] VEHICLE_MASS_KG=${VEHICLE_MASS_KG} VEHICLE_HOVER_THRUST=${VEHICLE_HOVER_THRUST} VEHICLE_F_MAX_N=${VEHICLE_F_MAX_N} VEHICLE_F_MIN_N=${VEHICLE_F_MIN_N} =="

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
# 2026-08-27新增：真实物理参数(质量/最大最小推力)同样要跟着VEHICLE_MASS_KG/
# VEHICLE_F_MAX_N/VEHICLE_F_MIN_N走，不能让mighty_node继续用vehicle_profile.yaml
# 里那份仿真机型的静态数字——上面VEHICLE_MASS_KG等变量已经支持外部覆盖，这里
# 只是把覆盖结果实际写进mighty_node/offboard_dynus_follower两处，跟v_max等
# 规划参数走同一条"生成运行时yaml"的路径。
params['mass'] = float(os.environ['VEHICLE_MASS_KG'])
params['f_max'] = float(os.environ['VEHICLE_F_MAX_N'])
params['f_min'] = float(os.environ['VEHICLE_F_MIN_N'])
follower_params = d.setdefault('offboard_dynus_follower', {}).setdefault('ros__parameters', {})
follower_params['mass'] = float(os.environ['VEHICLE_MASS_KG'])
follower_params['hover_thrust'] = float(os.environ['VEHICLE_HOVER_THRUST'])
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
#
# DEPLOY_TARGET=hw：没有PX4 SITL、没有UDP端口这回事，真实飞控走串口。
# FC_SERIAL_DEVICE（占位默认/dev/ttyTHS1，需要按实际接线确认，USB接法通常是
# /dev/ttyACM0）+FC_BAUD_RATE（默认921600，必须跟PX4侧该TELEM口的
# SER_TELx_BAUD参数一致，这个参数要用QGC通过USB直连飞控才能改，见
# docker_sim/真机部署操作清单.md"3.3 QGC直连飞控预配置"）。mavros的
# fcu_url对串口设备的写法是"设备路径:波特率"，跟UDP写法完全不同格式。
# 2026-09-09新增：对地测距雷达要接进MAVROS才能在ROS2侧被观察/使用（大赛
# 任务系统"仿地飞行"这项能力刚需，见gen_iris_mid360_sdf.py新增的
# merge_rangefinder()）。2026-09-10一度按用户指示把Gazebo侧焊的传感器
# 从model://lidar换成model://sonar，但实测sonar会直接把gzserver
# 段错误crash掉（跟mid360的Livox自定义射线插件在ODE碰撞检测层冲突，
# 详见DEBUG_JOURNAL.md 2026-09-10条目的gdb复现记录），已经改回
# model://lidar。
# mavros官方默认的px4_pluginlists.yaml把distance_sensor/rangefinder两个
# 插件放进了plugin_denylist（"Plugin distance_sensor ignored"这行日志），
# 跟PX4固件参数/接线完全无关，纯粹是mavros自己的保守默认值，必须解禁——
# 下面python脚本只做这一件事，不再往px4_config.yaml里插入任何自定义
# distance_sensor订阅条目（2026-09-10之前插过一条`lidar_0_sub:
# {subscriber: true, id: 0, ...}`，已经证明是画蛇添足、而且是错的，
# 见下一段说明）。
#
# 2026-09-10踩过的坑：以为mavros官方px4_config.yaml里distance_sensor
# 插件自带的订阅条目(sonar_1_sub id:2/laser_1_sub id:3)"没有id:0"、
# 需要自己补一条id:0的`_sub`条目才能接住MAVLink id=0的DISTANCE_SENSOR
# 消息（这个id=0的推导本身是对的，读gazebo_mavlink_interface.cpp+
# gazebo_lidar_plugin.cpp源码确认过，"lidar"这个嵌套模型名没有末尾
# 数字，PX4 SITL默认给它分配id=0）——但漏看了mavros官方px4_config.yaml
# 里其实**已经有**一条`hrlv_ez4_pub: {id: 0, frame_id: "hrlv_ez4_sonar",
# sensor_position: {x:0, y:0, z:-0.1}, ...}`条目，而且**没有**
# `subscriber: true`。mavros的distance_sensor插件里`_sub`(subscriber:
# true)和`_pub`(不写subscriber或者false)是两个方向完全相反的角色：
# `_sub`是"mavros订阅一个ROS话题、把数据当成外部传感器数据转发给FCU"
# （给真机接了独立测距模块、由companion computer喂数据给飞控这种场景
# 用的），`_pub`才是"mavros接收FCU自己发出来的MAVLink DISTANCE_SENSOR
# 消息、转发成ROS话题发布出去"（我们这个场景：Gazebo仿真/真实传感器的
# 数据本来就是FCU自己在算/自己在转发，mavros只需要单纯转成ROS话题）。
# 我们自己额外插入一条同样id=0、但`subscriber: true`的`lidar_0_sub`
# 条目，跟已经存在的`hrlv_ez4_pub`（同一个id=0，角色相反）直接冲突——
# 实测mavros日志刷屏报错"DS: lidar_0_sub (id 0) is subscriber, but i
# got sensor data for that id from FCU"，数据全被这个冲突吃掉，一条都
# 发布不出来ROS话题（`ros2 topic list`里看不到任何distance_sensor相关
# 话题）。修法：把这段自定义插入逻辑整个删掉，什么都不用加——stock配置
# 里的`hrlv_ez4_pub`（id:0）本来就能正确接住我们这颗雷达的数据，话题名
# 会是`/<namespace>/mavros/distance_sensor/hrlv_ez4_sonar`（按frame_id
# 命名，不是按yaml key名）。
#
# 顺手发现并统一解决的旧坑：hw分支（DEPLOY_TARGET=hw）2026-08-18就写了
# 注释说要引用`/opt/hw_launch/px4_hw.launch`+`px4_pluginlists_hw.yaml`
# 解决同一个denylist问题，但这两个文件从来没有被真正创建过（`find`/`grep`
# 全项目确认过），hw模式此前跑到这一步`ros2 launch`必然因为文件不存在
# 直接失败——不是这次改动引入的新问题，顺带用同一套生成逻辑修掉，sim/hw
# 两个部署目标共用同一份运行时生成的launch文件，不再分别维护两套。
python3 - <<'PYEOF'
MAVROS_LAUNCH_DIR = "/opt/ros/humble/share/mavros/launch"

# 1. pluginlists：逐行过滤删掉两行，不用yaml.safe_load再dump——那样会把
#    denylist列表之外的注释全部冲掉，这个文件本来结构很简单，直接删行
#    更保险，不会意外改动无关内容。
with open(f"{MAVROS_LAUNCH_DIR}/px4_pluginlists.yaml") as f:
    lines = f.readlines()
lines = [l for l in lines if l.strip() not in ("- distance_sensor", "- rangefinder")]
with open("/tmp/px4_pluginlists_contest.yaml", "w") as f:
    f.writelines(lines)

# 2. launch文件：照抄mavros官方px4.launch的内容，只把pluginlists_yaml
#    这一个value换成上面生成的denylist精简版，config_yaml原样保留指向
#    官方stock px4_config.yaml不动——2026-09-10发现stock配置里的
#    distance_sensor插件已经有一条`hrlv_ez4_pub: {id: 0, ...}`（不带
#    subscriber:true，receive-from-FCU/publish-to-ROS方向），正好接住
#    我们这颗id=0的雷达，不需要额外插入任何自定义订阅条目（之前插过一条
#    冲突的`lidar_0_sub`，已经证明是错的、删掉了，详见上面大段注释）。
with open(f"{MAVROS_LAUNCH_DIR}/px4.launch") as f:
    launch_content = f.read()
launch_content = launch_content.replace(
    '$(find-pkg-share mavros)/launch/px4_pluginlists.yaml', '/tmp/px4_pluginlists_contest.yaml',
)
with open("/tmp/px4_contest.launch", "w") as f:
    f.write(launch_content)
PYEOF
MAVROS_LAUNCH_FILE="/tmp/px4_contest.launch"

if [ "${DEPLOY_TARGET}" = "hw" ]; then
    FC_SERIAL_DEVICE="${FC_SERIAL_DEVICE:-/dev/ttyTHS1}"
    FC_BAUD_RATE="${FC_BAUD_RATE:-921600}"
    MAVROS_FCU_URL="${FC_SERIAL_DEVICE}:${FC_BAUD_RATE}"
    echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：MAVROS走真实串口（占位默认值，需要按实际接线确认）=="
else
    PX4_INSTANCE=$((AGENT_INDEX - 1))
    MAVROS_FCU_URL="udp://:$((14540 + PX4_INSTANCE))@127.0.0.1:$((14580 + PX4_INSTANCE))"
fi
echo "== [flight-stack:${NAMESPACE}] 启动 MAVROS (fcu_url=${MAVROS_FCU_URL} tgt_system=${AGENT_INDEX}) =="
ros2 launch "${MAVROS_LAUNCH_FILE}" \
    namespace:="${NAMESPACE}/mavros" fcu_url:="${MAVROS_FCU_URL}" tgt_system:="${AGENT_INDEX}" &
sleep 3

LOCALIZATION_SOURCE="${LOCALIZATION_SOURCE:-gt}"
# single_slam_only/single_uwb_imu/single_uwb_slam是单机专用模式（见文件头
# 2026-08-13说明）——双机compose下NUM_AGENTS硬编码/传参传成2时用了这三个值，
# 不是"能不能跑"的问题（frame_align_bridge在num_agents=1时本来就自然空转，
# origin_setter_node也完全不关心NUM_AGENTS），而是"跑起来的语义跟名字对不上"，
# 容易误判成"已经在双机场景下验证过单机模式"，这里直接卡死更安全。
case "${LOCALIZATION_SOURCE}" in
    single_*)
        if [ "${NUM_AGENTS:-2}" != "1" ]; then
            echo "!! [flight-stack:${NAMESPACE}] LOCALIZATION_SOURCE=${LOCALIZATION_SOURCE} 是单机专用模式，但 NUM_AGENTS=${NUM_AGENTS:-2}（应为1）——检查 docker-compose.yml 或环境变量，退出 !!" >&2
            exit 1
        fi
        ;;
esac
export CONTROLLER="${CONTROLLER:-px4ctrl}"
# 三选一：ros2_px4_stack自己发setpoint，px4ctrl/so3ctrl接管这个职责时
# 要跳过track_dynus_traj。原来只有px4ctrl一个可选项时是if/else，现在
# 三个选项了改成case，同时对未知值报错退出而不是静默当成
# ros2_px4_stack处理（打错字比如"px4Ctrl"之前会被静默吞掉，现在会
# 直接失败，更容易发现问题）。
case "${CONTROLLER}" in
    ros2_px4_stack)
        export RUN_OFFBOARD_FOLLOWER="true"
        ;;
    px4ctrl|so3ctrl|pt4ctrl)
        export RUN_OFFBOARD_FOLLOWER="false"
        ;;
    *)
        echo "!! [flight-stack:${NAMESPACE}] CONTROLLER=${CONTROLLER} 不是已知值(ros2_px4_stack/px4ctrl/so3ctrl/pt4ctrl)，退出 !!" >&2
        exit 1
        ;;
esac
# DEPLOY_TARGET=hw时曾经so3ctrl/pt4ctrl没有对应的hw版launch文件
# （no_RC=false版本）——pt4ctrl的hw版已于2026-08-29补齐(pt4ctrl_hw.
# launch.py)，so3ctrl的hw版已于2026-09-06补齐(so3ctrl_hw.launch.py，
# 逐字照抄px4ctrl_hw.launch.py的模式)，四个已知CONTROLLER值现在
# DEPLOY_TARGET=hw下全部有对应launch文件，这条fail fast名单已经空了，
# 留着这段判断只是为了将来新增CONTROLLER时不会忘记同步这里——提前在这里
# fail fast，不要等MAVROS/DLIO/真实驱动这些都已经启动起来了才发现控制器
# 这块起不来，浪费时间也容易让人误以为"系统起来了"。真正调用哪个launch
# 文件在下面CONTROLLER dispatch那段，这里只是提前拦截，逻辑跟那边保持
# 一致。
if [ "${DEPLOY_TARGET}" = "hw" ] && [ "${CONTROLLER}" != "px4ctrl" ] && [ "${CONTROLLER}" != "pt4ctrl" ] && [ "${CONTROLLER}" != "so3ctrl" ] && [ "${CONTROLLER}" != "ros2_px4_stack" ]; then
    echo "!! [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw 下 CONTROLLER=${CONTROLLER} 还没有对应的hw版launch文件（no_RC=false），拒绝静默退回仿真版（no_RC=true会让RC failsafe形同虚设），退出 !!" >&2
    exit 1
fi
echo "== [flight-stack:${NAMESPACE}] CONTROLLER=${CONTROLLER} (RUN_OFFBOARD_FOLLOWER=${RUN_OFFBOARD_FOLLOWER}) =="

export PLANNER="${PLANNER:-ego_planner}"
# calcSwarmCost用来在共享的/broadcast_bspline全局话题上区分"自己"和"别人"
# 轨迹的ID，两架飞机必须不同——直接从已经存在的AGENT_INDEX(1-based)推导成
# 0-based，不用再额外要求docker-compose.yml传一个新的环境变量、也不会跟
# AGENT_INDEX不同步。仍然留了覆盖口子，万一以后需要手动指定。
export DRONE_ID="${DRONE_ID:-$((AGENT_INDEX - 1))}"
echo "== [flight-stack:${NAMESPACE}] PLANNER=${PLANNER} (drone_id=${DRONE_ID}) =="

# DEPLOY_TARGET=hw：真实Mid-360驱动在这里统一启动一次，不放进下面
# LOCALIZATION_SOURCE的某个具体分支里——2026-08-14自查时发现，原始点云
# (mid360_PointCloud2/mid360/imu)被四条完全不同的下游路径消费，覆盖范围
# 比"只有uwb_slam家族需要"更广：① uwb_slam/single_uwb_slam/single_slam_only
# 分支下DLIO/point_lio直接消费；② PLANNER=mighty时global_mapper_ros无条件
# 订阅原始点云（不看LOCALIZATION_SOURCE，自己做TF查找，见下面mighty分支的
# lidar_point_cloud_topic:=mid360_PointCloud2）；③ PLANNER=ego_planner+
# LOCALIZATION_SOURCE=uwb_imu/single_uwb_imu时gt_cloud_bridge_node消费
# （见下面ego_planner分支）。真正不需要原始点云的只有LOCALIZATION_SOURCE=gt
# 这一种（且已经在上面的gt分支里对DEPLOY_TARGET=hw做了fail fast，走到这里
# 说明LOCALIZATION_SOURCE不是gt）——统一放在这里启动一次，比在三个不同分支
# 里各自判断、容易漏掉某个组合更不容易出错。
if [ "${DEPLOY_TARGET}" = "hw" ]; then
    # LIDAR_IP/LIDAR_HOST_IP/LIDAR_SCAN_FREQ（2026-08-17新增）——原来这三个值
    # 硬编码在镜像里的MID360_config.json（COPY进/opt/livox_ws/src，改了host
    # 上的源文件不会影响已经build好的镜像，只能重新build或者手动docker cp/
    # docker exec改运行中容器），换雷达/换网段/换Jetson都要重新build镜像才能
    # 生效，太重。改成entrypoint启动时从环境变量现生成一份配置文件到/tmp
    # （跟上面CycloneDDS配置同样的处理方式），镜像里那份MID360_config.json
    # 降级成"未设环境变量时的示例/手动调试兜底"，不再是真机启动路径实际读的
    # 那份。
    #   LIDAR_IP        雷达自身IP（对应lidar_configs[0].ip），默认192.168.1.122
    #                    只是沿用livox官方示例的占位值，几乎肯定要按现场实测
    #                    （ping/Livox Viewer2扫描）改。
    #   LIDAR_HOST_IP   Jetson这边接雷达的网口IP（host_net_info数组里host_ip
    #                    字段），必须跟`ip a`看到的真实网口IP一致，雷达才能把
    #                    点云/IMU数据发回来。
    #   LIDAR_SCAN_FREQ  对应driver的publish_freq参数，2026-08-19从抄来的
    #                    上游默认值10.0改成20.0（见read_hw.md对应小节），
    #                    compose文件里的默认值同步改了，这里的兜底默认值
    #                    也保持一致。
    #
    # 2026-08-17真机联调重大修正：这套雷达实测固件版本35.01.01.08，型号其实是
    # MID-360S（不是MID-360！），livox_ros_driver2/SDK2内部这两个型号走完全
    # 不同的解析分支（顶层JSON key"MID360" vs "Mid360s"，SDK内部device_type
    # 编号也不一样，8 vs 35）、host_net_info的schema也不同（MID360是扁平对象
    # 4个独立xxx_ip字段，MID360S是数组+单个host_ip字段）。之前一直用MID360
    # 这套schema，SDK能收到雷达的发现广播、但因为device_type/schema对不上，
    # 从未真正完成"设备已连接"握手，56101/56201/56301/56401这几个实际数据
    # 端口从来没打开过——这是当天从"容器起不来"排查到"崩溃"排查到"卡住不动"，
    # 最后定位到的真正根因，不是Docker/ARM/SDK版本的问题。用Livox-SDK2自带的
    # samples/livox_lidar_quick_start裸机验证过：换成下面这套MID360S schema后
    # 点云/IMU回调立刻正常触发。
    LIDAR_IP="${LIDAR_IP:-192.168.1.122}"
    LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.1.5}"
    LIDAR_SCAN_FREQ="${LIDAR_SCAN_FREQ:-20.0}"
    # 2026-08-20新增：Mid-360S的IMU发布频率。⚠️ 最初想法是用SDK的
    # SetLivoxLidarImuRange命令直接让雷达按更低频率出IMU数据(见
    # patches/曾经短暂存在过的livox_imu_out_rate_100hz.patch，已撤掉)，
    # 真机实测命令被雷达ACK"success"、但实际输出频率纹丝不动——网上多个
    # 独立来源确认Mid-360的IMU芯片固件把频率写死在200Hz，SDK这个命令对
    # 这颗芯片压根没实现，不是配置错了。改成在driver内部(lddc.cpp)做
    # 软件下采样，这个变量现在是"目标Hz"、不是SDK枚举值了，直接传给
    # mid360_real.launch.py的imu_out_rate_hz参数。DLIO和point_lio共用
    # 同一路mid360/imu，这里改一处两边同时生效——point_lio的高频里程计
    # 模式(odometry.publish_odometry_without_downsample)实测10分钟内存
    # 涨3.7倍(见read_hw.md)是这次改这个的直接动机，降到100Hz对DLIO的
    # 影响可忽略(DLIO自己按独立100Hz定时器发布，不直接跟着这个走；安全
    # 网能容忍dt<0.1s即>10Hz，100Hz远在安全范围内)。
    LIDAR_IMU_OUT_RATE_HZ="${LIDAR_IMU_OUT_RATE_HZ:-100.0}"
    echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：启动真实Mid-360S驱动（livox_ros_driver2，供DLIO/point_lio/global_mapper_ros/gt_cloud_bridge_node等下游任意组合消费），LIDAR_IP=${LIDAR_IP} LIDAR_HOST_IP=${LIDAR_HOST_IP} LIDAR_SCAN_FREQ=${LIDAR_SCAN_FREQ} LIDAR_IMU_OUT_RATE_HZ=${LIDAR_IMU_OUT_RATE_HZ} =="
    cat > /tmp/mid360_config_hw.json <<EOF
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "Mid360s": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "host_ip"        : "${LIDAR_HOST_IP}",
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 56401,
        "log_data_port"  : 56501
      }
    ]
  },
  "lidar_configs" : [
    {
      "ip" : "${LIDAR_IP}",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
EOF
    # 2026-08-19新增：xfer_format按SLAM_BACKEND决定——DLIO要PointCloud2(0)，
    # point_lio要Livox CustomMsg(1)，同一个物理驱动同一时刻只能是一种格式，
    # 不像仿真Gazebo插件能并行发布两路。这里读的是${SLAM_BACKEND:-dlio}这个
    # 环境变量本身(不依赖下面LOCALIZATION_SOURCE分支里那次export，bash读
    # 环境变量不需要先export)。2026-09-08：fast_lio跟point_lio一样走
    # CustomMsg(AVIA handler)，加进同一个条件。⚠️ 这个值能不能真的传到
    # 驱动，取决于mid360_real.launch.py本地这份文件有没有把xfer_format
    # 声明成可覆盖的launch参数——这次没有改动那个文件，是否生效需要
    # 上机核实，不在这次改动的验证范围内。
    if [ "${SLAM_BACKEND:-dlio}" = "point_lio" ] || [ "${SLAM_BACKEND:-dlio}" = "fast_lio" ]; then
        _mid360_xfer_format=1
    else
        _mid360_xfer_format=0
    fi
    ros2 launch /opt/hw_launch/mid360_real.launch.py \
        namespace:="${NAMESPACE}" \
        config_path:="/tmp/mid360_config_hw.json" \
        publish_freq:="${LIDAR_SCAN_FREQ}" \
        xfer_format:="${_mid360_xfer_format}" \
        imu_out_rate_hz:="${LIDAR_IMU_OUT_RATE_HZ}" &
    sleep 2
fi

case "${LOCALIZATION_SOURCE}" in
    gt)
        # gt模式的整个存在意义就是"跳过SLAM、直接读Gazebo仿真真值"——真机上
        # 没有Gazebo、没有/plug/model_states_plug这个话题，gt_odom_bridge_node
        # 起了也是永远收不到数据、安静地空等，"看起来启动成功"但飞机压根拿不到
        # 定位。这是三个LOCALIZATION_SOURCE分支里跟"是否连仿真"绑得最死的一个
        # （uwb_imu/uwb_slam都有对应的single_*真机模式，gt没有），这里直接
        # fail fast，不要让它在真机上悄悄跑成一个没有任何数据源的空转节点。
        if [ "${DEPLOY_TARGET}" = "hw" ]; then
            echo "!! [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw 下 LOCALIZATION_SOURCE=gt 没有意义（真机没有Gazebo真值），退出 !!" >&2
            exit 1
        fi
        echo "== [flight-stack:${NAMESPACE}] 定位模式=gt：跳过DLIO，改用Gazebo仿真真值 (gt_odom_bridge) =="
        ros2 run gt_odom_bridge gt_odom_bridge_node \
            --ros-args -r __ns:="/${NAMESPACE}" -p entity_name:="${NAMESPACE}" \
            -p lidar_mount_t:="${LIDAR_MOUNT_T:-0.0,0.0,0.0}" \
            -p lidar_tilt_deg:="${LIDAR_TILT_DEG:-0.0}" &
        sleep 2
        ;;
    uwb_imu|single_uwb_imu)
        # 情景一(UWB+IMU替代SLAM，见docker_sim/多机空间坐标对齐问题_设计讨论
        # 纪要.docx第8.1节)：跳过DLIO和gt_odom_bridge_node，起uwb_imu_fusion_node
        # （UWB位置+yaw、IMU的roll/pitch原样拼装成完整位姿，直接发mavros/
        # vision_pose/pose_cov）+ local_position_readback_node（改名重发成
        # dlio/odom_node/odom+对应TF，跟gt/uwb_slam两种模式提供给下游规划栈的
        # 接口完全一样）。repub_odom节点这一步不用手动管——launch/dynus_
        # mavros.launch.py（ros2_px4_stack_skip_repub_odom_uwb_imu.patch）会读
        # 同一个LOCALIZATION_SOURCE环境变量自动跳过它，避免它跟
        # uwb_imu_fusion_node抢vision_pose/pose_cov这个话题。
        # single_uwb_imu（2026-08-13新增）：单机版，节点/行为跟uwb_imu逐字相同
        # （uwb_imu_fusion_node/local_position_readback_node只处理自己
        # namespace下的数据，从来不关心NUM_AGENTS），单独列一个case分支只是
        # 为了让上面的NUM_AGENTS=1校验和文件头文档能对上这个名字，见文件头
        # 2026-08-13说明。
        # 2026-08-13"路线A"高频化：local_position_readback_node原来读的是
        # mavros/local_position/odom（PX4内部锁步～47Hz往外吐，见DEBUG_JOURNAL.md
        # 同日"gazebo_mavlink_interface硬编码250Hz锁步"排查）。中间试过换成
        # robot_localization包的ekf_node（真正的error-state卡尔曼滤波），
        # 排查了一整圈（话题名、QoS、YAML数组写法）都没能让它的pose0_config/
        # imu0_config这些参数被节点正确识别——`ros2 service call .../
        # list_parameters`直接查询确认了这一点，具体是这个apt包版本本身的
        # 问题还是这套CycloneDDS环境的某种特殊性没有查清楚，放弃这条路，见
        # uwb_imu_fusion_node.py文件头完整记录。现在改回在uwb_imu_fusion_node
        # 里手搓的位置+速度互补滤波（第二次修订：位置的平滑时间常数从0.15秒
        # 调大到跟速度一样的0.5秒，换更强的平滑），直接发一路uwb_imu_odom
        # 给这个节点读。飞控那一路(vision_pose/pose_cov)完全不变，规划器/
        # 控制器这一路(dlio/odom_node/odom)绕开PX4锁步瓶颈直接拿高频数据。
        echo "== [flight-stack:${NAMESPACE}] 定位模式=${LOCALIZATION_SOURCE}：跳过DLIO，启动 uwb_imu_fusion_node(纯转发UWB->vision_pose) + local_position_readback_node(从飞控回读local_position/odom) =="
        # 2026-09-18新增：imu_topic外置成UWB_IMU_TOPIC环境变量，跟下面
        # DLIO_IMU_TOPIC是同一个模式(见docker-compose.hw.yml对应注释)，
        # 两个开关互相独立——改这个不影响DLIO那条链路，反之亦然。默认值
        # 同日改成mavros/imu/data_raw(飞控原始IMU，零偏干净)，理由跟
        # DLIO_IMU_TOPIC切换到mavros/imu/data_raw是同一个(mid360自带6轴
        # IMU零偏/噪声尤其Y轴陀螺明显偏大)——但2026-09-18当天真机排查
        # 实测这条MAVLink链路当时没有数据输出(见DEBUG_JOURNAL.md同日
        # "真机排查"条目)，根因还没查清楚，用前须先确认这台机器上
        # mavros/imu/data_raw确实有数据在流动。
        ros2 run uwb_origin_bridge uwb_imu_fusion_node \
            --ros-args -r __ns:="/${NAMESPACE}" \
            -p imu_topic:="${UWB_IMU_TOPIC:-mavros/imu/data_raw}" &
        # 2026-08-29：去掉 -p local_position_topic:=uwb_imu_odom 覆盖，
        # 恢复默认值 mavros/local_position/odom —— 里程计改为**从飞控回读**。
        # 原因：当初绕开飞控的理由是"PX4内部锁步~47Hz往外吐"(2026-08-13结论)，
        # 而 2026-08-27 调高PX4流速率后实测已达 94Hz，那个前提失效了。
        # 回读的收益：① 飞控和规划器看同一个位置估计（原来看到的是两个不同的
        # 估计，是系统性控制偏差的来源）；② EKF2 是成熟的 error-state EKF，
        # 比 uwb_imu_fusion_node 里手搓的互补滤波好；③ 融合节点因此不需要姿态，
        # 顺带修掉"roll/pitch恒为0"那个真bug（mid360是6轴IMU，orientation恒为
        # 单位四元数）。详见 read_hw.md 2026-08-29 条目。
        # ⚠️ 代价：多两次MAVLink串口穿越，延迟约20~50ms；且依赖 EKF2_EV_CTRL /
        # EKF2_EV_DELAY 配置正确——配错时 EKF2 会静默拒绝融合 vision，下游只
        # 看到"一个频率正常的odom"，没有任何报错。验收判据：静置时
        # mavros/local_position/odom 应收敛到跟 uwb/pose_abs（减去起点）一致。
        ros2 run gt_odom_bridge local_position_readback_node \
            --ros-args -r __ns:="/${NAMESPACE}" \
            -p lidar_mount_t:="${LIDAR_MOUNT_T:-0.0,0.0,0.0}" \
            -p lidar_tilt_deg:="${LIDAR_TILT_DEG:-0.0}" &
        sleep 2
        ;;
    uwb_slam|single_uwb_slam|single_slam_only)
        # single_uwb_slam/single_slam_only（2026-08-13新增）：DLIO启动逻辑本身
        # 跟uwb_slam逐字相同，两者的区别不在这里、在entrypoint末尾要不要起
        # uwb_origin_bridge（single_slam_only=纯SLAM单机、不依赖任何UWB，
        # 跳过；single_uwb_slam=单机也要验证UWB一键定原点，照常起），见文件头
        # 2026-08-13说明和entrypoint末尾uwb_origin_bridge那一段。
        # docker_sim 2026-08-12：这个值以前拆成两个("dlio"跟"uwb_slam")，
        # 在flight-stack这边的实际行为从一开始就完全一样——都是起真实DLIO
        # SLAM，唯一的差别在origin_setter_node.py：这个节点无条件常驻跑
        # （不看LOCALIZATION_SOURCE），一直在做SE(2)在线旋转估计，"dlio"
        # 模式下θ*照样会被算出来，只是没人特意去看yaw_estimate这个诊断
        # 话题。既然行为完全一样，拆成两个值纯粹是历史包袱（当初想用改个
        # 名字来表达"这次是真的要验证旋转对齐"这个意图，但没必要靠改
        # LOCALIZATION_SOURCE这种影响一整条落地路径的开关来传达这么轻的
        # 语义），合并成一个，"仿真真值要不要参与frame_align"这个真正独立
        # 的维度改用PUBLISH_FRAME_ALIGN单独控制，见文件头说明和
        # DEBUG_JOURNAL.md 2026-08-12"评估LOCALIZATION_SOURCE功能重复"
        # 那条。
        # DEPLOY_TARGET=hw：真实Mid-360驱动已经在上面LOCALIZATION_SOURCE
        # case语句之前统一启动过一次（覆盖DLIO/point_lio/global_mapper_ros/
        # gt_cloud_bridge_node这几条不同的下游消费路径，不止这一个分支需要
        # 它，2026-08-14自查时从"只在这里启动"改成"提到前面统一启动"，
        # 见那段注释的完整说明），这里不用再起一次。

        # SLAM_BACKEND（2026-08-14新增）：dlio=原默认，point_lio=可选后端
        # （point_lio_ros2，见docker/Dockerfile.flight-stack "7b. point_lio"
        # 一节），两者都remap到dlio/odom_node/odom、
        # dlio/odom_node/pointcloud/deskewed这两个固定话题名+{ns}/odom、
        # {ns}/base_link这两个TF frame名字，下游的name_label_node/
        # ego_planner_bridge/gt_odom_bridge/mighty/ros2_px4_stack完全不用
        # 感知这里切的是哪个后端。
        export SLAM_BACKEND="${SLAM_BACKEND:-dlio}"
        case "${SLAM_BACKEND}" in
            dlio)
                echo "== [flight-stack:${NAMESPACE}] 定位模式=${LOCALIZATION_SOURCE}：启动 DLIO（SLAM_BACKEND=dlio） =="
                # patches/dlio_namespace.patch 让 dlio.launch.py 的 namespace:= 真正生效（Node
                # 会被套上 namespace=NX01/NX02），所以 pointcloud_topic/imu_topic 这两个remap
                # 目标必须传"相对"话题名（不带NX01前缀）——节点自己的namespace会在运行时把它们
                # 解析成 /NX01/mid360_PointCloud2、/NX01/mid360/imu；如果这里还手动拼上
                # "${NAMESPACE}/..."前缀，会被namespace再套一层变成 /NX01/NX01/mid360_PointCloud2
                # 这种双重前缀，订阅不到Gazebo实际发布的 /NX01/mid360_PointCloud2。
                # 2026-08-27新增lidar_tilt_deg：雷达+IMU一体绕Y轴前倾角度，原来是
                # cfg/dlio.yaml里写死的旋转矩阵字面量(25°)，现在外置成launch参数，
                # dlio.launch.py内部按这个角度现算R_y矩阵覆盖掉yaml里的默认值(yaml
                # 里的默认值已经改成0°/单位矩阵，见该文件说明)。默认0°只是占位值
                # (不假设任何真实安装角度)，真机实际是25°，必须在.env里显式设
                # LIDAR_TILT_DEG=25.0，不能保留这个占位值就直接起飞——跟LIDAR_IP/
                # FC_SERIAL_DEVICE同样的模式。
                # 2026-09-10新增DLIO_IMU_TOPIC：mid360自带IMU实测零偏/噪声（尤其
                # Y轴陀螺噪声）明显偏大，跟飞控自己的IMU（mavros/imu/data_raw，
                # 已验证零偏干净）对比过，真机上改喂给DLIO飞控IMU的口子——
                # 跟lidar_tilt_deg同样的外置模式，默认值保持"mid360/imu"不变
                # （不改变现有行为），真机要切换在.env里显式设
                # DLIO_IMU_TOPIC=mavros/imu/data_raw。相对话题名，不带NX01
                # 前缀（原因见上面注释）。
                ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
                    namespace:="${NAMESPACE}" \
                    pointcloud_topic:="mid360_PointCloud2" \
                    imu_topic:="${DLIO_IMU_TOPIC:-mid360/imu}" \
                    lidar_tilt_deg:="${LIDAR_TILT_DEG:-0.0}" &
                sleep 3
                ;;
            point_lio)
                # 2026-08-19：hw下的fail fast已撤掉——mid360_real.launch.py
                # 的xfer_format现在可配置（上面统一启动livox驱动那段按
                # SLAM_BACKEND算好了），point_lio这边common.lid_topic也改成
                # 可覆盖的launch参数了（见point_lio_docker_sim_launch.patch
                # 2026-08-19那次修订）。真机第一次实测这条路径，
                # point_lio本身连仿真都还没验证过，真机这次是首次实测。
                echo "== [flight-stack:${NAMESPACE}] 定位模式=${LOCALIZATION_SOURCE}：启动 point_lio（SLAM_BACKEND=point_lio，2026-08-19真机首次实测） =="
                # 跟上面DLIO同样的坑：point_lio_docker_sim_launch.patch让
                # laserMapping这个Node自己namespace=namespace，lid_topic/
                # imu_topic/输出remap目标都必须是"相对"话题名，这里的
                # namespace:=只是传给launch文件去拼{ns}/odom等TF frame名字，
                # 不能再手动拼一次话题前缀。common.imu_topic仍然写死
                # "mid360/imu"（真机/仿真这路话题名一直一致，不用改）；
                # common.lid_topic：DEPLOY_TARGET=hw时传
                # lid_topic:=mid360_PointCloud2（真机driver remap的实际话题
                # 名，不管xfer_format是0还是1都是这个名字，只是消息类型不同），
                # 仿真下保留launch文件里的默认值'mid360'（Gazebo插件专属，
                # 不能传hw的话题名，仿真根本没有这个话题）。
                if [ "${DEPLOY_TARGET}" = "hw" ]; then
                    _point_lio_lid_topic_arg="lid_topic:=mid360_PointCloud2"
                else
                    _point_lio_lid_topic_arg=""
                fi
                # POINT_LIO_CPU_CORES（2026-08-14新增，默认4）——2026-08-14
                # 实测双机场景下单个pointlio_mapping进程CPU占用冲到~1150%
                # （11+核），host load average冲到52（32核机器），点云/里程计
                # 速率被拖到个位数Hz。查源码：point_lio自己和ikd-Tree都没调
                # omp_set_num_threads()限流，CMakeLists.txt里
                # find_package(OpenMP)把-fopenmp加进全局编译选项，libgomp
                # 默认按nproc开线程池；ikd_Tree.cpp:201的pthread_create另外
                # 开一条增量重建后台线程（日志里"Multi thread started"的
                # 来源）——不管具体是哪部分贡献的，taskset钉核这个手段跟
                # 具体是OpenMP还是pthread无关，直接在OS调度层面把这个进程
                # 后续开出来的所有线程都摁在固定核范围内。按AGENT_INDEX
                # (1-based)错开NX01/NX02各自的核范围，避免两架飞机的
                # point_lio抢同一批核（跟Gazebo/mavros/ego_planner等同容器
                # 内其它进程之间没有隔离，taskset只管这一个进程）。
                # OMP_NUM_THREADS同时设成一样的值，直接封顶OpenMP线程池
                # 大小（即使taskset已经在调度层面限制了核数，libgomp线程池
                # 大小本身默认还是按nproc算的，线程数远多于能跑的核数会有
                # 额外的上下文切换开销，两个都设更稳妥）。
                POINT_LIO_CPU_CORES="${POINT_LIO_CPU_CORES:-4}"
                # 2026-09-06新增：mapping_mid360.launch.py的high_freq_odom参数
                # （对应odometry.publish_odometry_without_downsample）此前
                # entrypoint从未传递，永远吃launch文件自己的默认值false，没有
                # 任何地方能打开它——外置成环境变量，跟POINT_LIO_CPU_CORES同一个
                # 模式。
                # ⚠️⚠️ 2026-09-06真机实测确认：这个模式目前不可用，不要在任何
                # 实际场景下打开——环境变量/launch参数确认生效、频率确认提升到
                # ~100Hz，但发布的位置/速度不跟随真实运动，飞机被实际移动时
                # 数值只有1-2厘米量级噪声，不反映真实位移。已排除开关未生效/
                # 频率未提升/use_imu_as_input分支不匹配这几种可能，根因还在
                # point_lio上游`publish_odometry_without_downsample`实现更深层，
                # 需要实际调试才能继续，详见DEBUG_JOURNAL.md/read_hw.md同日
                # 记录。此前的内存增长风险（200Hz IMU+高频10分钟+437MB，100Hz
                # IMU下+2.6MB）依然有效，但现在有更优先的不可用结论——默认
                # false不变，在根因定位修复之前不要打开。
                POINT_LIO_HIGH_FREQ_ODOM="${POINT_LIO_HIGH_FREQ_ODOM:-false}"
                _point_lio_core_start=$(( (AGENT_INDEX - 1) * POINT_LIO_CPU_CORES ))
                _point_lio_core_end=$(( _point_lio_core_start + POINT_LIO_CPU_CORES - 1 ))
                echo "== [flight-stack:${NAMESPACE}] point_lio限制在CPU核${_point_lio_core_start}-${_point_lio_core_end}（POINT_LIO_CPU_CORES=${POINT_LIO_CPU_CORES}，按AGENT_INDEX=${AGENT_INDEX}错开，POINT_LIO_HIGH_FREQ_ODOM=${POINT_LIO_HIGH_FREQ_ODOM}） =="
                OMP_NUM_THREADS="${POINT_LIO_CPU_CORES}" taskset -c "${_point_lio_core_start}-${_point_lio_core_end}" \
                    ros2 launch point_lio mapping_mid360.launch.py \
                    namespace:="${NAMESPACE}" \
                    rviz:="false" \
                    high_freq_odom:="${POINT_LIO_HIGH_FREQ_ODOM}" \
                    ${_point_lio_lid_topic_arg} &
                sleep 3
                ;;
            fast_lio)
                # 2026-09-07新增，当时仅sim——fast_lio_docker_sim_launch.patch
                # 新增的launch/fast_lio_docker_sim.launch.py是仿真专属。
                # 2026-09-08：接通hw——新增fast_lio_hw_launch.patch/
                # launch/fast_lio_hw.launch.py，去掉原来无条件fail fast的
                # guard。接通不等于验证过：fast_lio目前只在仿真里做过部分
                # 实测（QoS/点云稀疏两个问题已修，地图无界增长那个问题
                # 缓解措施效果待确认，见DEBUG_JOURNAL.md），真机这条路径
                # 是首次接入，跟point_lio当年"真机首次实测"是同一个阶段，
                # 不是"已验证可用"。
                # ⚠️ 已知限制：本地这份docker/entrypoints/hw_launch/
                # mid360_real.launch.py目前xfer_format仍是硬编码的Python
                # 字面量0，不是可覆盖的launch参数——下面统一启动真实
                # Mid-360驱动那段传的xfer_format:="${_mid360_xfer_format}"
                # 对这份本地文件不会生效，驱动实际会一直发PointCloud2格式，
                # 不是fast_lio/point_lio需要的CustomMsg。这个限制point_lio
                # 也一样受影响，不是fast_lio独有的新问题，这次没有改动
                # mid360_real.launch.py本身（不在这次任务授权范围内）。
                if [ "${DEPLOY_TARGET}" = "hw" ]; then
                    _fast_lio_lid_topic_arg="lid_topic:=mid360_PointCloud2"
                    _fast_lio_launch_file="fast_lio_hw.launch.py"
                    echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：启动 FAST-LIO2（SLAM_BACKEND=fast_lio，2026-09-08真机首次接通，未经真机实测验证） =="
                else
                    _fast_lio_lid_topic_arg=""
                    _fast_lio_launch_file="fast_lio_docker_sim.launch.py"
                    echo "== [flight-stack:${NAMESPACE}] 定位模式=${LOCALIZATION_SOURCE}：启动 FAST-LIO2（SLAM_BACKEND=fast_lio，仅做过部分仿真实测） =="
                fi
                # 跟DLIO/point_lio同样的坑：fast_lio的两份launch文件都让
                # fastlio_mapping这个Node自己namespace=namespace，lid_topic/
                # 输出remap目标都必须是"相对"话题名，这里的namespace:=只是
                # 传给launch文件去拼{ns}/odom等TF frame名字，不能再手动拼
                # 一次话题前缀。fast_lio没有point_lio那种taskset/CPU核数
                # 限制——point_lio加这个是2026-08-14双机实测CPU冲到1150%
                # 之后才补的针对性修复，fast_lio这边2026-09-07仿真实测过
                # loadavg（32核机器1.94，远没占满），没有同样的证据支撑，
                # 这里不照搬taskset，避免加一个没有依据的"优化"。
                ros2 launch fast_lio "${_fast_lio_launch_file}" \
                    namespace:="${NAMESPACE}" \
                    ${_fast_lio_lid_topic_arg} &
                sleep 3
                ;;
            *)
                echo "!! [flight-stack:${NAMESPACE}] SLAM_BACKEND=${SLAM_BACKEND} 不是已知值(dlio/point_lio/fast_lio)，退出 !!" >&2
                exit 1
                ;;
        esac
        ;;
    *)
        echo "!! [flight-stack:${NAMESPACE}] 未知的 LOCALIZATION_SOURCE='${LOCALIZATION_SOURCE}'，只接受 gt/uwb_slam/uwb_imu/single_slam_only/single_uwb_imu/single_uwb_slam，退出 !!" >&2
        exit 1
        ;;
esac

echo "== [flight-stack:${NAMESPACE}] 启动 name_label_node（RViz里头顶跟随的名字标注）=="
# 订阅dlio/odom_node/odom（跟上面LOCALIZATION_SOURCE=uwb_slam/gt两种模式发布
# 的是同一个话题名，不用关心当前用的是哪种定位源），发布一个跟着飞机位置走的
# TEXT_VIEW_FACING marker，文字默认取namespace（NX01/NX02）。
ros2 run gt_odom_bridge name_label_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

# Gazebo雷达插件livox_points_plugin.cpp给点云header.frame_id用的是
# "${NAMESPACE}/${NAMESPACE}_livox"（SDF里sensor名字，见gen_iris_mid360_sdf.py），
# 跟DLIO/gt_odom_bridge发布的雷达外参TF"${NAMESPACE}/lidar"是两套不搭界的
# 命名习惯，中间没有任何TF连起来。补一条恒等静态TF把两个名字接上（物理上是
# 同一个传感器、同一个位置，纯粹是两边代码各自叫法不同）。原来只在
# PLANNER=mighty分支里发布（global_mapper_ros才用得到），现在提到分支之前
# 无条件发布——PLANNER=ego_planner+LOCALIZATION_SOURCE=gt时，新增的
# gt_cloud_bridge_node同样要沿这条别名做TF查找，两个规划器共用同一条别名，
# 不用各自维护一份。节点名带上${NAMESPACE}前缀，避免两架飞机撞节点名
# （之前`ros2 node list`实测确认过"share an exact name"警告）。
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 "${NAMESPACE}/lidar" "${NAMESPACE}/${NAMESPACE}_livox" \
    --ros-args -r __node:="${NAMESPACE}_static_tf_livox_alias" &

if [ "${PLANNER}" = "mighty" ]; then
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
# 完全不搭界的名字，中间没有任何TF把它们连起来。这条恒等别名TF已经在前面
# （PLANNER分支之前）无条件发布了，global_mapper_node查
# "NX01/map -> NX01/NX01_livox"时能沿着map->odom->base_link->lidar->NX01_livox
# 这条完整链路解出来。

echo "== [flight-stack:${NAMESPACE}] 启动 global_mapper_ros（原始点云->occupancy_grid/unknown_grid）=="
ros2 launch global_mapper_ros global_mapper_node.launch.py \
    quad:="${NAMESPACE}" hardware:=false \
    global_frame:="${NAMESPACE}/map" drone_frame:="${NAMESPACE}/lidar" \
    depth_pointcloud_topic:=mid360_PointCloud2 pose_topic:=state &
sleep 2

# 2026-08-10用户反馈：RViz里指点目标、看目标点/规划轨迹时，只要Fixed Frame
# 不是当前飞机自己的frame，显示就会有一个跟出生点偏移量对得上的错位——查了
# global_mapper_ros.cc（只有tf2_ros::Buffer/TransformListener，grep
# TransformBroadcaster/sendTransform零命中，只订阅TF、从不发布）确认：跟
# 2026-08-08那次ego_planner"点云/轨迹显示不出来"是同一类缺口，只是这次的
# 表现不是"完全画不出来"而是"画错位置"——mighty_node.cpp的目标点/轨迹可视化
# marker全部用`marker.header.frame_id = par_.map_frame_id`（也就是
# "${NAMESPACE}/map"，因为上面use_hardware:=true已经生效），但
# "${NAMESPACE}/map"和"${NAMESPACE}/odom"这两个名字之间，整条mighty链路
# （global_mapper_ros/mighty_node本身/DLIO）里没有任何一处发布过TF——UWB的
# frame_align机制只桥接了"NX01/map<->NX02/map"两机的map之间，管不到单机
# 自己map<->odom这一环。RViz渲染时要从marker的frame_id沿TF树查到Fixed
# Frame，这一环缺失，Fixed Frame设成不是"${NAMESPACE}/map"本身的任何值
# （包括这架飞机自己的odom）时，要么画不出来要么退化成某个不对的默认
# 变换——具体走哪条取决于RViz内部对unresolvable TF的兜底行为，没有深究，
# 反正结果都是错的。
# 修复：照抄ego_planner那边同一个位置已经验证过的做法——补一条恒等静态TF
# "${NAMESPACE}/map"<->"${NAMESPACE}/odom"，零偏移。这里同样成立"数值上是
# 同一个坐标系"的前提：global_mapper_ros喂给它的点云/位姿全部来自DLIO的
# "${NAMESPACE}/odom"（pose_topic:=state读的是convert_odom_to_state转发的
# DLIO里程计，不是另一条独立SLAM/回环修正过的位姿），mighty自己没有对
# "map"做任何额外的漂移修正，"map"和"odom"从一开始就是同一份数据的两个
# 名字，零偏移完全合适，不是凑合的近似值。
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 "${NAMESPACE}/map" "${NAMESPACE}/odom" \
    --ros-args -r __node:="${NAMESPACE}_static_tf_mighty_map_odom" &

# 2026-08-10用户再次实测反馈：补了上面那条map<->odom的TF之后，"显示错位"
# 还在——回头查错了层：上面那条TF解决的是"RViz怎么把已经算对的坐标画
# 出来"（渲染层），但用户描述的"固定偏移"、"给另一架飞机点目标时会差一个
# 平移量"，根子在更上游：**目标点数据本身就没算对**，RViz显示只是如实
# 反映了这个错误数据。
# 查了multi_mighty.rviz里两个"2D Goal Pose"工具（rviz_default_plugins/
# SetGoal），Topic直接硬编码"/NX01/term_goal"/"/NX02/term_goal"——RViz这
# 类工具发布的PoseStamped，pose.position永远是"点击位置在当前Fixed Frame
# 下的原始数值"，header.frame_id填的是当前Fixed Frame的名字。而
# mighty_node.cpp的sub_terminal_goal_回调（terminalGoalCallbackImpl）直接
# 读msg.pose.position.x/y，完全不看msg.header.frame_id、不做任何TF变换，
# 把这个原始数值当成"已经是这架飞机自己odom系下的坐标"直接使用——Fixed
# Frame恰好等于这架飞机自己的frame时凑巧算对，切到任意别的frame（尤其是
# "给另一架飞机点目标"这种场景，天然会先切到那架飞机的frame再点）就会
# 把"点击位置在那个frame下的数值"错当成"这架飞机自己frame下的数值"，
# 差出两个frame原点之间的平移量——这跟2026-08-08 ego_planner那次"目标点
# 送错地方"是完全同一个bug模式，而且那次已经有一个验证过、可以直接复用
# 的修复：`ego_planner_bridge`包里的`rviz_goal_bridge_node`（订阅
# rviz_goal_world、用tf2把消息自带frame_id变换到"<namespace>/odom"、
# 发布term_goal），这个节点的实现本身完全不依赖ego_planner的任何东西
# （只用namespace算target_frame，topic都是相对名），mighty这边直接复用
# 同一个可执行文件，不需要另写一份。/opt/ego_planner_ws/install/setup.bash
# 在脚本最上面已经无条件source过，这里能直接ros2 run/launch。
# 配套改了multi_mighty.rviz两个SetGoal工具的Topic，从直接绑
# "/NX0x/term_goal"改成绑"/NX0x/rviz_goal_world"，让点击先经过这个节点
# 转换再进mighty_node.cpp，不再是直接绕过去。
echo "== [flight-stack:${NAMESPACE}] 启动 rviz_goal_bridge_node（RViz 2D Goal Pose -> TF变换 -> term_goal，mighty复用ego_planner_bridge同一个节点）=="
ros2 run ego_planner_bridge rviz_goal_bridge_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

elif [ "${PLANNER}" = "ego_planner" ]; then
echo "== [flight-stack:${NAMESPACE}] 启动 ego_planner（ego-planner-swarm规划器核心，drone_id=${DRONE_ID}）=="
# 前端直接吃dlio/odom_node/odom + dlio/odom_node/pointcloud/deskewed（跟
# mighty同一份数据源的odom部分，点云则是DLIO已经配准到odom系的版本——
# ego_planner自己的plan_env/grid_map.cpp不做TF lookup，纯粹信任收到的点云
# header.frame_id就是odom系，不需要global_mapper_ros那套TF查找）。
# 后端发相对话题名position_cmd（ego_planner_traj_server_relative_poscmd.patch
# 打过）。2026大赛任务系统阶段3起，px4ctrl_bridge那边不再直接remap接
# position_cmd，中间插了position_cmd_relay_node（下面紧接着启动）转一手，
# 默认normal模式下行为透传等价，见该节点文件头拓扑说明。
ros2 launch ego_planner_bridge ego_planner_docker_sim.launch.py \
    namespace:="${NAMESPACE}" drone_id:="${DRONE_ID}" &
sleep 2

# 2026大赛任务系统阶段3：position_cmd中继/仲裁节点，只在PLANNER=ego_planner
# 时启动（mighty模式下traj_server不存在，goal_to_poscmd直接发到'cmd'相对名，
# 中继在这个拓扑里没有插入点，见执行方案阶段0锁定PLANNER=ego_planner的
# 理由）。这里不区分CONTROLLER（px4ctrl/so3ctrl/pt4ctrl三条launch文件的
# ego_planner分支都已经改成接position_cmd_relayed），一份中继节点服务
# 无论最终起的是哪个控制器。
echo "== [flight-stack:${NAMESPACE}] 启动 position_cmd_relay（阶段3中继/仲裁节点，relay_mode默认normal） =="
ros2 run contest_mission position_cmd_relay_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

# 2026大赛任务系统阶段4.1：立柱检测节点，同样只在PLANNER=ego_planner时
# 启动——订阅的'grid_map/occupancy'是ego_planner的plan_env/grid_map.cpp
# 发布的，mighty模式下没有这个话题（mighty有自己独立的建图逻辑，不共用
# 这个话题名），启动了也是空等，不如干脆不起。
echo "== [flight-stack:${NAMESPACE}] 启动 pillar_detector_node（阶段4.1立柱检测） =="
ros2 run contest_mission pillar_detector_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

# 2026-09-09新增：高层着火点瞄准节点——检测到贴在某根立柱上的AprilTag
# ID1之后，飞到"正对贴tag那个面、机头对准中心"的悬停点，同时算出任务机
# 等待点。依赖pillar_candidates（上面pillar_detector_node发的，带朝向）
# +vision/detections（阶段2检测节点）+dlio/odom_node/odom，只在
# PLANNER=ego_planner时有意义，同样放在这个分支里。
echo "== [flight-stack:${NAMESPACE}] 启动 fire_pillar_aim_node（高层着火点瞄准+任务机等待点计算） =="
ros2 run contest_mission fire_pillar_aim_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

# 2026大赛任务系统阶段A3新增：编队跟随节点——每架飞机都要起（角色是
# 运行时参数，不是机身编号决定的，见方案1.4.2节/1.5节双机对称性
# 原则），默认enabled=False常驻待命，不订阅任何长机、不发布任何目标，
# 需要SDK调一次~/set_parameters（leader_namespace+enabled=true）才
# 真正开始跟随。跟fire_pillar_aim_node同样只在PLANNER=ego_planner时
# 有意义（目标点最终发给rviz_goal_world，由ego_planner_bridge的
# rviz_goal_bridge_node消费，mighty模式下没有这条链路），所以放在
# 同一个分支里，不需要额外判断LOCALIZATION_SOURCE（uwb_imu/gt/
# single_uwb_imu三种定位源下这个节点的坐标处理逻辑完全一样，见文件
# 头说明——uwb_imu模式下两机局部系天然对齐，不做任何坐标转换）。
echo "== [flight-stack:${NAMESPACE}] 启动 formation_follower_node（阶段A3编队跟随，默认enabled=False待命） =="
ros2 run contest_mission formation_follower_node \
    --ros-args -r __ns:="/${NAMESPACE}" &

if [ "${LOCALIZATION_SOURCE}" = "gt" ] || [ "${LOCALIZATION_SOURCE}" = "uwb_imu" ] || [ "${LOCALIZATION_SOURCE}" = "single_uwb_imu" ]; then
    echo "== [flight-stack:${NAMESPACE}] 定位模式=${LOCALIZATION_SOURCE}：启动 gt_cloud_bridge_node（原始点云->odom系，补上DLIO deskewed话题的等价物）=="
    # gt/uwb_imu(含single_uwb_imu)三种模式下DLIO都不跑，dlio/odom_node/pointcloud/deskewed这个
    # 话题不存在——ego_planner的grid_map.cpp又不做TF lookup、必须直接收到已经
    # 配准到odom系的点云。这个节点订阅原始mid360_PointCloud2（frame_id=
    # "${NAMESPACE}/${NAMESPACE}_livox"），沿上面无条件发布的
    # "${NAMESPACE}/lidar -> ${NAMESPACE}/${NAMESPACE}_livox"别名+
    # gt_odom_bridge_node/local_position_readback_node发布的"${NAMESPACE}/odom
    # -> ${NAMESPACE}/base_link"+"${NAMESPACE}/base_link -> ${NAMESPACE}/lidar"
    # 这条完整TF链（uwb_imu模式下由local_position_readback_node提供，格式跟
    # gt_odom_bridge_node发的完全一样，这个节点不用关心背后是哪种定位源），
    # 把点云变换到odom系，重新发到同一个话题名——ego_planner_docker_sim.launch.py
    # 的remap不用区分dlio/gt/uwb_imu，三种模式下这个话题名都存在。
    ros2 run gt_odom_bridge gt_cloud_bridge_node \
        --ros-args -r __ns:="/${NAMESPACE}" &
fi

# grid_map/frame_id="${NAMESPACE}/map"只是occupancy_inflate等消息header里的
# 一个字符串标签，ego_planner自己从不发对应的TF——2026-08-08用户反馈
# multi_ego_planner.rviz里点云/建图/轨迹全都显示不出来，`ros2 topic echo /tf`
# 实测确认：DLIO正常发布"${NAMESPACE}/odom -> ${NAMESPACE}/base_link"，
# UWB frame_align机制（跟PLANNER无关，一直在跑）也正常发布
# "NX01/map -> NX02/map"把两机的map连在一起，但"${NAMESPACE}/map"跟
# "${NAMESPACE}/odom"这两个名字之间从来没有任何TF——RViz要渲染一条消息，
# 需要从消息的frame_id沿着TF树连到Fixed Frame，这一环缺失导致所有
# frame_id="${NAMESPACE}/map"的显示内容（occupancy_inflate等）永远连不到
# Fixed Frame，等于什么都画不出来；同时因为两机的map->odom各自都缺这一环，
# NX01/NX02也没法通过"NX01/map<->NX02/map"这条已有的桥连起来一起显示。
# 补一条恒等静态TF——ego_planner里grid_map/odom直接remap吃DLIO的odom
# （见ego_planner_docker_sim.launch.py），"map"和"odom"在这套集成里数值上
# 就是同一个坐标系，没有实际的位姿差异，零偏移刚好合适。
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 "${NAMESPACE}/map" "${NAMESPACE}/odom" \
    --ros-args -r __node:="${NAMESPACE}_static_tf_ego_planner_map_odom" &

# 2026-08-27新增，2026-08-28收窄范围：只给地面站/RViz显示用的障碍点云
# (grid_map/occupancy_inflate)高度过滤，DEPLOY_TARGET=hw才起——封闭空间
# 飞行时天花板点云会挡住俯视图，只滤掉给人看的这一路，原始话题完全不受
# 影响，避障用的还是没过滤过的完整点云。2026-08-28不再碰dlio/odom_node/
# pointcloud/deskewed(SLAM里程计模块自己输出的点云)——排查DLIO段错误崩溃
# 那次已经确认这个过滤节点架构上不影响DLIO(只读订阅，没有反馈回路)，但
# SLAM输出没有必要被这类纯展示用途碰，去掉更干净，也省下几乎一半的CPU
# 开销(deskewed那路点云量比occupancy_inflate大得多)。
# PCLOUD_FILTER_ENABLED新增开关(默认true)：不需要看点云画面时传false
# 关掉，不用再手动kill进程(之前没有开关，容器重建/重启会把手动kill的
# 进程带回来)。PCLOUD_MIN_Z/PCLOUD_MAX_Z是**相对飞机当前高度**(读dlio/
# odom_node/odom)的偏移量米数，不是odom/map坐标系下的绝对高度——飞机
# 爬升/下降时这个窗口跟着一起动。现场层高/空间跟这两个默认值(-0.2/1.0)
# 不一样时要在compose/.env里改。
if [ "${DEPLOY_TARGET}" = "hw" ]; then
    echo "== [flight-stack:${NAMESPACE}] 启动 pointcloud_z_filter（仅障碍点云给地面站/RViz显示用，enabled=${PCLOUD_FILTER_ENABLED:-true} 相对飞机当前高度min_z=${PCLOUD_MIN_Z:--0.2} max_z=${PCLOUD_MAX_Z:-1.0}，不影响避障原始点云） =="
    ros2 launch pointcloud_z_filter pointcloud_z_filter.launch.py \
        namespace:="${NAMESPACE}" \
        enabled:="${PCLOUD_FILTER_ENABLED:-true}" \
        min_z:="${PCLOUD_MIN_Z:--0.2}" \
        max_z:="${PCLOUD_MAX_Z:-1.0}" &
fi

if [ "${CONTROLLER}" = "ros2_px4_stack" ]; then
    echo "== [flight-stack:${NAMESPACE}] 启动 poscmd_to_goal_node（ego_planner的position_cmd -> ros2_px4_stack吃的Goal）=="
    # px4ctrl/so3ctrl直接remap('cmd','position_cmd')吃ego_planner发的
    # PositionCommand，但ros2_px4_stack的track_dynus_traj只认dynus_interfaces/
    # Goal、订阅绝对话题名/${VEH_NAME}/goal，两者字段语义一一对应，跟
    # px4ctrl_bridge的goal_to_poscmd刚好是反方向的同类桥接。
    ros2 run ego_planner_bridge poscmd_to_goal_node \
        --ros-args -r __ns:="/${NAMESPACE}" &
fi

fi

# ════════════════════════════════════════════════════════════════════════
# 2026-08-29：改成按 LOCALIZATION_SOURCE 判断——只在**非 uwb_imu 系列**时启动。
#
# ⚠️ 判断维度必须是 LOCALIZATION_SOURCE，不是 CONTROLLER。
# repub_odom 的职责是"把 dlio/odom_node/odom 喂给飞控的 vision_pose/pose_cov"，
# 这件事**跟用哪个控制器无关，只跟定位源有关**：
#   · uwb_imu / single_uwb_imu → uwb_imu_fusion_node 自己发 vision_pose，
#     repub_odom 会抢同一话题（实测 189.66Hz ≈ 两个发布者），必须跳过
#   · uwb_slam / single_uwb_slam / single_slam_only / gt
#       → **repub_odom 是把 SLAM 定位喂给 EKF2 的唯一桥梁**，绝不能跳过，
#         跳了飞控就没有任何外部位置参考、EKF2 直接发散
# （本文件曾一度写成按 CONTROLLER 判断，那会打断上面三个 SLAM 模式的
#  vision_pose 链路。全仓库只有 repub_odom 和 uwb_imu_fusion_node 两个
#  vision_pose 发布者，别的都没有。）
#
# 这个 launch 文件是 dynus 项目原本配套的（动捕 + Fast-LIO + D455 实验室环境），
# 它启动的节点在 uwb_imu 系列下**全部无用或有害**：
#
#   · repub_odom           ❌ 有害。订阅 dlio/odom_node/odom 转发到
#                             mavros/vision_pose/pose_cov，跟 uwb_imu_fusion_node
#                             抢同一话题（实测该话题 189.66Hz ≈ 两个发布者叠加）。
#                             2026-08-29 里程计改「从飞控回读」后更成致命闭环：
#                             EKF2 -> local_position/odom -> local_position_readback
#                             -> dlio/odom_node/odom -> repub_odom -> vision_pose
#                             -> EKF2，正反馈；实测静止飞机高度以 0.30m/s 下坠到 -185m。
#   · mocap_to_livox_frame ❌ 无用。需要动捕系统，本项目没有。
#   · 6 条静态 TF          ❌ 无用。实测 TF 树里是孤立分支
#                             (world -> world_mocap -> NX01/init_pose，下游无人使用)，
#                             我们的栈全用带命名空间的帧(NX01/map -> NX01/odom ->
#                             NX01/base_link -> NX01/lidar)。而且其中两个
#                             static_transform_publisher **重名 map_to_odom**，
#                             ROS2 里后启动的顶掉先启动的——该文件的既有 bug。
#   · track_dynus_traj     — 本来就被 RUN_OFFBOARD_FOLLOWER=false 跳过。
#
# ⚠️ mavros 本身**不是**这个文件启动的（尽管名字叫 dynus_mavros）——真正启动
# mavros 的是上面的 ${MAVROS_LAUNCH_FILE}（2026-09-09起sim/hw统一变成运行时
# 生成的 /tmp/px4_contest.launch，见上面那段解禁distance_sensor插件的
# python脚本，不再是分开的px4.launch/px4_hw.launch两份）。所以跳过这个
# 文件完全不影响 mavros。
#
# 收益：省 8 个节点（实测 repub_odom 28.3% + mocap_to_livox 19.5% ≈ 0.5 核）、
# 消除 EKF2 闭环、TF 树不再有孤立分支和重名节点。
#
# 另：patches/ros2_px4_stack_skip_repub_odom_uwb_imu.patch 原本想在 uwb_imu
# 模式下跳过 repub_odom，但实测**它从来没应用成功过**（排在 31 个 patch 链的
# 后段，上下文早被前面的 patch 改动，git apply 一直失败，而 Dockerfile 用
# `|| echo` 兜底、失败不中断 build）。该 patch 保留不动——ros2_px4_stack 模式下
# 仍有意义；当前这条 if 已从更外层解决问题，不再依赖它。
# ════════════════════════════════════════════════════════════════════════
if [ "${LOCALIZATION_SOURCE}" != "uwb_imu" ] && [ "${LOCALIZATION_SOURCE}" != "single_uwb_imu" ]; then
    echo "== [flight-stack:${NAMESPACE}] 启动 ros2_px4_stack 支撑节点 (repub_odom把${LOCALIZATION_SOURCE}的odom喂给飞控vision_pose；RUN_OFFBOARD_FOLLOWER=${RUN_OFFBOARD_FOLLOWER}时一并起track_dynus_traj) =="
    ros2 launch ros2_px4_stack dynus_mavros.launch.py \
        namespace:="${NAMESPACE}" fcu_url:="${MAVROS_FCU_URL}" &
else
    echo "== [flight-stack:${NAMESPACE}] LOCALIZATION_SOURCE=${LOCALIZATION_SOURCE}：跳过 dynus_mavros.launch.py（uwb_imu系列由uwb_imu_fusion_node发vision_pose，repub_odom会抢同一话题并形成EKF2闭环，见上方注释） =="
fi

# DEPLOY_TARGET=hw时px4ctrl换成px4ctrl_hw.launch.py（no_RC=false，需要真实
# 遥控器，2026-08-14新增）；pt4ctrl同理换成pt4ctrl_hw.launch.py（2026-08-29
# 新增）；so3ctrl同理换成so3ctrl_hw.launch.py（2026-09-06新增，逐字照抄
# px4ctrl_hw.launch.py的模式）。三个都已经在上面CONTROLLER校验那个case块里
# 确认过hw launch文件存在（不存在的CONTROLLER值会在那里直接fail fast，
# 不会执行到这里），这里只负责按DEPLOY_TARGET二选一。
case "${CONTROLLER}" in
    px4ctrl)
        if [ "${DEPLOY_TARGET}" = "hw" ]; then
            echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：启动 px4ctrl_hw（no_RC=false，需要真实遥控器已按RC_MAP_*配好） =="
            ros2 launch px4ctrl_bridge px4ctrl_hw.launch.py &
        else
            echo "== [flight-stack:${NAMESPACE}] 启动 px4ctrl (ROS2版，实验性——见README已知待办) =="
            ros2 launch px4ctrl_bridge px4ctrl_docker_sim.launch.py &
        fi
        ;;
    so3ctrl)
        if [ "${DEPLOY_TARGET}" = "hw" ]; then
            echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：启动 so3ctrl_hw（no_RC=false，需要真实遥控器已按RC_MAP_*配好）⚠️ so3ctrl_hw本身仍是未实测状态，首次使用务必先不装桨走全流程 =="
            ros2 launch px4ctrl_bridge so3ctrl_hw.launch.py &
        else
            echo "== [flight-stack:${NAMESPACE}] 启动 so3ctrl (px4ctrl飞行状态机+kr_mav_control的SO3控制律，实验性——见README已知待办) =="
            ros2 launch px4ctrl_bridge so3ctrl_docker_sim.launch.py &
        fi
        ;;
    pt4ctrl)
        if [ "${DEPLOY_TARGET}" = "hw" ]; then
            echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：启动 pt4ctrl_hw（no_RC=false，需要真实遥控器已按RC_MAP_*配好）⚠️ pt4ctrl本身仍是未实测状态，首次使用务必先不装桨走全流程 =="
            ros2 launch px4ctrl_bridge pt4ctrl_hw.launch.py &
        else
            echo "== [flight-stack:${NAMESPACE}] 启动 pt4ctrl (px4ctrl飞行状态机+砍掉控制律、改发轨迹setpoint给PX4自己的位置控制环，2026-08-13新增、全新实现，未实测——见DEBUG_JOURNAL.md) =="
            ros2 launch px4ctrl_bridge pt4ctrl_docker_sim.launch.py &
        fi
        ;;
esac

# 2026-08-13：LOCALIZATION_SOURCE=single_slam_only（纯SLAM单机、不依赖任何
# UWB硬件，见文件头说明）是唯一一个不起uwb_origin_bridge的模式——这个包的
# origin_setter/frame_align_bridge两个节点在没有UWB发布uwb/pose_abs时不会
# 报错（只是安静等数据），但语义上"完全没有UWB硬件"这种部署形态不应该起
# 一个功能上依赖UWB的包。其余所有模式（含gt/uwb_imu/uwb_slam三个非single_*
# 的原值）继续无条件启动，不受这次改动影响。
if [ "${LOCALIZATION_SOURCE}" != "single_slam_only" ]; then
    # DEPLOY_TARGET=hw：仿真模式下uwb/pose_abs由uwb_sim（Gazebo真值+高斯
    # 噪声模拟）发布，真机模式下要换成真实UWB驱动nlink_uwb_bridge（2026-
    # 08-13新增，一直编译进镜像但没有被entrypoint启动过）。跟single_slam_
    # only判断条件相同（那个模式下uwb_origin_bridge本来就不启动，UWB驱动
    # 也没有存在的必要），放在uwb_origin_bridge之前启动，让它订阅时已经有
    # 发布者。
    #
    # 2026-08-18双标签定位定向改造：这台Jetson实际接了两个LinkTrack标签
    # (uwb_a=USB/CH340那路，uwb_b=板载UART/ttyTHS0那路)，nlink_uwb_bridge
    # 现在一次launch两路（见nlink_uwb_bridge.launch.py文件头说明），各自
    # 发布到uwb_a/pose_abs、uwb_b/pose_abs；再起uwb_dual_tag_fusion_node
    # 消费这两路解算出水平位置+yaw（高度取飞控距离传感器，见该文件头算法
    # 说明），发布到uwb_imu_fusion_node/origin_setter_node已经在用的
    # uwb/pose_abs标准接口——这两个下游节点因此完全不用改。UWB_BASELINE_
    # LENGTH_M是两个标签实际安装间距(米)，只用于yaw解算合理性校验，
    # 2026-08-18现场实测=0.29米，默认值已同步；如果之后改了安装方案，
    # 记得连同dual_tag_fusion_node.py里的默认值一起改成新的实测值。
    # 2026-08-26新增UWB_YAW_OFFSET_DEG：uwb_a->uwb_b连线方向转多少度才是
    # 机头朝向，原来是dual_tag_fusion_node.py里硬编码的90°字面量，现在
    # 外置成参数，默认改成135°——这个值是跟uwb_a/uwb_b实际安装角度绑定的，
    # 装法变了要跟着改，见dual_tag_fusion_node.py文件头算法说明。
    # ⚠️ nlink_pose_bridge_node.py
    # 文件头标注了2个未验证假设（quaternion分量顺序/pos_3d单位，协议帧
    # 类型那条已实测确认），接线后务必现场核实，见docker_sim/
    # 真机部署操作清单.md"3.6 裸机联调"一节。
    if [ "${DEPLOY_TARGET}" = "hw" ]; then
        echo "== [flight-stack:${NAMESPACE}] DEPLOY_TARGET=hw：启动真实UWB驱动，双标签uwb_a(USB/CH340)+uwb_b(板载UART)一起拉起（nlink_uwb_bridge） =="
        ros2 launch nlink_uwb_bridge nlink_uwb_bridge.launch.py \
            namespace:="${NAMESPACE}" \
            port_name_a:="${UWB_A_SERIAL_DEVICE:-/dev/ttyCH341USB0}" \
            port_name_b:="${UWB_B_SERIAL_DEVICE:-/dev/ttyTHS0}" \
            baud_rate:="${UWB_BAUD_RATE:-921600}" &
        sleep 2
        echo "== [flight-stack:${NAMESPACE}] 启动 uwb_dual_tag_fusion_node（uwb_a+uwb_b双标签解算水平位置+yaw，高度取飞控距离传感器，发布到uwb/pose_abs） =="
        # 2026-09-04：这两个值必须以浮点字面量传下去。ROS2 的 `-p name:=value`
        # 按字面量推断类型，环境变量里写成整数（比如 UWB_YAW_OFFSET_DEG=135）会被
        # 解析成 INTEGER，与 dual_tag_fusion_node.py 里
        # declare_parameter('yaw_offset_deg', 135.0) 声明的 DOUBLE 冲突，节点抛
        # InvalidParameterTypeException 启动即退出（[ros2run]: Process exited with
        # failure 1），而且不会重启。这个故障非常难认：uwb_a/uwb_b/pose_abs 照常
        # 50Hz，只有融合后的 uwb/pose_abs 永远没数据，从话题层面看像"UWB 还没起来"。
        # 见 read_hw.md 2026-09-04 条目。printf 兜底，环境变量写整数也不会再崩。
        _uwb_baseline_length_m="$(printf '%.6f' "${UWB_BASELINE_LENGTH_M:-0.29}")"
        _uwb_yaw_offset_deg="$(printf '%.6f' "${UWB_YAW_OFFSET_DEG:-135.0}")"
        ros2 run uwb_origin_bridge dual_tag_fusion_node \
            --ros-args -r __ns:="/${NAMESPACE}" \
            -p baseline_length_m:="${_uwb_baseline_length_m}" \
            -p yaw_offset_deg:="${_uwb_yaw_offset_deg}" &
        sleep 1
    fi

    echo "== [flight-stack:${NAMESPACE}] 启动 uwb_origin_bridge（起飞点一键锁定+frame_align_bridge跨机map对齐，跟PLANNER/CONTROLLER无关）=="
    ros2 launch uwb_origin_bridge uwb_origin_bridge.launch.py \
        namespace:="${NAMESPACE}" &

    # 2026-09-15用户要求：容器启动就自动触发一次`set_origin_from_uwb`，
    # 不用每次都手动`ros2 service call`——`uwb_ground_truth_node.py`
    # 文件头注释早就写明"起飞前必须先给两机各自触发一次
    # set_origin_from_uwb，不然不会有world->{ns}/odom这条TF"，但这一步
    # 之前一直没有任何地方自动做，容器/测试脚本重跑几次之后没人记得
    # 手动点一次，选手的`sdk.world_to_local()`会一直卡等这条永远不会
    # 出现的TF直到超时（真实表现：起飞正常、绕飞/goto_world第一步就
    # 报`world_to_local`超时，跟绕飞逻辑、跟障碍物、跟识别都无关）。
    # 参考sim-world-entrypoint.sh"自动触发一次reset_scenario"同一个
    # 写法：放后台子shell、sleep一段时间再调用，不阻塞后面飞控/规划栈
    # 的启动。这里锁定的是"静止锁定"（`_handle_set_origin`按锁定那一刻
    # 的位置均值当SE(2)锚点），必须在飞机还没起飞、原地不动的时候触发
    # ——不猜"等几秒应该够了"：`origin_setter_node`内部本来就有明确的
    # 就绪判断（UWB样本数在`sample_window_sec`窗口内是否达到
    # `min_samples`，不够会在响应里给出"UWB样本不足(x/5)"这种明确失败
    # 原因，不是含糊超时），这个判断本身就是最权威的"到底好没好"信号，
    # 不需要在entrypoint这一层另外猜一个延迟/重试次数上限。改成不设
    # 上限、每3秒重试一次，直到响应里出现`success: true`再退出循环——
    # sim-world和flight-stack是两个独立容器，各自启动耗时本来就会跨run
    # 波动（PX4 SITL/DLIO/px4ctrl这些都要起来才有稳定odom），"该试几次"
    # 交给服务自己的就绪判断决定，比在这里猜一个次数上限更准确。
    (
        _attempt=0
        while true; do
            _attempt=$((_attempt + 1))
            sleep 3
            _result="$(ros2 service call "/${NAMESPACE}/set_origin_from_uwb" std_srvs/srv/Trigger "{}" 2>&1)"
            echo "== [flight-stack:${NAMESPACE}] set_origin_from_uwb第${_attempt}次尝试 =="
            echo "${_result}"
            # 2026-09-15实测踩坑：`ros2 service call`对`std_srvs/srv/
            # Trigger`打印的实际是Python repr格式`success=True`（等号+
            # 首字母大写，不是YAML格式`success: true`）——写成后者匹配
            # 不上，哪怕服务早就真的返回成功了也会误判成失败、无限重试
            # 下去，实测复现过锁定成功后还傻等了几十次。
            if echo "${_result}" | grep -q "success=True"; then
                echo "== [flight-stack:${NAMESPACE}] set_origin_from_uwb锁定成功（第${_attempt}次尝试） =="
                break
            fi
        done
    ) &
else
    echo "== [flight-stack:${NAMESPACE}] LOCALIZATION_SOURCE=single_slam_only：不启动 uwb_origin_bridge（纯SLAM单机模式，不依赖任何UWB硬件）=="
fi

# 2026大赛任务系统阶段2.3：仿真侧感知栈只在DEPLOY_TARGET!=hw时启动——真机
# 走独立的vision-stack容器（阶段11.1：接口本来就对齐，不需要这个节点）。
# cameras默认值要跟sim-world-entrypoint.sh里`gen_iris_mid360_sdf.py
# --cameras`那处保持一致——2026-09-08用户明确要求不分角色，所有机（NX01/
# NX02/以后任何NX0N）都统一前视+下视两路，两处是各自独立维护的相同默认值，
# 以后如果改分工要记得两处一起改。跟CAMERAS_${NS}（sim-world那边用来覆盖
# 单机相机挂载）是两个不同的环境变量，这里的CAMERAS不带NS后缀，因为
# flight-stack容器本来就是单机的，不需要在一个进程里区分多机。
if [ "${DEPLOY_TARGET}" != "hw" ]; then
    # 2026-09-18：默认值改成"switchable"——单相机+可动关节两档预设视角
    # （见sim-world那边gen_iris_mid360_sdf.py::merge_switchable_camera()
    # 的说明），跟这架飞机SDF里唯一那个相机的`camera_name`
    # （`{ns}_switchable_camera`）对应，`qr_apriltag_detect_node.py`按
    # `{ns}_{cam}_camera/image_raw`这套既有命名规则去订阅，cam='switchable'
    # 不需要改这个节点一行代码。两架飞机现在都重新挂真实相机了（之前
    # "NX02不挂相机"是单相机方案出来之前的过渡方案，见DEBUG_JOURNAL.md
    # 2026-09-17/09-18记录）。
    CAMERAS_DEFAULT="switchable"
    # `:-`在变量被显式设成空字符串时也会触发默认值替换，改用单杠`-`
    # （只在真的没设置这个环境变量时才用默认值），需要临时完全不挂相机
    # 时仍可传CAMERAS=""覆盖（走下面的空值分支）。
    CAMERAS="${CAMERAS-${CAMERAS_DEFAULT}}"
    if [ -z "${CAMERAS}" ]; then
        # 2026-09-17：NX02改成完全不挂相机（见docker-compose.yml里
        # CAMERAS_NX02的完整说明——真实相机的Gazebo Classic共享渲染队列
        # 问题、换成logical_camera之后更深的跨容器DDS问题，两条路都踩过坑，
        # 最终拍板简化成"NX02不带相机，检测需求全部由NX01覆盖"）——这里
        # 直接不启动任何检测/桥接节点，没有相机就没有vision/detections
        # 这个话题的发布者，下游precision_servo_node/fire_pillar_aim_node
        # 保持待命（不影响启动，只是收不到检测结果，这两个节点对"没有
        # 检测消息"本来就有超时/待命的防御逻辑）。
        echo "== [flight-stack:${NAMESPACE}] CAMERAS为空，不挂相机，跳过启动任何视觉检测节点 =="
    else
        # 2026-09-18：logical_camera这条路（纯几何视锥判断，不做图像渲染，
        # 曾经打算用来绕开Gazebo Classic相机渲染的各种问题）排查了一整天，
        # 确认插件在gzserver进程内publish()之后消息完全送不到任何订阅者，
        # 根因未定位到（跨容器/单容器/单机/多机、1个或2个传感器都复现，
        # "挪到主线程发布"这个修复也验证过无效），用户决定放弃这条路，
        # 相关代码（SDF模板/插件patch/桥接节点）已删除，见DEBUG_JOURNAL.md
        # 2026-09-17/09-18记录。现在只有一条检测节点路径。
        # 2026-09-09新增：MJPEG拉流端口——跟真机vision-stack同一套机制
        # （mjpeg_server.py原样搬过来，见该文件/qr_apriltag_detect_node.py
        # 的说明），每机前视/下视各占一个端口。三个容器都是network_mode:
        # host、共享宿主机端口空间，NX01/NX02不能用同一个端口，按AGENT_INDEX
        # （1-based）错开：NX01=8180起、NX02=8190起，每机预留10个端口的
        # 间隔（当前只用2个，front/down各一个，留余量给以后可能加的相机）。
        # 8180起是刻意避开GCS自己占用的8080，不是随便挑的数字。
        MJPEG_PORT_BASE=$((8180 + (AGENT_INDEX - 1) * 10))
        echo "== [flight-stack:${NAMESPACE}] 启动 qr_apriltag_detect_node（仿真侧感知栈，cameras=${CAMERAS}，MJPEG端口起始=${MJPEG_PORT_BASE}） =="
        ros2 run contest_mission qr_apriltag_detect_node \
            --ros-args -r __ns:="/${NAMESPACE}" \
            -p cameras:="[${CAMERAS}]" \
            -p mjpeg_port_base:="${MJPEG_PORT_BASE}" &
    fi

    # 2026大赛任务系统阶段6.2：精降视觉伺服，同样只在仿真侧启动（依赖
    # 阶段2的vision/detections + dlio/odom_node/odom，真机侧走独立的
    # apriltag_ros流水线，阶段11再考虑要不要移植这套简化实现）。默认只
    # 是发布到precision_land_cmd，阶段3的position_cmd_relay不切到
    # precision_land模式的话这些消息没有任何下游消费者，不会误控制。
    echo "== [flight-stack:${NAMESPACE}] 启动 precision_servo_node（阶段6.2精降视觉伺服） =="
    ros2 run contest_mission precision_servo_node \
        --ros-args -r __ns:="/${NAMESPACE}" &

    # 2026大赛任务系统阶段6.1：执行器（符号化动作）节点，纯状态记录，
    # 不依赖odom/vision/position_cmd任何一路话题，理论上sim/hw都能跑，
    # 但暂时归在同一个DEPLOY_TARGET!=hw分支里，等阶段11正式做真机移植
    # 时再决定要不要放开——不提前在这一步就把hw模式的行为改了。
    echo "== [flight-stack:${NAMESPACE}] 启动 actuator_action_node（阶段6.1执行器符号化动作） =="
    ros2 run contest_mission actuator_action_node \
        --ros-args -r __ns:="/${NAMESPACE}" &
    # 2026-09-20：起飞完成判定下沉到机载（Takeoff action server）。对下
    # 仍然发takeoff_land话题，所以pt4ctrl/px4ctrl/so3ctrl三个控制器通用，
    # 不需要各实现一份；CONTROLLER=ros2_px4_stack没有这条话题，那条路线
    # 下这个节点不起作用（既有约束，不是这次引入的）。SDK侧探测不到这个
    # server时会自动退回原来的选手侧判定，所以镜像可以分开重建。
    echo "== [flight-stack:${NAMESPACE}] 启动 takeoff_monitor_node（起飞完成判定，机载） =="
    ros2 run contest_mission takeoff_monitor_node \
        --ros-args -r __ns:="/${NAMESPACE}" &
fi

wait
