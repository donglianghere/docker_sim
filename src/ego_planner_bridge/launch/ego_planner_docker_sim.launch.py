"""docker_sim集成专用launch文件：ego_planner_node + traj_server 两个节点，
挂在同一个namespace(NX01/NX02)下，前端接DLIO/Gazebo，后端发px4ctrl_ros2
吃的相对话题名position_cmd（配合patches/ego_planner_traj_server_relative_
poscmd.patch）。

只用ego-planner-swarm的规划器核心（ego_planner_node/traj_server来自
staging/ego-planner-swarm的src/planner，不带src/uav_simulator那批跟
mighty重名的辅助包），不启动它自己的深度相机仿真/drone_detect视觉互检测
——这套docker_sim用真实mid-360激光雷达点云，不用深度相机路径（plan_env/
grid_map.cpp的grid_map/cloud+grid_map/odom这条独立点云输入路径，跟深度
相机路径互不依赖，实测确认不喂grid_map/depth不会触发任何"odom or depth
lost"报错）。

跟mighty共存：PLANNER环境变量决定起mighty还是这套launch文件，两者从不
同时跑，不会同时抢占px4ctrl的cmd话题或同一份点云/odom订阅资源。

参数大部分照抄自ego-planner-swarm自己的
src/planner/plan_manage/launch/advanced_param.launch.py（地图尺寸/
感知范围/动力学限制/优化权重全部原样保留，未经实测调参，上线前应结合
mid-360实际点云质量和飞行空间重新验证），只改了以下几处集成相关的点：
  - 去掉了原launch文件"drone_<id>_"字符串前缀的多机命名方式（那是给
    单进程内跑多个drone_id用的），改用ROS2 namespace=${NAMESPACE}
    （跟mighty/DLIO/px4ctrl同一套约定），drone_id参数仍然保留、仍然
    必须两机不同——它是calcSwarmCost用来在共享的/broadcast_bspline
    全局话题上区分"自己"和"其他飞机"轨迹的ID，不是话题命名空间。
  - grid_map/odom、odom_world remap到dlio/odom_node/odom里程计
    （LOCALIZATION_SOURCE=gt时gt_odom_bridge也发到这同一个相对话题名，
    不用分支处理）。

    ⚠️ grid_map/cloud **不能**直接接mid360_PointCloud2原始点云——
    2026-08-07实测双机复现：PLANNER=ego_planner+CONTROLLER=px4ctrl，
    给NX01发目标点(-3,0,1)，飞机径直撞上房间中心的柱子，规划器完全
    没有绕障的趋势。根因是plan_env/grid_map.cpp的cloudCallback()不做
    任何TF变换，直接假设收到的点云已经跟odom处于同一个坐标系——但
    mid360_PointCloud2是Gazebo雷达插件发布的原始点云，header.frame_id
    是雷达自身随飞机姿态转动的传感器帧（"{ns}/{ns}_livox"），跟
    dlio/odom_node/odom所在的、静止不动的odom_frame完全不是一回事，
    直接喂给grid_map会把"雷达此刻朝向"误当成"障碍物在世界里的位置"，
    构建出来的占据栅格是错的，规划器等于在盲飞。mighty不会踩这个坑是
    因为它从不直接消费原始点云，是靠global_mapper_ros做过TF变换之后
    的occupancy_grid/unknown_grid。
    改成接DLIO自己发布的dlio/odom_node/pointcloud/deskewed——DLIO的
    odom.cc里publishCloud()对这份点云做了
    `pcl::transformPointCloud(...)`并把header.frame_id设成跟odom消息
    同一个this->odom_frame，是已经变换到odom坐标系、跟里程计天然一致
    的点云，grid_map.cpp那套"点云和odom同坐标系"的假设在这份数据上
    才成立。
    ⚠️ 2026-08-07第一次修复时把话题名错写成`dlio/odom_node/deskewed`
    （少了中间的`pointcloud/`），实测确认这个话题从来不存在——
    `create_publisher<...>("deskewed", 1)`里的"deskewed"是相对当前
    node的话题名，但DLIO的odom_node实际把它建在"pointcloud"这个子
    命名空间下（`ros2 node info /NX01/ego_planner_node`能看到
    Subscribers列着这个订不到任何东西的死话题），等于第一次"修复"完全
    没生效，grid_map一直是空的——真机测试直接撞墙复现，`ros2 topic hz
    /NX01/dlio/odom_node/pointcloud/deskewed`实测确认真实话题在
    ~11.4Hz发布、frame_id跟odom一致（NX01/odom），这次改成这个真实
    路径。
    LOCALIZATION_SOURCE=gt时DLIO根本不跑，这个话题原本不存在——现在
    flight-stack-entrypoint.sh在PLANNER=ego_planner+LOCALIZATION_SOURCE=gt
    时会额外起gt_odom_bridge包的gt_cloud_bridge_node，订阅原始
    mid360_PointCloud2、沿TF变换到odom系后以同一个话题名重新发布，
    仿照的正是global_mapper_ros的做法，这里的remap因此不用区分
    dlio/gt两种模式，两边都能拿到这同一个话题名。
  - grid_map/use_depth_filter改成False（明确表示不用深度相机路径，
    纯粹是文档意义上的清晰，不影响实际行为）。
  - grid_map/frame_id改成"${NAMESPACE}/map"，跟项目里"${NAMESPACE}/map"
    的既有frame命名习惯对齐。
  - fsm/flight_type用MANUAL_TARGET(=1)而不是原文件默认的PRESET_TARGET
    (=2)：PRESET_TARGET要求提前在参数里写死目标点列表，MANUAL_TARGET
    则是运行时订阅一个PoseStamped话题作为目标点，更适合手动测试。
    原本硬编码的绝对话题名"/move_base_simple/goal"remap成"term_goal"
    ——跟mighty用的命名空间化的"/${NAMESPACE}/term_goal"统一成同一个
    约定（namespace=${NAMESPACE}下相对名"term_goal"解析出来正好是这个
    绝对路径），scripts/dual_goal_input.py自带"世界坐标->每机局部坐标"
    换算（按AGENT_INDEX*3的INIT_X偏移）再发到这个话题，对ego_planner
    同样适用（它的odom也是DLIO在each机自己spawn点为原点的局部坐标系，
    跟mighty同一套假设）。2026-08-07撞柱子事故里，目标点很可能就是通过
    某个没有world->local换算的临时路径直接送进了这个话题，(-3,0,1)当作
    局部坐标解读正好落在NX01本地地图原点附近、也就是世界坐标的房间
    中心。

    ⚠️ 当时以为"统一到term_goal之后就不会再出现坐标解读不一致"——这个
    判断只对`dual_goal_input.py`成立，漏掉了RViz自带的"2D Goal Pose"
    工具：`ego_replan_fsm.cpp::waypointCallback()`直接读
    `msg->pose.position.x/y`，完全不看`header.frame_id`、不做任何TF
    变换，会把RViz点击时那个"当前Fixed Frame下的原始坐标"直接当成局部
    坐标——如果`multi_ego_planner.rviz`里的"2D Goal Pose"工具（跟
    `dual_goal_input.py`一样）直接把Topic连到`term_goal`，点出来的坐标
    就完全绕开了换算，跟2026-08-07那次事故是同一类坑，2026-08-08
    复现过一次。改成：RViz的"2D Goal Pose"工具连到`rviz_goal_world`
    （相对名，同一namespace下解析成`/${NAMESPACE}/rviz_goal_world`），
    新增的`rviz_goal_bridge_node`（见下面）订阅这个话题、转发到
    `term_goal`，`dual_goal_input.py`不受影响（它一直是自己算好局部
    坐标直接发`term_goal`，没有经过RViz这层，逻辑不变）。

    ⚠️ 第一版`rviz_goal_bridge_node`的换算方式本身也是错的：假设
    `rviz_goal_world`发布的是"世界坐标"、减去这架飞机的`init_x`
    （`AGENT_INDEX*3`）就能得到局部坐标——2026-08-08用`grid_map/
    obstacles_inflation`调大之后的一次实测发现"目标点被判定在障碍物里，
    但RViz里肉眼看完全不在"，交叉核对发现`multi_ego_planner.rviz`的
    Fixed Frame设的是`NX02/odom`（某一架具体飞机自己的frame，不是
    世界系！），RViz的2D Goal Pose工具发布的坐标永远是"点击位置在当前
    Fixed Frame下的数值"——对NX02自己来说，原始点击本来就已经是
    NX02/odom系的数值、跟`odom_pos_`是同一个系，根本不需要再减
    `init_x`；对NX01来说更糟，该加offset差值却在减，符号都反了。飞机
    因此飞向了偏离用户实际点击好几米的错误坐标，被灌进不该去的占据
    格子里——不是占据栅格本身错了，是目标点坐标先送错了地方。
  - 新增`rviz_goal_bridge_node`（本包`ego_planner_bridge`自己的节点，
    见`ego_planner_bridge/rviz_goal_bridge_node.py`）：订阅`rviz_goal_
    world`，用tf2把消息自带的`header.frame_id`（也就是RViz发布时刻
    实际的Fixed Frame，不硬编码假设是哪个）变换到这架飞机自己的
    `<namespace>/odom`，再发布到`term_goal`——不管Fixed Frame设成
    哪一架飞机的frame，只要TF树里查得到变换就能算对，不用再关心
    "到底该不该减init_x、减多少"这类容易搞反符号的手动换算。
  - 没有起map_generator/mockamap（那是ego-planner自带demo用来生成
    虚拟障碍物地图的节点，这里用真实Gazebo世界，不需要）。
  - grid_map/obstacles_inflation从原demo默认的0.099改成0.35。
    2026-08-07点云话题修好之后（occupancy_inflate确认有数据）实测
    还是基本不避障，交叉核对发现：simple_room世界里的柱子是半径0.25m
    的圆柱（见patches/mighty_simple_room_world.patch的
    `<cylinder><radius>0.25</radius>`），飞机（iris+mid360）的碰撞箱
    是0.47x0.47x0.11米（半宽0.235米，见flight-stack-entrypoint.sh的
    INIT_Z注释）——grid_map.cpp把飞机当成一个点来规划，obstacles_inflation
    是唯一一个负责把"点规划"变回"考虑飞机自身体积"的参数，0.099米
    完全没把飞机半宽0.235米算进去：飞机中心刚好贴着膨胀后的占据边界
    飞（半径0.25+0.099=0.349米）时，机身还会伸进真实柱子表面
    0.25-(0.349-0.235)=0.136米——也就是说就算规划器完全遵守膨胀边界，
    飞机本体依然会撞进柱子，跟"避障参数太小、没考虑飞机尺寸"这个猜测
    完全对得上。改成0.35（0.235半宽+0.115余量），
    optimization/dist0(=0.5)是在这层膨胀之上另加的软惯性代价缓冲，
    数值本身不用动。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    drone_id = LaunchConfiguration('drone_id')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('NAMESPACE', default_value='NX01'),
        description='NX01/NX02——必须跟MAVROS/mighty/DLIO/px4ctrl用同一个值',
    )
    declare_drone_id = DeclareLaunchArgument(
        'drone_id',
        default_value=EnvironmentVariable('DRONE_ID', default_value='0'),
        description=(
            '给calcSwarmCost用来在共享的/broadcast_bspline全局话题上区分'
            '"自己"和"其他飞机"的轨迹，两架飞机必须不同（NX01=0, NX02=1），'
            '跟namespace是两件独立的事，不能只设namespace不设这个'
        ),
    )
    # docker_sim 2026-08-09：照mighty的/frame_align模式给ego_planner补的坐标
    # 对齐开关——calcSwarmCost/checkCollision原来直接比较两机各自DLIO局部
    # odom坐标，没有考虑两机spawn点本身就有偏移，导致"机间距离"系统性偏离
    # 真值（具体分析和数据见README"ego_planner的机间避障直接比较两架飞机
    # 各自局部里程计坐标"那次记录）。默认跟着这套仿真真实用的hw_mighty.yaml
    # 一致设成true（mighty那边这个配置本来就是给本项目"每机独立local odom"
    # 这个架构准备的），需要临时关掉（比如odom改成共享全局系的场景）时设
    # EGO_USE_FRAME_ALIGNMENT=false，不用改代码。
    use_frame_alignment = LaunchConfiguration(
        'use_frame_alignment', default=EnvironmentVariable('EGO_USE_FRAME_ALIGNMENT', default_value='true'))
    declare_use_frame_alignment = DeclareLaunchArgument(
        'use_frame_alignment', default_value=use_frame_alignment,
        description='是否在收到/broadcast_bspline广播轨迹时用/frame_align做跨机坐标变换',
    )
    # 复用docker-compose.yml里已有的NUM_AGENTS（当前项目固定2），决定要订阅
    # 几个/frame_align/<own_ns>/<other_ns>话题——跟drone_id一样两机必须一致。
    num_agents = LaunchConfiguration(
        'num_agents', default=EnvironmentVariable('NUM_AGENTS', default_value='2'))
    declare_num_agents = DeclareLaunchArgument(
        'num_agents', default_value=num_agents,
        description='集群总飞机数，用来推导需要订阅哪些/frame_align/<own_ns>/<other_ns>话题',
    )

    # "${NAMESPACE}/map"字符串拼接——parameters=[]的字典值不支持像
    # name=[...]那样直接传list做拼接（会被当成数组类型参数），用
    # PythonExpression拼出普通字符串。
    map_frame_id = PythonExpression(["'", namespace, "' + '/map'"])

    # 2026-09-14用户直接指定：缓冲区60x60x5米、局部窗口40x40米（对应
    # 下面local_update_range_xy=半宽20.0）——排查D3实测"飞机径直撞上
    # obstacle_cylinder"时发现原来的42x30米缓冲区+15米局部窗口（半宽）
    # 跟房间实际尺度（20x25米）、20米长的单段目标比起来偏紧，见
    # DEBUG_JOURNAL.md 2026-09-14相关记录。
    map_size_x = LaunchConfiguration('map_size_x_', default=60.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=60.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    # docker_sim: 原来max_vel/max_acc是这个demo launch文件自己的硬编码默认值
    # (2.0/3.0)，从来没跟docker-compose.yml/flight-stack-entrypoint.sh里
    # V_MAX/A_MAX这两个环境变量联动过——mighty那条路径一直在用这两个环境
    # 变量（默认V_MAX=1.0），是这套仿真+px4ctrl组合实测跑得动的速度，
    # ego_planner这边max_vel默认2.0是mighty调好的速度的2倍，跟px4ctrl配合
    # 表现明显更差（2026-08-08实测：轨迹可视化完美绕开障碍物，飞机本体
    # 还是贴着侧面撞上）——同一个px4ctrl、同一套增益，mighty用1.0m/s没事，
    # 换成2.0m/s跟踪误差明显变大，是ego_planner比mighty更容易撞的一个具体、
    # 可复现的候选原因。改成默认读V_MAX/A_MAX（不设置这两个环境变量时的
    # 默认值1.0/3.0参考docker-compose.yml），复用mighty已经验证过的速度，
    # 而不是upstream demo从未针对这套仿真调过的数字。
    max_vel = LaunchConfiguration('max_vel', default=EnvironmentVariable('V_MAX', default_value='1.0'))
    max_acc = LaunchConfiguration('max_acc', default=EnvironmentVariable('A_MAX', default_value='3.0'))
    # docker_sim: traj_server.cpp原来把最大偏航角速度写死成constexpr PI
    # (~180°/s)——上一轮误以为mighty那边对应的调好的值是
    # OMEGA_MAX=0.10472rad/s(6°/s)，改完之后用户指出这个数字不对：
    # `omega_max`（hw_mighty.yaml里那个"Maximum angular velocity"）读代码
    # 确认是lbfgs_solver.cpp轨迹优化器内部的**机体角速度**可行性约束
    # （roll/pitch/yaw合起来的动力学约束，用在优化目标函数里），根本不是
    # "最终发给px4ctrl的yaw朝向变化速率"这个概念；真正对应traj_server.cpp
    # 这个yaw_dot_max的是mighty.cpp里`next_goal.dyaw = std::clamp(
    # next_goal.dyaw, -par_.w_max, par_.w_max)`用的`w_max`参数——
    # `patches/mighty_avoidance_tuning.patch`把它从1.0调到了
    # **0.8rad/s(约45.8°/s)**，这个数字也和PX4固件本身的
    # `MPC_YAWRAUTO_MAX`默认值45°/s（见`patches/px4_iris_mpc_yawrate_max.
    # patch`的完整历史注释：曾经试过跟着mighty把这个PX4参数从45°/s提到
    # 180°/s，结果两架飞机反复炸机——一个还没查清根因的yaw持续偏差bug，
    # 45°/s时只会造成缓慢的朝向漂移，180°/s时会让PX4以更快的速率追这个
    # 发散的yaw误差，推力打满、彻底失控，最后把这个PX4参数改回了stock的
    # 45°/s）几乎精确对上——PX4固件层面本来就有这个45°/s的硬顶，w_max=0.8
    # 基本就是贴着这个硬顶调的。改成读新的专用环境变量`EGO_YAW_DOT_MAX`
    # （不再错误地复用`OMEGA_MAX`），默认值0.8，跟mighty这边真正的yaw限速
    # 参数保持一致。
    yaw_dot_max = LaunchConfiguration('yaw_dot_max', default=EnvironmentVariable('EGO_YAW_DOT_MAX', default_value='0.8'))
    # docker_sim: 下面这5个原来都是硬编码数字，2026-08-08用户实测反馈之后
    # 一起改成读环境变量（docker-compose.yml里配，改完不用重新build镜像，
    # 跟mighty那边V_MAX/A_MAX/...的既有约定一致）：
    #   - planning_horizon 7.5->15：单次规划看得太近，遇到障碍物之前留给
    #     规划器"发现-绕开"的反应距离不够。
    #   - obstacles_inflation 0.35->0.6：实测"规划NX01越过正中间柱子时，
    #     轨迹仅有极小圆弧紧贴环绕柱子"——0.35（0.235半宽+0.115余量）理论
    #     上刚好够、但完全没有给跟踪误差留余量（见README"轨迹完美绕障、
    #     飞机侧面撞上"那次排查），0.6留了接近双倍的余量。
    #   - thresh_replan_time 1.0->0.1：跟mighty对照发现的差异——mighty
    #     的timer_replanning_每10ms就检查一次要不要重规划（mighty_node.cpp
    #     `create_wall_timer(10ms, ...)`），ego_planner这个1.0秒的阈值
    #     等于人为把重规划频率下限锁定在1Hz，节流太狠。这里没有直接照抄
    #     10ms——B样条轨迹优化本身的计算耗时大概率超不过10ms这个数量级，
    #     阈值设得比实际计算耗时还小没有意义，0.1秒是"明显比原来1秒响应快
    #     一个数量级、但没有细到可能引发计算跟不上/来回抖动"的一个折中，
    #     实测下来如果CPU余量够、可以继续往下调。
    #   - lambda_collision 0.5->1.0、lambda_feasibility 0.1->0.3：优化目标
    #     函数里避障代价、动力学可行性代价相对lambda_smooth(=1.0，平滑度)
    #     权重偏低，容易在障碍物附近为了"更平滑"牺牲避障裕度/可行性——
    #     两个都调高，让优化器更愿意为了避障和动力学可行性牺牲一些平滑度。
    planning_horizon = LaunchConfiguration('planning_horizon', default=EnvironmentVariable('EGO_PLANNING_HORIZON', default_value='15.0'))
    obstacles_inflation = LaunchConfiguration('obstacles_inflation', default=EnvironmentVariable('EGO_OBSTACLES_INFLATION', default_value='0.6'))
    # 竖直膨胀半径（配合 ego_planner_grid_map_inflation_z.patch）——点云路径
    # (cloudCallback) 原来把竖直膨胀写死成1格(±0.1m)，水平却是0.6m，差6倍。
    # Mid360逐帧的竖直覆盖并不均匀，稍远处的障碍在rviz里明显呈"层状"，层与层
    # 之间正好可能落在飞行高度上；再加上grid_map每帧resetBuffer推倒重建、这条
    # 路径没有任何时间累积，一帧没打到就等于那一帧地图上没有障碍。2026-09-26
    # 实测：飞行中4.2~5.8%的帧里3#立柱在"跟飞机同高±0.1m"内一个点都没有，单次
    # 最长0.46秒。默认0.1保持原行为；调大能把层间空隙补上，代价是每个输入点
    # 要写的格子数从(2*6+1)^2*3=507线性涨（±0.3m -> 1183，±0.6m -> 2197），而
    # 逐点膨胀本来就已经吃满一个核，调之前先看CPU。
    obstacles_inflation_z = LaunchConfiguration('obstacles_inflation_z', default=EnvironmentVariable('EGO_OBSTACLES_INFLATION_Z', default_value='0.1'))
    thresh_replan_time = LaunchConfiguration('thresh_replan_time', default=EnvironmentVariable('EGO_REPLAN_THRESH', default_value='0.1'))
    lambda_collision = LaunchConfiguration('lambda_collision', default=EnvironmentVariable('EGO_LAMBDA_COLLISION', default_value='1.0'))
    lambda_feasibility = LaunchConfiguration('lambda_feasibility', default=EnvironmentVariable('EGO_LAMBDA_FEASIBILITY', default_value='0.3'))
    # docker_sim 2026-08-11：min_ray_length本来是给的，但ego-planner-swarm自己
    # 的grid_map.cpp里真正用它的那行判断被注释掉了（ego_grid_map_min_ray_
    # length.patch取消注释），改之前这个参数形同虚设——mid360实测点云里有
    # ~12%的点落在传感器0.3米内、最近只有4厘米，是LiDAR扫到飞机自己机身/
    # 桨叶的自扫描伪点，被当障碍物写进栅格，飞机自己所在的位置就被判定为
    # "起点在障碍物里"，A*永远搜不到路径。默认值取"机身中心到桨叶尖端"的
    # 物理半径：iris.sdf.jinja里rotor_0的pose是(0.13, -0.22)（最大那组），
    # 臂长=sqrt(0.13^2+0.22^2)≈0.256m，桨叶碰撞半径0.128m
    # （rotor_x_collision的<radius>），加起来≈0.384m；mid360安装在(0,0,0.06)
    # 只有Z向偏移、无XY偏移（gen_iris_mid360_sdf.py），对这个水平半径影响
    # 可忽略。取0.4m，比精确值略留一点余量。
    min_ray_length = LaunchConfiguration('min_ray_length', default=EnvironmentVariable('EGO_MIN_RAY_LENGTH', default_value='0.4'))
    # docker_sim: 2026-08-08用户实测反馈"目标7-8米远，却只规划出一米多的
    # 轨迹""终点跟点击的目标点差三四米"——查`ros2 param get`确认活的容器里
    # `grid_map/max_ray_length`=4.5、`grid_map/local_update_range_x/y`=5.5，
    # 这两个是upstream demo自己的默认值（给短量程深度相机场景留的，从没
    # 针对mid-360的真实探测距离20米+重新核实过，文件头注释早就提过这个
    # 疑虑），从头到尾没有跟着上一轮"planning_horizon 7.5->15"这个改动
    # 一起调——占据栅格的"局部地图"范围/单帧点云的有效标记距离都远小于
    # 新的15米规划视野，B样条优化只能信任这4.5~5.5米以内的占据信息，
    # 超出这个范围形同盲飞，规划器没法、也不敢真的把轨迹伸到15米外，
    # 观测到的"只有一米多""差三四米"完全对得上"局部地图看不了那么远"
    # 这个解释。改成读专用环境变量，默认跟`planning_horizon`看齐（15米），
    # 消除这个新出现的不匹配——**代价**：局部地图缓冲区尺寸（`grid_map.
    # cpp`里跟`local_update_range_x/y/z`成正比的三维数组）会明显变大，
    # 这套仿真本身已经因为实时因子只有0.44而计算吃紧（见前面"仿真跑不到
    # 实时速度"那次排查），这个改动有可能让CPU负担更重、实时因子进一步
    # 下降，需要实测验证，不是稳赚不赔的改动。Z方向没有跟着改（保持
    # 4.5米不变，跟房间层高本身匹配，垂直方向不需要看这么远，改大只会
    # 白白增加缓冲区体积）。
    local_update_range_xy = LaunchConfiguration('local_update_range_xy', default=EnvironmentVariable('EGO_LOCAL_UPDATE_RANGE_XY', default_value='15.0'))
    max_ray_length = LaunchConfiguration('max_ray_length', default=EnvironmentVariable('EGO_MAX_RAY_LENGTH', default_value='15.0'))
    # docker_sim 2026-08-09：用户实测反馈"避障不稳定——明明有距离障碍更远
    # 的轨迹，偏要飞更近的轨迹""经常擦着障碍物柱子边飞过""两机相遇，一机从
    # 另一机头顶飞过"。读bspline_optimizer.cpp的calcDistanceCostRebound/
    # calcSwarmCost确认这不是bug、是ego-planner本身的代价函数设计：距离一旦
    # 大于安全阈值（dist0对障碍物、swarm_clearance对同伴），代价直接归零
    # （dist_err<0时"do nothing"）——这是"够安全就行"的硬截断约束，不是
    # "越远越好"的连续势场，优化器在一堆"已经安全"的候选轨迹里只会挑平滑度/
    # 能耗更优的那条，不会因为某条轨迹离障碍物更远就偏爱它。用户也指出
    # px4ctrl跟踪轨迹本身很精确，说明问题出在这两个阈值设得偏小，不是
    # 跟踪误差——把阈值调大是唯一能在不改代价函数结构的前提下起作用的
    # 手段。
    #   - dist0 0.5->1.0：障碍物安全距离阈值，配合已经调到0.6的
    #     obstacles_inflation（膨胀半径，两者是分开累加的两层缓冲，见
    #     该参数注释），让"贴着柱子边擦过"这种在0.5米阈值下已经判定为
    #     "安全"的轨迹不再达标，逼优化器往更远推。
    #   - swarm_clearance 0.5->1.0：同伴间隔安全阈值。calcSwarmCost用的是
    #     各向异性椭球距离（bspline_optimizer.cpp第880行硬编码
    #     `a=2.0,b=1.0`，Z方向半轴是XY方向的2倍），要达到零代价，纯Z方向
    #     分离需要`2*swarm_clearance*2=4*swarm_clearance`米、纯XY方向分离
    #     需要`2*swarm_clearance`米——0.5米时纯Z只需2.0米即被判定"安全"，
    #     调到1.0后升到4.0米，让"从头顶飞过"这种躲法需要真正拉开的高度差
    #     明显变大，更接近真实旋翼下洗气流需要的安全间隔。这个2:1的各向
    #     异性比例是C++里硬编码的常量，没有暴露成参数，如果调大
    #     swarm_clearance之后"头顶飞过"还是频繁出现，说明问题在这个比例
    #     本身太小，需要再写一个patch把`a`/`b`也提出来当参数，这次先不做。
    # 2026-09-22 虚拟天花板 2.9 -> 4.5 米（用户决定，配合 2.5 米巡航搜索地面火情）。
    # 2.9 米时，天花板这一层体素在 2.8 米（代码是 floor(...)-1），再减掉优化器
    # 安全距离 dist0，推开代价从 1.8 米就开始了——2.5 米巡航的飞机一直处在
    # 天花板的推开区里，持续受到向下的推力。空旷处这股推力跟"跟住目标高度"
    # 相互抵消，飞机稳在 2.36~2.46 米；一旦旁边有立柱，侧向推力叠加上来，
    # 合力就是往下：实测 2.5 米穿 3 号立柱时一路掉到 0.8 米、卡在立柱跟前。
    # 注意天花板这一层是在膨胀之后直接写进膨胀地图的（grid_map.cpp 的
    # "add virtual ceiling"），本身**不**膨胀，只有 0.1 米厚。
    # 4.5 米时天花板层在 4.4 米，dist0=1.5 的推开区从 2.9 米开始，2.5 米巡航
    # 留 0.4 米余量。**改巡航高度或 dist0 时必须回头核这条关系**：
    #     巡航高度 < virtual_ceil_height - 0.1 - dist0
    # 上限还受地图高度约束：grid_map.cpp 会把它钳到 ground_height+map_size_z
    # （这里是 -0.01+5.0=4.99）。
    # ⚠️ 仿真场地高 6 米；真实场馆的层高要现场确认，天花板不能高于真实屋顶。
    virtual_ceil_height = LaunchConfiguration(
        'virtual_ceil_height',
        default=EnvironmentVariable('EGO_VIRTUAL_CEIL_HEIGHT', default_value='4.5'))
    # 2026-09-22 dist0 默认 1.0 -> 1.5（用户决定），同时见上面天花板那段的高度约束。
    # 调大之后绕障碍物的弯会更大（离障碍物表面至少 膨胀+dist0 才没有代价）。
    dist0 = LaunchConfiguration('dist0', default=EnvironmentVariable('EGO_DIST0', default_value='1.5'))
    swarm_clearance = LaunchConfiguration('swarm_clearance', default=EnvironmentVariable('EGO_SWARM_CLEARANCE', default_value='1.0'))
    # docker_sim 2026-08-13：原来硬编码1.0（upstream demo默认值），改读
    # 环境变量。含义：离目标点还剩多远时"停止重规划、冻结当前这条轨迹
    # 飞完"（ego_replan_fsm.cpp的EXEC_TRAJ状态处理，见DEBUG_JOURNAL.md
    # 2026-08-13"抵达目标一米多位置时飞机明显停一下"那次排查）——飞机
    # 接近目标过程中，只要距离还大于这个值就每thresh_replan_time秒重新
    # 规划一次，每条轨迹自己"减速到静止"那段收尾永远没机会真正飞完就被
    # 下一条新轨迹替换掉；一旦进入这个半径，重规划整个停止，飞机被迫把
    # 当时那一条（唯一一条）轨迹完整飞完，观测到的"停一下再飞向终点"
    # 正是这段收尾第一次真正被执行的表现。
    # ⚠️ 2026-08-13曾经试过调小到0.2（期望重规划持续到更接近目标才停，
    # 减少"冻结轨迹"跟"实际状态"的落差），但排查另一次"the drone is in
    # obstacle"炸机时发现`planner_manager.cpp::reboundReplan()`自己写死
    # 了一个`(start_pt-local_target_pt).norm() < 0.2`的"离目标太近直接
    # 放弃"分支（第55-60行），失败后的随机初值重试路径（`callReboundReplan
    # (true,true)`）里有一处`(start_pt-local_target_pt).cross(...)
    # .normalized()`——两点距离趋近0.2米这个量级时对趋近零向量的叉积做
    # 单位化，数值上不稳定，Eigen对零向量`.normalized()`会产出NaN，NaN
    # 控制点很可能是"the drone is in obstacle. This should not happen"
    # 这次复发的根因（未100%实锤，见DEBUG_JOURNAL.md完整排查记录）。
    # 用户要求改回1.0，避开这个撞车区间——这是ego-planner-swarm上游代码
    # 本身的数值稳定性问题，不是靠调这个参数能根治的，只是把使用场景
    # 挪出这个已知有问题的距离区间，"停一下再飞向终点"这个（无害的）
    # 视觉观感问题会重新出现，两害相权取其轻。
    thresh_no_replan_meter = LaunchConfiguration(
        'thresh_no_replan_meter', default=EnvironmentVariable('EGO_NO_REPLAN_THRESH', default_value='1.0'))

    declare_tuning_args = [
        DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X'),
        DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y'),
        DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z'),
        DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity'),
        DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration'),
        DeclareLaunchArgument('yaw_dot_max', default_value=yaw_dot_max, description='Maximum yaw angular rate (rad/s)'),
        DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon'),
        DeclareLaunchArgument('obstacles_inflation', default_value=obstacles_inflation, description='Obstacle inflation radius (m)'),
        DeclareLaunchArgument('obstacles_inflation_z', default_value=obstacles_inflation_z, description='Obstacle inflation radius along Z (m), cloud path only'),
        DeclareLaunchArgument('thresh_replan_time', default_value=thresh_replan_time, description='Minimum time before considering a replan (s)'),
        DeclareLaunchArgument('lambda_collision', default_value=lambda_collision, description='Collision-avoidance cost weight'),
        DeclareLaunchArgument('lambda_feasibility', default_value=lambda_feasibility, description='Dynamic-feasibility cost weight'),
        DeclareLaunchArgument('local_update_range_xy', default_value=local_update_range_xy, description='Local occupancy map half-extent along X/Y (m)'),
        DeclareLaunchArgument('max_ray_length', default_value=max_ray_length, description='Max raycasting distance for occupancy updates (m)'),
        DeclareLaunchArgument('min_ray_length', default_value=min_ray_length, description='Min raycasting distance for occupancy updates (m) — drops self-hit points closer than the vehicle body/prop radius'),
        DeclareLaunchArgument('dist0', default_value=dist0, description='Obstacle safety-distance threshold (m)'),
        DeclareLaunchArgument('swarm_clearance', default_value=swarm_clearance, description='Inter-agent safety-distance threshold (m)'),
        DeclareLaunchArgument('thresh_no_replan_meter', default_value=thresh_no_replan_meter, description='Stop replanning and coast out the last trajectory within this distance of the goal (m)'),
    ]

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        namespace=namespace,
        output='screen',
        remappings=[
            ('odom_world', 'dlio/odom_node/odom'),
            ('grid_map/odom', 'dlio/odom_node/odom'),
            # 不能直接接mid360_PointCloud2原始点云（雷达自身转动帧，跟odom
            # 不是同一坐标系，grid_map.cpp不做TF变换）——用DLIO已经变换到
            # odom_frame的deskewed点云。只在LOCALIZATION_SOURCE=uwb_slam时存在，
            # =gt时DLIO不跑，见文件头部说明。
            ('grid_map/cloud', 'dlio/odom_node/pointcloud/deskewed'),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            # 跟mighty统一用term_goal（namespace=${NAMESPACE}下解析成
            # /${NAMESPACE}/term_goal，跟RViz现成的"2D Goal Pose"工具、
            # scripts/dual_goal_input.py发的是同一个话题，不用改任何现有
            # 工具）。
            ('/move_base_simple/goal', 'term_goal'),
        ],
        parameters=[
            {'fsm/flight_type': 1},  # MANUAL_TARGET，运行时订阅term_goal
            {'fsm/thresh_replan_time': thresh_replan_time},
            {'fsm/thresh_no_replan_meter': thresh_no_replan_meter},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},

            # PRESET_TARGET模式才用的waypoint列表，MANUAL_TARGET模式下FSM
            # 不读这几个参数，但waypoint_num必须声明（哪怕是0）否则
            # ego_replan_fsm.cpp里对应的declare_parameter会用默认值-1
            # 触发"Wrong waypoint_num_"报错分支（该分支只在PRESET_TARGET
            # 模式下才会被走到，MANUAL_TARGET模式下这个检查不会执行，
            # 这里仍然给出去0是为了参数声明完整、避免歧义）。
            {'fsm/waypoint_num': 0},

            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': local_update_range_xy},
            {'grid_map/local_update_range_y': local_update_range_xy},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': obstacles_inflation},  # 飞机半宽0.235m+跟踪误差余量，见文件头说明
            {'grid_map/obstacles_inflation_z': obstacles_inflation_z},  # 竖直膨胀，默认0.1=原行为，见上面声明处的说明
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            # 地面回波过滤（配合ego_planner_grid_map_ground_filter.patch）——
            # mid-360前倾30度装在飞机顶部，飞行高度不高时会打到大片地板，
            # cloudCallback原来对每个点一视同仁地标记+膨胀，地面因此变成
            # 一整片"占据"区域（2026-08-08真机反馈"地面膨胀的点云还是一大片"、
            # "撞墙是必然"）。0.15米——房间地板在世界z=0，飞机碰撞箱最低点在
            # 机体原点下方0.055米（见flight-stack-entrypoint.sh的INIT_Z注释），
            # 0.15米比这个还留了将近10厘米余量，房间里的障碍物（柱子/墙）都是
            # 顶天立地的全高结构，没有低于15厘米的真实障碍物需要保留，这个
            # 阈值只会丢地面噪声，不会丢真实障碍物。
            {'grid_map/ground_filter_height': 0.15},
            # 深度相机路径完全不用（见文件头说明），这几个相机内参/深度
            # 滤波参数留着原始默认值即可，反正永远不会有数据喂进来。
            {'grid_map/cx': 321.04638671875},
            {'grid_map/cy': 243.44969177246094},
            {'grid_map/fx': 387.229248046875},
            {'grid_map/fy': 387.229248046875},
            {'grid_map/use_depth_filter': False},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            # local fusion（点云路径也会用到，跟深度相机路径共用同一套
            # occupancy grid概率更新逻辑）
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': min_ray_length},
            {'grid_map/max_ray_length': max_ray_length},

            {'grid_map/virtual_ceil_height': virtual_ceil_height},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 2},  # ODOMETRY（跟odom_sub_同步用，不是POSE_STAMPED）
            {'grid_map/frame_id': map_frame_id},

            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': True},
            {'manager/drone_id': drone_id},

            {'swarm/use_frame_alignment': use_frame_alignment},
            {'swarm/num_agents': num_agents},

            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': lambda_collision},
            {'optimization/lambda_feasibility': lambda_feasibility},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': dist0},
            {'optimization/swarm_clearance': swarm_clearance},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # Object prediction —— 原demo用于预测虚拟动态障碍物，这里没有
            # 起对应的障碍物发布节点，obj_num=0表示不预测任何动态障碍物。
            {'prediction/obj_num': 0},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0},
        ],
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        namespace=namespace,
        output='screen',
        parameters=[
            # docker_sim: 2026-08-08查NX01/attitude_thrust_debug.log实测
            # 记录，发现px4ctrl收到的target yaw在几秒内持续单向旋转
            # 数百度（不是固定偏差，是真的转圈），最终thrust打满、机身
            # 翻转——读traj_server.cpp::calculate_yaw()的dir计算：
            # `dir = traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos`，
            # `time_forward_`如果是负数，算出来的是"当前位置指向
            # time_forward_秒*之前*那个轨迹点"的向量，等于让机头指向
            # **来时的方向**而不是前进方向；ego_replan_fsm.cpp里
            # `fsm/thresh_replan_time`/`fsm/thresh_no_replan_meter`这些
            # 参数在traj_server.cpp/ego_replan_fsm.cpp里的声明默认值也是
            # -1.0，这套代码里-1.0是"占位符，调用方必须覆盖成真实值"的
            # 通用约定——原来这里跟demo一样直接抄了-1.0这个占位符默认值，
            # 从没真正覆盖成有意义的正数，是这次埋头查yaw乱转问题时才
            # 发现的遗漏。改成+1.0（往前看1秒的轨迹点算朝向，符号翻正，
            # 数值本身沿用原来demo这个占位符的量级）。对照mighty的yaw
            # 计算方式（mighty.cpp直接用当前位置指向存住的全局终点算
            # desired_yaw，不依赖"沿自己轨迹往前/往后看"这种自指式计算，
            # 更稳健，不会有正负号搞反这类风险）——这次没有照抄mighty的
            # 设计思路重写，只是先修正这个明确的符号错误，用回
            # ego_planner自己原有的"沿轨迹看方向"机制。
            {'traj_server/time_forward': 1.0},
            {'traj_server/yaw_dot_max': yaw_dot_max},
        ],
        # planning/bspline（ego_planner_node发，traj_server收）和
        # position_cmd（打过ego_planner_traj_server_relative_poscmd.patch
        # 后是相对名）都不需要remap，两个节点在同一个namespace下天然对上。
    )

    rviz_goal_bridge_node = Node(
        package='ego_planner_bridge',
        executable='rviz_goal_bridge_node',
        name='rviz_goal_bridge_node',
        namespace=namespace,
        output='screen',
        # 不再需要init_x参数——改用tf2从消息自带的header.frame_id变换到
        # 这架飞机自己的<namespace>/odom，见rviz_goal_bridge_node.py。
        # 订阅rviz_goal_world/发布term_goal都是相对名，跟ego_planner_node
        # 同一个namespace下解析成/${NAMESPACE}/rviz_goal_world、
        # /${NAMESPACE}/term_goal，不需要remap。
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_drone_id)
    ld.add_action(declare_use_frame_alignment)
    ld.add_action(declare_num_agents)
    for arg in declare_tuning_args:
        ld.add_action(arg)
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)
    ld.add_action(rviz_goal_bridge_node)
    return ld
