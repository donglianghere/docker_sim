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
    改成接DLIO自己发布的dlio/odom_node/deskewed——DLIO的odom.cc里
    publishCloud()对这份点云做了`pcl::transformPointCloud(...)`并把
    header.frame_id设成跟odom消息同一个this->odom_frame，是已经变换到
    odom坐标系、跟里程计天然一致的点云，grid_map.cpp那套"点云和odom同
    坐标系"的假设在这份数据上才成立。
    代价：LOCALIZATION_SOURCE=gt时DLIO根本不跑，没有deskewed这个话题
    ——ego_planner+gt目前没有可用的点云源，是明确的已知限制，还没做
    （需要一个订阅原始点云+TF、发布变换后点云的小节点，仿照
    global_mapper_ros的做法），验证ego_planner目前只能用
    LOCALIZATION_SOURCE=dlio。
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
    绝对路径），这样scripts/dual_goal_input.py和RViz现成的"2D Goal
    Pose (NX01/NX02)"工具不用改、也不用关心当前是哪个规划器，直接就能
    用。2026-08-07发现的撞柱子事故里，目标点很可能就是通过某个没有
    world->local换算的临时路径直接送进了这个话题，(-3,0,1)当作局部坐标
    解读正好落在NX01本地地图原点附近、也就是世界坐标的房间中心——统一
    到term_goal之后，dual_goal_input.py自带的"世界坐标->每机局部坐标"
    换算（按AGENT_INDEX*3的INIT_X偏移）对ego_planner同样适用（它的
    odom也是DLIO在each机自己spawn点为原点的局部坐标系，跟mighty同一套
    假设），不会再出现坐标解读不一致的问题。
  - 没有起map_generator/mockamap（那是ego-planner自带demo用来生成
    虚拟障碍物地图的节点，这里用真实Gazebo世界，不需要）。
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

    # "${NAMESPACE}/map"字符串拼接——parameters=[]的字典值不支持像
    # name=[...]那样直接传list做拼接（会被当成数组类型参数），用
    # PythonExpression拼出普通字符串。
    map_frame_id = PythonExpression(["'", namespace, "' + '/map'"])

    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)

    declare_tuning_args = [
        DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X'),
        DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y'),
        DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z'),
        DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity'),
        DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration'),
        DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon'),
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
            # odom_frame的deskewed点云。只在LOCALIZATION_SOURCE=dlio时存在，
            # =gt时DLIO不跑，见文件头部说明。
            ('grid_map/cloud', 'dlio/odom_node/deskewed'),
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
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
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
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.099},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
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
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},

            {'grid_map/virtual_ceil_height': 2.9},
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

            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},
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
            {'traj_server/time_forward': -1.0},
        ],
        # planning/bspline（ego_planner_node发，traj_server收）和
        # position_cmd（打过ego_planner_traj_server_relative_poscmd.patch
        # 后是相对名）都不需要remap，两个节点在同一个namespace下天然对上。
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_drone_id)
    for arg in declare_tuning_args:
        ld.add_action(arg)
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)
    return ld
