"""docker_sim集成专用launch文件：px4ctrl_node + goal_to_poscmd + px4_param_relax
+ takeoff_gate 四个节点打包一起启动，全部挂在同一个namespace（NX01/NX02）下。

跟flight-stack-entrypoint.sh现有的调用习惯对齐：
  - namespace通过NAMESPACE环境变量传入（跟mighty/DLIO/ros2_px4_stack一致）
  - mass/悬停油门通过VEHICLE_MASS_KG/VEHICLE_HOVER_THRUST环境变量传入
    （跟ros2_px4_stack_dynus.patch里get_thrust()/get_angular()读的是同一份，
    保证两个控制器组件不会出现质量/悬停推力数字不同步的问题）

参数优先级：px4ctrl的基础config/ctrl_param_fpv.yaml先加载，环境变量覆盖项
放在parameters=[]列表后面——ROS2 launch的parameters列表按顺序合并，后面的
条目覆盖前面同名参数，不需要像mighty那样在entrypoint里现算一份yaml。

PLANNER环境变量（默认mighty）决定px4ctrl_node的cmd话题接到哪个规划器：
  - mighty：goal_to_poscmd把mighty的Goal消息转成PositionCommand发到相对名
    'cmd'，px4ctrl_node的'cmd'不remap，天然对上（现状不变）。
  - ego_planner：ego-planner-swarm的traj_server原生就发PositionCommand，
    相对话题名是'position_cmd'（打过
    patches/ego_planner_traj_server_relative_poscmd.patch），不需要
    goal_to_poscmd这个消息转换桥，px4ctrl_node的'cmd'直接remap到
    'position_cmd'；goal_to_poscmd_node这时不启动（订阅的mighty Goal消息
    根本不会有人发布，起了也是空等）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    px4ctrl_share = get_package_share_directory('px4ctrl')
    base_param_file = os.path.join(px4ctrl_share, 'config', 'ctrl_param_fpv.yaml')

    namespace = LaunchConfiguration('namespace')
    planner = LaunchConfiguration('planner')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('NAMESPACE', default_value='NX01'),
        description='NX01/NX02——必须跟MAVROS(namespace=<ns>/mavros)、mighty、DLIO用同一个值',
    )
    declare_planner = DeclareLaunchArgument(
        'planner',
        default_value=EnvironmentVariable('PLANNER', default_value='ego_planner'),
        description='mighty/ego_planner——决定px4ctrl的cmd话题接goal_to_poscmd还是position_cmd',
    )
    is_ego_planner = PythonExpression(["'", planner, "' == 'ego_planner'"])

    px4ctrl_base_kwargs = dict(
        package='px4ctrl',
        executable='px4ctrl_node',
        name='px4ctrl',
        namespace=namespace,
        output='screen',
        parameters=[
            base_param_file,
            {
                # mass/hover_percentage覆盖成跟mighty/ros2_px4_stack同一份数字，
                # 而不是ctrl_param_fpv.yaml里那个1.2kg/0.30的通用默认值。
                'mass': ParameterValue(
                    EnvironmentVariable('VEHICLE_MASS_KG', default_value='1.2'), value_type=float),
                'thrust_model.hover_percentage': ParameterValue(
                    EnvironmentVariable('VEHICLE_HOVER_THRUST', default_value='0.30'), value_type=float),
                # 2026-08-27新增：跟px4ctrl_hw.launch.py同步的一批外置参数，
                # 完整说明见那份文件同一处注释——姿态角限幅/手动模式限速/
                # 低电压阈值/精确推力模型标定系数(K1/K2/K3)/级联PID增益
                # (Kp/Kv/KAng三组，Kvi*/Kvd*两组yaml标注"No use now"没有外置)。
                'max_angle': ParameterValue(
                    EnvironmentVariable('PX4CTRL_MAX_ANGLE_DEG', default_value='30.0'), value_type=float),
                'max_manual_vel': ParameterValue(
                    EnvironmentVariable('PX4CTRL_MAX_MANUAL_VEL', default_value='1.0'), value_type=float),
                'low_voltage': ParameterValue(
                    EnvironmentVariable('PX4CTRL_LOW_VOLTAGE', default_value='13.2'), value_type=float),
                'thrust_model.K1': ParameterValue(
                    EnvironmentVariable('PX4CTRL_THRUST_K1', default_value='0.7583'), value_type=float),
                'thrust_model.K2': ParameterValue(
                    EnvironmentVariable('PX4CTRL_THRUST_K2', default_value='1.6942'), value_type=float),
                'thrust_model.K3': ParameterValue(
                    EnvironmentVariable('PX4CTRL_THRUST_K3', default_value='0.6786'), value_type=float),
                'gain.Kp0': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KP0', default_value='1.5'), value_type=float),
                'gain.Kp1': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KP1', default_value='1.5'), value_type=float),
                'gain.Kp2': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KP2', default_value='1.5'), value_type=float),
                'gain.Kv0': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KV0', default_value='1.5'), value_type=float),
                'gain.Kv1': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KV1', default_value='1.5'), value_type=float),
                'gain.Kv2': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KV2', default_value='1.5'), value_type=float),
                'gain.KAngR': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KANGR', default_value='20.0'), value_type=float),
                'gain.KAngP': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KANGP', default_value='20.0'), value_type=float),
                'gain.KAngY': ParameterValue(
                    EnvironmentVariable('PX4CTRL_GAIN_KANGY', default_value='20.0'), value_type=float),
                # 2026-09-07新增：yaw锁定开关，见docker_sim/DEBUG_JOURNAL.md
                # 同日期条目完整设计讨论。默认false(行为不变)，仿真里先验证
                # 再决定要不要在真机上开。
                'yaw_lock_enabled': ParameterValue(
                    EnvironmentVariable('PX4CTRL_YAW_LOCK_ENABLED', default_value='false'), value_type=bool),
                # docker_sim容器里没有真遥控器，用px4ctrl自带的no_RC模式（等价于
                # RC永远处于"hover挡+command挡+摇杆居中"状态），配合
                # takeoff_gate节点手动触发起飞、cmd话题持续喂目标点
                # 自动进CMD_CTRL状态跟踪规划器轨迹，不需要人在回路操作遥控器。
                'auto_takeoff_land.no_RC': True,
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
                # 2026-09-26：起降爬升/下降速度也拉成环境变量。这个速度跟规划器
                # 的 max_vel(V_MAX) 毫无关系——AUTO_TAKEOFF/AUTO_LAND 是开环匀速
                # 积分（PX4CtrlFSM.cpp::get_takeoff_land_des()：
                # des.p = 起飞点 + (0,0,speed*Δt)），不经过规划器、不发轨迹；
                # V_MAX 只约束 B 样条轨迹的速度可行性，起飞完成切到 CMD_CTRL
                # 跟踪轨迹之后才起作用。yaml 里原值 0.3 m/s，这里保持同一个默认。
                # 降落用的是同一个参数取负号，所以调大它起飞和降落一起变快。
                'auto_takeoff_land.takeoff_land_speed': ParameterValue(
                    EnvironmentVariable('PX4CTRL_TAKEOFF_LAND_SPEED', default_value='0.3'),
                    value_type=float),
            },
        ],
    )
    # odom这条remap两种planner都一样（DLIO/gt_odom_bridge发的都是这同一个
    # 相对话题名），cmd这条按planner二选一——同一个px4ctrl_node定义不了
    # "remappings列表按条件变化"，用两份Node+IfCondition/UnlessCondition
    # 互斥代替，同一时刻只有一份会真正启动，不会两边都抢px4ctrl这个节点名。
    px4ctrl_node_mighty = Node(
        **px4ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'不remap——goal_to_poscmd在同一个namespace下发布到相对名
            # 'cmd'，天然对上。
        ],
        condition=UnlessCondition(is_ego_planner),
    )
    px4ctrl_node_ego_planner = Node(
        **px4ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 2026大赛任务系统阶段3：不再直接接traj_server的'position_cmd'，
            # 改接position_cmd_relay_node（contest_mission包）的中继输出
            # 'position_cmd_relayed'——traj_server/px4ctrl_node本身都没有
            # 改一行代码，只是这里的remap目标换了个名字，中继节点默认
            # relay_mode=normal时原样透传，行为跟改造前完全一致。见
            # contest_mission/position_cmd_relay_node.py文件头的拓扑说明。
            ('cmd', 'position_cmd_relayed'),
        ],
        condition=IfCondition(is_ego_planner),
    )

    goal_to_poscmd_node = Node(
        package='px4ctrl_bridge',
        executable='goal_to_poscmd',
        name='goal_to_poscmd',
        namespace=namespace,
        output='screen',
        # ego_planner模式下不需要这条Goal->PositionCommand转换桥（traj_server
        # 原生发PositionCommand），起了也没有mighty发Goal消息喂给它，纯粹
        # 空等，不启动更干净。
        condition=UnlessCondition(is_ego_planner),
    )

    # 一次性节点：放宽PX4失控保护参数。放在px4ctrl_node之后起没有强制先后
    # 依赖关系（它只是走mavros/param/set服务，跟px4ctrl本身是否已经起来
    # 无关），但把它跟px4ctrl放在同一个launch文件里能保证"只要py4ctrl这条
    # 链路被启动，参数放宽就一定会跑一次"，不需要在entrypoint.sh里单独
    # 再记一行。
    px4_param_relax_node = Node(
        package='px4ctrl_bridge',
        executable='px4_param_relax',
        name='px4_param_relax',
        namespace=namespace,
        output='screen',
    )

    # 一次性节点：等/tmp/takeoff_go出现再触发起飞，配合
    # docker_sim/scripts/launch_control.sh的操作习惯。
    takeoff_gate_node = Node(
        package='px4ctrl_bridge',
        executable='takeoff_gate',
        name='takeoff_gate',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_planner,
        px4ctrl_node_mighty,
        px4ctrl_node_ego_planner,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])
