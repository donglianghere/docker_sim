"""docker_sim集成专用launch文件：so3ctrl_node + goal_to_poscmd + px4_param_relax
+ takeoff_gate 四个节点打包一起启动，全部挂在同一个namespace（NX01/NX02）下。

跟px4ctrl_docker_sim.launch.py是同一个模式（so3ctrl包本身就是px4ctrl的
飞行状态机+kr_mav_control控制律拼出来的独立包，见
docker_sim/vendor/px4ctrl_ros2/so3ctrl/NOTICE.md）——goal_to_poscmd/
px4_param_relax/takeoff_gate这三个节点是px4ctrl_bridge包里已经写好、
跟具体走哪个控制器无关的公共胶水节点，直接复用，不用为so3ctrl单独
再写一份。这个launch文件本身跟px4ctrl_docker_sim.launch.py唯一的区别
就是package/executable/name从'px4ctrl'/'px4ctrl_node'/'px4ctrl'换成
'so3ctrl'/'so3ctrl_node'/'so3ctrl'，其余（namespace/mass/hover_thrust
环境变量透传、PLANNER决定cmd话题接goal_to_poscmd还是position_cmd）
逐字照抄。

跟flight-stack-entrypoint.sh现有的调用习惯对齐：
  - namespace通过NAMESPACE环境变量传入（跟mighty/DLIO/ros2_px4_stack/
    px4ctrl一致）
  - mass/悬停油门通过VEHICLE_MASS_KG/VEHICLE_HOVER_THRUST环境变量传入
    （跟ros2_px4_stack_dynus.patch/px4ctrl_docker_sim.launch.py读的是
    同一份，保证三个控制器组件不会出现质量/悬停推力数字不同步的问题）

参数优先级：so3ctrl的基础config/ctrl_param_fpv.yaml先加载，环境变量覆盖项
放在parameters=[]列表后面——ROS2 launch的parameters列表按顺序合并，后面的
条目覆盖前面同名参数。

PLANNER环境变量（默认mighty）决定so3ctrl_node的cmd话题接到哪个规划器：
  - mighty：goal_to_poscmd把mighty的Goal消息转成PositionCommand发到相对名
    'cmd'，so3ctrl_node的'cmd'不remap，天然对上（现状不变）。
  - ego_planner：ego-planner-swarm的traj_server原生就发PositionCommand，
    相对话题名是'position_cmd'，不需要goal_to_poscmd这个消息转换桥，
    so3ctrl_node的'cmd'直接remap到'position_cmd'；goal_to_poscmd_node
    这时不启动（订阅的mighty Goal消息根本不会有人发布，起了也是空等）。
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
    so3ctrl_share = get_package_share_directory('so3ctrl')
    base_param_file = os.path.join(so3ctrl_share, 'config', 'ctrl_param_fpv.yaml')

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
        description='mighty/ego_planner——决定so3ctrl的cmd话题接goal_to_poscmd还是position_cmd',
    )
    is_ego_planner = PythonExpression(["'", planner, "' == 'ego_planner'"])

    so3ctrl_base_kwargs = dict(
        package='so3ctrl',
        executable='so3ctrl_node',
        name='so3ctrl',
        namespace=namespace,
        output='screen',
        parameters=[
            base_param_file,
            {
                # mass/hover_percentage覆盖成跟mighty/ros2_px4_stack/px4ctrl
                # 同一份数字，而不是ctrl_param_fpv.yaml里那个1.2kg/0.30的
                # 通用默认值。
                'mass': ParameterValue(
                    EnvironmentVariable('VEHICLE_MASS_KG', default_value='1.2'), value_type=float),
                'thrust_model.hover_percentage': ParameterValue(
                    EnvironmentVariable('VEHICLE_HOVER_THRUST', default_value='0.30'), value_type=float),
                # 2026-09-07新增：yaw锁定开关，见docker_sim/DEBUG_JOURNAL.md
                # 同日期条目完整设计讨论。默认false(行为不变)，仿真里先验证
                # 再决定要不要在真机上开。
                'yaw_lock_enabled': ParameterValue(
                    EnvironmentVariable('PX4CTRL_YAW_LOCK_ENABLED', default_value='false'), value_type=bool),
                # docker_sim容器里没有真遥控器，用so3ctrl(继承自px4ctrl)
                # 自带的no_RC模式（等价于RC永远处于"hover挡+command挡+
                # 摇杆居中"状态），配合takeoff_gate节点手动触发起飞、cmd
                # 话题持续喂目标点自动进CMD_CTRL状态跟踪规划器轨迹。
                'auto_takeoff_land.no_RC': True,
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
            },
        ],
    )
    # odom这条remap两种planner都一样（DLIO/gt_odom_bridge发的都是这同一个
    # 相对话题名），cmd这条按planner二选一——同一个so3ctrl_node定义不了
    # "remappings列表按条件变化"，用两份Node+IfCondition/UnlessCondition
    # 互斥代替，同一时刻只有一份会真正启动，不会两边都抢so3ctrl这个节点名。
    so3ctrl_node_mighty = Node(
        **so3ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'不remap——goal_to_poscmd在同一个namespace下发布到相对名
            # 'cmd'，天然对上。
        ],
        condition=UnlessCondition(is_ego_planner),
    )
    so3ctrl_node_ego_planner = Node(
        **so3ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 2026大赛任务系统阶段3：接position_cmd_relay_node的中继输出，
            # 不再直接接traj_server，见px4ctrl_docker_sim.launch.py同一处
            # 改动的注释、contest_mission/position_cmd_relay_node.py文件头。
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

    # 一次性节点：放宽PX4失控保护参数。跟走哪个板外控制器无关，
    # px4ctrl_bridge包里已有、两条CONTROLLER路径共用同一份代码。
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
        so3ctrl_node_mighty,
        so3ctrl_node_ego_planner,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])
