"""docker_sim集成专用launch文件：pt4ctrl_node + goal_to_poscmd + px4_param_relax
+ takeoff_gate 四个节点打包一起启动，全部挂在同一个namespace（NX01/NX02）下。

跟px4ctrl_docker_sim.launch.py（这份文件的原型，逐段对照着改的）比，唯一
实质差异是px4ctrl_base_kwargs的parameters列表里少了mass/
thrust_model.hover_percentage这两个覆盖项——pt4ctrl没有控制律，
config/ctrl_param_fpv.yaml里根本不存在mass/thrust_model这两个key，
PX4CtrlParam.cpp也不会去读，硬传这两个参数不会报错（ROS2允许传多余的
parameters，未被declare_parameter读取的条目会被静默忽略），但会让人误以为
这两个数字对pt4ctrl也有意义，索性不传，避免维护两份"看起来一样、其实只有
一份真正生效"的VEHICLE_MASS_KG/VEHICLE_HOVER_THRUST覆盖逻辑。

其余部分（namespace/planner参数声明、mighty/ego_planner两个cmd remap
分支、goal_to_poscmd/px4_param_relax/takeoff_gate三个共用节点）跟
px4ctrl_docker_sim.launch.py完全一致，见那份文件的完整注释，这里不重复。
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
    pt4ctrl_share = get_package_share_directory('pt4ctrl')
    base_param_file = os.path.join(pt4ctrl_share, 'config', 'ctrl_param_fpv.yaml')

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
        description='mighty/ego_planner——决定pt4ctrl的cmd话题接goal_to_poscmd还是position_cmd',
    )
    is_ego_planner = PythonExpression(["'", planner, "' == 'ego_planner'"])

    pt4ctrl_base_kwargs = dict(
        package='pt4ctrl',
        executable='pt4ctrl_node',
        name='pt4ctrl',
        namespace=namespace,
        output='screen',
        parameters=[
            base_param_file,
            {
                # docker_sim容器里没有真遥控器，用pt4ctrl自带的no_RC模式（等价于
                # RC永远处于"hover挡+command挡+摇杆居中"状态），配合
                # takeoff_gate节点手动触发起飞、cmd话题持续喂目标点
                # 自动进CMD_CTRL状态跟踪规划器轨迹，不需要人在回路操作遥控器。
                'auto_takeoff_land.no_RC': True,
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
                # 2026-09-07新增：yaw锁定开关，见docker_sim/DEBUG_JOURNAL.md
                # 同日期条目完整设计讨论。默认false(行为不变)，仿真里先验证
                # 再决定要不要在真机上开。
                'yaw_lock_enabled': ParameterValue(
                    EnvironmentVariable('PX4CTRL_YAW_LOCK_ENABLED', default_value='false'), value_type=bool),
            },
        ],
    )
    pt4ctrl_node_mighty = Node(
        **pt4ctrl_base_kwargs,
        remappings=[
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'不remap——goal_to_poscmd在同一个namespace下发布到相对名
            # 'cmd'，天然对上。
        ],
        condition=UnlessCondition(is_ego_planner),
    )
    pt4ctrl_node_ego_planner = Node(
        **pt4ctrl_base_kwargs,
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
        condition=UnlessCondition(is_ego_planner),
    )

    # px4_param_relax/takeoff_gate两个节点跟具体控制器完全无关（前者只走
    # mavros/param/set服务，后者只发quadrotor_msgs/TakeoffLand），
    # px4ctrl_bridge包里已有的实现直接复用，不需要为pt4ctrl另写一份。
    px4_param_relax_node = Node(
        package='px4ctrl_bridge',
        executable='px4_param_relax',
        name='px4_param_relax',
        namespace=namespace,
        output='screen',
    )

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
        pt4ctrl_node_mighty,
        pt4ctrl_node_ego_planner,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])
