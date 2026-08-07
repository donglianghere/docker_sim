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
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    px4ctrl_share = get_package_share_directory('px4ctrl')
    base_param_file = os.path.join(px4ctrl_share, 'config', 'ctrl_param_fpv.yaml')

    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('NAMESPACE', default_value='NX01'),
        description='NX01/NX02——必须跟MAVROS(namespace=<ns>/mavros)、mighty、DLIO用同一个值',
    )

    px4ctrl_node = Node(
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
                # docker_sim容器里没有真遥控器，用px4ctrl自带的no_RC模式（等价于
                # RC永远处于"hover挡+command挡+摇杆居中"状态），配合
                # takeoff_gate节点手动触发起飞、goal_to_poscmd持续喂cmd话题
                # 自动进CMD_CTRL状态跟踪mighty轨迹，不需要人在回路操作遥控器。
                'auto_takeoff_land.no_RC': True,
                'auto_takeoff_land.enable': True,
                'auto_takeoff_land.enable_auto_arm': True,
            },
        ],
        remappings=[
            # DLIO(patches/dlio_namespace.patch)在namespace=<ns>下实际发布的
            # 里程计话题是相对名"dlio/odom_node/odom"，跟mighty订阅state用的
            # convert_odom_to_state remap目标是同一个话题名，这里复用。
            ('odom', 'dlio/odom_node/odom'),
            # 'cmd'/'takeoff_land'不remap——goal_to_poscmd/takeoff_gate都在
            # 同一个namespace下发布到相对名'cmd'/'takeoff_land'，天然对上。
        ],
    )

    goal_to_poscmd_node = Node(
        package='px4ctrl_bridge',
        executable='goal_to_poscmd',
        name='goal_to_poscmd',
        namespace=namespace,
        output='screen',
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
        px4ctrl_node,
        goal_to_poscmd_node,
        px4_param_relax_node,
        takeoff_gate_node,
    ])
