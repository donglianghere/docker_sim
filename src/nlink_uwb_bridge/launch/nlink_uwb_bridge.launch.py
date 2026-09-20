"""docker_sim集成专用launch文件：真实UWB驱动(nlink_parser2的linktrack_node)+
nlink_pose_bridge_node两个节点一起拉起，跟uwb_origin_bridge.launch.py的
origin_setter+frame_align_bridge是同一个封装习惯。

⚠️ 这个launch文件目前没有被flight-stack-entrypoint.sh引用、不会随容器启动
自动跑起来——串口设备名(port_name)要按实际接线核对、具体在哪个
LOCALIZATION_SOURCE模式下启动这个真实UWB驱动，留到真机联调阶段再决定，
现在只保证镜像里编译好了、手动`ros2 launch nlink_uwb_bridge
nlink_uwb_bridge.launch.py namespace:=NX01 port_name:=/dev/ttyXXX`能跑起来。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='NX01/NX02——必须跟MAVROS/mighty/DLIO/uwb_origin_bridge用同一个值',
    )
    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyCH343USB0',
        description='LinkTrack标签的串口设备名，按实际接线核对',
    )
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
    )

    linktrack_node = Node(
        package='nlink_parser2',
        executable='linktrack_node',
        name='linktrack0',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port_name': port_name,
            'baud_rate': baud_rate,
        }],
    )

    nlink_pose_bridge_node = Node(
        package='nlink_uwb_bridge',
        executable='nlink_pose_bridge_node',
        name='nlink_pose_bridge',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_port_name,
        declare_baud_rate,
        linktrack_node,
        nlink_pose_bridge_node,
    ])
