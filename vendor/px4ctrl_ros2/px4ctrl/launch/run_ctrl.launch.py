import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("px4ctrl")
    param_file = os.path.join(pkg_share, "config", "ctrl_param_fpv.yaml")

    # ROS1原版run_ctrl.launch用<remap from="~odom" to="/vins_fusion/imu_propagate"/>
    # 把私有话题"~odom"重映射到实际的VIO里程计话题；ROS2下px4ctrl_node.cpp里
    # 订阅的是相对话题名"odom"/"cmd"（不带~前缀），用launch_ros的remappings=
    # 做同样的事情，效果完全对应。
    px4ctrl_node = Node(
        package="px4ctrl",
        executable="px4ctrl_node",
        name="px4ctrl",
        output="screen",
        parameters=[param_file],
        remappings=[
            ("odom", "/vins_fusion/imu_propagate"),
            ("cmd", "/position_cmd"),
        ],
    )

    return LaunchDescription([px4ctrl_node])
