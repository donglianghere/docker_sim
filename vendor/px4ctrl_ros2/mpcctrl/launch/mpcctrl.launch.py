import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mpcctrl")
    param_file = os.path.join(pkg_share, "config", "mpc_ctrl_param.yaml")

    # 跟px4ctrl的run_ctrl.launch.py同一个remap约定：节点内部订阅相对话题名
    # "odom"/"cmd"，这里用remappings接到实际的里程计/轨迹指令话题上。
    mpcctrl_node = Node(
        package="mpcctrl",
        executable="mpcctrl_node",
        name="mpcctrl",
        output="screen",
        parameters=[param_file],
        remappings=[
            ("odom", "/vins_fusion/imu_propagate"),
            ("cmd", "/position_cmd"),
        ],
    )

    return LaunchDescription([mpcctrl_node])
