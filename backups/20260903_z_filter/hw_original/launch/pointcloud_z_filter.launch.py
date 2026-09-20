"""起一路点云高度过滤，只给地面站/RViz显示用，不改动原始话题——
只过滤grid_map/occupancy_inflate(障碍点云)，不再碰dlio/odom_node/
pointcloud/deskewed(SLAM里程计模块自己输出的点云)。

2026-08-28：用户明确要求去掉对SLAM输出点云的滤波，只保留障碍点云这一路
——SLAM/DLIO那路点云是定位解算的输入/诊断数据，不应该被这类纯展示用途
的下游处理碰，即使这个节点本身只读不回写、架构上不影响DLIO（2026-08-27
那次DLIO段错误排查已经确认过这一点），保留碰它的必要性也不大，干脆去掉
更干净、也少一路~50%CPU的Python点云处理开销。

同一次改动加了`enabled`开关(默认true)——不需要看点云画面时可以直接关掉，
不用去手动kill进程，也避免容器重建/重启时进程"复活"回来的问题(之前只能
手动kill、但DEPLOY_TARGET=hw每次重启都会无条件重新拉起，没有真正的开关)。

2026-08-27改成相对飞机当前高度的窗口(不再是固定绝对高度)，默认
min_z=-0.2/max_z=1.0，即"飞机当前高度往下0.2米到往上1米"这个区间。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    enabled = LaunchConfiguration('enabled')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='跟flight-stack-hw共用的机身命名空间',
    )
    declare_enabled = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='是否起点云高度过滤节点，默认开启；不需要看点云画面时'
                     '(比如省这~0.5核CPU)可以传false关掉',
    )
    declare_min_z = DeclareLaunchArgument(
        'min_z',
        default_value='-0.2',
        description='保留点云的高度下限(米，相对飞机当前高度的偏移量，'
                     '不是绝对高度)，低于"飞机当前高度+min_z"的点(比如地面'
                     '噪声)会被滤掉',
    )
    declare_max_z = DeclareLaunchArgument(
        'max_z',
        default_value='1.0',
        description='保留点云的高度上限(米，相对飞机当前高度的偏移量)，'
                     '高于"飞机当前高度+max_z"的点(比如天花板)会被滤掉，'
                     '现场层高/空间不一样要跟着改这个值',
    )

    occupancy_filter = Node(
        package='pointcloud_z_filter',
        executable='z_filter_node',
        name='pointcloud_z_filter_occupancy',
        namespace=namespace,
        output='screen',
        condition=IfCondition(enabled),
        parameters=[{
            'input_topic': 'grid_map/occupancy_inflate',
            'output_topic': 'grid_map/occupancy_inflate_z_filtered',
            'min_z': min_z,
            'max_z': max_z,
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_enabled,
        declare_min_z,
        declare_max_z,
        occupancy_filter,
    ])
