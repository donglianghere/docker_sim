"""起两路点云高度过滤，只给地面站/RViz显示用，不改动原始话题——
dlio/odom_node/pointcloud/deskewed 和 grid_map/occupancy_inflate 各起一份
z_filter_node实例，共用同一对min_z/max_z相对高度窗口(两路都是同一个
odom/map坐标系、跟着同一个dlio/odom_node/odom走，没必要分开配两套)。

2026-08-27改成相对飞机当前高度的窗口(不再是固定绝对高度)，默认
min_z=-0.2/max_z=1.0，即"飞机当前高度往下0.2米到往上1米"这个区间。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')
    max_rate_hz = LaunchConfiguration('max_rate_hz')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='跟flight-stack-hw共用的机身命名空间',
    )
    declare_min_z = DeclareLaunchArgument(
        'min_z',
        default_value='-0.2',
        description='保留点云的高度下限(米，相对飞机当前高度的偏移量，'
                     '不是绝对高度)，低于"飞机当前高度+min_z"的点(比如地面'
                     '噪声)会被滤掉',
    )
    declare_max_rate = DeclareLaunchArgument(
        'max_rate_hz',
        default_value='0.0',
        description='处理频率上限[Hz]，0=不限速(默认，保持原有行为)。这个节点是'
                    'Python逐点处理全密度点云，真机实测吃满一个核，是整机负载最大'
                    '的单点；而下游是占据栅格/避障，10Hz足够(障碍物不会在100ms内'
                    '跑掉)。输入点云实测20Hz，限到10Hz省一半算力且不改变功能。',
    )

    declare_max_z = DeclareLaunchArgument(
        'max_z',
        default_value='1.0',
        description='保留点云的高度上限(米，相对飞机当前高度的偏移量)，'
                     '高于"飞机当前高度+max_z"的点(比如天花板)会被滤掉，'
                     '现场层高/空间不一样要跟着改这个值',
    )

    deskewed_filter = Node(
        package='pointcloud_z_filter',
        executable='z_filter_node',
        name='pointcloud_z_filter_deskewed',
        namespace=namespace,
        output='screen',
        parameters=[{
            'input_topic': 'dlio/odom_node/pointcloud/deskewed',
            'output_topic': 'dlio/odom_node/pointcloud/deskewed_z_filtered',
            'min_z': min_z,
            'max_z': max_z,
            'max_rate_hz': ParameterValue(max_rate_hz, value_type=float),
        }],
    )

    occupancy_filter = Node(
        package='pointcloud_z_filter',
        executable='z_filter_node',
        name='pointcloud_z_filter_occupancy',
        namespace=namespace,
        output='screen',
        parameters=[{
            'input_topic': 'grid_map/occupancy_inflate',
            'output_topic': 'grid_map/occupancy_inflate_z_filtered',
            'min_z': min_z,
            'max_z': max_z,
            'max_rate_hz': ParameterValue(max_rate_hz, value_type=float),
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_min_z,
        declare_max_z,
        declare_max_rate,
        deskewed_filter,
        occupancy_filter,
    ])
