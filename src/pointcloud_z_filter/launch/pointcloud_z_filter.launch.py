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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    enabled = LaunchConfiguration('enabled')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')
    max_rate_hz = LaunchConfiguration('max_rate_hz')
    projector_enabled = LaunchConfiguration('projector_enabled')
    projector_resolution = LaunchConfiguration('projector_resolution')
    projector_rate_hz = LaunchConfiguration('projector_rate_hz')
    lazy_subscribe = LaunchConfiguration('lazy_subscribe')

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
    declare_max_rate = DeclareLaunchArgument(
        'max_rate_hz',
        default_value='5.0',
        description='处理频率上限[Hz]，0=不限速。这个节点用Python处理整帧点云，'
                    '真机实测占满一个核；而它唯一的下游是地面站网页俯视图，'
                    '5Hz足够看障碍物。',
    )

    declare_lazy = DeclareLaunchArgument(
        'lazy_subscribe',
        default_value='true',
        description='输出话题没订阅者时连输入都不订阅。上游GridMap::'
                    'publishMapInflate()本身就有"没订阅者就直接return"的判断，'
                    '所以这里不订阅=上游不构造不序列化不发送，全链路零开销。',
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

    declare_projector_enabled = DeclareLaunchArgument(
        'projector_enabled',
        default_value='true',
        description='是否起 occupancy_projector（把三维占据点云投影成 2D '
                    'OccupancyGrid，49 KB/帧 @0.1m，比点云小约 100 倍）。'
                    '地面站不看障碍物时可以传 false 关掉',
    )
    declare_projector_resolution = DeclareLaunchArgument(
        'projector_resolution',
        default_value='0.1',
        description='投影栅格分辨率(米)。0.1 时 20x25m 场地 = 200x250 = 49 KB/帧；'
                    '要更省就调大到 0.2(12 KB)',
    )
    declare_projector_rate_hz = DeclareLaunchArgument(
        'projector_rate_hz',
        default_value='2.0',
        description='投影输出的频率上限(Hz)。0=不限。显示用 2Hz 足够，'
                    '实测整条热路径 1.49 ms/帧，2Hz 时约 0.3% CPU',
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
            'max_rate_hz': ParameterValue(max_rate_hz, value_type=float),
            'lazy_subscribe': ParameterValue(lazy_subscribe, value_type=bool),
            'min_z': min_z,
            'max_z': max_z,
        }],
    )

    # 2026-10-03 新增：2D 投影支路。和上面的 z_filter 是**并列**的两条显示支路，
    # 都直接接原始 occupancy_inflate，互不依赖——投影不需要 odom（z_filter 要读
    # odom 算相对高度窗口，没收到 odom 就整帧跳过），少一个依赖少一类卡死。
    # 两条都惰性订阅，地面站只订其中一条时另一条完全不干活。
    occupancy_projector = Node(
        package='pointcloud_z_filter',
        executable='occupancy_projector_node',
        name='occupancy_projector',
        namespace=namespace,
        output='screen',
        condition=IfCondition(projector_enabled),
        parameters=[{
            'input_topic': 'grid_map/occupancy_inflate',
            'output_topic': 'grid_map/occupancy_2d',
            'resolution': ParameterValue(projector_resolution, value_type=float),
            'max_rate_hz': ParameterValue(projector_rate_hz, value_type=float),
            'lazy_subscribe': ParameterValue(lazy_subscribe, value_type=bool),
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_enabled,
        declare_min_z,
        declare_max_z,
        declare_max_rate,
        declare_lazy,
        declare_projector_enabled,
        declare_projector_resolution,
        declare_projector_rate_hz,
        occupancy_filter,
        occupancy_projector,
    ])
