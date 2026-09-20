"""真机部署专用：真实 Mid-360 驱动（livox_ros_driver2）启动文件。

2026-08-14新增，DEPLOY_TARGET=hw 时被 flight-stack-entrypoint.sh 用绝对路径
调用（`ros2 launch /opt/hw_launch/mid360_real.launch.py namespace:=...`），
不是安装到某个 ROS2 包里的 launch 文件，纯粹放在镜像的一个固定路径下。

不直接用 livox_ros_driver2 自带的 launch_ROS2/msg_MID360_launch.py，是因为
那份文件把 xfer_format/frame_id/publish_freq 等参数写死成 Python 顶层变量，
不是 launch 参数，改不了。这个项目的 DLIO 这条 SLAM 路径吃标准
sensor_msgs/PointCloud2（dlio.launch.py 的 pointcloud_topic 参数），跟
livox_ros_driver2 上游默认的 xfer_format=1（Livox CustomMsg 格式）不匹配，
必须用可配置的 launch 文件才能把 xfer_format 改成 0。

⚠️ 两处未经真实设备验证的假设，上机接线后务必核实（见
docker_sim/真机部署操作清单.md"3.6 裸机联调"一节）：
1. pointcloud_src_topic/imu_src_topic 默认值 'livox/lidar'/'livox/imu' 是
   Livox ROS2 驱动 multi_topic=0 时的常见默认命名，没有连过真实设备验证。
   上机后先 `ros2 topic list` 确认驱动实际发布的原始话题名，如果对不上，
   在 entrypoint 调用这个launch文件时加
   `pointcloud_src_topic:=xxx imu_src_topic:=xxx` 覆盖，不用改这个文件。
2. publish_freq=10.0 是从 livox_ros_driver2 自带 launch 文件抄来的默认值，
   没有针对这个项目实际点云处理负载单独调过。

frame_id 设成 "{namespace}/{namespace}_livox"——这是为了对齐仿真里 Gazebo
插件的命名习惯（entrypoint.sh 里有一条无条件发布的静态TF
"{ns}/lidar -> {ns}/{ns}_livox"，livox_ros_driver2_imu_frame_id.patch 让
IMU消息的frame_id也读这个可配置成员、不再硬编码"livox_frame"）——这样接上
真实驱动之后，DLIO/point_lio下游的TF查找链路不需要跟着改一行代码。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    config_path = LaunchConfiguration('config_path')
    publish_freq = LaunchConfiguration('publish_freq')
    pointcloud_src_topic = LaunchConfiguration('pointcloud_src_topic')
    imu_src_topic = LaunchConfiguration('imu_src_topic')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='NX01/NX02——必须跟MAVROS/DLIO/point_lio用同一个值',
    )
    declare_config_path = DeclareLaunchArgument(
        'config_path',
        default_value='/opt/livox_ws/src/livox_ros_driver2/config/MID360_config.json',
        description='雷达网络参数（host_net_info/lidar_configs里的IP），'
                     '当前是仓库里的占位IP(192.168.1.5/192.168.1.12)，'
                     '按实际接线改这个文件',
    )
    declare_publish_freq = DeclareLaunchArgument(
        'publish_freq', default_value='10.0',
        description='点云发布频率，抄自livox_ros_driver2自带launch文件默认值，未针对本项目单独调过',
    )
    declare_pointcloud_src_topic = DeclareLaunchArgument(
        'pointcloud_src_topic', default_value='livox/lidar',
        description='驱动实际发布的原始点云话题名——未验证的假设，上机后用ros2 topic list核实，不对就传这个参数覆盖',
    )
    declare_imu_src_topic = DeclareLaunchArgument(
        'imu_src_topic', default_value='livox/imu',
        description='驱动实际发布的原始IMU话题名，同上，未验证的假设',
    )

    frame_id = ParameterValue(
        PythonExpression(["'", namespace, "/", namespace, "_livox'"]),
        value_type=str,
    )

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'xfer_format': 0,          # 0=sensor_msgs/PointCloud2，DLIO这条路径需要的格式
                                        # （上游默认1=Livox CustomMsg，point_lio吃这个格式，
                                        # 但当前只有DLIO这一条真机SLAM路径接了真实驱动，见
                                        # entrypoint.sh里point_lio分支的DEPLOY_TARGET=hw拦截）
            'multi_topic': 0,          # 0=单雷达共用一个话题（这个项目一机一雷达，不需要区分）
            'data_src': 0,             # 0=lidar（数据源是雷达本身，不是录制文件回放）
            'publish_freq': ParameterValue(publish_freq, value_type=float),
            'output_data_type': 0,
            'frame_id': frame_id,
            'lvx_file_path': '/home/livox/livox_test.lvx',  # data_src=0时不会用到，保留上游默认值
            'user_config_path': config_path,
            'cmdline_input_bd_code': 'livox0000000001',      # 上游默认占位值，MID360真实IP走config_path里的host_net_info/lidar_configs
        }],
        remappings=[
            (pointcloud_src_topic, 'mid360_PointCloud2'),
            (imu_src_topic, 'mid360/imu'),
        ],
    )

    return LaunchDescription([
        declare_namespace,
        declare_config_path,
        declare_publish_freq,
        declare_pointcloud_src_topic,
        declare_imu_src_topic,
        livox_driver_node,
    ])
