"""vision-stack目前唯一的节点：yolo_detector_node，起一路YOLO实例。

2026-08-25第二路相机接上后澄清的使用模式：两路相机(sensor_id=0/1)都要能
按需独立切换跑YOLO/QR/AprilTag中任意一个，不是"前视固定YOLO/下视固定
QR"这种绑死的分工。这个launch文件只管起**一路**YOLO（用哪个sensor_id/
topic/frame_id由调用方传参决定，不预设是前视还是下视）；两路都要跑YOLO
就调用两次（第二次传不同的sensor_id/topic/frame_id）；同一路相机同一
时刻只能有一个Argus会话，不能既跑YOLO又跑QR/AprilTag——见
docs/vision_stack.md阶段8"使用模式澄清"、operation.md"起节点"一节。

2026-08-24加了`namespace`参数，跟flight-stack-hw用同一套`NAMESPACE`环境
变量+同样的默认值(NX01)——两边命名空间必须保持一致，不能各设各的，见
read_hw.md同日"vision-stack命名空间统一"条目。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    engine_path = LaunchConfiguration('engine_path')
    conf_thres = LaunchConfiguration('conf_thres')
    sensor_id = LaunchConfiguration('sensor_id')
    topic = LaunchConfiguration('topic')
    frame_id = LaunchConfiguration('frame_id')
    node_name = LaunchConfiguration('node_name')
    mjpeg_enabled = LaunchConfiguration('mjpeg_enabled')
    mjpeg_port = LaunchConfiguration('mjpeg_port')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        # 默认值读容器自己的NAMESPACE环境变量（docker-compose.hw.yml已经
        # 设成跟flight-stack-hw同一个变量+默认值NX01），没有这个环境变量
        # 时（比如本地不在容器里直接跑）兜底NX01，跟flight-stack-entrypoint.
        # sh的兜底值保持一致。
        default_value=os.environ.get('NAMESPACE', 'NX01'),
        description='跟flight-stack-hw共用的机身命名空间，多机部署时每架机固定一个',
    )
    declare_engine_path = DeclareLaunchArgument(
        'engine_path',
        default_value='/models/yolo26n.engine',
        description='TensorRT引擎文件路径，跟硬件/TensorRT版本绑定，见docs/vision_stack.md',
    )
    declare_conf_thres = DeclareLaunchArgument(
        'conf_thres',
        default_value='0.25',
        description='检测置信度阈值',
    )
    # 2026-08-25：sensor_id/topic/frame_id/node_name都改成可传参，默认值
    # 保持原来的前视配置不变（不传参数照旧行为不变）。要在下视/第二路相机
    # 上起YOLO，调用方自己传一套配对的sensor_id+topic+frame_id+node_name，
    # 不要跟另一路正在跑的实例撞名/撞话题，见本文件头注释。
    declare_sensor_id = DeclareLaunchArgument(
        'sensor_id',
        default_value='0',
        description='要跑YOLO的那一路相机的argus sensor-id',
    )
    declare_topic = DeclareLaunchArgument(
        'topic',
        default_value='vision/front/detections',
        description='检测结果发布话题，换相机时记得跟着改，避免跟另一路撞名',
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_front',
        description='对应sensor_id那一路相机的frame_id，需要跟sensor_id手动配对传',
    )
    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='yolo_detector_front',
        description='节点名，换相机时记得跟着改，避免跟另一路撞名',
    )
    declare_mjpeg_enabled = DeclareLaunchArgument(
        'mjpeg_enabled',
        default_value='false',
        description='是否开本地MJPEG拉流预览，默认关闭不占用额外CPU',
    )
    declare_mjpeg_port = DeclareLaunchArgument(
        'mjpeg_port',
        default_value='8080',
        description='MJPEG预览端口，两路相机同时开预览时要传不同端口',
    )

    yolo_node = Node(
        package='vision_stack',
        executable='yolo_detector_node',
        name=node_name,
        namespace=namespace,
        output='screen',
        parameters=[{
            'sensor_id': sensor_id,
            'engine_path': engine_path,
            'conf_thres': conf_thres,
            'topic': topic,
            'frame_id': frame_id,
            'mjpeg_enabled': mjpeg_enabled,
            'mjpeg_port': mjpeg_port,
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_engine_path,
        declare_conf_thres,
        declare_sensor_id,
        declare_topic,
        declare_frame_id,
        declare_node_name,
        declare_mjpeg_enabled,
        declare_mjpeg_port,
        yolo_node,
    ])
