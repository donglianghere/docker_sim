"""AprilTag精准降落桥接：image_publisher_node（把GstCameraCapture桥接成标准
sensor_msgs/Image+CameraInfo）+ apriltag_ros的apriltag_node。

apriltag_ros是独立C++包，不在vision_stack代码里，走标准image_transport接口
订阅图像，这也是为什么需要image_publisher_node这一层桥接（YOLO/QR节点是
自己内部取流+推理一体，不需要这层，只有接第三方现成ROS包时才需要）。

⚠️ CameraInfo用的是占位内参，未真实标定，位姿解算精度不能代表真值，
见image_publisher_node.py文件头注释和docs/vision_stack.md阶段6。

2026-08-24加了`namespace`参数，跟flight-stack-hw/vision_stack.launch.py
用同一套NAMESPACE环境变量+默认值(NX01)，见read_hw.md同日"vision-stack
命名空间统一"条目。同时修了一个之前没发现的bug：下面`image_publisher`的
parameters字典键名一直写的是`image_topic`，但image_publisher_node.py
declare_parameter的实际参数名是`image_base_topic`——键名对不上，之前这个覆盖形同虚设(ROS2不会报错，只是这个参数没被真正传
进去，节点用的是自己代码里的默认值)。这次改成了正确的键名
`image_base_topic`。之前恰好没出问题是因为传进去的值和节点自身默认值
(camera/image_rect)刚好相同，纯属侥幸，不是这段代码本身正确。

2026-08-25第二路相机接上后加了`frame_id`参数：原来节点名/frame_id写死
`image_publisher_down`/`camera_down`，隐含"AprilTag只会跑在下视相机"的
假设。两路相机现在都要能按需切换跑YOLO/QR/AprilTag中任意一个（不是固定
分工），所以这两个改成跟`sensor_id`一样可传参——指哪个sensor_id就应该
配对传对应的frame_id（比如sensor_id=1配camera_down，sensor_id=0配
camera_front），调用方自己保证两者对应，节点本身不做强绑定。节点名固定
留`image_publisher`+`apriltag`不再叠加"_down"后缀，因为现在两个名字不
再必然对应下视相机，加后缀反而误导。

同一次改动加了`mjpeg_enabled`/`mjpeg_port`透传（跟yolo_detector_node.py/
qr_detect_node.py同款约定）：地面站视频统一改成MJPEG拉流，不管这一路相机
当前跑的是YOLO/QR/AprilTag中哪一个都应该能看画面，不是只有YOLO才配预览。

⚠️ 2026-08-25满载压测时踩到的真实bug：上面"节点名不加后缀"这个改动本身
是有问题的——只改了`frame_id`可传参，`image_base_topic`/`camera_info_
topic`/节点名`image_publisher`/`apriltag`还是写死的，没有按相机区分。
两路相机同时跑AprilTag（`cam0`+`cam1`都是sensor_id不同但都传
apriltag_stack.launch.py默认值）时，`ros2 node list`直接报警"nodes in
the graph that share an exact name"——两个`image_publisher`实例、两个
`apriltag`实例撞名，`camera/image_rect`/`camera/camera_info`/
`detections`这几个话题也撞在一起，两路相机的图像帧被同一个apriltag_node
实例混着处理，检测结果完全不可信。补救：加`label`参数(跟YOLO/QR节点的
`label`概念一致，默认`down`跟`frame_id`默认值配对)，节点名/话题名统一
带上`label`后缀区分（`image_publisher_<label>`/`apriltag_<label>`/
`camera/<label>/image_rect`等），两路各自独立，不再共享。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    sensor_id = LaunchConfiguration('sensor_id')
    frame_id = LaunchConfiguration('frame_id')
    label = LaunchConfiguration('label')
    mjpeg_enabled = LaunchConfiguration('mjpeg_enabled')
    mjpeg_port = LaunchConfiguration('mjpeg_port')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=os.environ.get('NAMESPACE', 'NX01'),
        description='跟flight-stack-hw共用的机身命名空间，多机部署时每架机固定一个',
    )
    declare_sensor_id = DeclareLaunchArgument(
        'sensor_id',
        default_value='1',
        description='要跑AprilTag的那一路相机的argus sensor-id，跟frame_id一起传，'
                     '调用方自己保证两者对应同一路物理相机（比如0配camera_front，'
                     '1配camera_down）——精准降落一般用下视，所以默认给1',
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_down',
        description='对应sensor_id那一路相机的frame_id，需要跟sensor_id手动配对传，'
                     '见上面sensor_id参数说明',
    )
    declare_label = DeclareLaunchArgument(
        'label',
        default_value='down',
        description='节点名/话题名用的标签，两路相机同时跑AprilTag时必须不同'
                     '（比如cam0传front、cam1传down），不然节点名/话题名会撞在'
                     '一起，见本文件头2026-08-25条目那次真实踩过的坑',
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

    tags_config = get_package_share_directory('vision_stack') + '/config/tags.yaml'

    # 话题/节点名都拼上label区分，两路相机同时跑AprilTag时不会撞名/撞话题
    # （2026-08-25满载压测踩过的坑，见文件头注释）。
    image_rect_topic = ['camera/', label, '/image_rect']
    camera_info_topic = ['camera/', label, '/camera_info']
    detections_topic = ['apriltag/', label, '/detections']

    image_publisher = Node(
        package='vision_stack',
        executable='image_publisher_node',
        name=['image_publisher_', label],
        namespace=namespace,
        output='screen',
        parameters=[{
            'sensor_id': sensor_id,
            'image_base_topic': image_rect_topic,
            'camera_info_topic': camera_info_topic,
            'frame_id': frame_id,
            'mjpeg_enabled': mjpeg_enabled,
            'mjpeg_port': mjpeg_port,
        }],
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name=['apriltag_', label],
        namespace=namespace,
        output='screen',
        remappings=[
            ('image_rect', image_rect_topic),
            ('camera_info', camera_info_topic),
            ('detections', detections_topic),
        ],
        parameters=[tags_config],
    )

    return LaunchDescription([
        declare_namespace,
        declare_sensor_id,
        declare_frame_id,
        declare_label,
        declare_mjpeg_enabled,
        declare_mjpeg_port,
        image_publisher,
        apriltag_node,
    ])
