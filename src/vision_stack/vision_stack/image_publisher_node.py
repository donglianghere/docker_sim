#!/usr/bin/env python3
"""把GstCameraCapture取到的帧发布成标准sensor_msgs/CompressedImage + CameraInfo，
供apriltag_ros经image_transport的compressed transport订阅。

存在的意义：apriltag_ros是独立的C++包（不在vision_stack内），走标准ROS2
image_transport接口订阅图像，不能直接调我们的Python GstCameraCapture类。这个
节点是两者之间的桥——YOLO/QR节点自己内部取流+推理一体，不需要这一层，只有
要接第三方现成ROS包（apriltag_ros）时才需要"发布标准图像话题"这一步。

⚠️ CameraInfo这里发布的是**占位内参**（未标定），不是真实相机标定结果——
畸变系数全0、焦距是按imx219常见视场角粗估的。AprilTag检测本身（找到tag在
像素坐标系的位置）不受影响，但位姿解算(6D pose，精准降落要用的那个)精度
不能代表真实值，正式用于降落前必须换成真实标定结果（`camera_calibration`
包+棋盘格标定板）。见docs/vision_stack.md阶段6。

2026-08-22实测踩坑记录（完整排查过程见read_hw.md对应小节，这里只记结论）：
- 没用`cv_bridge`——这台机`ros-humble-cv-bridge`(3.2.1)编译的
  `cv_bridge_boost`扩展内部类型编码表跟纯Python的`encoding_to_cvtype2()`
  对不上（通道乘数32 vs标准的8），是环境本身的包兼容性问题。
- 一开始发原始未压缩`sensor_msgs/Image`（1280x720 BGR，~2.76MB/帧），
  实测只能到~1~1.3Hz；换CycloneDDS一样慢，证明瓶颈不在RMW传输层，是rclpy
  序列化这么大的`uint8[]`字节数组本身就要515ms+。**改成发布JPEG压缩后的
  `sensor_msgs/CompressedImage`**——体积通常只有原始的1/10~1/20，序列化
  成本按比例大幅下降，可以在1280x720全分辨率下用。
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from vision_stack.camera_capture import GstCameraCapture
from vision_stack.mjpeg_server import MjpegServer


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_framerate', 30)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        # image_transport的compressed transport约定：base topic是
        # "camera/image_rect"，实际发布话题是它的兄弟话题
        # "camera/image_rect/compressed"（sensor_msgs/CompressedImage），
        # apriltag_ros那边只要把image_transport参数设成"compressed"就会
        # 自动订阅这个话题并透明解压，不需要改它的代码。
        self.declare_parameter('image_base_topic', 'camera/image_rect')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('frame_id', 'camera_down')
        # imx219常见配置约78度水平视场角，按针孔模型粗估fx/fy——不是标定值，
        # 见文件头注释
        self.declare_parameter('approx_hfov_deg', 78.0)
        # 2026-08-25加了MJPEG预览（跟yolo_detector_node.py/qr_detect_node.py
        # 同款做法）：AprilTag桥接跑的这一路相机也要能拉流看画面。这里显示
        # 的是桥接前的原始帧，不含AprilTag检测框（画框逻辑在apriltag_ros
        # 内部，这一层拿不到检测结果，只做"确认画面本身正常"用途）。
        self.declare_parameter('mjpeg_enabled', False)
        self.declare_parameter('mjpeg_port', 8080)

        sensor_id = self.get_parameter('sensor_id').value
        cam_w = self.get_parameter('camera_width').value
        cam_h = self.get_parameter('camera_height').value
        cam_fps = self.get_parameter('camera_framerate').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        image_base_topic = self.get_parameter('image_base_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        hfov_deg = self.get_parameter('approx_hfov_deg').value
        mjpeg_enabled = self.get_parameter('mjpeg_enabled').value
        mjpeg_port = self.get_parameter('mjpeg_port').value

        self.camera = GstCameraCapture(sensor_id=sensor_id, width=cam_w, height=cam_h, framerate=cam_fps)
        self.camera.start()

        self.mjpeg_server = None
        if mjpeg_enabled:
            self.mjpeg_server = MjpegServer(port=mjpeg_port)
            self.mjpeg_server.start()
            self.get_logger().info(f'MJPEG预览已启动，浏览器打开 http://<jetson-ip>:{mjpeg_port}/')

        self.camera_info = self._make_placeholder_camera_info(cam_w, cam_h, hfov_deg)

        compressed_topic = image_base_topic + '/compressed'
        self.image_pub = self.create_publisher(CompressedImage, compressed_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().warn(
            'CameraInfo内参是占位估算值，未真实标定——AprilTag位姿解算精度'
            '不能代表真值，正式用于降落前必须先做相机标定'
        )
        self.get_logger().info(
            f'publishing "{compressed_topic}" (JPEG q={self.jpeg_quality}) + '
            f'"{camera_info_topic}" at {publish_rate} Hz'
        )

    @staticmethod
    def _make_placeholder_camera_info(width: int, height: int, hfov_deg: float) -> CameraInfo:
        import math
        fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
        fy = fx  # 假设方形像素
        cx, cy = width / 2.0, height / 2.0

        info = CameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _numpy_to_compressed_imgmsg(self, frame) -> CompressedImage:
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError('JPEG编码失败')
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        return msg

    def on_timer(self):
        frame = self.camera.read()
        if frame is None:
            self.get_logger().warn('camera read timeout, no frame', throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self._numpy_to_compressed_imgmsg(frame)
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.info_pub.publish(self.camera_info)

        if self.mjpeg_server is not None:
            self.mjpeg_server.update_frame(frame)

    def destroy_node(self):
        self.camera.stop()
        if self.mjpeg_server is not None:
            self.mjpeg_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
