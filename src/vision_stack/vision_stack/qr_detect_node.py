#!/usr/bin/env python3
"""QR二维码识别ROS2节点。

阶段5起步版本：下视相机整帧直接喂pyzbar解码（纯CPU），先验证功能正确，
GPU定位裁剪(用YOLO加一个"QR区域"类别做ROI裁剪再喂给pyzbar)是后续性能优化项，
见docs/vision_stack.md阶段5——这版先不做，等阶段8实测CPU余量不够用再加。

检测结果复用vision_msgs/Detection2DArray，不新建自定义消息——Detection2D的
`results[].hypothesis.class_id`是字符串字段，直接拿来装解码出的文本内容，
`bbox`装二维码在图像里的像素位置，完全覆盖了"解码内容+像素坐标+时间戳"这个
需求，没必要为了这点差异单独起一个rosidl接口包（vision_stack是纯
ament_python包，加自定义.msg还得另起一个ament_cmake接口包，得不偿失）。

2026-08-25加了MJPEG预览（跟yolo_detector_node.py同款做法）：两路相机现在
都要能按需切换跑YOLO/QR/AprilTag，不管当前跑哪个功能都应该能拉流看画面，
不是只有YOLO才配有预览。
"""
import cv2
import rclpy
from pyzbar import pyzbar
from rclpy.node import Node
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from vision_stack.camera_capture import GstCameraCapture
from vision_stack.mjpeg_server import MjpegServer


class QrDetectNode(Node):
    def __init__(self):
        super().__init__('qr_detect_node')

        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_framerate', 30)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('topic', 'vision/down/qr_detections')
        self.declare_parameter('frame_id', 'camera_down')
        # 端口跟yolo_detector_node.py的mjpeg_port是同一套约定：两路相机
        # 同时要看画面时端口要分开传，不能共用。
        self.declare_parameter('mjpeg_enabled', False)
        self.declare_parameter('mjpeg_port', 8080)

        sensor_id = self.get_parameter('sensor_id').value
        cam_w = self.get_parameter('camera_width').value
        cam_h = self.get_parameter('camera_height').value
        cam_fps = self.get_parameter('camera_framerate').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        mjpeg_enabled = self.get_parameter('mjpeg_enabled').value
        mjpeg_port = self.get_parameter('mjpeg_port').value

        self.get_logger().info(f'opening sensor-id={sensor_id} {cam_w}x{cam_h}@{cam_fps} for QR decode')
        self.camera = GstCameraCapture(sensor_id=sensor_id, width=cam_w, height=cam_h, framerate=cam_fps)
        self.camera.start()

        self.mjpeg_server = None
        if mjpeg_enabled:
            self.mjpeg_server = MjpegServer(port=mjpeg_port)
            self.mjpeg_server.start()
            self.get_logger().info(f'MJPEG预览已启动，浏览器打开 http://<jetson-ip>:{mjpeg_port}/')

        self.pub = self.create_publisher(Detection2DArray, topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(f'publishing QR detections on "{topic}" at {publish_rate} Hz')

    def on_timer(self):
        frame = self.camera.read()
        if frame is None:
            self.get_logger().warn('camera read timeout, no frame', throttle_duration_sec=5.0)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        symbols = pyzbar.decode(gray)

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for sym in symbols:
            if sym.type != 'QRCODE':
                continue
            try:
                text = sym.data.decode('utf-8')
            except UnicodeDecodeError:
                text = repr(sym.data)

            x, y, w, h = sym.rect

            det_msg = Detection2D()
            det_msg.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = text
            hyp.hypothesis.score = 1.0  # pyzbar不给置信度，解码成功即视为确定
            det_msg.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center.position.x = float(x + w / 2.0)
            bbox.center.position.y = float(y + h / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = float(w)
            bbox.size_y = float(h)
            det_msg.bbox = bbox

            msg.detections.append(det_msg)

        self.pub.publish(msg)

        if self.mjpeg_server is not None:
            self._update_preview(frame, symbols)

    def _update_preview(self, frame, symbols):
        # 画QR框只为了MJPEG预览好看，不影响上面已经发布的检测消息
        preview = frame.copy()
        for sym in symbols:
            if sym.type != 'QRCODE':
                continue
            x, y, w, h = sym.rect
            cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.mjpeg_server.update_frame(preview)

    def destroy_node(self):
        self.camera.stop()
        if self.mjpeg_server is not None:
            self.mjpeg_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
