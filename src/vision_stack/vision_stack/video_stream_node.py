#!/usr/bin/env python3
"""纯视频直通节点：取流 -> MJPEG拉流，中间不做任何推理、不画任何叠加。

2026-09-23新增。起因：任务机(NX02)两路相机都跑着yolo_detector_node，
地面站8080/8081拉到的画面是**画完检测框之后**的图，用户要的是"直通
视频"——相机出来什么样，地面就看什么样。

两种需求两条路，不要混：
- 还要YOLO检测结果、只是画面不要叠加框 → 不用这个节点，给
  yolo_detector_node传 `mjpeg_draw_detections:=false`，检测消息照常
  发布，只是预览推原始帧（还省掉每帧一次frame.copy()+画框的开销）。
- 压根不需要推理、只要一路干净视频 → 用这个节点，连engine都不加载，
  省掉GPU推理那部分算力。

⚠️ 同一路相机同一时刻只能有一个Argus会话（见docs/vision_stack.md
阶段8），所以这个节点跟同一sensor_id上的YOLO/QR/AprilTag节点是**互斥**
的，不能为了"既要检测又要直通"在同一路相机上同时起两个——那种场景走
上面第一条路（关掉画框），不是起两个节点。

"直通"指的是不叠加任何检测结果，不是"不编码"——MJPEG本身仍然要把BGR
帧软编码成JPEG（mjpeg_server.py的选型理由见该文件头注释）。
"""
import rclpy
from rclpy.node import Node

from vision_stack.camera_capture import GstCameraCapture
from vision_stack.mjpeg_server import MjpegServer


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')

        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_framerate', 30)
        # 往MJPEG缓冲区写帧的频率。相机本身30fps，这里默认15Hz跟其他
        # 检测节点对齐；纯直通没有推理开销，要更流畅可以往上调到30，
        # 代价是JPEG软编码的CPU按比例涨。
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('mjpeg_port', 8080)

        sensor_id = self.get_parameter('sensor_id').value
        cam_w = self.get_parameter('camera_width').value
        cam_h = self.get_parameter('camera_height').value
        cam_fps = self.get_parameter('camera_framerate').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        mjpeg_port = self.get_parameter('mjpeg_port').value

        self.get_logger().info(
            f'video passthrough: sensor-id={sensor_id} {cam_w}x{cam_h}@{cam_fps}, '
            f'MJPEG port={mjpeg_port}'
        )
        self.camera = GstCameraCapture(
            sensor_id=sensor_id, width=cam_w, height=cam_h, framerate=cam_fps,
        )
        self.camera.start()

        self.mjpeg_server = MjpegServer(port=mjpeg_port)
        self.mjpeg_server.start()
        self.get_logger().info(
            f'直通视频已启动（无检测叠加），浏览器打开 http://<jetson-ip>:{mjpeg_port}/'
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

    def on_timer(self):
        frame = self.camera.read()
        if frame is None:
            self.get_logger().warn('camera read timeout, no frame', throttle_duration_sec=5.0)
            return
        # 直接推原始帧，不copy、不画框
        self.mjpeg_server.update_frame(frame, jpeg_quality=self.jpeg_quality)

    def destroy_node(self):
        self.camera.stop()
        self.mjpeg_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
