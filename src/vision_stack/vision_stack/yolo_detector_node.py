#!/usr/bin/env python3
"""YOLO26 TensorRT目标识别ROS2节点。

取流(camera_capture.GstCameraCapture) -> 推理(yolo_trt_infer.YoloTrtDetector)
-> 发布vision_msgs/Detection2DArray。不经过Ultralytics的Python推理封装
（原因见docs/vision_stack.md阶段4/read_hw.md 2026-08-22小节：Ultralytics的
TensorRT后端硬性要求CUDA版torch，这里故意用CPU-only torch，冲突）。
"""
import cv2
import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)

from vision_stack.camera_capture import GstCameraCapture
from vision_stack.mjpeg_server import MjpegServer
from vision_stack.yolo_trt_infer import YoloTrtDetector, COCO_NAMES


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('engine_path', '/models/yolo26n.engine')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_framerate', 30)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('topic', 'vision/front/detections')
        self.declare_parameter('frame_id', 'camera_front')
        # 2026-08-22新增，2026-08-25起改成地面站看画面的唯一方式（拉流，
        # 不再有RTP推流）：浏览器直接打开http://<jetson-ip>:<mjpeg_port>/
        # 就能看画面（含YOLO检测框）。默认关闭，不占用额外CPU；两路相机
        # 都要看画面时端口要分开传(比如前视8080/下视8081)，不能共用。
        self.declare_parameter('mjpeg_enabled', False)
        self.declare_parameter('mjpeg_port', 8080)
        # 2026-09-23新增：MJPEG画面是否叠加检测框。地面站要"直通视频"
        # （相机原始画面，不带任何检测叠加）时传false——检测消息
        # (Detection2DArray)照常发布、频率不变，只是预览推的是原始帧，
        # 顺带省掉每帧一次frame.copy()+画框画字的CPU。
        # 完全不需要推理、只要一路干净视频时，用video_stream_node，
        # 不要用这个参数（那样还白跑一遍YOLO），见video_stream_node.py头注释。
        self.declare_parameter('mjpeg_draw_detections', True)

        sensor_id = self.get_parameter('sensor_id').value
        engine_path = self.get_parameter('engine_path').value
        conf_thres = self.get_parameter('conf_thres').value
        cam_w = self.get_parameter('camera_width').value
        cam_h = self.get_parameter('camera_height').value
        cam_fps = self.get_parameter('camera_framerate').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        mjpeg_enabled = self.get_parameter('mjpeg_enabled').value
        mjpeg_port = self.get_parameter('mjpeg_port').value
        self.mjpeg_draw_detections = self.get_parameter('mjpeg_draw_detections').value

        self.get_logger().info(
            f'loading engine {engine_path}, opening sensor-id={sensor_id} '
            f'{cam_w}x{cam_h}@{cam_fps}'
        )
        self.detector = YoloTrtDetector(engine_path, conf_thres=conf_thres)
        self.camera = GstCameraCapture(
            sensor_id=sensor_id, width=cam_w, height=cam_h, framerate=cam_fps,
        )
        self.camera.start()

        self.mjpeg_server = None
        if mjpeg_enabled:
            self.mjpeg_server = MjpegServer(port=mjpeg_port)
            self.mjpeg_server.start()
            overlay = '含检测框' if self.mjpeg_draw_detections else '直通原始画面，不叠加检测框'
            self.get_logger().info(
                f'MJPEG预览已启动({overlay})，浏览器打开 http://<jetson-ip>:{mjpeg_port}/'
            )

        self.pub = self.create_publisher(Detection2DArray, topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(f'publishing detections on "{topic}" at {publish_rate} Hz')

    def on_timer(self):
        frame = self.camera.read()
        if frame is None:
            self.get_logger().warn('camera read timeout, no frame', throttle_duration_sec=5.0)
            return

        dets = self.detector.infer(frame)

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for d in dets:
            det_msg = Detection2D()
            det_msg.header = msg.header

            hyp = ObjectHypothesisWithPose()
            class_id = d['class_id']
            hyp.hypothesis.class_id = (
                COCO_NAMES[class_id] if class_id < len(COCO_NAMES) else str(class_id)
            )
            hyp.hypothesis.score = d['conf']
            det_msg.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center.position.x = (d['x1'] + d['x2']) / 2.0
            bbox.center.position.y = (d['y1'] + d['y2']) / 2.0
            bbox.center.theta = 0.0
            bbox.size_x = d['x2'] - d['x1']
            bbox.size_y = d['y2'] - d['y1']
            det_msg.bbox = bbox

            msg.detections.append(det_msg)

        self.pub.publish(msg)

        if self.mjpeg_server is not None:
            if self.mjpeg_draw_detections:
                self._update_preview(frame, dets)
            else:
                self.mjpeg_server.update_frame(frame)

    def _update_preview(self, frame, dets):
        # 画检测框只为了MJPEG预览好看，不影响上面已经发布的检测消息
        preview = frame.copy()
        for d in dets:
            class_id = d['class_id']
            name = COCO_NAMES[class_id] if class_id < len(COCO_NAMES) else str(class_id)
            p1 = (int(d['x1']), int(d['y1']))
            p2 = (int(d['x2']), int(d['y2']))
            cv2.rectangle(preview, p1, p2, (0, 255, 0), 2)
            cv2.putText(preview, f"{name} {d['conf']:.2f}", (p1[0], max(p1[1] - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.mjpeg_server.update_frame(preview)

    def destroy_node(self):
        self.camera.stop()
        self.detector.close()
        if self.mjpeg_server is not None:
            self.mjpeg_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
