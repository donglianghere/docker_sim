#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单路相机的二维码 / AprilTag 检测节点。**一路相机起一个实例**，只做一件事：
订阅相机图像话题 -> 检测 -> 发布结果。

    ros2 run contest_mission camera_tag_detect_node --ros-args \\
        -r __ns:=/NX01 -r __node:=camera_tag_detect_down -p camera:=down

为什么替换掉 qr_apriltag_detect_node（2026-09-22）：
先说清楚"某一路相机收不到图像"的根因——**不是检测节点**，是 socket 接收
缓冲太小。图像一帧约900KB，宿主机默认 net.core.rmem_max 只有208KB，仿真
把CPU压满时DDS的接收线程来不及取，分片被内核丢掉（/proc/net/snmp 的
RcvbufErrors 实测10秒涨1731次），一帧缺一片就整帧作废。哪一路能凑齐完整帧
全凭调度运气，所以表现为"这次这一路好、下次那一路好"。这个节点在没调缓冲时
同样一路满帧、一路0帧；调大缓冲（宿主机 rmem_max + CycloneDDS
SocketReceiveBufferSize）之后两路都稳定满帧率。

换成这个节点是用户要求的"每路相机一个独立节点"，另外带来下面几条好处。

跟原节点的区别：
1. 一路相机一个进程：一路出问题不会连带另一路，检测也真正并行。
2. **每一帧都发布**，没检测到就发空数组。原节点只在检测到东西时才发布，
   有两个问题：看话题频率分不出"没看到目标"和"节点卡死了"；而且下游按
   相机缓存"最新一条"时，最后那条"命中"会一直留着，目标早就离开画面了，
   之后再问"看到没有"仍然答"看到了"。
3. 状态日志按进程计，一路一个进程，限流天然按相机分开。原节点两路共用
   一个限流点，日志里某一路的行数不能说明它有没有在处理。
4. 没有 MJPEG 推送：检测节点只管检测。

输出格式跟原节点完全一致（vision_msgs/Detection2DArray，header.frame_id
形如 NX01_camera_down_optical_frame，class_id 为 'apriltag:<id>' 或二维码
文本），SDK 和其它下游不用改。
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from pyzbar import pyzbar
except Exception:  # noqa: BLE001 - 没装 pyzbar 就只做 AprilTag
    pyzbar = None
import apriltag


class CameraTagDetectNode(Node):

    def __init__(self):
        super().__init__('camera_tag_detect')
        self.declare_parameter('camera', 'down')
        self.declare_parameter('detections_topic', 'vision/detections')
        self.declare_parameter('apriltag_family', 'tag36h11')

        self.cam = str(self.get_parameter('camera').value)
        ns = self.get_namespace().strip('/')
        self.frame_id = f'{ns}_camera_{self.cam}_optical_frame'
        topic = f'{ns}_{self.cam}_camera/image_raw'

        self.bridge = CvBridge()
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families=str(self.get_parameter('apriltag_family').value)))
        self.pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter('detections_topic').value), 10)
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

        self._frames = 0
        self._hits = 0
        self._t_last = time.monotonic()
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'[{self.cam}] 订阅 "{topic}"，结果发到 '
                               f'"{self.get_parameter("detections_topic").value}"（frame_id={self.frame_id}）')

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - 单帧转换失败不该让节点退出
            self.get_logger().warn(f'[{self.cam}] cv_bridge 转换失败: {exc}', throttle_duration_sec=5.0)
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        out = Detection2DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        if pyzbar is not None:
            for sym in pyzbar.decode(gray):
                if sym.type != 'QRCODE':
                    continue
                try:
                    text = sym.data.decode('utf-8')
                except UnicodeDecodeError:
                    text = repr(sym.data)
                x, y, w, h = sym.rect
                out.detections.append(self._det(out.header, text, x + w / 2.0, y + h / 2.0, w, h))

        for r in self.detector.detect(gray):
            xs, ys = r.corners[:, 0], r.corners[:, 1]
            out.detections.append(self._det(
                out.header, f'apriltag:{r.tag_id}', float(r.center[0]), float(r.center[1]),
                float(xs.max() - xs.min()), float(ys.max() - ys.min())))

        self.pub.publish(out)          # 每帧都发，见文件头第2条
        self._frames += 1
        self._hits += bool(out.detections)

    def _report(self):
        now = time.monotonic()
        dt = now - self._t_last
        self.get_logger().info(
            f'[{self.cam}] 最近{dt:.0f}秒处理 {self._frames} 帧（{self._frames / dt:.1f}Hz），'
            f'其中 {self._hits} 帧有检测结果')
        if self._frames == 0:
            self.get_logger().warn(f'[{self.cam}] 最近{dt:.0f}秒一帧都没收到，检查相机话题')
        self._frames = 0
        self._hits = 0
        self._t_last = now

    @staticmethod
    def _det(header, class_id: str, cx: float, cy: float, w: float, h: float) -> Detection2D:
        d = Detection2D()
        d.header = header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = 1.0      # 解码成功即视为确定，pyzbar/apriltag 都不给置信度
        d.results.append(hyp)
        box = BoundingBox2D()
        box.center.position.x = cx
        box.center.position.y = cy
        box.size_x = w
        box.size_y = h
        d.bbox = box
        return d


def main(args=None):
    rclpy.init(args=args)
    node = CameraTagDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():             # Ctrl-C 时 rclpy 已经自己 shutdown 过
            rclpy.shutdown()


if __name__ == '__main__':
    main()
