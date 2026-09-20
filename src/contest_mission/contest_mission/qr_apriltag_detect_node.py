#!/usr/bin/env python3
"""仿真侧感知栈：QR二维码+AprilTag统一检测节点。

对应《2026大赛任务系统开发执行方案.md》阶段2.2。跟真机vision-stack
（`docker_real/docker_sim/src/vision_stack/`）不是同一套实现——真机是
"YOLO/QR自己内部取流"+"AprilTag走独立的apriltag_ros第三方包，通过
image_publisher_node桥接"，QR和AprilTag分别发布到不同话题/不同消息
格式（apriltag_ros发布的是`apriltag_msgs/AprilTagDetectionArray`，
不是vision_msgs）。仿真这边订阅Gazebo相机插件已经发布好的
`sensor_msgs/Image`（不需要直接取流那层），QR用pyzbar、AprilTag用纯
Python的`apriltag`库（同一个进程内两种都解，不单开一个apriltag_ros
子节点——仿真不需要真机那样的完整6DOF位姿解算，阶段2的验收标准只要
ID/文本+像素bbox），统一发布成`vision_msgs/Detection2DArray`到一个
话题，这是仿真侧特有的简化，理由见执行方案阶段2.2正文。

class_id字段约定：QR是解码文本原样（跟真机qr_detect_node.py一致）；
AprilTag是`apriltag:<tag_id>`（比如`apriltag:0`）——真机因为QR/AprilTag
分属不同话题，class_id不需要区分来源；这里两种检测结果共用一个话题，
不加前缀会有歧义（QR文本恰好是纯数字时没法跟AprilTag ID区分），所以
仿真这边多做了这一步区分，真机对接时不需要照抄这个约定。

2026-09-09新增MJPEG拉流：每路相机（front/down）各起一个独立端口的
`MjpegServer`（原样搬自真机`vision_stack/mjpeg_server.py`，见该文件
头说明），画好检测框的预览帧持续推进去——跟真机`qr_detect_node.py`/
`yolo_detector_node.py`同一套"检测节点顺手兼职推流"的模式，不单独起
一个专门的推流节点/桥接节点。两机共4路（NX01/NX02各前视+下视），
端口分配靠`mjpeg_port_base`参数（entrypoint按`AGENT_INDEX`算好传
进来，两机不会撞端口，见`flight-stack-entrypoint.sh`对应注释），
节点内部固定用`{'front': 0, 'down': 1}`这个偏移量映射相机名到端口
（不是按`cameras`参数的列表顺序动态分配——这样"前视永远是base_port+0、
下视永远是base_port+1"这个映射关系是确定性的，不会因为传参顺序变化
（比如`cameras:=[down,front]`）而跟着变）。
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from contest_mission.mjpeg_server import MjpegServer

# 相机名->端口偏移量，见文件头说明。以后如果加新相机（比如"up"）在这里
# 补一条映射即可，不需要改调用方逻辑。
_CAMERA_PORT_OFFSET = {'front': 0, 'down': 1, 'switchable': 0}

try:
    from pyzbar import pyzbar
except ImportError:
    pyzbar = None

try:
    import apriltag
except ImportError:
    apriltag = None


class QrApriltagDetectNode(Node):
    def __init__(self):
        super().__init__('qr_apriltag_detect_node')

        # 默认只订阅下视相机——NX01额外挂了前视相机时，entrypoint会传
        # cameras:=['front','down']覆盖。跟gen_iris_mid360_sdf.py里
        # CAMERA_MOUNTS的key（front/down）一一对应，传别的名字这个节点
        # 会订阅一个根本不存在的话题、安静收不到消息（不会报错崩溃，
        # 但也检测不出东西，排查时先查这个参数传的名字对不对）。
        self.declare_parameter('cameras', ['down'])
        self.declare_parameter('detections_topic', 'vision/detections')
        self.declare_parameter('apriltag_family', 'tag36h11')
        # MJPEG拉流起始端口，每路相机按_CAMERA_PORT_OFFSET固定偏移
        # （front=+0/down=+1）分配，entrypoint按AGENT_INDEX算好传进来，
        # 两机不会撞端口，见该文件头说明+flight-stack-entrypoint.sh
        # 对应注释。
        self.declare_parameter('mjpeg_port_base', 8180)
        self.declare_parameter('mjpeg_jpeg_quality', 80)
        # 2026-09-16临时诊断参数：怀疑检测回调间歇性假死是MJPEG拉流
        # （mjpeg_server.py的ThreadingTCPServer，每个没正常断开的连接
        # 都会留一个线程一直卡在Condition.wait，实测抓到过79个线程）
        # 拖慢/卡住了单线程rclpy.spin()——先加这个开关整体跳过MJPEG
        # 服务器启动，验证检测是不是就不再假死，不是永久性修复。
        # 2026-09-16诊断过程（分两步分别验证，不要合并成一次测就下结论）：
        # ①单独关MJPEG（还是单线程executor）：front独自能稳定跑5分钟不假死，
        #   但down从"稀疏但间歇性还工作"变成完全0——排除了"MJPEG是down
        #   问题唯一根因"这个假设。
        # ②单独分线程（MJPEG还开着）：front/down处理次数从156:47的悬殊
        #   比例变成30:24这样均衡很多——证明"两路相机共用单线程executor
        #   互相抢处理机会"这个假设成立，分线程确实有效缓解了这一点；
        #   但两路最后还是在同一时刻(约64秒后)一起停止了，说明MJPEG线程
        #   泄漏这个问题只要MJPEG开着依然存在，只是从"单独拖累down"变成
        #   "两路一起拖累"。
        # 结论：这是两个独立、叠加的问题，不是同一个根因的两种表现，都要
        # 修——分线程解决"处理机会分配不均"，关MJPEG解决"最终一起假死"，
        # 两个方案一起上，默认值最终定为False。
        self.declare_parameter('enable_mjpeg_stream', False)

        cameras = self.get_parameter('cameras').value
        detections_topic = self.get_parameter('detections_topic').value
        apriltag_family = self.get_parameter('apriltag_family').value
        mjpeg_port_base = int(self.get_parameter('mjpeg_port_base').value)
        enable_mjpeg_stream = bool(self.get_parameter('enable_mjpeg_stream').value)

        if pyzbar is None:
            self.get_logger().warn('pyzbar未安装，QR检测能力不可用（AprilTag检测不受影响）')
        if apriltag is None:
            self.get_logger().warn('apriltag库未安装，AprilTag检测能力不可用（QR检测不受影响）')
        # 2026-09-16：改成每路相机各自一个独立的apriltag.Detector()实例
        # （字典，按cam名索引），不再是所有相机共用同一个实例——这个改动
        # 是跟下面"每路相机独立callback group+MultiThreadedExecutor"配套
        # 的，两个改动分开测都有效果，但合在一起测的时候front/down几乎
        # 立刻双双假死，比单独任何一个问题都更严重：怀疑是`apriltag.
        # Detector`这个C库对象本身不是线程安全的，两路相机现在真的在两个
        # 不同OS线程上同时调用同一个`self.apriltag_detector.detect()`，
        # 触发了C库内部状态的并发读写，比原来"单线程排队/MJPEG线程堆积"
        # 这两个问题都严重（直接卡死，不是变慢/变稀疏）。给每路相机分配
        # 各自独立的Detector实例，从根上消除这个共享可变状态的并发访问。
        self.apriltag_detectors = {}
        if apriltag is not None:
            for cam in cameras:
                self.apriltag_detectors[cam] = apriltag.Detector(
                    apriltag.DetectorOptions(families=apriltag_family)
                )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, detections_topic, 10)

        # 2026-09-08 build后实测发现：libgazebo_ros_camera.so的ROS2移植版
        # 不认单相机模式CAMERA_SDF_TEMPLATE里配的<imageTopicName>/
        # <cameraInfoTopicName>标签（这两个是ROS1年代的参数名，ROS2端口
        # 静默忽略，不报错也不warn，排查花了一番功夫），实际发布话题是
        # 拿传感器自己的<sensor name>属性做前缀。
        # 2026-09-16几轮multicamera改造（详见gen_iris_mid360_sdf.py里
        # CAMERA_SDF_TEMPLATE上面那段注释）最终证明对down无效（内容渲染
        # 不出来，不是没发布），退回独立`<sensor type='camera'>`——话题
        # 是`<camera_name>/image_raw`（独立sensor没有子相机名这一层），
        # 即`{ns}_{cam}_camera/image_raw`。
        ns = self.get_namespace().strip('/')
        self._subs = []
        self._mjpeg_servers = {}
        # 2026-09-16尝试过给每路相机的订阅各自单独一个
        # MutuallyExclusiveCallbackGroup+main()里改成MultiThreadedExecutor
        # （想解决front/down共用单线程executor导致的"down被front挤掉处理
        # 机会"问题）——实测三次都比不改还差：front/down几乎立刻双双假死
        # （各自只在启动那一刻响一次，之后彻底沉默），改成每路相机独立
        # apriltag.Detector()实例（怀疑是共享Detector并发访问导致）也没
        # 解决，怀疑还有别的共享状态在并发访问下出问题（比如CvBridge共享
        # 实例），没能在合理时间内定位到，已撤回这个方向，退回最简单的
        # 单线程订阅（跟原来一样，所有订阅共用默认callback group+
        # rclpy.spin()）。"down比front稀疏"这个问题目前没有修，只是没有
        # 比这更糟——不要再往"分线程"这个方向尝试，除非先找到真正共享的
        # 是哪个对象。
        for cam in cameras:
            topic = f'{ns}_{cam}_camera/image_raw'
            frame_id = f'{ns}_camera_{cam}_optical_frame'
            sub = self.create_subscription(
                Image, topic,
                lambda msg, frame_id=frame_id, cam=cam: self._on_image(msg, frame_id, cam),
                qos_profile_sensor_data,
            )
            self._subs.append(sub)
            self.get_logger().info(f'订阅 "{topic}" (frame_id={frame_id})')

            if not enable_mjpeg_stream:
                continue
            if cam in _CAMERA_PORT_OFFSET:
                port = mjpeg_port_base + _CAMERA_PORT_OFFSET[cam]
                server = MjpegServer(port=port)
                server.start()
                self._mjpeg_servers[cam] = server
                self.get_logger().info(f'{cam}相机MJPEG拉流已启动，浏览器打开 http://<host-ip>:{port}/')
            else:
                self.get_logger().warn(
                    f"相机名'{cam}'不在_CAMERA_PORT_OFFSET里，跳过MJPEG拉流（不影响检测功能），"
                    f'加新相机时记得同步补一条端口偏移映射'
                )

        self.get_logger().info(f'检测结果发布到 "{detections_topic}"')

    def _on_image(self, msg: Image, frame_id: str, cam: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - cv_bridge报错类型不固定，转成警告日志而不是崩节点
            self.get_logger().warn(f'cv_bridge转换失败: {exc}', throttle_duration_sec=5.0)
            return

        # 2026-09-16临时诊断：用户要求截图看看multicamera改造之后front/
        # down两路实际拍到的画面，每路只存一次（存在文件就跳过），验证
        # 完记得删掉这段。
        import os
        _snapshot_path = f'/tmp/frame_{cam}.png'
        if not os.path.exists(_snapshot_path):
            cv2.imwrite(_snapshot_path, frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        out = Detection2DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = frame_id

        if pyzbar is not None:
            for sym in pyzbar.decode(gray):
                if sym.type != 'QRCODE':
                    continue
                try:
                    text = sym.data.decode('utf-8')
                except UnicodeDecodeError:
                    text = repr(sym.data)
                x, y, w, h = sym.rect
                out.detections.append(self._make_detection(out.header, text, x + w / 2.0, y + h / 2.0, w, h))

        apriltag_detector = self.apriltag_detectors.get(cam)
        if apriltag_detector is not None:
            apriltag_results = apriltag_detector.detect(gray)
            # 2026-09-14临时调试（E6排查down相机识别为什么live管线一直
            # 检测不到，用户要求加日志再测，排查完要记得删掉）：每帧打印
            # detect()原始返回个数，throttle 1秒避免刷屏。
            self.get_logger().info(
                f'[临时调试][{cam}] gray.shape={gray.shape} mean={gray.mean():.1f} std={gray.std():.1f} '
                f'detect()返回{len(apriltag_results)}个',
                throttle_duration_sec=1.0,
            )
            for det in apriltag_results:
                xs = det.corners[:, 0]
                ys = det.corners[:, 1]
                w = float(xs.max() - xs.min())
                h = float(ys.max() - ys.min())
                class_id = f'apriltag:{det.tag_id}'
                out.detections.append(
                    self._make_detection(out.header, class_id, float(det.center[0]), float(det.center[1]), w, h)
                )

        if out.detections:
            self.pub.publish(out)

        server = self._mjpeg_servers.get(cam)
        if server is not None:
            quality = int(self.get_parameter('mjpeg_jpeg_quality').value)
            server.update_frame(self._draw_preview(frame, out.detections), jpeg_quality=quality)

    @staticmethod
    def _draw_preview(frame, detections):
        """在原图拷贝上画检测框+class_id，推给MJPEG拉流用——不改
        发布出去的Detection2DArray本身（那个不需要可视化信息），纯粹
        是给人眼在浏览器里看的画面，跟真机`qr_detect_node.py`/
        `yolo_detector_node.py`往`preview`帧上画框再推流是同一个做法。
        """
        preview = frame.copy()
        for det in detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x0, y0 = int(cx - w / 2.0), int(cy - h / 2.0)
            x1, y1 = int(cx + w / 2.0), int(cy + h / 2.0)
            cv2.rectangle(preview, (x0, y0), (x1, y1), (0, 255, 0), 2)
            label = det.results[0].hypothesis.class_id if det.results else ''
            cv2.putText(
                preview, label, (x0, max(y0 - 6, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
            )
        return preview

    @staticmethod
    def _make_detection(header, class_id: str, cx: float, cy: float, w: float, h: float) -> Detection2D:
        det = Detection2D()
        det.header = header

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = 1.0  # 解码成功即视为确定，pyzbar/apriltag都不给置信度
        det.results.append(hyp)

        bbox = BoundingBox2D()
        bbox.center.position.x = cx
        bbox.center.position.y = cy
        bbox.center.theta = 0.0
        bbox.size_x = w
        bbox.size_y = h
        det.bbox = bbox

        return det


def main(args=None):
    rclpy.init(args=args)
    node = QrApriltagDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
