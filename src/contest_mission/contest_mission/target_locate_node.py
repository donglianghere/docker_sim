#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""下视相机目标定位：把检测结果的像素位置换算成目标在地面上的实际坐标。

    ros2 run contest_mission target_locate_node --ros-args -r __ns:=/NX01 \\
        -p use_sim_time:=false -p odom_stamp_offset_s:=1735689600.0

2026-09-22 新增。原来"发现火点"记下的是**飞机自己**的位置：火点在画面边缘
时，2.5 米高度下两者能差 1.5 米；飞机在飞，检测结果又比拍照时刻晚，还会再
差一截。这个节点放在飞行栈里（用户要求），因为只有这里能同时拿到按时间对齐
的位姿历史、相机内参和完整姿态；SDK 只读结果。

每一帧检测：
1. 拍照时刻的位姿：按图像时间戳在里程计历史里插值（位置线性、姿态 slerp）。
   图像比最新一条里程计还新时，按最近的速度外推——这就是速度补偿；飞机以
   1 m/s 飞、检测晚 0.2 秒，不补偿就差 0.2 米。
2. 射线：相机内参（camera_info 的 K，有畸变系数时先去畸变）把像素变成相机
   光学系下的方向，再经安装外参、机体完整姿态（含横滚/俯仰，飞行中前倾几度
   在 2.5 米高度上就是十几厘米）转到局部系。
3. 地面高度：定高测距仪读数 × cos(倾角) = 机体到地面的竖直距离，机体高度减去
   它就是脚下地面的高度，取最近几秒的中位数。射线和这个水平面求交得到目标
   坐标。假设目标所在的地面跟飞机正下方同高（离得不过两三米）。

时间基准（仿真里三路不一样，别弄混）：
- 图像 header.stamp：仿真时间（真机：墙钟）。
- 里程计 header.stamp：仿真里 gt_odom_bridge 打的是"仿真时间 + 1735689600"
  （见该文件头），所以查历史前要减掉 odom_stamp_offset_s；真机 DLIO 是墙钟，
  偏移为 0。
- 测距仪 header.stamp：mavros 打的墙钟，跟图像对不上，所以不按时间戳配对，
  而是跟"收到测距时最新的一条里程计"配对算地面高度——地面高度变化慢，够用。

输出 `vision/target_positions`（vision_msgs/Detection3DArray）：
- header.stamp = 图像时间戳，header.frame_id = 里程计的 frame_id（飞机自己的
  局部系，跟 SDK get_local_position() 同一个系）。
- 每个目标一条 Detection3D：results[0].hypothesis.class_id 同检测结果，
  results[0].pose.pose.position 是目标坐标（z 为估计的地面高度）。
- 没有检测到目标、或者算不出来（没位姿/射线不朝地面）的帧不发布。
"""
import bisect
import math
import statistics
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Range
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

try:
    import cv2
except Exception:  # noqa: BLE001 - 没有 cv2 就不做去畸变（仿真相机本来就没有畸变）
    cv2 = None


def _stamp(s) -> float:
    return s.sec + s.nanosec * 1e-9


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(dot)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


# 相机 link 系（SDF/URDF 约定：+X 是镜头朝向）-> 光学系（x 右、y 下、z 镜头朝向）。
# 光学系的三个轴在 link 系下分别是 -Y、-Z、+X。
R_LINK_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def ray_ground_intersection(
    cam_pos: np.ndarray, ray_dir: np.ndarray, ground_z: float,
) -> Optional[np.ndarray]:
    """射线与水平面 z=ground_z 的交点；射线不朝下（平行或朝上）返回 None。"""
    if ray_dir[2] >= -1e-6:
        return None
    s = (ground_z - cam_pos[2]) / ray_dir[2]
    if s <= 0.0:
        return None
    return cam_pos + s * ray_dir


class TargetLocateNode(Node):

    def __init__(self):
        super().__init__('target_locate_node')
        self.declare_parameter('camera', 'down')
        # 相机 link 在机体系下的安装位姿，SDF 约定（+X 是镜头朝向）。
        # 默认值 = 仿真模型 iris_mid360 的 NX0x_down_camera_link。
        self.declare_parameter('camera_xyz', [0.0, 0.0, -0.0217])
        self.declare_parameter('camera_rpy', [0.0, 1.5707963, 0.0])
        self.declare_parameter('odom_topic', 'dlio/odom_node/odom')
        self.declare_parameter('odom_stamp_offset_s', 0.0)   # 见文件头"时间基准"
        self.declare_parameter('range_topic', 'mavros/hrlv_ez4_pub')
        # 测距仪装在机体系下的位置（竖直方向），用来把读数换成机体原点到地面的距离
        self.declare_parameter('range_z_offset_m', 0.0)
        self.declare_parameter('ground_window_s', 2.0)
        self.declare_parameter('max_extrapolate_s', 0.3)
        self.declare_parameter('history_s', 3.0)
        self.declare_parameter('detections_topic', 'vision/detections')
        self.declare_parameter('output_topic', 'vision/target_positions')

        self.cam = str(self.get_parameter('camera').value)
        ns = self.get_namespace().strip('/')
        self._frame_tag = f'_camera_{self.cam}_'
        rx, ry, rz = [float(v) for v in self.get_parameter('camera_rpy').value]
        self._R_body_opt = rpy_to_rot(rx, ry, rz) @ R_LINK_OPTICAL
        self._t_body_cam = np.array([float(v) for v in self.get_parameter('camera_xyz').value])
        self._odom_offset = float(self.get_parameter('odom_stamp_offset_s').value)
        self._range_dz = float(self.get_parameter('range_z_offset_m').value)
        self._ground_window = float(self.get_parameter('ground_window_s').value)
        self._max_extrap = float(self.get_parameter('max_extrapolate_s').value)
        self._history_s = float(self.get_parameter('history_s').value)

        # 位姿历史：按（已去掉偏移的）时间升序
        self._hist_t: deque = deque()
        self._hist: deque = deque()          # (pos ndarray, quat ndarray[x,y,z,w])
        self._odom_frame = 'map'
        # 地面高度样本：(里程计时间, ground_z)
        self._ground: deque = deque()
        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None
        self._last_lag: Optional[float] = None   # 图像时间 - 最新里程计时间，自检用
        self._stats = {'det': 0, 'ok': 0, 'no_pose': 0, 'no_ground': 0, 'no_hit': 0}

        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self._on_odom, 50)
        # mavros 按传感器 QoS 发布，RELIABLE 订阅收不到（formation_follower_node 同样的坑）
        self.create_subscription(Range, str(self.get_parameter('range_topic').value), self._on_range,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CameraInfo, f'{ns}_{self.cam}_camera/camera_info',
                                 self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, str(self.get_parameter('detections_topic').value),
                                 self._on_detections, 10)
        self.pub = self.create_publisher(
            Detection3DArray, str(self.get_parameter('output_topic').value), 10)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'[{self.cam}] 目标定位就绪：相机内参 {ns}_{self.cam}_camera/camera_info，'
            f'里程计时间偏移 {self._odom_offset:.1f}s')

    # ---------------- 输入 ----------------

    def _on_odom(self, msg: Odometry):
        t = _stamp(msg.header.stamp) - self._odom_offset
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if self._hist_t and t <= self._hist_t[-1]:
            return                       # 时间戳不前进（重复/乱序）的丢掉，保证历史有序
        self._odom_frame = msg.header.frame_id or self._odom_frame
        self._hist_t.append(t)
        self._hist.append((np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])))
        while self._hist_t and self._hist_t[0] < t - self._history_s:
            self._hist_t.popleft()
            self._hist.popleft()

    def _on_range(self, msg: Range):
        r = float(msg.range)
        if not (msg.min_range <= r <= msg.max_range) or not self._hist:
            return
        t = self._hist_t[-1]
        pos, q = self._hist[-1]
        R = quat_to_rot(*q)
        # 测距仪沿机体 -Z 测；R[2,2] = cos(倾角)，把斜距换成竖直距离
        ground_z = pos[2] - (r + self._range_dz) * R[2, 2]
        self._ground.append((t, ground_z))
        while self._ground and self._ground[0][0] < t - self._ground_window:
            self._ground.popleft()

    def _on_camera_info(self, msg: CameraInfo):
        if self._K is None:
            self.get_logger().info(f'[{self.cam}] 收到相机内参 fx={msg.k[0]:.2f} fy={msg.k[4]:.2f} '
                                   f'cx={msg.k[2]:.2f} cy={msg.k[5]:.2f}')
        self._K = np.array(msg.k, dtype=float).reshape(3, 3)
        self._D = np.array(msg.d, dtype=float) if any(abs(v) > 1e-12 for v in msg.d) else None

    # ---------------- 计算 ----------------

    def _pose_at(self, t: float) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """图像时刻 t 的位姿。历史里有就插值；比最新一条还新就按速度外推。"""
        if len(self._hist_t) < 2 or t < self._hist_t[0]:
            return None
        i = bisect.bisect_left(self._hist_t, t)
        if i == 0:
            return self._hist[0]
        if i < len(self._hist_t):
            t0, t1 = self._hist_t[i - 1], self._hist_t[i]
            (p0, q0), (p1, q1) = self._hist[i - 1], self._hist[i]
            a = (t - t0) / (t1 - t0)
            return p0 + a * (p1 - p0), slerp(q0, q1, a)
        dt = t - self._hist_t[-1]
        if dt > self._max_extrap:
            return None
        # 速度补偿：用最近 ~0.1 秒的位移算速度（不用 twist 字段——各定位源
        # twist 的坐标系不统一，gt_odom_bridge 给的是世界系、DLIO 给的是机体系）
        j = bisect.bisect_left(self._hist_t, self._hist_t[-1] - 0.1)
        j = min(j, len(self._hist_t) - 2)
        v = (self._hist[-1][0] - self._hist[j][0]) / (self._hist_t[-1] - self._hist_t[j])
        return self._hist[-1][0] + v * dt, self._hist[-1][1]

    def _ray_opt(self, u: float, v: float) -> np.ndarray:
        if self._D is not None and cv2 is not None:
            pts = cv2.undistortPoints(np.array([[[u, v]]], dtype=float), self._K, self._D)
            x, y = float(pts[0, 0, 0]), float(pts[0, 0, 1])
        else:
            x = (u - self._K[0, 2]) / self._K[0, 0]
            y = (v - self._K[1, 2]) / self._K[1, 1]
        return np.array([x, y, 1.0])

    def _on_detections(self, msg: Detection2DArray):
        if self._frame_tag not in msg.header.frame_id or not msg.detections:
            return
        self._stats['det'] += 1
        if self._K is None:
            return
        t_img = _stamp(msg.header.stamp)
        if self._hist_t:
            self._last_lag = t_img - self._hist_t[-1]
        pose = self._pose_at(t_img)
        if pose is None:
            self._stats['no_pose'] += 1
            return
        if not self._ground:
            self._stats['no_ground'] += 1
            return
        ground_z = statistics.median(g for _, g in self._ground)
        pos, q = pose
        R_wb = quat_to_rot(*q)
        cam_pos = pos + R_wb @ self._t_body_cam
        R_wo = R_wb @ self._R_body_opt

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        for det in msg.detections:
            if not det.results:
                continue
            ray = R_wo @ self._ray_opt(det.bbox.center.position.x, det.bbox.center.position.y)
            hit = ray_ground_intersection(cam_pos, ray, ground_z)
            if hit is None:
                continue
            d3 = Detection3D()
            d3.header = out.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det.results[0].hypothesis.class_id
            hyp.hypothesis.score = det.results[0].hypothesis.score
            hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = (
                float(hit[0]), float(hit[1]), float(hit[2]))
            hyp.pose.pose.orientation.w = 1.0
            d3.results.append(hyp)
            d3.bbox.center.position = hyp.pose.pose.position
            d3.bbox.center.orientation.w = 1.0
            out.detections.append(d3)
        if not out.detections:
            self._stats['no_hit'] += 1
            return
        self.pub.publish(out)
        self._stats['ok'] += 1

    def _report(self):
        s = self._stats
        if s['det']:
            ground = (f'{statistics.median(g for _, g in self._ground):.2f}m'
                      if self._ground else '无')
            self.get_logger().info(
                f'[{self.cam}] 最近10秒 {s["det"]} 帧有检测，定位成功 {s["ok"]}，'
                f'缺位姿 {s["no_pose"]}，缺地面高度 {s["no_ground"]}，射线不朝地面 {s["no_hit"]}；'
                f'当前地面高度 {ground}')
        if s['no_pose'] and self._last_lag is not None and abs(self._last_lag) > 1.0:
            self.get_logger().warn(
                f'[{self.cam}] 图像时间比里程计时间{"晚" if self._last_lag > 0 else "早"} '
                f'{abs(self._last_lag):.1f} 秒，两者不在一个时间基准上——检查 '
                f'odom_stamp_offset_s（仿真 1735689600，真机 0），见文件头"时间基准"')
        if s['det'] and self._K is None:
            self.get_logger().warn(f'[{self.cam}] 有检测结果但一直没收到 camera_info，无法定位')
        for k in s:
            s[k] = 0


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocateNode()
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
