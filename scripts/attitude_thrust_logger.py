#!/usr/bin/env python3
"""持续记录PX4自己上报的姿态/推力目标（mavros/setpoint_raw/target_attitude），
写到一个跟mighty_debug.log/WALLHIT同一路数的持久化日志文件，而不是只在
终端里一闪而过——这样下次再掉高度/炸机，能完整拿到崩溃前后的姿态/推力
序列，而不是只有崩溃后现场查到的一个孤立瞬间快照。

背景：现场用ros2 topic echo抓到过一次type_mask=0、thrust=1.0（推力打满）、
orientation四元数换算出来的旋转角接近180度（w分量接近0）的瞬间快照——
比"悬停推力不够"这种平缓掉高度严重得多，看起来更像是姿态控制本身在
崩溃过程中出现了大幅度、甚至接近倒扣的异常指令，但只有一个孤立样本，
不知道是不是转瞬即逝的数值抖动还是真的有一段持续的失控过程。这个脚本
把target_attitude（PX4自己算出来打算怎么飞）和local_position/pose（飞机
实际当前姿态，来自PX4自己的EKF融合估计）都记下来，两者对照才能分清楚
"PX4想让飞机做什么"和"飞机实际在做什么"是不是对得上。

用法（在flight-stack容器内部跑，namespace从参数读）：
  python3 attitude_thrust_logger.py NX01
输出：追加写到 /logs/<namespace>/attitude_thrust_debug.log（`/logs`是
docker-compose挂载到宿主机`runtime_logs/`的volume，容器销毁/重建数据
不会丢，参见README"运行时数据记录"一节）；如果`/logs`这个挂载点不存在
（比如手动在容器外单独调试这个脚本），退回旧行为写到`/tmp`，不报错。
每行前面带秒级时间戳，跟mighty_debug.log的debug_log_t()格式风格保持
一致，方便对照着看。
"""

import os
import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped


def log_path_for(ns: str) -> str:
    logs_dir = f'/logs/{ns}'
    if os.path.isdir('/logs'):
        os.makedirs(logs_dir, exist_ok=True)
        return f'{logs_dir}/attitude_thrust_debug.log'
    return '/tmp/attitude_thrust_debug.log'


def quat_rotation_angle_deg(x, y, z, w):
    """四元数代表的旋转角度（绝对值，0~180度）——不区分具体转轴方向，
    只看"离水平/初始姿态偏了多少"，用来快速判断是不是出现了大幅度姿态
    异常（比如接近180度=接近倒扣）。"""
    w = max(-1.0, min(1.0, w))
    return math.degrees(2 * math.acos(abs(w)))


def quat_to_euler_deg(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class AttitudeThrustLogger(Node):
    def __init__(self, ns: str):
        super().__init__(f'{ns.lower()}_attitude_thrust_logger')
        self.ns = ns
        self.t0 = time.monotonic()
        self.log_file = open(log_path_for(ns), 'a', buffering=1)  # line-buffered

        self.latest_actual = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            AttitudeTarget, f'/{ns}/mavros/setpoint_raw/target_attitude',
            self._target_cb, qos)
        self.create_subscription(
            PoseStamped, f'/{ns}/mavros/local_position/pose',
            self._actual_cb, qos)

        self.log_file.write(
            f"# {ns} attitude_thrust_logger started, t0(monotonic)={self.t0}\n")

    def _t(self):
        return time.monotonic() - self.t0

    def _actual_cb(self, msg: PoseStamped):
        self.latest_actual = msg

    def _target_cb(self, msg: AttitudeTarget):
        q = msg.orientation
        tgt_angle = quat_rotation_angle_deg(q.x, q.y, q.z, q.w)
        tgt_r, tgt_p, tgt_y = quat_to_euler_deg(q.x, q.y, q.z, q.w)

        actual_str = "actual=NONE"
        if self.latest_actual is not None:
            ap = self.latest_actual.pose.position
            aq = self.latest_actual.pose.orientation
            a_angle = quat_rotation_angle_deg(aq.x, aq.y, aq.z, aq.w)
            a_r, a_p, a_y = quat_to_euler_deg(aq.x, aq.y, aq.z, aq.w)
            actual_str = (
                f"actual_pos=({ap.x:+.2f},{ap.y:+.2f},{ap.z:+.2f}) "
                f"actual_rpy=({a_r:+.1f},{a_p:+.1f},{a_y:+.1f}) "
                f"actual_tilt_from_level={a_angle:.1f}deg"
            )

        line = (
            f"{self._t():.3f} [{self.ns}] TARGET_ATTITUDE "
            f"thrust={msg.thrust:.3f} "
            f"body_rate=({msg.body_rate.x:+.2f},{msg.body_rate.y:+.2f},{msg.body_rate.z:+.2f}) "
            f"target_rpy=({tgt_r:+.1f},{tgt_p:+.1f},{tgt_y:+.1f}) "
            f"target_tilt_from_level={tgt_angle:.1f}deg "
            f"{actual_str}\n"
        )
        self.log_file.write(line)


def main():
    if len(sys.argv) < 2:
        print("用法: attitude_thrust_logger.py <NAMESPACE, 如 NX01>", file=sys.stderr)
        sys.exit(1)
    ns = sys.argv[1]

    rclpy.init()
    node = AttitudeThrustLogger(ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
