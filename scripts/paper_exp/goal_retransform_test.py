#!/usr/bin/env python3
"""E13：论文3.2节"目标点一次性变换 vs 持续重变换"的对照实验。

复现的场景就是论文3.2节描述的那次事故：操作员在**世界系**里指定一个目标点，
系统在收到的那一刻用当时的(θ*,t)把它换算成局部坐标发给规划器。刚起飞时飞机
还没怎么动过，θ*样本极少、离真值很远（本实验里两机真值分别是30°和−45°，而
θ*初值是0），这一次性换算因此是错的；之后θ*被在线标定修正，但已经发出去的
目标点不会被追溯修正——飞机最终停在一个偏转了真值角度的错误位置上。

测法：
  1. 起飞后**立刻**（θ*还没收敛时）往`/{ns}/rviz_goal_world`发一个世界系目标点，
     走的是`rviz_goal_bridge_node`这条真实消费链路（不是直接发term_goal，
     那样就绕开了被测对象）。
  2. 等飞机稳定后，从`/{ns}/uwb/pose_truth`（Gazebo真值旁路）读它实际停在哪。
  3. 报告"实际落点 − 指定的世界目标点"的水平误差。
  4. `GOAL_RETRANSFORM=false`起容器 = 修复前；`true` = 修复后。同一个脚本、
     同一个目标点，两次的落点误差直接对比。

理论预期（修复前）：落点≈把目标点绕当前机体位置旋转了−θ_true的位置，误差量级
= 2·|目标点到出生点距离|·sin(θ_true/2)。

用法： python3 goal_retransform_test.py NX01 --wx 2.0 --wy 3.0 [--wait 90]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

TAKEOFF_TARGET_HEIGHT_M = 1.0
TAKEOFF_ARRIVE_TOLERANCE_M = 0.15
TAKEOFF_STABLE_HOLD_SEC = 1.0
TAKEOFF_WAIT_TIMEOUT_SEC = 90.0
POLL = 0.2


class GoalRetransformTest(Node):
    def __init__(self, ns):
        super().__init__('goal_retransform_test')
        self.ns = ns
        self.truth = None
        self.local = None
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, f'/{ns}/uwb/pose_truth',
                                 self._truth_cb, 10)
        self.create_subscription(Odometry, f'/{ns}/dlio/odom_node/odom',
                                 self._odom_cb, be)
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{ns}/rviz_goal_world', 10)

    def _truth_cb(self, m):
        self.truth = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def _odom_cb(self, m):
        p = m.pose.pose.position
        self.local = (p.x, p.y, p.z)

    def _spin(self, pred, timeout):
        t_end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=POLL)
            if pred():
                return True
        return False

    def _spin_stable(self, pred, hold, timeout):
        t_end = time.monotonic() + timeout
        since = None
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=POLL)
            if pred():
                since = since or time.monotonic()
                if time.monotonic() - since >= hold:
                    return True
            else:
                since = None
        return False

    def run(self, wx, wy, wait_sec):
        if not self._spin(lambda: self.local is not None and self.truth is not None, 30.0):
            self.get_logger().error('等不到里程计或真值，中止')
            return 1
        if not self._spin_stable(
                lambda: abs(self.local[2] - TAKEOFF_TARGET_HEIGHT_M)
                <= TAKEOFF_ARRIVE_TOLERANCE_M,
                TAKEOFF_STABLE_HOLD_SEC, TAKEOFF_WAIT_TIMEOUT_SEC):
            self.get_logger().error('起飞未在超时内到位，中止')
            return 1
        start_truth = self.truth
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = wx
        msg.pose.position.y = wy
        msg.pose.position.z = self.local[2]
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info(
            f'已发世界系目标点({wx:.2f},{wy:.2f})，起飞点真值='
            f'({start_truth[0]:.2f},{start_truth[1]:.2f})，等待{wait_sec}秒')
        t_end = time.monotonic() + wait_sec
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=POLL)
        ex, ey = self.truth[0] - wx, self.truth[1] - wy
        print(f'RESULT {self.ns} goal=({wx:.3f},{wy:.3f}) '
              f'final_truth=({self.truth[0]:.3f},{self.truth[1]:.3f}) '
              f'err_x={ex:+.3f} err_y={ey:+.3f} err={math.hypot(ex, ey):.3f}',
              flush=True)
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ns')
    ap.add_argument('--wx', type=float, required=True)
    ap.add_argument('--wy', type=float, required=True)
    ap.add_argument('--wait', type=float, default=90.0)
    a = ap.parse_args()
    rclpy.init()
    n = GoalRetransformTest(a.ns)
    try:
        rc = n.run(a.wx, a.wy, a.wait)
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
