#!/usr/bin/env python3
"""论文实验用的标定机动——比`auto_calibration_flight.py`长得多的一条闭合路径。

为什么不直接用`auto_calibration_flight.py`：那个动作是"前进1.1米再退回"，
合计约2.2米、只能切出4段左右位移，而论文表1的滑窗是W=50段×d_min=0.5米，
**要装满滑窗至少需要25米的运动**。样本不够时θ*停在瞬时值上，测出来的不是
"收敛后的精度"，而是"只攒了4段的精度"，跟论文式(13)要验证的东西对不上。

这里改成在局部系里绕边长`SIDE_M`的正方形飞`LAPS`圈（默认3米×3圈=36米，
足够装满滑窗还有余量），并且**两机走各自局部系的不同半平面**：NX01沿
局部+y侧、NX02沿局部−y侧。两机spawn朝向差75°(30°/−45°)，这样换算到世界系
之后NX01整条路径在y≥0、NX02在y≤0，天然分开，不依赖规划器的机间避障也不会
撞上（20×20米的simple_room装得下，最远点离出生点约4.2米）。

起飞完成判定、到点判定、超时语义全部照抄`auto_calibration_flight.py`（那两
处判据是踩过真实事故改出来的，不重新发明）：起飞是硬前提，超时必须中止；
航段到点是软判据，超时打警告继续。

用法（由`run_sim_run.sh`用docker exec拉起，也可以单独手动跑）：
    python3 paper_flight.py NX01 [--side 3.0] [--laps 3]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

TAKEOFF_TARGET_HEIGHT_M = 1.0
TAKEOFF_ARRIVE_TOLERANCE_M = 0.15
TAKEOFF_STABLE_HOLD_SEC = 1.0
TAKEOFF_WAIT_TIMEOUT_SEC = 90.0
ARRIVE_TOLERANCE_M = 0.4
ARRIVE_TIMEOUT_SEC = 45.0
POLL_INTERVAL_SEC = 0.2
SETTLE_SEC = 2.0


class PaperFlight(Node):
    def __init__(self, ns, side, laps):
        super().__init__('paper_flight')
        self.ns = ns
        self.side = side
        self.laps = laps
        self.local_pos = None
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Odometry, f'/{ns}/dlio/odom_node/odom', self._odom_cb, best_effort)
        self.goal_pub = self.create_publisher(PoseStamped, f'/{ns}/term_goal', 10)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.local_pos = (p.x, p.y, p.z)

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_INTERVAL_SEC)
            if predicate():
                return True
        return False

    def _spin_until_stable(self, predicate, hold_sec, timeout_sec):
        """要求predicate连续hold_sec秒都成立——爬升途中会短暂经过目标高度，
        单次采样判定无论容差多紧都可能被抓拍到，必须连续稳定才算数。"""
        deadline = time.monotonic() + timeout_sec
        ok_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_INTERVAL_SEC)
            if predicate():
                ok_since = ok_since or time.monotonic()
                if time.monotonic() - ok_since >= hold_sec:
                    return True
            else:
                ok_since = None
        return False

    def _send_goal(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'      # 只作标注，ego_replan_fsm不读这个字段
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        for _ in range(3):               # DDS discovery没跟上时单发可能白发
            self.goal_pub.publish(msg)
            time.sleep(0.1)

    def run(self):
        if not self._spin_until(lambda: self.local_pos is not None, 30.0):
            self.get_logger().error('等不到里程计，中止')
            return 1
        if not self._spin_until_stable(
                lambda: abs(self.local_pos[2] - TAKEOFF_TARGET_HEIGHT_M)
                <= TAKEOFF_ARRIVE_TOLERANCE_M,
                TAKEOFF_STABLE_HOLD_SEC, TAKEOFF_WAIT_TIMEOUT_SEC):
            self.get_logger().error('起飞未在超时内稳定到位，中止（硬前提，不能将就往下走）')
            return 1
        x0, y0, z0 = self.local_pos
        self.get_logger().info(f'起飞到位，基准点 local=({x0:.2f},{y0:.2f},{z0:.2f})')
        time.sleep(SETTLE_SEC)

        s = self.side
        # NX01走局部+y半平面、NX02走−y半平面，换算到世界系后两机路径分开
        sign = 1.0 if self.ns.endswith('01') else -1.0
        corners = [(s, 0.0), (s, sign * s), (0.0, sign * s), (0.0, 0.0)]
        total = 0.0
        prev = (0.0, 0.0)
        for lap in range(self.laps):
            for (dx, dy) in corners:
                tx, ty = x0 + dx, y0 + dy
                self._send_goal(tx, ty, z0)
                arrived = self._spin_until(
                    lambda tx=tx, ty=ty: math.hypot(self.local_pos[0] - tx,
                                                    self.local_pos[1] - ty)
                    <= ARRIVE_TOLERANCE_M, ARRIVE_TIMEOUT_SEC)
                total += math.hypot(dx - prev[0], dy - prev[1])
                prev = (dx, dy)
                self.get_logger().info(
                    f'第{lap+1}圈 -> ({dx:+.1f},{dy:+.1f}) '
                    f'{"到达" if arrived else "超时(软继续)"}，累计路径{total:.1f}m')
        self.get_logger().info(f'机动结束，累计路径长度约{total:.1f}米')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ns')
    ap.add_argument('--side', type=float, default=3.0)
    ap.add_argument('--laps', type=int, default=3)
    a = ap.parse_args()
    rclpy.init()
    node = PaperFlight(a.ns, a.side, a.laps)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
