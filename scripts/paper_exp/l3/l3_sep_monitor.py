#!/usr/bin/env python3
"""现场实时监视两标签间距——用来手动找"双标签解算健康"的朝向。

背景：2026-09-03真机实测发现tag_b在多数朝向下被机体自遮挡，测距整体偏长，
解出的位置被推离锚点0.5~0.6米，两标签间距因此从物理的0.28米被放大到
0.44~0.92米，双标签yaw随之带上几十度的朝向相关偏置。只有少数朝向下间距
回到0.28米、yaw才可信。这个脚本把间距实时打出来，人一边慢慢转机头一边看，
读数稳定接近物理基线时就是健康朝向。

用法（容器内）：
    python3 /logs/l3_sep_monitor.py [--ns NX01] [--baseline 0.28] [--tol 0.07]
按 Ctrl-C 退出。
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SepMonitor(Node):
    def __init__(self, ns, baseline, tol):
        super().__init__('l3_sep_monitor')
        self.baseline, self.tol = baseline, tol
        self.a = self.b = None
        self.yaw = float('nan')
        self.buf = []
        self.create_subscription(PoseStamped, f'/{ns}/uwb_a/pose_abs',
                                 lambda m: setattr(self, 'a', m.pose.position), 10)
        self.create_subscription(PoseStamped, f'/{ns}/uwb_b/pose_abs',
                                 lambda m: setattr(self, 'b', m.pose.position), 10)
        self.create_subscription(Odometry, f'/{ns}/dlio/odom_node/odom',
                                 self._odom, 10)
        self.create_timer(0.5, self._tick)

    def _odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                           1 - 2 * (q.y * q.y + q.z * q.z)))

    def _tick(self):
        if self.a is None or self.b is None:
            print('  等待两路标签数据...', flush=True)
            return
        d = math.hypot(self.b.x - self.a.x, self.b.y - self.a.y)
        self.buf.append(d)
        self.buf = self.buf[-10:]                      # 最近5秒
        avg = sum(self.buf) / len(self.buf)
        ok = abs(avg - self.baseline) <= self.tol
        bar = '#' * min(int(avg / 0.02), 50)
        print(f'  朝向{self.yaw:+7.1f}°  间距 瞬时{d:.3f} 均值{avg:.3f} m  '
              f'{"✅健康" if ok else "❌偏大"}  |{bar}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='NX01')
    ap.add_argument('--baseline', type=float, default=0.28)
    ap.add_argument('--tol', type=float, default=0.07)
    a = ap.parse_args()
    rclpy.init()
    n = SepMonitor(a.ns, a.baseline, a.tol)
    print(f'监视 {a.ns} 两标签间距，目标 {a.baseline}±{a.tol} m。慢慢转机头，'
          f'读数进入✅并稳住就是健康朝向。Ctrl-C 退出。')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
