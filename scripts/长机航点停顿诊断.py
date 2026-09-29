#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断长机在每个航点停多久、这段时间到底在等什么。

背景：实测长机在每个航点速度掉到 0.02~0.19 m/s、持续 2.9~8.4 秒。而
max_vel=1.0 / max_acc=3.0，从 1 m/s 刹停再起步理论上不到 1 秒——所以这
3~8 秒**不是刹车**，是在等什么。之前两次靠读代码猜（终点刹车距离、
traj_server 的 yaw_hold）都猜错了，所以这次直接量。

把每次停顿拆成三段：
    刹停 → [等新轨迹] → 新轨迹到达 → [起步加速] → 恢复巡航
看哪一段占大头：
  - "等新轨迹"占大头 → 瓶颈在"判到点→下发新目标→规划器出新轨迹"这条链路；
  - "起步加速"占大头 → 瓶颈在加速度/速度剖面本身；
  - 两段都不占（停顿期间新轨迹一直在来）→ 飞机收到了轨迹却不动，问题在
    轨迹内容或下游控制器。

只订阅，不发布任何指令。
用法（flight-stack 容器里，跟飞行同时跑）：
    python3 /opt/host_scripts/长机航点停顿诊断.py --leader NX01 --secs 220
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

SLOW = 0.25      # 低于这个算"停住"
MOVING = 0.45    # 高于这个算"重新跑起来了"
MIN_DWELL = 1.0  # 短于这个不算停顿


class Diag(Node):
    def __init__(self, leader):
        super().__init__('leader_dwell_diag')
        self.samples = []       # [(t, x, y, speed)]
        self.traj_t = []        # 新轨迹到达时刻
        self.create_subscription(Odometry, f'/{leader}/dlio/odom_node/odom',
                                 self._on_odom, 20)
        try:
            from traj_utils.msg import Bspline
            self.create_subscription(Bspline, f'/{leader}/planning/bspline',
                                     self._on_traj, 20)
        except Exception as exc:
            print(f'[诊断] 订不上 bspline（{exc}），只能量停顿时长', flush=True)

    def _now(self):
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def _on_traj(self, _msg):
        self.traj_t.append(self._now())

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.samples.append((self._now(), p.x, p.y, math.hypot(v.x, v.y)))

    def report(self):
        S = self.samples
        print(f'\n里程计 {len(S)} 帧，新轨迹 {len(self.traj_t)} 条')
        if len(S) < 100:
            print('样本太少'); return
        # 找停顿段：speed < SLOW 连续 >= MIN_DWELL
        dwells, cur = [], None
        for t, x, y, sp in S:
            if sp < SLOW:
                cur = [t, t, x, y] if cur is None else [cur[0], t, cur[2], cur[3]]
            elif cur is not None:
                if cur[1] - cur[0] >= MIN_DWELL:
                    dwells.append(cur)
                cur = None
        print(f'\n停顿段（速度<{SLOW} 持续≥{MIN_DWELL}s）共 {len(dwells)} 处：')
        print(f'{"停住时刻":>9} {"时长":>6} {"位置":>16} {"期间新轨迹":>10} '
              f'{"最后一条轨迹->起步":>16} {"起步->巡航":>10}')
        for t0, t1, x, y in dwells:
            n_traj = sum(1 for tt in self.traj_t if t0 <= tt <= t1)
            last_before_move = max([tt for tt in self.traj_t if tt <= t1], default=None)
            wait = (t1 - last_before_move) if last_before_move else float('nan')
            # 起步到恢复巡航
            accel = float('nan')
            for t, _x, _y, sp in S:
                if t > t1 and sp >= MOVING:
                    accel = t - t1
                    break
            print(f'{t0 % 10000:>9.1f} {t1 - t0:>6.1f}s ({x:>6.2f},{y:>6.2f}) '
                  f'{n_traj:>10d} {wait:>14.2f}s {accel:>9.2f}s')
        if dwells:
            tot = sum(b - a for a, b, _x, _y in dwells)
            print(f'\n停顿合计 {tot:.1f} 秒 / 全程 {S[-1][0] - S[0][0]:.0f} 秒'
                  f'（占 {100 * tot / (S[-1][0] - S[0][0]):.0f}%）')
            print('判读：期间新轨迹条数多 = 飞机拿着轨迹却不动，问题在轨迹内容或下游控制器；'
                  '\n      条数少且"最后一条->起步"长 = 卡在"判到点→下发目标→出新轨迹"这条链路。')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--leader', default='NX01')
    ap.add_argument('--secs', type=float, default=220.0)
    args = ap.parse_args()
    rclpy.init()
    node = Diag(args.leader)
    print(f'[诊断] 盯 {args.leader} 的停顿，{args.secs:.0f} 秒…', flush=True)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.secs:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.report()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
