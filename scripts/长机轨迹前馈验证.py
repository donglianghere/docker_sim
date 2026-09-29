#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读验证：长机广播的规划轨迹（planning/bspline）能不能当僚机的前馈信息用。

背景：现在僚机跟的是长机**已经飞过的位置**（uwb/pose_abs），只能事后知道长机
干了什么——本会话一路在补的速度前馈、偏置积分、长机照顾模式，都是在补这一个
滞后。而长机的 ego_planner 本来就在发 `planning/bspline`，那是它**未来几秒**
要飞的 B 样条（含速度剖面），僚机完全没用。

这个脚本不改任何飞行逻辑，只回答一个问题：**拿这条轨迹预测长机 Δ 秒后的位置，
到底准不准？** 并且跟"假设长机匀速直线外推"这个朴素基线比——如果赢不过基线，
这条路就不值得走。

用法（在 flight-stack 容器里跑，跟飞行同时进行）：
    python3 /opt/host_scripts/长机轨迹前馈验证.py --leader NX01 --secs 180
"""
import argparse
import bisect
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from traj_utils.msg import Bspline


def de_boor(knots, ctrl, degree, u):
    """标准 de Boor 求值。ctrl 是 [(x,y,z), ...]，返回 (x,y,z)。

    ego_planner 的 UniformBspline.evaluateDeBoorT(t) 等价于 evaluateDeBoor(t + u_[p])，
    所以调用方传进来的 u 已经加过 knots[degree] 了。
    """
    n = len(ctrl) - 1
    lo, hi = knots[degree], knots[n + 1]
    u = min(max(u, lo), hi)
    k = bisect.bisect_right(knots, u) - 1
    k = min(max(k, degree), n)
    d = [list(ctrl[j + k - degree]) for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = j + k - degree
            den = knots[i + degree + 1 - r] - knots[i]
            a = 0.0 if den <= 1e-12 else (u - knots[i]) / den
            for c in range(3):
                d[j][c] = (1.0 - a) * d[j - 1][c] + a * d[j][c]
    return d[degree]


class Check(Node):
    def __init__(self, leader, horizons):
        super().__init__('leader_traj_ff_check')
        self.horizons = horizons
        self.traj = None            # (start_sec, order, knots, ctrl)
        self.odom = []              # [(t, x, y)] 按时间递增
        self.pred = {h: [] for h in horizons}   # 样条预测 [(target_t, x, y)]
        self.base = {h: [] for h in horizons}   # 匀速外推基线
        self.n_traj = 0
        self.create_subscription(Bspline, f'/{leader}/planning/bspline', self._on_traj, 10)
        self.create_subscription(Odometry, f'/{leader}/dlio/odom_node/odom',
                                 self._on_odom, 20)
        # 用 odom 系比对：bspline 就发在长机自己的 odom 系里，这样绕开坐标变换，
        # 单纯考察"预测准不准"这一件事。真要上线还要处理 odom->UWB 的变换。

    def _on_traj(self, msg):
        t0 = msg.start_time.sec + msg.start_time.nanosec * 1e-9
        self.traj = (t0, msg.order, list(msg.knots),
                     [(p.x, p.y, p.z) for p in msg.pos_pts])
        self.n_traj += 1

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.odom.append((t, p.x, p.y))
        if self.traj is None:
            return
        t0, order, knots, ctrl = self.traj
        if len(ctrl) <= order or len(knots) <= len(ctrl):
            return
        for h in self.horizons:
            # ① 样条预测：把"从轨迹起点算起的时间"喂给 de Boor
            try:
                q = de_boor(knots, ctrl, order, (t + h - t0) + knots[order])
            except Exception:
                continue
            self.pred[h].append((t + h, q[0], q[1]))
            # ② 基线：假设长机保持当前速度直线飞
            self.base[h].append((t + h, p.x + v.x * h, p.y + v.y * h))

    def report(self):
        ts = [q[0] for q in self.odom]
        print(f'\n收到轨迹 {self.n_traj} 条，里程计 {len(self.odom)} 帧')
        if len(self.odom) < 50 or self.n_traj == 0:
            print('样本太少，没法下结论'); return
        print(f'{"预测提前量":>10} {"样条误差 中位/90%":>22} {"匀速外推 中位/90%":>22}   结论')
        for h in self.horizons:
            rows = []
            for name, src in (('spline', self.pred[h]), ('base', self.base[h])):
                errs = []
                for tt, px, py in src:
                    i = bisect.bisect_left(ts, tt)
                    if i <= 0 or i >= len(ts):
                        continue
                    if abs(ts[i] - tt) > 0.15:
                        continue
                    errs.append(math.hypot(self.odom[i][1] - px, self.odom[i][2] - py))
                errs.sort()
                rows.append((errs[len(errs) // 2], errs[int(len(errs) * 0.9)], len(errs))
                            if len(errs) > 20 else None)
            if not all(rows):
                print(f'{h:>9.1f}s  样本不足'); continue
            (sm, s9, n), (bm, b9, _) = rows
            verdict = '样条更准' if sm < bm * 0.8 else ('差不多' if sm < bm * 1.2 else '基线更准')
            print(f'{h:>9.1f}s  {sm:>8.2f} / {s9:>8.2f} m  {bm:>8.2f} / {b9:>8.2f} m   '
                  f'{verdict}（n={n}）')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--leader', default='NX01')
    ap.add_argument('--secs', type=float, default=180.0)
    ap.add_argument('--horizons', default='0.5,1.0,2.0')
    args = ap.parse_args()
    hs = [float(v) for v in args.horizons.split(',')]
    rclpy.init()
    node = Check(args.leader, hs)
    print(f'[前馈验证] 订阅 /{args.leader}/planning/bspline 与 odom，测 {args.secs:.0f} 秒…',
          flush=True)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.secs:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.report()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
