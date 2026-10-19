#!/usr/bin/env python3
"""僚机跟随实时探针（2026-10-10）。

要回答的唯一问题：**编队节点发给飞控的那个 xy，到底是不是"原地"**。

现有 diag 已经排除了上游所有环节：参考点 s_cmd 正常推进（全程精确等于
total_len - fd_eff，说明顶在 s_limit_gap 上）、前馈 v_ff≈0.26 m/s（由 s_cmd
推进速率反算）、offset/投影正确（s_proj=0 与"僚机没动"一致）、sep_hold 只
触发 5 秒、_yaw_turning 被 yaw_follow_leg=False 门控挡住。而僚机的 z 被正确
执行（高度稳在 1.95/2.00），z 和 xy 在同一条 PositionCommand 里，所以 xy
"没被执行"只能是它的值本身就等于别动。

但 cx,cy / sx,sy 是 diag 里唯一没落盘的量，所以直接订话题看。

判据（看 d_cmd 这一列）：
  · d_cmd ≈ max_lead_m(1.5) → 指令是对的（吊在飞机前方 1.5 米），飞机不跟
                              → 问题在 pt4ctrl/PX4 执行层
  · d_cmd ≈ 0               → 指令本身就是原地
                              → 问题在 point_at_arc_length 或 offset

坐标系：formation_cmd 的 position 是**odom 系**（cx = target + offset 换算过），
所以必须跟 dlio/odom_node/odom 比，不能跟 uwb/pose_abs 比（那是 UWB 绝对系）。

用法（飞行中，地面站另开一个终端）：
    docker exec real_monitor python3 -u /scripts/follow_probe.py
    docker exec real_monitor python3 -u /scripts/follow_probe.py --ns NX01   # 看长机
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from quadrotor_msgs.msg import PositionCommand


# BEST_EFFORT 订阅能收 RELIABLE 发布者，反过来不行。这些话题的发布端
# QoS 各不相同（uwb/pose_abs 是 RELIABLE、odom 可能是 SensorData），
# 统一用 BEST_EFFORT 最大兼容，免得某一条静默收不到。
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10)


class Probe(Node):
    def __init__(self, ns: str, out: str, period: float):
        super().__init__('follow_probe')
        self.ns = ns
        self.t0 = time.monotonic()
        self.cmd = None          # (x, y, z, vx, vy, yaw)
        self.cmd_stamp = None
        self.odom = None         # (x, y, z)
        self.odom_prev = None
        self.odom_v = 0.0
        self.uwb = None
        self.uwb_prev = None
        self.uwb_jump = 0.0
        self.diag = None
        # diag 话题每个控制周期发一次，所以"离上次收到多久"就是机载控制循环的
        # 心跳。2026-10-10：第 3 轮 start_formation_follow 的参数服务卡了 10 秒
        # 超时，怀疑节点被别的事占满；这一列能直接看出控制循环有没有停摆。
        self.diag_stamp = None
        self.diag_gap_max = 0.0

        self.create_subscription(PositionCommand, f'/{ns}/formation_cmd',
                                 self._on_cmd, QOS)
        self.create_subscription(Odometry, f'/{ns}/dlio/odom_node/odom',
                                 self._on_odom, QOS)
        self.create_subscription(PoseStamped, f'/{ns}/uwb/pose_abs',
                                 self._on_uwb, QOS)
        self.create_subscription(Float64MultiArray, f'/{ns}/formation_diag',
                                 self._on_diag, QOS)

        self.f = None
        try:
            self.f = open(out, 'w', encoding='utf-8')
            self.f.write('t,cmd_x,cmd_y,cmd_z,cmd_vx,cmd_vy,cmd_v,'
                         'odom_x,odom_y,odom_z,odom_v,d_cmd,'
                         's_cmd,s_proj,total,v_ff,sep_line,uwb_jump,diag_hb_max\n')
        except Exception as exc:
            print(f'!! CSV 打不开（{exc}），只打屏', flush=True)

        self.create_timer(period, self._tick)
        print(f'[probe] 订阅 /{ns}/ 的 formation_cmd、dlio odom、uwb/pose_abs、'
              f'formation_diag；每 {period:.1f}s 一行', flush=True)
        print('[probe] 看 d_cmd 这列：≈1.5 = 指令对、飞机不跟（执行层）；'
              '≈0 = 指令本身原地（参考点换算）', flush=True)
        print('', flush=True)
        print('    t   cmd(x,y,z)              cmd_v   odom(x,y,z)             '
              'odom_v  d_cmd | s_cmd  s_proj  total   v_ff  sep  uwb跳 心跳', flush=True)

    # ---- 回调 ----
    def _on_cmd(self, m):
        p, v = m.position, m.velocity
        self.cmd = (p.x, p.y, p.z, v.x, v.y, getattr(m, 'yaw', 0.0))
        self.cmd_stamp = time.monotonic()

    def _on_odom(self, m):
        p = m.pose.pose.position
        cur = (p.x, p.y, p.z)
        if self.odom_prev is not None:
            d = math.dist(cur[:2], self.odom_prev[:2])
            # 低通一下，odom 本身有抖动
            self.odom_v = 0.7 * self.odom_v + 0.3 * (d / 0.1)
        self.odom_prev = cur
        self.odom = cur

    def _on_uwb(self, m):
        p = m.pose.position
        cur = (p.x, p.y, p.z)
        if self.uwb_prev is not None:
            self.uwb_jump = max(self.uwb_jump,
                                math.dist(cur[:2], self.uwb_prev[:2]))
        self.uwb_prev = cur
        self.uwb = cur

    def _on_diag(self, m):
        now = time.monotonic()
        if self.diag_stamp is not None:
            self.diag_gap_max = max(self.diag_gap_max, now - self.diag_stamp)
        self.diag_stamp = now
        self.diag = list(m.data)

    # ---- 每拍输出 ----
    def _tick(self):
        t = time.monotonic() - self.t0
        g = lambda i: (self.diag[i] if self.diag and len(self.diag) > i
                       else float('nan'))
        s_cmd, s_proj, total = g(1), g(2), g(3)
        v_ff, sep = g(6), g(8)

        if self.cmd is None:
            print(f'{t:6.1f}   （还没收到 formation_cmd：编队没开始，或话题/域不对）',
                  flush=True)
            return
        cx, cy, cz, vx, vy, _ = self.cmd
        cmd_v = math.hypot(vx, vy)
        age = time.monotonic() - (self.cmd_stamp or 0.0)

        if self.odom is None:
            print(f'{t:6.1f}   cmd=({cx:7.2f},{cy:7.2f},{cz:5.2f})  '
                  f'v={cmd_v:4.2f}   （还没收到 odom）', flush=True)
            return
        ox, oy, oz = self.odom
        d_cmd = math.hypot(cx - ox, cy - oy)
        flag = '' if age < 0.5 else f'  ⚠cmd已停发{age:.1f}s'
        # 机载控制循环心跳：diag 本拍内最大间隔。正常 <0.2s；>1s 说明节点卡住，
        # 那时参数服务/action 也会跟着超时（10-10 第 3 轮就是这样）。
        d_age = (time.monotonic() - self.diag_stamp) if self.diag_stamp else float('inf')
        hb = f'{self.diag_gap_max:4.2f}'
        if self.diag_gap_max > 1.0 or d_age > 1.0:
            hb += '⚠'

        print(f'{t:6.1f}   ({cx:7.2f},{cy:7.2f},{cz:5.2f})  {cmd_v:5.2f}   '
              f'({ox:7.2f},{oy:7.2f},{oz:5.2f})  {self.odom_v:5.2f}  '
              f'{d_cmd:6.2f} | {s_cmd:6.2f} {s_proj:6.2f} {total:6.2f} '
              f'{v_ff:5.2f} {sep:5.1f} {self.uwb_jump:5.2f} {hb}{flag}', flush=True)

        if self.f:
            try:
                self.f.write(
                    f'{t:.3f},{cx:.4f},{cy:.4f},{cz:.4f},{vx:.4f},{vy:.4f},'
                    f'{cmd_v:.4f},{ox:.4f},{oy:.4f},{oz:.4f},{self.odom_v:.4f},'
                    f'{d_cmd:.4f},{s_cmd:.4f},{s_proj:.4f},{total:.4f},'
                    f'{v_ff:.4f},{sep:.4f},{self.uwb_jump:.4f},'
                    f'{self.diag_gap_max:.4f}\n')
                self.f.flush()
            except Exception:
                pass
        self.uwb_jump = 0.0      # 每拍重置，这样这列是"本拍内最大单步跳变"
        self.diag_gap_max = 0.0  # 同理，心跳列是"本拍内最大 diag 间隔"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ns', default='NX02', help='看哪架（默认僚机 NX02）')
    ap.add_argument('--out', default='', help='CSV 路径，默认 /logs/follow_probe_<ns>.csv')
    ap.add_argument('--period', type=float, default=0.5, help='打印周期（秒）')
    args = ap.parse_args()
    out = args.out or f'/logs/follow_probe_{args.ns}.csv'

    rclpy.init()
    node = Probe(args.ns, out, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.f:
            try:
                node.f.close()
                print(f'\n[probe] 数据已存 {out}', flush=True)
            except Exception:
                pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
