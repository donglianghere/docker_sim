#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合任务的"出题裁判"：控制两处火情什么时候出现、这一轮出哪一个。

用户 2026-09-30 定的规则：
  · 综合任务跑三轮——第一轮编队飞行，第二、三轮各处置一处火情；
  · 火情**随机**，但两轮不重复：第二轮出地面火，第三轮必出高层火，反之亦然；
  · 火情在**侦察机过 G 点之后**才出现（不能一开始就摆在那儿让它提前看见）。

实现方式：两处火情标识（`fire_point_marker` 地面 / `fire_apriltag_marker` 高层）
本来就是 world 里两个独立模型。开局先把它们**删掉**，盯着侦察机的位置，等它过
G 点就把本轮抽中的那个**重新生成**回原位。

为什么用删除/生成（`/delete_entity` + `/spawn_entity`），不用瞬移藏/放
（`/plug/set_entity_state` 挪到地下再挪回来）：
    **gzclient 不会重绘运行中被热改位姿的模型**（DEBUG_JOURNAL 2026-09-15 记过，
    2026-09-30 又因此误判过一次"火情在 2# 还是 1#"）。瞬移的话相机看到的是新位置、
    界面上却还停在旧位置，操作员没法相信自己看到的东西。增删模型 gzclient 是认的，
    界面所见即真值。两种都实测过，选不骗人的那个。

用法（在 sim-world 容器里跑，跟飞行同时进行）：
    python3 /opt/host_scripts/referee.py --leader NX01

只读侦察机位置 + 增删两个标识模型，不碰任何飞行逻辑，也不改 scenario_reset_node。
"""
import argparse
import json
import math
import os
import random
import re
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import DeleteEntity, SpawnEntity

#: 火情种类。名字同时用作 spawn 的实体名（跟 world 里一致，别改）。
GROUND = 'fire_point_marker'
HIGH = 'fire_apriltag_marker'

#: G 点世界坐标。侦察机进到这个半径内就算"过了 G 点"。
G_XY = (17.0, 16.0)
G_NEAR_M = 1.5
POLL_S = 0.2

#: 什么时候才允许出题——**上一轮结束之后**。
#: 一轮结束的标志是任务机**降落在自己的起降点上**。
#:
#: ⚠️ 不能只数"降落次数"：任务机每轮要落好几次——地面火情轮是"物资点取弹 +
#: 自己起降点"共 2 次，高层火情轮是"物资点取器材 + 物资点放器材 + 自己起降点"
#: 共 3 次。只数次数的话，第 2 轮任务机一在物资点落地计数就 +1、封锁当场解除，
#: 而地面火情轮侦察机通报完要**飞回 G 点**等投弹——这一下就把第 3 轮的火情
#: 提前放出来了（用户 2026-09-30 指出）。
#: 所以只认"落在自己起降点附近"这一种：起降点 (12,3) 跟物资点 (10,8) 相距
#: 5.4 米，位置上分得很清楚。
#:
#: 2026-09-30 首跑的教训：原来写的是"侦察机离 G 点够远就置位、进到 G 点就出题"，
#: 开局飞机停在起降点、离 G 有 15.8 米，当场就置位了；而**编队航线本身就两次
#: 经过 G 点**（A B C G E F G D），于是第 1 轮的两次过 G 把两处火情全放了出来，
#: 第 2 轮侦察机一飞到 G 就看见了本该第 3 轮才出现的地面火情。
LANDED_Z = 0.35          # 低于这个高度算贴地
AIRBORNE_Z = 1.0         # 高于这个算在空中
LANDED_HOLD_S = 2.0      # 贴地持续这么久才算真的降落了
SUPPLY_PAD_XY = (12.0, 3.0)   # 任务机自己的起降点
PAD_NEAR_M = 1.5              # 落点离它这么近才算"回自己起降点了"


def extract_marker_sdf(world_path):
    """从 world 文件里把两个标识的 <model> 段原样抠出来，spawn 时要用。

    不手写一份 SDF：手写的迟早跟 world 里的版本对不上（贴图路径、尺寸、
    standoff 都在里面），而 world 本身是 gen_sample_room_world.py 生成的。
    """
    s = open(world_path, encoding='utf-8').read()
    out = {}
    for name in (GROUND, HIGH):
        i = s.index(f"<model name='{name}'>")
        depth, j = 0, i
        while True:
            m = re.compile(r'</?model\b').search(s, j)
            depth += 1 if s[m.start():m.start() + 7] == '<model ' else -1
            j = m.end()
            if depth == 0:
                break
        j = s.index('>', j) + 1
        out[name] = f"<sdf version='1.6'>{s[i:j]}</sdf>"
    return out


class Referee(Node):
    def __init__(self, leader, follower, sdf, order):
        super().__init__('mission_referee')
        self.sdf = sdf
        self.order = order          # 本次的出题顺序，比如 [GROUND, HIGH]
        self.round_idx = 0          # 已经出到第几个了
        self.pos = None
        # 任务机的起降状态：见 LANDED_Z 那段——它降落一次就代表一轮结束
        self._sup_airborne_seen = False
        self._sup_low_since = None
        self.sup_landings = 0
        self.create_subscription(PoseStamped, f'/{leader}/uwb/pose_abs',
                                 self._on_pose, 10)
        self.create_subscription(PoseStamped, f'/{follower}/uwb/pose_abs',
                                 self._on_sup_pose, 10)
        self.del_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

    def _on_pose(self, msg):
        p = msg.pose.position
        self.pos = (p.x, p.y)

    def _on_sup_pose(self, msg):
        """数任务机降落了几次。每降落一次 = 又跑完一轮。"""
        z = msg.pose.position.z
        if z > AIRBORNE_Z:
            self._sup_airborne_seen = True
            self._sup_low_since = None
            return
        if z >= LANDED_Z or not self._sup_airborne_seen:
            return
        # 只认落在自己起降点上的那次——中途去物资点取放器材也会落地，
        # 但那不代表一轮结束，见 SUPPLY_PAD_XY 上方的说明。
        p = msg.pose.position
        if math.hypot(p.x - SUPPLY_PAD_XY[0], p.y - SUPPLY_PAD_XY[1]) > PAD_NEAR_M:
            return
        now = time.monotonic()
        if self._sup_low_since is None:
            self._sup_low_since = now
        elif now - self._sup_low_since >= LANDED_HOLD_S:
            self.sup_landings += 1
            self._sup_airborne_seen = False      # 下次再起飞再落才算下一轮
            self._sup_low_since = None
            print(f'[裁判] 任务机第 {self.sup_landings} 次回到自己起降点 = 第 '
                  f'{self.sup_landings} 轮结束', flush=True)

    def _call(self, cli, req, what):
        if not cli.wait_for_service(timeout_sec=10.0):
            print(f'[裁判] {what}：服务不可用', flush=True)
            return False
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        r = fut.result()
        ok = bool(r and r.success)
        print(f'[裁判] {what}：{"成功" if ok else "失败 " + (r.status_message if r else "无响应")}',
              flush=True)
        return ok

    def remove(self, name):
        req = DeleteEntity.Request()
        req.name = name
        return self._call(self.del_cli, req, f'移除 {name}')

    def place(self, name):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = self.sdf[name]
        return self._call(self.spawn_cli, req, f'放置 {name}')

    def dist_to_g(self):
        if self.pos is None:
            return None
        return math.hypot(self.pos[0] - G_XY[0], self.pos[1] - G_XY[1])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--leader', default='NX01')
    ap.add_argument('--follower', default='NX02')
    ap.add_argument('--world', default='/opt/mighty_ws/install/mighty/share/mighty/worlds/sample_room.world')
    ap.add_argument('--seed', type=int, default=None, help='复现用；不给就每次都随机')
    ap.add_argument('--secs', type=float, default=2400.0)
    ap.add_argument('--truth-out', default='/logs/mission_truth.json',
                    help='把出题顺序写到这里，测试脚本可以读它比对')
    args = ap.parse_args()

    if not os.path.exists(args.world):
        print(f'[裁判] 找不到 world 文件 {args.world}', file=sys.stderr)
        return 1
    sdf = extract_marker_sdf(args.world)

    rng = random.Random(args.seed) if args.seed is not None else random
    # 两轮不重复：把 [地面, 高层] 洗牌，第二轮用第一个、第三轮用第二个
    order = [GROUND, HIGH]
    rng.shuffle(order)
    label = {GROUND: '地面火情', HIGH: '高层火情'}
    print(f'[裁判] 本次出题顺序：第2轮={label[order[0]]}，第3轮={label[order[1]]}', flush=True)

    rclpy.init()
    node = Referee(args.leader, args.follower, sdf, order)

    # 开局先把两处火情都撤掉——侦察机第一轮编队飞行会经过 G/E 一带，
    # 这时候不能让它看见任何火情。
    for name in (GROUND, HIGH):
        node.remove(name)

    try:
        with open(args.truth_out, 'w', encoding='utf-8') as f:
            json.dump({'round2': order[0], 'round3': order[1],
                       'label': {k: label[v] for k, v in
                                 (('round2', order[0]), ('round3', order[1]))}},
                      f, ensure_ascii=False, indent=2)
        print(f'[裁判] 出题顺序已写入 {args.truth_out}', flush=True)
    except Exception as exc:
        print(f'[裁判] 出题顺序写不进 {args.truth_out}（{exc}），只在日志里留痕', flush=True)

    print(f'[裁判] 盯 {args.leader} 过 G 点 {G_XY}…', flush=True)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.secs:
            rclpy.spin_once(node, timeout_sec=0.05)
            d = node.dist_to_g()
            if d is None:
                continue
            if node.round_idx >= len(order):
                continue
            # 出题资格：任务机已经降落过的轮数，必须多于已经出过的题数。
            # 第 1 轮编队跑完任务机降落 -> sup_landings=1 > round_idx=0 -> 允许，
            # 于是第 2 轮飞到 G 才放第一处火情。编队航线里那两次过 G 都发生在
            # sup_landings 还是 0 的时候，不会误触发。
            if node.sup_landings <= node.round_idx:
                continue
            if d <= G_NEAR_M:
                name = order[node.round_idx]
                node.round_idx += 1
                print(f'[裁判] 侦察机已到 G 点（离 {d:.2f} m），'
                      f'放出第 {node.round_idx + 1} 轮的{label[name]}', flush=True)
                node.place(name)
    finally:
        print(f'[裁判] 结束，共出题 {node.round_idx} 次', flush=True)
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
