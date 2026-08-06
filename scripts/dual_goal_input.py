#!/usr/bin/env python3
"""双机目标点手动输入面板——世界坐标输入一次，自动换算成NX01/NX02
各自的局部坐标，同时发给两架飞机的/term_goal。支持两种输入：
三个数"x y z"当同一个目标点同时发给双机；六个数
"x1 y1 z1 x2 y2 z2"当两个不同目标点，前三个给NX01、后三个给NX02。

背景：RViz的"2D Goal Pose"工具一次只能点一个点、发给一架飞机（见
mighty_rviz_nx02_setgoal.patch），要同时给双机下目标点得点两次、
自己心算INIT_X偏移，容易出错；命令行现场拼ros2 topic pub也一样
要手动换算，还容易踩"--once在DDS发现完成前就退出、消息丢了"这个坑
（这次session里已经踩过好几次）。这个脚本常驻一个rclpy节点、常驻
publisher，不是每次现拼一次性命令，从根上避开这个坑。

用法（在能同时访问NX01/NX02话题的容器里跑，比如flight-stack-nx01，
ROS2话题同一个DDS domain下跨容器可见）：
  python3 dual_goal_input.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

# 每架飞机local map原点相对world的偏移，只在x方向——跟本session里
# docker-compose.yml/AGENT_INDEX*3的约定一致，NX01=3, NX02=6。
INIT_X = {
    'NX01': 3.0,
    'NX02': 6.0,
}

BOX_WIDTH = 66


def display_width(s):
    """中文/全角字符在终端里占两列，len()只算一个字符——按East Asian
    Width分类估算实际显示宽度，box画线对齐要用这个而不是len()。"""
    import unicodedata
    width = 0
    for ch in s:
        width += 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1
    return width


def box_line(text=''):
    pad = max(0, (BOX_WIDTH - 4) - display_width(text))
    return '│ ' + text + ' ' * pad + ' │'


def box_top():
    return '┌' + '─' * (BOX_WIDTH - 2) + '┐'


def box_bottom():
    return '└' + '─' * (BOX_WIDTH - 2) + '┘'


def box_sep():
    return '├' + '─' * (BOX_WIDTH - 2) + '┤'


class DualGoalInput(Node):
    def __init__(self):
        super().__init__('dual_goal_input')
        self.pubs = {
            ns: self.create_publisher(PoseStamped, f'/{ns}/term_goal', 10)
            for ns in INIT_X
        }
        self.history = []  # 最近几条: (world_xyz, {ns: local_xyz}, ok)

        # 双机当前世界坐标——订阅各自mavros的local_position/pose（local
        # map系下的实际位置），加回INIT_X换算成world系，输入面板里常驻
        # 显示，方便决定下一个目标点该给多少。
        self.latest_local_pos = {ns: None for ns in INIT_X}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        for ns in INIT_X:
            self.create_subscription(
                PoseStamped, f'/{ns}/mavros/local_position/pose',
                self._make_pose_cb(ns), qos)

    def _make_pose_cb(self, ns):
        def cb(msg: PoseStamped):
            p = msg.pose.position
            self.latest_local_pos[ns] = (p.x, p.y, p.z)
        return cb

    def current_world_positions(self):
        """{ns: (wx,wy,wz)}，还没收到过位置数据的飞机值是None。"""
        result = {}
        for ns, local in self.latest_local_pos.items():
            if local is None:
                result[ns] = None
            else:
                lx, ly, lz = local
                result[ns] = (lx + INIT_X[ns], ly, lz)
        return result

    def subscriber_counts(self):
        return {ns: pub.get_subscription_count() for ns, pub in self.pubs.items()}

    def send_goals(self, world_targets):
        """world_targets: {ns: (wx,wy,wz)}——同一目标点(3个数输入)时两个
        ns的值相同，各给各的(6个数输入)时不同。按各自的INIT_X换算成
        local坐标后分别发布。"""
        locals_ = {}
        for ns, world_xyz in world_targets.items():
            wx, wy, wz = world_xyz
            lx = wx - INIT_X[ns]
            locals_[ns] = (lx, wy, wz)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = lx
            msg.pose.position.y = wy
            msg.pose.position.z = wz
            msg.pose.orientation.w = 1.0
            self.pubs[ns].publish(msg)
        self.history.append((dict(world_targets), locals_))
        if len(self.history) > 5:
            self.history.pop(0)
        return locals_


def draw(node: DualGoalInput, message=''):
    sys.stdout.write('\033[2J\033[H')
    lines = []
    lines.append(box_top())
    lines.append(box_line('双机目标点手动输入面板（世界坐标）'))
    lines.append(box_sep())
    lines.append(box_line('双机当前世界坐标：'))
    world_pos = node.current_world_positions()
    for ns in INIT_X:
        wp = world_pos.get(ns)
        if wp is None:
            lines.append(box_line(f'  {ns}: 还没收到位置数据...'))
        else:
            wx, wy, wz = wp
            lines.append(box_line(f'  {ns}: ({wx:+.2f}, {wy:+.2f}, {wz:+.2f})'))
    lines.append(box_sep())
    lines.append(box_line('各飞机local map原点相对world的偏移（仅x方向）：'))
    for ns, off in INIT_X.items():
        lines.append(box_line(f'  {ns}: world_x - {off:.1f} = local_x'))
    lines.append(box_sep())
    counts = node.subscriber_counts()
    sub_str = '  '.join(f'{ns}订阅数={c}' for ns, c in counts.items())
    lines.append(box_line(sub_str))
    if any(c == 0 for c in counts.values()):
        lines.append(box_line('!! 有飞机的mighty_node还没连上，指令可能收不到 !!'))
    lines.append(box_sep())
    if node.history:
        lines.append(box_line('最近发送记录：'))
        for world_targets, locals_ in node.history[-3:]:
            same = len(set(world_targets.values())) == 1
            if same:
                wx, wy, wz = next(iter(world_targets.values()))
                lines.append(box_line(f'  world=({wx:+.2f},{wy:+.2f},{wz:+.2f}) [同一目标]'))
            else:
                lines.append(box_line('  [两个不同目标]'))
                for ns, (wx, wy, wz) in world_targets.items():
                    lines.append(box_line(f'    {ns} world=({wx:+.2f},{wy:+.2f},{wz:+.2f})'))
            for ns, (lx, ly, lz) in locals_.items():
                lines.append(box_line(f'    -> {ns} local=({lx:+.2f},{ly:+.2f},{lz:+.2f})'))
        lines.append(box_sep())
    if message:
        lines.append(box_line(message))
        lines.append(box_sep())
    lines.append(box_line('输入 "x y z"（3个数）: 同一目标点同时发给NX01/NX02'))
    lines.append(box_line('输入 "x1 y1 z1 x2 y2 z2"（6个数）: 前3个给NX01，后3个给NX02'))
    lines.append(box_line('输入 q 退出'))
    lines.append(box_bottom())
    sys.stdout.write('\n'.join(lines) + '\n')
    sys.stdout.flush()


def main():
    rclpy.init()
    node = DualGoalInput()

    # 刚起来时给DDS发现一点时间，避免第一条指令因为还没匹配上订阅者
    # 而实际发不出去（跟ros2 topic pub --once同一类坑，这里等最多2秒）。
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(c > 0 for c in node.subscriber_counts().values()):
            break

    # 关键坑：下面主循环里的input('> ')是阻塞调用，会把整个单线程事件循环
    # （包括rclpy.spin_once）一起冻结住——面板只在每次用户敲完一行回车之后
    # 才会重新spin+重绘一次，两次按键之间哪怕话题一直在正常发布，回调也不
    # 会被处理。start.sh全新起容器时，goal窗口从tmux建好到MAVROS/PX4真正
    # 开始发布local_position/pose中间有几十秒空档，这段时间没人会去点这个
    # 窗口敲回车，于是面板就永远停在刚启动那一帧"还没收到位置数据..."上，
    # 哪怕之后飞机早就正常起飞了也不会自动恢复（之前误判是DDS跨容器发现慢
    # 的问题，实测跨容器发现只要一两秒；真正原因是这里）。
    # 修复：进交互输入前先自己常驻spin，每秒重绘一次进度，直到两架飞机都
    # 至少收到过一帧位置数据（或者用户按Ctrl-C跳过），避免被后面的
    # input()阻塞卡在启动瞬间的空状态上。
    if not all(v is not None for v in node.current_world_positions().values()):
        wait_t0 = time.monotonic()
        last_draw = 0.0
        try:
            while not all(v is not None for v in node.current_world_positions().values()):
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_draw > 1.0:
                    elapsed = now - wait_t0
                    draw(node, f'等待双机位置数据到齐...（已等{elapsed:.0f}秒，Ctrl-C可跳过直接进入面板）')
                    last_draw = now
        except KeyboardInterrupt:
            pass

    message = ''
    try:
        while True:
            # 每次重绘前抽干积压的位置消息，保证显示的世界坐标是当前的
            # （不是刚起来时那一帧），最多花0.3秒，不会让人等太久。
            spin_until = time.monotonic() + 0.3
            while time.monotonic() < spin_until:
                rclpy.spin_once(node, timeout_sec=0.05)
            draw(node, message)
            try:
                raw = input('> ').strip()
            except EOFError:
                break
            message = ''

            if raw.lower() in ('q', 'quit', 'exit'):
                break
            if not raw:
                continue

            parts = raw.split()
            if len(parts) not in (3, 6):
                message = (f'!! 需要3个数"x y z"(同一目标)或6个数'
                           f'"x1 y1 z1 x2 y2 z2"(两个目标)，收到{len(parts)}个: {raw!r} !!')
                continue
            try:
                nums = [float(p) for p in parts]
            except ValueError:
                message = f'!! 不是有效的数字: {raw!r} !!'
                continue

            if len(nums) == 3:
                world_targets = {ns: tuple(nums) for ns in INIT_X}
            else:
                world_targets = {
                    'NX01': tuple(nums[0:3]),
                    'NX02': tuple(nums[3:6]),
                }

            rclpy.spin_once(node, timeout_sec=0.0)
            locals_ = node.send_goals(world_targets)
            if len(nums) == 3:
                wx, wy, wz = nums
                message = f'已发送同一目标 world=({wx:+.2f},{wy:+.2f},{wz:+.2f}) 给 NX01/NX02'
            else:
                message = '已分别发送两个不同目标给 NX01/NX02'
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
