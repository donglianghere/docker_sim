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

界面用curses实现（标准库自带，不引入新依赖），不是拿ANSI转义码手写的
"清屏+整页重绘"——早期版本是常驻线程每秒`\\033[2J`清屏重绘整个面板，
跟主线程的`input()`共用同一块终端区域；用户实测反馈"终端不停更新，没法
输入坐标"：每秒一次的整屏清空会把用户正在`input()`里敲到一半的字符
连带清掉，两者天然打架，不是加锁能解决的（锁只能避免两个线程同时写
乱序，解决不了"画面被覆盖"这个根本冲突）。

中间还试过一版"面板+输入框各用一个独立curses子窗口、输入框用mvwin()
跟着面板实际内容行数动态挪位置"的方案，实测这个方案本身有bug（挪动后
输入行直接不显示了，没有继续深挖是两个子窗口哪一步的刷新时序没对上）。
现在这版改成整个界面（面板+输入行）画在同一个stdscr上、每次重绘都是
"擦掉重画"，不是拿两个窗口叠加/挪动——后台线程每秒重绘一次、主线程
每敲一个字符也重绘一次，两边共用同一把锁、画的是同一块缓冲区，不存在
"两个窗口谁盖谁""光标听谁的"这类问题，每次重绘最后一步都会把光标定位
到输入行当前应该在的位置。
"""

import time
import threading
import curses

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


def build_lines(node: DualGoalInput, message=''):
    """把面板内容拼成一个字符串列表——跟老版本的draw()是同一份逻辑，
    只是不再直接往stdout写(ANSI清屏+print)，改成返回行列表，交给调用方
    决定怎么呈现（curses的dash_win.addstr，或者其它场景下的plain
    print）。"""
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
    return lines


class Locked:
    """一个通用的、带锁的共享值容器——后台重绘线程和主线程之间要共享
    "提示消息"和"当前输入框里敲了多少"这两样东西，两边都会读/写，用锁
    保护，不赌"CPython里大概率不会真的炸"。"""

    def __init__(self, value=''):
        self._lock = threading.Lock()
        self._value = value

    def set(self, value):
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value


def render(stdscr, node, message, input_buf, screen_lock):
    """一次性画完"面板+输入行"，全部画在同一个stdscr上，不是面板和输入框
    分开两个curses子窗口。

    第一版试过dash_win+input_win两个独立子窗口、输入框用mvwin()跟着面板
    实际内容的行数动态挪位置——实测这个方案本身就有bug：input_win挪到
    新位置之后画面上完全看不到输入行，两个窗口各自独立的缓冲区在
    noutrefresh/doupdate的合成时序上有没理清楚的地方，一直没找到确切
    是哪一步出的错。与其继续debug两个窗口叠加的时序问题，不如换成根本
    不会有这类问题的架构：只用一个窗口，面板内容和输入行在同一次
    erase+addstr里画完、同一次refresh()提交，天然不存在"两个窗口谁盖
    谁""光标该听哪个窗口的"这些问题——两个线程（后台定时重绘、主线程
    处理完一条命令后的即时重绘）调用的是同一个函数，用screen_lock保证
    不会同时画，最后一步永远是把物理光标停在输入行当前应该在的位置，
    不管是哪个线程画的这一帧，下一个用户按键输入的字符都会出现在正确
    位置。
    """
    with screen_lock:
        lines = build_lines(node, message)
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()
        for i, line in enumerate(lines):
            if i >= max_y - 1:
                break
            try:
                stdscr.addstr(i, 0, line[:max_x - 1])
            except curses.error:
                pass  # 极端窄终端下越界，忽略，不让线程崩掉
        input_row = min(len(lines), max_y - 1)
        prompt = '> ' + input_buf
        try:
            stdscr.addstr(input_row, 0, prompt[:max_x - 1])
        except curses.error:
            pass
        stdscr.move(input_row, min(len(prompt), max_x - 1))
        stdscr.refresh()


def redraw_loop(stdscr, node, shared_message, shared_buf, screen_lock, stop_event):
    """后台常驻线程：一直spin+每秒重绘一次，不依赖用户按键触发。

    关键坑（之前只修了启动阶段、后来又修了"整屏清空跟input()打架"这两
    个版本，这次是同一个bug的第三次迭代）：不管重绘逻辑长什么样，只要
    "刷新画面"和"读取用户输入"这两件事绑在同一个线程的同一个循环里，
    就必然会有一方阻塞/打断另一方。彻底的解法是让它们分别常驻在两个
    线程里，靠共享状态（这里是shared_message/shared_buf）加锁通信，
    互不阻塞——后台线程只管按自己的节奏重绘，主线程只管读键盘，谁都不
    等谁。
    """
    wait_t0 = time.monotonic()
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.2)
        message = shared_message.get()
        if not message and not all(
                v is not None for v in node.current_world_positions().values()):
            elapsed = time.monotonic() - wait_t0
            message = f'等待双机位置数据到齐...（已等{elapsed:.0f}秒）'
        render(stdscr, node, message, shared_buf.get(), screen_lock)
        stop_event.wait(1.0)


def curses_main(stdscr, node):
    curses.curs_set(1)  # 显示光标，停在输入框里
    curses.noecho()     # 自己手动回显输入字符，不用curses自带的echo
    stdscr.keypad(True)  # 让方向键/退格键这些返回curses的KEY_*常量

    shared_message = Locked('')
    shared_buf = Locked('')
    screen_lock = threading.Lock()
    stop_event = threading.Event()
    redraw_thread = threading.Thread(
        target=redraw_loop,
        args=(stdscr, node, shared_message, shared_buf, screen_lock, stop_event),
        daemon=True)
    redraw_thread.start()

    buf = ''
    try:
        while True:
            ch = stdscr.getch()  # 阻塞，但只阻塞主线程，后台重绘线程不受影响

            if ch in (curses.KEY_ENTER, 10, 13):
                raw = buf.strip()
                buf = ''
                shared_buf.set(buf)
                if raw.lower() in ('q', 'quit', 'exit'):
                    break
                if not raw:
                    continue

                parts = raw.split()
                if len(parts) not in (3, 6):
                    shared_message.set(
                        f'!! 需要3个数"x y z"(同一目标)或6个数'
                        f'"x1 y1 z1 x2 y2 z2"(两个目标)，收到{len(parts)}个: {raw!r} !!')
                    render(stdscr, node, shared_message.get(), buf, screen_lock)
                    continue
                try:
                    nums = [float(p) for p in parts]
                except ValueError:
                    shared_message.set(f'!! 不是有效的数字: {raw!r} !!')
                    render(stdscr, node, shared_message.get(), buf, screen_lock)
                    continue

                if len(nums) == 3:
                    world_targets = {ns: tuple(nums) for ns in INIT_X}
                else:
                    world_targets = {
                        'NX01': tuple(nums[0:3]),
                        'NX02': tuple(nums[3:6]),
                    }

                node.send_goals(world_targets)
                if len(nums) == 3:
                    wx, wy, wz = nums
                    shared_message.set(
                        f'已发送同一目标 world=({wx:+.2f},{wy:+.2f},{wz:+.2f}) 给 NX01/NX02')
                else:
                    shared_message.set('已分别发送两个不同目标给 NX01/NX02')

                # 立即触发一次重绘，不用等后台线程的下一个整秒，操作反馈更跟手。
                render(stdscr, node, shared_message.get(), buf, screen_lock)

            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                buf = buf[:-1]
                shared_buf.set(buf)
                render(stdscr, node, shared_message.get(), buf, screen_lock)
            elif ch in (curses.KEY_RESIZE,):
                render(stdscr, node, shared_message.get(), buf, screen_lock)
            elif 32 <= ch <= 126:  # 可打印ASCII——只需要数字/空格/负号/小数点
                buf += chr(ch)
                shared_buf.set(buf)
                render(stdscr, node, shared_message.get(), buf, screen_lock)
    finally:
        stop_event.set()
        redraw_thread.join(timeout=2.0)


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

    try:
        # curses.wrapper负责初始化终端、异常/退出时恢复终端状态（哪怕
        # curses_main内部抛异常也会正确复原，不会把终端留在乱码状态），
        # 比自己手写try/finally去调initscr()/endwin()更不容易漏边界情况。
        curses.wrapper(curses_main, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
