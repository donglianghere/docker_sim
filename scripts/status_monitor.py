#!/usr/bin/env python3
"""多机状态监控面板——从DLIO/PX4里拆出来的几项专用信息，集中、低频、
表格形式显示，一个进程订阅所有飞机的话题、原地刷新，不再是滚动式打印。

背景：DLIO自己的控制台输出（[dlio_odom_node-N]前缀那一大段多行状态面板，
包含Ang Velocity/Accel Bias/Registration/GICP等一堆内部调试字段）刷新太
频繁，把NX01/NX02两个tmux窗口刷屏刷得没法看别的日志（mighty/mavros的
输出全被冲掉了）。watch_sim.sh里改成用grep -v过滤掉了[dlio_odom_node这个
前缀，DLIO本身继续正常跑、继续发布话题，只是不再往这两个窗口里打印。

这个脚本单独订阅各飞机的<ns>/dlio/odom_node/odom（同一个话题，只是换一个
消费者）和<ns>/mavros/setpoint_raw/target_attitude（PX4自己上报的推力/
油门指令，跟attitude_thrust_logger.py用的是同一个话题），按用户要求提取
"位置、姿态(欧拉角，度)、推力/油门、内存消耗、发布频率"这几项。

最初版本是每架飞机各起一个进程、每秒各自print一行，两边输出交替刷屏，
在tmux窗口里看起来一直在滚动/闪烁。改成一个进程同时订阅所有飞机的话题，
每秒用ANSI转义码清屏+回到左上角，原地重绘一张固定表格（飞机数不变、
表格行数不变，只有单元格内容在更新），不再有滚动感。

用法（在能同时访问所有飞机话题的容器内跑，比如任意一个flight-stack
容器——ROS2话题在同一个DDS domain下跨容器可见，不需要专门起在"每架
飞机自己的"容器里）：
  python3 status_monitor.py NX01 NX02

用户明确要求同时显示局部坐标和全局坐标，两栏并排：局部坐标直接是
`<ns>/dlio/odom_node/odom`本身（每架飞机自己DLIO SLAM原点为(0,0,0)的
局部系，也是发goal给term_goal时mighty内部实际使用的坐标系）；全局坐标
是局部坐标加上每架飞机在世界坐标系下的x方向偏移`INIT_X`（跟
`dual_goal_input.py`用的是同一套换算、同一组数值，NX01=3.0/NX02=6.0，
仅x方向有偏移——两机spawn时y相同、只在x轴错开，且DLIO重力对齐后yaw
也接近0，所以偏移是纯平移不需要旋转）。两栏坐标数值上差的正好是这个
固定偏移，摆在一起方便肉眼核对"两架飞机在世界里到底隔多远"，不用再
心算。
"""

import os
import sys
import time
import math
import subprocess
import unicodedata
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget

# 每架飞机local map原点相对world的偏移，只在x方向——跟dual_goal_input.py
# 里的INIT_X是同一套约定(NX01=3, NX02=6)，两处保持一致，不要单独改一处。
INIT_X = {
    'NX01': 3.0,
    'NX02': 6.0,
}


def build_config_banner():
    """规划器/板外控制器/定位方式这几项是"这次跑的是哪套组合"的关键信息，
    2026-08-07第一次真正对比px4ctrl和ros2_px4_stack稳定性之后用户要求
    加到status面板里——光看NX01/NX02窗口的滚动日志很难第一时间确认当前
    到底是拿哪套配置在跑，容易跟前一次的测试结果搞混。

    规划器这个项目里目前只接了mighty一种（没有能切换规划器的环境变量），
    先写死；CONTROLLER/LOCALIZATION_SOURCE都是flight-stack-entrypoint.sh
    里读的环境变量，这个脚本本来就跑在某一个flight-stack容器内部（docker
    exec进来的），直接读同一份环境变量即可，不需要额外传参。"""
    controller = os.environ.get('CONTROLLER', 'ros2_px4_stack')
    if controller == 'ros2_px4_stack':
        control_law = os.environ.get('CONTROL_LAW', 'trajectory')
        controller_label = f"ros2_px4_stack (control_law={control_law})"
    elif controller == 'px4ctrl':
        controller_label = "px4ctrl (ROS2版，实验性)"
    else:
        controller_label = controller

    loc_source = os.environ.get('LOCALIZATION_SOURCE', 'dlio')
    loc_label = {
        'dlio': 'dlio (真实SLAM)',
        'gt': 'gt (Gazebo仿真真值)',
    }.get(loc_source, loc_source)

    return f"规划器: mighty  |  板外控制器: {controller_label}  |  定位方式: {loc_label}"


def quat_to_euler_deg(x, y, z, w):
    """标准ZYX（航空）欧拉角约定，跟这次session排查yaw反向问题时用的是
    同一套公式，保持一致，方便直接拿这个工具的输出去对照。"""
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


def display_width(s):
    """中文表头（"局部坐标"这类）在终端里每个字占两列，len()只算一个
    字符——表头是中文、数据行(NX01/±3.04这些)基本是纯ASCII，两者按
    len()对齐会导致表头文字比数据行的边框宽，竖线对不齐。这里跟
    dual_goal_input.py的display_width()是同一个思路，按East Asian
    Width分类估算实际显示宽度，再用它来做padding。"""
    width = 0
    for ch in s:
        width += 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1
    return width


def pad(s, width):
    """按实际显示宽度补空格到width列，而不是按len()补——配合
    display_width()解决中文表头对不齐的问题。"""
    return s + ' ' * max(0, width - display_width(s))


def get_container_total_rss_mb():
    """当前容器（这个脚本本来就跑在flight-stack容器内部，docker exec进来的）
    所有进程RSS总和——docker exec天然加入目标容器自己的PID namespace，
    ps在这里看到的就是这个容器自己的进程列表，不用额外配置。注意：多机
    场景下这个数字只是"脚本自己所在的那一个容器"的内存，不是所有飞机的
    总和，所以只在这个容器对应的那一行显示，其它飞机的行留空。"""
    try:
        out = subprocess.check_output(['ps', '-eo', 'rss='], text=True)
        total_kb = sum(int(x) for x in out.split() if x.strip())
        return total_kb / 1024.0
    except Exception:
        return float('nan')


class AgentState:
    def __init__(self):
        self.odom = None
        self.thrust = None
        self.recv_times = deque(maxlen=30)  # 滑动窗口算发布频率，30帧够平滑


class StatusMonitor(Node):
    def __init__(self, namespaces):
        super().__init__('multi_status_monitor')
        self.namespaces = namespaces
        self.agents = {ns: AgentState() for ns in namespaces}
        self.local_mem_mb = float('nan')  # 只有本容器所在的那个ns能拿到
        # 只在启动时读一次环境变量——运行期间这几个值不会变，没必要每秒
        # 重新拼一次字符串。
        self.config_banner = build_config_banner()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        for ns in namespaces:
            self.create_subscription(
                Odometry, f'/{ns}/dlio/odom_node/odom',
                self._make_odom_cb(ns), qos)
            self.create_subscription(
                AttitudeTarget, f'/{ns}/mavros/setpoint_raw/target_attitude',
                self._make_thrust_cb(ns), qos)

        self.create_timer(1.0, self._redraw)

    def _make_odom_cb(self, ns):
        def cb(msg: Odometry):
            agent = self.agents[ns]
            agent.odom = msg
            agent.recv_times.append(time.monotonic())
        return cb

    def _make_thrust_cb(self, ns):
        def cb(msg: AttitudeTarget):
            self.agents[ns].thrust = msg.thrust
        return cb

    def _hz(self, agent: AgentState):
        if len(agent.recv_times) < 2:
            return 0.0
        span = agent.recv_times[-1] - agent.recv_times[0]
        if span <= 1e-6:
            return 0.0
        return (len(agent.recv_times) - 1) / span

    def _row(self, ns):
        agent = self.agents[ns]
        if agent.odom is None:
            pos_local = "  等待数据...   "
            pos_world = "  等待数据...   "
            rpy = "  等待数据...   "
        else:
            p = agent.odom.pose.pose.position
            q = agent.odom.pose.pose.orientation
            roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
            init_x = INIT_X.get(ns, 0.0)
            pos_local = f"{p.x:+6.2f},{p.y:+6.2f},{p.z:+6.2f}"
            pos_world = f"{p.x + init_x:+6.2f},{p.y:+6.2f},{p.z:+6.2f}"
            rpy = f"{roll:+6.1f},{pitch:+6.1f},{yaw:+6.1f}"

        thrust = "  -  " if agent.thrust is None else f"{agent.thrust*100:5.1f}%"
        hz = f"{self._hz(agent):5.1f}"
        return ns, pos_local, pos_world, rpy, thrust, hz

    def _redraw(self):
        self.local_mem_mb = get_container_total_rss_mb()

        cols = [
            ("飞机", 6), ("局部坐标 x,y,z (m)", 22), ("全局坐标 x,y,z (m)", 22),
            ("姿态 r,p,y (deg)", 22), ("推力/油门", 9), ("发布频率", 8),
        ]
        sep = "+" + "+".join("-" * (w + 2) for _, w in cols) + "+"
        header = "|" + "|".join(f" {pad(name, w)} " for name, w in cols) + "|"

        lines = []
        lines.append(self.config_banner)
        lines.append(sep)
        lines.append(header)
        lines.append(sep)
        for ns in self.namespaces:
            ns_, pos_local, pos_world, rpy, thrust, hz = self._row(ns)
            row = (
                f"| {pad(ns_, 6)} | {pad(pos_local, 22)} | {pad(pos_world, 22)} | "
                f"{pad(rpy, 22)} | {pad(thrust, 9)} | {pad(hz, 8)} |"
            )
            lines.append(row)
        lines.append(sep)
        lines.append("局部坐标=各飞机自己DLIO SLAM原点为(0,0,0)的坐标系(mighty内部/term_goal用的就是这个)；"
                      "全局坐标=局部坐标+INIT_X偏移(NX01=+3.0,NX02=+6.0，仅x方向)")
        lines.append(f"本容器内存占用: {self.local_mem_mb:.0f}MB"
                      f"  (只统计这个脚本所在容器自己的进程，不是全部飞机的总和)")
        lines.append(f"刷新时间: {time.strftime('%H:%M:%S')}"
                      "   (原地刷新，不滚动——按 Ctrl-a 切到别的tmux窗口看完整日志)")

        # ANSI: 清屏(\033[2J) + 光标回左上角(\033[H)，原地重绘整张表，
        # 不再每秒往下追加新行造成滚动/闪烁感。
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def main():
    if len(sys.argv) < 2:
        print("用法: status_monitor.py <NAMESPACE1> [NAMESPACE2 ...]，如 status_monitor.py NX01 NX02",
              file=sys.stderr)
        sys.exit(1)
    namespaces = sys.argv[1:]

    rclpy.init()
    node = StatusMonitor(namespaces)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
