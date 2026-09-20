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
"位置、姿态(欧拉角，度)、推力/油门、发布频率"这几项。

2026-08-12改版：页面固定为3张表格（"1 飞行参数"/"2 空间对齐"/"3 子系统
健康状态"），每张表格只有标题+表格主体，之前的配置横幅(规划器/控制器/
定位方式)、内存占用、刷新时间、各表格下方的文字说明全部去掉——用户明确
要求"其他说明性文字不要"。tmux状态栏(status-right)也同一次改版从"每个
子系统的健康状态图标"换成"每架飞机的实时飞行时长"，子系统健康状态现在
只在这个status窗口的表格3里看，不再常驻状态栏。

最初版本是每架飞机各起一个进程、每秒各自print一行，两边输出交替刷屏，
在tmux窗口里看起来一直在滚动/闪烁。改成一个进程同时订阅所有飞机的话题，
每秒用ANSI转义码清屏+回到左上角，原地重绘一张固定表格（飞机数不变、
表格行数不变，只有单元格内容在更新），不再有滚动感。

用法（在能同时访问所有飞机话题的容器内跑，比如任意一个flight-stack
容器——ROS2话题在同一个DDS domain下跨容器可见，不需要专门起在"每架
飞机自己的"容器里）：
  python3 status_monitor.py NX01 NX02

======================================================================
子系统健康监控（2026-08-09新增）
======================================================================
起因：NX02的DLIO（SLAM）中途挂了，但当时没人在看NX01/status这几个窗口
（tmux窗口切走了、注意力在别处），表格本身原地刷新+发布频率这一列虽然
会掉到0，但只有真正切到status窗口才看得见——这套系统本来就有"窗口在
后台也继续跑"这个tmux特性（这也是这个脚本能一直在后台统计发布频率的
前提），但反过来这也意味着"后台出问题"完全可以在后台无声无息地发生，
不会主动跳出来提醒人。

只让这张表格本身更好看没用，问题的本质是"健康信息只在你恰好在看的那个
窗口里可见"。当时的修复是把关键健康位挪到tmux状态栏（status-right，
`set -g status-interval 1`本来就已经是1秒刷新）——状态栏在任何窗口下都
常驻可见，不管你当前停在launch/NX01/NX02/goal/record/logs哪一个窗口，
一眼扫过去就知道哪架飞机的哪个子系统掉线了，不用先猜"是不是该去status
窗口看看"。

⚠️ 2026-08-12更新：status-right这块位置改成显示飞行时间了（见下面
`_format_status_bar()`），不再显示这几个子系统健康图标——用户明确要求
"tmux窗口右下角的健康状态显示改为飞行时间显示"。下面这一整段"健康信息
只在你恰好在看的那个窗口里可见"的历史背景描述仍然成立、健康监控本身
(判定逻辑/阈值/健康事件日志/status窗口里的"3 子系统健康状态"表格)一个
都没有少，只是"常驻状态栏"这一层可见性保证被换成了飞行时间——如果又出现
一次"没人盯着status窗口、后台子系统悄悄挂了"，现在不会像2026-08-09那次
一样在状态栏上直接看到，只能靠事后翻`health_events.log`或恰好切进status
窗口才发现，这是这次改动的已知取舍，用户已经知情。

具体做法：这个脚本本来就是一个ROS2节点、本来就在1Hz的_redraw()定时器
里跑（这个定时器不需要新起，复用），额外订阅几个能代表"每个子系统进程
是不是还活着"的话题，按"多久没收到新消息"判定健康/掉线，每次_redraw()
顺带把结果写成一行tmux格式化文本，原子写入到
`/logs/health_status.txt`（这个路径是docker-compose.yml里三个容器都挂载
的`./runtime_logs:/logs`卷，宿主机侧路径是`runtime_logs/health_status.txt`，
不需要额外挂载）。宿主机上tmux的status-right配置成
`#(scripts/render_tmux_status.sh)`，`status-interval 1`会每秒重新执行这个
命令、把文件内容原样贴到状态栏里——tmux会解析文本里的`#[fg=...]`片段做
彩色高亮（红=掉线，绿=正常，灰=启动中/该配置下不监控这一项）。

监控的5个子系统、每个用一个汉字做状态栏里的紧凑标签（147列宽的tmux窗口
放不下"雷达点云/SLAM定位/规划器/板外控制器/MAVROS链路"这种完整词）：
  雷 = Gazebo雷达点云传感器反馈（<ns>/mid360_PointCloud2）——处在DLIO
       上游，跟"位"一起看能区分"Gazebo没在发点云"还是"DLIO自己收到点云
       但处理挂了"这两种情况，对应这次的排查经验：不能光看DLIO自己的
       输出话题，上游数据源本身有没有到达也要单独确认。
  位 = 定位/SLAM（<ns>/dlio/odom_node/odom）——LOCALIZATION_SOURCE=uwb_slam
       和=gt两种模式发布的是同一个话题名（entrypoint里的注释原话），
       这里不用关心当前具体用的是哪种定位源。这就是这次真正挂掉的那个
       子系统。
  规 = 规划器（<ns>/point_current_state，mighty_node.cpp收到state消息
       就会发布，等于是"规划器还在正常处理输入"的心跳）——只在
       PLANNER=mighty时订阅；PLANNER=ego_planner时这个话题不存在
       （ego_planner没有对应的心跳话题，也还没实测飞过，标成"不监控"
       而不是硬造一个可能不准的判定）。
  控 = 板外控制器（<ns>/mavros/setpoint_raw/target_attitude，跟原来
       表格里"推力/油门"那一列同一个话题）——只在CONTROLLER=
       ros2_px4_stack时判定；=px4ctrl时RUN_OFFBOARD_FOLLOWER=false，
       track_dynus_traj根本不会起、这个话题不会被这套板外控制器发布，
       同样标成"不监控"，不强行判定。
  链 = MAVROS/PX4链路（<ns>/mavros/state，用消息本身的connected字段，
       不是单纯的消息新鲜度——万一MAVROS进程还在正常发布state消息、
       但connected=false，说明是PX4那一侧断了，只看"有没有收到消息"
       会漏掉这种情况）。
另外一项不分飞机、只有一个（两机共用同一个UWB模拟节点）：
  UWB = 双机坐标系对齐（/frame_align/<两机namespace按字典序排列后的第
       一对>，uwb_ground_truth_node.py发布），不区分具体哪对，两机场景
       下只有这一对。

状态分四种：
  正常(绿) —— 最近一次收到消息距今没超过对应阈值（阈值按各话题预期发布
              频率给的，不是统一值，见STALE_THRESHOLDS_SEC）；MAVROS链路
              额外要求connected=true。
  掉线(红) —— 曾经收到过消息，但已经超过阈值没有新消息——这是最值得
              关注的一种，代表"本来在跑，现在不跑了"，NX02那次DLIO
              就是这种。
  启动中(灰) —— 从这个监控节点自己启动到现在还没超过
              HEALTH_STARTUP_GRACE_SEC（默认90秒，环境变量可调）、且一次
              都没收到过消息——容器刚起来时PX4 SITL/DLIO/mighty这些都要
              花时间初始化，这段时间内没数据是正常的，不应该判定成
              "掉线"制造误报。超过这个宽限期还是一次没收到过，才算掉线
              （见下面'fail'状态里"从来没启动成功"和"启动后又挂了"合并
              处理的说明）。
  不监控(灰) —— 当前PLANNER/CONTROLLER组合下这个子系统本来就不会发布
              这个话题（见上面"规"/"控"的说明），不是掉线，只是这个
              指标在当前配置下没有意义。

只判定"正常"和"掉线"这两种状态之间的转换会被记一笔到
`/logs/incidents/health_events.log`（追加写，不滚动删除，跟incidents/
目录下save_incident.sh手动留证的那份不是同一个东西——这个是自动的、
持续的事件时间线，save_incident.sh是"事发时手动多留一份rosbag+日志
现场"）——这样即便真的又发生一次"没人在看状态栏"，事后也能翻这个文件
查到"到底几点几分哪个子系统开始没数据的"，不用再靠回忆或者猜。

用户明确要求同时显示局部坐标和全局坐标，两栏并排：局部坐标直接是
`<ns>/dlio/odom_node/odom`本身（每架飞机自己DLIO SLAM原点为(0,0,0)的
局部系，也是发goal给term_goal时mighty内部实际使用的坐标系）；全局坐标
是局部坐标换算到world系。

⚠️ 2026-08-12改：全局坐标原来是"局部坐标+INIT_X"这种只在x方向平移的
硬编码换算(NX01=3.0/NX02=6.0)，假设两机spawn yaw都是0、局部系跟世界系
只差平移。这个假设已经被本次session早些时候"给两机各自设置非零spawn
yaw"这个改动打破(推导过程见dual_goal_input.py文件头同一天的说明，两处是同一个
根因)——改成用`origin_setter_node`广播的`world -> {ns}/map`这条TF现查
现算(tf2_ros.Buffer，含θ*旋转)，公式、z轴处理都跟dual_goal_input.py的
local_to_world()保持一致，两处独立实现、不共享代码(各自是独立脚本、
docker cp各自现改现塞，没有共同依赖的公共库)，但数学上必须保持同步——
以后改这套SE(2)换算逻辑要记得两个文件一起改。还没完成起飞点锁定时
（查不到这条TF）全局坐标列显示"未标定"，不再用旧的错误算法硬凑一个
看似正常但实际错误的数字。
"""

import os
import sys
import time
import math
import unicodedata
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State, ExtendedState
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Float64, Int32
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# ExtendedState.landed_state在"离地"这几种取值里都算"在空中"，不只是稳定
# 巡航的IN_AIR——起飞/降落这两个过渡态本身也应该计入飞行时间，不然刚离地
# 那几秒会被漏掉。ON_GROUND/UNDEFINED不算。
_AIRBORNE_LANDED_STATES = {
    ExtendedState.LANDED_STATE_IN_AIR,
    ExtendedState.LANDED_STATE_TAKEOFF,
    ExtendedState.LANDED_STATE_LANDING,
}

# origin_setter_node.py里rotation_min_segments参数的默认值——这里只是给
# 显示面板算"还差几段"用的展示常量，不是真的去读对方节点的实时参数值
# （待补：更严谨的做法是用ros2 param get现场查一次，这里先用静态常量，
# 如果以后改了origin_setter_node那边的默认值，这里也要跟着手动改）。
ROTATION_MIN_SEGMENTS = 1

def _quat_yaw(qz, qw):
    """跟dual_goal_input.py的_quat_yaw()是同一个公式，两处独立实现，见
    文件头2026-08-12说明。"""
    return 2.0 * math.atan2(qz, qw)


def local_to_world(tf, lx, ly, lz):
    """跟dual_goal_input.py的local_to_world()是同一个SE(2)公式，两处
    独立实现，见文件头2026-08-12说明。"""
    t = tf.transform.translation
    theta = _quat_yaw(tf.transform.rotation.z, tf.transform.rotation.w)
    c, s = math.cos(theta), math.sin(theta)
    wx = t.x + lx * c - ly * s
    wy = t.y + lx * s + ly * c
    wz = t.z + lz
    return wx, wy, wz

# 子系统健康监控——话题名/字段含义见上面文件头的大段说明。key是状态栏里的
# 紧凑标签(单个汉字)，per_agent=True表示每架飞机各判定一份，=False表示
# 全局只有一份(目前只有UWB)。
HEALTH_SUBSYSTEMS = [
    # key   标签  中文全称                    per_agent
    ('lidar', '雷', 'Gazebo雷达点云传感器反馈', True),
    ('slam',  '位', '定位/SLAM',                True),
    ('plan',  '规', '规划器',                   True),
    ('ctrl',  '控', '板外控制器',               True),
    ('link',  '链', 'MAVROS/PX4链路',           True),
    ('uwb',   'UWB', '双机坐标系对齐',          False),
]

# 各话题预期发布频率不一样，"多久没收到新消息算掉线"也应该跟着不一样——
# 统一用一个阈值要么让高频话题(点云/推力setpoint)反应太慢，要么让低频话题
# (mavros/state通常按MAVLink心跳约定1Hz左右)误报。数字来自各话题的实际
# 设计频率(mid360点云/DLIO odom/setpoint_raw通常几十Hz、UWB模拟节点默认
# publish_rate_hz=10)，留了3-5倍余量，不是掐着实测频率卡的下限。
STALE_THRESHOLDS_SEC = {
    'lidar': 2.0,
    'slam': 2.0,
    'plan': 2.0,
    'ctrl': 2.0,
    'link': 6.0,   # PX4心跳按MAVLink约定通常1Hz，留够余量避免抖动误报
    'uwb': 3.0,
}

HEALTH_FILE = os.environ.get('HEALTH_FILE', '/logs/health_status.txt')
HEALTH_INCIDENT_LOG = os.environ.get('HEALTH_INCIDENT_LOG', '/logs/incidents/health_events.log')
# 容器刚起来时PX4 SITL/DLIO/mighty这些都要花时间初始化(entrypoint.sh里
# 自己就有好几处sleep 2/3/8)，这段时间内一次都没收到消息是正常现象，不该
# 判定成"掉线"——给够宽限期再开始真正判定，默认90秒留了充足余量。
HEALTH_STARTUP_GRACE_SEC = float(os.environ.get('HEALTH_STARTUP_GRACE_SEC', '90'))

# 2026-08-12新增"4 资源消耗"表格用——这个文件由宿主机侧的
# scripts/collect_container_stats.sh每秒写一次（docker stats + nvidia-smi
# 只有宿主机能看，容器内部因为PID/cgroup namespace隔离看不到"隔壁容器"的
# 资源占用，见该脚本文件头说明），跟HEALTH_FILE走的是同一个runtime_logs/
# 共享卷（容器里挂载成/logs）。
CONTAINER_STATS_FILE = os.environ.get('CONTAINER_STATS_FILE', '/logs/container_stats.txt')
CONTAINER_STATS_STALE_SEC = 5.0  # 采集脚本没在跑/卡住时不能显示旧数据当新数据
CONTAINER_DISPLAY_NAMES = {
    'docker_sim-sim-world-1': 'sim-world',
    'docker_sim-flight-stack-nx01-1': 'NX01',
    'docker_sim-flight-stack-nx02-1': 'NX02',
}


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


class AgentState:
    def __init__(self):
        self.odom = None
        self.thrust = None
        # 滑动窗口算发布频率，30帧够平滑——按子系统分开存，'点云频率'/'SLAM
        # 里程计输出频率'/'规划器输出频率'各自独立统计，不能共用一个deque。
        self.hz_recv_times = {
            'lidar': deque(maxlen=30),
            'slam': deque(maxlen=30),
            'plan': deque(maxlen=30),
        }
        # 健康监控——每个子系统key对应"最近一次收到消息"的单调时钟时间戳，
        # 没收到过就是None(区分"从没启动成功"和"启动过又断了"，两者在
        # _subsystem_state()里都按同一个健康位判定，但语义不同，保留这个
        # None区分是为了万一以后要单独统计"从没起来过"这类问题)。
        self.last_recv = {}
        self.mavros_connected = None  # None=还没收到过mavros/state消息
        # 情景二SE(2)在线旋转估计的实时展示——见origin_setter_node.py
        # 2026-08-12新增的.../origin_setter/yaw_estimate、
        # .../origin_setter/yaw_sample_count两个诊断话题。sample_count在
        # 攒够ROTATION_MIN_SEGMENTS段之前这两个话题都从来不会发布过一次
        # （origin_setter_node自己的设计如此，不是这个监控脚本的问题），
        # None表示"还没攒够、这两个话题至今一次都没发布过"，不是"掉线"。
        self.yaw_estimate_rad = None
        self.yaw_sample_count = None
        # 飞行时间——mavros/extended_state的landed_state字段，见
        # _AIRBORNE_LANDED_STATES。landed_state=None表示还没收到过消息；
        # takeoff_monotonic=None表示当前不在空中(还没起飞，或已经落地)，
        # 非None时是最近一次"从非空中转入空中"那一刻的单调时钟时间戳，
        # 拿它跟当前时间做差就是实时飞行时长。
        self.landed_state = None
        self.takeoff_monotonic = None


class StatusMonitor(Node):
    def __init__(self, namespaces):
        super().__init__('multi_status_monitor')
        self.namespaces = namespaces
        self.agents = {ns: AgentState() for ns in namespaces}
        # 只在启动时读一次环境变量——运行期间这几个值不会变，_compute_health()
        # 里判定"规"/"控"这两项是否要标成"不监控"要用到。
        self.planner = os.environ.get('PLANNER', 'ego_planner')
        self.controller = os.environ.get('CONTROLLER', 'px4ctrl')

        # 健康监控用——见文件头大段说明。node_start_time给启动宽限期计时；
        # prev_health记上一次_redraw()算出的状态，只有状态真的变化时才往
        # health_events.log里追加一行，不然会变成每秒一行的刷屏日志。
        self.node_start_time = time.monotonic()
        self.prev_health = {}
        self.uwb_last_recv = None
        self.uwb_pair = tuple(sorted(namespaces)[:2]) if len(namespaces) >= 2 else None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        for ns in namespaces:
            self.create_subscription(
                Odometry, f'/{ns}/dlio/odom_node/odom',
                self._make_odom_cb(ns), qos)
            self.create_subscription(
                AttitudeTarget, f'/{ns}/mavros/setpoint_raw/target_attitude',
                self._make_thrust_cb(ns), qos)
            self.create_subscription(
                PointCloud2, f'/{ns}/mid360_PointCloud2',
                self._make_recv_cb(ns, 'lidar'), qos)
            self.create_subscription(
                State, f'/{ns}/mavros/state',
                self._make_mavros_state_cb(ns), qos)
            self.create_subscription(
                ExtendedState, f'/{ns}/mavros/extended_state',
                self._make_extended_state_cb(ns), qos)
            # "规"(规划器心跳)只在PLANNER=mighty时有对应话题可订阅——
            # ego_planner没有等价的心跳话题，见文件头说明，不强行造一个。
            if self.planner == 'mighty':
                self.create_subscription(
                    PointStamped, f'/{ns}/point_current_state',
                    self._make_recv_cb(ns, 'plan'), qos)
            # 情景二θ*在线估计实时展示——origin_setter_node发布用的是默认
            # QoS(RELIABLE)，不是上面这个BEST_EFFORT的qos，两个话题都
            # 单独订阅、各自用匹配的QoS（这次session里已经踩过好几次QoS
            # 不匹配导致"订阅了但永远收不到消息、还不报错"的坑，这里显式
            # 对齐，不复用上面那个qos变量）。
            self.create_subscription(
                Float64, f'/{ns}/origin_setter/yaw_estimate',
                self._make_yaw_estimate_cb(ns), 10)
            self.create_subscription(
                Int32, f'/{ns}/origin_setter/yaw_sample_count',
                self._make_yaw_sample_count_cb(ns), 10)

        if self.uwb_pair is not None:
            ns_a, ns_b = self.uwb_pair
            self.create_subscription(
                TransformStamped, f'/frame_align/{ns_a}/{ns_b}',
                self._uwb_cb, qos)

        # 全局坐标换算用——见文件头2026-08-12说明，world -> {ns}/map这条TF
        # 由origin_setter_node广播，含θ*旋转。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self._redraw)

    def _lookup_world_map_tf(self, ns):
        try:
            return self.tf_buffer.lookup_transform('world', f'{ns}/map', RclTime())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _make_odom_cb(self, ns):
        def cb(msg: Odometry):
            agent = self.agents[ns]
            agent.odom = msg
            now = time.monotonic()
            agent.hz_recv_times['slam'].append(now)
            agent.last_recv['slam'] = now
        return cb

    def _make_thrust_cb(self, ns):
        def cb(msg: AttitudeTarget):
            self.agents[ns].thrust = msg.thrust
            self.agents[ns].last_recv['ctrl'] = time.monotonic()
        return cb

    def _make_recv_cb(self, ns, key):
        """通用的"只关心有没有收到消息、不关心内容"回调——雷达点云/规划器
        心跳这两项只用来判定健康，不需要像odom/thrust那样解析字段。key在
        hz_recv_times里有对应deque时顺带记一帧，用于算发布频率(目前是
        lidar/plan两个key，slam走_make_odom_cb单独记，ctrl/link不需要
        频率显示)。"""
        def cb(_msg):
            now = time.monotonic()
            agent = self.agents[ns]
            agent.last_recv[key] = now
            if key in agent.hz_recv_times:
                agent.hz_recv_times[key].append(now)
        return cb

    def _make_mavros_state_cb(self, ns):
        def cb(msg: State):
            agent = self.agents[ns]
            agent.last_recv['link'] = time.monotonic()
            agent.mavros_connected = msg.connected
        return cb

    def _make_extended_state_cb(self, ns):
        def cb(msg: ExtendedState):
            agent = self.agents[ns]
            was_airborne = agent.landed_state in _AIRBORNE_LANDED_STATES
            agent.landed_state = msg.landed_state
            now_airborne = msg.landed_state in _AIRBORNE_LANDED_STATES
            if now_airborne and not was_airborne:
                agent.takeoff_monotonic = time.monotonic()
            elif not now_airborne:
                agent.takeoff_monotonic = None
        return cb

    def _uwb_cb(self, _msg):
        self.uwb_last_recv = time.monotonic()

    def _make_yaw_estimate_cb(self, ns):
        def cb(msg: Float64):
            self.agents[ns].yaw_estimate_rad = msg.data
        return cb

    def _make_yaw_sample_count_cb(self, ns):
        def cb(msg: Int32):
            self.agents[ns].yaw_sample_count = msg.data
        return cb

    def _hz_for(self, agent: AgentState, key):
        times = agent.hz_recv_times.get(key)
        if not times or len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        if span <= 1e-6:
            return 0.0
        return (len(times) - 1) / span

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
            pos_local = f"{p.x:+6.2f},{p.y:+6.2f},{p.z:+6.2f}"
            tf = self._lookup_world_map_tf(ns)
            if tf is None:
                pos_world = "未标定"
            else:
                wx, wy, wz = local_to_world(tf, p.x, p.y, p.z)
                pos_world = f"{wx:+6.2f},{wy:+6.2f},{wz:+6.2f}"
            rpy = f"{roll:+6.1f},{pitch:+6.1f},{yaw:+6.1f}"

        thrust = "  -  " if agent.thrust is None else f"{agent.thrust*100:5.1f}%"
        hz = f"{self._hz_for(agent, 'slam'):5.1f}"
        return ns, pos_local, pos_world, rpy, thrust, hz

    def _subsystem_state(self, key, last_recv, extra_ok=True):
        """单个子系统当前健康状态——'ok'/'fail'/'wait'。extra_ok给mavros
        链路用：消息本身新鲜(没超阈值)不代表PX4真的连上了，还要看
        connected字段，两个条件都满足才算'ok'。"""
        if last_recv is None:
            in_grace = (time.monotonic() - self.node_start_time) < HEALTH_STARTUP_GRACE_SEC
            return 'wait' if in_grace else 'fail'
        age = time.monotonic() - last_recv
        if age <= STALE_THRESHOLDS_SEC[key] and extra_ok:
            return 'ok'
        return 'fail'

    def _compute_health(self):
        """返回 {(ns_or_None, key): state} 的字典，key='uwb'时ns_or_None是
        None(全局只有一份，不分飞机)。"""
        health = {}
        for ns in self.namespaces:
            agent = self.agents[ns]
            for key, _label, _desc, per_agent in HEALTH_SUBSYSTEMS:
                if not per_agent:
                    continue
                if key == 'plan' and self.planner != 'mighty':
                    health[(ns, key)] = 'na'
                    continue
                if key == 'ctrl' and self.controller != 'ros2_px4_stack':
                    health[(ns, key)] = 'na'
                    continue
                if key == 'link':
                    extra_ok = agent.mavros_connected is True
                    health[(ns, key)] = self._subsystem_state(key, agent.last_recv.get(key), extra_ok)
                else:
                    health[(ns, key)] = self._subsystem_state(key, agent.last_recv.get(key))
        if self.uwb_pair is not None:
            health[(None, 'uwb')] = self._subsystem_state('uwb', self.uwb_last_recv)
        return health

    _HEALTH_STATE_LABEL = {
        'ok': '正常', 'fail': '掉线', 'wait': '启动中', 'na': '不监控',
    }

    def _log_health_transitions(self, health):
        """只记'正常'<->'掉线'之间的转换，'wait'/'na'不算——见文件头说明。
        追加写、不滚动删除，跟save_incident.sh手动留证是两回事，这里是
        自动的持续事件时间线，供事后回查"到底几点几分开始没数据的"。"""
        interesting = {'ok', 'fail'}
        changed_lines = []
        for (ns, key), state in health.items():
            prev = self.prev_health.get((ns, key))
            if state in interesting and prev in interesting and state != prev:
                label = next(l for k, l, _d, _p in HEALTH_SUBSYSTEMS if k == key)
                desc = next(d for k, _l, d, _p in HEALTH_SUBSYSTEMS if k == key)
                who = ns if ns else 'UWB(双机共用)'
                ts = time.strftime('%Y-%m-%d %H:%M:%S')
                changed_lines.append(
                    f"[{ts}] {who} {label}({desc}) 从{self._HEALTH_STATE_LABEL[prev]}"
                    f"变为{self._HEALTH_STATE_LABEL[state]}"
                )
        self.prev_health = health
        if not changed_lines:
            return
        try:
            os.makedirs(os.path.dirname(HEALTH_INCIDENT_LOG), exist_ok=True)
            with open(HEALTH_INCIDENT_LOG, 'a') as f:
                for line in changed_lines:
                    f.write(line + '\n')
        except OSError:
            pass  # 事件日志写失败不应该影响状态栏本身继续工作

    _TMUX_COLOR = {
        'ok': '#[fg=colour46]', 'fail': '#[fg=colour196]',
        'wait': '#[fg=colour244]', 'na': '#[fg=colour244]',
    }
    _ANSI_COLOR = {'ok': '\033[32m', 'fail': '\033[31m', 'wait': '\033[90m', 'na': '\033[90m'}

    def _format_status_bar(self):
        """给tmux status-right用的紧凑单行——`#[fg=...]`只有tmux状态栏会
        解析，不能直接print到终端(会看到裸的"#[fg=colour46]"文字)。

        2026-08-12改：原来这里显示的是每个子系统的健康状态图标(雷/位/规/
        控/链)，用户明确要求改成飞行时间显示——子系统健康状态现在只在
        status窗口里的"3 子系统健康状态"表格显示，不再常驻tmux右下角。"""
        parts = []
        for ns in self.namespaces:
            agent = self.agents[ns]
            if agent.takeoff_monotonic is None:
                parts.append(f'{self._TMUX_COLOR["wait"]}{ns}未起飞#[fg=default]')
            else:
                elapsed = int(time.monotonic() - agent.takeoff_monotonic)
                mm, ss = divmod(elapsed, 60)
                parts.append(f'{self._TMUX_COLOR["ok"]}{ns}飞行{mm:02d}:{ss:02d}#[fg=default]')
            parts.append(' ')
        parts.append(time.strftime('%H:%M:%S'))
        return ''.join(parts)

    def _colored_pad(self, label, width, state):
        """跟pad()一样按显示宽度补空格，但只给标签本身套ANSI颜色、不给
        补齐用的空格套色——否则一整块背景色/前景色会在有的终端里看起来
        像整个单元格被"填色"了，观感比只有文字变色要吵。pad()内部按
        display_width()算出的补齐结果里，前len(label)个字符必然就是label
        原样（补的空格全在后面），可以直接按字符数切片拿到"纯空格"部分。"""
        color = self._ANSI_COLOR[state]
        padded_plain = pad(label, width)
        return f'{color}{label}\033[0m{padded_plain[len(label):]}'

    def _freq_cell_text(self, agent, key, state):
        """点云/SLAM里程计/规划器这三项用实际Hz数值代替"正常/掉线"文字——
        2026-08-12用户明确要求监控这三项的输出频率，不只是"活着/掉线"这个
        二元状态。na/wait两种状态没有意义的Hz数字可看，仍然用文字；
        ok/fail两种状态下都显示实测Hz(fail时也显示，通常是0.0，比单纯一个
        "掉线"文字更能看出"是完全没数据"还是"只是采样窗口还没攒够")。"""
        if state in ('na', 'wait'):
            return self._HEALTH_STATE_LABEL[state]
        return f"{self._hz_for(agent, key):5.1f} Hz"

    def _format_health_table(self, health):
        """另起一张表格（不是加到主表格那几列里，主表格是位置/姿态这些
        连续数值，健康位是离散的正常/掉线/启动中/不监控四态，混在一起
        列宽很难两边都好看）——2026-08-09用户明确要求"用另一个表格加颜色
        表示，而不仅仅在状态栏显示"。2026-08-12：点云/SLAM里程计/规划器
        三列改成显示实测Hz(见_freq_cell_text)，板外控制器/MAVROS链路两列
        没有"频率"这个概念，继续用状态文字。"""
        cols = [
            ("飞机", 6), ("点云频率", 10), ("SLAM里程计频率", 16),
            ("规划器频率", 12), ("板外控制器", 10), ("MAVROS链路", 10),
        ]
        keys = ['lidar', 'slam', 'plan', 'ctrl', 'link']
        sep = "+" + "+".join("-" * (w + 2) for _, w in cols) + "+"
        header = "|" + "|".join(f" {pad(name, w)} " for name, w in cols) + "|"

        lines = [sep, header, sep]
        for ns in self.namespaces:
            agent = self.agents[ns]
            cells = [pad(ns, cols[0][1])]
            for key, (_name, w) in zip(keys, cols[1:]):
                state = health.get((ns, key), 'wait')
                if key in ('lidar', 'slam', 'plan'):
                    text = self._freq_cell_text(agent, key, state)
                else:
                    text = self._HEALTH_STATE_LABEL[state]
                cells.append(self._colored_pad(text, w, state))
            lines.append("| " + " | ".join(cells) + " |")
        lines.append(sep)

        if self.uwb_pair is not None:
            uwb_state = health.get((None, 'uwb'), 'wait')
            uwb_label = self._HEALTH_STATE_LABEL[uwb_state]
            lines.append(f"UWB双机坐标系对齐({self.uwb_pair[0]}<->{self.uwb_pair[1]}): "
                          f"{self._ANSI_COLOR[uwb_state]}{uwb_label}\033[0m")
        return lines

    def _format_calibration_table(self):
        """情景二SE(2)在线旋转估计θ*的实时展示——2026-08-12用户明确要求
        "应该在tmux的status终端专门开辟一块区域，实时显示θ标定结果"（之前
        θ*只能靠ros2 topic echo碰运气去看，样本不够时这两个话题从来没
        发布过，"看不到输出"跟"真的卡住了"从外面完全分不清，这次NaN卡死
        的bug就是这么发现的）。"""
        cols = [("飞机", 6), ("θ*估计", 10), ("样本段数", 10), ("标定状态", 14)]
        sep = "+" + "+".join("-" * (w + 2) for _, w in cols) + "+"
        header = "|" + "|".join(f" {pad(name, w)} " for name, w in cols) + "|"

        lines = [sep, header, sep]
        for ns in self.namespaces:
            agent = self.agents[ns]
            if agent.yaw_sample_count is None:
                theta_str, count_str = "  -  ", "0/%d" % ROTATION_MIN_SEGMENTS
                status, state = "未标定(样本不足)", 'wait'
            else:
                theta_deg = math.degrees(agent.yaw_estimate_rad) if agent.yaw_estimate_rad is not None else 0.0
                theta_str = f"{theta_deg:+6.1f}°"
                count_str = f"{agent.yaw_sample_count}段"
                status, state = "已标定(持续刷新)", 'ok'
            cells = [
                pad(ns, cols[0][1]), pad(theta_str, cols[1][1]),
                pad(count_str, cols[2][1]), self._colored_pad(status, cols[3][1], state),
            ]
            lines.append("| " + " | ".join(cells) + " |")
        lines.append(sep)
        return lines

    def _read_container_stats(self):
        """返回(rows, gpu_line, stale)。rows是[(display_name,cpu,mem,netio)]，
        gpu_line是nvidia-smi那行原始CSV文本(或None，没采集到时)，
        stale=True表示文件不存在/太久没更新(采集脚本没在跑或卡住)——不能把
        陈旧数据当新数据显示，同一个坑`render_tmux_status.sh`已经防过一次。"""
        try:
            mtime = os.path.getmtime(CONTAINER_STATS_FILE)
        except OSError:
            return [], None, True
        if time.time() - mtime > CONTAINER_STATS_STALE_SEC:
            return [], None, True
        try:
            with open(CONTAINER_STATS_FILE) as f:
                raw_lines = f.read().splitlines()
        except OSError:
            return [], None, True

        rows = []
        gpu_line = None
        in_gpu = False
        for line in raw_lines:
            if line.strip() == '---GPU---':
                in_gpu = True
                continue
            if in_gpu:
                if line.strip():
                    gpu_line = line.strip()
                continue
            parts = line.split('|')
            if len(parts) != 4:
                continue
            name, cpu, mem, netio = parts
            rows.append((CONTAINER_DISPLAY_NAMES.get(name, name), cpu, mem, netio))
        return rows, gpu_line, False

    def _format_resource_table(self):
        """三个容器的CPU/内存/网络I/O(docker stats) + GPU利用率(nvidia-smi)，
        数据来自宿主机侧collect_container_stats.sh每秒写的CONTAINER_STATS_FILE
        ——容器内部因为PID/cgroup namespace隔离，看不到"隔壁容器"用了多少
        资源，这些数据只能在宿主机侧采集，见该脚本文件头说明。GPU是整机
        层面的数值(这套仿真只有sim-world容器实际用GPU，nvidia-smi本身也不
        按容器拆分利用率)，只填在sim-world那一行，其它行留空自己说明"不
        适用"，不额外写说明文字。"""
        cols = [("容器", 10), ("CPU", 8), ("内存", 20), ("网络I/O", 22), ("GPU", 22)]
        sep = "+" + "+".join("-" * (w + 2) for _, w in cols) + "+"
        header = "|" + "|".join(f" {pad(name, w)} " for name, w in cols) + "|"
        lines = [sep, header, sep]

        rows, gpu_line, stale = self._read_container_stats()
        if stale or not rows:
            cells = [pad('-', cols[0][1])] + [pad('采集中...', w) for _, w in cols[1:]]
            lines.append("| " + " | ".join(cells) + " |")
            lines.append(sep)
            return lines

        gpu_display = '-'
        if gpu_line:
            gp = [x.strip() for x in gpu_line.split(',')]
            if len(gp) == 4:
                util_gpu, _util_mem, mem_used, mem_total = gp
                gpu_display = f"{util_gpu}%util {mem_used}/{mem_total}MiB"

        rows_by_name = {r[0]: r for r in rows}
        for name in ('sim-world', 'NX01', 'NX02'):
            r = rows_by_name.get(name)
            if r is None:
                cells = [pad(name, cols[0][1])] + [pad('未采集到', w) for _, w in cols[1:]]
            else:
                _, cpu, mem, netio = r
                gpu_cell = gpu_display if name == 'sim-world' else '-'
                cells = [
                    pad(name, cols[0][1]), pad(cpu, cols[1][1]),
                    pad(mem, cols[2][1]), pad(netio, cols[3][1]), pad(gpu_cell, cols[4][1]),
                ]
            lines.append("| " + " | ".join(cells) + " |")
        lines.append(sep)
        return lines

    def _write_health_file(self, bar_text):
        # 原子写：先写临时文件再rename，避免tmux那边(render_tmux_status.sh
        # 每秒cat一次)读到写了一半的内容——同目录内rename在Linux上是原子
        # 操作，不会有这个问题。
        tmp_path = HEALTH_FILE + '.tmp'
        try:
            with open(tmp_path, 'w') as f:
                f.write(bar_text + '\n')
            os.replace(tmp_path, HEALTH_FILE)
        except OSError:
            pass  # /logs没挂载或没权限时静默跳过，不影响这个脚本本身的表格显示

    def _update_health(self):
        health = self._compute_health()
        self._log_health_transitions(dict(health))  # 传副本，prev_health内部会被整体替换
        self._write_health_file(self._format_status_bar())
        return health

    def _format_flight_table(self):
        cols = [
            ("飞机", 6), ("局部坐标 x,y,z (m)", 22), ("全局坐标 x,y,z (m)", 22),
            ("姿态 r,p,y (deg)", 22), ("推力/油门", 9), ("发布频率", 8),
        ]
        sep = "+" + "+".join("-" * (w + 2) for _, w in cols) + "+"
        header = "|" + "|".join(f" {pad(name, w)} " for name, w in cols) + "|"

        lines = [sep, header, sep]
        for ns in self.namespaces:
            ns_, pos_local, pos_world, rpy, thrust, hz = self._row(ns)
            row = (
                f"| {pad(ns_, 6)} | {pad(pos_local, 22)} | {pad(pos_world, 22)} | "
                f"{pad(rpy, 22)} | {pad(thrust, 9)} | {pad(hz, 8)} |"
            )
            lines.append(row)
        lines.append(sep)
        return lines

    def _redraw(self):
        health = self._update_health()

        # 2026-08-12用户明确要求：整个页面只保留"1 飞行参数"/"2 空间对齐"/
        # "3 子系统健康状态"/"4 资源消耗"这4张表格，每张表格只有标题+表格
        # 主体，不再有配置横幅(规划器/控制器/定位方式)、内存占用、刷新
        # 时间、各表格下面的文字说明——这些说明性文字原来是为了方便新用户
        # 理解才加的，用户现在明确不需要，看表格本身就够了。
        lines = []
        lines.append("1 飞行参数")
        lines.extend(self._format_flight_table())
        lines.append("")
        lines.append("2 空间对齐")
        lines.extend(self._format_calibration_table())
        lines.append("")
        lines.append("3 子系统健康状态")
        lines.extend(self._format_health_table(health))
        lines.append("")
        lines.append("4 资源消耗")
        lines.extend(self._format_resource_table())

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
