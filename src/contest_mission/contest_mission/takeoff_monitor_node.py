#!/usr/bin/env python3
"""起飞 Action server：把"起飞完成判定"从选手侧下沉到机载。

**为什么要有这个节点**（2026-09-20新增，依据见《模块运行位置与消息流转
设计表.docx》第十五章"从时序推导代码归属"）：

原来`sdk.takeoff()`的做法是——选手容器发布一条`TakeoffLand{TAKEOFF}`到
`takeoff_land`话题，然后**在选手侧**轮询`mavros/state`的`armed`和
`dlio/odom_node/odom`的位置，自己判断"起飞完成"。这个判定本身是一个
采样周期远小于200毫秒的回路，而且依赖里程计——真机部署时选手程序可能
跑在地面站电脑上，里程计要过WiFi、且常用BEST_EFFORT QoS，链路一抖就
可能误判成"还没起来"而超时，实际上飞机好好地悬停着。

按"回路必须完整待在机载"这条判据，判定应该下沉。本节点承担这段判定：
收到goal后自己发`takeoff_land`、自己看`armed`和位置、自己判完成，把
结论作为action result返回。选手侧从"自己算"变成"等结论"，链路延迟只
影响拿到结论的快慢，不影响判定本身的正确性。

**为什么用Action而不是"参数触发+状态话题"**（两种都能实现下沉，这次
选了前者）：起飞耗时十几到几十秒、有真实失败模式、且存在真实的取消
需求（现在起飞指令发出后收不回来）。Action天然提供goal确认、带类型的
终态、cancel、feedback和标准状态机，且`ros2 action send_goal`可直接
调试；代价只是`quadrotor_msgs`多一个接口定义（4行改动），`contestant-sdk`
镜像本来就从flight-stack拷这个包的安装产物，SDK侧自动拿得到。

**跟控制器的关系：不改任何控制器，一份实现通吃三个**。本节点对下仍然
发`takeoff_land`话题，`pt4ctrl`/`px4ctrl`/`so3ctrl`都订阅它（三者共享
`vendor/px4ctrl_ros2`里同一套FSM结构，`MANUAL_CTRL -> AUTO_TAKEOFF`
都由这条话题触发），所以不需要为每个控制器各写一份server。
⚠️ 例外：`CONTROLLER=ros2_px4_stack`那条路线没有`takeoff_land`话题
（它走`takeoff_and_track_trajectory`那套），本节点在该组合下不工作——
但这是既有约束，不是本次引入的：现在的`sdk.takeoff()`同样是发这条
话题，在该组合下一样起不了飞。将来若要支持，应在本节点内加一个后端
分支（参数选controller_kind），而不是再写第二个server。

**判定逻辑跟原来SDK里那段逐行对应**（含2026-09-14踩过的那个坑，见下面
`_pos_stable`里的说明），不是重新发明的：
0. **起飞前就绪判定**（2026-09-21新增，见`preflight_ready()`）：飞控已
   连接、里程计已收到、合速度连续`static_hold_s`秒低于
   `static_speed_max_mps`，三条都满足才下发起飞指令。**收到goal不等于
   立刻起飞**——飞机刚上电/刚重启时定位源还没收敛，这时候发指令只会被
   控制器按"非静止起飞"拒掉（实测NX02里程计速度0.51m/s被拒），或者在
   定位发散的状态下起飞；
1. 等`armed=True`（期间按`takeoff_retry_interval_s`重发起飞指令兜底，
   防止指令落在控制器状态机切换的缝隙里）；
2. 记下armed那一刻的高度作基线，等位置连续`stable_window_s`秒都落在
   `pos_tolerance_m`容差内，且相对基线至少爬升了`min_climb_m`
   （"停在地面不动"不算数）；
3. 再额外强制悬停`hover_after_s`秒，给控制器收敛裕量，然后才算完成。

失败原因按阶段分开报（result.stage = `preflight`/`armed`/`stable`），这是
下沉带来的另一个收益——原来选手侧只能笼统超时，分不清是"没准备好"、
"没解锁"还是"解锁了爬升不到位"，这三种情况的处置完全不同。

`goal.timeout_s`只覆盖第1~3步，从就绪那一刻起算；等就绪用它自己的
`preflight_timeout_s`（默认90秒），因为定位收敛可能比起飞动作本身还慢。

**这个节点不做的事**：声光反馈、偏航模式设置仍然留在SDK里——那些是
决策/提示，不是判定回路，放在选手侧没有问题，也方便选手自己替换。

**多写者提醒**：`takeoff_land`这条话题上同时还有别的发布者——
`takeoff_gate`（读容器本地`/tmp/takeoff_go`）、GCS仿真卡片（后端
`docker exec`连发3次）、GCS真机面板（前端rosbridge连发3次）。本节点
订阅这条话题，发现不是自己发出的TAKEOFF就记一笔，并在result.message
里标注，至少保证事后可追溯（见《设计表》第十二章12.4）。
"""
import math
import threading
import time
from typing import List, Optional, Tuple

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.action import Takeoff
from quadrotor_msgs.msg import TakeoffLand

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

#: 判定回路的采样周期。原来在SDK侧是`_poll_until`的轮询间隔，这里10Hz
#: 足够（里程计本身也就几十Hz），同时也是feedback的发布节奏。
_TICK_PERIOD_S = 0.1


def mean_speed_vector(samples: List[Tuple[float, float, float]]) -> float:
    """一批速度**矢量**的平均值的模长（米/秒）。

    2026-09-21（用户："等待preflight时间太长"）：原来判"静止"用的是
    **瞬时速度大小** `|v|` 连续若干秒都低于门限。这个统计量选错了——

    `|v|` 是模长、恒为正，所以零均值的速度噪声取模之后均值**不是 0**。
    实测 NX02 停在地上时 `|v|` 的采样是 0.26/0.13/0.12/0.11/0.10/0.09/
    0.08/0.068/0.04，均值约 0.11，比 0.08 的门限还高。要求连续 3 秒每一
    帧都低于门限，等于要连中十几次，概率很低，所以一等就是几十秒。对
    `|v|` 取平均同样解决不了，因为均值本身就超标。

    对**矢量**求平均再取模就正确了：飞机真静止时三轴噪声零均值、互相
    抵消，平均矢量趋近 0；飞机真在动时，平均矢量就是它的真实速度。
    """
    if not samples:
        return float('inf')
    n = float(len(samples))
    mx = sum(v[0] for v in samples) / n
    my = sum(v[1] for v in samples) / n
    mz = sum(v[2] for v in samples) / n
    return math.sqrt(mx * mx + my * my + mz * mz)


def preflight_ready(
    connected: Optional[bool],
    speed: Optional[float],
    vel_samples: Optional[List[Tuple[float, float, float]]],
    window_full: bool,
    static_speed_max: float,
    instant_speed_max: float,
) -> Tuple[bool, str]:
    """起飞前就绪判定：返回(是否可以发起飞指令, 不可以的原因)。

    2026-09-21（用户指出"不能飞机一出世就给起飞命令"）：原来的实现是
    收到goal就立刻发`TakeoffLand{TAKEOFF}`，控制器那边按
    `odom_data.v.norm() > 0.1`判"非静止起飞"直接拒绝
    （`vendor/px4ctrl_ros2/px4ctrl/src/PX4CtrlFSM.cpp`），实测NX02刚
    重启、uwb_imu还没收敛时里程计速度0.51m/s，一条指令被拒之后没有
    任何人再发，就干等到`armed`超时。

    改成三条前置条件都满足才发：
    1. `mavros/state`收到过且`connected=True`——飞控链路通了；
    2. 里程计收到过——控制器进AUTO_TAKEOFF本身要求有odom；
    3. 合速度连续`static_hold_s`秒低于`static_speed_max`——定位源收敛
       且飞机确实静止。只看瞬时一帧不够，刚启动时抖动会偶然抽到低值。

    `static_since`是"最近一次开始持续静止的时刻"，由调用方维护（速度
    一旦超门限就清成None）。
    """
    if connected is not True:
        return False, '飞控未连接（mavros/state 还没有 connected=True）'
    if speed is None or not vel_samples:
        return False, '还没收到里程计数据'
    if not window_full:
        return False, '正在积累里程计采样窗口'
    mean_v = mean_speed_vector(vel_samples)
    if mean_v > static_speed_max:
        return False, (f'里程计速度矢量均值{mean_v:.3f}m/s > {static_speed_max:.2f}m/s，'
                       f'飞机还没静止或定位源还没收敛')
    # 平均值达标之后还要看这一帧的瞬时值：控制器按瞬时 odom_data.v.norm()
    # 判"非静止起飞"，指令发出去的那一刻瞬时值超标一样会被拒。
    if speed > instant_speed_max:
        return False, (f'均值已达标，但这一帧瞬时速度{speed:.2f}m/s > '
                       f'{instant_speed_max:.2f}m/s，等下一帧再发')
    return True, ''


class TakeoffMonitorNode(Node):
    def __init__(self):
        super().__init__('takeoff_monitor_node')

        # 判定参数，默认值跟SDK里原来的常量逐一对应
        self.declare_parameter('stable_window_s', 1.5)        # TAKEOFF_STABLE_WINDOW_S
        self.declare_parameter('min_climb_m', 0.6)            # TAKEOFF_MIN_CLIMB_M
        self.declare_parameter('pos_tolerance_m', 0.3)        # TAKEOFF_STABLE_POS_TOLERANCE_M
        self.declare_parameter('hover_after_s', 5.0)          # HOVER_AFTER_TAKEOFF_S
        self.declare_parameter('default_timeout_s', 60.0)     # goal.timeout_s<=0时用这个
        # 等armed这一段的单独超时。
        #
        # 2026-09-21：一度放宽到120秒，因为仿真容器刚起来时 PX4 自检还没过
        # （`ARM rejected by PX4!`，实测约40秒后才接受解锁）。但那是在拿
        # 超时兜一个本该在启动阶段解决的问题——现在启动脚本会等 PX4 自己报
        # `Ready for takeoff!` 之后才放任务程序进来，到这里 PX4 一定已经
        # 可以解锁了，所以收回到 45 秒。留这点余量是给"指令落在控制器状态机
        # 切换缝隙里"这类偶发情况重发用的，不是给启动窗口用的。
        self.declare_parameter('armed_timeout_s', 45.0)

        # "已在空中"判定阈值（2026-09-20实测补上）：收到goal时如果飞机
        # 已经解锁且高度超过这个值，直接返回成功，不重复下发起飞指令。
        #
        # 为什么需要：原来（以及改造前的SDK判定）在飞机已经悬停时再调一次
        # takeoff()，爬升基线`z_at_armed`会取成当前悬停高度，"相对基线再爬
        # 升min_climb_m"这个条件永远不成立，于是白白等到超时才报错——实测
        # 复现过。正确行为是立刻返回成功。
        #
        # 阈值取odom坐标系下的绝对高度，不是AGL：实测地面静止时z≈0.33m、
        # 悬停时z≈0.9~1.3m（见DEBUG_JOURNAL 2026-09-20测试记录），0.5m
        # 落在两者中间。不同场地/定位源下这个数会变，所以做成参数。
        self.declare_parameter('already_airborne_z_m', 0.5)

        # 2026-09-21：起飞指令要重发，不能只发一次。实测NX02刚重启、
        # uwb_imu融合还没收敛时，里程计瞬时速度有0.51m/s，px4ctrl直接
        # `Reject AUTO_TAKEOFF. Odom_Vel=0.512840m/s, non-static takeoff
        # is not allowed!`——一条指令被拒之后没有任何人再发，本节点就
        # 干等到`armed`超时。控制器那边阈值写死在
        # `vendor/px4ctrl_ros2/px4ctrl/src/PX4CtrlFSM.cpp`的
        # `odom_data.v.norm() > 0.1`，这里的默认值取得比它更严一点，
        # 留一点"发出去到控制器处理"之间的余量。
        self.declare_parameter('takeoff_retry_interval_s', 3.0)
        self.declare_parameter('static_speed_max_mps', 0.08)
        # 持续静止时长：瞬时一帧低于门限不算收敛（uwb_imu刚启动时里程计
        # 速度会来回抖，能抽到低值的瞬间），要求连续`static_hold_s`秒都
        # 低于门限才认为定位源真的收敛了、飞机真的停着。
        # 求平均用的采样窗口长度（秒）。窗口越长噪声压得越干净，但也越晚
        # 能起飞；1.5 秒在 30Hz 里程计下是 45 个样本，足够把零均值噪声压掉。
        self.declare_parameter('static_hold_s', 1.5)
        # 发指令那一刻允许的**瞬时**速度上限。控制器自己的门限是 0.1，
        # 这里留一点"发出去到控制器处理"之间的余量。
        self.declare_parameter('instant_speed_max_mps', 0.09)
        # 2026-09-21 从 90 放宽到 150：实测 NX02 停在地上时里程计合速度就在
        # 0.08~0.14m/s 之间抖（控制器自己的拒绝门限是 0.1），凑够连续 3 秒
        # 低于 0.08 需要碰运气，90 秒偏紧。放宽只影响"最坏情况等多久才
        # 报失败"，不影响正常路径——定位收敛得快就立刻起飞。
        # 2026-09-21：一度放宽到150秒，那是判据用错统计量（对速度**大小**
        # 取平均，均值恒高于门限）时的兜底。改成速度矢量窗口平均之后，实测
        # 1.5秒就通过，这个值收回到 45 秒——它现在只覆盖"定位源真的迟迟不
        # 收敛"这种异常，不再是正常路径的一部分。
        self.declare_parameter('preflight_timeout_s', 45.0)

        self._armed: Optional[bool] = None
        self._odom_xyz: Optional[Tuple[float, float, float]] = None
        self._odom_speed: Optional[float] = None
        self._odom_vel: Optional[Tuple[float, float, float]] = None
        self._connected: Optional[bool] = None
        self._busy = threading.Lock()
        self._external_takeoff_seen = False
        self._own_publish_count = 0

        # 回调组用Reentrant + MultiThreadedExecutor：execute_callback里要
        # 长时间循环等待，同时订阅回调必须继续更新armed/odom，单线程
        # executor会把自己饿死（跟SDK侧_rclpy_runtime选MultiThreadedExecutor
        # 是同一个理由，也跟scenario_reset_node那次service死锁同源）。
        cb_group = ReentrantCallbackGroup()

        self.takeoff_land_pub = self.create_publisher(TakeoffLand, 'takeoff_land', 10)
        self.create_subscription(
            State, 'mavros/state', self._on_state, qos_profile_sensor_data, callback_group=cb_group
        )
        self.create_subscription(
            Odometry, 'dlio/odom_node/odom', self._on_odom, qos_profile_sensor_data,
            callback_group=cb_group,
        )
        # 多写者监测：这条话题上还有takeoff_gate/GCS两路发布者，见文件头
        self.create_subscription(
            TakeoffLand, 'takeoff_land', self._on_takeoff_land, 10, callback_group=cb_group
        )

        self._server = ActionServer(
            self, Takeoff, 'takeoff',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb_group,
        )

        self.get_logger().info(
            'takeoff_monitor_node就绪：起飞完成判定已下沉到机载，action名 "takeoff"。'
            '调试可用 ros2 action send_goal /<ns>/takeoff quadrotor_msgs/action/Takeoff "{timeout_s: 60.0}"'
        )

    # ---------------- 订阅回调 ----------------

    def _on_state(self, msg) -> None:
        self._armed = bool(msg.armed)
        self._connected = bool(msg.connected)

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        self._odom_xyz = (p.x, p.y, p.z)
        v = msg.twist.twist.linear
        # 存**矢量**而不只是模长——判"静止"要对矢量做平均再取模，见
        # `mean_speed_vector()` 的说明。模长仍然保留一份，因为发指令那一刻
        # 要跟 px4ctrl 的瞬时判据对齐。
        self._odom_vel = (v.x, v.y, v.z)
        self._odom_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _on_takeoff_land(self, msg) -> None:
        """监测这条话题上的外部发布者（takeoff_gate / GCS 两路）。

        自己发的那几帧会先把`_own_publish_count`加上，这里逐帧抵扣；
        抵扣不掉的就是别人发的。只做记录，不做拦截——本节点没有、也不
        应该有独占起飞权（要独占得改控制器只认本节点的指令，那要动
        vendor的C++，见文件头）。
        """
        if msg.takeoff_land_cmd != TakeoffLand.TAKEOFF:
            return
        if self._own_publish_count > 0:
            self._own_publish_count -= 1
            return
        self._external_takeoff_seen = True
        self.get_logger().warn('检测到外部发布的TakeoffLand{TAKEOFF}（takeoff_gate或GCS），已记录')

    # ---------------- Action 回调 ----------------

    def _on_goal(self, goal_request) -> GoalResponse:
        if self._busy.locked():
            self.get_logger().warn('已有起飞流程在执行，拒绝新的goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('收到取消请求，将中止起飞判定')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._busy:
            return self._run(goal_handle)

    def _run(self, goal_handle):
        timeout_s = float(goal_handle.request.timeout_s)
        if timeout_s <= 0.0:
            timeout_s = float(self.get_parameter('default_timeout_s').value)
        armed_timeout_s = float(self.get_parameter('armed_timeout_s').value)

        self._external_takeoff_seen = False
        history: List[Tuple[float, Tuple[float, float, float]]] = []
        first_sample_time: Optional[float] = None
        z_at_armed: Optional[float] = None

        # 已经在空中了就直接返回成功，不重复下发起飞指令（见
        # `already_airborne_z_m` 参数上面的说明）。注意这里**不发**
        # TakeoffLand：飞机正在 AUTO_HOVER，再塞一条 TAKEOFF 进去只会
        # 扰动控制器状态机，没有任何好处。
        if self._already_airborne():
            z = self._odom_xyz[2] if self._odom_xyz is not None else 0.0
            goal_handle.succeed()
            return self._result(
                True, 'already_airborne',
                f'飞机已解锁且高度{z:.2f}m，判定为已在空中，未重复下发起飞指令'
            )

        # 发起飞指令。控制器（pt4ctrl/px4ctrl/so3ctrl）订阅这条话题，
        # MANUAL_CTRL -> AUTO_TAKEOFF 由它触发，本节点不碰控制器。
        retry_interval_s = float(self.get_parameter('takeoff_retry_interval_s').value)
        static_speed_max = float(self.get_parameter('static_speed_max_mps').value)
        static_hold_s = float(self.get_parameter('static_hold_s').value)
        instant_speed_max = float(self.get_parameter('instant_speed_max_mps').value)
        vel_samples: List[Tuple[float, float, float]] = []
        window_start: Optional[float] = None
        preflight_timeout_s = float(self.get_parameter('preflight_timeout_s').value)

        started = time.monotonic()
        # 从 preflight 开始，不是一上来就喊起飞——见 preflight_ready()
        # 的docstring（2026-09-21用户指出的问题）。
        phase = 'preflight'
        phase_start = started
        last_cmd_t = 0.0
        attempts = 0

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, phase, '调用方取消了起飞')
            now = time.monotonic()
            # 总超时只管起飞动作本身（第1~3步），不管 preflight——preflight
            # 等的是定位源收敛，有自己的 preflight_timeout_s。
            #
            # 2026-09-21 追加：`wait_armed` 同理也排除在外。等 PX4 自己变得
            # 可解锁，跟等定位收敛是同一类"等飞机就绪"，不属于起飞动作本身。
            # 实测：仿真容器刚起来时 PX4 会 `ARM rejected by PX4!`（自检还
            # 没过），约40秒后才接受解锁；`armed_timeout_s` 放宽到120秒但
            # 总超时60秒仍会先到期，所以两处必须一起改。
            # 注：试过用 mavros/state 的 system_status 当"可解锁"信号，实测
            # 它全程停在 0(MAV_STATE_UNINIT)，这套 PX4 SITL 没填这个字段，
            # 用不了，只能靠重试等。
            #
            # 2026-09-21 修：原来这个检查在 preflight 期间也在跑，于是
            # preflight 实际只有 `goal.timeout_s`（客户端默认60秒）这么长，
            # 而不是 preflight_timeout_s（90秒），下面那句 started = now 形同
            # 摆设。实测 NX02 地面里程计噪声卡在 0.08~0.11m/s 边界，60 秒内
            # 没凑够 3 秒静止窗口就被判 `stage=preflight 总超时`，起飞失败。
            if phase not in ('preflight', 'wait_armed') and now - started > timeout_s:
                goal_handle.abort()
                return self._result(False, phase, f'总超时（{timeout_s:.0f}秒）仍未完成，卡在阶段{phase}')

            self._publish_feedback(goal_handle, phase, z_at_armed)

            # 维护速度矢量的滑动采样窗口。跟原来"速度一超门限就重新计时"
            # 不同——矢量平均本来就能容忍个别超标样本，不需要清零重来，
            # 这也是等待时间能从几十秒降到几秒的原因。
            speed = self._odom_speed
            if self._odom_vel is not None:
                if window_start is None:
                    window_start = now
                vel_samples.append(self._odom_vel)
                # 按时间长度裁剪窗口：保留最近 static_hold_s 秒的样本
                max_samples = max(1, int(static_hold_s / _TICK_PERIOD_S))
                if len(vel_samples) > max_samples:
                    del vel_samples[:len(vel_samples) - max_samples]
            window_full = (window_start is not None
                           and now - window_start >= static_hold_s)

            if phase == 'preflight':
                ready, why = preflight_ready(
                    self._connected, speed, vel_samples, window_full,
                    static_speed_max, instant_speed_max,
                )
                if ready:
                    self.get_logger().info(
                        f'起飞前就绪：飞控已连接、最近{static_hold_s:.1f}秒里程计'
                        f'速度矢量均值{mean_speed_vector(vel_samples):.3f}m/s'
                        f'（瞬时{speed:.3f}m/s），现在下发起飞指令'
                    )
                    phase, phase_start = 'wait_armed', now
                    last_cmd_t = now - retry_interval_s   # 让下一圈立刻发
                    # 总超时从"真正开始起飞"起算，不把等就绪的时间算进去
                    # ——`timeout_s`是给起飞动作本身的预算（SDK默认60秒），
                    # 等定位收敛可能比这还久，两者用各自的超时。
                    started = now
                    continue
                if now - phase_start > preflight_timeout_s:
                    goal_handle.abort()
                    return self._result(
                        False, 'preflight',
                        f'起飞前就绪判定超时（{preflight_timeout_s:.0f}秒）：{why}。'
                        f'没有下发任何起飞指令——飞机没准备好就发指令只会被控制器'
                        f'拒绝（非静止起飞），或者在定位发散的状态下起飞。'
                    )
                self.get_logger().info(f'等待起飞前就绪：{why}', throttle_duration_sec=3.0)

            elif phase == 'wait_armed':
                if self._armed is not True and now - last_cmd_t >= retry_interval_s:
                    # 就绪之后才会走到这里。仍然保留重发：指令有可能在
                    # 控制器状态机切换的缝隙里被丢/被拒，重发是兜底。
                    msg = TakeoffLand()
                    msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
                    self._own_publish_count += 1
                    self.takeoff_land_pub.publish(msg)
                    last_cmd_t = now
                    attempts += 1
                    self.get_logger().info(
                        f'已发布TakeoffLand{{TAKEOFF}}（第{attempts}次，'
                        f'里程计速度{speed if speed is None else round(speed, 3)}m/s）'
                    )

                if self._armed is True:
                    # armed那一刻的高度作为爬升基线；还没收到里程计时用
                    # 0.0占位（防御性兜底，AUTO_TAKEOFF本身要求先有odom
                    # 才会真的开始爬升，理论上走不到这个分支）。
                    z_at_armed = self._odom_xyz[2] if self._odom_xyz is not None else 0.0
                    phase, phase_start = 'wait_stable', now
                    started = now      # 总超时从"真正开始爬升"起算，见上面说明
                    self.get_logger().info(f'已解锁，爬升基线z={z_at_armed:.2f}m，等待爬升到位+稳定')
                elif now - phase_start > armed_timeout_s:
                    goal_handle.abort()
                    return self._result(
                        False, 'armed',
                        f'等待解锁超时（{armed_timeout_s:.0f}秒）：飞控一直没有armed=True，'
                        f'共发出{attempts}次起飞指令，最后一次里程计速度'
                        f'{self._odom_speed if self._odom_speed is None else round(self._odom_speed, 2)}m/s。'
                        f'常见原因：①定位源没收敛导致控制器拒绝非静止起飞'
                        f'（控制器阈值0.1m/s）②飞控未连接③解锁前置条件未满足'
                    )

            elif phase == 'wait_stable':
                stable, history, first_sample_time = self._pos_stable(
                    history, first_sample_time, z_at_armed
                )
                if stable:
                    phase, phase_start = 'hover', now
                    self.get_logger().info('爬升到位且位置已稳定，进入起飞后强制悬停')

            elif phase == 'hover':
                # 额外强制悬停，给控制器收敛裕量再把控制权交还调用方
                # （对应SDK里原来HOVER_AFTER_TAKEOFF_S那段time.sleep）。
                if now - phase_start >= float(self.get_parameter('hover_after_s').value):
                    goal_handle.succeed()
                    note = '；期间检测到外部发布的起飞指令' if self._external_takeoff_seen else ''
                    return self._result(True, 'done', f'起飞完成，已稳定悬停{note}')

            time.sleep(_TICK_PERIOD_S)

    # ---------------- 判定 ----------------

    def _already_airborne(self) -> bool:
        """飞机是不是已经解锁并且在空中了。

        两个条件都要满足：`armed=True`，且里程计高度超过
        `already_airborne_z_m`。只看 armed 不够——地面上解锁待命（电机
        转了但没起飞）也是 armed，那种情况必须继续走正常起飞流程。
        """
        if self._armed is not True or self._odom_xyz is None:
            return False
        return self._odom_xyz[2] >= float(self.get_parameter('already_airborne_z_m').value)

    def _pos_stable(self, history, first_sample_time, z_at_armed):
        """位置稳定窗口检测。逻辑跟SDK里原来那段逐行对应，包括那个坑。

        ⚠️ 2026-09-14踩过的坑（原样保留这个写法）：不能用trim之后剩下的
        `history[0]`去判断"窗口是否攒够时长"——trim本身就是"只保留
        age<=window的条目"，剩下的最老条目age必然满足条件，那个判断会
        永远为真、函数永远提前退出，导致判定永远不可能成立（实测表现为
        takeoff稳定卡在60秒超时）。所以额外单独记一个`first_sample_time`
        （只在第一次采样时赋值一次，不随trim变化），两件事分开判断。
        """
        if self._odom_xyz is None:
            return False, history, first_sample_time
        window_s = float(self.get_parameter('stable_window_s').value)
        now = time.monotonic()
        if first_sample_time is None:
            first_sample_time = now
        history.append((now, self._odom_xyz))
        while history and now - history[0][0] > window_s:
            history.pop(0)
        if now - first_sample_time < window_s:
            return False, history, first_sample_time  # 还没攒满一个窗口时长
        if self._odom_xyz[2] - (z_at_armed or 0.0) < float(self.get_parameter('min_climb_m').value):
            return False, history, first_sample_time  # "停在地面不动"不算数
        xs = [p[0] for _, p in history]
        ys = [p[1] for _, p in history]
        zs = [p[2] for _, p in history]
        spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        ok = spread <= float(self.get_parameter('pos_tolerance_m').value)
        return ok, history, first_sample_time

    # ---------------- 辅助 ----------------

    def _publish_feedback(self, goal_handle, phase: str, z_at_armed: Optional[float]) -> None:
        fb = Takeoff.Feedback()
        fb.phase = phase
        fb.armed = bool(self._armed)
        z = self._odom_xyz[2] if self._odom_xyz is not None else 0.0
        fb.current_z = float(z)
        fb.climb_m = float(z - z_at_armed) if z_at_armed is not None else 0.0
        goal_handle.publish_feedback(fb)

    def _result(self, success: bool, stage: str, message: str) -> Takeoff.Result:
        res = Takeoff.Result()
        res.success = success
        res.stage = stage
        res.message = message
        res.final_z = float(self._odom_xyz[2]) if self._odom_xyz is not None else 0.0
        self.get_logger().info(f'起飞判定结束: success={success} stage={stage} {message}')
        return res


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffMonitorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
