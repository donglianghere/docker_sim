#!/usr/bin/env python3
"""RViz "2D Goal Pose"点击目标点 -> TF变换到这架飞机自己的odom系 -> term_goal。

背景：multi_ego_planner.rviz里的两个"2D Goal Pose"（rviz_default_plugins/
SetGoal）工具之前直接把Topic设成了/NX01/term_goal、/NX02/term_goal，完全
绕开了坐标换算——RViz发布的PoseStamped用的是当前Fixed Frame下的原始点击
坐标，而ego-planner-swarm的ego_replan_fsm.cpp::waypointCallback()直接读
msg->pose.position.x/y、完全不看header.frame_id、也不做任何TF变换，会把
这个原始坐标当成已经是自己局部odom系下的坐标——跟ego_planner_docker_sim.
launch.py文档记录的2026-08-07事故是同一类问题。

2026-08-08第一版这里的修复方案是"假设rviz_goal_world发布的是世界坐标，
减去这架飞机自己的INIT_X偏移得到局部坐标"——这个假设是错的：这套集成的
multi_ego_planner.rviz里Fixed Frame设的是"NX02/odom"（某一架具体飞机自己
的frame，不是一个中立的世界系），RViz的2D Goal Pose工具发布的坐标永远是
"点击位置在当前Fixed Frame下的数值"，不是世界坐标。实测复现：两次点击
`rviz_goal_world`分别是-9.8563/-1.2012，`term_goal`（减INIT_X=6之后）
分别是-15.8563/-7.2012，差值恒为-6——对NX02自己来说这个减法完全是多余的
（原始点击本来就已经是NX02/odom系的数值，跟ego_planner_node自己的
odom_pos_是同一个系，不需要再减一次），对NX01来说更糟（该加offset差值
却在减，符号都反了）。飞机因此飞向了一个偏离用户实际点击位置好几米的
错误坐标，是这一整轮"目标点被判定在障碍物里、但肉眼看明明不在"排查的
真正根因——不是占据栅格错了，是目标点坐标本身就送错了地方。

改成用tf2做真正的坐标变换：不管Fixed Frame设成什么（不用像旧版那样
硬编码假设"一定是世界系"或"一定是某个特定frame"），只要TF树里能查到
从msg.header.frame_id（RViz实际发布时用的、当前Fixed Frame的名字）到
这架飞机自己的"<namespace>/odom"的变换，就能算出正确的局部坐标——
这套项目里NX01/odom<->NX02/odom已经有UWB frame_align+静态TF连通，
两架飞机各自的"2D Goal Pose"工具不管Fixed Frame切换成哪一架飞机的
frame都能正确换算，不用再关心具体是哪个frame、偏移是多少。

2026-08-22新增：航点队列执行器（GCS网页"规划航线"功能的机载侧）。跟
上面这段rviz_goal_world转发逻辑完全独立、共用同一个term_goal发布者，
互不干扰。GCS网页在俯视图上画折线，把整条航点表（已经在浏览器端用
world_to_local()换算成这架飞机自己的局部坐标，跟单点点击发目标点用
的是同一套换算，不需要这个节点再做TF转换）一次性发过来，这里只负责
"按序执行"：发第一个点的term_goal，订阅里程计判断是否进入0.3米阈值，
到点就发下一个，直到发完。取消队列不是简单地"不再发下一个"——无人机
还在朝当前目标点飞、有速度，直接不管它不会让它停下来；也不能把
"当前位置"原样重发一次当term_goal：查过ego_planner的
planGlobalTraj/one_segment_traj_gen（五次多项式轨迹生成，
polynomial_traj.cpp），起点终点距离趋近0、但起点速度不为0又要求终点
速度为0时，六元线性方程组里"终点位置约束"那一行在t→0时会退化成跟
"起点位置约束"那一行完全相同，系数矩阵奇异/病态，解出来的轨迹系数
发散——不是安全刹车，是数值噪声，可能给px4ctrl喂一段发散的
position_cmd。这里的取消逻辑因此改成沿当前速度方向（DLIO在odom.cc里
线速度显式填的是里程计世界系state.v.lin.w，不是REP-103机体系约定，
2026-08-22查过DLIO源码确认，可以直接拿v.x/v.y当odom系分量用，不需要
再用yaw旋转）顺延一小段安全距离（clamp到0.5~3米）当新term_goal，
让距离保持远离0，规划器按正常逻辑生成一段减速轨迹飞过去停住——全程
只是"发一个正常的term_goal"，不碰ego_planner/traj_server核心代码
一行。
⚠️ 2026-09-03新增：目标点持续重变换（论文《基于滑窗最小二乘的局部-全局坐标系
在线标定方法》3.2节提出的修复，见`docker_sim/实验方案_SE2在线标定论文补充
实验.md` L2层改动(d)）。

原来的问题：目标点从操作界面到规划器内部目标是**一次性**坐标变换——收到目标
点的那一刻查一次当时的(θ*,t)算出局部坐标，发出去就固定了，之后θ*再怎么被
在线标定修正，都不会追溯应用到已经发出去的目标点上。实测后果：一次双机对穿
测试里两机最终停留位置系统性偏离预期目标点0.07~0.16米，用"目标点变换完全不做
旋转补偿"这个假说反算出来的预测落点，跟两次独立实测都吻合到厘米级（论文图4）。

修复思路：**把目标点在世界系下的原始坐标记住**，而不是只记换算完的局部坐标；
每隔`goal_retransform_rate_hz`(默认2Hz)重新用当前TF把它换算成局部坐标，跟上次
真正发出去的差超过`goal_retransform_min_delta_m`(默认0.05米)才重发一次
term_goal，直到飞机到达为止。这样θ*每次被修正，目标点都跟着修正。
两道闸门保证它平时是"静默"的：①θ*收敛后局部目标几乎不再变化，差值达不到阈值
就不会重发，不会持续骚扰规划器重规划；②查不到`world -> {ns}/odom`这条TF
（起飞点还没锁定）时直接跳过，行为退回修复之前，不会因为标定没就绪就卡住。
`GOAL_RETRANSFORM=false`可以整个关掉——这是论文E13实验"修复前/修复后"对照组
要用的开关，关掉时行为跟2026-09-03之前逐字节一致。

同一套逻辑同时覆盖两条目标点入口：RViz的2D Goal Pose(`rviz_goal_world`)和GCS
网页的航点队列(`waypoint_queue`)。队列这一路存的也改成**世界坐标**，因为队列里
后面那些还没飞到的航点同样会被θ*的后续修正影响，只修正当前这一个是不够的。

"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty, Int32, String
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
# 必须import这个模块本身（哪怕代码里不直接调用它的函数）——tf2_ros的
# Buffer.transform()是靠类型注册表分发到具体消息类型的do_transform_*
# 实现，import tf2_geometry_msgs的副作用就是把PoseStamped注册进这张表，
# 不import的话Buffer.transform(PoseStamped类型)会直接抛
# "Type ... is not loaded or supported"（2026-08-08现场验证时就是这样
# 崩的，漏掉这一行）。
import tf2_geometry_msgs  # noqa: F401

ARRIVAL_THRESHOLD_M = 0.3   # 航点抵达判定距离，用户指定
BRAKE_LOOKAHEAD_SEC = 1.0   # 取消队列时"沿当前速度方向顺延"的时间量纲
BRAKE_MIN_M = 0.5           # 顺延距离下限——避免速度接近0时顺延距离也
                             # 趋近0，重新撞上"起点终点重合"的数值奇异
BRAKE_MAX_M = 3.0           # 顺延距离上限——避免高速飞行时刹车点甩得
                             # 太远，失去"就近悬停"的意义


class RvizGoalBridgeNode(Node):

    def __init__(self):
        super().__init__('rviz_goal_bridge_node')

        # 节点用namespace=${NAMESPACE}起（跟ego_planner_node同一个约定），
        # get_namespace()返回"/NX01"这种带前导斜杠的形式，拼target frame
        # 时要去掉。
        ns = self.get_namespace().strip('/')
        self.target_frame = f'{ns}/odom' if ns else 'odom'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PoseStamped, 'term_goal', qos)
        self.create_subscription(PoseStamped, 'rviz_goal_world', self._cb, qos)

        # ---- 航点队列执行器 ----
        # 2026-09-03：队列从"局部坐标"改成存**世界坐标**，见文件头说明——
        # 局部坐标会随θ*的在线修正而失效，世界坐标才是操作员真正指定的那个点。
        self._queue = []       # 收到队列那一刻换算好的局部坐标（世界系
                                # 查不到TF时的兜底，行为退回修复之前）
        self._queue_world = []  # 待执行航点 [(wx,wy,wz), ...]（世界系）
        self._queue_idx = -1   # 当前正飞往第几个航点，-1=空闲
        self._latest_odom = None
        # 单点目标(rviz_goal_world那一路)的世界坐标；队列那一路用_queue_world
        self._single_goal_world = None
        self._last_pub_local = None   # 上次真正发出去的局部目标，差值判据用

        self.declare_parameter(
            'goal_retransform_enabled',
            os.environ.get('GOAL_RETRANSFORM', 'true').lower()
            not in ('0', 'false', 'no'))
        self.declare_parameter('goal_retransform_rate_hz', 2.0)
        self.declare_parameter('goal_retransform_min_delta_m', 0.05)
        self._retransform_enabled = self.get_parameter(
            'goal_retransform_enabled').value
        self._retransform_min_delta = self.get_parameter(
            'goal_retransform_min_delta_m').value
        _rate = float(self.get_parameter('goal_retransform_rate_hz').value)
        self.progress_pub = self.create_publisher(Int32, 'waypoint_progress', qos)
        self.state_pub = self.create_publisher(String, 'waypoint_state', qos)
        self.create_subscription(Path, 'waypoint_queue', self._queue_cb, qos)
        self.create_subscription(Empty, 'waypoint_cancel', self._cancel_cb, qos)
        self.create_subscription(Odometry, 'dlio/odom_node/odom', self._odom_cb, 10)
        self._publish_state('idle')
        if self._retransform_enabled and _rate > 0.0:
            self.create_timer(1.0 / _rate, self._retransform_tick)

        self.get_logger().info(
            f'rviz_goal_bridge_node已启动，目标frame={self.target_frame}，'
            f'订阅rviz_goal_world -> TF变换 -> 发布term_goal；'
            f'另订阅waypoint_queue/waypoint_cancel，实现航点队列执行；'
            f'目标点持续重变换={"开" if self._retransform_enabled else "关"}')

    def _cb(self, msg: PoseStamped):
        # 2026-08-09现场实测：RViz发布2D Goal Pose时msg.header.stamp盖的是
        # RViz自己节点的真实墙钟（没有配置use_sim_time），而DLIO/UWB这整棵
        # TF树（NX0x/odom、NX0x/map、两机之间的frame_align）全部活在
        # mighty_imu_sim_time.patch/livox_imu_lidar_sim_time.patch引入的
        # "假epoch"时钟域里（仿真时间+固定偏移1735689600，即2025-01-01，
        # 详见patches/mighty_imu_sim_time.patch的注释）——
        # 这两个时钟域相差一年半，如果直接按msg.header.stamp（真实墙钟）去
        # 查TF，这个时间点在假epoch的TF树里永远不存在，Buffer.transform()
        # 每次都会100%报"Lookup would require extrapolation into the
        # future"，不是偶发的时序竞态。改成按时间戳0查（tf2里0代表"取最新
        # 可用数据"，不代表真的查时刻0），绕开这两个时钟域对不上的问题——
        # 代价是查到的是"最新"而不是"点击那一刻"的TF，两机相对位姿变化
        # 远比点击的人手速度慢，这点滞后可以忽略。
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.5))
            local = tf2_geometry_msgs.do_transform_pose_stamped(msg, transform)
        except TransformException as ex:
            # 查不到变换就不转发——总比把原始点击坐标当成本机局部坐标直接
            # 发出去（旧版那种"假设Fixed Frame就是世界系"的错误）安全，
            # 后者会让飞机飞向一个跟点击位置毫不相关的坐标。
            self.get_logger().error(
                f'从{msg.header.frame_id}变换到{self.target_frame}失败：'
                f'{ex}，这次点击丢弃，不转发term_goal')
            return
        self.pub.publish(local)
        self._last_pub_local = (local.pose.position.x, local.pose.position.y,
                                local.pose.position.z)
        # 2026-09-03：记住这个目标点在**世界系**下的坐标，之后θ*每次更新都
        # 重新换算一遍(见_retransform_tick)。查不到world这条链路(起飞点还没
        # 锁定)时就不记，行为退回修复之前的一次性变换。
        # 只记单点目标，不动航点队列的状态——2026-09-03这次修复不打算顺手
        # 改变"RViz点目标点时正在执行的队列该怎么办"这个既有语义；
        # _active_goal_world()本来就优先返回队列里的当前航点，队列在跑的时候
        # 这个单点目标不会生效。
        self._single_goal_world = self._to_world(local)

    # ---- 航点队列执行器 ----

    def _publish_state(self, state: str):
        self.state_pub.publish(String(data=state))

    def _publish_progress(self):
        self.progress_pub.publish(Int32(data=self._queue_idx))

    def _publish_local_goal(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.frame_id = 'map'  # 只作标注，接收端(ego_replan_fsm)不看这个字段
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self._last_pub_local = (x, y, z)

    # ---- 2026-09-03 目标点持续重变换（论文3.2节的修复），见文件头说明 ----

    def _to_world(self, pose_local: PoseStamped):
        """局部(target_frame) -> 世界(world)。查不到TF返回None。

        为什么"先换成世界系再存"而不是让上游直接给世界坐标：GCS网页和RViz
        这两条入口发过来的本来就是各自坐标系下的点，改协议要动前端；而这里
        收到的一刻是用**当时**的TF换算的，反变换回去得到的正是操作员真正
        意指的那个世界坐标，等价于把操作意图完整还原出来，不用改任何上游接口。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', self.target_frame, Time(), timeout=Duration(seconds=0.2))
        except TransformException:
            return None
        src = PoseStamped()
        src.header.frame_id = self.target_frame
        src.pose = pose_local.pose
        w = tf2_geometry_msgs.do_transform_pose_stamped(src, tf)
        return (w.pose.position.x, w.pose.position.y, w.pose.position.z)

    def _to_local(self, world_xyz):
        """世界(world) -> 局部(target_frame)。查不到TF返回None。

        ⚠️ timeout必须是0：这个函数在_odom_cb里被逐帧调用(10~50Hz)，
        `world -> {ns}/odom`是一条静态TF——要么已经在树里、立刻就能查到，
        要么起飞点还没锁定、等多久都不会有。给个非零timeout只会在"还没锁定"
        这个完全正常的阶段把里程计回调每帧阻塞住，白白拖慢整条链路。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, 'world', Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        src = PoseStamped()
        src.header.frame_id = 'world'
        (src.pose.position.x, src.pose.position.y,
         src.pose.position.z) = world_xyz
        src.pose.orientation.w = 1.0
        loc = tf2_geometry_msgs.do_transform_pose_stamped(src, tf)
        return (loc.pose.position.x, loc.pose.position.y, loc.pose.position.z)

    def _active_goal_world(self):
        """当前还没到达的那个目标点的世界坐标；没有活跃目标时返回None。"""
        if 0 <= self._queue_idx < len(self._queue_world):
            return self._queue_world[self._queue_idx]
        return self._single_goal_world

    def _current_local_goal(self):
        """当前活跃目标换算到局部系的坐标（换算不了时退回上次发出去的值）。"""
        w = self._active_goal_world()
        if w is None:
            return self._last_pub_local
        return self._to_local(w) or self._last_pub_local

    def _retransform_tick(self):
        """定时用当前TF重新换算活跃目标点，变化超过阈值才重发term_goal。

        θ*收敛之后局部目标几乎不动，差值达不到阈值，这个timer就什么都不做
        ——不会持续给规划器发新目标、触发无意义的重规划。
        """
        w = self._active_goal_world()
        if w is None:
            return
        loc = self._to_local(w)
        if loc is None:
            return
        if self._last_pub_local is not None:
            if math.dist(loc, self._last_pub_local) < self._retransform_min_delta:
                return
        self.get_logger().info(
            f'θ*更新导致目标点局部坐标变化，重发term_goal: '
            f'({loc[0]:.2f}, {loc[1]:.2f}, {loc[2]:.2f})')
        self._publish_local_goal(*loc)

    def _queue_cb(self, msg: Path):
        # GCS发来的每个点已经是局部坐标（浏览器用跟单点点击相同的
        # world_to_local()换算过），这里不做任何TF转换，直接当term_goal
        # 用。新队列无条件替换掉旧的（不管旧队列是不是还没飞完）——GCS
        # 网页的"单点发目标点"也统一改走这条路径（发一个只有1个点的
        # 队列），所以"发一个新目标点"天然就是"打断旧队列、执行新的"，
        # 不会跟旧队列的term_goal互相覆盖打架。
        self._queue = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                        for p in msg.poses]
        # 2026-09-03：同时记一份世界坐标，之后θ*每次更新都用它重算局部目标
        # （见文件头说明）。查不到TF时这里是None，_active_goal_world会因此
        # 返回None、重变换自动停用，行为退回修复之前，队列本身照常执行。
        self._single_goal_world = None
        self._queue_world = []
        for (lx, ly, lz) in self._queue:
            ps = PoseStamped()
            ps.header.frame_id = self.target_frame
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = lx, ly, lz
            ps.pose.orientation.w = 1.0
            self._queue_world.append(self._to_world(ps))
        if not self._queue:
            self._queue_idx = -1
            self._publish_state('idle')
            self._publish_progress()
            return
        self._queue_idx = 0
        self._publish_state('executing')
        self._publish_progress()
        x, y, z = self._queue[0]
        self._publish_local_goal(x, y, z)
        self.get_logger().info(
            f'收到航线，共{len(self._queue)}个航点，飞往第1个：'
            f'({x:.2f}, {y:.2f}, {z:.2f})')

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg
        if self._queue_idx < 0 or self._queue_idx >= len(self._queue):
            # 单点目标(rviz_goal_world那一路)没有队列状态机，到达判断放这里：
            # 进了到点半径就把活跃目标清掉，停止持续重变换。
            if self._single_goal_world is not None:
                loc = self._to_local(self._single_goal_world)
                if loc is not None:
                    p0 = msg.pose.pose.position
                    if math.dist((p0.x, p0.y, p0.z), loc) <= ARRIVAL_THRESHOLD_M:
                        self._single_goal_world = None
            return
        # 2026-09-03：到点判断要跟"现在真正在飞的那个目标点"比——θ*更新后
        # 目标点已经被_retransform_tick重算过，还拿收到队列那一刻的旧局部
        # 坐标判到点，会在修正量比较大时误判(提前或推迟切下一个航点)。
        tx, ty, tz = self._current_local_goal() or self._queue[self._queue_idx]
        p = msg.pose.pose.position
        dist = math.sqrt((p.x - tx) ** 2 + (p.y - ty) ** 2 + (p.z - tz) ** 2)
        if dist > ARRIVAL_THRESHOLD_M:
            return
        self._queue_idx += 1
        self._publish_progress()
        if self._queue_idx >= len(self._queue):
            self._publish_state('completed')
            # 到达之后不再重变换——论文3.2节的修复语义是"持续修正直到到达
            # 为止"，到了还继续改目标点只会让飞机在终点附近被反复推来推去。
            self._queue_world = []
            self._single_goal_world = None
            self.get_logger().info('航线执行完毕，最后一个航点已到达')
            return
        x, y, z = self._current_local_goal() or self._queue[self._queue_idx]
        self._publish_local_goal(x, y, z)
        self.get_logger().info(
            f'到达第{self._queue_idx}个航点，飞往第{self._queue_idx + 1}个：'
            f'({x:.2f}, {y:.2f}, {z:.2f})')

    def _cancel_cb(self, _msg: Empty):
        if self._queue_idx < 0:
            return  # 没有正在执行的队列，忽略
        self._queue = []
        # 取消时同样要清掉活跃目标，否则刹车点发出去之后重变换还会把飞机
        # 拽回那个已经被取消的航点（2026-09-03修复引入的新交互，容易漏）
        self._queue_world = []
        self._single_goal_world = None
        self._queue_idx = -1
        self._publish_progress()
        if self._latest_odom is None:
            self._publish_state('cancelled')
            self.get_logger().warn(
                '收到取消指令，但还没收到过里程计，无法计算刹车点，只清空队列')
            return
        p = self._latest_odom.pose.pose.position
        v = self._latest_odom.twist.twist.linear
        # DLIO在odom.cc里线速度显式填的是里程计世界系(state.v.lin.w，
        # 见文件头注释)，可以直接拿v.x/v.y当这架飞机局部坐标系下的
        # 速度分量用，不需要额外用yaw旋转。
        speed = math.hypot(v.x, v.y)
        if speed > 0.05:
            dx, dy = v.x / speed, v.y / speed
        else:
            # 接近悬停，没有明确的速度方向可顺延——退化用当前机头朝向，
            # 仍要保证顺延距离不为0（不能把当前位置原样发回去，见文件头
            # 注释里"起点终点重合导致轨迹生成数值奇异"那段论证）。
            q = self._latest_odom.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
            dx, dy = math.cos(yaw), math.sin(yaw)
        brake_dist = min(max(speed * BRAKE_LOOKAHEAD_SEC, BRAKE_MIN_M), BRAKE_MAX_M)
        self._publish_local_goal(p.x + dx * brake_dist, p.y + dy * brake_dist, p.z)
        self._publish_state('cancelled')
        self.get_logger().info(
            f'收到取消指令，沿当前速度方向顺延{brake_dist:.2f}米发送刹车目标点')


def main():
    rclpy.init()
    node = RvizGoalBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
