#!/usr/bin/env python3
"""起飞点"一键锁定"节点——docker_sim里"UWB现场一键确定起始点"需求的原型实现。

背景：当前spawn/局部坐标原点完全靠仿真专用的`INIT_X = AGENT_INDEX*3`环境变量
算出来的固定偏移，跟UWB毫无关系；`dual_goal_input.py`发目标点时还要让操作员
自己按这个偏移心算。真机上没有这种"预先写死的spawn偏移"可用，需要现场用UWB
测一次绝对位置，换算出"DLIO局部坐标 -> 统一任务坐标系"的偏移量。

设计：
  1. 订阅一路"绝对位置"话题（真机是真实UWB模块驱动；这套仿真里是
     `uwb_sim`新增的`/{ns}/uwb/pose_abs`，数据源是Gazebo世界真值+高斯噪声，
     见uwb_ground_truth_node.py的说明——这个节点完全不知道、也不关心背后
     是真UWB还是仿真替身，只认这一个话题格式，真机切换时这个节点不用改
     一行代码）。
  2. 提供一个Trigger服务，操作员/launch控制台一键触发：
     取最近`sample_window_sec`秒内的UWB样本，数量/抖动(标准差)都达标才认为
     可信，跟当前DLIO里程计位置算差值，得到offset；把offset广播成一条
     静态TF（world_frame -> map_frame），并持久化到磁盘文件，防止节点
     重启后丢失。
  3. 假设两机spawn时yaw接近0，只处理平移，不处理旋转对齐——如果实际部署时
     飞机起飞朝向差异明显，这里需要补一个姿态对齐环节，当前版本不做这个
     假设之外的事。（⚠️ 这一条已经被下面"2026-08-12新增"那段的在线SE(2)
     旋转估计取代，只处理平移只是最初版本的简化，现在θ*会自动估计出真实
     旋转偏移，不再要求spawn yaw≈0——保留这段是为了记录设计演进过程。）

⚠️ 2026-08-12新增：情景二(`LOCALIZATION_SOURCE=uwb_slam`——DLIO SLAM正常
工作，但局部系yaw跟全局系之间有未知旋转偏移，UWB只给位置不给yaw)的在线
SE(2)旋转估计，见`docker_sim/多机空间坐标对齐问题_设计讨论纪要.docx`
第四节公式推导、第八节8.2实操步骤。原来的锁定逻辑只处理平移（假设spawn
yaw跟world_frame一致），这里升级成"平移仍然是一次性静止锁定，旋转θ*
飞机一动起来就持续在线估计、不需要用户手动触发"：
  1. 持续订阅DLIO odom(局部系)和UWB位置(全局系)，飞机每累积
     `rotation_seg_min_disp_m`米位移就记一段位移向量对(u_i局部, v_i全局)，
     进滑窗（`rotation_window_size`段，滑出的旧段从累加量里减掉，不是
     重新全量算一遍）。
  2. 按最小二乘公式`θ*=atan2(S,C)`，`S=Σ(u_ix·v_iy-u_iy·v_ix)`，
     `C=Σ(u_ix·v_ix+u_iy·v_iy)`在线维护。`atan2`是闭式解，单独一段就是
     一个良定的方程，数学上第一段就能解——`rotation_min_segments`默认
     改成1，第一段记录完立刻发布，不强行等凑够多段才给结果（第一版
     默认5段才发布，样本卡在1-4段时诊断话题会一直"存在但从未发布过"，
     从外面完全看不出到底是卡住了还是只是样本还没攒够，2026-08-12排查
     origin_setter_node卡死那次因此走了不少弯路，教训是"能给的中间过程
     就先给，不要憋到达标才给"）。样本越多θ*越平滑（滑窗平均掉噪声），
     但不是"没达到某个数量就完全不能用"。
  3. 静止锁定`_handle_set_origin`触发时记的是"锁定那一刻"的局部/全局
     位置均值(`_local_lock`/`_uwb_lock`)，作为SE(2)变换的锚点——之后θ*
     每次更新，都用这个固定锚点重新算平移`t=uwb_lock-R(θ*)·local_lock`
     再重新广播TF，不是锚点本身也跟着变，这样变换在锚点这一个已知点上
     永远精确，只是绕锚点的朝向在持续修正。
  4. 这套旋转估计逻辑不感知`LOCALIZATION_SOURCE`是什么（这个节点从设计
     上就不关心PLANNER/CONTROLLER/LOCALIZATION_SOURCE，见文件头一直以来
     的说明）——`gt`/`uwb_imu`模式下局部系本来就跟全局系同向，θ*会自然
     收敛到接近0，不会因为多跑了这段逻辑就出问题，不需要额外开关。

⚠️ 2026-08-10实测踩坑修正：最初这里把静态TF发成`world_frame -> odom_frame`
（直接拿`{ns}/odom`当子frame），结果导致RViz里NX01整个消失、`2D Goal Pose`
点目标点NX02纹丝不动——现场排查发现`flight-stack-entrypoint.sh`里本来就
无条件发布一条恒等静态TF`{ns}/map -> {ns}/odom`（给ego_planner的rviz可见性
用，见entrypoint.sh里"2026-08-08用户反馈multi_ego_planner.rviz...看不到"
那段注释），这条TF已经把`{ns}/odom`的父frame定成了`{ns}/map`——同一个
`{ns}/odom`不能同时有两个不同的父frame（`{ns}/world`和`{ns}/map`），
tf2的树结构下两个静态发布者互相打架，其中一个会导致对应机器人的整棵子树
从Fixed Frame断开，`ros2 run tf2_ros tf2_echo {ns}/world {ns}/odom`实测
直接报"TF has two or more unconnected trees"。修复：改成往`{ns}/map`
（而不是`{ns}/odom`）挂这条`world_frame -> map_frame`的静态TF——`{ns}/map`
在当前架构里本来就是`{ns}/odom`的父frame、且跟它是恒等关系（零偏移），
数值上"锁定{ns}/map在world系下的偏移"和"锁定{ns}/odom在world系下的偏移"
完全等价，但只把`{ns}/world`接到已有链条的最上游（`{ns}/map`原来没有
父frame，现在补上），不会跟已有的`{ns}/map -> {ns}/odom`产生"重复认领
同一个子frame"的冲突。
⚠️ 2026-09-03新增（SE(2)在线标定论文2.7节的抗差扩展，见
`docker_sim/实验方案_SE2在线标定论文补充实验.md` L2层改动(c)）：
`rotation_robust_enabled`打开后，θ*不再直接读增量维护的atan2(S,C)，改成对
滑窗内W段做一轮迭代重加权最小二乘(IRLS)：按上一次估计算每段残差
r_i=‖v_i-R(θ̂)u_i‖，用Huber权重w_i=min(1, k/r_i)重新加权求解，k按窗口内
残差的中位数绝对偏差(MAD)自适应设定(k=k_scale×MAD)。动机是UWB多径/NLOS
会让某帧观测偏离真实值几十厘米到几米，而原始式(6)的隐式权重正比于|u_i||v_i|
——被污染的段模长更大、反而获得更大权重，方向刚好是错的。
**默认关闭(false)**，关闭时行为跟以前逐字节一致（还是走O(1)增量的S/C）；
打开后每次更新要遍历窗口内W段、跑几轮迭代，代价是O(W·iters)，在W=50这个
量级上仍然可以忽略（实测见论文实验E16）。
两个诊断话题`origin_setter/residual_mad`(米)和`origin_setter/downweighted`
(被降权的段数)无论开关与否都会发布——不打开抗差时它们就是"当前观测质量"
的只读指标，可以先用它们看清楚现场到底有没有野值、有多少，再决定要不要
打开抗差，而不是盲目开着。

"""
import json
import math
import os
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


class OriginSetterNode(Node):

    def __init__(self):
        super().__init__('origin_setter')

        ns = self.get_namespace().strip('/')  # rclpy的namespace带前导'/'，去掉方便拼frame名

        self.declare_parameter('uwb_pose_topic', 'uwb/pose_abs')
        self.declare_parameter('odom_topic', 'dlio/odom_node/odom')
        self.declare_parameter('sample_window_sec', 2.0)
        self.declare_parameter('min_samples', 5)
        # 2026-08-25临时放宽：真机首次联调时UWB样本Z轴std_dev实测0.49m，
        # 卡在原来0.15m的阈值上锁不了原点，用户要求"先忽略这一点走通全流程"，
        # 不是判定阈值本身设错了——0.15m对应"飞机基本静止"这个假设，真机
        # UWB高度解算目前噪声明显比这个假设大，先放宽到2.0m让流程能走下去，
        # 定位精度验证/UWB质量排查是后续单独的任务，不应该现在卡住整条
        # 起飞流程。等定位链路调稳后应该调回一个经过实测验证的更严格阈值，
        # 不是永久性的改动。
        self.declare_parameter('max_std_dev_m', 2.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        # 2026-08-10第二处勘误（用户实测报告"NX01/world在场景中央、NX02/world
        # 消失"）：这里原来默认是`f'{ns}/world'`，两架飞机各有一个互不相干的
        # world——但真实UWB系统里，所有锚点定义的是全队共享的同一套坐标系，
        # 不是每架飞机各自一套；仿真里对应的也是Gazebo唯一的世界坐标系。
        # 两个各自独立、互不连通的"NX01/world"/"NX02/world"必然是两棵各自
        # 孤立的TF子树，RViz的Fixed Frame只能连到其中一边，另一边自然渲染
        # 不出来（或者渲染在原点，取决于RViz对"找不到路径"这种情况的具体
        # 处理）。改成不带命名空间前缀的共享`world`——两架飞机的
        # `world -> {ns}/map`各自独立锁定、互不冲突（子frame不同），但共用
        # 同一个父frame，从此两机的整棵TF树能通过这一个共同祖先连通。
        self.declare_parameter('world_frame', 'world')
        # 挂静态TF用的子frame——特意选{ns}/map而不是{ns}/odom，见文件头
        # 2026-08-10勘误：{ns}/odom已经有一个固定父frame({ns}/map，
        # flight-stack-entrypoint.sh无条件发布的恒等TF)，不能再认第二个
        # 父frame。{ns}/map跟{ns}/odom数值上是恒等关系，锁哪个效果一样。
        self.declare_parameter('map_frame', f'{ns}/map')
        self.declare_parameter('persist_path', f'/tmp/uwb_origin_{ns}.json')
        # 情景二SE(2)在线旋转估计参数，见文件头2026-08-12新增说明。
        # 2026-09-14新增：rotation_estimation_enabled——这套旋转估计逻辑
        # 原本"不感知LOCALIZATION_SOURCE"（文件头2026-08-12说明原话），
        # 理由是"gt/uwb_imu模式下局部系本来就跟全局系同向，θ*会自然收敛
        # 到接近0"，但这个假设只在"样本足够多、噪声被滑窗平均掉"之后才
        # 成立——`rotation_min_segments`默认只要1段就直接发布，实测踩过
        # 坑：容器刚重启、UWB/里程计还没完全稳定的窗口里，一次瞬态噪声
        # 就能被当成>0.5米的真实位移记成1段，`atan2()`算出一个完全是
        # 噪声、跟真实无关的θ*（2026-09-14实测复现：NX01/NX02分别算出
        # 164.9°/70.5°，把`world_to_local()`换算全部污染，导致飞机飞向
        # 错误位置、贴着障碍物卡死），且`_handle_set_origin()`只重新锚定
        # 平移、不会重置`_theta`/`_segments`，坏状态只能靠重启整个进程
        # 清掉。用户确认的架构事实：`gt`/`uwb_imu`（含`single_uwb_imu`）
        # 模式下局部系与全局系之间架构上就是纯平移，不存在真实旋转需要
        # 估计——SE(2)在线标定真正要解决的场景只有`uwb_slam`/
        # `single_uwb_slam`（DLIO SLAM局部系yaw相对全局系有未知漂移）。
        # 这个参数由`uwb_origin_bridge.launch.py`按`LOCALIZATION_SOURCE`
        # 计算好传进来，仅`uwb_slam`/`single_uwb_slam`为True，其余模式
        # False——False时`_maybe_record_segment()`整个不执行，不累积任何
        # 段、θ永远保持初始值0.0，等价于"强制θ*=0"，不是把阈值调高这种
        # 治标不治本的做法。默认值True是为了任何遗漏传参的调用场景（比如
        # 以后新增的真机launch文件忘记传这个参数）保持"旧行为"，不会因为
        # 默默改变默认值导致uwb_slam模式意外被关掉这套估计——真正决定
        # 每种LOCALIZATION_SOURCE该不该开的地方在launch文件里，不在这个
        # 默认值上。
        self.declare_parameter('rotation_estimation_enabled', True)
        self.declare_parameter('rotation_seg_min_disp_m', 0.5)
        self.declare_parameter('rotation_window_size', 50)
        self.declare_parameter('rotation_min_segments', 1)
        # 2026-09-03抗差扩展，见文件头说明。默认关，行为不变。
        self.declare_parameter('rotation_robust_enabled', False)
        self.declare_parameter('rotation_huber_iters', 3)
        self.declare_parameter('rotation_huber_k_scale', 3.0)

        # 2026-09-24新增：锁定前检查"里程计高度跟测距雷达对得上"。
        # 实测踩的坑：LOCALIZATION_SOURCE=uwb_imu时odom的z就是离地高度
        # （uwb_imu_fusion_node把range*cos(roll)*cos(pitch)喂给飞控当高度
        # 观测），但容器刚起来那几秒`mavros/hrlv_ez4_pub`还没发出第一帧
        # （实测头一帧range是nan），飞控高度估计这段时间没有任何观测拉着，
        # 自由漂移——NX01就是在这个窗口里锁的原点：真值z=0.052m、odom z却
        # 已经漂到-0.415m，于是`world -> {ns}/odom`这条静态TF的z偏移被记成
        # 0.467m（正常应该≈起降点离地面的0.05m）。雷达一上线odom z立刻被
        # 拉回真实离地高度，偏移却永久留在TF里，结果`world_to_local()`换算
        # 出来的每一个高度都系统性偏低0.47米：飞机按局部z=1.53飞（实际离地
        # 1.53米），选手以为是2.0米；再叠上traj_server定高开关钉在另一个值
        # 时，goto()的三维到点判据(0.3米)永远满足不了，飞机水平到位却判不到
        # 点，一直卡到超时（2026-09-24编队第一个航点卡死就是这么来的）。
        # 判据不靠"多等几秒应该够了"：直接比对两个高度源，对不上就返回失败，
        # entrypoint那个每3秒重试一次的循环会自然等到它们一致（雷达上线、
        # 飞控高度收敛）才锁。只在"odom的z来自测距雷达"的定位源下开，由
        # launch按LOCALIZATION_SOURCE决定。
        self.declare_parameter('height_check_enabled', False)
        self.declare_parameter('range_topic', 'mavros/hrlv_ez4_pub')
        self.declare_parameter('max_height_mismatch_m', 0.15)
        # 2026-09-24同一批：锁定前还要求"里程计自己是静止的"。上面那个高度
        # 检查只管z，x/y有一模一样的坑：飞控/DLIO刚起来那十几秒位置估计还在
        # 收敛，实测NX02在锁定那一刻odom读到(0.414, -0.673)，而它一直停在
        # 起降点没动过，十几秒后同一个话题读到的是(0.078, -0.003)——锁定拿的
        # 是瞬态值，于是world->{ns}/odom的平移被永久记偏0.67米（锁定日志里
        # 的偏移是(-2.418, -8.828)，真实起降点是(-2.0, -9.5)）。后果是这架飞机
        # 所有`world_to_local()`换算出来的位置都系统性偏0.67米：goto()看着
        # "到点了"，实际停在偏0.67米的地方。
        # 判据是"窗口内里程计的峰峰值"——瞬态期间它在动，静止收敛后只有几
        # 厘米的抖动。真机上这个阈值可能需要按实测放宽（参数化，不写死）。
        self.declare_parameter('max_odom_p2p_m', 0.15)

        self.sample_window_sec = self.get_parameter('sample_window_sec').value
        self.min_samples = self.get_parameter('min_samples').value
        self.max_std_dev_m = self.get_parameter('max_std_dev_m').value
        self.odom_timeout_sec = self.get_parameter('odom_timeout_sec').value
        self.world_frame = self.get_parameter('world_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.persist_path = self.get_parameter('persist_path').value
        self.rotation_estimation_enabled = self.get_parameter('rotation_estimation_enabled').value
        self.rotation_seg_min_disp_m = self.get_parameter('rotation_seg_min_disp_m').value
        self.rotation_window_size = self.get_parameter('rotation_window_size').value
        self.rotation_min_segments = self.get_parameter('rotation_min_segments').value
        self.rotation_robust_enabled = self.get_parameter('rotation_robust_enabled').value
        self.rotation_huber_iters = int(self.get_parameter('rotation_huber_iters').value)
        self.rotation_huber_k_scale = float(
            self.get_parameter('rotation_huber_k_scale').value)
        self.height_check_enabled = bool(self.get_parameter('height_check_enabled').value)
        self.max_height_mismatch_m = float(self.get_parameter('max_height_mismatch_m').value)
        self.max_odom_p2p_m = float(self.get_parameter('max_odom_p2p_m').value)

        self._uwb_samples = deque()  # (monotonic_recv_time, x, y, z)
        self._odom_pos = None
        self._odom_recv_time = None
        self._odom_samples = deque()  # (recv_time, x, y, z)，只留sample_window_sec窗口，用来判静止
        self._range_m = None          # 最近一帧测距雷达高度（米），nan/inf不记
        self._range_recv_time = None
        self._origin_locked = False
        # 静止锁定那一刻的局部/全局位置均值——SE(2)变换的固定锚点，θ*更新时
        # 复用这一对，不是每次都重新取当前位置当锚点。
        self._local_lock = None
        self._uwb_lock = None

        # 情景二在线旋转估计的运行时状态，见文件头2026-08-12新增说明。
        self._latest_uwb_xy = None
        self._seg_anchor_local = None
        self._seg_anchor_uwb = None
        self._segments = deque()  # 每段: (ux, uy, vx, vy)
        self._seg_S = 0.0
        self._seg_C = 0.0
        self._theta = 0.0  # 当前最优估计，弧度；样本不够前一直是0（等价于纯平移假设）

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._locked_pub = self.create_publisher(Bool, 'origin_locked', 10)
        # docx第8.3节验证命令期望的话题路径是".../origin_setter/yaw_estimate"
        # ——显式给话题名加"origin_setter/"前缀，不是靠节点名自动生成的默认
        # 前缀（这个节点自己叫"origin_setter"，但发布者话题名默认不会自动
        # 带上节点名这一段）。
        self._yaw_estimate_pub = self.create_publisher(Float64, 'origin_setter/yaw_estimate', 10)
        self._yaw_sample_count_pub = self.create_publisher(Int32, 'origin_setter/yaw_sample_count', 10)
        # 2026-09-03：观测质量诊断，见文件头说明。抗差开不开都发。
        self._residual_mad_pub = self.create_publisher(
            Float64, 'origin_setter/residual_mad', 10)
        self._downweighted_pub = self.create_publisher(
            Int32, 'origin_setter/downweighted', 10)

        self.create_subscription(
            PoseStamped, self.get_parameter('uwb_pose_topic').value, self._uwb_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)

        if self.height_check_enabled:
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(
                Range, self.get_parameter('range_topic').value, self._range_cb,
                qos_profile_sensor_data)   # 传感器话题是BEST_EFFORT，用默认可靠QoS订不上

        self.create_service(Trigger, 'set_origin_from_uwb', self._handle_set_origin)

        # 起飞点状态属于"需要长期能看到"的信息，不是一次性事件，用低频timer
        # 常驻重发，方便launch控制台/status_monitor这类后来才订阅的工具也能
        # 立刻看到当前状态，不用非得抓住那一次事件瞬间。
        self.create_timer(1.0, self._publish_locked_status)

        self._try_load_persisted_origin()

        self.get_logger().info(
            f"origin_setter就绪：uwb_pose_topic={self.get_parameter('uwb_pose_topic').value}, "
            f"odom_topic={self.get_parameter('odom_topic').value}, "
            f"world_frame={self.world_frame}, map_frame={self.map_frame}, "
            f"rotation_estimation_enabled={self.rotation_estimation_enabled}, "
            f"height_check_enabled={self.height_check_enabled}"
            f"{'（θ*强制为0，纯平移）' if not self.rotation_estimation_enabled else ''}")

    def _uwb_cb(self, msg: PoseStamped):
        now = time.monotonic()
        self._uwb_samples.append((now, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        cutoff = now - self.sample_window_sec
        while self._uwb_samples and self._uwb_samples[0][0] < cutoff:
            self._uwb_samples.popleft()
        self._latest_uwb_xy = (msg.pose.position.x, msg.pose.position.y)

    def _odom_cb(self, msg: Odometry):
        self._odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self._odom_recv_time = time.monotonic()
        self._odom_samples.append((self._odom_recv_time,) + self._odom_pos)
        cutoff = self._odom_recv_time - self.sample_window_sec
        while self._odom_samples and self._odom_samples[0][0] < cutoff:
            self._odom_samples.popleft()
        self._maybe_record_segment((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def _range_cb(self, msg: Range):
        r = float(msg.range)
        if not math.isfinite(r):
            return                      # 刚上线那几帧实测是nan，当成"还没数据"
        self._range_m = r
        self._range_recv_time = time.monotonic()

    def _maybe_record_segment(self, local_xy):
        """情景二在线旋转估计——飞机每累积rotation_seg_min_disp_m米位移，
        记一段(局部位移向量, 全局位移向量)，见文件头2026-08-12新增说明。

        ⚠️ 2026-08-12实测踩坑：现场对比一个独立诊断脚本(复刻同一套逻辑单独
        跑一份)发现，真机跑的origin_setter_node实例从容器启动起就一段没
        记过，累计移动20多米也没用，但独立诊断脚本在同一段真实移动窗口里
        正常记到了25段——排除了算法逻辑本身的问题，锁定是这个长期运行的
        实例自己的状态被"卡死"了。最可能的机制：DLIO/UWB刚启动的瞬间可能
        发过一帧NaN（SLAM/定位系统收敛前的瞬态值很常见），如果NaN被设成了
        _seg_anchor_local/_seg_anchor_uwb，之后math.hypot(...)算出来的
        disp恒为NaN，Python里`NaN < 阈值`恒为False——不管飞机真实移动
        多远，这个判断永远不通过，而且没有任何自我恢复路径，一旦第一次被
        污染就会卡到进程重启为止。这里补一道有限数值校验：当前点或锚点
        任意一个不是有限数(NaN/inf)时，直接当"还没有可用锚点"处理，重新
        设一次锚点，不再往下走距离判断——不管是最初设锚点污染，还是运行
        中途来了一帧坏数据，都能在下一帧正常数据到达时自动恢复，不用等
        进程重启。
        """
        if not self.rotation_estimation_enabled:
            # 2026-09-14新增：gt/uwb_imu(含single_uwb_imu)模式下架构上
            # 就是纯平移，不进这套逻辑——不累积任何段，θ永远保持__init__
            # 里设的初始值0.0，等价于"强制θ*=0"。见__init__里该参数声明
            # 处的完整说明。
            return
        if self._latest_uwb_xy is None:
            return
        if (self._seg_anchor_local is None
                or not all(math.isfinite(v) for v in self._seg_anchor_local)
                or not all(math.isfinite(v) for v in self._seg_anchor_uwb)
                or not all(math.isfinite(v) for v in local_xy)
                or not all(math.isfinite(v) for v in self._latest_uwb_xy)):
            if all(math.isfinite(v) for v in local_xy) and all(math.isfinite(v) for v in self._latest_uwb_xy):
                self._seg_anchor_local = local_xy
                self._seg_anchor_uwb = self._latest_uwb_xy
            return
        ulx = local_xy[0] - self._seg_anchor_local[0]
        uly = local_xy[1] - self._seg_anchor_local[1]
        if math.hypot(ulx, uly) < self.rotation_seg_min_disp_m:
            return
        vux = self._latest_uwb_xy[0] - self._seg_anchor_uwb[0]
        vuy = self._latest_uwb_xy[1] - self._seg_anchor_uwb[1]
        self._add_segment(ulx, uly, vux, vuy)
        # 段的终点变成下一段的起点，首尾相接，不是每段都从同一个原点算。
        self._seg_anchor_local = local_xy
        self._seg_anchor_uwb = self._latest_uwb_xy

    def _add_segment(self, ux, uy, vx, vy):
        self._segments.append((ux, uy, vx, vy))
        self._seg_S += ux * vy - uy * vx
        self._seg_C += ux * vx + uy * vy
        if len(self._segments) > self.rotation_window_size:
            old = self._segments.popleft()
            self._seg_S -= old[0] * old[3] - old[1] * old[2]
            self._seg_C -= old[0] * old[2] + old[1] * old[3]

        if len(self._segments) < self.rotation_min_segments:
            return  # 样本太少（默认1，第一段就发布），θ继续保持0

        # 抗差关闭时走O(1)增量的S/C（跟2026-09-03之前完全一致）；打开时
        # 对窗口内W段跑一轮IRLS，见文件头说明和_solve_theta_robust。
        if self.rotation_robust_enabled:
            self._theta = self._solve_theta_robust(self._theta)
        else:
            self._theta = math.atan2(self._seg_S, self._seg_C)
        self._publish_residual_diagnostics()
        self._yaw_estimate_pub.publish(Float64(data=self._theta))
        self._yaw_sample_count_pub.publish(Int32(data=len(self._segments)))
        if self._origin_locked:
            self._republish_with_theta()

    def _segment_residuals(self, theta):
        """每段在给定θ下的残差模长 r_i=‖v_i-R(θ)u_i‖（论文式16）。"""
        c, s = math.cos(theta), math.sin(theta)
        out = []
        for ux, uy, vx, vy in self._segments:
            rx = vx - (ux * c - uy * s)
            ry = vy - (ux * s + uy * c)
            out.append(math.hypot(rx, ry))
        return out

    @staticmethod
    def _median(xs):
        ys = sorted(xs)
        n = len(ys)
        if n == 0:
            return 0.0
        return ys[n // 2] if n % 2 else 0.5 * (ys[n // 2 - 1] + ys[n // 2])

    def _huber_weights(self, res):
        """Huber权重w_i=min(1, k/r_i)，k=k_scale×MAD（论文式17）。

        返回(weights, k)。MAD≈0时(比如无噪声仿真、或者段数太少)不降权，
        直接全1退化成原始闭式解——宁可不加权，也不要因为尺度估计不稳
        把正常段误杀。
        """
        med = self._median(res)
        mad = self._median([abs(r - med) for r in res])
        k = self.rotation_huber_k_scale * mad
        if k < 1e-9:
            return [1.0] * len(res), k
        return [1.0 if r <= k else k / r for r in res], k

    def _solve_theta_robust(self, theta_init):
        """迭代重加权最小二乘（论文式18）：用上一轮θ算残差->定权重->重解θ。"""
        theta = theta_init
        for _ in range(max(1, self.rotation_huber_iters)):
            w, _k = self._huber_weights(self._segment_residuals(theta))
            S = C = 0.0
            for wi, (ux, uy, vx, vy) in zip(w, self._segments):
                S += wi * (ux * vy - uy * vx)
                C += wi * (ux * vx + uy * vy)
            if S == 0.0 and C == 0.0:
                return theta
            theta = math.atan2(S, C)
        return theta

    def _publish_residual_diagnostics(self):
        """观测质量诊断：残差MAD(米) + 会被Huber降权的段数。

        抗差开关关着的时候这两个值不参与任何计算，纯粹是给运维/实验看的
        只读指标——先看清楚现场有没有野值，再决定要不要打开抗差。
        """
        res = self._segment_residuals(self._theta)
        if not res:
            return
        med = self._median(res)
        mad = self._median([abs(r - med) for r in res])
        k = self.rotation_huber_k_scale * mad
        n_down = sum(1 for r in res if k > 1e-9 and r > k)
        self._residual_mad_pub.publish(Float64(data=mad))
        self._downweighted_pub.publish(Int32(data=n_down))

    def _publish_locked_status(self):
        self._locked_pub.publish(Bool(data=self._origin_locked))

    def _handle_set_origin(self, request, response):
        now = time.monotonic()
        cutoff = now - self.sample_window_sec
        samples = [s for s in self._uwb_samples if s[0] >= cutoff]

        if len(samples) < self.min_samples:
            response.success = False
            response.message = (
                f"UWB样本不足（{len(samples)}/{self.min_samples}，"
                f"窗口{self.sample_window_sec}秒内）——检查uwb_pose_topic是否有发布者、"
                f"飞机是否已经通电静止在起飞点上")
            self.get_logger().error(response.message)
            return response

        if self._odom_pos is None or (now - self._odom_recv_time) > self.odom_timeout_sec:
            response.success = False
            response.message = "没有收到里程计数据（或已超时）——检查DLIO/gt_odom_bridge是否在正常发布odom"
            self.get_logger().error(response.message)
            return response

        odom_win = [q for q in self._odom_samples if q[0] >= cutoff]
        if len(odom_win) < 2:
            response.success = False
            response.message = (
                f"里程计样本不足（{len(odom_win)}条，窗口{self.sample_window_sec}秒内）——"
                "判不了飞机是不是静止，不锁定原点")
            self.get_logger().warn(response.message)
            return response
        p2p = tuple(max(q[i] for q in odom_win) - min(q[i] for q in odom_win) for i in (1, 2, 3))
        if max(p2p) > self.max_odom_p2p_m:
            response.success = False
            response.message = (
                f"里程计还在动（{self.sample_window_sec}秒内峰峰值"
                f"{tuple(round(v, 3) for v in p2p)}m > {self.max_odom_p2p_m}m）——"
                "位置估计还在收敛或飞机没停稳，这时锁定会把瞬态值永久记进"
                "world->odom这条TF里，不锁，等重试")
            self.get_logger().warn(response.message)
            return response

        if self.height_check_enabled:
            if self._range_m is None or (now - self._range_recv_time) > self.odom_timeout_sec:
                response.success = False
                response.message = (
                    f"还没收到测距雷达高度（{self.get_parameter('range_topic').value}）——"
                    "这个定位源下odom的z就是雷达高度，雷达没上线时飞控高度估计在自由漂移，"
                    "现在锁定会把漂移量永久记进world->odom这条TF的z偏移里，不锁，等重试")
                self.get_logger().warn(response.message)
                return response
            # 锁定时飞机静止在起降点上，roll/pitch≈0，cos修正项可以忽略（<1%）
            mismatch = abs(self._odom_pos[2] - self._range_m)
            if mismatch > self.max_height_mismatch_m:
                response.success = False
                response.message = (
                    f"里程计高度({self._odom_pos[2]:.3f}m)跟测距雷达高度({self._range_m:.3f}m)"
                    f"差{mismatch:.3f}m > {self.max_height_mismatch_m}m——飞控高度估计还没被雷达"
                    "观测拉回来（雷达刚上线/还在收敛），不锁，等重试")
                self.get_logger().warn(response.message)
                return response

        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        zs = [s[3] for s in samples]
        std_xyz = (
            statistics.pstdev(xs) if len(xs) > 1 else 0.0,
            statistics.pstdev(ys) if len(ys) > 1 else 0.0,
            statistics.pstdev(zs) if len(zs) > 1 else 0.0,
        )
        if max(std_xyz) > self.max_std_dev_m:
            response.success = False
            response.message = (
                f"UWB样本抖动过大（std_xyz={tuple(round(v, 3) for v in std_xyz)}m，"
                f"阈值{self.max_std_dev_m}m）——飞机可能还没静止，或者UWB信号质量差，不锁定原点")
            self.get_logger().error(response.message)
            return response

        uwb_mean = (statistics.fmean(xs), statistics.fmean(ys), statistics.fmean(zs))

        # 情景二：锁定这一刻的局部/全局位置均值当SE(2)变换的固定锚点，
        # 见文件头2026-08-12新增说明——θ*(当前_theta，样本不够时是0，退化
        # 成纯平移，行为等价于旧版)之后每次更新都复用这对锚点重新算平移。
        self._local_lock = tuple(self._odom_pos)
        self._uwb_lock = uwb_mean
        self._broadcast_and_persist()

        offset = tuple(uwb_mean[i] - self._odom_pos[i] for i in range(3))
        response.success = True
        response.message = (
            f"起飞点已锁定：{self.map_frame} 在 {self.world_frame} 系下的偏移 = "
            f"({offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f})m"
            f"（θ*={math.degrees(self._theta):.1f}°，{len(self._segments)}段位移样本），"
            f"里程计峰峰值{tuple(round(v, 3) for v in p2p)}m，"
            f"UWB样本std_xyz={tuple(round(v, 3) for v in std_xyz)}m（{len(samples)}个样本）")
        self.get_logger().info(response.message)
        return response

    def _republish_with_theta(self):
        """θ*在线更新时重新广播TF——复用锁定时的锚点，只有朝向在变。"""
        if self._local_lock is None or self._uwb_lock is None:
            return
        self._broadcast_and_persist()

    def _broadcast_and_persist(self):
        cy, sy = math.cos(self._theta), math.sin(self._theta)
        lx, ly, lz = self._local_lock
        ux, uy, uz = self._uwb_lock
        # t = uwb_lock - R(θ)·local_lock，只转x,y，z是普通高度差不受yaw影响。
        tx = ux - (lx * cy - ly * sy)
        ty = uy - (lx * sy + ly * cy)
        tz = uz - lz

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.map_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.z = math.sin(self._theta / 2.0)
        t.transform.rotation.w = math.cos(self._theta / 2.0)
        self._tf_broadcaster.sendTransform(t)
        self._origin_locked = True
        self._publish_locked_status()

        payload = {
            'world_frame': self.world_frame,
            'map_frame': self.map_frame,
            'local_lock_xyz': list(self._local_lock),
            'uwb_lock_xyz': list(self._uwb_lock),
            'theta_rad': self._theta,
            'locked_at_ros_time_sec': self.get_clock().now().nanoseconds / 1e9,
        }
        tmp_path = self.persist_path + '.tmp'
        with open(tmp_path, 'w') as f:
            json.dump(payload, f)
        os.replace(tmp_path, self.persist_path)  # 原子替换，避免读到写一半的文件

    def _try_load_persisted_origin(self):
        if not os.path.exists(self.persist_path):
            return
        try:
            with open(self.persist_path) as f:
                payload = json.load(f)
            if payload.get('world_frame') != self.world_frame or payload.get('map_frame') != self.map_frame:
                self.get_logger().warn(
                    f"{self.persist_path}里的frame名跟当前配置不一致，忽略这份持久化数据"
                    f"（文件={payload.get('world_frame')}->{payload.get('map_frame')}，"
                    f"当前={self.world_frame}->{self.map_frame}）")
                return
            # 2026-08-12：持久化格式从"offset_xyz"改成了"local_lock_xyz/
            # uwb_lock_xyz/theta_rad"（情景二SE(2)升级，见文件头说明）——
            # 旧格式的文件里没有theta_rad这个key，直接当"读取失败"忽略掉，
            # 不做旧格式兼容转换（这是仿真里的临时状态文件，不是要长期
            # 维护兼容性的用户数据，忽略掉重新锁一次比较简单可靠）。
            self._local_lock = tuple(payload['local_lock_xyz'])
            self._uwb_lock = tuple(payload['uwb_lock_xyz'])
            self._theta = payload['theta_rad']
        except (json.JSONDecodeError, KeyError, OSError) as e:
            self.get_logger().warn(f"读取持久化原点文件{self.persist_path}失败，忽略: {e}")
            return

        self._broadcast_and_persist()
        self.get_logger().info(
            f"从{self.persist_path}恢复了上次锁定的起飞点: local_lock="
            f"{tuple(round(v, 3) for v in self._local_lock)}, uwb_lock="
            f"{tuple(round(v, 3) for v in self._uwb_lock)}, "
            f"θ*={math.degrees(self._theta):.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = OriginSetterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
