#!/usr/bin/env python3
"""起飞后自标定动作——2026-08-12用户明确要求：点起飞按钮之后，飞机在
自己的局部坐标系里自动飞一段"前进1.1米、等3秒、退回起飞点、等3秒"的
往返机动，不需要操作员再额外开goal窗口手动打坐标。

背景：`origin_setter_node`的在线SE(2)旋转估计(θ*)需要飞机真的动起来
才能积累位移样本（每移动`rotation_seg_min_disp_m`=0.5米记一段），之前
只能靠操作员在goal窗口手动飞、或者随便飞飞任务轨迹顺带积累，没有一个
专门、可控、飞完就知道段数够不够的标定动作。1.1米这个距离不是随便挑的：
单程1.1米能稳定切出至少2段(1.1/0.5=2.2)，往返合计4段左右，虽然
`rotation_min_segments`已经改成1(第一段就能用)，但段数多一些能让滑窗
平均掉更多噪声，θ*收敛更稳。

⚠️ 全程只在飞机自己的局部坐标系(`dlio/odom_node/odom`)里操作，不涉及
任何世界坐标换算——这跟`dual_goal_input.py`/`status_monitor.py`那次
"局部转世界要用θ*才对"的修复刚好是同一枚硬币的两面：那次是"已知θ*，
换算成世界坐标给人看/给人发目标点用"；这次是"还不知道θ*，用纯局部系
自己的位移去帮origin_setter_node把θ*算出来"——如果这里反过来依赖世界
坐标换算，就是拿着还没标定出来的东西去标定它自己，本末倒置。局部系
+x方向定义为"飞机spawn那一刻自己朝的方向"（DLIO SLAM/gt模式的既有
约定），"往前飞"就是奔着局部系+x。

流程（每架飞机独立执行，两机互不依赖）：
  1. 等第一帧局部里程计，记下起飞点水平坐标(x_launch,y_launch)。
  2. 轮询局部里程计，直到实时位置进入目标点(x_launch,y_launch,
     TAKEOFF_TARGET_HEIGHT_M)的ARRIVE_TOLERANCE_M半径内，视为"真正爬升
     到位"（按实时距离判定，不是固定死时间/不是某个离散状态量翻转——
     2026-08-12实测踩过两版坑，见下面bug修复说明），带超时保护、超时
     直接中止（安全关键的硬前提条件，不能将就着往下走）。
  3. 用刚到位那一刻的实时坐标(x0,y0,z0)当基准点。
  4. 原地等待3秒（爬升到悬停之间的惯性残留有个消化的时间，也让UWB/DLIO
     读数先稳定下来，不用发任何新指令——飞机在这个阶段本来就应该是
     悬停状态）。
  5. 发一个目标点(x0+FORWARD_DIST, y0, z0)。
  6. 轮询局部里程计，进入目标点ARRIVE_TOLERANCE_M半径内视为到达；等不到
     也不会卡住不动——超时后照样往下走，见ARRIVE_TIMEOUT_SEC说明（这一步
     不是安全关键的，超时软继续没问题，跟第2步的硬中止语义不同）。
  7. 到达(或超时)后原地等待3秒。
  8. 发目标点(x0,y0,z0)，飞回起飞点正上方。
  9. 等到达(或超时)，之后不再发新目标点——node直接退出，让ego_planner/
     mighty自己继续悬停在这个位置（term_goal到达后本来就是这个行为，
     不需要额外发"保持悬停"指令）。

用法（起飞按钮触发时由launch_control.py用`docker exec -d`起，不需要
操作员手动跑；单独调试也可以手动跑）：
  python3 auto_calibration_flight.py NX01

⚠️ 2026-08-12实测bug修复：第一版`ARRIVE_TOLERANCE_M=0.15`米，实测两机都
在"前进1.1米"这一步卡住，`auto_calibration_flight_stdout.log`里能看到
准时在30秒超时后打出ERROR、自标定动作中止，从没发过"退回起飞点"那个
目标点——查了两个规划器"判定到达目标点"的真实标准，发现0.15米这个容差
比两者都严：`mighty`用`goal_radius`参数，这套docker-compose部署里实际
运行值是**0.3米**（`docker-compose.yml`里`GOAL_RADIUS`环境变量，源码
默认0.5米，源码见`staging/mighty_ws_src/mighty/src/mighty/mighty.cpp`
`needReplan()`里`dist_to_term_G < par_.goal_radius`那个判定）；
`ego_planner`（当前默认规划器）**压根没有距离判定**，`ego_replan_fsm.cpp`
里`have_target_`只按"规划好的轨迹时长是否已经跑完"来清空目标(`t_cur >
info->duration_`)，唯一沾边的距离参数`thresh_no_replan_meter`默认1.0米，
含义也只是"离目标够近就不用再重新规划"，不是"到达确认"。也就是说旧的
0.15米比两个规划器自己的"到达"标准都更严，实测中飞机大概率已经飞到了
`mighty`/`ego_planner`自己认为"到"的地方、内部转入悬停/不再主动修正，
但0.15米这个更严的检查一直不通过，只能干等到超时。

修复两处：①`ARRIVE_TOLERANCE_M`从0.15米放宽到0.4米（比`mighty`实际的
0.3米再留点余量，`ego_planner`没有严格收敛保证，也不该比它自己的判据
更严）；②到达等待改成"软等待"——超时不再直接放弃整个自标定动作，只打
一条WARNING然后继续走下一步（原来的实现里，等到达用的是`_spin_until`，
超时=中止；现在到达检测跟"起飞完成"这类真正的前提条件分开成两个不同的
辅助函数，前者超时可以将就着继续，后者超时必须真的停下来，语义不一样，
不能用同一套"超时即失败"逻辑）。

⚠️ 2026-08-12第二次实测bug修复（比上面那个更早触发、是这次"前飞不动、
飞机自己又落地了"的真正根因）：`起飞完成`判定原来抄了`status_monitor.py`
的`_AIRBORNE_LANDED_STATES`（`IN_AIR`+`TAKEOFF`+`LANDING`都算"在空中"）。
这个集合对`status_monitor.py`的飞行计时器场景是对的（爬升阶段也要算
飞行时间），但这里要判断的是"爬升是否真的已经完成、能不能开始记基准点
/发精确目标点"，语义不一样。实测容器日志锁定：`起飞`口令下达后，`mavros/
extended_state.landed_state`几乎瞬间（1.6秒）就从`ON_GROUND`跳到
`TAKEOFF`，旧代码认为这就算"起飞完成"，当场记下的基准点local=
(-0.00,-0.00,+0.00)——z还在地面高度；而`so3ctrl`自己真正的开环起飞状态机
（`MANUAL_CTRL->AUTO_TAKEOFF->AUTO_HOVER`）要12.5秒才真正走完。旧代码
比真正爬升完成早了约11秒发出"前进1.1米"目标点，这个目标点的z直接继承了
还在地面高度的基准点，把mighty的规划目标点在了`findAandAtime()`的
`z_min=0.3`硬下限之下（见`mighty.cpp`）——这个z越界检查是硬失败且没有
自愈机制，一旦命中就永久卡死、再也算不出新轨迹，飞机表现为"起飞、卡住
不动、mighty容器日志刷屏`A (...) is out of the map`、最后又落回地面"。
当时的修复：新增`_STABLE_AIRBORNE_LANDED_STATES`，只认`LANDED_STATE_
IN_AIR`（真正稳定在空中，`TAKEOFF`/`LANDING`这两个过渡态都不算"已完成"），
故意不跟`status_monitor.py`共享同一个变量/同一个宽松定义。

⚠️ 2026-08-12第三次修复（用户直接指出上面这版还是不够彻底）：`landed_
state`本质上还是个离散状态量，不管卡在哪个取值当"已完成"的判据，都只是
换了个门槛更严的死时间/死状态而已，状态量翻转的时刻和飞机实际爬到多高
之间没有必然的量化关系（不像`TAKEOFF`那版1.6秒就翻转那么夸张，但同一类
问题原则上还在）。手动起飞、手动在goal窗口打坐标之所以没事，根源不是
"用了哪个landed_state取值"，而是人肉眼确认真悬停了才去发目标点，时机
天然就晚——这才是能安全跳过mighty那个"`plan_`只有一次机会种活状态"死
坑的真正原因（`plan_`的坑本身没有被绕开，只是被"足够晚发第一个目标点"
这件事天然避开了触发条件）。真正对应的修复：不再依赖`landed_state`这个
离散信号，改成跟"前进1.1米"/"退回起飞点"两段完全同一套逻辑——直接拿
实时里程计算距离起飞目标高度（`TAKEOFF_TARGET_HEIGHT_M`，取so3ctrl/
px4ctrl/ros2_px4_stack三条起飞路径共同的默认起飞高度1.0米）还有多远，
进入`ARRIVE_TOLERANCE_M`容差就算到位，跟"死时间"或"哪个状态量翻转了"
彻底脱钩。删掉了`_STABLE_AIRBORNE_LANDED_STATES`和`mavros/extended_
state`订阅（不再需要）。这一步保留`_spin_until`的硬中止语义（超时=中
止，不能软继续）——没真正爬升到位就发下一个目标点，会直接复现上面两次
事故的根因。
"""
import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# 跟origin_setter_node.py的rotation_seg_min_disp_m(=0.5m)配套设计，见文件
# 头说明——改这个值要连带想一下origin_setter_node那边的分段阈值是否还
# 匹配。
FORWARD_DIST_M = 1.1
WAIT_SEC = 3.0
# 2026-08-12从0.15放宽到0.4——比mighty实际部署的goal_radius(0.3米)留有
# 余量，ego_planner没有独立的距离收敛判据，不该拿一个比它自己的"到达"
# 标准更严的容差卡它，详见文件头bug修复说明。这个容差是给"前进1.1米"/
# "退回起飞点"这两段**水平**到点判断校准的，不适合给下面起飞高度判断
# 复用——见TAKEOFF_ARRIVE_TOLERANCE_M的说明。
ARRIVE_TOLERANCE_M = 0.4
# so3ctrl/px4ctrl(vendor/px4ctrl_ros2/{so3ctrl,px4ctrl}/config/ctrl_param_
# fpv.yaml的takeoff_height)、ros2_px4_stack(track_dynus_traj.py的
# track_trajectory(altitude=1.0))三条起飞路径全都用1.0米当起飞目标高度，
# 这里跟它们保持一致，不是随便挑的数。
TAKEOFF_TARGET_HEIGHT_M = 1.0
# 2026-08-22新增：起飞高度判断原来复用了ARRIVE_TOLERANCE_M(0.4米)——这个
# 值是给"前进/退回"两段的水平到点判断校准的，套在起飞爬升上是个离谱的
# 容差(对1.0米目标来说相当于±40%)，实测两架飞机都在飞到0.6米(刚好卡在
# 1.0-0.4的边界)时就被判定"已到达起飞高度"，取那一刻的瞬时坐标当基准点，
# 后续"前进1.1米"/"退回起飞点"两个目标点全部继承这个0.6米。这本来不是
# 问题——ego_planner的waypointCallback以前不管term_goal.z传什么都会强制
# 改成1.0，这个过早判定被悄悄纠正了；`ego_replan_fsm.cpp`那次z轴修复
# (2026-08-22，见DEBUG_JOURNAL.md同日期记录)把z轴改成老实采纳后，这个
# 0.6米被真的执行，飞机全程只飞到0.6米、从没真正到过1.0米——是这次修复
# 暴露的第三处"依赖旧bug当安全网"的地方(前两处是GCS单点发目标点、这个
# 脚本本身)，不是新引入的问题。改成单独给起飞高度判断一个更紧的容差+
# 要求连续稳定一段时间才算数（见_spin_until_stable），不能只收紧容差
# 数字——飞机爬升过程本身就会短暂经过0.6~1.4米这个区间，单次采样判断
# 无论容差多紧都有一定概率刚好在爬升途中被采样到，"连续N秒都在容差内"
# 才能真正排除"还在爬升途中被抓拍到一帧"这种情况。
TAKEOFF_ARRIVE_TOLERANCE_M = 0.15
TAKEOFF_STABLE_HOLD_SEC = 1.0
TAKEOFF_WAIT_TIMEOUT_SEC = 60.0
ARRIVE_TIMEOUT_SEC = 30.0
POLL_INTERVAL_SEC = 0.2

# 2026-08-12实测bug修复（比ARRIVE_TOLERANCE那个更早触发的根因）：原来用
# `mavros/extended_state`的landed_state判断"起飞完成"，先后试过两版都不
# 可靠——第一版把TAKEOFF/LANDING这两个过渡态也算"完成"，landed_state从
# ON_GROUND跳到TAKEOFF几乎是起飞口令一下达就瞬间发生（1.6秒），远早于
# 真正爬升到位；改成只认IN_AIR之后仍然是"死时间"式判断的变种——不管
# 换成哪个landed_state取值当阈值，本质上都是"信一个离散状态量，而不是
# 直接量距离"，状态量什么时候翻转跟飞机实际爬到多高之间没有必然的量化
# 关系。真实事故复盘：`auto_calibration_flight.py`把"起飞完成"判定早
# 通过后，记下的基准点local=(-0.00,-0.00,+0.00)——z还在地面高度，之后发
# 的"前进1.1米"目标点z也继承了这个地面高度，直接把mighty的target点在了
# `findAandAtime()`的z_min=0.3硬下限之下——mighty这个z越界检查是硬失败
# 且不会自愈（见mighty.cpp `findAandAtime()`，往回追到`plan_`只有"收到
# 第一个目标点后的下一条里程计"这一次机会种活状态，种到还在地面的坏值
# 就永久锁死，没有第二次机会），一旦命中就永久卡死，从此再也算不出新
# 轨迹，飞机表现为"起飞、飞不动、最后又落回地面"。手动起飞+手动在goal
# 窗口打坐标之所以没事，是因为人肉眼确认真悬停了才去发目标点，时机上
# 自然晚——这次改成跟"前进1.1米"/"退回起飞点"两段完全同一套逻辑：直接
# 拿实时里程计算距离目标高度还有多远，进入容差就算到位，不再靠任何
# 离散状态量或固定秒数去猜。


class AutoCalibrationFlight(Node):
    def __init__(self, ns):
        super().__init__('auto_calibration_flight')
        self.ns = ns
        self.local_pos = None

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Odometry, f'/{ns}/dlio/odom_node/odom', self._odom_cb, best_effort)
        self.goal_pub = self.create_publisher(PoseStamped, f'/{ns}/term_goal', 10)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.local_pos = (p.x, p.y, p.z)

    def _spin_until(self, predicate, timeout_sec, what):
        """等predicate()变True，同时继续处理回调(不能用time.sleep干等，
        那样订阅回调收不到新消息，predicate永远看到的是旧数据)。超时返回
        False，调用方要能处理"没等到"这种情况，不能假装成功继续往下走。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_INTERVAL_SEC)
            if predicate():
                return True
        self.get_logger().error(f'[{self.ns}] 等待"{what}"超时({timeout_sec:.0f}秒)，自标定动作中止')
        return False

    def _spin_until_stable(self, predicate, hold_sec, timeout_sec, what):
        """跟_spin_until的区别：predicate()必须连续为True满hold_sec才算数，
        中途只要有一次False就重新计时——2026-08-22新增，专治"飞机爬升
        经过目标高度附近时被单次采样抓拍到、误判成已经到位"这种问题（见
        TAKEOFF_ARRIVE_TOLERANCE_M的说明）。超时语义跟_spin_until一样是
        硬中止，不能软继续——起飞未完成就往下走会复现2026-08-12那次
        mighty永久死锁事故的同一类根因。"""
        deadline = time.monotonic() + timeout_sec
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_INTERVAL_SEC)
            now = time.monotonic()
            if predicate():
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= hold_sec:
                    return True
            else:
                stable_since = None
        self.get_logger().error(f'[{self.ns}] 等待"{what}"超时({timeout_sec:.0f}秒)，自标定动作中止')
        return False

    def _publish_goal(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _distance_to(self, target, include_z=True):
        """include_z=False给前进/退回两段用——见_wait_for_arrival的说明，
        mighty的force_goal_z会把收到的目标点z强制改成default_goal_z，
        跟这里target算好的z（水平位移标定用不到、但一开始沿用了z0）对
        不上，用z算距离在这两段场景下不可靠，只算水平距离才对。起飞
        高度检测(TAKEOFF_TARGET_HEIGHT_M那处)明确要判断z，继续用
        include_z=True(默认值)不受影响。"""
        if self.local_pos is None:
            return float('inf')
        dx = self.local_pos[0] - target[0]
        dy = self.local_pos[1] - target[1]
        dz = (self.local_pos[2] - target[2]) if include_z else 0.0
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _wait(self, seconds):
        """纯计时等待，同时继续处理回调——不能用time.sleep干等，那样订阅
        回调收不到新消息(self.local_pos会一直是等待前的旧值)。这跟
        _spin_until是两个不同的用途：_spin_until等一个条件成立、条件一直
        不成立要报错；这里就是单纯地等一段时间，超时是预期的、正常结束
        条件，不是错误，两者不能共用同一个"超时=出错"的实现，会打印出
        误导性的ERROR日志。"""
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_INTERVAL_SEC)

    def _wait_for_arrival(self, target, timeout_sec, what):
        """等距离进入容差范围，但超时不当错误处理、不中止整个自标定
        动作——这是跟_spin_until的关键区别，2026-08-12实测bug就是原来
        这里也用_spin_until(超时=中止)，规划器自己的"到达"判据比这个
        脚本设的容差更松，导致飞机实际已经到了、规划器也已经不再主动
        修正位置，但这个脚本的检查一直不通过，干等到超时就直接放弃了
        后续"退回起飞点"这一步。到达检测本质上只是"知道飞得差不多了、
        该进入下一步了"的参考信号，不是安全关键的前提条件(不像"确认
        已经起飞"那种——没起飞就继续操作是真的有问题)，超时了大概率
        飞机已经停止移动，继续走后续步骤没有实际风险，用WARNING而不是
        ERROR记录，跟真正的中止条件区分开。

        2026-08-13第二次实测bug修复：只算水平(x,y)距离，不把z算进去。
        根因：mighty_node.cpp的terminalGoalCallbackImpl()在`force_goal_
        z=true`(实际部署值)时，会把收到的/term_goal的z**无条件**改成
        `default_goal_z`(实际部署1.5米)，不管消息里写的是什么——这个
        设计是给RViz手动点2D目标点用的(RViz点击目标z恒为0，需要一个
        默认高度)，但这个脚本发的目标点z是精确算好的当前悬停高度z0
        (比如0.6米左右)，同样会被无条件覆盖成1.5米。结果：mighty实际
        飞向的目标z=1.5米，这个脚本却拿z0算距离——z方向凭空多出
        |1.5-z0|(实测约0.89米)的固定误差，怎么飞都到不了这个脚本认为
        的"目标点"，前进、退回两段实测都卡在30秒超时、离目标"0.8~0.92
        米"（跟这个z误差量级吻合）。这个偏差是mighty服务端强加的，脚本
        这边没有办法、也不需要知道mighty实际会用哪个z——标定动作本身
        只关心水平位移(给origin_setter_node的θ*估计攒样本，是纯2D
        问题)，改成只看水平距离，从根上绕开这个z不可控的问题，不用去
        猜/同步mighty那边的default_goal_z是多少。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_INTERVAL_SEC)
            if self._distance_to(target, include_z=False) <= ARRIVE_TOLERANCE_M:
                return True
        self.get_logger().warn(
            f'[{self.ns}] 等待"{what}"超时({timeout_sec:.0f}秒)，'
            f'当前水平距目标{self._distance_to(target, include_z=False):.2f}米'
            '——不中止，直接继续下一步')
        return False

    def run(self):
        self.get_logger().info(f'[{self.ns}] 自标定动作：等待第一帧局部里程计...')
        if not self._spin_until(lambda: self.local_pos is not None, 5.0, '第一帧局部里程计'):
            return

        x_launch, y_launch, _ = self.local_pos
        takeoff_target = (x_launch, y_launch, TAKEOFF_TARGET_HEIGHT_M)
        self.get_logger().info(
            f'[{self.ns}] 等待真正爬升到起飞高度{TAKEOFF_TARGET_HEIGHT_M:.1f}米'
            f'(容差{TAKEOFF_ARRIVE_TOLERANCE_M}米，需连续稳定{TAKEOFF_STABLE_HOLD_SEC:.0f}秒，'
            f'不用固定死时间)...')
        # 这一步是安全关键的硬前提条件，必须用_spin_until_stable(超时=中止)，
        # 不能用_wait_for_arrival那种"超时也无所谓、继续往下走"的软等待——
        # 没真正爬升到位就发下一个目标点，会复现2026-08-12那次mighty永久
        # 死锁事故，见文件头/常量区说明。2026-08-22改成_spin_until_stable
        # （单次采样判断+连续稳定一段时间才算数），不再用_spin_until单次
        # 采样——原来的0.4米容差+单次采样在爬升途中就被抓拍到0.6米当成
        # "到位"，详见TAKEOFF_ARRIVE_TOLERANCE_M的说明。
        if not self._spin_until_stable(
                lambda: self._distance_to(takeoff_target) <= TAKEOFF_ARRIVE_TOLERANCE_M,
                TAKEOFF_STABLE_HOLD_SEC, TAKEOFF_WAIT_TIMEOUT_SEC,
                f'爬升到起飞高度{TAKEOFF_TARGET_HEIGHT_M:.1f}米'):
            return

        # 用真正到位那一刻的实时位置当基准点，不是爬升前的旧值——起飞
        # 目标高度只保证爬到容差范围内，实际z不一定精确等于
        # TAKEOFF_TARGET_HEIGHT_M。
        x0, y0, z0 = self.local_pos
        self.get_logger().info(
            f'[{self.ns}] 已到达起飞高度，基准点local=({x0:+.2f},{y0:+.2f},{z0:+.2f})，'
            f'原地等待{WAIT_SEC:.0f}秒')
        self._wait(WAIT_SEC)

        forward = (x0 + FORWARD_DIST_M, y0, z0)
        self.get_logger().info(f'[{self.ns}] 前进{FORWARD_DIST_M:.1f}米 -> local={forward}')
        self._publish_goal(*forward)
        self._wait_for_arrival(forward, ARRIVE_TIMEOUT_SEC, f'到达前进点(容差{ARRIVE_TOLERANCE_M}米)')

        self.get_logger().info(f'[{self.ns}] 已到达(或已超时不再等待)前进点，原地等待{WAIT_SEC:.0f}秒')
        self._wait(WAIT_SEC)

        origin = (x0, y0, z0)
        self.get_logger().info(f'[{self.ns}] 退回起飞点 -> local={origin}')
        self._publish_goal(*origin)
        self._wait_for_arrival(origin, ARRIVE_TIMEOUT_SEC, f'退回起飞点(容差{ARRIVE_TOLERANCE_M}米)')

        self.get_logger().info(f'[{self.ns}] 自标定动作完成，飞机保持悬停在起飞点上空')


def main():
    if len(sys.argv) < 2:
        print('用法: auto_calibration_flight.py <NAMESPACE>，如 auto_calibration_flight.py NX01',
              file=sys.stderr)
        sys.exit(1)
    ns = sys.argv[1]

    rclpy.init()
    node = AutoCalibrationFlight(ns)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
