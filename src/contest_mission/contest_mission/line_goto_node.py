#!/usr/bin/env python3
"""直线航点飞行节点（2026-10-13 新增）——**替代 ego_planner + traj_server**。

## 为什么要这个节点

用户 2026-10-13 的判断："当下不需要任何避障功能，但是无论什么手段，
ego-planner 都可能规划一条弯曲的轨迹出来，这是不能容忍的。"

这个判断是对的。`patches/ego_planner_lock_to_line.patch`（梯度投影锁死直线）
有**四道门**，任一不过就退回完整的 ego 行为：开关没开 / 弦长 < 0.5 m /
初值不共线 / 直线在膨胀图上被占据。而膨胀图已知会出假占据（点云每帧
resetBuffer 重建无累积、长机把僚机当障碍、竖直贯通、虚拟天花板）。更根本的是
**锁住也不等于直**：锁只冻住横向分量，初值有多少横偏就保留多少（上限
lock_max_init_dev=0.15 m）。所以只要指令还从优化器出来，就没有"保证"可言。

这个节点的直线是**结构性**的：

    p(s) = 航点[i] + û_i · (s − s_i)

没有优化器、没有代价函数、没有地图、没有梯度。输出的位置**永远精确落在折线
上**，这是代数恒等式，不是调参结果。

## 为什么能直接发 PositionCommand

`position_cmd_relay_node` 有五个 relay_mode，每个一条独立输入话题。
`formation_follower_node` 从 2026-09-20 起就是这么干的——30 Hz 直接发
`PositionCommand` 到 `formation_cmd`，relay 转给 pt4ctrl，**完全不过规划器**，
僚机的整条航线都是这么飞的。下游链路（relay → pt4ctrl → PX4）早就验证过。
本节点是同一个模式，第六个 mode：`line_goto` ← `line_goto_cmd`。

## 速度剖面：整条航线连续，不在每个航点停

用户要求"接管 fly_route，整条航线速度剖面连续"。做法是 CNC/机床那套经典的
**前瞻速度规划**（look-ahead），不是每段各自跑一个梯形：

  ① 按折线断点建弧长坐标；
  ② 给每个断点定限速：起点和终点 0；中间拐点按**转角**定——拐点处速度方向
     突变，速度增量是 |Δv| = 2·v·sin(θ/2)，要求它不超过 corner_dv_max，于是
         v_corner ≤ corner_dv_max / (2·sin(θ/2))
     再夹到 [0, cruise]。这一条顺带免费解决了"接近共线的航点不该减速"：
     θ=10° 时算出来 2.3 m/s 远超巡航，等于不限速；θ=90° 给 0.28 m/s；
     θ=180°（原地折返）给 0.2 m/s。单调、只有一个物理参数。
  ③ 前向/后向两趟扫描把限速修正成"按 accel_mps2 真能达到"：
         前向 v[i+1] = min(v[i+1], sqrt(v[i]²   + 2a·Δs))
         后向 v[i]   = min(v[i],   sqrt(v[i+1]² + 2a·Δs))
     后向那一趟是关键：它保证**提前**开始减速，短段也不会冲过头。
  ④ 运行时任意 s 处的速度上限有闭式解，不用存轨迹：
         v_ceil(s) = min(cruise,
                         sqrt(v[i]²   + 2a(s − s_i)),
                         sqrt(v[i+1]² + 2a(s_{i+1} − s)))

**终点速度恰好为 0**（断点限速里钉死的），**起步缓慢加速**（每拍最多改
accel_mps2·dt）。减速侧不额外限幅——v_ceil 的斜率本身就是 −a，两者自洽。

## 三条安全线（都是别处踩出来的）

- **设定点只沿航线前进，不许跑到飞机前面太远**：`s ≤ s_proj + max_lead_m`。
  `formation_follower_node` 的 max_lead_m 注释写着"2026-09-20 实测摔过一架"
  （一次给出好几米外的点被 PX4 当成远距离阶跃）。这里的做法比它更强：限的是
  **弧长**不是直线距离，所以设定点**始终精确落在折线上**，直线保证不被破坏。
- **航线自交必须用 last_s 消歧**：本场航线是 A→B→C→G→E→F→G→D，**G 出现两次**。
  不带 last_s 的最近点投影会 snap 到错的那个 G。复用
  `formation_follower_node.RouteRuler`（滑动窗口 + 逃逸重捕获），不另写一份。
- **到点之后主动停发**：跟 `ego_planner_traj_server_stop_after_finish.patch`
  同一个理由——pt4ctrl 的 CMD_CTRL 要靠"cmd 流中断超过 0.5 s"才会回落
  AUTO_HOVER，而 AUTO_LAND 只能从 AUTO_HOVER 触发。一直发就永远降不下来。

## 到达判据三条都要

`s ≥ 全长` 且 实测位置误差 ≤ arrival_tol_m 且 |v| ≤ arrival_v_eps_mps，
连续保持 arrival_hold_s。第三条是新加的：precision_servo 的 coordinate_goto
就是只判距离，结果减速区（6 cm）比到点球（30 cm）小 5 倍，判到点那一刻还是
满速——这个节点不重复那个错。

## 坐标系

route_xy / route_z 全部是 **UWB 全局坐标**，跟 `dlio/odom_node/odom` 的数值
同一个系（LOCALIZATION_SOURCE=uwb_imu + UWB_GLOBAL_FRAME=true，见 .env）。
节点内部不做任何坐标变换。高度维护一个目标**绝对**高度 `_target_z`：拿到测距读数时
按离地误差闭环推它（仿地，受 alt_rate_mps 限速）；拿不到就把航线给的高度当绝对高度用。
⚠️ 不要写成 `target_z = own_z`（“保持当前高度”）——那是棘轮不是保持，
2026-10-13 把长机一路沉到地面过一次，详见 _on_tick 里那段注释。
跟 formation_follower_node 的 follow_altitude_agl_m 同一套（拿不到测距读数就
保持当前高度，不猜）。
"""
import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Range
from std_msgs.msg import Float64MultiArray, String

# 航线自交时的投影消歧逻辑在这个类里，已经飞过一个月，不另写一份。
from contest_mission.formation_follower_node import RouteRuler


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LinePath:
    """折线：弧长 <-> 位置/方向。位置永远精确落在折线上。

    投影（弧长的反向查询）交给 `RouteRuler`，那边有航线自交的消歧逻辑。
    """

    def __init__(self, pts_xy: List[Tuple[float, float]], zs: List[float]):
        # 去掉重复点：零长度段会让单位方向向量除零，也会让转角无定义。
        self.pts: List[Tuple[float, float]] = []
        self.zs: List[float] = []
        #: 保留下来的点在**原始输入**里的下标。去重会打乱下标，而 stop_at 是按
        #: 原始下标给的，必须靠这张表对齐——**不能按坐标匹配**：本场航线
        #: A->B->C->G->E->F->G->D 里 G 出现两次，按坐标匹配会把两个 G 的标记并起来。
        self.src_idx: List[int] = []
        for k, (((x, y)), z) in enumerate(zip(pts_xy, zs)):
            if self.pts and math.hypot(x - self.pts[-1][0], y - self.pts[-1][1]) < 1e-6:
                continue
            self.pts.append((float(x), float(y)))
            self.zs.append(float(z))
            self.src_idx.append(k)
        self.arc: List[float] = [0.0]
        self.dirs: List[Tuple[float, float]] = []
        for i in range(len(self.pts) - 1):
            dx = self.pts[i + 1][0] - self.pts[i][0]
            dy = self.pts[i + 1][1] - self.pts[i][1]
            L = math.hypot(dx, dy)
            self.arc.append(self.arc[-1] + L)
            self.dirs.append((dx / L, dy / L))
        self.ruler = RouteRuler(self.pts) if len(self.pts) >= 2 else None

    def ok(self) -> bool:
        return len(self.pts) >= 2

    def total(self) -> float:
        return self.arc[-1]

    def seg_at(self, s: float) -> int:
        """s 落在第几段（0..n-2）。"""
        s = max(0.0, min(s, self.total()))
        # 恰好落在断点上时归到**后一段**（>= 不是 >）：归前一段的话，停留航点
        # 处的速度上限会一直是 0（那个断点限速就是 0），停留结束也起不来。
        lo, hi = 0, len(self.dirs) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if s >= self.arc[mid + 1]:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def point_at(self, s: float) -> Tuple[float, float, float, Tuple[float, float]]:
        """返回 (x, y, z, 单位方向)。x/y 精确落在折线上。"""
        s = max(0.0, min(s, self.total()))
        i = self.seg_at(s)
        ds = s - self.arc[i]
        ux, uy = self.dirs[i]
        x = self.pts[i][0] + ux * ds
        y = self.pts[i][1] + uy * ds
        L = self.arc[i + 1] - self.arc[i]
        t = (ds / L) if L > 1e-9 else 0.0
        z = self.zs[i] + (self.zs[i + 1] - self.zs[i]) * t
        return x, y, z, (ux, uy)

    def project(self, x: float, y: float, last_s: Optional[float]) -> Optional[float]:
        if self.ruler is None or not self.ruler.ok():
            return None
        return self.ruler.project(x, y, last_s)


class SpeedProfile:
    """前瞻速度规划：断点限速 + 前后向可达性修正 + 任意 s 处的速度上限。

    算法说明见模块头 ②③④。
    """

    def __init__(self, path: LinePath, cruise: float, accel: float,
                 corner_dv_max: float, stop_at: Optional[List[bool]] = None,
                 decel: Optional[float] = None):
        self.path = path
        self.cruise = max(1e-3, float(cruise))
        self.accel = max(1e-3, float(accel))
        # 刹车单独一个值。不给就等于 accel（行为跟改造前逐位相同）。
        # 物理上加减速能力本来不对称，而且"起步要柔和"跟"刹车要利落"是两件事；
        # 更要紧的是刹车距 = v²/(2·decel)，它决定离墙多远开始减速——巡航提到
        # 2.0 m/s 时 decel=0.5 的刹车距是 4.0 m，而最短航段只有 5.3 m。
        self.decel = max(1e-3, float(decel if decel is not None else accel))
        n = len(path.pts)
        # ② 断点限速
        v = [self.cruise] * n
        v[0] = 0.0
        v[n - 1] = 0.0
        for i in range(1, n - 1):
            ax, ay = path.dirs[i - 1]
            bx, by = path.dirs[i]
            dot = max(-1.0, min(1.0, ax * bx + ay * by))
            theta = math.acos(dot)                      # 0 = 直着过，pi = 原地折返
            half = math.sin(theta / 2.0)
            if half < 1e-6:
                lim = self.cruise                       # 共线，不限速
            else:
                lim = corner_dv_max / (2.0 * half)
            v[i] = max(0.0, min(self.cruise, lim))
            if stop_at and i < len(stop_at) and stop_at[i]:
                v[i] = 0.0                              # 这个航点要停（转向/等僚机）
        # ③ 前向
        for i in range(n - 1):
            ds = path.arc[i + 1] - path.arc[i]
            v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2.0 * self.accel * ds))
        # ③ 后向
        for i in range(n - 2, -1, -1):
            ds = path.arc[i + 1] - path.arc[i]
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * self.decel * ds))
        self.v = v

    def ceiling(self, s: float) -> float:
        """④ 任意 s 处的速度上限（闭式）：巡航限速 + 刹得住下一个断点。

        ⚠️ 只保留"刹得住"这一侧，**不要**再乘一个 sqrt(v[i]²+2a(s−s_i)) 的
        加速侧上限。两个原因：
          ① 它跟运行时"每拍最多改 accel·dt"的限幅重复；
          ② 它会死锁——在 stop_at 标记的航点处 v[i]=0，s 走到 arc[i] 时这一项
             算出 0，于是 v=0、s 不再前进，停留结束也永远起不来。
        后向扫描已经保证 v[i+1] ≤ sqrt(v[i+2]²+2a·Δs)，所以只约束到下一个断点
        就等价于约束到所有后续断点。
        """
        total = self.path.total()
        s = max(0.0, min(s, total))
        i = self.path.seg_at(s)
        dn = math.sqrt(max(0.0, self.v[i + 1] ** 2
                           + 2.0 * self.decel * (self.path.arc[i + 1] - s)))
        return min(self.cruise, dn)


class LineGotoNode(Node):

    #: 停留判据：s 进到断点前这么近就算"到了这个航点"（只看弧长，理由见
    #: _on_tick 里那段注释）。
    #: 到达停留点的判据：弧长差小于它就冻结。2026-10-14 从 0.05 改 0.02。
    #: 它直接决定**到点残余速度** = sqrt(2*decel*DWELL_REACH_M)：
    #:   decel=1.0 时 0.05 -> 0.316 m/s，0.02 -> 0.200 m/s。
    #: 0.316 比 90 度拐点的限速 0.4/(2*sin45)=0.283 还高，于是**停留点的过冲
    #: 反而比不停的拐点大**（实测 B/C/E/F 0.53~0.64 m vs A/G/D 0.40~0.42 m）。
    #: 下限由离散积分定：接近航点时每拍前进 v*dt≈0.2*0.02=0.004 m，
    #: 0.02 的窗口还有 5 拍，判据触发得了；再小就有漏判风险。
    DWELL_REACH_M = 0.02

    def __init__(self):
        super().__init__('line_goto')

        # ---- 航线（UWB 全局坐标）。空数组 = 待命，不发任何指令 ----
        # route_xy 是拉平的 [x0,y0,x1,y1,...]，跟 formation_follower_node 的
        # leg_route_xy 同一个编码；ROS2 参数不支持嵌套数组。
        self.declare_parameter('route_xy', [0.0])
        # 每个航点的目标**离地高度**。只给一个值 = 全程同一高度。
        self.declare_parameter('route_agl_m', [2.0])
        # 哪些航点要停（1=停）。拉平的 0/1 数组，长度同航点数；空 = 都不停。
        # 要停的场景：拐点转向（turn_at_waypoint）、长机在航点等僚机收间距。
        self.declare_parameter('stop_at', [0.0])
        # 每次下发航线自增，节点靠它判断"这是一条新航线"而不是靠比数组内容——
        # 同一条航线重飞一遍时内容一样，必须能区分开。
        self.declare_parameter('route_seq', 0)

        # ---- 速度剖面 ----
        self.declare_parameter('cruise_mps', 0.8)
        # [m/s²] 起步加速度上限。用户要求"缓慢加速"，0.5 → 0 到 0.8 m/s 用 1.6 s、
        # 走 0.64 m。减速侧由速度上限的闭式解自动给到 −accel，不另设参数。
        self.declare_parameter('accel_mps2', 0.5)
        # [m/s²] 刹车（减速）上限，<=0 = 跟 accel_mps2 相同。见 SpeedProfile 里说明。
        self.declare_parameter('decel_mps2', 0.0)
        # [m/s] 允许在拐点处瞬时发生的速度增量上限，见模块头 ②。
        self.declare_parameter('corner_dv_max_mps', 0.4)
        # [s] stop_at 标记的航点上停多久。
        self.declare_parameter('dwell_s', 2.0)

        # ---- 到达判据（三条都要，见模块头）----
        self.declare_parameter('arrival_tol_m', 0.3)
        # [m] 离地高度误差。跟 precision_servo 的 arrival_z_threshold_m 同口径。
        self.declare_parameter('arrival_z_tol_m', 0.3)
        self.declare_parameter('arrival_v_eps_mps', 0.1)
        self.declare_parameter('arrival_hold_s', 0.3)

        # ---- 安全线 ----
        # [m] 设定点最多领先飞机**弧长** max_lead_m。顶到就把 s 冻住等飞机跟上。
        self.declare_parameter('max_lead_m', 1.5)
        # [s] 里程计断流超过这么久就停发指令（交回 pt4ctrl 的 AUTO_HOVER）。
        self.declare_parameter('odom_timeout_s', 0.5)

        # [m/s] 爬升/下降速率上限。沿用 .env 里 EGO_ALT_RATE_MAX=0.6 的口径。
        # ⚠️ 为什么必须有：高度是按离地走的（target_z = own_z + (目标AGL − 当前AGL)），
        # 不限速的话整个 AGL 误差会**一拍之内**变成位置台阶。真实会踩到的场景：
        # G(16.9,12.4)->D(17.0,2.8) 这一段沿 x≈17 正好压过黄色仿地模块
        # （中心 (17,7.5)、高 0.6 m），飞机跨上模块边缘时测距读数从 2.0 跳到 1.4，
        # 于是命令高度一拍跳 0.6 m —— 正是我在 precision_servo_node 的 z 通道里
        # 挑出来的同一个毛病（那边 z 既不限速也没有速度前馈）。
        # 限速之后这一段就是正常的"仿地飞行"：以 <=0.6 m/s 爬 0.6 m、过去再降回来。
        # 顺带把 velocity.z 当前馈发出去（2026-09-15 那条教训：只给位置不给速度，
        # 等于每拍都告诉 PX4"这一点应该是静止的"）。
        self.declare_parameter('alt_rate_mps', 0.6)
        # 高度模式：'absolute'（默认）= 把航线给的高度当**绝对 z** 直接下发，
        # 测距雷达完全不进回路；'agl' = 用测距读数做离地闭环（仿地飞行）。
        #
        # 默认 'agl'：**仿地飞行是考核项**（场地那个 0.6 m 黄色模块就是考它的），
        # 用绝对 z 会让仿地功能直接失效，所以不能拿它当默认。
        # 'absolute' 只作为应急开关保留——测距链路确认有问题时可以先飞起来。
        # ⚠️ 2026-10-13 我曾把默认设成 absolute，依据是"测距读数 49 帧里 33 帧是
        # 0.000"——那是**落地之后**采的样本，飞机就在地上，0 本来就是对的。
        # 用落地数据去判传感器好坏是错的推断，已撤回。
        self.declare_parameter('alt_mode', 'agl')
        # 离地闭环的比例增益和死区。2026-10-13 实测定出来的：
        # 测距读数在 1.70~2.32 之间跳（±0.3 m 噪声/量化），而限幅是
        # alt_rate/rate_hz = 0.6/50 = 0.012 m —— 不加增益和死区的话**任何**
        # 大于 1.2 cm 的误差都让 dz 饱和，这个环就变成 bang-bang 振荡器
        # （实测 cmd_vz 全程死在 ±0.600、从不归零），均值偏负、飞机一路漏高度。
        # kp=0.5：0.1 m 误差给 0.05 m/s，要 1.2 m 误差才饱和到 0.6 m/s。
        # 死区 0.12 m：略大于观测到的噪声半幅，噪声不再驱动执行器。
        # [m] 仿地叠加允许偏离"把航线高度当绝对 z"多远。0.6 m 的仿地模块在带内，
        # 而传感器坏掉时指令**不可能**跑出这个带——这是抗饱和的硬保障。
        self.declare_parameter('alt_band_m', 1.5)
        # ---- 落后恢复 / 跟踪看门狗（2026-10-13）----
        # [s] 设定点顶在 max_lead_m 上超过这么久就判失败、停发、交还 AUTO_HOVER。
        # 原来没有这个：飞机一旦跟不上（比如在掉高度），s 就**永久冻死**，
        # 任何瞬态都变成永久停摆，SDK 那边要等满 600 秒才超时。
        self.declare_parameter('lead_stall_timeout_s', 6.0)
        # [m]/[s] 实测位置离设定点超过这么远、持续这么久就判失败。
        # 原来完全没有跟踪监督：飞机掉到地上了，节点还在推进 s。
        self.declare_parameter('track_err_max_m', 1.5)
        self.declare_parameter('track_err_hold_s', 2.0)
        self.declare_parameter('range_topic', 'mavros/hrlv_ez4_pub')
        self.declare_parameter('cmd_rate_hz', 50.0)
        self.declare_parameter('enabled', True)
        # hold=True 把 s 冻在原处（位置指令继续发，所以不会掉 CMD_CTRL）。
        # 给编队用：长机走到关键航点要等僚机把间距收回来，等多久不是固定值
        # （_wait_follower_settled 看的是僚机自报的落后量），所以不能只靠
        # dwell_s 那个固定停留。SDK 收到 'waypoint:<i>' 状态就置 hold，
        # 握手完再放开。
        self.declare_parameter('hold', False)
        # 航向：'hold' = 锁住建航线那一刻的朝向（默认）；'leg' = 机头始终指向
        # 当前航段方向。
        # ⚠️ 为什么航向必须由**这个节点**管：SDK 的 face_yaw() 靠的是
        # traj_server 的 yaw_hold 补丁（"空闲时收到新朝向就地转"），走的是
        # position_cmd 那一路——而 relay_mode='line_goto' 下那一路是被丢弃的，
        # C 步关掉规划器之后 traj_server 更是根本不存在。
        # ⚠️ 真机上 PX4CTRL_YAW_LOCK_ENABLED=true，航向指令本来就是空动作
        # （见记忆 project_yaw_lock_real_only），所以 'leg' 现在在真机上不会
        # 有可见效果；先把通路留出来。
        self.declare_parameter('yaw_mode', 'hold')

        self.add_on_set_parameters_callback(self._on_set_params)

        # ---- 状态 ----
        self._odom_xy: Optional[Tuple[float, float]] = None
        self._odom_z: Optional[float] = None
        self._odom_yaw: Optional[float] = None
        self._odom_rx: float = 0.0
        self._agl: Optional[float] = None
        self._path: Optional[LinePath] = None
        self._profile: Optional[SpeedProfile] = None
        self._seq_built: Optional[int] = None
        self._s = 0.0
        self._v = 0.0
        self._last_s_proj: Optional[float] = None
        self._last_tick: Optional[float] = None
        self._yaw_ref: Optional[float] = None
        self._dwell_until: Optional[float] = None
        self._dwell_done: set = set()
        self._arrive_since: Optional[float] = None
        self._target_z: Optional[float] = None   # 目标**绝对**高度，见 _on_tick
        self._lead_since: Optional[float] = None   # 顶在 max_lead 上起始时刻
        self._track_bad_since: Optional[float] = None
        self._state = 'idle'
        self._stop_at_flags: List[bool] = []

        self.cmd_pub = self.create_publisher(PositionCommand, 'line_goto_cmd', 10)
        self.status_pub = self.create_publisher(String, 'line_goto_status', 10)
        self.diag_pub = self.create_publisher(Float64MultiArray, 'line_goto_diag', 10)

        self.create_subscription(Odometry, 'dlio/odom_node/odom', self._on_odom, 10)
        # 离地高度：真机这条话题是 mavros/hrlv_ez4_pub（见记忆 project_altitude_chain_real）
        # ⚠️ 必须 BEST_EFFORT：mavros 的传感器话题是 best-effort 发的，
        # RELIABLE 订阅一帧都收不到（照抄 formation_follower_node 的 range 订阅）。
        self.create_subscription(
            Range, str(self.get_parameter('range_topic').value), self._on_range,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        rate = max(1.0, float(self.get_parameter('cmd_rate_hz').value))
        self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            f'line_goto 就绪：cmd_rate={rate:.0f}Hz '
            f'cruise={self.get_parameter("cruise_mps").value}m/s '
            f'accel={self.get_parameter("accel_mps2").value}m/s² '
            f'corner_dv_max={self.get_parameter("corner_dv_max_mps").value}m/s。'
            f'直线是结构性的（p=航点+û·s），不经任何优化器')

    # ------------------------------------------------------------------
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'route_xy' and len(p.value) % 2 != 0:
                return SetParametersResult(
                    successful=False,
                    reason=f'route_xy 必须是偶数个（拉平的 x,y 对），收到 {len(p.value)} 个')
        return SetParametersResult(successful=True)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._odom_xy = (p.x, p.y)
        self._odom_z = p.z
        self._odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._odom_rx = time.monotonic()

    def _on_range(self, msg: Range):
        r = float(msg.range)
        if msg.min_range <= r <= msg.max_range:   # 过量程的读数不要（同 follower）
            self._agl = r

    def _publish_status(self, state: str):
        if state == self._state:
            return
        self._state = state
        self.status_pub.publish(String(data=state))
        self.get_logger().info(f'line_goto 状态 -> {state}')

    def _publish_waypoint(self, idx: int):
        """航点到位广播 'waypoint:<i>'。**不走 _publish_status 的去重**——
        每个航点都要报一次，而去重只在状态字符串变化时才发。"""
        self.status_pub.publish(String(data=f'waypoint:{idx}'))
        self._state = f'waypoint:{idx}'

    # ------------------------------------------------------------------
    def _route_points(self) -> Tuple[List[Tuple[float, float]], List[float], List[bool]]:
        flat = list(self.get_parameter('route_xy').value or [])
        pts = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        agls = [float(v) for v in (self.get_parameter('route_agl_m').value or [])]
        if not agls:
            agls = [2.0]
        if len(agls) < len(pts):                 # 只给一个值 = 全程同高
            agls = agls + [agls[-1]] * (len(pts) - len(agls))
        raw = [float(v) for v in (self.get_parameter('stop_at').value or [])]
        stops = [bool(round(v)) for v in raw]
        if len(stops) < len(pts):
            stops = stops + [False] * (len(pts) - len(stops))
        return pts, agls, stops

    def _build(self) -> bool:
        """按当前参数建航线+速度剖面。起点用飞机**当前位置**。"""
        pts, agls, stops = self._route_points()
        if len(pts) < 1 or self._odom_xy is None:
            return False
        # 起点插飞机当前位置：route_xy 里装的是**目标**航点，这样 goto()（1 个点）
        # 和 fly_route()（N 个点）走同一套机制，不用两份代码。
        start_agl = agls[0]
        full_pts = [self._odom_xy] + pts
        full_agls = [start_agl] + agls
        full_stops = [False] + stops
        path = LinePath(full_pts, full_agls)
        if not path.ok():
            self.get_logger().warn('航线建不起来：去重后不足 2 个点（目标点就在脚下？）')
            return False
        self._path = path
        # LinePath 去重后下标会变，用它记下的 src_idx 对齐 stop_at（按下标，
        # 不按坐标——G 出现两次）。
        aligned: List[bool] = [full_stops[k] for k in path.src_idx]
        self._stop_at_flags = aligned
        self._profile = SpeedProfile(
            path,
            cruise=float(self.get_parameter('cruise_mps').value),
            accel=float(self.get_parameter('accel_mps2').value),
            corner_dv_max=float(self.get_parameter('corner_dv_max_mps').value),
            stop_at=aligned,
            decel=(float(self.get_parameter('decel_mps2').value) or None))
        self._s = 0.0
        self._v = 0.0
        self._last_s_proj = 0.0
        self._dwell_until = None
        self._dwell_done = set()
        self._arrive_since = None
        self._target_z = None
        self._lead_since = None
        self._track_bad_since = None
        self._yaw_ref = self._odom_yaw if self._odom_yaw is not None else 0.0
        self.get_logger().info(
            f'航线已建立：{len(path.pts)} 个点（含起点）、总长 {path.total():.2f} m，'
            f'断点限速 {[round(x, 2) for x in self._profile.v]}，'
            f'朝向锁定 {math.degrees(self._yaw_ref):.1f}°')
        return True

    # ------------------------------------------------------------------
    #: 判失败之后就停在这儿的两个终态。进了之后本节点不再发指令
    #: （relay 已经交还 pt4ctrl 的 AUTO_HOVER），控制回路也不该再跑。
    _TERMINAL_STATES = ('stalled', 'tracking_lost')

    def _on_tick(self):
        if not bool(self.get_parameter('enabled').value):
            self._publish_status('idle')
            return

        seq = int(self.get_parameter('route_seq').value)
        pts, _agls, _stops = self._route_points()
        if not pts:
            self._publish_status('idle')
            self._seq_built = None
            return
        if seq != self._seq_built:
            if not self._build():
                return
            self._seq_built = seq
            self._last_tick = None
            self._publish_status('executing')

        # 2026-10-16：**终态早退**——判失败之后不再跑控制回路。
        # 原来判失败只 `return`，下一拍又从头判一遍，后果有两个（当天仿真实测）：
        #   ① ERROR 以控制频率（50 Hz）刷屏，把真正的首次失败淹掉；
        #   ② `_lead_since`/`_track_bad_since` 只在"条件不满足"时清零，终态下
        #      条件一直成立，日志里"已持续 N 秒"一路涨到 105.8 s，让人以为飞机
        #      卡了 105 秒，**真实失败发生在第 6 秒**（lead_stall_timeout_s=6.0）。
        # ⚠️ 位置很关键，必须放在上面那段 `_build()` **之后**：
        # 新航线是靠 route_seq 变化在本函数里 build 的，早退一旦放到函数开头，
        # 进了终态就再也走不到 build，**新的 goto 全部石沉大海**——SDK 那边表现
        # 成"line_goto 状态=None"然后等满 60 秒超时（2026-10-16 我把它放错位置，
        # 当场让任务机起飞后飞不到物资点）。放在这里，新航线会先被 build 并
        # 把状态改成 executing，自然解除终态。
        if self._state in self._TERMINAL_STATES:
            return

        if self._path is None or self._profile is None:
            return

        # 里程计断流：停发，交回 pt4ctrl 的 AUTO_HOVER。不猜位置。
        if (self._odom_xy is None
                or time.monotonic() - self._odom_rx > float(
                    self.get_parameter('odom_timeout_s').value)):
            self.get_logger().warn('里程计断流，停发 line_goto_cmd',
                                   throttle_duration_sec=2.0)
            self._last_tick = None
            return

        now = time.monotonic()
        dt = 0.0 if self._last_tick is None else min(0.2, now - self._last_tick)
        self._last_tick = now

        total = self._path.total()
        accel = float(self.get_parameter('accel_mps2').value)
        decel = float(self.get_parameter('decel_mps2').value) or accel
        # rate_hz 在这里取，不要放到下面高度那段再取——速度段（dt<=0 时的
        # 减速收敛）先用到它，放后面就是 UnboundLocalError，而那种死法在空中
        # 看起来是"飞机正常悬停"，从现象上根本认不出来。
        rate_hz = max(1.0, float(self.get_parameter('cmd_rate_hz').value))

        # 飞机在航线上的投影（last_s 消歧：本场航线 G 出现两次，自交）
        s_proj = self._path.project(self._odom_xy[0], self._odom_xy[1], self._last_s_proj)
        if s_proj is not None:
            self._last_s_proj = s_proj
        else:
            s_proj = self._last_s_proj if self._last_s_proj is not None else 0.0

        # ---- hold：SDK 要求暂停（编队握手）。指令继续发，只是 s 不前进。 ----
        if bool(self.get_parameter('hold').value):
            dt = 0.0

        # ---- 航点到位 / 停留 ----
        # 一个分支统一处理，别拆成两段：拆开写过一版，"dwell_s=0 也报航点"那段
        # 会先把 i 加进 _dwell_done，于是 dwell_s>0 的那段 `i not in _dwell_done`
        # 不成立、停留根本不触发。
        dwell_s = float(self.get_parameter('dwell_s').value)
        if self._dwell_until is not None:
            if now < self._dwell_until:
                dt = 0.0                       # 停留中：冻住 s，位置不动
            else:
                self._dwell_until = None
        else:
            for i, flag in enumerate(self._stop_at_flags):
                if not flag or i in self._dwell_done:
                    continue
                # 判据**只看弧长**，不看速度。原来还要求 v <= 0.05，实测一次都
                # 没触发过：刹车曲线 v=sqrt(2a·剩余) 在离散积分下到不了 0——
                # 最后一拍的剩余距离约 v·dt，解出来 v 停在 0.07~0.1 就被 s 跨过去
                # 了。而 stop_at 标记的断点限速本来就是 0，剖面保证到这儿速度已经
                # 很小，直接冻住就行；实测速度打进日志，真有意外看日志能发现。
                if self._s < self._path.arc[i] - self.DWELL_REACH_M:
                    continue
                self._dwell_done.add(i)
                # 不管停不停都要广播：单机任务段 dwell_s=0「全不停」，但 SDK 的
                # 进度打印和编队解散看门狗仍要知道过了哪个点。
                self._publish_waypoint(i)
                if dwell_s > 0.0:
                    self._dwell_until = now + dwell_s
                    dt = 0.0
                    self.get_logger().info(
                        f'航点 {i}/{len(self._path.pts) - 1} 到位（s={self._s:.2f}，'
                        f'此刻速度 {self._v:.3f} m/s），停 {dwell_s:.1f} s')
                else:
                    self.get_logger().info(
                        f'航点 {i}/{len(self._path.pts) - 1} 过点（s={self._s:.2f}，'
                        f'速度 {self._v:.3f} m/s），不停')
                break

        # ---- 速度剖面推进 ----
        # dt==0 表示这一拍被 hold 或航点停留冻住了。此时弧长不前进，速度前馈
        # 也必须跟着往 0 收——否则会发出"位置钉在这儿、但速度 0.25 m/s"这种自相
        # 矛盾的指令（实测停留触发点的速度就是 0.249 m/s，整个 2 秒都在发它）。
        if dt <= 0.0:
            self._v = max(0.0, self._v - decel / rate_hz)
        v_ceil = self._profile.ceiling(self._s)
        # 加速受限幅——"缓慢加速"就是这一条。
        v_new = min(self._v + accel * dt, v_ceil)
        # 减速**也**限幅。连续意义下 v_ceil 的斜率恰好是 −accel，所以这个下限
        # 正常不会起作用；它挡的是离散积分的毛刺：刹车曲线 v=sqrt(2a·剩余距离)
        # 在终点处斜率无穷，最后一拍实测会冲到 0.693 m/s²（v 从 0.025 掉到
        # 0.011，物理上微不足道，但指令速度不该有台阶）。
        # ⚠️ 代价：v_ceil 真的突降时这个下限会压住刹车，于是可能略微冲过拐点
        # 限速。后向扫描保证了 v_ceil 在连续意义下刹得住，所以只会差一个离散
        # 量级；真要是差很多，说明 accel_mps2 设得比飞机能力还大。
        self._v = max(0.0, max(v_new, self._v - decel * dt))
        # 设定点不许领先飞机太多（弧长口径，所以位置**仍然精确落在折线上**）
        s_cap = s_proj + float(self.get_parameter('max_lead_m').value)
        self._s = min(total, min(self._s + self._v * dt, s_cap))

        # point_at 的第三个返回值就是该处的目标**离地高度**（LinePath 的 zs
        # 装的是 route_agl_m，不是 odom z）。
        x, y, target_agl, (ux, uy) = self._path.point_at(self._s)

        # ---- ② 落后恢复：顶在 max_lead_m 上不能无限期冻着 ----
        lead_m = float(self.get_parameter('max_lead_m').value)
        if self._s - s_proj >= lead_m - 1e-3 and self._s < total - 1e-6:
            if self._lead_since is None:
                self._lead_since = now
            elif now - self._lead_since > float(
                    self.get_parameter('lead_stall_timeout_s').value):
                self.get_logger().error(
                    f'设定点顶在 max_lead_m={lead_m} m 上超过 '
                    f'{now - self._lead_since:.1f} s 仍不前进（s={self._s:.2f} '
                    f's_proj={s_proj:.2f}）——飞机跟不上，判失败、停发指令交还 '
                    f'pt4ctrl 的 AUTO_HOVER')
                self._publish_status('stalled')
                return
        else:
            self._lead_since = None

        # ---- ③ 跟踪看门狗：飞机离设定点太远就判失败，别自己往前推 ----
        # 必须带上 z。原来只算水平，实测指令 2.04 m、飞机趴在 0.054 m，
        # 2 米的高度误差持续 200 秒，看门狗一声没响。
        # 这里用**上一拍**的 self._target_z（本拍的 target_z 要到下面高度段才算
        # 出来，提前引用就是 UnboundLocalError）——差 20 ms，对 2 秒的判据无影响。
        _tz = self._target_z
        _oz = self._odom_z
        _dz_err = (abs(_tz - _oz) if (_tz is not None and _oz is not None) else 0.0)
        track_err = math.sqrt((x - self._odom_xy[0]) ** 2
                              + (y - self._odom_xy[1]) ** 2 + _dz_err ** 2)
        if track_err > float(self.get_parameter('track_err_max_m').value):
            if self._track_bad_since is None:
                self._track_bad_since = now
            elif now - self._track_bad_since > float(
                    self.get_parameter('track_err_hold_s').value):
                self.get_logger().error(
                    f'实测位置离设定点 {track_err:.2f} m，已持续 '
                    f'{now - self._track_bad_since:.1f} s——判跟踪失败、停发指令。'
                    f'（设定点 ({x:.2f},{y:.2f}) 实测 ({self._odom_xy[0]:.2f},'
                    f'{self._odom_xy[1]:.2f})'
                    # 分量必须打全：2026-10-16 长机在 E 点判失败那次，只有
                    # 合成的 2.64 m 可看，水平/垂直各占多少全靠反推几何去猜。
                    f'｜水平 {math.hypot(x - self._odom_xy[0], y - self._odom_xy[1]):.2f} m'
                    f'、垂直 {_dz_err:.2f} m（指令z={_tz if _tz is None else round(_tz, 2)}'
                    f' 实测z={_oz if _oz is None else round(_oz, 2)}'
                    f' 测距agl={self._agl if self._agl is None else round(self._agl, 2)}'
                    f' 航线agl={target_agl:.2f}））')
                self._publish_status('tracking_lost')
                return
        else:
            self._track_bad_since = None

        # ---- 高度：按离地走，跟 formation_follower_node 同一套 ----
        own_z = self._odom_z if self._odom_z is not None else 0.0
        alt_rate = float(self.get_parameter('alt_rate_mps').value)
        dz_max = alt_rate / rate_hz
        vz = 0.0
        # ---- ① 高度：z 是**路径的一部分**，仿地只是有界叠加 ----
        # 2026-10-13 第三版。前两版都坏，而且是对称的两种坏法：
        #   · `self._target_z += dz`（积分）=> **windup**：落地后测距读数无效、
        #     误差恒为正，指令高度无界爬到 40 m（飞机在地上）。
        #   · `own_z + dz`（锚回当前高度）=> **棘轮**：指令永远只比飞机高 1.2 cm，
        #     飞机不跟就跟着飞机往下走，掉了永远爬不回来。实测 own_z=0.054、
        #     target_z=0.066、agl=0.22，指令就这么贴在地面上。
        # 这一版不用反馈环，直接算**绝对目标**：
        #     脚下地面绝对高度 = own_z − 实测离地
        #     目标绝对高度     = 脚下地面 + 航线要求的离地
        # 它天然就是仿地（地面抬高 0.6 m，目标跟着抬 0.6 m），没有积分器所以不会
        # windup，是绝对值所以不会棘轮、能把高度爬回来。再夹进 ±alt_band_m 作为
        # 传感器坏掉时的硬保障。ego-planner 当年就是让 z 由规划轨迹直接给绝对值、
        # 从不跑 AGL 反馈环——这一版是回到那个结构。
        band = float(self.get_parameter('alt_band_m').value)
        if str(self.get_parameter('alt_mode').value) == 'agl' and self._agl is not None:
            ground_z = own_z - self._agl
            want = ground_z + target_agl
            want = max(target_agl - band, min(target_agl + band, want))
        else:
            # 没有测距读数（或显式要绝对高度）：按平地算，地面 z≈0。
            # odom 已经是绝对系，所以航线给的离地高度直接当绝对 z 用。
            want = target_agl
        if self._target_z is None:
            self._target_z = own_z          # 起步从当前高度开始，避免一拍阶跃
        step = max(-dz_max, min(dz_max, want - self._target_z))
        self._target_z += step
        # ⚠️ 垂直方向**不发速度前馈**（2026-10-13 实测定出来的）。
        # vz 原来取 step*rate_hz，而 step 一碰限幅就满，所以 agl 任何跳动都让 vz
        # 饱和到 ±0.6 m/s；pt4ctrl 把它整包转给 PX4 位置环当前馈，PX4 就真按
        # 0.6 m/s 去动——实测长机因此在巡航中掉到 0.918 m 再爬回来。
        # 本质上那是"把测距噪声微分一遍再喂给执行器"。
        # 水平那两路前馈要留：它们是沿已知路径的巡航速度、有真实物理含义
        # （2026-09-15 补它们是为了让 PX4 知道"这点不是要停在这"）；垂直这一路
        # 没有对应的"规划速度"，位置指令本身已经够稳。
        vz = 0.0
        target_z = self._target_z

        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'world'
        cmd.position.x = float(x)
        cmd.position.y = float(y)
        cmd.position.z = float(target_z)
        cmd.velocity.x = float(ux * self._v)
        cmd.velocity.y = float(uy * self._v)
        cmd.velocity.z = float(vz)
        a_now = accel if self._v < v_ceil - 1e-6 else 0.0
        cmd.acceleration.x = float(ux * a_now)
        cmd.acceleration.y = float(uy * a_now)
        cmd.acceleration.z = 0.0
        if str(self.get_parameter('yaw_mode').value) == 'leg':
            cmd.yaw = float(math.atan2(uy, ux))     # 机头指向当前航段
        else:
            cmd.yaw = float(self._yaw_ref if self._yaw_ref is not None else 0.0)
        cmd.yaw_dot = 0.0

        # ---- 到达：三条都要 ----
        dist_end = math.hypot(self._path.pts[-1][0] - self._odom_xy[0],
                              self._path.pts[-1][1] - self._odom_xy[1])
        z_err = (abs(self._agl - target_agl) if self._agl is not None else 0.0)
        arrived = (self._s >= total - 1e-6
                   and dist_end <= float(self.get_parameter('arrival_tol_m').value)
                   and z_err <= float(self.get_parameter('arrival_z_tol_m').value)
                   and self._v <= float(self.get_parameter('arrival_v_eps_mps').value))
        if arrived:
            if self._arrive_since is None:
                self._arrive_since = now
            if now - self._arrive_since >= float(self.get_parameter('arrival_hold_s').value):
                self._publish_status('completed')
                # 主动停发：pt4ctrl 的 CMD_CTRL 要靠 cmd 流中断才回落 AUTO_HOVER，
                # 而 AUTO_LAND 只能从 AUTO_HOVER 触发。一直发就永远降不下来。
                # 同 ego_planner_traj_server_stop_after_finish.patch 的理由。
                return
        else:
            self._arrive_since = None

        self.cmd_pub.publish(cmd)

        diag = Float64MultiArray()
        # 2026-10-13 加了后四项：高度环守不住时，只看 s/v 那几项看不出任何东西，
        # 必须能同时看到"目标AGL / 实测AGL / 目标z / 实际z"四个量。
        diag.data = [float(self._s), float(total), float(self._v), float(v_ceil),
                     float(s_proj), float(dist_end),
                     float(self._s - s_proj),
                     float(target_agl),
                     float(self._agl) if self._agl is not None else float('nan'),
                     float(target_z), float(own_z)]
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = LineGotoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException:
        # 2026-10-14 加（line_goto_node 同款缺口）：原来只接 KeyboardInterrupt，
        # 其余异常会穿过 spin() 落到 finally，节点被干净拆掉、进程**静默退出**，
        # 异常从没被打印。空中表现就是"飞机正常悬停"，根因只能事后 ps 推断。
        # 用 BaseException 而非 Exception：SystemExit/GeneratorExit 这类更难查。
        import traceback
        try:
            node.get_logger().fatal('line_goto_node 因未捕获异常退出——完整栈：\n'
                                    + traceback.format_exc())
        except Exception:
            pass
        traceback.print_exc()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
