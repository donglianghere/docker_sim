#!/usr/bin/env python3
"""编队跟随节点：SUPPLY角色（或任何被指定为"跟随者"的一方）机载常驻的
实时控制回路，持续读长机（leader）的里程计，沿长机实际飞过的轨迹按
固定弧长回溯算出"我现在应该在哪"，把这个目标点发给现成的
`rviz_goal_world`入口（`ego_planner_bridge/rviz_goal_bridge_node.py`
第126行`self.create_subscription(PoseStamped, 'rviz_goal_world', ...)`，
接口已确认存在），复用ego_planner现有的目标点执行链路，不新造一套
"发目标点"的机制。

对应《2026大赛任务系统全流程任务仿真实现方案.md》1.4/1.4.1/1.4.2节，
《...可执行实施清单.md》阶段A的A3项。

==== 为什么这个控制回路必须放在飞行栈里，不能放SDK跨网络实现（1.4.2节）====
编队跟随需要"读长机实时位置→算目标点→发新目标"以5-10Hz频率持续跑，
如果这个回路放在选手程序（跑在跟无人机有网络延迟的地面站/选手容器）
里，每个控制周期都要承受一次往返延迟，跟随响应会明显滞后、变得不
稳定——这是1.0节"决策可以有延迟，控制不能有延迟"这条原则的具体应用。
所以这个回路常驻在**跟随角色所在那架飞机自己的flight-stack容器**里，
跟长机的odom是同一个局域网/同一台宿主机内部的话题通信，没有额外的
跨网络延迟。SDK那边`sdk.start_formation_follow()`只是"发一次调用，
告诉这个节点该跟谁、用什么参数"，不做任何持续订阅/计算——控制回路
本身完全在这里。

==== 关于坐标系：uwb_imu模式下不需要任何跨机坐标转换 ====
⚠️ **2026-09-20订正：下面这段原来的说法是错的，实测不成立。**

原文曾写"uwb_imu模式下两机的odom就是同一个UWB锚点下的全局系坐标，
长机局部系=跟随者局部系=世界系，天然对齐……直接消费长机odom的x/y
当成跟随者自己局部系下的坐标使用，不做任何TF查询"。按这个假设实现
之后连续出了三次事故（僚机摔机、僚机自己慢慢降落、僚机跟着一条错位
的轨迹飞被误判成"抄近道"），实测数据把它证伪了：

  飞机   odom读数            Gazebo真值           world->odom平移
  NX01   (-9.40,  0.01, 0.91) (-7.97,-10.07,1.31)  (+1.43,-10.08,+0.06)
  NX02   ( 5.97, 13.56, 0.27) ( 4.49,  3.54,0.05)  (-1.48,-10.01,+0.06)

**每架飞机的odom原点锚在它自己的起降点上**（NX01在(1.5,-10)、NX02在
(-1.5,-10)），两机x差3米、z也差0.64米。θ*旋转在uwb_imu下确实是0
（这部分原文没说错），但**平移不是0**，而编队跟随恰恰对平移敏感。

因此现在的做法是：长机轨迹点先用TF（`<长机ns>/odom` -> 自己的
`<ns>/odom`，经world串联）变换到跟随者自己的坐标系，再做弧长回溯。
变换查一次缓存起来（两个系都是开机时锁定的静态系）；查不到就跳过
这一帧不记点——宁可暂时不跟，也不能跟着一条平移过的轨迹飞。
高度不走轨迹、也不跟长机（见`follow_altitude_agl_m`参数说明）：按机载
定高雷达读数保持固定AGL。这样既不受跨机z基准差影响，也不会取到轨迹里
长机还停在地面的那一段，更重要的是——长机飞过仿地模块时会抬升世界系
高度，僚机在平地上不该跟着抬升，它应该在自己脚下的地形上保持同样的
离地高度。

==== 节点常驻但默认不生效（1.4.2节）====
跟其它flight-stack节点一样，这个节点每架飞机的容器里都会常驻起来
（角色是运行时决定的，不能只给某一架飞机装这个能力，见1.5节双机
对称性原则），但默认`enabled=False`，定时器空转不发布任何东西。
需要收到一次"启用"指令（标准`~/set_parameters`，带上`enabled=True`+
`leader_namespace=<长机命名空间>`）才开始真正工作。这样即使选手程序
决定"什么时候开始编队"这件事本身有延迟，也只影响"编队什么时候开始"
这个决策时机，不影响编队跟随本身的控制质量。

==== 接口设计：几何队形/控制策略都是策略模式的可插拔接口（1.4.1节）====
`FormationGeometry`（几何队形）和`FormationControlStrategy`（控制
策略）是两条独立的可替换轴，这次任务只**实现**纵向(tandem)+长机-
僚机(leader_follower)这一种组合，但接口留出`LineAbreastGeometry`/
`VirtualStructureStrategy`两个占位子类，以后要换队形/换策略只需要
新增子类，不用碰节点主循环逻辑。
"""
import bisect
import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from quadrotor_msgs.action import FormationFollow
from quadrotor_msgs.msg import PositionCommand


def _seg_progress(a, b, px: float, py: float) -> float:
    """点(px,py)在线段 a->b 上的归一化投影（0=在 a，1=在 b，>1=已越过 b）。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    if den < 1e-9:
        return 1.0
    return ((px - a[0]) * dx + (py - a[1]) * dy) / den


def _ang_diff(a: float, b: float) -> float:
    """两个角度之间的最小夹角（弧度，恒为非负）。"""
    if a is None or b is None:
        return 0.0
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy


# ============================================================
# 第1部分：LeaderPathBuffer——全节点逻辑最复杂的部分，独立成类、
# 不跟ROS节点逻辑混在一起写，方便单独写synthetic单元测试（见文件末尾
# 的`if __name__ == '__main__':`自测代码块，或同目录
# `test_leader_path_buffer.py`）。
# ============================================================

class LeaderPathBuffer:
    """缓存长机走过的轨迹点（只存x/y，z不参与——这个类只负责"沿哪条
    水平路径回溯固定弧长"这一件事，跟随者目标点的高度由节点层面另外
    决定，见FormationFollowerNode里对`z`的处理），支持按弧长回溯查询
    "长机实际走过的折线上，往回数follow_distance_m米的那个点"。

    ⚠️ 全节点最容易犯错、之前版本设计错过一次的坑（见方案1.4节"必须
    沿长机实际走过的弧线追、不能走直线抄近道"）：`point_at_arc_length_
    behind()`不能简单地"从终点沿直线方向后退follow_distance_m米"，
    必须**沿折线本身**（逐段累加线段长度）往回走，遇到直角拐弯/连续
    转弯时，直线回溯和沿折线回溯算出来的点会明显不同——直线回溯会让
    跟随者在长机刚过一个拐角时直接抄近道切过去，不会真的沿着长机走
    过的路径follow，这正是纵向编队(tandem)选这种几何队形的意义所在
    （"跟随目标只需要沿一条一维的路径弧长追踪"），如果实现上抄了近道，
    这个设计选择就完全落空了。
    """

    def __init__(self, min_point_gap_m: float = 0.30, max_buffer_length_m: float = 150.0):
        # min_point_gap_m：odom回调频率可能远高于这个节点的控制频率
        # （DLIO/uwb_imu融合通常几十到上百Hz），如果每一帧odom都append
        # 一个点，缓冲区会迅速堆积大量几乎重合的点，白白浪费内存/计算，
        # 对回溯精度也没有任何帮助（两点间距远小于任何有意义的
        # follow_distance_m）。所以两点间距小于这个阈值时直接忽略新点
        # 不追加。
        #
        # 2026-09-21：默认值从0.05米改成0.30米。0.05米这个值有一个致命
        # 问题——它比定位噪声还小。长机位置来自`uwb/pose_abs`，仿真里
        # `uwb_ground_truth_node`的`range_noise_std`默认0.05米/轴，相邻
        # 两帧的噪声位移是Rayleigh分布、均值约0.089米，**恒大于0.05米的
        # 门限**，于是长机停在地上一动不动时，每一帧噪声都被当成"走了一
        # 小段"追加进来，缓冲区以大约1.5米/秒的速率虚增"轨迹长度"。
        # 实测后果：长机还没起步，`total_length()`就已经有40多米，`hold`
        # 阶段（等长机起步）形同不存在，那条"轨迹"其实是起飞点周围的一团
        # 随机游走，僚机在追一个没有物理意义的参考点——之前反复出现的
        # "抄近道""左摇右晃""落后量42米不收敛"全部源于此。
        # 0.30米比噪声位移的分布尾部（约0.25米）还高一截，同时对1.0米/秒
        # 的巡航速度意味着每0.3秒采一个点，对折线弧长精度完全够用。
        # 这个值必须随定位噪声一起调：**门限要明显大于噪声位移**。
        self._min_point_gap_m = float(min_point_gap_m)
        # max_buffer_length_m：缓冲区裁剪上限——只需要保留"最新点往回数
        # 到最大可能follow_distance_m还有余量"这么长的历史，更老的点
        # 对当前查询毫无用处，裁掉避免无限增长。
        # 2026-09-21：从60米改成150米。僚机是沿**整条**已走轨迹往前推进
        # 的，不是只看最近一小段——这次的4点航线全程约76米，60米的缓冲区
        # 会把僚机正在走的那一段裁掉。150米在0.30米采点间距下也只有500个
        # 点，开销可以忽略；真的不够时_trim内部逻辑本身也不
        # 会出错，只是回溯到缓冲区起点就停住（见point_at_arc_length_
        # behind的兜底行为）。
        self._max_buffer_length_m = float(max_buffer_length_m)
        # 三个并行列表，索引对齐：_xs[i]/_ys[i]/_ts[i]是第i个轨迹点。
        self._xs: List[float] = []
        self._ys: List[float] = []
        self._zs: List[float] = []   # 2026-09-20：轨迹跟随要"高度相同"，z也要沿轨迹取
        self._ts: List[float] = []
        # 累积弧长表的缓存。2026-09-28 加：航线基准下一拍要二分取好几次
        # 参考点（每次都要一张弧长表），缓冲区又有几百个点，不缓存的话这
        # 一段是纯浪费。只在缓冲区变动时作废。
        self._arc_cache: Optional[List[float]] = None

    def __len__(self) -> int:
        return len(self._xs)

    def clear(self) -> None:
        """清空缓冲区——切换长机(leader_namespace变了)时必须调用，
        否则旧长机的轨迹点会被新长机的跟随逻辑误用（见FormationFollower
        Node._sync_leader_subscription）。"""
        self._xs.clear()
        self._ys.clear()
        self._zs.clear()
        self._ts.clear()
        self._arc_cache = None

    def append(self, x: float, y: float, z: float, t: float) -> None:
        """往缓冲区追加长机的一个新轨迹点。t是时间戳（秒，浮点数，用
        `Time.sec + Time.nanosec*1e-9`这种形式换算），这次任务的弧长
        回溯算法本身不需要t（回溯只看空间距离，不看时间），保留这个
        参数是为了以后如果要按时间窗口而不是弧长裁剪缓冲区、或者需要
        估算长机速度时能直接用，不需要改调用方的接口。"""
        if self._xs:
            last_x, last_y = self._xs[-1], self._ys[-1]
            if math.hypot(x - last_x, y - last_y) < self._min_point_gap_m:
                return  # 跟上一个点太近，不追加（见__init__注释）
        self._xs.append(x)
        self._ys.append(y)
        self._zs.append(z)
        self._ts.append(t)
        self._arc_cache = None
        self._trim()

    def _trim(self) -> None:
        """裁掉缓冲区起始端超出max_buffer_length_m的旧点，见__init__
        注释。从头部开始丢点，直到"从当前起点到终点的总弧长"不超过
        上限——每次只在超限时丢一个点，不是整体重建，均摊开销很小。"""
        while len(self._xs) > 2:
            total = self._total_length()
            if total <= self._max_buffer_length_m:
                break
            self._xs.pop(0)
            self._ys.pop(0)
            self._zs.pop(0)
            self._ts.pop(0)
            self._arc_cache = None

    def last_xy(self) -> Optional[Tuple[float, float]]:
        """长机当前位置（缓冲区最后一个点）。拐点前馈要用。"""
        if not self._xs:
            return None
        return (self._xs[-1], self._ys[-1])

    def total_length(self) -> float:
        """缓冲区里这条折线的总水平弧长（米）。跟随者据此判断长机是否
        已经真正走起来——太短就原地等，别被起飞点附近那一小簇点拉走。"""
        return self._total_length()

    def _total_length(self) -> float:
        if len(self._xs) < 2:
            return 0.0
        return self._arc_table()[-1]

    def point_at_arc_length_behind(self, follow_distance_m: float) -> Optional[Tuple[float, float]]:
        """从缓冲区里"最新的那个点"开始，**沿折线本身**往回走
        follow_distance_m米弧长，返回那个位置的(x, y)。

        算法：从最后一个点开始，逐段（相邻两点连成的线段）往回累加
        线段长度，一旦累加到的长度跨过了follow_distance_m，说明目标点
        落在"当前正在检查的这一段"上，按剩余弧长在这一段内做线性插值
        算出精确坐标——这一步是关键：插值只在**单个折线段内部**做
        （两点之间本来就是直线，段内插值不是"抄近道"，是折线本身的
        定义），不是跨越多个点、跨越拐角去直线插值。

        兜底行为：
        - 缓冲区为空 -> 返回None（还没收到过长机odom，调用方应该跳过
          这一次，不发布任何目标）。
        - 缓冲区只有1个点 -> 直接返回这个点（没有"路径"可言，长机刚
          起步）。
        - follow_distance_m超过缓冲区里能表达的总弧长（比如长机刚起飞
          没多久，还没飞出follow_distance_m这么远）-> 返回缓冲区里最
          老的那个点（缓冲区能追溯到的最远位置），不是报错也不是外插
          到折线以外——这个阶段跟随者会暂时"贴"在长机刚起飞时的位置
          附近，等长机飞得足够远之后自然恢复到正常的固定跟随距离，
          这是合理的、安全的过渡行为，不需要特殊处理。
        """
        n = len(self._xs)
        if n == 0:
            return None
        if n == 1:
            return (self._xs[0], self._ys[0])

        d = float(follow_distance_m)
        if d <= 0.0:
            return (self._xs[-1], self._ys[-1])

        accumulated = 0.0
        # i从最后一个点的下标(n-1)往回走到1，每一步检查线段
        # [points[i-1], points[i]]（points[i]离终点更近，points[i-1]
        # 更远/更老）。
        for i in range(n - 1, 0, -1):
            x1, y1 = self._xs[i], self._ys[i]        # 段的"近端"（离终点近）
            x0, y0 = self._xs[i - 1], self._ys[i - 1]  # 段的"远端"（离终点远）
            seg_len = math.hypot(x0 - x1, y0 - y1)
            if seg_len < 1e-9:
                continue  # 退化的零长度段（理论上append时已经过滤，兜底跳过）
            if accumulated + seg_len >= d:
                remaining = d - accumulated
                frac = remaining / seg_len  # 0~1，从近端往远端走的比例
                x = x1 + (x0 - x1) * frac
                y = y1 + (y0 - y1) * frac
                return (x, y)
            accumulated += seg_len

        # 走完整条缓冲区还没凑够d，说明follow_distance_m超出缓冲区
        # 能表达的总弧长，见docstring"兜底行为"第3条。
        return (self._xs[0], self._ys[0])

    def sample_behind(self, follow_distance_m: float):
        """轨迹跟随用的完整采样（2026-09-20新增）。

        在`point_at_arc_length_behind`只给(x, y)的基础上，补齐轨迹跟随
        真正需要的另外三样东西：

        - **z**：跟x/y一样沿轨迹插值。用户要求"高度相同"——指的是跟长机
          飞到该位置时的高度相同，不是跟长机此刻的高度相同（长机可能已经
          在爬升/下降了）。
        - **速度前馈(vx, vy)**：方向取该点所在折线段的切线方向，大小取
          长机飞过这一段时的实际速度（段长/段耗时，所以缓冲区必须存时间
          戳）。没有速度前馈，pt4ctrl只能靠位置误差追，跟随会持续滞后。
        - **yaw**：取切线方向，也就是"沿航迹前进的方向"——一字纵队里
          僚机机头应该朝着自己前进的方向。

        返回 (x, y, z, vx, vy, yaw)，缓冲区为空时返回None。
        """
        n = len(self._xs)
        if n == 0:
            return None
        if n == 1:
            return (self._xs[0], self._ys[0], self._zs[0], 0.0, 0.0, 0.0)

        d = max(0.0, float(follow_distance_m))
        accumulated = 0.0
        for i in range(n - 1, 0, -1):
            x1, y1, z1, t1 = self._xs[i], self._ys[i], self._zs[i], self._ts[i]
            x0, y0, z0, t0 = self._xs[i - 1], self._ys[i - 1], self._zs[i - 1], self._ts[i - 1]
            seg_len = math.hypot(x0 - x1, y0 - y1)
            if seg_len < 1e-9:
                continue
            if accumulated + seg_len >= d:
                frac = (d - accumulated) / seg_len   # 0~1，从近端往远端
                x = x1 + (x0 - x1) * frac
                y = y1 + (y0 - y1) * frac
                z = z1 + (z0 - z1) * frac
                # 切线方向：从远端指向近端，也就是长机前进的方向
                dx, dy = x1 - x0, y1 - y0
                yaw = math.atan2(dy, dx)
                dt = t1 - t0
                speed = seg_len / dt if dt > 1e-6 else 0.0
                return (x, y, z, speed * math.cos(yaw), speed * math.sin(yaw), yaw)
            accumulated += seg_len

        # 弧长超出缓冲区总长：贴在最老的点上（见上面的兜底说明），
        # 速度给0，yaw取最老那一段的方向。
        dx, dy = self._xs[1] - self._xs[0], self._ys[1] - self._ys[0]
        return (self._xs[0], self._ys[0], self._zs[0], 0.0, 0.0, math.atan2(dy, dx))

    def _arc_table(self) -> List[float]:
        """累积弧长表：_arc[i] 是从缓冲区起点走到第i个点的弧长。"""
        if self._arc_cache is not None and len(self._arc_cache) == len(self._xs):
            return self._arc_cache
        arc = [0.0]
        for i in range(1, len(self._xs)):
            arc.append(arc[-1] + math.hypot(self._xs[i] - self._xs[i - 1],
                                            self._ys[i] - self._ys[i - 1]))
        self._arc_cache = arc
        return arc

    def _point_at_arc(self, arc: List[float], s: float):
        """弧长坐标 s 处的 (x, y, z, yaw切线, 该段速度)。"""
        s = max(0.0, min(s, arc[-1]))
        i = 1
        while i < len(arc) - 1 and arc[i] < s:
            i += 1
        seg = arc[i] - arc[i - 1]
        frac = 0.0 if seg < 1e-9 else (s - arc[i - 1]) / seg
        x = self._xs[i - 1] + (self._xs[i] - self._xs[i - 1]) * frac
        y = self._ys[i - 1] + (self._ys[i] - self._ys[i - 1]) * frac
        z = self._zs[i - 1] + (self._zs[i] - self._zs[i - 1]) * frac
        yaw = math.atan2(self._ys[i] - self._ys[i - 1], self._xs[i] - self._xs[i - 1])
        dt = self._ts[i] - self._ts[i - 1]
        speed = seg / dt if dt > 1e-6 else 0.0
        return x, y, z, yaw, speed

    # 投影搜索窗口（米）：只在上次弧长的 ±这个范围内找。窗口跟着飞机走，
    # 天然排除闭合回路另一端那个分支。
    SEARCH_WINDOW_M = 5.0
    # 逃逸门限（米）：窗口内最近的点也离飞机这么远，说明飞机根本不在这一段
    # 附近（定位跳变、重新入列、轨迹被裁剪…），退回全局搜索重新捕获。
    # 没有这条的话窗口会变成闩锁——2026-09-28 实测：用"离上次最近"做消歧义，
    # 一旦锁在错误分支上就永远出不来，落后量冻在 10.64 m、两机互相等死。
    REACQUIRE_DIST_M = 2.0

    def project_arc_length(self, own_x: float, own_y: float,
                           last_s: Optional[float] = None) -> Optional[float]:
        """僚机当前位置投影到长机折线上，返回它"走到哪了"的弧长坐标。

        只用水平坐标（用户要求：距离判断只考虑水平距离，不考虑垂直距离）。

        `last_s`（2026-09-28 新增）：上一次的投影弧长，用来消歧义。
        航线绕一圈回到起降点时长机轨迹是闭合的，僚机停在起降点附近时，
        轨迹**起点**和**末端**到它的距离几乎一样近，只比距离的话选哪个是
        任意的——实测长机返航到家那一刻，僚机的投影 snap 回轨迹起点，
        lag = s_limit_gap − s_proj 瞬间变成 +72 米。这个数喂给入列判据、
        action 反馈、以及长机的照顾模式（看到 72 会直接切 hold 把长机停死）。

        做法是**滑动窗口 + 逃逸**：只在 last_s ± SEARCH_WINDOW_M 内搜，窗口
        跟着飞机走，自然排除另一端；窗口内最近距离超过 REACQUIRE_DIST_M 就
        退回全局搜索重新捕获。第一版用的是"在相近候选里选离上次最近的"，
        那个没有逃逸路径、会闩死（见 REACQUIRE_DIST_M 的注释）。
        """
        n = len(self._xs)
        if n < 2:
            return None
        arc = self._arc_table()
        cands = []
        for i in range(1, n):
            x0, y0, x1, y1 = self._xs[i - 1], self._ys[i - 1], self._xs[i], self._ys[i]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((own_x - x0) * dx + (own_y - y0) * dy) / seg2))
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (own_x - px) ** 2 + (own_y - py) ** 2
            cands.append((d2, arc[i - 1] + t * math.sqrt(seg2)))
        if not cands:
            return None
        if last_s is None:
            return min(cands, key=lambda c: c[0])[1]
        in_win = [c for c in cands if abs(c[1] - last_s) <= self.SEARCH_WINDOW_M]
        if in_win:
            best = min(in_win, key=lambda c: c[0])
            if best[0] <= self.REACQUIRE_DIST_M ** 2:
                return best[1]
        # 窗口内没有、或者窗口内最近的点也太远 -> 全局重新捕获
        return min(cands, key=lambda c: c[0])[1]

    def point_at_arc_length(self, s_query: float):
        """取折线上弧长坐标为`s_query`的那个点，返回(x, y, z)。

        沿折线插值，所以取出来的点一定落在长机走过的路径上——这是"不抄
        近道"的全部保证，不需要任何额外的控制逻辑。
        """
        if len(self._xs) < 2:
            return None
        arc = self._arc_table()
        s_clamped = max(0.0, min(float(s_query), arc[-1]))
        x, y, z, _yaw, _speed = self._point_at_arc(arc, s_clamped)
        return (x, y, z)

def _de_boor(knots, ctrl, degree, u):
    """标准 de Boor 求值。ctrl 是 [(x,y,z), ...]，返回 (x, y, z)。

    ego_planner 的 UniformBspline.evaluateDeBoorT(t) 等价于
    evaluateDeBoor(t + u_[p])，所以调用方传进来的 u 要先加过 knots[degree]。
    """
    n = len(ctrl) - 1
    u = min(max(u, knots[degree]), knots[n + 1])
    k = bisect.bisect_right(knots, u) - 1
    k = min(max(k, degree), n)
    d = [list(ctrl[j + k - degree]) for j in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = j + k - degree
            den = knots[i + degree + 1 - r] - knots[i]
            a = 0.0 if den <= 1e-12 else (u - knots[i]) / den
            for c in range(3):
                d[j][c] = (1.0 - a) * d[j - 1][c] + a * d[j][c]
    return d[degree]


class LeaderPlannedTraj:
    """长机广播的规划轨迹（traj_utils/Bspline），用来取它**未来**的速度。

    2026-09-29 新增。在这之前，僚机的速度前馈来自"长机已经飞过的轨迹长度
    每拍增长了多少"——那是长机的**过去**速度，天然滞后一个缓冲区。长机减速
    拐弯、绕障碍，僚机都要等它飞过去了才知道。本会话一路在补的前馈/偏置积分/
    长机照顾模式，补的都是这一个滞后。
    而长机的 ego_planner 本来就在发 `planning/bspline`（它未来几秒要飞的
    B 样条，含速度剖面），一直没人用。

    只取**速度大小**，不取位置——这一点很关键：bspline 发在长机自己的 odom 系，
    跟僚机工作的 UWB 共享系差一个平移（各机 odom 原点锁在自己起降点）。
    速度大小对平移和旋转都不变，所以**不需要任何坐标变换**，方向仍然用轨迹
    切线（那个本来就在共享系里算好的）。这是这条路子能小改动落地的原因。

    实测（scripts/长机轨迹前馈验证.py，877 条轨迹/11000+ 样本）：
      提前 0.5 s，样条预测误差中位 0.18 m，而"匀速外推"基线是 0.62 m；
      提前 1.0 s，0.63 m vs 1.24 m。所以这条信息是真有增量的。
    """

    def __init__(self):
        self._start = None      # 轨迹起点时刻（秒）
        self._deg = 0
        self._knots = []
        self._vctrl = []        # 速度样条的控制点
        self._stamp = 0.0       # 收到这条轨迹的本地时刻（判过期用）

    def update(self, msg, now_s: float) -> bool:
        pos = [(q.x, q.y, q.z) for q in msg.pos_pts]
        knots = list(msg.knots)
        p = int(msg.order)
        if p < 1 or len(pos) <= p or len(knots) < len(pos) + p + 1:
            return False
        # 位置样条求导 -> 速度样条：控制点 V_i = p*(P_{i+1}-P_i)/(u_{i+p+1}-u_{i+1})，
        # 次数降一阶，节点向量掐掉首尾各一个。
        v = []
        for i in range(len(pos) - 1):
            den = knots[i + p + 1] - knots[i + 1]
            if den <= 1e-9:
                v.append((0.0, 0.0, 0.0))
            else:
                v.append(tuple(p * (pos[i + 1][c] - pos[i][c]) / den for c in range(3)))
        self._start = msg.start_time.sec + msg.start_time.nanosec * 1e-9
        self._deg = p - 1
        self._knots = knots[1:-1]
        self._vctrl = v
        self._stamp = now_s
        return True

    def speed_at(self, t_abs: float, now_s: float, stale_s: float):
        """绝对时刻 t_abs 的规划速度大小（m/s）；没有可用轨迹返回 None。"""
        if self._start is None or (now_s - self._stamp) > stale_s:
            return None
        if self._deg < 1 or len(self._vctrl) <= self._deg:
            return None
        try:
            vx, vy, _vz = _de_boor(self._knots, self._vctrl, self._deg,
                                   (t_abs - self._start) + self._knots[self._deg])
        except Exception:
            return None
        return math.hypot(vx, vy)


class RouteRuler:
    """把**规划航线**当一把里程尺：只量进度，不决定飞哪儿。

    2026-09-28 加这个类的缘由（用户 run36 复盘"第一个航点附近有错乱"）：
    僚机沿长机**实飞轨迹**跟随，弧长也按实飞轨迹算。长机在拐点处会
    冲过头、原地转向时飘、下一段起手又往旁边让（实测 1# 航点处冲过
    0.5 m 后倒退 2.2 m、在那儿悬停 3 秒，t=36~62 实飞弧长 20.5 m 而
    起终点直线只有 7.7 m）。这个"回钩"既虚增了弧长，又被僚机原样复刻，
    间距先扎到 1.51 m 再窜到 6.96 m——全程所有超过 1 m 的偏差都出在
    这一段。

    治法不是让僚机改飞航线——**那会撞柱子**：僚机在编队模式下
    relay_mode=formation，绕开了自己的 ego_planner，没有独立避障能力，
    "沿长机实飞轨迹飞"正是它唯一的避障保证（长机那条线是 ego_planner
    绕出来的，天然无障碍）。所以分成两件事：

      飞哪条线 -> 还是长机实飞轨迹（不变，避障照旧）
      量多远   -> 换成航线弧长（这把尺）

    航线是干净折线，没有回钩、没有悬停抖动，量出来的进度跟评价指标
    （沿航线的间距）也是同一个基准。
    """

    SEARCH_WINDOW_M = 6.0     # 投影消歧的滑动窗口，同 LeaderPathBuffer
    REACQUIRE_DIST_M = 3.0    # 窗口内最近点也比这远 -> 全局重新捕获

    def __init__(self, pts, back_extend_m: float = 0.0):
        self._xs: List[float] = []
        self._ys: List[float] = []
        pts = [(float(a), float(b)) for a, b in pts]
        if len(pts) >= 2 and back_extend_m > 0.0:
            # 往起点**之前**延一段：僚机的起始站位在长机起飞点后方，
            # 不延的话它投影出来恒等于 0，跟"正好站在起点上"分不开，
            # 那段身位差会变成一笔看不见的欠账（跟长机轨迹那边的
            # _path_prepad_m 同一个道理）。两机的 s 都平移同一个常数，
            # 差值不受影响。
            dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
            n = math.hypot(dx, dy)
            if n > 1e-6:
                pts.insert(0, (pts[0][0] - dx / n * back_extend_m,
                               pts[0][1] - dy / n * back_extend_m))
        for x, y in pts:
            if self._xs and math.hypot(x - self._xs[-1], y - self._ys[-1]) < 1e-6:
                continue
            self._xs.append(x)
            self._ys.append(y)
        self._arc = [0.0]
        for i in range(len(self._xs) - 1):
            self._arc.append(self._arc[-1] + math.hypot(
                self._xs[i + 1] - self._xs[i], self._ys[i + 1] - self._ys[i]))

    def ok(self) -> bool:
        return len(self._xs) >= 2

    def total(self) -> float:
        return self._arc[-1] if self._arc else 0.0

    def project(self, x: float, y: float, last_s: Optional[float] = None) -> Optional[float]:
        """位置投到航线上，返回弧长坐标。消歧同 LeaderPathBuffer：滑动窗口+逃逸。"""
        n = len(self._xs)
        if n < 2:
            return None
        cands = []
        for i in range(1, n):
            x0, y0, x1, y1 = self._xs[i - 1], self._ys[i - 1], self._xs[i], self._ys[i]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
            px, py = x0 + t * dx, y0 + t * dy
            cands.append(((x - px) ** 2 + (y - py) ** 2, self._arc[i - 1] + t * math.sqrt(seg2)))
        if not cands:
            return None
        if last_s is None:
            return min(cands, key=lambda c: c[0])[1]
        in_win = [c for c in cands if abs(c[1] - last_s) <= self.SEARCH_WINDOW_M]
        if in_win:
            best = min(in_win, key=lambda c: c[0])
            if best[0] <= self.REACQUIRE_DIST_M ** 2:
                return best[1]
        return min(cands, key=lambda c: c[0])[1]

    def project_unwrapped(self, x: float, y: float, state: dict) -> Optional[float]:
        """带圈数展开的投影：航线绕一圈回起降点是**闭合**的，原始弧长会在
        终点处突然跳回 0。这里记住上一次的原始值和已经绕过的圈数，跨越
        首尾时加一圈，返回的是单调递增的"累计里程"，两机相减才有意义。
        """
        raw = self.project(x, y, state.get('raw'))
        if raw is None:
            return None
        L = self.total()
        laps = state.get('laps', 0)
        prev = state.get('raw')
        if prev is not None and L > 1e-6:
            if raw - prev < -L * 0.5:
                laps += 1
            elif raw - prev > L * 0.5:
                laps -= 1
        state['raw'] = raw
        state['laps'] = laps
        return raw + laps * L


# ============================================================
# 第2部分：几何队形 / 控制策略——策略模式的两层可替换接口（1.4.1节）
# ============================================================

@dataclass
class Pose2D:
    """一个轻量的2D位姿数据类，只在这个节点内部使用，不是ROS消息类型
    ——FormationGeometry算出来的"目标点"先用这个纯Python类型表达，
    转成具体ROS消息（PoseStamped）是FormationControlStrategy这一层
    才做的事，两层各自的职责边界更清楚（哪个队形该怎么算目标点，跟
    目标点最终该用什么消息类型/话题发出去，是两个独立可以分别替换
    的问题）。"""
    x: float
    y: float
    yaw: float = 0.0


class FormationGeometry(ABC):
    """几何队形接口：给定"我"当前的角色参数，从长机的轨迹缓冲区里
    算出"我现在应该在哪个目标点"。不同队形只是这个方法的不同实现。
    """

    @abstractmethod
    def compute_target(self, leader_path: LeaderPathBuffer, params: Dict) -> Optional[Pose2D]:
        raise NotImplementedError


class TandemGeometry(FormationGeometry):
    """纵向编队：沿长机实际轨迹回溯固定弧长（这次任务实际使用的实现，
    见方案1.4节"维度A：几何队形"里对tandem的选择理由——单路纵队，
    横向间距为0，只有前后距离，规避走廊最窄、实现最简单）。"""

    def compute_target(self, leader_path: LeaderPathBuffer, params: Dict) -> Optional[Pose2D]:
        follow_distance_m = float(params['follow_distance_m'])
        result = leader_path.point_at_arc_length_behind(follow_distance_m)
        if result is None:
            return None
        x, y = result
        return Pose2D(x=x, y=y, yaw=0.0)


class LineAbreastGeometry(FormationGeometry):
    """横向编队：在长机当前朝向的法线方向偏移固定距离。

    预留接口，这次任务不实现——v2初稿最早试过这个方案又撤回了（计算
    "侧向偏移量"要考虑航向实时变化，见DEBUG_JOURNAL.md 2026-09-12
    历史记录），这次任务的编队形式已经确定用tandem（纵向）。保留这个
    类只是为了满足"几何队形要做成可插拔接口"这条要求（用户明确要求
    不要把这次的选择焊死），不代表以后一定会启用它——"接口留着、这次
    任务不用"和"接口都没有、以后要用时重新设计"是两回事，保留成本很
    低，不该因为这次不用就不留。
    """

    def compute_target(self, leader_path: LeaderPathBuffer, params: Dict) -> Optional[Pose2D]:
        raise NotImplementedError(
            'LineAbreastGeometry(横向编队)是预留接口，这次2026大赛任务'
            '不实现——方案1.4节维度A的讨论已经解释过原因（v2初稿试过又'
            '撤回），需要横向编队时再补具体实现。'
        )


class FormationControlStrategy(ABC):
    """控制策略接口：拿到FormationGeometry算出来的目标点之后，决定
    怎么把这个目标发出去（比如要不要做平滑/预测，用什么频率、什么
    接口发布）。"""

    @abstractmethod
    def track(self, target: Pose2D, params: Dict) -> None:
        raise NotImplementedError


class LeaderFollowerStrategy(FormationControlStrategy):
    """长机-僚机：直接把目标点当成一次性目标发给`rviz_goal_world`
    （`geometry_msgs/PoseStamped`），复用ego_planner_bridge现成的目标
    点执行链路（`rviz_goal_bridge_node.py`第126行订阅这个话题），这次
    任务实际使用的实现。

    params需要包含：
      - 'publisher'：发布`rviz_goal_world`的Publisher对象（由
        FormationFollowerNode持有并常驻，不是每次track都新建）。
      - 'frame_id'：这个PoseStamped的header.frame_id，取跟随者自己的
        `<namespace>/odom`（跟rviz_goal_bridge_node自己的target_frame
        完全一致），这样rviz_goal_bridge_node内部查`lookup_transform
        (target_frame, msg.header.frame_id, ...)`时source==target，
        tf2直接返回单位变换——这一步刻意不依赖任何真实TF查询，因为
        uwb_imu模式下长机/跟随者的坐标本来就是同一个全局系，不需要
        任何变换，用"同frame_id"这个技巧让下游现成节点的变换逻辑自然
        退化成"不变换"，不需要碰rviz_goal_bridge_node一行代码。
      - 'z'：目标点的高度，见FormationFollowerNode里的说明（跟随者
        直接采用长机当前实时高度，不参与弧长回溯——两机纵向编队时
        通常保持同一个巡航AGL高度飞行，高度本身不需要"追溯长机走过的
        高度轨迹"这种处理）。
      - 'stamp'：header.stamp，用节点自己的`get_clock().now()`。
    """

    def track(self, target: Pose2D, params: Dict) -> None:
        publisher: Publisher = params['publisher']
        msg = PoseStamped()
        msg.header.frame_id = params['frame_id']
        msg.header.stamp = params['stamp']
        msg.pose.position.x = target.x
        msg.pose.position.y = target.y
        msg.pose.position.z = float(params.get('z', 0.0))
        msg.pose.orientation.w = 1.0  # yaw=0的单位四元数；这次任务不需要跟随者
        # 严格对齐长机朝向，term_goal链路本身也只看position，见
        # rviz_goal_bridge_node.py文件头对Fixed Frame/坐标变换的说明。
        publisher.publish(msg)


class VirtualStructureStrategy(FormationControlStrategy):
    """虚拟结构法：把整个编队当成一个刚体，所有飞机相对一个虚拟中心点
    保持固定偏移，中心点轨迹单独规划。

    预留接口，不在这次任务里实现——双机场景下用长机-僚机就够，不需要
    上这一档复杂度（见方案1.4节维度B的讨论）。以后如果编队规模变大/
    需要更刚性的队形，在这里加具体实现即可，不用碰FormationGeometry
    那一层。
    """

    def track(self, target: Pose2D, params: Dict) -> None:
        raise NotImplementedError(
            'VirtualStructureStrategy(虚拟结构法)是预留接口，这次2026'
            '大赛任务不实现——双机场景用leader_follower就够，见方案'
            '1.4节维度B的讨论。'
        )


# 队形/策略的名字 -> 具体实现类，节点按参数字符串从这里查表选择，新增
# 队形/策略只需要在这里添加一行，不用改节点主循环逻辑。
_GEOMETRY_REGISTRY = {
    'tandem': TandemGeometry,
    'line_abreast': LineAbreastGeometry,
}
_STRATEGY_REGISTRY = {
    'leader_follower': LeaderFollowerStrategy,
    'virtual_structure': VirtualStructureStrategy,
}


# ============================================================
# 第3部分：节点本体
# ============================================================

class FormationFollowerNode(Node):
    """常驻的编队跟随控制回路。每架飞机的flight-stack容器都会起这个
    节点（角色运行时决定，不是只给SUPPLY角色的飞机装这个能力，见方案
    1.5节双机对称性原则），默认`enabled=False`空转，收到`~/set_
    parameters`把`enabled`设为`True`+指定`leader_namespace`之后才真正
    开始工作。
    """

    def __init__(self):
        super().__init__('formation_follower_node')

        self.declare_parameter('geometry_type', 'tandem')
        self.declare_parameter('strategy_type', 'leader_follower')
        # 3.5米是D3节里SDK调用`sdk.start_formation_follow(follow_
        # distance_m=3.5)`用的值，这里同步一个一致的默认值，避免"没
        # 显式传这个参数就跟随距离为0/未定义"这种容易被忽略的边界情况；
        # SDK仍然会显式传这个值，这个默认值主要是防呆。
        self.declare_parameter('follow_distance_m', 3.5)
        # leader_namespace默认空字符串——刻意不给一个"看起来合理"的
        # 默认命名空间（比如不能默认填'NX01'），那样等于变相硬编码了
        # 角色分配，违反1.5节"角色运行时决定，不能焊死在机身编号上"的
        # 原则。空字符串在_on_set_parameters里会被当成"未配置"，跟
        # enabled=True一起出现时直接拒绝这次参数变更。
        self.declare_parameter('leader_namespace', '')
        self.declare_parameter('enabled', False)
        # 5-10Hz区间取的默认值，见方案1.4.2节。这个频率只决定"多久
        # 重新算一次目标点、发一次rviz_goal_world"，不是飞控内部控制
        # 环的频率（那个由px4ctrl/pt4ctrl自己的内部循环决定，跟这个
        # 节点无关）。
        self.declare_parameter('rate_hz', 8.0)

        # ---- 2026-09-20 新增：入列(join)阶段的判据 ----
        # 原来 enabled=True 后立刻进入稳态跟随，没有"入列"这个阶段：启用
        # 那一刻若两机间距还很远，第一个目标点就是个大机动，且没有任何
        # 收敛判据能告诉调用方"队形组好了没有"。现在拆成 join/track/break
        # 三段，join 达标才算入列完成（见《设计表》第十章 10.1）。
        self.declare_parameter('join_tolerance_m', 1.2)      # 实际位置与应在位置的距离容差
        self.declare_parameter('join_hold_s', 1.0)           # 容差内持续这么久才算入列
        self.declare_parameter('join_timeout_s', 45.0)
        # 2026-09-21：`hold`阶段（已就位、等长机起步）的超时。必须比
        # `join_timeout_s`宽得多——长机那边要等僚机报"已进入跟随待命"才
        # 起步，这段时间僚机就停在自己起飞点上空保持，属于正常等待，不是
        # 故障。入列超时只从长机真正走起来之后才开始计。
        self.declare_parameter('leader_start_timeout_s', 180.0)       # goal.join_timeout_s<=0 时用这个
        self.declare_parameter('leader_lost_timeout_s', 3.0) # 多久收不到长机odom算丢失

        # ---- 2026-09-20：改成真正的轨迹跟随 ----
        # 原来把目标点发给 rviz_goal_world -> term_goal -> ego_planner，
        # 由规划器自己重新规划一条到该点的路径——规划器不知道也不关心长机
        # 是怎么飞过去的，会抄近道切内弯；再加上目标点 8Hz 不停变化，规划器
        # 被反复触发重规划，实测轨迹很乱。
        #
        # 轨迹跟随要的是"设定点逐点复现长机走过的路径"，所以改走中继层已有
        # 的"完全接管"模式（跟 precision_land/pillar_aim 同一套）：本节点直接
        # 发 PositionCommand 到 formation_cmd，position_cmd_relay 转给 pt4ctrl，
        # 完全绕开 ego_planner。
        #
        # 代价：跟随者不再有规划器避障。但这恰恰是轨迹跟随的前提——它走的是
        # 长机刚飞过的同一条路径，本身就是无碰撞的。
        self.declare_parameter('cmd_rate_hz', 30.0)
        # 设定点相对飞机当前位置的**步长上限**（不是控制增益）。PX4 的
        # position_cmd 语义是"你现在就该在这个点"，一次给一个好几米外的点
        # 会被当成远距离阶跃、直奔过去大幅倾斜加速（2026-09-20 实测摔过
        # 一架）。这一条只是把单步位移限住，防止那种阶跃，飞多快由 PX4
        # 自己的位置环和速度限幅决定，本节点不参与。
        self.declare_parameter('max_lead_m', 1.5)
        # 推进点最多领先僚机实际投影位置多少弧长。这一条防止"参考点自己
        # 匀速跑了、飞机跟不上"——跟不上时参考点等它，不会越拉越远最后
        # 变成一条脱离轨迹的直线。
        self.declare_parameter('lookahead_m', 1.2)
        # 僚机自己的巡航速度（米/秒）。2026-09-21（用户："都是各自飞各自
        # 的，匀速，只是间距大于阈值，僚机沿长机轨迹飞即可"）：参考点沿
        # 长机轨迹**匀速**前进，不再靠位置误差出速度（那样越接近越慢，是
        # 变速爬行，不是匀速）。默认值跟长机 ego_planner 的 max_vel 一致
        # （V_MAX 环境变量，docker-compose.yml 里默认 1.0），这样两机是
        # 同一个巡航速度、各飞各的。
        self.declare_parameter('cruise_speed_mps', 1.0)
        # ---- 2026-09-28 新增：追赶项 ----
        # 原来参考点是**恒定**速度前进，跟长机同速。两个同速的点，间距恒定不变——
        # 也就是说起步阶段（长机已走、僚机还没入列）拉开的那段距离，在设计上
        # 永远收不回来。落后量 `_last_lag_m` 一直算着，但从来没有回到速度上。
        # 现在：落后超过 catchup_deadband_m 就按比例提速，回到位自动退回匀速。
        #   cruise_eff = cruise * clamp(1 + gain*(lag - deadband), 1.0, ratio_max)
        # gain 默认 0.0 = 行为跟改造前一字不变。ratio_max 限制最多比长机快多少，
        # 别设太大：僚机的设定点还受 max_lead_m(1.5) 的单步限幅和 PX4 位置环
        # 自身速度限幅约束，参考点跑太快只会顶到 lookahead_m 那条线空转。
        # 参考点变化率相对巡航速度的余量。1.0 = 参考点最快只能跟长机同速（旧行为，
        # 落后了就追不回来）；>1 时 s_limit_gap 才是真正的约束，弧长间距被钉死。
        # 别设太大：参考点跑太快只会顶到 lookahead_m 那条线空转，而且设定点
        # 单步还受 max_lead_m(1.5) 限幅、实际速度由 PX4 位置环决定。
        self.declare_parameter('reference_rate_headroom', 1.5)
        # 参考点推进速率的**变化率**上限（m/s²）。只限速度不限加速度的话，
        # 参考点可以一拍之内从 1.0 跳到 1.5，僚机跟着猛加速，观感上速度变化
        # 很剧烈。限住之后僚机的加减速是平滑的。0 = 不限（旧行为）。
        self.declare_parameter('reference_accel_limit', 0.3)
        # ---- 拐点前馈 ----
        # 长机每到一个航点都"先悬停转向再前飞"，这是个**已知的、可预告的**
        # 减速事件：航线僚机手里就有，长机现在在哪它也知道。与其等长机停了、
        # 落后量涨起来再从反馈里发现，不如提前收油。
        # 做法：长机离任一航点小于 corner_ff_radius_m 时，按距离线性压低参考点
        # 推进速率（最低压到 corner_ff_min_ratio）。半径设 0 即关闭。
        self.declare_parameter('corner_ff_radius_m', 2.0)
        self.declare_parameter('corner_ff_min_ratio', 0.4)
        # ---- 2026-09-28 新增：速度前馈用"达成速率"而不是"限幅值" ----
        # reference_rate_headroom 引入之后，cruise 只是参考点推进速率的**上限**
        # （1.5），实际推进量被 s_limit_gap 钳成长机的速度（1.0）。之前把 cruise
        # 直接当速度前馈发出去，等于告诉 pt4ctrl "你该按 1.5 m/s 飞"，多喂了
        # 50%，僚机持续往前顶，实测出现负落后（-0.49，僚机反超参考点）。
        # 现在：前馈取 (Δs_cmd/dt) 的低通值。长机轨迹是按里程计一帧帧长出来的，
        # s_limit_gap 呈台阶式增长，瞬时速率抖得厉害，所以要滤。
        self.declare_parameter('ff_rate_tau_s', 0.5)
        # ---- 长机规划轨迹前馈（2026-09-29，方法 A）----
        # 订长机的 planning/bspline，拿它**未来**的速度当前馈，替掉原来那个
        # 由"长机已飞过的轨迹长度"反推的过去速度。详见 LeaderPlannedTraj。
        # 收不到/过期就自动退回原来的算法，不会把飞行搞停。
        self.declare_parameter('leader_traj_ff_enabled', True)
        self.declare_parameter('leader_traj_ff_lead_s', 0.5)   # 取未来多久的速度
        self.declare_parameter('leader_traj_ff_stale_s', 1.0)  # 轨迹多久没更新就算过期
        # ---- 2026-09-28：航线里程尺只用来**清洗长机轨迹**，不当间距基准 ----
        # 试过让间距也按航线里程算（route_basis），实测更差：run37 间距
        # -2.55~14.49，对照 run36 的 1.51~6.96。机理是僚机**飞**的是长机实飞
        # 轨迹、却按**航线**里程约束间距，两把尺长度不等——绕柱子那段实飞轨迹
        # 比航线长，僚机得超速才能维持航线间距，超不动就掉队，绕完又过冲。
        # 基准不统一，比统一到任何一边都差。那套双尺切换（二分查找、只读投影）
        # 已整体删除，别再加回来。
        # 长机轨迹清洗：只有让**航线进度前进**的点才进缓冲区。拐点冲过头再
        # 倒回来、原地转向时的飘移，航线弧长不增反减，直接丢掉，僚机就不会
        # 跟着复刻。绕柱子那种绕行**是前进的**（偏离航线但里程一直在涨），
        # 照留不误——这是这条判据跟"按偏离航线远近过滤"的本质区别。
        # 护栏：丢点造成的跨度超过这个值就不丢了，老老实实照抄——万一长机是
        # "倒车绕障"，抄近道那条弦可能正好穿过障碍物。
        self.declare_parameter('path_prune_max_chord_m', 2.5)
        # ---- 2026-09-28 新增：间距偏置补偿（慢积分）----
        # 实测落后量中位 +0.39 米——这是个**稳定偏置**（飞机始终落在参考点后面，
        # 位置环的稳态误差 + 单步限幅），不是噪声。既然稳定就能直接补掉：
        # 把有效跟随距离慢慢往下修，让**实际**间距收敛到 follow_distance_m，
        # 而不是让参考点收敛到它。
        # 时间常数远慢于僚机位置环和长机照顾模式（~1 s），不会跟它们打架。
        # gain 设 0 = 关闭，退回改造前行为。
        self.declare_parameter('bias_trim_gain', 0.15)     # 1/s，trim += gain*lag*dt
        # 收敛后 trim 大致等于参考点到飞机那段不可消除的落后（lookahead +
        # 飞机自身跟踪误差，实测 0.6~1.2），上限要留余量，顶到栏杆就不再调节了。
        self.declare_parameter('bias_trim_max_m', 1.5)     # 最多把跟随距离修小这么多
        self.declare_parameter('bias_trim_min_m', -0.5)    # 最多修大这么多（僚机反超时）
        self.declare_parameter('catchup_gain', 0.0)
        self.declare_parameter('catchup_ratio_max', 1.3)
        self.declare_parameter('catchup_deadband_m', 0.5)
        self.declare_parameter('max_z_lead_m', 0.4)

        # ---- 定高（2026-09-20订正，用户要求）----
        # 原来"锁定入列瞬间两机高度差、之后跟随长机高度变化"有两个问题：
        # ① 僚机先起飞、长机后起飞，锁定那一刻长机还在地面，基准固化了
        #    0.8 米误差（实测僚机全程比长机高 0.8 米）；
        # ② 跟随的是长机的**世界系**高度变化——长机飞过仿地模块
        #    (terrain_module，航线第一段正好从它上方过) 会抬升，僚机在平地
        #    上也跟着抬升，这不对。
        # 改成按**固定 AGL 定高**：目标高度来自机载定高雷达，僚机在自己脚下
        # 的地形上保持设定离地高度。这既是任务真正需要的量，也跟真机完全
        # 一致（同一颗雷达、同一条话题），不依赖任何仿真专有信息。
        self.declare_parameter('follow_altitude_agl_m', 1.5)
        self.declare_parameter('range_topic', 'mavros/hrlv_ez4_pub')

        # ---- 分段航向（2026-09-24 用户要求，长机僚机同一套动作）----
        # 用户原话："从起飞点开始，将 yaw 角转动一次方向，大小为当前航点指向下一个
        # 航点的那个航向角，飞行途中锁定这个角，而后到下一个航点时以此类推。航向
        # 转动时悬停，航向角到位后再前飞。长机、从机都一样的要求。"
        # 僚机没有"航点"这个概念——它沿长机走过的折线飞，所以这里的等价物是
        # **轨迹在参考点处的切线方向**：长机现在每段都锁死航向直飞，切线在一段之内
        # 就是常值，拐角处才会跳变，跳变量超过 yaw_leg_change_deg 就认为"到了下一个
        # 航点"，于是：锁新航向 -> 原地悬停（参考点不再前进、速度给 0）-> 转到
        # yaw_leg_tol_deg 以内 -> 继续前飞。
        # 默认关（False）= 保持 2026-09-20 起的行为（入列时锁定自身朝向、全程不变），
        # 由选手程序按需打开，不改默认行为。
        self.declare_parameter('yaw_follow_leg', False)
        self.declare_parameter('yaw_leg_tol_deg', 5.0)
        # ---- 协调转弯（2026-09-28 用户要求：拐点处不停留）----
        # yaw_follow_leg 打开后原本是"到了拐点就地停下、把机头转到下一段航向、
        # 转到位再走"。停下来这件事本身是编队的大扰动：僚机停 3 秒，长机没停，
        # 间距就涨 3 米，之后还得追回来。
        # turn_in_place=False 改成：机头按限速**连续**转向当前段航向，参考点
        # 一刻不停。长机侧的对应改动在 formation.py 的 _start_coordinated_yaw。
        self.declare_parameter('yaw_turn_in_place', True)
        self.declare_parameter('yaw_slew_rate_dps', 60.0)
        # 航线（世界坐标，[x0,y0,x1,y1,...]，第一个点是长机起飞点）。航向**只能**
        # 来自这条航线：每段一个精确值，全程只在起飞点和各航点变一次，中途恒定。
        # 2026-09-24 先试过"从长机走过的轨迹算切线"，无论短基线还是长基线弦向都
        # 不行——长机轨迹是里程计采样点连成的折线，本身带噪声，直线段上算出来的
        # 方向就在 ±12° 之间跳，僚机一路走走停停摇头（用户实测指出"从机中途航向
        # 角一直在变化"）。航线是已知量，没有任何理由去估计它。
        self.declare_parameter('leg_route_xy', [0.0])
        # 距拐点还有这么远就算走完本段、换下一段的航向。不能用"投影 >= 1.0"这种
        # 严格判据：长机的到点阈值是 0.3 米，它在离航点 0.2~0.3 米处就判到点、直接
        # 转弯走下一段了，参考点沿它走过的轨迹永远到不了那个角点，投影卡在 0.99
        # ——2026-09-24 实测就是这么卡住的，僚机转完前两段之后一路保持 90°，第 3、
        # 4 段该朝西朝南时机头还指着北（用户指出"任务机航向又搞错了"）。
        self.declare_parameter('leg_corner_slack_m', 0.8)

        # ---- 跨机对齐（2026-09-21订正）----
        # 原来用 TF（<长机ns>/odom -> 自己的 odom）做跨机变换，实测不可靠：
        # 每架飞机的 odom 原点是开机时用 UWB 测量值各自锁定的，两次锁定误差
        # 直接叠进编队位置——同一套代码两次运行，TF 平移一次是 3.04m、一次
        # 是 4.19m，而两个起降点实际只差 3.00m，误差 1.19m。
        # 改用 UWB 绝对系：两机的 uwb/pose_abs 本来就在同一个锚点坐标系下
        # （uwb_imu_fusion_node 一直在消费这条话题），不存在各自锁定的问题。
        # 设定点发布前再用"自己 odom 减自己 UWB"这个实时偏移换回 odom 系，
        # 原点锁定误差自动抵消。
        # ⚠️ 只能用 uwb/pose_abs（带噪声/偏置的传感器模拟，真机上是真 UWB
        # 驱动发的同名话题），**不能用 uwb/pose_truth**——那是无噪声纯真值，
        # 文件头写明"评估专用旁路，机上模块不许订阅"。
        self.declare_parameter('uwb_topic', 'uwb/pose_abs')
        self.declare_parameter('auto_switch_relay_mode', True)
        self.declare_parameter('relay_set_parameters_service', 'position_cmd_relay/set_parameters')

        # 长机轨迹缓冲区+当前订阅状态。_current_leader_ns记录"现在
        # 实际订阅的是哪个命名空间"，跟leader_namespace参数值分开存
        # 一份，是因为参数变更的生效时机是在_on_set_parameters回调
        # 里、订阅还没切换之前就需要读到"新值"，不能直接用
        # self.get_parameter(...)（那个要等回调返回、rclpy真正写入
        # 参数存储之后才会更新）。
        self._leader_path = LeaderPathBuffer()
        # 长机轨迹**向后延伸**的那一段的长度，见 _on_leader_pose 里的说明。
        # 0 表示没延伸（拿不到航线时退回旧行为）。
        self._path_prepad_m = 0.0
        self._leader_latest_z: float = 0.0
        self._current_leader_ns: str = ''
        self._leader_sub = None

        # 跟rviz_goal_bridge_node.py的target_frame约定完全一致
        # （ns + '/odom'），见LeaderFollowerStrategy.track()的说明。
        own_ns = self.get_namespace().strip('/')
        self._own_odom_frame = f'{own_ns}/odom' if own_ns else 'odom'

        self.goal_pub = self.create_publisher(PoseStamped, 'rviz_goal_world', 10)
        # 轨迹跟随的真正输出口（见 cmd_rate_hz 参数上面的说明）
        self.cmd_pub = self.create_publisher(PositionCommand, 'formation_cmd', 10)
        # 诊断话题（2026-09-28）：把跟随回路内部真正看到的几个量发出来，供
        # scripts/monitor.py 和长机的"照顾模式"用。外面自己拿两机位置去算
        # 弧长是算不准的——折线基准、采样率、向后延伸那一段都跟这里不一样。
        #   data[0] = lag_m        落后量 = s_limit_gap - s_proj（>0 表示落后）
        #   data[1] = s_cmd        参考点弧长
        #   data[2] = s_proj       僚机投影弧长
        #   data[3] = total_length 长机轨迹总弧长（含向后延伸段）
        #   data[4] = 弧长间距 = total_length - s_proj
        self.diag_pub = self.create_publisher(Float64MultiArray, 'formation_diag', 10)

        # 2026-09-20：入列判据需要知道"我自己在哪"，原来这个节点只订阅
        # 长机odom、不看自己的位置（稳态跟随不需要）。
        self._own_xy: Optional[Tuple[float, float]] = None
        self._own_z: Optional[float] = None
        self._last_lag_m: Optional[float] = None
        self._last_s_proj: Optional[float] = None   # 上一次的投影弧长，消歧义用
        self._s_rate: Optional[float] = None        # 参考点当前推进速率，限加速度用
        self._s_rate_ff: Optional[float] = None     # 参考点**达成**推进速率（低通），速度前馈用
        self._leader_traj = LeaderPlannedTraj()     # 长机规划轨迹（方法 A 前馈）
        self._leader_traj_sub = None
        self._leader_traj_ff_used = 0               # 用上/退回的计数，只为日志
        self._fd_trim: float = 0.0                  # 跟随距离的偏置补偿量（慢积分）
        # ---- 航线里程尺（2026-09-28）----
        self._route_ruler: Optional[RouteRuler] = None
        self._route_key: Optional[tuple] = None     # 建尺时用的 leg_route_xy，变了就重建
        self._rs_lead: dict = {}                    # 长机、僚机、参考点各自的展开状态
        self._rs_own: dict = {}
        self._lead_route_s_max: Optional[float] = None   # 长机航线进度的历史最大值
        self._s_route_lead: Optional[float] = None       # 长机当前的航线里程（长机回调里算）
        self._leader_raw_xy: Optional[Tuple[float, float]] = None  # 长机最新实测位置（未经清洗）
        # 高度基准（2026-09-20炸机+自降两次事故后加）：两机的 odom z **不在
        # 同一个基准上**——`world->{ns}/odom` 的 z 平移量是每架飞机开机时
        # 各自运行时锁定的（uwb_imu 定位源的已知性质），实测同在地面时
        # NX01 读 0.91、NX02 读 0.27，差 0.64 米。所以绝不能把长机的 z
        # 直接当僚机的目标高度。
        # 而且轨迹缓冲区从节点启动就开始记，**包含长机还在地面的那一段**，
        # 僚机刚入列时投影落在轨迹起点附近，取到的就是长机停在地面时的
        # 高度——这正是"起飞一会后自己慢慢降落"的直接原因。
        # 改成相对量跟随：目标高度 = 僚机入列时自身高度 + 长机相对它入列
        # 时的高度变化。用僚机自己的基准，跨机偏差不存在；跟的是长机的
        # 爬升/下降，这才是"高度相同"的实际含义。
        self._own_yaw: Optional[float] = None
        # 2026-09-20（用户明确要求"不考虑偏航角"）：编队跟随只管位置，不管
        # 朝向。原来把 cmd.yaw 设成航迹切线方向，拐角处会出现90度的偏航
        # 甩动，既无必要也增加失稳风险。改成入列时锁定自身当前朝向，全程
        # 保持不变。
        self._yaw_ref: Optional[float] = None
        self._yaw_turning: bool = False      # 分段航向：正在原地转向（此时不前进）
        self._leg_idx: int = 0               # 参考点当前走在第几段（只前进不后退）
        self._leader_last_rx: float = 0.0
        self._last_target_xy: Optional[Tuple[float, float]] = None
        self._action_busy = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            Odometry, 'dlio/odom_node/odom', self._on_own_odom, 10, callback_group=self._cb_group
        )
        self._agl: Optional[float] = None
        # 参考点沿长机轨迹的弧长进度（匀速推进），入列时从飞机当前投影起步
        self._s_cmd: Optional[float] = None
        self._s_cmd_t: Optional[float] = None
        # QoS 必须是 BEST_EFFORT：这条话题的发布端按"高频传感器数据"发
        # （mavros 侧是 SensorDataQoS），用默认的 RELIABLE 订阅会直接
        # QoS 不兼容、一条都收不到（实测启动即报 "offering incompatible
        # QoS ... Last incompatible policy: RELIABILITY"，_agl 永远是
        # None，定高退化成"保持当前高度"）。写法跟 uwb_imu_fusion_node
        # 订同一条话题时保持一致。
        self.create_subscription(
            Range, str(self.get_parameter('range_topic').value), self._on_range,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self._cb_group,
        )
        self.relay_set_params_cli = self.create_client(
            SetParameters,
            str(self.get_parameter('relay_set_parameters_service').value),
            callback_group=self._cb_group,
        )
        self._own_uwb_xy: Optional[Tuple[float, float]] = None
        self.create_subscription(
            PoseStamped, str(self.get_parameter('uwb_topic').value),
            self._on_own_uwb, 10, callback_group=self._cb_group,
        )
        # 2026-09-20（第三次事故后加）：两机的 odom **不是同一个坐标系**。
        # 文件头原来那段"uwb_imu 模式下两机 odom 天然对齐、不需要任何转换"
        # 是错的——实测每架飞机的 odom 原点锚在自己的起降点上：NX01 的
        # world->odom 平移是 (+1.43,-10.08)、NX02 是 (-1.48,-10.01)，x 差
        # 整整 3 米（z 也差 0.64 米）。直接拿长机 odom 当僚机坐标用，等于
        # 跟着一条整体平移过的轨迹飞，看起来就是"抄近道 / 跟不上"。
        # 正确做法：长机轨迹点先经 TF 变换到僚机自己的 odom 系再做弧长计算。
        self._action_server = ActionServer(
            self, FormationFollow, 'formation_follow',
            execute_callback=self._execute_formation,
            goal_callback=self._on_action_goal,
            cancel_callback=self._on_action_cancel,
            callback_group=self._cb_group,
        )

        # 策略模式：每种队形/策略各自只实例化一份，节点主循环只认
        # FormationGeometry/FormationControlStrategy这两个抽象接口，
        # 不关心具体是哪个实现（见文件头接口设计说明）。
        self._geometries = {name: cls() for name, cls in _GEOMETRY_REGISTRY.items()}
        self._strategies = {name: cls() for name, cls in _STRATEGY_REGISTRY.items()}

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # 轨迹跟随要以较高频率喂设定点给 pt4ctrl（原来 8Hz 是"发目标点给
        # 规划器"的节奏，直接喂控制器不够用），所以用 cmd_rate_hz。
        rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.create_timer(1.0 / rate_hz, self._on_control_tick)

        self.get_logger().info(
            'formation_follower_node就绪，默认enabled=False（待命，不发布任何目标）。'
            "启用方式：ros2 param set <ns>/formation_follower_node leader_namespace <长机ns> "
            "&& ros2 param set <ns>/formation_follower_node enabled true"
            "（或一次~/set_parameters调用同时带上这两个参数）。"
        )

    # -------------------- 参数变更：启用/切换长机 --------------------

    def _on_set_parameters(self, params: List) -> SetParametersResult:
        """标准`~/set_parameters`回调——跟这个包里`position_cmd_relay`/
        `fire_pillar_aim_node`同一种风格（不新造自定义service），SDK
        的`sdk.start_formation_follow()`最终调的就是这个。

        校验+副作用（切换长机订阅）都在这个回调内部完成，参照
        `fire_pillar_aim_node.py`"锁定成功那一刻自己去调其它service"
        的写法——这里没有跨节点service调用、不存在那个文件头提到的
        死锁风险，不需要MutuallyExclusiveCallbackGroup。

        ⚠️ 校验/切换订阅时用的是这次调用"生效之后的最终取值"（当前
        参数值+这次请求要改的值合并），不是逐个参数孤立校验——因为
        SDK一次`set_parameters`调用可能同时带上`leader_namespace`+
        `enabled`两个参数，如果只看"这次改了哪个"、不考虑合并结果，
        会在"enabled=True和leader_namespace=X在同一次调用里一起设"
        这种最常见的用法上误判。
        """
        incoming = {p.name: p.value for p in params}

        def effective(name, current_getter):
            return incoming[name] if name in incoming else current_getter()

        eff_enabled = effective('enabled', lambda: bool(self.get_parameter('enabled').value))
        eff_leader_ns = effective('leader_namespace', lambda: str(self.get_parameter('leader_namespace').value)).strip('/')
        eff_geometry = effective('geometry_type', lambda: str(self.get_parameter('geometry_type').value))
        eff_strategy = effective('strategy_type', lambda: str(self.get_parameter('strategy_type').value))
        eff_follow_dist = effective('follow_distance_m', lambda: float(self.get_parameter('follow_distance_m').value))

        if bool(eff_enabled) and not eff_leader_ns:
            return SetParametersResult(
                successful=False,
                reason=(
                    'enabled=True时leader_namespace不能是空字符串——必须先/同时指定'
                    '"跟随谁"，不允许启用一个不知道该订阅哪个长机odom的跟随回路。'
                ),
            )
        if eff_geometry not in _GEOMETRY_REGISTRY:
            return SetParametersResult(
                successful=False,
                reason=f'geometry_type必须是{tuple(_GEOMETRY_REGISTRY)}之一，收到{eff_geometry!r}',
            )
        if eff_strategy not in _STRATEGY_REGISTRY:
            return SetParametersResult(
                successful=False,
                reason=f'strategy_type必须是{tuple(_STRATEGY_REGISTRY)}之一，收到{eff_strategy!r}',
            )
        if float(eff_follow_dist) <= 0.0:
            return SetParametersResult(
                successful=False,
                reason=f'follow_distance_m必须是正数，收到{eff_follow_dist}',
            )

        # 校验通过，允许这次参数变更落地——在返回successful=True之前
        # 用"生效之后的值"同步一次订阅状态，保证订阅切换跟参数变更是
        # 同一次调用原子完成的，不会有"参数已经改了、订阅还没跟上"的
        # 中间状态被其它回调/定时器看到。
        desired_ns = eff_leader_ns if bool(eff_enabled) else ''
        self._sync_leader_subscription(desired_ns)

        if 'enabled' in incoming:
            self.get_logger().info(f"enabled -> {eff_enabled}")
        if 'leader_namespace' in incoming:
            self.get_logger().info(f"leader_namespace -> '{eff_leader_ns}'")

        return SetParametersResult(successful=True)

    def _sync_leader_subscription(self, desired_ns: str) -> None:
        """按"这次应该跟踪的长机命名空间"（空字符串表示不跟踪任何人）
        动态创建/切换跨命名空间订阅。

        对应清单A3第3条要求："leader_namespace运行时可变，这个订阅
        需要能在收到启用指令时动态创建/切换，不能在__init__时就固定
        订阅哪个namespace"——所以`__init__`里完全没有创建这个订阅，
        只有走到这个方法才会创建，且只在desired_ns真正变化时才动
        （避免同一个长机反复销毁/重建订阅）。
        """
        if desired_ns == self._current_leader_ns:
            return

        if self._leader_sub is not None:
            self.destroy_subscription(self._leader_sub)
            self._leader_sub = None

        # 切换长机（或从"有长机"切到"没有长机"）时清空轨迹缓冲区——
        # 旧长机的历史轨迹点对新长机的跟随逻辑毫无意义，留着只会让
        # TandemGeometry第一次调用时用旧数据算出一个错误的目标点。
        self._leader_path.clear()
        self._path_prepad_m = 0.0
        self._last_s_proj = None
        self._leader_latest_z = 0.0
        self._current_leader_ns = desired_ns

        if self._leader_traj_sub is not None:
            self.destroy_subscription(self._leader_traj_sub)
            self._leader_traj_sub = None
        self._leader_traj = LeaderPlannedTraj()

        if desired_ns:
            topic = f'/{desired_ns}/' + str(self.get_parameter('uwb_topic').value)
            self._leader_sub = self.create_subscription(PoseStamped, topic, self._on_leader_uwb, 10)
            # 长机的规划轨迹（方法 A）。traj_utils 来自 ego_planner_ws，按理一定在，
            # 但订不上也不能把编队搞挂——退回原来的前馈就是了。
            try:
                from traj_utils.msg import Bspline
                self._leader_traj_sub = self.create_subscription(
                    Bspline, f'/{desired_ns}/planning/bspline', self._on_leader_traj, 10)
                self.get_logger().info(f'已订阅长机规划轨迹 /{desired_ns}/planning/bspline（前馈用未来速度）')
            except Exception as exc:
                self.get_logger().warn(
                    f'订不上 /{desired_ns}/planning/bspline（{exc}），'
                    f'速度前馈退回"由长机已飞轨迹反推"的旧算法')
            self.get_logger().info(f'开始订阅长机UWB绝对位置：{topic}（跨机共享系）')
        else:
            self.get_logger().info('已停止跟随（不再订阅任何长机里程计）')

    def _on_own_odom(self, msg: Odometry) -> None:
        """自身里程计。只用于入列判据（算实际位置与应在位置的距离），
        稳态跟随本身不需要——目标点是从长机航迹回溯出来的，跟自己当前
        在哪无关。"""
        p = msg.pose.pose.position
        self._own_xy = (p.x, p.y)
        self._own_z = p.z
        q = msg.pose.pose.orientation
        self._own_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_range(self, msg: Range) -> None:
        """定高雷达。uwb_imu 定位源本来就在用同一条话题做 z 融合（见
        uwb_imu_fusion_node 的 range_topic 参数），真机上是同一颗雷达、
        同一个话题名，所以这里不引入任何仿真专有依赖。"""
        r = float(msg.range)
        if msg.min_range <= r <= msg.max_range:
            self._agl = r

    def _on_own_uwb(self, msg: PoseStamped) -> None:
        """自己的 UWB 绝对位置（跨机共享系）。"""
        self._own_uwb_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_leader_uwb(self, msg: PoseStamped) -> None:
        """长机的 UWB 绝对位置——轨迹就记在这个共享系里。

        为什么不用长机的 odom（2026-09-21订正）：两机 odom 原点各自锁定，
        误差会直接叠进编队位置，实测同一套代码两次运行差 1.19 米。UWB 绝对
        系是两机共用的同一个锚点系，没有这个问题。
        """
        self._leader_last_rx = time.monotonic()
        p = msg.pose.position
        # ---- 2026-09-28：第一个点之前先补一段"向后延伸" ----
        # 为什么要补：僚机的起始站位在长机起飞点**后方** follow_distance_m 处
        # （编队示例 2026-09-28 起会先飞过去站好）。而 project_arc_length() 把
        # 投影钳在 [0, 全长] 内，站在起点之前的僚机投影出来就是 0——跟"正好站在
        # 长机起飞点上"完全无法区分。于是那 follow_distance_m 的身位差成了一笔
        # **隐形欠账**：间距约束 s_limit_gap = 全长 - follow_distance 仍然按
        # "僚机在起点上"来算，僚机得多飞这一段才能补回来，而它跟长机同速，
        # 只能靠追赶项一点点还，实测起步阶段明显落后。
        # 补上这一段之后，弧长 0 就是僚机**当前实际位置**、长机从一开始就在
        # 一个身位之外，两边的账本对齐，起步瞬间 lag=0。
        #
        # 2026-09-28 订正（用户指出）：延伸的终点直接取僚机的实测位置，不要用
        # "航线第一段方向 × follow_distance"算出来的名义站位——僚机站位有零点
        # 几米的误差，用实测值账本才真对得上。而且这样**判据一个字都不用改**：
        # 原来的 total_length >= follow_distance 一上来就被这段延伸满足，僚机
        # 直接进入跟随态；而跟随的数学本身会把它按住不动（s_limit_gap =
        # 全长 - follow_distance ≈ 0，参考点就停在僚机自己身上），长机不动它就
        # 不动。hold 那一段等于自动失效，不需要再判"长机飞够 3.5 米没有"——
        # 上一版扣掉 prepad 反而把等待拉长到 7 秒（叠上起步缓加速后更明显）。
        if len(self._leader_path) == 0:
            # 僚机自己在 UWB 系里的位置——节点本来就订着自己的 uwb/pose_abs
            # 存在这个属性里（见 _on_own_uwb），跟长机轨迹同一个系，直接用。
            anchor = self._own_uwb_xy
            if anchor is not None:
                d = math.hypot(p.x - anchor[0], p.y - anchor[1])
                self._leader_path.append(anchor[0], anchor[1], p.z,
                                         self._stamp_to_sec(msg.header.stamp))
                self._path_prepad_m = d
                self.get_logger().info(
                    f'长机轨迹向后延伸到僚机当前位置 ({anchor[0]:.2f}, {anchor[1]:.2f})，'
                    f'长 {d:.2f} m，弧长0对齐到僚机')
        self._leader_raw_xy = (p.x, p.y)
        # ---- 轨迹清洗：只有让航线进度前进的点才进缓冲区（见 path_prune_max_chord_m）----
        prune_chord = float(self.get_parameter('path_prune_max_chord_m').value)
        ruler = self._route_ruler_or_none() if prune_chord > 0.0 else None
        if ruler is not None:
            # 长机的航线里程只在这里算一次，控制循环直接读 _s_route_lead——
            # 两处各算一次的话，两个线程会抢同一份展开状态。
            s_lead = ruler.project_unwrapped(p.x, p.y, self._rs_lead)
            self._s_route_lead = s_lead
            if s_lead is not None and len(self._leader_path) > 0:
                if self._lead_route_s_max is None:
                    self._lead_route_s_max = s_lead
                elif s_lead <= self._lead_route_s_max + 1e-6:
                    # 航线进度没前进：冲过头往回退、原地转向飘移之类。丢掉，
                    # 除非丢掉会让轨迹出现一段过长的直线跨越（见参数说明）。
                    last = self._leader_path.last_xy()
                    chord = (math.hypot(p.x - last[0], p.y - last[1])
                             if last is not None else 0.0)
                    if chord <= prune_chord:
                        return
                else:
                    self._lead_route_s_max = s_lead
        self._leader_path.append(p.x, p.y, p.z, self._stamp_to_sec(msg.header.stamp))

    def _route_ruler_or_none(self) -> Optional['RouteRuler']:
        """按当前 leg_route_xy 取（必要时重建）航线里程尺；没航线就返回 None。

        这把尺只用来**清洗长机轨迹**（丢掉不让航线进度前进的点），不当间距
        基准——当间距基准试过，更差，原委见 declare_parameter 那一段。
        """
        pts = self._leg_route_points()
        if len(pts) < 2:
            return None
        key = tuple(pts)
        if key != self._route_key:
            # 往起点之前延一段，长度取跟随距离再加点余量：僚机的起始站位在
            # 长机起飞点后方，不延的话它的投影会被钳在 0（见 RouteRuler）。
            back = float(self.get_parameter('follow_distance_m').value) + 2.0
            self._route_ruler = RouteRuler(pts, back_extend_m=back)
            self._route_key = key
            self._rs_lead, self._rs_own = {}, {}
            self._lead_route_s_max = None
            self.get_logger().info(
                f'航线里程尺已建立：{len(pts)} 个航点，总长 {self._route_ruler.total():.1f} m'
                f'（向后延伸 {back:.1f} m）。进度与间距按航线量，飞行路径仍是长机实飞轨迹')
        return self._route_ruler if (self._route_ruler and self._route_ruler.ok()) else None

    def _on_leader_traj(self, msg) -> None:
        self._leader_traj.update(msg, self._now_sec())

    def _now_sec(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def _leg_route_points(self) -> List[Tuple[float, float]]:
        """把 leg_route_xy 这个拉平的 double 数组还原成 [(x, y), ...]。"""
        flat = list(self.get_parameter('leg_route_xy').value or [])
        return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

    def _uwb_to_odom_offset(self) -> Optional[Tuple[float, float]]:
        """UWB 系 -> 自己 odom 系 的实时平移。

        用自己的两套读数直接相减，所以不依赖任何跨机标定，也不依赖开机时
        锁定的原点——原点锁定有多少误差，这个偏移就自动补偿多少。
        """
        if self._own_xy is None or self._own_uwb_xy is None:
            return None
        return (self._own_xy[0] - self._own_uwb_xy[0], self._own_xy[1] - self._own_uwb_xy[1])

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    # -------------------- 主控制循环 --------------------

    def _on_control_tick(self) -> None:
        if not bool(self.get_parameter('enabled').value):
            return  # 待命状态：空跑，不发布任何目标（清单A3验收标准第1条）

        geometry_type = str(self.get_parameter('geometry_type').value)
        strategy_type = str(self.get_parameter('strategy_type').value)
        geometry = self._geometries[geometry_type]
        strategy = self._strategies[strategy_type]

        follow_distance_m = float(self.get_parameter('follow_distance_m').value)

        # ---- 轨迹跟随主路径（2026-09-20）：直接发 PositionCommand ----
        # 设定点逐点复现长机走过的轨迹，不经过 ego_planner，见 cmd_rate_hz
        # 参数上面的说明。geometry/strategy 两层抽象仍然保留给"非纵列队形"
        # 用（横排并飞那类目标点不在长机航迹上，仍需规划器接管）。
        if geometry_type == 'tandem':
            if self._own_xy is None:
                self.get_logger().warn('还没收到自己的里程计，暂不发布指令', throttle_duration_sec=5.0)
                return
            offset = self._uwb_to_odom_offset()
            if offset is None:
                self.get_logger().warn('还没收到自己的UWB位置，暂不发布指令', throttle_duration_sec=5.0)
                return

            # 长机还没真正走起来时原地保持：此时缓冲区只有它起飞点附近一小簇
            # 点，"落后3.5米"会落在缓冲区起点，僚机会被拉着往长机起飞点飘
            # （实测现象：长机还没动，僚机先飞了）。等轨迹长度够了再入列。
            if self._leader_path.total_length() < follow_distance_m:
                self._publish_hold(own_z_hold=True)
                self.get_logger().info(
                    f'长机轨迹长度{self._leader_path.total_length():.1f}m < 跟随距离'
                    f'{follow_distance_m}m，原地保持等待长机起步',
                    throttle_duration_sec=5.0,
                )
                return

            # 在 UWB 共享系里算 carrot（自己的位置也换到 UWB 系）
            own_uwb_x = self._own_xy[0] - offset[0]
            own_uwb_y = self._own_xy[1] - offset[1]

            # ---- 参考点沿长机轨迹匀速前进 ----
            # 2026-09-21 按用户描述重写："都是各自飞各自的，匀速，只是间距
            # 大于阈值，僚机沿长机轨迹飞即可"。
            # 三句话就是全部逻辑，没有控制律：
            #   ① 参考点弧长每个 tick 前进 cruise_speed_mps * dt（匀速）；
            #   ② 不越过"落后长机 follow_distance_m"那条线（间距下限）；
            #   ③ 不领先僚机实际投影位置超过 lookahead_m（飞机跟不上时等它）。
            # 取点永远沿折线插值，所以走的就是长机走过的路径，不会抄近道。
            # 传上一次的投影弧长消歧义：航线闭合时僚机在起降点附近会 snap 到
            # 轨迹起点，lag 瞬间变成几十米（见 project_arc_length 的说明）。
            s_proj = self._leader_path.project_arc_length(
                own_uwb_x, own_uwb_y, last_s=self._last_s_proj)
            if s_proj is not None:
                self._last_s_proj = s_proj
            if s_proj is None:
                self.get_logger().warn(
                    f'enabled=True但还没收到过长机({self._current_leader_ns})的里程计，暂不发布指令',
                    throttle_duration_sec=5.0,
                )
                return

            now = time.monotonic()
            dt = 0.0 if self._s_cmd_t is None else max(0.0, min(0.5, now - self._s_cmd_t))
            self._s_cmd_t = now
            if self._s_cmd is None:
                self._s_cmd = s_proj      # 入列瞬间从飞机当前投影位置起步
            cruise = float(self.get_parameter('cruise_speed_mps').value)
            # 有效跟随距离 = 标称值 - 偏置补偿量（见 bias_trim_gain 处说明）
            fd_eff = follow_distance_m - self._fd_trim
            s_limit_gap = max(0.0, self._leader_path.total_length() - fd_eff)
            s_limit_lead = s_proj + float(self.get_parameter('lookahead_m').value)
            # 真实间距：沿长机（已清洗的）实飞轨迹，从僚机投影到长机当前位置
            gap_now = self._leader_path.total_length() - s_proj
            # ---- 2026-09-28 改：参考点由"匀速推进"改成"锁在间距线上，速度只做限幅" ----
            # 原来 cruise 跟长机的 max_vel 一模一样（都是 1.0），于是 s_cmd 每拍
            # 前进的量跟 s_limit_gap 每拍增长的量**恰好相等**——一旦落后就永远
            # 追不回来，长机每到一个航点还要停下转向（先悬停转向再前飞），走走停停
            # 之间落后量不断累积。实测间距在 2.7~5.3 m 之间来回摆、中位 3.86。
            # 现在：给一个高于长机巡航速度的**变化率上限**，让真正起约束作用的是
            # s_limit_gap（= 长机弧长 - 跟随距离）。这样长机停、参考点就停；长机
            # 加速、参考点跟着加速；弧长间距恒等于 follow_distance_m，只剩飞机
            # 自身跟踪滞后那一点点误差。
            # rate_headroom 是相对长机巡航速度的余量，1.0 等于退回旧行为。
            cruise *= max(1.0, float(self.get_parameter('reference_rate_headroom').value))
            # 拐点前馈：长机快到航点了就提前收油（前馈，不等落后量涨起来）
            ff_r = float(self.get_parameter('corner_ff_radius_m').value)
            if ff_r > 0.0:
                # 用长机**实测**位置，不是缓冲区末点——轨迹清洗开着的时候，
                # 长机在拐点回钩那几秒里的点会被丢掉，缓冲区末点是停滞的。
                lead_xy = self._leader_raw_xy or self._leader_path.last_xy()
                pts = self._leg_route_points()
                if lead_xy is not None and len(pts) >= 2:
                    d_corner = min(math.hypot(lead_xy[0] - px, lead_xy[1] - py)
                                   for px, py in pts)
                    if d_corner < ff_r:
                        ff_min = float(self.get_parameter('corner_ff_min_ratio').value)
                        cruise *= max(ff_min, d_corner / ff_r)
            # 参考点速率限加速度：目标速率是 cruise，但每拍最多改
            # reference_accel_limit * dt，避免一拍跳满、僚机猛加速
            accel_lim = float(self.get_parameter('reference_accel_limit').value)
            if accel_lim > 0.0:
                if self._s_rate is None:
                    self._s_rate = 0.0
                step = accel_lim * dt
                self._s_rate += max(-step, min(step, cruise - self._s_rate))
                cruise = self._s_rate
            s_hold = self._s_cmd          # 转向期间要钉回来的弧长（分段航向用）
            s_before = self._s_cmd
            # 分两步，别合并：
            #   s_want —— 只受**真实物理约束**（速率上限 + 不能贴长机太近）；
            #   s_cmd  —— 再套上 lookahead，那只是"设定点别跑到飞机前面太远"的
            #             安全阀，不是物理约束。
            s_want = min(self._s_cmd + cruise * dt, s_limit_gap)
            self._s_cmd = min(s_want, s_limit_lead)
            # 前馈取 s_want 的推进速率，**不是** s_cmd 的。
            # 2026-09-28 踩过的坑：一开始拿 s_cmd 的达成速率当前馈，起步时
            # 参考点被 lookahead 钳在静止飞机前方 0.6 m，达成速率≈0 -> 前馈≈0
            # -> 飞机只靠 0.6 m 位置误差慢慢挪 -> 参考点继续被钳住，自锁。
            # 实测僚机起步后纹丝不动 4 秒、间距拉到 7.5 m（run38 t=29~33）。
            # 用 s_want：长机一动，间距线就以长机的速度推进，前馈立刻是 1 m/s；
            # 稳态时 s_want 和 s_cmd 推进速率本来就相等，也不会多喂。
            if dt > 1e-3:
                rate_now = max(0.0, (s_want - s_before) / dt)
                tau = max(1e-3, float(self.get_parameter('ff_rate_tau_s').value))
                a_ff = min(1.0, dt / tau)
                self._s_rate_ff = (rate_now if self._s_rate_ff is None
                                   else self._s_rate_ff + a_ff * (rate_now - self._s_rate_ff))
            # 只前进不后退：长机轨迹是单向增长的，参考点回退没有物理意义
            self._s_cmd = max(self._s_cmd, s_proj)

            target = self._leader_path.point_at_arc_length(self._s_cmd)
            if target is None:
                return
            cx = target[0] + offset[0]     # 换回自己的 odom 系再发给 pt4ctrl
            cy = target[1] + offset[1]
            self._last_target_xy = (cx, cy)
            # 落后量：实际间距超出有效跟随距离多少（>0 = 落后）。航线基准下
            # 这两个量都按航线里程算，跟评价指标同一把尺。
            self._last_lag_m = gap_now - fd_eff
            diag = Float64MultiArray()
            diag.data = [float(self._last_lag_m), float(self._s_cmd), float(s_proj),
                         float(self._leader_path.total_length()),
                         float(gap_now),
                         float(self._fd_trim),
                         float(self._s_rate_ff if self._s_rate_ff is not None else 0.0),
                         float(self._leader_traj_ff_used)]
            self.diag_pub.publish(diag)

            # 步长限幅：设定点最多领先当前位置 max_lead_m。参考点已经落在
            # 轨迹上，这一步只是防止"一次给出好几米外的点"被 PX4 当成远距离
            # 阶跃（2026-09-20 实测摔过一架）。僚机刚从自己起降点靠拢轨迹
            # 时会走到这一支。
            max_lead = float(self.get_parameter('max_lead_m').value)
            dx, dy = cx - self._own_xy[0], cy - self._own_xy[1]
            dist = math.hypot(dx, dy)
            if dist > max_lead and dist > 1e-6:
                k = max_lead / dist
                sx, sy = self._own_xy[0] + dx * k, self._own_xy[1] + dy * k
            else:
                sx, sy = cx, cy

            # 高度：固定 AGL 定高（见 follow_altitude_agl_m 参数说明）。
            # 目标离地高度与当前离地高度之差，就是 odom 系下该走的 z 增量。
            own_z = self._own_z if self._own_z is not None else 0.0
            if self._yaw_ref is None:
                self._yaw_ref = self._own_yaw if self._own_yaw is not None else 0.0
                self.get_logger().info(
                    f'朝向锁定：{math.degrees(self._yaw_ref):.1f}度（编队期间保持不变）'
                )
            target_agl = float(self.get_parameter('follow_altitude_agl_m').value)
            if self._agl is not None:
                target_z = own_z + (target_agl - self._agl)
            else:
                # 拿不到雷达读数就保持当前高度，不猜——宁可不升不降
                target_z = own_z
                self.get_logger().warn(
                    '收不到定高雷达读数，暂时保持当前高度', throttle_duration_sec=5.0
                )
            max_z_lead = float(self.get_parameter('max_z_lead_m').value)
            sz = max(own_z - max_z_lead, min(own_z + max_z_lead, target_z))

            cmd = PositionCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = self._own_odom_frame
            cmd.position.x, cmd.position.y, cmd.position.z = float(sx), float(sy), float(sz)
            # 速度字段留 0：本节点不做控制律。2026-09-21（用户："搞什么
            # 控制啊，越搞越复杂"）把原来的"长机历史速度前馈 + 按落后量的
            # 追赶增益 + 速度上限"整套删掉了——间距 3.5 米是**下限**不是
            # 目标值（用户："只要轨迹上的距离大于间距就行，不是非得保持
            # 那个间距"），所以根本不需要追赶：沿轨迹往前推进、推进点不
            # 越过"落后长机 3.5 米"那条线，就已经满足要求。飞多快交给
            # PX4 自己的位置环。
            # 速度字段：参考点正以 cruise 匀速沿轨迹前进，把这个速度按
            # 当前前进方向给出去当前馈——PX4 的 position_cmd 语义里 velocity
            # 就是"这一点上的速度"，给了它飞机才能真的匀速走，而不是靠位置
            # 误差出速度（那样越接近越慢）。方向取轨迹在参考点处的切线，
            # 不是指向设定点的直线方向，所以不会把弯道切掉。
            # 这不是控制增益：大小恒为 cruise，不随误差变化。
            tangent = self._leader_path.point_at_arc_length(self._s_cmd + 0.3)
            if tangent is not None and (tangent[0] != target[0] or tangent[1] != target[1]):
                tdir = math.atan2(tangent[1] - target[1], tangent[0] - target[0])
            else:
                tdir = self._yaw_ref if self._yaw_ref is not None else 0.0
            moving = self._s_cmd < s_limit_gap - 1e-3    # 顶到间距下限就停下等

            # 分段航向：走到下一段就地停下转到该段航向，转到位再走（见参数声明处）
            if bool(self.get_parameter('yaw_follow_leg').value):
                tol = math.radians(float(self.get_parameter('yaw_leg_tol_deg').value))
                own_yaw = self._own_yaw if self._own_yaw is not None else self._yaw_ref
                pts = self._leg_route_points()
                if len(pts) >= 2 and not self._yaw_turning:
                    # 参考点 target 就在共享（UWB/世界）系里，直接拿它定位在第几段。
                    # 只前进不后退：投影超过本段末端就进下一段。
                    slack = float(self.get_parameter('leg_corner_slack_m').value)
                    while self._leg_idx < len(pts) - 2:
                        a, b = pts[self._leg_idx], pts[self._leg_idx + 1]
                        leg_len = math.hypot(b[0] - a[0], b[1] - a[1])
                        done_at = 1.0 if leg_len < 1e-6 else max(0.5, 1.0 - slack / leg_len)
                        if _seg_progress(a, b, target[0], target[1]) < done_at:
                            break
                        self._leg_idx += 1
                    a, b = pts[self._leg_idx], pts[self._leg_idx + 1]
                    leg_dir = math.atan2(b[1] - a[1], b[0] - a[0])
                    if not bool(self.get_parameter('yaw_turn_in_place').value):
                        # 协调转弯：机头按限速连续转过去，**不停**（见参数说明）。
                        # 每拍挪一点，所以这里不设 _yaw_turning，参考点照常推进。
                        step = math.radians(
                            float(self.get_parameter('yaw_slew_rate_dps').value)) * dt
                        d = math.atan2(math.sin(leg_dir - self._yaw_ref),
                                       math.cos(leg_dir - self._yaw_ref))
                        self._yaw_ref = math.atan2(
                            math.sin(self._yaw_ref + max(-step, min(step, d))),
                            math.cos(self._yaw_ref + max(-step, min(step, d))))
                    elif _ang_diff(leg_dir, self._yaw_ref) > tol:
                        self._yaw_ref = leg_dir        # 这一段的航向，中途不再变
                        self._yaw_turning = True
                        self.get_logger().info(
                            f'第{self._leg_idx + 1}段：先转到 {math.degrees(leg_dir):.0f}° 再前飞'
                            f'（当前 {math.degrees(own_yaw):.0f}°）')
                if self._yaw_turning:
                    if _ang_diff(own_yaw, self._yaw_ref) <= tol:
                        self._yaw_turning = False
                        self.get_logger().info(
                            f'航向已到位（{math.degrees(own_yaw):.0f}°），继续前飞')
                    else:
                        # 转向期间悬停：参考点不前进，位置指令钉在转向前那一点
                        self._s_cmd = s_hold
                        moving = False
                        if self._last_target_xy is not None:
                            sx, sy = self._last_target_xy

            # 前馈速度：参考点**实际**推进的速率（见 ff_rate_tau_s 处说明）。
            # cruise 只是上限，发它会多喂 50%。
            v_ff = self._s_rate_ff if self._s_rate_ff is not None else 0.0
            # ---- 方法 A：有长机规划轨迹就用它的**未来**速度当前馈 ----
            # 只取速度大小，方向仍用轨迹切线 tdir（那个已经在共享系里算好）。
            # 速度大小对平移/旋转都不变，所以不需要 odom->UWB 的坐标变换。
            if bool(self.get_parameter('leader_traj_ff_enabled').value):
                lead = float(self.get_parameter('leader_traj_ff_lead_s').value)
                now_s = self._now_sec()
                v_lead = self._leader_traj.speed_at(
                    now_s + lead, now_s,
                    float(self.get_parameter('leader_traj_ff_stale_s').value))
                if v_lead is not None:
                    v_ff = v_lead
                    self._leader_traj_ff_used += 1
                elif self._leader_traj_ff_used:
                    self.get_logger().warn(
                        '长机规划轨迹过期/缺失，速度前馈退回旧算法',
                        throttle_duration_sec=5.0)
            v_ff = max(0.0, min(v_ff, cruise))
            # 2026-09-29 订正：前馈**不再**拿 moving 去硬砍成 0。
            # 原来是 `v_ff if moving else 0.0`，而 moving = 参考点还没顶到间距线。
            # 后果是僚机一追上、参考点贴上间距线，前馈瞬间归零、飞机只剩位置误差
            # 驱动，迅速减速；掉队之后前馈又满血恢复——一个 bang-bang，表现就是
            # 走走停停。实测 run58 全程十几次连续低速(<0.3 m/s) 5~8 秒，**直道
            # 中段也停**（t=92~99 在 B->C 中段、t=162~170 在 D 段中段），跟拐点无关。
            # v_ff 本身就是"参考点推进速率"的低通值：长机停了它自然趋零，不需要
            # 额外的开关。真正要钉住的只有"原地转向悬停"那一种情况。
            hold = self._yaw_turning
            cmd.position.x, cmd.position.y = float(sx), float(sy)
            cmd.velocity.x = 0.0 if hold else float(v_ff * math.cos(tdir))
            cmd.velocity.y = 0.0 if hold else float(v_ff * math.sin(tdir))
            cmd.velocity.z = 0.0
            cmd.yaw = float(self._yaw_ref if self._yaw_ref is not None else 0.0)
            cmd.trajectory_id = 1
            self.cmd_pub.publish(cmd)

            # 偏置补偿积分：只在"正常巡航"时积（转向悬停、顶到间距下限时飞机
            # 本来就该停，那时的落后量不是稳态误差，积进去会积饱和）。
            trim_gain = float(self.get_parameter('bias_trim_gain').value)
            if trim_gain > 0.0 and moving and not self._yaw_turning and dt > 1e-3:
                # 积的是**真实间距误差**，不是 _last_lag_m。
                # lag = s_limit_gap - s_proj = (实际间距 - fd_eff) = err + trim，
                # 拿它当误差等于把积分器自己的输出也积进去（正反馈）；而且参考点
                # 被 lookahead 钳在 s_proj+0.6 时 lag>=0.6 恒成立，积分永远不收敛，
                # 必然一路顶到限幅（2026-09-28 run33 实测 trim railed 到 1.0、
                # 实际间距被拉近到 3.14）。真实间距 = 长机轨迹全长 - 僚机投影。
                err = gap_now - follow_distance_m
                self._fd_trim += trim_gain * err * dt
                self._fd_trim = max(float(self.get_parameter('bias_trim_min_m').value),
                                    min(float(self.get_parameter('bias_trim_max_m').value),
                                        self._fd_trim))
            return

        params = {'follow_distance_m': follow_distance_m}
        try:
            target = geometry.compute_target(self._leader_path, params)
        except NotImplementedError as exc:
            # LineAbreastGeometry这类占位实现被选中时会走到这里——不让
            # 节点崩掉，只报警告，方便排查"是不是选了个还没实现的
            # geometry_type"这种配置错误。
            self.get_logger().error(f'当前geometry_type={geometry_type!r}还未实现: {exc}', throttle_duration_sec=5.0)
            return

        if target is None:
            # 还没收到过长机的任何odom（缓冲区为空）——正常的启动瞬间
            # 状态，不是错误，节流打个提示方便排查"怎么一直不动"。
            self.get_logger().warn(
                f'enabled=True但还没收到过长机({self._current_leader_ns})的里程计，暂不发布目标',
                throttle_duration_sec=5.0,
            )
            return

        track_params = {
            'publisher': self.goal_pub,
            'frame_id': self._own_odom_frame,
            'stamp': self.get_clock().now().to_msg(),
            'z': self._leader_latest_z,
        }
        self._last_target_xy = (target.x, target.y)
        try:
            strategy.track(target, track_params)
        except NotImplementedError as exc:
            self.get_logger().error(f'当前strategy_type={strategy_type!r}还未实现: {exc}', throttle_duration_sec=5.0)


    # ==================== Action：入列 / 保持 / 出列 ====================
    #
    # 2026-09-20新增。原来只有"设 enabled 参数"这一个入口，没有阶段、没有
    # 反馈、不能取消，调用方也无从知道"队形组好了没有"。现在补成 action：
    #   join  入列：启用跟随，等实际位置与应在位置的距离进容差并保持一段时间
    #   track 保持：稳态跟随，feedback 持续回报间距与长机可见性
    #   break 出列：收到 cancel，停止发布目标并交还控制权，返回 CANCELED
    #
    # 参数式接口（enabled/leader_namespace/follow_distance_m）保留不动，
    # 老调用方与 `ros2 param set` 手工调试都不受影响；SDK 探测不到 action
    # server 时会自动退回那条路。

    def _on_action_goal(self, goal_request) -> GoalResponse:
        if self._action_busy.locked():
            self.get_logger().warn('已有编队跟随 goal 在执行，拒绝新的 goal')
            return GoalResponse.REJECT
        if not goal_request.leader_namespace:
            self.get_logger().error('goal 里 leader_namespace 为空，拒绝')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_action_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('收到取消请求，将出列并停止发布目标')
        return CancelResponse.ACCEPT

    def _execute_formation(self, goal_handle):
        with self._action_busy:
            return self._run_formation(goal_handle)

    def _run_formation(self, goal_handle):
        req = goal_handle.request
        leader_ns = str(req.leader_namespace)
        follow_distance_m = float(req.follow_distance_m)
        join_timeout_s = float(req.join_timeout_s)
        if join_timeout_s <= 0.0:
            join_timeout_s = float(self.get_parameter('join_timeout_s').value)

        # 复用既有的参数路径来真正启用跟随：这样 action 与 `ros2 param set`
        # 两个入口操作的是同一份状态，不会出现"action 以为在跟、参数说没在跟"
        # 这种分裂。
        self.set_parameters([
            Parameter('leader_namespace', Parameter.Type.STRING, leader_ns),
            Parameter('follow_distance_m', Parameter.Type.DOUBLE, follow_distance_m),
            Parameter('enabled', Parameter.Type.BOOL, True),
        ])
        self._yaw_ref = None
        self._s_cmd = None
        self._last_s_proj = None
        self._s_rate = None
        self._s_rate_ff = None
        self._fd_trim = 0.0
        self._route_key = None          # 强制重建里程尺（跟随距离变了、航线可能也变了）
        self._route_ruler = None
        self._rs_lead, self._rs_own = {}, {}
        self._lead_route_s_max = None
        self._s_route_lead = None
        self._s_cmd_t = None
        self._yaw_turning = False
        self._leg_idx = 0
        # 接管控制权：跟随者的设定点由本节点直接喂给 pt4ctrl，绕开 ego_planner
        self._set_relay_mode('formation')
        self.get_logger().info(
            f'编队跟随 goal 开始：跟随{leader_ns}，间距{follow_distance_m}米，'
            f'入列超时{join_timeout_s:.0f}秒（轨迹跟随，relay_mode=formation）'
        )

        # 三段：hold（已接管、原地保持，等长机起步）-> join（长机动了，
        # 沿轨迹追到落后量容差内）-> track（保持跟随）。
        # hold 这一段是 2026-09-21 加的：原来一上来就是 join，而 join 有
        # 45秒超时——长机如果在等僚机报"准备好了"才起步，两边互相等，
        # 僚机这边45秒后直接 abort，编队根本起不来。
        phase = 'hold'
        phase_start = time.monotonic()
        within_since: Optional[float] = None
        tick = 1.0 / max(1.0, float(self.get_parameter('rate_hz').value))

        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self._disable_follow()
                    goal_handle.canceled()
                    return self._formation_result(True, 'break', '已出列，停止发布目标')

                now = time.monotonic()
                gap = self._current_gap_m()
                leader_visible = (now - self._leader_last_rx) <= float(
                    self.get_parameter('leader_lost_timeout_s').value
                ) if self._leader_last_rx else False
                self._publish_formation_feedback(goal_handle, phase, gap, leader_visible)

                if phase == 'hold':
                    # 长机走出超过一个跟随距离，才算"起步了"——这跟
                    # `_on_control_tick`里"轨迹长度不够就原地保持"是同一个
                    # 判据，保证 feedback 里的阶段跟实际控制行为一致。
                    moved = self._leader_path.total_length() >= follow_distance_m
                    if moved:
                        phase, phase_start = 'join', now
                        self.get_logger().info(
                            f'长机已起步（轨迹长度{self._leader_path.total_length():.1f}m ≥ '
                            f'{follow_distance_m}m），开始入列'
                        )
                    elif now - phase_start > float(
                            self.get_parameter('leader_start_timeout_s').value):
                        self._disable_follow()
                        goal_handle.abort()
                        return self._formation_result(
                            False, 'hold',
                            f'等长机起步超时（'
                            f'{float(self.get_parameter("leader_start_timeout_s").value):.0f}秒）：'
                            f'长机轨迹长度只有{self._leader_path.total_length():.1f}m，'
                            f'一直没有走出{follow_distance_m}m。检查长机是否起飞/是否收到'
                            f'长机的UWB绝对位置'
                        )

                elif phase == 'join':
                    tol = float(self.get_parameter('join_tolerance_m').value)
                    hold = float(self.get_parameter('join_hold_s').value)
                    if gap is not None and gap <= tol:
                        if within_since is None:
                            within_since = now
                        elif now - within_since >= hold:
                            phase, phase_start = 'track', now
                            self.get_logger().info(f'入列完成（间距{gap:.2f}m ≤ 容差{tol}m）')
                    else:
                        within_since = None
                    if phase == 'join' and now - phase_start > join_timeout_s:
                        self._disable_follow()
                        goal_handle.abort()
                        gap_text = f'{gap:.2f}m' if gap is not None else '未知'
                        return self._formation_result(
                            False, 'join',
                            f'入列超时（{join_timeout_s:.0f}秒）：当前间距{gap_text}仍未进入容差。'
                            f'可能原因：长机里程计收不到、跟随者被卡住、或间距参数设得过小'
                        )

                elif phase == 'track':
                    if not leader_visible:
                        self._disable_follow()
                        goal_handle.abort()
                        return self._formation_result(
                            False, 'track',
                            f'长机{leader_ns}的里程计超过'
                            f'{self.get_parameter("leader_lost_timeout_s").value}秒没有更新，判定长机丢失'
                        )

                time.sleep(tick)
        except Exception:  # noqa: BLE001 - 任何异常都要先把跟随关掉再抛
            self._disable_follow()
            raise

    def _set_relay_mode(self, mode: str) -> bool:
        """把 position_cmd_relay 的 relay_mode 切到指定值。

        ⚠️ 这里**不能**用 `rclpy.spin_until_future_complete()`——本方法是在
        action 的 execute_callback 里调的，而那个回调本身就跑在 executor
        线程上，再去 spin 同一个节点会死锁。清单135记过同款事故
        （`_set_relay_mode()`阻塞调用死锁，E6排查时修过一次）。改成轮询
        future 的完成标志，executor 照常在别的线程处理回调。

        失败只记日志不抛异常：接管/交还失败不该让节点崩掉，飞机停在原来的
        relay_mode 下比节点消失更安全。
        """
        if not bool(self.get_parameter('auto_switch_relay_mode').value):
            return True
        if not self.relay_set_params_cli.service_is_ready():
            self.get_logger().error(
                f'{self.relay_set_params_cli.srv_name}服务不可用，无法把relay_mode切到{mode}'
            )
            return False
        req = SetParameters.Request()
        pm = ParameterMsg()
        pm.name = 'relay_mode'
        pm.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode)
        req.parameters = [pm]
        future = self.relay_set_params_cli.call_async(req)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'切换relay_mode到{mode}超时/失败')
            return False
        self.get_logger().info(f'relay_mode -> {mode}')
        return True

    def _disable_follow(self) -> None:
        """出列：停止发布目标点。只关 enabled，不动 leader_namespace/
        follow_distance_m——留着方便下次直接再启用，也方便事后查"上次跟的谁"。

        注意这里**不**额外发降落或悬停指令：停止发布目标后，飞机停在最后
        一个目标点上由 pt4ctrl 的 AUTO_HOVER 维持，这本身就是安全状态。
        """
        self.set_parameters([Parameter('enabled', Parameter.Type.BOOL, False)])
        self._yaw_ref = None        # 下次入列重新锁朝向
        # 交还控制权，让 ego_planner 重新接管普通 goto
        self._set_relay_mode('normal')

    def _publish_hold(self, own_z_hold: bool = True) -> None:
        """原地保持：把当前位置当设定点发出去。

        relay_mode 已经切到 formation，这条通路上必须持续有指令，否则
        pt4ctrl 收不到 position_cmd 会退回 AUTO_HOVER——那样虽然也停得住，
        但状态来回跳变不干净。发当前位置等于"钉在原地"，语义明确。
        """
        if self._own_xy is None:
            return
        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._own_odom_frame
        cmd.position.x = float(self._own_xy[0])
        cmd.position.y = float(self._own_xy[1])
        cmd.position.z = float(self._own_z if (own_z_hold and self._own_z is not None) else 0.0)
        cmd.yaw = float(self._yaw_ref if self._yaw_ref is not None
                        else (self._own_yaw if self._own_yaw is not None else 0.0))
        cmd.trajectory_id = 1
        self.cmd_pub.publish(cmd)

    def _current_gap_m(self) -> Optional[float]:
        """僚机相对"应在位置"沿轨迹的落后量（米）。

        2026-09-20改：原来算的是"当前位置到设定点的直线距离"，而设定点现在
        是贴着飞机的 carrot，那个距离恒等于 max_lead，看不出跟随质量。改成
        沿轨迹的落后弧长——0 表示正好在队形位置上，正值表示落后。
        """
        return self._last_lag_m

    def _publish_formation_feedback(self, goal_handle, phase: str, gap, leader_visible: bool) -> None:
        fb = FormationFollow.Feedback()
        fb.phase = phase
        # 用 NaN 表示"还算不出来"，不用 -1.0：落后量为**负**是正常且有意义的
        # 值（僚机略微超前于"落后 follow_distance_m"那条线），而那恰恰是跟得好
        # 时的稳态。用负数当哨兵会让"一切正常"被显示成"间距未知"——2026-09-21
        # 一键测试脚本的结果摘要就是这么被坑的。
        fb.gap_m = float(gap) if gap is not None else float('nan')
        fb.leader_visible = bool(leader_visible)
        tx, ty = self._last_target_xy if self._last_target_xy else (0.0, 0.0)
        fb.target_x, fb.target_y = float(tx), float(ty)
        goal_handle.publish_feedback(fb)

    def _formation_result(self, success: bool, stage: str, message: str):
        res = FormationFollow.Result()
        res.success = success
        res.stage = stage
        res.message = message
        gap = self._current_gap_m()
        res.final_gap_m = float(gap) if gap is not None else -1.0
        self.get_logger().info(f'编队跟随结束: success={success} stage={stage} {message}')
        return res


def main(args=None):
    rclpy.init(args=args)
    node = FormationFollowerNode()
    # action 的 execute_callback 里要长时间循环，订阅回调必须继续更新
    # 长机/自身里程计，单线程 executor 会把自己饿死。
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
