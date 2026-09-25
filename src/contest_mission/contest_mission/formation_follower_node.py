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

    def total_length(self) -> float:
        """缓冲区里这条折线的总水平弧长（米）。跟随者据此判断长机是否
        已经真正走起来——太短就原地等，别被起飞点附近那一小簇点拉走。"""
        return self._total_length()

    def _total_length(self) -> float:
        total = 0.0
        for i in range(len(self._xs) - 1):
            total += math.hypot(self._xs[i + 1] - self._xs[i], self._ys[i + 1] - self._ys[i])
        return total

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
        arc = [0.0]
        for i in range(1, len(self._xs)):
            arc.append(arc[-1] + math.hypot(self._xs[i] - self._xs[i - 1],
                                            self._ys[i] - self._ys[i - 1]))
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

    def project_arc_length(self, own_x: float, own_y: float) -> Optional[float]:
        """僚机当前位置投影到长机折线上，返回它"走到哪了"的弧长坐标。

        只用水平坐标（用户要求：距离判断只考虑水平距离，不考虑垂直距离）。
        """
        n = len(self._xs)
        if n < 2:
            return None
        arc = self._arc_table()
        best_s, best_d2 = 0.0, float('inf')
        for i in range(1, n):
            x0, y0, x1, y1 = self._xs[i - 1], self._ys[i - 1], self._xs[i], self._ys[i]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((own_x - x0) * dx + (own_y - y0) * dy) / seg2))
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (own_x - px) ** 2 + (own_y - py) ** 2
            if d2 < best_d2:
                best_d2, best_s = d2, arc[i - 1] + t * math.sqrt(seg2)
        return best_s

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
        self.declare_parameter('follow_distance_m', 4.0)
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
        # 起步前先对准第一段航向、到位后再等这么久才开始跟随（用户 2026-09-25
        # 要求"变化到位等3秒再跟随，别一边偏航一边飞"）。只作用于**入列前那一次**
        # 对准；拐角处的转向本来就已经是"停下转、转到位再走"。
        self.declare_parameter('pre_follow_settle_s', 3.0)
        # 航点吸附：参考点走到航线上某个航点这么近时，位置指令直接给那个航点的
        # 精确坐标，保证僚机**真的过点**而不是"过附近"。2026-09-25 实测未吸附时
        # 僚机离四个航点最近 0.17~0.43 米——能过，但那是控制精度碰出来的，不是
        # 保证。只吸附中间的航点，不吸附首尾两个（那是长机的起降点，僚机要回
        # 自己的起降点）。给 0 关闭。
        # 到点判据：走完一段之后，先飞到该航点的**精确坐标**、确认进到这个半径
        # 以内，再转向、再走下一段——跟长机的 goto() 到点判据是一个套路。
        # 2026-09-25 实测：只做"参考点经过时把指令吸到航点上"不够，参考点在
        # 吸附半径内只待约 1.6 秒，飞机还没收敛指令就走了，最近距离只从 0.43 米
        # 改善到 0.22 米。给 0 关闭（退回"只跟轨迹、不保证过点"）。
        self.declare_parameter('waypoint_reach_m', 0.15)
        self.declare_parameter('waypoint_reach_timeout_s', 15.0)

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

        # 2026-09-20：入列判据需要知道"我自己在哪"，原来这个节点只订阅
        # 长机odom、不看自己的位置（稳态跟随不需要）。
        self._own_xy: Optional[Tuple[float, float]] = None
        self._own_z: Optional[float] = None
        self._last_lag_m: Optional[float] = None
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
        self._first_align_done: bool = False  # 入列前那次对准是否已完成（含等待）
        self._wp_hold: Optional[Tuple[float, float]] = None   # 正在飞向并确认到点的航点（世界系）
        self._wp_hold_t0: Optional[float] = None
        self._yaw_reached_t: Optional[float] = None
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
        self._leader_latest_z = 0.0
        self._current_leader_ns = desired_ns

        if desired_ns:
            topic = f'/{desired_ns}/' + str(self.get_parameter('uwb_topic').value)
            self._leader_sub = self.create_subscription(PoseStamped, topic, self._on_leader_uwb, 10)
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
        self._leader_path.append(p.x, p.y, p.z, self._stamp_to_sec(msg.header.stamp))

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
                # 起步前的朝向对准放在**等待期间**做。原来放在下面的跟随分支里，
                # 于是顺序变成"长机先飞出 4 米 -> 僚机才开始转向 -> 再稳 3 秒 ->
                # 才起步"，白白慢 5 秒，之后只能靠加速去追（用户 2026-09-25 指出
                # "起步反应慢、后边使劲追"）。挪到这里之后，长机飞够 4 米时僚机
                # 已经对好方向、稳定完毕，可以立刻跟上，追赶幅度自然小。
                if bool(self.get_parameter('yaw_follow_leg').value) and not self._first_align_done:
                    tol0 = math.radians(float(self.get_parameter('yaw_leg_tol_deg').value))
                    settle0 = float(self.get_parameter('pre_follow_settle_s').value)
                    flat0 = list(self.get_parameter('leg_route_xy').value or [])
                    pts0 = [(flat0[i], flat0[i + 1]) for i in range(0, len(flat0) - 1, 2)]
                    if len(pts0) >= 2:
                        leg0 = math.atan2(pts0[1][1] - pts0[0][1], pts0[1][0] - pts0[0][0])
                        own0 = self._own_yaw if self._own_yaw is not None else leg0
                        if self._yaw_ref is None or _ang_diff(leg0, self._yaw_ref) > tol0:
                            self._yaw_ref = leg0
                            self._yaw_reached_t = None
                            self.get_logger().info(
                                f'等长机起步期间先把机头转到第1段航向 '
                                f'{math.degrees(leg0):.0f}°（当前 {math.degrees(own0):.0f}°）')
                        elif _ang_diff(own0, leg0) <= tol0:
                            now0 = time.monotonic()
                            if self._yaw_reached_t is None:
                                self._yaw_reached_t = now0
                            elif now0 - self._yaw_reached_t >= settle0:
                                self._first_align_done = True
                                self.get_logger().info(
                                    f'航向已到位并稳定 {settle0:.0f} 秒，长机一走就能跟上')
                        else:
                            self._yaw_reached_t = None
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
            s_proj = self._leader_path.project_arc_length(own_uwb_x, own_uwb_y)
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
            s_limit_gap = max(0.0, self._leader_path.total_length() - follow_distance_m)
            s_limit_lead = s_proj + float(self.get_parameter('lookahead_m').value)
            s_hold = self._s_cmd          # 转向期间要钉回来的弧长（分段航向用）
            self._s_cmd = min(self._s_cmd + cruise * dt, s_limit_gap, s_limit_lead)
            # 只前进不后退：长机轨迹是单向增长的，参考点回退没有物理意义
            self._s_cmd = max(self._s_cmd, s_proj)

            target = self._leader_path.point_at_arc_length(self._s_cmd)
            if target is None:
                return
            cx = target[0] + offset[0]     # 换回自己的 odom 系再发给 pt4ctrl
            cy = target[1] + offset[1]
            self._last_target_xy = (cx, cy)
            self._last_lag_m = s_limit_gap - s_proj   # 沿轨迹的落后量，仅用于反馈/入列判据

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
                flat = list(self.get_parameter('leg_route_xy').value or [])
                pts = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
                if len(pts) >= 2 and not self._yaw_turning and self._wp_hold is None:
                    # 参考点 target 就在共享（UWB/世界）系里，直接拿它定位在第几段。
                    # 只前进不后退：投影超过本段末端就进下一段。
                    slack = float(self.get_parameter('leg_corner_slack_m').value)
                    prev_idx = self._leg_idx
                    while self._leg_idx < len(pts) - 2:
                        a, b = pts[self._leg_idx], pts[self._leg_idx + 1]
                        leg_len = math.hypot(b[0] - a[0], b[1] - a[1])
                        done_at = 1.0 if leg_len < 1e-6 else max(0.5, 1.0 - slack / leg_len)
                        if _seg_progress(a, b, target[0], target[1]) < done_at:
                            break
                        self._leg_idx += 1
                    if self._leg_idx != prev_idx and 0 < self._leg_idx < len(pts) - 1 \
                            and float(self.get_parameter('waypoint_reach_m').value) > 0.0:
                        # 刚走完一段：先飞到这个航点的精确坐标并确认到点，再转向
                        self._wp_hold = pts[self._leg_idx]
                        self._wp_hold_t0 = now
                        self.get_logger().info(
                            f'到第{self._leg_idx}段末端，先飞到航点 '
                            f'({self._wp_hold[0]:.1f}, {self._wp_hold[1]:.1f}) 确认到点')
                    else:
                        a, b = pts[self._leg_idx], pts[self._leg_idx + 1]
                        leg_dir = math.atan2(b[1] - a[1], b[0] - a[0])
                        if _ang_diff(leg_dir, self._yaw_ref) > tol:
                            self._yaw_ref = leg_dir        # 这一段的航向，中途不再变
                            self._yaw_turning = True
                            self.get_logger().info(
                                f'第{self._leg_idx + 1}段：先转到 {math.degrees(leg_dir):.0f}° 再前飞'
                                f'（当前 {math.degrees(own_yaw):.0f}°）')

                hold_motion = False
                if self._wp_hold is not None:
                    # 正在"飞到航点并确认到点"：位置指令给航点精确坐标、不再前进
                    reach = float(self.get_parameter('waypoint_reach_m').value)
                    t_out = float(self.get_parameter('waypoint_reach_timeout_s').value)
                    d_wp = math.hypot(own_uwb_x - self._wp_hold[0], own_uwb_y - self._wp_hold[1])
                    if d_wp <= reach or (now - self._wp_hold_t0) > t_out:
                        self.get_logger().info(
                            f'航点已到（差 {d_wp:.2f} m'
                            + ('' if d_wp <= reach else f'，等了 {t_out:.0f} 秒超时') + '）')
                        self._wp_hold = None
                        a, b = pts[self._leg_idx], pts[self._leg_idx + 1]
                        leg_dir = math.atan2(b[1] - a[1], b[0] - a[0])
                        if _ang_diff(leg_dir, self._yaw_ref) > tol:
                            self._yaw_ref = leg_dir
                            self._yaw_turning = True
                            self.get_logger().info(
                                f'第{self._leg_idx + 1}段：先转到 {math.degrees(leg_dir):.0f}° 再前飞'
                                f'（当前 {math.degrees(own_yaw):.0f}°）')
                    else:
                        hold_motion = True
                        self._s_cmd = s_hold
                        moving = False
                        sx = self._wp_hold[0] + offset[0]
                        sy = self._wp_hold[1] + offset[1]
                if self._wp_hold is None and self._yaw_turning:
                    if _ang_diff(own_yaw, self._yaw_ref) <= tol:
                        self._yaw_turning = False
                        self._yaw_reached_t = now
                        self.get_logger().info(
                            f'航向已到位（{math.degrees(own_yaw):.0f}°）')
                    else:
                        hold_motion = True
                if not self._yaw_turning and not self._first_align_done:
                    # 入列前那次对准：到位之后还要原地稳 pre_follow_settle_s 秒才起步，
                    # 不允许"一边偏航一边飞"（用户 2026-09-25 要求）
                    settle = float(self.get_parameter('pre_follow_settle_s').value)
                    if self._yaw_reached_t is None:
                        self._yaw_reached_t = now       # 本来就对着，从现在开始算
                    if now - self._yaw_reached_t < settle:
                        hold_motion = True
                    else:
                        self._first_align_done = True
                        self.get_logger().info(
                            f'航向已稳定 {settle:.0f} 秒，开始跟随长机')
                if hold_motion and self._wp_hold is None:
                    # 悬停：参考点不前进、速度给 0、位置钉住不动
                    self._s_cmd = s_hold
                    moving = False
                    if not self._first_align_done:
                        sx, sy = self._own_xy    # 入列前还没贴上航迹，钉自己当前位置
                    elif self._last_target_xy is not None:
                        sx, sy = self._last_target_xy

            cmd.position.x, cmd.position.y = float(sx), float(sy)
            cmd.velocity.x = float(cruise * math.cos(tdir)) if moving else 0.0
            cmd.velocity.y = float(cruise * math.sin(tdir)) if moving else 0.0
            cmd.velocity.z = 0.0
            cmd.yaw = float(self._yaw_ref if self._yaw_ref is not None else 0.0)
            cmd.trajectory_id = 1
            self.cmd_pub.publish(cmd)
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
        self._s_cmd_t = None
        self._yaw_turning = False
        self._leg_idx = 0
        self._first_align_done = False
        self._yaw_reached_t = None
        self._wp_hold = None
        self._wp_hold_t0 = None
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
