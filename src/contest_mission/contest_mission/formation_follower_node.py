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
`origin_setter_node`（在线θ*旋转对齐）是`uwb_slam`模式专属的节点，
只有那种模式下两机各自的局部系可能有相对旋转偏差、需要对齐。这次
contest task用`LOCALIZATION_SOURCE=uwb_imu`定位，两机的odom本身
就是同一个UWB锚点下的全局系坐标——长机局部系=跟随者局部系=世界系，
天然对齐，不存在θ*收敛这个环节。因此这个节点**直接消费长机odom的
x/y**当成跟随者自己局部系下的坐标使用，不做任何TF查询/旋转/平移
换算。（如果以后要支持`uwb_slam`模式下的双机编队，需要在这里加一层
经过`origin_setter_node`当前θ*估计的旋转变换——但那是另一个模式的
另一个问题，这次任务范围内不做，也不预留这个转换的接口占位，避免
"预留了但没人验证过、真正要用时反而更容易出错"。）

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
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.publisher import Publisher


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

    def __init__(self, min_point_gap_m: float = 0.05, max_buffer_length_m: float = 60.0):
        # min_point_gap_m：odom回调频率可能远高于这个节点的控制频率
        # （DLIO/uwb_imu融合通常几十到上百Hz），如果每一帧odom都append
        # 一个点，缓冲区会迅速堆积大量几乎重合的点，白白浪费内存/计算，
        # 对回溯精度也没有任何帮助（两点间距远小于任何有意义的
        # follow_distance_m）。所以两点间距小于这个阈值时直接忽略新点
        # 不追加，这是LeaderPathBuffer内部的实现细节，不是ROS参数
        # （不需要暴露给SDK/选手调），默认0.05米足够细，不影响弧长回溯
        # 精度，同时能把点数控制在合理范围。
        self._min_point_gap_m = float(min_point_gap_m)
        # max_buffer_length_m：缓冲区裁剪上限——只需要保留"最新点往回数
        # 到最大可能follow_distance_m还有余量"这么长的历史，更老的点
        # 对当前查询毫无用处，裁掉避免无限增长。60米对这次任务
        # follow_distance_m=3.5这种量级留了远超十倍的余量，即使以后
        # 换更大的跟随距离参数也够用；真的不够时_trim内部逻辑本身也不
        # 会出错，只是回溯到缓冲区起点就停住（见point_at_arc_length_
        # behind的兜底行为）。
        self._max_buffer_length_m = float(max_buffer_length_m)
        # 三个并行列表，索引对齐：_xs[i]/_ys[i]/_ts[i]是第i个轨迹点。
        self._xs: List[float] = []
        self._ys: List[float] = []
        self._ts: List[float] = []

    def __len__(self) -> int:
        return len(self._xs)

    def clear(self) -> None:
        """清空缓冲区——切换长机(leader_namespace变了)时必须调用，
        否则旧长机的轨迹点会被新长机的跟随逻辑误用（见FormationFollower
        Node._sync_leader_subscription）。"""
        self._xs.clear()
        self._ys.clear()
        self._ts.clear()

    def append(self, x: float, y: float, t: float) -> None:
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
            self._ts.pop(0)

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

        # 策略模式：每种队形/策略各自只实例化一份，节点主循环只认
        # FormationGeometry/FormationControlStrategy这两个抽象接口，
        # 不关心具体是哪个实现（见文件头接口设计说明）。
        self._geometries = {name: cls() for name, cls in _GEOMETRY_REGISTRY.items()}
        self._strategies = {name: cls() for name, cls in _STRATEGY_REGISTRY.items()}

        self.add_on_set_parameters_callback(self._on_set_parameters)

        rate_hz = float(self.get_parameter('rate_hz').value)
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
            topic = f'/{desired_ns}/dlio/odom_node/odom'
            self._leader_sub = self.create_subscription(Odometry, topic, self._on_leader_odom, 10)
            self.get_logger().info(f'开始订阅长机里程计：{topic}')
        else:
            self.get_logger().info('已停止跟随（不再订阅任何长机里程计）')

    def _on_leader_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._leader_path.append(p.x, p.y, self._stamp_to_sec(msg.header.stamp))
        # z不参与弧长回溯（见LeaderPathBuffer/LeaderFollowerStrategy的
        # 说明），单独记最新值，供_on_control_tick组装目标点高度用。
        self._leader_latest_z = p.z

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

        params = {'follow_distance_m': float(self.get_parameter('follow_distance_m').value)}
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
        try:
            strategy.track(target, track_params)
        except NotImplementedError as exc:
            self.get_logger().error(f'当前strategy_type={strategy_type!r}还未实现: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = FormationFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
