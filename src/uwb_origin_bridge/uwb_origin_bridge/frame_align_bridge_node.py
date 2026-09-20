#!/usr/bin/env python3
"""硬件可迁移版frame_align桥接节点——把"双机map系互相对齐"这个量，从仿真端
的Gazebo真值实现，换成真机也能跑的、纯靠已标定localization得到的实现。

背景：`uwb_sim/uwb_ground_truth_node.py`里的`_compute_relative_transform`
已经在发布`/frame_align/{i}/{j}`（喂给mighty_node的use_frame_alignment闭环，
`flight-stack-entrypoint.sh`里`use_frame_alignment:=true`那行已经在用）——
但那个实现直接订阅Gazebo的`/plug/model_states_plug`拿两机真实位姿做差，是
纯仿真专用的作弊数据源，没法原样搬到真机上（真机没有model_states这种上帝
视角真值）。（该文件里原来还有一个`broadcast_map_tf`，专供RViz可视化用的
另一条独立TF边，2026-08-12已整个删除，见DEBUG_JOURNAL.md——现在RViz的跨机
可视化也统一改成依赖这个节点背后的`world -> {ns}/map`链路，不再有单独实现。）

真机上能拿到的，只有每架飞机自己独立跑的localization（DLIO SLAM或UWB+IMU）
再加上`origin_setter_node`做的"起飞点一键锁定+在线SE(2)旋转估计"——这两样
东西合起来，已经通过TF把每架飞机的`{ns}/map`挂到了同一个共享父frame
`world`下面（`world -> {ns}/map`，见origin_setter_node.py）。一旦两架飞机
都完成了各自的起飞点锁定，`world`就是两棵TF子树的公共祖先，直接用tf2的
`lookup_transform(target={own_ns}/map, source={other_ns}/map)`，tf2会自动
帮你把`{own_ns}/map -> world -> {other_ns}/map`这条链路组合乘出来，语义
正好等价于`_compute_relative_transform`要的"{other_ns}机map原点在{own_ns}机
map系下的位置"——不需要自己手写任何矩阵乘法，也不需要关心背后到底是仿真
UWB还是真实UWB模块，这正是这个节点要做的事：把TF查询结果原样重新发布成
`/frame_align/{own_ns}/{other_ns}` (geometry_msgs/msg/TransformStamped)，
话题名/消息类型完全照抄仿真版的既有约定，下游mighty_node/ego_planner_swarm
（uncommitted WIP里的`swarm/use_frame_alignment`消费逻辑）不用改一行代码就
能把数据源从"仿真真值"切换成"真实标定结果"。

⚠️ 跟uwb_ground_truth_node.py的frame_align发布是同一个话题的两个潜在生产者，
不能同时开——见`docker/entrypoints/sim-world-entrypoint.sh`里
`publish_frame_align`参数：LOCALIZATION_SOURCE=uwb_slam场景下会关掉仿真端
那个发布者，改用这个节点的真实计算结果（因为情景二本来就是要验证"不依赖
仿真真值，光靠标定也能对齐"这件事，继续用仿真真值发布会让这个验证失去意义）；
其它模式(gt/dlio/uwb_imu)保持仿真端真值发布不变，不引入这个节点的行为变化，
避免影响已经跑通验证过的组合。

⚠️ 只发布"以自己为target"的那一路（`/frame_align/{own_ns}/{other_ns}`，不管
`/frame_align/{other_ns}/{own_ns}`），因为每架飞机的flight-stack容器只运行
自己命名空间下的一份mighty_node/ego_planner_swarm，只消费以自己为第一段
namespace的话题——跟`origin_setter_node`一样是完全去中心化的设计，每架飞机
只算自己需要的那一份，不假设有一个全局中心节点能看到所有飞机。

两机都还没完成起飞点锁定时，`{other_ns}/map`在TF树里还没接到`world`下面，
lookup_transform会抛LookupException——这是正常的启动瞬态，不是错误，捕获后
跳过这次发布、留到下个周期重试即可。
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FrameAlignBridgeNode(Node):

    def __init__(self):
        super().__init__('frame_align_bridge')

        own_ns = self.get_namespace().strip('/')
        self.own_ns = own_ns

        self.declare_parameter('num_agents', 2)
        self.declare_parameter('namespace_prefix', 'NX')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate_hz', 10.0)

        num_agents = self.get_parameter('num_agents').value
        ns_prefix = self.get_parameter('namespace_prefix').value
        self.world_frame = self.get_parameter('world_frame').value
        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        agent_names = [f'{ns_prefix}{i:02d}' for i in range(1, num_agents + 1)]
        self.other_ns_list = [n for n in agent_names if n != own_ns]

        self.own_map_frame = f'{own_ns}/map'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publishers_map = {}
        for other_ns in self.other_ns_list:
            topic = f'/frame_align/{own_ns}/{other_ns}'
            self.publishers_map[other_ns] = self.create_publisher(TransformStamped, topic, 10)

        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'frame_align_bridge up: own_ns={own_ns}, other_agents={self.other_ns_list}, '
            f'own_map_frame={self.own_map_frame}, world_frame={self.world_frame}')

    def _tick(self):
        for other_ns in self.other_ns_list:
            other_map_frame = f'{other_ns}/map'
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.own_map_frame, other_map_frame, Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'{other_map_frame} 尚未对齐到 {self.own_map_frame}（两机是否都已完成'
                    f'起飞点锁定？）: {e}', throttle_duration_sec=5.0)
                continue

            out = TransformStamped()
            out.header.stamp = tf.header.stamp
            out.header.frame_id = self.own_ns
            out.child_frame_id = other_ns
            out.transform = tf.transform
            self.publishers_map[other_ns].publish(out)


def main():
    rclpy.init()
    node = FrameAlignBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
