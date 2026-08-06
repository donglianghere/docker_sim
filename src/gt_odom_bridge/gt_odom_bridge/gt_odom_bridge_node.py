#!/usr/bin/env python3
"""
"跳过DLIO SLAM、直接用Gazebo仿真真值做定位"测试模式用的桥接节点。

背景：flight-stack-entrypoint.sh里 mighty 规划器（经 convert_odom_to_state，
remap odom->dlio/odom_node/odom）和 ros2_px4_stack 的 repub_odom.py（默认
odom_type=livox，默认订阅话题名 <namespace>/dlio/odom_node/odom）两条下游链路
都硬编码订阅同一个最终话题名 <namespace>/dlio/odom_node/odom——这是真正的
dlio_odom_node（namespace=<namespace>时）发布的话题。

这个节点订阅 sim-world 容器发布的 /plug/model_states_plug（gazebo_ros_state
插件，Gazebo世界里所有model的真值位姿+速度，跨容器走host网络+共享
ROS_DOMAIN_ID），从里面按 entity_name 找出自己这架飞机的真值，转成
nav_msgs/Odometry发布到跟真实DLIO完全同名的话题上——下游convert_odom_to_state/
repub_odom.py因此不用改一行代码，直接"以为"自己订阅的还是DLIO的输出。

这样可以单独验证 mighty 规划器 + ros2_px4_stack 板外控制器 + PX4 飞控这条链路，
排除DLIO SLAM本身收敛/漂移的影响；森林/房间场景本身仍然要生成真实点云（mighty的
lidar_cloud_in走的是Gazebo雷达插件的原始点云，跟定位源无关），点云本身有没有障碍物
不受这个节点影响，测的是"给定完美定位，规划+控制端到端能不能跑通"。

跟真正的 dlio_odom_node 二选一启动（flight-stack-entrypoint.sh 里
LOCALIZATION_SOURCE=gt 时用这个、=dlio 时用真的DLIO），不会同时跑——两个发布者
抢同一个话题，下游收到的数据来源会变得不确定。

不发布TF（<namespace>/odom -> <namespace>/base_link 等）——已经确认mighty的
点云建图（mighty_node.cpp的mapCallback/occupancyMapCallback/unknownMapCallback）
不查TF，只用state话题的位姿数值自己做变换；唯一会查map系TF的
getInitialPoseHwCallback只在use_hardware=true时由100ms定时器触发，是外部定位
硬件的初始位姿获取逻辑，跟这个GT测试模式的目标（验证规划+控制，不是验证TF树）
无关，先不补。如果后续发现某个下游确实需要这几条TF，再补。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class GtOdomBridgeNode(Node):

    def __init__(self):
        super().__init__('gt_odom_bridge_node')

        self.declare_parameter('entity_name', '')
        self.declare_parameter('model_states_topic', '/plug/model_states_plug')

        entity_name = self.get_parameter('entity_name').value
        if not entity_name:
            self.get_logger().error(
                "必须传 -p entity_name:=NX01 这样的参数（对应Gazebo里spawn_entity.py "
                "-entity 用的那个名字，不是ROS namespace）")
            sys.exit(1)
        self.entity_name = entity_name

        ns = self.get_namespace().strip('/')
        self.odom_frame = f'{ns}/odom' if ns else 'odom'
        self.baselink_frame = f'{ns}/base_link' if ns else 'base_link'

        self.pub = self.create_publisher(Odometry, 'dlio/odom_node/odom', 10)

        # model_states 是经典Gazebo classic的常见BEST_EFFORT高频话题
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        model_states_topic = self.get_parameter('model_states_topic').value
        self.sub = self.create_subscription(
            ModelStates, model_states_topic, self._model_states_cb, qos)

        self.get_logger().info(
            f"gt_odom_bridge up: entity_name={self.entity_name}, "
            f"model_states_topic={model_states_topic}, "
            f"publishing to {self.get_namespace()}/dlio/odom_node/odom")

    def _model_states_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.entity_name)
        except ValueError:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.baselink_frame
        odom.pose.pose = msg.pose[idx]
        odom.twist.twist = msg.twist[idx]
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GtOdomBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
