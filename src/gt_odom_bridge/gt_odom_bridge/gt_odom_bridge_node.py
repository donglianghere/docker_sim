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

⚠️ 2026-08-10勘误：下面这段曾经写"不发布TF"，是错的——当时只验证过mighty自己的
mapCallback/occupancyMapCallback/unknownMapCallback不查TF，但mighty实际收到
障碍物数据靠的是global_mapper_ros这个独立节点（global_mapper_node.launch.py，
PLANNER=mighty分支才起），它的PointCloudCallback用tf_buffer_ptr_->lookupTransform
把原始点云配准到map系，这条TF链（map->odom->base_link->lidar->{ns}_livox）
真实DLIO模式下由DLIO自己发布odom->base_link/base_link->imu/base_link->lidar
三条动态TF撑着，gt模式下完全没有替代实现——LOCALIZATION_SOURCE=gt+PLANNER=mighty
这个组合此前很可能一直静默没有障碍物数据（没人实测碰到过，因为默认PLANNER是
mighty、但gt模式主要是拿来测ego_planner/纯控制链路，没人交叉测过这个具体组合）。
现在补上：动态广播"{ns}/odom -> {ns}/base_link"（数值直接用Gazebo真值位姿，
跟_model_states_cb发布的odom消息同一份数据，保证两者任何时刻都一致，不会出现
odom话题和TF对不上的情况），以及静态广播"{ns}/base_link -> {ns}/lidar"（固定
外参，数值抄自dlio.yaml的extrinsics/baselink2lidar，t=[0,0,0.06]、R=identity——
这是物理传感器安装位置，不随定位源变化，两边必须保持一致，如果以后改了mid360的
安装高度，这里也要跟着改）。有了这条链路，global_mapper_ros在gt模式下也能正常
工作；同时这也是新增的gt_cloud_bridge_node（另见同目录）在gt模式下给ego_planner
补点云时依赖的同一条TF链，两个节点共享，不用各自维护一份。

⚠️ 2026-08-10第二次勘误（用户实测报告"起飞前点云/占据栅格地图都看不到"，现场
查`docker logs`发现`gt_cloud_bridge_node`持续刷`lookup_transform`失败："Lookup
would require extrapolation into the past. Requested time 1735690168.xxx but
the earliest data is at time 1786366306.xxx"——这两个数字分别是"假epoch时间"
（`livox_imu_lidar_sim_time.patch`把Gazebo雷达/IMU插件的header.stamp强制改成
"仿真时间+固定偏移1735689600"）和"真实墙钟时间"（本条注释写下时的实际UTC时间
戳，差值対上"今天是2026年"）：下面`_model_states_cb`原来用`self.get_clock().
now()`给odom消息+"{ns}/odom -> {ns}/base_link"这条动态TF盖时间戳，这是真实
墙钟；而`gt_cloud_bridge_node`拿去做TF查询的`mid360_PointCloud2`点云
header.stamp是上面说的假epoch——两个时间域一个在"2025年+仿真流逝秒数"、一个在
"2026年真实墙钟"，TF buffer里永远只有"未来"（相对假epoch而言）的记录，
`lookup_transform`按点云的假epoch时间戳查找必然是"we要求的时间比buffer里最早
的记录还早"，永远失败——不是偶发抖动，是结构性、每一帧都会失败，
LOCALIZATION_SOURCE=gt+PLANNER=ego_planner这个组合此前从未被端到端测过，这个
bug一直没暴露过。

修复：不再用`self.get_clock().now()`真实墙钟给odom/TF盖时间戳，改成"借用"
`mid360/imu`（跟点云同一个假epoch时间域，来自同一个Gazebo传感器插件套件）最新
收到的一帧的`header.stamp`——两条链路（IMU→本节点的odom/TF，雷达点云→
gt_cloud_bridge_node的TF查询）从此共用同一个假epoch时钟基准，`lookup_transform`
才能查到落在同一个时间窗口内的记录。IMU本身的数值内容不参与任何计算，只是
借用它的时间戳，跟uwb_sim的broadcast_map_tf之前修的同一类"跨时钟域TF查询失败"
问题是同一个模式（那次是借用DLIO odom的header.stamp，这次因为没有DLIO、借用
IMU的）。
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# 跟dlio.yaml的extrinsics/baselink2lidar保持一致（patches/dlio_extrinsics_mount.patch
# 改出来的值）——固定的物理安装偏移，R=identity（0,0,0,1）。
BASELINK2LIDAR_T = (0.0, 0.0, 0.06)


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
        self.lidar_frame = f'{ns}/lidar' if ns else 'lidar'

        self.pub = self.create_publisher(Odometry, 'dlio/odom_node/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        static_tf_broadcaster = StaticTransformBroadcaster(self)
        baselink2lidar = TransformStamped()
        baselink2lidar.header.stamp = self.get_clock().now().to_msg()
        baselink2lidar.header.frame_id = self.baselink_frame
        baselink2lidar.child_frame_id = self.lidar_frame
        baselink2lidar.transform.translation.x = BASELINK2LIDAR_T[0]
        baselink2lidar.transform.translation.y = BASELINK2LIDAR_T[1]
        baselink2lidar.transform.translation.z = BASELINK2LIDAR_T[2]
        baselink2lidar.transform.rotation.w = 1.0
        static_tf_broadcaster.sendTransform(baselink2lidar)

        # model_states 是经典Gazebo classic的常见BEST_EFFORT高频话题
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        model_states_topic = self.get_parameter('model_states_topic').value
        self.sub = self.create_subscription(
            ModelStates, model_states_topic, self._model_states_cb, qos)

        # 只借用header.stamp（假epoch时间域，见文件头2026-08-10第二次勘误），
        # 不用消息内容——BEST_EFFORT对齐Gazebo IMU插件的发布约定。收到第一帧
        # 之前_sim_stamp是None，_model_states_cb会退回真实墙钟（此时TF还没
        # 有下游消费者在等，不影响正确性，等IMU第一帧一到就切过去）。
        self._sim_stamp = None
        # 2026-08-10第三次勘误（用户实测origin_setter算出的偏移接近零、
        # 追问"不应该是SLAM零点在全局坐标系下的偏移吗"，现场查`ros2 topic
        # echo`证实：这里发布的odom.pose直接是Gazebo世界系绝对真值
        # （NX01实测x=3.0，正好等于它自己的INIT_X世界系spawn偏移），不是
        # 真实DLIO那种"局部系原点=SLAM启动时所在位置=(0,0,0)"的相对坐标。
        # 项目里其它地方（status_monitor.py文件头注释、dual_goal_input.py
        # 的INIT_X换算）统一假设"dlio/odom_node/odom"是局部系，需要
        # +INIT_X才能换算成world系——gt模式这里一直没有遵守这个假设，是
        # 一个此前没人发现的不一致：mavros/local_position/pose不受影响
        # （PX4的EKF2自己在初始化时用第一帧vision估计值重新定零，不管
        # 上游vision话题的绝对数值是多少，所以dual_goal_input.py靠mavros
        # local_position读到的值本来就是正常的局部系，没有暴露这个问题），
        # 但直接订阅这个odom话题的消费者（这次新增的origin_setter、以及
        # status_monitor.py"局部坐标"那一列）都会看到绝对世界坐标而不是
        # 局部坐标，前者表现为"算出来的偏移总是接近UWB噪声量级、而不是
        # 真正的spawn平移量"，后者会在gt模式下把INIT_X重复加一次。
        # 用第一帧收到的绝对位置当"局部系原点"，后续每一帧都减掉这个值再
        # 发布——这样不管飞机spawn在世界系哪个位置，局部系都从(0,0,0)
        # 附近开始，跟真实DLIO行为一致，也是origin_setter这类"把局部系
        # 注册到全局系"功能真正有意义的前提（如果局部系本来就已经是
        # 全局系，注册这个动作永远是平凡的零偏移，测不出真实逻辑）。
        self._spawn_xyz = None
        self.imu_sub = self.create_subscription(
            Imu, 'mid360/imu', self._imu_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info(
            f"gt_odom_bridge up: entity_name={self.entity_name}, "
            f"model_states_topic={model_states_topic}, "
            f"publishing to {self.get_namespace()}/dlio/odom_node/odom, "
            f"broadcasting {self.odom_frame} -> {self.baselink_frame} -> {self.lidar_frame}")

    def _imu_cb(self, msg: Imu):
        self._sim_stamp = msg.header.stamp

    def _model_states_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.entity_name)
        except ValueError:
            return

        # 用IMU借来的假epoch时间戳，而不是self.get_clock().now()真实墙钟——
        # 否则gt_cloud_bridge_node拿假epoch时间戳的点云去查这条TF会永远
        # "extrapolation into the past"（两个时钟域一个在2025年+仿真流逝
        # 秒数、一个在真实2026年，永不相交），见文件头2026-08-10第二次勘误。
        stamp = self._sim_stamp if self._sim_stamp is not None else self.get_clock().now().to_msg()

        # 局部系原点=第一帧收到时飞机所在的世界系绝对位置（模拟"DLIO启动时
        # 所在位置就是局部系(0,0,0)"），只做平移，不碰姿态——跟项目里其它
        # 地方（uwb_sim的broadcast_map_tf等）一致，假设spawn时yaw≈0，
        # 没有额外的旋转需要移除。
        p = msg.pose[idx].position
        if self._spawn_xyz is None:
            self._spawn_xyz = (p.x, p.y, p.z)
        lx = p.x - self._spawn_xyz[0]
        ly = p.y - self._spawn_xyz[1]
        lz = p.z - self._spawn_xyz[2]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.baselink_frame
        odom.pose.pose = msg.pose[idx]
        odom.pose.pose.position.x = lx
        odom.pose.pose.position.y = ly
        odom.pose.pose.position.z = lz
        odom.twist.twist = msg.twist[idx]
        self.pub.publish(odom)

        # odom消息和这条TF用同一个(局部化后的)pose、同一个stamp——
        # global_mapper_ros/gt_cloud_bridge_node沿这条TF查到的位姿，跟
        # 下游直接订阅odom话题拿到的数值任何时刻都一致，不会出现两条链路
        # 各自算出不同答案的情况。
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.baselink_frame
        tf_msg.transform.translation.x = lx
        tf_msg.transform.translation.y = ly
        tf_msg.transform.translation.z = lz
        tf_msg.transform.rotation = msg.pose[idx].orientation
        self.tf_broadcaster.sendTransform(tf_msg)


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
