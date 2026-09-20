#!/usr/bin/env python3
"""情景一(UWB+IMU替代SLAM)的"读回+改名重发"桥接节点——docker_sim
`多机空间坐标对齐问题_设计讨论纪要.docx`第8.1节方案的实现，跟`gt_odom_bridge_node`
是同一种"接口适配"角色，只是数据源从Gazebo真值换成PX4自己的定位输出。

背景：规划器(ego_planner/mighty)、点云建图(global_mapper_ros/gt_cloud_bridge_node)、
TF树全都只认`dlio/odom_node/odom`这一个固定话题名+`{ns}/odom -> {ns}/base_link`
这一条固定TF，不关心背后到底是DLIO真跑SLAM、Gazebo给真值、还是这里的UWB+IMU
拼出来的——`LOCALIZATION_SOURCE=gt`模式下`gt_odom_bridge_node`订阅Gazebo真值
做这件事，`LOCALIZATION_SOURCE=uwb_imu`模式下这个节点订阅PX4自己算出来的
`mavros/local_position/odom`做同一件事。

为什么要绕一圈经过PX4（历史设计，2026-08-13起`LOCALIZATION_SOURCE=uwb_imu`
默认不再这样接）：`uwb_imu_fusion_node`把UWB+IMU拼好的位姿发给PX4之后，
PX4内部EKF2会结合自己的IMU高频传播模型再融合一次，得到的`local_position/odom`
比UWB+IMU的原始拼装结果更平滑、更高频（这是原始设计的假设——IMU传播速率
远高于UWB测距速率）——更重要的是，飞控自己的姿态/位置控制环(px4ctrl/
so3ctrl)用的就是这份EKF2内部状态，绕这一圈能保证"规划器看到的世界模型"和
"飞控真正在飞的状态"是同一份估计的两个观察窗口，不是两条独立、会互相跑偏
的账。

2026-08-13实测推翻了"更高频"这个假设：PX4 SITL经Gazebo classic的
`gazebo_mavlink_interface`插件锁步，硬编码"总是250Hz仿真时间"喂传感器数据
给PX4（`gazebo_mavlink_interface.cpp`），乘上这台机器实测RTF≈0.44，
`local_position/odom`真实只有~47Hz，比直接用UWB（提速后~145Hz）还慢——
"绕一圈经过PX4更快"这个前提不成立了。改法（"路线A"，经过两次迭代）：
`flight-stack-entrypoint.sh`里`uwb_imu`/`single_uwb_imu`分支现在把这个节点的
`local_position_topic`参数指向`uwb_imu_fusion_node`发布的`uwb_imu_odom`
（~145Hz，位置+速度互补滤波，见uwb_imu_fusion_node.py文件头——中间试过换成
`robot_localization`包的`ekf_node`，排查一整圈发现它的`pose0_config`/
`imu0_config`这类参数在这套环境里死活不被节点正确识别，放弃换回自己的
互补滤波），不再默认指向`mavros/local_position/odom`。飞控那一路
(`vision_pose/pose_cov`)完全不受影响，仍然走PX4自己的EKF2——"规划器/控制器
看到的世界模型"和"飞控真正在飞的状态"从此是两条独立算出来的账（都来自
`uwb_imu_fusion_node`同一次UWB+IMU观测，但传播/滤波逻辑不再共享，飞控内部
EKF2还会再做一次自己的传播），上面这段"绕一圈"的历史顾虑理论上重新出现，
需要实测验证pt4ctrl会不会因为这个不同步而震荡。这个节点本身完全没变，
只是被指去读了一个不同的话题名，其余逻辑（TF广播、局部系原点处理）照旧。

⚠️ 局部系原点问题，走过的弯路（2026-08-11实测发现）：最初以为要在这个节点
里"用第一帧收到的绝对位置当局部系原点"（照抄gt_odom_bridge_node的套路），
但试了两版都不对——第一版直接用`mavros/local_position/odom`自己的第一帧当
原点：Gazebo的model_states（gt_odom_bridge_node的数据源）从第一帧起就是
真值，没有收敛过程，但PX4的EKF2不是，刚起来时还没吃到vision数据，会先用
纯IMU从(0,0,0)推算，直到vision融合真正生效、触发一次位置重置后才会跳到
接近UWB给的绝对坐标——本节点的订阅大概率抢在这次重置之前就收到了第一帧，
把这个还没收敛的瞬态值(≈0)当成了原点，等于没做归零，`dlio/odom_node/odom`
变成了绝对坐标的复制品。第二版改成用`uwb/pose_abs`的第一帧当原点（UWB本身
没有收敛过程）——这一版数学上是对的，但真正的根源其实更上游：
`uwb_imu_fusion_node`喂给PX4的位置本来就是UWB的世界系绝对坐标，没有归零，
导致PX4整个内部状态（不只是这个节点读的`local_position/odom`，
`dual_goal_input.py`直接读的`local_position/pose`同样受影响）都是绝对坐标
——真正该在"喂给PX4之前"就归零，不是"读回来之后"才归零，否则`dual_goal_
input.py`这类直接读`local_position/pose`（不经过这个节点）的消费者永远
拿到的还是没归零的绝对值。

修复落在了`uwb_imu_fusion_node.py`里（用第一帧UWB位置当局部系原点，喂给
PX4之前先减掉）——PX4内部状态从此整体上就是"局部系原点=spawn位置=(0,0,0)"，
跟`dlio`/`gt`两种模式完全同构。这个节点因此**不需要再做任何归零**，恢复成
单纯的"读回来、改名重发"，跟它的名字（readback）实际该做的事一致。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# 跟dlio.yaml的extrinsics/baselink2lidar、gt_odom_bridge_node保持一致——固定的
# 物理安装偏移，R=identity(0,0,0,1)，不随定位源变化。
BASELINK2LIDAR_T = (0.0, 0.0, 0.06)


class LocalPositionReadbackNode(Node):

    def __init__(self):
        super().__init__('local_position_readback_node')

        ns = self.get_namespace().strip('/')
        self.odom_frame = f'{ns}/odom' if ns else 'odom'
        self.baselink_frame = f'{ns}/base_link' if ns else 'base_link'
        self.lidar_frame = f'{ns}/lidar' if ns else 'lidar'

        self.declare_parameter('local_position_topic', 'mavros/local_position/odom')

        # 2026-08-11实测踩坑：mavros/local_position/odom发布用的是BEST_EFFORT
        # （`ros2 topic info -v`确认：Reliability=BEST_EFFORT, History=
        # KEEP_LAST(5)），这里最初用了create_subscription默认的RELIABLE QoS，
        # 两者不兼容——ROS2的QoS兼容规则下RELIABLE订阅者收不到BEST_EFFORT
        # 发布者的任何消息（日志刷"offering incompatible QoS. No messages
        # will be received from it."），这个节点因此从来没真正收到过一帧
        # local_position/odom，dlio/odom_node/odom和对应TF从此没有发布过，
        # 连带导致RViz空白、status_monitor局部/全局坐标"等待数据"、
        # uwb_ground_truth_node的frame_align("UWB双机坐标系对齐")因为拿不到
        # latest_local_odom也一直"掉线"——三个症状是同一个根因。

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

        # 借用mid360/imu的header.stamp（假epoch时间域，跟gt_cloud_bridge_node
        # 查这条TF用的点云时间戳同一个域）——原理、修法都跟gt_odom_bridge_node
        # 2026-08-10第二次勘误完全一样：mavros/local_position/odom自己的
        # header.stamp是mavros对PX4时钟的同步结果，不在Gazebo传感器插件的
        # 假epoch域里，如果直接拿来给TF盖戳，gt_cloud_bridge_node用点云假
        # epoch时间戳去查这条TF会永远"extrapolation into the past"，是结构性
        # 失败，不是偶发抖动。
        self._sim_stamp = None
        self.imu_sub = self.create_subscription(
            Imu, 'mid360/imu', self._imu_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.create_subscription(
            Odometry, self.get_parameter('local_position_topic').value, self._odom_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info(
            f"local_position_readback_node就绪：订阅"
            f"{self.get_parameter('local_position_topic').value}，"
            f"发布到{self.get_namespace()}/dlio/odom_node/odom，"
            f"广播{self.odom_frame} -> {self.baselink_frame} -> {self.lidar_frame}")

    def _imu_cb(self, msg: Imu):
        self._sim_stamp = msg.header.stamp

    def _odom_cb(self, msg: Odometry):
        # 纯透传，不做任何归零——local_position/odom进这个节点之前，
        # PX4喂进去的位置(uwb_imu_fusion_node)已经是spawn归零后的局部坐标，
        # 见文件头说明。
        stamp = self._sim_stamp if self._sim_stamp is not None else self.get_clock().now().to_msg()

        p = msg.pose.pose.position

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.baselink_frame
        odom.pose.pose.position.x = p.x
        odom.pose.pose.position.y = p.y
        odom.pose.pose.position.z = p.z
        odom.pose.pose.orientation = msg.pose.pose.orientation
        odom.twist.twist = msg.twist.twist
        self.pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.baselink_frame
        tf_msg.transform.translation.x = p.x
        tf_msg.transform.translation.y = p.y
        tf_msg.transform.translation.z = p.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPositionReadbackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
