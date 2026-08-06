#!/usr/bin/env python3
"""
UWB 真值模拟节点。

角色（用户在设计阶段明确选定）：作为双机间坐标系对齐的绝对位置源，进入 mighty
的 use_frame_alignment 闭环——不是仅评估用的 ground truth。

数据来源：订阅 /plug/model_states_plug（gazebo_msgs/ModelStates，world文件里
gazebo_ros_state插件套了'/plug'namespace + remap之后的实际话题名，不是默认的
/gazebo/model_states——那个话题在这套world里没有任何发布者），每帧包含世界里
所有model的名字+位姿，按 namespace_prefix + 序号 找出各架飞机的真值位姿。

输出：对每一对(agent_i, agent_j)，发布 agent_j 相对 agent_i 的坐标变换到
  /frame_align/{ns_i}/{ns_j}   (geometry_msgs/msg/TransformStamped)
这个话题命名规则和消息类型，是照抄 mighty_node.cpp 里 frame_align 订阅逻辑
（frameAlignCallback）的期望格式，做到不用改mighty一行代码就能接上。

噪声/时延：range_noise_std 给位置加高斯噪声（模拟UWB测距误差，默认0.05m只是
占位量级，不是任何实测UWB模块的真实指标——真实UWB模块（如DecaWave/Nooploop）
的实测误差需要你自己去查具体型号的数据手册再定）；latency_ms 用一个简单的
环形缓冲区延迟发布，模拟测距+解算耗时。

⚠️ 未验证项：
双机UWB测距天然只能给出"距离"，这里为了简化直接用Gazebo真值的完整6DOF相对
位姿(而不是先算出单个"距离"标量、再反过来估计位姿)，这是"理想化"的简化，等价于
假设你有多锚点阵列能解出完整位姿，不是单基线UWB测距的真实建模——如果你需要更
真实的单基线测距噪声模型，需要在这基础上把"直接读四元数"这部分换成"多次测距
三角化"的过程，目前没有实现这一层。
"""

import math
import random
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class UwbGroundTruthNode(Node):

    def __init__(self):
        super().__init__('uwb_ground_truth_node')

        self.declare_parameter('num_agents', 2)
        self.declare_parameter('namespace_prefix', 'NX')
        self.declare_parameter('range_noise_std', 0.05)
        self.declare_parameter('latency_ms', 30.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        # 实际发布的话题是 /plug/model_states_plug，不是默认的 /gazebo/model_states
        # ——base_mighty.launch.py起的world里gazebo_ros_state插件套了'/plug'
        # namespace、把model_states remap成了model_states_plug（跟另一个默认也叫
        # model_states的话题分开，避免混淆）。这里原来订阅的是没人发布的
        # /gazebo/model_states，`ros2 topic info -v`实测确认：0个发布者，
        # 这个节点自己是唯一的订阅者——本节点自成立以来就没真正收到过一帧
        # model_states，下游/frame_align/*话题也就从来没真正发布过任何数据
        # （`ros2 topic hz`实测超时确认），多机坐标系对齐功能一直是空转的。
        self.declare_parameter('model_states_topic', '/plug/model_states_plug')

        self.num_agents = self.get_parameter('num_agents').value
        self.ns_prefix = self.get_parameter('namespace_prefix').value
        self.noise_std = self.get_parameter('range_noise_std').value
        self.latency_s = self.get_parameter('latency_ms').value / 1000.0
        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        self.agent_names = [f'{self.ns_prefix}{i:02d}' for i in range(1, self.num_agents + 1)]

        self.publishers_map = {}
        for i_name in self.agent_names:
            for j_name in self.agent_names:
                if i_name == j_name:
                    continue
                topic = f'/frame_align/{i_name}/{j_name}'
                self.publishers_map[(i_name, j_name)] = self.create_publisher(
                    TransformStamped, topic, 10)

        # model_states 是经典Gazebo classic的常见BEST_EFFORT高频话题
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        model_states_topic = self.get_parameter('model_states_topic').value
        self.sub = self.create_subscription(
            ModelStates, model_states_topic, self.model_states_cb, qos)

        # 每个 (i,j) 一个延迟队列，元素是 (发布时刻, TransformStamped)
        self.delay_queues = {k: deque() for k in self.publishers_map}
        self.latest_poses = {}  # name -> geometry_msgs/Pose

        # RViz多机可视化用：把frame_align广播成一条{i}/map -> {j}/map的TF，
        # 让Fixed Frame挂在其中一架飞机的map/odom下时，另一架飞机的点云/
        # 轨迹/TF树也能正确摆到相对位置上——这跟上面/frame_align/*话题
        # 喂给mighty避让用的东西是两个不同的消费者，数学上也不是同一个量
        # （见下面broadcast_map_tf的注释），所以单独实现，不复用
        # _compute_relative_transform的结果。
        # 需要额外知道每架飞机在"自己的地图"里的位置（DLIO局部odom），
        # 不能只有下面这条UWB相对测量。
        self.latest_local_odom = {}  # name -> geometry_msgs/Point (body在自己map系下的位置)
        self.latest_local_odom_stamp = {}  # name -> builtin_interfaces/Time (见broadcast_map_tf的时钟域注释)
        for name in self.agent_names:
            self.create_subscription(
                Odometry, f'/{name}/dlio/odom_node/odom',
                lambda msg, n=name: self.local_odom_cb(msg, n), qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        # 只广播每对(i,j)一次（i在agent_names里排在j前面），避免同一条边
        # 两个方向都发、互相打架。
        self.map_tf_pairs = [
            (a, b) for idx, a in enumerate(self.agent_names)
            for b in self.agent_names[idx + 1:]
        ]

        self.timer = self.create_timer(period, self.flush_delayed)

        self.get_logger().info(
            f'UWB ground-truth sim up: agents={self.agent_names}, '
            f'noise_std={self.noise_std}m, latency={self.latency_s*1000:.0f}ms')

    def model_states_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name in self.agent_names:
                self.latest_poses[name] = pose

        now = self.get_clock().now()
        for (i_name, j_name), pub in self.publishers_map.items():
            if i_name not in self.latest_poses or j_name not in self.latest_poses:
                continue
            # mighty的applyFrameAlignTransform把这个transform直接乘到j机
            # 自己local map系下的轨迹点上（frameAlignCallback），需要的是
            # "j机map原点在i机map系下的位置"，不是"此刻j机身相对i机身的
            # UWB测距量"——这两个量只有在两架飞机都恰好站在自己map原点时
            # 才相等，飞起来之后差的正是各自当前离自己map原点的位移，量级
            # 跟飞行范围一样大（本例中曾经现场量到>1米的偏差，直接导致mighty
            # 把另一架飞机定位到偏差好几米外的错误位置，避障因此完全失效——
            # 两机各自绕着自己错认的"对方位置"规划，实际上从没真正避开过
            # 对方）。修正量L_i-L_j下面这条TF广播（broadcast_map_tf，给
            # RViz可视化用）已经在算了，这里之前一直没跟上，属于遗漏。
            if i_name not in self.latest_local_odom or j_name not in self.latest_local_odom:
                continue
            t = self._compute_relative_transform(i_name, j_name, now)
            # 进延迟队列，到点再真正publish，模拟测距+解算耗时
            self.delay_queues[(i_name, j_name)].append((now, t))

    def _compute_relative_transform(self, i_name, j_name, stamp) -> TransformStamped:
        pi = self.latest_poses[i_name].position
        pj = self.latest_poses[j_name].position
        li = self.latest_local_odom[i_name]
        lj = self.latest_local_odom[j_name]

        # D = 当前UWB测距解出的(j机身 - i机身)世界系位移；mighty这边要的是
        # "j机map原点在i机map系下的位置" = D + L_i - L_j（跟broadcast_map_tf
        # 用的是同一个公式，见那边注释里的推导）。
        dx = (pj.x - pi.x) + li.x - lj.x + random.gauss(0.0, self.noise_std)
        dy = (pj.y - pi.y) + li.y - lj.y + random.gauss(0.0, self.noise_std)
        dz = (pj.z - pi.z) + li.z - lj.z + random.gauss(0.0, self.noise_std)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = i_name
        t.child_frame_id = j_name
        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = dz
        # 单基线UWB测距本身给不出朝向，这里简化为单位四元数（不做相对姿态估计）。
        # 如果 mighty 侧的 applyFrameAlignTransform 需要完整6DOF旋转才能正确工作，
        # 这一处简化需要你根据实际效果决定要不要补一个姿态估计环节。
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        return t

    def flush_delayed(self):
        now = self.get_clock().now()
        for key, pub in self.publishers_map.items():
            q = self.delay_queues[key]
            while q and (now.nanoseconds - q[0][0].nanoseconds) * 1e-9 >= self.latency_s:
                _, t = q.popleft()
                pub.publish(t)
        self.broadcast_map_tf()

    def local_odom_cb(self, msg: Odometry, name: str):
        self.latest_local_odom[name] = msg.pose.pose.position
        self.latest_local_odom_stamp[name] = msg.header.stamp

    def broadcast_map_tf(self):
        """把{i}/map -> {j}/map广播成TF，纯粹服务于RViz多机可视化。

        跟_compute_relative_transform发布的/frame_align/*不是同一个量：那个是
        "此刻i机身相对j机身的UWB测距量"，随两架飞机飞行实时变化，直接原样
        应用到共享轨迹上（frameAlignCallback/applyTransformToTraj）本身没问题，
        因为那是mighty每次收到新轨迹时都重新做一次的即时变换。但TF不一样——
        TF里的map frame被定义成"飞机自己DLIO SLAM的固定原点"，{i}/map和
        {j}/map之间的相对位置理论上应该是个常量（两架飞机的SLAM原点各自不动），
        如果直接把"两机身当前相对位置"这种随时间变化的量当成map-to-map的TF
        广播出去，会变成"NX02整棵地图跟着NX02机身一起满屋子飘"，不对。

        正确做法：用每架飞机自己的DLIO里程计（body在自己map系下的位置，L_i/L_j）
        把UWB测出来的"机身对机身"世界系位移（D）换算成"map原点对map原点"的
        位移。假设两架飞机spawn时yaw都是0、DLIO重力对齐后也接近0（这套仿真
        目前如此），世界系/两机map系三者只差平移不差旋转，位移可以直接矢量
        相加减：
            P(map_j原点 在 map_i系下) = D + L_i - L_j
        其中 D = 当前UWB测得的(j机身 - i机身)世界系位移，L_i/L_j分别是i/j
        机身在自己map系下的位置（DLIO里程计直接给的）。旋转部分沿用现有
        frame_align的简化（单基线UWB给不出朝向），仍用单位四元数。

        时间戳踩过的坑：这条TF不能盖`self.get_clock().now()`（真实墙钟，
        ~2026年那个epoch）。DLIO/mighty那一侧全部跑在Gazebo仿真时钟上，
        `mighty_imu_sim_time.patch`/`livox_imu_lidar_sim_time.patch`把
        IMU/点云的header.stamp都换成了"sim时间 + 固定偏移1735689600
        (2025-01-01T00:00:00Z)"这个假epoch（原因见那两个patch的注释：
        规避Gazebo真实时间因子漂移+DLIO点云类型探测的1e14ns阈值)，DLIO自己
        发布的`{ns}/odom -> {ns}/base_link`继承的正是这个假epoch。如果这条
        `{i}/map -> {j}/map`盖真墙钟时间，就会跟DLIO那条动态TF差出真实时间
        `2026-xx`减假epoch`2025-01-01`的巨大间隔（实测量到接近5000万秒），
        tf2的多跳lookup要求链路上所有边有共同有效时间窗，跨两个相差一年半的
        clock域必然找不到公共时间，报"Lookup would require extrapolation
        into the past"——单独看`NX01/map->NX02/map`或单独看
        `NX02/odom->NX02/base_link`各自都能解出来（各自都在自己的clock域内
        自洽），但拼起来查`NX01/map->NX02/base_link`这种真正需要展示NX02
        点云/位姿的完整链路必然失败，RViz里表现就是"Fixed Frame设对了，
        NX02的东西还是不显示"。改用两机DLIO里程计各自最新的header.stamp
        （取较新的一个）作为这条TF的时间戳，让它落在跟DLIO/mighty同一个假
        epoch时钟域里，才能被tf2正常组合进多跳链路。
        """
        for i_name, j_name in self.map_tf_pairs:
            if i_name not in self.latest_poses or j_name not in self.latest_poses:
                continue
            if i_name not in self.latest_local_odom or j_name not in self.latest_local_odom:
                continue
            pi = self.latest_poses[i_name].position
            pj = self.latest_poses[j_name].position
            li = self.latest_local_odom[i_name]
            lj = self.latest_local_odom[j_name]

            stamp_i = self.latest_local_odom_stamp[i_name]
            stamp_j = self.latest_local_odom_stamp[j_name]
            dlio_stamp = stamp_i if (stamp_i.sec, stamp_i.nanosec) >= (stamp_j.sec, stamp_j.nanosec) else stamp_j

            t = TransformStamped()
            t.header.stamp = dlio_stamp
            t.header.frame_id = f'{i_name}/map'
            t.child_frame_id = f'{j_name}/map'
            t.transform.translation.x = (pj.x - pi.x) + li.x - lj.x
            t.transform.translation.y = (pj.y - pi.y) + li.y - lj.y
            t.transform.translation.z = (pj.z - pi.z) + li.z - lj.z
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = UwbGroundTruthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
