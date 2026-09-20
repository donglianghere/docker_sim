#!/usr/bin/env python3
"""
UWB 真值模拟节点。

角色（用户在设计阶段明确选定）：作为双机间坐标系对齐的绝对位置源，进入 mighty
的 use_frame_alignment 闭环——不是仅评估用的 ground truth。

数据来源：订阅 /plug/model_states_plug（gazebo_msgs/ModelStates，world文件里
gazebo_ros_state插件套了'/plug'namespace + remap之后的实际话题名，不是默认的
/gazebo/model_states——那个话题在这套world里没有任何发布者），每帧包含世界里
所有model的名字+位姿，按 namespace_prefix + 序号 找出各架飞机的真值位姿。

输出：
1. 对每一对(agent_i, agent_j)，发布 agent_j 相对 agent_i 的坐标变换到
   /frame_align/{ns_i}/{ns_j}   (geometry_msgs/msg/TransformStamped)
   这个话题命名规则和消息类型，是照抄 mighty_node.cpp 里 frame_align 订阅逻辑
   （frameAlignCallback）的期望格式，做到不用改mighty一行代码就能接上。
2. （2026-08-10新增）每架飞机自己的世界系绝对位置，发布到
   /{ns}/uwb/pose_abs   (geometry_msgs/msg/PoseStamped, frame_id="world")。
   这不是"双机互相测距"那类相对量，是"单机在统一任务坐标系下的绝对定位"——
   真实UWB系统里对应"锚点阵列解出tag的绝对坐标"这个功能，这套仿真里直接
   拿Gazebo世界真值代替，同样叠加range_noise_std高斯噪声模拟测距误差。
   这是`uwb_origin_bridge`包"起飞点一键锁定"功能的模拟数据源——真机上把
   这个话题的发布者换成真实UWB模块驱动即可，下游`uwb_origin_bridge`不用
   改一行代码。用`publish_absolute_pose:=false`可以关掉（比如你只想单独
   测原来的机间frame_align功能，不需要这部分）。

⚠️ 2026-09-03新增（SE(2)在线标定论文的补充实验用，见
`docker_sim/实验方案_SE2在线标定论文补充实验.md` L2层改动(a)）：
1. `/{ns}/uwb/pose_truth`——同一时刻**无噪声**的Gazebo真值位姿（含真实yaw），
   类型同样是geometry_msgs/PoseStamped。这是纯评估用的旁路，机上任何模块都
   不许订阅它（订了就是拿真值作弊）。特意发成PoseStamped而不是让离线评估
   工具自己去解`/plug/model_states_plug`，是因为宿主机上装的ROS没有
   gazebo_msgs这个包，解不了ModelStates那个类型，评估脚本根本跑不起来。
2. 观测污染注入参数：`abs_bias_xy`（恒定偏置，验证论文2.4节式(10)(11)的偏置
   免疫性）、`abs_bias_drift_mps`（偏置随时间线性增长，构造论文6.2节说的
   "偏置随时间变化则免疫性失效"那种场景）、`abs_outlier_prob`/
   `abs_outlier_mag_m`（多径/NLOS野值，验证论文2.7节的Huber抗差扩展）。
   默认全0，什么都不注入，行为跟以前完全一致。
3. `abs_pose_rate_hz`/`abs_pose_latency_ms`——限制绝对位置观测的发布率、给它
   加时延。**默认都是0，即保持原有行为**。
   ⚠️ 这里要说清楚一件之前没注意、写论文时必须交代的事实：
   `/{ns}/uwb/pose_abs`是在`model_states_cb`里逐帧发的，真实发布率等于world
   文件里`gazebo_ros_state`插件的`update_rate`（本项目各个world都是100 Hz
   仿真时间），**而且完全没有走延迟队列**；节点参数里那个`publish_rate_hz`
   =10/`latency_ms`=30从来只作用于`/frame_align/*`那一路，跟pose_abs无关。
   也就是说仿真里的绝对定位源比真实UWB乐观得多（100 Hz、零时延 vs 真实模块
   的10~50 Hz、几十毫秒时延）。论文报仿真精度时必须写明这一条，或者把这两个
   参数设成真实值重跑一遍做观测率/时延敏感性对照。

⚠️ 2026-09-13新增（对应A0清单/实现方案1.1a节，`EKF2_HGT_REF=3`高度语义修复
的第一步）：`/{ns}/uwb/pose_abs`的姿态分量，roll/pitch从"强制置0占位值"改成
直接透传`self.latest_poses[name].orientation`（Gazebo model_states真实姿态
四元数）里的roll/pitch分量，只有yaw继续保留独立加噪（`yaw_noise_std_deg`）。
**这不是"开天眼"**：真机上UWB+IMU/AHRS融合系统本来就能给出精度足够好的
roll/pitch（重力对齐，跟位置测距无关），仿真里直接用Gazebo姿态真值模拟
这一路传感器输出，跟"直接读地图坐标当自己位置"这种作弊行为性质不同——是
模拟一个真实存在、精度良好的传感器，理由类比本文件之前"range_noise_std
模拟UWB测距误差""publish_truth_pose是评估专用旁路"这几处已经在做的"传感器
能力可以直接建模、上帝视角信息不能写代码"的区分原则。改这个的直接动因：
`uwb_imu_fusion_node.py`原来靠订阅`mid360/imu`单独解一路roll/pitch，跟
`pose_abs`的yaw不是同一个消息来源、时间戳对不齐；这次改完之后`pose_abs`
自己就带完整6DoF姿态，`uwb_imu_fusion_node`可以直接从这一条消息里取
roll/pitch+yaw，不用再维护两路数据源的时间对齐问题（具体见该文件的
2026-09-13新增说明）。**再次强调**：这个改动只影响`pose_abs`，不动
`pose_truth`那条评估专用旁路——`pose_truth`本来就是无噪声全量真值
（含真实yaw），机上模块不许订阅它这条规则完全不受这次改动影响。

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
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry


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
        self.declare_parameter('publish_absolute_pose', True)
        # 2026-08-12新增：LOCALIZATION_SOURCE=uwb_slam场景专用开关。这个节点的
        # /frame_align/*是拿Gazebo真值算的，是"作弊"数据源；情景二要验证的正是
        # "不靠仿真真值、光靠origin_setter_node的标定结果也能做到跨机map对齐"，
        # 继续用这条真值发布会让frame_align_bridge_node发布的真实计算结果被真值
        # 覆盖/混在一起（两个发布者抢同一个话题），验证就失去意义了。sim-world-
        # entrypoint.sh在LOCALIZATION_SOURCE=uwb_slam时把这个参数设成false，
        # 其它模式(gt/dlio/uwb_imu)保持true不变，不影响已经验证过的mighty+
        # use_frame_alignment组合。
        self.declare_parameter('publish_frame_align', True)
        # 情景一(UWB+IMU替代SLAM)用——单独给yaw加噪声，跟位置noise_std是两个
        # 独立参数（真实UWB测angle-of-arrival/多锚点解算yaw的误差量级，
        # 跟测距误差没有必然联系，不能共用一个噪声源）。roll/pitch不在这里
        # 模拟，情景一里这两个分量固定由IMU提供，见uwb_imu_fusion_node.py。
        self.declare_parameter('yaw_noise_std_deg', 2.0)
        # 2026-09-03新增，见文件头说明。默认值全部是"什么都不做"，只有跑
        # 论文实验时才由docker-compose的环境变量显式打开。
        self.declare_parameter('publish_truth_pose', True)
        self.declare_parameter('abs_bias_xy', [0.0, 0.0])
        self.declare_parameter('abs_bias_drift_mps', [0.0, 0.0])
        self.declare_parameter('abs_outlier_prob', 0.0)
        self.declare_parameter('abs_outlier_mag_m', 0.0)
        self.declare_parameter('abs_pose_rate_hz', 0.0)      # 0=不限速
        self.declare_parameter('abs_pose_latency_ms', 0.0)   # 0=不延迟

        self.num_agents = self.get_parameter('num_agents').value
        self.ns_prefix = self.get_parameter('namespace_prefix').value
        self.noise_std = self.get_parameter('range_noise_std').value
        self.latency_s = self.get_parameter('latency_ms').value / 1000.0
        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.publish_absolute_pose = self.get_parameter('publish_absolute_pose').value
        self.publish_frame_align = self.get_parameter('publish_frame_align').value
        self.yaw_noise_std_rad = math.radians(self.get_parameter('yaw_noise_std_deg').value)
        self.publish_truth_pose = self.get_parameter('publish_truth_pose').value
        self.abs_bias_xy = list(self.get_parameter('abs_bias_xy').value)
        self.abs_bias_drift = list(self.get_parameter('abs_bias_drift_mps').value)
        self.abs_outlier_prob = float(self.get_parameter('abs_outlier_prob').value)
        self.abs_outlier_mag = float(self.get_parameter('abs_outlier_mag_m').value)
        self.abs_pose_rate_hz = float(self.get_parameter('abs_pose_rate_hz').value)
        self.abs_pose_latency_s = float(
            self.get_parameter('abs_pose_latency_ms').value) / 1000.0
        self._abs_start_sec = None       # 偏置漂移的时间基准，第一帧时定下来
        self._abs_last_pub_sec = {}      # name -> 上次发布的仿真时刻（限速用）
        self._abs_delay_queue = deque()  # (入队时刻, name, PoseStamped)

        self.agent_names = [f'{self.ns_prefix}{i:02d}' for i in range(1, self.num_agents + 1)]

        # 每架飞机一个绝对位置发布者，话题名用相对NAMESPACE的形式
        # （/{ns}/uwb/pose_abs），跟其它模块（mavros/dlio等）的命名习惯一致。
        self.abs_pose_publishers = {}
        if self.publish_absolute_pose:
            for name in self.agent_names:
                self.abs_pose_publishers[name] = self.create_publisher(
                    PoseStamped, f'/{name}/uwb/pose_abs', 10)

        # 真值旁路发布者（纯评估用，见文件头2026-09-03说明）
        self.truth_pose_publishers = {}
        if self.publish_truth_pose:
            for name in self.agent_names:
                self.truth_pose_publishers[name] = self.create_publisher(
                    PoseStamped, f'/{name}/uwb/pose_truth', 10)

        self.publishers_map = {}
        if self.publish_frame_align:
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

        # _compute_relative_transform算/frame_align/*需要知道每架飞机在"自己的
        # 地图"里的位置（DLIO局部odom，L_i/L_j），不能只有UWB相对测量这一路。
        # 2026-08-12：这里原来还带一份"RViz多机可视化"专用的broadcast_map_tf
        # （直接广播{i}/map -> {j}/map这条TF边），已整个删掉——它旋转部分写死
        # 单位四元数(单基线UWB给不出朝向)，本来就只是"两机spawn yaw都是0"这个
        # 简化假设下的权宜之计，两机各自设置非零spawn yaw之后这个假设已经不成立，
        # 这条边在所有LOCALIZATION_SOURCE模式下都会给出错的可视化结果。现在
        # 统一改成完全依赖`uwb_origin_bridge`包（控制栈，硬件可迁移）里
        # `origin_setter_node`广播的`world -> {ns}/map`静态TF——两机各自锁定后，
        # tf2会自动沿共享的`world`父frame把`{i}/map -> world -> {j}/map`这条
        # 链路组合出来给RViz用，不需要仿真这边再手动算一条替代边，也顺带避免了
        # 旧实现里"DLIO假epoch时钟域"那一整套时间戳适配（静态TF在tf2里对所有
        # 时间都有效，不需要跟动态TF对表）。代价：起飞前必须先给两机各自触发
        # 一次`set_origin_from_uwb`，RViz里两机地图才会摆到正确的相对位置——
        # 不再是容器一启动就自动有一条（哪怕是错的）TF可看。
        self.latest_local_odom = {}  # name -> geometry_msgs/Point (body在自己map系下的位置)
        for name in self.agent_names:
            self.create_subscription(
                Odometry, f'/{name}/dlio/odom_node/odom',
                lambda msg, n=name: self.local_odom_cb(msg, n), qos)

        self.timer = self.create_timer(period, self.flush_delayed)

        self.get_logger().info(
            f'UWB ground-truth sim up: agents={self.agent_names}, '
            f'noise_std={self.noise_std}m, latency={self.latency_s*1000:.0f}ms')

    def model_states_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name in self.agent_names:
                self.latest_poses[name] = pose

        now = self.get_clock().now()

        self._publish_abs_and_truth(now)
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
            # 对方）。修正量L_i-L_j。
            if i_name not in self.latest_local_odom or j_name not in self.latest_local_odom:
                continue
            t = self._compute_relative_transform(i_name, j_name, now)
            # 进延迟队列，到点再真正publish，模拟测距+解算耗时
            self.delay_queues[(i_name, j_name)].append((now, t))

    def _publish_abs_and_truth(self, now):
        """发布 /{ns}/uwb/pose_abs（带噪声/偏置/野值的观测）和
        /{ns}/uwb/pose_truth（无噪声真值，评估专用）。

        2026-09-03重构：这段原来直接内联在model_states_cb里，现在抽出来并
        加上真值旁路和三类观测污染注入，见文件头说明。污染顺序是
        真值 -> +高斯噪声 -> +恒定/漂移偏置 -> 按概率+野值，跟论文2.4/2.7节的
        观测模型一致（偏置加在**绝对位置**上，所以差分构造v_i时才会精确抵消，
        这正是论文式(10)要验证的性质）。
        """
        if not self.publish_absolute_pose and not self.publish_truth_pose:
            return
        now_sec = now.nanoseconds * 1e-9
        if self._abs_start_sec is None:
            self._abs_start_sec = now_sec
        elapsed = now_sec - self._abs_start_sec
        bias_x = self.abs_bias_xy[0] + self.abs_bias_drift[0] * elapsed
        bias_y = self.abs_bias_xy[1] + self.abs_bias_drift[1] * elapsed

        for name in self.agent_names:
            if name not in self.latest_poses:
                continue
            p = self.latest_poses[name].position
            q = self.latest_poses[name].orientation
            true_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            if self.publish_truth_pose:
                # 真值旁路：不限速、不延迟、不加任何噪声——它不是"观测"而是
                # 评估基准，加了任何东西就没法当基准用了
                tm = PoseStamped()
                tm.header.stamp = now.to_msg()
                tm.header.frame_id = 'world'
                tm.pose.position.x = p.x
                tm.pose.position.y = p.y
                tm.pose.position.z = p.z
                tm.pose.orientation.z = math.sin(true_yaw / 2.0)
                tm.pose.orientation.w = math.cos(true_yaw / 2.0)
                self.truth_pose_publishers[name].publish(tm)

            if not self.publish_absolute_pose:
                continue
            # 限速：0表示不限速，跟着model_states逐帧发（2026-09-03之前的
            # 行为，实际约100Hz仿真时间）
            if self.abs_pose_rate_hz > 0.0:
                last = self._abs_last_pub_sec.get(name)
                if last is not None and (now_sec - last) < 1.0 / self.abs_pose_rate_hz:
                    continue
                self._abs_last_pub_sec[name] = now_sec

            pose_msg = PoseStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = 'world'
            # 单基线/单锚点阵列UWB测距噪声的简化模拟：给三个平移轴各自
            # 独立加高斯噪声，跟frame_align那条相对量用的是同一个噪声
            # 模型/同一个range_noise_std参数，方便对照。
            x = p.x + random.gauss(0.0, self.noise_std) + bias_x
            y = p.y + random.gauss(0.0, self.noise_std) + bias_y
            z = p.z + random.gauss(0.0, self.noise_std)
            # 多径/NLOS野值：整帧观测被一个有限但显著的偏移污染，方向随机
            if self.abs_outlier_prob > 0.0 and random.random() < self.abs_outlier_prob:
                ang = random.uniform(-math.pi, math.pi)
                x += self.abs_outlier_mag * math.cos(ang)
                y += self.abs_outlier_mag * math.sin(ang)
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            # 2026-09-13改：roll/pitch不再强制置0，直接从q（Gazebo真值姿态
            # 四元数）解出、原样透传——模拟一个精度良好的姿态传感器（详见
            # 文件头2026-09-13新增说明），不是"开天眼"。只有yaw继续保留
            # 独立加噪（yaw_noise_std_deg），跟原来的处理方式一致——真实
            # UWB多锚点解算的yaw误差量级跟IMU重力对齐给出的roll/pitch误差
            # 量级没有必然联系，不能共用同一个噪声源，这条原有原则不变。
            # 提取公式（atan2/asin标准姿态角分解）跟uwb_imu_fusion_node.py
            # 的_imu_cb保持一致，方便对照。
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            true_roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            sinp = max(-1.0, min(1.0, sinp))  # 数值误差可能让sinp略超[-1,1]，asin会炸
            true_pitch = math.asin(sinp)

            noisy_yaw = true_yaw + random.gauss(0.0, self.yaw_noise_std_rad)
            # 用真实roll/pitch + 带噪yaw重新合成输出四元数（ZYX yaw-pitch-roll
            # 顺序，跟uwb_imu_fusion_node.py._uwb_cb组装vision_pose时用的
            # 合成公式同构，两处保持一致方便交叉核对）。
            cy, sy = math.cos(noisy_yaw * 0.5), math.sin(noisy_yaw * 0.5)
            cp, sp = math.cos(true_pitch * 0.5), math.sin(true_pitch * 0.5)
            cr, sr = math.cos(true_roll * 0.5), math.sin(true_roll * 0.5)
            pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
            pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy

            if self.abs_pose_latency_s > 0.0:
                self._abs_delay_queue.append((now, name, pose_msg))
            else:
                self.abs_pose_publishers[name].publish(pose_msg)

    def _compute_relative_transform(self, i_name, j_name, stamp) -> TransformStamped:
        pi = self.latest_poses[i_name].position
        pj = self.latest_poses[j_name].position
        li = self.latest_local_odom[i_name]
        lj = self.latest_local_odom[j_name]

        # D = 当前UWB测距解出的(j机身 - i机身)世界系位移；mighty这边要的是
        # "j机map原点在i机map系下的位置" = D + L_i - L_j。
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
        # 绝对位置观测的延迟队列（只有abs_pose_latency_ms>0时才有东西）。
        # ⚠️ 这个timer的周期是1/publish_rate_hz（默认0.1秒），比典型的几十
        # 毫秒时延粗，所以实际时延是"设定值向上取整到timer节拍"，做时延
        # 敏感性实验时要按这个粒度理解结果。
        while (self._abs_delay_queue
               and (now.nanoseconds - self._abs_delay_queue[0][0].nanoseconds) * 1e-9
               >= self.abs_pose_latency_s):
            _, name, msg = self._abs_delay_queue.popleft()
            self.abs_pose_publishers[name].publish(msg)
        for key, pub in self.publishers_map.items():
            q = self.delay_queues[key]
            while q and (now.nanoseconds - q[0][0].nanoseconds) * 1e-9 >= self.latency_s:
                _, t = q.popleft()
                pub.publish(t)

    def local_odom_cb(self, msg: Odometry, name: str):
        self.latest_local_odom[name] = msg.pose.pose.position


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
