#!/usr/bin/env python3
"""情景一(UWB+IMU替代SLAM)的融合节点——docker_sim`多机空间坐标对齐问题_设计
讨论纪要.docx`第8.1节方案的实现。

背景：情景一处理的是"SLAM完全失效，只能靠UWB+IMU飞"这种退化场景。跟情景二
（DLIO正常工作、只是局部系yaw跟全局系有未知旋转偏移）不同，情景一里UWB给的
位置+yaw、IMU给的roll/pitch，全都已经是全局系下的量——UWB锚点阵列本来就是
按统一任务坐标系标定的，IMU的roll/pitch靠重力对齐、天然跟全局系一致（重力
方向不随飞机偏航角变化）。这意味着情景一**不存在**情景二讨论了很久的"局部系
->全局系旋转求解"问题：这里只是把两路已经在同一个坐标系下的数据原样拼装成
一个完整6-DoF位姿，不需要任何旋转估计。

数据流：
  /{ns}/uwb/pose_abs (geometry_msgs/PoseStamped，uwb_ground_truth_node发布)
      取position(x,y,z) + orientation里的yaw分量(roll/pitch在源头就是0，
      是仿真对"UWB测不出姿态"这个物理限制的模拟，不能拿来用)
  /{ns}/mid360/imu (sensor_msgs/Imu，Gazebo IMU插件发布)
      取roll/pitch分量(重力对齐)，yaw分量不用(IMU的yaw靠积分角速度得来，
      长时间会漂移，UWB的yaw更可信，两者对同一个物理量有分歧时，情景一
      的设计就是"各管一段"而不是加权融合——没有重叠、没有冲突需要调和)
      -> /{ns}/mavros/vision_pose/pose_cov (PoseWithCovarianceStamped)
      跟repub_odom.py发去PX4的是同一个目标话题，PX4侧EKF2_EV_CTRL的融合
      配置不需要跟着LOCALIZATION_SOURCE切换而改变。

这个节点本身只做"组装"，不做概率意义上的"融合"（没有卡尔曼滤波/加权，因为
UWB和IMU各自贡献互不重叠的分量）。真正的多源时序融合发生在PX4内部的EKF2，
这个节点发布出去之后就跟ROS2/DDS域没有关系了。

⚠️ 2026-09-13新增（对应A0清单/实现方案1.1a节，跟`uwb_ground_truth_node.py`
同批改动，两个文件必须一起看）：这次修的是"z这一路的观测源"，动因是查证
本机flight-stack的PX4航空框架patch后确认——这套仿真固定配置
`EKF2_EV_CTRL=15`+`EKF2_HGT_REF=3`，PX4的高度主参考源直接就是这个节点发
的`vision_pose/pose_cov`里的z，不是气压计，这条配置不分`LOCALIZATION_
SOURCE`、全局生效。也就是说`uwb_imu`模式下PX4内部`local_position.z`到底
是"稳定世界系Z"还是"离地高度AGL"，完全取决于这个节点塞进去的z是什么——
不是一个天然独立存在的物理量。真机上这套定位方案的z本来就是"AHRS姿态+
测距雷达range合成"（不是"UWB测的z"），之前这个节点直接把`uwb/pose_abs`
的z分量（Gazebo真值+高斯噪声，模拟UWB绝对定位）当z喂给PX4，没有对齐真机
实现，需要修。修完之后`uwb_imu`模式下PX4的`local_position.z`天然就是
AGL——不需要新增任何"agl_hold"模式开关，全程默认生效（原方案在
`position_cmd_relay_node`加`agl_hold`模式的设计已作废，见实现方案1.1节
"v3-g订正"）。

**为什么z的观测源要跟x/y分开处理**（这是这次改动的核心结构性变化，不是
细枝末节）：x/y的物理意义是"世界系(局部化后)水平位置"，UWB锚点阵列本来
就是按这个物理量标定的，直接用没有问题；但z一旦换成"AGL"这个物理量，
UWB测的绝对高度（`pose_abs`的z）就完全不对——那是"离世界系原点的高度"，
跟"离正下方地面的高度"是两个不同的量，飞过仿地模块（地形起伏）时前者
不变、后者会变，继续用前者会让"全程AGL定高"这个目标从根上失败。能测AGL
的传感器是测距雷达（下视，沿机体轴向下），不是UWB，所以z必须换一个完全
不同的观测源（`mavros/hrlv_ez4_pub`，`sensor_msgs/Range`），且这个传感器
测的是"斜距"不是"垂直距离"，飞机有倾角时需要额外做水平倾斜补偿
（`compensated_height = range * cos(roll) * cos(pitch)`）——x/y没有类似
的几何换算需求。另外测距雷达贴地/起降瞬间会跌出量程（本方案`range_min_
valid_m`留量0.25m，参考原`agl_hold`设计的0.19-0.2m最小量程），这个"读数
可能失效"的边界问题x/y完全不存在（UWB绝对定位没有量程下限这个概念）。
两路观测源的物理意义、误差来源、失效模式三方面都不同，继续用同一个
`for i in range(3)`循环共享逐分量处理逻辑会把这些本质不同的问题混在
一起看不清楚，所以这次把循环拆开，x/y保持原样、z单独处理。

roll/pitch的来源也顺带换了：从订阅`mid360/imu`（Gazebo IMU传感器插件）
改成直接从`uwb/pose_abs`的姿态分量取——`uwb_ground_truth_node.py`这次
改完之后`pose_abs`自带Gazebo真值透传的roll/pitch（不再强制置0，见该文件
2026-09-13新增说明），跟这里同一条消息里的yaw是同一个时间戳、同一个数据
源，不用再像以前那样从`mid360/imu`单独解一路、承受两路数据（`pose_abs`
的yaw + `mid360/imu`的roll/pitch）时间戳不对齐的风险。`mid360/imu`话题
不整个丢弃——线加速度/角速度部分（`_imu_cb`里的位置/速度积分传播）仍然
要用，只是不再从它的`orientation`字段解姿态角。

2026-08-13：`dlio/odom_node/odom`高频化（"路线A"，详见DEBUG_JOURNAL.md同日
系列条目）——PX4经Gazebo classic锁步链路(`gazebo_mavlink_interface`硬编码
250Hz仿真时间)把`mavros/local_position/odom`摁在~47Hz，规划器/控制器需要
更快的反馈。三次迭代：
  1) 第一版手搓"IMU积分传播、UWB周期修正"互补滤波，只做速度、位置直接用
     UWB原始值——实测太抖（UWB本身σ=0.05m测距噪声，原来靠PX4内部EKF2抹平，
     绕开PX4之后没人滤这一步）。
  2) 第二版给位置也加了互补滤波（时间常数0.15秒，比速度的0.5秒短很多）——
     用户反馈"还是太抖"，同时要求换开源方案。
  3) 换成`robot_localization`包的`ekf_node`——排查一整圈（话题名漏了
     `mavros/`前缀、怀疑QoS不兼容、怀疑YAML数组写法有问题）之后，用
     `ros2 service call .../list_parameters`直接查询运行中节点，确认
     `pose0_config`/`imu0_config`这些数组参数无论怎么改YAML写法都没有被
     节点声明/识别——这个apt包(ros-humble-robot-localization 3.5.4)在这套
     环境里的实际参数加载行为跟官方文档/示例对不上，具体是这个版本本身的
     问题还是这套环境（CycloneDDS+多层docker exec诊断工具本身也反复表现出
     不可靠，见DEBUG_JOURNAL.md同日排查记录）的什么特殊性导致，没有查清楚，
     放弃这条路。
  4) 回到手搓互补滤波，但这次位置的时间常数也大幅调大（跟速度用同一个默认
     值），做更强的平滑——见下面`_p_state`/`_v_state`两组互补滤波状态。

⚠️ 2026-08-11实测发现、补的坑：`dlio`/`gt`两种模式喂给PX4的位置本来就是
"局部系原点=启动/spawn时所在位置=(0,0,0)"的相对坐标（DLIO自己天然如此，
`gt_odom_bridge_node`则是显式减掉第一帧），这套约定被`dual_goal_input.py`/
`status_monitor.py`当成项目级前提——它们直接读`mavros/local_position/pose`
当"局部坐标"，再加`INIT_X`换算成世界坐标显示/反算目标点。这个节点最初的
实现直接把UWB的世界系绝对坐标（比如NX01≈x=3.0）原样喂给PX4，导致PX4整个
内部状态（`local_position/pose`、`local_position/odom`）都变成了绝对坐标，
`dual_goal_input.py`的"局部坐标+INIT_X"换算因此把spawn偏移重复加了一次——
不只是显示错，如果这段时间往`dual_goal_input.py`里按世界坐标输过目标点，
飞机实际收到的目标点也是错的（差了一个INIT_X）。
修复：用第一帧UWB位置当"局部系原点"，之后每一帧都减掉这个值再喂给PX4——
PX4内部状态从此也是"局部系原点=spawn位置=(0,0,0)"，跟`dlio`/`gt`两种模式
完全同构，`dual_goal_input.py`/`status_monitor.py`/`local_position_readback_
node`都不用为了适配这个新模式而改动"局部坐标+INIT_X=全局坐标"这条假设。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range

GRAVITY = 9.80665  # 跟dlio.yaml的odom/gravity保持一致


def _rotate_body_to_world(vx, vy, vz, roll, pitch, yaw):
    """ZYX(yaw-pitch-roll)机体系->世界系旋转，纯标量实现，不引入numpy依赖。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    return (
        r00 * vx + r01 * vy + r02 * vz,
        r10 * vx + r11 * vy + r12 * vz,
        r20 * vx + r21 * vy + r22 * vz,
    )


class UwbImuFusionNode(Node):

    def __init__(self):
        super().__init__('uwb_imu_fusion_node')

        self.declare_parameter('uwb_pose_topic', 'uwb/pose_abs')
        self.declare_parameter('imu_topic', 'mid360/imu')
        self.declare_parameter('pose_pub_topic', 'mavros/vision_pose/pose_cov')
        self.declare_parameter('odom_pub_topic', 'uwb_imu_odom')
        # 2026-09-13新增（A0清单/实现方案1.1a节）：z融合改用测距雷达做AGL
        # 观测源，见文件头说明。`hrlv_ez4_pub`是mavros这套仿真里实际发布
        # 下视测距雷达读数的话题名（对应真实硬件Lightware/Garmin一类的
        # 单点测距模块，`sensor_msgs/Range`类型）。
        self.declare_parameter('range_topic', 'mavros/hrlv_ez4_pub')
        # 测距雷达最小可信量程，参考原agl_hold设计给的0.19-0.2米实测最小
        # 量程，这里留一点余量取0.25米——低于这个值的读数视为"贴地/超出
        # 量程"，不能拿来当AGL观测修正z，只保留IMU纯积分传播。
        # 2026-09-25 用户决定：改成 0.0，**雷达读数是多少就是多少**，不再因为
        # "低于量程下限"把读数丢掉。原来默认 0.25，本意是挡贴地/起降瞬间跌破量程
        # 的乱码，副作用是：飞机一触地雷达读 0.000，被判"不可信"后 z 这一路失去
        # 外部观测、只剩纯 IMU 积分，静止在地面上却一路飘到 0.30 米并停住；喂给
        # PX4 的视觉高度就成了 0.3 米，PX4 认为飞机还在空中，永远不报
        # LANDED_STATE_ON_GROUND，pt4ctrl 因此不发解锁（PX4CtrlFSM.cpp:319），
        # land() 等 30 秒超时，下一次起飞还会被 "Reject AUTO_TAKEOFF. land detector
        # says that the drone is not landed now!" 拒掉。2026-09-25 实测取证：触地
        # 后真值 z 恒为 0.05，里程计 z 却 0.06->0.24->0.32->0.30。
        # 0 恰恰是"我就在地上"的最强证据，不该被当成噪声丢掉。
        self.declare_parameter('range_min_valid_m', 0.0)
        # 互补滤波时间常数（秒）——越大越平滑但修正IMU零偏/积分漂移越慢，
        # 越小越贴近UWB原始值/差分速度、噪声也越大。2026-08-13第二次修订时
        # position用了比velocity短很多的0.15秒，实测"还是太抖"；这次两个都
        # 用同一个更大的默认值，牺牲一些响应速度换平滑度。没有标定过，纯
        # 保守起点，抖或滞后都可以再调。
        self.declare_parameter('velocity_correction_tau', 0.5)
        self.declare_parameter('position_correction_tau', 0.5)
        # 协方差是手调的固定值，不是从UWB/IMU消息自己的噪声字段动态读出来的
        # ——跟repub_odom.py（_livox_odom_cb/_mocap_pose_cb）的既有做法一致，
        # 这套仿真里Gazebo IMU插件的orientation_covariance是否有意义没有
        # 核实过，不敢直接信；量级上跟uwb_ground_truth_node.py默认的
        # range_noise_std=0.05m/yaw_noise_std_deg=2°对应换算成方差
        # （0.05^2=0.0025，2°≈0.035rad -> 0.035^2≈0.0012），如果改了那边的
        # 噪声参数，这里也要跟着手动改，两边没有做成同一个数据源。
        self.declare_parameter('pos_variance', 0.0025)
        self.declare_parameter('yaw_variance', 0.0012)
        # roll/pitch来自IMU重力对齐，没有实测过Gazebo IMU插件的姿态噪声量级，
        # 给一个比yaw_variance宽松一些的保守默认值，不是精确标定过的数字。
        self.declare_parameter('roll_pitch_variance', 0.005)

        self.pos_variance = self.get_parameter('pos_variance').value
        self.yaw_variance = self.get_parameter('yaw_variance').value
        self.roll_pitch_variance = self.get_parameter('roll_pitch_variance').value
        self.velocity_correction_tau = self.get_parameter('velocity_correction_tau').value
        self.position_correction_tau = self.get_parameter('position_correction_tau').value
        self.range_min_valid_m = float(self.get_parameter('range_min_valid_m').value)

        self._last_uwb_yaw = None  # 弧度，来自最近一帧uwb/pose_abs
        self._last_uwb_xyz = None
        self._last_roll = 0.0
        self._last_pitch = 0.0
        self._last_angular_velocity = (0.0, 0.0, 0.0)  # 机体系，IMU陀螺仪直接透传
        # 局部系原点=第一帧收到的UWB绝对位置，见文件头2026-08-11勘误。
        self._origin_xyz = None

        # 位置/速度互补滤波状态：由IMU传播(~220Hz)，由UWB周期性修正(~145Hz)，
        # 都在世界系(ENU)下，局部系原点跟_origin_xyz一致（起点为0）。
        self._p_state = [0.0, 0.0, 0.0]
        self._v_state = [0.0, 0.0, 0.0]
        self._last_imu_time = None
        self._last_uwb_time = None

        # 2026-09-13新增：测距雷达读数缓存，z融合观测源（见文件头说明）。
        # _last_compensated_height记上一次"有效"的水平倾斜补偿高度，专门
        # 给z这一路的速度互补滤波修正用差分算粗糙速度——跟x/y那两行用
        # prev_uwb_xyz差分是同一个道理，只是z这里的"上一帧观测"未必是
        # 上一次_uwb_cb回调时刻的（range读数无效时会跳过更新，不能直接
        # 复用"上一帧UWB位置"这套下标对应关系）。
        self._last_range_m = None
        self._last_range_time = None
        self._last_compensated_height = None

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('pose_pub_topic').value, 10)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_pub_topic').value,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.create_subscription(
            PoseStamped, self.get_parameter('uwb_pose_topic').value, self._uwb_cb, 10)
        # Gazebo IMU插件是常见的BEST_EFFORT高频话题，跟gt_odom_bridge_node
        # 订阅mid360/imu用的QoS保持一致。
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._imu_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        # 2026-09-13新增：测距雷达读数订阅，同样是常见的BEST_EFFORT高频
        # 话题，QoS跟imu/uwb两路保持一致的写法。
        self.create_subscription(
            Range, self.get_parameter('range_topic').value, self._range_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info(
            f"uwb_imu_fusion_node就绪：uwb_pose_topic={self.get_parameter('uwb_pose_topic').value}, "
            f"imu_topic={self.get_parameter('imu_topic').value}, "
            f"range_topic={self.get_parameter('range_topic').value}, "
            f"pose_pub_topic={self.get_parameter('pose_pub_topic').value}, "
            f"odom_pub_topic={self.get_parameter('odom_pub_topic').value}")

    def _imu_cb(self, msg: Imu):
        # 2026-09-13改：roll/pitch不再从这条IMU消息的orientation字段解出
        # ——self._last_roll/self._last_pitch现在由_uwb_cb从uwb/pose_abs
        # 的姿态分量更新（见文件头2026-09-13新增说明，原因是跟yaw同源、
        # 时间戳对齐，不用再单独维护mid360/imu这一路姿态解算）。这个回调
        # 继续保留、不整个删掉——线加速度/角速度的积分传播仍然要用
        # mid360/imu，只是不再从它身上取姿态角这两个分量。
        self._last_angular_velocity = (
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

        # 速度/位置传播：机体系比力(accelerometer读数) -> 世界系 -> 减重力 ->
        # 积分得速度 -> 再积分(半隐式欧拉)得位置。yaw用最近一次UWB给的值
        # （IMU本身的yaw不可信，见文件头说明），第一帧UWB到达前用0。
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self._last_imu_time is not None:
            dt = t - self._last_imu_time
            # dt<=0：时间戳回退/重复，跳过；dt过大：可能是话题短暂中断后
            # 重新开始，此时用陈旧的dt积分误差会很大，同样跳过更安全。
            if 0.0 < dt < 0.1:
                yaw = self._last_uwb_yaw if self._last_uwb_yaw is not None else 0.0
                ax, ay, az = _rotate_body_to_world(
                    msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z, self._last_roll, self._last_pitch, yaw)
                az -= GRAVITY
                self._v_state[0] += ax * dt
                self._v_state[1] += ay * dt
                self._v_state[2] += az * dt
                self._p_state[0] += self._v_state[0] * dt
                self._p_state[1] += self._v_state[1] * dt
                self._p_state[2] += self._v_state[2] * dt
        self._last_imu_time = t

    def _range_cb(self, msg: Range):
        """缓存最近一次测距雷达读数+时间戳（2026-09-13新增，见文件头
        说明）。这里只做缓存，不做"是否可信"的判断——量程下限
        （range_min_valid_m）和"是否超时"这两条有效性判断都放在实际
        消费这个缓存值的_uwb_cb里做，缓存本身不应该因为这一帧读数超出
        量程就丢弃，"存不存"和"信不信"是两件独立的事。
        """
        self._last_range_m = msg.range
        self._last_range_time = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    def _uwb_cb(self, msg: PoseStamped):
        if self._origin_xyz is None:
            self._origin_xyz = (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        prev_uwb_xyz = self._last_uwb_xyz  # 修正用，取本次覆盖之前的上一帧
        self._last_uwb_xyz = (
            msg.pose.position.x - self._origin_xyz[0],
            msg.pose.position.y - self._origin_xyz[1],
            msg.pose.position.z - self._origin_xyz[2])
        q = msg.pose.orientation
        self._last_uwb_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 2026-09-13新增：roll/pitch改从这条uwb/pose_abs消息里解出（不再
        # 依赖mid360/imu的orientation，见文件头/_imu_cb里的说明）。
        # uwb_ground_truth_node.py这次改完之后，pose_abs的roll/pitch分量
        # 是Gazebo真值透传，跟这里的yaw是同一条消息、同一个时间戳，解法
        # 跟_imu_cb原来那套atan2/asin公式一致（只是数据源换了）。
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._last_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))  # 数值误差可能让sinp略超出[-1,1]，asin会炸
        self._last_pitch = math.asin(sinp)

        # 位置/速度互补滤波修正：位置往观测值拉，速度往连续两帧观测差分
        # 出的粗糙速度拉——两者都只用来防止纯惯性积分的长期漂移，不直接
        # 把观测原始值/差分结果当输出。
        # 2026-09-13改：x/y/z原来统一用同一个for i in range(3)循环处理，
        # 这次拆开——x/y两路数据源不变（仍是uwb/pose_abs的x/y分量），
        # z这一路换成测距雷达+水平倾斜补偿（见文件头"为什么z的观测源要
        # 跟x/y分开处理"一节，这里不是想统一处理却疏漏，是两路观测的
        # 物理意义/误差来源/失效模式都不同，故意分开写）。
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self._last_uwb_time is not None and prev_uwb_xyz is not None:
            dt_uwb = t - self._last_uwb_time
            if 0.0 < dt_uwb < 1.0:
                tau_p = self.position_correction_tau
                alpha_p = dt_uwb / (tau_p + dt_uwb) if tau_p > 0.0 else 1.0
                tau_v = self.velocity_correction_tau
                alpha_v = dt_uwb / (tau_v + dt_uwb) if tau_v > 0.0 else 1.0

                # x/y两路：跟改动前完全一样，数据源仍是uwb/pose_abs的
                # x/y分量，不受这次z融合改动影响。
                for i in (0, 1):
                    self._p_state[i] = (1.0 - alpha_p) * self._p_state[i] + alpha_p * self._last_uwb_xyz[i]
                    v_uwb_diff = (self._last_uwb_xyz[i] - prev_uwb_xyz[i]) / dt_uwb
                    self._v_state[i] = (1.0 - alpha_v) * self._v_state[i] + alpha_v * v_uwb_diff

                # z这一路：观测源换成测距雷达range，先判断这一帧range读数
                # 是否可信——量程下限（range_min_valid_m，贴地/起降瞬间会
                # 跌破）+ 是否超时（0.5秒，跟本文件_imu_cb里"dt过大丢弃"
                # 用的0.1秒是同一档次的字面量阈值，留给range话题正常发布
                # 间隔足够容忍量，又不至于拿陈旧读数去修正z）。
                range_age = (t - self._last_range_time) if self._last_range_time is not None else None
                range_fresh = range_age is not None and 0.0 <= range_age < 0.5
                if (self._last_range_m is not None and range_fresh
                        and math.isfinite(self._last_range_m)
                        and self._last_range_m >= self.range_min_valid_m):
                    # 水平倾斜补偿：测距雷达测的是沿机体轴向下的斜距，飞机
                    # 有倾角时斜距会比真实垂直离地高度更长，不修正会系统性
                    # 高估高度——乘cos(roll)*cos(pitch)换算成竖直分量（这两
                    # 个角刚从这条消息里解出来，是当前最新值）。
                    compensated_height = (
                        self._last_range_m * math.cos(self._last_roll) * math.cos(self._last_pitch))
                    self._p_state[2] = (1.0 - alpha_p) * self._p_state[2] + alpha_p * compensated_height
                    if self._last_compensated_height is not None:
                        v_range_diff = (compensated_height - self._last_compensated_height) / dt_uwb
                        self._v_state[2] = (1.0 - alpha_v) * self._v_state[2] + alpha_v * v_range_diff
                    self._last_compensated_height = compensated_height
                # range无效/超时（贴地/起降瞬间常见）：跳过z这一路的外部
                # 观测修正，只保留_imu_cb已经在做的纯IMU积分传播——
                # _p_state[2]/_v_state[2]不会因为这一刻range失效就跳变或
                # 停止更新，这正是原agl_hold方案"读数不可信时不要硬算Z"
                # 这条设计原则搬到这里的地方。
        self._last_uwb_time = t

        roll, pitch, yaw = self._last_roll, self._last_pitch, self._last_uwb_yaw
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.pose.pose.position.x = self._last_uwb_xyz[0]
        out.pose.pose.position.y = self._last_uwb_xyz[1]
        # 2026-09-13补：z绝对不能用self._last_uwb_xyz[2]（UWB绝对高度）
        # ——这条out消息才是真正发给PX4（mavros/vision_pose/pose_cov）的
        # 那一路，EKF2_HGT_REF=3意味着PX4的高度主参考源就是这里的z，前面
        # 331-355行改的_p_state[2]/_v_state[2]只影响这个节点自己另外发的
        # uwb_imu_odom（给规划器/控制器用），完全不影响PX4实际收到什么。
        # 如果这里不改，A0整个改动对PX4来说等于没做——PX4还是收着UWB
        # 绝对高度，"uwb_imu模式下AGL全程默认生效"这个结论不成立。
        # 用self._last_compensated_height（range*cos(roll)*cos(pitch)的
        # 原始值，不是_p_state[2]）——跟下面"PX4要吃原始观测量，不要喂
        # 经过互补滤波的平滑值，否则两个卡尔曼滤波器串联互相干扰"这条
        # 既有原则保持一致，只是观测源从UWB换成了range，原则不变。
        # range还没来第一帧有效读数之前（刚起飞贴地的短暂窗口）用
        # self._last_uwb_xyz[2]兜底，此时两者数值上都接近0，影响很小。
        #
        # ⚠️ 2026-09-13阶段C第一次端到端测试`sdk.land()`卡死超时后，
        # 曾经怀疑过"这里冻结的z跟pt4ctrl的AUTO_LAND状态机打架"这条理论，
        # 改成了`self._p_state[2]`——但当场用户提出质疑（"肯定跟range值
        # 无关""真机不可能出这个问题"），重新起容器+全程监控
        # `mavros/extended_state.landed_state`复测了一次：**同样是这里
        # 冻结的z（改动前的代码），这次降落完全正常，landed_state干净地
        # 走了1(ON_GROUND)→2(IN_AIR)→1(ON_GROUND)，没有卡住**——直接
        # 用证据推翻了"z冻结必然卡死"这个理论，说明第一次的卡死更可能是
        # 这次会话里大量手动`ros2 topic pub`/service调用（强制解锁、
        # 重发LAND等）留下的残留状态干扰的偶发问题，不是这段代码本身
        # 稳定复现的bug，已经把基于错误推理做的`_p_state[2]`改动撤回
        # 到原样。真正的落地卡死根因还没有查清楚，暂不下结论，见
        # DEBUG_JOURNAL.md 2026-09-13"用户质疑z融合理论"这条记录。
        out.pose.pose.position.z = (
            self._last_compensated_height
            if self._last_compensated_height is not None
            else self._last_uwb_xyz[2])
        out.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        out.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        out.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        out.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        cov = [0.0] * 36
        cov[0] = self.pos_variance   # x
        cov[7] = self.pos_variance   # y
        cov[14] = self.pos_variance  # z
        cov[21] = self.roll_pitch_variance  # roll
        cov[28] = self.roll_pitch_variance  # pitch
        cov[35] = self.yaw_variance         # yaw
        out.pose.covariance = cov

        self._pose_pub.publish(out)

        # 高频里程计：姿态跟上面vision_pose一样，但位置/速度用的是互补滤波
        # 状态_p_state/_v_state（平滑的惯性传播结果），不是原始观测值——只给
        # 规划器/控制器用，不影响PX4那一路（PX4继续吃上面`out`里的原始观测量
        # ——x/y是UWB原始值，z是range原始值：PX4自己的EKF2就是设计来吃带
        # 噪声的原始观测量的，喂进去之前先滤一遍反而是错的，两个卡尔曼滤波器
        # 串联会互相干扰各自对噪声统计特性的假设）。
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = msg.header.frame_id
        odom.pose.pose.position.x = self._p_state[0]
        odom.pose.pose.position.y = self._p_state[1]
        odom.pose.pose.position.z = self._p_state[2]
        odom.pose.pose.orientation = out.pose.pose.orientation
        odom.pose.covariance = cov
        odom.twist.twist.linear.x = self._v_state[0]
        odom.twist.twist.linear.y = self._v_state[1]
        odom.twist.twist.linear.z = self._v_state[2]
        odom.twist.twist.angular.x = self._last_angular_velocity[0]
        odom.twist.twist.angular.y = self._last_angular_velocity[1]
        odom.twist.twist.angular.z = self._last_angular_velocity[2]
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = UwbImuFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
