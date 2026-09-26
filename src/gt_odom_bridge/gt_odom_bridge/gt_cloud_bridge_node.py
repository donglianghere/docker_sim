#!/usr/bin/env python3
"""LOCALIZATION_SOURCE=gt/uwb_imu两种模式下，把Gazebo原始mid360点云变换到
odom系，补上DLIO本来提供的dlio/odom_node/pointcloud/deskewed话题的等价物。

背景：ego_planner的plan_env/grid_map.cpp直接信任收到的点云已经在odom系下
（不做任何TF lookup，见ego_planner_docker_sim.launch.py文件头注释），真实
LOCALIZATION_SOURCE=uwb_slam模式下这份点云由DLIO自己的odom.cc::publishCloud()
用解算出的位姿做pcl::transformPointCloud()配准后发布；=gt/uwb_imu模式下
DLIO不跑，这个话题从设计上就不存在，ego_planner因此完全收不到障碍物数据
（不是崩溃，是静默地对障碍物一无所知）。

⚠️ 2026-08-13重写：原来的实现是"订阅点云 -> 按点云自己的时间戳去TF树里
lookup_transform(odom_frame, cloud_frame_id, 点云时间戳)"，这条路径在
LOCALIZATION_SOURCE=uwb_imu模式下暴露了一个真实bug——TF树里
"{ns}/odom -> {ns}/base_link"这条动态变换由local_position_readback_node
广播，它的时间戳不是`mavros/local_position/odom`自己的时间戳（实测过，
那是真墙钟，跟仿真的假epoch时钟域相差1.6年），而是"借用"最近一帧
`mid360/imu`的时间戳贴上去的——这个"借"的动作只解决了时钟域冲突，不保证
"贴上去的时间"真的对应这份由PX4 EKF2融合出来的位置数据实际生效的那一刻。
`uwb_imu`模式下位置数据经EKF2+MAVLink有真实处理延迟（这也是这条链路
更新率上限只有~48Hz的同一个根因），"贴错时间标签"因此不是无害的重新
标记，而是让点云配准用的位姿跟grid_map.cpp自己内部缓存的当前飞机位置
（md_.camera_pos_，同样来自dlio/odom_node/odom，但可能是更新的一帧）
存在偏差。飞机移动时这个偏差被放大，mid360扫到飞机自己机身/桨叶的近距离
自扫描伪点（正常应该被grid_map.cpp里`ego_planner_grid_map_min_ray_length.
patch`加的0.4米半径过滤器挡掉）配准后"看起来"超出了0.4米，滤波器判断
"这不是自扫描点"而放行，最终把飞机自己所在的格子写成占据——ego_planner
的check_collision_and_rebound()因此报"the drone is in obstacle. This
should not happen."，陷进拒绝规划的死循环。`gt`模式下这套"借时间戳"手法
凑巧无害（数据源是Gazebo真值，没有处理延迟，"贴错时间"不对应真实位姿
差异），所以这个bug只在`uwb_imu`模式下才会实际发作。

修复思路：不再通过TF按时间戳查找动态位姿，改成"点云直接用里程计做变换"
——直接订阅dlio/odom_node/odom，缓存收到的最新一帧位姿，点云来了就直接
拿这个缓存的位姿变换，不做任何时间戳匹配/插值。TF只保留一处用途：
{ns}/base_link -> 点云frame_id（{ns}/{ns}_livox）这条纯静态的传感器安装
外参，这条变换不随时间变化，用tf2查一次、缓存下来即可，不存在"哪个时刻"
的问题。这样点云变换用的位姿，跟grid_map.cpp自己缓存的camera_pos_始终
是"同一份最新数据"（或至多差一帧的正常滞后，不是被系统性贴错时间标签
带来的偏差），min_ray_length过滤器能正常生效。

只做xyz搬运，不透传intensity/tag/line等livox自定义字段——grid_map.cpp
只关心点的三维坐标，DLIO的deskewed点云本身也是普通pcl::PointCloud<PointType>
经pcl::toROSMsg()转换出来的标准xyz(+可选intensity)布局，没有必要在这个
桥接节点里保留livox驱动私有字段的精确二进制布局。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数(x,y,z,w) -> 3x3旋转矩阵，标准公式，跟scipy.spatial.transform.Rotation
    结果一致（ros2_px4_stack/dynus_offboard_node.py里也用同一个库做同类换算，
    这里手写避免多引入一个scipy依赖，纯numpy）。里程计位姿和静态传感器外参
    共用这一个函数，不重复写两遍。"""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class GtCloudBridgeNode(Node):

    def __init__(self):
        super().__init__('gt_cloud_bridge_node')

        ns = self.get_namespace().strip('/')
        self.odom_frame = f'{ns}/odom' if ns else 'odom'
        self.base_link_frame = f'{ns}/base_link' if ns else 'base_link'

        # TF只用来查一次性的静态传感器外参({ns}/base_link -> 点云frame_id)，
        # 不再用来查随时间变化的位姿——静态变换没有"哪个时刻"的问题，用
        # 默认的非阻塞lookup_transform（不传timeout）查，查不到就跳过这一帧、
        # 下一帧再试，不会阻塞回调，也就不需要再像旧版那样开
        # MultiThreadedExecutor规避"自己等自己"的死锁，见main()。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._static_base_to_sensor = {}  # cloud frame_id -> (t_xyz, R) 缓存，查到一次不再重复查

        # 最新一帧里程计位姿缓存——点云到达时直接用这个，不等、不插值、
        # 不按时间戳去TF树里找。跟gt_odom_bridge_node/local_position_
        # readback_node发布dlio/odom_node/odom用的默认QoS(depth=10)对齐。
        # ---- 已知静态障碍物（立柱）注入，2026-09-26 按用户要求加 ----
        # 起因：grid_map 的点云路径每帧 resetBuffer 推倒重建、没有任何时间累积
        # （见 plan_env/src/grid_map.cpp::cloudCallback），所以"这一帧雷达没扫到"
        # 就等于"地图上没有障碍"。实测两种情况都会发生：
        #   1) 竖直覆盖空洞——飞行中 4.2~5.8% 的帧里，3# 立柱在"跟飞机同高
        #      ±0.1m"范围内一个点都没有（竖直膨胀恰好也只有 ±0.1m，补不上），
        #      单次最长 0.46 秒；
        #   2) 整片点云异常——曾测到连续 2.6 秒三根立柱同时一个点都没有，而
        #      点云本身照常是 20000 点（偶发，尚未定案）。
        # 立柱是赛场固定设施，位置尺寸已知，把它们每帧直接写进点云，上面两种
        # 情况就都影响不到立柱了。注入点在 odom 系里按锁定后的 world->odom 静态
        # TF 算出来，**不经过上面那条"最新里程计缓存"的变换链路**，所以那条
        # 链路出问题时注入的柱子依然在正确位置。
        self.declare_parameter('static_obstacles_enabled', True)
        # 世界系坐标，按 [x1,y1, x2,y2, ...] 拉平。默认是本赛场三根立柱。
        self.declare_parameter('static_obstacles_xy', [4.5, 7.0, -4.5, 7.0, 0.0, 0.0])
        self.declare_parameter('static_obstacle_half_m', 0.5)     # 立柱半宽(1x1米)
        self.declare_parameter('static_obstacle_top_m', 5.0)      # 采样到多高
        self.declare_parameter('static_obstacle_bottom_m', 0.2)   # 从多高开始（避开地面过滤区）
        # xy 采样间距：要让最外圈采样点落在柱子表面上，膨胀才是从表面算起的。
        # 0.5 米 = 半宽，正好给出 -0.5/0/+0.5 三档、每层 9 个点。
        self.declare_parameter('static_obstacle_xy_step_m', 0.5)
        # z 采样间距：必须 <= 2*竖直膨胀半径，否则层与层之间留空。默认 0.15
        # 是按最保守的竖直膨胀 ±0.1m（grid_map 默认 obstacles_inflation_z=0.1）
        # 取的；把那个参数调大之后这里也可以跟着放宽以省点数。
        self.declare_parameter('static_obstacle_z_step_m', 0.15)
        self.declare_parameter('world_frame', 'world')
        self._static_obstacles_enabled = bool(
            self.get_parameter('static_obstacles_enabled').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self._injected_points = None      # (M,3) odom系，算出来一次就缓存
        self._injected_warned = False

        self._latest_odom_t = None  # (x, y, z)
        self._latest_odom_R = None  # 3x3

        self.odom_sub = self.create_subscription(
            Odometry, 'dlio/odom_node/odom', self._on_odom, 10)

        # 跟Gazebo雷达插件livox_points_plugin.cpp发布点云用的QoS对齐
        # （经典Gazebo传感器话题一贯的BEST_EFFORT高频约定）。
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # 跟DLIO的deskewed_pub（create_publisher<PointCloud2>("deskewed", 1)，
        # 默认QoS）对齐，ego_planner的grid_map/cloud订阅按这个默认QoS建的。
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(
            PointCloud2, 'dlio/odom_node/pointcloud/deskewed', pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, 'mid360_PointCloud2', self._on_cloud, sub_qos)

        self.get_logger().info(
            f'gt_cloud_bridge up: mid360_PointCloud2 -> dlio/odom_node/pointcloud/deskewed '
            f'(target frame={self.odom_frame}, 位姿直接取自dlio/odom_node/odom缓存，不查TF时间戳)')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._latest_odom_t = np.array([p.x, p.y, p.z])
        self._latest_odom_R = _quat_to_rot(q.x, q.y, q.z, q.w)

    def _get_static_base_to_sensor(self, sensor_frame: str):
        """查{ns}/base_link -> sensor_frame这条静态外参，查到一次就缓存住，
        不重复查——这条变换不随时间变化（雷达在机体上的安装位置固定），
        跟里程计位姿是两件独立的事，混在一起查才是旧版真正的问题所在。"""
        cached = self._static_base_to_sensor.get(sensor_frame)
        if cached is not None:
            return cached
        try:
            tf = self.tf_buffer.lookup_transform(self.base_link_frame, sensor_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'查静态外参({self.base_link_frame} <- {sensor_frame})失败，'
                f'这一帧点云先跳过，下一帧重试: {exc}',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        result = (np.array([t.x, t.y, t.z]), _quat_to_rot(q.x, q.y, q.z, q.w))
        self._static_base_to_sensor[sensor_frame] = result
        return result

    def _build_injected_points(self):
        """把已知立柱在 odom 系里采样成点，成功一次就缓存。

        world->odom 这条变换是起飞时一次性锁定的静态 TF（origin_setter_node 发
        world -> {ns}/map，{ns}/map -> {ns}/odom 是恒等），锁定之前查不到——那就
        先不注入，下一帧再试；飞机那时还在停机坪上，没有避障需求。

        ⚠️ 注入位置的准确性完全取决于这条锁定 TF。原点锁在里程计瞬态上会让整
        套 world_to_local 都带偏（2026-09-24 踩过，已加静止+雷达一致性判据），
        那种情况下注入的柱子也会跟着偏 —— 现场若发现柱子位置对不上，先查原点
        锁定，或者直接把 static_obstacles_enabled 置 False 关掉注入。
        """
        if self._injected_points is not None:
            return self._injected_points
        if not self._static_obstacles_enabled:
            return None
        flat = list(self.get_parameter('static_obstacles_xy').value or [])
        if len(flat) < 2 or len(flat) % 2 != 0:
            if not self._injected_warned:
                self.get_logger().warn(
                    f'static_obstacles_xy 长度 {len(flat)} 不是成对的世界坐标，不注入')
                self._injected_warned = True
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.world_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().info(
                f'起飞点原点还没锁定（查 {self.odom_frame} <- {self.world_frame} 失败：{exc}），'
                f'暂不注入已知立柱，下一帧重试',
                throttle_duration_sec=5.0)
            return None

        half = float(self.get_parameter('static_obstacle_half_m').value)
        top = float(self.get_parameter('static_obstacle_top_m').value)
        bottom = float(self.get_parameter('static_obstacle_bottom_m').value)
        xy_step = max(1e-3, float(self.get_parameter('static_obstacle_xy_step_m').value))
        z_step = max(1e-3, float(self.get_parameter('static_obstacle_z_step_m').value))

        n_xy = max(1, int(round(2 * half / xy_step)) + 1)
        offs = np.linspace(-half, half, n_xy)
        zs = np.arange(bottom, top + 1e-6, z_step)
        pts = []
        for k in range(0, len(flat), 2):
            cx, cy = float(flat[k]), float(flat[k + 1])
            gx, gy, gz = np.meshgrid(offs + cx, offs + cy, zs, indexing='ij')
            pts.append(np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()]))
        world_pts = np.vstack(pts)

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        odom_pts = world_pts @ R.T + np.array([t.x, t.y, t.z])
        self._injected_points = odom_pts.astype(np.float64)
        self.get_logger().info(
            f'已知立柱注入就绪：{len(flat) // 2} 根 x 每根 {n_xy}x{n_xy}x{len(zs)} 点 '
            f'= 共 {len(odom_pts)} 点/帧（world->odom 平移 '
            f'({t.x:.2f}, {t.y:.2f}, {t.z:.2f})）')
        return self._injected_points

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._latest_odom_t is None:
            # 还没收到过一帧里程计，没有位姿可用——安全跳过，等第一帧
            # dlio/odom_node/odom到达后自然恢复，不需要额外重试逻辑。
            return

        static = self._get_static_base_to_sensor(msg.header.frame_id)
        if static is None:
            return
        t_bs, R_bs = static

        # 不用read_points_numpy——它在转换前会断言"消息里*全部*字段
        # datatype一致"，livox驱动的点云实际混了float32(x/y/z/intensity)和
        # uint8(tag/line)等字段，会直接触发这个断言失败。改用read_points()
        # 先按field_names过滤出只剩x/y/z的structured array（这三个字段本身
        # datatype一致，没有这个限制），再手动拆成普通(N,3)浮点数组。
        structured = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.shape[0] == 0:
            return
        points = np.column_stack([structured['x'], structured['y'], structured['z']]).astype(np.float64)

        # 组合两段变换：先传感器外参(sensor->base_link)，再里程计位姿
        # (base_link->odom)——p_odom = R_ob @ (R_bs @ p_sensor + t_bs) + t_ob
        #                          = (R_ob @ R_bs) @ p_sensor + (R_ob @ t_bs + t_ob)
        # 两者都是当前时刻各自最新的缓存值，不做任何跨节点的时间戳匹配。
        R_ob, t_ob = self._latest_odom_R, self._latest_odom_t
        rot = R_ob @ R_bs
        trans = R_ob @ t_bs + t_ob
        transformed = points @ rot.T + trans

        # 把已知立柱拼进去（在雷达点之后，顺序对 grid_map 没有影响——它只是
        # 逐点标占据）。注入点是固定的 odom 系坐标，不参与上面那套位姿变换。
        injected = self._build_injected_points()
        if injected is not None:
            transformed = np.vstack([transformed, injected])

        out = point_cloud2.create_cloud_xyz32(msg.header, transformed.astype(np.float32))
        out.header.frame_id = self.odom_frame
        out.header.stamp = msg.header.stamp
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GtCloudBridgeNode()
    # 2026-08-13起改回rclpy.spin(node)默认的单线程执行器——旧版需要
    # MultiThreadedExecutor是因为_on_cloud()内部有阻塞的lookup_transform调用
    # （查动态位姿，需要另一个线程同时处理TransformListener的/tf回调，见
    # git历史/DEBUG_JOURNAL.md 2026-08-10记录）。现在_on_cloud()只做非阻塞的
    # 静态外参查询+纯numpy计算，没有任何阻塞等待，单线程执行器完全够用，
    # 不需要再引入多线程带来的额外复杂度。
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
