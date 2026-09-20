#!/usr/bin/env python3
"""按"相对飞机当前高度"的窗口过滤PointCloud2，只用于地面站/RViz显示，
不改动原始话题。

背景：封闭空间飞行时想看点云俯视图，天花板的点会把下面的障碍物/地面全部
遮住。这个节点订阅原始点云，把z不在[飞机当前z+min_z, 飞机当前z+max_z]
区间内的点丢掉，发布到一个新话题——原始话题(`dlio/odom_node/pointcloud/
deskewed`、`grid_map/occupancy_inflate`)完全不受影响，避障用的还是没
过滤过的完整点云，这个过滤只是给人看的。

2026-08-27：min_z/max_z原来是world系(odom/map)下的固定绝对高度，改成
相对飞机当前高度的偏移量——飞机爬升/下降时这个过滤窗口跟着一起动，不会
出现飞机飞出固定高度带之后自己周围的点云也被滤掉的情况。代价是多订阅一路
里程计(`odom_topic`)，且过滤结果依赖里程计更新是否及时——里程计还没收到
过第一帧之前，这个节点不发布任何东西(安全跳过，不拿假设的高度乱滤)。

用`sensor_msgs_py.point_cloud2`纯Python读写，不依赖PCL——过滤逻辑简单到
不需要PCL的C++性能，用Python实现改起来更方便。
"""
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ZFilterNode(Node):

    def __init__(self):
        super().__init__('pointcloud_z_filter')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        # dlio/odom_node/odom跟deskewed/occupancy_inflate两路点云是同一个
        # odom/map坐标系(这套集成里两者数值上是同一个坐标系，见
        # flight-stack-entrypoint.sh里那条恒等静态TF的说明)，直接读
        # position.z跟点云里的z可以直接相减/相加，不需要额外坐标变换。
        self.declare_parameter('odom_topic', 'dlio/odom_node/odom')
        self.declare_parameter('min_z', -0.2)
        self.declare_parameter('max_z', 1.0)
        # 2026-09-03新增：处理频率上限[Hz]，0=不限速（保持原有行为）。
        # 背景：这个节点是Python实现、逐点处理全密度点云，真机实测吃满一个核
        # （8核Orin NX上占104% CPU），是整机负载里最大的单点。而它的下游是
        # 占据栅格/避障，10Hz完全够用——障碍物不会在100ms内跑掉。输入点云
        # 实测20Hz，限到10Hz直接省掉一半计算量，且不改变任何功能。
        self.declare_parameter('max_rate_hz', 0.0)

        self._max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        self._last_proc_t = 0.0

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        if not input_topic or not output_topic:
            raise ValueError('input_topic/output_topic不能为空，必须显式传参')
        odom_topic = self.get_parameter('odom_topic').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value

        self._drone_z = None

        # 订阅端不显式传QoS(用rclpy默认的RELIABLE)——DLIO的deskewed/kf_cloud/
        # odom和grid_map的occupancy_inflate发布时都是裸整数depth构造的
        # publisher(对应默认RELIABLE)，不是SensorDataQoS，订阅端用默认QoS
        # 才能连上，用BEST_EFFORT反而会因为QoS不兼容收不到任何消息——这个坑
        # DLIO自己的点云订阅端之前踩过一次(dlio_pointcloud_qos.patch)，这里
        # 对齐实际发布端用的QoS，不要想当然套SensorDataQoS。
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(PointCloud2, input_topic, self._cb, 5)

        self.get_logger().info(
            f'pointcloud_z_filter就绪：{input_topic} -> {output_topic}，'
            f'跟随{odom_topic}的相对高度窗口[{self.min_z}, {self.max_z}]（米）')

    def _odom_cb(self, msg: Odometry):
        self._drone_z = msg.pose.pose.position.z

    def _cb(self, msg: PointCloud2):
        # 限速：距上次真正处理不足一个周期就直接丢掉这帧。用单调时钟，
        # 不受系统时间跳变影响；丢帧发生在做任何解析之前，省的是全部开销。
        if self._max_rate_hz > 0.0:
            now = time.monotonic()
            if now - self._last_proc_t < 1.0 / self._max_rate_hz:
                return
            self._last_proc_t = now

        if self._drone_z is None:
            # 还没收到过里程计，没有基准高度可用——安全跳过，等第一帧odom
            # 到达后自然恢复，不拿假设的高度乱滤。
            return

        lo = self._drone_z + self.min_z
        hi = self._drone_z + self.max_z
        # read_points在Humble返回的是带具名字段的结构化numpy数组，直接用
        # points['z']做向量化布尔筛选，比逐点Python循环快得多——点云单帧
        # 可能有几万个点，这个回调按点云原始频率(~20Hz)触发，纯Python循环
        # 逐点判断在这个数据量级下会明显拖慢。
        points = point_cloud2.read_points(msg, skip_nans=True)
        mask = (points['z'] >= lo) & (points['z'] <= hi)
        filtered = points[mask]

        # 2026-08-27修复：原来这里调用point_cloud2.create_cloud(msg.header,
        # msg.fields, filtered)，实测两路点云（deskewed/occupancy_inflate）
        # 双双必崩：create_cloud()内部会重新算一次dtype_from_fields(fields)
        # 但**不传point_step**，而上面read_points()构造filtered时是带着
        # point_step算的(dtype_from_fields(msg.fields, point_step=msg.
        # point_step))——原始点云字段之间/末尾只要有一点padding(msg.
        # point_step大于各字段紧密排列的字节数，这两路点云都有)，两次算出
        # 来的dtype的itemsize就对不上，create_cloud()内部"points.dtype ==
        # dtype_from_fields(fields)"这行断言必炸(AssertionError)；deskewed
        # 这路炸得更彻底，字段声明顺序/偏移量的组合触发了numpy更严格的buffer
        # 协议检查，直接ValueError("dtypes with overlapping or out-of-order
        # fields are not representable as buffers")，create_cloud()内部转
        # memoryview那步都走不到。两个节点因此从上线起就一直在crash-loop，
        # `grid_map/occupancy_inflate_z_filtered`等两个输出话题实测从来没
        # 成功发布过一条消息。
        # 修复思路：filtered本来就是read_points()内部用
        # `np.ndarray(..., buffer=msg.data)`直接reinterpret原始字节数组
        # 筛出来的、itemsize已经严格等于msg.point_step的结构化数组——不需要
        # 再让create_cloud()按它自己那套不带point_step的dtype重新校验/转换
        # 一遍，直接绕开create_cloud()，复用msg的其它元数据(fields/
        # point_step/is_bigendian/is_dense都不受这次过滤影响)+
        # filtered.tobytes()手工拼出输出消息，两种崩溃场景都不会再触发。
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(filtered)
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * len(filtered)
        out.is_dense = msg.is_dense
        out.data = filtered.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ZFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
