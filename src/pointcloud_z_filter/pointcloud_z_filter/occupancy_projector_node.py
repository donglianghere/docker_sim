#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把三维占据点云**投影**成二维 `nav_msgs/OccupancyGrid`，只给地面站/RViz 显示用。

为什么要有它（2026-10-03）：`ego_planner` 的 `grid_map/occupancy_inflate` 是
把膨胀后的每个占据体素都当一个点发出来的 `PointCloud2`——实测 **4.33 Hz ×
20.77 MB/s，每帧 4.8 MB**。这个量在机内走 loopback 无所谓，一旦有远程订阅者
（地面站的 `rosbridge_websocket`）就要上 WiFi，实测把上行打到 113 Mbit/s，
`rtw89` 驱动的 TX 队列冲不出去（`timed out to flush pci txch`）→ 丢 beacon →
整条链路断开重连。那次排查见 DEBUG_JOURNAL 2026-10-03。

投影之后同样的信息只要 **一字节一格**：

    0.1 m 分辨率、20×25 m 场地 = 200×250 = 50,000 格 = 49 KB/帧

比点云小约 100 倍。而且 `OccupancyGrid` 自带 `info.origin` 和 `info.resolution`，
RViz 和网页都原生渲染，能和航点、飞机位置对齐在同一坐标系里——这是它比
"发一张截图"强的地方：截图丢掉度量信息，没法叠加。

**它不改动任何原始话题**，和 `z_filter_node` 同一个定位：纯显示支路。规划器
自己用的还是原始的三维点云，一个字节都没动。

两个节点的关系（可以只用一个，也可以串起来）：

    occupancy_inflate ──► z_filter_node ──► occupancy_inflate_z_filtered
            │                （按飞机相对高度切一层薄片，仍是 PointCloud2）
            └──────────► occupancy_projector ──► occupancy_2d
                             （拍扁成 2D OccupancyGrid）

`input_topic` 默认直接接原始点云——这样**不依赖里程计**（z_filter 要读 odom
算相对高度窗口，没收到 odom 就整帧跳过）。想先切片再投影，把 `input_topic`
指到 `grid_map/occupancy_inflate_z_filtered` 即可。

坐标范围的处理：默认**自动贴合**当前帧的点云包围盒，再按 `snap_m`（默认
1 m）对齐原点，避免每帧原点都抖一点、网页上的底图跟着晃。给了
`fixed_extent` 就用写死的范围（显示更稳，但要知道点云所在坐标系的原点在哪
——真机上那是起飞点，不是场地角点）。
"""
import array
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class OccupancyProjectorNode(Node):

    def __init__(self):
        super().__init__('occupancy_projector')
        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('resolution', 0.1)
        # 绝对高度窗口（点云自身坐标系，不是相对飞机）。留空=不过滤、全高度
        # 一起拍扁。投影本来就把 z 合并掉了，所以这里一般不用设；只有天花板/
        # 地面噪点明显时才收一下。不读 odom 是故意的——少一个依赖少一类卡死。
        self.declare_parameter('use_z_window', False)
        self.declare_parameter('z_min', -1.0)
        self.declare_parameter('z_max', 3.0)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('lazy_subscribe', True)
        # 自动贴合时在包围盒外留的余量，和原点对齐的粒度
        self.declare_parameter('margin_m', 1.0)
        self.declare_parameter('snap_m', 1.0)
        # 写死范围：[x_min, y_min, x_max, y_max]，四个都是 0 表示不启用
        self.declare_parameter('fixed_extent', [0.0, 0.0, 0.0, 0.0])
        # 一格里有点就标这个值。OccupancyGrid 约定 0~100 是占据概率，-1 未知
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('unknown_as_free', True)

        self._in = self.get_parameter('input_topic').value
        self._out = self.get_parameter('output_topic').value
        if not self._in or not self._out:
            raise ValueError('input_topic/output_topic 不能为空，必须显式传参')
        self._res = float(self.get_parameter('resolution').value)
        if self._res <= 0.0:
            raise ValueError(f'resolution 必须为正，收到 {self._res}')
        self._use_zw = bool(self.get_parameter('use_z_window').value)
        self._zmin = float(self.get_parameter('z_min').value)
        self._zmax = float(self.get_parameter('z_max').value)
        self._rate = float(self.get_parameter('max_rate_hz').value)
        self._lazy = bool(self.get_parameter('lazy_subscribe').value)
        self._margin = float(self.get_parameter('margin_m').value)
        self._snap = float(self.get_parameter('snap_m').value)
        fe = [float(v) for v in self.get_parameter('fixed_extent').value]
        self._fixed = fe if any(v != 0.0 for v in fe) else None
        self._occ = int(self.get_parameter('occupied_value').value)
        self._free = 0 if bool(self.get_parameter('unknown_as_free').value) else -1

        self._last = 0.0
        self._pub = self.create_publisher(OccupancyGrid, self._out, 1)
        self._sub = None
        if self._lazy:
            # 没人看就不订阅输入——上游 ego_planner 那边也会随之停止向本节点
            # 发送，省掉整条链路的开销。跟 z_filter_node 同一套做法。
            self.create_timer(1.0, self._lazy_tick)
        else:
            self._sub = self.create_subscription(PointCloud2, self._in, self._cb, 5)

        self.get_logger().info(
            f'occupancy_projector就绪：{self._in} -> {self._out}，'
            f'分辨率={self._res} m，惰性订阅={self._lazy}，'
            f'频率上限={"不限" if self._rate <= 0 else str(self._rate) + "Hz"}，'
            f'高度窗口={"[%g, %g]" % (self._zmin, self._zmax) if self._use_zw else "全高度"}，'
            f'范围={"写死 %s" % (self._fixed,) if self._fixed else "自动贴合"}')

    def _lazy_tick(self):
        want = self._pub.get_subscription_count() > 0
        if want and self._sub is None:
            self._sub = self.create_subscription(PointCloud2, self._in, self._cb, 5)
            self.get_logger().info('检测到下游订阅者，开始订阅输入点云')
        elif not want and self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
            self._last = 0.0
            self.get_logger().info('下游无订阅者，已退订输入点云')

    def _cb(self, msg: PointCloud2):
        # 限速放在解析之前，丢帧省掉的是这一帧的全部开销。单调时钟，不受
        # 系统时间跳变影响。
        if self._rate > 0.0:
            now = time.monotonic()
            if now - self._last < 1.0 / self._rate:
                return
            self._last = now

        # read_points 在 Humble 返回带具名字段的结构化 numpy 数组，后面全程
        # 向量化——单帧几万个点，纯 Python 逐点循环在这个量级会明显拖慢。
        pts = point_cloud2.read_points(msg, skip_nans=True)
        if len(pts) == 0:
            return
        xs = np.asarray(pts['x'], dtype=np.float64)
        ys = np.asarray(pts['y'], dtype=np.float64)
        if self._use_zw:
            zs = np.asarray(pts['z'], dtype=np.float64)
            keep = (zs >= self._zmin) & (zs <= self._zmax)
            xs, ys = xs[keep], ys[keep]
            if xs.size == 0:
                return

        if self._fixed:
            x0, y0, x1, y1 = self._fixed
        else:
            # 贴合包围盒 + 留余量，再把原点按 snap_m 向下对齐——原点只在
            # snap_m 的整数倍上跳，底图不会每帧抖一点。
            x0 = float(np.floor((xs.min() - self._margin) / self._snap) * self._snap)
            y0 = float(np.floor((ys.min() - self._margin) / self._snap) * self._snap)
            x1 = float(np.ceil((xs.max() + self._margin) / self._snap) * self._snap)
            y1 = float(np.ceil((ys.max() + self._margin) / self._snap) * self._snap)

        w = int(np.ceil((x1 - x0) / self._res))
        h = int(np.ceil((y1 - y0) / self._res))
        if w <= 0 or h <= 0:
            return
        if w * h > 4_000_000:
            # 400 万格已经是 4 MB，比点云本身还大，说明范围或分辨率设得不对。
            # 直接报出来而不是默默发一条巨大的消息——那正是本节点要避免的事。
            self.get_logger().warn(
                f'投影后 {w}x{h}={w*h} 格过大（>4e6），本帧跳过。'
                f'调大 resolution 或用 fixed_extent 限定范围')
            return

        ix = ((xs - x0) / self._res).astype(np.int32)
        iy = ((ys - y0) / self._res).astype(np.int32)
        inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[inb], iy[inb]

        grid = np.full(w * h, self._free, dtype=np.int8)
        # row-major：OccupancyGrid 的 data 是 y 行优先、x 列优先，
        # index = y * width + x（nav_msgs/OccupancyGrid 定义）
        grid[iy * w + ix] = self._occ

        out = OccupancyGrid()
        out.header = msg.header          # 保留原点云的 frame_id 和时间戳
        out.info.resolution = self._res
        out.info.width = w
        out.info.height = h
        out.info.origin.position.x = x0
        out.info.origin.position.y = y0
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0   # 不设的话四元数全 0，RViz 会报无效
        # `array.array('b', ...)` 而不是 `grid.tolist()`：实测 0.01 ms vs 2.61 ms
        # （30 万点/4.58 MB 帧，整条热路径 4.03 -> 1.49 ms）。'b' 是 signed char，
        # 正好对应 int8[]。**不能直接赋 numpy 数组**——rclpy 对 int8[] 的校验要
        # Python int 序列，直接赋会 AssertionError（"must be a set or sequence and
        # each value of type 'int'"），2026-10-03 实测确认过。
        out.data = array.array('b', grid.tobytes())
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyProjectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
