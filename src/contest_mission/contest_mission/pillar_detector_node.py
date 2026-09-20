#!/usr/bin/env python3
"""立柱检测节点：从ego_planner已经在维护的occupancy点云里聚类出候选立柱。

对应《2026大赛任务系统开发执行方案.md》阶段4.1。方法是"栅格连通域+高度
延伸判断"（不引入PCL欧氏聚类等重量级点云库）：
  1. 把`grid_map/occupancy`（ego_planner的plan_env/grid_map.cpp发布，
     `sensor_msgs/PointCloud2`，只在有订阅者时才发布——订阅本身就会
     触发它开始publish）持续累积进一个二维(x,y)栅格，每个格子记录
     该格子里出现过的点的z值范围[z_min, z_max]。这是个跨消息持久累积
     的过程——ego_planner的occupancy地图是"局部窗口"（跟着飞机位置
     移动），飞机飞过哪里、哪里的格子才会被更新，累积起来才能拼出
     "曾经见过的全部结构"，不是每帧都重新聚类。
  2. 排除贴地板/贴天花板的点（z在[0, z_exclude_margin]或
     [room_height-z_exclude_margin, room_height]区间）再判断"这个格子
     的z范围是否够高"——**不排除的话地板+天花板会在几乎每个格子都造成
     "z范围高达房间整高"的假阳性**，因为地板铺满整个房间、天花板也铺满
     整个房间，两者数值上会让所有格子都"看起来像"贯穿房间高度的立柱。
  3. 对"够高"的格子做二维4连通域分组（纯Python BFS，格子数量级是几百，
     不需要scipy/networkx）。
  4. 每个连通域算bounding box算出的"直径"（较长边）+格子数近似算出的
     "横截面积"，按面积过滤——**用面积而不是直径做主要判据**：立柱
     0.6x0.6米方形截面积0.36平方米，独立障碍物圆柱直径0.5米(半径
     0.25米)截面积仅0.196平方米，面积差了近1.8倍，比直径差（0.6 vs
     0.5，只差1.2倍）在栅格量化误差下更容易分辨——面积阈值取两者的
     中点，从`fire_drill_room_layout.yaml`里的`pillar_size`/
     `obstacle_cylinder.diameter`动态算出来，不是硬编码的魔数。连通域
     的直径（bounding box较长边）如果远超这个量级（比如超过1.5米），
     基本可以确定是墙面（墙是又长又薄的连通域，会被这个上限过滤掉），
     不是候选立柱。

2026-09-09新增：每个候选立柱额外拟合一个朝向(yaw)，编进输出`PoseArray`
每个`Pose.orientation`里（原来一直是恒等四元数占位，没有实际意义）——
`fire_pillar_aim_node.py`要靠这个朝向把"检测命中时刻的方位角"吸附到
最近的候选竖直面，才能算出"正对着火面"的悬停点。

⚠️ 拟合方法故意不用PCA/协方差矩阵：**实心正方形的二阶矩（协方差矩阵）
在任意旋转角度下都是各向同性的**（4重旋转对称性导致的必然结果，不是
实现细节），PCA主轴方向对旋转角完全不敏感，这条路走不通。改用
`cv2.minAreaRect`（最小外接矩形/旋转卡壳算法）——这个方法看的是点集
的凸包边界形状，不是二阶矩，旋转正方形的最小外接矩形边就是它的真实
边，能正确反映旋转角。`cv2`已经是这个包的既有依赖（`qr_apriltag_
detect_node.py`用cv_bridge/cv2处理图像），不新增依赖。角度精度受栅格
分辨率（默认0.1米，相对0.6米立柱边长）限制，且正方形4重对称导致
`minAreaRect`返回值本身就有0/90/180/270度的模糊性（OpenCV不同版本对
这个模糊性的具体表现也不完全一致）——**这些都不影响最终结果**，因为
下游`fire_pillar_approach_helper.snap_to_nearest_face()`本来就是把
任意方位角吸附到最近的4个候选面之一，粗略/模糊的原始估计值经过吸附
之后天然收敛到正确答案，不需要精确到十分之一度。
"""
import math
from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# 4连通（上下左右），不用8连通——立柱/障碍物圆柱是紧凑团块，4连通已经
# 够把同一个团块连起来，8连通反而更容易把栅格化后本不该相邻的两个
# 团块（比如两根离得比较近的立柱）错误连成一个。
_NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
_CELL_KEY_OFFSET = 1 << 20
_CELL_KEY_MULT = 1 << 21


def _load_layout_geometry():
    """从fire_drill_room_layout.yaml读房间/道具尺寸，单一权威来源
    （见该yaml文件头注释），避免这个节点里重复硬编码一份容易跟world
    文件实际尺寸脱节的数字。找不到文件时用跟yaml默认值一致的兜底值，
    保证节点在意外情况下也能跑（只是精度可能对不上实际场景）。"""
    try:
        share_dir = get_package_share_directory('contest_mission')
        with open(f'{share_dir}/config/fire_drill_room_layout.yaml') as f:
            layout = yaml.safe_load(f)
        return {
            'room_size_x': float(layout['room']['size_x']),
            'room_size_y': float(layout['room']['size_y']),
            'room_height': float(layout['room']['height']),
            'pillar_size': float(layout['pillar_size']),
            'obstacle_diameter': float(layout['obstacle_cylinder']['diameter']),
        }
    except Exception:
        return {
            'room_size_x': 20.0, 'room_size_y': 25.0, 'room_height': 6.0,
            'pillar_size': 0.6, 'obstacle_diameter': 0.5,
        }


class PillarDetectorNode(Node):
    def __init__(self):
        super().__init__('pillar_detector_node')

        geo = _load_layout_geometry()
        pillar_area = geo['pillar_size'] ** 2
        cylinder_area = math.pi * (geo['obstacle_diameter'] / 2.0) ** 2

        self.declare_parameter('input_topic', 'grid_map/occupancy')
        self.declare_parameter('output_topic', 'pillar_candidates')
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('height_span_min', 3.0)
        self.declare_parameter('z_exclude_margin', 0.3)
        # 面积阈值：两种目标横截面积的中点，动态算出来而不是写死。
        self.declare_parameter('min_area_m2', (pillar_area + cylinder_area) / 2.0)
        # 连通域"直径"（bbox较长边）上限——远超这个量级基本能确定是墙，
        # 留了将近3倍立柱边长的余量，容忍栅格化/噪声让立柱看起来偏大。
        self.declare_parameter('max_diameter_m', geo['pillar_size'] * 3.0)
        self.declare_parameter('room_size_x', geo['room_size_x'])
        self.declare_parameter('room_size_y', geo['room_size_y'])
        self.declare_parameter('room_height', geo['room_height'])
        self.declare_parameter('publish_rate_hz', 1.0)

        self.res = float(self.get_parameter('grid_resolution').value)
        self.height_span_min = float(self.get_parameter('height_span_min').value)
        self.z_margin = float(self.get_parameter('z_exclude_margin').value)
        self.min_area_m2 = float(self.get_parameter('min_area_m2').value)
        self.max_diameter_m = float(self.get_parameter('max_diameter_m').value)
        self.room_half_x = float(self.get_parameter('room_size_x').value) / 2.0
        self.room_half_y = float(self.get_parameter('room_size_y').value) / 2.0
        self.room_height = float(self.get_parameter('room_height').value)

        # 持久累积字典：(ix, iy) -> [z_min, z_max]。跨消息保留，飞机飞过
        # 哪块区域，哪块区域的格子就会被更新/加进来，不会因为局部地图
        # 窗口移走了就丢失之前见过的结构。
        self._cells = {}

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(PoseArray, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'订阅"{input_topic}"累积occupancy，min_area_m2={self.min_area_m2:.3f}'
            f'（立柱≈{pillar_area:.3f}，独立障碍物圆柱≈{cylinder_area:.3f}），'
            f'候选立柱发布到"{output_topic}"'
        )

    def _on_cloud(self, msg: PointCloud2):
        points = point_cloud2.read_points(msg, skip_nans=True)
        if len(points) == 0:
            return
        xs = np.asarray(points['x'], dtype=np.float64)
        ys = np.asarray(points['y'], dtype=np.float64)
        zs = np.asarray(points['z'], dtype=np.float64)

        # 房间边界裁剪 + 排除贴地板/贴天花板的点（见文件头说明，避免
        # 地板+天花板被误判成"贯穿房间高度的立柱"）。
        mask = (
            (xs >= -self.room_half_x) & (xs <= self.room_half_x)
            & (ys >= -self.room_half_y) & (ys <= self.room_half_y)
            & (zs >= self.z_margin) & (zs <= self.room_height - self.z_margin)
        )
        xs, ys, zs = xs[mask], ys[mask], zs[mask]
        if len(xs) == 0:
            return

        ix = np.floor(xs / self.res).astype(np.int64) + _CELL_KEY_OFFSET
        iy = np.floor(ys / self.res).astype(np.int64) + _CELL_KEY_OFFSET
        keys = ix * _CELL_KEY_MULT + iy

        uniq_keys, inverse = np.unique(keys, return_inverse=True)
        zmin_per_cell = np.full(len(uniq_keys), np.inf)
        zmax_per_cell = np.full(len(uniq_keys), -np.inf)
        np.minimum.at(zmin_per_cell, inverse, zs)
        np.maximum.at(zmax_per_cell, inverse, zs)

        uniq_ix = (uniq_keys // _CELL_KEY_MULT) - _CELL_KEY_OFFSET
        uniq_iy = (uniq_keys % _CELL_KEY_MULT) - _CELL_KEY_OFFSET

        for k in range(len(uniq_keys)):
            cell = (int(uniq_ix[k]), int(uniq_iy[k]))
            zmin, zmax = float(zmin_per_cell[k]), float(zmax_per_cell[k])
            if cell in self._cells:
                old_min, old_max = self._cells[cell]
                self._cells[cell] = (min(old_min, zmin), max(old_max, zmax))
            else:
                self._cells[cell] = (zmin, zmax)

    def _on_timer(self):
        tall_cells = {
            cell for cell, (zmin, zmax) in self._cells.items()
            if (zmax - zmin) >= self.height_span_min
        }
        if not tall_cells:
            return

        components = self._connected_components(tall_cells)
        candidates = []
        for comp in components:
            ixs = [c[0] for c in comp]
            iys = [c[1] for c in comp]
            width_x = (max(ixs) - min(ixs) + 1) * self.res
            width_y = (max(iys) - min(iys) + 1) * self.res
            diameter = max(width_x, width_y)
            area = len(comp) * self.res * self.res

            if diameter > self.max_diameter_m:
                continue  # 太长太扁，基本是墙，不是候选立柱
            if area < self.min_area_m2:
                continue  # 横截面积太小，判定是独立障碍物圆柱，不是候选立柱

            center_x = (min(ixs) + max(ixs) + 1) / 2.0 * self.res
            center_y = (min(iys) + max(iys) + 1) / 2.0 * self.res
            yaw = self._estimate_component_yaw(comp)
            candidates.append((center_x, center_y, diameter, area, yaw))

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'  # 跟grid_map/frame_id数值上一致（本项目map/odom数值重合的既有假设，见ego_planner_docker_sim.launch.py）
        for cx, cy, diameter, area, yaw in candidates:
            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.position.z = 0.0
            p.orientation.z = math.sin(yaw / 2.0)
            p.orientation.w = math.cos(yaw / 2.0)
            out.poses.append(p)
        self.pub.publish(out)

        if candidates:
            self.get_logger().info(
                f'候选立柱x{len(candidates)}: ' +
                ', '.join(
                    f'({cx:.2f},{cy:.2f}) d={d:.2f}m area={a:.2f}m2 yaw={math.degrees(yaw):.0f}°'
                    for cx, cy, d, a, yaw in candidates
                ),
                throttle_duration_sec=5.0,
            )

    def _estimate_component_yaw(self, comp: list) -> float:
        """拟合一个连通域(格子坐标列表)的朝向——见文件头说明，用
        `cv2.minAreaRect`而不是PCA。返回值本身受正方形4重对称影响，
        只是0/90/180/270度里的某一个代表值，不是"真实唯一"的朝向，
        调用方（`fire_pillar_aim_node.py`）会再吸附到最近的候选面，
        这里不需要、也做不到消除这个模糊性。"""
        pts = np.array([[cx * self.res, cy * self.res] for cx, cy in comp], dtype=np.float32)
        (_, _), (_, _), angle_deg = cv2.minAreaRect(pts)
        return math.radians(float(angle_deg))

    @staticmethod
    def _connected_components(cells: set) -> list:
        remaining = set(cells)
        components = []
        while remaining:
            start = next(iter(remaining))
            remaining.discard(start)
            comp = [start]
            queue = deque([start])
            while queue:
                cx, cy = queue.popleft()
                for dx, dy in _NEIGHBORS_4:
                    nb = (cx + dx, cy + dy)
                    if nb in remaining:
                        remaining.discard(nb)
                        comp.append(nb)
                        queue.append(nb)
            components.append(comp)
        return components


def main(args=None):
    rclpy.init(args=args)
    node = PillarDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
