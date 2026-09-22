"""`geometry_helpers.py`的独立单元测试（B3清单/方案第5节验收要求："至少给
`generate_orbit_waypoints`/`generate_ground_scan_waypoints`这两个纯几何函数
...写独立的、不需要真实ROS2环境的单元测试"）。

`geometry_helpers.py`本身`import math`之外不依赖任何东西（不`import rclpy`/
不`import`任何ROS消息类型），这里的测试同样全程不需要ROS2环境，直接
`python3 test_geometry_helpers.py`（或`python3 -m unittest
test_geometry_helpers`）就能跑。
"""
import math
import os
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_PARENT_DIR = os.path.dirname(_THIS_DIR)
if _PACKAGE_PARENT_DIR not in sys.path:
    sys.path.insert(0, _PACKAGE_PARENT_DIR)

from contest_sdk.geometry_helpers import goto_stalled, pull_waypoints_out_of_circles  # noqa: E402
from contest_sdk.geometry_helpers import (  # noqa: E402
    angle_from_center_to_point,
    compute_ground_footprint,
    generate_ground_scan_waypoints,
    generate_orbit_waypoints,
)


class TestGenerateOrbitWaypoints(unittest.TestCase):
    def test_basic_circle_radius_and_count(self):
        """生成的每个点到圆心的距离都应该恰好等于radius，且z全部一致。"""
        pts = generate_orbit_waypoints(center_x=10.0, center_y=-5.0, radius=3.0, z=2.5, num_points=8)
        self.assertEqual(len(pts), 8)
        for x, y, z in pts:
            self.assertAlmostEqual(math.hypot(x - 10.0, y - (-5.0)), 3.0, places=9)
            self.assertAlmostEqual(z, 2.5)

    def test_default_num_points_is_16(self):
        pts = generate_orbit_waypoints(0.0, 0.0, 1.0, 0.0)
        self.assertEqual(len(pts), 16)

    def test_first_point_at_start_angle(self):
        pts = generate_orbit_waypoints(0.0, 0.0, 2.0, 0.0, num_points=4, start_angle_rad=0.0)
        self.assertAlmostEqual(pts[0][0], 2.0, places=9)
        self.assertAlmostEqual(pts[0][1], 0.0, places=9)

    def test_counterclockwise_default_direction(self):
        """默认逆时针：从角度0开始，下一个点角度应该增大（y从0变正）。"""
        pts = generate_orbit_waypoints(0.0, 0.0, 1.0, 0.0, num_points=4, start_angle_rad=0.0, clockwise=False)
        self.assertGreater(pts[1][1], 0.0)

    def test_clockwise_direction_flips_sign(self):
        pts = generate_orbit_waypoints(0.0, 0.0, 1.0, 0.0, num_points=4, start_angle_rad=0.0, clockwise=True)
        self.assertLess(pts[1][1], 0.0)

    def test_no_duplicate_endpoint(self):
        """首尾不重复——最后一个点不应该跟第一个点坐标相同。"""
        pts = generate_orbit_waypoints(0.0, 0.0, 1.0, 0.0, num_points=6)
        self.assertNotEqual(pts[0], pts[-1])

    def test_num_points_below_3_raises(self):
        with self.assertRaises(ValueError):
            generate_orbit_waypoints(0.0, 0.0, 1.0, 0.0, num_points=2)

    def test_non_positive_radius_raises(self):
        with self.assertRaises(ValueError):
            generate_orbit_waypoints(0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            generate_orbit_waypoints(0.0, 0.0, -1.0, 0.0)


class TestAngleFromCenterToPoint(unittest.TestCase):
    def test_cardinal_directions(self):
        self.assertAlmostEqual(angle_from_center_to_point(0, 0, 1, 0), 0.0, places=9)
        self.assertAlmostEqual(angle_from_center_to_point(0, 0, 0, 1), math.pi / 2, places=9)
        self.assertAlmostEqual(abs(angle_from_center_to_point(0, 0, -1, 0)), math.pi, places=9)

    def test_roundtrip_with_orbit_waypoints(self):
        """算出来的角度喂回generate_orbit_waypoints的start_angle_rad，
        应该能重新生成出跟原始点一致的第一个航点（验证两个函数的角度
        约定是一致的，都是以+x方向为0、逆时针为正）。"""
        cx, cy, r = 5.0, 5.0, 4.0
        point_x, point_y = cx + r * math.cos(1.234), cy + r * math.sin(1.234)
        angle = angle_from_center_to_point(cx, cy, point_x, point_y)
        pts = generate_orbit_waypoints(cx, cy, r, 0.0, num_points=6, start_angle_rad=angle)
        self.assertAlmostEqual(pts[0][0], point_x, places=6)
        self.assertAlmostEqual(pts[0][1], point_y, places=6)


class TestComputeGroundFootprint(unittest.TestCase):
    def test_higher_altitude_means_larger_footprint(self):
        w1, h1 = compute_ground_footprint(altitude_agl=2.0, hfov_rad=1.3963)
        w2, h2 = compute_ground_footprint(altitude_agl=4.0, hfov_rad=1.3963)
        self.assertGreater(w2, w1)
        self.assertGreater(h2, h1)
        # 针孔相机模型：覆盖宽度跟高度成正比（高度翻倍，覆盖宽高也翻倍）。
        self.assertAlmostEqual(w2 / w1, 2.0, places=6)
        self.assertAlmostEqual(h2 / h1, 2.0, places=6)

    def test_invalid_hfov_raises(self):
        with self.assertRaises(ValueError):
            compute_ground_footprint(altitude_agl=2.0, hfov_rad=0.0)
        with self.assertRaises(ValueError):
            compute_ground_footprint(altitude_agl=2.0, hfov_rad=math.pi)

    def test_invalid_altitude_raises(self):
        with self.assertRaises(ValueError):
            compute_ground_footprint(altitude_agl=0.0, hfov_rad=1.0)


class TestGenerateGroundScanWaypoints(unittest.TestCase):
    def test_boustrophedon_pattern_covers_room_with_margin(self):
        """所有航点都应该落在"房间边界收缩wall_margin"之后的矩形内部。"""
        pts = generate_ground_scan_waypoints(
            room_min_x=-10.0, room_max_x=10.0, room_min_y=-6.0, room_max_y=6.0,
            altitude_agl=3.0, wall_margin=1.0,
        )
        self.assertGreater(len(pts), 0)
        for x, y, z in pts:
            self.assertGreaterEqual(x, -10.0 + 1.0 - 1e-6)
            self.assertLessEqual(x, 10.0 - 1.0 + 1e-6)
            self.assertGreaterEqual(y, -6.0 + 1.0 - 1e-6)
            self.assertLessEqual(y, 6.0 - 1.0 + 1e-6)
            self.assertAlmostEqual(z, 3.0)

    def test_alternating_left_right_direction(self):
        """弓字形：第一行从x_min到x_max，第二行应该反过来从x_max到x_min。"""
        pts = generate_ground_scan_waypoints(
            room_min_x=0.0, room_max_x=20.0, room_min_y=0.0, room_max_y=20.0,
            altitude_agl=2.0, wall_margin=1.0, overlap_ratio=0.3,
        )
        # 第一行两个点：(x0,y0)->(x1,y0)
        self.assertLess(pts[0][0], pts[1][0])
        # 找到y变化的地方（第二行开始），方向应该反过来。
        first_y = pts[0][1]
        second_row_start = next(i for i, p in enumerate(pts) if p[1] != first_y)
        self.assertGreater(pts[second_row_start][0], pts[second_row_start + 1][0])

    def test_last_row_reaches_far_boundary(self):
        """最后一行的y坐标应该贴到room_max_y-wall_margin（不会因为
        row_spacing没对齐边界而漏掉最后一条）。"""
        pts = generate_ground_scan_waypoints(
            room_min_x=0.0, room_max_x=20.0, room_min_y=0.0, room_max_y=20.0,
            altitude_agl=2.0, wall_margin=1.0, overlap_ratio=0.3,
        )
        max_y_reached = max(p[1] for p in pts)
        self.assertAlmostEqual(max_y_reached, 20.0 - 1.0, places=6)

    def test_wall_margin_too_large_raises(self):
        with self.assertRaises(ValueError):
            generate_ground_scan_waypoints(
                room_min_x=0.0, room_max_x=2.0, room_min_y=0.0, room_max_y=2.0,
                altitude_agl=2.0, wall_margin=5.0,
            )

    def test_overlap_ratio_out_of_range_raises(self):
        with self.assertRaises(ValueError):
            generate_ground_scan_waypoints(
                room_min_x=0.0, room_max_x=20.0, room_min_y=0.0, room_max_y=20.0,
                altitude_agl=2.0, overlap_ratio=1.0,
            )
        with self.assertRaises(ValueError):
            generate_ground_scan_waypoints(
                room_min_x=0.0, room_max_x=20.0, room_min_y=0.0, room_max_y=20.0,
                altitude_agl=2.0, overlap_ratio=-0.1,
            )


class TestGotoStalled(unittest.TestCase):
    """goto() 卡住检测（2026-09-21）。"""

    W, MOVE, DIST = 5.0, 0.5, 0.5

    def _hist(self, pts, dt=0.2):
        return [(i * dt, x, y, z) for i, (x, y, z) in enumerate(pts)]

    def test_normal_flight_not_stalled(self):
        # 以 1m/s 朝 x 飞，5 秒走 5 米，不算卡住
        h = self._hist([(0.2 * i, 0.0, 1.5) for i in range(40)])
        self.assertFalse(goto_stalled(h, (20.0, 0.0, 1.5), self.W, self.MOVE, self.DIST))

    def test_stuck_at_obstacle_edge_is_stalled(self):
        # 目标在障碍圆柱正中心(7,0)，飞机停在它边上 0.9 米处不动——
        # 这正是"目标点在障碍物里"的典型样子
        h = self._hist([(6.1, 0.0, 1.5)] * 40)
        self.assertTrue(goto_stalled(h, (7.0, 0.0, 1.5), self.W, self.MOVE, self.DIST))

    def test_threshold_1m_would_miss_the_typical_case(self):
        # 说明为什么 min_dist 不能取 1 米：障碍圆柱半径0.25+膨胀0.6，
        # 飞机停在约 0.9 米处，门限 1 米会把它当成"已经很近了"放过
        h = self._hist([(6.1, 0.0, 1.5)] * 40)
        self.assertFalse(goto_stalled(h, (7.0, 0.0, 1.5), self.W, self.MOVE, 1.0))

    def test_hovering_at_goal_not_stalled(self):
        # 已经到了（离目标 < 0.5 米）只是 waypoint_state 还没更新，不算卡住
        h = self._hist([(6.8, 0.0, 1.5)] * 40)
        self.assertFalse(goto_stalled(h, (7.0, 0.0, 1.5), self.W, self.MOVE, self.DIST))

    def test_position_noise_does_not_hide_stall(self):
        # 位置有 ±0.1 米抖动（实测量级），5 秒内来回晃但没有真的前进
        import random
        random.seed(1)
        h = self._hist([(6.1 + random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), 1.5)
                        for _ in range(40)])
        self.assertTrue(goto_stalled(h, (7.0, 0.0, 1.5), self.W, self.MOVE, self.DIST))

    def test_pure_climb_not_stalled(self):
        # 只升高度（x、y 不变）——按水平位移算会误判成卡住，所以必须用三维
        h = self._hist([(0.0, 0.0, 0.5 + 0.1 * i) for i in range(40)])
        self.assertFalse(goto_stalled(h, (0.0, 0.0, 6.0), self.W, self.MOVE, self.DIST))

    def test_not_enough_history(self):
        # 刚起步（采样不满一个窗口）一律不算卡住——规划器还在出轨迹
        h = self._hist([(0.0, 0.0, 1.5)] * 10)   # 只有 1.8 秒
        self.assertFalse(goto_stalled(h, (10.0, 0.0, 1.5), self.W, self.MOVE, self.DIST))

    def test_detour_not_stalled(self):
        # 绕障碍物时离目标的距离会先变大，但飞机在动，不能判成卡住
        h = self._hist([(5.0, 0.2 * i, 1.5) for i in range(40)])
        self.assertFalse(goto_stalled(h, (8.0, 0.0, 1.5), self.W, self.MOVE, self.DIST))


class TestPullWaypointsOutOfCircles(unittest.TestCase):
    """把落进已知立柱（含余量）里的航点沿来路挪出来（2026-09-21）。"""

    PILLAR = (4.0, 4.0, 0.42)   # 边长0.6米的方立柱，半对角线0.42

    def test_outside_untouched(self):
        wps = [(0.0, 0.0, 1.5), (2.0, 0.0, 1.5)]
        self.assertEqual(pull_waypoints_out_of_circles(wps, [self.PILLAR], 0.8), wps)

    def test_endpoint_inside_pulled_back_along_lane(self):
        # 扫描线 (0,4)->(4,4)，端点正好在立柱中心
        wps = [(0.0, 4.0, 1.5), (4.0, 4.0, 1.5)]
        out = pull_waypoints_out_of_circles(wps, [self.PILLAR], 0.8)
        self.assertEqual(len(out), 2)
        x, y, z = out[1]
        self.assertAlmostEqual(y, 4.0, places=6)                    # 仍在原扫描线上
        self.assertAlmostEqual(z, 1.5, places=6)
        self.assertAlmostEqual(4.0 - x, 0.42 + 0.8, places=4)      # 刚好退到圈外
        self.assertLess(x, 4.0)                                      # 是往回退，不是穿过去

    def test_clearance_covers_real_inflation(self):
        # 挪出来的点到立柱中心的距离必须 >= 半对角线 + 余量，否则规划器照样到不了
        wps = [(0.0, 3.5, 1.5), (4.3, 3.9, 1.5)]
        out = pull_waypoints_out_of_circles(wps, [self.PILLAR], 0.8)
        x, y, _ = out[1]
        self.assertGreaterEqual(math.hypot(x - 4.0, y - 4.0), 0.42 + 0.8 - 1e-4)

    def test_first_waypoint_uses_next_segment(self):
        # 第一个航点在圈里：没有"上一个"，沿它和下一个航点那条线段挪
        wps = [(4.0, 4.0, 1.5), (0.0, 4.0, 1.5)]
        out = pull_waypoints_out_of_circles(wps, [self.PILLAR], 0.8)
        self.assertEqual(len(out), 2)
        self.assertGreaterEqual(math.hypot(out[0][0] - 4.0, out[0][1] - 4.0), 1.22 - 1e-4)

    def test_whole_segment_inside_dropped(self):
        # 参照点也在圈里（整段都在障碍物上）：丢掉，那块地本来就被占着
        wps = [(4.1, 4.0, 1.5), (4.0, 4.1, 1.5)]
        out = pull_waypoints_out_of_circles(wps, [self.PILLAR], 0.8)
        self.assertEqual(out, [])

    def test_does_not_modify_input(self):
        wps = [(0.0, 4.0, 1.5), (4.0, 4.0, 1.5)]
        before = list(wps)
        pull_waypoints_out_of_circles(wps, [self.PILLAR], 0.8)
        self.assertEqual(wps, before)

    def test_real_lawnmower_with_three_pillars(self):
        # 用真的弓字航线 + 题目给的3根立柱。注意：弓字航线沿 x 来回扫，端点
        # 只落在左右边界上，立柱在场地内部时根本不会有端点落进去（那种情况
        # 靠规划器绕行，不需要挪点）。所以这里故意把右边界设成让端点 x=4.0，
        # 正好压在立柱 (4,4) 那一列上，先确认输入里**确实有**违规点，否则
        # 这个测试什么都没验证。
        pillars = [(4.0, 4.0, 0.42), (0.0, 8.0, 0.42), (-5.0, 2.0, 0.42)]
        wps = generate_ground_scan_waypoints(
            room_min_x=-10.0, room_max_x=5.0, room_min_y=-12.5, room_max_y=12.5,
            altitude_agl=1.5)

        def violations(points):
            return sum(1 for x, y, _ in points for cx, cy, r in pillars
                       if math.hypot(x - cx, y - cy) < r + 0.8 - 1e-4)

        self.assertGreater(violations(wps), 0, '输入里没有违规点，测试没有意义')
        out = pull_waypoints_out_of_circles(wps, pillars, 0.8)
        self.assertEqual(violations(out), 0)

if __name__ == '__main__':
    unittest.main()
