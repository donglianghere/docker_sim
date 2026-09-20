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


if __name__ == '__main__':
    unittest.main()
