"""`mission_geometry.py`的单元测试——纯Python，不需要ROS2环境，
`python3 test_mission_geometry.py`直接跑。"""
import math
import unittest

from mission_geometry import (
    interpolate_waypoints,
    select_nearest_pillar_index,
    sort_pillars_deterministic,
)


class TestSortPillarsDeterministic(unittest.TestCase):
    def test_sorts_by_x_then_y(self):
        pillars = [(4.0, 4.0), (0.0, 8.0), (-6.0, 2.0)]
        self.assertEqual(
            sort_pillars_deterministic(pillars),
            [(-6.0, 2.0), (0.0, 8.0), (4.0, 4.0)],
        )

    def test_ties_on_x_break_by_y(self):
        pillars = [(1.0, 5.0), (1.0, 2.0), (1.0, 8.0)]
        self.assertEqual(
            sort_pillars_deterministic(pillars),
            [(1.0, 2.0), (1.0, 5.0), (1.0, 8.0)],
        )

    def test_stable_and_repeatable(self):
        pillars = [(4.0, 4.0), (0.0, 8.0), (-6.0, 2.0)]
        self.assertEqual(
            sort_pillars_deterministic(pillars),
            sort_pillars_deterministic(list(reversed(pillars))),
        )


class TestSelectNearestPillarIndex(unittest.TestCase):
    def test_picks_nearest(self):
        sorted_pillars = [(-6.0, 2.0), (0.0, 8.0), (4.0, 4.0)]
        # 离(-5,2)最近的显然是下标0(-6,2)
        self.assertEqual(
            select_nearest_pillar_index((-5.0, 2.0), sorted_pillars), 0)

    def test_tie_breaks_to_smaller_index(self):
        # 方案4.3.2节给的具体例子：立柱1#(4,4)、3#(-6,2)到地面火情点
        # (0,-2)的距离精确相等(√52)。排序后1#/3#分别是下标2/0——
        # 这里直接验证"距离确实相等"+"平局时选下标小的"。
        sorted_pillars = sort_pillars_deterministic(
            [(4.0, 4.0), (0.0, 8.0), (-6.0, 2.0)])
        ground_fire_point = (0.0, -2.0)
        idx_pillar_1 = sorted_pillars.index((4.0, 4.0))
        idx_pillar_3 = sorted_pillars.index((-6.0, 2.0))
        dist_1 = math.hypot(4.0 - 0.0, 4.0 - (-2.0))
        dist_3 = math.hypot(-6.0 - 0.0, 2.0 - (-2.0))
        self.assertAlmostEqual(dist_1, dist_3, places=9)
        self.assertAlmostEqual(dist_1, math.sqrt(52), places=9)

        selected = select_nearest_pillar_index(ground_fire_point, sorted_pillars)
        self.assertEqual(selected, min(idx_pillar_1, idx_pillar_3))

    def test_skips_visited(self):
        sorted_pillars = [(-6.0, 2.0), (0.0, 8.0), (4.0, 4.0)]
        selected = select_nearest_pillar_index(
            (-5.0, 2.0), sorted_pillars, visited_indices=(0,))
        self.assertNotEqual(selected, 0)

    def test_raises_when_all_visited(self):
        sorted_pillars = [(-6.0, 2.0), (0.0, 8.0), (4.0, 4.0)]
        with self.assertRaises(ValueError):
            select_nearest_pillar_index(
                (0.0, 0.0), sorted_pillars, visited_indices=(0, 1, 2))


class TestInterpolateWaypoints(unittest.TestCase):
    def test_short_distance_returns_single_endpoint(self):
        result = interpolate_waypoints((0.0, 0.0, 1.5), (2.0, 0.0, 1.5), max_step_m=3.0)
        self.assertEqual(result, [(2.0, 0.0, 1.5)])

    def test_long_distance_splits_into_steps_under_max(self):
        start = (7.0, -10.0, 1.5)
        end = (7.0, 10.0, 1.5)  # 距离20米，方案4.2节的实际用例
        result = interpolate_waypoints(start, end, max_step_m=3.0)
        self.assertEqual(result[-1], end)
        prev = start
        for point in result:
            step = math.dist(prev, point)
            self.assertLessEqual(step, 3.0 + 1e-9)
            prev = point

    def test_zero_distance_returns_endpoint_not_empty(self):
        result = interpolate_waypoints((1.0, 1.0, 1.5), (1.0, 1.0, 1.5), max_step_m=3.0)
        self.assertEqual(result, [(1.0, 1.0, 1.5)])

    def test_rejects_non_positive_max_step(self):
        with self.assertRaises(ValueError):
            interpolate_waypoints((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), max_step_m=0.0)


if __name__ == '__main__':
    unittest.main()
