#!/usr/bin/env python3
"""`LeaderPathBuffer.point_at_arc_length_behind()`的synthetic单元测试。

对应《2026大赛任务系统全流程任务可执行实施清单.md》A3项验收标准第1条
"先做synthetic单元测试"、方案第5节"用synthetic轨迹数据测试，不要直接
上真实双机飞行验证"这条建议——不依赖rclpy/ROS2环境，可以直接：

    python3 src/contest_mission/contest_mission/test_leader_path_buffer.py

跑通（也兼容`pytest`/`python3 -m unittest`发现执行）。

测试思路：喂三种典型轨迹形状（直线、直角拐弯、连续S弯），每种都用一个
跟实现完全独立的几何校验函数`polyline_arc_length_from_point_to_end()`
反向验证"回溯查到的点，沿折线走到终点的弧长，是否真的等于查询时给的
follow_distance_m"——这个校验函数自己重新做一遍"找点落在哪个线段上+
累加该段之后所有线段长度"的几何计算，不是直接照抄被测函数的实现，
避免"用同一套错误逻辑验证自己"这种没有意义的循环校验。

直角拐弯这个用例专门用来抓方案里强调过的坑："沿折线回溯"和"沿直线
抄近道回溯"在拐角附近算出的结果必然不同——测试里额外加了一个明显
写错的"直线抄近道"实现`_naive_straight_line_shortcut()`作对照，断言
正确实现在这个用例上的结果跟这个错误实现的结果明显不同（差距超过
一个不可能是浮点误差的阈值），证明测试本身确实有能力抓出"抄近道"
这类bug，不是随便碰巧也能通过。
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contest_mission.formation_follower_node import LeaderPathBuffer  # noqa: E402


def polyline_arc_length_from_point_to_end(points, point, tol=1e-3):
    """独立于被测实现的几何校验函数：在`points`（[(x,y), ...]，折线，
    索引0最老/索引-1最新）里找出`point`落在哪一段上，返回"从这个点
    沿折线走到最后一个点"的弧长。找不到（point不在折线的任何一段上）
    返回None。

    判定"point是否在某段上"：point到该段所在直线的垂距<tol，且投影
    参数落在[0,1]（容忍一点浮点误差）范围内。
    """
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg_dx, seg_dy = x1 - x0, y1 - y0
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-9:
            continue
        t = ((point[0] - x0) * seg_dx + (point[1] - y0) * seg_dy) / (seg_len ** 2)
        if -1e-3 <= t <= 1 + 1e-3:
            proj_x, proj_y = x0 + seg_dx * t, y0 + seg_dy * t
            perp = math.hypot(proj_x - point[0], proj_y - point[1])
            if perp < tol:
                t_clamped = min(max(t, 0.0), 1.0)
                dist = (1.0 - t_clamped) * seg_len
                for j in range(i + 1, len(points) - 1):
                    dist += math.hypot(points[j + 1][0] - points[j][0], points[j + 1][1] - points[j][1])
                return dist
    return None


def _naive_straight_line_shortcut(points, follow_distance_m):
    """故意写错的"抄近道"实现，只用来在测试里当反例对照，不是给节点
    用的——从终点沿"终点->起点方向的直线"后退follow_distance_m米，
    完全不管折线中间的拐弯。方案文档强调过的坑就是这种写法。"""
    x1, y1 = points[-1]
    x0, y0 = points[0]
    total = math.hypot(x1 - x0, y1 - y0)
    if total < 1e-9:
        return (x1, y1)
    frac = min(follow_distance_m / total, 1.0)
    return (x1 + (x0 - x1) * frac, y1 + (y0 - y1) * frac)


def _build_buffer(points, gap_eps=1e-6):
    """把一串(x,y)点喂进一个新LeaderPathBuffer，min_point_gap_m设成
    极小值——测试里直接控制点序列本身的密度，不需要LeaderPathBuffer
    的去重逻辑介入，避免测试用例的点被意外过滤掉。"""
    buf = LeaderPathBuffer(min_point_gap_m=gap_eps, max_buffer_length_m=1e9)
    for i, (x, y) in enumerate(points):
        buf.append(x, y, float(i))
    return buf


class TestStraightLine(unittest.TestCase):
    """轨迹形状1：直线。"""

    def setUp(self):
        # (0,0) -> (10,0)，均匀取点，总长10米。
        self.points = [(float(x), 0.0) for x in range(0, 11)]
        self.buf = _build_buffer(self.points)

    def test_exact_backtrack_matches_expected_point(self):
        result = self.buf.point_at_arc_length_behind(3.5)
        self.assertIsNotNone(result)
        # 直线上回溯3.5米，预期正好是(6.5, 0)。
        self.assertAlmostEqual(result[0], 6.5, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)

    def test_independent_arc_length_verifier_agrees(self):
        result = self.buf.point_at_arc_length_behind(3.5)
        arc = polyline_arc_length_from_point_to_end(self.points, result)
        self.assertIsNotNone(arc)
        self.assertAlmostEqual(arc, 3.5, delta=1e-3)

    def test_distance_exceeds_buffer_returns_earliest_point(self):
        # 总长只有10米，查20米应该兜底返回最老的点(0,0)。
        result = self.buf.point_at_arc_length_behind(20.0)
        self.assertAlmostEqual(result[0], 0.0, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=6)


class TestRightAngleTurn(unittest.TestCase):
    """轨迹形状2：直角拐弯——专门用来抓"沿折线回溯"vs"直线抄近道"
    这个坑（方案文档强调过的、之前版本设计错过一次的问题）。"""

    def setUp(self):
        # (0,0) -> (10,0)（沿x），再(10,0) -> (10,6)（沿y，直角拐弯）。
        # 每段2米一个点，总长10+6=16米。
        self.points = [
            (0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.0),
            (10.0, 2.0), (10.0, 4.0), (10.0, 6.0),
        ]
        self.buf = _build_buffer(self.points)

    def test_exact_backtrack_across_corner(self):
        # 手算：从终点(10,6)往回走，(10,6)-(10,4)=2，累计2；
        # (10,4)-(10,2)=2，累计4；(10,2)-(10,0)段需要remaining=1，
        # frac=0.5 -> (10, 2-1)=(10,1)。
        result = self.buf.point_at_arc_length_behind(5.0)
        self.assertAlmostEqual(result[0], 10.0, places=6)
        self.assertAlmostEqual(result[1], 1.0, places=6)

    def test_independent_arc_length_verifier_agrees(self):
        for d in (1.0, 3.0, 5.0, 7.5, 10.0, 15.9):
            result = self.buf.point_at_arc_length_behind(d)
            arc = polyline_arc_length_from_point_to_end(self.points, result)
            self.assertIsNotNone(arc, f'd={d}: 返回的点{result}没有落在折线的任何一段上')
            self.assertAlmostEqual(arc, d, delta=1e-3, msg=f'd={d}: 沿折线弧长校验不一致')

    def test_correct_result_differs_from_naive_straight_line_shortcut(self):
        """核心防回归用例：确认"沿折线回溯"跟"直线抄近道回溯"在拐角
        附近算出的结果明显不同——如果这个断言失败（两者结果几乎一样），
        说明被测实现可能退化成了直线抄近道，正是方案文档强调过的坑。
        """
        d = 5.0
        correct = self.buf.point_at_arc_length_behind(d)
        naive = _naive_straight_line_shortcut(self.points, d)
        dist_between = math.hypot(correct[0] - naive[0], correct[1] - naive[1])
        # 两者差距应该有实质量级（远大于任何浮点误差），这里用0.5米做
        # 门槛——已经比典型数值误差(1e-6量级)大了几十万倍，足够说明
        # 两种算法给出了本质不同的答案。
        self.assertGreater(
            dist_between, 0.5,
            f'正确结果{correct}跟直线抄近道结果{naive}差距只有{dist_between:.4f}米，'
            f'太接近了，怀疑被测实现退化成了直线抄近道',
        )


class TestSCurve(unittest.TestCase):
    """轨迹形状3：连续S弯（正弦曲线离散化成折线）。"""

    def setUp(self):
        # x从0到4π，y=2*sin(x)，步长0.05，构成一条来回摆动的连续S弯，
        # 点间距不均匀（sin曲线曲率变化的地方点与点之间的直线距离会
        # 略有不同），比前两个用例更接近真实飞行轨迹的不规则程度。
        n = 260
        self.points = []
        for i in range(n):
            x = i * (4.0 * math.pi / (n - 1))
            y = 2.0 * math.sin(x)
            self.points.append((x, y))
        self.buf = _build_buffer(self.points)

    def test_independent_arc_length_verifier_agrees_multiple_distances(self):
        total_len = polyline_arc_length_from_point_to_end(self.points, self.points[0])
        self.assertIsNotNone(total_len)
        for d in (0.5, 1.0, 3.5, 6.0, min(10.0, total_len - 0.1)):
            result = self.buf.point_at_arc_length_behind(d)
            self.assertIsNotNone(result)
            arc = polyline_arc_length_from_point_to_end(self.points, result)
            self.assertIsNotNone(arc, f'd={d}: 返回的点{result}没有落在折线的任何一段上')
            self.assertAlmostEqual(arc, d, delta=1e-3, msg=f'd={d}: 沿折线弧长校验不一致（S弯用例）')

    def test_result_point_is_not_a_straight_line_shortcut(self):
        """S弯上一个足够大的follow_distance会跨越好几个波峰波谷，沿
        折线回溯的点应该明显偏离"起点到终点直线"上对应比例的点——
        否则说明实现可能又退化成了直线插值。"""
        d = 10.0
        result = self.buf.point_at_arc_length_behind(d)
        naive = _naive_straight_line_shortcut(self.points, d)
        dist_between = math.hypot(result[0] - naive[0], result[1] - naive[1])
        self.assertGreater(
            dist_between, 0.3,
            f'S弯用例下正确结果{result}跟直线抄近道结果{naive}差距只有'
            f'{dist_between:.4f}米，怀疑退化成了直线抄近道',
        )


class TestEdgeCases(unittest.TestCase):
    def test_empty_buffer_returns_none(self):
        buf = LeaderPathBuffer()
        self.assertIsNone(buf.point_at_arc_length_behind(3.5))

    def test_single_point_buffer_returns_that_point(self):
        buf = LeaderPathBuffer()
        buf.append(1.23, 4.56, 0.0)
        result = buf.point_at_arc_length_behind(3.5)
        self.assertAlmostEqual(result[0], 1.23, places=6)
        self.assertAlmostEqual(result[1], 4.56, places=6)

    def test_min_point_gap_dedup(self):
        # 默认min_point_gap_m=0.05，间距远小于这个值的点应该被过滤，
        # 验证append()确实起到了防止缓冲区被高频odom灌爆的作用。
        buf = LeaderPathBuffer(min_point_gap_m=0.05)
        buf.append(0.0, 0.0, 0.0)
        buf.append(0.001, 0.0, 0.01)  # 远小于0.05米，应该被忽略
        buf.append(0.002, 0.0, 0.02)  # 同上
        self.assertEqual(len(buf), 1)
        buf.append(1.0, 0.0, 1.0)  # 超过0.05米，应该被接受
        self.assertEqual(len(buf), 2)

    def test_max_buffer_length_trims_old_points(self):
        buf = LeaderPathBuffer(min_point_gap_m=0.0, max_buffer_length_m=5.0)
        for i in range(20):
            buf.append(float(i), 0.0, float(i))
        # 总长上限5米，最新点在(19,0)，最老的点应该被裁剪到只剩下
        # 大约(14,0)附近（19-5=14），不会一直保留(0,0)。
        result = buf.point_at_arc_length_behind(100.0)  # 超出缓冲区总长，兜底返回最老点
        self.assertGreaterEqual(result[0], 14.0 - 1e-6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
