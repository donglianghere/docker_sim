#!/usr/bin/env python3
"""SE(2)在线标定估计器的**纯函数版核心**——真机侧(L3)实验专用。

跟`scripts/paper_exp/se2_core.py`是同一份代码（仿真侧实验用的那份），
复制到这里是为了让真机侧这套工具不依赖仿真侧目录也能独立使用。

为什么单独抽一个文件：论文里要做的数值实验(mc_se2_sim.py)和录包离线复算
(se2_calib_offline_eval.py)必须跟机上真正跑的
`src/uwb_origin_bridge/uwb_origin_bridge/origin_setter_node.py`用**同一套**
分段/滑窗/读数逻辑，否则"实验验证的是论文里那个估计器"这句话就不成立。
但origin_setter_node是个ROS节点(要rclpy、要订阅话题)，不能直接import到
宿主机上跑几千次蒙特卡洛。这里把算法部分原样抄成无依赖的纯函数/纯类，
抄的时候逐行对照了原节点，对应关系写在每个函数的docstring里。

⚠️ 这个文件跟origin_setter_node.py是"同一份逻辑的两个副本"，改任何一边
都要同步改另一边，否则实验结论跟机上实际行为会悄悄脱节。
"""
import math
from collections import deque


def wrap_pi(a):
    """把角度归一化到(-pi, pi]，算角度误差前必须过一遍，否则359°和-1°会
    被当成358°的误差。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def solve_theta(segments, weights=None):
    """闭式解θ*=atan2(S,C)——论文式(6)(7)，权重版是式(18)。

    segments: [(ux, uy, vx, vy), ...]
    weights:  None(等权，退化成式(6)) 或 与segments等长的权重列表。
    """
    S = 0.0
    C = 0.0
    for i, (ux, uy, vx, vy) in enumerate(segments):
        w = 1.0 if weights is None else weights[i]
        S += w * (ux * vy - uy * vx)
        C += w * (ux * vx + uy * vy)
    if S == 0.0 and C == 0.0:
        return 0.0
    return math.atan2(S, C)


def residuals(segments, theta):
    """每段在给定θ下的残差模长——论文式(16)。"""
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for ux, uy, vx, vy in segments:
        rx = vx - (ux * c - uy * s)
        ry = vy - (ux * s + uy * c)
        out.append(math.hypot(rx, ry))
    return out


def _median(xs):
    ys = sorted(xs)
    n = len(ys)
    if n == 0:
        return 0.0
    return ys[n // 2] if n % 2 else 0.5 * (ys[n // 2 - 1] + ys[n // 2])


def huber_weights(res, k_scale=3.0, eps=1e-9):
    """Huber权重——论文式(17)，k按滑窗内残差的MAD设定(设计取k=3×MAD)。

    返回 (weights, k)。全部残差都小于k时权重恒为1，此时式(18)退化成式(6)，
    跟论文里"正常数据下w_i=1，退化为原始闭式解"这句对应。
    """
    med = _median(res)
    mad = _median([abs(r - med) for r in res])
    k = k_scale * mad
    if k < eps:                       # 残差几乎全相同(如无噪声合成数据)，不降权
        return [1.0] * len(res), k
    return [1.0 if r <= k else k / r for r in res], k


def solve_theta_irls(segments, n_iter=3, k_scale=3.0, theta_init=None):
    """迭代重加权最小二乘(论文2.7节)：用上一轮θ算残差→定权重→重解θ。

    n_iter=0 等价于不做抗差，直接返回式(7)的结果。
    """
    theta = solve_theta(segments) if theta_init is None else theta_init
    k = float('inf')
    for _ in range(n_iter):
        w, k = huber_weights(residuals(segments, theta), k_scale=k_scale)
        theta = solve_theta(segments, w)
    return theta, k


class SlidingWindowEstimator:
    """滑窗累加器——对应origin_setter_node里
    `_maybe_record_segment`/`_add_segment`的分段+滑窗+O(1)增量更新逻辑
    (论文2.5节)。这里不做TF广播/持久化，只保留估计本身。

    robust=True时每次读数走一遍IRLS(论文2.7节)，代价是不能再用增量的
    S/C，必须遍历窗口内W段——论文里说的"计算成本仍为滑窗内的O(W)操作"。
    """

    def __init__(self, d_min=0.5, window=50, min_segments=1,
                 robust=False, huber_iters=3, k_scale=3.0):
        self.d_min = d_min
        self.window = window
        self.min_segments = min_segments
        self.robust = robust
        self.huber_iters = huber_iters
        self.k_scale = k_scale
        self.segments = deque()
        self._S = 0.0
        self._C = 0.0
        self.theta = 0.0
        self.anchor_local = None
        self.anchor_global = None
        self.n_total_segments = 0     # 累计记过多少段(不受滑窗滚动影响)

    # --- 对应 _maybe_record_segment ---------------------------------
    def update(self, local_xy, global_xy):
        """喂一帧(局部位置, 全局位置)。返回True表示这一帧切出了新段。

        有限性校验对应论文3.1节那个NaN卡死问题的修复：任一输入非有限时
        当作"没有可用锚点"重设，而不是让NaN污染锚点后永久卡死。
        """
        finite = (all(map(math.isfinite, local_xy))
                  and all(map(math.isfinite, global_xy)))
        if (self.anchor_local is None
                or not all(map(math.isfinite, self.anchor_local))
                or not all(map(math.isfinite, self.anchor_global))
                or not finite):
            if finite:
                self.anchor_local = tuple(local_xy)
                self.anchor_global = tuple(global_xy)
            return False
        ux = local_xy[0] - self.anchor_local[0]
        uy = local_xy[1] - self.anchor_local[1]
        if math.hypot(ux, uy) < self.d_min:
            return False
        vx = global_xy[0] - self.anchor_global[0]
        vy = global_xy[1] - self.anchor_global[1]
        self.add_segment(ux, uy, vx, vy)
        # 段首尾相接：这一段的终点就是下一段的起点
        self.anchor_local = tuple(local_xy)
        self.anchor_global = tuple(global_xy)
        return True

    # --- 对应 _add_segment ------------------------------------------
    def add_segment(self, ux, uy, vx, vy):
        self.segments.append((ux, uy, vx, vy))
        self._S += ux * vy - uy * vx
        self._C += ux * vx + uy * vy
        self.n_total_segments += 1
        if len(self.segments) > self.window:
            old = self.segments.popleft()
            self._S -= old[0] * old[3] - old[1] * old[2]
            self._C -= old[0] * old[2] + old[1] * old[3]
        if len(self.segments) < self.min_segments:
            return
        if self.robust:
            self.theta, _ = solve_theta_irls(
                list(self.segments), n_iter=self.huber_iters,
                k_scale=self.k_scale, theta_init=self.theta)
        else:
            self.theta = math.atan2(self._S, self._C)

    @property
    def n_segments(self):
        return len(self.segments)


class BatchEstimator(SlidingWindowEstimator):
    """批量(全历史)最小二乘对照组——论文4.1节/表3第1项要对比的基线。

    实现上就是把滑窗设成无穷大：所有历史段一直累加，永不滑出。
    """

    def __init__(self, d_min=0.5, **kw):
        kw.pop('window', None)
        super().__init__(d_min=d_min, window=10 ** 9, **kw)
