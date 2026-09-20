#!/usr/bin/env python3
"""真机侧(L3)实验的统一分析工具——读rosbag，出指标，落JSON。

每个子命令对应文档《实验方案_SE2在线标定真机侧L3.md》里的一个实验：
    static  H1 静态观测噪声与野值率
    yaw     H2 双标签测向精度与基线标度律（式21）
    theta   H3 θ*在线标定精度（真值=双标签yaw−DLIO yaw）
    sweep   H4 参数敏感性/批量对比/时间配对（纯离线，复用H3的数据）
    robust  H5 抗差扩展（真实NLOS数据 + 可选人工注入野值）
    timing  H6 机载单次更新耗时

所有子命令都把结果写成 results/<实验号>_<标签>.json，最后用 l3_report.py 汇总。

⚠️ 时间基准：统一用**bag的接收时间戳**，不用header.stamp。这套系统里
DLIO与UWB两路数据不共享时钟域（DLIO活在仿真/假epoch时钟里、UWB节点用
系统墙钟），header.stamp跨话题不可比；而接收时间戳是录包进程的单一时钟，
正好也复刻了机上估计器"取当前最新一帧"的真实行为。

依赖：rosbag2_py + rclpy（容器里自带）；numpy/matplotlib可选（没有则跳过画图）。
"""
import argparse
import json
import math
import os
import sys
import time
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from l3_core import (SlidingWindowEstimator, BatchEstimator, wrap_pi,   # noqa: E402
                     solve_theta, solve_theta_irls, residuals)

RESULT_DIR = 'results'


# ------------------------------------------------------------------ 工具
def _quat_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_bag(bag_dir, ns='NX01'):
    """返回按话题分类的时间序列，时间基准为bag接收时间戳（秒）。

    位置类话题 -> [(t, x, y, z, yaw)]；Float64 -> [(t, v)]；Int32 -> [(t, v)]。
    话题不存在时对应的列表为空，调用方自己判断。
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    ns = ns.strip('/')
    want = {
        f'/{ns}/dlio/odom_node/odom': 'odom',
        f'/{ns}/uwb/pose_abs': 'abs',
        f'/{ns}/uwb_a/pose_abs': 'tag_a',
        f'/{ns}/uwb_b/pose_abs': 'tag_b',
        f'/{ns}/origin_setter/yaw_estimate': 'yaw_on',
        f'/{ns}/origin_setter/yaw_sample_count': 'nseg_on',
    }
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
    except RuntimeError:
        # 录制被强杀时 metadata.yaml 没写出来，自动识别就失效了；但 .db3 本身
        # 是完整的，显式指定 storage_id 并直接指向那个文件就能照常读。
        # （2026-09-03实测：ssh端的timeout掐断会话会连带杀掉docker exec里的
        #  录制进程，来不及写metadata。数据没丢，不用重录。）
        import glob
        cand = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
        if not cand:
            raise
        print(f'  （{os.path.basename(bag_dir)} 缺metadata.yaml，直接按sqlite3读 '
              f'{os.path.basename(cand[0])}）')
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=cand[0], storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = defaultdict(list)
    missing = [t for t in want if t not in types]
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        key = want.get(topic)
        if key is None:
            continue
        t = t_ns * 1e-9
        msg = deserialize_message(raw, get_message(types[topic]))
        if key in ('yaw_on', 'nseg_on'):
            out[key].append((t, float(msg.data)))
            continue
        if key == 'odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
        else:
            p, q = msg.pose.position, msg.pose.orientation
        out[key].append((t, p.x, p.y, p.z, _quat_yaw(q.x, q.y, q.z, q.w)))
    for k in out:
        out[k].sort(key=lambda r: r[0])
    out['_missing'] = missing
    return out


def _stats(v):
    n = len(v)
    if n == 0:
        return dict(n=0)
    m = sum(v) / n
    var = sum((x - m) ** 2 for x in v) / n
    s = sorted(v)
    return dict(n=n, mean=m, std=math.sqrt(var), median=s[n // 2],
                p95=s[int(0.95 * (n - 1))], min=s[0], max=s[-1])


def _mad(v):
    if not v:
        return 0.0, 0.0
    s = sorted(v)
    med = s[len(s) // 2]
    d = sorted(abs(x - med) for x in v)
    return med, d[len(d) // 2]


def _save(name, payload):
    os.makedirs(RESULT_DIR, exist_ok=True)
    path = os.path.join(RESULT_DIR, name + '.json')
    with open(path, 'w') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f'  -> {path}')
    return path


def _nearest(series, t):
    """在(t,...)序列里取时间最接近t的一条（二分）。"""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    if t <= series[0][0]:
        return series[0]
    if t >= series[-1][0]:
        return series[-1]
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    return series[lo] if abs(series[lo][0] - t) <= abs(series[hi][0] - t) else series[hi]


# ------------------------------------------------------------------ H1
def cmd_static(a):
    """H1：静止状态下的观测噪声、分布形状、野值率。

    指标：三轴样本标准差（论文表1的σ）、残差绝对中位差、超过"中位数+3MAD"
    的样本占比（野值率）、偏度与峰度（正态性的粗判据）。
    对同一场地的LOS/NLOS两次采集分别跑一次，用--tag区分。
    """
    d = read_bag(a.bag, a.ns)
    src = a.topic
    key = {'abs': 'abs', 'a': 'tag_a', 'b': 'tag_b'}[src]
    rows = d[key]
    if not rows:
        print(f'!! bag里没有{src}对应的话题，缺失清单: {d["_missing"]}')
        return 1
    t0 = rows[0][0]
    dur = rows[-1][0] - t0
    res = dict(bag=a.bag, tag=a.tag, topic=src, n=len(rows),
               duration_s=dur, rate_hz=len(rows) / dur if dur > 0 else 0.0)
    for i, axis in ((1, 'x'), (2, 'y'), (3, 'z')):
        v = [r[i] for r in rows]
        st = _stats(v)
        # 去掉均值后的残差（静止时均值就是真实位置的最优估计）
        e = [x - st['mean'] for x in v]
        # 野值判据用标准的"3倍稳健σ"：med/MAD 都取**带符号残差**的，再用
        # 1.4826 把 MAD 换算成正态等效标准差。
        # ⚠️ 不能像早先那样对 |e| 取 med+3·MAD 当阈值——那个阈值对纯高斯数据
        # 也会判出约10%的"野值"（med(|e|)=0.674σ、MAD(|e|)≈0.32σ，阈值只有
        # 1.63σ），会把干净数据误报成多径污染，正好把这个实验的结论搞反。
        med, mad = _mad(e)
        sigma_rob = 1.4826 * mad
        thr = 3.0 * sigma_rob
        n_out = sum(1 for x in e if abs(x - med) > thr) if sigma_rob > 0 else 0
        m2 = sum(x * x for x in e) / len(e)
        m3 = sum(x ** 3 for x in e) / len(e)
        m4 = sum(x ** 4 for x in e) / len(e)
        res[axis] = dict(mean=st['mean'], std=st['std'], max_abs_err=max(abs(x) for x in e),
                         mad=mad, sigma_robust=sigma_rob, outlier_thr=thr,
                         outlier_ratio=n_out / len(e),
                         skew=m3 / m2 ** 1.5 if m2 > 0 else 0.0,
                         kurtosis=m4 / m2 ** 2 if m2 > 0 else 0.0)
    res['sigma_xy'] = math.sqrt(0.5 * (res['x']['std'] ** 2 + res['y']['std'] ** 2))
    print(f"  样本{res['n']}帧 时长{dur:.1f}s 实测发布率{res['rate_hz']:.1f}Hz")
    for axis in 'xyz':
        r = res[axis]
        print(f"  {axis}: std={r['std']:.4f} m  最大偏差={r['max_abs_err']:.3f} m  "
              f"野值率={r['outlier_ratio']:.2%}  偏度={r['skew']:+.2f} 峰度={r['kurtosis']:.2f}")
    print(f"  水平合成 σ_xy = {res['sigma_xy']:.4f} m   （论文表1取0.05 m）")
    if max(res[ax]['max_abs_err'] for ax in 'xy') > 0.5:
        print('  ⚠️ 水平偏差超过0.5 m，这段数据大概率不是静止采集的——H1必须用'
              '完全静止的数据，否则算出来的"噪声"里混的是真实运动')
        res['warning'] = 'not_static'
    _save(f'H1_{a.tag}', res)
    return 0


# ------------------------------------------------------------------ H2
def _yaw_std_blockavg(pairs, M):
    """把连续M帧的两标签位置各自块平均后再算yaw，返回(样本数, yaw标准差[rad])。

    用途见cmd_yaw的说明：块平均把等效单站噪声从σ降到σ/√M，按式(21)
    σ_yaw应当同比例下降，于是 σ_yaw(M)·√M 应当是常数。这条检验只需要现有的
    固定基线，不用拆装标签换基线长度。
    """
    ys = []
    for i in range(0, len(pairs) - M + 1, M):
        blk = pairs[i:i + M]
        ax = sum(b[0] for b in blk) / M
        ay = sum(b[1] for b in blk) / M
        bx = sum(b[2] for b in blk) / M
        by = sum(b[3] for b in blk) / M
        ys.append(math.atan2(by - ay, bx - ax) + math.pi / 2)
    if len(ys) < 10:
        return len(ys), float('nan')
    ref = ys[len(ys) // 2]
    unw = [ref + wrap_pi(y - ref) for y in ys]
    m = sum(unw) / len(unw)
    return len(unw), math.sqrt(sum((x - m) ** 2 for x in unw) / len(unw))


def cmd_yaw(a):
    """H2：双标签测向精度与式(21)的基线标度律。

    从uwb_a/uwb_b两路原始位置**离线重算**yaw（不依赖机上dual_tag_fusion，
    这样同一份数据可以换基线、换配对方式重复分析）：
        yaw = normalize(atan2(dy, dx) + 90°)
    静止时的样本标准差就是σ_yaw，直接与式(21) √2σ/L 对照，不需要真值。
    多个基线长度各录一段，检查 σ_yaw × L 是否为常数（这是比对单点值更强的检验）。
    """
    d = read_bag(a.bag, a.ns)
    A, B = d['tag_a'], d['tag_b']
    if not A or not B:
        print(f'!! 缺uwb_a/uwb_b原始话题，缺失清单: {d["_missing"]}')
        return 1
    yaws, seps = [], []
    for ra in A:                       # 以a为基准，给每帧a配时间最近的b
        rb = _nearest(B, ra[0])
        if rb is None or abs(rb[0] - ra[0]) > a.max_dt:
            continue
        dx, dy = rb[1] - ra[1], rb[2] - ra[2]
        seps.append(math.hypot(dx, dy))
        yaws.append(wrap_pi(math.atan2(dy, dx) + math.pi / 2))
    if len(yaws) < 30:
        print(f'!! 有效配对样本太少({len(yaws)})，检查两路标签是否都在发布')
        return 1
    # 角度要在解缠后统计（避免±180°附近的绕回把标准差算爆）
    ref = yaws[len(yaws) // 2]
    unw = [ref + wrap_pi(y - ref) for y in yaws]
    st = _stats(unw)
    sep = _stats(seps)
    L = a.baseline if a.baseline > 0 else sep['mean']
    sigma = a.sigma
    pred = math.degrees(math.sqrt(2.0) * sigma / L)
    res = dict(bag=a.bag, tag=a.tag, n=len(unw), baseline_used_m=L,
               baseline_measured_mean_m=sep['mean'], baseline_measured_std_m=sep['std'],
               yaw_mean_deg=math.degrees(st['mean']), yaw_std_deg=math.degrees(st['std']),
               yaw_p95_abs_dev_deg=math.degrees(max(abs(x - st['mean']) for x in unw)),
               sigma_assumed_m=sigma, sigma_yaw_pred_deg=pred,
               ratio_meas_over_pred=math.degrees(st['std']) / pred if pred > 0 else 0.0,
               sigma_yaw_times_L=math.degrees(st['std']) * L)
    print(f"  有效样本{res['n']}  基线实测 {sep['mean']:.3f}±{sep['std']:.3f} m（取L={L:.3f}）")
    print(f"  yaw 均值 {res['yaw_mean_deg']:+.2f}°  标准差 {res['yaw_std_deg']:.2f}°")
    print(f"  式(21)预测 √2σ/L = {pred:.2f}°   实测/预测 = {res['ratio_meas_over_pred']:.2f}")
    print(f"  σ_yaw × L = {res['sigma_yaw_times_L']:.3f} °·m （各基线之间应当一致）")

    # ---- 块平均标度检验：不换基线也能验证式(21)分子上的√2σ ----
    pairs = []
    for ra in A:
        rb = _nearest(B, ra[0])
        if rb is None or abs(rb[0] - ra[0]) > a.max_dt:
            continue
        pairs.append((ra[1], ra[2], rb[1], rb[2]))
    scale = []
    print('  -- 块平均标度检验（σ_yaw(M)·√M 应当是常数）--')
    for M in (1, 2, 5, 10, 20, 50):
        n, sd = _yaw_std_blockavg(pairs, M)
        if sd != sd:
            continue
        sd_deg = math.degrees(sd)
        scale.append(dict(M=M, n_blocks=n, sigma_yaw_deg=sd_deg,
                          sigma_times_sqrtM=sd_deg * math.sqrt(M)))
        print(f"    M={M:<3d} 块数{n:<5d} σ_yaw={sd_deg:6.2f}°  σ_yaw·√M={sd_deg*math.sqrt(M):6.2f}")
    res['block_average_scaling'] = scale
    if len(scale) >= 3:
        v = [x['sigma_times_sqrtM'] for x in scale]
        spread = (max(v) - min(v)) / (sum(v) / len(v))
        res['block_scaling_spread'] = spread
        print(f"    σ_yaw·√M 的相对离散度 {spread:.1%}"
              '（<30%说明噪声近似白、式(21)的√2σ这一项成立；'
              '明显偏大说明UWB噪声在时间上相关，块平均降不下去——'
              '这本身是有用的结论，说明有效独立采样率远低于发布率）')
    _save(f'H2_{a.tag}', res)
    return 0


# ------------------------------------------------------ 降级真值：地面基准线
def cmd_refline(a):
    """用**单个标签**自标定一条地面基准方位线，作为双标签不可用时的θ真值来源。

    做法：把标签（连同整机）静止放在地面标记的A点采一段、再放到同一条直线上
    相距L_ref的B点采一段，两段各取均值得到A、B在UWB世界系下的坐标，
    连线方位角 = atan2(By−Ay, Bx−Ax)。

    为什么这样能当真值：方位角误差 ≈ √2·σ/(√N·L_ref)。取σ=0.05 m、L_ref=2.5 m、
    每点N=500帧，得到约0.09°——比被测的θ*精度(预期1~3°)好一个数量级以上。
    这正是论文式(21)的同一套误差传播，只是把"物理基线"换成了"长基线+多帧平均"。

    ⚠️ 用它当θ真值时还要叠加一项**机械对准误差**：飞机机体x轴要对准这条线，
    目视/直尺对准的残差约1~2°，且是常数偏置。这一项无法用平均消掉，报告里
    必须单独列出。另外这个方法只给出**标定那一刻**的θ_true，无法跟踪DLIO
    后续的yaw漂移——所以它只适合评估θ*的绝对准确度，不适合评估长时间跟踪。
    """
    ra = read_bag(a.bag_a, a.ns)
    rb = read_bag(a.bag_b, a.ns)
    key = {'abs': 'abs', 'a': 'tag_a', 'b': 'tag_b'}[a.topic]
    A, B = ra[key], rb[key]
    if not A or not B:
        print(f'!! 两段数据里至少一段没有{a.topic}话题')
        return 1
    ax = sum(r[1] for r in A) / len(A); ay = sum(r[2] for r in A) / len(A)
    bx = sum(r[1] for r in B) / len(B); by = sum(r[2] for r in B) / len(B)
    L = math.hypot(bx - ax, by - ay)
    az = math.degrees(math.atan2(by - ay, bx - ax))
    n_eff = min(len(A), len(B))
    unc = math.degrees(math.sqrt(2) * a.sigma / (math.sqrt(n_eff) * L)) if L > 0 else float('nan')
    res = dict(bag_a=a.bag_a, bag_b=a.bag_b, tag=a.tag, n_a=len(A), n_b=len(B),
               point_a=[ax, ay], point_b=[bx, by], baseline_m=L,
               azimuth_deg=az, sigma_assumed_m=a.sigma,
               azimuth_uncertainty_deg=unc,
               mech_align_error_deg=a.mech_align,
               theta_truth_deg=az + a.mech_offset,
               theta_truth_total_uncertainty_deg=math.hypot(unc, a.mech_align))
    print(f"  A点({ax:.3f}, {ay:.3f}) 取自{len(A)}帧；B点({bx:.3f}, {by:.3f}) 取自{len(B)}帧")
    print(f"  基线长度 {L:.3f} m，方位角 {az:+.3f}°，方位角不确定度 ±{unc:.3f}°")
    print(f"  叠加机械对准误差 ±{a.mech_align:.1f}° 后，θ真值 = {res['theta_truth_deg']:+.2f}° "
          f"± {res['theta_truth_total_uncertainty_deg']:.2f}°")
    print(f"  用法：把这个值传给 theta 子命令的 --theta-truth-deg")
    _save(f'REF_{a.tag}', res)
    return 0


# ------------------------------------------------------------------ H3
def build_truth_series(d):
    """构造真值序列 [(t, θ_true)]，θ_true = ψ_dualtag − ψ_dlio。

    ⚠️ 必须**逐帧先作差、再平滑**，不能先平滑ψ再作差：飞行中机体本身在
    偏航，ψ_dualtag和ψ_dlio都在大幅变化（几十度），对它们各自做窗口平均
    会在转弯段引入巨大误差；而两者之差才是真正近似恒定的那个量（局部系
    到全局系的旋转），对它平滑才有意义。

    ψ_dualtag优先取融合话题uwb/pose_abs里的orientation（机上
    dual_tag_fusion_node已经算好的yaw）；如果该字段全为0（说明发布者没
    填姿态），退回用uwb_a/uwb_b两路原始位置现算。
    """
    src = None
    if d['abs'] and any(abs(r[4]) > 1e-9 for r in d['abs'][:500]):
        src = [(r[0], r[4]) for r in d['abs']]
    elif d['tag_a'] and d['tag_b']:
        src = []
        for ra in d['tag_a']:
            rb = _nearest(d['tag_b'], ra[0])
            if rb is None or abs(rb[0] - ra[0]) > 0.05:
                continue
            src.append((ra[0], wrap_pi(math.atan2(rb[2] - ra[2],
                                                  rb[1] - ra[1]) + math.pi / 2)))
    if not src or not d['odom']:
        return []
    out = []
    for (t, psi_w) in src:
        od = _nearest(d['odom'], t)
        if od is None or abs(od[0] - t) > 0.2:
            continue
        out.append((t, wrap_pi(psi_w - od[4])))
    return out


def truth_at(series, t, win):
    """取t前后win秒内θ_true的平均（解缠后平均，避免±180°绕回）。"""
    if not series:
        return None
    vals = [v for (ts, v) in series if abs(ts - t) <= win]
    if not vals:
        near = _nearest([(ts, v) for (ts, v) in series], t)
        return near[1] if near else None
    ref = vals[len(vals) // 2]
    return ref + sum(wrap_pi(v - ref) for v in vals) / len(vals)


def cmd_theta(a):
    """H3：θ*在线标定精度。

    真值取"双标签测出的机体朝向 ψ_dualtag"减去"DLIO里程计报出的朝向
    ψ_dlio"，二者之差就是局部系到全局系的旋转θ_true。单帧噪声大（式21给出
    约13.9°），但它与被测量完全独立，在滑动窗口内平均后不确定度降到
    13.9°/√N；本函数同时输出这个真值本身的标准误，报告里必须一起给。
    """
    d = read_bag(a.bag, a.ns)
    if not d['odom']:
        print(f'!! 没有里程计数据，缺失清单: {d["_missing"]}')
        return 1
    src_key = {'abs': 'abs', 'a': 'tag_a', 'b': 'tag_b'}[a.abs_topic]
    absrc = d[src_key]
    if not absrc:
        print(f'!! 没有{a.abs_topic}对应的绝对观测话题')
        return 1
    const_truth = (math.radians(a.theta_truth_deg)
                   if a.theta_truth_deg is not None else None)
    truth = [] if const_truth is not None else build_truth_series(d)
    if const_truth is None and not truth:
        print('!! 构造不出真值序列：需要uwb/pose_abs带姿态，或uwb_a/uwb_b两路原始'
              '位置；双标签都不可用时改用 refline 子命令标定基准线，再用'
              ' --theta-truth-deg 传常数真值')
        return 1
    if const_truth is not None:
        print(f'  真值来源：常数 {a.theta_truth_deg:+.2f}°（地面基准线标定）'
              '——只评估绝对准确度，不评估对DLIO偏航漂移的跟踪')
    est = SlidingWindowEstimator(d_min=a.d_min, window=a.window, robust=a.robust)
    ai, latest = 0, None
    hist = []
    for (t, x, y, z, yaw_odom) in d['odom']:
        while ai < len(absrc) and absrc[ai][0] <= t:
            latest = (absrc[ai][1], absrc[ai][2])
            ai += 1
        if latest is None:
            continue
        if not est.update((x, y), latest):
            continue
        th_true = (const_truth if const_truth is not None
                   else truth_at(truth, t, a.truth_window))
        if th_true is None:
            continue
        hist.append((t, est.theta, th_true, est.n_segments))
    if not hist:
        print('!! 没有切出任何位移段——检查累计位移是否够（需要≥W×d_min米）')
        return 1
    t0 = hist[0][0]
    # 统一剔除末尾若干次更新：采集结束时人要把飞机放到地上，这段的位移段
    # 由"放置动作"而不是正常运动产生，属于瞬态，会污染稳态统计。对所有
    # 架次用同一个固定值，不按数据好坏挑。
    if a.drop_tail > 0 and len(hist) > a.drop_tail + 10:
        hist = hist[:-a.drop_tail]
    errs = [wrap_pi(h[1] - h[2]) for h in hist]
    k = max(1, int(len(errs) * (1 - a.tail)))
    tail = errs[k:]
    st = _stats([math.degrees(e) for e in tail])
    # 真值自身的标准误：单帧σ_yaw / sqrt(有效帧数)
    if const_truth is not None:
        # 常数真值来自refline标定，不确定度由那一步给出，不是双标签那套公式
        sigma_yaw_single = float('nan')
        truth_se = (a.truth_uncertainty_deg if a.truth_uncertainty_deg is not None
                    else float('nan'))
    else:
        n_truth = len([1 for r in d['abs'] if abs(r[4]) > 1e-9]) or len(d['tag_a'])
        sigma_yaw_single = math.degrees(math.sqrt(2) * a.sigma / a.baseline)
        truth_se = sigma_yaw_single / math.sqrt(max(n_truth, 1))
    n_conv = next((i + 1 for i in range(len(errs))
                   if all(abs(e) < math.radians(2.0) for e in errs[i:])), None)
    res = dict(bag=a.bag, tag=a.tag, abs_topic=a.abs_topic, d_min=a.d_min, window=a.window, robust=a.robust,
               n_updates=len(hist), n_segments_final=hist[-1][3],
               path_len_m_est=hist[-1][3] * a.d_min,
               theta_final_deg=math.degrees(hist[-1][1]),
               theta_true_final_deg=math.degrees(hist[-1][2]),
               err_mean_deg=st['mean'], err_std_deg=st['std'],
               err_rmse_deg=math.sqrt(st['mean'] ** 2 + st['std'] ** 2),
               err_max_abs_deg=max(abs(st['min']), abs(st['max'])),
               truth_single_frame_sigma_deg=sigma_yaw_single,
               truth_standard_error_deg=truth_se,
               n_segments_at_convergence=n_conv,
               duration_s=hist[-1][0] - t0)
    print(f"  切出{res['n_segments_final']}段（累计位移约{res['path_len_m_est']:.0f} m），"
          f"更新{res['n_updates']}次，时长{res['duration_s']:.0f}s")
    print(f"  θ*末值 {res['theta_final_deg']:+.2f}°  真值末值 {res['theta_true_final_deg']:+.2f}°")
    print(f"  收敛后误差：均值{res['err_mean_deg']:+.2f}° 标准差{res['err_std_deg']:.2f}° "
          f"RMSE {res['err_rmse_deg']:.2f}° 最大{res['err_max_abs_deg']:.2f}°")
    if const_truth is not None:
        se_txt = f"{truth_se:.2f}°" if truth_se == truth_se else "未提供"
        print(f"  真值自身不确定度：{se_txt}（来自refline标定，用"
              f"--truth-uncertainty-deg传入；必须小于上面的RMSE，"
              f"否则测的是真值误差不是估计器误差）")
    else:
        print(f"  真值自身不确定度：单帧{sigma_yaw_single:.1f}° / 平均后{truth_se:.2f}°"
              f"（必须小于上面的RMSE才说明测的是估计器而不是真值噪声）")
    if n_conv:
        print(f"  收敛于第{n_conv}段")
    if a.plot:
        _plot_theta(hist, t0, a.tag)
    _save(f'H3_{a.tag}', res)
    return 0


def _plot_theta(hist, t0, tag):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        for fam in ('Noto Sans CJK SC', 'WenQuanYi Zen Hei', 'DejaVu Sans'):
            try:
                matplotlib.font_manager.findfont(fam, fallback_to_default=False)
                plt.rcParams['font.sans-serif'] = [fam]
                break
            except Exception:
                continue
        plt.rcParams['axes.unicode_minus'] = False
    except Exception as e:
        print(f'  （没有matplotlib，跳过画图：{e}）')
        return
    t = [h[0] - t0 for h in hist]
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    ax[0].plot(t, [math.degrees(h[2]) for h in hist], 'k--', lw=1, label='真值(双标签−DLIO)')
    ax[0].plot(t, [math.degrees(h[1]) for h in hist], lw=1.2, label='在线估计 θ*')
    ax[0].set_xlabel('时间 [s]'); ax[0].set_ylabel('航向角偏差 [°]'); ax[0].grid(alpha=.3)
    ax[0].legend(fontsize=8); ax[0].set_title('(a) 收敛过程')
    ax[1].plot(t, [math.degrees(wrap_pi(h[1] - h[2])) for h in hist], color='C3', lw=1)
    ax[1].axhline(0, color='k', lw=.6); ax[1].fill_between(t, -2, 2, color='gray', alpha=.15)
    ax[1].set_xlabel('时间 [s]'); ax[1].set_ylabel('估计误差 [°]'); ax[1].grid(alpha=.3)
    ax[1].set_title('(b) 误差（灰带±2°）')
    os.makedirs(RESULT_DIR, exist_ok=True)
    p = os.path.join(RESULT_DIR, f'H3_{tag}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight')
    print(f'  -> {p}')


# ------------------------------------------------------------------ H4
def cmd_sweep(a):
    """H4：在同一段真实数据上扫W/d_min、批量vs滑窗、时间配对方式。"""
    d = read_bag(a.bag, a.ns)
    src_key = {'abs': 'abs', 'a': 'tag_a', 'b': 'tag_b'}[a.abs_topic]
    absrc = d[src_key]
    if not d['odom'] or not absrc:
        print('!! 数据不全')
        return 1

    const_truth = (math.radians(a.theta_truth_deg)
                   if getattr(a, 'theta_truth_deg', None) is not None else None)
    truth = [] if const_truth is not None else build_truth_series(d)
    if const_truth is None and not truth:
        print('!! 构造不出真值序列（可用 --theta-truth-deg 传常数真值）')
        return 1

    def run(d_min, window, batch=False, robust=False, pairing='latest'):
        est = (BatchEstimator(d_min=d_min, robust=robust) if batch
               else SlidingWindowEstimator(d_min=d_min, window=window, robust=robust))
        ai, latest = 0, None
        errs, nseg = [], 0
        for (t, x, y, z, yaw_odom) in d['odom']:
            while ai < len(absrc) and absrc[ai][0] <= t:
                latest = (absrc[ai][1], absrc[ai][2])
                ai += 1
            g = latest
            if pairing == 'interp' and ai >= 1 and ai < len(d['abs']):
                b0, b1 = d['abs'][ai - 1], d['abs'][ai]
                dt = b1[0] - b0[0]
                r = 0.0 if dt <= 0 else (t - b0[0]) / dt
                g = (b0[1] + r * (b1[1] - b0[1]), b0[2] + r * (b1[2] - b0[2]))
            if g is None or not est.update((x, y), g):
                continue
            th = (const_truth if const_truth is not None
                  else truth_at(truth, t, a.truth_window))
            if th is None:
                continue
            errs.append(math.degrees(wrap_pi(est.theta - th)))
            nseg = est.n_segments
        if not errs:
            return None
        k = max(1, int(len(errs) * 0.7))
        tail = errs[k:]
        m = sum(tail) / len(tail)
        return dict(n_seg=nseg, bias=m,
                    rmse=math.sqrt(sum(e * e for e in tail) / len(tail)))

    res = dict(bag=a.bag, tag=a.tag, sweep=[], batch=None, pairing={}, robust={})
    print('  -- W / d_min 扫描 --')
    for d_min in (0.3, 0.5, 1.0):
        for W in (5, 10, 20, 50, 100):
            r = run(d_min, W)
            if r:
                res['sweep'].append(dict(d_min=d_min, W=W, **r))
                print(f"    d_min={d_min} W={W:<4d} 段数{r['n_seg']:<4d} "
                      f"RMSE {r['rmse']:.2f}° 偏差{r['bias']:+.2f}°")
    print('  -- 批量 vs 滑窗 --')
    rb = run(0.5, 50, batch=True)
    rs = run(0.5, 50)
    res['batch'] = dict(batch=rb, sliding=rs)
    if rb and rs:
        print(f"    批量 RMSE {rb['rmse']:.2f}°   滑窗(W=50) RMSE {rs['rmse']:.2f}°")
    print('  -- 时间配对方式 --')
    for p in ('latest', 'interp'):
        r = run(0.5, 50, pairing=p)
        res['pairing'][p] = r
        if r:
            print(f"    {p:7s} RMSE {r['rmse']:.2f}° 偏差{r['bias']:+.2f}°")
    print('  -- 抗差开关 --')
    for rb_flag in (False, True):
        r = run(0.5, 50, robust=rb_flag)
        res['robust'][str(rb_flag)] = r
        if r:
            print(f"    robust={rb_flag}: RMSE {r['rmse']:.2f}°")
    _save(f'H4_{a.tag}', res)
    return 0


# ------------------------------------------------------------------ H5
def cmd_robust(a):
    """H5：抗差扩展。在真实数据上做"同一份数据、开/关抗差"的受控对照，
    并给出滑窗残差的分布（现场判断要不要开抗差的依据）。
    --inject-prob/--inject-mag 可在真实数据上再人工注入野值，用来把污染
    比例推到真实环境达不到的水平，检验崩溃点。"""
    import random
    random.seed(a.seed)
    d = read_bag(a.bag, a.ns)
    if not d['odom'] or not d['abs']:
        print('!! 数据不全')
        return 1
    abs_rows = [list(r) for r in d['abs']]
    n_inj = 0
    if a.inject_prob > 0:
        for r in abs_rows:
            if random.random() < a.inject_prob:
                ang = random.uniform(-math.pi, math.pi)
                r[1] += a.inject_mag * math.cos(ang)
                r[2] += a.inject_mag * math.sin(ang)
                n_inj += 1
    const_truth = (math.radians(a.theta_truth_deg)
                   if getattr(a, 'theta_truth_deg', None) is not None else None)
    truth = [] if const_truth is not None else build_truth_series(d)
    if const_truth is None and not truth:
        print('!! 构造不出真值序列（可用 --theta-truth-deg 传常数真值）')
        return 1
    out = dict(bag=a.bag, tag=a.tag, inject_prob=a.inject_prob,
               inject_mag=a.inject_mag, n_injected=n_inj, modes={})
    for robust in (False, True):
        est = SlidingWindowEstimator(d_min=a.d_min, window=a.window, robust=robust)
        ai, latest, errs = 0, None, []
        for (t, x, y, z, yaw_odom) in d['odom']:
            while ai < len(abs_rows) and abs_rows[ai][0] <= t:
                latest = (abs_rows[ai][1], abs_rows[ai][2])
                ai += 1
            if latest is None or not est.update((x, y), latest):
                continue
            th = (const_truth if const_truth is not None
                  else truth_at(truth, t, a.truth_window))
            if th is None:
                continue
            errs.append(math.degrees(wrap_pi(est.theta - th)))
        if not errs:
            continue
        k = max(1, int(len(errs) * 0.7))
        tail = errs[k:]
        m = sum(tail) / len(tail)
        mode = dict(rmse=math.sqrt(sum(e * e for e in tail) / len(tail)), bias=m)
        if not robust:
            r = residuals(list(est.segments), est.theta)
            med, mad = _mad(r)
            thr = med + 3 * mad
            mode['residual'] = dict(median=med, mad=mad, max=max(r),
                                    p95=sorted(r)[int(0.95 * (len(r) - 1))],
                                    outlier_seg_ratio=sum(1 for x in r if x > thr) / len(r))
        out['modes']['robust' if robust else 'plain'] = mode
        print(f"  robust={robust}: RMSE {mode['rmse']:.2f}° 偏差{mode['bias']:+.2f}°")
        if not robust and 'residual' in mode:
            q = mode['residual']
            print(f"    滑窗残差：中位数{q['median']:.3f} m MAD {q['mad']:.3f} m "
                  f"p95 {q['p95']:.3f} m 最大{q['max']:.3f} m 野值段占比{q['outlier_seg_ratio']:.1%}")
    _save(f'H5_{a.tag}', out)
    return 0


# ------------------------------------------------------------------ H6
def cmd_timing(a):
    """H6：机载单次更新耗时。分"未切段"和"切出新段"两条路径分别统计——
    绝大多数调用只做距离判定就返回，混在一起会把真实开销严重低估。"""
    d = read_bag(a.bag, a.ns)
    src_key = {'abs': 'abs', 'a': 'tag_a', 'b': 'tag_b'}[a.abs_topic]
    absrc = d[src_key]
    if not d['odom'] or not absrc:
        print('!! 数据不全')
        return 1
    out = dict(bag=a.bag, tag=a.tag, platform=a.platform,
               abs_topic=a.abs_topic, modes={})
    for robust in (False, True):
        est = SlidingWindowEstimator(d_min=a.d_min, window=a.window, robust=robust)
        ai, latest = 0, None
        plain, cut = [], []
        for (t, x, y, z, yaw_odom) in d['odom']:
            while ai < len(absrc) and absrc[ai][0] <= t:
                latest = (absrc[ai][1], absrc[ai][2])
                ai += 1
            if latest is None:
                continue
            t0 = time.perf_counter()
            did = est.update((x, y), latest)
            dt = (time.perf_counter() - t0) * 1e6
            (cut if did else plain).append(dt)
        m = {}
        for name, arr in (('no_segment', plain), ('segment_cut', cut)):
            if not arr:
                continue
            arr.sort()
            n = len(arr)
            m[name] = dict(n=n, median_us=arr[n // 2], p95_us=arr[int(0.95 * (n - 1))],
                           max_us=arr[-1])
            print(f"  抗差{'开' if robust else '关'} {name}: 中位数 {arr[n//2]:.2f} µs "
                  f"p95 {arr[int(0.95*(n-1))]:.2f} µs 最大 {arr[-1]:.2f} µs ({n}次)")
        out['modes']['robust' if robust else 'plain'] = m
    _save(f'H6_{a.tag}', out)
    return 0


def main():
    ap = argparse.ArgumentParser(description='真机侧L3实验分析工具')
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(p):
        p.add_argument('bag')
        p.add_argument('--ns', default='NX01')
        p.add_argument('--tag', default='run1', help='结果文件名后缀，用来区分不同架次')

    p = sub.add_parser('static', help='H1 静态噪声与野值率'); common(p)
    p.add_argument('--topic', choices=('abs', 'a', 'b'), default='abs')
    p.set_defaults(func=cmd_static)

    p = sub.add_parser('refline', help='用单标签自标定地面基准方位线（降级真值）')
    p.add_argument('bag_a', help='A点静止数据')
    p.add_argument('bag_b', help='B点静止数据')
    p.add_argument('--ns', default='NX01')
    p.add_argument('--tag', default='line1')
    p.add_argument('--topic', choices=('abs', 'a', 'b'), default='abs')
    p.add_argument('--sigma', type=float, default=0.05, help='单站位置噪声σ[m]，取H1实测值')
    p.add_argument('--mech-align', type=float, default=1.5,
                   help='机体轴对准该直线的残差[°]，目视/直尺约1~2')
    p.add_argument('--mech-offset', type=float, default=0.0,
                   help='机体轴与该直线之间刻意留的已知夹角[°]')
    p.set_defaults(func=cmd_refline)

    p = sub.add_parser('yaw', help='H2 双标签测向与式(21)'); common(p)
    p.add_argument('--baseline', type=float, default=0.0,
                   help='基线长度[m]，0=用两标签实测间距的均值')
    p.add_argument('--sigma', type=float, default=0.05, help='单站位置噪声σ[m]，取H1实测值')
    p.add_argument('--max-dt', type=float, default=0.05, help='两标签配对的最大时间差[s]')
    p.set_defaults(func=cmd_yaw)

    for name, fn, helptxt in (('theta', cmd_theta, 'H3 θ*精度'),
                              ('sweep', cmd_sweep, 'H4 参数敏感性/批量/配对'),
                              ('robust', cmd_robust, 'H5 抗差'),
                              ('timing', cmd_timing, 'H6 计算开销')):
        p = sub.add_parser(name, help=helptxt); common(p)
        p.add_argument('--d-min', type=float, default=0.5)
        p.add_argument('--window', type=int, default=50)
        p.add_argument('--truth-window', type=float, default=5.0,
                       help='真值取该时刻前后多少秒内双标签yaw的平均')
        p.add_argument('--sigma', type=float, default=0.05)
        p.add_argument('--baseline', type=float, default=0.29)
        p.add_argument('--abs-topic', choices=('abs', 'a', 'b'), default='abs',
                       help='用哪一路当绝对观测：abs=双标签融合(默认)，a/b=单标签原始')
        p.add_argument('--theta-truth-deg', type=float, default=None,
                       help='双标签不可用时：用refline标定出的常数真值[°]')
        p.add_argument('--truth-uncertainty-deg', type=float, default=None,
                       help='配合--theta-truth-deg：该真值自身的不确定度[°]，'
                            '取refline输出的theta_truth_total_uncertainty_deg')
        if name == 'theta':
            p.add_argument('--robust', action='store_true')
            p.add_argument('--tail', type=float, default=0.3)
            p.add_argument('--drop-tail', type=int, default=3,
                           help='剔除末尾N次更新(放下飞机的瞬态)，默认3')
            p.add_argument('--plot', action='store_true')
        if name == 'robust':
            p.add_argument('--inject-prob', type=float, default=0.0)
            p.add_argument('--inject-mag', type=float, default=2.0)
            p.add_argument('--seed', type=int, default=1)
        if name == 'timing':
            p.add_argument('--platform', default='Jetson Orin NX 16G / JetPack 6.2')
        p.set_defaults(func=fn)

    a = ap.parse_args()
    print(f'== {a.cmd} : {a.bag} ==')
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
