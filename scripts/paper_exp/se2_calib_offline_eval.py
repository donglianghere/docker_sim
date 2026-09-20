#!/usr/bin/env python3
"""L1层：从rosbag离线复算SE(2)在线标定（论文E7–E10）。

对应`docker_sim/实验方案_SE2在线标定论文补充实验.md`第二节L1层。核心思路：
真机/仿真里**飞一次**、把原始数据流录下来，之后所有参数扫描(W、d_min、
批量vs滑窗、Huber开关、时间配对方式)都在宿主机上离线复算，不用每换一组
参数就重飞一次——飞行本身才是这套实验里最贵的一步。

需要bag里有这几路（`scripts/record_rosbag.sh` 2026-09-03起已经加进去了）：
  /{ns}/dlio/odom_node/odom            nav_msgs/Odometry      局部位姿
  /{ns}/uwb/pose_abs                   geometry_msgs/PoseStamped 带噪声绝对观测
  /{ns}/uwb/pose_truth                 geometry_msgs/PoseStamped 真值(评估用)
  /{ns}/origin_setter/yaw_estimate     std_msgs/Float64       机上估计器输出
真值θ_true(t) = yaw_truth(t) − yaw_odom(t)：不是固定的spawn yaw常数，而是
逐时刻的真值——这样连DLIO自己的yaw漂移一起算进去，评估的是"局部系到全局系
的真实变换"而不是"出生时那个角度"。没有pose_truth的老bag可以用
`--theta-truth-deg`退化成常数真值。

用法：
  # 自检（不需要真实bag，自己合成一个再读回来跑通全流程）
  python3 scripts/paper_exp/se2_calib_offline_eval.py --self-test
  # 单次复算
  python3 scripts/paper_exp/se2_calib_offline_eval.py runtime_logs/rosbag/bag_xxx --ns NX01
  # 参数扫描（E7）+ 批量对比（E8）+ 时间配对消融（E9）+ 残差统计（E10）
  python3 scripts/paper_exp/se2_calib_offline_eval.py BAG --ns NX01 --sweep
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from se2_core import (SlidingWindowEstimator, BatchEstimator,  # noqa: E402
                      solve_theta, residuals, wrap_pi)

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'paper_exp_results'))

TOPIC_ODOM = '{ns}/dlio/odom_node/odom'
TOPIC_ABS = '{ns}/uwb/pose_abs'
TOPIC_TRUTH = '{ns}/uwb/pose_truth'
TOPIC_YAW = '{ns}/origin_setter/yaw_estimate'


def _quat_yaw(z, w, x=0.0, y=0.0):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ------------------------------------------------------------ bag读取
def read_bag(bag_dir, ns, time_source='recv'):
    """返回 {'odom': [(t,x,y,yaw)], 'abs': [(t,x,y)], 'truth': [(t,x,y,yaw)],
    'yaw_on': [(t,theta)]}。

    ⚠️ 时间基准默认用**bag的接收时间戳**而不是header.stamp。2026-09-03实测
    踩到的坑：这套系统里两路数据根本不在同一个时钟域——DLIO(和它下游的
    /dlio/odom_node/odom)活在`mighty_imu_sim_time.patch`引入的"假epoch"
    仿真时钟里(仿真时间+1735689600，即2025-01-01)，而`uwb_sim`节点没开
    use_sim_time、用的是宿主机墙钟(2026年)，两者的header.stamp差了5千多万秒；
    更麻烦的是二者的**走时速率也不同**(仿真时钟按RTF≈0.3~0.5倍速走)，所以连
    "减一个固定偏移"都对不齐。按header.stamp配对的结果是一段都切不出来。

    接收时间戳则是录包进程自己的单一时钟，天然可比，而且它正好复刻了机上
    `origin_setter_node`的真实行为——那个节点根本不看header.stamp，只用
    "当前最新收到的那一帧"(`self._latest_uwb_xy`)。也就是说这个坑不只是离线
    工具的问题：这套系统里**基于时间戳的配对本来就不可用**，第4.7节的结论
    因此不仅是"没必要对齐"，而是"没有共同时基可对齐"。

    time_source='header'保留给单一时钟域的数据集（比如--self-test合成的bag），
    在本项目的真实bag上会切不出段。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    ns = ns.strip('/')
    want = {f'/{TOPIC_ODOM.format(ns=ns)}': 'odom',
            f'/{TOPIC_ABS.format(ns=ns)}': 'abs',
            f'/{TOPIC_TRUTH.format(ns=ns)}': 'truth',
            f'/{TOPIC_YAW.format(ns=ns)}': 'yaw_on'}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {k: [] for k in ('odom', 'abs', 'truth', 'yaw_on')}
    missing = [t for t in want if t not in types]
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        key = want.get(topic)
        if key is None:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if key == 'yaw_on':
            out[key].append((t_ns * 1e-9, float(msg.data)))
            continue
        if time_source == 'header':
            h = msg.header.stamp
            t = h.sec + h.nanosec * 1e-9
            if t <= 0.0:
                t = t_ns * 1e-9
        else:
            t = t_ns * 1e-9
        if key == 'odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
        else:
            p, q = msg.pose.position, msg.pose.orientation
        yaw = _quat_yaw(q.z, q.w, q.x, q.y)
        out[key].append((t, p.x, p.y) if key == 'abs' else (t, p.x, p.y, yaw))
    for k in out:
        out[k].sort(key=lambda r: r[0])
    if missing:
        print(f'  ⚠️ bag里缺这些话题（相关指标会退化/跳过）: {missing}')
    return out


# ------------------------------------------------------------ 复算内核
def _interp_xy(series, t):
    """在(t,x,y[,yaw])序列上按时间线性插值；越界时取端点。"""
    if not series:
        return None
    if t <= series[0][0]:
        return series[0][1], series[0][2]
    if t >= series[-1][0]:
        return series[-1][1], series[-1][2]
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, x0, y0 = series[lo][0], series[lo][1], series[lo][2]
    t1, x1, y1 = series[hi][0], series[hi][1], series[hi][2]
    k = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return x0 + k * (x1 - x0), y0 + k * (y1 - y0)


def replay(data, W=50, d_min=0.5, batch=False, robust=False,
           pairing='latest', theta_truth_deg=None):
    """按时间顺序重放bag，返回逐次更新的记录。

    pairing='latest'：复刻机上行为——里程计回调那一刻取"最新收到的一帧UWB"；
    pairing='interp'：按时间戳把UWB插值到里程计时刻（E9消融用的对照方案）。
    """
    est = (BatchEstimator(d_min=d_min, robust=robust) if batch
           else SlidingWindowEstimator(d_min=d_min, window=W, robust=robust))
    abs_series = data['abs']
    truth = data['truth']
    ai = 0
    latest_abs = None
    hist = []      # (t, theta_hat, theta_true, n_seg)
    for (t, lx, ly, lyaw) in data['odom']:
        while ai < len(abs_series) and abs_series[ai][0] <= t:
            latest_abs = (abs_series[ai][1], abs_series[ai][2])
            ai += 1
        if pairing == 'interp':
            g = _interp_xy(abs_series, t)
        else:
            g = latest_abs
        if g is None:
            continue
        changed = est.update((lx, ly), g)
        if not changed:
            continue
        if theta_truth_deg is not None:
            th_true = math.radians(theta_truth_deg)
        elif truth:
            # 逐时刻真值：θ_true = yaw_truth − yaw_odom
            j = min(range(len(truth)), key=lambda i: abs(truth[i][0] - t)) \
                if len(truth) < 4000 else _nearest_idx(truth, t)
            th_true = wrap_pi(truth[j][3] - lyaw)
        else:
            th_true = float('nan')
        hist.append((t, est.theta, th_true, est.n_segments))
    return est, hist


def _nearest_idx(series, t):
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    return lo if abs(series[lo][0] - t) <= abs(series[hi][0] - t) else hi


def summarize(hist, tail_frac=0.3):
    """收敛后(末尾tail_frac部分)的误差统计。"""
    if not hist:
        return dict(n=0, final_err_deg=float('nan'), rmse_deg=float('nan'),
                    bias_deg=float('nan'), std_deg=float('nan'),
                    n_seg=0, t_converge=float('nan'), n_converge=float('nan'))
    errs = [wrap_pi(h[1] - h[2]) for h in hist if not math.isnan(h[2])]
    if not errs:
        return dict(n=len(hist), final_err_deg=float('nan'),
                    rmse_deg=float('nan'), bias_deg=float('nan'),
                    std_deg=float('nan'), n_seg=hist[-1][3],
                    t_converge=float('nan'), n_converge=float('nan'))
    k = max(1, int(len(errs) * (1 - tail_frac)))
    tail = errs[k:]
    mean = sum(tail) / len(tail)
    var = sum((e - mean) ** 2 for e in tail) / len(tail)
    rmse = math.sqrt(sum(e * e for e in tail) / len(tail))
    # 收敛时刻：第一次进入±2°并且之后不再跑出去
    # 收敛判据：从第i次更新起误差再没跑出过±2°。同时给出"此时已经攒了多少段"
    # ——按时间报会被飞行速度和仿真实时因子影响，按段数报才是这个估计器自身的
    # 收敛尺度，跟式(13)的W可以直接对照。
    t_conv = float('nan')
    n_conv = float('nan')
    for i in range(len(errs)):
        if all(abs(e) < math.radians(2.0) for e in errs[i:]):
            t_conv = hist[i][0] - hist[0][0]
            n_conv = i + 1
            break
    return dict(n=len(errs), final_err_deg=math.degrees(errs[-1]),
                rmse_deg=math.degrees(rmse), bias_deg=math.degrees(mean),
                std_deg=math.degrees(math.sqrt(var)), n_seg=hist[-1][3],
                t_converge=t_conv, n_converge=n_conv)


def residual_stats(est):
    """E10：滑窗内残差分布——检验高斯假设、给野值率，为2.7节抗差提供实证动机。"""
    segs = list(est.segments)
    if not segs:
        return {}
    r = residuals(segs, est.theta)
    r_sorted = sorted(r)
    n = len(r)
    med = r_sorted[n // 2]
    mad = sorted(abs(x - med) for x in r)[n // 2]
    return dict(n=n, mean=sum(r) / n, median=med, mad=mad,
                p95=r_sorted[int(0.95 * (n - 1))], max=r_sorted[-1],
                outlier_ratio=sum(1 for x in r if x > med + 3 * mad) / n)


# ------------------------------------------------------------ 扫描实验
def sweep(data, ns, theta_truth_deg=None):
    rows = []
    print('== E7 参数敏感性（真实数据）==')
    for W in (5, 10, 20, 50, 100, 200):
        for d in (0.3, 0.5, 1.0, 2.0):
            est, hist = replay(data, W=W, d_min=d,
                               theta_truth_deg=theta_truth_deg)
            s = summarize(hist)
            rows.append(['sliding', W, d, 'latest', 0, s['n_seg'],
                         s['bias_deg'], s['std_deg'], s['rmse_deg'],
                         s['final_err_deg'], s['t_converge']])
            print(f'  W={W:<4d} d={d:<4} 段数{s["n_seg"]:<4d} '
                  f'RMSE {s["rmse_deg"]:.2f}° 偏差{s["bias_deg"]:+.2f}°')
    print('== E8 批量 vs 滑窗（真实数据）==')
    for d in (0.5,):
        est, hist = replay(data, d_min=d, batch=True,
                           theta_truth_deg=theta_truth_deg)
        s = summarize(hist)
        rows.append(['batch', -1, d, 'latest', 0, s['n_seg'], s['bias_deg'],
                     s['std_deg'], s['rmse_deg'], s['final_err_deg'],
                     s['t_converge']])
        print(f'  批量 d={d}: RMSE {s["rmse_deg"]:.2f}° 偏差{s["bias_deg"]:+.2f}°')
    print('== E9 时间配对方式消融 ==')
    for pairing in ('latest', 'interp'):
        est, hist = replay(data, pairing=pairing,
                           theta_truth_deg=theta_truth_deg)
        s = summarize(hist)
        rows.append(['sliding', 50, 0.5, pairing, 0, s['n_seg'], s['bias_deg'],
                     s['std_deg'], s['rmse_deg'], s['final_err_deg'],
                     s['t_converge']])
        print(f'  {pairing:7s}: RMSE {s["rmse_deg"]:.2f}° 偏差{s["bias_deg"]:+.2f}°')
    print('== E4b Huber开关（真实数据）==')
    for robust in (False, True):
        est, hist = replay(data, robust=robust,
                           theta_truth_deg=theta_truth_deg)
        s = summarize(hist)
        rows.append(['sliding', 50, 0.5, 'latest', int(robust), s['n_seg'],
                     s['bias_deg'], s['std_deg'], s['rmse_deg'],
                     s['final_err_deg'], s['t_converge']])
        print(f'  robust={robust}: RMSE {s["rmse_deg"]:.2f}°')
        if robust is False:
            rs = residual_stats(est)
            if rs:
                print(f'== E10 残差统计: 中位数{rs["median"]:.3f} m  MAD {rs["mad"]:.3f} m '
                      f'p95 {rs["p95"]:.3f} m  最大{rs["max"]:.3f} m  '
                      f'野值率(>中位数+3MAD) {rs["outlier_ratio"]:.1%}')
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f'l1_offline_sweep_{ns}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['estimator', 'W', 'd_min', 'pairing', 'robust',
                    'n_segments', 'bias_deg', 'std_deg', 'rmse_deg',
                    'final_err_deg', 't_converge_s'])
        w.writerows(rows)
    print(f'  -> {path}')
    return rows


# ------------------------------------------------------------ 自检
def self_test(tmp_dir=None):
    """不依赖真实bag：用rosbag2_py写一个合成bag（话题名/类型跟真实录制
    完全一致），再走一遍完整读取+复算流程，验证这个工具本身是通的。"""
    import shutil
    import random
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped

    tmp_dir = tmp_dir or os.path.join(
        os.environ.get('TMPDIR', '/tmp'), 'se2_selftest_bag')
    shutil.rmtree(tmp_dir, ignore_errors=True)
    ns, theta = 'NX01', math.radians(30.0)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=tmp_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for name, typ in ((f'/{TOPIC_ODOM.format(ns=ns)}', 'nav_msgs/msg/Odometry'),
                      (f'/{TOPIC_ABS.format(ns=ns)}', 'geometry_msgs/msg/PoseStamped'),
                      (f'/{TOPIC_TRUTH.format(ns=ns)}', 'geometry_msgs/msg/PoseStamped')):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=name, type=typ, serialization_format='cdr'))
    rng = random.Random(5)
    sigma = 0.05
    c, s = math.cos(theta), math.sin(theta)
    for i in range(1200):                       # 60 s @20 Hz，蛇形轨迹
        t = i / 20.0
        lx, ly = 1.0 * t, 2.0 * math.sin(0.25 * t)
        gx, gy = lx * c - ly * s, lx * s + ly * c
        def _stamp(m):
            m.header.stamp.sec = int(t)
            m.header.stamp.nanosec = int((t % 1.0) * 1e9)
            m.header.frame_id = 'world'
            return m
        od = Odometry()
        _stamp(od)
        od.pose.pose.position.x, od.pose.pose.position.y = lx, ly
        od.pose.pose.orientation.w = 1.0        # 局部系yaw恒为0
        writer.write(f'/{TOPIC_ODOM.format(ns=ns)}',
                     serialize_message(od), int(t * 1e9))
        if i % 2 == 0:                          # UWB 10 Hz
            pa = _stamp(PoseStamped())
            pa.pose.position.x = gx + rng.gauss(0, sigma)
            pa.pose.position.y = gy + rng.gauss(0, sigma)
            pa.pose.orientation.w = 1.0
            writer.write(f'/{TOPIC_ABS.format(ns=ns)}',
                         serialize_message(pa), int(t * 1e9))
            pt = _stamp(PoseStamped())
            pt.pose.position.x, pt.pose.position.y = gx, gy
            pt.pose.orientation.z = math.sin(theta / 2)
            pt.pose.orientation.w = math.cos(theta / 2)
            writer.write(f'/{TOPIC_TRUTH.format(ns=ns)}',
                         serialize_message(pt), int(t * 1e9))
    del writer
    print(f'合成bag: {tmp_dir}')
    data = read_bag(tmp_dir, ns)
    print(f'  读回: odom {len(data["odom"])} 帧, abs {len(data["abs"])} 帧, '
          f'truth {len(data["truth"])} 帧')
    est, hist = replay(data)
    st = summarize(hist)
    print(f'  复算: 段数{st["n_seg"]}, θ*={math.degrees(est.theta):.3f}° '
          f'(真值30°), 稳态偏差{st["bias_deg"]:+.3f}° RMSE {st["rmse_deg"]:.3f}°')
    ok = abs(wrap_pi(est.theta - math.radians(30.0))) < math.radians(1.5)
    print('  自检' + ('通过 ✅' if ok else '失败 ❌'))
    return 0 if ok else 1


def convergence_figure(data, ns, theta_truth_deg=None, tag=''):
    """E11主实验的插图：θ*(t)与逐时刻真值的对照 + 误差曲线（论文图5b）。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for fam in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'WenQuanYi Zen Hei'):
        try:
            matplotlib.font_manager.findfont(fam, fallback_to_default=False)
            plt.rcParams['font.sans-serif'] = [fam]
            break
        except Exception:
            continue
    plt.rcParams['axes.unicode_minus'] = False
    est, hist = replay(data, theta_truth_deg=theta_truth_deg)
    if not hist:
        print('  没有可画的数据（没切出任何位移段）')
        return None
    t0 = hist[0][0]
    t = [h[0] - t0 for h in hist]
    th = [math.degrees(h[1]) for h in hist]
    tt = [math.degrees(h[2]) for h in hist]
    err = [math.degrees(wrap_pi(h[1] - h[2])) for h in hist]
    nseg = [h[3] for h in hist]
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    axes[0].plot(t, tt, 'k--', lw=1.2, label='真值 $\\theta_{true}(t)$')
    axes[0].plot(t, th, lw=1.2, label='在线估计 $\\theta^*$')
    axes[0].set_xlabel('时间 [s]'); axes[0].set_ylabel('航向角偏差 [°]')
    axes[0].set_title(f'(a) {ns} 收敛过程'); axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    ax2 = axes[1]
    ax2.plot(t, err, lw=1.0, color='C3')
    ax2.axhline(0, color='k', lw=0.6)
    ax2.fill_between(t, -2, 2, color='gray', alpha=0.15, label='±2°')
    ax2.set_xlabel('时间 [s]'); ax2.set_ylabel('估计误差 [°]')
    ax2.set_title('(b) 误差与滑窗段数'); ax2.grid(alpha=0.3)
    ax3 = ax2.twinx()
    ax3.plot(t, nseg, lw=0.8, color='C0', alpha=0.6)
    ax3.set_ylabel('滑窗内段数', color='C0')
    ax2.legend(fontsize=8, loc='upper right')
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f'fig_e11_convergence_{ns}{tag}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  -> {path}')
    return path


def aggregate(bags, ns, theta_truth_deg=None, time_source='recv'):
    """E11的N次重复聚合（缺口G14：不能只报单次数值，要mean±std）。"""
    rows = []
    for b in bags:
        try:
            data = read_bag(b, ns, time_source)
        except Exception as e:
            print(f'  跳过{b}: {type(e).__name__} {e}')
            continue
        est, hist = replay(data, theta_truth_deg=theta_truth_deg)
        st = summarize(hist)
        rows.append([os.path.basename(b.rstrip('/')), st['n_seg'],
                     st['bias_deg'], st['std_deg'], st['rmse_deg'],
                     st['final_err_deg'], st['t_converge'], st['n_converge']])
        print(f'  {rows[-1][0]}: RMSE {st["rmse_deg"]:.2f}° 稳态偏差'
              f'{st["bias_deg"]:+.2f}° 收敛于第{st["n_converge"]:.0f}段'
              f'({st["t_converge"]:.1f}s)')
    if not rows:
        return rows
    import statistics as stx
    for i, name in ((4, 'RMSE'), (2, '稳态偏差'), (6, '收敛耗时[s]'),
                    (7, '收敛所需段数')):
        vals = [r[i] for r in rows if not math.isnan(r[i])]
        if len(vals) >= 2:
            print(f'  == {name}: {stx.fmean(vals):.2f} ± {stx.stdev(vals):.2f} '
                  f'(N={len(vals)})')
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f'l2_e11_runs_{ns}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['bag', 'n_segments', 'bias_deg', 'std_deg', 'rmse_deg',
                    'final_err_deg', 't_converge_s', 'n_seg_at_converge'])
        w.writerows(rows)
    print(f'  -> {path}')
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?', help='rosbag目录')
    ap.add_argument('--ns', default='NX01')
    ap.add_argument('--sweep', action='store_true', help='跑E7–E10全套扫描')
    ap.add_argument('--theta-truth-deg', type=float, default=None,
                    help='没有pose_truth的老bag用：给一个常数真值角度')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--time-source', choices=('recv', 'header'), default='recv',
                    help='配对用的时间基准，默认recv(bag接收时间)，见read_bag说明')
    ap.add_argument('--fig', action='store_true',
                    help='画E11收敛曲线（论文图5b）')
    ap.add_argument('--aggregate', nargs='+', metavar='BAG',
                    help='多个bag聚合成mean±std（E11的N次重复）')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if a.aggregate:
        aggregate(a.aggregate, a.ns, a.theta_truth_deg, a.time_source)
        return 0
    if not a.bag:
        ap.error('需要给bag目录，或者用 --self-test / --aggregate')
    data = read_bag(a.bag, a.ns, a.time_source)
    print(f'{a.ns}: odom {len(data["odom"])} 帧, abs {len(data["abs"])} 帧, '
          f'truth {len(data["truth"])} 帧, 机上yaw {len(data["yaw_on"])} 帧')
    if a.fig:
        convergence_figure(data, a.ns, a.theta_truth_deg)
    if a.sweep:
        sweep(data, a.ns, a.theta_truth_deg)
    if not a.sweep:
        est, hist = replay(data, theta_truth_deg=a.theta_truth_deg)
        st = summarize(hist)
        print(f'θ*={math.degrees(est.theta):.3f}°  段数{st["n_seg"]}  '
              f'稳态偏差{st["bias_deg"]:+.2f}°  RMSE {st["rmse_deg"]:.2f}°  '
              f'收敛耗时{st["t_converge"]:.1f}s')
        if data['yaw_on']:
            print(f'  机上估计器最后一次输出: '
                  f'{math.degrees(data["yaw_on"][-1][1]):.3f}°（应与离线复算接近）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
