#!/usr/bin/env python3
"""E14（机间链式复合误差）+ E16（计算开销）——都只吃已经录好的bag，不用重飞。

E14：论文1.1/2.1节声称"多机部署只需对每个智能体分别独立调用，若还需要机间相对
位姿，两台各自标定出世界系到自身地图系的变换后做一次坐标链式复合即可"。这里
量化这句话的实际误差：
  ① 单机全局定位误差——用本机标定出的(θ*,t)把DLIO局部位置映射到世界系，
     跟Gazebo真值比。这是标定质量的端到端**米制**度量（论文目前只有角度指标）。
  ② 机间相对位置误差——两机各自映射到世界系后作差，跟真值作差比。
(θ*,t)直接从bag里的/tf_static读（origin_setter每次θ*更新都会重新广播这条
world -> {ns}/map的静态TF，所以bag里是一条随时间更新的序列，不是只有一帧）。

E16：把同一份真实数据流喂给滑窗估计器，测单次update()的耗时分布，支撑论文
"每次更新是滑窗内O(W)标量运算、适合机载"这句话。抗差开/关分别测。

用法：
  python3 scripts/paper_exp/pairwise_eval.py runtime_logs/paper_exp/e11_run01
  python3 scripts/paper_exp/pairwise_eval.py <bag> --timing
"""
import argparse
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from se2_core import SlidingWindowEstimator, wrap_pi  # noqa: E402

OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'paper_exp_results'))
NS_LIST = ('NX01', 'NX02')


def _yaw(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def read_all(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {ns: {'odom': [], 'truth': [], 'calib': []} for ns in NS_LIST}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if topic == '/tf_static':
            msg = deserialize_message(raw, get_message(types[topic]))
            for tr in msg.transforms:
                for ns in NS_LIST:
                    # origin_setter广播的是 world -> {ns}/map
                    if tr.child_frame_id == f'{ns}/map' and tr.header.frame_id == 'world':
                        q = tr.transform.rotation
                        data[ns]['calib'].append(
                            (t, tr.transform.translation.x,
                             tr.transform.translation.y, _yaw(q.z, q.w)))
            continue
        for ns in NS_LIST:
            if topic == f'/{ns}/dlio/odom_node/odom':
                m = deserialize_message(raw, get_message(types[topic]))
                p = m.pose.pose.position
                data[ns]['odom'].append((t, p.x, p.y))
            elif topic == f'/{ns}/uwb/pose_truth':
                m = deserialize_message(raw, get_message(types[topic]))
                p = m.pose.position
                data[ns]['truth'].append((t, p.x, p.y))
    for ns in NS_LIST:
        for k in data[ns]:
            data[ns][k].sort(key=lambda r: r[0])
    return data


def _at(series, t):
    """取series里时间不晚于t的最后一条（复刻"当前生效值"的语义）。"""
    lo, hi = 0, len(series) - 1
    if not series or t < series[0][0]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    return series[hi] if series[hi][0] <= t else series[lo]


def evaluate(bag_dir, tail_frac=0.3):
    data = read_all(bag_dir)
    for ns in NS_LIST:
        n = {k: len(v) for k, v in data[ns].items()}
        print(f'  {ns}: odom {n["odom"]}, truth {n["truth"]}, 标定TF {n["calib"]}')
        if not data[ns]['calib']:
            print('  ⚠️ bag里没有world->{ns}/map的静态TF，无法评估'.format(ns=ns))
            return None

    rows = []
    for (t, ox, oy) in data['NX01']['odom']:
        rec = {}
        ok = True
        for ns in NS_LIST:
            od = _at(data[ns]['odom'], t)
            tr = _at(data[ns]['truth'], t)
            cb = _at(data[ns]['calib'], t)
            if od is None or tr is None or cb is None:
                ok = False
                break
            _, cx, cy, th = cb
            c, s = math.cos(th), math.sin(th)
            wx = cx + od[1] * c - od[2] * s
            wy = cy + od[1] * s + od[2] * c
            rec[ns] = dict(est=(wx, wy), truth=(tr[1], tr[2]), theta=th)
        if not ok:
            continue
        e1 = math.dist(rec['NX01']['est'], rec['NX01']['truth'])
        e2 = math.dist(rec['NX02']['est'], rec['NX02']['truth'])
        rel_est = (rec['NX02']['est'][0] - rec['NX01']['est'][0],
                   rec['NX02']['est'][1] - rec['NX01']['est'][1])
        rel_tru = (rec['NX02']['truth'][0] - rec['NX01']['truth'][0],
                   rec['NX02']['truth'][1] - rec['NX01']['truth'][1])
        rows.append((t, e1, e2, math.dist(rel_est, rel_tru),
                     math.degrees(rec['NX01']['theta']),
                     math.degrees(rec['NX02']['theta'])))
    if not rows:
        print('  ⚠️ 没有可评估的时刻')
        return None
    k = int(len(rows) * (1 - tail_frac))
    tail = rows[k:]

    def stat(idx):
        v = [r[idx] for r in tail]
        m = sum(v) / len(v)
        return m, math.sqrt(sum((x - m) ** 2 for x in v) / len(v)), max(v)

    print(f'  样本数 {len(rows)}，统计取末尾{int(tail_frac*100)}%（{len(tail)}个）：')
    labels = {1: 'NX01全局定位误差', 2: 'NX02全局定位误差', 3: '机间相对位置误差'}
    out = {}
    for idx, name in labels.items():
        m, sd, mx = stat(idx)
        out[name] = (m, sd, mx)
        print(f'    {name}: 均值 {m:.3f} m, 标准差 {sd:.3f} m, 最大 {mx:.3f} m')
    print(f'    收敛后 θ*: NX01 {tail[-1][4]:.2f}°, NX02 {tail[-1][5]:.2f}°')
    os.makedirs(OUT_DIR, exist_ok=True)
    tag = os.path.basename(bag_dir.rstrip('/'))
    path = os.path.join(OUT_DIR, f'l2_e14_pairwise_{tag}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'nx01_global_err_m', 'nx02_global_err_m',
                    'inter_agent_err_m', 'nx01_theta_deg', 'nx02_theta_deg'])
        w.writerows(rows)
    print(f'  -> {path}')
    return out


def timing(bag_dir, ns='NX01'):
    """E16：用真实数据流测单次update()耗时（抗差关/开各测一遍）。"""
    data = read_all(bag_dir)
    odom = data[ns]['odom']
    truth = data[ns]['truth']
    print(f'  用 {ns} 的 {len(odom)} 帧里程计做计时')
    for robust in (False, True):
        est = SlidingWindowEstimator(d_min=0.5, window=50, robust=robust)
        durs, cut_durs = [], []
        j = 0
        for (t, x, y) in odom:
            while j + 1 < len(truth) and truth[j + 1][0] <= t:
                j += 1
            g = (truth[j][1], truth[j][2])
            t0 = time.perf_counter()
            cut = est.update((x, y), g)
            dt = (time.perf_counter() - t0) * 1e6           # 微秒
            (cut_durs if cut else durs).append(dt)
        # 两条路径必须分开报：绝大多数调用只是"位移还不够、直接返回"，
        # 真正做完整读数(抗差时还要跑IRLS)的只有切出新段的那些调用，
        # 混在一起算中位数会把开销严重低估。
        for name, arr in (('未切段(仅距离判定)', durs), ('切出新段(含读数)', cut_durs)):
            if not arr:
                continue
            arr.sort()
            n = len(arr)
            print(f'    抗差{"开" if robust else "关"} {name}: '
                  f'中位数 {arr[n//2]:.2f} µs, p95 {arr[int(0.95*n)]:.2f} µs, '
                  f'最大 {arr[-1]:.2f} µs, {n}次')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--timing', action='store_true')
    a = ap.parse_args()
    print(f'== {a.bag} ==')
    if a.timing:
        timing(a.bag)
    else:
        evaluate(a.bag)
    return 0


if __name__ == '__main__':
    sys.exit(main())
