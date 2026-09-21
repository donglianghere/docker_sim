# -*- coding: utf-8 -*-
"""测量静止时里程计速度噪声，用数据定起飞前"静止"判据的窗口长度。

    docker run --rm --network host -v $PWD/scripts:/w contestant-sdk:latest \
        python3 -u /w/measure_static_noise.py 90

录一段两机静止在地上时的速度序列，算不同窗口长度下"速度矢量均值的模"的
分布，输出每个窗口的均值/最大值/超门限占比。

为什么要有这个工具：takeoff_monitor_node 判"静止"用的是速度矢量的窗口
平均（对矢量平均、不是对速度大小平均，理由见那边的注释）。窗口多长才够，
取决于定位源的噪声水平——仿真里是 uwb_ground_truth_node 的 range_noise_std
（默认 0.05m/轴），真机上是 nlink 的实际噪声。**换定位源、换场地、改噪声
参数之后都应该重新跑一次这个脚本，再回去核 static_hold_s**。

2026-09-21 在仿真里的实测结果见 takeoff_monitor_node.py 里 static_hold_s
参数上方的表格。
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
THRESH = 0.08
WINDOWS = (0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0)

rclpy.init()
n = Node('win')
series = {'NX01': [], 'NX02': []}


def mk(ns):
    def cb(m):
        v = m.twist.twist.linear
        series[ns].append((time.time(), v.x, v.y, v.z))
    return cb


for ns in series:
    n.create_subscription(Odometry, f'/{ns}/dlio/odom_node/odom', mk(ns), qos_profile_sensor_data)

t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
rclpy.shutdown()

for ns, rows in series.items():
    if len(rows) < 50:
        print(f'{ns}: 样本太少({len(rows)})'); continue
    rate = len(rows) / (rows[-1][0] - rows[0][0])
    print(f'\n===== {ns}   {len(rows)}个样本，{rate:.0f}Hz')
    print(f'{"窗口":>6} {"均值":>8} {"最大":>8} {"超门限占比":>10}   判断')
    for w in WINDOWS:
        k = max(1, int(w * rate))
        if k >= len(rows):
            continue
        means = []
        for i in range(0, len(rows) - k):
            seg = rows[i:i + k]
            mx = sum(r[1] for r in seg) / k
            my = sum(r[2] for r in seg) / k
            mz = sum(r[3] for r in seg) / k
            means.append(math.sqrt(mx * mx + my * my + mz * mz))
        over = sum(1 for m in means if m > THRESH) / len(means) * 100
        verdict = '合格' if over == 0 else ('偶发超标' if over < 5 else '经常超标')
        print(f'{w:>5.1f}s {sum(means)/len(means):>8.3f} {max(means):>8.3f} {over:>9.1f}%   {verdict}')
