#!/usr/bin/env python3
"""E13分析：把goal_retransform_test.py的落点结果整理成论文表格。

论文3.2节的假说是"目标点只在收到那一刻变换一次、之后θ*的修正不再追溯应用"，
它给出一个**可证伪的定量预测**：飞机最终会停在"把指定的世界目标点绕标定锚点
旋转了+θ_true之后"的位置上——因为发目标点那一刻θ*还是0，换算出的局部坐标
相当于把世界点当成了局部点，飞机飞到那个局部点之后，它在世界系里的真实位置
就是 anchor + R(θ_true)·(goal − anchor)。

误差幅度的解析式：|err| = 2·|goal − anchor|·sin(θ_true/2)。

这个脚本读 runtime_logs/paper_exp_e13_goal_results.txt，逐条算出预测落点、
预测误差幅度，跟实测对比，并按修复前/修复后分组汇总。
"""
import math
import os
import re
import sys
import csv

# 两机的标定锚点≈出生点（sim-world-entrypoint.sh里spawn在(3i,0)），实测锁定
# 消息给出的偏移是(3.003,0.004)和(5.995,-0.001)，跟标称值差毫米级
ANCHOR = {'NX01': (3.0, 0.0), 'NX02': (6.0, 0.0)}
THETA_TRUE_DEG = {'NX01': 30.0, 'NX02': -45.0}

RES_FILE = 'runtime_logs/paper_exp_e13_goal_results.txt'
OUT = 'paper_exp_results/l2_e13_goal_retransform.csv'

PAT = re.compile(
    r'(?P<tag>\S+)\s+RESULT\s+(?P<ns>NX\d+)\s+goal=\((?P<gx>[-\d.]+),(?P<gy>[-\d.]+)\)\s+'
    r'final_truth=\((?P<fx>[-\d.]+),(?P<fy>[-\d.]+)\)')


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else RES_FILE
    rows = []
    for line in open(path):
        m = PAT.search(line)
        if not m:
            continue
        ns = m.group('ns')
        gx, gy = float(m.group('gx')), float(m.group('gy'))
        fx, fy = float(m.group('fx')), float(m.group('fy'))
        ax, ay = ANCHOR[ns]
        th = math.radians(THETA_TRUE_DEG[ns])
        rx, ry = gx - ax, gy - ay
        px = ax + rx * math.cos(th) - ry * math.sin(th)
        py = ay + rx * math.sin(th) + ry * math.cos(th)
        lever = math.hypot(rx, ry)
        err_meas = math.hypot(fx - gx, fy - gy)
        err_pred = 2.0 * lever * math.sin(abs(th) / 2.0)
        d_pred = math.hypot(fx - px, fy - py)     # 实测落点与"假说预测落点"的距离
        rows.append([m.group('tag'), ns, gx, gy, fx, fy, px, py,
                     lever, err_meas, err_pred, d_pred])
        print(f'{m.group("tag"):14s} {ns} 目标({gx:.2f},{gy:.2f}) '
              f'实测落点({fx:.3f},{fy:.3f}) 假说预测落点({px:.3f},{py:.3f}) '
              f'| 落点误差 实测{err_meas:.3f}m 假说预测{err_pred:.3f}m '
              f'| 实测与预测落点相距{d_pred:.3f}m')

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['tag', 'ns', 'goal_x', 'goal_y', 'final_x', 'final_y',
                    'pred_x', 'pred_y', 'lever_m', 'err_measured_m',
                    'err_predicted_m', 'dist_to_prediction_m'])
        w.writerows(rows)
    print(f'-> {OUT}')

    for grp in ('before', 'after'):
        v = [r[9] for r in rows if r[0].startswith(grp)]
        if not v:
            continue
        mean = sum(v) / len(v)
        sd = math.sqrt(sum((x - mean) ** 2 for x in v) / len(v)) if len(v) > 1 else 0.0
        print(f'== {grp:6s} 落点误差: {mean:.3f} ± {sd:.3f} m (N={len(v)}, '
              f'最大{max(v):.3f} m)')


if __name__ == '__main__':
    main()
