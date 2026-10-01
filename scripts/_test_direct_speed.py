#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实测 set_direct_speed() 到底有没有用：goto_direct 在不同限速下的真实速度。

背景：`coordinate_max_speed_mps` 曾经**设了也不管用**——不管填 0.3/0.5/1.0，
实际速度死死卡在约 0.02 m/s（precision_servo_node 发给 pt4ctrl 的 cmd.velocity
一直是全零，PX4 把"位置+零速度"当成"这点应该静止悬停"）。2026-09-15 补了速度
前馈之后应该跟得上了，但一直没人量过。这个脚本就是去量。

测速航段 (6,6) -> (14,6)，长 8 m，离最近障碍（仿地模块）2.24 m，离墙 6 m，
离避障圆柱 3.6 m，离 NX02 停机坪 3 m。**直飞没有避障，这条线是算过的。**
"""
import math
import threading
import time

from contest_sdk import DroneSDK

AGL = 2.0
P0, P1 = (6.0, 6.0), (14.0, 6.0)
SPEEDS = (0.3, 0.6, 1.0, 1.5)


def main():
    sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
    try:
        sdk.takeoff(height_m=AGL)
        sdk.goto_world(P0[0], P0[1], AGL, what='测速起点')   # 走规划器过去，安全
        here, far = P0, P1
        print('\n限速设定   实际用时   直线距离   平均速度   峰值速度(1s窗)', flush=True)
        for v in SPEEDS:
            sdk.set_direct_speed(v)
            lx, ly, lz = sdk.world_to_local(far[0], far[1], AGL)
            samples = []
            stop = threading.Event()

            def _sample():
                while not stop.is_set():
                    try:
                        x, y, _ = sdk.get_local_position()
                        samples.append((time.time(), x, y))
                    except Exception:
                        pass
                    time.sleep(0.1)

            th = threading.Thread(target=_sample, daemon=True)
            th.start()
            t0 = time.time()
            sdk.goto_direct(lx, ly, lz)
            dt = time.time() - t0
            stop.set()
            th.join(timeout=2.0)

            dist = math.dist(here, far)
            peak = 0.0
            for i in range(len(samples)):
                for j in range(i + 1, len(samples)):
                    if samples[j][0] - samples[i][0] >= 1.0:
                        d = math.dist(samples[i][1:], samples[j][1:])
                        peak = max(peak, d / (samples[j][0] - samples[i][0]))
                        break
            print(f'  {v:4.1f} m/s  {dt:7.1f} s  {dist:7.1f} m  '
                  f'{dist / dt:7.2f} m/s  {peak:7.2f} m/s', flush=True)
            here, far = far, here          # 下一轮往回飞
        sdk.set_direct_speed(0.3)          # 恢复出厂默认，别留着高限速影响后面的精修
        sdk.return_home()
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()
