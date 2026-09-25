#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯爬升实验（临时诊断脚本）：只有"同一 x/y 从 1.5 米爬到 2.5 米"这一个变量。

起飞 -> 直飞到场地空旷处 1.5 米（不经过规划器）-> 对同一 x/y 发 2.5 米的 goto。
附近没有障碍物、没有水平运动，用来判断"爬不上去"到底是规划器把指令压掉了，
还是别的环节。同时对照跑一次开定高的版本。
"""
import os, time
from contest_sdk import DroneSDK
from contest_sdk.exceptions import ContestSdkError

SPOT = (2.0, -4.0)      # 世界坐标，空旷处：离 3# 立柱(0,0)、火点(0,-4)、圆柱(7,2) 都远
LOW, HIGH = 1.5, 2.5
FIXED = os.environ.get('CLIMB_FIXED_ALT', '0') == '1'

def main():
    sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
    try:
        sdk.takeoff()
        low = sdk.world_to_local(SPOT[0], SPOT[1], LOW)
        print(f'[爬升] 直飞到 {SPOT} 的 {LOW} m：局部{tuple(round(v,2) for v in low)}', flush=True)
        sdk.goto_direct(*low, timeout=60.0)
        time.sleep(3.0)
        print(f'[爬升] 就位：{tuple(round(v,2) for v in sdk.get_local_position())}', flush=True)

        high = sdk.world_to_local(SPOT[0], SPOT[1], HIGH)
        print(f'[爬升] 现在对同一 x/y 发 {HIGH} m 的 goto：局部{tuple(round(v,2) for v in high)}'
              f'，定高={"开" if FIXED else "关"}', flush=True)
        t0 = time.monotonic()
        try:
            if FIXED:
                with sdk.fixed_altitude(high[2]):
                    sdk.goto(*high)
            else:
                sdk.goto(*high)
            print(f'[爬升] 到了，用时 {time.monotonic()-t0:.0f} 秒', flush=True)
        except ContestSdkError as exc:
            print(f'[爬升] 失败（{time.monotonic()-t0:.0f} 秒）：{exc}', flush=True)
        print(f'[爬升] 结束位置：{tuple(round(v,2) for v in sdk.get_local_position())}', flush=True)
    finally:
        sdk.shutdown()

if __name__ == '__main__':
    main()
