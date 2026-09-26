#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只复现"从高层瞄准位置返航"这一段（临时诊断）。

起飞 -> 直飞到 2# 柱正对面的瞄准位置（世界 (-1, 7)，不经过规划器）-> 调用跟全流程
同一个返航函数。这条直线回起降点 (2,-9.5) 离 3# 柱心只有 0.27 米，等于正穿柱子，
正是 2026-09-25 实测撞柱的那一段。
"""
import time
from contest_sdk import DroneSDK
from contest_sdk.exceptions import ContestSdkError
import 高楼火情绕飞版示例 as 高楼

AIM = (-1.0, 7.0)
AGL = 2.5

def main():
    sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
    try:
        sdk.takeoff(height_m=AGL)
        p = sdk.world_to_local(AIM[0], AIM[1], AGL)
        print(f'[返航复现] 飞到瞄准位置 {AIM} -> 局部{tuple(round(v,2) for v in p)}'
              f'（**走规划器**：去程这条直线也正穿 3# 柱，用 goto_direct 摆位等于'
              f'自己开着无避障往柱子上撞，2026-09-25 第一次复现就栽在这儿）', flush=True)
        with sdk.fixed_altitude(p[2]):
            sdk.goto(*p)
        time.sleep(2.0)
        print('[返航复现] 就位，开始返航（跟全流程同一个函数）', flush=True)
        t0 = time.monotonic()
        try:
            高楼.recon_return_and_land(sdk)
            print(f'[返航复现] 返航结束，用时 {time.monotonic()-t0:.0f} 秒', flush=True)
        except ContestSdkError as exc:
            print(f'[返航复现] 返航失败：{exc}', flush=True)
    finally:
        sdk.shutdown()

if __name__ == '__main__':
    main()
