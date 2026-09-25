#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定向复现 ego-planner 的"轨迹尾端在障碍里 -> 永久重规划失败"死锁（临时诊断脚本）。

做法：先用 goto_direct（直飞、不经过规划器）把飞机送到障碍圆柱正南 1.5 米，
再 goto 圆柱正北的点——"当前位置 -> 目标点"的连线正穿圆柱，且起点离膨胀边界
只有 0.65 米（dist0=0.6 时），这正是 2026-09-25 那次卡死的几何条件。
"""
import time
from contest_sdk import DroneSDK
from contest_sdk.exceptions import ContestSdkError

CYL = (7.0, 2.0)
AGL = 2.0
import os
FIXED_ALT = os.environ.get('REPRO_FIXED_ALT', '1') == '1'   # 默认开定高（不开会被规划器压高度）

def main():
    sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
    try:
        sdk.takeoff()
        south = sdk.world_to_local(CYL[0], CYL[1] - 1.5, AGL)
        print(f'[复现] 直飞到圆柱正南 1.5 米 {south}', flush=True)
        sdk.goto_direct(*south, timeout=60.0)
        time.sleep(3.0)
        x, y, z = sdk.get_local_position()
        print(f'[复现] 就位：局部({x:.2f},{y:.2f},{z:.2f})，开始 goto 圆柱正北的点', flush=True)
        north = sdk.world_to_local(CYL[0], 9.5, AGL)
        t0 = time.monotonic()
        try:
            if FIXED_ALT:
                # 对照：打开定高再飞同一段，看"规划器把轨迹压下去"是否消失
                with sdk.fixed_altitude(north[2]):
                    sdk.goto(*north)
            else:
                sdk.goto(*north)
            print(f'[复现] 没卡住，{time.monotonic()-t0:.0f} 秒飞到了', flush=True)
        except ContestSdkError as exc:
            print(f'[复现] 卡住了（{time.monotonic()-t0:.0f} 秒）：{exc}', flush=True)
        x, y, z = sdk.get_local_position()
        print(f'[复现] 结束位置：局部({x:.2f},{y:.2f},{z:.2f})', flush=True)
    finally:
        sdk.shutdown()

if __name__ == '__main__':
    main()
