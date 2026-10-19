#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编队飞行（消重版）：长机带队跑航线，僚机保持纵向间距跟随。

跟 `formation_lite.py` **行为等价**，区别只是航线常量搬进了 `rounds.py`（四个任务
共用一份）。原版保留不动，两版可以并存、也可以混搭（事件名是同一组）。
"""
from contest_sdk import DroneSDK

from venue import ROUTE_A, ROUTE_D, CRUISE_AGL_M
from rounds import ROUTE_FULL, FORMATION_STOPS


def leader(sdk: DroneSDK):
    """长机：起飞 -> 带队跑航线 -> 回 A 点悬停待命（不降落）。"""
    sdk.takeoff(height_m=CRUISE_AGL_M)
    # disband_at / tail_direct 为什么这么给，见 rounds.formation_home 的说明。
    # stop_at_waypoints：2026-10-13 用户口径"编队段保留关键航点停留"。
    # 不给的话默认 9 个航点全停，哪些算关键见 rounds.FORMATION_STOPS 的说明。
    sdk.lead_formation(ROUTE_FULL, agl_m=CRUISE_AGL_M,
                       final_xy=ROUTE_A, disband_at=ROUTE_D, tail_direct=True)
    # sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M, seconds=15.0)
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='侦察机任务完成')
    # sdk.announce('侦察机任务完成')


def follower(sdk: DroneSDK):
    """僚机：起飞 -> 入列跟队 -> 解散后回自己起降点降落。"""
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    # 2026-10-13 去掉 direct=True：主航段改走 goto()（即 line_goto，直线+梯形
    # 剖面+终点速度为 0+落后/跟踪看门狗），跟长机同一套控制律。
    # direct=True 走的是 goto_direct -> precision_servo 的 coordinate_goto：
    # P 控制律、到点判据 0.3 m 球而减速区只有 0.06 m（满速交接）、
    # coordinate_goto_max_wait_s=0.0 永不超时。
    # 注意 return_home 末段那一小截精修仍然走 goto_direct（落点精度高一个
    # 量级，那是它分两段的原因），这里换掉的只是远距离主航段。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=leader, follower=follower, description=__doc__)
