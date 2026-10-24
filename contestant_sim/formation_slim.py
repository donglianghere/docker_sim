#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务1：编队飞行——长机带队跑航线，僚机保持纵向间距跟随。"""
from contest_sdk import DroneSDK

from venue import ROUTE_D, CRUISE_AGL_M
from rounds import ROUTE_FULL, FORMATION_STOPS


def leader(sdk: DroneSDK):
    """长机：起飞 -> 带队跑航线 -> 回自己起降点降落。"""
    sdk.takeoff(height_m=CRUISE_AGL_M)
    # 不传 stop_at_waypoints 则每个航点都停；候选见 rounds.FORMATION_STOPS。
    # final_xy 给自己的起降点：解散后直接朝家飞，不再绕 A 点中转。
    sdk.lead_formation(ROUTE_FULL, agl_m=CRUISE_AGL_M,
                       final_xy=sdk.own_pad(), disband_at=ROUTE_D,
                       tail_direct=True)
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='侦察机任务完成')


def follower(sdk: DroneSDK):
    """僚机：起飞 -> 入列跟队 -> 解散后回自己起降点降落。"""
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=leader, follower=follower, description=__doc__)
