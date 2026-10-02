#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编队飞行（简化版）：长机带队跑航线，僚机保持纵向间距跟随。

跟 formation.py 的关系：**行为相同，实现全部搬进了 SDK**。老版 841 行里有
420 行是编队控制（分段限速、航点握手、两种解散判据），现在是
`sdk.lead_formation()` / `sdk.follow_formation()` 两个调用。老版一行没动，
两版可以并存、也可以混搭（事件名是同一组）。
"""
from contest_sdk import DroneSDK


# 场地参数全在 venue.py——仿真与真机只有那一个文件不同，本程序两边逐字节相同。
from venue import (ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_D,
                   CRUISE_AGL_M)

ROUTE = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_G, ROUTE_D]
POINT_A, POINT_D = ROUTE_A, ROUTE_D


def leader(sdk: DroneSDK):
    """长机：起飞 -> 带队跑航线 -> 回 A 点悬停待命（不降落）。"""
    sdk.takeoff(height_m=CRUISE_AGL_M)
    # disband_at=D（航线最后一个航点，也是两机分头回家的分岔口）。不给的话
    # 默认判据是"长机飞回自己起飞点上空"，而那个点不在航线上、只是碰巧跟最后
    # 一段共线，编队会在半路散掉。
    sdk.lead_formation(ROUTE, agl_m=CRUISE_AGL_M,
                       final_xy=POINT_A, disband_at=POINT_D, tail_direct=True)
    sdk.hold_at(POINT_A[0], POINT_A[1], agl_m=CRUISE_AGL_M, seconds=15.0)
    sdk.announce('侦察机任务完成')


def follower(sdk: DroneSDK):
    """僚机：起飞 -> 入列跟队 -> 解散后回自己起降点降落。"""
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='station')
    sdk.return_home(sound='任务机已降落', direct=True)


if __name__ == '__main__':
    DroneSDK.run(leader=leader, follower=follower, description=__doc__)
