#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2（简化版）：地面火情侦查 -> 取物资 -> 投放灭火弹 -> 编队返航。

跟 groundfire.py 行为相同，实现全部搬进了 SDK。老版 recon() 89 行里有 47 行
（56%）是在跟线程、检测轮询、刹停时序、异常兜底打交道，现在是
`sdk.search_along()` 一个调用。老版一行没动，两版并存。
"""
from contest_sdk import DroneSDK


# 场地参数全在 venue.py——仿真与真机只有那一个文件不同，本程序两边逐字节相同。
from venue import (ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_D,
                   SUPPLY_XY, SUPPLY_TAG, GROUND_FIRE, CRUISE_AGL_M)

ROUTE_TO_G = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G]      # A B C G
POINT_E, POINT_G, POINT_D, POINT_A = ROUTE_E, ROUTE_G, ROUTE_D, ROUTE_A
PHOTO_DIR = '/logs/任务2照片'
EV_FIRE, EV_DROPPED = 'ground_fire_found', 'extinguisher_dropped'


def recon(sdk: DroneSDK):
    """侦察机：航点飞行 -> G->E 边飞边找火情 -> 通报 -> 回 G 等 -> 编队返航。"""
    # 只开需要的那一路相机（侦察机搜地面火情用下视）。仿真下是空操作。
    sdk.set_camera_mode('down')
    sdk.open_inbox(EV_DROPPED)
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.fly_route(ROUTE_TO_G, agl_m=CRUISE_AGL_M, names=['A', 'B', 'C', 'G'])

    fire = sdk.search_along(POINT_E, GROUND_FIRE, camera='down',
                            agl_m=CRUISE_AGL_M, what='地面火情')
    if fire is None:
        raise RuntimeError('没能定位地面火情，任务2 中止')
    sdk.announce('侦察机发现地面火情')
    sdk.announce('侦察机通报地面火情')
    sdk.send_to_teammate(EV_FIRE, x=fire[0], y=fire[1])

    sdk.hold_at(POINT_G[0], POINT_G[1], agl_m=CRUISE_AGL_M)
    sdk.wait_event(EV_DROPPED, 420.0)
    sdk.lead_formation([POINT_G, POINT_D], agl_m=CRUISE_AGL_M,
                       start_xy=POINT_G, final_xy=POINT_A, disband_at=POINT_D,
                       tail_direct=True)
    sdk.hold_at(POINT_A[0], POINT_A[1], agl_m=CRUISE_AGL_M, seconds=15.0)
    sdk.announce('侦察机任务完成')


def supply(sdk: DroneSDK):
    """任务机：等通报 -> 取灭火弹 -> 投放 -> 拍照回传 -> 编队返航 -> 降落。"""
    # 只开需要的那一路相机（任务机取弹精降 + 对准火点都用下视）。仿真下是空操作。
    sdk.set_camera_mode('down')
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV_FIRE)
    d = sdk.wait_event(EV_FIRE, 300.0)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火弹', agl_m=CRUISE_AGL_M,
                   sound='任务机抓取灭火弹')
    sdk.goto_world(d['x'], d['y'], CRUISE_AGL_M, what='地面火情点')
    sdk.aim_at(GROUND_FIRE, camera='down', what='地面火情')
    sdk.announce('任务机投放灭火弹')
    sdk.grip(release=True, label='投放')
    sdk.snapshot('地面火情已扑灭', camera='down')   # 灭火完成的交付凭证
    sdk.send_to_teammate(EV_DROPPED)

    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    sdk.return_home(sound='任务机已降落', direct=True)


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
