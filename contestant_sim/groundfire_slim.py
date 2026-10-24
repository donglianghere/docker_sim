#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2：地面火情侦查 -> 取物资 -> 投放灭火弹 -> 编队返航。"""
from contest_sdk import DroneSDK

from venue import ROUTE_E, ROUTE_G, GROUND_FIRE_ANY, CRUISE_AGL_M
from rounds import (Events, ROUTE_TO_G, NAMES_TO_G,
                    formation_home, recon_ground, supply_ground, use_camera)

EV = Events(ground='ground_fire_found', dropped='extinguisher_dropped')
#: snapshot() 的存图目录。
PHOTO_DIR = '/logs/任务2照片'
#: A->B 段的限速（米/秒），过了 B 点还原成原值。
AB_MAX_VEL_MPS = 0.8


def recon(sdk: DroneSDK):
    """侦察机：航点飞行 -> G->E 边飞边找火情 -> 通报 -> 回 G 等 -> 编队返航降落。"""
    use_camera(sdk, 'down')          # 搜地面火情用下视
    sdk.open_inbox(EV.dropped)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    # fly_route 不支持分段限速，所以拆成两次调用：前半段带限速、后半段还原。
    old_vel = sdk.set_max_vel(AB_MAX_VEL_MPS)
    try:
        sdk.fly_route(ROUTE_TO_G[:2], agl_m=CRUISE_AGL_M, names=NAMES_TO_G[:2])
    finally:
        # 读不到原值时 set_max_vel() 返回 0.0，不能拿 0 去设（会抛 ValueError）。
        if old_vel > 0:
            sdk.set_max_vel(old_vel)
    sdk.fly_route(ROUTE_TO_G[2:], agl_m=CRUISE_AGL_M, names=NAMES_TO_G[2:])

    # leg_start=ROUTE_G：机头航向取航线上 G->E 的方向角，不受到点误差影响。
    fire = sdk.search_along(ROUTE_E, GROUND_FIRE_ANY, camera='down',
                            agl_m=CRUISE_AGL_M, what='地面火情', leg_start=ROUTE_G)
    if fire is None:
        raise RuntimeError('没能定位地面火情，任务2 中止')
    recon_ground(sdk, EV, fire)
    # 编队返航的终点直接给自己的起降点，解散后不再绕 A 点中转。
    formation_home(sdk, sdk.own_pad())
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='侦察机任务完成')


def supply(sdk: DroneSDK):
    """任务机：等通报 -> 取灭火弹 -> 投放 -> 拍照回传 -> 编队返航 -> 降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV.ground)
    d = sdk.wait_event(EV.ground, 300.0)
    supply_ground(sdk, d, EV)
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    # return_home 不给 direct=True：出列后要横穿场地，走规划器避障。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
