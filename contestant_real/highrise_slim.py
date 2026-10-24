#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3：高层火情巡检拍摄 -> 协同破窗灭火 -> 编队返回。"""
from contest_sdk import DroneSDK

from venue import CRUISE_AGL_M
from rounds import (Events, ROUTE_TO_G, NAMES_TO_G, cruise_limit, formation_home,
                    recon_high, supply_high_once, supply_return_gear, use_camera)

#: snapshot() 的存图目录。
PHOTO_DIR = '/logs/任务3照片'
EV = Events(high='high_fire_found', at_e='supply_at_standby',
            breached='breach_done', spot_clear='spot_clear',
            done='extinguish_done', inspect_done='inspect_done')
#: 任务机 -> 侦察机：已落在自己起降点上。
EV_HOME = 'supply_landed_home'


def recon(sdk: DroneSDK):
    """侦察机：巡检拍照 -> 遇火情插一轮协同灭火 -> 到 G 等 -> 编队返航降落。"""
    use_camera(sdk, 'front')         # 巡检拍照 + 瞄高层火情用前视
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV.at_e, EV.done, EV_HOME)
    cruise_limit(sdk)
    sdk.takeoff(height_m=CRUISE_AGL_M)
    # 出发段只到 G：E 是巡检第一站，patrol 自己飞 G->E（两头都留 E 会下发零长度航段）。
    sdk.fly_route(ROUTE_TO_G, agl_m=CRUISE_AGL_M, names=NAMES_TO_G)
    recon_high(sdk, EV)
    # 编队返航的终点直接给自己的起降点，解散后不再绕 A 点中转。
    formation_home(sdk, sdk.own_pad())
    sdk.return_home(agl_m=CRUISE_AGL_M)
    # 等任务机落地再报完成；等不到也要报，不能卡死在这一步。
    sdk.wait_event(EV_HOME, 300.0, required=False)
    sdk.announce('侦察机任务完成')


def supply(sdk: DroneSDK):
    """任务机：等通报 -> 取器材 -> E 点待命 -> 破窗后发射 -> 编队返回 -> 还器材 -> 降落。"""
    use_camera(sdk, 'front')         # 瞄高层火情用前视
    sdk.open_inbox(EV.high, EV.breached, EV.spot_clear, EV.inspect_done)
    cruise_limit(sdk)
    airborne = False
    # 循环只为容错：赛制上两栋楼二选一，正常只通报一次。
    while True:
        ev, d = sdk.wait_any_event([EV.high, EV.inspect_done], 420.0)
        if ev == EV.inspect_done:
            break
        supply_high_once(sdk, d, EV, takeoff=not airborne)
        airborne = True
    if not airborne:
        sdk.takeoff(height_m=CRUISE_AGL_M)        # 没着火也要入列返航
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    if airborne:
        # 没着火那一支根本没取器材，不用白跑一趟物资点降落张爪。
        supply_return_gear(sdk)
    # return_home 不给 direct=True：出列后要横穿场地，走规划器避障。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落', report=EV_HOME)


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
