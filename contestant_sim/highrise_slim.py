#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3（消重版）：巡检拍摄 -> 协同灭火 -> 编队返回。

跟 `highrise_lite.py` **行为等价**，巡检段/发射段/编队返航段搬进了 `rounds.py`
（与 mission 的高层火情轮共用一份）。原版保留不动，两版并存。
"""
from contest_sdk import DroneSDK

from venue import ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_E, ROUTE_G, CRUISE_AGL_M
from rounds import (Events, cruise_limit, formation_home, recon_high,
                    supply_high_once, supply_return_gear, use_camera)

ROUTE_TO_E = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E]
NAMES_TO_E = ['A', 'B', 'C', 'G', 'E']
PHOTO_DIR = '/logs/任务3照片'
EV = Events(high='high_fire_found', at_e='supply_at_standby',
            breached='breach_done', spot_clear='spot_clear',
            done='extinguish_done', inspect_done='inspect_done')
#: 任务机 -> 侦察机：已落在自己起降点上。侦察机等它再报"任务完成"。
EV_HOME = 'supply_landed_home'


def recon(sdk: DroneSDK):
    """侦察机：巡检拍照 -> 遇火情插一轮协同灭火 -> 到 G 等 -> 编队返回 A 点。"""
    use_camera(sdk, 'front')         # 巡检拍照 + 瞄高层火情用前视
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV.at_e, EV.done, EV_HOME)
    cruise_limit(sdk)                # 全程限速，见 rounds.cruise_limit
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.fly_route(ROUTE_TO_E, agl_m=CRUISE_AGL_M, names=NAMES_TO_E)
    recon_high(sdk, EV)
    formation_home(sdk, ROUTE_A)
    sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M, seconds=15.0)
    # 等任务机真的落在自己起降点上再报完成；等不到也要报，不能卡死在这一步
    sdk.wait_event(EV_HOME, 300.0, required=False)
    sdk.announce('侦察机任务完成')


def supply(sdk: DroneSDK):
    """任务机：等通报 -> 取器材 -> E 点待命 -> 破窗后进场发射 -> 编队返回 -> 还器材 -> 降落。

    写成循环：可能着火的那两栋楼，侦察机可能通报两次。巡检结束没着火时侦察机发
    `inspect_done`，循环就从这儿跳出去。

    ⚠️ **赛制上两栋楼只会二选一、不会同时着火**（用户 2026-10-18 明确），所以正常
    只跑一遍：侦察机破窗 1 发、任务机灭火 4 发（`venue.SHOTS`），全程各一次。这个
    循环留着只当容错，别据它去推"要发两次"。
    """
    use_camera(sdk, 'front')         # 瞄高层火情用前视
    sdk.open_inbox(EV.high, EV.breached, EV.spot_clear, EV.inspect_done)
    cruise_limit(sdk)                # 全程限速
    airborne = False
    while True:
        ev, d = sdk.wait_any_event([EV.high, EV.inspect_done], 420.0)
        if ev == EV.inspect_done:
            break
        supply_high_once(sdk, d, EV, takeoff=not airborne)
        airborne = True
    if not airborne:
        sdk.takeoff(height_m=CRUISE_AGL_M)        # 两栋楼都没着火，照样入列返航
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    supply_return_gear(sdk)
    # 2026-10-10 跟 mission 同步：return_home 不给 direct=True，回家走规划器避障。
    # 整段直飞不经规划器，中途出列时要横穿场地、擦着立柱过。默认 False 是「远距离
    # 走 goto(规划器+避障)、最后一段才直线精修落点」，落点精度不丢。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落', report=EV_HOME)


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
