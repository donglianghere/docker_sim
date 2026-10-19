#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合模拟验证（消重版）：三轮连贯 —— 编队飞行 + 两种火情各一轮，不重复。

第 1 轮纯编队；第 2、3 轮侦察机飞到 G 点后，裁判（scripts/referee.py）才把本轮
抽中的火情放出来，所以到 G 之前场上什么都没有。侦察机沿 G->E 用下视相机搜一遍：
搜到就是地面火情轮，搜不到就是高层火情轮，判定结果通报给任务机。

跟 `mission_lite.py` **行为等价**，各轮的飞行段搬进了 `rounds.py`（与 formation /
groundfire / highrise 共用一份）。原版保留不动，两版并存。
"""
from contest_sdk import DroneSDK

from venue import (ROUTE_A, ROUTE_D, ROUTE_E, ROUTE_G, GROUND_FIRE, CRUISE_AGL_M)
from rounds import (Events, ROUTE_FULL, ROUTE_TO_G, NAMES_TO_G, cruise_limit,
                    formation_home, recon_ground, recon_high, supply_ground,
                    supply_high_once, supply_return_gear, use_camera)

PHOTO_DIR = '/logs/综合任务照片'
KIND_GROUND, KIND_HIGH = 'ground', 'high'
EV = Events(ground='mission_ground_fire', dropped='mission_dropped',
            high='mission_high_fire', at_e='mission_at_standby',
            breached='mission_breached', spot_clear='mission_spot_clear',
            done='mission_extinguished')
#: 侦察机 -> 任务机：本轮是哪种火情。任务机据此直接走对应分支，所以**不需要**
#: highrise 那条 `inspect_done` 握手（EV 里那一项留空）。
EV_KIND = 'mission_round_kind'
#: 任务机 -> 侦察机：本轮已降落停稳。侦察机靠它卡住下一轮的起飞时机。
EV_ROUND_DONE = 'mission_round_done'


# ---------------------------------------------------------------------------
# 侦察机 NX01
# ---------------------------------------------------------------------------
def _home_after_round(sdk: DroneSDK, last: bool) -> None:
    """一轮火情处置完的编队返航。

    last=True（第 3 轮）：**不去 A 点了**，编队终点直接设成自己的起降点，解散后
    就地降落。回家走规划器避障（不给 direct=True，理由见 highrise_slim 尾段）。
    """
    formation_home(sdk, sdk.own_pad() if last else ROUTE_A)
    if last:
        sdk.return_home(agl_m=CRUISE_AGL_M)
    else:
        sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M)


def recon(sdk: DroneSDK):
    """侦察机：编队 -> 判断并处置第一种火情 -> 处置另一种 -> 回起飞点降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV.dropped, EV.at_e, EV.done, EV_ROUND_DONE)
    cruise_limit(sdk)                # 全程限速，见 rounds.cruise_limit
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('===== 第 1 轮：编队飞行 =====')
    use_camera(sdk, 'none')          # 这一轮只有 lead_formation + hold_at，两路全停
    # disband_at / tail_direct 为什么这么给，见 rounds.formation_home 的说明。
    sdk.lead_formation(ROUTE_FULL, agl_m=CRUISE_AGL_M,
                       final_xy=ROUTE_A, disband_at=ROUTE_D, tail_direct=True)
    sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M)

    done = []
    for rnd in (2, 3):
        sdk.wait_event(EV_ROUND_DONE, 900.0)      # 等任务机本轮降落停稳
        sdk.progress(f'===== 第 {rnd} 轮：开始巡检 =====')
        # 两轮不重复：地面火情上轮处置过的话，本轮必然是高层，不用再搜一遍。这个
        # 判断在**起飞前**就成立，所以相机也在这儿配好——A->B->C->G 整段飞行把切换
        # 的 4 秒吸收掉，比到 G 之后再切更早、更不占时间。
        may_be_ground = KIND_GROUND not in done
        use_camera(sdk, 'down' if may_be_ground else 'front')
        sdk.fly_route(ROUTE_TO_G, agl_m=CRUISE_AGL_M, names=NAMES_TO_G)
        if may_be_ground:
            # leg_start=ROUTE_G：机头航向严格取航线上 G->E 的方向角，而不是"飞机
            # 停在 G 的实际位置 -> E"（到点误差会让后者偏几度）。
            fire_xy = sdk.search_along(ROUTE_E, GROUND_FIRE, camera='down',
                                       agl_m=CRUISE_AGL_M, what='地面火情',
                                       leg_start=ROUTE_G)
        else:
            # 2026-10-12 用户要求：**高层火情轮侦察机也必须经过 E 点**。原来这里
            # 直接跳过 G->E 整段（"本轮必然是高层，不用再搜一遍"），省了一段飞行
            # 时间，但赛题要求这一段必须飞到。只在"已知本轮是高层"时走这条分支，
            # 所以**不开相机**（用户 2026-10-12 指定）：保持前视、用普通 goto 飞
            # 过去，不做下视搜索，省掉一路 yolo（约整机 12% CPU）和两次 4 秒切换。
            sdk.goto_world(ROUTE_E[0], ROUTE_E[1], agl_m=CRUISE_AGL_M, what='E点')
            fire_xy = None
        kind = KIND_GROUND if fire_xy else KIND_HIGH
        if kind == KIND_HIGH and may_be_ground:
            # 过了 E 点整段都没看到地面火情 -> 本轮是高层。这时才把下视换成前视
            # （上面"已知是高层"那条路已经开着前视了，不用重复切）。
            use_camera(sdk, 'front')
        sdk.send_to_teammate(EV_KIND, kind=kind, round=rnd)
        sdk.progress(f'本轮判定：{"地面火情" if kind == KIND_GROUND else "高层火情"}')
        if kind == KIND_GROUND:
            # kind 就是由 fire_xy 真假推出来的，这里必然非 None。写出来有两个
            # 用处：①把这个不变量显式化，万一上面的判定逻辑以后改坏了，在这儿
            # 就地炸、而不是把 None 传进去在 fire_xy[0] 处报一个看不懂的错；
            # ②pyright 没法穿过 kind 这个中间变量做收窄，不加它会报
            # reportArgumentType（2026-10-14 清检查时修）。
            assert fire_xy is not None, 'kind=KIND_GROUND 却没有火情坐标'
            recon_ground(sdk, EV, fire_xy)
        else:
            recon_high(sdk, EV)
        _home_after_round(sdk, last=(rnd == 3))
        done.append(kind)

    sdk.wait_event(EV_ROUND_DONE, 900.0)          # 等任务机收尾
    sdk.progress('===== 三轮全部完成 =====')
    sdk.announce('侦察机任务完成')


# ---------------------------------------------------------------------------
# 任务机 NX02
# ---------------------------------------------------------------------------
def supply(sdk: DroneSDK):
    """任务机：编队 -> 按侦察机通报的种类处置两轮火情 -> 每轮都回起降点降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV_KIND, EV.ground, EV.high, EV.breached, EV.spot_clear)
    cruise_limit(sdk)                # 全程限速
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('===== 第 1 轮：编队飞行 =====')
    use_camera(sdk, 'none')          # 这一轮只有 follow_formation + return_home
    # 2026-10-10 用户要求：join 由 'station' 改 'nearest'（就地入列）。'station'
    # 会先飞到「长机起飞点后方 spacing(5.0) 米」那个点，而两机实际摆放间距只有约
    # 3.99 米，于是僚机起飞后先横移 1.0 米补差——看起来像「往右移动一下」。
    # 'nearest' 跳过这趟预站位，初始 lag 带约 1 米误差，小于入列容差 1.2 米，再由
    # FORMATION_CATCHUP_GAIN=0.3 的追赶吃掉。
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    # 2026-10-10 用户要求：去掉 direct=True，回家走规划器避障。整段直飞不经规划器，
    # 而僚机可能在**航线中段**被迫出列（长机 goto 失败时 lead_formation 的 finally
    # 会补发 route_done），那时直飞回家要横穿场地、擦着立柱过。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落', report=EV_ROUND_DONE)

    for rnd in (2, 3):
        d = sdk.wait_event(EV_KIND, 900.0)
        kind = str(d.get('kind', KIND_GROUND))
        sdk.progress(f'===== 第 {rnd} 轮：'
                     f'{"地面火情" if kind == KIND_GROUND else "高层火情"} =====')
        if kind == KIND_GROUND:
            d = sdk.wait_event(EV.ground, 420.0)
            supply_ground(sdk, d, EV)
        else:
            d = sdk.wait_event(EV.high, 420.0)
            supply_high_once(sdk, d, EV)
        sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
        if kind == KIND_HIGH:
            # 器材是抓在手上带去发射的，打完还在机上 -> 解散后先还回物资点。
            supply_return_gear(sdk)
        sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落',
                        report=EV_ROUND_DONE)

    sdk.progress('===== 三轮全部完成 =====')


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
