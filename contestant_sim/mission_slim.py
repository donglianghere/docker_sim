#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合任务：三轮连贯——第 1 轮编队飞行，第 2、3 轮两种火情各一轮、不重复。"""
from contest_sdk import DroneSDK

from venue import (ROUTE_A, ROUTE_D, ROUTE_E, ROUTE_G, GROUND_FIRE_ANY, CRUISE_AGL_M)
from rounds import (Events, ROUTE_FULL, ROUTE_TO_G, NAMES_TO_G, cruise_limit,
                    formation_home, recon_ground, recon_high, supply_ground,
                    supply_high_once, supply_return_gear, use_camera)

#: snapshot() 的存图目录。
PHOTO_DIR = '/logs/综合任务照片'
KIND_GROUND, KIND_HIGH = 'ground', 'high'
EV = Events(ground='mission_ground_fire', dropped='mission_dropped',
            high='mission_high_fire', at_e='mission_at_standby',
            breached='mission_breached', spot_clear='mission_spot_clear',
            done='mission_extinguished', inspect_done='mission_inspect_done')
#: 侦察机 -> 任务机：本轮是哪种火情，任务机据此直接走对应分支。
EV_KIND = 'mission_round_kind'
#: 任务机 -> 侦察机：本轮已降落停稳，侦察机靠它卡下一轮的起飞时机。
EV_ROUND_DONE = 'mission_round_done'


# ---------------------------------------------------------------------------
# 侦察机 NX01
# ---------------------------------------------------------------------------
def _home_after_round(sdk: DroneSDK, last: bool) -> None:
    """一轮火情处置完的编队返航：last=True（第 3 轮）直接回起降点降落。"""
    formation_home(sdk, sdk.own_pad() if last else ROUTE_A)
    if last:
        sdk.return_home(agl_m=CRUISE_AGL_M)
    else:
        sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M)


def recon(sdk: DroneSDK):
    """侦察机：编队 -> 判断并处置第一种火情 -> 处置另一种 -> 回起飞点降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV.dropped, EV.at_e, EV.done, EV_ROUND_DONE)
    cruise_limit(sdk)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('===== 第 1 轮：编队飞行 =====')
    # 给飞行报告分段计时：监视器按相邻两条 mark_phase 的时间差算每圈用时。
    sdk.mark_phase('第1圈编队飞行')
    use_camera(sdk, 'none')          # 这一轮不用相机
    sdk.lead_formation(ROUTE_FULL, agl_m=CRUISE_AGL_M,
                       final_xy=ROUTE_A, disband_at=ROUTE_D, tail_direct=True)
    sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M)

    done = []
    for rnd in (2, 3):
        sdk.wait_event(EV_ROUND_DONE, 900.0)      # 等任务机本轮降落停稳
        sdk.progress(f'===== 第 {rnd} 轮：开始巡检 =====')
        sdk.mark_phase(f'第{rnd}圈协同灭火')
        # 两轮不重复：上轮处置过地面火情，本轮必然是高层，不用再搜一遍。
        may_be_ground = KIND_GROUND not in done
        # 相机在起飞前配好，切换的 4 秒被 A->B->C->G 这段飞行盖住。
        use_camera(sdk, 'down' if may_be_ground else 'front')
        sdk.fly_route(ROUTE_TO_G, agl_m=CRUISE_AGL_M, names=NAMES_TO_G)
        if may_be_ground:
            # leg_start=ROUTE_G：机头航向取航线上 G->E 的方向角，不受到点误差影响。
            fire_xy = sdk.search_along(ROUTE_E, GROUND_FIRE_ANY, camera='down',
                                       agl_m=CRUISE_AGL_M, what='地面火情',
                                       leg_start=ROUTE_G)
        else:
            # 已知本轮是高层：G->E 这一段仍要飞到，但不开下视、不做搜索。
            sdk.goto_world(ROUTE_E[0], ROUTE_E[1], agl_m=CRUISE_AGL_M, what='E点')
            fire_xy = None
        kind = KIND_GROUND if fire_xy else KIND_HIGH
        if kind == KIND_HIGH and may_be_ground:
            use_camera(sdk, 'front')      # 搜了一路没有地面火情，换前视
        sdk.send_to_teammate(EV_KIND, kind=kind, round=rnd)
        sdk.progress(f'本轮判定：{"地面火情" if kind == KIND_GROUND else "高层火情"}')
        if kind == KIND_GROUND:
            assert fire_xy is not None, 'kind=KIND_GROUND 却没有火情坐标'
            recon_ground(sdk, EV, fire_xy)
        else:
            recon_high(sdk, EV)
        _home_after_round(sdk, last=(rnd == 3))
        done.append(kind)

    sdk.wait_event(EV_ROUND_DONE, 900.0)          # 等任务机收尾
    # 收尾也要打一条，否则第 3 圈没有结束时刻、算不出用时。
    sdk.mark_phase('完成')
    sdk.progress('===== 三轮全部完成 =====')
    sdk.announce('侦察机任务完成')


# ---------------------------------------------------------------------------
# 任务机 NX02
# ---------------------------------------------------------------------------
def supply(sdk: DroneSDK):
    """任务机：编队 -> 按侦察机通报的种类处置两轮火情 -> 每轮都回起降点降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV_KIND, EV.ground, EV.high, EV.breached, EV.spot_clear,
                   EV.inspect_done)
    cruise_limit(sdk)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('===== 第 1 轮：编队飞行 =====')
    use_camera(sdk, 'none')          # 这一轮不用相机
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    # return_home 不给 direct=True：出列后要横穿场地，走规划器避障。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落', report=EV_ROUND_DONE)

    for rnd in (2, 3):
        d = sdk.wait_event(EV_KIND, 900.0)
        kind = str(d.get('kind', KIND_GROUND))
        sdk.progress(f'===== 第 {rnd} 轮：'
                     f'{"地面火情" if kind == KIND_GROUND else "高层火情"} =====')
        fought = False
        if kind == KIND_GROUND:
            d = sdk.wait_event(EV.ground, 420.0)
            supply_ground(sdk, d, EV)
            fought = True
        else:
            # kind=high 只说明"这一轮不是地面火情"，**不保证**侦察机真的看见了
            # 高层火情。它巡检完三栋楼没发现时会发 inspect_done，这里必须跟
            # EV.high 一起等——只等 EV.high 的话会干等 420 秒，而侦察机那会儿
            # 已经在 G 点等僚机入列，两边互等、双机悬停不动。
            # highrise_slim 早就有这道兜底（wait_any_event），综合任务这里漏了。
            ev_name, d = sdk.wait_any_event([EV.high, EV.inspect_done], 420.0)
            if ev_name == EV.high:
                supply_high_once(sdk, d, EV)
                fought = True
            else:
                sdk.progress('侦察机巡检完没发现高层火情，本轮不灭火，直接入列返航')
                # 没走 supply_high_once 就没起飞过（上一轮 return_home 落地了），
                # 要自己起飞才能入列。
                sdk.takeoff(height_m=CRUISE_AGL_M)
        sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
        if kind == KIND_HIGH and fought:
            supply_return_gear(sdk)   # 器材抓在手上带去发射的，打完要还回物资点
        sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落',
                        report=EV_ROUND_DONE)

    sdk.progress('===== 三轮全部完成 =====')


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
