#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合模拟验证（简化版）：三轮连贯 —— 编队飞行 + 两种火情各一轮，不重复。

第 1 轮纯编队；第 2、3 轮侦察机飞到 G 点后，裁判（scripts/referee.py）才把
本轮抽中的火情放出来，所以到 G 之前场上什么都没有。侦察机沿 G->E 用下视相机
搜一遍：搜到就是地面火情轮，搜不到就是高层火情轮，判定结果通报给任务机。

跟 mission.py 行为相同，实现全部搬进了 SDK。老版 617 行里绝大部分是编队控制、
巡检循环、让位放行、收件箱管理，现在都是 SDK 调用。老版一行没动，两版并存。
"""
from contest_sdk import DroneSDK

ROUTE_A, ROUTE_B, ROUTE_C = (3.0, 3.0), (3.0, 22.0), (17.0, 22.0)
ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_D = (17.0, 16.0), (10.0, 16.0), (14.0, 14.0), (17.0, 3.0)
FORMATION_ROUTE = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_G, ROUTE_D]
ROUTE_TO_G = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G]
POINT_M, POINT_N = (6.0, 16.0), (14.0, 16.0)
SUPPLY_XY, SUPPLY_TAG = (10.0, 8.0), 'apriltag:0'
GROUND_FIRE, HIGH_FIRE = 'apriltag:2', 'apriltag:1'
# 火情只贴在楼的 -Y 面 -> 正对楼面就是机头朝正北。任务机**必须显式转这一下**：
# 第 1 轮编队最后一段 D->A 航向 180°，带着它到发射点会偏 89°、标识出半视场。
FACADE_YAW_DEG = 90.0
STATIONS = (('3#楼', POINT_M, 'M', -90.0, False),
            ('2#楼', POINT_M, 'M',  90.0, True),
            ('1#楼', POINT_N, 'N',  90.0, True))
PHOTO_DIR = '/logs/综合任务照片'
CRUISE_AGL_M = OBSERVE_AGL_M = 2.0
FORWARD_BEFORE_FIRE_M, SHOTS = 1.5, 4
KIND_GROUND, KIND_HIGH = 'ground', 'high'

EV_KIND, EV_GROUND, EV_DROPPED = 'mission_round_kind', 'mission_ground_fire', 'mission_dropped'
EV_HIGH, EV_AT_E, EV_BREACHED = 'mission_high_fire', 'mission_at_standby', 'mission_breached'
EV_SPOT_CLEAR, EV_EXTINGUISHED = 'mission_spot_clear', 'mission_extinguished'
EV_ROUND_DONE = 'mission_round_done'
RECON_EVENTS = (EV_DROPPED, EV_AT_E, EV_EXTINGUISHED, EV_ROUND_DONE)
SUPPLY_EVENTS = (EV_KIND, EV_GROUND, EV_HIGH, EV_BREACHED, EV_SPOT_CLEAR)


# ---------------------------------------------------------------------------
# 侦察机 NX01
# ---------------------------------------------------------------------------
def _formation_home_recon(sdk: DroneSDK, last):
    """从 G 起编队返航，任务机过 D 点就解散。

    last=True（第 3 轮）：**不去 A 点了**，编队终点直接设成自己的起降点，
    解散后就地降落。
    """
    final = sdk.own_pad() if last else ROUTE_A
    sdk.lead_formation([ROUTE_G, ROUTE_D], agl_m=CRUISE_AGL_M,
                       start_xy=ROUTE_G, final_xy=final, disband_at=ROUTE_D,
                       tail_direct=True)
    if last:
        sdk.return_home(direct=True)
    else:
        sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M)


def _ground_round_recon(sdk: DroneSDK, fire_xy, last):
    """地面火情轮：通报 -> 回 G 点等任务机投弹 -> 编队返航。"""
    sdk.announce('侦察机发现地面火情')
    sdk.announce('侦察机通报地面火情')   # 先播报再发事件，否则跟"任务机起飞"抢顺序
    sdk.send_to_teammate(EV_GROUND, x=fire_xy[0], y=fire_xy[1])
    sdk.hold_at(ROUTE_G[0], ROUTE_G[1], agl_m=CRUISE_AGL_M)
    sdk.wait_event(EV_DROPPED, 420.0)
    _formation_home_recon(sdk, last)


def _high_round_recon(sdk: DroneSDK, last):
    """高层火情轮：三栋楼巡检拍照 -> 发现火情就协同灭火 -> 回 G 编队返航。"""
    def on_fire(_det, bldg):
        sdk.aim_at(HIGH_FIRE, camera='front', what='高层火情')
        sdk.snapshot(bldg)
        spot = sdk.step_forward(FORWARD_BEFORE_FIRE_M, what='发射点')
        sdk.announce('侦察机通报高层火情')
        sdk.send_to_teammate(EV_HIGH, x=spot[0], y=spot[1], z=spot[2], at=bldg)
        sdk.wait_event(EV_AT_E, 300.0)
        sdk.shoot(label='发射破窗弹', sound='侦察机发射破窗弹')
        sdk.send_to_teammate(EV_BREACHED)
        sdk.announce('侦察机破窗完成')
        sdk.yield_spot(spot[:2], EV_SPOT_CLEAR)   # 发射完就走，走到哪算让开了

    fired = sdk.patrol(STATIONS, HIGH_FIRE, agl_m=OBSERVE_AGL_M, on_found=on_fire,
                       scan_sound='侦察机排查高层火情', found_sound='侦察机发现高楼火情')
    sdk.fly_route([ROUTE_G], agl_m=OBSERVE_AGL_M, names=['G'])
    if fired:
        sdk.wait_event(EV_EXTINGUISHED, 420.0)
    _formation_home_recon(sdk, last)


def recon(sdk: DroneSDK):
    """侦察机：编队 -> 判断并处置第一种火情 -> 处置另一种 -> 回起飞点降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(*RECON_EVENTS)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('===== 第 1 轮：编队飞行 =====')
    # disband_at=D 要显式给。不给的话默认判据是"长机飞回**自己起飞点**上空"，
    # 而 (8,3) 根本不在航线上，只是碰巧落在最后一段 D(17,3)->A(3,3) 的连线上
    # （都在 y=3）——编队会在离终点还有 5 米的半路上散掉，散在一个跟任务无关的
    # 点上；起降点哪天挪开这条线，这个判据就永远不成立。D 点是两机路径真正的
    # 分岔口（僚机回 (12,3)、长机去 A），跟第 2/3 轮同一套规则。
    sdk.lead_formation(FORMATION_ROUTE, agl_m=CRUISE_AGL_M,
                       final_xy=ROUTE_A, disband_at=ROUTE_D, tail_direct=True)
    sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M)

    done = []
    for rnd in (2, 3):
        sdk.wait_event(EV_ROUND_DONE, 900.0)      # 等任务机本轮降落停稳
        sdk.progress(f'===== 第 {rnd} 轮：开始巡检 =====')
        # 裁判盯着"侦察机过 G 点"才放火情，到 G 之前场上什么都没有。
        sdk.fly_route(ROUTE_TO_G, agl_m=CRUISE_AGL_M, names=['A', 'B', 'C', 'G'])
        if KIND_GROUND in done:
            # 两轮不重复：地面火情上轮处置过 -> 本轮必然是高层，不用再搜一遍
            fire_xy = None
        else:
            fire_xy = sdk.search_along(ROUTE_E, GROUND_FIRE, camera='down',
                                       agl_m=CRUISE_AGL_M, what='地面火情')
        kind = KIND_GROUND if fire_xy else KIND_HIGH
        sdk.send_to_teammate(EV_KIND, kind=kind, round=rnd)
        sdk.progress(f'本轮判定：{"地面火情" if kind == KIND_GROUND else "高层火情"}')
        if kind == KIND_GROUND:
            _ground_round_recon(sdk, fire_xy, last=(rnd == 3))
        else:
            _high_round_recon(sdk, last=(rnd == 3))
        done.append(kind)

    sdk.wait_event(EV_ROUND_DONE, 900.0)          # 等任务机收尾
    sdk.progress('===== 三轮全部完成 =====')
    sdk.announce('侦察机任务完成')


# ---------------------------------------------------------------------------
# 任务机 NX02
# ---------------------------------------------------------------------------
def _ground_round_supply(sdk: DroneSDK):
    """地面火情轮：起飞 -> 取灭火弹 -> 投放 -> 拍照回传 -> 编队返航。"""
    d = sdk.wait_event(EV_GROUND, 420.0)
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
    # 灭火弹已经投到火点上了，没东西可还，直接回家


def _high_round_supply(sdk: DroneSDK):
    """高层火情轮：起飞取器材 -> E 点待命 -> 破窗后进场发射 -> 编队返航 -> 还器材。"""
    d = sdk.wait_event(EV_HIGH, 420.0)
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火器材', agl_m=CRUISE_AGL_M,
                   sound='任务机抓取灭火弹')
    sdk.goto_world(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M, what='E点待命位')
    sdk.send_to_teammate(EV_AT_E)
    sdk.announce('任务机高层灭火已就位')
    sdk.wait_event(EV_BREACHED, 300.0)
    sdk.wait_event(EV_SPOT_CLEAR, 150.0)          # 发射点上这会儿还杵着侦察机
    sdk.goto_world(d['x'], d['y'], d['z'], what='侦察机发射点', accept_m=2.0)
    sdk.announce('任务机到达瞄准点')
    sdk.aim_at(HIGH_FIRE, camera='front', face_yaw_deg=FACADE_YAW_DEG, what='高层火情')
    sdk.shoot(SHOTS, interval_s=1.0, label='发射灭火弹', sound='任务机发射灭火弹')
    sdk.send_to_teammate(EV_EXTINGUISHED)
    sdk.set_agl(CRUISE_AGL_M)        # 对准时被挪到了着火点高度，入列前收回来
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    # 器材是抓在手上带去发射的，打完还在机上 -> 解散后先还回物资点
    sdk.release_at(SUPPLY_XY, SUPPLY_TAG, what='灭火器材', agl_m=CRUISE_AGL_M,
                   direct=True)


def supply(sdk: DroneSDK):
    """任务机：编队 -> 按侦察机通报的种类处置两轮火情 -> 每轮都回起降点降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(*SUPPLY_EVENTS)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    sdk.progress('===== 第 1 轮：编队飞行 =====')
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='station')
    sdk.return_home(sound='任务机已降落', report=EV_ROUND_DONE, direct=True)

    for rnd in (2, 3):
        d = sdk.wait_event(EV_KIND, 900.0)
        kind = str(d.get('kind', KIND_GROUND))
        sdk.progress(f'===== 第 {rnd} 轮：'
                     f'{"地面火情" if kind == KIND_GROUND else "高层火情"} =====')
        (_ground_round_supply if kind == KIND_GROUND else _high_round_supply)(sdk)
        sdk.return_home(sound='任务机已降落', report=EV_ROUND_DONE, direct=True)

    sdk.progress('===== 三轮全部完成 =====')


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
