#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3（简化版）：巡检拍摄 -> 协同灭火 -> 编队返回。

跟 highrise.py 行为相同，实现全部搬进了 SDK。老版 747 行里，巡检站点循环、
拍照回传、让位放行 watchdog、发射机构时序、落点核对加起来 390 行，现在是
`sdk.patrol()` / `sdk.snapshot()` / `sdk.yield_spot()` / `sdk.shoot()` /
`sdk.return_home(sound=, report=)`。老版一行没动，两版并存。
"""
from contest_sdk import DroneSDK

ROUTE_TO_E = [(3, 3), (3, 22), (17, 22), (17, 16), (10, 16)]   # A B C G E
POINT_M, POINT_N = (6.0, 16.0), (14.0, 16.0)      # 观察位，正北 3.5 m 是 2#/1# 楼
POINT_E, POINT_G, POINT_D, POINT_A = (10.0, 16.0), (17.0, 16.0), (17.0, 3.0), (3.0, 3.0)
SUPPLY_XY, SUPPLY_TAG = (10.0, 8.0), 'apriltag:0'
HIGH_FIRE = 'apriltag:1'
# 火情只贴在楼的 -Y 面，所以"正对楼面"就是机头朝正北。任务机到发射点后**必须
# 显式转这一下**：从起飞到那里一次都没设过朝向，老版靠 NX02 出生朝向恰好也是
# 90° 蒙对了，那是巧合不是设计（见 highrise.py 里那段说明）。
FACADE_YAW_DEG = 90.0
# (标签, 观察位, 观察位名, 机头朝向°, 这站要不要查火情)。按 3#->2#->1# 走，
# 3# 只拍照——火情只可能在 1#/2#。2# 跟 3# 同在 M 点，原地转 180° 即可。
STATIONS = (('3#楼', POINT_M, 'M', -90.0, False),
            ('2#楼', POINT_M, 'M',  90.0, True),
            ('1#楼', POINT_N, 'N',  90.0, True))
PHOTO_DIR = '/logs/任务3照片'
CRUISE_AGL_M = OBSERVE_AGL_M = 2.0
FORWARD_BEFORE_FIRE_M = 1.5     # 对准后前移这么多再发射。⚠️ 这是弹丸的要求，不是可调裕度
SHOTS = 4
EV_FIRE, EV_AT_E, EV_BREACHED = 'high_fire_found', 'supply_at_standby', 'breach_done'
EV_DONE, EV_INSPECT_DONE = 'extinguish_done', 'inspect_done'
EV_SPOT_CLEAR, EV_HOME = 'spot_clear', 'supply_landed_home'


def recon(sdk: DroneSDK):
    """侦察机：巡检拍照 -> 遇火情插一轮协同灭火 -> 到 G 等 -> 编队返回 A 点。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV_AT_E, EV_DONE, EV_HOME)
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.fly_route(ROUTE_TO_E, agl_m=CRUISE_AGL_M, names=['A', 'B', 'C', 'G', 'E'])

    def on_fire(_det, bldg):
        """对准 -> 拍交付照 -> 前移到发射点 -> 通报 -> 等任务机到位 -> 破窗 -> 让位。"""
        sdk.aim_at(HIGH_FIRE, camera='front', what='高层火情')
        sdk.snapshot(bldg)
        spot = sdk.step_forward(FORWARD_BEFORE_FIRE_M, what='发射点')
        sdk.announce('侦察机通报高层火情')     # 先播报再发事件，否则跟"任务机起飞"抢顺序
        sdk.send_to_teammate(EV_FIRE, x=spot[0], y=spot[1], z=spot[2], at=bldg)
        sdk.wait_event(EV_AT_E, 300.0)
        sdk.shoot(label='发射破窗弹', sound='侦察机发射破窗弹')
        sdk.send_to_teammate(EV_BREACHED)
        sdk.announce('侦察机破窗完成')
        sdk.yield_spot(spot[:2], EV_SPOT_CLEAR)   # 发射完就走，走到哪算让开了

    fired = sdk.patrol(STATIONS, HIGH_FIRE, agl_m=OBSERVE_AGL_M, on_found=on_fire,
                       scan_sound='侦察机排查高层火情', found_sound='侦察机发现高楼火情')
    sdk.send_to_teammate(EV_INSPECT_DONE)
    sdk.fly_route([POINT_G], agl_m=OBSERVE_AGL_M, names=['G'])
    if fired:
        sdk.wait_event(EV_DONE, 420.0)
    sdk.lead_formation([POINT_G, POINT_D], agl_m=CRUISE_AGL_M,
                       start_xy=POINT_G, final_xy=POINT_A, disband_at=POINT_D,
                       tail_direct=True)
    sdk.hold_at(POINT_A[0], POINT_A[1], agl_m=CRUISE_AGL_M, seconds=15.0)
    # 等任务机真的落在自己起降点上再报完成；等不到也要报，不能卡死在这一步
    sdk.wait_event(EV_HOME, 300.0, required=False)
    sdk.announce('侦察机任务完成')


def supply(sdk: DroneSDK):
    """任务机：等通报 -> 取器材 -> E 点待命 -> 破窗后进场发射 -> 编队返回 -> 放器材 -> 降落。

    写成循环：1#/2# 两栋都可能着火，侦察机可能通报两次。
    """
    sdk.open_inbox(EV_FIRE, EV_BREACHED, EV_SPOT_CLEAR, EV_INSPECT_DONE)
    airborne = False
    while True:
        ev, d = sdk.wait_any_event([EV_FIRE, EV_INSPECT_DONE], 420.0)
        if ev == EV_INSPECT_DONE:
            break
        if not airborne:
            sdk.takeoff(height_m=CRUISE_AGL_M)
            sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火器材',
                           agl_m=CRUISE_AGL_M, sound='任务机抓取灭火弹')
            airborne = True
        sdk.goto_world(POINT_E[0], POINT_E[1], CRUISE_AGL_M, what='E点待命位')
        sdk.send_to_teammate(EV_AT_E)
        sdk.announce('任务机高层灭火已就位')
        sdk.wait_event(EV_BREACHED, 300.0)
        sdk.wait_event(EV_SPOT_CLEAR, 150.0)      # 发射点上这会儿还杵着侦察机
        sdk.goto_world(d['x'], d['y'], d['z'], what='侦察机发射点', accept_m=2.0)
        sdk.announce('任务机到达瞄准点')
        sdk.aim_at(HIGH_FIRE, camera='front', face_yaw_deg=FACADE_YAW_DEG,
                   what='高层火情')
        sdk.shoot(SHOTS, interval_s=1.0, label='发射灭火弹', sound='任务机发射灭火弹')
        sdk.send_to_teammate(EV_DONE)
        sdk.set_agl(CRUISE_AGL_M)        # 对准时被挪到了着火点高度，入列前收回来
    if not airborne:
        sdk.takeoff(height_m=CRUISE_AGL_M)        # 两栋楼都没着火，照样入列返航
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    sdk.release_at(SUPPLY_XY, SUPPLY_TAG, what='灭火器材', agl_m=CRUISE_AGL_M,
                   direct=True)
    sdk.return_home(sound='任务机已降落', report=EV_HOME, direct=True)


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
