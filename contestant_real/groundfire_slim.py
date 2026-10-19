#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2（消重版）：地面火情侦查 -> 取物资 -> 投放灭火弹 -> 编队返航。

跟 `groundfire_lite.py` **行为等价**，通报段/投弹段/编队返航段搬进了 `rounds.py`
（与 mission 的地面火情轮共用一份）。原版保留不动，两版并存。
"""
from contest_sdk import DroneSDK

from venue import ROUTE_A, ROUTE_E, ROUTE_G, GROUND_FIRE, CRUISE_AGL_M
from rounds import (Events, ROUTE_TO_G, NAMES_TO_G,
                    formation_home, recon_ground, supply_ground, use_camera)

EV = Events(ground='ground_fire_found', dropped='extinguisher_dropped')
PHOTO_DIR = '/logs/任务2照片'
#: A->B 这一段单独限速，米/秒（2026-10-09 用户指定）。过了 B 点就还原成原值。
AB_MAX_VEL_MPS = 0.8


def recon(sdk: DroneSDK):
    """侦察机：航点飞行 -> G->E 边飞边找火情 -> 通报 -> 回 G 等 -> 编队返航。"""
    use_camera(sdk, 'down')          # 搜地面火情用下视；仿真下是空操作
    sdk.open_inbox(EV.dropped)
    sdk.takeoff(height_m=CRUISE_AGL_M)

    # A->B 单独限速。fly_route 不支持分段限速（内部对每个航点循环调 goto），所以
    # 拆成两次调用：前半段带限速、后半段还原。set_max_vel() 改的是 ego_planner 的
    # manager/max_vel，是**节点级**参数、不会自己复位，所以用 try/finally 还回原值
    # ——不然后面 C->G、搜索段和编队返航都会带着这个限速跑。
    #   拆开的唯一行为差异：fly_route 内部用**上一个航点的理论坐标**算下一段航向，
    # 而第二次调用会重新 get_local_position()，所以 B->C 的航向按 B 点的实际落点
    # 算、可能偏几度。每个航点本来都要 face_yaw + 停 2 秒，这点偏差无妨。
    old_vel = sdk.set_max_vel(AB_MAX_VEL_MPS)
    try:
        sdk.fly_route(ROUTE_TO_G[:2], agl_m=CRUISE_AGL_M, names=NAMES_TO_G[:2])
    finally:
        # 读不到原值时 set_max_vel() 返回 0.0，那种情况别拿 0 去设（会抛
        # ValueError 把真正的异常盖掉），留给下次启动按 launch 默认值恢复。
        if old_vel > 0:
            sdk.set_max_vel(old_vel)
    sdk.fly_route(ROUTE_TO_G[2:], agl_m=CRUISE_AGL_M, names=NAMES_TO_G[2:])

    # leg_start=ROUTE_G：这一段的机头航向严格取航线上 G->E 的方向角，而不是"飞机
    # 停在 G 的实际位置 -> E"——到点误差会让后者偏几度。
    fire = sdk.search_along(ROUTE_E, GROUND_FIRE, camera='down',
                            agl_m=CRUISE_AGL_M, what='地面火情', leg_start=ROUTE_G)
    if fire is None:
        raise RuntimeError('没能定位地面火情，任务2 中止')
    recon_ground(sdk, EV, fire)
    formation_home(sdk, ROUTE_A)
    sdk.hold_at(ROUTE_A[0], ROUTE_A[1], agl_m=CRUISE_AGL_M, seconds=15.0)
    sdk.announce('侦察机任务完成')


def supply(sdk: DroneSDK):
    """任务机：等通报 -> 取灭火弹 -> 投放 -> 拍照回传 -> 编队返航 -> 降落。"""
    sdk.PHOTO_DIR = PHOTO_DIR
    sdk.open_inbox(EV.ground)
    d = sdk.wait_event(EV.ground, 300.0)
    supply_ground(sdk, d, EV)
    sdk.follow_formation(agl_m=CRUISE_AGL_M, join='nearest')
    # 2026-10-14 跟 highrise/mission 对齐（那两个 10-10 就改了，这里是漏网的最后一处）：
    # return_home 不给 direct=True。两条理由——
    #   ① 整段直飞不经规划器，中途出列时要横穿场地、擦着立柱过；
    #   ② direct 走的 coordinate_goto **没有刹车段**：被 _direct_leg 提速到 1.0 m/s 后，
    #      只要离目标还有 0.2 m 以上，速度前馈就恒等于 1.0 m/s（与剩余距离无关），而位置
    #      设定点只领先 0.1 m——等于定常速度追踪，在目标点外侧绕成半径 v·τ≈0.5 m 的极限环，
    #      套不进 arrival_threshold_m=0.3 的到达球。2026-10-14 真机 mission 的物资点就这么
    #      绕了 24 s（见 rounds.supply_return_gear 的说明）。
    # 默认 False 是「远距离走 goto(规划器+避障)、最后一段才直线精修落点」，落点精度不丢，
    # 而且那最后一段很短、限速仍是默认 0.3 m/s（r≈0.15 m 落在到达球内），不会绕圈。
    sdk.return_home(agl_m=CRUISE_AGL_M, sound='任务机已降落')


if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply, description=__doc__)
