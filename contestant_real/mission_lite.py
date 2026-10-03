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
from contest_sdk.exceptions import ContestSdkError


# 场地参数全在 venue.py——仿真与真机只有那一个文件不同，本程序两边逐字节相同。
from venue import (ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_D,
                   POINT_M, POINT_N, SUPPLY_XY, SUPPLY_TAG, GROUND_FIRE, HIGH_FIRE,
                   FACADE_YAW_DEG, STATIONS, CRUISE_AGL_M, OBSERVE_AGL_M,
                   FORWARD_BEFORE_FIRE_M, SHOTS)

FORMATION_ROUTE = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_G, ROUTE_D]
ROUTE_TO_G = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G]
PHOTO_DIR = '/logs/综合任务照片'
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
def _use_camera(sdk: DroneSDK, which: str) -> None:
    """配相机：`'front'` / `'down'` 只开那一路并停掉另一路，`'none'` 两路全停。

    为什么值得这么做：一个 yolo_detector_node 实测占机载约 12% 整机 CPU，
    单路比双路省约 9%（真机实测 86% vs 95%）。而本任务的两路从来不重叠：
    地面火情用下视、高层火情瞄准用前视、取放物资用下视（精降看 AprilTag）。

    切换代价实测 **4 秒**（停掉后该路数据立刻为 0，重起 4 秒出数据）。所以
    调用点都选在**后面紧跟一段飞行**的位置，4 秒被转场时间盖住、等于不花时间。

    `'none'` 用在整段不碰相机的阶段（第 1 轮编队飞行只有 lead_formation 和
    hold_at），两路全停省得更多。

    仿真下 `set_camera_mode()` 是空操作（没有 control_server，检测节点随飞行栈
    一起起），所以这个函数在仿真和真机都能原样跑——本程序两边逐字节相同。

    Args:
        which: `'front'` 前视 / `'down'` 下视 / `'none'` 两路全停。
    """
    try:
        if which == 'none':
            # 编队飞行那种整段不用相机的，两路全停。
            sdk.set_camera_mode('front', 'stop')
            sdk.set_camera_mode('down', 'stop')
            return
        other = 'down' if which == 'front' else 'front'
        sdk.set_camera_mode(other, 'stop')      # 先停，先把 CPU 让出来
        sdk.set_camera_mode(which, 'yolo')      # 再起，wait=True 等到真出数据
    except ContestSdkError as exc:
        # **降级而不是中止。** 省 CPU 不该有权力打断一次飞行：切换走 WiFi
        # 打到飞机，那条链路会间歇性断（SDK 内部已经重试 3 次，撞上长断链
        # 还是会抛）。这里退回"两路全开"——多占约 13% 机载 CPU，但任务能
        # 接着飞，后面要用的那一路一定在。
        sdk.progress(f'[警告] 配相机失败（{exc!r}），降级为两路全开继续飞')
        for cam in ('front', 'down'):
            try:
                sdk.set_camera_mode(cam, 'yolo', wait=False)
            except ContestSdkError:
                pass   # 连降级都失败就只能这样了，让后面的 wait_for_detection 去报
        sdk.progress('[警告] 已降级：两路相机都开着，本轮不再切换')


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
    # 前视由调用方（recon 的轮次循环）在进来之前就配好了——它那儿才知道
    # "本轮是不是高层"，在这里再切一次是重复的。

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
    # 这一轮全程不用相机（只有 lead_formation + hold_at），两路全停。
    _use_camera(sdk, 'none')
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
        # 两轮不重复：地面火情上轮处置过的话，本轮必然是高层，不用再搜一遍。
        # 这个判断在**起飞前**就成立，所以相机也在这儿配好——A->B->C->G 整段
        # 飞行把切换的 4 秒吸收掉，比到 G 之后再切更早、更不占时间。
        #   还可能是地面火情 -> 开下视（到 E 点那段要边飞边找）
        #   已知是高层      -> 直接开前视，整轮用不到下视
        may_be_ground = KIND_GROUND not in done
        _use_camera(sdk, 'down' if may_be_ground else 'front')
        # 裁判盯着"侦察机过 G 点"才放火情，到 G 之前场上什么都没有。
        sdk.fly_route(ROUTE_TO_G, agl_m=CRUISE_AGL_M, names=['A', 'B', 'C', 'G'])
        if not may_be_ground:
            fire_xy = None
        else:
            fire_xy = sdk.search_along(ROUTE_E, GROUND_FIRE, camera='down',
                                       agl_m=CRUISE_AGL_M, what='地面火情')
        kind = KIND_GROUND if fire_xy else KIND_HIGH
        if kind == KIND_HIGH and may_be_ground:
            # 过了 E 点整段都没看到地面火情 -> 本轮是高层。这时才把下视换成
            # 前视（上面"已知是高层"那条路已经开着前视了，不用重复切）。
            _use_camera(sdk, 'front')
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
    # 本轮全程下视（fetch_from 精降看标识、aim_at/snapshot 都是下视）。
    # 放在 takeoff 之前：等事件时本来就是停着的，这 4 秒完全不占飞行时间。
    _use_camera(sdk, 'down')
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
    # 先下视：fetch_from 取器材是精降，看物资点的 AprilTag。
    _use_camera(sdk, 'down')
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火器材', agl_m=CRUISE_AGL_M,
                   sound='任务机抓取灭火弹')
    sdk.goto_world(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M, what='E点待命位')
    sdk.send_to_teammate(EV_AT_E)
    sdk.announce('任务机高层灭火已就位')
    sdk.wait_event(EV_BREACHED, 300.0)
    sdk.wait_event(EV_SPOT_CLEAR, 150.0)          # 发射点上这会儿还杵着侦察机
    # 改前视：下面 aim_at 用前视瞄高层火情。切换放在飞往发射点**之前**，
    # 这段飞行把 4 秒盖住；放到 aim_at 前面就要停在那儿干等。
    _use_camera(sdk, 'front')
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
    # 这一轮全程不用相机（follow_formation + return_home），两路全停。
    _use_camera(sdk, 'none')
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
