#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合模拟验证：编队飞行 + 地面火情处置 + 高层火情处置，三轮连贯跑完。

用户 2026-09-30 定的流程：

  第 1 轮  编队飞行（A B C G E F G D，规则同 formation.py）
           编队解散后：侦察机回 A 点悬停，任务机回自己起降点降落停稳。

  第 2 轮  侦察机从 A 点出发巡检，**自己判断有没有地面火情**：
             有 -> 走地面火情处置（流程同 groundfire.py）
             没有 -> 走高层火情处置（流程同 highrise.py）
           处置完：侦察机回 A 点悬停，任务机回自己起降点降落停稳。

  第 3 轮  必然是另一种火情（两轮不重复），流程同上。
           全部结束后两机各自回起飞点降落。

火情什么时候出现、这一轮出哪一种，由 scripts/referee.py（裁判）控制：开局把两处
火情标识都从 world 里删掉，等侦察机过 G 点再把本轮抽中的那个生成回来。所以侦察机
在第 2 轮飞到 G 之前，场上是没有任何火情的——"判断有没有地面火情"这件事才有意义。

—— 跟三个单任务程序的关系 ——
formation.py / groundfire.py / highrise.py 是**考核项，本文件一个字都不改它们**。
可以直接当函数用的（真正的可复用件）就 import：
    formation.leader_route / follow_formation / _land_at_pad / _hold_at_point_a / _Inbox
    utils.descend_onto / fire_launcher
    highrise._fly_route / _inspect_here / center_fire_in_view / _step_in_to_launch /
             _capture_photo / _supply_point_action / _start_spot_clear_watch
只内联在单任务流程里、没法单独调用的那几段（地面火情的"边飞边找+对准解算"、
物资点取弹、火情点投放），**照抄过来**并在原位注明出处。

—— 为什么不直接调 groundfire.recon() / highrise.recon() ——
那两个是整条流程的编排：从 takeoff() 开始、从 A 点把航线重飞一遍、最后接编队返航。
综合任务里侦察机每轮开始时**已经在 A 点悬停**，而且三轮之间的衔接、分支通报都要
本文件自己掌握，套用整条流程会多飞一趟航线，也没法在中途插入分支判定。
"""
import argparse
import math
import threading
import time

import formation
import highrise
import utils
from contest_sdk import DroneSDK

CRUISE_AGL_M = formation.CRUISE_AGL_M
SPACING_M = 4.0

ROUTE_A = (3.0, 3.0)
ROUTE_B = (3.0, 22.0)
ROUTE_C = (17.0, 22.0)
ROUTE_G = (17.0, 16.0)
ROUTE_E = (10.0, 16.0)
ROUTE_F = (14.0, 14.0)
ROUTE_D = (17.0, 3.0)
FORMATION_ROUTE = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_G, ROUTE_D]

SUPPLY_XY = (10.0, 8.0)               # 物资点，AprilTag ID0
GROUND_FIRE = 'apriltag:2'            # 地面火情标识
SUPPLY_TAG = 'apriltag:0'

GRAB_PWM = 800                        # 抓紧（真机实测值，见 project_nx02_servos 记录）
DROP_PWM = 2000                       # 松开
SERVO_TRAVEL_S = 2.0                  # 舵机没有位置反馈，只能等
HOVER_AFTER_TAKEOFF_S = 2.0
DETECT_POLL_S = 0.5

# 高层火情只可能贴在楼的 -Y 面（layout 的 fire_apriltag_random.face），所以
# "正对楼面"就是机头朝正北。侦察机在 M/N 观察位上用的也是这个角度。
FACADE_YAW_DEG = 90.0

KIND_GROUND = 'ground'
KIND_HIGH = 'high'

# ---- 跨机事件。全部用本文件自己的名字 ----
# 不能复用单任务程序里的事件名：可靠事件通道是"一个事件名只能挂一个处理函数"
# （reliability.py 里 `self._handlers[name] = callback`，重复注册直接覆盖），
# 两轮之间如果重名，后注册的会把前一轮的信箱顶掉。
EV_ROUND_KIND = 'mission_round_kind'    # NX01 -> NX02：本轮是哪种火情
EV_GROUND_FIRE = 'mission_ground_fire'  # NX01 -> NX02：地面火情世界坐标
EV_DROPPED = 'mission_dropped'          # NX02 -> NX01：灭火弹已投放
EV_HIGH_FIRE = 'mission_high_fire'      # NX01 -> NX02：高层火情 + 侦察机发射点
EV_AT_E = 'mission_at_standby'          # NX02 -> NX01：我到 E 点待命了
EV_BREACHED = 'mission_breached'        # NX01 -> NX02：破窗完成
EV_SPOT_CLEAR = 'mission_spot_clear'    # NX01 -> NX02：发射点已腾出
EV_EXTINGUISHED = 'mission_extinguished'  # NX02 -> NX01：灭火弹发射完毕
EV_ROUND_DONE = 'mission_round_done'    # NX02 -> NX01：本轮我已降落停稳

ROUND_WAIT_S = 900.0
FIRE_WAIT_S = 420.0
AT_E_WAIT_S = 300.0
BREACH_WAIT_S = 300.0
SPOT_CLEAR_WAIT_S = 150.0
EXTINGUISH_WAIT_S = 420.0
DROP_WAIT_S = 420.0


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _goto_world(sdk, wx, wy, what, z_agl=CRUISE_AGL_M):
    """飞到某个世界坐标上方，全程锁高（走规划器，有避障）。"""
    lx, ly, lz = sdk.world_to_local(wx, wy, z_agl)
    print(f'[{sdk.namespace}] 飞往{what} ({wx:.2f}, {wy:.2f})', flush=True)
    with sdk.fixed_altitude(lz):
        sdk.goto(lx, ly, lz)


def _drive_servos(sdk, pwm, label):
    """抓取/投放机构。仿真里飞控不一定配了舵机输出，动不了就打印、继续飞完流程。"""
    try:
        sdk.set_servos({s: pwm for s in sorted(sdk.servos)})
    except Exception as exc:
        print(f'[{sdk.namespace}] {label}：舵机没动（{exc}）'
              f'——真机需要飞控配好 MAIN7/MAIN9', flush=True)
        return
    time.sleep(SERVO_TRAVEL_S)
    print(f'[{sdk.namespace}] {label}完成', flush=True)


def _fly_waypoints(sdk, names_xy, z_agl=CRUISE_AGL_M):
    """按航点序列飞，每个航点先把机头转到下一段方向、停住再走。

    直接用 highrise._fly_route——它就是干这个的，规则（停 2 秒、航段内航向恒定）
    跟编队段一致，没必要再写一份。
    """
    highrise._fly_route(sdk, names_xy, z_agl=z_agl)


# ---------------------------------------------------------------------------
# 地面火情：这三段在 groundfire.py 里是内联在 recon()/supply() 中的，没法单独
# 调用，所以照抄过来。原文件是考核项，一个字没动。
# ---------------------------------------------------------------------------
def search_ground_fire(sdk):
    """从 G 点沿 G->E 边飞边用下视找地面火情，找到就停车对准并解算坐标。

    返回火情**世界坐标** (x, y)；整条 G->E 飞完都没看到、或对准/解算失败返回 None。

    抄自 groundfire.recon() 的"G -> E 边飞边找"+"对准"两段（那里找不到是 raise，
    这里改成 return None——综合任务要靠"没找到"切到高层火情分支，不是中止）。
    """
    # goto 是阻塞的，所以先把目标发出去，在后台线程里飞，主线程盯检测；
    # 一发现就 cancel_goto 停在当场，避免飞过头。
    ex, ey, ez = sdk.world_to_local(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M)
    leg_done = threading.Event()

    def _fly_leg():
        try:
            with sdk.fixed_altitude(ez):
                sdk.goto(ex, ey, ez)
        except Exception as exc:
            print(f'[{sdk.namespace}] G->E 航段结束（{exc}）', flush=True)
        finally:
            leg_done.set()

    threading.Thread(target=_fly_leg, daemon=True).start()

    seen = False
    t0 = time.time()
    while time.time() - t0 < FIRE_WAIT_S:
        try:
            sdk.wait_for_detection(GROUND_FIRE, timeout=DETECT_POLL_S, camera='down')
        except Exception:
            if leg_done.is_set():
                break            # 整条 G->E 飞完都没看到
            continue
        print(f'[{sdk.namespace}] 发现地面火情，停车对准', flush=True)
        sdk.cancel_goto()
        time.sleep(1.0)          # 等刹停，别带着速度去对准
        seen = True
        break
    if not seen:
        return None

    # center_on_target 让 precision_servo_node 把目标居中（只居中不下降），
    # 再用 locate_target 拿目标在地面的实际坐标。
    try:
        sdk.center_on_target(GROUND_FIRE, timeout=40.0)
        fire_local = sdk.locate_target(GROUND_FIRE, timeout=5.0)
    except Exception as exc:
        print(f'[{sdk.namespace}] 对准/解算失败（{exc}）', flush=True)
        return None
    fx, fy = sdk.local_to_world(fire_local.x, fire_local.y, 0.0)[:2]
    print(f'[{sdk.namespace}] 地面火情世界坐标 ({fx:.2f}, {fy:.2f})', flush=True)
    return float(fx), float(fy)


def pick_up_extinguisher(sdk):
    """物资点：边瞄准边降落到底，抓取，再起飞。抄自 groundfire.supply()。"""
    _goto_world(sdk, SUPPLY_XY[0], SUPPLY_XY[1], '物资点')
    utils.descend_onto(sdk, SUPPLY_TAG, '灭火弹')
    print(f'[{sdk.namespace}] 已降落在物资点，开始抓取', flush=True)
    sdk.play_sound_light('任务机抓取灭火弹')
    _drive_servos(sdk, GRAB_PWM, '抓取')
    sdk.takeoff(height_m=CRUISE_AGL_M)
    time.sleep(HOVER_AFTER_TAKEOFF_S)


def drop_on_fire(sdk, fx, fy):
    """飞到通报的火情坐标上方，对准，投放。抄自 groundfire.supply()。"""
    _goto_world(sdk, fx, fy, '地面火情点')
    try:
        sdk.center_on_target(GROUND_FIRE, timeout=40.0)
        print(f'[{sdk.namespace}] 已对准地面火情', flush=True)
    except Exception as exc:
        print(f'[{sdk.namespace}] 没对上火情标识（{exc}），按通报坐标投放', flush=True)
    sdk.play_sound_light('任务机投放灭火弹')
    _drive_servos(sdk, DROP_PWM, '投放')


# ---------------------------------------------------------------------------
# 一轮"火情处置"的收尾：从 G 点起编队返航，侦察机回 A 待命、任务机回起降点降落。
# 两种火情轮共用（跟单任务版一致：用户要求"任务转换期间处理方式和单任务一样"）。
# ---------------------------------------------------------------------------
def _formation_home_recon(sdk, last=False):
    """侦察机侧：从 G 起编队，任务机过 D 点就解散。

    last=False（还有下一轮）：编队终点是 A 点，解散后在 A 点悬停待命。
    last=True （最后一轮）：**不去 A 点了**，编队终点直接设成自己的起降点，
        解散后就地降落（用户 2026-10-01：最后一个任务完成后侦察机不用再去 A 点）。
    """
    final = formation._own_pad(sdk) if last else ROUTE_A
    formation.leader_route(sdk, [ROUTE_G, ROUTE_D], spacing_m=SPACING_M,
                           start_xy=ROUTE_G, final_xy=final,
                           disband_after_follower_passes=ROUTE_D)
    if last:
        formation._land_at_pad(sdk)
    else:
        formation._hold_at_point_a(sdk, ROUTE_A)


def _formation_home_supply(sdk, boxes, release=False):
    """任务机侧：就地入列 -> 跟队 -> 解散 -> （需要的话先还器材）-> 回起降点降落。

    goto_station=False：此刻任务机就在长机附近，再飞一趟"航线起点后方 spacing 米"
    的站位点纯属绕路。

    release=True：解散后先飞物资点降落、松开机械抓把灭火器材放回去，再回自己
    起降点（用户 2026-10-01）。只有**高层火情轮**需要——那一轮器材是抓在手上
    带去发射的，打完还在机上；地面火情轮的灭火弹已经投到火点上了，没东西可还。
    """
    formation.follow_formation(sdk, SPACING_M, inbox=boxes['route_done'],
                               plan=boxes['route_plan'], goto_station=False)
    if release:
        highrise._supply_point_action(sdk, highrise.RELEASE_PWM, '释放灭火器材')
    formation._land_at_pad(sdk)


# ---------------------------------------------------------------------------
# 地面火情轮
# ---------------------------------------------------------------------------
def ground_round_recon(sdk, fire_xy, boxes, last=False):
    """侦察机：通报火情 -> 回 G 点等任务机投弹 -> 编队返航。"""
    fx, fy = fire_xy
    sdk.play_sound_light('侦察机发现地面火情')
    # 先播报再发事件：任务机一收到就起飞并播"任务机起飞"，两条播报会抢在一起。
    sdk.play_sound_light('侦察机通报地面火情')
    sdk.send_to_teammate(EV_GROUND_FIRE, x=float(fx), y=float(fy))
    print(f'[{sdk.namespace}] 已通报任务机', flush=True)

    _goto_world(sdk, ROUTE_G[0], ROUTE_G[1], '航点G（等待点）')
    print(f'[{sdk.namespace}] 已在 G 点悬停，等任务机投放灭火弹', flush=True)
    _wait(boxes, 'dropped', DROP_WAIT_S)
    print(f'[{sdk.namespace}] 任务机已投放灭火弹，就地开始编队返航', flush=True)
    _formation_home_recon(sdk, last=last)


def ground_round_supply(sdk, boxes):
    """任务机：起飞 -> 取灭火弹 -> 飞火情点投放 -> 编队返航 -> 降落。"""
    d = _wait(boxes, 'ground_fire', FIRE_WAIT_S)
    fx, fy = float(d['x']), float(d['y'])
    print(f'[{sdk.namespace}] 收到火情坐标 ({fx:.2f}, {fy:.2f})', flush=True)

    sdk.takeoff(height_m=CRUISE_AGL_M)
    time.sleep(HOVER_AFTER_TAKEOFF_S)
    pick_up_extinguisher(sdk)
    drop_on_fire(sdk, fx, fy)
    sdk.send_to_teammate(EV_DROPPED)
    print(f'[{sdk.namespace}] 已通知侦察机', flush=True)
    _formation_home_supply(sdk, boxes)          # 灭火弹已投出，没东西可还


# ---------------------------------------------------------------------------
# 高层火情轮。编排照着 highrise.recon()/supply() 的三段式来，但积木直接用
# highrise 里那些小函数，不复制。
# ---------------------------------------------------------------------------
def high_round_recon(sdk, boxes, last=False):
    """侦察机：三栋楼巡检拍照 -> 发现火情就协同灭火 -> 巡检完回 G 编队返航。"""
    at_station = None
    fired = False
    pending_spot_clear = None

    for idx, (bldg, pt, pt_name, yaw_deg, detect) in enumerate(highrise.INSPECT_STATIONS):
        if at_station != pt_name:
            _fly_waypoints(sdk, [(pt_name, pt)], z_agl=highrise.OBSERVE_AGL_M)
            at_station = pt_name
            if pending_spot_clear is not None:
                pending_spot_clear(f'下一个巡检点 {pt_name}')
                pending_spot_clear = None
        print(f'[{sdk.namespace}] 在 {pt_name} 点转到 {yaw_deg:.0f}° 巡检 {bldg} 楼', flush=True)
        sdk.face_yaw(math.radians(yaw_deg), timeout=formation.TURN_TIMEOUT_S,
                     tolerance_deg=formation.TURN_TOL_DEG)

        if not detect or fired:
            highrise._capture_photo(sdk, f'{bldg}楼')
            if detect:
                print(f'[{sdk.namespace}] 火情已处置，{bldg} 楼只拍照不再查', flush=True)
            continue

        sdk.play_sound_light('侦察机排查高层火情')
        det = highrise._inspect_here(sdk)
        if det is None:
            highrise._back_to_observe_alt(sdk)
            highrise._capture_photo(sdk, f'{bldg}楼')
            print(f'[{sdk.namespace}] {bldg} 楼没有火情，继续巡检', flush=True)
            continue

        # ---- 协同灭火 ----
        sdk.play_sound_light('侦察机发现高楼火情')
        highrise.center_fire_in_view(sdk, '高层火情')
        highrise._capture_photo(sdk, f'{bldg}楼')          # 对准之后再拍，火情居中
        fire_pos = highrise._step_in_to_launch(sdk)
        at_station = None
        print(f'[{sdk.namespace}] {bldg} 楼有火情，发射点 '
              f'({fire_pos[0]:.2f}, {fire_pos[1]:.2f})，通报任务机并原地等待', flush=True)
        sdk.play_sound_light('侦察机通报高层火情')
        sdk.send_to_teammate(EV_HIGH_FIRE, x=fire_pos[0], y=fire_pos[1],
                             z=fire_pos[2], at=bldg)

        _wait(boxes, 'at_e', AT_E_WAIT_S)
        print(f'[{sdk.namespace}] 任务机已到 E 点，发射破窗弹', flush=True)
        sdk.play_sound_light('侦察机发射破窗弹')
        utils.fire_launcher(sdk, '发射破窗弹')
        sdk.send_to_teammate(EV_BREACHED)
        sdk.play_sound_light('侦察机破窗完成')
        # 发射点马上要让给任务机。两条放行路径：离得够远（看门狗），或者已经
        # 飞到下一个落脚点（下面显式调）。highrise 那个看门狗直接拿来用，只是
        # 它发的事件名是 highrise 自己的，这里得用本文件的名字，所以自己发。
        pending_spot_clear = _make_spot_clear_notifier(sdk, fire_pos[:2])
        fired = True

    print(f'[{sdk.namespace}] 三栋楼巡检拍照完毕', flush=True)
    _fly_waypoints(sdk, [('G', ROUTE_G)])
    if pending_spot_clear is not None:
        pending_spot_clear('G 点')
    if fired:
        print(f'[{sdk.namespace}] 在 G 点等任务机灭火完成…', flush=True)
        _wait(boxes, 'extinguished', EXTINGUISH_WAIT_S)
    _formation_home_recon(sdk, last=last)


def _make_spot_clear_notifier(sdk, spot_world):
    """破窗后盯着侦察机什么时候离开发射点，离开了就通知任务机进场。

    逻辑跟 highrise._start_spot_clear_watch 一样（离得够远 or 已到下一个落脚点），
    只是事件名换成本文件的 EV_SPOT_CLEAR。
    """
    sx, sy = float(spot_world[0]), float(spot_world[1])
    state = {'sent': False}

    def _send(why):
        if state['sent']:
            return
        state['sent'] = True
        try:
            sdk.send_to_teammate(EV_SPOT_CLEAR)
            print(f'[{sdk.namespace}] {why}，通知任务机进场', flush=True)
        except Exception as exc:
            print(f'[{sdk.namespace}] "已让开"没送到任务机（{exc}）', flush=True)

    def _loop():
        t0 = time.time()
        while time.time() - t0 < SPOT_CLEAR_WAIT_S and not state['sent']:
            try:
                cx, cy, _ = sdk.get_local_position()
                wx, wy = sdk.local_to_world(cx, cy, 0.0)[:2]
            except Exception:
                time.sleep(0.2)
                continue
            d = math.hypot(wx - sx, wy - sy)
            if d >= highrise.SPOT_CLEAR_M:
                _send(f'已离开发射点（{d:.1f} m）')
                return
            time.sleep(0.2)
        if not state['sent']:
            _send(f'等了 {SPOT_CLEAR_WAIT_S:.0f} 秒仍没离开发射点（超时兜底）')

    threading.Thread(target=_loop, daemon=True).start()
    return lambda where: _send(f'已飞到{where}，发射点已腾出')


def high_round_supply(sdk, boxes):
    """任务机：起飞 -> 取器材 -> E 点待命 -> 破窗后进场发射灭火弹 -> 编队返航。"""
    d = _wait(boxes, 'high_fire', FIRE_WAIT_S)
    fx, fy, fz = float(d['x']), float(d['y']), float(d.get('z', CRUISE_AGL_M))
    print(f'[{sdk.namespace}] 收到火情：{d.get("at")} 楼，侦察机发射点 '
          f'({fx:.2f}, {fy:.2f})', flush=True)

    sdk.takeoff(height_m=CRUISE_AGL_M)
    time.sleep(HOVER_AFTER_TAKEOFF_S)
    highrise._supply_point_action(sdk, highrise.GRAB_PWM, '抓取灭火器材',
                                  sound='任务机抓取灭火弹')

    _goto_world(sdk, ROUTE_E[0], ROUTE_E[1], 'E点待命位')
    sdk.send_to_teammate(EV_AT_E)
    sdk.play_sound_light('任务机高层灭火已就位')
    print(f'[{sdk.namespace}] 已在 E 点待命，等侦察机破窗', flush=True)
    _wait(boxes, 'breached', BREACH_WAIT_S)
    print(f'[{sdk.namespace}] 已破窗，等侦察机让开发射点…', flush=True)
    _wait(boxes, 'spot_clear', SPOT_CLEAR_WAIT_S + 30.0)

    lx, ly, _lz = sdk.world_to_local(fx, fy, CRUISE_AGL_M)
    print(f'[{sdk.namespace}] 飞往侦察机位置 ({fx:.2f}, {fy:.2f})', flush=True)
    try:
        with sdk.fixed_altitude(fz):
            sdk.goto(lx, ly, fz)
    except Exception as exc:
        print(f'[{sdk.namespace}] 没能精确到点（{exc}），就地发射', flush=True)
    sdk.play_sound_light('任务机到达瞄准点')
    # **先把机头对正楼面再居中**（用户 2026-10-01 指出任务机朝向不对）。
    # center_fire_in_view 是"锁住当前朝向、只做横向平移"的（它的 docstring 写着
    # "调用方进来之前已经 face_yaw 到位"）——侦察机在观察位上确实先转过了，
    # 而任务机是 goto 飞到发射点后直接调它，朝向是规划器留下的航向，根本没对正
    # 楼面。于是画面里居中了、机身却斜着，弹丸打出去是斜的。
    # 火情只可能贴在楼的 -Y 面（见 layout 的 fire_apriltag_random.face），
    # 所以正对楼面就是机头朝正北 +90°。
    sdk.face_yaw(math.radians(FACADE_YAW_DEG), timeout=formation.TURN_TIMEOUT_S,
                 tolerance_deg=formation.TURN_TOL_DEG)
    highrise.center_fire_in_view(sdk, '高层火情')

    sdk.play_sound_light('任务机发射灭火弹')
    for i in range(1, highrise.EXTINGUISHER_SHOTS + 1):
        utils.fire_launcher(sdk, f'发射灭火弹 {i}/{highrise.EXTINGUISHER_SHOTS}')
        if i < highrise.EXTINGUISHER_SHOTS:
            time.sleep(highrise.SHOT_INTERVAL_S)
    sdk.send_to_teammate(EV_EXTINGUISHED)
    print(f'[{sdk.namespace}] 灭火弹发射完毕，已通知侦察机', flush=True)

    # 灭火完毕把高度拉回编队巡航高度：对准火情时飞机被挪到了着火点的高度
    # （1.5 或 2.5 m），带着它入列会让队形在竖直方向上错开。
    cx, cy, _ = sdk.get_local_position()
    _, _, cruise_z = sdk.world_to_local(0.0, 0.0, CRUISE_AGL_M)
    print(f'[{sdk.namespace}] 回到编队巡航高度 {CRUISE_AGL_M:.1f} m', flush=True)
    sdk.goto_direct(cx, cy, cruise_z)
    _formation_home_supply(sdk, boxes, release=True)   # 器材还在机上，返航后要还回物资点


# ---------------------------------------------------------------------------
# 三轮编排
# ---------------------------------------------------------------------------
def _open_boxes(sdk, role):
    """**一开始就把所有事件信箱注册好**。

    可靠事件通道是"先回 ACK 再查处理函数"，没注册的事件会被确认后丢弃；而且
    一个事件名只能挂一个处理函数（重复注册直接覆盖）。所以不能等用到了再注册，
    也不能两轮各注册一次——开局一次性建好，三轮共用。
    """
    if role == 'recon':
        names = {'dropped': EV_DROPPED, 'at_e': EV_AT_E,
                 'extinguished': EV_EXTINGUISHED, 'round_done': EV_ROUND_DONE}
    else:
        names = {'round_kind': EV_ROUND_KIND, 'ground_fire': EV_GROUND_FIRE,
                 'high_fire': EV_HIGH_FIRE, 'breached': EV_BREACHED,
                 'spot_clear': EV_SPOT_CLEAR}
    boxes = {k: formation._Inbox(sdk, v) for k, v in names.items()}
    # 记下每个信箱装的是哪个事件：formation._Inbox 一个信箱可以装多个事件，
    # 所以 wait() 的签名是 wait(event, timeout_s)——**两个参数**。
    # （highrise._Inbox 是一箱一事件、wait(timeout) 只要一个参数，两套习惯很容易
    #  串。2026-09-30 首跑就栽在这儿：TypeError 缺参数，第一轮编队跑完当场崩。）
    # 统一走 _wait(boxes, key, timeout)，调用处不用记事件名，也传不错。
    boxes['_event'] = dict(names)
    if role == 'supply':
        # 编队段自己的两个事件，follow_formation 要用
        boxes['route_done'] = formation._Inbox(sdk, formation.ROUTE_DONE)
        boxes['route_plan'] = formation._Inbox(sdk, formation.ROUTE_PLAN)
        boxes['_event']['route_done'] = formation.ROUTE_DONE
        boxes['_event']['route_plan'] = formation.ROUTE_PLAN
        # 三轮共用同一批信箱，每轮开头要复位，否则上一轮的旧事件会当成本轮的
        boxes['_resettable'] = ['round_kind', 'ground_fire', 'high_fire',
                                'breached', 'spot_clear', 'route_done', 'route_plan']
    else:
        boxes['_resettable'] = ['dropped', 'at_e', 'extinguished', 'round_done']
    return boxes


def _wait(boxes, key, timeout_s):
    """等某个信箱里的事件，返回随事件带来的数据（dict）。

    两处坑都在这儿兜住：
      · formation._Inbox.wait() 的签名是 wait(event, timeout_s)——**两个参数**
        （一个信箱能装多个事件）；highrise._Inbox.wait(timeout) 只要一个。
      · formation._Inbox.wait() **不返回数据**，数据在 self.data 里；
        highrise._Inbox.wait() 是返回的。直接拿返回值会得到 None，
        `d.get(...)` 当场 AttributeError（2026-09-30 首跑实测）。
    """
    boxes[key].wait(boxes['_event'][key], timeout_s)
    return boxes[key].data or {}


def _reset_boxes(boxes):
    for k in boxes['_resettable']:
        b = boxes.get(k)
        if b is not None:
            b._seen.clear()
            b.data = {}


def recon(sdk):
    """侦察机：编队 -> 判断并处置第一种火情 -> 处置另一种 -> 回起飞点降落。"""
    boxes = _open_boxes(sdk, 'recon')
    standby = formation.listen_standby(sdk)   # READY / IN_POSITION，编队段要用

    sdk.takeoff(height_m=CRUISE_AGL_M)

    # ================= 第 1 轮：编队飞行 =================
    print(f'[{sdk.namespace}] ===== 第 1 轮：编队飞行 =====', flush=True)
    formation.leader_route(sdk, FORMATION_ROUTE, standby, spacing_m=SPACING_M)
    formation._hold_at_point_a(sdk, ROUTE_A)

    kinds_done = []
    for rnd in (2, 3):
        print(f'[{sdk.namespace}] 等任务机降落停稳…', flush=True)
        _wait(boxes, 'round_done', ROUND_WAIT_S)
        _reset_boxes(boxes)
        print(f'[{sdk.namespace}] ===== 第 {rnd} 轮：开始巡检 =====', flush=True)

        # 航点飞到 G。裁判就是盯着"侦察机过 G 点"才把本轮的火情放出来的，
        # 所以到 G 之前场上什么都没有，先飞过去。
        _fly_waypoints(sdk, [('A', ROUTE_A), ('B', ROUTE_B),
                             ('C', ROUTE_C), ('G', ROUTE_G)])
        print(f'[{sdk.namespace}] 已到 G 点，开始用下视相机搜索地面火情', flush=True)

        if KIND_GROUND in kinds_done:
            # 地面火情上一轮已经处置过，两轮不重复 -> 本轮必然是高层火情，
            # 不用再沿 G->E 搜一遍地面。
            kind, fire_xy = KIND_HIGH, None
            print(f'[{sdk.namespace}] 地面火情上一轮已处置，本轮直接查高层', flush=True)
        else:
            fire_xy = search_ground_fire(sdk)
            kind = KIND_GROUND if fire_xy else KIND_HIGH
            if fire_xy is None:
                print(f'[{sdk.namespace}] 没有地面火情，本轮按高层火情处置', flush=True)

        sdk.send_to_teammate(EV_ROUND_KIND, kind=kind, round=rnd)
        print(f'[{sdk.namespace}] 本轮判定：{"地面火情" if kind == KIND_GROUND else "高层火情"}',
              flush=True)

        last = (rnd == 3)          # 最后一轮：编队终点直接设成自己的起降点
        if kind == KIND_GROUND:
            ground_round_recon(sdk, fire_xy, boxes, last=last)
        else:
            high_round_recon(sdk, boxes, last=last)
        kinds_done.append(kind)

    # 第 3 轮的编队终点就是自己的起降点，上面 _formation_home_recon(last=True)
    # 里已经降落了，这里不用再降一次。
    print(f'[{sdk.namespace}] 已降落在自己起降点，等任务机完成收尾…', flush=True)
    _wait(boxes, 'round_done', ROUND_WAIT_S)
    print(f'[{sdk.namespace}] ===== 三轮全部完成 =====', flush=True)
    sdk.play_sound_light('侦察机任务完成')


def supply(sdk):
    """任务机：编队 -> 按侦察机通报的种类处置两轮火情 -> 每轮都回起降点降落。"""
    boxes = _open_boxes(sdk, 'supply')

    sdk.takeoff(height_m=CRUISE_AGL_M)

    # ================= 第 1 轮：编队飞行 =================
    print(f'[{sdk.namespace}] ===== 第 1 轮：编队飞行 =====', flush=True)
    formation.follow_formation(sdk, SPACING_M, inbox=boxes['route_done'],
                               plan=boxes['route_plan'])
    formation._land_at_pad(sdk)
    sdk.play_sound_light('任务机已降落')
    sdk.send_to_teammate(EV_ROUND_DONE, round=1)

    for rnd in (2, 3):
        _reset_boxes(boxes)
        print(f'[{sdk.namespace}] 等侦察机通报本轮火情种类…', flush=True)
        d = _wait(boxes, 'round_kind', ROUND_WAIT_S)
        kind = str(d.get('kind', KIND_GROUND))
        print(f'[{sdk.namespace}] ===== 第 {rnd} 轮：'
              f'{"地面火情" if kind == KIND_GROUND else "高层火情"} =====', flush=True)
        if kind == KIND_GROUND:
            ground_round_supply(sdk, boxes)
        else:
            high_round_supply(sdk, boxes)
        sdk.play_sound_light('任务机已降落')
        sdk.send_to_teammate(EV_ROUND_DONE, round=rnd)

    print(f'[{sdk.namespace}] ===== 三轮全部完成 =====', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--namespace', required=True)
    ap.add_argument('--role', required=True,
                    choices=('leader', 'follower', 'recon', 'supply'))
    ap.add_argument('--teammate', required=True)
    ap.add_argument('--spacing', type=float, default=SPACING_M)
    args = ap.parse_args()

    sdk_role = 'recon' if args.role in ('leader', 'recon') else 'supply'
    sdk = DroneSDK(namespace=args.namespace, role=sdk_role,
                   teammate_namespace=args.teammate)
    try:
        if sdk_role == 'recon':
            recon(sdk)
        else:
            supply(sdk)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()
