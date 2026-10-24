#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四个选手程序共用的飞行段。装在 SDK 里，`venue` 由选手目录提供（两套场地各一份）。"""
import os
import sys
from typing import Any, Dict, NamedTuple, Optional, Tuple

import contest_sdk
from contest_sdk import DroneSDK
from contest_sdk.exceptions import ContestSdkError

import venue
from venue import (ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_D, ROUTE_E, ROUTE_F, ROUTE_G,
                   SUPPLY_XY, SUPPLY_TAG, HIGH_FIRE, FACADE_YAW_DEG, STATIONS,
                   SUPPLY_XY_GROUND, SUPPLY_XY_HIGH, SUPPLY_TAG_HIGH,
                   CRUISE_AGL_M, OBSERVE_AGL_M, SHOTS, FORWARD_BEFORE_FIRE_M, YOLO_ENGINE,
                   CAMERA_MODE, SKIP_INFER, CONF_THRES_FRONT, CONF_THRES_DOWN,
                   RECON_AIM_TIMEOUT_S, SUPPLY_AIM_TIMEOUT_S, RECON_REAIM_TIMEOUT_S,
                   DROP_AGL_M, DROP_TOL_M, DROP_ALIGN_S)

# venue 必须来自选手目录（运行器把它挂成 /workspace 并排在 PYTHONPATH 最前），
# 装进 SDK 的那份只会是某一套场地，用错了整条航线都偏。
_SDK_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(contest_sdk.__file__)))
if os.path.abspath(venue.__file__).startswith(_SDK_ROOT + os.sep):
    raise RuntimeError(f'venue.py 必须来自选手目录，现在解析到了 {venue.__file__}')
print(f'[rounds] 公共飞行段 {__file__}；场地表 {venue.__file__}', file=sys.stderr)

#: 编队飞行整条航线（F 是虚拟点，E->F->G 是绕回 G 的那段）。
ROUTE_FULL = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_G, ROUTE_D]
#: 可传给 lead_formation(stop_at_waypoints=) 的关键航点下标（B/C/E/F，含 final_xy 后共 9 点）。
FORMATION_STOPS = (1, 2, 4, 5)

#: 侦察机每轮的出发段：裁判盯着"侦察机过 G 点"才放火情。
ROUTE_TO_G = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G]
NAMES_TO_G = ['A', 'B', 'C', 'G']

#: 全程规划器巡航限速的兜底值（米/秒），见 cruise_limit()。
DEFAULT_MAX_VEL_MPS = 0.8


class Events(NamedTuple):
    """一个程序自己的事件名表，按名字传给下面的公共段。"""
    ground: str = ''        #: 地面火情坐标（x, y）
    dropped: str = ''       #: 灭火弹已投放
    high: str = ''          #: 高层火情发射点（x, y, z, at）
    at_e: str = ''          #: 任务机已在 E 点待命
    breached: str = ''      #: 破窗完成
    spot_clear: str = ''    #: 发射点已让开
    done: str = ''          #: 高层火情已扑灭
    inspect_done: str = ''  #: 巡检结束、本轮没着火（只有 highrise 用）


# ---------------------------------------------------------------------------
# 通用动作
# ---------------------------------------------------------------------------
def cruise_limit(sdk: DroneSDK, default: float = DEFAULT_MAX_VEL_MPS) -> None:
    """设全程巡航限速：运行器的 `--max-vel` 优先，没给就用 `default`。"""
    # 这是 ego_planner 的节点级参数，管所有经规划器的航段，不管 goto_direct。
    sdk.set_max_vel(sdk.max_vel_mps or default)


#: 每一路相机的 YOLO 置信度阈值，取自 venue（选手在那儿调）。
_CONF_BY_CAMERA = {'front': CONF_THRES_FRONT, 'down': CONF_THRES_DOWN}


def use_camera(sdk: DroneSDK, which: str,
               conf_thres: Optional[float] = None) -> None:
    """配相机：`'front'` / `'down'` 只开那一路，`'none'` 只留前视 `raw` 守会话。

    Args:
        conf_thres: 临时覆盖这一路的 YOLO 置信度阈值。不给就按相机路取
            venue 里的 `CONF_THRES_FRONT` / `CONF_THRES_DOWN`——**常规调参
            改 venue.py 就行，不用动程序**，那个文件是挂载进容器的。
    """
    # 先算好：下面 except 分支也要用 conf，留在 try 里的话，'none' 那一路
    # 抛异常时会走到一个还没赋值的名字上（UnboundLocalError）。
    conf = _CONF_BY_CAMERA.get(which, 0.0) if conf_thres is None \
        else float(conf_thres)
    try:
        if which == 'none':
            # 不能两路全停：真机 cam1 长时间全空后 Argus 会话再也起不来，只能断电。
            sdk.set_camera_mode('front', 'raw')
            return
        # 模式恒为 yolo（真机 AprilTag 内嵌在那个节点里）；USE_YOLO 只切类别名，
        # 它为 False 时 SKIP_INFER=True，机载就只跑 tag、不加载 engine。
        sdk.set_camera_mode(which, CAMERA_MODE, engine_path=YOLO_ENGINE,
                            skip_infer=SKIP_INFER, conf_thres=conf)
    except ContestSdkError as exc:
        # 降级而不是中止：省 CPU 不该有权力打断一次飞行。
        sdk.progress(f'[警告] 配相机失败（{exc!r}），重试一次只开 {which} 那一路')
        try:
            sdk.set_camera_mode(which, CAMERA_MODE, wait=False,
                                engine_path=YOLO_ENGINE, skip_infer=SKIP_INFER,
                                conf_thres=conf)
        except ContestSdkError:
            sdk.progress('[警告] 相机配置彻底失败，本轮不再切换；后面要用相机的'
                         '步骤会在 wait_for_detection 处报超时')
        else:
            sdk.progress(f'[警告] 已降级：{which} 相机已请求开启但没等数据，'
                         f'本轮不再切换')


def formation_home(sdk: DroneSDK, final_xy: Tuple[float, float]) -> None:
    """侦察机带队从 G 点编队返航，过 D 点解散；`final_xy` 是长机解散后去哪儿。"""
    # disband_at=D 要显式给：默认判据是"长机回到自己起飞点上空"，会半路散队。
    sdk.lead_formation([ROUTE_G, ROUTE_D], agl_m=CRUISE_AGL_M, start_xy=ROUTE_G,
                       final_xy=final_xy, disband_at=ROUTE_D, tail_direct=True)


# ---------------------------------------------------------------------------
# 侦察机：一轮火情
# ---------------------------------------------------------------------------
def recon_ground(sdk: DroneSDK, ev: Events,
                 fire_xy: Tuple[float, float]) -> None:
    """地面火情轮（侦察机侧）：通报坐标 -> 回 G 点等任务机投弹。"""
    # 先播报再发事件，否则跟"任务机起飞"抢声光播报的顺序。
    sdk.announce('侦察机发现地面火情')
    sdk.announce('侦察机通报地面火情')
    sdk.send_to_teammate(ev.ground, x=fire_xy[0], y=fire_xy[1])
    sdk.hold_at(ROUTE_G[0], ROUTE_G[1], agl_m=CRUISE_AGL_M)
    sdk.wait_event(ev.dropped, 420.0)


def recon_high(sdk: DroneSDK, ev: Events,
               forward_m: float = FORWARD_BEFORE_FIRE_M) -> bool:
    """高层火情轮（侦察机侧）：巡检拍照 -> 协同破窗 -> 回 G 等灭火；返回本轮是否着火。"""
    # 进来之前前视相机就该配好了。
    def on_fire(_det: Any, bldg: str) -> None:
        """瞄准 -> 前移到发射点 -> 拍交付照 -> 通报 -> 等任务机 -> 复瞄 -> 破窗 -> 让位。"""
        if not sdk.aim_at(HIGH_FIRE, camera='front', what='高层火情',
                          timeout_s=RECON_AIM_TIMEOUT_S):
            sdk.progress('侦察机瞄准超时，按当前朝向即刻前移到发射点')
        # 瞄准只动朝向和高度，所以这一步是沿已经对准目标的机头方向走的。
        spot = sdk.step_forward(forward_m, what='发射点')
        sdk.snapshot(bldg)
        sdk.announce('侦察机通报高层火情')     # 先播报再发事件，抢顺序见上
        sdk.send_to_teammate(ev.high, x=spot[0], y=spot[1], z=spot[2], at=bldg)
        sdk.wait_event(ev.at_e, 300.0)
        # 等任务机最长要 300 秒，发射前补一次复瞄；复瞄不动水平位置，spot 仍有效。
        if not sdk.aim_at(HIGH_FIRE, camera='front', what='高层火情',
                          timeout_s=RECON_REAIM_TIMEOUT_S):
            sdk.progress('侦察机复瞄超时，按当前朝向即刻发射破窗弹')
        sdk.shoot(label='发射破窗弹', sound='侦察机发射破窗弹')
        sdk.send_to_teammate(ev.breached)
        sdk.announce('侦察机破窗完成')
        sdk.yield_spot(spot[:2], ev.spot_clear)   # 发射完就走，走到哪算让开了
        # aim_at 是"改朝向和高度、不平移"，两次瞄准都把飞机抬到了着火点高度；
        # 不收回来就一路带着飞，后面 formation_home 以 CRUISE_AGL_M 下发编队
        # 航线时垂直偏差过大，line_goto 直接判 tracking_lost 停发指令、双机卡死。
        # 任务机侧 supply_high_once() 结尾本来就有这一步，这里是补上对称的那半。
        sdk.set_agl(CRUISE_AGL_M)

    fired = sdk.patrol(STATIONS, HIGH_FIRE, agl_m=OBSERVE_AGL_M, on_found=on_fire,
                       scan_sound='侦察机排查高层火情', found_sound='侦察机发现高楼火情')
    if ev.inspect_done:
        sdk.send_to_teammate(ev.inspect_done)
    # 跟 recon_ground() 用同一个收尾：hold_at 会把高度一并钉到 CRUISE_AGL_M，
    # 而 fly_route 只管飞到 G。下面那个 wait_event 最长等 420 秒，进去之前
    # 必须保证高度是对的，否则等醒了直接进编队返航就会撞上垂直偏差。
    sdk.hold_at(ROUTE_G[0], ROUTE_G[1], agl_m=CRUISE_AGL_M)
    if fired:
        sdk.wait_event(ev.done, 420.0)
    return fired


# ---------------------------------------------------------------------------
# 任务机：一轮火情
# ---------------------------------------------------------------------------
def supply_ground(sdk: DroneSDK, d: Dict[str, Any], ev: Events,
                  drop_agl_m: float = DROP_AGL_M,
                  drop_tol_m: float = DROP_TOL_M,
                  drop_align_s: float = DROP_ALIGN_S) -> None:
    """地面火情轮（任务机侧）：起飞 -> 取灭火弹 -> 投放 -> 拍照回传；`d` 取自 ev.ground。"""
    # 开爪和切相机都放在起飞前：这会儿飞机还停着，不占飞行时间。
    sdk.grip(release=True, label='打开夹爪')
    use_camera(sdk, 'down')
    sdk.takeoff(height_m=CRUISE_AGL_M)
    # 2026-10-23 赛题改动：物资点上摆了两件，地面火情这一轮抓西边那件（ID0）。
    # precise=True 显式写出来（本来就是默认值）：两件只隔 0.6 米、在 2 米高度的
    # 下视里同时可见，必须靠 tag 对准挑对那一件，两轮的取物资口径保持一致。
    sdk.fetch_from(SUPPLY_XY_GROUND, SUPPLY_TAG, what='灭火弹', agl_m=CRUISE_AGL_M,
                   sound='任务机抓取灭火弹', precise=True)
    # 抓着弹时下视被挡住，这一段不做视觉对准，全程按侦察机给的坐标走。
    sdk.goto_world(d['x'], d['y'], CRUISE_AGL_M, what='地面火情点')
    tx, ty, tz = sdk.world_to_local(d['x'], d['y'], drop_agl_m)
    sdk.progress(f'降到 {drop_agl_m:.1f} m 并对准火情点，'
                 f'要求 {drop_tol_m*100:.0f} cm 以内、最多 {drop_align_s:.0f} 秒')
    # 这一步必须走 goto_direct：只有它能把到点判据收紧到 tol_m。
    sdk.goto_direct(tx, ty, tz, tol_m=drop_tol_m, max_wait_s=drop_align_s)
    sdk.announce('任务机投放灭火弹')
    sdk.grip(release=True, label='投放')
    sdk.snapshot('地面火情已扑灭', camera='down')   # 灭火完成的交付凭证
    sdk.set_agl(CRUISE_AGL_M)                      # 升回巡航高度，准备入列
    sdk.send_to_teammate(ev.dropped)


def supply_high_once(sdk: DroneSDK, d: Dict[str, Any], ev: Events,
                     takeoff: bool = True) -> None:
    """高层火情轮（任务机侧）一次灭火：（起飞取器材 ->）E 点待命 -> 破窗后进场发射。"""
    # `d` 取自 ev.high；已经在空中（第二次调用）就给 takeoff=False。
    if takeoff:
        # 开爪只能放在这一支：第二次调用时器材还抓在手上，开爪会把它扔下去。
        sdk.grip(release=True, label='打开夹爪')
        use_camera(sdk, 'down')      # 抓器材要看地面的物资标识
        sdk.takeoff(height_m=CRUISE_AGL_M)
        # precise=False：取器材只是降下去做个抓取动作，不要求对准标识。
        # 2026-10-23 赛题改动：高层火情这一轮也要取物资，抓东边那件（ID3）。
        # ⚠️ 这里**去掉了 precise=False**。原来不精降的依据是用户 2026-10-07
        #    "高楼火情物资点降落只是做一个动作，不需要精降"——那时物资点上只有
        #    一件东西，落在哪儿都是它。现在两件中心只隔 2*SUPPLY_TAG_OFFSET_M
        #    = 0.6 米、边缘才 0.1 米间隙，而 goto 到点误差约 0.3 米，不对准就会
        #    抓到旁边那件（或者抓空）。要改回去，前提是把两件拉得足够远。
        sdk.fetch_from(SUPPLY_XY_HIGH, SUPPLY_TAG_HIGH, what='灭火器材',
                       agl_m=CRUISE_AGL_M, sound='任务机抓取灭火弹', precise=True)
    sdk.goto_world(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M, what='E点待命位')
    sdk.send_to_teammate(ev.at_e)
    sdk.announce('任务机高层灭火已就位')
    sdk.wait_event(ev.breached, 300.0)
    sdk.wait_event(ev.spot_clear, 150.0)      # 发射点上这会儿还杵着侦察机
    use_camera(sdk, 'front')                  # 切换放在飞行之前，4 秒被转场盖住
    sdk.goto_world(d['x'], d['y'], d['z'], what='侦察机发射点', accept_m=2.0)
    sdk.announce('任务机到达瞄准点')
    # face_yaw_deg 是扫描的中心朝向，给上立面朝向能少扫几步就捕获。
    if not sdk.aim_at(HIGH_FIRE, camera='front', face_yaw_deg=FACADE_YAW_DEG,
                      what='高层火情', timeout_s=SUPPLY_AIM_TIMEOUT_S):
        sdk.progress('任务机瞄准超时，按当前朝向即刻发射')
    sdk.shoot(SHOTS, interval_s=1.0, label='发射灭火弹', sound='任务机发射灭火弹')
    sdk.send_to_teammate(ev.done)
    sdk.set_agl(CRUISE_AGL_M)        # 对准时被挪到了着火点高度，入列前收回来


def supply_return_gear(sdk: DroneSDK) -> None:
    """把灭火器材还回物资点（放东西不需要精降）。"""
    # 飞过去那一段不能用 direct=True：coordinate_goto 没有刹车段，会在点上绕圈。
    use_camera(sdk, 'down')
    # 器材是从东边（ID3）取的，放回原处。
    sdk.release_at(SUPPLY_XY_HIGH, SUPPLY_TAG_HIGH, what='灭火器材',
                   agl_m=CRUISE_AGL_M)
