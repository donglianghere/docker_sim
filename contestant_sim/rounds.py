#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四个选手程序的公共飞行段（消重版共用，`*_slim.py` 导入它）。

为什么有这个文件：`formation / groundfire / highrise / mission` 四个任务是**嵌套
子集**关系——mission 第 1 轮就是 formation，mission 的地面火情轮就是 groundfire，
高层火情轮就是 highrise。原来的 `*_lite.py` 四份各写一遍，同一段逻辑存在 2~4 份
拷贝（取弹投弹、巡检 on_fire 闭包、编队返航、限速样板），改一处要改四处、漏一处
就悄悄分叉。本文件把这些段落收成一份，每条教训也就只有一个家。

与 `*_lite.py` 的关系：**行为等价，原版保留不动**。两套可以并存（事件名各自在
自己程序里定义，互不干扰）。

场地参数全在 venue.py——仿真与真机只有那一个文件不同，本文件两边逐字节相同。
"""
from typing import Any, Dict, NamedTuple, Tuple

from contest_sdk import DroneSDK
from contest_sdk.exceptions import ContestSdkError

from venue import (ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_D, ROUTE_E, ROUTE_F, ROUTE_G,
                   SUPPLY_XY, SUPPLY_TAG, HIGH_FIRE, FACADE_YAW_DEG, STATIONS,
                   CRUISE_AGL_M, OBSERVE_AGL_M, SHOTS)

# ---------------------------------------------------------------------------
# 投放灭火弹的三个参数（2026-10-16 用户定，选手层可改）
# ---------------------------------------------------------------------------
#: 释放灭火弹时的离地高度（米）。原来是在巡航高度 CRUISE_AGL_M(2.0) 直接开爪，
#: 弹要自由落体 2 米——落体时间约 0.64 s，期间飞机的残余水平速度会被原样带进
#: 落点误差。降到 1.0 m 之后落体时间约 0.45 s，同样的残余速度带来的偏移少一半。
DROP_AGL_M = 1.0
#: 释放前要求对准到多少米以内。走 coordinate_goto 的到点判据（水平和垂直共用），
#: 机载默认是 0.30 m（那是"最后一段精修"的尺度），投弹收紧到 0.10 m。
DROP_TOL_M = 0.10
#: 对准最多花几秒。到点就放行——**收不进 DROP_TOL_M 也照投**，按当时的位置。
#: 这是 coordinate_goto_max_wait_s，机载侧的兜底，所以不会把流程卡死
#: （2026-10-07 那次"任务机在起降点上空一直盘旋降不下来"就是这个兜底没开）。
DROP_ALIGN_S = 5.0

#: 编队飞行整条航线。E->F->G 是从小区里边绕回 G 的那段（F 是虚拟点，2026-10-11）。
ROUTE_FULL = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G, ROUTE_E, ROUTE_F, ROUTE_G, ROUTE_D]
#: 编队段"关键航点"下标——只有这些航点停下来等僚机收间距（2026-10-13）。
#:
#: 下标是 `lead_formation()` 内部拼出来的 **wps** 的下标，不是 ROUTE_FULL 的：
#: 它会把 `final_xy` 接到尾巴上，所以 formation_slim 那条实际是
#: ``[A, B, C, G, E, F, G, D, A]``，共 9 个点、下标 0~8。0 号点是起步点，
#: `lead_formation` 自己会跳过（`_on_wp` 里 `i <= 0` 直接返回）。
#:
#: 为什么是这四个（1=B, 2=C, 4=E, 5=F）：那个停顿是给僚机收间距用的，
#: 所以要停在"间距最容易被拉开"的地方。
#:   · B(1) / C(2)：A→B 19.0 m、B→C 14.0 m 是全航线最长的两段，落后量按里程
#:     累积，这两段末尾最大。
#:   · E(4) / F(5)：E→F→G 那个折返只有 5.3 m 一段但折角 131°/98°，是
#:     ``project_formation_collision_arclength`` 记的"弧长间距失真"发源地
#:     （弧长涨了而两机空间距离没动）。
#: 去掉的是 3=G、6=G、7=D、8=A：G(3) 和 G(6) 前面都是短段（9.5/5.3 m），
#: D(7) 和 A(8) 在解散段上、僚机已经出列，停了没有意义。
#:
#: ⚠️ 停留同时也是速度剖面的 **0 速断点**。不停的拐点速度仍受
#: ``corner_dv_max_mps`` 限（0.4 m/s 配 90° 折角 => 0.28 m/s），不是满速过弯。
FORMATION_STOPS = (1, 2, 4, 5)

#: 侦察机每轮出发段：裁判盯着"侦察机过 G 点"才放火情，到 G 之前场上什么都没有。
ROUTE_TO_G = [ROUTE_A, ROUTE_B, ROUTE_C, ROUTE_G]
NAMES_TO_G = ['A', 'B', 'C', 'G']

#: 高层火情瞄准的墙钟预算（秒，用户 2026-10-14 定）。超时都**不阻塞流程**：
#: 侦查机超时即刻播报坐标给任务机，任务机超时即刻发射灭火弹。
#: 侦查机给得宽一些：它要从巡检姿态开始扫描捕获；任务机已经按侦查机给的坐标
#: 进场、目标大概率在视场内，只需精调。
RECON_AIM_TIMEOUT_S = 30.0
SUPPLY_AIM_TIMEOUT_S = 15.0
#: 侦查机**发射破窗弹之前**的复瞄预算（秒）。短，因为目标已在视场内、
#: 只需精调，不用扫描。超时同样即刻发射。
RECON_REAIM_TIMEOUT_S = 10.0

#: 全程规划器巡航限速兜底值，米/秒（2026-10-09 用户指定）。见 cruise_limit()。
DEFAULT_MAX_VEL_MPS = 0.8


class Events(NamedTuple):
    """一个程序自己的事件名表。

    四个任务的握手时序完全一样，只有事件名不同（groundfire 叫
    `ground_fire_found`、mission 叫 `mission_ground_fire`）。把名字收进一张表当
    参数传，公共段就不用为了名字再复制一遍。

    字段按"谁发给谁"分组：
      侦察机 -> 任务机：ground / high / breached / spot_clear / inspect_done
      任务机 -> 侦察机：dropped / at_e / done
    """
    ground: str = ''        #: 地面火情坐标（x, y）
    dropped: str = ''       #: 灭火弹已投放
    high: str = ''          #: 高层火情发射点（x, y, z, at）
    at_e: str = ''          #: 任务机已在 E 点待命
    breached: str = ''      #: 破窗完成
    spot_clear: str = ''    #: 发射点已让开
    done: str = ''          #: 高层火情已扑灭
    inspect_done: str = ''  #: 巡检结束、本轮没着火（只有 highrise 用，见 recon_high）


# ---------------------------------------------------------------------------
# 通用动作
# ---------------------------------------------------------------------------
def cruise_limit(sdk: DroneSDK, default: float = DEFAULT_MAX_VEL_MPS) -> None:
    """设全程巡航限速。运行器的 `--max-vel` 优先，没给就用 `default`。

    改的是 `ego_planner` 的 `manager/max_vel`，管所有经规划器的航段
    （goto / goto_world / fly_route / lead_formation）。**不管 `goto_direct`**
    ——那条是 precision_servo 的 `coordinate_max_speed_mps`，由 set_direct_speed()
    另管。

    为什么每个入口只设一次就够全程：这是**节点级**参数、不会自己复位；而编队那套
    "每段起步压限速再放回巡航"（_slow_leg_start）的巡航值取的是**压之前的当前值**
    （cruise 初始为 None -> 取 set_max_vel 的返回值），所以它压完 LEG_SLOW_MPS 之后
    会放回这里设的值，不会把限速顶回 V_MAX。

    ⚠️ 节点级参数任务结束后**会残留**在 ego_planner 里。真机每次飞行前都重建容器
    （断电重启流程）所以无妨；如果不重启就接着跑别的程序，注意它仍是这个值。
    """
    sdk.set_max_vel(sdk.max_vel_mps or default)


def use_camera(sdk: DroneSDK, which: str) -> None:
    """配相机：`'front'` / `'down'` 只开那一路，`'none'` 只留前视守会话。

    为什么值得这么做（2026-10-03 真机实测，8 核整机）：0 路 63.4%、1 路 73~77%、
    2 路 88~90%，一路 yolo_detector_node 约值 12 个百分点。而本项目的两路从来不
    重叠：地面火情用下视、高层火情瞄准用前视、取放物资用下视（精降看 AprilTag）。
    切换代价实测 **4 秒**，所以调用点都选在后面紧跟一段飞行的位置，4 秒被转场
    盖住、等于不花时间。

    `set_camera_mode()` 自己已经做了两路互斥（开一路必先停另一路），所以这个函数
    **不是**为了"少写一行"而存在，它只做两件 SDK 不该管的事：
      ① `'none'` ——整段不碰相机的阶段（编队飞行只有 lead_formation/hold_at、
         follower 侧只有 follow_formation/return_home），停掉下视、前视降到
         `raw`（见下面代码里 2026-10-18 那段：**两路不能同时全空**）；
      ② **失败不中止**——见下面 except 分支。省 CPU 不该有权力打断一次比赛。

    仿真下 `set_camera_mode()` 是空操作（没有 control_server，检测节点随飞行栈
    一起起），所以这个函数在仿真和真机都能原样跑。

    Args:
        which: `'front'` 前视 / `'down'` 下视 / `'none'` 只留前视 `raw` 守会话。
    """
    try:
        if which == 'none':
            # 🔴 **不能两路全停。** 2026-10-18 真机实测：NX02 的 cam1 一旦经历
            #    "两路 Argus 会话同时全空超过几秒"就再也起不来——Argus 照样报
            #    会话建立成功，紧接着 `nvbuf_utils: dmabuf_fd -1 mapped entry
            #    NOT found` + `camera read timeout, no frame`，而且先起健康的
            #    cam0 再起它也救不回，重启 nvargus-daemon / vision 容器都无效，
            #    只能给飞机断电。
            #
            #    这就是 t1_抓取.py 次次成功、mission 次次失败的**全部**差别：
            #    两边写法一样，但 t1 是从 cam1 running 直接重启（全空只有
            #    kill_vision_detector_processes() 里那 1 秒），mission 在编队段
            #    把两路晾了好几分钟。短暂全空(1~2s)安全，长时间全空致命。
            #    NX01 对全空免疫（同序列三轮全过），NX02 不免疫。
            #
            #    留前视跑 raw（纯视频直通、不推理）守住 Argus 会话即可绕开。
            #    set_camera_mode() 的互斥会顺带把下视停掉，所以这一行就够；
            #    raw 不是 'yolo'，不会等检测数据，立即返回。
            #    代价实测（top 的 idle 反推）：raw 守会话 +5.7pt 整机(80.1%)，
            #    换成开第二路 yolo 要 +12.2pt(86.6%)。
            sdk.set_camera_mode('front', 'raw')
            return
        sdk.set_camera_mode(which, 'yolo')      # 互斥在 SDK 里；wait=True 等出数据
    except ContestSdkError as exc:
        # **降级而不是中止。** 省 CPU 不该有权力打断一次飞行：切换走 WiFi 打到
        # 飞机，那条链路会间歇性断（SDK 内部已经重试 3 次，撞上长断链还是会抛）。
        #
        # 不能退回"两路全开"：互斥之后做不到（依次对 front/down 各调一次 yolo，
        # 第二次会把第一次那路停掉）。只重试一次本阶段真正要用的那一路，不等
        # 数据——等的事交给后面的 wait_for_detection 报。
        sdk.progress(f'[警告] 配相机失败（{exc!r}），重试一次只开 {which} 那一路')
        try:
            sdk.set_camera_mode(which, 'yolo', wait=False)
        except ContestSdkError:
            sdk.progress('[警告] 相机配置彻底失败，本轮不再切换；后面要用相机的'
                         '步骤会在 wait_for_detection 处报超时')
        else:
            sdk.progress(f'[警告] 已降级：{which} 相机已请求开启但没等数据，'
                         f'本轮不再切换')


def formation_home(sdk: DroneSDK, final_xy: Tuple[float, float]) -> None:
    """侦察机带队从 G 点编队返航，任务机过 D 点就解散。

    `disband_at=D` 要显式给。不给的话默认判据是"长机飞回**自己起飞点**上空"，而
    起降点根本不在航线上、只是碰巧跟最后一段共线——编队会在离终点还有几米的半路
    上散掉，散在一个跟任务无关的点上；起降点哪天挪开这条线，这个判据就永远不成立。
    D 点是两机路径真正的分岔口。

    `tail_direct=True`：解散后各自回家的尾段走直飞。那段不在航线上，没有避障考核
    要求。**航线段一律走规划器**——2026-10-07 18:31 撞机那趟长机在 C 点打圈就发生
    在直飞段上（goto_direct 走 precision_servo 的 P 控制，同样没有终端减速）。

    Args:
        final_xy: 长机解散后去哪儿。回 A 点待命就给 ROUTE_A，就地降落给
            `sdk.own_pad()`。
    """
    sdk.lead_formation([ROUTE_G, ROUTE_D], agl_m=CRUISE_AGL_M, start_xy=ROUTE_G,
                       final_xy=final_xy, disband_at=ROUTE_D, tail_direct=True)


# ---------------------------------------------------------------------------
# 侦察机：一轮火情
# ---------------------------------------------------------------------------
def recon_ground(sdk: DroneSDK, ev: Events,
                 fire_xy: Tuple[float, float]) -> None:
    """地面火情轮（侦察机侧）：通报坐标 -> 回 G 点等任务机投弹。

    先播报再发事件，否则跟"任务机起飞"抢声光播报的顺序。
    """
    sdk.announce('侦察机发现地面火情')
    sdk.announce('侦察机通报地面火情')
    sdk.send_to_teammate(ev.ground, x=fire_xy[0], y=fire_xy[1])
    sdk.hold_at(ROUTE_G[0], ROUTE_G[1], agl_m=CRUISE_AGL_M)
    sdk.wait_event(ev.dropped, 420.0)


def recon_high(sdk: DroneSDK, ev: Events, forward_m: float = 1.0) -> bool:
    """高层火情轮（侦察机侧）：逐楼巡检拍照 -> 遇火情协同破窗 -> 回 G 等灭火完成。

    返回本轮**有没有着火**。进来之前前视相机就该配好了——调用方才知道"本轮是不是
    高层"，在这里再切一次是重复的。

    `ev.inspect_done` 给了就在巡检结束后发一次（任务机那边用 wait_any_event 盯着
    它跳出等待循环）。mission 不用这个握手：它的任务机是靠 EV_KIND 预先知道本轮
    种类的，不需要"巡检完了但没着火"这条通知。

    Args:
        forward_m: 从发现点沿机头方向前移多少米到发射点（2026-10-14 用户要求提成
            参数，默认 1.0；原来写死在 venue.FORWARD_BEFORE_FIRE_M=1.5，那个常量
            已删——它不是场地参数，是跟弹药射程有关的战术参数）。
    """
    def on_fire(_det: Any, bldg: str) -> None:
        """瞄准 -> 前移到发射点 -> 拍交付照 -> 通报 -> 等任务机到位 -> 复瞄 -> 破窗 -> 让位。

        2026-10-14 按用户要求改了三处：①交付照挪到**前移之后**拍（原来在发现点拍）；
        ②前移距离提成参数 forward_m（默认 1.0 m）；③**发射破窗弹之前补一次复瞄**。
        """
        # ① 发现点瞄准，预算 30 秒。**超时不阻塞**：aim_at 返回 False 也照常往下走。
        if not sdk.aim_at(HIGH_FIRE, camera='front', what='高层火情',
                          timeout_s=RECON_AIM_TIMEOUT_S):
            sdk.progress('侦察机瞄准超时，按当前朝向即刻前移到发射点')
        # ② 沿机头方向前移到发射点。瞄准只动朝向和高度、不动水平位置，所以这一步
        #    是沿"已经对准目标的机头方向"走的，方位角不会变坏（距离更近，同样的
        #    方位容差对应的横向偏差还更小）。
        spot = sdk.step_forward(forward_m, what='发射点')
        # ③ 到发射点之后再拍交付照（用户 2026-10-14 要求）。
        sdk.snapshot(bldg)
        sdk.announce('侦察机通报高层火情')     # 先播报再发事件，抢顺序见上
        sdk.send_to_teammate(ev.high, x=spot[0], y=spot[1], z=spot[2], at=bldg)
        sdk.wait_event(ev.at_e, 300.0)
        # ④ 发射前复瞄：上面那次瞄准到这会儿**最多已经过去 300 秒**（等任务机到
        #    E 点），悬停期间 UWB 漂移/风/位置环抖动都会累积，而破窗弹才是侦查机
        #    真正要命中的一发。短预算 10 秒，目标已在视场内、只需精调。
        #    复瞄只改朝向和高度，不动水平位置，所以上面播报的 spot[:2] 仍然有效
        #    （下面 yield_spot 还要用它）；高度可能被微调，但任务机到场后自己也会
        #    再瞄一次，不影响。
        if not sdk.aim_at(HIGH_FIRE, camera='front', what='高层火情',
                          timeout_s=RECON_REAIM_TIMEOUT_S):
            sdk.progress('侦察机复瞄超时，按当前朝向即刻发射破窗弹')
        sdk.shoot(label='发射破窗弹', sound='侦察机发射破窗弹')
        sdk.send_to_teammate(ev.breached)
        sdk.announce('侦察机破窗完成')
        sdk.yield_spot(spot[:2], ev.spot_clear)   # 发射完就走，走到哪算让开了

    fired = sdk.patrol(STATIONS, HIGH_FIRE, agl_m=OBSERVE_AGL_M, on_found=on_fire,
                       scan_sound='侦察机排查高层火情', found_sound='侦察机发现高楼火情')
    if ev.inspect_done:
        sdk.send_to_teammate(ev.inspect_done)
    sdk.fly_route([ROUTE_G], agl_m=OBSERVE_AGL_M, names=['G'])
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
    """地面火情轮（任务机侧）：起飞 -> 取灭火弹 -> 投放 -> 拍照回传。

    全程下视（fetch_from 边瞄边降看标识、snapshot 也是下视）。相机放在 takeoff
    **之前**切：等事件时本来就是停着的，这 4 秒完全不占飞行时间。

    Args:
        d: 侦察机 `ev.ground` 事件的载荷，取 `d['x']` / `d['y']`。
        drop_agl_m: 释放灭火弹的离地高度（米）。默认 `DROP_AGL_M`。
        drop_tol_m: 释放前要求对准到多少米以内。默认 `DROP_TOL_M`。
        drop_align_s: 对准最多花几秒，到点就放行（对不准也照投）。
            默认 `DROP_ALIGN_S`。
    """
    # 2026-10-15 用户要求：收到火情消息就先把夹爪打开，别等到了物资点才开。
    # 这会儿飞机还停在地上等事件，开爪的行程时间（GRIP_TRAVEL_S）完全不占飞行
    # 时间——理由跟下面 use_camera 放在 takeoff 之前是同一条。
    # 幂等：上一轮投放/归还时爪子本来就已经松开了，再发一次没有副作用；它挡的是
    # "上一轮结束时爪子不知为何停在闭合位，这一轮到了物资点抓不住"那种失败。
    sdk.grip(release=True, label='打开夹爪')
    use_camera(sdk, 'down')
    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火弹', agl_m=CRUISE_AGL_M,
                   sound='任务机抓取灭火弹')
    # 抓着灭火弹的时候下视相机被弹体挡住，看不到地面火情 tag——这一段**不做视觉
    # 对准**，全程按侦察机给的坐标走（用户 2026-10-06 说明）。原来这儿调的
    # aim_at(camera='down') 走的是 center_on_target 图像伺服，相机被挡时它只会
    # 一直等到超时（2026-10-06 实测卡死 40 秒就是这条路）。
    #
    # 2026-10-16 用户要求把投放从"飞到就开爪"改成三步：**降到 DROP_AGL_M +
    # 对准到 DROP_TOL_M + 投完升回巡航高度**。
    # ① 先用 goto_world 走规划器过去（有避障），高度仍是巡航高度；
    # ② 再用 goto_direct 降到投放高度并精确对准——它走机载 coordinate_goto，
    #    拿目标坐标跟里程计作差做闭环，不碰视觉（正好，相机这会儿被弹体挡着）。
    #    tol_m 把到点判据从机载默认的 0.30 收紧到 0.10；收不进也不会卡死，
    #    max_wait_s 到点后机载按当前位置放行（见 goto_direct 里那段事故说明）。
    #    这一步**不能用 goto_world**：它走 ego_planner，到点判据是 0.3 m 量级、
    #    也没有"对准到指定精度"这个概念。
    # ③ 投完 set_agl 原地升回巡航高度，后面的入列返航才有正常的飞行高度。
    sdk.goto_world(d['x'], d['y'], CRUISE_AGL_M, what='地面火情点')
    tx, ty, tz = sdk.world_to_local(d['x'], d['y'], drop_agl_m)
    sdk.progress(f'降到 {drop_agl_m:.1f} m 并对准火情点，'
                 f'要求 {drop_tol_m*100:.0f} cm 以内、最多 {drop_align_s:.0f} 秒')
    sdk.goto_direct(tx, ty, tz, tol_m=drop_tol_m, max_wait_s=drop_align_s)
    sdk.announce('任务机投放灭火弹')
    sdk.grip(release=True, label='投放')
    # 拍照放在升高之前：弹已经脱手、下视不再被挡，这个高度拍得比巡航高度清楚。
    sdk.snapshot('地面火情已扑灭', camera='down')   # 灭火完成的交付凭证
    sdk.set_agl(CRUISE_AGL_M)                      # 升回巡航高度，准备入列
    sdk.send_to_teammate(ev.dropped)
    # 灭火弹已经投到火点上了，没东西可还，调用方直接入列返航


def supply_high_once(sdk: DroneSDK, d: Dict[str, Any], ev: Events,
                     takeoff: bool = True) -> None:
    """高层火情轮（任务机侧）一次灭火：（起飞取器材 ->）E 点待命 -> 破窗后进场发射。

    写成"一次"：highrise 那边可能着火的两栋楼，侦察机可能通报两次，所以调用方套
    一层循环、第二次起传 `takeoff=False`。器材是抓在手上带去发射的，打完还在机上,
    还回物资点由调用方决定（mission 要还、groundfire 没有这一步）。

    Args:
        d: 侦察机 `ev.high` 事件的载荷，取 `d['x']` / `d['y']` / `d['z']`。
        takeoff: 还在地上就给 True（起飞 + 取器材）；已经在空中就给 False。
    """
    if takeoff:
        # 2026-10-15 用户要求：收到火情消息就先把夹爪打开（同 supply_ground）。
        # ⚠️ **只能放在 takeoff 这一支里**：highrise 两栋楼都着火时这个函数会被调
        # 用第二次（takeoff=False），那时**器材还抓在手上**（打完还在机上，还回
        # 物资点由调用方决定），在那儿开爪会把器材当场扔下去。
        sdk.grip(release=True, label='打开夹爪')
        # 抓器材要看地面上的物资标识，必须切下视——两路相机互斥，为瞄高层火情切的
        # 前视会让下视没有任何检测数据，land_on 拿不到坐标就只能退化成普通降落
        # （2026-10-06 真机实测：任务机飞到物资点上空后一直悬停不降，就是这么来的）。
        use_camera(sdk, 'down')
        sdk.takeoff(height_m=CRUISE_AGL_M)
        # precise=False：取灭火器材只是"降下去做个抓取动作"，不要求对准标识
        # （用户 2026-10-07 定）。走精降反而会在物资点上空等视觉、降不下来。
        # 地面火情轮取灭火弹仍然精降——那个不对准抓不到。
        sdk.fetch_from(SUPPLY_XY, SUPPLY_TAG, what='灭火器材', agl_m=CRUISE_AGL_M,
                       sound='任务机抓取灭火弹', precise=False)
    sdk.goto_world(ROUTE_E[0], ROUTE_E[1], CRUISE_AGL_M, what='E点待命位')
    sdk.send_to_teammate(ev.at_e)
    sdk.announce('任务机高层灭火已就位')
    sdk.wait_event(ev.breached, 300.0)
    sdk.wait_event(ev.spot_clear, 150.0)      # 发射点上这会儿还杵着侦察机
    # 改前视：下面 aim_at 用前视瞄高层火情。切换放在飞往发射点**之前**，这段飞行
    # 把 4 秒盖住；放到 aim_at 前面就要停在那儿干等。
    use_camera(sdk, 'front')
    sdk.goto_world(d['x'], d['y'], d['z'], what='侦察机发射点', accept_m=2.0)
    sdk.announce('任务机到达瞄准点')
    # 瞄准预算 15 秒（用户 2026-10-14 定）——比侦查机短，因为它已经拿到侦查机
    # 给的发射点、目标大概率已在视场内，只需要精调朝向和高度。
    # **超时即刻发射**：aim_at 返回 False 也照常 shoot，不把流程堵死。
    # face_yaw_deg 现在是**扫描的中心朝向**（不再是全程锁死的朝向），给上立面
    # 朝向能少扫几步就捕获。
    if not sdk.aim_at(HIGH_FIRE, camera='front', face_yaw_deg=FACADE_YAW_DEG,
                      what='高层火情', timeout_s=SUPPLY_AIM_TIMEOUT_S):
        sdk.progress('任务机瞄准超时，按当前朝向即刻发射')
    sdk.shoot(SHOTS, interval_s=1.0, label='发射灭火弹', sound='任务机发射灭火弹')
    sdk.send_to_teammate(ev.done)
    sdk.set_agl(CRUISE_AGL_M)        # 对准时被挪到了着火点高度，入列前收回来


def supply_return_gear(sdk: DroneSDK) -> None:
    """把灭火器材还回物资点。

    放东西**不需要精降**（用户 2026-10-07 定），release_at 默认就不碰视觉；这里
    仍把相机切回下视是道防线：上面瞄高层火情时切成了前视，万一哪天把这一步改成
    `precise=True`，前视是看不到物资点标识的——highrise 就因为这个（相机停在前视
    去做精降）在物资点上空怎么都降不下来。

    ⚠️ **飞过去那一段不能用 `direct=True`**，要和上面 `supply_take_gear` 取器材
    走同一个飞行函数（`goto_world` 默认路径 -> `goto` -> line_goto）。
    2026-10-14 真机实测：同一趟飞行、同一个物资点，取器材（默认路径）10 s 干净
    到位，放回（`direct=True`）在物资点上空**绕了 24 s 的圈**才落，最后还偏了
    0.4 m。原因是 `direct` 走 `coordinate_goto`，它是给"最后半米精修"设计的纯 P
    控制、没有刹车段：`_direct_leg` 把限速提到 1.0 m/s 后，只要离目标还有 0.2 m
    以上，发给 PX4 的速度前馈就恒等于 1.0 m/s（和剩余距离无关），而位置设定点只
    领先 0.1 m——等于定常速度追踪，叠加约 0.5 s 闭环延迟就在目标点外侧绕成半径
    v·τ≈0.5 m 的极限环（实测正是 0.5 m），套不进 `arrival_threshold_m=0.3` 的
    到达球。限速留默认 0.3 m/s 时半径只有 0.15 m、落在球内，所以这个坑只在被
    `_direct_leg` 提速的长航段上暴露。line_goto 有完整梯形速度剖面、终点速度
    归零，没有这个问题。
    """
    use_camera(sdk, 'down')
    sdk.release_at(SUPPLY_XY, SUPPLY_TAG, what='灭火器材', agl_m=CRUISE_AGL_M)
