#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机地面灭火：侦察机搜火情并通报，任务机取灭火弹、投放、返航。

    bash 运行仿真.sh --task 地面火情搜索示例.py

一份代码两架飞机各起一个容器，按 --role 分工（运行脚本会传）：
  leader（NX01，侦察机）：弓字搜索 -> 发现火点 -> 悬停解算坐标 -> 通报队友 -> 返航降落
  follower（NX02，任务机）：等通报 -> 起飞 -> 取灭火弹（精准降落+抓取）-> 飞到火点
                            -> 对准 -> 投放 -> 返回起飞点精准降落

侦察机这边，主线程按弓字航线逐点飞；后台线程全程盯着下视相机，一看到火点就
打断当前航段——必须这样，一条航段要飞十几秒，等它飞完再检查早就飞过火点了。

火点坐标用 sdk.locate_target() 取：飞行栈按相机内参、拍照时刻的位姿和地面
高度算出的火点实际位置，不是飞机自己的位置（火点在画面边缘时两者能差 1.5 米）。

通报走 send_to_teammate()：带应用层 ACK + 1Hz 重发，对方确认收到才返回，
所以两架的启动先后无所谓（任务机先起来就一直等）。

关键节点都有声光：起飞、降落由 takeoff()/land() 按角色自动播，其余（发现/
通报火情、收到通报、发现灭火弹、抓取、投放、任务完成）在下面各步里播。
"""
import argparse
import threading
import time

from contest_sdk import DroneSDK
from contest_sdk.exceptions import (
    ActionFailedError,
    DetectionTimeoutError,
    GotoUnreachableError,
    TeammateUnreachableError,
)

# 搜索区域：编队航线四个航点围成的四边形（世界坐标）
AREA = [(7.0, -9.5), (7.0, 9.5), (-7.0, 9.5), (-7.0, -9.5)]
CRUISE_AGL_M = 2.5          # 飞高一点覆盖更宽；必须低于 规划器天花板-0.1-dist0（见launch注释）
GROUND_FIRE = 'apriltag:2'
FIRE_MARKER_SIZE_M = 0.5    # 地面火点标志尺寸：航线间距要扣掉它，保证目标能完整入画
OVERLAP = 0.2               # 扣掉目标尺寸之后，再留 20% 给定位误差
HOVER_S = 10.0              # 找到后在火点上方悬停多久，再返航降落
FIRE_REPORT_EVENT = '地面火情通报'   # 两架飞机约定的事件名，改一处就要改两处

# ---- 任务机（follower）用的常量 ----
WAIT_REPORT_S = 900.0        # 等通报等多久：侦察机要扫完弓字航线才可能发现火点
SUPPLY_POINT = (-4.0, -6.0)  # 物资点（灭火弹）世界坐标，题目给定，贴 AprilTag ID0
SUPPLY_TAG = 'apriltag:0'
GRAB_PWM = 800               # 抓紧——2026-09-21 NX02 真机实测确认
DROP_PWM = 2000              # 松开
SERVO_TRAVEL_S = 2.0         # 等舵机转到位（舵机没有位置反馈，只能等）
PRECISION_LAND_S = 30.0      # 边瞄准边降落这一段的总时限，到点就交给普通降落
DESCENT_STEP_M = 0.4         # 每一步下降多少：降一点就重新解算一次目标位置
HANDOFF_AGL_M = 0.7          # 降到离地这么高就交给普通降落（再低下视相机看不全标志）
HANDOFF_TOL_M = 0.15         # 到交接高度附近就算到了：悬停本身有零点几十厘米的起伏，
                             # 死等"严格低于交接高度"会一直卡在上面空耗（实测卡满30秒）
DROP_HOLD_S = 3.0            # 投放后在火点上方多停一会，确认弹已脱手

# 题目给的 3 根立柱（坐标已知，可以写进程序）。r 是方立柱的半对角线（截面
# 1 米 -> 0.71），余量必须不小于规划器的障碍物膨胀半径（仿真0.6、真机0.8）。
# ⚠️ 3# 立柱在 (0,0)，正落在搜索区中心：弓字行距 2.52 米、立柱禁入半径
# 0.71+0.6=1.31 米，相邻两行跨过柱心时最好也只能各离 1.26 米（行距的一半），
# 也就是**总有一行会压进膨胀区**，靠规划器绕过去，绕的那一下会慢几秒。
# 想彻底避开只能缩小行距（航线更密、更慢）或者把搜索区让开这根柱子。
PILLARS = [(4.5, 7.0, 0.71), (-4.5, 7.0, 0.71), (0.0, 0.0, 0.71)]
CLEARANCE_M = 0.8


class FireWatcher:
    """后台线程：只认下视相机，看到火点就算出火点坐标、打断当前航段。"""

    def __init__(self, sdk):
        self.sdk = sdk
        self.found = threading.Event()
        self.fire_local = None          # 解算出的火点局部坐标（x, y），没解算出来是 None
        self.seen_from = None           # 看到火点时飞机的局部坐标，只用来飞回去再看，不当火点坐标
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def join(self, timeout=3.0):
        """等这个线程收尾。**发现目标后必须先 join 再发新的飞行指令**：
        线程在命中后会连发两次 cancel_goto()（间隔 0.5 秒），主线程如果紧接着
        就 goto()，会被那第二次取消掉——实测过一次，飞机因此停在 9 米外没能
        飞到瞄准位置，日志里是"目标点已被取消，goto()提前返回"。"""
        self._thread.join(timeout)

    def _run(self):
        while not self._stop.is_set():
            try:
                self.sdk.wait_for_detection(GROUND_FIRE, timeout=5.0, camera='down')
            except DetectionTimeoutError:
                continue
            self.seen_from = self.sdk.get_local_position()
            # 趁火点还在画面里、边飞边定位：飞行栈按拍照时刻的位姿算，已经
            # 补偿了飞行中的检测延迟。只取 3 帧，求快——悬停后还会再精测一次。
            try:
                fire = self.sdk.locate_target(GROUND_FIRE, timeout=2.0, samples=3)
                self.fire_local = (fire.x, fire.y)
            except DetectionTimeoutError:
                pass                    # 火点擦过画面边缘就出去了；悬停阶段飞回去再解算
            self.found.set()
            self.sdk.play_sound_light('侦察机发现地面火情')
            # 连打断两次：主线程可能正好在两个航点之间，第一次打断落空，
            # 紧接着又发出了下一个航点
            for _ in range(2):
                self.sdk.cancel_goto()
                time.sleep(0.5)
            return


def build_route(sdk):
    xs = [p[0] for p in AREA]
    ys = [p[1] for p in AREA]
    # 从离飞机近的那一端起扫（用户 2026-09-24 提的）：起降点在场地 x 正方向
    # 这一半，默认的"从 x_min 端起扫"要先空飞整个场地宽度（8 米）到对面去才
    # 开始扫，而这条去程贴着两机起降点的连线飞——按赛场规则火情目标不会摆在
    # 那条线上，这段纯属浪费。两种起法覆盖范围完全一样，只是省掉这段去程。
    here = sdk.local_to_world(*sdk.get_local_position())
    route = sdk.generate_ground_scan_waypoints(
        room_min_x=min(xs), room_max_x=max(xs),
        room_min_y=min(ys), room_max_y=max(ys),
        altitude_agl=CRUISE_AGL_M, target_size_m=FIRE_MARKER_SIZE_M, overlap_ratio=OVERLAP,
        start_from_x_max=here[0] > (min(xs) + max(xs)) / 2.0)
    return sdk.pull_waypoints_out_of_circles(route, PILLARS, CLEARANCE_M)


def search(sdk, watcher):
    route = build_route(sdk)
    print(f'[{sdk.namespace}] 弓字搜索开始，共 {len(route)} 个航点', flush=True)
    # 整条搜索航线定高：航点本来就都是同一个高度，钉住之后下视相机的地面覆盖
    # 宽度也恒定（行距就是按这个高度算的），不会因为轨迹高度波动漏扫
    with sdk.fixed_altitude(sdk.world_to_local(0.0, 0.0, CRUISE_AGL_M)[2]):
        for i, (wx, wy, wz) in enumerate(route, start=1):
            if watcher.found.is_set():
                break
            print(f'[{sdk.namespace}] 航点 {i}/{len(route)}: ({wx:.1f}, {wy:.1f})', flush=True)
            try:
                sdk.goto(*sdk.world_to_local(wx, wy, wz))
            except GotoUnreachableError:
                # 航点落在障碍物里（未知坐标的那些），跳过——火点不会在障碍物底下
                print(f'[{sdk.namespace}] 航点 {i} 不可达，跳过', flush=True)
    return watcher.found.is_set()


def hover_over_fire(sdk, watcher):
    """飞到火点上方悬停，报告火点坐标。报告的坐标只来自二维码解算，
    不拿飞机自己的位置充数；解算不出来就如实报"没解算出来"。"""
    _, _, z = sdk.get_local_position()
    # 1. 飞到火点上方：飞行中解算出来了就直接去火点正上方；没解算出来就
    #    回到看到它的位置（那里火点一定在画面里）。刚飞过、只有一两米，直线飞。
    if watcher.fire_local is not None:
        tx, ty = watcher.fire_local
    else:
        tx, ty, _ = watcher.seen_from
    sdk.goto_direct(tx, ty, z)

    # 2. 在上方再精测一次：火点靠近画面中央，飞机也停稳了，比飞行中擦边看到
    #    时准。偏差超过 0.1 米就挪到正上方。
    try:
        fire = sdk.locate_target(GROUND_FIRE, timeout=5.0, samples=10)
        if abs(fire.x - tx) > 0.1 or abs(fire.y - ty) > 0.1:
            sdk.goto_direct(fire.x, fire.y, z)
        fx, fy = fire.x, fire.y
        how = f'悬停时解算（{fire.samples}帧，离散 {fire.spread_m:.2f}m）'
    except DetectionTimeoutError:
        if watcher.fire_local is None:
            print(f'[{sdk.namespace}] 看到了地面火情但没能解算出坐标（火点不在下视画面里），不报坐标',
                  flush=True)
            time.sleep(HOVER_S)
            return
        fx, fy = watcher.fire_local
        how = '飞行中解算'
    wx, wy, _ = sdk.local_to_world(fx, fy, 0.0)
    print(f'[{sdk.namespace}] 悬停在地面火情上方，火点世界坐标 ({wx:.2f}, {wy:.2f})，{how}',
          flush=True)
    sdk.play_sound_light('侦察机通报地面火情')
    # 通报给任务机：带 ACK + 1Hz 重发，对方确认收到才返回；对方没起来时
    # 抛超时——这时也别卡住不动，照样返航降落，坐标已经打印在日志里了
    try:
        sdk.send_to_teammate(FIRE_REPORT_EVENT, x=wx, y=wy)
    except TeammateUnreachableError as exc:
        print(f'[{sdk.namespace}] 火情通报没送达队友：{exc}', flush=True)
    time.sleep(HOVER_S)


def recon_return_and_land(sdk):
    pad = sdk.local_to_world(0.0, 0.0, 0.0)
    home = sdk.world_to_local(pad[0], pad[1], CRUISE_AGL_M)
    try:
        with sdk.fixed_altitude(home[2]):   # 转场段定高：不然规划器高频重规划会把轨迹高度压下去（见SDK fixed_altitude）
            sdk.goto(*home)         # 远距离回程走规划器，有避障
    except GotoUnreachableError:
        pass
    sdk.goto_direct(*home)          # 最后一段收准，落点精度高一个量级
    sdk.land()                      # 自动播"侦察机降落"


# ======================== 任务机（follower） ========================


class FireReport:
    """等侦察机发来的火情通报。回调在 SDK 的后台线程里跑，只存数据、不干活。"""

    def __init__(self):
        self.received = threading.Event()
        self.world_xy = None

    def on_event(self, x=None, y=None, **_ignored):
        if x is None or y is None:      # 字段不全的通报当没收到，继续等下一条
            return
        self.world_xy = (float(x), float(y))
        self.received.set()

    def wait(self, timeout):
        return self.received.wait(timeout)


def drive_servos(sdk, pwm, label):
    """抓取/投放机构：两个舵机同时动。仿真里飞控不一定配了舵机输出，
    动不了就打印出来、继续飞完流程，不让整个任务中断。"""
    try:
        sdk.set_servos({s: pwm for s in sorted(sdk.servos)})
    except (ActionFailedError, ValueError) as exc:
        print(f'[{sdk.namespace}] {label}：舵机没动（{exc}）——真机上需要飞控配好 MAIN7/MAIN9',
              flush=True)
        return
    time.sleep(SERVO_TRAVEL_S)


def fly_above(sdk, world_x, world_y, what):
    """飞到某个世界坐标的上方（走规划器，有避障）。"""
    print(f'[{sdk.namespace}] 飞往{what} ({world_x:.2f}, {world_y:.2f})', flush=True)
    leg = sdk.world_to_local(world_x, world_y, CRUISE_AGL_M)
    with sdk.fixed_altitude(leg[2]):    # 转场段定高。2026-09-25 漏了这一处，实测被
        sdk.goto(*leg)                  # 规划器一路压到 1.02 米，整段取弹任务因此丢掉


def aim_at(sdk, tag, what):
    """对准目标：先确认下视相机看得见，再让飞机挪到目标正上方。

    返回解算出的目标世界坐标（对不准就返回 None）。
    """
    try:
        sdk.wait_for_detection(tag, timeout=15.0, camera='down')
    except DetectionTimeoutError:
        print(f'[{sdk.namespace}] 下视相机没看到{what}（{tag}）', flush=True)
        return None
    try:
        sdk.center_on_target(tag, timeout=30.0)
    except ActionFailedError as exc:
        print(f'[{sdk.namespace}] 对准{what}没收敛（{exc}），按当前位置继续', flush=True)
    try:
        target = sdk.locate_target(tag, timeout=5.0, samples=10)
    except DetectionTimeoutError:
        return None
    wx, wy, _ = sdk.local_to_world(target.x, target.y, 0.0)
    print(f'[{sdk.namespace}] 已对准{what}，解算坐标 ({wx:.2f}, {wy:.2f})'
          f'（{target.samples}帧，离散 {target.spread_m:.2f}m）', flush=True)
    return wx, wy


def descend_onto(sdk, tag, what):
    """边瞄准边降落：每降一小段就把目标位置重新解算一次，直接命令"目标正上方、
    低一点"那个位置——水平修正和下降在同一条指令里完成；降到交接高度后交给
    普通降落收尾。

    为什么不用 precision_land_and_confirm()：那是"对准一点、下降一点、再对准"
    的分级下降，从 2.5 米下来要 40 秒以上，30 秒的时限内根本走不完，每次都会
    走超时兜底（2026-09-23 实测），等于精度只做了一半。这里换成连续修正，同一
    时间既在对准也在下降，30 秒够用；最后 0.7 米交给 land()，那一段本来就只能
    垂直下降，再修也无意义。
    """
    deadline = time.monotonic() + PRECISION_LAND_S
    while time.monotonic() < deadline:
        _, _, z = sdk.get_local_position()
        try:
            t = sdk.locate_target(tag, timeout=2.0, samples=3)
        except DetectionTimeoutError:
            print(f'[{sdk.namespace}] 下降中看不到{what}了，就地转普通降落', flush=True)
            break
        agl = z - t.z                       # t.z 是解算出的地面高度
        if agl <= HANDOFF_AGL_M + HANDOFF_TOL_M:
            print(f'[{sdk.namespace}] 已降到离地 {agl:.2f} m，交给普通降落', flush=True)
            break
        next_agl = max(HANDOFF_AGL_M, agl - DESCENT_STEP_M)
        print(f'[{sdk.namespace}] 对准{what} ({t.x:.2f}, {t.y:.2f}) 并降到离地 '
              f'{next_agl:.2f} m（当前 {agl:.2f} m，{t.samples}帧离散 {t.spread_m:.2f}m）',
              flush=True)
        sdk.goto_direct(t.x, t.y, t.z + next_agl)
    else:
        print(f'[{sdk.namespace}] 边瞄准边降落用满 {PRECISION_LAND_S:.0f} 秒，转普通降落',
              flush=True)
    sdk.land()                              # 最后一段普通降落


def pick_up_supply(sdk):
    """飞到物资点，对准灭火弹，精准降落并抓取，再起飞。"""
    fly_above(sdk, SUPPLY_POINT[0], SUPPLY_POINT[1], '物资点')
    if aim_at(sdk, SUPPLY_TAG, '灭火弹') is not None:
        sdk.play_sound_light('任务机发现灭火弹')

    descend_onto(sdk, SUPPLY_TAG, '灭火弹')
    print(f'[{sdk.namespace}] 已降落在物资点，开始抓取', flush=True)

    sdk.play_sound_light('任务机抓取灭火弹')
    drive_servos(sdk, GRAB_PWM, '抓取')
    sdk.takeoff()               # 自动播"任务机起飞"


def drop_on_fire(sdk, fire_world_xy):
    """飞到火点，对准后投放灭火弹。"""
    fly_above(sdk, fire_world_xy[0], fire_world_xy[1], '地面火情')
    solved = aim_at(sdk, GROUND_FIRE, '地面火情')
    if solved is not None:
        dx = solved[0] - fire_world_xy[0]
        dy = solved[1] - fire_world_xy[1]
        print(f'[{sdk.namespace}] 自己解算的火点与侦察机通报的相差 '
              f'({dx:+.2f}, {dy:+.2f}) 米', flush=True)

    sdk.play_sound_light('任务机投放灭火弹')
    drive_servos(sdk, DROP_PWM, '投放')
    time.sleep(DROP_HOLD_S)
    print(f'[{sdk.namespace}] 灭火弹已投放', flush=True)


def supply_return_and_land(sdk):
    """返回自己的起飞点降落（起飞点就是局部系原点）。

    回程这一段的不可达要吃掉：规划器经常把飞机停在离起飞点半米左右就不再推进
    （超过 goto() 的 0.3 米到点阈值，于是被判不可达），但那时其实已经到家门口了，
    后面的 goto_direct 足够收准。2026-09-24 实测过一次没吃这个异常，整段任务在
    最后一步抛异常退出、飞机留在空中。
    """
    pad_x, pad_y, _ = sdk.local_to_world(0.0, 0.0, 0.0)
    try:
        fly_above(sdk, pad_x, pad_y, '起飞点')
    except GotoUnreachableError as exc:
        print(f'[{sdk.namespace}] 回程判不可达（{exc}），用直飞收尾', flush=True)
    sdk.goto_direct(0.0, 0.0, CRUISE_AGL_M)     # 最后一段收准再落
    sdk.land()                                  # 自动播"任务机降落"
    sdk.play_sound_light('任务机已降落')
    print(f'[{sdk.namespace}] 已返回起飞点降落', flush=True)


def recon_search_and_report(sdk):
    """侦察机的地面火情任务段：搜索 -> 悬停解算 -> 通报任务机。返回是否找到。

    **不起飞、不返航**——留给调用方决定，这样《双机灭火任务示例.py》可以把
    这一段跟高楼火情那一段串起来，中间不落地。
    """
    watcher = FireWatcher(sdk)
    try:
        watcher.start()
        found = search(sdk, watcher)
        watcher.stop()
        watcher.join()          # 等它的 cancel_goto 发完，见 join() 的说明
        if found:
            sdk.cancel_goto()
            hover_over_fire(sdk, watcher)
        else:
            print(f'[{sdk.namespace}] 整个区域扫完，没有发现地面火情', flush=True)
        return found
    finally:
        watcher.stop()


def run_recon(sdk):
    """侦察机：起飞 -> 搜索通报 -> 返航降落（单独跑这个示例时的完整流程）。"""
    sdk.takeoff()                   # 自动播"侦察机起飞"
    found = recon_search_and_report(sdk)
    recon_return_and_land(sdk)
    if found:
        sdk.play_sound_light('侦察机任务完成')


def listen_for_report(sdk):
    """提前注册"地面火情通报"的处理函数，返回接收器。

    ⚠️ 必须在侦察机可能发出通报之前就注册：可靠事件通道收到事件是**先回 ACK
    再查处理函数**的（reliability.py::_handle_event），没注册就等于"确认收到
    然后丢掉"，发送方还以为送达了。
    """
    report = FireReport()
    sdk.on_teammate_event(FIRE_REPORT_EVENT, report.on_event)
    return report


def run_supply(sdk, teammate, report=None):
    """任务机：等火情通报，取灭火弹投到火点，返回起飞点。

    `report` 可以传一个提前注册好的接收器（见 listen_for_report()）；不传就
    在这里注册，适合单独跑这个示例。
    """
    if report is None:
        report = listen_for_report(sdk)
    print(f'[{sdk.namespace}] 等 {teammate} 通报地面火情…', flush=True)
    if not report.wait(WAIT_REPORT_S):
        print(f'[{sdk.namespace}] {WAIT_REPORT_S:.0f} 秒内没收到火情通报，不起飞', flush=True)
        return
    fire_world_xy = report.world_xy
    print(f'[{sdk.namespace}] 收到火情通报：火点世界坐标 '
          f'({fire_world_xy[0]:.2f}, {fire_world_xy[1]:.2f})', flush=True)
    sdk.play_sound_light('任务机收到地面火情')

    sdk.takeoff()                   # 自动播"任务机起飞"
    try:
        pick_up_supply(sdk)
        drop_on_fire(sdk, fire_world_xy)
    except GotoUnreachableError as exc:
        # 目标点被判定不可达（落在障碍物里等），不硬飞，直接返航
        print(f'[{sdk.namespace}] 航点不可达：{exc}，提前返航', flush=True)
    supply_return_and_land(sdk)


def main():
    ap = argparse.ArgumentParser(description='双机地面灭火（按 --role 分工）')
    ap.add_argument('--namespace', default='NX01')
    ap.add_argument('--role', default='leader', choices=['leader', 'follower'])
    ap.add_argument('--teammate', default='NX02')
    args = ap.parse_args()

    is_recon = args.role == 'leader'
    sdk = DroneSDK(namespace=args.namespace,
                   role='recon' if is_recon else 'supply',
                   teammate_namespace=args.teammate)
    try:
        if is_recon:
            run_recon(sdk)
        else:
            run_supply(sdk, args.teammate)
        print(f'[{sdk.namespace}] 结束', flush=True)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()
