#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双机高楼火情：侦察机逐栋绕飞找着火点，找到就破窗并通知任务机接着灭火。

    bash 运行仿真.sh --task 高楼火情示例.py

一份代码两架飞机各起一个容器，按 --role 分工（运行脚本会传）：
  leader（NX01，侦察机）：起飞 -> 逐栋高楼绕飞侦察（机头始终对准楼、前视相机
      找着火点）-> 找到就悬停对准、通知任务机、发射破窗弹 -> 返航降落
  follower（NX02，任务机）：等通知 -> 起飞 -> 飞到侦察机指定的瞄准位置
      -> 对准 -> 发射灭火弹 -> 返航降落

绕飞逻辑：对每栋楼按半径生成一圈航点，从**离飞机最近**的那个点开始绕（省掉
一段无谓的横穿）；绕的过程中 yaw 用 POINT 模式锁定楼的坐标，机头自然始终对着
楼，前视相机一直朝着楼面。一圈绕完没看到着火点就换下一栋，全绕完还没有就返航。

任务机直接飞到侦察机给的瞄准位置，两机撞不上靠飞控栈的机间回避。

高楼位置写在 BUILDINGS 里，几栋都行——仿真里是 3 根立柱，换场地只改这个列表。
"""
import argparse
import math
import threading
import time

from contest_sdk import DroneSDK
from contest_sdk.exceptions import (
    ActionFailedError,
    DetectionTimeoutError,
    GotoUnreachableError,
    TeammateUnreachableError,
)
from contest_sdk.geometry_helpers import angle_from_center_to_point

# ---- 场地 ----
# 代表高楼的立柱世界坐标，按这个顺序逐栋侦察。不局限于 3 栋，加几个点就多几栋。
BUILDINGS = [(4.0, 4.0), (0.0, 8.0), (-5.0, 2.0)]
HIGH_FIRE = 'apriltag:1'     # 高层着火点标识（贴在某一栋的某个立面上）

# ---- 绕飞 ----
# 半径要同时满足：比 立柱半对角线0.42+规划器膨胀0.6 大（不然规划器认为进不去），
# 又不能太远——前视相机看 0.5 米的标志，太远了像素太小认不出来。
ORBIT_RADIUS_M = 3.0         # 2.5 米时实测规划器会贴着立柱膨胀区反复重规划、
                             # 走不动还掉高（整栋楼被跳过），放到 3 米余量够用，
                             # 标志在 3 米处还有约 63 像素，检测没问题
ORBIT_AGL_M = 2.5            # 绕飞高度；着火点贴在 3 米高处，前视相机仰视能看到
ORBIT_POINTS = 12            # 一圈几个点：12 个（每 30°）足够密，多了徒增航点等待

# ---- 前视相机（用于把着火点转到画面正中）----
IMAGE_W = 640
FOCAL_PX = 381.35            # = (IMAGE_W/2)/tan(HFOV/2)，HFOV=80°，跟 camera_info 一致
AIM_TOLERANCE_RAD = math.radians(8.0)   # 画面偏差小于这个角度就算对准
AIM_MAX_TRIES = 2            # "改朝向 + 飞一下让它生效 + 复查"最多来几轮
YAW_SETTLE_S = 4.0           # 机头转到位要几秒；没转到位时画面里根本没有楼
BUILDING_HALF_M = 0.42       # 立柱半对角线，用来把"到楼心的距离"折成"到立面的距离"

# ⚠️ 改朝向用 sdk.face_point()，不要用 set_yaw_mode_point() 之后干等：
# 后者只是把目标发给 traj_server，悬停时机头是锁死的（pt4ctrl 的 AUTO_HOVER
# 把 yaw 冻在进入悬停那一刻）。face_point() 对应飞行栈里的"原地转向"，会一直
# 发指令直到转到位。另外 goto_direct() 不经过 traj_server，用它挪位置时机头
# 不会转——绕飞圈上挪位置要用 goto()。
# 发现火点后在绕飞圈上往两边各试几个点，挑标志在画面里最大的那个位置停：
# 那里最正对立面。发现的那一刻往往是"刚从侧面瞥见"，入射角能有 70 度以上，
# AprilTag 在这种斜角下时断时续（实测：停在发现点对准时目标就丢了）。
REFINE_STEPS = 2             # 往两边各试几个绕飞点（每个点 360/ORBIT_POINTS 度）
ENTRY_CANDIDATES = (0, 3, 6, 9)   # 进圈备选点：最近的进不去就换 1/4 圈外的点试
REFINE_LOOK_S = 2.5          # 每个点上等多久看有没有目标

# ---- 发射机构（侦察机发破窗弹、任务机发灭火弹，动作一样）----
FIRE_PWM = 2000              # 松开=发射——2026-09-21 NX02 真机实测的两个位置
LOAD_PWM = 800               # 抓紧=装填/复位
SERVO_TRAVEL_S = 2.0

FIRE_EVENT = '高层火情通报'   # 侦察机 -> 任务机：瞄准位置 + 楼坐标
WAIT_NOTIFY_S = 900.0        # 任务机等通知


class TagWatcher:
    """后台线程：盯着某一路相机，看到目标就记下来并打断当前航段。

    必须后台盯——一个航点要飞好几秒，等 goto() 返回再查，早就飞过去了。
    """

    def __init__(self, sdk, tag, camera):
        self.sdk = sdk
        self.tag = tag
        self.camera = camera
        self.found = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            try:
                self.sdk.wait_for_detection(self.tag, timeout=5.0, camera=self.camera)
            except DetectionTimeoutError:
                continue
            self.found.set()
            self.sdk.play_sound_light('侦察机发现高楼火情')
            # 连打断两次：主线程可能正好在两个航点之间，第一次打断落空
            for _ in range(2):
                self.sdk.cancel_goto()
                time.sleep(0.5)
            return


def fire_launcher(sdk, label):
    """发射：把发射机构的舵机推到松开位置，等它到位，再复位装填。
    没配舵机的飞机（见 sdk.servos）只打印提示，不让流程中断。"""
    if not sdk.servos:
        print(f'[{sdk.namespace}] {label}：这架飞机没有配置舵机，跳过（见 SDK 的 SERVO_CONFIG）',
              flush=True)
        return
    try:
        sdk.set_servos({s: FIRE_PWM for s in sorted(sdk.servos)})
        time.sleep(SERVO_TRAVEL_S)
        sdk.set_servos({s: LOAD_PWM for s in sorted(sdk.servos)})
        time.sleep(SERVO_TRAVEL_S)
    except (ActionFailedError, ValueError) as exc:
        print(f'[{sdk.namespace}] {label}：舵机没动（{exc}）', flush=True)


def pixel_offset_rad(det):
    """标志中心相对画面中心的水平角（右正）。画面右 = 机体右 = yaw 要减小
    （FLU 里 yaw 逆时针为正）。"""
    return math.atan2(det.bbox_x - IMAGE_W / 2.0, FOCAL_PX)


def estimate_fire_xy(sdk, det, building_local):
    """由"飞机在哪、机头朝哪、标志在画面里偏多少"反算着火点的水平坐标
    （飞机自己的局部系）。距离用"到楼心的距离减去立柱半对角线"——标志贴在
    立面上，这个近似的误差就是立柱这点尺寸，对朝向来说足够。"""
    x, y, _ = sdk.get_local_position()
    bearing = sdk.get_current_yaw() - pixel_offset_rad(det)
    d = max(math.hypot(building_local[0] - x, building_local[1] - y) - BUILDING_HALF_M, 0.5)
    return x + d * math.cos(bearing), y + d * math.sin(bearing)


def aim_at_fire(sdk, fire_local):
    """把机头对准着火点本身（不是楼心），然后复查画面偏差。

    返回是否对上了——没对上就是朝着大概方向空放，必须打出来。
    """
    for _ in range(AIM_MAX_TRIES):
        sdk.face_point(*fire_local)     # 原地转向，转到位才返回
        try:
            det = sdk.wait_for_detection(HIGH_FIRE, timeout=5.0, camera='front')
        except DetectionTimeoutError:
            print(f'[{sdk.namespace}] 对准后前视相机看不到着火点', flush=True)
            continue
        delta = pixel_offset_rad(det)
        print(f'[{sdk.namespace}] 着火点在画面 x={det.bbox_x:.0f}（偏差 '
              f'{math.degrees(delta):+.1f}°，宽 {det.bbox_width:.0f} 像素）', flush=True)
        if abs(delta) <= AIM_TOLERANCE_RAD:
            print(f'[{sdk.namespace}] 已对准着火点', flush=True)
            return True
        # 还差得多说明火点坐标估得不准，用这一帧重估一次再来
        fire_local = estimate_fire_xy(sdk, det, fire_local)
        time.sleep(YAW_SETTLE_S)
    print(f'[{sdk.namespace}] 没能对准着火点，按当前朝向继续', flush=True)
    return False


def orbit_building(sdk, watcher, world_xy):
    """绕一栋楼一圈，边绕边用前视相机找着火点。

    返回 (是否找到, 整圈航点, 发现时正飞向的那个航点下标)——后两个给
    refine_aim() 用：从发现的位置附近挑一个正对立面的点。

    航点直接在飞机自己的局部系里生成：整圈是同一个圆，没必要一个点一个点
    做世界系换算。
    """
    bx, by = world_xy
    lbx, lby, lz = sdk.world_to_local(bx, by, ORBIT_AGL_M)
    x, y, _ = sdk.get_local_position()
    start = angle_from_center_to_point(lbx, lby, x, y)   # 从离飞机最近的点起绕
    ring = sdk.generate_orbit_waypoints(
        lbx, lby, ORBIT_RADIUS_M, lz, num_points=ORBIT_POINTS, start_angle_rad=start)

    # 进圈：最近的那个点进不去就换几个点试，别因为一个点就放弃整栋楼
    # （实测过：飞往某栋楼的绕飞起点时规划器卡住，那栋楼直接被跳过，而火点
    # 就在那栋）。
    print(f'[{sdk.namespace}] 开始侦察高楼 ({bx:.1f}, {by:.1f})：先飞绕飞起点', flush=True)
    entry = None
    for k in ENTRY_CANDIDATES:
        j = k % len(ring)
        try:
            sdk.goto(*ring[j])      # 到起点这一段走规划器，有避障
        except GotoUnreachableError:
            print(f'[{sdk.namespace}] 绕飞起点 {j} 进不去，换一个', flush=True)
            continue
        entry = j
        break
    if watcher.found.is_set():
        return True, ring, entry or 0
    if entry is None:
        print(f'[{sdk.namespace}] 这栋楼的绕飞点都进不去，跳过', flush=True)
        return False, ring, 0

    # 机头锁定楼的坐标：绕圈过程中机头始终对着楼，前视相机一直扫楼面
    sdk.set_yaw_mode_point(lbx, lby)
    # 绕满一周 = 从进圈那个点走完其余点再回到它
    for step in range(1, len(ring) + 1):
        idx = (entry + step) % len(ring)
        if watcher.found.is_set():
            return True, ring, idx
        print(f'[{sdk.namespace}] 绕飞点 {step + 1}/{len(ring) + 1}', flush=True)
        try:
            sdk.goto(*ring[idx])
        except GotoUnreachableError:
            continue                # 个别点被别的障碍物占了就跳过，继续绕
        if watcher.found.is_set():
            return True, ring, idx
    return False, ring, 0


def refine_aim(sdk, ring, idx, building_local):
    """在发现点两侧的绕飞点上各看一眼，挑标志显得最大的那个点（最正对
    立面），停在那个点上，返回在那里估出的火点坐标；都看不到返回 None。

    绕飞点之间用 goto()：同一个圆上相邻两点的连线离圆心最近也有
    R*cos(180/N) 米（12 点、半径 2.5 时是 2.41 米），比立柱半对角线加膨胀
    还宽，不需要规划器绕。
    """
    best = None                     # (标志宽度像素, 航点, 估出的火点坐标)
    sdk.set_yaw_mode_point(*building_local)     # 挪位置时机头继续对着楼
    for k in range(-REFINE_STEPS, REFINE_STEPS + 1):
        wp = ring[(idx + k) % len(ring)]
        try:
            sdk.goto(*wp)           # 必须用 goto：goto_direct 不走 traj_server，机头不会转
        except GotoUnreachableError:
            continue
        try:
            det = sdk.wait_for_detection(HIGH_FIRE, timeout=REFINE_LOOK_S, camera='front')
        except DetectionTimeoutError:
            continue                # 这个角度看不到，换下一个
        fire_local = estimate_fire_xy(sdk, det, building_local)
        print(f'[{sdk.namespace}] 候选瞄准点 {k:+d}：标志宽 {det.bbox_width:.0f} 像素，'
              f'火点估计 ({fire_local[0]:.2f}, {fire_local[1]:.2f})', flush=True)
        if best is None or det.bbox_width > best[0]:
            best = (det.bbox_width, wp, fire_local)
    if best is None:
        print(f'[{sdk.namespace}] 发现点附近都看不清标志', flush=True)
        return None
    sdk.goto(*best[1])              # 回到最正对立面的那个点
    return best[2]


def recon_return_and_land(sdk):
    pad = sdk.local_to_world(0.0, 0.0, 0.0)
    home = sdk.world_to_local(pad[0], pad[1], ORBIT_AGL_M)
    sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw or sdk.get_current_yaw())
    try:
        sdk.goto(*home)             # 远距离回程走规划器，有避障
    except GotoUnreachableError:
        pass
    sdk.goto_direct(*home)          # 最后一段收准
    sdk.land()                      # 自动播"侦察机降落"


def recon_orbit_and_fire(sdk):
    """侦察机的高楼火情任务段：逐栋绕飞 -> 对准 -> 通报任务机 -> 发射破窗弹。
    返回是否找到着火点。

    **不起飞、不返航**——留给调用方决定，这样《双机灭火任务示例.py》可以把
    这一段接在地面火情之后，中间不落地。
    """
    watcher = TagWatcher(sdk, HIGH_FIRE, camera='front')
    try:
        sdk.play_sound_light('侦察机排查高层火情')
        watcher.start()

        hit = ring = None
        hit_idx = 0
        for world_xy in BUILDINGS:
            found, ring, hit_idx = orbit_building(sdk, watcher, world_xy)
            if found:
                hit = world_xy
                break
        watcher.stop()

        if hit is None:
            print(f'[{sdk.namespace}] {len(BUILDINGS)} 栋楼都绕完了，没有发现高层火情', flush=True)
            return False

        sdk.cancel_goto()
        lbx, lby, _ = sdk.world_to_local(hit[0], hit[1], ORBIT_AGL_M)
        # 先在发现点附近挑一个正对立面的位置，再把机头对准火点本身
        fire_local = refine_aim(sdk, ring, hit_idx, (lbx, lby))
        if fire_local is None:
            fire_local = (lbx, lby)     # 退而求其次：对着楼心
        aim_at_fire(sdk, fire_local)
        hx, hy, hz = sdk.local_to_world(*sdk.get_local_position())
        fx, fy, _ = sdk.local_to_world(fire_local[0], fire_local[1], 0.0)
        print(f'[{sdk.namespace}] 在高楼 ({hit[0]:.1f}, {hit[1]:.1f}) 发现高层火情：'
              f'着火点约 ({fx:.2f}, {fy:.2f})，瞄准位置 ({hx:.2f}, {hy:.2f}, {hz:.2f})',
              flush=True)

        # 通知任务机：瞄准位置 + 着火点坐标（任务机进场前就把朝向设成着火点，
        # 这样飞过去的过程中机头已经转好了）
        sdk.play_sound_light('侦察机通报高层火情')
        try:
            sdk.send_to_teammate(FIRE_EVENT, x=hx, y=hy, z=hz, fire_x=fx, fire_y=fy)
        except TeammateUnreachableError as exc:
            print(f'[{sdk.namespace}] 高层火情通报没送达队友：{exc}', flush=True)

        sdk.play_sound_light('侦察机发射破窗弹')
        fire_launcher(sdk, '发射破窗弹')
        sdk.play_sound_light('侦察机破窗完成')
        return True
    finally:
        watcher.stop()


def run_recon(sdk):
    """侦察机：起飞 -> 绕飞侦察破窗 -> 返航降落（单独跑这个示例时的完整流程）。"""
    sdk.takeoff()                   # 自动播"侦察机起飞"
    found = recon_orbit_and_fire(sdk)
    recon_return_and_land(sdk)
    if found:
        sdk.play_sound_light('侦察机任务完成')


class Notice:
    """等队友的一条事件；回调在 SDK 后台线程里跑，只存数据。"""

    def __init__(self):
        self.received = threading.Event()
        self.data = {}

    def on_event(self, **kwargs):
        self.data = kwargs
        self.received.set()

    def wait(self, timeout):
        return self.received.wait(timeout)


def listen_for_report(sdk):
    """提前注册"高层火情通报"的处理函数，返回接收器。

    ⚠️ 必须在侦察机可能发出通报之前就注册：可靠事件通道收到事件是**先回 ACK
    再查处理函数**的（reliability.py::_handle_event），没注册就等于"确认收到
    然后丢掉"，发送方还以为送达了。串行做多个任务时，后面那个任务的通报很
    可能在前一个任务还没做完时就到了——实测就是这么丢的一条。
    """
    notice = Notice()
    sdk.on_teammate_event(FIRE_EVENT, notice.on_event)
    return notice


def run_supply(sdk, teammate, notice=None):
    """任务机：等通知，飞到侦察机给的瞄准位置灭火，再返航。

    `notice` 可以传一个提前注册好的接收器（见 listen_for_report()）；不传就
    在这里注册，适合单独跑这个示例。
    """
    if notice is None:
        notice = listen_for_report(sdk)

    print(f'[{sdk.namespace}] 等 {teammate} 通报高层火情…', flush=True)
    if not notice.wait(WAIT_NOTIFY_S):
        print(f'[{sdk.namespace}] {WAIT_NOTIFY_S:.0f} 秒内没收到高层火情通报，不起飞', flush=True)
        return
    aim = (float(notice.data['x']), float(notice.data['y']), float(notice.data['z']))
    fire = (float(notice.data['fire_x']), float(notice.data['fire_y']))
    print(f'[{sdk.namespace}] 收到高层火情通报：瞄准位置 ({aim[0]:.2f}, {aim[1]:.2f}, {aim[2]:.2f})，'
          f'着火点 ({fire[0]:.2f}, {fire[1]:.2f})', flush=True)

    sdk.takeoff()                   # 自动播"任务机起飞"
    try:
        # 直接飞到瞄准位置：侦察机可能还在附近，靠飞控栈的机间回避。
        # 出发前先把朝向目标设成着火点，飞过去的路上机头就转好了。
        fire_local = sdk.world_to_local(fire[0], fire[1], aim[2])[:2]
        sdk.set_yaw_mode_point(*fire_local)     # 飞过去的路上机头就转好
        print(f'[{sdk.namespace}] 飞往瞄准位置 ({aim[0]:.2f}, {aim[1]:.2f}, {aim[2]:.2f})', flush=True)
        sdk.goto(*sdk.world_to_local(*aim))
        sdk.play_sound_light('任务机到达瞄准点')
        aim_at_fire(sdk, fire_local)

        sdk.play_sound_light('任务机发射灭火弹')
        fire_launcher(sdk, '发射灭火弹')
    except GotoUnreachableError as exc:
        print(f'[{sdk.namespace}] 航点不可达：{exc}，提前返航', flush=True)

    pad = sdk.local_to_world(0.0, 0.0, 0.0)
    sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw or sdk.get_current_yaw())
    try:
        sdk.goto(*sdk.world_to_local(pad[0], pad[1], aim[2]))
    except GotoUnreachableError:
        pass
    sdk.precision_land_at(0.0, 0.0, timeout=90.0)   # 最后一段收准，不走规划器
    sdk.play_sound_light('任务机已降落')


def main():
    ap = argparse.ArgumentParser(description='双机高楼火情（按 --role 分工）')
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
