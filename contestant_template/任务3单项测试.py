#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务3 单项测试：巡检拍摄 -> 协同灭火 -> 编队返回。

任务3 分三个过程：**巡检拍摄 -> 协同灭火 -> 编队返回**（2026-09-30 用户定义）。

① 巡检拍摄（侦察机）
   三栋楼**每栋都必须拍一张照片并回传**，按楼号 3 -> 2 -> 1 的顺序：
       3#  在 M(6,16) 朝 -90°（正南）拍      —— 只拍照
       2#  在 M(6,16) 朝 +90°（正北）拍      —— 拍照 + 开高楼火情识别
       1#  在 N(14,16) 朝 +90°（正北）拍     —— 拍照 + 开高楼火情识别
   面朝 2#/1# 时识别到火情就当场对准、插入一轮协同灭火；灭完继续巡检下一栋。
   **有火情的那栋楼，照片等对准之后再拍**（用户 2026-09-30）——那张照片要能
   看清着火点，火情落在画面边上等于没拍到；没火情的楼转到位就拍。
   三栋走完即"巡检完毕"，进入编队返回。
   （楼的坐标来自 sample_room_layout.yaml：3#(5.5,12.5) 2#(5.5,19.5) 1#(13.5,19.5)，
     M/N 是**观察位**不是楼本身，各在对应楼南侧 3.5 m。火情高度 1.5 或 2.5 m。）

② 协同灭火（只可能发生在 2#、1# 两栋楼前）
   侦察机 NX01                          任务机 NX02
   ───────────────────────────────      ──────────────────────────────
   识别到火情 -> 对准 -> 前移 1.5 m
   通报火情 + 播报，**原地等待** ─────►  起飞 -> 飞 E 点待命
   等任务机到 E ◄───────────────────     到位通报
   发射破窗弹 + 播报 ───────────────►   收到"已破窗"
   有下一个巡检点就去下一个，            飞到侦察机位置 -> 发射 4 发灭火弹
   没有就去 G 等灭火完成                 发射完毕通报 ────────────┐
                                        侦察机已巡检完毕 -> 就近入列│
                                        没完 -> 等它完 -> 就近入列 ┘

   火情在 2# 时：侦察机灭火协同完就去 N 拍 1#，拍完直接去 G；
   火情在 1# 时：1# 是最后一站，协同完直接去 G。两种情况任务机都是灭完火就近入列。

③ 编队返回
   从 G 开始编队 G -> D -> A，**任务机过 D 点就解散**（不是默认的"长机回自己
   起飞点上空"）。解散后：
       NX01 -> 飞回 A 点待命（悬停，不降落）
       NX02 -> 先飞物资点降落、松开机械抓释放灭火器材 -> 起飞 -> 回自己起降点降落

物资点 (10,8) 的两次动作：任务机**起飞后**先去抓取灭火器材再去待命点参与灭火；
**编队解散后**再去释放器材。都是"飞过去 -> descend_onto 边瞄边降到底 -> 驱动
机械抓 -> 起飞"。

两栋楼都可能有火情，所以灭火周期两边都写成**可重入**的：事件信箱收完复位，
任务机每轮结束后等"下一次火情通报"或"侦察机巡检完毕"，先到哪个走哪个。

拍照走 SDK 的 `capture_photo()`（2026-09-30 新增），存进 /logs（挂给地面站的
目录）即视为回传。
"""
import argparse
import math
import time

import 编队飞行示例 as 编队
import 地面火情搜索示例 as 地面
import 高楼火情绕飞版示例 as 高楼
from contest_sdk import DroneSDK
from contest_sdk.exceptions import GotoUnreachableError

CRUISE_AGL_M = 编队.CRUISE_AGL_M
SPACING_M = 4.0

ROUTE_A, ROUTE_B = (3.0, 3.0), (3.0, 22.0)
ROUTE_C, ROUTE_G = (17.0, 22.0), (17.0, 16.0)
ROUTE_E, ROUTE_D = (10.0, 16.0), (17.0, 3.0)
POINT_M = (6.0, 16.0)                 # 观察位，正北 3.5 m 是 2# 楼
POINT_N = (14.0, 16.0)                # 观察位，正北 3.5 m 是 1# 楼

HIGH_FIRE = 'apriltag:1'              # 高层着火点标识
OBSERVE_AGL_M = 2.0                   # 用户指定：在 M 点悬停于 2 米高度
# 2026-09-30 用户明确了 M 点的动作顺序：先给 3# 楼拍照回传，再原地转 180°
# 对准 2# 楼查火情并拍照；灭完火之后才去 N 点给 1# 楼拍照，然后编队返航。
# 三栋楼相对观察位的方位（楼的坐标来自 sample_room_layout.yaml）：
#   3# (5.5,12.5) 在 M(6,16) 的**正南**  -> 机头 -90°
#   2# (5.5,19.5) 在 M(6,16) 的**正北**  -> 机头 +90°（从 3# 原地转 180° 过来）
#   1# (13.5,19.5) 在 N(14,16) 的**正北** -> 机头 +90°
# 巡检站点表：(楼号, 观察位, 观察位名字, 机头朝向°, 这栋楼要不要查火情)
# 按楼号 3 -> 2 -> 1 的顺序走。3# 只拍照不查火情——火情只可能在 1#/2# 两栋。
INSPECT_STATIONS = (
    ('3#', POINT_M, 'M', -90.0, False),
    ('2#', POINT_M, 'M',  90.0, True),
    ('1#', POINT_N, 'N',  90.0, True),
)
PHOTO_DIR = '/logs/任务3照片'          # 拍照存这儿（/logs 是挂给地面站的目录）
# 火情只可能在这两个高度（二楼/三楼层高正中）。2026-09-30 起**不再**用它做
# 升降扫描——两个高度从 2.0 m 观察位上本来就都在前视画面里，见 _inspect_here。
# 留着是因为它说明了随机化的取值范围，也跟 layout 的 fire_apriltag_random 对应。
FIRE_HEIGHTS_M = (1.5, 2.5)
# 对准后沿机头前移这么多再发射。实测最近点落在离火情点/楼面 1.5~1.7 米
# （观察位离 tag 约 3.0 米，前移 1.5 米后剩约 1.5 米）。
# ⚠️ **这个距离是发射弹丸的要求，不是可调的安全裕度**（2026-09-30 用户确认）。
# 真机上旋翼离楼面只有一米出头、有壁面效应，看着危险，但不能为此调小——
# 调小了弹丸打不进窗。要改只能连同发射机构的性能一起重新定。
FORWARD_BEFORE_FIRE_M = 1.5
EXTINGUISHER_SHOTS = 4                # 任务机连发几发灭火弹
SHOT_INTERVAL_S = 1.0

DETECT_TIMEOUT_S = 3.0                # 等一次检测多久。检测节点约 7 Hz，标志在
                                      # 画面里的话一两帧就出结果；等 6 秒纯属在
                                      # 没火情的楼前白耗（2026-09-30 从 6.0 降下来）
DETECT_TRIES = 2                      # 试几次（转向刚到位时画面可能还没稳）
DETECT_SETTLE_S = 1.0                 # 两次之间缓一下
AT_E_WAIT_S = 300.0
BREACH_WAIT_S = 300.0
EXTINGUISH_WAIT_S = 420.0
# 侦察机离发射点多远算"让开了"。这个门限只是**提前放行**用的——真正保证放行的
# 是"侦察机已经飞到下一个落脚点"那一路（见 _start_spot_clear_watch 的 notify）。
# 2026-09-30 从 3.0 降到 2.0：火情随机到 1# 东单元时发射点 (14.40,17.33)，而
# 侦察机的下一个落脚点是 G(17,16)，两点只差 2.92 m，3.0 的门限永远满足不了，
# watchdog 耗满 120 秒超时才放行，任务机白等两分钟。
SPOT_CLEAR_M = 2.0
SPOT_CLEAR_WAIT_S = 120.0
SPOT_POLL_S = 0.2
# 任务机飞到发射点时的容忍：规划器把终点推到边缘、停在这个距离以内就认了，
# 反正接下来还要把火情居中到前视画面，差一两米不影响发射。
ARRIVE_ACCEPT_M = 2.0
# ---- 物资点取放（2026-09-30 用户要求）----
# 任务机起飞后先到物资点降落抓取灭火器材，再去待命点参与灭火；编队解散后再回
# 物资点降落、松开机械抓，模拟释放器材，然后才回自己起降点降落。
SUPPLY_XY = (10.0, 8.0)               # 物资点，AprilTag ID0
SUPPLY_TAG = 'apriltag:0'
GRAB_PWM = 800                        # 抓紧（真机实测值，见 project_nx02_servos 记录）
RELEASE_PWM = 2000                    # 松开
SERVO_TRAVEL_S = 2.0                  # 舵机没有位置反馈，只能等
HOVER_AFTER_TAKEOFF_S = 2.0
# ---- 前视居中对准（2026-09-30 用户要求"火情点要在前视相机中居中"）----
# **不能用 sdk.center_on_target()**：那条路走 precision_servo_node，而它
# 第 536 行明写着 `if '_camera_down_' not in msg.header.frame_id: return`
# ——只认**下视**相机的检测，是给地面标靶精准降落用的。高层火情贴在楼立面上、
# 走前视相机，所以它永远收敛不了，实测一直是 40 秒超时后走兜底。
# 改成自己按画面偏差转机头，跟《高楼火情绕飞版示例》的 aim_at_fire 同一套算法。
IMAGE_W = 640
IMAGE_H = 480
FOCAL_PX = 381.35             # = (IMAGE_W/2)/tan(HFOV/2)，HFOV=80°，跟 camera_info 一致
TAG_SIZE_M = 0.5              # 火情标志实际边长，用来按框宽估距离
AIM_TOL_XY_M = 0.12           # 横向（平行楼面）差这么多以内算对准
AIM_TOL_Z_M = 0.12            # 垂直方向差这么多以内算对准
AIM_MAX_TRIES = 5
AIM_SETTLE_S = 1.5            # 转完等画面稳定
# 垂直为什么要靠改高度而不是俯仰相机：前视/下视是两个**独立的固定安装**相机
# （模型里两个 camera_joint 都是 type='fixed'），装死了，没有俯仰自由度。
# 火情随机化之后可能在二楼(1.5m)或三楼(2.5m)，而飞机巡航在 2.0 m——差 0.5 m、
# 3 米距离上约 9.5°，检测得到（垂直视场约 64°）但画面里不居中，发射也偏。
# 所以按框宽估出距离，再把这个垂直角换算成高度差，飞机自己升降过去。
AIM_Z_MIN_M = 0.8             # 再低就贴地了，别往下钻
AIM_Z_MAX_M = 4.0

# 跨机事件
EV_FIRE = 'high_fire_found'           # NX01 -> NX02：火情位置 + 侦察机发射点
EV_AT_E = 'supply_at_standby'         # NX02 -> NX01：我到 E 点待命了
EV_BREACHED = 'breach_done'           # NX01 -> NX02：破窗完成
EV_EXTINGUISHED = 'extinguish_done'   # NX02 -> NX01：灭火弹发射完毕
EV_INSPECT_DONE = 'inspect_done'      # NX01 -> NX02：三栋楼都巡检拍照完了
EV_SPOT_CLEAR = 'spot_clear'          # NX01 -> NX02：我已离开发射点，位置让给你了
EV_LANDED_HOME = 'supply_landed_home' # NX02 -> NX01：我已降落在**自己的起降点**上
LANDED_HOME_TOL_M = 1.0               # 离自己起降点这么近才算"落在起降点上"
LANDED_HOME_WAIT_S = 300.0


class _Inbox:
    """一个事件一个信箱。必须在对方可能发之前创建——可靠事件通道先 ACK 再分发，
    没注册处理函数的事件会被确认后丢弃。"""

    def __init__(self, sdk, name):
        import threading
        self.name, self.data = name, {}
        self._ev = threading.Event()
        sdk.on_teammate_event(name, self._on)

    def _on(self, **kw):
        self.data = kw
        self._ev.set()

    def wait(self, timeout):
        if not self._ev.wait(timeout):
            raise TimeoutError(f'等事件 {self.name} 超过 {timeout:.0f} 秒')
        return self.data

    def ready(self):
        return self._ev.is_set()

    def clear(self):
        """收完一次复位，好接下一次。1#/2# 两栋楼都可能有火情，灭火周期要能跑第二轮。"""
        self._ev.clear()
        self.data = {}


def _wait_any(boxes, timeout):
    """等这几个信箱里**任意一个**来消息，返回先到的那个；都没来就抛超时。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for b in boxes:
            if b.ready():
                return b
        time.sleep(0.1)
    raise TimeoutError(f'等事件 {[b.name for b in boxes]} 超过 {timeout:.0f} 秒')


def _capture_photo(sdk, tag, camera='front'):
    """拍一张存到 PHOTO_DIR 并打印路径（"回传"= 存进挂给地面站的 /logs）。

    走 SDK 的 `capture_photo()`（2026-09-30 新增）。**不发声光事件**：声光事件
    是固定枚举（见 _sound_light_port.py 的 SOUND_LIGHT_EVENTS），表里没有
    "拍照"这一项，自造事件名会直接抛 ValueError 把整个任务打断——首次跑
    任务3 就是这么挂的。真要给拍照配声光，得先在事件表里加一项。

    拍不到不让整个任务失败：照片是交付物，不是流程前提。
    """
    x, y, _ = sdk.get_local_position()
    wx, wy = sdk.local_to_world(x, y, 0.0)[:2]
    stamp = time.strftime('%H%M%S')
    path = f'{PHOTO_DIR}/{sdk.namespace}_{stamp}_{tag}.png'
    try:
        # 前视/下视是两个独立的固定安装相机（模型里两个 camera_joint 都是
        # type='fixed'），不存在"切视角"这回事，capture_photo 按名字订阅
        # 对应话题就够了。
        sdk.capture_photo(path, camera=camera)
        print(f'[{sdk.namespace}] 拍照回传 {tag}：{path}'
              f'（位置 ({wx:.2f}, {wy:.2f})，朝向 {math.degrees(sdk.get_current_yaw()):.0f}°）',
              flush=True)
        return path
    except Exception as exc:
        print(f'[{sdk.namespace}] 拍照失败 {tag}（{exc}），流程继续', flush=True)
        return None


def _goto_world(sdk, wx, wy, what, z_agl=CRUISE_AGL_M):
    lx, ly, lz = sdk.world_to_local(wx, wy, z_agl)
    print(f'[{sdk.namespace}] 飞往{what} ({wx:.2f}, {wy:.2f})', flush=True)
    with sdk.fixed_altitude(lz):
        sdk.goto(lx, ly, lz)


def _start_spot_clear_watch(sdk, spot_world):
    """后台盯着侦察机什么时候离开发射点，离开了就通知任务机。

    为什么需要：任务机被要求"前往侦察机位置"，但那个点上此刻**正杵着侦察机**。
    ego_planner 看到目标点被占，会把轨迹终点推到障碍边缘，飞机原地不动，5 秒后
    被 goto() 判成不可达——2026-09-30 实测任务机收到破窗通知立刻出发（07:26:51.74），
    侦察机同一瞬间才开始离开（07:26:51.94），6 秒后任务机就挂了。

    侦察机照常"发射完就走"（用户要求的顺序不变），只是多发一条"我让开了"，
    任务机收到这条才真的飞进去。
    """
    import threading
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
                time.sleep(SPOT_POLL_S)
                continue
            d = math.hypot(wx - sx, wy - sy)
            if d >= SPOT_CLEAR_M:
                _send(f'已离开发射点（{d:.1f} m ≥ {SPOT_CLEAR_M:.1f} m）')
                return
            time.sleep(SPOT_POLL_S)
        if not state['sent']:
            # 超时也要放行，不然任务机会一直等。走到这儿说明两条路都没触发，
            # 属于异常，要在日志里说清楚。
            _send(f'等了 {SPOT_CLEAR_WAIT_S:.0f} 秒仍没离开发射点（超时兜底）')

    threading.Thread(target=_loop, daemon=True).start()

    def _notify_arrived(where):
        """侦察机已经飞到下一个落脚点了——不管离发射点多远，位置都已经腾出来。

        这是**主放行路径**：距离门限只是提前放行。下一个落脚点有可能离发射点
        很近（火情在 1# 东单元时发射点到 G 只有 2.92 m），光靠门限会一直等不到。
        """
        _send(f'已飞到{where}，发射点已腾出')

    return _notify_arrived


def _fly_route(sdk, legs, z_agl=CRUISE_AGL_M):
    """按航点序列飞，**每个航点先把机头转到下一段的方向、停住，再走**，
    航段之间航向不再变化（2026-09-30 用户要求，跟编队飞行同一条规则）。

    legs 是 [(名字, (wx, wy)), ...]，从飞机**当前位置**出发依次飞过去。
    停顿时长/转向超时/容差都复用《编队飞行示例》的常量，两边保持一致。
    """
    cx, cy, _ = sdk.get_local_position()
    for name, (wx, wy) in legs:
        tx, ty, tz = sdk.world_to_local(wx, wy, z_agl)
        heading = math.atan2(ty - cy, tx - cx)
        print(f'[{sdk.namespace}] 飞往航点{name} ({wx:.2f}, {wy:.2f})，'
              f'航向 {math.degrees(heading):.0f}°（停 {编队.WAYPOINT_HOLD_S:.0f} 秒转向）',
              flush=True)
        t0 = time.time()
        if not sdk.face_yaw(heading, timeout=编队.TURN_TIMEOUT_S,
                            tolerance_deg=编队.TURN_TOL_DEG):
            print(f'[{sdk.namespace}] 航向没转到位，仍继续前飞', flush=True)
        left = 编队.WAYPOINT_HOLD_S - (time.time() - t0)
        if left > 0:
            time.sleep(left)
        with sdk.fixed_altitude(tz):
            sdk.goto(tx, ty, tz)
        cx, cy = tx, ty


def _inspect_here(sdk):
    """在当前位置、当前朝向找高层火情。找到返回 Detection，没有返回 None。

    2026-09-30 去掉了原来"逐个候选高度升降过去再看一遍"的扫描（用户：没有
    火情就拍个照赶紧去下一个点，别在楼前等半天）。两条理由：
      · 几何上不需要——飞机悬停在 2.0 m、距楼约 3 m，候选高度 1.5/2.5 只偏
        ±9.5°，而前视相机垂直视场 64°，两个高度本来就都在画面里；
      · 实测也没用过——五轮下来每次都是"原地看到"（最快 1.4 秒），扫描分支
        一次都没有找到过火情，却在没有火情的楼前白白花掉 23 秒
        （实测 2# 站共 33.3 秒，其中升降两趟占 23 秒）。
    保留一次短重试：转向刚到位时画面可能还在稳，给一帧缓冲。
    """
    for i in range(1, DETECT_TRIES + 1):
        try:
            det = sdk.wait_for_detection(HIGH_FIRE, timeout=DETECT_TIMEOUT_S,
                                         camera='front')
            print(f'[{sdk.namespace}] 原地看到高层火情', flush=True)
            return det
        except Exception:
            if i < DETECT_TRIES:
                time.sleep(DETECT_SETTLE_S)
    return None


def _drive_servos(sdk, pwm, label):
    """抓取/释放机构。仿真里飞控不一定配了舵机输出，动不了就打印、继续飞完流程。"""
    try:
        sdk.set_servos({sv: pwm for sv in sorted(sdk.servos)})
    except Exception as exc:
        print(f'[{sdk.namespace}] {label}：舵机没动（{exc}）'
              f'——真机需要飞控配好 MAIN7/MAIN9', flush=True)
        return
    time.sleep(SERVO_TRAVEL_S)
    print(f'[{sdk.namespace}] {label}完成', flush=True)


def _supply_point_action(sdk, pwm, label, sound=None):
    """飞到物资点 -> 边瞄准边降落到底 -> 驱动机械抓 -> 起飞回巡航高度。

    降落复用《地面火情搜索示例》的 `descend_onto()`：**不要用
    `precision_land_and_confirm()`**，那是"对准一点、下降一点"的分级下降，
    从 2.5 m 下来要 40 秒以上、30 秒时限内走不完，每次都走超时兜底
    （2026-09-23 实测）。descend_onto 是连续修正，同一时间既对准也下降。
    """
    _goto_world(sdk, SUPPLY_XY[0], SUPPLY_XY[1], '物资点')
    地面.descend_onto(sdk, SUPPLY_TAG, '灭火器材')
    print(f'[{sdk.namespace}] 已降落在物资点，开始{label}', flush=True)
    if sound:
        sdk.play_sound_light(sound)
    _drive_servos(sdk, pwm, label)
    sdk.takeoff(height_m=CRUISE_AGL_M)
    time.sleep(HOVER_AFTER_TAKEOFF_S)


def center_fire_in_view(sdk, tag='火情'):
    """把飞机挪到**正对火情窗口**的位置：机头垂直于楼面不动，横向平移 + 升降。
    返回是否对上了。

    2026-09-30 订正：上一版只转机头让 tag 在画面里居中——那是**斜着瞄准**，
    不是对准。飞机停在观察位 M(6,16)，而火情随机到东单元时 tag 在 x=6.5，
    横向差 0.5 m；转机头能把它转到画面中心，但飞机仍然斜对着墙，沿机头前移
    之后落点离窗口正前方还差近 0.5 m。弹丸要穿窗，得正对。

    所以改成**机头锁死在垂直于楼面的方向**（调用方进来之前已经 face_yaw 到位），
    两个误差都用平移消除：
      · 横向：按框宽估距离（tag 实际 0.5 m），`dist * tan(atan2(bbox_x-W/2, f))`
        就是飞机相对窗口中心的横向偏移，沿机头**右手方向**平移这么多；
      · 垂直：同样的距离乘垂直角，整机升降（相机固定安装、不能俯仰）。
    两轴都进容差才算对准。

    为什么不用 `sdk.center_on_target()`：见 IMAGE_W 上面那段注释——它只认下视相机。
    """
    yaw0 = sdk.get_current_yaw()            # 垂直于楼面的朝向，全程不动
    for i in range(1, AIM_MAX_TRIES + 1):
        try:
            det = sdk.wait_for_detection(HIGH_FIRE, timeout=4.0, camera='front')
        except Exception:
            print(f'[{sdk.namespace}] 第{i}轮对准：前视相机看不到{tag}', flush=True)
            time.sleep(AIM_SETTLE_S)
            continue
        dx_rad = math.atan2(det.bbox_x - IMAGE_W / 2.0, FOCAL_PX)
        dy_rad = math.atan2(det.bbox_y - IMAGE_H / 2.0, FOCAL_PX)   # 画面 y 向下为正
        dist = FOCAL_PX * TAG_SIZE_M / max(1.0, det.bbox_width)
        lat = dist * math.tan(dx_rad)        # >0：窗口在飞机右侧，要往右挪
        dz = -dist * math.tan(dy_rad)        # 目标在画面下方 -> 要降高度
        print(f'[{sdk.namespace}] 第{i}轮对准：{tag}在画面 ({det.bbox_x:.0f},{det.bbox_y:.0f})，'
              f'框宽 {det.bbox_width:.0f}px -> 距离约 {dist:.2f} m，'
              f'横向 {lat:+.2f} m，高度 {dz:+.2f} m', flush=True)
        if abs(lat) <= AIM_TOL_XY_M and abs(dz) <= AIM_TOL_Z_M:
            print(f'[{sdk.namespace}] {tag}已对准（横向 {lat:+.2f} m，垂直 {dz:+.2f} m）',
                  flush=True)
            return True
        cx, cy, cz = sdk.get_local_position()
        # 机头右手方向（ENU/yaw 逆时针为正时，right = (sin yaw, -cos yaw)）
        rx, ry = math.sin(yaw0), -math.cos(yaw0)
        _, _, z_lo = sdk.world_to_local(0.0, 0.0, AIM_Z_MIN_M)
        _, _, z_hi = sdk.world_to_local(0.0, 0.0, AIM_Z_MAX_M)
        tx, ty = cx + rx * lat, cy + ry * lat
        tz = min(max(cz + dz, z_lo), z_hi)
        print(f'[{sdk.namespace}] 平移到 ({tx:.2f}, {ty:.2f}, {tz:.2f})（局部系）', flush=True)
        sdk.goto_direct(tx, ty, tz)          # 直线平移/升降，不经规划器
        # 平移不改朝向，但 goto_direct 之后朝向可能被带偏，复位一次
        sdk.face_yaw(yaw0, timeout=编队.TURN_TIMEOUT_S, tolerance_deg=2.0)
        time.sleep(AIM_SETTLE_S)
    print(f'[{sdk.namespace}] {AIM_MAX_TRIES} 轮仍没对准{tag}，按当前状态继续', flush=True)
    return False


def _back_to_observe_alt(sdk):
    """回到观察高度。_inspect_here 找不到火情时会升降到 1.5/2.5 米去看，
    回来拍那张交付照片之前得先把高度收回去。"""
    cx, cy, _ = sdk.get_local_position()
    _, _, lz = sdk.world_to_local(0.0, 0.0, OBSERVE_AGL_M)
    sdk.goto_direct(cx, cy, lz)


def _step_in_to_launch(sdk):
    """沿机头方向前移 FORWARD_BEFORE_FIRE_M，返回发射点世界坐标。

    调用之前必须已经 center_fire_in_view() 对准——2026-09-30 把对准和前移
    拆开了，中间要插一张"对准后的交付照片"。
    """
    cx, cy, cz = sdk.get_local_position()
    yaw = sdk.get_current_yaw()
    tx = cx + FORWARD_BEFORE_FIRE_M * math.cos(yaw)
    ty = cy + FORWARD_BEFORE_FIRE_M * math.sin(yaw)
    print(f'[{sdk.namespace}] 沿机头前移 {FORWARD_BEFORE_FIRE_M:.1f} m 到发射点', flush=True)
    sdk.goto_direct(tx, ty, cz)
    wx, wy = sdk.local_to_world(tx, ty, 0.0)[:2]
    return float(wx), float(wy), float(cz)


# ----------------------------------------------------------------------------
# 侦察机 NX01
# ----------------------------------------------------------------------------
def recon(sdk):
    """侦察机：巡检拍照 -> （遇火情就走一轮协同灭火）-> 巡检完毕 -> 编队返回。"""
    at_e = _Inbox(sdk, EV_AT_E)
    done = _Inbox(sdk, EV_EXTINGUISHED)
    landed_home = _Inbox(sdk, EV_LANDED_HOME)

    sdk.takeoff(height_m=CRUISE_AGL_M)
    sdk.play_sound_light('侦察机起飞')
    # 巡检航点：每个点先转向下一个点再走，航段内航向恒定
    _fly_route(sdk, [('A', ROUTE_A), ('B', ROUTE_B), ('C', ROUTE_C),
                     ('G', ROUTE_G), ('E', ROUTE_E)])

    # ================= 过程① 巡检拍摄 =================
    # 三栋楼每栋必须拍一张并回传，按 3 -> 2 -> 1 走。面朝 2#/1# 时开火情识别，
    # 识别到就当场插入一轮协同灭火（过程②），灭完继续巡检下一栋。
    at_station = None          # 当前停在哪个观察位，同一个位就不重复飞
    fired_any = False
    pending_spot_clear = None  # 破窗后待发的"发射点已腾出"通知，见 _start_spot_clear_watch
    for idx, (bldg, pt, pt_name, yaw_deg, detect) in enumerate(INSPECT_STATIONS):
        if at_station != pt_name:
            _fly_route(sdk, [(pt_name, pt)], z_agl=OBSERVE_AGL_M)
            at_station = pt_name
            if pending_spot_clear is not None:
                pending_spot_clear(f'下一个巡检点 {pt_name}')
                pending_spot_clear = None
        print(f'[{sdk.namespace}] 在 {pt_name} 点转到 {yaw_deg:.0f}° 巡检 {bldg} 楼', flush=True)
        sdk.face_yaw(math.radians(yaw_deg), timeout=编队.TURN_TIMEOUT_S,
                     tolerance_deg=编队.TURN_TOL_DEG)
        # 没火情的楼：转到位就拍，照片是硬性交付物（每栋楼必须有一张），
        # 不能让后面的识别失败把它带掉。
        # **有火情的楼等对准之后再拍**（用户 2026-09-30）——那张照片要能看清
        # 着火点，火情在画面边上等于没拍到。
        if not detect or fired_any:
            _capture_photo(sdk, f'{bldg}楼')
            if not detect:
                continue
            # 火情只可能有一处（premise：1#/2# 其中一栋）。已经协同灭过火了，
            # 后面的楼只补拍那张必交的照片就走，不必再跑一遍识别+升降扫高度
            # ——用户 2026-09-30："侦查机到 1# 楼，拍摄完即可前往 G 点等待"。
            print(f'[{sdk.namespace}] 火情已处置，{bldg} 楼只拍照不再查', flush=True)
            continue

        sdk.play_sound_light('侦察机排查高层火情')
        det = _inspect_here(sdk)
        if det is None:
            # 没找到：_inspect_here 会为了找它升降到 1.5/2.5 米，先回观察高度
            # 再拍，免得这张交付照片是从一个歪高度上拍的。
            _back_to_observe_alt(sdk)
            _capture_photo(sdk, f'{bldg}楼')
            print(f'[{sdk.namespace}] {bldg} 楼没有火情，继续巡检', flush=True)
            continue

        # ================= 过程② 协同灭火 =================
        sdk.play_sound_light('侦察机发现高楼火情')
        # 先对准，再拍这栋楼的交付照片——这样火情在画面正中
        center_fire_in_view(sdk, '高层火情')
        _capture_photo(sdk, f'{bldg}楼')
        fire_pos = _step_in_to_launch(sdk)
        at_station = None       # 前移过了，不再在观察位上
        print(f'[{sdk.namespace}] {bldg} 楼有火情，发射点 '
              f'({fire_pos[0]:.2f}, {fire_pos[1]:.2f})，通报任务机并原地等待', flush=True)
        # **先播报再发事件**（用户 2026-09-30）：任务机一收到 EV_FIRE 就起飞并
        # 播报"任务机起飞"，两条播报会抢在一起、顺序不定。把"侦察机通报高层
        # 火情"提到发事件之前，时间上就必然排在"任务机起飞"前面。
        sdk.play_sound_light('侦察机通报高层火情')
        sdk.send_to_teammate(EV_FIRE, x=fire_pos[0], y=fire_pos[1],
                             z=fire_pos[2], at=bldg)

        at_e.wait(AT_E_WAIT_S)          # 原地等任务机到 E 点待命
        print(f'[{sdk.namespace}] 任务机已到 E 点，发射破窗弹', flush=True)
        sdk.play_sound_light('侦察机发射破窗弹')
        高楼.fire_launcher(sdk, '发射破窗弹')
        sdk.send_to_teammate(EV_BREACHED)
        sdk.play_sound_light('侦察机破窗完成')
        # 发射点马上要让给任务机了。两条放行路径：离得够远（watchdog 线程），
        # 或者已经飞到下一个落脚点（下面显式调 spot_clear_notify）。
        spot_clear_notify = _start_spot_clear_watch(sdk, (fire_pos[0], fire_pos[1]))
        at_e.clear()                    # 复位，下一栋楼要是也有火情还能再收一次
        done.clear()
        fired_any = True
        # 发射完就走：还有下一个巡检点就去下一个，没有就到 G 等灭火完成。
        # 不在这里等 EV_EXTINGUISHED——那是任务机的活，侦察机的巡检不该被它挡住。
        if idx + 1 < len(INSPECT_STATIONS):
            print(f'[{sdk.namespace}] 还有下一个巡检点，先去巡检', flush=True)
        # 巡检循环的下一轮（或循环结束后的"飞 G 点"）会把飞机挪走；到位之后
        # 由 _fly_route 的调用点负责通知。这里只把回调存下来。
        pending_spot_clear = spot_clear_notify

    # ---- 巡检完毕 ----
    print(f'[{sdk.namespace}] 三栋楼巡检拍照完毕', flush=True)
    sdk.send_to_teammate(EV_INSPECT_DONE)
    _fly_route(sdk, [('G', ROUTE_G)])
    if pending_spot_clear is not None:
        # 火情在最后一站（1#）时走这一支：飞到 G 就算把发射点腾出来了。
        # G(17,16) 离 1# 东单元的发射点只有 2.9 m，光靠距离门限放不了行。
        pending_spot_clear('G 点')
        pending_spot_clear = None
    if fired_any:
        print(f'[{sdk.namespace}] 在 G 点等任务机灭火完成…', flush=True)
        done.wait(EXTINGUISH_WAIT_S)

    # ================= 过程③ 编队返回 =================
    # start_xy 要的是**世界坐标**（航线本身就是世界系）。这里飞机刚飞到 G，
    # 直接用 ROUTE_G——早先传 get_local_position() 的局部坐标，被当成世界坐标
    # 用，站位点算到了 3# 楼那一片、飞不过去（2026-09-29 实测）。
    # 解散判据：**任务机过 D 点**就解散（用户 2026-09-30 要求），不是默认的
    # "长机飞回自己起飞点上空"。解散后长机继续飞到 A 点待命，任务机去物资点
    # 释放器材再回自己起降点。
    编队.leader_route(sdk, [ROUTE_G, ROUTE_D], spacing_m=SPACING_M,
                      start_xy=ROUTE_G, final_xy=ROUTE_A,
                      disband_after_follower_passes=ROUTE_D)
    编队._hold_at_point_a(sdk, ROUTE_A)
    # 任务完成的播报时机：**任务机确实落在自己的起降点上**（用户 2026-09-30）。
    # 不能拿"降落动作结束"当判据——任务机全程要降落三次（取器材、放器材、回家），
    # 前两次都不是任务结束。所以由任务机自己核对落点坐标后发事件，这边等它。
    try:
        d = landed_home.wait(LANDED_HOME_WAIT_S)
        print(f'[{sdk.namespace}] 任务机已降落在自己起降点 '
              f'({float(d.get("x", 0)):.2f}, {float(d.get("y", 0)):.2f})', flush=True)
    except TimeoutError:
        print(f'[{sdk.namespace}] 等任务机回起降点超过 {LANDED_HOME_WAIT_S:.0f} 秒，仍播报完成',
              flush=True)
    sdk.play_sound_light('侦察机任务完成')


# ----------------------------------------------------------------------------
# 任务机 NX02
# ----------------------------------------------------------------------------
def supply(sdk):
    """任务机：等火情通报 -> 飞 E 待命 -> 破窗后到侦察机位置发射灭火弹 ->
    （侦察机巡检完了就）就近入列，编队返回。

    灭火周期写成循环：1#/2# 两栋楼都可能有火情，侦察机可能通报两次。每轮结束
    后等"下一次火情通报"或"侦察机巡检完毕"，先到哪个走哪个。
    """
    fire = _Inbox(sdk, EV_FIRE)
    breached = _Inbox(sdk, EV_BREACHED)
    spot_clear = _Inbox(sdk, EV_SPOT_CLEAR)
    inspect_done = _Inbox(sdk, EV_INSPECT_DONE)
    route_done = 编队._Inbox(sdk, 编队.ROUTE_DONE)
    route_plan = 编队._Inbox(sdk, 编队.ROUTE_PLAN)

    airborne = False
    print(f'[{sdk.namespace}] 等侦察机通报高层火情…', flush=True)
    while True:
        box = _wait_any([fire, inspect_done], EXTINGUISH_WAIT_S)
        if box is inspect_done and not fire.ready():
            print(f'[{sdk.namespace}] 侦察机已巡检完毕，准备入列', flush=True)
            break

        d = fire.wait(1.0)
        fx, fy, fz = float(d['x']), float(d['y']), float(d.get('z', CRUISE_AGL_M))
        print(f'[{sdk.namespace}] 收到火情：{d.get("at")} 楼，侦察机发射点 '
              f'({fx:.2f}, {fy:.2f})', flush=True)
        fire.clear()

        if not airborne:
            sdk.takeoff(height_m=CRUISE_AGL_M)
            sdk.play_sound_light('任务机起飞')
            time.sleep(HOVER_AFTER_TAKEOFF_S)
            # 用户 2026-09-30 要求：参与灭火之前先到物资点降落抓取灭火器材
            _supply_point_action(sdk, GRAB_PWM, '抓取灭火器材',
                                 sound='任务机抓取灭火弹')
            airborne = True

        # ---- 到 E 点待命，通报到位，等破窗 ----
        _goto_world(sdk, ROUTE_E[0], ROUTE_E[1], 'E点待命位')
        sdk.send_to_teammate(EV_AT_E)
        sdk.play_sound_light('任务机高层灭火已就位')
        print(f'[{sdk.namespace}] 已在 E 点待命，等侦察机破窗', flush=True)
        breached.wait(BREACH_WAIT_S)
        breached.clear()
        # 发射点上这会儿还杵着侦察机，直接飞过去规划器会把终点推到障碍边缘、
        # 飞机原地不动然后被判不可达（2026-09-30 实测）。等它让开再进场。
        print(f'[{sdk.namespace}] 已破窗，等侦察机让开发射点…', flush=True)
        spot_clear.wait(SPOT_CLEAR_WAIT_S + 30.0)
        spot_clear.clear()

        # ---- 到侦察机的发射点，连发 4 发灭火弹 ----
        lx, ly, _lz = sdk.world_to_local(fx, fy, CRUISE_AGL_M)
        print(f'[{sdk.namespace}] 飞往侦察机位置 ({fx:.2f}, {fy:.2f})', flush=True)
        try:
            with sdk.fixed_altitude(fz):
                sdk.goto(lx, ly, fz)
        except GotoUnreachableError as exc:
            d = float(getattr(exc, 'distance_m', 1e9))
            if d > ARRIVE_ACCEPT_M:
                raise
            # 规划器把终点推到了膨胀区边缘。接下来还要把火情居中到前视画面，
            # 差一两米不影响发射，不值得让整个任务失败。
            print(f'[{sdk.namespace}] 没能精确到点（还差 {d:.2f} m ≤{ARRIVE_ACCEPT_M:.0f} m），'
                  f'就地发射', flush=True)
        sdk.play_sound_light('任务机到达瞄准点')
        # 同样走前视居中，不用 center_on_target（只认下视相机，见 IMAGE_W 注释）
        center_fire_in_view(sdk, '高层火情')

        sdk.play_sound_light('任务机发射灭火弹')
        for i in range(1, EXTINGUISHER_SHOTS + 1):
            高楼.fire_launcher(sdk, f'发射灭火弹 {i}/{EXTINGUISHER_SHOTS}')
            if i < EXTINGUISHER_SHOTS:
                time.sleep(SHOT_INTERVAL_S)
        sdk.send_to_teammate(EV_EXTINGUISHED)
        print(f'[{sdk.namespace}] {EXTINGUISHER_SHOTS} 发灭火弹发射完毕，已通知侦察机',
              flush=True)
        # 灭火完毕把高度拉回编队巡航高度（用户 2026-09-30）：对准火情时飞机被
        # 挪到了着火点的高度（1.5 或 2.5 m），跟编队巡航的 2.0 m 不一样，带着
        # 这个高度直接入列会让队形在竖直方向上错开。
        cx, cy, _ = sdk.get_local_position()
        _, _, cruise_z = sdk.world_to_local(0.0, 0.0, CRUISE_AGL_M)
        print(f'[{sdk.namespace}] 回到编队巡航高度 {CRUISE_AGL_M:.1f} m', flush=True)
        sdk.goto_direct(cx, cy, cruise_z)
        # 回到循环开头：要么还有下一栋楼的火情通报，要么侦察机巡检完毕。

    if not airborne:
        # 一次火情都没有（两栋楼都没着火）——照样要起飞入列跟着返航
        sdk.takeoff(height_m=CRUISE_AGL_M)
        sdk.play_sound_light('任务机起飞')

    # ================= 过程③ 编队返回 =================
    # goto_station=False = **就近入列**：任务机这会儿就在侦察机附近，再飞一趟
    # "航线起点后方 spacing 米"的站位点纯属绕路。起降由本脚本自己管，编队只管空中。
    编队.follow_formation(sdk, SPACING_M, inbox=route_done, plan=route_plan,
                          goto_station=False)

    # 解散在**任务机过 D 点**之后（用户 2026-09-30 要求）。解散后先回物资点
    # 降落、松开机械抓模拟释放器材，再回自己起降点降落。
    _supply_point_action(sdk, RELEASE_PWM, '释放灭火器材')
    编队._land_at_pad(sdk)
    # 核对**真的落在自己的起降点上**再播报（用户 2026-09-30）：全程要降落三次
    # （取器材、放器材、回家），"降落动作完成"本身说明不了任务结束。
    pad = 编队._own_pad(sdk)
    px, py, _ = sdk.get_local_position()
    wx, wy = sdk.local_to_world(px, py, 0.0)[:2]
    d = math.hypot(wx - pad[0], wy - pad[1])
    if d <= LANDED_HOME_TOL_M:
        print(f'[{sdk.namespace}] 已降落在自己起降点 ({wx:.2f}, {wy:.2f})，'
              f'离标称点 {d:.2f} m', flush=True)
        sdk.play_sound_light('任务机已降落')
    else:
        print(f'[{sdk.namespace}] ⚠️ 落点 ({wx:.2f}, {wy:.2f}) 离自己起降点 {d:.2f} m '
              f'(>{LANDED_HOME_TOL_M:.1f} m)，不算落在起降点上', flush=True)
    # 无论落点准不准都要通知侦察机，否则它会一直等到超时
    sdk.send_to_teammate(EV_LANDED_HOME, x=wx, y=wy, ok=bool(d <= LANDED_HOME_TOL_M))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--namespace', required=True)
    # 运行器（运行仿真.sh）传的是 leader/follower，这里同时接受语义化的
    # recon/supply 两个别名——长机=侦察机、僚机=任务机。
    ap.add_argument('--role', required=True,
                    choices=('leader', 'follower', 'recon', 'supply'))
    ap.add_argument('--teammate', required=True)
    ap.add_argument('--spacing', type=float, default=SPACING_M)
    args = ap.parse_args()

    sdk_role = 'recon' if args.role in ('leader', 'recon') else 'supply'
    sdk = DroneSDK(namespace=args.namespace, role=sdk_role,
                   teammate_namespace=args.teammate)
    try:
        if args.role in ('leader', 'recon'):
            recon(sdk)
        else:
            supply(sdk)
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()
