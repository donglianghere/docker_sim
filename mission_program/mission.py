#!/usr/bin/env python3
"""2026大赛contest task——第四部分最终交付物：全流程任务程序（对应清单D2-D7
/方案第四部分4.0-4.5节）。

**这是一份普通的选手Python程序，不是`contest_mission`包里的ROS2节点**：只
`import contest_sdk`，不`import rclpy`、不碰任何ROS消息类型（`DroneSDK`
已经把这些都封装掉了，见`capabilities.py`模块头"封装完整性"的说明）。

**双机对称性（方案1.5节，硬性要求）**：全文件只有一份代码，RECON/SUPPLY
两个角色的行为差异全部通过`if sdk.role == 'recon': ... else: ...`分支
体现，不出现任何"如果namespace=='NX01'"这类写法——唯一允许"查
namespace"的地方是`mission_constants.own_landing_pad_xy(sdk.namespace)`
（查自己是谁、不是判断该做什么，D1文件里已经明确这是允许的例外）。

──────────────────────────────────────────────────────────────────
**这份程序里三个需要自己拍板设计的判断点**（任务要求写清楚理由，这里
先给一个索引，具体理由写在各自函数的docstring/注释里，不在这里重复）：

1. **SUPPLY怎么知道RECON已经飞完第一部分航线、可以停止编队跟随了**
   ——见`_estimate_route_wait_seconds()` + `supply_main()`里
   "D3判断点①"注释块。
2. **SUPPLY的STANDBY循环具体怎么实现**（`on_teammate_event()`回调要不
   要在回调里直接执行阻塞的sdk调用）——见`_EventInbox`类的docstring
   + `supply_main()`里"D4判断点②"注释块。
3. **两个检测器（地面火情/高层火情）"并行监听"具体怎么实现**——见
   `_spawn_detection_watcher()`的docstring + `_run_search_and_response()`
   里"D4判断点③"注释块。

──────────────────────────────────────────────────────────────────
**已解决的接口缺口记录（2026-09-13）**：写这份程序时最初发现`contest_
sdk`没有覆盖D6需要的两个能力——读`~/fire_pillar_staging_pose`（SUPPLY
要去的任务机等待点坐标）、读`~/fire_pillar_aim_pose`（RECON自己锁定的
瞄准点坐标），`DroneSDK`原来的方法都是"发一个触发指令+等一个明确的
完成信号"这种模式，没有"订阅一个持续广播的状态话题、取最新值"这类
封装。当时先用几何近似占位函数顶上让流程结构能跑通，随后已经在
`contest_sdk/capabilities.py`里正式补上了`read_fire_pillar_staging_
pose()`/`read_fire_pillar_aim_pose()`两个真实能力（跟`get_mission_
state()`/`centered_pose`是同一种"持续订阅、取最新值"的封装模式），
这份文件已经改用真实调用，不再需要占位近似。
──────────────────────────────────────────────────────────────────
"""
import argparse
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import mission_constants as mc
import mission_geometry as mg
import mission_vocab as mv

from contest_sdk import DroneSDK
from contest_sdk.exceptions import ContestSdkError, DetectionTimeoutError

# ============================================================================
# 常量
# ============================================================================

# ---- 视觉检测的class_id（方案4.3节，两种火情标识）----
GROUND_FIRE_CLASS_ID = 'apriltag:2'
HR_FIRE_CLASS_ID = 'apriltag:1'

# ---- 房间边界（方案D4："房间尺寸size_x=20,size_y=25可以当已知的场地
#      边界常量使用"——这跟"火情/障碍物坐标不能写死"不是一回事，房间
#      本身的边界属于场地信息，不是要发现的目标）----
# 这两个常量特意放在mission.py而不是mission_constants.py：D1的三个helper
# 文件已经写好+测过，任务要求不去改它们，房间尺寸是D4新引入的场地常量，
# 放在实际用到它的这个文件里，比回头改D1文件更符合"不碰D1"的边界。
ROOM_SIZE_X_M = 20.0
ROOM_SIZE_Y_M = 25.0
# 房间以原点为中心（方案D4的建议），可飞行范围再留一点墙体安全边距，
# 这个margin直接传给generate_ground_scan_waypoints的wall_margin参数，
# 不需要在这里再减一次。
ROOM_WALL_MARGIN_M = 1.0

# ---- 各种阻塞调用的超时（秒）——沿用SDK方法自己的默认值就够用的，
#      不在这里重复传参；这里只列需要显式覆盖/新增的 ----
CENTER_ON_TARGET_TIMEOUT_S = 30.0
PRECISION_LAND_TIMEOUT_S = 90.0  # 精降下降+等真正autoland，比普通goto更久，留够余量
GRAB_TAKEOFF_RETRY_TIMEOUT_S = 30.0
# `fire_pillar_aim_node`锁定高层火情之后应该几乎立刻开始广播这两个话题
# （见contest_sdk的`read_fire_pillar_staging_pose()`/`read_fire_pillar_
# aim_pose()`docstring），超时给10秒纯粹是留个安全余量，不是预期真的
# 要等这么久。
FIRE_PILLAR_POSE_READ_TIMEOUT_S = 10.0

# 2026-09-14：这里原来有一个`TAKEOFF_SETTLE_WAIT_S`常量+两处`time.
# sleep()`，是"起飞后立刻goto()"这个问题排查阶段的临时缓解措施（固定
# 等3秒再继续）。根本原因后来查清楚是`sdk.takeoff()`本身只等`armed=
# True`就返回，不等飞控真正爬升到位、状态转换稳定——已经直接在
# `contest_sdk/capabilities.py::takeoff()`里修好（阻塞到位置连续
# 稳定一段时间才返回，不是瞎猜一个固定秒数），这个文件里的临时等待
# 因此不再需要，删掉了，不留着当"双重保险"（`sdk.takeoff()`已经保证
# 了这件事，选手侧没有必要在不知道具体等待时长是否够用的情况下再自己
# 猜一个数字兜底）。

# 后台检测线程单次wait_for_detection()的超时上限——不是"检测响应延迟"，
# 是"这条后台线程整场任务最多陪跑多久"的一个安全上限（见
# `_spawn_detection_watcher()`docstring），设成比整场任务合理时长更长
# 的一个宽松值，正常情况下根本不会撞到。
DETECTION_WATCH_TIMEOUT_S = 1800.0

# STANDBY循环/事件收件箱轮询间隔——不需要很密，事件到达的判定延迟允许
# 有零点几秒（跨机消息本身走ACK+1Hz重发协议，延迟量级本来就是秒级），
# 密集轮询只会白白占CPU。
STANDBY_POLL_INTERVAL_S = 0.5

# RECON等SUPPLY关键消息的超时——故意给得很宽松（远超过`send_to_teammate()`
# 自己的45秒ACK超时），因为这里等的不是"消息送达确认"，是"SUPPLY真的
# 飞到/做完一个物理动作"，包含飞行时间本身，量级是分钟不是秒。这两个
# 值设成"正常不可能撞到、只在真的卡死时兜底"的松超时，不是精确算出来的。
SUPPLY_READY_WAIT_TIMEOUT_S = 300.0
SUPPLY_LANDED_WAIT_TIMEOUT_S = 3600.0

# ---- D3判断点①用到的常量：SUPPLY估算"RECON飞完全程大概要多久"----
# 保守估计的巡航速度（米/秒）——故意比ego_planner实际巡航速度悲观（宁可
# 等久一点误判"还没飞完"，也不要提前停止编队跟随导致SUPPLY掉队/降落
# 在半路）。这不是一个精确的运动学参数，只是"够用的下界"。
ASSUMED_CRUISE_SPEED_MPS = 0.4
# 额外安全余量——覆盖起飞/避障重规划/立柱这几个正常波动，不是给巡航速度
# 本身留的余量（巡航速度已经悲观估计过一次了）。
SUPPLY_WAIT_SAFETY_MARGIN_S = 60.0


# ---- D7节：20个通知点里，6个"纯通知、不要求对方处理"的补充事件名 ----
# 这些不在D1的mission_vocab.py里（那份文件只收方案4.1节明确列出的4个
# 跨机协同事件），因为它们不是"双机协同必需"的事件，纯粹是为了满足
# 方案4.4节20点表格里标记"通知方式=mission_events"的那几个点——事件名
# 本身是选手自定义字符串，SDK/mission_vocab.py都不限定，直接在这个文件
# 里补充定义，不需要回头改D1文件。
_NOTIFY_SUPPLY_GRABBED = 'supply_grabbed'                          # D7 第8点
_NOTIFY_SUPPLY_DROPPED = 'supply_dropped'                          # D7 第9点
_NOTIFY_PILLAR_SELECTED = 'pillar_selected'                        # D7 第10点
_NOTIFY_HR_FIRE_DETECTED = 'hr_fire_detected'                      # D7 第12点
_NOTIFY_SUPPLY_ARRIVED_AIM_POINT = 'supply_arrived_at_aim_point'   # D7 第17点
_NOTIFY_SUPPLY_LAUNCH_DONE = 'supply_launch_done'                  # D7 第18点
# 这一个不在D7的20点表格里，是D4收尾同步逻辑需要的额外信号，见
# `supply_main()`/`recon_main()`末尾"D4收尾同步"注释块的说明。
_NOTIFY_SUPPLY_LANDED = 'supply_landed'
# 2026-09-15用户新增要求："巡查完一个立柱，没火情则发布一个该立柱巡查
# 完毕的消息"——之前只在选中/命中两个时机发消息（10/12两点），"绕完一
# 整圈但没找到"这个结果之前是静默的，队友完全不知道RECON查完了哪些、
# 还是卡住了。同样是D7 20点表格之外的补充点，跟_NOTIFY_SUPPLY_LANDED
# 一样直接在这里补，不回头改D1的mission_vocab.py。
_NOTIFY_PILLAR_CLEARED = 'pillar_cleared'

# SUPPLY侧不需要处理、只是为了避免"收到事件但没注册处理回调"这条warning
# 日志噪音而注册的"纯知会"事件名清单（RECON发的10/12两个点）。
_RECON_ORIGINATED_PURE_NOTIFY_EVENTS = (
    _NOTIFY_PILLAR_SELECTED, _NOTIFY_HR_FIRE_DETECTED, _NOTIFY_PILLAR_CLEARED,
)
# 反过来，RECON侧同理需要"知道但不处理"SUPPLY发的8/9/17/18四个纯通知点。
_SUPPLY_ORIGINATED_PURE_NOTIFY_EVENTS = (
    _NOTIFY_SUPPLY_GRABBED,
    _NOTIFY_SUPPLY_DROPPED,
    _NOTIFY_SUPPLY_ARRIVED_AIM_POINT,
    _NOTIFY_SUPPLY_LAUNCH_DONE,
)


# ============================================================================
# 通用小工具（不依赖角色，RECON/SUPPLY都可能用到）
# ============================================================================

class _PositionTracker:
    """本地维护"当前位置(x, y)"的近似值——弥补`capabilities.py`没有暴露
    "查询自己当前位置"这个公开方法的接口缺口（`DroneSDK`内部确实缓存了
    `_odom_xyz`，但那是下划线开头的私有属性，任务要求"只能调用DroneSDK
    暴露的方法"，不允许伸手进去读私有属性）。

    这个缺口在这份程序里冒出来两次，用的是同一套近似思路，所以抽成一个
    小类复用，不重复发明：
    1. D3：RECON做长距离分段插值时，每一段插值的"起点"需要知道当前位置
       （方案4.2节"飞完一个途经点后下一段插值的起点就是这个途经点"）。
    2. D6：RECON选择"离当前位置最近"的立柱时，`mission_geometry.select_
       nearest_pillar_index()`的`current_xy`参数需要一个当前位置。

    **近似依据**：RECON全程只通过`sdk.goto()`移动（没有别的移动方式），
    `goto()`本身是阻塞调用，返回时（不管是`waypoint_state`变成
    `'completed'`还是被`cancel_goto()`打断变成`'cancelled'`）都代表"这次
    移动已经结束"，此时飞机的实际位置约等于"最近一次成功下达的目标点"
    ——如果是`'completed'`，这个近似就是精确值（`waypoint_state`本身就是
    到达确认）；如果是`'cancelled'`（被检测命中打断），飞机可能还没真正
    飞到目标点，存在一定误差，但这个追踪值唯一的用途是"给立柱选择规则
    打破队列顺序"这种粗粒度判断（立柱之间相距至少几米），远小于立柱间距
    的误差不影响选择结果的正确性。

    2026-09-13补：`contest_sdk`后来确实补上了`sdk.get_local_position()`
    ——但这个类维护的`xy`是**世界坐标**（跟D1的`mission_constants`常量、
    `mission_geometry`的立柱/绕飞几何计算统一用同一套坐标系），而
    `get_local_position()`返回的是这架飞机自己的局部坐标（见该方法
    docstring），两者不是一回事，不能直接拿它替换这个类——每次读都要
    再套一层`local_to_world()`转换，比这里"用最近一次成功下达的世界
    坐标目标点当近似值"这套现成的做法更麻烦，所以保留这个类不变。
    """

    def __init__(self, initial_xy: Tuple[float, float]):
        self.xy = initial_xy

    def update(self, xy: Tuple[float, float]) -> None:
        self.xy = xy


class _EventInbox:
    """跨机事件的线程安全"收件箱"——RECON/SUPPLY两个角色的事件等待/分派
    逻辑都基于这个类，是D4判断点②（"STANDBY循环具体怎么实现"）的核心
    实现，这里统一说清楚理由，两个角色各自的调用点不重复解释。

    **为什么不在`on_teammate_event()`的回调里直接执行阻塞的sdk调用**
    （比如收到`GROUND_FIRE_FOUND`就在回调里直接`sdk.goto(...)`）：

    1. `capabilities.py::on_teammate_event()`的docstring已经明确警告过
       "回调会在`RclpyRuntime`的后台spin线程里被调用……如果选手的回调
       内部要做比较重的事，建议自己另起线程"——但这条警告背后还有一个
       更硬的技术原因，值得在这里说透：`_rclpy_runtime.py`用
       `MultiThreadedExecutor`起了多个spin线程，但**这个SDK内部创建的
       所有订阅/service client，没有一个显式指定了`callback_group`**
       （逐个查过`capabilities.py`/`reliability.py`的`create_subscription`/
       `create_client`调用，全部用的默认callback group）——rclpy的规则
       是"同一个节点上不显式指定group的回调，全部落进同一个默认的
       `MutuallyExclusiveCallbackGroup`"，这意味着**不管`MultiThreadedExecutor`
       开了几个线程，同一个默认group里的回调之间永远是互斥串行执行的**，
       跟"多线程"这四个字面意思相反。
    2. 后果：如果在`mission_events`订阅的回调（收到`GROUND_FIRE_FOUND`
       之后触发）里直接调用`sdk.goto()`，`goto()`内部要靠**另一个订阅
       回调**（`waypoint_state`）把`self._waypoint_state`更新成
       `'completed'`才能返回——但这个`waypoint_state`订阅回调跟正在执行
       的`mission_events`回调用的是同一个默认callback group，被互斥锁住
       无法运行，`goto()`会永久卡在`_poll_until()`里等一个永远不会来的
       状态更新，直到超时抛异常——**这是一个必然会触发的自锲死锁，不是
       偶发的时序问题**。
    3. 结论：`on_teammate_event()`注册的回调**只能做"瞬间完成、不等待
       任何后续ROS2消息"的轻量操作**（比如把事件塞进一个普通Python
       list），真正的业务处理（调`goto()`/`do_action()`等阻塞方法）必须
       挪到调用方自己的主线程里执行——这正是这个类的设计：回调只做
       `_make_handler()`里那一行`list.append()`，主线程用`while True`
       + `time.sleep()`轮询`pop_all()`取出事件再处理，两者用一把
       `threading.Lock`保护，不会有竞态。

    **为什么不用"每个事件名对应一个`threading.Event`+等它被set"这种更
    简单的写法**（SUPPLY这边尤其考虑过）：SUPPLY在STANDBY阶段要同时等
    "地面火情"和"高层火情"两种互相独立、到达顺序不确定、且高层火情
    本身还要连续收两条消息（先`HR_FIRE_LOCKED`再`RECON_LAUNCH_DONE`）
    的事件，用一个通用队列+主循环里按当前状态分派，比维护好几个独立
    `Event`对象+处理"两个`Event`几乎同时被set"这种边界情况更简单、更
    不容易漏——这也是方案第5节风险清单第5条"双检测器/双事件到达顺序
    交错"这个坑在SUPPLY侧的对应解法（RECON侧的解法在
    `_spawn_detection_watcher()`那边单独说明，两边思路一致：都是"谁先来
    谁先处理，处理完检查还有没有没处理的"，不假设任何到达顺序）。

    **另一个必须让"回调只做轻量操作"成立的前提**：`reliability.py::
    _handle_event()`收到一个事件时，如果`event_name`在当前"没有注册任何
    处理回调"，会**永久丢弃**这个事件（打一条警告日志，ACK照常回，但
    业务回调不会被延迟触发/重放）——也就是说`register_event_handler()`
    必须在对方"可能发出这个事件"之前就注册好，不能指望"先注册晚了也
    没关系，反正消息会在queue里等着"。所以这个类的调用惯例是：**在角色
    主函数一开始、还没做任何可能触发对方发消息的动作之前，就把这次任务
    全程会用到的事件名一次性注册完**，不是"用到哪个事件再临时注册哪个"。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._items: List[Tuple[str, Dict[str, Any]]] = []

    def _make_handler(self, event_name: str) -> Callable[..., None]:
        def _cb(**kwargs: Any) -> None:
            # 全部逻辑只有这一行list.append()（受锁保护）——不做任何
            # 可能阻塞/等待的事，理由见类docstring第1-3点。
            with self._lock:
                self._items.append((event_name, kwargs))
        return _cb

    def register(self, sdk: DroneSDK, *event_names: str) -> None:
        for name in event_names:
            sdk.on_teammate_event(name, self._make_handler(name))

    def pop_all(self) -> List[Tuple[str, Dict[str, Any]]]:
        with self._lock:
            items, self._items = self._items, []
        return items

    def wait_for(self, event_name: str, timeout_s: float) -> Dict[str, Any]:
        """阻塞等待某一个特定事件名到达（用于"这一步就是专等这一个
        事件"的场景，比如RECON等`supply_ready`）——跟STANDBY循环走的是
        同一套"主线程自己sleep轮询pop_all()"机制，不是另开一套基于
        `threading.Event`的等待逻辑，避免同时维护两套并行的等待实现。

        如果轮询过程中弹出了不是这次在等的事件名，说明是"预注册了但这
        一刻还用不上"的其它事件（比如RECON在等`supply_ready`的同时，
        `pillar_selected`这类纯通知事件也可能混进来），这里选择先记下
        警告再丢弃，不放回队列重新排队——正常流程里不应该发生（每个
        等待点等的事件名跟当前阶段是对应好的），如果真的出现，更可能
        是选手/这份代码逻辑写错了事件名，打印出来方便发现，比悄悄吞掉
        更好。
        """
        deadline = time.monotonic() + timeout_s
        while True:
            for name, kwargs in self.pop_all():
                if name == event_name:
                    return kwargs
                print(f"[警告] 等待'{event_name}'期间收到了另一个事件"
                      f"'{name}'（kwargs={kwargs}），已丢弃——如果这不是"
                      f"预期内的纯通知事件，检查一下事件名有没有对错。")
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"等待队友事件'{event_name}'超过{timeout_s}秒仍未到达"
                    f"——检查队友程序是否还在运行、双机消息链路是否正常。"
                )
            time.sleep(STANDBY_POLL_INTERVAL_S)


def _spawn_detection_watcher(
    sdk: DroneSDK,
    class_id: str,
    hit_event: threading.Event,
    already_done_fn: Callable[[], bool],
) -> threading.Thread:
    """D4判断点③：两种火情检测"并行监听"的具体实现。

    **设计**：为`class_id`起一个独立的后台daemon线程，线程里只做一件事
    ——调用一次`sdk.wait_for_detection(class_id, timeout=一个很宽松的
    上限)`。这个调用本身就是"阻塞直到出现匹配的检测帧或者超时"，`_poll_
    until()`内部每0.2秒检查一次最新一帧缓存的检测结果——也就是说，只要
    我们起两个这样的线程（一个盯`apriltag:2`，一个盯`apriltag:1`），
    **不需要写任何轮询/重试循环**，`wait_for_detection()`自己的实现就已
    经是"持续监听直到命中"，两个线程各自独立阻塞在自己的`wait_for_
    detection()`调用里，天然就是"并行"的（不是"飞哪段航线就看哪种检测
    器"这种分段判断，两个线程从RECON进入D4开始就一直在跑，跟主线程正在
    飞地面段还是立柱段完全无关）。

    命中后：如果这种火情还没被处理过（`already_done_fn()`为`False`），
    就把`hit_event`置位，并调用`sdk.cancel_goto()`——`cancel_goto()`是
    "尽力而为、不阻塞等确认"的调用（见`capabilities.py::cancel_goto()`
    docstring），从这个后台线程直接调用是安全的，不违反"回调别做重活"
    这条规则（`cancel_goto()`本身就设计成不阻塞）。主线程正阻塞在
    `sdk.goto()`里的话，`waypoint_state`会变成`'cancelled'`，`goto()`
    会正常返回（不抛异常），主线程回来后通过检查`hit_event`发现"有新
    情况要处理"。

    **为什么每种火情只起一次性的单次`wait_for_detection()`调用，不是
    "命中后继续起下一轮监听"**：这次任务里每种火情只会被处理一次
    （`ground_fire_done`/`hr_fire_done`一旦置`True`就不会变回`False`），
    线程命中一次、把`hit_event`交给主线程处理之后，这个线程本身的
    使命就完成了，可以直接退出，不需要活到整个任务结束——两个真正决定
    "这种火情算不算处理完了"的标志位（`ground_fire_done`/`hr_fire_done`）
    始终由主线程维护、主线程读写，这里的`hit_event`只是"通知主线程有
    新命中"这一个单向信号，不是权威状态。

    Returns:
        创建的线程对象（主要是方便测试场景里`join()`它，正常任务流程
        不需要用这个返回值）。
    """

    def _watch() -> None:
        try:
            sdk.wait_for_detection(class_id, timeout=DETECTION_WATCH_TIMEOUT_S)
        except DetectionTimeoutError:
            # 整场任务期间都没等到——正常情况下不应该发生（两种火情都是
            # 题目保证存在的目标），打个警告方便复盘，不是代码异常。
            print(
                f"[警告] 后台检测线程等待'{class_id}'超过"
                f"{DETECTION_WATCH_TIMEOUT_S}秒仍未命中，该检测线程退出——"
                f"如果任务确实需要更长的搜索时间，调大DETECTION_WATCH_TIMEOUT_S。"
            )
            return
        if not already_done_fn():
            hit_event.set()
            sdk.cancel_goto()

    thread = threading.Thread(target=_watch, daemon=True, name=f'detect-{class_id}')
    thread.start()
    return thread


def _estimate_route_wait_seconds(sdk: DroneSDK) -> float:
    """D3判断点①：SUPPLY估算"RECON飞完第一部分航线大概需要多久"。

    **背景**：SUPPLY角色在D3只是被动跟随（`start_formation_follow()`
    之后不需要自己发航点），方案原文只说"RECON到达终点后自然停止移动，
    SUPPLY没有独立的完成信号"，没有给出具体的判断方案，这是任务要求
    我自己拍板的设计点。

    **考虑过的其它方案 + 为什么不用**：
    - 查询自己实时位置、判断是否已经贴近RECON终点附近稳定不动——
      `DroneSDK`没有暴露"查询自己当前位置"的公开方法（`_odom_xyz`是
      私有属性，任务要求不允许伸手进去读），行不通。
    - 订阅`mission_judge/checkpoints`（方案4.2节提到的权威确认手段，
      `DiagnosticArray`话题）——这是flight-stack发布的一个原生ROS2
      话题，`capabilities.py`没有把它封装成任何公开方法，选手侧订阅
      任意话题这件事本身就是任务要求不允许跨过的SDK边界（跟D6两个
      占位话题是同一类"SDK没覆盖到"的缺口，但这次不需要专门为它占位，
      因为下面这个方案已经够用）。

    **选定方案：固定等待时长，按路线几何长度+保守巡航速度估算，不是
    拍一个随便的数字**——RECON全程走的是D1的4个途经点（`mission_
    constants.ROUTE_WAYPOINTS_XY`），起点是RECON自己的起降点（用
    `mission_constants.own_landing_pad_xy(sdk.teammate_namespace)`查，
    这里"查队友的起降点坐标"跟D1文件允许的"查自己的起降点坐标"是同一
    条豁免——只是查一张已知坐标表，不是按namespace分支业务逻辑）。把
    这几段距离加起来，除以一个**故意悲观**的巡航速度假设
    （`ASSUMED_CRUISE_SPEED_MPS=0.4m/s`，明显比`ego_planner`实际巡航
    速度慢），再加一个固定安全余量（`SUPPLY_WAIT_SAFETY_MARGIN_S=60`秒，
    覆盖起飞爬升、避障重规划绕路、AGL贴地飞行变速这些没法精确建模的
    波动）——宁可等久一点再降落，也不要提前判断"飞完了"导致SUPPLY掉队
    （提前`stop_formation_follow()`+`land()`意味着后续物资投送/编队
    位置全部错位，代价远大于"多等半分钟"）。

    这不是一个精确的完成判据，是"用几何+保守假设算出来的、有明确依据
    的固定等待时长"，比"拍一个随便的固定数字（比如直接写死120秒）"更
    经得起路线本身改变时的检验——如果以后`ROUTE_WAYPOINTS_XY`换了更长
    的路线，这个函数会自动算出更长的等待时间，不需要手动同步调一个魔法
    数字。
    """
    start_xy = mc.own_landing_pad_xy(sdk.teammate_namespace)
    total_distance_m = 0.0
    prev_xy = start_xy
    for wp_xy in mc.ROUTE_WAYPOINTS_XY:
        total_distance_m += math.hypot(wp_xy[0] - prev_xy[0], wp_xy[1] - prev_xy[1])
        prev_xy = wp_xy
    return total_distance_m / ASSUMED_CRUISE_SPEED_MPS + SUPPLY_WAIT_SAFETY_MARGIN_S


# ============================================================================
# 坐标系边界转换（2026-09-13阶段D单机RECON实测发现的bug，补的两个helper）
# ============================================================================
#
# 背景：D1的`mission_constants.py`存的坐标全部是"抄自`fire_drill_room_
# layout.yaml`的世界坐标字面量"（方案0.4节/D1清单明确要求，不读yaml、
# 但数值就是yaml里的世界坐标）；`position_tracker`/`mission_geometry.py`
# 的几何计算（插值/绕飞/最近立柱选择）全部假设"大家在同一套世界坐标系
# 下算距离"，这个假设本身没错。但`sdk.goto()`的坐标系约定是**这架飞机
# 自己的局部坐标系**（`capabilities.py::goto()`docstring明确写"如果
# 选手手上是世界坐标，需要先调sdk.world_to_local()"），两架飞机局部系
# 原点不同（起降点相距几米）。第一版实现漏了这一步转换，实测复现过
# `GotoTimeoutError`（飞机被指向偏出去好几米的错误局部坐标，规划器
# 怎么也飞不到，见DEBUG_JOURNAL.md对应条目）。这两个helper把"世界坐标
# ->局部坐标"（给`sdk.goto()`用）和"局部坐标->世界坐标"（给跨机广播用，
# 比如`sdk.center_on_target()`/`sdk.read_fire_pillar_staging_pose()`
# 这类返回本机局部坐标的方法，广播前必须先转换）这两个方向的转换收拢
# 到一处，避免每个调用点各自记一遍"这里要不要转换"。
# ============================================================================

def _goto_world_xy(sdk: DroneSDK, world_x: float, world_y: float, world_z: float) -> None:
    """世界坐标(x, y, z) -> 本机局部坐标 -> `sdk.goto()`。D1的`mission_
    constants`/`mission_geometry`算出来的目标点都是世界坐标，任何要真
    正飞过去的地方都必须经过这一步转换，不能直接传给`goto()`。
    """
    local_x, local_y, local_z = sdk.world_to_local(world_x, world_y, world_z)
    sdk.goto(local_x, local_y, local_z)


def _local_xy_to_world_xy(sdk: DroneSDK, local_x: float, local_y: float) -> Tuple[float, float]:
    """本机局部坐标(x, y) -> 世界坐标(x, y)，跨机广播前必须转换的另一个
    方向。z统一传0.0占位——`sdk.local_to_world()`的x/y转换结果不受z
    影响（`origin_setter_node.py::_broadcast_and_persist()`里`tx`/`ty`
    只由x/y项算出，z是单独的高度差，见该文件说明），这里只关心水平
    坐标，调用方不需要关心z。
    """
    world_x, world_y, _ = sdk.local_to_world(local_x, local_y, 0.0)
    return world_x, world_y


# ============================================================================
# D3：第一部分任务——航线编队飞行
# ============================================================================

def _fly_route(sdk: DroneSDK, position_tracker: _PositionTracker) -> None:
    """RECON分支：依次飞过`mission_constants.ROUTE_WAYPOINTS_XY`的4个
    途经点，每个途经点直接一次`goto()`，不再自己按`MAX_GOTO_STEP_M`
    拿直线插值出中间子航点。

    2026-09-14订正（用户实测直接指出这个问题）：D1最初设计"自己按最大
    步长3米插值"是为了绕开`ego_planner`长距离目标卡`GEN_NEW_TRAJ`重
    规划死循环这个坑（方案4.2节），但`interpolate_waypoints()`是纯
    直线几何插值，完全不知道场地里有障碍物——`ROUTE_WAYPOINTS_XY`第
    一段`(7,-10)->(7,10)`是沿x=7这条直线走，`fire_drill_room_layout.
    yaml`里的`obstacle_cylinder`正好架在`(7.0, 0.0)`（这条直线正中间）、
    `terrain_module`架在`(7.0, -6.0)`（也在线上）——直线插值算出来的
    中间点会直接落在障碍物里，`ego_planner`的A*连起点/终点都解析不了，
    实测复现过`the drone is in obstacle`刷屏。这是"自己抄近路直线切"
    这个思路本身的问题，插值算法实现没有bug。正确做法是把整段
    `(7,-10)->(7,10)`原样交给`ego_planner`自己的全局A*+局部B样条规划
    —— 规划器本来就该负责"沿途怎么绕开障碍物"这件事，选手程序不该越权
    替它把路线拆成看似更"安全"、实际上更危险的直线小段。`MAX_GOTO_
    STEP_M`/`interpolate_waypoints()`两个仍然保留在`mission_constants.
    py`/`mission_geometry.py`里（后者有独立单元测试覆盖，纯几何函数
    本身没有问题），只是这个函数不再调用它们——万一以后有其它场景
    （比如两点间确定没有障碍物、只是单纯距离太远）需要拆分步长，仍然
    可以复用。
    """
    for wp_xy in mc.ROUTE_WAYPOINTS_XY:
        _goto_world_xy(sdk, wp_xy[0], wp_xy[1], mc.CRUISE_AGL_M)
        position_tracker.update(wp_xy)


# ============================================================================
# D5：地面火情响应子流程（方案4.3.1节）
# ============================================================================

def _recon_handle_ground_fire(sdk: DroneSDK, position_tracker: _PositionTracker) -> None:
    """RECON侧：命中'apriltag:2'之后，居中定位+广播坐标给SUPPLY。"""
    pose = sdk.center_on_target(GROUND_FIRE_CLASS_ID, timeout=CENTER_ON_TARGET_TIMEOUT_S)
    # `pose.x/pose.y`是RECON自己的局部坐标（见sdk.center_on_target()
    # 2026-09-13订正后的文档），跨机广播前必须先换算成世界坐标——SUPPLY
    # 收到之后会再用它自己的sdk.world_to_local()换算回SUPPLY自己的局部
    # 坐标，两架飞机局部系原点不同，不能直接传局部坐标数值。
    world_x, world_y = _local_xy_to_world_xy(sdk, pose.x, pose.y)
    # D7 第7点（必需的跨机协同事件，方案4.1/4.3.1节）。
    sdk.send_to_teammate(mv.GROUND_FIRE_FOUND, x=world_x, y=world_y)
    # 居中完成后飞机基本悬停在目标正上方，用这个坐标顺手刷新一下位置
    # 追踪器（D6的立柱最近距离选择会用到，`position_tracker`统一维护
    # 世界坐标，见_PositionTracker docstring）。
    position_tracker.update((world_x, world_y))


def _supply_handle_ground_fire(sdk: DroneSDK, x: float, y: float, **_ignored_kwargs: Any) -> None:
    """SUPPLY侧：收到`GROUND_FIRE_FOUND`之后，去物资点精降抓取，再飞去
    地面火情坐标投放。`**_ignored_kwargs`兜住以后万一多带别的字段，不
    因为参数不匹配直接报错。

    `x/y`是RECON发来的**世界坐标**（RECON侧`_recon_handle_ground_fire()`
    发送前已经用`local_to_world()`换算过），这里直接传给`_goto_world_xy()`
    再转成SUPPLY自己的局部坐标即可，不需要（也不能）再假设它是SUPPLY
    的局部坐标。
    """
    _goto_world_xy(sdk, mc.SUPPLY_POINT_XY[0], mc.SUPPLY_POINT_XY[1], mc.CRUISE_AGL_M)
    sdk.precision_land_and_confirm(mc.SUPPLY_POINT_APRILTAG_ID, timeout=PRECISION_LAND_TIMEOUT_S)
    sdk.do_action(mv.GRAB_SUPPLY)
    sdk.send_to_teammate(_NOTIFY_SUPPLY_GRABBED)  # D7 第8点（纯通知）
    # 精降落地抓取后必须重新起飞才能继续投放——这条链路依赖`CONTROLLER=
    # pt4ctrl`支持反复起降（方案1.7节/A5验收标准2已经确认过），这里
    # 直接调用，不需要额外判断。
    sdk.takeoff(timeout=GRAB_TAKEOFF_RETRY_TIMEOUT_S)
    _goto_world_xy(sdk, x, y, mc.CRUISE_AGL_M)
    sdk.do_action(mv.DROP_SUPPLY)  # 投放不需要落地，悬停触发即可（方案4.3.1节）
    sdk.send_to_teammate(_NOTIFY_SUPPLY_DROPPED)  # D7 第9点（纯通知）


# ============================================================================
# D6：立柱绕飞+高层火情响应子流程（方案4.3.2节）
# ============================================================================

def _recon_handle_hr_fire(sdk: DroneSDK, inbox: _EventInbox) -> None:
    """RECON侧：命中`apriltag:1`（`fire_pillar_aim_node`已经自动锁定+
    接管`pillar_aim`模式，这里不需要重新实现锁定逻辑，方案4.3.2节已经
    说明这一点）之后的完整响应链路：广播等待点 -> 等SUPPLY到位 -> 水平
    投射 -> 广播瞄准点 -> reset_aim -> 交还调用方标记`hr_fire_done`。

    2026-09-13补：等待点/瞄准点坐标原来是"没有对应SDK能力，用几何近似
    顶上"的占位实现，现在`contest_sdk`已经补上`read_fire_pillar_
    staging_pose()`/`read_fire_pillar_aim_pose()`这两个真实能力（直接
    读`fire_pillar_aim_node`广播的话题），不再需要`pillar_cx`/`pillar_
    cy`/`current_xy`这几个只是为了算几何近似才需要的参数，函数签名跟着
    简化。
    """
    # D7 第12点（纯通知——"高层火情被发现"，跟第13点"等待点广播"是两个
    # 不同粒度的通知，这里先发一条轻量的"发现了"信号）。
    sdk.send_to_teammate(_NOTIFY_HR_FIRE_DETECTED)

    staging_x, staging_y, staging_yaw = sdk.read_fire_pillar_staging_pose(
        timeout=FIRE_PILLAR_POSE_READ_TIMEOUT_S
    )
    # `read_fire_pillar_staging_pose()`返回RECON自己的局部坐标（见该
    # 方法docstring），跨机广播前必须先换算成世界坐标，理由跟D5的
    # `_recon_handle_ground_fire()`完全一样。yaw不做换算——`uwb_imu`
    # 模式下两机局部系天然同向（θ*≈0，方案4.2节订正已确认），跳过
    # 旋转分量的换算在这个模式下没有实际误差，不是遗漏。
    staging_world_x, staging_world_y = _local_xy_to_world_xy(sdk, staging_x, staging_y)
    # D7 第13点（必需的跨机协同事件）。
    sdk.send_to_teammate(
        mv.HR_FIRE_LOCKED, staging_x=staging_world_x, staging_y=staging_world_y, staging_yaw=staging_yaw
    )

    # 等SUPPLY确认已经飞到等待点（D7第14点由SUPPLY那边发出，这里只是等）。
    inbox.wait_for(mv.SUPPLY_READY, timeout_s=SUPPLY_READY_WAIT_TIMEOUT_S)

    sdk.do_action(mv.HORIZONTAL_LAUNCH)
    aim_x, aim_y, aim_yaw = sdk.read_fire_pillar_aim_pose(
        timeout=FIRE_PILLAR_POSE_READ_TIMEOUT_S
    )
    aim_world_x, aim_world_y = _local_xy_to_world_xy(sdk, aim_x, aim_y)
    # D7 第15点（必需的跨机协同事件）。
    sdk.send_to_teammate(mv.RECON_LAUNCH_DONE, aim_x=aim_world_x, aim_y=aim_world_y, aim_yaw=aim_yaw)
    sdk.reset_aim()


def _supply_handle_hr_fire_locked(
    sdk: DroneSDK, staging_x: float, staging_y: float, staging_yaw: float = 0.0, **_ignored: Any
) -> None:
    """SUPPLY侧：收到`HR_FIRE_LOCKED`，飞到等待点+广播就位信号。

    `staging_x/staging_y`是RECON广播的**世界坐标**（RECON侧发送前已经
    用`local_to_world()`换算过），这里用`_goto_world_xy()`换算成SUPPLY
    自己的局部坐标再飞。
    """
    _goto_world_xy(sdk, staging_x, staging_y, mc.CRUISE_AGL_M)
    sdk.send_to_teammate(mv.SUPPLY_READY)  # D7 第14点（必需的跨机协同事件）


def _supply_handle_recon_launch_done(
    sdk: DroneSDK, aim_x: float, aim_y: float, aim_yaw: float = 0.0, **_ignored: Any
) -> None:
    """SUPPLY侧：收到`RECON_LAUNCH_DONE`，飞到RECON给的瞄准点+水平投射。

    ⚠️注意这个函数**不**在这里调用`sdk.land()`——D6原始流程描述里SUPPLY
    投射完直接飞回起降点降落，但这份实现把"最终降落"统一挪到D4主循环
    "两个火情标志位都True之后"才做（见`supply_main()`里"D4收尾同步"
    注释块），理由是地面火情/高层火情谁先发生不做假设：如果高层火情
    先发生、地面火情还没找到，这里如果直接降落，SUPPLY会在地面火情
    需要它响应的时候已经落地退出了任务循环——挪到"两个都做完才降落"
    能正确处理这种到达顺序颠倒的情况（方案第5节风险清单第5条提到的
    "双检测器到达顺序交错"问题，这里是它在SUPPLY侧、跨两个事件的对应
    表现）。

    `aim_x/aim_y`同样是RECON广播的世界坐标，用`_goto_world_xy()`换算。
    """
    _goto_world_xy(sdk, aim_x, aim_y, mc.CRUISE_AGL_M)
    sdk.send_to_teammate(_NOTIFY_SUPPLY_ARRIVED_AIM_POINT)  # D7 第17点（纯通知）
    sdk.do_action(mv.HORIZONTAL_LAUNCH)
    sdk.send_to_teammate(_NOTIFY_SUPPLY_LAUNCH_DONE)  # D7 第18点（纯通知）


# ============================================================================
# D4：双火情响应总控（方案4.3节状态机骨架）
# ============================================================================

def _run_search_and_response(
    sdk: DroneSDK, position_tracker: _PositionTracker, inbox: _EventInbox
) -> Tuple[bool, bool]:
    """RECON分支的D4主循环：弓字形地面航段 + 3根立柱依次绕飞航段，两个
    检测器全程并行监听，命中就打断+响应+恢复剩余航段。

    Returns:
        `(ground_fire_done, hr_fire_done)`——正常情况下两个都应该是
        `True`，如果搜索航线飞完了还有没找到的（题目理论上保证两种
        火情都存在，这属于不应该发生的边界情况），调用方只打印警告、
        不抛异常卡死整个任务（见函数末尾的说明）。
    """
    ground_fire_hit = threading.Event()
    hr_fire_hit = threading.Event()
    ground_fire_done = False
    hr_fire_done = False

    # ---- D4判断点③：两个检测器"并行监听"的落地实现 ----
    # 具体设计理由见`_spawn_detection_watcher()`的docstring，这里只是
    # 调用——两个线程从进入D4的这一刻起就独立运行，不区分"当前正在飞
    # 哪一段"，满足任务要求的"不是分段判断，两个检测器全程并行"。
    _spawn_detection_watcher(sdk, GROUND_FIRE_CLASS_ID, ground_fire_hit, lambda: ground_fire_done)
    _spawn_detection_watcher(sdk, HR_FIRE_CLASS_ID, hr_fire_hit, lambda: hr_fire_done)

    def handle_pending() -> None:
        """检查两个命中标志位，命中且还没处理过的就地处理。

        **处理顺序固定为"先地面、后高层"**——这是应对方案第5节风险清单
        第5条"处理A事件时刚好又检测到B事件"这种交错场景的关键：不管
        两个`threading.Event`是几乎同时被后台线程置位、还是有先后，这
        个函数每次被调用时都按同一个固定顺序检查+处理，保证行为是
        确定性的、可复现的，不依赖两个后台线程谁先抢到GIL/谁先执行到
        `.set()`这种线程调度细节。

        `ground_fire_done`/`hr_fire_done`这两个布尔值只在**这个函数**
        （运行在主线程里）里被赋值，`_spawn_detection_watcher()`传进去
        的判断闭包只读它们——单个属性/局部变量的整体读写在CPython下是
        原子的，这个约定跟`capabilities.py`模块头"单个属性的整体赋值
        是原子的"那条说明是同一个道理，这里不用额外加锁。
        """
        nonlocal ground_fire_done, hr_fire_done
        if ground_fire_hit.is_set() and not ground_fire_done:
            ground_fire_hit.clear()
            _recon_handle_ground_fire(sdk, position_tracker)
            ground_fire_done = True
        if hr_fire_hit.is_set() and not hr_fire_done:
            hr_fire_hit.clear()
            _recon_handle_hr_fire(sdk, inbox)
            hr_fire_done = True

    # ---- 弓字形地面航段 ----
    sdk.set_mission_state('searching:ground_scan')
    # 2026-09-17：按航段动态开关相机这个优化已经整体删除（见
    # `set_camera_enabled()`删除时的说明）——front/down两路全程都开着，
    # 两个检测器全程并行监听，不再区分航段。
    room_min_x = -ROOM_SIZE_X_M / 2.0
    room_max_x = ROOM_SIZE_X_M / 2.0
    room_min_y = -ROOM_SIZE_Y_M / 2.0
    room_max_y = ROOM_SIZE_Y_M / 2.0
    ground_scan_waypoints = sdk.generate_ground_scan_waypoints(
        room_min_x, room_max_x, room_min_y, room_max_y,
        altitude_agl=mc.CRUISE_AGL_M, wall_margin=ROOM_WALL_MARGIN_M,
    )

    i = 0
    while i < len(ground_scan_waypoints) and not (ground_fire_done and hr_fire_done):
        handle_pending()
        if ground_fire_done and hr_fire_done:
            break
        wp = ground_scan_waypoints[i]
        _goto_world_xy(sdk, wp[0], wp[1], wp[2])
        # 在`handle_pending()`（会清掉命中标志位）之前先探一眼有没有
        # 命中——用来判断"刚才这次goto()是正常到达还是被打断"，决定要
        # 不要把这个航点当作"已经飞完"（见下面if判断的注释）。
        interrupted = ground_fire_hit.is_set() or hr_fire_hit.is_set()
        position_tracker.update((wp[0], wp[1]))
        handle_pending()
        if not interrupted:
            i += 1
        # 如果被打断（不管是哪种火情），不递增i——下一轮循环会重新尝试
        # 飞往同一个航点，这就是任务要求的"恢复剩余航段，从打断的地方
        # 接着飞，不是重新从头开始"的具体实现（`goto()`本身不区分"到达"
        # 和"被取消"两种正常返回，这里用"返回时命中标志位是否被置位"
        # 这个时序上的代理信号来判断，存在极小概率的边界误判——比如
        # 命中恰好发生在goto()自然到达的同一瞬间——但即便判断错了，最坏
        # 后果只是把这一个航点多飞一次，不影响正确性）。

    # ---- 3根立柱依次绕飞航段 ----
    pillar_sorted = mg.sort_pillars_deterministic(mc.CANDIDATE_PILLARS_XY)
    visited_pillar_indices: List[int] = []

    while len(visited_pillar_indices) < len(pillar_sorted) and not hr_fire_done:
        handle_pending()
        if hr_fire_done:
            break
        pillar_idx = mg.select_nearest_pillar_index(
            position_tracker.xy, pillar_sorted, visited_indices=visited_pillar_indices
        )
        cx, cy = pillar_sorted[pillar_idx]
        sdk.send_to_teammate(_NOTIFY_PILLAR_SELECTED, pillar_index=pillar_idx, cx=cx, cy=cy)  # D7 第10点
        sdk.set_mission_state(f'searching:orbit_pillar_{pillar_idx}')  # D7 第11点

        # 2026-09-15用户要求"机头始终对准立柱中心"：绕这根立柱之前把
        # yaw旁路开关切到POINT模式，目标点是这根立柱的中心——
        # set_yaw_mode_point()要局部坐标（跟sdk.goto()同一套坐标系），
        # 不能直接传cx/cy这两个世界坐标，先转一次。开关是traj_server
        # 进程级全局状态，绕完/命中火情都必须切回默认状态（见下面
        # finally风格的收尾），不能让"这次该看立柱"这个临时状态漏到
        # 之后的返航等普通飞行段。
        # 2026-09-16订正：收尾切回的目标从VELOCITY改成CONSTANT(0.0)——
        # 用户实测发现"朝速度方向"这套算法在急转弯时会跟位置/高度控制
        # 抢电机推力分配资源，仿真掉高、真机炸机过，`traj_server`的默认
        # 值已经从VELOCITY改成CONSTANT（见ego_planner_traj_server_yaw_
        # mode.patch），这里收尾也要跟着改成CONSTANT，不能收尾成一个
        # 已经不再是"安全默认值"的VELOCITY模式。
        local_cx, local_cy, _ = sdk.world_to_local(cx, cy, mc.PILLAR_ORBIT_Z_AGL_M)
        sdk.set_yaw_mode_point(local_cx, local_cy)

        orbit_waypoints = sdk.generate_orbit_waypoints(
            cx, cy, radius=mc.PILLAR_ORBIT_RADIUS_M, z=mc.PILLAR_ORBIT_Z_AGL_M,
            num_points=mc.PILLAR_ORBIT_NUM_POINTS,
        )
        # 补上"回到起点"这一段，保证整整绕一圈、不留缺口（`generate_
        # orbit_waypoints()`文档明确说明首尾不重复，需要调用方自己补）。
        orbit_waypoints = list(orbit_waypoints) + [orbit_waypoints[0]]

        try:
            j = 0
            while j < len(orbit_waypoints) and not hr_fire_done:
                handle_pending()
                if hr_fire_done:
                    break
                wp = orbit_waypoints[j]
                _goto_world_xy(sdk, wp[0], wp[1], wp[2])
                interrupted = ground_fire_hit.is_set() or hr_fire_hit.is_set()
                position_tracker.update((wp[0], wp[1]))
                handle_pending()
                if not interrupted:
                    j += 1
        finally:
            # 2026-09-17订正：不再写死0.0——`sdk.takeoff()`已经把起飞前
            # 真实yaw角记在`sdk.pretakeoff_yaw`里，绕飞收尾应该恢复到
            # 这个值，不是任意一个固定角度（0.0跟飞机实际停机朝向对
            # 不上是"起飞后掉高"这个问题的根因，详见`capabilities.py`
            # 里`takeoff()`的docstring）。
            sdk.set_yaw_mode_constant(sdk.pretakeoff_yaw)

        visited_pillar_indices.append(pillar_idx)

        # 2026-09-15用户新增要求：这根立柱绕完了（不管是自然绕完一圈还是
        # 被中途打断），如果打断原因不是"这根立柱命中了高层火情"（可能是
        # 命中了地面火情、或者单纯绕完一圈没找到），要明确告诉队友"这根
        # 立柱查完了、没在这根上发现高层火情"，不能静默换下一根——不然
        # 队友完全不知道RECON进度，D7 20点表格要求的"检测不到就别装作
        # 什么都没发生"同一个精神，这里是"查完了没找到也要说一声"。
        # 用`hr_fire_done`（不是`hr_fire_hit`）判断——`handle_pending()`
        # 每次调用都会把命中的`hr_fire_hit`立刻`.clear()`掉、改记到
        # `hr_fire_done`这个不会变回False的持久标志位上，绕飞循环内部
        # 每一轮都调用过`handle_pending()`，退出循环这一刻`hr_fire_hit`
        # 几乎必然已经被清空，只有`hr_fire_done`能可靠反映"这次绕飞有没有
        # 真的命中过"。
        if not hr_fire_done:
            sdk.send_to_teammate(_NOTIFY_PILLAR_CLEARED, pillar_index=pillar_idx, cx=cx, cy=cy)

    if not (ground_fire_done and hr_fire_done):
        # 题目保证两种火情都存在，理论上不应该走到这里——如果真的发生
        # （比如场景配置有问题、检测器没触发），不抛异常卡死整个任务，
        # 只打印警告，让RECON带着"至少完成了搜索航线"这个结果继续往下
        # 走（D4主流程末尾会再等SUPPLY，如果SUPPLY那边也卡在等一个永远
        # 不会来的事件，那是另一层问题，不是这个函数的职责）。
        print(f'[警告] 搜索航线已经飞完，但ground_fire_done={ground_fire_done}、'
              f'hr_fire_done={hr_fire_done}，至少有一种火情没有被发现——'
              f'检查场景配置/检测器是否正常。')

    return ground_fire_done, hr_fire_done


# ============================================================================
# 角色主函数
# ============================================================================

def recon_main(sdk: DroneSDK) -> None:
    """RECON角色的完整任务流程（D3+D4，D5/D6作为子流程被D4调用）。"""
    # D4判断点②同一套"提前注册"原则在RECON侧的应用——`supply_ready`/
    # `supply_landed`这两个事件都是SUPPLY发的，注册时机要在SUPPLY可能
    # 发出它们之前，最安全的做法就是任务一开始就注册好（见`_EventInbox`
    # docstring最后一段）。这里额外把SUPPLY侧的4个纯通知事件也注册成
    # 会被塞进收件箱、但主循环里不特别处理的"已知但忽略"事件，避免
    # `reliability.py`打"没有注册处理回调"的警告日志噪音。
    inbox = _EventInbox()
    inbox.register(sdk, mv.SUPPLY_READY, _NOTIFY_SUPPLY_LANDED, *_SUPPLY_ORIGINATED_PURE_NOTIFY_EVENTS)

    sdk.takeoff()  # 现在内部已经等到位置稳定才返回，见其docstring
    sdk.set_mission_state('enroute:takeoff_done')  # D7 第1点

    pad_xy = mc.own_landing_pad_xy(sdk.namespace)
    position_tracker = _PositionTracker(pad_xy)

    _fly_route(sdk, position_tracker)
    sdk.set_mission_state('enroute:route_completed')  # D7 第4点（RECON视角）

    sdk.set_mission_state('searching:fire_response_start')  # D7 第6点
    ground_fire_done, hr_fire_done = _run_search_and_response(sdk, position_tracker, inbox)
    if not (ground_fire_done and hr_fire_done):
        print('[警告] recon_main：搜索阶段结束但火情标志位不全为True，'
              '仍然继续走完剩余流程（等SUPPLY确认降落后返航）。')

    # ---- D4收尾同步："两个都True且SUPPLY也确认降落后"才返航 ----
    # 方案4.3节D4原文只说"两个标志位都True且两机都已降落后，RECON飞回
    # 起降点"，但没有给出RECON怎么知道"SUPPLY已经降落"的具体机制——
    # `sdk.get_mission_state()`只能读自己这个DroneSDK实例设置过的状态，
    # 读不到队友那边的`mission_state`（两架飞机的`mission_state`话题各
    # 自独立，SDK没有暴露"查队友mission_state"这个方法），所以这里额外
    # 定义了一个跨机同步事件`_NOTIFY_SUPPLY_LANDED`（不在方案4.4节20点
    # 表格里，是这次实现补的同步信号，不是重复定义D7的点）：SUPPLY真正
    # 落地后广播这个事件，RECON在这里阻塞等它，这样"SUPPLY也确认降落"
    # 这句话才有一个具体、可靠的实现，不是靠猜时间。
    inbox.wait_for(_NOTIFY_SUPPLY_LANDED, timeout_s=SUPPLY_LANDED_WAIT_TIMEOUT_S)

    pad_x, pad_y = mc.own_landing_pad_xy(sdk.namespace)
    _goto_world_xy(sdk, pad_x, pad_y, mc.CRUISE_AGL_M)
    sdk.land()
    sdk.set_mission_state('done:recon_landed')  # D7 第16点
    sdk.set_mission_state('done:mission_complete')  # D7 第20点


def supply_main(sdk: DroneSDK) -> None:
    """SUPPLY角色的完整任务流程（D3编队跟随+D4 STANDBY循环，D5/D6子流程
    的SUPPLY侧处理函数已经拆到模块级函数里，这里只负责分派）。
    """
    # 跟recon_main()同样的"提前注册"原则——SUPPLY要处理的3个核心协同
    # 事件，加上RECON侧发的2个纯通知事件（避免警告日志噪音）。
    inbox = _EventInbox()
    inbox.register(
        sdk, mv.GROUND_FIRE_FOUND, mv.HR_FIRE_LOCKED, mv.RECON_LAUNCH_DONE,
        *_RECON_ORIGINATED_PURE_NOTIFY_EVENTS,
    )

    sdk.takeoff()  # 现在内部已经等到位置稳定才返回，见其docstring
    sdk.set_mission_state('enroute:takeoff_done')  # D7 第1点

    # ---- D3判断点①：编队跟随，直到RECON"大概"飞完全程 ----
    # `uwb_imu`模式下不需要等待任何θ*收敛判据（方案4.2节订正），起飞
    # 完成即可直接启用编队跟随。
    sdk.start_formation_follow(follow_distance_m=3.5)
    sdk.set_mission_state('enroute:formation_follow_started')  # D7 第2点

    wait_s = _estimate_route_wait_seconds(sdk)
    print(f'[{sdk.namespace}] 编队跟随中，按路线几何长度估算RECON飞完全程'
          f'大概需要{wait_s:.1f}秒，SUPPLY将等待这么久再停止跟随+降落'
          f'（具体估算依据见_estimate_route_wait_seconds()docstring）。')
    time.sleep(wait_s)

    sdk.set_mission_state('enroute:formation_follow_assumed_complete')  # D7 第4点（SUPPLY视角，近似判定）
    sdk.stop_formation_follow()
    sdk.land()
    sdk.set_mission_state('idle:standby_after_route')  # D7 第5点

    # ---- D4：STANDBY循环 ----
    # `hr_stage`用一个小状态机（None -> 'locked' -> 'launch_done'）串起
    # 高层火情响应需要连续处理的两条消息（`HR_FIRE_LOCKED`然后`RECON_
    # LAUNCH_DONE`），跟`ground_fire_done`这个单事件布尔标志位是两种不同
    # 的粒度，分开管理更清楚（D4判断点②的具体状态管理落地）。
    ground_fire_done = False
    hr_stage: Optional[str] = None  # None / 'locked' / 'launch_done'

    while not (ground_fire_done and hr_stage == 'launch_done'):
        for event_name, kwargs in inbox.pop_all():
            if event_name == mv.GROUND_FIRE_FOUND and not ground_fire_done:
                _supply_handle_ground_fire(sdk, **kwargs)
                ground_fire_done = True
            elif event_name == mv.HR_FIRE_LOCKED and hr_stage is None:
                _supply_handle_hr_fire_locked(sdk, **kwargs)
                hr_stage = 'locked'
            elif event_name == mv.RECON_LAUNCH_DONE and hr_stage == 'locked':
                _supply_handle_recon_launch_done(sdk, **kwargs)
                hr_stage = 'launch_done'
            elif event_name in _RECON_ORIGINATED_PURE_NOTIFY_EVENTS:
                # D7第10/12点：纯通知，SUPPLY不需要处理，忽略即可（能走
                # 到这个分支说明_EventInbox.register()已经注册过，不会
                # 触发"没有处理回调"的警告日志）。
                pass
            else:
                # 正常流程不应该走到这里（比如同一个事件在已经处理过之后
                # 又收到一次，或者事件到达顺序跟当前hr_stage不匹配）——
                # 打印出来方便发现问题，不静默吞掉，也不因此崩溃。
                print(f"[警告] STANDBY循环收到意外/重复事件'{event_name}'"
                      f"（当前ground_fire_done={ground_fire_done}, "
                      f"hr_stage={hr_stage!r}），已忽略：{kwargs}")
        time.sleep(STANDBY_POLL_INTERVAL_S)

    # ---- D4收尾同步：两个都做完了，真正返航降落 ----
    # 理由见`_supply_handle_recon_launch_done()`docstring——不在处理完
    # 某一个子流程时就单独降落，统一挪到这里，保证不管两种火情谁先
    # 发生，SUPPLY都只会在"两个都做完"之后降落一次。
    pad_x, pad_y = mc.own_landing_pad_xy(sdk.namespace)
    _goto_world_xy(sdk, pad_x, pad_y, mc.CRUISE_AGL_M)
    sdk.land()
    sdk.set_mission_state('done:supply_landed')  # D7 第19点
    # 通知RECON"我确认降落了"——见recon_main()里"D4收尾同步"注释块的
    # 说明，这是这次实现补的跨机同步信号，不是方案4.4节20点表格的重复。
    sdk.send_to_teammate(_NOTIFY_SUPPLY_LANDED)


# ============================================================================
# 主程序骨架（D2）
# ============================================================================

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='2026大赛contest task全流程任务程序')
    parser.add_argument('--namespace', required=True, help="这架飞机的命名空间（比如'NX01'）")
    parser.add_argument('--role', required=True, choices=('recon', 'supply'), help='这次运行分配到的角色')
    parser.add_argument('--teammate-namespace', required=True, dest='teammate_namespace', help='队友那架飞机的命名空间')
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    sdk = DroneSDK(namespace=args.namespace, role=args.role, teammate_namespace=args.teammate_namespace)
    try:
        # 顶层按角色分支（方案4.3节"代码结构提示"/1.5节双机对称性原则的
        # 硬性要求）——两架飞机跑同一份代码，只是启动参数`--role`不同，
        # 全文件除了`own_landing_pad_xy(sdk.namespace)`这一处D1允许的例外
        # 之外，没有任何地方按namespace分支业务逻辑。
        if sdk.role == 'recon':
            recon_main(sdk)
        else:
            supply_main(sdk)
    except ContestSdkError as exc:
        # 兜住所有contest_sdk自己的异常类型，打印一条清楚的中文提示再
        # 重新抛出——不吞掉异常（这份任务程序失败了就应该让人看见失败
        # 原因，跟`trigger_alarm()`那种"故意不抛"是完全不同的场景，见
        # 方案2.3节"链路D"的说明，那条豁免只适用于声光装置这一个方法）。
        print(f'[{sdk.namespace}] 任务程序出错终止：{exc!r}')
        raise
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()
