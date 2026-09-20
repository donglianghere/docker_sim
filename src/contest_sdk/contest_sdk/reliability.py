"""跨机通信可靠性层（对应方案2.3节"链路B"：双机之间的消息）。

**这一层要解决什么问题**：`gcs/backend/app.py`里`_publish_takeoff_land()`函数的注释
早就写过一条血泪教训——"连发3次而不是一次，因为`ros2 topic pub --once`在DDS
discovery没跟上时可能白发"；同一个文件的`_recent_px4ctrl_feedback()`则是"读
`docker logs`确认指令到底有没有被真正执行"。这两个函数共同证明这个项目的开发者
早就意识到**"指令/消息发出去 ≠ 对方真的收到了/处理了"**，只是当年的应对方式很
朴素（连发几次 + 读日志）。这个模块把同样的思路做得更系统、更适应"选手程序在
另一台机器上跑，压根访问不到对方容器日志"的真实部署场景：纯ROS2话题层面的
应用层ACK + 固定频率重发协议，不依赖`docker logs`、不依赖任何机器互相能docker
exec到对方。

**协议设计（跟延迟量级无关，是通用可靠性设计，方案2.3节原话）**：
1. 纯topic发布是"尽力而为"——DDS的RELIABLE QoS只保证已匹配订阅者之间不丢包，
   对方那一刻掉线照样永久丢失且发布方不知道，所以裸topic发布不够。
2. 应用层ACK+重发协议：每条事件带唯一ID，发送方以固定低频率（1Hz）持续重发，
   直到收到对方回发的ack或者超时；接收方按ID去重，避免同一个事件因为对方还
   没收到ack、持续重发，被接收方重复处理业务逻辑多次。
3. `mission_events`话题QoS用`RELIABLE`+足够的`history depth`（这里默认10），
   可选加`TRANSIENT_LOCAL`durability，这样进程重启重新订阅时也能拿到最近几条
   历史事件。

**延迟预算（为什么超时给到30-60秒这么宽松，方案2.3节"延迟预算"一节的结论）**：
双机之间走的是"飞机A→路由器→飞机B"两跳WiFi，实测延迟通常是几毫秒到几十毫秒
量级，比GCS-飞机那条链路还更干净。但这套ACK+1Hz重发协议本身**跟延迟量级无关**
——不管单程延迟是5毫秒还是500毫秒，协议都能工作，只是"确认送达"这件事花的
时间长短不同。真正需要关注的是给`send_and_wait_ack()`一个够宽松的总超时，
覆盖"正常WiFi延迟 + 一次路由器短暂抖动重连"这种情况，不能设得太紧张导致
正常场景也频繁误报超时——所以默认超时取方案建议区间（30-60秒）的中间值
45秒，同时把这个值做成调用方可覆盖的参数。

**这一层对上层（B3 `capabilities.py`）暴露的语义**：选手完全无感，`send_to_teammate()`
等方法内部调用这里的`ReliableEventChannel.send_and_wait_ack()`，对上层就是一句
"阻塞直到确认送达，超时抛`TeammateUnreachableError`"——选手看不到ID/ack/重发这些
协议细节。

**跟B2 `_rclpy_runtime.py`同样的设计原则**：这个模块不自己调`rclpy.init()`，也不
自己管理rclpy生命周期，由调用方（B3）传入一个已经初始化好、具备`create_publisher`/
`create_subscription`/`create_timer`/`get_logger`标准方法的rclpy Node对象。

**关于"等待期间谁来处理ROS2回调"这一点，跟B2的`RclpyRuntime`是强绑定的假设**：
`RclpyRuntime`把传出来的`node`放在一个专职的后台线程里，用`MultiThreadedExecutor`
持续`spin()`——也就是说本模块创建的订阅回调（收到队友的ack/事件）、定时器回调
（1Hz重发）**已经在被后台线程持续处理**，`send_and_wait_ack()`阻塞等待期间只需要
在**调用方自己的线程**里简单`sleep`轮询"有没有等到ack"就够了，绝对不能在等待循环
里自己再调用`rclpy.spin_once(node, ...)`——那样会跟后台正在跑的`MultiThreadedExecutor`
抢同一个节点，触发`RuntimeError: Executor is already spinning`（这不是理论推测，
是本模块开发过程中真实用两个rclpy节点做烟雾测试时踩到、又用单节点自环验证过
"改成sleep之后一切正常"的坑）。这也正好对应`_rclpy_runtime.py`文件头注释里
"两个线程各司其职，互不干扰：一个专职spin，阻塞方法留在调用方自己的线程里用
threading.Event/轮询等待"这条设计说明——本模块的等待循环就是那个"调用方自己的
线程"该做的事。

**可测试性设计**：真实ROS2话题收发被隔离在`_setup_ros_transport()`里，且只有在
真的传入了`node`时才会`import rclpy`相关符号——这样单元测试完全可以用
`node=None`+自己注入的`publish_fn`/`wait_tick_fn`来模拟一个"丢包/延迟可控"的
假网络层，测试ACK+重发+去重这套协议逻辑本身，不需要依赖真实DDS通信、不需要
真实ROS2环境（对应方案第5节"先用synthetic丢包/延迟场景测试，不要直接上真实
网络环境验证"这条建议）。见同目录`test_reliability.py`。
"""

import collections
import itertools
import json
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from contest_sdk.exceptions import TeammateUnreachableError

# ---------------------------------------------------------------------------
# 协议相关的默认参数，全部来自方案2.3节"链路B"的具体建议值，允许调用方按需覆盖。
# ---------------------------------------------------------------------------

#: `mission_events`话题名（不带命名空间前缀，实际发布/订阅时会拼上各自namespace）。
DEFAULT_TOPIC_NAME = "mission_events"

#: 发送方重发频率——方案建议"固定低频率（1Hz）持续重发"。
DEFAULT_RESEND_INTERVAL_S = 1.0

#: `send_and_wait_ack()`默认总超时——方案给的建议区间是30-60秒，这里取中间值
#: 作为默认值，调用方（B3）需要更松/更紧时可以在调用处自行传参覆盖。
DEFAULT_TIMEOUT_S = 45.0

#: `mission_events`话题的QoS history depth——方案2.3节第3条建议值。
DEFAULT_HISTORY_DEPTH = 10

#: 接收方按ID去重的缓存最大条数（超出后按最早收到的先淘汰），避免长任务
#: 跑很久之后这个缓存无限增长占内存——纯粹是工程上的保险措施，不是协议
#: 本身要求的行为。
DEFAULT_DEDUP_CACHE_SIZE = 256


class _NullLogger:
    """`node`为`None`（一般只在单元测试里出现）时使用的哑日志器，
    接口跟rclpy `Logger`常用的几个方法对齐但什么都不做，避免到处判空。
    """

    def info(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def warn(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def warning(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def error(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def debug(self, *_args: Any, **_kwargs: Any) -> None:
        pass


@dataclass
class _PendingSend:
    """一条正在等待对方ack的发送记录（`send_and_wait_ack()`内部状态）。"""

    #: 发出去的完整JSON payload（`dict`形式，重发时原样再发一次，
    #: 不重新生成——ID必须保持不变，否则对方没法用ID识别"这是同一条事件的重发"）。
    payload: Dict[str, Any]
    #: 是否已经收到对方的ack。由收到ack时的回调（`_handle_ack`）置`True`，
    #: `send_and_wait_ack()`的等待循环轮询这个标志。
    acked: bool = False


class ReliableEventChannel:
    """双机之间`mission_events`话题的应用层ACK+重发可靠性通道（方案2.3节"链路B"）。

    **上层（B3 `capabilities.py`）怎么用这个类**：

    1. 在`DroneSDK.__init__()`里，拿到已经初始化好的rclpy节点后构造一个本实例
       （每个`DroneSDK`实例一个`ReliableEventChannel`即可，不需要每次发消息都
       新建）::

           channel = ReliableEventChannel(node, namespace="drone1",
                                           teammate_namespace="drone2")

    2. `send_to_teammate(event, **kwargs)`内部直接转调
       `channel.send_and_wait_ack(event, timeout_s=..., **kwargs)`——阻塞直到
       对方确认收到，超时会抛`TeammateUnreachableError`（选手在`capabilities.py`
       这一层看到的就是这一个异常类型，不会接触到ID/ack这些协议细节）。

    3. `on_teammate_event(event_name, callback)`内部直接转调
       `channel.register_event_handler(event_name, callback)`——对方发来这个
       事件时，本通道会先自动回ack、再按ID去重，确认是"没处理过的新事件"之后
       才调用`callback(**kwargs)`。选手的`callback`不需要关心重发/去重逻辑。

    **命名空间与话题的对应关系**（避免"同一个话题两边互相听到自己刚发的消息"这种
    自我循环）：每个实例只在**自己的**命名空间下发布（`/{namespace}/mission_events`），
    只订阅**队友**命名空间下的话题（`/{teammate_namespace}/mission_events`）。
    这样drone1发的事件/ack都走`/drone1/mission_events`，drone2订阅这个话题收到；
    反过来drone2发的事件/ack都走`/drone2/mission_events`，drone1订阅这个话题收到
    ——两边完全对称，不会收到自己发出去的消息。
    """

    def __init__(
        self,
        node: Optional[Any],
        namespace: str,
        teammate_namespace: str,
        *,
        topic_name: str = DEFAULT_TOPIC_NAME,
        resend_interval_s: float = DEFAULT_RESEND_INTERVAL_S,
        history_depth: int = DEFAULT_HISTORY_DEPTH,
        transient_local: bool = True,
        dedup_cache_size: int = DEFAULT_DEDUP_CACHE_SIZE,
        clock_fn: Optional[Callable[[], float]] = None,
        wait_tick_fn: Optional[Callable[[float], None]] = None,
        publish_fn: Optional[Callable[[str], None]] = None,
    ) -> None:
        """构造一个可靠事件通道。

        Args:
            node: 已经`rclpy.init()`过、由调用方管理生命周期的rclpy Node对象，
                只要求具备`create_publisher`/`create_subscription`/
                `create_timer`/`get_logger`这几个标准方法即可（跟B2
                `_rclpy_runtime.py`一样，本模块不自己管rclpy生命周期）。传入
                的`node`还必须已经在被后台持续`spin()`（`RclpyRuntime`就是
                这么做的）——本模块自己**不会**也**不能**在等待循环里调用
                `rclpy.spin_once()`，理由见模块文档"关于等待期间谁来处理
                ROS2回调"一节。**仅在编写不依赖真实ROS2环境的单元测试时**
                才允许传`None`——这种情况下必须同时显式传入`publish_fn`和
                `wait_tick_fn`自己模拟"发送到网络"和"等待/推进网络处理"
                这两个动作，生产代码里必须传真实节点。
            namespace: 本机（自己这架飞机）的命名空间字符串，例如`"drone1"`。
            teammate_namespace: 队友那架飞机的命名空间字符串。
            topic_name: 话题名（不含命名空间前缀），默认`mission_events`。
            resend_interval_s: 重发频率，方案建议1Hz，即默认`1.0`秒一次。
            history_depth: QoS的history depth，方案建议至少10。
            transient_local: 是否给QoS加`TRANSIENT_LOCAL` durability
                （方案2.3节"可选"项，默认开启，进程重启重新订阅也能拿到
                最近几条历史事件；设`False`则退化为普通`VOLATILE`）。
            dedup_cache_size: 接收方按ID去重缓存的最大条数（防止无限增长）。
            clock_fn: 获取"当前时间"的函数，默认`time.monotonic`，单测里
                一般不需要替换（配合足够小的`resend_interval_s`/`timeout_s`
                即可让测试跑得快）。
            wait_tick_fn: `send_and_wait_ack()`阻塞等待期间，每一步"等一下"
                具体做什么，默认是简单`time.sleep(...)`（因为ROS2回调已经
                由后台spin线程处理，这里不需要、也不能自己spin，见
                `_default_wait_slice`文档）。单测里可以注入自己的假网络
                "推进一拍"的函数（顺带模拟延迟/丢包，并在需要时调用
                `resend_pending()`模拟后台1Hz重发定时器触发了一次）。
            publish_fn: 生产环境下默认包一层"把JSON字符串发布到自己的
                `mission_events`话题"，单测里可以注入直接调用对方通道
                `handle_incoming_json()`的假发送函数。
        """
        self._node = node
        self._namespace = namespace
        self._teammate_namespace = teammate_namespace
        self._resend_interval_s = resend_interval_s
        self._dedup_cache_size = dedup_cache_size
        self._clock_fn = clock_fn or time.monotonic

        # 保护下面这几个会被"调用方自己的线程"（`send_and_wait_ack`）和
        # "后台spin线程"（ROS2订阅/定时器回调）并发读写的共享状态——
        # `RclpyRuntime`用`MultiThreadedExecutor`常驻后台spin，这不是纸面
        # 假设，是这个模块真实要面对的并发场景（详见模块文档），所以这里
        # 老老实实加锁，不指望"反正CPython有GIL、单个dict操作原子"这种
        # 隐式保证扛住所有场景。
        self._state_lock = threading.Lock()
        # event_id -> 正在等待ack的发送记录。
        self._pending: Dict[str, _PendingSend] = {}
        # 接收方按ID去重用的缓存（`OrderedDict`当一个有大小上限的"记忆集合"用，
        # 最早收到的最先被淘汰）。
        self._seen_ids: "collections.OrderedDict[str, None]" = collections.OrderedDict()
        # event_name -> 选手注册的回调。
        self._handlers: Dict[str, Callable[..., None]] = {}
        # 生成"自己这一侧唯一事件ID"用的单调递增计数器，配合namespace前缀，
        # 两架飞机各自计数互不冲突（不需要引入uuid库增加依赖）。
        self._counter = itertools.count(1)

        self._resend_timer: Optional[Any] = None

        if node is not None:
            # 真实ROS2环境：自己建好发布者/订阅者，并且挂一个周期性定时器
            # 自动做1Hz重发检查——选手/上层完全不需要手动"tick"这个通道。
            self._setup_ros_transport(node, topic_name, history_depth, transient_local)
            self._wait_tick_fn = wait_tick_fn or self._default_wait_slice
            self._resend_timer = node.create_timer(resend_interval_s, self._on_resend_timer)
            self._logger = node.get_logger()
        else:
            # 单元测试专用路径：不碰任何rclpy符号，靠调用方注入的
            # publish_fn/wait_tick_fn模拟"网络"这一层（含丢包/延迟），
            # 从而独立测试ACK+重发+去重协议本身，不需要真实ROS2环境。
            if publish_fn is None or wait_tick_fn is None:
                raise ValueError(
                    "node为None时（一般只在单元测试里这样用），必须显式传入"
                    "publish_fn和wait_tick_fn来自己模拟发送/等待行为；生产环境"
                    "下请始终传入已初始化好的真实rclpy Node。"
                )
            self._publish_fn = publish_fn
            self._wait_tick_fn = wait_tick_fn
            self._logger = _NullLogger()

    # ------------------------------------------------------------------
    # 生产环境ROS2收发管线的搭建（只在传入真实node时才会执行到，且相关
    # rclpy符号延迟到这里才import，避免"只是想跑一下不依赖ROS2环境的单测"
    # 场景也被迫要求装好rclpy/ROS2环境）。
    # ------------------------------------------------------------------

    def _setup_ros_transport(
        self,
        node: Any,
        topic_name: str,
        history_depth: int,
        transient_local: bool,
    ) -> None:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        self._string_cls = String

        own_topic = self._topic_for(self._namespace, topic_name)
        teammate_topic = self._topic_for(self._teammate_namespace, topic_name)

        # 方案2.3节第3条：RELIABLE + 足够的history depth，可选TRANSIENT_LOCAL。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=history_depth,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if transient_local
                else DurabilityPolicy.VOLATILE
            ),
        )

        # 只在自己的命名空间下发布，只订阅队友命名空间下的话题——两边对称，
        # 不会收到自己刚发出去的消息，不需要额外的"跳过自己"过滤逻辑。
        self._pub = node.create_publisher(String, own_topic, qos)
        self._sub = node.create_subscription(String, teammate_topic, self._on_ros_message, qos)
        self._publish_fn = self._publish_via_ros

    @staticmethod
    def _topic_for(namespace: str, topic_name: str) -> str:
        return f"/{namespace.strip('/')}/{topic_name}"

    def _publish_via_ros(self, raw: str) -> None:
        msg = self._string_cls()
        msg.data = raw
        self._pub.publish(msg)

    def _on_ros_message(self, msg: Any) -> None:
        self.handle_incoming_json(msg.data)

    def _default_wait_slice(self, timeout_sec: float) -> None:
        """`send_and_wait_ack()`阻塞等待期间，默认每一步"等一下"具体做什么。

        **这里故意不调用`rclpy.spin_once(node, ...)`**：`RclpyRuntime`（B2）
        已经把传进来的`node`放进一个专职的后台线程，用`MultiThreadedExecutor`
        持续`spin()`——订阅回调（收到队友的ack/事件）、定时器回调（1Hz重发）
        全部由那个后台线程自动处理。如果这里再自己调一次`spin_once`，就是
        两个线程同时想spin同一个节点，会直接触发
        `RuntimeError: Executor is already spinning`（这是本模块开发时真实
        跑两节点集成烟雾测试踩到的坑，改成这里的纯`sleep`之后单节点自环
        烟雾测试验证过可以正常收发）。所以默认实现只需要简单`sleep`——真正
        让"ack到了"这件事发生的，是后台线程里的回调，不是这个函数本身；
        这个函数只负责让调用方所在的线程"歇一下"，避免忙等空转烧CPU。
        """
        time.sleep(timeout_sec)

    def shutdown(self) -> None:
        """清理这个通道创建的定时器/发布者/订阅者（生命周期仍然由调用方管理，
        这里只是主动归还自己在`node`上占用的资源，供`DroneSDK`析构/退出时调用；
        `node`为`None`的单测模式下什么都不用做）。
        """
        if self._node is None:
            return
        if self._resend_timer is not None:
            self._node.destroy_timer(self._resend_timer)
            self._resend_timer = None
        self._node.destroy_publisher(self._pub)
        self._node.destroy_subscription(self._sub)

    # ------------------------------------------------------------------
    # 对上层（B3 capabilities.py）暴露的核心接口。
    # ------------------------------------------------------------------

    def send_and_wait_ack(
        self,
        event: str,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        **kwargs: Any,
    ) -> None:
        """发送一个事件给队友，阻塞直到收到对方ack，超时抛`TeammateUnreachableError`。

        这是这一层对上层暴露的核心语义：选手/`capabilities.py`只需要知道"调用会
        阻塞到确认送达，或者超时抛异常"，完全不需要关心ID/ack/1Hz重发这些协议
        细节——那些都在这个方法内部完成。

        Args:
            event: 事件名字符串（比如`"SUPPLY_READY"`），需要跟接收方
                `register_event_handler()`注册的名字完全一致（大小写敏感）。
            timeout_s: 总超时秒数，默认取方案建议区间（30-60秒）的中间值45秒。
                方案2.3节"延迟预算"结论：这个超时跟WiFi实际延迟量级无关，是
                "正常延迟+一次路由器抖动重连"都要能容忍的宽松值，不要因为
                "双机WiFi延迟通常只有几毫秒到几十毫秒"就把这个值调得很紧。
            **kwargs: 随事件一起带给对方的业务数据，会被JSON序列化，因此这里
                只能传JSON能表示的类型（`str`/`int`/`float`/`bool`/`None`/
                `list`/`dict`及其嵌套），不能传自定义对象。

        Raises:
            TeammateUnreachableError: 超过`timeout_s`秒仍未收到对方ack。
        """
        event_id = self._next_event_id()
        payload: Dict[str, Any] = {
            "id": event_id,
            "event": event,
            "kwargs": kwargs,
            "ack": False,
        }
        pending = _PendingSend(payload=payload)
        with self._state_lock:
            self._pending[event_id] = pending
        try:
            # 不等第一个1Hz定时器周期，构造后立即先发一次——否则正常情况下
            # （对方本来就在线、马上就能收到）也要白白多等最多1秒才发出第一份，
            # 没有必要。之后的重发全部交给`_on_resend_timer`/`resend_pending`
            # 按固定周期统一处理，这里只负责"等"。
            self._send_raw(payload)

            deadline = self._clock_fn() + timeout_s
            # 等待期间每次最多"睡"这么长时间就回来检查一次deadline/acked，
            # 保证即使`resend_interval_s`设得很大，超时判断也不会迟到太久。
            poll_slice = min(0.2, self._resend_interval_s)
            while True:
                # `pending.acked`是单个bool属性的读取，被后台线程
                # （`_handle_ack`）以单次属性赋值的方式置位，CPython下这个
                # 读/写本身是原子的，不需要额外加锁就能安全轮询。
                if pending.acked:
                    return
                remaining = deadline - self._clock_fn()
                if remaining <= 0:
                    raise TeammateUnreachableError(
                        event=event,
                        timeout_s=timeout_s,
                        teammate_namespace=self._teammate_namespace,
                    )
                # 生产环境：默认只是sleep（ROS2回调交给后台spin线程处理，
                # 见`_default_wait_slice`）；单测环境：驱动调用方注入的假
                # 网络往前推进一拍（顺带模拟丢包/延迟，以及触发
                # `resend_pending()`模拟1Hz重发定时器触发了一次）。
                self._wait_tick_fn(max(0.001, min(poll_slice, remaining)))
        finally:
            # 不管是正常收到ack返回，还是超时抛异常，都要把这条记录从等待
            # 队列里摘掉——避免`resend_pending()`之后还傻傻地对着一条已经
            # 放弃等待的记录继续重发。
            with self._state_lock:
                self._pending.pop(event_id, None)

    def register_event_handler(self, event_name: str, callback: Callable[..., None]) -> None:
        """注册"收到队友发来的某个事件时"的处理回调。

        本通道内部保证：
        1. 不管这个事件是不是重复送达（对方还没收到我们的ack导致的重发），
           每次收到都会立刻回一个ack给对方——这样即使我们自己之前回的ack丢了，
           对方靠1Hz重发迟早能收到这一次的ack，从而停止重发。
        2. 但`callback`本身只会在"第一次见到这个事件ID"时被调用一次
           ——按ID去重，避免同一个事件的业务逻辑被重复执行多次
           （比如"收到一次SUPPLY_READY就该投一次货"，不能因为重发被投两次）。

        Args:
            event_name: 事件名字符串，跟对方`send_and_wait_ack(event=...)`
                传的字符串完全一致才能匹配上。
            callback: 形如`callback(**kwargs)`的函数，`kwargs`就是对方
                `send_and_wait_ack()`调用时传的那些业务数据（JSON反序列化
                之后的结果）。同一个`event_name`重复注册会直接覆盖旧的回调。
        """
        self._handlers[event_name] = callback

    def handle_incoming_json(self, raw: str) -> None:
        """处理一条从"网络"收进来的原始JSON字符串（事件或者ack）。

        生产环境下由ROS2订阅回调（`_on_ros_message`）自动调用，选手/上层不需要
        手动调这个方法；这里做成`public`方法主要是为了给单元测试用——测试里
        可以直接拿一个通道实例的`handle_incoming_json()`当作"这条消息送到了"
        的模拟入口，不需要真的搭一套ROS2话题收发。
        """
        try:
            data = json.loads(raw)
        except (TypeError, ValueError) as exc:
            self._logger.warning(
                f"[reliability] 收到一条无法解析的mission_events消息，已丢弃：{exc!r}"
            )
            return

        if data.get("ack"):
            self._handle_ack(data)
        else:
            self._handle_event(data)

    def resend_pending(self) -> None:
        """把当前所有"还没收到ack"的发送记录各重发一次。

        生产环境下由内部的ROS2定时器（`_on_resend_timer`，周期=`resend_interval_s`，
        默认1Hz）自动触发，选手/上层不需要手动调；单测环境下（`node=None`）
        没有真实定时器，需要由测试自己注入的`wait_tick_fn`在合适的时机调用
        这个方法，模拟"后台1Hz重发定时器触发了一次"。
        """
        # 先在锁内拍一份快照再解锁去做实际的发送——不在拿着锁的时候调用
        # `_send_raw`（可能触发真实的网络I/O，或者单测里同步直达对方的
        # `handle_incoming_json`），避免锁被占用太久、也避免测试里"假网络"
        # 同步重入时可能引发的各种意外。
        with self._state_lock:
            snapshot = list(self._pending.values())
        for pending in snapshot:
            if not pending.acked:
                self._send_raw(pending.payload)

    def _on_resend_timer(self) -> None:
        """ROS2定时器回调的薄包装，实际逻辑见`resend_pending()`。"""
        self.resend_pending()

    # ------------------------------------------------------------------
    # 内部协议实现细节。
    # ------------------------------------------------------------------

    def _next_event_id(self) -> str:
        """生成`f"{namespace}-{单调递增计数器}"`形式的唯一事件ID。

        两架飞机各自的`namespace`不同、各自维护自己的计数器，天然不会撞号，
        不需要为此引入uuid库增加依赖（方案原话"建议UUID或者这种简单方案"里
        选的是后者）。
        """
        seq = next(self._counter)
        return f"{self._namespace}-{seq}"

    def _send_raw(self, payload: Dict[str, Any]) -> None:
        raw = json.dumps(payload, ensure_ascii=False)
        self._publish_fn(raw)

    def _send_ack(self, event_id: str) -> None:
        """给对方回一个ack消息，`ack_of`字段指明这是在确认哪条事件ID。

        ack消息本身也带一个自己的`id`（主要是为了消息格式统一、方便调试时
        `ros2 topic echo`直接看懂，协议逻辑上ack消息的`id`字段目前没有被
        用来做去重判断——去重只针对事件消息）。
        """
        ack_payload: Dict[str, Any] = {
            "id": self._next_event_id(),
            "ack": True,
            "ack_of": event_id,
        }
        self._send_raw(ack_payload)

    def _handle_ack(self, data: Dict[str, Any]) -> None:
        ack_of = data.get("ack_of")
        with self._state_lock:
            pending = self._pending.get(ack_of) if ack_of is not None else None
        if pending is not None:
            # 单个bool属性赋值，CPython下原子，不需要在锁内做。
            pending.acked = True
        # 找不到对应的pending记录（比如已经超时放弃、或者是对方对一条我们
        # 从没发过的ID回的ack）就静默忽略，不是需要报错的异常情况。

    def _handle_event(self, data: Dict[str, Any]) -> None:
        event_id = data.get("id")
        event_name = data.get("event", "")
        kwargs = data.get("kwargs") or {}

        # 不管是不是第一次收到，先回ack——如果这是一次重发（对方还没收到
        # 我们之前回的ack），我们必须每次都补发ack，直到对方确认收到、
        # 停止重发为止。
        if event_id is not None:
            self._send_ack(event_id)

        # "检查是否见过这个ID"+"如果没见过就记下来"必须在同一把锁里原子
        # 完成，否则理论上两个几乎同时到达的重复投递（比如后台线程正在处理
        # 一条事件消息的同时又收到它自己的重发）可能都判断成"没见过"，导致
        # 业务回调被调用两次——这正是"接收方按ID去重"这条验收要求要防止的
        # 情况。
        with self._state_lock:
            is_duplicate = event_id in self._seen_ids
            if not is_duplicate and event_id is not None:
                self._seen_ids[event_id] = None
                while len(self._seen_ids) > self._dedup_cache_size:
                    self._seen_ids.popitem(last=False)

        if is_duplicate:
            self._logger.debug(
                f"[reliability] 收到重复事件'{event_name}'（id={event_id}），"
                f"已处理过，只补发ack，不重复调用回调"
            )
            return

        handler = self._handlers.get(event_name)
        if handler is None:
            self._logger.warning(
                f"[reliability] 收到来自{self._teammate_namespace}的事件"
                f"'{event_name}'，但本机没有注册对应的处理回调（ack已回复，"
                f"但业务逻辑被忽略）——如果选手代码本应处理这个事件，检查一下"
                f"`on_teammate_event()`注册的事件名字符串是否跟对方发送时完全一致。"
            )
            return
        handler(**kwargs)
