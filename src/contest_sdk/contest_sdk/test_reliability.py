"""`reliability.py`的独立单元测试（B4验收要求 + 方案第5节"先用synthetic丢包/
延迟场景测试，不要直接上真实网络环境验证"这条建议）。

**这里测的是什么**：ACK+重发+去重这套应用层可靠性协议本身的正确性，跟真实
ROS2/DDS通信完全无关。`ReliableEventChannel`在构造时传`node=None`会走"测试
专用路径"（见`reliability.py`构造函数），要求调用方自己注入`publish_fn`/
`wait_tick_fn`来模拟"网络"这一层——这里用自己写的`FakeNetwork`（支持人为
按规则丢包/按拍数延迟投递）+`VirtualClock`（虚拟时钟，测试不需要真的等墙钟
时间流逝）把两个通道互相接起来，覆盖：

1. 正常路径（无丢包无延迟）：ack顺利触发，回调恰好被调用一次。
2. 去程丢包（事件消息被丢几次）：验证1Hz重发协议能扛住，最终仍能确认成功。
3. 回程丢包（ack被丢几次）：验证接收方重复收到同一个事件ID时不会重复调用
   业务回调（按ID去重），但每次都要重新尝试回ack。
4. 完全丢包（100%丢包）：验证超过`timeout_s`后正确抛出`TeammateUnreachableError`，
   而不是永久阻塞或者误判成功；且超时后不残留待确认记录。
5. 接收方没注册回调：验证发送方不会被"对方没处理这个事件"卡住/误报超时——
   ack跟"有没有业务回调"是两件独立的事。
6. 网络延迟（消息要走几拍才到，但仍在超时窗口内）：验证不会被误判失败。
7. 协议层直接测试去重（不经过重发，直接反复投同一条原始消息）：业务回调只
   触发一次，但每次投递都补发一次ack。
8. 构造函数契约：`node=None`时不给`publish_fn`/`wait_tick_fn`必须报错。

**怎么跑**：`python3 test_reliability.py`（或者`python3 -m unittest
test_reliability`），不需要colcon/ROS2环境——本文件全程不`import rclpy`，
`reliability.py`本身也只有真的传入`node`（非`None`）时才会在内部
`import rclpy`/`std_msgs`，这里所有测试都传`node=None`，完全绕开那一段代码。
"""

import json
import os
import sys
import unittest
from typing import Callable, Dict, List, Optional, Tuple

# contest_sdk是纯Python包（没有走pip install/colcon安装），这里手动把包所在
# 目录（`.../src/contest_sdk`，即`contest_sdk`包目录本身的上一级）塞进
# sys.path，让`import contest_sdk.xxx`能找到——靠的是Python3隐式命名空间包
# 机制（没有`__init__.py`也能被识别成包），所以即使B5还没补上真正的
# `__init__.py`，这里也能正常import。
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_PARENT_DIR = os.path.dirname(_THIS_DIR)
if _PACKAGE_PARENT_DIR not in sys.path:
    sys.path.insert(0, _PACKAGE_PARENT_DIR)

from contest_sdk.exceptions import TeammateUnreachableError  # noqa: E402
from contest_sdk.reliability import ReliableEventChannel  # noqa: E402


class VirtualClock:
    """测试专用的虚拟时钟：只在测试代码主动调用`advance()`时才往前走，不依赖
    真实墙钟时间——这样即使某个测试用的是跟生产默认值一致的超时/重发间隔，
    也能在毫秒级的真实运行时间内跑完，不需要真的`time.sleep()`。
    """

    def __init__(self) -> None:
        self._now = 0.0

    def time(self) -> float:
        return self._now

    def advance(self, dt: float) -> None:
        self._now += dt


class FakeNetwork:
    """测试专用的假"网络"：把一个通道`publish_fn`发出去的消息转投给另一个通道的
    `handle_incoming_json()`，支持：

    - `set_drop_rule()`注入任意丢包规则（拿到`(src, dst, payload_dict)`判断
      要不要丢弃这一条），模拟"应用层ACK+1Hz重发"要应对的真实场景——单条消息
      偶尔丢、但重发几次总能送到。
    - 构造参数`delay_ticks`模拟消息要走几拍网络才到，配合`wait_tick_fn`每次
      调用推进一拍的节奏，测试"消息晚点到但仍在超时窗口内"的场景。

    完全不用真实socket/DDS；`tick()`就是"网络处理了一拍"，由测试注入进
    `ReliableEventChannel`的`wait_tick_fn`在通道阻塞等待期间主动调用。
    """

    def __init__(self, delay_ticks: int = 0) -> None:
        self._channels: Dict[str, ReliableEventChannel] = {}
        self._delay_ticks = delay_ticks
        self._drop_rule: Optional[Callable[[str, str, dict], bool]] = None
        # 在途消息队列，每条是[剩余延迟拍数, 目的namespace, 原始JSON字符串]。
        self._in_flight: List[list] = []
        # 记录所有"尝试发出"的消息（不管最终有没有被丢包），方便测试断言
        # "确实发生了多次重发"这类过程性行为，不是只看最终成功/失败结果。
        self.sent_log: List[Tuple[str, str, dict]] = []

    def register(self, namespace: str, channel: ReliableEventChannel) -> None:
        self._channels[namespace] = channel

    def set_drop_rule(self, rule: Callable[[str, str, dict], bool]) -> None:
        self._drop_rule = rule

    def make_publish_fn(self, src_namespace: str, dst_namespace: str) -> Callable[[str], None]:
        def _publish(raw: str) -> None:
            payload = json.loads(raw)
            self.sent_log.append((src_namespace, dst_namespace, payload))
            if self._drop_rule is not None and self._drop_rule(src_namespace, dst_namespace, payload):
                return  # 模拟丢包：网络层直接吞掉，收件方永远看不到这一条
            self._in_flight.append([self._delay_ticks, dst_namespace, raw])

        return _publish

    def tick(self) -> None:
        """推进一拍：所有在途消息的剩余延迟拍数-1，归零的立刻投递给目的通道。"""
        still_in_flight = []
        to_deliver = []
        for entry in self._in_flight:
            entry[0] -= 1
            if entry[0] <= 0:
                to_deliver.append(entry)
            else:
                still_in_flight.append(entry)
        self._in_flight = still_in_flight
        for _, dst_namespace, raw in to_deliver:
            self._channels[dst_namespace].handle_incoming_json(raw)


class _WaitTickBox:
    """解决"构造`ReliableEventChannel`时必须传`wait_tick_fn`，但`wait_tick_fn`
    的真正实现又需要引用channel自己"这个先有鸡先有蛋的问题——先给构造函数传一个
    转发到`self.fn`的占位调用，等channel真正构造完成后再把`self.fn`换成正式实现
    （见`_build_channel_pair`）。
    """

    def __init__(self) -> None:
        self.fn: Callable[[float], None] = lambda timeout_sec: None

    def __call__(self, timeout_sec: float) -> None:
        self.fn(timeout_sec)


def _make_wait_tick_fn(
    channel: ReliableEventChannel,
    network: FakeNetwork,
    clock: VirtualClock,
    resend_interval_s: float,
) -> Callable[[float], None]:
    """模拟生产环境下"阻塞等待期间反复spin"这件事：每次调用让虚拟时钟前进
    `timeout_sec`、让假网络处理一拍，并按`resend_interval_s`节奏调用
    `resend_pending()`——生产环境里这两件事分别是"rclpy执行器处理进来的回调"
    和"内部ROS2定时器周期性触发重发"，这里在单线程虚拟时钟下手动模拟出等效行为。
    """
    state = {"last_resend_at": clock.time()}

    def _wait_tick(timeout_sec: float) -> None:
        clock.advance(timeout_sec)
        network.tick()
        if clock.time() - state["last_resend_at"] >= resend_interval_s:
            channel.resend_pending()
            state["last_resend_at"] = clock.time()

    return _wait_tick


def _build_channel_pair(
    network: FakeNetwork,
    clock: VirtualClock,
    resend_interval_s: float = 0.1,
) -> Tuple[ReliableEventChannel, ReliableEventChannel]:
    """按方案约定的"drone1"/"drone2"两个命名空间各建一个通道，并互相接到同一个
    `FakeNetwork`上，返回`(channel_drone1, channel_drone2)`。
    """
    box_a = _WaitTickBox()
    box_b = _WaitTickBox()

    channel_a = ReliableEventChannel(
        node=None,
        namespace="drone1",
        teammate_namespace="drone2",
        resend_interval_s=resend_interval_s,
        publish_fn=network.make_publish_fn("drone1", "drone2"),
        wait_tick_fn=box_a,
        clock_fn=clock.time,
    )
    channel_b = ReliableEventChannel(
        node=None,
        namespace="drone2",
        teammate_namespace="drone1",
        resend_interval_s=resend_interval_s,
        publish_fn=network.make_publish_fn("drone2", "drone1"),
        wait_tick_fn=box_b,
        clock_fn=clock.time,
    )

    network.register("drone1", channel_a)
    network.register("drone2", channel_b)

    # channel真正构造完成之后，才能把wait_tick_fn的占位实现换成引用了channel
    # 自己的正式实现。
    box_a.fn = _make_wait_tick_fn(channel_a, network, clock, resend_interval_s)
    box_b.fn = _make_wait_tick_fn(channel_b, network, clock, resend_interval_s)

    return channel_a, channel_b


class ReliableEventChannelTests(unittest.TestCase):
    """覆盖方案第5节要求的"发送方发出消息、模拟一定丢包率/延迟、接收方按ID
    去重、ack是否正确触发/超时是否正确触发"这几类场景。
    """

    def test_normal_path_no_loss_ack_triggers_and_handler_called_once(self) -> None:
        """完全没有丢包/延迟的正常路径：`send_and_wait_ack`应该顺利返回（不抛
        异常），对方注册的回调应该被调用恰好一次，且kwargs原样透传。
        """
        network = FakeNetwork(delay_ticks=0)
        clock = VirtualClock()
        drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        received = []
        drone2.register_event_handler("SUPPLY_READY", lambda **kw: received.append(kw))

        drone1.send_and_wait_ack("SUPPLY_READY", timeout_s=5.0, x=1.0, y=2.0, note="ok")

        self.assertEqual(received, [{"x": 1.0, "y": 2.0, "note": "ok"}])

    def test_lossy_outbound_network_recovers_before_timeout_via_resend(self) -> None:
        """去程（事件消息）前2次尝试都被人为丢弃，第3次才放行——验证1Hz重发
        协议能扛住短暂丢包，最终仍能在超时窗口内成功确认，且回调只被调用一次
        （不会因为重发被多次触发业务逻辑）。
        """
        network = FakeNetwork(delay_ticks=0)
        clock = VirtualClock()
        drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        received = []
        drone2.register_event_handler("SUPPLY_READY", lambda **kw: received.append(kw))

        drop_state = {"count": 0}

        def _drop_rule(src: str, dst: str, payload: dict) -> bool:
            # 只丢事件消息（ack字段为False），不丢ack本身，模拟更常见的
            # "去程偶尔丢包"场景；前2次尝试丢弃，第3次及以后放行。
            if payload.get("ack"):
                return False
            if drop_state["count"] < 2:
                drop_state["count"] += 1
                return True
            return False

        network.set_drop_rule(_drop_rule)

        drone1.send_and_wait_ack("SUPPLY_READY", timeout_s=5.0, seq=1)

        self.assertEqual(received, [{"seq": 1}])
        # 确认真的发生了重发（至少3次尝试发送这条事件），不是"侧面刚好绕过了
        # 丢包逻辑"这种巧合导致测试通过。
        event_attempts = [
            p for (src, _dst, p) in network.sent_log if src == "drone1" and not p.get("ack")
        ]
        self.assertGreaterEqual(len(event_attempts), 3)

    def test_lossy_ack_still_dedups_and_recovers_via_resend(self) -> None:
        """事件消息本身送到了，但回程的ack被丢了几次——发送方感知不到"对方其实
        已经处理过了"，只能继续按1Hz重发；接收方必须对重复到达的事件消息按ID
        去重（不重复调用回调），但每次都要重新尝试回ack，直到某次ack终于送达
        为止。
        """
        network = FakeNetwork(delay_ticks=0)
        clock = VirtualClock()
        drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        received = []
        drone2.register_event_handler("SUPPLY_READY", lambda **kw: received.append(kw))

        drop_state = {"count": 0}

        def _drop_rule(src: str, dst: str, payload: dict) -> bool:
            if payload.get("ack") and drop_state["count"] < 2:
                drop_state["count"] += 1
                return True
            return False

        network.set_drop_rule(_drop_rule)

        drone1.send_and_wait_ack("SUPPLY_READY", timeout_s=5.0, seq=1)

        # 关键断言：即使事件消息因为对方重发被投递了不止一次，业务回调也只能
        # 被调用一次——这就是"接收方按ID去重"这条验收要求。
        self.assertEqual(received, [{"seq": 1}])

    def test_total_packet_loss_times_out_and_raises_teammate_unreachable(self) -> None:
        """事件消息完全送不到对方（100%丢包），必须在`timeout_s`之后抛出
        `TeammateUnreachableError`，而不是永久阻塞或者误判成功；异常文本要
        带上事件名/队友namespace/超时值，方便选手排查。
        """
        network = FakeNetwork(delay_ticks=0)
        clock = VirtualClock()
        drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        drone2.register_event_handler("SUPPLY_READY", lambda **kw: None)
        network.set_drop_rule(lambda src, dst, payload: True)  # 全部丢弃

        with self.assertRaises(TeammateUnreachableError) as ctx:
            drone1.send_and_wait_ack("SUPPLY_READY", timeout_s=1.0, seq=1)

        message = str(ctx.exception)
        self.assertIn("SUPPLY_READY", message)
        self.assertIn("drone2", message)
        self.assertIn("1.0", message)
        # 超时之后不应该再残留这条待确认记录，否则后台重发定时器会对着一条
        # 已经放弃等待的记录继续白白重发。
        self.assertEqual(len(drone1._pending), 0)  # noqa: SLF001（白盒测试内部状态）

    def test_unhandled_event_on_receiver_side_still_acks_sender(self) -> None:
        """接收方压根没给这个事件名注册回调——发送方不应该因此被卡住/超时，
        因为"回ack"跟"有没有注册业务回调"是两件独立的事：协议层保证送达确认，
        选手要不要处理这个事件是选手自己的事。
        """
        network = FakeNetwork(delay_ticks=0)
        clock = VirtualClock()
        drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        # 故意不给drone2注册任何回调。
        drone1.send_and_wait_ack("UNKNOWN_EVENT", timeout_s=5.0)  # 不应该抛异常/不应该卡住

    def test_delayed_delivery_within_timeout_window_still_succeeds(self) -> None:
        """事件+ack都要经过几拍网络延迟才能到，但延迟远小于超时窗口——应该
        最终成功，不应该因为"不是立刻到"就被误判失败（对应方案"双机WiFi延迟
        通常几毫秒到几十毫秒，重发协议本身跟延迟量级无关"这条结论）。
        """
        network = FakeNetwork(delay_ticks=3)
        clock = VirtualClock()
        drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        received = []
        drone2.register_event_handler("SUPPLY_READY", lambda **kw: received.append(kw))

        drone1.send_and_wait_ack("SUPPLY_READY", timeout_s=5.0, seq=1)

        self.assertEqual(received, [{"seq": 1}])

    def test_duplicate_delivery_dedup_at_protocol_level(self) -> None:
        """不经过重发机制，直接反复调用`handle_incoming_json`喂同一条原始
        JSON（模拟"同一条事件消息被投递了3次"这个最终效果），验证：
        1）业务回调只在第一次被调用；2）每一次投递都会补发一次ack
        （接收方没法预先知道对方是不是已经拿到过之前的ack，只能每次都回）。
        """
        network = FakeNetwork()
        clock = VirtualClock()
        _drone1, drone2 = _build_channel_pair(network, clock, resend_interval_s=0.1)

        received = []
        drone2.register_event_handler("SUPPLY_READY", lambda **kw: received.append(kw))

        raw = json.dumps(
            {"id": "drone1-1", "event": "SUPPLY_READY", "kwargs": {"seq": 1}, "ack": False}
        )
        for _ in range(3):
            drone2.handle_incoming_json(raw)

        self.assertEqual(received, [{"seq": 1}])
        ack_sent = [p for (src, _dst, p) in network.sent_log if src == "drone2" and p.get("ack")]
        self.assertEqual(len(ack_sent), 3)

    def test_headless_mode_requires_publish_and_spin_fn(self) -> None:
        """`node=None`时如果不提供`publish_fn`/`wait_tick_fn`应该直接报错，
        不能让调用方悄悄拿到一个"看起来能用但实际收发不了消息"的坏实例。
        """
        with self.assertRaises(ValueError):
            ReliableEventChannel(node=None, namespace="drone1", teammate_namespace="drone2")


if __name__ == "__main__":
    unittest.main()
