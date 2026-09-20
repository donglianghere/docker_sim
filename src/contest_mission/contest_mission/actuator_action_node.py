#!/usr/bin/env python3
"""执行器（符号化动作→真实mavros/rc/override驱动）节点。

对应《2026大赛任务系统开发执行方案.md》阶段6.1 + 《2026大赛任务系统
全流程任务仿真实现方案.md》1.6节（v3-e新增，用户明确抓取/投放/发射
要真实触发，不能再停留在"纯状态记录"）。

**历史：这个节点最初只做了"记录状态"这一半**（详见git历史/
DEBUG_JOURNAL.md），当时的方案原文是"提供一个perform_action(action_
name)service，内部只是记录状态+做一个约定好的飞行动作……不做任何
物理建模"，且用户在阶段3明确要求"各模块尽量独立，不要影响别的模块"，
所以刻意没有让这个节点去控制飞行本身。

**这次（1.6节）改造的结论**：`grab_supply`/`drop_supply`/
`horizontal_launch`这三个动作不再是纯状态记录，而是真正往
`mavros/rc/override`话题（`mavros_msgs/OverrideRCIn`，18通道
`uint16`数组）发布数据，复用飞机上已经接好的遥控器开关通道触发真实
机构——**仿真环境也走这条真实链路**（哪怕Gazebo里没有对应的抓取臂/
投放舱物理模型去响应这个信号，链路本身——SDK触发→本节点发布
override→PX4接收→写入`input_rc`——在仿真里就要能完整跑通，跟真机
部署时是同一套代码路径，不需要额外适配）。这跟"不做物理建模"并不
矛盾：仿真里依然不建模机构本身的物理响应，只是链路这一层（发布
override消息）现在是真实的，不再是空转的状态记录。

**为什么必须持续发布+完成后显式释放，不能"发一次就不管"**（这次
直接查了本机`/home/robots/PX4-Autopilot`源码`mavlink_receiver.cpp`/
`rc_update.cpp`+mavros源码`rc_io.cpp`确认，不是猜的，完整依据见方案
1.6节）：
1. mavros的`rc/override`插件收到一条ROS消息就转发一条MAVLink
   `RC_CHANNELS_OVERRIDE`，**自己不会重发**，发布节奏完全靠调用方
   （也就是本节点）。
2. PX4收到`RC_CHANNELS_OVERRIDE`后直接把值写进`input_rc`这个uORB
   话题（`rc_lost=false`），消费它的`rc_update`模块是**订阅回调
   驱动**（`SubscriptionCallbackWorkItem`），不是定时轮询——PX4
   底层**没有**"隔多久没收到override消息就自动判失效/回退"这个
   机制。
3. 如果飞机上的真实遥控器接收机同时在正常工作（大概率会，作为安全
   后备），它会用远高于override频率的硬件帧率持续往`input_rc`发布
   数据，等于不断把这个话题"抢回去"——只发一次override，很快就会
   被下一帧真实的、没被拨动的开关状态覆盖回去，机构可能触发不了/
   触发不够久。
4. 如果没有真实接收机在工作，停止发布也不会自动复位（uORB话题会
   停留在最后一次写入的值），必须显式发一帧把该通道设成
   `CHAN_NOCHANGE`（65535）交还控制权，否则这个通道会一直卡在触发
   PWM值上。

因此本节点的做法是：收到触发后，用一个`create_timer`以≥10Hz的频率
持续发布`OverrideRCIn`，只改配置的那一个通道、其余17个通道全部填
`CHAN_NOCHANGE`（不影响别的通道，尤其不能影响姿态/油门相关通道），
持续到配置的`hold_duration_s`，然后**再发一帧把该通道也设回
`CHAN_NOCHANGE`**释放控制权，最后才广播`action_status`为
`done:<name>`。这个"持续发布+完成后释放"的短时回路必须放在
flight-stack内机载执行（1.0节"跨网络不适合做持续/短时高频回路"的
架构原则的又一次应用），所以放在这个节点里，不是SDK里。

**perform_action怎么"调"（触发接口不变，SDK这边`sdk.do_action()`的
调用形状完全不用改）**：跟阶段3的`relay_mode`用的是同一套零接口包
技巧——`action_name`是一个ROS2节点参数，调用方通过节点自带的标准
`~/set_parameters`服务设置它（`ros2 param set /<ns>/actuator_action
action_name <name>`），本节点的参数回调收到后启动上面说的override
发布回路，完成后通过`action_status`话题广播`"done:<name>"`，不需要
为"perform_action"这个概念专门生成一个自定义.srv接口包。

**未决问题（不是这个节点能自己决定的，见方案1.6节"未决问题"、清单
A4节"未决问题"）**：三个动作具体对应哪个RC通道号、什么PWM值触发、
需要保持多久，这是真实机构/遥控器接线的硬件细节，方案层面没有答案，
需要跟负责机构接线的人确认后再改。**这次实现把这三组数字做成了
ROS2参数（见下面`declare_parameter`），先用统一的占位值（通道9、
PWM 2000、保持1秒）让链路跑通——这三个占位值不代表最终真实配置**，
真实通道确定后改launch参数就行，不需要改这个文件的代码。

**2026-09-20新增：`action_goal_id`（幂等去重 + 结果关联）**。原来的触发
方式只设一个`action_name`参数、完成后广播`done:<name>`，有两个真实隐患：
1. **没有幂等**：参数重复设成同一个值（调用方重试、或者防DDS发现没跟上
   而连发几次——SDK里`goto()`/`cancel_goto()`就有这种连发防护）会被当成
   两次独立触发，抓取动作可能真的执行两次。
2. **结果关联不上**：`done:<name>`里没有任何标识，调用方分不清收到的是
   "这一次"的完成，还是上一次遗留的状态——只能靠调用前清空本地缓存这种
   client侧的约定去规避，跨进程重启就失效。

做法（不引入自定义.srv/action接口包，维持"零接口包"的既有风格）：调用方
在**同一次**`~/set_parameters`调用里同时设`action_name`和`action_goal_id`
（SDK的`_call_set_parameters_blocking()`本来就是把dict一次性打包成一个
SetParameters请求，天然满足"同一批"这个要求）。本节点按`goal_id`判重：
- 同一个`goal_id`正在执行 -> 不重复触发，重新广播一次当前`running`状态；
- 同一个`goal_id`已经完成 -> 不重复触发，重新广播那次的`done`状态（这样
  调用方即使错过了第一次广播，重试一次也能拿到结果，而不是把动作再做一遍）；
- 新的`goal_id`但上一个动作还没结束 -> 明确广播`rejected:...:busy`，让调用
  方立刻失败，而不是干等到超时（原来的行为是只打一条warn日志就丢弃）。

状态话题格式因此扩展为`<kind>:<name>:<goal_id>`（kind取running/done/
rejected，rejected再多一段原因）。**向后兼容**：`goal_id`为空字符串时
（老调用方只设`action_name`），完全维持原来的`done:<name>`格式和原来的
行为不变，不会破坏任何现有调用方。
"""
import time
from collections import OrderedDict
from typing import Optional, Tuple

from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

# 这次任务已知的三个符号化动作名。方案1.6节/清单A4节明确这三个动作名
# 是任务层面定好的（不是选手自定义字符串——跟SDK里`do_action(name)`
# 的`name`参数本身是选手自定义字符串这件事不矛盾：选手调用时传的字符
# 串最终就是这三个之一，只是SDK这一层不关心/不校验具体取值），
# 每个动作名对应一组"RC通道号+目标PWM值+保持时长"参数，具体数值见
# 下面`declare_parameter`调用，全部是占位值。
_KNOWN_ACTIONS = ('grab_supply', 'drop_supply', 'horizontal_launch')

# mavros_msgs/OverrideRCIn.channels是18个元素的uint16数组，对应RC
# 通道1~18（数组下标0对应通道1）。这里的"通道号"参数按行业惯例用
# 1-indexed（跟QGroundControl/遥控器面板上标的"通道9"这种说法一致），
# 使用时再换算成数组下标。
_NUM_RC_CHANNELS = 18

# 记住多少个已完成的goal_id用于重复下发时重播结果（有界FIFO，防止长时间
# 运行后无限增长）。一次任务里的动作数量是个位数量级，32足够宽裕。
_COMPLETED_GOALS_MAX = 32


class ActuatorActionNode(Node):
    def __init__(self):
        super().__init__('actuator_action_node')

        self.declare_parameter('action_name', '')
        # 调用方为这一次触发生成的唯一标识（见文件头"action_goal_id"一节）。
        # 空字符串=老调用方，维持原有行为。
        self.declare_parameter('action_goal_id', '')

        # ≥10Hz是方案1.6节的硬性要求（PX4侧没有超时自动失效机制，必须
        # 靠持续发布维持覆盖状态）；默认给20Hz留一点余量，不是刚好卡在
        # 10Hz这个下限上。
        self.declare_parameter('override_publish_rate_hz', 20.0)

        # === 动作→硬件映射参数，逐个动作声明3个参数 ===
        # ⚠️ 占位值：通道9、PWM 2000、保持1秒。这三个数字是真实机构/
        # 遥控器接线才能确定的硬件细节，方案层面拿不到（见文件头"未决
        # 问题"），这里只是先填一组统一的占位值让"SDK触发→本节点发布
        # override→PX4接收"这条链路能跑起来、能在仿真里验证，**不代表
        # 这是最终真实配置**。真实通道/PWM/时长确定后，直接在launch
        # 文件里覆盖这几个参数即可，不需要改这份代码。
        for action_name in _KNOWN_ACTIONS:
            self.declare_parameter(f'{action_name}_rc_channel', 9)
            self.declare_parameter(f'{action_name}_pwm', 2000)
            self.declare_parameter(f'{action_name}_hold_duration_s', 1.0)

        self._last_action: Optional[str] = None
        self._last_action_time = None

        # 幂等去重用的运行时状态（见文件头"action_goal_id"一节）。
        # _active_goal_id: 当前正在执行的那次触发的goal_id（空串=老调用方）。
        # _completed_goals: 最近完成过的goal_id -> 当时广播的完整状态字符串，
        #   用于"同一个goal_id重复下发时重播结果而不是重做动作"。用
        #   OrderedDict当有界FIFO，避免长时间运行后无限增长。
        self._active_goal_id: str = ''
        self._completed_goals: "OrderedDict[str, str]" = OrderedDict()

        # 当前正在执行的动作的运行时状态（None表示当前没有动作在进行）。
        self._override_timer = None
        self._active_channel_1indexed: Optional[int] = None
        self._active_pwm: Optional[int] = None
        self._action_hold_duration_s: Optional[float] = None
        self._action_start_monotonic: Optional[float] = None

        self.status_pub = self.create_publisher(String, 'action_status', 10)
        # 真正驱动机构的话题：mavros的rc/override插件订阅这个话题，
        # 收到一条就转一条MAVLink RC_CHANNELS_OVERRIDE给PX4，自己不
        # 重发（见文件头依据），所以下面必须靠定时器持续publish。
        self.override_pub = self.create_publisher(OverrideRCIn, 'mavros/rc/override', 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._publish_status('idle')
        self.get_logger().info(
            "actuator_action_node就绪（真实mavros/rc/override驱动），调用方式："
            "ros2 param set /<ns>/actuator_action action_name <grab_supply|drop_supply"
            "|horizontal_launch>。当前RC通道/PWM/保持时长均为占位值，真实硬件接线"
            "确定后请通过launch参数覆盖，见文件头说明。"
        )

    def _on_set_parameters(self, params):
        """参数回调。

        注意这是"设置前"的校验回调——此时新值还没写进参数存储，所以
        `goal_id`必须从这一批`params`里取，不能用`self.get_parameter()`
        （那样拿到的是上一次的旧值）。调用方把`action_name`和
        `action_goal_id`放在同一次set_parameters请求里，这里就能在同一
        批里同时看到两者。
        """
        action_name = None
        goal_id = None
        for p in params:
            if p.name == 'action_name':
                action_name = p.value
            elif p.name == 'action_goal_id':
                goal_id = p.value
        if action_name:
            # goal_id缺省（老调用方只设action_name）时用空串，走兼容路径。
            self._trigger_action(action_name, goal_id or '')
        # action_name为空字符串时不触发动作，只是"清空"当前记录的动作名
        return SetParametersResult(successful=True)

    def _trigger_action(self, action_name: str, goal_id: str = ''):
        mapping = self._resolve_action_mapping(action_name)
        if mapping is None:
            self.get_logger().warn(
                f'未知动作名: {action_name}（已知动作: {_KNOWN_ACTIONS}），没有对应的RC'
                f'通道映射，忽略这次触发'
            )
            self._publish_action_status('rejected', action_name, goal_id, reason='unknown_action')
            return

        if goal_id:
            # 幂等：同一个goal_id重复下发，不重做动作，只重播当前/历史状态。
            if goal_id == self._active_goal_id and self._override_timer is not None:
                self.get_logger().info(
                    f'goal_id={goal_id} 正在执行中，这次重复下发不再触发动作，只重播running状态'
                )
                self._publish_action_status('running', action_name, goal_id)
                return
            if goal_id in self._completed_goals:
                self.get_logger().info(
                    f'goal_id={goal_id} 之前已完成，这次重复下发不再触发动作，只重播完成状态'
                )
                self._publish_status(self._completed_goals[goal_id])
                return

        if self._override_timer is not None:
            # 上一个动作的override发布回路还没结束（还没到hold_duration_s/
            # 还没释放通道），此时如果再叠加一个新动作，两个动作可能争抢
            # 同一个RC通道，行为会变得不可预测——所以这里选择直接拒绝新
            # 触发，而不是打断上一个动作或者排队。跟改造前的区别：现在会
            # 明确广播一条rejected，调用方可以立刻失败，不用干等到超时。
            self.get_logger().warn(
                f'动作{self._last_action}的override发布回路尚未结束，拒绝新的触发请求: '
                f'{action_name}'
            )
            self._publish_action_status('rejected', action_name, goal_id, reason='busy')
            return

        rc_channel_1indexed, pwm, hold_duration_s = mapping
        self._last_action = action_name
        self._active_goal_id = goal_id
        self._last_action_time = self.get_clock().now()
        self._active_channel_1indexed = rc_channel_1indexed
        self._active_pwm = pwm
        self._action_hold_duration_s = hold_duration_s
        self._action_start_monotonic = time.monotonic()

        rate_hz = float(self.get_parameter('override_publish_rate_hz').value)
        self.get_logger().info(
            f'执行动作: {action_name} -> RC通道{rc_channel_1indexed}=PWM{pwm}，'
            f'持续{hold_duration_s}s，发布频率{rate_hz}Hz（真实驱动mavros/rc/override，'
            f'当前为占位映射值，见文件头说明）'
        )
        # 先立即发一帧，不等第一个定时器周期，避免"保持时长"很短时（比如
        # 占位值1秒、频率20Hz）第一帧被无谓地延迟到1/20秒之后才发出去。
        self._publish_override(rc_channel_1indexed, pwm)
        self._publish_action_status('running', action_name, goal_id)
        self._override_timer = self.create_timer(1.0 / rate_hz, self._on_override_tick)

    def _resolve_action_mapping(self, action_name: str) -> Optional[Tuple[int, int, float]]:
        if action_name not in _KNOWN_ACTIONS:
            return None
        rc_channel = int(self.get_parameter(f'{action_name}_rc_channel').value)
        pwm = int(self.get_parameter(f'{action_name}_pwm').value)
        hold_duration_s = float(self.get_parameter(f'{action_name}_hold_duration_s').value)
        return rc_channel, pwm, hold_duration_s

    def _on_override_tick(self):
        elapsed = time.monotonic() - self._action_start_monotonic
        if elapsed < self._action_hold_duration_s:
            self._publish_override(self._active_channel_1indexed, self._active_pwm)
            return

        # 保持时长到了：显式发一帧把该通道设回CHAN_NOCHANGE，交还控制权
        # （见文件头"为什么必须……显式释放"第4点——PX4不会自动复位这个
        # 通道的值，停止发布不等于释放）。
        self._publish_override(self._active_channel_1indexed, OverrideRCIn.CHAN_NOCHANGE)
        self._override_timer.cancel()
        self._override_timer = None
        finished_action = self._last_action
        finished_goal_id = self._active_goal_id
        self._active_channel_1indexed = None
        self._active_pwm = None
        self._action_hold_duration_s = None
        self._action_start_monotonic = None
        self._active_goal_id = ''
        self.get_logger().info(
            f'动作{finished_action}保持时长结束，已释放对应RC通道（设回CHAN_NOCHANGE）'
        )
        done_status = self._publish_action_status('done', finished_action, finished_goal_id)
        if finished_goal_id:
            # 记住这次的结果，供同一个goal_id重复下发时重播（有界FIFO）。
            self._completed_goals[finished_goal_id] = done_status
            while len(self._completed_goals) > _COMPLETED_GOALS_MAX:
                self._completed_goals.popitem(last=False)

    def _publish_override(self, rc_channel_1indexed: int, value: int):
        """只改`rc_channel_1indexed`这一个通道，其余17个通道全部填
        CHAN_NOCHANGE（65535），不触碰任何别的通道（尤其不能影响姿态/
        油门相关通道，见方案1.6节要求）。
        """
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * _NUM_RC_CHANNELS
        index = rc_channel_1indexed - 1  # 1-indexed通道号 -> 0-indexed数组下标
        if not (0 <= index < _NUM_RC_CHANNELS):
            self.get_logger().error(
                f'配置的RC通道号{rc_channel_1indexed}超出合法范围[1,{_NUM_RC_CHANNELS}]，'
                f'本帧不发布，检查对应action的_rc_channel参数是否填错'
            )
            return
        msg.channels[index] = value
        self.override_pub.publish(msg)

    def _publish_action_status(
        self, kind: str, action_name: str, goal_id: str, reason: str = ''
    ) -> str:
        """按`<kind>:<name>:<goal_id>`格式广播动作状态，返回广播出去的字符串。

        `goal_id`为空（老调用方）时退化成原来的`<kind>:<name>`格式，保证
        既有调用方不受影响——注意这种情况下`running`/`rejected`这两种新增
        的kind同样会广播，但老调用方只匹配`done:<name>`，多出来的状态会被
        它忽略，不影响行为。
        """
        status = f'{kind}:{action_name}' if not goal_id else f'{kind}:{action_name}:{goal_id}'
        if reason:
            status = f'{status}:{reason}'
        self._publish_status(status)
        return status

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
