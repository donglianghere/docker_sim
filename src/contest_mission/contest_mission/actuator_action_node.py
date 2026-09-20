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
"""
import time
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


class ActuatorActionNode(Node):
    def __init__(self):
        super().__init__('actuator_action_node')

        self.declare_parameter('action_name', '')

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
        for p in params:
            if p.name != 'action_name':
                continue
            action_name = p.value
            if not action_name:
                continue  # 空字符串不触发动作，只是"清空"当前记录的动作名
            self._trigger_action(action_name)
        return SetParametersResult(successful=True)

    def _trigger_action(self, action_name: str):
        mapping = self._resolve_action_mapping(action_name)
        if mapping is None:
            self.get_logger().warn(
                f'未知动作名: {action_name}（已知动作: {_KNOWN_ACTIONS}），没有对应的RC'
                f'通道映射，忽略这次触发'
            )
            return
        if self._override_timer is not None:
            # 上一个动作的override发布回路还没结束（还没到hold_duration_s/
            # 还没释放通道），此时如果再叠加一个新动作，两个动作可能争抢
            # 同一个RC通道，行为会变得不可预测——所以这里选择直接忽略新
            # 触发、打警告，而不是打断上一个动作或者排队。
            self.get_logger().warn(
                f'动作{self._last_action}的override发布回路尚未结束，忽略新的触发请求: '
                f'{action_name}'
            )
            return

        rc_channel_1indexed, pwm, hold_duration_s = mapping
        self._last_action = action_name
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
        self._active_channel_1indexed = None
        self._active_pwm = None
        self._action_hold_duration_s = None
        self._action_start_monotonic = None
        self.get_logger().info(
            f'动作{finished_action}保持时长结束，已释放对应RC通道（设回CHAN_NOCHANGE）'
        )
        self._publish_status(f'done:{finished_action}')

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
