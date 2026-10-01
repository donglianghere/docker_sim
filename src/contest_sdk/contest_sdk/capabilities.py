#!/usr/bin/env python3
"""L2层：`DroneSDK`——选手真正调用的那个类（对应方案2.2节分层设计`capabilities.py`）。

**这个文件的职责边界**：只负责"按2.1节能力清单，把L1（`_rclpy_runtime.py`
拿到的节点句柄）+L3（`reliability.py`的可靠事件通道）组装成选手能直接调用
的方法"——每一项能力都是"一次性指令+等待/轮询结果"这种模式（1.0节架构
原则："决策/一次性指令放SDK，实时控制回路放飞行栈"），这个文件里**没有
一行代码是持续高频闭环的控制逻辑**，那些全部在对应的flight-stack节点里
（`formation_follower_node`/`precision_servo_node`/`position_cmd_relay`
等），这里只负责"发一次配置/一次性指令，然后等对方广播结果"。

**跟rclpy的关系**（呼应`_rclpy_runtime.py`文件头"边界划在哪里"的说明）：
这个文件不`import rclpy`、不调用`rclpy.init()`/`spin()`/`shutdown()`，
但会`from rclpy.time import Time`/`from rclpy.duration import Duration`
这类"调用一个已经活起来的节点的方法时需要用到的辅助类型"（TF查询、
构造消息时间戳都要用），跟"管理rclpy生命周期"是两件不同的事——`_rclpy_
runtime.py`文件头已经把这条边界讲清楚了，这里延续同一个原则。

**关于"为什么不用`rclpy.spin_until_future_complete()`等service调用结果"**
（`do_action`/`start_formation_follow`/`center_on_target`等好几个方法都
要调`~/set_parameters`/`~/reset_aim`这类service）：`RclpyRuntime`已经用
`MultiThreadedExecutor`把这个节点放进一个专职后台线程持续`spin()`（见
`_rclpy_runtime.py`文件头），如果这里再调一次`rclpy.spin_until_future_
complete()`，就是两个线程同时想spin同一个节点，会直接触发`RuntimeError:
Executor is already spinning`——这正是`reliability.py`文件头详细记录过的
坑（本项目`fire_pillar_aim_node.py`的写法是单线程节点内部调用，跟这里
"节点已经在被别的线程spin"的场景不一样，不能照抄）。这个文件统一用
`_call_service_blocking()`这个helper：`call_async()`发起调用+在
`add_done_callback()`里用`threading.Event`置位，调用方线程只负责
`sleep`等待——跟`reliability.py`的`send_and_wait_ack()`是同一个模式。

**B6可调试性打点**：`_progress()`是"SDK自己的进度打印"这一层封装
（方案2.2.2节要求，不直接用`self._node.get_logger()`裸ROS2日志）——
现在只是简单包一层`print()`，以后要改成输出到文件/GUI，只需要改这
一个函数，不需要改下面几十处调用点。`_poll_until()`是所有"轮询等待
+定期打印进度"的阻塞方法共用的小循环，避免每个方法都重复写一遍
"多久检查一次、多久打印一次进度、超时怎么判断"这套逻辑。
"""
import contextlib
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import (Any, Callable, Dict, List, Literal, Optional, Sequence,
                    Tuple, overload)

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger

from contest_sdk._rclpy_runtime import RclpyRuntime
from contest_sdk._sound_light_port import (
    DEFAULT_SOUND_LIGHT_BAUDRATE,
    DEFAULT_SOUND_LIGHT_PORT,
    DEFAULT_SOUND_LIGHT_TIMEOUT_S,
    MUTE_COMMAND_ARGS as _SOUND_LIGHT_MUTE_ARGS,
    REQUEST_TOPIC as _SOUND_LIGHT_REQUEST_TOPIC,
    SOUND_LIGHT_EVENTS,
)
from contest_sdk._sound_light_port import SoundLightPort as _SoundLightPort
from contest_sdk._sound_light_port import build_request as _build_sound_light_request
from contest_sdk._sound_light_port import encode_command as _encode_sound_light_command
from contest_sdk.exceptions import (
    ActionFailedError,
    DetectionTimeoutError,
    GotoTimeoutError,
    GotoUnreachableError,
    LandTimeoutError,
    SoundLightError,
    TakeoffTimeoutError,
)
from contest_sdk.geometry_helpers import (
    generate_ground_scan_waypoints as _generate_ground_scan_waypoints,
)
from contest_sdk.geometry_helpers import (
    generate_orbit_waypoints as _generate_orbit_waypoints,
)
from contest_sdk.geometry_helpers import goto_stalled as _goto_stalled
from contest_sdk.geometry_helpers import (
    pull_waypoints_out_of_circles as _pull_waypoints_out_of_circles,
)
from contest_sdk.mission_state_helpers import VALID_MISSION_STATES, is_valid_mission_state
from contest_sdk.reliability import ReliableEventChannel

# `VALID_MISSION_STATES`/`is_valid_mission_state()`的vendor实现+"为什么支持
# 冒号子状态不需要额外扩展"的说明，见`mission_state_helpers.py`模块头
# （单独拆出来是为了让这段纯字符串校验逻辑能在不装ROS2环境的情况下被
# 独立单元测试，见该文件文件头"为什么单独拆成这一个文件"一节）。


# ---------------------------------------------------------------------------
# `sdk.trigger_alarm()`的占位GCS接口配置——方案2.1节第9条/链路D明确写了
# "具体接口方式目前是未知数，需要先跟GCS开发者对接确定"，这次实现只能先
# 占位。选`HTTP POST`是三个候选（HTTP/串口/USB继电器）里最容易占位、
# 最容易在没有真实硬件的情况下先把"发出去、等确认、失败不崩溃"这套壳子
# 搭起来的一种，具体URL/协议格式等GCS那边定下来后再改，不需要改调用方
# （`trigger_alarm()`的方法签名不用变）。
# ---------------------------------------------------------------------------
#: ⚠️ 占位地址——GCS后端目前没有这个接口，等跟GCS开发者对接确定协议之后
#: 再改成真实地址。选一个明显不会被误当成真实生产地址的写法（内网私有
#: 地址段+一看就知道是占位的路径），避免以后忘了改还照着这个值发请求。
DEFAULT_ALARM_URL = 'http://gcs-host.invalid:8000/api/alarm/trigger'
DEFAULT_ALARM_TIMEOUT_S = 3.0
DEFAULT_ALARM_RETRIES = 2

# ---------------------------------------------------------------------------
# `sdk.play_sound_light()`的机载声光反馈板配置——协议已经从《声光反馈
# 程序接口.xlsx》拿到并在真实硬件（`/dev/ttyUSB0`）上验证过九条命令全部
# 正常（2026-09-16，见`DEBUG_JOURNAL.md`同日条目），不是`trigger_alarm()`
# 那种"协议未知只能先占位"的情况，可以直接实现真实串口通信。这个板子
# 挂在每架飞机自己的机载电脑上（选手自己接线），不是GCS共享设备，所以
# 跟`trigger_alarm()`（方案2.1节第9条/链路D，触发GCS所在机器的装置）是
# 两个独立的能力，不是同一个功能的两种实现——不要把两者合并。
#
# 端口号支持`SOUND_LIGHT_PORT`环境变量覆盖（见`__init__`里`os.environ.get()`
# 那一行）：USB转串口设备在Linux下的`/dev/ttyUSB*`编号不是稳定的，插拔顺序/
# 多台设备同时接入时都可能变化，两架飞机各自的机载电脑上这个板子也未必
# 都刚好是`ttyUSB0`——写死成常量的话每次环境不一样都要改代码重新pip
# install，改成环境变量之后选手/部署脚本在容器`docker run -e SOUND_LIGHT_
# PORT=/dev/ttyUSB1 ...`或者直接`export`就能覆盖，不用碰SDK代码本身。
# 没设置这个环境变量时才落回默认值（`DEFAULT_SOUND_LIGHT_PORT`，定义在
# `_sound_light_port.py`，常驻程序共用）。
#
# 2026-09-21：地面站只有一块板子、两架飞机的任务进程共用，默认改成
# "server"模式——SDK不再自己开串口，而是往`/sound_light/request`话题发
# 请求，由常驻程序`sound_light_server.py`独占串口、排队发送（理由见该
# 文件模块头）。`SOUND_LIGHT_MODE=direct`退回原来的"本进程直接开串口"，
# 只适合单进程、板子直接插在本机的调试场景，跟常驻程序不能同时用
# （串口以exclusive方式打开，后开的一方会报错）。
# ---------------------------------------------------------------------------
SOUND_LIGHT_MODES = ('server', 'direct')
#: `takeoff()`/`land()`自动播报用：角色 -> 事件名前缀（表格里"侦察机"/"任务机"）。
_SOUND_LIGHT_ROLE_PREFIX = {'recon': '侦察机', 'supply': '任务机'}

@dataclass(frozen=True)
class ServoSpec:
    """一路舵机的硬件接线/飞控配置，必须跟飞控里的参数一致（改了飞控参数
    就同步改这里）。"""
    actuator_set: int   # PWM_MAIN_FUNCn = 300 + actuator_set（Peripheral via Actuator Set N）
    output: str         # 接在飞控哪个输出口，只用于报错/打印
    pwm_min: int        # = PWM_MAIN_MINn，对应归一化值-1
    pwm_max: int        # = PWM_MAIN_MAXn，对应归一化值+1


# `sdk.set_servo()`用的舵机表：飞机编号 -> {舵机编号: 配置}。
# 2026-09-21用户给定：两个舵机都在NX02上，MAIN7/MAIN9，50Hz，PWM 800~2000。
# MAIN7(TIM2组)和MAIN9(TIM3组)各自的组频率都要设成50Hz（PWM_MAIN_TIM2/
# PWM_MAIN_TIM3=50），上锁时也要能动需要COM_PREARM_MODE=2——这些都是飞控
# 侧设置，SDK这边只负责按表把PWM换算成归一化值发出去。
SERVO_CONFIG: Dict[str, Dict[int, ServoSpec]] = {
    'NX01': {
        # 2026-09-23用户要求新增：侦察机也要能驱动舵机发射（高层火情那一步）。
        # ⚠️ 接线和飞控参数还没在NX01真机上核对过，暂按NX02的第一路照搬
        # （MAIN7、PWM 800~2000）。上真机前必须确认：PWM_MAIN_FUNC7=301、
        # 该组频率PWM_MAIN_TIM2=50、COM_PREARM_MODE=2，以及发射机构确实接在
        # MAIN7——接错口会驱动别的输出。核对后如有不同，改这里一行即可。
        1: ServoSpec(actuator_set=1, output='MAIN7', pwm_min=800, pwm_max=2000),
    },
    'NX02': {
        1: ServoSpec(actuator_set=1, output='MAIN7', pwm_min=800, pwm_max=2000),
        2: ServoSpec(actuator_set=2, output='MAIN9', pwm_min=800, pwm_max=2000),
    },
}


def servo_pwm_to_normalized(spec: ServoSpec, pwm: int) -> float:
    """PWM微秒值 -> `MAV_CMD_DO_SET_ACTUATOR`用的-1~1。PX4输出侧按
    `interpolate(v, -1, 1, MIN, MAX)`再四舍五入换算回PWM，这里是它的逆运算。"""
    if not spec.pwm_min <= pwm <= spec.pwm_max:
        raise ValueError(
            f"舵机({spec.output})的PWM必须在{spec.pwm_min}~{spec.pwm_max}之间，收到{pwm}"
        )
    return (pwm - spec.pwm_min) / (spec.pwm_max - spec.pwm_min) * 2.0 - 1.0


#: `send_to_teammate()`默认总超时——方案2.3节"延迟预算"建议区间(30-60秒)
#: 的中间值，跟`reliability.py`的`DEFAULT_TIMEOUT_S`保持一致，这里单独
#: 声明一份是因为选手看到的是`capabilities.py`这一层的默认值，`reliability.
#: py`那份默认值是`send_and_wait_ack()`自己的默认参数，两者语义上是同一个
#:数字，物理上各自独立声明，改一边不会悄悄影响另一边的默认行为。
DEFAULT_SEND_TO_TEAMMATE_TIMEOUT_S = 45.0

#: `takeoff(height_m=...)` 允许的起飞高度范围（米）。下限避开地效区，上限见那个
#: 方法里的说明（规划器天花板约束）。
TAKEOFF_HEIGHT_MIN_M = 0.5
TAKEOFF_HEIGHT_MAX_M = 2.8

#: 所有阻塞方法的"进度打印"节流间隔——方案2.2.2节建议"每2-3秒打印一行"，
#: 取中间值2.5秒。
PROGRESS_INTERVAL_S = 2.5

#: `takeoff()`判定"真正起飞到位、状态已转换"用的位置稳定窗口——见
#: `takeoff()`docstring的完整说明。窗口时长跟允许的抖动幅度都是凭经验
#: 取的"够用"量级，不是精确计算出来的：窗口太短容易在爬升途中的短暂
#: 匀速段被误判成"已经稳定"，太长会让`takeoff()`不必要地多等。
TAKEOFF_STABLE_WINDOW_S = 1.5
#: `takeoff()`额外要求"确实已经爬升过"才开始判断稳定——2026-09-14实测
#: 发现只判断"位置稳定"这一条本身有漏洞：刚解锁、电机预热阶段还没真正
#: 起飞时，飞机停在地面纹丝不动，同样满足"位置稳定"，会被误判成"起飞
#: 完成"（实测复现：`takeoff()`3秒就返回，但返回时高度只有0.28米，
#: 根本没有真正爬升）。改成额外要求当前高度比"armed刚确认那一刻"的
#: 高度至少爬升了这么多，才允许开始判断稳定，排除"还没起飞就被判成
#: 稳定"这个假阳性。
#:
#: ⚠️ 2026-09-14第二次实测踩坑（同一天，修完上面那个假阳性之后又发现
#: 一个）：这个值最初跟下面`TAKEOFF_STABLE_POS_TOLERANCE_M`用的是同一个
#: 数字（0.3米）——爬升过程如果不是匀速、中途有一段短暂的减速/停顿
#: （仿真里偶发），飞机可能刚好在"累计爬升略微超过0.3米"这个时间点
#: 附近停顿了一下，窗口内的位置变化恰好也小于0.3米容差，两个条件同时
#: 被满足，`takeoff()`在只爬升到约0.35米（远低于`ctrl_param_fpv.yaml`
#: 里`takeoff_height: 1.0`那个真实目标高度）时就被判定"稳定"提前返回
#: ——用户实测直接指出这个问题（"必须起飞完成，到达指定高度后，再给
#: 航点"）。根子是这两个阈值凑巧相等，"刚够上最低爬升要求"和"位置已经
#: 不再变化"这两件事在爬升曲线的减速点附近可能同时为真，给了假阳性
#: 可乘之机。改成把最低爬升量提到明显高于位置稳定容差的量级（0.3->0.6
#: 米，是稳定容差的2倍）——不需要知道`takeoff_height`具体配的是1.0米
#: （设计意图不变，见上面注释），只需要保证"最低爬升量"和"稳定容差"
#: 这两个数字之间留出足够余量，让爬升曲线中途的普通减速/停顿不足以
#: 同时满足这两个条件，必须是真的爬到接近目标高度附近才会两个条件
#: 一起成立。
TAKEOFF_MIN_CLIMB_M = 0.6
#: 2026-09-14实测订正：第一版用0.15米，实测起飞爬升完成后的正常悬停
#: 抖动本身就有0.3~0.4米量级的振荡（不是没收敛，是这套仿真在uwb_imu
#: 模式+pt4ctrl这套组合下的正常悬停精度），0.15米太严格，30秒超时窗口
#: 内等不到。改用跟`ego_planner_bridge/rviz_goal_bridge_node.py::
#: ARRIVAL_THRESHOLD_M`同一个量级（0.3米）——这个值已经是这个项目里
#: "到点/稳定"判据的既有标准，不是另外凭空定的新数字。
TAKEOFF_STABLE_POS_TOLERANCE_M = 0.3

#: 2026-09-17新增：用户要求"起飞后应该悬停5秒，稳定后出发，这是所有
#: 起飞都应该用的流程"——`_pos_stable()`判定的"稳定"只是"最近1.5秒窗口
#: 内位置没有明显漂移"，不等于"已经充分稳定、可以放心接第一个goto()"，
#: 额外强制悬停这么久再返回，给控制器更多收敛裕量。
HOVER_AFTER_TAKEOFF_S = 5.0

#: 探测机载 Takeoff action server 是否存在的等待时长（2026-09-20新增）。
#: 只是探测，不是等起飞——server 不在就立刻退回老路，不能在这里白等。
#: "已在空中"判定阈值（2026-09-20实测补上，跟机载 takeoff_monitor_node
#: 的 already_airborne_z_m 参数同义）：调 takeoff() 时飞机已经解锁且高度
#: 超过这个值，直接返回成功，不重复下发起飞指令。
#:
#: 为什么需要：爬升基线取的是"armed那一刻的高度"，飞机已经在悬停时再调
#: 一次 takeoff()，基线就是当前悬停高度，"相对基线再爬升
#: TAKEOFF_MIN_CLIMB_M"这个条件永远不成立，于是白等到超时才报错——实测
#: 复现过。这条短路让两条路径（action 与退回的选手侧判定）行为一致。
ALREADY_AIRBORNE_Z_M = 0.5

#: 等"僚机进入跟随待命"的上限（2026-09-21）。原来这里等的是"入列完成"
#: （feedback 报 track），但 track 要等长机走起来、长机又要等僚机报准备好
#: ——互相等会死锁，所以改成等 hold（僚机已接管控制权、原地保持）。这个
#: 信号在 goal 被接受后一两个 feedback 周期内就会出现，不需要很长，
#: 20秒足够；入列/保持本身的超时由机载 server 的 join_timeout_s /
#: leader_start_timeout_s 负责。
FORMATION_STANDBY_WAIT_S = 20.0

#: 判定"起飞前 yaw 已稳定"的采样条件（2026-09-21）。
#:
#: 为什么要等稳定：`takeoff()` 要把起飞前的真实机头朝向记下来，再用
#: `set_yaw_mode_constant()` 命令飞机全程保持这个朝向。但里程计的**第一帧**
#: 姿态往往是单位四元数（yaw=0）——定位源还没收敛、还没把 UWB 的绝对朝向
#: 融合进去。实测：飞机物理朝向（Gazebo 真值）全程是 90.0°，里程计稳定后
#: 也是 89.7~90.4°，而 `get_current_yaw()` 在起飞前读到的是 **0.0°**，于是
#: `set_yaw_mode_constant(0°)` 反过来把机头从 90° 转到了 0°——本来是要
#: "保持朝向不变"的一行代码，成了把朝向转走的元凶。
#:
#: 判据：连续 YAW_SETTLE_SAMPLES 次采样的最大偏差小于
#: YAW_SETTLE_TOLERANCE_DEG，取这批样本的圆周均值。
YAW_SETTLE_SAMPLES = 8
YAW_SETTLE_INTERVAL_S = 0.25
YAW_SETTLE_TOLERANCE_DEG = 3.0
YAW_SETTLE_TIMEOUT_S = 20.0

#: 里程计 yaw 与 UWB 绝对 yaw 的允许偏差（度，2026-09-21 追加）。
#:
#: 为什么只看"稳定"不够：定位源还没收敛时，里程计发的是**单位四元数**
#: （yaw 恒为 0.0），方差精确等于零——"稳定"判据会把"稳定地错"当成"稳定地
#: 对"直接采纳。实测 NX02 前 14 秒 odom_yaw 一直是 0.0，而同期 UWB 报
#: 87~93 度、Gazebo 真值 90 度，于是 pretakeoff_yaw 记成了 0。
#:
#: `uwb/pose_abs` 的 yaw 从第一帧起就是对的（仿真里 uwb_ground_truth_node、
#: 真机上 nlink_pose_bridge_node 都直接给绝对朝向，不需要收敛过程），而且
#: 跟里程计是同一套 yaw 约定（收敛后实测 odom 89.9 / uwb 90.0 / 真值 90.0
#: 三者一致）。所以用它当交叉校验：两个源对得上，才说明里程计真的收敛了。
#:
#: 10 度的余量覆盖 UWB 自身噪声（uwb_ground_truth_node 的 yaw_noise_std_deg
#: 默认 2 度）加上两条链路的时间差。UWB 话题拿不到时退回"只看稳定"，不因为
#: 少一个校验源就拒绝起飞。
YAW_SOURCE_AGREE_DEG = 10.0

#: 等机载起飞 action 返回结果的**唯一**兜底上限（2026-09-21）。
#:
#: 为什么客户端这边不再自己算时间预算：判定权已经下沉到机载
#: `takeoff_monitor_node`，它对每个阶段都有自己的超时
#: （`preflight_timeout_s` / `armed_timeout_s` / `goal.timeout_s`），
#: 任何情况下都会返回一个带 stage 的结果。客户端再维护一套并行的预算，
#: 两套算法就必然会在某些时序下给出相反的结论——实测出现过"机载报
#: success=True、客户端同时抛 TakeoffTimeoutError"（起因是机载的总超时
#: 从"就绪那一刻"起算，而客户端从"发出 goal"起算，机载等定位收敛的几十秒
#: 全被算进了客户端的预算里）。
#:
#: 所以这里只保留一个明显大于机载所有阶段超时之和的兜底值，它唯一的作用
#: 是"机载节点挂了、永远不返回结果"这种情况下不要无限等。正常路径上永远
#: 不会走到这个值——失败也是机载先返回失败结果。
#: 这个值必须大于机载各阶段超时之和，否则客户端会先于服务端到期，又变回
#: "两端各有一套预算、结论可能相反"那个已经修过一次的问题。当前机载是
#: preflight_timeout_s(45) + armed_timeout_s(45) + goal.timeout_s(60) = 150，
#: 取 240 留余量。改机载那三个值时必须回头核这一行。
TAKEOFF_ACTION_HARD_TIMEOUT_S = 240.0

#: goto() 卡住检测（2026-09-21）：规划器接收目标之后，飞机在这么多秒内
#: 三维位移小于 GOTO_STALL_MIN_MOVE_M、并且离目标还有 GOTO_STALL_MIN_DIST_M
#: 以上，就判定到不了，尽快抛 GotoUnreachableError，不再干等满 timeout。
#: 三个数的取值理由见 geometry_helpers.goto_stalled() 的说明。
GOTO_STALL_WINDOW_S = 5.0
GOTO_STALL_MIN_MOVE_M = 0.5
GOTO_STALL_MIN_DIST_M = 0.5

ACTION_SERVER_PROBE_TIMEOUT_S = 2.0

#: 等 action goal 被 accept/reject 的时长。这一步是一次 service 往返，
#: 正常是毫秒级，给 5 秒是留给 DDS 发现没跟上的余量。
ACTION_GOAL_ACCEPT_TIMEOUT_S = 5.0

#: 2026-09-17新增：起飞前置检查的超时时间——UWB/里程计数据就绪、PX4
#: 飞控连接，这两项检查各自用这个超时（不复用`timeout`参数，因为这两项
#: 正常情况下应该在容器/飞控完全启动后的几秒内就绪，用总的起飞超时
#: 时间量级来卡这两步没有意义，独立给一个更短的超时能更快暴露"根本没
#: 连上"这类问题，不用干等到总超时才报错）。
PREFLIGHT_CHECK_TIMEOUT_S = 20.0


@dataclass
class Detection:
    """`sdk.wait_for_detection()`的返回值类型——`vision_msgs/Detection2D`
    的简化封装（选手不需要认识`results[0].hypothesis.class_id`这种嵌套
    ROS消息字段，方案2.2.1节"封装完整性"要求）。

    ⚠️ **字段命名的选择，需要在这里说明清楚（方案2.2.1节示例代码给的字段
    名是`det.local_x`/`det.local_y`，这次实现改了，是刻意的、有判断依据
    的改动，不是笔误）**：

    示例代码里`local_x`/`local_y`这个命名暗示"这已经是能直接喂给
    `sdk.goto()`的局部坐标"，但单帧2D检测（视觉节点发布的`vision_msgs/
    Detection2DArray`只有像素级bbox，没有深度信息）本身解不出任何世界/
    局部坐标——这个项目里`precision_servo_node.py`/`fire_pillar_aim_
    node.py`都专门写了大段注释解释"没有用AprilTag库自带的6DOF位姿解算"、
    "改用像素偏移+当前高度+相机FOV换算"，而且换算方式随用途不同（`center_
    only`只算水平偏移；`precision_land`同样只算水平偏移；`fire_pillar_
    aim_node`用的是激光雷达立柱朝向+方位角吸附，完全不看bbox几何）——
    这些"从像素到坐标"的换算逻辑分散在各自的节点里，且各自依赖"当前高度"
    这个运行时状态，不是`wait_for_detection()`这一个通用检测接口能一次性
    做对、做全的事。如果这里沿用`local_x`/`local_y`，选手很容易望文生义
    直接拿这两个字段去调`sdk.goto()`，实际上传的其实是像素坐标，会让飞机
    飞去完全不相关的位置——这是"封装完整性"反而弄巧成拙的一个例子（为了
    贴合示例代码字面，给选手埋了一个坐标语义不对但类型系统上看不出错的
    坑）。所以这里改成`bbox_x`/`bbox_y`+`bbox_width`/`bbox_height`，命名
    上明确标注"这是图像里的像素位置"，选手一看字段名就知道不能直接拿去
    `goto()`，需要用的话要走`center_on_target()`（拿到的是真正的收敛后
    世界坐标）或者自己实现换算逻辑。

    Attributes:
        class_id: 检测到的类别/标签字符串（比如`'apriltag:2'`），跟
            调用`wait_for_detection(class_id=...)`时传的字符串完全一致。
        bbox_x/bbox_y: 检测框中心在图像里的像素坐标（不是世界/局部坐标，
            见上面的说明）。
        bbox_width/bbox_height: 检测框的像素宽高。
        score: 检测置信度（`[0, 1]`区间，具体含义取决于视觉检测节点本身
            怎么定义，SDK这一层不做任何加工）。
        stamp_sec: 这次检测的时间戳（秒，来自消息自带的`header.stamp`，
            已经转换成单个`float`，不是ROS`builtin_interfaces/Time`
            消息类型——2.2.1节"选手不应该接触裸ROS消息类型"同样适用于
            时间戳这种看起来很基础的字段）。
    """

    class_id: str
    bbox_x: float
    bbox_y: float
    bbox_width: float
    bbox_height: float
    score: float
    stamp_sec: float


@dataclass
class CenteredPose:
    """`sdk.center_on_target()`收敛完成后的返回值——`precision_servo_node`
    的`center_only`模式广播的`centered_pose`话题（`geometry_msgs/
    PointStamped`）的简化封装，只留水平坐标（该话题z字段本身固定填0.0，
    见`precision_servo_node.py::_publish_centered_pose()`的说明，这里
    不暴露没有意义的z字段，避免选手误以为这也是有效信息）。
    """

    x: float
    y: float


@dataclass
class TargetPosition:
    """`sdk.locate_target()`的返回值：目标在**飞机自己的局部系**下的实际
    坐标（跟`get_local_position()`同一个系，要报世界坐标用`local_to_world()`
    转）。由飞行栈`target_locate_node`用相机内参、拍照时刻的位姿和测距仪
    估的地面高度算出来，不是飞机自己的位置。

    `spread_m`是参与取中位数的各帧结果离中位数的最大水平距离，几厘米说明
    结果稳定；零点几米以上说明各帧不一致（比如目标在画面边缘时进出画面），
    可以飞近一点再测一次。
    """

    x: float
    y: float
    z: float
    samples: int
    spread_m: float


def _stamp_to_sec(stamp: Any) -> float:
    """`builtin_interfaces/Time`（`header.stamp`）-> 单个float秒数。"""
    return stamp.sec + stamp.nanosec * 1e-9


def _make_parameter(name: str, value: Any) -> Parameter:
    """把一个Python值包成`rcl_interfaces/Parameter`，供`~/set_parameters`
    service用——跟`fire_pillar_aim_node.py::_set_relay_mode()`同一种写法
    习惯（这个项目里"调标准`~/set_parameters`service"的既有模式）。

    ⚠️ `bool`必须在`int`之前判断——Python里`isinstance(True, int)`是
    `True`（`bool`是`int`的子类），如果先判断`int`，传入的`enabled=True`
    会被错误地当成`PARAMETER_INTEGER`打包，跟`formation_follower_node.py`
    实际`declare_parameter('enabled', False)`声明的`bool`类型不匹配，
    `~/set_parameters`会直接拒绝这次调用（ROS2参数系统按类型严格匹配）。
    """
    param = Parameter()
    param.name = name
    if isinstance(value, bool):
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, int):
        param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    elif isinstance(value, float):
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    elif isinstance(value, str):
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    elif isinstance(value, (list, tuple)) and all(
            isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
        # 2026-09-24新增：double数组。编队节点的`leg_route_xy`（航线，一串
        # x,y）是第一个用到数组参数的地方——节点那边也必须按 double 数组声明，
        # ROS2 参数系统按类型严格匹配，声明成 int 数组会被直接拒绝。
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                     double_array_value=[float(v) for v in value])
    else:
        raise TypeError(
            f"_make_parameter不支持的参数值类型: {type(value)!r}（name={name!r}，"
            f"value={value!r}）——contest_sdk内部实现bug，不是选手能触发的场景，"
            f"选手调用的所有能力方法参数类型都是SDK自己内部转换好的。"
        )
    return param


class DroneSDK:
    """选手真正调用的入口类（方案2.2节`capabilities.py`唯一对外暴露的类）。

    构造参数`namespace`/`role`/`teammate_namespace`三个都必填（方案1.5节：
    双机硬件/软件完全对称，角色是运行时参数，不能焊死在NX01/NX02身上）。
    选手代码大概长这样（方案2.2.1节示例）::

        from contest_sdk import DroneSDK
        sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
        sdk.takeoff()
        if sdk.role == 'supply':
            sdk.start_formation_follow(follow_distance_m=3.5)
        sdk.goto(7, -10, 1.5)
        det = sdk.wait_for_detection('apriltag:2', timeout=60)
        sdk.send_to_teammate('ground_fire_found', x=det.bbox_x, y=det.bbox_y)
        sdk.do_action('grab_supply')
        sdk.play_sound_light('侦察机发现地面火情')
    """

    #: `play_sound_light(event=...)`的20个合法取值 -> (R, G, B, 声音编号, 循环次数, 间隔ms)，
    #: 挂成类属性方便选手用`DroneSDK.SOUND_LIGHT_EVENTS`查有哪些事件名，
    #: 不需要单独`import`内部模块（跟本文件"只暴露DroneSDK一个类"的约定
    #: 保持一致，见`__init__.py`文件头说明）。
    SOUND_LIGHT_EVENTS = SOUND_LIGHT_EVENTS

    def __init__(self, namespace: str, role: str, teammate_namespace: str):
        if not namespace:
            raise ValueError('namespace不能为空——必须显式传入这架飞机的命名空间（方案1.5节）。')
        if not role:
            raise ValueError('role不能为空——必须显式传入这次任务里这架飞机扮演的角色（方案1.5节）。')
        if not teammate_namespace:
            raise ValueError('teammate_namespace不能为空——必须显式传入队友那架飞机的命名空间（方案1.5节/2.1节第3条）。')

        self.namespace = namespace.strip('/')
        # takeoff()/land() 自动播的角色声光，每种动作只播第一次——见
        # _play_role_sound_light()。任务流程里一趟要起降三次，重复播是噪声。
        self._role_sound_light_done: set = set()
        self.role = role
        self.teammate_namespace = teammate_namespace.strip('/')

        # ROS_DOMAIN_ID一致性校验已经在RclpyRuntime构造时做了（B2完成的
        # 部分），这里不重复实现，只是转调。
        self._runtime = RclpyRuntime(self.namespace)
        self._node = self._runtime.node

        self._target_frame = f'{self.namespace}/odom'

        # L3：跨机可靠事件通道（B4完成的部分），每个DroneSDK实例一个即可，
        # 不需要每次send_to_teammate都新建——见reliability.py文件头说明。
        self._event_channel = ReliableEventChannel(self._node, self.namespace, self.teammate_namespace)

        # TF：world_to_local()内部使用，只在SDK内部处理frame_id概念（2.2.1
        # 节要求），跟`ego_planner_bridge/rviz_goal_bridge_node.py`的
        # `_to_local()`用的是同一套TF查询方式（world -> {namespace}/odom）。
        from tf2_ros import Buffer, TransformListener  # 延迟import，跟reliability.py同样的
        self._tf_buffer = Buffer()                       # 理由：只有真的需要TF功能时才要求
        self._tf_listener = TransformListener(self._tf_buffer, self._node)  # 环境里装好tf2_ros。

        self._alarm_url = DEFAULT_ALARM_URL
        self._alarm_timeout_s = DEFAULT_ALARM_TIMEOUT_S

        self._sound_light_mode = os.environ.get('SOUND_LIGHT_MODE', 'server')
        if self._sound_light_mode not in SOUND_LIGHT_MODES:
            raise ValueError(
                f"SOUND_LIGHT_MODE环境变量只能是{SOUND_LIGHT_MODES}之一，当前值={self._sound_light_mode!r}"
            )
        # server模式的请求发布者，第一次play_sound_light()时才创建（跟下面
        # 直连模式的懒加载同样的理由：不用声光的任务不应该多出一个发布者）。
        self._sound_light_pub = None
        self._sound_light_port = os.environ.get('SOUND_LIGHT_PORT', DEFAULT_SOUND_LIGHT_PORT)
        self._sound_light_baudrate = DEFAULT_SOUND_LIGHT_BAUDRATE
        self._sound_light_timeout_s = DEFAULT_SOUND_LIGHT_TIMEOUT_S
        # 懒加载：真正调用`play_sound_light()`/`mute_sound_light()`之前不
        # 触碰串口——很多任务/仿真环境里根本没接这块硬件，构造`DroneSDK`
        # 不应该因为这个可选外设不存在就失败。一旦创建就贯穿整个`DroneSDK`
        # 生命周期复用同一个连接，不是每次发送都新开一个（见`_sound_light_
        # port.py`模块头"真机踩坑"说明——每次开关会踩中板子的DTR复位窗口，
        # 指令会被硬件默默吞掉）。
        self._sound_light_conn: Optional[_SoundLightPort] = None

        self._setup_transport()

        # 编队握手的四个事件，**构造时就把收件箱建好**。
        # 可靠事件通道是"先回 ACK 再查处理函数"，没注册的事件会被确认后丢弃；
        # 而起飞要二十多秒，谁先飞完谁就先发事件，另一边还没注册就永久丢。
        # 2026-10-01 实测两种翻车方式都出现过：
        #   · 航线丢 -> 僚机拿不到 leg_route，分段航向没了，机头全程不变，
        #     而且退化成"就地入列"；
        #   · 加了"长机等 READY 再发航线"之后，READY 自己也撞同一个坑 ->
        #     两边各等满 60 秒超时，白白多等两分钟才开始编队。
        # 老版是靠选手在起飞前手写 _Inbox 躲过去的，那正是要消除的负担。
        # 放在这里注册，不管选手什么时候调 takeoff/lead/follow 都不会漏。
        self.open_inbox(self.EV_READY, self.EV_ROUTE_PLAN,
                        self.EV_IN_POSITION, self.EV_ROUTE_DONE)

        self._progress(
            f"DroneSDK就绪（role={self.role}，teammate={self.teammate_namespace}）"
        )

    # ------------------------------------------------------------------
    # B6可调试性打点：SDK自己的进度打印/轮询等待helper（贯穿下面所有阻塞
    # 方法，见文件头说明）。
    # ------------------------------------------------------------------

    def _progress(self, text: str) -> None:
        """SDK自己的进度打印——现在只是包一层`print()`，选手看到的每一行
        都带`[namespace]`前缀方便双机同时跑时区分。方案2.2.2节要求"不直接
        用裸ROS2日志"，这里完全不碰`self._node.get_logger()`，往标准输出
        打印选手能看懂的中文人话，以后要改成输出到文件/GUI只需要改这一个
        函数，不需要改下面几十处调用点。
        """
        print(f'[{self.namespace}] {text}', flush=True)

    def _poll_until(
        self,
        check_fn: Callable[[], bool],
        timeout_s: Optional[float],
        progress_fn: Callable[[], None],
        poll_interval_s: float = 0.2,
    ) -> bool:
        """通用轮询等待：每`poll_interval_s`检查一次`check_fn()`是否为真，
        每`PROGRESS_INTERVAL_S`秒调用一次`progress_fn()`打印进度。

        `check_fn()`为真时立刻返回`True`；超过`timeout_s`秒仍未为真则
        返回`False`（不在这里抛异常——调用方各自知道该抛哪个具体异常
        类型+该往异常构造函数传什么参数，这个helper只负责"等到了/没等到"
        这个布尔判断，异常类型的选择留给调用方）。

        `timeout_s=None`表示**真的不设超时**（2026-09-15新增，用户明确
        要求"必须去掉，删除"，不是找个很大的数字糊弄过去）——会一直等到
        `check_fn()`为真才返回，调用方需要自己判断这样用合不合适（比如
        诊断测试里明确想看"到底能不能收敛"这个问题本身，不希望超时机制
        提前替它下结论）。仍然只在需要传`None`的调用点才会真的不设
        超时，其它调用点默认行为不变。

        不会忙等：`time.sleep()`的时长永远不超过`poll_interval_s`，也不会
        超过"离下次进度打印/离超时还有多久"，保证CPU占用很低，也保证
        超时判断不会因为睡太久而迟到太多。
        """
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        last_progress = 0.0
        while True:
            if check_fn():
                return True
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                return False
            if now - last_progress >= PROGRESS_INTERVAL_S:
                progress_fn()
                last_progress = now
            sleep_s = poll_interval_s if deadline is None else min(poll_interval_s, deadline - now)
            time.sleep(max(0.001, sleep_s))

    def _run_with_progress_ticker(
        self,
        work_fn: Callable[[], Any],
        progress_fn: Callable[[], None],
        interval_s: float = PROGRESS_INTERVAL_S,
    ) -> Any:
        """给"内部已经是一个不可拆分的阻塞调用"（比如`reliability.py`的
        `send_and_wait_ack()`——它自己有一整套等待逻辑，这个文件不应该
        也不能碰它内部的循环）补上B6要求的定期进度打印：另起一个后台
        daemon线程，每`interval_s`秒调一次`progress_fn()`，直到`work_fn()`
        在调用方线程里返回/抛异常为止再停掉。

        跟`_poll_until()`是同一个"打印进度"需求的两种实现方式：`_poll_
        until()`用在"这个方法本来就要自己写轮询循环"的场景（顺手在循环
        里打印）；这个方法用在"轮询循环已经被封装在别的模块里、这里只能
        拿到一个会阻塞的函数调用"的场景（`send_to_teammate()`/
        `reset_aim()`就是这种）。
        """
        stop_event = threading.Event()

        def _ticker() -> None:
            while not stop_event.wait(interval_s):
                progress_fn()

        ticker_thread = threading.Thread(target=_ticker, daemon=True)
        ticker_thread.start()
        try:
            return work_fn()
        finally:
            stop_event.set()
            ticker_thread.join(timeout=1.0)

    def _call_service_blocking(
        self,
        client: Any,
        request: Any,
        timeout_s: float,
    ) -> Tuple[bool, Optional[Any]]:
        """通用service阻塞调用：`(是否在超时内拿到响应, 响应对象或None)`。

        ⚠️ **不能用`rclpy.spin_until_future_complete()`**——见本文件文件头
        "为什么不用..."一节，`RclpyRuntime`已经在后台线程持续`spin()`这个
        节点，这里再自己spin一次会跟后台线程抢节点，直接`RuntimeError`。
        改成`call_async()`+`add_done_callback()`里用`threading.Event`
        置位，调用方线程只负责`sleep`等待——跟`reliability.py::send_and_
        wait_ack()`是同一个模式，那边的模块文档也详细记录过这个坑。
        """
        if not client.service_is_ready():
            # 快速探测一下：常见场景是目标节点还没起来/还没完成service
            # 发现，给一个不超过总超时的小窗口再确认一次，避免"节点其实
            # 马上就要起来了"这种边界情况被误判成"service不存在"。
            if not client.wait_for_service(timeout_sec=min(2.0, timeout_s)):
                return False, None

        event = threading.Event()
        box: Dict[str, Any] = {}

        def _on_done(fut: Any) -> None:
            try:
                box['response'] = fut.result()
            except Exception as exc:  # noqa: BLE001 - 转换成"没拿到响应"，不透传裸rclpy异常
                box['error'] = exc
            event.set()

        future = client.call_async(request)
        future.add_done_callback(_on_done)
        got = event.wait(timeout=timeout_s)
        if not got or 'response' not in box:
            return False, None
        return True, box['response']

    def _call_set_parameters_blocking(
        self,
        client: Any,
        params: Dict[str, Any],
        timeout_s: float,
    ) -> bool:
        """在`_call_service_blocking()`基础上，专门给`~/set_parameters`
        调用包一层"把dict转成Parameter列表+检查每个参数是否都被接受"。
        返回`True`表示全部参数都被对方接受；其余情况（service不可达/
        超时/任意一个参数被拒绝）返回`False`，由调用方决定抛哪个异常。
        """
        request = SetParameters.Request()
        request.parameters = [_make_parameter(name, value) for name, value in params.items()]
        ok, response = self._call_service_blocking(client, request, timeout_s)
        if not ok or response is None:
            return False
        return all(result.successful for result in response.results)

    # ------------------------------------------------------------------
    # 常驻通信资源的搭建（方案2.3节"链路A"第1条：常驻publisher，不要每次
    # 指令都新起一次性进程/新建对象）。所有publisher/subscriber/service
    # client只在这里创建一次，`takeoff()`/`goto()`/`do_action()`等方法
    # 反复调用时直接复用同一份，不重新创建。
    # ------------------------------------------------------------------

    def _setup_transport(self) -> None:
        from geometry_msgs.msg import PointStamped, PoseStamped
        from mavros_msgs.msg import State
        from sensor_msgs.msg import Range
        from nav_msgs.msg import Odometry, Path
        from quadrotor_msgs.msg import PositionCommand, TakeoffLand
        from std_msgs.msg import Empty, String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._TakeoffLand = TakeoffLand

        # 2026-09-20：机载起飞判定用的 action 类型，跟 TakeoffLand 同一个
        # 包（contestant-sdk 镜像本来就从 flight-stack 拷它的安装产物）。
        # quadrotor_msgs 还是旧版（没有 action 定义）时置 None，takeoff()
        # 会自动退回选手侧判定那条老路。
        try:
            from quadrotor_msgs.action import Takeoff as _TakeoffAction
            from rclpy.action import ActionClient as _ActionClient
            self._Takeoff = _TakeoffAction
            self._takeoff_action_cli = _ActionClient(self._node, _TakeoffAction, 'takeoff')
            from quadrotor_msgs.action import FormationFollow as _FormationAction
            self._FormationFollow = _FormationAction
            self._formation_action_cli = _ActionClient(
                self._node, _FormationAction, 'formation_follow')
        except ImportError:
            self._Takeoff = None
            self._takeoff_action_cli = None
            self._FormationFollow = None
            self._formation_action_cli = None
        self._Path = Path
        self._Empty = Empty

        # === 最新状态缓存（后台spin线程里的订阅回调更新，调用方线程
        # 轮询读取）——单个属性的整体赋值在CPython下是原子的（跟
        # reliability.py文件头的说明一致），这里不额外加锁。===
        self._armed: Optional[bool] = None
        # 2026-09-17新增：PX4飞控连接状态（`mavros/state.connected`，跟
        # `armed`是两个独立字段——`connected`只代表mavros跟FCU之间的
        # MAVLink链路通不通，`armed`代表飞控是否已解锁，起飞前置检查
        # 需要分别确认这两件事）。
        self._connected: Optional[bool] = None
        self._odom_xyz: Optional[Tuple[float, float, float]] = None
        # 2026-09-17新增：持续缓存当前yaw角（局部坐标系，跟goto()/set_yaw_
        # mode_constant()同一套约定），从里程计四元数换算，配合下面
        # takeoff()自动读取"起飞前真实yaw角"这个安全修复用——见
        # DEBUG_JOURNAL.md 2026-09-17"起飞掉高根因"记录：CONSTANT模式
        # 默认写死0.0弧度，但飞机实际停机朝向往往不是0度（比如NX01
        # spawn yaw=90度），起飞转入航点飞行那一刻控制器被迫大幅转向去
        # 匹配这个不match的0.0目标，跟位置/高度控制抢电机推力资源，这才是
        # "起飞后一转入飞行就急剧掉高、甚至直接掉地上"的真正根因——不是
        # 绕飞本身的yaw跟踪算法问题（那个问题更早之前就已经通过默认切到
        # CONSTANT模式处理过了），是CONSTANT目标值本身选错了。
        self._current_yaw: Optional[float] = None
        self._uwb_yaw: Optional[float] = None
        # takeoff()调用时刻实测的yaw角，绕飞结束后应该恢复到这个值（不是
        # 硬编码0.0）——mission.py里"绕飞时用POINT模式对准立柱，绕完/命中
        # 后恢复固定朝向"的收尾逻辑要用这个，不能再写死0.0。
        self.pretakeoff_yaw: Optional[float] = None
        self._waypoint_state: Optional[str] = None
        self._action_status: Optional[str] = None
        #: `do_action()`给每次触发生成goal_id用的自增计数器（见该方法
        #: docstring里"幂等去重+结果关联"一节）。
        self._action_goal_counter: int = 0
        #: 正在执行的编队 goal 句柄与 result future（stop 时用来 cancel）
        self._formation_goal_handle = None
        self._formation_result_future = None
        self._mission_state: Optional[str] = None
        self._servo_status: Optional[str] = None
        self._centered_pose_xy: Optional[Tuple[float, float]] = None
        self._fire_pillar_staging_pose_xyyaw: Optional[Tuple[float, float, float]] = None
        self._fire_pillar_aim_pose_xyyaw: Optional[Tuple[float, float, float]] = None

        # ---- 链路A：mavros/state（takeoff/land共用） ----
        self._node.create_subscription(State, 'mavros/state', self._on_mavros_state, 10)
        self._takeoff_land_pub = self._node.create_publisher(TakeoffLand, 'takeoff_land', 10)

        # ---- 自身里程计（进度打印用，非必须但让B6提示更有信息量） ----
        self._node.create_subscription(Odometry, 'dlio/odom_node/odom', self._on_odom, 10)

        # ---- 对地测距（定高雷达）：仿地飞行要用它才知道"离地"多高 ----
        # QoS 必须是 BEST_EFFORT：mavros 按 SensorDataQoS 发这条话题，默认的
        # RELIABLE 订阅会直接 QoS 不兼容、一条都收不到（flight-stack 的
        # formation_follower_node/uwb_imu_fusion_node 订同一条话题时也是这么写的）。
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self._agl: Optional[float] = None
        self._node.create_subscription(
            Range, 'mavros/hrlv_ez4_pub', self._on_range,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        # 只用于起飞前 yaw 的交叉校验，见 YAW_SOURCE_AGREE_DEG
        self._node.create_subscription(PoseStamped, 'uwb/pose_abs', self._on_uwb_pose, 10)

        # ---- goto()/cancel_goto()：waypoint_queue/waypoint_state/waypoint_cancel ----
        self._waypoint_queue_pub = self._node.create_publisher(Path, 'waypoint_queue', 10)
        self._waypoint_cancel_pub = self._node.create_publisher(Empty, 'waypoint_cancel', 10)
        self._node.create_subscription(String, 'waypoint_state', self._on_waypoint_state, 10)

        # ---- set_mission_state/get_mission_state ----
        self._mission_state_pub = self._node.create_publisher(String, 'mission_state', 10)
        self._node.create_subscription(String, 'mission_state', self._on_mission_state, 10)

        # ---- set_yaw_mode_velocity()/set_yaw_mode_constant()/set_yaw_mode_point() ----
        # 2026-09-15新增（D6绕飞"机头始终对准立柱中心"需求）：对应
        # `traj_server.cpp`（patches/ego_planner_traj_server_yaw_mode.patch）
        # 新增的两个话题，`Int32`选模式(0=VELOCITY/1=CONSTANT/2=POINT)、
        # `Point`带该模式的参数（CONSTANT用x，POINT用x/y）——两个都是纯
        # 发布、不需要等确认（`traj_server`那边没有回执机制，模式切换在
        # 下一帧cmdCallback()就会生效，不是"发了就一定立刻生效"这种需要
        # 轮询确认的场景，不套`_poll_until()`那一套）。
        from std_msgs.msg import Int32
        from geometry_msgs.msg import Point as _YawParamPoint
        self._yaw_mode_pub = self._node.create_publisher(Int32, 'traj_server/yaw_mode', 10)
        # 2026-09-24：仿地飞行的两个话题，跟上面 yaw 那两个同一个套路
        self._alt_mode_pub = self._node.create_publisher(Int32, 'traj_server/alt_mode', 10)
        self._alt_param_pub = self._node.create_publisher(
            _YawParamPoint, 'traj_server/alt_param', 10)
        self._yaw_param_pub = self._node.create_publisher(_YawParamPoint, 'traj_server/yaw_param', 10)

        # set_camera_view() 已删除（2026-09-30）：它发的是给
        # `{ns}_switchable_camera_joint` 的 JointTrajectory，而当前相机方案
        # （docker-compose.yml 里 CAMERA_TYPE_NX01/02=camera）生成的模型里
        # 是**两个独立的固定安装相机**，两个 camera_joint 都是 type='fixed'，
        # 那个可动关节根本不存在，调用是空转。选哪一路相机靠订阅对应话题
        # （见 capture_photo / wait_for_detection 的 camera 参数），不需要切视角。

        # ---- wait_for_detection ----
        # 常驻订阅（不是每次wait_for_detection都新建/销毁），最新一帧检测
        # 结果的原始消息缓存在_latest_detections，wait_for_detection()自己
        # 决定要不要拿这一帧、要不要继续等下一帧。
        self._latest_detections: Optional[Any] = None
        # 2026-09-22：按相机分别缓存最新一条。前视、下视两路检测都发到同一个
        # vision/detections，只靠 header.frame_id 区分；只存"最新一条"的话，
        # 两路交替发布，下视的检测有一半时间会被前视的空结果覆盖掉。
        self._latest_detections_by_frame: Dict[str, Any] = {}
        self._node.create_subscription(Detection2DArray, 'vision/detections', self._on_detections, 10)
        # 2026-09-22：飞行栈 target_locate_node 算好的目标实际坐标，只存最近
        # 一小段（locate_target() 只取调用之后新到的）。
        self._target_positions: List[Tuple[float, str, float, float, float]] = []
        self._target_positions_lock = threading.Lock()
        self._node.create_subscription(
            Detection3DArray, 'vision/target_positions', self._on_target_positions, 10)

        # ---- do_action：actuator_action_node ----
        self._node.create_subscription(String, 'action_status', self._on_action_status, 10)
        self._actuator_action_params_cli = self._node.create_client(
            SetParameters, 'actuator_action_node/set_parameters'
        )

        # ---- start_formation_follow/stop_formation_follow：formation_follower_node ----
        # 起飞高度是 pt4ctrl 的参数（auto_takeoff_land.takeoff_height），
        # takeoff(height_m=...) 要在发起飞指令之前先改它，见那个方法的说明。
        self._pt4ctrl_params_cli = self._node.create_client(
            SetParameters, 'pt4ctrl/set_parameters')
        self._formation_follower_params_cli = self._node.create_client(
            SetParameters, 'formation_follower_node/set_parameters'
        )

        # ---- center_on_target/precision_land_and_confirm：precision_servo_node ----
        self._node.create_subscription(String, 'servo_status', self._on_servo_status, 10)
        self._node.create_subscription(PointStamped, 'centered_pose', self._on_centered_pose, 10)
        self._precision_servo_params_cli = self._node.create_client(
            SetParameters, 'precision_servo_node/set_parameters'
        )
        # ---- set_max_vel：ego_planner 的巡航限速（2026-09-28）----
        self._ego_planner_params_cli = self._node.create_client(
            SetParameters, 'ego_planner_node/set_parameters'
        )
        self._ego_planner_get_params_cli = self._node.create_client(
            GetParameters, 'ego_planner_node/get_parameters'
        )
        # ---- 队友编队落后量（2026-09-28）：长机"照顾模式"用 ----
        # 订的是队友 formation_follower_node 发的诊断话题（绝对话题名，跨命名空间）。
        # 为什么不自己拿两机位置去算：外面算弧长用的折线基准、采样率、有没有
        # "向后延伸"那一段都跟跟随节点内部不一样，算出来的数不是控制回路看到的那个。
        from std_msgs.msg import Float64MultiArray as _F64MA
        self._teammate_lag_m: Optional[float] = None
        self._node.create_subscription(
            _F64MA, f'/{self.teammate_namespace}/formation_diag',
            self._on_teammate_formation_diag, 10)

        # ---- reset_aim：fire_pillar_aim_node ----
        self._reset_aim_cli = self._node.create_client(Trigger, 'fire_pillar_aim_node/reset_aim')

        # ---- set_actuator：MAV_CMD_DO_SET_ACTUATOR（一次性指令，PX4收到后
        # 自己持续保持这个输出值，不需要常驻发布循环，所以能直接放选手
        # 容器里调用——跟`do_action()`那种必须常驻flight-stack的RC-override
        # 持续发布/显式释放机制是两种不同的技术路线，不要合并，见
        # `set_actuator()`docstring）----
        from mavros_msgs.srv import CommandLong
        self._CommandLong = CommandLong
        self._command_long_cli = self._node.create_client(CommandLong, 'mavros/cmd/command')

        # ---- read_fire_pillar_staging_pose/read_fire_pillar_aim_pose：
        # fire_pillar_aim_node（方案1.2节/A1新增）——2026-09-13实现阶段D
        # 的mission.py时发现的缺口：D6子流程需要读这两个持续广播的等待
        # 点/瞄准点坐标，B3清单原来只顾着封装"触发+等完成信号"这类一次性
        # 调用，漏了"持续订阅一个状态话题、取最新值"这类能力（`get_
        # mission_state()`/`centered_pose`已经是这个模式的先例，这里补
        # 两个同类型的新话题，不是新发明一种模式）。这两个话题只有
        # `fire_pillar_aim_node`真正锁定高层火情之后才会开始发布，锁定
        # 前订阅不到任何消息，跟`centered_pose`只在收敛后才有数据是同一
        # 种"读不到就还没到那个阶段"的语义。
        self._node.create_subscription(
            PositionCommand, 'fire_pillar_staging_pose',
            self._on_fire_pillar_staging_pose, 10)
        self._node.create_subscription(
            PositionCommand, 'fire_pillar_aim_pose',
            self._on_fire_pillar_aim_pose, 10)

    # ---- 订阅回调（全部在RclpyRuntime的后台spin线程里被调用） ----

    def _on_mavros_state(self, msg: Any) -> None:
        self._armed = bool(msg.armed)
        self._connected = bool(msg.connected)

    def _on_odom(self, msg: Any) -> None:
        p = msg.pose.pose.position
        self._odom_xyz = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_range(self, msg: Any) -> None:
        r = float(msg.range)
        if msg.min_range <= r <= msg.max_range:     # 贴地/超量程的读数丢掉
            self._agl = r

    def _on_uwb_pose(self, msg: Any) -> None:
        q = msg.pose.orientation
        self._uwb_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_waypoint_state(self, msg: Any) -> None:
        self._waypoint_state = msg.data

    def _on_action_status(self, msg: Any) -> None:
        self._action_status = msg.data

    def _on_mission_state(self, msg: Any) -> None:
        self._mission_state = msg.data

    def _on_servo_status(self, msg: Any) -> None:
        self._servo_status = msg.data

    def _on_centered_pose(self, msg: Any) -> None:
        self._centered_pose_xy = (msg.point.x, msg.point.y)

    def _on_fire_pillar_staging_pose(self, msg: Any) -> None:
        self._fire_pillar_staging_pose_xyyaw = (
            msg.position.x, msg.position.y, msg.yaw)

    def _on_fire_pillar_aim_pose(self, msg: Any) -> None:
        self._fire_pillar_aim_pose_xyyaw = (
            msg.position.x, msg.position.y, msg.yaw)

    def _on_target_positions(self, msg: Any) -> None:
        now = time.monotonic()
        with self._target_positions_lock:
            for det in msg.detections:
                if det.results:
                    p = det.results[0].pose.pose.position
                    self._target_positions.append(
                        (now, det.results[0].hypothesis.class_id, p.x, p.y, p.z))
            del self._target_positions[:-200]

    def _on_detections(self, msg: Any) -> None:
        self._latest_detections = msg
        self._latest_detections_by_frame[msg.header.frame_id] = msg

    # ------------------------------------------------------------------
    # 2.1节能力1：统一的检测结果接口
    # ------------------------------------------------------------------

    def wait_for_detection(
        self, class_id: str, timeout: float, camera: Optional[str] = None,
    ) -> Detection:
        """阻塞等待`vision/detections`里出现`class_id`匹配的检测结果。

        Args:
            class_id: 要等待的类别/标签字符串（比如`'apriltag:2'`），跟
                视觉检测节点实际发布的`results[0].hypothesis.class_id`
                完全一致才算匹配（大小写敏感）。
            timeout: 超时秒数。
            camera: 只认哪一路相机的检测（`'down'`/`'front'`），默认`None`
                不限（跟原来行为一致）。2026-09-22新增：地面火情应该只认
                下视相机——前视相机斜着看到地面标签也会命中，那时飞机还在
                火点前方两三米，"发现即悬停"就停错了地方。

        Returns:
            `Detection`实例（见该类docstring关于字段命名的说明）。

        Raises:
            DetectionTimeoutError: 超过`timeout`秒仍未出现匹配的检测结果。
        """
        holder: Dict[str, Detection] = {}

        def _check() -> bool:
            if camera is None:
                # 不限相机时查每一路各自的最新一条，不能只看"全局最新一条"：
                # 检测节点每帧都发布（没检测到就发空数组，2026-09-22 起），
                # 两路交替发，只看全局最新的话，一路的命中随时会被另一路紧接着
                # 发来的空结果覆盖掉。
                msgs = list(self._latest_detections_by_frame.values())
            else:
                # frame_id 形如 NX01_camera_down_optical_frame
                tag = f'_camera_{camera}_'
                msgs = [m for fid, m in list(self._latest_detections_by_frame.items()) if tag in fid]
            for msg in msgs:
                if _match(msg):
                    return True
            return False

        def _match(msg: Any) -> bool:
            for det in msg.detections:
                if det.results and det.results[0].hypothesis.class_id == class_id:
                    holder['det'] = Detection(
                        class_id=class_id,
                        bbox_x=det.bbox.center.position.x,
                        bbox_y=det.bbox.center.position.y,
                        bbox_width=det.bbox.size_x,
                        bbox_height=det.bbox.size_y,
                        score=det.results[0].hypothesis.score,
                        stamp_sec=_stamp_to_sec(det.header.stamp),
                    )
                    return True
            return False

        ok = self._poll_until(
            _check,
            timeout,
            lambda: self._progress(f"等待检测目标'{class_id}'出现…"),
        )
        if not ok:
            raise DetectionTimeoutError(class_id=class_id, timeout_s=timeout, namespace=self.namespace)
        return holder['det']

    def clear_detections(self, camera: Optional[str] = None) -> None:
        """丢掉已经缓存的检测结果，让下一次`wait_for_detection()`只认**之后**
        新收到的帧。

        2026-09-23新增。`wait_for_detection()`查的是"每路相机最新一条检测
        消息"的缓存，而检测节点是每帧都发（约7Hz），所以缓存里那条可能是
        0.1秒前、飞机还在动/机头还在转的时候拍的——"停稳对准之后再测一次"
        这种要求光靠调用顺序保证不了，实测就出现过"机头转动过程中检出目标"。
        在开始测之前调一次这个方法，缓存清空，拿到的第一条一定是清空之后
        新拍的帧。

        Args:
            camera: 只清某一路（`'down'`/`'front'`），默认全清。
        """
        if camera is None:
            self._latest_detections_by_frame.clear()
            self._latest_detections = None
            return
        tag = f'_camera_{camera}_'
        for fid in [k for k in self._latest_detections_by_frame if tag in k]:
            del self._latest_detections_by_frame[fid]

    def locate_target(self, class_id: str, timeout: float = 5.0, samples: int = 5) -> TargetPosition:
        """目标在地面上的**实际坐标**（飞机自己的局部系）。

        坐标由飞行栈的`target_locate_node`算：下视相机检测结果的像素位置 +
        相机内参 + 拍照时刻的位姿（按图像时间戳插值，飞行中按速度外推补偿
        检测延迟）+ 测距仪估的地面高度。这里只等调用之后新到的`samples`帧
        结果、取中位数，飞行中调用也可以。只对下视相机看到的目标有效。

        Raises:
            DetectionTimeoutError: `timeout`秒内没凑够`samples`帧（目标不在
                下视画面里，或者飞行栈没起`target_locate_node`）。
        """
        t0 = time.monotonic()

        def _fresh() -> List[Tuple[float, float, float]]:
            with self._target_positions_lock:
                return [(x, y, z) for t, cid, x, y, z in self._target_positions
                        if t >= t0 and cid == class_id]

        ok = self._poll_until(
            lambda: len(_fresh()) >= samples,
            timeout,
            lambda: self._progress(f"等待目标'{class_id}'的定位结果（{len(_fresh())}/{samples}帧）…"),
            poll_interval_s=0.05,
        )
        pts = _fresh()
        if not ok:
            raise DetectionTimeoutError(class_id=class_id, timeout_s=timeout, namespace=self.namespace)
        pts = pts[:samples]
        mx = sorted(p[0] for p in pts)[len(pts) // 2]
        my = sorted(p[1] for p in pts)[len(pts) // 2]
        mz = sorted(p[2] for p in pts)[len(pts) // 2]
        spread = max(math.hypot(p[0] - mx, p[1] - my) for p in pts)
        return TargetPosition(x=mx, y=my, z=mz, samples=len(pts), spread_m=spread)

    # ------------------------------------------------------------------
    # 2.1节能力2：任务状态发布/查询
    # ------------------------------------------------------------------

    def set_mission_state(self, state: str) -> None:
        """发布任务状态到`mission_state`话题（`std_msgs/String`），封装
        `mission_state.py`契约（vendor版本，见本文件模块头说明）。

        Args:
            state: 顶层值必须是`VALID_MISSION_STATES`之一，允许"顶层值:
                子状态"带冒号写法（比如`'searching:orbit_pillar_2'`）。

        Raises:
            ValueError: `state`顶层值不合法——这不是超时/网络类的运行时
                失败，是选手传参写错了，跟`RclpyRuntime.__init__()`对
                空`namespace`的处理是同一种"参数校验用标准`ValueError`，
                不需要专门定义contest_sdk异常类型"的既有约定（`ValueError`
                本身不是rclpy/DDS的裸异常，不违反2.2.1节封装完整性要求）。
        """
        from std_msgs.msg import String

        if not is_valid_mission_state(state):
            raise ValueError(
                f"'{state}'不是合法的任务状态，顶层值必须是{VALID_MISSION_STATES}之一"
                f"（也可以用'顶层值:子状态'带冒号的写法，比如'searching:orbit_pillar_2'，"
                f"子状态部分不限制内容，只校验冒号前的顶层值）"
            )
        msg = String()
        msg.data = state
        self._mission_state_pub.publish(msg)
        # 立即更新本地缓存——不等这条消息真的通过DDS环回到自己的订阅回调
        # 才更新，否则get_mission_state()在set_mission_state()刚返回那一刻
        # 调用可能读到还没环回、依然是上一个状态的旧值（DDS环回本身通常
        # 只有几毫秒延迟，但没必要制造这种可以避免的短暂不一致窗口）。
        # 订阅回调(_on_mission_state)之后收到同一条消息只是再确认一次同一
        # 个值，不会覆盖成别的值。
        self._mission_state = state
        self._progress(f"任务状态 -> '{state}'")

    def get_mission_state(self) -> Optional[str]:
        """返回最近一次`set_mission_state()`设置的状态字符串；如果这个
        `DroneSDK`实例从未调用过`set_mission_state()`，返回`None`（不是
        `'idle'`——`None`更清楚地表达"还没设置过"，不需要选手去猜`'idle'`
        是真的状态还是"默认值"这种歧义）。
        """
        return self._mission_state

    # ------------------------------------------------------------------
    # 2026-09-15新增：绕飞朝向控制（D6"机头始终对准立柱中心"需求）
    # ------------------------------------------------------------------

    def set_yaw_mode_velocity(self) -> None:
        """朝向跟随速度方向（`ego_planner`原有算法，飞哪个方向机头就
        转向哪个方向）。

        ⚠️ **2026-09-16不再是默认值，谨慎使用**：实测发现急转弯时这套
        算法要求的yaw瞬时大幅跳变会跟同一时段的位置/高度控制抢电机推力
        分配资源，仿真上表现为明显掉高，真机上更严重、直接炸机过——
        `traj_server`的默认值已经改成`CONSTANT`（见`set_yaw_mode_
        constant()`），这个方法只在明确需要"机头跟随飞行方向"这个效果
        时才调用，D6绕飞收尾/其它场景一律用`set_yaw_mode_constant(sdk.
        pretakeoff_yaw)`切回起飞前的真实朝向，不要再用这个方法当"复位"
        手段。2026-09-17订正：不要写死`set_yaw_mode_constant(0.0)`——
        `0.0`只是个任意角度，跟飞机实际停机朝向（`sdk.pretakeoff_yaw`，
        `takeoff()`会自动读取并记录）不一定一致，写死0.0会在起飞/切回
        普通飞行的瞬间引入一次不必要的大幅转向，这正是当时"起飞后一
        转入飞行就急剧掉高"这个问题的根因（见`takeoff()`docstring）。
        """
        from std_msgs.msg import Int32

        msg = Int32()
        msg.data = 0  # YAW_MODE_VELOCITY，见traj_server.cpp的YawMode枚举
        self._yaw_mode_pub.publish(msg)
        self._progress('朝向模式 -> 朝速度方向（默认）')

    def set_yaw_mode_constant(self, yaw_rad: float) -> None:
        """朝向固定成一个不变的角度（弧度，跟这架飞机局部坐标系的
        yaw=0方向约定一致），不随位置变化重新计算。

        2026-09-16起`traj_server`默认就是这个模式（`yaw_constant_`
        默认值0.0）——急转弯不会让机头跟着大幅甩动，从根上避免"跟位置/
        高度控制抢电机资源导致掉高/炸机"这个实测过的真实风险（见
        `set_yaw_mode_velocity()`docstring）。D6绕飞结束后也回到这个
        模式（传`0.0`），不是`set_yaw_mode_velocity()`。

        Args:
            yaw_rad: 目标朝向，弧度。
        """
        from std_msgs.msg import Int32
        from geometry_msgs.msg import Point as _YawParamPoint

        param = _YawParamPoint()
        param.x = float(yaw_rad)
        self._yaw_param_pub.publish(param)
        msg = Int32()
        msg.data = 1  # YAW_MODE_CONSTANT
        self._yaw_mode_pub.publish(msg)
        self._progress(f'朝向模式 -> 固定角度({math.degrees(yaw_rad):.0f}°)')

    def set_yaw_mode_point(self, x: float, y: float) -> None:
        """朝向持续指向一个固定目标点，飞机每飞到新位置都会重新算一次
        朝向那个点的角度（绕着这个点转一圈，机头全程对着它）。

        ⚠️ **坐标系约定**：`x/y`是这架飞机自己的局部坐标系（跟`sdk.
        goto()`/`sdk.world_to_local()`返回值同一套坐标系），不是世界
        坐标——D6绕飞立柱中心是世界坐标，调用前需要先`sdk.world_to_
        local()`换算一次（`traj_server`内部拿当前轨迹位置`pos`直接跟
        这个点算差值，`pos`本身就是局部坐标系下的量，传世界坐标进来
        算出的角度会是错的）。

        Args:
            x/y: 目标点在这架飞机自己局部坐标系下的水平坐标（z不需要，
                只算水平朝向）。
        """
        from std_msgs.msg import Int32
        from geometry_msgs.msg import Point as _YawParamPoint

        param = _YawParamPoint()
        param.x = float(x)
        param.y = float(y)
        self._yaw_param_pub.publish(param)
        msg = Int32()
        msg.data = 2  # YAW_MODE_POINT
        self._yaw_mode_pub.publish(msg)
        self._progress(f'朝向模式 -> 指向固定点({x:.2f}, {y:.2f})')

    # ------------------------------------------------------------------
    # 2026-09-18新增：单相机+可动关节两档预设视角
    # ------------------------------------------------------------------

    def face_point(self, x: float, y: float, timeout: float = 10.0,
                   tolerance_deg: float = 5.0) -> bool:
        """悬停着把机头转到对准某个点，**返回是否真的转到位了**（超时返回False）。

        2026-09-23新增。`set_yaw_mode_point()`只是把朝向目标发给`traj_server`，
        飞机**在飞**的时候才会跟着转；停着的时候机头锁在进入悬停那一刻的角度
        （pt4ctrl的AUTO_HOVER用的是`hover_pose(3)`，见`PX4CtrlFSM.cpp`）。
        飞行栈的`traj_server`打了`ego_planner_traj_server_yaw_hold.patch`之后，
        空闲时收到新的朝向目标会以当前位置为目标持续发`position_cmd`、只转 yaw。

        这里**每秒重发一次**目标，直到转到位或超时。原因是`goto()`一到点就返回，
        而`ego_planner`到点后还会继续重规划一小会儿，每来一条新轨迹都会把
        `traj_server`里"等着开始转"的那次请求作废——只发一次的话，紧跟在
        `goto()`后面的这次转向十有八九会落空（2026-09-23实测：每个观察点位的
        第一个朝向都转不过去、干等10秒超时）。重发本身不会让机头抖：补丁里
        `beginYawHold()`对"已经在转时又收到同一目标"是不做重置的（重置会把
        控制器的提前量清零，那才是之前看着像摇头的原因）。

        返回值一定要用：没转到位就说明相机没指向预期方向，这时候拿到的画面
        不代表那个方向的情况，不该接着做检测/判断。

        Args:
            x/y: 要对准的点，这架飞机自己的局部坐标系（跟`goto()`同一套）。
            timeout: 最多等多久。
            tolerance_deg: 朝向差多少度以内算对准。
        """
        tol = math.radians(tolerance_deg)

        def _aimed() -> bool:
            px, py, _ = self._odom_xyz or (0.0, 0.0, 0.0)
            want = math.atan2(y - py, x - px)
            err = (self.get_current_yaw() - want + math.pi) % (2 * math.pi) - math.pi
            return abs(err) <= tol

        deadline = time.monotonic() + timeout
        ok = False
        while not ok:
            self.set_yaw_mode_point(x, y)       # 每轮重发一次，见上面说明
            left = deadline - time.monotonic()
            if left <= 0:
                break
            ok = self._poll_until(
                _aimed, min(1.0, left),
                lambda: self._progress(f'转向对准({x:.2f}, {y:.2f})中…'),
                poll_interval_s=0.2,
            )
        self._progress(f'朝向 {math.degrees(self.get_current_yaw()):.1f}°'
                       + ('' if ok else f'（{timeout:.0f}秒内没转到位）'))
        return ok

    def face_yaw(self, yaw_rad: float, timeout: float = 15.0,
                 tolerance_deg: float = 5.0) -> bool:
        """悬停着把机头转到**指定航向角**，转到位（或超时）才返回，返回是否转到位。

        2026-09-24新增。跟`face_point()`是同一套机制、同一个坑，区别只是这里给的是
        角度不是要对准的点：`face_point()`盯着一个点，飞机一动、朝向跟着变；这里
        锁死一个角度，整条航段不再变——编队"转到航向角再前飞"要的就是后者。

        实现要点跟`face_point()`完全一致，原因见那个方法的说明：
        · 停着的时候机头锁在进入悬停那一刻的角度（pt4ctrl的AUTO_HOVER用
          `hover_pose(3)`），必须靠飞行栈`ego_planner_traj_server_yaw_hold.patch`
          那条"空闲时收到新朝向就地转"的通路，`traj_server`才会只转yaw不动位置；
        · **每秒重发一次**：`goto()`一到点就返回，而ego_planner到点后还会再重规划
          一小会儿，每来一条新轨迹都会把"等着开始转"的那次请求作废，只发一次的话
          紧跟在`goto()`后面的转向十有八九落空。重发不会让机头抖（补丁里
          `beginYawHold()`对"已经在转时又收到同一目标"不做重置）。

        Args:
            yaw_rad: 目标航向角（弧度），跟`get_current_yaw()`同一套约定。
            timeout: 最多等多久。转 180° 实测要十几秒，默认给 15 秒。
            tolerance_deg: 差多少度以内算到位。

        Returns:
            True=转到位；False=超时没转到（调用方自己决定是照飞还是放弃这一段）。
        """
        tol = math.radians(tolerance_deg)

        def _aimed() -> bool:
            err = (self.get_current_yaw() - yaw_rad + math.pi) % (2 * math.pi) - math.pi
            return abs(err) <= tol

        deadline = time.monotonic() + timeout
        ok = False
        while not ok:
            self.set_yaw_mode_constant(yaw_rad)     # 每轮重发一次，见上面说明
            left = deadline - time.monotonic()
            if left <= 0:
                break
            ok = self._poll_until(
                _aimed, min(1.0, left),
                lambda: self._progress(
                    f'转向 {math.degrees(yaw_rad):.0f}° 中…当前 '
                    f'{math.degrees(self.get_current_yaw()):.0f}°'),
                poll_interval_s=0.2,
            )
        self._progress(f'朝向 {math.degrees(self.get_current_yaw()):.1f}°'
                       + ('' if ok else f'（{timeout:.0f}秒内没转到 {math.degrees(yaw_rad):.0f}°）'))
        return ok

    def set_fixed_altitude(self, height_m: float) -> None:
        """把飞行高度钉死在设定值，规划器轨迹里的高度变化不再生效。

        2026-09-24新增。**配合本项目默认的 LOCALIZATION_SOURCE=uwb_imu，这就是
        仿地飞行**：那套定位方案的 z 本来就是"离地高度"（`uwb_imu_fusion_node`
        把 测距雷达range×cos(roll)×cos(pitch) 喂给飞控当高度观测，x/y 才用
        UWB），所以命令高度恒定 = 离地高度恒定，地面抬高时飞机自己跟着抬。
        选手程序这边不需要读雷达、不需要知道地形在哪。

        为什么还需要这个开关：规划器是高频重规划的，它给出的轨迹高度本身在
        波动（实测同一段航线里轨迹 z 在 0.97~2.33 米之间跑），地形起伏带来的
        那点高度变化会被这种波动淹没，飞机不会稳定跟着地面走。钉死之后才是
        稳定的仿地行为。

        ⚠️ 定位源不是 uwb_imu 时（比如 gt/dlio，z 是绝对高度），这个开关就只是
        "定高飞行"，不具备仿地效果。
        ⚠️ 用完记得`set_fixed_altitude_off()`：开关持续生效，返航/降落段通常
        不想要它。

        Args:
            height_m: 要钉住的高度，**这架飞机自己局部坐标系下的 z**，跟
                `goto()`的第三个参数同一套。世界高度要先换算：

                    _, _, z = sdk.world_to_local(0.0, 0.0, 世界高度)
                    sdk.set_fixed_altitude(z)

                ⚠️ 两者必须一致，否则飞机会卡住：`goto()`按三维距离判到点
                （阈值0.3米），定高把 z 钉在别处时高度差永远消不掉，飞机水平
                到位了却判不到点，一直等到超时。2026-09-24 实测过——局部系 z
                原点对应世界0.467米，直接把世界高度2.0传进来，飞机钉在局部
                2.0、航点要的是局部1.533，差0.45米，编队第一个航点就卡死。
        """
        from geometry_msgs.msg import Point as _AltParamPoint
        from std_msgs.msg import Int32

        param = _AltParamPoint()
        param.x = float(height_m)
        self._alt_param_pub.publish(param)
        msg = Int32()
        msg.data = 1  # ALT_MODE_FIXED
        self._alt_mode_pub.publish(msg)
        self._progress(f'定高已开启（钉在 {height_m:.2f} m）')

    @contextlib.contextmanager
    def fixed_altitude(self, height_m: float):
        """`with`块里飞的这段航线全程定高，块退出（含异常）自动关掉。

        2026-09-24新增，用户要求："远距离转场这些走ego-planner的航段，飞行高度
        钉死在比如2米"。远距离`goto()`由规划器高频重规划，轨迹 z 本身在波动
        （实测同一段航线里 0.97~2.33 米），实际表现就是转场途中飞机会压得很低
        ——高层火情段返航时实测到过。钉死之后整段航线高度恒定。

        用法（钉住的高度必须跟这段航线 `goto()` 用的 z 一致，见 Args）：

            with sdk.fixed_altitude(2.0):
                sdk.goto(x, y, 2.0)        # 转场段，高度全程 2.0
            sdk.goto_direct(x, y, 0.8)     # 出了with，定高已关，可以自由升降

        Args:
            height_m: 要钉住的高度，跟`goto()`第三个参数同一套局部坐标系。
                **必须跟这段航线 goto 的 z 相等**，否则飞机水平到位了高度差
                消不掉、永远判不到点（详见`set_fixed_altitude()`的说明）。

        Note:
            只管本机走 traj_server 的那条路（`goto()`）。`goto_direct()`/精准
            降落/僚机编队跟随都是另一条旁路，不受这个开关影响；降落也不受影响
            （`land()`走飞控自己的AUTO_LAND）。
        """
        self.set_fixed_altitude(height_m)
        try:
            yield
        finally:
            self.set_fixed_altitude_off()

    def set_fixed_altitude_off(self) -> None:
        """关掉定高，高度回到"按规划器轨迹走"（默认行为）。"""
        from std_msgs.msg import Int32

        msg = Int32()
        msg.data = 0  # ALT_MODE_TRAJ
        self._alt_mode_pub.publish(msg)
        self._progress('定高已关闭（高度按轨迹走）')

    # ------------------------------------------------------------------
    # 抓一帧相机图存盘（2026-09-30 新增，任务3"拍照回传"要用）
    # ------------------------------------------------------------------

    #: 图像话题是 640x480x3 @ 7Hz ≈ 6 MB/s，绝大多数任务脚本用不到。所以**不**在
    #: `_setup_transport()` 里常驻订阅，等第一次调用 `capture_photo()` 再建，
    #: 建好之后保留（同一次飞行往往要拍好几张）。
    _CAMERA_TOPIC_SUFFIX = {'front': '_front_camera/image_raw',
                            'down': '_down_camera/image_raw'}

    def _ensure_image_sub(self, camera: str) -> None:
        if not hasattr(self, '_image_subs'):
            self._image_subs: Dict[str, Any] = {}
            self._latest_image: Dict[str, Any] = {}
        if camera in self._image_subs:
            return
        from sensor_msgs.msg import Image as _Image
        topic = f'{self.namespace}{self._CAMERA_TOPIC_SUFFIX[camera]}'

        def _on(msg: Any, _c: str = camera) -> None:
            self._latest_image[_c] = msg

        self._image_subs[camera] = self._node.create_subscription(_Image, topic, _on, 1)

    #: 5x7 点阵字模，只覆盖时间戳要用的字符（数字、'-'、':'、空格）。
    #: 镜像里没有 PIL/cv2，画一行字不值得为此装一套图形库；时间戳只有这几个
    #: 字符，手写字模三十行就够，还不用担心字体文件在不在。
    _GLYPHS_5X7 = {
        '0': ('01110', '10001', '10011', '10101', '11001', '10001', '01110'),
        '1': ('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
        '2': ('01110', '10001', '00001', '00010', '00100', '01000', '11111'),
        '3': ('01110', '10001', '00001', '00110', '00001', '10001', '01110'),
        '4': ('00010', '00110', '01010', '10010', '11111', '00010', '00010'),
        '5': ('11111', '10000', '11110', '00001', '00001', '10001', '01110'),
        '6': ('00110', '01000', '10000', '11110', '10001', '10001', '01110'),
        '7': ('11111', '00001', '00010', '00100', '01000', '01000', '01000'),
        '8': ('01110', '10001', '10001', '01110', '10001', '10001', '01110'),
        '9': ('01110', '10001', '10001', '01111', '00001', '00010', '01100'),
        '-': ('00000', '00000', '00000', '11111', '00000', '00000', '00000'),
        ':': ('00000', '00100', '00100', '00000', '00100', '00100', '00000'),
        '.': ('00000', '00000', '00000', '00000', '00000', '01100', '01100'),
        ' ': ('00000', '00000', '00000', '00000', '00000', '00000', '00000'),
    }

    @classmethod
    def _stamp_image(cls, arr: Any, text: str, scale: int = 3) -> None:
        """把 text 画在图像**右下角**（白字 + 黑描边，亮暗背景上都看得见）。原地改 arr。"""
        gw, gh, gap = 5, 7, 1
        cw = (gw + gap) * scale
        tw, th = cw * len(text), gh * scale
        pad = 6 * scale // 3 + 4
        h, w, _ = arr.shape
        x0, y0 = w - tw - pad, h - th - pad
        if x0 < 0 or y0 < 0:
            return
        for i, ch in enumerate(text):
            rows = cls._GLYPHS_5X7.get(ch)
            if rows is None:
                continue
            for ry, row in enumerate(rows):
                for rx, bit in enumerate(row):
                    if bit != '1':
                        continue
                    px = x0 + i * cw + rx * scale
                    py = y0 + ry * scale
                    # 先描一圈黑边再填白，浅色背景上也不会糊掉
                    arr[max(0, py - 1):py + scale + 1,
                        max(0, px - 1):px + scale + 1] = 0
        for i, ch in enumerate(text):
            rows = cls._GLYPHS_5X7.get(ch)
            if rows is None:
                continue
            for ry, row in enumerate(rows):
                for rx, bit in enumerate(row):
                    if bit != '1':
                        continue
                    px = x0 + i * cw + rx * scale
                    py = y0 + ry * scale
                    arr[py:py + scale, px:px + scale] = 255

    @staticmethod
    def _encode_png(arr: Any) -> bytes:
        """把 HxWx3 的 uint8 RGB 数组编成 PNG 字节流。

        选手镜像里没有 cv2 / PIL / cv_bridge（只有 numpy），为一次存图去装
        OpenCV 不划算，所以直接按 PNG 规范拼：8 位真彩、每行前面加一个 0
        （filter type None），zlib 压一下。三十行，没有新依赖。
        """
        import struct
        import zlib
        h, w, _ = arr.shape
        raw = bytearray()
        for y in range(h):
            raw.append(0)
            raw += arr[y].tobytes()

        def chunk(tag: bytes, data: bytes) -> bytes:
            return (struct.pack('>I', len(data)) + tag + data
                    + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))

        return (b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(bytes(raw), 6))
                + chunk(b'IEND', b''))

    def capture_photo(self, path: str, camera: str = 'front',
                      timeout: float = 8.0, fresh: bool = True,
                      timestamp: bool = True) -> str:
        """抓当前相机的一帧存成 PNG，返回实际写入的路径。

        Args:
            path: 存到哪。目录不存在会自动建。
            camera: `'front'` 前视 / `'down'` 下视。前视和下视是**两个独立的
                固定安装相机**（模型里两个 camera_joint 都是 `type='fixed'`），
                各自一路话题，这个参数就是选订阅哪一路——不存在"切视角"
                这回事（相机装死的，2026-09-30 已删掉 set_camera_view）。
            timeout: 等一帧图最多等多久。
            fresh: True（默认）只接受**调用之后**新到的帧，避免拿到切视角前
                的旧画面；False 则有缓存就直接用。
            timestamp: True（默认）在**右下角**烧上拍摄时刻
                （`YYYY-MM-DD HH:MM:SS`，白字黑描边）。

        Raises:
            DetectionTimeoutError: 超时没等到图（相机没开、视角没切过去、
                或者 DDS 还没发现这个话题）。

        回传：这里只负责**存盘**。存到挂载给地面站的目录（仿真里是 /logs）
        就等于回传了；真要走别的通道（HTTP/图传）是地面站侧的协议问题，
        不在 SDK 这一层定。
        """
        if camera not in self._CAMERA_TOPIC_SUFFIX:
            raise ValueError(f"camera 只能是 'front' 或 'down'，收到 {camera!r}")
        self._ensure_image_sub(camera)
        t_call = time.time()
        if fresh:
            self._latest_image.pop(camera, None)

        def _got() -> bool:
            msg = self._latest_image.get(camera)
            if msg is None:
                return False
            if not fresh:
                return True
            return _stamp_to_sec(msg.header.stamp) <= 0 or time.time() >= t_call

        if not self._poll_until(_got, timeout,
                                lambda: self._progress(f'等{camera}相机图像…')):
            raise DetectionTimeoutError(
                class_id=f'{camera}_camera/image_raw', timeout_s=timeout,
                namespace=self.namespace)

        msg = self._latest_image[camera]
        import numpy as _np
        buf = _np.frombuffer(bytes(msg.data), dtype=_np.uint8)
        enc = (msg.encoding or 'rgb8').lower()
        step = msg.step or (len(buf) // max(1, msg.height))
        nch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(enc)
        if nch is None:
            raise ValueError(f'不认识的图像编码 {msg.encoding!r}（支持 rgb8/bgr8/rgba8/bgra8/mono8）')
        # 按 step 切行再裁掉行尾 padding——ROS 的 step 不一定等于 width*nch
        img = buf[:msg.height * step].reshape(msg.height, step)[:, :msg.width * nch]
        img = img.reshape(msg.height, msg.width, nch)
        if enc in ('bgr8', 'bgra8'):
            img = img[:, :, 2::-1] if nch == 3 else img[:, :, [2, 1, 0]]
        elif enc == 'rgba8':
            img = img[:, :, :3]
        elif enc == 'mono8':
            img = _np.repeat(img, 3, axis=2)
        # **必须真拷贝一份**：`np.frombuffer()` 返回的是只读数组，而
        # `ascontiguousarray()` 在数组本来就连续时直接返回原对象、不拷贝，
        # 只读标志也就跟着留下来——烧时间戳那一步写进去会抛
        # "assignment destination is read-only"，2026-09-30 实测三张照片全废。
        img = _np.array(img, dtype=_np.uint8, order='C', copy=True)
        if timestamp:
            self._stamp_image(img, time.strftime('%Y-%m-%d %H:%M:%S'))

        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, 'wb') as f:
            f.write(self._encode_png(img))
        self._progress(f'已拍照存盘 {path}（{msg.width}x{msg.height}，{camera}）')
        return path

    # ------------------------------------------------------------------
    # 2.1节能力6：坐标系转换
    # ------------------------------------------------------------------

    def world_to_local(self, x: float, y: float, z: float, timeout: float = 5.0) -> Tuple[float, float, float]:
        """世界坐标 -> 这架飞机自己的局部坐标系（`{namespace}/odom`），
        只在SDK内部处理`frame_id`概念，对外只收发`(x, y, z)`三元组（方案
        2.2.1节要求，选手不需要知道TF/frame_id这些概念）。

        实现方式参考`ego_planner_bridge/rviz_goal_bridge_node.py::_to_
        local()`同一套TF查询（`lookup_transform(target_frame, 'world',
        Time(), timeout=Duration(seconds=0.0))`）——那个文件的注释解释
        过为什么内部timeout给0：`world -> {namespace}/odom`是一条静态TF，
        要么已经在树里能立刻查到，要么起飞点还没锁定、内部等多久都不会
        有。这里改成外部用`_poll_until()`轮询+打印进度，比内部死等一个
        非零timeout更好的地方在于：等待期间能持续给选手打印中文进度
        （方案2.2.2节要求），不会在TF还没建立好的那几秒里完全沉默。

        Args:
            x/y/z: 世界坐标系下的坐标。
            timeout: 等待TF变换建立的超时秒数（正常情况下起飞点一旦锁定
                就能立刻查到，这个超时主要覆盖"刚起飞、UWB定位还没收敛"
                这种短暂窗口）。

        Returns:
            `(local_x, local_y, local_z)`——这架飞机自己局部坐标系下的
            坐标，可以直接传给`sdk.goto()`。

        Raises:
            ActionFailedError: 超过`timeout`秒仍查不到TF变换（复用这个
                异常类型，不新造一个"TF查询失败"专用异常——2.2.1节的
                封装完整性要求核心是"选手不接触裸rclpy异常"，`ActionFailedError`
                的文本虽然是围绕`do_action()`写的，但"某个能力调用失败/
                超时"这个语义本身是通用的，比新增一个只会在这一个方法里
                触发、选手更难联想到的异常类型更简单）。
        """
        from geometry_msgs.msg import PoseStamped
        import tf2_geometry_msgs  # noqa: F401 - import副作用：注册PoseStamped的tf2类型转换
        from tf2_ros import TransformException

        holder: Dict[str, Tuple[float, float, float]] = {}

        def _try_transform() -> bool:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame, 'world', Time(), timeout=Duration(seconds=0.0)
                )
            except TransformException:
                return False
            src = PoseStamped()
            src.header.frame_id = 'world'
            src.pose.position.x, src.pose.position.y, src.pose.position.z = float(x), float(y), float(z)
            src.pose.orientation.w = 1.0
            local = tf2_geometry_msgs.do_transform_pose_stamped(src, tf)
            p = local.pose.position
            holder['result'] = (p.x, p.y, p.z)
            return True

        ok = self._poll_until(
            _try_transform,
            timeout,
            lambda: self._progress(f'坐标转换中…等待world -> {self._target_frame}这条TF建立'),
        )
        if not ok:
            raise ActionFailedError(action_name='world_to_local', timeout_s=timeout, namespace=self.namespace)
        return holder['result']

    def local_to_world(self, x: float, y: float, z: float, timeout: float = 5.0) -> Tuple[float, float, float]:
        """这架飞机自己的局部坐标系(`{namespace}/odom`) -> 世界坐标，
        跟`world_to_local()`反方向、同一套TF查询模式（只是`lookup_
        transform`的两个frame参数对调），2026-09-13阶段D单机实测时
        补的方法。

        **补这个方法的直接原因**：`sdk.center_on_target()`/`sdk.read_
        fire_pillar_staging_pose()`/`sdk.read_fire_pillar_aim_pose()`
        这几个方法返回的坐标，实际都是"这架飞机自己的局部坐标"（分别
        来自`precision_servo_node`/`fire_pillar_aim_node`各自订阅的
        本机`dlio/odom_node/odom`，`frame_id`都是`{ns}/map`，跟`{ns}/
        odom`恒等——不是这几个方法早期文档字面写的"世界坐标"，那几处
        文档措辞是错的，已经一并订正，实际实现从来没变过）。这些坐标
        如果要`send_to_teammate()`广播给队友，队友收到后必然要用**自己
        的**`sdk.world_to_local()`转换成它自己的局部坐标才能传给`sdk.
        goto()`——两架飞机局部系原点不同（起降点本身相距几米），任何
        跨机传递的坐标必须先转换成两边都认的世界坐标这个共同语言，不能
        直接把一架飞机的局部坐标数值原样发给另一架飞机使用。

        Args:
            x/y/z: 这架飞机自己局部坐标系下的坐标（比如`sdk.center_on_
                target()`/`sdk.read_fire_pillar_staging_pose()`的返回值，
                或者`sdk.get_local_position()`读到的自身当前位置）。
            timeout: 等待TF变换建立的超时秒数，语义跟`world_to_local()`
                一致。

        Returns:
            `(world_x, world_y, world_z)`——世界坐标系下的坐标，可以
            直接传给`send_to_teammate()`广播，或者留着自己用（跟`world_
            to_local()`返回值反过来同样可以直接传给`sdk.goto()`不是
            这个方法的用途）。

        Raises:
            ActionFailedError: 超过`timeout`秒仍查不到TF变换（跟`world_
                to_local()`同一个理由，复用这个异常类型不新造）。
        """
        from geometry_msgs.msg import PoseStamped
        import tf2_geometry_msgs  # noqa: F401 - import副作用：注册PoseStamped的tf2类型转换
        from tf2_ros import TransformException

        holder: Dict[str, Tuple[float, float, float]] = {}

        def _try_transform() -> bool:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'world', self._target_frame, Time(), timeout=Duration(seconds=0.0)
                )
            except TransformException:
                return False
            src = PoseStamped()
            src.header.frame_id = self._target_frame
            src.pose.position.x, src.pose.position.y, src.pose.position.z = float(x), float(y), float(z)
            src.pose.orientation.w = 1.0
            world = tf2_geometry_msgs.do_transform_pose_stamped(src, tf)
            p = world.pose.position
            holder['result'] = (p.x, p.y, p.z)
            return True

        ok = self._poll_until(
            _try_transform,
            timeout,
            lambda: self._progress(f'坐标转换中…等待{self._target_frame} -> world这条TF建立'),
        )
        if not ok:
            raise ActionFailedError(action_name='local_to_world', timeout_s=timeout, namespace=self.namespace)
        return holder['result']

    # ------------------------------------------------------------------
    # 2.1节能力4：动作执行
    # ------------------------------------------------------------------

    def do_action(self, name: str, timeout: float = 30.0) -> None:
        """触发`actuator_action_node`的一个符号化动作（比如`'grab_supply'`），
        阻塞直到收到`action_status`广播`f'done:{name}'`确认。

        Args:
            name: 动作名字符串（选手自定义，具体取值由这次任务的
                `actuator_action_node`已知支持哪些决定，SDK这一层不限定）。
            timeout: 超时秒数（覆盖"设置action_name参数"+"等待done确认"
                两个阶段加在一起的总时长）。

        Raises:
            ActionFailedError: 参数设置被拒绝、机载节点明确拒绝这次触发
                （比如上一个动作还没结束），或者超过`timeout`秒仍未收到
                完成确认。

        **幂等去重 + 结果关联（2026-09-20新增）**：每次调用生成一个
        `goal_id`（`<namespace>-<name>-<序号>`），跟`action_name`放在
        **同一次**`set_parameters`请求里下发，机载节点按它判重：
        - 同一个`goal_id`重复下发不会让动作执行第二次（`grab_supply`
          执行两次是真实风险，原来靠"调用方别重发"这个约定来规避，
          但SDK自身为了防DDS发现没跟上本来就有连发防护，两者冲突）；
        - 完成状态回带同一个`goal_id`，所以能确认"收到的是这一次的
          完成"，而不是上一次遗留的状态——原来只能靠调用前清空本地
          缓存来规避，跨进程重启就失效。

        兼容老机载节点（只认`action_name`、只发`done:<name>`）：下面
        的完成判据同时接受带`goal_id`和不带`goal_id`两种格式，所以
        SDK升级后即使机载镜像还没重新build，行为也跟原来一致。
        """
        self._action_goal_counter += 1
        goal_id = f'{self.namespace}-{name}-{self._action_goal_counter}'

        self._action_status = None  # 清掉可能残留的上一次done状态，避免误判
        set_ok = self._call_set_parameters_blocking(
            self._actuator_action_params_cli,
            {'action_name': name, 'action_goal_id': goal_id},
            timeout_s=min(5.0, timeout),
        )
        if not set_ok:
            raise ActionFailedError(action_name=name, timeout_s=timeout, namespace=self.namespace)

        done_with_id = f'done:{name}:{goal_id}'
        done_legacy = f'done:{name}'  # 老机载节点的格式，见上面docstring
        rejected_prefix = f'rejected:{name}'

        def _finished() -> bool:
            status = self._action_status
            if status is None:
                return False
            # 机载明确拒绝（busy/unknown_action）时立刻结束等待，不干等到超时
            if status.startswith(rejected_prefix) and (not goal_id or goal_id in status):
                raise ActionFailedError(
                    action_name=f'{name}（机载拒绝: {status}）',
                    timeout_s=timeout,
                    namespace=self.namespace,
                )
            return status in (done_with_id, done_legacy)

        ok = self._poll_until(
            _finished,
            timeout,
            lambda: self._progress(f"执行动作'{name}'中…当前action_status={self._action_status!r}"),
        )
        if not ok:
            raise ActionFailedError(action_name=name, timeout_s=timeout, namespace=self.namespace)
        self._progress(f"动作'{name}'已完成")

    def set_actuator(self, index: int, value: float, timeout: float = 5.0) -> None:
        """直接设置一路"Peripheral via Actuator Set"外设输出（比如舵机），
        走标准`MAV_CMD_DO_SET_ACTUATOR`（命令187）+`mavros/cmd/command`。

        2026-09-18跟用户讨论确定的结论：这个能力**不走`do_action()`那条
        RC-override路线**（`actuator_action_node`常驻flight-stack、必须
        ≥10Hz持续发布才能对抗真实接收机在同一个`input_rc`上的竞争写入，
        见`do_action()`docstring）——`MAV_CMD_DO_SET_ACTUATOR`是纯一次性
        指令，PX4收到后会在自己的输出侧一直保持这个值直到下一次设置，
        调用方不需要维护任何持续循环，所以可以（也应该）直接放在选手
        SDK里调用，不需要占用flight-stack资源。两条路线服务的是不同的
        硬件配置——舵机接在飞控自己的MAIN/AUX输出、且该输出在QGC里配置成
        `Peripheral via Actuator Set{index}`时用这个方法；舵机接在遥控器
        接收机自己的通道输出上、要靠"抢"RC通道触发时才用`do_action()`。

        ⚠️ 真实设备用的板子固件可能没有编译`Servo1~8`这个Function（常见于
        纯多旋翼固件），QGC里只有`Peripheral via Actuator Set1~6`可选时，
        这个方法就是对应的驱动方式——跟`do_action()`底层机制完全不同，
        不要混用。

        Args:
            index: 目标槽位，1~6，对应QGC里把某个物理输出的Function配置成
                `Peripheral via Actuator Set{index}`的那个`{index}`。
            value: 归一化目标值，-1.0~1.0（对应该输出自己配置的PWM MIN~MAX，
                换算关系由PX4的mixer按那个输出自己的参数决定，这一层不关心
                具体PWM微秒数）。
            timeout: 等待`mavros/cmd/command`服务响应的超时秒数。

        Raises:
            ValueError: `index`不在1~6之间，或`value`不在[-1.0, 1.0]之间。
            ActionFailedError: 服务调用超时，或PX4返回`success=False`
                （常见原因：目标输出没有配置成`Peripheral via Actuator
                Set{index}`这个Function、或者mavros/PX4链路本身没连上）。
        """
        if not 1 <= index <= 6:
            raise ValueError(f"index必须在1~6之间，收到{index}")
        if not -1.0 <= value <= 1.0:
            raise ValueError(f"value必须在-1.0~1.0之间，收到{value}")

        # 只填目标槽位，其余5个槽位显式填NaN（不是留默认0.0！）——PX4那边
        # 对每个param的判断是"只要是有限数值就应用"，默认0.0是有限数值，
        # 会被当成"把Set里另外5个槽位也设成0.0"误发出去，误伤同一个Set里
        # 配置了别的外设的其它输出（见2026-09-18讨论，这是这个命令最容易
        # 踩的坑，不是理论上的边界情况）。
        params = [float('nan')] * 6
        params[index - 1] = float(value)
        self._send_actuator_set(params, f'set_actuator[{index}]={value}', timeout)
        self._progress(f"已设置Actuator Set{index} = {value}")

    def _send_actuator_set(self, params: List[float], action_name: str, timeout: float) -> None:
        """发一条`MAV_CMD_DO_SET_ACTUATOR`，`params`是Set1~6六个槽位的值，
        不改的槽位必须是NaN（见`set_actuator()`里的说明）。"""
        request = self._CommandLong.Request()
        request.broadcast = False
        request.command = 187  # MAV_CMD_DO_SET_ACTUATOR
        request.confirmation = 0
        request.param1, request.param2, request.param3 = params[0], params[1], params[2]
        request.param4, request.param5, request.param6 = params[3], params[4], params[5]
        request.param7 = 0.0  # Actuator Set索引，0=Set1~6（这个方法只支持这一组）

        ok, response = self._call_service_blocking(self._command_long_cli, request, timeout_s=timeout)
        if not ok or response is None or not response.success:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)

    @property
    def servos(self) -> Dict[int, ServoSpec]:
        """这架飞机上可以用`set_servo()`控制的舵机：{舵机编号: 配置}，
        没有舵机的飞机返回空字典。"""
        return dict(SERVO_CONFIG.get(self.namespace, {}))

    def set_servo(self, servo: int, pwm: int, timeout: float = 5.0) -> None:
        """把舵机转到指定PWM位置（微秒）。发出后飞控会一直保持这个位置，
        直到下一次设置，不需要反复调用。

        用法（NX02上的两个舵机，PWM范围800~2000）::

            sdk.set_servo(1, 2000)   # 舵机1（MAIN7）转到2000
            sdk.set_servo(2, 800)    # 舵机2（MAIN9）转到800

        飞控把`COM_PREARM_MODE`设成2之后，**不解锁也能动**（电机不会转）。
        这个方法只等飞控回"收到"，舵机没有位置反馈，转到位需要的时间
        （通常零点几秒）要自己`time.sleep()`等。

        Args:
            servo: 舵机编号（1或2），可用的编号见`sdk.servos`。
            pwm: 目标PWM（微秒），必须在该舵机配置的最小~最大值之间。
            timeout: 等飞控确认的超时秒数。

        Raises:
            ValueError: 这架飞机没有这个舵机，或PWM超出范围。
            ActionFailedError: 飞控没有确认（mavros/飞控链路没连上等）。
        """
        self.set_servos({servo: pwm}, timeout=timeout)

    def set_servos(self, positions: Dict[int, int], timeout: float = 5.0) -> None:
        """同时设置多个舵机：一条指令发给飞控，几个舵机同一时刻开始转
        （分开调用`set_servo()`会差几十毫秒）。

        用法::

            sdk.set_servos({1: 2000, 2: 2000})   # 两个舵机同时到2000

        Args:
            positions: {舵机编号: 目标PWM}。
            timeout: 等飞控确认的超时秒数。

        Raises:
            ValueError: 有舵机编号不存在或PWM超出范围——此时一个都不会发。
            ActionFailedError: 飞控没有确认。
        """
        if not positions:
            raise ValueError('positions不能为空')
        config = SERVO_CONFIG.get(self.namespace, {})
        params = [float('nan')] * 6
        for servo, pwm in positions.items():
            spec = config.get(servo)
            if spec is None:
                raise ValueError(
                    f"{self.namespace}上没有舵机{servo!r}，"
                    + (f"可用的舵机编号：{sorted(config)}" if config else
                       f"这架飞机没有配置舵机（舵机在：{sorted(SERVO_CONFIG)}）")
                )
            params[spec.actuator_set - 1] = servo_pwm_to_normalized(spec, int(pwm))
        desc = '，'.join(f"舵机{k}（{config[k].output}）-> PWM {int(v)}" for k, v in positions.items())
        self._send_actuator_set(params, f'set_servos {positions}', timeout)
        self._progress(desc)

    # ------------------------------------------------------------------
    # 2.1节能力7：编队跟随开关（一次性触发调用，不做持续订阅/计算）
    # ------------------------------------------------------------------

    def _on_teammate_formation_diag(self, msg: Any) -> None:
        if len(msg.data) >= 1:
            self._teammate_lag_m = float(msg.data[0])

    def teammate_formation_lag(self) -> Optional[float]:
        """队友（僚机）沿长机轨迹的落后量，米。正值=落后，0 附近=在队形位置上。

        只有队友那边的 `formation_follower_node` 真在跟随时才有数；没启用、
        还没入列、或者话题还没到，返回 None。

        典型用法是长机的"照顾模式"：落后多了就减速等一等，而不是一路飞到底
        把僚机甩开（僚机那边只有追赶、没有让长机等的手段）。
        """
        return self._teammate_lag_m

    def _get_ego_planner_double(self, name: str, timeout: float) -> float:
        """读 ego_planner 的一个 double 参数。读不到返回 0.0（不抛异常——这只是
        为了给调用方一个"用完好恢复"的旧值，读失败不该让整个动作失败）。"""
        req = GetParameters.Request()
        req.names = [name]
        try:
            ok, resp = self._call_service_blocking(
                self._ego_planner_get_params_cli, req, timeout_s=timeout)
        except Exception as exc:                      # noqa: BLE001 - 读失败不致命
            self._progress(f'读不到 {name}（{exc}），旧值按 0 处理')
            return 0.0
        if not ok or resp is None or not resp.values:
            return 0.0
        return float(resp.values[0].double_value)

    def set_max_vel(self, max_vel_mps: float, timeout: float = 5.0) -> float:
        """改`ego_planner`的巡航限速，返回改之前的值（方便用完恢复）。

        用在"起步缓加速"这类场景：编队起步时长机一上来就顶到 1.0 m/s 会把僚机
        甩开，先压到 0.4、几秒后再恢复。

        改的是`manager/max_vel`——它进的是初值多项式的**时间分配**（梯形剖面），
        限速压低，加速段就被拉长。优化器那边的`optimization/max_vel`（动力学可行性
        罚项）**不动**：那一项只在超限时才罚，限速压低时根本不会绑定，两边不打架。

        ⚠️ 需要`ego_planner_max_vel_runtime.patch`——原版这个参数只在
        `initPlanModules()`读一次，运行时改了不生效。镜像没打这个补丁时本方法
        依然会"成功"（参数确实设上了），但规划器不会理它。

        Args:
            max_vel_mps: 新的限速，米/秒。必须为正。
            timeout: 等参数服务响应的超时秒数。

        Returns:
            改之前的限速值；读不到就返回 0.0。

        Raises:
            ValueError: `max_vel_mps <= 0`。
            ActionFailedError: 参数服务调用超时或被拒绝。
        """
        if max_vel_mps <= 0.0:
            raise ValueError(f'max_vel_mps 必须为正，收到 {max_vel_mps}')
        old = self._get_ego_planner_double('manager/max_vel', timeout)
        ok = self._call_set_parameters_blocking(
            self._ego_planner_params_cli,
            {'manager/max_vel': float(max_vel_mps)}, timeout_s=timeout)
        if not ok:
            raise ActionFailedError(action_name=f'set_max_vel({max_vel_mps})',
                                    timeout_s=timeout, namespace=self.namespace)
        self._progress(f'规划器限速 -> {max_vel_mps:.2f} m/s（原 {old:.2f}）')
        return old

    def set_direct_speed(self, max_speed_mps: float, timeout: float = 5.0) -> None:
        """改 `goto_direct()` 的限速（`precision_servo_node` 的
        `coordinate_max_speed_mps`），**飞行中可以随时改**。

        跟 `set_max_vel()` 是两条完全不同的链路，别混：
          · `set_max_vel()` 改的是 `ego_planner` 的巡航限速，管 `goto()`
            / `fly_route()` / `lead_formation()` 这些**走规划器、有避障**的飞行；
          · 本方法改的是 precision_servo 的直线伺服限速，管 `goto_direct()`
            / `return_home()` 最后一段 / `aim_at()` 的平移这些**不避障**的飞行。

        默认值只有 0.3 m/s，那是给"最后一段精修落点"调的；拿它跑长距离会慢到
        离谱（实测 5.4 米走了 45 秒）。改了记得改回去——这个参数是节点级的，
        不会自己复位，带着高限速跑 `return_home()` 的落点精修和 `aim_at()` 的
        平移，精度都会变差。

        ⚠️ **不要拿它去"加速走直线绕过规划器"**：避障本身是考核点，航线上的
        `obstacle_cylinder` 就是为这一项摆的。这个方法只用于已经在走直飞的
        那些短段（落点精修、对准平移）。

        Args:
            max_speed_mps: 新的限速，米/秒。必须为正。

        Raises:
            ValueError: `max_speed_mps <= 0`。
            ActionFailedError: 参数服务调用超时或被拒绝。
        """
        if max_speed_mps <= 0.0:
            raise ValueError(f'max_speed_mps 必须为正，收到 {max_speed_mps}')
        ok = self._call_set_parameters_blocking(
            self._precision_servo_params_cli,
            {'coordinate_max_speed_mps': float(max_speed_mps)}, timeout_s=timeout)
        if not ok:
            raise ActionFailedError(action_name=f'set_direct_speed({max_speed_mps})',
                                    timeout_s=timeout, namespace=self.namespace)
        self._progress(f'直飞限速 -> {max_speed_mps:.2f} m/s')

    def start_formation_follow(self, follow_distance_m: float, timeout: float = 10.0,
                               altitude_agl_m: Optional[float] = None,
                               turn_in_place: Optional[bool] = None,
                               leg_route: Optional[List[Tuple[float, float]]] = None) -> None:
        """启用`formation_follower_node`（跟随目标固定是构造`DroneSDK`时
        传入的`teammate_namespace`，选手不需要指定跟谁——方案2.1节第7条：
        "队友的namespace不是SDK自己猜的"）。

        这是"发一次调用+确认对方接受了参数"，不是持续订阅/计算（1.0节
        架构原则：编队跟随的实时控制回路常驻在`formation_follower_node`
        里，SDK这一层绝对不实现控制回路本身）。

        Args:
            follow_distance_m: 沿长机轨迹的跟随间距下限。
            timeout: 等对方确认的超时秒数。
            altitude_agl_m: 僚机保持的**离地高度**。不传就沿用
                `formation_follower_node`自己的默认值（1.5米）。

                为什么要能传：僚机的高度既不跟轨迹也不跟长机，而是这个节点
                按机载定高雷达保持的固定离地高度（所以僚机天然就是仿地飞行）。
                长机的巡航高度写在选手程序里、僚机的写在节点参数里，两处
                各管各的——2026-09-24 把长机编队高度从1.5提到2.5时就撞上了：
                长机2.5米、僚机还在1.5米，编队变成一高一低。传这个参数等于
                让任务程序统一决定两机高度，不再靠"两个默认值碰巧相等"。

        Raises:
            ActionFailedError: `~/set_parameters`调用超时或被拒绝。
        """
        if leg_route is not None:
            flat: List[float] = []
            for px, py in leg_route:
                flat += [float(px), float(py)]
            ok = self._call_set_parameters_blocking(
                self._formation_follower_params_cli, {'leg_route_xy': flat}, timeout_s=timeout)
            if not ok:
                raise ActionFailedError(
                    action_name='start_formation_follow:leg_route', timeout_s=timeout,
                    namespace=self.namespace)
            self._progress(f'僚机航线已下发（{len(leg_route)} 个点，用来定每段航向）')
        if turn_in_place is not None:
            # 分段航向：僚机跟长机一样"拐点先停下转到位、再前飞"（2026-09-24用户
            # 要求"长机、从机都一样"）。僚机没有航点，等价物是长机轨迹的切线跳变，
            # 判据和阈值都在 formation_follower_node 那边，见该文件参数声明处说明。
            ok = self._call_set_parameters_blocking(
                self._formation_follower_params_cli,
                {'yaw_follow_leg': bool(turn_in_place)},
                timeout_s=timeout,
            )
            if not ok:
                raise ActionFailedError(
                    action_name='start_formation_follow:turn_in_place', timeout_s=timeout,
                    namespace=self.namespace)
            self._progress(f'僚机分段航向（拐点先转向再前飞）{"开" if turn_in_place else "关"}')
        if altitude_agl_m is not None:
            # 高度参数对两条通路（action / 参数）都生效：节点每个控制周期都重新
            # 读它（见 formation_follower_node 里 follow_altitude_agl_m 的用法）
            ok = self._call_set_parameters_blocking(
                self._formation_follower_params_cli,
                {'follow_altitude_agl_m': float(altitude_agl_m)},
                timeout_s=timeout,
            )
            if not ok:
                raise ActionFailedError(
                    action_name='start_formation_follow:altitude', timeout_s=timeout,
                    namespace=self.namespace)
            self._progress(f'僚机保持离地高度设为 {altitude_agl_m:.2f} m')
        # 2026-09-20：优先走机载的 FormationFollow action——它把跟随拆成
        # 入列(join)/保持(track)/出列(break)三个阶段，这个调用会阻塞到
        # "入列完成"才返回，调用方因此第一次能确切知道"队形组好了"，而
        # 不是设完参数就走、靠估时间。server 不在时自动退回原来的参数路径。
        if self._try_formation_via_action(follow_distance_m, timeout):
            return

        self._progress('机载编队 action 不可用，退回参数方式启用（行为与改造前一致）')
        ok = self._call_set_parameters_blocking(
            self._formation_follower_params_cli,
            {
                'leader_namespace': self.teammate_namespace,
                'follow_distance_m': float(follow_distance_m),
                'enabled': True,
            },
            timeout_s=timeout,
        )
        if not ok:
            raise ActionFailedError(
                action_name='start_formation_follow', timeout_s=timeout, namespace=self.namespace
            )
        self._progress(f'编队跟随已启用（跟随{self.teammate_namespace}，距离{follow_distance_m}米）')

    def _try_formation_via_action(self, follow_distance_m: float, timeout: float,
                                  retried: bool = False) -> bool:
        """尝试走机载 FormationFollow action，阻塞到**僚机已接管并进入待命**。

        2026-09-21改（原来是阻塞到入列完成，即 feedback 报 `track`）：
        `track` 要等长机真的走出一个跟随距离才可能达成，而长机那边要等
        僚机报"我准备好了"才起步——两边互相等，必然死锁。现在改成等
        `hold`（僚机已接管控制权、在自己起飞点上空原地保持），这才是
        "可以让长机起步了"的确切信号。后续的 join->track 由机载回路自己
        完成，选手程序不需要在那里阻塞。

        Returns:
            True 表示 action 路径走通且僚机已进入跟随待命；False 表示
            server 不在，调用方应退回参数路径。

        Raises:
            ActionFailedError: server 在但拒绝 goal、或入列阶段明确失败
                （入列超时/长机丢失）。这种情况不退回参数路径重来一次——
                参数路径连入列判据都没有，退回去只会掩盖问题。
        """
        if self._formation_action_cli is None:
            return False
        if not self._formation_action_cli.wait_for_server(
                timeout_sec=ACTION_SERVER_PROBE_TIMEOUT_S):
            return False

        goal = self._FormationFollow.Goal()
        goal.leader_namespace = self.teammate_namespace
        goal.follow_distance_m = float(follow_distance_m)
        goal.join_timeout_s = 0.0  # 用 server 默认

        # `hold`/`join`/`track` 任意一个到达都说明机载回路已经接管——
        # `hold` 是最早的那个信号，也是"可以让长机起步了"的判据。
        standby = {'ok': False}

        def _on_feedback(msg: Any) -> None:
            fb = msg.feedback
            # NaN 才是"算不出来"；负值是正常的（僚机略微超前于队形位置）
            gap_text = '未知' if math.isnan(fb.gap_m) else f'{fb.gap_m:.2f}m'
            self._progress(
                f'编队{fb.phase}阶段…间距{gap_text}，长机可见={fb.leader_visible}（回路在机载）'
            )
            if fb.phase in ('hold', 'join', 'track'):
                standby['ok'] = True

        send_future = self._formation_action_cli.send_goal_async(
            goal, feedback_callback=_on_feedback)
        if not self._wait_future(send_future, ACTION_GOAL_ACCEPT_TIMEOUT_S):
            raise ActionFailedError(
                action_name='start_formation_follow(goal无响应)', timeout_s=timeout,
                namespace=self.namespace)
        goal_handle = send_future.result()
        if not goal_handle.accepted and not retried:
            # 2026-09-24：goal 被拒绝几乎只有一个原因——机载还有一个上一轮
            # **没收尾的** goal 在跑（上一次任务程序崩了/被 docker rm 掉，容器
            # 没了但机载这一侧的跟随还活着，要等它自己超时 180 秒才释放）。
            # 实测就是这么卡住的：第一次跑崩掉，第二次跑僚机 goal 被拒、直接
            # 异常退出，长机在那边等"僚机就位"等到 300 秒超时。
            # 处置：发一次 cancel 把旧 goal 清掉，再重投一次。清不掉才算真失败。
            self._progress('编队 goal 被拒绝（机载可能还有上一轮没收尾的跟随），'
                           '先出列再重投一次')
            self._cancel_stale_formation_goal(timeout)
            return self._try_formation_via_action(follow_distance_m, timeout, retried=True)
        if not goal_handle.accepted:
            raise ActionFailedError(
                action_name='start_formation_follow(goal被拒绝)', timeout_s=timeout,
                namespace=self.namespace)

        # 记住句柄：stop_formation_follow() 要用它发 cancel
        self._formation_goal_handle = goal_handle
        self._formation_result_future = goal_handle.get_result_async()

        # 阻塞到僚机进入跟随待命（feedback 报出 hold/join/track）或 goal
        # 提前结束。不等 track——见本方法 docstring 里的死锁说明。
        deadline = time.monotonic() + FORMATION_STANDBY_WAIT_S
        while time.monotonic() < deadline:
            if standby['ok']:
                self._progress(
                    f'编队跟随已就位待命（跟随{self.teammate_namespace}，间距'
                    f'{follow_distance_m}米，入列/保持回路在机载）'
                )
                return True
            if self._formation_result_future.done():
                res = self._formation_result_future.result().result
                self._formation_goal_handle = None
                raise ActionFailedError(
                    action_name=f'start_formation_follow（机载: {res.message}）',
                    timeout_s=timeout, namespace=self.namespace)
            time.sleep(0.1)
        raise ActionFailedError(
            action_name=f'start_formation_follow(等就位待命超过{FORMATION_STANDBY_WAIT_S:.0f}秒)',
            timeout_s=timeout, namespace=self.namespace)

    def _cancel_stale_formation_goal(self, timeout: float) -> None:
        """清掉机载上一轮遗留的编队 goal（本进程没有句柄，只能按目标全量取消）。

        `cancel_goal_async()` 要有句柄才能发，而"上一轮"的句柄跟着上一个容器一起
        没了。ROS2 的 action client 提供了按 client 全量取消的接口
        （`_cancel_goal_async` 走的是同一个 CancelGoal 服务，goal_id 全 0 = 取消
        该 server 上所有 goal），这里就用它；不可用时退回参数通路把 enabled 关掉，
        节点那一侧同样会收尾。
        """
        try:
            from action_msgs.srv import CancelGoal

            req = CancelGoal.Request()   # goal_id 全 0 + stamp 0 = 取消全部
            future = self._formation_action_cli._cancel_client.call_async(req)
            self._wait_future(future, timeout)
        except Exception as exc:        # 接口不可用/版本差异都不该让任务挂掉
            self._progress(f'按 action 取消遗留 goal 没成功（{exc!r}），改用参数方式出列')
            self._call_set_parameters_blocking(
                self._formation_follower_params_cli, {'enabled': False}, timeout_s=timeout)
        time.sleep(1.0)                 # 给节点一点时间真正收尾再重投

    def set_formation_leg_route(self, leg_route: List[Tuple[float, float]],
                                timeout: float = 10.0) -> None:
        """把航线（世界坐标，第一个点是长机起飞点）下发给僚机的编队节点，用来定
        每一段的航向。

        2026-09-24新增。为什么航线要单独发一次而不是只在`start_formation_follow()`
        里传：僚机是**先接管、再通知长机"我就位了"**（不这样会死锁，见那个方法的
        说明），而航线只有长机知道——长机要等到"僚机就位"才会把航线发过来，那时
        编队跟随早就启动了。节点每个控制周期都重新读这个参数，晚一点下发没关系：
        长机自己还要先转向、再飞出一个跟随距离，僚机才会真的开始走。
        """
        flat: List[float] = []
        for px, py in leg_route:
            flat += [float(px), float(py)]
        ok = self._call_set_parameters_blocking(
            self._formation_follower_params_cli, {'leg_route_xy': flat}, timeout_s=timeout)
        if not ok:
            raise ActionFailedError(
                action_name='set_formation_leg_route', timeout_s=timeout, namespace=self.namespace)
        self._progress(f'僚机航线已下发（{len(leg_route)} 个点，用来定每段航向）')

    def stop_formation_follow(self, timeout: float = 10.0) -> None:
        """停用编队跟随：出列并停止发布目标点。

        走 action 时发 cancel，server 负责出列收尾后返回 CANCELED；飞机
        停在最后一个目标点由 pt4ctrl 的 AUTO_HOVER 维持，本方法不额外发
        任何降落/悬停指令。
        """
        if self._formation_goal_handle is not None:
            handle, self._formation_goal_handle = self._formation_goal_handle, None
            cancel_future = handle.cancel_goal_async()
            self._wait_future(cancel_future, timeout)
            if self._formation_result_future is not None:
                self._wait_future(self._formation_result_future, timeout)
                self._formation_result_future = None
            self._progress('编队跟随已出列（action cancel）')
            return

        ok = self._call_set_parameters_blocking(
            self._formation_follower_params_cli, {'enabled': False}, timeout_s=timeout
        )
        if not ok:
            raise ActionFailedError(
                action_name='stop_formation_follow', timeout_s=timeout, namespace=self.namespace
            )
        self._progress('编队跟随已停用')

    # ------------------------------------------------------------------
    # 方案2.3节"链路A"：takeoff()/land()/goto()
    # ------------------------------------------------------------------

    def _publish_with_retry(self, pub: Any, msg: Any, repeat: int = 3, gap_s: float = 0.2) -> None:
        """连发几次同一条消息——照抄`gcs/backend/app.py::_publish_
        takeoff_land()`的防御写法（注释原话"ros2 topic pub --once在DDS
        discovery没跟上时可能白发"），提炼成通用小工具，供任何"一次性
        关键指令"复用（原来只在`_publish_takeoff_land`里用，这次实测
        发现`goto()`/`cancel_goto()`发`waypoint_queue`/`waypoint_cancel`
        时没套用这条防线，2026-09-13端到端测试里`goto()`因此偶发"发了
        但对方没收到、之后一直没人再发第二次"，导致`ego_planner`没有
        真正拿到目标点、`sdk.goto()`空等到超时——提炼出来是为了避免同一
        类"关键一次性指令"以后新增方法时又漏掉这条防线）。

        ⚠️ 这里假设传入的`pub`本身是常驻的（`_setup_transport()`里创建
        一次，不是每次调用才新建），已经满足方案2.3节"链路A"第1条"常驻
        publisher，不要每次指令都新起一次性进程"的要求——但即便publisher
        对象本身常驻，如果选手程序刚构造完`DroneSDK`就立刻调用某个方法，
        这个publisher跟对端订阅之间的DDS discovery仍然可能还没完成匹配
        （discovery是异步的，跟"publisher对象创建没创建"是两件不完全
        同步的事）。所以仍然沿用GCS后端那条"连发几次"的经验，作为
        discovery窗口的额外保险，跟"publisher是不是常驻"这条要求并不
        矛盾——常驻解决的是"避免每次都要重新建立DDS匹配"，连发解决的是
        "这次调用发生的时刻，匹配可能还没完成"，两个问题成因不同，两条
        防线都要留着。
        """
        for _ in range(repeat):
            pub.publish(msg)
            time.sleep(gap_s)

    def _publish_takeoff_land(self, cmd_value: int, repeat: int = 3, gap_s: float = 0.2) -> None:
        msg = self._TakeoffLand()
        msg.takeoff_land_cmd = cmd_value
        self._publish_with_retry(self._takeoff_land_pub, msg, repeat=repeat, gap_s=gap_s)

    def takeoff(self, timeout: float = 60.0, height_m: Optional[float] = None) -> None:
        """起飞：发布`TakeoffLand{TAKEOFF}`，阻塞直到`armed=True`**且**
        飞控自己的起飞状态机真正爬升到位、转入稳定悬停——不是`armed=
        True`就立刻返回。

        2026-09-17新增（用户明确要求"这是所有起飞都应该用的流程"，对
        所有飞机一视同仁）：完整流程现在是——①前置检查PX4飞控已连接、
        UWB/里程计定位数据已就绪（各自独立超时，报错能说清楚具体卡在
        哪一步）；②给一次声光反馈（按角色播'侦察机起飞'/'任务机起飞'，
        硬件不存在时只警告不阻断）；③发布起飞指令，等`armed=True`+
        位置稳定（原有逻辑，见下面说明）；④额外强制悬停`HOVER_AFTER_
        TAKEOFF_S`（5秒）才真正返回，给控制器更充分的收敛裕量。全程
        每个关键节点都用`_progress()`报状态。

        2026-09-14订正（用户直接指出这个问题）：原来这个方法只等
        `mavros/state`确认`armed=True`就返回，但`armed=True`只代表
        `PX4`接受了解锁请求，不代表`pt4ctrl`自己的`AUTO_TAKEOFF`状态
        已经真正爬升到目标高度、转入稳定悬停的`AUTO_HOVER`（
        `PX4CtrlFSM.cpp`里`case AUTO_TAKEOFF`分支：先有几秒电机预热
        延迟，再持续爬升，直到`odom_data.p(2) >= 起飞前高度+takeoff_
        height`才`state = AUTO_HOVER`）——`armed=True`和这次状态转换
        之间隔着这几秒的爬升过程，如果这时候立刻发第一个`goto()`，
        飞机实际还在爬升途中，规划器拿到的起点跟真实物理状态对不上，
        这正是D3实测反复复现"起飞后立刻`goto()`必卡/甚至撞障碍物"这
        一族问题的根子（详见`DEBUG_JOURNAL.md` 2026-09-14相关记录）。

        `pt4ctrl`没有对外发布FSM状态话题，没法直接查"是不是已经进入
        `AUTO_HOVER`"，也没有现成的跨节点接口查它自己的`auto_takeoff_
        land.takeoff_height`参数具体是多少——改成检测一个等价的外部
        可观测信号："armed=True之后，位置连续`TAKEOFF_STABLE_WINDOW_S`
        秒都没有明显变化"（用`_odom_xyz`采样，窗口内任意两个采样点的
        距离都不超过`TAKEOFF_STABLE_POS_TOLERANCE_M`）——只要飞机还在
        爬升，位置就在持续变化，这个判据不会通过；爬升真正停止、转入
        悬停之后，位置才会稳定下来满足这个窗口条件。这个判据不需要知道
        `takeoff_height`具体配的是多少，对这个值以后被调整是安全的。

        Args:
            timeout: 总超时秒数，同时覆盖"等`armed=True`"和"等位置
                稳定"这两段。默认从30秒提到60秒——实测确认这套仿真
                （`uwb_imu`+`pt4ctrl`组合）从解锁到真正稳定悬停，中间
                有一段爬升超调+振荡衰减的过程，衰减到0.3米容差以内
                经常要20~30秒量级，30秒的旧默认值不够用。
            height_m: 起飞到多高（米，离地）。不给就用飞控 `pt4ctrl` 里
                配置的 takeoff_height。
        Raises:
            TakeoffTimeoutError: 前置检查超时（PX4未连接/定位数据未就绪），
                或者超过`timeout`秒仍未等到`armed=True`，或者等到了
                `armed=True`但位置一直没有稳定下来——具体是哪一种看
                异常消息本身，`stage`字段区分了这几种情况。

        2026-09-17新增安全修复（实测直接定位到"起飞后一转入航点飞行就
        急剧掉高、甚至直接掉地上"的根因）：`traj_server`的`CONSTANT`
        朝向模式默认目标值硬编码`0.0`弧度，但飞机实际停机朝向往往不是
        0度（比如NX01 spawn yaw=90度）——起飞解锁那一刻控制器就要开始
        朝着这个目标转向，如果目标值跟飞机真实朝向对不上，等于是逼着
        飞机在起飞爬升、转入航点飞行的同时还要completing一次大幅度
        转向，跟位置/高度控制抢电机推力资源，这才是掉高的真正根因（
        跟`set_yaw_mode_point()`绕飞时的yaw跟踪算法是两回事，那个问题
        更早之前就已经通过"默认切到CONSTANT模式"处理过了）。
        这里在发布起飞指令**之前**先读一次真实yaw角、显式设成CONSTANT
        目标，从根上消除这个不匹配——`self.pretakeoff_yaw`记录下来，
        供后续"绕飞完/命中后要恢复到起飞前朝向"这类场景使用（不要再
        写死`0.0`）。
        """
        # 2026-09-25 用户要求：起飞可以直接带高度，省掉"起飞到1米再 goto 爬上去"
        # 这一次纯垂直规划。不传就用飞控里原来的值（1.0 米），行为跟以前一字不差。
        # 上限 TAKEOFF_HEIGHT_MAX_M 不是拍的：规划器有天花板约束
        # `巡航高度 < virtual_ceil_height(4.5) - 0.1 - dist0(1.5) = 2.9`，起飞高度
        # 超过它，起飞后第一次重规划就会被天花板代价往下推。
        if height_m is not None:
            if not (TAKEOFF_HEIGHT_MIN_M <= height_m <= TAKEOFF_HEIGHT_MAX_M):
                raise ValueError(
                    f'起飞高度 {height_m} 米超出允许范围 '
                    f'[{TAKEOFF_HEIGHT_MIN_M}, {TAKEOFF_HEIGHT_MAX_M}]——下限是地效区，'
                    f'上限受规划器天花板约束（virtual_ceil_height-0.1-dist0）')
            ok = self._call_set_parameters_blocking(
                self._pt4ctrl_params_cli,
                {'auto_takeoff_land.takeoff_height': float(height_m)},
                timeout_s=5.0,
            )
            if not ok:
                raise ActionFailedError(
                    action_name='takeoff:set_height', timeout_s=5.0, namespace=self.namespace)
            self._progress(f'起飞高度设为 {height_m:.2f} m（pt4ctrl 的 takeoff_height）')

        # 2026-09-17新增：所有飞机起飞都要走的统一前置检查+反馈流程（用户
        # 明确要求）——起飞前先确认PX4飞控已连接、UWB/里程计定位数据已经
        # 就绪，两项检查各自独立报超时原因（不要笼统报成"没等到armed"，
        # 那样排查时看不出到底是连接问题还是定位问题）；给一次声光反馈
        # （按角色播'侦察机起飞'/'任务机起飞'）；起飞过程/前置检查的每
        # 个关键节点都用`_progress()`往外报状态（这是这个SDK一贯的"回传
        # 消息"机制，所有阻塞方法都在用，不是新发明一套）。
        ok = self._poll_until(
            lambda: self._connected is True,
            PREFLIGHT_CHECK_TIMEOUT_S,
            lambda: self._progress(f'起飞前置检查…等待PX4飞控连接，当前connected={self._connected}'),
        )
        if not ok:
            raise TakeoffTimeoutError(
                timeout_s=PREFLIGHT_CHECK_TIMEOUT_S, namespace=self.namespace,
                stage='preflight_connected')
        self._progress('起飞前置检查：PX4飞控已连接')

        ok = self._poll_until(
            lambda: self._odom_xyz is not None and self._current_yaw is not None,
            PREFLIGHT_CHECK_TIMEOUT_S,
            lambda: self._progress('起飞前置检查…等待UWB/里程计定位数据就绪'),
        )
        if not ok:
            raise TakeoffTimeoutError(
                timeout_s=PREFLIGHT_CHECK_TIMEOUT_S, namespace=self.namespace,
                stage='preflight_uwb')
        self._progress('起飞前置检查：UWB/里程计定位数据已就绪')

        # 声光反馈是可选外设——很多仿真环境根本没接这块硬件，板子不存在
        # 不应该阻止飞机起飞，`_play_role_sound_light()`只打警告不抛异常。
        self._play_role_sound_light('起飞')

        # 必须用"稳定后"的 yaw，不能用 get_current_yaw() 的瞬时第一帧——
        # 见 YAW_SETTLE_SAMPLES 上面那段实测说明（起飞前读到 0°、飞机实际
        # 朝向 90°，结果这行代码把机头转走了）。
        self.pretakeoff_yaw = self._settled_yaw()
        self.set_yaw_mode_constant(self.pretakeoff_yaw)

        # 2026-09-20：起飞完成判定已下沉到机载（takeoff_monitor_node 的
        # Takeoff action）。这里优先走 action：发一个 goal、等 result，
        # 判定回路整个留在飞机上，链路延迟只影响拿到结论的快慢。
        #
        # 机载节点不在（flight-stack 镜像还没重新 build）时自动退回下面
        # 那条老路——自己发 takeoff_land 话题 + 在选手侧轮询判定。两个
        # 镜像因此可以分开重建，中间状态不会坏。
        if self._try_takeoff_via_action(timeout):
            return

        self._progress('机载起飞 action 不可用，退回选手侧判定（行为与改造前一致）')

        # 跟机载 server 同一条短路：已经在空中就直接返回，不重复下发起飞
        # 指令（飞机正在 AUTO_HOVER，再塞一条 TAKEOFF 只会扰动状态机）。
        if self._armed is True and self._odom_xyz is not None \
                and self._odom_xyz[2] >= ALREADY_AIRBORNE_Z_M:
            self._progress(
                f'飞机已解锁且高度{self._odom_xyz[2]:.2f}m，判定为已在空中，跳过起飞'
            )
            return

        self._publish_takeoff_land(self._TakeoffLand.TAKEOFF)

        def _progress_armed() -> None:
            z = self._odom_xyz[2] if self._odom_xyz is not None else None
            z_text = f'{z:.2f}m' if z is not None else '未知（还没收到里程计）'
            self._progress(f'起飞中…当前高度{z_text}，armed={self._armed}')

        start_time = time.monotonic()
        ok = self._poll_until(lambda: self._armed is True, timeout, _progress_armed)
        if not ok:
            raise TakeoffTimeoutError(timeout_s=timeout, namespace=self.namespace)

        # armed刚确认那一刻的高度基线——见下面_pos_stable()里
        # TAKEOFF_MIN_CLIMB_M那部分的说明，没有里程计数据时用0.0占位
        # （极端情况下armed确认了但还没收到过一帧里程计，理论上不应该
        # 发生，因为AUTO_TAKEOFF本身要求先有odom才会真的开始爬升，这里
        # 只是防御性兜底，不代表预期会走到这个分支）。
        z_at_armed = self._odom_xyz[2] if self._odom_xyz is not None else 0.0

        # 位置稳定窗口检测：滑动窗口只保留最近TAKEOFF_STABLE_WINDOW_S
        # 秒内的采样，一旦"已经连续采样满了这个窗口时长"（用单独的
        # first_sample_time判断，不是用trim剩下的history[0]，见下面
        # 2026-09-14踩坑说明）且窗口内任意两点距离都在容差内，就认为
        # 爬升+状态转换已经完成。
        #
        # ⚠️ 2026-09-14实测踩坑：第一版把"窗口是否攒够时长"这个判断也
        # 建立在trim剩下的`history[0]`上（`now - history[0][0] < WINDOW`
        # 才返回False），但上面那行trim本身就是"只保留age<=WINDOW的
        # 条目"——trim完之后剩下的最老条目age必然满足`<= WINDOW`，这个
        # "是否攒够时长"的判断因此**永远为真**（返回False），不管实际
        # 已经采样了多久，函数永远在这一步提前退出，`_pos_stable()`
        # 永远不可能返回True，`takeoff()`因此稳定在60秒超时（实测复现：
        # 加了一行调试print确认后面判断真实占据的那几行代码从未被执行
        # 到过）。改成额外单独记一个`first_sample_time`（只在第一次
        # 调用时赋值一次，不随trim变化），"是否攒够窗口时长"改用这个
        # 独立时间戳判断，不再依赖trim剩下的条目，两件事分开判断。
        history: list = []  # [(monotonic_time, (x, y, z)), ...]
        first_sample_time: Optional[float] = None

        def _pos_stable() -> bool:
            nonlocal first_sample_time
            if self._odom_xyz is None:
                return False
            now = time.monotonic()
            if first_sample_time is None:
                first_sample_time = now
            history.append((now, tuple(self._odom_xyz)))
            while history and now - history[0][0] > TAKEOFF_STABLE_WINDOW_S:
                history.pop(0)
            if now - first_sample_time < TAKEOFF_STABLE_WINDOW_S:
                return False  # 还没连续采样满一个窗口时长，先不判断
            if self._odom_xyz[2] - z_at_armed < TAKEOFF_MIN_CLIMB_M:
                return False  # 还没真正爬升过，"停在地面不动"不算数
            xs = [p[0] for _, p in history]
            ys = [p[1] for _, p in history]
            zs = [p[2] for _, p in history]
            spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            return spread <= TAKEOFF_STABLE_POS_TOLERANCE_M

        def _progress_stable() -> None:
            z = self._odom_xyz[2] if self._odom_xyz is not None else None
            z_text = f'{z:.2f}m' if z is not None else '未知（还没收到里程计）'
            self._progress(f'起飞已解锁，等待爬升到位+状态转换稳定…当前高度{z_text}')

        remaining = max(0.0, timeout - (time.monotonic() - start_time))
        ok = self._poll_until(_pos_stable, remaining, _progress_stable)
        if not ok:
            raise TakeoffTimeoutError(timeout_s=timeout, namespace=self.namespace, stage='stable')
        self._progress('起飞完成，armed=True，位置已稳定')

        # 2026-09-17新增：用户明确要求"起飞后应该悬停5秒，稳定后出发"——
        # 上面`_pos_stable()`判定的"稳定"只是最近1.5秒窗口内位置没有明显
        # 漂移，强度不够，额外强制悬停这么久再把控制权交还给调用方，让
        # 控制器有更充分的收敛裕量再接第一个goto()。
        self._progress(f'起飞后额外悬停{HOVER_AFTER_TAKEOFF_S:.0f}秒，稳定后再出发…')
        time.sleep(HOVER_AFTER_TAKEOFF_S)
        self._progress('悬停确认稳定，可以出发')

    def _try_takeoff_via_action(self, timeout: float) -> bool:
        """尝试走机载的 Takeoff action。

        Returns:
            True 表示 action 路径走完且起飞成功；False 表示机载 server
            不可用（镜像还没重建、或这次用的控制器组合没起这个节点），
            调用方应退回老路。

        Raises:
            TakeoffTimeoutError: server 在、但明确返回失败或超时——这种
                情况不能退回老路重发一次起飞指令（飞机可能已经在空中），
                直接把失败抛给调用方，`stage` 带上机载判定的阶段。
        """
        if self._takeoff_action_cli is None:
            return False
        # server 不在就立刻退回，不白等：这一步只是探测，不是等起飞
        if not self._takeoff_action_cli.wait_for_server(timeout_sec=ACTION_SERVER_PROBE_TIMEOUT_S):
            return False

        goal = self._Takeoff.Goal()
        goal.timeout_s = float(timeout)

        def _on_feedback(msg: Any) -> None:
            fb = msg.feedback
            self._progress(
                f'起飞中…阶段{fb.phase}，armed={fb.armed}，当前高度{fb.current_z:.2f}m，'
                f'已爬升{fb.climb_m:.2f}m（判定在机载）'
            )

        send_future = self._takeoff_action_cli.send_goal_async(goal, feedback_callback=_on_feedback)
        if not self._wait_future(send_future, ACTION_GOAL_ACCEPT_TIMEOUT_S):
            raise TakeoffTimeoutError(
                timeout_s=ACTION_GOAL_ACCEPT_TIMEOUT_S, namespace=self.namespace,
                stage='action_goal_no_response')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            # server 在但拒绝了——通常是上一次起飞流程还没结束
            raise TakeoffTimeoutError(
                timeout_s=timeout, namespace=self.namespace, stage='action_goal_rejected')

        result_future = goal_handle.get_result_async()
        # 只等机载返回结果，不在这里维护第二套时间预算——见
        # TAKEOFF_ACTION_HARD_TIMEOUT_S 的说明。机载每个阶段都有自己的
        # 超时，失败也会返回带 stage 的结果，客户端照原样报出去就行。
        if not self._wait_future(result_future, TAKEOFF_ACTION_HARD_TIMEOUT_S):
            raise TakeoffTimeoutError(
                timeout_s=TAKEOFF_ACTION_HARD_TIMEOUT_S, namespace=self.namespace,
                stage='action_no_result')

        result = result_future.result().result
        if not result.success:
            self._progress(f'机载起飞判定失败：{result.message}')
            raise TakeoffTimeoutError(
                timeout_s=timeout, namespace=self.namespace, stage=result.stage)
        self._progress(f'起飞完成（机载判定）：{result.message}')
        return True

    def _wait_future(self, future: Any, timeout_s: float) -> bool:
        """在调用方线程里轮询等待一个 future 完成。

        不能用 `rclpy.spin_until_future_complete()`：这个 SDK 的 rclpy
        运行时已经有一条后台 spin 线程在跑（见 `_rclpy_runtime.py`），
        再在别的线程里 spin 同一个 executor 会互相打架。后台线程照常
        处理回调，这里只需要轮询 future 的完成标志。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

    def land(self, timeout: float = 30.0) -> None:
        """降落：发布`TakeoffLand{LAND}`，阻塞直到`mavros/state`确认
        `armed`变为`False`。

        Raises:
            LandTimeoutError: 超过`timeout`秒仍未确认`armed=False`（见
                `exceptions.py::LandTimeoutError`docstring关于"为什么
                不复用`TakeoffTimeoutError`"的说明）。
        """
        # 2026-09-17新增：跟`takeoff()`对称的声光反馈（同样是可选外设，
        # 板子不存在不应该阻止降落）。
        self._play_role_sound_light('降落')

        self._publish_takeoff_land(self._TakeoffLand.LAND)

        def _progress() -> None:
            z = self._odom_xyz[2] if self._odom_xyz is not None else None
            z_text = f'{z:.2f}m' if z is not None else '未知（还没收到里程计）'
            self._progress(f'降落中…当前高度{z_text}，armed={self._armed}')

        ok = self._poll_until(lambda: self._armed is False, timeout, _progress)
        if not ok:
            raise LandTimeoutError(timeout_s=timeout, namespace=self.namespace)
        self._progress('降落完成，armed=False')

    def goto(self, x: float, y: float, z: float, timeout: float = 60.0) -> None:
        """飞往一个点，阻塞直到到达确认或超时。

        底层用现成的`waypoint_queue`（`nav_msgs/Path`）接口——发一个只含
        一个点的`Path`消息，订阅`waypoint_state`等到达确认（跟`ego_
        planner_bridge/rviz_goal_bridge_node.py`的实现对应：收到新队列
        立刻广播`'executing'`，飞完最后一个航点广播`'completed'`）。选用
        `waypoint_queue`而不是单点的`rviz_goal_world`话题，理由：①
        `waypoint_queue`这条链路自带`waypoint_cancel`（`cancel_goto()`
        正好能对上）和`waypoint_progress`，`rviz_goal_world`那条单点链路
        没有对应的"取消"话题；②两条链路的到达判定/状态广播逻辑在
        `rviz_goal_bridge_node.py`里是分开维护的两套状态机（`_queue_idx`
        vs `_single_goal_world`），混用容易在"单点目标"和"队列执行"之间
        产生"到底哪个正在生效"的歧义（该文件`_active_goal_world()`的
        优先级规则是"队列优先"），统一走队列接口（哪怕队列只有一个点）
        更简单、行为更可预测。

        ⚠️ **坐标系约定**：`x/y/z`是这架飞机自己的局部坐标系（跟`sdk.
        world_to_local()`的返回值、跟方案2.2.1节示例代码`sdk.goto(7,
        -10, 1.5)`直接传字面量数字的用法一致）——`waypoint_queue`的消费方
        `rviz_goal_bridge_node.py::_queue_cb()`直接把每个`PoseStamped.
        pose.position`当局部坐标使用，不做任何TF变换（只有单点的
        `rviz_goal_world`话题才会走TF变换，`waypoint_queue`不会）。如果
        选手手上是世界坐标（比如队友广播过来的检测坐标），需要先调
        `sdk.world_to_local()`换算一次再传给`goto()`。

        Args:
            x/y/z: 目标点在这架飞机自己局部坐标系下的坐标。
            timeout: 超时秒数。

        Raises:
            GotoTimeoutError: 超过`timeout`秒仍未确认到达。
        """
        path = self._Path()
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.header.frame_id = self._target_frame
        pose = _make_pose_stamped(path.header, x, y, z)
        path.poses = [pose]

        # 发布前先清掉可能残留的上一次goto()留下的状态（比如上一次是
        # 'completed'/'cancelled'），避免这次还没真正开始执行就被上一次
        # 的残留状态误判成"已到达"——rviz_goal_bridge_node收到新队列会
        # 立刻广播'executing'，所以这个窗口很短，但依然要防。
        # 2026-09-13端到端实测踩过的坑：这里原来只发一次，跟`takeoff()`/
        # `land()`用的`_publish_with_retry()`连发几次的防线不一致——单发
        # 一次赶上DDS discovery/对端还没完全就位的窗口会被静默丢弃，且
        # `waypoint_queue`这条链路没有类似`mavros/state`那种持续状态可以
        # "反正最终会追上"，一旦这一次丢了`ego_planner`就永远不会收到这个
        # 目标点，`goto()`会一直等到超时（实测复现过：`waypoint_state`
        # 确实收到过`rviz_goal_bridge_node`广播的`'executing'`——这只
        # 说明`waypoint_queue`这一条消息本身送达了`_queue_cb`，但不代表
        # `ego_planner`那边真的响应了`term_goal`；换成连发几次之后，只要
        # 有一次送到discovery已经稳定的窗口，`ego_planner`就会正确执行）。
        self._waypoint_state = None
        self._publish_with_retry(self._waypoint_queue_pub, path)

        def _progress() -> None:
            cur = self._odom_xyz
            cur_text = f'({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})' if cur is not None else '未知'
            self._progress(
                f'飞往({x:.2f}, {y:.2f}, {z:.2f})中…当前位置{cur_text}，'
                f'waypoint_state={self._waypoint_state!r}'
            )

        # 卡住检测：只在规划器已经接收目标（waypoint_state=executing）之后
        # 才开始记位置。还没 executing 就不动，说明目标点根本没送达/没被
        # 接收，那是另一类问题，走下面原有的超时，不能报成"不可达"。
        history: List[Tuple[float, float, float, float]] = []
        stalled = {'hit': False}

        def _check() -> bool:
            # 'cancelled'见下面的说明：这里判断"已经不需要继续等待"，
            # 不是判断"已经到达"——goto()对cancelled的处理见下方。
            if self._waypoint_state in ('completed', 'cancelled'):
                return True
            if self._waypoint_state == 'executing' and self._odom_xyz is not None:
                now = time.monotonic()
                history.append((now,) + tuple(self._odom_xyz))
                while history and history[0][0] < now - 2 * GOTO_STALL_WINDOW_S:
                    history.pop(0)
                if _goto_stalled(history, (x, y, z), GOTO_STALL_WINDOW_S,
                                 GOTO_STALL_MIN_MOVE_M, GOTO_STALL_MIN_DIST_M):
                    stalled['hit'] = True
                    return True
            return False

        ok = self._poll_until(_check, timeout, _progress)
        if stalled['hit']:
            cur = self._odom_xyz
            dist = math.sqrt(sum((cur[i] - (x, y, z)[i]) ** 2 for i in range(3)))
            # 先撤掉这个目标再抛：不然规划器还在反复尝试一个到不了的点，
            # 调用方如果没有立刻发下一个 goto，飞机就一直在那儿折腾。
            self.cancel_goto()
            self._progress(
                f'目标点({x:.2f}, {y:.2f}, {z:.2f})不可达：已停在'
                f'({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})，离目标{dist:.2f}米，放弃这个点'
            )
            raise GotoUnreachableError(
                target_xyz=(x, y, z), stopped_xyz=tuple(round(v, 2) for v in cur),
                distance_m=dist, namespace=self.namespace)
        if not ok:
            raise GotoTimeoutError(timeout_s=timeout, target_xyz=(x, y, z), namespace=self.namespace)
        if self._waypoint_state == 'cancelled':
            # 判断依据：这次goto()是被别的线程/回调调用cancel_goto()主动
            # 打断的，不是飞不到/超时——这是选手代码自己触发的控制流，
            # 属于"正常但没有到达目标"的结果，不当成失败抛异常（如果这里
            # 抛异常，选手每次故意调用cancel_goto()打断当前goto都要包一层
            # try/except，体验很差；而且跟"确实没到达、也没被取消、纯粹
            # 超时"这两种真正的失败场景混在一起报同一个异常类型也不合理）。
            # 只打印进度提示，正常返回。
            self._progress('目标点已被取消，goto()提前返回（未到达目标点）')
            return
        self._progress(f'已到达({x:.2f}, {y:.2f}, {z:.2f})')

    def goto_route(self, points, timeout: float = 300.0) -> None:
        """一次下发整条航线，阻塞到最后一个航点到达确认或超时。

        跟连着调好几次 `goto()` 的区别，**不只是少几次往返**：

        - 逐段 `goto()` 时，每一段对 ego_planner 都是一个独立的终点，规划出来
          的轨迹**终点速度为零**；桥接节点又要等飞机进到 0.3 米球内才判到点、
          才发下一个目标。于是每个航点必然"减速到 0 → 判到点 → 从 0 重新加速"。
          2026-09-28 实测编队航线四个拐点最低速度 0.01~0.09 m/s、速度<0.3 的
          时长各 2.2~5.4 秒，长机巡航速度中位只有 0.59 m/s（限速 1.0）。
        - 整条航线一次下发之后，桥接节点自己按顺序推进；配合它的
          `flythrough_radius_m` 参数（环境变量 `WAYPOINT_FLYTHROUGH_M`），
          **中间**航点可以提前切换，ego_planner 从"还在动"的状态重新规划，
          带着速度拐过去。最后一个航点仍然要真的到位。

        坐标系同 `goto()`：每个点都是这架飞机**自己局部坐标系**下的 (x, y, z)，
        `waypoint_queue` 的消费方不做任何 TF 变换。世界坐标要先过
        `sdk.world_to_local()`。

        Args:
            points: [(x, y, z), ...]，至少一个点。
            timeout: 整条航线的总超时秒数。

        Raises:
            GotoTimeoutError: 超时仍未确认飞完最后一个航点。
        """
        pts = [tuple(float(v) for v in p) for p in points]
        if not pts:
            raise ValueError('goto_route() 至少要给一个航点')
        path = self._Path()
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.header.frame_id = self._target_frame
        path.poses = [_make_pose_stamped(path.header, x, y, z) for x, y, z in pts]
        # 连发几次的理由同 goto()：单发一次赶上 DDS discovery 窗口会被静默丢弃，
        # 而 waypoint_queue 这条链路没有"反正最终会追上"的持续状态兜底。
        self._waypoint_state = None
        self._publish_with_retry(self._waypoint_queue_pub, path)
        last = pts[-1]

        def _progress() -> None:
            cur = self._odom_xyz
            cur_text = f'({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})' if cur is not None else '未知'
            self._progress(
                f'航线执行中（{len(pts)}个航点，终点 {last[0]:.2f}, {last[1]:.2f}）…'
                f'当前位置{cur_text}，waypoint_state={self._waypoint_state!r}'
            )

        # 这里**不做**逐点的卡死检测：中间航点是穿越式切换的，飞机在某个点
        # 附近"没怎么动"是正常的（它正在拐弯），拿 goto() 那套判据会误报。
        # 整条航线只靠 timeout 兜底。
        def _check() -> bool:
            return self._waypoint_state in ('completed', 'cancelled')

        self._progress(f'下发航线：{len(pts)}个航点，终点({last[0]:.2f}, {last[1]:.2f}, {last[2]:.2f})')
        if not self._poll_until(_check, timeout, _progress):
            raise GotoTimeoutError(timeout_s=timeout, target_xyz=last, namespace=self.namespace)

        if self._waypoint_state == 'cancelled':
            self._progress('航线已被取消，goto_route()提前返回（未飞完）')
            return
        self._progress(f'航线执行完毕，终点({last[0]:.2f}, {last[1]:.2f}, {last[2]:.2f})')

    def cancel_goto(self) -> None:
        """打断当前正在进行的`goto()`（发布`Empty`到`waypoint_cancel`）。

        清单B3遗漏、D4/D6节已经在用的方法，这次实现顺手补上——不是2.1节
        编号列出的能力，但跟`goto()`是同一条链路的一体两面（`goto()`选用
        `waypoint_queue`接口的一个理由就是它自带`waypoint_cancel`能对应
        上，见`goto()`docstring）。

        这个方法本身不阻塞、不等待确认——跟`goto()`的"发送+等待状态反馈"
        不是同一种语义：`cancel_goto()`更像是一个"尽力而为的中断信号"，
        典型用法是从另一个线程/回调（比如`on_teammate_event()`注册的
        回调）打断当前正阻塞在`goto()`里的主线程，如果这里也设计成阻塞
        等待"取消确认"，反而会跟"谁在等谁"的场景绑得更死（发起取消的
        那个回调本身往往也不适合被再次阻塞）。真正的"取消生效了"确认，
        由正在执行`goto()`的那个调用自己通过`waypoint_state`变成
        `'cancelled'`来感知（见`goto()`的`_check()`逻辑）。
        """
        # 同样套用`_publish_with_retry()`（见2026-09-13实测踩坑说明，
        # `goto()`那边的注释）——`cancel_goto()`本身是"尽力而为、不阻塞
        # 等确认"的语义没有变，只是把"这一次发送"这个动作本身做得更可靠，
        # 跟"要不要等对方确认"是两件独立的事。
        self._publish_with_retry(self._waypoint_cancel_pub, self._Empty(), repeat=3, gap_s=0.2)
        self._progress('已发送取消当前航点指令')

    # ------------------------------------------------------------------
    # 2.1节能力3：跨机通信（转调B4 reliability.py，选手完全无感底层协议）
    # ------------------------------------------------------------------

    def send_to_teammate(
        self, event: str, timeout_s: float = DEFAULT_SEND_TO_TEAMMATE_TIMEOUT_S, **kwargs: Any
    ) -> None:
        """发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛
        `TeammateUnreachableError`（方案2.3节"链路B"，`reliability.py`
        已经实现好ACK+1Hz重发协议，这里只是转调）。

        Args:
            event: 事件名字符串（选手自定义，需要跟队友`on_teammate_
                event()`注册的名字完全一致）。
            timeout_s: 总超时秒数，默认45秒（方案建议区间30-60秒的
                中间值），可以覆盖。
            **kwargs: 随事件一起带给对方的业务数据（必须是JSON能表示的
                类型）。
        """
        # send_and_wait_ack()内部是一整套不可拆分的阻塞等待循环（1Hz
        # 重发+等ack），这个文件不应该也不能碰它内部的逻辑（那是B4的
        # 职责），所以用_run_with_progress_ticker()在外面另起一个线程
        # 定期打印进度，不动reliability.py一行代码。
        self._run_with_progress_ticker(
            lambda: self._event_channel.send_and_wait_ack(event, timeout_s=timeout_s, **kwargs),
            lambda: self._progress(f"发送事件'{event}'给{self.teammate_namespace}，等待对方确认…"),
        )
        self._progress(f"事件'{event}'已被{self.teammate_namespace}确认收到")

    def on_teammate_event(self, event_name: str, callback: Callable[..., None]) -> None:
        """注册"收到队友发来的某个事件时"的处理回调（方案2.3节"链路B"，
        转调`reliability.py::register_event_handler()`）。

        Args:
            event_name: 事件名字符串，跟队友`send_to_teammate(event=...)`
                传的字符串完全一致才能匹配上。
            callback: 形如`callback(**kwargs)`的函数，会在`RclpyRuntime`
                的后台spin线程里被调用（不是选手代码自己的主线程）——
                如果选手的回调内部要做比较重的事，建议自己另起线程，
                避免拖慢这个进程里其它ROS2回调的处理。
        """
        self._event_channel.register_event_handler(event_name, callback)

    # ------------------------------------------------------------------
    # 2.1节能力8：通用航点生成helper（vendor自geometry_helpers.py，转调
    # 模块级纯函数，见该文件模块头说明——两种调用方式都留：
    # `sdk.generate_orbit_waypoints(...)`和`from contest_sdk.geometry_
    # helpers import generate_orbit_waypoints`都能用）。
    # ------------------------------------------------------------------

    def generate_orbit_waypoints(self, *args: Any, **kwargs: Any):
        return _generate_orbit_waypoints(*args, **kwargs)

    def generate_ground_scan_waypoints(self, *args: Any, **kwargs: Any):
        return _generate_ground_scan_waypoints(*args, **kwargs)

    def pull_waypoints_out_of_circles(self, *args: Any, **kwargs: Any):
        """把落进已知圆形障碍物（含余量）里的航点沿来路往回挪到外面。
        见 geometry_helpers.pull_waypoints_out_of_circles()。"""
        return _pull_waypoints_out_of_circles(*args, **kwargs)

    # ------------------------------------------------------------------
    # 2.1节能力9：驱动地面站声光装置（链路D，唯一一处刻意不抛异常）
    # ------------------------------------------------------------------

    def trigger_alarm(self, pattern: str, duration_s: Optional[float] = None) -> None:
        """触发GCS所在机器的声光装置（方案2.1节第9条/链路D）。

        ⚠️ **具体接口方式目前是未知数**（需要跟GCS开发者对接确定，这次
        做不了）——这里实现成一个占位版本：内部尝试一次简单的HTTP POST
        调用（`DEFAULT_ALARM_URL`是明显的占位地址，见本文件模块头常量
        定义处的说明），失败/超时**吞掉异常、只打警告日志，不向选手抛
        出去**（方案2.3节"链路D"明确这是2.2.1节"选手不应该接触裸异常"
        原则之外，唯一一处刻意选择不向选手抛出异常的能力——声光装置
        纯粹是人机提示体验，不应该让一个外设/协议还没定下来的功能拖垮
        真正的任务执行逻辑）。**这跟本文件其它方法"超时要抛contest_sdk
        异常"的模式刻意不一样，不要套用别的方法的错误处理模板。**

        Args:
            pattern: 声光模式字符串（选手自定义，比如`'fire_found'`）。
            duration_s: 持续时长（秒），`None`表示用GCS那边的默认值
                （具体默认值由GCS后端决定，这一层不关心）。
        """
        payload = {'pattern': pattern, 'duration_s': duration_s, 'namespace': self.namespace}
        last_error: Optional[BaseException] = None
        for attempt in range(1, DEFAULT_ALARM_RETRIES + 1):
            try:
                body = json.dumps(payload).encode('utf-8')
                request = urllib.request.Request(
                    self._alarm_url,
                    data=body,
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urllib.request.urlopen(request, timeout=self._alarm_timeout_s):
                    pass
                self._progress(f"声光装置触发请求已发出（pattern={pattern!r}）")
                return
            except (urllib.error.URLError, OSError, ValueError) as exc:
                last_error = exc
                self._progress(
                    f"声光装置触发第{attempt}/{DEFAULT_ALARM_RETRIES}次尝试失败：{exc!r}"
                )
        # 全部尝试都失败：只打警告，不抛异常——见方法docstring"⚠️"一段。
        self._progress(
            f"[警告] 声光装置触发最终失败（pattern={pattern!r}），已放弃重试，"
            f"不影响任务继续执行；最后一次错误：{last_error!r}。GCS侧接口目前是"
            f"占位实现（{self._alarm_url}），具体协议待跟GCS开发者确定后再联调。"
        )

    # ------------------------------------------------------------------
    # 机载声光反馈板（协议来自《声光反馈程序接口.xlsx》，2026-09-16在
    # 真实硬件上验证通过）——注意这不是`trigger_alarm()`的实现，是独立
    # 的能力，见上面`DEFAULT_SOUND_LIGHT_PORT`常量定义处的说明。
    # ------------------------------------------------------------------

    def play_sound_light(
        self, event: str, repeat: Optional[int] = None, interval_ms: Optional[int] = None,
    ) -> None:
        """触发地面站声光反馈板（WS2812D灯带+扬声器），播放20条预置事件之一。

        默认（`SOUND_LIGHT_MODE=server`）只是往`/sound_light/request`话题发
        一条请求就返回，真正写串口的是常驻程序`sound_light_server.py`——
        两架飞机共用一块板子，由它统一排队（两条之间至少间隔2.5秒，不会
        互相覆盖）。常驻程序没在线时这里只打一行警告，不抛异常。
        `SOUND_LIGHT_MODE=direct`时退回本进程直接开串口（第一次调用额外
        等约2秒板子复位，见`_sound_light_port.py`模块头"真机踩坑"）。

        ⚠️ direct模式下板子收到新指令会立刻切换，**不会排队**——连续调用
        之间要自己加够`time.sleep()`；server模式由常驻程序负责间隔。

        Args:
            event: 事件名，必须是`DroneSDK.SOUND_LIGHT_EVENTS`的20个键之一
                （逐字对应《声光反馈程序接口.xlsx》"语音"列，比如
                `'侦察机发现地面火情'`/`'任务机抓取灭火弹'`）。
            repeat: 循环播放次数，默认`None`=用表格里的值。
            interval_ms: 每次循环之间的间隔（毫秒），默认`None`=用表格里的值。

        Raises:
            ValueError: `event`不在20条预置事件之内，或`repeat`/`interval_ms`
                为负数。
            SoundLightError: 仅direct模式——串口打开/写入失败（设备未接好、
                被占用、权限不足等，见该异常docstring的排查提示）。
        """
        if self._sound_light_mode == 'server':
            self._request_sound_light(
                _build_sound_light_request(event, repeat, interval_ms, source=self.namespace), event)
            return
        if event not in SOUND_LIGHT_EVENTS:
            raise ValueError(
                f"未知声光事件：{event!r}，可选：{list(SOUND_LIGHT_EVENTS)}"
            )
        r, g, b, sound, default_repeat, default_interval_ms = SOUND_LIGHT_EVENTS[event]
        command = _encode_sound_light_command(
            r, g, b, sound,
            default_repeat if repeat is None else repeat,
            default_interval_ms if interval_ms is None else interval_ms,
        )
        self._send_sound_light(command)
        self._progress(f"声光反馈：{event}（{command.strip()!r}）")

    def mute_sound_light(self) -> None:
        """熄灯+静音（`0,0,0,0,1,0`）。server模式下会清空常驻程序的待播队列。"""
        if self._sound_light_mode == 'server':
            self._request_sound_light(_build_sound_light_request(mute=True, source=self.namespace), '熄灯静音')
            return
        command = _encode_sound_light_command(*_SOUND_LIGHT_MUTE_ARGS)
        self._send_sound_light(command)
        self._progress("声光反馈：熄灯静音")

    def _play_role_sound_light(self, action: str) -> None:
        """`takeoff()`/`land()`用：按角色播"侦察机起飞"/"任务机降落"这类
        事件。角色不是recon/supply时表格里没有对应语音，跳过。

        **每种动作只播第一次**（2026-09-30 用户要求："任务机起飞降落播报只保留
        第一次"）。任务流程里任务机一趟要起降三次——初次起飞、物资点取器材、
        物资点放器材——`takeoff()`/`land()` 每次都自动播一遍，实测"任务机起飞"
        播了 3 遍、"任务机降落"同理。评分/裁判如果按播报识别流程节点，重复的
        那两遍就是噪声。所以同一个 DroneSDK 实例内，'起飞'和'降落'各只播一次。
        """
        prefix = _SOUND_LIGHT_ROLE_PREFIX.get(self.role)
        if prefix is None:
            return
        if action in self._role_sound_light_done:
            self._progress(f"{prefix}{action}已播过，不再重复")
            return
        self._role_sound_light_done.add(action)
        try:
            self.play_sound_light(f'{prefix}{action}')
        except SoundLightError as exc:
            self._progress(f'声光反馈板不可用，跳过（不影响{action}）：{exc}')

    def _request_sound_light(self, payload: str, label: str) -> None:
        """server模式：往常驻程序的请求话题发一条（懒加载发布者）。"""
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        if self._sound_light_pub is None:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                depth=50, durability=DurabilityPolicy.VOLATILE,
            )
            self._sound_light_pub = self._node.create_publisher(String, _SOUND_LIGHT_REQUEST_TOPIC, qos)
            # 刚创建的发布者要等DDS发现常驻程序的订阅者才能投递，否则第一条
            # 会发到空处。本机回环发现通常几十毫秒，最多等1秒。
            deadline = time.monotonic() + 1.0
            while self._sound_light_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.05)
        if self._sound_light_pub.get_subscription_count() == 0:
            self._progress(
                f"[警告] 声光常驻程序不在线（{_SOUND_LIGHT_REQUEST_TOPIC}没有订阅者），"
                f"'{label}'这条声光反馈不会被播放，不影响任务继续。检查"
                f"start_sound_light_server.sh起的容器是否在运行。"
            )
        self._sound_light_pub.publish(String(data=payload))
        self._progress(f"声光反馈：{label}（已发给常驻程序）")

    def _send_sound_light(self, command: str) -> None:
        """direct模式：`play_sound_light()`/`mute_sound_light()`共用，懒加载/
        复用同一个`SoundLightPort`连接（见该类docstring"真机踩坑"说明——
        不能每次发送都各自开关串口）。
        """
        if self._sound_light_conn is None:
            self._sound_light_conn = _SoundLightPort(
                self._sound_light_port, self._sound_light_baudrate, self._sound_light_timeout_s
            )
        try:
            self._sound_light_conn.send(command)
        except OSError as exc:
            raise SoundLightError(self._sound_light_port, repr(exc)) from exc

    # ------------------------------------------------------------------
    # 2.1节能力10：精确对准两种模式（center_only / precision_land）
    # ------------------------------------------------------------------

    def center_on_target(self, class_id: str, timeout: float = 30.0) -> CenteredPose:
        """触发`precision_servo_node`的`center_only`模式（只居中定位，
        不下降），阻塞直到收到`'centered'`确认，返回收敛时的坐标。

        ⚠️ **坐标系约定（2026-09-13订正）**：返回值是**这架飞机自己的
        局部坐标系**（`precision_servo_node.py::_publish_centered_pose()`
        直接发布本机`_odom_xy_z_yaw`，不是世界坐标——这里之前的文档
        说法有误，已订正，实现本身从来没变过）。如果要`send_to_
        teammate()`把这个坐标广播给队友，需要先调`sdk.local_to_world()`
        换算成世界坐标再发送，队友收到后再用它自己的`sdk.world_to_
        local()`换算回它自己的局部坐标——两架飞机局部系原点不同，不能
        跨机直传局部坐标数值。

        Raises:
            ActionFailedError: 参数设置被拒绝，或超时仍未收到`'centered'`。
        """
        self._servo_status = None
        self._centered_pose_xy = None
        set_ok = self._call_set_parameters_blocking(
            self._precision_servo_params_cli,
            {'servo_mode': 'center_only', 'target_class_id': class_id},
            timeout_s=min(5.0, timeout),
        )
        action_name = f'center_on_target:{class_id}'
        if not set_ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)

        ok = self._poll_until(
            lambda: self._servo_status == 'centered',
            timeout,
            lambda: self._progress(f"居中对准'{class_id}'中…servo_status={self._servo_status!r}"),
        )
        if not ok or self._centered_pose_xy is None:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)
        x, y = self._centered_pose_xy
        self._progress(f"已居中对准'{class_id}'，坐标=({x:.2f}, {y:.2f})")
        return CenteredPose(x=x, y=y)

    def precision_land_and_confirm(self, class_id: str, timeout: float = 60.0) -> None:
        """触发`precision_servo_node`的`precision_land`模式（精降抓取，
        收敛后下降到底并触发真正降落），阻塞直到`armed`确认变`False`
        （节点内部广播`servo_status='landed'`）。

        Raises:
            ActionFailedError: 参数设置被拒绝，或超时仍未收到`'landed'`。
        """
        self._servo_status = None
        set_ok = self._call_set_parameters_blocking(
            self._precision_servo_params_cli,
            {'servo_mode': 'precision_land', 'target_class_id': class_id},
            timeout_s=min(5.0, timeout),
        )
        action_name = f'precision_land_and_confirm:{class_id}'
        if not set_ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)

        ok = self._poll_until(
            lambda: self._servo_status == 'landed',
            timeout,
            lambda: self._progress(f"精降抓取'{class_id}'中…servo_status={self._servo_status!r}"),
        )
        if not ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)
        self._progress(f"精降抓取'{class_id}'已完成并确认落地")

    def stop_precision_servo(self, timeout: float = 5.0) -> None:
        """让`precision_servo_node`退回待命（`servo_mode=''`），把控制权交还给
        正常的位置指令通路。

        2026-09-23新增。`precision_land_and_confirm()`/`precision_land_at()`
        超时抛异常时，精降节点**并没有停**——它还在按自己的分级下降逻辑发
        `precision_land_cmd`、relay 也还在`precision_land`模式，这时候如果
        直接调`land()`，两条控制通路会抢同一架飞机。要放弃精降改用普通降落，
        必须先调这个方法。

        Raises:
            ActionFailedError: 参数设置被拒绝（节点没起来等）。
        """
        ok = self._call_set_parameters_blocking(
            self._precision_servo_params_cli, {'servo_mode': ''}, timeout_s=timeout)
        if not ok:
            raise ActionFailedError(
                action_name='stop_precision_servo', timeout_s=timeout, namespace=self.namespace)
        self._progress('精降视觉伺服已退回待命')

    def precision_land_at(self, x: float, y: float, timeout: float = 90.0) -> None:
        """飞到一个**提前已知的坐标点**精确降落（2026-09-14新增，用户
        提出的场景：起飞前记录起降点坐标，返回降落时用精准降落，不是
        普通`goto()`+`land()`——普通`goto()`只保证收敛进`ARRIVAL_
        THRESHOLD_M`(0.3米)这个到点阈值，`land()`触发的`AUTO_LAND`只在
        触发那一刻的水平位置垂直下降、不再继续修正，如果`goto()`留下
        的水平误差没消掉，落地点也会跟着偏，对"精确停回起降点/对接点"
        这类场景不够精确）。

        跟`precision_land_and_confirm()`是同一套`precision_servo_node`
        分级下降+`AUTO_LAND`交接机制（"对准一点、下降一点、再对准、
        再下降，最后一段高度交给AUTO_LAND"，2026-09-14实测验证过），
        唯一区别是水平误差的来源：这个方法不需要看得见任何AprilTag/
        二维码，直接拿`x`/`y`这个已知目标坐标跟里程计位置作差算水平
        误差——不需要目标点上贴任何视觉标识，适合"起降点"这类提前知道
        精确坐标、但不一定有tag可看的场景。

        ⚠️ **不走`ego_planner`，没有避障能力**（用户2026-09-14原话确认）
        ——飞行路径完全由这个方法自己的P控制决定，不会绕开任何障碍物。
        正确用法是先用`sdk.goto()`飞到目标点正上方（`ego_planner`带
        避障），到位之后再调用这个方法做最后一段精确降落——不要用它
        飞跨越较长、可能有障碍物的距离。

        Args:
            x/y: 目标点在这架飞机自己局部坐标系下的坐标（跟`goto()`
                同一套坐标系约定——世界坐标要先用`sdk.world_to_
                local()`换算）。
            timeout: 超时秒数。

        Raises:
            ActionFailedError: 参数设置被拒绝，或超时仍未收到`'landed'`。
        """
        self._servo_status = None
        set_ok = self._call_set_parameters_blocking(
            self._precision_servo_params_cli,
            {'servo_mode': 'coordinate_land', 'target_x': float(x), 'target_y': float(y)},
            timeout_s=min(5.0, timeout),
        )
        action_name = f'precision_land_at:({x:.2f},{y:.2f})'
        if not set_ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)

        ok = self._poll_until(
            lambda: self._servo_status == 'landed',
            timeout,
            lambda: self._progress(f"精确降落到({x:.2f}, {y:.2f})中…servo_status={self._servo_status!r}"),
        )
        if not ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)
        self._progress(f"已精确降落到({x:.2f}, {y:.2f})")

    def goto_direct(self, x: float, y: float, z: float, timeout: Optional[float] = None) -> None:
        """飞到一个坐标点并悬停（2026-09-14新增，`goto()`的"不避障、
        直达版"）——用户提出："coordinate_land不走ego_planner的话，
        就没有避障能力。但是goto可以有一个coordinate版，即不避障、
        直达版。"

        跟`precision_land_at()`是同一套`precision_servo_node`机制，
        区别是这个方法**不会触发降落**——三维同时朝目标坐标收敛（不像
        `precision_land_at()`那样先收敛水平再分级下降，因为不涉及
        贴地/地效，没有必要分级），到点后悬停在那，交还控制权。

        ⚠️ **不走`ego_planner`，没有避障能力**——只应该在已知没有障碍物
        的路段使用（比如起降点附近的开阔区域），不能拿来替代`goto()`
        跑常规、可能有障碍物的航线。不确定路径上有没有障碍物时，用
        `goto()`，不要用这个方法。

        ⚠️ **不避障、长距离时可能很慢**：底层P控制`target = 当前位置 +
        kp*(目标-当前位置)`原本是为"已经很接近目标、做最后一段精修"这个
        场景调的；2026-09-15已经修过一次"长距离实测卡死不动"的bug（根因
        是发给`pt4ctrl`的`PositionCommand`一直没填`velocity`前馈字段，
        跟距离/控制增益本身无关，见`precision_servo_node.py`同日期说明），
        修完之后长距离也能真收敛，但速度仍然由`coordinate_max_speed_mps`
        这个限速参数决定，不是`goto()`那种`ego_planner`直接规划的速度。

        Args:
            x/y/z: 目标点在这架飞机自己局部坐标系下的坐标（跟`goto()`
                同一套坐标系约定）。
            timeout: 超时秒数。**默认`None`=不设超时**（2026-09-15用户
                明确要求"goto函数不需要超时设置"——这个方法本来就该等到
                真收敛为止，不该由调用方猜一个数字，猜小了半路截断、
                猜大了没意义）：一直等到`servo_status=='arrived'`为止，
                不会中途放弃。只有明确传一个正数时才会真的限时等待。

        Raises:
            ActionFailedError: 参数设置被拒绝；`timeout`为正数时超时仍未
                收到`'arrived'`也会抛出（`timeout=None`时不会因超时抛出，
                只会因参数设置失败抛出）。
        """
        self._servo_status = None
        set_ok = self._call_set_parameters_blocking(
            self._precision_servo_params_cli,
            {
                'servo_mode': 'coordinate_goto',
                'target_x': float(x), 'target_y': float(y), 'target_z': float(z),
            },
            timeout_s=5.0,
        )
        action_name = f'goto_direct:({x:.2f},{y:.2f},{z:.2f})'
        if not set_ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)

        ok = self._poll_until(
            lambda: self._servo_status == 'arrived',
            timeout,
            lambda: self._progress(f"直飞(不避障)到({x:.2f}, {y:.2f}, {z:.2f})中…servo_status={self._servo_status!r}"),
        )
        if not ok:
            raise ActionFailedError(action_name=action_name, timeout_s=timeout, namespace=self.namespace)
        self._progress(f"已直飞(不避障)到({x:.2f}, {y:.2f}, {z:.2f})")

    def reset_aim(self, timeout: float = 5.0) -> None:
        """清空`fire_pillar_aim_node`的瞄准锁定状态（对应现成的
        `~/reset_aim` `std_srvs/srv/Trigger` service），阻塞等待service
        返回即可——不需要设计成通用的"调任意service"接口，专门封装这
        一个就够（清单B3第10条明确的要求）。

        Raises:
            ActionFailedError: service调用超时，或者对方返回`success=False`。
        """
        request = Trigger.Request()
        ok, response = self._call_service_blocking(self._reset_aim_cli, request, timeout_s=timeout)
        if not ok or response is None or not response.success:
            raise ActionFailedError(action_name='reset_aim', timeout_s=timeout, namespace=self.namespace)
        self._progress('瞄准状态已清空（reset_aim完成）')

    # ------------------------------------------------------------------
    # 2026-09-13实现阶段D的mission.py时发现的补充能力——D6子流程需要读
    # `fire_pillar_aim_node`持续广播的等待点/瞄准点坐标，B3清单原来漏了
    # 这类"持续订阅一个状态话题、取最新值"的能力（跟`get_mission_state()`
    # 是同一种模式，不是新发明的接口风格）。
    # ------------------------------------------------------------------

    def read_fire_pillar_staging_pose(self, timeout: float = 30.0) -> Tuple[float, float, float]:
        """读取`fire_pillar_aim_node`锁定高层火情后广播的任务机等待点
        坐标（`~/fire_pillar_staging_pose`，方案1.2节），返回`(x, y,
        yaw)`（局部坐标系，跟`sdk.goto()`的用法一致）。

        这个话题只有在`fire_pillar_aim_node`真正锁定高层火情、自动接管
        `pillar_aim`模式之后才会开始发布——调用这个方法前应该已经确认
        高层火情被锁定（比如RECON侧已经打断绕飞、准备执行D6子流程），
        不是在毫无征兆的时候随便调用。

        Raises:
            DetectionTimeoutError: 超过`timeout`秒仍未收到任何一帧
                `fire_pillar_staging_pose`消息（复用这个异常类型，跟
                "等待某个检测/状态出现"的语义一致，不需要单独定义新类型）。
        """
        ok = self._poll_until(
            lambda: self._fire_pillar_staging_pose_xyyaw is not None,
            timeout,
            lambda: self._progress('等待fire_pillar_staging_pose（任务机等待点）广播…'),
        )
        if not ok:
            raise DetectionTimeoutError(
                class_id='fire_pillar_staging_pose', timeout_s=timeout, namespace=self.namespace)
        self._progress(f'已读到fire_pillar_staging_pose={self._fire_pillar_staging_pose_xyyaw}')
        return self._fire_pillar_staging_pose_xyyaw

    def read_fire_pillar_aim_pose(self, timeout: float = 30.0) -> Tuple[float, float, float]:
        """读取`fire_pillar_aim_node`锁定高层火情后广播的瞄准点坐标
        （`~/fire_pillar_aim_pose`，方案1.2节/A1新增），返回`(x, y,
        yaw)`（局部坐标系）——内容跟`fire_pillar_aim_cmd`（驱动
        `position_cmd_relay`的`pillar_aim`模式那条控制指令）一致，但
        这是专门给第四部分程序读的只读广播，语义上是"状态"不是"指令"。

        Raises:
            DetectionTimeoutError: 超过`timeout`秒仍未收到任何一帧
                `fire_pillar_aim_pose`消息。
        """
        ok = self._poll_until(
            lambda: self._fire_pillar_aim_pose_xyyaw is not None,
            timeout,
            lambda: self._progress('等待fire_pillar_aim_pose（瞄准点）广播…'),
        )
        if not ok:
            raise DetectionTimeoutError(
                class_id='fire_pillar_aim_pose', timeout_s=timeout, namespace=self.namespace)
        self._progress(f'已读到fire_pillar_aim_pose={self._fire_pillar_aim_pose_xyyaw}')
        return self._fire_pillar_aim_pose_xyyaw

    def get_local_position(self, timeout: float = 10.0) -> Tuple[float, float, float]:
        """读取自己当前的局部坐标`(x, y, z)`——`_odom_xyz`这个状态本来
        就已经在`_setup_transport()`里常驻订阅`dlio/odom_node/odom`
        缓存着（原来只在B6的进度打印里用），2026-09-13实现阶段D的
        mission.py时发现"选手需要知道自己当前在哪"这个需求本身没有一个
        公开方法可以调用（比如判断"队友是否已经飞完一段航线、可以停止
        跟随了"这类场景），补一个薄封装直接暴露出去，不需要选手自己
        猜测/绕开SDK边界。

        Raises:
            DetectionTimeoutError: 超过`timeout`秒仍未收到过任何一帧
                里程计数据（正常情况下起飞后很快就会有数据，超时基本
                只会在刚构造完`DroneSDK`立刻调用、DDS discovery还没
                完成时发生）。
        """
        ok = self._poll_until(
            lambda: self._odom_xyz is not None,
            timeout,
            lambda: self._progress('等待里程计数据…'),
        )
        if not ok:
            raise DetectionTimeoutError(
                class_id='dlio/odom_node/odom', timeout_s=timeout, namespace=self.namespace)
        return self._odom_xyz

    def _settled_yaw(self, timeout: float = YAW_SETTLE_TIMEOUT_S) -> float:
        """等里程计 yaw 稳定下来，返回这批样本的圆周均值（弧度）。

        见 YAW_SETTLE_SAMPLES 上面的说明：不能用第一帧，那一帧可能还是
        定位源没收敛时的单位四元数。超时就退回当前瞬时值并给出提示——
        宁可用一个可能不准的值继续起飞，也不要因为 yaw 读不稳就不让起飞。
        """
        tol_rad = math.radians(YAW_SETTLE_TOLERANCE_DEG)
        deadline = time.monotonic() + timeout
        samples: List[float] = []
        while time.monotonic() < deadline:
            y = self._current_yaw
            if y is None:
                time.sleep(YAW_SETTLE_INTERVAL_S)
                continue
            samples.append(y)
            if len(samples) > YAW_SETTLE_SAMPLES:
                samples.pop(0)
            if len(samples) == YAW_SETTLE_SAMPLES:
                # 圆周均值，避免 ±180° 附近取平均直接算错
                mean = math.atan2(
                    sum(math.sin(v) for v in samples) / len(samples),
                    sum(math.cos(v) for v in samples) / len(samples),
                )
                spread = max(abs(math.atan2(math.sin(v - mean), math.cos(v - mean)))
                             for v in samples)
                if spread <= tol_rad:
                    # 稳定了还要跟 UWB 对得上——见 YAW_SOURCE_AGREE_DEG
                    uwb = self._uwb_yaw
                    if uwb is None:
                        return mean
                    diff = abs(math.atan2(math.sin(mean - uwb), math.cos(mean - uwb)))
                    if math.degrees(diff) <= YAW_SOURCE_AGREE_DEG:
                        return mean
                    self._progress(
                        f'里程计朝向{math.degrees(mean):.1f}°已稳定，但跟UWB绝对朝向'
                        f'{math.degrees(uwb):.1f}°差{math.degrees(diff):.1f}°'
                        f'（超过{YAW_SOURCE_AGREE_DEG:.0f}°）——定位源还没收敛，继续等'
                    )
                    samples.clear()
                self._progress(
                    f'等机头朝向稳定…最近{YAW_SETTLE_SAMPLES}次采样偏差'
                    f'{math.degrees(spread):.1f}°（要求小于{YAW_SETTLE_TOLERANCE_DEG:.0f}°）'
                )
            time.sleep(YAW_SETTLE_INTERVAL_S)
        # 超时兜底优先用 UWB 的绝对朝向：它不需要收敛过程，比"可能还停在单位
        # 四元数上的里程计瞬时值"更可信。两个都没有才用 0.0。
        if self._uwb_yaw is not None:
            self._progress(
                f'等机头朝向稳定超时（{timeout:.0f}秒），退回UWB绝对朝向'
                f'{math.degrees(self._uwb_yaw):.1f}°继续起飞'
            )
            return self._uwb_yaw
        fallback = self._current_yaw if self._current_yaw is not None else 0.0
        self._progress(
            f'等机头朝向稳定超时（{timeout:.0f}秒），且收不到UWB绝对位置，'
            f'退回里程计瞬时值{math.degrees(fallback):.1f}°继续起飞'
        )
        return fallback

    def get_agl(self, timeout: float = 5.0) -> float:
        """离地高度（米），来自机载朝下的定高雷达，不是里程计的 z。

        2026-09-24新增，给"仿地飞行"用：里程计的 z 是相对起飞点的**绝对高度**，
        地面本身有起伏（比赛场地里有个 0.25 米高的仿地模块台面）时，保持 z 不变
        等于离地高度在变；要贴着地形飞就得按这个读数调 z。

        真机和仿真是同一条话题（`mavros/hrlv_ez4_pub`，同一颗朝下的雷达），
        所以用它写的仿地逻辑不含仿真专有依赖。

        Raises:
            DetectionTimeoutError: 超过`timeout`秒没收到有效读数（贴地时读数会
                低于雷达最小量程被丢掉，刚起飞那一刻可能取不到）。
        """
        ok = self._poll_until(
            lambda: self._agl is not None,
            timeout,
            lambda: self._progress('等待定高雷达读数…'),
        )
        if not ok:
            raise DetectionTimeoutError(
                class_id='mavros/hrlv_ez4_pub', timeout_s=timeout, namespace=self.namespace)
        return self._agl

    def get_current_yaw(self, timeout: float = 10.0) -> float:
        """读取自己当前的实际yaw角（弧度，局部坐标系，跟`goto()`/
        `set_yaw_mode_constant()`同一套约定），从里程计四元数换算。

        2026-09-17新增，配合`takeoff()`自动读取"起飞前真实yaw角"这个
        安全修复（详见`takeoff()`docstring）——一般不需要选手自己调用，
        暴露出来是因为这个值本身也有查阅价值（比如任务代码想知道"现在
        机头朝哪"）。

        Raises:
            DetectionTimeoutError: 超过`timeout`秒仍未收到过任何一帧
                里程计数据。
        """
        ok = self._poll_until(
            lambda: self._current_yaw is not None,
            timeout,
            lambda: self._progress('等待里程计数据…'),
        )
        if not ok:
            raise DetectionTimeoutError(
                class_id='dlio/odom_node/odom', timeout_s=timeout, namespace=self.namespace)
        return self._current_yaw

    # ------------------------------------------------------------------
    # 生命周期收尾（不是2.1节编号能力，但跟_rclpy_runtime.py/reliability.py
    # 各自暴露的shutdown()对称，选手程序退出前调用一次做干净清理）。
    # ------------------------------------------------------------------

    # ==================================================================
    # 2026-10-01 新增：把选手程序里反复手写的常见模式收进来（**纯增量**）。
    #
    # 为什么加这一层：实测 groundfire.py 的 recon() 共 89 行，其中 47 行（56%）
    # 是在跟线程、检测轮询、刹停时序、异常兜底打交道，跟比赛题目无关，而且每个
    # 队都要重写一遍、各踩一遍坑。这些方法把那些机制收进 SDK，让选手程序只写
    # "做什么"。
    #
    # **不改任何现有方法**：全是新名字，老程序完全不受影响，新老两版共用同一个
    # 镜像。
    # ==================================================================

    def announce(self, event: str) -> None:
        """播报一次声光事件。`play_sound_light()` 的别名，名字更贴近用途。

        Args:
            event: 声光事件名。**必须是固定枚举里的一项**（见
                `_sound_light_port.py` 的 SOUND_LIGHT_EVENTS），自造名字会
                直接抛 ValueError 把整个任务打断。
        """
        self.play_sound_light(event)

    #: set_direct_speed() 用完恢复成这个值（= precision_servo 的出厂默认）。
    #: 0.3 是给"最后一段精修落点"调的，长距离直飞必须先提上来再用。
    DIRECT_SPEED_DEFAULT_MPS = 0.3
    #: 长距离直飞用的限速。2026-10-01 实测 8 m 航段：0.3->40.3 s、1.0->14.6 s，
    #: 再往上到 1.5 反而变慢（17.4 s，冲过头要回收），所以 1.0 就是拐点。
    DIRECT_CRUISE_MPS = 1.0

    def goto_world(self, wx: float, wy: float, agl_m: Optional[float] = None,
                   what: str = '', accept_m: float = 0.0,
                   direct: bool = False) -> None:
        """飞到一个**世界坐标**上方并锁高（走 ego_planner，有避障）。

        选手程序里 `world_to_local` + `fixed_altitude` + `goto` 这三件套出现了
        十几次，收成一个方法。agl_m 不给就用当前高度。

        Args:
            wx: 目标点的世界坐标 x（米）。
            wy: 目标点的世界坐标 y（米）。
            agl_m: 飞过去之后锁住的离地高度（米）。不给就用当前高度。
            what: 这个点叫什么，只用于日志（"飞往{what} (x, y)"）。
            accept_m: 判成"到不了"但其实已经在这个距离以内时按到达处理。
                默认 0 = 不容忍、照常抛异常。目标点贴着障碍或刚被别的飞机占过
                时规划器会把终端推到膨胀区边缘，给个 1~2 米的容忍更实用。
            direct: True = 走直线（`goto_direct`），**不经规划器、没有避障**。

                ⚠️ 只在**算过整条直线余量**的航段上开（用 `tools/check_route.py`）。
                算的时候布局里的 `obstacle_cylinder` / `pillars` /
                `terrain_module` / 四面墙一个都不能漏——2026-10-01 我只翻了
                `pillars` 就下结论，把 A(3,3)->B(3,22) 判成"干净"，而 φ0.5
                高 6 m 的 `obstacle_cylinder` 正坐在 (3.0, 8.0)，直飞就是直接
                撞上去。更要紧的是：**航线上的避障本身是考核点**，不能为了快
                把它绕过去。这个开关只用于编队解散之后各自回家那几段。
        """
        if agl_m is None:
            agl_m = self.get_agl()
        lx, ly, lz = self.world_to_local(wx, wy, agl_m)
        if what:
            self._progress(f'飞往{what} ({wx:.2f}, {wy:.2f})'
                           f'{"（直线，不避障）" if direct else ""}')
        if direct:
            self._direct_leg(lx, ly, lz)
            self._spot_clear_arrived(what or f'({wx:.1f}, {wy:.1f})')
            return
        try:
            with self.fixed_altitude(lz):
                self.goto(lx, ly, lz)
        except GotoUnreachableError as exc:
            d = float(getattr(exc, 'distance_m', 1e9))
            if accept_m <= 0.0 or d > accept_m:
                raise
            # 规划器把终点推到了膨胀区边缘（目标点上停着别的飞机、或贴着障碍）。
            # 调用方说了这点距离无所谓，就别让整个任务为它失败。
            self._progress(f'没能精确到点（还差 {d:.2f} m ≤ {accept_m:.1f} m），就地继续')
        self._spot_clear_arrived(what or f'({wx:.1f}, {wy:.1f})')

    def _direct_leg(self, lx: float, ly: float, lz: float) -> None:
        """直飞一段：先把限速提到巡航值，飞完恢复出厂默认。

        限速是 precision_servo **节点级**参数，不会自己复位；不恢复的话后面
        `return_home()` 的落点精修、`aim_at()` 的平移都会带着高限速跑，精度
        会变差（那两处本来就是要慢的）。
        """
        self.set_direct_speed(self.DIRECT_CRUISE_MPS)
        try:
            self.goto_direct(lx, ly, lz)
        finally:
            self.set_direct_speed(self.DIRECT_SPEED_DEFAULT_MPS)

    #: fly_route() 判成"到不了"但其实已经够近时的容忍（米）。
    #: 规划器轨迹终点速度为零、收敛是渐近的，最后半米要爬好几秒，而 goto() 的
    #: 卡住判据是"5 秒内位移不足 0.5 米且离目标还有 0.5 米以上"——爬行平台停在
    #: 0.5 米出头就正好落进判定区。那时候飞机其实已经到点了。
    FLY_ROUTE_ACCEPT_M = 0.7
    FLY_ROUTE_RETRY_S = 2.0

    #: 航点离飞机当前位置近到这个程度，就当成"已经在这儿了"直接跳过。
    #: 2026-10-01 从 0.05 提上来——0.05 m 比悬停漂移还小，这条分支等于永远
    #: 不成立。现场：综合任务第 2/3 轮都从 A 点出发，而 A 正是航线首点，飞机
    #: 已经在上面悬停了一分多钟。0.05 m 判不出重合，于是对着十几厘米的残差
    #: 向量算 atan2，得到的航向是纯噪声（实测 -9°），飞机先从上一段的 180°
    #: 猛转到 -9°（189°）、挪十几厘米、再转到 90° 去下一个点（99°）——将近
    #: 290° 的无谓旋转，看到的现象就是"偏航特别快、原地动了好几次"。
    #: 取 0.3（用户 2026-10-01 定）：比悬停漂移大、能挡住上面那种噪声航向，
    #: 又比 FLY_ROUTE_ACCEPT_M(0.7) 紧，半米出头的短航段仍然会老老实实飞。
    ROUTE_SKIP_M = 0.3

    #: 解散看门狗判"飞机已经过了某个航点"的半径。比 FLY_ROUTE_ACCEPT_M(0.7)
    #: 松一点：到点是 goto 判的，这里只是事后确认"确实到过"，宁可松不可紧——
    #: 判不出"过点"的后果是整个解散判据永远不成立。
    #: 三个阈值的关系：ROUTE_SKIP_M(0.3) < FLY_ROUTE_ACCEPT_M(0.7) < PASSED_M(1.2)。
    PASSED_M = 1.2

    def fly_route(self, waypoints: Sequence[Tuple[float, float]],
                  agl_m: float = 2.0, hold_s: float = 2.0,
                  names: Optional[List[str]] = None) -> None:
        """按航点序列飞：**每个航点先把机头转到下一段方向、停住，再走**，
        航段之间航向不变。

        Args:
            waypoints: [(wx, wy), ...] 世界坐标。
            agl_m: 全程锁的离地高度。
            hold_s: 每个航点停多久（转向跟停顿同时进行，不足的部分补足）。
            names: 航点名字，只用于日志；不给就按序号。

        自带两层兜底，都是实测踩出来的：
          · 零长度航段跳过——起点跟首航点重合时 atan2(0,0) 会给出 0°，飞机会
            朝正东白转一次；
          · 航点判成"不可达"但离得很近时按到达处理，还远就等 2 秒重发一次目标
            （规划器的安全检查会触发 EMERGENCY_STOP，几秒后自己恢复，而 goto()
            的卡住判据比它快半秒，会抢在恢复前把任务判死）。
        """
        cx, cy, _ = self.get_local_position()
        for i, (wx, wy) in enumerate(waypoints, start=1):
            nm = names[i - 1] if names and i <= len(names) else str(i)
            tx, ty, tz = self.world_to_local(wx, wy, agl_m)
            d0 = math.hypot(tx - cx, ty - cy)
            if d0 < self.ROUTE_SKIP_M:
                self._progress(f'航点{nm} ({wx:.1f}, {wy:.1f}) 已经在脚下'
                               f'（差 {d0:.2f} m < {self.ROUTE_SKIP_M:.1f} m），跳过')
                continue
            heading = math.atan2(ty - cy, tx - cx)
            self._progress(f'航点{nm} ({wx:.1f}, {wy:.1f})，'
                           f'航向 {math.degrees(heading):.0f}°（停 {hold_s:.0f} 秒转向）')
            t0 = time.time()
            self.face_yaw(heading)
            left = hold_s - (time.time() - t0)
            if left > 0:
                time.sleep(left)
            with self.fixed_altitude(tz):
                self._goto_with_retry(tx, ty, tz, f'航点{nm}')
            cx, cy = tx, ty

    def _goto_with_retry(self, x: float, y: float, z: float, tag: str) -> None:
        """goto() 外面包一层：够近就认、还远就重试一次。见 fly_route 的说明。"""
        for attempt in (1, 2):
            try:
                self.goto(x, y, z)
                self._spot_clear_arrived(tag)
                return
            except GotoUnreachableError as exc:
                d = float(getattr(exc, 'distance_m', 1e9))
                if d <= self.FLY_ROUTE_ACCEPT_M:
                    self._progress(f'{tag}：规划器终端爬行被判卡住，但只差 {d:.2f} m，'
                                   f'按到达处理')
                    self._spot_clear_arrived(tag)
                    return
                if attempt == 1:
                    self._progress(f'{tag}：还差 {d:.2f} m 就被判卡住'
                                   f'（规划器可能刚触发过安全急停），等 '
                                   f'{self.FLY_ROUTE_RETRY_S:.0f} 秒重试一次')
                    time.sleep(self.FLY_ROUTE_RETRY_S)
                    continue
                raise

    def hold_at(self, wx: float, wy: float, agl_m: float = 2.0,
                seconds: float = 0.0, direct: bool = False) -> None:
        """飞到某个世界坐标悬停待命，**不降落**。seconds>0 就停够这么久再返回。

        direct 的含义和前提同 `goto_world()`。

        Args:
            wx: 待命点的世界坐标 x（米）。
            wy: 待命点的世界坐标 y（米）。
            agl_m: 悬停的离地高度（米）。
            seconds: 到位后再停多久（秒）。0 = 到了就返回，不额外等。
            direct: True = 走直线（`goto_direct`），**不经规划器、没有避障**。
                含义和前提同 `goto_world()`——必须先算过这条直线的余量。
        """
        self.goto_world(wx, wy, agl_m, what=f'待命点 ({wx:.1f}, {wy:.1f})',
                        direct=direct)
        self._progress(f'已到 ({wx:.1f}, {wy:.1f})，悬停待命')
        if seconds > 0:
            time.sleep(seconds)

    #: return_home() 里离起降点超过这么远就先走规划器，最后一段才交给
    #: goto_direct 精修。goto_direct 走 precision_servo 的 coordinate_goto，
    #: 限速默认 0.3 m/s，是给"最后一段精修"调的，拿它跑长距离会慢到离谱
    #: （实测 5.4 米走了 45 秒）。
    RETURN_HOME_FAR_M = 2.0

    #: land() 等不到解锁确认时，自己核"是不是其实已经落地了"的判据。
    #: 判据不靠"多等几秒"，而是**看飞机自己到底落没落地**：贴地 + 静止。
    LANDED_AGL_M = 0.45      # 低于这个离地高度就算贴地（雷达最小量程约 0.3 m）
    LANDED_MOVE_M = 0.10     # 这段时间内位移小于这个值才算停住了
    LANDED_WATCH_S = 2.0

    def land_or_confirm(self) -> bool:
        """降落；`land()` 等不到解锁确认时，自己核一下是不是其实已经落地了。

        返回 True=已落地（正常解锁，或超时但确认贴地静止），False=确实还在空中。

        为什么需要：飞控偶尔就是不把 armed 置回 false，飞机明明已经稳稳坐在
        地上。2026-10-01 综合lite 实测侦察机最后一次降落：高度从 1.89 m 一路
        降到 0.00 m 并在那儿待了 25 秒，armed 始终 True，`land()` 30 秒超时
        抛 LandTimeoutError，把跑完三轮的整个任务在最后一步判死，队友接着
        因为它已经退出而 TeammateUnreachableError。
        老版 `utils.land_or_confirm()` 早就兜住了这一条，`return_home()` 当时
        直接调的 `land()`，漏了。任务流程里的降落都该走这个。
        """
        try:
            self.land()
            return True
        except LandTimeoutError as exc:
            self._progress(f'降落没等到解锁确认（{exc}），自己核一下是不是已经落地')
        x0, y0, z0 = self.get_local_position()
        time.sleep(self.LANDED_WATCH_S)
        x1, y1, z1 = self.get_local_position()
        moved = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
        try:
            agl = self.get_agl(timeout=2.0)
        except Exception:
            agl = z1          # 读不到雷达就拿里程计的 z 顶上
        if agl <= self.LANDED_AGL_M and moved <= self.LANDED_MOVE_M:
            self._progress(f'已确认落地：离地 {agl:.2f} m、{self.LANDED_WATCH_S:.0f} 秒内'
                           f'只动了 {moved:.2f} m（飞控没解锁，但飞机确实在地上），继续')
            return True
        self._progress(f'没能确认落地：离地 {agl:.2f} m、位移 {moved:.2f} m，还在空中')
        return False

    #: return_home(report=/sound=) 核对落点时的容差：离标称起降点这么近才算
    #: "落在自己的起降点上"。
    PAD_TOL_M = 1.0

    def return_home(self, land: bool = True, agl_m: float = 2.0,
                    sound: Optional[str] = None,
                    report: Optional[str] = None,
                    direct: bool = False) -> bool:
        """回自己的起飞点，默认降落。返回**是否真的落在自己的起降点上**。

        分两段：远距离用 goto()（走规划器、有避障、巡航速度）飞到起降点上空，
        最后一段用 goto_direct（直线、不经规划器，落点精度高一个量级）精修。

        Args:
            land: True = 到位后降落（默认）；False = 只飞回起飞点上空悬停。
            agl_m: 飞回去时锁的离地高度（米）。
            sound: 核对落点通过后才播的声光事件。**不要在调用方自己播**——
                任务机全程要降落三次（取器材、放器材、回家），"降落动作完成"
                本身说明不了任务结束，得按落点坐标判。
            report: 给队友发的事件名，带 x/y/ok 三个字段。落点不准也发，
                否则队友会一直等到超时。
            direct: True = 整段走直线，不经规划器。含义和前提同 `goto_world()`
                ——**必须先算过这条直线的余量**。编队解散之后各自回家那一段
                适合开（算过：D->各自起降点最小余量 3.00 m，离墙最近）。
        """
        hx, hy, _ = self.local_to_world(0.0, 0.0, 0.0)
        lx, ly, lz = self.world_to_local(hx, hy, agl_m)
        try:
            cx, cy, _ = self.get_local_position()
            far = math.hypot(cx - lx, cy - ly)
        except Exception:
            far = 0.0
        if far > self.RETURN_HOME_FAR_M and not direct:
            self._progress(f'回起飞点 ({hx:.2f}, {hy:.2f})，还有 {far:.1f} m，'
                           f'先走规划器飞过去')
            try:
                with self.fixed_altitude(lz):
                    self.goto(lx, ly, lz)
            except Exception as exc:
                self._progress(f'规划器飞不过去（{exc}），改用直线飞')
        elif direct and far > self.RETURN_HOME_FAR_M:
            self._progress(f'回起飞点 ({hx:.2f}, {hy:.2f})，还有 {far:.1f} m，'
                           f'直线飞过去（不避障）')
            self._direct_leg(lx, ly, lz)
        self.goto_direct(lx, ly, lz)         # 最后一段精修，用默认 0.3 收准
        if land:
            self.land_or_confirm()
        px, py, _ = self.get_local_position()
        wx, wy = self.local_to_world(px, py, 0.0)[:2]
        d = math.hypot(wx - hx, wy - hy)
        ok = bool(d <= self.PAD_TOL_M)
        if ok:
            self._progress(f'已降落在自己起降点 ({wx:.2f}, {wy:.2f})，离标称点 {d:.2f} m')
            if sound:
                self.play_sound_light(sound)
        else:
            self._progress(f'⚠️ 落点 ({wx:.2f}, {wy:.2f}) 离自己起降点 {d:.2f} m '
                           f'(>{self.PAD_TOL_M:.1f} m)，不算落在起降点上')
        if report:
            self.send_to_teammate(report, x=float(wx), y=float(wy), ok=ok)
        return ok

    # ---- 跨机协同：收件箱 ----
    # 可靠事件通道有两个坑，这里一次性兜住：
    #   ① 先回 ACK 再查处理函数，**没注册的事件会被确认后丢弃**——所以必须在
    #      对方可能发之前就把所有事件注册好，不能等用到了才注册；
    #   ② 一个事件名只能挂一个处理函数（register_event_handler 是直接赋值），
    #      重复注册会把前一个顶掉。
    # 选手自己写 _Inbox 的话这两条都要自己记，实测我们自己就在②上栽过一次。

    def open_inbox(self, *events: str) -> None:
        """注册这些跨机事件的收件箱。**在任务一开始就全部注册**，见上面说明。

        重复注册同一个事件是安全的（幂等），不会把已收到的内容清掉。

        Args:
            *events: 要注册的事件名，可以一次给多个。
                重复注册同一个事件是安全的（幂等），不会把已收到的内容清掉。
        """
        if not hasattr(self, '_inbox'):
            self._inbox: Dict[str, Dict[str, Any]] = {}
            self._inbox_seen: set = set()
        for ev in events:
            if ev in self._inbox:
                continue
            self._inbox[ev] = {}

            def _on(_ev: str = ev, **kw: Any) -> None:
                self._inbox[_ev] = kw
                self._inbox_seen.add(_ev)

            self.on_teammate_event(ev, _on)
            self._progress(f"收件箱已注册：'{ev}'")

    @overload
    def wait_event(self, event: str, timeout_s: float = ..., clear: bool = ...,
                   required: 'Literal[True]' = ...) -> Dict[str, Any]: ...

    @overload
    def wait_event(self, event: str, timeout_s: float = ..., clear: bool = ...,
                   *, required: 'Literal[False]') -> Optional[Dict[str, Any]]: ...

    def wait_event(self, event: str, timeout_s: float = 300.0,
                   clear: bool = True, required: bool = True
                   ) -> Optional[Dict[str, Any]]:
        """等一个跨机事件，**返回它带来的数据**（dict，没有数据就是空 dict）。

        Args:
            event: 事件名，必须先 `open_inbox()` 注册过。
            timeout_s: 等多久。
            clear: 取走后把收件箱复位（默认 True）。多轮流程里同一个事件会来
                好几次，不复位的话第二轮一进来就立刻返回上一轮的旧数据。
            required: False = 超时就**返回 None**，不抛异常。用在"等到更好、
                等不到也得往下走"的地方——比如最后等队友报告已降落，等不到
                也该把任务完成播出去，不能让整个任务在这一步失败。

        Raises:
            TimeoutError: 超时还没等到（required=True 时）。
            RuntimeError: 这个事件没注册过收件箱（多半是忘了 open_inbox）。
        """
        if not hasattr(self, '_inbox') or event not in self._inbox:
            raise RuntimeError(f"事件 '{event}' 没有注册收件箱，"
                               f"任务开始时要先 sdk.open_inbox('{event}')")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if event in self._inbox_seen:
                data = dict(self._inbox[event])
                if clear:
                    self._inbox_seen.discard(event)
                    self._inbox[event] = {}
                return data
            time.sleep(0.1)
        if not required:
            self._progress(f"等队友事件 '{event}' 超过 {timeout_s:.0f} 秒未到达，继续往下走")
            return None
        raise TimeoutError(f"等队友事件 '{event}' 超过 {timeout_s:.0f} 秒未到达")

    def event_ready(self, event: str) -> bool:
        """这个事件到了没有（不阻塞、不取走）。等多个事件里先到的那个时用。"""
        return hasattr(self, '_inbox') and event in self._inbox_seen

    def wait_any_event(self, events: Sequence[str], timeout_s: float = 300.0
                       ) -> Tuple[str, Dict[str, Any]]:
        """等这几个事件里**先到的那一个**，返回 (事件名, 数据)。

        Args:
            events: 要等的事件名列表。**按列表顺序检查**，所以同一轮里
                有多个都已到达时，返回列表中靠前的那个。
            timeout_s: 等多久（秒）。超时抛 TimeoutError。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for ev in events:
                if self.event_ready(ev):
                    return ev, self.wait_event(ev, timeout_s=1.0)
            time.sleep(0.1)
        raise TimeoutError(f'等事件 {events} 超过 {timeout_s:.0f} 秒')

    # ---- 编队 ----
    #: 编队握手用的事件名。**跟老版 formation.py 用的是同一组字符串**，所以
    #: 新老两版可以混搭（lite 长机配老僚机也能飞），不是另起一套协议。
    EV_READY = 'formation_standby'          # 僚机 -> 长机：我就位了
    EV_ROUTE_PLAN = 'route_plan'            # 长机 -> 僚机：整条航线
    EV_IN_POSITION = 'formation_in_position'  # 僚机 -> 长机：已到站位、机头已转好
    EV_ROUTE_DONE = 'route_done'            # 长机 -> 僚机：解散

    #: 每段起步先把规划器限速压到这里，等僚机也过了这个航点再分档放回巡航。
    #: 长机在每个航点都停下转向，对它自己来说**过每个航点都相当于重新起步**；
    #: 而僚机沿长机轨迹落后一个间距，长机站在拐点上时它还在上一段没拐弯。这时
    #: 长机一脚油门顶到巡航速度，间距就一次性放大（实测峰值 7.02 m）。
    #: 下限不能低于 0.5：goto() 的卡住检测要求持续跑到 0.1 m/s 以上，而限速
    #: 0.25 时实际只跑 0.15 m/s，会被判成不可达。
    LEG_SLOW_MPS = 0.5
    LEG_SLOW_MIN_LEG_M = 8.0      # 短于这个的航段不压限速——没有把速度还回去的余量
    LEG_RAMP_MUST_START_M = 6.0   # 剩这么远还没等到僚机过点，就必须开始放速度
    LEG_RAMP_STEP_MPS = 0.25
    LEG_RAMP_PERIOD_S = 1.0

    def lead_formation(self, route: Sequence[Tuple[float, float]], spacing_m: float = 4.0,
                       agl_m: float = 2.0, hold_s: float = 2.0,
                       start_xy: Optional[Tuple[float, float]] = None,
                       final_xy: Optional[Tuple[float, float]] = None,
                       disband_at: Optional[Tuple[float, float]] = None,
                       wait_follower_s: float = 300.0,
                       tail_direct: bool = False) -> None:
        """长机带队飞一条航线。**只管空中段**，起降由调用方自己决定。

        Args:
            route: 航线（世界坐标）。
            spacing_m: 目标纵向间距。
            agl_m: 全程锁的离地高度（米）。
            hold_s: 每个航点停多久（秒）。转向跟停顿同时进行，不足的部分补足。
            start_xy: 从哪儿起步（默认自己当前位置换算成的起飞点）。编队段从
                半路开始时要给，否则僚机算出来的起始站位会跑到场外。
            final_xy: 航线跑完再飞到哪儿（默认 route[0]）。
            disband_at: 给了就按"**僚机过了这个航点**"解散；不给按"长机飞回
                自己起飞点上空"解散。
            wait_follower_s: 等僚机到站位的上限，等不到也照飞。
            tail_direct: 最后一段（飞往 final_xy 那一段）走直线。
                **解散就发生在这一段上**——disband_at 的判据是"长机已经离开
                那个航点 spacing+lag 米"，所以长机走到这一段中途才解散，剩下
                的路本质上是"解散后各自回家"。前提同 `goto_world()` 的
                direct：必须算过这条直线的余量。
        """
        import threading
        self.open_inbox(self.EV_READY, self.EV_IN_POSITION)
        hx, hy, _ = self.local_to_world(0.0, 0.0, 0.0)
        start = tuple(start_xy) if start_xy is not None else (hx, hy)
        tail = tuple(final_xy) if final_xy is not None else tuple(route[0])
        wps = list(route)
        if tuple(wps[-1]) != tail:
            wps.append(tail)
        plan = list(wps) if math.hypot(start[0] - wps[0][0], start[1] - wps[0][1]) <= 0.3 \
            else [start] + list(wps)
        # 航线**立刻发**，不等僚机报到。收件箱在 DroneSDK 构造时就建好了
        # （见那里的说明），不存在"发早了被丢"的问题；反过来等 READY 会在僚机
        # 起飞慢时白等一整个超时，实测两边互等各 60 秒。
        try:
            self.send_to_teammate(self.EV_ROUTE_PLAN, route=[list(p) for p in plan])
        except Exception as exc:
            self._progress(f'航线没送到僚机（{exc}），僚机将保持入列时的朝向')
        self._progress('等僚机到起始站位并转向…')
        try:
            self.wait_event(self.EV_IN_POSITION, wait_follower_s)
            self._progress('僚机已到位，起步')
        except TimeoutError:
            self._progress(f'等了 {wait_follower_s:.0f} 秒没等到僚机到位，直接起步')

        state = {'stop': False, 'sent': False}
        stop_watch = self._start_disband_watch(state, spacing_m, disband_at)
        cruise = None
        legs = list(zip([start] + wps[:-1], wps))
        try:
            for i, (frm, to) in enumerate(legs, start=1):
                fx, fy, _ = self.world_to_local(frm[0], frm[1], agl_m)
                tx, ty, tz = self.world_to_local(to[0], to[1], agl_m)
                leg_len = math.hypot(tx - fx, ty - fy)
                if leg_len < self.ROUTE_SKIP_M:
                    self._progress(f'航点 {i}/{len(legs)} 已经在脚下'
                                   f'（差 {leg_len:.2f} m < {self.ROUTE_SKIP_M:.1f} m），跳过')
                    continue
                heading = math.atan2(ty - fy, tx - fx)
                self._progress(f'航点 {i}/{len(legs)}: ({to[0]}, {to[1]})，'
                               f'航向 {math.degrees(heading):.0f}°（停 {hold_s:.0f} 秒转向）')
                t0 = time.time()
                self.face_yaw(heading)
                left = hold_s - (time.time() - t0)
                if left > 0:
                    time.sleep(left)
                if not state['sent']:
                    self._wait_follower_settled(f'航点 {i}/{len(legs)}')
                if tail_direct and i == len(legs):
                    # 尾段直飞：解散就在这一段中途发生，后半程本就是各自回家。
                    # 不压起步限速——那套压的是规划器的 max_vel，对直飞无效。
                    self._direct_leg(tx, ty, tz)
                    continue
                if leg_len >= self.LEG_SLOW_MIN_LEG_M:
                    cruise = self._slow_leg_start((fx, fy), (tx, ty), spacing_m,
                                                  f'航点 {i}/{len(legs)}', cruise)
                with self.fixed_altitude(tz):
                    self._goto_with_retry(tx, ty, tz, f'航点 {i}/{len(legs)}')
        finally:
            stop_watch()
            if cruise is not None:
                try:
                    self.set_max_vel(cruise)
                except Exception:
                    pass
            if not state['sent']:
                try:
                    self.send_to_teammate(self.EV_ROUTE_DONE)
                    self._progress('航线已飞完，补发编队解散通知')
                except Exception:
                    pass

    def follow_formation(self, spacing_m: float = 4.0, agl_m: float = 2.0,
                         join: str = 'station', route_wait_s: float = 60.0,
                         done_wait_s: float = 600.0) -> None:
        """僚机跟队。**只管空中段**，起降由调用方自己决定。

        Args:
            spacing_m: 跟在长机后方多少米。要跟长机那边给的一致。
            agl_m: 入列和跟队时的离地高度（米）。
            join: `'station'` 先飞到"航线起点后方 spacing 米"的站位点再入列
                （编队从头开始时用）；`'nearest'` 就地入列（任务流程里僚机刚
                做完事就在长机附近，再飞一趟站位点纯属绕路）。
            route_wait_s: 等长机下发航线的上限（秒）。等不到就只跟队、不做
                分段航向，机头全程不变。
            done_wait_s: 等长机发"解散"的上限（秒）。等不到也会自己出列，
                免得长机那边出问题时僚机永远挂着。
        """
        self.open_inbox(self.EV_ROUTE_PLAN, self.EV_ROUTE_DONE)
        self.send_to_teammate(self.EV_READY)
        route: List[Tuple[float, float]] = []
        try:
            d = self.wait_event(self.EV_ROUTE_PLAN, route_wait_s)
            route = [tuple(p) for p in d.get('route', [])]
        except TimeoutError:
            self._progress('没收到长机航线，跳过预站位')

        if join == 'station' and len(route) >= 2:
            (lx0, ly0), (nx0, ny0) = route[0], route[1]
            heading = math.atan2(ny0 - ly0, nx0 - lx0)
            sx = lx0 - math.cos(heading) * spacing_m
            sy = ly0 - math.sin(heading) * spacing_m
            self._progress(f'起始站位 ({sx:.2f}, {sy:.2f})，在长机起飞点后方 '
                           f'{spacing_m:.1f} 米')
            try:
                self.goto_world(sx, sy, agl_m)
                self.face_yaw(heading)
            except Exception as exc:
                self._progress(f'站位点飞不过去（{exc}），就当前位置入列')
        else:
            self._progress('就地入列（跳过预站位）')

        self.start_formation_follow(follow_distance_m=spacing_m, altitude_agl_m=agl_m,
                                    turn_in_place=True, leg_route=route or None)
        self.send_to_teammate(self.EV_IN_POSITION)
        self.wait_event(self.EV_ROUTE_DONE, done_wait_s)
        self.stop_formation_follow()

    # ---- lead_formation 的三个内部件 ----

    def _wait_follower_settled(self, tag: str, tol_m: float = 0.5,
                               max_wait_s: float = 8.0) -> float:
        """长机在航点上等僚机收拢到队形位置。返回实际等了多久。

        判据是僚机自报的落后量（`teammate_formation_lag()`）。只判"不落后太多"、
        不判 |lag|——落后能靠僚机追上来消掉，太近却没法后退（参考点只前进不后退）。
        读不到就不等，退回没有握手的行为。
        """
        if self.teammate_formation_lag() is None:
            return 0.0
        t0 = time.time()
        while time.time() - t0 < max_wait_s:
            lag = self.teammate_formation_lag()
            if lag is None:
                break
            if lag <= tol_m:
                waited = time.time() - t0
                self._progress(f'{tag}：僚机已入位（落后 {lag:+.2f} m），'
                               f'等了 {waited:.1f} 秒后起步')
                return waited
            time.sleep(0.1)
        self._progress(f'{tag}：等了 {time.time() - t0:.1f} 秒僚机仍落后，按超时起步')
        return time.time() - t0

    def _slow_leg_start(self, corner: Tuple[float, float], end: Tuple[float, float],
                        spacing_m: float, tag: str, cruise: Optional[float]) -> float:
        """每段起步压限速，等**僚机也过了这个航点**再分档放回巡航。返回巡航限速值。

        "僚机已过点"的判据：僚机沿轨迹落在长机后方 `spacing + lag` 米，而本段
        是直线，所以"长机离拐点的直线距离 ≥ spacing + lag"就等价于它已走过拐点。
        长机自己就能算，不需要僚机额外上报。

        限速只在飞机**停着的时候**压下去；放速度那一下也要赶在终端减速段之前
        （LEG_RAMP_MUST_START_M），因为改限速会逼规划器重做时间分配，在减速段
        重算刹车剖面会冲过头。
        """
        import threading
        if not hasattr(self, 'set_max_vel'):
            return cruise or 1.0
        try:
            old = self.set_max_vel(self.LEG_SLOW_MPS)
        except Exception as exc:
            self._progress(f'{tag}：起步限速没设上（{exc}），按原速起步')
            return cruise or 1.0
        cruise = cruise if cruise is not None else old
        cx0, cy0 = corner
        ex, ey = end
        self._progress(f'{tag}：起步限速 {self.LEG_SLOW_MPS} m/s，'
                       f'等僚机也过这个航点再分档放回 {cruise} m/s')

        def _restore() -> None:
            t0 = time.time()
            time.sleep(1.0)
            why = '压满下限'
            while time.time() - t0 < 30.0:
                try:
                    px, py, _ = self.get_local_position()
                except Exception:
                    break
                remain = math.hypot(px - ex, py - ey)
                if remain <= self.LEG_RAMP_MUST_START_M:
                    why = f'剩余 {remain:.2f} m 已到必须放速度的距离'
                    break
                lag = self.teammate_formation_lag()
                need = spacing_m + 0.5 + max(0.0, lag or 0.0)
                gone = math.hypot(px - cx0, py - cy0)
                if gone >= need:
                    why = f'僚机已过点（长机离拐点 {gone:.2f} m ≥ {need:.2f} m）'
                    break
                time.sleep(0.1)
            self._progress(f'{tag}：{why}，开始分档放速度')
            v = self.LEG_SLOW_MPS
            try:
                while v < cruise - 1e-3:
                    v = min(cruise, v + self.LEG_RAMP_STEP_MPS)
                    self.set_max_vel(v)
                    if v < cruise - 1e-3:
                        time.sleep(self.LEG_RAMP_PERIOD_S)
            except Exception as exc:
                self._progress(f'{tag}：限速没恢复（{exc}）')

        threading.Thread(target=_restore, daemon=True).start()
        return cruise

    def _start_disband_watch(self, state: Dict[str, Any], spacing_m: float,
                             disband_at: Optional[Tuple[float, float]]):
        """后台盯着什么时候该解散编队，到了就发 EV_ROUTE_DONE。返回停止函数。

        两种判据：
          · disband_at 给了：**僚机过了那个航点**就解散。长机拿不到僚机位置，但
            拿得到它自报的落后量——僚机落在长机后方 spacing+lag 米，本段是直线，
            所以"长机离该点 ≥ spacing+lag"等价于僚机已过点。要先判长机自己过点，
            不然刚起步时离得也很远，会当场误触发。
          · 不给：长机飞回**自己起飞点**上空就解散。要先判"已经出发过"，不然
            起飞那一刻就在起飞点上，立刻误触发。
        """
        import threading
        hx, hy, _ = self.local_to_world(0.0, 0.0, 0.0)
        tgt = tuple(disband_at) if disband_at is not None else (hx, hy)
        lx, ly, _lz = self.world_to_local(tgt[0], tgt[1], 2.0)
        by_follower = disband_at is not None

        def _loop() -> None:
            passed = False
            departed = False
            while not state['stop'] and not state['sent']:
                try:
                    cx, cy, _ = self.get_local_position()
                except Exception:
                    time.sleep(0.2)
                    continue
                d = math.hypot(cx - lx, cy - ly)
                if by_follower:
                    if not passed:
                        if d <= self.PASSED_M:
                            passed = True
                            self._progress(f'已过航点 ({tgt[0]:.1f}, {tgt[1]:.1f})，'
                                           f'开始等僚机过点')
                        time.sleep(0.2)
                        continue
                    lag = self.teammate_formation_lag()
                    if lag is None:
                        # 读不到就按"没有额外落后"算，但**必须说出来**：原来这里
                        # 是 `lag or 0.0` 静默退化，日志上分不出"量到 0"和"根本
                        # 没量到"，而后者会让解散早触发（僚机其实还没到点）。
                        if not state.get('warned_lag'):
                            state['warned_lag'] = True
                            self._progress('⚠️ 读不到僚机自报落后量，解散判据按 '
                                           f'{spacing_m:.1f} m 算，可能偏早')
                    need = spacing_m + max(0.0, lag or 0.0)
                    if d >= need:
                        if self._fire_disband(
                                state, f'僚机已过航点 ({tgt[0]:.1f}, {tgt[1]:.1f})'):
                            return
                else:
                    if not departed:
                        if d >= 5.0:
                            departed = True
                    elif d <= self.PASSED_M:
                        if self._fire_disband(
                                state, f'已飞回自己起飞点上空（离 {d:.1f} m）'):
                            return
                time.sleep(0.2)

        t = threading.Thread(target=_loop, daemon=True)
        t.start()

        def _stop() -> None:
            state['stop'] = True
            t.join(timeout=2.0)
        return _stop

    def _fire_disband(self, state: Dict[str, Any], why: str) -> bool:
        """发解散通知。**发成功才置 sent**，返回是否送达。

        原来是先置位再发：`send_to_teammate` 是带 ACK 的可靠通道，收不到确认
        会抛异常（45 秒超时），一旦抛了 sent 已经是 True，于是
        `lead_formation` 的 `finally` 里那句 `if not state['sent']: 补发`
        判不成立，**兜底被自己堵死**，僚机只能干等满 done_wait_s（600 秒）。
        而日志上还照打"编队解散"，排查时会被带偏。
        """
        try:
            self.send_to_teammate(self.EV_ROUTE_DONE)
        except Exception as exc:
            self._progress(f'⚠️ 解散通知没送到僚机（{exc}），稍后重试')
            return False
        state['sent'] = True
        self._progress(f'{why}，编队解散')
        return True

    # ---- 抓放与精准降落 ----
    #: 机械抓的 PWM。真机实测值：800=抓紧、2000=松开。
    GRIP_CLOSE_PWM = 800
    GRIP_OPEN_PWM = 2000
    GRIP_TRAVEL_S = 2.0           # 舵机没有位置反馈，只能等
    #: land_on() 的分步下降参数
    LAND_ON_TIMEOUT_S = 30.0      # 边瞄边降这一段的总时限，到点就交给普通降落
    LAND_ON_STEP_M = 0.8          # 每步降多少：降一点就重新解算一次目标位置
    LAND_ON_HANDOFF_M = 0.7       # 降到离地这么高就交给普通降落（再低下视看不全标志）
    LAND_ON_HANDOFF_TOL_M = 0.15  # 悬停本身有起伏，死等"严格低于"会一直卡在上面

    def grip(self, release: bool = False, label: str = '') -> None:
        """驱动机械抓。release=False 抓紧，True 松开。

        仿真里飞控不一定配了舵机输出，动不了就打印、继续飞完流程，不中断任务。

        Args:
            release: False = 抓紧（PWM 800），True = 松开（PWM 2000）。
                真机实测值，见 GRIP_CLOSE_PWM / GRIP_OPEN_PWM。
            label: 日志里怎么称呼这个动作（比如 '抓取灭火弹'）。
                不给就按 release 自动用"抓取"/"松开"。
        """
        pwm = self.GRIP_OPEN_PWM if release else self.GRIP_CLOSE_PWM
        what = label or ('松开' if release else '抓取')
        try:
            self.set_servos({s: pwm for s in sorted(self.servos)})
        except Exception as exc:
            self._progress(f'{what}：舵机没动（{exc}）——真机需要飞控配好 MAIN7/MAIN9')
            return
        time.sleep(self.GRIP_TRAVEL_S)
        self._progress(f'{what}完成')

    def land_on(self, class_id: str, what: str = '目标') -> None:
        """边瞄准边降落到底：每降一小段就把目标位置重新解算一次，直接命令
        "目标正上方、低一点"那个位置——水平修正和下降在同一条指令里完成；
        降到交接高度后交给普通降落收尾。

        为什么不用 `precision_land_and_confirm()`：那是"对准一点、下降一点、
        再对准"的分级下降，从 2.5 米下来要 40 秒以上，30 秒时限内根本走不完，
        每次都走超时兜底（2026-09-23 实测），等于精度只做了一半。这里是连续
        修正，同一时间既对准也下降；最后 0.7 米交给 land()，那一段本来就只能
        垂直下降，再修也无意义。
        """
        deadline = time.monotonic() + self.LAND_ON_TIMEOUT_S
        while time.monotonic() < deadline:
            _, _, z = self.get_local_position()
            try:
                t = self.locate_target(class_id, timeout=2.0, samples=3)
            except DetectionTimeoutError:
                self._progress(f'下降中看不到{what}了，就地转普通降落')
                break
            agl = z - t.z
            if agl <= self.LAND_ON_HANDOFF_M + self.LAND_ON_HANDOFF_TOL_M:
                self._progress(f'已降到离地 {agl:.2f} m，交给普通降落')
                break
            nxt = max(self.LAND_ON_HANDOFF_M, agl - self.LAND_ON_STEP_M)
            self._progress(f'对准{what} ({t.x:.2f}, {t.y:.2f}) 并降到离地 {nxt:.2f} m'
                           f'（当前 {agl:.2f} m）')
            self.goto_direct(t.x, t.y, t.z + nxt)
        else:
            self._progress(f'边瞄准边降落用满 {self.LAND_ON_TIMEOUT_S:.0f} 秒，转普通降落')
        self.land_or_confirm()

    def fetch_from(self, wxy: Tuple[float, float], class_id: str,
                   what: str = '物资', agl_m: float = 2.0,
                   sound: Optional[str] = None, direct: bool = False) -> None:
        """飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。

        Args:
            wxy: 物资点的世界坐标 (x, y)。
            class_id: 物资点上贴的标识，比如 'apriltag:0'，用来边瞄边降。
            what: 物资叫什么，进日志也进降落提示（"已降落在{what}点"）。
            agl_m: 飞过去时的巡航高度，抓完也回到这个高度（米）。
            sound: 声光播报内容。**播报点在落地之后、舵机动作之前**——裁判
                听到的那一声要对上"正在抓"这个瞬间；写在 `fetch_from()`
                前面会提前十几秒（还在飞往物资点的路上就播了）。
            direct: True = 飞过去那一段走直线，不经规划器。含义和前提同
                `goto_world()`。
        """
        self.goto_world(wxy[0], wxy[1], agl_m, what=what + '点', direct=direct)
        self.land_on(class_id, what)
        self._progress(f'已降落在{what}点，开始抓取')
        if sound:
            self.play_sound_light(sound)
        self.grip(release=False, label='抓取' + what)
        self.takeoff(height_m=agl_m)

    def release_at(self, wxy: Tuple[float, float], class_id: str,
                   what: str = '物资', agl_m: float = 2.0,
                   sound: Optional[str] = None, direct: bool = False) -> None:
        """飞到物资点 -> 边瞄边降到底 -> 松开 -> 起飞回巡航高度。

        Args:
            wxy: 物资点的世界坐标 (x, y)。
            class_id: 物资点上贴的标识，用来边瞄边降。
            what: 物资叫什么，进日志。
            agl_m: 飞过去时的巡航高度，放完也回到这个高度（米）。
            sound: 同 `fetch_from()`，落地后、舵机动作前播报。
            direct: True = 飞过去那一段走直线。含义和前提同 `goto_world()`。
        """
        self.goto_world(wxy[0], wxy[1], agl_m, what=what + '点', direct=direct)
        self.land_on(class_id, what)
        self._progress(f'已降落在{what}点，开始释放')
        if sound:
            self.play_sound_light(sound)
        self.grip(release=True, label='释放' + what)
        self.takeoff(height_m=agl_m)

    # ---- 视觉搜索与对准 ----
    #: 相机内参。640x480、HFOV 80°，跟 camera_info 一致。
    IMAGE_W, IMAGE_H = 640, 480
    FOCAL_PX = 381.35             # = (IMAGE_W/2)/tan(HFOV/2)
    TAG_SIZE_M = 0.5              # AprilTag 实际边长，用来按框宽估距离
    AIM_TOL_M = 0.12              # 横向/垂直差这么多以内算对准
    AIM_MAX_TRIES = 5
    AIM_SETTLE_S = 1.5
    AIM_Z_MIN_M, AIM_Z_MAX_M = 0.8, 4.0

    def aim_at(self, class_id: str, camera: str = 'front',
               face_yaw_deg: Optional[float] = None, what: str = '目标') -> bool:
        """把目标**挪到画面正中**：横向平移 + 升降，机头朝向全程不变。返回是否对上。

        Args:
            class_id: 要对准的目标类别，比如 'apriltag:1'。
            camera: 'front'（前视，贴在立面上的目标）或 'down'（下视，地面目标）。
                **两条路完全不同**：前视走本方法自己的几何解算；下视自动改走
                `center_on_target()` 的 precision_servo 闭环。
            face_yaw_deg: 给了就先把机头转到这个朝向再对准。**正对立面的场景
                一定要给**——本方法是"锁住当前朝向只做平移"的，朝向不对的话
                画面里居中了、机身却斜着，弹丸打出去也是斜的。
            what: 目标叫什么，只用于日志。
        """
        if face_yaw_deg is not None:
            self.face_yaw(math.radians(face_yaw_deg))
        if camera == 'down':
            try:
                self.center_on_target(class_id, timeout=40.0)
            except Exception as exc:
                self._progress(f'没对上{what}（{exc}），按当前位置继续')
                return False
            self._progress(f'{what}已对准（下视，走 precision_servo 闭环）')
            return True
        yaw0 = self.get_current_yaw()
        for i in range(1, self.AIM_MAX_TRIES + 1):
            try:
                det = self.wait_for_detection(class_id, timeout=4.0, camera=camera)
            except Exception:
                self._progress(f'第{i}轮对准：{camera}相机看不到{what}')
                time.sleep(self.AIM_SETTLE_S)
                continue
            dx = math.atan2(det.bbox_x - self.IMAGE_W / 2.0, self.FOCAL_PX)
            dy = math.atan2(det.bbox_y - self.IMAGE_H / 2.0, self.FOCAL_PX)
            dist = self.FOCAL_PX * self.TAG_SIZE_M / max(1.0, det.bbox_width)
            lat = dist * math.tan(dx)       # >0：目标在飞机右侧
            dz = -dist * math.tan(dy)       # 目标在画面下方 -> 要降高度
            self._progress(f'第{i}轮对准：{what}在画面 ({det.bbox_x:.0f},{det.bbox_y:.0f})，'
                           f'框宽 {det.bbox_width:.0f}px -> 距离约 {dist:.2f} m，'
                           f'横向 {lat:+.2f} m，高度 {dz:+.2f} m')
            if abs(lat) <= self.AIM_TOL_M and abs(dz) <= self.AIM_TOL_M:
                self._progress(f'{what}已对准（横向 {lat:+.2f} m，垂直 {dz:+.2f} m）')
                return True
            cx, cy, cz = self.get_local_position()
            rx, ry = math.sin(yaw0), -math.cos(yaw0)      # 机头右手方向
            _, _, z_lo = self.world_to_local(0.0, 0.0, self.AIM_Z_MIN_M)
            _, _, z_hi = self.world_to_local(0.0, 0.0, self.AIM_Z_MAX_M)
            self.goto_direct(cx + rx * lat, cy + ry * lat,
                             min(max(cz + dz, z_lo), z_hi))
            self.face_yaw(yaw0)             # 平移可能带偏朝向，复位
            time.sleep(self.AIM_SETTLE_S)
        self._progress(f'{self.AIM_MAX_TRIES} 轮仍没对准{what}，按当前状态继续')
        return False

    def search_along(self, leg_end: Tuple[float, float], class_id: str,
                     camera: str = 'down', agl_m: float = 2.0,
                     timeout_s: float = 420.0, what: str = '目标'
                     ) -> Optional[Tuple[float, float]]:
        """从当前位置飞向 leg_end，**边飞边找**；看到就刹停、对准、解算坐标。

        返回目标的**世界坐标** (x, y)；整段飞完都没看到、或解算失败返回 None。

        goto 是阻塞的，所以把航段放后台线程飞、主线程盯检测，一发现就
        `cancel_goto()` 停在当场，避免飞过头。

        Args:
            leg_end: 这一段飞到哪儿（世界坐标 (x, y)）。从**当前位置**出发。
            class_id: 边飞边找什么，比如 'apriltag:2'。
            camera: 用哪个相机找，'down'（找地面目标）或 'front'。
            agl_m: 这一段锁的离地高度（米）。
            timeout_s: 整段最多花多久（秒）。超时按"没找到"返回 None。
            what: 目标叫什么，只用于日志。
        """
        import threading
        ex, ey, ez = self.world_to_local(leg_end[0], leg_end[1], agl_m)
        done = threading.Event()

        def _fly() -> None:
            try:
                with self.fixed_altitude(ez):
                    self.goto(ex, ey, ez)
            except Exception as exc:
                self._progress(f'搜索航段结束（{exc}）')
            finally:
                done.set()

        threading.Thread(target=_fly, daemon=True).start()
        seen = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                self.wait_for_detection(class_id, timeout=0.5, camera=camera)
            except Exception:
                if done.is_set():
                    break
                continue
            self._progress(f'发现{what}，停车对准')
            self.cancel_goto()
            time.sleep(1.0)                 # 等刹停，别带着速度去对准
            seen = True
            break
        if not seen:
            return None
        try:
            self.center_on_target(class_id, timeout=40.0)
            t = self.locate_target(class_id, timeout=5.0)
        except Exception as exc:
            self._progress(f'对准/解算失败（{exc}）')
            return None
        wx, wy = self.local_to_world(t.x, t.y, 0.0)[:2]
        self._progress(f'{what}世界坐标 ({wx:.2f}, {wy:.2f})')
        return float(wx), float(wy)

    # ---- 巡检拍摄 ----
    #: snapshot() 默认存这儿。选手想换目录就 `sdk.PHOTO_DIR = '/logs/xxx'`
    #: （实例属性会盖掉类属性），不用每次调用都带参数。/logs 是挂给地面站的。
    #: 标注成 str 而不是让它推断成 Literal['/logs/照片']——不标的话编辑器会
    #: 认为只能赋这一个值，选手换目录时报类型错。
    PHOTO_DIR: str = '/logs/照片'

    #: `run()` 把命令行 `--spacing` 存到这里，任务函数直接读。
    #: **必须在类上声明**：只在 run() 里 `sdk.spacing_m = ...` 的话，类型
    #: 信息里没有这个属性，选手打 `sdk.spacing_m` 编辑器会报"属性不存在"。
    spacing_m: float = 4.0

    def snapshot(self, tag: str, camera: str = 'front',
                 directory: Optional[str] = None) -> Optional[str]:
        """拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。

        **拍不到不让任务失败**：照片是交付物，不是流程前提。

        **不发声光事件**：声光事件是固定枚举（见 `_sound_light_port.py` 的
        SOUND_LIGHT_EVENTS），表里没有"拍照"，自造事件名会直接抛 ValueError
        把整个任务打断——首次跑任务3 就是这么挂的。

        Args:
            tag: 这张照片叫什么，会进文件名（`{机号}_{时分秒}_{tag}.png`）。
            camera: 用哪个相机，'front'（前视）或 'down'（下视）。
                两个都是**固定安装**的独立相机，不存在"切视角"。
            directory: 存到哪个目录。不给就用 `self.PHOTO_DIR`。
        """
        d = directory or self.PHOTO_DIR
        path = f"{d}/{self.namespace}_{time.strftime('%H%M%S')}_{tag}.png"
        try:
            self.capture_photo(path, camera=camera)
        except Exception as exc:
            self._progress(f'拍照失败 {tag}（{exc}），流程继续')
            return None
        try:
            x, y, _ = self.get_local_position()
            wx, wy = self.local_to_world(x, y, 0.0)[:2]
            where = (f'（位置 ({wx:.2f}, {wy:.2f})，'
                     f'朝向 {math.degrees(self.get_current_yaw()):.0f}°）')
        except Exception:
            where = ''
        self._progress(f'拍照回传 {tag}：{path}{where}')
        return path

    #: look_for() 的参数。检测节点约 7 Hz，标志在画面里的话一两帧就出结果；
    #: 等 6 秒纯属在没有目标的地方白耗。留一次短重试是因为转向刚到位时画面
    #: 可能还在稳，给一帧缓冲。
    LOOK_TIMEOUT_S = 3.0
    LOOK_TRIES = 2
    LOOK_SETTLE_S = 1.0

    def look_for(self, class_id: str, camera: str = 'front',
                 tries: Optional[int] = None, timeout_s: Optional[float] = None,
                 what: str = '目标') -> Optional[Any]:
        """在**当前位置、当前朝向**找一次目标。找到返回 Detection，没找到返回 None。

        跟 `wait_for_detection()` 的区别：那个找不到会抛异常，这个返回 None
        ——巡检时"这栋楼没有火情"是正常结果，不是错误。
        """
        n = self.LOOK_TRIES if tries is None else tries
        t = self.LOOK_TIMEOUT_S if timeout_s is None else timeout_s
        for i in range(1, n + 1):
            try:
                det = self.wait_for_detection(class_id, timeout=t, camera=camera)
                self._progress(f'原地看到{what}')
                return det
            except Exception:
                if i < n:
                    time.sleep(self.LOOK_SETTLE_S)
        return None

    def patrol(self, stations: Sequence[Tuple[Any, ...]], class_id: Optional[str] = None,
               agl_m: float = 2.0, on_found: Optional[Any] = None,
               scan_sound: Optional[str] = None, found_sound: Optional[str] = None,
               once: bool = True) -> bool:
        """逐站巡检：飞到观察位 -> 转到指定机头朝向 -> 拍交付照片 -> （该查就查）。

        返回**整轮下来有没有找到过目标**。

        Args:
            stations: `[(标签, (wx, wy), 观察位名, 机头朝向°, 这站要不要查), ...]`。
                连续两站是同一个观察位时不重复飞，只原地转向（任务3 的 M 点
                要先朝南拍 3# 楼、再原地转 180° 查 2# 楼）。
            class_id: 要查的目标类别。`要不要查` 为 False 的站不会用到它。
            agl_m: 观察位的离地高度（米）。
            on_found: `on_found(det, 标签)`。找到目标时调用，**这一站的照片由它
                负责拍**——要等它把目标对准到画面正中再拍，火情在画面边上
                等于没拍到。它很可能把飞机挪走（比如前移到发射点），所以回来
                之后不再认为飞机还在观察位上。
            scan_sound: 每个要查的站开查之前播的声光事件。不给就不播。
            found_sound: 找到目标时播的声光事件，在 on_found 之前播。
            once: 找到一次之后，后面要查的站只补拍照片、不再跑识别。任务3 里
                火情只可能有一处，灭完了就不必在剩下的楼前再等识别超时。
        """
        at, found = None, False
        for tag, pt, pt_name, yaw_deg, detect in stations:
            if at != pt_name:
                self.fly_route([(float(pt[0]), float(pt[1]))], agl_m=agl_m,
                               names=[pt_name])
                at = pt_name
            self._progress(f'在 {pt_name} 点转到 {yaw_deg:.0f}° 巡检 {tag}')
            self.face_yaw(math.radians(yaw_deg))
            # 不查的站、或已经处置过的站：转到位就拍。照片是硬性交付物
            # （每栋楼必须有一张），不能让后面的识别失败把它带掉。
            if not detect or (found and once):
                self.snapshot(tag)
                if detect:
                    self._progress(f'{tag} 已处置过，只拍照不再查')
                continue
            if scan_sound:
                self.play_sound_light(scan_sound)
            det = self.look_for(class_id, camera='front', what=tag)
            if det is None:
                self.set_agl(agl_m)      # 查的过程里可能升降过，回观察高度再拍
                self.snapshot(tag)
                self._progress(f'{tag} 没有目标，继续巡检')
                continue
            found = True
            if found_sound:
                self.play_sound_light(found_sound)
            if on_found is not None:
                on_found(det, tag)
            at = None                    # on_found 很可能把飞机挪走了
        return found

    # ---- 原地机动 ----
    def own_pad(self) -> Tuple[float, float]:
        """自己起降点的世界坐标。起飞时飞机就在起降点上，局部原点换过去就是。"""
        wx, wy, _ = self.local_to_world(0.0, 0.0, 0.0)
        return (round(float(wx), 2), round(float(wy), 2))

    def progress(self, text: str) -> None:
        """往日志里打一行带机号前缀的进度。选手程序里 `print(f'[{ns}] ...')`

        Args:
            text: 要打的内容。会自动加上 `[机号]` 前缀并 flush，
                不用自己拼。
        满篇都是，用这个就不必每次自己拼前缀、也不会漏 flush=True。"""
        self._progress(text)

    def set_agl(self, agl_m: float) -> None:
        """原地升降到指定离地高度，水平位置不动（直线、不经规划器）。

        Args:
            agl_m: 目标离地高度（米）。水平位置不动，只升降。
        """
        cx, cy, _ = self.get_local_position()
        _, _, lz = self.world_to_local(0.0, 0.0, agl_m)
        self._progress(f'回到离地高度 {agl_m:.1f} m')
        self.goto_direct(cx, cy, lz)

    def step_forward(self, meters: float, what: str = '落脚点'
                     ) -> Tuple[float, float, float]:
        """沿**当前机头方向**平移 meters 米，返回落脚点的 (世界x, 世界y, 离地高度)。

        返回的第三个值是**离地高度**，可以直接喂给 `goto_world()`——别拿局部
        系的 z 当 AGL 传给队友，两机的 odom 原点不在同一处。

        Args:
            meters: 沿机头方向前移多少米。正数向前。
            what: 落脚点的名字，只用于日志。
        """
        cx, cy, cz = self.get_local_position()
        yaw = self.get_current_yaw()
        tx, ty = cx + meters * math.cos(yaw), cy + meters * math.sin(yaw)
        self._progress(f'沿机头前移 {meters:.1f} m 到{what}')
        self.goto_direct(tx, ty, cz)
        wx, wy = self.local_to_world(tx, ty, 0.0)[:2]
        return float(wx), float(wy), float(self.get_agl())

    # ---- 发射机构 ----
    #: 发射弹丸的舵机行程。2000=发射位、800=装填/复位（跟机械抓是同一组舵机，
    #: 只是语义不同：抓放是"保持"，发射是"推出去再复位"）。
    LAUNCH_FIRE_PWM = 2000
    LAUNCH_LOAD_PWM = 800
    LAUNCH_TRAVEL_S = 2.0

    def shoot(self, shots: int = 1, interval_s: float = 1.0,
              label: str = '发射', sound: Optional[str] = None) -> None:
        """发射弹丸：舵机推到发射位、等到位、再复位装填。shots>1 就连发。

        没配舵机的飞机（`sdk.servos` 为空）只打印提示，不中断流程。

        Args:
            shots: 连发几发。默认 1。
            interval_s: 两发之间隔多久（秒）。只在 shots>1 时有意义。
            label: 日志里怎么称呼（比如 '发射灭火弹'）。连发时自动加 "i/n"。
            sound: 第一发之前播的声光事件。连发只播一次。
        """
        if sound:
            self.play_sound_light(sound)
        for i in range(1, shots + 1):
            tag = label if shots == 1 else f'{label} {i}/{shots}'
            if not self.servos:
                self._progress(f'{tag}：这架飞机没有配置舵机，跳过（见 SERVO_CONFIG）')
            else:
                try:
                    self.set_servos({sv: self.LAUNCH_FIRE_PWM
                                     for sv in sorted(self.servos)})
                    time.sleep(self.LAUNCH_TRAVEL_S)
                    self.set_servos({sv: self.LAUNCH_LOAD_PWM
                                     for sv in sorted(self.servos)})
                    time.sleep(self.LAUNCH_TRAVEL_S)
                    self._progress(f'{tag}完成')
                except Exception as exc:
                    self._progress(f'{tag}：舵机没动（{exc}）')
            if i < shots and interval_s > 0:
                time.sleep(interval_s)

    # ---- 把占着的点让给队友 ----
    #: 离让出的点多远算"让开了"。这个门限只是**提前放行**用的——真正保证放行的
    #: 是"已经飞到下一个落脚点"那一路（任何 goto_world/fly_route 到达都算）。
    #: 2026-09-30 实测：火情随机到 1# 东单元时发射点 (14.40,17.33)，而侦察机的
    #: 下一个落脚点是 G(17,16)，两点只差 2.92 m，3.0 的门限永远满足不了，
    #: watchdog 要耗满超时才放行，队友白等两分钟。所以降到 2.0 并加了到达那一路。
    SPOT_CLEAR_M = 2.0
    SPOT_CLEAR_WAIT_S = 120.0
    SPOT_POLL_S = 0.2

    def yield_spot(self, spot_xy: Tuple[float, float], event: str,
                   timeout_s: Optional[float] = None) -> None:
        """宣告"我马上让开这个点"，**真的让开了**就给队友发一次 event。

        为什么需要：队友被要求飞到"我现在站的那个点"，而那个点上此刻正杵着
        我。ego_planner 看到目标点被占，会把轨迹终点推到障碍边缘、飞机原地
        不动，5 秒后被 goto() 判成不可达——2026-09-30 实测任务机收到破窗通知
        立刻出发（07:26:51.74），侦察机同一瞬间才开始离开（07:26:51.94），
        6 秒后任务机就挂了。

        调用方的动作顺序不用改（照常"发射完就走"），只是多发一条"我让开了"，
        队友收到这条才真的飞进去。

        两条放行路径，先到哪条算哪条：
          · 距离门限：后台线程盯着，离 `SPOT_CLEAR_M` 远就放行（提前放行）；
          · **到达下一个落脚点**：之后任何 `goto_world()`/`fly_route()` 到点就
            放行。这是主路径——下一个落脚点有可能离让出的点很近，光靠门限
            放不了行。
        超时也放行并在日志里说清楚，不然队友会一直等。

        Args:
            spot_xy: 要让出的那个点的世界坐标 (x, y)。
            event: 让开之后给队友发哪个事件名。
            timeout_s: 兜底超时（秒）。不给用 SPOT_CLEAR_WAIT_S（120 秒）。
                两条放行路径都没触发时，到点也会放行并在日志里说清楚——
                不然队友会一直等下去。
        """
        import threading
        sx, sy = float(spot_xy[0]), float(spot_xy[1])
        wait_s = self.SPOT_CLEAR_WAIT_S if timeout_s is None else timeout_s
        state = {'sent': False}
        self._spot_watch = state

        def _send(why: str) -> None:
            if state['sent']:
                return
            state['sent'] = True
            try:
                self.send_to_teammate(event)
                self._progress(f'{why}，通知队友进场')
            except Exception as exc:
                self._progress(f'"已让开"没送到队友（{exc}）')

        state['send'] = _send

        def _loop() -> None:
            t0 = time.time()
            while time.time() - t0 < wait_s and not state['sent']:
                try:
                    cx, cy, _ = self.get_local_position()
                    wx, wy = self.local_to_world(cx, cy, 0.0)[:2]
                except Exception:
                    time.sleep(self.SPOT_POLL_S)
                    continue
                d = math.hypot(wx - sx, wy - sy)
                if d >= self.SPOT_CLEAR_M:
                    _send(f'已离开让出的点（{d:.1f} m ≥ {self.SPOT_CLEAR_M:.1f} m）')
                    return
                time.sleep(self.SPOT_POLL_S)
            # 走到这儿说明两条路都没触发，属于异常，但也要放行。
            _send(f'等了 {wait_s:.0f} 秒仍没离开让出的点（超时兜底）')

        threading.Thread(target=_loop, daemon=True).start()

    def _spot_clear_arrived(self, where: str) -> None:
        """到达任何一个落脚点 —— 不管离让出的点多远，位置都已经腾出来了。

        `yield_spot()` 的主放行路径，由 `goto_world()` / `_goto_with_retry()`
        自动调用，选手不用管。
        """
        state = getattr(self, '_spot_watch', None)
        if state and not state['sent']:
            state['send'](f'已飞到{where}，让出的点已腾出')

    # ---- 选手程序的 main() ----
    @staticmethod
    def run(leader: Any, follower: Any, description: Optional[str] = None,
            spacing_m: float = 4.0) -> None:
        """选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。

        三个单任务程序的 `main()` 本来一字不差地重复同样 14 行 argparse，
        收到这里。用法：

            if __name__ == '__main__':
                DroneSDK.run(leader=recon, follower=supply)

        `--spacing` 存成 `sdk.spacing_m`，任务函数直接读。`--route` 老运行器会
        传给编队那一支，这里一并吃掉，免得 lite 版因为多一个参数就起不来。

        Args:
            leader: 长机那一支要跑的函数，签名是 `f(sdk)`。
                `--role` 是 leader 或 recon 时调它。
            follower: 僚机那一支要跑的函数，签名同上。
            description: 命令行 `--help` 里显示的说明，一般传 `__doc__`。
            spacing_m: `--spacing` 的默认值（米）。实际取到的值会存成
                `sdk.spacing_m`，任务函数直接读。
        """
        import argparse
        ap = argparse.ArgumentParser(description=description)
        ap.add_argument('--namespace', required=True)
        ap.add_argument('--role', required=True,
                        choices=('leader', 'follower', 'recon', 'supply'))
        ap.add_argument('--teammate', required=True)
        ap.add_argument('--spacing', type=float, default=spacing_m)
        ap.add_argument('--route', nargs='*')
        args = ap.parse_args()
        is_leader = args.role in ('leader', 'recon')
        sdk = DroneSDK(namespace=args.namespace,
                       role='recon' if is_leader else 'supply',
                       teammate_namespace=args.teammate)
        sdk.spacing_m = float(args.spacing)
        try:
            (leader if is_leader else follower)(sdk)
        finally:
            sdk.shutdown()

    def shutdown(self) -> None:
        """选手程序整体退出时应该调用一次：先归还`reliability.py`占用的
        定时器/发布者/订阅者，再关闭`RclpyRuntime`（停止后台spin、销毁
        节点、必要时关闭rclpy全局context）。幂等——重复调用不会报错。
        """
        self._event_channel.shutdown()
        self._runtime.shutdown()
        if self._sound_light_conn is not None:
            self._sound_light_conn.close()


def _make_pose_stamped(header: Any, x: float, y: float, z: float) -> Any:
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header = header
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    return pose
