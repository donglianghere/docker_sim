#!/usr/bin/env python3
"""AprilTag精准降落视觉伺服节点。

对应《2026大赛任务系统开发执行方案.md》阶段6.2。订阅阶段2
`qr_apriltag_detect_node`发布的`vision_msgs/Detection2DArray`
（`vision/detections`），取bbox中心相对图像中心的像素偏移，换算成
机体系水平偏差，用简单P控制算出修正后的`quadrotor_msgs/
PositionCommand`（位置微调+水平误差收敛后才开始固定速率下降），
发布到`precision_land_cmd`——阶段3的`position_cmd_relay_node`切到
`precision_land`模式时，消费的就是这个话题（该节点的`_on_precision_
land_cmd`回调、`precision_land_cmd`这个topic名字，两边阶段3写的时候
就已经对齐好了）。

**没有用AprilTag库自带的6DOF位姿解算**（`apriltag.Detector.detection_
pose()`其实支持，给定相机内参+tag实际边长能解出真实的相对位姿）——
故意选了更简单的路径：直接用bbox中心的像素偏移+当前高度+相机FOV换算
成米制偏移（复用阶段5.1`ground_scan_helper.compute_ground_footprint()`
同一套针孔相机换算公式，不重复定义一遍）。理由：
1. 6DOF位姿解算引入旋转矩阵→四元数转换这一整块新的正确性风险，而
   这次精降只需要"水平往哪个方向修正、修正多少"这一个标量级的信息，
   不需要tag的完整3D姿态。
2. 阶段2的下视相机朝向本身还没有实机(GUI)验证过（见阶段2记录，
   `CAMERA_MOUNTS`里的pitch角是按标准SDF写法推算的，没有肉眼确认过）
   ——在朝向这个前提都还不确定的情况下，用像素偏移这条更简单的路径，
   出问题时更容易定位（"符号/轴对错了"比"整个6DOF解算哪里错了"好排查
   得多）。

⚠️ **图像坐标轴→机体系的映射是这次任务里最大的未验证假设**：默认假定
"图像上方(像素y减小方向)对应机体系+X(机头前方)、图像右方(像素x增大
方向)对应机体系-Y"——这是没有做过相机roll实测的猜测值，不是查资料
查到的确定结论。`invert_x`/`invert_y`/`swap_xy`三个参数就是留给实测
之后调的旋钮：真实降落时如果发现水平误差不收敛反而越修越偏，先怀疑
这三个参数的组合不对，不是控制算法本身有问题（控制算法的收敛性已经
用独立的闭环仿真测试验证过，见本文件同目录的测试脚本）。

**2026-09-13补充：拆成`servo_mode`两种模式**（对应《2026大赛任务系统
全流程任务仿真实现方案.md》1.7节/清单A5节，用户直接给的设计）——原来
这个节点只有一种写死的行为：水平误差收敛后开始下降，下降到
`min_descent_z`后停在那个高度悬停，既不触发真正降落，也不对外广播
"我现在的位置"。这个行为只够支撑"精降到底、原地悬停等抓取"这一种
用途（新参数`servo_mode='precision_land'`），支撑不了"命中地面火情后
只需要悬停居中读自己坐标广播给队友，完全不需要下降"这种用途（新参数
`servo_mode='center_only'`）。两种模式共享同一套水平误差收敛计算逻辑
（`_on_detections`一行没改），只是收敛之后的行为分叉——`center_only`
广播`centered_pose`+`servo_status='centered'`后交还控制权（停止发布
`precision_land_cmd`）；`precision_land`在到达`min_descent_z`后**新增**
复用`quadrotor_msgs/TakeoffLand{cmd:LAND}`触发真正降落（跟`px4ctrl_
bridge/takeoff_gate.py`起飞时用的、`gcs/backend/app.py`
`_publish_takeoff_land()`底层调的是同一个消息接口，不新造降落机制），
订阅`mavros/state`等`armed`变`false`确认真正落地后广播
`servo_status='landed'`。

`servo_mode`故意不给合法默认值（默认空字符串会在构造时直接抛异常）——
这两种模式的飞行后果差异极大（一个绝不下降，一个必然触发真正的
AUTO_LAND降落），不能让调用方在没意识到的情况下落入某个隐含默认值。

⚠️ **2026-09-14修复：自动接管`position_cmd_relay`的`relay_mode`**（照抄
`fire_pillar_aim_node.py`2026-09-10"锁定后自动接管"那次修复，同一个
坑、这个节点当时漏改）——`position_cmd_relay_node.py`默认`relay_mode
='normal'`，只透传`ego_planner`的常规轨迹指令，本节点发到
`precision_land_cmd`的位置指令会被直接丢弃，不管`center_only`还是
`precision_land`模式都一样（两种模式共用同一个publisher）。之前没有
任何代码去把`relay_mode`切到`'precision_land'`，本节点内部的"水平
误差是否收敛"判断纯粹是自己算出来的、飞机实际执行的还是被relay透传
的旧指令，两者对不上——`servo_status`永远等不到`'centered'`/`'landed'`，
即使`precision_land`模式误判"已收敛"触发了`TakeoffLand(LAND)`，飞机
真实位置也大概率没有真正对准目标（2026-09-14实测复现：检测确实在跑，
但`servo_status`全程是`None`，`DEBUG_JOURNAL.md`同日期条目完整记录了
排查过程）。修复：本节点自己在接管开始时调`position_cmd_relay`的
`~/set_parameters`把`relay_mode`切成`'precision_land'`，center_only
收敛广播`'centered'`/precision_land触发`_trigger_land()`交还控制权
的同一时刻对称地切回`'normal'`——谁接管的谁负责交还，跟
`fire_pillar_aim_node.py`的`auto_switch_relay_mode`是同一套参数设计。

⚠️ **同一天的第二次修复：`_set_relay_mode()`不能照抄`fire_pillar_aim_
node.py`用`rclpy.spin_until_future_complete()`阻塞等结果这个实现**——
第一版照抄了，配上`MultiThreadedExecutor`+独立callback group（当时
以为这是`fire_pillar_aim_node.py`"死锁教训"注释要求的标准做法），
实测直接把整个节点锁死：`_on_control_tick`是10Hz定时器回调，不是
`fire_pillar_aim_node.py`那种只触发一次的subscription回调，在已经被
外层`MultiThreadedExecutor`持续高频调度的节点里再嵌套`spin_until_
future_complete()`，"开始接管飞控"那条日志打完之后整个节点彻底停止
响应（`ros2 param get`都会超时），不是短暂卡顿。改成完全不阻塞的
`call_async()`+`add_done_callback()`，见`_set_relay_mode()`/`_on_
relay_mode_set_done()`的完整说明。`MultiThreadedExecutor`+独立
callback group还是保留了（异步回调本身也需要被调度到，用MultiThreaded
更保险），但不再靠"阻塞等待+多线程"这个组合来避免死锁。
"""
import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, Quaternion
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from rcl_interfaces.msg import (
    Parameter, ParameterDescriptor, ParameterType, ParameterValue, SetParametersResult,
)
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from contest_mission.ground_scan_helper import compute_ground_footprint

# 四种模式的取值，跟position_cmd_relay_node.py的VALID_MODES是同一种写法
# 习惯——一个模块级常量元组，构造时校验+日志提示里都复用它，不各处重复
# 写字面量列表。
# 2026-09-14新增`coordinate_land`：用户提出"给定坐标点的精降模式，如
# 返回起降点时的降落——起飞前记录起降点坐标，返回降落时用精准降落"。
# 跟`precision_land`共用完全相同的分级下降+AUTO_LAND交接逻辑（已经
# 实测验证过），唯一区别是水平误差的来源：`precision_land`/`center_
# only`靠视觉检测(`_on_detections`)算像素偏移换算成世界系偏移；
# `coordinate_land`没有视觉这一环，直接拿`target_x`/`target_y`这个
# 提前设好的目标坐标跟当前里程计位置作差——不需要看得见任何AprilTag/
# 二维码，纯靠已知坐标+里程计精度。见`_on_control_tick()`里的分支
# 说明。
# 2026-09-14再新增`coordinate_goto`：用户指出`coordinate_land`不走
# `ego_planner`，没有避障能力，这只在"已知路径肯定畅通"（比如起降点
# 正上方）这种场景下安全；但同样这套"不避障、直达"的能力反过来对
# 普通飞行也有用——"goto可以有一个coordinate版，即不避障、直达版"。
# `coordinate_goto`就是这个：三维同时朝目标坐标收敛（不像`coordinate_
# land`那样先收敛水平再分级下降——没有"地效强、要交给AUTO_LAND"这个
# 顾虑，直接3D一起走），到点后广播`servo_status='arrived'`+交还控制权，
# **不会**触发降落——用户必须清楚这条路径完全不避障，只应该在已知
# 无障碍物的路段使用（比如起降点附近、开阔区域），不能拿来替代
# `goto()`跑有障碍物的常规航线。
VALID_SERVO_MODES = ('center_only', 'precision_land', 'coordinate_land', 'coordinate_goto')

# precision_land模式下发TakeoffLand(LAND)的重发次数——照抄takeoff_gate.py/
# gcs后端_publish_takeoff_land()同款"连发几次而不是一次"的防御写法（注释
# 原话："ros2 topic pub --once在DDS discovery没跟上时可能白发"），这里没有
# 起飞门那种一次性子进程，是本节点自己常驻的publisher，所以改成分散在接下来
# 几个控制周期里补发，而不是time.sleep()阻塞——这个节点跑在10Hz的定时器
# 回调里，绝不能在回调内部sleep卡住整个rclpy executor。
_LAND_CMD_REPEAT = 3

# 判定"已经下降到land_handoff_height_m"的容差——分级下降每一级用
# `max(z - descent_step_m, handoff_height)`夹到交接高度，理论上最后
# 一级会精确落在handoff_height上，这个小容差只是防浮点误差，不是像
# 废弃的旧连续下压逻辑那样需要容忍"到底前还差一点点"的情况。
_DESCENT_ARRIVED_TOLERANCE_M = 0.02


def pixel_offset_to_body_frame(
    dx_px: float,
    dy_px: float,
    altitude_agl: float,
    hfov_rad: float,
    image_width: int,
    image_height: int,
    invert_x: bool = False,
    invert_y: bool = False,
    swap_xy: bool = False,
) -> Tuple[float, float]:
    """像素偏移(tag中心-图像中心) -> 机体系水平偏移(米)。见文件头
    "图像坐标轴→机体系的映射"那段说明，默认映射是未经实测验证的假设。
    """
    if altitude_agl <= 0:
        raise ValueError(f'altitude_agl必须>0，收到{altitude_agl}')
    footprint_w, footprint_h = compute_ground_footprint(
        altitude_agl, hfov_rad, image_height / image_width
    )
    dx_m = dx_px * (footprint_w / image_width)
    dy_m = dy_px * (footprint_h / image_height)

    body_x = -dy_m  # 假设：图像上方=机体+X（前）
    body_y = -dx_m  # 假设：图像右方=机体-Y
    if swap_xy:
        body_x, body_y = body_y, body_x
    if invert_x:
        body_x = -body_x
    if invert_y:
        body_y = -body_y
    return body_x, body_y


def body_offset_to_world(body_x: float, body_y: float, yaw_rad: float) -> Tuple[float, float]:
    """机体系水平偏移 -> 世界系水平偏移，按当前yaw旋转。"""
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    world_x = body_x * cos_yaw - body_y * sin_yaw
    world_y = body_x * sin_yaw + body_y * cos_yaw
    return world_x, world_y


def quaternion_to_yaw(q: Quaternion) -> float:
    """只取yaw（绕Z），精降阶段假设roll/pitch接近0，不需要完整欧拉角。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PrecisionServoNode(Node):
    def __init__(self):
        super().__init__('precision_servo_node')

        self.declare_parameter('target_class_id', 'apriltag:0')  # ID0=物资点，见fire_drill_room_layout.yaml
        # 2026-09-14新增：`coordinate_land`模式专用，见VALID_SERVO_MODES
        # 声明处说明——目标坐标，这架飞机自己的局部坐标系(跟odom/goto()
        # 同一套)，不是世界坐标。`target_class_id`对这个模式没有意义
        # （不看视觉），调用方不需要设置。
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        # 2026-09-14新增：`coordinate_goto`模式专用（见VALID_SERVO_MODES
        # 声明处说明），三维目标里的Z分量，跟`target_x`/`target_y`同一套
        # 局部坐标系。`coordinate_land`不需要这个参数——它的目标Z隐含
        # 是"地面"，靠`land_handoff_height_m`+`AUTO_LAND`控制，不是
        # 一个提前给定的固定Z坐标。
        self.declare_parameter('target_z', 0.0)
        # 2026-09-14新增：`coordinate_goto`判定"已到点"用的阈值——跟
        # `horizontal_converge_threshold_m`分开声明（虽然默认值一样），
        # 因为这个是三维距离阈值，语义上跟"水平误差"阈值不是同一个量，
        # 未来两个值有必要分开调时不用共用一个参数打架。
        self.declare_parameter('arrival_threshold_m', 0.3)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_hfov_rad', 1.3963)  # 跟gen_iris_mid360_sdf.py的CAMERA_MOUNTS['down']一致
        self.declare_parameter('kp_xy', 0.5)
        # 2026-09-15新增：`coordinate_land`/`coordinate_goto`专用的水平
        # 限速——这两个模式不走ego_planner，没有速度规划/限幅，`target_x
        # = x + kp*(目标-x)`这一步单纯是P控制，误差大的时候算出来的
        # 单个控制周期(10Hz)位移step本身就很大，实测20米长距离飞行中段
        # 出现过瞬时超过3m/s的速度尖峰（这套模式完全没有避障能力，速度
        # 一快、离障碍物的反应时间就更短）。用户直接要求"这个测试速度
        # 只能是0.3米/秒"——限的是这两个模式自己的水平位移速率上限，
        # 不是重新引入被用户明确禁止的"分段插值"（分段插值是调用方在
        # 目标点之间自己插一堆中间点、绕不开"不知道中间有什么"这个问题；
        # 这里是控制器内部限制单个控制周期的最大步长，目标点还是一次性
        # 给的原始目标，两者不是一回事）。
        self.declare_parameter('coordinate_max_speed_mps', 0.3)
        # 2026-09-14用户重新设计精降下降逻辑，`descent_rate_mps`+
        # `min_descent_z`（连续匀速下压到很低高度才触发降落）这套逻辑
        # 整个废弃，改成`land_handoff_height_m`+`descent_step_m`这套
        # "对准一点、下降一点、再对准、再下降"的分级下降+提前交接，见
        # 下面`land_handoff_height_m`声明处的完整说明。
        # 2026-09-14用户实测直接指出：0.15米比这套仿真(uwb_imu+pt4ctrl组合)
        # 本身的正常悬停抖动(0.3~0.4米量级，见capabilities.py::TAKEOFF_
        # STABLE_POS_TOLERANCE_M同一个结论的完整实测说明)还要紧——飞控
        # 自己的位置保持精度都到不了0.15米，要求视觉伺服收敛进比这更紧
        # 的阈值，原理上就不可能达成，不是控制算法调参能解决的，是阈值
        # 本身定得不现实。改成跟`ARRIVAL_THRESHOLD_M`/`TAKEOFF_STABLE_
        # POS_TOLERANCE_M`同一个量级(0.3米)——这个值已经是这个项目里
        # "悬停/到点"判据的既有标准，不是另外凭空定的新数字。
        self.declare_parameter('horizontal_converge_threshold_m', 0.3)
        # 2026-09-14用户重新设计的分级下降逻辑——实测发现"用位置指令
        # (CMD_CTRL)一路压到很低高度(原min_descent_z=0.15米)才触发
        # AUTO_LAND"这套做法行不通：即使水平已经在threshold内、即使加了
        # 硬超时强制跳过收敛判断，`z`实测也压不下去，卡在0.32~0.43米
        # 反复摆动，从来到不了0.15米（可能是pt4ctrl普通位置追踪在接近
        # 地面时精度/响应本身就到不了这么低，也可能是地效导致的扰动，
        # 没有细查根因）——但同样这个高度区间，普通`sdk.land()`（直接走
        # `AUTO_LAND`状态机）几秒内就干净落地了，说明`AUTO_LAND`本身
        # 可靠，问题出在"精降节点自己用位置指令硬压高度"这个设计上。
        # 用户给的新思路："对准了就往下降一点，而后再对准，而后再往下降
        # ……最后50cm高对准后就直接用autoland指令降落，否则地效太强，
        # 也无法对准"——不再连续匀速下压到很低高度，改成分级下降：
        # 每次水平误差收敛(持续满足converge_hold_duration_s)就下降
        # `descent_step_m`一截，然后要求在新高度重新持续对准才能再下一
        # 级；一旦高度降到`land_handoff_height_m`附近且这一级也已经
        # 对准，不再继续自己往下压，直接触发`_trigger_land()`交给
        # `AUTO_LAND`做剩下的最后一段——这一段本来就已知不需要精降节点
        # 插手也能可靠完成。
        self.declare_parameter('land_handoff_height_m', 0.5)
        self.declare_parameter('descent_step_m', 0.3)
        # 2026-09-14新增：精降模式的硬超时兜底——用户直接指出"是一个时间
        # 限制，超过60秒不管如何都降落"。纯P控制(没有积分项)在存在持续
        # 扰动/系统性偏差时可能有稳态误差，水平误差理论上可能永远收敛
        # 不进threshold，分级下降也可能因此永远迈不出第一步。与其让飞机
        # 无限期悬停等一个可能永远不会发生的"精确收敛"，不如定一个硬
        # 超时：从进入精降控制(relay_mode真正切过去)那一刻起计时，超过
        # 这个时长后不再管水平误差/当前下降到第几级，直接触发`_trigger_
        # land()`——反正`AUTO_LAND`本身可靠，用当前位置(不管对得准不准)
        # 触发也比"选手SDK那层60秒超时抛异常、飞机被晾在半空"安全得多，
        # 这才是"精降"该有的兜底行为：尽力对准，但绝不能无限期悬停不
        # 落地。
        self.declare_parameter('precision_land_max_wait_s', 60.0)
        # 2026-09-15新增：`coordinate_goto`专用超时兜底，跟`precision_
        # land_max_wait_s`分开——那个是`precision_land`/`coordinate_
        # land`（真正降落场景）"绝不能无限期悬停不落地"这条安全网，
        # 用户当时明确要求过，不能动。但`coordinate_goto`（纯"飞到点
        # 悬停"，不涉及降落）复用同一个60秒超时，在仿地飞行诊断测试里
        # 反而帮了倒忙——20米长距离配合0.3m/s限速需要60多秒，一到60秒
        # 硬超时就在飞机根本没到点的情况下强行广播`servo_status='arrived'`，
        # 让调用方误以为真的到点了，掩盖了"到底有没有真收敛"这个本来
        # 想看清楚的问题。默认值0.0表示关闭（不设上限，收敛多久等多久，
        # 参数为0/负数时`_on_control_tick`里判断`>0`才生效，跟原来
        # `precision_land_max_wait_s`的行为不冲突，需要时可以单独调回
        # 一个正数重新启用兜底）。
        self.declare_parameter('coordinate_goto_max_wait_s', 0.0)
        self.declare_parameter('detection_timeout_s', 1.0)
        self.declare_parameter('control_rate_hz', 10.0)
        # 三个轴映射修正旋钮，见文件头说明。
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('swap_xy', False)

        # 2026-09-14新增：自动接管position_cmd_relay的relay_mode，见文件头
        # 同日期说明——照抄fire_pillar_aim_node.py的auto_switch_relay_mode/
        # relay_set_parameters_service两个参数，同一套开关。
        self.declare_parameter('auto_switch_relay_mode', True)
        self.declare_parameter('relay_set_parameters_service', 'position_cmd_relay/set_parameters')

        # servo_mode：见文件头"2026-09-13补充"说明，故意用空字符串当默认值——
        # 空字符串不在VALID_SERVO_MODES里，构造时下面的校验会直接抛异常，
        # 强制调用方（launch文件/`ros2 run ... --ros-args -p servo_mode:=xxx`）
        # 显式设置，不给一个"看起来能跑但可能选错模式"的隐藏默认。
        self.declare_parameter(
            'servo_mode', '',
            ParameterDescriptor(
                description=(
                    f'必须显式设置为{VALID_SERVO_MODES}之一：'
                    "center_only=仅居中定位(收敛后不下降，广播坐标即结束)；"
                    "precision_land=精降抓取(收敛后下降到底并触发真正降落)。"
                    "不给合法默认值，见文件头说明。"
                )
            ),
        )
        # 水平误差收敛后要持续稳定这么久才算"真的收敛"，避免瞬间抖动
        # （比如检测噪声导致某一帧误差刚好skim过threshold）被误判为已收敛——
        # 0.5~1.0秒是清单A5节给的建议区间，这里取中间值0.7秒。
        self.declare_parameter('converge_hold_duration_s', 0.7)

        servo_mode = self.get_parameter('servo_mode').value
        if servo_mode not in VALID_SERVO_MODES:
            # 2026-09-13实测发现的问题：这个节点是flight-stack-entrypoint.sh
            # 每架飞机启动时就常驻拉起的（跟formation_follower_node同一类，
            # 角色/用途运行时才通过SDK的set_parameters决定，容器启动那一刻
            # 不可能知道该是center_only还是precision_land）——如果这里在
            # __init__里直接raise，容器一起来这个节点就崩溃退出，跟
            # formation_follower_node"节点常驻但默认不生效"这条既有设计
            # 模式冲突。改成：构造时只打警告、不崩溃，节点保持"待命"状态
            # （下面_on_control_tick第309-319行本来就有对非法servo_mode的
            # 运行时防御，跳过控制、不发布），等第四部分程序通过
            # `ros2 param set <ns>/precision_servo_node servo_mode <mode>`
            # （或contest_sdk的center_on_target()/precision_land_and_
            # confirm()底层调的~/set_parameters）显式设置后才会真正生效。
            # "两种模式后果完全不同、不能有隐含默认值"这条设计意图不变——
            # 变的只是"拿什么手段防止蒙混过关"：不是让节点直接崩溃退出
            # （反而更危险，容器里少了一个节点比"暂时待命"更不容易被
            # 发现），是让控制逻辑在没有显式设置前压根不会执行。
            self.get_logger().warn(
                f"servo_mode参数当前是{servo_mode!r}，不在{VALID_SERVO_MODES}"
                f"之列——节点保持待命，不会发布precision_land_cmd，直到通过"
                f"`ros2 param set <ns>/precision_servo_node servo_mode "
                f"<center_only|precision_land>`显式设置为合法值。'仅居中"
                f"定位'和'精降抓取'是两种飞行后果完全不同的行为（前者绝不"
                f"下降，后者必然触发真正的AUTO_LAND降落），调用方必须显式"
                f"选择，这里故意不给一个可能选错的隐含默认值。"
            )

        #: "待命"提示是否已经打过（2026-09-20）：servo_mode为空是预期状态，
        #: 只在第一次进入时提示一次，拿到合法模式后复位，避免刷屏。
        self._standby_logged = False

        self._odom_xy_z_yaw: Optional[Tuple[float, float, float, float]] = None
        self._last_world_offset: Optional[Tuple[float, float]] = None
        self._last_detection_time: Optional[float] = None

        # === center_only模式专用状态 ===
        # 记录"持续满足水平收敛条件"是从哪个time.monotonic()时刻开始的，
        # None表示当前不在"持续收敛"区间内（还没收敛，或者中途又超出过阈值）。
        self._converge_since: Optional[float] = None
        # 是否已经广播过一次centered_pose+servo_status='centered'——一次
        # 任务只应该广播一次，广播完就交还控制权，不重复触发。
        self._centered_notified = False

        # === coordinate_goto模式专用状态 ===
        # 跟`_converge_since`/`_centered_notified`是同一套模式，但
        # `coordinate_goto`是三维同时收敛（不是center_only那种只管
        # 水平），单独一份状态，不跟center_only混用。
        self._goto_converge_since: Optional[float] = None
        self._arrived_notified = False

        # === precision_land模式专用状态 ===
        # 是否已经触发过TakeoffLand(LAND)真正降落——触发之后本节点不再
        # 发布precision_land_cmd（见_on_control_tick顶部的分支），把飞行
        # 控制权完全交还给px4ctrl自己的AUTO_LAND状态机。
        self._landing_triggered = False
        # 2026-09-14新增：分级下降用的"持续对准"计时起点，见文件头/
        # land_handoff_height_m声明处的完整说明——跟center_only模式的
        # `_converge_since`是同一种"必须持续满足阈值这么久才算数"的
        # 判据，但这是精降模式自己单独的一份状态（每下降一级都要清空
        # 重新计时，不能跟center_only共用一个变量）。
        self._descent_converge_since: Optional[float] = None
        # 触发降落后还需要补发几次TakeoffLand(LAND)（见_LAND_CMD_REPEAT
        # 的注释，防御DDS discovery/丢包，不是time.sleep()阻塞式重发）。
        self._land_repeat_remaining = 0
        # 是否已经广播过一次servo_status='landed'，避免armed反复抖动
        # 触发多次广播。
        self._landed_notified = False

        # 2026-09-14新增：relay_mode是否已经确认切到'precision_land'——
        # 见文件头同日期说明。惰性切换（第一次真正要发precision_land_cmd
        # 时才切），不是构造时就切，避免节点常驻待命(servo_mode还没被
        # 外部设成合法值)时就抢占relay_mode，跟formation_follower_node
        # "常驻但默认不生效"是同一个设计原则。
        self._relay_mode_active = False
        # 2026-09-14新增：precision_land_max_wait_s硬超时兜底计时起点——
        # 跟_relay_mode_active在同一个时刻(真正接管飞控那一刻)一起置位，
        # 见_on_control_tick里的用法。
        self._control_start_time: Optional[float] = None

        self.pub = self.create_publisher(PositionCommand, 'precision_land_cmd', 10)
        self.create_subscription(Odometry, 'dlio/odom_node/odom', self._on_odom, 10)
        self.create_subscription(Detection2DArray, 'vision/detections', self._on_detections, 10)

        # centered_pose：center_only模式收敛完成后广播的坐标x/y（这架
        # 飞机自己的局部坐标系`{ns}/odom`，不是跨机共享的world坐标系——
        # 2026-09-13订正：这条注释和下面_publish_centered_pose()原来都
        # 写"世界坐标"，是措辞错误，实际发布的一直是本机局部坐标，从
        # 没变过；contest_sdk的center_on_target()文档同步订正过，跨机
        # 广播前需要选手自己调sdk.local_to_world()换算）。用
        # geometry_msgs/PointStamped而不是复用PositionCommand——这里只是
        # "一个带时间戳的三维点"这个语义，PositionCommand上那一整套速度/
        # 加速度/jerk/yaw/trajectory_id字段在"仅居中定位、不下降、不改变
        # 控制状态"这个场景下全部没有意义，硬塞进去反而容易让消费方
        # （比如contest_sdk的center_on_target()）误以为这些字段也有信息，
        # PointStamped更轻量、语义更准确。
        self.centered_pose_pub = self.create_publisher(PointStamped, 'centered_pose', 10)
        # servo_status：跟actuator_action_node.py的action_status同一种
        # "String话题广播一次性状态字符串、方便SDK轮询"的风格，取值
        # 'centered'/'landed'，见文件头/_on_control_tick里的具体触发点。
        self.status_pub = self.create_publisher(String, 'servo_status', 10)
        # takeoff_land：precision_land模式下降到底后触发真正降落用的现有
        # 接口，跟px4ctrl_bridge/takeoff_gate.py起飞时用的、gcs后端
        # _publish_takeoff_land()底层调的是同一个TakeoffLand消息+同一个
        # 话题名（相对话题名，靠launch层的namespace区分双机），不新造
        # 降落机制。
        self._land_pub = self.create_publisher(TakeoffLand, 'takeoff_land', 10)
        # 订阅mavros/state确认armed变false，即"真正落地"（不能只信自己
        # 发过LAND指令就假装已经落地——同样是"指令发出去≠指令被执行了"
        # 这条项目里反复踩过的坑，见gcs/backend/app.py `_recent_px4ctrl_
        # feedback()`那段说明；这里换成订阅状态话题确认，而不是那个函数
        # 用的读docker logs方式，因为本节点在飞控容器内部就是常驻节点，
        # 直接订阅话题比读日志更直接可靠）。
        self.create_subscription(State, 'mavros/state', self._on_mavros_state, 10)

        # 2026-09-14新增：调position_cmd_relay的~/set_parameters切relay_mode，
        # 见文件头同日期说明。单独的callback group+main()里改成
        # MultiThreadedExecutor——照抄fire_pillar_aim_node.py"死锁教训"：
        # 在subscription/timer回调内部同步调其它service，跟这个节点默认
        # 的callback group共用会在单线程executor下自己等自己死锁。
        client_cb_group = MutuallyExclusiveCallbackGroup()
        relay_svc_name = self.get_parameter('relay_set_parameters_service').value
        self.relay_set_params_cli = self.create_client(
            SetParameters, relay_svc_name, callback_group=client_cb_group,
        )

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_control_tick)

        # 2026-09-15新增：修复"同一个节点连续接第二次任务时被自己的
        # '已完成'状态锁死"这个bug——`_centered_notified`/`_arrived_
        # notified`/`_landing_triggered`/`_landed_notified`这几个"已经
        # 广播/触发过一次，不用再做了"的标记，原来只在__init__里初始化
        # 一次、完成时置True，从来没有在"新一轮任务开始"时被重置回False。
        # 实测复现：`goto_direct()`第一次调用成功到点后，第二次换一个新
        # 目标坐标再调`goto_direct()`，本节点`_on_control_tick`里`if
        # servo_mode == 'coordinate_goto' and self._arrived_notified:
        # return`这条早退检查因为`_arrived_notified`还停留在上一次的
        # True，直接把第二次任务整个短路掉——不发布任何指令，飞机原地
        # 不动，日志上看起来像"控制律对长距离失效"，实际上是状态没清零，
        # 跟距离、控制增益都没关系。`center_only`/`precision_land`/
        # `coordinate_land`三种模式的对应标记是同一个模式的bug，一并修。
        # 用`add_on_set_parameters_callback`而不是在`_on_control_tick`里
        # 比较"target有没有变"，是因为SDK每次发起新任务都会重新调一次
        # `~/set_parameters`把`servo_mode`（哪怕值不变）连同新的目标坐标
        # 一起发过来，这是"一次新任务开始"这个事件在协议层面唯一、可靠
        # 的信号——比较目标坐标数值在"回到同一个点"这种场景下会误判成
        # "没有新任务"。
        self.add_on_set_parameters_callback(self._on_params_set)

        self.get_logger().info(
            f"precision_servo就绪，servo_mode={servo_mode}，target_class_id="
            f"{self.get_parameter('target_class_id').value}，target_x/y/z="
            f"{self.get_parameter('target_x').value}/{self.get_parameter('target_y').value}/"
            f"{self.get_parameter('target_z').value}（coordinate_land只用x/y，"
            f"coordinate_goto用x/y/z），"
            f"发布到precision_land_cmd（阶段3中继要切到precision_land模式才会消费）。"
            f"center_only模式收敛完成后广播centered_pose+servo_status=centered；"
            f"coordinate_goto三维收敛完成后广播servo_status=arrived（不避障，只应该在"
            f"已知无障碍物路段用）；precision_land/coordinate_land两种模式分级下降"
            f"到位后触发真正降落+等待armed=false后广播servo_status=landed（前者靠"
            f"视觉检测算水平误差，后者直接拿target_x/y跟里程计位置作差）。"
        )

    def _on_params_set(self, params) -> SetParametersResult:
        # 见__init__里注册这个回调时的说明——只要这次set_parameters调用
        # 里包含'servo_mode'，就认定是"一次新任务开始"，把上一轮任务
        # 遗留的"已完成/已触发"锁存状态和收敛计时器清零，否则第二次任务
        # 会被第一次任务留下的标记直接短路掉。不在这里挋绝/修改参数值
        # 本身（只借这个回调当"新任务开始"的信号），永远返回successful，
        # 不影响参数系统正常赋值。
        if any(p.name == 'servo_mode' for p in params):
            # 2026-09-23：被设回待命（servo_mode=''）时，把relay_mode交还
            # 'normal'——文件头"谁接管的谁负责交还"那条原则，原来只在正常
            # 完成（landed/centered/arrived）的路径上做了，"半路被叫停"这条
            # 路径漏了：选手放弃精降改用普通降落时（SDK的
            # stop_precision_servo()）节点确实不再发指令，但relay还停在
            # 'precision_land'，后面所有goto()的position_cmd都会被中继丢掉，
            # 飞机看着"收到指令却不动"。这里要在下面那批状态被清零之前读
            # _relay_mode_active，因为紧接着就会把它置False。
            new_mode = next(
                (p.value for p in params if p.name == 'servo_mode'), None)
            if not new_mode and self._relay_mode_active:
                self._set_relay_mode('normal')
            self._centered_notified = False
            self._arrived_notified = False
            self._landing_triggered = False
            self._landed_notified = False
            self._converge_since = None
            self._goto_converge_since = None
            self._descent_converge_since = None
            self._land_repeat_remaining = 0
            self._relay_mode_active = False
            self._control_start_time = None
        return SetParametersResult(successful=True)

    def _on_odom(self, msg: Odometry):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._odom_xy_z_yaw = (
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z, yaw,
        )

    def _on_detections(self, msg: Detection2DArray):
        if self._odom_xy_z_yaw is None:
            return
        # 2026-09-22：只认下视相机。像素偏移->地面偏移的换算只对朝下的相机
        # 成立，前视相机斜着看到同一个标签时算出来的偏移是错的，会把飞机
        # 带偏。两路相机的检测结果发在同一个话题上，靠frame_id区分。
        if '_camera_down_' not in msg.header.frame_id:
            return
        target_id = self.get_parameter('target_class_id').value
        match = None
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.class_id == target_id:
                match = det
                break
        if match is None:
            return

        image_w = int(self.get_parameter('image_width').value)
        image_h = int(self.get_parameter('image_height').value)
        hfov = float(self.get_parameter('camera_hfov_rad').value)
        _, _, z, yaw = self._odom_xy_z_yaw
        altitude_agl = max(z, 0.05)  # 避免altitude<=0传进换算公式报错(贴地边缘情况)

        dx_px = match.bbox.center.position.x - image_w / 2.0
        dy_px = match.bbox.center.position.y - image_h / 2.0
        body_x, body_y = pixel_offset_to_body_frame(
            dx_px, dy_px, altitude_agl, hfov, image_w, image_h,
            invert_x=bool(self.get_parameter('invert_x').value),
            invert_y=bool(self.get_parameter('invert_y').value),
            swap_xy=bool(self.get_parameter('swap_xy').value),
        )
        world_dx, world_dy = body_offset_to_world(body_x, body_y, yaw)

        self._last_world_offset = (world_dx, world_dy)
        self._last_detection_time = time.monotonic()

    def _on_control_tick(self):
        if self._odom_xy_z_yaw is None:
            return

        servo_mode = self.get_parameter('servo_mode').value
        if servo_mode not in VALID_SERVO_MODES:
            # 2026-09-20清理日志噪音：这里要分两种情况，原来混在一起按
            # ERROR每5秒刷一条，空闲时会持续刷屏（实测日志里满屏都是它）。
            #
            # ① 空字符串 = 刻意设计的"待命"状态，不是错误。这个节点是
            #    entrypoint每架飞机常驻拉起的，容器启动那一刻不可能知道该
            #    是center_only还是precision_land，就是要等SDK显式设置
            #    （见文件头"2026-09-13补充"+构造函数里那段warn）。待命是
            #    预期行为，只在第一次进入时补一条info，之后静默跳过。
            # ② 非空但不合法 = 真的设错了，保留ERROR。
            if not servo_mode:
                if not self._standby_logged:
                    self._standby_logged = True
                    self.get_logger().info(
                        'servo_mode尚未设置，节点保持待命、不发布precision_land_cmd'
                        '（等SDK的center_on_target()/precision_land_and_confirm()等'
                        '方法显式设置模式后自动生效，这是预期状态，不是故障）'
                    )
                return
            self.get_logger().error(
                f'servo_mode参数当前值{servo_mode!r}不合法（应为{VALID_SERVO_MODES}之一），'
                f'本周期跳过、不发布precision_land_cmd',
                throttle_duration_sec=5.0,
            )
            return
        # 拿到合法模式了：复位待命标志，这样将来模式被清回空字符串时，
        # 还会再提示一次"进入待命"，不会因为标志一直是True而静默掉。
        self._standby_logged = False

        # 2026-09-14新增：`coordinate_land`/`coordinate_goto`都不看
        # 视觉，没有"检测超时"这个概念——目标坐标是提前给定的常量，
        # 里程计只要在正常发布就随时能算出误差，不需要等一条检测消息。
        # `center_only`/`precision_land`两个视觉模式保留原有检测超时
        # 保护（见下面说明）。
        if servo_mode not in ('coordinate_land', 'coordinate_goto'):
            timeout = float(self.get_parameter('detection_timeout_s').value)
            if (
                self._last_detection_time is None
                or (time.monotonic() - self._last_detection_time) > timeout
            ):
                self.get_logger().warn(
                    '还没收到过目标tag检测结果，或检测已超时，本周期不发布precision_land_cmd（保持沉默比拿过期数据瞎修正更安全）',
                    throttle_duration_sec=2.0,
                )
                return

        # center_only模式一旦完成过一次"居中"广播，任务就算完成了——不再
        # 发布precision_land_cmd（清单A5节要求③"停止发布…把控制权交还给
        # position_cmd_relay当前其它模式"），本节点后续什么都不做。
        #
        # ⚠️ 2026-09-14实测踩坑：这两条"已经完成/已经触发过，什么都不用
        # 再做了"的早退检查，原来写在下面relay_mode接管块**之后**——
        # `_trigger_land()`触发降落的同一帧会把`_relay_mode_active`重置
        # 成`False`（是为了让"以后如果这个节点还要再做一次全新的精降"
        # 能重新正确接管），但下一个控制周期(100ms后)一进`_on_control_
        # tick`，先跑到的是接管块（因为它在早退检查之前），`_relay_mode_
        # active`是False，判断成"还没接管，得接管"，把relay_mode**又
        # 切回了'precision_land'**——直到这里的早退检查才把这一帧后续
        # 逻辑拦住，但relay_mode已经被错误地重新抢占，之后再也没有代码
        # 会把它切回来（`_trigger_land()`不会重复触发，见该方法开头的
        # `if self._landing_triggered: return`）。结果是：整套精降流程
        # 真正跑完一次之后，relay_mode永久卡在'precision_land'，后续
        # 任何`sdk.goto()`/编队跟随的指令全部被relay静默丢弃——第二次
        # E6完整验证时实测复现：精降+抓取+重新起飞全部成功，但重新起飞
        # 后飞回起降点的`goto()`死活不动（`ego_planner`卡在`WAIT_
        # TARGET`），查`ros2 param get .../position_cmd_relay relay_mode`
        # 确认还是`precision_land`。修复：把这两条早退检查挪到接管块
        # **之前**——已经完成/已经触发过的情况下，压根不应该再走到接管
        # 逻辑，不给"重新抢占relay_mode"这个错误路径任何被执行到的机会。
        if servo_mode == 'center_only' and self._centered_notified:
            return
        if servo_mode == 'coordinate_goto' and self._arrived_notified:
            return
        if servo_mode in ('precision_land', 'coordinate_land') and self._landing_triggered:
            if self._land_repeat_remaining > 0:
                self._publish_land_cmd()
                self._land_repeat_remaining -= 1
            return

        # 2026-09-14新增：确保position_cmd_relay已经切到'precision_land'
        # 才发布控制指令，见文件头同日期说明——`_relay_mode_active`保证
        # 这个service调用一次任务里只真正发生一次，不是每个控制周期都去
        # 调。`auto_switch_relay_mode=False`时（留给以后任务状态机自己
        # 接管这个决策）`_set_relay_mode()`直接返回False，这种情况不能
        # 当成"接管失败、跳过本周期"处理，否则整个节点会变成永远不发布
        # 指令——只有"开关打开但service调用真失败"才重试。
        if not self._relay_mode_active:
            auto_switch = bool(self.get_parameter('auto_switch_relay_mode').value)
            if not auto_switch:
                self._relay_mode_active = True  # 开关关闭：视为"不归本节点管"，直接放行
            elif self._set_relay_mode('precision_land'):
                self._relay_mode_active = True
                self.get_logger().info('position_cmd_relay已自动切到precision_land模式，开始接管飞控')
            else:
                return  # 开关开着但这次service调用真失败了，本周期先不发指令，下个周期重试
            self._control_start_time = time.monotonic()  # 硬超时兜底计时起点，见__init__/文件头说明

        x, y, z, yaw = self._odom_xy_z_yaw
        if servo_mode in ('coordinate_land', 'coordinate_goto'):
            # 2026-09-14新增：直接拿目标坐标跟当前里程计位置作差，不经过
            # 视觉/像素偏移这一整套换算——见VALID_SERVO_MODES声明处说明。
            target_x_param = float(self.get_parameter('target_x').value)
            target_y_param = float(self.get_parameter('target_y').value)
            world_dx = target_x_param - x
            world_dy = target_y_param - y
        else:
            world_dx, world_dy = self._last_world_offset
        kp = float(self.get_parameter('kp_xy').value)
        step_x = kp * world_dx
        step_y = kp * world_dy
        if servo_mode in ('coordinate_land', 'coordinate_goto'):
            max_speed = float(self.get_parameter('coordinate_max_speed_mps').value)
            rate_hz = float(self.get_parameter('control_rate_hz').value)
            max_step = max_speed / rate_hz if rate_hz > 0.0 else max_speed
            step_norm = math.hypot(step_x, step_y)
            if step_norm > max_step and step_norm > 1e-9:
                scale = max_step / step_norm
                step_x *= scale
                step_y *= scale
        target_x = x + step_x
        target_y = y + step_y

        horizontal_error = math.hypot(world_dx, world_dy)
        threshold = float(self.get_parameter('horizontal_converge_threshold_m').value)

        if servo_mode == 'center_only':
            target_z = z  # 仅居中模式永远不下降，见文件头"2026-09-13补充"说明
            if horizontal_error <= threshold:
                now = time.monotonic()
                if self._converge_since is None:
                    self._converge_since = now
                hold_s = float(self.get_parameter('converge_hold_duration_s').value)
                if now - self._converge_since >= hold_s:
                    self._publish_centered_pose(x, y)
                    self._publish_servo_status('centered')
                    self._centered_notified = True
                    # 2026-09-14新增：交还控制权的同时把relay_mode切回
                    # 'normal'，见文件头同日期说明——谁接管的谁负责交还，
                    # 不留在'precision_land'模式里让下一次goto()/编队
                    # 跟随的指令被silently丢弃。auto_switch关闭时本来就
                    # 没有接管过，不需要交还。
                    if bool(self.get_parameter('auto_switch_relay_mode').value):
                        self._set_relay_mode('normal')
                    self._relay_mode_active = False
                    self.get_logger().info(
                        f'水平误差{horizontal_error:.3f}m已持续收敛超过'
                        f'converge_hold_duration_s={hold_s}s，判定为已居中，'
                        f'广播centered_pose=({x:.3f},{y:.3f})+'
                        f"servo_status='centered'，本节点停止发布"
                        f'precision_land_cmd（交还控制权，relay_mode切回normal）'
                    )
                    return  # 这一帧不再发布precision_land_cmd
            else:
                self._converge_since = None  # 中途又超出阈值，"持续收敛"计时清零重来
        elif servo_mode == 'coordinate_goto':
            # 2026-09-14新增：不避障、直达版goto，见VALID_SERVO_MODES
            # 声明处说明——三维同时朝目标坐标收敛，不像`coordinate_land`
            # 那样先收敛水平再分级下降（没有"地效强、要交给AUTO_LAND"
            # 这个顾虑）。到点后广播`servo_status='arrived'`+交还控制权，
            # 不触发降落。
            #
            # ⚠️ 2026-09-14实测发现：这套P控制（`target = x + kp*(目标-x)`）
            # 是为`precision_land`/`coordinate_land`那种"已经很接近目标、
            # 做最后一段精修"场景调的，直接拿来跑一段~7米的长距离实测
            # 收敛非常慢（60秒还没到）——跟`goto()`背后`ego_planner`那种
            # 直接规划到目标的方式完全不是一回事，不能拿这个方法替代
            # `goto()`跑长距离。复用跟precision_land同一个硬超时兜底
            # (`precision_land_max_wait_s`)：超时后不再等三维误差收敛，
            # 按"当前位置已经是能做到的最好结果"直接广播`arrived`——比
            # 让飞机无限期悬停等一个可能收敛很慢的过程更安全，但这不是
            # "解决"了慢收敛这个问题，只是加了个安全网；真正的解决办法
            # 是用法上只把这个方法用于短距离最后一段修正，长距离还是用
            # `goto()`。
            max_wait = float(self.get_parameter('coordinate_goto_max_wait_s').value)
            timed_out = (
                max_wait > 0.0
                and self._control_start_time is not None
                and time.monotonic() - self._control_start_time > max_wait
            )
            if timed_out:
                self.get_logger().warn(
                    f'直飞已等待超过coordinate_goto_max_wait_s={max_wait}s，'
                    f'仍未收敛到目标点——不再等待，按当前位置广播已到点',
                )
                self._publish_servo_status('arrived')
                self._arrived_notified = True
                if bool(self.get_parameter('auto_switch_relay_mode').value):
                    self._set_relay_mode('normal')
                self._relay_mode_active = False
                return

            target_z_param = float(self.get_parameter('target_z').value)
            world_dz = target_z_param - z
            kp_z = kp  # 复用kp_xy，垂直方向没有单独调过一份增益的必要
            target_z = z + kp_z * world_dz

            arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
            total_error = math.hypot(horizontal_error, world_dz)
            if total_error <= arrival_threshold:
                now = time.monotonic()
                if self._goto_converge_since is None:
                    self._goto_converge_since = now
                hold_s = float(self.get_parameter('converge_hold_duration_s').value)
                if now - self._goto_converge_since >= hold_s:
                    self._publish_servo_status('arrived')
                    self._arrived_notified = True
                    # 交还控制权，同center_only/precision_land那几处
                    # 同款注释——谁接管的谁负责交还。
                    if bool(self.get_parameter('auto_switch_relay_mode').value):
                        self._set_relay_mode('normal')
                    self._relay_mode_active = False
                    self.get_logger().info(
                        f'三维误差{total_error:.3f}m已持续收敛超过'
                        f'converge_hold_duration_s={hold_s}s，判定为已到点'
                        f'({target_x_param:.3f},{target_y_param:.3f},{target_z_param:.3f})，'
                        f"广播servo_status='arrived'，本节点停止发布"
                        f'precision_land_cmd（交还控制权，relay_mode切回normal）'
                    )
                    return  # 这一帧不再发布precision_land_cmd
            else:
                self._goto_converge_since = None  # 中途又超出阈值，"持续收敛"计时清零重来
        else:  # servo_mode == 'precision_land' or 'coordinate_land'——分级下降+AUTO_LAND交接逻辑完全共用
            # 2026-09-14新增：硬超时兜底，见文件头/__init__同日期说明——
            # 用户直接指出"是一个时间限制，超过60秒不管如何都降落"。超时
            # 后不再管当前对没对准、降到第几级，直接触发`_trigger_land()`
            # 交给可靠的`AUTO_LAND`处理剩下的一切，不再自己发位置指令。
            max_wait = float(self.get_parameter('precision_land_max_wait_s').value)
            timed_out = (
                self._control_start_time is not None
                and time.monotonic() - self._control_start_time > max_wait
            )
            if timed_out:
                self.get_logger().warn(
                    f'精降已等待超过precision_land_max_wait_s={max_wait}s，'
                    f'水平误差{horizontal_error:.3f}m/当前高度{z:.3f}m——'
                    f'不再等待，直接触发降落',
                )
                self._trigger_land()
                return

            # 2026-09-14新增：分级下降，见文件头/land_handoff_height_m
            # 声明处的完整说明——不再连续匀速下压到很低高度，改成"对准
            # 一点、下降一点、再对准、再下降"，降到land_handoff_height_m
            # 附近且这一级也对准了，就不再自己往下压，直接交给AUTO_LAND。
            handoff_height = float(self.get_parameter('land_handoff_height_m').value)
            if horizontal_error > threshold:
                target_z = z  # 水平误差还没收敛，先只修水平，不下降
                self._descent_converge_since = None  # 中途又超出阈值，"持续对准"计时清零重来
            else:
                now = time.monotonic()
                if self._descent_converge_since is None:
                    self._descent_converge_since = now
                hold_s = float(self.get_parameter('converge_hold_duration_s').value)
                if now - self._descent_converge_since < hold_s:
                    target_z = z  # 还在确认"是不是真的持续对准"，先不动高度
                elif z <= handoff_height + _DESCENT_ARRIVED_TOLERANCE_M:
                    # 已经对准+已经降到交接高度附近：不再自己往下压（这个
                    # 高度区间地效强，位置指令追踪本身也不精确，见文件头
                    # 说明），直接触发真正降落交给AUTO_LAND。
                    target_z = z
                    self._trigger_land()
                else:
                    # 对准了、也还没到交接高度：下降一级，然后要求在新
                    # 高度重新持续对准满converge_hold_duration_s才能再往
                    # 下一级（不是每个控制周期都下降，避免刚下降一点点
                    # 扰动了水平对准还继续往下冲）。
                    step = float(self.get_parameter('descent_step_m').value)
                    target_z = max(z - step, handoff_height)
                    self._descent_converge_since = None

        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.position.x = target_x
        cmd.position.y = target_y
        cmd.position.z = target_z
        cmd.yaw = yaw  # 精降阶段保持当前朝向，不额外转向
        cmd.yaw_dot = 0.0
        # 2026-09-15新增：`coordinate_land`/`coordinate_goto`长距离飞行
        # 时实测发现——不管`coordinate_max_speed_mps`设0.3/0.5/1.0，实际
        # 速度死死卡在约0.02m/s不变，跟设定值毫无关系。查了`pt4ctrl`源码
        # （PX4CtrlFSM.cpp::publish_trajectory_setpoint()）才搞清楚：
        # `cmd.velocity`这个字段一直是消息默认的全零，从来没被这个节点
        # 填过；pt4ctrl把position/velocity/acceleration一起原样转发给
        # PX4自己的位置控制环（`mavros/setpoint_trajectory/local`），
        # PX4那边是按"位置给的是当前该到的点、速度是这一点上的前馈速度"
        # 这套轨迹跟踪逻辑设计的——一直发速度=0，等于每个周期都在告诉
        # PX4"这个点应该是静止悬停的"，纯位置误差项(P)在这种"目标点几乎
        # 贴着当前位置"的用法下（`coordinate_max_speed_mps`限速本来就是
        # 让目标点只比当前位置领先一点点）算出来的响应天然很保守，跟我们
        # 在这条消息之外单独限速多少没有关系——限的是目标点领先多少，
        # 不是PX4实际会用多快去追。这里把"这一步本来打算移动多快"
        # (`step_x`/`step_y`除以控制周期时长)当前馈速度一起发出去，PX4
        # 才有"这个点不是要停在这，是正在移动"这个信息，能真正按预期
        # 速度跟踪而不是每个周期都当成一次新的"精确停在这一点"来收敛。
        # 只加在`coordinate_land`/`coordinate_goto`（本来就没有速度前馈、
        # 且是本次问题的现场）——`center_only`/`precision_land`那套视觉
        # 伺服短距离场景一直工作正常，不动它，避免引入不必要的回归风险。
        if servo_mode in ('coordinate_land', 'coordinate_goto'):
            rate_hz = float(self.get_parameter('control_rate_hz').value)
            if rate_hz > 0.0:
                cmd.velocity.x = step_x * rate_hz
                cmd.velocity.y = step_y * rate_hz
        self.pub.publish(cmd)

    def _publish_centered_pose(self, x: float, y: float):
        """center_only模式收敛完成时，把当前坐标x/y广播出去——给RECON
        角色用，命中地面火情标记后收敛完成时读到的x/y就近似等于地面
        火情的坐标（理由见方案0.2节，不做单目位姿解算）。z字段故意填
        0.0，不填当前悬停高度——这个话题只承载"水平坐标"这一个语义，
        塞一个跟"地面火情坐标"无关的悬停高度进去反而容易误导消费方
        （比如contest_sdk的center_on_target()）。

        ⚠️ **坐标系（2026-09-13订正）**：`x/y`直接来自`self._odom_xy_
        z_yaw`，是**这架飞机自己的局部坐标系**（`frame_id='map'`，跟
        `{ns}/odom`恒等），不是跨机共享的world系——这份文档原来写"世界
        坐标"是措辞错误，实现从来没变过。`contest_sdk::center_on_
        target()`把这个值原样转发给选手，选手要跨机广播前需要自己调
        `sdk.local_to_world()`换算。
        """
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.centered_pose_pub.publish(msg)

    def _set_relay_mode(self, mode: str) -> bool:
        """调position_cmd_relay的标准~/set_parameters service切relay_mode。

        2026-09-14新增，最初照抄`fire_pillar_aim_node.py`的同名方法（用
        `rclpy.spin_until_future_complete()`阻塞等结果），**当场实测直接
        把整个节点锁死**——这个节点的`_on_control_tick`是10Hz定时器回调，
        跟`fire_pillar_aim_node.py`那次调用点（只触发一次的subscription
        回调`_on_detections`）不是同一种场景：`spin_until_future_
        complete()`内部会再次进入这个节点的spin逻辑，在已经由外层
        `MultiThreadedExecutor`持续高频调度这同一个节点的场景下嵌套
        再spin，实测复现"开始接管飞控"那条日志打完之后，这个节点**彻底
        停止响应**，连`ros2 param get`这种最基础的service查询都超时——
        不是短暂卡顿，是永久性死锁，用户实测直接发现"NX02还悬着"、
        `ros2 param get`确认卡死后紧急`force disarm`才救回来。
        `fire_pillar_aim_node.py`那个调用点为什么没暴露这个问题，没有
        深挖，但两边场景差异明显（一次性触发vs.持续10Hz定时器），不能
        假设那边"验证过所以这边照抄也安全"。

        改成完全不阻塞的`call_async()`+`add_done_callback()`——发出请求
        就立刻返回（不等结果），成功/失败靠`_on_relay_mode_set_done()`
        异步回调打日志，不在调用方（`_on_control_tick`）这条执行路径上
        做任何等待。代价是调用方拿不到"确实切换成功了"这个同步确认，
        只能确认"请求已经发出去"——可接受：极端情况下头一两个控制周期
        (100ms量级)的`precision_land_cmd`可能在relay_mode真正切换生效
        之前就发出去、被relay按旧模式丢弃，比之前"整个节点死锁、飞机
        永远悬在半空"这个后果轻得多。
        """
        if not bool(self.get_parameter('auto_switch_relay_mode').value):
            return False
        if not self.relay_set_params_cli.service_is_ready():
            self.get_logger().error(
                f'{self.relay_set_params_cli.srv_name}服务不可用，无法把relay_mode切到{mode}——'
                f'检查relay_set_parameters_service参数是否对应真实的position_cmd_relay节点名'
            )
            return False

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'relay_mode'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode)
        req.parameters = [param]

        future = self.relay_set_params_cli.call_async(req)
        future.add_done_callback(lambda f, mode=mode: self._on_relay_mode_set_done(f, mode))
        return True  # 只代表"请求已发出"，不是"已确认切换成功"，见上面docstring

    def _on_relay_mode_set_done(self, future, mode: str) -> None:
        """`_set_relay_mode()`的异步结果回调，纯打日志，不影响控制逻辑
        （控制逻辑已经在`_set_relay_mode()`发出请求那一刻就乐观地继续
        往下走了，见该方法docstring）。"""
        exc = future.exception()
        if exc is not None:
            self.get_logger().error(f'切换relay_mode到{mode}的service调用异常: {exc!r}')
            return
        result = future.result()
        if result is None or not result.results:
            self.get_logger().error(f'切换relay_mode到{mode}没有收到有效响应')
            return
        if not result.results[0].successful:
            self.get_logger().error(f'切换relay_mode到{mode}被拒绝: {result.results[0].reason}')

    def _publish_servo_status(self, status: str):
        """跟actuator_action_node.py的_publish_status()同款风格：一次性
        状态字符串广播，方便SDK轮询（跟本节点内部控制逻辑没有耦合，纯粹
        是对外的完成通知）。
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_land_cmd(self):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.LAND
        self._land_pub.publish(msg)

    def _trigger_land(self):
        """精降抓取模式分级下降到land_handoff_height_m附近（或者硬超时
        兜底触发）后，触发真正的降落。

        复用`px4ctrl_bridge/takeoff_gate.py`起飞时用的同一个接口——
        `quadrotor_msgs/msg/TakeoffLand{takeoff_land_cmd: LAND}`，
        PX4CtrlFSM.cpp收到这条消息后进入AUTO_LAND状态；`gcs/backend/
        app.py`的`_publish_takeoff_land()`（GCS网页"降落"按钮/
        `sdk.land()`最终调用链路）底层调的也是同一个消息+同一个话题
        （相对话题名`takeoff_land`，靠launch层的namespace区分双机），
        这里直接复用，不新造一套降落机制。

        触发之后本节点**不再继续发布`precision_land_cmd`**（见
        `_on_control_tick`顶部对`_landing_triggered`的分支处理）——
        飞行控制权已经交给px4ctrl自己的AUTO_LAND状态机，这个节点继续
        按P控制发位置指令只会跟AUTO_LAND状态机的下降/disarm时机打架，
        这跟center_only模式收敛后"停止发布、交还控制权"是同一个设计
        原则的两次应用。
        """
        if self._landing_triggered:
            return  # 已经触发过，不重复触发（正常路径下走不到这里，见调用处的分支保护）
        self._landing_triggered = True
        self._land_repeat_remaining = _LAND_CMD_REPEAT - 1  # 这一帧先发一次，下面再补发剩余次数
        self._publish_land_cmd()
        # 2026-09-14新增：控制权交给AUTO_LAND状态机的同时把relay_mode切回
        # 'normal'，见文件头同日期说明+_publish_centered_pose()调用处同款
        # 注释——谁接管的谁负责交还。
        if bool(self.get_parameter('auto_switch_relay_mode').value):
            self._set_relay_mode('normal')
        self._relay_mode_active = False
        self.get_logger().info(
            '已触发TakeoffLand(LAND)真正降落（水平对准+分级下降到交接高度，'
            '或者硬超时兜底），本节点停止发布precision_land_cmd'
            '（relay_mode切回normal），等待mavros/state的armed变false确认真正落地'
        )

    def _on_mavros_state(self, msg: State):
        """precision_land模式触发降落之后，靠armed变false确认"真正落地"，
        不能只信自己发过LAND指令就假装已经落地——这个项目里已经反复踩过
        "指令发出去≠指令被执行了"的坑（见gcs/backend/app.py
        `_recent_px4ctrl_feedback()`那段说明），这里换成本节点常驻订阅
        mavros/state直接确认，比读docker logs更直接可靠。用
        `_landing_triggered`做前置条件，避免起飞前armed本来就是false
        的正常状态被误判成"落地"。
        """
        if self._landing_triggered and not self._landed_notified and not msg.armed:
            self._landed_notified = True
            self._publish_servo_status('landed')
            self.get_logger().info(
                "mavros/state确认armed=false，判定为已真正落地，广播"
                "servo_status='landed'"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionServoNode()
    # 2026-09-14改成MultiThreadedExecutor，配合__init__里client_cb_group
    # 的用意——见文件头同日期"死锁教训"说明：单线程executor在"回调里
    # 再调其它service"的场景下会死锁，不是这里可以随便省掉的细节。跟
    # fire_pillar_aim_node.py的main()同一套改法。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
