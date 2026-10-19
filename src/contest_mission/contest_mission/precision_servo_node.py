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
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray

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
VALID_SERVO_MODES = ('center_only', 'precision_land', 'coordinate_land',
                     'coordinate_goto', 'track_target', 'track_land',
                     'aim_front')

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
        # 2026-10-08：上面那个阈值从"三维球半径"改成**水平**阈值，Z 单独一个。
        #   起因：高层火情轮任务机 release_at(direct=True) 回物资点放器材，在点
        #   上空盘旋很久不降。判据原来是 hypot(水平误差, dz) <= 0.3 —— 一个球，
        #   Z 吃掉多少预算，水平就少多少。那趟定高全程钉在 1.87 m 而请求的是
        #   2.0 m，光高度差就占掉 0.13，水平只剩 0.27，再叠上 UWB 噪声和悬停
        #   漂移就长时间够不到，而 goto_direct(timeout=None) 和
        #   coordinate_goto_max_wait_s=0.0 两端都不超时，于是一直等。
        #   ⚠ 为什么不干脆去掉 Z 只看水平：set_agl() 走的就是 coordinate_goto
        #   （goto_direct(cx, cy, lz)，水平不动只改高度），判据里没有 Z 的话
        #   第一个控制周期就会广播 arrived，高度一厘米没动就返回——patrol() 每
        #   站查完都调一次 set_agl()，全会变成空动作。所以 Z 必须留着把关，
        #   只是跟水平分开算。
        #   两个都填 0.3 相对原来是**纯放宽**（球 -> 盒，对角放到 0.52），
        #   现在能到点的场景不会退化。
        self.declare_parameter('arrival_z_threshold_m', 0.3)
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
        # [m/s] 2026-10-13 新增：**垂直**方向的速率上限。
        # ⚠️ 原来 z 这一路完全是裸的：`target_z = z + kp_z * world_dz`，
        # `coordinate_max_speed_mps` 的夹限只作用在 step_x/step_y（那段 hypot
        # 只含水平两轴），而且 `cmd.velocity.z` 从来没被赋过值。后果是
        # 一次 2 m 的高度变化，第一拍高度设定点就跳 1 m——垂直速度完全靠
        # PX4 自己的 MPC_Z_VEL_MAX 兜着，这个节点一点都没管。
        # `set_agl()` 走的就是 coordinate_goto，是纯 z 运动，这条路最吃亏。
        # 单独一个参数、不复用水平那个：`_direct_leg()` 会把水平限速临时提到
        # 1.0 m/s 跑航段，垂直速率不该跟着一起变。
        # 取 0.6，跟 .env 的 EGO_ALT_RATE_MAX / LINE_ALT_RATE_MPS 同一个口径。
        # ---- aim_front：机载前视瞄准（2026-10-14 用户要求搬到机上）----
        # 为什么必须在机上：这是"看一眼动一下"的闭环，跟 track_target 同一类。
        # 放地面侧循环的话每轮都要跨链路取检测、下指令、轮询到位——用户
        # 2026-10-06 就为 track_target 定过这条口径，这次先写成地面版是我的疏漏。
        # 跟 track_target 的区别：那个靠 target_locate_node 解出**地面坐标**再飞
        # 过去（只对朝下相机成立）；这个只用**方位角+俯仰角**，对任何朝向的相机
        # 都成立，所以不走像素->地面偏移那套换算，也因此必须单独一条检测路径。
        self.declare_parameter('aim_timeout_s', 15.0)
        self.declare_parameter('aim_yaw_tol_deg', 2.0)     # 方位容差
        self.declare_parameter('aim_z_tol_m', 0.12)        # 高度容差
        self.declare_parameter('aim_yaw_gain', 0.8)        # <1 防过冲
        self.declare_parameter('aim_scan_deg', 60.0)       # 扫描总张角
        self.declare_parameter('aim_scan_rate_dps', 12.0)  # 扫描角速度（缓慢）
        self.declare_parameter('aim_tag_size_m', 0.5)      # 按框宽估距离用
        self.declare_parameter('aim_z_min_m', 0.8)
        self.declare_parameter('aim_z_max_m', 4.0)
        self.declare_parameter('aim_det_stale_s', 0.5)     # 检测新鲜度
        self.declare_parameter('coordinate_max_z_speed_mps', 0.6)
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
        # ---- track_target 模式（2026-10-06 用户要求）----
        # 做法：目标坐标由 target_locate_node 解算（内参+位姿+测距仪，不碰
        # pixel_offset_to_body_frame 那套配置），本节点把飞机一轮轮送到那个
        # 坐标正上方；像素误差只当收敛判据。闭环整个留在机上，地面只发起+等结果。
        self.declare_parameter('center_tol_px', 50.0)    # 目标框中心离图像主点多少像素算到位
        self.declare_parameter('track_timeout_s', 30.0)  # 总时限；超时**按成功处理**，见下
        self.declare_parameter('track_agl_m', 0.0)       # 逼近时保持的高度，<=0 表示保持当前高度
        # ---- track_land 模式（2026-10-06 用户要求：land_on 下沉到机载）----
        # 边按解算坐标对准边下降，水平和垂直在同一条指令里——不是"对准一点、降
        # 一点、再对准"的分级下降（那套从 2.5 m 下来要 40 s 以上，30 s 时限内走
        # 不完，每次都走超时兜底，等于精度只做了一半）。默认值取自地面版 land_on
        # 的 LAND_ON_* 常量，逐个对应。
        self.declare_parameter('land_step_m', 0.8)           # 每步降多少，降完重新解算
        self.declare_parameter('land_handoff_m', 0.7)        # 降到离地这么高交给 AUTO_LAND
        self.declare_parameter('land_handoff_tol_m', 0.15)   # 悬停有起伏，死等"严格低于"会卡住
        self.declare_parameter('track_land_timeout_s', 30.0) # 总时限，到点直接交给 AUTO_LAND
        self.declare_parameter('track_lost_grace_s', 5.0)    # 丢目标超过这么久才放弃，用最后坐标撑住
        # ⚠️ 解算坐标的保质期（2026-10-06 实测事故）：缓存没有时效检查时，apriltag:0
        # 在飞机还贴地时被看到过的那份结果会一直有效——地面 z 跟飞机 z 几乎相等，于是
        # 飞到物资点上空一进 track_land 就算出 agl≈0.04 m，第一拍判"已到交接高度"直接
        # 交给 AUTO_LAND，视觉精降一步没做。超过这个岁数的坐标只能用来"飞回去找"，
        # 不能用来算 agl。
        self.declare_parameter('track_cache_max_age_s', 3.0)
        self.declare_parameter('control_rate_hz', 10.0)
        # 三个轴映射修正旋钮，见文件头说明。
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('swap_xy', False)

        # 2026-09-14新增：自动接管position_cmd_relay的relay_mode，见文件头
        # 同日期说明——照抄fire_pillar_aim_node.py的auto_switch_relay_mode/
        # relay_set_parameters_service两个参数，同一套开关。
        self.declare_parameter('auto_switch_relay_mode', True)
        # 2026-10-05新增：接管期间**周期性重申** relay_mode 的间隔（秒，<=0 关闭）。
        # 为什么需要：原来只在首次接管时切一次（见 _relay_mode_active），之后整个
        # servo_mode 会话内永不重申，于是别的节点一旦把 relay_mode 改回去，本节点
        # 完全察觉不到，继续往 precision_land_cmd 发指令而中继全丢，飞机表现为
        # "收到指令却不动"。10-05 11:36 formation_lite 实测事故：僚机出列时
        # formation_follower_node 迟到的 relay_mode='normal' 顶掉了本节点的接管，
        # 回家那段根本没被伺服、靠 AUTO_HOVER 漂了 37.6 秒才漂进到点球里
        # （长机同样 1.0m/s 直飞只用 1.0 秒收住，对照组）。
        # 重申是幂等的，而且 _set_relay_mode() 是非阻塞 call_async，代价可忽略；
        # _on_relay_mode_set_done() 本来只在失败时打日志，所以重申不会刷屏。
        self.declare_parameter('relay_reassert_period_s', 1.0)
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

        # === aim_front 模式专用状态 ===
        self._aim_dyaw: Optional[float] = None    # 方位角（rad，>0=目标在画面右）
        self._aim_dz: Optional[float] = None      # 高度差（m，>0=要升）
        self._aim_dist: Optional[float] = None    # 按框宽估的距离（m）
        self._aim_rx: Optional[float] = None      # 最近一帧前视检测的时刻
        self._aim_done = False
        self._aim_scan_center: Optional[float] = None   # 接管时的朝向，扫描以它为中心
        self._aim_yaw_cmd: Optional[float] = None       # 本拍要下发的 yaw（覆写 cmd.yaw）
        self._aim_target_z: float = 0.0                 # 本拍目标高度（所有分支都必须有定义）
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
        # 上一次（重）申 relay_mode 的时刻，配合 relay_reassert_period_s 使用
        self._relay_last_assert_t: Optional[float] = None
        # 2026-09-14新增：precision_land_max_wait_s硬超时兜底计时起点——
        # 跟_relay_mode_active在同一个时刻(真正接管飞控那一刻)一起置位，
        # 见_on_control_tick里的用法。
        self._control_start_time: Optional[float] = None

        self.pub = self.create_publisher(PositionCommand, 'precision_land_cmd', 10)
        self.create_subscription(Odometry, 'dlio/odom_node/odom', self._on_odom, 10)
        self.create_subscription(Detection2DArray, 'vision/detections', self._on_detections, 10)
        # track_target 用：解算坐标（target_locate_node 的输出，frame_id 是本机
        # odom 系，跟 self._odom_xy_z_yaw 同一个系）和图像主点（camera_info）。
        # class_id -> (x, y, z, 收到时刻)，见 _on_target_positions 的说明
        self._track_cache: dict = {}
        self._track_target_z: float = 0.0   # track_* 这一拍要去的高度，见控制回路
        self._track_xy: Optional[Tuple[float, float]] = None
        self._track_z: float = 0.0
        self._track_seen_t: Optional[float] = None
        self._track_px: Optional[float] = None
        self._track_done = False
        self._principal: Optional[Tuple[float, float]] = None
        self._fx: Optional[float] = None       # camera_info 的 K[0]，aim_front 用
        self.create_subscription(Detection3DArray, 'vision/target_positions',
                                 self._on_target_positions, 10)
        _ns = self.get_namespace().strip('/') or 'NX01'
        # 2026-10-14：**两路都订**。原来只订 _down_ 那一路，于是 aim_front
        # （前视）永远拿不到内参、每次都走回退——真机实测后果：fx 从真值 1200.7
        # 掉到回退的 381.5（差 3.15 倍），主点从 646.3 掉到 320（差 326 px，
        # 在真 fx 下相当于 15.2°）。于是节点报"方位 +1.4° 已锁定"，实际对着偏
        # 15° 的方向，解出的距离也只有真实值的 1/3.15（日志 1.32 m 实为 4.15 m）。
        # 现象就是用户说的"有瞄准动作，但是没有真瞄准"。
        # 内参由机载 control_server.py 的 start_vision_mode() 按标定值发，话题名
        # {ns}_{label}_camera/camera_info，label 取自 _CAM_DEFAULTS：
        # cam0->front、cam1->down。两路主点差是装配决定的，必须分别存、不能互套。
        self._caminfo_front: Optional[Tuple[float, float, float]] = None
        self._caminfo_down: Optional[Tuple[float, float, float]] = None
        self.create_subscription(
            CameraInfo, f'{_ns}_down_camera/camera_info',
            lambda m: self._on_caminfo(m, 'down'), qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, f'{_ns}_front_camera/camera_info',
            lambda m: self._on_caminfo(m, 'front'), qos_profile_sensor_data)

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
            self._track_done = False
            # 2026-10-14：aim_front 那套也必须在这儿复位。漏掉的后果：第一次
            # 瞄准锁定后 _aim_done 永久为 True，第二次进 aim_front 直接撞上
            # 控制循环开头的 `if aim_front and _aim_done: return`，机载侧一行
            # 日志都不打，地面侧等满 预算+10s 报"机载节点没在跑？"。
            # 实测现象：侦查机发现点瞄准成功(2.2s 锁定)，发射前复瞄却必然超时。
            # 这类"闩锁没复位"的错当天已经踩过一次（own_pad 被二次起飞改写），
            # 加新模式时要连带检查**所有**同族状态的复位点。
            self._aim_done = False
            self._aim_dyaw = None
            self._aim_dz = None
            self._aim_dist = None
            self._aim_rx = None
            self._aim_scan_center = None
            self._aim_yaw_cmd = None
            self._track_px = None
            self._track_xy = None
            self._track_seen_t = None
            self._landing_triggered = False
            self._landed_notified = False
            self._converge_since = None
            self._goto_converge_since = None
            self._descent_converge_since = None
            self._land_repeat_remaining = 0
            self._relay_mode_active = False
            self._relay_last_assert_t = None
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
        is_down = '_camera_down_' in msg.header.frame_id
        # aim_front 走**独立**的前视解算：只要方位角和俯仰角，不做像素->地面
        # 偏移那套换算（那套只对朝下相机成立），所以必须在这里分叉。两路相机
        # 互斥、同一时刻只有一路在跑，所以按"不是下视帧就当前视帧"收，比写死
        # frame_id 字面量更耐名字变化。
        if str(self.get_parameter('servo_mode').value) == 'aim_front':
            if is_down:
                return
            self._on_detections_front(msg)
            return
        # 2026-09-22：其余模式只认下视相机。像素偏移->地面偏移的换算只对朝下的
        # 相机成立，前视相机斜着看到同一个标签时算出来的偏移是错的，会把飞机
        # 带偏。两路相机的检测结果发在同一个话题上，靠frame_id区分。
        if not is_down:
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
        # track_target 的收敛判据：目标框中心离图像主点多少像素。用主点而不是
        # image_width/2 —— 那两个参数真机上本来就配错了（640x480 vs 实际 1280x720）。
        if self._principal is not None:
            pcx, pcy = self._principal
            self._track_px = math.hypot(
                match.bbox.center.position.x - pcx, match.bbox.center.position.y - pcy)

    def _on_detections_front(self, msg: Detection2DArray):
        """前视瞄准解算：像素偏移 -> 方位角 / 俯仰角 / 距离。

        跟下视那条链的本质区别：不需要"目标在地面哪个坐标"，只需要"目标偏机头
        多少度、偏视线多少度"——这两个量对任何朝向的相机都成立，而且不依赖离地
        高度（下视那套要拿 altitude 去投影）。弹道沿机头出去，所以方位角直接
        驱动 yaw、俯仰角换算成高度差驱动升降。
        """
        target_id = self.get_parameter('target_class_id').value
        match = None
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.class_id == target_id:
                match = det
                break
        if match is None:
            return
        # 优先用真内参（camera_info 的 K）。回退是最后手段——真机实测
        # image_width/height 本来就配错过（640x480 vs 实际 1280x720）。
        if self._caminfo_front is not None:
            fx, cx, cy = self._caminfo_front
        else:
            cx = float(self.get_parameter('image_width').value) / 2.0
            cy = float(self.get_parameter('image_height').value) / 2.0
            hfov = float(self.get_parameter('camera_hfov_rad').value)
            fx = (cx / math.tan(hfov / 2.0)) if hfov > 0.0 else 1.0
            self.get_logger().warn(
                'aim_front 没拿到 camera_info，回退用 image_width/hfov 估内参'
                '——方位角和距离都会偏，优先把 camera_info 发上来',
                throttle_duration_sec=10.0)
        tag = float(self.get_parameter('aim_tag_size_m').value)
        bw = max(1.0, float(match.bbox.size_x))
        dist = fx * tag / bw
        self._aim_dyaw = math.atan2(match.bbox.center.position.x - cx, fx)
        dpitch = math.atan2(match.bbox.center.position.y - cy, fx)
        self._aim_dz = -dist * math.tan(dpitch)   # 目标在画面下方 -> 要降高度
        self._aim_dist = dist
        self._aim_rx = time.monotonic()

    def _on_caminfo(self, msg: CameraInfo, cam: str = 'down'):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]，主点就是"图像中心"的准确定义
        # ⚠️ CameraInfo.k 是 9 元素数组，不能直接 if msg.k（numpy 下抛
        # "truth value of an array ... is ambiguous"，节点收到第一帧就崩）
        if len(msg.k) >= 6 and float(msg.k[2]) > 0.0 and float(msg.k[5]) > 0.0:
            cxcy = (float(msg.k[2]), float(msg.k[5]))
            fx = float(msg.k[0]) if float(msg.k[0]) > 0.0 else None
            if cam == 'front':
                if fx is not None:
                    self._caminfo_front = (fx, cxcy[0], cxcy[1])
                return
            self._caminfo_down = ((fx if fx is not None else 0.0), cxcy[0], cxcy[1])
            # 下视那条链（track_target 的像素判据）一直用这两个字段，保持不变。
            self._principal = cxcy
            if fx is not None:
                self._fx = fx

    def _on_target_positions(self, msg: Detection3DArray):
        # ⚠️ 2026-10-06 修：原来只存"当前 target_class_id 匹配"的那一条，结果
        # target_class_id 是 SDK 调 track_* 那一刻才设的（之前一直是默认
        # apriltag:0），于是**设模式之前解算到的坐标全被丢掉**；设完模式时目标
        # 要是已经出了画面，target_locate_node 就不再发它，_track_xy 永远是
        # None，机载只能原地干等——实测"识别到了却没有瞄准动作"就是这个。
        # 改成按类别全缓存，谁当目标就取谁的最近一条，坐标跟踪才真的"丢不了"。
        now_m = time.monotonic()
        for det in msg.detections:
            if det.results:
                pos = det.results[0].pose.pose.position
                self._track_cache[det.results[0].hypothesis.class_id] = (
                    float(pos.x), float(pos.y), float(pos.z), now_m)
        self._refresh_track_from_cache()

    def _refresh_track_from_cache(self):
        """把当前 target_class_id 的最近一条解算坐标取出来给控制回路用。"""
        hit = self._track_cache.get(self.get_parameter('target_class_id').value)
        if hit is not None:
            self._track_xy = (hit[0], hit[1])
            self._track_z = hit[2]
            self._track_seen_t = hit[3]

    def _track_is_fresh(self) -> bool:
        """手上的解算坐标够不够新，能不能拿来算 agl。见 track_cache_max_age_s。"""
        if self._track_seen_t is None:
            return False
        return (time.monotonic() - self._track_seen_t
                <= float(self.get_parameter('track_cache_max_age_s').value))

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
        # 2026-10-14：aim_front 也必须豁免。它的检测新鲜度由自己的
        # aim_det_stale_s 判（见那个分支里的 fresh），而且**扫描阶段本来就
        # 没有检测**——转着找目标正是"还没看到"时要做的事。被这条守卫挡住的
        # 后果：节点进了 aim_front 模式却永远到不了那个分支、永不上报
        # 'aimed'，地面侧等满 预算+10s 然后报"机载节点没在跑？"，整段瞄准
        # 静默消失（实测现象就是"没有机载瞄准动作"）。
        if servo_mode not in ('coordinate_land', 'coordinate_goto',
                              'track_target', 'track_land', 'aim_front'):
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
        if servo_mode == 'track_target' and self._track_done:
            return
        if servo_mode == 'aim_front' and self._aim_done:
            return
        # track_land 也要在这儿早退：否则交棒后下一拍走到 relay 接管块，
        # 看见 _relay_mode_active=False 又把 relay 抢回 precision_land，
        # 在 normal/precision_land 之间横跳，后续 goto 指令时断时续被丢弃。
        if servo_mode in ('precision_land', 'coordinate_land', 'track_land') \
                and self._landing_triggered:
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
        # ⚠️ 2026-10-05 补充：上面"一次任务里只真正发生一次"说的是**首次接管**；
        # 接管之后现在会按 relay_reassert_period_s 周期性重申一次（见下面 else
        # 分支），原来那种"切完就再也不管"的写法会被别的节点静默顶掉。
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
            self._relay_last_assert_t = time.monotonic()
        else:
            # 2026-10-05：接管期间周期性重申 relay_mode，防止被别的节点顶掉而本
            # 节点察觉不到（见 relay_reassert_period_s 声明处的事故说明）。
            period = float(self.get_parameter('relay_reassert_period_s').value)
            if period > 0.0 and bool(self.get_parameter('auto_switch_relay_mode').value):
                now_m = time.monotonic()
                if (self._relay_last_assert_t is None
                        or now_m - self._relay_last_assert_t >= period):
                    self._set_relay_mode('precision_land')
                    self._relay_last_assert_t = now_m

        x, y, z, yaw = self._odom_xy_z_yaw
        if servo_mode in ('track_target', 'track_land'):
            # target_class_id 可能刚被 SDK 改掉，这里立刻按新目标取缓存，
            # 不等下一帧 target_positions 到达。
            self._refresh_track_from_cache()
        if servo_mode == 'aim_front':
            # ---- 机载前视瞄准：方位角 -> yaw，俯仰角 -> 高度，水平位置不动 ----
            # 水平不动是刻意的：飞机此刻停在侦查机给的发射点上（那个距离按弹药
            # 射程选的），平移会把它挪走；弹道沿机头出去，命中靠方位角保证。
            tmo = float(self.get_parameter('aim_timeout_s').value)
            ytol = math.radians(float(self.get_parameter('aim_yaw_tol_deg').value))
            ztol = float(self.get_parameter('aim_z_tol_m').value)
            gain = float(self.get_parameter('aim_yaw_gain').value)
            stale = float(self.get_parameter('aim_det_stale_s').value)
            now_m = time.monotonic()
            elapsed = (now_m - self._control_start_time
                       if self._control_start_time is not None else 0.0)
            if self._aim_scan_center is None:
                self._aim_scan_center = yaw
            fresh = (self._aim_rx is not None and (now_m - self._aim_rx) <= stale
                     and self._aim_dyaw is not None and self._aim_dz is not None)
            locked = fresh and abs(self._aim_dyaw) <= ytol and abs(self._aim_dz) <= ztol
            timed_out = tmo > 0.0 and elapsed > tmo
            if locked or timed_out:
                # 超时**按成功上报**（同 track_target 2026-10-06 那条口径）：
                # 调用方语义是"超时即刻发射"，把状态报出去比让它干等有用。
                self._publish_servo_status('aimed')
                self._aim_done = True
                if bool(self.get_parameter('auto_switch_relay_mode').value):
                    self._set_relay_mode('normal')
                self._relay_mode_active = False
                if locked:
                    self.get_logger().info(
                        '\033[32maim_front 已锁定：方位 %+.1f° 高度 %+.2f m '
                        '距离 %.2f m（%.1f s）\033[32m'
                        % (math.degrees(self._aim_dyaw), self._aim_dz,
                           self._aim_dist if self._aim_dist is not None else float('nan'),
                           elapsed))
                else:
                    self.get_logger().warn(
                        'aim_front 超时 %.0f s（%s），按当前朝向上报'
                        % (tmo, ('方位 %+.1f° 高度 %+.2f m'
                                 % (math.degrees(self._aim_dyaw), self._aim_dz))
                           if fresh else '始终没看到目标'))
                return
            world_dx = world_dy = 0.0          # 水平不动
            if fresh:
                # 捕获了：方位角驱动 yaw。**减**不是加——dyaw>0 表示目标在画面
                # 右侧，而 REP-103 机体系 y 轴朝左，"右"是 -y，所以右转要让 yaw
                # 变小。地面版那次写成加号，实测方位角 +13.7->+12.2->+19.5 一路
                # 发散到目标出视场（用户看到的现象是"朝向偏左太多"）。
                self._aim_yaw_cmd = yaw - self._aim_dyaw * gain
                self._aim_target_z = min(
                    max(z + self._aim_dz, float(self.get_parameter('aim_z_min_m').value)),
                    float(self.get_parameter('aim_z_max_m').value))
            else:
                # 还没捕获（或丢了）：以接管时朝向为中心缓慢来回扫，高度保持。
                rate = float(self.get_parameter('aim_scan_rate_dps').value)
                half = float(self.get_parameter('aim_scan_deg').value) / 2.0
                if half > 0.0 and rate > 0.0:
                    period = 4.0 * half / rate          # 一个完整来回
                    ph = (elapsed % period) / period    # 0..1
                    tri = (4.0 * ph - 1.0) if ph < 0.5 else (3.0 - 4.0 * ph)
                    self._aim_yaw_cmd = self._aim_scan_center + math.radians(half * tri)
                else:
                    self._aim_yaw_cmd = self._aim_scan_center
                self._aim_target_z = z
        elif servo_mode == 'track_target':
            tol = float(self.get_parameter('center_tol_px').value)
            tmo = float(self.get_parameter('track_timeout_s').value)
            agl = float(self.get_parameter('track_agl_m').value)
            elapsed = (time.monotonic() - self._control_start_time
                       if self._control_start_time is not None else 0.0)
            px = self._track_px
            timed_out = tmo > 0.0 and elapsed > tmo
            if (px is not None and px <= tol) or timed_out:
                # ⚠️ 超时**按成功处理**（用户 2026-10-06 定）：目标本来就已经发现了，
                # 超时只意味着"没能飞到正上方"，坐标照样是解算出来的真坐标，只是
                # 精度差一点。报出去比把整个任务判死有用得多。
                tx, ty = self._track_xy if self._track_xy is not None else (x, y)
                self._publish_servo_status('centered')
                self._publish_centered_pose(tx, ty)
                self._track_done = True
                if bool(self.get_parameter('auto_switch_relay_mode').value):
                    self._set_relay_mode('normal')
                self._relay_mode_active = False
                if timed_out:
                    self.get_logger().warn(
                        f'track_target 超时 {tmo:.0f}s（像素误差 '
                        f'{"%.0f" % px if px is not None else "未知"}），'
                        f'按当前解算坐标 ({tx:.2f}, {ty:.2f}) 上报')
                else:
                    self.get_logger().info(
                        f'\033[32mtrack_target 已到目标正上方：像素误差 {px:.0f} '
                        f'<= {tol:.0f} px，坐标 ({tx:.2f}, {ty:.2f})\033[32m')
                return
            # 还没到位：朝最近一次解算的坐标飞。目标暂时丢出画面也照飞不误——
            # 这正是"按坐标跟踪"相对图像伺服的意义（图像伺服这时只能干等）。
            # ⚠️ 这条链的产物是**相对当前位置的偏移** world_dx/dy，下游统一
            # 按 step=kp*world_d、target=当前位置+step 发指令；直接写 target_x
            # 会让 world_dx 没定义（2026-10-06 实测 UnboundLocalError 把节点打死）。
            if self._track_xy is None:
                world_dx = world_dy = 0.0     # 还没有解算结果，先原地稳住
            else:
                world_dx = self._track_xy[0] - x
                world_dy = self._track_xy[1] - y
            self._track_target_z = z if agl <= 0.0 else agl
        elif servo_mode == 'track_land':
            step = float(self.get_parameter('land_step_m').value)
            handoff = float(self.get_parameter('land_handoff_m').value)
            tol_m = float(self.get_parameter('land_handoff_tol_m').value)
            tmo = float(self.get_parameter('track_land_timeout_s').value)
            grace = float(self.get_parameter('track_lost_grace_s').value)
            now_m = time.monotonic()
            elapsed = (now_m - self._control_start_time
                       if self._control_start_time is not None else 0.0)
            # 从没拿到过坐标时，宽限从**进入模式**算起（原来直接给 1e9，等于第一拍
            # 就判"丢了"立刻交棒，根本不给看见目标的机会）。
            lost_for = (now_m - self._track_seen_t
                        if self._track_seen_t is not None else elapsed)
            # 只有新鲜坐标才能算 agl，陈旧的一律当"还没看到"，见 track_cache_max_age_s。
            agl = (z - self._track_z
                   if (self._track_xy is not None and self._track_is_fresh()) else 1e9)
            why = None
            if self._track_xy is not None and agl <= handoff + tol_m:
                why = f'已降到离地 {agl:.2f} m'
            elif tmo > 0.0 and elapsed > tmo:
                why = f'用满 {tmo:.0f} s'
            elif lost_for > grace:
                # 地面版 land_on 在这儿是 locate_target 2 秒超时就直接放弃；机载版
                # 手上有最后一次解算的坐标，能多撑 track_lost_grace_s 再交棒。
                why = f'目标丢了 {lost_for:.1f} s'
            if why is not None:
                self.get_logger().info(
                    f'\033[32mtrack_land：{why}，交给 AUTO_LAND 收尾\033[32m')
                self._trigger_land()
                # ⚠️ 交还 relay（track_land 2026-10-06 实测事故）：_trigger_land()
                # 之后本节点不再发 precision_land_cmd，降落由 px4ctrl 的 AUTO_LAND
                # 自己走完，relay 没有任何理由继续停在 'precision_land'。不交还的
                # 后果是**抓取完重新起飞后，traj_server 的 position_cmd 全被中继
                # 静默丢弃**——飞机收到指令却不动，停在空中（实测任务机抓完弹起飞
                # 后悬停不动就是这个）。原来的 precision_land 流程不暴露，是因为
                # 地面版 land_on 走 goto_direct+land()，根本不接管 relay。
                if bool(self.get_parameter('auto_switch_relay_mode').value):
                    self._set_relay_mode('normal')
                self._relay_mode_active = False
                return
            # 水平对准 + 下降一步，同一条指令（跟地面版 land_on 的语义一致）。
            # 水平给偏移、垂直给绝对高度，分别交给下游两条链，理由同上。
            # ⚠️ 2026-10-06：这里原来直接取 `self._track_xy[0]`，而目标从没被解算过时
            # 它是 None → `TypeError: 'NoneType' object is not subscriptable`，**节点当场
            # 被打死**，relay 停在 precision_land 没人交还，飞机指令全被中继丢弃。
            # 真机复现：highrise 的任务机开着前视去抓物资，下视没有任何检测数据。
            # 还没坐标时原地稳住等（等多久由 track_lost_grace_s 兜底，到点转 AUTO_LAND）。
            if self._track_xy is None:
                world_dx = world_dy = 0.0     # 还没有解算结果，原地稳住等
                self._track_target_z = z
            else:
                world_dx = self._track_xy[0] - x
                world_dy = self._track_xy[1] - y
                self._track_target_z = self._track_z + max(handoff, agl - step)
        elif servo_mode in ('coordinate_land', 'coordinate_goto'):
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
        if servo_mode in ('coordinate_land', 'coordinate_goto',
                          'track_target', 'track_land'):
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
        # 2026-10-13：z 这一步单独记下来，下面发布时要当速度前馈用。
        # **在这里初始化**而不是只在 coordinate_goto 分支里赋值——其余模式
        # （precision_land/coordinate_land/track_*/center_only）走的是别的
        # target_z 计算路径，不给它们 z 前馈，但这个名字必须在所有分支下都
        # 有定义，否则发布那段会 UnboundLocalError 把节点打死，而那种死法
        # 在空中看起来就是"飞机正常悬停"（见记忆 feedback_verify_onboard_node_edits）。
        step_z = 0.0

        horizontal_error = math.hypot(world_dx, world_dy)
        threshold = float(self.get_parameter('horizontal_converge_threshold_m').value)

        if servo_mode == 'aim_front':
            target_z = self._aim_target_z
        elif servo_mode in ('track_target', 'track_land'):
            # 高度在上面那条链里按模式算好了：track_target 保持高度，
            # track_land 每拍降一步。
            target_z = self._track_target_z
        elif servo_mode == 'center_only':
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
            # 2026-10-13：加垂直速率夹限，口径跟水平那段完全一致
            # （水平是 max_step = max_speed / rate_hz，这里同形）。
            # 不加的话 kp_z=0.5 会把整个高度误差的一半一拍发出去：2 m 的
            # 高度变化 => 设定点一拍跳 1 m。
            step_z = kp_z * world_dz
            _max_z_speed = float(self.get_parameter('coordinate_max_z_speed_mps').value)
            _rate_hz = float(self.get_parameter('control_rate_hz').value)
            _max_step_z = (_max_z_speed / _rate_hz) if _rate_hz > 0.0 else _max_z_speed
            if abs(step_z) > _max_step_z:
                step_z = math.copysign(_max_step_z, step_z)
            target_z = z + step_z

            arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
            arrival_z_threshold = float(self.get_parameter('arrival_z_threshold_m').value)
            total_error = math.hypot(horizontal_error, world_dz)   # 仅用于日志
            if horizontal_error <= arrival_threshold and abs(world_dz) <= arrival_z_threshold:
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
                        f'水平误差{horizontal_error:.3f}m(阈值{arrival_threshold:.2f})、'
                        f'垂直误差{abs(world_dz):.3f}m(阈值{arrival_z_threshold:.2f})、'
                        f'三维{total_error:.3f}m已持续收敛超过'
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
        # aim_front 要主动给朝向（瞄准靠转机头）；其余模式保持当前朝向不动。
        cmd.yaw = (self._aim_yaw_cmd
                   if (servo_mode == 'aim_front' and self._aim_yaw_cmd is not None)
                   else yaw)
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
        if servo_mode in ('coordinate_land', 'coordinate_goto',
                          'track_target', 'track_land'):
            rate_hz = float(self.get_parameter('control_rate_hz').value)
            if rate_hz > 0.0:
                cmd.velocity.x = step_x * rate_hz
                cmd.velocity.y = step_y * rate_hz
                # 2026-10-13：z 也给前馈。理由跟 2026-09-15 补 x/y 前馈时
                # 一模一样——只发位置、速度留 0，等于每个周期都告诉 PX4
                # "这一点应该是静止悬停的"，纯位置误差项算出来的响应天然保守。
                # step_z 在 coordinate_goto 之外的模式下是 0，所以这一行对
                # 那些模式没有任何影响（行为逐位不变）。
                cmd.velocity.z = step_z * rate_hz
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
            # 双保险：确认落地这一刻再交还一次 relay。上面 _trigger_land() 那次
            # 如果 service 调用恰好失败，这里补上；重复切是幂等的。
            if bool(self.get_parameter('auto_switch_relay_mode').value):
                self._set_relay_mode('normal')
            self._relay_mode_active = False
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
    except BaseException:
        # 2026-10-14 加（precision_servo_node 同款缺口）：原来只接 KeyboardInterrupt，
        # 其余异常会穿过 spin() 落到 finally，节点被干净拆掉、进程**静默退出**，
        # 异常从没被打印。空中表现就是"飞机正常悬停"，根因只能事后 ps 推断。
        # 用 BaseException 而非 Exception：SystemExit/GeneratorExit 这类更难查。
        import traceback
        try:
            node.get_logger().fatal('precision_servo_node 因未捕获异常退出——完整栈：\n'
                                    + traceback.format_exc())
        except Exception:
            pass
        traceback.print_exc()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
