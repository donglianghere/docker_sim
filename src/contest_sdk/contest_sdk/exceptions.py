"""contest_sdk自己的异常类型（对应方案2.2.1节"封装完整性"要求）。

选手在任何情况下（包括出错时）都不应该接触到裸rclpy/DDS的异常类型——
`capabilities.py`/`reliability.py`内部捕获底层可能抛出的异常后，统一
转换成这个文件里定义的类型再往外抛，选手写`except`只需要认这几个类。

每个异常的`__init__`都按方案2.2.2节"可调试性"要求，在消息里带上"可能
原因"的排查提示，不是只报"超时"两个字——选手是竞赛参赛者，不是这个
项目的开发者，不能指望他们会主动开终端敲`ros2 topic echo`去排查问题，
异常文本本身就要把人往正确的排查方向上推一把。
"""


class ContestSdkError(Exception):
    """所有contest_sdk异常的公共基类，方便选手写`except ContestSdkError`
    一次性兜住所有SDK自己抛出的错误，不需要逐个类型枚举。
    """


class EnvironmentMisconfiguredError(ContestSdkError):
    """环境配置不对（目前只有`ROS_DOMAIN_ID`不一致这一种触发场景，见
    `_rclpy_runtime.py`），构造时不需要额外拼接提示——这类错误本身
    就是"环境没配对"，调用方看到这条异常本身就该去检查环境变量，
    不需要像超时类异常那样区分"是我代码问题还是网络/对方问题"。
    """

    def __init__(self, message: str):
        super().__init__(message)


class TakeoffTimeoutError(ContestSdkError):
    """`sdk.takeoff()`超时——`armed`一直没变`true`，或者`armed=true`了
    但飞控爬升到位+状态转换一直没有稳定下来（2026-09-14`takeoff()`
    改成两段等待之后，这个类要能区分说清楚是卡在哪一段，不能不管哪段
    失败都说成"没等到armed=true"，那样会误导排查方向）。
    """

    def __init__(self, timeout_s: float, namespace: str, stage: str = 'armed'):
        if stage == 'preflight_connected':
            super().__init__(
                f"起飞前置检查失败（等待{timeout_s}秒仍未确认PX4飞控已连接，"
                f"namespace={namespace}）。可能原因：①PX4 SITL/飞控固件是否已经完全启动；"
                f"②mavros节点是否正常运行、有没有崩溃重启；③容器刚起来时过早调用"
                f"takeoff()，PX4还没来得及跟mavros建立MAVLink连接。"
            )
        elif stage == 'preflight_uwb':
            super().__init__(
                f"起飞前置检查失败（等待{timeout_s}秒仍未收到有效的定位数据，"
                f"namespace={namespace}）。可能原因：①UWB/里程计融合链路是否正常工作"
                f"（`dlio/odom_node/odom`话题有没有发布）；②UWB基站/标签是否已经准备妥当；"
                f"③容器刚起来时过早调用takeoff()，定位链路还没完全初始化。"
            )
        elif stage == 'armed':
            super().__init__(
                f"起飞超时（等待{timeout_s}秒仍未确认armed=true，namespace={namespace}）。"
                f"可能原因：①仿真/真机是否已经完全启动（PX4 SITL/飞控固件初始化需要时间，"
                f"容器刚起来时过早调用takeoff()容易撞上这个窗口）；②飞控是否处于允许起飞的"
                f"前置状态（比如还在等待GPS/定位源收敛、或者被安全检查拒绝）；③"
                f"`ROS_DOMAIN_ID`/网络配置是否正确（选手程序是否真的连上了这架飞机的话题域，"
                f"参考2.4节网络检查清单）。"
            )
        else:
            super().__init__(
                f"起飞超时（armed=true已确认，但等{timeout_s}秒位置仍未稳定下来，"
                f"namespace={namespace}）。可能原因：①飞控的AUTO_TAKEOFF状态一直没能"
                f"转入AUTO_HOVER（比如反复被某种条件打断、悬停本身在明显振荡），"
                f"检查pt4ctrl自己的日志有没有反复在几个状态之间跳变；②飞机在爬升途中"
                f"撞到了障碍物或者卡在什么地方，实际根本没有真正在爬升；③仿真本身负载"
                f"过重、实时因子过低，导致里程计更新变慢，位置稳定窗口迟迟凑不够。"
            )


class GotoTimeoutError(ContestSdkError):
    """`sdk.goto()`等待到达目标点确认超时。"""

    def __init__(self, timeout_s: float, target_xyz, namespace: str):
        super().__init__(
            f"飞往目标点{target_xyz}超时（等待{timeout_s}秒仍未确认到达，"
            f"namespace={namespace}）。可能原因：①目标点是否在障碍物内部/仿真世界"
            f"边界外，规划器可能一直规划不出可行轨迹；②长距离目标是否触发了"
            f"`ego_planner`的重规划死循环（本项目实测踩过的坑，建议按最大步长3米"
            f"自己分段插值，不要一次性丢一个很远的目标点）；③飞机当前是否处于"
            f"别的接管模式（比如精降/编队跟随）导致压根没有响应这次goto。"
        )


class GotoUnreachableError(GotoTimeoutError):
    """`sdk.goto()`检测到飞机已经卡住、到不了目标点（2026-09-21新增）。

    典型原因是目标点落在障碍物（含规划器的膨胀区）里：规划器会把轨迹终点
    推到障碍物边缘，飞机停在那儿悬停，`waypoint_state`一直是`executing`，
    永远不会变成`completed`。原来`goto()`只能干等满`timeout`秒抛
    `GotoTimeoutError`；现在检测到"飞机不动了、离目标还有距离"就尽快抛这个。

    继承`GotoTimeoutError`：原来捕获`GotoTimeoutError`的代码不用改，照样能
    接住这种情况，只是更早拿到。想区分"卡住"和"真超时"时单独捕获这个。

    弓字形搜索这类场景的推荐用法是捕获它、跳过这个航点、飞下一个——地面
    火情不可能在障碍物底下，而飞机绕行时下视相机已经扫过了障碍物周边。
    """

    def __init__(self, target_xyz, stopped_xyz, distance_m: float, namespace: str):
        ContestSdkError.__init__(
            self,
            f"目标点{target_xyz}不可达：飞机停在{stopped_xyz}不动，离目标还有"
            f"{distance_m:.2f}米（namespace={namespace}）。最常见的原因是目标点落在"
            f"障碍物或规划器的膨胀区里，规划器把轨迹终点推到了障碍物边缘。"
            f"搜索类任务可以捕获这个异常，跳过这个航点继续飞下一个。"
        )
        self.target_xyz = target_xyz
        self.stopped_xyz = stopped_xyz
        self.distance_m = distance_m


class LandTimeoutError(ContestSdkError):
    """`sdk.land()`等待`armed`变为`false`超时。

    实现B3时新增的异常类型（不是B1原有的5个之一）——方案清单原话允许
    `land()`"复用`TakeoffTimeoutError`或者你觉得需要单独定义一个land
    专用异常也行"。这里选择单独定义，理由：`TakeoffTimeoutError`的报错
    文本硬编码了"起飞超时"+"armed=true"这些起飞语境的措辞，如果`land()`
    复用它，选手调用`sdk.land()`失败时会看到一句提到"起飞"的报错，跟
    实际在做的事（降落）完全对不上，直接违反方案2.2.2节"可调试性"要求
    ——异常文本要把人往正确的排查方向上推，而不是让人先困惑"我明明在
    降落，为什么报起飞超时"。两个类结构几乎一样（都是等`armed`确认），
    但措辞和"可能原因"提示分别贴合各自的场景。
    """

    def __init__(self, timeout_s: float, namespace: str):
        super().__init__(
            f"降落超时（等待{timeout_s}秒仍未确认armed=false，namespace={namespace}）。"
            f"可能原因：①飞控是否卡在某个中间状态没有真正进入AUTO_LAND（比如还在"
            f"执行别的接管模式，如精降/编队跟随，没有响应这次land()发出的"
            f"`TakeoffLand{{LAND}}`指令）；②飞机当前高度/速度是否满足`land_"
            f"detector`判定为已落地的条件（比如卡在半空悬停迟迟不下降）；③"
            f"`ROS_DOMAIN_ID`/网络配置是否正确（参考2.4节网络检查清单，`mavros/"
            f"state`话题是否真的收到了数据）。"
        )


class ActionFailedError(ContestSdkError):
    """`sdk.do_action()`等待`action_status`确认`done:<name>`超时/失败。"""

    def __init__(self, action_name: str, timeout_s: float, namespace: str):
        super().__init__(
            f"动作'{action_name}'执行失败或超时（等待{timeout_s}秒仍未收到"
            f"'done:{action_name}'确认，namespace={namespace}）。可能原因："
            f"①`action_name`拼写是否正确（跟接收方`actuator_action_node`约定的"
            f"字符串必须完全一致，区分大小写）；②对应的RC通道/PWM映射参数是否"
            f"已经配置（真实硬件接线确定前用的是占位值，行为可能跟预期不符）；"
            f"③飞控/mavros链路是否正常（`mavros/rc/override`能不能真的送达飞控）。"
        )


class TeammateUnreachableError(ContestSdkError):
    """`sdk.send_to_teammate()`等待对方ack确认超时（方案2.3节"链路B"）。"""

    def __init__(self, event: str, timeout_s: float, teammate_namespace: str):
        super().__init__(
            f"发给队友（{teammate_namespace}）的事件'{event}'超过{timeout_s}秒"
            f"仍未收到确认。可能原因：①请确认队友程序是否已经启动（这条消息需要"
            f"对方的SDK实例在线才能应答）；②两台设备是否在同一个路由器/WiFi下、"
            f"`ROS_DOMAIN_ID`是否一致（跨机通信最常见的踩坑点）；③路由器/网络"
            f"是否存在临时抖动或客户端隔离（AP client isolation）没有关闭；④"
            f"如果双方都确认在线且网络正常，检查一下事件名字符串两边是否完全一致。"
        )


class SoundLightError(ContestSdkError):
    """`sdk.play_sound_light()`访问机载声光反馈板失败（串口打开/写入出错）。

    板子是纯单向UART接收、不回ACK，这里没有"等待对方确认"这个环节——
    出错只可能是本地串口层面的问题，不是"等超时"这种失败模式，所以
    没有复用`ActionFailedError`（措辞是"等待XX秒确认"，跟这里"写一次
    就算发出去"的实际语义对不上，参考`LandTimeoutError`docstring里
    "不能不管什么失败都套同一个异常措辞"这条教训）。
    """

    def __init__(self, port: str, reason: str):
        super().__init__(
            f"声光反馈板串口访问失败（port={port}）：{reason}。可能原因："
            f"①USB转串口设备是否已经插好、系统里是否真的存在这个设备节点"
            f"（`ls -l {port}`确认）；②当前用户是否在`dialout`组里、对该"
            f"设备节点有没有读写权限；③端口是否已经被别的进程占用（比如"
            f"另一个SDK实例、`minicom`/`screen`等调试工具还开着没关）；"
            f"④`port`路径是否配对（多台USB转串口设备时`/dev/ttyUSB*`编号"
            f"可能跟预期不一致，必要时改用`/dev/serial/by-id/...`稳定路径）。"
        )


class DetectionTimeoutError(ContestSdkError):
    """`sdk.wait_for_detection()`等待指定`class_id`出现超时。"""

    def __init__(self, class_id: str, timeout_s: float, namespace: str):
        super().__init__(
            f"等待检测目标'{class_id}'超过{timeout_s}秒仍未出现（namespace="
            f"{namespace}）。可能原因：①飞机是否真的飞到了目标附近、朝向/高度"
            f"是否能让相机看到目标；②`class_id`字符串是否跟视觉检测节点实际"
            f"发布的命名一致（比如`apriltag:1`这种带冒号的格式，容易打错）；"
            f"③相机/视觉检测链路本身是否正常工作（可以确认`vision/detections`"
            f"话题是否有任何输出，哪怕是别的class_id）。"
        )
