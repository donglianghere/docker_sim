#!/usr/bin/env python3
"""L1层：rclpy生命周期封装（对应方案2.2节分层设计`_rclpy_runtime.py`）。

**这一层的定位**：`contest_sdk`选手层（`DroneSDK`/`capabilities.py`）不
应该直接接触rclpy——不是因为rclpy用起来很难，而是方案2.2.1节"封装完整性"
的硬性要求：选手是竞赛参赛者，不是这个项目的开发者，任何情况下（包括
出错的时候）都不应该看到裸rclpy/DDS的异常类型、日志格式、消息类型。
`rclpy.init()`/`spin()`/`shutdown()`这套生命周期管理本身就是"ROS2内部
概念"的一部分——一旦把这些暴露给`capabilities.py`，`capabilities.py`
就得自己操心"spin线程什么时候起、什么时候该停、进程里是不是已经有
别的rclpy运行时在跑"这些跟"选手能力怎么实现"完全无关的细节。这一层
把这些细节全部收进来，`capabilities.py`只需要问这一层要一个"节点句柄"，
拿到手之后调的是`rclpy.node.Node`暴露的标准方法（`create_publisher()`/
`create_subscription()`/`create_client()`/`create_timer()`/`get_clock()`
等）——这不算"碰rclpy"，`capabilities.py`不需要`import rclpy`、不需要
知道`init()`/`spin()`怎么调，跟直接用rclpy写一个ROS2节点是两件不同的
事，边界就划在"生命周期管理"和"调用一个已经活起来的节点的方法"之间。

**为什么这里选`MultiThreadedExecutor`而不是最朴素的`rclpy.spin(node)`**：
这个项目已经在`contest_mission`包里真实踩过"单线程executor下、回调内部
又发起另一次阻塞调用会自己等自己死锁"这个坑——`scenario_reset_node.py`
的`~/reset_scenario`service回调、`fire_pillar_aim_node.py`的检测回调
内部调`position_cmd_relay`的`~/set_parameters`service，两处都因此显式
换成了`MultiThreadedExecutor`+`MutuallyExclusiveCallbackGroup`才躲开
死锁（见`fire_pillar_aim_node.py`文件头注释）。这一层没法预知L2/L3
（`capabilities.py`/`reliability.py`）具体会怎么组织回调，但方案2.3节
已经明确写清楚SDK内部本身就存在"多种回调需要同时活着"的场景——
`sdk.send_to_teammate()`一边靠定时器以1Hz重发事件、一边靠订阅回调等
对方回发的ack；`sdk.takeoff()`一边等状态订阅回调确认`armed`，未来如果
某个能力需要在回调内部发起service调用去做二次确认，单线程spin会立刻
重蹈`scenario_reset_node`的覆辙。用`MultiThreadedExecutor`把"回调之间
会不会互相阻塞"这个风险从"L1一旦选错执行器全盘遇险"降级成"L2/L3
按需要给某个callback套`MutuallyExclusiveCallbackGroup`就能局部解决"，
代价只是多占一点线程资源，对这个SDK的规模（每个选手进程只服务一架
飞机）来说完全可以接受。

**为什么spin放在后台线程、不是让选手调用某个"跑起来"的方法阻塞主线程**：
`capabilities.py`里"一次性指令+等待/轮询结果"这种模式（2.2节架构原则）
要求调用方（选手代码里的`sdk.goto(...)`那一行）能够阻塞等待，同时ROS2
的订阅/定时器回调必须在后台持续被处理，两者不能挤在同一个线程——所以
`RclpyRuntime`自己起一个专职spin的后台线程，选手调用的那些阻塞方法
（`capabilities.py`里实现）留在调用方自己的线程里用`threading.Event`/
轮询等待，两个线程各司其职，互不干扰。
"""
import os
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from contest_sdk.exceptions import EnvironmentMisconfiguredError

# 这次仿真/比赛系统全场约定使用的ROS_DOMAIN_ID（方案2.4节"部署现场网络
# 配置检查清单"第4条）——GCS、双机flight-stack容器、选手的contest_sdk
# 容器全部必须用这一个值，否则DDS discovery会各自活在独立的域里、互相
# 看不见对方的节点/话题。这类问题的故障现象通常不是"连接失败"这种能
# 立刻定位的报错，而是"某个调用一直卡住直到超时"，排查成本很高，所以
# 选择在SDK最早期（`rclpy.init()`之前）就主动校验、快速失败。
EXPECTED_ROS_DOMAIN_ID = '21'


class RclpyRuntime:
    """封装一份"能收发ROS2消息的能力"，给`capabilities.py`当底座用。

    对上层暴露的唯一接口是`node`属性——拿到的是一个已经在后台线程持续
    spin、随时可以`create_publisher()`/`create_subscription()`/
    `create_client()`/`create_timer()`的标准`rclpy.node.Node`实例。
    `rclpy.init()`/后台spin线程/`rclpy.shutdown()`这些生命周期动作全部
    由这个类自己管理，`capabilities.py`不需要、也不应该关心。
    """

    # 正常情况下每架飞机的选手程序都是独立进程，一个进程只会创建一个
    # `RclpyRuntime`实例；但测试代码/未来的复用场景不排除同一个进程里
    # 出现不止一个实例。rclpy的全局context只应该被`init()`一次、
    # `shutdown()`一次——用类级别的引用计数代替"每个实例各自无条件
    # init/shutdown"，避免"实例B还在用，实例A却把全局context关掉"
    # 这种互相干扰，也满足"进程里已有rclpy运行时时不能重复init崩溃"
    # 这条要求。
    _lock = threading.Lock()
    _active_count = 0

    def __init__(self, namespace: str):
        if not namespace:
            raise ValueError(
                'namespace不能为空——这是跟物理飞机绑定的身份标识（方案'
                '1.5节），必须由调用方显式传入，不能有默认值。'
            )
        self._namespace = namespace
        self._is_shutdown = False

        # ---- 第一步：ROS_DOMAIN_ID一致性校验 ----
        # 故意放在`rclpy.init()`之前做：域配置不对的话，DDS discovery
        # 在错误的域里照样能跑起来、不会立刻报错，往往是后面某个能力
        # 调用（比如takeoff等armed确认）"卡住直到超时"才暴露问题，那
        # 时候选手很难第一时间联想到"是不是ROS_DOMAIN_ID配错了"——不如
        # 在最早期就把这类环境问题挡在门口。
        actual = os.environ.get('ROS_DOMAIN_ID')
        if actual != EXPECTED_ROS_DOMAIN_ID:
            raise EnvironmentMisconfiguredError(
                f"ROS_DOMAIN_ID环境变量配置不对：当前值={actual!r}，"
                f"期望值={EXPECTED_ROS_DOMAIN_ID!r}。请检查ROS_DOMAIN_ID"
                f"环境变量——这个仿真/比赛系统全场（GCS、双机飞控容器、"
                f"选手SDK容器）都必须使用同一个ROS_DOMAIN_ID，否则DDS "
                f"discovery会各自活在独立的域里、互相看不见对方的节点，"
                f"故障现象通常是后续某个能力调用一直卡住直到超时，而不是"
                f"立刻报出一个明确的连接错误。"
            )

        # ---- 第二步：rclpy.init()只应该被调用一次 ----
        with RclpyRuntime._lock:
            if not rclpy.ok():
                # 进程里还没有任何存活的rclpy运行时——最常见的路径（每
                # 架飞机的选手程序都是独立进程）。
                rclpy.init()
            # else分支：`rclpy.ok()`为真说明进程里已经有别的代码（可能
            # 是另一个`RclpyRuntime`实例，也可能是宿主进程自己）先调用
            # 过`rclpy.init()`了——直接复用同一个全局context即可，绝对
            # 不能再调一次`rclpy.init()`（重复init会直接抛异常崩溃）。
            RclpyRuntime._active_count += 1

        # ---- 第三步：创建节点 ----
        # 节点名拼上namespace，纯粹是可读性/可调试性考虑（多机场景下
        # `ros2 node list`/日志能一眼分清是哪架飞机的选手SDK在跑），
        # 不影响任何功能逻辑。
        #
        # ⚠️ 2026-09-13实现B3时发现的真实接口不匹配bug，这里顺手修了一下：
        # 必须显式传`namespace=`给`Node()`，不能只把namespace拼进节点名字
        # 符串里——这个项目里其它节点（`contest_mission`/`ego_planner_bridge`
        # 等）的ROS图命名空间从来不是在Python代码里设置的，是靠
        # `flight-stack-entrypoint.sh`里`ros2 run ... --ros-args -r
        # __ns:="/${NAMESPACE}"`这个启动参数setup的（见该脚本里
        # `actuator_action_node`/`precision_servo_node`等的启动方式）。
        # 但`contest_sdk`是选手直接`python3 我的任务.py`跑起来的普通Python
        # 脚本，没有任何机制会自动往`sys.argv`里塞这个`--ros-args -r
        # __ns:=...`——如果这里不显式设置`namespace=`，`capabilities.py`
        # 里所有相对话题名（`vision/detections`/`mission_state`/
        # `action_status`/`takeoff_land`/`waypoint_queue`等，为了跟
        # flight-stack一侧的命名习惯保持一致，全部特意写成相对话题名，
        # 不是绝对路径）都会解析到ROS图的根命名空间（`/vision/detections`），
        # 而不是这架飞机自己的`/{namespace}/vision/detections`，会完全
        # 收不到/发不到对的话题，且不会有任何报错提示（DDS discovery在
        # 错误的话题名下一样能跑起来，只是没有匹配的对端），排查成本
        # 很高。这里补上`namespace=`参数，让这个节点的ROS图命名空间
        # 真正对应到`DroneSDK`构造时传入的`namespace`参数，跟flight-stack
        # 那一侧`--ros-args -r __ns:=`达到的效果完全一致。
        self._node = Node(f'{namespace}_contest_sdk', namespace=namespace.strip('/'))

        # ---- 第四步：起一个后台线程持续spin ----
        # 选`MultiThreadedExecutor`而不是`rclpy.spin(node)`的理由见文件
        # 头说明。`daemon=True`只是一层兜底：选手程序万一忘了调用
        # `shutdown()`就直接退出解释器，这个后台线程不应该把整个进程
        # 挂住退不出去——但这不代表鼓励跳过`shutdown()`，干净退出还是
        # 应该走`shutdown()`那条路径（会正常destroy节点、释放DDS资源）。
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name=f'{namespace}_contest_sdk_spin',
            daemon=True,
        )
        self._spin_thread.start()

    @property
    def node(self) -> Node:
        """`capabilities.py`唯一应该使用的入口：拿到这个`rclpy.node.
        Node`句柄之后，直接调它的标准方法（`create_publisher()`/
        `create_subscription()`/`create_client()`/`create_timer()`/
        `get_clock()`等）即可——不需要`import rclpy`，也不需要关心这个
        节点是怎么被init/spin起来的，那些细节全部封装在这个类内部。
        """
        return self._node

    @property
    def namespace(self) -> str:
        """回传构造时传入的namespace，方便`capabilities.py`/
        `reliability.py`在拼话题名、打日志时复用，不需要自己再存一份。
        """
        return self._namespace

    def shutdown(self) -> None:
        """选手SDK整体退出时应该调用一次，做干净清理：停止后台spin、
        销毁节点、（只有当自己是最后一个还在用全局context的实例时）
        才真正关闭rclpy。

        这个方法是幂等的——重复调用（比如选手代码里手动调了一次，
        `DroneSDK.__del__`/`atexit`又兜底调了一次）不会报错。
        """
        if self._is_shutdown:
            return
        self._is_shutdown = True

        # 先让executor停止spin循环，再等spin线程真正退出，避免"节点
        # 正在被destroy的过程中，spin线程还在并发访问这个节点"这种
        # 竞态。超时5秒纯粹是兜底（正常情况`shutdown()`几乎立即让
        # `spin()`返回），避免个别异常场景下`shutdown()`本身被卡死。
        self._executor.shutdown()
        self._spin_thread.join(timeout=5.0)
        self._node.destroy_node()

        with RclpyRuntime._lock:
            RclpyRuntime._active_count = max(0, RclpyRuntime._active_count - 1)
            # 只有"最后一个还在用这个全局context的RclpyRuntime实例"才
            # 真正调用`rclpy.shutdown()`——否则会把别的实例还在用的
            # context关掉，导致对方后续任何rclpy调用报错崩溃。
            if RclpyRuntime._active_count == 0 and rclpy.ok():
                rclpy.shutdown()
