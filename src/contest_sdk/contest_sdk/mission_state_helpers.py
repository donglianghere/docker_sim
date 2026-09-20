#!/usr/bin/env python3
"""`sdk.set_mission_state()`/`sdk.get_mission_state()`要封装的契约——vendor自
`src/contest_mission/contest_mission/mission_state.py`（不是`import
contest_mission`，理由跟`geometry_helpers.py`一样：选手容器不应该依赖这个
跟ROS2节点耦合的大包，见方案2.1节第8条/清单B3第2条边界说明）。

**为什么单独拆成这一个文件，不直接放进`capabilities.py`**：这两个东西
（`VALID_MISSION_STATES`常量+`is_valid_mission_state()`纯函数）本身跟ROS
完全无关，只是字符串校验；但`capabilities.py`模块顶层要`import rclpy.*`/
`rcl_interfaces.*`/`std_srvs.*`这些ROS2 Python包（`DroneSDK`本身要用它们），
如果把这两个东西也定义在`capabilities.py`里，任何人想单独测试"冒号子状态
格式校验对不对"这么小的一段逻辑，都要被迫先装好整套ROS2环境——违反方案
第5节"每个能力方法写单元测试"里"不依赖真实ROS2环境"这条要求（跟`test_
reliability.py`把ACK协议单独隔离出来测试是同一个考虑）。拆成这个零ROS
依赖的小文件后，`test_mission_state_helpers.py`可以直接`python3`跑，不需要
`source /opt/ros/.../setup.bash`。

⚠️ **实现前确认过的一件事（清单B3第2条明确要求先确认、不要凭空假设）**：
`mission_state.py`现有的`is_valid_mission_state()`**已经**支持"顶层值:
子状态"这种带冒号的写法（`state.split(':', 1)[0]`只取冒号前半部分校验），
方案4.4节要用的`'searching:orbit_pillar_2'`这种格式**不需要额外扩展**就能
通过校验——下面这份vendor拷贝直接照抄原逻辑，没有做任何"扩展校验支持
冒号"的改动，因为原函数本来就支持，没有需要扩展的地方（如果原函数当年
不支持，这里才需要动手扩展，但实际读过源码后发现不需要）。
"""

#: 跟`contest_mission/mission_state.py`原文完全一致，没有额外增删——
#: 发布方如果需要更细粒度的状态，应该在字符串里追加冒号分隔的子状态
#: （比如`'searching:orbit_pillar_2'`），而不是往这个枚举里加新的顶层值。
VALID_MISSION_STATES = ('idle', 'searching', 'loading', 'enroute', 'aiming', 'done')


def is_valid_mission_state(state: str) -> bool:
    """允许"顶层值"或"顶层值:子状态"两种形式，只校验顶层值。

    跟`contest_mission/mission_state.py::is_valid_mission_state()`逐字
    一致（vendor拷贝，不是import）。
    """
    top_level = state.split(':', 1)[0]
    return top_level in VALID_MISSION_STATES
