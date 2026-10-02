# -*- coding: utf-8 -*-
"""把 ROS 相关模块桩掉，好在**没装 ROS 的机器上**导入 contest_sdk。

为什么要这样：`capabilities.py` 顶层 import 了 rclpy / rcl_interfaces /
std_srvs，而这些只装在 docker 镜像里。但 SDK 里大量逻辑（航点跳过判据、
收件箱语义、解散判据、几何换算）跟 ROS 一点关系都没有，没理由为了测它们
去起一整套仿真——那要 5~16 分钟，而这些测试是秒级的。

做法是装一个 import 钩子：遇到下面列出的包就现造一个空壳模块，属性按需
生成。**不碰 contest_sdk 自己的模块**，那些要测的是真代码。
"""
import sys
import types

#: 要桩掉的顶层包。只列真正装在镜像里的那些。
ROS_PACKAGES = ('rclpy', 'rcl_interfaces', 'std_srvs', 'std_msgs',
                'geometry_msgs', 'sensor_msgs', 'nav_msgs', 'tf2_ros',
                'quadrotor_msgs', 'contest_msgs', 'builtin_interfaces',
                'rosidl_runtime_py', 'ament_index_python', 'cv_bridge')


class _Any:
    """桩对象：取任何属性都返回新的桩，调用返回新的桩。

    这样 `Parameter()`、`ParameterType.PARAMETER_DOUBLE`、`Duration(seconds=1)`
    这类写法都不会炸，而我们要测的逻辑根本不碰它们的真实行为。
    """

    def __init__(self, *a, **kw):
        pass

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        setattr(self, name, _Any())
        return getattr(self, name)

    def __call__(self, *a, **kw):
        return _Any()


class _StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        v = _Any()
        setattr(self, name, v)
        return v


class _Finder:
    def find_module(self, fullname, path=None):
        return self if fullname.split('.')[0] in ROS_PACKAGES else None

    def load_module(self, fullname):
        if fullname in sys.modules:
            return sys.modules[fullname]
        m = _StubModule(fullname)
        m.__path__ = []          # 当成包，允许 from x.y import z
        m.__loader__ = self
        sys.modules[fullname] = m
        return m


def install():
    """装钩子并返回 capabilities 模块。重复调用安全。"""
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())
    import os
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    sdk = os.path.join(root, 'src', 'contest_sdk')
    if sdk not in sys.path:
        sys.path.insert(0, sdk)
    from contest_sdk import capabilities
    return capabilities
