"""`contest_sdk`——2026大赛选手SDK的对外入口（对应方案2.2节/清单B5节）。

选手只应该从这里`import`，不应该直接`import contest_sdk._rclpy_runtime`
或`import contest_sdk.reliability`——这两个是L1/L3内部实现细节（方案
2.2.1节"封装完整性"要求的一部分：选手不应该知道SDK内部是怎么用rclpy/
应用层ACK协议搭起来的，只需要认`DroneSDK`这一个类+几个异常类型）。

`from .exceptions import *`能拿到哪些名字，由`exceptions.py`自己控制
（该文件目前没有定义`__all__`，`import *`会拿到其中所有不以下划线开头
的模块级名字——也就是`ContestSdkError`基类+`EnvironmentMisconfiguredError`/
`TakeoffTimeoutError`/`GotoTimeoutError`/`LandTimeoutError`/
`ActionFailedError`/`TeammateUnreachableError`/`DetectionTimeoutError`
这7个具体异常类，选手写`except ActionFailedError`或者`except
ContestSdkError`一次性兜底都可以）。
"""
from .capabilities import DroneSDK
from .exceptions import *  # noqa: F401,F403 - 有意的通配导出，见上面模块docstring
