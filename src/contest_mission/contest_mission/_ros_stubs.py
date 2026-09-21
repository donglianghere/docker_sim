# -*- coding: utf-8 -*-
"""给不依赖ROS运行时的单元测试用的rclpy/消息包桩。

编队跟随的几何算法必须能在没有ROS的机器上直接跑单测（方案第5节"用
synthetic轨迹数据测试，不要直接上真实双机飞行验证"），所以把节点文件
顶部那些`import rclpy`之类的依赖在这里一次性顶掉，测试文件只管几何。
"""
import importlib.util
import sys
import types

_MODULES = [
    'rclpy', 'rclpy.action', 'rclpy.callback_groups', 'rclpy.executors', 'rclpy.node',
    'rclpy.parameter', 'rclpy.publisher', 'rclpy.qos', 'rclpy.time', 'rcl_interfaces', 'rcl_interfaces.msg',
    'rcl_interfaces.srv', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
    'quadrotor_msgs', 'quadrotor_msgs.action', 'quadrotor_msgs.msg', 'sensor_msgs',
    'sensor_msgs.msg', 'tf2_ros',
]

_ATTRS = {
    'rclpy.action': ['ActionServer', 'CancelResponse', 'GoalResponse'],
    'rclpy.callback_groups': ['ReentrantCallbackGroup'],
    'rclpy.executors': ['MultiThreadedExecutor'],
    'rclpy.parameter': ['Parameter'],
    'rclpy.publisher': ['Publisher'],
    'rclpy.qos': ['QoSProfile', 'ReliabilityPolicy'],
    'rclpy.time': ['Time'],
    'rcl_interfaces.msg': ['Parameter', 'ParameterType', 'ParameterValue', 'SetParametersResult'],
    'rcl_interfaces.srv': ['SetParameters'],
    'geometry_msgs.msg': ['PoseStamped'],
    'nav_msgs.msg': ['Odometry'],
    'quadrotor_msgs.action': ['FormationFollow'],
    'quadrotor_msgs.msg': ['PositionCommand'],
    'sensor_msgs.msg': ['Range'],
    'tf2_ros': ['Buffer', 'TransformListener'],
}


def install() -> None:
    """把ROS相关模块替换成空壳，可重复调用。"""
    for name in _MODULES:
        if name not in sys.modules or not isinstance(sys.modules[name], types.ModuleType) \
                or not hasattr(sys.modules[name], '__stub__'):
            m = types.ModuleType(name)
            m.__path__ = []
            m.__stub__ = True
            sys.modules[name] = m
    sys.modules['rclpy.node'].Node = type('Node', (), {'__init__': lambda s, *a, **k: None})
    for mod, attrs in _ATTRS.items():
        for attr in attrs:
            setattr(sys.modules[mod], attr, object)


def load_node_module(path: str, alias: str = 'ffn'):
    """装好桩之后按文件路径加载节点模块，不走包导入，避免触发__init__。"""
    install()
    spec = importlib.util.spec_from_file_location(alias, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
