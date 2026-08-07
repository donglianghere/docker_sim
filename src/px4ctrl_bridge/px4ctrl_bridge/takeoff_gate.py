#!/usr/bin/env python3
"""px4ctrl版"起飞口令"闸门，跟ros2_px4_stack路径下
docker_sim/patches/ros2_px4_stack_takeoff_gate.patch加的`/tmp/takeoff_go`
机制是同一个操作习惯：docker_sim/scripts/launch_control.sh在tmux里按回车
才`touch`这个文件，两架飞机各自的px4ctrl才会真正起飞，避免容器一起来
就自动爬升、操作员来不及反应。

px4ctrl自己的AUTO_TAKEOFF状态是靠收到一次
`quadrotor_msgs/msg/TakeoffLand{takeoff_land_cmd: TAKEOFF}`触发的
（见PX4CtrlFSM.cpp的MANUAL_CTRL分支），这个节点要做的事情很简单：
等文件出现，发几次触发消息，然后退出——不是长驻节点。
"""
import os
import time

import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import TakeoffLand

GATE_FILE = '/tmp/takeoff_go'


class TakeoffGate(Node):
    def __init__(self):
        super().__init__('takeoff_gate')
        self._pub = self.create_publisher(TakeoffLand, 'takeoff_land', 10)

    def run(self, poll_interval_sec: float = 0.5, publish_repeat: int = 5) -> None:
        self.get_logger().info(f"等待起飞口令文件 {GATE_FILE} ...")
        while rclpy.ok() and not os.path.exists(GATE_FILE):
            time.sleep(poll_interval_sec)

        if not rclpy.ok():
            return

        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        # px4ctrl的takeoff_land订阅QoS depth=100，是可靠传输，理论上发一次就够；
        # 多发几次纯粹是防御性的（万一节点刚起来、订阅还没握手完成就错过第一条），
        # 对状态机没有副作用——AUTO_TAKEOFF只在MANUAL_CTRL状态下才会响应这个
        # 触发，重复收到不会重复起飞。
        for _ in range(publish_repeat):
            self._pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info("起飞口令已发送")


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffGate()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
