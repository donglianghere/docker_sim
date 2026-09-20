#!/usr/bin/env python3
"""启动时放宽几个PX4失控保护参数，让仿真里能自动起飞而不被PX4自己的
安全逻辑踢出OFFBOARD/拒绝解锁。

这段逻辑原本长在ros2_px4_stack的`OffboardDynusFollower._set_px4_param()`/
`_kick_offboard()`里（见docker_sim/patches/ros2_px4_stack_dynus.patch），
跟"发布setpoint给PX4"这个职责耦合在同一个类里。px4ctrl接管setpoint发布
之后，这几个PX4参数放宽本身跟走哪个板外控制器无关（两边都需要），所以
单独拆成一个一次性跑完就退出的节点，不跟任何具体的offboard follower
类绑定——CONTROLLER=ros2_px4_stack或CONTROLLER=px4ctrl两条路径都可以
复用这一个节点，不用各自维护一份。

跑一次、把三个参数都设置成功（或超时放弃）就退出，不是长驻节点。

2026-08-07实测踩坑：第一版`mavros/param/set`服务等待超时只给了15秒，
就在同一次事故的容器日志里看到这个节点"process has died, exit code 1"——
这套仿真里mavros/PX4完整建链实测要40秒以上（`px4ctrl_node`自己那边
"Unable to connnect to PX4"也刷了快40秒），15秒的等待预算大概率总是等不到，
这几个失控保护参数因此大概率从来没被放宽过。跟
`ros2_px4_stack_kick_offboard_timeout.patch`当时遇到的"仿真real-time-factor
远低于1、原来10秒真实时间预算不够用"是同一类问题，这里直接抄那次的做法，
把预算放宽到120秒真实时间，并且改成"服务调用失败会重试"而不是试一次就放弃。

2026-08-14新增DEPLOY_TARGET区分（读同名环境变量，默认"sim"，容器级由
flight-stack-entrypoint.sh统一export，这个节点自己不需要额外的launch参数
传递）：仿真下三个参数的取值不变（原有行为，见下面PARAMS_SIM）；真机下
（DEPLOY_TARGET=hw）这三个参数不再是"图省事让仿真自动起飞更顺畅"的宽松值，
是真实的失控保护开关，逐条重新评估：
  - COM_DISARM_PRFLT：真机不放宽，保留PX4默认的起飞前置检测（原来-1是
    关闭这项检测，仿真里图方便，真机上过松是安全隐患）——直接不出现在
    PARAMS_HW里，不去设置它，PX4沿用固件自己的默认值。
  - COM_OF_LOSS_T：仿真给的5.0秒（OFFBOARD信号丢失容忍5秒才失控保护）
    对真机可能偏长，这里收紧到2.0——⚠️ 这个数字只是一个凭经验给的起点值，
    不确定是不是比PX4固件出厂默认值更保守，也没有针对真实WiFi/图传链路
    的延迟/丢包特性测过，真机联调时必须重新评估这个值，不要直接信任。
  - COM_DISARM_LAND：两种模式都保持-1不变——这条本身不是"仿真专用放宽"，
    是刻意让px4ctrl自己的落地检测器（位置/速度双约束保持3秒，
    PX4CtrlFSM.cpp里的land_detector逻辑）接管上锁时机，取代PX4自带检测器，
    真机上同样适用这个设计。但这个检测器本身在真实地面（不是Gazebo理想
    平面）的噪声下会不会误判，需要真机测试验证，不是这个节点能保证的。
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue

# (参数名, 目标值) —— 仿真模式数值和含义原样照抄自
# docker_sim/patches/ros2_px4_stack_dynus.patch 里已经在仿真里验证过的组合：
#   COM_DISARM_PRFLT=-1  关闭"起飞前置检测失败自动上锁"
#   COM_OF_LOSS_T=5      OFFBOARD信号丢失容忍时间放宽到5秒
#   COM_DISARM_LAND=-1   关闭"检测到已降落自动上锁"（AUTO_LAND状态机自己控制上锁时机）
PARAMS_SIM = (
    ("COM_DISARM_PRFLT", -1.0),
    ("COM_OF_LOSS_T", 5.0),
    ("COM_DISARM_LAND", -1.0),
)

# 真机模式：COM_DISARM_PRFLT不出现在这里，故意不设置，PX4沿用固件默认值。
# COM_OF_LOSS_T=2.0是未经真实链路验证的保守起点值。COM_DISARM_LAND跟仿真
# 保持一致，理由见上面docstring。完整取舍说明见文件头docstring
# "2026-08-14新增DEPLOY_TARGET区分"一节。
PARAMS_HW = (
    ("COM_OF_LOSS_T", 2.0),
    ("COM_DISARM_LAND", -1.0),
)

DEPLOY_TARGET = os.environ.get("DEPLOY_TARGET", "sim")
PARAMS = PARAMS_HW if DEPLOY_TARGET == "hw" else PARAMS_SIM

# 总预算120秒，跟ros2_px4_stack_kick_offboard_timeout.patch的range(240)*0.5s
# 是同一个数字来源（仿真real-time-factor可能远低于1，10~15秒真实时间在
# RTF=0.3下对应不到5秒仿真时间，PX4/mavros建链经常来不及）。
SERVICE_WAIT_TIMEOUT_SEC = 120.0
PER_CALL_TIMEOUT_SEC = 5.0
RETRY_INTERVAL_SEC = 1.0
MAX_RETRIES_PER_PARAM = 20


class Px4ParamRelax(Node):
    def __init__(self):
        super().__init__('px4_param_relax')
        self._cli = self.create_client(ParamSet, 'mavros/param/set')
        self.get_logger().info(
            f"DEPLOY_TARGET={DEPLOY_TARGET}，本次要设置的参数: {PARAMS}")

    def _set_one_param(self, name: str, value: float) -> bool:
        for attempt in range(1, MAX_RETRIES_PER_PARAM + 1):
            req = ParamSet.Request()
            req.param_id = name
            req.value = ParamValue()
            req.value.integer = 0
            req.value.real = float(value)

            future = self._cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=PER_CALL_TIMEOUT_SEC)

            if future.done() and future.result() is not None:
                self.get_logger().info(f"PX4参数放宽成功: {name} = {value} (第{attempt}次尝试)")
                return True

            self.get_logger().warn(
                f"PX4参数放宽失败/超时: {name} = {value} (第{attempt}/{MAX_RETRIES_PER_PARAM}次尝试)，"
                f"{RETRY_INTERVAL_SEC}s后重试")
            time.sleep(RETRY_INTERVAL_SEC)

        self.get_logger().error(f"PX4参数放宽彻底失败（重试{MAX_RETRIES_PER_PARAM}次仍未成功）: {name}")
        return False

    def run(self) -> bool:
        if not self._cli.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            self.get_logger().error(
                f"mavros/param/set 在{SERVICE_WAIT_TIMEOUT_SEC}s内不可用，放弃参数放宽——"
                "PX4可能还没连上MAVROS，或者MAVROS没有以正确的namespace启动。"
            )
            return False

        all_ok = True
        for name, value in PARAMS:
            if not self._set_one_param(name, value):
                all_ok = False
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = Px4ParamRelax()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
