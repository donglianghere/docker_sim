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
from rcl_interfaces.msg import ParameterType, ParameterValue

# 2026-10-17：mavros 的参数服务有两代接口，而且**服务发现按"名字+类型"匹配**，
# 类型对不上 wait_for_service 就永远返回 False——本仓库原来写死旧的 ParamSet，
# 而仿真容器里 mavros 提供的是 ParamSetV2，于是这个节点从来没成功过一次，
# 三个保护参数一次都没设上去。后果最严重的是 COM_DISARM_LAND 保持默认 2.0 秒
# （"检测到已降落 2 秒自动上锁"），仿真时间跳变让 PX4 的 land detector 误判
# "已着陆"时，飞机就被直接上锁掉在地上（实测 NX01/NX02 都这么掉过）。
# 这里两代都建 client、谁可用用谁，免得改完仿真反过来把真机弄坏。
try:
    from mavros_msgs.srv import ParamSetV2
except ImportError:                     # 老版本 mavros 没有 V2
    ParamSetV2 = None

# (参数名, 目标值) —— 仿真模式数值和含义原样照抄自
# docker_sim/patches/ros2_px4_stack_dynus.patch 里已经在仿真里验证过的组合：
#   COM_DISARM_PRFLT=-1  关闭"起飞前置检测失败自动上锁"
#   COM_OF_LOSS_T=5      OFFBOARD信号丢失容忍时间放宽到5秒
#   COM_DISARM_LAND=-1   关闭"检测到已降落自动上锁"（AUTO_LAND状态机自己控制上锁时机）
#
# ⚠️ **顺序有意义**：第一轮每个参数只试 FIRST_PASS_RETRIES 次（见下面那段说明），
# 所以**越要紧的排越前**，已知"mavros 参数表还没拉到"的那种排最后。
# 2026-10-18 实测：COM_DISARM_PRFLT 排第一时把 20 次配额全耗在
# `PR: Unknown parameter to set` 上、**整批参数被它堵了 20 秒**才轮到后面几个
# （那轮 21:42:15 才设上 EKF2_RNG_CTRL=0，而起飞是 21:42:33，只剩 18 秒余量）。
PARAMS_SIM = (
    # 2026-10-18 兜底，排第一位：没有它，EV 一停摆飞机就被 PX4 放到地上。见下面长注释。
    ("NAV_RCL_ACT", 0),
    ("COM_DISARM_LAND", -1.0),
    ("COM_OF_LOSS_T", 5.0),
    # 2026-10-17：关掉 EKF2 的测距高度融合。**整数型参数**，所以写成 int 字面量
    # （下面按 Python 类型选 PARAMETER_INTEGER / PARAMETER_DOUBLE）。
    #
    # 为什么关：场地里那个 0.6 m 高的仿地模块(terrain_module @17.0,7.5)正压在
    # 航线 G(17,12.5)->D(17,3) 下面。EKF2_RNG_CTRL=1 是"条件启用测距辅助高度"，
    # 判据是 EKF2_RNG_A_HMAX=5.0 m —— 巡航高度 2 m 远在阈值内，所以 range aid
    # 全程激活。飞过模块那一下测距从 2.0 突变到 1.4，EKF2 当成"飞机掉了 0.6 m"，
    # 高度估计跳变 → 位置被判不可信 → PX4 从 OFFBOARD 一路降级到 ALTCTL(nav_state=1)
    # → 没有遥控器输入就 `invalid setpoints` → `Failsafe: blind land` 把飞机放下去。
    # 实测两机都是在仿地模块附近挂的（NX01 最近 0.44 m，就停在它旁边）。
    #
    # 为什么能关：本套配置 EKF2_HGT_REF=3(Vision)，高度参考本来就是外部视觉，
    # 不靠测距；而"仿地飞行"是 line_goto 自己读 mavros/hrlv_ez4_pub 在上层实现的
    # （见 project_altitude_chain_real），不需要 EKF2 掺和。两者互不影响。
    ("EKF2_RNG_CTRL", 0),
    # ⚠️ 这一项排最后是**故意的**：它是这张表里**最晚才能设上**的一个。mavros 的
    # 参数表是启动后边跑边从 PX4 拉的，拉到它之前任何 set 都回
    # `PR: Unknown parameter to set`——2026-10-17/18 两轮实测，它排第一时第一轮
    # 必定全败，而**补设那一轮第 1 次就成**（固件里有这个参数，不是缺失；
    # 我一度据第一轮的失败误判成"固件没有它"，别再这么推）。
    # 排最后 + 第一轮限 3 次，它就不会再占着别人的时间。
    ("COM_DISARM_PRFLT", -1.0),
)

# 2026-10-18 `NAV_RCL_ACT=0` 的来历，以及**为什么这里没有 `EKF2_EV_DELAY`**：
#
# 现象：飞行中途飞机**无故降落**——NX02 在编队段从 2.15 m 以 0.35->0.7 m/s
# 降到地面，NX01 同一轮也中招但窗口短、撑住了。
# 链条（实测，不是推断）：
#   EKF2 停止融合 vision（cs_ev_pos/cs_ev_hgt/cs_ev_yaw 1->0）
#   -> vehicle_local_position.xy_valid/v_xy_valid -> 0
#   -> mode OFFBOARD -> **ALTCTL**（estimator_status 同秒 pos_horiz_rel=false）
#   -> 没有遥控器输入，PX4 就把飞机往下放
# 根因：`estimator_aid_src_ev_pos`/`_ev_hgt` 整段空白 15.7 秒（EKF2 一条 EV 样本
# 都没取），而 `vehicle_visual_odometry` 同期**一直 33.3 Hz 在到**（最大间隔 48 ms、
# reset_counter 不变、test_ratio 全程 0.00、innovation ±2 cm）——**数据到了 uORB，
# EKF2 就是不用**。因为 EV 样本的 `timestamp_sample` 比发布时刻**超前**，实测从 0
# 爬到 **+86 ms**（mavros↔PX4 的时间偏移在锯齿漂移，每 ~51 秒被 timesync 重置拉回
# 一次，日志里就是那条反复出现的 `TM: Time jump detected.`；根子是 lockstep 仿真
# 时钟比宿主机真实时钟慢）。EKF2 取样是 `pop_first_older_than(imu_delayed)`，
# 样本一旦超前出 fusion horizon 就**永远取不到**，于是整个 EV 源停摆。
# 实测停摆 >4 秒就会掉 OFFBOARD；NX01 18 分钟内停摆 10 次（1.4~11.1 秒），
# **这是周期性必然事件，不是偶发**。
#
# 🔴 **上游根因是两套参考时间源**（用户点出，已定量证实）：PX4 SITL 是 lockstep，
# 时钟由 Gazebo 步进驱动；ROS 侧节点用墙钟 epoch。实测 **RTF=0.9992**，两套时钟
# 以 −0.83 ms/s 分离 = 每 51 s 差 −42 ms，与 ulog `timesync_status` 实测的
# "每 51 s 跳 −52 ms"台阶、mavros 每 51 s 一条 `Time jump detected` 完全对上。
# mavros 把墙钟戳换算成 PX4 时钟戳，漂移累积到阈值就一次性阶跃 ~50 ms，
# 阶跃瞬间 vision 的时间戳跳 ±100 ms ⇒ 样本出窗 ⇒ EV 停摆。
#
# ❌ **`EKF2_EV_DELAY=150` 试过，是错的，已回退**（2026-10-18 当天两轮实测）：
# 偏移是**双向**的（实测 min −101.8 / p50 −9.5 / p95 +52.4 / max +117.3，
# **跨度 219 ms 已经超过 `EKF2_DELAY_MAX=200`**），EV_DELAY 只是把安全区间
# **平移**、不会**扩大**它：
#   · EV_DELAY=0  时正偏移(+86/+117) 让样本**超前**出窗 ⇒ 停摆 ⇒ NX02 掉地；
#   · EV_DELAY=150 时负偏移 −98 叠加成 **−248 ms**、超出 200 ms buffer，样本因
#     **太旧**被丢 ⇒ 照样停摆 17.8 s ⇒ NX01 从 1.94 m 掉到 0.00 m。
#     而同样的 −98 ms 在 EV_DELAY=0 下**完全无害**——等于把这个方向从无害改成致命。
# 教训：**平移一个双向抖动的区间之前，先量它的跨度**。要治这条，得
# `EKF2_DELAY_MAX` 加到 400 再配 EV_DELAY=150（⚠️ DELAY_MAX 是 reboot-required，
# 运行中设不生效，得写进 airframe 文件 `10015_gazebo-classic_iris`），或者干脆
# 统一时间源（见 gen_iris_mid360_sdf.py 的 set_lockstep()）。
#
# ✅ 所以这里改成**兜底**：`NAV_RCL_ACT` 出厂是 **2（Return）**——位置一失效、
# ALTCTL 下又没有遥控器，RC-loss 动作想 RTL 却没有位置可用，于是落到
# `Failsafe: blind land` 把飞机放下去。改成 0（Disabled）：仿真里本来就没有
# 遥控器，RC 丢失不该触发任何动作；EV 再停摆时飞机只是保持，不会被放到地上。
# （`COM_POSCTL_NAVL` 已经是 0、`COM_RCL_EXCEPT` 是 0，都不用动。）
# ⚠️ **整数参数**，写 0 不是 0.0。
# ⚠️ **只给仿真**：真机的 RC-loss 保护是真实安全功能，绝对不要进 PARAMS_HW。

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
# 第一轮每个参数只试这么几次。mavros 的参数表是启动后边跑边从 PX4 拉的，拉全
# 之前对未知参数会直接回 `PR: Unknown parameter to set`——**第一轮的目的只是
# "能设的赶紧设上"**，设不上的一律留给第二轮（那时表早就拉全了）。
#
# 为什么不是沿用 20 次：重试是**串行**的，排在前面的参数每失败一次就卡
# RETRY_INTERVAL_SEC，20 次 = 20 秒，期间后面的参数一个都没设上。
# 2026-10-17 实测 COM_DISARM_PRFLT 第一轮 20 次全败（mavros 参数表还没拉到它），
# 2026-10-18 实测后果：整批参数被它堵到 21:42:15 才设上，而飞机 21:42:33 就起飞，
# 只剩 18 秒余量 —— 起飞早一点，EKF2_RNG_CTRL=0 / EKF2_EV_DELAY 就赶不上。
# 光靠"把它排到最后"治不了本：哪天再往前面加一个"表拉全前设不上"的参数就复发。
#
# ⚠️ 这个改动**不会让参数更早落地**：瓶颈是 mavros 把参数表拉全要 ~27 秒，不是
# 重试策略（2026-10-18 改后实测：节点 22:18:46 起，第一轮 5 项全败到 22:18:56，
# 等 10 秒，补设轮 22:19:13 全部成功 = 启动后 27 秒，跟改之前的 28 秒一样）。
# 换来的是**健壮性**：任何一项都不能再独占 20 秒配额、要紧的参数排在最前。
# 真要更早落地，得改成"等 mavros 参数表就绪事件再设"，而不是盲目重试。
FIRST_PASS_RETRIES = 3
# 第一轮没设上的参数，隔这么久再补一轮（这时给满 MAX_RETRIES_PER_PARAM 次）。
SECOND_PASS_DELAY_SEC = 10.0


class Px4ParamRelax(Node):
    def __init__(self):
        super().__init__('px4_param_relax')
        # 两代接口各建一个 client，run() 里按实际可用的那个走
        self._cli_v2 = (self.create_client(ParamSetV2, 'mavros/param/set')
                        if ParamSetV2 is not None else None)
        self._cli_v1 = self.create_client(ParamSet, 'mavros/param/set')
        self._cli = None                # run() 里定下来用哪个
        self._use_v2 = False
        self.get_logger().info(
            f"DEPLOY_TARGET={DEPLOY_TARGET}，本次要设置的参数: {PARAMS}")

    def _set_one_param(self, name: str, value: float,
                       max_retries: int = MAX_RETRIES_PER_PARAM) -> bool:
        for attempt in range(1, max_retries + 1):
            # PX4 参数分 INT32 和 FLOAT 两种，类型发错会被拒（或静默取错值）。
            # 这里按 Python 字面量的类型决定：写 0 是整数参数，写 0.0 是浮点参数。
            is_int = isinstance(value, int) and not isinstance(value, bool)
            if self._use_v2:
                req = ParamSetV2.Request()
                req.force_set = False
                req.param_id = name
                req.value = ParameterValue()
                if is_int:
                    req.value.type = ParameterType.PARAMETER_INTEGER
                    req.value.integer_value = int(value)
                else:
                    req.value.type = ParameterType.PARAMETER_DOUBLE
                    req.value.double_value = float(value)
            else:
                req = ParamSet.Request()
                req.param_id = name
                req.value = ParamValue()
                if is_int:
                    req.value.integer = int(value)
                    req.value.real = 0.0
                else:
                    req.value.integer = 0
                    req.value.real = float(value)

            future = self._cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=PER_CALL_TIMEOUT_SEC)

            res = future.result() if future.done() else None
            # 只看"有回包"不够：飞控拒绝时照样回包，success=False。真机那次
            # 执行器参数的教训就是全链路报成功而值没进去。
            if res is not None and getattr(res, 'success', True):
                self.get_logger().info(f"PX4参数放宽成功: {name} = {value} (第{attempt}次尝试)")
                return True

            last = attempt == max_retries
            self.get_logger().warn(
                f"PX4参数放宽失败/超时: {name} = {value} (第{attempt}/{max_retries}次尝试)，"
                + ("留到补设那一轮再试" if last else f"{RETRY_INTERVAL_SEC}s后重试"))
            if not last:                # 最后一次失败就别再白等一个间隔
                time.sleep(RETRY_INTERVAL_SEC)

        self.get_logger().error(f"PX4参数放宽彻底失败（重试{max_retries}次仍未成功）: {name}")
        return False

    def run(self) -> bool:
        # 轮着等两代接口：总预算不变，谁先出现用谁
        deadline = time.time() + SERVICE_WAIT_TIMEOUT_SEC
        while time.time() < deadline:
            if self._cli_v2 is not None and self._cli_v2.wait_for_service(timeout_sec=1.0):
                self._cli, self._use_v2 = self._cli_v2, True
                self.get_logger().info("mavros 参数服务接口=ParamSetV2")
                break
            if self._cli_v1.wait_for_service(timeout_sec=1.0):
                self._cli, self._use_v2 = self._cli_v1, False
                self.get_logger().info("mavros 参数服务接口=ParamSet（旧版）")
                break
        if self._cli is None:
            self.get_logger().error(
                f"mavros/param/set 在{SERVICE_WAIT_TIMEOUT_SEC}s内不可用（ParamSetV2/ParamSet 都没等到），"
                "放弃参数放宽——PX4可能还没连上MAVROS，或者MAVROS没有以正确的namespace启动。"
            )
            return False

        failed = []
        for name, value in PARAMS:
            # 第一轮只给 FIRST_PASS_RETRIES 次：谁也别把后面的参数堵在队列里
            if not self._set_one_param(name, value, FIRST_PASS_RETRIES):
                failed.append((name, value))
        if not failed:
            return True

        # 补一轮：此时后面的参数都设过了，mavros 的参数表早已拉全
        self.get_logger().warn(
            f"第一轮有 {len(failed)} 个参数没设上（{[n for n, _ in failed]}），"
            f"等 {SECOND_PASS_DELAY_SEC}s 让 mavros 参数表拉全后再补一轮")
        time.sleep(SECOND_PASS_DELAY_SEC)
        all_ok = True
        for name, value in failed:
            if not self._set_one_param(name, value):
                self.get_logger().error(f"补设仍失败: {name}")
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
