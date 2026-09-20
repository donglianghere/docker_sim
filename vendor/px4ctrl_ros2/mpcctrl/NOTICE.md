# 版权/许可归属说明

`mpcctrl`是三部分组合而成：uzh-rpg/rpg_mpc的MPC求解器核心、px4ctrl的
FSM/输入处理/参数框架（逐字复用，跟`so3ctrl`同一个套路）、本项目新写的
ROS2胶水层。来源和许可证分别是：

| 文件 | 来源 | 许可证 |
|---|---|---|
| `model/quadrotor_mpc_codegen/*` | 原样复制自 [uzh-rpg/rpg_mpc](https://github.com/uzh-rpg/rpg_mpc) 的 `model/quadrotor_mpc_codegen/`（ACADO工具链离线生成的求解器C代码，10状态/4输入/N=20步/dt=0.1s的四旋翼质点+姿态运动学模型），未做任何修改 | GPLv3（见`COPYING-rpg_mpc`） |
| `externals/qpoases/*` | 原样复制自 rpg_mpc 的 `externals/qpoases/`（qpOASES QP求解器），未做任何修改 | LGPLv2.1（见`externals/qpoases/LICENSE.txt`），与GPLv3兼容 |
| `include/mpcctrl/mpc_wrapper.h`、`src/mpc_wrapper.cpp` | 移植自 rpg_mpc 的 `include/rpg_mpc/mpc_wrapper.h`/`src/mpc_wrapper.cpp`，调用ACADO求解器的算法逻辑逐行保留，只改了namespace、把`ROS_ERROR`/`ROS_WARN`换成不依赖ROS的打印宏、把`acadoVariables`/`acadoWorkspace`两个全局变量的定义从头文件搬到.cpp（原版直接定义在头文件里，依赖老编译器"common symbol"合并行为，GCC10+默认`-fno-common`下会报multiple definition链接错误） | GPLv3 |
| `include/mpcctrl/mpc_controller.h`、`src/mpc_controller.cpp` | 移植自 rpg_mpc 的 `include/rpg_mpc/mpc_controller.h`/`src/mpc_controller.cpp`，`run()`/`setStateEstimate()`/`setReference()`/`updateControlCommand()`/`preparationThread()`/`setNewParams()`六个核心函数算法逻辑逐行保留；裁掉了原版`point_of_interest`(相机对准点)、`autopilot/off`(原版autopilot状态机专用回调)、`publishPrediction`(rviz可视化预测轨迹)三个ROS耦合功能，`quadrotor_common`的ROS消息封装类型换成本项目自己的`types.h`；新增`resetWarmStart()`公开方法供`controller.cpp`在状态切换时强制下一次求解重新收敛 | GPLv3 |
| `include/mpcctrl/mpc_params.h` | 移植自 rpg_mpc 的 `include/rpg_mpc/mpc_params.h`，参数名/默认值校验逻辑原样保留，取参数的API从`ros::NodeHandle`+`quadrotor_common::getParam`换成`rclcpp::Node::declare_parameter`/`get_parameter` | GPLv3 |
| `include/mpcctrl/types.h` | 本项目新写，替代原版依赖ROS消息的`quadrotor_common::{QuadStateEstimate,Trajectory,TrajectoryPoint,ControlCommand}` | 本项目 |
| `include/mpcctrl/PX4CtrlFSM.h`、`src/PX4CtrlFSM.cpp` | 逐字复制自`docker_sim/vendor/px4ctrl_ros2/px4ctrl`，仅日志文本里的`[px4ctrl]`字面量改成`[mpcctrl]` | GPLv3 |
| `include/mpcctrl/input.h`、`src/input.cpp` | 同上，逐字复制（含`rclcpp::get_logger("px4ctrl")`→`"mpcctrl"`） | GPLv3 |
| `include/mpcctrl/PX4CtrlParam.h`、`src/PX4CtrlParam.cpp` | 同上，逐字复制，字段/校验逻辑一个没改（`gain.*`/`rotor_drag.*`这些mpcctrl控制律用不到的字段也原样保留，因为`read_essential_param()`调用列表没删减） | GPLv3 |
| `include/mpcctrl/controller.h`、`src/controller.cpp` | 改编自`px4ctrl`/`so3ctrl`的同名文件（外层`Desired_State_t`/`Controller_Output_t`/类名`LinearControl`/`calculateControl`/`estimateThrustModel`/`resetThrustMapping`签名不变，这样`PX4CtrlFSM.h`不用改一个字），内部实现换成调用`mpcctrl::MpcController`；构造函数比px4ctrl/so3ctrl原版多一个`rclcpp::Node*`参数，专门用来加载MPC自己的代价矩阵/限幅参数(`mpcctrl::MpcParams::loadParameters`)——`PX4CtrlFSM.h`从不构造`LinearControl`，这个签名改动零耦合；加速度-油门自适应估计(RLS)算法逐行照搬px4ctrl/so3ctrl的`estimateThrustModel`/`computeDesiredCollectiveThrustSignal`/`resetThrustMapping` | GPLv3 |
| `src/mpcctrl_node.cpp` | 改编自`px4ctrl_node.cpp`/`so3ctrl_node.cpp`（订阅/发布/服务/主循环结构逐字照搬，只改节点名"mpcctrl"、调试话题"/debugMpcctrl"、`LinearControl`构造多传`this`） | 本项目 |
| `config/mpc_ctrl_param.yaml`、`launch/mpcctrl.launch.py` | 本项目新写：上半段(`Parameter_t`要求的字段)照抄`px4ctrl/config/ctrl_param_fpv.yaml`默认值，`use_bodyrate_ctrl`改成`true`；下半段(MPC自己的Q/R/限幅)参考rpg_mpc原版`parameters/`目录示例；launch文件remap约定跟`px4ctrl/launch/run_ctrl.launch.py`一致 | 本项目 |

GPLv3全文见`COPYING-rpg_mpc`，LGPLv2.1全文见`externals/qpoases/LICENSE.txt`。
qpOASES是宽松的LGPL许可、兼容GPL，组合作品按更严格的GPLv3分发不违反
LGPL的任何条款（仅要求保留版权声明，已保留在原始文件头注释里）。

## 架构说明：为什么直接复用px4ctrl的FSM，而不是重新写一套

用户要求"补上完整的FSM"时，参考了`so3ctrl`已经验证过的现成模式：
`PX4CtrlFSM`只通过`LinearControl &controller`这一个类型名跟控制律耦合，
从不关心`LinearControl`内部到底是P+D线性近似(px4ctrl)、SO3几何控制律
(so3ctrl)还是MPC(这次的mpcctrl)——只要`calculateControl`/
`estimateThrustModel`/`resetThrustMapping`三个方法签名不变，`Controller_
Output_t`带着`q`/`bodyrates`/`thrust`三个字段，`PX4CtrlFSM.h`/`.cpp`就可以
完全不碰、直接复用。这样`AUTO_HOVER`/`AUTO_TAKEOFF`/`AUTO_LAND`/
`MANUAL_CTRL`状态切换、RC安全开关联动、arm/disarm、odom/imu/rc/cmd/bat
全套超时检查，这些经过px4ctrl/so3ctrl两次真机验证的逻辑，mpcctrl一次
没有重新发明就直接继承了下来。

`use_bodyrate_ctrl: true`（跟px4ctrl默认值`false`不同）让FSM走
`publish_bodyrate_ctrl()`那条路径——这个函数本来就在`PX4CtrlFSM.cpp`里
存在（`IGNORE_ATTITUDE`+body_rate+thrust），MPC的输出恰好完全匹配这个
既有接口，不需要新增任何发布逻辑。

## 已知缺口（上真机前必须处理，不是"锦上添花"）

1. **FSM代码本身是复用px4ctrl的成熟实现，但"MPC控制律+这套FSM"这个
   组合从没有实际跑过一次**——状态切换时机（比如`AUTO_TAKEOFF`爬升阶段
   des.a给一条指数曲线、`resetThrustMapping()`同时触发`mpc_->
   resetWarmStart()`强制ACADO重新从悬停解收敛）对MPC这种有内部预测
   状态/热启动机制的控制器是否表现正常，只是逻辑上说得通，没有实测过。
2. **odom丢失时的兜底策略未经验证**：`odom_is_received()`超时后FSM会
   自动退回`MANUAL_CTRL`并退出OFFBOARD，这条路径本身在px4ctrl/so3ctrl
   上真机验证过，但从没有专门测过"MPC控制下触发这条退出"时的实际表现。
3. **限幅参数(`max_bodyrate_xy/z`、`min/max_thrust`)、代价矩阵(Q/R)全部
   是从rpg_mpc示例配置抄来的初值**，没有针对本项目真机(质量/转动惯量)
   重新标定或哪怕粗调过，直接拿去飞大概率姿态响应过软或过硬。
4. **ACADO codegen模型固定10Hz控制频率**（`dt=0.1s`），如果需要更高频率
   闭环，需要重新跑一遍ACADO代码生成（改`model/`下的建模脚本，这次移植
   范围内没有做，也没有拿到rpg_mpc原始的ACADO建模Python/MATLAB脚本，
   只搬了生成后的C代码产物），跟FSM本身100Hz的`ctrl_freq_max`主循环
   不一致——`controller.calculateControl()`每次被FSM调用都会触发一次
   完整的MPC `run()`（含ACADO求解），实际有效替换频率被ACADO内部
   `dt=0.1s`的假设间接限制，这次没有验证100Hz调用节奏对ACADO热启动/
   `preparation_thread_`并发调度会不会有问题（比如上一次`run()`还没
   完全结束、下一次100Hz的调用就来了）。
5. **odom"内容新鲜度"外推没有接入这一版**：mpcctrl第一版(纯裸控制回路，
   没有FSM)做过的新鲜度外推实验(DEBUG_JOURNAL 2026-09-13"补测mpcctrl"
   那条记录)是独立于FSM的一次性验证，这次接入完整FSM时用户明确说
   "先不管这个新鲜度了"，`controller.cpp`没有做任何odom陈旧度检测/
   外推——`odom_is_received()`延续px4ctrl原有的"消息到达心跳"检查
   （不检查内容新鲜度），这跟这次会话前几天诊断出的"px4ctrl对内容
   陈旧完全没有防护"是同一个已知问题，mpcctrl目前也没解决它，只是
   暂缓处理。
6. 已经在本机(`/opt/ros/jazzy`原生colcon)编译、运行、端到端验证过
   MPC核心本身（静止悬停求解正确、PositionCommand方向一致性正确），
   但这次新增的FSM接入代码（`controller.cpp`的构造函数、
   `resetThrustMapping`里的`resetWarmStart`调用、`mpcctrl_node.cpp`的
   完整订阅/发布/服务列表）**这次没有重新跑colcon build验证**，需要
   下次连真机/仿真前先补一次纯编译验证。
7. 完全没有在仿真容器(Gazebo+PX4 SITL)或真机上跑过，只是本地纯逻辑
   验证，离"能试飞"还有很长距离，第一次实机测试必须有安全飞手持
   遥控器随时能接管。
