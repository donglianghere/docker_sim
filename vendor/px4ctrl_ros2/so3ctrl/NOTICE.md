# 版权/许可归属说明

`so3ctrl` 这个包是"px4ctrl 的飞行状态机 + kr_mav_control 的 SO3 几何位置环
控制律"这两份第三方代码的组合，外加 docker_sim 项目自己写的少量新代码
（`controller.cpp` 里的 SO3Control 调用/换算 glue、`so3ctrl_node.cpp` 的
命名空间适配）。三部分来源和许可证分别是：

| 文件 | 来源 | 许可证 |
|---|---|---|
| `src/PX4CtrlFSM.h`、`src/PX4CtrlFSM.cpp` | 逐字复制自 `docker_sim/vendor/px4ctrl_ros2/px4ctrl`（ZJU-FAST-Lab，ROS2/Humble移植版，原版来自 Fast-Drone-250/src/realflight_modules/px4ctrl），仅日志文本里的`[px4ctrl]`字面量改成`[so3ctrl]` | GPLv3 |
| `src/input.h`、`src/input.cpp` | 同上，逐字复制（`input.cpp`同样只改了几处`rclcpp::get_logger()`用到的字符串） | GPLv3 |
| `src/PX4CtrlParam.h`、`src/PX4CtrlParam.cpp` | 改编自 `px4ctrl`（新增`Ki0-2`/`Kib0-2`六个增益字段及对应读取，其余原样） | GPLv3 |
| `src/controller.h`、`src/controller.cpp` | 改编自 `px4ctrl`（外层类型/类名/函数签名不变，内部控制律实现改成调用`SO3Control`） | GPLv3 |
| `src/so3ctrl_node.cpp` | 改编自 `px4ctrl` 的 `px4ctrl_node.cpp`（节点名、相对话题名、docker_sim命名空间适配） | GPLv3 |
| `src/SO3Control.hpp`、`src/SO3Control.cpp` | 原样复制自 [KumarRobotics/kr_mav_control](https://github.com/KumarRobotics/kr_mav_control)（`humble`分支）的 `kr_mav_controllers` 包，未做任何逻辑改动，仅`#include`路径因拍平目录而改写 | BSD-3-Clause，见同目录 `LICENSE-kr_mav_control` |
| `config/ctrl_param_fpv.yaml` | 改编自 `px4ctrl`（新增`Ki0-2`/`Kib0-2`默认值，注释更新） | GPLv3 |

GPLv3 全文未在本仓库单独附一份（`docker_sim/vendor/px4ctrl_ros2/px4ctrl`
本身也没有——这是既有状况，不是本次新增的疏漏），标准文本见
<https://www.gnu.org/licenses/gpl-3.0.txt>。BSD-3-Clause是宽松许可、
兼容GPL，组合作品按更严格的GPLv3分发不违反BSD-3的任何条款（仅要求保留
版权声明，已保留在`LICENSE-kr_mav_control`里）。

`kr_mav_control` 本身的克隆位置：`/home/robots/ai_uav/kr_mav_control`
（项目根目录，`humble`分支，不在`docker_sim/`范围内，属于本次移植评估
过程中另外clone的参考仓库）。
