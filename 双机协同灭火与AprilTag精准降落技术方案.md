# 双机协同灭火与 AprilTag 精准降落技术方案

> 2026-08-23。纯设计方案，尚未创建任何代码/世界文件——先摸清现有平台能力边界
> 再设计，避免凭空假设。背景：本项目当前无人机已具备定位（DLIO/UWB/GT三选一）、
> 导航、避障、规划能力（mighty/ego-planner二选一），但**相机、视觉检测、精准
> 降落、载荷机构均未落地**，是这次要新增的部分，不是"修 bug"。

---

## 〇、前置结论：现状核实（做设计前必须先知道的边界）

逐项来自对 `docker_sim` 代码的实际核查，不是推测：

| 项目 | 现状 | 证据 |
|---|---|---|
| 前视相机 | **未启用**。源码里 d435 相机存在，但被 `patches/mighty_disable_d435.patch` 整段注释掉 | 原因：`realsense_gazebo_plugin::RealSensePlugin::Load()` 有未判空解引用 bug，双机同时初始化时容易撞上传感器管理器时序竞态，导致 `gzserver` 段错误崩溃（引用上游 `pal-robotics/realsense_gazebo_plugin#17`） |
| 下视相机 | **从未存在**。`patches/mighty_sensor_mount.patch` 明确写"这里没有下视相机" | 同上 |
| 可用的相机宏 | `generic_camera.urdf.xacro` 存在但被注释，用 `libgazebo_ros_camera.so` 插件（比 realsense 插件简单、无已知崩溃 bug），但上游作者自己留了 `TODO: Not sure why but can't put them in namespace`——多机命名空间下这个宏有已知未解决问题 | `staging/mighty_ws_src/mighty/urdf/generic_camera.urdf.xacro` |
| AprilTag / 二维码检测 | **完全没有**，全仓库 0 命中 `apriltag`/`qrcode`/`pyzbar`/`cv_bridge` | 仅有 `libopencv-dev` 作为其他依赖的系统库 |
| 编程式发目标点接口 | **已有现成节点**：`rviz_goal_bridge_node.py`，订阅 `waypoint_queue`(`nav_msgs/msg/Path`)/`waypoint_cancel`，发布 `term_goal`(`geometry_msgs/msg/PoseStamped`) 驱动 **ego-planner**，抵达阈值 0.3m | `src/ego_planner_bridge/ego_planner_bridge/rviz_goal_bridge_node.py` |
| 双机通信 | **有先例**：`frame_align_bridge_node` 用 `lookup_transform` 跨命名空间查另一架飞机的 TF；`ROS_DOMAIN_ID` 固定 20 + `network_mode: host`，**一个节点可以直接订阅另一个命名空间的话题，不需要额外网桥** | `src/uwb_origin_bridge/uwb_origin_bridge/frame_align_bridge_node.py`；README.md |
| 精准降落 | **未集成**，只有 PX4 上游未启用的 irlock vendor 代码，项目自身 0 命中 `precland`/`PLD_` | 目前只有 MAVROS `AUTO.LAND`，无视觉/标志引导 |
| Gazebo world 资产 | 默认 `hard_forest.world`；**已有先例**直接改/新增 `.world` 文件加自定义静态模型，走 `git apply` 应用 patch 的既定模式 | `patches/mighty_forest_boundary_walls.patch`、`patches/mighty_simple_room_world.patch` |
| 载荷/抓取机构 | **完全没有**，无 gripper/投放舱；`f_max=26N` vs 悬停需 19N，约 1.4 倍余量**未专门为载荷预留** | `vehicle_profile.yaml` |
| 历史记录 | `TODO.md`/`DEBUG_JOURNAL.md` 无相关记录；此前一次调参会话中用户明确说"感知模块（HSV/YOLO/二维码）……真机才做"——是**有意延后**，不是没考虑过 | `DEBUG_JOURNAL.md` |
| 话题命名规律 | 单机私有：`/<NS>/<模块>/<话题>`（如 `/NX01/dlio/odom_node/odom`）；跨机：`/<功能>/<own_ns>/<other_ns>`（如 `/frame_align/NX01/NX02`），两种模式并存 | 多处实证 |

**结论**：这次要做的不是接一个现成模块，而是**从零搭建一条视觉感知链路 + 一套任务级状态机**，相机能不能稳定跑是两个子任务共同的前置阻塞项，必须先解决。因此整个方案按"先分层降低风险、再逐步补齐真实感知"的思路设计，不是一次性端到端实现。

---

## 一、总体设计原则

1. **复用"Gazebo 真值当传感器"的既有惯例**——本项目 UWB 定位本来就是读 Gazebo 真值模拟出来的（见 CLAUDE.md 顶层需求），这次沿用同一思路：先用 Gazebo 真值把"任务流程"（状态机、双机通信、装载、飞行、判定）跑通，视觉识别作为**可插拔的第二层**后补，两层用同一套下游接口，互不影响彼此的开发/验证进度。
2. **相机用简单的 `generic_camera` 宏，不修复 `realsense` 插件**——本任务只需要 2D 彩色图像做分类/解码，不需要深度信息，没必要去啃一个上游已知有崩溃 bug 的插件；`generic_camera.urdf.xacro` 更轻量、插件更简单，缺的只是命名空间适配，工作量和风险都更低。
3. **火情识别用 HSV 颜色阈值，不引入 YOLO**——窗户图像本身是"正常/着火"二选一（外加材料类型编码），颜色/纹理差异可以人为设计得足够大（比如火焰窗贴橙红色纹理，正常窗贴灰蓝色），HSV 阈值分类完全够用，跟此前"感知模块 HSV/YOLO 真机才做"的记录里 HSV 是预期的最简方案一致；YOLO 留给以后真机做真实火焰识别时再引入。
4. **载荷不做物理建模**——没有抓取机构，机身推力余量也没为载荷预留，物理级的"挂载/投放"改质心/改质量的工程量与当前任务目标（验证任务流程正确性）不成比例。载荷用**符号状态**模拟（飞机内部记一个"当前携带材料类型"的状态量，悬停在装载点足够时长即视为装载完成），如后续需要视觉可信度可以叠加一个纯视觉的挂载模型（不影响物理），本方案不做为必须项。
5. **降落阶段脱离常规规划器**——mighty/ego-planner 是为避障导航设计的，不适合做"厘米级视觉伺服对准"，精准降落沿用现有"起飞门"式的做法：规划器负责把飞机导航到降落点上方一定范围内（粗对准），再切换到一个专门的**近距离伺服控制器**接管最后阶段（细对准+下降），这个控制器同时也是任务一里"1米水平正对 3 秒"判定的技术基础，两个任务共享同一个组件。

---

## 二、共享基础设施：相机使能方案

两个任务共同的前置依赖，必须先做。

### 2.1 技术选型

用 `staging/mighty_ws_src/mighty/urdf/generic_camera.urdf.xacro`（`libgazebo_ros_camera.so` 插件）分别在 NX01 挂前视 + 下视两个实例，NX02 视需要挂下视一个实例（用于验证性解码地面二维码，见 4.5 节）。**不修复/不启用 realsense 插件**——2D 彩色图像已经够用，避免重蹈双机初始化时序崩溃的坑。

### 2.2 需要解决的具体问题

- **命名空间适配**：上游宏本身有 `TODO: Not sure why but can't put them in namespace`，需要新增一个 patch（如 `patches/mighty_generic_camera_namespace_fix.patch`），显式在宏参数里传入 `robotNamespace`/`topicName` 前缀，或者在 spawn 阶段用 `-remap` 的方式把话题名从默认值改成 `<ns>/front_camera/image_raw`、`<ns>/down_camera/image_raw`——具体做法参照本项目其它话题（如 `dlio/odom_node/odom`）已经验证过的命名空间拼接方式，保持风格一致。
- **分辨率/帧率**：分类和二维码/AprilTag 解码都不需要高分辨率，建议 640×480、10Hz，降低双机同时渲染两路相机对 GPU 的压力（GCS 部署篇已经记录过 Gazebo 渲染是 GPU 负载主要来源）。
- **稳定性验证**：双机同时起两路相机（NX01 前视+下视，NX02 下视，共 3 个相机插件实例）必须先做一轮纯稳定性冒烟测试（起飞前静止跑 5 分钟不崩溃），再进入功能开发，避免重蹈 `mighty_disable_d435.patch` 的教训。

### 2.3 新增话题（沿用 `/<NS>/<模块>/<话题>` 命名规律）

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/NX01/front_camera/image_raw` | `sensor_msgs/msg/Image` | 前视，火情识别用 |
| `/NX01/down_camera/image_raw` | `sensor_msgs/msg/Image` | 下视，可选，用于起飞点/地面标志辅助定位 |
| `/NX02/down_camera/image_raw` | `sensor_msgs/msg/Image` | 下视，材料二维码解码 + 降落 AprilTag 检测复用同一路相机 |

---

## 三、任务一：双机协同灭火

### 3.1 场景资产设计

**火情墙**：新建一面竖直墙（新增 Gazebo world 或在现有 `hard_forest.world` 旁新增一个专用场景，见 3.7 节），墙面按"楼层×每层若干窗户"划分成 N 个矩形贴图区（比如 4 层 × 3 列 = 12 个窗口），每个区域是一个独立的 `<visual>` 平面 + `<material><script>` 贴图。贴图设计成两类：

- **正常窗**：统一贴一种"灰蓝色"纹理；
- **着火窗**（仅 1 个）：贴橙红色/带烟雾纹理，且**用不同的色调编码对应 3 种灭火材料里的哪一种**（例如：橙红=木质火→材料A，蓝紫=电气火→材料B，暗黄=油类火→材料C）——这样"识别着火窗口"和"判断该用哪种材料"是同一次分类动作的两个输出，不需要额外的判断逻辑。

**地面材料装载点**：3 个二维码贴纸（生成 PNG，`qrencode`/Python `qrcode` 库即可），分别编码材料标识（如 `MAT_A`/`MAT_B`/`MAT_C`），贴在地面或立牌上，各自有固定的世界坐标。作为 Gazebo 平面模型的贴图纹理，跟墙面窗户贴图是同一种资产制作方式。

### 3.2 分层感知方案（关键设计）

| 层级 | 火情识别 | 材料定位 | 适用阶段 |
|---|---|---|---|
| **L0（真值层，先做）** | NX01 任务节点直接查 Gazebo `get_model_state`（或订阅 `/gazebo/model_states`）读"着火窗"标记模型的位姿和预先烤好的材料类型属性 | NX02 任务节点直接读取 3 个二维码模型的已知固定世界坐标（配置文件里静态写死，因为二维码贴纸本来就不会移动） | 用于先把任务流程（通信、状态机、装载、飞行、判定）跑通、验证正确性，不受视觉可靠性影响 |
| **L1（视觉层，后做）** | NX01 前视相机图像做 HSV 颜色分类，扫描到匹配"着火窗"色调的区域即输出火情位置+材料类型 | NX02 下视相机在装载点上方悬停，解码二维码（`pyzbar` 或 OpenCV `QRCodeDetector`）核对材料类型是否匹配 L0 的静态配置，作为真实感知的验证/替代 | 相机链路（第二章）稳定之后叠加，替换/校验 L0 的结果 |

L0 和 L1 对下游（通信协议、状态机、装载判定、飞行、完成判定）**接口完全一致**——都是"发布一条 `FireReport` 消息"和"提供一个材料坐标查询"，因此可以先用 L0 把整条链路跑通，再把感知实现换成 L1，中间不需要改任何下游代码。

### 3.3 新增 ROS2 消息定义

新增一个自定义接口包 `fire_mission_msgs`（跟项目里已有的 `quadrotor_msgs`、`traj_utils`、`dynus_interfaces` 同一种"自定义消息独立成包"的模式）：

```
fire_mission_msgs/msg/FireReport.msg
  std_msgs/Header header
  geometry_msgs/PoseStamped target_pose     # 着火窗世界坐标（用于导航+判定）
  uint8 material_type                       # 1=MAT_A, 2=MAT_B, 3=MAT_C
  string window_id                          # 调试用，比如"floor3_col2"

fire_mission_msgs/msg/MissionState.msg
  std_msgs/Header header
  string stage                              # "idle"/"searching"/"reported"/"loading"/"enroute"/"aiming"/"done"
```

### 3.4 双机通信协议

沿用 `frame_align_bridge_node` 已验证的跨命名空间订阅模式，不需要新增网桥容器：

- NX01 侧：任务节点 `nx01_recon_node` 完成识别后，在**自己的命名空间**发布一次 `/NX01/mission/fire_report`（`FireReport`，latched/`transient_local` QoS，保证 NX02 后起飞也能收到）；
- NX02 侧：任务节点 `nx02_firefight_node` **直接订阅** `/NX01/mission/fire_report`（硬编码伙伴命名空间，跟 `frame_align_bridge_node` 里"跨命名空间直接查另一架飞机"是同一种做法），不经过任何中间网关；
- 双方各自的 `MissionState` 发布在自己命名空间下（`/NX01/mission/state`、`/NX02/mission/state`），供 GCS 网页后续订阅展示（见第五章）。

### 3.5 NX02 任务状态机

```
IDLE
  └─(收到NX01的FireReport)→ TAKEOFF（复用现有起飞机制，见第一份GCS手册第五部分的起飞逻辑）
TAKEOFF → GOTO_MATERIAL_SITE
  └─ 用 rviz_goal_bridge_node 的 waypoint_queue 接口，发一个目标点=对应material_type的二维码坐标
GOTO_MATERIAL_SITE → LOADING
  └─ 到位（沿用 rviz_goal_bridge_node 的 0.3m 抵达阈值）后悬停 T_load 秒（如3秒）
  └─（L1层）下视相机解码二维码，核对材料类型；不匹配则报警/中止（异常处理，MVP阶段可先只做L0）
  └─ 悬停结束，任务节点内部状态量记 carrying_material = material_type（符号化装载，见3.6节）
LOADING → ENROUTE
  └─ 计算"站位点"：从 FireReport.target_pose 沿墙面法线方向后退1米、朝向对准墙面
     （见3.6节几何计算），作为新目标点发给 waypoint_queue
ENROUTE → AIMING
  └─ 到位后，规划器交出控制权，切换到"近距离伺服控制器"（第四章共享组件）做最终的
     位置+朝向精修，同时启动"1米水平正对3秒"判定定时器
AIMING → DONE（判定通过）或保持AIMING（判定中断则计时器清零重来）
```

### 3.6 站位点几何计算与完成判定

**站位点**：`target_pose` 是着火窗中心的世界坐标，墙面法线方向 `n`（垂直于墙面朝外，场景搭建时已知，写入配置）。站位点 = `target_pose.position - 1.0 * n`，朝向 = 指向 `target_pose` 的偏航角 `atan2(n.y, n.x)` 的反方向（即机头指着墙）。这个带朝向的目标点，通过 `term_goal`（`geometry_msgs/msg/PoseStamped`，本身就带 orientation 字段）发给 ego-planner——**需要确认 ego-planner 是否真的用了目标点里的 yaw**（`rviz_goal_bridge_node.py` 现有实现里这一点未经调研确认，见第六章风险项），如果规划器忽略 yaw，只能等飞机到位后由近距离伺服控制器（第四章）单独转到位。

**完成判定**（独立的 `mission_judge_node`，或作为 `nx02_firefight_node` 的一部分）：

- 水平距离 `d = ||(pos_xy - target_xy)||`（不看Z轴高度差，"水平正面瞄准"按XY平面理解）
- 朝向误差 `θ_err`：机头朝向与"机身位置指向着火点"方向的夹角
- 判定条件：`d ≤ 1.0m` 且 `θ_err ≤ 阈值（建议±10°）`
- 用一个简单的状态计时器：条件连续满足则计时器累加，一旦中断立即清零重新计时（"连续3秒"而非"累计3秒"，更符合"正面瞄准"的字面意思）；计时器达到 3.0s 判定 `DONE`，发布任务完成标志。

### 3.7 Gazebo world 资产改动清单

沿用 `patches/mighty_simple_room_world.patch` 新增整份 world 文件的先例，**建议新增一个专用场景**（如 `fire_drill.world`），而不是往 `hard_forest.world` 里塞——避免影响现有避障基准测试场景，通过 `WORLD_ENV` 环境变量切换（`docker/entrypoints/sim-world-entrypoint.sh` 已有此机制）：

- 火情墙模型（含 N 个可独立贴图的窗户面）
- 3 个二维码地面标志模型
- 1 个"火情标记"隐藏辅助模型（仅 L0 真值层用，`<static>true</static>`，无实体碰撞，模型名编码楼层/材料信息供 `get_model_state` 查询）
- 材料装载点、着火墙与飞机初始起飞点之间要留出规划器可通行的空间（避障链路要能正常工作，不能让墙体把可飞行空间切死）

### 3.8 载荷符号化模拟

`nx02_firefight_node` 内部维护一个字符串/整型状态 `carrying_material`（`0=空载`，`1/2/3=对应材料`），在 LOADING 阶段悬停满足时长后置位，AIMING 阶段完成时清零并广播（`MissionState.stage="done"`）。**不改变飞机物理质量/推力**，如果后续需要在 GCS 网页或 RViz 里可视化"挂着一个包裹"的效果，可以额外挂一个跟随飞机 TF 的纯视觉 marker（不参与物理），作为独立于本方案之外的锦上添花项，不建议第一阶段做。

---

## 四、任务二：AprilTag 精准降落

### 4.1 场景资产

新增一个降落点地面模型，贴一张 AprilTag（建议用 `tag36h11` 系列，生成方式：AprilTag 官方 `apriltag-imgs` 仓库或在线生成工具出 PNG，与二维码贴图同样的 Gazebo 材质贴图方式），位置在场景里预留出飞机能安全降落的平坦区域，世界坐标写入配置文件。

### 4.2 检测方案

用 `apriltag_ros`（ROS2 官方维护的 AprilTag 检测包，标准选型，比自己写检测算法更省工作量也更可靠）订阅 NX02（或需要精准降落的那架飞机）的下视相机话题 `/NX02/down_camera/image_raw`，输出检测到的 tag 相对相机的位姿（`apriltag_msgs/msg/AprilTagDetectionArray` 或类似类型，具体以 `apriltag_ros` ROS2 分支实际接口为准，需要在实施阶段核实并 vendor 进 `staging/`）。

**依赖新增**：`apriltag_ros` 及其依赖（`apriltag` C 库）需要新增到 `docker/Dockerfile.flight-stack`（或专门给降落功能新增的镜像分层），走本项目已有的"vendor 第三方 ROS2 包进 `staging/`，Dockerfile 里 `colcon build`"的既定模式（跟 GCS 镜像里编译 `decomp_util`/`quadrotor_msgs` 是同一套路）。

### 4.3 降落控制律：粗对准 + 视觉伺服两阶段

- **粗对准阶段**：跟任务一的"站位点"思路一致，先用 `rviz_goal_bridge_node` 的 `waypoint_queue` 接口把飞机导航到降落点正上方一定高度（如 2m）附近，允许较大误差（规划器的 0.3m 抵达阈值足够）。
- **精对准+下降阶段**：粗对准到位后，**规划器让出控制权**（这是需要新增的机制，见 4.4 节），切换到 `precision_land_node`：
  1. 订阅 `apriltag_ros` 输出的 tag 位姿，换算成"飞机当前相对 tag 中心的水平偏差 (Δx, Δy)"；
  2. 用一个简单的 P/PID 控制器，把 (Δx, Δy) 转成小幅度的水平速度/位置修正指令，通过 mavros `setpoint_position`/`setpoint_velocity` 直接下发（**绕开规划器**，类似 `px4ctrl`/`so3ctrl` 这类板外控制器直接对接 PX4 的方式）；
  3. 水平误差收敛到阈值内（如 <0.1m）后，开始按固定速率下降，下降过程持续用同样的水平修正逻辑纠偏（防止下降气流扰动导致偏移）；
  4. 接近地面（如 <0.3m，或触发 PX4 `LANDED` 状态判定）后切 `AUTO.LAND`/直接 disarm，完成降落。

这一套"粗规划器导航 + 精视觉伺服接管"的分层模式，跟任务一 3.6 节的"站位点+近距离伺服控制器"是**同一个技术模式**，建议实现为一个可复用的通用组件（如 `fine_approach_controller`），任务一的最终瞄准和任务二的精准降落都调用它，只是目标源（着火点固定位姿 vs. 实时 AprilTag 检测位姿）和终止条件不同。

### 4.4 与现有起降机制的整合

现有起降是"话题/service 一次性触发 + 飞控内部状态机执行"（`TakeoffLand` 话题或起飞门文件，见 GCS 操作手册第五部分），本身不支持"规划器导航到半途、中途转交给另一个控制器"这种模式。需要新增一个**"降落模式"标志位**（类似现有 `TAKEOFF_GATE_FILE` 的做法，或一个新话题 `/<NS>/mission/landing_mode`），`nx02_firefight_node`/`precision_land_node` 用它跟规划器桥接节点协商"何时暂停常规目标点跟踪、把控制权交给精准降落"，需要在 `ego_planner_bridge`（或对应 mighty 版本）里新增一个"暂停/恢复接受新 term_goal"的开关，避免精对准阶段规划器还在发新的路径指令跟视觉伺服打架。

---

## 五、与 GCS 网页地面站的集成（可选，建议放在最后阶段）

复用已有的地面站基础设施（见此前生成的《地面站系统介绍与操作手册》），不需要新增容器：

- 前端订阅 `/NX01/mission/state`、`/NX02/mission/state`、`/NX01/mission/fire_report`（rosbridge 直连，跟现有状态表格同一种做法），在飞机卡片旁新增一个"任务状态"展示；
- 地图面板可以叠加渲染火情墙位置、材料装载点、降落点这几个静态地标（新增几个已知坐标的标记点，画法参照现有障碍物栅格图叠加逻辑）；
- 若要在网页上手动触发任务（而不是自动开始），可以在 backend 新增一个简单的 `POST /api/mission/start` 接口，效仿现有起飞接口的 `docker exec` 转发模式。

---

## 六、风险与待确认项

| 风险/待确认项 | 说明 | 建议应对 |
|---|---|---|
| `generic_camera.urdf.xacro` 多机命名空间问题 | 上游遗留 TODO，具体报错/表现尚未实测 | 第二章列为独立冒烟测试阶段，先验证再往上叠功能 |
| `term_goal` 是否真的用了目标点的 yaw | `rviz_goal_bridge_node.py` 现有实现里这点未经调研确认 | 实施阶段先用一个简单脚本单独测试；不支持则站位朝向完全交给近距离伺服控制器处理 |
| mighty 规划器没有等价的编程式目标点接口 | `rviz_goal_bridge_node` 目前只服务 ego-planner（在 `ego_planner_bridge` 包下） | 若要求同时支持 `PLANNER=mighty`，需要新写一个对应桥接节点；本方案默认场景跑在 `PLANNER=ego_planner` 下 |
| 双机同时渲染 3 路相机的性能/稳定性 | 尚无实测数据，Gazebo 渲染已知是 GPU 负载主要来源 | 先跑纯稳定性冒烟测试（第二章），必要时降分辨率/帧率 |
| `apriltag_ros` 的 ROS2 Humble 分支具体接口/依赖 | 尚未调研确认具体消息类型和依赖链 | 实施阶段第一步先确认可编译可用，参照 GCS 镜像里 vendor 三方包的流程 |
| 精准降落与规划器控制权切换机制 | 目前没有"规划器暂停接受新目标点"这类开关，需要新增 | 4.4 节已给出设计思路，属于本方案里工作量较大的一块，建议单独排期验证 |
| 载荷是否需要视觉可见 | 当前方案是纯符号化，不挂视觉模型 | 如答辩/演示需要"看得见的包裹"，作为独立的锦上添花任务后补，不阻塞主线 |

---

## 七、实施路线图（建议分阶段，每阶段可独立验证）

1. **Phase 0 相机使能**：`generic_camera` 命名空间修复 + 三路相机双机稳定性冒烟测试
2. **Phase 1 灭火任务 MVP（L0真值层）**：world 资产（火情墙+二维码+隐藏标记）、双机通信协议、NX02 状态机、站位点几何、完成判定，全走 Gazebo 真值，不依赖视觉，验证任务流程正确性
3. **Phase 2 灭火任务视觉化（L1视觉层）**：HSV 火情窗口分类、二维码解码校验，替换/对照 Phase 1 的真值结果
4. **Phase 3 AprilTag 精准降落**：降落点资产、`apriltag_ros` 接入、近距离伺服控制器（与任务一 3.6 节共享组件）、规划器控制权切换机制
5. **Phase 4 GCS 网页集成**：任务状态可视化（可选，锦上添花）

每个阶段结束都有独立可演示的成果，不需要等全部完成才能看到效果。
