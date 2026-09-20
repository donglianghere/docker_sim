# 真机部署与 Docker 开发知识梳理

> 本文档梳理 2026-08-14 关于"docker_sim 双机仿真系统移植到 Nvidia Jetson Orin
> NX Super 16G（JetPack 6.2）真机"的讨论内容：既包含具体的部署方案，也包含
> 讨论过程中涉及的 Docker / Linux / ROS2 基础知识点，按主题重新组织，不是
> 按时间顺序的问答流水账。逐字的原始问答记录见 `DEBUG_JOURNAL.md`
> 2026-08-14 当天的几条记录。

---

## 一、真机部署总体方案

### 1.1 角色划分

- **本机（Ubuntu 24.04）= 地面站（GCS）**，不跑 `flight-stack` 容器。
- **单机阶段**：1 台 Jetson Orin NX Super 16G 跑一份 `flight-stack`
  （`NUM_AGENTS=1`，相当于现有的 `flight-stack-nx01`）。
- **双机阶段**：2 台 Jetson 各自独立跑 `flight-stack`，GCS + 2 架飞机共 3 台
  物理机，ROS2 DDS 要跨真实网络发现彼此。
- `sim-world` 服务/仿真概念全部不上真机。

### 1.2 分阶段路线图

| 阶段 | 目标 | 退出判据 |
|---|---|---|
| 0（现状） | 仿真侧收尾 | `uwb_slam`×(`mighty`/`ego_planner`)×(`px4ctrl`/`so3ctrl`) 4种、`uwb_imu`×(`mighty`/`ego_planner`)×`ros2_px4_stack` 2种共6种组合实测通过 |
| 1 | Jetson 系统级准备 | JetPack 6.2 烧录完成，Docker/网络/时钟就绪 |
| 2 | 镜像/代码改造落地 | `flight-stack` 能在 Jetson 上 build 成功 |
| 3 | 地面站侧改造 | GCS 能跨机看到 Jetson 上的 ROS2 图（或用 NoMachine 绕开这一步） |
| 4 | 单机裸机联调（不装桨） | 真实 PX4/雷达/UWB 全部联通，DLIO 真机收敛 |
| 5 | 单机地面/低空悬停试飞 | 稳定悬停+起降 |
| 6 | 真实 UWB 一键定原点验证 | 精度达标 |
| 7 | 双机组网扩展 | 双机同域协同飞行 |
| 8（后续） | 前视/下视相机 | 当前代码库完全未实现，独立立项 |

### 1.3 真机首飞确认组合

```
LOCALIZATION_SOURCE=single_uwb_slam
PLANNER=ego_planner
CONTROLLER=px4ctrl
SLAM_BACKEND=dlio
```
选择依据：这是少数几个端到端实测过的组合之一；同期新加的 `pt4ctrl`、
`point_lio`、UWB+IMU 高频融合"路线A"当时都还在调试中，不建议真机首飞使用。

### 1.4 Docker + NVIDIA Container Toolkit 安装（JetPack 6.2）

flight-stack 本身（DLIO/mighty/ego_planner/px4ctrl）全部是纯 CPU 算法，
不依赖 GPU，这一步是为后续前视/下视相机阶段预留。

```bash
# Docker Engine + compose v2插件
sudo apt update
sudo apt install -y docker.io docker-compose-plugin
sudo usermod -aG docker $USER && newgrp docker
docker compose version

# NVIDIA Container Toolkit（走JetPack自带的L4T APT源）
sudo apt install -y nvidia-container-toolkit   # 具体包名需在真实设备上核实

# 配置Docker默认runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 验证（Jetson没有传统nvidia-smi，用l4t-base镜像验证）
sudo docker info | grep -i runtime
sudo docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:r36.4.0 uname -a
```

### 1.5 真机侧（Jetson）改造清单

| 改动点 | 现状（仿真） | 真机改法 |
|---|---|---|
| `docker-compose.hw.yml`（新建） | 三容器（sim-world+2个flight-stack） | 单容器，去掉GPU reservations，串口设备`devices:`直通 |
| `fcu_url` | SITL UDP端口公式 | 真实串口，如 `/dev/ttyTHS1:921600` |
| `INIT_X/Y/Z` | 按`AGENT_INDEX*3`算，喂给`world_mocap->init_pose`静态TF | 单机固定 `0/0/0` |
| CycloneDDS网卡锁定 | 锁死 `lo`（仅同机容器发现） | 换真实网卡名，加GCS静态IP作Peer |
| 真实Mid-360驱动 | 未启动（仅编译进镜像备用） | 新增 `msg_MID360_launch.py` 启动步骤，`xfer_format`改`0`（PointCloud2）对齐DLIO预期，remap到`${NAMESPACE}/mid360_PointCloud2`+`${NAMESPACE}/mid360/imu` |
| 真实UWB驱动 | 未启动 | 新增 `nlink_uwb_bridge.launch.py` 启动，补 `source /opt/nlink_ws/install/setup.bash`（当前sourcing链遗漏） |
| `no_RC` | `px4ctrl_docker_sim.launch.py`硬编码`True` | 改`False`，配合真实遥控器核对`RC_MAP_*`通道映射 |
| `px4_param_relax` | 无条件放宽`COM_DISARM_PRFLT=-1`/`COM_OF_LOSS_T=5.0`/`COM_DISARM_LAND=-1` | 真机上都是真实失控保护开关，需逐条重新评估，建议单独一份`px4ctrl_hw.launch.py` |
| `vehicle_profile.yaml` | 仿真iris数值（mass 1.935kg/hover_thrust 0.671） | 真机实测重新标定 |

### 1.6 本机侧（GCS）改造方案

- **ROS2环境**：Ubuntu 24.04 的 apt 源没有 ROS2 Humble 的官方二进制包
  （Humble 只对应 22.04），原生装不了——实际可行路径是在 GCS 上也跑一个
  **轻量容器**（基于 `ros:humble-ros-base-jammy`，只装ROS2底座+这个项目的
  自定义消息包，不编译DLIO/mighty等算法实现），`network_mode: host` 让它
  用宿主机真实网卡参与DDS发现。
- **DDS配置**：`PLANNER=ego_planner` 会强制 Jetson 端用
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，GCS 必须用同样的RMW+同
  `ROS_DOMAIN_ID`。单机阶段（2台主机）可以先试同域组播，若WiFi AP屏蔽
  组播就配CycloneDDS静态Peer列表；双机阶段（3台主机）建议直接上
  **Fast-DDS/CycloneDDS Discovery Server**，把"两两互连"简化成"大家都连
  一个中心"。
- **QGroundControl**：建议接独立数传链路，不与机载WiFi数据链复用，避免
  单点故障。
- **运维脚本**：`launch_control.py`/`status_monitor.py`等纯rclpy脚本原理上
  可以原样在GCS本机跑，但要去掉所有`docker exec`/`docker cp`（那是假设
  Docker和目标容器同机的仿真专属写法），改成直接走DDS话题/service call，
  或SSH到Jetson执行。

### 1.7 GCS 监控方式的两个选项

| 方案 | 优点 | 缺点 |
|---|---|---|
| 跨机 DDS（Discovery Server） | 可编程、可同时监控多机、GCS本机能跑自己的控制脚本 | 需要解决跨机DDS配置，仿真期间从未验证过 |
| NoMachine 远程桌面进 Jetson | 完全绕开DDS跨机发现问题，网络上传的只是桌面画面 | Jetson本身CPU已经比较满（DLIO/ego_planner），不适合长期跑重型3D可视化（如rviz2点云），适合轻量文本监控 |

单机阶段图省事可以先用 NoMachine，双机阶段需要"一屏同时看两架飞机+GCS自己
跑脚本"时再上跨机DDS。

### 1.8 关键风险清单

- 雷达安装角：已拍板水平安装，需正式同步给结构设计做总装确认。
- RC failsafe：`no_RC`真机前必须改`false`，核对通道映射。
- 悬停油门/物理参数：不能照搬仿真iris数值，必须真机重新标定。
- `px4_param_relax`放宽的失控保护参数：真机上过松是安全隐患。
- Livox-SDK2版本必须与`livox_ros_driver2`配套（driver和SDK同期发布，只换
  一边会编译报错，已踩过坑）。
- DLIO仿真里长期未完全收敛，真机前必须单独验证（本次讨论时已确认收敛）。
- SDK Manager烧录host：官方只支持Ubuntu 20.04/22.04，本机24.04不在支持
  列表，需要额外准备。
- 跨机DDS组网：仿真期间从未验证过，双机阶段是全新风险点。

---

## 二、ROS2 / DDS 网络基础知识

### 2.1 DDS 发现机制

DDS 默认的发现机制本身是**自动**的：每个节点上线自己广播"我在这"，其余
节点自动听到，不需要手动一个个连接。之所以有时需要手动配置，根源是**很多
WiFi 路由器/AP 会屏蔽组播/广播包**（无线网络的常见限制，不是DDS本身要求
手动连接）。

三种发现方式的取舍：

| 方式 | 适用场景 | 特点 |
|---|---|---|
| 默认组播 | 同一网段、AP不屏蔽组播 | 全自动，无需配置 |
| 静态 Peer 列表 | 组播被屏蔽、节点数量少 | 每对节点手动写对方IP，双向配置 |
| Discovery Server | 组播被屏蔽、节点数量多（如GCS+2架飞机） | 所有节点只需指向一个中心地址，不需要两两配对，心智模型类似"接一个交换机" |

### 2.2 "一堆协议"的误解澄清

ROS2 在线上传输走的协议只有**一种**——RTPS（DDS的标准wire协议），不是
"一堆协议"。容易混淆的地方是 DDS 有好几家厂商实现（FastDDS、CycloneDDS、
RTI等），这些实现之间不完全互通，所以同一张网里所有节点要统一使用
同一个实现——这是一次性配置好就不用再操心的事，不是持续增加的复杂度。

### 2.3 这个项目为什么选 CycloneDDS

`PLANNER=ego_planner` 时切到 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，
原因是 ego-planner-swarm 官方文档记录"FastDDS会导致明显卡顿，原因未知，
建议换CycloneDDS"，项目实测复现过类似症状（mavros/imu话题只有12-20Hz），
换CycloneDDS后恢复正常。这个选择意味着 GCS 和所有飞机必须统一用
CycloneDDS，不能各用各的默认实现。

---

## 三、Docker / Linux 基础知识

### 3.1 镜像、容器、可写层三者关系

- **镜像（image）**：build产物，一层层只读文件系统叠加而成，静态、不可
  原地修改。
- **容器（container）**：镜像的运行实例，= 镜像的只读层 + 一层专属的
  可写层。
- **可写层**：容器运行时的所有改动（`docker exec`进去改文件、日志输出等）
  都落在这一层，容器删除后这一层随之消失，**不会**自动回写到镜像或宿主机
  （除非显式`docker cp`导出，或本来就是bind mount）。

### 3.2 COPY（build 时）vs volume（运行时）

| | 发生时机 | 效果 |
|---|---|---|
| `Dockerfile`里的`COPY` | `docker build`那一刻 | 把宿主机文件**复制**进镜像层，之后宿主机文件再变，镜像不会自动同步，必须重新build |
| `docker-compose.yml`里的`volumes:`（bind mount） | 容器启动时 | 让容器内某路径和宿主机某目录**实时联动**（同一份东西），不需要重新build |

这个项目当前只有 `./runtime_logs:/logs` 一处是bind mount（飞行日志/rosbag
记录用），其余所有代码/配置都是build时COPY进去的一次性快照。

### 3.3 `/opt` 与 `/home` 的 FHS 约定

Linux 文件系统标准（FHS）里，`/home/用户名` 是真人用户的个人文件；`/opt`
是"第三方/附加软件包"专用目录——容器里跑的是一整套软件栈，不是"某个用户
的个人文件"，放`/opt`是这类场景下的常规做法。

### 3.4 `docker exec` / `docker cp` 用法

```bash
docker exec -it <容器名> bash          # 交互式进入容器，开一个终端
docker exec <容器名> ros2 topic list   # 只跑一条命令，不进终端

docker cp 宿主机路径 <容器名>:容器内路径   # 宿主机 → 容器
docker cp <容器名>:容器内路径 宿主机路径   # 容器 → 宿主机
```
容器默认用 root 跑（Dockerfile 没设 `USER`），理论上想放哪个目录都行，
没有严格意义上"不能碰"的目录；但 `/proc`、`/sys` 是内核虚拟出来的、
塞文件没有意义，真机上直通进来的 `/dev/ttyXXX` 设备节点不要手滑删除/覆盖。
`docker cp` 拷进去的东西只存在于容器的可写层，容器一删就没了，不适合当作
长期部署代码的手段。

### 3.5 entrypoint 机制

`ENTRYPOINT ["/opt/entrypoint.sh"]` 只是"容器启动时默认执行这个"：
- **名字可以改**——把 `Dockerfile` 里对应的 `COPY` 目标名和 `ENTRYPOINT`
  路径同步改掉即可。
- **不是 Docker 强制要求**——可以完全不设，容器起来后手动敲命令。
- **可以覆盖成手动启动**：
  ```yaml
  entrypoint: ["/bin/bash"]
  tty: true
  stdin_open: true
  ```
  这样容器起来只开一个空 bash，方便调试某一步到底卡在哪。

### 3.6 容器重启策略

现状：`docker-compose.yml` 里三个 service 都是 `restart: "no"`——主机重启
后容器不会自动起来，需要手动 `docker compose up -d`。想要开机自动拉起，
把 `restart: "no"` 改成 `restart: unless-stopped`，配合 Docker 服务本身
开机自启（`sudo systemctl enable docker`，默认已开启）。

### 3.7 没有 Dockerfile，还能不能改镜像

- **镜像本身不能原地编辑**——每一层只读，没有"直接改镜像里某个文件"这种
  操作。
- **容器能改**——`docker exec`进去改文件，改动落在容器自己的可写层。
- **例外**：即使没有 Dockerfile，也能通过 `docker commit <容器名>
  new-image:tag` 把一个改过的容器状态固化成一份**新镜像**——只是这样
  丢失了"清楚记录改了什么"这个好处，纯粹是"容器现在长什么样就固化成
  什么样"，不推荐作为常规手段。

---

## 四、本项目代码组织规则

### 4.1 三处代码

| 目录 | 是什么 | 进不进 git | 改了之后 |
|---|---|---|---|
| `staging/` | 第三方原始代码（DLIO/mighty/ego-planner-swarm/ros2_px4_stack/livox驱动/nlink驱动/PX4），`fetch_sources.sh`按固定commit拉取 | 不进（`.gitignore`排除） | 不能直接改完就用，要走patch流程 |
| `vendor/` | 引入并直接托管进本仓库的第三方衍生代码（目前只有`px4ctrl_ros2`） | 进 | 直接改，跟改自己代码一样 |
| `src/` | 这个项目自己原创的胶水/桥接包（`gt_odom_bridge`/`ego_planner_bridge`/`px4ctrl_bridge`/`uwb_origin_bridge`/`uwb_sim`/`nlink_uwb_bridge`） | 进 | 直接改 |

`staging/` 之所以不直接进git、也不能直接改，是因为这几个第三方项目体量
较大（`mighty_ws_src` 968M、`PX4-Autopilot` 2.6G、`pointlio` 1.4G），
全部fork进这个项目自己的git历史会让仓库体积暴涨；`vendor/px4ctrl_ros2`
体量小得多，直接托管的代价可以接受。

### 4.2 patch 机制原理

一份 `.patch` 文件是"跟某个基准版本相比改了哪几行"的纯文本差异记录
（`git diff` 生成的 unified diff），本身不是完整代码。`Dockerfile.
flight-stack` build镜像时，先把 `staging/xxx` 原始代码复制进镜像，
再执行 `git apply patches/xxx.patch` 把差异叠上去——**镜像里最终代码 =
原始代码 + 所有patch叠加结果**。这么做而不是直接把改过的代码提交进这个
仓库，是为了：
1. 不用把体量巨大的第三方代码库塞进本项目的git历史。
2. 清楚记录"跟上游相比到底改了什么、为什么改"（patch文件名+详细注释）。

**重要结论**：build完之后，`staging/` 和镜像里的代码**不再完全一样**——
`COPY`那一刻两者相同，但patch只应用在镜像内部，从未回写到`staging/`，
`staging/` 永远保持"未打任何补丁的原始状态"，这是刻意设计，不是疏漏。

### 4.3 改 `staging/` 代码的标准流程

```bash
# 1. 直接在staging/里改（真实git checkout，pin在某个commit）
vim staging/dlio_ws_src/direct_lidar_inertial_odometry/src/xxx.cpp

# 2. 生成patch
cd staging/dlio_ws_src/direct_lidar_inertial_odometry
git diff > ../../../patches/dlio_my_fix.patch

# 3. 撤销staging/里的改动（保持"干净"，否则Dockerfile的git apply会冲突）
git checkout -- src/xxx.cpp

# 4. 把这个patch加进 Dockerfile.flight-stack 对应的 COPY+git apply 那两行

# 5. 重新build，改动才真正生效
docker compose build flight-stack-nx01
```
项目历史上更常见的做法是在 `/tmp` 另外clone一份临时副本改，而不是直接动
`staging/`——效果一样，更不容易忘记第3步的撤销。

---

## 五、代码搬运与体积管理

### 5.1 实测体积分布

```
docker_sim 整个目录：      82G
├── runtime_logs/：        73G  ← 真正的大头，仿真飞行录的rosbag/日志，
│                                  跟代码/真机部署无关，不需要传给任何地方
├── staging/：             9.3G
│   ├── gazebo_models_external:  3.9G  ← 真机不需要
│   ├── PX4-Autopilot:           2.6G  ← 真机不需要（真实飞控走独立烧录的
│   │                                     NuttX固件，不是这份POSIX SITL构建）
│   ├── PX4-SITL_gazebo-classic: 312M  ← 真机不需要
│   └── 真机需要的部分（dlio/mighty/ros2_px4_stack/ego-planner/livox/
│       nlink/pointlio）：约2.6G
└── git仓库实际tracked的内容（patches/src/vendor/config/docker/README等）：
                            仅 1.2M
```

### 5.2 git push 到 GitHub 的可行性

`.gitignore` 已经排除了 `staging/` 和 `runtime_logs/`，真正会被
`git push` 上去的内容只有 **1.2MB**，随便传，80多G里几乎全是"本来就不该
进版本控制的东西"，不是代码本身很大。

### 5.3 rsync 原理与命令详解

**原理**：
1. 先比较源和目标两边文件列表（文件名/大小/修改时间），没变的文件跳过。
2. 内容有变化的文件，用"滚动校验和"算法切成小块比较，**只传变了的块**，
   不是整份重传。
3. 目标端没有的文件正常整份传。
4. 默认不删除目标端多出来的文件（除非加`--delete`）。

**需要网络吗**：跨机场景（本机→Jetson）需要，走局域网+SSH传输通道，
**不需要连外网/GitHub**——这也是它比`git clone`更适合Jetson现场没有外网
access场景的原因。

**完整命令**：
```bash
rsync -avz --progress \
  --exclude 'runtime_logs' \
  --exclude 'staging/gazebo_models_external' \
  --exclude 'staging/PX4-Autopilot' \
  --exclude 'staging/PX4-SITL_gazebo-classic' \
  --exclude '.git' \
  /home/robots/ai_uav/docker_sim/ \
  jetson用户名@192.168.x.x:~/docker_sim/
```
`-a`归档模式（保留权限/符号链接/时间戳，递归）；`-v`显示传了哪些文件；
`-z`传输时压缩；`--progress`进度条；`--exclude`排除目录。冒号前面
`jetson用户名@192.168.x.x` 就是目标机器，也可以先在 `~/.ssh/config`
配一个别名（如`nx01`）代替敲IP。

第一次同步是全量传输（约2.6G+几MB，不是82G），确实需要传完整数据，这不是
rsync变慢了，是第一次没有旧版本可以比对差异；**第二次以后**改一行代码
再同步，只传变化的部分，几秒钟搞定。如果第一次这2.6G传得异常慢，大概率
是WiFi链路带宽/信号问题，可以用`iperf3`量一下实际网络吞吐量确认；现场
网络实在差，第一次的大头数据也可以用U盘拷过去，以后再用rsync做增量同步。

---

## 六、调试模式方案对比

日常改代码要么在`src/`/`vendor/`直接改（无额外步骤），要么在`staging/`
按第四章流程走patch——后者对"频繁试错调试"来说比较繁琐，讨论了三种
简化方案。

### 6.1 方案A：按 workspace volume 挂载 + 手动补 patch

把某个模块的 `staging/xxx` 挂载进容器对应路径，**但不能直接挂原始
`staging/`**（那是未打补丁的版本，会让已经修好的bug复现）。正确做法是
先复制一份出来、手动按Dockerfile里的清单把现有patch都apply上去、
`git init`打一个基准，再拿这份"打好补丁的工作副本"去挂载。调试完
`git diff`直接得到新patch，追加进Dockerfile的apply清单末尾。

缺点：需要手动维护"这个模块现在生效的patch都有哪些、顺序是什么"，容易漏。

### 6.2 方案B：从已 build 镜像整体提取 `/opt`

```bash
docker create --name tmp_extract flight-stack:latest
docker cp tmp_extract:/opt ./opt_extracted
docker rm tmp_extract
cd opt_extracted && git init && git add -A && git commit -m "baseline"
```
用一个 `docker-compose.debug.yml` override 文件把整个 `/opt` 挂载成
这份提取出来的目录，不动 `Dockerfile.flight-stack` 本体。

优点：不用手动猜/补patch，导出的就是"已经打好所有patch、还编译好了"的
真实最终状态，比方案A更不容易出错。

**关键限制**：这份提取出来的 `install/`/`build/` 是**x86_64编译产物**，
架构跟Jetson（arm64）不兼容，**不能**直接搬给Jetson用——这个方法只适合
在这台x86开发机上调试仿真，跟真机部署是两条不相干的路，调试完仍需要
把改动落回一份正式的patch文件才能对Jetson生效。

### 6.3 "从一开始就 volume"的架构级权衡

如果项目从设计之初就用volume挂载源码（不走COPY+patch+build），日常
编辑确实会顺畅很多，但会在这个项目真正需要的能力上打折扣：

- **镜像不再自包含**——`/opt`内容永远依赖宿主机挂载源，镜像变成空壳子。
  "把源码搬到新机器、在目标架构上重新编译一次"这个动作躲不掉，volume-
  first只是把它从"build镜像时做一次、存进镜像层复用"变成"每台新机器
  现场手动做一次"，工作量没有消失。
- **改动记录**要么继续靠某种机制维护（比如把`staging/`几个大项目也全部
  fork进自己git仓库，像`vendor/`现在做的那样），要么就没有任何记录——
  前者会让"git仓库只有1.2MB"这个优点消失。

**结论**：如果项目只打算永远在一台机器上跑，volume-first会更好；但因为
真实需求包含"移植到Jetson（不同架构）"，当前"COPY+patch+build"的设计
是刻意的权衡，不是疏漏。

### 6.4 推荐折中方案

`Dockerfile.flight-stack` 保持不变，继续作为"能在任何机器上重新生成一套
可用系统"的权威定义；日常调试用 `docker-compose.override.yml`（Compose
会自动加载，不需要`-f`参数）常驻挂载正在改的一两个模块的"打好补丁的
工作副本"——体验上接近"从一开始就是volume形式"，但没有牺牲"给Jetson
重新生成可复现镜像"的能力。（本文档撰写时这个override文件还未创建，
待后续按需实施。）

---

## 七、本次实际代码改动

### 7.1 第一轮：source_all.sh 整理

把 `flight-stack-entrypoint.sh` 里10行分散的 `source /opt/xxx_ws/install/
setup.bash` 合并成一个可复用文件。

- **新建** `docker/entrypoints/source_all.sh`：统一收纳所有workspace的
  `source`语句，保留原有的顺序依赖说明（ROS2底座必须最先；`livox_ws`
  必须排在`point_lio_ws`之前，否则`point_lio`运行时找不到CustomMsg类型
  符号）。必须用 `source /opt/source_all.sh` 调用，不能直接执行——
  `source`才会在当前shell生效，直接执行只在子shell里生效，退出后环境
  变量全部消失。
- **修改** `docker/entrypoints/flight-stack-entrypoint.sh`：原来的10行
  source+说明注释，替换成一行 `source /opt/source_all.sh` + `set -u`。
- **修改** `docker/Dockerfile.flight-stack`：新增
  `COPY docker/entrypoints/source_all.sh /opt/source_all.sh`。

### 7.2 第二轮：真机部署骨架代码第一批（2026-08-14）

把之前"真机侧还缺哪些文件"清单里不依赖真实硬件信息的部分（A类）全部
落地：

| 文件 | 说明 |
|---|---|
| `docker-compose.hw.yml`（新建） | 真机单机部署compose，单容器+`DEPLOY_TARGET=hw`+2026-08-14确认组合默认值 |
| `docker/entrypoints/hw_launch/mid360_real.launch.py`（新建） | 真实Mid-360驱动launch文件，`xfer_format=0`对齐DLIO，话题名假设未经验证 |
| `src/px4ctrl_bridge/launch/px4ctrl_hw.launch.py`（新建） | 基于sim版改`no_RC=False` |
| `docker-compose.override.yml.example`（新建） | volume调试模式模板 |
| `docker/systemd/flight-stack-hw.service`（新建） | 开机自启模板 |
| `gcs/Dockerfile`+`gcs/docker-compose.yml`+`gcs/cyclonedds_gcs.xml.example`（新建） | GCS轻量容器，只装ROS2底座+`quadrotor_msgs`/`traj_utils`两个消息包 |
| `docker/entrypoints/flight-stack-entrypoint.sh`（修改） | 新增`DEPLOY_TARGET`环境变量串联真机分支+3处fail-fast安全拦截 |
| `docker/entrypoints/source_all.sh`（修改） | 补`source /opt/nlink_ws/install/setup.bash`（之前的遗漏） |
| `src/px4ctrl_bridge/px4ctrl_bridge/px4_param_relax.py`（修改） | 按`DEPLOY_TARGET`区分失控保护参数取值 |

**自查发现并修复的3处真实问题**（按完善性/缺失/错误/效率/可操作性五个
维度自查，方法同第6轮操作清单review）：
1. 真实Mid-360驱动第一版只在`uwb_slam`分支启动，漏了`mighty`规划器
   （`global_mapper_ros`无条件订阅原始点云）和`ego_planner`+`uwb_imu`
   （`gt_cloud_bridge_node`需要原始点云）两条下游路径，还会导致重复
   启动同名节点的冲突——改成在`LOCALIZATION_SOURCE`分发之前统一启动
   一次。
2. `docker-compose.hw.yml`镜像tag跟仿真版`flight-stack:latest`撞名，
   改成独立的`flight-stack-hw:latest`。
3. `gcs/Dockerfile`漏了这台开发机build镜像必需的代理支持（`ARG
   HTTP_PROXY`等），会导致在这台机器上直接build失败。

以上全部只做了语法/静态检查（`bash -n`/`py_compile`/`docker compose
config -q`/`docker build --check`/`systemd-analyze verify`），**没有跑
`docker build`真正编译、没有连过任何真实硬件**，遵循本项目"手动build"的
既有约定。详细清单+自查过程见`DEBUG_JOURNAL.md`同日期记录。

---

## 附：延伸阅读索引

原始逐字问答记录见 `DEBUG_JOURNAL.md`，2026-08-14 当天以下几条：
- "用户要求给出...真机机载平台...具体方案和操作步骤"（总体架构+路线图）
- "用户确认真机首飞组合...Docker+NVIDIA Container Toolkit安装方法+真机侧
  改造方案+本机侧改造方案"（文件级改造清单）
- 后续几轮关于 ROS2网络/Docker镜像容器机制/patch工作流/调试模式对比的
  纯讨论内容，本文档已按主题重新整理，不再单独列出每一条对应位置。
