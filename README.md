# 双机仿真系统：DLIO + mighty + ros2_px4_stack + PX4 v1.16 + 经典Gazebo + UWB真值

Ubuntu22.04 + ROS2 Humble + PX4 v1.16 + 经典Gazebo(不是Harmonic)，宿主机Ubuntu24.04。

## 目录结构

```
docker_sim/
├── fetch_sources.sh              # 【你手动跑】把所有源码clone到staging/
├── staging/                      # fetch_sources.sh 的产物，.gitignore掉
├── docker/
│   ├── Dockerfile.base           # 公共基础镜像
│   ├── Dockerfile.sim-world      # Gazebo + 双机PX4 SITL + UWB真值节点，跑1份
│   ├── Dockerfile.flight-stack   # DLIO+mighty+ros2_px4_stack，跑2份(NX01/NX02)
│   └── entrypoints/
├── patches/                      # 针对已知问题的补丁，构建时自动尝试应用
├── config/vehicle_profile.yaml   # 唯一的飞机物理参数来源(mass/推力/包围盒等)
├── src/uwb_sim/                  # UWB真值模拟节点(自己写的ROS2包)
└── docker-compose.yml
```

## 使用步骤

```bash
# 1. 宿主机侧手动预取全部源码（用你自己的代理/网络，别指望docker build里下）
export http_proxy=http://127.0.0.1:7897/; export https_proxy=$http_proxy
git config --global http.proxy $http_proxy
cd docker_sim
./fetch_sources.sh                       # 全量；或 ./fetch_sources.sh --only px4,dlio,mighty,ros2px4 先拉核心链路

# 2. 建公共基础镜像
# --network=host：代理是 http://127.0.0.1:7897/，这个地址在容器默认网络里指向
# 容器自己而不是宿主机，会连不上代理；加 --network=host 让构建过程直接共享
# 宿主机网络栈，127.0.0.1 才能正确解析到宿主机上的代理。
docker build --network=host \
             --build-arg HTTP_PROXY=$http_proxy --build-arg HTTPS_PROXY=$https_proxy \
             -f docker/Dockerfile.base -t mighty-base:humble .

# 3. 建另外两个镜像 + 起容器
# docker-compose.yml 里每个service的build段已经加了 network: host，同样是为了
# 让 127.0.0.1 代理在构建阶段能被正确访问到。
#
# 注意：不要裸跑 `docker compose build`（不带 service 名）——sim-world 和
# flight-stack-nx01 会被并发构建，叠加各自内部 colcon/make 的并行编译，在核数远超
# 内存的机器上（比如32核/31G）很容易把内存吃穿、系统死机/卡死。下面分两条命令
# 按顺序build，保证同一时间只有一个镜像在编译：
docker compose build sim-world
docker compose build flight-stack-nx01

# 4. 起容器前先给容器授权访问宿主机X server（Gazebo/RViz2的GUI窗口要用）
# xhost授权是宿主机X server自己的运行时ACL，只在当前图形会话存活期间有效，
# 重启宿主机/注销重登都会失效，需要重新执行；这一步没法写进docker-compose.yml
# 里（compose规范没有能在`up`之前自动跑宿主机命令的钩子），只能手动跑或者
# 用下面的start.sh包装脚本：
xhost +local:docker
docker compose up
# 或者：./start.sh   （等价于上面两条，帮你自动做xhost授权再up）
```

### 编译内存不够 / 构建时系统卡死

单个镜像内部的编译并发度由 Dockerfile 里的 `BUILD_JOBS`（默认4）控制，colcon/PX4的make/Livox的make
都显式吃这个值，不再各自默认拉满nproc。默认值是按"内存紧张"估的保守值，内存宽裕可以调大：

```bash
docker compose build --build-arg BUILD_JOBS=8 sim-world
docker compose build --build-arg BUILD_JOBS=8 flight-stack-nx01
```

粗略换算：每个并发编译任务峰值按1.5~3GB算，`BUILD_JOBS=4`约占6~12GB。只要还是分开两条命令
顺序build（同一时间只有一个镜像在编译），`BUILD_JOBS`可以按"可用内存(GB)/2"来估。

## 容器操作

三个容器：`sim-world`（Gazebo+双机PX4 SITL+UWB真值，1份）、`flight-stack-nx01`/
`flight-stack-nx02`（DLIO+规划器+板外控制器，各1份）。compose服务名是
`sim-world`/`flight-stack-nx01`/`flight-stack-nx02`，容器名是
`docker_sim-sim-world-1`/`docker_sim-flight-stack-nx01-1`/
`docker_sim-flight-stack-nx02-1`（`docker ps`能看到）。

### 启动 / 停止 / 重启

```bash
./start.sh                    # 【推荐】xhost授权+docker compose up -d+自动拉起tmux监控面板
                               # 如果检测到sim-world已经在跑，会连带flight-stack两个一起
                               # 重启（三个容器运行时状态强耦合，见start.sh里的注释）
./scripts/up_and_watch.sh     # 等价于start.sh去掉xhost那一步，没有GUI/已经手动xhost过时用

docker compose down           # 停止并删除三个容器（不删镜像/不删runtime_logs）
docker compose stop           # 只停止，不删除容器（保留容器内部临时状态，一般用不上）
docker compose restart flight-stack-nx02   # 只重启某一个服务
```

⚠️ **改了`docker-compose.yml`里的环境变量之后，必须`docker compose up -d`让容器"重建"，
`docker compose restart`不会应用新的环境变量**（restart只是重启现有容器进程，不会重新
读取compose文件的environment段；up -d会先diff配置，发现变了才重建容器）。

### 常用启动组合（带参数一键命令）

`LOCALIZATION_SOURCE`（`gt`/`uwb_slam`/`uwb_imu`，另有三个单机专用值
`single_slam_only`/`single_uwb_imu`/`single_uwb_slam`，见下面"单机仿真
彩排"一节，不计入这里的组合数）、`PLANNER`（`mighty`/
`ego_planner`）、`CONTROLLER`（`ros2_px4_stack`/`px4ctrl`/`so3ctrl`，另有
2026-08-13新增的`pt4ctrl`——**2026-09-13更新：`uwb_imu`定位源下已确认
真机+仿真均验证通过，不再是"完全未实测"，见下表新增的两行**；`gt`/
`uwb_slam`定位源下仍然没有专门验证记录，不计入下面18种组合的主表，单独
列在表后）三个开关一共3×2×3=18种组合。**下面所有命令统一按
`LOCALIZATION_SOURCE PLANNER CONTROLLER`这个变量顺序书写**（之前几个例子
顺序不统一，这次改成固定顺序，方便照抄时直接改中间那截）。**推荐用
`VAR=value ./start.sh`这个形式**而不是裸`docker compose up -d`——`start.sh`
不会清空/隔离环境，命令行前面写的变量会原样传给它内部调的
`docker compose up -d`，效果完全一样，但多了xhost自动授权+tmux监控面板
自动拉起（见上面"启动 / 停止 / 重启"一节）；只有在明确不需要这两个附加
动作时（比如已经手动xhost过、不想要tmux面板）才用裸`docker compose up -d`。

规划器发的指令都有对应桥接节点接住，各定位源下障碍物点云都有对应节点补齐，
不存在"某个组合下规划器发的话题没人订阅/收不到点云"这种静默失效的情况
（具体桥接实现见`docker/entrypoints/flight-stack-entrypoint.sh`头部注释）——
**但这不等于18种组合都实测飞过**，下表"状态"列区分"实测验证过"/"实测确认
不能用（会炸机）"/"没有专门的完整流程验证记录"三种，后一种不代表有问题，
只是还没有人专门测过那个具体组合，跟"接口对不对"是两回事：

| LOCALIZATION_SOURCE | PLANNER | CONTROLLER | 命令 | 状态 |
|---|---|---|---|---|
| `gt` | `mighty` | `ros2_px4_stack` | `LOCALIZATION_SOURCE=gt PLANNER=mighty CONTROLLER=ros2_px4_stack ./start.sh` | 接口跟下面已验证的`uwb_slam`同组合完全一致，没有见到专门的完整流程验证记录 |
| `gt` | `mighty` | `px4ctrl` | `LOCALIZATION_SOURCE=gt PLANNER=mighty CONTROLLER=px4ctrl ./start.sh` | `px4ctrl`实验性（仅编译+启动冒烟测试），没有专门验证记录 |
| `gt` | `mighty` | `so3ctrl` | `LOCALIZATION_SOURCE=gt PLANNER=mighty CONTROLLER=so3ctrl ./start.sh` | `so3ctrl`实验性（仅代码交叉核实），没有专门验证记录 |
| `gt` | `ego_planner` | `ros2_px4_stack` | `LOCALIZATION_SOURCE=gt PLANNER=ego_planner CONTROLLER=ros2_px4_stack ./start.sh` | 没有专门的完整流程验证记录 |
| `gt` | `ego_planner` | `px4ctrl` | `LOCALIZATION_SOURCE=gt PLANNER=ego_planner CONTROLLER=px4ctrl ./start.sh` | 当前默认组合；`px4ctrl`实验性，没有专门验证记录 |
| `gt` | `ego_planner` | `so3ctrl` | `LOCALIZATION_SOURCE=gt PLANNER=ego_planner CONTROLLER=so3ctrl ./start.sh` | `so3ctrl`实验性，没有专门验证记录 |
| `uwb_slam` | `mighty` | `ros2_px4_stack` | `LOCALIZATION_SOURCE=uwb_slam PLANNER=mighty CONTROLLER=ros2_px4_stack ./start.sh` | ✅ 实测验证过，没有问题（最早跑通完整起飞-跟踪-降落流程的组合） |
| `uwb_slam` | `mighty` | `px4ctrl` | `LOCALIZATION_SOURCE=uwb_slam PLANNER=mighty CONTROLLER=px4ctrl ./start.sh` | ✅ 实测验证过，没有问题（2026-08-13确认） |
| `uwb_slam` | `mighty` | `so3ctrl` | `LOCALIZATION_SOURCE=uwb_slam PLANNER=mighty CONTROLLER=so3ctrl ./start.sh` | ✅ 实测验证过，没有问题（2026-08-13确认） |
| `uwb_slam` | `ego_planner` | `ros2_px4_stack` | `LOCALIZATION_SOURCE=uwb_slam PLANNER=ego_planner CONTROLLER=ros2_px4_stack ./start.sh` | 没有专门的完整流程验证记录 |
| `uwb_slam` | `ego_planner` | `px4ctrl` | `LOCALIZATION_SOURCE=uwb_slam PLANNER=ego_planner CONTROLLER=px4ctrl ./start.sh` | ✅ 实测验证过，没有问题（2026-08-13确认） |
| `uwb_slam` | `ego_planner` | `so3ctrl` | `LOCALIZATION_SOURCE=uwb_slam PLANNER=ego_planner CONTROLLER=so3ctrl ./start.sh` | ✅ 实测验证过，没有问题（2026-08-13确认） |
| `uwb_imu` | `mighty` | `ros2_px4_stack` | `LOCALIZATION_SOURCE=uwb_imu PLANNER=mighty CONTROLLER=ros2_px4_stack ./start.sh` | ✅ 实测验证过，没有问题 |
| `uwb_imu` | `mighty` | `px4ctrl` | — | ⛔ **禁止**，2026-08-12实测炸机（反馈频率不够，见下面参数表） |
| `uwb_imu` | `mighty` | `so3ctrl` | — | ⛔ **禁止**，跟`px4ctrl`同一根因（共用`PX4CtrlFSM`），同样炸机 |
| `uwb_imu` | `ego_planner` | `ros2_px4_stack` | `LOCALIZATION_SOURCE=uwb_imu PLANNER=ego_planner CONTROLLER=ros2_px4_stack ./start.sh` | ✅ 实测验证过，没有问题 |
| `uwb_imu` | `ego_planner` | `px4ctrl` | — | ⛔ **禁止**，2026-08-12实测炸机 |
| `uwb_imu` | `ego_planner` | `so3ctrl` | — | ⛔ **禁止**，跟`px4ctrl`同一根因，同样炸机 |
| `uwb_imu` | `mighty` | `pt4ctrl` | `LOCALIZATION_SOURCE=uwb_imu PLANNER=mighty CONTROLLER=pt4ctrl ./start.sh` | ✅ 2026-09-13确认：真机+仿真均已验证，没有问题（`pt4ctrl`不计入上面3×2×3=18组合的主表，单独加在这里） |
| `uwb_imu` | `ego_planner` | `pt4ctrl` | `LOCALIZATION_SOURCE=uwb_imu PLANNER=ego_planner CONTROLLER=pt4ctrl ./start.sh` | ✅ 2026-09-13确认：真机+仿真均已验证，没有问题——2026大赛contest task（`docker_sim/2026大赛任务系统全流程任务仿真实现方案.md`）实际选用的组合 |

⚠️ 2026-08-13更新：18种组合里已实测确认的扩大到7种——`uwb_slam`这一组
除了`ego_planner`+`ros2_px4_stack`之外的其余5种（`mighty`/`ego_planner`
×`ros2_px4_stack`/`px4ctrl`/`so3ctrl`里`uwb_slam`+`ego_planner`+
`ros2_px4_stack`还没测过，其余5种都测过），加上`uwb_imu`+`ros2_px4_stack`
（`mighty`/`ego_planner`各一种），全部✅没有问题；`uwb_imu`+`px4ctrl`/
`so3ctrl`四种组合⛔确认炸机、禁止使用；`gt`这一组6种、以及`uwb_slam`+
`ego_planner`+`ros2_px4_stack`这一种，仍然是"接口设计上应该没问题"的
推断，不是实测结论，跟"接口对不对"是两回事。

⚠️ 2026-09-13更新：`pt4ctrl`（2026-08-13新增）不再是"全新代码、完全
未实测"——`uwb_imu`定位源下`pt4ctrl`真机+仿真均已验证通过，没有问题
（上表新增两行），是2026大赛contest task实际选用的`CONTROLLER`。`gt`/
`uwb_slam`定位源下`pt4ctrl`仍然没有专门的验证记录，不代表有问题，只是
还没有人专门测过那个组合。README里另一处说
"ego_planner还没实测飞过"这句话已确认过时——`ego_planner`+`px4ctrl`/
`so3ctrl`（`uwb_slam`定位源下）都已实测确认没问题，这句话需要你自己找
到具体位置改掉（这次没有全文排查有没有其它地方还留着同样过时的表述）。

其它常用变量组合（跟上面18种正交，不是`LOCALIZATION_SOURCE`/`PLANNER`/
`CONTROLLER`这三个维度的一部分）：

```bash
# 没有UWB真值来源时（跟LOCALIZATION_SOURCE无关，这套仿真本身UWB真值节点
# 一直在跑；这个开关是留给真的没有UWB硬件/仿真源的场景），ego_planner机间
# 轨迹坐标变换会一直等不到、退化成完全不避让，需要额外关掉frame_align
LOCALIZATION_SOURCE=gt PLANNER=ego_planner CONTROLLER=px4ctrl EGO_USE_FRAME_ALIGNMENT=false ./start.sh

# 只验证"不靠仿真真值、光靠origin_setter_node的真实标定也能对齐"（详见
# frame_align_bridge_node一节），跟LOCALIZATION_SOURCE正交，18种组合上都能加
PUBLISH_FRAME_ALIGN=false LOCALIZATION_SOURCE=uwb_slam PLANNER=mighty CONTROLLER=ros2_px4_stack ./start.sh

# 切换sim-world场景（森林/建筑等），仅sim-world服务读这个变量
WORLD_ENV=hard_forest ./start.sh
```

### 单机仿真彩排（Jetson Orin NX单机真机移植前的验证用，2026-08-13新增）

真机首台部署目标是单个真飞机（Jetson Orin NX Super 16G），docker_sim默认
是双机（sim-world + flight-stack-nx01 + flight-stack-nx02）。为了在真机
之前用仿真提前把单机场景走一遍，`LOCALIZATION_SOURCE`新增三个单机专用值，
对应真机部署方案里讨论过的三种真机场景（详见`DEBUG_JOURNAL.md`
2026-08-13"用户要求...给出具体文件级实现步骤"和"新增三个单机
LOCALIZATION_SOURCE"两条记录）：

| 值 | 对应的真机场景 | 实际行为 |
|---|---|---|
| `single_slam_only` | 完全没有UWB硬件，纯DLIO SLAM | 跟`uwb_slam`一样起DLIO，但**不启动`uwb_origin_bridge`**（没有UWB就不该起依赖UWB的包） |
| `single_uwb_imu` | UWB+IMU替代SLAM（SLAM完全失效场景） | 跟`uwb_imu`逐字相同（该模式下的节点本来就只处理自己namespace的数据，不关心是不是双机） |
| `single_uwb_slam` | DLIO SLAM + UWB一键定原点 | 跟`uwb_slam`逐字相同 |

**启动方式**：只起`sim-world`+`flight-stack-nx01`两个服务（不起
`flight-stack-nx02`），并且`NUM_AGENTS`必须传1——三个`single_*`值在
entrypoint里会强制校验`NUM_AGENTS==1`，不等于1直接报错退出，防止手滑在
双机compose默认配置下误用：

```bash
NUM_AGENTS=1 LOCALIZATION_SOURCE=single_uwb_slam PLANNER=mighty CONTROLLER=ros2_px4_stack \
    ./start.sh sim-world flight-stack-nx01
```

⚠️ **已知限制，本次没有一并改**：`scripts/watch_sim.sh`（tmux监控面板）、
`scripts/status_monitor.py`、`scripts/launch_control.py`、
`scripts/dual_goal_input.py`这几个运维脚本目前都硬编码假设NX01/NX02两架
飞机同时存在（比如`status_monitor.py NX01 NX02`、tmux固定开NX02日志窗口）
——单机彩排时`flight-stack-nx02`容器根本没起来，这些脚本会卡在等NX02数据
或者报错。单机验证阶段建议不用`./start.sh`自动拉起的tmux面板，改用
`docker compose logs -f flight-stack-nx01`看日志、`docker exec`手动调用
`ros2 service call .../launch_control`等效命令；这批脚本改造成能自适应
单/双机是后续单独的工作量，不在这次改动范围内。

⚠️ 只做了entrypoint的分支逻辑+`NUM_AGENTS`可覆盖化，**没有实际
`docker compose build`/`docker compose up`跑过这三个新值**（遵循手动build
约定），需要你自己构建验证。

⚠️ 如果`sim-world`容器已经在跑，`start.sh`内部会先做一次`docker compose restart`
（补xhost授权用），`restart`本身不应用新环境变量，但紧接着脚本自己会再跑一次
`docker compose up -d`，这一步会重新diff配置、检测到环境变量变了就重建容器——
最终环境变量还是会正确生效，只是已经在跑的情况下会多一轮"重启又重建"，不是bug。

### 进入容器手动调试

```bash
docker exec -it docker_sim-flight-stack-nx01-1 bash
```

进容器之后手动跑`ros2`命令（`ros2 topic list`/`ros2 param get`等）之前，必须先把这几个
workspace的`setup.bash`都source一遍（缺一个可能话题类型认不出来、缺自定义消息包）：

```bash
source /opt/ros/humble/setup.bash
source /opt/decomp_ws/install/setup.bash 2>/dev/null || true
source /opt/mighty_ws/install/setup.bash
source /opt/dlio_ws/install/setup.bash
source /opt/ros2_px4_stack_ws/install/setup.bash
source /opt/ego_planner_ws/install/setup.bash
source /opt/px4ctrl_ws/install/setup.bash
```

**`PLANNER=ego_planner`时额外要做一步**：`docker exec`开的是全新shell进程，跟
entrypoint.sh自己那个shell完全独立，`ego_planner`模式下的节点实际用CycloneDDS
（跟FastDDS默认值不同），不额外切RMW的话`ros2 topic list`/`ros2 node list`这些
命令会看不到任何真实节点。直接`source scripts/ros2_env_setup.sh`（这个文件已经
处理好了"按PLANNER环境变量判断切不切、容器刚重启配置文件还没写出来"这些细节，
不用自己重新判断）：

```bash
docker cp scripts/ros2_env_setup.sh docker_sim-flight-stack-nx01-1:/tmp/ros2_env_setup.sh
docker exec -it docker_sim-flight-stack-nx01-1 bash -c \
  'source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && bash'
```

如果`ros2 node list`/`ros2 topic echo`看起来"能连上但看不到任何东西"，大概率是
`ros2 daemon`用旧的RMW配置缓存住了，`ros2 daemon stop`强制它用当前环境重新起一次
（连着的daemon不会自动感知环境变量变化）。

### ROS2常用命令速查

以下命令都要在上面"进入容器手动调试"那一步source好环境之后再跑。示例统一用NX01/
`PLANNER=ego_planner`举例，`mighty`模式下把话题/节点名换成对应的（比如没有
`ego_planner_node`、`/broadcast_bspline`换成`/trajs`）。

**话题（topic）**：

```bash
ros2 topic list                                    # 列出当前能看到的全部话题
ros2 topic echo /NX01/mavros/state                 # 打印话题内容（Ctrl-C退出）
ros2 topic echo /broadcast_bspline --once           # 只打印一条就退出
ros2 topic hz /NX01/mavros/local_position/pose      # 实测发布频率
ros2 topic bw /NX01/grid_map/occupancy_inflate      # 实测带宽（找占带宽大户时用，
                                                     # 比如record_rosbag.sh文件头注释
                                                     # 提过unknown_grid比occupancy_grid
                                                     # 带宽高一个数量级）
ros2 topic info /broadcast_bspline --verbose        # 看有哪些发布者/订阅者、QoS是否匹配
                                                     # ——QoS不匹配是"两边都显示存在、
                                                     # 但收不到数据"的常见根因之一
```

⚠️ **`ros2 topic pub`发一次性消息时慎用`--once`**：`--once`在DDS discovery真正
完成之前就可能已经退出，消息实际没发出去，这个项目已经踩过好几次（`dual_goal_input.py`
文件头注释里有完整说明）。需要手动发目标点用goal窗口的`dual_goal_input.py`，不要
现拼`ros2 topic pub --once`；真要一次性发别的消息，要么去掉`--once`让它常驻发几秒
再Ctrl-C，要么确认收发双方都已经在`ros2 topic list`里互相看得到再发。

**节点（node）**：

```bash
ros2 node list                                      # 列出当前能看到的全部节点
ros2 node info /NX01/ego_planner_node                # 看某个节点的全部订阅/发布/服务
```

**参数（parameter）**：

```bash
ros2 param list /NX01/ego_planner_node                          # 列出某节点的全部参数
ros2 param get /NX01/ego_planner_node swarm/use_frame_alignment  # 读一个参数当前值
ros2 param set /NX01/ego_planner_node optimization/dist0 1.5     # 运行时临时改一个参数
```

⚠️ `ros2 param set`**只是运行时临时改**，改的是这一次运行中的内存值，不会写回
`docker-compose.yml`/launch文件，容器重启或`docker compose up -d`之后就会恢复成
配置里写的默认值——想要"改完还能一直生效"，得改上面"参数与环境变量参考"一节里对应的
环境变量再重新`up -d`。`ros2 param set`适合"现在就想看看调这个参数有没有效果、
还不确定要不要固定下来"这种临时试验场景。

**服务（service）**——mavros自带的解锁/切模式服务，紧急情况下手动介入用：

```bash
ros2 service list | grep mavros
ros2 service call /NX01/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"  # 强制上锁
ros2 service call /NX01/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'AUTO.LAND'}"  # 强制切降落模式
```

**rosbag**（项目自动录制见下面"数据记录"一节，这里是手动临时操作）：

```bash
ros2 bag info runtime_logs/rosbag/bag_20260809_022312     # 看某个bag块里有哪些话题/时间范围
ros2 bag play runtime_logs/rosbag/bag_20260809_022312     # 回放（会往真实话题名发布，
                                                            # 容器还活着的时候回放会跟
                                                            # 实时数据混在一起，一般只在
                                                            # 容器停掉之后单独回放分析用）
ros2 bag record -o /tmp/my_manual_bag /NX01/mavros/state /NX02/mavros/state  # 手动指定
                                                            # 话题临时录一段，不受
                                                            # record_rosbag.sh滚动清理管
```

**TF树**：

```bash
ros2 run tf2_ros tf2_echo NX01/odom NX01/base_link          # 查两个frame之间的实时变换
ros2 topic echo /tf_static --once                           # 静态TF一次性打印全量
```

### tmux监控面板（`start.sh`/`up_and_watch.sh`自动拉起，8个窗口）

session名`mighty_sim`。切窗口：`Shift+左右方向键`（不用按前缀）、鼠标点状态栏窗口名、
或`Ctrl-a`+数字键(0~7，tmux默认从0开始编号)；detach（不关闭）：`Ctrl-a d`；彻底关闭：
`Ctrl-a k`（有二次确认）；鼠标滚轮翻看历史输出（scrollback 5万行）；重新`attach`回
已存在的session直接再跑一次`./scripts/watch_sim.sh`。

| # | 窗口(颜色) | 内容 | 说明 |
|---|---|---|---|
| 0 | sim-world(青) | `docker compose logs -f sim-world` | Gazebo/PX4 SITL/UWB真值节点日志 |
| 1 / 2 | NX01(绿) / NX02(橙) | 对应flight-stack容器日志 | 过滤掉了DLIO自己刷屏的多行状态面板（`[dlio_odom_node`前缀），DLIO本身照常跑、只是不打印到这里 |
| 3 | launch(红) | 起飞/降落/锁定原点/清空记录控制台 | attach进来默认停在这里（从左往右排第4个，但attach后会自动切过去，不用手动找）；容器起来后飞机只会解锁/进OFFBOARD/原地悬停，curses实现的七个按钮`[1]NX01起飞 [2]NX01降落 [3]NX02起飞 [4]NX02降落 [5]NX01锁定原点 [6]NX02锁定原点 [7]清空记录文件`（`scripts/launch_control.py`），**鼠标真的能点**，也可以按数字键，控制台上会把实际执行的命令打印出来。起飞/降落触发机制按`CONTROLLER`自动切换：`px4ctrl`/`so3ctrl`走`quadrotor_msgs/msg/TakeoffLand`话题（TAKEOFF/LAND可反复触发，降落完能再起飞，但降落要求飞机已在悬停状态，正在执行任务会被拒绝）；`ros2_px4_stack`起飞靠`touch /tmp/takeoff_go`、降落靠直接调mavros`AUTO.LAND`，**这条链路架构上只支持起飞一次，降落后无法用这个控制台再次起飞**。锁定原点按钮跟`CONTROLLER`/`PLANNER`/`LOCALIZATION_SOURCE`都无关（见下面"起飞点一键锁定"一节），起飞前建议先按一次，飞机必须已经通电静止在起飞点上。**2026-08-12新增：起飞口令发送成功后会自动在后台拉起`auto_calibration_flight.py`**——飞机悬停3秒、在自己的局部系里前进1.1米、到达后等3秒、退回起飞点上空悬停，不需要操作员手动开goal窗口打坐标，专门给`origin_setter_node`的θ*估计攒位移样本（见"起飞点一键锁定"一节）。`[7]`清空记录文件删除`rosbag/`滚动块+三个容器持久化日志+双机姿态推力调试日志+自动事件日志，**不删`incidents/`下`save_incident.sh`手动留证的文件夹**，破坏性操作需要连按两次(5秒内)确认；这几个文件都是容器内root身份写出来的，删除动作走`docker exec`用容器内root身份执行，不是宿主机侧直接删（宿主机用户对这些文件没有写权限） |
| 4 | status(品红) | 双机集中状态表格 | `status_monitor.py`原地刷新位置/姿态欧拉角/推力油门/发布频率；同时后台起`attitude_thrust_logger.py`持续写`runtime_logs/<NS>/attitude_thrust_debug.log` |
| 5 | goal(黄) | 双机目标点手动输入面板 | `dual_goal_input.py`，见下面"发送目标点"一节 |
| 6 | record(紫) | 运行时数据滚动录制 | `record_rosbag.sh`+`prune_rosbag.sh`常驻后台跑，见下面"数据记录"一节 |
| 7 | logs(灰) | 三容器stdout持久化 | `tail_persist_logs.sh`，写到`runtime_logs/container_logs/`，容器`down`掉也不丢 |

**tmux状态栏**（屏幕最下面一行，任何窗口下都常驻可见）实时显示两机各子系统"雷位规控链"
（雷达点云/SLAM/规划器/板外控制器/MAVROS）健康位：绿=正常、红=掉线、灰=启动中或不监控。
出现红色说明对应子系统真的停了，切到status/NX01/NX02窗口查日志；事件时间线另外记在
`runtime_logs/incidents/health_events.log`，没盯着状态栏也能事后翻。

### 发送目标点

两种方式：

1. **RViz "2D Goal Pose"工具**——双机各有一个独立的工具实例（分别绑定`/NX01/rviz_goal_world`
   /`/NX02/rviz_goal_world`），只能点一个点、给一架飞机。2026-08-10之前点出来的是"当前
   Fixed Frame下的原始坐标"、不管Fixed Frame是不是这架飞机自己的frame都直接照发，切了
   Fixed Frame或者给另一架飞机点目标时会差出一个固定平移量；现在中间多了一层
   `rviz_goal_bridge_node`（`ego_planner_bridge`包，mighty/ego_planner两条规划器路径
   共用同一个节点），会用tf2把点击消息自带的`header.frame_id`（即当前Fixed Frame）
   变换到目标飞机自己的`<namespace>/odom`再发布，不管Fixed Frame设成哪一架飞机的frame
   都能点对，查不到TF变换时会拒绝转发（打印ERROR日志），不会用错误坐标飞出去。
2. **goal窗口的`dual_goal_input.py`**（推荐，一次给双机下目标点，自动换算）：输入
   **世界坐标**，支持两种格式：
   - `x y z`——同一个目标点同时发给双机
   - `x1 y1 z1 x2 y2 z2`——前三个给NX01、后三个给NX02
   
   脚本内部用`origin_setter_node`广播的`world -> {ns}/map`这条TF（含θ*旋转，
   2026-08-12起——之前是`INIT_X`纯x方向平移的简化公式，两机各自设置非零spawn yaw后
   这个简化假设不再成立，详见`dual_goal_input.py`文件头说明）自动换算成每架飞机自己的
   局部坐标再发布，不用手动心算。⚠️ 依赖两机都已完成起飞点锁定（见下一节）——没锁定
   时这个换算做不了，面板会提示"未标定"，指令也不会发送。

### 起飞点一键锁定（`uwb_origin_bridge`，2026-08-10新增）

⚠️ **2026-08-12起，这一步对所有`LOCALIZATION_SOURCE`都是必须的，不再只是
`uwb_slam`场景专属**——原来`uwb_sim/uwb_ground_truth_node.py`里有一份
`broadcast_map_tf()`，容器一启动就自动无条件广播一条`{i}/map -> {j}/map`
的TF（假设两机spawn yaw都是0、旋转部分写死单位四元数），让RViz就算没人
触发起飞点锁定也能勉强看到两机的相对位置。这份代码已经删掉了（两机各自
设置非零spawn yaw后，这个"都是0"的假设本来就已经错了，留着只会给出错误的
可视化结果，详见`DEBUG_JOURNAL.md` 2026-08-12"θ*标定出44.7度后RViz点云仍未
纠正"那条）。现在RViz要正确显示双机相对位置，**必须先给两机各自触发一次
下面这个`set_origin_from_uwb`**，靠`origin_setter_node`广播的`world ->
{ns}/map`让tf2把两机地图挂到同一棵树上——这也是唯一的数据源了，不再有
"没锁定也凑合能看"的旧备份行为。

真机上没有仿真里`INIT_X=AGENT_INDEX*3`这种预先写死的spawn偏移可用，需要
现场用UWB测一次绝对位置。用法：飞机通电、静止在起飞点上之后，按launch
控制台`[5]`/`[6]`按钮（或手动`ros2 service call /{NAMESPACE}/set_origin_from_uwb
std_srvs/srv/Trigger "{}"`——注意服务名不带中间的`origin_setter`节点名，
是ROS2相对名解析规则，跟`takeoff_gate`发布`/{NAMESPACE}/takeoff_land`是
同一个道理）——节点会取最近2秒内的
UWB样本，数量≥5个且标准差≤0.15m才认为可信，跟当前DLIO里程计位置算差值，
广播成一条`world -> {NAMESPACE}/map`静态TF（`world`是全队共享的单一父
frame，不带命名空间前缀——两机各自的`world -> {NAMESPACE}/map`独立锁定、
但共用同一个父frame才能让两机的TF树连通）并持久化到
`/tmp/uwb_origin_{NAMESPACE}.json`（容器重启会自动从这个文件恢复，不用
重新锁定）。返回`response.message`会直接说明成功/失败原因（样本不足/
抖动过大/里程计缺失三选一）。

这套仿真里没有真实UWB模块，数据源是`uwb_sim`新增的`/{NAMESPACE}/uwb/pose_abs`
话题（Gazebo世界真值+高斯噪声，跟已有的`frame_align`机间测距用同一个
`range_noise_std`噪声模型，语义不同：那个是"机间相对测距"，这个是"单机
绝对定位"）——先在仿真里验证这条"一键锁定"链路本身对不对，真机部署时只
需要把这一个话题的发布者换成真实UWB驱动，`uwb_origin_bridge`不用改一行
代码。

**2026-08-12新增：局部系跟全局系之间的旋转对齐（不再只处理平移）**——
`origin_setter_node`会持续订阅DLIO odom(局部系)和UWB位置(全局系)，飞机
每移动`rotation_seg_min_disp_m`(默认0.5)米就记一段(局部位移向量,全局
位移向量)，进`rotation_window_size`(默认50)段的滑窗，按最小二乘公式
`θ*=atan2(S,C)`在线估计局部系相对全局系的旋转角，累积到
`rotation_min_segments`(默认1)段就开始采信这个值（样本不够前θ*保持0，
退化成纯平移，等价于旧版行为）。飞机一旦开始移动就自动收敛，不需要手动
触发；起飞点静止锁定（上面`set_origin_from_uwb`那次性触发）记的是锁定
那一刻的局部/全局位置均值，当SE(2)变换的固定锚点，之后θ*每次更新都复用
这个锚点重新算平移`t=uwb_lock-R(θ*)·local_lock`并重新广播TF，不是锚点
本身也跟着漂移。诊断话题`/{NAMESPACE}/origin_setter/yaw_estimate`
（`std_msgs/Float64`，弧度）、`/{NAMESPACE}/origin_setter/yaw_sample_count`
（当前滑窗内的段数）可以实时看收敛过程，对应`LOCALIZATION_SOURCE=uwb_slam`
这个情景（见上面参数表）。这套逻辑不感知`LOCALIZATION_SOURCE`，`gt`/
`uwb_imu`模式下同样在跑，只是这两种模式局部系本来就跟全局系同向，
θ*会自然收敛到接近0。`dual_goal_input.py`和`status_monitor.py`的全局坐标
显示/换算（2026-08-12起）都已经改成读这条`world -> {ns}/map` TF，不再是
各自硬编码一份`INIT_X`纯平移——两处是独立实现（各自是独立脚本，不共享
公共库），但SE(2)公式必须保持一致，以后改这块要两个文件一起改。

**2026-08-12新增：`frame_align_bridge_node`（跨机map系对齐的硬件可迁移
实现）**——`uwb_origin_bridge.launch.py`跟`origin_setter_node`一起无条件
启动。`mighty`的机间避让闭环(`use_frame_alignment:=true`)需要一条
`/frame_align/{i}/{j}`(`geometry_msgs/msg/TransformStamped`)告诉自己"僚机
map原点在自己map系下的位置"，原来这条话题只有`uwb_sim/uwb_ground_truth_node.py`
一个仿真专用实现（直接读Gazebo世界真值算差），没法搬到真机。`frame_align_bridge_node`
是它的硬件可迁移版本：不算任何矩阵，直接用上面`world -> {ns}/map`这条已经
标定好的TF，`lookup_transform(target=f'{own_ns}/map', source=f'{other_ns}/map')`，
tf2会沿共享的`world`父frame自动把两段链路组合乘出来，语义完全等价。
`PUBLISH_FRAME_ALIGN=false`时会关掉`uwb_sim`那个真值版本（`uwb_ground_truth_node`
的`publish_frame_align`参数），两个数据源不会同时抢占同一个话题；默认`true`时
仍用真值版本，不受影响——这个开关跟`LOCALIZATION_SOURCE`是正交的，2026-08-12
之前是从`LOCALIZATION_SOURCE=uwb_slam`自动推导，现在拆成独立环境变量（见上面
参数表），任何`LOCALIZATION_SOURCE`取值下都能单独验证"不靠仿真真值也能对齐"。
`ego-planner-swarm`那边有一份未提交
的`swarm/use_frame_alignment`WIP，语义跟`mighty`的这套一致但目前没有接入
任何launch文件，尚未启用。

**2026-08-12新增：起飞后自标定动作（`scripts/auto_calibration_flight.py`）**
——launch窗口的起飞按钮发送成功后自动在后台拉起，不需要操作员再手动开goal窗口
打坐标积累θ*样本：悬停3秒→在自己的局部系里前进1.1米→到达后等3秒→退回起飞点
上空悬停。1.1米是配合`rotation_seg_min_disp_m`=0.5米这个分段阈值选的（单程能
稳定切出至少2段）。全程只在飞机自己的局部坐标系里操作，不依赖任何世界坐标
换算——这跟"用θ*把局部坐标换算成世界坐标"是同一件事的两个方向，这里反过来
依赖世界坐标换算就本末倒置了。两机各自独立执行，互不依赖。

### 数据记录与事故留证

record窗口后台常驻：`record_rosbag.sh`每5分钟滚动录一个bag块（TF/双机共享轨迹/
mavros状态位姿/目标点/规划轨迹可视化/占据栅格/UWB真值等关键话题，**不含原始点云**，
体积太大）到`runtime_logs/rosbag/`；`prune_rosbag.sh`只保留最近12个块（约1小时
滚动窗口），超过自动删最老的。

事情刚发生（炸机、规划异常）时手动跑：

```bash
docker exec docker_sim-flight-stack-nx01-1 /tmp/save_incident.sh "NX02悬停不动报fopt17647"
```

把最近3个bag块（覆盖事发前后约15分钟）连同两机的姿态/推力文本日志，复制一份到
`runtime_logs/incidents/<时间戳>_<描述>/`长期保留，不受滚动清理影响。

### 镜像导出 / 迁移到新机器

```bash
./scripts/bundle.sh export [输出目录]              # 打包三个镜像+项目配置成可迁移的bundle
./scripts/bundle.sh restore <bundle目录> [--dest 目标目录]   # 在新机器上恢复
```

原理是`docker save`/`docker load`整份镜像层，跳过"重新拉源码+重新colcon build"，
恢复速度只取决于传文件和解包，不需要重新跑`fetch_sources.sh`（要网络/代理）。三个
镜像共享同一个`mighty-base:humble`父镜像的层在磁盘上只存一份，一次性对三个镜像
一起`docker save`比分别导出省好几GB。新机器上仍需自行装好`nvidia-container-toolkit`
（GUI硬件加速）和有可用的X server（`start.sh`会自动跑`xhost`，但得先有个X session）。

## 参数与环境变量参考

改动方式统一：编辑`docker-compose.yml`里对应的默认值，或者用环境变量覆盖后再
`docker compose up -d`（例如`V_MAX=1.5 docker compose up -d`）——这些参数都是
entrypoint运行时读取/传给launch文件的，**不需要重新build镜像**，但需要重建容器
（`up -d`，不是`restart`，见上面"启动/停止/重启"一节）。带"两机必须一致"标注的
参数，NX01/NX02两份`environment`要同步改，不一致会破坏多机协同的假设。

### 顶层开关（两机必须一致）

| 变量 | 默认值 | 可选值/说明 |
|---|---|---|
| `PLANNER` | `ego_planner`（2026-08-10改，原默认`mighty`） | `mighty`——原默认，本项目自研规划器；两机必须一致，不然`/broadcast_bspline`机间防撞互相看不到对方。可以跟`CONTROLLER`/`LOCALIZATION_SOURCE`任意组合 |
| `CONTROLLER` | `px4ctrl`（2026-08-10改，原默认`ros2_px4_stack`）——ROS2(Humble)移植版。2026-08-13确认：`LOCALIZATION_SOURCE=uwb_slam`下`PLANNER=mighty`/`ego_planner`都已实测起飞-跟踪-降落验证通过，不再是"仅编译冒烟测试"级别；**`LOCALIZATION_SOURCE=uwb_imu`时仍然禁止使用**（见下面LOCALIZATION_SOURCE那一行，2026-08-12实测炸机，这条限制没有变）。`gt`定位源下这个组合还没有专门验证记录，见上面"常用启动组合"表格。 | `ros2_px4_stack`——原默认，唯一经过完整起飞-跟踪-降落仿真验证的板外控制器，也是目前唯一跟`LOCALIZATION_SOURCE=uwb_imu`兼容的控制器，**架构上只支持起飞一次**（降落靠直接调mavros `AUTO.LAND`，跟起飞是两个互不知情的一次性动作，降落后无法用launch控制台再次起飞）；`so3ctrl`——复用px4ctrl飞行状态机+kr_mav_control的SO3几何位置环控制律，2026-08-13确认：`LOCALIZATION_SOURCE=uwb_slam`下`PLANNER=mighty`/`ego_planner`都已实测验证通过，见`vendor/px4ctrl_ros2/so3ctrl/NOTICE.md`，同样**`LOCALIZATION_SOURCE=uwb_imu`时禁止使用**；`pt4ctrl`（2026-08-13新增）——复用px4ctrl的飞行状态机（起飞/悬停/降落/RC失控保护逐字未改，因此**支持反复起降**），但删掉了推力模型估计+姿态解算这两步控制律，改成把状态机算出的位置/速度/加速度/偏航参考量打包成`trajectory_msgs/MultiDOFJointTrajectory`发到`mavros/setpoint_trajectory/local`，交给PX4固件自己的位置控制环解算姿态/推力（仿`ros2_px4_stack`的转发方式），代码在`vendor/px4ctrl_ros2/pt4ctrl`。设计目标是同时具备"能反复起降"（继承px4ctrl状态机）和"对更新率不敏感、兼容`LOCALIZATION_SOURCE=uwb_imu`"（继承`ros2_px4_stack`的转发方式）两个优点——**2026-09-13更新：`LOCALIZATION_SOURCE=uwb_imu`下`pt4ctrl`真机+仿真均已验证通过，没有问题，不再是"全新代码、完全未知"**（上面"常用启动组合"表格新增了`uwb_imu`+`mighty`/`ego_planner`+`pt4ctrl`两行✅），是2026大赛contest task（`docker_sim/2026大赛任务系统全流程任务仿真实现方案.md`）实际选用的组合；`gt`/`uwb_slam`定位源下`pt4ctrl`仍然没有专门验证记录。四者都支持`PLANNER=ego_planner`（`ros2_px4_stack`靠`poscmd_to_goal_node`桥接，见`ego_planner_bridge`包） |
| `LOCALIZATION_SOURCE` | `gt`（2026-08-10改，原默认`dlio`/`uwb_slam`） | 三选一（2026-08-12前是四选一，`dlio`和`uwb_slam`两个值合并成了`uwb_slam`一个——两者在flight-stack这边的实际行为从一开始就完全一样，都是起真实DLIO SLAM，拆成两个值纯粹是历史包袱，详见`DEBUG_JOURNAL.md`"评估LOCALIZATION_SOURCE功能重复"那条）：`uwb_slam`——真实DLIO SLAM，三个模块（DLIO/规划器/控制器）一起测，同时配合`origin_setter_node.py`无条件常驻跑的在线SE(2)旋转估计——用滑窗最小二乘（飞机每移动`rotation_seg_min_disp_m`=0.5米记一段局部/全局位移向量对，进`rotation_window_size`=50段的滑窗，累积到`rotation_min_segments`=1段（第一段就发布））估计局部系相对全局系的旋转角θ*，飞机一动起来就自动收敛，不需要手动触发；起飞点静止锁定（`set_origin_from_uwb`服务）仍然是一次性的，把锁定那一刻的局部/全局位置均值当SE(2)变换的固定锚点，之后θ*每次更新都复用这个锚点重新算平移`t=uwb_lock-R(θ*)·local_lock`并重新广播TF。诊断话题`.../origin_setter/yaw_estimate`（弧度）、`.../origin_setter/yaw_sample_count`可以实时看收敛过程；`gt`跳过DLIO SLAM，直接用Gazebo真值定位（只测规划+控制）。`gt_odom_bridge_node`发布odom话题+TF，`PLANNER=ego_planner`时额外起`gt_cloud_bridge_node`把原始点云变换到odom系，两个规划器在`gt`模式下都能正常避障；`uwb_imu`——情景一(SLAM完全失效，UWB+IMU替代，见`docker_sim/多机空间坐标对齐问题_设计讨论纪要.docx`第8.1节)，`uwb_imu_fusion_node`把UWB(位置+yaw)+IMU(roll/pitch)直接拼装成完整位姿发给`mavros/vision_pose/pose_cov`，`local_position_readback_node`把PX4融合后的`mavros/local_position/odom`读回改名重发，接口跟`gt`/`uwb_slam`两种模式完全一样，`gt_cloud_bridge_node`同样复用。⚠️ **`uwb_imu`模式下`px4ctrl`/`so3ctrl`禁止使用**（`PLANNER=mighty`/`ego_planner`下都是）——这两个板外几何姿态控制器要求companion computer持续高频喂姿态目标，`uwb_imu`模式的位置反馈要经PX4内部EKF2+MAVLink往返（不像`uwb_slam`模式能直连DLIO原生话题），实测速率上限在~48Hz左右（`px4_onboard_position_rate_firmware_default.patch`已经把MAVLink固件配置从30Hz提到100Hz，实测也只能到这个数量级，往上调配置值没用），对`px4ctrl`/`so3ctrl`不够用，2026-08-12实测两次炸机，详见`DEBUG_JOURNAL.md`。**能配的控制器是`ros2_px4_stack`（原本唯一验证过的，`PLANNER=mighty`/`ego_planner`都没问题，但架构上只支持起飞一次）和`pt4ctrl`（2026-08-13新增，2026-09-13确认真机+仿真均已验证，支持反复起降，是2026大赛contest task实际选用的）**——`pt4ctrl`对更新率不敏感（继承`ros2_px4_stack`的转发式控制思路），所以能兼容`uwb_imu`的低反馈速率，同时又继承了`px4ctrl`的飞行状态机、支持反复起降，弥补了`ros2_px4_stack`"只能起飞一次"的限制。真机部署前需要单独用`uwb_slam`验证完整DLIO SLAM链路——默认`gt`排除了SLAM本身收敛/漂移的影响，规划+控制链路问题更容易定位，但也意味着默认组合下不覆盖DLIO这一段。2026-08-13新增三个单机专用值`single_slam_only`/`single_uwb_imu`/`single_uwb_slam`（分别对应`uwb_slam`不起`uwb_origin_bridge`/`uwb_imu`/`uwb_slam`，强制要求`NUM_AGENTS=1`），是Jetson Orin NX单机真机移植前的仿真彩排配置，用法见上面"单机仿真彩排"一节，双机场景不要用这三个值。 |
| `SLAM_BACKEND` | `dlio`（2026-08-14新增） | 只在`LOCALIZATION_SOURCE=uwb_slam`/`single_uwb_slam`/`single_slam_only`（即真的起SLAM的几个值）下生效，`gt`/`uwb_imu`不跑SLAM对这个开关没有意义。`dlio`——原有行为不变，经过实测；`point_lio`——`point_lio_ros2`（`dfloreaa`第三方ROS2移植，官方`hku-mars/Point-LIO`没有ROS2分支），本项目补了两个patch：`point_lio_avia_custom_msg.patch`恢复被这个fork注释掉的Livox CustomMsg支持（本项目仿真的mid360点云走的正是这个消息类型），`point_lio_docker_sim_launch.patch`做命名空间+接口映射（跟DLIO一样remap到`dlio/odom_node/odom`、`dlio/odom_node/pointcloud/deskewed`两个固定话题名+`<ns>/odom`→`<ns>/base_link`TF，`mighty`/`ego_planner_bridge`/`gt_odom_bridge`/`ros2_px4_stack`不用改一行代码就能切换后端）。⚠️ **只做过patch可应用性+`colcon build`层面的验证，还没有实际跑过仿真起飞**，`mapping.satu_acc`/`satu_gyro`（IMU饱和量程）、`mapping.extrinsic_T`/`extrinsic_R`（LiDAR-IMU外参）两组参数沿用`point_lio`官方示例默认值/`hku-mars`硬件标定值，未针对本项目仿真单独核实，见`docker/Dockerfile.flight-stack`"7b. point_lio"一节和`DEBUG_JOURNAL.md`相关记录。⚠️ 2026-08-14实测双机场景下`pointlio_mapping`单进程CPU占用能冲到11+核（`point_lio`/`ikd-Tree`都没有自己限制OpenMP/pthread线程数），下面`POINT_LIO_CPU_CORES`就是为这个问题加的；`fast_lio`——`hku-mars/FAST_LIO`官方ROS2分支，2026-09-07新增仿真集成（`fast_lio_namespace.patch`命名空间/frame_id、`fast_lio_pointcloud_qos.patch`修复Livox CustomMsg订阅QoS不兼容导致收不到点云的问题、`fast_lio_imu_last_lidar_end_time_init.patch`修一处未初始化内存的防御性问题、`fast_lio_docker_sim_launch.patch`新增仿真专用launch文件+针对仿真稀疏点云（每帧固定1000点，只有真实Mid-360的1/20~1/30）调整的`point_filter_num`/`filter_size_surf`/`filter_size_map`/`mapping.det_range`/`cube_side_length`五项参数），2026-09-08接通`DEPLOY_TARGET=hw`（`fast_lio_hw_launch.patch`新增hw专用launch文件，故意不搬那五项仿真调参、全部沿用upstream默认值，理由和已知缺口见该文件头docstring）。⚠️ **point_lio/fast_lio均尚未实机/仿真长时间验证过**，fast_lio目前只在仿真里做过部分实测（QoS/稀疏点云两个问题已确认修复，本地地图无界增长导致的周期性卡顿问题仅部分缓解、根因未完全定位），真机这条路径2026-09-08才刚接通、还没有实际飞行验证过 |
| `POINT_LIO_CPU_CORES` | `4`（2026-08-14新增） | 只在`SLAM_BACKEND=point_lio`时生效。用`taskset`把`pointlio_mapping`钉在固定CPU核范围内（按`AGENT_INDEX`错开，`NX01`用第0..N-1个核、`NX02`用第N..2N-1个核，两机不抢同一批核），同时把`OMP_NUM_THREADS`设成同一个值封顶OpenMP线程池大小——不限制的话单进程能冲到11+核，双机场景下host load average能冲到32核机器的52，点云/里程计速率被拖到个位数Hz。这个值×双机数量不能超过宿主机实际核数，机器核数不够或想留更多余量给Gazebo/其它节点时调小 |
| `PUBLISH_FRAME_ALIGN` | `true` | 只影响`sim-world`容器，控制`uwb_ground_truth_node`要不要发布仿真真值版的`/frame_align/*`（喂给`mighty`的`use_frame_alignment`机间避让闭环）。设成`false`时，跨机map系对齐的唯一数据源变成`frame_align_bridge_node`（硬件可迁移，靠TF查`origin_setter_node`标定出来的`world`系链路，见`uwb_origin_bridge`包），专门用来验证"不靠仿真真值、光靠真实标定也能对齐"这件事，跟`LOCALIZATION_SOURCE`是两个正交的开关，任何`LOCALIZATION_SOURCE`取值下都能单独设（2026-08-12新增，之前是从`LOCALIZATION_SOURCE=uwb_slam`自动推导的）|
| `WORLD_ENV`（仅sim-world） | `fire_drill_room`（2026-09-09改，原默认`simple_room`） | `fire_drill_room`——大赛正式任务场景（20x25x6米，起降点/立柱/物资点/火情标识等`contest_mission`任务道具都在这个场景里），改成默认值是因为之前有一次没显式传`WORLD_ENV`就直接`docker compose up -d`，容器悄悄回退成`simple_room`、任务道具全部不存在但日志毫无ERROR，很隐蔽；`simple_room`——20x20x5米室内房间+4根柱子的早期占位场景，仍可用，需要显式传`WORLD_ENV=simple_room`；`hard_forest`/`easy_forest`/`medium_forest`/`dynamic_forest`等森林场景零额外依赖；`hospital`/`office`/`tunnel`需要额外资产，未实测验证 |

### mighty规划/避障/控制参数（`PLANNER=mighty`时生效，两机必须一致）

| 变量 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `V_MAX` | 1.0 | m/s | 最大飞行速度 |
| `A_MAX` | 3.0 | m/s² | 最大加速度 |
| `J_MAX` | 5.0 | m/s³ | 最大jerk |
| `OMEGA_MAX` | 0.10472 | rad/s | 最大角速度（约6°/s，轨迹优化器内部机体角速度可行性约束，不是yaw朝向变化速率） |
| `TIME_WEIGHT` | 1.5e+2 | - | 轨迹用时代价权重，越大越倾向"晚刹车"的激进速度曲线，太大会导致到达目标点超调 |
| `GOAL_SEEN_RADIUS` | 3.0 | m | 进入这个半径开始用硬终态(零速度)约束规划 |
| `GOAL_RADIUS` | 0.3 | m | 判定"到达目标点"的半径 |
| `DYNAMIC_WEIGHT` | 1e+2 | - | 动态障碍物/僚机避让权重 |
| `PLANNER_CW` | 3.0 | m | 动态避障安全间距阈值 |
| `DYN_CONSTR_THRUST_WEIGHT` | 1e+3 | - | 推力约束违反的惩罚权重，0=关闭推力限制 |

### ego-planner规划/避障参数（仅`PLANNER=ego_planner`时生效，两机必须一致）

| 变量 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `EGO_PLANNING_HORIZON` | 15.0 | m | 单次规划前瞻距离 |
| `EGO_OBSTACLES_INFLATION` | 0.6 | m | 障碍物膨胀半径（含飞机半宽+跟踪误差余量） |
| `EGO_REPLAN_THRESH` | 0.1 | s | 重规划最小间隔 |
| `EGO_LAMBDA_COLLISION` | 1.0 | - | 避障代价权重（同时也是机间避障`calcSwarmCost`共用的权重） |
| `EGO_LAMBDA_FEASIBILITY` | 0.3 | - | 动力学可行性代价权重 |
| `EGO_YAW_DOT_MAX` | 0.8 | rad/s | 最大偏航角速度（约45.8°/s，跟PX4固件`MPC_YAWRAUTO_MAX`硬顶吻合） |
| `EGO_LOCAL_UPDATE_RANGE_XY` | 15.0 | m | 局部占据地图X/Y半径 |
| `EGO_MAX_RAY_LENGTH` | 15.0 | m | 单帧点云raycasting有效标记距离 |
| `EGO_DIST0` | 1.0 | m | 障碍物安全距离阈值（硬截断代价，够远直接归零，不是越远越好） |
| `EGO_SWARM_CLEARANCE` | 1.0 | m | 同伴间隔安全距离阈值（各向异性椭球，纯Z向安全间隔=4倍本参数、纯XY向=2倍本参数） |
| `EGO_NO_REPLAN_THRESH` | 1.0 | m | 离目标点还剩多远时停止重规划、把当前这条轨迹飞完（`fsm/thresh_no_replan_meter`）。1.0米时实测反馈"抵达目标一米多位置时飞机明显停一下，再飞向终点"——接近过程中一直在重规划，每条轨迹自己"减速到静止"的收尾段没机会真正飞完就被换掉，一进这个半径重规划停止，飞机被迫把当时唯一那条轨迹飞完，"停顿"正是这段收尾第一次真正执行的表现，是已知但无害的视觉观感问题。⚠️ **不要调到0.2附近**——2026-08-13实测过，`planner_manager.cpp::reboundReplan()`自己写死了一个`(start_pt-local_target_pt).norm() < 0.2`的"离目标太近放弃"分支，失败重试路径里有一处对趋近零向量的叉积做`.normalized()`（Eigen对零向量单位化会产出NaN），很可能是"the drone is in obstacle"这次复发的根因（未100%实锤），已改回1.0避开这个ego-planner-swarm上游代码本身的数值稳定性问题，详见`DEBUG_JOURNAL.md` |
| `EGO_USE_FRAME_ALIGNMENT` | `true` | - | 是否用`/frame_align`把收到的僚机轨迹坐标变换到本机局部系再用；没有UWB真值来源时应设`false`，否则轨迹会一直因为等不到变换而被丢弃、退化成完全不避让 |

### 双机身份与场景参数

| 变量 | 说明 |
|---|---|
| `NAMESPACE` | `NX01`/`NX02`，必须跟MAVROS/规划器/DLIO/控制器统一 |
| `AGENT_INDEX` | `1`/`2`，决定每架飞机在世界系里的spawn点X偏移（`INIT_X = AGENT_INDEX*3`） |
| `NUM_AGENTS` | 固定`2` |
| `ROS_DOMAIN_ID` | 固定`21`（2026-08-30改，仿真专用域，跟真机`docker-compose.hw.yml`固定的`20`刻意不同——比赛场景下选手自己写的ROS2代码不显式设`ROS_DOMAIN_ID`时默认连不上仿真，防止命名空间`NX01`/`NX02`两边同名撞车导致指令发错目标），本机同时跑其它ROS2节点时注意别跟`21`冲突（`network_mode: host`下话题全局共享） |
| `BUILD_JOBS` | 镜像内编译并发度，默认4，按"可用内存(GB)/2"估算可调大，见上面"编译内存不够"一节 |

## 调试日志

从项目启动至今的完整调试/排查/问答记录（按时间顺序，逐条追加）搬到了
`DEBUG_JOURNAL.md`，本文件只保留"怎么用这个项目"这部分。查某个具体问题的
历史排查过程，去那边用关键词/日期`grep`；不建议从头通读，条目按时间顺序
堆叠，早期条目不一定还反映当前状态。

阶段性"当前状态"整理见根目录几份docx（`双机UAV仿真系统调试记录.docx`、
`无人机集群规划器选型调研.docx`、`双机避障排查与规划器原理问答记录.docx`）
——这几份是从调试日志里定期蒸馏出来的干净版本，比直接翻日志更适合查"现在到底
是什么状态"。
