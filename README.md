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

## 三个镜像划分依据（回顾）

- `sim-world`：Gazebo Classic世界 + 双机PX4 v1.16 SITL + UWB真值节点，只跑1份，因为两架飞机共享同一个物理仿真时钟和同一张地图。
- `flight-stack`：DLIO+mighty+ros2_px4_stack，跑2份（一机一容器），这个边界特意对齐"以后部署到真机上的机载电脑"的形态——今后上真机时，理论上只需要把这个镜像里"仿真传感器话题"换成"真实驱动话题"，容器结构基本不用大改。

## 已经落实的三处已知问题修复（patches/）

1. `patches/ros2_px4_stack_dynus.patch`：`create_rate().sleep()`死锁修复 + 补自动解锁/切OFFBOARD逻辑 + 把硬编码的`m=2.906`/`hover_thrust=0.50`改成读环境变量（由entrypoint从`vehicle_profile.yaml`解析后export）。
2. `patches/mighty_lidar_frame_id.patch`：`lidar_frame_id_`硬编码"NX01"的bug修复。
3. `patches/mighty_sensor_mount.patch`：Mid-360前倾角从原来的0.3弧度(~17.2°)改成30°(0.5236弧度)。

## 场景覆盖

`WORLD_ENV`环境变量对应mighty的`worlds/*.world`，全部场景都能选：
- **零额外依赖，直接能用**：`hard_forest`/`easy_forest`/`medium_forest`/`dynamic_forest`等森林类（约23个world文件都是这一档）
- **需要额外资产，见`fetch_sources.sh`里`gz_assets`那组**：`hospital`（AWS RoboMaker资产包）、`office`（willowgarage模型）、`tunnel`（DARPA SubT赛道资产）——这三个我只确认了外部资产的"仓库存在"，没有实测验证`model://`路径能不能在这套目录结构下正确解析，建议**优先用森林类场景把主链路跑通，医院/办公室/隧道这三个放第二批单独调**。

---

## ⚠️ 老实说：这一整套没有被实际build/run验证过

我在当前环境里没有条件真正跑一遍`docker build`+`docker compose up`（没有GPU/Gazebo/PX4编译环境），下面这些文件是**基于整个对话过程中反复核实过的真实仓库地址、真实文件路径、真实代码内容**写出来的最佳努力搭建方案，不是"已验证能跑通"的成品。上线前请重点核对这几类风险：

1. **`spawn_agent.launch.py`**：`sim-world-entrypoint.sh`里调用了这个launch文件来给单个机型做多机spawn，但我们在mighty仓库里实际确认存在的是`onboard_mighty.launch.py`里内嵌的spawn逻辑，没有单独确认过一个叫`spawn_agent.launch.py`的可复用文件——这个文件大概率需要你自己从`onboard_mighty.launch.py`里把spawn那段摘出来改造一下，不是开箱即用的。
2. **PX4经典Gazebo多机端口/环境变量细节**：`sim-world-entrypoint.sh`里PX4 SITL的启动命令是照着官方`sitl_multiple_run.sh`的思路手写的简化版，没有对照官方脚本里MAVLink端口偏移的具体规则逐一核实，建议直接参考PX4源码里`Tools/simulation/gazebo-classic/sitl_multiple_run.sh`的真实实现来对一遍。
3. **三个patch文件**：都是根据本次对话里实际读到的源码文本手写的unified diff，行号/上下文可能跟你实际拉到的代码有细微出入，Dockerfile里已经做了`git apply --check`失败时不中断构建、只打印警告的容错，但打印警告不代表问题被解决了，**必须去日志里确认这三个patch真的生效了**，没生效就要手动改。
4. **`hospital`/`office`/`tunnel`三个world的外部模型路径**：`GAZEBO_MODEL_PATH`目前只是把整个`gazebo_models_external`目录囫囵扔进去，具体`model://`名字能不能在这个目录结构下被正确检索到，没有实测，大概率需要你手动把子目录理一遍。
5. **`uwb_sim`节点**：`/gazebo/model_states`话题名、`gazebo_msgs/ModelStates`消息格式是经典Gazebo+`gazebo_ros`桥接的标准配置，逻辑上应该对，但这个节点代码本身没有实际跑过，建议先单独`colcon build`看语法/依赖有没有问题，再接入完整系统调试。
6. **DLIO话题名**：`flight-stack-entrypoint.sh`里假设DLIO的launch文件接受`pointcloud_topic`/`imu_topic`这两个参数名，这是照着`vectr-ucla/direct_lidar_inertial_odometry`惯常的参数命名猜的，没有逐一核对`feature/ros2`分支当前的实际launch参数名，请以你`fetch_sources.sh`拉下来的那份代码为准核实一遍。

简单说：**骨架和思路是扎实的（每一处设计决策都能追溯到这次对话里核实过的具体证据），但细节层面的文件名/参数名/端口号存在"最佳猜测但未实测"的部分，请按上面6条清单逐一过一遍，而不是直接当成能一键跑通的成品。**

## 简单室内场景 `simple_room`（用来替代森林场景先跑通SLAM/全流程）

森林场景（`easy_forest`/`hard_forest`）在接入iris+mid360融合模型后复现了一个
`gzserver` `exit code -6`崩溃，出现在"NX01模型spawn完、还没起PX4实例"这个时间点，
跟MAVLink时序无关；ros2 launch的日志聚合在崩溃瞬间会丢内容，没抓到详细abort信息。
判断是复杂场景（169+个障碍物模型）本身的问题，暂不深挖，改用一个更简单的场景把
主链路先跑通：`worlds/simple_room.world`——20x20x5米室内房间，4面墙 + 5根直径
0.5米/高5米的柱子当障碍物。`env:=simple_room`（`docker-compose.yml`里
`WORLD_ENV=simple_room`），`base_mighty.launch.py`的`world_mapping`已加映射。

**柱子摆放的教训**：第一版为了"离两机spawn点(3,0,3)/(6,0,3)留够净空"，把柱子摆在
`(-5,-5)/(-5,5)/(0,-7)/(9,6)`，结果全堆到了房间四角贴墙——柱子是要给规划器当
障碍物绕的，堆墙角起不到任何作用。改成摆在房间中部、交错不对称分布
（`(-6,3)/(-3,-6)/(1,4)/(4,-4)/(7,3)`，5根），同时保证离四面墙、离两机spawn点都
至少留了3米净空。

**发现的新bug（跟森林场景崩溃是两回事）**：`sim-world-entrypoint.sh`原来的循环是
逐架飞机"spawn模型→紧接着起PX4实例"交替进行。NX01的PX4实例一起来就按lockstep
协议高频轮询传感器数据，但紧接着立刻又在gzserver主线程里做NX02那份重活（渲染
jinja、`spawn_entity`同步RPC、`LivoxPointsPlugin`加载80万行`mid360.csv`，实测
耗时数秒）——这几秒物理循环被占住，NX01的轮询在这个窗口里连续超时
（`ERROR [simulator_mavlink] poll timeout`），而且这个lockstep一旦在启动阶段
错位就再也回不来了：哪怕之后几分钟world完全空闲，NX01的日志也只会一直刷
poll timeout，永远到不了`Ready for takeoff`；MAVROS那边`connected`会显示
`true`（心跳走独立低频通道），但`state`话题的`system_status`会一直卡在
`0=MAV_STATE_UNINIT`，飞控实际没真正起来，规划/控制那层完全用不了。NX02因为是
最后一个spawn的，起PX4轮询时后面没有别的重活跟它抢，从来没复现过——两架表现
不一致正是这个先后顺序竞态的特征。**修复**：把entrypoint里的循环拆成两阶段
——第一阶段把所有飞机的模型全部spawn完（重活集中在这一阶段做完，此时还没有
任何PX4实例在轮询），全部spawn完之后统一等3秒让物理稳定，第二阶段再统一起所有
PX4实例。**已验证**：用户手动rebuild+up之后，日志里能看到两阶段的新log行，
NX01/NX02都到了`Ready for takeoff!`，`mavros/state`话题两边`system_status`都是
`3=STANDBY`（不再是卡死的`0=UNINIT`）。

## 定位数据来源（DLIO SLAM / Gazebo真值）切换开关

系统自带三个要分别验证的模块——DLIO SLAM、mighty规划器、板外控制器
ros2_px4_stack——不一定要绑在一起测。排查数据流时确认：mighty的`state`话题
和PX4的`vision_pose`（经`repub_odom.py`）**最终订阅的是同一个话题名**
`<namespace>/dlio/odom_node/odom`（真正的`dlio_odom_node`发布的话题），
所以只要有别的东西发布到这同一个话题名，下游两条链路完全不用改代码。

新增`src/gt_odom_bridge`包（跟`src/uwb_sim`同级的一手代码，不是patch）：
订阅sim-world容器发布的`/plug/model_states_plug`（Gazebo世界真值），按
entity_name（NX01/NX02）找出自己的位姿+速度，转发成跟DLIO同名的
`nav_msgs/Odometry`话题。flight-stack-entrypoint.sh新增
`LOCALIZATION_SOURCE`环境变量（`docker-compose.yml`里两个flight-stack服务都能配）：
- `dlio`（默认）：跑真正的DLIO SLAM，三个模块一起测。
- `gt`：跳过DLIO，改跑`gt_odom_bridge`——只测规划器+板外控制器+飞控，排除SLAM
  收敛/漂移的影响。

两种模式互斥（entrypoint里`if`二选一启动，不会同时跑），因为两边发布的是同一个
话题名，同时跑会有两个发布者抢话题。**不发布TF**（DLIO自己发布的
`odom->base_link`等三条TF在gt模式下不存在）——已经确认mighty的点云建图
（`mapCallback`/`occupancyMapCallback`/`unknownMapCallback`）不查TF，只用
`state`话题的位姿数值自己做变换，所以gt模式下点云建图不受影响；唯一会查TF的
`getInitialPoseHwCallback`只在硬件模式下由定时器触发，跟这个仿真测试模式无关。
这个开关本身还没有重新build验证过（只验证过其中一部分——见下面两条bug是在验证
这个开关设计过程中顺带发现的，跟开关本身能不能跑通是两码事）。

## 顺带发现的bug：UWB真值节点从来没收到过数据（已验证修好）

排查定位切换开关时，在跑着的容器里用`ros2 topic info`挨个核对了一遍
"话题名两边到底对不对"，揪出一个跟当前问题无关、但一直在静默生效的bug：
`uwb_ground_truth_node.py`订阅的是默认话题名`/gazebo/model_states`，但
`base_mighty.launch.py`起的world里`gazebo_ros_state`插件套了`/plug`
namespace、把话题remap成了`/plug/model_states_plug`。`ros2 topic info -v`
实测确认：`/gazebo/model_states`是0个发布者、这个节点自己是唯一订阅者；
`/plug/model_states_plug`是1个发布者、0个订阅者——两边各说各话，`ros2 topic
hz /frame_align/NX01/NX02`当时直接超时，证实这个话题从成立以来就没真正发布过
任何数据，多机坐标系对齐功能一直是空转的。**修复**：直接改
`src/uwb_sim/uwb_sim/uwb_ground_truth_node.py`（一手代码，不用走patch），
订阅话题改成参数化，默认值改成`/plug/model_states_plug`。**已验证**：重新
build后`ros2 topic hz /frame_align/NX01/NX02`能看到~57Hz的数据。

## mighty规划器一直收不到任何一帧点云——第一次判断错了，真正原因是缺一个建图节点

同一轮排查里，一开始判断是`onboard_mighty.launch.py`里
`lidar_point_cloud_topic = 'livox/lidar' if use_hardware else 'mid360_PointCloud2'`
的问题——`use_hardware:=true`之后`livox/lidar`这个话题在仿真里根本不存在
（`ros2 topic info`确认0发布者），照这个思路加了`patches/mighty_lidar_point_cloud_topic_arg.patch`
给`onboard_mighty.launch.py`加了独立的`lidar_point_cloud_topic`覆盖参数，
entrypoint里显式传`mid360_PointCloud2`覆盖掉。

**⚠️ 这个判断是错的**。下一轮重新build验证时，直接读了`mighty_node.cpp`源码
+ `ros2 node info /NX01/mighty_node`查实际订阅列表，发现mighty_node.cpp
**压根没有一个叫`lidar_cloud_in`的订阅**——`onboard_mighty.launch.py`里
`mighty_node`那个`remappings=[('lidar_cloud_in', lidar_point_cloud_topic), ...]`
是一段死代码，映射到哪个话题名都不影响mighty_node的实际行为，之前那个patch
无害但没有真正起作用。

真正的订阅是`occupancy_grid`+`unknown_grid`这对同步的PointCloud2（不管
`use_hardware=true`还是`sim_env=gazebo`都一样，只有`sim_env=fake_sim`才直接吃
原始点云）。这两个话题应该由`acl-mapping/global_mapper_ros`包的
`global_mapper_node`节点从原始点云+位姿算出来——这个节点在仓库里、编译得出来
（`colcon build`日志里能看到`Finished <<< global_mapper_ros`），参数也基本
跟这套仿真对得上（`global_mapper_node.launch.py`默认`depth_pointcloud_topic`
就是`mid360_PointCloud2`、`pose_topic`默认`state`、`hardware:=false`时
`occupancy_grid_topic`/`unknown_grid_topic`默认remap成`occupancy_grid`/
`unknown_grid`，正好是mighty订阅的裸话题名），但**从来没有在这套docker_sim的
任何entrypoint/launch文件里被启动过**。mighty规划器从最开始到现在，实际上
一直没有收到过任何障碍物数据——这才是真正原因，跟话题名对不对无关，是整个
中间节点都没起。

**修复**：`flight-stack-entrypoint.sh`里mighty启动之后新增一步，起
`global_mapper_ros global_mapper_node.launch.py`：
- `hardware:=false`（不是true）——`hardware:=true`会选用`cfg/hw_global_mapper.yaml`，
  那份配置是给地面机器人调的（`world_dimensions: 15x15`、z轴以1.5m为中心）；
  `hardware:=false`选用的`cfg/global_mapper.yaml`里`world_dimensions: 20x20`、
  `origin: (0,0,3)`，注释就写着"grid covers x,y∈[-10,+10]"，正好对得上
  `simple_room`世界的范围。
- `global_frame:="${NAMESPACE}/map"`——单独覆盖（这个参数不跟`hardware`联动，
  可以独立传），跟mighty自己在`use_hardware:=true`时用的`"${NAMESPACE}/map"`
  保持一致，否则两边TF/坐标系对不上。

`patches/mighty_lidar_point_cloud_topic_arg.patch`本身没删，留着无害（万一以后
mighty_node.cpp重新加回这个订阅名就用得上），但**不要再当成点云链路的修复**，
真正的修复是上面这个global_mapper_ros启动步骤。**已验证**：重新build后
`global_mapper_ros`进程确实起来了，`/NX01/occupancy_grid`/`unknown_grid`话题
也确实在以8~9Hz发布——但发布不代表数据有用，见下面两条。

## global_mapper_ros接入后暴露的两个新bug（已修复，待验证）

1. **`{namespace}/map`这个TF frame从来没被发布过**：`global_mapper_node`的日志
   狂刷`lookupTransform(NX01/map -> NX01/NX01_livox) failed: "NX01/map" ...
   does not exist`——每一帧点云的坐标变换回调都失败、直接跳过，
   `occupancy_grid`/`unknown_grid`话题虽然在正常发布（订阅者收得到消息），但
   payload大概率一直是空的/不更新的。根因：`onboard_mighty.launch.py`里
   `static_tf_node`（发布`{namespace}/map -> {namespace}/odom`恒等TF）早就
   定义好了，但quadrotor+`use_hardware`+`use_onboard_localization`这个分支
   拼`nodes_to_start`列表时只append了`hw_odom_to_state_node`，`static_tf_node`
   从来没被真正加进去过（隔壁star_robot/red_rover分支代码里甚至留了个
   `#static_tf_node`的孤立注释，像是本来想加、后来漏掉了，没人发现因为
   之前压根没人真正用过global_mapper_ros这条链路去触发这个lookupTransform）。
   **修复**：`patches/mighty_static_tf_fix.patch`把`static_tf_node`加进这个
   分支的`nodes_to_start`。**已验证**：重新build后再看
   `docker exec ... ros2 topic echo /tf_static --once`，能看到
   `NX01/map -> NX01/odom`确实在发布了；`static_tf_map_to_odom`进程也确认在跑。

2. **飞机初始化在3米高空、没解锁就自由落体砸到地面**：`sim-world-entrypoint.sh`
   spawn飞机时`-z "3"`（3米悬空），`mighty_enable_gravity.patch`打开了重力，
   但飞机armed一直是false（电机不转），俩条件叠加就是从3米高空自由落体——
   实测确认过：`mavros/local_position/pose`的z从3掉到了-0.09，几分钟内一直
   停在地面附近没恢复。真机这样摔早就炸了，仿真里虽然不会"报废"，但物理冲击+
   姿态翻滚会让DLIO/global_mapper_ros在一个错误的初始状态下开始收敛，不是想要
   的测试起点。**第一次修复**：spawn高度改成`z=0.83`——PX4官方
   `Tools/simulation/gazebo-classic/sitl_multiple_run.sh`里`spawn_model()`自己
   spawn iris用的地面净空高度。**这个高度对这个具体的iris模型来说还是不够**：
   查了`iris.sdf.jinja`，`base_link_inertia_collision`是个以base_link原点为
   中心的`0.47x0.47x0.11`米箱体，箱底在原点下方0.055米，也就是说z=0.055才是
   箱体刚好贴地、零净空的高度，0.83米依然是明显悬空。**改成z=0.1**（比零净空
   高度只多留1.5厘米，肉眼几乎看不出下落），让飞机像真机一样"停在地上"等解锁
   起飞。`sim-world-entrypoint.sh`的spawn坐标和`flight-stack-entrypoint.sh`的
   `INIT_Z`（TF用）两边都要改、保持一致。

## 已修复：occupancy_grid的QoS不匹配（找到根因了：rclcpp的一个经典陷阱）

实测`ros2 topic info /NX01/occupancy_grid -v`：`global_mapper_ros`的发布者是
`BEST_EFFORT`，`mighty_node`的订阅者显示的是`RELIABLE`——RELIABLE订阅者连不上
BEST_EFFORT发布者（DDS QoS兼容性规则：发布者的可靠性必须≥订阅者请求的），
mighty_node因此实际一条`occupancy_grid`/`unknown_grid`消息都收不到，即使话题
本身在正常发布。用`ros2 param get /NX01/mighty_node use_hardware`确认运行时
`par_.use_hardware`确实是`true`，对应`mighty_node.cpp`里
`sub_occupancy_grid_`/`sub_unknown_grid_`那两处订阅，QoS写的是
`rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))`。
**根因**：这是rclcpp一个广为人知的陷阱——`QoSInitialization::from_rmw()`
只把profile里的`history`+`depth`两个字段抄过去，`reliability`/`durability`
这些其它字段依然是`rclcpp::QoS`的默认值(`RELIABLE`)，不是`profile`本身的
`BEST_EFFORT`。证据确凿：这个文件里另一处ESDF订阅的注释早就点破过这个坑
（`"unlike QoSInitialization::from_rmw which only sets history+depth"`），
用的是正确写法`rclcpp::SensorDataQoS()`，但当时没有把
occupancy_grid/unknown_grid/sensor_point_cloud这三处一起改。**修复**：
`patches/mighty_occupancy_grid_qos_fix.patch`把这三处统一换成
`rclcpp::SensorDataQoS()`（`sim_env=gazebo`分支的message_filters同步订阅
`occup_grid_sub_.subscribe(..., rmw_qos_profile_sensor_data, ...)`没有这个坑
——那是直接传`rmw_qos_profile_t`给message_filters的API，不经过
`QoSInitialization`这层转换，没改）。**已验证**：重新build后
`ros2 topic info /NX01/occupancy_grid -v`两边（`global_mapper_ros`发布者、
`mighty_node`订阅者）都是`BEST_EFFORT`了。这个patch同时接入了`sim-world`和
`flight-stack`两个镜像的patch链（`mighty_node.cpp`两边都编译）。

## 已修复：global_mapper_ros查点云TF用的frame名字跟Gazebo插件发布的对不上

上面第1条修好`{namespace}/map`之后，重新build验证：`NX01/map -> NX01/odom`
这条TF确实在发布了（`static_tf_map_to_odom`进程活着，`/tf_static`里能看到），
但`global_mapper_node`日志里的报错还在，从"NX01/map does not exist"变成了
"NX01/NX01_livox does not exist"，`lookupTransform failed`计数还在涨（7000+）。

第一次诊断猜错了方向：以为是`global_mapper_ros.cc`里`drone_frame`参数为空时
自己猜的雷达TF frame名字`{quad}/{quad}_livox`跟DLIO实际发布的
`{namespace}/lidar`对不上，加了`drone_frame:="${NAMESPACE}/lidar"`覆盖——这个
改动本身没错（`ros2 param get`确认参数确实传到位了），**但它解决的是
`lidar_frame_`这个成员变量参与的几处lookupTransform（FOV/可见性相关计算），
不是实测报错刷屏的那处**。真正刷屏的
`[PointCloudCallback] lookupTransform failed: "NX01/NX01_livox"`来自
`global_mapper_ros.cc`的`PointCloudCallback`函数，它查的是**点云消息自带的
`header.frame_id`**（`cloud_msg->header.frame_id`），根本不读`drone_frame`/
`lidar_frame_`这个参数，是完全独立的一条路径。往上追到Gazebo雷达插件
`livox_points_plugin.cpp`：`cloud.header.frame_id = ns_ + "/" +
raySensor->Name();`——SDF里雷达sensor本身就叫`{ns}_livox`（见
`gen_iris_mid360_sdf.py`），拼出来正好是"NX01/NX01_livox"，这是Gazebo插件
自己的命名习惯，跟DLIO的"NX01/lidar"是两套完全不搭界的名字，中间没有任何TF
把它们连起来。**真正的修复**：在`flight-stack-entrypoint.sh`里补一条恒等
静态TF（物理上是同一个传感器、同一个位置，纯粹是两边代码各自叫法不同）：
`NX01/lidar -> NX01/NX01_livox`，零偏移。这样`global_mapper_node`查
"NX01/map -> NX01/NX01_livox"时能沿着
`map->odom->base_link->lidar->NX01_livox`这条完整链路解出来。这两个修复
（`drone_frame`覆盖 + 恒等TF alias）都保留了，各自解决不同的lookupTransform
调用点。还没有重新build验证过。

## 待查：dlio_odom_node偶发SIGABRT崩溃（内存看起来不是原因）

跑了一段时间（日志里能看到几轮正常的性能统计输出）之后，`dlio_odom_node`
以`exit code -6`崩溃，没能捕获到具体的abort原因——per-node日志文件
（`/root/.ros/log/dlio_odom_node_*.log`）是0字节，Docker聚合的stdout里也没有
任何"process has died"或崩溃相关字样，`/var/crash/`（host）里也没有对应的
apport报告——跟这个项目里之前好几次崩溃（gzserver在forest场景下的exit -6）
同一个模式，崩溃瞬间的诊断信息全部丢失，找不到直接证据。

**排查是否内存泄漏**：检查了崩溃后依然存活的同一条DLIO流水线的兄弟进程
`dlio_map_node`——运行了约17分钟（1002秒）之后RSS只有72.5MB，容器总内存占用
只有822MB/31GB（2.59%），host端`dmesg`里也没有任何OOM killer的痕迹。这些迹象
都不支持"内存泄漏导致崩溃"——如果真的在泄漏，这条兄弟进程或者容器总内存应该
already有明显偏高的迹象。**结论：目前没有证据支持内存泄漏，更可能是assertion
失败/未捕获异常/数值问题（比如退化点云导致的PCL/GTSAM内部错误）之类的
"崩溃型"故障，但没有实际的崩溃堆栈，无法进一步确认具体是哪一种**。要往下查
的话，需要先解决"崩溃诊断信息丢失"这个基础设施问题（比如把DLIO单独重定向到一个
持久化日志文件、或者想办法让coredump真正落盘到容器里能读到的路径），这个还
没有做，暂缓排查。

## 已修复：DLIO自己的调试仪表盘刷屏（每一帧点云都打印一次，人眼看不清）

用户反馈DLIO的终端输出太多、看不清。查了`direct_lidar_inertial_odometry`
（DLIO自己的上游代码，这个仓库之前没碰过这块）的`src/dlio/odom.cc`：
`callbackPointCloud`（每一帧点云的回调，这套mid360仿真是100Hz）末尾原来是
```cpp
this->debug_thread = std::thread( &dlio::OdomNode::debug, this );
this->debug_thread.detach();
```
——每一帧点云都新起一个detached线程，清屏（`printf("\033[2J\033[1;1H")`）+
打印一份完整的状态仪表盘（位置、姿态、计算耗时、CPU/RAM占用等）。100Hz意味着
理论上每秒清屏刷新100次，人眼完全看不清，而且多个并发的detached线程互相抢着
清屏/打印，之前日志里出现过的乱码状文本（几份仪表盘的内容交织在一起）就是这个
原因。**修复**：`patches/dlio_debug_throttle.patch`加了一个时间戳成员
`last_debug_print_time_`，在`callbackPointCloud`里判断距上次打印是否已经过了
`debug_print_period_`（硬编码1秒，没有加成config可调的yaml参数——如果以后
觉得1秒还是太快/太慢，直接改这个常量就行，没必要为这么小的事再加一层yaml配置）
才真正起线程打印，否则跳过。只接入了`flight-stack`镜像（`sim-world`不build
DLIO）。还没有重新build验证过。

## DLIO SLAM发散根因深挖：点云30%是"打空了"的假点，房间没天花板

output限流生效后终于看清了崩溃前的最后状态（见上一节3.13对应内容）：飞机静止不动，
DLIO却算出"走了1762米"、位置飘到`(-55,-32,-34)`——SLAM本身发散了，
`NotEnoughMemoryException`只是发散之后关键帧暴涨、某条消息序列化超限的最终症状。
按用户要求从"雷达数据是否正常/足够"、"雷达真实朝向是否正确"两个方向查起。

**雷达朝向核验（结论：没问题）**：验证DLIO配置的`baselink2lidar`外参旋转矩阵
`[0.866025,0,0.500001; 0,1,0; -0.500001,0,0.866025]`，代入标准Ry(30°)公式：
把机体前向向量(1,0,0)代入这个矩阵，算出`(0.866,0,-0.5)`——z分量为负，说明雷达"前方"
相对机体确实是前倾向下30度，跟`quadrotor.urdf.xacro`里`joint origin rpy="0 0.5236 0"`
的设计意图完全一致，不是装反了。

**点云质量分析（找到关键证据）**：用rclpy直接订阅解析一帧`mid360_PointCloud2`
（需要用BEST_EFFORT QoS匹配Gazebo插件的发布端，rclpy默认RELIABLE订阅会因QoS不兼容
收不到任何消息——又是一次QoS陷阱模式的复现）。距离分布：
```
[0.0-1.0m): 251 (25.1%)   ← 极近场，贴地扫描
[1.0-8.0m): 162
[8.0-12.0m): 268 (26.8%)  ← 大概率是墙面真实回波
[12.0-16.0m): 19
[16.0-19.9m): 0           ← 完全空的，很反常
[19.9-21.0m): 300 (30.0%) ← 卡在最大量程
```
`[16-19.9m)`完全没有点、`[19.9-20m)`却聚集30%，这种"断崖式"分布是"传感器打空后按
配置的最大量程(20米)填充哨兵点"的典型特征，不是真的探测到了20米外的东西。往回查
`simple_room.world`：只有地面+四面墙，没有天花板。mid360前倾30度后仍有相当一部分
射视场朝上（原始视场本身在传感器自身坐标系下偏向上方，min_angle=-7.2°、
max_angle=+55.2°），房间没有顶，这些射线在开放空间里永远打不到东西。

**交叉核实DLIO预处理**：`src/dlio/odom.cc`的CropBox滤波器（默认1.0米）能挡掉
25.1%的近场噪声（这部分设计合理，防止扫到自身），但体素降采样（默认5cm）之外
完全没有基于距离/命中有效性的过滤——30%的满量程假点会被当成真实观测，参与GICP
配准。这些假点角度相邻、距离几乎完全相同，在5cm体素网格里会大量重叠合并，
用户追问"deskewed points只有497个是否太少"：497是预处理最终产出（1000原始点→
去掉约251个近场点→体素降采样），数字本身不算异常，问题不是"点数不够"，是这些点里
混入了大量不代表真实几何的假点，污染了配准依赖的表面法向量/对应关系估计。

**修复**：给`simple_room.world`补一个天花板（z=5，跟地面对称的plane），让原来
朝上打空的射线也有真实几何可反射。按用户要求，天花板`<visual>`加
`<transparency>0.9</transparency>`做成几乎透明（GUI里能看到房间内部/飞机），
`<collision>`保持不变——SDF里视觉外观和射线传感器检测用的碰撞几何体是完全独立的
两个块，视觉透明不影响激光反射。只涉及`sim-world`镜像（world文件只在sim-world的
Dockerfile里被使用）。已通过完整patch链验证，**尚未重新build验证点云质量/SLAM
发散是否真的改善**。

## 双重修复：打空的射线不该填最大量程，该填0让下游滤波器去掉

用户指出：给房间补天花板只是从"场景设计"角度绕开问题，更根本的修复应该在
`livox_points_plugin.cpp`（Gazebo雷达仿真插件）本身——射线真的打空时，不应该在
最大量程处硬造一个"探测到了"的假点，应该填0，让下游滤波器自然去掉。

查了插件源码`OnNewLaserScans()`里处理每个测距结果的逻辑，发现打空判断早就有了，
但处理方式不对：
```cpp
if (range >= maxDist)
{
    range = maxDist;
    // continue;   ← 这行本来是想跳过打空的点，被注释掉、换成了填maxDist
}
```
`// continue;`这行注释本身就说明原作者曾经想"跳过"打空的点，后来改成了填充
maxDist（原因不明，可能是怕`continue`导致CustomMsg和PointCloud2两路消息长度对
不上）。**修复**：`patches/livox_zero_out_of_range.patch`把`range = maxDist`
改成`range = 0`——这样`point = range * axis`直接坍缩成`(0,0,0)`，天然落在DLIO
自己CropBox滤波器（`src/dlio/odom.cc`，默认`crop_size_=1.0`米，`setNegative
(true)`保留框外的点、去掉框内的点，本来就是为了滤掉贴近雷达原点的自身遮挡噪声
设计的）的去除范围正中心，会被这同一个已有的滤波器顺带滤掉，不需要在插件里
再写一遍过滤逻辑，也不依赖"世界一定有天花板"这个前提——哪怕以后换到一个没有
封闭天花板的场景，打空的射线也不会再被误当成"探测到了"。

这个坑其实是这个项目里*已经追踪过*的同一条线索：`livox_pointcloud_timestamp.patch`
的注释里早就记录过"两架飞机accel bias顶到`abias_max: 5.0`上限、原地飘出几十到
上千米"这个症状（跟这次发现的"走了1762米"是同一类问题），当时诊断为"GICP退化成
整帧配准、精度低"，加了per-point timestamp让DLIO走逐点deskew的精细路径——这个
修复是对的，但只解决了这一层，没有解决"点云本身混入大量假点"这另一层问题，
两个坑叠在一起才是本次静止发散的完整picture。

`patches/livox_zero_out_of_range.patch`只接入了`sim-world`镜像（这个插件只在
仿真里存在，真机换成真实Livox驱动就不需要它了）。已通过完整patch链验证，
尚未重新build验证。

## 交叉核实：DLIO的IMU外参变换在重力对齐之前就应用了，不是"没变换到水平坐标系"

用户追问是否可能是DLIO最终输出没有正确变换到水平的世界坐标系（怀疑倾斜外参没在
姿态估计前生效）。查了`transformImu()`函数（`src/dlio/odom.cc`）：这个函数在
IMU标定/重力对齐代码运行**之前**就被调用，内部用`extrinsics.baselink2imu.R`把
角速度和线加速度从倾斜的mid360 IMU坐标系转回水平的base_link坐标系（线加速度
还带了考虑`baselink2imu.t`杆臂效应的修正项）。同样，点云预处理
`preprocessPoints()`里`this->T_prior * this->extrinsics.baselink2lidar_T`也是
在送入GICP之前就把倾斜坐标系下的点变换回base_link参考系。**结论：外参变换的
调用时机和实现都是对的**，不是这次发散的原因，排除这个假说。

## 交叉核实：飞机机体IMU和mid360自己的IMU没有被混用

用户提醒要区分飞机自带IMU和mid360自己的IMU，不能混用。查了两者的SDF定义：
- iris机体自带IMU：`iris.sdf.jinja`里`libgazebo_imu_plugin.so`，挂在
  `/imu_link`（未倾斜，随机体本身姿态），发布话题`/imu`，只给PX4自己的
  `mavlink_interface`插件（`imuSubTopic`）用，供PX4内部EKF使用。
- mid360自己的IMU：`gen_iris_mid360_sdf.py`里`libimu_plugin.so`，挂在
  `{ns}_mid360_link`（跟激光雷达同一个link，同样带30度前倾姿态，没有额外的
  `<pose>`覆盖，正确继承倾斜），发布话题`mid360/imu`。

两者话题名（`/imu` vs `mid360/imu`）完全独立不冲突，`flight-stack-entrypoint.sh`
里DLIO启动参数明确传的是`imu_topic:="mid360/imu"`——静态检查确认DLIO消费的
确实是mid360自己的倾斜IMU，没有跟机体IMU混用。这一轮排查是静态代码检查，容器当时
处于停止状态，**尚未做运行时的实际验证（比如对比两个话题的实时数据/静止状态下
mid360 IMU算出来的重力矢量应该带~30度倾斜分量）**，等下次重新build起来之后应该
补一次实时确认。

## 新回归：加了天花板之后，飞机会陷到地面以下、严重侧翻（已定位，已修复待验证）

重新build+up之后用户反馈"飞机出现而后又消失了，一架都看不到"。排查：

- `docker ps`确认容器没有崩溃重启，`gzserver`/`gzclient`进程都还活着。
- `/plug/model_states_plug`话题确认NX01/NX02都还在场景的模型列表里（没有被删除），
  但直接查两架飞机的实际世界坐标：**NX01 z=-2.633，NX02 z=-2.643**——两架都陷到
  地面（z=0）以下超过2.6米，而且姿态四元数换算出来倾斜了约73度，是"炸飞/陷入"级别
  的物理异常，不是正常的贴地静止状态。整个过程日志里没有任何报错/警告。
- PX4日志显示`system_status`卡在`0=UNINIT`、反复报`Preflight Fail: ekf2 missing
  data`——这是陷入地下+严重侧翻这个异常物理状态的自然后果（IMU读数被
  contact-penetration回弹力污染，EKF2判定数据不可信、拒绝初始化），不是独立的
  新bug。
- 顺带发现`gzclient`（GUI渲染进程）CPU占用异常高（约950%，整个`sim-world`容器
  900%+），host端1分钟负载16.01——记录下来，但没有确凿证据证明这是"陷地"的
  直接原因，只是一个值得关注的伴生现象。

**根因判断**：这是加了天花板之后新出现的问题，之前没有天花板时飞机从来没有出现
过这种情况。房间的地面用`<plane>`（法线朝上、无限延伸的半空间几何体），天花板
第一版也用了`<plane>`（法线朝下）——这是这套world里第一次出现"两个无限半空间
正对着"的碰撞体组合，ODE物理引擎在这种配置下接触解算容易不稳定。四面墙从一开始
用的都是有限厚度的`<box>`，从来没出过问题。

**修复**：把天花板的`<collision>`和`<visual>`都从`<plane>`改成跟墙一致的
`<box><size>20.3 20.3 0.1</size></box>`（0.1米厚、四周留了跟墙一样的0.3米
外扩避免边缘留缝），保留`<visual>`的`<transparency>0.9</transparency>`。只涉及
`sim-world`镜像。**已验证**：重新build后飞机停在`z≈0.054`米（跟理论零净空
高度0.055米几乎精确吻合），姿态四元数是单位四元数（完全水平），陷地/侧翻
问题确认修好了。

## 天花板/陷地修好之后，SLAM还是照样发散——继续深挖，找到了dt=0以外的另一个数值陷阱

天花板改box之后，飞机不再陷地，但DLIO还是照样崩：`NX01`崩溃前"走了2097米"、
`NX02`崩溃前"走了542米"（accel bias精确顶到`params.yaml`里`abias_max`配置的
`5.00000000`上限），说明"点云30%假点"不是（唯一）根因——用同样的方法重新
核实了点云质量：加了天花板+`livox_zero_out_of_range.patch`之后，实测一帧点云
里满量程假点已经归零（`[19.90-21.00m): 0`），点云本身现在是健康的，问题在
别处。

**核实雷达外参/传感器读数是否物理自洽**：直接echo一帧`mid360/imu`原始数据，
`orientation`换算出来正好是30度（跟安装角度吻合），`linear_acceleration`是
`(-4.9, 0, 8.487)`，代入"世界系重力反作用力`(0,0,9.8)`经`Ry(-30°)`转换到
mid360自己倾斜坐标系"的公式算出来完全一致——传感器读数本身在物理上是自洽、
正确的，不是装反了或者算错了。

**对比机体自己的IMU（用户建议）**：`mavros/imu/data`显示的orientation主要是
yaw分量（roll/pitch接近0，符合"机体本身是水平的"预期），`linear_acceleration`
量级也对（~9.49，跟g接近），但`orientation_covariance`/`linear_acceleration_
covariance`是正常的非零值，而`mid360/imu`的对应字段全是0——不过查了DLIO源码，
它压根不读IMU消息里的这几个covariance字段（用的是自己`dlio.yaml`里配置的
噪声参数），这个差异只是"表面不同"，不是发散的真正原因，排除。

**真正找到的问题：mid360/imu发布间隔极不规律，存在近零dt尖峰**：
`ros2 topic hz /mid360/imu`实测：平均~470Hz（跟声明的500Hz接近），但
`min interval`显示`0.000s`（近零间隔的相邻消息），偶尔还有`0.104s`的大间隙。
`odom.cc`里有两处直接拿"相邻两帧IMU消息的时间差"当除数用：
- `transformImu()`里的杆臂修正项：
  `(ang_vel_cg - ang_vel_cg_prev) / dt`，结果被加进"修正后"的线加速度里
  （这个"修正后"的消息就是DLIO实际用来做估计的输入）。
- 主IMU回调（IMU标定完成后的分支）里算出来的`dt`会存进`imu_meas.dt`，直接被
  `propagateState()`拿去做速度/位置积分。

两处原来都只用`if (dt == 0)`挡"正好是0"这一种情况，挡不住"非零但极小"——而
实测证实"非零但极小"（近零间隔）比"正好是0"更常见。极小dt配上仿真里几乎零
噪声的IMU读数中偶发的微小抖动，除法/积分时会被放大成数值尖峰，是DLIO静止
状态下依然发散的一个具体、可验证的推手。

**用户提议的备选方案**：改用飞机自己的IMU（经mavros）而不是mid360自己的IMU，
避免这套自定义Gazebo IMU插件的发布时序问题。实测对比：`mavros/imu/data_raw`
的`min interval`稳定在`0.015s`（没有近零尖峰，明显更规律），但频率只有
~47Hz——受MAVLink传输带宽限制，比mid360的~470Hz慢了一个数量级，对DLIO做
点云去畸变需要的插值精度来说偏粗。两者各有取舍，没有直接采用"换IMU源"这个
方案，改为在原地做数值防护。

**修复**：`patches/dlio_imu_dt_clamp.patch`把两处`if (dt == 0)`都改成
`if (dt < 1e-4)`，用同样的`1/200`兜底值兜住"过小"而不只是"等于0"的情况。
接入`flight-stack`镜像的patch链（`sim-world`不跑DLIO）。**已验证：这版修复
没有解决发散**——重新build后NX01"走了2328米"、NX02同样崩溃，比之前更严重，
dt钳位不是（唯一）根因，SLAM发散问题目前仍未解决。

## 用户问：两个SLAM有没有互相串扰？—— 排查结果：DLIO本身没有，但发现了几个节点名冲突

`ros2 node list`实测直接报了`"share an exact name"`警告，确认存在真实的节点名
冲突：

- **`/static_tf_map_to_odom`**（本session之前加的`static_tf_node`修复）和
  **`/static_tf_livox_alias`**（本session之前加的雷达TF别名修复）——这两个都
  是**我自己这轮引入的bug**：`Node()`定义时没设置`namespace=`/节点名没带
  `${NAMESPACE}`前缀，导致NX01/NX02两边起的节点literally同名。发布的TF
  **数据本身**是正确加了命名空间的（`{namespace}/map`、
  `{namespace}/lidar`），撞名不影响已经发布出去的内容，但节点名冲突本身是
  真实的ROS graph卫生问题。**已修复**：
  `patches/mighty_static_tf_namespace_fix.patch`给`static_tf_node`加了
  `namespace=namespace`；`flight-stack-entrypoint.sh`里雷达TF别名的节点名
  加了`${NAMESPACE}`前缀。

- **`/map_to_odom`（重复两次同一进程内）、`/init_pose_to_camera_init`、
  `/body_to_base_link`、`/base_to_d455`**——这几个是`ros2_px4_stack`自己
  `dynus_mavros.launch.py`里的**既有代码**（不是这个session引入的），源头
  是这套launch文件是从单机Fast-LIO时代的DYNUS代码改过来的，`camera_init`/
  `body`/`map`/`world_mocap`这些帧名当时是全局单例设计，没有为多机场景加
  命名空间。查了`repub_odom.py`/`get_init_pose.py`两个消费者的源码：都只是
  **注释里提到**这几个帧名，实际代码**没有任何`lookupTransform`/
  `TransformListener`调用**去真正读取它们的解算结果——是从Fast-LIO时代遗留
  的死代码，两架飞机各自发布的`camera_init`/`body`会互相覆盖对方的TF父子
  关系（child frame不能同时有两个parent），这是真实的TF树冲突，但**目前
  没有任何东西实际消费这个冲突的结果**，不影响DLIO的SLAM计算，也不影响
  板外控制的实际数据路径。记录下来但没有修，这块要修需要给
  `dynus_mavros.launch.py`整体补命名空间，工作量不小，且当前确认无害，
  优先级放低。

**结论**：DLIO自己的SLAM计算链路（`{namespace}/odom`、`{namespace}/base_link`、
`{namespace}/lidar`、`{namespace}/imu`等全部正确命名空间隔离，`dlio_namespace.patch`
早就处理过）**没有发现互相串扰**，两架飞机的点云/IMU/里程计数据都是独立的。
发现的节点名冲突要么已修（本session引入的两处），要么确认是无害死代码
（`ros2_px4_stack`的既有问题）。**SLAM静止状态下依然发散的根本原因，到目前
为止还没有找到**——已经排除了点云假点、雷达外参、IMU covariance差异、
双机串扰四个方向；IMU dt钳位这个方向也**实测验证过没有效果**（重新build后
NX01"走了2328米"，比修复前更严重）。

## 用户提供的关键线索：真实硬件上IMU几十Hz都很稳，从来不用碰dt这类参数

用户反馈：在真实传感器上用DLIO，哪怕IMU只有几十Hz也非常稳，从来不需要动
`dt`钳位这类参数。这跟"仿真里470Hz的高频IMU反而更容易发散"直接矛盾，说明
问题大概率不在"频率高/dt瞬时过小"本身。真正的差异更可能是：**真实IMU天然
带噪声，仿真里这个IMU是完全理论精确值、零噪声**。查了
`gen_iris_mid360_sdf.py`里mid360 IMU传感器的定义：只有雷达那个ray sensor
显式配了`stddev=0.0`的零噪声，IMU sensor这边**压根没有`<imu>`噪声块**——
跟之前直接echo一帧`mid360/imu`发现读数精确符合理论公式、没有任何残差这个
观察完全对得上。某些依赖噪声做数值正则化的估计器/偏置估计逻辑，在完全零
噪声的输入下反而容易出问题（比如把浮点舍入误差当成有意义的"真信号"去
拟合）。

**修复**：给mid360 IMU传感器补上一组常见MEMS IMU量级的高斯噪声（角速度
噪声密度~2e-4 rad/s、加速度噪声密度~1.7e-2 m/s²——这是仿真里常用的量级
参考值，不是Livox Mid-360实测datasheet数字，如果之后有真实标定数据应该
换成那个）。`gen_iris_mid360_sdf.py`是直接COPY进`sim-world`镜像的一手代码
（不走patch），改完直接生效，语法和`.format()`调用生成的XML片段都已校验
通过。只影响`sim-world`镜像。**已验证：IMU噪声修复同样没有解决发散**——
重新build后DLIO跑了1-2分钟依然崩溃（"走了1813/1784米"），噪声也不是
根因。目前点云假点/雷达外参/IMU covariance/双机串扰/IMU dt尖峰/IMU零噪声
六个方向都已排除或修复但均未解决核心问题，SLAM静止状态下发散的根因
仍未找到。

## 端到端数据链路检查（定位→规划→控制→飞控，含集群通信）

用户要求检查整条链路是否打通（"别管合理不合理，先看数据通没有"）。结果：

- **点云、IMU（原始传感器数据）**：正常流动，不依赖DLIO。
- **`occupancy_grid`（global_mapper→mighty）**：连上了（1发布者1订阅者）。
- **`/trajs`（多机轨迹共享，集群通信）**：**2发布者2订阅者**，两架飞机确实在
  互相看到对方的规划轨迹。
- **`frame_align`（UWB坐标系对齐，另一条集群通信）**：~58-61Hz，正常。
- **`mavros/state`（飞控状态反馈）**：`connected:true`、`system_status:3`，正常。
- **DLIO里程计→state→规划→控制** 这条核心链路：在DLIO存活的窗口期（崩溃前
  那1-2分钟）确认是通的，`repub_odom_py`/`convert_odom_to_state`/`mighty`都
  正确订阅上了，链路本身没有断点，DLIO一崩这条链路就跟着断（`state`断流、
  `mavros/setpoint_raw/local`变成0发布者）——这是DLIO不稳定的下游连锁反应，
  不是链路设计问题。

**顺带发现并修复的隐患**：`mavros/vision_pose/pose_cov`在DLIO已经崩溃之后
**还在正常速率发布**——`repub_odom.py`的republish线程不管数据是什么时候
收到的，只要不是`None`就一直按100Hz重发"最后一帧"。PX4那边看这个话题的
更新频率完全正常，检测不出数据其实已经冻结了，真机上这是个安全隐患
（vision timeout保护会被这种"假心跳"绕过）。**修复**：
`patches/ros2_px4_stack_vision_pose_staleness.patch`加了一个"超过
`stale_timeout_sec`（默认0.5秒，可配置参数）没收到新数据就停止转发"的
看门狗，让PX4自己的vision timeout机制能正常触发，而不是被绕过去。接入
`flight-stack`镜像。已通过完整patch链验证，尚未重新build验证。

## 工具链：tmux分窗格监控 + RViz2看点云/建图

用户要求分开监控sim-world/NX01/NX02三边的日志（一个终端窗口挤三份日志太乱），
以及"Gazebo不显示雷达射线，改用RViz2看"（RViz2只管渲染，不像Gazebo那样在
物理仿真主循环里，不会重蹈"射线可视化拖慢物理导致PX4 poll timeout"的覆辙）。

- **`scripts/watch_sim.sh`** + **`scripts/mighty_sim.tmux.conf`**：装了`tmux`
  （host上装的，不需要重新build镜像），跑`./scripts/watch_sim.sh`会开一个
  tmux会话、横向三个窗格分别`docker compose logs -f`三个服务。按用户要求
  自定义了快捷键：前缀键从默认`Ctrl-b`换成`Ctrl-a`；`Ctrl-a k`（带二次确认）
  彻底关闭整个session（所有窗格一起关，跟只关当前窗格的`Ctrl-a x`区分开）；
  方向键切窗格（`prefix+方向键`，tmux默认行为，显式写进配置里不依赖系统
  默认tmux.conf）。
- **RViz2**：`base_mighty.launch.py`新增`rviz_config`覆盖参数（原来rviz配置
  文件名是根据`use_ground_robot`硬编码的，没法指定用哪份）。
  `rviz/multi_mighty.rviz`这份配置已经按NX01/NX02两机命名空间配好了（146处
  引用），双机场景直接能用。`sim-world-entrypoint.sh`默认`USE_RVIZ=true`，
  用`rviz_config:=multi_mighty.rviz`。跟`gzclient`共用同一套
  `DISPLAY`/`/tmp/.X11-unix`转发（docker-compose.yml里`sim-world`服务已经
  配了），不需要额外配置。接入`sim-world`镜像的patch链，已通过完整patch链
  验证。**已验证：RViz2确实启动了、用的确实是multi_mighty.rviz**，但暴露
  出下面两个新问题。

## RViz2接入后暴露的问题：DDS共享内存资源耗尽 + 点云Display默认关闭

**问题一：长时间session累积的Fast-DDS共享内存段耗尽，新参与者（含RViz2和
诊断命令）都抢不到端口**

用户反馈"rviz里看不到NX01的点云"。排查：`/NX01/mid360_PointCloud2`实测
**0个发布者**，连我自己的`ros2 topic`诊断命令都报错：
```
RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port12417: open_and_lock_file failed
```
查`/dev/shm`（因为`ipc: host`，这是宿主机真实共享内存，三个容器公用）：
**积压了1834个Fast-DDS共享内存段文件**。这套仿真跑了很久，DLIO反复崩溃
重启、期间还跑了几十条诊断命令，每个ROS2参与者都会占用几个SHM端口，长时间
累积后新参与者（这次新加的RViz2、我自己的诊断命令）抢不到可用端口，发现/
订阅全部失败。

**排查过程**：先按用户要求做了轻量级清理（用`lsof +D /dev/shm`找出当前
真正被进程占用的段、只删除没有任何进程占用的1695个孤儿文件），但没有
彻底解决——剩下的问题端口（如`fastrtps_port12413`）被**6个不同的
`static_transform_publisher`进程同时占用**（这个项目加了不少静态TF
发布节点：`static_tf_map_to_odom`、`static_tf_livox_alias`、`map_to_odom`
等，双机加起来一大堆），是Fast-DDS域内所有参与者共享的发现端口，新参与者
加锁时跟这么多长期占用者产生竞争。**最终修复**：完整重启整套仓构
（`docker compose down && up`，容器全停之后`/dev/shm`里的残留段确认0进程
占用，彻底清空，重新up）。**已验证**：重启后`/NX01/mid360_PointCloud2`
恢复正常发布（~85-90Hz），不再报SHM端口错误。

用户顺带问"Fast-DDS不是容易出问题吗，换一种DDS怎么样"——Cyclone DDS确实
不用Fast-DDS这套复杂的固定端口SHM预分配方案，多进程场景下确实更少见这类
问题，但这是影响全项目的基础设施决定，本轮没有实施，只是记录下来供以后
参考。

**问题二：即使话题本身通了，RViz这边的点云Display默认是关闭的**

DDS问题修复、确认点云话题本身在正常发布之后，用`ros2 topic info -v`查
`/NX01/mid360_PointCloud2`：`global_mapper_ros`订阅上了，但**RViz2完全
没有订阅**——问题不在命名空间也不在QoS（配置里`Reliability Policy: Best
Effort`本来就是对的，跟发布者匹配），而是`multi_mighty.rviz`里"NX01 Lidar
PointCloud"/"NX02 Lidar PointCloud"这两个Display的`Enabled`字段本来就是
`false`（勾选框没打开，RViz不会为禁用的Display创建订阅）。**修复**：
`patches/mighty_rviz_two_agents.patch`把这两个Display翻成`Enabled: true`
（顺带完成了用户要求的"删除多余飞机"——见下条）。

## 按用户要求裁剪RViz配置：只留NX01/NX02（原来有10架，不是9架）

`multi_mighty.rviz`原来是给最多10架飞机（NX01-NX10）准备的通用模板，这套
docker_sim只跑双机，RViz左侧面板堆了8架用不上的飞机分组。用户反馈"删除7架
飞机"——数的时候少数了一架，实际清点是10架不是9架，删的是8个分组
（NX03-NX10）不是7个，但目标一致：只留NX01/NX02。`patches/mighty_rviz_two_agents.patch`
删除了NX03-NX10对应的：顶层Group分组（每个约670行，含轨迹/点云/建图等
一整套per-agent Display）、TF显示里的`Frames:`白名单和`Tree:`层级列表、
`Window Geometry`里的`NX0X Front Camera: collapsed`停靠位置记录。`QMainWindow
State`那一大段Qt二进制序列化的窗口布局状态里虽然也编码了NX03-NX10相关
dock widget的历史位置，但那是UTF-16字节编码、不是明文ASCII，没有动它——
这个字段只影响"恢复上次窗口布局"这个体验细节，RViz对已经不存在的dock
widget记录会直接跳过，不会报错，没必要为了这个去动风险更高的二进制blob。
删除后YAML能正常parse，顶层`Displays`列表确认只剩`Common`/`Benchmarking`/
`Temporal SFC Debug`/`NX01`/`NX02`五项。只涉及`sim-world`镜像，已通过
完整patch链验证。

## 重新build运行后rviz依旧没有点云——找到真正的第三层原因：Fixed Frame设错了

上面三处都改完重新build运行之后，用户反馈"已经up了但是rviz上没有任何点云
出现"。直接进容器实测排查（不是猜，逐层核实）：

**先确认数据本身没问题**：`ros2 topic hz /NX01/mid360_PointCloud2`实测
~87-90Hz，正常。`ros2 topic info -v`确认**RViz2确实创建了DDS订阅**
（`Subscription count: 2`，其中一个`Node name: rviz2`），QoS双方都是
`BEST_EFFORT`匹配——说明上一轮"点云Display默认Enabled: false"那个bug
确实修好了，RViz真的在收数据，问题出在别处。

**真正原因：`Global Options → Fixed Frame`配的是裸`map`，这个frame在双机
命名空间体系里根本不存在于机身这条TF链上**。`tf2_echo map NX01/NX01_livox`
实测报"Could not find a connection... Tf has two or more unconnected
trees"——`map`这个frame确实存在（来自`ros2_px4_stack`遗留死代码发布的
`world -> map`静态TF，见前面3.17节记录的"Fast-LIO时代遗留死代码"），但
它是一棵完全独立、跟`NX01/NX01_livox`这条真实点云所在的TF树压根没有任何
连接的孤岛。RViz要把点云画到Fixed Frame下必须能解出一条完整变换链，解不出来
就直接不渲染，不会弹窗报错，非常容易被当成"点云没数据"。

**这一层还叠了SLAM本身的问题**：往下查`NX01/odom -> NX01/base_link`这条
动态TF（本该由`dlio_odom_node`持续发布）时发现`/tf`话题当时完全没有任何
消息——因为`dlio_odom_node`已经崩溃退出了（`ros2 node list`只剩存活的
`dlio_map_node`，`dlio_odom_node`不在列表里）。查日志确认：还是本文档
反复记录、至今未解决的静止发散老问题——本次实测NX01"走了2968~3016米"，
跑了约100秒后同样的`NotEnoughMemoryException`崩溃。也就是说：即使
Fixed Frame改对了，只要DLIO还是会在一两分钟内必然发散崩溃，RViz点云的
可视窗口天然就只有DLIO存活的这一小段时间——`NX01/lidar -> NX01/NX01_livox`
这条静态TF没问题，但上游`NX01/odom -> NX01/base_link`这条动态链路一断，
点云数据本身还在正常发布（不依赖DLIO，直接来自Gazebo插件），却再也没法
解算到任何Fixed Frame下渲染，这正好回答了用户的追问"DLIO很快就挂了，
但rviz的点云应该一直存在啊，为什么没有"——点云数据确实一直都在，缺的是
"数据"和"画布"之间那条依赖DLIO存活的TF链路。

**修复**（`patches/mighty_rviz_two_agents.patch`追加两处改动）：
1. `Global Options: Fixed Frame`从`map`改成`NX01/odom`——`NX01/odom`是
   `static_tf_map_to_odom`节点发布的恒等静态TF根节点（`NX01/map -> NX01/odom`），
   全程稳定存在、不依赖DLIO，只要DLIO一存活，`NX01/odom`往下到
   `NX01/NX01_livox`这条链路就能解出来，点云就能画出来；DLIO崩溃之后自然
   又会跟之前一样没法渲染（这是发散bug本身的后果，不是RViz配置能单独解决的）。
2. 顺带发现两个`NX0X Lidar PointCloud` Display除了`Enabled`字段外，紧跟在
   `Use rainbow: true`后面还各有一个独立的`Value: false`字段——`Grid`等
   确认能正常显示的Display对照下`Enabled`和`Value`应该是成对一致的
   （都是true）。上一轮的修复脚本只翻了`Enabled`那一处，漏了这个
   `Value`。两处一起改成`true`，避免这个字段本身也造成显示异常。

只涉及`sim-world`镜像，已通过完整patch链验证。

## 重新build运行后实测：RViz修复确认生效，"点云位置波动大"其实是SLAM发散老问题第一次被亲眼看见

用户重新build+up之后反馈"rviz可以看到点云了，就二号机点云没跑丢，但是位置
波动太大"。进容器直接核实：

**确认RViz这边的三处修复都部署对了**：容器里`multi_mighty.rviz`实测
`Fixed Frame: NX01/odom`（不是`map`了），两个`NX0X Lidar PointCloud`
Display的`Enabled`/`Value`都是`true`——跟patch的预期完全一致，没有部署
错误。

**"波动太大"不是新bug，是3.13-3.18节记录的DLIO静止发散老问题第一次被'看见'**：
分别查两架飞机当时的`dlio_odom_node`存活状态和崩溃日志——NX01"走了
2968~3016米"崩溃，NX02"走了255~402米"崩溃，都是同一个
`NotEnoughMemoryException`、同一个发散模式，只是两架飞机发散/崩溃的具体
时刻不同步（谁先崩，谁的画面就先卡住/消失；还活着的那架，观感上就是
"点云还在，但位置疯狂跳"——这正是发散过程本身的实时视觉表现，只是这次
是重新build运行后第一次真正能在RViz里亲眼看到"发散"这个过程，而不是像
之前几轮那样只能事后翻崩溃日志里的位置数字）。换句话说：这一轮的RViz
修复本身是成功的、达到了预期效果（点云能看见了），"波动大"恰恰是数据
本身有问题（未解决的DLIO发散）在正常渲染下的真实反映，不是渲染或TF'
配置又出了新问题。

## 新增：一条命令同时up三个容器+拉起tmux三窗格监控

`scripts/up_and_watch.sh`：`docker compose up -d`（后台起三个容器）之后
自动`exec`到`watch_sim.sh`把tmux三窗格监控拉起来，不用再分两步
（先手动`up`、再手动跑`watch_sim.sh`）。host上跑，不涉及镜像，不需要
重新build。

## 用户提出关键怀疑：mid360的IMU传感器插件本身有问题——顺着这条线挖到了目前为止最有说服力的一条根因

用户直接提出"我觉得是mid360雷达的imu有问题，也就是这个传感器插件有问题"。
查真正的插件源码（不是urdf注释提到的标准`gazebo_ros_pkgs`实现，是`mighty`
自己vendor进来的`src/sim/gazebo_ros_imu_sensor.cpp`，编译成`libimu_plugin.so`）：

**找到的问题**：`OnUpdate()`里`msg_->header.stamp`用的是裸
`rclcpp::Clock().now()`（永远是系统墙钟时间），而紧邻的两行被注释掉的
原始代码用的是`sensor_->LastUpdateTime()`（Gazebo仿真时钟）——像是有人
特意把sim时间换成了墙钟。DLIO的`odom.cc`只用`imu->header.stamp`算
`dt`（当前减上一帧）喂积分（`velocity += accel*dt`），如果这个`dt`是
墙钟时间流逝量而不是仿真物理时间真正流逝的量，只要两者不是严格1:1，
`dt`就会系统性偏差，双重积分（加速度→速度→位置）下误差会越滚越大。

**实测量化了这个偏差**：`gz stats`直接测——sim time 360.18s，real
time 403.01s，real-time-factor持续在0.96~0.97，400秒内累计漂了约
43秒，不是瞬时抖动，是持续性偏差（双机+双DLIO+gzclient渲染一起抢这台
主机CPU，上一轮记录的"gzclient单进程950% CPU"就是这个负载的侧面证据）。
这跟用户之前提供的关键线索完全对得上："真实硬件上IMU几十Hz都很稳"——
真机只有一个时钟，压根不存在sim/real两条时间线打架这回事，这类bug在
真机上根本不可能出现，只会在仿真里现形。

**为什么不能简单地直接切回sim time**：`patches/livox_pointcloud_timestamp.patch`
自己的注释里记录了教训——之前有人真的试过给点云盖sim time戳，直接导致
DLIO自己的Livox/Hesai传感器类型自动识别（`odom.cc`里`timestamp > 1e14`
这条阈值判断）把这种量级小得多的"仿真启动以来的秒数"误判成Hesai格式，
deskew逻辑找不到匹配的IMU区间，静默卡死、点云再也配准不出来（不报错、
不崩溃，`deskewed points`永远卡0，比现在的"发散"还糟）。

**修复思路**：给Gazebo sim time加一个固定的大常数偏移量再盖戳
（`SimTimeAsFakeEpoch()`），数值上看起来还是一个"正常"的绝对epoch时间，
能骗过那个`>1e14`阈值判断，但每两个连续时间戳之间的**差值**严格等于
仿真物理时间真正流逝的量，不再受real-time-factor漂移影响。IMU
（`patches/mighty_imu_sim_time.patch`）和点云
（`patches/livox_imu_lidar_sim_time.patch`）两边用同一个偏移常数，
让两条流继续共用同一条时间基准（deskew的IMU区间查找需要这个）。

只涉及`sim-world`镜像（`imu_plugin`目标在`CMakeLists.txt`里挂在
`if(BUILD_SIMULATION)`下，`flight-stack`不编译这份源码），已通过
完整patch链验证（mighty链13个、livox链5个都从pristine依次apply成功），
**尚未重新build验证是否真的解决了核心发散问题**——这是6轮排查以来
第一个有实测数据支撑（gz stats量化的持续性RTF偏差）、且有明确因果
机制（dt系统性偏差→双重积分误差累积→accel bias顶到上限）的候选根因，
但鉴于之前好几轮"看起来很有道理"的修复最后都被验证无效，这次的结论
同样要以重新build之后的实测发散情况为准。

## 实测结果：这次的sim time修复没有解决发散——第七个排除的假说

重新build+up之后立刻盯着日志看。**先确认部署确实生效**：直接echo了
`/NX01/mid360/imu`和`/NX01/mid360_PointCloud2`的`header.stamp`，两边
都是`sec: 1735689624`附近（等于约定的固定偏移`1735689600` + 十几秒
sim time），跟当时的真实墙钟epoch（`date +%s`实测约`1785733383`，
比stamp里的值大了近5800万秒）差得非常远——确认IMU和点云这次真的在用
Gazebo仿真时钟盖戳，不是之前的墙钟，patch确实部署对了、不是"以为改了
其实没生效"这种情况。

**但发散没有变好，NX01反而崩得更快了**：这次NX01从"0米"到accel bias
双轴顶到`±5.0`上限、"走了125米"、`NotEnoughMemoryException`崩溃，全程
只用了约20-30秒（之前几轮普遍是80-100秒才崩）。NX02同一时刻还活着，
但"走了1433米"、还在持续爬升，跟之前几轮的发散曲线没有本质区别。

**结论**：给IMU/点云换成不受real-time-factor漂移影响的仿真时钟，这个
改动本身是对的（部署确认生效，逻辑上也没有问题），但**没有解决核心的
发散问题**，甚至可能让它更快现形（也可能只是随机性，样本量太小不能
下强结论）。这是继点云假点、雷达外参、IMU covariance差异、双机串扰、
IMU dt尖峰、IMU零噪声之后，第七个"听起来很有道理但实测无效"的假说。
时间戳基准这个patch本身没有害处（sim time确实比墙钟更"对"，不受主机
负载影响），先保留，但说明IMU积分dt的系统性偏差不是（至少不是唯一）
压垮DLIO的那根稻草，核心矛盾大概率在GICP配准这一层本身——快速翻了一遍
`cfg/params.yaml`里的GICP参数（`maxCorrespondenceDistance: 0.5`、
`kCorrespondences: 16`、`leafSize: 0.25`等），数量级上没看出跟20米
房间这个场景明显不匹配的地方，但还没有深入去看配准本身的收敛质量
（fitness/residual这类指标），这是接下来一个值得挖的方向。

## 用户追问："mighty原本的仿真程序里根本没跑DLIO吧？"——查证属实，重新定位了问题的性质

直接查mighty自己的源码（不是这个项目改过的部分，是pristine上游代码）：
`launch/onboard_mighty.launch.py`里`use_hardware=false`（原生仿真分支）
那段代码明确标注**"=== EXISTING SIM CODE — COMPLETELY UNCHANGED ==="**，
用的是`fake_sim_node`（`fake_sim.cpp`，不经过Gazebo物理，直接对规划轨迹
做解析积分），Gazebo在这条原生流程里只负责渲染障碍物/点云，飞机的"状态"
根本不经过任何SLAM。**DLIO只出现在`use_hardware=true`这个硬件分支里**
（代码注释写的是"HW: Odom to state (DLIO remapping)"）——翻遍整个mighty
仓库，`use_onboard_localization`（DLIO开关）、`dlio/odom_node/odom`这些
字样只出现在硬件相关代码路径和真机地面车脚本里，从没在任何一个Gazebo
仿真模式里出现过。

这套docker_sim为了端到端验证真实机载软件栈，故意让mighty在Gazebo里走
`use_hardware=true`分支（之前`ros2 param get`确认过运行时确实是true），
这是mighty原作者从没测试过的组合——DLIO本身也是拿真实Livox+真实IMU调出
来的参数。

**顺着这个思路查DLIO自己的配置文件**：`git log --follow`看`cfg/dlio.yaml`
和`cfg/params.yaml`的历史，这两个文件从来没被这个项目自己的开发者改过
（`dlio_extrinsics_tilt.patch`是这次会话唯一碰过`dlio.yaml`的地方，只改了
外参，历史上其余全是DLIO上游自己的commit——加ROS2支持、加Hesai传感器
支持这些）。也就是说GICP配准阈值、关键帧插入阈值、体素/地图分辨率、
accel/gyro bias上限——全部还是DLIO官方示例（大尺度Ouster室外/校园场景）
的原始默认值，从没针对这套20×20米小房间+Mid-360（20米量程、单帧仅
1000点、飞机静止不动）重新调过。

**这次做了两处改动，都偏诊断/保守，不是"确信能解决"的根因修复**：

1. `patches/dlio_gicp_fitness_debug.patch`——纯诊断，在仪表盘里加一行
   GICP收敛状态+fitness score（`NanoGICP`经`LsqRegistration`继承自
   `pcl::Registration`标准API，`hasConverged()`/`getFitnessScore()`本来
   就有，只是代码里算了从没打印过）。想直接看清"配准本身是不是在正常
   收敛"，得先有这个数据支撑，不能再靠"走了几千米"这种事后症状去反推。
   零风险，不改变任何行为。
2. `patches/dlio_bias_bounds_tighten.patch`——把`odom/geo/abias_max`从
   `5.0`（m/s²）收紧到`0.3`、`gbias_max`从`0.5`（rad/s）收紧到`0.05`。
   5.0 m/s²对任何真实MEMS IMU来说都是荒谬的量级（真实bias一般在
   0.01~0.3量级），这么松的上限相当于给在线bias估计一个几乎没有约束的
   容器，配准误差可以被它悄悄吸收、越攒越大，直到真的顶到5.0才在崩溃
   日志里现出原形。收紧上限本身不解决配准误差从哪来，但能让这类误差在
   到达灾难性量级之前就先被"卡住"、更早暴露出来，是诊断性质更强的改动。
   只影响`odom/geo/`这两个运行时钳位（geometric observer每次更新后的
   钳位），不影响启动时的一次性IMU标定（那段代码不读这两个参数，已确认
   `state.b.accel = accel_avg - grav_vec`不受这两个参数约束）。

只涉及`flight-stack`镜像，已通过完整patch链验证（DLIO链7个patch从
pristine依次apply成功）。

## 重新build运行后：自己新加的诊断patch本身有bug，把DLIO段错误干挂了——已修复

重新build+up之后两架飞机的`dlio_odom_node`都在启动几秒内就崩了，而且是
**新的崩溃方式**：`exit code -11`（SIGSEGV段错误），不是之前反复见到的
`exit code -6`（`NotEnoughMemoryException`）；崩溃前`Accel Bias`/
`Distance Traveled`看着都很正常（bias在0.001量级，走了0.0000米），说明
这次根本不是发散把它冲垮的，是别的东西直接把进程弄崩了。

**根因是自己这一轮新加的`dlio_gicp_fitness_debug.patch`本身有bug**：
`getNextPose()`里`this->gicp.align()`是每一帧都无条件调用的，但真正给
GICP设置target点云/kdtree的`registerInputTarget()`只在`submap_hasChanged`
为true时才调用——这个成员变量初始值就是`true`，第一次submap准备好之前，
target kdtree压根没设过。新加的诊断打印没判断这个前提，直接对着一个还没
`registerInputTarget()`过的`this->gicp`调`getFitnessScore()`（PCL
`Registration`基类API，内部要对target的kdtree做最近邻查找）——kdtree是空
的，直接段错误，比之前的"发散→NotEnoughMemoryException"这个老崩溃方式
更快触发，而且什么有用的诊断信息都没打印出来就死了，属于弄巧成拙。

**第一次修复不够彻底，重新build后依然段错误**：加了`submap_hasChanged`
判断之后重新build+up，两架飞机还是在启动几秒内就`exit code -11`崩了，
且日志里连一行"GICP"诊断输出都没打印出来过——用户反馈"这次连飘的点云都
没有了"，比之前崩得还快。说明`submap_hasChanged`这个前提判断本身没错，
但不是唯一的问题。

**真正原因：`debug()`跑在detached线程里，直接调用`this->gicp`的方法，
线程不安全**。`this->gicp`（`NanoGICP`实例）本身没有任何锁保护，主线程
的`getNextPose()`一直在持续对它做`registerInputTarget()`/`align()`；
`debug()`是每次点云回调结束时新起的一个detached线程，如果在这个线程里
直接调`this->gicp.getFitnessScore()`（内部要读target的kdtree/
correspondence状态），跟主线程当时正在写的同一份状态产生数据竞争——
`submap_hasChanged`判断只能保证"目前已经注册过一次target"，挡不住
"debug线程读的时候，主线程恰好正在改"这种时序竞态，实测确实会段错误。

**修复**：不在`debug()`线程里碰`this->gicp`了。改成在`callbackPointCloud`
所在的主线程里（跟已有的`gicp_hasConverged`赋值同一行紧挨着，`align()`
刚返回、还没进入下一次回调之前，这个时间点是安全的），把`getFitnessScore()`
的结果存进一个新的`std::atomic<double> gicp_fitness_score_`成员，
`debug()`线程只读这个原子值，不再直接调用`this->gicp`的任何方法。已更新
`patches/dlio_gicp_fitness_debug.patch`（这次同时改了`odom.h`加成员声明
和`odom.cc`两处逻辑）并通过完整patch链重新验证。

**用户重新build时这次连编译都没过**：`colcon build`报
`error: use of deleted function 'std::atomic<double>::atomic(const
std::atomic<double>&)'`——`to_string_with_precision(const T a_value, ...)`
是个模板函数、参数按值传递，直接把`std::atomic<double>`类型的
`gicp_fitness_score_`传进去时，模板参数`T`会被推导成`std::atomic<double>`
本身（不是它隐式转换后的`double`——模板实参推导先于隐式转换发生，直接用
实参的静态类型），按值传参需要拷贝构造这个`T`，而`std::atomic`的拷贝
构造函数是显式`deleted`的，编译器直接报错。这次是编译期报错，没有变成
运行时的段错误，反而更早、更明确地暴露出来了。**修复**：先把
`this->gicp_fitness_score_`赋给一个普通的局部`double`变量，再传给
`to_string_with_precision()`（三元表达式里的`this->gicp_hasConverged ?
"yes" : "NO"`没有这个问题，是因为三元表达式条件位置的类型转换走的是
"contextual conversion to bool"，这条路径本来就允许隐式类型转换，跟
模板实参推导是两套完全不同的机制）。已更新patch并通过完整patch链重新
验证。

**编译过了，但运行时又段错误了——这次是真正的根因：`getFitnessScore()`
对`NanoGICP`这个子类本质上就不安全，不是时序问题**。用户重新build运行
后`dlio_odom_node`还是`exit code -11`，而且这次连"Distance Traveled"这行
最基础的调试输出都没打印出来过，比前两次都更早崩溃。查了`getFitnessScore()`
是PCL `Registration`基类的方法，`NanoGICP`从来没重写过它——它内部要用
基类自己的`tree_`（`pcl::search::KdTree`）做最近邻查找。但`NanoGICP`
自己的`registerInputTarget()`/`computeTransformation()`走的是完全独立的
一套`nanoflann::KdTreeFLANN`（`target_kdtree_`成员），从来不会触碰/构建
基类那个`tree_`——**不管等多久、不管`submap_hasChanged`是不是已经翻了
false，基类`tree_`永远是空的**，调`getFitnessScore()`对`NanoGICP`这个
子类来说从一开始就是不安全的操作，前两轮"判断submap_hasChanged"、
"挪到主线程调用"这些修复方向都没抓住真正的根因——那两个问题（时序竞态、
target是否注册过）都是真实存在的次要问题，但都不是这次段错误的最终
原因。

**修复**：换成`LsqRegistration`（`NanoGICP`真正的父类）自己提供、真正
在用的`getFinalError()`——GICP优化器自己收敛时的代价函数值，纯粹读一个
已经算好的成员变量（`return final_error_;`，查了实现确认没有任何树/
指针访问），什么时候调用都安全，语义上也比通用的"最近邻平均距离"更
贴切——就是GICP自己在优化的那个目标函数值本身。已更新patch并通过完整
patch链重新验证。

## 这次真的跑起来了：GICP自己报告"收敛完美"，但轨迹照样跑飞——嫌疑转移到关键帧插入逻辑

用户重新build+up后，`dlio_odom_node`这次是`exit code -6`（回到熟悉的
`NotEnoughMemoryException`老崩溃方式，不再是我自己那三轮引入的段错误），
诊断patch终于正常打印出了数据：

```
Distance Traveled :: 1.2077 → 3.4327 → 5.1892 → 8.7225 → 12.6550 → 15.4350 米
Accel Bias        :: 陆续爬到 0.30000001（正好卡在这轮收紧的0.3上限）
GICP               :: converged: yes, fitness: 0.0000（几乎每一帧都这样）
```

**GICP全程报告"收敛成功、残差几乎为零"，跟"飞机原地不动却越飘越远"这个
表象完全对不上**——如果真是点云配准本身有问题（比如房间太空旷、特征不够
导致配准病态），GICP应该会表现出收敛困难/残差偏大，但它没有。这基本
排除了"GICP配准精度/房间几何退化"这个假说，把嫌疑转移到更上游的地方。

直接查`updateKeyframes()`的关键帧插入判据（`src/dlio/odom.cc`）：
`newKeyframe`完全基于`this->state.p`（**估计出来的**位置）去跟已有关键帧
比距离/角度，**没有任何跟"飞机是否真的物理移动过"的交叉核实**——如果
估计值本身已经在飘，飘过`keyframe_thresh_dist_`（1.0米）阈值照样会插入
新关键帧，把当前（其实是错的）位姿和点云焊进地图。后续扫描很可能对着
这个已经被污染的地图配准得"很好"（因为都是同一条漂移轨迹自洽推出来
的），这能完整解释"GICP显示完美收敛，但整条轨迹早就飘出去了"这个现象——
一种经典的"地图被自身漂移污染、又反过来让漂移看起来自洽合理"的正反馈
循环。

**新增诊断**：`patches/dlio_keyframe_insertion_debug.patch`——在关键帧
真正插入的那一刻打印触发它的`dd`（距最近关键帧的距离）和当前估计位置，
直接在`callbackPointCloud`所在的主线程里同步打印，不经过`debug()`那个
detached线程、不碰GICP/PCL任何内部状态，只读`updateKeyframes()`内部
本来就已经算好的普通局部变量——前面三轮GICP fitness诊断折腾出的教训
（PCL基类方法对`NanoGICP`这个子类不安全、跨线程调用GICP对象有数据竞争）
这次都刻意避开了。只涉及`flight-stack`镜像，已通过完整patch链验证
（DLIO链现在8个patch），尚未重新build验证。

如果这次实测确认"关键帧确实在飞机静止时被不停新增"，就实锤了这个正反馈
循环，下一步大概率要去看`keyframe_thresh_dist_`是不是该配合这套仿真的
IMU噪声水平放宽、或者要不要给关键帧插入加一层"估计速度是否在合理范围内"
这类的sanity check。

## 实测确认：关键帧插入正反馈循环实锤——NX02静止状态下插了465个关键帧

用户重新build+up之后核实：**假说完全成立**。日志里刷出来一长串
`[updateKeyframes] new keyframe #N`：

```
NX01: #1 dd=5.04m estimated pos=(3.13, 3.95, -0.00) → ... → #12（还在涨）
NX02: #1 dd=3.82m estimated pos=(-2.77, -2.60, -0.37)
      → #10 estimated pos=(-7.36, -1.35, -22.74)
      → ...
      → #465 estimated pos=(-23.92, 42.30, -99.13)
```

飞机全程静止在地面，NX02这边却插了465个关键帧，估计位置从起点一路"飘"到
`(-23.92, 42.30, -99.13)`——每一次估计值刚飘过阈值就立刻触发一次新关键帧，
把当前（其实完全错误）的位姿和点云焊进地图，后续的扫描接着对着这个
越来越离谱的地图继续"配准得很好"（这也是为什么GICP fitness全程接近0——
它对的是自己刚刚参与污染出来的那份地图，不是客观真值）。这条正反馈
循环现在是有完整实测数据支撑的确凿机制了，不再是推测。

顺带发现一个可能相关的细节：日志里触发关键帧插入的阈值显示的是
`thresh=5.00m`，不是`cfg/params.yaml`里配置的`odom/keyframe/threshD: 1.0`——
`dlio.yaml`里`adaptive: true`打开了DLIO自己的自适应参数机制
（`setAdaptiveParams()`/`computeSpaciousness()`），会根据检测到的"空间
开阔程度"动态放大关键帧插入阈值。这套仿真只有一个20米见方的小房间，
DLIO却把阈值调到了默认值的5倍，说明它的"空间开阔度"判断本身可能也跟这个
场景的真实尺度不太匹配——这可能是另一层值得后续排查的问题，本轮没有
深入。

## 用户要求：修复关键帧机制

`patches/dlio_keyframe_velocity_check.patch`——给`updateKeyframes()`的
关键帧插入加一层物理合理性检查：新关键帧触发时，用这次的位移`dd`除以
距最近关键帧的时间差算出"隐含速度"，超过`keyframe_max_velocity_`
（新增参数`odom/keyframe/maxVelocity`，默认`5.0`m/s——这套仿真是20米
见方小房间里的四旋翼，正常飞行不可能达到，参考此前日志里
"dd=6.99m"级别、几乎发生在同一时刻的跳变，几乎肯定不是真实物理运动）
就拒绝这次插入，只打印警告，地图保持在最后一个"合理"的关键帧状态、
不再继续被污染。第一个关键帧无条件接受（没有"上一个"可比较）。

**要如实说清楚这不是什么**：这不是"确认解决发散"的根因修复——估计值
本身为什么会开始飘（IMU积分误差？GICP在这个房间里的弱约束方向？两者
叠加？）还没有查清楚。这是一个直接针对已经用实测数据确认过的机制
（关键帧插入对估计值盲目信任、把错误焊进地图、形成自我强化循环）的
定向circuit breaker——地图不再被污染之后，接下来要看GICP会不会真的
把估计值拉回来，还是estimate继续自己飘、只是不再连累地图（那样的话
"Distance Traveled"可能还是会涨，但关键帧数量应该会趋于稳定，不再
无限增长）。

只涉及`flight-stack`镜像，已通过完整patch链验证（DLIO链现在9个
patch），尚未重新build验证。

## 用户要求：查IMU积分本身+查DLIO是否官方clone、有哪些已知issue——挖到了目前为止证据最充分的根因

先确认：`staging/dlio_ws_src/direct_lidar_inertial_odometry`确实是从
`https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git`官方
clone的，但用的是`feature/ros2`这个**社区贡献的ROS2移植分支**，不是
用户口中"ROS1版本很好用"的`master`分支——查证：`feature/ros2`比`master`
落后23个commit，有个还没合并的PR #100明确写着想把master后续的bug修复
同步过来。

**顺着"ROS2移植可能有独有bug"这条线，查到两个跟本项目症状高度吻合的
官方issue**：
- **#78"ros2 IMU calibration"**（Ouster）：Z轴accel bias锁死在-5导致
  严重漂移，作者原话"there is no calibration step when compared to the
  main branch"。
- **#71**（Livox Mid-360）：同样accel bias锁死在-5附近。

两个都是"静止/低速状态下Z轴/整体发散、accel bias顶到硬边界"——跟这个
项目从最开始"走了两千多米"到上一轮"六万八千米"这整条主线症状完全对得上。
修这两个issue的PR #79只改了`cfg/dlio.yaml`的外参数值（跟本项目自己的
`dlio_extrinsics_tilt.patch`做的事类似），**完全没碰源码**——issue描述
的"缺少标定步骤"这层代码逻辑问题从来没被真正修过。

**顺着这条线挖到了issue #92"Set prev_imu_stamp on IMU calibration"，
直接对上了`src/dlio/odom.cc`里能独立复现验证的一个真实bug**：

`this->prev_imu_stamp`在构造函数里初始化成`0.`，标定阶段（`imu_calibrated`
还是`false`的那个if分支）整个过程从来没碰过它——`calibrate_accel_`/
`calibrate_gyro_`只在标定结束那一刻跑一次，标定期间的每条IMU消息走的是
外层"没标定"分支，压根不会执行到下面"已标定"分支里
`double dt = imu_stamp_secs - this->prev_imu_stamp;`这一行。等标定真正
结束、第一条"已标定"消息进来才第一次执行到这行，`prev_imu_stamp`依然是
那个从没被更新过的`0.`，`imu_stamp_secs`此时已经是十亿量级的epoch绝对
时间戳（这套仿真里是固定偏移`1735689600`+几秒，真机上是墙钟epoch，同样
是十亿级）——**算出来的`dt`是一个天文数字**，直接喂进`propagateState()`
的`0.5*dt*dt*(...)`这一项，一步就能把position/velocity冲到无穷大或者
巨大数值。

**这一个bug能完整解释这整条调查线上此前几轮"看起来有道理但实测无效"的
修复为什么都没用**：
- 解释了为什么"accel bias秒杀式顶到`abias_max`上限"贯穿了从第一次发现
  发散到现在的每一轮排查——都是这个天文数字级`dt`一步冲出去的直接后果。
- 解释了为什么之前测的"墙钟"（3.21节前）和"仿真时钟"（3.25/3.26节）两种
  时间戳方案**都同样发散**：不管哪种，两者都是epoch量级的绝对值，减去
  `0`都是天文数字，这个bug对两种clock策略都成立，此前那整轮"换成sim
  time"的排查完全没碰到这个真正的病灶。
- 也部分解释了后续观测到的"逐渐发散"（`Distance Traveled`几十秒内爬升）
  ——很可能是标定一结束这一次天文数字级`dt`造成单次灾难性冲击后，
  geometric observer（`updateState()`，增益`dt*Kp_`很小）在缓慢往回拉
  这一次巨大误差的过程，不是持续性的积分误差累积。

**修复**：`patches/dlio_prev_imu_stamp_fix.patch`——标定完成的那一刻把
`prev_imu_stamp`显式设成当前时间戳，让"已标定"分支第一次算`dt`时用的是
正确的、真正的采样间隔，不是从`0`开始算的天文数字。跟DLIO官方仓库
issue #92的社区诊断完全一致。

顺带加了一个纯诊断`patches/dlio_propagate_debug.patch`——在
`propagateState()`（纯IMU死推算，不经GICP/geometric observer矫正）里
打印`world_accel`/净加速度/速度/位置，直接在`callbackImu()`所在的单一
线程内、`geo.mtx`这把已有锁保护范围内打印，用来后续直接观察"纯IMU积分"
这条链路本身是否健康。

只涉及`flight-stack`镜像，已通过完整patch链验证（DLIO链现在11个
patch），尚未重新build验证。这是8轮排查以来第一个**同时有独立GitHub
issue佐证、有本地源码直接确认、且有清晰因果机制**的候选根因，但鉴于
此前太多轮"确信有效"的修复最后被验证无效，结论依然要以重新build之后的
实测发散情况为准。

## 用户要求：系统比对DLIO的master(ROS1)和feature/ros2分支IMU处理差异 + 评估换成FAST_LIO_ROS2的工作量

### `fix-imu-init`分支：确认就是上一轮那个bug的官方修复，独立验证

用户直接问`fix-imu-init`这个分支是不是已经修了上一轮发现的`prev_imu_stamp`
bug——本地`git clone`已经拉了全部remote分支，直接查：

```
commit 4816c3c "Set prev_imu_stamp on IMU calibration"（作者：DLIO官方
维护者/贡献者Nathan Chan，2025-06-04）
+ this->prev_imu_stamp = imu->header.stamp.toSec();
```

加在`this->imu_calibrated = true;`那一行的正下方——跟我们独立推导、已经
实现的修复**逻辑完全一致**（只是ROS1/ROS2 API调用方式不同）。用
`git merge-base --is-ancestor`确认：这个commit**既不在`master`里，也不
在`feature/ros2`里**——是一个从2025年6月就挂在那儿、一直没被合并进任何
主线分支的独立修复。这不光印证了我们自己找到的根因是对的，也说明这个
bug连DLIO官方自己都已经内部确认过，只是社区贡献的修复还没走完合并流程。

### DLIO master vs feature/ros2 IMU代码系统比对：又发现两处真实差异

派了个agent直接在本地repo上跑`git diff master feature/ros2`逐行比对IMU
相关代码，核心积分数学（`propagateState`/`updateState`/`integrateImu`）
确认**跟master逐字节一致**，没有额外差异——但发现两处配置/其它代码上的
真实差异，都已本地`git diff`逐字核实：

1. **`cfg/params.yaml`的`odom/imu/approximateGravity`**：`feature/ros2`
   是`false`，`master`是`true`。两个分支在共同祖先"Release v1.0.0"时
   都是`false`，master后来改成`true`但`feature/ros2`那时候已经分叉、
   从没跟上这次默认值变化。这个参数控制标定阶段要不要根据实测重力方向
   重新估计初始姿态（`state.q`）——为`false`时姿态直接保持默认值不做
   这一步修正，如果`base_link`没有恰好完全水平，这个跳过的修正会在
   重力扣除计算里引入系统性残差。**已改回`true`**，对齐master的
   （更新过的）默认值。

2. **`computeSpaciousness()`里一处越界读**：`for (int i = 0; i <= 
   this->original_scan->points.size(); i++)`多读了一个不存在的元素
   （未定义行为）。master在commit c389288
   （"Fix segmentation fault caused by out-of-bounds array access"）里
   已经改回`<`，但`feature/ros2`分支落后master 23个commit、从来没同步
   到这个修复。这个函数算出来的`spaciousness`直接喂给`setAdaptiveParams()`
   ——就是之前发现"关键帧插入阈值被自适应放大到默认值5倍"那个机制的
   输入，越界读到的垃圾值有可能是那次异常放大的一部分原因。**已修复**。

顺带核实了一处**不是ROS2移植独有、master和feature/ros2都有**的已知
upstream bug：Livox IMU读数未按g单位换算（`feature/livox-support`分支
有未合并的修复）——这套仿真里mid360 IMU插件直接输出SI单位m/s²（之前
实测静止时线加速度模长约9.8，不是约1.0），不受这个问题影响，本轮未
处理，留给以后接真实Livox硬件时再核对。

新增`patches/dlio_gravity_align_default.patch`（`cfg/params.yaml`一行）+
`patches/dlio_spaciousness_oob_fix.patch`（`odom.cc`越界修复），只涉及
`flight-stack`镜像，已通过完整patch链验证（DLIO链现在13个patch），
尚未重新build验证。

### 评估换成FAST_LIO_ROS2（`Ericsii/FAST_LIO_ROS2`）：建议先原型验证，不建议直接全面替换

另派了个agent专门调研这个仓库，结论：

- **项目本身**：香港科大FAST-LIO2的真ROS2移植（不是简单wrapper），
  持续维护中（最近commit 2025-11-17），支持Humble，仓库自带现成的
  `config/mid360.yaml`（比DLIO更省一步适配工作），依赖更轻（不需要
  Sophus、不需要Livox-SDK2，本项目已vendor的`livox_ros_driver2`可
  直接复用）。
- **架构性短板（相对这个项目最看重的双机命名空间隔离而言）**：直接
  读源码确认——发布话题是硬编码的绝对话题名（`/Odometry`、`/path`等），
  TF的`frame_id`（`"camera_init"`/`"body"`）**写死在代码里**，不是
  参数。ROS2的自动命名空间机制只对相对话题名生效，对绝对话题名和
  消息体内写死的frame_id字符串完全无效——两个实例同时跑会互相覆盖
  发布同名的`/tf: camera_init -> body`，直接冲突。这不是配置问题，
  需要改源码（给frame_id/节点名加参数化前缀）才能解决。相比之下，
  DLIO在这一点上明显做得更成熟（不然也不可能双机同跑到现在）。
- **一个巧合的额外发现**：agent通读`src/IMU_Processing.hpp`时发现
  `last_lidar_end_time_`这个成员变量**声明时未初始化**、构造函数和
  `Reset()`都没给它赋初值，标定完成后第一次调用的函数里就会读到它——
  这是跟DLIO刚修的`prev_imu_stamp`**同一类缺陷模式**（标定完成瞬间读
  到未初始化的"上一次"时间戳，可能产生异常dt）。没有对应的已提交issue，
  是agent自己走读代码的推测性发现，不是已确认问题，但值得在迁移前
  重点复核。
- **结论**：值得先做小范围原型验证（单机、单命名空间，跑通稳定性和
  倾斜外参精度），不建议现在就直接全面替换双机部署——命名空间这个
  架构性短板需要动源码才能解决，加上那处未初始化变量的潜在同源风险，
  贸然全面切换的收益不确定，成本不低。

## 用户追问NX02现象的真正原因：不是IMU积分，是GICP优化器一个"静默伪成功"的退化bug——目前为止挖到最深的一层

用户直接问："出现NX02这个现象的原因究竟是什么？就是IMU积分吗？还是IMU
本身就有问题，现在已经不是DLIO的问题了？换飞机本体的IMU呢？"

顺着GICP fitness全程可疑地卡在`0.0000`这条线，直接读了`NanoGICP`/
`LsqRegistration`（`nano_gicp.cc`、`lsq_registration.cc`）的优化器源码，
找到了**目前为止最深的一层根因**：

**不是IMU积分本身的问题，是GICP优化器对"完全没有对应点"这个退化情况
没有任何防护**。机制：

1. 一旦`T_prior`（IMU推算的初始猜测）偏得太远，超出
   `maxCorrespondenceDistance`（0.5米），`target_kdtree_`里所有点的
   最近邻搜索全部失败（`correspondences_`全是`-1`）。
2. `NanoGICP::linearize()`算出的`sum_errors`/`H`/`b`全部**精确为0**。
3. 喂进`LsqRegistration::step_lm()`的`rho=(y0-yi)/(d·(lambda*d-b))`
   变成`0/0=NaN`。
4. C++里`"rho < 0"`这个判断**对NaN永远是false**，代码因此走进
   "success"分支：`x0`基本不变（等于原样返回了没修正的`T_prior`）、
   `final_error_`被设成精确的`0.0`（仪表盘上看起来像"配准完美"）、
   `is_converged()`对着一个几乎是单位阵的`delta`自然也判定为`true`。
5. **GICP从这一刻起悄悄变成纯直通IMU先验的空转状态，却一直报"收敛、
   误差0"，完全看不出已经失效**。
6. 后续`updateState()`拿GICP这份（其实等于什么都没修正的）输出去跟
   同样靠IMU积分出来的`state.p`比，两者本来就接近一致，误差信号也跟着
   失真变小，整个几何观测器因此也停止有效矫正，位姿彻底自由发散、只受
   纯IMU积分本身残留的微小偏置驱动，表现为近似匀速的持续漂移——跟NX02
   实测最终稳定在约1800~4100 m/s量级的"匀速"发散、直到崩溃时走了64万米
   完全吻合。

**直接回答用户的问题**：
- `propagateState`/`updateState`/`integrateImu`三处核心积分数学本身跟
  DLIO官方master分支逐字节一致（上一轮agent已核实），**不是积分公式
  写错了**。
- 也不是"IMU本身数据有问题"——用的还是同一个mid360仿真IMU，前几轮已经
  核实过读数在物理上是自洽的。
- 是**GICP这一层在失去对应点之后没有诚实地报告失败，反而伪装成功**，
  导致上游依赖"GICP还在工作"这个假设的整条矫正链路失效。
- **换飞机本体IMU换不掉这个bug**：不管用哪个IMU源，只要某一次积分误差
  恰好让先验漂出0.5米对应点搜索半径，同样的NaN伪成功链路照样会触发，
  区别只是"多久之后触发"，不是"会不会触发"——这也解释了为什么NX01这次
  表现明显更好（前面几轮修复确实让它撑得更久才触发这个退化状态）、但
  NX02还是触发了：本质上是同一个bug，只是两架飞机因为IMU噪声的随机性
  不同、触发时机不同。

**修复**：`patches/dlio_gicp_lost_track_fallback.patch`——在
`getNextPose()`里用`this->gicp.num_correspondences`（NanoGICP这次实际
配对上的点数，真正配准成功时是几百，退化到空转时是0或极少）识破这种
伪装：低于`gicp_min_num_points_`（已有参数，配置里是64，之前一直没被
用来做这个用途）时，故意不更新`this->T_corr`/`this->T`，保留上一次
真正配准成功时的值，让`propagateGICP()`把`lidarPose`钉在最后一个可信
锚点上，`updateState()`的`err = pin - state.p`才能继续反映"当前积分
状态相对最后一次可信观测究竟飘了多远"这个有意义的量，而不是对着一个
自己也在跟着飘的"影子目标"算出虚假的小误差。

**这是这一整轮IMU/SLAM排查里目前为止改动最深、涉及信任判断逻辑本身的
一次修复**（不只是诊断或参数调整），风险和潜在收益都比之前几轮更大，
只涉及`flight-stack`镜像，已通过完整patch链验证（DLIO链现在14个
patch），尚未重新build验证。

## 用户追问："现实中这个封闭场景很好定位，为什么仿真定不了位？是不是点数太少？"——实测数字确认，改成20000点

直接查了实测数字：Gazebo仿真雷达每帧原始点数**精确1000个**
（`gen_iris_mid360_sdf.py`里`<samples>1000</samples>`，`downsample=1`
不做二次抽稀），DLIO自己预处理（CropBox+体素降采样）之后实际参与配准的
点数只有**240~730个**（平均约450）——而且这一轮`keyframes: 1`全程没变
过，确认上一轮的关键帧速度检查确实生效了、地图没有再被污染，问题纯粹
是这几百个点配不上。

真实Mid-360规格是**每秒约20万点**，哪怕按真实驱动常见的10Hz出帧频率
折算，一帧也有约2万点——是仿真里实际参与配准点数的**40倍以上**，比
DLIO官方自己的调参参考对象（Ouster OS0/OS1这类，单帧几千到几万点）也
稀薄了一个数量级都不止。点数密度直接决定对应点搜索的"容错空间"：真实
密度下房间里到处都密密麻麻是点，先验偏一点也大概率能碰上对应点；仿真
里450个点撒在20×20米房间，点间距本身可能就有大半米，先验稍微一偏，
变换后的源点云就容易整体落进"没有任何点"的空隙——跟实测"0个对应点"
直接对得上。

**改动**：`gen_iris_mid360_sdf.py`（一手代码，不走patch）把plugin级别
的`<samples>`从`1000`改成`20000`，对齐真实Mid-360规格。`<horizontal>`/
`<vertical>`那两处`<samples>`（100/360）是雷达传感器SDF schema要求的
底层光线投射网格声明，不是实际决定输出点数的地方——`livox_points_plugin.cpp`
里`LivoxPointsPlugin::Load()`是直接按plugin级别`<samples>`那个数字，
从`mid360.csv`（80万行非重复扫描角度表，够用不会撞车）逐条精确取
方位角/俯仰角调用`rayShape->AddRay()`真实新增光线，不受那两个粗网格
参数限制，所以没有一并改动。

**代价要如实说清楚**：这些光线对象是在插件`Load()`时一次性创建、每个
物理/传感器更新周期（约100Hz）都要重新光线投射一次的，不是一次性开销
——点数从1000提到20000，相当于每架飞机每个周期的光线投射计算量涨了
20倍，双机同跑，这套仿真这一整轮反复遇到的CPU负载问题（`gz stats`
实测real-time-factor原本就只有0.96~0.97）大概率会因此更紧张、跑起来
可能会更卡/更慢，但不会重新引入之前那类"real-time-factor漂移导致dt
错误"的bug（IMU/点云时间戳已经改成用仿真时钟本身，不受墙钟/实际速度
影响）。

只涉及`sim-world`镜像，容器启动时（不是build时）重新跑这个脚本生成
SDF，所以`sim-world`需要重新build。

## 用户要求：20000点密度修复稳定之后，现在就试解锁起飞，高度1.5米

查了这套栈的起飞触发机制：`dynus_mavros.launch.py`启动的
`track_dynus_traj_py`节点（`track_dynus_traj.py`）本来就在自动跑
`takeoff_and_track_trajectory()`，目标高度原来硬编码`1.0`米；自动解锁/
切OFFBOARD逻辑在`ros2_px4_stack_dynus.patch`里、`base_mavros_interface.py`
里已经有了（"含三处已知修复"之一），**整套流程本来就是全自动的**，
不需要额外手动触发arm/offboard服务调用。

之前一直没真正飞起来，是因为PX4 EKF2要求vision数据合理才会真正允许
解锁/保持OFFBOARD，DLIO之前一直在发散，喂给它的位姿本身就是垃圾——现在
20000点密度修复验证稳定（连续9分钟`Distance Traveled`精确0.0000）之后，
应该有机会真的起飞了。

**改动**：`patches/ros2_px4_stack_takeoff_altitude.patch`——把
`track_dynus_traj.py`里`main()`调用`track_trajectory()`的目标高度从
`1.0`改成`1.5`米，配合这个已有的全自动流程使用，不涉及arm/offboard
逻辑本身（那部分已经有了，不用动）。只涉及`flight-stack`镜像，已通过
完整patch链验证。

## 用户反馈"起飞程序没有被执行"——查到真正原因：自动解锁的线程压根没启动过

重新build+up之后飞机没有反应。查了`mavros/state`：`connected: true`，
但**`armed: false`、`mode: AUTO.LOITER`**——飞机就那么一直停在地上。

顺着这条线查`ros2_px4_stack_dynus.patch`里加的`_kick_offboard()`（自动
解锁+切OFFBOARD那个方法）：**这个方法从头到尾只写了方法体，从来没有
被实际调用过**——`__init__()`里只给`_publish_setpoint()`（发setpoint
那个线程）起了线程，`_kick_offboard`没有对应的`Thread(...)`/`.start()`。
起飞状态机（`takeoff_and_track_trajectory`）确实在正常发setpoint（日志
里能看到`track_dynus_traj_py`节点活着、在正常处理连接状态变化），但
PX4从来没被真正解锁/切到OFFBOARD——不是数据/配置问题，是这个patch本身
写漏了最后一步"真正启动这个线程"，纯粹的死代码。

**修复**：`patches/ros2_px4_stack_kick_offboard_fix.patch`——补上跟
`_publish_setpoint`同样写法的`Thread(target=self._kick_offboard, ...)`
并`.start()`。只涉及`flight-stack`镜像，已通过完整patch链验证（4个
ros2_px4_stack patch都能从pristine依次apply成功），尚未重新build验证。

## 用户反馈"二号机没起来"——20000点改动带来的RTF下降，让固定10秒的解锁重试预算变得太紧

实测`mavros/state`：NX01最终`armed: true`、`mode: OFFBOARD`（成功了），
NX02停在`armed: false`、`mode: OFFBOARD`，过了5分钟重新查依然没变化。
两架飞机日志里"Sensor Rates: Livox"这时都只有约**22Hz**（20000点密度
改动之前是~90-100Hz），`gz stats`实测real-time-factor只有**0.27~0.30**
——而且两架飞机的传感器速率几乎完全一样，**不是NX02比NX01更卡**。

真正原因：`_kick_offboard()`原来是`range(20)`、每次`sleep(0.5)`秒，
总共**10秒真实时间**的重试预算——这个预算是按real-time-factor≈1设计的。
PX4的解锁/切模式安全检查是跟着Gazebo仿真时钟lockstep走的，10秒真实时间
在RTF=0.3下只对应不到3秒仿真时间；两架飞机谁能在这个被严重压缩的窗口
内蹭过去纯粹看运气——这次NX01蹭过去了，NX02没蹭过去。而且这个函数一旦
超时就直接放弃、打印一行最终状态、线程直接退出，**不会自己再重试**，
所以NX02才会卡死不动。

**修复**：`patches/ros2_px4_stack_kick_offboard_timeout.patch`——把重试
预算从`range(20)`（10秒）放宽到`range(240)`（120秒），留出实测RTF这个
量级下足够的仿真时间余量。只涉及`flight-stack`镜像，已通过完整patch链
验证（5个ros2_px4_stack patch都能从pristine依次apply成功），尚未重新
build验证。

## 用户提问："现在板外控制器的控制律是什么？"

查了`dynus_offboard_node.py`，控制律分两个阶段，**用的是微分平坦
（differential flatness）方法**，不是简单的PID位置环：

- **TAKEOFF阶段**：发布位置setpoint（`MultiDOFJointTrajectory`）给PX4
  自己的位置控制器，让PX4内部的位置/速度环把飞机拉到起飞高度——这一段
  完全信任PX4自己的控制器，机载这边不接管。
- **TRAJECTORY阶段**（收到DYNUS规划器的`Goal`之后）：`_pack_into_attitude()`
  直接**跳过PX4的位置/速度控制器**，机载这边自己用微分平坦公式把
  规划器给的位置/速度/加速度/偏航（`p, v, a, yaw`）轨迹点转换成
  **姿态四元数 + 机体角速度 + 归一化推力**，通过`mavros/setpoint_raw/attitude`
  直接发给PX4的姿态/角速度控制器（PX4这一层只做最内环的姿态跟踪，不再
  自己算"要往哪飞"）。

具体计算（`get_orientation`/`get_angular`/`get_thrust`三个函数）：
- **推力**：`thrust = |m*(a + g·z_W)| / (m*g)`，质量`m=2.906`kg、
  `g=9.81`，归一化到PX4期望的`[0,1]`区间（悬停对应约50%油门，可调）。
  加速度分量小于`0.1`m/s²时会被clamp到0，避免悬停/纯偏航时里程计噪声
  引起的微小加速度被误当成真实机动指令、导致姿态抖动。
- **姿态**：从期望合力方向反解出机体`z`轴（`z_B`），配合偏航参考构造
  出完整旋转矩阵，再转成四元数——这是差分平坦四旋翼控制的标准做法
  （期望姿态由"需要多大的合力才能实现期望加速度"反推出来，而不是先
  给一个独立的姿态目标）。
- **机体角速度**：同样通过微分平坦公式，从加加速度（jerk）和偏航角
  速度解析计算得到`p, q, r`——纯偏航（零加速度）时退化成`p=0, q=0,
  r=dyaw`。

简单说：这是一个**开环微分平坦前馈控制器**（不是闭环反馈PID），完全
信任规划器给出的轨迹在动力学上是可行的，直接算出"要达到这个轨迹点
理论上需要的姿态和推力"发给PX4，PX4这层只负责姿态/角速度这个最内环
的闭环跟踪，不做位置闭环。

## 用户反馈"飞机起来了，但用RViz的2D Goal Pose给点，飞机不动"——找到DLIO的debug日志，确认根因

先查`mighty_node.cpp`的debug日志（`/tmp/mighty_debug.log`，
`debug_log_file`参数指定的路径）确认RViz发的`term_goal`**确实收到了
两次**：

```
0.0000  TERM_GOAL  4.1270 -3.3065 0.0000  from_user=1 frame=NX01/odom
29.1619 TERM_GOAL -3.7283 -2.6457 0.0000  from_user=1 frame=NX01/odom
```

话题连通性、QoS都没问题（`ros2 topic info -v /NX01/term_goal`确认
1发布者2订阅者，都是`RELIABLE`），**不是连接问题**。

**真正原因**：RViz的"2D Goal Pose"工具只能设x/y，z恒为`0`；
`mighty_node.cpp`里本来就有一段专门处理这个的逻辑——`force_goal_z`为
`true`时用`default_goal_z`覆盖掉RViz传来的`z=0`，这是mighty原作者早就
考虑过的场景（配置文件里的注释原话："Rviz GUI's goal's z is 0"）。但
这套docker_sim用的`hw_mighty.yaml`（`use_hardware=true`分支加载这份
配置）里`force_goal_z`是`false`——横向比较了mighty仓库其余四份配置
文件（`mighty.yaml`、`mighty_ground_robot.yaml`、
`hw_mighty_ground_robot.yaml`、`multi_mighty.yaml`），**全部都是
`force_goal_z: true`，只有`hw_mighty.yaml`这一份是`false`**，大概率
是个遗漏，不是故意的。

原始`z=0.0`低于`z_min`（实测`0.8`），被`mighty_node.cpp`自己的边界
检查直接拒绝，容器日志里能看到两条对应的报错：
```
[ERROR] [NX01.mighty_node]: Goal z is out of bounds: 0.000000
[ERROR] [NX01.mighty_node]: Goal z is out of bounds: 0.000000
```
——跟RViz点了两次的debug日志记录完全对得上。

**修复**：`patches/mighty_force_goal_z.patch`——`hw_mighty.yaml`里
`force_goal_z`改成`true`（对齐其余四份配置文件的既有做法），
`default_goal_z`从`1.0`改成`1.5`（配合这次起飞高度1.5米，2D Goal
Pose点的目标点会保持在跟起飞一样的高度，不用额外爬升/下降）。这次
只涉及`flight-stack`镜像，已通过完整patch链验证（flight-stack这条
mighty patch链现在10个patch），尚未重新build验证。

## 用户提问："双机系统2D Goal Pose给出后，究竟哪架飞机会飞那？"——只有NX01会响应，加了第二个工具

查了`multi_mighty.rviz`的Tools列表，**只有一个`SetGoal`工具实例**，
写死发到`/NX01/term_goal`：

```yaml
- Class: rviz_default_plugins/SetGoal
  Topic:
    Value: /NX01/term_goal
```

不是这次改动引入的问题，是原始配置模板本来就只配了一个——RViz的工具
按钮不会"自动感知当前选中哪架飞机"，每个按钮就是死死绑定一个话题。
**NX02原来根本没有对应的2D Goal Pose入口**，点了也没用。

**修复**：`patches/mighty_rviz_nx02_setgoal.patch`——加第二份`SetGoal`
工具实例，绑定`/NX02/term_goal`，两个都加了`Name`属性
（`2D Goal Pose (NX01)`/`2D Goal Pose (NX02)`）方便在工具面板上区分。
只涉及`sim-world`镜像，已通过完整patch链验证（sim-world这条mighty
patch链现在14个patch），尚未重新build验证。

## 用户要求：房间正中间加一根柱子

`patches/mighty_simple_room_world.patch`新增`column_6`（半径0.25米、
高5米，跟其余5根柱子规格一致），位置`(0, 0, 2.5)`——房间正中心，离两机
spawn点(3,0,3)/(6,0,3)分别还有3米/6米净空，不会挡到起飞。只涉及
`sim-world`镜像，已通过完整patch链验证，尚未重新build验证。

## 用户报告：SLAM调好之后新的三个问题——1)指点飞行不避障 2)规划轨迹跟实际轨迹平行but差1~3米、双机不同 3)有头模式朝向疑似不对

系统当时已经`docker compose up`跑起来，直接进容器做实时诊断（`ros2 topic
echo`/`docker logs`/发布测试`term_goal`），而不是只读代码——这轮комбинates
静态代码阅读和运行时实测，找到了一个能同时解释问题1和2的根因。

**先验证spawn/坐标基础事实**：`sim-world-entrypoint.sh:125`
`-x "$((i*3))" -y "0" -z "0.1"`——NX01 spawn世界坐标(3,0,0.1)，NX02
(6,0,0.1)，跟`/plug/model_states_plug`实测的真值完全对得上。
`flight-stack-entrypoint.sh:47-49`导出的`INIT_X=$((AGENT_INDEX*3))`/
`INIT_Y=0`/`INIT_Z=0.1`跟spawn坐标一致。

**根因：`hw_mighty.yaml`里`provide_goal_in_global_frame: true`打开了一套
"局部系→全局系"的坐标转换，但只有状态那一半接上了，目标点和障碍物点云那
两半没接上，三者的参考系互相对不上。**

链路细节（`mighty.cpp`/`mighty_node.cpp`，都是实读代码+容器内`docker
logs`/`ros2 topic echo`交叉验证过的）：

1. `ros2_px4_stack/launch/dynus_mavros.launch.py:38,44,50`用`INIT_X/Y/Z`
   发一条`world_mocap -> {veh}/init_pose`静态TF（平移=各自spawn偏移），
   同时发`world -> map`、`world -> world_mocap`两条恒等TF，把`map`和
   `world_mocap`焊成同一个点。`mighty_node.cpp:1896-1920`
   `getInitialPoseHwCallback()`每100ms查一次`lookupTransform("map",
   "{ns}/init_pose")`，成功一次后算出`init_pose_transform_`（本质就是
   "这架飞机的DLIO局部原点，相对世界系的偏移"）。容器日志`docker logs
   flight-stack-nx01`实测确认这条链路真的连通、`setInitialPose()`真的被
   调用过（`yaw_init_offset_: 0`两次打印，NX01/NX02各一次）。
2. `mighty.cpp:1497-1518``updateState()`：当`use_hardware &&
   provide_goal_in_global_frame`（`hw_mighty.yaml:18-19`实测容器内确认
   都是`true`——注意这是**编译进镜像里的`install`目录**的值，跟host上
   `staging/`目录里未打patch的pristine源码不是一回事，下面第4点会再提这个
   坑）为真时，把`state_`（内部使用的"当前位置"）从DLIO局部系
   通过`init_pose_transform_`转换成全局系（≈world系）。`mighty.cpp:
   1582-1600``getNextGoal()`在弹出控制指令前用`init_pose_transform_inv_`
   把内部全局系的下一个setpoint转换回局部系再发布——这一步本身自洽
   （正变换+逆变换严格互逆，单机自己给自己的目标不会因为这个机制本身产生
   误差）。
3. **但`terminalGoalCallbackImpl()`（`mighty_node.cpp:1757-1827`）接收
   `term_goal`时直接用`msg.pose.position`原始数值，没有做任何坐标转换**
   ——设计意图是"term_goal本来就应该是全局系"（对应真实硬件+Vicon场景，
   yaml注释里也写"true for hardware with Vicon"），但这套仿真的
   `term_goal`来自RViz的`2D Goal Pose`，RViz的`Fixed Frame`被此前的调试
   （见"Fixed Frame设错了"一节）固定设成了`NX01/odom`——**是局部系，不是
   全局系**。同理`occupancyMapCallback`/`mapCallback`/`unknownMapCallback`
   （`mighty_node.cpp:3173-3218`）拿到`global_mapper_ros`发布的
   `occupancy_grid`/`unknown_grid`点云后直接`pcl::fromROSMsg`塞给
   `updateMap()`，全程没有一处调用`init_pose_transform_`——障碍物点云
   全程停留在局部系。规划器内部用来判断"我在哪、地图窗口该往哪居中"的
   `computeMapSize(local_state.pos, ...)`（`mighty.cpp:1942,1993`）用的
   却是转换过的全局系`state_`。三方（"我在哪"=全局系、"目标在哪"=被误当
   成全局系的局部数值、"障碍物在哪"=局部系）参考系互相对不上，且偏移量
   正好等于各自飞机的spawn偏移（NX01≈3米，NX02≈6米），双机不同，跟
   用户报告的"1到3米、双机不同、像整体平移"完全吻合。
4. **踩过的一个坑，记录一下方法论**：一开始派agent读`staging/`目录里的
   `hw_mighty.yaml`，发现`force_goal_z: false`，得出"3.28节那个patch没
   生效"的结论——**这个结论是错的**。`staging/`是`fetch_sources.sh`
   clone下来的pristine源码，patch是在`docker build`时应用、装进镜像
   `/opt/mighty_ws/install/...`里的，host上的`staging/`不会跟着变。直接
   `docker exec`进`flight-stack-nx01`容器读`/opt/mighty_ws/install/
   mighty/share/mighty/config/hw_mighty.yaml`才是真正在跑的配置——实测
   `force_goal_z: true`、`default_goal_z: 1.5`（patch确实生效了），但
   `provide_goal_in_global_frame: true`、`z_min: 0.8`、`z_max: 2.0`
   （这两个从来没被任何patch碰过，是原始值）。**以后排查"某个改动是否
   生效"，必须进容器读`install/`目录里的实际文件，不能只读host上的
   `staging/`。**

**实测复现（问题2的直接证据）**：读`/tmp/mighty_debug.log`（
`mighty_node.cpp`自带的调试日志，记录每一条收发的`goal`/`term_goal`）
实测到这样一行：`state=-9.04,-1.39,1.48 ... G=-9.1149,-1.0913,1.5000
Gterm=-9.1149,-1.0913,1.5000`，同时发布出去给控制器的`GOAL`却是
`-12.15,-1.35,1.38`——`state`（全局系，接近`term_goal`原始数值，说明
飞机已经"到"了）和`GOAL`（局部系输出）之间正好差3米（NX01的spawn偏移），
跟`/NX01/state`话题直接echo到的DLIO局部值（约-12.1）对得上——证实
"全局系内部状态"和"局部系ROS话题"两套数值同时存在、且系统性相差一个
spawn偏移量。

**实测复现（问题1的间接证据，及一次意外事故）**：先发了一个刻意贴近房间
中心柱子（真实世界坐标(0,0)，换算成NX01局部系约(-3,0)）的`term_goal
(-3,-1,1.5)`，被`sanitizeTerminalGoal`正确拒绝（"Goal ... is in occupied
space"）——说明"目标点是否被占据"这一处检查用的是局部系数值、跟局部系
障碍物点云是对齐的，没有被这次发现的frame问题影响到。接着发了一个会让
直线路径穿过同一根柱子的目标`(3,1,1.5)`（局部系），实测世界真值位置
（`/plug/model_states_plug`）飞行途中从z=1.73骤降到z≈0.05~0.18（贴地
高度），而同一时刻DLIO自己汇报的局部z还是1.30——**飞机在没有报错、没有
disarm的情况下（`mavros/state`显示`armed:true, mode:OFFBOARD,
system_status:4`）真实世界高度骤降到贴地**，跟"避障没生效、飞行中撞上/
蹭到障碍物"的表现吻合，但受限于时间没有进一步用世界真值轨迹逐帧核实是否
真的物理接触了柱子，**这一条只能算强烈提示，不算实锤**。飞机目前应该还
停在这个状态（局部系里没有报错，用户下次进入容器/看Gazebo GUI时会看到
NX01貼地），如有需要应该reset一下这架飞机的仿真状态再继续测试。

**建议的修复（尚未实施，需要用户确认后再动手）**：这套仿真没有Vicon/
真全局定位系统，每架飞机都只有自己的局部DLIO SLAM，`hw_mighty.yaml`里
`provide_goal_in_global_frame: true`这个"要跟Vicon硬件配合用"的开关本
不该在这套纯DLIO仿真里打开。最直接、风险最低的修复是新增一个patch把它
改成`false`——三方（state/term_goal/occupancy_grid）从此统一全部停留在
各自局部系，跟系统里其余所有环节（DLIO输出、`repub_odom.py`转发给EKF2
的`vision_pose`、`mavros/local_position`、RViz`Fixed Frame`）已经在用的
局部系保持一致，从根上消除这个偏移，问题1大概率也会随之明显改善（`state_`
和障碍物点云回到同一个参考系）。副作用需要留意：双机之间共享轨迹
（`/trajs`，供对方当动态障碍物）走的是`use_frame_alignment`那一套独立
机制（不是这个开关管的），理论上不受影响，但改完之后建议重点复核一遍双机
互相避让是否正常。另外`z_min: 0.8`/`z_max: 2.0`跟柱子实际跨越的高度
范围（0~5米）相比明显偏窄，建议放宽到覆盖实际飞行包线（比如0.3~5.0），
避免飞机稍微爬升/下降就触发`findAandAtime()`里的"A is out of the map"
把重规划直接判失败。

**问题3（有头模式朝向）**：`mighty.cpp`里`yaw = atan2(dir.y(), dir.x())`
和`dynus_offboard_node.py`的`get_orientation()`差分平坦公式逐行核对过，
没发现叉乘顺序颠倒/cos-sin写反/ENU-NED重复转换这类bug，两边约定自洽。
现场想再采样一段"飞机高速直线飞行"的真实速度方向vs姿态yaw做交叉验证，
但飞机在问题1/2的测试过程中意外贴地（见上），没能拿到干净的高速飞行样本，
这一条暂时只能给出"代码逻辑没找到bug"这个弱结论，不算最终验证——等
问题1/2的修复验证完、飞机能正常巡航之后，应该补一次这个实测。

顺带核实了一个跟这次问题3无关、但容易混淆的历史patch：
`patches/mighty_headless_yaw_fix.patch`（本session之前就有）修的是另一个
bug——`getNextGoal()`原来`if (par_.use_hardware)`就把`next_goal.yaw`/
`dyaw`清零，而这套仿真UAV也开了`use_hardware=true`（为了让DLIO真实里程计
接进来），结果误伤成"无人机整个飞行过程朝向锁死不转"（真正的"无头模式"）。
patch把判断条件改成`par_.use_hardware && par_.vehicle_type ==
"ground_robot"`，只对地面机器人清零yaw。这个patch解释了"为什么现在飞机
是有头模式"（能转向），但不影响这次用户问的"转向的角度对不对"——两个是
不同层面的问题，未来如果重新怀疑yaw有问题，别把这两个patch搞混。

## 用户提出集群架构设想：mighty规划器全用局部系，集群层面（目标分发/多机避让）统一用全局系，中间加一个坐标转换节点——评估结果：避让方向已经这么做了，目标分发方向只有骨架

针对上面`provide_goal_in_global_frame`那个bug，用户提出一个更完整的架构
思路：mighty内部规划永远留在各机自己的局部系（因为DLIO天然就是局部系），
但集群层面的两类交互——(a)统一给整个集群下发目标点、(b)多机互相共享轨迹
用于避让——应该经过一个专门的局部⇄全局坐标转换节点，全局系仿真里用
Gazebo真值、实飞用GPS/UWB。派agent实读代码评估这个思路距现状有多远，
结论：**(b)已经是这个架构、已经跑通，不用动；(a)只有骨架，需要新写**。

- **(b) 多机避让**：`uwb_sim/uwb_ground_truth_node.py`发布的`frame_align`
  话题语义是**两两之间的相对变换**（agent_j位置减agent_i位置，仿真里从
  `/plug/model_states_plug`真值算，未来接真UWB测距解算替换即可），不是
  "每机相对统一全局系的绝对位姿"。`mighty_node.cpp`的`frameAlignCallback`
  收到这个变换存进`frame_align_transforms_[agent_id]`，`trajCallback`
  收到别的飞机共享的`/trajs`时用`applyFrameAlignTransform()`把对方局部系
  数据变换到自己局部系，再当动态障碍物用于避让——全程没有出现真正的
  "全局系"，只是两两局部系互相对齐，本来就是用户设想的那个思路，而且是
  实测跑通的成品（3.9节记录过`frame_align`~58-61Hz），不需要为了这次的
  架构调整改动。
- **(a) 目标分发**：`mighty_node.cpp`订阅了`/swarm_goal`
  （`swarmGoalCallback`把常量`formation_self_offset[0/1/2]`加到收到的
  PoseStamped上再转给`terminalGoalCallbackImpl`），但**全项目grep不到
  任何`/swarm_goal`的发布者**，是个死话题；`formation_self_offset`也只是
  yaml里写死的编队队形常量偏移，不是真正基于"这架飞机局部系相对全局系
  在哪、差多少度"算出来的坐标变换——如果要支持`spawn`朝向不同或飞行中
  动态重新对齐，这个模型不够用。

**关键待决问题**（需要用户先确认再展开这部分设计）：未来真UWB部署是
**纯机间测距**（只能拿到两两相对位姓，"全局系"本质上仍然只能是"选一架
飞机的局部系当基准"，跟现有`frame_align`是同一回事，不需要额外做绝对
定位节点）还是**有固定测量过坐标的地面锚点**（能给出每机真正的绝对
位姓，才需要一个独立的"局部→绝对全局"转换节点）。这个决定了(a)方向
要新写的转换节点具体该怎么设计，本session未展开，留作后续独立任务；
新节点的职责边界已经明确：**只单向负责"全局目标→这架飞机的局部
`term_goal`"，直接发到现有`term_goal`话题上，绝对不能碰`state`和
`occupancy_grid`**（这正是`provide_goal_in_global_frame`这次翻车的
教训——它把`state_`也一起转成了全局系，污染了避障用的参考系）。

## 修复已落地：新增`patches/mighty_local_frame_only.patch`

只改`config/hw_mighty.yaml`两处：
1. `provide_goal_in_global_frame: true → false`——mighty内部
   state/term_goal/障碍物点云三方从此统一停留在各自局部系，跟
   `repub_odom.py`喂给EKF2的`vision_pose`、`mavros/local_position`、
   RViz`Fixed Frame`（`NX01/odom`）本来就在用的局部系保持一致。
2. `z_min: 0.8 → 0.3`、`z_max: 2.0 → 5.0`——原窗口比柱子实际跨越的高度
   范围（0~5米）明显偏窄，容易在飞机稍微爬升/下降时触发
   `findAandAtime()`里的"A is out of the map"、导致重规划直接判失败。

已在host用git diff生成、`git apply --check`验证能对pristine
`staging/mighty_ws_src/mighty`干净应用，`Dockerfile.flight-stack`里
接到`mighty_headless_yaw_fix.patch`之后的patch链末尾。**尚未
rebuild+up验证**——按本文档一贯的规矩，这里如实记录：这是目前证据链
最完整的一次修复（有实测debug log数值、容器内`install/`目录读到的
运行时配置、代码路径逐行追踪三重印证），但还是要等rebuild之后问题1/2
是否真的改善、有没有引入新问题（尤其是双机互相避让，理论上不受这个
开关影响，但要实测确认）来定论，不能提前当成已解决。

## rebuild验证结果：问题2（轨迹固定偏移）确认修好，问题1（避障）改善但仍会撞——挖到更深一层：机头转向跟不上平移速度

用户rebuild+up之后反馈：**问题2彻底解决**（"轨迹不再偏离"）。问题1
（避障）**有实质改善但没根治**——之前完全不避障，现在确认规划器真的在
绕障（`REPLAN ok=1 hgp=1`、实测轨迹在接近柱子时确实往侧向偏了
0.85~0.9米），但用户反馈"穿越柱子直接撞了"，进一步追问后确认了具体
机制：**顶置雷达前倾30度安装，只能看前方偏上方的扇区，飞机是"有头模式"
（机头应转向朝向速度方向），但很多时候飞机已经飞到障碍物跟前、机头还
没转过来，雷达探测不到，就撞上了**——是"转向速度跟不上平移速度导致
探测滞后"，不是规划器完全不工作。

现场实测确认了`mighty_local_frame_only.patch`生效的直接证据：
`/tmp/mighty_debug.log`里`state`（内部当前位置）和发布给控制器的`GOAL`
数值终于一致了（之前差了固定3米的spawn偏移，见上一节），且能找到一段
`state`从(-2.46,0.85)平滑移动到(-1.7,0.11)、绕过局部系(-3,0)柱子、全程
`ok=1 hgp=1`的成功绕障记录。但同一柱子附近也有实际碰撞发生——侧向clearance
只有0.85~0.9米，减去柱子半径0.25米、`drone_bbox`半宽0.6米，飞机机身
边缘理论上贴着柱子表面，余量几乎为零，稍有偏差就会真的蹭上。

**派agent实读代码，找到三处跟"探测滞后"直接相关的证据**：

1. **`w_unknown: 0.0`——全局规划器把"没探测过的未知空间"和"已知空闲
   空间"代价算得完全一样**（`hgp/graph_search.cpp:676-678`确认，权重0时
   对应的`if`直接短路）。前倾雷达视场本来就窄，配合这个"未知区域零代价"，
   JPS/A*会自然选择几何最短路径直穿从没被扫描过的盲区，等真正转到能
   看见障碍物时往往已经太近——跟用户描述的现象完全吻合。
2. **`getDesiredYaw()`（`mighty.cpp`）本身没问题（瞬时`atan2`方向+
   `alpha_filter_dyaw`低通滤波，100Hz下几十毫秒收敛），真正卡脖子的是
   硬限幅`w_max`**：按`v=ωr`估算，`v_max`配合一个绕障急转弯（转弯半径
   小于`v_max/w_max`）需要的瞬时角速度会超过`w_max`，机头物理上跟不上
   规划要求的转向速度。原来`w_max: 1.0`（rad/s），文件里自己的注释写着
   "~4.0 for Hardware"——说明这个1.0是明显偏保守的仿真默认值，硬件那边
   自己都用到4.0。
3. **UAV完全没有"转向跟不上就自动减速"这类保护耦合**：唯一存在的
   "先转向再平移"逻辑只对地面机器人生效（`needReplan()`里判定条件是
   `vehicle_type=="ground_robot" && corridor_hop_enabled`），UAV分支
   完全跳过，没有任何"平移速度随朝向误差自动降速"的保护机制——这是
   设计上的真空，不是这次改坏的。

顺带核实了一个容易和这次问题混淆的历史patch：`mighty_headless_yaw_fix.patch`
（本session之前就有）修的是"UAV机头完全不转、无头模式"这个不同的bug
（`getNextGoal()`原来`use_hardware`一律清零yaw，patch改成只对
`ground_robot`清零）。这个patch本身没问题、也是这次"有头模式确实在
工作"的前提，跟这次"转得不够快"是两个不同层面的问题，以后排查yaw
问题时不要搞混。

## 新增`patches/mighty_avoidance_tuning.patch`：调`w_unknown`和`w_max`

只改`config/hw_mighty.yaml`两处（纯参数调整，不碰任何代码逻辑）：
1. `w_unknown: 0.0 → 0.4`——给"未探测空间"一点正代价，让全局规划器
   对没扫描过的方向天然保守一点，不再无脑走最短几何路径穿盲区。0.4是
   凭经验先给的起始值，不是精确调出来的，如果绕障绕得太夸张（对没有
   障碍物的未知区域也过度避让）可以往下调，如果还是照样直穿盲区可以
   往上调。
2. `w_max: 1.0 → 3.0`（rad/s）——大幅提高yaw跟踪的硬限幅，让机头能够
   更快转向朝向速度/避障方向，向文件自己注释里"硬件用4.0"这个已知能用
   的数值靠拢，但没有直接顶到4.0，留了一点余量（毕竟这个数字是仿真里
   第一次尝试，没有实测验证过跟这套姿态控制链路配合的稳定性）。

**没有改的**：`planner_Co: 0.3`（局部轨迹优化的静态障碍物clearance）
和`inflation_hgp: 0.9`（全局JPS用的膨胀半径）不一致（相差3倍）这个问题
留着没动——上面两个参数改完之后，如果"贴着障碍物飞、余量几乎为零"这个
现象还在，下一步应该调这两个clearance参数，让局部优化实际执行的
安全边界跟全局规划假设的安全边界对齐。

只涉及`flight-stack`镜像（`hw_mighty.yaml`只在`flight-stack`里用），
已验证能在`mighty_local_frame_only.patch`之后干净应用（两个patch改的是
不同行，顺序无关）。**尚未rebuild验证**。

## 新增：`frame_align`同时广播成TF，解决RViz一次只能看一架飞机的问题

用户反馈RViz的Fixed Frame只能挂在一架飞机的局部系下，另一架飞机的点云/
轨迹/位置全都显示不出来——这是`provide_goal_in_global_frame`改成`false`
之后的直接后果：NX01/NX02的TF树被正确地各自独立了，这对"人工监控双机
是否会互撞"这个场景不够用。

`docker_sim/src/uwb_sim/uwb_sim/uwb_ground_truth_node.py`（一手代码，
不走patch）新增：额外订阅两架飞机各自的`/{ns}/dlio/odom_node/odom`
（DLIO局部里程计，机身在自己map系下的位置），跟原有`/frame_align/*`
话题算法逻辑分开、新增一个`broadcast_map_tf()`，把`{i}/map -> {j}/map`
广播成一条TF。

**这里有一个容易踩的坑，专门记录**：原来`/frame_align/*`话题发布的是
"此刻两机机身的UWB相对测距"，随两架飞机飞行实时变化——`frameAlignCallback`
直接拿这个当变换用在共享轨迹上没问题（本来就是每次收到新轨迹都重新做
一次的即时变换）。但**TF不能这么用**：`{ns}/map`被定义成"这架飞机DLIO
SLAM的固定原点"，`{i}/map`和`{j}/map`之间理论上应该是个常量（两架飞机
的SLAM原点各自都不会动），如果直接把"两机身当前相对位置"这个随时间
变化的量当TF广播出去，会变成"NX02整棵地图跟着NX02机身一起满屋子飘"，
明显不对。**正确算法**：用两架飞机各自的DLIO里程计（机身在自己map系
下的位置，`L_i`/`L_j`）把UWB测出来的"机身对机身"世界系位移`D`换算成
"map原点对map原点"的位移：`P(map_j原点 在 map_i系下) = D + L_i - L_j`
——这个换算假设两架飞机spawn时yaw都是0、DLIO重力对齐后也接近0（这套
仿真目前如此，世界系/两机map系三者只差平移不差旋转，位移可以直接
矢量加减，不需要旋转矩阵）；旋转部分沿用现有`frame_align`的简化
（单基线UWB给不出朝向），仍用单位四元数。

`package.xml`补了`nav_msgs`/`tf2_ros`两个新依赖。只涉及`sim-world`
镜像（`uwb_sim`节点跑在这个容器里）。**尚未rebuild验证**——尤其要
确认：(1) RViz Fixed Frame挂`NX01/map`时能不能看到NX02的点云/轨迹摆到
正确的相对位置；(2) 广播频率（跟着`publish_rate_hz`默认10Hz走）够不够
流畅，需要的话可以调。

## 待查：开机一段时间后RViz点云和规划轨迹都不显示了

用户反馈的新现象，现场排查了一轮但**没有找到确凿根因**，如实记录一下
已经排除的方向，避免以后重复排查：

- 不是DDS发现/订阅断开：`ros2 topic info -v`确认rviz2节点仍然是
  `/NX01/mid360_PointCloud2`的匹配订阅者，现场用`ros2 topic hz`（新起
  的诊断命令，最容易踩到之前记录过的Fast-DDS SHM端口耗尽那个坑）实测
  确认点云还在以~12Hz发布，数据没断流。
- 不是TF链断裂：`tf2_echo NX01/odom NX01/NX01_livox`当场能解出正常的
  变换数值（第一行有个"Invalid frame ID"的瞬时警告，但紧接着就正常
  出数据了，猜测是tf2_echo自己刚启动时buffer还没填满的正常瞬态，不是
  真的断裂）。
- 不是`dlio_odom_node`崩溃：`ps aux`确认NX01/NX02的`dlio_odom_node`
  进程都还活着（之前这个进程反复出现过SIGABRT崩溃，一崩点云的TF链路
  就没法解算到Fixed Frame下，见前面章节——这次不是这个原因）。
- 检查了`/dev/shm`：这次现场只有712个fastrtps段（之前记录过的严重故障
  是积压到1834个），没有復现"新参与者抢不到SHM端口"报错，容器日志里
  也grep不到相关报错字符串。
- 观察到`rviz2`进程CPU占用达到92.8%（单核跑满），`gzserver`
  116%——**怀疑是长时间session里DLIO地图/关键帧/marker持续累积，
  导致RViz渲染负载越来越重、逐渐卡到看起来像"不更新"，而不是真的断连**，
  但这只是基于CPU占用的推测，没有拿到确凿证据（比如没有对比"卡住前"
  和"卡住后"rviz2的实际渲染帧率/queue积压情况）。

**建议**：如果再次遇到，先看`rviz2`进程CPU/内存是不是明显偏高（跟这次
92.8%这个量级比），如果是，大概率是长时间运行的渲染负载堆积，之前
记录过的"完整`docker compose down && up`清空`/dev/shm`重新起"这个remedy
之前对类似问题有效、代价也低，可以先试；如果CPU占用正常但确实不显示，
再回头查DDS/TF这两个方向。这条尚未真正解决，下次复现时应该趁着还在
发生、直接对比"正常时"和"故障时"的rviz2资源占用/日志，而不是等好了
之后再排查。

## 用户反馈：`docker compose up`三份日志糊在一个终端里，不好看——反复调整了三版，最后定在"一个tmux session、三个窗口(window)、窗口内不再拆分"

**第一版**：`scripts/watch_sim.sh`用tmux横向拆三个窗格（sim-world/NX01/
NX02各一份完整日志），`scripts/up_and_watch.sh`负责`up -d`之后自动拉起。
但当时`start.sh`自己还是前台裸跑`docker compose up`，没接上这套监控，
用户还是看到糊在一起的日志——先把`start.sh`也改成`up -d` +
`exec ./scripts/watch_sim.sh`，统一到一条路径上。

**第二版**：用户反馈"一个终端分三个窗格屏幕不够大"，改成三个独立的
操作系统终端窗口（探测`gnome-terminal`/`x-terminal-emulator`/`konsole`/
`xterm`），NX01/NX02两个窗口内部还各自按"SLAM/mighty规划/板外控制"三个
模块用`grep`从混合日志里分流出三个tmux横向窗格。

**第三版（当前版本）**：用户反馈三个独立操作系统窗口不好管理，要求改回
tmux统一管理，但用tmux的"窗口"（window，类似浏览器分页/tab，一次整屏
显示一个、互相切换着看，不是同屏拆分）而不是"窗格"（pane，同屏拆分）
——即取消了第二版里NX01/NX02内部按模块拆窗格那一层细分，退回到"一个
tmux session、三个窗口，每个窗口整屏显示一个服务的完整日志"这个更简单
的方案。`watch_sim.sh`重写：`tmux new-session` + 两次`tmux new-window`，
`sim-world`/`NX01`/`NX02`各一个窗口，窗口内部不再有任何pane分割。切换
窗口用tmux默认行为（`Ctrl-a`+窗口号，或`Ctrl-a n`/`Ctrl-a p`翻下一个/
上一个），复用已有的`mighty_sim.tmux.conf`（前缀键`Ctrl-a`、`Ctrl-a k`
二次确认关闭整个session）——这份配置里原来给"窗格切换"绑的方向键
（`select-pane`）现在没有pane可切了，留着无害，不影响窗口切换。

`start.sh`/`up_and_watch.sh`两个入口不用改，接口没变（无参调用
`watch_sim.sh`），行为随脚本重写自动生效。

## 继续补tmux体验：Shift+方向键切窗口、鼠标点击/滚轮、每个窗口固定颜色

用户在上一版基础上补了三个具体要求：窗口切换支持Shift+方向键、支持鼠标
点击切换、每个窗口在状态栏上颜色要不一样。`mighty_sim.tmux.conf`加了：

- `bind -n S-Left previous-window` / `bind -n S-Right next-window`——
  `-n`表示不需要先按前缀键`Ctrl-a`，直接Shift+左右方向键切窗口。
- `set -g mouse on`——顺带解决了鼠标点状态栏窗口名切换、以及用户紧接着
  提的另一个要求"窗口内容能用鼠标滚轮翻看"（tmux鼠标模式下滚轮本来就会
  触发copy-mode翻历史，翻到底自动退出，不用额外配置）。配套把
  `history-limit`从默认2000行调到50000行，不然一直刷的ROS日志翻不了
  多久就到头。
- 每个窗口的状态栏颜色不是在`.conf`里能配的（`.conf`只能配"当前激活的
  窗口"和"其余窗口"两种样式，配不出"三个窗口各自不同颜色"），改成在
  `watch_sim.sh`里窗口建好之后用`tmux set-window-option -t <window>
  window-status-style`给每个窗口单独设一次：sim-world青色、NX01绿色、
  NX02橙色。`.conf`里`window-status-current-style`（加粗反色）负责标出
  "当前正在看哪个"，跟每个窗口自己的身份色是两层不同的东西，不冲突。

**顺带发现并修的一个坑**：本地起了一个没有真实终端的headless测试
session验证这几个改动时，发现`sim-world`窗口如果自己的命令
（`docker compose logs -f sim-world`）提前退出（比如容器没在跑、或者
一时半会连不上），因为它当时是session里唯一的窗口，tmux默认行为是
"窗口的进程退出就关掉这个窗口"——如果关掉的是最后一个窗口，整个session
会跟着消失，后面`NX01`/`NX02`两个`new-window`还没来得及加进来就已经没
session可加了。加了`set -g remain-on-exit on`修掉这个：窗口的命令退出后
窗口本身保留、显示"进程已退出"提示，不会自动关闭、更不会连累整个
session消失——这样即使某个服务临时没起来，三个窗口的框架也不会跟着塌，
等服务起来后在那个窗口里重新跑一次命令就行。

以上改动都在无真实终端的headless环境里用`tmux -L`独立socket实测过：
三个窗口都建出来了、颜色分别是`colour51`/`colour46`/`colour214`、
`mouse on`/`history-limit 50000`/`remain-on-exit on`三个全局选项都生效。
**没有测过的部分**：Shift+方向键切换和鼠标点击切换，因为这两个要有真实
终端交互才能测，等用户在自己的终端里跑`./start.sh`时顺带确认一下。

## 用户要求：mid360从30度前倾改成水平安装、紧贴机体上表面、外观改成半球形，同步调DLIO外参

用户反馈前倾30度"似乎没什么意义，反而带来不好影响"——这跟前面"机头转向
跟不上"那一节里agent找到的证据完全对得上：mid360原生垂直FOV是
-7.2°~+55.2°（本来就整体偏上，不是以水平面为中心对称分布），前倾30度
之后变成22.8°~85.2°，几乎完全偏离水平面，正前方同高度的障碍物本来就不
在扫描范围内，必须先转机头才可能扫到——这是avoidance失效的物理层面根因，
跟`w_max`/`w_unknown`那次参数调整是两个不同层面但会叠加的问题。改成水平
安装后，原生FOV本身就跨越水平面，不再需要转向对准正前方就能看见。

**改了三处，一次性保持一致**（否则TF树和DLIO外参对不上，会重新引入"重力
对齐从第一步就是错的"这类问题——见`dlio_extrinsics_tilt.patch`原来的
教训）：

1. **`docker/scripts/gen_iris_mid360_sdf.py`**（一手代码，不走patch，直接
   决定Gazebo里实际spawn出来的传感器长什么样/装在哪）：
   - `MID360_MOUNT_RPY`从`"0 0.5236 0"`（30度前倾）改成`"0 0 0"`（水平）。
   - `MID360_MOUNT_XYZ`从`"0 0 0.2"`（悬空20cm）改成`"0 0 0.06"`——查了
     `iris.sdf.jinja`里`base_link_inertia_collision`的`<size>0.47 0.47
     0.11</size>`、pose在原点，机体collision box顶面在z=0.055，留5mm
     给安装支架厚度，做到真正"紧贴机体上表面"而不是悬空。
   - 视觉外观从`<box><size>0.1 0.06 0.06</size></box>`改成
     `<sphere><radius>0.05</radius></sphere>`——SDF没有现成的半球图元，
     用一个完整球体、球心卡在安装面上，下半球被机体自己的visual挡住看
     不见，露出来的上半球看起来就是扣在机体上的半球形整流罩，不用额外
     做/找真正的半球mesh。纯视觉近似，跟原来一样没配`<collision>`
     （没有新增碰撞体，维持原状）。
2. **`patches/mighty_sensor_mount.patch`**（改`urdf/quadrotor.urdf.xacro`
   里的`<origin>`，这份URDF不进Gazebo物理仿真，是给`robot_state_publisher`
   发布TF用的，必须跟上面SDF里的实际挂载位置保持一致，否则TF说的位置和
   Gazebo实际仿真的位置对不上）：同样改成`xyz="0 0 0.06" rpy="0 0 0"`。
3. **`patches/dlio_extrinsics_mount.patch`**（原名`dlio_extrinsics_tilt.patch`
   ，这次改名+重写内容，理由见下）：`baselink2imu/t`、`baselink2lidar/t`
   都改成`[0, 0, 0.06]`，两个`R`矩阵这次**不需要再覆盖**——水平安装意味着
   lidar/imu坐标系跟base_link之间不需要额外旋转，DLIO配置文件pristine默认
   本来就是单位矩阵，改回水平之后"不覆盖R"正好就是对的，比之前那版
   Ry(30°)矩阵的patch更简单。改名是因为原来的"tilt"已经不准确（现在是水平
   安装，没有tilt了），继续叫`_tilt.patch`容易让人以为这个patch还在做
   前倾相关的事，误导后续排查。

**验证过patch本身能各自独立对pristine源码干净应用**（`git apply --check`
逐个过了）。

**顺带发现一个这次没有动的、独立于本次改动的既有问题，记录下来避免以后
重复排查**：验证过程中发现`patches/mighty_disable_d435.patch`**即使对
纯pristine的`urdf/quadrotor.urdf.xacro`也无法干净应用**（`git apply
--check`直接报"patch does not apply"，跟这次mid360改动完全无关、
独立复现），大概率是mighty上游仓库这个文件后来又变了、这个patch的
上下文行已经过时。Dockerfile里这类patch失败走的是"打印警告、不中断
构建"的容错分支，所以现在的镜像构建不会因为这个失败而报错，但实际效果是
**这个patch这次很可能一直没有真正生效**（d435相机可能仍然是启用状态，
不是之前记录的"已注释禁用"状态）——之前README记录的"double相机导致
gzserver段错误"那个问题（3.x节，realsense_gazebo_plugin issue #17）如果
最近又复现了，这个失效的patch是一个值得优先怀疑的方向。这次没有动手修，
只是发现顺带记一笔。

**另外顺带发现**：核实这些patch能否干净应用时，发现host上
`staging/mighty_ws_src/mighty`这个目录本身**已经不是pristine状态**
——`urdf/quadrotor_base.urdf.xacro`、`worlds/easy_forest.world`、
`worlds/hard_forest.world`是修改状态，`launch/spawn_agent.launch.py`、
`worlds/simple_room.world`是未跟踪的新文件，猜测是更早之前某次会话直接
在`staging/`里改过东西验证、没有走patch也没有revert干净留下的。**这不
影响实际构建正确性**——`docker build`每次都是从`staging/`当前内容
`COPY`进镜像、再把`patches/`目录里的补丁重新套一遍，`patches/`才是唯一
的权威来源，`staging/`只是个fetch缓存，脏一点不会让最终镜像跑偏（顶多
是某个patch因为"已经应用过"直接跳过、容错分支吞掉报错，效果上是等价
的）。但**如果以后要在`staging/`里手动改东西验证，记得改完对着具体文件
用`git diff`/`git checkout`，不要留下没走patch记录的修改**——这次操作
过程中已经把因为本次验证而弄脏的几个文件（`mighty_node.cpp`、
`launch/base_mighty.launch.py`、`rviz/multi_mighty.rviz`、
`src/mighty/mighty.cpp`、`src/sim/gazebo_ros_imu_sensor.cpp`）revert回
去了，没有动前面提到的几个"看起来更早就已经脏了"的文件，不确定它们的
来历、不敢乱动。

只涉及`sim-world`镜像（雷达SDF生成脚本）+两个镜像共同引用的
`mighty_sensor_mount.patch`（sim-world和flight-stack各自的mighty源码里
都要打）+`flight-stack`镜像的`dlio_extrinsics_mount.patch`。
`Dockerfile.flight-stack`里COPY列表和`git apply`那行都已经从
`dlio_extrinsics_tilt.patch`改成新名字。**尚未rebuild验证**——这是这轮
改动里除了patch本身能否干净应用之外，都还没有实测确认的部分，包括：
半球视觉效果是否符合预期、水平安装后DLIO重力对齐是否正常收敛（水平
安装理论上比之前倾斜安装更简单，风险应该更低，但没有实测不能打包票）、
以及最关键的——水平安装之后飞机绕障时是不是真的不再需要等机头转过来
才能探测到障碍物。

## 一批小改动：雷达球半径减半、RViz名字标注跟随、2D Goal Pose按钮改名、goal_radius收紧、规划器重试上限

用户一次提了六件事，逐条记录：

1. **仿真机机头方向确认**：直接读PX4官方`iris.sdf.jinja`（pristine源码，
   这套docker_sim没改过这部分）确认：4个rotor里`rotor_0`(0.13,-0.22)和
   `rotor_2`(0.13,0.22)——都在x=+0.13这一侧——材质是`Gazebo/Blue`，另外
   两个（x=-0.13侧）是`Gazebo/DarkGrey`。**结论：蓝色桨叶那一侧（+X方向）
   就是机头方向**，这是PX4官方iris模型本来的约定，没有被这个项目改过。
2. **雷达球体半径减半**：`gen_iris_mid360_sdf.py`里`<sphere><radius>`从
   0.05改成0.025（一手代码，直接改，不用走patch）。
3. **`patches/mighty_rviz_name_label.patch`（新增）**：加了一个新节点
   `gt_odom_bridge/name_label_node.py`（放进已有的`gt_odom_bridge`包，
   新增一个console_script入口），订阅`dlio/odom_node/odom`（不是`state`
   ——这个话题名不管`LOCALIZATION_SOURCE`是`dlio`还是`gt`都存在且同名，
   不用关心当前用哪种定位源），发布一个跟着飞机位置走的
   `TEXT_VIEW_FACING` marker（文字默认取namespace）。`multi_mighty.rviz`
   里NX01/NX02两个分组各加了一条"NX0X Name Label" `Marker` Display订阅
   `/NX0X/name_label`。`flight-stack-entrypoint.sh`里加了一行启动这个
   节点（在DLIO/gt_odom_bridge定位源二选一之后，跟mighty启动之前）。
4. **2D Goal Pose按钮改名**：`mighty_rviz_nx02_setgoal.patch`里两个
   `SetGoal`工具的`Name`字段直接改成`UAV01 Goal`/`UAV02 Goal`（原来是
   `2D Goal Pose (NX01)`/`2D Goal Pose (NX02)`），这个patch本来就是新增
   这两个工具的地方，直接改内容不用另开新patch。
5. **`goal_radius: 0.5→0.3`**：加进已有的`mighty_avoidance_tuning.patch`
   （跟`w_unknown`/`w_max`同一个"行为调优"主题，不单独开patch）。
6. **规划器重试上限**——这条改动最深，细节见下一节。

`mighty_rviz_name_label.patch`接在`mighty_rviz_nx02_setgoal.patch`之后、
`mighty_headless_yaw_fix.patch`之前，只涉及`sim-world`镜像（RViz只在
这个容器里跑）。已验证能在完整sim-world mighty patch链（16个patch）
里干净应用，链上仍然只有`mighty_disable_d435`/`mighty_spawn_agent_launch`
/`mighty_simple_room_world`这三个跟这次改动无关的既有问题（后两个是
因为`staging/`里那两个未跟踪文件已经存在导致"already exists"，不是
patch本身坏了；`mighty_disable_d435`是之前发现的、独立于这次改动的
pristine-apply-失败，见上面章节）。`gt_odom_bridge`的`package.xml`/
`setup.py`补了`visualization_msgs`依赖和新entry point。**尚未rebuild
验证实际显示效果**。

## 规划器重试上限：先用"失败次数"设计，被用户追问戳穿了单位不可靠，改成"挂钟时间(秒)"

**第一版**：加`max_replan_failures`（次数），默认3000。**用户追问"为什么
定3000、这需要多久才能达到"**，倒逼重新核实了`needReplan()`
（`mighty.cpp:347-394`）的真实触发频率：确认UAV分支完全没有节流，
只要没到`goal_radius`、没在`YAWING`/`GOAL_REACHED`，每个100Hz
replanCallback周期都会真的尝试一次`replan()`——但同时发现
`hgp_timeout_duration_ms: 10000`（10秒）是**单次HGP求解允许的最长
耗时**，如果某次求解真的卡到接近这个超时才判定失败（JPS/A*类图搜索
理论上通常会较快穷尽搜索空间快速判失败，但不能保证一定如此），单次
尝试耗时可以从几毫秒到10秒不等——3000次在两种极端下换算出来的挂钟
时间能差出上千倍（最好情况~30秒，最坏情况~8小时），"次数"这个单位
本身对"要等多久"这个问题给不出可靠答案。

**改成挂钟时间版**：新增`double max_replan_failure_duration`（秒），
不再数"失败了几次"，而是记录`last_replan_success_time_`（上次成功
replan或上次设新目标的时间戳，`mighty.hpp`新增两个私有成员：这个
时间戳+一个`gave_up_on_current_goal_`去重打印用的bool），`replan()`
开头判断`current_time - last_replan_success_time_ >=
max_replan_failure_duration`就放弃（不再做HGP/局部优化的实际计算，
`plan_`队列不动，飞机保持最后一次有效轨迹/悬停），跟"尝试了几次"完全
脱钩，语义变成"精确N秒后放弃"，不管每次尝试实际耗时多长。
`setTerminalGoal()`里重置这个时间戳（用`local_state.t`当"现在"，因为
这个函数本来没有current_time参数），保证每个新目标都有一份完整的
重试预算，不会被上一个目标的失败历史拖累。

默认值`max_replan_failure_duration: 30.0`（秒）——判断依据：长到能扛过
瞬时遮挡/传感器噪声这类短暂失败，短到"卡住"不会变成静默重试几分钟。
**刻意设得比`yaw_spinning_threshold`（10000次）换算出来的典型真实时长
短**——按现在的默认值，规划器会在原地自旋这个兜底行为真正触发之前就
先放弃目标，也就是说对手动/正常目标而言，自旋这个分支现在实际上很难
再被触发到（除非把这个新参数调到比10000次对应的真实秒数更大）。这是
刻意的取舍、不是疏漏，如果想保留自旋这个"还在挣扎"的可见信号，需要
把`max_replan_failure_duration`调大到明显超过自旋阈值对应的真实时长。

改了4个文件：`mighty.hpp`（新增2个成员）、`mighty_type.hpp`（新增1个
参数字段）、`mighty_node.cpp`（declare/get/log三处，照抄
`yaw_spinning_threshold`的写法）、`mighty.cpp`（`replan()`开头的放弃
判断、成功路径重置时间戳、`setTerminalGoal()`重置时间戳）+
`hw_mighty.yaml`（新参数默认值），合并成一个patch
`patches/mighty_replan_failure_cap.patch`。这个patch只接入
`flight-stack`（不接入`sim-world`——虽然.cpp/.hpp部分对sim-world也是
安全的纯净新增，但`hw_mighty.yaml`那部分依赖`mighty_force_goal_z`/
`mighty_local_frame_only`/`mighty_avoidance_tuning`三个先决patch已经
在同一个文件里落地过的中间状态，而这三个先决patch本来就只在
flight-stack链里，跟`mighty_local_frame_only`/`mighty_avoidance_tuning`
保持同样的"纯运行时调优只进flight-stack"的先例，不去sim-world那边
制造一个必然apply失败的多余patch）。

已验证能接在完整flight-stack mighty patch链（14个patch）最后干净
应用，链上仍然只有`mighty_disable_d435`/`mighty_enable_gravity`这两个
跟这次改动无关的既有问题。C++部分因为没有colcon编译环境，**只做了
人工读代码+插入位置的目视核对，没有实际编译验证过语法/类型正确性**，
这是这次改动里风险最高的一块，rebuild时如果编译报错，大概率是这四个
文件之一。

## 后续两次小改动：2D Goal Pose按钮名改成NX0X、激光点云Decay Time/Color Transformer

用户反馈按钮名"UAV01 Goal"不如直接叫"NX01 Goal"跟话题名一致好认——
直接改`mighty_rviz_nx02_setgoal.patch`里两处`Name:`字段，验证过接
`mighty_rviz_name_label.patch`（读同一个区域）的链路仍然干净。

`patches/mighty_rviz_pointcloud_style.patch`（新增）：`NX01/NX02 Lidar
PointCloud`这两个Display（订阅`/NX0X/mid360_PointCloud2`，不是DLIO的
deskewed/keyframe点云，也不是occupancy/unknown grid——这几个是同一个
文件里其余的PointCloud2 Display，这次没碰）的`Decay Time`从9999改成1、
`Color Transformer`从`Intensity`改成`AxisColor`。接在
`mighty_rviz_name_label.patch`之后、`mighty_headless_yaw_fix.patch`
之前，只涉及`sim-world`镜像。已验证能在完整sim-world mighty patch链
（17个patch）里干净应用，链上仍然只有之前记录过的三个既有问题
（`mighty_disable_d435`/`mighty_spawn_agent_launch`/
`mighty_simple_room_world`，跟这次改动无关）。**尚未rebuild验证实际
显示效果**。

## 意外发现：直接在`staging/`里测patch这个工作方式，会跟用户自己跑的`docker build`产生竞态

用户反馈"2D Goal Pose按钮改名没成功，两个按钮还是都叫2D Goal Pose"、
"镜像build过了"。进容器直接查`/opt/mighty_ws/src/mighty/rviz/
multi_mighty.rviz`（镜像里的实际文件，不是host上的`staging/`）确认：
两个`SetGoal`工具**连`Name:`字段都没有**——不是"改名没生效"，是这次
patch链里`Name:`字段这一层改动整个没进到镜像里。核对了镜像ID
（`docker images sim-world:latest`）确实跟容器实际跑的image一致、
build时间也确实在最近一次改动之后，排除了"用户没重新build"这个可能。

**根因判断**：host上`staging/mighty_ws_src/mighty`是我这次会话反复用来
"apply patch→验证→revert"的工作目录，用户跑`docker compose build`那一刻，
`COPY staging/mighty_ws_src /opt/mighty_ws/src`这一步读的是host磁盘上
**当时那一瞬间**的文件内容——如果那一刻正好撞上我某次测试序列的中间
状态（比如刚apply完`mighty_rviz_two_agents.patch`、还没来得及把
`Name:`字段的编辑做完），build就会把这个不完整的中间态焊进镜像里。
这是我这套验证流程本身的一个真实风险：`staging/`是用户实际build时会
读取的目录，不是一个跟真实构建流程隔离的沙箱，我在上面做的任何测试都
可能被并发的`docker build`撞见。

**当下的应对**：确认host上`staging/`此刻确认是干净的pristine状态，
请用户重新build一次。**以后的应对**：以后验证patch是否能干净应用，
优先用临时目录/独立git worktree做，不再直接在`staging/`这个用户会
实际build的目录上做反复的apply/edit/diff/revert循环，避免再次撞车。

## 又一批RViz小调整：点云Size (Pixels)统一改3、ACL Map Occupied默认勾选

在上面按钮改名的同一个`mighty_rviz_pointcloud_style.patch`里追加了这次
的改动（跟上一版的Decay Time/Color Transformer改动合并成一个patch，
没有拆开）：

- `NX01/NX02 D435 PointCloud`（1→3）、`NX01/NX02 Lidar PointCloud`
  （1.5→3）、`NX01/NX02 2D Occupied Grid`（5→3）——`Size (Pixels)`
  统一改成3。D435 PointCloud这个Display当前是`Enabled: false`（D435
  相机本来就没启用，见前面`mighty_disable_d435.patch`那段既有问题），
  改它的Size不会有可见效果，但用户要求"点云的Size (Pixels)值都改为3"，
  没有排除掉这个，就一起改了。
- `NX01/NX02 ACL Map Occupied`（订阅`/NX0X/occupancy_grid`，`Style:
  Flat Squares`）——`Enabled`和末尾的`Value`都从`false`改成`true`，
  勾选上默认显示。

已验证能在完整sim-world mighty patch链（17个patch）里干净应用。**尚未
rebuild验证实际显示效果**——鉴于上一条记录的竞态问题，这次生成/验证
patch时全程只在`staging/`里做了apply-diff-revert，中间没有暂停等待，
但不能100%排除再次撞车的可能，建议用户看到效果不对时先确认一下容器里
`/opt/mighty_ws/src/mighty/rviz/multi_mighty.rviz`的实际内容，而不是
默认相信这次一定生效了。

## 质量链路修对：真实质量比配置里假设的轻了近一半，还挖出一个patch覆盖顺序bug

用户反馈"感觉飞机控制不住自己、动力不足、扭不过来、总擦着障碍物边缘、
跟踪轨迹差一步"，追问该调哪个参数。这次没有直接猜参数，先派了四个agent
并行核实PX4控制器参数匹配、飞机真实质量、终点超调根因、避障/多机避让
机制，拼出来一个具体、可核实的故事。

**真实质量核算**：`iris.sdf.jinja`里`base_link`(1.5kg)+`imu_link`
(0.015kg)+4个rotor(各0.005kg)，iris本体≈1.535kg；加上
`gen_iris_mid360_sdf.py`里`{ns}_mid360_link`的`<mass>0.4</mass>`（mid360+
IMU负载），真实总质量≈**1.935kg**。

**真实最大推力核算**：`iris.sdf.jinja`单电机`motorConstant(5.84e-6) *
maxRotVelocity(1100)^2 = 7.0664N`，4电机理论上限28.27N。

**发现两条配置链路都严重低估了这个数字**：
- `hw_mighty.yaml`（mighty实际生效的）：`mass: 1.0kg`——比真实轻**94%**
- `vehicle_profile.yaml`（本该是"单一物理参数来源"）：`mass: 1.60kg`——
  比真实轻21%，而且这个1.60kg**从来没有真正生效过**——挖到一个真实
  bug：`mighty_onboard_vehicle_profile.patch`把`vehicle_profile.yaml`的
  覆盖逻辑插在了`hw_mighty.yaml`覆盖块**之前**，导致`hw_mighty.yaml`的
  `mass:1.0`后执行、把`vehicle_profile.yaml`本该生效的1.60kg又覆盖掉了。
  mighty实际用的自始至终是1.0kg，`vehicle_profile.yaml`这个"单一来源"
  设计被顺序bug架空了。

`mass`/`f_min`/`f_max`不是摆设，`lbfgs_solver.cpp`里真实参与推力可行性
约束的优化计算（`mighty_node.cpp`第666-668/1014-1016行declare/读入，
`lbfgs_solver.cpp`多处`f_mean`/`f_radi`计算里用到）。低估质量意味着
规划器以为推力余量远比实际宽松——真实悬停需要1.935×9.81≈19N，之前
`f_max=12N`配`mass=1.0`时规划器以为余量有5倍多，实际按真实质量算只有
约1.4倍，这是"动力不足、扭不过来"这个感觉一个具体、可核实的根因。

**改了三处**：
1. `patches/mighty_onboard_vehicle_profile.patch`（重写）：把
   `vehicle_profile.yaml`的覆盖逻辑挪到`hw_mighty.yaml`覆盖块**之后**，
   让它真正成为最终生效的那一层，修掉顺序bug。
2. `hw_mighty.yaml`（并入`mighty_avoidance_tuning.patch`）：
   `mass: 1.0→1.935`、`f_min: 2.0→3.0`、`f_max: 12.0→26.0`（留约8%
   余量，不假设电机能一直顶满理论最大转速）——这是没有
   `vehicle_profile:=`参数传入时的兜底默认值，独立生效。
3. `docker_sim/config/vehicle_profile.yaml`（直接改，非patch）：同步
   改成`mass: 1.935`、`f_min: 3.0`、`f_max: 26.0`、
   `hover_thrust: 0.67`（=1.935×9.81/28.27）；**顺带把`v_max`/`a_max`/
   `j_max`/`omega_max`/`tilt_max_rad`/`drone_bbox`这几个"规划行为调优"
   参数从这份文件里删掉了**——这些不是物理参数，真正生效的版本应该只在
   `hw_mighty.yaml`里跟着这次别的调参（避障余量、多机避让权重）一起维护，
   之前这份文件里放着一套跟`hw_mighty.yaml`不同步的数值，全靠上面那个
   顺序bug才没有意外生效，这次一起清理掉，避免"改了一个地方、另一个
   地方悄悄生效成别的值"这种坑再犯一次。

**同时确认了ros2_px4_stack这边这几个数字目前没有实际控制效果**：
`get_thrust()`/`get_angular()`（读mass/hover_thrust算姿态+推力那条
差分平坦控制路径）本身是死代码（`_pack_into_attitude`从未被调用，
`dynus_offboard_node.py`里`self.attitude_setpoint`只在`__init__`
赋过`None`），实际控制路径走`_pack_into_traj`/`MultiDOFJointTrajectory`，
PX4自己的位置控制器接管。改这几个数字对当前行为没有直接影响，只是保持
跟mighty那边数值一致、以后如果重新启用姿态直控这条路径能直接用。

只涉及`flight-stack`镜像。三处改动都已验证：patch能在完整flight-stack
mighty patch链（14个patch）里干净应用（链上仍只有`mighty_disable_d435`
这一个跟这次改动无关的既有问题），`vehicle_profile.yaml`过了YAML语法
校验。**尚未rebuild验证实际飞行效果**。

## 用户追问：PX4自己的MPC增益针对的默认质量是多少？不同质量该怎么调？

这个问题派agent查过：**PX4的位置/速度环增益（`MPC_XY_P`/`MPC_Z_P`等）
本身不是按某个具体质量标定的**——PX4的级联控制器设计是外层位置/速度环
输出一个"期望加速度"，这一层在数学上跟质量无关（加速度本来就是
质量无关量）；真正让质量进入这个模型的，是"期望加速度→实际推力/姿态"
这一步转换，靠的是**`MPC_THR_HOVER`**这一个标定点（"多大油门百分比能
悬停"）+ 一条推力曲线模型（`THR_MDL_FAC`），不是一组连续的质量参数。

`MPC_THR_HOVER`官方默认值是**0.5**（50%油门悬停），这套仿真的iris机型
airframe文件（`10015_gazebo-classic_iris`）**没有覆盖过这个参数**，
用的就是这个默认值。反推这个默认值隐含假设的质量：0.5×28.27N
(4电机理论最大推力) / 9.81 ≈ **1.44kg**——比iris光机身(1.535kg)略轻
一点、但明显轻于这套仿真真实的1.935kg（含mid360/IMU负载）。也就是说
PX4当前认为的"隐含质量"跟真实质量差了约26%（1.44kg vs 1.935kg），这个
差距会导致PX4自己的推力/姿态换算这一层出现系统性偏差。

**面对不同质量飞机该怎么调**：核心只需要改一个数——把`MPC_THR_HOVER`
设成`(真实全备质量kg × 9.81) / 最大推力N`算出来的值。这次算出来是
**0.671**（≈0.67）。PX4还有一个在线悬停推力估计器（`MPC_USE_HTE`，
默认开启=1），飞行中会自己微调修正这个值，但只是在`MPC_THR_HOVER`这个
初始种子值基础上小幅修正——种子值差得太远（尤其是起飞、大机动这类瞬态
场景，估计器还没收敛），初期响应还是会跟着不准，静态设对初始值仍然
有必要。

**这条建议先不动手改**：`MPC_THR_HOVER`是PX4飞控自己的参数，这套项目
目前完全没有覆盖过PX4的任何`MPC_*`/`MC_*`参数（airframe文件只设了
`CA_ROTOR*`/`PWM_MAIN_FUNC*`电机布局相关的），加这第一个PX4参数覆盖
需要决定用什么机制注入（`sim-world-entrypoint.sh`里PX4实例是直接跑
`px4`二进制、没有走`param set`这类脚本化的路子，需要另外设计一套
不会跟现有entrypoint冲突的注入方式），风险和之前改mighty/DLIO的patch
不是一个量级，想先明确这是不是你想要的下一步再动手。

## 用户要求：`planner_Co`调到0.6、`dynamic_weight`调到1e2，已并入`mighty_avoidance_tuning.patch`

`planner_Co: 0.3→0.6`（上一版agent核实过这是软惩罚阈值本身偏薄，
0.3m只够1m/s飞行约0.3秒反应时间）、`dynamic_weight: 1e1→1e2`
（上一版agent核实过这个用三次方hinge、远距离时惩罚接近零，只有贴近了
才硬顶，调大让"早期/温和避让"更有效）——两处都是用户直接给的目标值，
不是这次重新调研出来的，跟质量链路的三个改动一起并入同一版
`mighty_avoidance_tuning.patch`，已验证能在完整flight-stack patch链
里跟质量改动一起干净应用。

## 用户要求：检查互传轨迹通不通、坐标统一了吗——进容器直接查的

`/trajs`（`ros2 topic info -v`实测）：2个发布者+2个订阅者，NX01/NX02
双向都有，链路本身是通的。`/frame_align/NX01/NX02`：实测~36Hz持续
发布，取样一帧`translation: (3.15, -5.41, 0.01)`；同一时刻另外查了
`/plug/model_states_plug`世界真值，NX01/NX02位置差值算出来是
`(3.58, -4.99, 0.00)`——量级和符号都对得上（不是完全精确重合，两次
查询不是同一时刻采的样，这段时间飞机还在动，差个0.4米左右是正常的
采样时间差，不是坐标系没统一的证据）。**结论：`/trajs`链路通，
`frame_align`坐标语义正确，双机之间"知道对方在哪"这一层没有问题。**

**顺带发现一个值得知道的特性，不确定算不算bug**：`mighty_node.cpp`第
1625行`if (replanning_result && par_.share_traj) publishOwnTraj();`
——只有**这次replan成功**才会往`/trajs`发一帧。飞机进入`GOAL_REACHED`
悬停状态之后不会再replan、自然也不会再往`/trajs`发新数据；而
`cleanUpOldTrajs`按`traj_lifetime`（7秒）清理过期轨迹——也就是说**一架
飞机悬停超过7秒之后，在另一架飞机眼里就"消失"了**，避让计算里完全不
再把它当障碍物。实测当时`/trajs`确实有5秒钟没有任何新消息，跟这个
机制吻合（当时可能双机都在悬停/替换阶段）。这算不算问题取决于实际
使用场景——如果两架飞机确实会长时间悬停在彼此路径附近，这是一个真实的
安全缺口；如果飞机一直在持续执行任务、极少长时间悬停，问题不大。
这次没有改动这部分逻辑，只是记录下来，用户可以决定要不要处理（比如
悬停时改成定时重发最后一次成功的轨迹，而不是完全停止发布）。

## 用户提出真机部署阶段的问题：两架飞机分别独立部署在不同机载电脑上，地面站怎么建立联系、怎么远程启动，不可能一个个手动

只是讨论，还没有开始实施，先把方案记下来：

**要分清两件独立的事**：软件栈怎么免手动启动，跟飞控层面解锁起飞是两回事——后者已经解决了（entrypoint里`_kick_offboard`那套自动解锁切OFFBOARD逻辑，软件栈一起来飞机自己会解锁起飞）。真正要设计的是"怎么让机载软件栈不用人工逐台登录就跑起来"。

**推荐方案：systemd开机自启 + SSH脚本化fleet命令，先不上重型编排工具**：
- 每台机载电脑（Jetson Orin NX Super 16G，JetPack 6.2）开机自启：JetPack支持Docker（NVIDIA Container Toolkit），现在这套`docker-compose`式entrypoint理论上可以原样搬到真机——每台飞机只需要跑现在的`flight-stack`这一个容器（不需要`sim-world`，真实传感器替代Gazebo）。配一个systemd unit开机`docker compose up -d`，飞机通电软件栈就自己起来，这是"不用一个个手动联系"的核心。
- 地面站按需控制（重启/停止/查状态）：写脚本对已知IP列表批量SSH执行命令，一条命令覆盖全部飞机而不是逐台登录。飞机规模到2~10架这个量级这个方案够用，比Ansible/K8s这类编排工具轻得多。
- 组网：室内WiFi，每台飞机固定IP或mDNS主机名（如`nx01.local`）。ROS2跨机发现建议用**Fast-DDS Discovery Server**而不是默认组播——很多AP会挡组播/广播，Discovery Server用单播更稳，这跟"地面站GCS要连上每架飞机的ROS2图"是同一层问题，之前讨论地面站方案时提过。
- 可选冗余通道：PX4可以额外接一路独立MAVLink链路（数传电台或WiFi），跟机载电脑的ROS2/WiFi数据链分开，万一软件栈或WiFi出问题还能看到基础飞控状态/发紧急指令——是否需要看对故障场景的容忍度。

等真机到手、真正开始部署阶段再具体实施（比如写SSH fleet脚本、systemd unit模板）。

## 用户报告：重新rebuild之后，NX01飞着飞着趴地上了，穿过NX02时也没有任何避让，到达目标点过冲明显，规划轨迹目标高度还越来越低

进容器现场排查（`docker exec`看`/tmp/mighty_debug.log`、`ros2 topic echo`、
`ros2 node info`），四个现象分开查，根因不是同一个：

**1) NX01趴地上——不是SLAM发散也不是撞机，是推力/悬停点没标定**：日志
里`REPLAN ok=1 hgp=1`全程正常（规划器本身持续在成功工作），但`state.z`
持续单调下降直到落地。对照前面"质量链路"那次修复：`mighty`那边的
`mass`/`f_max`已经改成真实值（1.935kg/26N），但**PX4飞控自己的
`MPC_THR_HOVER`参数（悬停油门估计的种子值）从来没有针对这架飞机改过**，
一直是PX4默认的`0.5`（对应"油门50%时刚好悬停"这个假设）。查了真实质量
链条：iris机体1.5kg + imu_link 0.015kg + 4个rotor各0.005kg = 1.535kg，
加mid360+IMU负载0.4kg，合计**1.935kg**；4电机最大总推力
`4 × motorConstant(5.84e-6) × maxRotVelocity(1100)² ≈ 28.27N`；真实
悬停油门应该是`1.935×9.81/28.27 ≈ 0.671`，跟默认值`0.5`差了34%。
`MPC_USE_HTE`（PX4在线悬停推力估计器）默认开着，但只是在种子值附近
小幅微调，种子值本身错34%，在线估计器来不及/不足以纠正过来，表现
为持续掉高度。**修复**：新增`patches/px4_iris_mpc_thr_hover.patch`
——`ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris`
里追加`param set-default MPC_THR_HOVER 0.671`，跟`vehicle_profile.yaml`
里已经算好的`hover_thrust: 0.67`保持一致。这是本项目第一个改动
PX4-Autopilot本身（而非mighty/DLIO/ros2_px4_stack）的patch，接在
`Dockerfile.sim-world`里`COPY staging/PX4-Autopilot`之后、真正
`make`编译PX4固件之前。

**2) NX01穿过NX02时没有任何避让——不是避障算法失效，是NX02压根没在往
`/trajs`发数据，对NX01来说NX02"不存在"**：`mighty_node.cpp`里
`publishOwnTraj()`只在`if (replanning_result && par_.share_traj)`
这个条件下才发布（见前面"检查互传轨迹通不通"那次记录里发现的特性）——
也就是说**只有这一帧replan成功了才广播自己的轨迹**，静止/未收到目标点
的飞机从来不发。实测确认：NX02当时`/trajs`话题上5秒内没有任何新消息，
跟这个机制完全吻合。这是本session之前就记录过、但一直没修的"缺口"——
这次用户明确要求修：**静止的飞机也应该被当成障碍物**，不能"不动就不用
避让"。**修复**：新增`patches/mighty_share_traj_heartbeat.patch`——
在`mighty_node.cpp`构造函数里新增一个1秒周期的心跳定时器
`timer_share_heartbeat_`，绑定新函数`publishSelfAsStaticObstacleCallback()`：
不管有没有收到目标点、不管有没有在replan，只要`par_.share_traj`开着，
每秒都用当前状态构造一条"静止在原地"的`PieceWiseQuinticPol`（6系数
多项式，除常数项外全部为0，即数学上的一个静止点），持续时间60秒，
按跟`publishOwnTraj()`同样的格式（bbox/id/mode="pwp"/is_agent=true）
发布到`/trajs`。跟原有的"replan成功才发"逻辑并存、不冲突——这个心跳
定时器**故意不像`timer_replanning_`/`timer_goal_`那样在没收到目标点时
被cancel掉**，这样一架从开机到收到第一个目标点之间的飞机也能被
其他飞机看见。

**3) 规划轨迹目标高度越来越低——是上面第1点的下游症状，不是独立的
"规划器偏爱低空"的bug**：查了`computeG()`（生成中间子目标点的函数）——
`projectPointToSphere(A.pos, G_term.pos, horizon=15.0)`，这是纯粹的
**线性插值**：从当前位置`A.pos`朝固定终点`G_term.pos`方向、按
`horizon`半径投影出一个子目标点。子目标的z值完全取决于`A.pos.z`
（当前实际高度）和终点z，没有任何"优先同一高度"的偏好项。当NX01因为
第1点持续掉高度时，`A.pos.z`本身在下降，`computeG()`算出来的子目标
自然跟着往下"追"当前这个已经在下沉的高度——**规划器没有做错什么，
是在用一个正在下沉的起点，正确地朝终点做插值**。第1点（`MPC_THR_HOVER`）
修好、飞机不再持续掉高度之后，这个现象应该会自然消失，不需要单独改
`computeG()`。

**4) 到达目标点过冲——本次没有单独定位根因，留待rebuild之后现场验证**：
过冲通常是速度环增益（PX4的`MPC_XY_VEL_P`/`MPC_Z_VEL_P`等）或者
规划器末端减速曲线（`goal_radius`/`goal_seen_radius`附近的减速逻辑）
没调好，两种都有可能；也可能是第1点悬停推力错误导致的整体控制不稳定
的连带表现——建议先rebuild验证第1点修完之后过冲是否自然改善，再决定
要不要单独调速度环增益。

只涉及`sim-world`（PX4 patch）和`flight-stack`（mighty patch）两个镜像。
`px4_iris_mpc_thr_hover.patch`、`mighty_share_traj_heartbeat.patch`
均已过`git apply --check`验证（分别对着PX4-Autopilot pristine源码、
完整flight-stack mighty patch链）。**尚未rebuild验证实际飞行效果**——
第4点过冲问题也需要rebuild之后现场核对。

## 用户重新追问板外控制器控制律问题："两种控制律都完整吗？当前用哪一种？有没有切换参数？"

比之前(1403行那次)问得更精确，重新确认一遍代码现状（rebuild之前的
状态）：

- **两种控制律的计算逻辑都是完整、正确的**：`_pack_into_traj()`
  （轨迹模式）和`_pack_into_attitude()`/`get_orientation()`/
  `get_angular()`/`get_thrust()`（姿态模式，微分平坦前馈）都是写完的、
  经过代码审查确认数学上正确的实现，不是半成品。
- **当前只用轨迹模式，姿态模式是100%死代码**：`self.attitude_setpoint`
  从`__init__`赋值`None`之后，在原代码里**从来没有在任何地方被重新
  赋值过**——`takeoff_and_track_trajectory()`状态机的TRAJECTORY分支
  只调用`_pack_into_traj()`、只赋值`self.trajectory_setpoint`。
  `_publish_setpoint()`里虽然写了`if self.attitude_setpoint is not None:
  ... elif self.trajectory_setpoint is not None: ...`这个分支判断（说明
  设计者原本是打算做两条路都能走的），但因为`attitude_setpoint`永远是
  `None`，这个分支实际上永远走不到。
- **当前完全没有切换参数**：原代码里没有任何`declare_parameter`/环境
  变量/配置项能决定走哪条控制律——不是"有开关但默认关着"，是**压根没有
  开关这个东西**，走哪条路完全由"有没有代码给`attitude_setpoint`赋值"
  这一个事实硬编码决定。

## 用户要求："另一种控制律也给完善，并设定参数控制切换"

新增`patches/ros2_px4_stack_control_law.patch`（`dynus_offboard_node.py`）+
`patches/ros2_px4_stack_control_law_launch.patch`（`launch/dynus_mavros.launch.py`）：

1. `OffboardDynusFollower.__init__`新增ROS2参数`control_law`
   （`declare_parameter('control_law', 'trajectory')`），只接受
   `'trajectory'`/`'attitude'`两个值（非法值直接`assert`报错，不静默
   兜底成默认值——写错参数应该在启动时就炸出来，而不是悄悄飞成另一种
   模式）。
2. `takeoff_and_track_trajectory()`状态机补全：
   - `TAKEOFF`阶段：不管`control_law`是什么，永远走位置setpoint
     （`self.attitude_setpoint = None`）——起飞爬升本身没有理由用微分
     平坦前馈，而且PX4需要一路连续的位置setpoint流才能解锁/进入
     OFFBOARD，姿态setpoint流替代不了这个作用。
   - `TRAJECTORY`阶段：新增`if self.control_law == 'attitude':`分支，
     调用`self._pack_into_attitude(...)`并把`self.trajectory_setpoint`
     清`None`；否则走原来的`_pack_into_traj(...)`并把
     `self.attitude_setpoint`清`None`。两个setpoint变量互斥，
     跟`_publish_setpoint()`原有的`if attitude_setpoint is not None:
     ... elif trajectory_setpoint is not None: ...`判断逻辑对上——
     这个判断逻辑之前就已经写好、只是从来没被真正触发过，这次改动是
     "接上"而不是"新建"这条路径。
   - `RETURN`阶段：同`TAKEOFF`，固定走位置setpoint。
3. `launch/dynus_mavros.launch.py`：读环境变量`CONTROL_LAW`（默认
   `trajectory`），通过`parameters=[{'control_law': control_law}]`
   传给`track_dynus_traj_py`节点。
4. `docker/entrypoints/flight-stack-entrypoint.sh`（直接改，非patch）：
   新增`export CONTROL_LAW="${CONTROL_LAW:-trajectory}"`，跟已有的
   `LOCALIZATION_SOURCE`模式保持一致，容器启动时可以通过
   docker-compose环境变量整机切换。

**`control_law='attitude'`这条路径从写完到现在从未实际试飞过**——
数学计算逻辑经过代码审查确认正确，但"公式对"不等于"飞过验证过"，
下面这次联网调研也确认了这不只是本项目自己的问题（见下一节）。
建议先在`trajectory`模式下把上面第1点的悬停推力问题rebuild验证完，
确认基本飞行稳定之后，再单独找一次开阔场景专门测`attitude`模式，
不要两个变量一起改。

只涉及`flight-stack`镜像。两个新patch已在完整patch链（7个
ros2_px4_stack补丁，按`dynus→vision_pose_staleness→takeoff_altitude→
kick_offboard_fix→kick_offboard_timeout→control_law→control_law_launch`
顺序）里过了`git apply --check`验证。**尚未rebuild验证**。

## 用户要求：联网查DYNUS作者自己的`ros2_px4_stack`版本（jrached/kotakondo），确认attitude模式历史上有没有真的被飞过

背景：用户的直觉是"作者做敏捷飞行实验，必须用attitude模式"（轨迹/位置
setpoint模式理论上响应更慢，敏捷机动一般确实更依赖姿态直控）。查了
三个能找到的版本：

- **`jrached/ros2_px4_stack`**（`jrached`= Juan Rached Viso，DYNUS
  论文作者之一，main分支）
- **`kotakondo/ros2_px4_stack`**（Kota Kondo，DYNUS论文一作，main分支）
- **`kotakondo/ros2_px4_stack`的`dynus`分支**（本项目实际基于的版本，
  最近一次更新到2026-07-20，是三者里最新的）

**三个版本结论完全一致**：`_pack_into_attitude()`/`get_orientation()`/
`get_angular()`/`get_thrust()`这套微分平坦姿态解算代码都存在，但
`attitude_setpoint`在状态机里**从来没有被赋值过**——跟本项目rebuild
之前的原始代码是同一种"写完但没接上"的状态，不是本项目自己引入的疏漏。
`dynus`分支的提交历史里能看到真实硬件试飞记录（比如"full static max
vel 1.0m/s working on PX03"这类提交信息，`PX03`是他们的真机代号），
但翻了提交信息列表，没有任何一条提到attitude模式被测试过（受限于
只能看到commit标题，没有逐条看diff，不能100%排除某次提交里静默
改过又撤回，但从标题看没有痕迹）。

**交叉验证**：DYNUS论文本身（arXiv 2504.16734）描述硬件下发接口的
原文是"The position trajectory p(t) and yaw trajectory ψ(t) are then
transmitted to the low-level controller"——即论文作者自己描述的硬件
实验数据流是位置+偏航轨迹，对应的正是轨迹模式（`MultiDOFJointTrajectory`），
不是姿态/推力量。

**结论**：至少在这几个可查的版本和论文原文里，**没有证据支持"作者的
敏捷飞行硬件实验真正用过attitude模式"**——姿态直控路径更像是预留的
未来工作，实际飞过、验证过的是轨迹模式。这不代表attitude模式的数学
是错的（前面代码审查已确认是对的），只是提醒：`control_law='attitude'`
这条路径没有任何已知的真实飞行先例可以参考，真的要测的话应该当成
"从零开始的新功能验证"来对待（先低速小范围、有人值守、随时能切回
`trajectory`或接管），而不是假设"论文作者验证过所以稳"。

## 用户实测反馈：双机运行一段时间后，运动飞机经过悬停飞机时会停顿卡住；之后重新规划时轨迹自己走、飞机不动，起点是上一次规划的终点

进两个容器直接读`/tmp/mighty_debug.log`（NX01/NX02各自十几万行，写了个
Python脚本解析`REPLAN ok=%d hgp=%d state=... A=... G=... Gterm=...`这行
格式，按时间戳找`ok=1`到`ok=0`的转折点），同时对照`mavros/local_position/
pose`实时查真实位姿，确认了两件事——**症状是真的，且两架飞机当时都已经
卡住很久了**：

- NX02：从sim时间~651s开始`REPLAN ok=0 hgp=0`，往后10000+行全是`ok=0`，
  一直卡到我实测时的~1281s（十分钟以上）。当时`A`（规划起点）固定在
  `(5.25,4.65,1.35)`（半空中），但实测`mavros/local_position/pose`是
  `(3.12,5.84,0.04)`——飞机实际上已经**落地在完全不同的位置**，A和真实
  状态相差约3米，但规划器完全不知道，一直拿这个3米外的"幽灵点"当起点。
- NX01：从sim时间~205s开始同样`ok=0 hgp=0`，一路卡到~1358s，不过NX01
  这边A和实际状态只差~0.15m——飞机其实稳稳悬停在离目标只差一点点的位置
  （离目标0.30m，正好卡在`goal_radius=0.3`的边界上），没有真摔，只是
  永远差一口气"到达"，然后规划器已经放弃。

**根因分两层，一层是本session早前刚加的时间上限give-up机制的副作用，
另一层是更底层的架构问题**：

1. `mighty_replan_failure_cap.patch`（今天之前的session加的）：连续
   replan失败超过`max_replan_failure_duration`（30秒）就永久放弃当前
   目标，`gave_up_on_current_goal_=true`之后每次replan直接在最前面
   短路返回，不再调用`checkReadyToReplan`/`needReplan`/`findAandAtime`
   等任何一步——这本身没错（防止无限重试烧CPU是用户明确要求的），但
   放弃之后没有任何补救动作，飞机就晾在那不管了。

2. **更深的根因，在`findAandAtime()`（`mighty.cpp`）里**：查了这个函数
   才发现，规划起点`A`从来都不是"当前实际状态"，而是从`plan_`（上一次
   成功规划提交的那条轨迹）里按`k_value_`往后数取一个"预计算耗时之后
   飞机应该到达的点"——这是DYNUS这类实时重规划算法的标准设计（让局部
   优化器提前于飞机实际位置去算下一段，不用每周期等飞机追上才敢算），
   平时没问题。但**这个假设从来没有跟真实定位状态做过校验**：一旦飞机
   真的没能跟上这条轨迹（被卡住、掉落、被避障逼停），`A`就永远停留在
   飞机"应该在"却实际不在的地方，规划器在一个跟现实完全脱节的"幽灵
   位置"上继续规划，真实飞机反而被彻底遗忘。而且这个问题**在give-up
   触发之后仍然存在**：即使发一个全新的目标点让`gave_up_on_current_goal_`
   复位（`setTerminalGoal()`里本来就会重置这个标志），下一次`findAandAtime()`
   还是照样从同一个`plan_`的旧尾巴取`A`——这正是用户观察到的"起点是
   上一次规划的终点"，给新目标也没用。

**修复：`patches/mighty_replan_state_resync.patch`**——在`findAandAtime()`
算出`A`之后，跟`getState()`拿到的真实定位状态比对：飞机在`A_time -
current_time`这段预计时间内、按`v_max`最多能走多远，乘以2倍余量作为
"合理超前距离"的上限；真实状态和`A`的距离一旦超过这个上限，就说明飞机
根本没跟上这条轨迹，直接把`A`换成真实状态、`A_time`换成当前时间，让
下一次规划老老实实从飞机真正在的地方重新算。这个阈值只跟`v_max`（飞机
自己的动力学参数）和经过的时间有关，不涉及任何场景相关的坐标，换房间、
换真机都一样适用。实测数字校验过量级：正常悬停时A和真实状态差~0.15m，
真正卡住时差~3m，公式在默认参数下（`v_max=1.0`）算出的正常余量在
2-3米量级，能把这两种情况分开。

## 用户实测反馈："为什么选墙外边目标点也能规划出路径呀"、"规划到墙外的点，无人机直接撞上墙了"——第一版修复被用户当场指出是瞎搞，推倒重来

**第一版修复（已废弃，记录下来提醒自己不要再犯）**：一开始的思路是查
`hw_mighty.yaml`里`x_min/x_max/y_min/y_max`这几个参数——发现默认值是
`±1000`（相当于没有边界），就想着照抄`simple_room.world`里读到的四面墙
坐标（东西墙x=±10、南北墙y=±10），把这几个参数硬编码改成`±9.5`。**用户
当场指出这是错的**："越界检测哪能根据场景的具体范围判断？这不是乱弹琴
吗，换一个环境呢？不是应该根据点云或占据栅格地图判断吗" —— 完全正确，
把某一个特定房间的墙体坐标写死进规划器配置，换个房间、换成真机在别的
场地飞，这个"边界"就完全是错的，而且没有任何机制会提醒你去改它。已经
把这版patch删掉，没有留在`patches/`目录里。

**回头认真查了`x_min/x_max`到底控制什么，才发现连"硬编码这几个参数"这个
思路本身都站不住脚**——顺着`HGPManager::setParameters()`（`hgp_manager.cpp`）
查到`map_util_ = std::make_shared<VoxelMapUtil>(..., par.x_min, par.x_max,
par.y_min, par.y_max, ...)`，再查`MapUtil`构造函数调的`setMapSize()`
（`map_util.hpp`），发现`setMapSize()`只是把这几个数存进`x_map_min_`/
`x_map_max_`几个成员变量，**根本没有拿它们去分配实际的占据栅格数组**——
真正决定`isOutside()`/`isOccupied()`用到的`dim_`/`total_size_`/
`origin_d_`，是在另一处专门"用真实点云/占据栅格消息更新地图"的函数里
按接收到的栅格数据算出来的（数组大小、原点全部来自实际收到的
occupancy_grid/unknown_grid消息，不是`x_min/x_max`）。也就是说
**`x_min/x_max`这几个参数本来就管不到"某个点是不是在已知地图范围内"这件
事**，就算按用户当时的思路把它们改成房间坐标，也不会对"墙外目标点"这个
bug产生任何实际效果——一开始的修复方向从根上就是错的，不只是"数字不该
硬编码"这么简单。

**真正的bug，在`map_util.hpp`的`isOccupied()`里**：
```cpp
inline bool isOccupied(const Veci<Dim>& pn) const {
  if (isOutside(pn))
    return false;   // <-- 超出栅格数组范围 = "没被占据"
  else
    return isOccupied(getIndex(pn));
}
```
任何超出栅格数组实际分配范围（`dim_`）的点，无论物理上是不是真的有堵
墙，一律判定成"free"（没被占据）。这个约定对JPS/A*内部的图搜索本身没
问题（搜索节点扩展天然就不会跑到数组外面去），但`sanitizeTerminalGoal()`
（决定要不要接受一个新目标点的函数）复用的正是这同一个`checkIfPointOccupied()`
——目标点如果落在栅格数组范围之外（意味着建图系统压根没有那里的任何
数据，不管是因为那是真实世界里的一堵墙，还是仅仅因为传感器还没扫到那么
远），会被当成"clear"直接放行，规划器于是心安理得地算出一条穿墙路径，
飞机也就真的照着飞过去撞墙了。

**修复：`patches/mighty_goal_outside_map_rejected.patch`**——在
`HGPManager::checkIfPointOccupied()`里加一行：栅格数组范围之外的点，
直接判定成"occupied"（而不是维持原来的"free"）。这样`sanitizeTerminalGoal()`
既有的重定位逻辑（BFS找最近的free/unknown格子，找不到就直接拒绝目标点，
`mighty_node.cpp`里的回调已经处理了拒绝的情况，只是打个ERROR日志、不
接受目标）自动就适用于"目标在地图范围外"这种情况，不需要再单独写一套
处理逻辑。这个判断完全基于`map_util_`实际持有的栅格数组边界，这个边界
本身来自建图系统实际收到的传感器/占据栅格数据——**跟房间多大、墙在哪
完全没关系，换任何环境，只要建图系统对那块空间没有数据，目标点就会被
挡下来**，不会重演"硬编码这个房间的墙"的错误。

**两个patch都已验证**：分别对着flight-stack（14个mighty前置patch）和
sim-world（17个mighty前置patch）两条完整链跑过`git apply --check`，
链上唯一失败的是`mighty_disable_d435`（跟这两个新patch无关的既有冲突，
之前已经记录过）。已加进两个Dockerfile的patch链。**尚未rebuild验证
实际效果**——特别是`mighty_goal_outside_map_rejected.patch`，需要重新
点一次墙外的目标点，确认RViz里目标点被拒绝（日志里能看到"could not
escape occupied region"或类似的ERROR）而不是真的往墙外規划；
`mighty_replan_state_resync.patch`需要人为制造一次"飞机没跟上规划轨迹"
的场景（比如让它撞一次墙、或近距离掠过悬停飞机触发局部优化器失败），
确认后续replan能重新用真实位置续上，而不是永远卡在幽灵点上。

## 用户第二次指出上一节的修复也是错的：`mighty_goal_outside_map_rejected.patch`已撤销

**上一节的`mighty_goal_outside_map_rejected.patch`（`isOutside()`一律当
occupied）已经从`patches/`目录删除、从两个Dockerfile的patch链里去掉**。
用户指出的问题：

> 墙外点不等于就是局部地图即动态窗口之外的点。窗口外的点可以认为是
> free，也必须free。规划器只需要把局部地图即窗口内的路径找出来，路径
> 到了窗口边沿如果也是free，从这到目标点可以先假设全是free，这样整体
> 轨迹就能规划了，但是防碰撞检测只做窗口内的。因为窗口是动态的，窗口
> 外的那些假设为free的点，等窗口移动覆盖后自然就看清了，规划也是动态
> 的，free的假设不会影响什么。

这是对的，而且点出的是这类receding-horizon（滚动时域）规划器的**设计
本质**，不只是"漏了一种情况"这么简单：`wdx_/wdy_/wdz_`这个动态窗口
（`computeMapSize()`，跟着飞机走，大小由离子目标`G`的距离决定，`G`
本身被`horizon=15.0`封顶）本来就只覆盖飞机附近一小片，窗口之外没有
传感器数据、也不该有——"假设为free"正是让规划器能对着一个远处的目标
先算出一条大致朝向那边的轨迹，随着飞机往前飞、窗口跟着往前滑动，之前
"假设free"的空间逐渐被真实点云覆盖，如果发现是障碍物，后续的每周期
重规划自然会绕开——**碰撞安全性是靠这套"持续重规划+只在窗口内做真正的
避障检查"来保证的，不是靠一次性拒绝掉一个暂时看不清的目标点**。我把
`isOutside()`直接判成occupied，等于把这套设计的前提破坏了：任何当前
恰好在15米+缓冲窗口之外的目标点（不只是墙外的，随便一个稍微远一点的
正常目标都可能被误伤）都会被`sanitizeTerminalGoal`拒掉，这比"偶尔选到
墙外点"这个原问题更糟。

**这意味着"选墙外目标点也能规划出路径"本身很可能不是bug，是设计的一部分
——真正需要解释的是"为什么飞机真的撞上了墙"，而这必须发生在墙已经进入
当前动态窗口、理论上已经被点云扫到并标记成occupied的情况下**（不然
按上面这套设计，那本来就该是"假设free"、按下一步重规划自然避开的正常
过程，不该被苛责）。这一节没有得出结论，需要下一步现场复现："撞墙那
一刻，那面墙对应的格子在当时的局部地图里到底是occupied还是unknown"——
如果是unknown，说明是感知覆盖不足（雷达还没扫到/FOV盲区，这跟本session
早前把mid360从前倾30度改成水平安装那次讨论的盲区问题可能是同一类
原因），不是规划器逻辑错误；如果确认是occupied、规划器仍然算出一条
穿过去的轨迹，那才是局部优化器（L-BFGS，`planner_Co`/`stat_weight`等
碰撞约束权重）的真实bug，需要另外查。**这一层还没有查，需要先复现
一次真实的撞墙场景、现场核对当时那面墙格子的occupied/unknown状态。**

## 现场复现：NX02/NX01先后真的撞墙了，实测抓到墙体点云上的真实缺口；发现JPS的3D模式缺一道corner-cutting检查

**用户提问推了一步很关键的追问**："夹角怎么能穿过呢？碰撞检测也应该
是过不了的，还是最终轨迹没有窗口内的碰撞检测处理？"——查了JPS路径
之后还有没有独立的碰撞检查这一层，答案是：**有，但检查的是离散点云，
不是连续实体几何**。`cvxEllipsoidDecomp()`（`hgp_manager.cpp`）用
`decomp_util`第三方库的`ellip.dilate(seg_path, ok)`围绕每段JPS路径
生长一个不含给定障碍点的凸多面体（安全走廊）——如果JPS那条对角线路径
恰好从两个障碍体素中心之间的空隙穿过，而空隙里没有任何一个障碍点卡着，
`dilate()`照样能围出一个"合法"的走廊，这一层算法本身没有错，它只对
给它的点集负责，管不了点集之间物理上到底连不连续。

**同时读到JPS图搜索本身一个实打实的缺陷**：`graph_search.cpp`的
`jump()`函数里有一段"防止对角线移动贴着两个障碍夹角抄近道"的检查：
```cpp
// Prevent corner-cutting in 2D mode: for diagonal moves, check axis-aligned neighbors
if (zDim_ == 1) {
  const int move_norm = std::abs(dx) + std::abs(dy);
  if (move_norm >= 2 && dx != 0 && dy != 0) {
    if (!isFree(x + dx, y, z) || !isFree(x, y + dy, z)) return false;
  }
}
```
**这段检查显式写死`if (zDim_ == 1)`——只在2D地面机器人模式生效，UAV
用的3D模式完全没有这道检查**，注释本身就说明作者当时知道这个问题，
只是没有把2D的修复顺手扩展到3D。

**现场实测数据**（NX02，2026-08-05）：`REPLAN`日志显示最后一次成功
replan（`hgp=1`）发生在`state.x≈-9.2`（局部帧，恰好卡在墙体inflation
边界），此后3秒内飞机实际位置就到了`x=-10.74`（世界坐标，已经穿墙
而出，实测world pose确认飞机最终停在`(9.56,-3.0,0.05)`——紧贴东墙、
落地）。用`--qos-reliability best_effort --truncate-length 100000`
现场查了当时`/NX02/occupancy_grid`（这就是喂给mighty内部地图、
`mighty_node`实际订阅的那个话题，不是纯可视化）的真实点云：墙体主层
（局部x=3.6，对应世界x≈9.6）在y方向上是一条几乎连续的密集线（每0.3米
一个点），**但恰好在NX02撞墙的y坐标（y≈-3.0）附近，这条线上有一个
0.6~0.9米宽的真实缺口**（y=-3.6和y=-2.7都有点，中间y=-3.3、y=-3.0
完全没有点）——跟撞墙坐标严丝合缝对上。`inflation_hgp=0.9`按理说应该
能糊住这种量级的缺口，缝隙依然存在，说明感知覆盖缺口（那个方向雷达没
扫到/扫得稀疏）和JPS 3D缺corner-cut检查，**大概率是叠加在一起共同
导致的**：真实缺口本来就有，inflation之后可能只是变窄没有完全消失，
残留的窄缝正好是只有装了corner-cut检查才会被挡住的那种。

**修复：`patches/mighty_replan_state_resync.patch`**（复用之前那个
patch名字下的同一个改动，实际这次没有改这个文件——见下面新增的
`mighty_trajectory_wall_check_log.patch`）。真正新增的是**诊断能力
而不是修复**：用户建议"应该把轨迹打印或记下来，拿出来分析会更好些"——
现场用手动查`ros2 topic echo`凑巧才抓到这次的证据，运气成分太大，不该
依赖"刚好在飞机撞墙那一刻手动去查"。新增
**`patches/mighty_trajectory_wall_check_log.patch`**（`mighty_node.cpp`）：
每次replan成功后，用`mighty_ptr_->retrieveGoalSetpoints()`取出**实际
提交的局部轨迹**（按`dc`间隔的精细点列，不是上面`HGP_PATH`那种稀疏的
JPS路径点），逐点用`mighty_ptr_->getMapUtil()`（已有的公开接口）查
`isOccupied()`；只要有一个轨迹点真的落在occupied格子里，就在
`/tmp/mighty_debug.log`里记一行`WALLHIT`，带上违规点坐标和以它为中心
±3格的occupied/free/unknown邻域字符画（`O`/`.`/`?`），一次replan周期
最多记一次，不会持续刷屏。这样以后再发生类似情况，直接翻debug log就
能看到实锤，不需要再赌运气现场蹲一次。

只涉及`flight-stack`和`sim-world`两个镜像。两个patch都已对着两条完整
patch链跑过`git apply --check`（`flight-stack`16个mighty patch、
`sim-world`19个），链上唯一失败的依旧是`mighty_disable_d435`这个
既有的、跟这次改动无关的冲突。**`mighty_trajectory_wall_check_log.patch`
尚未rebuild验证——需要重新构建镜像后，再等一次真实撞墙（或者干脆手动
点一个贴墙的目标点去触发），确认`WALLHIT`真的会被记下来，且记录的
邻域字符画能跟这次手动查到的缺口对得上。**corner-cutting那道检查本身
这次没有动（3D场景怎么补这道检查、要不要连带查3D对角线的面对角线邻格，
需要先看WALLHIT记录累积几次真实案例、确认这确实是主因之后再动手，
不想在只有一次拼凑证据的情况下就去改JPS核心搜索逻辑）。

## rebuild之后现场复现：`WALLHIT`日志确实抓到了真撞墙；用户点破真正的根因——高度方向"越界即free"跟柱子/墙的不对称

**`mighty_trajectory_wall_check_log.patch`已确认在真实rebuild里生效**：
用户操作NX02飞行、真的又撞墙了（世界坐标落在东墙附近`(9.48,-0.05,0.05)`），
现场翻`WALLHIT`记录抓到718条命中。**但发现了这套新诊断工具自己的一个
假阳性来源**：绝大多数`traj_idx=0`（轨迹起点）的命中，很可能只是因为
`WALLHIT`检查用的`getMapUtil()`拿到的是`map_util_`（原始底图），而
真正规划时用的是`map_util_for_planning_`——一份`freeStart()`/`freeGoal()`
（`use_free_start: true`）已经手动把A点周围清成free的独立副本，两者
口径不一致，起点报警大概率不是真的从一开始就卡在障碍物里，是我这个
检查工具本身比对错了地图版本（还没修，先记录下来，后面几条真正有意义
的命中不受这个影响）。撞墙前最后几条命中在`traj_idx=14~17`（总长
347~348点，也就是刚执行了大约0.15~0.2秒的全新"成功"轨迹），邻域
±3格y方向**全是occupied、没有缝隙**——这次不是钻点云稀疏的空子，是
规划器把一条明知故犯穿墙的轨迹当"合法"提交了。

**用户的分析点破了真正的根因**：柱子这种孤立障碍物，飞机能在滑动窗口
内直接左右绕开；但一整面墙横贯整个窗口宽度，左右绕不开，规划器只能
往高度方向想办法——**规划器本身没有失效，是在按"窗口外一律当free"这条
设计规则正确地找一条"看起来能走"的路，只是这条路往上正好是墙延伸出
窗口顶部之外的部分**。用户原话："我觉得这种情况应该禁止高度出界，
高度范围外的都应该设为被占据。"——跟之前"窗口外的xy必须当free"那次
纠正看似矛盾，但其实是同一个原则在不同轴上该有的不同结论：**xy方向
"窗口外当前是free"之所以安全，是因为窗口会跟着飞机往前飞自然滑动过去，
之前看不到的地方飞近了自然就看清了，是会自愈的；但高度方向的窗口范围
是绕着飞机当前高度定的，不会因为飞机在同一个高度往墙的方向继续水平飞
而自动往上"滑出"更多天花板视野——真撞上之前，"上面"永远不会自己变
成"看清了"，这个assume-free假设在z轴上从来不会真正被验证，也永远
不会自愈。**

顺着这个思路去查`readMap()`（`map_util.hpp`）才发现：这个函数第8b步
本来就有一段"在窗口边界标记occupied、防止SFC走廊贴着全局y_min/y_max
往外长"的逻辑，**但当时的注释明确写着"z边界不用管，因为z_min/z_max
已经在约束地图了"**——用本session早前算过的数字可以证明这个假设是错的：
`min_wdz=4.0`米、飞机常见悬停高度~1.4米时，下边界会被`z_min=0.3`夹到
只剩约1.1米、上边界没有对应放大补偿，窗口顶部实测大约只到`z≈3.4米`，
根本到不了`z_max=5.0米`（正好等于墙的真实高度）——"z_min/z_max已经
约束了地图"这个假设只保证了窗口不会被放在天上或地下，从没保证窗口
"顶到"配置的那个高度上限时那一层格子真的有数据。

**修复：`patches/mighty_z_window_ceiling_occupied.patch`**——照着已有
的y边界标记逻辑的写法，在`readMap()`8b步里加一段：把当前窗口数组的
**最顶上一层z格子（`iz=dimZ-1`，不是配置的`z_max`，是这个窗口这次
实际分配到的那一层，两者常常对不上，前面已经算过为什么）**整层标记成
occupied。之前那段"z边界不用管"的注释也一并改写，说明白为什么这个
假设是错的、以及为什么只标顶层不标底层（窗口下边界被`z_ground`夹住
之后本来就常常等于地面，标记它跟"地面本来就该算障碍物"是一回事，
物理上说得通；上边界则完全是本次要修的问题，两者不对称，不该套同一套
处理）。这样一来，飞机被墙堵住、往高处硬闯的时候，会撞上这个新标记的
"天花板"、JPS/凸分解会正确地判定这条路走不通，`REPLAN`应该会正常变成
`ok=0`（老老实实告诉你"这个方向也堵死了"），而不是悄悄提交一条真的
往墙里钻的轨迹。

跟之前那次被撤销的`mighty_goal_outside_map_rejected.patch`（把xy方向
的"越界"也当occupied，被用户指出破坏了receding-horizon设计本身）不是
同一类改动——这次严格只动z轴、且只标记"当前窗口实际分配到的顶层"，
不触碰xy方向"越界即free"这条继续保留、continue正确工作的设计。

只涉及`flight-stack`和`sim-world`两个镜像，对着两条完整patch链
（17个、20个mighty patch）都跑过`git apply --check`，链上唯一失败的
依旧是那个跟这次改动无关的`mighty_disable_d435`既有冲突。**尚未
rebuild验证**——需要重新构建镜像后，再让飞机贴着墙飞一次，确认：
1）撞墙不再发生，或至少不再是"翻墙"这种模式；2）`REPLAN`在被墙+
高度上限双重堵死时能正确变成`ok=0`而不是继续算出穿墙的轨迹；
3）`WALLHIT`记录里`traj_idx>0`的真实命中数量应该明显下降。

## 用户提问："占据栅格用的是机体系还是本机局部系？"——确认是本机局部ENU系，据此收窄z_min/z_max强迫近似同高度飞行

现场查了一次真实`/NX01/occupancy_grid`消息的header，`frame_id`是
`NX01/map`——每架飞机自己的局部`map`坐标系，重力对齐（ENU），不随
机体姿态转动，不是机体系。z就是真实垂直高度，收窄`z_min`/`z_max`
是在直接约束真实海拔高度，思路成立。

**新增`patches/mighty_z_range_narrow.patch`（`hw_mighty.yaml`）**：
只改了`z_min: 0.3→1.0`、`z_max: 5.0→2.0`（现在这两个数字所在的那一行
context是`mighty_local_frame_only.patch`早前从0.8/2.0改到0.3/5.0之后
留下的，这次patch在完整patch链里排在它后面，衔接对得上）。**特意只改
这两个参数**——`min_wdz`/`initial_wdz`（窗口初始高度）不用跟着改，
`readMap()`里down/up最终都会被`z_min`/`z_max`夹紧，这两个参数本身
没有约束力，改了也白改；`force_goal_z: true`+`default_goal_z: 1.5`
已经在生效（RViz点的目标点z本来就会被强制成1.5米），新区间`[1.0,2.0]`
正好把1.5米夹在正中间、上下各留0.5米——这个余量是留给正常悬停高度
控制的抖动空间，太紧会让`findAandAtime()`的A点越界检查
（`ok=0`直接失败）在正常飞行的小幅度高度波动下就被频繁误触发。

只涉及`flight-stack`镜像——`sim-world`镜像的Dockerfile压根不用
`hw_mighty.yaml`/`onboard_mighty.launch.py`（查过，两个字符串在
`Dockerfile.sim-world`和`sim-world-entrypoint.sh`里都搜不到），
这次的改动跟sim-world无关，没有加进它的patch链。已经对着flight-stack
完整18个mighty patch链跑过`git apply --check`，链上唯一失败的依旧是
跟这次改动无关的`mighty_disable_d435`既有冲突。**尚未rebuild验证。**

## 用户提出另一种处理"目标点被墙挡住"的思路：不是拒绝/重定位到最近free格子，而是沿着"当前位置→目标点"这条连线，取最靠近目标、但仍然可达的点作为实际规划目标

只是记录下来，还没有实现，也没有确认要不要现在就做——跟现有
`sanitizeTerminalGoal()`的处理方式不是一回事：现有逻辑是BFS找**欧氏
距离最近的**free/unknown格子（可能被推到任意方向，包括侧向或者更远
离飞机的方向），用户提的这个思路是**专门沿着飞机当前位置指向目标点的
那条直线**往回退，找线上最后一个可达点——直觉上更合理（贴着障碍物
靠近，而不是被推去一个跟原目标毫不相关的方向），但需要新写一段逻辑
（沿直线步进查`checkIfPointOccupied`，加安全余量往回退一点），不是
改一两个参数就能做到的量级。这次对话里已经有三个还没rebuild验证的
mighty代码改动（状态重同步、WALLHIT诊断、z方向天花板），暂时先不追加
第四个未验证的改动，等这批先rebuild验证过、看`z_min/z_max`收窄+
z方向天花板这两个组合能不能已经解决大部分撞墙场景，再决定要不要单独
做这个"沿线回退"的优化。

## 用户提问："控制律切换能在compose里切换吗？"——`docker-compose.yml`补上`CONTROL_LAW`

`CONTROL_LAW`当时只在`flight-stack-entrypoint.sh`里有默认值兜底
（`export CONTROL_LAW="${CONTROL_LAW:-trajectory}"`），`docker-compose.yml`
里一直没有暴露这个环境变量（对比`LOCALIZATION_SOURCE`那次就已经加了）。
照着`LOCALIZATION_SOURCE`的写法给`flight-stack-nx01`/`flight-stack-nx02`
两个service都加了`CONTROL_LAW=${CONTROL_LAW:-trajectory}`——用
`${VAR:-default}`这种compose插值写法而不是直接写死`trajectory`，
两种切换方式都支持：改`docker-compose.yml`里这一行的默认值、或者不改
文件、用`CONTROL_LAW=attitude docker compose up`临时覆盖。两架飞机
的这一行要保持一致（都是`trajectory`或都是`attitude`，不要一个一种），
跟`LOCALIZATION_SOURCE`要求两边保持一致是同样的道理。

直接编辑`docker-compose.yml`（不是staging/patch体系管的文件），过了
YAML语法校验。改完要重新`docker compose up`（如果只是改环境变量、
镜像本身没变，不需要重新`build`）才会生效。

## 用户追问：attitude模式用的是SO(3)还是SE(3)控制律？有没有需要调的本体参数（质量等）？

重新对着代码精确核对了一遍（之前1403行那次回答"用的是微分平坦方法"，
这次给了更精确的定性）：**严格说都不是**——`get_orientation()`/
`get_angular()`/`get_thrust()`（`dynus_offboard_node.py`）是
Mellinger & Kumar (2011)那一路的**微分平坦度前馈参考生成器**，不是
Lee/Leok/McClamroch (2010)意义上带误差反馈项的SE(3)几何跟踪控制律：
从期望加速度反解`z_B`、配合偏航向量`x_C`叉乘构造旋转矩阵（这一步用的
是SO(3)姿态表示的标准几何构造，转成四元数），推力从`|m(a+g)|`归一化，
角速度从jerk投影算出——**全程没有`e_R`/`e_Ω`误差项、没有`K_R`/`K_Ω`
反馈增益**，纯粹是"给定期望轨迹反解姿态"，不看当前实际姿态，真正的
闭环纠偏在PX4自己的机载姿态/角速度控制器里（C++固件代码，不在这段
Python范围内）。

**本体参数**：`m`/`hover_thrust`已经不是硬编码——重新apply了
`ros2_px4_stack_dynus.patch`（第一个/最底层的patch）确认，读的是
环境变量`VEHICLE_MASS_KG`/`VEHICLE_HOVER_THRUST`（entrypoint从
`config/vehicle_profile.yaml`解析出来export的，跟mighty用同一个
`mass=1.935`/`hover_thrust=0.67`），不用单独调。**但有一个真实缺口**：
`attitude`模式完全绕开PX4位置/速度环，只靠PX4自己最内层的姿态/角速度
控制器（`MC_ROLLRATE_P/I/D`、`MC_PITCHRATE_*`、`MC_YAWRATE_*`、
`MC_ROLL_P`、`MC_PITCH_P`、`MC_YAW_P`），这组增益从头到尾还是iris的
出厂默认值，从没针对mid360+相机负载之后的实际质量/转动惯量重新调过——
`trajectory`模式下问题不大（外层位置环兜底），`attitude`模式下这组
增益的好坏直接决定飞行质量，真测之前应该先看这里。

## 用户要求：列举真正开源的多旋翼SO(3)/SE(3)控制器

联网查的，只是调研记录，没有改动代码。基本都是从Lee/Leok/McClamroch
"Geometric Tracking Control of a Quadrotor UAV on SE(3)" (CDC 2010)
这篇论文衍生出来的开源实现：

- **跟本项目PX4/mavros接口直接兼容**：[Jaeyoung-Lim/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers)
  的`geometric_controller`包（mavros+PX4 OFFBOARD，明确引用上面那篇
  SE(3)论文，还考虑了旋翼阻力）；[KumarRobotics/kr_mav_control](https://github.com/KumarRobotics/kr_mav_control)
  的`so3_control`（宾大GRASP实验室，`so3cmd_to_mavros_nodelet.cpp`
  有mavros接口，支持过Snapdragon Flight/Crazyflie/PX4）。
- **仿真器自带参考实现**：[ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator)
  的`lee_position_controller`（RotorS，命名直接对应Lee的论文，很多
  论文对比实验的默认baseline）。
- **偏研究/教学向**：[fdcl-gwu/uav_geometric_control](https://github.com/fdcl-gwu/uav_geometric_control)
  （GWU，C++/Python/Matlab都有，带decoupled-yaw变体）、
  [plusk01/se3quad](https://github.com/plusk01/se3quad)、
  [HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller)
  （带ROS集成和PX4支持）、[malintha/multi_uav_simulator](https://github.com/malintha/multi_uav_simulator)
  （Mavswarm，内部用Lee的几何跟踪控制器，最多10架异构四旋翼swarm）。

这几个跟本项目现在`_pack_into_attitude()`的本质区别：都带真正的
`e_R`/`e_Ω`误差反馈项，会主动纠正当前姿态和期望姿态的偏差，不是纯
前馈算完就完事。以后如果要把attitude模式从"能发指令"升级成"真正带
反馈、验证过"，`mavros_controllers`的`geometric_controller`是最值得
照着改的模板（接口最贴近，同样是mavros+PX4）。

## rebuild之后用户反馈"NX01落地不飞了"——`z_min=1.0`这版收窄直接导致炸机，属于严重回归

**根因确认**：`findAandAtime()`里A点越界检查（`A.pos[2] < par_.z_min ||
> par_.z_max`）是**硬失败**——不是"超出范围就规划保守一点"，是直接
`return false`，连replan都不尝试。之前把`z_min`收到`1.0`（只给
`default_goal_z=1.5`留0.5米余量）时完全没考虑到：**起飞爬升阶段飞机
本来就要从地面（~0.1米）经过`z_min`以下的高度爬升上去**，以及正常
悬停也会有小幅高度波动——只要真实高度一跌破`z_min`，replan立刻失败、
飞机拿不到任何纠偏轨迹去主动爬升，只会继续掉，掉了更低于`z_min`，
死循环，最后落地。**实测锁定**：容器stdout里"A (-0.680668, 0.350345,
0.510162) is out of the map"，z从0.51一路降到0.42，一直失败，从没
恢复；世界坐标最终确认NX01落在地面附近。

**同一个bug对`z_max`是完全对称的**：本session早前（MPC_THR_HOVER
修复之前）实测过一次悬停爬升超调到3.6米，同样的硬失败逻辑一样会在
超过`z_max`时把飞机焊死——不是只有下边界危险，上边界收得太紧
（第一版`z_max=2.0`只给1.5米留0.5米余量）风险是对等的，之前那条
"z_max这个方向没有死循环风险"的注释判断错了，已经改写。

**修复：`patches/mighty_z_range_narrow.patch`重写**——`z_min`改回`0.3`
（这一路session里一直安全使用、覆盖得住起飞爬升的值），`z_max`放宽到
`4.0`（给1.5米悬停目标留2.5米余量，比实测过的3.6米超调还宽松）。
"挡翻墙"这类高度异常的活主要交给`mighty_z_window_ceiling_occupied.patch`
——那是跟着飞机当前高度走的"窗口相对"天花板（大约当前高度+2米，不是
写死的绝对值），触发的是能恢复的正常replan失败，不会像这两个全局硬
边界一样直接把飞机焊死，更适合当第一道防线。`z_min`/`z_max`收窄到
`[0.3,4.0]`仍然比原始`[0.3,5.0]`窄，但主要作用降级为兜底，不是主力
防线。

**教训记录**：改这种"硬边界/硬失败"类型的参数时，不能只看"稳态目标值
±一点余量"，必须覆盖真实会经过的全部状态空间（起飞爬升路径、正常
波动/超调），而且要检查这个参数是不是在系统里被用作*不可恢复*的失败
条件（这次是`return false`+外层没有任何针对"A越界"这个具体原因的
纠偏逻辑）——如果是不可恢复的，余量必须往宽了给，宁可防线松一点，
不能拿"飞机会不会摔"去赌一个刚好卡着目标值的窄区间。

对应重新过了完整18个mighty patch链的`git apply --check`，链上唯一
失败的依旧是无关的`mighty_disable_d435`既有冲突。**这次改动还没有
重新rebuild验证**——需要重新构建镜像后确认：1）起飞爬升不再触发这个
死循环；2）正常悬停/巡航不会意外撞到收窄后的`z_max=4.0`；3）之前
"翻墙"那个场景，`mighty_z_window_ceiling_occupied.patch`这道防线本身
能不能独立生效（不依赖`z_max`收得多紧）。

## 用户追问："机头跟得慢是不是PX4固件参数达不到？"——确认是，新增`patches/px4_iris_mpc_yawrate_max.patch`

查了PX4源码（`mc_pos_control/multicopter_autonomous_params.c`）确认：
`MPC_YAWRAUTO_MAX`（自动/offboard模式下偏航设定值变化速率的硬顶）
默认**45°/s**，注释写得很明确："Limits the rate of change of the yaw
setpoint to avoid large control output and mixer saturation"。这次
session之前只调过mighty自己的`w_max`（1.0→3.0 rad/s≈172°/s），PX4
固件这边这个参数从没被碰过——`mavros`的`setpoint_trajectory`插件会把
mighty的yaw/yaw_rate转发给PX4，但PX4收到之后不管mighty给多大，都会被
`MPC_YAWRAUTO_MAX`砍到45°/s封顶，比mighty现在愿意给的172°/s窄了将近
4倍，是当前真正卡脖子的那一层。

同时确认了这套仿真里改PX4固件参数的三种方式：1）QGC/mavros实时改
（`ros2 service call .../mavros/param/set`或者QGC参数面板，改了立刻
生效但重启就丢，不进这套patch追溯体系）；2）`param set-default`打进
`airframe`启动脚本、编译进镜像（`px4_iris_mpc_thr_hover.patch`这次
用的就是这种，持久、可追溯，但重新生效要rebuild）；3）
`vehicle_profile.yaml`+entrypoint导出环境变量（只对代码里已经接了
读环境变量的参数有效，`MPC_YAWRAUTO_MAX`没接这条路）。用户明确要求
直接固化，跳过QGC现场试参数那一步。

**新增`patches/px4_iris_mpc_yawrate_max.patch`**——照着
`px4_iris_mpc_thr_hover.patch`同样的写法，在`10015_gazebo-classic_iris`
airframe脚本里追加`param set-default MPC_YAWRAUTO_MAX 180`。选180
（不是刚好等于mighty的172°/s）是特意留了余量，让mighty自己的`w_max`
继续当那个真正生效、专门为这架飞机/这个任务调过的限制，而不是让PX4
固件默认值在172°/s附近又重新卡一道、变成新的隐藏瓶颈。参数合法范围
是`[5,360]`deg/s，180在范围内。

这个文件（`10015_gazebo-classic_iris`）是运行时直接从磁盘读的
（`sim-world-entrypoint.sh`里`-d "$PWD/ROMFS/px4fmu_common"`），不参与
编译，跟`MPC_THR_HOVER`那次一样，改这个patch不需要等PX4那部分慢编译
重新跑一遍。只涉及`sim-world`镜像（`flight-stack`的Dockerfile完全不
引用`PX4-Autopilot`，确认过），接在`px4_iris_mpc_thr_hover.patch`
后面同一个`RUN`块里，过了`git apply --check`验证。**尚未rebuild
验证实际效果**——需要重新构建`sim-world`镜像后现场核对：给一个需要
大幅转向的目标点，机头转向速度应该明显比之前快，但仍然不超过mighty
自己`w_max=3.0 rad/s`这个上限（不应该比之前"172°/s但被砍到45°/s"更
激进到失控的地步）。

## 用户反馈"机头从来不对准飞行方向、机尾反而对着飞行方向"——排查一圈，结论是软件/传感器都没有方向反转bug，大概率是视觉误判

**排查过程**（按怀疑顺序）：
1. mighty自己算目标偏航角的逻辑（`getDesiredYaw()`）：`atan2(next_goal-
   current_state)`，标准"朝向行进方向"写法，代入验证正确。
2. `_pack_into_traj()`用的微分平坦度四元数构造（`get_orientation()`/
   `get_drone_frame()`）：代入零加速度巡航场景手算了一遍，
   `x_B=[cos(yaw),sin(yaw),0]`跟mighty自己算出来的yaw完全对得上，内部
   自洽。
3. mavros的`setpoint_trajectory.cpp`（本地`/home/robots/repos/
   unified_autonomy_stack/.../mavros/src/plugins/setpoint_trajectory.cpp`
   有源码）：确认会做标准`ENU_TO_NED`四元数转换再提取yaw，这是mavros
   处理所有姿态话题的通用路径，广泛验证过，不太可能单独在这里出错。
4. PX4这架iris的电机布局（`CA_ROTOR0~3_PX/PY`）：标准X构型，PX4自己
   的控制分配矩阵认的也是"+X=前"，跟上面链路一致。
5. 用户提出"雷达装反了？"这个新方向——查了`gen_iris_mid360_sdf.py`里
   雷达的水平扫描角度`<min_angle>0</min_angle><max_angle>6.2831852
   </max_angle>`，是完整360°，雷达朝向即使装偏了也不会造成"前方看不见"
   这种盲区（横向本来就全覆盖），但确实可能让DLIO内部把"哪个方向算
   yaw=0"这件事标记错。

**最终用实测数据实锤，不是猜的**：现场同一时刻分别取了Gazebo真值
（`/plug/model_states_plug`）和DLIO估计（`dlio/odom_node/odom`）的
姿态四元数，都换算成yaw角对比：**真值38.56° vs DLIO估计38.97°，
差0.41°，几乎完全重合，没有180°翻转**。这直接证明：DLIO姿态估计是
准的，雷达没有装反；结合前面1-4条已经确认的指令链路自洽性，**飞机
真实朝向、DLIO估计朝向、mighty/mavros/PX4指令链路预期朝向，三者
互相吻合，没有任何一处存在方向反转的bug**。

**推断"看起来朝向反了"的真实原因是视觉误判，不是bug**：用户是靠
"RViz中坐标系方向"和"Gazebo中桨叶颜色"两个线索判断的——桨叶网格
`iris_prop_ccw.dae`/`iris_prop_cw.dae`区分的是**转向（顺/逆时针），
不是前后位置**，标准X构型下同色桨叶是按对角线分布的（前左和后右
同色），机身本体`iris.stl`也是单色无标记，**这套模型本身没有可靠的
"机头"视觉标记**，凭桨叶颜色判断前后本身就不成立；RViz那边则可能是
看错了具体是哪个frame的坐标轴（这套系统同时存在`{ns}/base_link`、
`{ns}/init_pose`、`{ns}/lidar`等好几个frame）。建议之后在RViz里专门
找`{ns}/base_link`这个frame的Axes显示（红色=+X），拿它的指向跟实际
飞行方向做肉眼比对，应该是吻合的。

这一条没有产生任何代码/patch改动——纯排查，结论是"不用修"。记录下来
避免以后同一个问题被重新怀疑、重新排查一遍。

## 用户反馈"yaw跟运动方向的偏差是持续性的"——继续查mavros/PX4源码排除更多可能，同时新增status监控窗口方便后续采集数据

**继续排查mavros的ENU→NED四元数转换**（`ftf_frame_conversions.cpp`）：
```cpp
case StaticTF::ENU_TO_NED:
    return NED_ENU_Q * q;   // 只转换世界参考系，没转换机体系FLU→FRD
```
一开始怀疑这里漏做机体系转换（`AIRCRAFT_TO_BASELINK`）是bug，代入四元数
乘法公式手算了一遍标准yaw提取公式`atan2(2(wz+xy), 1-2(y²+z²))`在这个
"缺失"变换前后的值——**代数上证明完全不受影响**（绕前向轴180°旋转不
改变前向轴本身指向，只互换左右/上下，提取出来的yaw分子分母不变）。
这是mavros的无害简化，不是bug。

**后台agent专门查了PX4 v1.16的offboard yaw处理**（`mavlink_receiver.cpp`
第1109行、`PositionControl.cpp`第104行）：确认yaw字段从MAVLink消息到
最终姿态设定值全程原样透传，没有任何符号变换；`MPC_YAW_MODE`那套
"朝向waypoint"逻辑经`FlightModeManager.cpp`确认完全不会在OFFBOARD模式
下被触碰（只在Auto/Mission生效）。

**结论：mighty的atan2计算、微分平坦度四元数构造、mavros的ENU_NED转换、
PX4的yaw透传，逐层代数/源码验证下来都是对的**，加上前面已经用实测数据
证明DLIO姿态估计准确（跟真值差0.41°）——**软件链路目前查不出静态的
符号反转bug**。用户确认这个偏差是持续性的（不是过冲/瞬态问题），
这条线索目前还没有定论，需要更多实测数据支撑，先按下不表。

**新增`docker_sim/scripts/status_monitor.py`+改`watch_sim.sh`，方便后续
采集数据**：用户反馈DLIO自己的控制台状态面板（`[dlio_odom_node-N]`
前缀，Ang Velocity/Accel Bias/Registration/GICP等一大堆内部调试字段）
刷新太频繁，把NX01/NX02两个tmux窗口刷屏刷得看不了别的日志——实测确认
（最近300行日志里299行都是这个前缀，几乎独占整个输出）。

- **NX01/NX02两个窗口**：`docker compose logs`后面接
  `grep --line-buffered -v '\[dlio_odom_node'`过滤掉这部分输出，DLIO
  本身继续正常跑、继续发布话题，只是不再刷这两个窗口。过滤之后顺带
  发现一个之前被完全淹没、看不见的问题：`mavros`在周期性报"TM: Time
  jump detected. Resetting time synchroniser."（大概每51秒一次）——
  这次没有深挖，先记录下来。
- **新增第四个窗口`status`**：跑`status_monitor.py`（新文件，纯监控
  工具，不参与staging/patches编译流程，`watch_sim.sh`每次用`docker cp`
  现塞进容器，不用重新build镜像），用rclpy订阅`<ns>/dlio/odom_node/odom`，
  1秒刷新一次，输出位置、姿态（四元数转欧拉角，度，跟排查yaw问题时
  用的是同一套标准ZYX航空约定公式）、发布频率（滑动窗口算，30帧）、
  内存（`ps -eo rss=`对当前容器所有进程求和，`docker exec`天然加入
  目标容器自己的PID namespace，不需要额外配置）。NX01/NX02两架飞机的
  输出用`[NX01]`/`[NX02]`前缀交替打印在同一个窗口。实测跑通：
  `hz=100.0`、`mem=2187MB`，位置/欧拉角正常刷新。

`watch_sim.sh`改动已实测验证（现场跑过、确认DLIO过滤生效、status窗口
输出格式正确），不是靠猜的。这个新窗口接下来会用来采集"yaw持续性偏差"
问题的实测数据，继续排查。

## 用户反馈"UP了，NX02飞机飞着飞着就落地了"——rebuild之后又一次悬停掉高度崩溃，这次排除了之前最大的嫌疑，根因还没坐实

**现场排查过程**：`mighty_debug.log`显示从记录一开始（t=0.017s，飞机
已经在悬停高度z≈1.48米）就持续、平滑地掉高度，6.3秒内掉到地面，此后
`REPLAN`永久`ok=0`。曲线形态上跟本session早前MPC_THR_HOVER没修之前的
那次悬停推力不足是同一种模式，但这次先验证了几个之前的嫌疑，结果都
排除了：

1. **`MPC_THR_HOVER`确认是活的、正确的**：直接用mavros的
   `param/get`服务查当前PX4实例的实时值，返回`0.671`，跟
   `px4_iris_mpc_thr_hover.patch`设的值完全一致——不是被持久化的旧
   参数覆盖（这是本来担心的情况：PX4的`MPC_USE_HTE`在线悬停推力估计器
   默认开着，如果参数存储跨容器重启持久化，可能会用之前学习到的旧值
   覆盖新的`set-default`默认值；实测确认没有这个问题）。
2. **排除了明显碰撞**：`WALLHIT`在崩溃起始时刻（t≈0.02s）只命中
   `traj_idx=0`——这正是之前确认过的假阳性模式（`WALLHIT`检查用的
   `getMapUtil()`是原始底图，跟`freeStart()`实际清理过的规划用地图
   `map_util_for_planning_`口径不一致，不是真的从一开始就撞在障碍物
   里），不是新的真实碰撞证据。

**这次没能坐实根因，只有两个待验证的猜测，记录下来留待下次现场复现
时验证**：
1. PX4的`MPC_USE_HTE`在线估计器可能因为别的原因（EKF融合抖动、加速度
   计噪声）把估计值从正确的种子值`0.671`往错误方向拉偏——种子值本身
   没错，不代表实际飞行时用的推力值一直没错，这是两回事。
2. 这次也是本session里第一次把`MPC_YAWRAUTO_MAX`从45°/s提到180°/s
   之后跑的真实飞行——更激进的偏航速率有没有可能跟位置/姿态控制环
   产生耦合、引发更大幅度的机动扰动，让本来余量就不算特别宽裕的推力
   控制跟不上——只是时间上巧合，没有验证过是否真的相关。

**下次复现时该查什么，先记下来**：这次只查了mighty这边的位置日志，
没有抓PX4自己的姿态/推力遥测（比如`ATTITUDE_TARGET`里的实际thrust
输出，或者执行器输出）——光看位置日志只能看到"结果"（掉高度了），
看不到"过程"（推力到底给没给够、姿态有没有发散）。下次崩溃时应该
现场用`ros2 topic echo .../mavros/setpoint_raw/target_attitude`或者
类似话题，跟位置日志对照着看，才能真正区分开是HTE估计器的问题还是
姿态控制耦合的问题。

## 现场抓PX4姿态/推力遥测：两架飞机都是"推力打满+追一个够不着的偏航目标"，把`MPC_YAWRAUTO_MAX`退回45°/s止血

**新增`docker_sim/scripts/attitude_thrust_logger.py`**——持续订阅
`mavros/setpoint_raw/target_attitude`（PX4自己算出来打算怎么飞：姿态
四元数+机体角速度+推力）和`mavros/local_position/pose`（飞机实际当前
姿态，来自PX4自己的EKF），两者一起写进`/tmp/attitude_thrust_debug.log`，
带秒级时间戳，跟`mighty_debug.log`风格保持一致。用`docker cp`塞进
两个容器、`docker exec -d`后台跑，不用重新build。

**现场抓到的数据，两架飞机同时出现同一个模式**：
- NX01：`thrust=1.000`（推力打满）、`target_rpy=(+6.1,+16.4,+179.8)`，
  实际偏航是`-39.4°`，跟目标差一大截；`body_rate.z=-3.49 rad/s`
  （≈-200°/s，逼近当时`MPC_YAWRAUTO_MAX=180°/s`这个新上限）。
- NX02：`thrust=1.000`，`target_rpy=(-15.5,-8.2,+79.1)`，实际偏航
  `+46.8°`，差约32°，同样推力打满。
- 两架都是连续多个样本数值几乎不变——持续状态，不是瞬间抖动。

**没法确认这是不是"落地的原因"，但时间线上关联很强，先止血**：PX4只要
还armed+OFFBOARD就会一直算控制律，哪怕飞机已经蹲在地上动不了，所以
"推力打满、狂追一个够不着的偏航目标"有可能是已经炸机之后PX4还在徒劳
纠正的结果，不一定是导致炸机的原因——这次没能彻底分清楚因果。但这几次
连续炸机，都发生在当天把`MPC_YAWRAUTO_MAX`从45°/s提到180°/s之后：
如果本来就存在一个"偏航目标算错/追不上"的潜在bug（前面已确认是持续性
的，不是过冲瞬态），45°/s的时候这个bug只会让飞机慢悠悠地朝错的方向
转，不痛不痒；180°/s让PX4有权限更激进地去追这个本来就有问题的目标，
可能把一个温和的问题逼成了推力打满、机体角速度打满的剧烈失控。

**修复：`patches/px4_iris_mpc_yawrate_max.patch`重写，`MPC_YAWRAUTO_MAX`
从180退回45**（PX4出厂默认值）——纯粹是稳定性优先的止血措施，**不是
真正修好了偏航目标错误这个根因**，patch注释里写清楚了"这个数字不该
再往上调，除非底层yaw bug先查出来修好"，避免以后又不知道为什么被
重新调高。改这个patch的时候又犯了一次跟`mighty_z_range_narrow.patch`
那次同样的错误（对着已经打过旧180版本patch的索引做增量diff，而不是
对着"只打了`mpc_thr_hover`"这个正确的基线做完整diff，导致生成的patch
不是自包含的，链上第二步应用失败）——已经重新按正确流程走了一遍
（先`git apply thr_hover` + `git add`，再在这个基线上直接写出完整的
45版本内容，重新diff），过了两步都成功的链式`git apply --check`验证。

只涉及`sim-world`镜像（PX4只在这里build），`attitude_thrust_logger.py`
不参与staging/patches编译流程，纯监控工具。**`MPC_YAWRAUTO_MAX`退回
45的效果尚未rebuild验证**（需要重新构建`sim-world`才生效），
**"推力打满追不上的偏航目标"这个根因也还没查出来**——下次复现时应该
先看这套新的姿态/推力日志，从飞机还在正常飞、没崩溃的阶段就开始追，
才能分清楚是不是这个偏航目标计算从一开始就有问题、还是崩溃之后的
连锁反应。

## 用新装的姿态/推力日志逮到一次NX02"贴地又爬起来"的完整过程——找到`get_drone_frame()`里一个真实的数值不稳定bug，还没修

**完整还原了这段时间线**（`attitude_thrust_debug.log`）：
- t=353~361s：正常飞行，`thrust≈0.80`，目标姿态和实际姿态吻合，偏航
  稳定在30.5°左右。
- t=361.2s：目标偏航突然从30.5°跳到-16.1°，推力开始上升。
- t=363.5s起：`thrust=1.0`打满，**目标偏航角开始剧烈震荡**——171.9°→
  -145.0°→-105.6°→-61.0°→-18.0°→+25.0°→+68.0°……每0.25秒左右跳动
  几十度，横跨接近360°范围，持续15秒（到t≈376s）。同一时段高度从
  1.05米一路掉到**-0.69米**（贴地）。
- t=376.25s：震荡突然停止，目标偏航锁死在一个固定值不再变化。
- t=381.7s起：高度从-0.68米开始回升，推力从1.0降回0.67左右，一路爬回
  1.5米稳定悬停，目标和实际偏航重新收敛。

**目标偏航角剧烈乱跳，跟高度骤降/推力打满的时间窗口完全重合，震荡一停
就立刻开始恢复**——这个模式指向`get_drone_frame()`（`dynus_offboard_node.py`，
`_pack_into_traj()`/`_pack_into_attitude()`共用）里一处缺少数值保护
的地方：

```python
t = np.array([[ax, ay, az + g]]).T
z_B = t / np.linalg.norm(t)
```

`z_B`是机体Z轴（推力方向）在世界系下的表示，算法上等于"期望合力方向
归一化"。**如果期望加速度恰好让`az≈-g`（接近自由落体，垂直净力趋近于
零）且`ax`、`ay`也很小，`t`这个向量的模长会趋近于零，除以一个接近零
的数会让`z_B`对输入的极小扰动变得极度敏感、剧烈抖动**——而`z_B`是后面
`y_B`/`x_B`（进而整个姿态四元数、偏航角）计算的起点，`z_B`一乱，整个
姿态跟着乱，正好解释了实测到的偏航震荡。飞机确实在掉高度、经历接近
自由落体的过程，这个数值不稳定的窗口被真实触发了。

结合早前"持续性偏航偏差"那次排查（软件链路逐层验证都是对的）：现在
的理解是，这不是一个恒定的符号错误，是**间歇性的数值不稳定，专门在
接近自由落体的时刻发作**——这也解释了为什么之前提高`MPC_YAWRAUTO_MAX`
会让情况更糟：相当于给了PX4更大权限去追一个正在剧烈乱跳的目标。

**这次没有修，只是定位到了根因**——用户先去处理"起飞口令"这个功能，
`get_drone_frame()`要不要加`norm(t)`下限保护（比如小于某个阈值时
fallback成`z_B=[0,0,1]`或者沿用上一次的值）还没决定，留到下次。

## 用户要求：`docker compose up`不再自动起飞，tmux新增"起飞口令"窗口，按回车才起飞

**新增`patches/ros2_px4_stack_takeoff_gate.patch`**（`dynus_offboard_node.py`
的`takeoff_and_track_trajectory()`）：TAKEOFF阶段新增一个文件门槛
`/tmp/takeoff_go`——文件不存在时，setpoint钉在**当前位置**（不是目标
高度）循环发送，不会真的往上爬；但仍然持续发setpoint流，不是完全卡住
不发——PX4进OFFBOARD模式前必须已经有一路连续setpoint在发，如果这里
完全不发，反而会让`_kick_offboard()`的自动解锁/切OFFBOARD逻辑永远等
不到条件，起飞会更晚而不是更安全。文件一出现，立刻切换成原来的起飞
爬升逻辑，后续行为不变。

**新增`docker_sim/scripts/launch_control.sh`**——tmux新增的第一个窗口
"launch"，循环提示、按回车后给两个flight-stack容器都`docker exec touch
/tmp/takeoff_go`，飞机检测到文件就开始爬升。放在session的第一个窗口
（`watch_sim.sh`里`tmux new-session`直接建的就是这个），attach进去
默认就停在这里，不会漏按；状态栏颜色定为红色，提醒"这里要先操作"。

`watch_sim.sh`同时顺手把`attitude_thrust_logger.py`的自动部署也塞进了
`status`窗口的启动命令里（之前是手动`docker cp`+`docker exec -d`临时
挂的，现在跟`status_monitor.py`一样，每次`up_and_watch.sh`/`start.sh`
起来就自动带上，不用再手动补）。

只涉及`flight-stack`镜像（`ros2_px4_stack_takeoff_gate.patch`需要
rebuild才生效），`launch_control.sh`/`watch_sim.sh`是纯脚本改动，不
参与镜像构建。patch已经过完整8个ros2_px4_stack patch链的`git apply
--check`验证。**尚未rebuild验证实际效果**——需要重新构建`flight-stack`
镜像后确认：1）`docker compose up`之后飞机确实原地悬停不爬升；2）
`launch`窗口按回车后两架飞机确实开始起飞；3）悬停等待期间`_kick_offboard()`
的自动解锁/切OFFBOARD不受影响，飞机应该已经armed+OFFBOARD，只是不爬升。

## 用户要求：把`get_drone_frame()`的`norm(t)`数值不稳定bug修掉

**新增`patches/ros2_px4_stack_zb_norm_guard.patch`**（`dynus_offboard_node.py`）：

```python
t_norm = np.linalg.norm(t)
if t_norm < 1.0:
    z_B = np.array([[0.0], [0.0], [1.0]])   # fallback：接近自由落体时直接当成"朝上"
else:
    z_B = t / t_norm
```

两个设计选择：1）fallback选`[0,0,1]`（朝上/纯悬停姿态），不是"沿用
上一次的值"——后者需要跨调用保存状态（这个函数目前是无状态纯函数），
引入状态本身是新的复杂度/潜在bug源（多线程读写要不要加锁），而且"上
一次的值"不一定可靠（如果飞机已经开始翻滚，历史姿态也可能是坏的）；
"朝上"是最保守、最可预测的选择。2）阈值定为1.0——正常悬停/飞行时
`t`的模长稳定在9.81附近（`ax`/`ay`/`az`已经有±0.1的清零阈值滤掉里程计
噪声），只有垂直加速度指令真的接近-9.81（命令"自由落体"）才会触发，
留了足够余量，不会误伤正常机动。

**这个fix明确解决什么、不解决什么**（已经跟用户说清楚，避免预期过高）：
- **会**：接近自由落体的瞬间，姿态/偏航计算不再对输入的极小扰动剧烈
  敏感、乱跳，理论上能大幅缩短甚至避免类似"贴地又爬起来"那次15秒
  剧烈震荡事故的持续时间。
- **不会**：不修复导致飞机一开始进入接近自由落体状态的根本原因（是
  mighty规划出的加速度指令太猛，还是PX4推力/位置环跟不上，还没查）。
- **不确定**：这个fix能不能解释更早之前反馈的"持续性偏航偏差"——那次
  观察可能发生在正常飞行阶段（不一定是接近自由落体的瞬间），只是这一
  种已经在日志里实锤过的退化场景对症下药，不是全面修复。

对应过了完整9个ros2_px4_stack patch链的`git apply --check`验证（新增
在`ros2_px4_stack_takeoff_gate.patch`之后），并额外做了Python语法校验
（`ast.parse`）确认改动后的文件语法正确。只涉及`flight-stack`镜像。
**尚未rebuild验证实际效果**——需要重新构建`flight-stack`镜像后，最好
能人为制造一次接近自由落体的场景（比如让飞机贴近撞击/被逼停），用
`attitude_thrust_logger.py`确认目标偏航角不再剧烈震荡。

## rebuild之后现场验证：三个修复全部确认生效；给两架飞机各点了一个目标点，实测又挖出`get_drone_frame()`的第二处数值不稳定点，已修

**验证三个已知修复**：`z_B`保护、起飞口令、`MPC_YAWRAUTO_MAX=45`都直接
在新容器里现场核对过——`grep`确认代码存在、`/tmp/takeoff_go`不存在时
飞机确实armed+OFFBOARD但悬停不爬升（z≈0.05米）、`ros2 param get`确认
`MPC_YAWRAUTO_MAX`实时值是45.0。

**监控部署时发现一个小问题并修正**：`watch_sim.sh`新版本已经把
`attitude_thrust_logger.py`的自动部署整合进`status`窗口，用户用这个
新版本起来之后，我又手动`docker exec -d`了一遍，导致两个容器里各自
跑了两份重复的记录进程，写进同一个日志文件、但`t_rel`时间基准不一样，
数据会交替错乱——现场用`ps aux`发现、杀掉了多余的那份，清空日志重新
开始记录。

**用`ros2 topic pub`代替RViz点击给两架飞机各发了一个目标点**：NX01
第一次发的目标点（局部系`(7,-3)`）被`sanitizeTerminalGoal`拒绝了——
现场查`docker logs`确认是"goal is occupied and no free/unknown cell
found within BFS budget"——回头算了一下：NX01的`INIT_X=3`，局部
`(7,-3)`换算成世界坐标是`(10,-3)`，正好砸在东墙里，是我自己挑测试
坐标时没考虑到每架飞机的局部系偏移，不是系统的bug。换成局部`(4,-3)`
（世界坐标`(7,-3)`，房间内）之后正常规划、正常飞行。

**两架飞机都成功飞到目标点，但飞行过程中各出现过6次短暂的`thrust=1.0`
峰值**——比之前那次15秒持续震荡好得多（几个样本内就消退，没有演变成
坠落），但`target_rpy`确实还是有过0.5秒内跳动几十度的情况（NX01从
-12.6°→-62.5°→-105.5°，NX02从-84.8°→-35.0°→+51.2°），跟第一处
`t_norm`保护针对的"接近自由落体"场景不太一样，比对`target_tilt_from_level`
数值不算特别极端（14°~105°，不像上次接近180°），推断可能是
`get_drone_frame()`里另一处结构相同的数值不稳定点：

```python
cross_prod = np.cross(z_B.T[0], x_C.T[0])
y_B = cross_prod / np.linalg.norm(cross_prod)
```

`z_B`（期望推力方向）和`x_C`（水平偏航参考方向）都是单位向量，
`norm(cross_prod)`等于两者夹角的正弦值——只有当`z_B`几乎跟`x_C`平行
（也就是机体接近90度极限倾角、几乎贴着偏航方向侧飞）时才会趋近于零，
除法同样会让`y_B`/`x_B`（进而整个姿态）对输入的极小扰动剧烈敏感。

**新增`patches/ros2_px4_stack_yb_cross_norm_guard.patch`**：思路跟第
一处一致但fallback策略不同——这里没有"朝上"这种天然安全的默认值，
改成当`cross_norm<0.1`（对应`z_B`和`x_C`夹角约6度以内）时，换一个
固定的备用参考向量（世界Y轴`[0,1,0]`）重新算叉积，代替原来的`x_C`。
只在这个极窄的边缘场景损失精确偏航跟踪，但能保证算出来的姿态坐标系
始终数值稳定。

对应过了完整10个ros2_px4_stack patch链的`git apply --check`验证，
外加Python语法校验，接在`ros2_px4_stack_zb_norm_guard.patch`之后。
只涉及`flight-stack`镜像。**这次也还没rebuild验证**——两处保护加完
之后，需要再跑一次实测，看这几次短暂的推力打满/姿态跳动会不会也
消失。

## rebuild之后又炸机：NX01/NX02先后"趴地上"——这次不是`get_drone_frame()`的问题，是`mighty.cpp`里第三处同类数值不稳定点，根因坐实并已修

**现象**：两处`get_drone_frame()`保护（`z_B`的`t_norm`保护、`y_B`的
`cross_norm`保护）rebuild验证通过、两架飞机都成功起飞爬升到目标
高度、稳定悬停几百秒之后，用户先后报告"NX02又趴地上了"、
"NX01也趴地上了"。说明之前两处修复没有覆盖全部场景，得重新现场
抓数据。

**取证方法**：`docker cp`取出NX02的`/tmp/attitude_thrust_debug.log`
（2605行，覆盖起飞到坠毁后共675秒），用Python脚本解析出
`(t, thrust, target_rpy, actual_pos)`时间序列，先粗筛（每5秒抽样）
定位异常发生的大致时间窗，再在该窗口内逐行细看。同时用
`docker exec ... ps -eo pid,etimes` + 当前`date +%s`反推
`attitude_thrust_logger.py`进程的启动墙钟时间（`t0`是`CLOCK_MONOTONIC`
基准，日志里的`t`是相对`t0`的秒数），把日志里的可疑时刻换算成
UTC绝对时间，再用`docker compose logs -t`按这个UTC时间窗过滤出
`mighty`自己的详细日志，两边对照。

**发现的现象序列**（NX02，`t≈568.6s`开始，UTC约`13:02:45`）：

1. 悬停在`z≈1.48m`已经稳定了350多秒（`t≈218`到`t≈568`），`thrust≈0.80`，
   `target_rpy`长期贴着`(0,0,0)`附近的小值，一切正常。
2. `13:02:46.125`，`mighty`日志打出
   `Changing DroneStatus from status_=GOAL_REACHED to status_=TRAVELING`
   ——也就是刚才悬停在起飞点等待的状态结束，开始真正往目标点飞。
3. 几乎同一时刻（`t=568.584`），`thrust`从`0.80`掉到`0.746`，
   `target_rpy`的yaw分量从`-0.0°`跳到`-30.7°`；接下来的每个
   ~0.25秒采样点：`-74.8°→-115.0°→-159.7°→+162.5°(过零点折返)
   →+112.7°→+73.2°→+32.0°→-9.2°→-45.4°→...`——**yaw在几秒内几乎
   转了整整一圈**，`body_rate.z`（偏航角速度）持续钉在`±3.49rad/s`
   （约200°/s，是这套控制链里能跑到的速率上限），roll/pitch也跟着
   剧烈摆动，`thrust`很快打满到`1.000`。
4. 姿态失控的同时高度断崖式下跌：`z`从`1.48m`（`t=568`）掉到
   `-0.19m`（`t=573.7`，已经穿地），5秒之内摔到地上。
5. 摔地之后，`mighty`的`findAandAtime`（本session早前加的A点/真实
   状态偏离检测）开始疯狂报"resyncing"——规划器以为飞机还在飞、
   A点持续按原计划往前跑，但飞机已经趴在地上不动，每个周期都超过
   `2.02m`的"合理跟踪误差"阈值、resync、下个周期又超——30秒后
   `mighty`自己认输："Replanning has been failing for 30.01s
   (cap: 30s) -- giving up on the current terminal goal"。这一段是
   坠机之后的**连锁反应**，不是根因。

**根因定位**：`GOAL_REACHED→TRAVELING`切换的瞬间，飞机还没开始
真正移动，`mighty.cpp`里`getDesiredYaw()`对`TRAVELING`/`GOAL_SEEN`
状态用的偏航公式是：

```cpp
desired_yaw = atan2(next_goal.pos[1] - local_state.pos.y(),
                     next_goal.pos[0] - local_state.pos.x());
```

`next_goal.pos`是局部样条上**下一个采样点**（离当前位置通常只有
几厘米，不是终点目标`G_term`），这个向量在刚从悬停切到飞行的那一刻
几乎为零向量——`atan2`对着一个模长趋近于零的向量求角度，结果对
`dx`/`dy`里的噪声极度敏感：本来应该平滑变化的偏航角，会在相邻两个
控制周期之间（100Hz、`dc=0.01s`）从随便一个角度跳到随便另一个
角度，表现出来就是`target_rpy`的yaw分量疯狂扫圈。这跟已经修过的
`get_drone_frame()`里`z_B`（对近似自由落体的加速度求归一化方向）、
`y_B`（对近似平行的叉积求归一化方向）**是同一类bug**——都是"对一个
模长趋近于零的向量求方向"，只是这次出在`ros2_px4_stack`上游、
`mighty`规划器自己算偏航目标的地方，不在`get_drone_frame()`里。

也解释了本session更早时候排查过很久、当时没有坐实根因的"机头不对准
飞行方向"现象、以及`MPC_YAWRAUTO_MAX`调高到180后必炸机——飞机一直
在追一个每个周期都可能整体反向的偏航目标，调高跟踪速率只会让它更快
地把这种噪声跟出来，表现成剧烈震荡甚至炸机；调回45°/s只是把追踪
速度限制住、缓解了症状，没有碰到这个真正的根因。

**新增`patches/mighty_traveling_yaw_zero_dist_guard.patch`**：给这处
`atan2`加上前面两处一样的最小距离保护——`next_goal.pos`到
`local_state.pos`的距离低于`0.05m`时，不信任这个方向，直接沿用
`previous_yaw_`（跟`getDesiredYaw()`里`local_plan.size() < 5`分支已经
在用的兜底策略一致）：

```cpp
const double dx = next_goal.pos[0] - local_state.pos.x();
const double dy = next_goal.pos[1] - local_state.pos.y();
desired_yaw = (std::hypot(dx, dy) > 0.05) ? atan2(dy, dx) : previous_yaw_;
```

`flight-stack`和`sim-world`两个镜像里`mighty.cpp`的patch链前置内容
不完全一样（一个走硬件相关的一串patch，一个走仿真世界/RViz一串
patch），但这处改动落在两条链公共的、都没有再被后续patch改过的
代码上——分别在两条完整链的staged-baseline上验证过
`git apply --check`全部通过，两个Dockerfile都已经接入这个patch
（在各自链的末尾）。**这次也还没rebuild验证**——NX01/NX02两架飞机
先后炸机都发生在同一个`GOAL_REACHED→TRAVELING`切换点，根因定位
比较有把握，但仍需要重新build、up、现场起飞验证这处修复后
`TRAVELING`切换瞬间的偏航是否还会扫圈。

## rebuild之后现场验证：`mighty_traveling_yaw_zero_dist_guard.patch`修复确认生效——NX01被连续触发近40次`GOAL_REACHED↔TRAVELING`切换，全部平稳，无一次异常

**验证方法**：rebuild、`docker compose up`之后两架飞机都只是起飞爬升
悬停在起飞点（`TAKEOFF`保持阶段——`track_dynus_traj_py`要求先收到过
一次真实轨迹点才会转出这个状态，而`mighty`在没收到目标点之前一直
停在`GOAL_REACHED`，不会主动生成轨迹），得自己发`/NX01/term_goal`、
`/NX02/term_goal`才能真正触发到`TRAVELING`——之前一直炸机的那个
切换点。用`ros2 topic pub --once`发NX01的目标点时连续两次都没反应
（`ros2 topic pub --once`退出太快，跟`mighty_node`订阅方之间DDS
发现/匹配没来得及完成，消息实际没送达——这是`ros2 topic pub --once`
在临时进程场景下的已知坑，不是系统bug），换成`ros2 topic pub -r 5`
连续发布几秒再退出解决。

**副作用反而成了压力测试**：连续以5Hz重复发布同一个目标点，让
`mighty`在几秒内被反复当成"收到新目标"，来回切换了近40次
`GOAL_REACHED→TRAVELING→GOAL_SEEN→TRAVELING→...`——每一次都精确
命中修复前必炸机的那个瞬间。`docker cp`取出的姿态遥测显示这近40次
切换全程`thrust`稳定在0.80附近、`target_rpy`的roll/pitch摆动不到
1度、yaw稳定在-4.8°左右完全没有扫圈，飞机平稳飞到目标点附近
`(1.24,-1.14,1.34)`（目标是`(1.5,-1.0,1.5)`）。NX02同样成功飞到
目标点附近，遥测里唯一的`thrust=1.000`集中出现在起飞爬升阶段
（`z`从0单调爬升到1.19m，roll/pitch全程摆动在±2度内、平滑变化），
是本session早前已经确认过的"正常爬升需要接近满油门"良性现象，跟
坠机时那种`target_rpy`几秒内扫近360度、伴随`body_rate`钉在速率
上限的震荡模式完全不同。

两架飞机全程没有一次进入姿态震荡、没有一次高度骤降，`mighty_traveling
_yaw_zero_dist_guard.patch`这处修复确认有效。

## 用户提问："偏航角速度太大会因此损失升力吗？"——机制澄清：不是偏航角速度本身线性偷走升力，而是偏航目标失真通过两条路径间接导致升力损失

纯粹的高偏航角速度本身不消耗升力：只要`z_B`（机体推力方向）保持
竖直，绕`z_B`自转多快都不影响推力的竖直分量。真正的问题是这次
`getDesiredYaw()`的bug导致偏航目标在相邻控制周期间(100Hz)几乎
随机跳变，姿态控制器在追这种目标时，通过两条路径吃掉升力：

1. 四元数姿态误差在追随机跳变目标时通常不是纯偏航误差，会耦合出
   滚转/俯仰修正——实测遥测里`actual_rpy`（真实姿态，不是target）
   确实也在摆动。真实机体倾斜后，推力矢量偏离竖直，竖直分量（真正
   的升力）= 推力 × cos(倾角)，倾角越大升力损失越明显。
2. 四旋翼的偏航力矩本来就是三个转动轴里权限最小的一个（靠桨叶反
   扭矩差产生，比roll/pitch靠力臂产生的力矩小一个数量级）。电机
   混控器同时满足"巨大偏航力矩+滚转/俯仰修正+总推力"时容易饱和：
   为凑出偏航所需差速力矩，部分电机被顶到最大转速、没法再抬高共模
   转速，实际能输出的总推力就低于指令值——这也是实测里`thrust`已经
   钉在1.0却仍然掉高度的原因之一，不是电机不够力，是差速力矩需求
   挤占了本该用来抬升的共模空间。

## 用户反馈"NX01给点后摇头/走一半又退回原地，NX02也有奇异处"——排查发现是我自己验证用的`ros2 topic pub -r 5`残留进程没被真正杀掉，不是新bug

**现象**：`mighty_traveling_yaw_zero_dist_guard.patch`验证通过之后，
用户在RViz给NX01新的目标点，飞机要么原地转几下头不挪窝，要么顺着
规划轨迹走个几十厘米又掉头退回原地；NX02飞轨迹正常，但日志里有
"奇异处"。

**排查过程**：`docker compose logs`发现NX01在10分钟内`Changing
DroneStatus`打印了6020次（约10次/秒的高频flapping），且
`setTerminalGoal()`（`mighty.cpp:1793`）只要收到一条新的`term_goal`
消息就无条件`changeDroneStatus(TRAVELING)`——这种频率的状态切换
只可能是有什么东西在持续、高频地重新发布`/NX01/term_goal`。用
`docker exec docker_sim-flight-stack-nx01-1 ps aux`一查，果然找到
一个存活了25分钟的`ros2 topic pub -r 5 /NX01/term_goal ...`进程，
正是本session早前验证`mighty_traveling_yaw_zero_dist_guard.patch`
时手动起的那条命令（当时用`timeout 4 docker exec ... "ros2 topic
pub -r 5 ..."`，超时后shell显示"Terminated"，但那只杀掉了`docker
exec`客户端进程，不会把信号传进容器的PID命名空间杀掉里面真正在跑
的`ros2 topic pub`——这是`docker exec` + `timeout`组合的一个经典坑）。
这个残留进程一直以5Hz往`/NX01/term_goal`灌验证时用的旧目标点
`(1.5,-1.0,1.5)`，用户在RViz给的新目标点每次都在~200ms内被这条
旧目标覆盖回去，表现就是"刚要走就被拽回旧点"。

顺着这条线查NX02的"奇异处"：日志被
`TF_OLD_DATA ignoring data from the past for frame NX01/pose_af at
time 0.000000`持续刷屏。根源是`ros2_px4_stack/transforms/
mocap_to_livox_frame.py`第60行`tfs.header.stamp = msg.header.stamp`
——直接照抄收到的`term_goal`消息的时间戳去广播`{veh}/init_pose ->
{veh}/pose_af`这条TF，而我手写的`ros2 topic pub`命令没有显式设置
`header.stamp`（默认零值），于是持续广播出时间戳为0的`NX01/pose_af`
变换，NX02这边监听这个跨机坐标系时不断因为"数据比缓冲区已有的还旧"
被拒收——这也是那个残留进程造成的，不是NX02自己的独立问题。

**处理**：`docker exec ... kill -9`杀掉残留进程后，两边立即确认
恢复正常——NX01此后15秒内`Changing DroneStatus`降到0次，NX02此后
15秒内`TF_OLD_DATA`降到0次。

**经验教训**：以后要在容器里临时起一个后台/循环进程（比如
`ros2 topic pub -r`这种不会自己退出的命令）做验证，`timeout N
docker exec ...`不可靠——必须显式在容器内`kill`掉，或者用
`docker exec ... bash -c "cmd & sleep N; kill %1"`这种在容器内部
自行控制生命周期的写法，或者干脆改用`--once`/有限次数的发布方式，
避免留下这类容易被误判为"新bug"的验证残留。另外
`mocap_to_livox_frame.py`盲目信任外部消息的`header.stamp`这处，
本身也是一个可以加固的点（改成用`self.get_clock().now()`自己盖
时间戳），但只有在类似这次手写`ros2 topic pub`不设时间戳的场景才会
触发，RViz等正常发布路径不会有这个问题，暂时不算需要修的bug，先
记录在这里。

## NX01和NX02先后又坠机，但都不是之前已修的两个bug——发现一种新的、还没根因坐实的"反复短促姿态失稳、逐渐加重"模式，已开verbose日志等下次复现

**NX01这次的现象**：给NX01一个新目标点时，它当时正处于"接近上一个
目标点但还没完全到达"的`GOAL_SEEN`状态（不是完全悬停的
`GOAL_REACHED`）。目标点发出后1~2秒内，姿态遥测从平稳开始逐渐
恶化（推力先小幅下探，`target_rpy`小幅摆动，然后迅速放大成推力
打满、姿态大幅震荡），最终摔地。排除了两种嫌疑：
1. 不是已修的"起飞/到达终点瞬间`next_goal.pos`离当前位置太近导致
   atan2失稳"——那个guard对`TRAVELING`和`GOAL_SEEN`都生效，而且
   这次是逐渐恶化不是瞬间爆炸，特征对不上。
2. 不是mighty自带的"规划持续失败就原地自旋找路"保护——查了配置
   `yaw_spinning_threshold: 10000`次失败才触发、`max_replan_
   failure_duration: 30.0`秒后直接放弃目标点，两个参数就是故意
   设计成基本不会真触发，而且这次异常在给目标点后1~2秒内就发生，
   远够不到这两个阈值。

**NX02这次的现象（用户提示"这次飞的是NX02"之后现场分析）**：更细致。
不是一次性坠机，是一串**反复出现、逐渐变频繁**的短促姿态失稳
（`target_rpy`在几百毫秒内扫过大半个偏航范围，然后短暂收敛稳定在
一个新的偏航值，过几秒又发作一次），发作间隔从最初~10秒逐渐缩短，
每次发作后飞机的真实位置(`actual_pos`)也有明显、非单调的抖动/来回
（比如x从-6.61抖到-7.03又折回-6.40，不是朝目标平滑前进），最终
在其中一次发作没能收敛住时真正摔地。这种"反复发作、间隔越来越短、
中间夹杂着真实位置的非单调抖动"的模式，比之前两次修过的bug更像是
**局部轨迹重规划(L-BFGS/HGP)本身在连续重规划之间输出不稳定/不连续
的候选轨迹**，而不仅仅是偏航角计算这一处的数值退化——因为如果只是
偏航计算的问题，飞机的实际位置不应该跟着来回抖动。

**这次没能坐实根因**：`mighty_node`的详细失败原因日志（HGP全局路径
失败/安全走廊失败/L-BFGS优化失败，具体是哪一步）平时被`debug_verbose`
（默认关闭）和launch文件的`--log-level error`一起挡住了，两次事件
都没能抓到具体失败原因。已经用`ros2 param set /NX0{1,2}/mighty_node
debug_verbose true`现场把两架飞机的verbose日志都打开了（不需要
重新build），下次复现能抓到L-BFGS/HGP每一步的详细信息。

**结论**：这是一个新的、尚未根因坐实的问题，比之前两次修的范围更大，
怀疑点在轨迹重规划本身的数值稳定性，而不是偏航角计算这一层。已经
为下次复现做好更详细的日志准备。

## 用户点破关键规律："都是规划完成途中给下一个目标点打断当前规划开始下一个规划"——终于用debug_verbose现场坐实根因：不是规划失败，是mighty的w_max跟PX4实际跟踪能力差了4倍

**背景**：连续两次分别在NX01/NX02身上复现的坠机，都没能坐实根因——
第一次尝试打开`debug_verbose`用`ros2 param set`现场设置，结果发现
`mighty_node`的`debug_verbose`只在启动时读一次（`setParameters()`
没有`add_on_set_parameters_callback`），live设置完全不生效，白白
浪费了一次复现机会。改成直接进容器改
`/opt/mighty_ws/install/mighty/share/mighty/config/hw_mighty.yaml`
里的`debug_verbose: false`为`true`，再`docker compose restart`
才真正生效（不用重新build——这是运行时加载的yaml，不是编译产物）。

**用户的关键观察**："都是这种规划完成途中给下一个目标点打断当前
规划开始下一个规划的操作"——即：不是随便什么时候给目标点都会炸，
是飞机还在飞向A点的途中（带着速度），被打断塞进一个新目标点B，
让`mighty`在还没到达A的情况下立刻改规划去追B。

**现场抓到的verbose日志彻底推翻了"重规划持续失败"这个之前的怀疑
方向**：15:05:07打断→15:05:12开始发散的整个窗口里，`mighty`的
HGP全局路径规划和L-BFGS局部轨迹优化**每一个周期都成功**
（"Replanning succeeded"连续打印，一次"FAILED"都没有）。之前误以为
是规划失败触发某种保护机制的猜测（"keep spinning"自旋保护、
HGP/L-BFGS数值故障）全部不成立——规划器一直在正常工作。

**真正机制**：飞机带速度飞向A点时被打断给了方向完全不同的目标点B，
`mighty`确实成功算出了一条"保持当前速度→掉头奔向B"的可行轨迹，但
这条轨迹要求的偏航角速度超出了飞机物理上能跟踪的范围，导致真实
姿态失控——不是数值退化bug，是规划器允许的机动比飞机实际能跟踪的
更激进。

**对上号的数字**：`mighty`的`w_max`（规划器允许的最大偏航角速度）
在本session早前从`1.0 rad/s`调到了`3.0 rad/s`（约172°/s，见
`mighty_avoidance_tuning.patch`），为了解决"机头跟得慢"的抱怨；
但PX4的`MPC_YAWRAUTO_MAX`只有45°/s——两者差了将近4倍。早前
"MPC_YAWRAUTO_MAX调到180必炸机、退回45缓解"那次教训只堵住了PX4
这一侧，`mighty`自己发出的偏航角速度指令（很可能通过body_rate
直接下发、绕过了PX4那层限速）一直没跟着降，这次"半路打断掉头"的
场景终于把这个隐患暴露出来。

**修复**：`patches/mighty_avoidance_tuning.patch`里`w_max`从`3.0`
改成`0.8 rd/s`（≈45.8°/s，跟`MPC_YAWRAUTO_MAX=45°/s`对齐）。只影响
`flight-stack`镜像（`sim-world`不用`hw_mighty.yaml`）。这是手改
已有patch文件（不是staged-baseline重新生成），改完在完整18个patch
链上跑了`git apply --check`全量验证通过，YAML语法也用python校验过。

**代价**：机头转向会重新变慢，跟本session早前"机头跟不上"的抱怨
会有点反复——但这次证据链完整（debug_verbose实锤规划器工作正常、
差4倍的具体数字对得上、"半路打断"这个触发条件也吻合），判断这是
目前为止证据最扎实的一次修复。**还没rebuild验证**。

## 用户反馈"飞机之间避障没有实现，NX01撞上了NX02"——找到真根因：UWB真值节点喂给mighty避障的坐标变换少了一项修正，量级正好是几米

**排查过程**：先确认数据链路本身没断——`frame_align_received_`门禁
（`ros2 topic echo /frame_align/NX01/NX02`确认`uwb_ground_truth_node`
在正常发布数据）、`/trajs`轨迹共享（含本session早前加的1Hz心跳，
`mighty_share_traj_heartbeat.patch`）、`convertDynTrajMsg2DynTraj`
里`mode=="pwp"`时优先用`quintic_pwp`字段（心跳消息走的正是这条，
没被"pwp.times为空就丢弃"那个检查误伤）——全部确认工作正常，排除
了"根本没收到对方数据"这个方向。

**真正定位**：现场对比`/NX01/traj_transformed`可视化marker（NX02
在NX01本地系里的"感知位置"）跟NX02自己真实位置的变化趋势——
NX02几乎悬停不动，但marker坐标2秒内从(1.19,-0.24)漂到(0.42,-0.08)，
完全跟不上/对不上，确认变换本身有问题。翻`src/uwb_sim/uwb_ground_
truth_node.py`（本仓库自己写的UWB真值模拟节点，不在mighty的
staging/patches体系里，是独立的ROS2包）：

```python
def _compute_relative_transform(self, i_name, j_name, stamp):
    pi = self.latest_poses[i_name].position   # Gazebo世界系机身位置
    pj = self.latest_poses[j_name].position
    dx = (pj.x - pi.x) + noise   # 只有这一项
    ...
```

这个函数算的是"此刻j机身相对i机身的UWB测距量"（D=世界系机身位移），
直接发布到`/frame_align/{i}/{j}`。但mighty那边`applyFrameAlignTransform`
拿到这个变换后，是直接乘到**j机自己local map系下的轨迹坐标**上
（`trajCallback`→`applyTransformToTraj`）——它需要的量是"j机map
原点在i机map系下的位置"，不是"此刻两机机身的相对位置"。这两个量
只有两架飞机都恰好停在各自map原点（局部坐标(0,0,0)）时才相等，
一旦飞起来偏离了各自原点，就会差出各自当前的局部位移那么多——量级
跟飞行范围一样大（本例中差了超过1米，让NX01把NX02定位到偏差好几米
外的错误位置）。

巧的是同一个文件里`broadcast_map_tf`函数（专门给RViz可视化用的另一条
TF）**已经用对了正确公式**，注释里还写清楚了推导：

```python
# P(map_j原点 在 map_i系下) = D + L_i - L_j
# D = 当前UWB测得的(j机身 - i机身)世界系位移
# L_i/L_j = i/j机身在自己map系下的位置（DLIO里程计给的）
t.transform.translation.x = (pj.x - pi.x) + li.x - lj.x
```

`/frame_align/*`（喂给mighty避障用）那条分支漏了`+li-lj`这个修正项，
是个遗漏，不是设计如此——两个分支该用同一个公式。

**修复**：给`_compute_relative_transform`也补上`L_i - L_j`修正
（复用已有的`self.latest_local_odom`，跟`broadcast_map_tf`保持
一致），并且在两架飞机的DLIO里程计数据都到齐之前不发布（避免用
不完整数据算出错误变换）。

**现场验证**：rebuild `sim-world`镜像（`src/uwb_sim`是普通COPY+
colcon build，不走patches那套workflow）、重启三个容器、两架飞机
起飞悬停在各自原点附近后：`/frame_align/NX01/NX02`变换恢复成
`(2.96, 0.04, 0.0006)`——跟NX01/NX02的`INIT_X`相差3米完全吻合；
`/frame_align/NX02/NX01`对称验证也是`(-2.96, -0.02, -0.14)`；
`traj_transformed`可视化marker确认NX02在NX01本地系里正确落在
`(2.94, 0.03)`附近，不再漂移。**避障能否真正生效还没做实际的
双机对飞测试**——数学上和坐标数值上确认修复了，但还需要现场
飞一次交叉航线验证L-BFGS的动态避障约束真的会触发、真的能避开。

## 现场验证：UWB坐标变换修复之后，避障确认真正生效——用户在RViz里连续两次直接看到NX01绕开NX02的路径

**验证过程有点曲折**：一开始查日志一直查不到任何成功的目标点接受
记录（`Changing DroneStatus`没有新变化、也没有"Replanning succeeded"），
跟用户"看到NX01明显避开了NX02"的观察对不上——一度怀疑用户看到的是
RViz里的可视化残留。后来发现是自己检索日志的时间窗口一直没踩准
（`docker compose logs -f`不加`--since`默认会从容器启动开始重放全部
历史，第一次用这个方式"实时盯梢"结果看到的其实是老数据），加上重启
`sim-world`时顺带触发的地图重新初始化过程本身需要一点时间（`sanitize
TerminalGoal`用的`checkIfPointOccupied`在`map_util_for_planning_`
还没建立、`map_util_`兜底也没数据时会统一判"占用"，导致重启后短暂
一段时间内所有目标点都会被拒绝——这是本次会话中额外发现的一个独立
遗留问题，跟UWB坐标变换无关）。

**最终确认**：用带时间戳过滤的`docker compose logs -t --since=3m`
重新检索，找到了确凿证据——3分钟内`Replanning succeeded`打印了
1562次（说明持续在积极重规划，不是空转），并且有一条真实的
`GOAL_SEEN→TRAVELING`（16:54:51.9）→13秒后`TRAVELING→GOAL_SEEN`
（16:55:04.9）→`GOAL_SEEN→GOAL_REACHED`（16:55:10.9）的完整、成功
执行的飞行记录，时间点正好落在用户报告"看到NX01路径避开NX02"的
两次操作之间。UWB坐标变换的修复（补上`L_i-L_j`修正项）确认从根上
解决了双机避障失效的问题。

## 用户要求：状态监控面板改成表格、加推力/油门列，不再滚动闪烁

`scripts/status_monitor.py`原来是两架飞机各起一个独立进程、每秒各自
`print`一行，两边输出交替刷屏，在tmux窗口里看起来一直在滚动。改写成
一个进程同时订阅两架飞机的`<ns>/dlio/odom_node/odom`（位置/姿态）和
`<ns>/mavros/setpoint_raw/target_attitude`（PX4上报的推力，跟
`attitude_thrust_logger.py`用的是同一个话题）,每秒用ANSI转义码
（`\033[2J\033[H`清屏+光标回左上角）原地重绘一张固定表格，飞机数量
和表格行数不变，只有单元格内容刷新，不再有滚动/闪烁感。

用法从`status_monitor.py NX01`（单机）改成`status_monitor.py NX01
NX02`（多机，一次性传入所有namespace）。`watch_sim.sh`的status窗口
也同步改成只起一个`docker exec`进程（原来是两个`docker exec ... &`
背靠背跑再`wait`），部署时顺手改用`docker exec`同步阻塞、不再用
`-d`后台+`&`并发，避免重复深挖过的"容器重启会杀掉docker exec进程但
tmux pane不知道、显示卡在最后一帧画面"这类残留问题——现在只有一个
进程，容器重启后pane会直接报错退出，比"看起来卡住不动但其实数据已经
死了"更容易第一时间发现。

现场部署验证：表格正确显示两架飞机的位置/姿态/推力/发布频率，推力
列此前一直没有、这次是新加的（读`msg.thrust`乘100转成百分比）。

## 用户提问："这次好像也顺带修复了之前的顽疾——机头方向背离运动方向，什么原因？"——闭环解释：从来不是坐标系反了，是偏航计算长期不稳定+跟踪能力跟不上的双重误差表现

这轮结束时用户注意到，本session早前反复排查却一直没能坐实根因的
"机头从来不对准飞行方向、机尾反而对着飞行方向"这个顽疾，似乎也随着
今天的修复一起消失了。给出了闭环解释：

本session早前已经用四元数代数推导（180°体轴旋转对`atan2`偏航提取
公式的完整Hamilton积证明）+ DLIO航向跟Gazebo真值逐帧对比（误差
<0.5°）两条独立路径反复验证过——**坐标系/符号从来没有反过**，机头
跟随的一直是同一套正确的`atan2(方向向量)`公式，不存在"参考系装反"
这种静态bug。真正的原因是期望偏航角本身长期存在大幅度、高频率的
计算误差，随手一帧抓拍到的机头朝向自然经常是错的、有时甚至看着像
对着运动反方向——这是"误差幅度太大、太频繁"造成的观感，不是"方向
定义反了"。具体是两条今天才补上的病根：

1. `getDesiredYaw()`里`atan2`对近零向量求方向不稳定（`GOAL_REACHED
   →TRAVELING`等切换瞬间触发，`mighty_traveling_yaw_zero_dist_
   guard.patch`修的那处）。
2. `mighty`的`w_max`（3.0 rad/s≈172°/s）比PX4实际能跟踪的速率
   （`MPC_YAWRAUTO_MAX`=45°/s）快了近4倍，飞机长期处于"追不上目标、
   持续滞后/超调/来回震荡"的状态（今天改成0.8 rad/s，`mighty_
   avoidance_tuning.patch`那处）。

两条叠加，只要飞机一移动，期望偏航跟真实偏航之间就长期存在几十度
甚至上百度的误差——这次一起堵上之后，期望偏航既不再乱跳、飞机也
终于能跟得上，机头才终于能稳定、持续地对准飞行方向。是这两处独立
修复的自然结果，没有再单独动过任何"偏航方向定义"相关的代码。

## 用户反馈"NX01抵达目标点后yaw角还在不停大幅变动"——排查确认是PX4自己的EKF姿态估计器在收敛，不是规划问题，几分钟后自己稳定了

**现象**：NX01到达目标点、`DroneStatus=GOAL_REACHED`之后，`mavros/
local_position/pose`里的姿态四元数持续大幅跳动（连续5次采样，偏航
换算下来在-26°到+112°之间来回摆），位置本身却是稳定的。

**排查**：分三层现场抓取姿态数据对比：
1. `mighty`发给PX4的指令姿态（`mavros/setpoint_raw/target_attitude`）
   ——11秒内几乎不变（z≈-0.0390，w≈-0.9992），说明规划器没有在
   GOAL_REACHED之后还在乱发指令，前一轮怀疑的"离目标太近导致atan2
   方向计算被噪声放大"这个方向排除了。
2. `DLIO`自己的定位输出（`dlio/odom_node/odom`）——同样全程稳定
   （z从-0.033到-0.039），说明外部定位真值本身没有问题。
3. `mavros/local_position/pose`（PX4自己内部EKF融合出来的姿态估计）
   ——只有这一层在大幅摆动。

**结论**：不是规划问题、也不是DLIO定位问题，两者的数据全程一致且
稳定；问题出在PX4自己的姿态估计器（EKF）——外部视觉/定位真值
（DLIO）和PX4内部IMU积分的估计短时间内没对齐，需要几分钟磨合收敛，
磨合期间融合输出的姿态会跳动。

**触发时机核对（用户指出"7分钟前开始的，不是现在这样"后，回头查了
时间戳）**：最初以为是flight-stack容器重启/重连触发的（今天为了让
`w_max`/`debug_verbose`等改动生效重启过好几次）——但核对发现容器
启动时间是16:48，而振荡是17:11前后才稳定下来、往前推7分钟是17:04
左右，中间隔了23分钟，时间对不上，重启不是直接触发原因。真正对得上
时间戳的是**这段时间里的一次真实飞行**：日志显示17:03:55~17:04:25
NX01刚好经历了一次`GOAL_REACHED→TRAVELING→GOAL_SEEN→GOAL_REACHED`
（这轮验证双机避障时给的目标点触发的那次飞行），到达时刻17:04:25，
振荡持续到约17:11才收敛——时间上正好对得上"这次真实机动之后
EKF姿态估计扰动、花了7分多钟才重新收敛"，不是重启触发的。是否
每次真实飞行（尤其是涉及动态避障重规划的飞行）之后都会有这么长的
收敛期，还需要后续观察是否复现，如果频繁出现、影响正常使用，
再考虑调PX4的EKF2外部视觉融合相关参数（`EKF2_EV_*`系列）；不是
今天改的任何代码（`mighty_traveling_yaw_zero_dist_guard.patch`/
`w_max`/UWB坐标变换）引入的新问题，暂时不需要额外修复。

## 用户要求：新增双机目标点手动输入功能，世界坐标输入一次同时发给双机——新建`goal`窗口+`dual_goal_input.py`

**选型讨论**：status窗口不合适（每秒`\033[2J\033[H`原地重绘的循环，
跟阻塞等待键盘输入天然冲突）；RViz不合适（"2D Goal Pose"工具一次只能
点一个点发给一架飞机，要做到"一次输入同时发双机"得写自定义C++ Tool
插件，本session早前验证过这条路成本很高，连改个按钮名字都做不到）；
最终选定跟`launch`窗口同样风格——新开一个tmux窗口跑一个常驻的
Python/rclpy交互脚本。

**`scripts/dual_goal_input.py`**：新建的常驻rclpy节点，同时给
`/NX01/term_goal`和`/NX02/term_goal`各建一个publisher。核心设计：

1. **世界坐标输入，自动换算局部坐标**：用户只输入一次world系的
   `x y z`，脚本按`local_x = world_x - INIT_X[ns]`（NX01=3.0，
   NX02=6.0，仅x方向有偏移）分别算出两架飞机的局部坐标再发布。
2. **常驻节点+常驻publisher，不是一次性`ros2 topic pub --once`**——
   本session踩过好几次"--once在DDS发现完成前就退出、消息实际没发出
   去"的坑，常驻节点从根上避开：启动时等最多2秒直到两个publisher都
   匹配上订阅者，之后每次重绘前用`spin_once`循环抽干积压消息。
3. **面板常驻显示双机当前世界坐标**（用户明确要求的）——订阅两架
   飞机的`mavros/local_position/pose`（local系下的真实位置），加回
   `INIT_X`换算成world系实时显示，方便决定下一个目标点该给多少；
   每次重绘前花最多0.3秒`spin_once`保证显示的是最新数据。
4. **表格/边框风格的"类图形界面"**——用户要求"类似图形界面形式"，
   用box-drawing字符（`┌─┐│└┘├┤`）画一个固定边框面板，ANSI
   `\033[2J\033[H`原地重绘（跟`status_monitor.py`同一个套路）。
   踩过一个坑：中文字符在终端里占两列显示宽度但`len()`只算一个
   字符，直接用`.ljust()`对齐会导致边框错位——改用
   `unicodedata.east_asian_width()`判断每个字符实际显示宽度
   （'W'/'F'类算2，其余算1）再计算padding，边框才能对齐。

**部署**：`watch_sim.sh`新增第六个窗口`goal`（黄色），`docker cp`
现改现塞，不参与patches编译流程。

**现场验证**：两次实测——一次目标点因为落在飞机没探索过的区域撞上
了已知的"Map is not initialized"遗留问题（跟这个新工具无关，日志
确认消息已经送达并被`sanitizeTerminalGoal`处理，只是被判定占用而
拒绝）；另一次发一个离当前位置近的目标点，立刻触发了真实的
`GOAL_REACHED→TRAVELING→GOAL_SEEN`状态切换，确认消息投递、坐标
换算、双机同步发送全部正常工作。

## 用户追问："一个坐标同时发给两个飞机？"——确认是压力测试用法，进一步要求两种模式都要：3个数同一目标、6个数各自不同目标

用户确认了"两架飞机飞向同一个世界坐标"确实是有意为之的用法（当avoid
避障功能的压力测试用），但同时要求`dual_goal_input.py`两种输入模式
都支持：

- **3个数`x y z`**：同一个世界坐标，换算成各自局部坐标后同时发给
  NX01和NX02（原来的行为）。
- **6个数`x1 y1 z1 x2 y2 z2`**：两个不同的世界坐标，前3个给NX01，
  后3个给NX02，各自独立换算、独立发布。

`send_goal(world_xyz)`改成`send_goals(world_targets)`，参数从单个
坐标元组改成`{ns: world_xyz}`字典——3个数输入时两个ns填相同坐标、
6个数输入时各自不同，下游逻辑（换算局部坐标、发布、记录历史）统一
处理，不用分叉两套代码。面板的"最近发送记录"和提示文字也相应改成
区分显示"[同一目标]"还是"[两个不同目标]"。

顺手修了一个显示bug：加长提示文字后（"6个数x1 y1 z1 x2 y2 z2"那行）
实际显示宽度59列超过了原来`BOX_WIDTH=60`留给内容的56列空间，边框
错位——`BOX_WIDTH`从60调到66，两种输入模式的提示文字都能放得下。

现场验证：3个数和6个数两种输入都实测过，NX01两次都收到真实处理
（`DroneStatus`正常切换、飞完一次完整航线），NX02那次撞上已知的
"地图未初始化"遗留问题（跟这个工具无关，消息本身确认送达）。

## 用户反馈"start.sh启动后，goal窗口一直显示NX02还没收到位置数据"——真根因是`input()`阻塞冻结了整个spin循环，不是DDS跨容器发现慢

**第一次判断方向错了**：一开始怀疑是`network_mode:host`下跨容器DDS发现
慢，或者`ipc:host`没配全导致SHM传输静默丢包（`docker-compose.yml`里
确实为这个已知坑写过详细注释）。实测排除：三个服务的`network_mode:
host`+`ipc:host`都配全了；单独写一个诊断脚本（常驻订阅双机
`local_position/pose`、打印首帧到达的相对时间）测出NX01/NX02跨容器
发现只要0.7~1秒，跟"卡住不动"的现象对不上。

**真正复现路径**：用户指出关键——不是"容器已经在跑、单独重启
`dual_goal_input.py`"这种场景偶发，而是`docker compose down`之后
全新`./start.sh`必现。照着这个路径实测：`down`+`start.sh`之后
goal窗口冻结在"NX01/NX02: 还没收到位置数据..."整整6分钟不动
（用户反馈正常起飞只要半分钟），但同一时刻直接进容器`ros2 topic
hz /NX02/mavros/local_position/pose`看到话题一直在稳定11Hz发布，
`mavros/state`也显示`connected:true armed:true mode:OFFBOARD`——
数据链路完全正常，问题出在`dual_goal_input.py`这个消费者自己身上。

**根因**：`dual_goal_input.py`主循环是"重绘一次面板→`input('>
')`阻塞等用户敲一行命令→重绘"，`input()`是Python同步阻塞调用，
会把整个单线程事件循环（包括负责处理回调、更新`latest_local_pos`
的`rclpy.spin_once`）一起冻结住——两次用户按键之间，哪怕
`local_position/pose`话题一直在正常发布，DDS底层QoS缓存（`depth=10`）
里堆的消息也不会被取出处理，回调根本不会被触发。`start.sh`起完
tmux六个窗口后没人会立刻去点goal窗口敲键盘，而这个窗口的
`dual_goal_input.py`进程从tmux创建那一刻就已经启动、比MAVROS真正
开始发布数据早了几十秒，于是它永远冻结在刚启动那一帧"两架飞机都
还没发过位置"的空状态上，哪怕之后飞机早就正常起飞好几分钟了也不会
自动恢复。用`tmux send-keys`往这个窗口发一个空回车能立刻验证：
按键触发的那次`spin_once`把积压的消息一次性处理掉，面板瞬间显示出
正确坐标——反过来证实了阻塞点就是`input()`。

（用户中途反馈过"单独启动能看到NX01有数据、NX02没有"，这只是这个
根因的另一种表现——取决于用户手动按了几次回车、以及当时NX01/NX02
两边MAVROS各自连上的先后时间点，本质上还是同一个"两次按键之间数据
不会自动刷新"的问题，不是NX01/NX02两者之间有什么本质差异。）

**修复**：`scripts/dual_goal_input.py`的`main()`里，进入
"重绘+`input()`"交互循环之前，新增一段专门等待的循环：只要两架飞机
还没都收到过至少一帧位置数据，就持续`spin_once`、每秒重绘一次带
倒计时的等待提示（"等待双机位置数据到齐...（已等N秒，Ctrl-C可跳过
直接进入面板）"），不依赖用户按键就能把积压消息处理掉、把面板刷新
到最新状态；两架飞机都有数据后才进入原来的交互循环。

**已验证**：完整走了一遍`docker compose down` → `./start.sh`冷启动，
全程没有对goal窗口做任何按键操作，面板从"还没收到位置数据..."自动
经过带倒计时的等待提示，在MAVROS真正连上之后自动刷新出双机正确的
世界坐标——确认修复生效，不再需要手动敲一下回车才能"唤醒"面板。

## 用户问："RViz要以全局坐标系显示双机，需要发布哪些变换？"——排查发现两个独立的bug，都在`NX01/map -> NX02/base_link`这条跨机链路上

用户想把`multi_mighty.rviz`的双机可视化（里程计/占据栅格/点云/规划轨迹/
真实航迹）从"只能挂一架飞机的局部系"改成能同时看两架的效果。先梳理了
一遍现有机制：`{ns}/map -> {ns}/odom`（`static_tf_node`，恒等）→ DLIO
动态广播`{ns}/odom -> {ns}/base_link -> ... `，两机之间靠
`uwb_ground_truth_node`的`broadcast_map_tf()`（上一轮session加的，
"新增：`frame_align`同时广播成TF"那节）桥接`NX01/map -> NX02/map`。
现场`tf2_echo`确认这条桥接TF确实在正常发布、数值也对（3米，跟spawn间距
吻合），`multi_mighty.rviz`的`Fixed Frame`也已经是`NX01/odom`（不是裸
`map`）——单看这两段似乎该通了。

**用户实测反馈**：RViz里把Fixed Frame换成`NX01/map`，NX02的话题（尤其是
点云/base_link相关的）还是显示不出来。现场排查，找到两个独立的bug，
凑在一起造成这个现象：

### bug 1：`{ns}/base_link`同时被两个不同的父frame声明

`ros2 topic echo /tf`和`/tf_static`分别抓了`NX02/base_link`的来源，
实锤冲突：

- `/tf_static`：`ros2_px4_stack/launch/dynus_mavros.launch.py`里的
  `body_to_base_link`节点，声明`body -> NX02/base_link`（恒等）。
- `/tf`：DLIO的`odom.cc`实时广播`NX02/odom -> NX02/base_link`（真实
  位姿，33条消息全部一致）。

同一个child frame被两个不同的父frame同时认领，tf2解算到哪一条不保证
稳定；而`body`本身是个死胡同——没有任何节点发布过"谁是body的父
frame"，一旦tf2解算走了这条，`NX02/base_link`以及挂在它下面的雷达/
相机frame就从整棵TF树上断开，`NX01/map`不管怎么设都够不到NX02。

**根源**：`dynus_mavros.launch.py`是上游DYNUS/Fast-LIO单机+mocap架构的
遗留代码（注释原文"Fast-LIO publishes camera_init -> body"——这套仿真
用的是DLIO+mid360+多机UWB对齐，根本不是那条链路），里面一共起了6个
`static_transform_publisher`：

- **必须保留**（`mocap_to_livox_frame.py`的`get_gf_to_af_tf()`真的用
  tf2查了`{veh}/init_pose` <-> `world_mocap`，是`term_goal`全局系转
  局部系那条链路的实际依赖）：`{veh}_odom_to_mocap`
  （`world_mocap -> {veh}/init_pose`）、`world -> world_mocap`。
- **纯死代码，删掉无风险**（全仓库grep`camera_init`/`d455_link`/
  `body`，只在注释里出现过，没有任何活跃节点真的用tf2查它们）：
  `init_pose_to_camera_init`、`base_to_d455`、`world -> map`（另一个
  同名的`map_to_odom`节点——裸"map"是个跟`{ns}/map`毫无关系的孤立
  frame，留着容易在Fixed Frame下拉框里选中一个空世界）。
- **死代码+真冲突**：`body_to_base_link`——上面bug 1的根因。

**修复**：`patches/ros2_px4_stack_base_link_tf_conflict.patch`，删掉
`body_to_base_link`/`init_pose_to_camera_init`/`base_to_d455`/第二个
`map_to_odom`（`world->map`）这四个Node()，保留另外两个。已加入
`Dockerfile.flight-stack`的patch链（排在最后，跟`ros2_px4_stack_
control_law_launch.patch`等前面的patch共用同一个文件，实测在真实
Dockerfile的patch顺序下`git apply --check`全链路通过）。**已验证**：
rebuild后`/tf_static`里再也搜不到`body -> NX02/base_link`。

### bug 2：UWB桥接TF和DLIO动态TF不在同一个时钟域，跨机多跳lookup必然失败

修完bug 1，`tf2_echo NX01/map NX02/base_link`还是失败，报"Lookup would
require extrapolation into the past"，但奇怪的是`NX01/map->NX02/map`
和`NX02/odom->NX02/base_link`两段**分别单独查都完全正常**。抓
`/tf`原始时间戳对比才找到真根因：

- DLIO动态广播的`NX02/odom -> NX02/base_link`：`stamp.sec`
  约`1735689xxx`——这不是真实时间，是`mighty_imu_sim_time.patch`/
  `livox_imu_lidar_sim_time.patch`给IMU/点云盖的"假epoch"
  （sim时间 + 固定偏移`1735689600`=2025-01-01T00:00:00Z，规避Gazebo
  真实时间因子漂移+DLIO点云类型探测的1e14ns阈值，见那两个patch的
  注释），DLIO自己发布的TF继承的正是这个假epoch。
- `uwb_ground_truth_node.broadcast_map_tf()`盖的是
  `self.get_clock().now()`——真实墙钟，`stamp.sec`约`1785986xxx`
  （这套宿主机是2026年）。

两边差了将近5000万秒（约一年半）。tf2的多跳lookup要求链路上所有边
有一个公共有效时间窗，跨两个相差一年半的clock域必然找不到公共时间，
`NX01/map -> NX02/base_link`这种同时经过UWB桥接边和DLIO动态边的完整
链路必然失败——这正是"Fixed Frame设对了，NX02的东西还是显示不出来"
的第二层原因，跟bug 1是两个独立问题，叠在同一条链路上。

**修复**：`src/uwb_sim/uwb_sim/uwb_ground_truth_node.py`（一手代码，
不走patch）——`local_odom_cb`订阅DLIO里程计时顺手多存一份
`msg.header.stamp`；`broadcast_map_tf()`不再用
`self.get_clock().now()`，改用两机DLIO里程计各自最新header.stamp里
较新的一个，让这条桥接TF落进跟DLIO/mighty同一个假epoch时钟域。只涉及
`sim-world`镜像。**已验证**：rebuild+`docker compose up -d`重建
sim-world容器（注意：改了镜像内容必须用`up -d`重新创建容器，
`docker compose restart`只是重启已有容器进程、不会换成新镜像，
这次踩了一次坑）后，`tf2_echo NX01/map NX02/base_link`和
`tf2_echo NX01/map NX02/NX02_livox`都能正常解出`[3.0, 0, ~0]`米
（吻合NX01/NX02的3米spawn间距），双机跨机的完整TF链路（含点云/
base_link）确认打通。

## bug 3：occupancy_grid同样是"假epoch vs 墙钟"，但这次根源在global_mapper_ros自己身上

上面两个bug修完，用户现场用RViz实测：Fixed Frame切到`NX01/map`后双机
点云、里程计都能正常显示了，唯独`NX02/occupancy_grid`还是显示不出来。
追问一句"Fixed Frame切到NX02/map时NX02自己的occupancy_grid能不能
显示"，用户确认——能。这个"同namespace能看、跨namespace看不到"的
模式跟bug 2一模一样，直接照着bug 2的思路查：`ros2 topic echo
/NX02/occupancy_grid`的`header.stamp`是墙钟时间（跟`date +%s`对得上），
而bug 2已经把`NX01/map -> NX02/map`这条桥接TF改成了DLIO的假epoch
时钟域——`occupancy_grid`这条消息本身还是墙钟，又跟桥接TF的时钟域
对不上了，同一个bug模式换了个消息源重新出现一次。

`global_mapper_ros.cc`（`acl-mapping/global_mapper_ros`包，被
`global_mapper_node`加载，负责把点云+位姿转成occupancy_grid/
unknown_grid这些栅格话题）里，`PopulateOccupancyPointCloudMsg`/
`PopulateUnknownPointCloudMsg`/`PopulateDistancePointCloudMsg`/
`PopulateCostPointCloudMsg`/`PopulateEsdf2DMsg`/`PopulateOcc2DMsg`/
`PopulatePathMsg`这7处函数，清一色`header.stamp = this->now()`或
`rclcpp::Clock().now()`（node自己的墙钟），没有一处用输入点云的
时间戳。巧的是`PointCloudCallback`里已经有一个现成的
`tstampLastPclFused_`成员变量（记录"最近一次成功融合的点云的
header.stamp"，本来是给别处用的），直接复用它做这7处的时间戳来源，
不用新增任何订阅/成员变量。**修复**：`patches/global_mapper_ros_
sim_time_stamp.patch`，7处`this->now()`/`rclcpp::Clock().now()`
统一换成`tstampLastPclFused_`。

**踩到一个`git apply`的坑，专门记录**：`acl-mapping`是一个git仓库，
`global_mapper_ros`只是它下面的一个子目录（不是独立repo，这点跟
"mighty"那个repo结构不一样——mighty patches的cwd是`mighty_ws/src/
mighty`本身就是repo root，对应这次patch如果照抄那个套路把cwd设成
`mighty_ws/src/acl-mapping/global_mapper_ros`就错了）。第一次打这个
patch时，patch文件路径写的是相对`global_mapper_ros/`（`a/src/
global_mapper_ros.cc`），Dockerfile里`cd`到`global_mapper_ros`子
目录再`git apply`——**`git apply --check`和实际`git apply`都返回
exit 0（成功），但源码文件完全没被修改**：`git apply`检测到cwd在
一个git仓库内部时，会按仓库根目录（`acl-mapping`）而不是cwd去解析
patch里的相对路径，子目录深度差一层，路径对不上，但`git apply`在
这种情况下不会报错退出，就是安安静静什么都不改。build日志里看不出
任何异常（`RUN`那一层显示`CACHED`/`DONE`，没有`!! 未能自动应用 !!`
这行走fallback echo的痕迹），第一次rebuild后凭"没报错"就以为生效了，
后来用`docker run --rm --entrypoint grep flight-stack:latest -c
tstampLastPclFused_ .../global_mapper_ros.cc`直接查built image里的
源码才发现只有1处（原来就有的那处赋值），改的7处一个都没生效。
**教训**：patch"apply时exit 0"不能当成"确实改到了目标文件"的证据，
尤其是涉及git子目录/多repo嵌套结构时，验证patch生效与否必须直接
从build出来的image里`grep`/`diff`实际文件内容，不能只看构建日志
有没有报错。**修复**：patch文件里两个`global_mapper_ros.cc`路径
都改成相对`acl-mapping/`的`global_mapper_ros/src/global_mapper_
ros.cc`，Dockerfile里的`cd`也从`.../global_mapper_ros`改成
`.../acl-mapping`（repo root）。

**已验证**：rebuild后`docker run ... grep -c tstampLastPclFused_`
确认源码里有9处匹配（1处原有赋值+7处新替换+1处注释文本提到这个变量名，
计数对得上）；编译产物`global_mapper_node`二进制的mtime晚于源码文件
mtime，确认真的重新编译过。`docker compose up -d`重建
flight-stack-nx01/nx02两个容器（这次改动只涉及flight-stack镜像，
不用动sim-world）后，`/NX02/occupancy_grid`的`header.stamp`落进了
跟`NX01/map -> NX02/map`桥接TF同一个假epoch时钟域，`tf2_echo NX01/map
NX02/map`正常解出`[3.0, 0, 0]`米。用户现场在RViz里确认：Fixed Frame
挂`NX01/map`时，双机的occupancy_grid、里程计、点云终于能同时正确
显示了。

## 用户反馈"给双机发同一个目标点，第二次直接双机都无响应，报`Local Optimization Failed with status: 1, fopt: 17647.80`"——设计讨论存档，方案已定但代码还没动

**现象**：`dual_goal_input.py`发同一个世界坐标给双机（3个数模式）。第一次
`(-2.00,+0.00,+1.50)`，NX01到达，NX02只能在附近盘旋（"竞争上岗"）；
第二次发`(3.00,+0.00,+1.50)`，双机均无响应，日志刷`Local Optimization
Failed with status: 1, fopt: 17647.80`。

**根因**：`mighty`把NX01/NX02互相当动态障碍物处理（靠`/trajs`话题共享
各自规划好的轨迹，`share_traj: true`）。`lbfgs_solver.cpp`算避障代价
`J_dyn`时，在自己轨迹上取样、查对方轨迹同一时刻的位置：如果采样时刻
超出了对方**实际广播出来的轨迹时间范围**，`pwp.eval()`会把对方钳制在
"这条轨迹的终点位置"当成静止点持续查询。`mighty`是receding-horizon
局部规划（`horizon: 15`米+`local_box_size: [3,3,3]`米，按巡航速度换算
覆盖时间通常就几秒），邻机分享出来的轨迹天然不长，如果自己正在规划
一条更长的新航线（比如飞去一个稍远的新目标），采样点落在邻机轨迹
覆盖之外的部分会被钳制点反复命中，三次方(`h³`)惩罚失控冲高，
`fopt`冲过`hw_mighty.yaml`里`fopt_threshold: 10000`的阈值，规划整体
判定失败——哪怕optimizer其实收敛出了数值上有效的解（`status`是正数）。

一句话本质：**把"邻机还没规划到那么远的未来"（信息缺失）当成了
"邻机会永远停在轨迹终点不动"（确定性事实）去算代价，缺失的数据被当成
高置信度数据处罚**。

**已排除的疑似防护**：这套系统已经有一个1Hz心跳
（`mighty_share_traj_heartbeat.patch`加的
`publishSelfAsStaticObstacleCallback()`，跟`publishOwnTraj()`独立、
不受"只有重规划成功才发布"这条限制，专门给"完全空闲、从没收到过
term_goal"的飞机兜底可见性），但它只覆盖"对方压根没在规划"这一种
情况——NX02当时是持续在重规划（只是解都很短/被推来推去），走的是
`publishOwnTraj()`这条路，心跳被绕过去了，救不了这次的场景。

**评估过、确认不采用的方案**：在`dual_goal_input.py`里提前判断"两机
目标是否重合"、重合就只发一架、另一架保持原状悬停。讨论后确认没必要
——这只能防住"故意发同一目标点"这种极端压测触发路径，防不住更general
的诱因（两机目标不同、路径只是临时靠近，或者其中一方局部horizon比另
一方短，这是双机异步规划的常态，不需要目标点重合就会发生），而且会
削弱`dual_goal_input.py`3个数字模式本来就是故意设计的避障压力测试
能力。

**已查证：不是"多机=编队"能直接套的问题**——`formation_weight`/`J_form`
在这份代码里是个更窄的技术概念（维持跟指定邻机的固定相对偏移`δ_ij`，
吸引力代价，真正意义的编队队形），跟"各自独立奔目标、只是别撞上"
（`J_dyn`，排斥力代价）是两个方向相反的目标函数。直接打开
`formation_weight`不但不解决问题，因为`if (formation_weight_ > 0.0 &&
obs->is_agent) continue;`这行会让邻机整个从`J_dyn`里被跳过，等于双机
之间的避障保护反而没了，换成被强行拉向一个未配置好的相对位置，比现在
更糟。能复用的只是`getHorizon()`那段"查一下邻机数据够不够新"的记账
逻辑本身，不是"编队"整套语义。

**查过GitHub上游`mit-acl/mighty`所有分支，没有更完整的参考**：
`main`/`dev`/`dev-sim`/`dev-rr`/`dev-rr-mad`都是同一个状态，只有
`formation_weight_ > 0.0`这个窄条件下的`getHorizon()`保护（`25428ad
"formation flight"`这一个commit加的，当时是在调"5机星形编队仿真"时
撞上这个坑顺手修的，从没为普通避障场景generalize过）；`multiagent`/
`multiagent_hw`——听名字最应该有通用多机方案的两个分支——用
`git merge-base --is-ancestor`确认了，历史都在这个commit**之前**分叉
出去，压根没有这段保护，是完全裸奔的版本。本机代码里`J_form`那段已经
是整个上游仓库能找到的唯一参考，需要自己把它generalize到`J_dyn`上，
不是抄现成方案。

**候选方案**（未最终拍板，见下面的开放决策）：
- **A. 硬跳过**（照抄`J_form`写法）：超出邻机horizon直接不查、零代价。
  改动最小、是仓库里唯一有先例的写法，缺点是从"正常防护"到"零防护"是
  个硬边界，没有过渡。
- **B. 单点代价封顶**：`h³`设上限，钳制而不是无限增长。改动比A还小，
  但还是把钳位的幻影点当成确定存在的障碍物在算，只是不让它一家独大
  把总分冲爆。
- **C. 按超出时长衰减权重**：离邻机最后已知时间越远、可信度越低，
  惩罚力度跟着往下调，衰减到0之后等价于跳过。最贴近物理直觉（没有
  硬边界），但全仓库所有分支都没人写过这段逻辑，是原创实现，需要新的
  衰减函数+新参数，实现和验证成本都比A/B高。

**A方案的边界条件已经算过**（如果最终选A）：
- 不是100%保证死锁不再发生——只消除"陈旧/短horizon数据被误判成确定性
  障碍物"这一种假性失败；如果双机**当前真实有效**的轨迹恰好就是会
  冲突，`fopt`超阈值是optimizer如实反映"这条路径真不安全"，不是bug，
  A方案不该、也不会消除这种情况（多组并行初始猜测通常能绕开，除非
  真的无路可走）。
- 撞机风险：A方案引入的"看不见对方"窗口，最坏情况（对方持续规划
  失败、只剩1Hz心跳兜底）大致1秒量级（心跳周期），不会拖到
  `traj_lifetime`（`hw_mighty.yaml`里是7.0秒，不是mighty_node.cpp里
  写的默认值10.0，这套配置实际生效的是7秒）——完全收不到任何消息
  才会等到7秒后被`cleanUpOldTrajs`整个移除，这是现在就有的行为，
  A方案没让它变得更差。按`v_max: 1.0`m/s算，1秒最坏窗口对应最多1米
  漂移，比`planner_Cw: 3.0`米的软避让间距小，但已经在`drone_bbox:
  [1.2,1.2,1.2]`米的真实碰撞尺寸量级边上——不是绝对安全，是"按现在
  这组参数、双方都在高频重规划的前提下，风险窗口很窄"。安全边际依赖：
  心跳必须真实可靠运行（DDS丢包/CPU过载会让这个估算失效，需要实测
  验证）；`v_max`以后不能在没重新核算这个margin的情况下调高；上线后
  必须额外找一个双机路径交叉的场景，实测双机最小间距有没有靠近
  `drone_bbox`量级，不能只看"规划成功了没有"就算过关。

**当前状态**：讨论到这里，方案在A（简单、有先例、零防护窗口）和C
（更稳妥、无硬边界、但是全新代码）之间还没最终拍板，**代码还没有
任何改动**。下次接着做的时候，先确认选哪个方案，再动手写patch——
这次改的是`lbfgs_solver.cpp`，是flight-stack里跟真机共用的规划核心
代码，不是仿真专属，涉及双机避障的实际安全边界，值得按"安全相关代码
变更"的标准过一遍，不要因为"仿真里跑通了"就直接定稿。

## 用户要求：实现运行时数据记录方案（rosbag黑匣子滚动录制 + 容器stdout持久化 + 姿态日志落盘），顺带把项目补成git仓库

先补了一个基础动作：`docker_sim/`之前完全没有版本控制（`.gitignore`
早就有了、但没人跑过`git init`）。给顶层做了一次`git init`+首次提交
（`patches/`/两个`Dockerfile`/`scripts/`/`src/`/`README.md`等手写内容
共96个文件、1.2MB，`staging/`按`.gitignore`排除——那是`fetch_sources.sh`
拉的公开源码，7.9GB，丢了能重新拉，不需要版本控制）。git identity只设了
这个仓库本地的（`git config`不带`--global`），没碰全局配置。之后每次
改代码都能`git diff`/`git log`/出问题能`git revert`，是应对"改崩溃了"
最直接的保险；镜像本身（`flight-stack`9.2GB/`sim-world`22.9GB）没做
额外备份，因为都能从`patches/`+`staging/`确定性重新build出来，丢了是
"要花时间重建"不是"数据永久丢失"。

**数据记录方案分两层**，都是有界保留（滚动窗口/固定大小上限），不是
无限堆积：

**第一层——容器stdout持久化**（`scripts/tail_persist_logs.sh`，宿主机
侧跑，不在容器里）：`docker compose logs -f --tail=0 <service>`持续
追加进`runtime_logs/container_logs/<service>.log`，自己实现了个简化版
`logrotate`（超过50MB就转存成`.1`，最多留5份，约250MB/服务的上限）——
`docker compose down`会把容器自己的日志存储删掉，这层是为了让日志脱离
容器生命周期独立存活。`docker-compose.yml`三个服务也顺手加了原生
`logging: {driver: json-file, max-size: 20m, max-file: 5}`配置，作为
免维护但生命周期更短的第一层兜底（容器活着的时候`docker compose
logs`能看，容器一down就没了）。

**第二层——rosbag"黑匣子"滚动录制**（`scripts/record_rosbag.sh`+
`scripts/prune_rosbag.sh`，跑在flight-stack-nx01容器里，
`network_mode:host`下能看到全部双机话题）：不用`ros2 bag record
--max-bag-duration`在同一个bag目录里内部分片（分片共用一份
metadata.yaml，删旧分片会把索引和实际文件对不上），改成用`timeout`
每5分钟重开一个全新、独立的bag目录，`prune_rosbag.sh`只保留最近12个
（约1小时），超过的从最老的开始整个目录删掉。

**录制话题选型，实测踩了一个坑**：一开始想当然把`occupancy_grid`跟
`unknown_grid`都录了，`ros2 topic bw`实测：`occupancy_grid`确实只有
约36KB/s/机，但`unknown_grid`（虽然名字听着像轻量的
`nav_msgs/OccupancyGrid`，实际类型是`sensor_msgs/PointCloud2`——
"grid"只是话题名字沿用的旧称呼；20x20米房间里"未知"格子点数远多于
"占据"格子）飙到约410~460KB/s/机，双机合计能占滚动录制总带宽的七成
以上，实测整个话题列表录了1分钟涨到155MB（外推约4.7GB/小时），跟
"不能让存储爆"这个目标直接冲突。**修复**：默认话题列表去掉
`unknown_grid`，只留`occupancy_grid`，改完实测录制带宽从约1.32MB/s
降到约206KB/s（6倍以上），5分钟一个块约62MB，12块滚动窗口稳定在
~750MB量级，需要专门查frontier/exploration相关问题时再手动加回
`unknown_grid`单独录一次，不常驻默认列表。默认录的话题：`/tf`
`/tf_static`、`/trajs`、双机`term_goal`/`mavros/state`/
`local_position/pose`/`occupancy_grid`、`frame_align`两个方向、
`/plug/model_states_plug`（Gazebo真值）——原始点云同样默认不录（比
occupancy_grid还大一个数量级），道理跟排除unknown_grid一样。

**留证机制**（`scripts/save_incident.sh`）：出事故时手动跑一下，把
最近3个滚动bag块（约15分钟，覆盖事发前后）+双机姿态/推力文本日志
复制到`runtime_logs/incidents/<时间戳>_<描述>/`长期保留，不受
`prune_rosbag.sh`滚动清理影响——现场测过一次（`save_incident.sh "测试
留证"`），确认能正确抓取最近的bag块+两机日志。

**`scripts/attitude_thrust_logger.py`顺手改了输出路径**：从容器内
`/tmp/attitude_thrust_debug.log`（容器一删就没了，得手动`docker cp`）
改成`/logs/<namespace>/attitude_thrust_debug.log`（`docker-compose.yml`
新挂载的`runtime_logs/`volume，容器销毁/重建数据不丢）；`/logs`这个
挂载点不存在时（比如脱离docker-compose单独调这个脚本）自动退回旧的
`/tmp`路径，不报错。

**`docker-compose.yml`改动**：三个服务都加了`./runtime_logs:/logs`
挂载（同一个宿主机目录挂到容器内同一个路径，各脚本自己按
`/logs/nx01`、`/logs/rosbag`这些约定的子路径分开写，不额外拆分
per-service挂载点）+ 上面提到的`logging:`原生日志滚动配置。这个改动
不涉及镜像内容、不需要rebuild，`docker compose up -d`重建容器就生效。

**`scripts/watch_sim.sh`新增两个tmux窗口**：`record`（紫色，起
`record_rosbag.sh`+后台`prune_rosbag.sh`）、`logs`（灰色，起三个
`tail_persist_logs.sh`）——从原来六窗口变成八窗口。

**实测发现的一个无关但值得记的坑**：验证过程中`ros2 topic echo`/
`ros2 topic list`突然集体报`xmlrpc.client.Fault:
RuntimeError:!rclpy.ok()`，一度以为是MAVROS真的没连上（花了不少时间
排查），最后发现是容器里`ros2` CLI自己的后台daemon卡死了，
跟MAVROS/PX4本身毫无关系——`ros2 daemon stop && ros2 daemon start`
之后所有CLI命令立刻恢复正常，`mavros/state`一直都是
`connected:true armed:true mode:OFFBOARD`。以后再遇到"ros2 CLI集体
报`!rclpy.ok()`或者莫名其妙的RPC错误"，先重启一下daemon再深挖，别
一上来就怀疑是被查的系统本身出了问题。

**另外记一笔**：`ros2 bag record`/这些脚本在容器里是以root跑的，
`runtime_logs/`下产生的文件在宿主机上也是root所有——宿主机侧普通用户
手动清理（比如想直接`rm -rf`某个旧的bag块）需要`sudo`，正常的滚动
删除（`prune_rosbag.sh`自己在容器内跑，权限足够）不受影响，只是"人
手动伸进去删"这个操作需要注意。
