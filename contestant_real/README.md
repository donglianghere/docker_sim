# 真机选手程序（contestant_real）

这是 `contestant_sim/` 的**真机副本**。两边程序逻辑完全一样，区别只有一个：
**这里的场地坐标要按真实赛场改**。

分成独立目录是因为两套坐标必然不同，放在一起迟早改混。想对比改了什么：

```bash
diff -u ../contestant_sim/formation_lite.py formation_lite.py
```

---

## 一、必须改的：场地坐标

四个 `*_lite.py` 顶部都有一段用 `⚠⚠ 需要按真实场地修改的常量 ⚠⚠` 框起来的区域，
要改的全在里面，别处不用动。

现在的值来自**仿真场地** `src/contest_mission/config/sample_room_layout.yaml`：
房间 20×25 米、世界原点在**西南角**、坐标全为正（x∈[0,20]，y∈[0,25]）。

| 常量 | 现值（仿真） | 含义 |
|---|---|---|
| `ROUTE_A` | `(3.0, 3.0)` | A 点，起降区 |
| `ROUTE_B` | `(3.0, 22.0)` | B 点 |
| `ROUTE_C` | `(17.0, 22.0)` | C 点 |
| `ROUTE_G` | `(17.0, 16.0)` | G 点，编队集结/解散 |
| `ROUTE_E` | `(10.0, 16.0)` | E 点 |
| `ROUTE_F` | `(14.0, 14.0)` | F 点 |
| `ROUTE_D` | `(17.0, 3.0)` | D 点 |
| `POINT_M` | `(6.0, 16.0)` | 观察位 M（看 3#/2# 楼） |
| `POINT_N` | `(14.0, 16.0)` | 观察位 N（看 1# 楼） |
| `SUPPLY_XY` | `(10.0, 8.0)` | 物资点（灭火弹） |
| `FACADE_YAW_DEG` | `90.0` | 正对楼面的机头朝向。火情贴在楼的 -Y 面，所以是正北 |
| `CRUISE_AGL_M` | `2.0` | 巡航离地高度 |
| `FORWARD_BEFORE_FIRE_M` | `1.5` | 对准后前移距离。**这是弹丸的要求，不是场地参数，别乱动** |

`STATIONS` 里的 `-90.0 / 90.0` 是在观察位上的机头朝向（3# 朝南、2#/1# 朝北），
楼的相对方位变了才要改。

### 改之前必须想清楚的一件事

这里写的是**世界系**坐标，而 `goto()` 吃的是飞机**自己的局部系**——SDK 的
`world_to_local()` 负责换算，依赖**起飞点原点锁定**那条 TF。

所以坐标即便全改对了，**原点标定没做的话整条航线仍会整体偏移**。迁移清单 A4
那项（UWB 原点标定）必须先做。

### 改完怎么验

```bash
python3 tools/check_route.py          # 核航线有没有撞障碍
python3 tools/timeline.py             # 看任务时序
```

⚠ `check_route.py` 里的 `INFLATE_M` 写死是 `0.8`，正好等于真机
`EGO_OBSTACLES_INFLATION` 的值，所以真机用它是对的（仿真那边实际是 0.6，
用它核仿真航线会偏保守）。

---

## 二、不用改但要知道的

### class_id：真机靠 AprilTag 并行检测

程序找的是 `apriltag:0`（物资）/ `apriltag:1`（高层火情）/ `apriltag:2`（地面火情）。
真机的 YOLO 模型 `best_960` 只认 `StoveCabinet / Campfire / FireBomb`，**认不得
AprilTag**——所以 2026-10-03 在 `yolo_detector_node` 里加了 AprilTag 并行检测
（`cv2.aruco` 的 `APRILTAG_36h11`，跟 YOLO 共用同一帧）。

两套 class_id 现在**同时存在于同一个话题** `vision/detections` 里，靠
`header.frame_id` 区分前视/下视。所以程序不用改。

标靶边长 `TAG_SIZE_M = 0.5`，真机也是 0.5 米，SDK 里不用改。

### 相机内参：自动适配，不用管

`aim_at` 2026-10-03 起改成读 `camera_info`，仿真拿 Gazebo 相机的值、真机拿实测
标定值（前视 fx=1200.71 / 下视 1194.15），同一份 SDK 两边都对。

### 综合任务的火情：人工布置

`mission_lite` 在仿真里靠 `referee.py` 增删 Gazebo 模型让火情随机出现。真机没有
这个手段，火情按赛前约定**人工布置**。

---

## 三、怎么跑

```bash
./vision_real.sh up           # 起机载视觉（两机×两路），等 10 秒
./vision_real.sh status       # 体检：应有 2 个节点/机、10 条话题
./run_real.sh formation_lite  # 跑程序（五道前置闸门自动把关）
./stop_real.sh                # 急停
```

### ⚠ `stop_real.sh` 不会让飞机降落

选手程序发的是任务级指令，掐掉之后机载 `ego_planner` 会把最后一条轨迹执行完、
然后**悬停在那里**。真正的应急手段是**遥控器接管**——那是安全保障手段，飞行时
必须在手。`--land` 是显式可选项，就地降落、落点不保证是起降垫。

### `run_real.sh` 的五道闸门

| 闸门 | 不过会怎样 |
|---|---|
| 两机 ping 可达 | 中止 |
| `/status` 的 `flight_up` + `vision_up` | 中止 |
| 命名空间对应（.101→NX01、.102→NX02） | 中止——配反了话题会全错位且不报错 |
| 视觉就绪（两条 `vision/detections` 有发布者） | 中止——检测节点不随栈自启 |
| 声光域号必须是 20 | 警告 |

另外起飞前还会调 `scripts/check_env.sh real` 查仿真残留。

---

## 四、首飞前仍未解决的

- **A4 UWB 原点标定**没做——不做的话坐标改对了也会整体偏移
- **G1 舵机参数**未验：NX01 的 `PWM_MAIN_FUNC7=301` / `TIM2=50` /
  `COM_PREARM_MODE=2`，以及确实接在 MAIN7
- **F 组阈值**（`ROUTE_SKIP_M` / `LANDED_MOVE_M` / 编队间距 / takeoff 静止判据）
  仍是按仿真噪声定的，建议先用 `scripts/measure_static_noise.py` 实测真机噪声

完整清单见 `../Sim2Real迁移清单与步骤.docx`。
