# 论文补充实验工具包（SE(2)在线标定）

对应 `docker_sim/实验方案_SE2在线标定论文补充实验.md`。三个文件：

| 文件 | 作用 |
|---|---|
| `se2_core.py` | 估计器算法的**纯函数副本**（分段/滑窗/闭式解/Huber），跟机上 `origin_setter_node.py` 逐行对应。改任何一边都要同步改另一边。 |
| `mc_se2_sim.py` | L0层：纯数值蒙特卡洛 E1–E6，不依赖ROS/容器 |
| `se2_calib_offline_eval.py` | L1/L2层：从rosbag离线复算，扫参数、画收敛曲线、多次重复聚合 |

结果统一写到 `docker_sim/paper_exp_results/`（CSV原始数据 + PNG插图）。

---

## L0 数值实验（现在就能跑，不用容器）

```bash
cd /home/robots/ai_uav/docker_sim
python3 scripts/paper_exp/mc_se2_sim.py            # 全套，约14秒
python3 scripts/paper_exp/mc_se2_sim.py --exp e4   # 只跑某一个
```

## L1 录包 + 离线复算

```bash
# 1) 起仿真，两机锁定起飞点、起飞、跑自标定动作或任务航线
docker compose up -d
# 2) 录包（record_rosbag.sh 已包含论文需要的4路话题）
docker exec -d flight-stack-nx01 /opt/scripts/record_rosbag.sh   # 或宿主机上跑
# 3) 离线复算
python3 scripts/paper_exp/se2_calib_offline_eval.py --self-test          # 先自检
python3 scripts/paper_exp/se2_calib_offline_eval.py runtime_logs/rosbag/bag_XXX --ns NX01
python3 scripts/paper_exp/se2_calib_offline_eval.py runtime_logs/rosbag/bag_XXX --ns NX01 --sweep
python3 scripts/paper_exp/se2_calib_offline_eval.py runtime_logs/rosbag/bag_XXX --ns NX01 --fig
```

`--sweep` 一次性出 E7(参数敏感性) / E8(批量vs滑窗) / E9(时间配对消融) /
E4b(Huber开关) / E10(残差分布)；`--fig` 出 E11 的收敛曲线（论文图5b）。

## L2 在线闭环实验（**需要先重建镜像**）

需要重建：`sim-world`（uwb_sim 改了）和 `flight-stack`（origin_setter /
rviz_goal_bridge 改了）。所有开关都走环境变量，重建之后再做实验不用改代码。

```bash
# E11 收敛精度主实验：默认参数飞N次，每次录一个包，最后聚合
docker compose up -d
#   ...飞行、录包...
python3 scripts/paper_exp/se2_calib_offline_eval.py --aggregate \
        runtime_logs/rosbag/bag_1 runtime_logs/rosbag/bag_2 ... --ns NX01

# E12 真值角度全域扫描（验证全角域无符号/象限错误）
SPAWN_YAW_DEG_NX01=90 SPAWN_YAW_DEG_NX02=180 docker compose up -d

# E13 §3.2修复前/后对照（对穿测试落点误差）
GOAL_RETRANSFORM=false docker compose up -d    # 修复前
GOAL_RETRANSFORM=true  docker compose up -d    # 修复后（默认）

# E15 观测污染闭环验证
UWB_BIAS_X=0.5 UWB_BIAS_Y=-0.3 docker compose up -d              # 恒定偏置(2.4节)
UWB_BIAS_DRIFT_X=0.01 docker compose up -d                       # 时变偏置(6.2节)
UWB_OUTLIER_PROB=0.1 UWB_OUTLIER_MAG_M=2.0 ROTATION_ROBUST=true docker compose up -d

# 观测率/时延敏感性（把仿真调到真实UWB的量级，见下面"一条重要事实"）
UWB_ABS_RATE_HZ=10 UWB_ABS_LATENCY_MS=30 docker compose up -d

# 参数敏感性也可以在线做（离线扫更省事，这里主要用于交叉验证）
ROTATION_WINDOW_SIZE=10 ROTATION_SEG_MIN_DISP_M=1.0 docker compose up -d
```

诊断话题（重建后新增）：
```bash
ros2 topic echo /NX01/origin_setter/residual_mad     # 残差MAD[m]，观测质量
ros2 topic echo /NX01/origin_setter/downweighted     # 被Huber降权的段数
ros2 topic echo /NX01/uwb/pose_truth                 # 真值旁路（只许评估用）
```

---

## 一条写论文时必须交代的事实

`/{ns}/uwb/pose_abs` 是在 `model_states_cb` 里逐帧发的，真实发布率等于world文件里
`gazebo_ros_state` 插件的 `update_rate`（本项目各world都是 **100 Hz仿真时间**），
**而且完全没有走延迟队列**。节点参数里的 `publish_rate_hz=50`/`latency_ms=30`
只作用于 `/frame_align/*` 那一路，跟 pose_abs 无关。

也就是说仿真里的绝对定位源比真实UWB乐观得多（100 Hz、零时延 vs 真实模块的
10~50 Hz、几十毫秒时延）。论文报仿真精度时必须写明这一条，或者用
`UWB_ABS_RATE_HZ=10 UWB_ABS_LATENCY_MS=30` 重跑一组做对照。
