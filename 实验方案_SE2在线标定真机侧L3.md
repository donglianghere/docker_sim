# 实验方案：局部-全局SE(2)在线标定方法 真机侧(L3)实验

> 本文档是**自足的执行手册**：布置场地、执行实验、录取数据、分析结果、生成
> 报告所需的全部信息都在这里。真机上只需要跑**一个脚本**（第4节，全文照录），
> 其余分析都在录好的包上离线做，分析脚本见 `scripts/paper_exp/l3/`（已写好并
> 用仿真数据端到端验证过，接口见第6节和附录B）。
>
> 对应论文：《基于滑窗最小二乘的局部-全局坐标系在线标定方法》。仿真侧实验
> （数值蒙特卡洛 + 双机闭环）已完成并写入论文第4章，本文档只覆盖真机侧。

---

## 1 目标：要验证论文的哪几条

论文当前在6.2节明确写着"真机侧只验证了方法族的泛化能力，机载平台上的精度与
计算开销尚未实测"。本轮真机实验就是补上这一块。

| 实验 | 验证的论文条款 | 主指标 | 判据 |
|---|---|---|---|
| **H1** 静态观测噪声与野值率 | 表1中 σ≈0.05 m 的取值依据；2.7节抗差扩展的动机 | 水平位置样本标准差、野值率 | 水平σ ≤ 0.10 m；LOS下野值率≈0 |
| **H2** 双标签测向精度 | 4.6节式(21) σ_yaw ≈ √2σ/L | 静止时yaw的样本标准差 | 实测/预测 ∈ [0.5, 2.0]；σ_yaw(M)·√M 随块平均基本恒定 |
| **H3** θ\*在线标定精度 | 4.3.1节的真机对照；6.2节"机载精度未实测" | 收敛后RMSE、收敛所需段数 | RMSE ≤ 3°；真值标准误 < RMSE/2 |
| **H4** 参数敏感性/批量对比/时间配对 | 4.3.3、4.2.3、4.2.6节的真机对照 | RMSE随W、d_min的变化 | W增大时RMSE单调下降并进入平台 |
| **H5** 抗差扩展 | 2.7节、4.2.4、4.3.5节 | 同一份数据开/关抗差的RMSE | 注入野值下改善 ≥1.5倍 |
| **H6** 机载计算开销 | 4.3.6节的真机对照；6.2节"计算开销未实测" | 单次更新耗时 | 切段路径 ≤ 2000 µs |

**本轮不覆盖**（只有一台真机）：机间链式复合（论文4.3.4的真机版）。报告里要
如实写明。

---

## 2 前提条件与上电自检

### 2.1 硬件

- Jetson Orin NX 16G（JetPack 6.2）+ PX4 v1.16 飞控
- UWB：nooploop LinkTrack 双标签（`uwb_a` 走 USB/CH340，`uwb_b` 走板载 UART），
  锚点阵列已完成标定并给出统一世界坐标系
- 激光雷达 Mid-360（DLIO 或 point_lio 的输入）
- 遥控器（**可选**：默认手持采集不需要；只有做5.0节建议的飞行对照架次才用）
- **卷尺**（量基线，精度到 mm）、**一块硬直板**（H2 用来把两标签拉长基线）

### 2.2 软件与接口自检（每次上电都要做一遍）

进入飞控容器（真机服务名 `flight-stack-hw`）：

```bash
docker exec -it flight-stack-hw bash
# ROS CLI 必须自己补齐 DDS 环境，否则 ros2 topic list 只能看到 /rosout
# 正确的值从正在跑的节点进程里读，不要凭记忆写
pid=$(pgrep -f origin_setter | head -1)
tr '\0' '\n' < /proc/$pid/environ | grep -E 'RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DOMAIN_ID'
# 按读到的值导出，例如：
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/docker_sim_cyclonedds.xml
```

逐条确认下面这些话题都在发布（`ros2 topic hz <话题>` 看频率，不要只看 list）：

| 话题 | 类型 | 说明 | 期望频率 |
|---|---|---|---|
| `/NX01/dlio/odom_node/odom` | `nav_msgs/Odometry` | 局部位姿，估计器输入 | **实测 ~19 Hz**（2026-09-03，DLIO，Jetson load≈10；旧记录里的~100 Hz已不成立） |
| `/NX01/uwb/pose_abs` | `geometry_msgs/PoseStamped` | 双标签融合后的绝对位置（+yaw在orientation里） | **实测 ~48 Hz** |
| `/NX01/uwb_a/pose_abs` | `geometry_msgs/PoseStamped` | 标签a原始位置 | 实测 ~48 Hz |
| `/NX01/uwb_b/pose_abs` | `geometry_msgs/PoseStamped` | 标签b原始位置 | 实测 ~48 Hz |
| `/NX01/origin_setter/yaw_estimate` | `std_msgs/Float64` | 机上 θ\* 输出（弧度） | 每切一段发一次 |
| `/NX01/origin_setter/yaw_sample_count` | `std_msgs/Int32` | 滑窗内段数 | 同上 |
| `/NX01/origin_locked` | `std_msgs/Bool` | 起飞点是否已锁定 | 1 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | 含 `world → NX01/map`，即标定出的 (θ\*,t) | 每次θ\*更新重发 |

服务：`/NX01/set_origin_from_uwb`（`std_srvs/Trigger`）。

**必须现场核实的三件事**（`nlink_pose_bridge_node.py` 文件头列的未验证假设）：

1. **协议帧类型每次上电可能变**（`nodeframe2` ↔ `tagframe0`）。用
   `ros2 topic info /NX01/uwb_a/nlink_linktrack_nodeframe2` 之类逐个确认，
   两路标签可能不一样。
2. `pos_3d` 单位是否为米（拿卷尺量一个已知距离对一下量级）。
3. `quaternion` 分量顺序（本实验只用 position 与融合后的 yaw，影响有限）。

### 2.3 与仿真侧的接口差异（会影响分析）

| 项 | 仿真 | 真机 |
|---|---|---|
| `uwb/pose_truth` 真值旁路 | 有 | **没有** → 见第3节的真值方案 |
| 观测污染注入参数（`UWB_OUTLIER_*` 等） | 有 | 没有 → 用真实遮挡 + 离线注入替代 |
| `residual_mad` / `downweighted` 诊断话题 | 有 | **本轮真机镜像没有** → 全部改为离线计算 |
| `ROTATION_ROBUST` 机上抗差开关 | 有 | **本轮真机镜像没有** → 抗差在离线复算里开关 |

> **因此本方案不要求重建真机镜像。** 所有分析（残差统计、Huber抗差、W/d_min
> 扫描、批量对比）都在录好的包上离线完成。只有想验证"机上闭环抗差"时才需要
> 重建，那是可选项（第5.7节）。

---

## 3 真值方案（不需要动捕/全站仪）

真机没有真值旁路，但三个实验各自都有不依赖外部设备的解法：

**H1、H2 根本不需要真值。** σ（H1）和 σ_yaw（H2）都是**离散度**指标——把设备
放着不动采一段数据，样本标准差本身就是被测量。H2 里式(21)预测的也是标准差，
不是均值，所以机体朝向摆得准不准完全不影响结论。

**H3 的真值 = 双标签测出的机体朝向 − DLIO 报出的机体朝向。**

$$\theta_{true}(t) = \psi_{dualtag}(t) - \psi_{dlio}(t)$$

这条路径与被测量（θ\*）**完全独立**：前者靠两个标签的位置差算角度，后者靠
"局部位移与全局位移的相关性"算角度，二者除了共用同一批 UWB 锚点之外没有任何
共享环节。单帧噪声按式(21)约 14°，但 θ_true 本身近似恒定（局部系到全局系的
旋转），对 N 帧取平均后标准误降到 14°/√N —— 100 Hz 下飞 3 分钟 N≈18000，
标准误约 0.1°，比被测的 θ\* 精度（预期 1–3°）好一个数量级以上。

**两个必须注意的陷阱**：

1. **必须逐帧先作差、再平滑**，不能先分别平滑 ψ 再作差。飞行中机体本身在偏航，
   ψ_dualtag 和 ψ_dlio 各自都在大幅变化，先平滑再作差会在转弯段引入几十度误差。
   （这个坑在开发分析脚本时实测踩过：错误做法算出的 RMSE 是 19.37°，改对之后
   是 0.67°。）
2. **报告里必须同时给出真值自身的标准误**，并确认它小于 RMSE 的一半——否则
   测出来的"误差"里主要是真值噪声，不是估计器的误差。分析脚本会自动算这一项
   并在判据里检查。

---

## 4 机上唯一要跑的脚本：`l3_record.sh`

真机上要做的事只有三件：**锁起飞点 → 手动飞 → 录包**。下面这个脚本把这三件事
连同录制前后的自检都包了。存到真机 `flight-stack-hw` 容器里能访问的位置
（放 `/logs/l3_record.sh` 最省事，`/logs` 是挂载出去的目录），`chmod +x` 后使用。

```bash
#!/usr/bin/env bash
# 真机侧(L3)实验的唯一机上脚本：录一段实验数据。
#
# 用法（在真机的 flight-stack-hw 容器里跑）：
#   ./l3_record.sh <标签> <时长秒> [命名空间] [--lock]
# 例：
#   ./l3_record.sh h1_static_los 600            # H1 静态10分钟
#   ./l3_record.sh h2_base029    180            # H2 基线0.29m静止3分钟
#   ./l3_record.sh h3_run1       300 NX01 --lock # H3 先锁起点再录5分钟(手持走或飞)
#
# 标签重名保护：目标目录已存在时默认拒绝覆盖并退出，确认要丢弃旧数据才加 --force。
#
# 产物：/logs/l3/<标签>/    （/logs 是挂载到宿主机的目录，包直接落在机外可取）
# 录完会自动打印每个话题的帧数自检，当场就能判断这包能不能用，不用等回去分析。
# ⚠️ 这里不能开 set -u：ROS 的 /opt/ros/humble/setup.bash 内部会引用
# AMENT_TRACE_SETUP_FILES 等未定义变量，开了 -u 一 source 就直接退出
# （报 "AMENT_TRACE_SETUP_FILES: unbound variable"）。脚本里所有变量引用
# 都已经写成 ${VAR:-默认值} 或由参数校验保证，不依赖 -u 来兜底。
set -o pipefail

TAG="${1:?用法: l3_record.sh <标签> <时长秒> [命名空间] [--lock]}"
DUR="${2:?缺时长(秒)}"
NS="${3:-NX01}"
LOCK=""
FORCE=""
for arg in "$@"; do
    [ "$arg" = "--lock" ]  && LOCK=1
    [ "$arg" = "--force" ] && FORCE=1
done

OUT="/logs/l3/${TAG}"

# --- ROS/DDS 环境：docker exec 进来的 shell 不带节点进程里的那几个变量，
#     必须自己补齐，否则 ros2 命令看不到任何话题。取值从正在跑的节点进程里读，
#     不写死，免得跟部署配置漂移。
source /opt/ros/humble/setup.bash
source /opt/px4ctrl_ws/install/setup.bash 2>/dev/null || true
# 依次尝试几个必定在跑的节点，谁在就从谁的环境里抄
for pat in origin_setter dlio_odom_node mavros_node point_lio; do
    PID=$(pgrep -f "${pat}" | head -1)
    [ -z "${PID}" ] && continue
    [ -r "/proc/${PID}/environ" ] || continue
    eval "$(tr '\0' '\n' < /proc/${PID}/environ | grep -E '^(RMW_IMPLEMENTATION|CYCLONEDDS_URI|ROS_DOMAIN_ID)=' | sed 's/^/export /')"
    echo "== DDS环境取自进程 ${pat}(pid=${PID}) =="
    break
done
echo "== DDS: RMW=${RMW_IMPLEMENTATION:-未设置} DOMAIN=${ROS_DOMAIN_ID:-未设置} =="
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
    echo "   ⚠️ 没能自动取到DDS环境（飞控栈可能还没起来）。若下面话题自检全是[缺]，"
    echo "      先确认容器里的节点在跑，再手动 export RMW_IMPLEMENTATION / CYCLONEDDS_URI"
    echo "      / ROS_DOMAIN_ID 之后重跑本脚本。"
fi

# --- 要录的话题。缺哪一路只警告不中断：H1/H2 只需要其中一部分。
TOPICS=(
  "/${NS}/dlio/odom_node/odom"              # 局部位姿，估计器输入
  "/${NS}/uwb/pose_abs"                     # 双标签融合后的绝对位置(+yaw)
  "/${NS}/uwb_a/pose_abs"                   # 标签a原始位置（H2必需）
  "/${NS}/uwb_b/pose_abs"                   # 标签b原始位置（H2必需）
  "/${NS}/origin_setter/yaw_estimate"       # 机上θ*，用来跟离线复算对照
  "/${NS}/origin_setter/yaw_sample_count"
  "/${NS}/origin_locked"
  "/${NS}/mavros/state"
  "/tf" "/tf_static"
)

echo "== 录制前话题自检 =="
LIVE=$(ros2 topic list 2>/dev/null)
MISS=0
for t in "${TOPICS[@]}"; do
    if echo "${LIVE}" | grep -qx "$t"; then
        echo "   [有] $t"
    else
        echo "   [缺] $t"
        MISS=$((MISS+1))
    fi
done
[ "${MISS}" -gt 0 ] && echo "   ⚠️ 有${MISS}个话题不存在，相关实验的分析会缺数据（H2必须要uwb_a/uwb_b两路）"

# --- 可选：录制前锁定起飞点（H3 必须做，且必须在**静止**时做）
if [ -n "${LOCK}" ]; then
    echo "== 锁定起飞点（务必确认飞机此刻完全静止：放在地上/桌面，人离开一步）=="
    ros2 service call "/${NS}/set_origin_from_uwb" std_srvs/srv/Trigger 2>&1 | tail -2
    sleep 1
fi

mkdir -p /logs/l3
# 同名标签默认**拒绝覆盖**：现场重录一次的代价远小于把上一架次的数据删掉。
# 确实要丢弃旧数据时显式加 --force。
if [ -e "${OUT}" ]; then
    if [ -n "${FORCE}" ]; then
        echo "== ${OUT} 已存在，--force 生效，删除后重录 =="
        rm -rf "${OUT}"
    else
        echo "!! ${OUT} 已存在，里面是上次用同一个标签录的数据，拒绝覆盖。"
        echo "   现有内容："
        ls -la "${OUT}" 2>/dev/null | sed 's/^/     /'
        echo "   处理方式二选一："
        echo "     · 换一个标签重跑（推荐，例如加 _b 后缀）"
        echo "     · 确认旧数据不要了，在命令末尾加 --force 重跑"
        exit 1
    fi
fi
echo "== 开始录制 ${DUR} 秒 -> ${OUT} =="
# ⚠️ 必须 -s INT：timeout 默认发 SIGTERM，而 ros2 bag record 只对 SIGINT 做
# 优雅收尾（写完 metadata.yaml 再退出），收到 SIGTERM 会一直挂着不退，表现为
# "到点了还卡在录制中"。-k 10 是保险：发完 INT 再等10秒还不退就强杀。
# ⚠️ `< /dev/null` 不能省：ros2 bag record 会读 stdin 做键盘控制（空格暂停），
# 在 `docker exec -it` 分配的 TTY 下它属于后台进程组，一读 TTY 就收到 SIGTTIN
# 被停住（ps 里状态是 T），表现为"开始录制之后再无输出、也不生成bag目录"，
# 而且 timeout 也杀不掉它。重定向 stdin 之后读到 EOF，键盘控制自动失效。
timeout -s INT -k 10 "${DUR}" ros2 bag record -o "${OUT}" "${TOPICS[@]}" \
        < /dev/null > "/tmp/l3_bag_${TAG}.log" 2>&1
echo "== 录制结束 =="

if [ ! -d "${OUT}" ]; then
    echo "!! 没有生成 ${OUT}，ros2 bag record 的日志末尾："
    tail -20 "/tmp/l3_bag_${TAG}.log" 2>/dev/null | sed 's/^/   /'
    exit 1
fi

# --- 录完立刻自检：帧数够不够、是不是每路都有数据
echo "== 数据自检 =="
ros2 bag info "${OUT}" 2>/dev/null | sed -n '/Topic information/,$p' | sed 's/^/   /'
echo
echo "自检要点："
echo "  · odom 与 uwb/pose_abs 的 Count 应当 ≈ 各自频率×时长"
echo "    （2026-09-03 真机实测：odom≈19Hz、uwb≈48Hz，即20秒约380条/960条）"
echo "    某一路为0说明该链路没通；明显低于上面的量级说明该节点在掉帧"
echo "  · H2 还需要 uwb_a / uwb_b 两路都非空"
echo "  · H3 还要看 yaw_sample_count 的末值：<50 说明飞的路径不够长，"
echo "    需要累计位移≥30米（3米见方来回约10趟），当场补录一次"
echo
echo "把 ${OUT} 整个目录拷回分析机，用 l3_analyze.py 处理。"
```

### 4.1 在真机上执行

真机部署目录约定为 `~/docker_sim`，ssh 别名假定为 `nx01`，按实际替换。
`docker-compose.hw.yml` 里挂的是 `./runtime_logs:/logs`，所以**只要脚本放在真机的
`~/docker_sim/runtime_logs/` 下，容器里就是 `/logs/l3_record.sh`，不需要 scp
也不需要 docker cp**。

```bash
# ---------- ssh 到真机 ----------
ssh nx01
cd ~/docker_sim

# 1) 确认飞控栈在跑
docker compose -f docker-compose.hw.yml ps

# 2) 确认容器里看得到脚本（挂载是否生效，一眼就知道）
docker compose -f docker-compose.hw.yml exec flight-stack-hw ls -l /logs/l3_record.sh

# 3) 执行（脚本阻塞到录完，要在能看着的终端里跑）
docker compose -f docker-compose.hw.yml exec flight-stack-hw \
    bash /logs/l3_record.sh h1_center 600
```

用 `bash /logs/...` 调用，所以不需要给脚本加可执行权限。

**知道容器名之后更省事**：直接用 `docker exec`，不必先 `cd` 到 compose 文件
所在目录，命令也短得多（容器名一般是 `docker_sim-flight-stack-hw-1`，用
`docker ps --format '{{.Names}}'` 确认）：

```bash
C=docker_sim-flight-stack-hw-1
docker exec -it $C bash /logs/l3_record.sh h1_center 600
```

下面各实验的调用统一用这个短形式。

**怕 ssh 断线把录制打断**（脚本是前台阻塞的，ssh 一断就没了）：套一层 tmux，
断线后重连 `tmux attach -t l3` 接着看：

```bash
ssh nx01 -t "tmux new -As l3"
# 然后在 tmux 里执行上面第3步的命令
```

**万一 `ls /logs/l3_record.sh` 看不到**（挂载路径跟这里假设的不一致、或者
`runtime_logs/` 是 Docker 建的 root 属主目录、普通用户写不进去），退回用
`docker cp`，不需要 sudo：

```bash
docker ps --format '{{.Names}}'      # 一般是 docker_sim-flight-stack-hw-1
docker cp l3_record.sh docker_sim-flight-stack-hw-1:/tmp/l3_record.sh
docker compose -f docker-compose.hw.yml exec flight-stack-hw bash /tmp/l3_record.sh h1_center 600
```

做 H6（Jetson 计算开销）时，`l3_analyze.py` 和 `l3_core.py` 也按同样方式放到
`runtime_logs/` 下即可。

**各实验的具体调用**（`C=docker_sim-flight-stack-hw-1`，下同）：

```bash
# H1 静态：不飞、不锁，放着不动。这一段同时供H2用（条件一样：静止+水平），
# 不必再单独录 h2_base029
docker exec -it $C bash /logs/l3_record.sh h1_center 600
docker exec -it $C bash /logs/l3_record.sh h1_corner 300
docker exec -it $C bash /logs/l3_record.sh h1_nlos   300

# H2 双标签测向：本轮只做机上原装基线。上面的 h1_center 那段就够用，
# 只有想单独再录一段时才需要这条
docker exec -it $C bash /logs/l3_record.sh h2_base029 180

# H3 主实验：飞机放地上静止 -> 跑命令(它先锁起点) -> 拿起来水平举着走3-5分钟
docker exec -it $C bash /logs/l3_record.sh h3_run1 300 NX01 --lock
# ... 共5次

# H5 NLOS：另一个人走动遮挡
docker exec -it $C bash /logs/l3_record.sh h5_nlos 300 NX01 --lock

# H6 只在 Jetson 上跑一次（不录包，直接分析已有的包）
docker exec -it $C bash -c 'source /logs/l3_env.sh; \
    python3 /logs/l3_analyze.py timing /logs/l3/h3_run1 --tag jetson \
            --platform "Jetson Orin NX 16G / JetPack 6.2"'
```

**把数据取回开发机**（bag 是 Docker 建的 root:root，但目录 755、文件 644，
普通用户能读，不需要 sudo）：

```bash
# 在开发机上
rsync -av nx01:~/docker_sim/runtime_logs/l3/ /home/robots/ai_uav/docker_sim/runtime_logs/l3_hw/
# 没有 rsync 就用： scp -r nx01:~/docker_sim/runtime_logs/l3 ./runtime_logs/l3_hw
```

之后的分析和报告全部在开发机上离线做，见第6节。

**同名标签不会被悄悄覆盖**：目标目录 `/logs/l3/<标签>` 已存在时脚本直接拒绝并退出，列出已有内容、提示换标签或显式加 `--force`。现场重录一次的代价远小于把上一架次的数据删掉。

**为什么录制前后都要自检**：真机上最贵的是飞行本身。协议帧类型每次上电可能变、
某一路标签可能没上电、DDS 环境没配好都会让整包数据作废，而这些在现场花十秒钟
就能发现，回到分析机上才发现就要重飞。脚本录完会直接打印每个话题的帧数，
以及 `yaw_sample_count` 末值——这个值 <50 就说明飞的路径不够长，当场补飞即可。

---

## 5 实验现场步骤

### 5.0 采集方式、通用要求与安全

**本方案的采集方式默认是"手持整机步行"，不是飞行。** 本文方法的估计器只消费
两路数据——局部里程计（DLIO，激光+IMU）和 UWB 绝对位置——**跟飞控闭环、
电机、遥控完全无关**。手持走与飞行在这两路数据上是等价的，而且手持有三个实
际好处：安全（不转桨）、不受电池续航限制、可以走出远大于 3 m 见方的路径，
方向多样性和累计位移都更容易做够。

**手持采集必须守的三条**（不守会直接污染结论）：

| 要求 | 为什么 |
|---|---|
| **保持机体大致水平**（横滚/俯仰目视 ±10° 以内） | 本文方法是 SE(2)，前提就是机体近似水平。倾斜会同时破坏两件事：DLIO 的雷达/IMU 外参标定假设，以及双标签测向——两标签连线倾斜后投影到水平面的有效基线变短，σ_yaw 按式(21) 随之变大，而你以为用的还是 0.29 m |
| **高度大致恒定**（1.0–1.5 m，齐胸举着即可） | 同上，高度剧烈起伏会扰动 DLIO，也让双标签的水平投影不稳定 |
| **人体不要挡在标签与锚点之间** | 抱在胸前时人体正好是最大的遮挡物。建议把飞机举到身前一臂远或过头顶，转身时注意别用身体扫过某个锚点方向。这一条对 H1/H3 是干扰，但正是 H5 要故意制造的条件 |

**其余通用要求**：

- 场地 3 m × 3 m 起步；若 UWB 锚点覆盖范围更大，手持模式下走更大的范围更好
- UWB 标签与锚点保持视距（H5 的 NLOS 架次除外）
- 每次上电后先做第2.2节的接口自检，再开始实验
- **建议至少补 1–2 架次真实飞行做对照**：手持采集覆盖不到"旋翼振动 + 电机
  电磁干扰"这一层，而这两者会同时影响 UWB 测距质量和 IMU/DLIO。做一次飞行
  架次、与手持架次比较 H1 的 σ 和 H3 的 RMSE，就能确认手持结论能不能外推到
  飞行状态；若二者一致，论文里可以直接引用手持数据，若不一致则必须分别报告。
- 论文写作口径：**采集方式必须如实写明**（"手持整机步行采集"而非"飞行"）。
  核心估计器的结论不受影响（它不依赖飞控），但"飞行中验证"这个更强的说法
  只有做了飞行架次才能用。

### 5.1 H1 静态观测噪声与野值率

飞机通电、**旋翼不转**、静止放置。分三个位置各录一次：

| 架次标签 | 摆放位置 | 时长 | 备注 |
|---|---|---|---|
| `h1_center` | 场地中央 | 600 s | 主数据，σ 取这一段 |
| `h1_corner` | 场地角落（离最近锚点最远处） | 300 s | 看噪声是否随位置变化 |
| `h1_nlos` | 场地中央，但让一个人站在标签与某个锚点之间不动 | 300 s | 制造真实多径/非视距，给H5用 |

```bash
./l3_record.sh h1_center 600
./l3_record.sh h1_corner 300
./l3_record.sh h1_nlos   300
```

### 5.2 H2 双标签测向精度（式21验证）

**本轮只做机上原装基线 L≈0.29 m 这一个配置**（拆标签换基线暂时做不了）。
静止录一段即可，不需要真值：

```bash
./l3_record.sh h2_base029 180
```

要点：
- 飞机静止、**水平**放置（倾斜会缩短两标签连线在水平面上的投影，等于偷偷改了 L）
- 用卷尺量两个标签**天线中心**的距离，记到 mm，分析时用 `--baseline` 传实测值
- 录前确认两路标签的协议帧类型（`uwb_b` 重新上电后帧类型会变）

**换不了基线，怎么补回"标度律"这层验证**：式(21) 是 σ_yaw ≈ √2σ/L，分母 L
动不了，但**分子可以动**——把连续 M 帧的两标签位置各自块平均后再算 yaw，等效
单站噪声从 σ 降到 σ/√M，于是 σ_yaw(M)·√M 应当是常数。分析脚本会自动扫
M ∈ {1,2,5,10,20,50} 并给出这条曲线，用的还是同一段数据，不需要任何额外动作。

这条检验有两种结果，都有用：
- **σ_yaw·√M 基本恒定（相对离散度 <30%）**：式(21) 分子上的 √2σ 成立，噪声
  近似白，验证目的达成；
- **σ_yaw·√M 随 M 明显上升**（块平均降不下去）：说明 UWB 解算输出在时间上
  相关（100 Hz 的发布率背后并没有 100 Hz 的独立信息），有效独立采样率远低于
  发布率。这本身是个重要发现——它同时意味着 H3 里"真值平均 N 帧后标准误
  降到 14°/√N"这个说法要按有效独立帧数打折，报告里必须据此重估。

**遗留**：跨基线的标度律（L 变化时 σ_yaw×L 是否恒定）本轮不做，写进报告的
"未覆盖项"。等能拆装标签时补 0.6 m 与 1.0 m 两个配置即可，分析命令不变。

### 5.3 H3 θ\*在线标定精度（核心实验）

**前置**：飞机静止放在起点上（手持模式下就是放在地上或桌面上，人离开一步），
先锁一次起飞点（脚本 `--lock` 参数会做），**锁定之后不要再触发第二次**，
锁定完成后再把飞机拿起来开始走。

**运动要求**（手持或飞行都一样，决定这包数据能不能用）：

| 要求 | 数值 | 为什么 |
|---|---|---|
| 累计路径长度 | **≥30 m** | 滑窗 W=50、d_min=0.5 m，装满窗口需要25 m，留裕量 |
| 移动速度 | 0.3–0.8 m/s（正常步速的一半左右） | 太快则单段跨度大、DLIO 容易掉；太慢则时间不够 |
| 高度 | 1.0–1.5 m 大致恒定 | 本方法是 SE(2)，高度变化不参与估计但会扰动 DLIO |
| 姿态 | 横滚/俯仰目视 ±10° 以内 | 见5.0节：倾斜会同时污染 DLIO 外参假设和双标签有效基线 |
| 方向多样性 | 纵向、横向、对角都走 | 见论文第5节：方向多样性用于把里程计系统性偏置与真实旋转角分开 |
| 时长 | 3–5 分钟 | 同时保证真值平均的样本量 |

3 m 见方内怎么走够 30 m：**来回横穿约 10 趟**（每趟 2.5–3 m），中间穿插两次
对角线和一次绕方形。手持走不需要精确，反而比自动方形航线的方向多样性更好。
**注意机体朝向要随走向自然变化**（或有意转几次身）——θ 的真值靠"双标签朝向
减 DLIO 朝向"求得，机体一直不转的话这个差值虽然仍然成立，但少了一层交叉检验；
转身几次可以顺带确认两路朝向确实在同步变化、真值链路没接反。

**重复 ≥5 架次**（论文里所有仿真结果都是 N=5 的 mean±std，真机要对齐这个口径）。

```bash
./l3_record.sh h3_run1 300 NX01 --lock
# 降落、换电、重新静止，再来一次
./l3_record.sh h3_run2 300 NX01 --lock
# ... 共5次
```

飞完当场看脚本打印的 `yaw_sample_count` 末值：<50 就补飞。

### 5.4 H4 参数敏感性 / 批量对比 / 时间配对

**不需要额外飞行**，直接复用 H3 的包离线扫描。

### 5.5 H5 抗差扩展

两部分，第一部分需要一次专门的飞行架次：

```bash
# 采集过程中让**另一个人**在场地边缘来回走动、间歇性遮挡标签与锚点之间的视线
# （手持模式下持机人自己不要故意遮挡，否则遮挡与运动完全同步，分不清是哪个
#   因素导致的残差抬升）
./l3_record.sh h5_nlos 300 NX01 --lock
```

第二部分是在干净架次（H3 的包）上离线人工注入野值，把污染比例推到真实环境
达不到的水平，检验崩溃点——不需要飞行。

### 5.6 H6 机载计算开销

**必须在 Jetson 上跑**（测的就是这块板子的耗时）。把分析脚本拷到真机容器里，
对任意一个 H3 的包跑 `timing` 子命令即可；同时用另一个终端记录
`origin_setter` 进程的 CPU 占用：

```bash
# 终端1（容器内）
python3 l3_analyze.py timing /logs/l3/h3_run1 --tag jetson --platform "Jetson Orin NX 16G / JetPack 6.2"
# 终端2（真机宿主机）：采集期间同时采一段进程CPU
top -b -n 60 -d 1 | grep -E "origin_setter" | awk '{print $9}' > /tmp/origin_setter_cpu.txt
```

### 5.7 （可选）H7 机上闭环抗差

只有想验证"抗差在机上闭环运行"时才需要，**需要先用带本轮改动的代码重建真机
flight-stack 镜像**。重建后 `ROTATION_ROBUST=true` 起容器，重复一次 H5 的
NLOS 架次，比较机上 `yaw_estimate` 末值与同一份数据离线关抗差的复算结果。
不做这一项不影响论文结论（离线的受控对照证据更强，因为是同一份数据）。

---

## 6 离线分析与报告

把 `/logs/l3/` 整个目录拷回分析机。分析脚本三个文件（`l3_core.py`、
`l3_analyze.py`、`l3_report.py`）在 `scripts/paper_exp/l3/` 下，只依赖
`rosbag2_py` + `rclpy`（容器里自带），画图需要 matplotlib（没有则自动跳过）。

```bash
cd <放结果的工作目录>          # 结果会写到 ./results/ 下

# H1：三段静态数据各跑一次
python3 l3_analyze.py static /path/h1_center --tag center
python3 l3_analyze.py static /path/h1_corner --tag corner
python3 l3_analyze.py static /path/h1_nlos   --tag nlos

# H2：--baseline 填卷尺实测值，--sigma 填H1实测的水平σ；
#     块平均标度检验会自动跑，不用额外命令
python3 l3_analyze.py yaw /path/h2_base029 --tag base029 --baseline 0.291 --sigma 0.048

# H3：5个架次各跑一次（--plot 出收敛曲线）
for i in 1 2 3 4 5; do
  python3 l3_analyze.py theta /path/h3_run$i --tag run$i --plot
done

# H4：任选一个代表性架次
python3 l3_analyze.py sweep /path/h3_run1 --tag run1

# H5：真实NLOS架次 + 干净架次上人工注入
python3 l3_analyze.py robust /path/h5_nlos --tag nlos_real
python3 l3_analyze.py robust /path/h3_run1 --tag inject5pct --inject-prob 0.05 --inject-mag 2.0
python3 l3_analyze.py robust /path/h3_run1 --tag inject20pct --inject-prob 0.20 --inject-mag 2.0

# H6：在Jetson上跑（见5.6）
python3 l3_analyze.py timing /path/h3_run1 --tag jetson

# 汇总成报告
python3 l3_report.py --out 真机侧L3实验报告.md
```

`l3_report.py` 会扫描 `results/*.json`，生成带结果表、与仿真侧对照、判据核对
（✅/❌ 逐条）的 Markdown 报告，最后留一段"结论与遗留"按实际结果填写。

---

## 6.5 2026-09-03 首次实测得到的现场经验（后续架次必须遵守）

第一次真机实测暴露了四条会直接毁掉数据的现场要求，写在这里避免重复踩：

**1. 举在胸口/齐肩高度（1.2~1.5 m），不要举过头顶。**
举过头顶时 Mid-360 主要扫到天花板，无纹理大平面让 GICP 找不到对应点。实测
架次2录到 **670 条 `GICP LOST TRACK`**，DLIO 自报走了 318 m 而 UWB 说每段只动
0.42 m，70% 的位移段长度比离谱，整个架次作废。举在胸口的架次1完全正常
（|v|/|u| 中位数 1.05，异常段仅 2%）。而且举过头顶本身就偏离了论文要验证的
工况——真机飞行时雷达扫的是墙面，不是天花板。

**2. 真值必须与架次同时测（"夹心"协议）。**
实测 DLIO 偏航在 22 分钟内漂了约 9°（三次 θ_ref：+81.8° / +91.3° / +89.3°）。
拿架次开始前很久测的参考值去比，会凭空多出近 10° 的"误差"。正确做法：

```
θ_ref(A/B各60s) → 架次(5min) → θ_ref(A/B各60s) → 下一架次 → ...
```

相邻架次共用参考，该架次真值取前后两次的均值，两次之差就是这段时间的漂移量，
也是该架次真值不确定度的诚实估计。

**3. θ_ref 的两个端点必须同一朝向、人要退开。**
θ_ref = UWB位移方向 − DLIO位移方向。它不依赖机头指向、不需要机械对准、
恒定偏置在差分里自动消掉（论文2.4节的性质），但两端朝向不一致时机体固连的
偏置就不再抵消。实测三次 θ_ref 在不同位置、不同朝向下重复到 ~2°。

**4. 双标签 yaw 不能当真值（除非先修 tag_b）。**
实测两标签物理距离 0.28 m，解算间距随朝向在 **0.27~0.92 m** 之间变化，
只有朝向 ≈+85~95° 附近才是正确的 0.28 m。异常朝向下 tag_a 只偏 0.06 m 而
**tag_b 偏 0.64 m**（被推离锚点，典型的测距整体偏长）。双标签 yaw 因此带
±15° 以上的朝向相关偏置。用 `l3_sep_monitor.py` 可以现场实时看间距找健康朝向：

```bash
docker exec -it <容器> bash -c 'source /logs/l3_env.sh; python3 /logs/l3_sep_monitor.py'
```

**判断一个架次能不能用的两个数**（离线算，`l3_analyze.py` 会给）：
每段 |u| 应在 0.5~0.7 m（分段阈值 0.5 m），|v|/|u| 中位数应接近 1。
|u| 明显偏大 = DLIO 在跳；|v|/|u| 偏离 1 太多 = 两路数据有一路不可信。

---

## 7 判据汇总

| 编号 | 判据 | 不通过时怎么办 |
|---|---|---|
| H1 | 静态水平 σ ≤ 0.10 m | 若实测明显大于 0.05 m，论文表1的σ取值与"依据"一列要改引本报告，且4.2.1节的理论对照要用新的σ重算 |
| H1 | LOS 下野值率 ≈ 0，NLOS 下显著升高 | 若 LOS 下野值率就很高，说明锚点布局或标定有问题，先解决再继续 |
| H2 | 实测 σ_yaw / 式(21)预测 ∈ [0.5, 2.0] | 先核对基线长度是否量准、两标签是否同步；仍不符则式(21)在真机上不成立，论文4.6节要如实修正 |
| H2 | σ_yaw(M)·√M 随块平均的相对离散度 < 30% | 离散度大说明UWB噪声时间相关，需按有效独立帧数重估H3的真值不确定度 |
| H3 | 收敛后 RMSE ≤ 3° | 检查累计位移是否够、DLIO 是否掉过、UWB 是否有长时间遮挡（手持模式下常见的是被持机人自己的身体挡住） |
| H3 | 真值标准误 < RMSE/2 | 不满足说明测的是真值噪声，需要延长飞行时间增大 N |
| H4 | W 增大时 RMSE 单调下降并进入平台 | 若不单调，检查该架次是否有 DLIO 跳变 |
| H5 | 注入 5% 野值下抗差改善 ≥1.5 倍 | 若改善不明显，先看残差统计确认野值是否真的进了滑窗 |
| H6 | 切段路径耗时 ≤ 2000 µs | 超了要核对是不是 CPU 被其它节点占满 |

判据阈值集中写在 `l3_report.py` 顶部的 `CRITERIA` 字典里，要调只改那一处。

---

## 8 常见故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| 打印“开始录制”之后再无输出、也不生成 bag 目录，`ps` 里 `ros2 bag` 进程状态是 `T` | `ros2 bag record` 会读 stdin 做键盘控制，在 `docker exec -it` 的 TTY 下属于后台进程组，读 TTY 即收到 SIGTTIN 被停住，连 `timeout` 也杀不掉 | 录制命令必须带 `< /dev/null`（脚本已加）。已经卡住的用 `docker exec <容器> pkill -9 -f "ros2 bag record"` 清掉 |
| 到点了还卡在“开始录制”、迟迟不打印“录制结束” | `timeout` 默认发 SIGTERM，而 `ros2 bag record` 只对 SIGINT 做优雅收尾 | 脚本已改成 `timeout -s INT -k 10`；已经卡住的那次按 Ctrl-C 中断即可，数据其实已经写进去了，用 `ros2 bag info` 确认后可以照常用 |
| 容器里跑 python 工具报 `ModuleNotFoundError: No module named 'rclpy'` | `docker exec` 是全新 shell，没有 source ROS 环境 | 先 source 环境外壳：`docker exec -it <容器> bash -c 'source /logs/l3_env.sh; python3 /logs/xxx.py ...'`。`l3_record.sh` 自己内部已经做了这件事，所以它不受影响 |
| 一运行脚本就报 `AMENT_TRACE_SETUP_FILES: unbound variable` | 脚本开了 `set -u`，而 ROS 的 `setup.bash` 内部引用了未定义变量 | 脚本已改成 `set -o pipefail`（不开 `-u`）。用的是旧版就在真机上执行 `sed -i 's/^set -uo pipefail/set -o pipefail/' <脚本路径>` |
| `ros2 topic list` 只有 `/rosout` 和 `/parameter_events` | `docker exec` 的 shell 没有 DDS 环境变量 | 按第2.2节从节点进程 `/proc/<pid>/environ` 读出并导出 |
| `uwb_a` 或 `uwb_b` 没数据 | 标签协议帧类型上电后变了（`nodeframe2`↔`tagframe0`） | 用 `ros2 topic list \| grep nlink` 看实际帧类型，改 `frame_type` 参数重启 |
| 锁定起飞点失败："样本不足" | UWB 发布率不够或话题没通 | 先 `ros2 topic hz /NX01/uwb/pose_abs` 确认 |
| 锁定失败："抖动过大" | 飞机没静止，或 UWB 信号质量差 | 等飞机完全静止；检查锚点视距 |
| 离线分析报"没有切出任何位移段" | 累计位移不够，或两路数据时间戳对不上 | 看 `yaw_sample_count`；分析脚本已统一用 bag 接收时间戳，不受时钟域问题影响 |
| H3 的 RMSE 异常大（>10°） | 多半是真值算错（先平滑后作差） | 用本文档配套脚本，它是逐帧先作差再平滑的 |
| DLIO 中途跳变/丢跟踪 | 场地特征不足或飞行太快 | 降速；该架次作废重飞 |

---

## 附录 A 数据与结果目录约定

```
/logs/l3/<架次标签>/          真机上录的原始 rosbag（拷回分析机）
results/H1_<tag>.json         H1 结果
results/H2_<tag>.json         H2 结果
results/H3_<tag>.json + .png  H3 结果与收敛曲线
results/H4_<tag>.json         H4 结果
results/H5_<tag>.json         H5 结果
results/H6_<tag>.json         H6 结果
真机侧L3实验报告.md            l3_report.py 生成的最终报告
```

架次标签命名规则：`<实验号>_<条件>[_run<序号>]`，例如 `h3_run1`、`h2_base060`、
`h1_nlos`。分析时 `--tag` 用去掉实验号前缀的部分即可。

## 附录 B 分析脚本接口速查

| 脚本 | 作用 | 关键接口 |
|---|---|---|
| `l3_core.py` | 估计器算法的纯函数副本（分段/滑窗/闭式解/Huber），与机上 `origin_setter_node.py` 逐行对应 | `SlidingWindowEstimator(d_min, window, robust)`、`BatchEstimator`、`solve_theta`、`solve_theta_irls`、`residuals`、`wrap_pi` |
| `l3_analyze.py` | 六个子命令 `static/yaw/theta/sweep/robust/timing`，各自读 bag 出指标并落 JSON | 见第6节命令；统一用 bag 接收时间戳配对；`build_truth_series()` 实现"逐帧先作差再平滑"的真值 |
| `l3_report.py` | 扫描 `results/*.json` 生成报告 | `CRITERIA` 字典存判据阈值，`SIM_REF` 存仿真侧对照值 |
| `l3_record.sh` | 机上录制（第4节全文） | `l3_record.sh <标签> <时长> [ns] [--lock]` |

改任何一处算法时，`l3_core.py` 与机上 `origin_setter_node.py` 必须同步改——
论文"验证的就是部署的那个估计器"这句话依赖这一点。

## 附录 C 与仿真侧结果的对照（写报告时引用）

| 指标 | 仿真侧结果（论文第4章） | 真机侧填写 |
|---|---|---|
| 观测噪声 σ | 0.05 m（注入值） | H1 实测 |
| σ_yaw @ L=0.29 m | 13.9°（式(21)预测，未实测） | H2 实测 |
| θ\* 收敛后 RMSE | 0.74±0.13° / 0.67±0.20°（两机各5次） | H3 实测（N≥5） |
| 收敛所需段数 | 15.8±11.1 / 9.8±11.3 段 | H3 实测 |
| 批量 vs 滑窗（短架次） | 均 0.90°，无差别 | H4 实测 |
| 抗差改善（同一份数据） | RMSE 降约 3 倍 | H5 实测 |
| 单次更新耗时 | x86 i9：0.44/0.89 µs（不抗差）、0.67/40.88 µs（抗差） | H6 实测 |
| 机间链式复合误差 | 0.029±0.018 m | **本轮不做**（只有一台真机） |

## 附录 D 本轮不覆盖的项

1. **机间链式复合**（论文4.3.4的真机版）——只有一台真机，无法做。
2. **跨基线的标度律**（σ_yaw×L 是否随 L 恒定）——本轮拆装标签不可行，只做了
   机上原装的 0.29 m 单点，以及不换基线的块平均标度检验（5.2节）。
3. **机上闭环抗差**——真机镜像未含本轮改动，除非重建（第5.7节可选项）。
4. **SE(3) 推广**（论文2.8节）——仅有数学结构分析，两侧都未实现。

这几条在论文6.2节的局限里已经如实写明，报告里保持一致口径即可。
