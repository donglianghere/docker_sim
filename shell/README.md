# shell —— 容器起停入口

这一层只做一件事：**起停容器**。按操作对象分五类，和桌面五个文件夹一一对应。

```bash
cd ~/ai_uav/docker_sim/shell && ./start_gcs.sh
```

桌面 `GCS/ CONTEST/ SIM/ Docker/ SOUND/` 下全是指向本目录的软链（`enter_gcs.sh`
指向 `scripts/exec_gcs.sh`），双击即用，改这边桌面那边跟着变。

**本层唯一独有的东西是末尾的 `read -n 1` 暂停** —— 双击运行时窗口不会立刻关掉、
看得见报错。`scripts/` 下的脚本一个都没有，所以即使逻辑搬走了，这层壳也要留。

---

## 五类

### ① 地面站容器 → 桌面 `GCS/`

| | |
|---|---|
| `start_gcs.sh` | 转发 `scripts/up_gcs.sh`：xhost 授权、`gcs/.env` 校验（缺 `HOST_REPO_PATH` 直接报）、三个 json 从 `.example` 补齐、起完验 backend `/healthz` + 容器 running |
| `stop_gcs.sh` | 停 `gcs-gcs-1` + `gcs-backend-1` |
| `enter_gcs.sh`（桌面）| 软链到 `scripts/exec_gcs.sh`：进 `gcs-gcs-1` 的命令行 |

### ② 选手容器 → 桌面 `CONTEST/`

| | |
|---|---|
| `start_contestant_shell.sh` | 起常驻调试容器 `contestant-sdk-shell`，进去手敲 `ros2 topic` 用。`--real` 切 20 域，默认 21 |
| `stop_contestant_shell.sh` | 停它 |
| `enter_contestant.sh` | **进它的命令行。** 无参数=交互式 bash；带参数=跑一条就退出。进来先打出容器在哪个域 |
| `start_contestant_task.sh` | 一次性跑 `contestant_sim/我的任务.py`。⚠️ **那个文件现在不在仓库里**，所以这脚本当前会报"找不到任务文件" |

> 跑正式任务不走这里，走 `contestant_sim/run.sh` 或 `contestant_real/run_real.sh`。

### ③ 仿真容器 → 桌面 `SIM/`

| | |
|---|---|
| `start_sim.sh` | 转发 `scripts/start_sim.sh`：清选手容器 → `compose down`+`up` → 等两机 PX4 自检+节点就绪 → **验四路相机真的在出图** → 查 gzclient/rviz2 |
| `stop_sim.sh` | `compose down` |
| `build_sim.sh` | 构建仿真镜像，按序 sim-world → flight-stack-nx01 |
| `enter_sim.sh` | **进仿真容器的命令行。** `nx01`(默认) / `nx02` / `world` 三选一，带参数=跑一条就退出 |

> `scripts/start_sim.sh` 没有 `DISPLAY` 时**直接退出**（相机渲染必须有 X）。双击
> 必然有 DISPLAY；纯 SSH 下要起不带相机的仿真直接 `docker compose up -d`。
>
> 另有 `start.sh` + `scripts/up_and_watch.sh` 也能起仿真，那两个是 tmux 运维面板
> （`scripts/watch_sim.sh`）的入口，保留，别和这里混用。

### ④ 镜像与容器残留 → 桌面 `Docker/`

| | |
|---|---|
| `clean_docker_build_cache.sh` | 清 build cache |
| `clean_docker_containers.sh` | 清 Exited 容器，不动在跑的 |

> 同类还有 `scripts/bundle.sh`（导出/恢复已 build 的镜像）和
> `package_hw_deploy.sh`（真机部署打包），都在仓库根/`scripts/` 下，没搬过来。

### ⑤ 声光容器 → 桌面 `SOUND/`

| | |
|---|---|
| `start_sound_light_server.sh` | 起 `contestant-sound-light`（`contest_sdk.sound_light_server`）。串口默认 `/dev/ttyUSB0` |
| `stop_sound_light_server.sh` | 停它 |
| `enter_sound_light.sh` | **进它的命令行。** 看话题、看串口、手工发原始请求用 |
| `test_sound_light.sh` | **自检：让装置真的响一次、亮一次。** 无参数跑"蓝→红→绿"三条序列；`--sim`/`--real` 先断言域号再跑；`--list` 列全 20 个事件；`--event <名>` / `--sound 1..20` 放单个；`--mute` 熄灯静音；`--status` 只看域号+串口状态 |

> **仓库里没有别的东西能起它** —— `scripts/check_env.sh` 和
> `scripts/contestant_network.sh` 只是去查它的域号，不负责起。
> 注意 `brltty` 会抢 CH340，被抢了要先停它。

自检原理：常驻程序订阅 `/sound_light/request`（`std_msgs/String`），**纯文本
就行** —— 事件名 / 声音编号 1~20 / `mute`。`test_sound_light.sh` 从已在跑的
`contestant-sound-light` 容器里发，那上面已经有正确的 `ROS_DOMAIN_ID=20` 和
`CYCLONEDDS_URI`，不用另起容器也不会跟仿真的 21 域搞混。

两个时间约束（别和"没响"搞混）：**最小间隔 2.5 秒**（连发更密的会排队而不是
立刻播，所以序列里 sleep 3）、**过期 15 秒**（排队超时的请求被丢掉）。

没响的分法：看 `docker logs contestant-sound-light` 有没有 `发送：` 那一行
—— 有，问题在串口/装置那端（接线、电源、波特率）；没有，是请求没到常驻程序。

### 仿真 / 真机

`test_sound_light.sh` **自己不需要区分** —— 它 `docker exec` 进已在跑的容器里
发，用的就是那个容器的域，不可能发到另一个域去。

但声光常驻程序是**共享资源、同一时刻只能在一个域**（`--real`=20 真机 /
默认=21 仿真），于是有个假就绪的坑：**容器挂在 20 域时你测，装置照样响，可
仿真程序（21 域）的事件根本到不了它**。所以脚本每次都把域号打出来：

```
域 20 → **真机**场景（仿真程序发的事件到不了）
```

要上哪个场景，就用对应的断言跑一遍，不一致会拦住并给出换域的命令：

```
$ ./test_sound_light.sh --sim
!! 声光容器在域 20，但你要的是 sim 模式（应为 21）
   声光常驻程序只能在一个域，换域要重起它：
     ./stop_sound_light_server.sh
     ./start_sound_light_server.sh /dev/ttyUSB0
```

`scripts/check_env.sh sim|real` 也查这一项（`run.sh` / `run_real.sh` /
`run_test.sh` 起飞前都会调），两边用的是同一种取域号的方式。

---

## 进容器命令行

四类起容器的文件夹各有一个 `enter_*`（`Docker/` 不起容器，没有）：

| 桌面 | 脚本 | 目标容器 |
|---|---|---|
| `GCS/` | `enter_gcs.sh` → `scripts/exec_gcs.sh` | `gcs-gcs-1` |
| `CONTEST/` | `enter_contestant.sh` | `contestant-sdk-shell` |
| `SIM/` | `enter_sim.sh [nx01\|nx02\|world]` | 三个仿真容器 |
| `SOUND/` | `enter_sound_light.sh` | `contestant-sound-light` |

四个都是：**无参数 = 交互式 bash；带参数 = 跑一条就退出**；进去前打出容器在
哪个域；容器没跑就报错并给出起它的命令。

### 两类容器的环境差异（这是写这几个脚本的全部难点）

| | ROS_DOMAIN_ID / RMW / CYCLONEDDS_URI 从哪来 | `enter_*` 要补什么 |
|---|---|---|
| contestant 系（`contestant-sdk-shell`、`contestant-sound-light`） | `docker run -e` 的**容器级**变量（见 `scripts/contestant_network.sh` 的 `CONTESTANT_NET_ARGS`）+ 镜像 ENV | 只需 `source /opt/ros/humble/setup.bash`——镜像里 `ros2` 不在默认 PATH 上 |
| 仿真 flight-stack（`nx01`/`nx02`/`world`） | entrypoint 按 `PLANNER` **运行时 export**，容器级变量里只有 `PLANNER` 本身 | **必须额外 `source ros2_env_setup.sh`** 重新推导 |

第二行那个不补的后果是静默的：`PLANNER=ego_planner` 时你的 shell 还是默认
FastDDS，跟已切到 CycloneDDS 的真实节点对不上话题，`ros2 topic list` 是空的
——看着像连不上，实际只是 RMW 不一致。同一个坑 `scripts/exec_gcs.sh` 和
`scripts/ros2_env_setup.sh` 的文件头都记过。

`enter_sim.sh` 取 `ros2_env_setup.sh` 的方式分两路：`flight-stack-nx01/nx02`
把 `./scripts` 挂成了 `/opt/host_scripts`，直接 source；`sim-world` **没有**
这个挂载，先 `docker cp` 一份到 `/tmp`。

> `docker exec` 的 `-it`：只有交互式那条用 `-it`，"跑一条命令"那条**不加 `-t`**
> ——否则在非 TTY 环境（管道里、别的脚本里调）会报
> `cannot attach stdin to a TTY-enabled container`。第一版三个都加了 -t，实测踩到。

## 不属于这一层的

| | 在哪 |
|---|---|
| **完整程序运行** | `contestant_sim/{run,stop}.sh`、`contestant_real/{run_real,run_test,stop_real,vision_real}.sh` |
| **程序自动调用的** | `scripts/` 下 17 个：`contestant_network.sh`(被 8 处 source)、`check_env.sh`、`ros2_env_setup.sh`(9 处)、`launch_control.sh`、`record_rosbag.sh`/`prune_rosbag.sh`/`save_incident.sh`、`collect_container_stats.sh`/`render_tmux_status.sh`/`status_window.sh`/`watch_sim.sh`、`tail_persist_logs.sh`、`fetch_sources.sh`、`docker/entrypoints/`×4、`gcs/entrypoint.sh` 等。**这些路径被容器 COPY、两机 systemd、`src/**/launch`、`patches/` 钉死，不能移动改名** |

## 其它

`prepare_dataset.py`（YOLO 训练集整理）和 `仿真编译`（构建命令备忘，绕代理的
`env -u` 前缀）也在本目录，不属五类，放这儿是因为同属宿主机侧工具。

路径说明：转发用 `$(dirname "${BASH_SOURCE[0]}")/..` 定位；其余脚本仍写死
`/home/robots/ai_uav/docker_sim`，在 `.100`（`hx@`，家目录不同）上跑不了。
