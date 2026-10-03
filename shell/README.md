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

### ② 选手容器 → 桌面 `CONTEST/`

| | |
|---|---|
| `start_contestant_shell.sh` | 起常驻调试容器 `contestant-sdk-shell`，进去手敲 `ros2 topic` 用。`--real` 切 20 域，默认 21 |
| `stop_contestant_shell.sh` | 停它 |
| `start_contestant_task.sh` | 一次性跑 `contestant_sim/我的任务.py`。⚠️ **那个文件现在不在仓库里**，所以这脚本当前会报"找不到任务文件" |

> 跑正式任务不走这里，走 `contestant_sim/run.sh` 或 `contestant_real/run_real.sh`。

### ③ 仿真容器 → 桌面 `SIM/`

| | |
|---|---|
| `start_sim.sh` | 转发 `scripts/start_sim.sh`：清选手容器 → `compose down`+`up` → 等两机 PX4 自检+节点就绪 → **验四路相机真的在出图** → 查 gzclient/rviz2 |
| `stop_sim.sh` | `compose down` |
| `build_sim.sh` | 构建仿真镜像，按序 sim-world → flight-stack-nx01 |

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
| `test_sound_light.sh` | **自检：让装置真的响一次、亮一次。** 无参数跑"蓝→红→绿"三条序列；`--list` 列全 20 个事件；`--event <名>` / `--sound 1..20` 放单个；`--mute` 熄灯静音；`--status` 只看串口状态 |

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

---

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
