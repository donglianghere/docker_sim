# shell —— 宿主机侧的启停与维护脚本

2026-10-03 从 `/home/robots/ai_uav/` 顶层移进来。移之前这些脚本**不在任何
版本控制里**（`ai_uav` 不是 git 仓库，仓库根只到 `docker_sim`），改坏了没法
回退，磁盘回退也没有保护 —— 这是移进来的唯一原因。

**它们都用绝对路径 `/home/robots/ai_uav/docker_sim`，所以放在哪都能跑**；
移动本身没有改任何一行逻辑。

```bash
cd ~/ai_uav/docker_sim/shell
./start_gcs.sh
```

---

## 还在用的

| 脚本 | 说明 |
|---|---|
| `start_sound_light_server.sh`<br>`stop_sound_light_server.sh` | 声光反馈常驻程序（`contest_sdk.sound_light_server`，容器 `contestant-sound-light`）。**仓库里没有别的东西能起它** —— `scripts/check_env.sh` 和 `scripts/contestant_network.sh` 只是去查它的域号。串口默认 `/dev/ttyUSB0`，注意 `brltty` 会抢 CH340 |
| `start_gcs.sh` / `stop_gcs.sh` | 地面站网页栈（`gcs-gcs-1` + `gcs-backend-1`），在 `docker_sim/gcs` 下跑 compose |
| `start_contestant_shell.sh`<br>`stop_contestant_shell.sh` | 常驻调试选手容器（`contestant-sdk-shell`），进去手敲 `ros2 topic` 用。`run.sh` **没有**这个功能，所以留着 |
| `build_sim.sh` | 构建仿真镜像，按序 sim-world → flight-stack-nx01 |
| `clean_docker_build_cache.sh` | 清 build cache |
| `clean_docker_containers.sh` | 清 Exited 容器，不动在跑的 |
| `prepare_dataset.py` | 整理 YOLO 训练集，`prepare_dataset.py <图片目录> -o <输出>` |
| `仿真编译` | 构建命令备忘（绕代理的那串 `env -u` 前缀），不是可执行脚本 |

## 转发到 scripts/ 的（2026-10-03 去重）

`start_sim.sh` 和 `start_gcs.sh` 原来各有一份独立实现，跟 `scripts/` 下
同功能的版本是两套分叉——改一边另一边不知道。现在改成**薄转发**：

| 脚本 | 转发到 | 为什么留着这一层 |
|---|---|---|
| `start_sim.sh` | `scripts/start_sim.sh` | 那份 153 行，会等两机就绪 + **验四路相机真的出图** + 查 gzclient/rviz2 |
| `start_gcs.sh` | `scripts/up_gcs.sh` | 那份 88 行，多做 xhost 授权、`.env` 校验、三个 json 从 `.example` 补齐、起完验 `/healthz` |

**这一层唯一独有的东西是末尾那个 `read -n 1` 暂停** ——
双击运行时窗口不会立刻关掉，看得见报错。`scripts/` 下 12 个脚本一个都没有。
所以不能简单删掉 `shell/` 这两个，删了双击就看不到输出。

转发用 `$(dirname "${BASH_SOURCE[0]}")/..` 定位，不写死家目录，所以这两个在
`.100`（`hx@`，家目录不是 `/home/robots`）上也能跑。

> `scripts/start_sim.sh` 没有 DISPLAY 时**直接退出**（相机渲染必须有 X），
> 原来 `shell/` 那份只是警告后继续。双击必然有 DISPLAY，不影响；纯 SSH 下
> 要起不带相机的仿真直接 `docker compose up -d`。

## 功能上已被 run.sh 取代，但仍留着的

| 脚本 | 说明 |
|---|---|
| `stop_sim.sh` | `contestant_sim/run.sh` 结束时自己会收尾；单独停仿真时用这个 |
| `start_contestant_task.sh` | `contestant_sim/run.sh <程序名>` 是正路。这个跑的是 `contestant_sim/我的任务.py`（单机、一次性）——⚠️ **那个文件现在不在仓库里**，在 `~/桌面/CONTEST/` 下，所以这个脚本当前会报"找不到任务文件" |

## 其它副本

`~/桌面/CONTEST/` 下那三个（`start_contestant_task.sh`、
`start_contestant_shell.sh`、`stop_contestant_shell.sh`）**已于 2026-10-03
改成软链指向本目录**，分叉结束。原来桌面那份旧 12~13 行（缺 `--real` 切
20/21 域、不 `source contestant_network.sh`）。

桌面还剩 `我的任务.py` 和 `运行仿真.sh` 两个实体文件：前者是
`start_contestant_task.sh` 要找的任务文件（放错地方了，见上）；后者是
10-01 归档掉的旧入口，本目录没有对应物。

## 没移进来的

`camera_info.yaml`、`声光反馈程序接口.xlsx`（数据/文档）、`datasets/`、
`docker_real/`、`tools/` 三个目录仍在 `~/ai_uav/` 顶层，同样没有版本控制。

## 遗留问题

路径是硬编码的 `/home/robots/ai_uav/docker_sim`。第二台地面站是 `hx@192.168.2.100`
（家目录不是 `/home/robots`），这些脚本在那台上**跑不了**。要在两台通用，
应该改成相对本脚本定位：

```bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
```
