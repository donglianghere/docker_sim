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

## 已有更好替代的

| 脚本 | 替代 |
|---|---|
| `start_sim.sh` / `stop_sim.sh` | `contestant_sim/run.sh` 自己会 `compose down/up`；只想起仿真不跑程序的话，`scripts/start_sim.sh`（153 行）比这个（48 行）更全，它还会**验证相机真的在出图**才返回 |
| `start_contestant_task.sh` | `contestant_sim/run.sh <程序名>`。这个脚本跑的是 `contestant_sim/我的任务.py`（单机、一次性） |
| `start_gcs.sh` | `scripts/up_gcs.sh`（88 行 vs 32 行，带 xhost 授权的说明和校验） |

> ⚠️ **分叉提醒**：`start_sim.sh` 和 `start_gcs.sh` 在 `scripts/` 下各有一个
> 同名/同功能但更完整的版本，两套并存。改了一边另一边不会知道。等确认哪套
> 是权威之后应该合并去重。

## 其它副本

`~/桌面/CONTEST/` 下有 `start_contestant_task.sh`、`start_contestant_shell.sh`、
`stop_contestant_shell.sh` 的**旧副本** —— 它们缺 `--real`（切 20/21 域）这一段，
也不 `source contestant_network.sh`。本目录这份才是新的。桌面那份应该换成
软链指向这里，终结分叉。

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
