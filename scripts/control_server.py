#!/usr/bin/env python3
"""地面站远程控制端点：管理flight-stack-hw/vision-stack两个容器的生命周期、
vision-stack内部检测节点(YOLO/QR/AprilTag)的启停切换、命名空间改名，外加
一个UDP广播心跳，供地面站在不知道IP的情况下发现局域网里在线的机器。

必须在host上跑（不是容器内）——要执行docker compose命令操作两个容器
本身，容器内做这件事得挂载docker.sock，等于给一个已经privileged的
vision-stack容器再叠一层host级控制权，没必要地叠加风险；host上一个干净
的python进程反而更简单可控、职责边界也更清楚。

⚠️ 安全模型（这次先做到这个程度，够纯内网测试场景用，不是生产级方案）：
- 2026-08-25：X-Control-Token鉴权应用户明确要求整个删掉了（地面站那边
  也同步删了），控制端点现在对局域网内任何能连上8890端口的客户端完全
  开放——纯内网测试场景下这是用户主动接受的权衡（换取不用再折腾令牌
  反复配错的问题），不是默认推荐的做法，接到更大的网络/正式场景前
  必须重新加认证，不能原样照搬。
- 停/重启flight-stack这类会直接掐断飞控软件的操作，请求体必须显式带
  `"confirm": true`，没带就拒绝执行——防止误触发/脚本bug空手调用。
- 所有写操作都记审计日志（谁在什么时候调了什么、confirm带没带、结果
  如何），见AUDIT_LOG_PATH，方便事后查"到底是谁把飞控停了"。

命名空间来源：跟docker-compose.hw.yml的NAMESPACE环境变量是同一个概念，
但这里改成从本地文件读，不是shell环境变量——开机时不需要人在场手动
export，脚本自己读文件就行：
- NAMESPACE_FILE（默认docker_sim/.namespace）：纯文本一行，机身固定
  命名空间，开机时兜底用这个自启，不依赖地面站在场（地面站故障/断网时
  飞机仍然用这个命名空间正常工作）。
- 地面站发现在线机器之后可以调POST /rename改这个文件+重启flight-stack/
  vision-stack两个容器，切换到地面站分配的命名空间。

================================ 接口契约 ================================
地面站只读本文件即可实现客户端，不需要看 handler 实现。
运行时也可以 `GET /api` 拿到同一份契约（合法取值从 STACK_CHOICES/
HW_FORBIDDEN 常量生成，与服务端校验同源，不会出现文档与实现漂移）。

端口：HTTP 控制 8890（CONTROL_PORT），UDP 广播 8891（BEACON_PORT）
跨域：所有响应带 Access-Control-Allow-Origin: *，网页端可直接 fetch

--- 0. 发现机器（UDP 广播，免轮询） -----------------------------------
每 2 秒（BEACON_INTERVAL_SEC）广播一个 JSON 到 8891：
  {"namespace":"NX01", "ip":"192.168.2.104", "control_port":8890,
   "hostname":"uav", "flight_up":true, "vision_up":false,
   "stack":{"LOCALIZATION_SOURCE":"single_uwb_slam","SLAM_BACKEND":"dlio",
            "PLANNER":"ego_planner","CONTROLLER":"px4ctrl"},
   "flight_started_at":"2026-09-04T05:39:24.982725353Z", "cpu_temp_c":81.25,
   "cpu_usage_pct":23.4}
地面站监听这个端口就能同时拿到"有哪些机器在线"和"每台在跑什么模式"，
不用发任何请求。广播只带运行值，不带待生效值（保持包小）。

2026-09-04新增 flight_started_at / cpu_temp_c / cpu_usage_pct 三个字段
（地面站要监控"飞行栈启动时长"/"机载计算机温度"/"机载计算机CPU占用率"）：
- flight_started_at：flight-stack容器的真实启动时间(`docker inspect`的
  State.StartedAt，ISO8601 UTC字符串)，flight_up=false时为null。地面站
  用它减去当前时间就是准确的启动时长，不需要自己在浏览器端近似估算。
- cpu_temp_c：Jetson `tj-thermal`热区温度（摄氏度，四舍五入到1位小数）
  ——这是Jetson自己做温控节流判断用的综合温度，读不到时为null。
- cpu_usage_pct：机载计算机(Jetson)整体CPU占用率(%，1位小数)，读
  /proc/stat两次心跳间的差值算出来的，进程刚启动的头2秒内(还没有上一次
  采样可比)是null，见sample_cpu_usage_pct()定义处的说明。2026-09-04同日
  新增——原计划用mavros/imu/temperature_imu做"飞控温度"，但SSH实测这个
  话题连续6秒的读数一模一样，不像真实传感器数据，用户要求换成这一项。

⚠️ 2026-09-01改：飞机可能同时挂在多个网段上（这台机器是有线接雷达
192.168.1.0/24 + WiFi接地面站 192.168.2.0/24）。心跳现在**逐网段各发
一份**，每份分别发到该网段的定向广播地址和 255.255.255.255 两个目的地
（有些AP只放行其中一种）。所以：
- 地面站在哪个网段，就会从哪个网段收到心跳，不需要跟飞机在"主"网段上；
- 每份心跳里的 "ip" 是**发出这份心跳的那块网卡自己的地址**，即"从你这个
  网段能连通的地址"，地面站直接拿它拼 http://{ip}:{control_port} 即可，
  不要跨网段复用别处收到的 ip；
- 同一台飞机因此可能被看到多份心跳（每个网段一份），namespace/hostname
  相同而 ip 不同，属于正常现象——地面站按 namespace 去重即可。

--- 1. GET /status ----------------------------------------------------
  {"namespace":"NX01","flight_up":true,"vision_up":false,
   "hostname":"uav","ip":"192.168.1.5",
   "stack":         {四个模式的**当前运行值**，读自 docker inspect},
   "stack_pending": {四个模式的**待生效值**，读自 docker compose config+.env},
   "stack_mismatch":["CONTROLLER"],
   "flight_started_at":"2026-09-04T05:39:24.982725353Z", "cpu_temp_c":81.25,
   "cpu_usage_pct":23.4}
   （flight_started_at/cpu_temp_c/cpu_usage_pct 含义同上面 UDP 广播那三个
   同名字段）
⚠️ stack 与 stack_pending 的区别很重要：容器的环境变量是**创建时固化**的，
改完 .env 若没 recreate，两者就不一致。stack_mismatch 非空 = "配置已改但
未生效"，地面站应当据此提示用户重启。

--- 2. GET /api -------------------------------------------------------
返回本契约的机器可读版本，含 stack_choices / hw_forbidden 两张表，
地面站可据此动态生成下拉框，避免硬编码取值。

--- 3. POST /stack/mode —— 设定启动环境（不碰容器） --------------------
请求（四项均可省略，省略的沿用现值，支持"只换控制器"）：
  {"localization":"single_uwb_slam", "slam":"dlio",
   "planner":"ego_planner", "controller":"px4ctrl"}
成功（HTTP 200）：
  {"ok":true, "applied":{...}, "pending":{...}, "running":{...},
   "mismatch":["CONTROLLER"], "restart_required":true, "note":"..."}
失败（HTTP 400）：
  {"ok":false, "message":"真机模式下不能用 LOCALIZATION_SOURCE=gt —— ..."}
本端点**只写 .env、不动容器**，所以不需要 confirm。要让新模式生效，
再单独调 POST /stack {"stack":"flight","action":"up"}。
校验在写入之前完成，与 entrypoint 的 fail-fast 规则一一对应——非法组合
在这里就被拒，不会出现"容器起来 9 秒后自己退出"。

  合法取值（权威定义见 STACK_CHOICES 常量）：
    localization : gt / uwb_slam / uwb_imu /
                   single_slam_only / single_uwb_imu / single_uwb_slam
    slam         : dlio / point_lio / fast_lio
    planner      : mighty / ego_planner
    controller   : px4ctrl / so3ctrl / pt4ctrl
  真机禁用（见 HW_FORBIDDEN 常量）：
    localization=gt        真机没有 Gazebo 真值，entrypoint 会 exit 1
  另外 single_* 前缀是单机专用，要求 NUM_AGENTS=1。
  2026-09-08：controller 去掉了 ros2_px4_stack 这个选项（不再作为地面站
  可选项暴露，entrypoint.sh 本身仍然认识这个值，只是这个接口不再放行）；
  so3ctrl 的 hw 版 launch 文件（so3ctrl_hw.launch.py）已经在 2026-09-06
  补齐，从 HW_FORBIDDEN 里移除；slam 新增 fast_lio（2026-09-08 接通
  DEPLOY_TARGET=hw，仅完成接口接入，尚未真机验证，见 DEBUG_JOURNAL.md）。

--- 4. POST /stack —— 启停容器 ----------------------------------------
  {"stack":"flight"|"vision", "action":"up"|"down"|"restart",
   "confirm":true}
  · flight 的 down/restart 必须带 confirm:true（会掐断飞控软件）
  · action=up 与 restart 对 flight 都走 `up -d --force-recreate`，
    这样才能读到新的 .env（`docker compose restart` 沿用旧环境变量，
    改了配置却不生效且不报错——本项目实测踩过这个坑）
  · vision 的 restart 仍走 `docker compose restart`：vision-stack 没有
    volume 挂载，force-recreate 会连容器内的 colcon 编译产物一起销毁，
    后续重编会超过 120 秒超时

--- 5. POST /vision/mode ---------------------------------------------
  {"cam":"cam0"|"cam1"|"all", "mode":"yolo"|"qr"|"apriltag"|"raw"|"stop"}
  · raw = 纯视频直通（不推理，只把相机原始帧推给MJPEG）2026-09-23新增
  · mode=yolo 时可加 {"mjpeg_draw_detections": false}：检测照常发布，
    但地面拉到的是不带检测框的原始画面

--- 6. POST /rename ---------------------------------------------------
  {"namespace":"NX02"}   改 .namespace 文件并重启两个容器

--- 7. GET /params / POST /params —— 参数编辑器（2026-09-25新增） -------
地面站"④配置面板·参数编辑器"用（地面站 backend 的 /api/hw-fleet/{ns}/params
只做透传，校验/备份/回滚全在这里）。参数全集 = docker-compose.hw.yml 里
所有 ${KEY...} 引用的变量，再加上 .env 里写了但 compose 没引用的键。
GET 返回：
  {"flight_up":true, "compose_config_error":null|"...",
   "params":[{"key":"V_MAX", "default":"1.0",   ← compose 里 ${KEY:-默认值}
              "effective":"0.8",               ← 下次创建容器会用的值(.env 优先)
              "source":"env"|"default",        ← effective 来自 .env 还是基线
              "running":"1.0"|null,             ← 容器此刻固化的值，容器不存在为 null
              "stale":true,                     ← running 与 effective 不一致=改了没重启
              "choices":[...]|null,             ← 四个模式项给下拉框取值
              "comment":"...", "known":true}]}  ← known=false: compose 没引用
POST 请求：
  {"changes":{"V_MAX":"0.8"}, "remove":["EGO_DIST0"], "note":"v-max",
   "restart":false, "allow_unknown":false}
  · changes 只写这几行进 .env，已有的行就地改值、行尾注释保留；没有的追加
  · remove 删掉 .env 里的这几行，回到 compose 基线默认值
  · compose 没引用的键默认拒绝（多半是拼错了），确认要写就带 allow_unknown
  · NAMESPACE 不能在这里改，走 POST /rename
  · 写前备份成 .env.bak-<时间>-params-<note>，写后跑 `docker compose config`
    校验，不通过自动回滚
  · restart=true 时再 `up -d --force-recreate flight-stack-hw`，并回读容器
    环境变量核对，verify_mismatch 列出没对上的键

--- 响应格式约定 ------------------------------------------------------
成功 HTTP 200 / 失败 HTTP 400，body 一律含 "ok" 布尔字段。
handler 返回结构化数据时字段**直接平铺**在顶层（不套 message 字符串），
返回纯文本时放在 "message"。所以客户端判断逻辑是：
  if (!resp.ok) 显示 resp.message; else 用顶层的结构化字段。

--- 典型流程 ----------------------------------------------------------
  1) 监听 UDP 8891 发现机器，拿到 ip / namespace / 当前 stack
  2) GET /api 拉取合法取值，生成下拉框
  3) POST /stack/mode 设定模式 → 看 restart_required
  4) 若需重启：POST /stack {"stack":"flight","action":"up"}
  5) 轮询 GET /status，等 stack_mismatch 变空 = 新模式已生效
========================================================================

用法：
  python3 scripts/control_server.py
"""
import fcntl
import glob
import http.server
import json
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMPOSE_FILE = os.path.join(REPO_ROOT, "docker-compose.hw.yml")
ENV_FILE = os.path.join(REPO_ROOT, ".env")
NAMESPACE_FILE = os.path.join(REPO_ROOT, ".namespace")
AUDIT_LOG_PATH = os.path.join(REPO_ROOT, "runtime_logs", "control_server_audit.log")

CONTROL_PORT = int(os.environ.get("CONTROL_PORT", "8890"))
BEACON_PORT = int(os.environ.get("BEACON_PORT", "8891"))
BEACON_INTERVAL_SEC = 2.0

FLIGHT_SERVICE = "flight-stack-hw"
VISION_SERVICE = "vision-stack"
FLIGHT_CONTAINER = "docker_sim-flight-stack-hw-1"
VISION_CONTAINER = "docker_sim-vision-stack-1"

# vision-stack检测节点PID匹配模式——跟vision_stack_down.sh基本一致（精确
# 查PID再kill，不用pkill -f：那条命令自己的文本会反向匹配自杀，见
# read_hw.md 2026-08-22"pkill -f自杀"那次教训）。
# 2026-08-25两路相机接上后加了按sensor_id过滤：docker exec -d里
# `exec ros2 run/launch ...`会把bash进程替换成ros2进程本身（exec的作用），
# 所以`ps aux`里能看到的是`-p sensor_id:=N`/`sensor_id:=N`这些真实传给
# 节点的CLI参数（不是本来写的bash -c脚本文本，那部分已经被exec替换掉了）
# ——用这个来区分同时在跑的cam0/cam1两个检测进程，不传sensor_id时不过滤，
# 匹配所有相机的检测进程(kill_vision_detector_processes()全杀那个场景用)。
_VISION_NODE_PATTERN = (
    "yolo_detector_node|qr_detect_node|apriltag_node --|"
    "image_publisher_node --|video_stream_node|ros2 launch vision_stack"
)


def _vision_node_grep_cmd(sensor_id=None):
    cmd = f"ps aux | grep -E '{_VISION_NODE_PATTERN}' | grep -v grep | grep -v defunct"
    if sensor_id is not None:
        cmd += f" | grep -E 'sensor_id:=[\"]?{sensor_id}\\b'"
    return cmd + " | awk '{print $2}' | tr '\\n' ' '"


def read_namespace():
    if os.path.exists(NAMESPACE_FILE):
        with open(NAMESPACE_FILE) as f:
            ns = f.read().strip()
            if ns:
                return ns
    return "NX01"


def write_namespace(ns):
    with open(NAMESPACE_FILE, "w") as f:
        f.write(ns + "\n")


def audit_log(remote_addr, method, path, body, result):
    os.makedirs(os.path.dirname(AUDIT_LOG_PATH), exist_ok=True)
    entry = {
        "ts": datetime.now(timezone.utc).isoformat(),
        "remote_addr": remote_addr,
        "method": method,
        "path": path,
        "body": body,
        "result": result,
    }
    with open(AUDIT_LOG_PATH, "a") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def run_compose(args, extra_env=None):
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    cmd = ["docker", "compose", "-f", COMPOSE_FILE] + args
    result = subprocess.run(cmd, cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=120)
    return result.returncode == 0, (result.stdout + result.stderr)


def container_running(name):
    result = subprocess.run(
        ["docker", "inspect", "-f", "{{.State.Running}}", name],
        capture_output=True, text=True,
    )
    return result.returncode == 0 and result.stdout.strip() == "true"


# 2026-09-04新增：地面站要监控"飞行栈启动时长"，SSH实测确认
# `docker inspect`的State.StartedAt就是准确的容器启动时间，不需要地面站
# 自己在浏览器端近似估算（之前那版本刷新网页就会归零，见GCS前端
# DEBUG_JOURNAL同日记录）。只在flight_up为true时才有意义调用——见
# beacon_loop()/do_GET里的调用处，容器没在跑时不必浪费一次docker inspect。
def flight_started_at():
    """返回flight-stack容器的真实启动时间(ISO8601 UTC字符串)，读不到（容器
    没在跑/命令失败）返回None。"0001-01-01T00:00:00Z"是docker对"从未启动过"
    的零值时间戳，同样按None处理。"""
    result = subprocess.run(
        ["docker", "inspect", "-f", "{{.State.StartedAt}}", FLIGHT_CONTAINER],
        capture_output=True, text=True, timeout=5,
    )
    if result.returncode != 0:
        return None
    ts = result.stdout.strip()
    return ts if ts and not ts.startswith("0001-01-01") else None


# 2026-09-04新增：地面站要监控"机载计算机温度"。SSH实测这台Jetson的
# /sys/class/thermal/下有cpu/gpu/cv0-2/soc0-2/tj共9个热区，单位毫摄氏度。
# 取tj-thermal（thermal junction，Jetson自己做温控节流判断用的那个综合
# 温度，不是某一个子模块的局部温度）当"机载计算机温度"代表值，实测跟
# cpu-thermal数值很接近但语义上更贴合"这台机器整体热不热"。这个脚本本来
# 就"必须在host上跑，不是容器内"（见文件头说明），能直接读宿主机的
# thermal_zone文件，不需要docker exec。
def read_cpu_temp_c():
    """返回tj-thermal热区温度(摄氏度，1位小数)，读不到时返回None。"""
    for zone_dir in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(os.path.join(zone_dir, "type")) as f:
                if f.read().strip() != "tj-thermal":
                    continue
            with open(os.path.join(zone_dir, "temp")) as f:
                milli_c = int(f.read().strip())
            return round(milli_c / 1000.0, 1)
        except (OSError, ValueError):
            continue
    return None


# 2026-09-04新增：原计划用mavros/imu/temperature_imu做"飞控温度"，SSH实测
# 发现这个话题连续6秒/70多条消息数值一模一样(15.0)，没有任何真实传感器
# 该有的量化噪声——大概率是这颗IMU驱动/固件没有真的在报板温、mavros收到
# 的是个写死的占位值，不可信。用户明确要求换成"机载计算机CPU占用率"。
#
# 实现思路：读/proc/stat算CPU占用率，标准做法是"采样两次、中间sleep一段
# 时间、用差值算"——但这里不能在请求/心跳处理里现场sleep（会阻塞
# beacon_loop()本来就吃紧的2秒周期，或者拖慢/status的响应），改成"跨两次
# beacon_loop()调用取差值"：每2秒的心跳循环本身就是一个天然的采样间隔，
# 上一轮的/proc/stat快照存在模块级变量里，这一轮直接跟它比，不需要额外
# sleep。GET /status复用同一份缓存(_cpu_usage_pct_cache)而不是自己再采样
# 一次——原因同样是避免/status这种同步HTTP请求里塞一次sleep拖慢响应；
# 代价是/status返回的是"最近一次心跳周期"的值，不是"调用/status这一刻"的
# 瞬时值，但CPU占用率本来就是个统计量，2秒粒度的滞后可以接受。
_last_proc_stat_sample = None  # (total_jiffies, idle_jiffies)，上一次采样
_cpu_usage_pct_cache = None    # 最近一次算出的CPU占用率(%)，第一次心跳前是None


def _read_proc_stat_total_idle():
    """读/proc/stat第一行(全核汇总的"cpu"行)，返回(总jiffies, idle+iowait
    jiffies)。字段顺序固定为user/nice/system/idle/iowait/irq/softirq/
    steal/guest/guest_nice，跟内核版本无关，是/proc/stat这些年一直保持的
    ABI稳定性保证。"""
    with open("/proc/stat") as f:
        parts = f.readline().split()
    values = [int(v) for v in parts[1:]]
    idle = values[3] + values[4]  # idle + iowait都算"没在干活"
    return sum(values), idle


def sample_cpu_usage_pct():
    """beacon_loop()每轮(2秒)调用一次，更新_cpu_usage_pct_cache。跟上面
    read_cpu_temp_c()不同，这个函数不返回值给调用方直接用——它是"边采样
    边更新缓存"的副作用函数，真正取值走_cpu_usage_pct_cache这个模块级
    变量（GET /status和beacon都读它）。读/proc/stat失败(权限/文件不存在，
    理论上不该发生但防御一下)时保留上一次的缓存值不变，不主动清空成None
    ——一次读取失败不代表机器状态突然不可知了，沿用上一个数字比突然跳成
    "--"更合理。"""
    global _last_proc_stat_sample, _cpu_usage_pct_cache
    try:
        total, idle = _read_proc_stat_total_idle()
    except (OSError, ValueError, IndexError):
        return
    if _last_proc_stat_sample is not None:
        prev_total, prev_idle = _last_proc_stat_sample
        dtotal = total - prev_total
        didle = idle - prev_idle
        if dtotal > 0:
            _cpu_usage_pct_cache = round(100.0 * (dtotal - didle) / dtotal, 1)
    _last_proc_stat_sample = (total, idle)


# 地面站要显示的四个"当前运行模式"。2026-08-29新增。
#
# ⚠️ 为什么从**运行中的容器**读而不是从.env/compose读：
# 容器的环境变量是创建时固化的，改完.env之后如果没有recreate容器，
# .env和实际运行的值就会不一致——本项目实测踩过这个坑（compose默认值被改成
# CONTROLLER=pt4ctrl之后容器起不来；容器跑着旧镜像而源码已经改了）。
# 地面站要显示的是"现在实际在跑什么"，所以以 docker inspect 为准。
# 同时把.env/compose解析出来的"待生效值"也返回，两者不一致时地面站可以提示
# "配置已改但未重启生效"——这正是本项目反复出现的"静默失效"类问题之一。
# 四个模式的合法取值。**必须跟 flight-stack-entrypoint.sh 里的校验保持一致**，
# 否则地面站放行了、容器起来 9 秒后 fail fast 退出（本项目实测踩过：
# CONTROLLER 被改成 pt4ctrl 而当时没有对应的 hw launch，容器起来就退）。
STACK_CHOICES = {
    "LOCALIZATION_SOURCE": ("gt", "uwb_slam", "uwb_imu",
                            "single_slam_only", "single_uwb_imu", "single_uwb_slam"),
    "SLAM_BACKEND":        ("dlio", "point_lio", "fast_lio"),
    "PLANNER":             ("mighty", "ego_planner"),
    "CONTROLLER":          ("px4ctrl", "so3ctrl", "pt4ctrl"),
}

# DEPLOY_TARGET=hw 下明确不可用的取值，以及原因。对应 entrypoint 里两处 fail fast。
HW_FORBIDDEN = {
    "LOCALIZATION_SOURCE": {
        "gt": "真机没有Gazebo真值，gt_odom_bridge_node会永远收不到数据（entrypoint会exit 1）",
    },
    # 2026-09-08：CONTROLLER条目（so3ctrl）已删除——so3ctrl_hw.launch.py
    # 2026-08-29已经补齐，entrypoint.sh对应的fail fast也已撤掉，这条
    # 禁用规则已经不成立，见DEBUG_JOURNAL.md 2026-09-06记录。
}


def validate_stack_choice(req, deploy_target="hw", num_agents=1):
    """校验一组模式取值。返回 (ok, 错误信息)。req 是 {KEY: value} 的字典。"""
    for k, v in req.items():
        if k not in STACK_CHOICES:
            return False, f"未知的模式项 {k}"
        if v not in STACK_CHOICES[k]:
            return False, f"{k}={v} 不是合法取值，可选：{'/'.join(STACK_CHOICES[k])}"
        if deploy_target == "hw" and v in HW_FORBIDDEN.get(k, {}):
            return False, f"真机模式下不能用 {k}={v} —— {HW_FORBIDDEN[k][v]}"

    loc = req.get("LOCALIZATION_SOURCE")
    if loc and loc.startswith("single_") and int(num_agents) != 1:
        return False, f"{loc} 是单机专用模式，但 NUM_AGENTS={num_agents}（应为1）"
    return True, ""


# 2026-09-25新增：POST /stack/mode 和 POST /params 都会读改写 .env，
# ThreadingHTTPServer 下两个请求同时到会互相覆盖对方的改动，统一用这把锁。
_ENV_LOCK = threading.Lock()


def update_env_file(kv):
    with _ENV_LOCK:
        _update_env_file_locked(kv)


def _update_env_file_locked(kv):
    """把 kv 里的键值写进 .env（幂等：已存在就就地替换，不存在就追加）。

    为什么写 .env 而不是只用 extra_env 传给这一次 up：
    extra_env 只对当次 docker compose 调用有效，之后任何人手动 `docker compose
    up -d` 或 control_server 自己再拉一次，都会退回 .env 的值——模式就悄悄变回去了。
    写进 .env 才能让"当前模式"有唯一、持久的真相来源。
    """
    env_path = ENV_FILE
    try:
        lines = open(env_path, encoding="utf-8").read().splitlines()
    except FileNotFoundError:
        lines = []
    remaining = dict(kv)
    out = []
    for line in lines:
        stripped = line.lstrip()
        replaced = False
        if stripped and not stripped.startswith("#") and "=" in stripped:
            key = stripped.split("=", 1)[0].strip()
            if key in remaining:
                out.append(f"{key}={remaining.pop(key)}")
                replaced = True
        if not replaced:
            out.append(line)
    if remaining:
        out.append("")
        out.append(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} 由地面站 POST /stack/mode 写入")
        for k, v in remaining.items():
            out.append(f"{k}={v}")
    open(env_path, "w", encoding="utf-8").write("\n".join(out) + "\n")


STACK_KEYS = ("LOCALIZATION_SOURCE", "SLAM_BACKEND", "PLANNER", "CONTROLLER")


def read_stack_config():
    """返回 (running, pending, mismatch)。

    running : 运行中容器实际生效的四个值；容器没跑时为 {}
    pending : compose+.env 解析出来的值（下次 up -d 会生效的）
    mismatch: 两者不一致的 key 列表，空列表表示一致
    """
    running = {}
    try:
        r = subprocess.run(
            ["docker", "inspect", "-f", "{{range .Config.Env}}{{println .}}{{end}}",
             FLIGHT_CONTAINER],
            capture_output=True, text=True, timeout=5,
        )
        if r.returncode == 0:
            for line in r.stdout.splitlines():
                k, _, v = line.partition("=")
                if k in STACK_KEYS:
                    running[k] = v
    except Exception:
        pass

    pending = {}
    try:
        r = subprocess.run(
            ["docker", "compose", "-f", COMPOSE_FILE, "config", "--format", "json"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=15,
        )
        if r.returncode == 0:
            env = json.loads(r.stdout)["services"][FLIGHT_SERVICE]["environment"]
            for k in STACK_KEYS:
                if k in env:
                    pending[k] = str(env[k])
    except Exception:
        pass

    mismatch = [k for k in STACK_KEYS
                if k in running and k in pending and running[k] != pending[k]]
    return running, pending, mismatch


# ---- 参数编辑器（2026-09-25新增，GET/POST /params，契约见文件头第7节）----
# 地面站 2026-09-24 先上线了参数编辑器的前端和 backend 透传，飞机端这一半
# 一直没写，GET /params 返回 404，地面站就显示"读取失败: not found"。

# NAMESPACE 由 .namespace + POST /rename 管理，run_compose 每次都用
# extra_env 注入，写进 .env 也不会生效，反而制造两个真相来源。
PARAMS_READONLY = {"NAMESPACE": "命名空间由 .namespace 文件管理，改名请用 POST /rename"}

_COMPOSE_VAR_RE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)(?:(:?[-?])([^}]*))?\}")
_ENV_KEY_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def parse_compose_vars():
    """扫 docker-compose.hw.yml 里所有 ${KEY...} 引用，返回有序字典
    {KEY: {"default": 值或None, "comment": 该行行尾注释}}。

    不用 `docker compose config` 拿：它输出的是已经代入 .env 之后的结果，
    分不出哪个是基线默认值、哪个是本机覆盖——而"基线默认 / 本机覆盖"
    这一列正是参数编辑器要显示的东西。整行注释（# 开头）里举例用的
    ${VAR:-default} 不算。同一个键出现多次时取第一次出现的默认值。
    """
    out = {}
    with open(COMPOSE_FILE, encoding="utf-8") as f:
        for line in f:
            if line.lstrip().startswith("#"):
                continue
            last_brace = line.rfind("}")
            hash_pos = line.find("#", last_brace + 1)
            comment = ""
            code = line
            if hash_pos > 0 and line[hash_pos - 1].isspace():
                comment = line[hash_pos + 1:].strip()
                code = line[:hash_pos]
            for m in _COMPOSE_VAR_RE.finditer(code):
                key, op, arg = m.group(1), m.group(2), m.group(3)
                if key in out:
                    continue
                default = arg if op in (":-", "-") else None
                out[key] = {"default": default, "comment": comment}
    return out


def _parse_env_value(rest):
    """解析 .env 一行等号右边的部分，返回 (值, 值后面原样保留的尾巴)。
    规则跟 docker compose 一致：带引号的取引号内；不带引号的，空白后面的
    # 起算注释。尾巴（空白+注释）改值时原样拼回去，行尾注释不丢。"""
    if rest[:1] in ("'", '"'):
        q = rest[0]
        end = rest.find(q, 1)
        if end > 0:
            return rest[1:end], rest[end + 1:]
    m = re.search(r"\s+#", rest)
    if m:
        return rest[:m.start()].strip(), rest[m.start():]
    return rest.strip(), ""


def read_env_entries():
    """返回 (lines, entries)。lines 是 .env 原始行；entries 是
    {KEY: {"value": 值, "comment": 行尾注释, "idx": [行号...]}}，
    重复出现的键 value 取最后一次（compose 的行为），idx 记全部行号，
    改值/删除时所有重复行一起处理。"""
    try:
        with open(ENV_FILE, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except FileNotFoundError:
        lines = []
    entries = {}
    for i, line in enumerate(lines):
        s = line.strip()
        if not s or s.startswith("#") or "=" not in s:
            continue
        key, rest = line.split("=", 1)
        key = key.strip()
        if not _ENV_KEY_RE.match(key):
            continue
        value, tail = _parse_env_value(rest)
        e = entries.setdefault(key, {"idx": []})
        e["value"] = value
        e["comment"] = tail.strip().lstrip("#").strip()
        e["idx"].append(i)
    return lines, entries


def read_container_env(container):
    """容器此刻固化的环境变量 {KEY: 值}；容器不存在返回 None。"""
    try:
        r = subprocess.run(
            ["docker", "inspect", "-f", "{{range .Config.Env}}{{println .}}{{end}}", container],
            capture_output=True, text=True, timeout=5,
        )
    except Exception:
        return None
    if r.returncode != 0:
        return None
    env = {}
    for line in r.stdout.splitlines():
        k, sep, v = line.partition("=")
        if sep:
            env[k] = v
    return env


def compose_config_check():
    """跑一次 `docker compose config -q`，返回 None=通过，否则返回错误文本。"""
    try:
        r = subprocess.run(
            ["docker", "compose", "-f", COMPOSE_FILE, "config", "-q"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=30,
        )
    except Exception as e:
        return repr(e)
    if r.returncode == 0:
        return None
    return (r.stdout + r.stderr).strip()[-1000:] or f"exit {r.returncode}"


def collect_params():
    """GET /params 的返回体。"""
    compose_vars = parse_compose_vars()
    _, entries = read_env_entries()
    running_env = read_container_env(FLIGHT_CONTAINER)
    vision_env = read_container_env(VISION_CONTAINER)
    keys = [k for k in compose_vars if k not in PARAMS_READONLY]
    keys += sorted(k for k in entries if k not in compose_vars and k not in PARAMS_READONLY)
    params = []
    for key in keys:
        cv = compose_vars.get(key)
        default = cv["default"] if cv else None
        in_env = key in entries
        effective = entries[key]["value"] if in_env else default
        running = None
        for env in (running_env, vision_env):
            if env is not None and key in env:
                running = env[key]
                break
        params.append({
            "key": key,
            "default": default,
            "effective": effective,
            "source": "env" if in_env else "default",
            "running": running,
            "stale": running is not None and effective is not None and running != effective,
            "choices": list(STACK_CHOICES[key]) if key in STACK_CHOICES else None,
            "comment": (cv["comment"] if cv and cv["comment"] else
                        entries[key]["comment"] if in_env else ""),
            "known": cv is not None,
        })
    return {
        "flight_up": container_running(FLIGHT_CONTAINER),
        "compose_config_error": compose_config_check(),
        "env_file": ENV_FILE,
        "params": params,
    }


def _format_env_value(value):
    # 含空白或 # 的值要加引号，否则 compose 会把 # 后面当注释截掉
    if value == "" or re.search(r"[\s#'\"]", value) is None:
        return value
    return '"' + value + '"'


def write_env_params(changes, removes, note):
    """把 changes 写进 .env、把 removes 从 .env 删掉。调用方已持有 _ENV_LOCK。
    返回 (ok, 备份路径, 错误信息)。写前备份，写后 compose config 校验，
    校验不过就从备份恢复。"""
    lines, entries = read_env_entries()
    safe_note = re.sub(r"[^A-Za-z0-9_.-]+", "-", note or "edit").strip("-")[:40] or "edit"
    backup = f"{ENV_FILE}.bak-{time.strftime('%Y%m%d-%H%M%S')}-params-{safe_note}"
    if os.path.exists(ENV_FILE):
        shutil.copy2(ENV_FILE, backup)
    else:
        backup = None

    drop = set()
    for key in removes:
        drop.update(entries[key]["idx"])
    new_lines = list(lines)
    appended = []
    for key, value in changes.items():
        formatted = _format_env_value(value)
        if key in entries:
            for i in entries[key]["idx"]:
                k_part, rest = lines[i].split("=", 1)
                _, tail = _parse_env_value(rest)
                new_lines[i] = f"{k_part}={formatted}{tail}"
        else:
            appended.append(f"{key}={formatted}")
    new_lines = [l for i, l in enumerate(new_lines) if i not in drop]
    if appended:
        new_lines.append("")
        new_lines.append(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} 由地面站 POST /params 写入")
        new_lines.extend(appended)

    with open(ENV_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(new_lines) + "\n")

    err = compose_config_check()
    if err:
        if backup:
            shutil.copy2(backup, ENV_FILE)
        else:
            os.remove(ENV_FILE)
        return False, backup, f"写入后 docker compose config 校验失败，已回滚：{err}"
    return True, backup, ""


def get_local_ip():
    # 不真的发包，借connect()让内核按路由表选出本机对外用的网卡IP，
    # 常见的"拿本机局域网IP"技巧，UDP这里不会真的握手/发包。
    #
    # ⚠️ 只在 GET /status 里用——它返回的是"按默认路由算出来的那一个IP"，
    # 这台机器有线(enP8p1s0)+WiFi(wlP1p1s0)两个网段同时在用，默认路由
    # 只会选中其中一个。对 /status 无所谓（客户端是主动连上来的，本来就
    # 知道该用哪个地址）；但对 UDP 心跳是致命的，心跳那边**不要**用这个，
    # 见 list_broadcast_ifaces() / beacon_loop() 的说明。
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "0.0.0.0"
    finally:
        s.close()


# ---- UDP心跳的网卡枚举（2026-09-01新增，见下面beacon_loop的根因说明）----
# 纯stdlib的ioctl取网卡地址，不引第三方依赖(netifaces/psutil)，也不每2秒
# fork一个`ip addr`子进程——心跳循环是常驻热路径，ioctl开销可以忽略。
# 这几个SIOCGIF*常量是Linux ABI固定值，跨架构(x86_64/aarch64)一致。
_SIOCGIFADDR = 0x8915      # 取接口IPv4地址
_SIOCGIFFLAGS = 0x8913     # 取接口标志位
_SIOCGIFBRDADDR = 0x8919   # 取接口广播地址
_IFF_UP = 0x1
_IFF_BROADCAST = 0x2
_IFF_LOOPBACK = 0x8
_IFF_RUNNING = 0x40        # 有物理链路(carrier)，光有IFF_UP不够——docker0
                           # 这类网桥没插东西时IFF_UP仍然是1


def _ifreq(sock, request, ifname):
    return fcntl.ioctl(sock.fileno(), request,
                       struct.pack("256s", ifname[:15].encode("utf-8")))


def list_broadcast_ifaces():
    """返回 [(网卡名, 本机IP, 该网段广播地址), ...]，只含 up+running+可广播
    +非loopback 的IPv4接口。

    每次心跳都重新枚举一遍（而不是启动时算一次缓存住）——网卡IP是会变的：
    WiFi换AP、DHCP续租到新地址、开机时网络还没就绪，都会让启动那一刻算出
    的地址过期。2026-09-01实测踩过这个坑，见beacon_loop()的说明。
    """
    out = []
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _, ifname in socket.if_nameindex():
            try:
                flags = struct.unpack("H", _ifreq(s, _SIOCGIFFLAGS, ifname)[16:18])[0]
                if flags & _IFF_LOOPBACK:
                    continue
                if not (flags & _IFF_UP and flags & _IFF_RUNNING):
                    continue
                if not flags & _IFF_BROADCAST:
                    continue
                addr = socket.inet_ntoa(_ifreq(s, _SIOCGIFADDR, ifname)[20:24])
                brd = socket.inet_ntoa(_ifreq(s, _SIOCGIFBRDADDR, ifname)[20:24])
            except OSError:
                # 没配IPv4地址的接口(只有IPv6/刚up还没拿到租约)会在这里报
                # EADDRNOTAVAIL，属于正常情况，跳过即可
                continue
            if addr.startswith("169.254."):
                continue   # link-local自配地址，没有真实网段可言
            out.append((ifname, addr, brd))
    finally:
        s.close()
    return out


def kill_vision_detector_processes(sensor_id=None):
    """sensor_id=None杀掉两路相机全部检测进程；传具体sensor_id只杀那一路，
    另一路不受影响(2026-08-25两路相机接上后新增，见文件头_VISION_NODE_
    PATTERN注释)。"""
    if not container_running(VISION_CONTAINER):
        return ""
    check = subprocess.run(
        ["docker", "exec", VISION_CONTAINER, "bash", "-c", _vision_node_grep_cmd(sensor_id)],
        capture_output=True, text=True,
    )
    pids = check.stdout.strip()
    if pids:
        subprocess.run(["docker", "exec", VISION_CONTAINER, "bash", "-c", f"kill -9 {pids}"],
                        capture_output=True, text=True)
        time.sleep(1)
    return pids


def colcon_build_vision():
    result = subprocess.run(
        ["docker", "exec", "-w", "/opt/vision_ws", VISION_CONTAINER, "bash", "-c",
         "source /opt/ros/humble/setup.bash && colcon build --symlink-install"],
        capture_output=True, text=True, timeout=120,
    )
    return result.returncode == 0, (result.stdout + result.stderr)


# cam槽位默认值——两路相机现在都要能按需独立切换跑YOLO/QR/AprilTag中
# 任意一个(2026-08-25第二路相机接上后确定的使用模式，不是"前视固定YOLO/
# 下视固定QR"绑死的分工，见docs/vision_stack.md阶段2)。这里的sensor_id/
# label/mjpeg_port只是给调用方没有显式传参时的合理默认值，全部可以在
# 请求体里覆盖。
_CAM_DEFAULTS = {
    "cam0": {"sensor_id": 0, "label": "front", "mjpeg_port": 8080},
    "cam1": {"sensor_id": 1, "label": "down", "mjpeg_port": 8081},
}


def start_vision_mode(cam, mode, params):
    """直接拼ros2 run/launch命令docker exec -d起，不经过vision_stack_up.sh
    ——那个脚本会顺带docker compose up flight-stack-hw、每次都重新colcon
    build，对"切换检测模式"这个高频操作来说是不必要的副作用和延迟，这里
    假定容器已经跑着、workspace已经build过(POST /stack up的时候会build
    一次)，只做"杀旧进程(仅这一路相机)+起新进程"这一步。

    2026-08-25重写：cam0/cam1两路相机各自独立控制，不再是全局唯一一个
    检测节点；地面站视频统一MJPEG拉流，RTP推流(gcs_ip/stream_mode等参数)
    已经整个删掉，见read_hw.md同日条目。

    mode取值：yolo|qr|apriltag|raw|stop。2026-09-23新增raw(纯视频直通，
    不推理)，同时yolo模式支持params["mjpeg_draw_detections"]=false
    ——检测照跑、地面拉到的画面不叠加检测框。"""
    if cam not in ("cam0", "cam1"):
        return False, 'cam必须是cam0或cam1'
    defaults = _CAM_DEFAULTS[cam]
    ns = read_namespace()
    sensor_id = params.get("sensor_id", defaults["sensor_id"])
    kill_vision_detector_processes(sensor_id=sensor_id)
    if mode == "stop":
        return True, f"{cam}(sensor_id={sensor_id})已停止检测节点"

    label = params.get("label", defaults["label"])
    camera_width = params.get("camera_width", 1280)
    camera_height = params.get("camera_height", 720)
    camera_framerate = params.get("camera_framerate", 30)
    mjpeg_enabled = params.get("mjpeg_enabled", True)
    mjpeg_port = params.get("mjpeg_port", defaults["mjpeg_port"])
    mjpeg_args = f"-p mjpeg_enabled:={'true' if mjpeg_enabled else 'false'} -p mjpeg_port:={mjpeg_port} "

    if mode == "yolo":
        publish_rate_hz = params.get("publish_rate_hz", 15.0)
        topic = params.get("topic", f"vision/{label}/detections")
        frame_id = params.get("frame_id", f"camera_{label}")
        # 2026-09-23新增：mjpeg_draw_detections=false → 地面拉到的是相机
        # 原始画面(直通)，检测消息照常发布。完全不要推理只要视频用
        # mode="raw"，见下面raw分支。
        draw_detections = params.get("mjpeg_draw_detections", True)
        ros_args = (
            f"-p sensor_id:={sensor_id} -p camera_width:={camera_width} "
            f"-p camera_height:={camera_height} -p camera_framerate:={camera_framerate} "
            f"-p publish_rate_hz:={publish_rate_hz} -p topic:={topic} -p frame_id:={frame_id} "
            f"-p mjpeg_draw_detections:={'true' if draw_detections else 'false'} "
            f"{mjpeg_args}"
        )
        cmd = (
            "source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"exec ros2 run vision_stack yolo_detector_node --ros-args -r __ns:=/{ns} {ros_args} "
            f"> /tmp/{cam}_yolo_detector_node.log 2>&1"
        )
        subprocess.Popen(["docker", "exec", "-d", "-w", "/opt/vision_ws", VISION_CONTAINER, "bash", "-c", cmd])
        return True, f"{cam}(sensor_id={sensor_id}) yolo_detector_node已启动 (namespace={ns})"

    if mode == "qr":
        publish_rate_hz = params.get("publish_rate_hz", 5.0)
        topic = params.get("topic", f"vision/{label}/qr_detections")
        frame_id = params.get("frame_id", f"camera_{label}")
        cmd = (
            "source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"exec ros2 run vision_stack qr_detect_node --ros-args -r __ns:=/{ns} "
            f"-p sensor_id:={sensor_id} -p camera_width:={camera_width} "
            f"-p camera_height:={camera_height} -p camera_framerate:={camera_framerate} "
            f"-p publish_rate_hz:={publish_rate_hz} -p topic:={topic} -p frame_id:={frame_id} "
            f"{mjpeg_args}"
            f"> /tmp/{cam}_qr_detect_node.log 2>&1"
        )
        subprocess.Popen(["docker", "exec", "-d", "-w", "/opt/vision_ws", VISION_CONTAINER, "bash", "-c", cmd])
        return True, f"{cam}(sensor_id={sensor_id}) qr_detect_node已启动 (namespace={ns})"

    if mode == "apriltag":
        frame_id = params.get("frame_id", f"camera_{label}")
        cmd = (
            "source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"exec ros2 launch vision_stack apriltag_stack.launch.py "
            f"namespace:={ns} sensor_id:={sensor_id} frame_id:={frame_id} label:={label} "
            f"mjpeg_enabled:={'true' if mjpeg_enabled else 'false'} mjpeg_port:={mjpeg_port} "
            f"> /tmp/{cam}_apriltag_stack.log 2>&1"
        )
        subprocess.Popen(["docker", "exec", "-d", "-w", "/opt/vision_ws", VISION_CONTAINER, "bash", "-c", cmd])
        return True, f"{cam}(sensor_id={sensor_id}) apriltag_stack已启动 (namespace={ns})"

    if mode == "raw":
        # 2026-09-23新增：纯视频直通，不加载engine、不做任何推理，只把
        # 相机原始帧推给MJPEG。跟同一路相机上的yolo/qr/apriltag互斥
        # (一路相机同一时刻只能有一个Argus会话)，所以"既要检测又要干净
        # 画面"不是起两个节点，而是mode="yolo" + mjpeg_draw_detections
        # =false，见上面yolo分支。
        publish_rate_hz = params.get("publish_rate_hz", 15.0)
        jpeg_quality = params.get("jpeg_quality", 80)
        cmd = (
            "source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"exec ros2 run vision_stack video_stream_node --ros-args -r __ns:=/{ns} "
            f"-p sensor_id:={sensor_id} -p camera_width:={camera_width} "
            f"-p camera_height:={camera_height} -p camera_framerate:={camera_framerate} "
            f"-p publish_rate_hz:={publish_rate_hz} -p jpeg_quality:={jpeg_quality} "
            f"-p mjpeg_port:={mjpeg_port} "
            f"> /tmp/{cam}_video_stream_node.log 2>&1"
        )
        subprocess.Popen(["docker", "exec", "-d", "-w", "/opt/vision_ws", VISION_CONTAINER, "bash", "-c", cmd])
        return True, f"{cam}(sensor_id={sensor_id}) video_stream_node已启动，直通视频 http://<ip>:{mjpeg_port}/"

    return False, f"未知mode: {mode} (可选 yolo|qr|apriltag|raw|stop)"


def beacon_loop():
    """每 BEACON_INTERVAL_SEC 秒，往**每一个**在用网段各广播一份心跳。

    ⚠️ 2026-09-01重写。原来是这么写的（一个常驻socket + 一个受限广播地址）：

        sock = socket.socket(...)                 # 循环外建一次
        sock.sendto(payload, ("255.255.255.255", BEACON_PORT))

    问题：**255.255.255.255 只会从一个网口出去**。socket既没SO_BINDTODEVICE
    也没绑源地址，内核对这个目的地址只按路由表挑**一条默认路由**。这台机器
    同时挂着有线(enP8p1s0，雷达网段)和WiFi(wlP1p1s0，地面站网段)，两条默认
    路由谁的metric小谁赢——而metric是DHCP/NM按当时情况给的，会变（实测同一块
    WiFi网卡在两次连接里分别拿到过600和20600，后者直接输给有线的20100，
    出口就整个换到了另一个网口）。实测分网口tcpdump：wlP1p1s0抓到包、
    enP8p1s0一个都没有。也就是说"地面站能不能发现飞机"取决于一个没人显式
    配过、还会自己变的路由metric，这不是能依赖的行为。

    改法：每轮重新枚举网卡，逐个网段用**该网段自己的**源地址和广播地址发。
    - 源地址显式bind到该网卡的IP → 这是"让包从指定网口出去"的手段（多网段
      主机上单靠目的地址没法指定出口）
    - 目的地址用该网段的定向广播(如192.168.2.255)，再补一发255.255.255.255
      兜底（有些AP/协议栈只放行其中一种，两种都发成本可以忽略）
    - payload里的"ip"改成**该网卡自己的地址**：原来用get_local_ip()（按默认
      路由算），在"只从一个网口发"的旧行为下两者恰好自洽；现在既然逐网段都
      发，就必须逐网段给出对应地址，否则从有线段收到的心跳会告诉地面站一个
      WiFi的IP，那个地址在有线段根本连不上
    - 每轮重新枚举而不是启动时算一次：网卡IP会变(换AP/DHCP续租/开机时网络
      还没就绪)，缓存住就会过期
    每个网卡每轮用完即弃一个socket：2秒一次、网卡就那么几个，开销无所谓。

    ⚠️ 曾经误判、已订正（2026-09-01同日，避免后人重犯）：一度以为"循环外
    建的常驻socket会把源地址钉死在旧IP上"。**不成立**——ss显示该socket绑的是
    0.0.0.0(INADDR_ANY)，源IP是内核每次sendto按路由重新选的，网卡换IP会自动
    跟上。当时的"证据"是切完WiFi后抓到的包源地址还是旧网段的182.168.1.65，
    实为看漏了中间又切回过原网络（NetworkManager日志里22:06:28那次），
    抓包时接口本来就是那个地址，行为完全正确。逐网段发这个改动本身仍然
    需要，但理由只有上面"单出口"这一条。
    """
    while True:
        try:
            # 这几项对所有网卡都一样，每轮只算一次——container_running()和
            # read_stack_config()都要fork docker子进程，按网卡数重复算会把
            # 2秒的周期吃满。
            flight_up = container_running(FLIGHT_CONTAINER)
            # 2026-09-04新增：见sample_cpu_usage_pct()定义处的说明——每轮
            # beacon都调用一次，用这2秒的心跳周期本身当采样间隔，不额外sleep。
            sample_cpu_usage_pct()
            base = {
                "namespace": read_namespace(),
                "control_port": CONTROL_PORT,
                "hostname": socket.gethostname(),
                "flight_up": flight_up,
                "vision_up": container_running(VISION_CONTAINER),
                # 2026-08-29新增：四个运行模式也随广播发出，地面站不用轮询
                # /status 就能直接显示。只带运行值，不带pending（广播包要小）。
                "stack": read_stack_config()[0],
                # 2026-09-04新增：见flight_started_at()/read_cpu_temp_c()
                # 定义处的说明。flight_up已经是false时不必再花一次docker
                # inspect去确认"启动时间"，直接给None。
                "flight_started_at": flight_started_at() if flight_up else None,
                "cpu_temp_c": read_cpu_temp_c(),
                # 2026-09-04新增：原来的"飞控温度"(mavros/imu/temperature_imu)
                # 被确认是不可信的死数(见DEBUG_JOURNAL同日记录)，用户要求换成
                # 机载计算机CPU占用率——见sample_cpu_usage_pct()定义处的说明。
                "cpu_usage_pct": _cpu_usage_pct_cache,
            }
            ifaces = list_broadcast_ifaces()
            if not ifaces:
                print("[beacon] 没有可用于广播的网卡，本轮跳过", file=sys.stderr)
            for ifname, addr, brd in ifaces:
                payload = json.dumps(dict(base, ip=addr)).encode("utf-8")
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    # 绑到这块网卡自己的地址（端口0=让内核随便挑），源地址
                    # 就锁死在本网段，不会被路由表选到别的网口去
                    sock.bind((addr, 0))
                    for dst in (brd, "255.255.255.255"):
                        try:
                            sock.sendto(payload, (dst, BEACON_PORT))
                        except OSError as e:
                            # 单个目的地址发失败不影响另一个/别的网卡
                            print(f"[beacon] {ifname}({addr}) -> {dst} 发送失败: {e!r}",
                                  file=sys.stderr)
                finally:
                    sock.close()
        except Exception as e:
            print(f"[beacon] 本轮失败: {e!r}", file=sys.stderr)
        time.sleep(BEACON_INTERVAL_SEC)


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _send_json(self, status, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self._cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw)
        except Exception:
            return {}

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors_headers()
        self.end_headers()

    def do_GET(self):
        if self.path == "/status":
            running, pending, mismatch = read_stack_config()
            flight_up = container_running(FLIGHT_CONTAINER)
            self._send_json(200, {
                "namespace": read_namespace(),
                "flight_up": flight_up,
                "vision_up": container_running(VISION_CONTAINER),
                "hostname": socket.gethostname(),
                "ip": get_local_ip(),
                # 2026-08-29新增：当前实际运行的四个模式，见 read_stack_config()
                "stack": running,
                # 配置里的待生效值；跟 stack 不一致的 key 列在 stack_mismatch，
                # 地面站可据此提示"改了配置但没重启生效"
                "stack_pending": pending,
                "stack_mismatch": mismatch,
                # 2026-09-04新增：见flight_started_at()/read_cpu_temp_c()
                # 定义处的说明，跟UDP广播里同名字段同一份数据。
                "flight_started_at": flight_started_at() if flight_up else None,
                "cpu_temp_c": read_cpu_temp_c(),
                # cpu_usage_pct读的是beacon_loop()那份缓存（见
                # sample_cpu_usage_pct()说明，不在这里现场sleep重新采样），
                # 在beacon_loop()第一轮跑完之前(进程刚启动的头两秒内)会是
                # None，之后每2秒跟着心跳更新一次。
                "cpu_usage_pct": _cpu_usage_pct_cache,
            })
            return
        if self.path == "/api":
            # 自描述契约。2026-08-29新增：合法取值直接从 STACK_CHOICES/HW_FORBIDDEN
            # 生成，跟服务端校验用的是同一份常量，**不可能出现文档与实现漂移**。
            # 地面站可以在连上机器时拉一次，据此动态生成下拉框，不用硬编码取值表。
            self._send_json(200, {
                "version": 1,
                "control_port": CONTROL_PORT,
                "beacon_port": BEACON_PORT,
                "beacon_interval_sec": BEACON_INTERVAL_SEC,
                "endpoints": {
                    "GET /status": "当前状态：namespace/flight_up/vision_up/hostname/ip/"
                                   "stack(运行中的四个模式)/stack_pending/stack_mismatch/"
                                   "flight_started_at/cpu_temp_c/cpu_usage_pct",
                    "GET /api": "本文档",
                    "POST /stack": {
                        "body": {"stack": ["flight", "vision"],
                                 "action": ["up", "down", "restart"],
                                 "confirm": "flight 的 down/restart 必须为 true"},
                        "note": "启停容器。改完模式后用 action=up 让新模式生效",
                    },
                    "POST /stack/mode": {
                        "body": {"localization": None, "slam": None,
                                 "planner": None, "controller": None},
                        "note": "只写 .env 设定启动环境，不碰容器；四项均可省略，"
                                "省略的沿用现值。返回 restart_required 指示是否需要重启",
                    },
                    "POST /vision/mode": {"body": {
                        "mode": ["yolo", "qr", "apriltag", "raw", "stop"],
                        "cam": ["cam0", "cam1", "all"],
                        "mjpeg_draw_detections": "仅mode=yolo时有效，false=地面拉到原始画面不叠加检测框",
                    }},
                    "POST /rename": {"body": {"namespace": "新的 NAMESPACE"}},
                    "GET /params": "全部可调参数：key/default/effective/source/running/"
                                   "stale/choices/comment/known，外加 flight_up/"
                                   "compose_config_error",
                    "POST /params": {
                        "body": {"changes": {"KEY": "值"}, "remove": ["KEY"],
                                 "note": "备份文件名后缀", "restart": False,
                                 "allow_unknown": False},
                        "note": "只写改动的行，写前备份、写后 compose 校验失败自动回滚；"
                                "restart=true 再 force-recreate flight-stack 并回读核对",
                    },
                },
                "stack_choices": {k: list(v) for k, v in STACK_CHOICES.items()},
                "hw_forbidden": {k: dict(v) for k, v in HW_FORBIDDEN.items()},
                "response_format": {
                    "success": {"ok": True, "...": "handler 返回的结构化字段直接平铺"},
                    "failure": {"ok": False, "message": "错误原因（HTTP 400）"},
                },
            })
            return
        if self.path == "/params":
            try:
                self._send_json(200, collect_params())
            except Exception as e:
                self._send_json(500, {"ok": False, "message": f"读取参数失败：{e!r}"})
            return
        self._send_json(404, {"error": "not found"})

    def do_POST(self):
        body = self._read_json_body()
        if self.path == "/stack":
            ok, msg = self._handle_stack(body)
        elif self.path == "/stack/mode":
            ok, msg = self._handle_stack_mode(body)
        elif self.path == "/vision/mode":
            ok, msg = self._handle_vision_mode(body)
        elif self.path == "/rename":
            ok, msg = self._handle_rename(body)
        elif self.path == "/params":
            ok, msg = self._handle_params(body)
        else:
            audit_log(self.client_address[0], "POST", self.path, body, "404")
            self._send_json(404, {"error": "not found"})
            return
        # handler 可以返回 str（纯文本消息）或 dict（结构化数据）。
        # 返回 dict 时直接平铺进响应体，不要再套一层 JSON 字符串——
        # 否则客户端要 JSON.parse(resp.message) 二次解析，且跟其它端点不一致。
        audit_log(self.client_address[0], "POST", self.path, body,
                  f"{'ok' if ok else 'fail'}: "
                  f"{json.dumps(msg, ensure_ascii=False) if isinstance(msg, dict) else msg}")
        payload = {"ok": ok}
        if isinstance(msg, dict):
            payload.update(msg)
        else:
            payload["message"] = msg
        self._send_json(200 if ok else 400, payload)

    def _handle_stack(self, body):
        stack = body.get("stack")
        action = body.get("action")
        confirm = bool(body.get("confirm", False))
        if stack not in ("flight", "vision"):
            return False, "stack必须是flight或vision"
        if action not in ("up", "down", "restart"):
            return False, "action必须是up/down/restart"
        service = FLIGHT_SERVICE if stack == "flight" else VISION_SERVICE
        if stack == "flight" and action in ("down", "restart") and not confirm:
            return False, "停止/重启flight-stack会直接掐断飞控软件，需要显式带 confirm: true"

        if stack == "vision" and action in ("down", "restart"):
            kill_vision_detector_processes()

        if action == "up":
            ns = read_namespace()
            ok, out = run_compose(["up", "-d", service], extra_env={"NAMESPACE": ns})
            if ok and stack == "vision":
                build_ok, build_out = colcon_build_vision()
                ok = ok and build_ok
                out += "\n" + build_out
        elif action == "down":
            ok, out = run_compose(["stop", service])
        else:  # restart
            ns = read_namespace()
            # 2026-08-29：flight-stack的restart从`docker compose restart`改成
            # `up -d --force-recreate`。原因：容器的环境变量是**创建时固化**的，
            # `restart`只是把同一个容器停了再起，改完.env(VEHICLE_MASS_KG/
            # CONTROLLER/LOCALIZATION_SOURCE...)之后走restart路径**不会生效**，
            # 而且不报错——用户从地面站点"重启"，看到容器起来了，以为新配置生效了，
            # 实际还在跑旧参数。这是本仓库反复出现的"静默失效"类问题之一
            # (见read_hw.md 2026-08-28/29的多条记录)。
            # `up -d --force-recreate`会销毁重建容器，重新读取.env和compose，
            # 语义上也更符合用户点"重启"的预期。
            #
            # ⚠️ vision-stack保持`restart`不变，不是漏改：它的colcon build
            # 产物(/opt/vision_ws的build/install/log)全在容器可写层，没有挂载
            # 出来。force-recreate会把容器连同编译产物一起销毁，下面那个
            # colcon_build_vision()就从增量变成全量重编，而它的timeout只有
            # 120秒，在Jetson上大概率超时失败。vision-stack需要重新读配置时
            # 应该用up(它本来就是up -d)或者手动down+up。
            # 2026-09-23订正措辞：原文写的是"vision-stack没有任何volume挂载"，
            # 不准确——compose里实际挂了三个(/tmp/argus_socket、./models、
            # ./src/vision_stack→/opt/vision_ws/src/vision_stack)，所以改飞机上
            # 的vision_stack源码**不需要docker build**，scp到宿主机就进容器了，
            # 只要再colcon build一次。真正没挂出来、会被force-recreate清掉的
            # 是build/install产物，上面这个决策本身不受影响。
            if stack == "flight":
                ok, out = run_compose(["up", "-d", "--force-recreate", service],
                                      extra_env={"NAMESPACE": ns})
            else:
                ok, out = run_compose(["restart", service], extra_env={"NAMESPACE": ns})
            if ok and stack == "vision":
                build_ok, build_out = colcon_build_vision()
                ok = ok and build_ok
                out += "\n" + build_out
        return ok, out[-2000:]

    def _handle_stack_mode(self, body):
        """设定 flight-stack 的启动环境（四个模式），**不碰容器**。2026-08-29新增。

        设计上跟容器操作解耦：这个端点只负责把模式写进 .env；地面站要让它生效，
        再单独调已有的 POST /stack {stack:"flight", action:"up"}。
        好处是配置和启动是两个独立动作——可以先把模式设好、确认无误，再决定什么
        时候真正重启飞控软件，不会出现"改个规划器顺手就把飞控掐了"。

        body: {localization?, slam?, planner?, controller?}
        四项都可省略——省略的沿用 .env 里的现值，支持"只换控制器"这种局部改动。
        不需要 confirm：只写配置文件，不影响正在运行的容器。
        """
        field_map = {
            "localization": "LOCALIZATION_SOURCE",
            "slam":         "SLAM_BACKEND",
            "planner":      "PLANNER",
            "controller":   "CONTROLLER",
        }
        req = {}
        for short, key in field_map.items():
            if body.get(short) is not None:
                req[key] = str(body[short])
        if not req:
            return False, f"至少要指定一项：{'/'.join(field_map)}"

        # 校验放在动手之前——非法组合在这里就拒掉，不要等容器起来再fail fast退出
        running, pending, _ = read_stack_config()
        merged = dict(pending or running)
        merged.update(req)
        ok, err = validate_stack_choice(merged, deploy_target="hw",
                                        num_agents=os.environ.get("NUM_AGENTS", "1"))
        if not ok:
            return False, f"校验未通过：{err}"

        try:
            update_env_file(req)
        except Exception as e:
            return False, f"写入 .env 失败：{e!r}"

        # 只写配置，不碰容器。重新解析一次，把"待生效值"和"跟当前运行值的差异"
        # 一起返回，地面站可以直接显示"这些改动要重启才生效"。
        running_now, pending_now, mismatch_now = read_stack_config()
        return True, {
            "applied": req,
            "pending": pending_now,
            "running": running_now,
            "mismatch": mismatch_now,
            "restart_required": bool(mismatch_now),
            "note": ('已写入 .env。容器未被改动——要让新模式生效，'
                     '再调 POST /stack {"stack":"flight","action":"up"}'
                     if mismatch_now else "已写入 .env，与当前运行值一致，无需重启"),
        }

    def _handle_vision_mode(self, body):
        mode = body.get("mode")
        if mode not in ("yolo", "qr", "apriltag", "raw", "stop"):
            return False, "mode必须是yolo/qr/apriltag/raw/stop"
        cam = body.get("cam")
        if cam == "all":
            if mode != "stop":
                return False, 'cam="all"只能配合mode="stop"用，起检测节点必须指定具体是cam0还是cam1'
            if not container_running(VISION_CONTAINER):
                return False, "vision-stack容器没在跑，先POST /stack起容器 ({\"stack\":\"vision\",\"action\":\"up\"})"
            kill_vision_detector_processes()
            return True, "已停止两路相机上所有检测节点"
        if cam not in ("cam0", "cam1"):
            return False, 'cam必须是cam0/cam1/all(all只能配合mode="stop")'
        if not container_running(VISION_CONTAINER):
            return False, "vision-stack容器没在跑，先POST /stack起容器 ({\"stack\":\"vision\",\"action\":\"up\"})"
        return start_vision_mode(cam, mode, body)

    def _handle_params(self, body):
        """POST /params，见文件头第7节。restart 不另要 confirm：地面站在发请求
        前已经弹过确认框，而且 restart 本身就是一个显式开关，不会被空请求
        误触发（没带 changes/remove 直接拒绝）。"""
        changes = body.get("changes") or {}
        removes = body.get("remove") or []
        note = str(body.get("note") or "edit")
        restart = bool(body.get("restart", False))
        allow_unknown = bool(body.get("allow_unknown", False))
        if not isinstance(changes, dict) or not isinstance(removes, list):
            return False, "changes 必须是对象、remove 必须是数组"
        changes = {str(k): str(v) for k, v in changes.items()}
        removes = [str(k) for k in removes]
        if not changes and not removes:
            return False, "没有任何改动（changes 和 remove 都是空的）"

        with _ENV_LOCK:
            compose_vars = parse_compose_vars()
            _, entries = read_env_entries()
            for key in list(changes) + removes:
                if not _ENV_KEY_RE.match(key):
                    return False, f"参数名 {key!r} 不合法"
                if key in PARAMS_READONLY:
                    return False, f"{key} 不能在这里改：{PARAMS_READONLY[key]}"
            both = set(changes) & set(removes)
            if both:
                return False, f"{'/'.join(sorted(both))} 同时出现在 changes 和 remove 里"
            unknown = [k for k in changes if k not in compose_vars and k not in entries]
            if unknown and not allow_unknown:
                return False, (f"docker-compose.hw.yml 没有引用 {'/'.join(unknown)}，写进 .env "
                               "也不会传进容器（多半是拼错了）；确认要写请带 allow_unknown: true")
            not_in_env = [k for k in removes if k not in entries]
            if not_in_env:
                return False, f"{'/'.join(not_in_env)} 不在 .env 里，没有可删的行"
            for key, value in changes.items():
                if "\n" in value or "\r" in value or '"' in value or "'" in value:
                    return False, f"{key} 的值不能含换行或引号"

            # 四个模式项沿用 /stack/mode 的校验，非法组合在写之前就拒掉
            effective = {k: (entries[k]["value"] if k in entries else v["default"])
                         for k, v in compose_vars.items()}
            for k in removes:
                effective[k] = compose_vars.get(k, {}).get("default")
            effective.update(changes)
            if any(k in STACK_KEYS for k in list(changes) + removes):
                stack = {k: effective[k] for k in STACK_KEYS if effective.get(k)}
                ok, err = validate_stack_choice(stack, deploy_target="hw",
                                                num_agents=effective.get("NUM_AGENTS") or "1")
                if not ok:
                    return False, f"校验未通过：{err}"

            ok, backup, err = write_env_params(changes, removes, note)
        if not ok:
            return False, err

        result = {
            "changed": changes,
            "removed": removes,
            "backup": backup,
            "restarted": False,
            "verify_mismatch": [],
        }
        summary = f"已写入 .env（{len(changes)} 项修改、{len(removes)} 项恢复基线），备份 {os.path.basename(backup) if backup else '无'}"
        if not restart:
            result["message"] = summary + "。容器未动，新值要等 flight-stack 下次 force-recreate 才生效"
            return True, result

        ns = read_namespace()
        up_ok, out = run_compose(["up", "-d", "--force-recreate", FLIGHT_SERVICE],
                                 extra_env={"NAMESPACE": ns})
        result["restarted"] = up_ok
        if not up_ok:
            result["compose_output"] = out[-2000:]
            return False, summary + f"，但重启 flight-stack 失败：{out[-500:]}"
        running_env = read_container_env(FLIGHT_CONTAINER) or {}
        mismatch = []
        for key in list(changes) + removes:
            want = effective.get(key)
            if key in running_env and want is not None and running_env[key] != want:
                mismatch.append(key)
        result["verify_mismatch"] = mismatch
        result["message"] = summary + ("，flight-stack 已重建，回读核对一致" if not mismatch else
                                       f"，flight-stack 已重建，但这些键容器里的值跟 .env 对不上：{'/'.join(mismatch)}")
        return True, result

    def _handle_rename(self, body):
        new_ns = body.get("namespace")
        confirm = bool(body.get("confirm", False))
        if not new_ns or not str(new_ns).strip():
            return False, "namespace不能为空"
        if not confirm:
            return False, "改命名空间会重启flight-stack/vision-stack两个容器，需要显式带 confirm: true"
        new_ns = str(new_ns).strip()

        write_namespace(new_ns)
        kill_vision_detector_processes()
        ok1, out1 = run_compose(["stop", FLIGHT_SERVICE, VISION_SERVICE])
        ok2, out2 = run_compose(["up", "-d", FLIGHT_SERVICE, VISION_SERVICE],
                                 extra_env={"NAMESPACE": new_ns})
        ok3, out3 = (True, "")
        if ok2:
            # NAMESPACE变了，compose会检测到配置hash变化重新创建vision-stack
            # 容器——重建过的容器install/在旧容器可写层里，跟着一起没了，
            # 得重新colcon build一次（跟vision_stack_up.sh头部注释同一个坑）。
            ok3, out3 = colcon_build_vision()
        ok = ok1 and ok2 and ok3
        msg = f"已改成{new_ns}。stop: {out1[-300:]} | up: {out2[-300:]} | build: {out3[-300:]}"
        return ok, msg


def main():
    threading.Thread(target=beacon_loop, daemon=True).start()
    server = http.server.ThreadingHTTPServer(("0.0.0.0", CONTROL_PORT), Handler)
    print(f"[control_server] 监听 http://0.0.0.0:{CONTROL_PORT}，UDP心跳广播到 *:{BEACON_PORT}",
          file=sys.stderr)
    server.serve_forever()


if __name__ == "__main__":
    main()
