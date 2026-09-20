"""GCS后端（容器B），2026-08-20新增。

只负责浏览器做不到的几件事——本质上都是"改一个只在启动/运行时读一次
的东西+重启/启停相关容器/进程"，跟容器A（rosbridge）完全独立，不经过
它、不需要ROS2：

1. GCS自己的CycloneDDS网络配置（cyclonedds_gcs.xml）：本地改文件+
   `docker restart`重启gcs容器自己。
2. 真机flight-stack的规划/避障调参白名单：SSH到Jetson改`.env`文件+
   SSH远程`docker compose -f docker-compose.hw.yml up -d`重建容器让
   新值生效。SSH别名（nx01/nx02）复用《真机部署操作清单》阶段0已经
   配好的`~/.ssh/config`，不新增一套凭证管理，见方案文档8.1节的
   既定选型理由。
3. 机队启停（方案文档阶段6，2026-08-20新增）：仿真场景本地对
   sim-world/flight-stack-nx01/flight-stack-nx02三个容器做
   start/stop；真机场景SSH到Jetson做`docker compose -f
   docker-compose.hw.yml up -d`/`down`。

⚠️ 安全边界：这个容器挂了`/var/run/docker.sock`（等于拿到宿主机docker
daemon的完整控制权），是这套系统里权限最集中的一个点，见方案文档第五章
"容器B"那行提醒。2026-08-25删除了浏览器→这个后端之间的共享密钥鉴权
（`GCS_BACKEND_TOKEN`/`Authorization: Bearer`那套）——用户明确要求去掉，
理由是这个内部工具只在受信任局域网里用，多一层令牌反而经常跟真机的
X-Control-Token搞混（这次会话里连续踩过好几次，把两个概念不同的令牌
填错对象，见DEBUG_JOURNAL.md 2026-08-25记录）。同一天晚些时候，真机
自己那道X-Control-Token鉴权（本来是飞机端`control_server.py`强制的，
不是GCS这边能单方面绕过的）也被用户明确要求彻底删掉了——地面站和
飞机端都改了，见`docker_sim/scripts/control_server.py`同日改动，
`_hw_fleet_proxy()`不再往请求头里塞任何令牌。⚠️两道鉴权都删掉之后，
`/api/*`（这个后端）和飞机端8890端口（`control_server.py`）对局域网里
任何能连到这两个端口的人完全开放，包括机队启停、改命名空间、切换
检测模式这些写操作——只适合部署在真正受信任的内网，不能暴露到更大的
网络，这是用户主动接受的权衡（换取不用再折腾令牌反复配错的问题）。
"""
import json
import os
import re
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import requests
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI(title="GCS backend")

# 跨域——浏览器前端(8080端口)和这个后端(8000端口)不同源，没有这个浏览器
# 会直接拦掉fetch()。
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/healthz")
def healthz():
    return {"status": "ok"}


# ============================================================
# 1. GCS自己的网络配置：仿真/真机模式互斥切换
#
# 2026-08-30重构（比赛场景要求）：原来"仿真模式"/"真机模式"两个按钮只是
# 前端换pane，不切网络，同一DDS域里两种数据源共存——真实故障复现过一次：
# cyclonedds_gcs.xml残留几天前真机联调用的真实IP，GCS容器内rosapi_node的
# CycloneDDS参与者持续对不可达地址做UDP写入失败，拖垮整条DDS收发管线
# （进程不崩溃，但/rosapi/topics服务调用全部超时），前端表现为"卡片有壳
# 没数据"。改成两层隔离叠加：
#   ① ROS_DOMAIN_ID分域（仿真21/真机20，见根目录docker-compose.yml和
#      docker-compose.hw.yml）——结构性防止选手自己写的ROS2代码连错目标，
#      不经过网页也一样有效，只在这个模块负责GCS自己怎么跟着切。
#   ② 这个模块：GCS自己的CycloneDDS网络配置（本机回环 vs 站点真实IP）+
#      两者的持久化拆分（活文件 vs 真机参数快照，见下面_write_live_network/
#      _load_gcs_network_state的说明）。
# ============================================================

CYCLONEDDS_GCS_PATH = Path("/opt/gcs_config/cyclonedds_gcs.xml")
GCS_CONTAINER_NAME = os.environ.get("GCS_CONTAINER_NAME", "gcs-gcs-1")
GCS_NETWORK_STATE_PATH = Path(
    os.environ.get("GCS_NETWORK_STATE_PATH", "/opt/gcs_config/gcs_network_state.json")
)
_gcs_network_state_lock = threading.Lock()

SIM_INTERFACE_ADDRESS = "127.0.0.1"   # 仿真模式固定值，不需要用户输入
SIM_ROS_DOMAIN_ID = "21"              # 必须跟根目录docker-compose.yml的
                                       # sim-world/flight-stack保持一致
REAL_ROS_DOMAIN_ID = "20"             # 必须跟docker-compose.hw.yml保持一致

# 只用正则改这些属性值/区块，不解析/重写整个XML——保留文件里其余注释、
# 排版、Discovery/Peers那段是否被注释掉等原样不动，改动面越小，出错
# 时越容易看diff、越不容易把用户手工维护的注释搞坏。
_IFACE_RE = re.compile(r'(<NetworkInterface\b[^>]*\baddress=")([^"]*)(")')
# 2026-08-25改：原来_PEER_RE只匹配/替换*第一个*<Peer address="...">，只够
# 单机部署（GCS配1台Jetson）用——用户反馈"现在至少有两个真机"，CycloneDDS
# 的<Peers>本来就支持放多个<Peer address="X"/>子元素（静态互连列表），
# 改成整体重写<Peers>...</Peers>这个区块的内容，一次配多个IP。真要接入
# 更多机身，长期应该升级成CycloneDDS Discovery Server（把"两两互连"简化
# 成"大家连一个中心"，见cyclonedds_gcs.xml.example文件头的既有说明），
# 这次先把"能配多个静态peer"这个当下就能用的能力补上，不做discovery
# server那一步。
_PEERS_BLOCK_RE = re.compile(r'(<Peers>)(.*?)(</Peers>)', re.S)
_PEER_ADDR_RE = re.compile(r'<Peer\s+address="([^"]*)"\s*/>')


class GcsNetworkConfig(BaseModel):
    interface_address: str
    # 逗号/空白分隔的多个Jetson IP，比如"192.168.2.104, 192.168.2.107"，
    # 留空=不配置静态peer（纯靠AllowMulticast自动发现，同一个广播域内
    # 通常够用，跨子网/多播被屏蔽时才真的需要显式列出每台Jetson的IP）。
    peer_addresses: str = ""


class GcsModeReq(BaseModel):
    mode: str  # "sim" | "real"


# ---- 真机网络参数持久化快照：跟_load_hw_fleet_config()/_save_hw_fleet_
# config()（本文件"7. 真机集群远程控制"一节）同款读写模式——同样的锁、
# 同样的mkdir(parents=True, exist_ok=True)、同样的JSON容错读取，不新造
# 一套风格。只存用户手工填过的真机网络参数，独立于cyclonedds_gcs.xml这份
# 随时会被两种模式切换动作覆写的"活文件"，切到仿真模式绝不覆盖它——仿真
# 模式的参数是常量（SIM_INTERFACE_ADDRESS/无peer），不需要持久化。
def _load_gcs_network_state() -> dict:
    with _gcs_network_state_lock:
        if not GCS_NETWORK_STATE_PATH.exists():
            return {}
        try:
            return json.loads(GCS_NETWORK_STATE_PATH.read_text() or "{}")
        except json.JSONDecodeError:
            return {}


def _save_gcs_network_state(state: dict) -> None:
    with _gcs_network_state_lock:
        GCS_NETWORK_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        GCS_NETWORK_STATE_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2))


def _apply_peers(text: str, addrs: list[str]) -> str:
    """无条件按目标值重写<Peers>区块内容——addrs为空时清空到没有任何
    <Peer>子元素。2026-08-30修复：原来的实现只在addrs非空时才动这块，
    peer_addresses传空字符串不会清掉文件里已有的<Peer>，这正是这次真实
    故障复现的直接机制——"仿真模式=纯回环"这个承诺如果不清Peers就是假的，
    不可达的真机peer地址会原样留着，CycloneDDS持续对它做UDP写入失败，
    拖垮整条DDS收发管线（进程不崩溃，但服务调用全部超时）。
    """
    block = _PEERS_BLOCK_RE.search(text)
    if not block:
        if addrs:
            raise HTTPException(
                400,
                "文件里<Peers>...</Peers>这个区块当前是注释掉的状态（比如本地"
                "单机仿真测试时），不会自动取消注释——先手动编辑文件把这段"
                "取消注释，或者把peer_addresses留空跳过这一项",
            )
        return text  # 不需要peer、文件里也没有这个区块，什么都不用做
    peers_xml = "\n".join(f'        <Peer address="{a}"/>' for a in addrs)
    inner = ("\n" + peers_xml + "\n      ") if addrs else "\n      "
    return _PEERS_BLOCK_RE.sub(lambda m: m.group(1) + inner + m.group(3), text)


def _write_live_network(interface_address: str, peer_addresses_csv: str) -> None:
    if not CYCLONEDDS_GCS_PATH.exists():
        raise HTTPException(404, "cyclonedds_gcs.xml不存在，需要先从.example复制一份")
    text = CYCLONEDDS_GCS_PATH.read_text()
    if not _IFACE_RE.search(text):
        raise HTTPException(500, "没找到NetworkInterface address字段，文件格式跟预期不符，不敢自动改")
    text = _IFACE_RE.sub(lambda m: m.group(1) + interface_address + m.group(3), text)
    addrs = [a.strip() for a in re.split(r"[,\s]+", peer_addresses_csv) if a.strip()]
    text = _apply_peers(text, addrs)
    CYCLONEDDS_GCS_PATH.write_text(text)


def _classify_active_mode(iface: str) -> str | None:
    if not iface or iface in ("GCS_LOCAL_IP_PLACEHOLDER", "lo"):
        return None
    return "sim" if iface == SIM_INTERFACE_ADDRESS else "real"


def _live_ros_domain_id() -> str | None:
    result = subprocess.run(
        ["docker", "exec", GCS_CONTAINER_NAME, "printenv", "ROS_DOMAIN_ID"],
        capture_output=True, text=True, timeout=10,
    )
    return result.stdout.strip() if result.returncode == 0 else None


_GCS_ENV_VAR_RE_CACHE: dict[str, re.Pattern] = {}


def _write_env_var(env_path: Path, key: str, value: str) -> None:
    """更新（或追加）.env文件里的一行`KEY=VALUE`，保留其余行原样不动。
    2026-08-30新增：实测踩过的真实坑——GCS_ROS_DOMAIN_ID如果只当subprocess
    环境变量临时传给一次`docker compose up`、不落盘，以后任何人/任何脚本
    再手动跑一次`docker compose up -d`（不经过这条切换接口，比如改了别的
    卷挂载想重建一下backend）,compose会读回.env里的旧默认值，把
    ROS_DOMAIN_ID悄悄改回去，导致跟cyclonedds_gcs.xml里记录的模式对不上
    （interface说是sim，域却是真机的20）——这个函数把值真正落盘，保证
    "谁来重建都读到同一份、真实生效的值"。
    """
    text = env_path.read_text() if env_path.exists() else ""
    pattern = _GCS_ENV_VAR_RE_CACHE.setdefault(key, re.compile(rf'^{re.escape(key)}=.*$', re.M))
    if pattern.search(text):
        text = pattern.sub(f"{key}={value}", text, count=1)
    else:
        if text and not text.endswith("\n"):
            text += "\n"
        text += f"{key}={value}\n"
    env_path.write_text(text)


def _recreate_gcs_container(ros_domain_id: str) -> None:
    """ROS_DOMAIN_ID是容器创建时读的环境变量，普通`docker restart`不会
    应用新值，必须走`docker compose up -d --force-recreate`重建。先把
    目标值持久化写进gcs/.env（_write_env_var，保证以后任何方式的重建都读
    到一致的值），再以subprocess环境变量的形式同步传给这次`docker compose
    up`调用本身（双保险，不依赖文件写入和compose读取之间没有时序问题）。
    跟start_sim_fleet()同款`docker compose --project-directory <repo根>
    up -d`的做法一致——ROS_DOMAIN_ID/网络配置这类"创建时才生效"的参数，
    `docker start`/`docker restart`都做不到，只有重新`up`（隐式recreate）
    才行，这个坑本文件175-188行已经踩过一次。
    """
    if not HOST_REPO_PATH:
        raise HTTPException(500, "HOST_REPO_PATH未配置，见gcs/.env里的说明，无法重建GCS容器")
    gcs_env_path = Path(f"{HOST_REPO_PATH}/gcs/.env")
    try:
        _write_env_var(gcs_env_path, "GCS_ROS_DOMAIN_ID", ros_domain_id)
    except OSError as e:
        raise HTTPException(500, f"写入gcs/.env的GCS_ROS_DOMAIN_ID失败: {e}")
    # 2026-08-30新增：记下这次重建之前的时间戳，后面查崩溃日志时用--since
    # 过滤——docker logs默认会把重建前的旧日志也吐出来，不加这个过滤，直接
    # grep会把上一轮的崩溃traceback误判成这一轮的，跟旧up_gcs_hw.sh脚本里
    # 同款检查踩过的坑一样。
    recreate_ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")
    env = {**os.environ, "GCS_ROS_DOMAIN_ID": ros_domain_id}
    result = subprocess.run(
        ["docker", "compose", "--project-directory", f"{HOST_REPO_PATH}/gcs",
         "up", "-d", "--force-recreate", "gcs"],
        capture_output=True, text=True, timeout=60, env=env,
    )
    if result.returncode != 0:
        raise HTTPException(500, f"配置已写入，但重建{GCS_CONTAINER_NAME}容器失败: {result.stderr}")

    # `docker compose up -d`返回码为0只代表"容器在它检查的那一刻处于运行
    # 状态"，不代表容器里的rosbridge/rosapi进程留住了——这个容器的entrypoint
    # 是"后台起ros2 launch + sleep infinity占住PID1"这种写法（保证rosbridge
    # 挂了容器不会跟着退出，让前端网页能打开、按钮还能点着修），代价是
    # `docker inspect`看到的容器状态和`restart: "no"`这类机制对"里面的
    # ROS2进程已经崩溃"完全不敏感——实测确认过：NetworkInterface填了这台
    # 机器当下并不存在的IP（比如换了网络环境、DHCP重新分配了地址）时，
    # rosbridge_websocket/rosapi_node会直接崩溃退出并打印"process has
    # died"，但容器本身照样显示running。改成跟旧up_gcs_hw.sh脚本同款、
    # 已经验证有效的检测方式——查这次重建之后的日志里有没有这行文本，
    # 不看容器状态。不主动检查的话，这里会直接返回"success"，用户要等
    # 前端`waitForRosbridgeReady`那25秒超时才会看到一条不具体的"rosbridge
    # 没能恢复正常"提示，还得自己去看`docker logs`才知道真正原因；这里
    # 提前给出具体原因（比如"XXX: does not match an available interface"）。
    time.sleep(2)
    logs = subprocess.run(
        ["docker", "logs", "--since", recreate_ts, GCS_CONTAINER_NAME],
        capture_output=True, text=True, timeout=10,
    ).stdout
    if "process has died" in logs:
        raise HTTPException(
            500,
            f"配置已写入、容器已重建，但rosbridge/rosapi进程崩溃了（容器本身还在跑，"
            f"只是里面的ROS2进程死了）。常见原因是NetworkInterface填的IP在这台机器上"
            f"不存在，用`ip a`核对。最近日志：\n{logs[-1500:]}",
        )


@app.get("/api/gcs-network/mode")
def get_gcs_network_mode():
    """现读活文件+实际运行容器的ROS_DOMAIN_ID，不引入单独的"最后一次
    切换记录"旗标——旗标如果跟文件/容器实际状态脱节（比如有人绕过网页
    直接SSH改了cyclonedds_gcs.xml，这正是这次真实故障复现现场的来源），
    旗标会说谎，现读保证"网页看到的就是容器实际在用的"。文件不存在时
    返回200+active_mode:null，不抛异常——这是页面加载时无条件会调的
    接口，不该在全新环境上给用户吓人的红色报错。
    """
    if not CYCLONEDDS_GCS_PATH.exists():
        return {"active_mode": None, "reason": "cyclonedds_gcs.xml不存在，需要先从.example复制一份"}
    text = CYCLONEDDS_GCS_PATH.read_text()
    iface = _IFACE_RE.search(text)
    iface_val = iface.group(2) if iface else ""
    return {
        "active_mode": _classify_active_mode(iface_val),
        "live_interface_address": iface_val,
        "ros_domain_id": _live_ros_domain_id(),
    }


@app.post("/api/gcs-network/mode")
def set_gcs_network_mode(req: GcsModeReq):
    if req.mode not in ("sim", "real"):
        raise HTTPException(400, "mode只能是sim或real")
    if not CYCLONEDDS_GCS_PATH.exists():
        raise HTTPException(404, "cyclonedds_gcs.xml不存在，需要先从.example复制一份")

    if req.mode == "sim":
        target_iface, target_peers, target_domain = SIM_INTERFACE_ADDRESS, "", SIM_ROS_DOMAIN_ID
    else:
        real_cfg = _load_gcs_network_state().get("real") or {}
        if not real_cfg.get("interface_address"):
            raise HTTPException(
                400,
                "还没配置过真机网络参数——先展开下面'GCS网络设置'面板，"
                "填写本机网卡IP/Jetson对端IP并保存，再点这个按钮",
            )
        target_iface = real_cfg["interface_address"]
        target_peers = real_cfg.get("peer_addresses", "")
        target_domain = REAL_ROS_DOMAIN_ID

    # 2026-08-31新增：切到真机模式时顺带停掉本机仿真三容器——真实故障
    # 案例：本机仿真的flight-stack-nx01容器只要还在跑，前端nsSource()
    # 就会把命名空间撞车的真机NX01误判成"sim"来源(`SIM_NS_CONTAINERS`
    # 按容器运行状态判断，不看ROS_DOMAIN_ID)，导致hasTelemetry恒为
    # false、真机的位置/姿态/速度等遥测表格全部不渲染，只剩不受这个
    # 判断影响的"容器状态"和"融合模式"两块。只对`real`方向做，`sim`
    # 方向不做对称的"停真机flight-stack"——真机是物理飞机，网页切一下
    # 模式就远程停飞控相关进程风险太大，跟停本机仿真容器完全不是一个
    # 量级，不能类比。放在上面真机网络参数校验之后（校验失败就不该有
    # 任何副作用）、下面短路检查之前、无条件执行——即使这次请求会被
    # "已经是目标模式"短路掉网络重建，也可能是"域已经是真机、但仿真
    # 容器是切换之后才被人手动重新起来"这种场景，一样需要停。
    sim_stop_errors = _stop_sim_fleet_containers() if req.mode == "real" else {}
    sim_stop_suffix = (
        f"；本机仿真容器停止失败: {sim_stop_errors}" if sim_stop_errors
        else "；已顺带停止本机仿真三容器" if req.mode == "real" else ""
    )

    text = CYCLONEDDS_GCS_PATH.read_text()
    iface_match = _IFACE_RE.search(text)
    current_iface = iface_match.group(2) if iface_match else ""
    # 同时比对接口地址和ROS_DOMAIN_ID两者都已经是目标值才短路跳过重建——
    # 只看接口地址不够：如果上一次切换是"文件写成功、但docker compose
    # recreate失败"这种半成功状态，接口地址已经是目标值但域还没切过来，
    # 只看接口地址会误判"已经是目标模式"从而放弃重试，域永远修不好。
    if current_iface == target_iface and _live_ros_domain_id() == target_domain:
        return {"success": True, "mode": req.mode, "restarted": False,
                "message": f"当前已经是{req.mode}模式，未重复重建{sim_stop_suffix}"}

    _write_live_network(target_iface, target_peers)
    _recreate_gcs_container(target_domain)
    label = "仿真" if req.mode == "sim" else "真机"
    return {"success": True, "mode": req.mode, "restarted": True,
            "message": f"已切换到{label}模式（{target_iface}，ROS_DOMAIN_ID={target_domain}），"
                       f"GCS容器已重建，等待rosbridge重新可用{sim_stop_suffix}"}


@app.get("/api/gcs-network")
def get_gcs_network():
    """返回持久化的真机快照，不是活文件——否则仿真模式生效期间这个面板
    会显示127.0.0.1，把用户之前保存的真机IP盖住/骗用户以为丢了。
    """
    real_cfg = _load_gcs_network_state().get("real") or {}
    return {
        "interface_address": real_cfg.get("interface_address") or "",
        "peer_addresses": real_cfg.get("peer_addresses") or "",
        "configured": bool(real_cfg.get("interface_address")),
    }


@app.post("/api/gcs-network")
def set_gcs_network(cfg: GcsNetworkConfig):
    """永远先落盘真机快照；只有当前确实是真机模式时才顺带实时应用+
    重建，否则只保存、不碰活文件/不重启——避免在仿真模式下保存真机参数
    时意外把正在用的仿真配置冲掉。
    """
    if not cfg.interface_address.strip():
        raise HTTPException(400, "网卡IP不能为空")
    state = _load_gcs_network_state()
    state["real"] = {
        "interface_address": cfg.interface_address.strip(),
        "peer_addresses": cfg.peer_addresses.strip(),
    }
    _save_gcs_network_state(state)

    text = CYCLONEDDS_GCS_PATH.read_text() if CYCLONEDDS_GCS_PATH.exists() else ""
    iface_match = _IFACE_RE.search(text) if text else None
    current_mode = _classify_active_mode(iface_match.group(2) if iface_match else "")
    if current_mode != "real":
        return {"success": True, "applied_live": False,
                "message": "已保存真机网络参数（当前不是真机模式，未立即生效——"
                           "点击顶部'🛰️真机模式'按钮时会应用并重建GCS容器）"}
    _write_live_network(cfg.interface_address.strip(), cfg.peer_addresses.strip())
    _recreate_gcs_container(REAL_ROS_DOMAIN_ID)
    return {"success": True, "applied_live": True, "message": f"已写入配置并重建{GCS_CONTAINER_NAME}容器"}


# 2026-08-25：原来这里是"2. 真机flight-stack规划/避障参数白名单"整节
# （SSH到Jetson改.env的V_MAX/A_MAX等参数白名单+/api/hw-params/{ns}
# GET/POST），用户要求彻底删除前端"⚙规划/避障调参"入口，这节代码除了
# 给那个入口用没有别的调用方，一并删掉了（HW_PARAM_WHITELIST/HwTarget/
# HW_TARGETS/_ssh_run/_parse_env/_get_target/get_hw_params/
# set_hw_params）。真机的启停/检测模式/改名这些操作现在完全走第7节
# UDP发现+HTTP的control_server.py通道，不再有任何功能依赖SSH到Jetson，
# 这个后端也不再需要免密登录Jetson的能力——`gcs/docker-compose.yml`里
# 挂载`~/.ssh`那条卷线如果以后确认真的没别的地方用，也可以一起清理，
# 这次没有动docker-compose.yml/Dockerfile（怕误伤该服务其它未来可能
# 恢复的用途），只删了纯Python代码层面已确认无引用的部分。

# ============================================================
# 3. 机队启停控制（方案文档阶段6）
# ============================================================

# 仿真三个容器的名字——docker compose默认项目名取自目录名(docker_sim)，
# 服务名(sim-world/flight-stack-nx01/flight-stack-nx02)+序号1，跟
# GCS_CONTAINER_NAME(gcs-gcs-1)同样的命名规律。用环境变量兜底可覆盖，
# 万一以后改了目录名/compose项目名不用改代码。
SIM_FLEET_CONTAINERS = os.environ.get(
    "SIM_FLEET_CONTAINERS",
    "docker_sim-sim-world-1,docker_sim-flight-stack-nx01-1,docker_sim-flight-stack-nx02-1",
).split(",")

# sim-world在前（flight-stack靠它的Gazebo/PX4/点云话题），启动时先起
# 它，停止时最后再停它，是start.sh里同样的依赖顺序考虑；停止仍然用
# docker stop（见下面stop_sim_fleet），不需要改。
#
# 2026-08-21更新：启动这一步（start_sim_fleet）从纯`docker start`改成
# `docker compose up -d`——原来这里的注释解释过为什么不用compose（避免
# "容器B内部跑docker compose、但实际由宿主机dockerd创建容器"这套
# sibling-container场景的路径挂载复杂度），但那是在"只需要恢复已经建好
# 的容器"这个前提下成立的判断。用户现在明确要求能在网页上先选
# LOCALIZATION_SOURCE/PLANNER/CONTROLLER再启动仿真——这几个值是在容器
# *创建*时通过环境变量烤进去的，`docker start`不会重新读取新值，只有
# 重新`up`（必要时隐式recreate）才能应用网页上新选的参数，绕不开
# compose。之前顾虑的sibling-container路径问题这次正面解决：
# `gcs/backend/Dockerfile`装了docker-compose-plugin，
# `gcs/docker-compose.yml`把宿主机仓库根目录（`HOST_REPO_PATH`环境变量，
# 见该文件里的说明）以*相同绝对路径*只读挂进这个容器——`docker compose
# --project-directory ${HOST_REPO_PATH} up -d`算出来的卷路径因此跟
# 宿主机dockerd实际认识的路径完全对齐，不会错位。
HOST_REPO_PATH = os.environ.get("HOST_REPO_PATH", "")

# 只接受这几个组合里出现过的取值（跟README.md"18种正交组合"表格一致，
# single_*那三个单机专用值不在这里——这个接口面向的是SIM_FLEET_CONTAINERS
# 固定的双机全量启动，不是`./start.sh sim-world flight-stack-nx01`那种
# 挑选子集的单机彩排场景），拒绝任何不在白名单里的值，不盲目把前端传来
# 的字符串直接塞进子进程环境变量。
# 2026-09-07新增slam_backend——此前这个面板只有3个字段，跟真机集群面板
# （"🛰️真机集群"，见ensureHwFleetStackContract()/stack_choices，"融合模式/
# 定位器/规划器/控制器"四列表格）的4字段设计不一致，落后了。SLAM_BACKEND
# 只在localization_source=uwb_slam时真正生效（entrypoint.sh里gt/uwb_imu
# 两个分支完全不读这个变量），但这里不做"选了gt就隐藏/禁用这个下拉框"
# 这类联动——真机集群面板的四个字段也是平铺展示、不做互斥禁用，这里保持
# 同一个交互模式，选了不生效的组合无害（entrypoint.sh自己会忽略，不会
# 误起进程）。point_lio/fast_lio这两个可选后端都只做过patch可应用性/
# colcon build层面的验证，还没有实际跑过仿真起飞，仍然默认dlio。
SIM_PARAM_CHOICES = {
    "localization_source": ["gt", "uwb_slam", "uwb_imu"],
    "slam_backend": ["dlio", "point_lio", "fast_lio"],
    "planner": ["mighty", "ego_planner"],
    "controller": ["px4ctrl", "pt4ctrl", "so3ctrl"],
}


# 2026-09-07新增：单机仿真彩排（README"单机仿真彩排"一节，Jetson Orin
# NX单机真机移植前验证用）——只起sim-world+flight-stack-nx01，
# LOCALIZATION_SOURCE必须是这三个single_*专用值之一（entrypoint.sh里
# 这三个值会强制校验NUM_AGENTS==1，不等于1直接报错退出，防止手滑用双机
# 场景的普通值+单机数量这种不一致组合），不能沿用SIM_PARAM_CHOICES里
# 双机场景的gt/uwb_slam/uwb_imu三个值。
SIM_SINGLE_AGENT_LOCALIZATION_CHOICES = ["single_slam_only", "single_uwb_imu", "single_uwb_slam"]


class SimFleetStartParams(BaseModel):
    localization_source: str | None = None
    slam_backend: str | None = None
    planner: str | None = None
    controller: str | None = None
    single_agent: bool = False  # true=单机彩排模式，只起sim-world+flight-stack-nx01


def _docker_status(name: str) -> str:
    result = subprocess.run(
        ["docker", "inspect", "--format", "{{.State.Status}}", name],
        capture_output=True, text=True, timeout=10,
    )
    if result.returncode != 0:
        return "not_found"
    return result.stdout.strip()


@app.get("/api/fleet/sim/status")
def get_sim_fleet_status():
    return {name: _docker_status(name) for name in SIM_FLEET_CONTAINERS}


def _query_sim_stack_params(container: str) -> dict:
    """从指定flight-stack容器现查*实际*生效的四个模式环境变量——不是读
    docker-compose.yml文件本身（文件里的默认值可能跟上次实际传参启动时用
    的值不一样）。容器不存在/没在跑时返回running=False+四个字段全None，
    前端据此判断"这架飞机现在到底是什么模式"还是"查不到、不代表任何值"，
    不用docker-compose.yml的默认值瞎猜——因为NX01/NX02两个容器理论上
    应该由同一次`docker compose up`用同一组环境变量启动、值应该一致，
    但"应该一致"不代表"一定一致"（比如只重建了一个容器），分别查而不是
    只查NX01再假设NX02一样，能在两机意外不一致时如实暴露出来。
    """
    if _docker_status(container) == "running":
        result = _docker(
            ["exec", container, "printenv", "LOCALIZATION_SOURCE", "SLAM_BACKEND", "PLANNER", "CONTROLLER"],
            timeout=8,
        )
        lines = result.stdout.splitlines()
        if result.returncode == 0 and len(lines) >= 4:
            return {
                "running": True,
                "localization_source": lines[0].strip() or None,
                "slam_backend": lines[1].strip() or None,
                "planner": lines[2].strip() or None,
                "controller": lines[3].strip() or None,
            }
    return {
        "running": False,
        "localization_source": None,
        "slam_backend": None,
        "planner": None,
        "controller": None,
    }


@app.get("/api/fleet/sim/params")
def get_sim_fleet_params():
    # 沿用NX01代表整个仿真机队——"仿真工具"面板启动/重建时两机是同一次
    # `docker compose up`用同一组环境变量创建的，这个接口只是给那个面板
    # 的表单回填默认选中值用，不是逐机诊断，查一台就够。容器不存在/没在
    # 跑时返回docker-compose.yml的默认值兜底（不是None——这里前端要用来
    # 回填<select>的默认选中项，None会导致下拉框没有任何选项被选中）。
    # 2026-09-07：这几个兜底值改成跟docker-compose.yml当前默认值
    # （uwb_slam/fast_lio/ego_planner/px4ctrl）保持一致——之前是gt/dlio，
    # docker-compose.yml的默认值已经改了，这里没跟着改的话，容器没起来时
    # 面板显示的"默认选中项"会跟真正`docker compose up`不传参时实际生效
    # 的值对不上。
    result = _query_sim_stack_params("docker_sim-flight-stack-nx01-1")
    if result["running"]:
        return {
            "running": True,
            "localization_source": result["localization_source"] or "uwb_slam",
            "slam_backend": result["slam_backend"] or "fast_lio",
            "planner": result["planner"] or "ego_planner",
            "controller": result["controller"] or "px4ctrl",
        }
    return {
        "running": False,
        "localization_source": "uwb_slam",
        "slam_backend": "fast_lio",
        "planner": "ego_planner",
        "controller": "px4ctrl",
    }


# 2026-09-07新增：GCS网页"仿真模式"下每架飞机卡片要模拟真机集群卡片的
# "飞行栈模式"只读展示（融合模式/定位器/规划器/控制器四列，见
# hwFleetStackSummaryHtml()），跟上面/api/fleet/sim/params的区别是——那个
# 接口只查NX01代表整个机队（给"仿真工具"面板的表单回填用），这个接口
# 按namespace分别查，每张飞机卡片各自展示自己容器里真实生效的值，不假设
# 两机一定一致。仿真侧的模式是"两机共用同一份docker-compose.yml环境变量"，
# 不能像真机control_server.py那样对单机单独下发新模式——这个接口只读，
# 没有对应的POST，要改模式仍然只能去"仿真工具"面板整体重建。
@app.get("/api/fleet/sim/params/{ns}")
def get_sim_fleet_params_for_ns(ns: str):
    container = f"docker_sim-flight-stack-{ns.lower()}-1"
    if container not in SIM_FLEET_CONTAINERS:
        raise HTTPException(404, f"{ns} 不是仿真机队里的命名空间")
    return _query_sim_stack_params(container)


@app.post("/api/fleet/sim/start")
def start_sim_fleet(params: SimFleetStartParams = SimFleetStartParams()):
    if not HOST_REPO_PATH:
        raise HTTPException(500, "HOST_REPO_PATH未配置，见gcs/docker-compose.yml里这个变量的说明")
    # single_agent模式下localization_source的合法取值表整个换成单机
    # 专用的三个single_*值，其余三个字段（slam_backend/planner/
    # controller）校验规则不受影响，跟双机模式共用同一份白名单。
    localization_choices = (
        SIM_SINGLE_AGENT_LOCALIZATION_CHOICES if params.single_agent
        else SIM_PARAM_CHOICES["localization_source"]
    )
    for field, choices in SIM_PARAM_CHOICES.items():
        if field == "localization_source":
            choices = localization_choices
        value = getattr(params, field)
        if value is not None and value not in choices:
            raise HTTPException(400, f"{field}={value} 不在允许的取值范围 {choices} 内")

    # 只覆盖用户实际选了的字段——没选的字段不传给docker compose，让
    # docker-compose.yml自己的`${VAR:-default}`兜底生效，不强行帮用户
    # 决定"没选就是什么"。
    compose_env = dict(os.environ)
    if params.localization_source:
        compose_env["LOCALIZATION_SOURCE"] = params.localization_source
    if params.slam_backend:
        compose_env["SLAM_BACKEND"] = params.slam_backend
    if params.planner:
        compose_env["PLANNER"] = params.planner
    if params.controller:
        compose_env["CONTROLLER"] = params.controller

    compose_cmd = ["docker", "compose", "--project-directory", HOST_REPO_PATH,
                   "-f", f"{HOST_REPO_PATH}/docker-compose.yml", "up", "-d"]
    if params.single_agent:
        # entrypoint.sh里single_*三个值强制要求NUM_AGENTS==1，不等于1
        # 直接报错退出；docker-compose.yml的NUM_AGENTS默认值是2，这里必须
        # 显式覆盖成1，不能让用户自己再多传一个参数才能启动成功。
        # localization_source没选时不能沿用docker-compose.yml的默认值
        # （现在是uwb_slam，双机场景值，配unrecognized上NUM_AGENTS=1会被
        # entrypoint拒绝），给一个单机场景自己的默认值。
        compose_env["NUM_AGENTS"] = "1"
        if not params.localization_source:
            compose_env["LOCALIZATION_SOURCE"] = "single_slam_only"
        # 只起sim-world+flight-stack-nx01，跟README"单机仿真彩排"一节的
        # `docker compose up sim-world flight-stack-nx01`用法一致。先显式
        # 停掉flight-stack-nx02——如果之前是双机模式在跑，`docker compose
        # up -d sim-world flight-stack-nx01`只会创建/启动这两个服务，不会
        # 碰flight-stack-nx02的当前状态，遗留的nx02容器不停掉就不是真正的
        # "单机"。`docker stop`对已经停着/不存在的容器是no-op，无条件调用
        # 是安全的，跟_stop_sim_fleet_containers()同一个假设。
        subprocess.run(["docker", "stop", "docker_sim-flight-stack-nx02-1"],
                        capture_output=True, text=True, timeout=30)
        compose_cmd += ["sim-world", "flight-stack-nx01"]

    result = subprocess.run(compose_cmd, capture_output=True, text=True, timeout=180, env=compose_env)
    if result.returncode != 0:
        raise HTTPException(500, f"docker compose up -d失败: {(result.stdout + result.stderr).strip()}")
    chosen = {k: (getattr(params, k) or "(沿用当前默认值)") for k in SIM_PARAM_CHOICES}
    chosen["single_agent"] = params.single_agent
    mode_note = "单机彩排(仅NX01)" if params.single_agent else "双机(NX01+NX02)"
    return {"success": True, "message": f"已用参数重建并启动仿真机队[{mode_note}]: {chosen}"}


def _stop_sim_fleet_containers() -> dict:
    """`docker stop`本身对已经停着/不存在的容器是no-op（返回0），可以放心
    无条件调用，不需要先查状态。被`stop_sim_fleet()`和`set_gcs_network_mode()`
    （切到真机模式时顺带调用，见后者的说明）共用。
    """
    errors = {}
    for name in reversed(SIM_FLEET_CONTAINERS):
        result = subprocess.run(["docker", "stop", name], capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            errors[name] = result.stderr.strip() or result.stdout.strip()
    return errors


@app.post("/api/fleet/sim/stop")
def stop_sim_fleet():
    errors = _stop_sim_fleet_containers()
    if errors:
        raise HTTPException(500, f"部分容器停止失败: {errors}")
    return {"success": True, "message": "已停止sim-world/flight-stack-nx01/flight-stack-nx02"}


# 2026-08-25删除：/api/fleet/hw/{ns}/status|start|stop（SSH到Jetson做
# docker-compose.hw.yml up/down的"远程真机"机队启停面板）——前端"📡远程
# 真机"面板已经删除，功能被"🛰️真机集群"面板（UDP发现+HTTP直连
# control_server.py，见下面第7节）取代，不再依赖SSH。同一天稍后"⚙规划/
# 避障调参"(/api/hw-params/{ns})也被删了，见前面原"2. 真机flight-stack
# 规划/避障参数白名单"那节的说明——这个后端现在不再有任何功能依赖SSH
# 到Jetson。

# GCS_BACKEND_CONTAINER_NAME跟GCS_CONTAINER_NAME(104行)同样的命名规律，
# 都是"改了gcs/目录名或compose项目名要跟着改"这个已知限制，见那边的注释。
GCS_BACKEND_CONTAINER_NAME = os.environ.get("GCS_BACKEND_CONTAINER_NAME", "gcs-backend-1")


@app.post("/api/gcs/stop")
def stop_gcs():
    # 关闭GCS自己（gcs-gcs-1的rosbridge+前端、gcs-backend-1这个后端自己）
    # ——没有对应的"GET状态"/"start"接口：一旦两个容器都停了，就不会再有
    # 任何进程能响应这个网页发来的HTTP请求，"网页上一个按钮把网页自己
    # 关掉"这件事本身没法做成"关完还能在同一个网页上再点一次开"，重新
    # 拉起只能回到宿主机终端手动`docker start`或重新跑
    # scripts/up_gcs.sh，这是这个功能天然的边界，不是没做全。
    #
    # 自己停自己是这个接口里最需要注意的一点：如果在这个请求处理过程中
    # 直接同步`docker stop gcs-backend-1`，dockerd会给这个容器的PID1
    # (uvicorn主进程)发SIGTERM，触发uvicorn自己的优雅关闭流程，跟"这个
    # HTTP请求还没处理完、响应还没发回浏览器"这两件事会抢时间——实测这
    # 类"进程自己叫停自己所在的容器"场景，最简单可靠的做法是**先返回
    # 响应，用一个不受这次请求生命周期管辖的后台线程延迟一小段时间再
    # 真的执行docker stop**，保证浏览器先稳稳收到"已发送关闭指令"这个
    # 确认，不用去赌response有没有在容器死掉前flush出去。
    def _delayed_stop():
        time.sleep(1.5)
        # 先停gcs-gcs-1（前端+rosbridge，不是自己，没有自我引用的race），
        # 再停gcs-backend-1自己——自己放最后，此时已经不需要再用这个
        # 进程做任何事，没有下游依赖它继续活着。
        subprocess.run(["docker", "stop", GCS_CONTAINER_NAME], capture_output=True, timeout=30)
        subprocess.run(["docker", "stop", GCS_BACKEND_CONTAINER_NAME], capture_output=True, timeout=30)

    threading.Thread(target=_delayed_stop, daemon=True).start()
    return {
        "success": True,
        "message": (
            f"已发送关闭指令，{GCS_CONTAINER_NAME}（前端+rosbridge）和"
            f"{GCS_BACKEND_CONTAINER_NAME}（这个后端自己）几秒后会停止，"
            "网页会失去响应——需要在宿主机终端手动docker start两个容器，"
            "或重新跑 scripts/up_gcs.sh 才能恢复"
        ),
    }


# ============================================================
# 4. 起飞（含自标定动作）——移植自scripts/launch_control.py
# ============================================================
# 起飞后自动拉起的自标定脚本(scripts/auto_calibration_flight.py)必须靠
# docker exec在flight-stack容器里跑（局部坐标系里飞一段"前进1.1米再退回"
# 给origin_setter_node的θ*在线估计攒样本），浏览器/rosbridge做不到，
# 只能走这个后端——之前gcs/frontend/index.html的"起飞"按钮直接对rosbridge
# 发TakeoffLand话题，这次顺带按launch_control.py的do_takeoff()同款逻辑
# 移植过来，不是单纯加一个标定动作：原来那个按钮只覆盖了
# REPEATABLE_CONTROLLERS（px4ctrl/so3ctrl/pt4ctrl）这一条分支，
# CONTROLLER=ros2_px4_stack时发TakeoffLand话题没有任何节点订阅，那个按钮
# 实际上什么都不会发生，只是没人拿这个组合测过没暴露——这次一并按容器里
# 实际的CONTROLLER环境变量分流，行为跟launch_control.py完全对齐。
#
# ⚠️只支持仿真场景（本机docker exec）。真机场景要让Jetson上的flight-stack-hw
# 容器跑同样的脚本，这个后端得先把两个脚本SCP到Jetson（docker cp不行——
# 这个后端跟Jetson没有共享docker daemon），而且这个后端现在已经不再有
# 任何SSH到Jetson的能力（2026-08-25那次SSH相关代码全部删完了，见前面
# 的说明），要支持真机起飞得先重新搭一条通路，暂不支持，调用会直接
# 404，不做一半就上线。

REPEATABLE_CONTROLLERS = ("px4ctrl", "so3ctrl", "pt4ctrl")
TAKEOFF_GATE_FILE = "/tmp/takeoff_go"
SCRIPTS_DIR = Path("/app/scripts")

SIM_NS_CONTAINERS = {
    "NX01": "docker_sim-flight-stack-nx01-1",
    "NX02": "docker_sim-flight-stack-nx02-1",
}


def _get_sim_container(ns: str) -> str:
    container = SIM_NS_CONTAINERS.get(ns.upper())
    if not container:
        raise HTTPException(404, f"未知命名空间 {ns}，只支持 {sorted(SIM_NS_CONTAINERS)}（仅限仿真场景）")
    return container


def _docker(args, timeout=15):
    return subprocess.run(["docker"] + args, capture_output=True, text=True, timeout=timeout)


def _get_controller(container: str) -> str:
    result = _docker(["exec", container, "printenv", "CONTROLLER"], timeout=8)
    out = result.stdout.strip()
    # 兜底跟launch_control.py的get_controller()一致：printenv失败/取不到值
    # 时按当前docker-compose.yml的默认值处理，不是瞎猜。
    return out if (result.returncode == 0 and out) else "px4ctrl"


def _cp_helper_scripts(container: str) -> None:
    for name in ("ros2_env_setup.sh", "auto_calibration_flight.py"):
        result = _docker(["cp", str(SCRIPTS_DIR / name), f"{container}:/tmp/{name}"], timeout=10)
        if result.returncode != 0:
            raise HTTPException(500, f"docker cp {name} 到 {container} 失败: {result.stderr.strip()}")


def _publish_takeoff_land(container: str, ns: str, cmd_value: int) -> None:
    # 连发3次而不是一次——ros2 topic pub --once在DDS discovery没跟上时可能
    # 白发，照抄launch_control.py/takeoff_gate.py的防御写法。
    shell = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/px4ctrl_ws/install/setup.bash && "
        "source /tmp/ros2_env_setup.sh && "
        f"for i in 1 2 3; do ros2 topic pub --once /{ns}/takeoff_land "
        f"quadrotor_msgs/msg/TakeoffLand \"{{takeoff_land_cmd: {cmd_value}}}\" "
        "> /dev/null 2>&1; sleep 0.2; done"
    )
    result = _docker(["exec", container, "bash", "-c", shell], timeout=15)
    if result.returncode != 0:
        raise HTTPException(500, f"发布takeoff_land话题失败: {(result.stdout + result.stderr).strip()}")


def _recent_px4ctrl_feedback(container: str, controller: str, since_ts: int) -> str:
    # 起飞指令发出去之后，飞控（px4ctrl/so3ctrl/pt4ctrl三个控制器各自跑同一套
    # PX4CtrlFSM状态机的独立拷贝，ROS节点名各自叫"px4ctrl"/"so3ctrl"/"pt4ctrl"，
    # 不是共用一个节点名——查过三份PX4CtrlFSM.cpp源码，每份的RCLCPP_ERROR/INFO
    # 里硬编码的前缀跟节点名一一对应）到底有没有真的响应——是进了AUTO_TAKEOFF，
    # 还是被FSM按MANUAL_CTRL分支那一串前置条件（没收到odom/odom速度不是静止/
    # land detector认为没落地/RC没居中等）静默拒绝——这些判断结果只用
    # RCLCPP_INFO/ERROR打印到节点自己的ROS日志，从来没有发布成任何话题，网页端
    # 原来完全看不到。entrypoint里所有节点都是`&`后台起、继承容器stdout，没有
    # 重定向到独立文件，唯一能拿到这份反馈的办法是读`docker logs`（docker-
    # compose.yml里三个飞行相关服务都配的是json-file日志驱动，支持`docker logs`）。
    # 过滤条件必须按controller这个变量来（等于节点名），不能写死"px4ctrl"——
    # 写死会导致so3ctrl/pt4ctrl这两种控制器的日志一行都过滤不出来，误报成
    # "没有明确响应"。
    result = _docker(["logs", "--since", str(since_ts), container], timeout=8)
    if result.returncode != 0:
        return ""
    keywords = ("TAKEOFF", "Reject", "MANUAL_CTRL", "AUTO_HOVER")
    keep = [
        line for line in (result.stdout + result.stderr).splitlines()
        if controller in line and any(k in line for k in keywords)
    ]
    return "\n".join(keep[-10:])


def _start_auto_calibration(container: str, ns: str) -> None:
    shell = (
        "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && "
        f"python3 /tmp/auto_calibration_flight.py {ns} "
        ">> /tmp/auto_calibration_flight_stdout.log 2>&1"
    )
    # -d后台起，不阻塞这个HTTP请求的返回——跟launch_control.py同一个模式，
    # 全程约十几秒，飞机自己动，前端不需要等它跑完。
    result = _docker(["exec", "-d", container, "bash", "-c", shell], timeout=8)
    if result.returncode != 0:
        raise HTTPException(500, f"自标定动作启动失败: {(result.stdout + result.stderr).strip()}")


@app.post("/api/sim/{ns}/takeoff")
def sim_takeoff(ns: str):
    ns = ns.upper()
    container = _get_sim_container(ns)
    if _docker_status(container) != "running":
        raise HTTPException(409, f"{container} 不在running状态，先确认仿真机队已启动")

    controller = _get_controller(container)
    _cp_helper_scripts(container)

    since_ts = int(time.time())
    if controller in REPEATABLE_CONTROLLERS:
        _publish_takeoff_land(container, ns, 1)
        takeoff_note = f"CONTROLLER={controller}，已发布takeoff_land话题（连发3次）"
    else:
        result = _docker(["exec", container, "touch", TAKEOFF_GATE_FILE], timeout=8)
        if result.returncode != 0:
            raise HTTPException(500, f"touch起飞门失败: {(result.stdout + result.stderr).strip()}")
        takeoff_note = (f"CONTROLLER={controller}，已touch {TAKEOFF_GATE_FILE}"
                         "（这条链路只支持起飞一次，降落后不能再起飞）")

    # 指令发出去只代表docker exec/ros2 topic pub这些shell命令本身没报错，
    # 不代表飞控真的接受了——PX4CtrlFSM可能因为没收到odom、odom速度不是
    # 静止、land detector认为没落地等原因静默拒绝AUTO_TAKEOFF（只打印
    # RCLCPP_ERROR到自己的ROS日志），之前这里从来没检查过，网页上看到
    # "起飞指令已发送"跟飞机真的会不会飞没有任何关系。等1.5秒给FSM一个
    # process()周期把判断结果打进日志，再回读，把接受/拒绝的原始日志行
    # 直接带回前端，而不是想办法用ROS服务/话题重新实现一遍FSM内部逻辑。
    time.sleep(1.5)
    px4ctrl_feedback = _recent_px4ctrl_feedback(container, controller, since_ts)

    _start_auto_calibration(container, ns)
    message = f"{takeoff_note}；自标定动作已在后台启动（悬停→前进1.1米→退回起飞点，约十几秒）"
    if px4ctrl_feedback:
        message += f"\n飞控反馈：\n{px4ctrl_feedback}"
    else:
        message += "（未在日志里看到px4ctrl对这次指令的明确响应，CONTROLLER=ros2_px4_stack时这套日志本来就不适用）"
    return {"success": True, "message": message}


# ============================================================
# 5. 仿真诊断信息——环境变量/清空记录/资源消耗（移植自
#    scripts/status_monitor.py + scripts/launch_control.py的do_clear_logs()）
# ============================================================
# 三件事浏览器都做不到：①读容器里的PLANNER/CONTROLLER环境变量（前端要用
# 它判断"规划器心跳"/"板外控制器"这两项健康监控在当前配置下是否有意义，
# 逻辑跟status_monitor.py的_compute_health()一致）；②以root身份删
# runtime_logs/下的文件（宿主机侧非root用户对这些文件没有写权限，见
# launch_control.py do_clear_logs()的说明）；③读宿主机层面的docker stats/
# nvidia-smi（容器间因PID/cgroup namespace隔离互相看不到，只能在能看到
# 宿主机全貌的地方查——这个后端本来就已经挂了docker.sock，直接查一次，
# 不用像status_monitor.py那样依赖一个常驻的collect_container_stats.sh
# 宿主机脚本，用户不用记得手动起它）。


@app.get("/api/sim/{ns}/env")
def sim_env(ns: str):
    container = _get_sim_container(ns)
    result = _docker(["exec", container, "printenv", "PLANNER", "CONTROLLER"], timeout=8)
    lines = result.stdout.splitlines()
    # printenv按参数顺序逐行输出，取不到时用docker-compose.yml的默认值兜底，
    # 跟_get_controller()同样的兜底逻辑。
    planner = lines[0].strip() if len(lines) > 0 and lines[0].strip() else "ego_planner"
    controller = lines[1].strip() if len(lines) > 1 and lines[1].strip() else "px4ctrl"
    return {"planner": planner, "controller": controller}


@app.get("/api/sim/{ns}/slam-hz")
def sim_slam_hz(ns: str):
    # 网页上"SLAM频率"那一列走rosbridge订阅算出来的，实测发现只有6-7Hz，
    # 但DLIO自己原生发布是46-48Hz——nav_msgs/msg/Odometry自带两个6x6
    # 协方差矩阵，这个速率下序列化成JSON对rosbridge_server这个单线程
    # Python进程吞吐量太大，超出处理能力的消息直接被丢弃，不是DLIO本身
    # 慢。这个接口现查*原生*频率（容器内直接跑`ros2 topic hz`，不经过
    # rosbridge这层，没有同样的瓶颈），给网页一个对照的真实数字。
    #
    # 每次调用都要真的等几秒钟收集样本（`ros2 topic hz`不是查一次就有
    # 结果，需要真的订阅一段时间攒够窗口），比其它"现查一下就返回"的
    # 接口(sim_env等)慢得多——前端不应该像其它状态那样高频轮询这个
    # 接口，见gcs/frontend/index.html里调用它的地方的注释。
    ns = ns.upper()
    container = _get_sim_container(ns)
    if _docker_status(container) != "running":
        raise HTTPException(409, f"{container} 不在running状态")
    result = _docker(["cp", str(SCRIPTS_DIR / "ros2_env_setup.sh"), f"{container}:/tmp/ros2_env_setup.sh"], timeout=10)
    if result.returncode != 0:
        raise HTTPException(500, f"docker cp ros2_env_setup.sh 到 {container} 失败: {result.stderr.strip()}")
    shell = (
        "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && "
        f"timeout 6 ros2 topic hz /{ns}/dlio/odom_node/odom --window 30 2>&1"
    )
    result = _docker(["exec", container, "bash", "-c", shell], timeout=12)
    # 取最后一次"average rate"——ros2 topic hz每秒左右刷新一次输出，用
    # 攒够更多样本之后的那个数字，不是刚开始订阅、样本还很少的第一个。
    matches = re.findall(r"average rate:\s*([\d.]+)", result.stdout)
    if not matches:
        return {"hz": None, "message": "6秒内没能拿到有效样本（话题可能暂时没有消息，或DDS发现还没完成）"}
    return {"hz": float(matches[-1])}


CLEAR_LOGS_CONTAINER = "docker_sim-flight-stack-nx01-1"


@app.post("/api/clear-logs")
def clear_logs():
    # 破坏性操作——双重确认是前端(两次点击"武装"模式，5秒窗口，跟
    # launch_control.py的do_clear_logs()同款UX)的职责，这个接口本身不
    # 重复实现确认逻辑，只管执行。不删incidents/下save_incident.sh手动
    # 留证的文件夹——留证功能的设计初衷就是不受滚动清理影响。
    shell = (
        "shopt -s nullglob; "
        "rm -rf /logs/rosbag/* /logs/container_logs/* /logs/NX01/* /logs/NX02/*; "
        "rm -f /logs/health_status.txt /logs/prune_rosbag_stdout.log "
        "/logs/record_rosbag_stdout.log /logs/incidents/health_events.log; "
        "echo done"
    )
    result = _docker(["exec", CLEAR_LOGS_CONTAINER, "bash", "-c", shell], timeout=15)
    if result.returncode != 0:
        raise HTTPException(500, f"清空记录文件失败: {(result.stdout + result.stderr).strip()}")
    return {
        "success": True,
        "message": "已清空rosbag/容器持久化日志/姿态推力调试日志/health_events.log（未删除incidents/下手动留证的文件夹）",
    }


RESOURCE_CONTAINERS = {
    "docker_sim-sim-world-1": "sim-world",
    "docker_sim-flight-stack-nx01-1": "NX01",
    "docker_sim-flight-stack-nx02-1": "NX02",
}


@app.get("/api/container-stats")
def container_stats():
    result = _docker(
        ["stats", "--no-stream", "--format", "{{.Name}}|{{.CPUPerc}}|{{.MemUsage}}|{{.NetIO}}"]
        + list(RESOURCE_CONTAINERS),
        timeout=15,
    )
    rows = {}
    for line in result.stdout.splitlines():
        parts = line.split("|")
        if len(parts) != 4:
            continue
        name, cpu, mem, netio = parts
        rows[RESOURCE_CONTAINERS.get(name, name)] = {"cpu": cpu, "mem": mem, "netio": netio}

    # GPU利用率是整机层面的，nvidia-smi本身不按容器拆分——只有sim-world
    # 容器实际用GPU(Gazebo渲染)、也只有它装了nvidia-smi+GPU直通，跟
    # status_monitor.py"GPU这一列只填sim-world那一行"同样的理由。
    gpu = None
    gpu_result = _docker(
        ["exec", "docker_sim-sim-world-1", "nvidia-smi",
         "--query-gpu=utilization.gpu,memory.used,memory.total", "--format=csv,noheader,nounits"],
        timeout=8,
    )
    if gpu_result.returncode == 0 and gpu_result.stdout.strip():
        parts = [p.strip() for p in gpu_result.stdout.strip().splitlines()[0].split(",")]
        if len(parts) == 3:
            gpu = {"util_pct": parts[0], "mem_used_mib": parts[1], "mem_total_mib": parts[2]}

    return {"containers": rows, "gpu": gpu}


# ============================================================
# 6. GUI窗口按需启动——Gazebo图形界面/RViz2
# ============================================================
# 2026-08-21新增：docker-compose.yml的USE_GAZEBO_GUI/USE_RVIZ改回默认
# false（之前被验证GUI崩溃修复时顺手写死成true、忘了改回来，见那两行
# 的注释），默认不再自动弹出这两个窗口——但"万一想看"这个需求还在，
# 这两个接口负责按需现起，不需要重启sim-world整个容器（那样会把PX4/
# UWB节点也一起重启，代价太大）。
#
# gzclient必须在sim-world容器里起——它要连本地已经在跑的gzserver，Gazebo
# 的client/server是同机传输，不能跨容器连接。这里的source列表原样照抄
# `docker/entrypoints/sim-world-entrypoint.sh`起完整仿真前做的那一串
# source（尤其是`source /usr/share/gazebo/setup.sh`——TODO.md#11那次
# 排查GUI崩溃的真正修复点，GAZEBO_RESOURCE_PATH/OGRE_RESOURCE_PATH
# 不source这一行就是空的，会导致OGRE渲染初始化时空指针assert崩溃），
# 只是最后不再执行`ros2 launch ...`那一整套（不重新起仿真），改成只
# 起`gzclient`这一个进程去连接已经在跑的gzserver。
SIM_WORLD_CONTAINER = "docker_sim-sim-world-1"
# 2026-08-22查出的坑：sim-world容器这次是在DISPLAY还没确定/为空的某次
# `docker compose up`里起的，容器创建时刻就把DISPLAY环境变量固化成了
# 空字符串——docker exec不会重新解析宿主机当前的$DISPLAY，只会继承容器
# *创建时*那份（跟RMW_IMPLEMENTATION那个老坑同一个根源，见launch_rviz()
# 注释），实测现象是gzclient直接"dumped core"崩溃，不是清晰的X11报错。
# 正确修法是让sim-world容器重新创建来刷新这个环境变量，但这个容器当时
# 正有飞机在天上飞（NX01已解锁OFFBOARD），重建会让物理引擎/飞控状态
# 全部丢失，不能说改就改。这里改成docker exec时显式传-e覆盖，不依赖
# 容器自己创建时固化的值——这台开发机只有一个图形桌面会话，DISPLAY
# 固定是:1，不需要动态探测（GCS本来就是"本地/内网单操作员用"的定位，
# 见README，不用支持多显示器/多会话主机这种场景）。
HOST_DISPLAY = ":1"
GAZEBO_ENV_SOURCE_SHELL = (
    "source /opt/ros/humble/setup.bash && "
    "source /opt/decomp_ws/install/setup.bash 2>/dev/null; "
    "source /opt/livox_ws/install/setup.bash 2>/dev/null; "
    "source /opt/mighty_ws/install/setup.bash && "
    "source /opt/uwb_ws/install/setup.bash && "
    "source /usr/share/gazebo/setup.sh && "
    "source /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash "
    "/opt/PX4-Autopilot /opt/PX4-Autopilot/build/px4_sitl_default"
)


@app.post("/api/sim/gazebo-gui")
def launch_gazebo_gui():
    if _docker_status(SIM_WORLD_CONTAINER) != "running":
        raise HTTPException(409, f"{SIM_WORLD_CONTAINER} 不在running状态，先在'本地仿真'面板启动仿真机队")
    shell = f"{GAZEBO_ENV_SOURCE_SHELL} && gzclient"
    result = _docker(
        ["exec", "-d", "-e", f"DISPLAY={HOST_DISPLAY}", SIM_WORLD_CONTAINER, "bash", "-c", shell],
        timeout=8,
    )
    if result.returncode != 0:
        raise HTTPException(500, f"启动gzclient失败: {(result.stdout + result.stderr).strip()}")
    return {
        "success": True,
        "message": "已在宿主机X server上后台启动gzclient——如果几秒内没弹出窗口，先在宿主机终端跑一次scripts/up_gcs.sh（里面已经带了xhost授权），再点一次这个按钮",
    }


@app.post("/api/gcs/rviz")
def launch_rviz():
    # 借NX01容器现查PLANNER决定用哪份rviz配置，跟sim_env()同样的兜底
    # 逻辑；rviz2本身固定从gcs-gcs-1容器起（不是sim-world）——复用这个
    # 容器已经装好的rviz2+/opt/rviz/*.rviz配置和DISPLAY/X11转发设置，
    # 是《网页端地面站控制面板方案.md》里一直有效的手动操作
    # (`docker exec -it docker_sim-gcs-1 rviz2 -d /opt/rviz/...`)的
    # 按钮化版本，不是重新发明一套。
    container = _get_sim_container("NX01")
    planner = "ego_planner"
    if _docker_status(container) == "running":
        env_result = _docker(["exec", container, "printenv", "PLANNER"], timeout=8)
        planner = env_result.stdout.strip() or "ego_planner"
    rviz_config = "multi_mighty.rviz" if planner == "mighty" else "multi_ego_planner.rviz"
    # 2026-08-21修复：漏了RMW_IMPLEMENTATION——`docker exec`起的新shell只
    # 继承容器*创建时*就设好的环境变量(compose environment:里的
    # CYCLONEDDS_URI)，不会继承`gcs/entrypoint.sh`自己进程树里运行时
    # export的RMW_IMPLEMENTATION，这条坑TODO.md第19项/scripts/
    # ros2_env_setup.sh文件头都记过，这次在这个新按钮上又踩了一次——
    # 实测现象完全符合：rviz2进程真的起来了、CPU占用正常，但因为默认
    # 走FastDDS、根本连不上PLANNER=ego_planner时切到CycloneDDS的真实
    # DDS域，界面里空空如也收不到任何话题。跟gcs/entrypoint.sh用同一个
    # 判断条件（只有PLANNER=ego_planner时才需要切）。
    rmw_export = "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && " if planner == "ego_planner" else ""
    shell = (
        "source /opt/ros/humble/setup.bash && source /opt/gcs_ws/install/setup.bash 2>/dev/null; "
        f"{rmw_export}"
        f"rviz2 -d /opt/rviz/{rviz_config}"
    )
    # 跟launch_gazebo_gui()同样显式传-e DISPLAY覆盖——gcs-gcs-1这次碰巧
    # 创建时就有正确的DISPLAY(:1)，rviz2目前能弹出来不是靠运气该有的
    # 保障；显式传一次成本很低，防的是这个容器以后被recreate、又不巧撞在
    # DISPLAY为空的时机上，重蹈sim-world那次的覆辙。
    result = _docker(
        ["exec", "-d", "-e", f"DISPLAY={HOST_DISPLAY}", GCS_CONTAINER_NAME, "bash", "-c", shell],
        timeout=8,
    )
    if result.returncode != 0:
        raise HTTPException(500, f"启动rviz2失败: {(result.stdout + result.stderr).strip()}")
    return {
        "success": True,
        "message": f"已在宿主机X server上后台启动rviz2({rviz_config})——如果几秒内没弹出窗口，先在宿主机终端跑一次scripts/up_gcs.sh（里面已经带了xhost授权），再点一次这个按钮",
    }


# ============================================================
# 7. 真机集群远程控制（《单机接口描述文件.md》，2026-08-24新增）
# ============================================================
# 这是这个后端唯一的真机控制通路——不需要SSH凭证/docker.sock，飞机侧
# 只要在跑docker_sim/scripts/control_server.py就能被控制，机身数量不
# 写死在代码里（UDP广播动态发现，理论上支持任意数量飞机）。2026-08-25
# 之前这里还并存一条SSH通路（改flight-stack-hw的.env调参+机队启停），
# 已经全部删除（先删了SSH机队启停面板，后来"⚙规划/避障调参"也被要求
# 彻底删除），这个后端现在完全不需要挂载~/.ssh、也不再有任何SSH到
# Jetson的代码路径。
#
# 2026-08-25：X-Control-Token（原《单机接口描述文件.md》3.1节鉴权机制）
# 已经应用户要求整个删掉，这个后端不再持有/转发任何令牌——对飞机端的
# HTTP请求直接发，不带任何鉴权头，见_hw_fleet_proxy()。

DISCOVERY_PORT = 8891
DISCOVERY_STALE_SEC = 8  # 心跳周期2秒，取4个周期做超时判定，跟接口文档
                          # 建议的"5~10秒/2~5个心跳周期"一致

# namespace -> {"info": <心跳JSON dict>, "seen": time.monotonic()}，只有
# 收到过UDP心跳的飞机才会出现在这里；判定"在线"看seen距现在是否超过
# DISCOVERY_STALE_SEC，不在这个dict里单独维护一个"离线"状态，每次读的
# 时候现算，见hw_fleet_discovered()。
_discovered = {}
_discovery_lock = threading.Lock()


def _discovery_listener():
    # 单独的守护线程常驻监听，跟uvicorn的asyncio事件循环互不干扰——这里
    # 用的是阻塞式socket.recvfrom，不是asyncio datagram，图简单，反正
    # 这个线程唯一的工作就是"收包->更新dict"，没有并发调用其它逻辑的需要。
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
    except OSError as e:
        # 绑不上（比如端口被占用）不能让整个后端进程崩掉——发现功能失效，
        # 但改手动IP兜底、代理控制这些其它接口应该继续可用，错误只能靠
        # uvicorn启动日志里的这行print看到，没有更好的上报渠道。
        print(f"[hw-fleet-discovery] 绑定UDP:{DISCOVERY_PORT}失败，发现功能不可用: {e}")
        return
    while True:
        try:
            data, _addr = sock.recvfrom(4096)
            info = json.loads(data.decode("utf-8"))
            ns = info.get("namespace")
            if not ns:
                continue
            with _discovery_lock:
                _discovered[ns] = {"info": info, "seen": time.monotonic()}
        except Exception:
            # 单条心跳解析失败（比如局域网里凑巧收到别的UDP广播、不是
            # 合法JSON）不能打垮这个常驻线程——跳过这一条，继续收下一条。
            continue


@app.on_event("startup")
def _start_discovery_listener():
    threading.Thread(target=_discovery_listener, daemon=True).start()


# 手动IP兜底/检测模式默认值配置持久化到一个本地JSON文件——跟
# gcs/cyclonedds_gcs.xml同样的"改文件不改代码"思路。文件名
# hw_fleet_tokens.json是2026-08-25删掉令牌鉴权之前留下的历史命名，
# 文件本身不进git，见gcs/hw_fleet_tokens.json.example这份模板和
# .gitignore里的说明——内容已经不含任何令牌，名字暂时没跟着改，
# 避免连带影响已部署环境的HW_FLEET_CONFIG_PATH路径。
HW_FLEET_CONFIG_PATH = Path(
    os.environ.get("HW_FLEET_CONFIG_PATH", "/opt/gcs_config/hw_fleet_tokens.json")
)
_hw_fleet_config_lock = threading.Lock()


def _load_hw_fleet_config() -> dict:
    with _hw_fleet_config_lock:
        if not HW_FLEET_CONFIG_PATH.exists():
            return {}
        try:
            return json.loads(HW_FLEET_CONFIG_PATH.read_text() or "{}")
        except json.JSONDecodeError:
            return {}


def _save_hw_fleet_config(cfg: dict) -> None:
    with _hw_fleet_config_lock:
        HW_FLEET_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        HW_FLEET_CONFIG_PATH.write_text(json.dumps(cfg, ensure_ascii=False, indent=2))


def _resolve_hw_fleet_target(ns: str) -> tuple[str, int]:
    """返回(ip, control_port)。优先用UDP发现表里的活跃条目（IP/端口
    可能会变，比如飞机重新DHCP拿到新地址），发现表没有活跃条目时退化到
    配置里的手动IP/端口（跨子网收不到广播的场景，见接口文档第6节已知
    限制第4条）。
    """
    with _discovery_lock:
        entry = _discovered.get(ns)
        fresh = entry is not None and (time.monotonic() - entry["seen"]) <= DISCOVERY_STALE_SEC
        info = dict(entry["info"]) if fresh else None
    cfg = _load_hw_fleet_config().get(ns, {})
    if info:
        ip, port = info.get("ip"), info.get("control_port")
    elif cfg.get("ip") and cfg.get("control_port"):
        ip, port = cfg["ip"], cfg["control_port"]
    else:
        raise HTTPException(
            404,
            f"{ns} 当前不在线（UDP发现表里没有新鲜心跳），且没有配置手动IP/端口兜底——"
            "先确认飞机已开机并在广播，或者在'真机集群控制'面板里手动填IP/端口",
        )
    return ip, port


def _hw_fleet_proxy(ns: str, method: str, path: str, body: dict | None, timeout: float):
    ip, port = _resolve_hw_fleet_target(ns)
    url = f"http://{ip}:{port}{path}"
    try:
        resp = requests.request(method, url, json=body, timeout=timeout)
    except requests.exceptions.RequestException as e:
        raise HTTPException(502, f"连接{ns}({ip}:{port})失败: {e}")
    if resp.status_code >= 400:
        try:
            data = resp.json()
            detail = data.get("message") or data.get("error") or resp.text
        except ValueError:
            detail = resp.text or resp.reason
        raise HTTPException(resp.status_code, detail)
    try:
        return resp.json()
    except ValueError:
        raise HTTPException(502, f"{ns} 返回了非JSON响应（跟接口文档不符）: {resp.text[:200]}")


@app.get("/api/hw-fleet/discovered")
def hw_fleet_discovered():
    cfg = _load_hw_fleet_config()
    now = time.monotonic()
    with _discovery_lock:
        items = list(_discovered.items())
    drones = []
    for ns, entry in items:
        age = now - entry["seen"]
        if age > DISCOVERY_STALE_SEC:
            continue  # 判定离线，不列进结果——跟接口文档第2节的判定逻辑一致
        info = entry["info"]
        drones.append({
            "namespace": ns,
            "ip": info.get("ip"),
            "control_port": info.get("control_port"),
            "hostname": info.get("hostname"),
            "flight_up": info.get("flight_up"),
            "vision_up": info.get("vision_up"),
            "age_sec": round(age, 1),
            "configured": ns in cfg,
            # 2026-08-29新增：飞机端control_server.py的UDP心跳现在随包带
            # 四个运行模式（LOCALIZATION_SOURCE/SLAM_BACKEND/PLANNER/
            # CONTROLLER，见read_stack_config()），心跳本来就是常驻监听、
            # 不需要额外轮询就能拿到，原样透传——只是"当前运行值"，没有
            # pending/mismatch（那两项要读/status，见hw_fleet_stack_status()）。
            "stack": info.get("stack"),
            # 2026-09-04新增：SSH到真机确认control_server.py心跳现在随包带
            # flight_started_at(容器真实启动时间，docker inspect的
            # State.StartedAt)/cpu_temp_c(Jetson tj-thermal热区温度)——地面站
            # "飞行栈启动时长"/"机载计算机温度"这两项监控用，原样透传，不在
            # 这个后端做任何计算（时长换算/单位转换都在前端做，见gcs/frontend/
            # index.html的hwTimingHtml()）。旧版本control_server.py（还没
            # 部署这次更新的机器）心跳里没有这两个key，info.get()返回None，
            # 前端会显示"--"，不会报错。
            "flight_started_at": info.get("flight_started_at"),
            "cpu_temp_c": info.get("cpu_temp_c"),
            # 2026-09-04同日新增：原计划的"飞控温度"(前端订阅
            # mavros/imu/temperature_imu)被SSH实测确认是不可信的死数(连续
            # 6秒读数一模一样，见DEBUG_JOURNAL同日记录)，用户要求换成机载
            # 计算机CPU占用率——control_server.py同一批更新里加的字段，见
            # 该文件sample_cpu_usage_pct()的说明，同样原样透传。
            "cpu_usage_pct": info.get("cpu_usage_pct"),
        })
    drones.sort(key=lambda d: d["namespace"])
    return {"drones": drones}


@app.get("/api/hw-fleet/targets")
def hw_fleet_targets():
    # 2026-08-25：这一整架飞机的手动IP/端口+检测模式默认值都在这里回传
    # 给前端——"保存配置"/"删除本地配置"这两个按钮明确覆盖的是这整套
    # 配置(HW_FLEET_TARGET_FIELDS)，不是分字段管理。令牌(has_token字段)
    # 已经不存在了，X-Control-Token鉴权同一天被用户要求彻底删掉，地面
    # 站和飞机端都不再有这个概念，见文件头说明。
    cfg = _load_hw_fleet_config()
    return {
        ns: {k: v.get(k) for k in HW_FLEET_TARGET_FIELDS}
        for ns, v in cfg.items()
    }


# 一架真机的完整本地配置——"保存配置"/"删除本地配置"这两个按钮明确覆盖
# 的就是这一整套字段。这个元组里的字段都是"保存时整体覆盖、删除时整体
# 清空"，新增字段只需要加进这个元组，GET/POST/DELETE三个接口不用跟着改。
#
# 2026-08-25：两处breaking change叠在一起——
# (a) 飞机端接口（见docs/单机接口描述文件.md 2026-08-25条目，飞机上
#     SSH直接读的）：第二路相机接上后，视觉检测从"单路摄像头三选一"
#     改成"cam0(前视)/cam1(下视)两路各自独立三选一"，POST /vision/mode
#     新增必填字段cam；RTP(stream_mode/gcs_ip/stream_port/bitrate_kbps)
#     整个从接口里删掉，只剩MJPEG拉流。原来的vision_mode/stream_mode/
#     gcs_ip/mjpeg_port/stream_port这五个字段换成按摄像头分的
#     cam0_mode/cam0_mjpeg_port/cam1_mode/cam1_mjpeg_port。
# (b) X-Control-Token鉴权应用户明确要求整个删掉（地面站和飞机端
#     control_server.py都改了）——原来的token字段（连同"首次必填"
#     那道校验）一起删掉，不再是这里的字段之一。
# 2026-08-26新增planner字段：真机命名空间没有本机容器可以docker exec查
# PLANNER环境变量（sim_env()那条路只认SIM_NS_CONTAINERS写死的两个本机
# 仿真容器），之前真机命名空间也会被前端误当成仿真去查一次，查的是本机
# 一个跟真机同名但实际没在跑的容器，结果要么404导致障碍物栏格订阅整个
# 被跳过、要么退到硬编码默认值蒙对/蒙错——续33分析过这是"真机障碍物点云
# 显示不出来"的根因。这里改成跟cam0_mode/cam1_mode一样手工登记：网页
# "真机集群"面板选一次规划器类型、点保存，前端ensureSubscribed()直接读
# 这份静态配置决定订阅grid_map/occupancy_inflate还是occupancy_grid，
# 不再猜测。2026-08-26续35：没配置过(值是null)时前端按ego_planner当
# 默认值处理（对齐docker-compose.hw.yml的PLANNER默认值），不是"不填
# 就不订阅"了，只有真机实际跑mighty才需要显式覆盖——默认值逻辑在前端
# ensureOccAndTrajSubscribed()里，这里后端不用跟着改，原样存/传null。
HW_FLEET_TARGET_FIELDS = (
    "ip", "control_port",
    "cam0_mode", "cam0_mjpeg_port", "cam1_mode", "cam1_mjpeg_port",
    "planner",
)


class HwFleetTargetReq(BaseModel):
    namespace: str
    ip: str | None = None
    control_port: int | None = None
    cam0_mode: str | None = None
    cam0_mjpeg_port: int | None = None
    cam1_mode: str | None = None
    cam1_mjpeg_port: int | None = None
    planner: str | None = None


@app.post("/api/hw-fleet/targets")
def set_hw_fleet_target(req: HwFleetTargetReq):
    ns = req.namespace.strip()
    if not ns:
        raise HTTPException(400, "namespace不能为空")
    cfg = _load_hw_fleet_config()
    # 全部字段（ip/control_port/检测模式/MJPEG端口）整体覆盖成这次提交
    # 的值——前端每次点"保存配置"会把当前这一行表单里所有字段都提交
    # 上来，语义是"这整架飞机的配置现在长这样"，不是逐字段增量patch。
    cfg[ns] = {
        "ip": req.ip or None,
        "control_port": req.control_port or None,
        "cam0_mode": req.cam0_mode or None,
        "cam0_mjpeg_port": req.cam0_mjpeg_port or None,
        "cam1_mode": req.cam1_mode or None,
        "cam1_mjpeg_port": req.cam1_mjpeg_port or None,
        "planner": req.planner or None,
    }
    _save_hw_fleet_config(cfg)
    return {"success": True, "message": f"已保存 {ns} 的完整配置（手动IP端口/cam0与cam1检测模式/MJPEG端口/规划器类型）"}


@app.delete("/api/hw-fleet/targets/{ns}")
def delete_hw_fleet_target(ns: str):
    # 整条记录一起删——手动IP端口/cam0与cam1检测模式/MJPEG端口，没有"只删一部分"
    # 这种半吊子操作，跟set_hw_fleet_target()"整体覆盖"是同一个"针对
    # 整架飞机"的设计，不是分字段管理。
    cfg = _load_hw_fleet_config()
    if ns not in cfg:
        raise HTTPException(404, f"{ns} 未配置")
    del cfg[ns]
    _save_hw_fleet_config(cfg)
    return {"success": True, "message": f"已删除 {ns} 的完整本地配置（手动IP端口/cam0与cam1检测模式/MJPEG端口/规划器类型）"}


@app.get("/api/hw-fleet/{ns}/status")
def hw_fleet_status(ns: str):
    # 对应接口文档3.3节GET /status——含stack(运行值)/stack_pending(待生效值)/
    # stack_mismatch(两者不一致的key，非空=已改配置但没重启生效)，2026-08-29
    # 飞机端control_server.py新增，见该文件里read_stack_config()的说明。
    return _hw_fleet_proxy(ns, "GET", "/status", None, timeout=5)


@app.get("/api/hw-fleet/{ns}/contract")
def hw_fleet_contract(ns: str):
    # 代理飞机端GET /api——机器可读的接口契约，含stack_choices(四个模式
    # 各自的合法取值)/hw_forbidden(真机禁用的取值+原因)。2026-08-29新增：
    # 故意不在GCS这边照抄一份STACK_CHOICES/HW_FORBIDDEN常量——飞机端这份
    # 文档就是照着服务端校验用的同一份常量生成的（见control_server.py
    # 文件头"接口契约"说明"不会出现文档与实现漂移"），GCS这边如果自己抄
    # 一份，飞机端以后加新的CONTROLLER取值/改校验规则，GCS就会悄悄跟真实
    # 情况脱节——现查一次的成本(这个面板本来就要展开操作，不是高频轮询)
    # 远低于维护两份取值表可能漂移的风险。
    return _hw_fleet_proxy(ns, "GET", "/api", None, timeout=5)


class HwFleetStackModeReq(BaseModel):
    localization: str | None = None
    slam: str | None = None
    planner: str | None = None
    controller: str | None = None


@app.post("/api/hw-fleet/{ns}/stack-mode")
def hw_fleet_stack_mode(ns: str, req: HwFleetStackModeReq):
    # 代理飞机端POST /stack/mode——只写飞机上的.env，不碰容器，所以不需要
    # confirm（跟POST /stack的down/restart不一样，那个会直接掐断飞控软件）。
    # exclude_none：四项都可省略，省略的飞机端会沿用.env里的现值（"只换
    # 控制器"这种局部改动），不传等于不改那一项，不是传空字符串覆盖成空。
    body = req.model_dump(exclude_none=True)
    if not body:
        raise HTTPException(400, "localization/slam/planner/controller至少要指定一项")
    return _hw_fleet_proxy(ns, "POST", "/stack/mode", body, timeout=10)


class HwFleetStackReq(BaseModel):
    stack: str
    action: str
    confirm: bool = False


@app.post("/api/hw-fleet/{ns}/stack")
def hw_fleet_stack(ns: str, req: HwFleetStackReq):
    # 2026-08-25修复：原来只有vision+up给了130秒超时（对应vision+up会
    # 顺带colcon build、接口文档3.4节写的飞机端服务器自己120秒超时上限），
    # 其它组合(尤其是flight+up)给的是30秒——实测真机上flight+up（起
    # flight-stack-hw容器，包含PX4 SITL/MAVLink桥接等初始化）经常超过
    # 30秒才返回，表现是"飞机那边其实启动成功了，但GCS这边先超时断开、
    # 报一个ConnectionPool Read timed out的错误"，看起来像失败，实际是
    # 我们自己不耐烦。接口文档3.4节说的120秒上限看起来是飞机端服务器
    # 对"所有写操作"的通用超时，不只是vision+up专属，所以这里不再区分
    # stack/action，统一给一个盖过飞机端120秒上限的超时。
    timeout = 130
    result = _hw_fleet_proxy(ns, "POST", "/stack", req.model_dump(), timeout=timeout)
    # 2026-08-25新增：用户要求"启动vision/重启vision顺带自动调一次
    # vision-mode"——vision-stack容器起来之后本身是空的(entrypoint是
    # sleep infinity占位，实测SSH确认过)，真正拉起检测/MJPEG推流进程
    # 必须另外调POST /vision/mode，之前唯一能触发这一步的"应用检测
    # 模式"按钮已经按用户要求删掉了，改成这里自动补上，不需要用户在
    # GCS上再多点一次。用保存的本地配置(cam0_mode/cam1_mode，没配置
    # 就跳过那一路，不强加默认值)；两路摄像头分别调用、互不影响（跟
    # 接口文档3.5节"两路摄像头之间互不影响"一致），任何一路失败不影响
    # 另一路、也不影响这次/stack本身已经成功的判定——失败信息追加进
    # 返回的message里，不单独抛异常掩盖掉容器启动本身成功的事实。
    if req.stack == "vision" and req.action in ("up", "restart") and isinstance(result, dict):
        cfg = _load_hw_fleet_config().get(ns, {})
        notes = []
        for cam, mode_key, port_key in (
            ("cam0", "cam0_mode", "cam0_mjpeg_port"),
            ("cam1", "cam1_mode", "cam1_mjpeg_port"),
        ):
            mode = cfg.get(mode_key)
            if not mode:
                continue
            body = {"cam": cam, "mode": mode}
            if cfg.get(port_key):
                body["mjpeg_port"] = cfg[port_key]
            try:
                vm_result = _hw_fleet_proxy(ns, "POST", "/vision/mode", body, timeout=20)
                notes.append(f"{cam}: {vm_result.get('message', '已应用')}")
            except HTTPException as e:
                notes.append(f"{cam}应用检测模式失败: {e.detail}")
        if notes:
            result = {**result, "message": (result.get("message", "") + "\n" + "\n".join(notes)).strip()}
    return result


# 2026-08-25：飞机端POST /vision/mode接口breaking change（见docs/单机
# 接口描述文件.md同日条目）——cam是新增必填字段（"cam0"/"cam1"/"all"，
# all只能配合mode="stop"），RTP相关字段(stream_mode/gcs_ip/stream_port/
# bitrate_kbps)已经从飞机端接口整个删掉，这里的模型跟着同步，不再声明
# 这几个飞机端已经不认的字段——camera_width/height/framerate/
# publish_rate_hz/sensor_id这几个可选字段飞机端接口仍然支持，GCS这边
# 目前UI没有暴露对应输入框（跟老版本UI的范围保持一致，只暴露mode+
# mjpeg_port这两个最常用的），但模型里留着，以后要加输入框直接能用，
# 不用再改这个模型。
class HwFleetVisionModeReq(BaseModel):
    cam: str
    mode: str
    sensor_id: int | None = None
    camera_width: int | None = None
    camera_height: int | None = None
    camera_framerate: int | None = None
    publish_rate_hz: float | None = None
    mjpeg_port: int | None = None


@app.post("/api/hw-fleet/{ns}/vision-mode")
def hw_fleet_vision_mode(ns: str, req: HwFleetVisionModeReq):
    # exclude_none——没填的字段不传，让飞机端接口自己的默认值生效（接口
    # 文档3.5节表格里的默认值），不越权替用户做决定。
    return _hw_fleet_proxy(ns, "POST", "/vision/mode", req.model_dump(exclude_none=True), timeout=20)


class HwFleetRenameReq(BaseModel):
    namespace: str
    confirm: bool = True


@app.post("/api/hw-fleet/{ns}/rename")
def hw_fleet_rename(ns: str, req: HwFleetRenameReq):
    new_ns = req.namespace.strip()
    if not new_ns:
        raise HTTPException(400, "新namespace不能为空")
    if not req.confirm:
        raise HTTPException(400, "改命名空间会重启flight-stack和vision-stack两个容器，需要显式带 confirm: true")
    result = _hw_fleet_proxy(
        ns, "POST", "/rename", {"namespace": new_ns, "confirm": True}, timeout=90
    )
    # 改名成功后，本地配置的key也要跟着从旧namespace挪到新namespace——
    # 手动IP/端口/检测模式都不受改名影响（还是同一台飞机，见接口文档
    # 3.6节），只有namespace变了；不迁移的话下次用新namespace控制这架
    # 飞机时，配置里找不到对应记录，会被误判成"没配置过"。
    cfg = _load_hw_fleet_config()
    if ns in cfg and ns != new_ns:
        cfg[new_ns] = cfg.pop(ns)
        _save_hw_fleet_config(cfg)
    return result
