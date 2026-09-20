#!/usr/bin/env python3
"""tmux "launch"窗口用的起飞/降落/锁定原点/清空记录控制台——curses实现，
鼠标真的能点按钮（tmux鼠标模式在mighty_sim.tmux.conf里本来就开着），
同时保留数字键1~7当键盘快捷方式（鼠标转发在某些SSH/终端组合下不一定
生效，键盘是保底）。2026-08-12新增第7个按钮"清空记录文件"，见
do_clear_logs()。

之前bash版本有两个问题：1）"按钮"只是ANSI背景色块，本质还是数字菜单，
不是真的可以点；2）起飞用`touch /tmp/takeoff_go`，这个文件只被一次性的
`takeoff_gate`节点（px4ctrl/so3ctrl路径）或`dynus_offboard_node.py`里
只在flight_state=="TAKEOFF"这个阶段轮询的检查（ros2_px4_stack路径）
消费——降落一次之后再起飞，前者的节点早就跑完退出了，后者的状态机
已经离开TAKEOFF阶段、永远不会再回去，两条路径都不会响应第二次touch。

这一版按CONTROLLER（从容器里读环境变量）分流，两条路径行为不同：

- CONTROLLER=px4ctrl / so3ctrl / pt4ctrl：三者共用同一套PX4CtrlFSM状态机
  （PX4CtrlFSM.cpp头部那张图：MANUAL_CTRL<->AUTO_TAKEOFF、
  AUTO_HOVER<->AUTO_LAND本来就是个环），起飞/降落都改成直接往
  quadrotor_msgs/msg/TakeoffLand话题发消息（TAKEOFF=1/LAND=2）——这是
  长驻订阅、可以反复触发，降落完成、状态机回到MANUAL_CTRL之后，再发
  一次TAKEOFF能正常重新起飞，不再依赖一次性的文件监听节点。
  pt4ctrl（2026-08-13新增）是px4ctrl砍掉控制律、改发轨迹setpoint的
  改造版，状态机文件基本逐字复用，但**降落触发这一处2026-08-13又单独
  改过**：px4ctrl/so3ctrl原版`CMD_CTRL`状态下收到LAND会直接拒绝
  （"must be triggered in AUTO_HOVER"）——docker_sim实测发现mighty/
  ego_planner这类"持续发布反馈式setpoint流"的规划器就算已经悬停在
  目标点不再移动也不会主动停止发cmd，导致`CMD_CTRL`永远等不到
  "没有新指令"这个退回`AUTO_HOVER`的窗口，降落按钮形同虚设（见
  DEBUG_JOURNAL.md"降落按钮好像都不好用"那次排查）。pt4ctrl这份FSM
  已经改成`CMD_CTRL`下也能直接触发`AUTO_LAND`（前置条件`state ==
  CMD_CTRL`，RC/odom/cmd都还健康才允许，安全等级跟AUTO_HOVER那条LAND
  分支一样），**px4ctrl/so3ctrl还是原版行为，没有跟着改**——用这两个
  控制器时仍然要如实告知"飞机正在执行任务时按降落键不会立刻生效，
  得等它先进悬停"这条限制，只有pt4ctrl没有这个限制。

- CONTROLLER=ros2_px4_stack（默认）：起飞仍然是touch文件（这条路径唯一
  认的机制），降落改用直接调mavros的AUTO.LAND（这条路径没有内置的
  降落状态机，只能走这条外部指令）。**如实告知这条链路架构上不支持
  "降落了再重新起飞"**——dynus_offboard_node.py的flight_state没有
  "回到TAKEOFF"这条路径，_kick_offboard()那套自动解锁/切OFFBOARD逻辑
  也是一次性的，不会在这里假装能做到。
"""
import curses
import subprocess
import sys
import os
import time

NAMESPACES = [
    ("NX01", "docker_sim-flight-stack-nx01-1"),
    ("NX02", "docker_sim-flight-stack-nx02-1"),
]
GATE_FILE = "/tmp/takeoff_go"
REPEATABLE_CONTROLLERS = ("px4ctrl", "so3ctrl", "pt4ctrl")


def _run(cmd, timeout=8):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return False, "命令超时"
    except Exception as e:  # noqa: BLE001 —— 这里就是要把任何docker/subprocess异常都当失败处理并展示原因
        return False, str(e)


def get_controller(container):
    ok, out = _run(["docker", "exec", container, "printenv", "CONTROLLER"], timeout=5)
    out = out.strip()
    # 兜底默认跟docker-compose.yml/flight-stack-entrypoint.sh的CONTROLLER默认值
    # 保持一致（2026-08-10默认改成px4ctrl），只有printenv真的失败/取不到值时才用得上。
    return out if (ok and out) else "px4ctrl"


def publish_takeoff_land(container, ns, cmd_value, log):
    topic = f"/{ns}/takeoff_land"
    # ros2 topic pub --once在DDS discovery没跟上时可能白发（README里记过
    # 这个坑），照抄takeoff_gate.py自己的防御写法：连发3次，不是发一次就
    # 信了。
    # 实测踩过的坑：只source /opt/ros/humble/setup.bash时`ros2 topic pub`会报
    # "The passed message type is invalid"——quadrotor_msgs是so3ctrl/px4ctrl
    # 自己那个workspace built的自定义消息包，不在基础ROS2安装里，必须额外source
    # /opt/px4ctrl_ws/install/setup.bash（so3ctrl跟px4ctrl共用同一份
    # quadrotor_msgs，见docker_sim/DEBUG_JOURNAL.md关于quadrotor_msgs必须解析到
    # px4ctrl_ros2那份的记录）。同时也带上ros2_env_setup.sh——PLANNER=ego_planner
    # 时so3ctrl/px4ctrl节点自己也会被切到CycloneDDS（这个坑之前在mavros服务调用上
    # 踩过，topic pub是同一类问题，一并处理，不留后患）。
    cp_ok, cp_out = _run(["docker", "cp", "scripts/ros2_env_setup.sh",
                          f"{container}:/tmp/ros2_env_setup.sh"], timeout=5)
    if not cp_ok:
        return False, f"docker cp ros2_env_setup.sh失败: {cp_out}"
    shell = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/px4ctrl_ws/install/setup.bash && "
        "source /tmp/ros2_env_setup.sh && "
        f"for i in 1 2 3; do ros2 topic pub --once {topic} "
        f"quadrotor_msgs/msg/TakeoffLand \"{{takeoff_land_cmd: {cmd_value}}}\" "
        "> /dev/null 2>&1; sleep 0.2; done"
    )
    log(f"实际执行: ros2 topic pub --once {topic} "
        f"quadrotor_msgs/msg/TakeoffLand \"{{takeoff_land_cmd: {cmd_value}}}\"（连发3次，"
        "先source px4ctrl_ws+ros2_env_setup.sh）")
    ok, out = _run(["docker", "exec", container, "bash", "-c", shell], timeout=10)
    return ok, out


def start_auto_calibration(ns, container, log):
    """起飞口令发送成功后自动拉起自标定动作（局部系里前进1.1米再退回，
    给origin_setter_node的θ*估计攒样本，具体流程见
    auto_calibration_flight.py文件头说明）。2026-08-12用户明确要求"点起飞
    按钮之后"自动做这件事，不需要操作员再额外去goal窗口手动打坐标。

    docker exec -d后台跑，不阻塞这个按钮的返回、不影响控制台继续响应
    其它按钮——跟同一个文件里attitude_thrust_logger.py是同一个模式。
    自己独立cp一份ros2_env_setup.sh，不依赖调用方是不是已经cp过——两条
    起飞路径(REPEATABLE_CONTROLLERS走TakeoffLand话题/ros2_px4_stack走
    touch文件)只有前者会顺带cp这个文件，后者不会，这里不能假设它已经
    存在。"""
    cp_ok, cp_out = _run(["docker", "cp", "scripts/ros2_env_setup.sh",
                          f"{container}:/tmp/ros2_env_setup.sh"], timeout=5)
    if not cp_ok:
        log(f"!! {ns} 自标定动作没能启动(cp ros2_env_setup.sh失败): {cp_out} !!")
        return
    cp_ok, cp_out = _run(["docker", "cp", "scripts/auto_calibration_flight.py",
                          f"{container}:/tmp/auto_calibration_flight.py"], timeout=5)
    if not cp_ok:
        log(f"!! {ns} 自标定动作没能启动(cp auto_calibration_flight.py失败): {cp_out} !!")
        return
    shell = (
        "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && "
        f"python3 /tmp/auto_calibration_flight.py {ns} "
        ">> /tmp/auto_calibration_flight_stdout.log 2>&1"
    )
    ok, out = _run(["docker", "exec", "-d", container, "bash", "-c", shell], timeout=5)
    if ok:
        log(f"{ns} 自标定动作已在后台启动——悬停3秒后前进1.1米、再等3秒、退回起飞点，"
            "全程约十几秒，飞机会自己动，不用管它")
    else:
        log(f"!! {ns} 自标定动作启动失败: {out} !!")


def do_takeoff(ns, container, log):
    controller = get_controller(container)
    log(f"== {ns}: 下达起飞口令（CONTROLLER={controller}） ==")
    if controller in REPEATABLE_CONTROLLERS:
        ok, out = publish_takeoff_land(container, ns, 1, log)
        log(f"{ns} 起飞口令{'已发送' if ok else ('发送失败: ' + out)}")
    else:
        log(f"实际执行: docker exec {container} touch {GATE_FILE}")
        ok, out = _run(["docker", "exec", container, "touch", GATE_FILE], timeout=5)
        log(f"{ns} 起飞口令{'已下达' if ok else ('下达失败: ' + out)}")
        log("!! 注意：CONTROLLER=ros2_px4_stack这条链路只支持起飞一次——如果这架"
            "飞机已经降落过，这个键不会再让它起飞（dynus_offboard_node.py的状态机"
            "没有回到TAKEOFF这条路径，架构上不支持反复起降，不是这个按钮的bug）!!")
    if ok:
        start_auto_calibration(ns, container, log)


def do_land(ns, container, log):
    controller = get_controller(container)
    log(f"== {ns}: 下达降落口令（CONTROLLER={controller}） ==")
    if controller in REPEATABLE_CONTROLLERS:
        ok, out = publish_takeoff_land(container, ns, 2, log)
        log(f"{ns} 降落口令{'已发送' if ok else ('发送失败: ' + out)}")
        log("   注意：只有飞机处于悬停(AUTO_HOVER)状态才会响应这个指令——如果正在"
            "跟踪规划器轨迹(CMD_CTRL)，指令会被拒绝，需要先让它进入悬停（比如撤回"
            "目标点，或等它到达终点自然悬停）再按一次")
    else:
        svc = f"/{ns}/mavros/set_mode"
        log(f"实际执行: ros2 service call {svc} mavros_msgs/srv/SetMode "
            "\"{custom_mode: 'AUTO.LAND'}\"")
        cp_ok, _ = _run(["docker", "cp", "scripts/ros2_env_setup.sh",
                          f"{container}:/tmp/ros2_env_setup.sh"], timeout=5)
        if not cp_ok:
            log(f"!! {ns} 容器不存在或没起来，降落口令没有下达 !!")
            return
        shell = (
            "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && "
            f"timeout 5 ros2 service call {svc} mavros_msgs/srv/SetMode "
            "\"{custom_mode: 'AUTO.LAND'}\""
        )
        ok, out = _run(["docker", "exec", container, "bash", "-c", shell], timeout=10)
        log(f"{ns} 降落口令{'已下达（AUTO.LAND）' if ok else ('下达失败: ' + out)}")


def do_lock_origin(ns, container, log):
    # 起飞点一键锁定——跟CONTROLLER/PLANNER/LOCALIZATION_SOURCE无关，
    # uwb_origin_bridge这个包无条件启动（见flight-stack-entrypoint.sh），
    # 2026-08-10实测踩坑修正：origin_setter_node里create_service('set_origin_
    # from_uwb', ...)传的是相对名——ROS2里节点内部的相对话题/服务名是相对
    # "节点所在namespace"解析，不是相对"namespace/节点名"，所以实际注册的
    # 服务是/{ns}/set_origin_from_uwb，不带中间的"origin_setter"这一节
    # （跟同一个包里takeoff_gate发布'takeoff_land'相对名解析成/{ns}/
    # takeoff_land是同一个规则，本该照抄这个已有先例，一开始想当然写成
    # "带节点名的路径"是错的）。之前这里写的/{ns}/origin_setter/
    # set_origin_from_uwb根本不存在，`ros2 service call`只能一直
    # "waiting for service to become available"直到被`timeout`杀掉，
    # 表现成"服务调用失败"，实际是URL写错，不是服务没启动/没注册成功。
    svc = f"/{ns}/set_origin_from_uwb"
    log(f"== {ns}: 请求锁定起飞点原点 ==")
    log(f"实际执行: ros2 service call {svc} std_srvs/srv/Trigger \"{{}}\"")
    cp_ok, _ = _run(["docker", "cp", "scripts/ros2_env_setup.sh",
                      f"{container}:/tmp/ros2_env_setup.sh"], timeout=5)
    if not cp_ok:
        log(f"!! {ns} 容器不存在或没起来，锁定原点请求没有下达 !!")
        return
    shell = (
        "source /opt/ros/humble/setup.bash && source /tmp/ros2_env_setup.sh && "
        f"timeout 5 ros2 service call {svc} std_srvs/srv/Trigger \"{{}}\""
    )
    ok, out = _run(["docker", "exec", container, "bash", "-c", shell], timeout=10)
    if not ok:
        log(f"!! {ns} 服务调用失败（超时/服务不存在）: {out} !!")
        return
    # service call本身能跑通不代表逻辑上锁定成功——Trigger.success字段才是真正
    # 结果（样本不足/抖动过大/里程计缺失都会success=false，见response.message）。
    log(out)


_clear_logs_armed_until = {"t": 0.0}


def do_clear_logs(_ns, _container, log):
    """清空runtime_logs/下的常规记录文件——rosbag滚动块、三个容器持久化的
    stdout、双机姿态/推力调试日志、health_status.txt、prune/record_rosbag.sh
    自己的stdout、incidents/health_events.log这个自动事件日志。

    **不会**删除incidents/下面按时间戳命名的子目录——那些是save_incident.sh
    手动触发留证时复制出来的独立副本，专门设计成不受滚动清理影响（见
    save_incident.sh文件头），清空记录文件这个按钮如果连这个也删了，就
    跟"留证"这个功能的设计初衷正面冲突，所以显式跳过，只清同一目录下那份
    自动追加写的health_events.log。

    破坏性操作，两次按键确认：第一次按只是"举起手"，5秒内再按一次才真的
    删；超时或按了别的键则自动放弃，不会残留一个"下次误触就删"的悬空状态。
    另外：如果record窗口的record_rosbag.sh正在录制，清空时会删掉它当前
    正在写的那个bag目录——`ros2 bag record`持有的文件描述符不会报错，
    但那一段(通常几分钟内)的数据实际上丢了，下一个滚动周期会创建全新目录、
    自动恢复正常，这是清空操作的预期代价，不是bug。

    ⚠️ 2026-08-12实测修正：一开始直接在宿主机侧用os.remove/shutil.rmtree
    删，几乎每个文件都"Permission denied"——runtime_logs/下这些文件全是
    容器内部的进程(record_rosbag.sh/attitude_thrust_logger.py/
    status_monitor.py等)以root身份写出来的(`docker exec ... whoami`实测
    确认容器默认就是root)，宿主机这边跑launch_control.py的是非root的
    `robots`用户，对这些root拥有的文件没有写权限，`ls -la runtime_logs/`
    能看到明显的属主差异。修复：不在宿主机直接删，改成`docker exec`进
    flight-stack-nx01容器（这个脚本里其它地方——ros2_env_setup.sh/
    status_monitor.py等——本来就用这个容器当"能操作共享/logs卷"的代表），
    用容器内部本来就有的root身份删，跟当初写出这些文件的是同一个用户，
    权限天然匹配。
    """
    now = time.monotonic()
    if now >= _clear_logs_armed_until["t"]:
        _clear_logs_armed_until["t"] = now + 5.0
        log("!! 再按一次[7]清空记录确认(5秒内有效，超时自动取消)——会删除rosbag/"
            "容器持久化日志/姿态推力调试日志/health_events.log，"
            "不会删除incidents/下手动留证的文件夹 !!")
        return
    _clear_logs_armed_until["t"] = 0.0

    container = "docker_sim-flight-stack-nx01-1"
    # nullglob：目录已经是空的时候，/logs/rosbag/*这类glob不展开成字面量
    # 字符串，避免rm对着一个不存在的"*"文件名报错刷屏。
    shell = (
        "shopt -s nullglob; "
        "rm -rf /logs/rosbag/* /logs/container_logs/* /logs/NX01/* /logs/NX02/*; "
        "rm -f /logs/health_status.txt /logs/prune_rosbag_stdout.log "
        "/logs/record_rosbag_stdout.log /logs/incidents/health_events.log; "
        "echo done"
    )
    log(f"实际执行: docker exec {container} bash -c '{shell}'（容器内以root身份删，"
        "宿主机侧对这些文件没有写权限）")
    ok, out = _run(["docker", "exec", container, "bash", "-c", shell], timeout=15)
    if ok:
        log("== 清空记录文件完成 ==")
    else:
        log(f"!! 清空记录文件失败: {out} !!")
    log("   保留：incidents/下手动留证的文件夹(save_incident.sh生成的)没有被清空")


BUTTON_W, BUTTON_H = 22, 3
BUTTON_GAP_X, BUTTON_GAP_Y = 4, 1
ORIGIN_Y, ORIGIN_X = 3, 2


def build_buttons():
    # 2026-08-12用户明确要求的布局：3行——第1行NX01(锁定原点/起飞/降落)，
    # 第2行NX02(锁定原点/起飞/降落)，第3行清空记录(单独一个)。之前是
    # 2列布局、顺序是起飞/降落/起飞/降落/锁定原点/锁定原点/清空记录，
    # 跟"先锁定原点再起飞"这个推荐操作顺序对不上，改成现在这版每行都是
    # 一架飞机自己的完整操作顺序。
    specs = [
        ("[1] NX01 锁定原点", "origin", do_lock_origin, *NAMESPACES[0]),
        ("[2] NX01 起飞", "takeoff", do_takeoff, *NAMESPACES[0]),
        ("[3] NX01 降落", "land", do_land, *NAMESPACES[0]),
        ("[4] NX02 锁定原点", "origin", do_lock_origin, *NAMESPACES[1]),
        ("[5] NX02 起飞", "takeoff", do_takeoff, *NAMESPACES[1]),
        ("[6] NX02 降落", "land", do_land, *NAMESPACES[1]),
        ("[7] 清空记录", "clear", do_clear_logs, None, None),
    ]
    buttons = []
    for i, (label, kind, fn, ns, container) in enumerate(specs):
        row, col = divmod(i, 3)
        y = ORIGIN_Y + row * (BUTTON_H + BUTTON_GAP_Y)
        x = ORIGIN_X + col * (BUTTON_W + BUTTON_GAP_X)
        key = str(i + 1)
        buttons.append(dict(label=label, kind=kind, fn=fn, ns=ns, container=container,
                             key=key, y0=y, y1=y + BUTTON_H - 1, x0=x, x1=x + BUTTON_W - 1))
    return buttons


def main(stdscr):
    curses.curs_set(0)
    curses.mousemask(curses.ALL_MOUSE_EVENTS)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_GREEN)   # 起飞按钮
    curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_RED)     # 降落按钮
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_YELLOW)  # 提示文字
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_CYAN)    # 锁定原点按钮
    curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_MAGENTA)  # 清空记录文件按钮

    buttons = build_buttons()
    log_lines = []
    # 2026-08-12用户要求：按钮反馈输出要能滚鼠标查看——log_scroll是"从底部
    # 往上数第几行开始显示"的偏移，0=贴底(跟着最新输出走，默认状态)，
    # 数值越大表示往上滑得越多。新日志追加时不主动把这个偏移清零/拉回
    # 底部——用户滑上去看历史的时候，突然被新输出弹回底部体验很差，标准
    # 终端/日志查看器都是"停在你滑到的位置，除非你自己滑回底部"这个行为，
    # 这里照抄。
    log_scroll = [0]

    def log(msg):
        for line in str(msg).splitlines() or [""]:
            log_lines.append(line)
        while len(log_lines) > 300:
            log_lines.pop(0)
        redraw()

    def redraw():
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        title = "起飞 / 降落 / 锁定原点 / 清空记录 控制台 —— 鼠标点按钮，或按数字键 1~7，Ctrl-C 退出"
        stdscr.addstr(0, 2, title[:max(0, w - 4)], curses.A_BOLD)
        stdscr.addstr(1, 2, ("起飞: TAKEOFF话题触发(px4ctrl/so3ctrl/pt4ctrl,可重复)/touch "
                              "takeoff_go(ros2_px4_stack,仅一次)；降落: LAND话题(px4ctrl/"
                              "so3ctrl/pt4ctrl)/mavros AUTO.LAND(ros2_px4_stack)；锁定原点: 起飞前"
                              "先做一次，把UWB绝对位置跟DLIO里程计的差值存成TF；清空记录文件:"
                              "需连按两次确认，不删incidents/下手动留证的文件夹")[:max(0, w - 4)])
        for b in buttons:
            if b["y1"] >= h or b["x1"] >= w:
                continue
            color = {"takeoff": curses.color_pair(1),
                     "land": curses.color_pair(2),
                     "origin": curses.color_pair(4),
                     "clear": curses.color_pair(5)}[b["kind"]]
            for yy in range(b["y0"], b["y1"] + 1):
                try:
                    stdscr.addstr(yy, b["x0"], " " * (b["x1"] - b["x0"] + 1), color)
                except curses.error:
                    pass
            label = b["label"]
            label_y = b["y0"] + (b["y1"] - b["y0"]) // 2
            label_x = b["x0"] + max(0, ((b["x1"] - b["x0"] + 1) - len(label)) // 2)
            try:
                stdscr.addstr(label_y, label_x, label, color | curses.A_BOLD)
            except curses.error:
                pass

        num_rows = max((b["y0"] - ORIGIN_Y) // (BUTTON_H + BUTTON_GAP_Y) for b in buttons) + 1
        log_top = ORIGIN_Y + num_rows * (BUTTON_H + BUTTON_GAP_Y) + 1
        avail = max(0, h - log_top - 2)
        # 滚轮偏移clamp到[0, 能滑的最大行数]——log_lines在300行上限里可能
        # 被pop(0)砍掉过（见log()注释），这里每次重绘都重新clamp一次，
        # 不会因为历史行数变化而指向越界的位置崩掉。
        max_scroll = max(0, len(log_lines) - avail)
        log_scroll[0] = max(0, min(log_scroll[0], max_scroll))
        sep_line = "-" * max(0, min(w - 4, w - 1))
        if log_scroll[0] > 0:
            hint = f" 已上滑{log_scroll[0]}行，滚轮下滚回到最新 "
            sep_line = (sep_line[:max(0, len(sep_line) - len(hint))] + hint)[:len(sep_line)] if sep_line else hint
        if log_top < h:
            try:
                stdscr.addstr(log_top, 2, sep_line)
            except curses.error:
                pass
        end = len(log_lines) - log_scroll[0]
        shown = log_lines[max(0, end - avail):end] if avail > 0 else []
        for idx, line in enumerate(shown):
            try:
                stdscr.addstr(log_top + 1 + idx, 2, line[:max(0, w - 4)])
            except curses.error:
                pass
        stdscr.refresh()

    def hit_test(y, x):
        for b in buttons:
            if b["y0"] <= y <= b["y1"] and b["x0"] <= x <= b["x1"]:
                return b
        return None

    def trigger(b):
        b["fn"](b["ns"], b["container"], log)

    redraw()
    log("就绪——点按钮或按数字键")

    while True:
        try:
            ch = stdscr.getch()
        except KeyboardInterrupt:
            break

        if ch == curses.KEY_RESIZE:
            redraw()
            continue
        if ch == curses.KEY_MOUSE:
            try:
                _, mx, my, _, bstate = curses.getmouse()
            except curses.error:
                continue
            if bstate & (curses.BUTTON1_CLICKED | curses.BUTTON1_PRESSED | curses.BUTTON1_RELEASED):
                b = hit_test(my, mx)
                if b:
                    trigger(b)
            # 滚轮上/下滚——ncurses把滚轮报成BUTTON4(上)/BUTTON5(下)，不是所有
            # 终端/ncurses构建都保证有这两个常量，getattr兜底成0（永远不会被
            # bstate按位与命中），没有滚轮支持时这两个分支就是死代码，不会报错。
            elif bstate & getattr(curses, 'BUTTON4_PRESSED', 0):
                log_scroll[0] += 3
                redraw()
            elif bstate & getattr(curses, 'BUTTON5_PRESSED', 0):
                log_scroll[0] = max(0, log_scroll[0] - 3)
                redraw()
            continue
        if ch in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5"), ord("6"), ord("7")):
            key = chr(ch)
            b = next((b for b in buttons if b["key"] == key), None)
            if b:
                trigger(b)
            continue
        if ch in (3, ord("q"), ord("Q")):  # Ctrl-C / q 退出
            break


if __name__ == "__main__":
    os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        pass
    print("起飞/降落控制台已退出。这个窗口可以留着，或切到别的窗口看飞行状态。")
