#!/usr/bin/env bash
# 被 contestant_sim/run.sh、contestant_real/run_real.sh、run_test.sh **source**
# 的共享函数：把"飞行必须在跑"的共享栈拉起来。不是独立入口，别直接跑。
#
# 为什么要有这个（2026-10-03 用户要求）：声光栈和网页栈都不属于任何一个运行器
# 自己的容器，原来 check_env.sh 对它们只给个 △ 提示、不拦——于是可以一声不响
# 飞完整轮，事后才发现任务通报全缺。改成由运行器**必起**。
#
# 调用方约定：调用前必须已经有 $ROOT（docker_sim 仓库根）和 log() / die()。
#
# 两个函数都**幂等**：已经是对的就什么都不做，不会去重启一个正在服务的容器。

# 声光反馈装置的串口是否就位。缺了就问选手：忽略（本轮没有声光）还是退出去修。
# 选"忽略"会 export SOUND_LIGHT_IGNORE=1，下面 ensure_sound_light 也跟着不致命。
check_sound_light_port() {
    local port="${SOUND_LIGHT_PORT:-/dev/ttyUSB0}" ans why
    if [ -c "$port" ] && [ -r "$port" ] && [ -w "$port" ]; then
        log "声光串口 $port 就位（端口在不等于板子通电，不响先查供电）"
        return 0
    fi
    if [ -e "$port" ]; then
        why="存在但当前用户不能读写（不在 dialout 组？）"
    else
        why="设备不存在（板子没插、USB 线松、或被 brltty 抢了）"
    fi
    echo "!! 声光串口 $port 不可用：$why" >&2
    if systemctl is-active --quiet brltty 2>/dev/null; then
        echo "   brltty 正在跑，它会抢 CH340：sudo systemctl stop brltty" >&2
    fi
    echo "   忽略的后果：全程没有声光播报，而程序一切正常（静默失效）。" >&2
    if [ "${SOUND_LIGHT_IGNORE:-0}" = 1 ]; then
        log "SOUND_LIGHT_IGNORE=1：已跳过声光串口检查"
        return 0
    fi
    # 非交互（CI / nohup）时不卡住：默认按"必须修"处理。
    [ -t 0 ] || die "声光串口不可用，且不是交互运行；修好，或带 SOUND_LIGHT_IGNORE=1 重跑"
    ans=""
    read -r -p "   忽略这一项继续飞行？[y=继续 / 其它=退出去修]： " ans || true
    case "$ans" in
        y|Y|yes|YES) export SOUND_LIGHT_IGNORE=1
                     log "已选择忽略声光：本轮没有声光播报" ;;
        *) die "请先修好声光串口再飞（要忽略就重跑并输入 y）" ;;
    esac
}

# 声光栈：确保在跑、而且在对的域（仿真21 / 真机20）。
# 域不对必须重起——常驻程序只能在一个域，这是共享资源的本质限制。
ensure_sound_light() {   # ensure_sound_light sim|real
    local mode="$1" c=contestant-sound-light want d out
    [ "$mode" = real ] && want=20 || want=21
    local SH="$ROOT/shell" PORT="${SOUND_LIGHT_PORT:-/dev/ttyUSB0}"
    local REAL_FLAG=(); [ "$mode" = real ] && REAL_FLAG=(--real)

    if docker ps --format '{{.Names}}' | grep -qx "$c"; then
        d="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$c" 2>/dev/null \
             | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)"
        if [ "$d" = "$want" ]; then
            # 域号对了还要比**网卡集合有没有变**。device 模式按"当时有没有连上
            # 飞机网段"决定挂不挂 LAN 那张网卡；地面站接回 uav 之后不重起，容器
            # 就一直只在 lo 上，真飞时选手容器（real 模式、钉在 LAN 上）找不到
            # 它 —— 表现是"测试能响、真飞没声"，很难查。
            local cur="$HOME/.cache/contest_sdk/cyclonedds_device.xml"
            local started="$HOME/.cache/contest_sdk/cyclonedds_device.started.xml"
            if [ "$mode" = real ]; then
                ( source "$ROOT/scripts/contestant_network.sh"
                  contestant_net_args device "$ROOT" "$HOME/.cache/contest_sdk" >/dev/null 2>&1 ) || true
            fi
            if [ "$mode" = real ] && [ -f "$cur" ] && [ -f "$started" ] \
               && ! cmp -s "$cur" "$started"; then
                log "声光栈的网卡集合变了（地面站换过网？）—— 停掉重起，否则真飞时选手容器找不到它"
            else
                log "声光栈已在 $mode 域（$want），不动"
                return 0
            fi
        fi
        log "声光栈在域 $d，$mode 模式要 $want —— 停掉重起"
        # 喂一个**换行**不能省，而且不能用 `< /dev/null`：这两个脚本是给双击
        # 用的，末尾有 `read -n 1 ... 按任意键关闭窗口` 的 EXIT trap。不喂会
        # 卡住整个运行器；喂 /dev/null 则 read 拿到 EOF 返回 1，在 set -e 下
        # 把 trap 掐断、脚本退出码变成 1——**明明已经停掉了却报"失败"**
        # （2026-10-03 实测踩到）。给个换行让 read 正常返回 0。
        out="$(printf '\n' | "$SH/stop_sound_light_server.sh" 2>&1)" \
            || { echo "$out" >&2; die "停声光栈失败"; }
    fi

    log "起声光栈（$mode 域 $want，串口 $PORT）"
    if ! out="$(printf '\n' | "$SH/start_sound_light_server.sh" "${REAL_FLAG[@]}" "$PORT" 2>&1)"; then
        echo "$out" >&2
        [ "${SOUND_LIGHT_IGNORE:-0}" = 1 ] \
            && { echo "（起声光栈失败，已按选手选择忽略，本轮没有声光）" >&2; return 0; }
        die "起声光栈失败——串口被 brltty 抢了？试 sudo systemctl stop brltty"
    fi

    d="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$c" 2>/dev/null \
         | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)"
    if [ "$d" != "$want" ]; then
        [ "${SOUND_LIGHT_IGNORE:-0}" = 1 ] \
            && { echo "（声光栈域是 $d、期望 $want，已按选手选择忽略）" >&2; return 0; }
        die "声光栈起来了但域是 $d，期望 $want"
    fi
    # 串口没打开就等于没有声光：装置不响，但程序一切正常，属于静默失效
    echo "$out" | grep -q "已打开" \
        || echo "（声光串口还没打开，常驻程序会每3秒重试——板子插好了吗？）" >&2
}

# 网页栈：gcs-gcs-1 + gcs-backend-1。只在真机侧必起（仿真侧用户明确说不管）。
# 交给 scripts/up_gcs.sh——它幂等，而且比直接 compose up 多做 xhost 授权、
# gcs/.env 校验、三个 json 从 .example 补齐、起完验 backend /healthz。
ensure_gcs_web() {
    if docker ps --format '{{.Names}}' | grep -qx gcs-gcs-1 \
       && docker ps --format '{{.Names}}' | grep -qx gcs-backend-1; then
        log "网页栈已在跑，不动"
        return 0
    fi
    log "起网页栈（gcs-gcs-1 + gcs-backend-1）"
    local out
    out="$("$ROOT/scripts/up_gcs.sh" 2>&1)" || { echo "$out" >&2; die "起网页栈失败"; }
    log "网页栈就绪： http://localhost:8080"
}
