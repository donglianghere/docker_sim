#!/usr/bin/env bash
# 两个 ssh 窗口，演示并自证两台机载计算机直连互发数据（纯网络验证，不碰项目）
#   ~/uav_chat.sh          双向对聊：哪个窗口打字，另一个窗口出现（带发送方 IP 署名）
#   ~/uav_chat.sh auto     NX02(.102) 每秒自动发，NX01(.101) 窗口滚动显示
#   ~/uav_chat.sh rauto    NX01(.101) 每秒自动发，NX02(.102) 窗口滚动显示
#   ~/uav_chat.sh kill     清掉两机上残留的 nc / 发包循环（含强制释放端口）
#
# 窗口归属靠四重证明，全部由远端机器自己产出，本脚本无法代笔：
#   ①身份块 ：该机内核/固件给出的 网卡 IP、MAC、Jetson 序列号、本条 ssh 的来源
#   ②nc -vn ：netcat 自己打印 "Connection received on <对端IP> <端口>"
#   ③ss 行  ：内核视角的 TCP 连接两端 IP —— 这条连接在 .101 与 .102 之间
#   ④报文署名：每行字由发送机自己打上 [发送方IP]+MAC，对面看到的内容自带来源
set -u
MODE=${1:-chat}
PORT=${2:-9300}
IP1=192.168.2.101
IP2=192.168.2.102
U=nvidia
SSH="ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=6"

# ---------- 在飞机上执行的脚本（唯一一份，base64 送过去，躲开引号地狱）----------
REMOTE=$(cat <<'RSH'
#!/bin/bash
ROLE=$1; PEER=$2; PORT=$3; MODE=$4
W=$(ls /sys/class/net | grep -E '^wl' | head -1)
IP=$(ip -4 -o addr show dev "$W" 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
MAC=$(cat /sys/class/net/$W/address 2>/dev/null)
SN=$(tr -d '\0' < /proc/device-tree/serial-number 2>/dev/null)
printf '\033]0;本机 %s  <->  对端 %s\007' "$IP" "$PEER"
echo "┌──────────── 本机自报身份（以下全部由本机内核/固件输出）────────────"
echo "│  无线网卡      : $W"
echo "│  本机 IP       : $IP        <<< 本窗口属于哪架飞机"
echo "│  网卡 MAC      : $MAC"
echo "│  Jetson 序列号 : $SN"
echo "│  本条 ssh 来自 : ${SSH_CONNECTION%% *}   (地面站)"
echo "│  hostname      : $(hostname)   <- 两架同名"
echo "└────────────────────────────────────────────────────────────────────"
# 残留的 nc 会抢走端口和数据，让新窗口静默收不到东西，先清掉
if [ "$ROLE" = listen ] && sudo -n fuser -k ${PORT}/tcp 2>/dev/null; then
  echo "[清理] 端口 $PORT 上有残留进程，已杀掉"; sleep 1
fi
# 连接建成后，打印内核视角的连接两端
( for i in $(seq 15); do
    L=$(ss -tn 2>/dev/null | grep ":$PORT " | grep ESTAB | head -1)
    if [ -n "$L" ]; then
      echo; echo "[内核 ss 视角] $L"
      echo "   ^ 本窗口这条 nc 连接的本端/对端：$IP <-> $PEER，数据不经地面站"; echo
      break
    fi
    sleep 1
  done ) &
SIG="[$IP]"
case "$MODE" in
  chat)
    echo ">>> 在本窗口打字回车：对面会显示「收到 <- $SIG 你的话」"
    echo ">>> 对面发来的，本窗口显示为「收到 <- [对方IP] ...」"; echo
    if [ "$ROLE" = listen ]; then
      sed -u "s|^|$SIG |" | stdbuf -oL nc -vn -l "$PORT" | sed -u 's|^|收到 <- |'
    else
      sed -u "s|^|$SIG |" | stdbuf -oL nc -vn "$PEER" "$PORT" | sed -u 's|^|收到 <- |'
    fi
    ;;
  auto-send)
    echo ">>> 本机每秒往 $PEER:$PORT 发一条，署名用的是本机 IP 和 MAC"; echo
    while :; do
      i=0
      while :; do
        i=$((i+1))
        echo "$SIG #$i $(date +%H:%M:%S.%3N) mac=$MAC sn=${SN:0:8}" || break
        sleep 1
      done | stdbuf -oL nc -vn "$PEER" "$PORT"
      echo "[连接断开，2 秒后重连]"; sleep 2
    done
    ;;
  auto-recv)
    echo ">>> 本机($IP)在 $PORT 监听，下面每行都是对面发来的，署名是发送方的 IP/MAC"; echo
    stdbuf -oL nc -vn -lk "$PORT" | while read -r l; do
      echo "收到 <- $l   | 本机 $IP 于 $(date +%H:%M:%S.%3N) 读到"
    done
    ;;
esac
RSH
)
B64=$(printf '%s' "$REMOTE" | base64 -w0)

clean() {   # 方括号防止 pkill 匹配到自己这条命令
  for ip in $IP1 $IP2; do
    timeout 12 $SSH $U@$ip \
      "pkill -f '[u]av_chat_remote'; pkill -f '[n]c -v'; sudo -n fuser -k ${PORT}/tcp; true" \
      >/dev/null 2>&1
  done
}

win() {   # win <标题> <机器IP> <角色> <对端IP> <模式> <几何>
  gnome-terminal --title="$1" --geometry="$6" -- bash -c \
    "ssh -t -o StrictHostKeyChecking=no $U@$2 \
       'echo $B64 | base64 -d > /tmp/uav_chat_remote.sh; bash /tmp/uav_chat_remote.sh $3 $4 $PORT $5'; \
     printf '\n[连接已断开，回车关闭本窗口]'; read" &
}

clean
case "$MODE" in
  kill) echo "已清理两机上的 nc / 发包循环，端口 $PORT 已释放"; exit 0 ;;
  chat)
    win "窗口A → 该窗口应自报 192.168.2.101" $IP1 listen  $IP2 chat "96x24+60+40"
    sleep 2
    win "窗口B → 该窗口应自报 192.168.2.102" $IP2 connect $IP1 chat "96x24+60+500"
    echo "已开两个窗口。先核对各窗口开头自报的『本机 IP』，再在任一窗口打字回车看对面。"
    ;;
  auto)
    win "窗口A → 该窗口应自报 192.168.2.101（收）" $IP1 listen  $IP2 auto-recv "96x24+60+40"
    sleep 2
    win "窗口B → 该窗口应自报 192.168.2.102（发）" $IP2 connect $IP1 auto-send "96x24+60+500"
    echo "已开两个窗口：B 自动发、A 显示收到。A 里每行署名的 IP/MAC 应等于 B 自报的 IP/MAC。"
    ;;
  rauto)
    win "窗口A → 该窗口应自报 192.168.2.102（收）" $IP2 listen  $IP1 auto-recv "96x24+60+40"
    sleep 2
    win "窗口B → 该窗口应自报 192.168.2.101（发）" $IP1 connect $IP2 auto-send "96x24+60+500"
    echo "已开两个窗口：.101 自动发、.102 显示收到。"
    ;;
  *) echo "用法: $0 [chat|auto|rauto|kill] [端口]"; exit 1 ;;
esac

sleep 5
echo "--- 地面站侧独立核对：101 内核里这条连接的对端是谁 ---"
timeout 12 $SSH $U@$IP1 "ss -tn | grep ':$PORT ' || echo '（还没建立，稍等几秒再看窗口里的 ss 行）'"
