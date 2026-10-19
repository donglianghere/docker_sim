#!/usr/bin/env python3
"""两台机载计算机点对点互发数据的实况面板（纯链路验证，与无人机业务无关）。

      NX01 192.168.2.101   ⇄   NX02 192.168.2.102

每台机载机每秒给对方发一条自带署名的报文：  NX01->NX02 #42 <发出时刻>
面板同时盯着一条报文的两端：
  · 发送侧：该机确实把第 42 包写进了到对方的 TCP 连接
  · 接收侧：对方确实读到了第 42 包，报文里的署名是谁、晚了多少毫秒
两侧序号对得上 = 这一方向真的把数据送到了对面。另外附 ICMP 双向回声。

用法:  python3 uav_link_view.py [--seconds N]
       Ctrl-C 退出（会清掉两机上本脚本起的 ping/nc）
"""
import argparse
import re
import subprocess
import sys
import threading
import time

USER = "nvidia"
A = ("NX01", "192.168.2.101")
B = ("NX02", "192.168.2.102")
PORT = {"NX01": 9101, "NX02": 9102}   # 各机的监听端口，对方往这里发
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6",
       "-o", "ServerAliveInterval=5", "-o", "StrictHostKeyChecking=no"]

stop = threading.Event()
lock = threading.Lock()
st = {}
procs = []
PING_RE = re.compile(r"icmp_seq=(\d+).*time=([\d.]+) ms")
MSG_RE = re.compile(r"(NX\d+)->(NX\d+) #(\d+) ([\d.]+)")


def slot(k):
    return st.setdefault(k, {"seq": 0, "n": 0, "t": 0.0, "raw": "", "lat": None,
                             "sender": "", "rtt": None, "sent": 0, "up": False})


def spawn(host, cmd, merge=False):
    p = subprocess.Popen(
        SSH + ["%s@%s" % (USER, host), cmd], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT if merge else subprocess.DEVNULL,
        text=True, bufsize=1)
    procs.append(p)
    return p


def sender(src, dst, dst_ip, port):
    """在 src 机上每秒往 dst 发一条署名报文；写入成功后回显给地面站。"""
    cmd = ("while :; do i=0; while :; do i=$((i+1)); "
           "m=\"%s->%s #$i $(date +%%s.%%3N)\"; echo \"$m\" || break; "
           "echo \"$m\" >&2; sleep 1; done | nc %s %d; sleep 2; done"
           % (src, dst, dst_ip, port))
    key = "send_%s" % src
    while not stop.is_set():
        p = spawn(src_ip[src], cmd, merge=True)
        for line in p.stdout:
            if stop.is_set():
                break
            m = MSG_RE.search(line)
            if not m:
                continue
            with lock:
                s = slot(key)
                s["seq"] = int(m.group(3))
                s["n"] += 1
                s["t"] = time.time()
                s["raw"] = m.group(0)
                s["up"] = True
        with lock:
            slot(key)["up"] = False
        time.sleep(1)


def receiver(host_name, port):
    """在 host 上监听，读到的每条报文前面盖上本机到达时刻。"""
    cmd = ("nc -lk %d | while read -r l; do echo \"$(date +%%s.%%3N) $l\"; done"
           % port)
    key = "recv_%s" % host_name
    while not stop.is_set():
        p = spawn(src_ip[host_name], cmd)
        for line in p.stdout:
            if stop.is_set():
                break
            m = MSG_RE.search(line)
            if not m:
                continue
            with lock:
                s = slot(key)
                s["sender"] = m.group(1)
                s["seq"] = int(m.group(3))
                s["n"] += 1
                s["t"] = time.time()
                s["raw"] = m.group(0)
                s["up"] = True
                try:
                    s["lat"] = (float(line.split()[0]) - float(m.group(4))) * 1000.0
                except ValueError:
                    pass
        with lock:
            slot(key)["up"] = False
        time.sleep(1)


def pinger(src, dst_ip):
    key = "icmp_%s" % src
    while not stop.is_set():
        p = spawn(src_ip[src], "ping -i 0.5 -O %s" % dst_ip)
        with lock:
            slot(key).update(seq=0, n=0)
        for line in p.stdout:
            if stop.is_set():
                break
            m = PING_RE.search(line)
            with lock:
                s = slot(key)
                if m:
                    s["seq"] = int(m.group(1))
                    s["n"] += 1
                    s["rtt"] = float(m.group(2))
                    s["t"] = time.time()
                    s["up"] = True
                elif "no answer" in line:
                    s["seq"] += 1
                    s["up"] = False
        time.sleep(2)


G, R, Y, C, BD, Z = "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[1m", "\033[0m"


def lamp(s, fresh=3.0):
    return G + "●" + Z if s["up"] and time.time() - s["t"] < fresh else R + "○" + Z


def ago(t):
    return "%4.1fs前" % (time.time() - t) if t else "  --  "


def direction(src, dst, out):
    """渲染一个方向的收发两端。"""
    snd, rcv = slot("send_%s" % src), slot("recv_%s" % dst)
    ok = snd["seq"] and abs(snd["seq"] - rcv["seq"]) <= 1 and rcv["sender"] == src
    verdict = (G + "✓ 对方确实收到了 %s 发的数据" % src + Z) if ok else \
              (Y + "… 等待对上号" + Z)
    out.append("  %s%s(%s)  ──数据──▶  %s(%s)%s      TCP 端口 %d" %
               (BD, src, src_ip[src], dst, src_ip[dst], Z, PORT[dst]))
    out.append("     %s %s 发出   第 %4d 包   共 %4d 条   %s   报文 \"%s\"" %
               (lamp(snd), src, snd["seq"], snd["n"], ago(snd["t"]), snd["raw"] or "-"))
    out.append("     %s %s 收到   第 %4d 包   共 %4d 条   %s   署名 %s%s%s  晚 %s" %
               (lamp(rcv), dst, rcv["seq"], rcv["n"], ago(rcv["t"]),
                C, rcv["sender"] or "?", Z,
                "%5.1f ms" % rcv["lat"] if rcv["lat"] is not None else "  --  "))
    out.append("     对账     发 %d / 收 %d   %s" % (snd["seq"], rcv["seq"], verdict))
    out.append("")


def render(t0):
    out = ["\033[H\033[J"]
    out.append("%s  两台机载计算机点对点互发数据%s        %s   已运行 %ds\n" %
               (BD, Z, time.strftime("%H:%M:%S"), time.time() - t0))
    out.append("  " + "═" * 84 + "\n")
    with lock:
        direction(A[0], B[0], out)
        direction(B[0], A[0], out)
        out.append("  " + "─" * 84)
        for src, dst in ((A[0], B[0]), (B[0], A[0])):
            s = slot("icmp_%s" % src)
            loss = (1 - s["n"] / s["seq"]) * 100 if s["seq"] else 0.0
            out.append("  %s ICMP 回声  %s → %s   往返 %s   收 %4d/%-4d  丢包 %5.1f%%" %
                       (lamp(s), src, dst,
                        "%6.2f ms" % s["rtt"] if s["rtt"] is not None else "   --  ",
                        s["n"], s["seq"], loss))
    out.append("\n  两个方向都显示 %s✓%s 即双向互通：报文署名说明谁发的，"
               "序号对上说明对面真收到了。Ctrl-C 退出。" % (G, Z))
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


def cleanup():
    stop.set()
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass
    # 方括号防止 pkill 匹配到自己这条命令
    kill = "pkill -f '[p]ing -i 0.5 -O'; pkill -f '[n]c -lk 91'; pkill -f '[N]X0.->NX0.'; true"
    for name, host in (A, B):
        subprocess.run(SSH + ["%s@%s" % (USER, host), kill],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)


src_ip = {A[0]: A[1], B[0]: B[1]}


def preclean():
    """残留的 nc 会占着端口却不读数据，让新面板静默收不到东西。"""
    for name, host in (A, B):
        subprocess.run(SSH + ["%s@%s" % (USER, host),
                              "pkill -f '[n]c -lk 91'; sudo -n fuser -k %d/tcp; true" % PORT[name]],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0, help="跑够 N 秒自动退出（0=一直跑）")
    args = ap.parse_args()

    preclean()
    jobs = [(receiver, (A[0], PORT[A[0]])), (receiver, (B[0], PORT[B[0]])),
            (sender, (A[0], B[0], B[1], PORT[B[0]])),
            (sender, (B[0], A[0], A[1], PORT[A[0]])),
            (pinger, (A[0], B[1])), (pinger, (B[0], A[1]))]
    for fn, a in jobs:
        threading.Thread(target=fn, args=a, daemon=True).start()

    t0 = time.time()
    try:
        while not stop.is_set():
            render(t0)
            if args.seconds and time.time() - t0 >= args.seconds:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n  收尾中…\n")
        sys.stdout.flush()
        cleanup()


if __name__ == "__main__":
    main()
