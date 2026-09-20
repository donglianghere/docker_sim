#!/usr/bin/env python3
"""RTP+H.264(UDP推流) -> 本机GStreamer解码转JPEG -> MJPEG over HTTP。

背景：GCS网页地面站"俯视相机"标签页已经能直接播一路MJPEG(<img
src="http://.../stream.mjpg">，浏览器原生支持multipart/x-mixed-replace)。
2026-08-22又接入了第二路视频——RTP+H.264硬编码，从192.168.43.7:5000
(这台开发机自己的局域网IP+用户实测验证过的端口)推过来。浏览器没有任何
API能直接播放原始RTP包(不像MJPEG，RTP连HTTP都不是，是裸UDP+RTP协议头)，
必须先转码成浏览器能播的格式——这个脚本就是那道转码桥梁，不是可选项。

用法：
  python3 scripts/rtp_h264_to_mjpeg.py [--udp-port 5000] [--http-port 8081]
跑起来之后GCS网页"俯视相机"标签页的MJPEG推流地址填
http://192.168.43.7:8081/stream.mjpg（跟这台机器上跑gst-launch测试时
用的是同一个UDP端口，HTTP吐出的端口默认8081，避开已经在用的8080/8000）。

依赖：这台机器上的gst-launch-1.0（2026-08-22确认已装，1.24.2）+
jpegenc/multipartmux/avdec_h264这几个GStreamer元素（已确认装了）。这个
脚本本身只用Python标准库，不需要额外pip安装。

实现思路：gst-launch子进程把RTP流解码、转JPEG之后，逐帧写到自己的
stdout（每帧是一个完整的JPEG，首尾各有标准的SOI(0xFFD8)/EOI(0xFFD9)
标记，不需要gst额外加分隔符，直接从stdout里找这两个marker切帧）；这个
脚本自己起一个最简单的HTTP服务器，把最新一帧包成multipart/x-mixed-
replace格式吐给任意数量的浏览器连接——跟GCS网页现有那路MJPEG摄像头(用
`curl -sI`探测过是`BaseHTTP/0.6 Python/3.10.12`)显然是同一种实现思路，
不是另起一套新协议。

⚠️只处理"收到一路流、一直转发"这一件事，没有做"gst子进程挂了自动重启"
这类容错——如果RTP源断流/gst-launch本身崩了，这个脚本会打日志说
"多久没收到新帧了"但不会自己重启子进程，需要的话手动重跑这个脚本。
没有做，是因为不确定这个环境下"断流"和"该重启"的判断标准该怎么定
（正常网络抖动 vs 真的需要重启），宁可不做、留给用户观察到问题后手动
处理，也不做一个可能误判、把还在正常工作的流强行重启的自动机制。
"""
import argparse
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "frame"
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


def build_gst_cmd(udp_port, jpeg_quality):
    return [
        "gst-launch-1.0", "-q",
        "udpsrc", f"port={udp_port}",
        "caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000",
        "!", "rtph264depay",
        "!", "h264parse",
        "!", "avdec_h264",
        "!", "videoconvert",
        "!", "jpegenc", f"quality={jpeg_quality}",
        "!", "filesink", "location=/dev/stdout",
    ]


class FrameBus:
    """持有"最新一帧"，多个HTTP连接共用同一份解码结果，不用各自起一条
    gst管线——一路RTP源只解一次，不管有几个浏览器标签页在看。"""

    def __init__(self):
        self._frame = None
        self._seq = 0
        self._cond = threading.Condition()
        self.last_frame_time = 0.0

    def publish(self, frame: bytes):
        with self._cond:
            self._frame = frame
            self._seq += 1
            self.last_frame_time = time.monotonic()
            self._cond.notify_all()

    def wait_next(self, last_seq, timeout=5.0):
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout=timeout)
            return self._frame, self._seq


def reader_thread(bus: FrameBus, udp_port, jpeg_quality):
    cmd = build_gst_cmd(udp_port, jpeg_quality)
    print(f"[rtp_h264_to_mjpeg] 启动: {' '.join(cmd)}", file=sys.stderr)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    buf = b""
    frame_count = 0
    while True:
        chunk = proc.stdout.read(65536)
        if not chunk:
            print("[rtp_h264_to_mjpeg] gst-launch子进程stdout关闭了(流断了或进程退出)，"
                  "这个脚本不会自动重启子进程，需要的话手动重跑", file=sys.stderr)
            return
        buf += chunk
        while True:
            start = buf.find(JPEG_SOI)
            if start == -1:
                buf = b""  # 缓冲区里没有任何帧起始标记，清空避免无限增长
                break
            end = buf.find(JPEG_EOI, start + 2)
            if end == -1:
                if start > 0:
                    buf = buf[start:]  # 丢掉起始标记之前的垃圾字节，保留可能还没收全的这一帧
                break
            frame = buf[start:end + 2]
            buf = buf[end + 2:]
            bus.publish(frame)
            frame_count += 1
            if frame_count % 100 == 0:
                print(f"[rtp_h264_to_mjpeg] 已转发{frame_count}帧", file=sys.stderr)


def make_handler(bus: FrameBus):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path not in ("/", "/stream.mjpg"):
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            seq = 0
            try:
                while True:
                    frame, seq = bus.wait_next(seq)
                    if frame is None:
                        continue
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # 浏览器关掉标签页/切走了，正常情况，不用当错误处理

        def log_message(self, fmt, *args):
            pass  # BaseHTTPRequestHandler默认每个请求都打一行日志，MJPEG是长连接，不需要

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--udp-port", type=int, default=5000, help="接收RTP的UDP端口，默认5000")
    ap.add_argument("--http-port", type=int, default=8081, help="对外提供MJPEG的HTTP端口，默认8081")
    ap.add_argument("--jpeg-quality", type=int, default=80, help="JPEG编码质量(1-100)，默认80")
    args = ap.parse_args()

    bus = FrameBus()
    t = threading.Thread(target=reader_thread, args=(bus, args.udp_port, args.jpeg_quality), daemon=True)
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.http_port), make_handler(bus))
    print(f"[rtp_h264_to_mjpeg] MJPEG服务已启动: http://0.0.0.0:{args.http_port}/stream.mjpg"
          f"（正在监听udp:{args.udp_port}等RTP+H.264流）", file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
