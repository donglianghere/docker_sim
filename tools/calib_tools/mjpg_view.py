#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MJPEG 拉流预览 + 按空格抓图

用法: python3 mjpg_view.py [url] [保存目录]
按键: 空格 = 保存当前帧   q / ESC = 退出

设计要点(为了快):
1) 原始 socket 直连取流, 不经 urllib/requests —— 本机 http_proxy 会把
   192.168.* 的请求转给 127.0.0.1:7897 而返回 502。
2) 收流线程只保留"最新一帧", 一次 recv 里的中间帧全丢, 画面不积压延迟。
3) 抓图直接把该帧的原始 JPEG 字节写盘, 不做 decode->encode, 无画质损失且几乎零开销。
"""
import os
import socket
import sys
import threading
import time
from datetime import datetime
from urllib.parse import urlparse

URL = "http://192.168.2.101:8080/stream.mjpg"
SAVE_DIR = os.path.expanduser("~/图片")
SOI, EOI = b"\xff\xd8", b"\xff\xd9"   # JPEG 起止标记
MAX_BUF = 8 << 20                     # 缓冲上限, 防脏数据把内存吃光


class MjpegReader(threading.Thread):
    """后台收流, 只持有最新一帧的原始 JPEG 字节。"""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.lock = threading.Lock()
        self.jpeg = None      # 最新帧原始字节
        self.seq = 0          # 帧序号, 主线程据此判断有无新帧
        self.err = None       # 最近一次错误信息
        self.stopped = False

    def _connect(self):
        u = urlparse(self.url)
        host, port = u.hostname, u.port or 80
        path = (u.path or "/") + (("?" + u.query) if u.query else "")
        s = socket.create_connection((host, port), timeout=5)
        s.settimeout(5)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.sendall("GET {} HTTP/1.0\r\nHost: {}:{}\r\nAccept: */*\r\n\r\n"
                  .format(path, host, port).encode())
        return s

    def run(self):
        while not self.stopped:
            s = None
            try:
                s = self._connect()
                buf = bytearray()
                while b"\r\n\r\n" not in buf:          # 吃掉响应头
                    d = s.recv(4096)
                    if not d:
                        raise ConnectionError("连接被对端关闭")
                    buf += d
                head, _, rest = bytes(buf).partition(b"\r\n\r\n")
                status = head.split(b"\r\n")[0]
                if b" 200" not in status:
                    raise ConnectionError(status.decode("latin1"))
                buf = bytearray(rest)
                self.err = None

                while not self.stopped:
                    d = s.recv(65536)
                    if not d:
                        raise ConnectionError("流中断")
                    buf += d
                    end = buf.rfind(EOI)               # 本批里最后一个完整帧
                    if end < 0:
                        if len(buf) > MAX_BUF:
                            del buf[:-(1 << 20)]
                        continue
                    start = buf.rfind(SOI, 0, end)
                    if start < 0:
                        del buf[:end + 2]
                        continue
                    frame = bytes(buf[start:end + 2])
                    del buf[:end + 2]
                    with self.lock:
                        self.jpeg = frame
                        self.seq += 1
            except Exception as e:
                self.err = "{}: {}".format(type(e).__name__, e)
                if not self.stopped:
                    time.sleep(1.0)                    # 掉线后自动重连
            finally:
                if s is not None:
                    try:
                        s.close()
                    except OSError:
                        pass

    def latest(self):
        with self.lock:
            return self.seq, self.jpeg

    def stop(self):
        self.stopped = True


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else URL
    save_dir = os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2 else SAVE_DIR
    os.makedirs(save_dir, exist_ok=True)
    sys.stdout.reconfigure(line_buffering=True)  # 后台/重定向运行时也能即时看到抓图提示

    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("缺少依赖, 先装: pip3 install opencv-python numpy")

    reader = MjpegReader(url)
    reader.start()
    print("拉流:", url)
    print("存图:", save_dir)
    print("按 空格 抓图, q/ESC 退出")

    win = "MJPEG  [SPACE]=capture  [q]=quit"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    last_seq, cur_jpeg, img = -1, None, None
    shots, fps, n, t0 = 0, 0.0, 0, time.time()
    warned = ""

    try:
        while True:
            seq, jpeg = reader.latest()
            if jpeg is not None and seq != last_seq:
                frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    last_seq, cur_jpeg, img = seq, jpeg, frame
                    n += 1
                    dt = time.time() - t0
                    if dt >= 0.5:
                        fps, n, t0 = n / dt, 0, time.time()

            if img is not None:
                show = img              # 只在角上叠字, 不复制整帧
                cv2.putText(show, "{:.1f} fps  shots:{}".format(fps, shots),
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(win, show)
            elif reader.err and reader.err != warned:
                warned = reader.err
                print("[等待流]", reader.err)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == 32 and cur_jpeg is not None:          # 空格抓拍
                name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
                path = os.path.join(save_dir, name)
                with open(path, "wb") as f:            # 原始字节直写, 不重编码
                    f.write(cur_jpeg)
                shots += 1
                print("已保存:", path)

            try:      # 窗口被点掉后 Qt 后端会抛 NULL guiReceiver, 当成正常退出
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        cv2.destroyAllWindows()
        print("退出, 共保存 {} 张".format(shots))


if __name__ == "__main__":
    main()
