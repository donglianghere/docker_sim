"""极简MJPEG-over-HTTP服务器，浏览器原生支持(<img src="/stream.mjpg">)，
不需要WebRTC信令/ICE那一套。

2026-09-09从真机`docker_real/docker_sim/src/vision_stack/vision_stack/
mjpeg_server.py`原样搬过来（一个字节没改）——用户要求仿真侧拉流"与真机
处理模式类似"，这个文件本身跟ROS/相机取流方式完全无关（纯`cv2`+
标准库），可以直接复用，不用重写一遍。真机那份如果以后有修复/改进，
记得回头同步这份拷贝。调用方是`qr_apriltag_detect_node.py`（每路相机
起一个独立端口的`MjpegServer`实例），用法照抄真机`qr_detect_node.py`/
`yolo_detector_node.py`那套"`declare_parameter('mjpeg_port', ...)` +
`update_frame(preview)`"的调用方式。

2026-08-25起改成地面站看画面的正式方案（不再只是"本地快速验证"用途）：
原来的nvv4l2h264enc硬编码+RTP推流已经整个删掉——`udpsink`往不可达地址
推流会阻塞、连累同一相机的appsink分支(YOLO/QR)一起停摆(read_hw.md
2026-08-22"阶段7")，两路相机接上后这个风险会翻倍；WebRTC(WHEP)也已经在
2026-08-24因为这台环境DTLS握手兼容性问题放弃。MJPEG是拉流——地面站不主动
连接就不会发包，天然规避"推流目标不可达连累检测"这整类问题，代价是用CPU
软编码JPEG，不像硬编码RTP几乎不占CPU。

设计：跟YOLO/AprilTag等检测节点共用同一份已经从相机读到的帧（不新开
GstCameraCapture/不新开Argus会话），每次推理循环把最新一帧（画好检测框）
写进一个线程安全的共享缓冲区，HTTP服务器用独立线程从缓冲区拉取编码JPEG
发出去，两边靠一把锁同步，互不阻塞对方。

2026-08-22修正：第一版`get()`是纯轮询、没有节流，HTTP线程会在同一帧没
更新时也拼命重发，白白浪费带宽/CPU。改用`threading.Condition`——推理
循环写入新帧时`notify_all()`，HTTP线程`wait_for_new()`阻塞到真的有新帧
才发送，避免busy loop。
"""
import http.server
import socketserver
import threading

import cv2

_BOUNDARY = "frame"


class _LatestFrameBuffer:
    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg_bytes = None
        self._version = 0

    def update(self, frame, jpeg_quality: int = 80):
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            return
        with self._cond:
            self._jpeg_bytes = jpg.tobytes()
            self._version += 1
            self._cond.notify_all()

    def wait_for_new(self, last_version: int, timeout: float = 1.0):
        """阻塞到版本号变化(有新帧)或超时，返回(jpeg_bytes, 当前版本号)。"""
        with self._cond:
            self._cond.wait_for(lambda: self._version != last_version, timeout=timeout)
            return self._jpeg_bytes, self._version


def _make_handler(buffer: _LatestFrameBuffer):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # 静默HTTP访问日志，避免刷屏

        def do_GET(self):
            if self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header(
                    "Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}"
                )
                self.end_headers()
                last_version = -1
                try:
                    while True:
                        jpg, last_version = buffer.wait_for_new(last_version)
                        if jpg is None:
                            continue
                        self.wfile.write(f"--{_BOUNDARY}\r\n".encode())
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # 浏览器关闭页面/刷新，正常情况，不是错误
            elif self.path == "/":
                html = (
                    b"<html><body style='margin:0;background:#000'>"
                    b"<img src='/stream.mjpg' style='width:100%'></body></html>"
                )
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
            else:
                self.send_response(404)
                self.end_headers()

    return Handler


class _ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    # 2026-08-22实测踩坑：默认allow_reuse_address=False，节点重启测试时
    # 上一个进程刚释放的端口还在TIME_WAIT，立刻bind会报
    # "OSError: [Errno 98] Address already in use"，加SO_REUSEADDR后
    # 可以立刻复用，不用等内核TIME_WAIT超时。
    allow_reuse_address = True


class MjpegServer:
    def __init__(self, port: int = 8080):
        self.buffer = _LatestFrameBuffer()
        handler = _make_handler(self.buffer)
        # ThreadingTCPServer：每个连接(浏览器tab)独立线程，互不阻塞，
        # 允许多个浏览器/多次刷新同时连接
        self.httpd = _ReusableThreadingTCPServer(("0.0.0.0", port), handler)
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def update_frame(self, frame, jpeg_quality: int = 80):
        self.buffer.update(frame, jpeg_quality)

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
