"""基于GStreamer appsink的MIPI相机取流封装。

用PyGObject原生GStreamer绑定，不用cv2.VideoCapture(..., cv2.CAP_GSTREAMER)——
2026-08-22实测确认pip装的opencv-python没有编译GStreamer后端
(cv2.getBuildInformation()只有FFMPEG)，走这条路更可靠，也是Jetson官方相机
示例的标准写法。

2026-08-25删掉了RTP硬编码推流分支（`tee`+`nvv4l2h264enc`+`udpsink`）：
地面站给的视频统一改成拉流方式(MJPEG-over-HTTP，见mjpeg_server.py)，不再
主动推流。原因：`udpsink`往不可达地址发包时会在ARP解析上阻塞，连累同一个
Argus会话里的appsink分支(YOLO)一起卡死"camera read timeout"——完整踩坑
过程见read_hw.md 2026-08-22"阶段7"一节。两路相机接上后如果继续用推流，
这个风险会翻倍(两路都可能被地面站网络问题连累)，改拉流后地面站不主动连接
就不会发包，天然规避这个问题类。硬编码RTP的"几乎不占CPU"优势暂时放弃，
换MJPEG的CPU软编码JPEG，两路同时开时的CPU余量需要重新实测(阶段8的数据是
单路测的)。如果以后要把硬编码效率捡回来，参考`gst-rtsp-server`做"按需
推流"的RTSP拉流方案，不要走这次删掉的裸UDP推流路线，见read_hw.md 2026-08-25
条目。
"""
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402

import numpy as np  # noqa: E402

Gst.init(None)


class GstCameraCapture:
    """nvarguscamerasrc取流，appsink拉取BGR格式numpy帧。"""

    def __init__(
        self,
        sensor_id: int = 0,
        width: int = 1280,
        height: int = 720,
        framerate: int = 30,
    ):
        self.width = width
        self.height = height

        pipeline_str = (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM),width={width},height={height},"
            f"framerate={framerate}/1,format=NV12 ! "
            f"nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
            f"appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false "
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")

    def start(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("GStreamer pipeline failed to start")

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)

    def read(self, timeout_ns: int = 1_000_000_000):
        """拉一帧，超时或管线无数据返回None。"""
        sample = self.appsink.try_pull_sample(timeout_ns)
        if sample is None:
            return None
        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct = caps.get_structure(0)
        h = struct.get_value("height")
        w = struct.get_value("width")
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape((h, w, 3)).copy()
        finally:
            buf.unmap(mapinfo)
        return frame
