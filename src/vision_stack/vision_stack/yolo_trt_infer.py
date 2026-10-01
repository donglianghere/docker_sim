#!/usr/bin/env python3
"""独立于Ultralytics AutoBackend的YOLO26 TensorRT推理封装。

2026-08-21新增：Ultralytics的.engine后端内部仍然用CUDA版torch搬运输入/输出
张量，我们为了避开9GB多的无用CUDA13 wheel特意装的是CPU-only torch，两者冲突
（见docs/vision_stack.md风险清单）。这里绕开Ultralytics，直接用TensorRT
Python API + trt_common.py（原样拷贝自本机`/usr/src/tensorrt/samples/python/
common_runtime.py`，NVIDIA官方样例、Apache-2.0）做显存分配/拷贝，全程不需要
torch参与推理。

YOLO26是NMS-free架构，导出的ONNX/engine输出已经是解码好的
(1, max_det, 6) = [x1, y1, x2, y2, conf, class_id]（像素坐标，相对于640x640
输入尺寸），不需要自己写anchor解码或NMS。
"""
import argparse

import cv2
import numpy as np
import tensorrt as trt

from vision_stack.trt_common import allocate_buffers, do_inference, free_buffers

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


class YoloTrtDetector:
    def __init__(self, engine_path: str, conf_thres: float = 0.25):
        self.conf_thres = conf_thres
        with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings, self.stream = allocate_buffers(self.engine)

        input_name = self.engine.get_tensor_name(0)
        # NCHW，input_shape=(1,3,H,W)
        self.input_shape = tuple(self.engine.get_tensor_shape(input_name))
        self.input_h, self.input_w = self.input_shape[2], self.input_shape[3]

    def _letterbox(self, img: np.ndarray):
        """等比缩放+padding到(input_h, input_w)，返回处理后图像+还原用的scale/pad。"""
        h0, w0 = img.shape[:2]
        scale = min(self.input_w / w0, self.input_h / h0)
        new_w, new_h = int(round(w0 * scale)), int(round(h0 * scale))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_w, pad_h = self.input_w - new_w, self.input_h - new_h
        top, left = pad_h // 2, pad_w // 2
        padded = cv2.copyMakeBorder(
            resized, top, pad_h - top, left, pad_w - left,
            cv2.BORDER_CONSTANT, value=(114, 114, 114),
        )
        return padded, scale, left, top

    def infer(self, img_bgr: np.ndarray):
        padded, scale, pad_left, pad_top = self._letterbox(img_bgr)
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        chw = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        chw = np.ascontiguousarray(chw)

        self.inputs[0].host = chw.ravel()
        trt_outputs = do_inference(
            self.context, self.engine, self.bindings, self.inputs, self.outputs, self.stream
        )
        # 输出shape (1, max_det, 6)：[x1, y1, x2, y2, conf, class_id]，像素坐标相对640x640输入
        raw = trt_outputs[0].reshape(-1, 6)

        detections = []
        for x1, y1, x2, y2, conf, cls_id in raw:
            if conf < self.conf_thres:
                continue
            # 还原letterbox变换，映射回原图坐标。显式转成原生Python
            # float/int——raw是numpy数组，逐元素解包出来的是numpy标量类型
            # (float32等)，ROS2消息字段setter的类型检查只认原生float，塞
            # numpy.float32进去会在yolo_detector_node发布消息时报
            # "AssertionError: The 'x' field must be of type 'float'"
            # （2026-08-22实测踩过）。
            detections.append({
                "x1": float((x1 - pad_left) / scale),
                "y1": float((y1 - pad_top) / scale),
                "x2": float((x2 - pad_left) / scale),
                "y2": float((y2 - pad_top) / scale),
                "conf": float(conf),
                "class_id": int(cls_id),
            })
        return detections

    def close(self):
        free_buffers(self.inputs, self.outputs, self.stream)


# COCO 80类名称（公版YOLO26n预训练权重用的是COCO数据集，索引需跟训练时一致）
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    detector = YoloTrtDetector(args.engine, conf_thres=args.conf)
    img = cv2.imread(args.image)
    if img is None:
        raise FileNotFoundError(args.image)

    import time
    t0 = time.time()
    dets = detector.infer(img)
    t1 = time.time()

    print(f"inference wall time: {(t1 - t0) * 1000:.2f} ms")
    print(f"num detections (conf>={args.conf}): {len(dets)}")
    for d in dets:
        name = COCO_NAMES[d["class_id"]] if d["class_id"] < len(COCO_NAMES) else f"cls{d['class_id']}"
        print(f"  {name}: conf={d['conf']:.2f} "
              f"box=({d['x1']:.0f},{d['y1']:.0f},{d['x2']:.0f},{d['y2']:.0f})")

    detector.close()


if __name__ == "__main__":
    main()
