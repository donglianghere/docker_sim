#!/usr/bin/env bash
# 把训练好的 YOLO .pt 一键部署到机载机：导出 ONNX -> 上传 -> 编译成 Jetson TensorRT engine -> 验证
#
# 用法:
#   ./deploy_model.sh                                  # 默认 ./best.pt -> 192.168.2.102
#   ./deploy_model.sh --host 192.168.2.101
#   ./deploy_model.sh --imgsz 640                      # 用 640 换帧率(默认取训练时的 imgsz)
#   ./deploy_model.sh --name fire_v2                   # 自定义 engine 文件名
#
# 三个必须对上的地方(对不上机上就是错的, 脚本会逐个卡住):
#   1) 导出必须带 nms=True     -> 输出 (1,300,6) 端到端格式, 否则机上 yolo_trt_infer.py
#                                 按 (1,300,6) 解析会读到垃圾(默认导出是 (1,6,8400))
#   2) engine 必须在机载机上编译 -> TensorRT engine 不跨机器、不跨版本
#   3) 类别名机上是硬编码 COCO_NAMES -> 自定义类别要改机上代码, 见脚本结尾提示
set -euo pipefail

HOST="192.168.2.102"
USER_NAME="nvidia"
PT="best.pt"          # 跟脚本放同一个文件夹
IMGSZ=""                 # 留空=自动取 .pt 里训练时用的 imgsz
NAME=""                  # 留空=用 .pt 的文件名
PRECISION="--fp16"
REMOTE_MODELS="/home/nvidia/ai_uav/docker_sim/models"
CONTAINER="docker_sim-vision-stack-1"
TRTEXEC="/usr/src/tensorrt/bin/trtexec"
KEEP_ONNX=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host) HOST="$2"; shift 2;;
        --user) USER_NAME="$2"; shift 2;;
        --pt) PT="$2"; shift 2;;
        --imgsz) IMGSZ="$2"; shift 2;;
        --name) NAME="$2"; shift 2;;
        --fp32) PRECISION=""; shift;;
        --keep-onnx) KEEP_ONNX=1; shift;;
        -h|--help) sed -n '2,20p' "$0"; exit 0;;
        *) echo "未知参数: $1" >&2; exit 1;;
    esac
done

cd "$(dirname "$(readlink -f "$0")")"
[[ -f "$PT" ]] || { echo "找不到模型文件: $PT" >&2; exit 1; }
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 ${USER_NAME}@${HOST}"

echo "=========================================================="
echo " 模型部署: $PT  ->  ${USER_NAME}@${HOST}"
echo "=========================================================="

# ---- 1. 读模型元信息 ----------------------------------------------------
echo
echo "[1/6] 读取模型信息"
META=$(python3 - "$PT" <<'PYEOF'
import sys, warnings, json
warnings.filterwarnings("ignore")
import torch
ck = torch.load(sys.argv[1], map_location="cpu", weights_only=False)
m = ck.get("model")
names = getattr(m, "names", {}) or {}
ta = ck.get("train_args") or {}
print(json.dumps({
    "imgsz": int(ta.get("imgsz") or 640),
    "nc": len(names),
    "names": [names[k] for k in sorted(names)],
    "yaml": (getattr(m, "yaml", {}) or {}).get("yaml_file", "?"),
    "ver": ck.get("version", "?"),
}, ensure_ascii=False))
PYEOF
)
TRAIN_IMGSZ=$(echo "$META" | python3 -c "import json,sys;print(json.load(sys.stdin)['imgsz'])")
NC=$(echo "$META" | python3 -c "import json,sys;print(json.load(sys.stdin)['nc'])")
CLASS_NAMES=$(echo "$META" | python3 -c "import json,sys;print(', '.join(json.load(sys.stdin)['names']))")
ARCH=$(echo "$META" | python3 -c "import json,sys;print(json.load(sys.stdin)['yaml'])")
[[ -n "$IMGSZ" ]] || IMGSZ="$TRAIN_IMGSZ"
[[ -n "$NAME" ]] || NAME="$(basename "$PT" .pt)"
STEM="${NAME}_${IMGSZ}"
echo "  架构=$ARCH  类别数=$NC  类别=[$CLASS_NAMES]"
echo "  训练 imgsz=$TRAIN_IMGSZ, 本次导出 imgsz=$IMGSZ"
if [[ "$IMGSZ" != "$TRAIN_IMGSZ" ]]; then
    echo "  ⚠ 导出尺寸与训练尺寸不同, 精度会掉一些(换取帧率)"
fi

# ---- 2. 导出 ONNX(必须 nms=True) ---------------------------------------
echo
echo "[2/6] 导出 ONNX (nms=True, imgsz=$IMGSZ) —— 这一步要几分钟"
ONNX="${STEM}.onnx"
python3 - "$PT" "$IMGSZ" "$ONNX" <<'PYEOF'
import sys, os, shutil, warnings
warnings.filterwarnings("ignore")
from ultralytics import YOLO
pt, imgsz, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
# nms=True 才会导出成端到端的 (1,300,6), 与机上解析代码一致
p = YOLO(pt).export(format="onnx", imgsz=imgsz, nms=True, device="cpu", verbose=False)
if os.path.abspath(p) != os.path.abspath(out):
    shutil.move(p, out)
print("  ->", out)
PYEOF

# ---- 3. 校验 ONNX 维度(对不上就别浪费时间传了) --------------------------
echo
echo "[3/6] 校验 ONNX 输入输出维度"
python3 - "$ONNX" "$IMGSZ" <<'PYEOF'
import sys, onnx
mo = onnx.load(sys.argv[1]); imgsz = int(sys.argv[2])
dims = lambda t: [d.dim_value or d.dim_param for d in t.type.tensor_type.shape.dim]
i = dims(mo.graph.input[0]); o = dims(mo.graph.output[0])
print("  输入:", i, " 输出:", o)
assert i == [1, 3, imgsz, imgsz], "输入维度应为 [1,3,%d,%d]" % (imgsz, imgsz)
assert len(o) == 3 and o[0] == 1 and o[2] == 6, \
    "输出应为 (1,N,6) 端到端格式, 实际 %s —— 多半是漏了 nms=True" % o
print("  ✓ 端到端格式, 与机上 yolo_trt_infer.py 的解析方式一致")
PYEOF

# ---- 4. 上传并核对 md5 --------------------------------------------------
echo
echo "[4/6] 上传到 ${HOST}:${REMOTE_MODELS}/"
# 类别名文件: 机上 YoloTrtDetector 会自动读 engine 同名的 .names,
# 读不到才回落 COCO_NAMES —— 所以自定义类别不用改机上代码
echo "$META" | python3 -c "import json,sys;print('\n'.join(json.load(sys.stdin)['names']))" > "${STEM}.names"
echo "  类别名文件: ${STEM}.names ($NC 类)"
LOCAL_MD5=$(md5sum "$ONNX" | awk '{print $1}')
scp -o BatchMode=yes -q "$ONNX" "${STEM}.names" "${USER_NAME}@${HOST}:${REMOTE_MODELS}/"
REMOTE_MD5=$($SSH "md5sum ${REMOTE_MODELS}/$(basename "$ONNX") | awk '{print \$1}'")
[[ "$LOCAL_MD5" == "$REMOTE_MD5" ]] || { echo "  ✗ md5 不一致, 传输出错" >&2; exit 1; }
echo "  ✓ md5 一致: $LOCAL_MD5"

# ---- 5. 在机载机上编译 engine ------------------------------------------
echo
echo "[5/6] 在机载机上编译 TensorRT engine (Orin 上要几分钟, 别中断)"
$SSH "${TRTEXEC} --onnx=${REMOTE_MODELS}/${STEM}.onnx \
        --saveEngine=${REMOTE_MODELS}/${STEM}.engine ${PRECISION} 2>&1 | \
      grep -E 'Engine built|Throughput|mean:|median:|Latency|error|Error|failed' | head -12"
$SSH "test -s ${REMOTE_MODELS}/${STEM}.engine" || { echo "  ✗ engine 没有生成" >&2; exit 1; }

# ---- 6. 在推理容器内验证(必须容器内, 那才是真正跑推理的环境) -----------
echo
echo "[6/6] 在 vision-stack 容器内验证 engine"
$SSH "docker exec -i ${CONTAINER} python3 -" <<PYEOF
import tensorrt as trt
lg = trt.Logger(trt.Logger.ERROR)
with open("/models/${STEM}.engine", "rb") as f, trt.Runtime(lg) as rt:
    e = rt.deserialize_cuda_engine(f.read())
assert e is not None, "engine 反序列化失败(TensorRT 版本不匹配?)"
print("  TensorRT", trt.__version__)
ok = True
for i in range(e.num_io_tensors):
    n = e.get_tensor_name(i)
    mode = e.get_tensor_mode(n).name
    shape = tuple(e.get_tensor_shape(n))
    print("   ", mode, n, shape, e.get_tensor_dtype(n).name)
    if mode == "INPUT" and shape != (1, 3, ${IMGSZ}, ${IMGSZ}):
        ok = False
    if mode == "OUTPUT" and (len(shape) != 3 or shape[0] != 1 or shape[2] != 6):
        ok = False
print("  ✓ 维度校验通过" if ok else "  ✗ 维度不对, 机上解析会出错")
raise SystemExit(0 if ok else 1)
PYEOF

REMOTE_SIZE=$($SSH "du -h ${REMOTE_MODELS}/${STEM}.engine | cut -f1")
[[ $KEEP_ONNX -eq 1 ]] || { $SSH "rm -f ${REMOTE_MODELS}/${STEM}.onnx"; rm -f "$ONNX" "${STEM}.names"; }

cat <<EOF

==========================================================
 完成: ${REMOTE_MODELS}/${STEM}.engine  (${REMOTE_SIZE})
 容器内路径: /models/${STEM}.engine
==========================================================

类别名已自动处理: ${STEM}.names 已随 engine 上传, 机上 YoloTrtDetector
会自动读取(读不到才回落 COCO_NAMES), 不用改机上代码。

要让检测节点用上这个 engine。注意 2026-09-28 起 yolo_detector_node 的默认
engine 已改成 /models/best_960.engine(见其 declare_parameter), 所以:
  - 若这次部署的就是要当默认的模型, 改节点默认值并 colcon build 最省事,
    否则地面站网页/control_server 起的节点仍用旧默认值;
  - 若只是临时试新模型, 按下面显式传 engine_path 起节点。

  # 先停掉占着该路相机和端口的旧节点(Argus 一路相机只能有一个会话)
  ssh ${USER_NAME}@${HOST} "docker exec ${CONTAINER} bash -c \\
    'pgrep -f \"[y]olo_detector_node.*sensor_id:=1\" | xargs -r kill'"
  sleep 15   # 等端口和 Argus 会话释放, 等不够会 Address already in use

  # 再用新 engine 起
  ssh ${USER_NAME}@${HOST} "docker exec -d -w /opt/vision_ws ${CONTAINER} bash -c \\
    'source /opt/ros/humble/setup.bash && source install/setup.bash && \\
     exec ros2 run vision_stack yolo_detector_node --ros-args -r __ns:=/NX02 \\
     -p sensor_id:=1 -p camera_width:=1280 -p camera_height:=720 \\
     -p camera_framerate:=30 -p publish_rate_hz:=15.0 \\
     -p topic:=vision/down/detections -p frame_id:=camera_down \\
     -p engine_path:=/models/${STEM}.engine \\
     -p mjpeg_enabled:=true -p mjpeg_port:=8081 -p mjpeg_draw_detections:=true \\
     > /tmp/cam1_yolo_new.log 2>&1'"
EOF
