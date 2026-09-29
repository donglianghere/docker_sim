# 相机标定工具

给两路 IMX219 标内参（AprilTag / 二维码的位姿估计要用；YOLO 检测不需要）。

| 文件 | 作用 |
|---|---|
| `calib_camera.py` | 主脚本：拉流采集 + 标定 |
| `make_calib_board.py` | 生成棋盘格标定板（打印用） |
| `merge_calib.py` | 把多次采集合并重标，结果更稳 |
| `mjpg_view.py` | 拉流看画面 / 按空格抓图（标定脚本也依赖它） |

## 用法

```bash
cd ~/ai_uav/tools/calib_tools

# 1. 生成标定板并打印（A4 用 22mm，A3 用 30mm）
python3 make_calib_board.py --square 30 --paper A3
#    → ~/camera_calib/board/ 下的 PDF，必须 100% 实际大小打印，
#      打完用尺量一下方格实际边长，贴硬板上压平

# 2. 标定（--square 填你量到的实际边长）
python3 calib_camera.py front --board 9x6 --square 30   # 前视 8080
python3 calib_camera.py down  --board 9x6 --square 30   # 下视 8081

# 3. 单次结果不稳就多采几轮，然后合并
python3 merge_calib.py '~/camera_calib/cam0_front_*'
```

举板要点：**明显倾斜**（左右各转 30–45°、上下各仰俯 30–45°）、**距离要变**（走近走远）、
覆盖画面各个角落。光平移不倾斜会导致标定退化——RMS 看着很漂亮但内参是错的。

## 结果

输出在 `~/camera_calib/<相机>_<时间戳>/`：

- `camera_info.yaml` — ROS 格式内参，直接给节点用
- `poses.png` — 标定板姿态分布图，板都平行/都同距离就说明采集不合格
- `calib.json`、`images/`、`undistort.jpg`

**合格判据看最后两行**：折半交叉检验 **< 1%** 才算定了；有 `!! 标定退化警告` 就得重采。

## 当前结果（2026-09-27 实测，NX02 两路）

| | fx | fy | cx | cy | HFOV |
|---|---|---|---|---|---|
| 前视 cam0 | 1200.71 | 1200.17 | 646.32 | 327.88 | 56.12° |
| 下视 cam1 | 1194.15 | 1192.32 | 654.08 | 359.76 | 56.38° |

两路同型号同镜头，焦距只差 0.55%，但**主点差 31.9 px**（装配决定），所以**必须分别标定，不能互相套用**。

## 顺带

`mjpg_view.py` 也可以单独用来看流、按空格抓图（采训练数据就用它）：

```bash
python3 mjpg_view.py http://192.168.2.102:8080/stream.mjpg ~/uav_data/raw
```

详细原理、坑和排查见 `docker_real/ai_uav/docs/相机内参标定操作说明.md`。
