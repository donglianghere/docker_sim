# -*- coding: utf-8 -*-
"""场地参数——**真机**。四个选手程序两边逐字节相同，差异全收在这一个文件里。"""
# ⚠⚠ 坐标还是仿真场地的值，飞前必须逐个按真实赛场改掉；写的是世界系，goto() 吃局部系。

# ---- 航线点位 ----
ROUTE_A = (2.7, 2.7)      # 起降区
ROUTE_B = (2.7, 22.2)
ROUTE_C = (17.0, 22.5)
ROUTE_G = (17.0, 12.5)    # 编队集结 / 解散
ROUTE_E = (10.0, 12.5)
ROUTE_F = (10.0, 17.5)    # 虚拟点：E 绕回 G 的中间点，场上没有实体
ROUTE_D = (17.2, 2.7)

# ---- 观察位 ----
POINT_M = (6.5, 16.0)     # 看 2# 楼
POINT_N = (13.4, 16.0)    # 看 1# 楼

# ---- 物资点 ----
SUPPLY_XY = (10.0, 7.8)

# ---- 目标标识（SDK 按字符串精确匹配检测结果的 class_id）----
# 2026-10-22 换 YOLO 模型 best_640：两类火情改用模型的类别名，物资点仍是
# AprilTag——任务机下视那一路照常跑 yolo 模式，AprilTag 是并行检测的，不受影响。
#: **一个标志位切整套**：True=火情用 YOLO 模型 best_640 的两个类别名，
#: False=退回 AprilTag。物资点两种模式下都走 tag（AprilTag 检测在机载节点里
#: 跟 YOLO 并行、不受 engine 影响），所以 SUPPLY_TAG 不参与切换。
#: 切回 tag 时记得把场地上的 AprilTag 摆回去。
USE_YOLO = False
SUPPLY_TAG  = 'apriltag:0'                                      # 物资点·地面火情用（任务机下视）
SUPPLY_TAG_HIGH = 'apriltag:3'                                  # 物资点·高层火情用（2026-10-23 新增）
#: ---- 2026-10-23 赛题改动：物资点上有**两件**物资 ----
#: 高层火情处置也要先到物资点抓一件灭火物资，用 ID 3 的 tag 代表，跟地面
#: 火情用的 ID 0 区分开。两张 tag 沿**东西**方向摆、各偏物资点中心
#: SUPPLY_TAG_OFFSET_M：ID 0 在西(-X)、ID 3 在东(+X)。
#: 任务机到了物资点靠 class_id 挑自己这一轮该抓的那件，不靠位置猜——但两件
#: 只隔 2*0.3=0.6 米，**抓取必须精确对准**，否则会抓错旁边那件。
#: ⚠️ 东西分配要是跟实际摆放反了，把下面两行的 - 和 + 对调即可。
SUPPLY_TAG_OFFSET_M = 0.3
SUPPLY_XY_GROUND = (SUPPLY_XY[0] - SUPPLY_TAG_OFFSET_M, SUPPLY_XY[1])
SUPPLY_XY_HIGH   = (SUPPLY_XY[0] + SUPPLY_TAG_OFFSET_M, SUPPLY_XY[1])
HIGH_FIRE   = 'FireplaceBright' if USE_YOLO else 'apriltag:1'   # 壁炉 0.50 宽 × 0.40 高 m
GROUND_FIRE = 'BrazierBright'   if USE_YOLO else 'apriltag:2'   # 火盆 0.30 × 0.30 m
#: 地面火情在**下视**里认哪些类别——`search_along()` 任一命中即算发现。
#: 为什么两类都认：2026-10-23 真机实测，俯视下模型把同一个火盆的置信度在
#: BrazierBright 和 FireplaceBright 之间来回分（两类的框中心都落在火盆上，
#: 实测 24 次 vs 12 次），每一类单独都过不了阈值，合起来才够。靠**相机路**
#: 区分火情类型比靠模型的类间判别可靠：下视 2 米俯视看不到楼上的壁炉，前视
#: 巡检楼面也不会把地面火盆摆到视野中心。
#: tag 模式下只有一类，行为跟以前逐字一样。
GROUND_FIRE_ANY = (GROUND_FIRE, HIGH_FIRE) if USE_YOLO else (GROUND_FIRE,)
#: 这一路用哪个机上 engine。**两种模式都要给**：AprilTag 检测内嵌在
#: yolo_detector_node 里，节点起不来 tag 也没有，而节点必须先加载 engine 才能起。
YOLO_ENGINE = '/models/best_640.engine'
#: 相机检测模式**恒为 yolo**，不随 USE_YOLO 切。原因：机载 yolo_detector_node
#: 用 cv2.aruco 跟推理并行同帧跑 AprilTag，两者都发到 vision/detections；
#: 而 mode='apriltag' 起的是 apriltag_ros 另一套（话题 apriltag/<label>/detections、
#: 消息类型也不同），SDK 不订阅那条，切过去等于整个检测链路断掉。
CAMERA_MODE = 'yolo'
#: True=机载只跑 AprilTag，不加载 engine、不做 YOLO 推理。跟着 USE_YOLO 走——
#: tag 赛制下推理出来的类别名没有任何消费者。
#: ⚠️ 2026-10-23 实测：**省的不是 CPU**（开关前后 325% vs 323%，推理在 GPU 上）。
#: 省的是 engine 加载、显存和启动时间。CPU 大头是 AprilTag 本身（约 270%），
#: 要降得调 apriltag_every_n / apriltag_scale / 阈值窗口档数。
SKIP_INFER = not USE_YOLO

#: ---- YOLO 置信度阈值（**选手可调**）----
#: 按相机路分开给；0 = 用机载节点 declare_parameter 的默认值 0.25。
#: venue.py 是挂载进容器的（run_real.sh 的 -v $SRCDIR:/workspace:ro），改完
#: **不用重建镜像**，下次起选手程序就生效。
#: ⚠️ 2026-10-23 真机实测，同一个火盆、同一个 best_640.engine、同一段时间：
#:       前视侧视  853 次检出，conf 最高 0.75
#:       下视俯视   24 次检出，conf 全挤在 0.26~0.46
#:   差 35 倍。根子是训练集缺 2 米正俯视样本，下视调低只是把 0.15~0.25 这一段
#:   捞回来、**治标**；要根治得用现场实物补俯视样本重训。
#:   调低的代价是误检变多，所以地面火情那一路别只信单帧（见 GROUND_FIRE_ANY）。
CONF_THRES_FRONT = 0.25
CONF_THRES_DOWN  = 0.15

# ---- 朝向 ----
FACADE_YAW_DEG = 90.0     # 火情只贴楼的 -Y 面，正对楼面就是机头朝正北
# (标签, 观察位, 观察位名, 机头朝向°, 是否查火情)，顺序即飞行顺序；3# 在 E 点只拍照。
# ⚠️ 本文件用旧编号：同三栋楼在 contestant_sim/venue.py 里是 1#->2#->3#；E 点取景需现场核。
STATIONS = (('3#楼', ROUTE_E, 'E', 180.0, False),
            ('2#楼', POINT_M, 'M',  90.0, True),
            ('1#楼', POINT_N, 'N',  90.0, True))

# ---- 高度 ----
CRUISE_AGL_M  = 2.0
OBSERVE_AGL_M = 2.0

# ---- 战术参数（按赛场可调，不是坐标；rounds.py 从这里取）----
SHOTS = 4                       # 灭火弹发数（侦察机破窗固定 1 发，不在这里）
FORWARD_BEFORE_FIRE_M = 1.0     # 侦察机瞄准后沿机头前移多少米到发射点
RECON_AIM_TIMEOUT_S   = 30.0    # 侦察机发现点的瞄准预算（秒），超时不阻塞
SUPPLY_AIM_TIMEOUT_S  = 15.0    # 任务机进场后的瞄准预算（秒），超时即刻发射
RECON_REAIM_TIMEOUT_S = 10.0    # 侦察机发射破窗弹之前的复瞄预算（秒）
DROP_AGL_M   = 1.0              # 投放灭火弹时的离地高度（米），越低落点误差越小
DROP_TOL_M   = 0.10             # 投放前要求对准到多少米以内
DROP_ALIGN_S = 5.0              # 对准最多几秒，到点就放行（对不准也照投）
