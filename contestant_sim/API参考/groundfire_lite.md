# groundfire_lite.py 用到的 SDK API

> 这个程序用到 **19 个** API（SDK 全部能力有 80 个，其余是底层/备用接口，这个程序没用到）。
> 条目按**程序里出现的先后**排，括号里是首次用到的行号——对着 `groundfire_lite.py` 从上往下读，顺序能对上。
> 本文档由 `scripts/gen_sdk_api_doc.py` 自动生成，**不要手改**。


## 速查

| 行 | API | 分类 | 做什么 |
|---|---|---|---|
| 28 | [`set_camera_mode`](#set_camera_mode) | 视觉：识别与对准 | 切换自己这架飞机某一路相机的检测模式。 |
| 29 | [`open_inbox`](#open_inbox) | 跨机协同 | 注册这些跨机事件的收件箱。在任务一开始就全部注册，见上面说明。 |
| 30 | [`takeoff`](#takeoff) | 起飞 / 降落 / 返航 | 起飞：发布TakeoffLand{TAKEOFF}，阻塞直到armed=True且飞控自己的起飞状态机真 |
| 39 | [`set_max_vel`](#set_max_vel) | 航线飞行 | 改ego_planner的巡航限速，返回改之前的值（方便用完恢复）。 |
| 41 | [`fly_route`](#fly_route) | 航线飞行 | 按航点序列飞：每个航点先把机头转到下一段方向、停住，再走，航段之间航向不变。 |
| 51 | [`search_along`](#search_along) | 视觉：识别与对准 | 从当前位置飞向 leg_end，边飞边找；看到就刹停、对准、解算坐标。 |
| 56 | [`announce`](#announce) | 声光播报 | 播报一次声光事件。play_sound_light() 的别名，名字更贴近用途。 |
| 58 | [`send_to_teammate`](#send_to_teammate) | 跨机协同 | 发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛TeammateUnreachableEr |
| 60 | [`hold_at`](#hold_at) | 航线飞行 | 飞到某个世界坐标悬停待命，不降落。seconds>0 就停够这么久再返回。 |
| 61 | [`wait_event`](#wait_event) | 跨机协同 | 等一个跨机事件，返回它带来的数据（dict，没有数据就是空 dict）。 |
| 62 | [`lead_formation`](#lead_formation) | 编队 | 长机带队飞一条航线。只管空中段，起降由调用方自己决定。 |
| 73 | [`PHOTO_DIR`](#photo_dir) | 拍照 | snapshot() 的存图目录，用实例属性覆盖即可：sdk.PHOTO_DIR = '/logs/xx |
| 78 | [`fetch_from`](#fetch_from) | 抓放与发射 | 飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。 |
| 84 | [`goto_world`](#goto_world) | 航线飞行 | 飞到一个世界坐标上方并锁高（走 ego_planner，有避障）。 |
| 86 | [`grip`](#grip) | 抓放与发射 | 驱动机械抓。release=False 抓紧，True 松开（打开爪子）。 |
| 87 | [`snapshot`](#snapshot) | 拍照 | 拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。 |
| 90 | [`follow_formation`](#follow_formation) | 编队 | 僚机跟队。只管空中段，起降由调用方自己决定。 |
| 91 | [`return_home`](#return_home) | 起飞 / 降落 / 返航 | 回自己的起飞点，默认降落。返回是否真的落在自己的起降点上。 |
| 95 | [`run`](#run) | 程序入口 | 选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。 |

## 逐个说明

<a id="set_camera_mode"></a>
### `set_camera_mode(camera: str, mode: str='yolo', wait: bool=True, timeout: float=20.0)`

*groundfire_lite.py 第 28 行首次用到 · 视觉：识别与对准*

切换**自己这架飞机**某一路相机的检测模式。

**参数**

- camera: `'front'`（前视，= control_server 的 cam0）或 `'down'`（下视，= cam1）。
- mode: `'yolo'` 检测 / `'apriltag'` 只认标签 / `'raw'` 纯视频直通 / `'stop'` 停掉这一路。同一路相机上这几种模式互斥。
- wait: `mode='yolo'` 时是否等到这一路**真的出检测数据**才返回。 默认 True——只发请求不等的话，紧接着的 `wait_for_detection()` 很可能在节点还没起好时就开始等。
- timeout: 等出数据的超时秒数。实测正常 4 秒，默认给足余量。

<a id="open_inbox"></a>
### `open_inbox(*events: str)`

*groundfire_lite.py 第 29 行首次用到 · 跨机协同*

注册这些跨机事件的收件箱。**在任务一开始就全部注册**，见上面说明。

**参数**

- *events: 要注册的事件名，可以一次给多个。 重复注册同一个事件是安全的（幂等），不会把已收到的内容清掉。

> ⚠️ 可靠事件通道是“先回 ACK 再查处理函数”，没注册的事件会被确认后丢弃。任务一开始就把所有事件注册全，别等用到了再注册。

<a id="takeoff"></a>
### `takeoff(timeout: float=60.0, height_m: Optional[float]=None)`

*groundfire_lite.py 第 30 行首次用到 · 起飞 / 降落 / 返航*

起飞：发布`TakeoffLand{TAKEOFF}`，阻塞直到`armed=True`**且**飞控自己的起飞状态机真正爬升到位、转入稳定悬停——不是`armed=True`就立刻返回。

**参数**

- timeout: 总超时秒数，同时覆盖"等`armed=True`"和"等位置 稳定"这两段。默认从30秒提到60秒——实测确认这套仿真 （`uwb_imu`+`pt4ctrl`组合）从解锁到真正稳定悬停，中间 有一段爬升超调+振荡衰减的过程，衰减到0.3米容差以内 经常要20~30秒量级，30秒的旧默认值不够用。
- height_m: 起飞到多高（米，离地）。不给就用飞控 `pt4ctrl` 里 配置的 takeoff_height。

<a id="set_max_vel"></a>
### `set_max_vel(max_vel_mps: float, timeout: float=5.0)`

*groundfire_lite.py 第 39 行首次用到 · 航线飞行*

改`ego_planner`的巡航限速，返回改之前的值（方便用完恢复）。

**参数**

- max_vel_mps: 新的限速，米/秒。必须为正。
- timeout: 等参数服务响应的超时秒数。

<a id="fly_route"></a>
### `fly_route(waypoints: Sequence[Tuple[float, float]], agl_m: float=2.0, hold_s: float=2.0, names: Optional[List[str]]=None)`

*groundfire_lite.py 第 41 行首次用到 · 航线飞行*

按航点序列飞：**每个航点先把机头转到下一段方向、停住，再走**，航段之间航向不变。

**参数**

- waypoints: [(wx, wy), ...] 世界坐标。
- agl_m: 全程锁的离地高度。
- hold_s: 每个航点停多久（转向跟停顿同时进行，不足的部分补足）。
- names: 航点名字，只用于日志；不给就按序号。

<a id="search_along"></a>
### `search_along(leg_end: Tuple[float, float], class_id: str, camera: str='down', agl_m: float=2.0, timeout_s: float=420.0, what: str='目标', speed_mps: Optional[float]=None, face_leg: bool=True, leg_start: Optional[Tuple[float, float]]=None, center_tol_px: Optional[float]=None, center_timeout_s: Optional[float]=None)`

*groundfire_lite.py 第 51 行首次用到 · 视觉：识别与对准*

从当前位置飞向 leg_end，**边飞边找**；看到就刹停、对准、解算坐标。

**参数**

- leg_end: 这一段飞到哪儿（世界坐标 (x, y)）。从**当前位置**出发。
- class_id: 边飞边找什么，比如 'apriltag:2'。
- camera: 用哪个相机找，'down'（找地面目标）或 'front'。
- agl_m: 这一段锁的离地高度（米）。
- timeout_s: 整段最多花多久（秒）。超时按"没找到"返回 None。
- what: 目标叫什么，只用于日志。
- speed_mps: 这一段的巡航限速（米/秒）。不给就用 `SEARCH_SPEED_MPS` （0.4）——见那个常量上面的实测说明。用完自动恢复原限速。
- face_leg: 出发前先把机头转到这一段的方向（默认 True）。 为什么默认开：① 进入搜索段时机头还停在**上一段**的航向上 （`fly_route()` 只在它自己的航点之间转向，最后一个航点之后 没有"下一段"可转），2026-10-06 实测 G->E 段机头没对准 E； ② 下视画面是 1280x720，长边垂直于航向时横向覆盖 1.57 m、 纵向 0.9 m——搜索要的正是"横向扫得宽"，机头对准航向才是 这个朝向。转向期间飞机悬停，转完才起步。
- leg_start: 这一段的**标称起点**（世界坐标 (x, y)），只用来算航向。 给了就用"标称起点 -> leg_end"的方向角（用户 2026-10-06 要求： 这一段的偏航角严格等于航线上这一段的方向角）；不给就退回 "飞机当前实际位置 -> leg_end"。两者差别来自飞机停在航点上的 位置误差：G->E 长 8.3 m、到点误差几十厘米时，方向角差 3° 上下。 ⚠️ 算角度前两个点都会 `world_to_local()`，所以世界系和局部系 之间那点偏转（起飞点锁定带来的，实测 -0.37°）会被自动吃掉， 不能直接拿世界坐标 atan2 当局部系航向用。
- center_tol_px: 发现目标后飞到它正上方的像素收敛阈值，透传给 `track_over_target()`，不给用 `CENTER_TOL_PX`（50）。
- center_timeout_s: 对准的机载时限，透传给 `track_over_target()`， 不给用 `CENTER_TIMEOUT_S`（30）。**超时按成功处理**，会用当时 解算的坐标继续往下走。

<a id="announce"></a>
### `announce(event: str)`

*groundfire_lite.py 第 56 行首次用到 · 声光播报*

播报一次声光事件。`play_sound_light()` 的别名，名字更贴近用途。

**参数**

- event: 声光事件名。**必须是固定枚举里的一项**（见 `_sound_light_port.py` 的 SOUND_LIGHT_EVENTS），自造名字会 直接抛 ValueError 把整个任务打断。

<a id="send_to_teammate"></a>
### `send_to_teammate(event: str, timeout_s: float=DEFAULT_SEND_TO_TEAMMATE_TIMEOUT_S, **kwargs: Any)`

*groundfire_lite.py 第 58 行首次用到 · 跨机协同*

发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛`TeammateUnreachableError`（方案2.3节"链路B"，`reliability.py`已经实现好ACK+1Hz重发协议，这里只是转调）。

**参数**

- event: 事件名字符串（选手自定义，需要跟队友`on_teammate_ event()`注册的名字完全一致）。
- timeout_s: 总超时秒数，默认45秒（方案建议区间30-60秒的 中间值），可以覆盖。
- **kwargs: 随事件一起带给对方的业务数据（必须是JSON能表示的 类型）。

<a id="hold_at"></a>
### `hold_at(wx: float, wy: float, agl_m: float=2.0, seconds: float=0.0, direct: bool=False)`

*groundfire_lite.py 第 60 行首次用到 · 航线飞行*

飞到某个世界坐标悬停待命，**不降落**。seconds>0 就停够这么久再返回。

**参数**

- wx: 待命点的世界坐标 x（米）。
- wy: 待命点的世界坐标 y（米）。
- agl_m: 悬停的离地高度（米）。
- seconds: 到位后再停多久（秒）。0 = 到了就返回，不额外等。
- direct: True = 走直线（`goto_direct`），**不经规划器、没有避障**。 含义和前提同 `goto_world()`——必须先算过这条直线的余量。

<a id="wait_event"></a>
### `wait_event(event: str, timeout_s: float=300.0, clear: bool=True, required: bool=True)`

*groundfire_lite.py 第 61 行首次用到 · 跨机协同*

等一个跨机事件，**返回它带来的数据**（dict，没有数据就是空 dict）。

**参数**

- event: 事件名，必须先 `open_inbox()` 注册过。
- timeout_s: 等多久。
- clear: 取走后把收件箱复位（默认 True）。多轮流程里同一个事件会来 好几次，不复位的话第二轮一进来就立刻返回上一轮的旧数据。
- required: False = 超时就**返回 None**，不抛异常。用在"等到更好、 等不到也得往下走"的地方——比如最后等队友报告已降落，等不到 也该把任务完成播出去，不能让整个任务在这一步失败。

<a id="lead_formation"></a>
### `lead_formation(route: Sequence[Tuple[float, float]], spacing_m: Optional[float]=None, agl_m: float=2.0, hold_s: float=2.0, start_xy: Optional[Tuple[float, float]]=None, direct_legs: Optional[Sequence[Tuple[Tuple[float, float], Tuple[float, float]]]]=None, final_xy: Optional[Tuple[float, float]]=None, disband_at: Optional[Tuple[float, float]]=None, wait_follower_s: float=300.0, tail_direct: bool=False, turn_at_waypoint: bool=False)`

*groundfire_lite.py 第 62 行首次用到 · 编队*

长机带队飞一条航线。**只管空中段**，起降由调用方自己决定。

**参数**

- route: 航线（世界坐标）。
- spacing_m: 目标纵向间距（米）。**不给就用 `self.spacing_m`** （`run()` 从命令行 `--spacing` 存进去的值），所以一般不用传—— 写 `spacing_m=sdk.spacing_m` 等于把 SDK 自己的值原样传回来， 没有任何信息量。只有这一段要用跟全局不同的间距时才显式给。
- agl_m: 全程锁的离地高度（米）。
- hold_s: 每个航点停多久（秒）。转向跟停顿同时进行，不足的部分补足。
- start_xy: 从哪儿起步（默认自己当前位置换算成的起飞点）。编队段从 半路开始时要给，否则僚机算出来的起始站位会跑到场外。
- direct_legs: 指定哪些航段走**直飞**（`goto_direct`，不经规划器、 **没有避障**），元素是世界坐标的 `(起点, 终点)` 对，比如 `[(ROUTE_B, ROUTE_C), (ROUTE_C, ROUTE_G)]`。不在表里的航段照常 走规划器。 ⚠️ **每一段都必须先用 `tools/check_route.py --direct` 核过余量**。 2026-10-01 就是手算漏了 `obstacle_cylinder`，把正穿 6 米高柱子的 航段判成"干净"改成直飞，结果擦墙摔机（那根圆柱 2026-10-11 已被 用户去掉，但"不手算、用工具"这条照旧）。工具会把布局文件里的 立柱/避障圆柱/仿地模块/四面墙**全部**读出来算。 ⚠️ 工具只算**水平**余量，不看障碍高度。像"仿地模块"这种高 0.6 m （2026-10-11 用户从 0.5 改的）的地面凸起，它会报余量 0、判定 不可直飞，但巡航 1.5 m 时从上方飞过是安全的——这种例外要自己判断，并且记住：**巡航高度一旦降低， 结论就不再成立**。
- final_xy: 航线跑完再飞到哪儿（默认 route[0]）。
- disband_at: 给了就按"**僚机过了这个航点**"解散；不给按"长机飞回 自己起飞点上空"解散。
- wait_follower_s: 等僚机到站位的上限，等不到也照飞。
- tail_direct: 最后一段（飞往 final_xy 那一段）走直线。 **解散就发生在这一段上**——disband_at 的判据是"长机已经离开 那个航点 spacing+lag 米"，所以长机走到这一段中途才解散，剩下 的路本质上是"解散后各自回家"。前提同 `goto_world()` 的 direct：必须算过这条直线的余量。
- turn_at_waypoint: 拐点要不要把机头转到下一段航向。**默认 False=不转** （2026-10-05 用户要求改成默认不转向）。 · False：机头保持进入编队段时的朝向不变，整条航线不动。`hold_s` 仍然照停——那个停顿是给僚机收间距用的，跟转不转向无关。 · True：拐点先停下把机头转到该段航向再前飞（`face_yaw()`，容差 5°、超时 15 秒）。实际转速受两级限幅：traj_server 的 `yaw_dot_max`（EGO_YAW_DOT_MAX，真机 0.8 rad/s ≈ 45.8 °/s）和 飞控的 `MPC_YAWRAUTO_MAX`（真机 45.0 °/s，实际瓶颈），所以 90° 左右的拐点约 2 秒转完，跟 `hold_s=2.0` 刚好吻合。 ⚠️ 这一项只管长机。僚机那侧是 `follow_formation()` 的**同名参数** （默认值也是 False），它往下传成 `formation_follower_node` 的 `yaw_follow_leg`。两边分别设置、互不联动——要一致就两边给同一个值。

> ⚠️ disband_at 要显式给。不给的话默认判据是“长机飞回自己起飞点上空”，而起飞点不一定在航线上，编队可能在半路散掉。

<a id="photo_dir"></a>
### `PHOTO_DIR`

*groundfire_lite.py 第 73 行首次用到 · 拍照*

snapshot() 的存图目录，用实例属性覆盖即可：sdk.PHOTO_DIR = '/logs/xxx'。

<a id="fetch_from"></a>
### `fetch_from(wxy: Tuple[float, float], class_id: str, what: str='物资', agl_m: float=2.0, sound: Optional[str]=None, direct: bool=False, precise: bool=True)`

*groundfire_lite.py 第 78 行首次用到 · 抓放与发射*

飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。

**参数**

- wxy: 物资点的世界坐标 (x, y)。
- class_id: 物资点上贴的标识，比如 'apriltag:0'，用来边瞄边降。
- what: 物资叫什么，进日志也进降落提示（"已降落在{what}点"）。
- agl_m: 飞过去时的巡航高度，抓完也回到这个高度（米）。
- sound: 声光播报内容。**播报点在落地之后、舵机动作之前**——裁判 听到的那一声要对上"正在抓"这个瞬间；写在 `fetch_from()` 前面会提前十几秒（还在飞往物资点的路上就播了）。
- direct: True = 飞过去那一段走直线，不经规划器。含义和前提同 `goto_world()`。
- precise: True（默认）= 走 `land_on()` 的边瞄边降，对准标识再抓； False = **只飞到点上空直接降落**，不碰视觉。 什么时候用 False：抓取动作本身不要求对准标识的场合——比如综合任务 高层火情轮取灭火器材，用户 2026-10-07 明确："高楼火情物资点降落 只是做一个动作，不需要精降"。地面火情轮取灭火弹仍要精降（不对准 就抓不到）。

<a id="goto_world"></a>
### `goto_world(wx: float, wy: float, agl_m: Optional[float]=None, what: str='', accept_m: float=0.0, direct: bool=False)`

*groundfire_lite.py 第 84 行首次用到 · 航线飞行*

飞到一个**世界坐标**上方并锁高（走 ego_planner，有避障）。

**参数**

- wx: 目标点的世界坐标 x（米）。
- wy: 目标点的世界坐标 y（米）。
- agl_m: 飞过去之后锁住的离地高度（米）。不给就用当前高度。
- what: 这个点叫什么，只用于日志（"飞往{what} (x, y)"）。
- accept_m: 判成"到不了"但其实已经在这个距离以内时按到达处理。 默认 0 = 不容忍、照常抛异常。目标点贴着障碍或刚被别的飞机占过 时规划器会把终端推到膨胀区边缘，给个 1~2 米的容忍更实用。
- direct: True = 走直线（`goto_direct`），**不经规划器、没有避障**。

> ⚠️ direct=True 会关掉避障，只在算过余量的航段上开（tools/check_route.py）。航线上的避障本身是考核点，不能为了快绕过去。

<a id="grip"></a>
### `grip(release: bool=False, label: str='')`

*groundfire_lite.py 第 86 行首次用到 · 抓放与发射*

驱动机械抓。release=False 抓紧，True 松开（打开爪子）。

**参数**

- release: False = 抓紧（该路 `pwm_action`），True = 打开（`pwm_idle`）。
- label: 日志里怎么称呼这个动作（比如 '抓取灭火弹'）。 不给就按 release 自动用"抓取"/"松开"。

<a id="snapshot"></a>
### `snapshot(tag: str, camera: str='front', directory: Optional[str]=None)`

*groundfire_lite.py 第 87 行首次用到 · 拍照*

拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。

**参数**

- tag: 这张照片叫什么，会进文件名（`{机号}_{时分秒}_{tag}.png`）。
- camera: 用哪个相机，'front'（前视）或 'down'（下视）。 两个都是**固定安装**的独立相机，不存在"切视角"。
- directory: 存到哪个目录。不给就用 `self.PHOTO_DIR`。

> ⚠️ 不要给拍照配声光事件：声光事件是固定枚举，自造事件名会直接抛 ValueError 把整个任务打断。

<a id="follow_formation"></a>
### `follow_formation(spacing_m: Optional[float]=None, agl_m: float=2.0, join: str='station', route_wait_s: float=60.0, done_wait_s: float=600.0, turn_at_waypoint: bool=False)`

*groundfire_lite.py 第 90 行首次用到 · 编队*

僚机跟队。**只管空中段**，起降由调用方自己决定。

**参数**

- spacing_m: 跟在长机后方多少米。**不给就用 `self.spacing_m`**， 跟长机那边同一个来源（都来自 `--spacing`），天然一致。
- agl_m: 入列和跟队时的离地高度（米）。
- join: `'station'` 先飞到"航线起点后方 spacing 米"的站位点再入列 （编队从头开始时用）；`'nearest'` 就地入列（任务流程里僚机刚 做完事就在长机附近，再飞一趟站位点纯属绕路）。
- route_wait_s: 等长机下发航线的上限（秒）。等不到就只跟队、不做 分段航向，机头全程不变。
- done_wait_s: 等长机发"解散"的上限（秒）。等不到也会自己出列， 免得长机那边出问题时僚机永远挂着。
- turn_at_waypoint: 拐点要不要把机头转到下一段航向。**默认 False=不转** （2026-10-05 跟长机 `lead_formation()` 的同名参数对齐，默认值 也一样，两边一起不转才不会出现"长机机头不动、僚机在拐点转"的 不一致）。 · False：传 `turn_in_place=False` 给 `formation_follower_node` （也就是把它的 `yaw_follow_leg` 关掉），僚机全程机头不变； 入列前飞站位点那一次 `face_yaw()` 也一并跳过。 · True：拐点先停下转到该段航向再前飞。⚠️ 那条路的门控 （`_yaw_turning`）**只有"机头真的转到位"一个出口、没有超时逃生**， 航向指令一旦不生效（例如 PX4CTRL_YAW_LOCK_ENABLED=true）僚机会 永久悬停——2026-10-04 真机就是这么挂的。 ⚠️ 不管这一项取什么值，`leg_route` 都照常下发：它除了定航向，还 用来算**航线里程尺**（清洗长机轨迹）和**拐点前馈收油** （`corner_ff_radius_m`），关掉航向不等于不需要航线。

<a id="return_home"></a>
### `return_home(land: bool=True, agl_m: float=2.0, sound: Optional[str]=None, report: Optional[str]=None, direct: bool=False)`

*groundfire_lite.py 第 91 行首次用到 · 起飞 / 降落 / 返航*

回自己的起飞点，默认降落。返回**是否真的落在自己的起降点上**。

**参数**

- land: True = 到位后降落（默认）；False = 只飞回起飞点上空悬停。
- agl_m: 飞回去时锁的离地高度（米）。
- sound: 核对落点通过后才播的声光事件。**不要在调用方自己播**—— 任务机全程要降落三次（取器材、放器材、回家），"降落动作完成" 本身说明不了任务结束，得按落点坐标判。
- report: 给队友发的事件名，带 x/y/ok 三个字段。落点不准也发， 否则队友会一直等到超时。
- direct: True = 整段走直线，不经规划器。含义和前提同 `goto_world()` ——**必须先算过这条直线的余量**。编队解散之后各自回家那一段 适合开（算过：D->各自起降点最小余量 3.00 m，离墙最近）。

> ⚠️ 自带落点核对：落在起降点上才播 sound、才算数。任务流程里会降落好几次（取器材、放器材、回家），“降落动作完成”本身说明不了任务结束。

<a id="run"></a>
### `run(leader: Any, follower: Any, description: Optional[str]=None, spacing_m: float=5.0)`

*groundfire_lite.py 第 95 行首次用到 · 程序入口*  ·  *staticmethod*

选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。

**参数**

- leader: 长机那一支要跑的函数，签名是 `f(sdk)`。 `--role` 是 leader 或 recon 时调它。
- follower: 僚机那一支要跑的函数，签名同上。
- description: 命令行 `--help` 里显示的说明，一般传 `__doc__`。
- spacing_m: `--spacing` 的默认值（米）。实际取到的值会存成 `sdk.spacing_m`，任务函数直接读。

