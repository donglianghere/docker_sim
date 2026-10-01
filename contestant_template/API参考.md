# contest_sdk API 参考

> 由 `scripts/gen_sdk_api_doc.py` 从 `capabilities.py` 自动生成，**不要手改**。
> 改了 SDK 之后重新跑一遍这个脚本。

`DroneSDK` 共 80 个公开方法。构造：

```python
from contest_sdk import DroneSDK

sdk = DroneSDK(namespace='NX01', role='recon', teammate_namespace='NX02')
```

命令行参数的解析、按角色分派和收尾，用 `DroneSDK.run()` 就够了：

```python
if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply)
```


## 常用的 20 个

一个典型任务基本只用到这些，按任务里出现的先后排：

| 方法 | 做什么 |
|---|---|
| [`run()`](#run) | 选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。 |
| [`takeoff()`](#takeoff) | 起飞：发布`TakeoffLand{TAKEOFF}`，阻塞直到`armed=True`**且**飞控自己的起飞状态机真正爬升到位、转入稳定悬停——不是`armed=True`就立刻返回。 |
| [`fly_route()`](#fly_route) | 按航点序列飞：**每个航点先把机头转到下一段方向、停住，再走**，航段之间航向不变。 |
| [`goto_world()`](#goto_world) | 飞到一个**世界坐标**上方并锁高（走 ego_planner，有避障）。 |
| [`hold_at()`](#hold_at) | 飞到某个世界坐标悬停待命，**不降落**。 |
| [`return_home()`](#return_home) | 回自己的起飞点，默认降落。 |
| [`lead_formation()`](#lead_formation) | 长机带队飞一条航线。 |
| [`follow_formation()`](#follow_formation) | 僚机跟队。 |
| [`open_inbox()`](#open_inbox) | 注册这些跨机事件的收件箱。 |
| [`wait_event()`](#wait_event) | 等一个跨机事件，**返回它带来的数据**（dict，没有数据就是空 dict）。 |
| [`send_to_teammate()`](#send_to_teammate) | 发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛`TeammateUnreachableError`（方案2.3节"链路B"，`reliability.py`已经实现好ACK+1Hz重发协议，这里只是转调）。 |
| [`announce()`](#announce) | 播报一次声光事件。 |
| [`search_along()`](#search_along) | 从当前位置飞向 leg_end，**边飞边找**；看到就刹停、对准、解算坐标。 |
| [`aim_at()`](#aim_at) | 把目标**挪到画面正中**：横向平移 + 升降，机头朝向全程不变。 |
| [`patrol()`](#patrol) | 逐站巡检：飞到观察位 -> 转到指定机头朝向 -> 拍交付照片 -> （该查就查）。 |
| [`snapshot()`](#snapshot) | 拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。 |
| [`fetch_from()`](#fetch_from) | 飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。 |
| [`release_at()`](#release_at) | 飞到物资点 -> 边瞄边降到底 -> 松开 -> 起飞回巡航高度。 |
| [`grip()`](#grip) | 驱动机械抓。 |
| [`shoot()`](#shoot) | 发射弹丸：舵机推到发射位、等到位、再复位装填。 |

## 全部方法


### 程序入口

<a id="run"></a>
- **`run(leader: Any, follower: Any, description: Optional[str]=None, spacing_m: float=4.0)`**  *(staticmethod)*
  选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。
  > ⚠️ 选手程序的 main()：解析命令行、建 SDK、按角色分派、收尾。用法 `DroneSDK.run(leader=recon, follower=supply)`。
<a id="shutdown"></a>
- **`shutdown()`**
  选手程序整体退出时应该调用一次：先归还`reliability.py`占用的定时器/发布者/订阅者，再关闭`RclpyRuntime`（停止后台spin、销毁节点、必要时关闭rclpy全局context）。

### 起飞 / 降落 / 返航

<a id="takeoff"></a>
- **`takeoff(timeout: float=60.0, height_m: Optional[float]=None)`**
  起飞：发布`TakeoffLand{TAKEOFF}`，阻塞直到`armed=True`**且**飞控自己的起飞状态机真正爬升到位、转入稳定悬停——不是`armed=True`就立刻返回。
<a id="land"></a>
- **`land(timeout: float=30.0)`**
  降落：发布`TakeoffLand{LAND}`，阻塞直到`mavros/state`确认`armed`变为`False`。
  > ⚠️ 飞控偶尔不把 armed 置回 false，飞机明明已经在地上。任务流程里用`land_or_confirm()` 或 `return_home()`，它们兜住了这一条。
<a id="land_or_confirm"></a>
- **`land_or_confirm()`**
  降落；`land()` 等不到解锁确认时，自己核一下是不是其实已经落地了。
<a id="return_home"></a>
- **`return_home(land: bool=True, agl_m: float=2.0, sound: Optional[str]=None, report: Optional[str]=None, direct: bool=False)`**
  回自己的起飞点，默认降落。
<a id="own_pad"></a>
- **`own_pad()`**
  自己起降点的世界坐标。
<a id="land_on"></a>
- **`land_on(class_id: str, what: str='目标')`**
  边瞄准边降落到底：每降一小段就把目标位置重新解算一次，直接命令"目标正上方、低一点"那个位置——水平修正和下降在同一条指令里完成；降到交接高度后交给普通降落收尾。
<a id="precision_land_and_confirm"></a>
- **`precision_land_and_confirm(class_id: str, timeout: float=60.0)`**
  触发`precision_servo_node`的`precision_land`模式（精降抓取，收敛后下降到底并触发真正降落），阻塞直到`armed`确认变`False`（节点内部广播`servo_status='landed'`）。
<a id="precision_land_at"></a>
- **`precision_land_at(x: float, y: float, timeout: float=90.0)`**
  飞到一个**提前已知的坐标点**精确降落（2026-09-14新增，用户提出的场景：起飞前记录起降点坐标，返回降落时用精准降落，不是普通`goto()`+`land()`——普通`goto()`只保证收敛进`ARRIVAL_THRESHOLD_M`(0.3米)这个到点阈值，`land()`触发的`AUTO_LAND`只在触发那一刻的水平位置垂直下降、不再继续修正，如果`goto()`留下的水平误差没消掉，落地点也会跟着偏，对"精确停回起降点/对接点"这类场景不够精确）。

### 航线飞行

<a id="fly_route"></a>
- **`fly_route(waypoints: List[Tuple[float, float]], agl_m: float=2.0, hold_s: float=2.0, names: Optional[List[str]]=None)`**
  按航点序列飞：**每个航点先把机头转到下一段方向、停住，再走**，航段之间航向不变。
<a id="goto_world"></a>
- **`goto_world(wx: float, wy: float, agl_m: Optional[float]=None, what: str='', accept_m: float=0.0, direct: bool=False)`**
  飞到一个**世界坐标**上方并锁高（走 ego_planner，有避障）。
  > ⚠️ `direct=True` 会关掉避障，**只在算过余量的航段上开**（`scripts/check_route.py`）。航线上的避障本身是考核点，不能绕。
<a id="hold_at"></a>
- **`hold_at(wx: float, wy: float, agl_m: float=2.0, seconds: float=0.0, direct: bool=False)`**
  飞到某个世界坐标悬停待命，**不降落**。
<a id="goto"></a>
- **`goto(x: float, y: float, z: float, timeout: float=60.0)`**
  飞往一个点，阻塞直到到达确认或超时。
<a id="goto_route"></a>
- **`goto_route(points, timeout: float=300.0)`**
  一次下发整条航线，阻塞到最后一个航点到达确认或超时。
<a id="goto_direct"></a>
- **`goto_direct(x: float, y: float, z: float, timeout: Optional[float]=None)`**
  飞到一个坐标点并悬停（2026-09-14新增，`goto()`的"不避障、直达版"）——用户提出："coordinate_land不走ego_planner的话，就没有避障能力。
  > ⚠️ **不经规划器、没有避障**。传当前高度就是定高平飞。限速由`set_direct_speed()` 管，默认 0.3 m/s。
<a id="cancel_goto"></a>
- **`cancel_goto()`**
  打断当前正在进行的`goto()`（发布`Empty`到`waypoint_cancel`）。
<a id="set_agl"></a>
- **`set_agl(agl_m: float)`**
  原地升降到指定离地高度，水平位置不动（直线、不经规划器）。
<a id="step_forward"></a>
- **`step_forward(meters: float, what: str='落脚点')`**
  沿**当前机头方向**平移 meters 米，返回落脚点的 (世界x, 世界y, 离地高度)。
<a id="set_max_vel"></a>
- **`set_max_vel(max_vel_mps: float, timeout: float=5.0)`**
  改`ego_planner`的巡航限速，返回改之前的值（方便用完恢复）。
<a id="set_direct_speed"></a>
- **`set_direct_speed(max_speed_mps: float, timeout: float=5.0)`**
  改 `goto_direct()` 的限速（`precision_servo_node` 的`coordinate_max_speed_mps`），**飞行中可以随时改**。
  > ⚠️ 跟 `set_max_vel()` 是两条链路：那个管规划器（`goto`/`fly_route`/`lead_formation`），这个管直飞。实测 1.0 m/s 是拐点，再高反而慢。
<a id="set_fixed_altitude"></a>
- **`set_fixed_altitude(height_m: float)`**
  把飞行高度钉死在设定值，规划器轨迹里的高度变化不再生效。
<a id="fixed_altitude"></a>
- **`fixed_altitude(height_m: float)`**
  `with`块里飞的这段航线全程定高，块退出（含异常）自动关掉。
  > ⚠️ **只管 `goto()`**。`goto_direct()` / 精准降落 / 僚机编队跟随都是另一条旁路，不受它影响——那几条本来就是直接命令 z，不会飘。
<a id="set_fixed_altitude_off"></a>
- **`set_fixed_altitude_off()`**
  关掉定高，高度回到"按规划器轨迹走"（默认行为）。

### 朝向

<a id="face_yaw"></a>
- **`face_yaw(yaw_rad: float, timeout: float=15.0, tolerance_deg: float=5.0)`**
  悬停着把机头转到**指定航向角**，转到位（或超时）才返回，返回是否转到位。
<a id="face_point"></a>
- **`face_point(x: float, y: float, timeout: float=10.0, tolerance_deg: float=5.0)`**
  悬停着把机头转到对准某个点，**返回是否真的转到位了**（超时返回False）。
<a id="set_yaw_mode_constant"></a>
- **`set_yaw_mode_constant(yaw_rad: float)`**
  朝向固定成一个不变的角度（弧度，跟这架飞机局部坐标系的yaw=0方向约定一致），不随位置变化重新计算。
<a id="set_yaw_mode_point"></a>
- **`set_yaw_mode_point(x: float, y: float)`**
  朝向持续指向一个固定目标点，飞机每飞到新位置都会重新算一次朝向那个点的角度（绕着这个点转一圈，机头全程对着它）。
<a id="set_yaw_mode_velocity"></a>
- **`set_yaw_mode_velocity()`**
  朝向跟随速度方向（`ego_planner`原有算法，飞哪个方向机头就转向哪个方向）。
<a id="get_current_yaw"></a>
- **`get_current_yaw(timeout: float=10.0)`**
  读取自己当前的实际yaw角（弧度，局部坐标系，跟`goto()`/`set_yaw_mode_constant()`同一套约定），从里程计四元数换算。

### 位置与坐标

<a id="get_local_position"></a>
- **`get_local_position(timeout: float=10.0)`**
  读取自己当前的局部坐标`(x, y, z)`——`_odom_xyz`这个状态本来就已经在`_setup_transport()`里常驻订阅`dlio/odom_node/odom`缓存着（原来只在B6的进度打印里用），2026-09-13实现阶段D的mission.py时发现"选手需要知道自己当前在哪"这个需求本身没有一个公开方法可以调用（比如判断"队友是否已经飞完一段航线、可以停止跟随了"这类场景），补一个薄封装直接暴露出去，不需要选手自己猜测/绕开SDK边界。
<a id="get_agl"></a>
- **`get_agl(timeout: float=5.0)`**
  离地高度（米），来自机载朝下的定高雷达，不是里程计的 z。
<a id="world_to_local"></a>
- **`world_to_local(x: float, y: float, z: float, timeout: float=5.0)`**
  世界坐标 -> 这架飞机自己的局部坐标系（`{namespace}/odom`），只在SDK内部处理`frame_id`概念，对外只收发`(x, y, z)`三元组（方案2.2.1节要求，选手不需要知道TF/frame_id这些概念）。
<a id="local_to_world"></a>
- **`local_to_world(x: float, y: float, z: float, timeout: float=5.0)`**
  这架飞机自己的局部坐标系(`{namespace}/odom`) -> 世界坐标，跟`world_to_local()`反方向、同一套TF查询模式（只是`lookup_transform`的两个frame参数对调），2026-09-13阶段D单机实测时补的方法。

### 编队

<a id="lead_formation"></a>
- **`lead_formation(route: List[Tuple[float, float]], spacing_m: float=4.0, agl_m: float=2.0, hold_s: float=2.0, start_xy: Optional[Tuple[float, float]]=None, final_xy: Optional[Tuple[float, float]]=None, disband_at: Optional[Tuple[float, float]]=None, wait_follower_s: float=300.0, tail_direct: bool=False)`**
  长机带队飞一条航线。
<a id="follow_formation"></a>
- **`follow_formation(spacing_m: float=4.0, agl_m: float=2.0, join: str='station', route_wait_s: float=60.0, done_wait_s: float=600.0)`**
  僚机跟队。
<a id="teammate_formation_lag"></a>
- **`teammate_formation_lag()`**
  队友（僚机）沿长机轨迹的落后量，米。
<a id="start_formation_follow"></a>
- **`start_formation_follow(follow_distance_m: float, timeout: float=10.0, altitude_agl_m: Optional[float]=None, turn_in_place: Optional[bool]=None, leg_route: Optional[List[Tuple[float, float]]]=None)`**
  启用`formation_follower_node`（跟随目标固定是构造`DroneSDK`时传入的`teammate_namespace`，选手不需要指定跟谁——方案2.1节第7条："队友的namespace不是SDK自己猜的"）。
<a id="stop_formation_follow"></a>
- **`stop_formation_follow(timeout: float=10.0)`**
  停用编队跟随：出列并停止发布目标点。
<a id="set_formation_leg_route"></a>
- **`set_formation_leg_route(leg_route: List[Tuple[float, float]], timeout: float=10.0)`**
  把航线（世界坐标，第一个点是长机起飞点）下发给僚机的编队节点，用来定每一段的航向。

### 跨机协同

<a id="open_inbox"></a>
- **`open_inbox(*events: str)`**
  注册这些跨机事件的收件箱。
  > ⚠️ 可靠事件通道是**先回 ACK 再查处理函数**，没注册的事件会被确认后**丢弃**。任务一开始就把所有事件注册全，别等用到了再注册。
<a id="wait_event"></a>
- **`wait_event(event: str, timeout_s: float=300.0, clear: bool=True, required: bool=True)`**
  等一个跨机事件，**返回它带来的数据**（dict，没有数据就是空 dict）。
<a id="wait_any_event"></a>
- **`wait_any_event(events: List[str], timeout_s: float=300.0)`**
  等这几个事件里**先到的那一个**，返回 (事件名, 数据)。
<a id="event_ready"></a>
- **`event_ready(event: str)`**
  这个事件到了没有（不阻塞、不取走）。
<a id="send_to_teammate"></a>
- **`send_to_teammate(event: str, timeout_s: float=DEFAULT_SEND_TO_TEAMMATE_TIMEOUT_S, **kwargs: Any)`**
  发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛`TeammateUnreachableError`（方案2.3节"链路B"，`reliability.py`已经实现好ACK+1Hz重发协议，这里只是转调）。
<a id="on_teammate_event"></a>
- **`on_teammate_event(event_name: str, callback: Callable[..., None])`**
  注册"收到队友发来的某个事件时"的处理回调（方案2.3节"链路B"，转调`reliability.py::register_event_handler()`）。
<a id="yield_spot"></a>
- **`yield_spot(spot_xy: Tuple[float, float], event: str, timeout_s: Optional[float]=None)`**
  宣告"我马上让开这个点"，**真的让开了**就给队友发一次 event。

### 视觉：识别与对准

<a id="look_for"></a>
- **`look_for(class_id: str, camera: str='front', tries: Optional[int]=None, timeout_s: Optional[float]=None, what: str='目标')`**
  在**当前位置、当前朝向**找一次目标。
<a id="wait_for_detection"></a>
- **`wait_for_detection(class_id: str, timeout: float, camera: Optional[str]=None)`**
  阻塞等待`vision/detections`里出现`class_id`匹配的检测结果。
<a id="locate_target"></a>
- **`locate_target(class_id: str, timeout: float=5.0, samples: int=5)`**
  目标在地面上的**实际坐标**（飞机自己的局部系）。
<a id="search_along"></a>
- **`search_along(leg_end: Tuple[float, float], class_id: str, camera: str='down', agl_m: float=2.0, timeout_s: float=420.0, what: str='目标')`**
  从当前位置飞向 leg_end，**边飞边找**；看到就刹停、对准、解算坐标。
<a id="patrol"></a>
- **`patrol(stations: List[Tuple[Any, ...]], class_id: Optional[str]=None, agl_m: float=2.0, on_found: Optional[Any]=None, scan_sound: Optional[str]=None, found_sound: Optional[str]=None, once: bool=True)`**
  逐站巡检：飞到观察位 -> 转到指定机头朝向 -> 拍交付照片 -> （该查就查）。
<a id="aim_at"></a>
- **`aim_at(class_id: str, camera: str='front', face_yaw_deg: Optional[float]=None, what: str='目标')`**
  把目标**挪到画面正中**：横向平移 + 升降，机头朝向全程不变。
  > ⚠️ `camera='down'` 会自动改走 `center_on_target()`。前视那套几何把画面纵轴当成世界的高低，下视时纵轴其实是机体前后方向，照搬永远收敛不了。
<a id="center_on_target"></a>
- **`center_on_target(class_id: str, timeout: float=30.0)`**
  触发`precision_servo_node`的`center_only`模式（只居中定位，不下降），阻塞直到收到`'centered'`确认，返回收敛时的坐标。
  > ⚠️ 走 precision_servo_node，而它**只认下视相机**的检测（源码里 `if '_camera_down_' not in frame_id: return`）。
<a id="stop_precision_servo"></a>
- **`stop_precision_servo(timeout: float=5.0)`**
  让`precision_servo_node`退回待命（`servo_mode=''`），把控制权交还给正常的位置指令通路。
<a id="clear_detections"></a>
- **`clear_detections(camera: Optional[str]=None)`**
  丢掉已经缓存的检测结果，让下一次`wait_for_detection()`只认**之后**新收到的帧。

### 拍照

<a id="snapshot"></a>
- **`snapshot(tag: str, camera: str='front', directory: Optional[str]=None)`**
  拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。
  > ⚠️ **不要给拍照配声光事件**：声光事件是固定枚举，自造事件名会直接抛`ValueError` 把整个任务打断。
<a id="capture_photo"></a>
- **`capture_photo(path: str, camera: str='front', timeout: float=8.0, fresh: bool=True, timestamp: bool=True)`**
  抓当前相机的一帧存成 PNG，返回实际写入的路径。

### 抓放与发射

<a id="grip"></a>
- **`grip(release: bool=False, label: str='')`**
  驱动机械抓。
<a id="fetch_from"></a>
- **`fetch_from(wxy: Tuple[float, float], class_id: str, what: str='物资', agl_m: float=2.0, sound: Optional[str]=None, direct: bool=False)`**
  飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。
<a id="release_at"></a>
- **`release_at(wxy: Tuple[float, float], class_id: str, what: str='物资', agl_m: float=2.0, sound: Optional[str]=None, direct: bool=False)`**
  飞到物资点 -> 边瞄边降到底 -> 松开 -> 起飞回巡航高度。
<a id="shoot"></a>
- **`shoot(shots: int=1, interval_s: float=1.0, label: str='发射', sound: Optional[str]=None)`**
  发射弹丸：舵机推到发射位、等到位、再复位装填。
<a id="servos"></a>
- **`servos`**  *(property)*
  这架飞机上可以用`set_servo()`控制的舵机：{舵机编号: 配置}，没有舵机的飞机返回空字典。
<a id="set_servo"></a>
- **`set_servo(servo: int, pwm: int, timeout: float=5.0)`**
  把舵机转到指定PWM位置（微秒）。
<a id="set_servos"></a>
- **`set_servos(positions: Dict[int, int], timeout: float=5.0)`**
  同时设置多个舵机：一条指令发给飞控，几个舵机同一时刻开始转（分开调用`set_servo()`会差几十毫秒）。
<a id="set_actuator"></a>
- **`set_actuator(index: int, value: float, timeout: float=5.0)`**
  直接设置一路"Peripheral via Actuator Set"外设输出（比如舵机），走标准`MAV_CMD_DO_SET_ACTUATOR`（命令187）+`mavros/cmd/command`。
<a id="do_action"></a>
- **`do_action(name: str, timeout: float=30.0)`**
  触发`actuator_action_node`的一个符号化动作（比如`'grab_supply'`），阻塞直到收到`action_status`广播`f'done:{name}'`确认。

### 声光播报

<a id="announce"></a>
- **`announce(event: str)`**
  播报一次声光事件。
<a id="play_sound_light"></a>
- **`play_sound_light(event: str, repeat: Optional[int]=None, interval_ms: Optional[int]=None)`**
  触发地面站声光反馈板（WS2812D灯带+扬声器），播放20条预置事件之一。
<a id="mute_sound_light"></a>
- **`mute_sound_light()`**
  熄灯+静音（`0,0,0,0,1,0`）。
<a id="trigger_alarm"></a>
- **`trigger_alarm(pattern: str, duration_s: Optional[float]=None)`**
  触发GCS所在机器的声光装置（方案2.1节第9条/链路D）。

### 杂项 / 底层

<a id="progress"></a>
- **`progress(text: str)`**
  往日志里打一行带机号前缀的进度。
<a id="set_mission_state"></a>
- **`set_mission_state(state: str)`**
  发布任务状态到`mission_state`话题（`std_msgs/String`），封装`mission_state.py`契约（vendor版本，见本文件模块头说明）。
<a id="get_mission_state"></a>
- **`get_mission_state()`**
  返回最近一次`set_mission_state()`设置的状态字符串；如果这个`DroneSDK`实例从未调用过`set_mission_state()`，返回`None`（不是`'idle'`——`None`更清楚地表达"还没设置过"，不需要选手去猜`'idle'`是真的状态还是"默认值"这种歧义）。
<a id="generate_orbit_waypoints"></a>
- **`generate_orbit_waypoints(*args: Any, **kwargs: Any)`**
  
<a id="generate_ground_scan_waypoints"></a>
- **`generate_ground_scan_waypoints(*args: Any, **kwargs: Any)`**
  
<a id="pull_waypoints_out_of_circles"></a>
- **`pull_waypoints_out_of_circles(*args: Any, **kwargs: Any)`**
  把落进已知圆形障碍物（含余量）里的航点沿来路往回挪到外面。
<a id="reset_aim"></a>
- **`reset_aim(timeout: float=5.0)`**
  清空`fire_pillar_aim_node`的瞄准锁定状态（对应现成的`~/reset_aim` `std_srvs/srv/Trigger` service），阻塞等待service返回即可——不需要设计成通用的"调任意service"接口，专门封装这一个就够（清单B3第10条明确的要求）。
<a id="read_fire_pillar_staging_pose"></a>
- **`read_fire_pillar_staging_pose(timeout: float=30.0)`**
  读取`fire_pillar_aim_node`锁定高层火情后广播的任务机等待点坐标（`~/fire_pillar_staging_pose`，方案1.2节），返回`(x, y,yaw)`（局部坐标系，跟`sdk.goto()`的用法一致）。
<a id="read_fire_pillar_aim_pose"></a>
- **`read_fire_pillar_aim_pose(timeout: float=30.0)`**
  读取`fire_pillar_aim_node`锁定高层火情后广播的瞄准点坐标（`~/fire_pillar_aim_pose`，方案1.2节/A1新增），返回`(x, y,yaw)`（局部坐标系）——内容跟`fire_pillar_aim_cmd`（驱动`position_cmd_relay`的`pillar_aim`模式那条控制指令）一致，但这是专门给第四部分程序读的只读广播，语义上是"状态"不是"指令"。

---

每个方法的完整说明（为什么这么设计、踩过什么坑）在 `capabilities.py` 的 docstring 里，这里只放一句话摘要。

