# contest_sdk API 参考（选手版）

> **只收录四个选手程序真正用过的 API**，共 28 个。SDK 全部能力有 80 个，其余是底层/备用接口，写任务一般用不到。
> 本文档由 `scripts/gen_sdk_api_doc.py` 自动生成，**不要手改**；改了选手程序或 SDK 重新跑一遍即可。


## 怎么开始

命令行解析、建 SDK、按角色分派、收尾，一句就够：

```python
if __name__ == '__main__':
    DroneSDK.run(leader=recon, follower=supply)
```


## 速查表

| 方法 | 用在哪个程序 | 做什么 |
|---|---|---|
| [`run`](#run) | formation、groundfire、highrise、mission | 选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。 |
| [`spacing_m`](#spacing_m) | formation、groundfire、highrise、mission | run() 把命令行 --spacing 存成这个属性，任务函数直接读。 |
| [`takeoff`](#takeoff) | formation、groundfire、highrise、mission | 起飞：发布TakeoffLand{TAKEOFF}，阻塞直到armed=True且飞控自己的起飞状态机真正爬升到位、转入 |
| [`return_home`](#return_home) | formation、groundfire、highrise、mission | 回自己的起飞点，默认降落。返回是否真的落在自己的起降点上。 |
| [`own_pad`](#own_pad) | mission | 自己起降点的世界坐标。起飞时飞机就在起降点上，局部原点换过去就是。 |
| [`fly_route`](#fly_route) | groundfire、highrise、mission | 按航点序列飞：每个航点先把机头转到下一段方向、停住，再走，航段之间航向不变。 |
| [`goto_world`](#goto_world) | groundfire、highrise、mission | 飞到一个世界坐标上方并锁高（走 ego_planner，有避障）。 |
| [`hold_at`](#hold_at) | formation、groundfire、highrise、mission | 飞到某个世界坐标悬停待命，不降落。seconds>0 就停够这么久再返回。 |
| [`set_agl`](#set_agl) | highrise、mission | 原地升降到指定离地高度，水平位置不动（直线、不经规划器）。 |
| [`step_forward`](#step_forward) | highrise、mission | 沿当前机头方向平移 meters 米，返回落脚点的 (世界x, 世界y, 离地高度)。 |
| [`lead_formation`](#lead_formation) | formation、groundfire、highrise、mission | 长机带队飞一条航线。只管空中段，起降由调用方自己决定。 |
| [`follow_formation`](#follow_formation) | formation、groundfire、highrise、mission | 僚机跟队。只管空中段，起降由调用方自己决定。 |
| [`open_inbox`](#open_inbox) | groundfire、highrise、mission | 注册这些跨机事件的收件箱。在任务一开始就全部注册，见上面说明。 |
| [`wait_event`](#wait_event) | groundfire、highrise、mission | 等一个跨机事件，返回它带来的数据（dict，没有数据就是空 dict）。 |
| [`wait_any_event`](#wait_any_event) | highrise | 等这几个事件里先到的那一个，返回 (事件名, 数据)。 |
| [`send_to_teammate`](#send_to_teammate) | groundfire、highrise、mission | 发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛TeammateUnreachableError（方案2. |
| [`yield_spot`](#yield_spot) | highrise、mission | 宣告"我马上让开这个点"，真的让开了就给队友发一次 event。 |
| [`search_along`](#search_along) | groundfire、mission | 从当前位置飞向 leg_end，边飞边找；看到就刹停、对准、解算坐标。 |
| [`patrol`](#patrol) | highrise、mission | 逐站巡检：飞到观察位 -> 转到指定机头朝向 -> 拍交付照片 -> （该查就查）。 |
| [`aim_at`](#aim_at) | groundfire、highrise、mission | 把目标挪到画面正中：横向平移 + 升降，机头朝向全程不变。返回是否对上。 |
| [`snapshot`](#snapshot) | groundfire、highrise、mission | 拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。 |
| [`PHOTO_DIR`](#photo_dir) | groundfire、highrise、mission | snapshot() 的存图目录，用实例属性覆盖即可：sdk.PHOTO_DIR = '/logs/xxx'。 |
| [`grip`](#grip) | groundfire、mission | 驱动机械抓。release=False 抓紧，True 松开。 |
| [`fetch_from`](#fetch_from) | groundfire、highrise、mission | 飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。 |
| [`release_at`](#release_at) | highrise、mission | 飞到物资点 -> 边瞄边降到底 -> 松开 -> 起飞回巡航高度。 |
| [`shoot`](#shoot) | highrise、mission | 发射弹丸：舵机推到发射位、等到位、再复位装填。shots>1 就连发。 |
| [`announce`](#announce) | formation、groundfire、highrise、mission | 播报一次声光事件。play_sound_light() 的别名，名字更贴近用途。 |
| [`progress`](#progress) | mission | 往日志里打一行带机号前缀的进度。选手程序里 print(f'[{ns}] ...')满篇都是，用这个就不必每次自己拼前缀 |

## 逐个说明


### 程序入口

<a id="run"></a>
**`run(leader: Any, follower: Any, description: Optional[str]=None, spacing_m: float=4.0)`**  *(staticmethod)*

选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。

*用在：formation、groundfire、highrise、mission*

<a id="spacing_m"></a>
**`spacing_m`**

> ⚠️ run() 把命令行 --spacing 存成这个属性，任务函数直接读。

*用在：formation、groundfire、highrise、mission*


### 起飞 / 降落 / 返航

<a id="takeoff"></a>
**`takeoff(timeout: float=60.0, height_m: Optional[float]=None)`**

起飞：发布`TakeoffLand{TAKEOFF}`，阻塞直到`armed=True`**且**飞控自己的起飞状态机真正爬升到位、转入稳定悬停——不是`armed=True`就立刻返回。

- timeout: 总超时秒数，同时覆盖"等`armed=True`"和"等位置 稳定"这两段。默认从30秒提到60秒——实测确认这套仿真 （`uwb_imu`+`pt4ctrl`组合）从解锁到真正稳定悬停，中间 有一段爬升超调+振荡衰减的过程，衰减到0.3米容差以内 经常要20~30秒量级，30秒的旧默认值不够用。

*用在：formation、groundfire、highrise、mission*

<a id="return_home"></a>
**`return_home(land: bool=True, agl_m: float=2.0, sound: Optional[str]=None, report: Optional[str]=None, direct: bool=False)`**

回自己的起飞点，默认降落。返回**是否真的落在自己的起降点上**。

- sound: 核对落点通过后才播的声光事件。**不要在调用方自己播**—— 任务机全程要降落三次（取器材、放器材、回家），"降落动作完成" 本身说明不了任务结束，得按落点坐标判。
- report: 给队友发的事件名，带 x/y/ok 三个字段。落点不准也发， 否则队友会一直等到超时。
- direct: True = 整段走直线，不经规划器。含义和前提同 `goto_world()` ——**必须先算过这条直线的余量**。编队解散之后各自回家那一段 适合开（算过：D->各自起降点最小余量 3.00 m，离墙最近）。

> ⚠️ 自带落点核对：落在起降点上才播 sound、才算数。任务流程里会降落好几次（取器材、放器材、回家），“降落动作完成”本身说明不了任务结束。

*用在：formation、groundfire、highrise、mission*

<a id="own_pad"></a>
**`own_pad()`**

自己起降点的世界坐标。起飞时飞机就在起降点上，局部原点换过去就是。

*用在：mission*


### 航线飞行

<a id="fly_route"></a>
**`fly_route(waypoints: List[Tuple[float, float]], agl_m: float=2.0, hold_s: float=2.0, names: Optional[List[str]]=None)`**

按航点序列飞：**每个航点先把机头转到下一段方向、停住，再走**，航段之间航向不变。

- waypoints: [(wx, wy), ...] 世界坐标。
- agl_m: 全程锁的离地高度。
- hold_s: 每个航点停多久（转向跟停顿同时进行，不足的部分补足）。
- names: 航点名字，只用于日志；不给就按序号。

*用在：groundfire、highrise、mission*

<a id="goto_world"></a>
**`goto_world(wx: float, wy: float, agl_m: Optional[float]=None, what: str='', accept_m: float=0.0, direct: bool=False)`**

飞到一个**世界坐标**上方并锁高（走 ego_planner，有避障）。

- accept_m: 判成"到不了"但其实已经在这个距离以内时按到达处理。 默认 0 = 不容忍、照常抛异常。目标点贴着障碍或刚被别的飞机占过 时规划器会把终端推到膨胀区边缘，给个 1~2 米的容忍更实用。
- direct: True = 走直线（`goto_direct`），**不经规划器、没有避障**。

> ⚠️ direct=True 会关掉避障，只在算过余量的航段上开（tools/check_route.py）。航线上的避障本身是考核点，不能为了快绕过去。

*用在：groundfire、highrise、mission*

<a id="hold_at"></a>
**`hold_at(wx: float, wy: float, agl_m: float=2.0, seconds: float=0.0, direct: bool=False)`**

飞到某个世界坐标悬停待命，**不降落**。seconds>0 就停够这么久再返回。

*用在：formation、groundfire、highrise、mission*

<a id="set_agl"></a>
**`set_agl(agl_m: float)`**

原地升降到指定离地高度，水平位置不动（直线、不经规划器）。

*用在：highrise、mission*

<a id="step_forward"></a>
**`step_forward(meters: float, what: str='落脚点')`**

沿**当前机头方向**平移 meters 米，返回落脚点的 (世界x, 世界y, 离地高度)。

*用在：highrise、mission*


### 编队

<a id="lead_formation"></a>
**`lead_formation(route: List[Tuple[float, float]], spacing_m: float=4.0, agl_m: float=2.0, hold_s: float=2.0, start_xy: Optional[Tuple[float, float]]=None, final_xy: Optional[Tuple[float, float]]=None, disband_at: Optional[Tuple[float, float]]=None, wait_follower_s: float=300.0, tail_direct: bool=False)`**

长机带队飞一条航线。**只管空中段**，起降由调用方自己决定。

- route: 航线（世界坐标）。
- spacing_m: 目标纵向间距。
- start_xy: 从哪儿起步（默认自己当前位置换算成的起飞点）。编队段从 半路开始时要给，否则僚机算出来的起始站位会跑到场外。
- final_xy: 航线跑完再飞到哪儿（默认 route[0]）。
- disband_at: 给了就按"**僚机过了这个航点**"解散；不给按"长机飞回 自己起飞点上空"解散。
- wait_follower_s: 等僚机到站位的上限，等不到也照飞。
- tail_direct: 最后一段（飞往 final_xy 那一段）走直线。 **解散就发生在这一段上**——disband_at 的判据是"长机已经离开 那个航点 spacing+lag 米"，所以长机走到这一段中途才解散，剩下 的路本质上是"解散后各自回家"。前提同 `goto_world()` 的 direct：必须算过这条直线的余量。

> ⚠️ disband_at 要显式给。不给的话默认判据是“长机飞回自己起飞点上空”，而起飞点不一定在航线上，编队可能在半路散掉。

*用在：formation、groundfire、highrise、mission*

<a id="follow_formation"></a>
**`follow_formation(spacing_m: float=4.0, agl_m: float=2.0, join: str='station', route_wait_s: float=60.0, done_wait_s: float=600.0)`**

僚机跟队。**只管空中段**，起降由调用方自己决定。

- join: `'station'` 先飞到"航线起点后方 spacing 米"的站位点再入列 （编队从头开始时用）；`'nearest'` 就地入列（任务流程里僚机刚 做完事就在长机附近，再飞一趟站位点纯属绕路）。

*用在：formation、groundfire、highrise、mission*


### 跨机协同

<a id="open_inbox"></a>
**`open_inbox(*events: str)`**

注册这些跨机事件的收件箱。**在任务一开始就全部注册**，见上面说明。

> ⚠️ 可靠事件通道是“先回 ACK 再查处理函数”，没注册的事件会被确认后丢弃。任务一开始就把所有事件注册全，别等用到了再注册。

*用在：groundfire、highrise、mission*

<a id="wait_event"></a>
**`wait_event(event: str, timeout_s: float=300.0, clear: bool=True, required: bool=True)`**

等一个跨机事件，**返回它带来的数据**（dict，没有数据就是空 dict）。

- event: 事件名，必须先 `open_inbox()` 注册过。
- timeout_s: 等多久。
- clear: 取走后把收件箱复位（默认 True）。多轮流程里同一个事件会来 好几次，不复位的话第二轮一进来就立刻返回上一轮的旧数据。
- required: False = 超时就**返回 None**，不抛异常。用在"等到更好、 等不到也得往下走"的地方——比如最后等队友报告已降落，等不到 也该把任务完成播出去，不能让整个任务在这一步失败。

*用在：groundfire、highrise、mission*

<a id="wait_any_event"></a>
**`wait_any_event(events: List[str], timeout_s: float=300.0)`**

等这几个事件里**先到的那一个**，返回 (事件名, 数据)。

*用在：highrise*

<a id="send_to_teammate"></a>
**`send_to_teammate(event: str, timeout_s: float=DEFAULT_SEND_TO_TEAMMATE_TIMEOUT_S, **kwargs: Any)`**

发给队友一个事件，阻塞直到对方确认收到（应用层ACK），超时抛`TeammateUnreachableError`（方案2.3节"链路B"，`reliability.py`已经实现好ACK+1Hz重发协议，这里只是转调）。

- event: 事件名字符串（选手自定义，需要跟队友`on_teammate_ event()`注册的名字完全一致）。
- timeout_s: 总超时秒数，默认45秒（方案建议区间30-60秒的 中间值），可以覆盖。
- **kwargs: 随事件一起带给对方的业务数据（必须是JSON能表示的 类型）。

*用在：groundfire、highrise、mission*

<a id="yield_spot"></a>
**`yield_spot(spot_xy: Tuple[float, float], event: str, timeout_s: Optional[float]=None)`**

宣告"我马上让开这个点"，**真的让开了**就给队友发一次 event。

*用在：highrise、mission*


### 视觉：识别与对准

<a id="search_along"></a>
**`search_along(leg_end: Tuple[float, float], class_id: str, camera: str='down', agl_m: float=2.0, timeout_s: float=420.0, what: str='目标')`**

从当前位置飞向 leg_end，**边飞边找**；看到就刹停、对准、解算坐标。

*用在：groundfire、mission*

<a id="patrol"></a>
**`patrol(stations: List[Tuple[Any, ...]], class_id: Optional[str]=None, agl_m: float=2.0, on_found: Optional[Any]=None, scan_sound: Optional[str]=None, found_sound: Optional[str]=None, once: bool=True)`**

逐站巡检：飞到观察位 -> 转到指定机头朝向 -> 拍交付照片 -> （该查就查）。

- stations: `[(标签, (wx, wy), 观察位名, 机头朝向°, 这站要不要查), ...]`。 连续两站是同一个观察位时不重复飞，只原地转向（任务3 的 M 点 要先朝南拍 3# 楼、再原地转 180° 查 2# 楼）。
- on_found: `on_found(det, 标签)`。找到目标时调用，**这一站的照片由它 负责拍**——要等它把目标对准到画面正中再拍，火情在画面边上 等于没拍到。它很可能把飞机挪走（比如前移到发射点），所以回来 之后不再认为飞机还在观察位上。
- once: 找到一次之后，后面要查的站只补拍照片、不再跑识别。任务3 里 火情只可能有一处，灭完了就不必在剩下的楼前再等识别超时。

*用在：highrise、mission*

<a id="aim_at"></a>
**`aim_at(class_id: str, camera: str='front', face_yaw_deg: Optional[float]=None, what: str='目标')`**

把目标**挪到画面正中**：横向平移 + 升降，机头朝向全程不变。返回是否对上。

- face_yaw_deg: 给了就先把机头转到这个朝向再对准。**正对立面的场景 一定要给**——本方法是"锁住当前朝向只做平移"的，朝向不对的话 画面里居中了、机身却斜着，弹丸打出去也是斜的。

> ⚠️ camera='down' 会自动改走 precision_servo 闭环。前视那套几何把画面纵轴当成世界的高低，下视时纵轴其实是机体前后方向，照搬永远收敛不了。

*用在：groundfire、highrise、mission*


### 拍照

<a id="snapshot"></a>
**`snapshot(tag: str, camera: str='front', directory: Optional[str]=None)`**

拍一张交付照片存进 /logs 并打印路径（"回传"就是存进这个挂载目录）。

> ⚠️ 不要给拍照配声光事件：声光事件是固定枚举，自造事件名会直接抛 ValueError 把整个任务打断。

*用在：groundfire、highrise、mission*

<a id="photo_dir"></a>
**`PHOTO_DIR`**

> ⚠️ snapshot() 的存图目录，用实例属性覆盖即可：sdk.PHOTO_DIR = '/logs/xxx'。

*用在：groundfire、highrise、mission*


### 抓放与发射

<a id="grip"></a>
**`grip(release: bool=False, label: str='')`**

驱动机械抓。release=False 抓紧，True 松开。

*用在：groundfire、mission*

<a id="fetch_from"></a>
**`fetch_from(wxy: Tuple[float, float], class_id: str, what: str='物资', agl_m: float=2.0, sound: Optional[str]=None, direct: bool=False)`**

飞到物资点 -> 边瞄边降到底 -> 抓取 -> 起飞回巡航高度。

- sound: 声光播报内容。**播报点在落地之后、舵机动作之前**——裁判 听到的那一声要对上"正在抓"这个瞬间；写在 `fetch_from()` 前面会提前十几秒（还在飞往物资点的路上就播了）。

*用在：groundfire、highrise、mission*

<a id="release_at"></a>
**`release_at(wxy: Tuple[float, float], class_id: str, what: str='物资', agl_m: float=2.0, sound: Optional[str]=None, direct: bool=False)`**

飞到物资点 -> 边瞄边降到底 -> 松开 -> 起飞回巡航高度。

- sound: 同 `fetch_from()`，落地后、舵机动作前播报。

*用在：highrise、mission*

<a id="shoot"></a>
**`shoot(shots: int=1, interval_s: float=1.0, label: str='发射', sound: Optional[str]=None)`**

发射弹丸：舵机推到发射位、等到位、再复位装填。shots>1 就连发。

- sound: 第一发之前播的声光事件。连发只播一次。

> ⚠️ 连发时 sound 只播一次，播报点在第一发之前。

*用在：highrise、mission*


### 声光播报

<a id="announce"></a>
**`announce(event: str)`**

播报一次声光事件。`play_sound_light()` 的别名，名字更贴近用途。

*用在：formation、groundfire、highrise、mission*


### 日志

<a id="progress"></a>
**`progress(text: str)`**

往日志里打一行带机号前缀的进度。选手程序里 `print(f'[{ns}] ...')`满篇都是，用这个就不必每次自己拼前缀、也不会漏 flush=True。

*用在：mission*

