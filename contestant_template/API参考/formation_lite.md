# formation_lite.py 用到的 SDK API

> 这个程序用到 **8 个** API（SDK 全部能力有 80 个，其余是底层/备用接口，这个程序没用到）。
> 条目按**程序里出现的先后**排，括号里是首次用到的行号——对着 `formation_lite.py` 从上往下读，顺序能对上。
> 本文档由 `scripts/gen_sdk_api_doc.py` 自动生成，**不要手改**。


## 速查

| 行 | API | 分类 | 做什么 |
|---|---|---|---|
| 20 | [`takeoff`](#takeoff) | 起飞 / 降落 / 返航 | 起飞：发布TakeoffLand{TAKEOFF}，阻塞直到armed=True且飞控自己的起飞状态机真 |
| 24 | [`lead_formation`](#lead_formation) | 编队 | 长机带队飞一条航线。只管空中段，起降由调用方自己决定。 |
| 24 | [`spacing_m`](#spacing_m) | 程序入口 | run() 把命令行 --spacing 存成这个属性，任务函数直接读。 |
| 26 | [`hold_at`](#hold_at) | 航线飞行 | 飞到某个世界坐标悬停待命，不降落。seconds>0 就停够这么久再返回。 |
| 27 | [`announce`](#announce) | 声光播报 | 播报一次声光事件。play_sound_light() 的别名，名字更贴近用途。 |
| 33 | [`follow_formation`](#follow_formation) | 编队 | 僚机跟队。只管空中段，起降由调用方自己决定。 |
| 34 | [`return_home`](#return_home) | 起飞 / 降落 / 返航 | 回自己的起飞点，默认降落。返回是否真的落在自己的起降点上。 |
| 38 | [`run`](#run) | 程序入口 | 选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。 |

## 逐个说明

<a id="takeoff"></a>
### `takeoff(timeout: float=60.0, height_m: Optional[float]=None)`

*formation_lite.py 第 20 行首次用到 · 起飞 / 降落 / 返航*

起飞：发布`TakeoffLand{TAKEOFF}`，阻塞直到`armed=True`**且**飞控自己的起飞状态机真正爬升到位、转入稳定悬停——不是`armed=True`就立刻返回。

**参数**

- timeout: 总超时秒数，同时覆盖"等`armed=True`"和"等位置 稳定"这两段。默认从30秒提到60秒——实测确认这套仿真 （`uwb_imu`+`pt4ctrl`组合）从解锁到真正稳定悬停，中间 有一段爬升超调+振荡衰减的过程，衰减到0.3米容差以内 经常要20~30秒量级，30秒的旧默认值不够用。
- height_m: 起飞到多高（米，离地）。不给就用飞控 `pt4ctrl` 里 配置的 takeoff_height。

<a id="lead_formation"></a>
### `lead_formation(route: Sequence[Tuple[float, float]], spacing_m: float=4.0, agl_m: float=2.0, hold_s: float=2.0, start_xy: Optional[Tuple[float, float]]=None, final_xy: Optional[Tuple[float, float]]=None, disband_at: Optional[Tuple[float, float]]=None, wait_follower_s: float=300.0, tail_direct: bool=False)`

*formation_lite.py 第 24 行首次用到 · 编队*

长机带队飞一条航线。**只管空中段**，起降由调用方自己决定。

**参数**

- route: 航线（世界坐标）。
- spacing_m: 目标纵向间距。
- agl_m: 全程锁的离地高度（米）。
- hold_s: 每个航点停多久（秒）。转向跟停顿同时进行，不足的部分补足。
- start_xy: 从哪儿起步（默认自己当前位置换算成的起飞点）。编队段从 半路开始时要给，否则僚机算出来的起始站位会跑到场外。
- final_xy: 航线跑完再飞到哪儿（默认 route[0]）。
- disband_at: 给了就按"**僚机过了这个航点**"解散；不给按"长机飞回 自己起飞点上空"解散。
- wait_follower_s: 等僚机到站位的上限，等不到也照飞。
- tail_direct: 最后一段（飞往 final_xy 那一段）走直线。 **解散就发生在这一段上**——disband_at 的判据是"长机已经离开 那个航点 spacing+lag 米"，所以长机走到这一段中途才解散，剩下 的路本质上是"解散后各自回家"。前提同 `goto_world()` 的 direct：必须算过这条直线的余量。

> ⚠️ disband_at 要显式给。不给的话默认判据是“长机飞回自己起飞点上空”，而起飞点不一定在航线上，编队可能在半路散掉。

<a id="spacing_m"></a>
### `spacing_m`

*formation_lite.py 第 24 行首次用到 · 程序入口*

run() 把命令行 --spacing 存成这个属性，任务函数直接读。

<a id="hold_at"></a>
### `hold_at(wx: float, wy: float, agl_m: float=2.0, seconds: float=0.0, direct: bool=False)`

*formation_lite.py 第 26 行首次用到 · 航线飞行*

飞到某个世界坐标悬停待命，**不降落**。seconds>0 就停够这么久再返回。

**参数**

- wx: 待命点的世界坐标 x（米）。
- wy: 待命点的世界坐标 y（米）。
- agl_m: 悬停的离地高度（米）。
- seconds: 到位后再停多久（秒）。0 = 到了就返回，不额外等。
- direct: True = 走直线（`goto_direct`），**不经规划器、没有避障**。 含义和前提同 `goto_world()`——必须先算过这条直线的余量。

<a id="announce"></a>
### `announce(event: str)`

*formation_lite.py 第 27 行首次用到 · 声光播报*

播报一次声光事件。`play_sound_light()` 的别名，名字更贴近用途。

**参数**

- event: 声光事件名。**必须是固定枚举里的一项**（见 `_sound_light_port.py` 的 SOUND_LIGHT_EVENTS），自造名字会 直接抛 ValueError 把整个任务打断。

<a id="follow_formation"></a>
### `follow_formation(spacing_m: float=4.0, agl_m: float=2.0, join: str='station', route_wait_s: float=60.0, done_wait_s: float=600.0)`

*formation_lite.py 第 33 行首次用到 · 编队*

僚机跟队。**只管空中段**，起降由调用方自己决定。

**参数**

- spacing_m: 跟在长机后方多少米。要跟长机那边给的一致。
- agl_m: 入列和跟队时的离地高度（米）。
- join: `'station'` 先飞到"航线起点后方 spacing 米"的站位点再入列 （编队从头开始时用）；`'nearest'` 就地入列（任务流程里僚机刚 做完事就在长机附近，再飞一趟站位点纯属绕路）。
- route_wait_s: 等长机下发航线的上限（秒）。等不到就只跟队、不做 分段航向，机头全程不变。
- done_wait_s: 等长机发"解散"的上限（秒）。等不到也会自己出列， 免得长机那边出问题时僚机永远挂着。

<a id="return_home"></a>
### `return_home(land: bool=True, agl_m: float=2.0, sound: Optional[str]=None, report: Optional[str]=None, direct: bool=False)`

*formation_lite.py 第 34 行首次用到 · 起飞 / 降落 / 返航*

回自己的起飞点，默认降落。返回**是否真的落在自己的起降点上**。

**参数**

- land: True = 到位后降落（默认）；False = 只飞回起飞点上空悬停。
- agl_m: 飞回去时锁的离地高度（米）。
- sound: 核对落点通过后才播的声光事件。**不要在调用方自己播**—— 任务机全程要降落三次（取器材、放器材、回家），"降落动作完成" 本身说明不了任务结束，得按落点坐标判。
- report: 给队友发的事件名，带 x/y/ok 三个字段。落点不准也发， 否则队友会一直等到超时。
- direct: True = 整段走直线，不经规划器。含义和前提同 `goto_world()` ——**必须先算过这条直线的余量**。编队解散之后各自回家那一段 适合开（算过：D->各自起降点最小余量 3.00 m，离墙最近）。

> ⚠️ 自带落点核对：落在起降点上才播 sound、才算数。任务流程里会降落好几次（取器材、放器材、回家），“降落动作完成”本身说明不了任务结束。

<a id="run"></a>
### `run(leader: Any, follower: Any, description: Optional[str]=None, spacing_m: float=4.0)`

*formation_lite.py 第 38 行首次用到 · 程序入口*  ·  *staticmethod*

选手程序的 main()：解析命令行 -> 建 SDK -> 按角色分派 -> 收尾。

**参数**

- leader: 长机那一支要跑的函数，签名是 `f(sdk)`。 `--role` 是 leader 或 recon 时调它。
- follower: 僚机那一支要跑的函数，签名同上。
- description: 命令行 `--help` 里显示的说明，一般传 `__doc__`。
- spacing_m: `--spacing` 的默认值（米）。实际取到的值会存成 `sdk.spacing_m`，任务函数直接读。

