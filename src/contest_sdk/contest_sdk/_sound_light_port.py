"""内部实现：机载声光反馈板的串口协议编码/发送（L1层，选手不应该直接
`import`这个模块——跟`_rclpy_runtime.py`同样的边界原则，见`__init__.py`
文件头"选手只应该认DroneSDK这一个类"的说明，这里只给`capabilities.py`
的`DroneSDK.play_sound_light()`内部调用）。

协议来自《声光反馈程序接口.xlsx》（2026-09-16，硬件在`/dev/ttyUSB0`上
实测验证过九条命令全部正常，见`DEBUG_JOURNAL.md`同日条目）：
- 串口参数：115200波特率，8数据位，1停止位，无校验（8N1）。
- 指令格式（ASCII，逗号分隔，`\\r\\n`结尾）：
      R,G,B,声音编号,循环次数,间隔ms\\r\\n
  R/G/B是WS2812D灯带颜色(0-255)；声音编号0=静音，1-20对应
  sound1.wav~sound20.wav；循环次数/间隔ms控制这个效果重复播放几次、
  每次间隔多久（毫秒）。
- 板子是纯单向接收，不回任何确认/回执——这里没有"等待ack"的环节，
  发送成功与否只能靠串口层面写入是否抛异常来判断。

⚠️ **2026-09-16真机踩坑：不能"每次发送各自开关一次串口"**——最初的
实现是`send_command()`一个纯函数，每次调用都新开一个`serial.Serial()`
再写再关。真机联调时发现：单条命令+2秒间隔的批量测试（用同一个连接
从头发到尾）灯光/语音完全正常，但换成"每次都新开一次连接"之后**完全
没反应**。根因是USB转串口芯片`open()`时会把DTR/RTS线拉高（pyserial
默认行为，`ser.dtr`/`ser.rts`打开后确认过是`True`），这块板子的MCU外围
电路对DTR跳变的响应等同于很多Arduino兼容板"DTR触发复位"电路——每次
`open()`都会让板子经历一次硬件复位，复位到MCU真正能处理串口数据之间
有一段~2秒的"复位窗口"，这期间到达的字节会被吞掉。"每次开关一次"的
写法**必然**让每条指令都落进刚开完口的复位窗口，跟"多台设备同时接线
偶尔失灵"这种概率性问题完全不同，是必现的设计错误。

修复：`SoundLightPort`不再是"每次发送各自开关"，改成"只在第一次真正
发送时开一次连接、开完等一次`_OPEN_SETTLE_S`让板子完成复位、之后一直
复用同一个连接直到`DroneSDK`整个生命周期结束"——跟`DroneSDK`本身"一次
构造、贯穿整次任务"的生命周期天然吻合，只有第一次`play_sound_light()`
调用会多等这一次settle延迟，后续调用没有额外开销。

2026-09-21接口文档更新为20条事件（见下面`SOUND_LIGHT_EVENTS`），同时
新增常驻程序`sound_light_server.py`：地面站只有一块声光板、两架飞机的
任务进程共用，串口改由常驻程序独占持有，任务进程通过ROS2话题发请求
（仲裁/排队的理由见该文件模块头）。本模块仍然是两边共用的"协议编码+
串口连接"底层实现。
"""
import json
import time
from typing import Optional, Tuple

# pyserial只有真正开串口（常驻程序/direct模式）才需要。缺了它不能让整个
# `contest_sdk`导入失败——比如在飞行栈容器里用SDK调舵机，那里没装pyserial。
try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

#: 板子开串口时的硬件复位("DTR触发复位"，见模块头说明)到能正常处理指令
#: 之间的等待时间——2026-09-16真机实测2秒稳定有效，这里不做成可调参数
#: （只在`SoundLightPort`第一次真正发送时触发一次，不是每条指令都要等）。
_OPEN_SETTLE_S = 2.0

#: 串口默认参数（`SOUND_LIGHT_PORT`环境变量覆盖端口的理由见`capabilities.py`
#: 里这几个名字被导入处的说明）。SDK直连模式和常驻程序共用。
DEFAULT_SOUND_LIGHT_PORT = '/dev/ttyUSB0'
DEFAULT_SOUND_LIGHT_BAUDRATE = 115200
DEFAULT_SOUND_LIGHT_TIMEOUT_S = 1.0

#: 20条预置事件 -> (R, G, B, 声音编号, 循环次数, 间隔ms)，逐条取自
#: 《声光反馈程序接口.xlsx》2026-09-21版sheet1"命令"列，键名跟"语音"列
#: 逐字一致。板子上sound1~sound20.wav按这张表烧录，声音编号跟这里必须
#: 一一对应——旧版九条事件（'正在起飞'等）的编号已经被这次重新分配，
#: 不能再用。
#:
#: ⚠️ 表格第13行"侦察机任务完成"原文是`0,255,0,9,12,0`（声音9、循环12次），
#: 跟其余19行"第4位=序号、第5位=1"的规律对不上，且声音9是"侦察机发射
#: 破窗弹"——判断是笔误，这里按`0,255,0,12,1,0`实现，待跟表格作者确认。
SOUND_LIGHT_EVENTS = {
    '侦察机起飞': (50, 150, 255, 1, 1, 0),
    '任务机起飞': (50, 150, 255, 2, 1, 0),
    '侦察机降落': (255, 200, 0, 3, 1, 0),
    '任务机降落': (30, 180, 180, 4, 1, 0),
    '侦察机发现地面火情': (255, 0, 0, 5, 1, 0),
    '侦察机通报地面火情': (255, 100, 0, 6, 1, 0),
    '侦察机发现高楼火情': (255, 69, 0, 7, 1, 0),
    '侦察机通报高层火情': (150, 0, 200, 8, 1, 0),
    '侦察机发射破窗弹': (255, 200, 0, 9, 1, 0),
    '侦察机破窗完成': (255, 255, 255, 10, 1, 0),
    '侦察机排查高层火情': (100, 100, 150, 11, 1, 0),
    '侦察机任务完成': (0, 255, 0, 12, 1, 0),
    '任务机收到地面火情': (255, 100, 0, 13, 1, 0),
    '任务机发现灭火弹': (140, 80, 220, 14, 1, 0),
    '任务机抓取灭火弹': (30, 100, 255, 15, 1, 0),
    '任务机投放灭火弹': (0, 180, 60, 16, 1, 0),
    '任务机高层灭火已就位': (150, 0, 200, 17, 1, 0),
    '任务机到达瞄准点': (255, 200, 0, 18, 1, 0),
    '任务机发射灭火弹': (0, 200, 80, 19, 1, 0),
    '任务机已降落': (0, 255, 0, 20, 1, 0),
}

#: 声音编号 -> 事件名的反查表，常驻程序接受"只给编号"的请求时用。
SOUND_ID_TO_EVENT = {v[3]: k for k, v in SOUND_LIGHT_EVENTS.items()}

#: 熄灯静音指令的参数（R, G, B, 声音编号, 循环次数, 间隔ms）。
MUTE_COMMAND_ARGS = (0, 0, 0, 0, 1, 0)


def encode_command(r: int, g: int, b: int, sound: int, repeat: int, interval_ms: int) -> str:
    """按协议编码成即将写入串口的ASCII字符串（纯函数，不摸串口，方便
    脱离硬件单元测试——跟`geometry_helpers.py`/`mission_state_helpers.py`
    同样的"纯逻辑单独拆函数"约定）。
    """
    for name, val in (('r', r), ('g', g), ('b', b)):
        if not 0 <= val <= 255:
            raise ValueError(f"{name}必须在0-255之间，收到{val}")
    if not 0 <= sound <= 20:
        raise ValueError(f"sound必须在0-20之间（0=静音，1-20对应sound1.wav~sound20.wav），收到{sound}")
    if repeat < 0 or interval_ms < 0:
        raise ValueError(f"repeat/interval_ms不能为负数，收到repeat={repeat}, interval_ms={interval_ms}")
    return f"{r},{g},{b},{sound},{repeat},{interval_ms}\r\n"


#: 任务进程 -> 常驻程序的请求话题（绝对路径、不带飞机命名空间：两架
#: 飞机共用同一块板子、同一个常驻程序）。消息类型`std_msgs/String`。
REQUEST_TOPIC = '/sound_light/request'
#: 常驻程序的状态话题（`std_msgs/String`，JSON，transient_local），
#: 调试用：`ros2 topic echo /sound_light/status`看串口是否在线、最近播了什么。
STATUS_TOPIC = '/sound_light/status'


def build_request(
    event: Optional[str] = None,
    repeat: Optional[int] = None,
    interval_ms: Optional[int] = None,
    mute: bool = False,
    source: str = '',
) -> str:
    """任务进程侧：把一次声光请求编码成发往`REQUEST_TOPIC`的字符串（JSON）。
    `repeat`/`interval_ms`为`None`表示用表格里该事件的默认值。事件名在
    这里就校验——拼错的事件名应该在选手自己的进程里当场报错，而不是
    发出去之后在常驻程序那边默默丢掉。
    """
    if mute:
        return json.dumps({'mute': True, 'src': source}, ensure_ascii=False)
    if event not in SOUND_LIGHT_EVENTS:
        raise ValueError(f"未知声光事件：{event!r}，可选：{list(SOUND_LIGHT_EVENTS)}")
    if (repeat is not None and repeat < 0) or (interval_ms is not None and interval_ms < 0):
        raise ValueError(f"repeat/interval_ms不能为负数，收到repeat={repeat}, interval_ms={interval_ms}")
    return json.dumps(
        {'event': event, 'repeat': repeat, 'interval_ms': interval_ms, 'src': source},
        ensure_ascii=False,
    )


def parse_request(text: str) -> Tuple[str, Tuple[int, int, int, int, int, int], str]:
    """常驻程序侧：解析一条请求，返回`(显示名, 指令6元组, 来源)`。

    除了`build_request()`产生的JSON，也接受纯文本，方便命令行手工测试
    （`ros2 topic pub --once /sound_light/request std_msgs/String "data: 侦察机起飞"`）：
    事件名 / 声音编号`1`~`20` / `mute`或`静音`。JSON里也可以用`"sound": 5`
    代替`"event"`。非法请求抛`ValueError`。
    """
    text = text.strip()
    try:
        req = json.loads(text)
    except ValueError:
        req = None
    if not isinstance(req, dict):
        # 纯文本（或者是JSON但不是对象，比如裸数字`5`）
        if text.lower() == 'mute' or text == '静音':
            req = {'mute': True}
        elif text.isdigit():
            req = {'sound': int(text)}
        else:
            req = {'event': text}

    source = str(req.get('src') or '')
    if req.get('mute'):
        return '熄灯静音', MUTE_COMMAND_ARGS, source

    event = req.get('event')
    if event is None and req.get('sound') is not None:
        try:
            sound = int(req['sound'])
        except (TypeError, ValueError):
            raise ValueError(f"sound必须是1-20的整数，收到{req['sound']!r}") from None
        if sound not in SOUND_ID_TO_EVENT:
            raise ValueError(f"sound必须是1-20的整数，收到{sound}")
        event = SOUND_ID_TO_EVENT[sound]
    if event not in SOUND_LIGHT_EVENTS:
        raise ValueError(f"未知声光事件：{event!r}")

    r, g, b, sound, repeat, interval_ms = SOUND_LIGHT_EVENTS[event]
    if req.get('repeat') is not None:
        repeat = int(req['repeat'])
    if req.get('interval_ms') is not None:
        interval_ms = int(req['interval_ms'])
    args = (r, g, b, sound, repeat, interval_ms)
    encode_command(*args)  # 只为复用它的范围校验，越界在这里抛ValueError
    return event, args, source


class SoundLightPort:
    """一次真实串口连接的生命周期封装：只在第一次`send()`时真正`open()`
    +等一次复位settle时间，之后一直复用同一个连接（见模块头"真机踩坑"
    说明——不能每次都各自开关）。

    调用方（`capabilities.py`）应该给每个`DroneSDK`实例持有一个本类
    实例，贯穿整次任务的生命周期，不要每次发送都新建。
    """

    def __init__(self, port: str, baudrate: int, timeout_s: float):
        self._port = port
        self._baudrate = baudrate
        self._timeout_s = timeout_s
        self._ser: serial.Serial | None = None

    @property
    def port(self) -> str:
        return self._port

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    def ensure_open(self) -> None:
        """没开就现在开（附带一次复位settle等待）。常驻程序启动时主动调一次，
        把2秒复位窗口提前消化掉，不让它落在第一条真实事件上。
        """
        if self._ser is None:
            self._open()

    def _open(self) -> None:
        if serial is None:
            raise OSError('没有安装pyserial（apt install python3-serial），无法打开声光板串口')
        # exclusive=True：同一个串口被第二个进程打开会直接报错，而不是
        # 两边各自open()、互相触发DTR复位还悄无声息地交错写入（常驻程序
        # 和`SOUND_LIGHT_MODE=direct`的SDK同时跑时就是这种情况）。
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout_s,
            exclusive=True,
        )
        time.sleep(_OPEN_SETTLE_S)

    def send(self, command: str) -> None:
        """发送一条已经编码好的指令字符串。第一次调用会真正打开串口
        （附带一次复位settle等待），之后复用同一个连接。如果发送时
        发现连接已经失效（比如设备被拔掉重插过），关掉重开一次再重试
        一次——不是每次都无条件重开（那样又会踩回"每次都复位"的坑）。
        """
        self.ensure_open()
        try:
            self._ser.write(command.encode('ascii'))
            self._ser.flush()
        except OSError:
            self.close()
            self._open()
            self._ser.write(command.encode('ascii'))
            self._ser.flush()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None
