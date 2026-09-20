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

九个预置事件（接口文档sheet1表格）取默认循环1次/间隔0ms：
    发现地面火情 -> 255,0,0,1,1,0
    发现高楼火情 -> 255,69,0,2,1,0
    发射破窗弹   -> 255,200,0,3,1,0
    发射灭火弹   -> 0,200,80,4,1,0
    正在起飞     -> 50,150,255,5,1,0
    正在降落     -> 30,180,180,6,1,0
    发现灭火弹   -> 140,80,220,7,1,0
    抓取灭火弹   -> 30,100,255,8,1,0
    释放灭火弹   -> 0,180,60,9,1,0
"""
import time

import serial

#: 板子开串口时的硬件复位("DTR触发复位"，见模块头说明)到能正常处理指令
#: 之间的等待时间——2026-09-16真机实测2秒稳定有效，这里不做成可调参数
#: （只在`SoundLightPort`第一次真正发送时触发一次，不是每条指令都要等）。
_OPEN_SETTLE_S = 2.0

#: 九条预置事件 -> (R, G, B, 声音编号)，取自接口文档sheet1，键名跟
#: 表格里的中文语音文本逐字一致，方便选手对照文档直接抄事件名。
SOUND_LIGHT_EVENTS = {
    '发现地面火情': (255, 0, 0, 1),
    '发现高楼火情': (255, 69, 0, 2),
    '发射破窗弹': (255, 200, 0, 3),
    '发射灭火弹': (0, 200, 80, 4),
    '正在起飞': (50, 150, 255, 5),
    '正在降落': (30, 180, 180, 6),
    '发现灭火弹': (140, 80, 220, 7),
    '抓取灭火弹': (30, 100, 255, 8),
    '释放灭火弹': (0, 180, 60, 9),
}


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

    def _open(self) -> None:
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout_s,
        )
        time.sleep(_OPEN_SETTLE_S)

    def send(self, command: str) -> None:
        """发送一条已经编码好的指令字符串。第一次调用会真正打开串口
        （附带一次复位settle等待），之后复用同一个连接。如果发送时
        发现连接已经失效（比如设备被拔掉重插过），关掉重开一次再重试
        一次——不是每次都无条件重开（那样又会踩回"每次都复位"的坑）。
        """
        if self._ser is None:
            self._open()
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
