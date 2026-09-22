"""`_sound_light_port.py`的独立单元测试（跟`test_geometry_helpers.py`/
`test_mission_state_helpers.py`同样的"纯逻辑单独拆函数、脱离ROS2环境
单元测试"约定）。真实硬件（`/dev/ttyUSB0`）验证记录见`DEBUG_JOURNAL.md`
2026-09-16条目：`encode_command()`部分不摸串口直接测；`SoundLightPort`
的"只开一次连接、复用"这条2026-09-16真机踩坑修复的行为，用mock掉
`serial.Serial`/`time.sleep`的方式回归测试，避免真的跑2秒/依赖硬件。
"""
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_PARENT_DIR = os.path.dirname(_THIS_DIR)
if _PACKAGE_PARENT_DIR not in sys.path:
    sys.path.insert(0, _PACKAGE_PARENT_DIR)

from contest_sdk._sound_light_port import (  # noqa: E402
    SOUND_LIGHT_EVENTS,
    SoundLightPort,
    build_request,
    encode_command,
    parse_request,
)


class TestEncodeCommand(unittest.TestCase):
    def test_twenty_preset_events_match_interface_doc(self):
        """20条预置事件必须跟接口文档（2026-09-21版）sheet1"命令"列逐字一致。
        唯一例外是"侦察机任务完成"：原文`0,255,0,9,12,0`判断为笔误，
        按`0,255,0,12,1,0`实现（见`SOUND_LIGHT_EVENTS`上方注释）。
        """
        expected = {
            '侦察机起飞': '50,150,255,1,1,0\r\n',
            '任务机起飞': '50,150,255,2,1,0\r\n',
            '侦察机降落': '255,200,0,3,1,0\r\n',
            '任务机降落': '30,180,180,4,1,0\r\n',
            '侦察机发现地面火情': '255,0,0,5,1,0\r\n',
            '侦察机通报地面火情': '255,100,0,6,1,0\r\n',
            '侦察机发现高楼火情': '255,69,0,7,1,0\r\n',
            '侦察机通报高层火情': '150,0,200,8,1,0\r\n',
            '侦察机发射破窗弹': '255,200,0,9,1,0\r\n',
            '侦察机破窗完成': '255,255,255,10,1,0\r\n',
            '侦察机排查高层火情': '100,100,150,11,1,0\r\n',
            '侦察机任务完成': '0,255,0,12,1,0\r\n',
            '任务机收到地面火情': '255,100,0,13,1,0\r\n',
            '任务机发现灭火弹': '140,80,220,14,1,0\r\n',
            '任务机抓取灭火弹': '30,100,255,15,1,0\r\n',
            '任务机投放灭火弹': '0,180,60,16,1,0\r\n',
            '任务机高层灭火已就位': '150,0,200,17,1,0\r\n',
            '任务机到达瞄准点': '255,200,0,18,1,0\r\n',
            '任务机发射灭火弹': '0,200,80,19,1,0\r\n',
            '任务机已降落': '0,255,0,20,1,0\r\n',
        }
        self.assertEqual(list(SOUND_LIGHT_EVENTS), list(expected))
        for event, exp_cmd in expected.items():
            self.assertEqual(encode_command(*SOUND_LIGHT_EVENTS[event]), exp_cmd)

    def test_sound_ids_are_one_to_twenty_unique(self):
        self.assertEqual(sorted(v[3] for v in SOUND_LIGHT_EVENTS.values()), list(range(1, 21)))

    def test_general_format_matches_interface_doc_example(self):
        """接口文档图示例：255,0,0,1,3,500 -> 红色+声1，循环3次，间隔500ms。"""
        self.assertEqual(encode_command(255, 0, 0, 1, repeat=3, interval_ms=500), '255,0,0,1,3,500\r\n')

    def test_mute_is_all_zero(self):
        self.assertEqual(encode_command(0, 0, 0, 0, repeat=1, interval_ms=0), '0,0,0,0,1,0\r\n')

    def test_rejects_out_of_range_rgb(self):
        for kwargs in (dict(r=256), dict(g=-1), dict(b=300)):
            base = dict(r=0, g=0, b=0, sound=1, repeat=1, interval_ms=0)
            base.update(kwargs)
            with self.assertRaises(ValueError):
                encode_command(**base)

    def test_rejects_out_of_range_sound(self):
        with self.assertRaises(ValueError):
            encode_command(0, 0, 0, sound=21, repeat=1, interval_ms=0)
        with self.assertRaises(ValueError):
            encode_command(0, 0, 0, sound=-1, repeat=1, interval_ms=0)

    def test_rejects_negative_repeat_or_interval(self):
        with self.assertRaises(ValueError):
            encode_command(0, 0, 0, sound=1, repeat=-1, interval_ms=0)
        with self.assertRaises(ValueError):
            encode_command(0, 0, 0, sound=1, repeat=1, interval_ms=-1)


class TestRequestProtocol(unittest.TestCase):
    """任务进程 -> 常驻程序的请求编解码（`build_request`/`parse_request`）。"""

    def test_roundtrip_uses_table_defaults(self):
        label, args, src = parse_request(build_request('任务机抓取灭火弹', source='NX02'))
        self.assertEqual((label, args, src), ('任务机抓取灭火弹', (30, 100, 255, 15, 1, 0), 'NX02'))

    def test_roundtrip_overrides_repeat_interval(self):
        _, args, _ = parse_request(build_request('侦察机起飞', repeat=3, interval_ms=500))
        self.assertEqual(args, (50, 150, 255, 1, 3, 500))

    def test_mute(self):
        self.assertEqual(parse_request(build_request(mute=True))[1], (0, 0, 0, 0, 1, 0))

    def test_plain_text_forms(self):
        """命令行手工`ros2 topic pub`用的纯文本形式。"""
        self.assertEqual(parse_request('侦察机起飞')[0], '侦察机起飞')
        self.assertEqual(parse_request('  5 ')[0], '侦察机发现地面火情')
        self.assertEqual(parse_request('mute')[1], (0, 0, 0, 0, 1, 0))
        self.assertEqual(parse_request('静音')[1], (0, 0, 0, 0, 1, 0))
        self.assertEqual(parse_request('{"sound": 20}')[0], '任务机已降落')

    def test_build_rejects_unknown_event_early(self):
        """拼错的事件名要在选手进程里当场报错，不能发出去才被常驻程序丢掉。"""
        with self.assertRaises(ValueError):
            build_request('正在起飞')  # 旧版九条事件名，已作废
        with self.assertRaises(ValueError):
            build_request('侦察机起飞', repeat=-1)

    def test_parse_rejects_garbage(self):
        for text in ('不存在的事件', '0', '21', '{"sound": "x"}', '{"event": "侦察机起飞", "repeat": -1}'):
            with self.assertRaises(ValueError, msg=text):
                parse_request(text)


class TestSoundLightPort(unittest.TestCase):
    """回归测试2026-09-16真机踩坑："每次发送各自开关串口"会撞上板子的
    DTR复位窗口、指令被硬件吞掉——修复成"只在第一次send()时真正open()
    +等一次settle、之后复用同一个连接"，这里验证的就是这条行为不会
    被以后的改动悄悄改回去。
    """

    @patch('contest_sdk._sound_light_port.time.sleep')
    @patch('contest_sdk._sound_light_port.serial.Serial')
    def test_first_send_opens_once_and_settles(self, mock_serial_cls, mock_sleep):
        mock_ser = mock_serial_cls.return_value
        port = SoundLightPort('/dev/ttyUSB0', 115200, 1.0)

        port.send('255,0,0,1,1,0\r\n')
        port.send('0,0,0,0,1,0\r\n')

        self.assertEqual(mock_serial_cls.call_count, 1, "两次send()不应该各自重新open()")
        mock_sleep.assert_called_once()
        self.assertEqual(mock_ser.write.call_count, 2)
        mock_ser.write.assert_any_call(b'255,0,0,1,1,0\r\n')
        mock_ser.write.assert_any_call(b'0,0,0,0,1,0\r\n')

    @patch('contest_sdk._sound_light_port.time.sleep')
    @patch('contest_sdk._sound_light_port.serial.Serial')
    def test_reopens_once_after_write_failure(self, mock_serial_cls, mock_sleep):
        first_ser = MagicMock()
        first_ser.write.side_effect = OSError('设备掉线')
        second_ser = MagicMock()
        mock_serial_cls.side_effect = [first_ser, second_ser]

        port = SoundLightPort('/dev/ttyUSB0', 115200, 1.0)
        port.send('255,0,0,1,1,0\r\n')

        self.assertEqual(mock_serial_cls.call_count, 2, "写入失败应该重开一次连接再重试")
        second_ser.write.assert_called_once_with(b'255,0,0,1,1,0\r\n')


if __name__ == '__main__':
    unittest.main()
