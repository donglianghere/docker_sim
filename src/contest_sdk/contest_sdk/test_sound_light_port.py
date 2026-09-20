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
    encode_command,
)


class TestEncodeCommand(unittest.TestCase):
    def test_nine_preset_events_match_interface_doc(self):
        """九条预置事件（repeat=1, interval_ms=0）必须跟接口文档sheet1逐字一致。"""
        expected = {
            '发现地面火情': '255,0,0,1,1,0\r\n',
            '发现高楼火情': '255,69,0,2,1,0\r\n',
            '发射破窗弹': '255,200,0,3,1,0\r\n',
            '发射灭火弹': '0,200,80,4,1,0\r\n',
            '正在起飞': '50,150,255,5,1,0\r\n',
            '正在降落': '30,180,180,6,1,0\r\n',
            '发现灭火弹': '140,80,220,7,1,0\r\n',
            '抓取灭火弹': '30,100,255,8,1,0\r\n',
            '释放灭火弹': '0,180,60,9,1,0\r\n',
        }
        self.assertEqual(set(SOUND_LIGHT_EVENTS), set(expected))
        for event, exp_cmd in expected.items():
            r, g, b, sound = SOUND_LIGHT_EVENTS[event]
            self.assertEqual(encode_command(r, g, b, sound, repeat=1, interval_ms=0), exp_cmd)

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
