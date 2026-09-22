"""`sound_light_server.SoundLightScheduler`的单元测试：排队间隔、静音插队、
过期丢弃、串口掉线重连。串口用mock、时钟用可控的假时钟，不依赖硬件/ROS2、
不真的等。
"""
import os
import sys
import unittest
from unittest.mock import MagicMock

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_PARENT_DIR = os.path.dirname(_THIS_DIR)
if _PACKAGE_PARENT_DIR not in sys.path:
    sys.path.insert(0, _PACKAGE_PARENT_DIR)

from contest_sdk import sound_light_server as sls  # noqa: E402
from contest_sdk._sound_light_port import build_request  # noqa: E402


class _FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _fake_port():
    port = MagicMock()
    port.port = '/dev/ttyFAKE'
    port.is_open = True
    return port


class TestScheduler(unittest.TestCase):
    def setUp(self):
        self.clock = _FakeClock()
        self.port = _fake_port()
        self.s = sls.SoundLightScheduler(self.port, min_gap_s=2.5, max_age_s=15.0, clock=self.clock)

    def sent(self):
        return [c.args[0] for c in self.port.send.call_args_list]

    def test_two_drones_back_to_back_are_spaced(self):
        """双机几乎同时触发：第二条要等满间隔才发，不能把第一条瞬间覆盖。"""
        self.s.submit(build_request('侦察机发现地面火情', source='NX01'))
        self.s.submit(build_request('任务机收到地面火情', source='NX02'))
        self.s.run_once_for_test()
        self.s.run_once_for_test()
        self.assertEqual(self.sent(), ['255,0,0,5,1,0\r\n'])
        self.clock.t += 2.4
        self.s.run_once_for_test()
        self.assertEqual(len(self.sent()), 1)
        self.clock.t += 0.2
        self.s.run_once_for_test()
        self.assertEqual(self.sent()[1], '255,100,0,13,1,0\r\n')

    def test_repeat_scales_gap(self):
        self.s.submit(build_request('侦察机起飞', repeat=3, interval_ms=500))
        self.s.submit(build_request('任务机起飞'))
        self.s.run_once_for_test()
        self.clock.t += 2.5 * 3 + 1.0 - 0.1
        self.s.run_once_for_test()
        self.assertEqual(len(self.sent()), 1)
        self.clock.t += 0.2
        self.s.run_once_for_test()
        self.assertEqual(len(self.sent()), 2)

    def test_mute_jumps_queue_and_clears_it(self):
        for ev in ('侦察机起飞', '任务机起飞', '侦察机降落'):
            self.s.submit(build_request(ev))
        self.s.run_once_for_test()
        self.s.submit(build_request(mute=True))
        self.s.run_once_for_test()  # 仍在间隔内，但静音不等
        self.assertEqual(self.sent(), ['50,150,255,1,1,0\r\n', '0,0,0,0,1,0\r\n'])
        self.clock.t += 10
        self.s.run_once_for_test()
        self.assertEqual(len(self.sent()), 2, '静音应该清空了排队中的请求')

    def test_stale_requests_are_dropped(self):
        self.s.submit(build_request('侦察机起飞'))
        self.clock.t += 16
        self.s.run_once_for_test()
        self.assertEqual(self.sent(), [])
        self.assertEqual(self.s.status()['dropped'], 1)

    def test_invalid_request_is_ignored_not_raised(self):
        self.s.submit('胡说八道')
        self.s.run_once_for_test()
        self.assertEqual(self.sent(), [])

    def test_port_failure_requeues_and_reconnects(self):
        """写失败：标记串口不可用、请求放回队首；到重连时间后重开并补发。"""
        self.port.send.side_effect = [OSError('掉线'), None]
        self.s.submit(build_request('任务机已降落'))
        self.s.run_once_for_test()
        self.assertFalse(self.s.status()['connected'])
        self.assertEqual(self.s.status()['queued'], 1)
        self.clock.t += sls.RECONNECT_INTERVAL_S
        self.s.run_once_for_test()  # 重连
        self.port.ensure_open.assert_called()
        self.assertTrue(self.s.status()['connected'])
        self.s.run_once_for_test()  # 补发
        self.assertEqual(self.port.send.call_count, 2)
        self.assertEqual(self.port.send.call_args.args[0], '0,255,0,20,1,0\r\n')

    def test_port_missing_at_startup_keeps_waiting(self):
        self.port.ensure_open.side_effect = [OSError('没插'), None]
        self.s._try_connect()
        self.assertFalse(self.s.status()['connected'])
        self.s.submit(build_request('侦察机起飞'))
        self.s.run_once_for_test()
        self.port.send.assert_not_called()
        self.clock.t += sls.RECONNECT_INTERVAL_S
        self.s.run_once_for_test()
        self.s.run_once_for_test()
        self.assertEqual(self.sent(), ['50,150,255,1,1,0\r\n'])

    def test_queue_overflow_drops_oldest(self):
        s = sls.SoundLightScheduler(self.port, max_queue=2, clock=self.clock)
        for ev in ('侦察机起飞', '任务机起飞', '侦察机降落'):
            s.submit(build_request(ev))
        s.run_once_for_test()
        self.assertEqual(self.sent(), ['50,150,255,2,1,0\r\n'])

    def test_dry_run_does_not_touch_port(self):
        s = sls.SoundLightScheduler(None, clock=self.clock)
        s.submit(build_request('侦察机起飞'))
        s.run_once_for_test()
        self.assertEqual(s.status()['played'], 1)


if __name__ == '__main__':
    unittest.main()
