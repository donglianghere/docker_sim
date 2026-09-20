"""`mission_state_helpers.py`的独立单元测试（B3清单/方案第5节验收要求：
"mission_state格式校验（冒号子状态那部分）"要写独立的、不需要真实ROS2
环境的单元测试）。

`mission_state_helpers.py`本身不import任何ROS包，这里的测试同样全程不
需要ROS2环境，直接`python3 test_mission_state_helpers.py`就能跑——这也是
为什么这两个东西被特意从`capabilities.py`（模块顶层要import rclpy/
rcl_interfaces/std_srvs等ROS2包）拆到独立文件`mission_state_helpers.py`
里的原因，见该文件文件头说明。
"""
import os
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_PARENT_DIR = os.path.dirname(_THIS_DIR)
if _PACKAGE_PARENT_DIR not in sys.path:
    sys.path.insert(0, _PACKAGE_PARENT_DIR)

from contest_sdk.mission_state_helpers import (  # noqa: E402
    VALID_MISSION_STATES,
    is_valid_mission_state,
)


class TestValidMissionStates(unittest.TestCase):
    def test_exactly_six_top_level_states(self):
        """方案原文/`contest_mission/mission_state.py`约定只有6个顶层值，
        这里校验vendor过来的这份拷贝没有被不小心改动过。"""
        self.assertEqual(
            VALID_MISSION_STATES,
            ('idle', 'searching', 'loading', 'enroute', 'aiming', 'done'),
        )


class TestIsValidMissionState(unittest.TestCase):
    def test_all_six_top_level_values_alone_are_valid(self):
        for state in VALID_MISSION_STATES:
            with self.subTest(state=state):
                self.assertTrue(is_valid_mission_state(state))

    def test_unknown_top_level_value_is_invalid(self):
        self.assertFalse(is_valid_mission_state('unknown_state'))
        self.assertFalse(is_valid_mission_state(''))

    def test_colon_substate_format_is_valid_when_top_level_is_known(self):
        """方案4.4节要用的"顶层值:子状态"带冒号写法——这是这次B3实现前
        专门核查过的一点：现有`is_valid_mission_state()`已经支持这种
        格式，不需要扩展，见`mission_state_helpers.py`模块头说明。"""
        self.assertTrue(is_valid_mission_state('searching:orbit_pillar_2'))
        self.assertTrue(is_valid_mission_state('aiming:pillar_3'))
        self.assertTrue(is_valid_mission_state('done:'))  # 子状态部分可以是空字符串，不限制内容

    def test_colon_substate_format_invalid_when_top_level_unknown(self):
        """子状态部分不限制内容，但冒号前的顶层值仍然必须是6个合法值之一
        ——不能靠拼一个冒号就绕过顶层值校验。"""
        self.assertFalse(is_valid_mission_state('flying:orbit_pillar_2'))
        self.assertFalse(is_valid_mission_state('unknown:sub'))

    def test_substate_content_is_unrestricted(self):
        """子状态部分本身不限制内容——即使子状态里又带了冒号，仍然只看
        第一个冒号之前的顶层部分（`split(':', 1)`只切一次）。"""
        self.assertTrue(is_valid_mission_state('searching:orbit_pillar_2:extra:info'))
        self.assertTrue(is_valid_mission_state('idle:任意中文子状态都可以'))


if __name__ == '__main__':
    unittest.main()
