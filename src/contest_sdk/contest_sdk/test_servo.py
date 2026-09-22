"""`set_servo()`的PWM -> 归一化换算测试：发出去的值经PX4输出侧
`interpolate(v,-1,1,MIN,MAX)`+四舍五入（mixer_module.cpp
`output_limit_calc_single`）之后，必须精确还原成选手给的PWM。
需要ROS2环境（`capabilities.py`会import rclpy），在contestant-sdk镜像里跑。
"""
import unittest

from contest_sdk.capabilities import SERVO_CONFIG, ServoSpec, servo_pwm_to_normalized


def _px4_output(spec: ServoSpec, v: float) -> int:
    return round(spec.pwm_min + (v + 1.0) / 2.0 * (spec.pwm_max - spec.pwm_min))


class TestServo(unittest.TestCase):
    def test_nx02_config_matches_hardware(self):
        cfg = SERVO_CONFIG['NX02']
        self.assertEqual(cfg[1], ServoSpec(actuator_set=1, output='MAIN7', pwm_min=800, pwm_max=2000))
        self.assertEqual(cfg[2], ServoSpec(actuator_set=2, output='MAIN9', pwm_min=800, pwm_max=2000))

    def test_every_pwm_roundtrips_exactly(self):
        spec = SERVO_CONFIG['NX02'][1]
        for pwm in range(spec.pwm_min, spec.pwm_max + 1):
            v = servo_pwm_to_normalized(spec, pwm)
            self.assertTrue(-1.0 <= v <= 1.0)
            self.assertEqual(_px4_output(spec, v), pwm)

    def test_endpoints(self):
        spec = SERVO_CONFIG['NX02'][1]
        self.assertEqual(servo_pwm_to_normalized(spec, 800), -1.0)
        self.assertEqual(servo_pwm_to_normalized(spec, 2000), 1.0)

    def test_out_of_range_rejected(self):
        spec = SERVO_CONFIG['NX02'][1]
        for pwm in (799, 2001, 0):
            with self.assertRaises(ValueError):
                servo_pwm_to_normalized(spec, pwm)


if __name__ == '__main__':
    unittest.main()
