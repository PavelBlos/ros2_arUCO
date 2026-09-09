import sys
import threading
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from termit_api import TermitRobotAPI, HoldMode

class ControlTests(unittest.TestCase):
    def setUp(self):
        self.api = TermitRobotAPI()
        self.sent = []
        self.api._send_raw = self.sent.append

    def test_target_is_not_host_ramp(self):
        self.api.set_motor_speeds(700, -700, 0)
        self.api._control_tick(1.0)
        self.assertEqual(self.sent, ['s 700 -700 0', 'k'])
        self.sent.clear()
        for t in (1.025, 1.05, 1.1, 1.25):
            self.api._control_tick(t)
        self.assertEqual(self.sent, ['k'])

    def test_normal_stop_keeps_ramp_emergency_aborts(self):
        self.api.set_motor_speeds(500, 0, 0)
        self.api._control_tick(1)
        self.api.stop()
        self.assertEqual(self.sent[-1], 's 0 0 0')
        self.api.emergency_stop()
        self.assertEqual(self.sent[-1], 'x')
        self.sent.clear()
        self.api._control_tick(2)
        self.assertEqual(self.sent, ['k'])

    def test_disable_cancels_target(self):
        self.api.drive(0.1, 0.1)
        self.api._control_tick(1)
        self.api.set_holding_mode(HoldMode.DISABLED)
        self.sent.clear()
        self.api._control_tick(2)
        self.assertEqual(self.sent, ['k'])
        self.assertEqual(self.api._target_steps_locked(), (0, 0, 0))

    def test_reversal_and_small_speeds_unchanged(self):
        for speed in (1, 50, 120, 600, -50, -600):
            self.api.set_motor_speeds(speed, 0, 0)
            self.api._control_tick(speed + 1000)
            self.assertIn(f's {speed} 0 0', self.sent)

    def test_short_watchdog_gets_fast_heartbeat(self):
        self.api.config.watchdog_timeout_ms = 100
        self.api._control_tick(1)
        self.sent.clear()
        self.api._control_tick(1.025)
        self.api._control_tick(1.05)
        self.assertEqual(self.sent, ['k'])

    def test_finite_move_not_overwritten(self):
        self.api.microstep(1, 50)
        self.api._control_tick(1)
        self.assertEqual(self.sent, ['m 1 50', 'k'])
        self.api.stop()
        self.assertEqual(self.sent[-1], 's 0 0 0')

    def test_stop_serialized_with_inflight_control(self):
        entered, release = threading.Event(), threading.Event()
        def send(cmd):
            if cmd.startswith('s 500'):
                entered.set()
                release.wait(1)
            self.sent.append(cmd)
        self.api._send_raw = send
        self.api.set_motor_speeds(500, 0, 0)
        control = threading.Thread(target=self.api._control_tick, args=(1,))
        control.start()
        self.assertTrue(entered.wait(1))
        stop = threading.Thread(target=self.api.emergency_stop)
        stop.start()
        release.set()
        control.join()
        stop.join()
        self.assertEqual(self.sent[-1], 'x')
        self.api._control_tick(2)
        self.assertNotIn('s 500 0 0', self.sent[self.sent.index('x') + 1:])

    def test_rep103_forward_motion(self):
        self.api.drive(0.10, 0.0, 0.0)
        s1, s2, s3 = self.api._target_steps_locked()
        self.assertEqual(s1, 0, "Front wheel should have 0 speed during pure forward motion")
        self.assertLess(s2, 0, "Rear-right wheel should rotate backwards")
        self.assertGreater(s3, 0, "Rear-left wheel should rotate forwards")
        self.assertEqual(abs(s2), abs(s3), "Rear wheels should have equal and opposite speeds")

    def test_rep103_backward_motion(self):
        self.api.drive(-0.10, 0.0, 0.0)
        s1, s2, s3 = self.api._target_steps_locked()
        self.assertEqual(s1, 0, "Front wheel should have 0 speed during pure backward motion")
        self.assertGreater(s2, 0, "Rear-right wheel should rotate forwards")
        self.assertLess(s3, 0, "Rear-left wheel should rotate backwards")
        self.assertEqual(abs(s2), abs(s3))

    def test_rep103_left_motion(self):
        self.api.drive(0.0, 0.10, 0.0)
        s1, s2, s3 = self.api._target_steps_locked()
        self.assertLess(s1, 0, "Front wheel drives robot left")
        self.assertGreater(s2, 0, "Rear-right wheel supports leftward motion")
        self.assertGreater(s3, 0, "Rear-left wheel supports leftward motion")
        self.assertEqual(s2, s3, "Rear wheels should have identical speeds during pure strafe")
        self.assertEqual(abs(s1), 2 * s2, "Front wheel speed should equal 2 * rear wheel speed")

    def test_rep103_right_motion(self):
        self.api.drive(0.0, -0.10, 0.0)
        s1, s2, s3 = self.api._target_steps_locked()
        self.assertGreater(s1, 0)
        self.assertLess(s2, 0)
        self.assertLess(s3, 0)
        self.assertEqual(s2, s3)
        self.assertEqual(s1, 2 * abs(s2))

    def test_rep103_ccw_rotation(self):
        self.api.drive(0.0, 0.0, 0.5)
        s1, s2, s3 = self.api._target_steps_locked()
        self.assertGreater(s1, 0)
        self.assertGreater(s2, 0)
        self.assertGreater(s3, 0)
        self.assertEqual(s1, s2)
        self.assertEqual(s2, s3)

    def test_rep103_cw_rotation(self):
        self.api.drive(0.0, 0.0, -0.5)
        s1, s2, s3 = self.api._target_steps_locked()
        self.assertLess(s1, 0)
        self.assertLess(s2, 0)
        self.assertLess(s3, 0)
        self.assertEqual(s1, s2)
        self.assertEqual(s2, s3)

    def test_rep103_odometry_consistency(self):
        # 70mm wheel diameter check
        self.assertAlmostEqual(self.api.config.wheel_radius, 0.035, places=4)
        # Test odometry processing: simulate steps resulting from 0.1 m/s forward for 0.1s
        # Steps per meter:
        spm = self.api._steps_per_meter
        # Pure forward 0.1 m/s for 0.1s -> ds = 0.01m
        # For pure forward: ds1=0, ds2=-0.866025*0.01, ds3=+0.866025*0.01
        ds = 0.01
        sqrt3_2 = 0.8660254037844386
        dp1 = 0
        dp2 = int(round(-sqrt3_2 * ds * spm))
        dp3 = int(round(+sqrt3_2 * ds * spm))
        
        self.api._last_wheel_steps = (0, 0, 0)
        self.api._last_odom_time = 1.0
        self.api._process_odometry_update(dp1, dp2, dp3, 0, 0, 0)
        odom = self.api.get_odometry()
        # Robot dx_body should be approximately +0.01m forward, dy_body ~ 0
        self.assertAlmostEqual(odom.dx_body, ds, delta=0.001)
        self.assertAlmostEqual(odom.dy_body, 0.0, delta=0.001)


if __name__ == '__main__':
    unittest.main(verbosity=2)
