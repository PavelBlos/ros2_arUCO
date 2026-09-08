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

if __name__ == '__main__':
    unittest.main(verbosity=2)
