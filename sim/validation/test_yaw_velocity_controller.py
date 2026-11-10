"""Feedback safety invariants independent of ROS and MuJoCo."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime"))
from yaw_velocity_controller import YawVelocityController


class YawVelocityControllerTests(unittest.TestCase):
    def test_unreachable_rate_stays_inside_policy_envelope(self):
        controller = YawVelocityController()
        for _ in range(10000):
            output = controller.update(0.5, 0.0, 0.01)
            self.assertTrue(math.isfinite(output))
            self.assertLessEqual(abs(output), 0.8)
        self.assertEqual(controller.update(0.0, 0.0, 0.01), 0.0)
        self.assertEqual(controller.integral, 0.0)

    def test_invalid_feedback_clears_accumulated_command(self):
        for invalid in (math.nan, math.inf, -math.inf):
            controller = YawVelocityController()
            for _ in range(100):
                controller.update(0.2, 0.0, 0.01)
            self.assertEqual(controller.update(0.2, invalid, 0.01), 0.0)
            self.assertEqual(controller.output, 0.0)

    def test_suspended_clock_does_not_jump_to_full_command(self):
        controller = YawVelocityController()
        output = controller.update(0.5, 0.0, 100.0)
        self.assertLess(output, controller.policy_limit)
        self.assertEqual(controller.update(0.0, 0.0, 0.01), 0.0)

    def test_no_simulation_time_does_not_advance_controller(self):
        controller = YawVelocityController()
        self.assertEqual(controller.update(0.2, 0.0, 0.0), 0.0)
        self.assertEqual(controller.integral, 0.0)

    def test_reversal_and_negative_commands_are_bounded(self):
        controller = YawVelocityController()
        for _ in range(100):
            controller.update(0.5, 0.4, 0.01)
        outputs = [controller.update(-0.5, -0.4, 0.01) for _ in range(1000)]
        self.assertTrue(all(abs(value) <= 0.8 for value in outputs))
        self.assertLess(outputs[-1], 0.0)
        self.assertEqual(controller.update(0.0, -0.1, 0.01), 0.0)

    def test_invalid_policy_limits_are_rejected(self):
        for limit in (0.0, -1.0, math.nan, math.inf):
            with self.assertRaises(ValueError):
                YawVelocityController(limit)

    def test_creeping_rate_stops_when_ahead_and_resumes(self):
        controller = YawVelocityController()
        for _ in range(10):
            output = controller.update(-0.01, -0.10, 0.01)
        self.assertTrue(controller.holding)
        self.assertEqual(output, 0.0)
        for _ in range(200):
            output = controller.update(-0.01, 0.0, 0.01)
        self.assertFalse(controller.holding)
        self.assertLess(output, 0.0)
        self.assertEqual(controller.update(0.0, 0.0, 0.01), 0.0)
        self.assertEqual(controller.angle_error, 0.0)

    def test_leaving_creep_mode_releases_hold(self):
        controller = YawVelocityController()
        for _ in range(10):
            controller.update(0.01, 0.1, 0.01)
        self.assertTrue(controller.holding)
        self.assertGreater(controller.update(0.2, 0.0, 0.01), 0.0)
        self.assertFalse(controller.holding)

    def test_startup_stiction_preserves_requested_angle(self):
        controller = YawVelocityController()
        for _ in range(1000):
            output = controller.update(-0.02, 0.0, 0.01)
        self.assertLess(controller.angle_error, -0.1)
        self.assertLess(output, 0.0)
        self.assertFalse(controller.holding)
        self.assertEqual(controller.update(0.0, 0.0, 0.01), 0.0)
        self.assertEqual(controller.angle_error, 0.0)

    def test_command_roundoff_cannot_start_a_gait(self):
        for target in (1e-12, -1e-12):
            controller = YawVelocityController()
            for _ in range(100):
                self.assertEqual(controller.update(target, 0.0, 0.01), 0.0)


if __name__ == "__main__":
    unittest.main()
