"""Offline regressions for T3 policy commands."""

import unittest
from math import cos, radians, sin
from unittest.mock import patch

from controller import MAX_HEADING_YAW_RATE, MAX_PROPOSED_STEP, Controller, PolicyCommandStream
from policy import PERIOD_NS
from safety_block import PUBLISH_PERIOD_NS, load_collision_guard
from t2_checks import ManualWorker, publish_at, sample_command
from t2_checks import test_message_type as make_message


class HeadingHoldTest(unittest.TestCase):
    forward = (0.6, 0.0, 0.0)
    home = (0.0, 0.8, -1.5) * 4

    def make_controller(self, gain=None):
        settings = {} if gain is None else {"T3_HEADING_GAIN": gain}
        with patch.dict("os.environ", settings, clear=True), patch("controller.WalkTheseWays") as policy:
            policy.return_value.step.return_value = self.home
            return Controller(self.home)

    def command(self, controller, tick, heading, velocity=forward):
        half_angle = radians(heading) / 2
        # Scaled quaternion still represents the same attitude.
        quaternion = (2 * cos(half_angle), 0.0, 0.0, 2 * sin(half_angle))
        observation = quaternion, self.home, (0.0,) * 12
        controller.command(int(tick * PERIOD_NS), observation, velocity)
        return controller.policy.step.call_args.args[-1]

    def test_default_passes_through_and_invalid_gains_are_rejected(self):
        controller = self.make_controller()
        self.assertEqual(self.command(controller, 0, 45), self.forward)
        self.assertEqual(self.command(controller, 1, 90), self.forward)
        for invalid in ("invalid", "inf", "nan", "-0.1", "5.1"):
            with self.subTest(gain=invalid), self.assertRaisesRegex(ValueError, "T3_HEADING_GAIN"):
                self.make_controller(invalid)

    def test_negative_feedback_wraps_and_saturates(self):
        controller = self.make_controller("2")
        self.assertEqual(self.command(controller, 0, 179), self.forward)
        corrected = self.command(controller, 1, -179)
        self.assertEqual(corrected[:2], self.forward[:2])
        self.assertAlmostEqual(corrected[2], -2 * radians(2))
        self.assertEqual(self.command(controller, 2, -140)[2], -MAX_HEADING_YAW_RATE)
        self.assertEqual(self.command(controller, 3, 140)[2], MAX_HEADING_YAW_RATE)

    def test_new_straight_segment_captures_current_heading(self):
        controller = self.make_controller("2")
        self.command(controller, 0, 20)
        turn = (0.6, 0.0, 0.4)
        self.assertEqual(self.command(controller, 1, 35, turn), turn)
        self.assertEqual(self.command(controller, 2, 50), self.forward)

        # A lateral request between actor inference ticks also clears the reference.
        self.command(controller, 2.5, 65, (0.6, 0.1, 0.0))
        self.assertEqual(self.command(controller, 3, 75), self.forward)
        self.command(controller, 4, 90, (0.0, 0.0, 0.0))
        self.assertEqual(self.command(controller, 5, 100), self.forward)
        controller.reset()
        self.assertEqual(self.command(controller, 0, 115), self.forward)


class PolicyCommandStreamTest(unittest.TestCase):
    def test_reversal_uses_the_latest_accepted_angles(self):
        worker = ManualWorker()
        stream = PolicyCommandStream(make_message, load_collision_guard(), worker)
        home = stream.last_angles
        forward = sample_command(make_message, home)
        reverse = sample_command(make_message, home)
        forward.motor_cmd[0].q += radians(4.5)
        reverse.motor_cmd[0].q -= radians(4.5)

        tick = PUBLISH_PERIOD_NS
        stream.receive(forward, 0)
        publish_at(stream, tick)
        stream.receive(reverse, tick + 1)
        self.assertEqual(stream.last_angles, home)

        worker.finish()
        accepted = publish_at(stream, 2 * tick)
        self.assertEqual(accepted.motor_cmd[0].q, forward.motor_cmd[0].q)

        worker.finish()
        shaped = publish_at(stream, 3 * tick)
        expected_hip = accepted.motor_cmd[0].q - MAX_PROPOSED_STEP
        self.assertAlmostEqual(shaped.motor_cmd[0].q, expected_hip, places=7)
        self.assertEqual(stream.last_angles[1:], home[1:])


if __name__ == "__main__":
    unittest.main()
