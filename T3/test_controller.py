"""Offline regression for a policy target queued during a safety check."""

import unittest
from math import radians

from controller import MAX_PROPOSED_STEP, PolicyCommandStream
from safety_block import PUBLISH_PERIOD_NS, load_collision_guard
from t2_checks import ManualWorker, publish_at, sample_command
from t2_checks import test_message_type as make_message


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
