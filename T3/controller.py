"""T3 inference and shaping of the joint targets sent to T2."""

from math import isfinite

import numpy as np

from policy import KD as POLICY_KD
from policy import KP as POLICY_KP
from policy import PERIOD_NS, WalkTheseWays
from safety_block import HOLD_KD, HOLD_KP, JOINT_LIMITS, MAX_JOINT_STEP, CommandStream

JOINT_MIN, JOINT_MAX = np.tile(JOINT_LIMITS, (4, 1)).T
# Leave room for float32 ROS serialization at the boundary of the T2 limit.
MAX_PROPOSED_STEP = MAX_JOINT_STEP - 0.001


class Controller:
    def __init__(self, home_angles):
        self.home_angles = np.asarray(home_angles)
        self.policy = WalkTheseWays()
        self.reset()

    def reset(self):
        self.policy.reset()
        self.next_inference_ns = None
        self.target = self.home_angles

    def command(self, now_ns, observation, velocity):
        """Return joint positions and PD gains for /lowcmd_bare in Unitree order."""
        kp, kd = POLICY_KP, POLICY_KD
        if not any(velocity):
            if self.next_inference_ns is not None:
                self.reset()
            kp, kd = HOLD_KP, HOLD_KD
        elif self.next_inference_ns is None or now_ns >= self.next_inference_ns:
            self.target = self.policy.step(*observation, velocity)
            self.next_inference_ns = (now_ns // PERIOD_NS + 1) * PERIOD_NS

        return np.clip(self.target, JOINT_MIN, JOINT_MAX), kp, kd


class PolicyCommandStream(CommandStream):
    def prepare(self, message, previous_angles):
        # The queue can advance while collision validation is running. Apply the
        # step limit from the latest accepted target at validation time.
        for motor, previous in zip(message.motor_cmd, previous_angles):
            if isfinite(motor.q):
                motor.q = float(min(max(motor.q, previous - MAX_PROPOSED_STEP), previous + MAX_PROPOSED_STEP))
        return super().prepare(message, previous_angles)
