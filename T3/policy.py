"""CPU inference for the Walk These Ways Go1 policy."""

from pathlib import Path

import numpy as np


PERIOD_NS = 20_000_000
# The organizer's MuJoCo model adds passive joint damping of 2 N·m·s/rad.
KP = 40.0
KD = 0.5
OBSERVATION_COUNT = 70
HISTORY_LENGTH = 30
DEFAULT_CHECKPOINT = Path(__file__).with_name("checkpoints") / "walk_these_ways.npz"

# The permutation converts FR/FL/RR/RL to FL/FR/RL/RR and vice versa.
JOINT_ORDER = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
NEUTRAL = np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5], dtype=np.float32)
ACTION_SCALE = np.tile(np.array([0.125, 0.25, 0.25], dtype=np.float32), 4)
COMMAND_SCALE = np.array([2, 2, 0.25, 2, 1, 1, 1, 1, 1, 0.15, 0.3, 0.3, 1, 1, 1], dtype=np.float32)

# vx, vy, yaw rate, body height, frequency, gait phase/offset/bound,
# stance fraction, swing height, body pitch/roll, stance width/length, auxiliary.
TROT = np.array([0, 0, 0, 0, 3, 0.5, 0, 0, 0.5, 0.08, 0, 0, 0.33, 0.40, 0], dtype=np.float32)

NETWORKS = {
    "adaptation": (2100, 256, 128, 2),
    "body": (2102, 512, 256, 128, 12),
}


def load_network(weights, name):
    sizes = NETWORKS[name]
    layers = []
    for index, (inputs, outputs) in enumerate(zip(sizes, sizes[1:])):
        weight = weights[f"{name}_{index}_weight"]
        bias = weights[f"{name}_{index}_bias"]
        if weight.shape != (outputs, inputs) or bias.shape != (outputs,):
            raise ValueError(f"Unexpected shape of {name} layer {index}")
        if weight.dtype != np.float32 or bias.dtype != np.float32:
            raise ValueError(f"Expected float32 weights for {name} layer {index}")
        if not (np.isfinite(weight).all() and np.isfinite(bias).all()):
            raise ValueError(f"Non-finite weights for {name} layer {index}")
        layers.append((weight, bias))
    return layers


def run_network(layers, values):
    for weight, bias in layers[:-1]:
        values = weight @ values + bias
        values = np.maximum(values, 0) + np.expm1(np.minimum(values, 0))
    weight, bias = layers[-1]
    return weight @ values + bias


def finite_vector(values, size, name):
    vector = np.asarray(values, dtype=np.float32)
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} needs {size} finite values")
    return vector


class WalkTheseWays:
    def __init__(self, checkpoint=DEFAULT_CHECKPOINT):
        with np.load(checkpoint, allow_pickle=False) as weights:
            self.adaptation = load_network(weights, "adaptation")
            self.body = load_network(weights, "body")
        self.history = np.zeros((HISTORY_LENGTH, OBSERVATION_COUNT), dtype=np.float32)
        self.action = np.zeros(12, dtype=np.float32)
        self.previous_action = np.zeros_like(self.action)
        self.reset()

    def reset(self):
        self.history.fill(0)
        self.action.fill(0)
        self.previous_action.fill(0)
        self.phase = 0.0

    def step(self, quaternion_wxyz, q, dq, velocity):
        """Run one 20 ms step using Unitree-order joints and (vx, vy, yaw rate)."""
        quaternion = finite_vector(quaternion_wxyz, 4, "IMU quaternion")
        norm = np.linalg.norm(quaternion)
        if norm < 1e-6 or not np.isfinite(norm):
            raise ValueError("Invalid IMU quaternion norm")
        w, x, y, z = quaternion / norm
        # World down transformed into the body frame by the inverse IMU rotation.
        gravity = (2 * (w * y - x * z), -2 * (w * x + y * z), 2 * (x * x + y * y) - 1)
        angles = finite_vector(q, 12, "Motor angles")[JOINT_ORDER]
        rates = finite_vector(dq, 12, "Motor velocities")[JOINT_ORDER]
        command = TROT.copy()
        command[:3] = finite_vector(velocity, 3, "Velocity command")

        # The trot shifts FL and RR by half a cycle. Stance fraction 0.5 needs no warp.
        clock = np.sin(2 * np.pi * self.phase)
        self.history[:-1] = self.history[1:]
        observation = self.history[-1]
        observation[0:3] = gravity
        observation[3:18] = command * COMMAND_SCALE
        observation[18:30] = angles - NEUTRAL
        observation[30:42] = rates * 0.05
        observation[42:54] = self.action
        observation[54:66] = self.previous_action
        observation[66:70] = (-clock, clock, clock, -clock)
        np.clip(observation, -100, 100, out=observation)

        history = self.history.ravel()
        latent = run_network(self.adaptation, history)
        action = run_network(self.body, np.concatenate((history, latent)))
        if not np.isfinite(action).all():
            self.reset()
            raise ValueError("Policy produced a non-finite action")
        self.previous_action = self.action
        self.action = np.clip(action, -10, 10)
        self.phase = (self.phase + float(command[4]) * PERIOD_NS / 1e9) % 1.0
        return (NEUTRAL + ACTION_SCALE * self.action)[JOINT_ORDER]
