"""Run the T3 policy and the existing T2 Safety Block in one Participant."""

from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from unitree_go.msg import LowCmd, LowState

from controller import Controller, PolicyCommandStream
from safety_block import JOINT_NAMES, PUBLISH_PERIOD_NS, SERVO_MODE, create_safety_block

STATE_TIMEOUT_NS = 100_000_000
NUM_JOINTS = len(JOINT_NAMES)


class PolicyNode(Node):
    def __init__(self, home_angles):
        super().__init__("t3_policy", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.controller = Controller(home_angles)
        self.velocity = (0.0, 0.0, 0.0)
        self.observation = None
        self.last_state_ns = 0
        self.sim_time_ns = 0

        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.bare_commands = self.create_publisher(LowCmd, "/lowcmd_bare", latest)
        self.create_subscription(Twist, "/cmd_vel", self.on_velocity, latest)
        self.create_subscription(LowState, "/lowstate", self.on_state, latest)
        self.create_subscription(Clock, "/clock", self.on_clock, latest)

    def on_velocity(self, message):
        # Judge is allowed to publish once and keep the command for the episode.
        values = (message.linear.x, message.linear.y, message.angular.z)
        if np.isfinite(values).all():
            self.velocity = tuple(np.clip(values, -0.6, 0.6))

    def on_state(self, message):
        motors = message.motor_state[:NUM_JOINTS]
        quaternion = np.array(message.imu_state.quaternion, dtype=np.float32)
        q = np.array([motor.q for motor in motors], dtype=np.float32)
        dq = np.array([motor.dq for motor in motors], dtype=np.float32)
        if not all(np.isfinite(values).all() for values in (quaternion, q, dq)):
            self.observation = None
            return
        if np.linalg.norm(quaternion) < 0.5:
            self.observation = None
            return
        self.observation = (quaternion, q, dq)
        self.last_state_ns = self.sim_time_ns

    def on_clock(self, message):
        previous_ns = self.sim_time_ns
        now_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        self.sim_time_ns = now_ns
        if now_ns < previous_ns:
            self.observation = None
            self.velocity = (0.0, 0.0, 0.0)
        if self.observation is None or now_ns - self.last_state_ns >= STATE_TIMEOUT_NS:
            if self.controller.next_inference_ns is not None:
                self.controller.reset()
            return
        if now_ns <= 0 or now_ns // PUBLISH_PERIOD_NS == previous_ns // PUBLISH_PERIOD_NS:
            return

        angles, kp, kd = self.controller.command(now_ns, self.observation, self.velocity)
        command = LowCmd()
        for motor, angle in zip(command.motor_cmd, angles):
            motor.mode = SERVO_MODE
            motor.q = float(angle)
            motor.kp = kp
            motor.kd = kd
        self.bare_commands.publish(command)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    try:
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="t2_collision") as worker:
            safety = create_safety_block(
                worker,
                cli_args=["--ros-args", "-p", "use_sim_time:=true"],
                stream_type=PolicyCommandStream,
            )
            executor.add_node(safety)
            executor.add_node(PolicyNode(safety.stream.last_angles))
            executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        for node in executor.get_nodes():
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
