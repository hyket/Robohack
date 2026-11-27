"""T2: фильтрация целей и непрерывная публикация безопасных команд."""

import os
import sys
from copy import deepcopy
from functools import lru_cache
from math import degrees, isfinite, radians
from pathlib import Path

import numpy as np

from collision_guard import CollisionGuard

JOINT_LIMITS = ((-0.863, 0.863), (-0.686, 4.501), (-2.818, -0.888))
JOINT_NAMES = tuple(
    f"{leg}_{joint}" for leg in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")
)
SERVO_MODE = 0x0A
HOLD_KP = 40.0
HOLD_KD = 2.0
# Диапазоны полей LowCmd из уточнения организаторов.
# За их пределами сохраняем проверенную q и используем позиционное удержание.
PASSTHROUGH_RANGES = {
    "dq": (-20.0, 20.0),
    "kp": (0.0, 1000.0),
    "kd": (0.0, 100.0),
    "tau": (-100.0, 100.0),
}
NUMERIC_FIELDS = ("q", *PASSTHROUGH_RANGES)
MAX_JOINT_STEP = radians(5.0)
PUBLISH_PERIOD_NS = 10_000_000
HOLD_AFTER_NS = 400_000_000


def load_collision_guard():
    default_model = Path(__file__).with_name("model")
    model_dir = Path(os.environ.get("GO1_MUJOCO_MODEL_DIR", default_model))
    default_table = Path(__file__).with_name("t2_collision_pairs.csv")
    pair_table = os.environ.get("T2_COLLISION_CSV", default_table)
    return CollisionGuard(model_dir / "scene.xml", pair_table, JOINT_NAMES)


def command_error(message):
    """Проверить конечность полей, режим и пределы углов."""
    for index, motor in enumerate(message.motor_cmd):
        for field in NUMERIC_FIELDS:
            if not isfinite(getattr(motor, field)):
                return f"motor_cmd[{index}].{field}: NaN или Inf"

    for index, joint in enumerate(JOINT_NAMES):
        motor = message.motor_cmd[index]
        if motor.mode != SERVO_MODE:
            return f"{joint}.mode: ожидался 0x0A, получено {motor.mode}"
        lower, upper = JOINT_LIMITS[index % 3]
        if not lower <= motor.q <= upper:
            return f"{joint}.q: {motor.q:g} вне [{lower:g}, {upper:g}]"

    return None


class CommandFilter:
    """Проверка цели и перехода от последней опубликованной команды."""

    def __init__(self, collision_guard=None):
        self.check_transition = (
            lru_cache(maxsize=256)(collision_guard.transition_error)
            if collision_guard is not None
            else None
        )

    @staticmethod
    def angles(message):
        return tuple(motor.q for motor in message.motor_cmd[: len(JOINT_NAMES)])

    def rejection_reason(self, message, previous_angles):
        error = command_error(message)
        if error:
            return error

        angles = self.angles(message)
        if previous_angles is not None:
            for joint, previous, target in zip(JOINT_NAMES, previous_angles, angles):
                change = abs(target - previous)
                if change > MAX_JOINT_STEP:
                    return f"{joint}.q: изменение {degrees(change):.6f}° превышает 5°"

        if self.check_transition is not None and angles != previous_angles:
            error = self.check_transition(previous_angles, angles)
            if error:
                return error

        return None

    def prepare(self, message, previous):
        # Сустав и шаг образуют один допустимый интервал. Округляем внутрь
        # только тогда, когда получили float32-представление самой границы.
        corrected = message
        for index, motor in enumerate(message.motor_cmd[: len(JOINT_NAMES)]):
            lower, upper = JOINT_LIMITS[index % 3]
            if previous is not None:
                lower = max(lower, previous[index] - MAX_JOINT_STEP)
                upper = min(upper, previous[index] + MAX_JOINT_STEP)
            if motor.q < lower:
                boundary, inward = lower, upper
            elif motor.q > upper:
                boundary, inward = upper, lower
            else:
                continue
            edge = np.float32(boundary)
            if motor.q == float(edge):
                if corrected is message:
                    corrected = deepcopy(message)
                corrected.motor_cmd[index].q = float(np.nextafter(edge, np.float32(inward)))

        error = self.rejection_reason(corrected, previous)
        if error is not None:
            return None, error
        return with_safe_drive_fields(corrected), None


def set_hold_fields(motor):
    motor.mode = SERVO_MODE
    motor.dq = 0.0
    motor.tau = 0.0
    motor.kp = HOLD_KP
    motor.kd = HOLD_KD


def with_safe_drive_fields(message):
    """После проверки позы скорректировать приводы, сохранив входное сообщение."""
    corrected = message
    for index, motor in enumerate(message.motor_cmd[: len(JOINT_NAMES)]):
        if all(lo <= getattr(motor, field) <= hi for field, (lo, hi) in PASSTHROUGH_RANGES.items()):
            continue
        if corrected is message:
            corrected = deepcopy(message)
        set_hold_fields(corrected.motor_cmd[index])
    return corrected


def hold_command(message_type, angles):
    message = message_type()
    for motor, angle in zip(message.motor_cmd, angles):
        motor.q = angle
        set_hold_fields(motor)
    return message


class CommandStream:
    """Публикует сохранённую цель, пока рабочий поток проверяет новую."""

    def __init__(self, message_type, collision_guard, worker):
        self.message_type = message_type
        self.command_filter = CommandFilter(collision_guard)
        self.worker = worker
        self.checking = None
        self.pending = None
        self.last_input_ns = None
        self.holding = True

        # Сравниваем следующий шаг с теми углами float32, которые получит подписчик LowCmd.
        initial_angles = tuple(float(np.float32(q)) for q in collision_guard.home_angles)
        initial_hold = hold_command(message_type, initial_angles)
        initial, error = self.command_filter.prepare(initial_hold, None)
        if error:
            raise ValueError(f"Стартовая поза home не прошла проверку: {error}")
        self.last_angles = self.command_filter.angles(initial)
        self.last_command = self.hold = initial

    def receive(self, message, now_ns):
        self.pending = (message, now_ns)
        self.last_input_ns = now_ns

    @staticmethod
    def is_fresh(now_ns, received_ns):
        return received_ns is not None and 0 <= now_ns - received_ns < HOLD_AFTER_NS

    def accept(self, message):
        self.last_angles = self.command_filter.angles(message)
        self.last_command = message
        self.hold = hold_command(self.message_type, self.last_angles)
        self.holding = False

    def prepare(self, message, previous_angles):
        return self.command_filter.prepare(message, previous_angles)

    def collect_result(self, now_ns, fresh):
        if self.checking is None or not self.checking[0].done():
            return None
        future, received_ns = self.checking
        self.checking = None
        if future.cancelled():
            self.holding = True
            return "проверка геометрии отменена"
        failure = future.exception()
        if failure is not None:
            self.holding = True
            return f"сбой проверки геометрии: {failure}"
        message, error = future.result()

        if fresh and self.is_fresh(now_ns, received_ns):
            if error is None:
                self.accept(message)
            return error
        return None

    def publish_tick(self, now_ns, publish):
        fresh = self.is_fresh(now_ns, self.last_input_ns)
        if not fresh:
            self.holding = True
            self.pending = None

        error = self.collect_result(now_ns, fresh)
        publish(self.hold if self.holding else self.last_command)
        if self.checking is None and self.pending is not None:
            message, received_ns = self.pending
            self.pending = None
            future = self.worker.submit(self.prepare, message, self.last_angles)
            self.checking = (future, received_ns)
        return error


def create_safety_block(validation_worker, cli_args=None, stream_type=CommandStream):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMessage
    from unitree_go.msg import LowCmd

    collision_guard = load_collision_guard()

    class SafetyBlock(Node):
        def __init__(self):
            super().__init__("safety_block", cli_args=cli_args)
            self.sim_time_ns = 0
            self.stream = stream_type(LowCmd, collision_guard, worker=validation_worker)
            self.checked_commands = self.create_publisher(LowCmd, "/lowcmd", 20)
            command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(LowCmd, "/lowcmd_bare", self.on_command, command_qos)
            self.create_subscription(ClockMessage, "/clock", self.on_clock, clock_qos)

        def on_command(self, message):
            self.stream.receive(message, self.sim_time_ns)

        def on_clock(self, message):
            now_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec
            previous_ns = self.sim_time_ns
            self.sim_time_ns = now_ns
            if now_ns <= 0:
                return
            # Повтор /clock и дополнительные отметки в пределах 10 мс не публикуем.
            if previous_ns > 0 and previous_ns // PUBLISH_PERIOD_NS == now_ns // PUBLISH_PERIOD_NS:
                return
            error = self.stream.publish_tick(now_ns, self.checked_commands.publish)
            if error:
                self.get_logger().warning(f"Команда отклонена: {error}", throttle_duration_sec=1.0)

    return SafetyBlock()


def run_ros():
    from concurrent.futures import ThreadPoolExecutor

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    safety_block = None
    try:
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="t2_collision") as worker:
            safety_block = create_safety_block(worker)
            rclpy.spin(safety_block)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if safety_block is not None:
            safety_block.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] in (
        "--self-test",
        "--collision-test",
        "--clock-test",
        "--external-test",
    ):
        from t2_checks import run_check

        run_check(sys.argv[1])
    else:
        run_ros()
