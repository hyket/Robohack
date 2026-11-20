#!/usr/bin/env python3
"""T2, этап 4: проверка полей LowCmd, шага суставов и самоколлизий."""

import os
import sys
from math import ceil, degrees, isfinite, radians
from pathlib import Path

from collision_guard import CollisionGuard

JOINT_LIMITS = ((-0.863, 0.863), (-0.686, 4.501), (-2.818, -0.888))
JOINT_NAMES = tuple(
    f"{leg}_{joint}"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)
MOTOR_LIMITS = {
    "dq": (-20.0, 20.0),
    "kp": (0.0, 1000.0),
    "kd": (0.0, 100.0),
    "tau": (-100.0, 100.0),
}
NUMERIC_FIELDS = ("q", *MOTOR_LIMITS)
MAX_JOINT_STEP = radians(5.0)

# Пара поз эталонного Go1 для проверки столкновения в промежуточной точке.
COLLISION_START = (
    0.320791645, 0.542898732, -1.8607834, -0.5493525, 0.313605809, -1.615461681,
    0.320918185, 3.292168909, -2.074346309, -0.163734196, 1.458956409, -2.260628817,
)
COLLISION_END = (
    0.339921787, 0.462880971, -1.902365973, -0.53338043, 0.399825074, -1.572133109,
    0.259998592, 3.31342055, -2.063665095, -0.143715825, 1.538725473, -2.240161272,
)


def load_collision_guard():
    model_dir = Path(os.environ.get("GO1_MUJOCO_MODEL_DIR", "/opt/mujoco_menagerie/unitree_go1"))
    default_table = Path(__file__).with_name("t2_collision_pairs.csv")
    pair_table = os.environ.get("T2_COLLISION_CSV", default_table)
    return CollisionGuard(model_dir / "scene.xml", pair_table, JOINT_NAMES)


def command_error(message):
    """Вернуть причину отклонения или None, если числовые проверки пройдены."""
    for index, motor in enumerate(message.motor_cmd):
        for field in NUMERIC_FIELDS:
            if not isfinite(getattr(motor, field)):
                return f"motor_cmd[{index}].{field}: NaN или Inf"

    for index, joint in enumerate(JOINT_NAMES):
        motor = message.motor_cmd[index]
        lower, upper = JOINT_LIMITS[index % 3]
        if not lower <= motor.q <= upper:
            return f"{joint}.q: {motor.q:g} вне [{lower:g}, {upper:g}]"
        for field, (lower, upper) in MOTOR_LIMITS.items():
            value = getattr(motor, field)
            if not lower <= value <= upper:
                return f"{joint}.{field}: {value:g} вне [{lower:g}, {upper:g}]"

    return None


class CommandFilter:
    """Проверка цели и перехода от последней опубликованной команды."""

    def __init__(self, collision_guard=None):
        self.last_published_angles = None
        self.collision_guard = collision_guard

    def forward_if_valid(self, message, publish):
        error = command_error(message)
        if error:
            return error

        angles = tuple(motor.q for motor in message.motor_cmd[:len(JOINT_NAMES)])
        if self.last_published_angles is not None:
            for joint, previous, target in zip(JOINT_NAMES, self.last_published_angles, angles):
                change = abs(target - previous)
                if change > MAX_JOINT_STEP:
                    return f"{joint}.q: изменение {degrees(change):.6f}° превышает 5°"

        if self.collision_guard is not None:
            error = self.collision_guard.transition_error(self.last_published_angles, angles)
            if error:
                return error

        publish(message)
        self.last_published_angles = angles
        return None


def sample_command(message_type, angles=(0.0, 0.9, -1.8) * 4):
    message = message_type()
    for index, angle in enumerate(angles):
        motor = message.motor_cmd[index]
        motor.mode = 10
        motor.q = angle
        motor.kp = 25.0
        motor.kd = 1.0
    return message


def test_commands(message_type):
    """Примеры для автономной проверки и для обмена сообщениями в ROS 2."""
    yield "допустимая команда", sample_command(message_type), None
    for bound, label in enumerate(("нижние границы", "верхние границы")):
        message = sample_command(message_type)
        for field, interval in MOTOR_LIMITS.items():
            setattr(message.motor_cmd[11], field, interval[bound])
        yield label, message, None

    for index, field, value, expected in (
        (0, "q", 1.2, "FR_hip.q"),
        (1, "q", 4.6, "FR_thigh.q"),
        (11, "q", -0.8, "RL_calf.q"),
        (0, "kp", -1.0, "FR_hip.kp"),
        (11, "kp", 1001.0, "RL_calf.kp"),
        (3, "kd", -1.0, "FL_hip.kd"),
        (11, "kd", 101.0, "RL_calf.kd"),
        (6, "tau", -101.0, "RR_hip.tau"),
        (11, "tau", 101.0, "RL_calf.tau"),
        (7, "dq", -21.0, "RR_thigh.dq"),
        (11, "dq", 21.0, "RL_calf.dq"),
        (0, "q", float("nan"), "motor_cmd[0].q"),
        (11, "tau", float("inf"), "motor_cmd[11].tau"),
        (19, "dq", float("-inf"), "motor_cmd[19].dq"),
    ):
        message = sample_command(message_type)
        setattr(message.motor_cmd[index], field, value)
        yield f"{expected} = {value:g}", message, expected


def transition_commands(message_type):
    """Последовательность целей. Углы в таблице заданы относительно исходной позы."""
    for label, hip_offset, calf_offset, rejected_joint in (
        ("исходная цель", 0, 0, None),
        ("оба сустава изменились на 4°", 4, 4, None),
        ("FR_hip: с 4° до 10°", 10, 4, "FR_hip.q"),
        ("FR_hip: 11° сравниваются с принятыми 4°", 11, 4, "FR_hip.q"),
        ("FR_hip: с 4° до 7°", 7, 4, None),
        ("RL_calf: с 4° до 10°", 7, 10, "RL_calf.q"),
        ("FR_hip: с 7° до 3°", 3, 4, None),
        ("FR_hip: с 3° до −3°", -3, 4, "FR_hip.q"),
        ("возврат к исходной цели", 0, 0, None),
        ("FR_hip: ровно 5° от исходной цели", 5, 0, None),
        ("FR_hip: следующая цель 2°", 2, 0, None),
    ):
        message = sample_command(message_type)
        message.motor_cmd[0].q += radians(hip_offset)
        message.motor_cmd[11].q += radians(calf_offset)
        yield label, message, rejected_joint


def self_test():
    from types import SimpleNamespace

    def make_message():
        return SimpleNamespace(
            motor_cmd=[SimpleNamespace(**dict.fromkeys(NUMERIC_FIELDS, 0.0))
                       for _ in range(20)]
        )

    numeric_count = 0
    for label, message, expected in test_commands(make_message):
        error = command_error(message)
        actual = error.split(":", 1)[0] if error else None
        if actual != expected:
            raise RuntimeError(f"{label}: ожидалось {expected!r}, получено {error!r}")
        numeric_count += 1

    command_filter = CommandFilter()
    forwarded = []
    transition_count = 0
    for label, message, expected in transition_commands(make_message):
        forwarded.clear()
        error = command_filter.forward_if_valid(message, forwarded.append)
        actual = error.split(":", 1)[0] if error else None
        expected_output = [message] if expected is None else []
        if actual != expected or forwarded != expected_output:
            raise RuntimeError(f"{label}: ожидалось {expected!r}, получено {error!r}")
        transition_count += 1
    print(f"Проверка без ROS 2: {numeric_count} числовых и {transition_count} проверок переходов пройдено.")


def collision_test(guard):
    if guard.pose_collision(COLLISION_START) or guard.pose_collision(COLLISION_END):
        raise RuntimeError("Обе конечные позы тестового перехода должны быть допустимыми.")
    differences = tuple(end - start for start, end in zip(COLLISION_START, COLLISION_END))
    if max(map(abs, differences)) > MAX_JOINT_STEP:
        raise RuntimeError("Тестовый переход должен укладываться в 5°.")
    middle = tuple(start + change / 2 for start, change in zip(COLLISION_START, differences))
    hit = guard.pose_collision(middle)
    if hit is None or not guard.transition_error(COLLISION_START, COLLISION_END):
        raise RuntimeError("В промежуточной позе должна обнаруживаться самоколлизия.")
    if guard.transition_error(None, middle) is None:
        raise RuntimeError("Самоколлизия в первой целевой позе должна быть обнаружена.")
    print("MuJoCo: проверено 655 пар. Начальная и конечная позы допустимы.", flush=True)
    print(f"MuJoCo: {guard.describe(hit, 'середина тестового перехода')}.", flush=True)


def test_ros_exchange(executor, probe, message_type, input_topic, output_topic):
    from time import monotonic

    from rclpy.serialization import deserialize_message, serialize_message

    received = []
    sender = probe.create_publisher(message_type, input_topic, 20)
    probe.create_subscription(message_type, output_topic, received.append, 20)

    def spin_until(condition, timeout, error):
        deadline = monotonic() + timeout
        while not condition():
            if monotonic() >= deadline:
                raise RuntimeError(error)
            executor.spin_once(timeout_sec=0.05)

    def verify_unchanged(sent, actual, label):
        normalized = deserialize_message(serialize_message(sent), message_type)
        if normalized != actual:
            raise RuntimeError(f"{label}: фильтр изменил допустимую команду.")

    spin_until(
        lambda: sender.get_subscription_count() > 0
        and probe.count_publishers(output_topic) > 0,
        15.0,
        "Тестовые узлы ROS 2 не обнаружили друг друга.",
    )
    for test_id, (label, message, expected) in enumerate(test_commands(message_type), 1):
        received.clear()
        # Метка в crc связывает ответ с конкретным тестовым сообщением.
        message.crc = 2 * test_id
        marker = sample_command(message_type)
        marker.crc = message.crc + 1
        sender.publish(message)
        sender.publish(marker)
        spin_until(
            lambda: any(item.crc == marker.crc for item in received),
            3.0,
            f"{label}: контрольная команда не появилась на выходе.",
        )
        expected_ids = [marker.crc] if expected else [message.crc, marker.crc]
        actual_ids = [item.crc for item in received]
        if expected_ids != actual_ids or any(command_error(item) for item in received):
            raise RuntimeError(f"{label}: неверный результат фильтра, получено {actual_ids}.")
        if expected is None:
            verify_unchanged(message, received[0], label)
        print(f"OK — {label}: {'отклонено' if expected else 'пропущено'}", flush=True)

    received.clear()
    transitions = list(transition_commands(message_type))
    for test_id, (_, message, _) in enumerate(transitions, 100):
        message.crc = test_id
        sender.publish(message)

    final_id = transitions[-1][1].crc
    spin_until(
        lambda: any(item.crc == final_id for item in received),
        3.0,
        "Проверка переходов: последняя контрольная команда не появилась на выходе.",
    )
    actual_by_id = {item.crc: item for item in received}
    for label, message, expected in transitions:
        actual = actual_by_id.get(message.crc)
        should_publish = expected is None
        if (actual is not None) != should_publish:
            wanted = "публикация" if should_publish else "отклонение"
            raise RuntimeError(f"{label}: ожидалось {wanted}.")
        if actual is not None:
            verify_unchanged(message, actual, label)
        print(f"OK — {label}: {'отклонено' if expected else 'пропущено'}", flush=True)
    expected_ids = [message.crc for _, message, error in transitions if error is None]
    if [item.crc for item in received] != expected_ids:
        raise RuntimeError("Проверка переходов: команды опубликованы повторно или в неверном порядке.")

    # Подходим к начальной позе специального перехода шагами не больше 4°.
    previous = tuple(motor.q for motor in received[-1].motor_cmd[:len(JOINT_NAMES)])
    changes = tuple(end - start for start, end in zip(previous, COLLISION_START))
    segments = max(1, ceil(max(map(abs, changes)) / radians(4.0)))
    for index in range(1, segments + 1):
        received.clear()
        fraction = index / segments
        angles = tuple(start + fraction * change for start, change in zip(previous, changes))
        approach = sample_command(message_type, angles)
        approach.crc = 200 + index
        sender.publish(approach)
        spin_until(
            lambda: bool(received), 3.0,
            f"Безопасный шаг {index}/{segments} к тестовой позе отклонён.",
        )
        verify_unchanged(approach, received[0], f"Шаг к тестовой позе {index}")

    received.clear()
    colliding = sample_command(message_type, COLLISION_END)
    colliding.crc = 200 + segments + 1
    hold = sample_command(message_type, COLLISION_START)
    hold.crc = colliding.crc + 1
    sender.publish(colliding)
    sender.publish(hold)
    spin_until(
        lambda: any(item.crc == hold.crc for item in received), 3.0,
        "Команда с последней допустимой целью не появилась на выходе.",
    )
    if [item.crc for item in received] != [hold.crc]:
        raise RuntimeError("Переход с промежуточной самоколлизией попал на выход.")
    verify_unchanged(hold, received[0], "Последняя допустимая цель")
    print("OK — переход с самоколлизией отклонён по ROS 2.", flush=True)
    print("ЭТАП 4 ПРОЙДЕН: проверены самоколлизии по всему переходу.", flush=True)


def run_ros(integration_test=False):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from unitree_go.msg import LowCmd

    collision_guard = load_collision_guard()
    if integration_test:
        collision_test(collision_guard)

    class SafetyBlock(Node):
        def __init__(self, cli_args=None):
            super().__init__("safety_block", cli_args=cli_args)
            self.command_filter = CommandFilter(collision_guard)
            self.checked_commands = self.create_publisher(LowCmd, "/lowcmd", 20)
            self.create_subscription(LowCmd, "/lowcmd_bare", self.on_command, 20)

        def on_command(self, message):
            error = self.command_filter.forward_if_valid(message, self.checked_commands.publish)
            if error:
                self.get_logger().warning(f"Команда отклонена: {error}", throttle_duration_sec=1.0)

    test_input = "/t2_stage4/lowcmd_bare"
    test_output = "/t2_stage4/lowcmd_checked"
    rclpy.init(args=[] if integration_test else None)
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        # Тестовые команды проходят по отдельным топикам.
        remapping = ["--ros-args", "-r", f"/lowcmd_bare:={test_input}",
                     "-r", f"/lowcmd:={test_output}"] if integration_test else None
        safety_block = SafetyBlock(cli_args=remapping)
        nodes.append(safety_block)
        executor.add_node(safety_block)
        if integration_test:
            probe = Node("t2_stage4_probe")
            nodes.append(probe)
            executor.add_node(probe)
            test_ros_exchange(executor, probe, LowCmd, test_input, test_output)
        else:
            executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        if integration_test:
            raise
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        self_test()
    elif sys.argv[1:] == ["--collision-test"]:
        collision_test(load_collision_guard())
    else:
        run_ros(integration_test=sys.argv[1:] == ["--integration-test"])
