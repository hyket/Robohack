#!/usr/bin/env python3
"""T2, этап 2: проверка чисел и диапазонов полей LowCmd."""

import sys
from math import isfinite

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


def sample_command(message_type):
    message = message_type()
    for index in range(len(JOINT_NAMES)):
        motor = message.motor_cmd[index]
        motor.mode = 10
        motor.q = (0.0, 0.9, -1.8)[index % 3]
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


def self_test():
    from types import SimpleNamespace

    def make_message():
        return SimpleNamespace(
            motor_cmd=[SimpleNamespace(**dict.fromkeys(NUMERIC_FIELDS, 0.0))
                       for _ in range(20)]
        )

    count = 0
    for label, message, expected in test_commands(make_message):
        error = command_error(message)
        actual = error.split(":", 1)[0] if error else None
        if actual != expected:
            raise RuntimeError(f"{label}: ожидалось {expected!r}, получено {error!r}")
        count += 1
    print(f"Проверка фильтра без ROS 2: {count} тестов пройдено.")


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
            normalized = deserialize_message(serialize_message(message), message_type)
            if normalized != received[0]:
                raise RuntimeError(f"{label}: фильтр изменил допустимую команду.")
        print(f"OK — {label}: {'отклонено' if expected else 'пропущено'}", flush=True)
    print("ЭТАП 2 ПРОЙДЕН: проверен обмен командами по ROS 2.", flush=True)


def run_ros(integration_test=False):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from unitree_go.msg import LowCmd

    class SafetyBlock(Node):
        def __init__(self, cli_args=None):
            super().__init__("safety_block", cli_args=cli_args)
            self.checked_commands = self.create_publisher(LowCmd, "/lowcmd", 20)
            self.create_subscription(LowCmd, "/lowcmd_bare", self.on_command, 20)

        def on_command(self, message):
            error = command_error(message)
            if error:
                self.get_logger().warning(f"Команда отклонена: {error}", throttle_duration_sec=1.0)
                return
            self.checked_commands.publish(message)

    test_input = "/t2_stage2/lowcmd_bare"
    test_output = "/t2_stage2/lowcmd_checked"
    rclpy.init(args=[] if integration_test else None)
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        # На испытании первого фильтра перенаправляем топики от приводов MuJoCo.
        remapping = ["--ros-args", "-r", f"/lowcmd_bare:={test_input}",
                     "-r", f"/lowcmd:={test_output}"] if integration_test else None
        safety_block = SafetyBlock(cli_args=remapping)
        nodes.append(safety_block)
        executor.add_node(safety_block)
        if integration_test:
            probe = Node("t2_stage2_probe")
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
    else:
        run_ros(integration_test=sys.argv[1:] == ["--integration-test"])
