#!/usr/bin/env python3
"""T2: фильтрация целей и непрерывная публикация безопасных команд."""

import os
import sys
from math import degrees, isfinite, radians
from pathlib import Path
from struct import pack, unpack

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
PUBLISH_PERIOD_S = 0.01
HOLD_AFTER_NS = 400_000_000

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

    @staticmethod
    def angles(message):
        return tuple(motor.q for motor in message.motor_cmd[:len(JOINT_NAMES)])

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

        if self.collision_guard is not None and angles != previous_angles:
            error = self.collision_guard.transition_error(previous_angles, angles)
            if error:
                return error

        return None

    def forward_if_valid(self, message, publish):
        error = self.rejection_reason(message, self.last_published_angles)
        if error is None:
            publish(message)
            self.last_published_angles = self.angles(message)
        return error


def hold_command(message_type, angles):
    message = message_type()
    for motor, angle in zip(message.motor_cmd, angles):
        motor.mode = 10
        motor.q = angle
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = 40.0
        motor.kd = 2.0
    return message


class CommandStream:
    """Публикует сохранённую цель, пока рабочий поток проверяет новую."""

    def __init__(self, message_type, collision_guard, worker=None):
        self.message_type = message_type
        self.command_filter = CommandFilter(collision_guard)
        self.worker = worker
        self.checking = None
        self.pending = None
        self.last_input_ns = None
        self.holding = True

        # Сравниваем следующий шаг с теми углами float32, которые получит подписчик LowCmd.
        initial_angles = tuple(unpack("<f", pack("<f", q))[0] for q in collision_guard.home_angles)
        initial_hold = hold_command(message_type, initial_angles)
        error = self.command_filter.forward_if_valid(initial_hold, self.remember_command)
        if error:
            raise ValueError(f"Стартовая поза home не прошла проверку: {error}")

    def remember_command(self, message):
        angles = self.command_filter.angles(message)
        self.last_command = message
        self.hold = hold_command(self.message_type, angles)

    def receive(self, message, now_ns):
        self.pending = (message, now_ns)
        self.last_input_ns = now_ns

    @staticmethod
    def is_fresh(now_ns, received_ns):
        return received_ns is not None and 0 <= now_ns - received_ns < HOLD_AFTER_NS

    def accept(self, message):
        self.remember_command(message)
        self.command_filter.last_published_angles = self.command_filter.angles(message)
        self.holding = False

    def collect_result(self, now_ns, fresh):
        if self.checking is None or not self.checking[0].done():
            return None
        future, message, received_ns = self.checking
        self.checking = None
        try:
            error = future.result()
        except Exception as exc:
            self.holding = True
            return f"сбой проверки геометрии: {exc}"

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
        if self.worker is None and self.pending is not None:
            message, _ = self.pending
            self.pending = None
            error = self.command_filter.rejection_reason(
                message, self.command_filter.last_published_angles
            )
            if error is None:
                self.accept(message)

        publish(self.hold if self.holding else self.last_command)
        if self.worker is not None and self.checking is None and self.pending is not None:
            message, received_ns = self.pending
            self.pending = None
            future = self.worker.submit(
                self.command_filter.rejection_reason,
                message, self.command_filter.last_published_angles,
            )
            self.checking = (future, message, received_ns)
        return error


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


def test_message_type():
    from types import SimpleNamespace

    return SimpleNamespace(
        crc=0, motor_cmd=[SimpleNamespace(mode=0, **dict.fromkeys(NUMERIC_FIELDS, 0.0))
                          for _ in range(20)]
    )


def self_test():
    numeric_count = 0
    for label, message, expected in test_commands(test_message_type):
        error = command_error(message)
        actual = error.split(":", 1)[0] if error else None
        if actual != expected:
            raise RuntimeError(f"{label}: ожидалось {expected!r}, получено {error!r}")
        numeric_count += 1

    command_filter = CommandFilter()
    forwarded = []
    transition_count = 0
    for label, message, expected in transition_commands(test_message_type):
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


def verify_hold(message, accepted):
    count = len(JOINT_NAMES)
    for motor, previous in zip(message.motor_cmd[:count], accepted.motor_cmd[:count]):
        if (motor.q != previous.q or motor.mode != 10
                or motor.dq != 0.0 or motor.tau != 0.0):
            return False
    return command_error(message) is None


def stream_logic_test(guard):
    stream = CommandStream(test_message_type, guard)
    messages = []

    def tick(now, should_reject=False):
        messages.clear()
        error = stream.publish_tick(now, messages.append)
        if len(messages) != 1 or bool(error) != should_reject:
            raise RuntimeError(f"Тик {now}: неверная публикация или решение фильтра.")
        return messages[0]

    initial = tick(0)
    initial_angles = (motor.q for motor in initial.motor_cmd[:len(JOINT_NAMES)])
    if not verify_hold(initial, initial) or any(
        abs(actual - home) > 1e-6 for actual, home in zip(initial_angles, guard.home_angles)
    ):
        raise RuntimeError("Начальная публикация должна удерживать позу home.")

    accepted = sample_command(test_message_type)
    accepted.motor_cmd[0].q = radians(3.0)
    accepted.motor_cmd[0].dq = 0.5
    accepted.motor_cmd[0].tau = 6.25
    stream.receive(accepted, 10_000_000)
    if tick(20_000_000) is not accepted:
        raise RuntimeError("Допустимая цель не принята.")

    invalid = sample_command(test_message_type)
    invalid.motor_cmd[0].kp = 1001.0
    cutoff = 30_000_000 + HOLD_AFTER_NS
    stream.receive(invalid, 30_000_000)
    if tick(40_000_000, should_reject=True) is not accepted or tick(cutoff - 1) is not accepted:
        raise RuntimeError("Недопустимая цель поменяла последнюю принятую команду.")
    if not verify_hold(tick(cutoff), accepted):
        raise RuntimeError("Не началось удержание по истечении тайм-аута.")
    stream.receive(invalid, cutoff + 10_000_000)
    if not verify_hold(tick(cutoff + 20_000_000, should_reject=True), accepted):
        raise RuntimeError("Недопустимая команда прекратила удержание.")

    stream.receive(accepted, cutoff + 30_000_000)
    if tick(cutoff + 40_000_000) is not accepted:
        raise RuntimeError("Не восстановлена обработка после поступления допустимой команды.")
    print("Логика потока: старт, отклонение, удержание и возобновление проверены.", flush=True)


def background_logic_test(guard):
    from concurrent.futures import Future
    from types import SimpleNamespace

    jobs = []

    def submit(check, *args):
        future = Future()
        jobs.append((future, check, args))
        return future

    def finish_check():
        future, check, args = jobs.pop(0)
        future.set_result(check(*args))

    stream = CommandStream(test_message_type, guard, worker=SimpleNamespace(submit=submit))
    published = []
    first = sample_command(test_message_type)
    first.motor_cmd[0].q = radians(2)
    first.motor_cmd[0].tau = 6.0
    stream.receive(first, 1)
    stream.publish_tick(10_000_000, published.append)
    if published[-1].motor_cmd[0].q != 0.0 or len(jobs) != 1:
        raise RuntimeError("Непроверенная цель попала на выход.")
    finish_check()
    stream.publish_tick(20_000_000, published.append)
    if published[-1] is not first:
        raise RuntimeError("Проверенная цель не опубликована.")

    delayed = sample_command(test_message_type)
    delayed.motor_cmd[0].q = radians(4)
    stream.receive(delayed, 30_000_000)
    for now in range(30_000_000, 430_000_000, 10_000_000):
        stream.publish_tick(now, published.append)
        if published[-1] is not first or len(jobs) != 1:
            raise RuntimeError("Длительная проверка заблокировала или изменила публикацию.")
    stream.publish_tick(430_000_000, published.append)
    if not verify_hold(published[-1], first):
        raise RuntimeError("Длительная проверка заблокировала удержание.")

    recovery = sample_command(test_message_type)
    recovery.motor_cmd[0].q = radians(1)
    stream.receive(recovery, 440_000_000)
    finish_check()
    stream.publish_tick(450_000_000, published.append)
    if not verify_hold(published[-1], first) or len(jobs) != 1:
        raise RuntimeError("Устаревший результат проверки попал на выход.")
    finish_check()
    stream.publish_tick(460_000_000, published.append)
    if published[-1] is not recovery:
        raise RuntimeError("После задержки не принята новая проверенная цель.")
    print("Фоновая проверка: таймер продолжает публикацию и удержание при задержке MuJoCo.",
          flush=True)


def test_ros_exchange(executor, probe, message_type, input_topic, output_topic):
    from time import monotonic, monotonic_ns

    from rclpy.serialization import deserialize_message, serialize_message
    from rosgraph_msgs.msg import Clock as ClockMessage

    def now_ns():
        return probe.get_clock().now().nanoseconds

    received = []
    arrival_times = []
    clock_stamps = []
    sender = probe.create_publisher(message_type, input_topic, 10)

    def on_output(message):
        stamp = now_ns()
        received.append((stamp, message))
        arrival_times.append((stamp, monotonic_ns()))

    def on_clock(message):
        clock_stamps.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)

    probe.create_subscription(message_type, output_topic, on_output, 100)
    probe.create_subscription(ClockMessage, "/clock", on_clock, 100)

    def spin_until(condition, timeout, error):
        deadline = monotonic() + timeout
        while not condition():
            if monotonic() >= deadline:
                raise RuntimeError(error)
            executor.spin_once(timeout_sec=0.01)

    spin_until(
        lambda: now_ns() > 0 and sender.get_subscription_count() > 0 and bool(received),
        15.0,
        "Не получены /clock или начальный поток команд.",
    )

    active = sample_command(message_type)
    active.motor_cmd[0].q = radians(3.0)
    active.motor_cmd[0].dq = 0.5
    active.motor_cmd[0].tau = 6.25
    active.crc = 101
    normalized = deserialize_message(serialize_message(active), message_type)
    sender.publish(active)
    spin_until(
        lambda: any(cmd.crc == active.crc for _, cmd in received),
        5.0,
        "Допустимая подвижная цель не появилась на выходе.",
    )
    if received[-1][1] != normalized:
        raise RuntimeError("Допустимая команда передана с изменениями.")

    bad = sample_command(message_type)
    bad.motor_cmd[0].kp = 1001.0
    bad.crc = 102
    received.clear()
    last_sent = now_ns()
    sender.publish(bad)
    spin_until(
        lambda: any(verify_hold(cmd, normalized) for _, cmd in received),
        8.0,
        "После исчезновения входа не появилось удержание принятой цели.",
    )

    hold_started = next(stamp for stamp, cmd in received if verify_hold(cmd, normalized))
    delay_ms = (hold_started - last_sent) / 1_000_000
    if not 0 <= delay_ms <= 500.0:
        raise RuntimeError(f"Удержание началось через {delay_ms:.1f} мс, предел — 500 мс.")
    before_hold = [cmd for stamp, cmd in received if stamp < hold_started]
    if not before_hold or any(cmd != normalized for cmd in before_hold):
        raise RuntimeError("Недопустимая команда изменила поток до удержания.")
    if any(cmd.crc == bad.crc for _, cmd in received):
        raise RuntimeError("Недопустимая команда попала на выход.")
    print(f"OK — удержание через {delay_ms:.1f} мс. Цель q сохранена, dq и tau равны нулю.", flush=True)

    received.clear()
    sender.publish(bad)
    pause_end = now_ns() + 150_000_000
    spin_until(lambda: now_ns() >= pause_end, 5.0, "Время /clock остановилось.")
    if not received or any(not verify_hold(cmd, normalized) for _, cmd in received):
        raise RuntimeError("Недопустимая команда прервала удержание.")
    print("OK — недопустимая команда после тайм-аута не прерывает удержание.", flush=True)

    received.clear()
    start_ns = now_ns()
    end_ns = start_ns + 5_000_000_000
    next_input_ns = start_ns
    wall_deadline = monotonic() + 35.0
    counter = 0
    while now_ns() < end_ns:
        now = now_ns()
        if monotonic() >= wall_deadline:
            raise RuntimeError("Симуляция не прошла окно измерения в 5 секунд.")
        if now >= next_input_ns:
            candidate = sample_command(message_type)
            candidate.motor_cmd[0].q = radians(4.8 if counter % 2 else 0.0)
            candidate.crc = 1000 + counter
            sender.publish(candidate)
            next_input_ns = now + 20_000_000
            counter += 1
        executor.spin_once(timeout_sec=0.01)

    records = [(stamp, cmd) for stamp, cmd in received if start_ns <= stamp < end_ns]
    stamps = [start_ns, *(stamp for stamp, _ in records), end_ns]
    frequency = len(records) / 5.0
    max_gap_ms = max(end - start for start, end in zip(stamps, stamps[1:])) / 1_000_000
    window_clock = [stamp for stamp in clock_stamps if start_ns <= stamp <= end_ns]
    window_arrivals = [wall for stamp, wall in arrival_times if start_ns <= stamp < end_ns]

    def biggest_step(samples):
        return max((b - a for a, b in zip(samples, samples[1:])), default=0) / 1_000_000

    if len(window_clock) >= 2:
        print(f"Диагностика: наибольший скачок /clock — {biggest_step(window_clock):.1f} мс; "
              f"интервал доставки по часам компьютера — {biggest_step(window_arrivals):.1f} мс.",
              flush=True)
    else:
        print("Диагностика: недостаточно отдельных сообщений /clock для замера его скачков.", flush=True)
    if any(command_error(cmd) or cmd.crc == bad.crc for _, cmd in records):
        raise RuntimeError("Выходной поток содержит недопустимую команду.")
    published_targets = {
        round(degrees(cmd.motor_cmd[0].q), 1) for _, cmd in records if cmd.crc >= 1000
    }
    if published_targets != {0.0, 4.8}:
        raise RuntimeError("После удержания узел не отработал обе новые цели.")
    if frequency < 50 or max_gap_ms > 100:
        raise RuntimeError(
            f"Выход: {frequency:.1f} Гц, максимальный интервал {max_gap_ms:.1f} мс. "
            "Требуется ≥50 Гц и ≤100 мс за 5 секунд симуляции."
        )
    print(f"OK — 5 секунд симуляции: {frequency:.1f} Гц, максимальный интервал {max_gap_ms:.1f} мс.",
          flush=True)
    print("Проверены непрерывная публикация и удержание.", flush=True)


def run_external_test():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from unitree_go.msg import LowCmd

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    probe = None
    try:
        probe = Node("t2_participant_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
        executor.add_node(probe)
        test_ros_exchange(executor, probe, LowCmd, "/lowcmd_bare", "/lowcmd")
    finally:
        executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_ros(integration_test=False):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Thread

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from unitree_go.msg import LowCmd

    collision_guard = load_collision_guard()
    if integration_test:
        collision_test(collision_guard)
        stream_logic_test(collision_guard)
        background_logic_test(collision_guard)

    class SafetyBlock(Node):
        def __init__(self, cli_args=None):
            super().__init__(
                "safety_block", cli_args=cli_args,
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            self.stream = CommandStream(LowCmd, collision_guard, worker=validation_worker)
            self.checked_commands = self.create_publisher(LowCmd, "/lowcmd", 20)
            input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(LowCmd, "/lowcmd_bare", self.on_command, input_qos)
            # Публикация идёт по монотонным часам, а возраст команд — по /clock.
            self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(PUBLISH_PERIOD_S, self.on_tick, clock=self.timer_clock)

        def on_command(self, message):
            self.stream.receive(message, self.get_clock().now().nanoseconds)

        def on_tick(self):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns == 0:
                return
            error = self.stream.publish_tick(now_ns, self.checked_commands.publish)
            if error:
                self.get_logger().warning(f"Команда отклонена: {error}", throttle_duration_sec=1.0)

    test_input = "/t2_stage5/lowcmd_bare"
    test_output = "/t2_stage5/lowcmd_checked"
    rclpy.init(args=[] if integration_test else None)
    executor = SingleThreadedExecutor()
    validation_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="t2_collision")
    probe_executor = None
    control_thread = None
    nodes = []

    def run_controller():
        try:
            executor.spin()
        except ExternalShutdownException:
            return

    try:
        # Тестовые команды проходят по отдельным топикам.
        remapping = ["--ros-args", "-r", f"/lowcmd_bare:={test_input}",
                     "-r", f"/lowcmd:={test_output}"] if integration_test else None
        safety_block = SafetyBlock(cli_args=remapping)
        nodes.append(safety_block)
        executor.add_node(safety_block)
        if integration_test:
            probe = Node("t2_stage5_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
            nodes.append(probe)
            probe_executor = SingleThreadedExecutor()
            probe_executor.add_node(probe)
            control_thread = Thread(target=run_controller, name="t2_controller")
            control_thread.start()
            test_ros_exchange(probe_executor, probe, LowCmd, test_input, test_output)
        else:
            executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        if integration_test:
            raise
    finally:
        executor.shutdown()
        if control_thread is not None:
            control_thread.join()
        if probe_executor is not None:
            probe_executor.shutdown()
        validation_worker.shutdown(wait=True, cancel_futures=True)
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        self_test()
    elif sys.argv[1:] == ["--collision-test"]:
        guard = load_collision_guard()
        collision_test(guard)
        stream_logic_test(guard)
        background_logic_test(guard)
    elif sys.argv[1:] == ["--external-test"]:
        run_external_test()
    else:
        run_ros(integration_test=sys.argv[1:] == ["--integration-test"])
