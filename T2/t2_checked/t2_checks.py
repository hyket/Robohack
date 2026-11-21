"""Автономные проверки Т2 и измерение обмена сообщениями ROS 2."""

import os
import sys
from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
from itertools import pairwise, product
from math import degrees, radians
from time import monotonic
from types import SimpleNamespace

import numpy as np

from safety_block import (
    HOLD_AFTER_NS,
    HOLD_KD,
    HOLD_KP,
    JOINT_LIMITS,
    JOINT_NAMES,
    MAX_JOINT_STEP,
    NUMERIC_FIELDS,
    PASSTHROUGH_RANGES,
    PUBLISH_PERIOD_NS,
    SERVO_MODE,
    CommandFilter,
    CommandStream,
    command_error,
    create_safety_block,
    load_collision_guard,
    set_hold_fields,
)

MAX_OUTPUT_GAP_NS = 100_000_000

# Позы эталонного Go1 с самоколлизией только внутри перехода.
COLLISION_START = (
    0.320791645,
    0.542898732,
    -1.8607834,
    -0.5493525,
    0.313605809,
    -1.615461681,
    0.320918185,
    3.292168909,
    -2.074346309,
    -0.163734196,
    1.458956409,
    -2.260628817,
)
COLLISION_END = (
    0.339921787,
    0.462880971,
    -1.902365973,
    -0.53338043,
    0.399825074,
    -1.572133109,
    0.259998592,
    3.31342055,
    -2.063665095,
    -0.143715825,
    1.538725473,
    -2.240161272,
)


def sample_command(message_type, angles=(0.0, 0.9, -1.8) * 4):
    message = message_type()
    for index, angle in enumerate(angles):
        motor = message.motor_cmd[index]
        motor.mode = SERVO_MODE
        motor.q = angle
        motor.kp = 25.0
        motor.kd = 1.0
    return message


def numeric_cases():
    yield "допустимая команда", sample_command(test_message_type), None
    for bound, label in enumerate(("нижние границы", "верхние границы")):
        message = sample_command(test_message_type)
        for field, interval in PASSTHROUGH_RANGES.items():
            setattr(message.motor_cmd[11], field, interval[bound])
        yield label, message, None

    for index, angle in ((0, 1.2), (1, 4.6), (11, -0.8)):
        message = sample_command(test_message_type)
        message.motor_cmd[index].q = angle
        field = f"{JOINT_NAMES[index]}.q"
        yield f"{field} = {angle:g}", message, field

    for index, mode in product(range(len(JOINT_NAMES)), (0, 1, 255)):
        message = sample_command(test_message_type)
        message.motor_cmd[index].mode = mode
        field = f"{JOINT_NAMES[index]}.mode"
        yield f"{field} = {mode}", message, field

    for index, field, value in product(range(20), NUMERIC_FIELDS, (np.nan, np.inf, -np.inf)):
        message = sample_command(test_message_type)
        setattr(message.motor_cmd[index], field, value)
        path = f"motor_cmd[{index}].{field}"
        yield f"{path} = {value}", message, path


def test_message_type():
    return SimpleNamespace(
        crc=0,
        motor_cmd=[
            SimpleNamespace(mode=0, **dict.fromkeys(NUMERIC_FIELDS, 0.0)) for _ in range(20)
        ],
    )


def test_wire_boundaries():
    checks = 0
    filt = CommandFilter()
    for index in range(len(JOINT_NAMES)):
        lower, upper = JOINT_LIMITS[index % 3]
        for edge, outward in ((lower, -np.inf), (upper, np.inf)):
            msg = sample_command(test_message_type)
            rounded = np.float32(edge)
            msg.motor_cmd[index].q = float(rounded)
            checked, error = filt.prepare(msg, None)
            if error or not lower <= checked.motor_cmd[index].q <= upper:
                raise RuntimeError(f"{JOINT_NAMES[index]}: ошибка на границе {edge}")
            previous = filt.angles(checked)
            msg.motor_cmd[index].q = float(np.nextafter(rounded, np.float32(outward)))
            rejected, _ = filt.prepare(msg, previous)
            if rejected is not None:
                raise RuntimeError("Значение за пределом округления попало на выход")
            checks += 2

    for sign in (-1, 1):
        msg = sample_command(test_message_type)
        start = float(np.float32(sign * 0.1))
        msg.motor_cmd[0].q = start
        previous = filt.angles(msg)
        target = deepcopy(msg)
        wire = np.float32(start + sign * MAX_JOINT_STEP)
        target.motor_cmd[0].q = float(np.nextafter(wire, np.float32(sign * np.inf)))
        rejected, _ = filt.prepare(target, previous)
        if rejected is not None:
            raise RuntimeError("Принята команда за пределом округления шага 5°")
        target.motor_cmd[0].q = float(wire)
        checked, error = filt.prepare(target, previous)
        if error or not 0 < sign * (checked.motor_cmd[0].q - start) <= MAX_JOINT_STEP:
            raise RuntimeError("Граничный шаг 5° должен быть принят внутри лимита")
        checks += 2
    print(f"Проверки границ float32: {checks} пройдено.")


def self_test():
    test_wire_boundaries()
    command_filter = CommandFilter()

    def check(label, message, expected, previous=None):
        checked, error = command_filter.prepare(message, previous)
        actual = error.partition(":")[0] if error else None
        expected_output = message if expected is None else None
        if actual != expected or checked != expected_output:
            raise RuntimeError(f"{label}: ожидалось {expected!r}, получено {error!r}")
        return command_filter.angles(checked) if checked is not None else previous

    for numeric_count, case in enumerate(numeric_cases(), 1):
        check(*case)

    # Смещения от исходной позы и сустав, на котором переход должен быть отклонён.
    transitions = (
        (0, 0, None),
        (4, 4, None),
        (10, 4, "FR_hip.q"),
        (11, 4, "FR_hip.q"),
        (7, 4, None),
        (7, 10, "RL_calf.q"),
        (3, 4, None),
        (-3, 4, "FR_hip.q"),
        (0, 0, None),
        (5, 0, None),
        (2, 0, None),
    )
    previous = None
    for hip, calf, expected in transitions:
        message = sample_command(test_message_type)
        message.motor_cmd[0].q += radians(hip)
        message.motor_cmd[11].q += radians(calf)
        previous = check(f"FR_hip {hip}°, RL_calf {calf}°", message, expected, previous)
    print(
        f"Проверка без ROS 2: {numeric_count} проверок полей и {len(transitions)} проверок переходов."
    )


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
    command = sample_command(test_message_type, COLLISION_END)
    command.motor_cmd[11].kp = 1001.0
    checked, error = CommandFilter(guard).prepare(command, COLLISION_START)
    if checked is not None or not error or "самоколлизия" not in error:
        raise RuntimeError("Фильтр должен отклонить самоколлизию до коррекции привода.")
    print("MuJoCo: проверено 655 пар. Начальная и конечная позы допустимы.", flush=True)
    print(f"MuJoCo: {guard.describe(hit, 'середина тестового перехода')}.", flush=True)


def verify_hold(message, accepted):
    count = len(JOINT_NAMES)
    for motor, previous in zip(message.motor_cmd[:count], accepted.motor_cmd[:count]):
        if (
            motor.q != previous.q
            or motor.mode != SERVO_MODE
            or motor.dq != 0.0
            or motor.tau != 0.0
            or motor.kp != HOLD_KP
            or motor.kd != HOLD_KD
        ):
            return False
    return command_error(message) is None


class ManualWorker:
    def __init__(self):
        self.jobs = []

    def submit(self, check, *args):
        future = Future()
        self.jobs.append((future, check, args))
        return future

    def finish(self):
        future, check, args = self.jobs.pop(0)
        future.set_result(check(*args))


def publish_at(stream, now_ns, should_reject=False):
    messages = []
    error = stream.publish_tick(now_ns, messages.append)
    if len(messages) != 1 or bool(error) != should_reject:
        raise RuntimeError(f"Тик {now_ns}: неверная публикация или решение фильтра.")
    return messages[0]


def stream_logic_test(guard):
    worker = ManualWorker()
    stream = CommandStream(test_message_type, guard, worker)

    def process(message, now, should_reject=False):
        stream.receive(message, now)
        publish_at(stream, now)
        worker.finish()
        return publish_at(stream, now + PUBLISH_PERIOD_NS, should_reject)

    initial = publish_at(stream, 0)
    initial_angles = (motor.q for motor in initial.motor_cmd[: len(JOINT_NAMES)])
    if not verify_hold(initial, initial) or any(
        abs(actual - home) > 1e-6 for actual, home in zip(initial_angles, guard.home_angles)
    ):
        raise RuntimeError("Начальная публикация должна удерживать позу home.")

    accepted = sample_command(test_message_type)
    accepted.motor_cmd[0].q = radians(3.0)
    accepted.motor_cmd[0].dq = 0.5
    accepted.motor_cmd[0].tau = 6.25
    if process(accepted, 10_000_000) is not accepted:
        raise RuntimeError("Допустимая цель не принята.")

    invalid = sample_command(test_message_type)
    invalid.motor_cmd[0].mode = 0
    cutoff = 30_000_000 + HOLD_AFTER_NS
    if (
        process(invalid, 30_000_000, should_reject=True) is not accepted
        or publish_at(stream, cutoff - 1) is not accepted
    ):
        raise RuntimeError("Недопустимая цель поменяла последнюю принятую команду.")
    if not verify_hold(publish_at(stream, cutoff), accepted):
        raise RuntimeError("Не началось удержание по истечении тайм-аута.")
    if not verify_hold(process(invalid, cutoff + 10_000_000, should_reject=True), accepted):
        raise RuntimeError("Недопустимая команда прекратила удержание.")

    if process(accepted, cutoff + 30_000_000) is not accepted:
        raise RuntimeError("Не восстановлена обработка после поступления допустимой команды.")

    now = cutoff + 50_000_000
    for field, (lo, hi) in PASSTHROUGH_RANGES.items():
        for outside in (lo - 1.0, hi + 1.0):
            candidate = sample_command(test_message_type)
            candidate.motor_cmd[3].q = radians(2.0)
            candidate.motor_cmd[11].tau = 1.25
            setattr(candidate.motor_cmd[3], field, outside)
            corrected = process(candidate, now)
            expected_motor = deepcopy(candidate.motor_cmd[3])
            set_hold_fields(expected_motor)
            if (
                corrected.motor_cmd[3] != expected_motor
                or corrected.motor_cmd[:3] != candidate.motor_cmd[:3]
                or corrected.motor_cmd[4:] != candidate.motor_cmd[4:]
                or getattr(candidate.motor_cmd[3], field) != outside
            ):
                raise RuntimeError(
                    f"Коррекция {field}={outside} изменила цель, вход или соседний привод."
                )
            now += 20_000_000

    candidate.motor_cmd[19].tau = float("nan")
    if process(candidate, now, should_reject=True) is not corrected:
        raise RuntimeError("Частичная коррекция сообщения с NaN.")
    print("Логика потока: старт, отклонение, удержание и возобновление проверены.", flush=True)


def background_logic_test(guard):
    worker = ManualWorker()
    stream = CommandStream(test_message_type, guard, worker)
    first = sample_command(test_message_type)
    first.motor_cmd[0].q = radians(2)
    first.motor_cmd[0].tau = 6.0
    stream.receive(first, 1)
    if publish_at(stream, 10_000_000).motor_cmd[0].q != 0.0 or len(worker.jobs) != 1:
        raise RuntimeError("Непроверенная цель попала на выход.")
    worker.finish()
    if publish_at(stream, 20_000_000) is not first:
        raise RuntimeError("Проверенная цель не опубликована.")

    delayed = sample_command(test_message_type)
    delayed.motor_cmd[0].q = radians(4)
    stream.receive(delayed, 30_000_000)
    for now in range(30_000_000, 430_000_000, 10_000_000):
        if publish_at(stream, now) is not first or len(worker.jobs) != 1:
            raise RuntimeError("Длительная проверка заблокировала или изменила публикацию.")
    if not verify_hold(publish_at(stream, 430_000_000), first):
        raise RuntimeError("Длительная проверка заблокировала удержание.")

    recovery = sample_command(test_message_type)
    recovery.motor_cmd[0].q = radians(1)
    stream.receive(recovery, 440_000_000)
    worker.finish()
    if not verify_hold(publish_at(stream, 450_000_000), first) or len(worker.jobs) != 1:
        raise RuntimeError("Устаревший результат проверки попал на выход.")
    worker.finish()
    if publish_at(stream, 460_000_000) is not recovery:
        raise RuntimeError("После задержки не принята новая проверенная цель.")
    print(
        "Фоновая проверка: таймер продолжает публикацию и удержание при задержке MuJoCo.",
        flush=True,
    )


class SimulatorClockGap(RuntimeError):
    """Самый длинный разрыв выхода пришёлся на разрыв входного /clock."""


def spin_until(executor, condition, timeout, error):
    deadline = monotonic() + timeout
    while not condition():
        if monotonic() >= deadline:
            raise RuntimeError(error)
        executor.spin_once(timeout_sec=0.01)


def test_ros_exchange(executor, probe, message_type):
    from rclpy.serialization import deserialize_message, serialize_message
    from rosgraph_msgs.msg import Clock as ClockMessage

    def now_ns():
        return clock_stamps[-1] if clock_stamps else 0

    received = []
    clock_stamps = []
    sender = probe.create_publisher(message_type, "/lowcmd_bare", 10)

    def on_output(message):
        received.append((now_ns(), message))

    def on_clock(message):
        clock_stamps.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)

    probe.create_subscription(message_type, "/lowcmd", on_output, 100)
    probe.create_subscription(ClockMessage, "/clock", on_clock, 100)

    spin_until(
        executor,
        lambda: now_ns() > 0 and sender.get_subscription_count() > 0 and bool(received),
        15.0,
        "Не получены /clock или начальный поток команд.",
    )

    active = sample_command(message_type)
    active.motor_cmd[0].q = radians(3.0)
    active.motor_cmd[0].dq = 0.5
    active.motor_cmd[0].tau = 6.25
    active.motor_cmd[11].kp = 1001.0
    active.motor_cmd[11].q += radians(1.0)
    active.crc = 101
    normalized = deserialize_message(serialize_message(active), message_type)
    set_hold_fields(normalized.motor_cmd[11])
    sender.publish(active)
    spin_until(
        executor,
        lambda: received and received[-1][1].crc == active.crc,
        5.0,
        "Допустимая подвижная цель не появилась на выходе.",
    )
    if received[-1][1] != normalized:
        raise RuntimeError("После коррекции привода выход отличается от ожидаемой команды.")

    bad = sample_command(message_type)
    bad.motor_cmd[0].mode = 0
    bad.crc = 102
    received.clear()
    last_sent = now_ns()
    sender.publish(bad)
    spin_until(
        executor,
        lambda: received and verify_hold(received[-1][1], normalized),
        8.0,
        "После исчезновения входа не появилось удержание принятой цели.",
    )

    delay_ms = (received[-1][0] - last_sent) / 1_000_000
    if not 0 <= delay_ms <= 500.0:
        raise RuntimeError(f"Удержание началось через {delay_ms:.1f} мс, предел — 500 мс.")
    if len(received) < 2 or any(cmd != normalized for _, cmd in received[:-1]):
        raise RuntimeError("Недопустимая команда изменила поток до удержания.")
    if any(cmd.crc == bad.crc for _, cmd in received):
        raise RuntimeError("Недопустимая команда попала на выход.")
    print(
        f"OK — удержание через {delay_ms:.1f} мс. Цель q сохранена, dq и tau равны нулю.",
        flush=True,
    )

    received.clear()
    invalid_number = sample_command(message_type)
    invalid_number.motor_cmd[19].tau = float("nan")
    invalid_number.crc = 103
    sender.publish(invalid_number)
    pause_end = now_ns() + 150_000_000
    spin_until(executor, lambda: now_ns() >= pause_end, 5.0, "Время /clock остановилось.")
    if not received or any(not verify_hold(cmd, normalized) for _, cmd in received):
        raise RuntimeError("Команда с NaN прервала удержание.")
    print("OK — сообщение с NaN в неиспользуемом слоте отклонено.", flush=True)

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
    gap_start, gap_end = max(pairwise(stamps), key=lambda pair: pair[1] - pair[0])
    max_gap_ns = gap_end - gap_start
    if any(command_error(cmd) or cmd.crc in (bad.crc, invalid_number.crc) for _, cmd in records):
        raise RuntimeError("Выходной поток содержит недопустимую команду.")
    published_targets = {
        round(degrees(cmd.motor_cmd[0].q), 1) for _, cmd in records if cmd.crc >= 1000
    }
    if published_targets != {0.0, 4.8}:
        raise RuntimeError("После удержания узел не отработал обе новые цели.")
    summary = f"{frequency:.1f} Гц, максимальный интервал {max_gap_ns / 1_000_000:.1f} мс"
    if max_gap_ns > MAX_OUTPUT_GAP_NS:
        clock_jump_ns = max(
            min(end, gap_end) - max(start, gap_start) for start, end in pairwise(clock_stamps)
        )
        if clock_jump_ns > MAX_OUTPUT_GAP_NS:
            raise SimulatorClockGap(
                f"{summary}. Разрыв /clock: {clock_jump_ns / 1_000_000:.1f} мс."
            )
    if frequency < 50 or max_gap_ns > MAX_OUTPUT_GAP_NS:
        raise RuntimeError(f"{summary}. Требуется ≥50 Гц и ≤100 мс за 5 секунд симуляции.")
    print(f"OK — 5 секунд симуляции: {summary}.", flush=True)


def test_clock_publication(executor, probe, message_type, output_topic):
    from rosgraph_msgs.msg import Clock as ClockMessage

    received = []
    clock_pub = probe.create_publisher(ClockMessage, "/clock", 10)
    probe.create_subscription(message_type, output_topic, received.append, 10)

    def set_sim_time(milliseconds):
        stamp = ClockMessage()
        stamp.clock.sec, remainder = divmod(milliseconds, 1000)
        stamp.clock.nanosec = remainder * 1_000_000
        clock_pub.publish(stamp)

    spin_until(
        executor,
        lambda: clock_pub.get_subscription_count() > 0 and probe.count_publishers(output_topic) > 0,
        10.0,
        "Не обнаружены узел Safety Block или подписка на /clock.",
    )
    set_sim_time(1000)
    spin_until(
        executor, lambda: bool(received), 3.0, "Публикация не началась после первого /clock."
    )
    pause_until = monotonic() + 0.15
    while monotonic() < pause_until:
        executor.spin_once(timeout_sec=0.01)
    if len(received) != 1:
        raise RuntimeError("Публикация продолжается при остановленных часах /clock.")

    set_sim_time(1000)
    set_sim_time(1003)
    duplicate_until = monotonic() + 0.05
    while monotonic() < duplicate_until:
        executor.spin_once(timeout_sec=0.01)
    if len(received) != 1:
        raise RuntimeError("Лишняя публикация в уже обработанном интервале /clock.")

    set_sim_time(1020)
    spin_until(
        executor,
        lambda: len(received) >= 2,
        3.0,
        "Публикация не возобновилась при продвижении /clock.",
    )
    print("OK — выход следует за /clock: пауза, повтор и возобновление проверены.", flush=True)


def run_ros_test(clock_test):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from unitree_go.msg import LowCmd

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        probe = rclpy.create_node("t2_clock_probe" if clock_test else "t2_participant_probe")
        nodes.append(probe)
        executor.add_node(probe)
        if not clock_test:
            test_ros_exchange(executor, probe, LowCmd)
            return

        output_topic = "/t2_test/lowcmd"
        remapping = [
            "--ros-args",
            "-r",
            "/lowcmd_bare:=/t2_test/lowcmd_bare",
            "-r",
            f"/lowcmd:={output_topic}",
        ]
        with ThreadPoolExecutor(max_workers=1) as worker:
            safety_block = create_safety_block(worker, cli_args=remapping)
            nodes.append(safety_block)
            executor.add_node(safety_block)
            test_clock_publication(executor, probe, LowCmd, output_topic)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_check(option):
    if option == "--self-test":
        self_test()
    elif option == "--collision-test":
        guard = load_collision_guard()
        collision_test(guard)
        stream_logic_test(guard)
        background_logic_test(guard)
    else:
        try:
            run_ros_test(clock_test=option == "--clock-test")
        except SimulatorClockGap as error:
            print(error, file=sys.stderr)
            raise SystemExit(os.EX_TEMPFAIL) from None
