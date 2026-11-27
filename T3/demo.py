#!/usr/bin/env python3
"""Evaluate a T3 episode on the public simulator and render its recorded poses."""

import argparse
import bisect
import json
import math
import subprocess
import sys
import time
from pathlib import Path


WARMUP_S = 1.5
DISTANCE_M = 3.0
MAX_DURATION_S = 90.0
FALL_TIME_S = 0.5
MAX_TILT_RAD = math.radians(50)
JOINT_NAMES = [
    f'{leg}_{joint}_joint'
    for leg in ('FR', 'FL', 'RR', 'RL')
    for joint in ('hip', 'thigh', 'calf')
]


def euler_angles(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Episode:
    def __init__(self, speed, duration):
        self.speed = speed
        self.duration = duration
        self.frames = []
        self.origin = None
        self.heading = None
        self.violation_since = {}
        self.commands = {'/lowcmd_bare': [], '/lowcmd': []}
        self.distance = 0.0
        self.result = None

    def record(self, stamp, pose, joints):
        if self.frames and stamp <= self.frames[-1][0]:
            return
        roll, pitch, yaw = euler_angles(*pose[3:])
        if not self.frames:
            self.heading = (math.cos(yaw), math.sin(yaw))
        elapsed = stamp - self.frames[0][0] if self.frames else 0.0
        scored_time = elapsed - WARMUP_S
        if self.origin is None and scored_time >= 0:
            last = self.frames[-1]
            fraction = (stamp - scored_time - last[0]) / (stamp - last[0])
            self.origin = tuple(last[i] + (pose[i - 1] - last[i]) * fraction for i in (1, 2))
        if self.origin is not None:
            self.distance = sum((current - origin) * heading for current, origin, heading in zip(
                pose[:2], self.origin, self.heading,
            ))
        self.frames.append([stamp, *pose, *joints])

        for name, violated in (
            ('height', pose[2] < 0.16),
            ('roll', abs(roll) > MAX_TILT_RAD),
            ('pitch', abs(pitch) > MAX_TILT_RAD),
        ):
            if violated:
                start = self.violation_since.setdefault(name, stamp)
                if stamp - start > FALL_TIME_S:
                    self.finish('fall', scored_time, name)
                    return
            else:
                self.violation_since.pop(name, None)

        if scored_time <= self.duration and self.distance >= DISTANCE_M:
            self.finish('success', scored_time)
        elif scored_time >= self.duration:
            self.finish('timeout', scored_time)

    def finish(self, status, scored_time, fall_condition=None):
        elapsed = self.frames[-1][0] - self.frames[0][0]
        command_stats = {}
        for topic, stamps in self.commands.items():
            command_stats[topic] = {
                'messages': len(stamps),
                'mean_hz': len(stamps) / elapsed,
                'max_gap_s': max((b - a for a, b in zip(stamps, stamps[1:])), default=None),
            }
        self.result = {
            'status': status,
            'speed_mps': self.speed,
            'forward_m': self.distance,
            'elapsed_sim_s': elapsed,
            'scored_sim_s': max(0.0, scored_time),
            'fall_condition': fall_condition,
            'commands': command_stats,
        }


def capture(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from unitree_go.msg import LowCmd, LowState

    episode = Episode(args.speed, args.duration)
    rclpy.init()
    node = rclpy.create_node('t3_demo_observer')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    command = Twist()
    command.linear.x = args.speed
    telemetry_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
    )
    latest_state = None
    clock = None
    next_command = 0.0
    armed = False
    last_received = time.monotonic()

    def on_state(message):
        nonlocal latest_state
        latest_state = message

    def on_clock(message):
        nonlocal clock
        clock = message.clock.sec + message.clock.nanosec * 1e-9

    def on_lowcmd(topic):
        if episode.frames and clock is not None and episode.result is None:
            episode.commands[topic].append(clock)

    def on_odom(message):
        nonlocal last_received, next_command
        if not armed or latest_state is None or clock is None or episode.result is not None:
            return
        header = message.header.stamp
        stamp = header.sec + header.nanosec * 1e-9
        if episode.frames and stamp <= episode.frames[-1][0]:
            return
        pose = message.pose.pose
        p, q = pose.position, pose.orientation
        joints = [motor.q for motor in latest_state.motor_state[:12]]
        if stamp >= next_command:
            publisher.publish(command)
            next_command = stamp + 0.05
        episode.record(stamp, (p.x, p.y, p.z, q.w, q.x, q.y, q.z), joints)
        last_received = time.monotonic()

    node.create_subscription(LowState, '/lowstate', on_state, telemetry_qos)
    node.create_subscription(Clock, '/clock', on_clock, telemetry_qos)
    node.create_subscription(Odometry, '/odom/ground_truth', on_odom, telemetry_qos)
    for topic in episode.commands:
        node.create_subscription(LowCmd, topic, lambda _, topic=topic: on_lowcmd(topic), telemetry_qos)

    try:
        while episode.result is None:
            if not rclpy.ok():
                raise RuntimeError('ROS shutdown interrupted capture')
            if time.monotonic() - last_received > args.wait:
                raise RuntimeError('timed out waiting for simulator and participant telemetry')
            if not armed:
                armed = (node.count_subscribers('/cmd_vel') > 0 and
                         node.count_publishers('/lowcmd') > 0)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok() and armed:
            publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()

    print(json.dumps({'result': episode.result, 'frames': episode.frames}, separators=(',', ':')))
    print(json.dumps(episode.result, indent=2), file=sys.stderr)


def capture_episode(args):
    command = [
        sys.executable, str(args.launcher), '--task', 'T3', '--headless',
        '--sim-image', args.sim_image, '--participant-image', args.participant_image,
    ]
    launcher = subprocess.Popen(command, stdout=subprocess.PIPE, text=True, start_new_session=True)
    try:
        simulator = None
        for line in launcher.stdout:
            print(line, end='', file=sys.stderr, flush=True)
            if line.startswith('Simulator='):
                simulator = line.strip().split('=', 1)[1]
            if line.startswith('Pair started'):
                break
        else:
            raise RuntimeError('public launcher exited before both containers started')
        if simulator is None:
            raise RuntimeError('public launcher did not report its simulator container')

        with Path(__file__).open('rb') as observer:
            finished = subprocess.run([
                'docker', 'exec', '-i', simulator, 'bash', '-c',
                'source /etc/go1_sim_env.sh && exec python3 - capture "$@"', '_',
                '--speed', str(args.speed), '--duration', str(args.duration), '--wait', str(args.wait),
            ], stdin=observer, stdout=subprocess.PIPE, text=True, check=True)
        return finished.stdout
    finally:
        launcher.terminate()
        launcher.communicate(timeout=30)


def series(args):
    args.output.mkdir(parents=True, exist_ok=True)
    paths = [args.output / f'episode-{index:02d}.json' for index in range(1, args.count + 1)]
    if any(path.exists() for path in paths):
        raise FileExistsError('episode trace already exists in the selected output directory')
    passed = 0
    for path in paths:
        trace = capture_episode(args)
        result = json.loads(trace)['result']
        with path.open('x') as episode_file:
            episode_file.write(trace)
        passed += result['status'] == 'success'
    print(f'{passed}/{len(paths)} successful episodes')


def render(args):
    import mujoco
    import numpy as np

    frames = np.array(json.loads(args.trace.read_text())['frames'])
    stamps = frames[:, 0]
    origin = stamps[0]
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    model.vis.global_.offwidth = args.width
    model.vis.global_.offheight = args.height
    data = mujoco.MjData(model)
    addresses = [model.joint(name).qposadr[0] for name in JOINT_NAMES]
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = -112
    camera.elevation = -19
    camera.distance = 1.25

    with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
        duration = stamps[-1] - origin
        for frame in range(math.ceil(duration * args.fps) + 1):
            stamp = min(origin + frame / args.fps, stamps[-1])
            right = min(max(bisect.bisect_left(stamps, stamp), 1), len(stamps) - 1)
            left = right - 1
            fraction = (stamp - stamps[left]) / (stamps[right] - stamps[left])
            pose = frames[left, 1:] + (frames[right, 1:] - frames[left, 1:]) * fraction
            pose[3:7] /= np.linalg.norm(pose[3:7])
            data.qpos[:7] = pose[:7]
            data.qpos[addresses] = pose[7:]
            data.time = stamp
            mujoco.mj_forward(model, data)
            camera.lookat[:] = (data.qpos[0] + 0.12, data.qpos[1], 0.20)
            renderer.update_scene(data, camera=camera)
            sys.stdout.buffer.write(renderer.render().tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest='command', required=True)
    episode_options = argparse.ArgumentParser(add_help=False)
    episode_options.add_argument('--speed', type=float, required=True, help='forward command in m/s')
    episode_options.add_argument('--duration', type=float, default=MAX_DURATION_S, help='scored time limit in seconds')
    episode_options.add_argument('--wait', type=float, default=30.0, help='wall timeout waiting for telemetry')
    subcommands.add_parser('capture', parents=[episode_options], help='drive the episode and write JSON telemetry to stdout')
    runner = subcommands.add_parser('series', parents=[episode_options], help='run fresh public Simulator and Participant pairs')
    runner.add_argument('launcher', type=Path, help='path to the public simulator run_local.py')
    runner.add_argument('--sim-image', required=True)
    runner.add_argument('--participant-image', required=True)
    runner.add_argument('--output', type=Path, required=True, help='directory for episode JSON traces')
    runner.add_argument('--count', type=int, default=1)
    renderer = subcommands.add_parser('render', help='write raw rgb24 video frames to stdout')
    renderer.add_argument('trace', type=Path)
    renderer.add_argument('scene', type=Path, help='scene.xml used by the public simulator')
    renderer.add_argument('--width', type=int, default=960)
    renderer.add_argument('--height', type=int, default=540)
    renderer.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()
    if args.command in ('capture', 'series'):
        if not (0.1 <= args.speed <= 0.6 and 0 < args.duration <= MAX_DURATION_S and 0 < args.wait < math.inf):
            parser.error('episodes require speed 0.1..0.6 m/s, duration 0..90 s and a positive finite wait')
        if args.command == 'capture':
            capture(args)
        else:
            if args.count < 1:
                parser.error('series count must be positive')
            series(args)
    else:
        if min(args.width, args.height, args.fps) <= 0:
            parser.error('render dimensions and frame rate must be positive')
        render(args)


if __name__ == '__main__':
    main()
