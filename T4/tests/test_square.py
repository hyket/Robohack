"""Deterministic scan replay in a rectangular room, independent of ROS."""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "go1_t4"))
from go1_t4.estimator import ScanTracker, project_scan


def test_square():
    rng = np.random.default_rng(42)
    tracker = ScanTracker()
    angles = np.linspace(-np.pi, np.pi, 720)
    xy = np.zeros(2)
    yaw = 0.0

    def scan():
        mount = xy + 0.22 * np.array([np.cos(yaw), np.sin(yaw)])
        rays = np.array([np.cos(angles + yaw), np.sin(angles + yaw)])
        walls = np.where(rays > 0, np.array([[4], [3]]), np.array([[-3], [-2]]))
        with np.errstate(divide='ignore'):
            ranges = ((walls - mount[:, None]) / rays).min(axis=0)
        ranges += rng.normal(0, 0.01, len(angles))
        q = Rotation.from_euler('z', yaw).as_quat()
        tracker.update(project_scan(ranges, angles[0], angles[1] - angles[0], 0.12, 20, q))

    scan()
    errors = []
    for _ in range(4):
        start = tracker.xy.copy()
        direction = np.array([np.cos(yaw), np.sin(yaw)])
        for _ in range(100):
            xy[:] += 0.01 * direction
            scan()
        errors.append(np.linalg.norm(tracker.xy - start - direction))
        for _ in range(105):
            yaw += np.pi / 210
            scan()
    stop = tracker.xy.copy()
    for _ in range(200):
        scan()
    drift = np.linalg.norm(tracker.xy - stop)
    print(f'Mean 1 m leg error: {np.mean(errors):.4f} m; stationary drift: {drift:.4f} m')
    assert max(errors) < 0.05
    assert drift < 0.02


if __name__ == '__main__':
    test_square()
