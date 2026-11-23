"""CPU scan-to-keyframe odometry. No ROS, commands, map or ground truth."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def rz(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def valid_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
        return None
    return q / np.linalg.norm(q)


class Attitudes:
    def __init__(self):
        self.samples = []
        self.origin_yaw = None

    def add(self, stamp, quaternion):
        q = valid_quaternion(quaternion)
        if q is None or not math.isfinite(stamp):
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        if self.origin_yaw is None:
            self.origin_yaw = Rotation.from_quat(q).as_euler('xyz')[2]
        rot = Rotation.from_euler('z', -self.origin_yaw) * Rotation.from_quat(q)
        self.samples.append((stamp, rot.as_quat()))
        self.samples = self.samples[-600:]
        return True

    def at(self, stamp):
        if not self.samples or stamp < self.samples[0][0] or stamp > self.samples[-1][0]:
            return None
        times = np.array([s[0] for s in self.samples])
        j = int(np.searchsorted(times, stamp))
        if j == 0 or times[j] == stamp:
            return Rotation.from_quat(self.samples[j][1])
        if times[j]-times[j-1] > .1:
            return None
        return Slerp(times[j-1:j+1], Rotation.from_quat([self.samples[j-1][1], self.samples[j][1]]))([stamp])[0]


def project(ranges, angle_min, increment, lo, hi, attitude, offset):
    """Level rays, including trunk->laser lever arm. Preserve scan indices."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(ranges))*increment
    valid = np.isfinite(ranges) & (ranges > max(lo, .25)) & (ranges < min(hi-.02, 15.))
    indices = np.flatnonzero(valid)
    points = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum())))
    yaw = attitude.as_euler('xyz')[2]
    leveled = (Rotation.from_euler('z', -yaw)*attitude).apply(points+np.asarray(offset))
    return leveled[:, :2], indices, float(yaw)


@dataclass
class Result:
    accepted: bool
    reason: str
    rmse: float = math.inf
    support: int = 0


class Estimator:
    def __init__(self):
        self.position = np.zeros(2)
        self.reference = None
        self.last_stamp = None
        self.last_yaw = None
        self.velocity = np.zeros(2)

    def keyframe(self, points, indices, yaw):
        # Normals use adjacent original beams only, not a jump across missing rays.
        tangent = points[2:] - points[:-2]
        span = np.linalg.norm(tangent, axis=1)
        valid = (indices[2:]-indices[:-2] == 2) & (span > .015) & (span < .55)
        if valid.sum() < 40:
            return False
        self.reference = points[1:-1][valid].copy()
        self.normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))[valid] / span[valid, None]
        self.tree = cKDTree(self.reference)
        self.anchor = self.position.copy()
        self.anchor_yaw = yaw
        self.translation = np.zeros(2)
        return True

    def update(self, points, indices, yaw, stamp):
        if len(points) < 60 or not np.isfinite(points).all() or not np.isfinite([yaw, stamp]).all():
            return Result(False, 'invalid_scan')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return Result(False, 'nonmonotonic_scan')
        if self.reference is None:
            if not self.keyframe(points, indices, yaw):
                return Result(False, 'insufficient_geometry')
            self.last_stamp, self.last_yaw = stamp, yaw
            return Result(True, 'initialized', 0., len(self.reference))
        dt = stamp-self.last_stamp
        rotated = points @ rz(wrap(yaw-self.anchor_yaw)).T
        t = self.translation.copy()
        # Reacquire only against the existing reference, never invent motion.
        gate = min(.6, .12 + dt*.6)
        rmse, count = math.inf, 0
        for _ in range(15):
            moved = rotated+t
            dist, ids = self.tree.query(moved)
            good = dist < gate
            n = self.normals[ids[good]]
            residual = np.sum(n*(self.reference[ids[good]]-moved[good]), axis=1)
            if len(residual) < 40:
                return Result(False, 'low_overlap')
            keep = np.abs(residual) < max(.025, min(.15, np.quantile(np.abs(residual), .85)))
            n, residual = n[keep], residual[keep]
            count = len(residual)
            if count < max(35, int(.18*len(points))):
                return Result(False, 'low_support')
            weight = np.minimum(1., .025/np.maximum(np.abs(residual), 1e-9))
            h = n.T @ (weight[:, None]*n)
            if np.linalg.cond(h) > 80:
                return Result(False, 'degenerate_geometry')
            step = np.linalg.solve(h, n.T@(weight*residual))
            t += step
            rmse = float(np.sqrt(np.mean(residual**2)))
            if np.linalg.norm(step) < 1e-4:
                break
        candidate = self.anchor + rz(self.anchor_yaw)@t
        if rmse > .06 or np.linalg.norm(candidate-self.position) > min(1.0, .03+.8*dt):
            return Result(False, 'innovation_rejected', rmse, count)
        self.velocity = (candidate-self.position)/dt
        self.position = candidate
        self.translation = t
        self.last_stamp, self.last_yaw = stamp, yaw
        # Fixed keyframes while stationary avoid integrating frame-to-frame noise.
        if np.linalg.norm(t) > .20 or abs(wrap(yaw-self.anchor_yaw)) > .25:
            self.keyframe(points, indices, yaw)
        return Result(True, 'tracking', rmse, count)
