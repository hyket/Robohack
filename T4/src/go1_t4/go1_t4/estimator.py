"""Planar translation from levelled scan endpoints in odom axes."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def project_scan(ranges, angle_min, increment, minimum, maximum, quaternion):
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(ranges)) * increment
    valid = np.isfinite(ranges) & (ranges >= minimum) & (ranges < maximum)
    r = ranges[valid]
    a = angles[valid]
    # Fixed base_scan mount in the organizers' go1.urdf.xacro, in trunk axes.
    xyz = np.column_stack((0.22 + r * np.cos(a), r * np.sin(a), np.full(len(r), 0.11)))
    points = np.full((len(ranges), 2), np.nan)
    points[valid] = Rotation.from_quat(quaternion).apply(xyz)[:, :2]
    return points


class ScanTracker:
    def __init__(self):
        self.xy = np.zeros(2)
        self.tree = None
        self.failures = 0

    def anchor(self, points):
        # Use short contiguous surface segments. Reject corners and depth jumps.
        left = points[1:-1] - points[:-2]
        right = points[2:] - points[1:-1]
        lengths = np.linalg.norm(left, axis=1) * np.linalg.norm(right, axis=1)
        tangent = left + right
        span = np.linalg.norm(tangent, axis=1)
        good = ((lengths > 1e-8) & (span > 0.005) & (span < 0.45)
                & (np.sum(left * right, axis=1) > 0.65 * lengths))
        if good.sum() < 20:
            return
        t = tangent[good] / span[good, None]
        self.normals = np.column_stack((-t[:, 1], t[:, 0]))
        self.tree = cKDTree(points[1:-1][good])
        self.origin = self.xy.copy()
        self.failures = 0

    def update(self, points):
        if self.tree is None:
            self.anchor(points)
            return
        scan = points[np.isfinite(points).all(axis=1)]
        delta = self.xy - self.origin
        for _ in range(8):
            shifted = scan + delta
            distance, index = self.tree.query(shifted)
            keep = distance < 0.30
            if keep.sum() < max(20, len(scan) // 4):
                break
            normals = self.normals[index[keep]]
            diff = self.tree.data[index[keep]] - shifted[keep]
            residual = np.sum(normals * diff, axis=1)
            weights = np.sqrt(0.025 / np.maximum(0.025, np.abs(residual)))
            step, _, _, singular = np.linalg.lstsq(
                normals * weights[:, None], residual * weights, rcond=None)
            if singular[-1] < 0.08 * singular[0]:
                break
            delta = delta + step
            if np.linalg.norm(step) < 0.001:
                if np.median(np.abs(residual)) > 0.05:
                    break
                self.xy = self.origin + delta
                self.failures = 0
                if np.linalg.norm(delta) > 0.20:
                    self.anchor(points)
                return
        self.failures += 1
        if self.failures >= 5:
            self.anchor(points)
