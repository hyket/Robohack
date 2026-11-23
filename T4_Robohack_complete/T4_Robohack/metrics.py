"""Offline metrics: CSV columns t,x,y,z,qx,qy,qz,qw; no ROS dependency."""
import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load(path):
    rows = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    if len(rows) < 2 or rows.shape[1] != 8 or not np.isfinite(rows).all():
        raise ValueError(f'{path}: insufficient or nonfinite poses')
    if np.any(np.diff(rows[:,0]) <= 0):
        raise ValueError(f'{path}: timestamps must be strictly increasing')
    if not np.allclose(np.linalg.norm(rows[:,4:], axis=1), 1., atol=.001):
        raise ValueError(f'{path}: invalid quaternions')
    return rows


def interpolate(rows, t):
    p = np.column_stack([np.interp(t, rows[:,0], rows[:,j]) for j in (1,2,3)])
    r = Slerp(rows[:,0], Rotation.from_quat(rows[:,4:]))(t)
    return p, r


def evaluate(truth, estimate, stationary_start=None):
    overlap = truth[(truth[:,0] >= estimate[0,0]) & (truth[:,0] <= estimate[-1,0])]
    if len(overlap) < 2:
        raise ValueError('No overlapping trajectory')
    p, r = interpolate(estimate, overlap[:,0])
    g = overlap[:,1:4]
    gr = Rotation.from_quat(overlap[:,4:])
    distance = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(g,axis=0),axis=1))]
    start, errors = 0, []
    for end in range(1,len(g)):
        length = distance[end]-distance[start]
        if length >= 1.:
            estimated_delta = r[start].inv().apply(p[end]-p[start])
            true_delta = gr[start].inv().apply(g[end]-g[start])
            errors.append(float(np.linalg.norm(estimated_delta-true_delta)/length))
            start = end
    out = {'segments':len(errors), 'mean_rpe':float(np.mean(errors)) if errors else None,
           'segment_rpe':errors, 'odom_mean_hz':float((len(estimate)-1)/(estimate[-1,0]-estimate[0,0])),
           'odom_max_gap_s':float(np.max(np.diff(estimate[:,0])))}
    if stationary_start is not None:
        t = np.array([stationary_start,stationary_start+10.])
        if t[0] < estimate[0,0] or t[-1] > estimate[-1,0]:
            raise ValueError('The complete 10-second drift window is not recorded')
        pp, rr = interpolate(estimate,t)
        out['drift_m'] = float(np.linalg.norm(pp[1]-pp[0]))
        out['drift_deg'] = float(np.degrees((rr[0].inv()*rr[1]).magnitude()))
        out['drift_pass'] = out['drift_m'] <= .10 and out['drift_deg'] <= 5.
        if t[0] >= truth[0,0] and t[-1] <= truth[-1,0]:
            gp, gg = interpolate(truth,t)
            out['actual_end_displacement_m'] = float(np.linalg.norm(gp[1]-gp[0]))
            out['actual_end_rotation_deg'] = float(np.degrees((gg[0].inv()*gg[1]).magnitude()))
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('truth'); parser.add_argument('estimate')
    parser.add_argument('--stationary-start',type=float)
    args = parser.parse_args()
    print(json.dumps(evaluate(load(args.truth),load(args.estimate),args.stationary_start), indent=2))
