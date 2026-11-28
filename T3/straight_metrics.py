#!/usr/bin/env python3
"""Compare straight walking from episode traces recorded by demo.py series."""

import argparse
import json
import math
from pathlib import Path
from statistics import median

if __package__:
    from .demo import DISTANCE_M, MAX_DURATION_S, WARMUP_S, euler_angles
else:
    from demo import DISTANCE_M, MAX_DURATION_S, WARMUP_S, euler_angles


def episode_metrics(trace):
    """Measure poses from the scoring origin to the first sampled finish."""
    result = trace['result']
    status = result['status']
    if not isinstance(status, str):
        raise ValueError('expected a recorded episode status')

    frames = [tuple(map(float, frame[:8])) for frame in trace['frames']]
    if any(len(frame) != 8 or not all(map(math.isfinite, frame)) for frame in frames):
        raise ValueError('expected finite stamp, position and quaternion for each frame')
    if any(right[0] <= left[0] for left, right in zip(frames, frames[1:])):
        raise ValueError('expected strictly increasing frame timestamps')

    metrics = {
        'status': status,
        'success': False,
        'fall_condition': result.get('fall_condition'),
        'forward_m': None,
        'time_to_3m_s': None,
        'max_abs_cross_track_m': None,
        'cross_track_rmse_m': None,
        'end_lateral_m': None,
        'max_heading_deviation_deg': None,
        'heading_mae_deg': None,
    }
    if not frames:
        return {**metrics, 'failure': 'no pose samples'}

    initial = frames[0]
    initial_yaw = euler_angles(*initial[4:8])[2]
    heading_x = math.cos(initial_yaw)
    heading_y = math.sin(initial_yaw)
    origin = None
    max_cross_track = 0.0
    max_heading = 0.0
    scored_lateral_sq_area = 0.0
    scored_abs_heading_area = 0.0
    previous_scored_time = 0.0
    previous_lateral = 0.0
    previous_abs_heading = None
    for previous, frame in zip(frames, frames[1:]):
        stamp, x, y = frame[:3]
        scored_time = stamp - initial[0] - WARMUP_S
        if scored_time < 0.0:
            continue
        if origin is None:
            fraction = (stamp - scored_time - previous[0]) / (stamp - previous[0])
            origin = tuple(previous[i] + (frame[i] - previous[i]) * fraction for i in (1, 2))
        dx, dy = x - origin[0], y - origin[1]
        forward = dx * heading_x + dy * heading_y
        lateral = -dx * heading_y + dy * heading_x  # Positive is left of the starting heading.
        heading_error = math.remainder(euler_angles(*frame[4:8])[2] - initial_yaw, math.tau)
        abs_heading = abs(heading_error)
        if previous_abs_heading is None:
            previous_abs_heading = abs_heading
        interval = scored_time - previous_scored_time
        scored_lateral_sq_area += interval * (previous_lateral ** 2 + lateral ** 2) / 2
        scored_abs_heading_area += interval * (previous_abs_heading + abs_heading) / 2
        previous_scored_time = scored_time
        previous_lateral = lateral
        previous_abs_heading = abs_heading
        max_cross_track = max(max_cross_track, abs(lateral))
        max_heading = max(max_heading, abs_heading)
        metrics.update(
            forward_m=forward,
            max_abs_cross_track_m=max_cross_track,
            cross_track_rmse_m=math.sqrt(scored_lateral_sq_area / scored_time) if scored_time else 0.0,
            end_lateral_m=lateral,
            max_heading_deviation_deg=math.degrees(max_heading),
            heading_mae_deg=math.degrees(scored_abs_heading_area / scored_time if scored_time else abs_heading),
        )
        if scored_time <= MAX_DURATION_S and forward >= DISTANCE_M:
            if status == 'success':
                return {**metrics, 'success': True, 'time_to_3m_s': scored_time}
            break

    if status != 'success':
        failure = status
    elif origin is None:
        failure = 'no pose samples after warmup'
    else:
        failure = 'recorded success without reaching 3 m within the scoring window'
    return {**metrics, 'failure': failure}


def compare_series(directory):
    episodes = []
    for path in sorted(directory.glob('episode-*.json')):
        try:
            metrics = episode_metrics(json.loads(path.read_text()))
        except (OSError, KeyError, TypeError, ValueError) as exc:
            metrics = {'status': 'invalid_trace', 'success': False, 'failure': str(exc)}
        episodes.append({'episode': path.name, **metrics})

    successful = [episode for episode in episodes if episode['success']]
    summary = {
        'successful': len(successful),
        'total': len(episodes),
        'failed_episodes': [
            {key: episode[key] for key in ('episode', 'status', 'failure')}
            for episode in episodes if not episode['success']
        ],
    }
    for metric in ('time_to_3m_s', 'max_abs_cross_track_m', 'cross_track_rmse_m', 'end_lateral_m',
                   'max_heading_deviation_deg', 'heading_mae_deg'):
        values = [episode[metric] for episode in successful]
        summary[metric] = {
            'median': median(values) if values else None,
            'worst': max(values, key=abs) if values else None,
        }
    end_lateral = [abs(episode['end_lateral_m']) for episode in successful]
    summary['abs_end_lateral_m'] = {
        'median': median(end_lateral) if end_lateral else None,
        'worst': max(end_lateral) if end_lateral else None,
    }
    return {'directory': str(directory), 'summary': summary, 'episodes': episodes}


def stability_key(series):
    summary = series['summary']
    return (
        -summary['successful'] / summary['total'],
        summary['max_abs_cross_track_m']['median'],
        summary['max_abs_cross_track_m']['worst'],
        summary['time_to_3m_s']['median'],
        series['directory'],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', metavar='SERIES_DIR', type=Path, nargs='+',
                        help='directory containing episode-*.json traces')
    args = parser.parse_args()
    for directory in args.directories:
        if not directory.is_dir():
            parser.error(f'not a series directory: {directory}')

    series = [compare_series(directory) for directory in args.directories]
    ranked = sorted((item for item in series if item['summary']['successful']), key=stability_key)
    report = {
        'warmup_s': WARMUP_S,
        'distance_m': DISTANCE_M,
        'series': series,
        'stability_ranking': [item['directory'] for item in ranked],
    }
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
