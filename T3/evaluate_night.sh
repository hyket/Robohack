#!/usr/bin/env bash
set -euo pipefail

t3_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 - "$t3_dir" "$@" <<'PY'
import argparse
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import uuid
from pathlib import Path


def discover_runs(roots):
    runs = set()
    for root in roots:
        for directory, children, files in os.walk(root):
            if 'experiment.txt' in files:
                runs.add(Path(directory))
                children.clear()
    return sorted(runs)


def run_logged(command, log):
    with log.open('w') as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)


def score_series(directory, count, measure_cross_track):
    if measure_cross_track:
        from straight_metrics import episode_metrics

    statuses = {'success': 0, 'fall': 0, 'timeout': 0}
    times = []
    straightness = []
    for index in range(1, count + 1):
        trace = json.loads((directory / f'episode-{index:02d}.json').read_text())
        result = trace['result']
        status = result['status']
        statuses[status] += 1
        if status == 'success':
            scored_time = result['scored_sim_s']
            if not math.isfinite(scored_time) or not 0 <= scored_time <= 90:
                raise ValueError(f'invalid scored time for episode {index}: {scored_time}')
            times.append(scored_time)
            if measure_cross_track:
                metrics = episode_metrics(trace)
                if not metrics['success']:
                    raise ValueError(f'invalid winning trajectory for episode {index}: {metrics["failure"]}')
                straightness.append(metrics)
    scores = {
        'successes': statuses['success'],
        'falls': statuses['fall'],
        'timeouts': statuses['timeout'],
        'median_success_time_s': statistics.median(times) if times else None,
    }
    if measure_cross_track:
        cross_tracks = [metrics['max_abs_cross_track_m'] for metrics in straightness]
        scores['median_peak_cross_track_m'] = statistics.median(cross_tracks) if cross_tracks else None
        scores['worst_peak_cross_track_m'] = max(cross_tracks) if cross_tracks else None
        for metric in ('cross_track_rmse_m', 'heading_mae_deg'):
            values = [metrics[metric] for metrics in straightness]
            scores[f'median_{metric}'] = statistics.median(values) if values else None
    return scores


def ranking_key(candidate, primary):
    time_s = candidate['median_success_time_s']
    primary_value = candidate[primary]
    return (
        -candidate['successes'],
        math.inf if primary_value is None else primary_value,
        math.inf if time_s is None else time_s,
        candidate['run_dir'] or '',
    )


def save_summary(output, summary):
    primary = ('median_peak_cross_track_m' if summary['rank_by'] == 'cross-track'
               else 'median_success_time_s')
    ranked = sorted(
        (candidate for candidate in summary['candidates'] if candidate['status'] == 'evaluated'),
        key=lambda candidate: ranking_key(candidate, primary),
    )
    summary['ranking'] = [candidate['evidence_dir'] for candidate in ranked]
    time_limit = summary['max_median_time_s']
    min_successes = summary['min_successes']
    eligible = [candidate for candidate in ranked if candidate['run_dir'] and candidate['successes'] >= min_successes
                and (time_limit is None or candidate['median_success_time_s'] <= time_limit)]
    summary['best_trained'] = next(iter(eligible), None)
    if summary['rank_by'] == 'cross-track':
        baseline = next((candidate for candidate in ranked if not candidate['run_dir']), None)
        if baseline and baseline['successes']:
            summary['best_trained_beating_baseline'] = next(
                (candidate for candidate in eligible if candidate['successes'] >= baseline['successes']
                 and candidate['median_peak_cross_track_m'] < baseline['median_peak_cross_track_m']
                 and candidate['worst_peak_cross_track_m'] <= baseline['worst_peak_cross_track_m']),
                None,
            )
    temporary = output / '.summary.json.tmp'
    temporary.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    temporary.replace(output / 'summary.json')
    return ranked


def main():
    t3_dir = Path(sys.argv[1])
    sys.path.insert(0, str(t3_dir))
    parser = argparse.ArgumentParser(
        prog='bash T3/evaluate_night.sh',
        description='Compare completed T3 training runs on the same public MuJoCo setup.',
    )
    parser.add_argument('roots', metavar='RUN_OR_PARENT', type=Path, nargs='+')
    parser.add_argument('--launcher', type=Path, default=t3_dir /
                        'go1-simulator-public-3473f086fd8a111592b4b9d822500c79f20c811a/run_local.py')
    parser.add_argument('--sim-image', default='robohack-t2-sim-source:local', help='local Simulator image')
    parser.add_argument('--speed', type=float, default=0.6, help='same forward command for every candidate (default 0.6)')
    parser.add_argument('--count', type=int, default=10, help='episodes per candidate (default 10)')
    parser.add_argument('--output', type=Path, help='new output directory for logs, traces and summary.json')
    parser.add_argument('--baseline-image', help='also evaluate an already built Participant image')
    parser.add_argument('--rank-by', choices=('time', 'cross-track'), default='time',
                        help='tie-break successful counts by time or peak lateral deviation')
    parser.add_argument('--max-median-time', type=float, metavar='SECONDS',
                        help='median time limit for the selected trained run')
    parser.add_argument('--min-successes', type=int, default=1,
                        help='required successes for the selected trained run (default 1)')
    args = parser.parse_args(sys.argv[2:])
    if not 0.1 <= args.speed <= 0.6 or args.count < 1:
        parser.error('speed must be 0.1..0.6 m/s and episode count must be positive')
    if args.max_median_time is not None and not 0 < args.max_median_time <= 90:
        parser.error('median time limit must be in (0, 90] seconds')
    if not 1 <= args.min_successes <= args.count:
        parser.error('minimum successes must be between 1 and the episode count')

    launcher = args.launcher.resolve()
    if not launcher.is_file():
        parser.error(f'launcher not found: {launcher}. Pass --launcher /path/to/run_local.py')
    roots = [path.resolve() for path in args.roots]
    for root in roots:
        if not root.is_dir():
            parser.error(f'training directory not found: {root}')
    runs = discover_runs(roots)
    if not runs:
        parser.error('no training runs found in the specified directories')

    if args.output is None:
        parent = t3_dir.parent / 'output' / 't3-training'
        parent.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix='evaluation-', dir=parent))
    else:
        output = args.output.resolve()
        try:
            output.mkdir(parents=True)
        except FileExistsError:
            parser.error(f'output directory already exists: {output}')

    print(f'Evaluation output: {output}', flush=True)
    summary = {'launcher': str(launcher), 'sim_image': args.sim_image,
               'speed_mps': args.speed, 'episodes_per_candidate': args.count,
               'rank_by': args.rank_by, 'max_median_time_s': args.max_median_time,
               'min_successes': args.min_successes,
               'input_dirs': [str(root) for root in roots], 'candidates': []}
    candidates = [(run, None) for run in runs]
    if args.baseline_image:
        candidates.append((None, args.baseline_image))
    tag = uuid.uuid4().hex[:12]
    for index, (run, baseline_image) in enumerate(candidates, start=1):
        directory = output / f'candidate-{index:02d}'
        directory.mkdir()
        image = baseline_image or f'go1-participant:t3-eval-{tag}-{index}'
        candidate = {'run_dir': str(run) if run else None, 'image': image,
                     'evidence_dir': directory.name}
        print(f'[{index}/{len(candidates)}] {run or image}', flush=True)
        stage = 'build'
        try:
            if run:
                validation = run / 'validation-summary.json'
                if validation.is_file() and json.loads(validation.read_text())['selected_iteration'] == 0:
                    candidate['status'] = 'initial_checkpoint_retained'
                    print('  Isaac validation retained the starting checkpoint.', flush=True)
                    summary['candidates'].append(candidate)
                    ranked = save_summary(output, summary)
                    continue
                manifest = run / 'trained-checkpoints.sha256'
                if not manifest.is_file() or not manifest.stat().st_size:
                    raise ValueError('missing or empty trained-checkpoints.sha256')
                run_logged(['bash', str(t3_dir / 'build_trained.sh'), str(run), image], directory / 'build.log')
                shutil.copyfile(manifest, directory / manifest.name)
            stage = 'demo'
            traces = directory / 'episodes'
            run_logged([sys.executable, str(t3_dir / 'demo.py'), 'series', str(launcher),
                        '--sim-image', args.sim_image, '--participant-image', image,
                        '--speed', str(args.speed), '--count', str(args.count),
                        '--output', str(traces)], directory / 'demo.log')
            candidate.update(score_series(traces, args.count, args.rank_by == 'cross-track'), status='evaluated')
        except (OSError, subprocess.CalledProcessError, ValueError, KeyError, TypeError) as error:
            candidate.update(status=f'{stage}_failed', error=str(error))
            print(f'  {stage} failed. See {directory} and summary.json.', file=sys.stderr, flush=True)
        summary['candidates'].append(candidate)
        ranked = save_summary(output, summary)

    for rank, candidate in enumerate(ranked, start=1):
        median = candidate['median_success_time_s']
        time_text = f'{median:.3f} s median' if median is not None else 'no successful time'
        cross_track = candidate.get('median_peak_cross_track_m')
        if cross_track is not None:
            time_text += f', {cross_track:.3f} m median peak deviation'
        print(f"{rank}. {candidate['successes']}/{args.count} successes, {candidate['falls']} falls, "
              f"{candidate['timeouts']} timeouts, {time_text}  {candidate['run_dir'] or candidate['image']}")
    winner = summary['best_trained']
    if winner is None:
        print('No trained candidate with a successful episode met the selection criteria.', file=sys.stderr)
        return 1
    print(f"Best trained run: {winner['run_dir']} ({winner['image']})")
    if 'best_trained_beating_baseline' in summary:
        improved = summary['best_trained_beating_baseline']
        if improved:
            print(f"Trained run with improved cross-track error: {improved['run_dir']} ({improved['image']})")
        else:
            print('No trained run improved median peak cross-track error while preserving success and worst peak.')
    return 0


raise SystemExit(main())
PY
