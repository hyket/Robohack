#!/usr/bin/env bash
set -euo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output="$repo/output/t3-training"
hours=8
envs=1024
initial=
isaac=

usage() {
    cat <<'EOF'
Usage: bash T3/train_night.sh [options]

  --hours N        maximum wall-clock hours for the batch (default 8)
  --envs N         parallel environments for each run (default 1024)
  --checkpoint PT  starting weights for three branches (default latest completed 400-iteration run)
  --isaac-gym PATH pass the archive to the trainer if it has not been cached yet
EOF
}

die() { printf 'T3 night training: %s\n' "$*" >&2; exit 1; }

while (($#)); do
    case "$1" in
        --hours|--envs|--checkpoint|--isaac-gym)
            (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || die "missing value for $1"
            case "$1" in
                --hours) hours=$2 ;;
                --envs) envs=$2 ;;
                --checkpoint) initial=$2 ;;
                --isaac-gym) isaac=$2 ;;
            esac
            shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown option $1 (see --help)" ;;
    esac
done

[[ "$hours" =~ ^[1-9][0-9]*$ && "$envs" =~ ^[1-9][0-9]*$ ]] || die 'hours and envs must be positive integers'
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || die 'run on x86-64 Linux or WSL 2'
command -v timeout >/dev/null || die 'install GNU coreutils for the wall-clock limit'
mkdir -p -- "$output"

if [[ -n "$initial" ]]; then
    [[ -s "$initial" ]] || die "checkpoint not found: $initial"
    initial=$(realpath -- "$initial")
else
    latest_run=
    while IFS= read -r -d '' manifest; do
        run=${manifest%/*}
        if [[ "${run##*/}" > "${latest_run##*/}" && -s "$run/checkpoints/ac_weights_last.pt" ]] &&
            grep -qx 'iterations=400' "$run/experiment.txt" 2>/dev/null &&
            (cd -- "$run" && sha256sum --check --status trained-checkpoints.sha256); then
            latest_run=$run
        fi
    done < <(find "$output" -name trained-checkpoints.sha256 -type f -print0)
    if [[ -n "$latest_run" ]]; then
        initial="$latest_run/checkpoints/ac_weights_last.pt"
    fi
fi

night=$(mktemp -d "$output/night-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
run_batch() {
    printf 'Batch directory: %s\n' "$night"
    printf 'Starting checkpoint: %s\n' "${initial:-published Walk These Ways weights}"

    common=(--envs "$envs")
    if [[ -n "$isaac" ]]; then
        common+=(--isaac-gym "$isaac")
    fi

    # Each branch exports after both stages. The fresh branch starts independently.
    branches=(gentle standard fresh small-step)
    rates=(5e-5 1e-4 5e-5 2.5e-5)
    seeds=(42 42 43 42)
    checkpoints=("$initial" "$initial" '' "$initial")
    stages_left=$((${#branches[@]} * 2))
    completed=0
    budget=$((hours * 3600))
    pace_ms=2300
    worker=

    stop_batch() {
        if [[ -n "$worker" ]]; then
            kill -TERM "$worker" 2>/dev/null || true
            wait "$worker" 2>/dev/null || true
        fi
        exit "$1"
    }
    trap 'stop_batch 130' INT
    trap 'stop_batch 143' TERM
    SECONDS=0

    for round in 1 2; do
        for index in "${!branches[@]}"; do
            remaining=$((budget - SECONDS - 20))
            slot=$((remaining / stages_left))
            stages_left=$((stages_left - 1))
            if ((slot < 120)); then
                printf 'Time budget reached.\n'
                break 2
            fi

            # Round down to save time for Isaac Gym startup and export.
            iterations=$(((slot - 30) * 1000 / pace_ms / 100 * 100))
            ((iterations >= 100)) || iterations=100
            limit=$((slot + slot / 4))
            ((limit <= remaining)) || limit=$remaining

            label="${branches[index]}-$round"
            stage_output="$night/$label"
            args=("${common[@]}" --iterations "$iterations" --seed "${seeds[index]}"
                  --learning-rate "${rates[index]}" --output "$stage_output")
            if [[ -n "${checkpoints[index]}" ]]; then
                args+=(--checkpoint "${checkpoints[index]}")
            fi

            printf '\n%s | %s iterations | LR %s | seed %s | time limit %ss\n' \
                "$label" "$iterations" "${rates[index]}" "${seeds[index]}" "$limit"
            start=$SECONDS
            timeout --kill-after=15s "${limit}s" bash "$repo/T3/train.sh" "${args[@]}" </dev/null &
            worker=$!
            if wait "$worker"; then
                for run in "$stage_output"/run-*; do
                    [[ -s "$run/trained-checkpoints.sha256" && -s "$run/walk_these_ways.npz" &&
                       -s "$run/checkpoints/ac_weights_last.pt" ]] || continue
                    checkpoints[index]="$run/checkpoints/ac_weights_last.pt"
                    completed=$((completed + 1))
                    elapsed=$((SECONDS - start))
                    if ((elapsed > 30)); then
                        observed_ms=$((elapsed * 1000 / iterations))
                        pace_ms=$(((pace_ms + observed_ms * 3) / 4))
                    fi
                    printf 'Ready for MuJoCo: %s\n' "$run"
                    break
                done
            else
                status=$?
                printf '%s stopped with exit %s. Continuing from the last complete checkpoint for this branch.\n' \
                    "$label" "$status"
                if ((status == 124 || status == 137)); then
                    pace_ms=$((pace_ms * 5 / 4))
                fi
            fi
            worker=
        done
    done

    printf '\nFinished: %s complete candidates. Results: %s\n' "$completed" "$night"
    ((completed > 0)) || die 'no new complete candidates; check the trainer output above and the run logs'
}

run_batch | tee "$night/night.log"
