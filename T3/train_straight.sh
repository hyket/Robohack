#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf 'Usage: bash T3/train_straight.sh [--wandb | --wandb-online] [ITERATIONS [STRAIGHTNESS_WEIGHT]]\n'
    printf 'Defaults: up to 400 iterations per arm, straightness weight 0.5.\n'
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi

logging=
if [[ "${1:-}" == --wandb || "${1:-}" == --wandb-online ]]; then
    logging=$1
    shift
fi
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
iterations=${1:-400}
weight=${2:-0.5}
if (($# > 2)) || [[ ! "$iterations" =~ ^[1-9][0-9]*$ ]]; then
    usage >&2
    exit 2
fi
command -v python3.8 >/dev/null || { printf 'Install Python 3.8 first (see T3/TRAINING.md).\n' >&2; exit 1; }
if ! python3.8 -c 'import math, sys; w = float(sys.argv[1]); sys.exit(not (math.isfinite(w) and w > 0))' "$weight" 2>/dev/null; then
    printf 'Straightness weight must be positive and finite.\n' >&2
    exit 2
fi

output="$repo/output/t3-training"
mkdir -p -- "$output"
batch=$(mktemp -d "$output/straightness-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
common=(--iterations "$iterations" --envs 1024 --learning-rate 5e-5 --seed 42 --forward-speed-range 0.6 0.6)
common+=(--eval-every 100 --eval-envs 16 --early-stopping-rounds 2 --target-kl 0.02)
if [[ -n "$logging" ]]; then
    common+=("$logging")
fi
export WANDB_RUN_GROUP="${WANDB_RUN_GROUP:-${batch##*/}}"

(
    printf 'A/B results: %s\n' "$batch"
    printf 'Both runs start from published Walk These Ways weights.\n'
    printf '\nStraightness arm, reward weight %s\n' "$weight"
    WANDB_NAME="${batch##*/}-straightness" bash "$repo/T3/train.sh" "${common[@]}" \
        --straightness-weight "$weight" --output "$batch/straightness" &&
    printf '\nControl arm, reward weight 0\n' &&
    WANDB_NAME="${batch##*/}-control" bash "$repo/T3/train.sh" "${common[@]}" \
        --straightness-weight 0 --output "$batch/control" &&
    printf '\nBoth runs exported. Results: %s\n' "$batch"
) | tee "$batch/batch.log"
