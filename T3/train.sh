#!/usr/bin/env bash
set -euo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
revision=0e7236bdc81ce855cbe3d70345a7899452bdeb1c
source_url=https://github.com/Improbable-AI/walk-these-ways.git
baseline_hash=d1542b0b8c7b767753a286f1cbabe5b4dc86e67611fb95844e870b3c61f8869e
patch="$repo/T3/training/walk-these-ways.patch"
constraints="$repo/T3/training/constraints.txt"
cache="${XDG_CACHE_HOME:-$HOME/.cache}/robohack-t3"
output="$repo/output/t3-training"
isaac_input=
checkpoint=
iterations=2
envs=1024
seed=42
learning_rate=1e-4
wandb=false
prepare_only=false
verbose=false

usage() {
    cat <<'EOF'
Usage: bash T3/train.sh [options]

  --isaac-gym PATH    NVIDIA Isaac Gym Preview 4 tar.gz or extracted isaacgym directory
                      (needed once per cache directory)
  --iterations N      PPO iterations (default 2)
  --envs N            parallel environments (default 1024)
  --seed N            experiment seed (default 42)
  --learning-rate LR  policy PPO learning rate (default 1e-4)
  --wandb             also record an offline Weights & Biases run
  --checkpoint PATH   initial ac_weights checkpoint (default original Walk These Ways)
  --cache PATH        dependency cache (default ~/.cache/robohack-t3)
  --output PATH       experiment directory parent (default output/t3-training)
  --prepare-only      install and verify dependencies without running PPO
  --verbose           stream the full training log instead of one progress line
EOF
}

die() { printf 'T3 training: %s\n' "$*" >&2; exit 1; }

while (($#)); do
    case "$1" in
        --isaac-gym|--iterations|--envs|--seed|--learning-rate|--checkpoint|--cache|--output)
            (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || die "missing value for $1"
            case "$1" in
                --isaac-gym) isaac_input=$2 ;;
                --iterations) iterations=$2 ;;
                --envs) envs=$2 ;;
                --seed) seed=$2 ;;
                --learning-rate) learning_rate=$2 ;;
                --checkpoint) checkpoint=$2 ;;
                --cache) cache=$2 ;;
                --output) output=$2 ;;
            esac
            shift 2
            ;;
        --prepare-only) prepare_only=true; shift ;;
        --wandb) wandb=true; shift ;;
        --verbose) verbose=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown option $1 (see --help)" ;;
    esac
done

[[ "$envs" =~ ^[1-9][0-9]*$ && "$iterations" =~ ^[1-9][0-9]*$ ]] || die 'iterations and envs must be positive integers'
[[ "$seed" =~ ^[0-9]+$ ]] || die 'seed must be a non-negative integer'
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || die 'run on x86-64 Linux (Ubuntu 20.04 or WSL 2)'
command -v python3.8 >/dev/null || die 'install python3.8 and python3.8-venv first (see T3/TRAINING.md)'
[[ -z "$checkpoint" || -s "$checkpoint" ]] || die "checkpoint not found: $checkpoint"
python3.8 -c 'import math, sys; lr = float(sys.argv[1]); sys.exit(not (math.isfinite(lr) and lr > 0))' \
    "$learning_rate" 2>/dev/null || die 'learning rate must be a positive finite number'

mkdir -p -- "$cache"
cache=$(cd -- "$cache" && pwd)
exec 9>"$cache/lock"
flock -n 9 || die "another training command is using $cache"
scratch=
trap 'if [[ -n "$scratch" ]]; then rm -rf -- "$scratch"; fi' EXIT

isaac_binding=python/isaacgym/_bindings/linux-x86_64/gym_38.so
if [[ -d "$isaac_input" ]]; then
    isaac=$(cd -- "$isaac_input" && pwd)
elif [[ -n "$isaac_input" ]]; then
    [[ -f "$isaac_input" ]] || die "Isaac Gym archive not found: $isaac_input"
    isaac="$cache/isaacgym"
    if [[ ! -f "$isaac/$isaac_binding" ]]; then
        scratch=$(mktemp -d "$cache/isaacgym.XXXXXX")
        tar -xzf "$isaac_input" -C "$scratch" --no-same-owner --no-same-permissions
        [[ -f "$scratch/isaacgym/$isaac_binding" ]] || die 'expected the official Isaac Gym Preview 4 archive'
        mv -T -- "$scratch/isaacgym" "$isaac"
        rm -rf -- "$scratch"
        scratch=
    fi
elif [[ -f "$cache/isaac-path" ]]; then
    read -r isaac < "$cache/isaac-path"
else
    die 'provide the Isaac Gym Preview 4 download using --isaac-gym PATH (see T3/TRAINING.md)'
fi
[[ -f "$isaac/$isaac_binding" ]] || die "Python 3.8 binding not found in $isaac"
printf '%s\n' "$isaac" > "$cache/isaac-path"

source_repository="$cache/walk-these-ways"
if [[ ! -d "$source_repository" ]]; then
    scratch=$(mktemp -d "$cache/source.XXXXXX")
    git -C "$scratch" init -q
    git -C "$scratch" fetch -q --depth 1 "$source_url" "$revision"
    git -C "$scratch" checkout -q --detach FETCH_HEAD
    mv -T -- "$scratch" "$source_repository"
    scratch=
fi
[[ "$(git -C "$source_repository" rev-parse HEAD)" == "$revision" ]] || die 'unexpected source revision; use a fresh --cache directory'
patch_checksum=$(sha256sum < "$patch")
upstream="$cache/source-${patch_checksum:0:16}"
if [[ ! -d "$upstream" ]]; then
    git -C "$source_repository" worktree add --quiet --detach "$upstream" "$revision"
fi
[[ "$(git -C "$upstream" rev-parse HEAD)" == "$revision" ]] || die 'unexpected source revision; use a fresh --cache directory'
if git -C "$upstream" apply --index --check "$patch" 2>/dev/null; then
    git -C "$upstream" apply --index "$patch"
elif ! git -C "$upstream" apply --index --reverse --check "$patch" 2>/dev/null; then
    die 'training patch conflicts with the cached checkout; use a fresh --cache directory'
fi

if [[ -z "$checkpoint" ]]; then
    checkpoint="$upstream/runs/gait-conditioned-agility/pretrain-v0/train/025417.456545/checkpoints/ac_weights_last.pt"
    checksum=$(sha256sum < "$checkpoint")
    [[ "${checksum%% *}" == "$baseline_hash" ]] || die 'the bundled upstream checkpoint failed SHA-256 verification'
fi
checkpoint=$(realpath -- "$checkpoint")

python="$cache/venv/bin/python"
if [[ ! -x "$python" ]]; then
    python3.8 -m venv "$cache/venv" || die 'install python3.8-venv (see T3/TRAINING.md)'
fi
install_key=$({ printf '%s\n' "$upstream" "$isaac"; sha256sum "$constraints"; } | sha256sum)
if [[ ! -f "$cache/dependencies" || "$(cat "$cache/dependencies")" != "$install_key" ]]; then
    "$python" -m pip install 'pip==24.3.1' 'setuptools==69.5.1' wheel
    "$python" -m pip install -c "$constraints" torch torchvision -f https://download.pytorch.org/whl/cu113/torch_stable.html
    "$python" -m pip install -c "$constraints" -e "$isaac/python" -e "$upstream"
    "$python" -m pip check
    printf '%s\n' "$install_key" > "$cache/dependencies"
fi

wandb_enabled=0
if "$wandb"; then
    wandb_enabled=1
    export WANDB_ERROR_REPORTING=false
    if ! "$python" -c 'from importlib.metadata import version; assert version("wandb") == "0.18.7"' 2>/dev/null; then
        "$python" -m pip install -c "$constraints" 'wandb==0.18.7'
        "$python" -m pip check
    fi
fi

if [[ -d /usr/lib/wsl/lib ]]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
"$python" - <<'PY'
import isaacgym
import torch
from ml_logger import logger

if not torch.cuda.is_available():
    raise SystemExit("PyTorch cannot see an NVIDIA GPU. Check the driver and nvidia-smi.")
print(f"PyTorch {torch.__version__}, CUDA {torch.version.cuda}, GPU {torch.cuda.get_device_name(0)}")
PY

if "$prepare_only"; then
    printf 'Training dependencies ready in %s\n' "$cache"
    exit 0
fi

mkdir -p -- "$output"
output=$(cd -- "$output" && pwd)
run_dir=$(mktemp -d "$output/run-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
printf 'Experiment directory: %s\n' "$run_dir"
printf 'upstream=%s\nrevision=%s\niterations=%s\nenvs=%s\nseed=%s\nlearning_rate=%s\nwandb_offline=%s\n' \
    "$source_url" "$revision" "$iterations" "$envs" "$seed" "$learning_rate" "$wandb_enabled" > "$run_dir/experiment.txt"
sha256sum "$checkpoint" > "$run_dir/initial-checkpoint.sha256"
git -C "$upstream" diff --binary HEAD > "$run_dir/training.patch"
"$python" -m pip freeze > "$run_dir/dependencies.txt"

export WTW_INIT_WEIGHTS="$checkpoint" WTW_RUN_DIR="$run_dir"
export WTW_ITERATIONS="$iterations" WTW_NUM_ENVS="$envs" WTW_SEED="$seed"
export WTW_LEARNING_RATE="$learning_rate" WTW_WANDB="$wandb_enabled"

run_trainer() {
    (cd -- "$upstream" && "$python" -u scripts/train.py) 2>&1 | tee "$run_dir/training.log"
}

show_progress() {
    # Match the per-iteration summary from report_iteration in the training patch.
    local pattern='^PPO ([0-9]+)/([0-9]+) \| ([0-9.]+)s/iter \| [0-9]+ env steps/s \| reward ([^ ]+) \| forward RMSE ([^ ]+) m/s$'
    local line status='' width filled
    width=$(stty size <&2 2>/dev/null) || width=80
    width=${width##* }
    ((width > 1)) || width=80

    while IFS= read -r line; do
        if [[ "$line" =~ $pattern ]]; then
            filled=$((10 * BASH_REMATCH[1] / BASH_REMATCH[2]))
            printf -v status 'PPO [%.*s%.*s] %s/%s | %ss/iter | reward %s | RMSE %s m/s' \
                "$filled" '##########' "$((10 - filled))" '----------' \
                "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}" "${BASH_REMATCH[3]}" \
                "${BASH_REMATCH[4]}" "${BASH_REMATCH[5]}"
            # Leave the last terminal column free to avoid wrapping the progress line.
            printf '\r\033[K%.*s' "$((width - 1))" "$status" >&2
        fi
    done
    if [[ -n "$status" ]]; then
        printf '\n' >&2
    fi
}

if ! "$verbose" && [[ -t 2 && "${TERM:-dumb}" != dumb ]]; then
    printf 'Starting Isaac Gym. Full log: %s/training.log\n' "$run_dir"
    if run_trainer | show_progress; then
        printf 'Training complete. Exporting Participant weights.\n'
    else
        status=$?
        tail -n 20 -- "$run_dir/training.log" >&2
        exit "$status"
    fi
else
    run_trainer
fi

for file in ac_weights_last.pt adaptation_module_latest.jit body_latest.jit; do
    [[ -s "$run_dir/checkpoints/$file" ]] || die "missing training output: $run_dir/checkpoints/$file"
done
(
    cd -- "$repo"
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "$python" -m T3.checkpoints.convert \
        "$run_dir/checkpoints" "$run_dir/walk_these_ways.npz"
)
(
    cd -- "$run_dir"
    sha256sum checkpoints/ac_weights_last.pt checkpoints/adaptation_module_latest.jit \
        checkpoints/body_latest.jit walk_these_ways.npz > trained-checkpoints.sha256
)
printf 'Participant weights: %s/walk_these_ways.npz\n' "$run_dir"
