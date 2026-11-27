#!/usr/bin/env bash
set -euo pipefail

usage() { printf 'Usage: bash T3/build_trained.sh RUN_DIR [IMAGE_TAG]\n'; }
die() { printf 'T3 build: %s\n' "$*" >&2; exit 1; }

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi
if (($# < 1 || $# > 2)); then
    usage >&2
    exit 2
fi

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ -d "$1" ]] || die "experiment directory not found: $1"
run_dir=$(cd -- "$1" && pwd)
image=${2:-go1-participant:t3-trained}
checkpoint_files=(checkpoints/ac_weights_last.pt checkpoints/adaptation_module_latest.jit checkpoints/body_latest.jit)
for file in experiment.txt initial-checkpoint.sha256 training.patch dependencies.txt training.log \
    trained-checkpoints.sha256 walk_these_ways.npz "${checkpoint_files[@]}"; do
    [[ -s "$run_dir/$file" ]] || die "missing or empty training artifact: $run_dir/$file"
done

if command -v sha256sum >/dev/null; then
    sha256=(sha256sum)
else
    sha256=(shasum -a 256)
fi

context=$(mktemp -d "${TMPDIR:-/tmp}/robohack-t3-build.XXXXXX")
trap 'rm -rf -- "$context"' EXIT
checkpoints="$context/T3/checkpoints"
mkdir -p -- "$context/T2/t2_checked" "$checkpoints"
cp -- "$run_dir/walk_these_ways.npz" "$checkpoints/"

verified_hashes=$(cd -- "$run_dir" && "${sha256[@]}" "${checkpoint_files[@]}" &&
    cd -- "$checkpoints" && "${sha256[@]}" walk_these_ways.npz)
[[ "$verified_hashes" == "$(<"$run_dir/trained-checkpoints.sha256")" ]] ||
    die 'training artifacts do not match trained-checkpoints.sha256'

read -r initial_hash _ < "$run_dir/initial-checkpoint.sha256"
initial_hash=${initial_hash#\\}
[[ "$initial_hash" =~ ^[a-f0-9]{64}$ ]] || die 'invalid SHA-256 in initial-checkpoint.sha256'
[[ "${verified_hashes%% *}" != "$initial_hash" ]] || die 'final actor-critic checkpoint equals the initial checkpoint'

python3 - "$repo/T3/checkpoints/walk_these_ways.npz" "$checkpoints/walk_these_ways.npz" <<'PY'
import sys
from zipfile import BadZipFile, ZipFile

try:
    with ZipFile(sys.argv[1]) as baseline, ZipFile(sys.argv[2]) as trained:
        names = trained.namelist()
        if sorted(names) != sorted(baseline.namelist()):
            raise SystemExit("T3 build: trained NPZ has unexpected weight entries")
        if all(trained.read(name) == baseline.read(name) for name in names):
            raise SystemExit("T3 build: trained NPZ contains the original Walk These Ways weights")
        if trained.testzip() is not None:
            raise SystemExit("T3 build: trained NPZ has a corrupt weight entry")
except BadZipFile as error:
    raise SystemExit(f"T3 build: invalid NPZ checkpoint: {error}") from error
PY

cp -- "$repo/Dockerfile" "$repo/.dockerignore" "$context/"
cp -R -- "$repo/T2/t2_checked/"{model,vendor,NOTICE-go1-MIT.txt,requirements.lock,safety_block.py,collision_guard.py,t2_collision_pairs.csv} \
    "$context/T2/t2_checked/"
cp -- "$repo/T3/"{policy.py,controller.py,participant.py} "$context/T3/"
cp -- "$repo/T3/checkpoints/LICENSE-walk-these-ways.txt" "$checkpoints/"

{
    printf '# Происхождение весов T3\n\n'
    printf '`walk_these_ways.npz` получен дообучением политики Unitree Go1 на основе Walk These Ways.\n'
    printf '[Лицензия MIT](LICENSE-walk-these-ways.txt).\n\n'
    printf '## Параметры эксперимента\n\n```text\n'
    cat -- "$run_dir/experiment.txt"
    printf 'initial_checkpoint_sha256=%s\n```\n\n' "$initial_hash"
    printf '## SHA-256 результата\n\n```text\n%s\n```\n\n' "$verified_hashes"
    printf 'Конфигурация, список зависимостей и журнал сохранены в каталоге эксперимента.\n'
} > "$checkpoints/README.md"

sed 's/"self_trained_rl":false/"self_trained_rl":true/' "$repo/entrypoint.sh" > "$context/entrypoint.sh"
baseline_ping=$(bash "$repo/entrypoint.sh" ping)
trained_ping=${baseline_ping/\"self_trained_rl\":false/\"self_trained_rl\":true}
[[ "$trained_ping" != "$baseline_ping" && "$(bash "$context/entrypoint.sh" ping)" == "$trained_ping" ]] ||
    die 'could not set self_trained_rl to true in the staged ping response'

docker buildx build --platform linux/amd64 --load -t "$image" "$context"
image_details=$(docker image inspect --format '{{.Os}}/{{.Architecture}} {{.Config.User}}' "$image")
[[ "$image_details" == 'linux/amd64 participant' ]] || die "unexpected image platform or user: $image_details"
SECONDS=0
ping=$(docker run --rm --pull never --platform linux/amd64 --network none \
    --cap-drop ALL --security-opt no-new-privileges --entrypoint /entrypoint.sh "$image" ping)
((SECONDS <= 10)) || die 'offline ping exceeded 10 seconds'
[[ "$ping" == "$trained_ping" ]] || die "unexpected offline ping response: $ping"
printf '%s\n' "$ping"
