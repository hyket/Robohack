#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
sim="${GO1_SIM_DIR:-$root/go1-simulator-public}"
image="${SIM_IMAGE:-registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0}"
mode="${1:---gui}"

case "$mode" in
    --gui|--headless) ;;
    *) echo "Использование: bash T4/start_gazebo.sh [--gui|--headless]" >&2; exit 2 ;;
esac
docker info >/dev/null
docker compose version
if [[ ! -f "$sim/run_local.py" ]]; then
    git clone https://github.com/Urika000/go1-simulator-public.git "$sim"
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
    docker pull "$image"
fi
docker buildx build --platform linux/amd64 --load -t robohack-t4:local "$root"
docker run --rm --network none robohack-t4:local ping
exec python3 "$sim/run_local.py" --task T4 --sim-image "$image" \
    --participant-image robohack-t4:local "$mode" --logs
