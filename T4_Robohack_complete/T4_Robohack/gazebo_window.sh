#!/usr/bin/env bash
# Launch only the GUI for an existing simulation. Does not start another server.
set -e
cd -- "$(dirname -- "$0")"
mkdir -p results
docker info >/dev/null
if [ "$#" -eq 1 ]; then
    container="$1"
else
    mapfile -t candidates < <(docker ps --filter ancestor=registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 --format '{{.Names}}')
    [ "${#candidates[@]}" -eq 1 ] || { echo 'Pass the exact simulator container name as the only argument.'; exit 2; }
    container="${candidates[0]}"
fi
docker exec "$container" pgrep -x gzserver >/dev/null || { echo 'No gzserver in this container'; exit 1; }
if docker exec "$container" pgrep -x gzclient >/dev/null; then
    echo 'gzclient already exists; inspect its window/log rather than starting a duplicate.'; exit 1
fi
docker logs "$container" > results/simulator.log 2>&1
# CPU rendering is a diagnostic fallback for GPU/OpenGL driver problems.
# Do not weaken X11 permissions or alter host display settings.
set -o pipefail
docker exec -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  "$container" bash -c 'source /etc/go1_sim_env.sh; exec gzclient --verbose' 2>&1 | tee results/gzclient.log
