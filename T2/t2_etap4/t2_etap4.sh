#!/usr/bin/env bash
set -euo pipefail

repo_dir=${1:-"$HOME/go1-simulator-public"}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sim_image=${SIM_IMAGE:-registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0}
node_file="$script_dir/safety_block.py"
model_source_dir="$repo_dir/models/mujoco/unitree_go1"

fail() {
    printf 'Ошибка: %s\n' "$1" >&2
    exit 1
}

[[ -f $node_file && -f $script_dir/collision_guard.py ]] \
    || fail 'Распакуй оба файла Python рядом с этим скриптом.'
python3 "$node_file" --self-test
[[ -f $repo_dir/run_local.py ]] || fail "Укажи папку симулятора первым аргументом: $repo_dir"
[[ -f $model_source_dir/t2_collision_pairs.csv ]] \
    || fail "В $model_source_dir отсутствует таблица пар t2_collision_pairs.csv."
model_source_dir=$(cd -- "$model_source_dir" && pwd)
docker info --format '{{.ServerVersion}}' >/dev/null || fail 'Проверь доступ к Docker из этого терминала.'

run_log=$(mktemp "$HOME/t2-stage4-simulator-XXXXXX.log")
printf 'Запускаю T2 в Docker. Журнал симулятора: %s\n' "$run_log"
python3 "$repo_dir/run_local.py" --task T2 --sim-only --sim-image "$sim_image" \
    --headless --logs --timeout 180 >"$run_log" 2>&1 &
launcher_pid=$!

finish() {
    local result=$? launcher_status=0
    trap - EXIT
    kill -TERM "$launcher_pid" 2>/dev/null || true
    wait "$launcher_pid" || launcher_status=$?
    if (( result == 0 && launcher_status != 0 && launcher_status != 143 )); then
        printf 'Проверь остановку симулятора в журнале %s\n' "$run_log" >&2
        exit "$launcher_status"
    fi
    exit "$result"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

deadline=$((SECONDS + 215))
until grep -q '^Simulator started' "$run_log"; do
    if ! kill -0 "$launcher_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        tail -n 30 "$run_log" >&2
        fail "Симулятор не запустился. Полный журнал: $run_log"
    fi
    sleep 1
done

container=$(sed -n 's/^Simulator=//p' "$run_log" | head -n 1)
[[ -n $container ]] || fail "В журнале отсутствует имя контейнера: $run_log"
test_dir=$(docker exec "$container" mktemp -d /tmp/t2-stage4-XXXXXX)
tar -cf - -C "$script_dir" safety_block.py collision_guard.py \
    -C "$model_source_dir" t2_collision_pairs.csv \
    | docker exec -i "$container" tar -xf - -C "$test_dir"

printf 'Проверяю самоколлизии в MuJoCo и обмен по тестовым топикам ROS 2.\n'
docker exec "$container" bash -c \
    'source /etc/go1_sim_env.sh && exec timeout 80s python3 "$1/safety_block.py" --integration-test' \
    _ "$test_dir"
