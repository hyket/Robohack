#!/usr/bin/env bash
set -euo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sim_dir=${1:-"$HOME/go1-simulator-public"}
sim_image=${SIM_IMAGE:-registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0}
participant_image=${T2_PARTICIPANT_IMAGE:-go1-participant:t2}
launcher_pid=''

fail() {
    printf 'Ошибка: %s\n' "$1" >&2
    exit 1
}

stop_pair() {
    if [[ -z $launcher_pid ]]; then
        return 0
    fi
    local status=0
    kill -TERM "$launcher_pid" 2>/dev/null || true
    wait "$launcher_pid" || status=$?
    launcher_pid=''
    [[ $status == 0 || $status == 143 ]]
}

[[ -f $sim_dir/run_local.py ]] || fail "Передай путь к go1-simulator-public первым аргументом: $sim_dir"
docker info --format '{{.ServerVersion}}' >/dev/null || fail 'Проверь доступ к Docker из этого терминала.'
docker buildx version || fail 'На Ubuntu 24.04 установи Buildx: sudo apt install docker-buildx'
printf '\n[1/3] Собираю образ участника %s.\n' "$participant_image"
docker buildx build --platform linux/amd64 --load -t "$participant_image" "$project_dir"

printf '\n[2/3] Проверяю запуск участника без доступа к сети.\n'
platform=$(docker image inspect --format '{{.Os}}/{{.Architecture}}' "$participant_image")
[[ $platform == linux/amd64 ]] || fail "Требуется linux/amd64, получено $platform."
offline_run=(docker run --rm --platform linux/amd64 --pull never --network none)
entry_uid=$("${offline_run[@]}" --entrypoint id "$participant_image" -u)
[[ $entry_uid =~ ^[1-9][0-9]*$ ]] || fail 'Контейнер участника должен работать без root.'
ping=$(timeout 10s "${offline_run[@]}" --cap-drop ALL --security-opt no-new-privileges \
    --entrypoint /entrypoint.sh "$participant_image" ping)
python3 -c '
import json, sys
expected = {"protocol_version": "1.0", "supported_tasks": ["T2"],
            "self_trained_rl": False, "preferred_speed_mps": None}
report = json.loads(sys.argv[1])
if report != expected or report["self_trained_rl"] is not False:
    raise SystemExit("Некорректный JSON протокола ping")
print("OK — ping: один JSON, T2 заявлен; пользователь UID " + sys.argv[2])
' "$ping" "$entry_uid"
timeout 30s "${offline_run[@]}" --entrypoint /entrypoint.sh \
    "$participant_image" T2 --collision-test
timeout 30s "${offline_run[@]}" -e ROS_LOCALHOST_ONLY=1 --entrypoint /entrypoint.sh \
    "$participant_image" T2 --clock-test

run_log=$(mktemp "$HOME/t2-stage6-pair-XXXXXX.log")
check_log=$(mktemp "$HOME/t2-stage6-check-XXXXXX.log")
trap 'stop_pair || printf "Проверь остановку контейнеров в %s\n" "$run_log" >&2' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '\n[3/3] Запускаю два контейнера. Журнал: %s\n' "$run_log"
printf 'Журнал измерений /clock и /lowcmd: %s\n' "$check_log"
python3 "$sim_dir/run_local.py" --task T2 --sim-image "$sim_image" \
    --participant-image "$participant_image" --headless --logs --timeout 180 >"$run_log" 2>&1 &
launcher_pid=$!
deadline=$((SECONDS + 230))
until grep -q '^Pair started' "$run_log"; do
    if ! kill -0 "$launcher_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        tail -n 35 "$run_log" >&2
        fail "Пара Simulator и Participant не запустилась. Журнал: $run_log"
    fi
    sleep 1
done

participant=$(sed -n 's/^Participant=//p' "$run_log" | head -n 1)
[[ -n $participant ]] || fail "В журнале отсутствует имя контейнера участника: $run_log"
run_check() {
    docker exec "$participant" timeout 65s /entrypoint.sh T2 --external-test 2>&1 | tee -a "$check_log"
}

check_status=0
run_check || check_status=$?
if (( check_status == 75 )); then
    printf 'Первый замер не пройден из-за разрыва /clock. Запускаю один повтор с теми же порогами.\n' |
        tee -a "$check_log"
    check_status=0
    run_check || check_status=$?
fi
if (( check_status != 0 )); then
    tail -n 35 "$run_log" >&2
    fail "Проверка обмена между контейнерами не прошла. Измерения: $check_log. Журнал запуска: $run_log"
fi
stop_pair || fail "Не удалось штатно остановить контейнеры. Журнал: $run_log"
printf '\nЭТАП 6 ПРОЙДЕН. Локальный образ: %s.\n' "$participant_image"
