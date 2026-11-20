#!/usr/bin/env bash
set -Eeuo pipefail

# Использование: bash t2_etap1.sh [путь_к_go1-simulator-public]
repo_url='https://github.com/Urika000/go1-simulator-public.git'
repo_dir=${1:-"$HOME/go1-simulator-public"}
sim_image=${SIM_IMAGE:-registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0}
launcher_pid=''

fail() {
    printf 'Ошибка: %s\n' "$1" >&2
    exit 1
}

stop_simulator() {
    local status=0
    if [[ -n $launcher_pid ]]; then
        kill -TERM "$launcher_pid" 2>/dev/null || true
        wait "$launcher_pid" || status=$?
        launcher_pid=''
        # run_local.py возвращает 143 после штатной остановки сигналом SIGTERM.
        if [[ $status != 0 && $status != 143 ]]; then
            return "$status"
        fi
    fi
}

printf '[1/4] Проверяю окружение Ubuntu.\n'
[[ $(uname -m) == x86_64 ]] || fail 'Готовый образ симулятора рассчитан на x86_64 (amd64).'
command -v python3 >/dev/null || fail 'Установи Python: sudo apt install python3'
python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' || fail 'Нужен Python 3.10 или новее.'
command -v docker >/dev/null || fail 'Установи Docker Engine и Docker Compose для Ubuntu: https://docs.docker.com/engine/install/ubuntu/'
docker compose version || fail 'Нужен плагин Docker Compose: https://docs.docker.com/compose/install/linux/'
docker info --format '{{.ServerVersion}}' || fail 'Нет доступа к Docker Engine. Проверь сообщение Docker выше.'

printf '\n[2/4] Подготавливаю репозиторий и образ.\n'
if [[ ! -e $repo_dir && ! -L $repo_dir ]]; then
    command -v git >/dev/null || fail 'Установи Git: sudo apt install git'
    git clone --depth 1 --branch main "$repo_url" "$repo_dir" || fail 'Не удалось скачать репозиторий с GitHub.'
fi
cd -- "$repo_dir" || fail "Не удалось открыть $repo_dir."
for file in run_local.py docker-compose.yml docker-compose.headless.yml; do
    [[ -f $file ]] || fail "В $PWD отсутствует $file. Передай путь к репозиторию симулятора первым аргументом."
done

if ! docker image inspect "$sim_image" >/dev/null 2>&1; then
    docker pull --platform linux/amd64 "$sim_image" || fail 'Не удалось скачать образ симулятора из Harbor.'
fi
platform=$(docker image inspect --format '{{.Os}}/{{.Architecture}}' "$sim_image")
[[ $platform == linux/amd64 ]] || fail "Требуется образ linux/amd64, получен $platform."
SIM_IMAGE="$sim_image" docker compose -f docker-compose.yml -f docker-compose.headless.yml config --quiet \
    || fail 'Compose не смог прочитать конфигурацию. Если ошибка связана с !reset, обнови плагин Docker Compose.'

log_dir=$(mktemp -d "$HOME/t2-stage1-XXXXXX")
sim_log="$log_dir/simulator.log"
trap 'stop_simulator || printf "Проверь завершение симулятора в %s\n" "$sim_log" >&2' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf '\n[3/4] Запускаю MuJoCo без графического окна.\n'
printf 'Журнал запуска: %s\n' "$sim_log"
python3 run_local.py --task T2 --sim-only --sim-image "$sim_image" \
    --headless --logs --timeout 180 >"$sim_log" 2>&1 &
launcher_pid=$!

deadline=$((SECONDS + 215))
until grep -q '^Simulator started' "$sim_log"; do
    if ! kill -0 "$launcher_pid" 2>/dev/null || (( SECONDS >= deadline )); then
        tail -n 30 "$sim_log" >&2
        fail "Симулятор не дошёл до готовности. Полный журнал: $sim_log"
    fi
    sleep 1
done
sim_container=$(sed -n 's/^Simulator=//p' "$sim_log" | head -n 1)
[[ -n $sim_container ]] || fail "В журнале нет имени контейнера: $sim_log"

printf '\n[4/4] Читаю сообщения ROS 2 внутри контейнера.\n'
for topic in clock lowstate; do
    output="$log_dir/$topic.txt"
    if ! docker exec "$sim_container" bash -c \
        'source /etc/go1_sim_env.sh; exec timeout 20s ros2 topic echo "$1" --once --qos-reliability best_effort' \
        _ "/$topic" >"$output" 2>&1; then
        tail -n 20 "$output" >&2
        fail "Не удалось прочитать /$topic. Диагностика: $output"
    fi
done
grep -q '^clock:' "$log_dir/clock.txt" || fail "В /clock не получено время симуляции. См. $log_dir/clock.txt"
grep -q '^motor_state:' "$log_dir/lowstate.txt" || fail "В /lowstate нет поля motor_state. См. $log_dir/lowstate.txt"
printf '  /clock    — время симуляции получено.\n'
printf '  /lowstate — поле motor_state получено.\n'

stop_simulator || fail "Проверка прошла, но при остановке возникла ошибка. См. $sim_log"
printf '\nЭТАП 1 ПРОЙДЕН. Симулятор остановлен.\n'
printf 'Результаты проверки: %s\n' "$log_dir"
