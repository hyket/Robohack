#!/usr/bin/env bash
set -eo pipefail

case "${1:-ping}" in
    ping)
        printf '%s\n' '{"protocol_version":"1.0","supported_tasks":["T2"],"self_trained_rl":false,"preferred_speed_mps":null}'
        ;;
    T2)
        shift
        source /opt/ros/humble/setup.bash
        source /opt/unitree/local_setup.bash
        exec python3 /opt/t2/safety_block.py "$@"
        ;;
    *)
        printf 'Поддерживаются команды ping и T2. Получено: %s\n' "$1" >&2
        exit 2
        ;;
esac
