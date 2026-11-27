#!/bin/bash
set -eo pipefail

if [ "$#" -eq 1 ] && [ "$1" = 'ping' ]; then
    printf '%s\n' '{"protocol_version":"1.0","supported_tasks":["T3"],"self_trained_rl":false,"preferred_speed_mps":0.6}'
    exit 0
fi

if [ "${1:-}" = 'T3' ]; then
    shift
    source /opt/ros/humble/setup.bash
    source /opt/unitree/local_setup.bash
    exec python3 /opt/t3/participant.py "$@"
fi

printf '%s\n' 'Usage: /entrypoint.sh ping|T3' >&2
exit 2
