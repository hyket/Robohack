#!/bin/sh
set -e
case "${1:-}" in
  ping)
    [ "$#" -eq 1 ] || exit 2
    printf '%s\n' '{"protocol_version":"1.0","supported_tasks":["T4"],"self_trained_rl":false,"preferred_speed_mps":null}'
    ;;
  T4)
    [ "$#" -eq 1 ] || exit 2
    . /opt/ros/humble/setup.sh
    exec python3 /opt/t4/node.py
    ;;
  *) echo 'Usage: /entrypoint.sh ping|T4' >&2; exit 2 ;;
esac
