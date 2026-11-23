#!/usr/bin/env bash
# Starts local test motion; do not run alongside Judge or another controller.
set -e
cd -- "$(dirname -- "$0")"
exec docker run --rm --name go1-t4-trial --network host --ipc host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_LOG_DIR=/tmp/ros_logs -e ROS_DOMAIN_ID=151 \
  -e CYCLONEDDS_URI=file:///tmp/cyclonedds.xml \
  -v "$HOME/go1-simulator-public/user_data/ros2_custom_config_setup/cyclonedds_local.xml:/tmp/cyclonedds.xml:ro" \
  -v "$PWD:/work" --entrypoint /bin/bash go1-t4:v2 \
  -c 'source /opt/ros/humble/setup.bash; exec python3 /work/record_trial.py --drive --output /work/results "$@"' _ "$@"
