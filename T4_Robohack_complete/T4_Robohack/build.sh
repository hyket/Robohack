#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
[ "$(uname -m)" = x86_64 ] || { echo 'This build script requires an amd64 host'; exit 1; }
docker info >/dev/null
if docker buildx version >/dev/null 2>&1; then
    docker buildx build --platform linux/amd64 --load -t go1-t4:v2 .
else
    DOCKER_BUILDKIT=0 docker build -t go1-t4:v2 .
fi
docker run --rm --network none --cap-drop ALL --security-opt no-new-privileges go1-t4:v2 ping
