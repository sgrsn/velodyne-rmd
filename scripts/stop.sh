#!/usr/bin/env bash
# デモ停止。モータは停止指令(通電保持)を送ってからコンテナを落とす。
# --release を付けるとモータを脱力する(センサが自重で動く可能性あり)。
set -euo pipefail
cd "$(dirname "$0")/.."

if docker compose ps --status running velodyne_rmd >/dev/null 2>&1; then
    docker compose exec velodyne_rmd bash -c \
        "pkill -INT -f tilt_node.py 2>/dev/null || true"
    sleep 1
    if [[ "${1:-}" == "--release" ]]; then
        docker compose exec velodyne_rmd python3 /workspaces/velodyne-rmd/app/jog.py release || true
    else
        docker compose exec velodyne_rmd python3 /workspaces/velodyne-rmd/app/jog.py stop || true
    fi
fi

docker compose down
