#!/usr/bin/env bash
# コンテナ内のジョグCLIを呼ぶラッパ。例: ./scripts/jog.sh status
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose exec velodyne_rmd python3 /workspaces/velodyne-rmd/app/jog.py "$@"
