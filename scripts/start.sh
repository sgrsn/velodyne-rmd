#!/usr/bin/env bash
# Velodyne+RMDチルトデモ起動:
#   コンテナ起動 → velodyneドライバ → チルトノード(掃引+TF) → rviz2
# 事前にジョグ手順でconfig/limits.yamlを作成しておくこと。
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -f config/limits.yaml ]]; then
    echo "ERROR: config/limits.yaml がありません。" >&2
    echo "       先にジョグ手順で上限/下限を設定してください (README参照)。" >&2
    exit 1
fi

xhost +local: >/dev/null

echo "==> コンテナをビルド・起動"
docker compose up -d --build

echo "==> velodyneドライバ一式を起動"
docker compose exec -d velodyne_rmd bash -ic \
    "ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py"

echo "==> /velodyne_points の配信を確認中..."
if docker compose exec velodyne_rmd bash -ic \
    "timeout 10 ros2 topic echo --once /velodyne_points --no-arr >/dev/null 2>&1"; then
    echo "    OK: 点群を受信しています"
else
    echo "WARNING: /velodyne_points を受信できていません。" >&2
    echo "         ~/workspace/velodyne_demo/scripts/setup_network.sh を確認してください。" >&2
fi

echo "==> チルトノードを起動 (掃引 + TF配信)"
docker compose exec -d velodyne_rmd bash -ic \
    "cd /workspaces/velodyne-rmd/app && python3 tilt_node.py --ros-args -p speed_dps:=25.0 -p tf_rate_hz:=30.0 > /tmp/tilt_node.log 2>&1"

echo "==> rviz2 を起動（ウィンドウを閉じてもノードは動き続けます。停止は stop.sh）"
docker compose exec velodyne_rmd bash -ic \
    "rviz2 -d /workspaces/velodyne-rmd/config/vlp16_tilt.rviz"
