#!/usr/bin/env bash
# チルトデモ起動:
#   コンテナ起動 → 接続中のLiDARドライバ → チルトノード(掃引+TF) → rviz2
# VLP-16 / Livox Mid-360 はIP疎通で自動検出し、届いたものだけ起動する。
# 事前にジョグ手順でconfig/limits.yamlを作成しておくこと。
set -euo pipefail
cd "$(dirname "$0")/.."

VLP16_IP=192.168.1.201
MID360_IP=192.168.1.106

if [[ ! -f config/limits.yaml ]]; then
    echo "ERROR: config/limits.yaml がありません。" >&2
    echo "       先にジョグ手順で上限/下限を設定してください (README参照)。" >&2
    exit 1
fi

xhost +local: >/dev/null

echo "==> コンテナをビルド・起動"
docker compose up -d --build

echo "==> 接続センサを検出"
have_vlp16=false have_mid360=false
ping -c1 -W1 "$VLP16_IP"  >/dev/null 2>&1 && have_vlp16=true
ping -c1 -W1 "$MID360_IP" >/dev/null 2>&1 && have_mid360=true
echo "    VLP-16 ($VLP16_IP): $($have_vlp16 && echo 接続 || echo 未接続)"
echo "    Mid-360 ($MID360_IP): $($have_mid360 && echo 接続 || echo 未接続)"
if ! $have_vlp16 && ! $have_mid360; then
    echo "ERROR: どのLiDARにも到達できません。配線・電源を確認してください。" >&2
    exit 1
fi

check_topic() {
    docker compose exec velodyne_rmd bash -ic \
        "timeout 15 ros2 topic echo --once $1 --no-arr >/dev/null 2>&1"
}

if $have_vlp16; then
    echo "==> velodyneドライバ一式を起動"
    docker compose exec -d velodyne_rmd bash -ic \
        "ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py"
fi

if $have_mid360; then
    echo "==> livox_ros_driver2 を起動 (PointCloud2形式)"
    docker compose exec -d velodyne_rmd bash -ic \
        "ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
            -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
            -p publish_freq:=10.0 -p output_data_type:=0 \
            -p frame_id:=livox_frame \
            -p user_config_path:=/workspaces/velodyne-rmd/config/MID360_config.json \
            > /tmp/livox_driver.log 2>&1"
fi

if $have_vlp16; then
    echo "==> /velodyne_points の配信を確認中..."
    if check_topic /velodyne_points; then echo "    OK: 点群を受信しています"
    else echo "WARNING: /velodyne_points を受信できていません。" >&2; fi
fi
if $have_mid360; then
    echo "==> /livox/lidar の配信を確認中..."
    if check_topic /livox/lidar; then echo "    OK: 点群を受信しています"
    else echo "WARNING: /livox/lidar を受信できていません (/tmp/livox_driver.log 参照)。" >&2; fi
fi

echo "==> チルトノードを起動 (掃引 + TF配信)"
docker compose exec -d velodyne_rmd bash -ic \
    "cd /workspaces/velodyne-rmd/app && python3 tilt_node.py --ros-args -p speed_dps:=25.0 -p tf_rate_hz:=30.0 > /tmp/tilt_node.log 2>&1"

echo "==> rviz2 を起動（ウィンドウを閉じてもノードは動き続けます。停止は stop.sh）"
docker compose exec velodyne_rmd bash -ic \
    "rviz2 -d /workspaces/velodyne-rmd/config/vlp16_tilt.rviz"
