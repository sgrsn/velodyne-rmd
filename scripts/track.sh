#!/usr/bin/env bash
# ドローン追跡デモ起動:
#   コンテナ → Livoxドライバ → tilt_servo(速度制御) → perception → tracking_controller → rviz2
# 起動後の流れ: MAP(背景取得掃引 ~55s) → SEARCH(探索掃引) → 目標検出でTRACK。
# 停止は stop.sh(tilt_servo等もまとめて落ちる)。
set -euo pipefail
cd "$(dirname "$0")/.."

MID360_IP=192.168.1.106

if [[ ! -f config/limits.yaml ]]; then
    echo "ERROR: config/limits.yaml がありません。先にジョグ手順で較正してください。" >&2
    exit 1
fi

xhost +local: >/dev/null

echo "==> コンテナをビルド・起動"
docker compose up -d --build

echo "==> Mid-360 ($MID360_IP) を確認"
if ! ping -c1 -W1 "$MID360_IP" >/dev/null 2>&1; then
    echo "ERROR: Mid-360に到達できません。配線・電源を確認してください。" >&2
    exit 1
fi

# 再実行時の2重起動防止([ ]はpkill自身へのマッチ回避)
docker compose exec velodyne_rmd bash -c \
    "pkill -INT -f '[t]ilt_servo.py|[p]erception.py|[t]racking_controller.py|[t]ilt_node.py' 2>/dev/null; \
     sleep 1; pkill -f '[l]ivox_ros_driver2_node' 2>/dev/null; true"

echo "==> livox_ros_driver2 を起動 (PointCloud2形式)"
docker compose exec -d velodyne_rmd bash -ic \
    "ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
        -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
        -p publish_freq:=10.0 -p output_data_type:=0 \
        -p frame_id:=livox_frame \
        -p user_config_path:=/workspaces/velodyne-rmd/config/MID360_config.json \
        > /tmp/livox_driver.log 2>&1"

echo "==> /livox/lidar の配信を確認中..."
# コンテナ起動直後はROSデーモン初回起動+ドライバ初期化で1回目が落ちやすい→リトライ
lidar_ok=false
for _ in 1 2 3; do
    if docker compose exec velodyne_rmd bash -ic \
            "timeout 12 ros2 topic echo --once /livox/lidar --no-arr >/dev/null 2>&1"; then
        lidar_ok=true
        break
    fi
done
if $lidar_ok; then
    echo "    OK: 点群を受信しています"
else
    echo "ERROR: /livox/lidar を受信できません (/tmp/livox_driver.log 参照)。" >&2
    exit 1
fi

echo "==> tilt_servo を起動 (100Hz速度制御 + TF)"
docker compose exec -d velodyne_rmd bash -ic \
    "cd /workspaces/velodyne-rmd/app && python3 tilt_servo.py > /tmp/tilt_servo.log 2>&1"

echo "==> perception を起動 (背景差分 + KF追跡)"
docker compose exec -d velodyne_rmd bash -ic \
    "cd /workspaces/velodyne-rmd/app && python3 perception.py > /tmp/perception.log 2>&1"

echo "==> tracking_controller を起動 (MAP掃引開始)"
docker compose exec -d velodyne_rmd bash -ic \
    "cd /workspaces/velodyne-rmd/app && python3 tracking_controller.py > /tmp/tracking_controller.log 2>&1"

echo "==> rviz2 を起動（ウィンドウを閉じてもノードは動き続けます。停止は stop.sh）"
echo "    背景取得掃引が約1分続いた後、探索/追跡に移行します。"
docker compose exec velodyne_rmd bash -ic \
    "rviz2 -d /workspaces/velodyne-rmd/config/vlp16_tilt.rviz"
