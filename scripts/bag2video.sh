#!/usr/bin/env bash
# 記録済みbagを再生してrviz2の画面をmp4に録画する(仮想ディスプレイ使用、画面には出ない)。
# ドローン追跡セッション(ros2 bag record -a)の可視化・X投稿用動画づくりを想定。
# 使い方:
#   scripts/bag2video.sh <bagディレクトリ> [出力ファイル名.mp4] [再生速度]
# 例:
#   scripts/bag2video.sh ~/rosbag/rosbag2_2026_08_04-15_00_00
#   scripts/bag2video.sh bags/map_sweep_20260804 sweep.mp4 2.0
# 出力は videos/ 以下。bagはリポジトリ内か ~/rosbag 以下に置くこと(コンテナから見える場所)。
set -euo pipefail
cd "$(dirname "$0")/.."

BAG_HOST="$(realpath "${1:?使い方: bag2video.sh <bagディレクトリ> [出力.mp4] [再生速度]}")"
RATE="${3:-1.0}"
ROS_USER="${ROS_USER:-rosuser}"

[[ -f "$BAG_HOST/metadata.yaml" ]] || {
    echo "ERROR: $BAG_HOST はrosbag2ディレクトリではありません (metadata.yamlなし)" >&2
    exit 1
}

# ホストパス → コンテナ内パス
REPO="$(pwd)"
case "$BAG_HOST" in
    "$REPO"/*)         BAG_C="/workspaces/velodyne-rmd${BAG_HOST#"$REPO"}" ;;
    "$HOME"/rosbag/*)  BAG_C="/home/$ROS_USER/rosbag${BAG_HOST#"$HOME"/rosbag}" ;;
    *)  echo "ERROR: bagはリポジトリ内か ~/rosbag 以下に置いてください (コンテナ未マウント)" >&2
        exit 1 ;;
esac

mkdir -p videos
OUT_NAME="${2:-$(basename "$BAG_HOST").mp4}"
OUT_C="/workspaces/velodyne-rmd/videos/$(basename "$OUT_NAME")"

echo "==> コンテナを起動"
docker compose up -d --build

# 再生gotcha: 実機ドライバや追跡ノードが生きているとbagのトピック/TFと混ざって壊れる
docker compose exec velodyne_rmd bash -c \
    "pkill -INT -f '[t]ilt_servo.py|[p]erception.py|[t]racking_controller.py|[t]ilt_node.py|[t]rajectory_viz.py' 2>/dev/null; \
     sleep 1; pkill -f '[l]ivox_ros_driver2_node|[X]vfb|[r]viz2|[f]fmpeg' 2>/dev/null; true"

docker compose exec velodyne_rmd \
    /workspaces/velodyne-rmd/scripts/_bag2video_inner.sh "$BAG_C" "$OUT_C" "$RATE"

echo "==> 完了: videos/$(basename "$OUT_NAME")"
