#!/usr/bin/env bash
# bagを再生しながらrviz2を実画面に表示する(画角をリアルタイムに動かす用。画面録画は各自で)。
# 軌跡(緑ライン)・現在位置(オレンジ球)・状態テキストは trajectory_viz が重畳する。
# rviz2のウィンドウを閉じると再生・関連ノードも止まる。
# 使い方:
#   scripts/replay.sh <bagディレクトリ> [再生速度] [--loop]
# 例:
#   scripts/replay.sh ~/rosbag/track            # 等倍・1回再生
#   scripts/replay.sh ~/rosbag/track 1.0 --loop # ループ再生(画角探しに便利)
set -euo pipefail
cd "$(dirname "$0")/.."

BAG_HOST="$(realpath "${1:?使い方: replay.sh <bagディレクトリ> [再生速度] [--loop]}")"
RATE="${2:-1.0}"
LOOP_ARG=""
[[ "${3:-}" == "--loop" ]] && LOOP_ARG="--loop"
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

xhost +local: >/dev/null

echo "==> コンテナを起動"
docker compose up -d --build

# 再生gotcha: 実機ドライバや追跡ノードが生きているとbagのトピック/TFと混ざって壊れる
docker compose exec velodyne_rmd bash -c \
    "pkill -INT -f '[t]ilt_servo.py|[p]erception.py|[t]racking_controller.py|[t]ilt_node.py|[t]rajectory_viz.py' 2>/dev/null; \
     sleep 1; pkill -f '[l]ivox_ros_driver2_node|[r]os2 bag play' 2>/dev/null; true"

echo "==> trajectory_viz を起動"
docker compose exec -d velodyne_rmd bash -ic \
    "cd /workspaces/velodyne-rmd/app && python3 trajectory_viz.py --ros-args -p use_sim_time:=true > /tmp/trajectory_viz.log 2>&1"

echo "==> bag再生をrviz2の起動待ち(8秒)後に開始 (rate=$RATE ${LOOP_ARG})"
docker compose exec -d velodyne_rmd bash -ic \
    "sleep 8 && ros2 bag play '$BAG_C' --clock --rate '$RATE' $LOOP_ARG > /tmp/bag_play.log 2>&1"

echo "==> rviz2 を起動(ウィンドウを閉じると再生も停止します)"
echo "    視点を保存したい場合は rviz2 で Ctrl+S (bag2video.sh の画角にも反映)"
docker compose exec velodyne_rmd bash -ic \
    "rviz2 -d /workspaces/velodyne-rmd/config/replay_track.rviz --ros-args -p use_sim_time:=true"

docker compose exec velodyne_rmd bash -c \
    "pkill -f '[r]os2 bag play|[t]rajectory_viz.py' 2>/dev/null; true"
echo "==> 終了"
