#!/usr/bin/env bash
# bag2video.sh のコンテナ内本体。仮想ディスプレイ(Xvfb)上でrviz2を起動し、
# bag再生の画面をffmpegでmp4録画する。直接呼ばず scripts/bag2video.sh を使うこと。
set -euo pipefail

BAG="$1"        # コンテナ内のbagディレクトリパス
OUT="$2"        # コンテナ内の出力mp4パス
RATE="${3:-1.0}"

# ROSのsetup.bashは未定義変数を参照するので-uを一時解除
set +u
source /opt/ros/jazzy/setup.bash
set -u
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1

cleanup() {
    kill "${FF_PID:-}" "${RVIZ_PID:-}" "${VIZ_PID:-}" "${XVFB_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp -ac > /tmp/xvfb.log 2>&1 &
XVFB_PID=$!
for _ in $(seq 50); do
    xdpyinfo > /dev/null 2>&1 && break
    sleep 0.2
done
xdpyinfo > /dev/null 2>&1 || { echo "ERROR: Xvfbが起動しません (/tmp/xvfb.log)"; exit 1; }

python3 /workspaces/velodyne-rmd/app/trajectory_viz.py \
    --ros-args -p use_sim_time:=true > /tmp/trajectory_viz.log 2>&1 &
VIZ_PID=$!

rviz2 -d /workspaces/velodyne-rmd/config/replay_track.rviz \
    --ros-args -p use_sim_time:=true > /tmp/rviz_replay.log 2>&1 &
RVIZ_PID=$!

echo "==> rviz2の読み込みを待機中..."
sleep 10
kill -0 "$RVIZ_PID" 2>/dev/null || { echo "ERROR: rviz2が落ちました (/tmp/rviz_replay.log)"; exit 1; }

echo "==> 録画開始 → $OUT"
# cropはメニューバー/ツールバー/ステータスバーを除いて3Dビューだけを残す
ffmpeg -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1080 -i :99 \
    -vf "crop=1888:988:16:60" \
    -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart \
    -y "$OUT" &
FF_PID=$!

echo "==> bag再生 (rate=$RATE)"
ros2 bag play "$BAG" --clock --rate "$RATE"

sleep 2
kill -INT "$FF_PID"
wait "$FF_PID" 2>/dev/null || true
FF_PID=""
echo "==> 録画終了"
