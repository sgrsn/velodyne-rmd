# Velodyne VLP-16 + Livox Mid-360 + RMD-X8 Pro チルト機構

<img width="4240" height="3944" alt="IMG_9165" src="https://github.com/user-attachments/assets/c28bfd71-86df-4b10-9001-3c10e0d740e9" />

VLP-16(天面にMid-360)をRMD-X8 Proで上下に掃引し、垂直測定範囲を拡張する
デモ環境(ROS 2 Jazzy + Docker)。モータ実測角からTFを配信するため、掃引中も
rviz2上で環境は静止したまま点群が蓄積される。センサはIP疎通が取れたものだけ
起動する(片方のみでも可)。

## 使い方

```bash
./scripts/start.sh   # コンテナ → ドライバ → チルト掃引 → rviz2
./scripts/track.sh   # ドローン追跡デモ(Mid-360 + 速度制御チルト)
./scripts/stop.sh    # 停止(通電保持)。--release で脱力
```

前提:

- `can0` が1Mbpsでup、センサのネットワーク設定済み
  (`~/workspace/velodyne_demo/scripts/setup_network.sh`)。
- **モータ電源投入時はセンサをだいたい水平に**(±25°程度でOK)。水平は
  単回転エンコーダから自動復元される。ズレがロータ±120°を超えると起動を
  拒否する。`./scripts/jog.sh level` で較正済み水平位置へ移動できる。

## 較正(初回または機構変更時のみ)

限界は水平からの相対角で保存されるため、電源再投入だけなら再較正不要。

```bash
docker compose up -d
./scripts/jog.sh status      # 疎通確認(角度・温度・エラー)
./scripts/jog.sh release     # 脱力(手で位置決め可)
./scripts/jog.sh to <angle>  # 絶対角度へ移動(ガード付き)
```

水平・上限・下限の各位置で `status` を読み、`config/limits.yaml` に記入する
(フィールドの意味は同ファイルのコメント参照)。

**注意:** 脱力中に手で速く動かすと多回転カウンタが±360°単位で狂う。
ゆっくり動かし、`status` のencoder値(単回転絶対値)とクロスチェックすること。

## ドローン追跡(track.sh)

Mid-360の点群からドローンを検出・追跡し、チルトを速度制御で追従させる
3ノード構成:

```
perception          背景ボクセル差分 → クラスタリング → 等速KF追跡
                    → /perception/target (Odometry: 位置・速度・共分散)
tracking_controller 状態機械 MAP/SEARCH/TRACK/LOST、look-at変換
                    → /tilt/cmd (目標ピッチ角 + 角速度FF)
tilt_servo          100Hz速度制御ループ(0xA2)、可動域・過電流ガード、TF配信
```

起動後、背景取得掃引(約1分)→ 探索掃引 → 目標検出でロックオンの順に遷移する。
オフライン開発: `ros2 bag record` した掃引データを `--clock` 再生し、
`perception.py --ros-args -p use_sim_time:=true` で調整できる(bags/参照)。

## 構成

```
app/rmd_can.py             RMDプロトコル(SocketCAN, 標準ライブラリのみ)
app/jog.py                 ジョグCLI(過電流ガード+自動退避)
app/tilt_node.py           掃引 + TF配信ノード
app/tilt_servo.py          速度制御サーボノード(追跡用、tilt/cmd受け)
app/perception.py          背景差分+KF追跡ノード
app/tracking_controller.py 追跡状態機械ノード
app/verify_sweep.py        掃引補償の定量検証
app/probe_speed.py         0xA2速度制御の実測プローブ
app/test_servo_tracking.py tilt_servo追従帯域の実測テスト
app/test_controller_logic.py 状態機械ロジックテスト(HW不要)
config/limits.yaml         較正結果 + mounts(取付オフセット)
config/MID360_config.json  livox_ros_driver2用ネットワーク設定
config/vlp16_tilt.rviz     Fixed Frame=base_link, Decay Time=6s
scripts/                   start / track / stop / jog ラッパ
```

TF: `base_link → tilt_link`(動的、モータ角/gear_ratio)`→ velodyne / livox_frame`
(静的、limits.yamlのmounts:)。点群はヘッダ時刻のTFでbase_link系に変換される。

取付オフセットの根拠、Mid-360のネットワーク/ドライバ設定、RMDファームウェアの
挙動などの詳細は [docs/hardware.md](docs/hardware.md) を参照。
