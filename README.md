# Velodyne VLP-16 + Livox Mid-360 + RMD-X8 Pro チルト機構

VLP-16(天面にMid-360)をRMD-X8 Proで上下に掃引し、垂直測定範囲を拡張する
デモ環境(ROS 2 Jazzy + Docker)。モータ実測角からTFを配信するため、掃引中も
rviz2上で環境は静止したまま点群が蓄積される。センサはIP疎通が取れたものだけ
起動する(片方のみでも可)。

## 使い方

```bash
./scripts/start.sh   # コンテナ → ドライバ → チルト掃引 → rviz2
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

## 構成

```
app/rmd_can.py             RMDプロトコル(SocketCAN, 標準ライブラリのみ)
app/jog.py                 ジョグCLI(過電流ガード+自動退避)
app/tilt_node.py           掃引 + TF配信ノード
app/verify_sweep.py        掃引補償の定量検証
config/limits.yaml         較正結果 + mounts(取付オフセット)
config/MID360_config.json  livox_ros_driver2用ネットワーク設定
config/vlp16_tilt.rviz     Fixed Frame=base_link, Decay Time=6s
scripts/                   start / stop / jog ラッパ
```

TF: `base_link → tilt_link`(動的、モータ角/gear_ratio)`→ velodyne / livox_frame`
(静的、limits.yamlのmounts:)。点群はヘッダ時刻のTFでbase_link系に変換される。

取付オフセットの根拠、Mid-360のネットワーク/ドライバ設定、RMDファームウェアの
挙動などの詳細は [docs/hardware.md](docs/hardware.md) を参照。
