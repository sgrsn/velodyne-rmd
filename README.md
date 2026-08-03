# Velodyne VLP-16 + RMD-X8 Pro チルト機構

VLP-16をRMD-X8 Proで上下に掃引し、垂直測定範囲を約±65°に拡張するデモ環境
(ROS 2 Jazzy + Docker)。モータ実測角からTFを配信するため、掃引中も
rviz2上で環境は静止したまま点群が蓄積される。

## 使い方

```bash
./scripts/start.sh   # コンテナ → velodyneドライバ → チルト掃引 → rviz2
./scripts/stop.sh    # 停止(通電保持)。--release で脱力
```

前提:

- `can0` が1Mbpsでup、VLP-16のネットワーク設定済み
  (`~/workspace/velodyne_demo/scripts/setup_network.sh`)。
- **モータ電源投入時はセンサをだいたい水平にしておくこと**(±25°程度でOK)。
  厳密な水平は単回転エンコーダから自動復元される。ズレがロータ±120°を
  超えると誤復元防止のため起動を拒否する。`./scripts/jog.sh level` で
  較正済み水平位置へ移動できる。

## 較正 — 初回または機構変更時のみ

限界は水平からの相対角で保存されるため、電源再投入だけなら再較正不要。

```bash
docker compose up -d
./scripts/jog.sh status      # 疎通確認(角度・温度・エラー)
./scripts/jog.sh release     # 脱力(手で位置決め可)
./scripts/jog.sh to <angle>  # 絶対角度へ移動(ガード付き)
```

水平・上限・下限の各位置で `status` を読み、`config/limits.yaml` に記入する
(各フィールドの意味は同ファイルのコメント参照)。

**注意:** 脱力中に手で速く動かすと多回転カウンタが±360°単位で狂う。
ゆっくり動かし、各マークで `status` のencoder値(単回転絶対値、常に信頼可)と
クロスチェックすること。

## 構成

```
app/rmd_can.py           RMDプロトコル(SocketCAN, 標準ライブラリのみ)
app/jog.py               ジョグCLI(過電流ガード+自動退避)
app/tilt_node.py         掃引 + TF配信ノード
config/limits.yaml       較正結果(2026-08-03: 出力軸 約−52°〜+49°)
config/vlp16_tilt.rviz   Fixed Frame=base_link, Decay Time=6s
scripts/                 start / stop / jog ラッパ
```

TF: `base_link → tilt_link`(動的、モータ角/gear_ratio)`→ velodyne`(静的)。
点群はヘッダ時刻のTFでbase_link系に変換される。

## ハードウェアメモ(RMD-X8 Pro, レガシーLK系ファームウェア)

- CAN ID `0x141`、1Mbps。V3系コマンド(0x42等)は不応答。
- 0x92/0xA4の角度は**ロータ角**(出力軸角 = /6.2)。0x92はDATA[1..7]の
  符号付き56bit LE、0.01deg/LSB。
- 多回転基準は電源断で失われるが、単回転エンコーダ(0x9C DATA[6..7]、
  65536/回転)は絶対値。水平のencoder値を記録しておけば±180°(ロータ)
  以内から水平を一意に復元できる。
- 0xA4の速度制限はソフト。25dps以下なら指令にほぼ一致、40dps以上では
  最大+60%超過する。
