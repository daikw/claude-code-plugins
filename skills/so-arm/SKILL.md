---
name: so-arm
description: Use when working with SO-ARM100/101 robot arms. Covers hardware overview, assembly, servo configuration, wiring, calibration with LeRobot, and troubleshooting.
tags:
  - robotics
  - so-arm
  - lerobot
  - feetech
  - servo
---

# SO-ARM100/101 Robot Arm Guide

SO-ARM100/101 ロボットアームのセットアップから運用までをサポート。TheRobotStudio設計、HuggingFace LeRobot対応。

## When to Activate

- SO-ARM100/101 を組み立てるとき
- Feetech STS3215 サーボの設定をするとき
- Leader/Follower アームのテレオペレーションを構築するとき
- LeRobot でキャリブレーション・データ収集するとき
- 通信エラーやトルク問題をトラブルシューティングするとき

## Hardware Overview

### SO-ARM100 vs SO-ARM101

| 項目 | SO-ARM100 | SO-ARM101 |
|------|-----------|-----------|
| 設計 | 初代モデル | 改良版（2025年） |
| 配線 | 関節3で断線リスクあり | 改良された配線設計 |
| 組立 | ギア取り外し必要 | ギア取り外し不要 |
| ケーブルクリップ | なし | あり |
| Follower トルク | 7.4V: 19.5kg.cm | 12V: 30kg.cm (Pro版) |
| Leader ギア比 | 標準 | 最適化済み |
| 双方向追従 | なし | あり（ポリシー介入対応） |

### バリエーション

| キット | Follower電源 | Leader電源 | トルク | 用途 |
|--------|-------------|-----------|--------|------|
| Standard (7.4V) | 5V | 5V | 19.5kg.cm | 教育・軽量タスク |
| Pro (12V) | 12V | 5V | 30kg.cm | 高負荷アプリケーション |

### 仕様

- **自由度**: 6-DOF（6軸）
- **サーボ**: Feetech STS3215 x 6（各アーム）
- **エンコーダ**: 12bit磁気エンコーダ内蔵
- **通信**: TTLシリアルバス（デイジーチェーン接続）
- **位置分解能**: 4096ステップ/回転（0.088°）

## Parts List

### 1アーム分（Leader または Follower）

| 部品 | 数量 | 備考 |
|------|------|------|
| STS3215 サーボ (7.4V) | 6 | Follower用（または12V Pro版） |
| STS3215 サーボ (7.4V) | 6 | Leader用（常に7.4V） |
| バスサーボアダプタボード | 1 | FE-URT-1 等 |
| 電源アダプタ 5V 3A+ | 1 | 7.4Vサーボ用 |
| 電源アダプタ 12V 5A+ | 1 | Pro版Follower用のみ |
| サーボ接続ケーブル | 5 | 内部配線用 |
| USB Type-C ケーブル | 1 | PC接続用 |
| 3Dプリントパーツ一式 | 1 | 別売りまたは自作 |

### テレオペレーションセット（2アーム）

- 上記部品 x 2セット
- Follower: 7.4V or 12V (Pro)
- Leader: 7.4V（ギア比最適化版）

## Assembly

### 組立前の準備

1. **サーボIDの事前設定を推奨**（組立後は配線が困難）
2. **ラベリング**: F1-F6（Follower）、L1-L6（Leader）と記載
3. **ケーブルの事前挿入**: 組立後よりも事前挿入が容易

### サーボID設定（組立前に実施）

```bash
# LeRobot環境を有効化
conda activate lerobot
cd ~/lerobot

# サーボIDを1に設定（モーター1つだけ接続した状態で）
python lerobot/scripts/configure_motor.py \
  --port /dev/ttyACM0 \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 1

# ID 2-6 も同様に設定（1つずつ接続して実行）
python lerobot/scripts/configure_motor.py \
  --port /dev/ttyACM0 \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 2
```

サーボIDは0-253の範囲で設定可能。設定後、位置は2048（中間点）にリセットされる。

### 組立手順（概要）

1. **ベース（F1/L1）**
   - サーボNo.1を両ポートにケーブル接続
   - ベースに挿入、4本のネジで固定（上下各2本）

2. **肩（F2/L2）**
   - 肩ブラケットをサーボNo.2に取付
   - サーボNo.1からのケーブルを接続
   - 両端のネジで固定

3. **上腕（F3/L3）**
   - ケーブルをホルダー経由でサーボNo.3へ配線
   - 4本のネジで固定
   - **SO-ARM100注意**: この関節で断線リスクあり

4. **前腕（F4/L4）**
   - サーボNo.4をスライドイン
   - サーボNo.3からのケーブル接続
   - ケーブルホルダーにネジで固定

5. **手首（F5/L5）**
   - サーボNo.5を挿入
   - ケーブル配線

6. **グリッパー（F6/L6）**
   - グリッパーモーターを挿入
   - サーボNo.5からのケーブル接続
   - 両側3本ずつのネジで固定

### SO-ARM101 の追加作業

- ケーブルクリップの取付（配線整理用）
- ワイヤーグルーブは美観用（なくても可）

## Wiring

### 配線図

```
[PC] --USB-- [バスサーボアダプタ] --電源-- [電源アダプタ]
                    |
                    +--TTL-- [サーボ1] --TTL-- [サーボ2] --...-- [サーボ6]
```

### 電源接続

| キット | Follower電源 | Leader電源 |
|--------|-------------|-----------|
| Standard | 5V | 5V |
| Pro | 12V | 5V |

**重要**: Leader アームは常に5V電源を使用（7.4Vサーボのため）

### USB接続

```bash
# ポート確認
ls /dev/ttyACM*
# または
ls /dev/ttyUSB*

# パーミッション設定
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

### デバイスの識別

- Follower: `/dev/ttyACM0` または `/dev/ttyACM1`
- Leader: もう一方のポート

確認方法: 片方ずつ接続して `ls /dev/ttyACM*` で特定

## Leader/Follower Setup

### Leader アームの準備（SO-ARM100のみ）

SO-ARM100 の Leader アームでは、モーターのギアを取り外す必要がある。これにより位置エンコーディングのみ使用し、摩擦を低減して操作しやすくなる。

SO-ARM101 ではギア取り外し不要（ギア比が最適化済み）。

### テレオペレーション構成

```
[Leader Arm]                    [Follower Arm]
     |                               |
[バスサーボアダプタ]           [バスサーボアダプタ]
     |                               |
[5V電源]                       [5V/12V電源]
     |                               |
     +------- [PC] --------+
              |
         [LeRobot]
```

### 物理的な設置

- 両アームをクランプでしっかり固定（転倒防止）
- 作業スペースを確保
- カメラを使用する場合は適切な位置に配置

## Calibration with LeRobot

### キャリブレーションの目的

Leader/Follower アームが同じ物理位置で同じ位置値を持つようにする。これにより、異なるSO-10xロボット間でニューラルネットワークの転移が可能になる。

### 環境準備

```bash
# LeRobot環境を有効化
conda activate lerobot
cd ~/lerobot

# ポート確認
ls /dev/ttyACM*

# パーミッション設定
sudo chmod 666 /dev/ttyACM*
```

### Follower アームのキャリブレーション

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_awesome_follower_arm
```

**SO-ARM100の場合**: `--robot.type=so100_follower` を使用

### Leader アームのキャリブレーション

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_awesome_leader_arm
```

**SO-ARM100の場合**: `--teleop.type=so100_leader` を使用

### キャリブレーション手順

1. **中間位置への移動**
   - 全関節を可動範囲の中間位置に移動
   - Enter キーを押して確定

2. **可動範囲の記録**
   - 各関節をフルレンジで動かす
   - システムがMIN/MAX値を記録

3. **確認**
   - 各サーボの中間位置が2048（±50の許容範囲）であることを確認

### キャリブレーションデータの保存場所

- ロボット: `~/.cache/huggingface/lerobot/calibration/robots/`
- テレオペレーター: `~/.cache/huggingface/lerobot/calibration/teleoperators/`

### 再キャリブレーション

```bash
# キャリブレーションファイルを削除して再実行
rm -rf ~/.cache/huggingface/lerobot/calibration/robots/my_awesome_follower_arm
rm -rf ~/.cache/huggingface/lerobot/calibration/teleoperators/my_awesome_leader_arm
```

## Teleoperation

### テレオペレーションの実行

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_awesome_follower_arm \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_awesome_leader_arm
```

`Ctrl+C` で終了。

### データ収集

```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/so101.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/so101_dataset \
  --tags so101 manipulation \
  --warmup-time-s 5 \
  --episode-time-s 60 \
  --reset-time-s 10 \
  --num-episodes 50
```

## Servo Configuration

### STS3215 仕様

| 項目 | 7.4V版 | 12V版 |
|------|--------|-------|
| ストールトルク | 19.5kg.cm | 30kg.cm |
| 動作電圧 | 6-8.4V | 9-12.6V |
| 通信速度 | 38400-1Mbps | 38400-1Mbps |
| 位置分解能 | 4096 (12bit) | 4096 (12bit) |
| ギア比 | 1:345 | 1:345 |
| バックラッシュ | ≤0.5° | ≤0.5° |

### 動作モード

| モード | 説明 |
|--------|------|
| 0: Position Servo | 0-360°絶対位置制御（デフォルト） |
| 1: Speed Closed-loop | 負荷下で速度維持 |
| 2: Speed Open-loop | 負荷で速度低下 |
| 3: Stepping Mode | 相対位置移動 |
| Multi-Loop | ±7回転の高精度位置制御 |

### 保護機能

| 保護 | 条件 | 動作 |
|------|------|------|
| 過負荷 | ストールトルクの80%超、25ms以上 | 新しい位置コマンドで解除 |
| 過電流 | 2A超、2秒以上 | 出力無効化 |
| 過電圧 | <4V または >8V | 電圧正常化で自動復帰 |
| 過熱 | >70°C | トルク出力無効化 |

### 位置値

- 範囲: 0-4096（360°回転に対応）
- 中間位置: 2048
- 可動範囲: 中間から±2048ステップ（±180°）

### 中間位置のワンキーキャリブレーション

アドレス40に128を書き込むと、現在位置を中間位置として設定可能。

## Troubleshooting

### 通信エラー

**症状**: サーボが応答しない、読み取りエラー

**対処法**:

```bash
# 1. ポートのパーミッション確認
sudo chmod 666 /dev/ttyACM0

# 2. ポートの存在確認
ls /dev/ttyACM*

# 3. 他のプロセスがポートを使用していないか確認
sudo lsof /dev/ttyACM0

# 4. ボーレートの確認（デフォルト: 1000000）
# configure_motor.py で --baudrate オプションを調整

# 5. ケーブル接続の確認
# デイジーチェーンが正しく接続されているか確認
```

### トルク有効化でハング

**症状**: `"Activating torque on main follower arm."` でプログラムがハング

**対処法**:

1. 電源供給の確認（十分なアンペア数か）
2. Pro版は12V電源を使用しているか確認
3. サーボIDが正しく設定されているか確認（1-6）
4. ケーブル断線がないか確認

### トルク不足

**症状**: アームが負荷に耐えられない、動きが弱い

**対処法**:

1. **Standard版**: 12V Pro版へのアップグレードを検討
2. 電源アダプタの出力確認（5V 3A以上、12V 5A以上）
3. 電源ケーブルの接触確認

### SO-ARM100 関節3での断線

**症状**: 関節3付近でサーボが認識されなくなる

**対処法**:

1. ケーブルの状態確認・交換
2. SO-ARM101へのアップグレードを検討（配線改良済み）
3. ケーブルに余裕を持たせて配線

### キャリブレーション失敗

**症状**: キャリブレーション中にエラー、位置値が異常

**対処法**:

```bash
# キャリブレーションファイルを削除
rm -rf ~/.cache/huggingface/lerobot/calibration/

# サーボの中間位置確認（2048±50が正常）
# configure_motor.py で位置をリセット

# 再キャリブレーション
lerobot-calibrate --robot.type=so101_follower ...
```

### サーボID重複

**症状**: 複数サーボが同時に反応、不正な動作

**対処法**:

1. **1つずつサーボを接続してID設定**
2. すべてのサーボのIDが1-6でユニークか確認
3. IDラベルを物理的にサーボに貼付

### 温度上昇

**症状**: サーボが熱くなる、過熱保護で停止

**対処法**:

1. 連続動作時間を制限
2. 負荷を軽減
3. 冷却時間を設ける
4. 過熱保護閾値の確認（デフォルト: 70°C）

## Command Reference

| コマンド | 説明 |
|---------|------|
| `ls /dev/ttyACM*` | シリアルポート確認 |
| `sudo chmod 666 /dev/ttyACM*` | パーミッション設定 |
| `python configure_motor.py --ID N` | サーボID設定 |
| `lerobot-calibrate` | キャリブレーション実行 |
| `lerobot-teleoperate` | テレオペレーション実行 |
| `python control_robot.py record` | データ収集 |

## Resources

- [TheRobotStudio/SO-ARM100 GitHub](https://github.com/TheRobotStudio/SO-ARM100)
- [HuggingFace LeRobot SO-100 Docs](https://huggingface.co/docs/lerobot/so100)
- [Seeed Studio Wiki - SoArm in LeRobot](https://wiki.seeedstudio.com/lerobot_so100m_new/)
- [Waveshare Wiki - SO-ARM100/101](https://www.waveshare.com/wiki/SO-ARM100/101)
- [Feetech STS3215 Datasheet](https://www.feetechrc.com/2020-05-13_56655.html)
- [LeRobot GitHub](https://github.com/huggingface/lerobot)
