---
name: lerobot-fullstack
description: Use when deploying LeRobot models to physical robots. Covers SO-ARM100/101 setup, calibration, real-time inference, VLA models, and edge deployment to Jetson/Raspberry Pi.
tags:
  - lerobot
  - robotics
  - deployment
  - so-arm100
  - edge
  - vla
---

# LeRobot Full Stack Deployment Guide

LeRobot モデルを実機にデプロイするためのガイド。SO-ARM100/101 のセットアップからエッジデバイスへのデプロイまで。

> **Note**: LeRobot v0.4.0 以降、プラグインシステムによるハードウェア統合が可能。`pip install` だけで新しいロボットやカメラを追加できる。

## When to Activate

- SO-ARM100/101 を組み立て・設定するとき
- ロボットのキャリブレーションを行うとき
- トレーニング済みモデルを実機で動かすとき
- Jetson/Raspberry Pi にデプロイするとき
- VLA モデル (Pi0, GR00T) をデプロイするとき

## SO-ARM100/101 Setup

### SO-ARM100 vs SO-ARM101

| 項目 | SO-ARM100 | SO-ARM101 (推奨) |
|------|-----------|------------------|
| 配線 | 従来型 | 改良版（関節3での断線問題解消） |
| 組立 | ギア取り外し必要 | ギア取り外し不要 |
| リーダーアームモーター | 標準 | 最適化ギア比 |
| 双方向制御 | フォロワー→リーダーのみ | 双方向対応 |

### 部品リスト（2アーム構成）

| 部品 | 数量 | 備考 |
|------|------|------|
| Feetech STS3215 サーボ | 12 | フォロワー6 + リーダー6 |
| SO-ARM フレーム | 2セット | 3Dプリント or 購入 |
| モーターコントロールボード | 2 | USB接続 |
| 電源アダプタ | 2 | 7.4V (標準) or 12V (高トルク) |
| USB-C ケーブル | 2 | サーボドライバ用 |
| USB カメラ | 1-2 | 推奨: 720p以上 |
| テーブルクランプ | 4 | 固定用 |

**電源選択:**
- **7.4V**: 標準構成、16.5kg.cm トルク
- **12V**: 高トルク構成、30kg.cm トルク（要12V 5A+電源）

**コスト目安（米国）**: 2アーム約$230、フォロワーのみ約$122

### リーダー/フォロワー構成

```
リーダーアーム          フォロワーアーム
(人間が操作)            (模倣動作)
    ↓                      ↓
USB-C → PC/Jetson ← USB-C
    ↓
 電源(5V)               電源(7.4V/12V)
```

- **リーダー**: 人間が手動操作、5V電源
- **フォロワー**: リーダーの動きを模倣、7.4V/12V電源

### サーボID設定

サーボにラベルを貼ってID管理（F1-F6: フォロワー、L1-L6: リーダー）:

```python
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

# 1つずつ接続してID設定
bus = FeetechMotorsBus(port="/dev/ttyUSB0", motors={})
bus.write("ID", [1], [new_id])  # new_id: 1-6
```

推奨ID配置:
- ID 1: ベース回転
- ID 2: ショルダー
- ID 3: エルボー
- ID 4: リスト回転
- ID 5: リストチルト
- ID 6: グリッパー

### Feetech STS3215 サーボパラメータ

| パラメータ | アドレス | 説明 |
|-----------|---------|------|
| ID | - | サーボ識別子 |
| ボーレート | - | 通信速度 |
| ゼロ位置補正 | 31 | 中央位置補正 |
| ワンキーキャリブ | 40 | 128を書き込みで現在位置を中央(2048)に設定 |
| PID (P/I/D) | 21-23 | モーター制御パラメータ |

**動作モード:**
- Mode 0: 位置サーボ（デフォルト、0-360度）
- Mode 1: 速度クローズドループ
- Mode 2: 速度オープンループ
- Mode 3: ステッピングモード

### ポート確認

```bash
# シリアルポート検出
python lerobot/scripts/find_motors_bus_port.py

# または手動確認
ls /dev/ttyUSB* /dev/ttyACM*
```

## Calibration

### サーボキャリブレーション（LeRobot v0.4.0+）

```bash
# フォロワーアームのキャリブレーション
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower_arm

# リーダーアームのキャリブレーション
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader_arm
```

**重要**: `--robot.id` は同じセットアップで一貫して使用すること（テレオペ、記録、評価時）

### キャリブレーション時の姿勢

キャリブレーション時はアームを特定の姿勢にする必要がある:
1. グリッパーの向きに注意
2. リーダー/フォロワーで同じ姿勢に設定
3. 詳細は公式ガイドを参照

### Feetech ワンキーキャリブレーション

サーボ単体でのゼロ位置設定:

```python
# アドレス40に128を書き込み → 現在位置を中央(2048)に設定
bus.write("Goal_Position", [40], [128])
```

### カメラキャリブレーション

```bash
# カメラ確認
v4l2-ctl --list-devices

# カメラ設定確認
v4l2-ctl -d /dev/video0 --all

# 解像度・FPS設定
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --set-parm=30
```

### キャリブレーション検証（テレオペレーション）

```bash
# LeRobot v0.4.0+ CLI
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower_arm \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader_arm

# カメラ付き
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower_arm \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader_arm \
  --display_data=true
```

動作確認:
- 各関節がスムーズに動くか
- 可動範囲が適切か
- カメラ映像が正常か
- リーダーの動きをフォロワーが正確に追従するか

## Real-time Inference

### データ収集

```bash
# LeRobot v0.4.0+ CLI
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower_arm \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=my_leader_arm \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --dataset.num_episodes=50
```

**記録中のキーボード操作:**
- **→ (右矢印)**: 現在のエピソードを終了、次へ
- **← (左矢印)**: 現在のエピソードをキャンセル、再記録
- **ESC**: セッション停止、ビデオエンコード、アップロード

### ポリシー学習

```bash
# ACT ポリシー
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=act \
  --output_dir=outputs/train

# Diffusion ポリシー
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=diffusion \
  --output_dir=outputs/train
```

### 推論実行

```bash
# 学習済みモデルで推論
lerobot-eval \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=my_follower_arm \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
  --policy.path=outputs/train/checkpoints/last.pt
```

### VLA モデル (Vision-Language-Action)

LeRobot v0.4.0 で Pi0/Pi0.5 と GR00T N1.5 をサポート:

```bash
# Pi0.5 モデル
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=pi0 \
  --policy.repo_id=lerobot/pi05_base

# GR00T N1.5 (NVIDIA)
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=gr00t \
  --policy.repo_id=nvidia/GR00T-N1.5-3B
```

**VLA モデルの特徴:**
- オープンワールド汎化能力
- 言語指示に基づく動作生成
- クロスエンボディメント転移

### 推論パラメータ調整

```python
# 推論設定
inference_config = {
    "temporal_ensemble": True,     # 時間的アンサンブル
    "temporal_ensemble_coeff": 0.01,
    "action_chunk_size": 100,      # 一度に予測するアクション数
    "n_action_steps": 100,         # 実行するアクション数
}
```

## Safety

### 緊急停止の実装

```python
import signal
import sys

class EmergencyStop:
    def __init__(self, robot):
        self.robot = robot
        self.stopped = False
        signal.signal(signal.SIGINT, self.handle_stop)
        signal.signal(signal.SIGTERM, self.handle_stop)

    def handle_stop(self, signum, frame):
        print("\n[EMERGENCY STOP] Stopping robot...")
        self.stopped = True
        # サーボトルクオフ
        self.robot.disable_torque()
        sys.exit(0)

    def check(self):
        return self.stopped
```

### 速度・トルク制限

```python
# ソフトウェアリミット
MAX_VELOCITY = 0.5  # rad/s
MAX_TORQUE = 0.8    # 0-1 スケール

def safe_action(action, max_vel=MAX_VELOCITY):
    """アクションをクリップして安全な範囲に制限"""
    return np.clip(action, -max_vel, max_vel)
```

### ワークスペース制限

```python
WORKSPACE_LIMITS = {
    'x': (-0.3, 0.3),
    'y': (-0.3, 0.3),
    'z': (0.0, 0.4),
}

def check_workspace(position):
    """ワークスペース外ならTrue"""
    for axis, (min_val, max_val) in WORKSPACE_LIMITS.items():
        if position[axis] < min_val or position[axis] > max_val:
            return True
    return False
```

### 安全チェックリスト

- [ ] 緊急停止ボタンまたはキーボードショートカット設定済み
- [ ] サーボ電流制限設定済み
- [ ] ワークスペース制限設定済み
- [ ] 最初のテストは低速で実施
- [ ] 周囲に障害物がないことを確認
- [ ] 電源の即時遮断手段を確保

## Edge Deployment

### Jetson デバイス選択

| デバイス | メモリ | 性能 | 用途 |
|---------|-------|------|------|
| Jetson Orin Nano Super | 8GB | ~67 INT8 TOPS | プロトタイプ、ACT/Diffusion |
| Jetson AGX Orin | 32-64GB | ~275 TOPS | VLAモデル、本番デプロイ |

**注意**: VLA モデル (Pi0, GR00T) は Orin Nano では重い。AGX Orin 推奨。

### Jetson Orin Nano へのデプロイ

#### 環境構築（JetPack 6.x）

```bash
# Conda インストール
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh

# 環境作成
conda create -n lerobot python=3.10 -y
conda activate lerobot

# PyTorch (JetPack 6.x 対応)
pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu124

# LeRobot インストール
pip install lerobot
```

#### jetson-containers を使用する場合

```bash
git clone https://github.com/dusty-nv/jetson-containers
cd jetson-containers

# PyTorch コンテナ起動 (JetPack 6.x)
./run.sh -v /path/to/lerobot:/lerobot dustynv/pytorch:r36.2.0

# コンテナ内でインストール
cd /lerobot
pip install -e .
```

#### 電力モード最適化

```bash
# 最大パフォーマンス
sudo nvpmodel -m 0  # MAXN
sudo jetson_clocks

# 確認
nvpmodel -q
```

#### TensorRT 最適化（オプション）

```bash
# ONNX エクスポート
python -c "
import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy

policy = ACTPolicy.from_pretrained('outputs/train/checkpoints/last.pt')
dummy_input = {
    'observation.images.top': torch.randn(1, 3, 480, 640),
    'observation.state': torch.randn(1, 6)
}
torch.onnx.export(policy, dummy_input, 'policy.onnx', opset_version=17)
"

# TensorRT 変換 (FP16)
/usr/src/tensorrt/bin/trtexec \
  --onnx=policy.onnx \
  --saveEngine=policy.trt \
  --fp16
```

#### 推論スクリプト（Jetson用）

```python
# inference_jetson.py
import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot

# GPU設定
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
torch.backends.cudnn.benchmark = True

# モデルロード
policy = ACTPolicy.from_pretrained("outputs/train/checkpoints/last.pt")
policy = policy.to(device)

# FP16で推論（メモリ節約）
policy = policy.half()

# ロボット初期化
robot = ManipulatorRobot(robot_type="so100")

# 推論ループ
with torch.no_grad(), torch.cuda.amp.autocast():
    while True:
        observation = robot.get_observation()
        # numpy -> torch -> GPU
        obs_tensor = {
            k: torch.from_numpy(v).unsqueeze(0).half().to(device)
            for k, v in observation.items()
        }
        action = policy(obs_tensor)
        robot.send_action(action.cpu().numpy()[0])
```

### Jetson でのトレーニング

Jetson Orin Nano でもトレーニング可能（時間はかかる）:

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=act \
  --output_dir=outputs/train \
  --device=cuda
```

### Raspberry Pi へのデプロイ

#### 制限事項

- GPU なし → CPU推論のみ
- メモリ制限 → 軽量モデル推奨
- 処理速度 → FPS 制限

#### 軽量化オプション

```python
# モデル量子化
import torch

policy = torch.quantization.quantize_dynamic(
    policy,
    {torch.nn.Linear},
    dtype=torch.qint8
)
```

#### Raspberry Pi 5 での推論

```python
# inference_rpi.py
import torch
import time
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot

# CPU最適化
torch.set_num_threads(4)

# モデルロード
policy = ACTPolicy.from_pretrained("outputs/train/checkpoints/last.pt")

# 推論ループ（FPS制限）
target_fps = 10
frame_time = 1.0 / target_fps

with torch.no_grad():
    while True:
        start = time.time()

        observation = robot.get_observation()
        obs_tensor = {
            k: torch.from_numpy(v).unsqueeze(0)
            for k, v in observation.items()
        }
        action = policy(obs_tensor)
        robot.send_action(action.numpy()[0])

        # FPS制御
        elapsed = time.time() - start
        if elapsed < frame_time:
            time.sleep(frame_time - elapsed)
```

## Plugin System (v0.4.0+)

### カスタムハードウェアの追加

LeRobot v0.4.0 のプラグインシステムで、`pip install` だけで新しいハードウェアを追加:

```bash
# サードパーティプラグインのインストール
pip install lerobot_teleoperator_my_awesome_teleop

# 使用
lerobot-teleoperate --teleop.type=my_awesome_teleop
```

### 利用可能なハードウェア

- **ロボット**: SO100, SO101, LeKiwi, Koch, HopeJR, OMX, EarthRover, Reachy2
- **テレオペ**: リーダーアーム, ゲームパッド, キーボード, スマートフォン (iOS/Android)
- **カメラ**: OpenCV, Intel RealSense, CSI

### スマートフォンテレオペ

```bash
# iPhoneからテレオペ
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --teleop.type=phone
```

## Integration with Other Skills

### Jetson スキルとの連携

```bash
# tegrastats でモニタリング
tegrastats --interval 1000 &

# jtop で詳細確認
pip install jetson-stats
jtop

# 電力モード最適化
sudo nvpmodel -m 0  # MAXN for inference
sudo jetson_clocks
```

### edge-common スキルとの連携

- サーボ通信: UART (シリアル)
- カメラ: USB (CSI も可)
- 電源: 7.4V/12V サーボ用、5V ボード用

## Troubleshooting

### サーボが動かない

```bash
# ポート確認
ls /dev/ttyUSB*

# 権限確認
sudo chmod 666 /dev/ttyUSB0

# または udev ルール追加
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-feetech.rules
sudo udevadm control --reload-rules
```

### カメラが認識されない

```bash
# デバイス確認
v4l2-ctl --list-devices

# 複数カメラの場合、正しい /dev/videoX を指定
# configs/robot/so100.yaml で device_id を設定
```

### 推論が遅い

- Jetson: nvpmodel を MAXN に
- Jetson: TensorRT 最適化
- RPi: FPS を下げる
- 共通: 画像解像度を下げる

### アクションがガタつく

```python
# 時間的スムージング
action_history = []
def smooth_action(action, alpha=0.7):
    action_history.append(action)
    if len(action_history) > 5:
        action_history.pop(0)
    return sum(action_history) / len(action_history)
```

## Command Reference

### LeRobot v0.4.0+ CLI

| コマンド | 説明 |
|---------|------|
| `lerobot-calibrate` | キャリブレーション |
| `lerobot-teleoperate` | テレオペレーション |
| `lerobot-record` | データ収集 |
| `lerobot-train` | ポリシー学習 |
| `lerobot-eval` | モデル評価・推論 |
| `lerobot-edit-dataset` | データセット編集（マージ、削除等） |

### レガシーコマンド（v0.3.x以前）

| コマンド | 説明 |
|---------|------|
| `control_robot.py calibrate` | キャリブレーション |
| `control_robot.py teleoperate` | テレオペレーション |
| `control_robot.py record` | データ収集 |

## Resources

### 公式ドキュメント
- [LeRobot Documentation](https://huggingface.co/docs/lerobot/index)
- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [Getting Started with Real-World Robots](https://huggingface.co/docs/lerobot/en/getting_started_real_world_robot)
- [Bring Your Own Hardware](https://huggingface.co/docs/lerobot/en/integrate_hardware)

### ハードウェア
- [SO-ARM100/101 GitHub](https://github.com/TheRobotStudio/SO-ARM100)
- [Feetech サーボ公式](https://www.feetechrc.com/)
- [Seeed Studio Wiki - SO-ARM in LeRobot](https://wiki.seeedstudio.com/lerobot_so100m/)
- [Waveshare Wiki - ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo)

### VLA モデル
- [Pi0/Pi0.5 Models](https://huggingface.co/lerobot)
- [GR00T N1.5](https://huggingface.co/nvidia/GR00T-N1.5-3B)
- [LeRobot v0.4.0 Release Blog](https://huggingface.co/blog/lerobot-release-v040)

### 学習リソース
- [Hugging Face Robot Learning Course](https://huggingface.co/spaces/lerobot/robot-learning-tutorial)
- [LeRobot Discord Community](https://discord.gg/lerobot)
