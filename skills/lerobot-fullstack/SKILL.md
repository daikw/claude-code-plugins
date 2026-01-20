---
name: lerobot-fullstack
description: Use when deploying LeRobot models to physical robots. Covers SO-ARM100 setup, calibration, real-time inference, and edge deployment to Jetson/Raspberry Pi.
tags:
  - lerobot
  - robotics
  - deployment
  - so-arm100
  - edge
---

# LeRobot Full Stack Deployment Guide

LeRobot モデルを実機にデプロイするためのガイド。SO-ARM100 のセットアップからエッジデバイスへのデプロイまで。

## When to Activate

- SO-ARM100 を組み立て・設定するとき
- ロボットのキャリブレーションを行うとき
- トレーニング済みモデルを実機で動かすとき
- Jetson/Raspberry Pi にデプロイするとき

## SO-ARM100 Setup

### 部品リスト

| 部品 | 数量 | 備考 |
|------|------|------|
| Feetech STS3215 サーボ | 6 | または STS3032 |
| SO-ARM100 フレーム | 1セット | 3Dプリント or 購入 |
| Feetech サーボドライバ | 1 | USB接続 |
| 電源 (7.4V 2A以上) | 1 | サーボ用 |
| USB ケーブル | 2 | サーボドライバ + カメラ |
| USB カメラ | 1-2 | 推奨: 720p以上 |

### サーボ接続

```
電源(7.4V) → サーボドライバ → サーボ (デイジーチェーン)
                ↓
              USB
                ↓
             PC/Jetson
```

### サーボID設定

各サーボに固有のIDを設定（工場出荷時は全て同じID）:

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

### ロボット設定ファイル

```yaml
# configs/robot/so100.yaml
robot_type: so100
leader_arms:
  main:
    port: /dev/ttyUSB0
    motors:
      shoulder_pan: [1, feetech]
      shoulder_lift: [2, feetech]
      elbow_flex: [3, feetech]
      wrist_roll: [4, feetech]
      wrist_flex: [5, feetech]
      gripper: [6, feetech]

cameras:
  top:
    fps: 30
    width: 640
    height: 480
```

## Calibration

### サーボキャリブレーション

```bash
python lerobot/scripts/control_robot.py calibrate \
  --robot-path lerobot/configs/robot/so100.yaml
```

キャリブレーション手順:
1. 各関節を可動範囲の両端まで動かす
2. 中央位置（ホームポジション）を設定
3. 結果は `~/.cache/huggingface/lerobot/calibration/` に保存

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

### キャリブレーション検証

```bash
python lerobot/scripts/control_robot.py teleoperate \
  --robot-path lerobot/configs/robot/so100.yaml \
  --fps 30
```

動作確認:
- 各関節がスムーズに動くか
- 可動範囲が適切か
- カメラ映像が正常か

## Real-time Inference

### 推論実行

```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/so100.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/so100_test \
  -p outputs/train/checkpoints/last.pt \
  --warmup-time-s 5 \
  --episode-time-s 60 \
  --num-episodes 10
```

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

### 安全対策

```yaml
# configs/robot/so100.yaml に追加
safety:
  max_velocity: 0.5        # 最大速度制限
  max_acceleration: 1.0    # 最大加速度制限
  workspace_limits:        # ワークスペース制限
    x: [-0.3, 0.3]
    y: [-0.3, 0.3]
    z: [0.0, 0.4]
```

## Edge Deployment

### Jetson へのデプロイ

#### 環境構築

```bash
# jetson-containers を使用
git clone https://github.com/dusty-nv/jetson-containers
cd jetson-containers

# PyTorch コンテナ起動
./run.sh -v /path/to/lerobot:/lerobot dustynv/pytorch:r35.4.1

# コンテナ内でインストール
cd /lerobot
pip install -e .
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
torch.onnx.export(policy, dummy_input, 'policy.onnx', opset_version=11)
"

# TensorRT 変換
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

## Integration with Other Skills

### Jetson スキルとの連携

```bash
# tegrastats でモニタリング
tegrastats --interval 1000 &

# jtop で詳細確認
jtop

# 電力モード最適化
sudo nvpmodel -m 0  # MAXN for inference
```

### edge-common スキルとの連携

- サーボ通信: UART (シリアル)
- カメラ: USB (CSI も可)
- 電源: 7.4V サーボ用、5V ボード用

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

| コマンド | 説明 |
|---------|------|
| `control_robot.py calibrate` | キャリブレーション |
| `control_robot.py teleoperate` | テレオペレーション |
| `control_robot.py record` | データ収集 |
| `control_robot.py record -p model.pt` | モデル推論 |

## Resources

- [SO-ARM100 GitHub](https://github.com/TheRobotStudio/SO-ARM100)
- [Feetech サーボ データシート](https://www.feetechrc.com/)
- [LeRobot Deployment Guide](https://github.com/huggingface/lerobot/blob/main/examples/10_use_so100.md)
