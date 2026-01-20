---
name: lerobot-basics
description: Use when training robot manipulation models with HuggingFace LeRobot. Covers installation, dataset creation, training workflows, and fine-tuning.
tags:
  - lerobot
  - robotics
  - imitation-learning
  - huggingface
---

# LeRobot Training Guide

HuggingFace LeRobot を使用したロボット操作モデルのトレーニング。インストールからファインチューニングまでのワークフロー。

## When to Activate

- LeRobot でモデルをトレーニングするとき
- カスタムデータセットを作成するとき
- 事前学習モデルをファインチューニングするとき
- 評価メトリクスを解釈するとき
- VLAモデル（Pi0, SmolVLA等）を使用するとき

## Installation

### 基本インストール

```bash
# PyPIからインストール（推奨）
pip install lerobot

# または開発版（最新機能）
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[dev]"
```

### GPU環境（CUDA）

```bash
# PyTorch with CUDA
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# または conda
conda install pytorch torchvision pytorch-cuda=11.8 -c pytorch -c nvidia
```

### 依存関係確認

```bash
# バージョン確認
python -c "import lerobot; print(lerobot.__version__)"
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

## Dataset Structure (v3.0)

LeRobotDataset v3.0 (2025年9月リリース) は大規模データセット向けに最適化された新形式。

### ディレクトリ構造

```
dataset/
├── meta/
│   ├── info.json           # データセットメタ情報
│   ├── episodes.jsonl      # エピソード情報（リレーショナル）
│   └── stats.json          # 統計情報
├── data/
│   ├── train-00000-of-00001.parquet  # 複数エピソードを1ファイルに格納
│   └── ...
└── videos/
    ├── observation.images.top/
    │   ├── train-00000-of-00001.mp4  # 複数エピソードを1ファイルに格納
    │   └── ...
    └── ...
```

### v3.0 の主な変更点

| 項目 | v2.x | v3.0 |
|------|------|------|
| ファイル構成 | 1エピソード=1ファイル | 複数エピソード=1ファイル |
| メタデータ | ファイル名ベース | リレーショナルメタデータ |
| Hub連携 | ダウンロード必須 | StreamingLeRobotDataset対応 |
| スケーラビリティ | 数千エピソードまで | 数百万エピソード対応 |

### info.json の構造

```json
{
  "codebase_version": "v3.0",
  "robot_type": "so101",
  "fps": 30,
  "features": {
    "observation.state": {
      "dtype": "float32",
      "shape": [6],
      "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    },
    "observation.images.top": {
      "dtype": "video",
      "shape": [480, 640, 3],
      "video_info": {
        "video.fps": 30,
        "video.codec": "av1"
      }
    },
    "action": {
      "dtype": "float32",
      "shape": [6],
      "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    }
  }
}
```

### データセットの使用

```python
from lerobot.datasets import LeRobotDataset

# Hubから読み込み（自動ダウンロード）
dataset = LeRobotDataset("lerobot/aloha_static_coffee")

# ストリーミングモード（ダウンロード不要）
from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset
dataset = StreamingLeRobotDataset("lerobot/aloha_static_coffee")

# delta_timestamps: 時間ウィンドウで複数フレーム取得
dataset = LeRobotDataset(
    "lerobot/aloha_static_coffee",
    delta_timestamps={
        "observation.image": [-1.0, -0.5, -0.2, 0],  # 過去3フレーム + 現在
        "action": [0, 0.1, 0.2]  # 現在 + 未来2フレーム
    }
)
```

## Data Collection

### サポートされるハードウェア

- **アーム**: SO100, SO101, Koch, OMX, OpenARM, Aloha
- **モバイル**: LeKiwi, EarthRover, HopeJR
- **ヒューマノイド**: Reachy2, Unitree G1
- **入力デバイス**: ゲームパッド、キーボード、スマートフォン

### テレオペレーションでのデータ収集

```bash
# SO-101 の場合
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/so101.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/so101_test \
  --tags so101 tutorial \
  --warmup-time-s 5 \
  --episode-time-s 60 \
  --reset-time-s 10 \
  --num-episodes 50
```

### 収集のTips

- **一貫性**: 同じタスクを同じ方法で繰り返す
- **多様性**: 開始位置、物体配置を少しずつ変える
- **品質**: 失敗したエピソードは除外
- **量**: 最低50エピソード、理想は100以上

### データセットのHugging Faceへのアップロード

```bash
huggingface-cli login

python lerobot/scripts/push_dataset_to_hub.py \
  --raw-dir data/raw/so101_test \
  --repo-id ${HF_USER}/so101_test \
  --raw-format lerobot
```

## Policies

### 利用可能なポリシー（v0.4.0+）

| カテゴリ | ポリシー | 説明 | パラメータ数 |
|---------|---------|------|-------------|
| **模倣学習** | ACT | Action Chunking with Transformers。高精度操作向け | ~80M |
| | Diffusion | Diffusion Policy。複雑なタスク向け | ~100M |
| | VQ-BeT | Vector-Quantized BeT。離散アクション向け | ~50M |
| **強化学習** | HIL-SERL | Human-in-the-Loop Sample-Efficient RL | - |
| | TDMPC | Temporal Difference MPC。オンライン適応 | - |
| **VLAモデル** | Pi0Fast | π0 with FAST tokenizer。オートレグレッシブ | ~3B |
| | Pi0.5 | 汎化性能特化。新環境への適応 | ~3B |
| | SmolVLA | 軽量VLA。リソース効率重視 | ~450M |
| | GR00T N1.5 | NVIDIAの汎用VLA | ~2B |
| | XVLA | クロスエンボディメントVLA | ~1B |

### ポリシーの選択基準

| ユースケース | 推奨ポリシー |
|-------------|-------------|
| 単純な操作タスク、少ないデータ | ACT |
| 複雑なタスク、高精度要求 | Diffusion |
| リアルタイム推論、エッジデバイス | SmolVLA |
| 新環境への汎化が必要 | Pi0.5 |
| インタラクティブな学習 | HIL-SERL |

## Training

### CLI コマンド（推奨）

```bash
# ACTポリシーでトレーニング
lerobot-train \
  --dataset.repo_id=${HF_USER}/so101_test \
  --policy.type=act \
  --output_dir=outputs/train/act_so101 \
  --job_name=act_so101_test \
  --policy.device=cuda \
  --wandb.enable=true
```

### 環境指定でのトレーニング

```bash
# PushT環境でDiffusion Policyをトレーニング
lerobot-train \
  --output_dir=outputs/train/diffusion_pusht \
  --policy.type=diffusion \
  --dataset.repo_id=lerobot/pusht \
  --env.type=pusht \
  --batch_size=64 \
  --steps=200000 \
  --eval_freq=25000 \
  --save_freq=25000
```

### マルチGPUトレーニング

```bash
# torchrun を使用
torchrun --nproc_per_node=4 -m lerobot.scripts.train \
  --dataset.repo_id=${HF_USER}/so101_test \
  --policy.type=act \
  --output_dir=outputs/train/act_multigpu
```

### 設定ファイル（JSON形式）

```json
{
  "dataset": {
    "repo_id": "user/dataset_name"
  },
  "policy": {
    "type": "act",
    "chunk_size": 100,
    "n_action_steps": 100,
    "dim_model": 512,
    "n_heads": 8,
    "n_encoder_layers": 4,
    "n_decoder_layers": 1
  },
  "training": {
    "batch_size": 8,
    "lr": 1e-4,
    "steps": 100000
  }
}
```

```bash
# 設定ファイルを使用
lerobot-train --config_path=config.json
```

### 主要なトレーニングパラメータ

| パラメータ | 説明 | 推奨値 |
|-----------|------|--------|
| `--steps` | トレーニングステップ数 | 100000-500000 |
| `--batch_size` | バッチサイズ | 8-64 (GPUメモリに依存) |
| `--policy.lr` | 学習率 | 1e-4 - 1e-5 |
| `--policy.chunk_size` | 予測するアクション数 | 50-100 |
| `--grad_clip_norm` | 勾配クリッピング | 10.0 |

## VLA Model Training

### SmolVLA のファインチューニング

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=smolvla \
  --policy.pretrained_path=lerobot/smolvla_base \
  --task="pick up the red cube" \
  --output_dir=outputs/train/smolvla_custom \
  --batch_size=4 \
  --steps=50000
```

### Pi0 のファインチューニング

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --policy.type=pi0fast \
  --policy.pretrained_path=physical-intelligence/pi0-fast \
  --task="grasp the object and place it in the bin" \
  --output_dir=outputs/train/pi0_custom \
  --batch_size=2 \
  --gradient_accumulation_steps=8
```

## Fine-tuning

### 事前学習モデルからのファインチューニング

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/so101_newtask \
  --policy.type=act \
  --policy.pretrained_path=outputs/train/act_so101/checkpoints/last \
  --output_dir=outputs/finetune/act_newtask \
  --steps=50000 \
  --policy.lr=1e-5
```

### トレーニングの再開

```bash
lerobot-train \
  --config_path=outputs/train/act_so101/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

### ファインチューニングのコツ

- **学習率**: 元の1/10程度（1e-5）
- **ステップ数**: 元の1/2程度
- **データ量**: 最低20エピソード
- **Freeze**: 必要に応じてbackboneを固定

```python
# バックボーン固定の例
for param in model.vision_backbone.parameters():
    param.requires_grad = False
```

## Evaluation

### シミュレーション環境での評価

LeRobotはLIBERO、Meta-Worldなどの標準ベンチマークをサポート。

```bash
python lerobot/scripts/eval.py \
  -p outputs/train/checkpoints/last.pt \
  --env.type=pusht \
  --eval.n_episodes=50 \
  --eval.batch_size=10
```

### 評価メトリクス

| メトリクス | 説明 | 目標 |
|-----------|------|------|
| success_rate | タスク成功率 | > 0.8 |
| avg_reward | 平均報酬 | タスク依存 |
| action_mse | アクション予測誤差 | < 0.01 |

### リアルロボットでの評価

```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/so101.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/so101_eval \
  -p outputs/train/checkpoints/last.pt \
  --warmup-time-s 5 \
  --episode-time-s 60 \
  --num-episodes 20
```

## HIL-SERL Workflow

Human-in-the-Loop Sample-Efficient Reinforcement Learning を使用したリアルタイム学習。

```bash
# 1. 初期デモンストレーション収集
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/so101.yaml \
  --repo-id ${HF_USER}/hilserl_demo \
  --num-episodes 10

# 2. HIL-SERLでトレーニング（人間介入付き）
lerobot-train \
  --policy.type=hilserl \
  --dataset.repo_id=${HF_USER}/hilserl_demo \
  --output_dir=outputs/train/hilserl \
  --online_training=true
```

## Troubleshooting

### CUDA out of memory

```bash
# バッチサイズを小さく
--batch_size=4

# 勾配累積を使用
--gradient_accumulation_steps=4

# 画像サイズを小さく
--policy.input_shapes.observation.images.top=[3,240,320]

# 勾配チェックポイント有効化
--grad_checkpointing=true
```

### データセットのロードが遅い

```bash
# ストリーミングモードを使用
--dataset.streaming=true

# ローカルキャッシュを使用
export HF_DATASETS_CACHE=/path/to/fast/storage

# num_workers を増やす
--num_workers=8
```

### トレーニングが収束しない

- 学習率を下げる（1e-5）
- バッチサイズを増やす
- データ品質を確認
- 正規化の確認

### モデルがうまく動作しない

- データ収集時と同じロボット設定か確認
- アクション空間の正規化確認
- カメラ位置・角度の一貫性確認

## Command Reference

| コマンド | 説明 |
|---------|------|
| `lerobot-train` | トレーニング実行（CLI） |
| `python lerobot/scripts/eval.py` | 評価実行 |
| `python lerobot/scripts/control_robot.py record` | データ収集 |
| `python lerobot/scripts/push_dataset_to_hub.py` | HFへアップロード |
| `python lerobot/scripts/visualize_dataset.py` | データセット可視化 |

## Resources

- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [Robot Learning Tutorial (HF Space)](https://huggingface.co/spaces/lerobot/robot-learning-tutorial)
- [LeRobot v0.4.0 Release Blog](https://huggingface.co/blog/lerobot-release-v040)
- [LeRobotDataset v3.0 Blog](https://huggingface.co/blog/lerobot-datasets-v3)
- [SmolVLA Blog](https://huggingface.co/blog/smolvla)
- [HuggingFace Datasets](https://huggingface.co/datasets?other=LeRobot)
- [ACT Paper](https://arxiv.org/abs/2304.13705)
- [Diffusion Policy Paper](https://arxiv.org/abs/2303.04137)
