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

## Installation

### 基本インストール

```bash
# リポジトリクローン
git clone https://github.com/huggingface/lerobot.git
cd lerobot

# 仮想環境作成
python -m venv .venv
source .venv/bin/activate

# インストール
pip install -e .

# 開発用（テスト含む）
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

## Dataset Structure

### LeRobot データセット形式

```
dataset/
├── meta/
│   ├── info.json           # データセットメタ情報
│   ├── episodes.jsonl      # エピソード情報
│   └── stats.json          # 統計情報
├── data/
│   ├── chunk-000/
│   │   ├── episode_000000.parquet
│   │   ├── episode_000001.parquet
│   │   └── ...
│   └── ...
└── videos/                 # （オプション）動画ファイル
    ├── chunk-000/
    │   ├── observation.images.top/
    │   │   ├── episode_000000.mp4
    │   │   └── ...
    │   └── ...
    └── ...
```

### info.json の構造

```json
{
  "codebase_version": "v2.0",
  "robot_type": "so100",
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

## Data Collection

### テレオペレーションでのデータ収集

```bash
# SO-100 の場合
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/so100.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/so100_test \
  --tags so100 tutorial \
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
  --raw-dir data/raw/so100_test \
  --repo-id ${HF_USER}/so100_test \
  --raw-format lerobot
```

## Training

### 設定ファイル（Hydra）

```yaml
# configs/policy/act.yaml
policy:
  name: act

  # モデルアーキテクチャ
  chunk_size: 100
  n_action_steps: 100
  input_shapes:
    observation.images.top: [3, 480, 640]
    observation.state: [6]
  output_shapes:
    action: [6]

  # Vision Encoder
  vision_backbone: resnet18
  pretrained_backbone: true

  # Transformer
  dim_model: 512
  n_heads: 8
  n_encoder_layers: 4
  n_decoder_layers: 1
```

### トレーニング実行

```bash
# ACTポリシーでトレーニング
python lerobot/scripts/train.py \
  policy=act \
  env=so100 \
  dataset_repo_id=${HF_USER}/so100_test \
  training.num_epochs=100 \
  training.batch_size=8 \
  training.lr=1e-4 \
  wandb.enable=true \
  wandb.project=lerobot-so100
```

### 主要なトレーニングパラメータ

| パラメータ | 説明 | 推奨値 |
|-----------|------|--------|
| `training.num_epochs` | エポック数 | 100-500 |
| `training.batch_size` | バッチサイズ | 8-32 (GPUメモリに依存) |
| `training.lr` | 学習率 | 1e-4 - 1e-5 |
| `policy.chunk_size` | 予測するアクション数 | 50-100 |
| `training.grad_clip_norm` | 勾配クリッピング | 10.0 |

### 利用可能なポリシー

| ポリシー | 説明 | 用途 |
|---------|------|------|
| ACT | Action Chunking with Transformers | 高精度操作 |
| Diffusion | Diffusion Policy | 複雑なタスク |
| TDMPC | Temporal Difference MPC | オンライン適応 |
| VQ-BeT | Vector-Quantized BeT | 離散アクション |

## Fine-tuning

### 事前学習モデルからのファインチューニング

```bash
python lerobot/scripts/train.py \
  policy=act \
  env=so100 \
  dataset_repo_id=${HF_USER}/so100_newtask \
  hydra.run.dir=outputs/finetune \
  training.num_epochs=50 \
  training.lr=1e-5 \
  policy.pretrained_model_path=outputs/train/checkpoints/last.pt
```

### ファインチューニングのコツ

- **学習率**: 元の1/10程度（1e-5）
- **エポック数**: 元の1/2程度
- **データ量**: 最低20エピソード
- **Freeze**: 必要に応じてbackboneを固定

```python
# バックボーン固定の例
for param in model.vision_backbone.parameters():
    param.requires_grad = False
```

## Evaluation

### オフライン評価

```bash
python lerobot/scripts/eval.py \
  -p outputs/train/checkpoints/last.pt \
  eval.n_episodes=50 \
  eval.batch_size=10
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
  --robot-path lerobot/configs/robot/so100.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/so100_eval \
  -p outputs/train/checkpoints/last.pt \
  --warmup-time-s 5 \
  --episode-time-s 60 \
  --num-episodes 20
```

## Transformers Patches

> **TODO**: transformers ライブラリへの微修正について、詳細を後日追記予定。

LeRobot を特定の環境で使用する際、transformers ライブラリに対して微修正が必要な場合がある。

```python
# 例: カスタムモデル登録
# （詳細は後日追記）
```

## Troubleshooting

### CUDA out of memory

```bash
# バッチサイズを小さく
training.batch_size=4

# 画像サイズを小さく
policy.input_shapes.observation.images.top=[3,240,320]

# 勾配チェックポイント有効化
training.grad_checkpointing=true
```

### データセットのロードが遅い

```bash
# ローカルキャッシュを使用
export HF_DATASETS_CACHE=/path/to/fast/storage

# num_workers を増やす
training.num_workers=8
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
| `python lerobot/scripts/train.py` | トレーニング実行 |
| `python lerobot/scripts/eval.py` | 評価実行 |
| `python lerobot/scripts/control_robot.py record` | データ収集 |
| `python lerobot/scripts/push_dataset_to_hub.py` | HFへアップロード |
| `python lerobot/scripts/visualize_dataset.py` | データセット可視化 |

## Resources

- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [HuggingFace Datasets](https://huggingface.co/datasets?other=LeRobot)
- [ACT Paper](https://arxiv.org/abs/2304.13705)
- [Diffusion Policy Paper](https://arxiv.org/abs/2303.04137)
