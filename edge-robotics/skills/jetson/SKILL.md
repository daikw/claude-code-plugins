---
name: jetson
description: Use when developing on NVIDIA Jetson devices. Covers tegrastats monitoring, CUDA optimization, JetPack management, power modes, Super Mode, and jetson-containers.
tags:
  - jetson
  - nvidia
  - cuda
  - edge-ai
  - gpu
  - tensorrt
---

# NVIDIA Jetson Development Guide

NVIDIA Jetson シリーズでの開発・運用をサポート。GPU活用、システム監視、コンテナ利用のガイド。

## When to Activate

- Jetson デバイスで開発・運用するとき
- tegrastats の出力を解釈したいとき
- CUDA/TensorRT で推論を最適化したいとき
- jetson-containers でML環境を構築するとき
- 電力モード/Super Modeを調整したいとき
- Orin/Thor シリーズの性能を引き出したいとき

## Device Overview

### 現行ラインナップ (2025年時点)

| モデル | GPU | AI性能 | CPU | メモリ | 電力 | 用途 |
|--------|-----|--------|-----|--------|------|------|
| Jetson Orin Nano 4GB | 512 CUDA cores | 20 TOPS | 6x A78AE | 4GB | 7-15W | エントリー |
| Jetson Orin Nano 8GB | 1024 CUDA cores | 40 TOPS (67 Super) | 6x A78AE | 8GB | 7-25W | 標準エッジAI |
| Jetson Orin NX 8GB | 1024 CUDA cores | 70 TOPS (117 Super) | 6x A78AE | 8GB | 10-40W | 本番エッジAI |
| Jetson Orin NX 16GB | 1024 CUDA cores | 100 TOPS (157 Super) | 8x A78AE | 16GB | 10-40W | 高性能エッジ |
| Jetson AGX Orin 32GB | 2048 CUDA cores | 200 TOPS | 12x A78AE | 32GB | 15-60W | 最高性能 |
| Jetson AGX Orin 64GB | 2048 CUDA cores | 275 TOPS | 12x A78AE | 64GB | 15-60W | 大規模モデル |
| Jetson AGX Thor | Blackwell GPU | 2070 TFLOPS (FP4) | Grace CPU | 128GB | 40-130W | 次世代Physical AI |

### レガシーモデル

| モデル | GPU | CPU | メモリ | 用途 |
|--------|-----|-----|--------|------|
| Jetson Nano | 128 CUDA cores | 4x A57 | 4GB | 入門、プロトタイプ |
| Jetson Xavier NX | 384 CUDA cores | 6x Carmel | 8/16GB | 旧世代エッジAI |
| Jetson AGX Xavier | 512 CUDA cores | 8x Carmel | 32GB | 旧世代高性能 |

## JetPack Management

### バージョン確認

```bash
# JetPackバージョン確認
cat /etc/nv_tegra_release

# L4Tバージョン確認
head -n 1 /etc/nv_tegra_release

# CUDA バージョン
nvcc --version

# cuDNN バージョン
cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2

# TensorRT バージョン
dpkg -l | grep tensorrt

# jetson-stats でまとめて確認（推奨）
jetson_release
```

### JetPack / L4T 対応表

| JetPack | L4T | CUDA | cuDNN | TensorRT | VPI | 対応デバイス |
|---------|-----|------|-------|----------|-----|-------------|
| 6.2.1 | 36.4.4 | 12.6 | 9.3 | 10.3 | 3.2 | Orin series |
| 6.2 | 36.4.3 | 12.6 | 9.3 | 10.3 | 3.2 | Orin series (Super Mode対応) |
| 6.1 | 36.4 | 12.6 | 9.3 | 10.3 | 3.2 | Orin series |
| 6.0 | 36.3 | 12.2 | 8.9 | 8.6 | 3.1 | Orin series |
| 5.1.2 | 35.4.1 | 11.4 | 8.6 | 8.5 | 2.3 | Xavier, Orin |
| 4.6.4 | 32.7.4 | 10.2 | 8.2 | 8.2 | 1.2 | Nano, TX2, Xavier |

### JetPack アップグレード

```bash
# JetPack 6.x内でのアップグレード (OTA)
sudo apt update
sudo apt dist-upgrade
sudo apt install nvidia-jetpack

# メジャーバージョンアップ (5.x → 6.x) はSDK Managerかフラッシュが必要
```

## System Monitoring

### tegrastats

```bash
# 基本実行
tegrastats

# 1秒間隔でログ出力
tegrastats --interval 1000 --logfile /tmp/tegrastats.log
```

### tegrastats 出力の読み方

```
RAM 2345/3956MB (lfb 1x4MB) SWAP 0/1978MB (cached 0MB)
CPU [25%@1479,15%@1479,10%@1479,8%@1479]
EMC_FREQ 0%@1600 GR3D_FREQ 0%@76
APE 150 PLL@41C CPU@42.5C PMIC@50C GPU@41C AO@46C thermal@42.1C
```

| 項目 | 説明 |
|------|------|
| RAM | 使用中/総メモリ |
| lfb | 最大連続空きブロック |
| SWAP | スワップ使用量 |
| CPU | 各コアの使用率@周波数(MHz) |
| EMC_FREQ | メモリ帯域使用率@周波数 |
| GR3D_FREQ | GPU使用率@周波数 |
| PLL@, CPU@, GPU@ | 各部の温度 |

### jtop（推奨）

```bash
# インストール
sudo pip3 install -U jetson-stats

# 実行
jtop
```

**機能**:
- リアルタイムモニタリング
- 電力モード変更
- ファン制御
- プロセス一覧
- JetPack/L4T情報表示

### jtop Python API

```python
from jtop import jtop

with jtop() as jetson:
    # 基本情報
    print(jetson.board)      # ハードウェア情報
    print(jetson.jetpack)    # JetPackバージョン

    # リアルタイム監視
    while jetson.ok():
        stats = jetson.stats
        print(f"GPU: {stats['GPU']}%")
        print(f"CPU: {stats['CPU1']}%")
        print(f"Temp: {stats['Temp GPU']}°C")
        print(f"Power: {stats['Power TOT']}mW")
```

## Power Modes

### nvpmodel

```bash
# 現在のモード確認
nvpmodel -q

# モード一覧
nvpmodel -p --verbose

# モード変更（要root）
sudo nvpmodel -m <mode_id>
```

### Super Mode (JetPack 6.2+)

JetPack 6.2以降でOrin Nano/NXに追加されたハイパワーモード。GPU/DLA/CPUのクロックを引き上げ、最大2倍の生成AI性能を実現。

**性能向上**:
- Orin NX: 100 TOPS → 157 TOPS (70%向上)
- Orin Nano: 40 TOPS → 67 TOPS (67%向上)
- メモリ帯域: 68GB/s → 102GB/s (50%向上)

### Jetson Orin NX 16GB のモード例

| モード | 電力 | GPU周波数 | CPU | AI性能 | 用途 |
|--------|------|-----------|-----|--------|------|
| MAXN SUPER | 無制限 | 1173MHz | 8コア | 157 TOPS | 最高性能 |
| 40W SUPER | 40W | 1173MHz | 8コア | 157 TOPS | Super推奨 |
| 25W (MAXN) | 25W | 765MHz | 8コア | 100 TOPS | 標準最高 |
| 15W | 15W | 制限 | 6コア | - | バランス |
| 10W | 10W | 制限 | 4コア | - | 省電力 |

### Jetson Orin Nano 8GB のモード例

| モード | 電力 | GPU周波数 | CPU | AI性能 | 用途 |
|--------|------|-----------|-----|--------|------|
| MAXN SUPER | 無制限 | 1020MHz | 6コア | 67 TOPS | 最高性能 |
| 25W SUPER | 25W | 1020MHz | 6コア | 67 TOPS | Super推奨 |
| 15W (MAXN) | 15W | 625MHz | 6コア | 40 TOPS | 標準最高 |
| 7W | 7W | 制限 | 4コア | - | 省電力 |

```bash
# Super Modeを有効化（JetPack 6.2+）
sudo nvpmodel -m 0  # MAXN SUPERモード

# 確認
nvpmodel -q
```

### ファン制御

```bash
# 手動制御（jetson-stats使用）
sudo jetson_clocks --fan

# PWM直接制御
echo 255 | sudo tee /sys/devices/pwm-fan/target_pwm

# Super Mode使用時はファン強化を推奨
```

## GPU Optimization

### CUDA メモリ管理

```python
import torch

# GPU利用可能確認
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0))

# メモリ使用量確認
print(f"Allocated: {torch.cuda.memory_allocated() / 1024**2:.1f}MB")
print(f"Cached: {torch.cuda.memory_reserved() / 1024**2:.1f}MB")

# キャッシュクリア
torch.cuda.empty_cache()
```

### TensorRT 変換

```python
# PyTorch -> ONNX -> TensorRT

# 1. ONNX エクスポート
import torch
# model = YourModel()
dummy_input = torch.randn(1, 3, 224, 224).cuda()
torch.onnx.export(model, dummy_input, "model.onnx", opset_version=17)

# 2. TensorRT 変換
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

```bash
# trtexec でベンチマーク (TensorRT 10.3+)
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model.trt \
  --fp16 \
  --memPoolSize=workspace:1024MiB

# DLA (Deep Learning Accelerator) を使用
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model_dla.trt \
  --useDLACore=0 \
  --allowGPUFallback \
  --fp16
```

### Torch-TensorRT (推奨)

JetPack 6.x ではPyTorchから直接TensorRTを利用可能。

```python
import torch
import torch_tensorrt

# model = YourModel()
model.cuda()

# TensorRT コンパイル
trt_model = torch_tensorrt.compile(
    model,
    inputs=[torch_tensorrt.Input(
        shape=[1, 3, 224, 224],
        dtype=torch.float16
    )],
    enabled_precisions={torch.float16},
    workspace_size=1 << 30  # 1GB
)

# 推論
with torch.no_grad():
    output = trt_model(input_tensor.half())
```

### 推論最適化Tips

- **FP16**: `--fp16` で半精度、2倍高速化
- **INT8**: キャリブレーション必要だがさらに高速
- **DLA活用**: Orin系はDLAコプロセッサを2基搭載、GPUと並列実行可能
- **バッチ処理**: 複数フレームをまとめて推論
- **CUDA Streams**: 非同期処理で転送と計算をオーバーラップ
- **Unified Memory**: Jetsonは統合メモリ、CPU/GPU間コピー不要な場合あり

## Jetson Containers

[dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers) を活用したML環境構築。

### セットアップ (2025年最新)

```bash
# リポジトリクローン & インストール
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh

# 自動タグ検出で実行
jetson-containers run $(autotag l4t-pytorch)
```

### コンテナ実行

```bash
# PyTorch コンテナ (JetPack 6.x)
jetson-containers run dustynv/pytorch:2.4-r36.4.0

# Ollama (LLM)
jetson-containers run dustynv/ollama:r36.4.3

# vLLM (高速LLM推論)
jetson-containers run dustynv/vllm:r36.4.0-cu128

# Stable Diffusion WebUI
jetson-containers run dustynv/stable-diffusion-webui:r36.4.0
```

### 利用可能なコンテナ

| カテゴリ | コンテナ | 用途 |
|---------|---------|------|
| ML Framework | pytorch, tensorflow | モデル学習・推論 |
| LLM | ollama, vllm, llama-cpp, text-generation-webui | 大規模言語モデル |
| LLM Tools | SGLang, MLC, AWQ, AutoGPTQ, exllama | LLM最適化 |
| Diffusion | ComfyUI, stable-diffusion-webui, SD.Next | 画像生成 |
| Vision | opencv, realsense | 画像処理 |
| Robotics | ros2, isaac-ros | ロボット開発 |
| 3D | nerfstudio, gsplat, meshlab | 3D再構成 |

### Ubuntu 24.04 サポート

```bash
# Ubuntu 24.04向けビルド
LSB_RELEASE=24.04 jetson-containers build pytorch:2.8

# 24.04コンテナ実行
jetson-containers run dustynv/pytorch:2.8-r36.4-cu128-24.04
```

### カスタムコンテナ作成

```bash
# 既存コンテナをベースにビルド
jetson-containers build --name=my-container pytorch tensorrt

# Dockerfile生成のみ
jetson-containers build --name=my-container --skip-build pytorch
```

### ボリュームマウント

```bash
# データディレクトリをマウント
jetson-containers run -v /path/to/data:/data dustynv/pytorch:2.4-r36.4.0

# 複数マウント
jetson-containers run \
  -v /home/user/models:/models \
  -v /home/user/data:/data \
  dustynv/pytorch:2.4-r36.4.0
```

### SBSA対応 (GH200/GB200/Thor)

```bash
# CUDA 13.0 SBSA wheels (Thor/GB200向け)
uv pip install torch torchvision torchaudio \
  --index-url https://pypi.jetson-ai-lab.io/sbsa/cu129
```

## Troubleshooting

### CUDA out of memory

```python
# バッチサイズを小さくする
# FP16を使用する
model = model.half()

# 不要なテンソルを削除
del tensor
torch.cuda.empty_cache()

# 勾配計算を無効化（推論時）
with torch.no_grad():
    output = model(input)
```

### 温度が高すぎる

```bash
# ファンを最大に
sudo jetson_clocks --fan

# 電力モードを下げる
sudo nvpmodel -m 2

# 放熱対策（ヒートシンク、ファン追加）
```

### コンテナでGPUが認識されない

```bash
# nvidia-container-runtime 確認
docker info | grep -i runtime

# --runtime=nvidia オプション確認
docker run --runtime=nvidia ...

# または jetson-containers の run.sh を使用
```

## LLM Deployment Example

Orin上でLLMを動かすサンプル。

```bash
# Ollama でLlama3.1 8Bを実行
jetson-containers run dustynv/ollama:r36.4.3

# コンテナ内で
ollama run llama3.1:8b

# vLLM で高速推論 (Super Mode推奨)
jetson-containers run dustynv/vllm:r36.4.0-cu128
# Qwen2.5 7B や Gemma2 2B が動作確認済み
```

## Command Reference

| コマンド | 説明 |
|---------|------|
| `tegrastats` | システムモニタリング |
| `nvpmodel -q` | 電力モード確認 |
| `nvpmodel -m <id>` | 電力モード変更 |
| `jetson_clocks` | クロック固定（最大性能） |
| `jtop` | インタラクティブモニター |
| `jetson_release` | JetPack/L4T情報表示 |
| `nvcc --version` | CUDA バージョン |
| `jetson-containers run` | MLコンテナ実行 |

## Resources

- [JetPack 6.2.1 Release Notes](https://docs.nvidia.com/jetson/jetpack/release-notes/)
- [jetson-containers GitHub](https://github.com/dusty-nv/jetson-containers)
- [jetson-stats Documentation](https://rnext.it/jetson_stats)
- [Jetson AI Lab](https://www.jetson-ai-lab.com/)
- [NVIDIA Jetson Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)
