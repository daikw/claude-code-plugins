---
name: jetson
description: Use when developing on NVIDIA Jetson devices. Covers tegrastats monitoring, CUDA optimization, JetPack management, power modes, and jetson-containers.
tags:
  - jetson
  - nvidia
  - cuda
  - edge-ai
  - gpu
---

# NVIDIA Jetson Development Guide

NVIDIA Jetson シリーズでの開発・運用をサポート。GPU活用、システム監視、コンテナ利用のガイド。

## When to Activate

- Jetson デバイスで開発・運用するとき
- tegrastats の出力を解釈したいとき
- CUDA/TensorRT で推論を最適化したいとき
- jetson-containers でML環境を構築するとき
- 電力モードを調整したいとき

## Device Overview

| モデル | GPU | CPU | メモリ | 用途 |
|--------|-----|-----|--------|------|
| Jetson Nano | 128 CUDA cores | 4x A57 | 4GB | 入門、プロトタイプ |
| Jetson Xavier NX | 384 CUDA cores | 6x Carmel | 8/16GB | 本番エッジAI |
| Jetson AGX Xavier | 512 CUDA cores | 8x Carmel | 32GB | 高性能エッジ |
| Jetson Orin Nano | 1024 CUDA cores | 6x A78AE | 4/8GB | Nano後継 |
| Jetson AGX Orin | 2048 CUDA cores | 12x A78AE | 32/64GB | 最高性能 |

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
```

### JetPack / L4T 対応表

| JetPack | L4T | CUDA | cuDNN | TensorRT | 対応デバイス |
|---------|-----|------|-------|----------|-------------|
| 6.0 | 36.3 | 12.2 | 8.9 | 8.6 | Orin series |
| 5.1.2 | 35.4.1 | 11.4 | 8.6 | 8.5 | Xavier, Orin |
| 4.6.4 | 32.7.4 | 10.2 | 8.2 | 8.2 | Nano, TX2, Xavier |

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
sudo pip3 install jetson-stats

# 実行
jtop
```

**機能**:
- リアルタイムモニタリング
- 電力モード変更
- ファン制御
- プロセス一覧

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

### Jetson Orin Nano のモード例

| モード | 電力 | GPU | CPU | 用途 |
|--------|------|-----|-----|------|
| 0 (MAXN) | 15W | Max | 6コア | 最高性能 |
| 1 | 15W | 制限 | 6コア | バランス |
| 2 | 7W | 制限 | 4コア | 省電力 |

### ファン制御

```bash
# 手動制御（jetson-stats使用）
sudo jetson_clocks --fan

# PWM直接制御
echo 255 | sudo tee /sys/devices/pwm-fan/target_pwm
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
# PyTorch → ONNX → TensorRT

# 1. ONNX エクスポート
import torch
model = ...
dummy_input = torch.randn(1, 3, 224, 224).cuda()
torch.onnx.export(model, dummy_input, "model.onnx", opset_version=11)

# 2. TensorRT 変換
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

```bash
# trtexec でベンチマーク
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model.trt \
  --fp16 \
  --workspace=1024
```

### 推論最適化Tips

- **FP16**: `--fp16` で半精度、2倍高速化
- **INT8**: キャリブレーション必要だがさらに高速
- **バッチ処理**: 複数フレームをまとめて推論
- **CUDA Streams**: 非同期処理で転送と計算をオーバーラップ

## Jetson Containers

[dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers) を活用したML環境構築。

### セットアップ

```bash
# リポジトリクローン
git clone https://github.com/dusty-nv/jetson-containers
cd jetson-containers

# 必要ツールインストール
pip3 install -r requirements.txt
```

### コンテナ実行

```bash
# PyTorch コンテナ
./run.sh dustynv/pytorch:r35.4.1

# L4M (LLM) コンテナ
./run.sh dustynv/llama-cpp:r35.4.1

# Stable Diffusion
./run.sh dustynv/stable-diffusion-webui:r35.4.1
```

### 利用可能なコンテナ

| カテゴリ | コンテナ | 用途 |
|---------|---------|------|
| ML Framework | pytorch, tensorflow | モデル学習・推論 |
| LLM | llama-cpp, text-generation-webui | 大規模言語モデル |
| Vision | opencv, realsense | 画像処理 |
| Robotics | ros, isaac-ros | ロボット開発 |
| Generative | stable-diffusion | 画像生成 |

### カスタムコンテナ作成

```bash
# 既存コンテナをベースにビルド
./build.sh --name=my-container pytorch tensorrt

# Dockerfile生成のみ
./build.sh --name=my-container --skip-build pytorch
```

### ボリュームマウント

```bash
# データディレクトリをマウント
./run.sh -v /path/to/data:/data dustynv/pytorch:r35.4.1

# 複数マウント
./run.sh \
  -v /home/user/models:/models \
  -v /home/user/data:/data \
  dustynv/pytorch:r35.4.1
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

## Command Reference

| コマンド | 説明 |
|---------|------|
| `tegrastats` | システムモニタリング |
| `nvpmodel -q` | 電力モード確認 |
| `nvpmodel -m <id>` | 電力モード変更 |
| `jetson_clocks` | クロック固定（最大性能） |
| `jtop` | インタラクティブモニター |
| `nvcc --version` | CUDA バージョン |
