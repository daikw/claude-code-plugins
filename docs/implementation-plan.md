# スキル実装計画

## 参考リポジトリからのインサイト

[everything-claude-code](https://github.com/affaan-m/everything-claude-code) の構成から得られた知見。

### ファイル形式

#### スキル（skills/xxx/SKILL.md）

```yaml
---
name: skill-name
description: 使用場面と主要特徴を1行で記述
---
```

**本文構成**:
1. `# タイトル` - スキル概要
2. `## When to Activate` - 発動条件リスト
3. `## Core Principles` - 基本原則
4. `## Workflow Steps` - 番号付き手順
5. `## Command Reference` - コマンド一覧（テーブル形式）
6. コード例、✅/❌ パターン集

#### エージェント（agents/xxx.md）

```yaml
---
name: agent-name
description: 役割説明（PROACTIVELY等のトリガー条件含む）
tools: Read, Grep, Glob, Bash
model: opus
---
```

**本文構成**:
1. `## Your Role` - 役割定義
2. `## Process` - 実行プロセス（番号付き）
3. `## Output Format` - 出力フォーマット例

### 構成パターン

| ディレクトリ | 役割 | 当リポジトリでの対応 |
|-------------|------|---------------------|
| `skills/` | 再利用可能なワークフロー・領域知識 | メインコンテンツ |
| `agents/` | 特定タスクを委譲する専門エージェント | 必要に応じて追加 |
| `commands/` | スラッシュコマンド（即実行） | 将来的に検討 |
| `rules/` | 常に従うべき非交渉的ガイドライン | SKILL.md 内に統合 |
| `hooks/` | ツール実行時の自動トリガー | 将来的に検討 |

### 設計指針

- **コンテキストウィンドウ最適化**: MCP は 20-30 個設定、10 個以下を有効化
- **階層的責任分離**: エージェント（委譲）、スキル（ワークフロー）、ルール（制約）
- **実用性重視**: 過度な一般化を避け、始めやすくカスタマイズしやすい設計
- **フロントマッター必須**: すべてのスキル・エージェントに `name`, `description` を記載

---

## スキル実装計画

### 1. edge-common（エッジデバイス共通基盤）

**フロントマッター**:
```yaml
---
name: edge-common
description: Use when working with edge devices (Jetson, Raspberry Pi). Provides sensor/actuator catalogs and communication protocol guides.
---
```

**本文構成**:
- `## When to Activate` - エッジデバイス作業時
- `## Communication Protocols` - I2C, SPI, UART, GPIO
- `## Sensor Catalog` - 温度、加速度、距離、カメラ等
- `## Actuator Catalog` - モーター、サーボ、リレー等
- `## Power Management` - 電源管理の基礎

**resources/**:
- `sensor-catalog.md`
- `actuator-catalog.md`
- `communication-protocols.md`

---

### 2. jetson（NVIDIA Jetson 特化）

**フロントマッター**:
```yaml
---
name: jetson
description: Use when developing on NVIDIA Jetson devices. Covers tegrastats, CUDA optimization, JetPack management, and jetson-containers.
---
```

**本文構成**:
- `## When to Activate` - Jetson 開発・運用時
- `## Device Overview` - Nano, Xavier, Orin の違い
- `## JetPack Management` - バージョン管理
- `## Monitoring` - tegrastats の読み方
- `## GPU Optimization` - CUDA/cuDNN, TensorRT
- `## Power Modes` - nvpmodel 設定
- `## Containers` - jetson-containers 活用

**resources/**:
- `tegrastats-guide.md`
- `jetpack-versions.md`
- `cuda-optimization.md`
- `power-modes.md`
- `jetson-containers.md` - dusty-nv/jetson-containers の使い方

---

### 3. raspberry-pi（Raspberry Pi 特化）

**フロントマッター**:
```yaml
---
name: raspberry-pi
description: Use when developing on Raspberry Pi. Covers GPIO, camera modules, HATs, and OS configuration.
---
```

**本文構成**:
- `## When to Activate` - Raspberry Pi 開発・運用時
- `## Device Overview` - Pi 4, Pi 5, Zero 2W の違い
- `## GPIO` - ピン配置と使い方
- `## Camera` - カメラモジュール設定
- `## Peripherals` - HAT/pHAT 接続

**resources/**:
- `gpio-pinout.md`
- `peripherals.md`
- `camera-setup.md`

---

### 4. lerobot-basics（LeRobot 基本・トレーニング）

**フロントマッター**:
```yaml
---
name: lerobot-basics
description: Use when training robot manipulation models with LeRobot. Covers installation, dataset creation, and fine-tuning workflows.
---
```

**本文構成**:
- `## When to Activate` - LeRobot でのトレーニング時
- `## Installation` - 環境構築
- `## Dataset Creation` - カスタムデータセット作成
- `## Training Workflow` - 事前学習・ファインチューニング
- `## Evaluation` - メトリクス解釈
- `## Transformers Patches` - 必要な微修正（TODO: 詳細追記）

**resources/**:
- `installation.md`
- `dataset-creation.md`
- `finetuning-guide.md`
- `supported-robots.md`
- `transformers-patches.md` - （TODO: 詳細追記）

---

### 5. lerobot-fullstack（LeRobot フルスタック）

**フロントマッター**:
```yaml
---
name: lerobot-fullstack
description: Use when deploying LeRobot models to physical robots. Covers SO-ARM100 setup, calibration, and edge deployment.
---
```

**本文構成**:
- `## When to Activate` - 実機デプロイ時
- `## SO-ARM100 Setup` - 組み立てと設定
- `## Calibration` - カメラキャリブレーション
- `## Real-time Inference` - 推論設定
- `## Edge Deployment` - Jetson/RPi へのデプロイ
- `## Integration` - 他スキルとの連携

**resources/**:
- `so-arm100-setup.md`
- `calibration.md`
- `deployment.md`
- `integration.md`

---

## 依存関係

```
edge-common（基盤）
    ├── jetson（依存）
    ├── raspberry-pi（依存）
    └── lerobot-fullstack（参照）

lerobot-basics（独立）
    └── lerobot-fullstack（発展）
```

---

## TODO

- [ ] transformers への微修正内容を追記（ユーザーからの情報待ち）
- [ ] jetson-containers の詳細Tipsを調査・追記
- [ ] SO-ARM100 の具体的なセットアップ手順を追記
- [ ] 各スキルの SKILL.md をフロントマッター形式で作成
