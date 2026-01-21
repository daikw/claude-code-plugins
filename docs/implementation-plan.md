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

### 6. so-arm（SO-ARM100/101 ロボットアーム）

**フロントマッター**:
```yaml
---
name: so-arm
description: Use when setting up or operating SO-ARM100/101 robot arms. Covers assembly, servo configuration, calibration, and LeRobot integration.
---
```

**本文構成**:
- `## When to Activate` - SO-ARM セットアップ・操作時
- `## Hardware Overview` - SO-ARM100 vs SO-ARM101 の違い
- `## Assembly` - 組み立て手順
- `## Servo Configuration` - Feetech サーボ設定
- `## Calibration` - LeRobot キャリブレーション
- `## Troubleshooting` - 通信・トルクエラー対応

---

## 組込ソフトウェア開発スキル

Jetson/Raspberry Pi をゲートウェイとし、周辺にMCU（ESP32, nRF, Pico, STM32等）をセンサーノードとして配置するアーキテクチャが一般的。
以下のスキルは、MCUファームウェア開発をサポートする。

### アーキテクチャ概要

```
┌─────────────────────────────────────────────────────────────┐
│  クラウド / ローカルサーバー                                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  ゲートウェイ（Jetson / Raspberry Pi）                        │
│  - ML推論、データ集約、通信制御                                │
│  - LeRobot 等の高レベル制御                                   │
└─────────────────────────────────────────────────────────────┘
          │              │              │
          │ UART/USB     │ I2C/SPI      │ WiFi/BLE
          ▼              ▼              ▼
     ┌─────────┐   ┌─────────┐   ┌─────────┐
     │  ESP32  │   │  STM32  │   │   nRF   │
     │センサー │   │モーター │   │ BLEセンサ│
     └─────────┘   └─────────┘   └─────────┘
```

### 通信プロトコル

| プロトコル | 用途 | 特徴 |
|-----------|------|------|
| UART/USB Serial | デバッグ、コマンド | シンプル、1対1 |
| I2C | センサー接続 | アドレス指定、複数デバイス |
| SPI | 高速センサー/ディスプレイ | 高速、チップセレクト必要 |
| CAN Bus | 産業用、ロボット | 堅牢、リアルタイム |
| RS485/Modbus | 産業センサー | 長距離、ノイズ耐性 |
| WiFi/BLE | ワイヤレス | 柔軟、ESP32/nRF得意 |

---

### 7. embedded-basics（組込基礎）

**フロントマッター**:
```yaml
---
name: embedded-basics
description: Use when learning embedded systems fundamentals. Covers MCU architecture, memory, peripherals, debugging, and development workflow.
---
```

**本文構成**:
- `## When to Activate` - 組込開発の基礎を学ぶ時
- `## MCU Architecture` - CPU、メモリ、ペリフェラル
- `## Memory Model` - Flash, RAM, スタック/ヒープ
- `## Peripheral Basics` - GPIO, Timer, ADC, PWM
- `## Development Workflow` - ビルド、書き込み、デバッグ
- `## Debugging` - GDB, OpenOCD, JTAG/SWD
- `## Common Patterns` - 割り込み、DMA、低消費電力

---

### 8. esp32（ESP32/ESP-IDF）

**フロントマッター**:
```yaml
---
name: esp32
description: Use when developing firmware for ESP32 devices. Covers ESP-IDF, WiFi/BLE, FreeRTOS tasks, and peripheral drivers.
---
```

**本文構成**:
- `## When to Activate` - ESP32 ファームウェア開発時
- `## Device Overview` - ESP32, ESP32-S3, ESP32-C3 の違い
- `## ESP-IDF Setup` - 環境構築、idf.py の使い方
- `## WiFi/BLE` - 接続設定、スキャン、通信
- `## Peripherals` - GPIO, ADC, I2C, SPI, UART
- `## FreeRTOS Tasks` - タスク作成、同期
- `## Power Management` - Deep Sleep, Light Sleep
- `## OTA Updates` - ファームウェア更新

**resources/**:
- `esp-idf-setup.md`
- `wifi-ble-guide.md`
- `peripherals.md`
- `power-management.md`

---

### 9. nrf（Nordic nRF）

**フロントマッター**:
```yaml
---
name: nrf
description: Use when developing firmware for Nordic nRF devices. Covers nRF Connect SDK, Zephyr, BLE, and low-power design.
---
```

**本文構成**:
- `## When to Activate` - nRF ファームウェア開発時
- `## Device Overview` - nRF52840, nRF5340, nRF54L15 の違い
- `## nRF Connect SDK` - 環境構築、west の使い方
- `## BLE` - Advertising, GATT, Mesh
- `## Peripherals` - GPIO, TWI(I2C), SPIM, UARTE
- `## Power Optimization` - System OFF, 低消費電力設計
- `## DFU` - Bootloader, OTA更新

**resources/**:
- `nrf-connect-setup.md`
- `ble-guide.md`
- `power-optimization.md`

---

### 10. pico（Raspberry Pi Pico）

**フロントマッター**:
```yaml
---
name: pico
description: Use when developing firmware for Raspberry Pi Pico. Covers Pico SDK, PIO, and MicroPython.
---
```

**本文構成**:
- `## When to Activate` - Pico ファームウェア開発時
- `## Device Overview` - Pico, Pico W, Pico 2 の違い
- `## Pico SDK (C/C++)` - 環境構築、CMake
- `## MicroPython` - インタープリタ、REPL
- `## PIO (Programmable I/O)` - カスタムプロトコル実装
- `## Peripherals` - GPIO, ADC, I2C, SPI, UART, PWM
- `## USB` - CDC, HID デバイス

**resources/**:
- `pico-sdk-setup.md`
- `micropython-guide.md`
- `pio-examples.md`

---

### 11. stm32（STM32）

**フロントマッター**:
```yaml
---
name: stm32
description: Use when developing firmware for STM32 devices. Covers STM32CubeIDE, HAL/LL drivers, and debugging.
---
```

**本文構成**:
- `## When to Activate` - STM32 ファームウェア開発時
- `## Device Overview` - F4, G4, H7, U5 シリーズの違い
- `## STM32CubeIDE` - プロジェクト作成、CubeMX
- `## HAL vs LL` - ドライバ選択基準
- `## Peripherals` - GPIO, TIM, ADC, I2C, SPI, UART
- `## Debugging` - ST-Link, SWD, ITM
- `## RTOS Integration` - FreeRTOS, ThreadX

**resources/**:
- `stm32cube-setup.md`
- `hal-ll-guide.md`
- `debugging.md`

---

### 12. zephyr（Zephyr RTOS）

**フロントマッター**:
```yaml
---
name: zephyr
description: Use when developing with Zephyr RTOS. Covers west build system, devicetree, Kconfig, and multi-board support.
---
```

**本文構成**:
- `## When to Activate` - Zephyr RTOS 開発時
- `## Overview` - Zephyr の特徴、サポートボード
- `## West Setup` - 環境構築、ワークスペース管理
- `## Devicetree` - ハードウェア記述、オーバーレイ
- `## Kconfig` - コンフィグレーション
- `## Kernel Primitives` - Thread, Semaphore, Queue, Timer
- `## Subsystems` - Logging, Shell, Settings, Bluetooth
- `## Building & Flashing` - west build, west flash

**resources/**:
- `zephyr-setup.md`
- `devicetree-guide.md`
- `kconfig-guide.md`

---

### 13. freertos（FreeRTOS）

**フロントマッター**:
```yaml
---
name: freertos
description: Use when developing with FreeRTOS. Covers task management, synchronization primitives, and memory management.
---
```

**本文構成**:
- `## When to Activate` - FreeRTOS 開発時
- `## Overview` - FreeRTOS の特徴、ポート
- `## Task Management` - xTaskCreate, 優先度、スケジューリング
- `## Synchronization` - Semaphore, Mutex, Queue, EventGroup
- `## Timers` - Software Timer
- `## Memory Management` - heap_1 〜 heap_5
- `## Debugging` - Stack Overflow, Runtime Stats
- `## FreeRTOS+` - TCP, FAT, CLI

**resources/**:
- `freertos-setup.md`
- `task-management.md`
- `synchronization.md`

---

### 14. arduino（Arduino）

**フロントマッター**:
```yaml
---
name: arduino
description: Use when developing with Arduino framework. Covers Arduino IDE/CLI, libraries, and board support.
---
```

**本文構成**:
- `## When to Activate` - Arduino 開発時
- `## Overview` - Arduino の特徴、エコシステム
- `## Arduino IDE/CLI` - 環境構築、ボード管理
- `## Core API` - Digital/Analog I/O, Serial, Wire, SPI
- `## Libraries` - ライブラリ管理、よく使うライブラリ
- `## PlatformIO` - 代替開発環境
- `## ESP32/STM32 with Arduino` - 非公式ボードサポート

**resources/**:
- `arduino-cli-setup.md`
- `library-guide.md`
- `platformio-guide.md`

---

## 依存関係

```
edge-common（基盤）
    ├── jetson（依存）
    ├── raspberry-pi（依存）
    ├── so-arm（参照）
    └── lerobot-fullstack（参照）

lerobot-basics（独立）
    └── lerobot-fullstack（発展）
        └── so-arm（連携）

embedded-basics（基盤）
    ├── esp32（依存）
    ├── nrf（依存）
    ├── pico（依存）
    └── stm32（依存）

zephyr（独立、nRF/STM32と相互参照）
freertos（独立、ESP32/STM32と相互参照）
arduino（独立、全MCUと相互参照）
```

---

## エージェント設計

スキルは「知識・ワークフロー」を提供し、エージェントは「自律的なタスク実行」を担当する。
以下のエージェントは、エッジデバイスでの作業を自動化・支援する。

### 1. device-health-checker

**役割**: Jetson/Raspberry Pi のシステム健全性を診断

```yaml
---
name: device-health-checker
description: Diagnose Jetson/RPi system health. Checks CPU/GPU temp, memory, disk, and running services.
tools: Bash, Read, Grep
model: haiku
---
```

**使用場面**:
- デバイスが遅い・不安定な時
- デプロイ前の動作確認
- 定期的なヘルスチェック

**出力**: システム健全性レポート（温度、メモリ、ストレージ、サービス状態）

---

### 2. sensor-scanner

**役割**: I2C/SPI/USB/GPIO/カメラ/マイク/基板上センサーを自動検出・識別

```yaml
---
name: sensor-scanner
description: Auto-detect and identify connected sensors, cameras, microphones. Scans I2C, SPI, USB, GPIO, V4L2, ALSA.
tools: Bash, Read, WebSearch
model: sonnet
---
```

**使用場面**:
- 新しいセンサーを接続した時
- カメラやマイクが認識されない時
- 接続トラブルシューティング
- プロジェクト開始時のハードウェア確認

**出力**: 検出されたデバイスのリスト（I2C、USB、カメラ、マイク、GPIO状態、基板上センサー）

---

### 3. model-optimizer

**役割**: ML モデルをエッジデバイス向けに最適化

```yaml
---
name: model-optimizer
description: Optimize ML models for edge deployment using TensorRT, ONNX, quantization.
tools: Bash, Read, Write, Grep, WebSearch
model: sonnet
---
```

**使用場面**:
- 推論速度を向上させたい時
- モデルサイズを削減したい時
- Jetson/RPi へのデプロイ準備

**出力**: 最適化レポート（変換手順、ベンチマーク結果、推奨設定）

---

### 4. firmware-flasher

**役割**: USB接続されたMCUへのファームウェア検出・書き込み

```yaml
---
name: firmware-flasher
description: Detect and flash firmware to USB-connected MCUs (ESP32, nRF, Pico, STM32). Auto-detects board type and selects appropriate tool.
tools: Bash, Read, Grep, WebSearch
model: sonnet
---
```

**使用場面**:
- 新しいMCUボードを接続した時
- ファームウェアを書き込みたい時
- ブートローダーモードへの入り方がわからない時
- 書き込みエラーのトラブルシューティング

**検出フロー**:
1. USB デバイス列挙（lsusb, /dev/ttyUSB*, /dev/ttyACM*）
2. VID:PID からボード種類を推定
3. 適切な書き込みツールを選択
4. ファームウェア書き込み実行

**対応ボード・ツール**:

| ボード | VID:PID 例 | 書き込みツール |
|--------|-----------|---------------|
| ESP32 | 303a:1001 | esptool.py |
| ESP32 (CH340) | 1a86:7523 | esptool.py |
| nRF52/53 | 1915:521f | nrfjprog, west flash |
| Pico | 2e8a:0003 | picotool |
| STM32 (ST-Link) | 0483:3748 | st-flash, STM32CubeProgrammer |
| STM32 (DFU) | 0483:df11 | dfu-util |

**出力**: 検出レポート（ボード種類、接続ポート、推奨コマンド）、書き込み結果

---

## エージェントとスキルの使い分け

| 状況 | 使うもの | 理由 |
|------|---------|------|
| GPIO の使い方を知りたい | スキル (raspberry-pi) | 知識参照 |
| 接続センサーを確認したい | エージェント (sensor-scanner) | 実行が必要 |
| LeRobot のトレーニング方法 | スキル (lerobot-basics) | 知識参照 |
| SO-ARM のセットアップ方法 | スキル (so-arm) | 知識参照 |
| tegrastats の読み方 | スキル (jetson) | 知識参照 |
| システム状態を診断したい | エージェント (device-health-checker) | 実行が必要 |
| ESP-IDF の使い方を知りたい | スキル (esp32) | 知識参照 |
| MCUにファームウェアを書きたい | エージェント (firmware-flasher) | 実行が必要 |
| Zephyr devicetree の書き方 | スキル (zephyr) | 知識参照 |
| モデルをTensorRT変換したい | エージェント (model-optimizer) | 実行が必要 |

---

## TODO

### 完了
- [x] 各スキルの SKILL.md をフロントマッター形式で作成
- [x] jetson-containers の詳細Tipsを調査・追記
- [x] SO-ARM100 の具体的なセットアップ手順を追記
- [x] so-arm スキル作成
- [x] sensor-scanner エージェント拡張（カメラ、マイク、GPIO、基板上センサー）
- [x] embedded-basics スキル作成
- [x] esp32 スキル作成
- [x] nrf スキル作成
- [x] pico スキル作成
- [x] stm32 スキル作成
- [x] zephyr スキル作成
- [x] freertos スキル作成
- [x] arduino スキル作成
- [x] firmware-flasher エージェント作成
- [x] Rules 実装（12個: agents, security, testing, patterns, performance, hooks, memory-safety, realtime-safety, design-principles, state-concurrency, error-handling, deploy-data）

### 保留
- [ ] transformers への微修正内容を追記（ユーザーからの情報待ち）
