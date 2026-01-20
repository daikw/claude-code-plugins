---
name: edge-common
description: Use when working with edge devices (Jetson, Raspberry Pi). Provides sensor/actuator catalogs, communication protocol guides, and Edge AI integration.
tags:
  - edge-computing
  - sensors
  - actuators
  - embedded
  - tinyml
  - industrial
---

# Edge Computing Common Knowledge

エッジデバイス（Jetson、Raspberry Pi等）で共通して使用する概念、通信プロトコル、センサ・アクチュエータの知識。

## When to Activate

- エッジデバイスでセンサやアクチュエータを接続するとき
- I2C, SPI, UART, GPIO の選択に迷ったとき
- 産業用プロトコル（CAN, Modbus, RS485）を使用するとき
- 新しいセンサ・アクチュエータを調査するとき
- 電源設計・消費電力を検討するとき
- Edge AI / TinyML を組み込むとき

## Communication Protocols

### プロトコル選択ガイド

#### ボード内通信（短距離）

| プロトコル | 速度 | 配線数 | 用途 | 例 |
|-----------|------|--------|------|-----|
| **GPIO** | - | 1 | デジタル入出力 | LED, ボタン, リレー |
| **I2C** | 100kHz-3.4MHz | 2 (SDA, SCL) | 低速センサ、設定 | 温度、加速度、OLED |
| **SPI** | 10-50MHz | 4+ (MOSI, MISO, SCK, CS) | 高速データ転送 | ADC, ディスプレイ, Flash |
| **UART** | 9600bps-1Mbps | 2 (TX, RX) | シリアル通信 | GPS, Bluetooth, デバッグ |
| **PWM** | - | 1 | アナログ出力エミュレーション | サーボ, モーター速度, LED調光 |

#### 産業用・長距離通信

| プロトコル | 速度 | 距離 | 用途 | 例 |
|-----------|------|------|------|-----|
| **CAN Bus** | 1Mbps (CAN FD: 5Mbps) | 〜500m | 車載・産業用 | 自動車ECU, ロボット |
| **RS485** | 10Mbps | 〜1200m | マルチドロップ | 産業機器間通信 |
| **Modbus RTU** | 〜115.2kbps | 〜1200m (RS485上) | PLC・センサ | 工場自動化 |
| **1-Wire** | 16kbps | 〜300m | センサネットワーク | 温度センサ群 |

### I2C

```bash
# I2Cデバイスのスキャン
i2cdetect -y 1

# レジスタ読み取り
i2cget -y 1 0x48 0x00

# レジスタ書き込み
i2cset -y 1 0x48 0x01 0x60
```

**特徴**:
- アドレスベース（7bit: 0x00-0x7F、最大127デバイス）
- マルチデバイス対応（同一バス上に複数接続可）
- プルアップ抵抗必要（通常4.7kΩ、高速時は2.2kΩ推奨）

**速度モード**:
| モード | 速度 | 用途 |
|--------|------|------|
| Standard | 100kHz | 一般センサ |
| Fast | 400kHz | 多くのセンサ |
| Fast Plus (FM+) | 1MHz | 高速デバイス |
| High Speed | 3.4MHz | 特殊用途 |

### SPI

```bash
# SPIデバイス確認
ls /dev/spidev*
```

**特徴**:
- フルデュプレックス（同時送受信）
- チップセレクト（CS）でデバイス選択
- I2Cより高速だが配線が多い

### UART

```bash
# シリアルポート確認
ls /dev/ttyUSB* /dev/ttyAMA* /dev/ttyACM*

# minicom でシリアル接続
minicom -D /dev/ttyUSB0 -b 115200

# Python で UART 通信
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 115200)"
```

**特徴**:
- ポイントツーポイント通信（1対1）
- ボーレート設定が必要（両端で一致必須）
- デバッグ用途に最適
- I2C/SPIより長距離通信可能

### CAN Bus

```bash
# CAN インターフェース設定（Linux）
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# CAN メッセージ送信
cansend can0 123#DEADBEEF

# CAN メッセージ受信
candump can0
```

**特徴**:
- 差動信号（CAN-H, CAN-L）でノイズ耐性高
- マルチマスター対応（優先度ベース調停）
- 両端に120Ω終端抵抗必要
- 自動車・産業用途で標準

**CAN の種類**:
| 規格 | ペイロード | 速度 | 用途 |
|------|----------|------|------|
| CAN 2.0A | 8 bytes | 1Mbps | 従来車載 |
| CAN 2.0B | 8 bytes | 1Mbps | 拡張ID（29bit） |
| CAN FD | 64 bytes | 5Mbps | 新世代車載 |
| CAN XL | 2048 bytes | 10Mbps | 次世代（開発中） |

### RS485 / Modbus

```bash
# USB-RS485アダプタ経由でModbus RTU通信
pip install pymodbus

# Python Modbus クライアント例
python3 << 'EOF'
from pymodbus.client import ModbusSerialClient
client = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=9600)
client.connect()
result = client.read_holding_registers(0, 10, slave=1)
print(result.registers)
EOF
```

**特徴**:
- RS485: 物理層規格（差動信号、マルチドロップ）
- Modbus: RS485上で動作するプロトコル
- 最大32デバイス接続可能
- 工場自動化で広く使用

## Sensor Catalog

### 温度・環境センサ

| センサ | インターフェース | 精度 | 範囲 | 備考 |
|--------|----------------|------|------|------|
| DS18B20 | 1-Wire | ±0.5°C | -55〜125°C | 防水プローブあり |
| BME280 | I2C/SPI | ±1°C | -40〜85°C | 温度+湿度+気圧 |
| BME680 | I2C/SPI | ±1°C | -40〜85°C | +VOC空気質センサ |
| SHT40 | I2C | ±0.2°C | -40〜125°C | 高精度温湿度 |
| TMP117 | I2C | ±0.1°C | -55〜150°C | 医療グレード精度 |
| MLX90614 | I2C | ±0.5°C | -70〜380°C | 非接触（赤外線） |

### 空気質センサ

| センサ | インターフェース | 測定対象 | 備考 |
|--------|----------------|----------|------|
| SCD40/SCD41 | I2C | CO2 (400-5000ppm) | 光音響方式、高精度 |
| SGP40 | I2C | VOC | VOCインデックス |
| PMS5003 | UART | PM1.0/2.5/10 | パーティクルセンサ |
| MH-Z19 | UART/PWM | CO2 | NDIR方式 |
| CCS811 | I2C | eCO2, TVOC | 低コスト |

### 距離センサ

| センサ | 方式 | 範囲 | 精度 | 備考 |
|--------|------|------|------|------|
| HC-SR04 | 超音波 | 2cm-4m | ±3mm | 低コスト |
| VL53L0X | ToF (レーザー) | 〜2m | ±3% | I2C, 高精度 |
| VL53L1X | ToF (レーザー) | 〜4m | ±3% | VL53L0X後継 |
| VL53L5CX | ToF マルチゾーン | 〜4m | - | 8x8ゾーン測距 |
| TFmini Plus | LiDAR | 0.1-12m | ±1% | UART/I2C, 屋外可 |
| RPLIDAR A1 | 2D LiDAR | 0.15-12m | - | 360° スキャン |

### IMU（慣性計測ユニット）

| センサ | 軸数 | インターフェース | 備考 |
|--------|------|----------------|------|
| MPU6050 | 6軸 (加速度+ジャイロ) | I2C | 低コスト、DMP内蔵 |
| ICM-42688-P | 6軸 | I2C/SPI | 高精度、低ノイズ |
| BMI270 | 6軸 | I2C/SPI | ウェアラブル向け |
| BNO085 | 9軸 | I2C/SPI | 高精度フュージョン |
| ICM-20948 | 9軸 | I2C/SPI | 低消費電力 |

### カメラ

| カメラ | 解像度 | インターフェース | 備考 |
|--------|--------|----------------|------|
| Raspberry Pi Camera v3 | 12MP | CSI | Sony IMX708, AF対応 |
| Raspberry Pi AI Camera | 12MP | CSI | Sony IMX500, AI内蔵 |
| Intel RealSense D435i | 1080p + Depth + IMU | USB3 | 深度カメラ |
| OAK-D Lite | 4K + Depth | USB3 | Luxonis, エッジAI内蔵 |
| OAK-D Pro | 4K + Depth | USB3 | ToF深度、IR照明 |
| ESP32-CAM | 2MP | WiFi | 低コスト、WiFi内蔵 |

### ToF / 深度センサ

| センサ | 解像度 | 範囲 | インターフェース | 備考 |
|--------|--------|------|----------------|------|
| VL53L5CX | 8x8 | 〜4m | I2C | STマルチゾーン |
| TMF8820/8821 | 3x3/4x4 | 〜5m | I2C | AMS製 |
| Intel RealSense L515 | 1024x768 | 0.25-9m | USB3 | LiDAR深度 |

## Edge AI / TinyML

### AIアクセラレータ

| デバイス | 演算性能 | 消費電力 | インターフェース | 用途 |
|---------|---------|---------|----------------|------|
| Google Coral USB | 4 TOPS | 2W | USB | 画像分類、物体検出 |
| Google Coral M.2 | 4 TOPS | 2W | M.2/PCIe | 組み込み向け |
| Intel Neural Compute Stick 2 | 1 TOPS | 1.5W | USB | OpenVINO推論 |
| Hailo-8 | 26 TOPS | 2.5W | M.2/PCIe | 高性能推論 |
| Syntiant NDP120 | - | 1mW | I2S/SPI | 音声AI、超低消費電力 |

### TinyML フレームワーク

| フレームワーク | 対象 | 特徴 |
|---------------|------|------|
| TensorFlow Lite Micro | MCU (ARM Cortex-M) | Google公式、広いサポート |
| Edge Impulse | MCU/MPU | 開発プラットフォーム、GUI |
| ONNX Runtime | MCU/MPU | クロスプラットフォーム |
| microTVM | MCU | Apache TVM、自動最適化 |

### TinyML 対応ボード

| ボード | MCU | RAM | Flash | 特徴 |
|--------|-----|-----|-------|------|
| Arduino Nano 33 BLE Sense | nRF52840 | 256KB | 1MB | 複合センサ内蔵 |
| ESP32-S3 | Xtensa LX7 | 512KB | 8MB+ | WiFi/BLE、AI対応 |
| Raspberry Pi Pico | RP2040 | 264KB | 2MB | 低コスト |
| STM32H7 | Cortex-M7 | 1MB | 2MB | 高性能MCU |
| MAX78000 | Cortex-M4 + CNN | 512KB | 512KB | CNN内蔵 |

### モデル最適化手法

```python
# TensorFlow Lite 量子化の例
import tensorflow as tf

converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_dir)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_types = [tf.int8]  # INT8量子化
tflite_model = converter.convert()
```

**最適化テクニック**:
- **量子化**: FP32→INT8で75%サイズ削減、推論高速化
- **プルーニング**: 不要な重みを削除
- **知識蒸留**: 大きなモデルから小さなモデルへ知識転送

## Actuator Catalog

### モーター

| 種類 | 制御方式 | 用途 | 備考 |
|------|---------|------|------|
| DCモーター | PWM + H-Bridge | 移動ロボット | L298N, TB6612 |
| サーボモーター | PWM (50Hz) | 関節、パン・チルト | SG90, MG996R |
| ステッピングモーター | パルス | 精密位置決め | A4988, TMC2209 |
| ブラシレスDC | ESC | ドローン、高速回転 | - |

### サーボモーター制御

```python
# PWM周期: 20ms (50Hz)
# パルス幅: 0.5ms (-90°) 〜 2.5ms (+90°)

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
pwm = GPIO.PWM(18, 50)  # 50Hz
pwm.start(7.5)  # 中央位置 (1.5ms / 20ms * 100)

# 角度を変更
pwm.ChangeDutyCycle(2.5)   # -90°
pwm.ChangeDutyCycle(12.5)  # +90°
```

### リレー・ソレノイド

| 種類 | 制御 | 用途 |
|------|------|------|
| メカニカルリレー | GPIO + トランジスタ | AC機器制御 |
| SSR（ソリッドステートリレー） | GPIO | 高頻度スイッチング |
| ソレノイドバルブ | GPIO + MOSFET | 流体制御 |

## Power Management

### 電源設計の基本

```
必要電流 = Σ(各デバイスの最大消費電流) × 1.2 (マージン)
```

### 典型的な消費電流

| デバイス | 電圧 | 電流 | 備考 |
|---------|------|------|------|
| Raspberry Pi 5 | 5V | 800mA-2.5A | アイドル〜高負荷 |
| Raspberry Pi 4 | 5V | 600mA-1.2A | アイドル〜高負荷 |
| Raspberry Pi Zero 2W | 5V | 100-350mA | 省電力用途 |
| Jetson Nano | 5V | 2-4A | 10W/5Wモード |
| Jetson Orin Nano | 5-19V | 7-15W | AI性能向上 |
| ESP32 | 3.3V | 80-240mA | WiFi使用時 |
| ESP32 (Deep Sleep) | 3.3V | 10μA | 省電力モード |
| サーボ SG90 | 5V | 100-650mA | 無負荷〜ストール |
| DCモーター (小型) | 3-12V | 100mA-1A | サイズ依存 |
| センサ類 | 3.3-5V | 1-50mA | 種類による |
| AIアクセラレータ | 5V | 500mA-1A | 推論時 |

### 注意事項

- **レベル変換**: 3.3V ↔ 5V 変換が必要な場合あり
- **逆起電力**: モーター駆動時はフライバックダイオード必須
- **デカップリング**: 各ICの電源ピン近くに0.1μFコンデンサ
- **GND共通化**: すべてのGNDを接続

## Troubleshooting

### I2Cデバイスが見つからない

```bash
# 1. 配線確認（SDA, SCL, VCC, GND）
# 2. プルアップ抵抗確認（4.7kΩ）
# 3. I2C有効化確認
sudo raspi-config  # Interface Options → I2C

# 4. スキャン
i2cdetect -y 1
```

### SPIが動作しない

```bash
# 1. SPI有効化確認
ls /dev/spidev*

# 2. カーネルモジュール確認
lsmod | grep spi
```

### シリアル通信が不安定

- ボーレート一致確認
- GND接続確認
- ケーブル長（長すぎると信号劣化）
- ノイズ対策（ツイストペア、シールド）

### CAN Busが動作しない

```bash
# 1. CANインターフェース状態確認
ip -details link show can0

# 2. エラーカウンタ確認
cat /sys/class/net/can0/statistics/rx_errors

# 3. 終端抵抗確認（両端に120Ω必要）
# マルチメータで CAN-H と CAN-L 間を測定 → 約60Ω（並列）

# 4. バスオフ状態からの復帰
sudo ip link set can0 type can restart-ms 100
```

### RS485/Modbusが動作しない

- **方向制御**: RS485は半二重、DE/RE ピン制御が必要
- **終端抵抗**: バス両端に120Ω
- **バイアス抵抗**: アイドル状態安定化（470Ω〜1kΩ）
- **スレーブアドレス**: 1-247の範囲、重複禁止
- **応答タイムアウト**: 十分な待ち時間設定

## Best Practices

### プロトコル選択フローチャート

```
デバイス数は？
├── 1台 → ポイントツーポイント？
│   ├── 高速必要 → SPI
│   └── シンプル → UART
└── 複数台
    ├── 短距離（<1m）
    │   ├── 低〜中速 → I2C
    │   └── 高速 → SPI + CS切替
    └── 長距離（>1m）
        ├── 産業用/車載 → CAN Bus
        └── レガシー/PLC → RS485/Modbus
```

### デバッグツール

| ツール | 用途 | 備考 |
|--------|------|------|
| Logic Analyzer | 全プロトコル波形解析 | Saleae, DSLogic |
| I2C Scanner | I2Cアドレス検出 | i2cdetect |
| CAN Analyzer | CANフレーム解析 | PCAN, CANalyzer |
| オシロスコープ | 信号品質確認 | タイミング、ノイズ |
