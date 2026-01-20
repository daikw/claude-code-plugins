---
name: edge-common
description: Use when working with edge devices (Jetson, Raspberry Pi). Provides sensor/actuator catalogs and communication protocol guides.
tags:
  - edge-computing
  - sensors
  - actuators
  - embedded
---

# Edge Computing Common Knowledge

エッジデバイス（Jetson、Raspberry Pi等）で共通して使用する概念、通信プロトコル、センサ・アクチュエータの知識。

## When to Activate

- エッジデバイスでセンサやアクチュエータを接続するとき
- I2C, SPI, UART, GPIO の選択に迷ったとき
- 新しいセンサ・アクチュエータを調査するとき
- 電源設計・消費電力を検討するとき

## Communication Protocols

### プロトコル選択ガイド

| プロトコル | 速度 | 配線数 | 用途 | 例 |
|-----------|------|--------|------|-----|
| **GPIO** | - | 1 | デジタル入出力 | LED, ボタン, リレー |
| **I2C** | 100kHz-3.4MHz | 2 (SDA, SCL) | 低速センサ、設定 | 温度、加速度、OLED |
| **SPI** | 10MHz+ | 4+ (MOSI, MISO, SCK, CS) | 高速データ転送 | ADC, ディスプレイ, SD |
| **UART** | 9600-115200bps | 2 (TX, RX) | シリアル通信 | GPS, Bluetooth, デバッグ |
| **PWM** | - | 1 | アナログ出力エミュレーション | サーボ, モーター速度, LED調光 |

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
- アドレスベース（7bit: 0x00-0x7F）
- マルチデバイス対応（同一バス上に複数接続可）
- プルアップ抵抗必要（通常4.7kΩ）

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
```

**特徴**:
- ポイントツーポイント通信
- ボーレート設定が必要
- デバッグ用途に最適

## Sensor Catalog

### 温度センサ

| センサ | インターフェース | 精度 | 範囲 | 備考 |
|--------|----------------|------|------|------|
| DS18B20 | 1-Wire | ±0.5°C | -55〜125°C | 防水プローブあり |
| BME280 | I2C/SPI | ±1°C | -40〜85°C | 温度+湿度+気圧 |
| TMP102 | I2C | ±0.5°C | -40〜125°C | 低消費電力 |
| MLX90614 | I2C | ±0.5°C | -70〜380°C | 非接触（赤外線） |

### 距離センサ

| センサ | 方式 | 範囲 | 精度 | 備考 |
|--------|------|------|------|------|
| HC-SR04 | 超音波 | 2cm-4m | ±3mm | 低コスト |
| VL53L0X | ToF (レーザー) | 〜2m | ±3% | I2C, 高精度 |
| TFmini | LiDAR | 0.3-12m | ±1% | UART, 屋外可 |

### IMU（慣性計測ユニット）

| センサ | 軸数 | インターフェース | 備考 |
|--------|------|----------------|------|
| MPU6050 | 6軸 (加速度+ジャイロ) | I2C | 低コスト、DMP内蔵 |
| MPU9250 | 9軸 (+地磁気) | I2C/SPI | MPU6050後継 |
| BNO055 | 9軸 | I2C | センサフュージョン内蔵 |
| ICM-20948 | 9軸 | I2C/SPI | 低消費電力 |

### カメラ

| カメラ | 解像度 | インターフェース | 備考 |
|--------|--------|----------------|------|
| Raspberry Pi Camera v2 | 8MP | CSI | Sony IMX219 |
| Raspberry Pi Camera v3 | 12MP | CSI | Sony IMX708, AF対応 |
| Intel RealSense D435 | 1080p + Depth | USB3 | 深度カメラ |
| OAK-D | 4K + Depth | USB3 | エッジAI内蔵 |

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

| デバイス | 電圧 | 電流 |
|---------|------|------|
| Raspberry Pi 4 | 5V | 600mA-1.2A |
| Jetson Nano | 5V | 2-4A |
| サーボ SG90 | 5V | 100-650mA |
| DCモーター (小型) | 3-12V | 100mA-1A |
| センサ類 | 3.3-5V | 1-50mA |

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
