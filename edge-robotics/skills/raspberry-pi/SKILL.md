---
name: raspberry-pi
description: Use when developing on Raspberry Pi. Covers GPIO control, camera modules, HATs, OS configuration, and peripheral connections.
tags:
  - raspberry-pi
  - gpio
  - embedded
  - linux
---

# Raspberry Pi Development Guide

Raspberry Pi での開発・運用をサポート。GPIO制御、カメラ、周辺機器接続のガイド。

## When to Activate

- Raspberry Pi で開発・運用するとき
- GPIO ピン配置を確認したいとき
- カメラモジュールを設定するとき
- HAT/pHAT を接続するとき
- Raspberry Pi OS の設定を変更するとき

## Device Overview

| モデル | CPU | RAM | GPIO | 特徴 |
|--------|-----|-----|------|------|
| Pi 5 | 4x Cortex-A76 2.4GHz | 1-16GB | 40pin | RP1チップ, PCIe 2.0, USB3 5Gbps x2 |
| Pi 4 Model B | 4x Cortex-A72 1.5GHz | 1-8GB | 40pin | USB3, Gigabit Ethernet |
| Pi Zero 2 W | 4x Cortex-A53 1GHz | 512MB | 40pin | 小型、WiFi内蔵 |
| Pi 400 | 4x Cortex-A72 1.8GHz | 4GB | 40pin | キーボード一体型 |

### Pi 5 の新機能

- **RP1 サウスブリッジ**: Raspberry Pi 自社設計チップ。GPIO、USB、Ethernetなど I/O を担当
- **PCIe 2.0 x1**: NVMe SSD など高速ストレージ接続可能
- **デュアル 4Kp60 ディスプレイ**: VideoCore VII GPU による高性能グラフィック
- **電源ボタン**: 本体に電源ボタンを内蔵
- **リアルタイムクロック**: バッテリーバックアップ可能な RTC 内蔵
- **推奨電源**: 5V 5A（27W）USB-C 電源アダプター

## GPIO Pinout

### 40ピンヘッダー配置

```
                    3.3V [1]  [2]  5V
           GPIO2 (SDA1) [3]  [4]  5V
           GPIO3 (SCL1) [5]  [6]  GND
                 GPIO4  [7]  [8]  GPIO14 (TXD)
                   GND  [9]  [10] GPIO15 (RXD)
                GPIO17 [11]  [12] GPIO18 (PWM0)
                GPIO27 [13]  [14] GND
                GPIO22 [15]  [16] GPIO23
                  3.3V [17]  [18] GPIO24
      GPIO10 (SPI MOSI) [19]  [20] GND
       GPIO9 (SPI MISO) [21]  [22] GPIO25
      GPIO11 (SPI SCLK) [23]  [24] GPIO8 (SPI CE0)
                   GND [25]  [26] GPIO7 (SPI CE1)
          GPIO0 (ID_SD) [27]  [28] GPIO1 (ID_SC)
                 GPIO5 [29]  [30] GND
                 GPIO6 [31]  [32] GPIO12 (PWM0)
          GPIO13 (PWM1) [33]  [34] GND
          GPIO19 (PCM)  [35]  [36] GPIO16
                GPIO26 [37]  [38] GPIO20 (PCM)
                   GND [39]  [40] GPIO21 (PCM)
```

### 特殊機能ピン

| ピン | 機能 | 備考 |
|------|------|------|
| GPIO2, 3 | I2C1 (SDA, SCL) | 内蔵プルアップあり |
| GPIO14, 15 | UART (TXD, RXD) | シリアル通信 |
| GPIO10, 9, 11, 8, 7 | SPI0 | MOSI, MISO, SCLK, CE0, CE1 |
| GPIO12, 13, 18, 19 | PWM | ハードウェアPWM |
| GPIO4 | 1-Wire | DS18B20等 |

## GPIO Control

### 重要: Pi 5 の GPIO アーキテクチャ変更

Pi 5 では RP1 チップが GPIO を管理するため、従来のライブラリとの互換性に注意が必要。

| ライブラリ | Pi 4以前 | Pi 5 | 備考 |
|-----------|---------|------|------|
| gpiozero | ✅ | ✅ | **推奨**。lgpio バックエンド使用 |
| lgpio | ✅ | ✅ | gpiozero のデフォルトバックエンド |
| gpiod/libgpiod | ✅ | ✅ | 低レベル制御向け |
| RPi.GPIO | ✅ | ❌ | Pi 5 非対応 |
| pigpio | ✅ | ❌ | Pi 5 非対応 |

### Python (gpiozero) - 推奨

```python
from gpiozero import LED, Button, PWMOutputDevice
from time import sleep

# LED
led = LED(17)
led.on()
sleep(1)
led.off()
led.blink(on_time=1, off_time=1)

# ボタン
button = Button(18)
button.when_pressed = lambda: print("Pressed!")
button.wait_for_press()

# PWM
pwm = PWMOutputDevice(12)
pwm.value = 0.5  # 50%
```

### Python (gpiozero + lgpio) - Pi 5 明示的設定

```python
from gpiozero import Device, LED, Button
from gpiozero.pins.lgpio import LGPIOFactory

# Pi 5 では明示的に lgpio バックエンドを設定（推奨）
Device.pin_factory = LGPIOFactory(chip=4)  # Pi 5 は gpiochip4

led = LED(17)
led.on()
```

### Python (gpiod) - 低レベル制御

```python
import gpiod
import time

# Pi 5: gpiochip4, Pi 4以前: gpiochip0
# 2024年7月以降のカーネルでは Pi 5 も gpiochip0 に統一される予定
CHIP = 'gpiochip4'  # Pi 5
LINE = 17

chip = gpiod.Chip(CHIP)
line = chip.get_line(LINE)

# 出力設定
line.request(consumer="example", type=gpiod.LINE_REQ_DIR_OUT)
line.set_value(1)
time.sleep(1)
line.set_value(0)
line.release()
```

### Python (rpi-lgpio) - RPi.GPIO 互換レイヤー

```python
# RPi.GPIO のコードを Pi 5 で動かす場合
# pip install rpi-lgpio
import RPi.GPIO as GPIO  # 実際は rpi-lgpio が呼ばれる

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)
GPIO.cleanup()
```

### コマンドライン (pinctrl)

```bash
# pinctrl: Pi 5 以降の推奨ツール（raspi-gpio の後継）
# 全ピン状態確認
pinctrl

# 特定ピン確認
pinctrl get 17

# 出力設定
pinctrl set 17 op dh  # output, drive high
pinctrl set 17 op dl  # output, drive low

# 入力設定（プルアップ付き）
pinctrl set 17 ip pu

# Pi 4 以前: raspi-gpio（レガシー）
raspi-gpio get 17
raspi-gpio set 17 op dh
```

### gpiochip の確認

```bash
# 利用可能な gpiochip 一覧
ls /dev/gpiochip*

# GPIO 状態詳細（要 root）
cat /sys/kernel/debug/gpio

# Pi 5 の gpiochip4 ベース番号確認
# 例: base=571 の場合、GPIO13 = 571 + 13 = 584
```

## Camera Module

### カメラモジュール比較

| モジュール | センサ | 解像度 | 特徴 |
|-----------|--------|--------|------|
| Camera Module v2 | Sony IMX219 | 8MP | 標準、固定フォーカス |
| Camera Module v3 | Sony IMX708 | 12MP | オートフォーカス対応 |
| Camera Module v3 Wide | Sony IMX708 | 12MP | 広角120°、AF対応 |
| Camera Module v3 NoIR | Sony IMX708 | 12MP | 赤外線対応、AF対応 |
| HQ Camera | Sony IMX477 | 12.3MP | 交換レンズ(C/CS)対応 |
| Global Shutter Camera | Sony IMX296 | 1.6MP | グローバルシャッター、60fps、高速撮影向け |

### Pi 5 カメラ接続の注意

Pi 5 では MIPI コネクタが変更されている。Camera Module v3 等を使う場合は対応ケーブルが必要。

```bash
# カメラ検出
libcamera-hello --list-cameras

# Pi 5 のカメラポート: CAM0, CAM1（両方使用可能）
```

### libcamera コマンドライン

```bash
# 静止画撮影
libcamera-still -o image.jpg

# 解像度・品質指定
libcamera-still -o image.jpg --width 4608 --height 2592 -q 95

# HDR 撮影（対応カメラのみ）
libcamera-still -o hdr.jpg --hdr

# 動画撮影（H.264）
libcamera-vid -o video.h264 -t 10000  # 10秒

# 動画撮影（MP4、音声付き）
libcamera-vid -o video.mp4 -t 10000 --codec libav

# タイムラプス
libcamera-still -o frame%04d.jpg -t 60000 --timelapse 1000

# プレビュー表示
libcamera-hello -t 0  # 無期限

# カメラ情報表示
libcamera-hello --list-cameras
```

### Python (Picamera2)

```python
from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()

# === 静止画撮影 ===
config = picam2.create_still_configuration(
    main={"size": (4608, 2592)},  # 最大解像度
    buffer_count=2
)
picam2.configure(config)
picam2.start()
time.sleep(2)  # ウォームアップ
picam2.capture_file("image.jpg")
picam2.stop()

# === プレビュー付き撮影 ===
picam2.start_preview(Preview.QTGL)  # GUI プレビュー
picam2.start()
time.sleep(3)
picam2.capture_file("preview_shot.jpg")
picam2.stop_preview()
picam2.stop()
```

### Picamera2 オートフォーカス制御（Camera Module v3）

```python
from picamera2 import Picamera2
from libcamera import controls
import time

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# マニュアルフォーカス
picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 0.5})

# シングルオートフォーカス（一度だけ合焦）
picam2.set_controls({"AfMode": controls.AfModeEnum.Auto, "AfTrigger": controls.AfTriggerEnum.Start})

# コンティニュアスオートフォーカス（常時）
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

time.sleep(2)
picam2.capture_file("focused.jpg")
picam2.stop()
```

### Picamera2 動画撮影

```python
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput

picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (1920, 1080)},
    controls={"FrameRate": 30}
)
picam2.configure(video_config)

encoder = H264Encoder(bitrate=10000000)
output = FfmpegOutput("video.mp4")

picam2.start_recording(encoder, output)
time.sleep(10)
picam2.stop_recording()
```

### Picamera2 + OpenCV

```python
from picamera2 import Picamera2
import cv2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888"}))
picam2.start()

while True:
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    cv2.imshow("Camera", gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
```

## HAT/pHAT

### HAT 設定

HAT（Hardware Attached on Top）は EEPROM で自動認識。

```bash
# HAT情報確認
cat /proc/device-tree/hat/product
cat /proc/device-tree/hat/vendor

# I2C HAT確認
i2cdetect -y 1
```

### 人気のHAT

| HAT | 用途 | インターフェース |
|-----|------|----------------|
| Sense HAT | 環境センシング | I2C |
| PoE+ HAT | Power over Ethernet | - |
| AI Kit (Hailo-8L) | AI アクセラレータ | PCIe (Pi 5) |
| NVMe Base | NVMe SSD 接続 | PCIe (Pi 5) |
| Motor HAT | モーター制御 | I2C |
| ADC HAT | アナログ入力 | SPI/I2C |
| RTC HAT | リアルタイムクロック | I2C |

## OS Configuration

### Raspberry Pi OS バージョン

| バージョン | ベース | サポート期限 | 備考 |
|-----------|--------|------------|------|
| Trixie | Debian 13 | 2027年頃 | 最新、Pi 5 推奨 |
| Bookworm | Debian 12 | 2026年12月 | 安定版 |
| Bullseye | Debian 11 | レガシー | Pi 5 非対応 |

### raspi-config

```bash
sudo raspi-config

# よく使う設定
# 1. System Options → Password
# 2. Interface Options → SSH, I2C, SPI, VNC
# 3. Performance Options → GPU Memory
# 4. Localisation Options → Timezone, Locale
```

### /boot/firmware/config.txt（Pi 5 / Bookworm以降）

```ini
# カメラ自動検出
camera_auto_detect=1

# GPUメモリ割り当て（デスクトップ使用時）
gpu_mem=256

# I2C有効化・高速化
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000

# SPI有効化
dtparam=spi=on

# 1-Wire有効化（GPIO4）
dtoverlay=w1-gpio

# UART有効化
enable_uart=1

# Pi 5: ファン制御
# cooling_fan=1

# Pi 5: PCIe Gen 3 有効化（オプション、発熱注意）
# dtparam=pciex1_gen=3
```

### systemd サービス

```bash
# 自動起動サービス作成
sudo nano /etc/systemd/system/myapp.service
```

```ini
[Unit]
Description=My Application
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/app.py
WorkingDirectory=/home/pi
User=pi
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# 有効化・起動
sudo systemctl daemon-reload
sudo systemctl enable myapp
sudo systemctl start myapp
sudo systemctl status myapp
```

## Networking

### NetworkManager（Bookworm以降推奨）

```bash
# 接続一覧
nmcli connection show

# WiFi 接続
nmcli device wifi list
nmcli device wifi connect "SSID" password "password"

# 静的IP設定
nmcli connection modify "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 192.168.1.100/24 \
    ipv4.gateway 192.168.1.1 \
    ipv4.dns "8.8.8.8,8.8.4.4"
nmcli connection up "Wired connection 1"

# WiFi AP モード（ホットスポット）
nmcli device wifi hotspot ssid "MyPiAP" password "password123"
```

### dhcpcd（レガシー、Bullseye以前）

```bash
# /etc/dhcpcd.conf に追加
interface eth0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8
```

## Troubleshooting

### GPIO が動作しない

```bash
# Pi 5: gpiochip 確認
ls -la /dev/gpiochip*

# 権限確認
groups  # gpio グループに所属しているか

# グループ追加（再ログイン必要）
sudo usermod -aG gpio $USER

# RPi.GPIO エラーが出る場合（Pi 5）
# → gpiozero または rpi-lgpio を使用
pip install rpi-lgpio  # RPi.GPIO 互換レイヤー
```

### I2Cデバイスが見つからない

```bash
# I2C有効確認
ls /dev/i2c*

# 有効化（raspi-config）
sudo raspi-config
# Interface Options → I2C → Enable

# または config.txt に追加
# dtparam=i2c_arm=on

# スキャン
i2cdetect -y 1
```

### カメラが動作しない

```bash
# ケーブル接続確認（コネクタの向き注意）
# Pi 5: 22pin→15pin 変換ケーブルが必要な場合あり

# カメラ検出
libcamera-hello --list-cameras

# 何も表示されない場合
# 1. ケーブル接続を確認
# 2. config.txt に camera_auto_detect=1 があるか確認
# 3. 再起動

# ログ確認
dmesg | grep -i camera
journalctl -u camera
```

### SDカード破損防止

```bash
# Read-only ファイルシステム（overlayfs）
sudo raspi-config
# Performance Options → Overlay File System → Enable

# または手動で /etc/fstab に ro オプション追加
# 開発完了後の本番運用に推奨
```

### 温度・スロットリング確認

```bash
# CPU温度
vcgencmd measure_temp

# スロットリング状態
vcgencmd get_throttled
# 0x0 = 正常
# 0x50000 = 過去にスロットリング発生

# Pi 5: 適切な冷却が重要（アクティブクーラー推奨）
```

## Command Reference

| コマンド | 説明 |
|---------|------|
| `pinctrl` | GPIO状態確認・制御 (Pi 5) |
| `pinctrl get 17` | GPIO17 の状態確認 |
| `pinctrl set 17 op dh` | GPIO17 を出力HIGHに |
| `raspi-gpio get` | GPIO状態確認 (Pi 4以前) |
| `i2cdetect -y 1` | I2Cデバイススキャン |
| `libcamera-hello` | カメラプレビュー |
| `libcamera-still -o file.jpg` | 静止画撮影 |
| `libcamera-vid -o file.h264` | 動画撮影 |
| `vcgencmd measure_temp` | CPU温度 |
| `vcgencmd get_throttled` | スロットリング状態 |
| `raspi-config` | 設定ツール |
| `nmcli` | ネットワーク管理 (Bookworm+) |
