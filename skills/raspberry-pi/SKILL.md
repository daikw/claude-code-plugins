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
| Pi 4 Model B | 4x Cortex-A72 1.5GHz | 1-8GB | 40pin | USB3, Gigabit Ethernet |
| Pi 5 | 4x Cortex-A76 2.4GHz | 4-8GB | 40pin | PCIe, 高性能 |
| Pi Zero 2 W | 4x Cortex-A53 1GHz | 512MB | 40pin | 小型、WiFi内蔵 |
| Pi 400 | 4x Cortex-A72 1.8GHz | 4GB | 40pin | キーボード一体型 |

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

### Python (RPi.GPIO)

```python
import RPi.GPIO as GPIO
import time

# セットアップ
GPIO.setmode(GPIO.BCM)  # BCM番号を使用
GPIO.setwarnings(False)

# 出力
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)
time.sleep(1)
GPIO.output(17, GPIO.LOW)

# 入力
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
if GPIO.input(18) == GPIO.LOW:
    print("Button pressed")

# PWM
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12, 1000)  # 1kHz
pwm.start(50)  # 50% duty cycle
pwm.ChangeDutyCycle(75)
pwm.stop()

# クリーンアップ
GPIO.cleanup()
```

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

### コマンドライン (pinctrl)

```bash
# Pi 5 以降
pinctrl get 17
pinctrl set 17 op dh  # output, drive high
pinctrl set 17 op dl  # output, drive low

# 旧来の方法 (raspi-gpio)
raspi-gpio get 17
raspi-gpio set 17 op dh
```

## Camera Module

### カメラ有効化

```bash
# raspi-config で有効化
sudo raspi-config
# Interface Options → Camera → Enable

# または /boot/config.txt に追加
# Pi 4以前: start_x=1, gpu_mem=128
# Pi 5: camera_auto_detect=1
```

### libcamera (推奨、Pi 5対応)

```bash
# 静止画撮影
libcamera-still -o image.jpg

# 解像度指定
libcamera-still -o image.jpg --width 1920 --height 1080

# 動画撮影
libcamera-vid -o video.h264 -t 10000  # 10秒

# プレビュー
libcamera-hello

# 利用可能なカメラ一覧
libcamera-hello --list-cameras
```

### Python (Picamera2)

```python
from picamera2 import Picamera2
import time

# 初期化
picam2 = Picamera2()

# 静止画設定
config = picam2.create_still_configuration(
    main={"size": (1920, 1080)}
)
picam2.configure(config)

# 撮影
picam2.start()
time.sleep(2)  # ウォームアップ
picam2.capture_file("image.jpg")
picam2.stop()
```

### カメラモジュール比較

| モジュール | センサ | 解像度 | 特徴 |
|-----------|--------|--------|------|
| Camera Module v2 | Sony IMX219 | 8MP | 標準 |
| Camera Module v3 | Sony IMX708 | 12MP | AF対応 |
| Camera Module v3 Wide | Sony IMX708 | 12MP | 広角120° |
| HQ Camera | Sony IMX477 | 12.3MP | 交換レンズ対応 |

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
| Motor HAT | モーター制御 | I2C |
| ADC HAT | アナログ入力 | SPI/I2C |
| RTC HAT | リアルタイムクロック | I2C |

## OS Configuration

### raspi-config

```bash
sudo raspi-config

# よく使う設定
# 1. System Options → Password
# 2. Interface Options → SSH, I2C, SPI, Camera
# 3. Performance Options → GPU Memory
# 4. Localisation Options → Timezone, Locale
```

### /boot/config.txt (Pi 4以前) / /boot/firmware/config.txt (Pi 5)

```ini
# GPUメモリ割り当て
gpu_mem=256

# オーバークロック
#arm_freq=2000  # Pi 4
#over_voltage=6

# I2C高速化
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000

# SPI有効化
dtparam=spi=on

# 1-Wire有効化
dtoverlay=w1-gpio

# PWMオーディオ無効化（GPIO12,13使用時）
dtparam=audio=off
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

[Install]
WantedBy=multi-user.target
```

```bash
# 有効化
sudo systemctl enable myapp
sudo systemctl start myapp
sudo systemctl status myapp
```

## Networking

### 静的IP設定

```bash
# /etc/dhcpcd.conf に追加
interface eth0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8
```

### WiFi設定

```bash
# raspi-config で設定
sudo raspi-config
# System Options → Wireless LAN

# または手動
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

```conf
country=JP
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="MyNetwork"
    psk="password"
}
```

## Troubleshooting

### GPIOが動作しない

```bash
# 権限確認
groups  # gpio グループに所属しているか

# グループ追加
sudo usermod -aG gpio $USER
# 再ログイン必要

# /dev/gpiomem 確認
ls -la /dev/gpiomem
```

### I2Cデバイスが見つからない

```bash
# I2C有効確認
ls /dev/i2c*

# 有効化
sudo raspi-config
# Interface Options → I2C → Enable

# スキャン
i2cdetect -y 1
```

### カメラが動作しない

```bash
# ケーブル接続確認（コネクタの向き注意）

# Pi 5の場合、正しいポート確認
# CAM0 または CAM1

# カメラ検出
libcamera-hello --list-cameras

# ログ確認
dmesg | grep -i camera
```

### SDカード破損防止

```bash
# Read-onlyマウント（開発完了後）
# /etc/fstab に ro オプション追加

# または overlayfs 使用
sudo raspi-config
# Performance Options → Overlay File System
```

## Command Reference

| コマンド | 説明 |
|---------|------|
| `pinctrl get` | GPIO状態確認 (Pi 5) |
| `raspi-gpio get` | GPIO状態確認 (Pi 4以前) |
| `i2cdetect -y 1` | I2Cデバイススキャン |
| `libcamera-hello` | カメラプレビュー |
| `libcamera-still -o file.jpg` | 静止画撮影 |
| `vcgencmd measure_temp` | CPU温度 |
| `vcgencmd get_throttled` | スロットリング状態 |
| `raspi-config` | 設定ツール |
