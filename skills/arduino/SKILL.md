---
name: arduino
description: Use when developing with Arduino framework. Covers Arduino IDE/CLI, libraries, PlatformIO, and multi-platform support.
tags:
  - arduino
  - embedded
  - iot
  - platformio
---

# Arduino Development Skill

Arduino フレームワークを使った組み込み開発の包括的なガイド。

## 1. Overview - Arduino エコシステム

### Arduino とは

Arduino は、オープンソースのハードウェア・ソフトウェアプラットフォームで、電子工作やプロトタイピングを簡単に行えるようにする開発環境。

### Official vs Third-Party Boards

| カテゴリ | ボード例 | 特徴 |
|----------|----------|------|
| **Official Arduino** | Uno R4, Nano 33 IoT, Mega 2560 | 公式サポート、高い互換性 |
| **ESP32** | ESP32-WROOM, ESP32-S3 | Wi-Fi/Bluetooth内蔵、デュアルコア |
| **STM32** | STM32F4, STM32L4 | 高性能ARM Cortex-M、産業グレード |
| **RP2040** | Raspberry Pi Pico, Pico W | デュアルコアARM Cortex-M0+、PIO |

### 開発環境の選択肢

1. **Arduino IDE 2.x** - 公式GUI、初心者向け
2. **Arduino CLI** - コマンドライン、CI/CD向け
3. **PlatformIO** - プロ向け、VS Code統合

---

## 2. Arduino IDE 2.x / Arduino CLI

### Arduino IDE 2.x (最新: v2.3.7+)

Arduino IDE 2.x は2024年に多くの改善が行われた。IDE 2.3では、デバッグ機能が正式リリースとなり、実験段階を終えた。

#### 主要機能

- **オートコンプリート**: コード補完機能
- **コードナビゲーション**: 関数定義へのジャンプ
- **ライブデバッガ**: ブレークポイント、変数ウォッチ
- **シリアルプロッタ**: グラフ表示付きシリアルモニタ
- **ダークモード**: 目に優しいテーマ
- **Project Explorer**: ファイル管理

#### インストールと設定

```bash
# macOS (Homebrew)
brew install --cask arduino-ide

# Linux (AppImage)
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.7_Linux_64bit.AppImage
chmod +x arduino-ide_2.3.7_Linux_64bit.AppImage
```

### Arduino CLI (最新: v1.4.0)

Arduino CLI 1.0 が2024年9月にリリースされ、gRPCインターフェースやGoモジュールを提供。

#### インストール

```bash
# macOS
brew install arduino-cli

# Linux
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Windows (PowerShell)
choco install arduino-cli
```

#### 基本コマンド

```bash
# 設定の初期化
arduino-cli config init

# ボードマネージャの更新
arduino-cli core update-index

# コアのインストール（例: AVR）
arduino-cli core install arduino:avr

# ESP32コアのインストール
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32

# ライブラリのインストール
arduino-cli lib install "Adafruit NeoPixel"

# コンパイル
arduino-cli compile --fqbn arduino:avr:uno MySketch

# アップロード
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno MySketch

# 接続ボードの検出
arduino-cli board list
```

#### Board Manager URLs (サードパーティボード)

```bash
# ESP32
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# ESP8266
http://arduino.esp8266.com/stable/package_esp8266com_index.json

# RP2040 (Earle Philhower's Core)
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

# STM32
https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
```

---

## 3. Core API

### Digital I/O

```cpp
// ピン設定
pinMode(LED_BUILTIN, OUTPUT);  // 出力モード
pinMode(BUTTON_PIN, INPUT_PULLUP);  // 内部プルアップ付き入力

// デジタル出力
digitalWrite(LED_BUILTIN, HIGH);
digitalWrite(LED_BUILTIN, LOW);

// デジタル入力
int buttonState = digitalRead(BUTTON_PIN);
```

### Analog I/O

```cpp
// アナログ入力（10bit: 0-1023、ESP32は12bit: 0-4095）
int sensorValue = analogRead(A0);
float voltage = sensorValue * (3.3 / 1023.0);

// アナログ出力（PWM）
analogWrite(PWM_PIN, 128);  // 0-255（8bit）

// ESP32では解像度変更可能
analogReadResolution(12);  // 12bit
analogWriteResolution(10);  // 10bit PWM
```

### Serial 通信

```cpp
void setup() {
    Serial.begin(115200);  // ボーレート設定
    while (!Serial) { ; }  // USB接続待ち（Leonardo等）
}

void loop() {
    if (Serial.available() > 0) {
        String received = Serial.readStringUntil('\n');
        Serial.println("Received: " + received);
    }
}

// 複数シリアルポート（ESP32, STM32等）
Serial1.begin(9600);  // UART1
Serial2.begin(9600);  // UART2
```

### Wire (I2C)

```cpp
#include <Wire.h>

void setup() {
    Wire.begin();  // マスターとして初期化
    // Wire.begin(SDA_PIN, SCL_PIN);  // ESP32: ピン指定可能
}

void readI2CDevice(uint8_t address) {
    Wire.beginTransmission(address);
    Wire.write(0x00);  // レジスタアドレス
    Wire.endTransmission(false);  // リピーテッドスタート

    Wire.requestFrom(address, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte = Wire.read();
        int16_t value = (highByte << 8) | lowByte;
    }
}

// I2Cスキャナ
void scanI2C() {
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Device found at 0x%02X\n", addr);
        }
    }
}
```

### SPI

```cpp
#include <SPI.h>

// SPISettings: 速度, ビットオーダー, モード
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

void setup() {
    SPI.begin();
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
}

uint8_t spiTransfer(uint8_t data) {
    digitalWrite(CS_PIN, LOW);
    SPI.beginTransaction(settings);
    uint8_t result = SPI.transfer(data);
    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);
    return result;
}

// 複数バイト転送
void spiTransferBulk(uint8_t* buffer, size_t length) {
    digitalWrite(CS_PIN, LOW);
    SPI.beginTransaction(settings);
    SPI.transfer(buffer, length);
    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);
}
```

### PWM（高度な制御）

```cpp
// ESP32: LEDC (LED Control)
#include "esp32-hal-ledc.h"

void setup() {
    // チャンネル0, 5000Hz, 8bit解像度
    ledcSetup(0, 5000, 8);
    ledcAttachPin(LED_PIN, 0);
}

void loop() {
    // Fade効果
    for (int duty = 0; duty <= 255; duty++) {
        ledcWrite(0, duty);
        delay(10);
    }
}

// RP2040: PWM
// 自動的に適切なスライスとチャンネルを使用
analogWriteFreq(1000);  // 周波数設定
analogWriteRange(1000);  // 解像度設定
analogWrite(LED_PIN, 500);  // 50% duty
```

---

## 4. Libraries

### 人気ライブラリ

| カテゴリ | ライブラリ | 用途 |
|----------|------------|------|
| **ディスプレイ** | Adafruit_GFX, U8g2, TFT_eSPI | OLED, LCD, TFT |
| **センサー** | Adafruit_Sensor, DHT, BME280 | 温湿度、気圧 |
| **通信** | PubSubClient, ArduinoJson | MQTT, JSON |
| **LED** | FastLED, Adafruit NeoPixel | WS2812B等 |
| **モーター** | AccelStepper, Servo | ステッピング、サーボ |
| **ファイルシステム** | LittleFS, SPIFFS | フラッシュストレージ |

### library.properties の形式

```properties
name=MyAwesomeLibrary
version=1.0.0
author=Your Name <email@example.com>
maintainer=Your Name <email@example.com>
sentence=A short description of the library.
paragraph=A longer description of the library.
category=Sensors
url=https://github.com/yourname/MyAwesomeLibrary
architectures=*
includes=MyAwesomeLibrary.h
depends=Wire,SPI
```

### ライブラリの作成

```
MyLibrary/
├── src/
│   ├── MyLibrary.h
│   └── MyLibrary.cpp
├── examples/
│   └── BasicExample/
│       └── BasicExample.ino
├── library.properties
├── keywords.txt
└── README.md
```

#### ヘッダファイル (MyLibrary.h)

```cpp
#ifndef MY_LIBRARY_H
#define MY_LIBRARY_H

#include <Arduino.h>

class MyLibrary {
public:
    MyLibrary(uint8_t pin);
    void begin();
    void update();
    int getValue();

private:
    uint8_t _pin;
    int _value;
};

#endif
```

#### 実装ファイル (MyLibrary.cpp)

```cpp
#include "MyLibrary.h"

MyLibrary::MyLibrary(uint8_t pin) : _pin(pin), _value(0) {}

void MyLibrary::begin() {
    pinMode(_pin, INPUT);
}

void MyLibrary::update() {
    _value = analogRead(_pin);
}

int MyLibrary::getValue() {
    return _value;
}
```

#### keywords.txt（シンタックスハイライト用）

```
# クラス名
MyLibrary	KEYWORD1

# メソッド名
begin	KEYWORD2
update	KEYWORD2
getValue	KEYWORD2

# 定数
MY_CONSTANT	LITERAL1
```

---

## 5. PlatformIO

PlatformIO は900以上のボードをサポートする強力な代替開発環境。VS Code との統合が優れている。

### インストール

```bash
# VS Code Extension
# 1. VS Code を開く
# 2. Extensions (Ctrl+Shift+X) で "PlatformIO" を検索
# 3. Install

# CLI のみ
pip install platformio
# または
brew install platformio
```

### platformio.ini

```ini
[platformio]
default_envs = esp32dev

; ESP32 環境
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    adafruit/Adafruit NeoPixel@^1.12.0
    bblanchon/ArduinoJson@^7.0.0
build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_HAS_PSRAM
upload_speed = 921600

; Arduino Uno 環境
[env:uno]
platform = atmelavr
board = uno
framework = arduino

; Raspberry Pi Pico
[env:pico]
platform = raspberrypi
board = pico
framework = arduino

; STM32 Blue Pill
[env:bluepill]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = stlink

; 共通設定
[env]
check_tool = clangtidy
check_flags =
    clangtidy: --checks=-*,cert-*,clang-analyzer-*
```

### PlatformIO の利点

| 機能 | Arduino IDE | PlatformIO |
|------|-------------|------------|
| **デバッグ** | 限定的 | 完全サポート |
| **依存関係管理** | 手動 | 自動（platformio.ini） |
| **マルチ環境** | 不可 | 可能 |
| **ユニットテスト** | なし | 組み込みサポート |
| **静的解析** | なし | clang-tidy対応 |
| **CI/CD統合** | 限定的 | 優れている |

### PlatformIO CLI コマンド

```bash
# プロジェクト初期化
pio project init --board esp32dev

# ビルド
pio run

# 特定環境のみビルド
pio run -e esp32dev

# アップロード
pio run -t upload

# シリアルモニタ
pio device monitor

# ライブラリ検索
pio pkg search "neopixel"

# ライブラリインストール
pio pkg install -l "Adafruit NeoPixel"

# ユニットテスト
pio test

# クリーン
pio run -t clean
```

---

## 6. ESP32 / STM32 / RP2040 with Arduino

### ESP32

デュアルコア、Wi-Fi/Bluetooth内蔵。IoTプロジェクトに最適。

```cpp
// ESP32 Wi-Fi 接続
#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    WiFi.begin("SSID", "password");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.println(WiFi.localIP());
}

// デュアルコアタスク
void task1(void *parameter) {
    for (;;) {
        Serial.println("Core 0 task");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup() {
    xTaskCreatePinnedToCore(task1, "Task1", 10000, NULL, 1, NULL, 0);
}

// Deep Sleep
esp_sleep_enable_timer_wakeup(10 * 1000000);  // 10秒
esp_deep_sleep_start();
```

### STM32

産業グレードの信頼性、高速ADC、豊富なペリフェラル。

```cpp
// STM32 固有機能

// DMA転送（ADC）
#include <STM32ADC.h>

// ハードウェアタイマー
HardwareTimer timer(TIM2);

void setup() {
    timer.setOverflow(1000, HERTZ_FORMAT);  // 1kHz
    timer.attachInterrupt(timerCallback);
    timer.resume();
}

void timerCallback() {
    // 1kHz で呼び出される
}

// CAN Bus（対応ボードのみ）
#include <STM32_CAN.h>
STM32_CAN Can(CAN1, DEF);
```

### RP2040

Raspberry Pi Pico。PIO（Programmable I/O）が特徴的。

```cpp
// RP2040 Arduino Core (Earle Philhower)

// マルチコア使用
void setup1() {
    // Core 1 の setup
}

void loop1() {
    // Core 1 の loop
}

// PIO (Programmable I/O)
#include "hardware/pio.h"

// PWM with hardware slices
#include "hardware/pwm.h"

void setup() {
    // RP2040 固有の低レベルアクセス
    uint slice = pwm_gpio_to_slice_num(LED_PIN);
    pwm_set_wrap(slice, 65535);
    pwm_set_enabled(slice, true);
}

// USB Host/Device
#include <Adafruit_TinyUSB.h>
```

### プラットフォーム間の違い

| 機能 | ESP32 | STM32 | RP2040 |
|------|-------|-------|--------|
| **ADC 解像度** | 12bit | 12bit | 12bit |
| **analogWrite** | ledcWrite | analogWrite | analogWrite |
| **タイマー数** | 4 | 多数 | 限定的 |
| **DMA** | あり | あり | あり |
| **Wi-Fi** | 内蔵 | 外部モジュール必要 | Pico W のみ |
| **Deep Sleep** | 優れている | 優れている | 限定的 |

---

## 7. Arduino Cloud

Arduino Cloud は IoT ダッシュボード、リモートモニタリング、OTA アップデートを提供するクラウドサービス。

### 2024-2025 の新機能

- **ダッシュボード複製**: ワンクリックで複製可能
- **Image ウィジェット**: カスタム画像の表示
- **Triggers**: 条件ベースのアクション
- **AI アシスタント**: コード生成支援
- **BLE経由のWi-Fi設定**: ハードコーディング不要

### プラン (2025年更新)

| プラン | 対象 | 主な機能 |
|--------|------|----------|
| **Free** | 個人 | 2デバイス、基本機能 |
| **Maker** | メイカー | 無制限コンパイル、OTA、AI支援 |
| **Team** | 企業 | RBAC、50ユーザー、100デバイス |
| **Enterprise** | 大規模 | カスタム対応 |

### Arduino Cloud 接続例

```cpp
#include "thingProperties.h"

// Arduino Cloud で自動生成されるプロパティ
// cloud変数は自動同期される

void setup() {
    Serial.begin(115200);
    delay(1500);

    // Arduino Cloud への接続を初期化
    initProperties();
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);

    setDebugMessageLevel(2);
    ArduinoCloud.printDebugInfo();
}

void loop() {
    ArduinoCloud.update();

    // センサー読み取りと cloud 変数の更新
    temperature = readTemperature();
    humidity = readHumidity();
}

// cloud 変数が変更された時のコールバック
void onLedStatusChange() {
    digitalWrite(LED_PIN, ledStatus ? HIGH : LOW);
}
```

### サポートされるボード

- Arduino Nano 33 IoT / Nano RP2040 Connect
- Arduino MKR WiFi 1010 / MKR WAN 1310
- Arduino Portenta H7
- ESP32 / ESP8266
- Raspberry Pi Pico W

---

## 8. Best Practices

### メモリ最適化

```cpp
// 1. F() マクロでフラッシュにテキストを保存
Serial.println(F("This string is stored in flash"));

// 2. PROGMEM でconst配列をフラッシュに
const char longText[] PROGMEM = "Very long text...";

// 3. 適切なデータ型を選択
uint8_t smallValue;   // 0-255 なら uint8_t
int16_t mediumValue;  // int の代わりに

// 4. String クラスを避ける（フラグメンテーション対策）
// 悪い例
String message = "Value: " + String(value);

// 良い例
char buffer[32];
snprintf(buffer, sizeof(buffer), "Value: %d", value);

// 5. メモリ使用量の確認
void printMemory() {
    #ifdef ESP32
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    #elif defined(__AVR__)
    extern int __heap_start, *__brkval;
    int v;
    int freeMemory = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    Serial.print("Free RAM: ");
    Serial.println(freeMemory);
    #endif
}
```

### delay() を避ける（ノンブロッキング設計）

```cpp
// 悪い例: ブロッキング
void loop() {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
}

// 良い例: ノンブロッキング（millis使用）
unsigned long previousMillis = 0;
const unsigned long interval = 1000;
bool ledState = false;

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED, ledState);
    }

    // 他の処理も実行可能
    handleSerial();
    updateSensors();
}

// タスクスケジューラライブラリの使用も検討
#include <TaskScheduler.h>
Scheduler runner;

void blinkCallback();
Task blinkTask(1000, TASK_FOREVER, &blinkCallback);

void setup() {
    runner.addTask(blinkTask);
    blinkTask.enable();
}

void loop() {
    runner.execute();
}
```

### ISR（割り込みサービスルーチン）

```cpp
// 割り込み変数は volatile 宣言
volatile bool buttonPressed = false;
volatile unsigned long lastInterruptTime = 0;

// ISRは短く、最小限の処理に
void IRAM_ATTR buttonISR() {  // IRAM_ATTR は ESP32 用
    unsigned long interruptTime = millis();

    // デバウンス処理
    if (interruptTime - lastInterruptTime > 200) {
        buttonPressed = true;
    }
    lastInterruptTime = interruptTime;
}

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
}

void loop() {
    if (buttonPressed) {
        buttonPressed = false;
        // 重い処理はここで行う
        handleButtonPress();
    }
}

// ISRでやってはいけないこと:
// - delay() の使用
// - Serial.print() の使用
// - 長い処理
// - 他の割り込みを呼ぶ処理
```

### ウォッチドッグタイマー

```cpp
// ESP32
#include <esp_task_wdt.h>

void setup() {
    // 30秒のタイムアウトでWDTを初期化
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);  // 現在のタスクを監視
}

void loop() {
    // 正常動作時は定期的にリセット
    esp_task_wdt_reset();

    // 重い処理...
}

// AVR
#include <avr/wdt.h>

void setup() {
    wdt_enable(WDTO_8S);  // 8秒タイムアウト
}

void loop() {
    wdt_reset();  // ウォッチドッグリセット
}
```

### エラーハンドリング

```cpp
enum class SensorStatus {
    OK,
    ERROR_INIT,
    ERROR_TIMEOUT,
    ERROR_CRC
};

SensorStatus initSensor() {
    Wire.beginTransmission(SENSOR_ADDR);
    if (Wire.endTransmission() != 0) {
        return SensorStatus::ERROR_INIT;
    }

    // 初期化処理...
    return SensorStatus::OK;
}

void setup() {
    Serial.begin(115200);

    auto status = initSensor();
    if (status != SensorStatus::OK) {
        Serial.println(F("Sensor initialization failed!"));
        // エラー時の処理（LED点滅、再試行、etc.）
        while (true) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }
}
```

---

## References

### 公式ドキュメント

- [Arduino IDE 2.x](https://github.com/arduino/arduino-ide)
- [Arduino CLI Documentation](https://docs.arduino.cc/arduino-cli/)
- [Arduino Cloud](https://cloud.arduino.cc/)
- [PlatformIO Documentation](https://docs.platformio.org/)

### サードパーティボードコア

- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [STM32duino](https://github.com/stm32duino)
- [Arduino-Pico (RP2040)](https://github.com/earlephilhower/arduino-pico)

### コミュニティリソース

- [Arduino Forum](https://forum.arduino.cc/)
- [PlatformIO Community](https://community.platformio.org/)
- [DroneBot Workshop Tutorials](https://dronebotworkshop.com/)
