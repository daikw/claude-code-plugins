---
name: esp32
description: Use when developing firmware for ESP32 devices. Covers ESP-IDF, WiFi/BLE, FreeRTOS tasks, and peripheral drivers.
tags:
  - esp32
  - esp-idf
  - iot
  - wifi
  - ble
---

# ESP32 Development Guide

ESP32 シリーズでのファームウェア開発をサポート。ESP-IDF、WiFi/BLE通信、FreeRTOS、ペリフェラル制御のガイド。

## When to Activate

- ESP32/ESP32-S3/ESP32-C3/ESP32-C6 で開発するとき
- ESP-IDF プロジェクトをセットアップするとき
- WiFi/BLE 通信を実装するとき
- FreeRTOS タスクを作成・管理するとき
- GPIO、I2C、SPI、UART などペリフェラルを制御するとき
- OTA アップデートを実装するとき
- 省電力モード（Deep Sleep/Light Sleep）を設定するとき

## Device Overview

### ESP32 シリーズ比較 (2025年時点)

| モデル | CPU | クロック | RAM | Flash | WiFi | Bluetooth | GPIO | 特徴 |
|--------|-----|---------|-----|-------|------|-----------|------|------|
| ESP32 | Dual Xtensa LX6 | 240MHz | 520KB | 4-16MB | 802.11 b/g/n | BT 4.2 + BLE | 34 | 定番、高性能 |
| ESP32-S2 | Single Xtensa LX7 | 240MHz | 320KB | 4-16MB | 802.11 b/g/n | なし | 43 | USB OTG、低コスト |
| ESP32-S3 | Dual Xtensa LX7 | 240MHz | 512KB | 8-16MB | 802.11 b/g/n | BLE 5.0 | 45 | AI/ML、カメラ、LCD |
| ESP32-C3 | Single RISC-V | 160MHz | 400KB | 4MB | 802.11 b/g/n | BLE 5.0 | 22 | 低コスト、省電力 |
| ESP32-C6 | RISC-V + LP core | 160MHz | 512KB | 4-8MB | WiFi 6 (802.11ax) | BLE 5.3 | 30 | WiFi 6、Thread/Zigbee、Matter |
| ESP32-H2 | Single RISC-V | 96MHz | 320KB | 4MB | なし | BLE 5.3 | 19 | Thread/Zigbee専用、Matter |

### 選び方の目安

| ユースケース | 推奨チップ | 理由 |
|-------------|-----------|------|
| AI/ML エッジ処理 | ESP32-S3 | デュアルコア、ベクトル命令、PSRAM対応 |
| カメラ/ディスプレイ | ESP32-S3 | OPI PSRAM、DVP/LCD インターフェース |
| 低コスト IoT センサー | ESP32-C3 | 安価、BLE対応、省電力 |
| スマートホーム (Matter) | ESP32-C6 | WiFi 6、Thread、Zigbee対応 |
| Thread/Zigbee メッシュ | ESP32-C6/H2 | 802.15.4 対応 |
| 密集WiFi環境 | ESP32-C6 | WiFi 6 OFDMA、TWT省電力 |
| バッテリー駆動 | ESP32-C3/C6 | 低消費電力、Deep Sleep 40μA以下 |
| レガシープロジェクト | ESP32 | 豊富なライブラリ、情報量 |

### ESP32-C6 の特徴

ESP32-C6 は WiFi 6 (802.11ax)、BLE 5.3、802.15.4 (Thread/Zigbee) を統合した次世代チップ。

**WiFi 6 の利点**:
- **OFDMA**: 複数デバイスの同時通信、レイテンシ削減
- **TWT (Target Wake Time)**: デバイスのスリープスケジュール最適化、バッテリー寿命延長
- **MU-MIMO**: 下りマルチユーザー対応

**デュアルコア構成**:
- HP Core: 160MHz RISC-V（メイン処理）
- LP Core: 20MHz RISC-V（バックグラウンド/スリープ中処理）

## ESP-IDF Setup

### インストール (2025年推奨: v5.5.x)

```bash
# 前提条件
# - Git
# - Python 3.8+
# - CMake
# - Ninja

# ESP-IDF v5.5.2 クローン
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# セットアップスクリプト実行
cd ~/esp/esp-idf
./install.sh esp32,esp32s3,esp32c3,esp32c6  # 必要なターゲットを指定

# 環境変数設定（シェル起動時に毎回実行）
. ~/esp/esp-idf/export.sh

# または .bashrc/.zshrc に追加
echo 'alias get_idf=". ~/esp/esp-idf/export.sh"' >> ~/.bashrc
```

### VS Code 統合

```bash
# VS Code Espressif IDF Extension をインストール
# 1. VS Code で Extensions (Ctrl+Shift+X)
# 2. "Espressif IDF" を検索してインストール
# 3. Cmd+Shift+P → "ESP-IDF: Configure ESP-IDF Extension"
```

### バージョン確認

```bash
idf.py --version
# ESP-IDF v5.5.2

# 各コンポーネントバージョン
python -m pip show esptool
```

### プロジェクト作成

```bash
# サンプルプロジェクトからコピー
cp -r $IDF_PATH/examples/get-started/hello_world ~/my_project
cd ~/my_project

# または新規作成
idf.py create-project my_project
cd my_project
```

### idf.py コマンド一覧

```bash
# ターゲット設定（最初に1回）
idf.py set-target esp32      # ESP32
idf.py set-target esp32s3    # ESP32-S3
idf.py set-target esp32c3    # ESP32-C3
idf.py set-target esp32c6    # ESP32-C6

# ビルド
idf.py build

# フラッシュ（書き込み）
idf.py -p /dev/ttyUSB0 flash

# シリアルモニター
idf.py -p /dev/ttyUSB0 monitor
# 終了: Ctrl+]

# ビルド＋フラッシュ＋モニター
idf.py -p /dev/ttyUSB0 flash monitor

# コンフィグメニュー（menuconfig）
idf.py menuconfig

# クリーンビルド
idf.py fullclean

# プロジェクトサイズ分析
idf.py size
idf.py size-components
idf.py size-files

# パーティションテーブル確認
idf.py partition-table

# eFuse（一度だけ書き込める設定）確認
espefuse.py --port /dev/ttyUSB0 summary
```

### CMakeLists.txt の基本構造

```cmake
# プロジェクトルートの CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_project)
```

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "wifi.c" "sensors.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_event
)
```

## WiFi/BLE

### WiFi Station モード（接続側）

```c
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "wifi_station";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init_sta(const char *ssid, const char *password)
{
    // NVS 初期化（WiFi設定保存用）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // ネットワークスタック初期化
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // WiFi 初期化
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // イベントハンドラ登録
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);

    // WiFi 設定
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}
```

### WiFi AP モード（アクセスポイント）

```c
void wifi_init_softap(const char *ssid, const char *password)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "",
            .ssid_len = 0,
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strncpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = strlen(ssid);

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
}
```

### WiFi スキャン

```c
void wifi_scan(void)
{
    uint16_t number = 10;
    wifi_ap_record_t ap_info[10];
    uint16_t ap_count = 0;

    esp_wifi_scan_start(NULL, true);  // ブロッキングスキャン
    esp_wifi_scan_get_ap_records(&number, ap_info);
    esp_wifi_scan_get_ap_num(&ap_count);

    ESP_LOGI(TAG, "Found %d APs:", ap_count);
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(TAG, "  %s (RSSI: %d, Channel: %d)",
                 ap_info[i].ssid, ap_info[i].rssi, ap_info[i].primary);
    }
}
```

### BLE GATT Server

```c
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"

#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4

static uint8_t adv_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void ble_init(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    // GATTS/GAP コールバック登録
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);

    // アドバタイジング開始
    esp_ble_gap_start_advertising(&adv_params);
}
```

### BLE 5.x 機能 (ESP32-C3/C6/S3)

```c
// BLE 5.0 Extended Advertising
esp_ble_gap_ext_adv_params_t ext_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .interval_min = 0x30,
    .interval_max = 0x60,
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,  // 2M PHY で高速通信
    .sid = 0,
    .scan_req_notif = false,
};

// Coded PHY（長距離通信）
esp_ble_gap_ext_adv_params_t long_range_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .primary_phy = ESP_BLE_GAP_PHY_CODED,
    .secondary_phy = ESP_BLE_GAP_PHY_CODED,
    // ...
};
```

## Peripherals

### GPIO 制御

```c
#include "driver/gpio.h"

// 出力設定
gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << GPIO_NUM_2),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};
gpio_config(&io_conf);

// 出力操作
gpio_set_level(GPIO_NUM_2, 1);  // HIGH
gpio_set_level(GPIO_NUM_2, 0);  // LOW

// 入力設定（プルアップ付き）
io_conf.pin_bit_mask = (1ULL << GPIO_NUM_4);
io_conf.mode = GPIO_MODE_INPUT;
io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
gpio_config(&io_conf);

// 入力読み取り
int level = gpio_get_level(GPIO_NUM_4);
```

### GPIO 割り込み

```c
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    // ISR内では最小限の処理のみ（フラグ設定、キュー送信など）
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void gpio_interrupt_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // 立ち下がりエッジ
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0, gpio_isr_handler, (void *)GPIO_NUM_0);
}
```

### ADC (アナログ入力)

```c
#include "esp_adc/adc_oneshot.h"

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
};
adc_oneshot_new_unit(&init_config, &adc1_handle);

adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_12,
    .atten = ADC_ATTEN_DB_12,  // 0-3.3V 範囲
};
adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config);

// 読み取り
int raw_value;
adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw_value);
// raw_value: 0-4095 (12bit)

// 電圧変換 (ADC_ATTEN_DB_12 の場合、約 0-3.3V)
float voltage = raw_value * 3.3 / 4095;
```

### I2C Master

```c
#include "driver/i2c_master.h"

// I2C マスター初期化 (ESP-IDF v5.x 新API)
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_22,
    .sda_io_num = GPIO_NUM_21,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_config, &bus_handle);

// デバイス追加
i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68,  // MPU6050 等
    .scl_speed_hz = 400000,  // 400kHz
};
i2c_master_dev_handle_t dev_handle;
i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);

// 読み書き
uint8_t reg = 0x75;  // WHO_AM_I register
uint8_t data;
i2c_master_transmit_receive(dev_handle, &reg, 1, &data, 1, -1);
```

### SPI Master

```c
#include "driver/spi_master.h"

// SPI バス初期化
spi_bus_config_t bus_cfg = {
    .mosi_io_num = GPIO_NUM_23,
    .miso_io_num = GPIO_NUM_19,
    .sclk_io_num = GPIO_NUM_18,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
};
spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

// デバイス追加
spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = 10 * 1000 * 1000,  // 10MHz
    .mode = 0,
    .spics_io_num = GPIO_NUM_5,
    .queue_size = 7,
};
spi_device_handle_t spi_handle;
spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle);

// トランザクション
spi_transaction_t trans = {
    .length = 8,
    .tx_buffer = &tx_data,
    .rx_buffer = &rx_data,
};
spi_device_transmit(spi_handle, &trans);
```

### UART

```c
#include "driver/uart.h"

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};
uart_param_config(UART_NUM_1, &uart_config);
uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16, -1, -1);
uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);

// 送信
char *data = "Hello UART!";
uart_write_bytes(UART_NUM_1, data, strlen(data));

// 受信
uint8_t buf[128];
int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(100));
```

### PWM (LEDC)

```c
#include "driver/ledc.h"

// タイマー設定
ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 5000,  // 5kHz
    .clk_cfg = LEDC_AUTO_CLK,
};
ledc_timer_config(&timer_conf);

// チャンネル設定
ledc_channel_config_t channel_conf = {
    .gpio_num = GPIO_NUM_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 4096,  // 50% (13bit: 0-8191)
    .hpoint = 0,
};
ledc_channel_config(&channel_conf);

// デューティ比変更
ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 2048);  // 25%
ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

// フェード機能
ledc_fade_func_install(0);
ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191, 1000);
ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
```

## FreeRTOS Tasks

### タスク作成

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void sensor_task(void *pvParameters)
{
    while (1) {
        // センサー読み取り処理
        ESP_LOGI(TAG, "Reading sensor...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒待機
    }
}

void app_main(void)
{
    // タスク作成
    xTaskCreate(
        sensor_task,       // タスク関数
        "sensor_task",     // タスク名
        4096,              // スタックサイズ (バイト)
        NULL,              // パラメータ
        5,                 // 優先度 (0が最低、configMAX_PRIORITIES-1が最高)
        NULL               // タスクハンドル
    );

    // 特定のCPUコアに固定（ESP32デュアルコア）
    xTaskCreatePinnedToCore(
        wifi_task, "wifi", 4096, NULL, 5, NULL,
        0  // Core 0
    );
    xTaskCreatePinnedToCore(
        ble_task, "ble", 4096, NULL, 5, NULL,
        1  // Core 1
    );
}
```

### キュー（タスク間通信）

```c
#include "freertos/queue.h"

typedef struct {
    int sensor_id;
    float value;
} sensor_data_t;

QueueHandle_t sensor_queue;

void producer_task(void *pvParameters)
{
    sensor_data_t data = {.sensor_id = 1, .value = 25.5};
    while (1) {
        xQueueSend(sensor_queue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void consumer_task(void *pvParameters)
{
    sensor_data_t data;
    while (1) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Sensor %d: %.2f", data.sensor_id, data.value);
        }
    }
}

void app_main(void)
{
    sensor_queue = xQueueCreate(10, sizeof(sensor_data_t));
    xTaskCreate(producer_task, "producer", 2048, NULL, 5, NULL);
    xTaskCreate(consumer_task, "consumer", 2048, NULL, 5, NULL);
}
```

### セマフォ（排他制御）

```c
#include "freertos/semphr.h"

SemaphoreHandle_t i2c_mutex;

void task_using_i2c(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // I2C 操作（排他制御下）
            // ...
            xSemaphoreGive(i2c_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    i2c_mutex = xSemaphoreCreateMutex();
    xTaskCreate(task_using_i2c, "task1", 2048, NULL, 5, NULL);
    xTaskCreate(task_using_i2c, "task2", 2048, NULL, 5, NULL);
}
```

### イベントグループ

```c
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1

EventGroupHandle_t event_group;

void wifi_task(void *pvParameters)
{
    // WiFi 接続処理...
    xEventGroupSetBits(event_group, WIFI_CONNECTED_BIT);
}

void mqtt_task(void *pvParameters)
{
    // WiFi 接続を待つ
    xEventGroupWaitBits(event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    // MQTT 接続処理...
    xEventGroupSetBits(event_group, MQTT_CONNECTED_BIT);
}

void app_task(void *pvParameters)
{
    // WiFi と MQTT の両方の接続を待つ
    EventBits_t bits = xEventGroupWaitBits(
        event_group,
        WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT,
        pdFALSE, pdTRUE, portMAX_DELAY
    );
    ESP_LOGI(TAG, "All connections ready!");
}
```

## Power Management

### Deep Sleep

```c
#include "esp_sleep.h"

// タイマーウェイクアップ
void deep_sleep_timer(uint64_t sleep_time_us)
{
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    ESP_LOGI(TAG, "Entering deep sleep for %llu us", sleep_time_us);
    esp_deep_sleep_start();
    // ここには戻らない（再起動）
}

// GPIO ウェイクアップ
void deep_sleep_gpio(gpio_num_t gpio_num)
{
    esp_sleep_enable_ext0_wakeup(gpio_num, 0);  // LOW でウェイク
    esp_deep_sleep_start();
}

// ウェイクアップ原因確認
void check_wakeup_cause(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wakeup by timer");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG, "Wakeup by external signal (ext0)");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ESP_LOGI(TAG, "Wakeup by external signal (ext1)");
            break;
        default:
            ESP_LOGI(TAG, "Not a deep sleep wakeup");
    }
}
```

### Light Sleep

```c
// Light Sleep（RAMデータ保持）
void light_sleep_timer(uint64_t sleep_time_us)
{
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    esp_light_sleep_start();
    // ここに戻る
    ESP_LOGI(TAG, "Woke up from light sleep");
}

// 自動Light Sleep（WiFi使用時）
void enable_auto_light_sleep(void)
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = true,
    };
    esp_pm_configure(&pm_config);
}
```

### ULP (Ultra Low Power) Coprocessor

```c
// ULP プログラム（アセンブリ）は別ファイルで定義
// main/ulp/pulse_cnt.S

#include "esp_sleep.h"
#include "ulp.h"

void ulp_example(void)
{
    // ULP プログラムロード
    ulp_load_binary(0, ulp_main_bin_start,
                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));

    // ULP ウェイクアップ有効化
    esp_sleep_enable_ulp_wakeup();

    // ULP 実行開始
    ulp_run(&ulp_entry - RTC_SLOW_MEM);

    // Deep Sleep
    esp_deep_sleep_start();
}
```

### 消費電力の目安

| モード | ESP32 | ESP32-C3 | ESP32-C6 | 備考 |
|--------|-------|----------|----------|------|
| Active (WiFi TX) | 240mA | 350mA | 320mA | ピーク |
| Active (CPU max) | 80mA | 40mA | 35mA | |
| Modem Sleep | 20mA | 10mA | 10mA | WiFi維持 |
| Light Sleep | 0.8mA | 130μA | 100μA | RAM保持 |
| Deep Sleep | 10μA | 5μA | 7μA | RTC保持 |
| Hibernation | - | - | 2.5μA | 最小消費 |

## OTA Updates

### 基本OTA（HTTPSから）

```c
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

void ota_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting OTA...");

    esp_http_client_config_t config = {
        .url = "https://example.com/firmware.bin",
        .cert_pem = (char *)server_cert_pem_start,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA Failed: %s", esp_err_to_name(ret));
    }
}
```

### 進捗表示付きOTA

```c
esp_err_t ota_with_progress(const char *url)
{
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 5000,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    esp_err_t err = esp_https_ota_begin(&ota_config, &ota_handle);
    if (err != ESP_OK) {
        return err;
    }

    int total = esp_https_ota_get_image_size(ota_handle);
    int written = 0;

    while (1) {
        err = esp_https_ota_perform(ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        written = esp_https_ota_get_image_len_read(ota_handle);
        ESP_LOGI(TAG, "Progress: %d/%d (%d%%)", written, total, written * 100 / total);
    }

    if (err == ESP_OK && esp_https_ota_is_complete_data_received(ota_handle)) {
        err = esp_https_ota_finish(ota_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "OTA Complete!");
            return ESP_OK;
        }
    }

    esp_https_ota_abort(ota_handle);
    return err;
}
```

### パーティションテーブル（OTA用）

```csv
# partitions.csv (OTA 対応)
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x4000,
otadata,  data, ota,     0xd000,  0x2000,
phy_init, data, phy,     0xf000,  0x1000,
ota_0,    app,  ota_0,   0x10000, 0x1E0000,
ota_1,    app,  ota_1,   0x1F0000,0x1E0000,
```

```bash
# menuconfig でパーティションテーブル選択
idf.py menuconfig
# → Partition Table → Custom partition table CSV
```

### ロールバック

```c
// ブート後に検証、失敗時はロールバック
void validate_and_confirm_ota(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // 検証処理...
            bool valid = run_diagnostics();

            if (valid) {
                esp_ota_mark_app_valid_cancel_rollback();
                ESP_LOGI(TAG, "OTA validated!");
            } else {
                ESP_LOGE(TAG, "OTA validation failed, rolling back...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
}
```

## Common Libraries

### esp-mqtt

```c
#include "mqtt_client.h"

esp_mqtt_client_handle_t mqtt_client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            esp_mqtt_client_subscribe(mqtt_client, "/topic/test", 0);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Received: %.*s = %.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            break;
    }
}

void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.example.com",
        .credentials.username = "user",
        .credentials.authentication.password = "pass",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_publish(const char *topic, const char *data)
{
    esp_mqtt_client_publish(mqtt_client, topic, data, 0, 1, 0);
}
```

### esp-http-client

```c
#include "esp_http_client.h"

// GET リクエスト
void http_get(const char *url)
{
    char buffer[1024];
    esp_http_client_config_t config = {
        .url = url,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int len = esp_http_client_read(client, buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';
        ESP_LOGI(TAG, "Response: %s", buffer);
    }
    esp_http_client_cleanup(client);
}

// POST リクエスト（JSON）
void http_post_json(const char *url, const char *json)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Status: %d", esp_http_client_get_status_code(client));
    }
    esp_http_client_cleanup(client);
}
```

### cJSON

```c
#include "cJSON.h"

// JSON 生成
char *create_json(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device", "ESP32");
    cJSON_AddNumberToObject(root, "temperature", 25.5);
    cJSON_AddBoolToObject(root, "active", true);

    cJSON *sensors = cJSON_AddArrayToObject(root, "sensors");
    cJSON_AddItemToArray(sensors, cJSON_CreateNumber(100));
    cJSON_AddItemToArray(sensors, cJSON_CreateNumber(200));

    char *json_str = cJSON_Print(root);
    cJSON_Delete(root);
    return json_str;
}

// JSON パース
void parse_json(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        ESP_LOGE(TAG, "JSON parse error");
        return;
    }

    cJSON *device = cJSON_GetObjectItem(root, "device");
    if (cJSON_IsString(device)) {
        ESP_LOGI(TAG, "Device: %s", device->valuestring);
    }

    cJSON *temp = cJSON_GetObjectItem(root, "temperature");
    if (cJSON_IsNumber(temp)) {
        ESP_LOGI(TAG, "Temperature: %.1f", temp->valuedouble);
    }

    cJSON_Delete(root);
}
```

### NVS (Non-Volatile Storage)

```c
#include "nvs_flash.h"
#include "nvs.h"

// NVS 初期化
void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

// 値の保存
void nvs_save(const char *key, int32_t value)
{
    nvs_handle_t handle;
    nvs_open("storage", NVS_READWRITE, &handle);
    nvs_set_i32(handle, key, value);
    nvs_commit(handle);
    nvs_close(handle);
}

// 値の読み取り
int32_t nvs_load(const char *key, int32_t default_value)
{
    nvs_handle_t handle;
    int32_t value = default_value;
    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_i32(handle, key, &value);
        nvs_close(handle);
    }
    return value;
}

// 文字列の保存・読み取り
void nvs_save_string(const char *key, const char *value)
{
    nvs_handle_t handle;
    nvs_open("storage", NVS_READWRITE, &handle);
    nvs_set_str(handle, key, value);
    nvs_commit(handle);
    nvs_close(handle);
}
```

### ESP-NOW（チップ間通信）

```c
#include "esp_now.h"

uint8_t peer_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // ブロードキャスト

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "Send status: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "Fail");
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "Received %d bytes from " MACSTR, len, MAC2STR(info->src_addr));
}

void espnow_init(void)
{
    esp_now_init();
    esp_now_register_send_cb(espnow_send_cb);
    esp_now_register_recv_cb(espnow_recv_cb);

    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, peer_mac, 6);
    esp_now_add_peer(&peer);
}

void espnow_send(const uint8_t *data, size_t len)
{
    esp_now_send(peer_mac, data, len);
}
```

## Troubleshooting

### ビルドエラー

```bash
# クリーンビルド
idf.py fullclean
idf.py build

# コンポーネント依存関係エラー
# CMakeLists.txt の REQUIRES を確認
idf_component_register(
    SRCS "main.c"
    REQUIRES esp_wifi nvs_flash  # 必要なコンポーネント
)
```

### フラッシュエラー

```bash
# 権限エラー (Linux)
sudo usermod -aG dialout $USER
# 再ログイン必要

# ポート確認
ls /dev/ttyUSB* /dev/ttyACM*

# ブートモードに入れない
# → BOOT/GPIO0 ボタンを押しながらENボタンを押す

# フラッシュ速度を下げる
idf.py -p /dev/ttyUSB0 -b 115200 flash
```

### WiFi 接続失敗

```c
// デバッグログ有効化
esp_log_level_set("wifi", ESP_LOG_VERBOSE);
esp_log_level_set("wifi_init", ESP_LOG_VERBOSE);

// 一般的な原因
// 1. SSID/パスワードの誤り
// 2. 2.4GHz帯以外のAP（ESP32は2.4GHzのみ）
// 3. WPA3専用AP（WPA2互換モードに設定）
// 4. MACアドレスフィルタリング
```

### メモリ不足

```c
// ヒープ使用量確認
ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
ESP_LOGI(TAG, "Min free heap: %lu", esp_get_minimum_free_heap_size());

// 対策
// 1. スタックサイズを適切に（過剰に大きくしない）
// 2. 静的バッファを使用
// 3. menuconfig でログレベルを下げる
// 4. 不要なコンポーネントを無効化
```

### スタックオーバーフロー

```c
// タスクのスタック使用量確認
UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
ESP_LOGI(TAG, "Stack remaining: %u bytes", stack_remaining * sizeof(StackType_t));

// スタックサイズを増やす
xTaskCreate(task_func, "task", 8192, NULL, 5, NULL);  // 4096 → 8192
```

## Command Reference

| コマンド | 説明 |
|---------|------|
| `idf.py set-target <chip>` | ターゲットチップ設定 |
| `idf.py build` | ビルド |
| `idf.py flash` | フラッシュ書き込み |
| `idf.py monitor` | シリアルモニター (Ctrl+] で終了) |
| `idf.py flash monitor` | フラッシュ＋モニター |
| `idf.py menuconfig` | コンフィグメニュー |
| `idf.py fullclean` | クリーンビルド |
| `idf.py size` | バイナリサイズ分析 |
| `idf.py size-components` | コンポーネント別サイズ |
| `idf.py partition-table` | パーティションテーブル表示 |
| `idf.py erase-flash` | フラッシュ全消去 |
| `espefuse.py summary` | eFuse 情報表示 |

## Resources

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/stable/)
- [ESP-IDF GitHub](https://github.com/espressif/esp-idf)
- [ESP32 Datasheet](https://www.espressif.com/en/products/socs/esp32)
- [ESP32-C6 Product Page](https://www.espressif.com/en/products/socs/esp32-c6)
- [ESP32 Forum](https://esp32.com/)
- [Espressif Developer Portal](https://developer.espressif.com/)
- [ESP-IDF Examples](https://github.com/espressif/esp-idf/tree/master/examples)
