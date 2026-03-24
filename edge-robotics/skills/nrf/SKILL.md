---
name: nrf
description: Use when developing firmware for Nordic nRF devices. Covers nRF Connect SDK, Zephyr, BLE, and low-power design.
tags:
  - nrf
  - nordic
  - ble
  - zephyr
  - low-power
---

# Nordic nRF Development Skill

Nordic Semiconductor nRF シリーズでのファームウェア開発をサポートするスキル。nRF Connect SDK、Zephyr RTOS、BLE、低消費電力設計をカバーする。

## When to Activate

- nRF52/nRF53/nRF54 シリーズで開発するとき
- nRF Connect SDK（NCS）をセットアップするとき
- BLE アドバタイジング、GATT、Mesh を実装するとき
- Long Range（Coded PHY）通信を設定するとき
- Thread/Matter 対応デバイスを開発するとき
- 低消費電力設計（System OFF/ON モード）を行うとき
- DFU/OTA アップデートを実装するとき
- nRF Cloud / Edge Impulse と連携するとき

---

## 1. Device Overview - nRF52 vs nRF53 vs nRF54 Series

### nRF シリーズ比較 (2025年時点)

| モデル | CPU | クロック | RAM | Flash | Radio | TX Power | RX感度 | 特徴 |
|--------|-----|---------|-----|-------|-------|----------|--------|------|
| **nRF52840** | Cortex-M4F | 64MHz | 256KB | 1MB | BLE 5.0, 802.15.4 | +8dBm | -95dBm | 定番、マルチプロトコル |
| **nRF52833** | Cortex-M4F | 64MHz | 128KB | 512KB | BLE 5.1 | +8dBm | -95dBm | 方向検知、コスト優先 |
| **nRF5340** | Dual Cortex-M33 | 128/64MHz | 512+64KB | 1MB+256KB | BLE 5.4 | +3dBm | -98dBm | デュアルコア、セキュア |
| **nRF54L15** | Cortex-M33 | 128MHz | 256KB | 1.5MB | BLE 5.4, 802.15.4 | +8dBm | -98dBm | 超低消費電力、22nm |
| **nRF54H20** | Dual Cortex-M33 + RISC-V | Higher | Large | Large | BLE 5.4, 802.15.4 | +10dBm | -100dBm | 最高性能、ML対応 |

### 選び方ガイド

| ユースケース | 推奨デバイス | 理由 |
|-------------|-------------|------|
| 高速BLE + Thread/Zigbee | nRF52840 | 豊富な事例、安定性 |
| 低コストBLE製品 | nRF52833 | バランス良好 |
| セキュリティ重視 | nRF5340 | TrustZone、デュアルコア |
| 超低消費電力 | nRF54L15 | 22nmプロセス、40%省電力 |
| 高性能ML/エッジ | nRF54H20 | RISC-V、高処理能力 |
| Matter/スマートホーム | nRF52840/nRF5340 | Thread対応、認証済み |
| 長距離通信 | nRF52840 + Coded PHY | Long Range 4x |

### nRF52840 の特徴

nRF52シリーズの最上位機種。最も広く使われている。

**Key Features:**
- **CPU**: ARM Cortex-M4F @ 64MHz (FPU内蔵)
- **Memory**: 1MB Flash + 256KB RAM
- **Radio**: 2.4GHz (BLE 5.0, 802.15.4, ANT, 独自プロトコル)
- **Peripherals**: USB 2.0, NFC-A, QSPI, 48 GPIO
- **Security**: ARM TrustZone (Cortex-M33版), CryptoCell

### nRF5340 デュアルコアアーキテクチャ

```
┌─────────────────────────────────────────────────────────┐
│                      nRF5340                             │
│  ┌─────────────────────┐   ┌─────────────────────────┐  │
│  │   Application Core   │   │    Network Core         │  │
│  │   Cortex-M33 @128MHz │   │    Cortex-M33 @64MHz    │  │
│  │   1MB Flash, 512KB RAM│   │    256KB Flash, 64KB RAM│  │
│  │   TrustZone対応      │   │    BLE/802.15.4 Stack   │  │
│  │   (ユーザーアプリ)   │   │    (プロトコル処理)     │  │
│  └──────────┬──────────┘   └──────────┬──────────────┘  │
│             │         IPC             │                  │
│             └────────────────────────┘                  │
│                       │                                  │
│              ┌───────┴────────┐                         │
│              │   Peripherals  │                         │
│              │   GPIO, I2C,   │                         │
│              │   SPI, PWM...  │                         │
│              └────────────────┘                         │
└─────────────────────────────────────────────────────────┘
```

### nRF54L15 の革新

nRF54Lシリーズは第4世代BLE SoC。nRF52シリーズの後継として位置づけられる。

**Key Improvements:**
- **22nm FDSOI Process**: 業界初、大幅な省電力化
- **40% TX Current Reduction**: 8dBm時でも従来比40%削減
- **50% RX Current Reduction**: 受信電流ほぼ半減
- **128MHz CPU**: nRF52比で2倍の処理能力
- **14-bit ADC**: 12-bit → 14-bit で精度向上
- **Bluetooth 6.0 Channel Sounding**: 高精度測距対応

---

## 2. nRF Connect SDK Setup

### Overview

nRF Connect SDK（NCS）はNordic公式SDK。Zephyr RTOSをベースに構築されている。

**Current Version (2024-2025):**
- Latest: **v2.8.0** (November 2024)
- Key features: nRF54L Series support, Thread 1.4, BLE connection subrating

**Supported Protocols:**
- Bluetooth LE, Bluetooth Mesh
- Thread, Zigbee, Matter
- LTE-M, NB-IoT (nRF91 Series)
- ANT, 2.4GHz Proprietary

### Installation

#### Prerequisites

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-wheel xz-utils file

# macOS
brew install cmake ninja gperf python3 ccache qemu dtc wget

# Windows: nRF Command Line Tools インストーラーを使用
```

#### nRF Connect SDK のインストール

```bash
# 1. west インストール
pip3 install west

# 2. NCS をクローン (v2.8.0)
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.8.0 ~/ncs
cd ~/ncs
west update

# 3. Python 依存関係インストール
pip3 install -r zephyr/scripts/requirements.txt
pip3 install -r nrf/scripts/requirements.txt

# 4. ツールチェーンインストール
# nRF Command Line Tools (nrfjprog, nRF Util)
# https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools

# 5. 環境変数設定 (~/.bashrc or ~/.zshrc)
export ZEPHYR_BASE=~/ncs/zephyr
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=/path/to/gcc-arm-none-eabi
```

### nRF Connect for VS Code

VS Code拡張機能が推奨される開発環境。

```bash
# VS Code 拡張機能インストール
# 1. "nRF Connect for VS Code Extension Pack" を検索してインストール
# 2. Extension が自動的に SDK/Toolchain を検出

# 含まれる拡張機能:
# - nRF Connect
# - nRF DeviceTree
# - nRF Kconfig
# - nRF Terminal
```

### プロジェクト構造

```
my_app/
├── CMakeLists.txt
├── prj.conf                    # Kconfig 設定
├── Kconfig                     # カスタム Kconfig (オプション)
├── src/
│   └── main.c
├── boards/
│   └── nrf52840dk_nrf52840.overlay  # Board overlay
└── dts/
    └── bindings/               # カスタム devicetree bindings
```

### west コマンド一覧

```bash
# ビルド
west build -b nrf52840dk/nrf52840 samples/basic/blinky
west build -b nrf5340dk/nrf5340/cpuapp samples/basic/blinky
west build -b nrf54l15dk/nrf54l15/cpuapp samples/basic/blinky

# フラッシュ
west flash

# デバッグ
west debug

# RTT Viewer (ログ確認)
west attach

# ボード一覧
west boards | grep nrf

# クリーンビルド
west build -p always -b nrf52840dk/nrf52840 .
```

### CMakeLists.txt 例

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(my_app)

target_sources(app PRIVATE
    src/main.c
    src/ble_service.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

---

## 3. BLE - Advertising, GATT, Mesh, Long Range

### BLE 基本設定 (prj.conf)

```kconfig
# BLE 基本
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_DEVICE_NAME="nRF BLE Device"

# 接続設定
CONFIG_BT_MAX_CONN=1
CONFIG_BT_MAX_PAIRED=2

# GATT
CONFIG_BT_GATT_DYNAMIC_DB=y

# Security
CONFIG_BT_SMP=y
CONFIG_BT_SETTINGS=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_NVS=y
CONFIG_SETTINGS=y
```

### BLE Advertising

```c
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Advertising Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Scan Response Data */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err 0x%02x)\n", err);
    } else {
        printk("Connected\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int main(void)
{
    int err;

    printk("BLE Peripheral starting\n");

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    printk("Bluetooth initialized\n");

    /* Start Advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return err;
    }

    printk("Advertising successfully started\n");

    return 0;
}
```

### GATT Service 実装

```c
#include <zephyr/bluetooth/gatt.h>

/* カスタムサービス UUID */
#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)

#define BT_UUID_CUSTOM_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_CUSTOM_CHAR BT_UUID_DECLARE_128(BT_UUID_CUSTOM_CHAR_VAL)

static uint8_t custom_value[20] = {0};
static struct bt_gatt_ccc_cfg ccc_cfg[BT_GATT_CCC_MAX] = {};
static bool notify_enabled;

/* Read Callback */
static ssize_t read_custom_char(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             custom_value, sizeof(custom_value));
}

/* Write Callback */
static ssize_t write_custom_char(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags)
{
    if (offset + len > sizeof(custom_value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(custom_value + offset, buf, len);
    printk("Received %d bytes\n", len);

    return len;
}

/* CCC Changed Callback */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

/* GATT Service Definition */
BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_custom_char, write_custom_char, custom_value),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Notification送信 */
int send_notification(const uint8_t *data, uint16_t len)
{
    if (!notify_enabled) {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &custom_svc.attrs[1], data, len);
}
```

### BLE Long Range (Coded PHY)

Bluetooth 5の Long Range機能。通信距離を最大4倍に拡張。

```kconfig
# prj.conf - Long Range 設定
CONFIG_BT=y
CONFIG_BT_CTLR=y
CONFIG_BT_CTLR_PHY_CODED=y
CONFIG_BT_USER_PHY_UPDATE=y

# Extended Advertising (Long Range に必須)
CONFIG_BT_EXT_ADV=y
CONFIG_BT_EXT_ADV_MAX_ADV_SET=2
```

```c
#include <zephyr/bluetooth/bluetooth.h>

/* Extended Advertising パラメータ (Coded PHY) */
static struct bt_le_adv_param adv_param_coded = {
    .id = BT_ID_DEFAULT,
    .sid = 0,
    .secondary_max_skip = 0,
    .options = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED,
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
    .peer = NULL,
};

static const struct bt_data ad_coded[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

int start_coded_phy_advertising(void)
{
    struct bt_le_ext_adv *adv;
    int err;

    /* Extended Advertising Set 作成 */
    err = bt_le_ext_adv_create(&adv_param_coded, NULL, &adv);
    if (err) {
        printk("Failed to create advertising set (err %d)\n", err);
        return err;
    }

    /* Advertising Data 設定 */
    err = bt_le_ext_adv_set_data(adv, ad_coded, ARRAY_SIZE(ad_coded), NULL, 0);
    if (err) {
        printk("Failed to set advertising data (err %d)\n", err);
        return err;
    }

    /* Advertising 開始 */
    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        printk("Failed to start advertising (err %d)\n", err);
        return err;
    }

    printk("Coded PHY advertising started\n");
    return 0;
}

/* 接続後のPHY更新 */
void update_phy_to_coded(struct bt_conn *conn)
{
    struct bt_conn_le_phy_param phy_param = {
        .options = BT_CONN_LE_PHY_OPT_CODED_S8,  /* S=8 (Long Range最大) */
        .pref_tx_phy = BT_GAP_LE_PHY_CODED,
        .pref_rx_phy = BT_GAP_LE_PHY_CODED,
    };

    int err = bt_conn_le_phy_update(conn, &phy_param);
    if (err) {
        printk("PHY update failed (err %d)\n", err);
    }
}
```

### Long Range 性能比較

| PHY | Data Rate | Range (理論) | 用途 |
|-----|-----------|-------------|------|
| 1M PHY | 1 Mbps | 基準 | 標準通信 |
| 2M PHY | 2 Mbps | 0.8x | 高速データ転送 |
| Coded S=2 | 500 kbps | 2x | 中距離 |
| Coded S=8 | 125 kbps | 4x | 長距離、屋外 |

### Bluetooth Mesh

```kconfig
# prj.conf - Bluetooth Mesh
CONFIG_BT=y
CONFIG_BT_MESH=y
CONFIG_BT_MESH_RELAY=y
CONFIG_BT_MESH_FRIEND=y
CONFIG_BT_MESH_LOW_POWER=n
CONFIG_BT_MESH_PB_GATT=y
CONFIG_BT_MESH_GATT_PROXY=y

# Provisioning
CONFIG_BT_MESH_PB_ADV=y
CONFIG_BT_MESH_PROVISIONER=n

# Models
CONFIG_BT_MESH_MODEL_EXTENSIONS=y
```

```c
#include <zephyr/bluetooth/mesh.h>

/* Health Server Model */
static struct bt_mesh_health_srv health_srv = {
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/* Generic OnOff Server */
static uint8_t onoff_state;

static int gen_onoff_get(const struct bt_mesh_model *model,
                         struct bt_mesh_msg_ctx *ctx,
                         struct net_buf_simple *buf)
{
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1);
    bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
    net_buf_simple_add_u8(&msg, onoff_state);

    return bt_mesh_model_send(model, ctx, &msg, NULL, NULL);
}

static int gen_onoff_set(const struct bt_mesh_model *model,
                         struct bt_mesh_msg_ctx *ctx,
                         struct net_buf_simple *buf)
{
    onoff_state = net_buf_simple_pull_u8(buf);
    printk("OnOff state: %d\n", onoff_state);

    /* LED制御などの実処理 */
    // gpio_pin_set_dt(&led, onoff_state);

    return gen_onoff_get(model, ctx, buf);
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
    { BT_MESH_MODEL_OP_GEN_ONOFF_GET, 0, gen_onoff_get },
    { BT_MESH_MODEL_OP_GEN_ONOFF_SET, 2, gen_onoff_set },
    BT_MESH_MODEL_OP_END,
};

/* Model Composition */
static const struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
                  NULL, NULL),
};

static const struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = 0x0059,  /* Nordic Semiconductor */
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/* Provisioning Data */
static const uint8_t dev_uuid[16] = { 0x00, 0x01, 0x02, 0x03 /* ... */ };

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .complete = prov_complete,
    .reset = prov_reset,
};

int mesh_init(void)
{
    int err = bt_mesh_init(&prov, &comp);
    if (err) {
        printk("Mesh init failed (err %d)\n", err);
        return err;
    }

    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
    printk("Mesh initialized\n");

    return 0;
}
```

---

## 4. Peripherals - GPIO, TWIM, SPIM, UARTE, SAADC, PWM

### Devicetree Overlay 例

```dts
/* boards/nrf52840dk_nrf52840.overlay */

/ {
    aliases {
        led0 = &led0;
        sw0 = &button0;
    };

    leds {
        compatible = "gpio-leds";
        led0: led_0 {
            gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;
            label = "Green LED";
        };
    };

    buttons {
        compatible = "gpio-keys";
        button0: button_0 {
            gpios = <&gpio0 11 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "Button 0";
        };
    };
};

/* I2C センサー */
&i2c0 {
    status = "okay";
    compatible = "nordic,nrf-twim";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";

    bme280@76 {
        compatible = "bosch,bme280";
        reg = <0x76>;
    };
};

/* SPI Flash */
&spi1 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi1_default>;
    pinctrl-names = "default";
    cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;

    spi_flash: w25q128@0 {
        compatible = "jedec,spi-nor";
        reg = <0>;
        spi-max-frequency = <8000000>;
        jedec-id = [ef 40 18];
        size = <0x8000000>;
    };
};

/* UART */
&uart0 {
    status = "okay";
    current-speed = <115200>;
    pinctrl-0 = <&uart0_default>;
    pinctrl-1 = <&uart0_sleep>;
    pinctrl-names = "default", "sleep";
};

/* ADC */
&adc {
    status = "okay";
};

/* PWM */
&pwm0 {
    status = "okay";
    pinctrl-0 = <&pwm0_default>;
    pinctrl-names = "default";
};
```

### GPIO 制御

```c
#include <zephyr/drivers/gpio.h>

/* Devicetree から GPIO 取得 */
#define LED0_NODE DT_ALIAS(led0)
#define SW0_NODE  DT_ALIAS(sw0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    printk("Button pressed!\n");
    gpio_pin_toggle_dt(&led);
}

int gpio_init(void)
{
    int err;

    /* LED 初期化 */
    if (!gpio_is_ready_dt(&led)) {
        printk("LED device not ready\n");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        return err;
    }

    /* ボタン初期化 + 割り込み */
    if (!gpio_is_ready_dt(&button)) {
        printk("Button device not ready\n");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (err < 0) {
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        return err;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    return 0;
}

/* 操作 */
gpio_pin_set_dt(&led, 1);       /* ON */
gpio_pin_set_dt(&led, 0);       /* OFF */
gpio_pin_toggle_dt(&led);       /* Toggle */
int val = gpio_pin_get_dt(&button);  /* 読み取り */
```

### TWIM (I2C Master)

```c
#include <zephyr/drivers/i2c.h>

#define I2C_NODE DT_NODELABEL(i2c0)

static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

int i2c_sensor_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (!device_is_ready(i2c_dev)) {
        return -ENODEV;
    }

    /* レジスタ書き込み後にデータ読み取り */
    return i2c_write_read(i2c_dev, addr, &reg, 1, data, len);
}

int i2c_sensor_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write(i2c_dev, buf, sizeof(buf), addr);
}

/* BME280 読み取り例 */
void read_bme280(void)
{
    uint8_t data[8];
    int err = i2c_sensor_read(0x76, 0xF7, data, 8);
    if (err == 0) {
        /* データ処理 */
        printk("Raw data: %02x %02x %02x...\n", data[0], data[1], data[2]);
    }
}
```

### SPIM (SPI Master)

```c
#include <zephyr/drivers/spi.h>

#define SPI_NODE DT_NODELABEL(spi1)

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static struct spi_config spi_cfg = {
    .frequency = 4000000,  /* 4MHz */
    .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
};

int spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    struct spi_buf tx = { .buf = tx_buf, .len = len };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    struct spi_buf rx = { .buf = rx_buf, .len = len };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    return spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
}
```

### UARTE (UART)

```c
#include <zephyr/drivers/uart.h>

#define UART_NODE DT_NODELABEL(uart0)

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
static uint8_t rx_buf[64];

/* 割り込みコールバック */
void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        printk("Received %d bytes\n", evt->data.rx.len);
        /* データ処理 */
        break;
    case UART_RX_DISABLED:
        /* 受信再開 */
        uart_rx_enable(dev, rx_buf, sizeof(rx_buf), 100);
        break;
    default:
        break;
    }
}

int uart_init(void)
{
    if (!device_is_ready(uart_dev)) {
        return -ENODEV;
    }

    uart_callback_set(uart_dev, uart_callback, NULL);
    uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 100);

    return 0;
}

void uart_send(const char *str)
{
    for (int i = 0; str[i] != '\0'; i++) {
        uart_poll_out(uart_dev, str[i]);
    }
}
```

### SAADC (ADC)

```c
#include <zephyr/drivers/adc.h>

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_CHANNEL_ID 0
#define ADC_RESOLUTION 12

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_buffer;

static const struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_1_6,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10),
    .channel_id = ADC_CHANNEL_ID,
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0,  /* AIN0 */
};

int adc_init(void)
{
    if (!device_is_ready(adc_dev)) {
        return -ENODEV;
    }

    return adc_channel_setup(adc_dev, &channel_cfg);
}

int adc_read_mv(void)
{
    struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = &adc_buffer,
        .buffer_size = sizeof(adc_buffer),
        .resolution = ADC_RESOLUTION,
    };

    int err = adc_read(adc_dev, &sequence);
    if (err < 0) {
        return err;
    }

    /* mV 変換 (Internal Ref 0.6V, Gain 1/6 → 入力範囲 0-3.6V) */
    int32_t mv = adc_buffer;
    adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_6,
                          ADC_RESOLUTION, &mv);

    return mv;
}
```

### PWM

```c
#include <zephyr/drivers/pwm.h>

#define PWM_NODE DT_NODELABEL(pwm0)
#define PWM_CHANNEL 0

static const struct device *pwm_dev = DEVICE_DT_GET(PWM_NODE);

int pwm_init(void)
{
    if (!device_is_ready(pwm_dev)) {
        return -ENODEV;
    }
    return 0;
}

/* デューティ比設定 (0-100%) */
int pwm_set_duty(uint8_t duty_percent)
{
    uint32_t period_us = 20000;  /* 50Hz (サーボ用) */
    uint32_t pulse_us = (period_us * duty_percent) / 100;

    return pwm_set(pwm_dev, PWM_CHANNEL, PWM_USEC(period_us),
                   PWM_USEC(pulse_us), PWM_POLARITY_NORMAL);
}

/* LED 明るさ調整 */
int pwm_set_led_brightness(uint8_t brightness)
{
    uint32_t period_ns = 1000000;  /* 1kHz */
    uint32_t pulse_ns = (period_ns * brightness) / 255;

    return pwm_set(pwm_dev, PWM_CHANNEL, PWM_NSEC(period_ns),
                   PWM_NSEC(pulse_ns), PWM_POLARITY_NORMAL);
}
```

---

## 5. Power Optimization

### Power Modes

| モード | 電流 (typ.) | RAM保持 | 復帰時間 | 用途 |
|--------|-------------|---------|---------|------|
| **Active** | 3-10mA | Yes | - | 通常動作 |
| **Low Power** | 1.5μA | Yes | 1-2μs | BLE接続待機 |
| **System ON (Idle)** | 1.5μA | Yes | Fast | スリープ |
| **System OFF** | 0.3μA | No | Slow | 超省電力 |

### System ON (Low Power) Mode

```c
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

/* アイドル時に自動的にLow Powerモードに移行 */
/* Kconfig で設定 */
```

```kconfig
# prj.conf - Power Management
CONFIG_PM=y
CONFIG_PM_DEVICE=y
CONFIG_PM_DEVICE_RUNTIME=y
```

### System OFF Mode

```c
#include <zephyr/sys/poweroff.h>
#include <zephyr/drivers/gpio.h>

#define WAKEUP_BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec wakeup_btn = GPIO_DT_SPEC_GET(WAKEUP_BUTTON_NODE, gpios);

void enter_system_off(void)
{
    /* GPIO ウェイクアップ設定 */
    int err = gpio_pin_configure_dt(&wakeup_btn, GPIO_INPUT);
    if (err < 0) {
        printk("Failed to configure wake-up button\n");
        return;
    }

    /* SENSE (レベル検出) でウェイクアップ */
    nrf_gpio_cfg_sense_input(wakeup_btn.pin,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);

    printk("Entering System OFF mode...\n");
    k_msleep(100);  /* ログ出力待ち */

    /* System OFF */
    sys_poweroff();

    /* ここには戻らない（System OFFからはリセットで復帰） */
}

/* ウェイクアップ原因確認 */
void check_reset_reason(void)
{
    uint32_t reason = NRF_POWER->RESETREAS;

    if (reason & POWER_RESETREAS_OFF_Msk) {
        printk("Wake-up from System OFF\n");
    } else if (reason & POWER_RESETREAS_RESETPIN_Msk) {
        printk("Reset pin\n");
    } else {
        printk("Power-on reset\n");
    }

    /* フラグクリア */
    NRF_POWER->RESETREAS = reason;
}
```

### 省電力ベストプラクティス

```c
/* 1. 未使用ペリフェラルの無効化 */
static void disable_unused_peripherals(void)
{
    /* UART無効化（使用しない場合） */
    NRF_UART0->ENABLE = 0;

    /* 未使用GPIOをプルダウン/アップ設定 */
    for (int i = 0; i < 32; i++) {
        if (!gpio_is_used(i)) {
            nrf_gpio_cfg_default(i);
        }
    }
}

/* 2. DC/DC コンバータ有効化 */
static void enable_dcdc(void)
{
    NRF_POWER->DCDCEN = 1;
}

/* 3. 低速クロック設定 */
/* Kconfig */
// CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL=y  /* 外部32.768kHz水晶 */
// CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC=y    /* 内部RC（精度低下） */

/* 4. BLE接続パラメータ最適化 */
static struct bt_le_conn_param conn_param = {
    .interval_min = 800,   /* 1000ms */
    .interval_max = 800,
    .latency = 10,         /* 10回スキップ可能 */
    .timeout = 1000,       /* 10秒タイムアウト */
};
```

### Power Profiler Kit II (PPK2) との連携

```bash
# nRF Connect for Desktop → Power Profiler アプリ

# ソースモード（DK基板給電）で接続
# 1. PPK2 の VOUT を DK の VDD に接続
# 2. GND を共通化
# 3. nRF Connect for Desktop で測定開始

# 測定のコツ
# - "nRF Only" スイッチで CPU のみ測定
# - トリガーを設定して特定イベントをキャプチャ
# - 平均電流・ピーク電流を確認
```

### 消費電流の目安 (nRF52840)

| 状態 | 電流 |
|------|------|
| BLE TX (+4dBm) | 7.4mA |
| BLE TX (0dBm) | 4.8mA |
| BLE RX | 5.4mA |
| CPU Active (64MHz) | 3.3mA |
| System ON (Idle) | 1.5μA |
| System OFF | 0.3μA |
| System OFF + RAM保持 | 1.2μA |

---

## 6. DFU/OTA - MCUboot, nRF Connect App

### MCUboot 設定

nRF Connect SDK では MCUboot が標準ブートローダー。

```kconfig
# prj.conf - MCUboot 有効化
CONFIG_BOOTLOADER_MCUBOOT=y

# DFU over BLE (SMP)
CONFIG_MCUMGR=y
CONFIG_MCUMGR_TRANSPORT_BT=y
CONFIG_MCUMGR_GRP_IMG=y
CONFIG_MCUMGR_GRP_OS=y

# NCS特有の設定
CONFIG_NCS_SAMPLE_MCUMGR_BT_OTA_DFU=y
CONFIG_NCS_SAMPLE_MCUMGR_BT_OTA_DFU_SPEEDUP=y
```

### パーティションレイアウト

```
┌─────────────────────────────────────┐ 0x00000000
│           MCUboot (48KB)            │
├─────────────────────────────────────┤ 0x0000C000
│                                     │
│         Application Slot 0          │
│           (Primary, 440KB)          │
│                                     │
├─────────────────────────────────────┤ 0x00078000
│                                     │
│         Application Slot 1          │
│          (Secondary, 440KB)         │
│                                     │
├─────────────────────────────────────┤ 0x000E4000
│         Settings (24KB)             │
└─────────────────────────────────────┘ 0x000EA000
```

### DFU 実装

```c
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/os_mgmt/os_mgmt.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

int dfu_init(void)
{
    /* イメージ管理初期化 */
    img_mgmt_register_group();

    /* OS管理初期化（リセットコマンド等） */
    os_mgmt_register_group();

    /* BLE SMP トランスポート開始 */
    return smp_bt_register();
}

/* アプリケーションのBLE初期化後に呼び出し */
void main(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed\n");
        return;
    }

    /* DFU 初期化 */
    dfu_init();

    /* 通常のBLEアドバタイジング開始 */
    bt_le_adv_start(...);
}
```

### DFU イメージ生成

```bash
# ビルド時に app_update.bin が生成される
west build -b nrf52840dk/nrf52840 my_app

# 出力ファイル
# build/zephyr/zephyr.hex      - フル書き込み用
# build/zephyr/app_update.bin  - DFU用

# 署名付きイメージ（プロダクション用）
west sign -d build -t imgtool -- --key priv.pem
```

### nRF Connect App でのDFU

```
1. nRF Connect (iOS/Android) アプリを起動
2. スキャンしてデバイスを見つける
3. 接続
4. "DFU" アイコンをタップ
5. app_update.bin を選択
6. アップロード開始
7. 完了後、デバイスがリセットして新ファームウェアで起動
```

### DFU イベントハンドリング

```c
#include <zephyr/dfu/mcuboot.h>

/* ファームウェア確認（ロールバック防止） */
void confirm_image(void)
{
    if (!boot_is_img_confirmed()) {
        int err = boot_write_img_confirmed();
        if (err) {
            printk("Failed to confirm image: %d\n", err);
        } else {
            printk("Image confirmed\n");
        }
    }
}

/* ブート情報取得 */
void print_boot_info(void)
{
    struct mcuboot_img_header header;
    int err = boot_read_bank_header(FIXED_PARTITION_ID(slot0_partition),
                                    &header, sizeof(header));
    if (err == 0) {
        printk("Version: %d.%d.%d+%d\n",
               header.h.v1.sem_ver.major,
               header.h.v1.sem_ver.minor,
               header.h.v1.sem_ver.revision,
               header.h.v1.sem_ver.build_num);
    }
}
```

---

## 7. Debugging - nRF Connect for VS Code, RTT, Segger

### nRF Connect for VS Code デバッグ設定

```json
// .vscode/launch.json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "nRF Debug",
            "type": "nrf-connect",
            "request": "launch",
            "projectFolder": "${workspaceFolder}",
            "buildConfiguration": "build",
            "deviceType": "nRF52840_xxAA",
            "preLaunchTask": "nRF Connect: Build"
        }
    ]
}
```

### RTT (Real-Time Transfer) Logging

SEGGER RTT は UART より高速でCPU負荷が低いログ出力方式。

```kconfig
# prj.conf - RTT 設定
CONFIG_LOG=y
CONFIG_LOG_BACKEND_RTT=y
CONFIG_USE_SEGGER_RTT=y
CONFIG_RTT_CONSOLE=y

# ログレベル
CONFIG_LOG_DEFAULT_LEVEL=3  # INFO
CONFIG_LOG_MAX_LEVEL=4      # DEBUG
```

```c
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(my_module, LOG_LEVEL_DBG);

void example_function(void)
{
    LOG_INF("Information message");
    LOG_WRN("Warning: value=%d", 42);
    LOG_ERR("Error occurred!");
    LOG_DBG("Debug: ptr=%p", (void *)0x1234);

    /* Hexdump */
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    LOG_HEXDUMP_INF(data, sizeof(data), "Data:");
}
```

### RTT Viewer 起動

```bash
# JLinkRTTViewer (GUI)
JLinkRTTViewer

# コマンドライン
JLinkRTTClient

# または nRF Connect for Desktop → Serial Terminal
# RTT選択で接続
```

### GDB デバッグ

```bash
# JLinkGDBServer 起動
JLinkGDBServer -device nRF52840_xxAA -if SWD -speed 4000

# 別ターミナルで GDB 起動
arm-none-eabi-gdb build/zephyr/zephyr.elf
(gdb) target remote localhost:2331
(gdb) monitor reset
(gdb) load
(gdb) break main
(gdb) continue
```

### Ozone デバッガ (SEGGER)

SEGGER Ozone は nRF 開発に最適化されたグラフィカルデバッガ。

```
1. Ozone 起動
2. New Project Wizard
3. Device: nRF52840_xxAA
4. Peripherals: nRF52840.svd
5. ELF File: build/zephyr/zephyr.elf
6. J-Link 接続
7. Debug 開始
```

### ハードフォルト解析

```c
/* Fault Handler (Zephyr) */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
    printk("Fatal error: %u\n", reason);

    if (esf != NULL) {
        printk("PC:  0x%08x\n", esf->basic.pc);
        printk("LR:  0x%08x\n", esf->basic.lr);
        printk("PSR: 0x%08x\n", esf->basic.xpsr);
    }

    /* アドレスからソース行を特定 */
    /* addr2line -e build/zephyr/zephyr.elf 0x00012345 */

    while (1) { }
}
```

```bash
# アドレスからソース行特定
arm-none-eabi-addr2line -e build/zephyr/zephyr.elf 0x00012345
# → src/main.c:42

# 逆アセンブル
arm-none-eabi-objdump -d build/zephyr/zephyr.elf | grep -A 10 "12340:"
```

### メモリ使用量確認

```bash
# ビルド後のサイズ表示
west build -t rom_report
west build -t ram_report

# 詳細表示
arm-none-eabi-size build/zephyr/zephyr.elf
#    text    data     bss     dec     hex filename
#  145328    3248   22456  171032   29c18 zephyr.elf
```

---

## 8. Thread/Matter Support

### Thread 設定 (nRF52840/nRF5340)

```kconfig
# prj.conf - Thread
CONFIG_NETWORKING=y
CONFIG_NET_L2_OPENTHREAD=y
CONFIG_OPENTHREAD_NORDIC_LIBRARY_MTD=y  # Minimal Thread Device

# Thread 機能
CONFIG_OPENTHREAD_THREAD_VERSION_1_3=y
CONFIG_OPENTHREAD_JOINER=y
CONFIG_OPENTHREAD_SLAAC=y

# Network
CONFIG_NET_IPV6=y
CONFIG_NET_UDP=y
CONFIG_NET_SOCKETS=y
```

```c
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>

void thread_init(void)
{
    struct openthread_context *ot_context = openthread_get_default_context();
    otInstance *instance = ot_context->instance;

    /* Dataset 設定 (Joiner または手動) */
    otOperationalDataset dataset;
    memset(&dataset, 0, sizeof(dataset));

    /* Network Name */
    otDatasetSetNetworkName(&dataset, "MyThreadNetwork");

    /* Channel */
    dataset.mChannel = 15;
    dataset.mComponents.mIsChannelPresent = true;

    /* PAN ID */
    dataset.mPanId = 0x1234;
    dataset.mComponents.mIsPanIdPresent = true;

    /* Active Dataset 設定 */
    otDatasetSetActive(instance, &dataset);

    /* Thread 有効化 */
    otIp6SetEnabled(instance, true);
    otThreadSetEnabled(instance, true);
}
```

### Matter 設定

```kconfig
# prj.conf - Matter
CONFIG_CHIP=y
CONFIG_CHIP_PROJECT_CONFIG="include/CHIPProjectConfig.h"
CONFIG_CHIP_LIB_SHELL=y

# Matter over Thread
CONFIG_NET_L2_OPENTHREAD=y
CONFIG_OPENTHREAD_FTD=y

# Matter Device Type (Light Bulb)
CONFIG_CHIP_DEVICE_TYPE=257
```

### Matter サンプル構造

```
matter_app/
├── CMakeLists.txt
├── prj.conf
├── Kconfig
├── include/
│   └── CHIPProjectConfig.h
├── src/
│   ├── main.cpp
│   ├── app_task.cpp
│   └── zcl_callbacks.cpp
└── zap/
    └── lighting-app.zap
```

```cpp
// src/main.cpp - Matter Light Bulb
#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>

using namespace chip;

void MatterEventHandler(const DeviceLayer::ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case DeviceLayer::DeviceEventType::kCommissioningComplete:
        printk("Commissioning complete\n");
        break;
    case DeviceLayer::DeviceEventType::kThreadStateChange:
        printk("Thread state changed\n");
        break;
    default:
        break;
    }
}

int main(void)
{
    CHIP_ERROR err;

    /* Platform 初期化 */
    err = DeviceLayer::PlatformMgr().InitChipStack();
    if (err != CHIP_NO_ERROR) {
        printk("CHIP stack init failed\n");
        return -1;
    }

    /* Event Handler 登録 */
    DeviceLayer::PlatformMgr().AddEventHandler(MatterEventHandler, 0);

    /* Server 起動 */
    err = Server::GetInstance().Init();
    if (err != CHIP_NO_ERROR) {
        printk("Server init failed\n");
        return -1;
    }

    /* イベントループ */
    DeviceLayer::PlatformMgr().RunEventLoop();

    return 0;
}
```

### Matter 対応デバイス (NCS v2.7.0+)

| ボード | Matter over Thread | Matter over Wi-Fi |
|--------|-------------------|-------------------|
| nRF52840 DK | Yes | - |
| nRF5340 DK | Yes | - |
| nRF54L15 DK | Yes | - |
| nRF7002 DK | - | Yes |

---

## nRF Cloud / Edge Impulse Integration

### nRF Cloud 接続

```kconfig
# prj.conf - nRF Cloud (LTE デバイス用)
CONFIG_NRF_CLOUD=y
CONFIG_NRF_CLOUD_MQTT=y
CONFIG_NRF_CLOUD_AGNSS=y
CONFIG_NRF_CLOUD_PGPS=y

# BLE デバイスの場合は nRF Cloud Gateway 経由
```

### Edge Impulse 統合

Edge Impulse はエッジデバイス向けMLプラットフォーム。

```kconfig
# prj.conf - Edge Impulse
CONFIG_EDGE_IMPULSE=y
CONFIG_EDGE_IMPULSE_URI="https://studio.edgeimpulse.com"
CONFIG_CPP=y
CONFIG_STD_CPP17=y
```

```c
#include <edge_impulse/ei_wrapper.h>

/* 推論実行 */
void run_inference(float *features, size_t features_size)
{
    ei_wrapper_result_t result;

    int err = ei_wrapper_start(features, features_size, &result);
    if (err) {
        printk("Inference failed: %d\n", err);
        return;
    }

    /* 結果表示 */
    printk("Predictions:\n");
    for (size_t i = 0; i < result.classification_count; i++) {
        printk("  %s: %.2f\n",
               result.classification[i].label,
               result.classification[i].value);
    }
}
```

---

## Quick Reference

### よく使う west コマンド

```bash
# ビルド
west build -b <board> [path]
west build -p always               # クリーンビルド
west build -t menuconfig           # Kconfig GUI

# フラッシュ & デバッグ
west flash
west debug
west attach                        # RTT接続

# ボード
west boards | grep nrf

# サイズ
west build -t rom_report
west build -t ram_report
```

### 主要 Kconfig オプション

```kconfig
# BLE
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_CENTRAL=y
CONFIG_BT_CTLR_PHY_CODED=y  # Long Range

# ロギング
CONFIG_LOG=y
CONFIG_LOG_BACKEND_RTT=y

# 省電力
CONFIG_PM=y
CONFIG_PM_DEVICE=y

# DFU
CONFIG_BOOTLOADER_MCUBOOT=y
CONFIG_MCUMGR=y
```

### Devicetree マクロ

```c
DT_ALIAS(name)                     // エイリアスからノード取得
DT_NODELABEL(label)                // ラベルからノード取得
DT_PROP(node_id, prop)             // プロパティ取得
DT_NODE_HAS_STATUS(node, okay)     // ノード有効チェック
GPIO_DT_SPEC_GET(node, gpios)      // GPIO spec取得
```

---

## References

- [nRF Connect SDK Documentation](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)
- [nRF Connect SDK GitHub](https://github.com/nrfconnect/sdk-nrf)
- [Nordic DevZone](https://devzone.nordicsemi.com/)
- [Zephyr Documentation](https://docs.zephyrproject.org/)
- [Nordic DevAcademy](https://academy.nordicsemi.com/)
- [nRF52840 Product Page](https://www.nordicsemi.com/Products/nRF52840)
- [nRF5340 Product Page](https://www.nordicsemi.com/Products/nRF5340)
- [nRF54L15 Product Page](https://www.nordicsemi.com/Products/nRF54L15)
- [Matter GitHub](https://github.com/project-chip/connectedhomeip)
