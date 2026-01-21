---
name: zephyr
description: Use when developing with Zephyr RTOS. Covers west build system, devicetree, Kconfig, and multi-board support.
tags:
  - zephyr
  - rtos
  - embedded
  - devicetree
---

# Zephyr RTOS Development Skill

Zephyr RTOSを使った組み込み開発をサポートするスキル。west build system、devicetree、Kconfig、マルチボードサポートをカバーする。

## 1. Overview - Zephyrの特徴

### What is Zephyr?

Zephyrは Linux Foundation が管理するスケーラブルなオープンソースRTOS。2016年に開始され、小規模な組み込みデバイスから複雑なIoTシステムまで対応する。

**Current Version (2024-2025):**
- Latest: **v4.3.0** (November 2025)
- LTS: **v3.7** (July 2024, supported for 2.5 years)
- Release cadence: 6ヶ月ごと (April / October)

**Key Features:**
- Bluetooth 5.0 compliant (ESR10) + BLE Controller
- Bluetooth Mesh support
- 700+ supported boards (200+ STM32, Nordic, ESP32, etc.)
- Multiple architectures: ARM Cortex-M/A/R, x86, RISC-V, Xtensa, ARC
- Security: PSA Secure Storage API, Trusted Execution support
- OpenSSF Gold Badge (security commitment)

### Supported Boards

主要ベンダーのサポート状況:

| Vendor | Board Examples | Notes |
|--------|---------------|-------|
| Nordic Semiconductor | nRF52840 DK, nRF5340 DK, nRF9160 DK | BLE/Thread/LTE-M |
| STMicroelectronics | NUCLEO boards, Discovery kits | 200+ boards, full HAL/LL support |
| Espressif | ESP32, ESP32-S2/S3, ESP32-C3 | Wi-Fi, BLE |
| NXP | i.MX RT, LPC, Kinetis | Industrial grade |
| Raspberry Pi | Pico, Pico W | RP2040 |

```bash
# List all supported boards
west boards

# Filter by vendor
west boards -n nordic
west boards -n stm32
```

### Comparison with Other RTOS

| Feature | Zephyr | FreeRTOS | ThreadX (Azure RTOS) |
|---------|--------|----------|---------------------|
| Governance | Linux Foundation (open) | Amazon | Microsoft |
| ROM (minimal) | ~10-20KB | 6-12KB | ~6KB |
| Built-in Drivers | Extensive | Minimal | Moderate |
| Bluetooth Stack | Built-in | Separate (NimBLE) | Separate |
| Networking | Full IP stack | FreeRTOS+TCP | NetX |
| Configuration | Kconfig + Devicetree | Manual | Manual |
| Real-time Performance | Good (MPU optional) | Excellent | Excellent |
| Best For | IoT, connected devices | Simple embedded | Safety-critical |

**選択の指針:**
- **FreeRTOS**: シンプルで軽量、素早いプロトタイプ開発向け
- **Zephyr**: 長期サポート、セキュリティ重視、スケーラビリティが必要な場合
- **ThreadX**: 医療機器、航空宇宙など認証が必要な分野

---

## 2. West Setup - Workspace Management

### Installation

```bash
# 1. Install dependencies (Ubuntu/Debian)
sudo apt update
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# macOS
brew install cmake ninja gperf python3 ccache qemu dtc wget libmagic

# 2. Install west
pip3 install west

# 3. Initialize workspace (T1: star topology, zephyr as manifest)
west init ~/zephyrproject
cd ~/zephyrproject
west update

# 4. Export Zephyr CMake package
west zephyr-export

# 5. Install Python dependencies
pip3 install -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

### Workspace Topologies

Zephyrは3つのワークスペーストポロジーをサポート:

**T1: Star topology (default)** - Zephyrがmanifest repository
```
zephyrproject/
├── .west/
│   └── config
├── zephyr/              # manifest repository
│   └── west.yml
├── modules/
│   └── lib/zcbor/
└── tools/
```

**T2: Star topology** - Application as manifest (推奨 for products)
```bash
mkdir my-workspace && cd my-workspace
git clone https://github.com/zephyrproject-rtos/example-application my-app
west init -l my-app
west update
```

**T3: Forest topology** - Freestanding manifest repository

### Manifest File (west.yml)

```yaml
# west.yml - Application manifest example
manifest:
  version: "1.0"

  remotes:
    - name: zephyrproject-rtos
      url-base: https://github.com/zephyrproject-rtos

  projects:
    - name: zephyr
      remote: zephyrproject-rtos
      revision: v4.0.0
      import:
        # Only import required HAL modules
        name-allowlist:
          - cmsis
          - hal_nordic
          - hal_stm32

  self:
    path: my-application
```

### Essential West Commands

```bash
# Workspace management
west init -m <url> <directory>    # Initialize from manifest URL
west init -l <path>               # Initialize with local manifest
west update                        # Sync all projects
west list                          # List all projects

# Build
west build -b <board> <app_path>  # Build application
west build -p always              # Pristine build (clean + build)
west build -t menuconfig          # Open Kconfig GUI

# Flash & Debug
west flash                        # Flash to connected board
west debug                        # Start debugger
west debugserver                  # Start debug server

# Utilities
west boards                       # List supported boards
west zephyr-export                # Export CMake package
```

---

## 3. Devicetree - Hardware Description

### Devicetree Basics

Devicetreeはハードウェアをソフトウェアレベルで記述するためのデータ構造。Zephyrではボード固有のハードウェア情報を定義する。

**Key Files:**
- `<board>.dts` - Board devicetree source
- `<board>.overlay` - Application-specific overrides
- `*.yaml` - Binding files (property validation)

### DTS Syntax

```dts
/* Basic node structure */
/ {
    /* Root node */
    model = "My Custom Board";
    compatible = "vendor,board-name";

    chosen {
        zephyr,console = &uart0;
        zephyr,shell-uart = &uart0;
        zephyr,sram = &sram0;
        zephyr,flash = &flash0;
    };

    aliases {
        led0 = &green_led;
        sw0 = &user_button;
    };

    leds {
        compatible = "gpio-leds";
        green_led: led_0 {
            gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;
            label = "Green LED";
        };
    };

    buttons {
        compatible = "gpio-keys";
        user_button: button_0 {
            gpios = <&gpio0 11 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "User Button";
        };
    };
};
```

### Overlay Files

オーバーレイでボードDTSをアプリケーション向けにカスタマイズ:

```dts
/* boards/nrf52840dk_nrf52840.overlay */

/* Enable I2C sensor */
&i2c0 {
    status = "okay";

    bme280@76 {
        compatible = "bosch,bme280";
        reg = <0x76>;
        label = "BME280";
    };
};

/* Change UART baud rate */
&uart0 {
    current-speed = <921600>;
};

/* Add SPI device */
&spi1 {
    status = "okay";
    cs-gpios = <&gpio0 25 GPIO_ACTIVE_LOW>;

    spi_flash: w25q128@0 {
        compatible = "jedec,spi-nor";
        reg = <0>;
        spi-max-frequency = <80000000>;
        jedec-id = [ef 40 18];
        size = <0x8000000>;  /* 128 Mbit */
        label = "W25Q128";
    };
};
```

### Overlay File Placement

```
my-app/
├── CMakeLists.txt
├── prj.conf
├── src/
│   └── main.c
└── boards/
    ├── nrf52840dk_nrf52840.overlay    # Board-specific
    └── nucleo_f446re.overlay
```

Build systemは自動的に `boards/<BOARD>.overlay` を検出して適用する。

### Bindings

BindingsはDTSノードのプロパティを検証するYAMLファイル:

```yaml
# dts/bindings/sensor/my-sensor.yaml
description: My custom sensor

compatible: "vendor,my-sensor"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  int-gpios:
    type: phandle-array
    required: true
    description: Interrupt GPIO

  sample-rate:
    type: int
    default: 100
    description: Sampling rate in Hz
```

### Accessing Devicetree in Code

```c
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

/* Node identifiers */
#define LED0_NODE DT_ALIAS(led0)
#define BTN_NODE  DT_ALIAS(sw0)

/* Check if node exists */
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "LED0 not defined in devicetree"
#endif

/* Get GPIO spec from devicetree */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BTN_NODE, gpios);

/* Get properties */
#define UART_BAUD DT_PROP(DT_NODELABEL(uart0), current_speed)

int main(void)
{
    if (!gpio_is_ready_dt(&led)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_toggle_dt(&led);

    return 0;
}
```

---

## 4. Kconfig - Configuration System

### Overview

KconfigはLinux Kernelと同じ設定システム。ビルド時にカーネルとサブシステムを構成する。

**Configuration Sources (in order of precedence):**
1. Command line: `-DCONFIG_XXX=y`
2. Application: `prj.conf`
3. Board-specific: `boards/<BOARD>.conf`
4. Board defconfig: `<BOARD>_defconfig`

### prj.conf Syntax

```kconfig
# prj.conf - Application configuration

# Basic settings
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_HEAP_MEM_POOL_SIZE=4096

# Logging
CONFIG_LOG=y
CONFIG_LOG_MODE_DEFERRED=y
CONFIG_LOG_BUFFER_SIZE=2048

# Shell
CONFIG_SHELL=y
CONFIG_SHELL_BACKENDS=y

# Bluetooth
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_DEVICE_NAME="My BLE Device"

# Networking
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_TCP=y
CONFIG_NET_SOCKETS=y

# Debug
CONFIG_DEBUG=y
CONFIG_ASSERT=y
CONFIG_THREAD_NAME=y
```

### Board-Specific Configuration

```
my-app/
├── prj.conf                           # Common config
├── boards/
│   ├── nrf52840dk_nrf52840.conf      # Nordic-specific
│   └── nucleo_f446re.conf            # STM32-specific
```

```kconfig
# boards/nrf52840dk_nrf52840.conf
CONFIG_BT_CTLR=y
CONFIG_BT_LL_SW_SPLIT=y
```

### Interactive Configuration

```bash
# GUI configuration
west build -t menuconfig    # Terminal-based
west build -t guiconfig     # Qt-based GUI

# Generate diff from current build
west build -t hardenconfig  # Security hardening suggestions
```

### Common Configuration Options

**Kernel:**
```kconfig
CONFIG_NUM_PREEMPT_PRIORITIES=16
CONFIG_MAIN_STACK_SIZE=1024
CONFIG_ISR_STACK_SIZE=2048
CONFIG_IDLE_STACK_SIZE=256
CONFIG_TIMESLICE_SIZE=0
CONFIG_SCHED_MULTIQ=y
```

**Memory:**
```kconfig
CONFIG_HEAP_MEM_POOL_SIZE=8192
CONFIG_MINIMAL_LIBC_MALLOC_ARENA_SIZE=0
CONFIG_PICOLIBC=y
```

**Security:**
```kconfig
CONFIG_HW_STACK_PROTECTION=y
CONFIG_MPU_STACK_GUARD=y
CONFIG_STACK_CANARIES=y
CONFIG_USERSPACE=y
```

---

## 5. Kernel Primitives

### Threads

```c
#include <zephyr/kernel.h>

#define STACK_SIZE 1024
#define PRIORITY 5

K_THREAD_STACK_DEFINE(thread_stack, STACK_SIZE);
struct k_thread thread_data;

void thread_entry(void *p1, void *p2, void *p3)
{
    while (1) {
        printk("Thread running\n");
        k_msleep(1000);
    }
}

int main(void)
{
    k_tid_t tid = k_thread_create(&thread_data, thread_stack,
                                   K_THREAD_STACK_SIZEOF(thread_stack),
                                   thread_entry,
                                   NULL, NULL, NULL,
                                   PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(tid, "my_thread");

    /* Wait for thread to finish */
    k_thread_join(tid, K_FOREVER);

    return 0;
}
```

### Semaphores

```c
#include <zephyr/kernel.h>

/* Binary semaphore */
K_SEM_DEFINE(my_sem, 0, 1);

/* Counting semaphore (initial=3, max=10) */
K_SEM_DEFINE(resource_sem, 3, 10);

void producer_thread(void)
{
    while (1) {
        /* Produce data... */
        k_sem_give(&my_sem);  /* Signal consumer */
        k_msleep(100);
    }
}

void consumer_thread(void)
{
    while (1) {
        /* Wait for data (with timeout) */
        if (k_sem_take(&my_sem, K_MSEC(500)) == 0) {
            /* Process data... */
        } else {
            printk("Timeout waiting for data\n");
        }
    }
}
```

### Mutexes

```c
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(my_mutex);

struct shared_data {
    int value;
    char buffer[64];
};

static struct shared_data data;

void thread_safe_update(int new_value)
{
    /* Mutex supports priority inheritance */
    k_mutex_lock(&my_mutex, K_FOREVER);

    data.value = new_value;
    snprintf(data.buffer, sizeof(data.buffer), "Value: %d", new_value);

    k_mutex_unlock(&my_mutex);
}
```

### Message Queues

```c
#include <zephyr/kernel.h>

struct sensor_msg {
    uint32_t timestamp;
    int16_t temperature;
    int16_t humidity;
};

K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_msg), 10, 4);

void sensor_thread(void)
{
    struct sensor_msg msg;

    while (1) {
        msg.timestamp = k_uptime_get_32();
        msg.temperature = read_temperature();
        msg.humidity = read_humidity();

        /* Non-blocking put */
        if (k_msgq_put(&sensor_msgq, &msg, K_NO_WAIT) != 0) {
            /* Queue full - purge oldest */
            k_msgq_purge(&sensor_msgq);
        }

        k_msleep(100);
    }
}

void process_thread(void)
{
    struct sensor_msg msg;

    while (1) {
        /* Blocking get */
        k_msgq_get(&sensor_msgq, &msg, K_FOREVER);
        printk("T=%d: temp=%d, hum=%d\n",
               msg.timestamp, msg.temperature, msg.humidity);
    }
}
```

### Timers

```c
#include <zephyr/kernel.h>

void timer_expiry_fn(struct k_timer *timer)
{
    /* Called from ISR context */
    printk("Timer expired!\n");
}

void timer_stop_fn(struct k_timer *timer)
{
    printk("Timer stopped\n");
}

K_TIMER_DEFINE(my_timer, timer_expiry_fn, timer_stop_fn);

int main(void)
{
    /* One-shot timer: 1 second delay */
    k_timer_start(&my_timer, K_SECONDS(1), K_NO_WAIT);

    /* Periodic timer: 100ms initial, 500ms period */
    k_timer_start(&my_timer, K_MSEC(100), K_MSEC(500));

    /* Get remaining time */
    uint32_t remaining = k_timer_remaining_get(&my_timer);

    /* Stop timer */
    k_timer_stop(&my_timer);

    return 0;
}
```

### Work Queues

```c
#include <zephyr/kernel.h>

/* Use system workqueue */
struct k_work my_work;

void work_handler(struct k_work *work)
{
    /* This runs in workqueue thread context (not ISR) */
    printk("Work executed\n");
}

/* Delayed work */
struct k_work_delayable delayed_work;

void delayed_work_handler(struct k_work *work)
{
    printk("Delayed work executed\n");

    /* Reschedule */
    k_work_schedule(&delayed_work, K_MSEC(1000));
}

int main(void)
{
    /* Initialize work items */
    k_work_init(&my_work, work_handler);
    k_work_init_delayable(&delayed_work, delayed_work_handler);

    /* Submit to system workqueue */
    k_work_submit(&my_work);

    /* Schedule delayed work (1 second from now) */
    k_work_schedule(&delayed_work, K_SECONDS(1));

    return 0;
}
```

---

## 6. Subsystems

### Logging

```c
#include <zephyr/logging/log.h>

/* Register module for logging */
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

**prj.conf:**
```kconfig
CONFIG_LOG=y
CONFIG_LOG_MODE_DEFERRED=y
CONFIG_LOG_BUFFER_SIZE=4096
CONFIG_LOG_BACKEND_UART=y
CONFIG_LOG_PROCESS_THREAD_STACK_SIZE=2048
```

### Shell

```c
#include <zephyr/shell/shell.h>

static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Hello, %s!", argc > 1 ? argv[1] : "World");
    return 0;
}

static int cmd_led_on(const struct shell *sh, size_t argc, char **argv)
{
    gpio_pin_set_dt(&led, 1);
    shell_print(sh, "LED ON");
    return 0;
}

static int cmd_led_off(const struct shell *sh, size_t argc, char **argv)
{
    gpio_pin_set_dt(&led, 0);
    shell_print(sh, "LED OFF");
    return 0;
}

/* Subcommand hierarchy */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_led,
    SHELL_CMD(on, NULL, "Turn LED on", cmd_led_on),
    SHELL_CMD(off, NULL, "Turn LED off", cmd_led_off),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(hello, NULL, "Say hello", cmd_hello);
SHELL_CMD_REGISTER(led, &sub_led, "LED control", NULL);
```

**prj.conf:**
```kconfig
CONFIG_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL=y
CONFIG_SHELL_CMD_BUFF_SIZE=256
CONFIG_SHELL_PRINTF_BUFF_SIZE=128
```

### Settings (Persistent Storage)

```c
#include <zephyr/settings/settings.h>

static int32_t my_value = 42;
static char my_string[32] = "default";

static int settings_set(const char *name, size_t len,
                        settings_read_cb read_cb, void *cb_arg)
{
    if (!strcmp(name, "value")) {
        return read_cb(cb_arg, &my_value, sizeof(my_value));
    }
    if (!strcmp(name, "string")) {
        return read_cb(cb_arg, my_string, sizeof(my_string));
    }
    return -ENOENT;
}

static int settings_export(int (*cb)(const char *name, const void *value,
                                     size_t val_len))
{
    cb("myapp/value", &my_value, sizeof(my_value));
    cb("myapp/string", my_string, strlen(my_string) + 1);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(myapp, "myapp", NULL, settings_set,
                               NULL, settings_export);

int main(void)
{
    int err;

    /* Initialize settings subsystem */
    err = settings_subsys_init();
    if (err) {
        LOG_ERR("Settings init failed: %d", err);
        return err;
    }

    /* Load saved settings */
    settings_load();

    /* Modify and save */
    my_value = 100;
    settings_save_one("myapp/value", &my_value, sizeof(my_value));

    return 0;
}
```

**prj.conf:**
```kconfig
CONFIG_SETTINGS=y
CONFIG_SETTINGS_RUNTIME=y
CONFIG_NVS=y
CONFIG_SETTINGS_NVS=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_FLASH_PAGE_LAYOUT=y
```

### Bluetooth LE

```c
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>

#define DEVICE_NAME "Zephyr BLE"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int main(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed (err %d)", err);
        return err;
    }

    LOG_INF("Advertising started");

    return 0;
}
```

**prj.conf:**
```kconfig
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_DEVICE_NAME="Zephyr BLE"
CONFIG_BT_DEVICE_APPEARANCE=0
CONFIG_BT_MAX_CONN=1
CONFIG_BT_MAX_PAIRED=1

# For Nordic boards with built-in controller
CONFIG_BT_CTLR=y
```

### Networking (TCP/IP)

```c
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>

void tcp_client_example(void)
{
    int sock;
    struct sockaddr_in addr;
    char buf[128];

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Socket creation failed");
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(8080);
    inet_pton(AF_INET, "192.168.1.100", &addr.sin_addr);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Connection failed");
        close(sock);
        return;
    }

    send(sock, "Hello Server!", 13, 0);

    int len = recv(sock, buf, sizeof(buf) - 1, 0);
    if (len > 0) {
        buf[len] = '\0';
        LOG_INF("Received: %s", buf);
    }

    close(sock);
}
```

**prj.conf:**
```kconfig
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_TCP=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_SOCKETS_POSIX_NAMES=y

# For Ethernet
CONFIG_NET_L2_ETHERNET=y
CONFIG_ETH_DRIVER=y

# DHCP
CONFIG_NET_DHCPV4=y
```

---

## 7. Building & Flashing

### Basic Build Commands

```bash
# Build for specific board
west build -b nrf52840dk_nrf52840 samples/basic/blinky

# Build with specific config
west build -b nucleo_f446re -- -DCONF_FILE=prj_debug.conf

# Build with overlay
west build -b nrf52840dk_nrf52840 -- -DDTC_OVERLAY_FILE=custom.overlay

# Pristine build (clean first)
west build -p always -b esp32_devkitc_wroom

# Verbose output
west build -b stm32f4_disco -- -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Multiple Configurations

```bash
# Merge multiple conf files
west build -b nrf52840dk_nrf52840 -- \
    -DCONF_FILE="prj.conf;boards/nrf52840dk_nrf52840.conf;debug.conf"

# Extra overlay files
west build -b nrf52840dk_nrf52840 -- \
    -DEXTRA_DTC_OVERLAY_FILE="sensors.overlay;leds.overlay"
```

### Flash Commands

```bash
# Flash built application
west flash

# Flash with specific runner
west flash --runner jlink
west flash --runner openocd
west flash --runner nrfjprog
west flash --runner stm32cubeprogrammer

# Erase flash before programming
west flash --erase

# Flash specific hex file
west flash --hex-file build/zephyr/zephyr.hex

# Show available runners
west flash -H

# Show runner-specific options
west flash -H -r jlink
```

### Debug Commands

```bash
# Start GDB session
west debug

# Start debug server (connect with external GDB)
west debugserver

# Attach to running target
west attach

# Debug with specific runner
west debug --runner jlink
west debugserver --runner openocd

# Show debug context
west debug --context
```

### Build System Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `BOARD` | Target board | `nrf52840dk_nrf52840` |
| `CONF_FILE` | Kconfig files | `prj.conf;debug.conf` |
| `DTC_OVERLAY_FILE` | Devicetree overlays | `app.overlay` |
| `EXTRA_CONF_FILE` | Additional Kconfig | `extra.conf` |
| `EXTRA_DTC_OVERLAY_FILE` | Additional overlays | `sensors.overlay` |

### CMakeLists.txt Example

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(my_app)

# Add source files
target_sources(app PRIVATE
    src/main.c
    src/sensor.c
    src/bluetooth.c
)

# Add include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Conditional compilation
if(CONFIG_BT)
    target_sources(app PRIVATE src/ble_service.c)
endif()
```

---

## 8. Board Porting Basics

### Board Directory Structure

```
boards/
└── arm/
    └── my_custom_board/
        ├── Kconfig.board           # Board Kconfig menu
        ├── Kconfig.defconfig       # Default Kconfig values
        ├── my_custom_board_defconfig  # Board defconfig
        ├── my_custom_board.dts     # Board devicetree
        ├── my_custom_board.yaml    # Board metadata
        ├── board.cmake             # Flash/debug runner config
        ├── pinctrl.dtsi            # Pin control definitions
        └── doc/
            └── index.rst           # Board documentation
```

### Kconfig Files

**Kconfig.board:**
```kconfig
config BOARD_MY_CUSTOM_BOARD
    bool "My Custom Board"
    depends on SOC_STM32F446XX
    select SOC_SERIES_STM32F4X
```

**Kconfig.defconfig:**
```kconfig
if BOARD_MY_CUSTOM_BOARD

config BOARD
    default "my_custom_board"

# Enable console by default
config UART_CONSOLE
    default y

endif # BOARD_MY_CUSTOM_BOARD
```

**my_custom_board_defconfig:**
```kconfig
CONFIG_SOC_SERIES_STM32F4X=y
CONFIG_SOC_STM32F446XX=y
CONFIG_BOARD_MY_CUSTOM_BOARD=y

CONFIG_ARM_MPU=y
CONFIG_HW_STACK_PROTECTION=y

CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

CONFIG_GPIO=y
CONFIG_CLOCK_CONTROL=y

CONFIG_PINCTRL=y
```

### Devicetree (board.dts)

```dts
/dts-v1/;
#include <st/f4/stm32f446Xe.dtsi>
#include <st/f4/stm32f446r(c-e)tx-pinctrl.dtsi>
#include "pinctrl.dtsi"

/ {
    model = "My Custom Board";
    compatible = "vendor,my-custom-board";

    chosen {
        zephyr,console = &usart2;
        zephyr,shell-uart = &usart2;
        zephyr,sram = &sram0;
        zephyr,flash = &flash0;
    };

    leds {
        compatible = "gpio-leds";
        user_led: led_0 {
            gpios = <&gpioa 5 GPIO_ACTIVE_HIGH>;
            label = "User LED";
        };
    };

    aliases {
        led0 = &user_led;
    };
};

&clk_hse {
    clock-frequency = <DT_FREQ_M(8)>;
    status = "okay";
};

&pll {
    div-m = <4>;
    mul-n = <180>;
    div-p = <2>;
    div-q = <8>;
    clocks = <&clk_hse>;
    status = "okay";
};

&rcc {
    clocks = <&pll>;
    clock-frequency = <DT_FREQ_M(180)>;
    ahb-prescaler = <1>;
    apb1-prescaler = <4>;
    apb2-prescaler = <2>;
};

&usart2 {
    pinctrl-0 = <&usart2_tx_pa2 &usart2_rx_pa3>;
    pinctrl-names = "default";
    current-speed = <115200>;
    status = "okay";
};

&i2c1 {
    pinctrl-0 = <&i2c1_scl_pb8 &i2c1_sda_pb9>;
    pinctrl-names = "default";
    status = "okay";
};
```

### Board Metadata (board.yaml)

```yaml
identifier: my_custom_board
name: My Custom Board
type: mcu
arch: arm
toolchain:
  - zephyr
  - gnuarmemb
ram: 128
flash: 512
supported:
  - gpio
  - i2c
  - spi
  - uart
  - usb
  - pwm
vendor: vendor
```

### Flash/Debug Configuration (board.cmake)

```cmake
# board.cmake
board_runner_args(jlink "--device=STM32F446RE" "--speed=4000")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu)
board_runner_args(stm32cubeprogrammer "--port=swd")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
```

### Out-of-Tree Board

アプリケーション内でカスタムボードを定義:

```
my-app/
├── CMakeLists.txt
├── prj.conf
├── src/
│   └── main.c
└── boards/
    └── arm/
        └── my_board/
            ├── ...
```

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)

# Point to custom boards
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(my_app)
target_sources(app PRIVATE src/main.c)
```

```bash
# Build with custom board
west build -b my_board
```

---

## Quick Reference

### Useful West Commands

```bash
# Workspace
west init / west update / west list

# Build
west build -b <board> [path]
west build -p always          # Pristine build
west build -t menuconfig      # Kconfig GUI

# Flash & Debug
west flash / west debug / west debugserver

# Info
west boards                   # List boards
west flash --context          # Show flash info
```

### Common Kconfig Patterns

```kconfig
# Enable feature
CONFIG_FEATURE=y

# Disable feature
CONFIG_FEATURE=n

# Set numeric value
CONFIG_STACK_SIZE=2048

# Set string
CONFIG_DEVICE_NAME="My Device"
```

### Devicetree Macros

```c
DT_ALIAS(name)              // Get node by alias
DT_NODELABEL(label)         // Get node by label
DT_PROP(node_id, prop)      // Get property value
DT_NODE_HAS_STATUS(node, okay)  // Check if node is enabled
GPIO_DT_SPEC_GET(node, prop)    // Get GPIO spec
```

---

## References

- [Zephyr Project Documentation](https://docs.zephyrproject.org/latest/)
- [Zephyr GitHub Repository](https://github.com/zephyrproject-rtos/zephyr)
- [Zephyr Releases](https://github.com/zephyrproject-rtos/zephyr/releases)
- [Supported Boards](https://docs.zephyrproject.org/latest/boards/index.html)
- [Board Porting Guide](https://docs.zephyrproject.org/latest/hardware/porting/board_porting.html)
- [Devicetree Guide](https://docs.zephyrproject.org/latest/build/dts/howtos.html)
- [Kconfig Tips](https://docs.zephyrproject.org/latest/build/kconfig/tips.html)
- [West Documentation](https://docs.zephyrproject.org/latest/develop/west/index.html)
