---
name: pico
description: Use when developing firmware for Raspberry Pi Pico. Covers Pico SDK, PIO, MicroPython, and USB device development.
tags:
  - raspberry-pi-pico
  - rp2040
  - rp2350
  - micropython
  - pio
---

# Raspberry Pi Pico Development Guide

## 1. Device Overview / デバイス概要

### Pico Series Comparison / シリーズ比較

| Feature | Pico / Pico H | Pico W / Pico WH | Pico 2 | Pico 2 W |
|---------|---------------|------------------|--------|----------|
| MCU | RP2040 | RP2040 | RP2350 | RP2350 |
| CPU | Dual Cortex-M0+ @ 133MHz | Dual Cortex-M0+ @ 133MHz | Dual Cortex-M33 / Hazard3 @ 150MHz | Dual Cortex-M33 / Hazard3 @ 150MHz |
| SRAM | 264KB | 264KB | 520KB | 520KB |
| Flash | 2MB | 2MB | 4MB | 4MB |
| PIO | 8 state machines (2 PIO) | 8 state machines (2 PIO) | 12 state machines (3 PIO) | 12 state machines (3 PIO) |
| WiFi/BLE | - | CYW43439 (WiFi + BLE 5.2) | - | CYW43439 (WiFi + BLE 5.2) |
| Price | $4 | $6 | $5 | $7 |

### RP2040 vs RP2350 チップ比較

**RP2040:**
- Dual Cortex-M0+ @ 133MHz
- 264KB SRAM (6 banks)
- 2× SPI, 2× I2C, 2× UART, 16× PWM
- 8 PIO state machines
- USB 1.1 Host/Device

**RP2350:**
- Dual-architecture: Cortex-M33 (ARMv8-M) または Hazard3 (RISC-V RV32IMAC+)
- DSP命令、単精度FPU、簡易倍精度FPUコプロセッサ
- 520KB SRAM (10 banks) + 最大16MB PSRAM対応
- 12 PIO state machines (3 PIO blocks)
- HSTX (High-Speed Serial Transmit) for video output
- Security features: TrustZone, Secure Boot, SHA-256, TRNG
- 低消費電力: Dormant state < 20μA

---

## 2. Pico SDK (C/C++) Setup / SDK セットアップ

### Prerequisites / 必要なツール

```bash
# Ubuntu/Debian
sudo apt install cmake python3 build-essential \
    gcc-arm-none-eabi libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib

# macOS
brew install cmake python arm-none-eabi-gcc

# RISC-V toolchain for RP2350 (optional)
# Download from: https://github.com/raspberrypi/pico-sdk-tools
```

### SDK Installation / SDKインストール

```bash
# Clone SDK
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init

# Set environment variable
export PICO_SDK_PATH=$HOME/pico-sdk

# Clone examples
git clone https://github.com/raspberrypi/pico-examples.git
```

### CMakeLists.txt Template / テンプレート

```cmake
cmake_minimum_required(VERSION 3.13...3.27)

# Initialize SDK before project()
include(pico_sdk_import.cmake)

project(my_pico_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(my_app
    main.c
)

# Link libraries
target_link_libraries(my_app
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_pwm
    hardware_i2c
    hardware_spi
    hardware_pio
)

# Enable USB serial output (disable UART)
pico_enable_stdio_usb(my_app 1)
pico_enable_stdio_uart(my_app 0)

# Create UF2 file for drag-and-drop programming
pico_add_extra_outputs(my_app)
```

### Building for Different Boards / ボード指定ビルド

```bash
mkdir build && cd build

# For Pico (default, RP2040)
cmake -DPICO_BOARD=pico ..

# For Pico W
cmake -DPICO_BOARD=pico_w ..

# For Pico 2 (RP2350, ARM)
cmake -DPICO_BOARD=pico2 ..

# For Pico 2 (RP2350, RISC-V)
cmake -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-riscv ..

# For Pico 2 W
cmake -DPICO_BOARD=pico2_w ..

make -j4
```

### Minimal Blink Example (C) / Lチカサンプル

```c
#include "pico/stdlib.h"

#define LED_PIN 25  // On-board LED (Pico), use CYW43 for Pico W

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
}
```

---

## 3. MicroPython / CircuitPython Setup

### MicroPython vs CircuitPython 比較

| Feature | MicroPython | CircuitPython |
|---------|-------------|---------------|
| Official Support | Raspberry Pi公式 | Adafruit (Community) |
| File Transfer | rshell/Thonny経由 | USB MSCドライブ直接 |
| Library Ecosystem | 成長中 | 豊富 (Adafruit製) |
| USB HID | 手動実装 | Built-in |
| Dual Core | 実験的サポート | 限定的 |
| Resource Efficiency | より効率的 | 良好 |

### MicroPython Installation

```bash
# Download UF2 from:
# https://micropython.org/download/RPI_PICO/
# https://micropython.org/download/RPI_PICO_W/
# https://micropython.org/download/RPI_PICO2/

# 1. Hold BOOTSEL button and connect USB
# 2. Drag-and-drop UF2 file to RPI-RP2 drive
```

### REPL Access / REPLアクセス

```bash
# Linux/macOS
screen /dev/ttyACM0 115200

# Or use minicom
minicom -D /dev/ttyACM0 -b 115200

# Or use Thonny IDE (recommended for beginners)
```

### MicroPython Blink Example

```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)  # GPIO 25 for Pico, "LED" for Pico W

while True:
    led.toggle()
    time.sleep(0.5)
```

### Pico W LED (MicroPython)

```python
import network
from machine import Pin

# Pico W uses CYW43 for LED
wlan = network.WLAN(network.STA_IF)
led = Pin("LED", Pin.OUT)  # Special "LED" pin for Pico W

led.on()
```

### CircuitPython Installation

```bash
# Download UF2 from:
# https://circuitpython.org/board/raspberry_pi_pico/
# https://circuitpython.org/board/raspberry_pi_pico_w/
# https://circuitpython.org/board/raspberry_pi_pico2/

# After installation, CIRCUITPY drive appears
# Edit code.py directly on the drive
```

### CircuitPython Example

```python
# code.py
import board
import digitalio
import time

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = not led.value
    time.sleep(0.5)
```

---

## 4. PIO (Programmable I/O)

### PIO Architecture / アーキテクチャ

- RP2040: 2 PIO blocks × 4 state machines = 8 total
- RP2350: 3 PIO blocks × 4 state machines = 12 total
- 32 instruction shared memory per PIO block
- 2 FIFOs per state machine (TX/RX, 4 words each)
- 2 scratch registers (X, Y) per state machine
- クロック分周器: システムクロックの1/65536まで

### PIO Use Cases / ユースケース

- Custom protocols (WS2812B NeoPixel, 1-Wire, etc.)
- Precise timing-critical I/O
- Offloading CPU from bit-banging
- High-speed serial interfaces
- Video output (VGA, DVI via HSTX on RP2350)

### PIO Instructions / 命令セット

```
jmp     - Jump (conditional/unconditional)
wait    - Wait for GPIO/IRQ
in      - Shift data into ISR
out     - Shift data out of OSR
push    - Push ISR to RX FIFO
pull    - Pull from TX FIFO to OSR
mov     - Move data between registers
irq     - Set/clear/wait IRQ
set     - Set pins/pindirs/X/Y
```

### PIO Example: WS2812B LED Driver (C SDK)

**ws2812.pio:**
```
.program ws2812
.side_set 1

.define public T1 2
.define public T2 5
.define public T3 3

.wrap_target
bitloop:
    out x, 1       side 0 [T3 - 1]
    jmp !x do_zero side 1 [T1 - 1]
do_one:
    jmp  bitloop   side 1 [T2 - 1]
do_zero:
    nop            side 0 [T2 - 1]
.wrap
```

**main.c:**
```c
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#define WS2812_PIN 16
#define NUM_LEDS 8

static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

int main() {
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);

    while (true) {
        for (int i = 0; i < NUM_LEDS; i++) {
            put_pixel(pio, sm, urgb_u32(255, 0, 0));  // Red
        }
        sleep_ms(500);

        for (int i = 0; i < NUM_LEDS; i++) {
            put_pixel(pio, sm, urgb_u32(0, 255, 0));  // Green
        }
        sleep_ms(500);
    }
}
```

### PIO Example: Custom Protocol (MicroPython)

```python
import rp2
from machine import Pin

# Square wave generator using PIO
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def square_wave():
    wrap_target()
    set(pins, 1)   [31]  # High for 32 cycles
    set(pins, 0)   [31]  # Low for 32 cycles
    wrap()

# Initialize state machine
sm = rp2.StateMachine(0, square_wave, freq=2000, set_base=Pin(15))
sm.active(1)

# Output: ~31.25 Hz square wave on GPIO 15
```

### PIO Example: SPI Slave (MicroPython)

```python
import rp2
from machine import Pin

@rp2.asm_pio(
    out_init=rp2.PIO.OUT_LOW,
    autopush=True,
    push_thresh=8,
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    out_shiftdir=rp2.PIO.SHIFT_LEFT
)
def spi_slave():
    wrap_target()
    wait(0, pin, 1)      # Wait for CS low
    label("bitloop")
    wait(0, pin, 0)      # Wait for CLK low
    wait(1, pin, 0)      # Wait for CLK high
    in_(pins, 1)         # Sample MOSI
    jmp(pin, "bitloop")  # Continue if CS still low
    wrap()

# sm = rp2.StateMachine(0, spi_slave, in_base=Pin(MOSI), jmp_pin=Pin(CS))
```

---

## 5. Peripherals / ペリフェラル

### GPIO

**C SDK:**
```c
#include "hardware/gpio.h"

// Output
gpio_init(LED_PIN);
gpio_set_dir(LED_PIN, GPIO_OUT);
gpio_put(LED_PIN, 1);

// Input with pull-up
gpio_init(BUTTON_PIN);
gpio_set_dir(BUTTON_PIN, GPIO_IN);
gpio_pull_up(BUTTON_PIN);
bool state = gpio_get(BUTTON_PIN);

// Interrupt
void gpio_callback(uint gpio, uint32_t events) {
    printf("GPIO %d event: %d\n", gpio, events);
}
gpio_set_irq_enabled_with_callback(BUTTON_PIN,
    GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
```

**MicroPython:**
```python
from machine import Pin

# Output
led = Pin(25, Pin.OUT)
led.value(1)

# Input with pull-up
button = Pin(14, Pin.IN, Pin.PULL_UP)
print(button.value())

# Interrupt
def callback(pin):
    print("Button pressed!")
button.irq(trigger=Pin.IRQ_FALLING, handler=callback)
```

### ADC (Analog-to-Digital Converter)

- RP2040/RP2350: 12-bit, 500ksps
- Channels: GPIO26-29 (ADC0-3), internal temperature sensor (ADC4)

**C SDK:**
```c
#include "hardware/adc.h"

adc_init();
adc_gpio_init(26);  // ADC0
adc_select_input(0);

uint16_t result = adc_read();  // 0-4095
float voltage = result * 3.3f / 4096.0f;

// Temperature sensor
adc_set_temp_sensor_enabled(true);
adc_select_input(4);
uint16_t raw = adc_read();
float temp = 27.0f - (raw * 3.3f / 4096.0f - 0.706f) / 0.001721f;
```

**MicroPython:**
```python
from machine import ADC, Pin

# External ADC
adc = ADC(Pin(26))
value = adc.read_u16()  # 0-65535
voltage = value * 3.3 / 65535

# Internal temperature
sensor = ADC(4)
reading = sensor.read_u16() * 3.3 / 65535
temp = 27 - (reading - 0.706) / 0.001721
```

### I2C

**C SDK:**
```c
#include "hardware/i2c.h"

// Initialize I2C0 at 400kHz
i2c_init(i2c0, 400000);
gpio_set_function(4, GPIO_FUNC_I2C);  // SDA
gpio_set_function(5, GPIO_FUNC_I2C);  // SCL
gpio_pull_up(4);
gpio_pull_up(5);

// Write
uint8_t data[] = {0x00, 0x01};
i2c_write_blocking(i2c0, 0x50, data, 2, false);

// Read
uint8_t buf[4];
i2c_read_blocking(i2c0, 0x50, buf, 4, false);

// Write then read (register read pattern)
uint8_t reg = 0x00;
i2c_write_blocking(i2c0, 0x50, &reg, 1, true);  // nostop=true
i2c_read_blocking(i2c0, 0x50, buf, 4, false);
```

**MicroPython:**
```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# Scan for devices
devices = i2c.scan()
print(f"Found devices: {[hex(d) for d in devices]}")

# Write
i2c.writeto(0x50, bytes([0x00, 0x01]))

# Read
data = i2c.readfrom(0x50, 4)

# Write then read
i2c.writeto(0x50, bytes([0x00]))
data = i2c.readfrom(0x50, 4)
```

### SPI

**C SDK:**
```c
#include "hardware/spi.h"

// Initialize SPI0 at 1MHz
spi_init(spi0, 1000000);
gpio_set_function(16, GPIO_FUNC_SPI);  // RX (MISO)
gpio_set_function(17, GPIO_FUNC_SPI);  // CS (manual)
gpio_set_function(18, GPIO_FUNC_SPI);  // SCK
gpio_set_function(19, GPIO_FUNC_SPI);  // TX (MOSI)

// Manual CS control
gpio_init(17);
gpio_set_dir(17, GPIO_OUT);
gpio_put(17, 1);  // CS high (inactive)

// Transfer
uint8_t tx_buf[] = {0x01, 0x02};
uint8_t rx_buf[2];
gpio_put(17, 0);  // CS low
spi_write_read_blocking(spi0, tx_buf, rx_buf, 2);
gpio_put(17, 1);  // CS high
```

**MicroPython:**
```python
from machine import SPI, Pin

spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT, value=1)

# Transfer
tx_data = bytes([0x01, 0x02])
rx_data = bytearray(2)

cs.value(0)
spi.write_readinto(tx_data, rx_data)
cs.value(1)
```

### UART

**C SDK:**
```c
#include "hardware/uart.h"

// Initialize UART0
uart_init(uart0, 115200);
gpio_set_function(0, GPIO_FUNC_UART);  // TX
gpio_set_function(1, GPIO_FUNC_UART);  // RX

// Write
uart_puts(uart0, "Hello UART!\n");

// Read
while (uart_is_readable(uart0)) {
    char c = uart_getc(uart0);
    printf("Received: %c\n", c);
}
```

**MicroPython:**
```python
from machine import UART, Pin

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Write
uart.write("Hello UART!\n")

# Read
if uart.any():
    data = uart.read()
    print(f"Received: {data}")
```

### PWM

**C SDK:**
```c
#include "hardware/pwm.h"

gpio_set_function(15, GPIO_FUNC_PWM);
uint slice = pwm_gpio_to_slice_num(15);
uint channel = pwm_gpio_to_channel(15);

// 1kHz PWM with 50% duty
pwm_set_wrap(slice, 12500 - 1);  // 125MHz / 12500 = 10kHz
pwm_set_clkdiv(slice, 10.0f);    // 10kHz / 10 = 1kHz
pwm_set_chan_level(slice, channel, 6250);  // 50% duty
pwm_set_enabled(slice, true);

// Servo control (50Hz, 1-2ms pulse)
pwm_set_wrap(slice, 20000 - 1);
pwm_set_clkdiv(slice, 125.0f);  // 125MHz / 125 / 20000 = 50Hz
pwm_set_chan_level(slice, channel, 1500);  // 1.5ms = center
```

**MicroPython:**
```python
from machine import Pin, PWM

pwm = PWM(Pin(15))
pwm.freq(1000)      # 1kHz
pwm.duty_u16(32768) # 50% duty (0-65535)

# Servo
servo = PWM(Pin(15))
servo.freq(50)
servo.duty_ns(1500000)  # 1.5ms pulse
```

---

## 6. USB Device Development

### USB Modes

- **CDC (Communications Device Class)**: Serial communication
- **HID (Human Interface Device)**: Keyboard, mouse, gamepad
- **MSC (Mass Storage Class)**: USB drive
- **Composite**: Multiple classes combined

### TinyUSB Integration

TinyUSBはPico SDKに統合されており、USB device/host機能を提供する。

**CMakeLists.txt:**
```cmake
target_link_libraries(my_app
    pico_stdlib
    tinyusb_device
    tinyusb_board
)
```

### USB CDC Serial (Default)

```c
#include "pico/stdlib.h"
#include "tusb.h"

int main() {
    stdio_init_all();  // Enables USB CDC by default

    while (true) {
        printf("Hello USB!\n");
        sleep_ms(1000);
    }
}
```

### USB HID Keyboard Example

**tusb_config.h:**
```c
#define CFG_TUD_HID 1
#define CFG_TUD_HID_EP_BUFSIZE 16
```

**main.c:**
```c
#include "pico/stdlib.h"
#include "tusb.h"
#include "usb_descriptors.h"

// HID report descriptor for keyboard
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD()
};

void hid_task(void) {
    if (!tud_hid_ready()) return;

    // Key 'a' pressed
    uint8_t keycode[6] = {HID_KEY_A, 0, 0, 0, 0, 0};
    tud_hid_keyboard_report(0, 0, keycode);

    sleep_ms(10);

    // Key released
    tud_hid_keyboard_report(0, 0, NULL);
}

int main() {
    board_init();
    tusb_init();

    while (true) {
        tud_task();
        hid_task();
        sleep_ms(100);
    }
}
```

### USB HID with MicroPython (CircuitPython)

```python
# CircuitPython has better USB HID support
import usb_hid
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode
import board
import digitalio
import time

kbd = Keyboard(usb_hid.devices)
button = digitalio.DigitalInOut(board.GP14)
button.switch_to_input(pull=digitalio.Pull.UP)

while True:
    if not button.value:  # Button pressed
        kbd.send(Keycode.A)
        time.sleep(0.1)
```

---

## 7. Pico W WiFi/BLE (CYW43 Driver)

### Architecture / アーキテクチャ

- CYW43439: 2.4GHz 802.11n WiFi + Bluetooth 5.2
- BTstack: Bluetooth Classic + BLE (商用ライセンス付属)
- lwIP: TCP/IP stack

### WiFi Station Mode (C SDK)

**CMakeLists.txt:**
```cmake
target_link_libraries(my_app
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)
```

**main.c:**
```c
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

int main() {
    stdio_init_all();

    // Initialize WiFi
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    // Connect to AP
    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("SSID", "PASSWORD",
            CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect\n");
        return -1;
    }
    printf("Connected!\n");

    // Get IP address
    extern cyw43_t cyw43_state;
    printf("IP: %s\n", ip4addr_ntoa(
        netif_ip4_addr(netif_list)));

    while (true) {
        cyw43_arch_poll();
        sleep_ms(1);
    }
}
```

### WiFi Station Mode (MicroPython)

```python
import network
import time

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("SSID", "PASSWORD")

print("Connecting...")
while not wlan.isconnected():
    time.sleep(1)

print(f"Connected! IP: {wlan.ifconfig()[0]}")
```

### Simple Web Server (MicroPython)

```python
import network
import socket

# Connect to WiFi first (see above)

html = """<!DOCTYPE html>
<html><body>
<h1>Pico W Web Server</h1>
<p>Temperature: {temp:.1f}C</p>
</body></html>
"""

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print(f"Listening on {addr}")

while True:
    cl, addr = s.accept()
    print(f"Client connected from {addr}")

    request = cl.recv(1024)

    # Read temperature
    import machine
    sensor = machine.ADC(4)
    reading = sensor.read_u16() * 3.3 / 65535
    temp = 27 - (reading - 0.706) / 0.001721

    response = html.format(temp=temp)
    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(response)
    cl.close()
```

### BLE Setup (C SDK)

**CMakeLists.txt:**
```cmake
target_link_libraries(my_app
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_lwip_threadsafe_background
)

target_compile_definitions(my_app PRIVATE
    CYW43_ENABLE_BLUETOOTH=1
)
```

### BLE with MicroPython

```python
import bluetooth
from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

class BLEPeripheral:
    def __init__(self, name="Pico"):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)

        # Define service UUID and characteristic
        _SERVICE_UUID = bluetooth.UUID(0x181A)  # Environmental Sensing
        _TEMP_CHAR = (
            bluetooth.UUID(0x2A6E),  # Temperature
            bluetooth.FLAG_READ | bluetooth.FLAG_NOTIFY,
        )
        _SERVICE = (_SERVICE_UUID, (_TEMP_CHAR,),)

        ((self._temp_handle,),) = self._ble.gatts_register_services((_SERVICE,))

        self._advertise(name)

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            print("Connected")
        elif event == _IRQ_CENTRAL_DISCONNECT:
            print("Disconnected")
            self._advertise()

    def _advertise(self, name="Pico"):
        self._ble.gap_advertise(100000, adv_data=b'\x02\x01\x06')

    def set_temperature(self, temp_c):
        # Temperature in 0.01 degrees Celsius
        data = struct.pack('<h', int(temp_c * 100))
        self._ble.gatts_write(self._temp_handle, data)
        self._ble.gatts_notify(0, self._temp_handle)

ble = BLEPeripheral("PicoW-Temp")
```

---

## 8. Debugging (Picoprobe, SWD)

### Debug Options / デバッグ方法

1. **Raspberry Pi Debug Probe** ($12): 公式デバッグプローブ、SWD + UART
2. **Picoprobe**: 別のPicoをデバッガとして使用
3. **printf debugging**: USB/UART経由でログ出力

### Picoprobe Setup

```bash
# 1. Flash picoprobe firmware to debugger Pico
# Download from: https://github.com/raspberrypi/debugprobe/releases

# 2. Connect pins:
# Debugger Pico -> Target Pico
# GP2 (SWCLK)   -> SWCLK
# GP3 (SWDIO)   -> SWDIO
# GND           -> GND
# GP4 (UART TX) -> GP1 (UART RX) [optional]
# GP5 (UART RX) -> GP0 (UART TX) [optional]
```

### OpenOCD Configuration

```bash
# Install OpenOCD (Raspberry Pi fork recommended)
git clone https://github.com/raspberrypi/openocd.git --branch rp2040-v0.12.0
cd openocd
./bootstrap
./configure --enable-cmsis-dap
make -j4
sudo make install

# Run OpenOCD
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg

# For RP2350
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg
```

### Programming via SWD

```bash
# Flash program
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
    -c "adapter speed 5000" \
    -c "program my_app.elf verify reset exit"
```

### GDB Debugging

```bash
# Terminal 1: Start OpenOCD
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg

# Terminal 2: Start GDB
arm-none-eabi-gdb my_app.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) monitor reset init
(gdb) break main
(gdb) continue
```

### VS Code Configuration

**.vscode/launch.json:**
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Pico Debug",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/my_app.elf",
            "servertype": "openocd",
            "configFiles": [
                "interface/cmsis-dap.cfg",
                "target/rp2040.cfg"
            ],
            "searchDir": [],
            "runToEntryPoint": "main",
            "showDevDebugOutput": "none"
        }
    ]
}
```

### Printf Debugging

```c
#include "pico/stdlib.h"
#include <stdio.h>

int main() {
    stdio_init_all();

    // Wait for USB connection (optional)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("Debug: Starting application\n");

    int counter = 0;
    while (true) {
        printf("Counter: %d\n", counter++);
        sleep_ms(1000);
    }
}
```

---

## Quick Reference / クイックリファレンス

### Pin Functions (Pico / Pico 2)

| GPIO | Function 1 | Function 2 | Function 3 |
|------|------------|------------|------------|
| 0-1 | UART0 TX/RX | I2C0 SDA/SCL | SPI0 RX/CS |
| 2-3 | I2C1 SDA/SCL | UART0 CTS/RTS | SPI0 SCK/TX |
| 4-5 | I2C0 SDA/SCL | UART1 TX/RX | SPI0 RX/CS |
| 6-7 | I2C1 SDA/SCL | UART1 CTS/RTS | SPI0 SCK/TX |
| 8-9 | I2C0 SDA/SCL | UART1 TX/RX | SPI1 RX/CS |
| 10-11 | I2C1 SDA/SCL | UART1 CTS/RTS | SPI1 SCK/TX |
| 12-13 | I2C0 SDA/SCL | UART0 TX/RX | SPI1 RX/CS |
| 14-15 | I2C1 SDA/SCL | UART0 CTS/RTS | SPI1 SCK/TX |
| 16-19 | SPI0 | I2C0 | UART0 |
| 20-21 | I2C0 SDA/SCL | UART1 TX/RX | SPI0 RX/CS |
| 22 | GPIO only | | |
| 25 | GPIO (LED) | | |
| 26-29 | ADC0-3 | I2C1 | SPI1 |

### Common Libraries (CMake)

```cmake
pico_stdlib          # Standard library
hardware_gpio        # GPIO
hardware_adc         # ADC
hardware_i2c         # I2C
hardware_spi         # SPI
hardware_uart        # UART
hardware_pwm         # PWM
hardware_pio         # PIO
hardware_timer       # Timer
hardware_watchdog    # Watchdog
hardware_clocks      # Clock management
hardware_dma         # DMA
pico_multicore       # Dual-core support
pico_cyw43_arch_*    # WiFi/BLE (Pico W)
tinyusb_device       # USB Device
tinyusb_host         # USB Host
```

---

## Sources / 参考資料

- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)
- [Pico SDK GitHub](https://github.com/raspberrypi/pico-sdk)
- [Pico Examples](https://github.com/raspberrypi/pico-examples)
- [RP2350 Wikipedia](https://en.wikipedia.org/wiki/RP2350)
- [Pico 2 Datasheet](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf)
- [PIO Introduction - Blues Wireless](https://blues.com/blog/raspberry-pi-pico-pio/)
- [MicroPython vs CircuitPython - Adafruit](https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython/micropython-or-circuitpython)
- [Pico W WiFi Connection Guide](https://vanhunteradams.com/Pico/UDP/Connecting.html)
- [Debug Probe Documentation](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html)
- [TinyUSB](https://github.com/hathach/tinyusb)
