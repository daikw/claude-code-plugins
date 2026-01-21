---
name: embedded-basics
description: Use when learning embedded systems fundamentals. Covers MCU architecture, memory, peripherals, debugging, and development workflow.
tags:
  - embedded
  - mcu
  - firmware
---

# Embedded Systems Basics (組込基礎)

MCU（マイコン）を使った組込システム開発の基礎知識。アーキテクチャ、メモリ、ペリフェラル、デバッグ手法を体系的にカバー。

## When to Activate

- MCU/マイコンの基礎を学びたいとき
- ARM Cortex-M 系の開発を始めるとき
- リンカスクリプトやメモリマップを理解したいとき
- ペリフェラル（GPIO, Timer, ADC等）の仕組みを知りたいとき
- 組込みデバッグ（JTAG/SWD, GDB）を行うとき
- 割り込みやステートマシンの実装パターンを学びたいとき

## MCU Architecture Basics

### MCU の構成要素

```
┌─────────────────────────────────────────────────────────────┐
│                        MCU                                   │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│  │   CPU   │  │  Flash  │  │   RAM   │  │  NVIC   │        │
│  │(Cortex) │  │ (Code)  │  │ (Data)  │  │(割込み) │        │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘        │
│       │            │            │            │              │
│  ═════╪════════════╪════════════╪════════════╪═══════════   │
│       │         System Bus (AHB/APB)         │              │
│  ═════╪════════════╪════════════╪════════════╪═══════════   │
│       │            │            │            │              │
│  ┌────┴────┐  ┌────┴────┐  ┌────┴────┐  ┌────┴────┐        │
│  │  GPIO   │  │  Timer  │  │   ADC   │  │  UART   │        │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘        │
│                     Peripherals                             │
└─────────────────────────────────────────────────────────────┘
```

### ARM Cortex-M ファミリ比較

| コア | パイプライン | FPU | DSP | 用途 |
|------|------------|-----|-----|------|
| **Cortex-M0/M0+** | 2段 | なし | なし | 超低消費電力、シンプル |
| **Cortex-M3** | 3段 | なし | あり | 汎用、バランス型 |
| **Cortex-M4** | 3段 | オプション | あり | DSP処理、モーター制御 |
| **Cortex-M7** | 6段 | 倍精度 | あり | 高性能、リアルタイム処理 |
| **Cortex-M23** | 2段 | なし | なし | セキュリティ (TrustZone) |
| **Cortex-M33** | 3段 | オプション | あり | セキュリティ + DSP |
| **Cortex-M55** | - | あり | MVE | AI/ML エッジ処理 |
| **Cortex-M85** | - | あり | MVE | 最高性能、TrustZone |

### 代表的な MCU ベンダー・シリーズ

| ベンダー | シリーズ | コア | 特徴 |
|---------|---------|------|------|
| **STMicroelectronics** | STM32F4 | Cortex-M4 | 豊富なペリフェラル、CubeMX |
| | STM32H7 | Cortex-M7 | 高性能、デュアルコア版あり |
| | STM32L4 | Cortex-M4 | 低消費電力 |
| **NXP** | LPC1768 | Cortex-M3 | mbed対応 |
| | i.MX RT | Cortex-M7 | クロスオーバープロセッサ |
| **Nordic** | nRF52840 | Cortex-M4 | BLE 5.0、低消費電力 |
| **Espressif** | ESP32-S3 | Xtensa LX7 | WiFi/BLE、AI対応 |
| **Raspberry Pi** | RP2040 | Cortex-M0+ (x2) | 低コスト、PIO |

## Memory Model

### メモリマップ（ARM Cortex-M 典型例）

```
アドレス空間 (32bit = 4GB)
┌──────────────────────────────────────┐ 0xFFFF_FFFF
│          Vendor-specific             │
├──────────────────────────────────────┤ 0xE010_0000
│     Private Peripheral Bus (PPB)     │ ← System Control, NVIC
├──────────────────────────────────────┤ 0xE000_0000
│         External Device              │
├──────────────────────────────────────┤ 0xA000_0000
│         External RAM                 │
├──────────────────────────────────────┤ 0x6000_0000
│       Peripheral (APB/AHB)           │ ← GPIO, Timer, UART等
├──────────────────────────────────────┤ 0x4000_0000
│              SRAM                    │ ← スタック、ヒープ、変数
├──────────────────────────────────────┤ 0x2000_0000
│              Code                    │ ← Flash（プログラム格納）
├──────────────────────────────────────┤ 0x0000_0000
│         Vector Table                 │ ← 割り込みベクタ
└──────────────────────────────────────┘
```

### Flash と RAM の役割

| 領域 | 特徴 | 格納内容 |
|------|------|---------|
| **Flash (ROM)** | 不揮発性、読み込み専用（実行時）| コード (.text)、定数 (.rodata)、初期値 (.data の初期値) |
| **RAM (SRAM)** | 揮発性、読み書き可能 | 変数 (.data, .bss)、スタック、ヒープ |

### セクション構成

```
Flash Memory                          RAM Memory
┌────────────────────┐               ┌────────────────────┐
│   Vector Table     │ .isr_vector   │                    │
├────────────────────┤               │      Stack         │ ↓成長
│                    │               ├────────────────────┤
│   Code (.text)     │               │                    │
│                    │               │      Heap          │ ↑成長
├────────────────────┤               ├────────────────────┤
│  Read-only Data    │ .rodata       │   .bss (ゼロ初期化)│
├────────────────────┤               ├────────────────────┤
│  Initial Values    │ .data (LMA)   │   .data (VMA)      │
│  for .data         │ ──コピー──→   │   (初期化済み変数) │
└────────────────────┘               └────────────────────┘
```

### Linker Script 基礎

リンカスクリプトは、コンパイルされたオブジェクトファイルをどのメモリ領域に配置するかを定義する。

```ld
/* STM32F4 用リンカスクリプト例 */

/* メモリ領域定義 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
}

/* スタックサイズ定義 */
_Min_Stack_Size = 0x400;   /* 1KB */
_Min_Heap_Size  = 0x200;   /* 512B */

/* エントリポイント */
ENTRY(Reset_Handler)

/* セクション配置 */
SECTIONS
{
    /* Vector Table（Flashの先頭） */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Code セクション */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)         /* ARM/Thumb interworking */
        *(.glue_7t)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Read-only Data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* Initialized Data（FlashからRAMへコピー） */
    _sidata = LOADADDR(.data);  /* Flash上のアドレス (LMA) */

    .data :
    {
        . = ALIGN(4);
        _sdata = .;             /* RAMでの開始アドレス (VMA) */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH          /* VMA: RAM, LMA: Flash */

    /* Uninitialized Data（ゼロ初期化） */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* ヒープとスタック */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /* スタックトップ（RAMの末尾） */
    _estack = ORIGIN(RAM) + LENGTH(RAM);
}
```

### スタートアップコード

スタートアップコードは、main() 関数が呼ばれる前の初期化処理を行う。

```c
/* startup.c - Reset Handler */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);

void Reset_Handler(void) {
    uint32_t *src, *dst;

    /* 1. .data セクションを Flash から RAM へコピー */
    src = &_sidata;
    dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* 2. .bss セクションをゼロ初期化 */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    /* 3. システムクロック初期化（HAL使用時） */
    SystemInit();

    /* 4. main関数呼び出し */
    main();

    /* main から戻ってきた場合は無限ループ */
    while (1);
}

/* Vector Table */
__attribute__((section(".isr_vector")))
const uint32_t vector_table[] = {
    (uint32_t)&_estack,           /* Initial Stack Pointer */
    (uint32_t)Reset_Handler,      /* Reset Handler */
    (uint32_t)NMI_Handler,        /* NMI Handler */
    (uint32_t)HardFault_Handler,  /* Hard Fault Handler */
    /* ... 他の割り込みハンドラ ... */
};
```

### メモリ使用量の確認

```bash
# ELFファイルのセクションサイズ確認
arm-none-eabi-size firmware.elf
#    text    data     bss     dec     hex filename
#   12340     120    2048   14508    38ac firmware.elf

# 詳細なセクション情報
arm-none-eabi-objdump -h firmware.elf

# シンボルテーブル（変数・関数のアドレス）
arm-none-eabi-nm -S --size-sort firmware.elf | tail -20

# マップファイル生成（リンク時に -Map=output.map を指定）
# 最も詳細なメモリ配置情報を得られる
```

## Peripheral Basics

### GPIO (General Purpose Input/Output)

GPIOは最も基本的なペリフェラル。デジタル入出力を行う。

```c
/* STM32 HAL を使用した GPIO 制御 */
#include "stm32f4xx_hal.h"

/* 初期化（CubeMXで生成されることが多い） */
void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* クロック有効化（重要！） */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 を出力に設定（LED） */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  /* Push-Pull */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PC13 を入力に設定（ボタン） */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;          /* 内部プルアップ */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* 出力制御 */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);    /* HIGH */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  /* LOW */
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);                 /* トグル */

/* 入力読み取り */
GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
if (state == GPIO_PIN_RESET) {
    /* ボタンが押された（アクティブLOW） */
}
```

### レジスタ直接操作（低レベル制御）

```c
/* STM32F4 GPIO レジスタ直接操作 */

/* PA5 を出力に設定 */
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* クロック有効化 */
GPIOA->MODER &= ~(3U << (5 * 2));      /* クリア */
GPIOA->MODER |= (1U << (5 * 2));       /* 01: 出力モード */

/* PA5 を HIGH に設定 */
GPIOA->BSRR = (1U << 5);               /* Bit Set */

/* PA5 を LOW に設定 */
GPIOA->BSRR = (1U << (5 + 16));        /* Bit Reset (上位16bit) */

/* PA5 をトグル */
GPIOA->ODR ^= (1U << 5);
```

### Timer（タイマー）

タイマーは周期的な処理、PWM生成、入力キャプチャなどに使用。

```c
/* 基本タイマー設定（1ms周期割り込み） */
void Timer_Init(void) {
    TIM_HandleTypeDef htim2 = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1;       /* 84MHz / 84 = 1MHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1;        /* 1MHz / 1000 = 1kHz (1ms) */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    HAL_TIM_Base_Start_IT(&htim2);       /* 割り込み有効で開始 */
}

/* タイマー割り込みコールバック */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        /* 1ms毎に呼ばれる処理 */
        system_tick++;
    }
}
```

### PWM (Pulse Width Modulation)

```c
/* PWM出力設定（サーボモーター制御用、50Hz） */
void PWM_Init(void) {
    TIM_HandleTypeDef htim3 = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6 を TIM3_CH1 に設定（Alternate Function） */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* タイマー設定（50Hz = 20ms周期） */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 84 - 1;       /* 84MHz / 84 = 1MHz */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000 - 1;       /* 1MHz / 20000 = 50Hz */
    HAL_TIM_PWM_Init(&htim3);

    /* PWMチャンネル設定 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;              /* 1.5ms = 中央位置 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

/* デューティ比変更 */
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);  /* 1ms = -90° */
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);  /* 2ms = +90° */
```

### ADC (Analog-to-Digital Converter)

```c
/* ADC 初期化と読み取り */
ADC_HandleTypeDef hadc1;

void ADC_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA0 をアナログ入力に設定 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC設定 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;  /* 12bit: 0-4095 */
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    HAL_ADC_Init(&hadc1);

    /* チャンネル設定 */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* ADC 読み取り */
uint16_t ADC_Read(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

/* 電圧変換（3.3V基準、12bit） */
float voltage = (float)ADC_Read() * 3.3f / 4095.0f;
```

### DMA (Direct Memory Access)

DMAはCPUを介さずにメモリ間・ペリフェラル間でデータ転送を行う。

```c
/* DMAを使用したADC連続変換 */
#define ADC_BUFFER_SIZE 100

uint16_t adc_buffer[ADC_BUFFER_SIZE];
DMA_HandleTypeDef hdma_adc1;

void ADC_DMA_Init(void) {
    /* DMA設定 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;  /* 循環モード */
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* ADC連続変換モードで開始 */
    hadc1.Init.ContinuousConvMode = ENABLE;
    HAL_ADC_Init(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
}

/* DMA転送完了コールバック */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    /* adc_buffer にデータが格納される */
}
```

## Development Workflow

### ビルドシステム

#### CMake（推奨）

```cmake
# CMakeLists.txt for STM32
cmake_minimum_required(VERSION 3.20)

# ツールチェーン設定（クロスコンパイル）
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

project(firmware C CXX ASM)

# MCU固有の設定
set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS}")

# コンパイルオプション
add_compile_options(
    -Wall
    -fdata-sections
    -ffunction-sections
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-Os>
)

# リンカオプション
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407VGTx_FLASH.ld)
add_link_options(
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
)

# ソースファイル
file(GLOB_RECURSE SOURCES
    "Src/*.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)

# 実行ファイル生成
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# HEX/BIN生成
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
)
```

#### Makefile（シンプルなプロジェクト向け）

```makefile
# Makefile for ARM Cortex-M
TARGET = firmware
BUILD_DIR = build

# ツールチェーン
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
GDB = $(PREFIX)gdb

# MCU設定
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# ソースファイル
C_SOURCES = $(wildcard Src/*.c)
ASM_SOURCES = startup_stm32f407xx.s

# インクルードパス
C_INCLUDES = -IInc -IDrivers/CMSIS/Include

# コンパイルフラグ
CFLAGS = $(MCU) $(C_INCLUDES) -Wall -fdata-sections -ffunction-sections
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# デバッグ/リリース
ifdef DEBUG
CFLAGS += -g -gdwarf-2 -Og
else
CFLAGS += -Os
endif

# リンカスクリプト
LDSCRIPT = STM32F407VGTx_FLASH.ld
LDFLAGS = $(MCU) -T$(LDSCRIPT) --specs=nano.specs -Wl,--gc-sections

# オブジェクトファイル
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

# ビルドルール
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: Src/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

# フラッシュ書き込み
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $< 0x8000000

.PHONY: all clean flash
```

### クロスコンパイル環境構築

```bash
# Ubuntu/Debian
sudo apt install gcc-arm-none-eabi gdb-multiarch openocd

# macOS (Homebrew)
brew install --cask gcc-arm-embedded
brew install openocd

# Windows (Chocolatey)
choco install gcc-arm-embedded openocd

# バージョン確認
arm-none-eabi-gcc --version
openocd --version
```

### IDE / エディタ設定

#### VS Code + Cortex-Debug

```json
// .vscode/launch.json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Cortex Debug",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/firmware.elf",
            "request": "launch",
            "type": "cortex-debug",
            "runToEntryPoint": "main",
            "servertype": "openocd",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32f4x.cfg"
            ],
            "svdFile": "${workspaceFolder}/STM32F407.svd"
        }
    ]
}
```

```json
// .vscode/c_cpp_properties.json
{
    "configurations": [
        {
            "name": "ARM",
            "includePath": [
                "${workspaceFolder}/**",
                "${workspaceFolder}/Drivers/CMSIS/Include"
            ],
            "defines": [
                "STM32F407xx",
                "USE_HAL_DRIVER"
            ],
            "compilerPath": "/usr/bin/arm-none-eabi-gcc",
            "cStandard": "c11",
            "intelliSenseMode": "gcc-arm"
        }
    ]
}
```

## Debugging

### デバッグインターフェース比較

| インターフェース | ピン数 | 速度 | 機能 |
|----------------|-------|------|------|
| **JTAG** | 5+ (TMS, TCK, TDI, TDO, nTRST) | 低〜中 | フルデバッグ、境界スキャン |
| **SWD** | 2 (SWDIO, SWCLK) | 高速 | ARM専用、ピン数少 |
| **SWO** | 1 (追加) | - | トレース出力（printf） |

### デバッグプローブ

| プローブ | 対応 | 特徴 | 価格帯 |
|---------|------|------|--------|
| **ST-Link V2** | STM32 | 純正、安価 | $3-20 |
| **ST-Link V3** | STM32 | 高速、SWO対応 | $35 |
| **J-Link** | 汎用 | 高速、高機能、GDBサーバー | $50-600+ |
| **J-Link EDU** | 汎用 | 教育用、制限あり | $20 |
| **CMSIS-DAP** | 汎用 | オープン規格 | $10-30 |
| **Black Magic Probe** | 汎用 | GDB内蔵、オープンソース | $60 |

### OpenOCD 基本操作

```bash
# OpenOCD 起動（ST-Link + STM32F4）
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# 別ターミナルから telnet 接続
telnet localhost 4444

# OpenOCD コマンド
> reset halt          # リセット＆停止
> flash write_image erase firmware.elf  # 書き込み
> reset run           # リセット＆実行
> exit

# ワンライナーでフラッシュ書き込み
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program firmware.elf verify reset exit"
```

### GDB デバッグ

```bash
# GDB 起動
arm-none-eabi-gdb firmware.elf

# GDB コマンド
(gdb) target remote localhost:3333    # OpenOCD に接続
(gdb) monitor reset halt              # リセット
(gdb) load                            # プログラム転送
(gdb) break main                      # ブレークポイント設定
(gdb) continue                        # 実行
(gdb) next                            # ステップオーバー
(gdb) step                            # ステップイン
(gdb) print variable_name             # 変数表示
(gdb) info registers                  # レジスタ表示
(gdb) x/16xw 0x20000000               # メモリダンプ
(gdb) backtrace                       # コールスタック
(gdb) monitor reset run               # リセット＆実行
```

### printf デバッグ（Semihosting / SWO）

#### Semihosting

```c
/* Semihosting を使用した printf */
/* リンカオプション: --specs=rdimon.specs */
#include <stdio.h>

extern void initialise_monitor_handles(void);

int main(void) {
    initialise_monitor_handles();
    printf("Hello from MCU!\n");
    // ...
}
```

```bash
# OpenOCD で semihosting 有効化
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init; arm semihosting enable"
```

#### SWO (Serial Wire Output)

```c
/* SWO を使用した ITM printf */
/* より高速、実行への影響が少ない */

#include <stdio.h>

/* ITM送信関数 */
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

/* または直接 ITM を使用 */
void ITM_Print(const char *str) {
    while (*str) {
        ITM_SendChar(*str++);
    }
}
```

### ハードフォルト解析

```c
/* HardFault ハンドラでデバッグ情報を取得 */
void HardFault_Handler(void) {
    __asm volatile (
        " tst lr, #4                         \n"
        " ite eq                             \n"
        " mrseq r0, msp                      \n"
        " mrsne r0, psp                      \n"
        " ldr r1, [r0, #24]                  \n"  /* PC */
        " ldr r2, handler2_address_const     \n"
        " bx r2                              \n"
        " handler2_address_const: .word prvGetRegistersFromStack \n"
    );
}

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress) {
    volatile uint32_t r0  = pulFaultStackAddress[0];
    volatile uint32_t r1  = pulFaultStackAddress[1];
    volatile uint32_t r2  = pulFaultStackAddress[2];
    volatile uint32_t r3  = pulFaultStackAddress[3];
    volatile uint32_t r12 = pulFaultStackAddress[4];
    volatile uint32_t lr  = pulFaultStackAddress[5];  /* Link Register */
    volatile uint32_t pc  = pulFaultStackAddress[6];  /* Program Counter */
    volatile uint32_t psr = pulFaultStackAddress[7];

    /* ここにブレークポイントを設置 */
    /* pc の値からフォルト発生箇所を特定 */
    for(;;);
}
```

## Common Patterns

### 割り込み（Interrupt）

```c
/* 割り込み優先度の設定 */
/* 数値が小さいほど優先度が高い */
HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);  /* 最高優先度 */
HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* 外部割り込み（EXTI）の例 */
void EXTI0_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
        /* 割り込み処理（短く保つ） */
        button_pressed = 1;  /* フラグを立てるだけ */
    }
}

/* メインループで処理 */
while (1) {
    if (button_pressed) {
        button_pressed = 0;
        handle_button_press();  /* 重い処理はここで */
    }
}
```

### 割り込みセーフなコード

```c
/* クリティカルセクション */
__disable_irq();  /* 割り込み禁止 */
/* 共有リソースへのアクセス */
shared_variable++;
__enable_irq();   /* 割り込み許可 */

/* FreeRTOS の場合 */
taskENTER_CRITICAL();
shared_variable++;
taskEXIT_CRITICAL();

/* volatile キーワード */
volatile uint32_t tick_count;  /* 割り込みで変更される変数 */
```

### ステートマシン

```c
/* 基本的なステートマシン実装 */
typedef enum {
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR
} State_t;

typedef enum {
    EVENT_START,
    EVENT_STOP,
    EVENT_ERROR,
    EVENT_NONE
} Event_t;

State_t current_state = STATE_IDLE;

/* テーブル駆動型ステートマシン */
typedef void (*StateHandler)(Event_t event);

typedef struct {
    State_t state;
    Event_t event;
    State_t next_state;
    StateHandler action;
} Transition_t;

void action_start(Event_t e) { /* 開始処理 */ }
void action_stop(Event_t e)  { /* 停止処理 */ }
void action_error(Event_t e) { /* エラー処理 */ }

const Transition_t transitions[] = {
    { STATE_IDLE,    EVENT_START, STATE_RUNNING, action_start },
    { STATE_RUNNING, EVENT_STOP,  STATE_IDLE,    action_stop  },
    { STATE_RUNNING, EVENT_ERROR, STATE_ERROR,   action_error },
    { STATE_ERROR,   EVENT_STOP,  STATE_IDLE,    action_stop  },
};

void state_machine_run(Event_t event) {
    for (int i = 0; i < sizeof(transitions)/sizeof(transitions[0]); i++) {
        if (transitions[i].state == current_state &&
            transitions[i].event == event) {
            if (transitions[i].action) {
                transitions[i].action(event);
            }
            current_state = transitions[i].next_state;
            return;
        }
    }
}
```

### 低消費電力モード

```c
/* STM32 スリープモード */

/* Sleep モード（WFI命令で復帰） */
HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

/* Stop モード（より深い省電力） */
HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

/* Standby モード（最も省電力、RAM保持なし） */
HAL_PWR_EnterSTANDBYMode();

/* 低消費電力のベストプラクティス */
void enter_low_power(void) {
    /* 1. 未使用ペリフェラルのクロック停止 */
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();

    /* 2. 未使用ピンをアナログモードに */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 3. 割り込みで起床するよう設定 */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    /* 4. スリープに入る */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    /* 5. 起床後の処理 */
    SystemClock_Config();  /* クロック再設定が必要な場合 */
}
```

### リングバッファ

```c
/* 割り込みセーフなリングバッファ */
#define BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer_t;

RingBuffer_t rx_buffer = {0};

/* データ追加（割り込みハンドラから） */
int ringbuffer_put(RingBuffer_t *rb, uint8_t data) {
    uint32_t next_head = (rb->head + 1) % BUFFER_SIZE;
    if (next_head == rb->tail) {
        return -1;  /* バッファフル */
    }
    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return 0;
}

/* データ取得（メインループから） */
int ringbuffer_get(RingBuffer_t *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        return -1;  /* バッファ空 */
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    return 0;
}

/* UART受信割り込みでの使用例 */
void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t data = USART1->DR;
        ringbuffer_put(&rx_buffer, data);
    }
}
```

## Communication Protocols Overview

### UART (Universal Asynchronous Receiver/Transmitter)

```c
/* UART 初期化 */
UART_HandleTypeDef huart1;

void UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart1);
}

/* 送受信 */
HAL_UART_Transmit(&huart1, (uint8_t*)"Hello\n", 6, HAL_MAX_DELAY);

uint8_t rx_data[10];
HAL_UART_Receive(&huart1, rx_data, 10, 1000);  /* 1秒タイムアウト */

/* 割り込み受信 */
HAL_UART_Receive_IT(&huart1, rx_data, 1);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        /* 受信完了処理 */
        HAL_UART_Receive_IT(&huart1, rx_data, 1);  /* 再開 */
    }
}
```

### I2C (Inter-Integrated Circuit)

```c
/* I2C 初期化 */
I2C_HandleTypeDef hi2c1;

void I2C_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;  /* 400kHz Fast Mode */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c1);
}

/* レジスタ読み書き（センサ読み取りの典型パターン） */
#define SENSOR_ADDR 0x48

/* レジスタ書き込み */
uint8_t write_data[2] = {0x01, 0x60};  /* レジスタアドレス + データ */
HAL_I2C_Master_Transmit(&hi2c1, SENSOR_ADDR << 1, write_data, 2, HAL_MAX_DELAY);

/* レジスタ読み取り */
uint8_t reg = 0x00;
uint8_t read_data[2];
HAL_I2C_Master_Transmit(&hi2c1, SENSOR_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1, SENSOR_ADDR << 1, read_data, 2, HAL_MAX_DELAY);

/* または Mem系関数を使用（より簡潔） */
HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR << 1, 0x00, I2C_MEMADD_SIZE_8BIT,
                 read_data, 2, HAL_MAX_DELAY);
```

### SPI (Serial Peripheral Interface)

```c
/* SPI 初期化 */
SPI_HandleTypeDef hspi1;

void SPI_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;   /* CPOL = 0 */
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;       /* CPHA = 0 */
    hspi1.Init.NSS = SPI_NSS_SOFT;               /* ソフトウェアCS制御 */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi1);
}

/* SPI 通信（CS制御含む） */
void SPI_Transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  /* CS Low */
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);    /* CS High */
}
```

### CAN (Controller Area Network)

```c
/* CAN 初期化 */
CAN_HandleTypeDef hcan1;

void CAN_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;        /* 84MHz / 6 = 14MHz */
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    /* ビットレート = 14MHz / (1 + 11 + 2) = 1Mbps */
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan1);

    /* 受信フィルタ設定（全メッセージ受信） */
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* CAN メッセージ送信 */
void CAN_Transmit(uint32_t id, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

/* CAN 受信コールバック */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    /* 受信処理 */
}
```

### プロトコル選択ガイド

```
┌─────────────────────────────────────────────────────────────┐
│                    通信プロトコル選択                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ボード内通信？                                              │
│  ├─ Yes                                                      │
│  │   ├─ 高速データ転送（>1Mbps）？                          │
│  │   │   └─ Yes → SPI                                       │
│  │   ├─ 複数デバイス接続？                                  │
│  │   │   └─ Yes → I2C                                       │
│  │   └─ シンプル・デバッグ用？                              │
│  │       └─ Yes → UART                                      │
│  └─ No (長距離・ノイズ環境)                                 │
│      ├─ 車載・産業用？                                      │
│      │   └─ Yes → CAN Bus                                   │
│      ├─ 既存PLC接続？                                        │
│      │   └─ Yes → RS485/Modbus                              │
│      └─ センサネットワーク？                                │
│          └─ Yes → 1-Wire                                    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Quick Reference

### 開発フロー

```bash
# 1. プロジェクト生成（STM32の場合）
# STM32CubeMX でプロジェクト生成

# 2. ビルド
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

# 3. フラッシュ書き込み
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program firmware.elf verify reset exit"

# 4. デバッグ
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg &
arm-none-eabi-gdb firmware.elf -ex "target remote :3333"
```

### よく使うコマンド

| コマンド | 説明 |
|---------|------|
| `arm-none-eabi-gcc` | クロスコンパイラ |
| `arm-none-eabi-gdb` | デバッガ |
| `arm-none-eabi-objcopy` | バイナリ変換 |
| `arm-none-eabi-objdump` | 逆アセンブル |
| `arm-none-eabi-size` | セクションサイズ |
| `arm-none-eabi-nm` | シンボルテーブル |
| `openocd` | デバッグサーバー |
| `st-flash` | ST-Link書き込み |

### トラブルシューティング

| 症状 | 原因 | 対策 |
|------|------|------|
| プログラムが動かない | クロック未設定 | SystemInit() 確認 |
| ペリフェラルが動かない | クロック未有効化 | RCC_xxxENABLE() 確認 |
| HardFault発生 | スタックオーバーフロー | スタックサイズ増加 |
| HardFault発生 | NULL参照 | ポインタ確認 |
| 割り込みが来ない | NVIC未設定 | EnableIRQ() 確認 |
| I2Cが動かない | プルアップ抵抗なし | 外付け4.7kΩ追加 |

## References

- [ARM Cortex-M Programming Guide](https://developer.arm.com/documentation/)
- [STM32 Reference Manual](https://www.st.com/content/st_com/en/support/resources.html)
- [Embedded Artistry](https://embeddedartistry.com/)
- [Memfault Blog](https://memfault.com/blog/)
- [Interrupt Blog](https://interrupt.memfault.com/)
