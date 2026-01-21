---
name: stm32
description: Use when developing firmware for STM32 devices. Covers STM32CubeIDE, HAL/LL drivers, CubeMX, and debugging.
tags:
  - stm32
  - arm
  - cortex-m
  - hal
  - cubemx
---

# STM32 Firmware Development Guide

STM32マイコン向けファームウェア開発の包括的ガイド。

## 1. Device Overview - STM32シリーズ比較

### STM32 Series Comparison Table

| Series | Core | Max Clock | Process | RAM | Flash | Primary Use Case |
|--------|------|-----------|---------|-----|-------|------------------|
| **F4** | Cortex-M4F | 180 MHz | 90nm | 192KB + 64KB CCM | 512KB-2MB | General purpose, DSP, FPU |
| **G4** | Cortex-M4F | 170 MHz | 90nm | 32-128KB | 128-512KB | Analog processing, motor control |
| **H7** | Cortex-M7F (+M4F dual) | 480 MHz | 40nm | 1MB+ | 1-2MB | High performance, dual-core |
| **U5** | Cortex-M33 | 160 MHz | 40nm | 786KB | 2MB | Ultra-low power, TrustZone |
| **WB** | Cortex-M4F + M0+ | 64 MHz | - | 256KB | 1MB | BLE, Zigbee, Thread |
| **WL** | Cortex-M4 + M0+ | 48 MHz | - | 64KB | 256KB | LoRa, LoRaWAN, Sub-GHz |

### 各シリーズの特徴

#### STM32F4 Series
- 最も普及したシリーズ。豊富なライブラリとコミュニティサポート
- DSPとFPU命令をサポートした最初のSTM32シリーズ
- 用途: ドローンFC、産業機器、汎用組込み

```c
// F4の特徴的なCCM-SRAM使用例
__attribute__((section(".ccmram"))) uint32_t fast_buffer[1024];
```

#### STM32G4 Series
- F3/F4の後継として2019年登場
- アナログペリフェラルが充実（高分解能タイマー、DAC、コンパレータ）
- 用途: モーター制御、電源管理、デジタル電源

#### STM32H7 Series
- 最高性能シリーズ（40nmプロセス）
- 倍精度FPU、L1キャッシュ搭載
- オプションでCortex-M4Fとのデュアルコア構成
- 用途: グラフィックス、産業用HMI、高性能制御

#### STM32U5 Series
- 超低消費電力シリーズ
- Cortex-M33 with TrustZone（セキュリティ機能）
- 用途: IoTセンサー、ウェアラブル、バッテリー駆動機器

#### STM32WB Series
- マルチプロトコルワイヤレス（BLE 5.4, Zigbee, Thread, Matter）
- デュアルコア（M4F + M0+専用無線スタック）
- 用途: スマートホーム、産業IoT、ヘルスケア

#### STM32WL Series
- LoRa/LoRaWAN対応のSoC
- Sub-GHz無線内蔵
- 用途: LPWAN、スマートメーター、農業IoT

---

## 2. STM32CubeIDE Setup

### Installation

**最新バージョン: STM32CubeIDE 2.0.0** (2025年11月リリース)

主な変更点:
- STM32CubeMXが**分離**され、スタンドアロンツールに
- Eclipse 2024-09ベース（Copilot4Eclipseプラグイン対応）
- ST LLVM-basedツールチェーンのGUI統合
- ログイン要件の撤廃

```bash
# Linux: 必要な依存関係
sudo apt-get install libncurses5 libusb-1.0-0

# macOS: Homebrew経由でST-Linkドライバ
brew install stlink

# ダウンロード
# https://www.st.com/en/development-tools/stm32cubeide.html
```

### VS Code Extension (Alternative)

STMicroelectronicsは公式VS Code拡張も提供:

```bash
# VS Code Marketplace
code --install-extension stmicroelectronics.stm32-vscode-extension
```

特徴:
- GCC-14がデフォルトツールチェーン
- クイックリセットボタン（デバッグ時）
- macOS Apple Silicon対応（AArch64ネイティブ）

### Project Creation

```bash
# CubeMXでプロジェクト生成後、CubeIDEにインポート
# Project Structure (推奨):
MyProject/
├── Core/
│   ├── Inc/          # User headers
│   ├── Src/          # User sources
│   └── Startup/      # Startup assembly
├── Drivers/
│   ├── CMSIS/
│   └── STM32F4xx_HAL_Driver/
├── Middlewares/      # FreeRTOS, USB, etc.
├── .ioc              # CubeMX configuration
└── STM32F4xx.ld      # Linker script
```

---

## 3. STM32CubeMX - Code Generation

### Pinout Configuration

CubeMXでのピン設定のベストプラクティス:

1. **ピンラベル付け**: わかりやすい名前を設定
   ```
   PA0 -> "USER_BTN"
   PC13 -> "LED_STATUS"
   ```

2. **クロック設定**: Clock Configurationタブで最適化
   ```
   HSE: 8MHz Crystal
   PLL: /M=8, *N=336, /P=2 -> SYSCLK=168MHz (F4)
   ```

### Code Generation Settings

Project Manager > Code Generator で設定:

```
[x] Generate peripheral initialization as pair of '.c/.h' files
[x] Keep User Code when re-generating
[x] Copy only the necessary library files
```

### User Code Blocks

自動生成コードにカスタムコードを追加:

```c
/* USER CODE BEGIN Includes */
#include "my_module.h"
/* USER CODE END Includes */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  my_init();  // カスタム初期化
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    my_loop();  // カスタムメインループ
    /* USER CODE END 3 */
  }
}
```

### C++プロジェクトのワークアラウンド

CubeMXはC言語のみ生成するため、C++を使う場合:

```c
// main.c (generated)
/* USER CODE BEGIN PFP */
extern void alt_main(void);  // C++ entry point
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  alt_main();  // Never returns
  /* USER CODE END 2 */
}
```

```cpp
// alt_main.cpp
extern "C" void alt_main(void) {
    MyApplication app;
    app.run();  // C++ application entry
}
```

---

## 4. HAL vs LL Drivers

### 比較表

| 特性 | HAL | LL |
|------|-----|-----|
| 抽象度 | 高い | 低い（レジスタに近い） |
| コードサイズ | 大きい | 小さい |
| 実行速度 | 遅い | 速い |
| 移植性 | 高い（90%以上互換） | 低い |
| 学習コスト | 低い | 高い |
| SysTick依存 | あり | なし |
| DMA/割り込み | 組み込み済み | 自分で実装 |

### When to Use HAL

- プロトタイピング・学習目的
- MCU間の移植性が重要
- 開発速度優先
- リソースに余裕がある

```c
// HAL: GPIO Toggle (高レベルAPI)
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

// HAL: UART送信 (DMA対応)
HAL_UART_Transmit_DMA(&huart2, data, len);

// HAL: ADC読み取り
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, 100);
uint32_t value = HAL_ADC_GetValue(&hadc1);
```

### When to Use LL

- 性能・消費電力が重要
- コードサイズ制約がある
- リアルタイム制御（モーター、電源）
- ハードウェアの深い理解がある

```c
// LL: GPIO Toggle (レジスタ直接アクセス)
LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);

// LL: UART送信 (ポーリング)
while (!LL_USART_IsActiveFlag_TXE(USART2));
LL_USART_TransmitData8(USART2, byte);

// LL: ADC読み取り
LL_ADC_REG_StartConversion(ADC1);
while (!LL_ADC_IsActiveFlag_EOC(ADC1));
uint32_t value = LL_ADC_REG_ReadConversionData12(ADC1);
```

### Mixed Approach (推奨)

同一ペリフェラルでなければHALとLLの混在可能:

```c
// CubeMXで設定:
// GPIO: LL (高速トグル用)
// UART: HAL (DMA使用)
// SPI: LL (性能重視)
// I2C: HAL (プロトコル複雑)
```

---

## 5. Peripherals

### GPIO

```c
// 出力設定 (HAL)
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_13;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// 入力割り込み (EXTI)
GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
HAL_NVIC_EnableIRQ(EXTI0_IRQn);

// 割り込みコールバック
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        // ボタン押下処理
    }
}
```

### Timer (TIM)

```c
// PWM出力 (1kHz, 50%デューティ)
// CubeMXでTIM設定後:
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);  // 50%

// タイマー割り込み (1ms周期)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        system_tick++;
    }
}

// 入力キャプチャ (周波数測定)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        frequency = HAL_RCC_GetPCLK1Freq() / (htim->Init.Prescaler + 1) / capture;
    }
}
```

### ADC with DMA

```c
// 複数チャンネル連続変換
#define ADC_CHANNELS 4
uint16_t adc_values[ADC_CHANNELS];

// 初期化後に開始
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_CHANNELS);

// 変換完了コールバック
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // adc_values[] に最新値
    process_adc_data(adc_values);
}
```

### I2C

```c
// センサー読み取り (ブロッキング)
uint8_t data[6];
HAL_I2C_Mem_Read(&hi2c1,
    MPU6050_ADDR << 1,  // 7-bit address
    ACCEL_XOUT_H,       // Register
    I2C_MEMADD_SIZE_8BIT,
    data, 6,
    HAL_MAX_DELAY);

// ノンブロッキング (DMA)
HAL_I2C_Mem_Read_DMA(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len);

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        sensor_data_ready = 1;
    }
}
```

### SPI

```c
// フルデュプレックス通信
uint8_t tx_data[] = {0x80 | REG_ADDR, 0x00};  // Read command
uint8_t rx_data[2];

HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

// DMA使用 (大量データ転送)
HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, len);
```

### UART with Ring Buffer

```c
// 受信リングバッファ実装
#define RX_BUF_SIZE 256
uint8_t rx_buffer[RX_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

// IDLE Line Detection (可変長受信)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart2) {
        rx_head = Size;
        // データ処理
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUF_SIZE);
    }
}

// 初期化
HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_BUF_SIZE);
__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);  // Half-transfer割り込み無効
```

### USB (CDC Virtual COM Port)

```c
// CubeMXでUSB_DEVICE + CDC Class設定後:

// 送信
CDC_Transmit_FS((uint8_t*)message, strlen(message));

// 受信コールバック (usbd_cdc_if.c)
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
    // 受信データ処理
    process_usb_data(Buf, *Len);

    // 次の受信準備
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}
```

---

## 6. Debugging

### ST-Link / SWD

```c
// SWDピン構成
// SWDIO: PA13
// SWCLK: PA14
// SWO:   PB3 (オプション、トレース用)
// NRST:  リセットピン

// デバッグビルド設定
// -Og (最適化控えめ)
// -g3 (フルデバッグ情報)
```

### ITM Trace (SWO)

printf相当のデバッグ出力:

```c
// syscalls.c または別ファイル
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

// 使用例
printf("ADC value: %lu\n", adc_value);
```

CubeIDEでSWV ITM Data Console設定:
- Core Clock: システムクロック周波数
- SWO Clock: 2000000 (2MHz推奨)
- Port 0: Enable

### Hard Fault Analysis

```c
// ハードフォルトハンドラ改良版
void HardFault_Handler(void) {
    __asm volatile (
        "tst lr, #4                     \n"
        "ite eq                         \n"
        "mrseq r0, msp                  \n"
        "mrsne r0, psp                  \n"
        "b hard_fault_handler_c         \n"
    );
}

void hard_fault_handler_c(uint32_t *hardfault_args) {
    volatile uint32_t stacked_r0 = hardfault_args[0];
    volatile uint32_t stacked_r1 = hardfault_args[1];
    volatile uint32_t stacked_r2 = hardfault_args[2];
    volatile uint32_t stacked_r3 = hardfault_args[3];
    volatile uint32_t stacked_r12 = hardfault_args[4];
    volatile uint32_t stacked_lr = hardfault_args[5];
    volatile uint32_t stacked_pc = hardfault_args[6];  // フォルト発生アドレス
    volatile uint32_t stacked_psr = hardfault_args[7];

    volatile uint32_t cfsr = SCB->CFSR;   // Configurable Fault Status
    volatile uint32_t hfsr = SCB->HFSR;   // Hard Fault Status
    volatile uint32_t dfsr = SCB->DFSR;   // Debug Fault Status
    volatile uint32_t afsr = SCB->AFSR;   // Auxiliary Fault Status
    volatile uint32_t bfar = SCB->BFAR;   // Bus Fault Address
    volatile uint32_t mmfar = SCB->MMFAR; // MemManage Fault Address

    // ブレークポイントを設定してデバッグ
    __BKPT(0);
    while(1);
}
```

### Watchdog (IWDG)

```c
// Independent Watchdog設定
// CubeMXで IWDG有効化、プリスケーラとリロード値設定

// メインループでリフレッシュ
while (1) {
    // 通常処理
    process_tasks();

    // ウォッチドッグリフレッシュ
    HAL_IWDG_Refresh(&hiwdg);
}

// タイムアウト計算: T = (Prescaler * Reload) / LSI_freq
// 例: Prescaler=64, Reload=4095, LSI=32kHz → ~8秒
```

---

## 7. RTOS Integration

### FreeRTOS

CubeMXで Middleware > RTOS > FreeRTOS 選択:

```c
// タスク作成
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);

// タスク実装
void SensorTask(void *argument) {
    for (;;) {
        read_sensors();
        osDelay(100);  // 100ms周期
    }
}

// キュー
osMessageQueueId_t sensorQueueHandle;
sensorQueueHandle = osMessageQueueNew(16, sizeof(SensorData_t), NULL);

// 送信側
SensorData_t data = {.temperature = 25.5f};
osMessageQueuePut(sensorQueueHandle, &data, 0, 0);

// 受信側
SensorData_t received;
if (osMessageQueueGet(sensorQueueHandle, &received, NULL, osWaitForever) == osOK) {
    process_data(&received);
}

// セマフォ
osSemaphoreId_t uartSemHandle;
uartSemHandle = osSemaphoreNew(1, 1, NULL);

osSemaphoreAcquire(uartSemHandle, osWaitForever);
// クリティカルセクション
osSemaphoreRelease(uartSemHandle);
```

### ThreadX (Azure RTOS)

STM32U5/H7で推奨:

```c
// スレッド作成
TX_THREAD sensor_thread;
CHAR sensor_stack[1024];

tx_thread_create(&sensor_thread, "Sensor Thread",
    sensor_thread_entry, 0,
    sensor_stack, sizeof(sensor_stack),
    10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

void sensor_thread_entry(ULONG initial_input) {
    while (1) {
        read_sensors();
        tx_thread_sleep(100);  // 100 ticks
    }
}

// イベントフラグ
TX_EVENT_FLAGS_GROUP sensor_events;
tx_event_flags_create(&sensor_events, "Sensor Events");

// フラグセット
tx_event_flags_set(&sensor_events, 0x01, TX_OR);

// フラグ待機
ULONG actual_flags;
tx_event_flags_get(&sensor_events, 0x01, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
```

### Low Power with RTOS

```c
// FreeRTOS Tickless Idle (低消費電力)
// FreeRTOSConfig.h
#define configUSE_TICKLESS_IDLE 2  // ST Low Power Timer使用

// ThreadX Low Power
void tx_low_power_enter(void) {
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void tx_low_power_exit(void) {
    // Wake-up処理
}
```

---

## 8. Advanced Topics

### TouchGFX (GUI Development)

TouchGFXの構成:
- **TouchGFX Designer**: WYSIWYG GUIビルダー
- **TouchGFX Generator**: CubeMXプラグイン
- **TouchGFX Engine**: C++フレームワーク

最新版: TouchGFX 4.24.0 (2024年6月)

```cpp
// Screen implementation
class MainView : public MainViewBase {
public:
    MainView() {}
    virtual ~MainView() {}

    virtual void setupScreen() override {
        // 画面初期化
    }

    virtual void handleTickEvent() override {
        // フレームごとの更新 (60fps)
        updateGraph();
    }

    // ボタンコールバック
    virtual void buttonClicked() override {
        application().gotoSettingsScreen();
    }

private:
    void updateGraph() {
        // グラフ更新ロジック
    }
};

// Hardware interaction (Model.cpp)
void Model::tick() {
    // HAL/LLとの連携
    extern volatile uint16_t adc_value;
    modelListener->updateSensorValue(adc_value);
}
```

TouchGFXの特徴:
- 30種類以上のウィジェット（スワイプ、スクロール、3Dエフェクト）
- Chrom-ARTアクセラレータ対応（H7, U5等）
- アルファブレンディング、アンチエイリアスフォント
- ビデオ再生対応

### STM32Cube.AI (Edge ML)

STM32Cube.AIでニューラルネットワークをMCUにデプロイ:

```c
// 対応フォーマット
// - TensorFlow Lite (.tflite)
// - ONNX (.onnx)
// - Keras (.h5, .keras)
// - QKeras (量子化)

// AI推論の基本フロー
#include "network.h"
#include "network_data.h"

ai_handle network = AI_HANDLE_NULL;
ai_buffer ai_input[1];
ai_buffer ai_output[1];

// 初期化
ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
ai_network_init(network);

// 推論実行
ai_input[0].data = AI_HANDLE_PTR(input_data);
ai_output[0].data = AI_HANDLE_PTR(output_data);

ai_network_run(network, &ai_input[0], &ai_output[0]);

// 結果解釈
int prediction = argmax(output_data, AI_NETWORK_OUT_1_SIZE);
```

X-CUBE-AIの特徴:
- 8-bit量子化対応（メモリ削減、速度向上）
- ST Edge AI Developer Cloud（オンラインツール）
- Edge Impulse連携
- STM32 AI Model Zoo（事前学習モデル集）

### Motor Control (FOC)

STM32 Motor Control Workbench:

```c
// FOC (Field Oriented Control) 基本構造
typedef struct {
    int16_t Iq;       // q軸電流
    int16_t Id;       // d軸電流
    int16_t Vq;       // q軸電圧
    int16_t Vd;       // d軸電圧
    int16_t ElAngle;  // 電気角
} FOC_t;

// Park変換 (αβ → dq)
void Park(int16_t Ialpha, int16_t Ibeta, int16_t angle,
          int16_t *Id, int16_t *Iq) {
    int32_t cos_val = cos_table[angle];
    int32_t sin_val = sin_table[angle];

    *Id = (Ialpha * cos_val + Ibeta * sin_val) >> 15;
    *Iq = (-Ialpha * sin_val + Ibeta * cos_val) >> 15;
}

// 電流ループPID
int16_t PI_Controller(PI_t *pi, int16_t error) {
    pi->integral += (int32_t)error * pi->Ki;

    // Anti-windup
    if (pi->integral > pi->max_integral) pi->integral = pi->max_integral;
    if (pi->integral < -pi->max_integral) pi->integral = -pi->max_integral;

    int32_t output = (int32_t)error * pi->Kp + pi->integral;

    // 出力制限
    if (output > pi->max_output) output = pi->max_output;
    if (output < -pi->max_output) output = -pi->max_output;

    return (int16_t)output;
}
```

### Power Management

```c
// Sleep Mode
HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

// Stop Mode (より低消費電力)
HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

// Standby Mode (最低消費電力、RAM内容消失)
HAL_PWR_EnterSTANDBYMode();

// Wake-up設定 (RTC)
HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x1000, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

// STOP Mode復帰後のクロック再設定
void SystemClock_Config_AfterStop(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    // HSE再有効化、PLL再設定...
}
```

---

## Quick Reference

### ビルド設定

```makefile
# 最適化フラグ
# -O0: デバッグ用
# -Og: デバッグ + 軽い最適化
# -O2: リリース用
# -Os: サイズ最適化
# -O3: 速度最優先

# LTO (Link Time Optimization)
CFLAGS += -flto
LDFLAGS += -flto

# FPU設定 (F4/G4/H7)
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
```

### デバッグコマンド (GDB)

```gdb
# ブレークポイント
break main.c:100
break HAL_GPIO_TogglePin

# ウォッチポイント (変数監視)
watch my_variable
rwatch my_variable  # 読み取り時
awatch my_variable  # 読み書き時

# メモリダンプ
x/16xw 0x20000000  # 16ワード表示

# レジスタ表示
info registers
print $pc

# フォルトレジスタ
print/x *(uint32_t*)0xE000ED28  # CFSR
print/x *(uint32_t*)0xE000ED2C  # HFSR
```

### 便利なマクロ

```c
// ビットバンド (Cortex-M3/M4)
#define BITBAND_SRAM(addr, bit) \
    (*(volatile uint32_t*)(0x22000000 + ((addr) - 0x20000000) * 32 + (bit) * 4))

#define BITBAND_PERIPH(addr, bit) \
    (*(volatile uint32_t*)(0x42000000 + ((addr) - 0x40000000) * 32 + (bit) * 4))

// GPIO高速アクセス
#define LED_ON()  (GPIOC->BSRR = GPIO_PIN_13)
#define LED_OFF() (GPIOC->BSRR = GPIO_PIN_13 << 16)

// 配列サイズ
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// コンパイル時アサート
#define STATIC_ASSERT(cond) typedef char static_assert_##__LINE__[(cond) ? 1 : -1]
```

---

## References

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [TouchGFX Documentation](https://support.touchgfx.com/docs/introduction/welcome)
- [X-CUBE-AI](https://www.st.com/en/embedded-software/x-cube-ai.html)
- [ST Community](https://community.st.com/)
- [STM32 MCU Wiki](https://wiki.st.com/stm32mcu/)
