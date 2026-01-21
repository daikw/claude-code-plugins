---
name: freertos
description: Use when developing with FreeRTOS. Covers task management, synchronization primitives, and memory management.
tags:
  - freertos
  - rtos
  - embedded
  - real-time
---

# FreeRTOS Development Skill

## 1. Overview

### FreeRTOS とは

FreeRTOS は、組み込みシステム向けのオープンソースリアルタイムオペレーティングシステム（RTOS）カーネル。MIT ライセンスで提供されており、商用利用も可能。

### Latest Versions (2024-2025)

| バージョン | 説明 |
|-----------|------|
| **FreeRTOS Kernel V11.2.0** | 最新カーネル。SMP、MPU サポート強化 |
| **FreeRTOS 202406 LTS** | 長期サポート版。Kernel v11.1 + TCP v4.2.1 |
| **FreeRTOS 202411.00** | メインディストリビューション（2024年11月リリース） |

### Key Features

- **Preemptive / Cooperative Scheduling**: タスクスケジューリング方式の選択
- **Task Notifications**: 軽量な同期プリミティブ
- **Software Timers**: ソフトウェアタイマー
- **Queue / Semaphore / Mutex**: タスク間通信・同期
- **Memory Management**: 5種類のヒープ実装
- **SMP Support (V11+)**: マルチコア対応
- **MPU Support**: メモリ保護ユニット対応

### Supported Architectures (主要ポート)

- ARM Cortex-M (M0, M3, M4, M7, M23, M33, M55)
- ARM Cortex-A / Cortex-R
- RISC-V
- Xtensa (ESP32)
- x86
- その他多数

---

## 2. Task Management

### xTaskCreate - タスク作成

```c
#include "FreeRTOS.h"
#include "task.h"

// タスク関数のプロトタイプ
void vTaskFunction(void *pvParameters);

// タスク作成
BaseType_t xReturned;
TaskHandle_t xHandle = NULL;

xReturned = xTaskCreate(
    vTaskFunction,       // タスク関数
    "TaskName",          // タスク名（デバッグ用）
    configMINIMAL_STACK_SIZE,  // スタックサイズ（ワード単位）
    (void *)1,           // パラメータ
    tskIDLE_PRIORITY + 1,// 優先度
    &xHandle             // タスクハンドル
);

if (xReturned == pdPASS) {
    // タスク作成成功
}

// タスク関数の実装
void vTaskFunction(void *pvParameters) {
    int param = (int)pvParameters;

    for (;;) {
        // タスク処理
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 待機
    }

    // タスクは通常終了しない。終了する場合:
    // vTaskDelete(NULL);
}
```

### Static Allocation - 静的メモリ割り当て

```c
// FreeRTOSConfig.h で設定
// #define configSUPPORT_STATIC_ALLOCATION 1

static StaticTask_t xTaskBuffer;
static StackType_t xStack[configMINIMAL_STACK_SIZE];

TaskHandle_t xHandle = xTaskCreateStatic(
    vTaskFunction,
    "StaticTask",
    configMINIMAL_STACK_SIZE,
    NULL,
    tskIDLE_PRIORITY + 1,
    xStack,
    &xTaskBuffer
);
```

### Task Priorities

```c
// 優先度は 0 から (configMAX_PRIORITIES - 1) まで
// 数値が大きいほど優先度が高い

#define PRIORITY_LOW      (tskIDLE_PRIORITY + 1)
#define PRIORITY_NORMAL   (tskIDLE_PRIORITY + 2)
#define PRIORITY_HIGH     (tskIDLE_PRIORITY + 3)

// 実行時の優先度変更
vTaskPrioritySet(xHandle, PRIORITY_HIGH);

// 現在の優先度取得
UBaseType_t uxPriority = uxTaskPriorityGet(xHandle);
```

### Task Notifications - タスク通知

タスク通知は、セマフォやイベントグループよりも軽量・高速な同期機構。

```c
// 送信側（バイナリセマフォ的な使い方）
xTaskNotifyGive(xTaskToNotify);

// 受信側
uint32_t ulNotificationValue;
ulNotificationValue = ulTaskNotifyTake(
    pdTRUE,                    // カウンタをクリア
    pdMS_TO_TICKS(100)         // タイムアウト
);

// より柔軟な通知（ビット操作）
xTaskNotify(xTaskHandle, 0x01, eSetBits);     // ビットセット
xTaskNotify(xTaskHandle, 1, eIncrement);       // インクリメント
xTaskNotify(xTaskHandle, value, eSetValueWithOverwrite);

// 通知待ち
xTaskNotifyWait(
    0x00,              // エントリ時にクリアするビット
    ULONG_MAX,         // 終了時にクリアするビット
    &ulNotificationValue,
    portMAX_DELAY
);
```

### ISR からの通知

```c
void UART_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // ISR から通知
    vTaskNotifyGiveFromISR(xTaskHandle, &xHigherPriorityTaskWoken);

    // コンテキストスイッチが必要な場合
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

---

## 3. Synchronization Primitives

### Binary Semaphore - バイナリセマフォ

```c
#include "semphr.h"

SemaphoreHandle_t xSemaphore = NULL;

// 作成
xSemaphore = xSemaphoreCreateBinary();

// Take (P操作、待機)
if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
    // セマフォ取得成功
}

// Give (V操作、解放)
xSemaphoreGive(xSemaphore);

// ISR から Give
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
```

### Counting Semaphore - カウンティングセマフォ

```c
// 最大カウント: 10, 初期値: 0
SemaphoreHandle_t xCountingSem = xSemaphoreCreateCounting(10, 0);

// リソースプール管理の例
void vProducerTask(void *pvParameters) {
    for (;;) {
        // リソース生成
        xSemaphoreGive(xCountingSem);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vConsumerTask(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xCountingSem, portMAX_DELAY) == pdTRUE) {
            // リソース消費
        }
    }
}
```

### Mutex - ミューテックス

```c
SemaphoreHandle_t xMutex = NULL;

// 作成（優先度継承機能付き）
xMutex = xSemaphoreCreateMutex();

// クリティカルセクションの保護
void vAccessSharedResource(void) {
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // 共有リソースへのアクセス
        // ...

        xSemaphoreGive(xMutex);
    }
}
```

### Recursive Mutex - 再帰的ミューテックス

```c
SemaphoreHandle_t xRecursiveMutex = xSemaphoreCreateRecursiveMutex();

void vFunctionA(void) {
    if (xSemaphoreTakeRecursive(xRecursiveMutex, portMAX_DELAY) == pdTRUE) {
        vFunctionB();  // 同じタスク内で再度ロック可能
        xSemaphoreGiveRecursive(xRecursiveMutex);
    }
}

void vFunctionB(void) {
    if (xSemaphoreTakeRecursive(xRecursiveMutex, portMAX_DELAY) == pdTRUE) {
        // 処理
        xSemaphoreGiveRecursive(xRecursiveMutex);
    }
}
```

### Queue - キュー

```c
#include "queue.h"

typedef struct {
    int id;
    int value;
} Message_t;

QueueHandle_t xQueue = NULL;

// キュー作成（10要素、各要素のサイズは Message_t）
xQueue = xQueueCreate(10, sizeof(Message_t));

// 送信
void vSenderTask(void *pvParameters) {
    Message_t msg = {.id = 1, .value = 100};

    // 後方に追加（FIFO）
    xQueueSend(xQueue, &msg, pdMS_TO_TICKS(100));

    // 前方に追加（緊急メッセージ用）
    xQueueSendToFront(xQueue, &msg, pdMS_TO_TICKS(100));
}

// 受信
void vReceiverTask(void *pvParameters) {
    Message_t rxMsg;

    for (;;) {
        if (xQueueReceive(xQueue, &rxMsg, portMAX_DELAY) == pdTRUE) {
            printf("Received: id=%d, value=%d\n", rxMsg.id, rxMsg.value);
        }
    }
}

// ISR から送信
void ISR_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    Message_t msg = {.id = 0, .value = 999};

    xQueueSendFromISR(xQueue, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### Event Groups - イベントグループ

```c
#include "event_groups.h"

#define EVENT_BIT_0    (1 << 0)
#define EVENT_BIT_1    (1 << 1)
#define EVENT_BIT_2    (1 << 2)
#define ALL_EVENTS     (EVENT_BIT_0 | EVENT_BIT_1 | EVENT_BIT_2)

EventGroupHandle_t xEventGroup = NULL;

// 作成
xEventGroup = xEventGroupCreate();

// イベントセット
xEventGroupSetBits(xEventGroup, EVENT_BIT_0);

// イベント待機（いずれかのビット）
EventBits_t uxBits = xEventGroupWaitBits(
    xEventGroup,
    EVENT_BIT_0 | EVENT_BIT_1,  // 待機するビット
    pdTRUE,                      // 終了時にクリア
    pdFALSE,                     // いずれか1つでOK (pdTRUE なら全部必要)
    pdMS_TO_TICKS(100)
);

if ((uxBits & EVENT_BIT_0) != 0) {
    // EVENT_BIT_0 がセットされた
}

// 複数タスクの同期（ランデブー）
xEventGroupSync(
    xEventGroup,
    EVENT_BIT_0,       // このタスクがセットするビット
    ALL_EVENTS,        // 待機する全ビット
    portMAX_DELAY
);
```

### Stream Buffer - ストリームバッファ

```c
#include "stream_buffer.h"

// 100バイトのストリームバッファ、トリガーレベル10バイト
StreamBufferHandle_t xStreamBuffer = xStreamBufferCreate(100, 10);

// 送信（可変長）
size_t xBytesSent = xStreamBufferSend(
    xStreamBuffer,
    pvTxData,
    xDataLength,
    pdMS_TO_TICKS(100)
);

// 受信
size_t xReceivedBytes = xStreamBufferReceive(
    xStreamBuffer,
    pvRxData,
    sizeof(pvRxData),
    portMAX_DELAY
);
```

### Message Buffer - メッセージバッファ

```c
#include "message_buffer.h"

// 可変長メッセージ用
MessageBufferHandle_t xMessageBuffer = xMessageBufferCreate(256);

// 送信
size_t xBytesSent = xMessageBufferSend(
    xMessageBuffer,
    &txMessage,
    sizeof(txMessage),
    pdMS_TO_TICKS(100)
);

// 受信
size_t xReceivedBytes = xMessageBufferReceive(
    xMessageBuffer,
    &rxMessage,
    sizeof(rxMessage),
    portMAX_DELAY
);
```

---

## 4. Software Timers

### Timer Types

| タイプ | 説明 |
|--------|------|
| **One-shot** | 一度だけ実行。`xAutoReload = pdFALSE` |
| **Auto-reload (Periodic)** | 周期的に実行。`xAutoReload = pdTRUE` |

### Timer Creation and Usage

```c
#include "timers.h"

TimerHandle_t xOneShotTimer = NULL;
TimerHandle_t xPeriodicTimer = NULL;

// コールバック関数
void vTimerCallback(TimerHandle_t xTimer) {
    // タイマーIDで識別
    uint32_t ulTimerID = (uint32_t)pvTimerGetTimerID(xTimer);

    if (ulTimerID == 0) {
        printf("One-shot timer expired\n");
    } else if (ulTimerID == 1) {
        printf("Periodic timer tick\n");
    }
}

void vCreateTimers(void) {
    // ワンショットタイマー（5秒後に1回だけ）
    xOneShotTimer = xTimerCreate(
        "OneShotTimer",           // 名前
        pdMS_TO_TICKS(5000),      // 周期
        pdFALSE,                  // ワンショット
        (void *)0,                // タイマーID
        vTimerCallback            // コールバック
    );

    // 周期タイマー（1秒ごと）
    xPeriodicTimer = xTimerCreate(
        "PeriodicTimer",
        pdMS_TO_TICKS(1000),
        pdTRUE,                   // 自動リロード
        (void *)1,
        vTimerCallback
    );

    // タイマー開始
    if (xTimerStart(xOneShotTimer, 0) != pdPASS) {
        // タイマー開始失敗
    }

    if (xTimerStart(xPeriodicTimer, 0) != pdPASS) {
        // タイマー開始失敗
    }
}

// タイマー操作
void vTimerOperations(void) {
    // 停止
    xTimerStop(xPeriodicTimer, 0);

    // リセット（再スタート）
    xTimerReset(xPeriodicTimer, 0);

    // 周期変更
    xTimerChangePeriod(xPeriodicTimer, pdMS_TO_TICKS(2000), 0);

    // 削除
    xTimerDelete(xOneShotTimer, 0);
}
```

### ISR からのタイマー操作

```c
void ISR_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerResetFromISR(xPeriodicTimer, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

---

## 5. Memory Management

### Heap Implementations

| 実装 | 特徴 | 用途 |
|------|------|------|
| **heap_1** | allocate のみ、free 不可。決定論的。 | 静的なシステム |
| **heap_2** | allocate/free 可能。フラグメンテーション注意。非推奨。 | レガシー |
| **heap_3** | 標準 malloc/free をラップ。非決定論的。 | 標準ライブラリ利用時 |
| **heap_4** | 推奨。隣接ブロック結合でフラグメンテーション低減。 | 一般的な用途 |
| **heap_5** | heap_4 の機能 + 非連続メモリ領域対応。 | 分散メモリシステム |

### heap_4 Configuration (推奨)

```c
// FreeRTOSConfig.h
#define configTOTAL_HEAP_SIZE    ((size_t)(32 * 1024))  // 32KB
#define configAPPLICATION_ALLOCATED_HEAP    0

// heap_4.c を使用
// FreeRTOS/Source/portable/MemMang/heap_4.c をプロジェクトに追加
```

### heap_5 Configuration (複数メモリ領域)

```c
// メモリ領域の定義
HeapRegion_t xHeapRegions[] = {
    { (uint8_t *)0x20000000, 0x10000 },  // 内部SRAM: 64KB
    { (uint8_t *)0x60000000, 0x80000 },  // 外部SRAM: 512KB
    { NULL, 0 }                          // 終端
};

// main() の最初で初期化
void main(void) {
    vPortDefineHeapRegions(xHeapRegions);

    // その後で FreeRTOS オブジェクト作成
    // ...
}
```

### Static Allocation（ヒープ不使用）

```c
// FreeRTOSConfig.h
#define configSUPPORT_STATIC_ALLOCATION    1
#define configSUPPORT_DYNAMIC_ALLOCATION   0  // 動的割り当て無効化

// 必須コールバック関数の実装
void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer,
    uint32_t *pulIdleTaskStackSize
) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(
    StaticTask_t **ppxTimerTaskTCBBuffer,
    StackType_t **ppxTimerTaskStackBuffer,
    uint32_t *pulTimerTaskStackSize
) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
```

---

## 6. Debugging

### Stack Overflow Detection

```c
// FreeRTOSConfig.h
#define configCHECK_FOR_STACK_OVERFLOW    2  // Method 2 推奨

// フック関数の実装
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("Stack overflow in task: %s\n", pcTaskName);

    // システム停止またはリセット
    for (;;) {
        // デバッガでブレーク
    }
}
```

**検出方法:**
- **Method 1**: コンテキストスイッチ時にスタックポインタが有効範囲内かチェック
- **Method 2**: スタック末尾16バイトをパターンで初期化し、破壊をチェック（より確実）

### Malloc Failed Hook

```c
// FreeRTOSConfig.h
#define configUSE_MALLOC_FAILED_HOOK    1

void vApplicationMallocFailedHook(void) {
    printf("Memory allocation failed!\n");

    // ヒープ状態の確認
    size_t xFreeHeap = xPortGetFreeHeapSize();
    size_t xMinEverFreeHeap = xPortGetMinimumEverFreeHeapSize();

    printf("Free heap: %u, Min ever: %u\n", xFreeHeap, xMinEverFreeHeap);
}
```

### Runtime Statistics

```c
// FreeRTOSConfig.h
#define configGENERATE_RUN_TIME_STATS        1
#define configUSE_TRACE_FACILITY             1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

// タイマー設定（tick より10倍以上高速なタイマーを使用）
extern void vConfigureTimerForRunTimeStats(void);
extern uint32_t ulGetRunTimeCounterValue(void);

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()          ulGetRunTimeCounterValue()

// 統計表示
void vPrintRunTimeStats(void) {
    char pcWriteBuffer[1024];

    // タスクリスト
    vTaskList(pcWriteBuffer);
    printf("Task List:\n%s\n", pcWriteBuffer);

    // 実行時間統計
    vTaskGetRunTimeStats(pcWriteBuffer);
    printf("Run Time Stats:\n%s\n", pcWriteBuffer);
}
```

### Trace Tools

**Percepio Tracealyzer**:
- FreeRTOS 公式推奨のトレースツール
- タスクスケジューリング、API 呼び出し、CPU 負荷をビジュアル化
- FreeRTOS 202411.00 にサブモジュールとして含まれる

```c
// Tracealyzer 統合例
#include "trcRecorder.h"

void main(void) {
    // トレース開始
    vTraceEnable(TRC_START);

    // FreeRTOS 初期化
    // ...

    vTaskStartScheduler();
}
```

---

## 7. FreeRTOS+ Libraries

### FreeRTOS+TCP

軽量な TCP/IP スタック。Berkeley sockets API 互換。

```c
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

// ネットワーク初期化
const uint8_t ucIPAddress[4] = {192, 168, 1, 100};
const uint8_t ucNetMask[4] = {255, 255, 255, 0};
const uint8_t ucGatewayAddress[4] = {192, 168, 1, 1};
const uint8_t ucDNSServerAddress[4] = {8, 8, 8, 8};
const uint8_t ucMACAddress[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

FreeRTOS_IPInit(
    ucIPAddress,
    ucNetMask,
    ucGatewayAddress,
    ucDNSServerAddress,
    ucMACAddress
);

// TCP クライアント例
void vTcpClientTask(void *pvParameters) {
    Socket_t xSocket;
    struct freertos_sockaddr xRemoteAddress;

    xSocket = FreeRTOS_socket(
        FREERTOS_AF_INET,
        FREERTOS_SOCK_STREAM,
        FREERTOS_IPPROTO_TCP
    );

    xRemoteAddress.sin_addr = FreeRTOS_inet_addr("192.168.1.1");
    xRemoteAddress.sin_port = FreeRTOS_htons(80);

    if (FreeRTOS_connect(xSocket, &xRemoteAddress, sizeof(xRemoteAddress)) == 0) {
        const char *pcMessage = "GET / HTTP/1.1\r\n\r\n";
        FreeRTOS_send(xSocket, pcMessage, strlen(pcMessage), 0);

        char cBuffer[256];
        FreeRTOS_recv(xSocket, cBuffer, sizeof(cBuffer), 0);
    }

    FreeRTOS_closesocket(xSocket);
    vTaskDelete(NULL);
}
```

### FreeRTOS+FAT

FAT ファイルシステム実装。

```c
#include "ff_stdio.h"

// ファイル操作
void vFileOperationExample(void) {
    FF_FILE *pxFile;

    // ファイル書き込み
    pxFile = ff_fopen("/sdcard/test.txt", "w");
    if (pxFile != NULL) {
        ff_fprintf(pxFile, "Hello, FreeRTOS+FAT!\n");
        ff_fclose(pxFile);
    }

    // ファイル読み込み
    pxFile = ff_fopen("/sdcard/test.txt", "r");
    if (pxFile != NULL) {
        char pcBuffer[64];
        ff_fgets(pcBuffer, sizeof(pcBuffer), pxFile);
        printf("Read: %s", pcBuffer);
        ff_fclose(pxFile);
    }
}
```

### FreeRTOS+CLI

拡張可能なコマンドラインインターフェース。

```c
#include "FreeRTOS_CLI.h"

// コマンド実装
static BaseType_t prvTaskStatsCommand(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString
) {
    vTaskList(pcWriteBuffer);
    return pdFALSE;  // 完了
}

// コマンド定義
static const CLI_Command_Definition_t xTaskStatsCommand = {
    "task-stats",                              // コマンド名
    "task-stats: Displays task statistics\r\n", // ヘルプ
    prvTaskStatsCommand,                       // コールバック
    0                                          // パラメータ数
};

// コマンド登録
void vRegisterCLICommands(void) {
    FreeRTOS_CLIRegisterCommand(&xTaskStatsCommand);
}

// CLI 処理タスク
void vCLITask(void *pvParameters) {
    char cInputBuffer[64];
    char cOutputBuffer[512];

    for (;;) {
        // UART などから入力を受信
        if (xGetInputLine(cInputBuffer, sizeof(cInputBuffer))) {
            BaseType_t xMoreDataToFollow;

            do {
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand(
                    cInputBuffer,
                    cOutputBuffer,
                    sizeof(cOutputBuffer)
                );

                // 出力を送信
                vSendOutput(cOutputBuffer);
            } while (xMoreDataToFollow != pdFALSE);
        }
    }
}
```

### FreeRTOS+POSIX

POSIX 互換レイヤー。既存の POSIX コードの移植に便利。

```c
#include "FreeRTOS_POSIX.h"
#include <pthread.h>
#include <semaphore.h>

// pthread によるスレッド作成
void *thread_function(void *arg) {
    while (1) {
        // 処理
        sleep(1);
    }
    return NULL;
}

void create_posix_thread(void) {
    pthread_t thread;
    pthread_create(&thread, NULL, thread_function, NULL);
}
```

---

## 8. SMP (Symmetric Multiprocessing) Support

FreeRTOS V11.0 からメインラインに SMP サポートが統合された。

### Configuration

```c
// FreeRTOSConfig.h
#define configNUMBER_OF_CORES                    2
#define configRUN_MULTIPLE_PRIORITIES            1
#define configUSE_CORE_AFFINITY                  1
#define configUSE_TASK_PREEMPTION_DISABLE        1
```

### Core Affinity

```c
// 特定コアでのみ実行するタスク
TaskHandle_t xHandle;
xTaskCreate(vMyTask, "Task", 1024, NULL, 1, &xHandle);

// Core 0 でのみ実行
vTaskCoreAffinitySet(xHandle, (1 << 0));

// Core 1 でのみ実行
vTaskCoreAffinitySet(xHandle, (1 << 1));

// 両方のコアで実行可能（デフォルト）
vTaskCoreAffinitySet(xHandle, (1 << 0) | (1 << 1));
```

### SMP Considerations

```c
// SMP 環境での注意点

// 1. クリティカルセクション（全コア停止）
taskENTER_CRITICAL();
// 共有データへのアクセス
taskEXIT_CRITICAL();

// 2. ISR 内でのクリティカルセクション
UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
// 処理
taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

// 3. 特定タスクのプリエンプション無効化
vTaskPreemptionDisable(NULL);  // 現在のタスク
// 他のタスクに横取りされない処理
vTaskPreemptionEnable(NULL);
```

### Raspberry Pi Pico (RP2040) SMP Example

```c
// RP2040 デュアルコア設定例
#define configNUMBER_OF_CORES    2
#define configUSE_CORE_AFFINITY  1

void vCore0Task(void *pvParameters) {
    // Core 0 専用処理
    for (;;) {
        // センサー読み取りなど
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vCore1Task(void *pvParameters) {
    // Core 1 専用処理
    for (;;) {
        // 通信処理など
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void main(void) {
    TaskHandle_t xCore0Task, xCore1Task;

    xTaskCreate(vCore0Task, "Core0", 256, NULL, 1, &xCore0Task);
    xTaskCreate(vCore1Task, "Core1", 256, NULL, 1, &xCore1Task);

    vTaskCoreAffinitySet(xCore0Task, (1 << 0));
    vTaskCoreAffinitySet(xCore1Task, (1 << 1));

    vTaskStartScheduler();
}
```

---

## 9. Integration Examples

### ESP-IDF Integration

ESP-IDF は FreeRTOS を内部コンポーネントとして統合。デュアルコア SMP 対応。

```c
// ESP-IDF での FreeRTOS 使用例
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void) {
    ESP_LOGI(TAG, "Starting FreeRTOS tasks");

    // タスク作成（ESP-IDF スタイル）
    xTaskCreate(
        vMyTask,
        "my_task",
        4096,           // ESP32 ではバイト単位
        NULL,
        5,
        NULL
    );

    // 特定コアにピン留め
    xTaskCreatePinnedToCore(
        vCore0Task,
        "core0_task",
        4096,
        NULL,
        5,
        NULL,
        0               // Core 0
    );

    xTaskCreatePinnedToCore(
        vCore1Task,
        "core1_task",
        4096,
        NULL,
        5,
        NULL,
        1               // Core 1
    );
}
```

### STM32Cube Integration (X-CUBE-FREERTOS)

```c
// STM32CubeMX で生成されるコード構造
#include "main.h"
#include "cmsis_os.h"

// CMSIS-RTOS v2 API（STM32 推奨）
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

void StartDefaultTask(void *argument) {
    for (;;) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        osDelay(500);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    osKernelInitialize();

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    osKernelStart();

    while (1) {
        // ここには到達しない
    }
}
```

### Native FreeRTOS API on STM32

```c
// CMSIS-RTOS を使わない直接 API 使用
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

void vLEDTask(void *pvParameters) {
    for (;;) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vUARTTask(void *pvParameters) {
    char buffer[64];

    for (;;) {
        // UART 処理
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    xTaskCreate(vLEDTask, "LED", 128, NULL, 1, NULL);
    xTaskCreate(vUARTTask, "UART", 256, NULL, 2, NULL);

    vTaskStartScheduler();

    for (;;) {}
}

// FreeRTOS 用のタイムベース（SysTick）
void vPortSetupTimerInterrupt(void) {
    // STM32 HAL が設定済みの場合は不要
}

void SysTick_Handler(void) {
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
```

---

## 10. Best Practices

### タスク設計

1. **適切なスタックサイズ**: `uxTaskGetStackHighWaterMark()` で使用量を確認
2. **優先度設計**: デッドライン重要度に基づいて設定
3. **ブロッキング API の使用**: ポーリングを避け、`vTaskDelay()` や同期プリミティブを使用

### メモリ管理

1. **heap_4 を推奨**: 一般的な用途に最適
2. **静的割り当ての検討**: 安全性が重要なシステムでは `configSUPPORT_STATIC_ALLOCATION`
3. **ヒープ監視**: `xPortGetFreeHeapSize()` と `xPortGetMinimumEverFreeHeapSize()` で監視

### デバッグ

1. **Stack Overflow 検出を常に有効化**: `configCHECK_FOR_STACK_OVERFLOW 2`
2. **Malloc Failed Hook**: メモリ不足を早期検出
3. **Runtime Stats**: CPU 使用率の確認

### 同期プリミティブの選択

| 用途 | 推奨プリミティブ |
|------|-----------------|
| 単純な通知 | Task Notification |
| 排他制御 | Mutex |
| リソースカウント | Counting Semaphore |
| データ転送 | Queue |
| 複数イベント待機 | Event Group |
| ストリームデータ | Stream Buffer |
| 可変長メッセージ | Message Buffer |

---

## References

- [FreeRTOS Official Documentation](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/00-FreeRTOS-Kernel-features)
- [FreeRTOS GitHub Repository](https://github.com/FreeRTOS/FreeRTOS)
- [FreeRTOS-Kernel GitHub](https://github.com/FreeRTOS/FreeRTOS-Kernel)
- [Mastering the FreeRTOS Real Time Kernel](https://freertos.gitbook.io/mastering-the-freertos-tm-real-time-kernel/)
- [ESP-IDF FreeRTOS Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos.html)
- [X-CUBE-FREERTOS (STMicroelectronics)](https://github.com/STMicroelectronics/x-cube-freertos)
- [Percepio Tracealyzer](https://percepio.com/tracealyzer/freertostrace/)
