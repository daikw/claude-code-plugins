# Design Patterns Rules

組込/エッジ開発における設計パターン。

## API Response Pattern

統一的なレスポンス形式を使用する。

```c
// ✅ 統一的なエラーハンドリング
typedef enum {
    RESULT_OK = 0,
    RESULT_ERROR_INVALID_PARAM,
    RESULT_ERROR_TIMEOUT,
    RESULT_ERROR_BUSY,
    RESULT_ERROR_NO_MEMORY,
} result_t;

typedef struct {
    result_t status;
    void* data;
    size_t data_len;
    const char* error_msg;
} response_t;

// 使用例
response_t sensor_read(sensor_id_t id) {
    if (!is_valid_sensor(id)) {
        return (response_t){
            .status = RESULT_ERROR_INVALID_PARAM,
            .error_msg = "Invalid sensor ID"
        };
    }
    // ...
}
```

## Hardware Abstraction Layer (HAL)

ハードウェア依存部を抽象化する。

```c
// ✅ HAL インターフェース定義
typedef struct {
    int (*init)(void);
    int (*read)(uint8_t* buf, size_t len);
    int (*write)(const uint8_t* buf, size_t len);
    void (*deinit)(void);
} uart_driver_t;

// プラットフォーム固有の実装
extern const uart_driver_t stm32_uart_driver;
extern const uart_driver_t esp32_uart_driver;
extern const uart_driver_t mock_uart_driver;  // テスト用

// アプリケーションコード（プラットフォーム非依存）
void app_init(const uart_driver_t* driver) {
    driver->init();
    // ...
}
```

## State Machine Pattern

状態遷移を明示的に管理する。

```c
// ✅ テーブル駆動型ステートマシン
typedef enum {
    STATE_IDLE,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_ERROR,
    STATE_COUNT
} state_t;

typedef enum {
    EVENT_START,
    EVENT_CONNECTED,
    EVENT_DISCONNECT,
    EVENT_ERROR,
    EVENT_COUNT
} event_t;

typedef struct {
    state_t next_state;
    void (*action)(void);
} transition_t;

static const transition_t state_table[STATE_COUNT][EVENT_COUNT] = {
    [STATE_IDLE] = {
        [EVENT_START] = {STATE_CONNECTING, on_start},
    },
    [STATE_CONNECTING] = {
        [EVENT_CONNECTED] = {STATE_CONNECTED, on_connected},
        [EVENT_ERROR] = {STATE_ERROR, on_error},
    },
    // ...
};

void state_machine_process(event_t event) {
    transition_t t = state_table[current_state][event];
    if (t.action) t.action();
    current_state = t.next_state;
}
```

## Observer Pattern (Publish-Subscribe)

イベント駆動設計のためのパターン。

```c
// ✅ イベント購読パターン
typedef void (*event_callback_t)(void* data);

typedef struct {
    event_callback_t callbacks[MAX_SUBSCRIBERS];
    size_t count;
} event_topic_t;

void event_subscribe(event_topic_t* topic, event_callback_t cb) {
    if (topic->count < MAX_SUBSCRIBERS) {
        topic->callbacks[topic->count++] = cb;
    }
}

void event_publish(event_topic_t* topic, void* data) {
    for (size_t i = 0; i < topic->count; i++) {
        topic->callbacks[i](data);
    }
}
```

## Command Pattern

コマンド処理の統一化。

```c
// ✅ コマンドハンドラパターン
typedef struct {
    const char* name;
    result_t (*handler)(int argc, char** argv);
    const char* help;
} command_t;

static const command_t commands[] = {
    {"status", cmd_status, "Show system status"},
    {"reset", cmd_reset, "Reset device"},
    {"config", cmd_config, "Configure settings"},
    {NULL, NULL, NULL}  // 終端
};

result_t execute_command(const char* input) {
    char* argv[MAX_ARGS];
    int argc = parse_args(input, argv);

    for (const command_t* cmd = commands; cmd->name; cmd++) {
        if (strcmp(cmd->name, argv[0]) == 0) {
            return cmd->handler(argc, argv);
        }
    }
    return RESULT_ERROR_INVALID_PARAM;
}
```

## Ring Buffer Pattern

割り込みセーフなデータバッファ。

```c
// ✅ ロックフリーリングバッファ
typedef struct {
    uint8_t* buffer;
    size_t size;
    volatile size_t head;  // 書き込み位置
    volatile size_t tail;  // 読み取り位置
} ring_buffer_t;

bool ring_buffer_write(ring_buffer_t* rb, uint8_t data) {
    size_t next = (rb->head + 1) % rb->size;
    if (next == rb->tail) return false;  // Full
    rb->buffer[rb->head] = data;
    rb->head = next;
    return true;
}

bool ring_buffer_read(ring_buffer_t* rb, uint8_t* data) {
    if (rb->head == rb->tail) return false;  // Empty
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return true;
}
```

## Dependency Injection

テスト容易性のための依存性注入。

```c
// ✅ 依存性注入でテスト可能に
typedef struct {
    int (*get_time_ms)(void);
    void (*delay_ms)(int ms);
    int (*read_sensor)(int channel);
} system_deps_t;

// 本番用
static const system_deps_t production_deps = {
    .get_time_ms = hal_get_time_ms,
    .delay_ms = hal_delay_ms,
    .read_sensor = hal_adc_read,
};

// テスト用
static const system_deps_t test_deps = {
    .get_time_ms = mock_get_time_ms,
    .delay_ms = mock_delay_ms,
    .read_sensor = mock_adc_read,
};
```
