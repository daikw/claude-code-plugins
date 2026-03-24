# Design Principles Rules

組込/エッジ/ロボティクス開発における設計原則。ハードウェア・ソフトウェア非依存の普遍的な原則。

## Pre-Commit Checklist

コミット前に必ず確認する設計原則チェックリスト:

### Defensive Coding
- [ ] 全ての public 関数に precondition チェックがある
- [ ] 関数の終了時に postcondition を満たしている
- [ ] クラス/モジュールの invariant が維持されている
- [ ] 入力パラメータの境界チェックがある
- [ ] ポインタ/参照の null チェックがある

### Fail-Safe Design
- [ ] 異常時に安全な状態へ遷移する
- [ ] デフォルト値が安全側に設定されている
- [ ] タイムアウト処理が実装されている
- [ ] リソース解放が保証されている（エラー時も含む）

### Single Responsibility
- [ ] 1つのモジュール = 1つの責務
- [ ] 関数の副作用が最小限かつ明示されている
- [ ] 変更理由が1つだけである

### Separation of Concerns
- [ ] HAL/Driver/Application層が分離されている
- [ ] ビジネスロジックがハードウェア依存していない
- [ ] 各層の境界が明確である

### API Contracts
- [ ] 全 API の precondition が文書化されている
- [ ] 全 API の postcondition が文書化されている
- [ ] エラー条件と戻り値が明示されている
- [ ] 副作用が文書化されている

### Deterministic Behavior
- [ ] 同一入力に対して同一出力が保証されている
- [ ] 時間依存の動作が明示されている
- [ ] ランダム性がある場合はシード制御可能である

### Testability
- [ ] 依存性が注入可能である
- [ ] インターフェースがモック可能である
- [ ] 外部依存がない単体テストが可能である

---

## 1. Defensive Coding（防御的プログラミング）

失敗を検出・隔離し、可能であれば回復するためのコーディング手法。

### Precondition（事前条件）

関数が正しく動作するために、呼び出し側が満たすべき条件。

```c
// ✅ Good: 明確な precondition チェック
/**
 * @brief センサー値を読み取る
 * @pre sensor_id は 0 <= sensor_id < MAX_SENSORS の範囲内
 * @pre sensor は初期化済み（sensor_initialized[sensor_id] == true）
 * @param sensor_id センサーID
 * @return センサー値。エラー時は SENSOR_ERROR_VALUE
 */
int sensor_read(uint8_t sensor_id) {
    // Precondition check
    if (sensor_id >= MAX_SENSORS) {
        log_error("sensor_read: invalid sensor_id=%d", sensor_id);
        return SENSOR_ERROR_VALUE;
    }
    if (!sensor_initialized[sensor_id]) {
        log_error("sensor_read: sensor %d not initialized", sensor_id);
        return SENSOR_ERROR_VALUE;
    }

    // ... センサー読み取り処理
}
```

```c
// ❌ Bad: precondition チェックなし
int sensor_read(uint8_t sensor_id) {
    // 直接アクセス - 範囲外アクセスの危険
    return sensors[sensor_id].value;
}
```

### Postcondition（事後条件）

関数が正常終了時に保証すべき条件。

```c
// ✅ Good: postcondition を保証
/**
 * @brief バッファにデータを追加
 * @post 成功時: buffer->count が1増加
 * @post 成功時: 追加したデータが buffer->data[old_count] に格納
 * @post 失敗時: buffer の状態は呼び出し前と同一
 */
result_t buffer_push(buffer_t* buffer, const data_t* data) {
    if (buffer->count >= buffer->capacity) {
        return RESULT_ERROR_FULL;  // 状態変更なし
    }

    size_t old_count = buffer->count;

    // データコピー（失敗時は状態を戻す）
    if (!copy_data(&buffer->data[old_count], data)) {
        return RESULT_ERROR_COPY;  // 状態変更なし
    }

    buffer->count = old_count + 1;

    // Postcondition assertion (debug build only)
    assert(buffer->count == old_count + 1);
    assert(data_equals(&buffer->data[old_count], data));

    return RESULT_OK;
}
```

```c
// ❌ Bad: 部分的な状態変更のまま失敗
result_t buffer_push(buffer_t* buffer, const data_t* data) {
    buffer->count++;  // 先にカウンタを更新

    if (!copy_data(&buffer->data[buffer->count - 1], data)) {
        // count は増えたまま、データは不完全
        return RESULT_ERROR;
    }
    return RESULT_OK;
}
```

### Invariant（不変条件）

オブジェクト/モジュールが常に満たすべき条件。

```c
// ✅ Good: invariant を維持するリングバッファ
/**
 * Ring Buffer Invariants:
 * - 0 <= head < capacity
 * - 0 <= tail < capacity
 * - count == (head - tail + capacity) % capacity
 * - count <= capacity
 */
typedef struct {
    uint8_t* data;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
} ring_buffer_t;

// Invariant チェック関数（debug build用）
static bool ring_buffer_check_invariant(const ring_buffer_t* rb) {
    if (rb->head >= rb->capacity) return false;
    if (rb->tail >= rb->capacity) return false;
    if (rb->count > rb->capacity) return false;

    size_t expected_count = (rb->head - rb->tail + rb->capacity) % rb->capacity;
    if (rb->count != expected_count) return false;

    return true;
}

bool ring_buffer_write(ring_buffer_t* rb, uint8_t byte) {
    assert(ring_buffer_check_invariant(rb));  // Entry invariant

    if (rb->count >= rb->capacity) {
        return false;
    }

    rb->data[rb->head] = byte;
    rb->head = (rb->head + 1) % rb->capacity;
    rb->count++;

    assert(ring_buffer_check_invariant(rb));  // Exit invariant
    return true;
}
```

---

## 2. Fail-Safe Design（フェイルセーフ設計）

障害発生時に安全な状態を維持する設計。

### Safe State への遷移

```c
// ✅ Good: 異常検出時は安全状態へ
typedef enum {
    MOTOR_STATE_STOPPED,      // Safe state
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_ERROR         // Also safe (motor stopped)
} motor_state_t;

void motor_control_loop(void) {
    // ウォッチドッグタイマーをフィード
    watchdog_feed();

    // 異常検出
    if (detect_overcurrent() || detect_overheat()) {
        // 即座に安全状態へ
        motor_emergency_stop();
        set_state(MOTOR_STATE_ERROR);
        log_error("Safety shutdown triggered");
        return;
    }

    // 通常処理...
}

// ウォッチドッグタイムアウト時のハンドラ
void watchdog_timeout_handler(void) {
    // ソフトウェアがハングした場合も安全側へ
    motor_emergency_stop();
    system_reset();
}
```

```c
// ❌ Bad: 異常時もそのまま動作継続
void motor_control_loop(void) {
    if (detect_overcurrent()) {
        log_warning("Overcurrent detected");
        // 警告のみで継続 - 危険
    }
    // モーター制御継続...
}
```

### Safe Defaults（安全なデフォルト値）

```c
// ✅ Good: 安全側のデフォルト
typedef struct {
    bool enable_output;        // default: false (出力無効)
    uint16_t max_speed;        // default: 0 (停止)
    uint16_t timeout_ms;       // default: 1000 (短いタイムアウト)
    bool require_confirmation; // default: true (確認必要)
} motor_config_t;

// 初期化関数で安全なデフォルト値を設定
void motor_config_init(motor_config_t* config) {
    config->enable_output = false;
    config->max_speed = 0;
    config->timeout_ms = 1000;
    config->require_confirmation = true;
}

// 設定読み込み失敗時も安全なデフォルトを使用
result_t motor_config_load(motor_config_t* config) {
    motor_config_init(config);  // まず安全なデフォルトで初期化

    result_t result = config_read_from_storage(config);
    if (result != RESULT_OK) {
        log_warning("Config load failed, using safe defaults");
        // config は既に安全なデフォルト値
    }
    return result;
}
```

```c
// ❌ Bad: 危険なデフォルト値
typedef struct {
    bool enable_output;        // 未初期化 - 不定
    uint16_t max_speed;        // 未初期化 - 最大値の可能性
    uint16_t timeout_ms;       // 未初期化 - 0でタイムアウトなしの可能性
} motor_config_t;
```

### Graceful Degradation（優雅な機能低下）

```c
// ✅ Good: 段階的な機能低下
typedef enum {
    OPERATION_MODE_FULL,       // 全機能動作
    OPERATION_MODE_REDUCED,    // 縮退運転
    OPERATION_MODE_SAFE,       // 最小限の安全動作
    OPERATION_MODE_SHUTDOWN    // 完全停止
} operation_mode_t;

void system_health_check(void) {
    int healthy_sensors = count_healthy_sensors();
    int healthy_actuators = count_healthy_actuators();

    if (healthy_sensors >= MIN_SENSORS && healthy_actuators >= MIN_ACTUATORS) {
        set_operation_mode(OPERATION_MODE_FULL);
    } else if (healthy_sensors >= CRITICAL_SENSORS) {
        // センサー縮退: 冗長センサーで継続
        set_operation_mode(OPERATION_MODE_REDUCED);
        log_warning("Operating in reduced mode");
    } else if (can_safe_shutdown()) {
        // 安全に停止可能
        set_operation_mode(OPERATION_MODE_SAFE);
        log_error("Critical sensor failure, safe mode");
    } else {
        // 即座に停止
        set_operation_mode(OPERATION_MODE_SHUTDOWN);
        emergency_shutdown();
    }
}
```

---

## 3. Single Responsibility Principle（単一責任の原則）

1つのモジュールは1つの責務のみを持ち、変更理由は1つだけであるべき。

### 責務の分離

```c
// ✅ Good: 責務ごとにモジュールを分離

// sensor_reader.c - センサー読み取りのみ
typedef struct {
    int (*read_raw)(uint8_t channel);
} sensor_reader_t;

int sensor_read_temperature_raw(const sensor_reader_t* reader, uint8_t channel) {
    return reader->read_raw(channel);
}

// sensor_calibrator.c - キャリブレーションのみ
typedef struct {
    float offset;
    float scale;
} calibration_t;

float sensor_apply_calibration(int raw_value, const calibration_t* cal) {
    return (raw_value + cal->offset) * cal->scale;
}

// sensor_filter.c - フィルタリングのみ
typedef struct {
    float* buffer;
    size_t size;
    size_t index;
} moving_average_t;

float filter_apply_moving_average(moving_average_t* filter, float value) {
    filter->buffer[filter->index] = value;
    filter->index = (filter->index + 1) % filter->size;
    return calculate_average(filter->buffer, filter->size);
}

// sensor_service.c - 上位モジュールが組み合わせる
float sensor_service_get_temperature(sensor_service_t* service, uint8_t channel) {
    int raw = sensor_read_temperature_raw(&service->reader, channel);
    float calibrated = sensor_apply_calibration(raw, &service->calibration);
    float filtered = filter_apply_moving_average(&service->filter, calibrated);
    return filtered;
}
```

```c
// ❌ Bad: 1つの関数に複数の責務
float get_temperature(uint8_t channel) {
    // 1. ハードウェアアクセス
    int raw = ADC->DATA[channel];

    // 2. キャリブレーション
    float calibrated = (raw + OFFSET) * SCALE;

    // 3. フィルタリング
    static float buffer[10];
    static int idx = 0;
    buffer[idx++ % 10] = calibrated;
    float sum = 0;
    for (int i = 0; i < 10; i++) sum += buffer[i];

    // 4. ロギング（副作用）
    printf("Temperature: %.2f\n", sum / 10);

    // 5. アラート判定（別の責務）
    if (sum / 10 > TEMP_ALERT_THRESHOLD) {
        send_alert();  // 副作用
    }

    return sum / 10;
}
```

### 副作用の最小化と明示

```c
// ✅ Good: 副作用を明示し、純粋関数を優先

// 純粋関数（副作用なし）
float calculate_pid_output(
    float setpoint,
    float current,
    float kp, float ki, float kd,
    float integral,      // 状態は引数で渡す
    float prev_error     // 状態は引数で渡す
) {
    float error = setpoint - current;
    float derivative = error - prev_error;
    return kp * error + ki * integral + kd * derivative;
}

// 副作用を持つ関数は名前で明示
/**
 * @brief PIDコントローラの状態を更新する
 * @note この関数は controller の状態を変更する（副作用あり）
 */
void pid_controller_update_state(pid_controller_t* controller, float error) {
    controller->integral += error * controller->dt;
    controller->prev_error = error;
}
```

```c
// ❌ Bad: 隠れた副作用
float calculate_pid_output(float setpoint, float current) {
    // グローバル変数への暗黙の依存と変更
    float error = setpoint - current;
    g_integral += error;  // 隠れた副作用
    float output = g_kp * error + g_ki * g_integral + g_kd * (error - g_prev_error);
    g_prev_error = error;  // 隠れた副作用
    return output;
}
```

---

## 4. Separation of Concerns（関心の分離）

### 3層アーキテクチャ

```
┌─────────────────────────────────────────────────────┐
│                Application Layer                     │
│   (ビジネスロジック、ユースケース、状態管理)           │
│   - ハードウェア非依存                                │
│   - 単体テスト容易                                   │
├─────────────────────────────────────────────────────┤
│                  Driver Layer                        │
│   (デバイスドライバ、プロトコル実装)                  │
│   - HALを使用                                        │
│   - デバイス固有のロジック                            │
├─────────────────────────────────────────────────────┤
│          Hardware Abstraction Layer (HAL)            │
│   (レジスタアクセス、割り込み、タイミング)             │
│   - プラットフォーム固有                              │
│   - 最小限のインターフェース                          │
└─────────────────────────────────────────────────────┘
```

```c
// ✅ Good: 層ごとの責務分離

// === HAL Layer (platform_stm32.c) ===
// レジスタ直接アクセスはここだけ
void hal_gpio_write(uint8_t port, uint8_t pin, bool value) {
    if (value) {
        GPIO_PORTS[port]->BSRR = (1 << pin);
    } else {
        GPIO_PORTS[port]->BSRR = (1 << (pin + 16));
    }
}

uint32_t hal_timer_get_ms(void) {
    return SysTick->VAL / (SystemCoreClock / 1000);
}

// === Driver Layer (led_driver.c) ===
// デバイス固有ロジック、HALを使用
typedef struct {
    uint8_t port;
    uint8_t pin;
    bool active_high;
} led_config_t;

void led_driver_set(const led_config_t* led, bool on) {
    bool level = led->active_high ? on : !on;
    hal_gpio_write(led->port, led->pin, level);
}

// === Application Layer (notification_service.c) ===
// ビジネスロジック、ドライバを使用
void notification_show_error(notification_service_t* service) {
    // エラー表示のビジネスルール
    service->blink_count = 3;
    service->blink_interval_ms = 200;
    service->state = NOTIFICATION_BLINKING;

    // ドライバ経由でLED制御
    led_driver_set(&service->error_led, true);
}
```

```c
// ❌ Bad: 層の混在
void show_error_notification(void) {
    // Application層でレジスタ直接アクセス
    for (int i = 0; i < 3; i++) {
        GPIOA->BSRR = (1 << 5);           // HAL層の処理
        for (volatile int j = 0; j < 100000; j++);  // 遅延
        GPIOA->BSRR = (1 << (5 + 16));
        for (volatile int j = 0; j < 100000; j++);
    }
}
```

---

## 5. Abstraction Layers（抽象化層）

### HALによるポータビリティ

```c
// ✅ Good: HALインターフェース定義

// hal_interface.h - プラットフォーム非依存のインターフェース
typedef struct hal_uart_ops {
    int (*init)(uint32_t baudrate);
    int (*write)(const uint8_t* data, size_t len);
    int (*read)(uint8_t* data, size_t len, uint32_t timeout_ms);
    void (*deinit)(void);
} hal_uart_ops_t;

typedef struct hal_gpio_ops {
    int (*init)(uint8_t pin, uint8_t mode);
    void (*write)(uint8_t pin, bool value);
    bool (*read)(uint8_t pin);
} hal_gpio_ops_t;

typedef struct hal_timer_ops {
    uint32_t (*get_ms)(void);
    void (*delay_ms)(uint32_t ms);
} hal_timer_ops_t;

// HAL全体をまとめた構造体
typedef struct hal_interface {
    const hal_uart_ops_t* uart;
    const hal_gpio_ops_t* gpio;
    const hal_timer_ops_t* timer;
} hal_interface_t;

// hal_stm32.c - STM32用実装
static int stm32_uart_init(uint32_t baudrate) { /* ... */ }
static int stm32_uart_write(const uint8_t* data, size_t len) { /* ... */ }
// ...

const hal_uart_ops_t stm32_uart_ops = {
    .init = stm32_uart_init,
    .write = stm32_uart_write,
    .read = stm32_uart_read,
    .deinit = stm32_uart_deinit,
};

// hal_mock.c - テスト用モック実装
const hal_uart_ops_t mock_uart_ops = {
    .init = mock_uart_init,
    .write = mock_uart_write,
    .read = mock_uart_read,
    .deinit = mock_uart_deinit,
};

// アプリケーションコード（HAL非依存）
void app_init(const hal_interface_t* hal) {
    hal->uart->init(115200);
    hal->gpio->init(LED_PIN, GPIO_MODE_OUTPUT);
}
```

---

## 6. API Contracts（API契約）

### 完全な契約の文書化

```c
// ✅ Good: 完全なAPI契約

/**
 * @brief メッセージをキューに追加する
 *
 * @pre queue != NULL
 * @pre msg != NULL
 * @pre queue が初期化済み（queue_init() が呼ばれている）
 * @pre 割り込みコンテキストから呼び出し可能
 *
 * @post 成功時: queue のメッセージ数が1増加
 * @post 成功時: msg の内容がキューにコピーされる（msg自体は変更されない）
 * @post 失敗時: queue の状態は変更されない
 *
 * @param[in,out] queue  メッセージキューへのポインタ
 * @param[in]     msg    追加するメッセージへのポインタ
 *
 * @return RESULT_OK              成功
 * @return RESULT_ERROR_NULL      queue または msg が NULL
 * @return RESULT_ERROR_FULL      キューが満杯
 * @return RESULT_ERROR_NOT_INIT  キューが未初期化
 *
 * @note スレッドセーフ: 内部でロックを取得する
 * @note 割り込みセーフ: ISRから呼び出し可能（ただしタイムアウト=0で使用すること）
 *
 * @code
 * message_queue_t queue;
 * queue_init(&queue, buffer, sizeof(buffer));
 *
 * message_t msg = {.id = 1, .data = {0x01, 0x02}};
 * result_t result = queue_push(&queue, &msg);
 * if (result != RESULT_OK) {
 *     handle_error(result);
 * }
 * @endcode
 */
result_t queue_push(message_queue_t* queue, const message_t* msg);
```

```c
// ❌ Bad: 不十分な文書化
/**
 * キューにメッセージを追加
 */
int queue_push(void* queue, void* msg);
```

### エラー条件の明示

```c
// ✅ Good: エラー条件を型で表現
typedef enum {
    RESULT_OK = 0,

    // 引数エラー (1xx)
    RESULT_ERROR_NULL = 100,
    RESULT_ERROR_INVALID_PARAM = 101,
    RESULT_ERROR_OUT_OF_RANGE = 102,

    // 状態エラー (2xx)
    RESULT_ERROR_NOT_INITIALIZED = 200,
    RESULT_ERROR_ALREADY_INITIALIZED = 201,
    RESULT_ERROR_BUSY = 202,

    // リソースエラー (3xx)
    RESULT_ERROR_NO_MEMORY = 300,
    RESULT_ERROR_FULL = 301,
    RESULT_ERROR_EMPTY = 302,

    // 通信エラー (4xx)
    RESULT_ERROR_TIMEOUT = 400,
    RESULT_ERROR_CHECKSUM = 401,
    RESULT_ERROR_PROTOCOL = 402,

    // ハードウェアエラー (5xx)
    RESULT_ERROR_HARDWARE = 500,
    RESULT_ERROR_NOT_READY = 501,
} result_t;

// エラーメッセージ取得
const char* result_to_string(result_t result);

// エラー判定ヘルパー
static inline bool result_is_error(result_t r) { return r != RESULT_OK; }
static inline bool result_is_param_error(result_t r) { return r >= 100 && r < 200; }
static inline bool result_is_state_error(result_t r) { return r >= 200 && r < 300; }
```

---

## 7. Deterministic Behavior（決定論的動作）

### 再現可能な動作

```c
// ✅ Good: 決定論的な初期化とシード制御

typedef struct {
    uint32_t seed;
    uint32_t state;
} random_generator_t;

// シードを指定して初期化（再現可能）
void random_init(random_generator_t* rng, uint32_t seed) {
    rng->seed = seed;
    rng->state = seed;
}

// 決定論的な擬似乱数生成
uint32_t random_next(random_generator_t* rng) {
    // xorshift32
    uint32_t x = rng->state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng->state = x;
    return x;
}

// テスト時に再現可能
void test_random_behavior(void) {
    random_generator_t rng1, rng2;

    random_init(&rng1, 12345);
    random_init(&rng2, 12345);

    // 同じシードなら同じ結果
    assert(random_next(&rng1) == random_next(&rng2));
    assert(random_next(&rng1) == random_next(&rng2));
}
```

```c
// ❌ Bad: 非決定論的な動作
uint32_t get_random(void) {
    // 時刻に依存 - 再現不可能
    return HAL_GetTick() * 1103515245 + 12345;
}
```

### 時間依存の明示と抽象化

```c
// ✅ Good: 時間依存を注入可能に

typedef struct {
    uint32_t (*get_time_ms)(void);  // 時間取得関数を注入
    uint32_t interval_ms;
    uint32_t last_time_ms;
} periodic_task_t;

void periodic_task_init(
    periodic_task_t* task,
    uint32_t (*get_time_ms)(void),
    uint32_t interval_ms
) {
    task->get_time_ms = get_time_ms;
    task->interval_ms = interval_ms;
    task->last_time_ms = get_time_ms();
}

bool periodic_task_is_due(periodic_task_t* task) {
    uint32_t now = task->get_time_ms();
    if (now - task->last_time_ms >= task->interval_ms) {
        task->last_time_ms = now;
        return true;
    }
    return false;
}

// テスト用のモック時間関数
static uint32_t mock_time = 0;
uint32_t mock_get_time_ms(void) { return mock_time; }
void mock_advance_time_ms(uint32_t ms) { mock_time += ms; }

// テスト
void test_periodic_task(void) {
    periodic_task_t task;
    mock_time = 0;

    periodic_task_init(&task, mock_get_time_ms, 100);

    assert(!periodic_task_is_due(&task));

    mock_advance_time_ms(50);
    assert(!periodic_task_is_due(&task));

    mock_advance_time_ms(50);
    assert(periodic_task_is_due(&task));  // 100ms経過
}
```

---

## 8. Testability Design（テスト容易性設計）

### Dependency Injection（依存性注入）

```c
// ✅ Good: コンストラクタ注入パターン

// 依存関係のインターフェース定義
typedef struct {
    int (*read_temperature)(void);
    int (*read_humidity)(void);
} sensor_interface_t;

typedef struct {
    void (*send)(const char* data, size_t len);
} network_interface_t;

typedef struct {
    uint32_t (*get_time_ms)(void);
} time_interface_t;

// サービス構造体
typedef struct {
    const sensor_interface_t* sensors;
    const network_interface_t* network;
    const time_interface_t* time;
    uint32_t report_interval_ms;
} weather_reporter_t;

// 依存性を注入して初期化
void weather_reporter_init(
    weather_reporter_t* reporter,
    const sensor_interface_t* sensors,
    const network_interface_t* network,
    const time_interface_t* time
) {
    reporter->sensors = sensors;
    reporter->network = network;
    reporter->time = time;
    reporter->report_interval_ms = 60000;
}

// ビジネスロジック（依存性は注入されたものを使用）
void weather_reporter_tick(weather_reporter_t* reporter) {
    static uint32_t last_report_time = 0;
    uint32_t now = reporter->time->get_time_ms();

    if (now - last_report_time >= reporter->report_interval_ms) {
        int temp = reporter->sensors->read_temperature();
        int humidity = reporter->sensors->read_humidity();

        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer),
                          "{\"temp\":%d,\"humidity\":%d}", temp, humidity);

        reporter->network->send(buffer, len);
        last_report_time = now;
    }
}

// === テストコード ===

// モック実装
static int mock_temp = 25;
static int mock_humidity = 60;
static char mock_sent_data[256];
static uint32_t mock_time = 0;

int mock_read_temperature(void) { return mock_temp; }
int mock_read_humidity(void) { return mock_humidity; }
void mock_send(const char* data, size_t len) {
    memcpy(mock_sent_data, data, len);
}
uint32_t mock_get_time(void) { return mock_time; }

void test_weather_reporter(void) {
    // モックを注入
    sensor_interface_t mock_sensors = {
        .read_temperature = mock_read_temperature,
        .read_humidity = mock_read_humidity,
    };
    network_interface_t mock_network = {
        .send = mock_send,
    };
    time_interface_t mock_time_iface = {
        .get_time_ms = mock_get_time,
    };

    weather_reporter_t reporter;
    weather_reporter_init(&reporter, &mock_sensors, &mock_network, &mock_time_iface);
    reporter.report_interval_ms = 1000;

    // テスト: 時間経過前は送信しない
    mock_time = 500;
    weather_reporter_tick(&reporter);
    assert(mock_sent_data[0] == '\0');

    // テスト: 時間経過後は送信する
    mock_time = 1500;
    weather_reporter_tick(&reporter);
    assert(strstr(mock_sent_data, "\"temp\":25") != NULL);
}
```

### Seams（接合点）の設計

```c
// ✅ Good: リンク時の差し替え（Link-time Seam）

// production/hal_gpio.c
void hal_gpio_write(uint8_t pin, bool value) {
    GPIO->ODR = (GPIO->ODR & ~(1 << pin)) | (value << pin);
}

// test/mock_hal_gpio.c
static bool gpio_states[32];
void hal_gpio_write(uint8_t pin, bool value) {
    gpio_states[pin] = value;
}
bool mock_gpio_get_state(uint8_t pin) {
    return gpio_states[pin];
}

// ビルド時にリンクするファイルを切り替え
// make test    -> test/mock_hal_gpio.c をリンク
// make release -> production/hal_gpio.c をリンク
```

```c
// ✅ Good: プリプロセッサによる差し替え（Compile-time Seam）

#ifdef UNIT_TEST
    #define HAL_GPIO_WRITE(pin, val) mock_gpio_write(pin, val)
    #define HAL_DELAY_MS(ms)         mock_delay_ms(ms)
#else
    #define HAL_GPIO_WRITE(pin, val) hal_gpio_write(pin, val)
    #define HAL_DELAY_MS(ms)         hal_delay_ms(ms)
#endif

void led_blink(uint8_t pin, uint32_t duration_ms) {
    HAL_GPIO_WRITE(pin, true);
    HAL_DELAY_MS(duration_ms);
    HAL_GPIO_WRITE(pin, false);
}
```

---

## References

- [Embedded.com: Defensive Programming for Embedded Systems](https://medium.com/software-architecture-foundations/defensive-programmingfor-embedded-systems-c9b12c380996)
- [Embedded.com: Contracts Put the "I Do" in Code](https://www.embedded.com/contracts-put-the-i-do-in-code)
- [Jacob Beningo: Creating a Hardware Abstraction Layer (HAL) in C](https://www.embeddedrelated.com/showarticle/1596.php)
- [Embedded Artistry: Single Responsibility Principle](https://embeddedartistry.com/fieldmanual-terms/single-responsibility-principle/)
- [Embedded Artistry: Separation of Concerns](https://embeddedartistry.com/fieldmanual-terms/separation-of-concerns/)
- [Google Cloud: Design for Graceful Degradation](https://cloud.google.com/architecture/framework/reliability/graceful-degradation)
- [ETAS: Determinism in Embedded Real-Time Systems](https://edms.etas.com/explanations/determinism.html)
- [Lance Harvie: How to Achieve Deterministic Behavior in Real-Time Systems](https://www.embeddedrelated.com/showarticle/1742.php)
- [GoodByte: Embedded Software vs Unit Testing - Mocking and Dependency Injection](https://www.goodbyte.pl/en/embedded_blog/embedded-software-vs-unit-testing-mocking-and-dependency-injection)
- [CMU SEI: API Security through Contract-Driven Programming](https://www.sei.cmu.edu/blog/api-security-through-contract-driven-programming/)
- [AdaCore: Design by Contracts](https://learn.adacore.com/courses/intro-to-ada/chapters/contracts.html)
- [Wikipedia: Design by Contract](https://en.wikipedia.org/wiki/Design_by_contract)
- [Wikipedia: Single-responsibility Principle](https://en.wikipedia.org/wiki/Single-responsibility_principle)
