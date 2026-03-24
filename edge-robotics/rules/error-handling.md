# Error Handling Rules

組込/エッジ開発および安全重視システムにおけるエラーハンドリングルール。

本ドキュメントはハードウェア・ソフトウェア非依存であり、特定のMCU、RTOS、フレームワークに依存しない汎用的なガイドラインを提供する。

---

## Pre-Commit Checklist

コミット前に必ず確認する:

### Error Propagation（エラー伝播）
- [ ] 全ての関数が適切なエラーコードを返している
- [ ] 回復可能エラーと致命的エラーを明確に区別している
- [ ] エラー発生時にリソースが正しく解放されている
- [ ] エラー情報がコールスタック上位へ適切に伝播している

### Fault Tolerance（耐障害性）
- [ ] 一時的障害に対するリトライ戦略が実装されている
- [ ] リトライ回数に上限が設定されている
- [ ] 指数バックオフまたは適切な待機戦略を使用している

### Error Classification（エラー分類）
- [ ] エラーレベルが適切に分類されている（Fatal/Error/Warning/Info）
- [ ] 各レベルに応じた適切なアクションが定義されている

### Logging（ロギング）
- [ ] 本番環境でデバッグログが無効化されている
- [ ] ログにセンシティブ情報が含まれていない
- [ ] ログレベルが適切に設定されている

### Assert Usage（アサート使用）
- [ ] 本番ビルドでのアサート動作が定義されている
- [ ] アサートが防御的プログラミングの代替として誤用されていない

---

## 1. Error Propagation（エラー伝播）

### 基本原則

エラーはコールスタックを上位へ伝播させ、適切なレイヤーで処理する。

#### Recoverable vs Fatal Errors（回復可能 vs 致命的エラー）

| 種別 | 定義 | 対処 |
|------|------|------|
| **Recoverable** | リトライや代替処理で回復可能 | リトライ、フォールバック、ログ記録 |
| **Fatal** | システムの整合性が保証できない | 安全な状態へ移行、システム再起動 |

### ✅ Good: 統一的なエラー型と伝播

```c
// 統一的なエラー型定義
typedef enum {
    ERR_OK = 0,

    // Recoverable errors (1-99)
    ERR_TIMEOUT = 1,
    ERR_BUSY = 2,
    ERR_RETRY = 3,

    // Fatal errors (100+)
    ERR_FATAL_BASE = 100,
    ERR_MEMORY_CORRUPTION = 100,
    ERR_HARDWARE_FAULT = 101,
    ERR_STACK_OVERFLOW = 102,
} error_t;

// エラーが致命的かどうかを判定
static inline bool is_fatal_error(error_t err) {
    return err >= ERR_FATAL_BASE;
}

// 呼び出し元へエラーを伝播
error_t process_sensor_data(sensor_t* sensor) {
    error_t err;

    err = sensor_read(sensor);
    if (err != ERR_OK) {
        // エラー情報を付加して伝播
        log_error("sensor_read failed: %d", err);
        return err;
    }

    err = data_validate(sensor->data);
    if (err != ERR_OK) {
        return err;
    }

    return ERR_OK;
}
```

### ❌ Bad: エラーの黙殺・不適切な処理

```c
// エラーを無視
void bad_process(void) {
    sensor_read(sensor);  // 戻り値無視
    // ...処理続行...
}

// エラーを握りつぶす
int bad_wrapper(void) {
    int result = risky_operation();
    if (result < 0) {
        return 0;  // エラーを隠蔽
    }
    return result;
}
```

### リソース解放パターン

エラー発生時に確保したリソースを適切に解放する。

```c
// ✅ RAII風パターン（C言語版）
error_t safe_operation(void) {
    error_t err = ERR_OK;
    resource_t* res1 = NULL;
    resource_t* res2 = NULL;

    res1 = resource_acquire_type1();
    if (res1 == NULL) {
        err = ERR_NO_MEMORY;
        goto cleanup;
    }

    res2 = resource_acquire_type2();
    if (res2 == NULL) {
        err = ERR_NO_MEMORY;
        goto cleanup;
    }

    err = do_work(res1, res2);

cleanup:
    if (res2) resource_release_type2(res2);
    if (res1) resource_release_type1(res1);
    return err;
}
```

---

## 2. Fault Tolerance（耐障害性）

### リトライ戦略

一時的障害に対しては、適切なリトライ戦略を実装する。

### ✅ Good: 指数バックオフ付きリトライ

```c
typedef struct {
    uint8_t max_retries;
    uint32_t initial_delay_ms;
    uint32_t max_delay_ms;
    float backoff_multiplier;
} retry_config_t;

static const retry_config_t DEFAULT_RETRY = {
    .max_retries = 3,
    .initial_delay_ms = 100,
    .max_delay_ms = 5000,
    .backoff_multiplier = 2.0f,
};

error_t retry_with_backoff(
    error_t (*operation)(void* ctx),
    void* ctx,
    const retry_config_t* config
) {
    uint32_t delay_ms = config->initial_delay_ms;

    for (uint8_t attempt = 0; attempt <= config->max_retries; attempt++) {
        error_t err = operation(ctx);

        if (err == ERR_OK) {
            return ERR_OK;
        }

        if (is_fatal_error(err)) {
            // 致命的エラーはリトライしない
            return err;
        }

        if (attempt < config->max_retries) {
            log_warning("Operation failed (attempt %d/%d), retrying in %lu ms",
                        attempt + 1, config->max_retries, delay_ms);
            delay_ms_func(delay_ms);
            delay_ms = MIN(delay_ms * config->backoff_multiplier,
                          config->max_delay_ms);
        }
    }

    return ERR_MAX_RETRIES_EXCEEDED;
}
```

### ❌ Bad: 無限リトライ・固定リトライ

```c
// 無限リトライ（システムがハングする可能性）
while (operation() != SUCCESS) {
    // 永遠にリトライ
}

// 待機なしの即時リトライ（リソース浪費）
for (int i = 0; i < 10; i++) {
    if (operation() == SUCCESS) break;
    // 待機なし
}
```

### 一時的障害の検出

```c
// 一時的障害かどうかを判定
static bool is_transient_error(error_t err) {
    switch (err) {
        case ERR_TIMEOUT:
        case ERR_BUSY:
        case ERR_RESOURCE_UNAVAILABLE:
        case ERR_COMMUNICATION_ERROR:
            return true;
        default:
            return false;
    }
}
```

---

## 3. Error Classification（エラー分類）

### エラーレベル定義

| Level | 説明 | アクション |
|-------|------|-----------|
| **FATAL** | システム継続不能 | 安全状態へ移行、再起動 |
| **ERROR** | 機能障害だが継続可能 | エラーログ、通知、復旧試行 |
| **WARNING** | 潜在的問題 | 警告ログ、監視継続 |
| **INFO** | 正常な状態変化 | 情報ログ（本番環境では抑制） |
| **DEBUG** | 開発用詳細情報 | 開発環境のみ |

### ✅ Good: レベルに応じた適切なアクション

```c
typedef enum {
    SEVERITY_DEBUG,
    SEVERITY_INFO,
    SEVERITY_WARNING,
    SEVERITY_ERROR,
    SEVERITY_FATAL,
} severity_t;

typedef struct {
    error_t code;
    severity_t severity;
    const char* message;
    void (*handler)(error_t code);
} error_action_t;

// エラーテーブル：コードと対応アクションのマッピング
static const error_action_t error_actions[] = {
    {ERR_SENSOR_OFFLINE, SEVERITY_WARNING, "Sensor offline", handle_sensor_offline},
    {ERR_COMM_TIMEOUT, SEVERITY_ERROR, "Communication timeout", handle_comm_timeout},
    {ERR_MEMORY_CORRUPTION, SEVERITY_FATAL, "Memory corruption detected", handle_fatal},
    // ...
};

void process_error(error_t err) {
    const error_action_t* action = find_error_action(err);

    if (action == NULL) {
        log_error("Unknown error: %d", err);
        return;
    }

    switch (action->severity) {
        case SEVERITY_FATAL:
            log_fatal("%s (code: %d)", action->message, err);
            enter_safe_state();
            if (action->handler) action->handler(err);
            system_reset();
            break;

        case SEVERITY_ERROR:
            log_error("%s (code: %d)", action->message, err);
            increment_error_counter(err);
            if (action->handler) action->handler(err);
            break;

        case SEVERITY_WARNING:
            log_warning("%s (code: %d)", action->message, err);
            if (action->handler) action->handler(err);
            break;

        default:
            break;
    }
}
```

---

## 4. Error Codes and Messages（エラーコードとメッセージ）

### 体系的エラーコード設計

```c
// ✅ モジュール別エラーコードスキーム
// 形式: 0xMMEE (MM: モジュールID, EE: エラー番号)

#define MODULE_CORE     0x00
#define MODULE_SENSOR   0x01
#define MODULE_COMM     0x02
#define MODULE_STORAGE  0x03

#define MAKE_ERROR(module, code) (((module) << 8) | (code))
#define GET_MODULE(err) (((err) >> 8) & 0xFF)
#define GET_CODE(err) ((err) & 0xFF)

typedef enum {
    // Core errors (0x00XX)
    ERR_CORE_OK = MAKE_ERROR(MODULE_CORE, 0x00),
    ERR_CORE_INVALID_PARAM = MAKE_ERROR(MODULE_CORE, 0x01),
    ERR_CORE_NO_MEMORY = MAKE_ERROR(MODULE_CORE, 0x02),

    // Sensor errors (0x01XX)
    ERR_SENSOR_NOT_FOUND = MAKE_ERROR(MODULE_SENSOR, 0x01),
    ERR_SENSOR_READ_FAIL = MAKE_ERROR(MODULE_SENSOR, 0x02),
    ERR_SENSOR_CALIBRATION = MAKE_ERROR(MODULE_SENSOR, 0x03),

    // Communication errors (0x02XX)
    ERR_COMM_TIMEOUT = MAKE_ERROR(MODULE_COMM, 0x01),
    ERR_COMM_CRC_ERROR = MAKE_ERROR(MODULE_COMM, 0x02),
    ERR_COMM_DISCONNECTED = MAKE_ERROR(MODULE_COMM, 0x03),
} system_error_t;
```

### エラーメッセージ管理

```c
// ✅ エラーコードと人間可読メッセージのマッピング
typedef struct {
    system_error_t code;
    const char* name;
    const char* description;
} error_info_t;

static const error_info_t error_table[] = {
    {ERR_CORE_OK, "OK", "Operation completed successfully"},
    {ERR_CORE_INVALID_PARAM, "INVALID_PARAM", "Invalid parameter provided"},
    {ERR_SENSOR_NOT_FOUND, "SENSOR_NOT_FOUND", "Requested sensor not found"},
    {ERR_COMM_TIMEOUT, "COMM_TIMEOUT", "Communication timed out"},
    // ...
    {0, NULL, NULL}  // 終端
};

const char* error_to_string(system_error_t err) {
    for (const error_info_t* info = error_table; info->name; info++) {
        if (info->code == err) {
            return info->name;
        }
    }
    return "UNKNOWN";
}
```

### ❌ Bad: マジックナンバー・意味不明なエラー

```c
// マジックナンバー
if (result == -1) { /* ??? */ }
if (result == 0x1234) { /* ??? */ }

// 重複するエラーコード
#define ERR_FAIL 1  // センサーモジュール
#define ERR_FAIL 1  // 通信モジュール（衝突！）
```

---

## 5. Logging Policy（ログポリシー）

### ログレベル運用

| 環境 | 最小ログレベル | 説明 |
|------|---------------|------|
| 開発/デバッグ | DEBUG | 全てのログを出力 |
| テスト | INFO | INFO以上を出力 |
| 本番 | WARNING | WARNING以上のみ出力 |
| リソース制約環境 | ERROR | ERROR以上のみ出力 |

### ✅ Good: 構造化ログ・条件付きコンパイル

```c
// ログレベル定義
typedef enum {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,
    LOG_FATAL = 4,
} log_level_t;

// ビルド時のログレベル設定
#ifndef LOG_LEVEL
    #ifdef NDEBUG
        #define LOG_LEVEL LOG_WARNING
    #else
        #define LOG_LEVEL LOG_DEBUG
    #endif
#endif

// 条件付きログマクロ
#define LOG(level, fmt, ...) \
    do { \
        if ((level) >= LOG_LEVEL) { \
            log_output(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define LOG_DEBUG(fmt, ...) LOG(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) LOG(LOG_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) LOG(LOG_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) LOG(LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) LOG(LOG_FATAL, fmt, ##__VA_ARGS__)

// 構造化ログ出力
void log_output(log_level_t level, const char* file, int line,
                const char* fmt, ...) {
    // タイムスタンプ取得
    uint32_t timestamp = get_system_time_ms();

    // フォーマット: [TIMESTAMP][LEVEL][FILE:LINE] message
    printf("[%010lu][%s][%s:%d] ",
           timestamp,
           level_to_string(level),
           file,
           line);

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    printf("\n");
}
```

### 循環バッファによるログ管理

```c
// ✅ メモリ制約環境向け循環バッファログ
typedef struct {
    char entries[LOG_BUFFER_SIZE][LOG_ENTRY_MAX_LEN];
    size_t head;
    size_t count;
} log_buffer_t;

void log_to_buffer(log_buffer_t* buf, const char* message) {
    strncpy(buf->entries[buf->head], message, LOG_ENTRY_MAX_LEN - 1);
    buf->entries[buf->head][LOG_ENTRY_MAX_LEN - 1] = '\0';
    buf->head = (buf->head + 1) % LOG_BUFFER_SIZE;
    if (buf->count < LOG_BUFFER_SIZE) {
        buf->count++;
    }
}
```

### ❌ Bad: センシティブ情報のログ出力

```c
// パスワードやトークンをログに出力
LOG_DEBUG("Connecting with password: %s", password);

// メモリアドレスをログに出力（セキュリティリスク）
LOG_DEBUG("Buffer at %p", secret_buffer);

// 本番環境でDEBUGログを有効化
#define LOG_LEVEL LOG_DEBUG  // 本番では危険！
```

---

## 6. Assert Usage（アサートの使用）

### アサートの目的

- 開発中のバグを早期発見
- 前提条件（precondition）の明示化
- 不変条件（invariant）の検証

### ✅ Good: 開発・本番で適切なアサート動作

```c
// カスタムアサートマクロ
#ifdef NDEBUG
    // 本番ビルド: アサート失敗をログに記録し、安全な状態へ
    #define ASSERT(expr) \
        do { \
            if (!(expr)) { \
                log_fatal("ASSERT FAILED: %s at %s:%d", \
                          #expr, __FILE__, __LINE__); \
                save_crash_dump(); \
                enter_safe_state(); \
                system_reset(); \
            } \
        } while(0)
#else
    // デバッグビルド: 即座に停止
    #define ASSERT(expr) \
        do { \
            if (!(expr)) { \
                log_fatal("ASSERT FAILED: %s at %s:%d", \
                          #expr, __FILE__, __LINE__); \
                __builtin_trap(); \
            } \
        } while(0)
#endif

// 前提条件チェック
void motor_set_speed(motor_t* motor, int speed) {
    ASSERT(motor != NULL);
    ASSERT(speed >= SPEED_MIN && speed <= SPEED_MAX);
    // ...
}

// 不変条件チェック
void queue_push(queue_t* q, void* item) {
    ASSERT(q != NULL);
    ASSERT(q->count <= q->capacity);  // 不変条件
    // ...
    ASSERT(q->count <= q->capacity);  // 操作後も維持
}
```

### Defensive Programming vs Preemptive Programming

```c
// ❌ Defensive: エラーを隠蔽する可能性
void bad_defensive(int* ptr) {
    if (ptr == NULL) {
        return;  // 静かに失敗 - バグを隠す
    }
    *ptr = 42;
}

// ✅ Preemptive: 問題を早期に検出
void good_preemptive(int* ptr) {
    ASSERT(ptr != NULL);  // NULL が来たらバグ
    *ptr = 42;
}

// ✅ 正当な入力検証（外部入力）
error_t validate_user_input(const char* input) {
    if (input == NULL) {
        return ERR_INVALID_PARAM;  // 外部入力は検証が必要
    }
    // ...
    return ERR_OK;
}
```

### ❌ Bad: アサートの誤用

```c
// 副作用のある式をアサートに使用（NDEBUGで消える）
ASSERT(initialize_hardware() == SUCCESS);

// 外部入力の検証にアサートを使用
ASSERT(user_input != NULL);  // 外部入力は検証すべき

// アサートを完全に無効化
#define ASSERT(x) ((void)0)  // 本番で全て無視は危険
```

---

## 7. Exception-like Patterns in C（C言語での例外的パターン）

### setjmp/longjmpの考慮事項

setjmp/longjmpは非局所的ジャンプを提供するが、使用には注意が必要。

### ✅ Good: 限定的・安全な使用

```c
#include <setjmp.h>

// スレッドローカルなジャンプバッファ
static __thread jmp_buf error_jmp_buf;
static __thread bool jmp_buf_valid = false;

// エラー発生時のジャンプ
#define THROW(err) \
    do { \
        if (jmp_buf_valid) { \
            longjmp(error_jmp_buf, (int)(err)); \
        } else { \
            handle_unhandled_error(err); \
        } \
    } while(0)

// Try-Catch風パターン
error_t protected_operation(void) {
    volatile error_t result = ERR_OK;  // volatileが必須

    jmp_buf_valid = true;
    int jump_result = setjmp(error_jmp_buf);

    if (jump_result == 0) {
        // 通常の実行パス
        risky_operation();
        result = ERR_OK;
    } else {
        // エラー発生時のパス
        result = (error_t)jump_result;
    }

    jmp_buf_valid = false;
    return result;
}
```

### Cleanup Handler パターン

```c
// ✅ クリーンアップ登録パターン
typedef void (*cleanup_func_t)(void* ctx);

typedef struct cleanup_entry {
    cleanup_func_t func;
    void* ctx;
    struct cleanup_entry* next;
} cleanup_entry_t;

static __thread cleanup_entry_t* cleanup_stack = NULL;

void register_cleanup(cleanup_func_t func, void* ctx) {
    cleanup_entry_t* entry = malloc(sizeof(cleanup_entry_t));
    entry->func = func;
    entry->ctx = ctx;
    entry->next = cleanup_stack;
    cleanup_stack = entry;
}

void run_cleanup(void) {
    while (cleanup_stack) {
        cleanup_entry_t* entry = cleanup_stack;
        cleanup_stack = entry->next;
        entry->func(entry->ctx);
        free(entry);
    }
}
```

### ❌ Bad: setjmp/longjmpの危険な使用

```c
// setjmpを呼んだ関数からリターンした後のlongjmp（未定義動作）
jmp_buf buf;
void setup(void) {
    setjmp(buf);  // この関数からリターン後、bufは無効
}
void trigger(void) {
    longjmp(buf, 1);  // 未定義動作！
}

// volatileなしでsetjmp後に変更される変数
void bad_example(void) {
    int value = 0;  // volatileが必要
    if (setjmp(buf) == 0) {
        value = 42;
        longjmp(buf, 1);
    }
    // valueの値は不定
}

// リソースリークの可能性
void leaky(void) {
    void* mem = malloc(100);
    if (setjmp(buf) == 0) {
        risky_op();  // longjmpされるとmemがリーク
    }
}
```

---

## 8. Graceful Degradation（グレースフルデグラデーション）

### 設計原則

- 一部の障害でシステム全体を停止させない
- 縮退運転モードを設計段階で定義
- 重要度に基づいた機能の優先順位付け

### ✅ Good: 縮退運転モードの実装

```c
typedef enum {
    MODE_FULL,           // 全機能動作
    MODE_DEGRADED_L1,    // 軽度縮退
    MODE_DEGRADED_L2,    // 中度縮退
    MODE_SAFE,           // 最小安全モード
    MODE_EMERGENCY,      // 緊急停止
} operation_mode_t;

typedef struct {
    bool primary_sensor_ok;
    bool backup_sensor_ok;
    bool communication_ok;
    bool storage_ok;
} system_health_t;

operation_mode_t determine_operation_mode(const system_health_t* health) {
    // 全て正常
    if (health->primary_sensor_ok &&
        health->communication_ok &&
        health->storage_ok) {
        return MODE_FULL;
    }

    // プライマリセンサー障害、バックアップで継続
    if (!health->primary_sensor_ok && health->backup_sensor_ok) {
        log_warning("Primary sensor failed, using backup");
        return MODE_DEGRADED_L1;
    }

    // 通信障害、ローカル動作
    if (!health->communication_ok && health->primary_sensor_ok) {
        log_warning("Communication failed, operating locally");
        return MODE_DEGRADED_L2;
    }

    // バックアップセンサーも利用可能
    if (health->backup_sensor_ok) {
        return MODE_SAFE;
    }

    // 最低限の安全確保のみ
    return MODE_EMERGENCY;
}
```

### フォールバック戦略

```c
// ✅ フォールバックチェーン
typedef error_t (*data_source_func_t)(data_t* out);

typedef struct {
    const char* name;
    data_source_func_t func;
} data_source_t;

static const data_source_t data_sources[] = {
    {"primary_sensor", read_primary_sensor},
    {"backup_sensor", read_backup_sensor},
    {"cached_data", read_cached_data},
    {"default_value", get_default_value},
    {NULL, NULL}
};

error_t get_data_with_fallback(data_t* out) {
    for (const data_source_t* src = data_sources; src->func; src++) {
        error_t err = src->func(out);
        if (err == ERR_OK) {
            if (src != &data_sources[0]) {
                log_warning("Using fallback source: %s", src->name);
            }
            return ERR_OK;
        }
        log_warning("%s failed: %d", src->name, err);
    }

    log_error("All data sources failed");
    return ERR_ALL_SOURCES_FAILED;
}
```

### 部分障害の隔離

```c
// ✅ サブシステムの障害を隔離
typedef struct {
    const char* name;
    bool enabled;
    error_t (*init)(void);
    error_t (*process)(void);
    void (*shutdown)(void);
} subsystem_t;

void system_main_loop(subsystem_t* subsystems, size_t count) {
    while (system_running) {
        for (size_t i = 0; i < count; i++) {
            if (!subsystems[i].enabled) continue;

            error_t err = subsystems[i].process();

            if (is_fatal_error(err)) {
                log_error("Subsystem %s fatal error: %d, disabling",
                          subsystems[i].name, err);
                subsystems[i].shutdown();
                subsystems[i].enabled = false;

                // 重要なサブシステムなら全体を停止
                if (is_critical_subsystem(&subsystems[i])) {
                    enter_safe_state();
                    break;
                }
            }
        }
    }
}
```

---

## References

本ドキュメントは以下の資料を参考に作成した:

- [MISRA C Guidelines](https://www.misra.org.uk/) - Safety-critical C programming guidelines
- [ISO 26262](https://www.iso.org/standard/68383.html) - Functional safety for automotive
- [IEC 61508](https://www.iec.ch/functionalsafety) - Functional safety of E/E/PE systems
- [Using Asserts in Embedded Systems](https://interrupt.memfault.com/blog/asserts-in-embedded-systems) - Memfault
- [Design by Contract for Embedded Software](https://www.state-machine.com/dbc) - Quantum Leaps
- [Fault Tolerance in Embedded Systems Survey (2024)](https://arxiv.org/abs/2404.10509) - arXiv
- [Graceful Degradation in Embedded Systems](https://www.sciencedirect.com/topics/computer-science/graceful-degradation) - ScienceDirect
