# State Machine Design & Concurrency Safety Rules

組込/エッジ開発におけるステートマシン設計と並行性安全のルール。
ハードウェア/ソフトウェア非依存の汎用ガイドライン。

## Pre-Commit Checklist

コミット前に必ず確認する:

### State Machine
- [ ] 全ての状態遷移がテーブルまたは明示的なロジックで定義されている
- [ ] 無効な状態遷移が防止されている（default case または assertion）
- [ ] 各状態のentry/exit処理が定義されている
- [ ] 状態遷移ログが実装されている（デバッグ用）

### Initialization & Shutdown
- [ ] 初期化順序が依存関係に基づいている
- [ ] シャットダウン時のリソース解放順序が初期化の逆順
- [ ] 遅延初期化が必要な箇所で適切に実装されている
- [ ] リセット種別（soft/hard/watchdog）に応じた処理が定義されている

### Concurrency Safety
- [ ] 共有リソースへのアクセスがクリティカルセクションで保護されている
- [ ] アトミック操作が必要な箇所で適切に使用されている
- [ ] ロック取得順序が一貫している（デッドロック防止）
- [ ] タイムアウト付きロックが適切に使用されている

### Event-Driven Design
- [ ] ISR（割り込みサービスルーチン）は最小限の処理のみ
- [ ] イベントキューのオーバーフロー対策がある
- [ ] ポーリング vs 割り込みの選択が適切

---

## 1. State Machine Design（ステートマシン設計）

### 1.1 Explicit State Transitions（明示的な状態遷移）

**Principle**: 状態遷移は常に明示的に定義し、暗黙的な遷移を排除する。

#### ✅ Good: Table-Driven State Machine

```c
typedef enum {
    STATE_IDLE,
    STATE_INITIALIZING,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_SHUTDOWN,
    STATE_COUNT
} system_state_t;

typedef enum {
    EVENT_START,
    EVENT_INIT_COMPLETE,
    EVENT_ERROR,
    EVENT_STOP,
    EVENT_RECOVER,
    EVENT_COUNT
} system_event_t;

typedef struct {
    system_state_t next_state;
    void (*action)(void* context);
    bool (*guard)(void* context);  // 遷移条件
} transition_t;

// 状態遷移テーブル - 全ての遷移を明示的に定義
static const transition_t state_table[STATE_COUNT][EVENT_COUNT] = {
    [STATE_IDLE] = {
        [EVENT_START] = {STATE_INITIALIZING, action_start, NULL},
        // 他のイベントは無視（next_state = STATE_IDLE）
    },
    [STATE_INITIALIZING] = {
        [EVENT_INIT_COMPLETE] = {STATE_RUNNING, action_init_done, NULL},
        [EVENT_ERROR]         = {STATE_ERROR, action_handle_error, NULL},
        [EVENT_STOP]          = {STATE_SHUTDOWN, action_abort_init, NULL},
    },
    [STATE_RUNNING] = {
        [EVENT_ERROR] = {STATE_ERROR, action_handle_error, NULL},
        [EVENT_STOP]  = {STATE_SHUTDOWN, action_begin_shutdown, NULL},
    },
    [STATE_ERROR] = {
        [EVENT_RECOVER] = {STATE_IDLE, action_reset, guard_can_recover},
        [EVENT_STOP]    = {STATE_SHUTDOWN, action_emergency_stop, NULL},
    },
    [STATE_SHUTDOWN] = {
        // 終了状態 - 遷移なし
    },
};
```

#### ❌ Bad: Implicit/Scattered State Transitions

```c
// 状態遷移がコード全体に散らばっている
void process_data(void) {
    if (some_condition) {
        current_state = STATE_RUNNING;  // 遷移条件が不明確
    }
    // ...
    if (error_flag) {
        current_state = STATE_ERROR;  // どこからでも遷移可能
    }
}

void handle_event(void) {
    if (current_state == STATE_IDLE && button_pressed) {
        current_state = STATE_RUNNING;  // ガード条件が一貫していない
    }
}
```

### 1.2 Invalid Transition Prevention（無効遷移の防止）

#### ✅ Good: Validation with Logging

```c
typedef struct {
    system_state_t current_state;
    void (*on_invalid_transition)(system_state_t from, system_event_t event);
} state_machine_t;

bool state_machine_dispatch(state_machine_t* sm, system_event_t event, void* context) {
    if (event >= EVENT_COUNT) {
        log_error("Invalid event: %d", event);
        return false;
    }

    transition_t t = state_table[sm->current_state][event];

    // 遷移が定義されていない場合
    if (t.next_state == sm->current_state && t.action == NULL) {
        if (sm->on_invalid_transition) {
            sm->on_invalid_transition(sm->current_state, event);
        }
        log_warning("Ignored event %d in state %d", event, sm->current_state);
        return false;
    }

    // ガード条件チェック
    if (t.guard && !t.guard(context)) {
        log_debug("Guard prevented transition: %d -> %d",
                  sm->current_state, t.next_state);
        return false;
    }

    // 状態遷移を実行
    system_state_t prev_state = sm->current_state;
    if (t.action) {
        t.action(context);
    }
    sm->current_state = t.next_state;

    log_info("State transition: %d -> %d (event: %d)",
             prev_state, sm->current_state, event);
    return true;
}
```

#### ❌ Bad: No Validation

```c
void handle_event(int event) {
    switch (current_state) {
        case STATE_IDLE:
            if (event == EVENT_START) {
                current_state = STATE_RUNNING;
            }
            // 他のイベントは何も起きない（暗黙の無視）
            break;
        // ... no default case, no logging
    }
}
```

---

## 2. Initialization Order（初期化順序）

### 2.1 Dependency-Based Ordering（依存関係に基づく順序）

**Principle**: 初期化は依存関係を解決する順序で行い、依存関係を明示的に宣言する。

#### ✅ Good: Explicit Dependency Declaration

```c
typedef struct {
    const char* name;
    int (*init)(void);
    void (*deinit)(void);
    const char** dependencies;  // NULL terminated
    bool initialized;
} module_t;

// 依存関係を明示的に宣言
static const char* logger_deps[] = {NULL};  // 依存なし
static const char* storage_deps[] = {"logger", NULL};
static const char* network_deps[] = {"logger", "storage", NULL};
static const char* app_deps[] = {"logger", "storage", "network", NULL};

static module_t modules[] = {
    {"logger",  logger_init,  logger_deinit,  logger_deps,  false},
    {"storage", storage_init, storage_deinit, storage_deps, false},
    {"network", network_init, network_deinit, network_deps, false},
    {"app",     app_init,     app_deinit,     app_deps,     false},
    {NULL, NULL, NULL, NULL, false}
};

// 依存関係を解決して初期化
int init_module(const char* name) {
    module_t* mod = find_module(name);
    if (!mod || mod->initialized) return 0;

    // 依存モジュールを先に初期化
    for (const char** dep = mod->dependencies; *dep; dep++) {
        int result = init_module(*dep);
        if (result != 0) {
            log_error("Failed to init dependency: %s", *dep);
            return result;
        }
    }

    log_info("Initializing module: %s", mod->name);
    int result = mod->init();
    if (result == 0) {
        mod->initialized = true;
    }
    return result;
}
```

#### ❌ Bad: Hard-coded Order Without Documentation

```c
void system_init(void) {
    init_hardware();   // なぜこの順序？
    init_drivers();    // 依存関係不明
    init_services();   // 失敗時の処理なし
    init_app();
}
```

### 2.2 Lazy Initialization（遅延初期化）

**Principle**: リソースが最初に必要になるまで初期化を遅延させる。

#### ✅ Good: Thread-Safe Lazy Initialization

```c
typedef struct {
    bool initialized;
    bool init_in_progress;
    // ... resource data
} expensive_resource_t;

static expensive_resource_t resource = {0};
static mutex_t resource_mutex;

expensive_resource_t* get_resource(void) {
    // Fast path: already initialized
    if (resource.initialized) {
        return &resource;
    }

    // Slow path: needs initialization
    mutex_lock(&resource_mutex);

    // Double-check after acquiring lock
    if (!resource.initialized) {
        if (resource.init_in_progress) {
            mutex_unlock(&resource_mutex);
            return NULL;  // Initialization in progress by another thread
        }

        resource.init_in_progress = true;

        int result = initialize_expensive_resource(&resource);
        if (result == 0) {
            resource.initialized = true;
        }

        resource.init_in_progress = false;
    }

    mutex_unlock(&resource_mutex);
    return resource.initialized ? &resource : NULL;
}
```

---

## 3. Shutdown Sequence（シャットダウンシーケンス）

### 3.1 Resource Release Order（リソース解放順序）

**Principle**: シャットダウンは初期化の逆順で行い、依存されているリソースを最後に解放する。

#### ✅ Good: LIFO (Last In, First Out) Shutdown

```c
typedef struct {
    void (*cleanup)(void);
    const char* name;
} cleanup_handler_t;

#define MAX_CLEANUP_HANDLERS 32
static cleanup_handler_t cleanup_stack[MAX_CLEANUP_HANDLERS];
static int cleanup_count = 0;

// 初期化成功時にクリーンアップハンドラを登録
void register_cleanup(void (*handler)(void), const char* name) {
    if (cleanup_count < MAX_CLEANUP_HANDLERS) {
        cleanup_stack[cleanup_count].cleanup = handler;
        cleanup_stack[cleanup_count].name = name;
        cleanup_count++;
    }
}

// シャットダウン時に逆順でクリーンアップ
void system_shutdown(void) {
    log_info("Beginning system shutdown...");

    // 新しいリクエストの受付を停止
    stop_accepting_requests();

    // アクティブなタスクの完了を待機（タイムアウト付き）
    wait_for_active_tasks(SHUTDOWN_TIMEOUT_MS);

    // 逆順でリソースを解放
    while (cleanup_count > 0) {
        cleanup_count--;
        log_info("Cleaning up: %s", cleanup_stack[cleanup_count].name);
        cleanup_stack[cleanup_count].cleanup();
    }

    log_info("System shutdown complete");
}
```

### 3.2 Graceful Shutdown（グレースフルシャットダウン）

#### ✅ Good: Timeout-Based Graceful Shutdown

```c
typedef struct {
    volatile bool shutdown_requested;
    volatile int active_tasks;
    semaphore_t tasks_complete;
} shutdown_context_t;

void request_shutdown(shutdown_context_t* ctx) {
    ctx->shutdown_requested = true;
    log_info("Shutdown requested, waiting for %d active tasks",
             ctx->active_tasks);
}

void task_entry(shutdown_context_t* ctx) {
    atomic_increment(&ctx->active_tasks);

    while (!ctx->shutdown_requested) {
        // Process work
        process_next_item();
    }

    // Cleanup task resources
    cleanup_task_resources();

    if (atomic_decrement(&ctx->active_tasks) == 0) {
        semaphore_signal(&ctx->tasks_complete);
    }
}

int graceful_shutdown(shutdown_context_t* ctx, uint32_t timeout_ms) {
    request_shutdown(ctx);

    // Wait for all tasks with timeout
    int result = semaphore_wait_timeout(&ctx->tasks_complete, timeout_ms);

    if (result == TIMEOUT) {
        log_warning("Graceful shutdown timeout, %d tasks still active",
                    ctx->active_tasks);
        return -1;  // Caller may decide to force shutdown
    }

    return 0;
}
```

---

## 4. Reset Handling（リセット処理）

### 4.1 Reset Type Differentiation（リセット種別の区別）

**Principle**: リセット原因を識別し、適切な初期化処理を選択する。

#### ✅ Good: Reset Reason Handling

```c
typedef enum {
    RESET_POWER_ON,       // 電源投入
    RESET_SOFT,           // ソフトウェアリセット
    RESET_WATCHDOG,       // ウォッチドッグタイムアウト
    RESET_BROWNOUT,       // 電圧低下
    RESET_EXTERNAL,       // 外部リセットピン
    RESET_UNKNOWN
} reset_reason_t;

// Platform-agnostic: 各プラットフォームで実装
reset_reason_t get_reset_reason(void);
void clear_reset_flags(void);

typedef struct {
    uint32_t total_resets;
    uint32_t watchdog_resets;
    uint32_t brownout_resets;
    uint32_t last_reset_timestamp;
    reset_reason_t last_reset_reason;
} reset_statistics_t;

static reset_statistics_t reset_stats;

void system_early_init(void) {
    reset_reason_t reason = get_reset_reason();

    // Log reset statistics
    reset_stats.total_resets++;
    reset_stats.last_reset_reason = reason;

    switch (reason) {
        case RESET_POWER_ON:
            log_info("Power-on reset - performing full initialization");
            perform_full_init();
            break;

        case RESET_SOFT:
            log_info("Soft reset - restoring saved state");
            restore_saved_state();
            perform_partial_init();
            break;

        case RESET_WATCHDOG:
            reset_stats.watchdog_resets++;
            log_error("Watchdog reset detected!");
            save_crash_dump();

            // 連続watchdog resetの検出
            if (reset_stats.watchdog_resets > MAX_CONSECUTIVE_WDT_RESETS) {
                enter_safe_mode();
            } else {
                perform_recovery_init();
            }
            break;

        case RESET_BROWNOUT:
            reset_stats.brownout_resets++;
            log_warning("Brownout reset - checking power stability");
            check_power_stability();
            perform_full_init();
            break;

        default:
            log_warning("Unknown reset reason: %d", reason);
            perform_full_init();
            break;
    }

    clear_reset_flags();
}
```

### 4.2 Watchdog Management（ウォッチドッグ管理）

#### ✅ Good: Multi-Task Watchdog Monitoring

```c
typedef struct {
    uint32_t task_id;
    uint32_t last_check_in;
    uint32_t timeout_ms;
    const char* task_name;
    bool enabled;
} task_watchdog_t;

#define MAX_MONITORED_TASKS 16
static task_watchdog_t monitored_tasks[MAX_MONITORED_TASKS];
static volatile uint32_t system_tick_ms;

void task_check_in(uint32_t task_id) {
    if (task_id < MAX_MONITORED_TASKS) {
        monitored_tasks[task_id].last_check_in = system_tick_ms;
    }
}

// Called from a high-priority timer or dedicated watchdog task
void watchdog_monitor(void) {
    bool all_tasks_healthy = true;

    for (int i = 0; i < MAX_MONITORED_TASKS; i++) {
        task_watchdog_t* tw = &monitored_tasks[i];
        if (!tw->enabled) continue;

        uint32_t elapsed = system_tick_ms - tw->last_check_in;
        if (elapsed > tw->timeout_ms) {
            log_error("Task '%s' (id=%d) missed watchdog deadline by %lu ms",
                      tw->task_name, tw->task_id, elapsed - tw->timeout_ms);
            all_tasks_healthy = false;

            // Record which task failed for post-reset analysis
            save_failed_task_info(i);
        }
    }

    if (all_tasks_healthy) {
        hardware_watchdog_kick();  // Platform-specific
    }
    // If not healthy, let hardware watchdog expire
}
```

---

## 5. Concurrency Safety（並行性安全）

### 5.1 Race Condition Prevention（レースコンディション防止）

**Principle**: 共有データへのアクセスは必ず同期プリミティブで保護する。

#### ✅ Good: Protected Shared Resource Access

```c
typedef struct {
    int value;
    mutex_t mutex;
} shared_counter_t;

void counter_increment(shared_counter_t* counter) {
    mutex_lock(&counter->mutex);
    counter->value++;
    mutex_unlock(&counter->mutex);
}

int counter_get(shared_counter_t* counter) {
    mutex_lock(&counter->mutex);
    int value = counter->value;
    mutex_unlock(&counter->mutex);
    return value;
}
```

#### ❌ Bad: Unprotected Access

```c
static volatile int shared_counter = 0;

// Multiple tasks call this - RACE CONDITION!
void increment_counter(void) {
    shared_counter++;  // Read-modify-write is NOT atomic
}
```

### 5.2 Atomic Operations（アトミック操作）

#### ✅ Good: Using Atomic Primitives

```c
// Platform-agnostic atomic types (C11 or compiler-specific)
typedef struct {
    atomic_uint_fast32_t value;
} atomic_counter_t;

void atomic_counter_increment(atomic_counter_t* counter) {
    atomic_fetch_add(&counter->value, 1);
}

uint32_t atomic_counter_get(atomic_counter_t* counter) {
    return atomic_load(&counter->value);
}

// Compare-and-swap for lock-free algorithms
bool atomic_counter_compare_exchange(atomic_counter_t* counter,
                                     uint32_t* expected,
                                     uint32_t desired) {
    return atomic_compare_exchange_strong(&counter->value, expected, desired);
}
```

### 5.3 Critical Sections（クリティカルセクション）

#### ✅ Good: Minimal Critical Sections

```c
// Platform-agnostic critical section interface
typedef uint32_t irq_state_t;
irq_state_t enter_critical(void);
void exit_critical(irq_state_t state);

void update_shared_state(shared_state_t* state, new_data_t* data) {
    // Prepare data outside critical section
    new_data_t local_copy = *data;
    validate_data(&local_copy);

    // Minimal critical section - only the actual update
    irq_state_t irq_state = enter_critical();
    state->field1 = local_copy.field1;
    state->field2 = local_copy.field2;
    state->timestamp = get_timestamp();
    exit_critical(irq_state);

    // Post-processing outside critical section
    notify_listeners(state);
}
```

#### ❌ Bad: Over-Long Critical Sections

```c
void process_data(void) {
    disable_interrupts();  // BAD: Too long in critical section

    // Long-running operations block all interrupts!
    fetch_data_from_sensor();    // Could take ms
    process_complex_algorithm(); // Could take 100s of ms
    send_data_over_network();    // Could take seconds

    enable_interrupts();
}
```

---

## 6. Deadlock Prevention（デッドロック防止）

### 6.1 Lock Ordering（ロック順序）

**Principle**: 複数のロックを取得する場合、常に同じ順序で取得する。

#### ✅ Good: Consistent Lock Order by Address

```c
typedef struct {
    mutex_t mutex;
    int data;
} resource_t;

// Always acquire locks in address order to prevent deadlock
void transfer(resource_t* from, resource_t* to, int amount) {
    resource_t* first = (from < to) ? from : to;
    resource_t* second = (from < to) ? to : from;

    mutex_lock(&first->mutex);
    mutex_lock(&second->mutex);

    from->data -= amount;
    to->data += amount;

    mutex_unlock(&second->mutex);
    mutex_unlock(&first->mutex);
}
```

#### ❌ Bad: Inconsistent Lock Order

```c
// Thread 1 calls: transfer(A, B)
// Thread 2 calls: transfer(B, A)
// DEADLOCK possible!

void transfer_bad(resource_t* from, resource_t* to, int amount) {
    mutex_lock(&from->mutex);  // Thread 1: locks A, Thread 2: locks B
    mutex_lock(&to->mutex);    // Thread 1: waits B, Thread 2: waits A
    // ...
}
```

### 6.2 Timeout Locks（タイムアウト付きロック）

#### ✅ Good: Lock with Timeout and Recovery

```c
typedef enum {
    LOCK_SUCCESS,
    LOCK_TIMEOUT,
    LOCK_ERROR
} lock_result_t;

lock_result_t try_lock_with_timeout(mutex_t* mutex, uint32_t timeout_ms);

int safe_resource_operation(resource_t* res, uint32_t timeout_ms) {
    lock_result_t result = try_lock_with_timeout(&res->mutex, timeout_ms);

    switch (result) {
        case LOCK_SUCCESS:
            perform_operation(res);
            mutex_unlock(&res->mutex);
            return 0;

        case LOCK_TIMEOUT:
            log_warning("Lock timeout on resource, possible deadlock");
            record_lock_timeout_event(res);
            return -ETIMEDOUT;

        case LOCK_ERROR:
            log_error("Lock error on resource");
            return -EIO;
    }
    return -EINVAL;
}
```

### 6.3 Lock Hierarchy（ロック階層）

#### ✅ Good: Documented Lock Hierarchy

```c
/*
 * Lock Hierarchy (acquire in ascending order):
 *   Level 1: hardware_mutex     (lowest - always acquire first)
 *   Level 2: driver_mutex
 *   Level 3: service_mutex
 *   Level 4: application_mutex  (highest - acquire last)
 *
 * RULE: Never acquire a lower-level lock while holding a higher-level lock
 */

typedef enum {
    LOCK_LEVEL_HARDWARE = 1,
    LOCK_LEVEL_DRIVER = 2,
    LOCK_LEVEL_SERVICE = 3,
    LOCK_LEVEL_APPLICATION = 4,
} lock_level_t;

typedef struct {
    mutex_t mutex;
    lock_level_t level;
    const char* name;
} leveled_mutex_t;

// Debug mode: track current lock level per task
#ifdef DEBUG
static __thread lock_level_t current_lock_level = 0;

void leveled_mutex_lock(leveled_mutex_t* m) {
    if (m->level <= current_lock_level) {
        log_error("Lock hierarchy violation: trying to acquire %s (level %d) "
                  "while holding level %d", m->name, m->level, current_lock_level);
        // In debug builds, abort to catch this early
        abort();
    }
    mutex_lock(&m->mutex);
    current_lock_level = m->level;
}
#endif
```

---

## 7. Shared Resource Management（共有リソース管理）

### 7.1 Ownership Clarity（所有権の明確化）

**Principle**: 各リソースの所有者を明確にし、所有権の移譲を明示的に行う。

#### ✅ Good: Clear Ownership Model

```c
typedef struct {
    void* data;
    size_t size;
    task_id_t owner;      // Current owner
    bool transfer_pending;
} owned_buffer_t;

// Transfer ownership explicitly
int buffer_transfer_ownership(owned_buffer_t* buf,
                              task_id_t from,
                              task_id_t to) {
    if (buf->owner != from) {
        log_error("Ownership violation: task %d tried to transfer buffer "
                  "owned by %d", from, buf->owner);
        return -EPERM;
    }

    log_debug("Buffer ownership transferred: %d -> %d", from, to);
    buf->owner = to;
    return 0;
}

// Only owner can access
int buffer_write(owned_buffer_t* buf, task_id_t caller,
                 const void* data, size_t len) {
    if (buf->owner != caller) {
        log_error("Access denied: task %d cannot write to buffer owned by %d",
                  caller, buf->owner);
        return -EPERM;
    }
    // ... perform write
    return 0;
}
```

### 7.2 Resource Pools（リソースプール）

#### ✅ Good: Thread-Safe Resource Pool

```c
typedef struct {
    void* resources[POOL_SIZE];
    bool in_use[POOL_SIZE];
    mutex_t mutex;
    semaphore_t available;
    int available_count;
} resource_pool_t;

void* pool_acquire(resource_pool_t* pool, uint32_t timeout_ms) {
    // Wait for available resource
    if (semaphore_wait_timeout(&pool->available, timeout_ms) != 0) {
        return NULL;  // Timeout
    }

    mutex_lock(&pool->mutex);
    void* resource = NULL;
    for (int i = 0; i < POOL_SIZE; i++) {
        if (!pool->in_use[i]) {
            pool->in_use[i] = true;
            resource = pool->resources[i];
            pool->available_count--;
            break;
        }
    }
    mutex_unlock(&pool->mutex);

    return resource;
}

void pool_release(resource_pool_t* pool, void* resource) {
    mutex_lock(&pool->mutex);
    for (int i = 0; i < POOL_SIZE; i++) {
        if (pool->resources[i] == resource && pool->in_use[i]) {
            pool->in_use[i] = false;
            pool->available_count++;
            semaphore_signal(&pool->available);
            break;
        }
    }
    mutex_unlock(&pool->mutex);
}
```

---

## 8. Event-Driven Design（イベント駆動設計）

### 8.1 Polling vs Interrupt（ポーリング vs 割り込み）

**Use Polling When:**
- Low event frequency
- Timing is not critical
- Hardware doesn't support interrupts
- Deterministic timing is needed

**Use Interrupts When:**
- Immediate response required
- High-frequency events
- Power efficiency is important
- Asynchronous events

#### ✅ Good: Appropriate Use of Interrupts with Deferred Processing

```c
// ISR - minimal work, defer processing
static volatile bool data_ready = false;
static volatile uint8_t isr_buffer[ISR_BUFFER_SIZE];

void sensor_isr(void) {
    // Minimal ISR: just capture data and set flag
    read_sensor_hardware(isr_buffer);
    data_ready = true;
    // Do NOT process data here!
}

// Main loop or task - process deferred work
void sensor_task(void) {
    while (1) {
        if (data_ready) {
            uint8_t local_buffer[ISR_BUFFER_SIZE];

            // Copy data out of ISR buffer quickly
            irq_state_t state = enter_critical();
            memcpy(local_buffer, (void*)isr_buffer, ISR_BUFFER_SIZE);
            data_ready = false;
            exit_critical(state);

            // Heavy processing outside ISR context
            process_sensor_data(local_buffer);
        }
        task_yield();  // or sleep
    }
}
```

### 8.2 Event Queue Design（イベントキュー設計）

#### ✅ Good: Thread-Safe Event Queue with Overflow Handling

```c
typedef struct {
    uint32_t type;
    uint32_t timestamp;
    void* data;
} event_t;

typedef struct {
    event_t buffer[EVENT_QUEUE_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
    uint32_t overflow_count;  // Statistics
    void (*overflow_callback)(event_t* dropped_event);
} event_queue_t;

bool event_queue_post(event_queue_t* q, event_t* event) {
    irq_state_t state = enter_critical();

    if (q->count >= EVENT_QUEUE_SIZE) {
        q->overflow_count++;

        // Option 1: Drop oldest event
        event_t dropped = q->buffer[q->tail];
        q->tail = (q->tail + 1) % EVENT_QUEUE_SIZE;
        q->count--;

        if (q->overflow_callback) {
            exit_critical(state);
            q->overflow_callback(&dropped);
            state = enter_critical();
        }

        log_warning("Event queue overflow (total: %lu)", q->overflow_count);
    }

    q->buffer[q->head] = *event;
    q->head = (q->head + 1) % EVENT_QUEUE_SIZE;
    q->count++;

    exit_critical(state);
    return true;
}

bool event_queue_get(event_queue_t* q, event_t* event, uint32_t timeout_ms) {
    uint32_t start = get_tick_ms();

    while (1) {
        irq_state_t state = enter_critical();
        if (q->count > 0) {
            *event = q->buffer[q->tail];
            q->tail = (q->tail + 1) % EVENT_QUEUE_SIZE;
            q->count--;
            exit_critical(state);
            return true;
        }
        exit_critical(state);

        if (get_tick_ms() - start >= timeout_ms) {
            return false;  // Timeout
        }

        task_yield();
    }
}
```

### 8.3 Event Loop Pattern（イベントループパターン）

#### ✅ Good: Priority-Based Event Processing

```c
typedef enum {
    PRIORITY_CRITICAL = 0,  // Highest priority
    PRIORITY_HIGH,
    PRIORITY_NORMAL,
    PRIORITY_LOW,
    PRIORITY_COUNT
} event_priority_t;

typedef struct {
    event_queue_t queues[PRIORITY_COUNT];
    volatile bool shutdown_requested;
} event_dispatcher_t;

void event_loop(event_dispatcher_t* dispatcher) {
    while (!dispatcher->shutdown_requested) {
        event_t event;
        bool found = false;

        // Process events in priority order
        for (int pri = PRIORITY_CRITICAL; pri < PRIORITY_COUNT; pri++) {
            if (event_queue_get(&dispatcher->queues[pri], &event, 0)) {
                handle_event(&event);
                found = true;
                break;  // Check higher priorities again
            }
        }

        if (!found) {
            // No events, sleep to save power
            sleep_until_event_or_timeout(IDLE_TIMEOUT_MS);
        }
    }
}
```

---

## Summary: Key Principles

| Area | Principle | Enforcement |
|------|-----------|-------------|
| State Machine | 明示的な遷移定義 | Table-driven design |
| Initialization | 依存関係に基づく順序 | Dependency declaration |
| Shutdown | 初期化の逆順 | LIFO cleanup stack |
| Reset | 種別に応じた処理 | Reset reason check |
| Concurrency | 最小限のクリティカルセクション | Atomic operations |
| Deadlock | 一貫したロック順序 | Lock hierarchy |
| Resources | 所有権の明確化 | Explicit ownership |
| Events | ISRは最小限、処理は遅延 | Event queue pattern |

---

## References

- [Finite State Machines in Embedded Systems](https://www.embeddedrelated.com/showarticle/1665.php)
- [Embedded Rust Concurrency](https://docs.rust-embedded.org/book/concurrency/)
- [Race Conditions in Firmware Design](https://medium.com/@lanceharvieruntime/are-race-conditions-ruining-your-firmware-design-heres-how-to-fix-it-4d6d7b547ca)
- [Deadlock Prevention in Embedded Systems](https://www.numberanalytics.com/blog/deadlock-prevention-in-embedded-systems)
- [Watchdog Timer Best Practices](https://interrupt.memfault.com/blog/firmware-watchdog-best-practices)
- [Polling vs Interrupts](https://www.totalphase.com/blog/2023/10/polling-interrupts-exploring-differences-applications/)
- [Graceful Shutdown Patterns](https://www.geeksforgeeks.org/system-design/graceful-shutdown-in-distributed-systems-and-microservices/)
