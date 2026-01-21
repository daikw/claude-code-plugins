# Real-Time Safety Rules

リアルタイムシステムとタイミング安全性のためのルール。
ハードウェア/ソフトウェア非依存の汎用ガイドライン。

---

## Pre-Commit Checklist

コミット前に必ず確認する:

### Deadline & Timing
- [ ] すべてのデッドラインが明示的に定義されている
- [ ] ハードリアルタイム要件とソフトリアルタイム要件が区別されている
- [ ] 最悪実行時間（WCET）が分析されている
- [ ] タイミングクリティカルなコードが分離されている

### ISR (Interrupt Service Routine)
- [ ] ISR は最小限の処理のみ（フラグセット、リングバッファ書込み）
- [ ] ISR 内でブロッキング操作がない
- [ ] ISR 内で printf/ログ出力がない
- [ ] ISR 内で動的メモリ割り当てがない
- [ ] 共有変数に `volatile` が付いている

### Priority & Synchronization
- [ ] 優先度逆転の可能性を検討した
- [ ] 優先度継承または優先度上限プロトコルを使用している
- [ ] クリティカルセクションは最短時間に抑えている

### Watchdog
- [ ] ウォッチドッグタイマーが有効化されている
- [ ] すべてのクリティカルタスクがウォッチドッグ監視下にある
- [ ] リカバリ動作が定義されている

### Time Handling
- [ ] 32ビットミリ秒カウンタのオーバーフロー（49日）を考慮している
- [ ] 時間比較に符号なし算術を使用している
- [ ] ジッター許容値が定義されている

---

## 1. Deadline Compliance（デッドライン遵守）

### Hard Real-Time vs Soft Real-Time

| 種別 | 説明 | デッドライン違反時 |
|------|------|-------------------|
| **Hard Real-Time** | デッドライン違反は致命的障害 | システム障害、安全問題 |
| **Soft Real-Time** | デッドライン違反はQoS低下 | パフォーマンス劣化のみ |

### ✅ Good: デッドライン明示

```c
// タスクごとにデッドライン要件を文書化
/*
 * Task: motor_control_task
 * Type: Hard real-time
 * Period: 1ms
 * Deadline: 900us (90% of period)
 * WCET: 200us (measured)
 */
void motor_control_task(void) {
    // ...
}
```

### ❌ Bad: 暗黙のタイミング要件

```c
// デッドライン要件が不明
void some_control_task(void) {
    // このタスクはいつまでに完了すべき？
    // 遅延したらどうなる？
}
```

---

## 2. Interrupt Latency Constraints（割り込みレイテンシ制約）

### ISR 実行時間の制限

- **目標**: ISR は可能な限り短く（数十マイクロ秒以内）
- **原則**: 重い処理は通常タスクに委譲

### ✅ Good: 最小限の ISR

```c
volatile bool data_ready = false;
volatile uint8_t rx_data;

void UART_ISR(void) {
    rx_data = read_uart_register();
    data_ready = true;  // フラグを立てるだけ
    clear_interrupt_flag();
}

// メインループで処理
void main_loop(void) {
    if (data_ready) {
        data_ready = false;
        process_received_data(rx_data);  // 重い処理はここ
    }
}
```

### ❌ Bad: 重い処理を ISR 内で実行

```c
void UART_ISR(void) {
    uint8_t data = read_uart_register();

    // ❌ ISR 内で重い処理
    parse_protocol(data);
    update_database(data);
    send_response();       // ブロッキングの可能性
    printf("Received: %d\n");  // ❌ printf は禁止

    clear_interrupt_flag();
}
```

### ネスト割り込みの制御

```c
// ✅ 優先度の高い割り込みのみネストを許可
void low_priority_isr(void) {
    // 高優先度割り込みは許可
    enable_higher_priority_interrupts();

    // 処理...

    disable_interrupts();
}
```

---

## 3. Priority Inversion Prevention（優先度逆転防止）

優先度逆転とは、低優先度タスクがリソースを保持している間に高優先度タスクがブロックされる問題。
Mars Pathfinder（1997年）で有名になった古典的問題。

### 解決策1: Priority Inheritance（優先度継承）

```c
/*
 * 優先度継承プロトコル:
 * - 低優先度タスクがリソースを保持中に高優先度タスクが待機した場合
 * - 低優先度タスクの優先度を一時的に引き上げる
 */

// ✅ 優先度継承付きミューテックスを使用
mutex_attr_t attr;
attr.protocol = MUTEX_PRIO_INHERIT;
mutex_init(&shared_resource_mutex, &attr);
```

### 解決策2: Priority Ceiling Protocol（優先度上限プロトコル）

```c
/*
 * 優先度上限プロトコル:
 * - 各リソースに「上限優先度」を設定
 * - リソース取得時、タスク優先度を上限まで引き上げ
 * - デッドロック防止にも効果あり
 */

// ✅ 優先度上限を設定
mutex_attr_t attr;
attr.protocol = MUTEX_PRIO_CEILING;
attr.prioceiling = HIGHEST_TASK_PRIORITY_USING_THIS_RESOURCE;
mutex_init(&shared_resource_mutex, &attr);
```

### ❌ Bad: 保護なしの共有リソース

```c
// 優先度逆転が発生する可能性
void low_priority_task(void) {
    mutex_lock(&shared_mutex);  // 取得
    long_operation();           // 長時間保持
    mutex_unlock(&shared_mutex);
}

void high_priority_task(void) {
    mutex_lock(&shared_mutex);  // ここでブロック！
    critical_operation();
    mutex_unlock(&shared_mutex);
}
// 中優先度タスクが low_priority_task をプリエンプトすると
// high_priority_task は無期限にブロックされる
```

### クリティカルセクションのガイドライン

```c
// ✅ クリティカルセクションは最短に
mutex_lock(&mutex);
temp = shared_data;  // 読み取りのみ
mutex_unlock(&mutex);

process(temp);  // 処理はロック外で

mutex_lock(&mutex);
shared_data = result;  // 書き込みのみ
mutex_unlock(&mutex);
```

---

## 4. Watchdog Timer Policy（ウォッチドッグタイマーポリシー）

### 必須要件

1. **すべての本番システムでウォッチドッグを有効化**
2. **リリースビルドでウォッチドッグを無効化しない**
3. **リカバリ動作を明確に定義**

### タイムアウト設定ガイドライン

```c
/*
 * タイムアウトの選択基準:
 * - 通常動作の最大処理時間よりも十分長く（5-30秒が目安）
 * - システム応答要件を考慮（医療機器なら短く）
 * - フォールストリガーを避ける
 */

// ✅ 要件に基づいたタイムアウト設定
#define WDT_TIMEOUT_MS  10000  // 10秒

/*
 * 根拠:
 * - 最大タスク実行時間: 2秒
 * - 想定最悪ケース: 5秒
 * - マージン: 2倍
 */
```

### マルチタスク環境でのウォッチドッグ

```c
// ✅ 各タスクの健全性を確認してからフィード
typedef struct {
    volatile bool task_a_alive;
    volatile bool task_b_alive;
    volatile bool task_c_alive;
} system_health_t;

static system_health_t health;

void watchdog_task(void) {
    while (1) {
        if (health.task_a_alive &&
            health.task_b_alive &&
            health.task_c_alive) {

            watchdog_feed();

            // フラグをリセット（次の周期で再確認）
            health.task_a_alive = false;
            health.task_b_alive = false;
            health.task_c_alive = false;
        }
        sleep_ms(1000);
    }
}

// 各タスクは定期的にフラグをセット
void task_a(void) {
    while (1) {
        do_work();
        health.task_a_alive = true;
        sleep_ms(100);
    }
}
```

### ❌ Bad: 無条件のウォッチドッグフィード

```c
// メインループでただフィードするだけ
void main_loop(void) {
    while (1) {
        watchdog_feed();  // ❌ システム状態を確認していない
        // ループが回っていても各タスクがハングしている可能性
    }
}
```

### リカバリアクション

```c
// ✅ ウォッチドッグリセット後のリカバリ
void system_init(void) {
    reset_reason_t reason = get_reset_reason();

    if (reason == RESET_WATCHDOG) {
        // ウォッチドッグリセットを記録
        log_to_persistent_storage("WDT reset occurred");
        increment_wdt_counter();

        // LED点滅などで通知
        indicate_wdt_reset();

        // 連続リセットの場合はセーフモードへ
        if (get_wdt_counter() > 3) {
            enter_safe_mode();
        }
    }
}
```

---

## 5. Timing-Critical Code Isolation（タイミングクリティカルコード分離）

### 分離の原則

```c
// ✅ タイミングクリティカルなコードを分離
// timing_critical.c - このファイルには厳格なルール適用
void motor_pwm_update(void) {  // WCET: 50us
    // 決定論的な処理のみ
    // - ブロッキング禁止
    // - 動的割り当て禁止
    // - 浮動小数点演算は事前計算
}

// non_critical.c - 通常の制約
void update_display(void) {
    // ここは多少の遅延OK
}
```

### ❌ Bad: クリティカルパスにブロッキング

```c
void timing_critical_function(void) {
    read_sensor();

    // ❌ クリティカルパスにブロッキング操作
    mutex_lock(&log_mutex);
    write_log("sensor value");  // ディスクI/O
    mutex_unlock(&log_mutex);

    update_actuator();
}
```

### ✅ Good: ブロッキングを分離

```c
void timing_critical_function(void) {
    read_sensor();
    queue_log_message("sensor value");  // ノンブロッキング
    update_actuator();
}

// 別タスクでログ処理
void log_task(void) {
    while (1) {
        char* msg = dequeue_log_message();  // ブロッキングOK
        write_to_storage(msg);
    }
}
```

---

## 6. ISR Rules（ISR ルール）

### ISR の禁止事項

| 操作 | 理由 |
|------|------|
| `printf` / ログ出力 | ブロッキング、不定時間 |
| `malloc` / `free` | ブロッキング、断片化 |
| ミューテックス取得 | デッドロック、優先度逆転 |
| ファイルI/O | ブロッキング |
| ネットワーク送信 | ブロッキング |
| 浮動小数点演算 | コンテキスト保存コスト（一部アーキテクチャ） |

### ISR パターン

```c
// ✅ パターン1: フラグ方式
volatile bool event_occurred = false;

void ISR_Handler(void) {
    event_occurred = true;
    clear_interrupt();
}

void main_loop(void) {
    if (event_occurred) {
        event_occurred = false;
        handle_event();
    }
}
```

```c
// ✅ パターン2: リングバッファ方式
void UART_ISR(void) {
    uint8_t byte = read_uart();
    ring_buffer_put(&rx_buffer, byte);  // ロックフリー
    clear_interrupt();
}
```

```c
// ✅ パターン3: セマフォ方式（RTOS使用時）
void Sensor_ISR(void) {
    read_sensor_to_buffer();
    semaphore_give_from_isr(&data_ready_sem);
    clear_interrupt();
}

void sensor_task(void) {
    while (1) {
        semaphore_take(&data_ready_sem);  // ここでブロック
        process_sensor_data();
    }
}
```

### volatile の使い方

```c
// ✅ ISR とメインコード間で共有する変数
volatile uint32_t tick_count = 0;
volatile bool alarm_triggered = false;

// ❌ volatile なしは最適化で問題
uint32_t tick_count = 0;  // コンパイラが変更を見逃す可能性
```

---

## 7. Jitter Management（ジッター管理）

### ジッターとは

- 周期タスクの実行タイミングのばらつき
- 制御システムでは不安定性の原因となる

### ジッター低減策

```c
// ✅ ハードウェアタイマーで正確な周期を実現
void Timer_ISR(void) {
    // タイマー割り込みで直接起動
    // ソフトウェアスケジューリングより正確
    update_motor_control();
}
```

```c
// ✅ 高優先度で周期タスクを実行
task_create(
    "motor_control",
    motor_control_task,
    PRIORITY_HIGHEST,
    STACK_SIZE
);
```

### ❌ ジッターを引き起こす設計

```c
void periodic_task(void) {
    while (1) {
        do_work();

        // ❌ 実行時間を含めた遅延
        sleep_ms(100);  // do_work()の時間分ずれていく
    }
}
```

### ✅ ジッターを抑える設計

```c
void periodic_task(void) {
    uint32_t next_wake = get_tick();

    while (1) {
        next_wake += 100;  // 次の起床時刻を計算

        do_work();

        // ✅ 絶対時刻まで待機
        sleep_until(next_wake);
    }
}
```

### ジッター測定

```c
// ✅ ジッターを測定して記録
static uint32_t last_execution = 0;
static uint32_t max_jitter = 0;

void periodic_task(void) {
    uint32_t now = get_tick();

    if (last_execution != 0) {
        uint32_t period = now - last_execution;
        uint32_t jitter = abs(period - EXPECTED_PERIOD);

        if (jitter > max_jitter) {
            max_jitter = jitter;
        }
    }

    last_execution = now;
    // ...
}
```

---

## 8. Time Overflow Handling（時間オーバーフロー処理）

### 32ビットミリ秒カウンタの問題

- `uint32_t` のミリ秒カウンタは約49.7日でオーバーフロー
- 2^32 ms = 4,294,967,296 ms ≈ 49.71 日
- Windows 98 はこれで有名なバグを起こした

### ❌ Bad: オーバーフローを考慮しない比較

```c
uint32_t start_time = get_tick_ms();

// 49日後にオーバーフローして誤動作
if (get_tick_ms() > start_time + timeout) {
    handle_timeout();
}
```

### ✅ Good: 符号なし算術による正しい比較

```c
uint32_t start_time = get_tick_ms();

// ✅ 差分計算はオーバーフローしても正しく動作
if ((get_tick_ms() - start_time) >= timeout) {
    handle_timeout();
}
```

### 動作原理

```c
/*
 * 例: start=0xFFFFFFF0, now=0x00000010, timeout=0x30
 *
 * ❌ Bad: now > start + timeout
 *    → 0x10 > 0x20 = false（誤り）
 *
 * ✅ Good: (now - start) >= timeout
 *    → (0x10 - 0xFFFFFFF0) = 0x20 >= 0x30 = false（正しい）
 *    → 後で: (0x30 - 0xFFFFFFF0) = 0x40 >= 0x30 = true
 */
```

### 安全なタイムアウトマクロ

```c
// ✅ オーバーフロー安全なタイムアウトチェック
#define TIMEOUT_ELAPSED(start, timeout) \
    ((uint32_t)(get_tick_ms() - (start)) >= (timeout))

// 使用例
uint32_t start = get_tick_ms();
while (!TIMEOUT_ELAPSED(start, 1000)) {
    // 1秒間のループ
}
```

### 長期間タイマーが必要な場合

```c
// ✅ 64ビットカウンタを使用（584 million年まで対応）
typedef struct {
    volatile uint32_t high;
    volatile uint32_t low;
} time64_t;

uint64_t get_time64(void) {
    uint32_t h1, h2, l;
    do {
        h1 = timer.high;
        l = timer.low;
        h2 = timer.high;
    } while (h1 != h2);  // レースコンディション回避

    return ((uint64_t)h1 << 32) | l;
}

// タイマーオーバーフロー割り込み
void Timer_Overflow_ISR(void) {
    timer.high++;
}
```

---

## Quick Reference

### ISR チェックリスト

```
✅ 処理は最小限に（フラグ、リングバッファ、セマフォ）
✅ volatile を共有変数に使用
✅ 割り込みフラグをクリア
❌ printf / ログ出力
❌ malloc / free
❌ mutex_lock
❌ ブロッキング呼び出し
❌ 長い計算
```

### 優先度設計テンプレート

```
最高: ハードウェア割り込み
  ↓  ハードリアルタイムタスク
  ↓  ソフトリアルタイムタスク
  ↓  ウォッチドッグ監視タスク
  ↓  通信タスク
  ↓  ログタスク
最低: アイドルタスク
```

### タイムアウト設計

```c
// 短期タイムアウト: uint32_t で十分
uint32_t timeout_ms = 5000;  // 5秒

// 長期タイムアウト: 日単位の場合は注意
// 49日を超える場合は64ビットまたは秒単位を検討
uint32_t timeout_sec = days * 24 * 60 * 60;  // 秒単位
```

---

## References

- [Deterministic Behavior in Real-Time Embedded Systems](https://www.embeddedrelated.com/showarticle/1742.php)
- [Priority Inversion Prevention - Embedded.com](https://www.embedded.com/how-to-use-priority-inheritance/)
- [Priority Ceiling Protocol Guide](https://www.numberanalytics.com/blog/priority-ceiling-protocol-ultimate-guide)
- [ISR Best Practices - Embedded.com](https://www.embedded.com/5-best-practices-for-writing-interrupt-service-routines/)
- [Watchdog Timer Best Practices - Memfault](https://interrupt.memfault.com/blog/firmware-watchdog-best-practices)
- [Jitter Management in Embedded Systems](https://runtimerec.com/breaking-down-jitter-ensuring-reliable-timing-in-embedded-systems/)
- [Timer Overflow Handling](https://community.st.com/t5/stm32-mcus-embedded-software/what-happens-when-the-32bit-hal-uwtick-timer-variable-overflows/td-p/120367)
