# Memory Safety Rules

組込/エッジ開発におけるメモリ安全性ルール。ハードウェア・ソフトウェア非依存の汎用ガイドライン。

## Pre-Commit Checklist

コミット前に必ず確認する:

### Stack Safety
- [ ] 関数内でスタックに大きな配列/構造体を確保していないこと
- [ ] 再帰関数を使用していないこと（または深さ制限あり）
- [ ] スタックカナリが有効化されていること（`-fstack-protector-strong`）
- [ ] 割り込みハンドラのスタック使用量が計算済みであること

### Heap Safety
- [ ] 動的メモリ確保を最小限に抑えていること
- [ ] `malloc`/`free`のペアが適切に対応していること
- [ ] メモリプール使用時は固定サイズブロックであること
- [ ] 初期化フェーズ以外での動的確保を避けていること

### Buffer Safety
- [ ] 全バッファアクセスで境界チェックを実施していること
- [ ] `strcpy`, `sprintf`, `strcat`を使用していないこと
- [ ] `snprintf`, `strlcpy`等の安全な関数を使用していること
- [ ] 配列インデックスが範囲内であることを検証していること

### Resource Management
- [ ] 全リソースに対し確保・解放のペアが存在すること
- [ ] エラーパスでもリソース解放が行われること
- [ ] ダングリングポインタが発生しないこと

### Memory Access
- [ ] ハードウェアレジスタに`volatile`を使用していること
- [ ] 共有変数（割り込み/マルチスレッド）に`volatile`を使用していること
- [ ] メモリアライメント要件を満たしていること
- [ ] キャッシュコヒーレンシを考慮していること（DMA使用時）

---

## 1. Stack Overflow Prevention

スタックオーバーフローは組込システムで最も危険なバグの一つ。

### Stack Size Estimation

スタックサイズは以下を考慮して決定する:
- 最深呼び出しチェーンの解析
- 割り込みネスト時の追加スタック
- ローカル変数サイズの合計

```c
// ❌ Bad: スタック上に大きな配列
void process_data(void) {
    uint8_t buffer[4096];  // スタックを圧迫
    char temp[1024];       // さらに追加
    // ...
}

// ✅ Good: 静的確保またはヒープ（初期化時のみ）
static uint8_t g_process_buffer[4096];  // .bss セクション

void process_data(void) {
    // g_process_buffer を使用
}
```

### Stack Canaries

スタックカナリはオーバーフローを実行時に検出する仕組み。

```c
// GCC/Clang: コンパイルオプションで有効化
// -fstack-protector        : 8バイト以上のchar配列がある関数のみ
// -fstack-protector-strong : より広いカバレッジ（推奨）
// -fstack-protector-all    : 全関数（オーバーヘッド大）

// カナリ検出時のハンドラ実装
void __stack_chk_fail(void) {
    // ログ出力、システム停止、再起動など
    system_halt("Stack overflow detected");
}

// カナリ値の初期化（ランダム値推奨）
uintptr_t __stack_chk_guard = 0xDEADBEEF;  // 本番では乱数で初期化
```

### Recursion Avoidance

```c
// ❌ Bad: 再帰によるスタック消費が予測不能
int factorial(int n) {
    if (n <= 1) return 1;
    return n * factorial(n - 1);  // スタックが深くなる
}

// ✅ Good: ループに変換
int factorial(int n) {
    int result = 1;
    for (int i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
}
```

---

## 2. Heap Fragmentation Prevention

フラグメンテーションは長時間稼働する組込システムで深刻な問題。

### Static Allocation Preference

```c
// ❌ Bad: 実行時に動的確保
void init_sensors(int count) {
    sensors = malloc(count * sizeof(sensor_t));  // フラグメントの原因
}

// ✅ Good: 静的確保
#define MAX_SENSORS 16
static sensor_t g_sensors[MAX_SENSORS];

void init_sensors(int count) {
    assert(count <= MAX_SENSORS);
    // g_sensors を使用
}
```

### Memory Pools

固定サイズブロックのプールでフラグメンテーションを防止。

```c
// ✅ Good: メモリプール実装
typedef struct {
    uint8_t* pool;
    size_t block_size;
    size_t block_count;
    uint32_t* free_bitmap;
} memory_pool_t;

void* pool_alloc(memory_pool_t* p) {
    for (size_t i = 0; i < p->block_count; i++) {
        if (!(p->free_bitmap[i / 32] & (1U << (i % 32)))) {
            p->free_bitmap[i / 32] |= (1U << (i % 32));
            return p->pool + (i * p->block_size);
        }
    }
    return NULL;  // プール枯渇
}

void pool_free(memory_pool_t* p, void* ptr) {
    size_t index = ((uint8_t*)ptr - p->pool) / p->block_size;
    p->free_bitmap[index / 32] &= ~(1U << (index % 32));
}
```

### Startup-Only Dynamic Allocation

```c
// ✅ Good: 起動時のみ動的確保を許可
static bool g_init_phase = true;

void* safe_malloc(size_t size) {
    if (!g_init_phase) {
        // 本番コードではアサート失敗/ログ/リセット
        assert(!"malloc after init phase");
        return NULL;
    }
    return malloc(size);
}

void end_init_phase(void) {
    g_init_phase = false;
    // 以降の malloc は禁止
}
```

---

## 3. Buffer Overflow Prevention

バッファオーバーフローはセキュリティ脆弱性の主要因。

### Bounds Checking

```c
// ❌ Bad: 境界チェックなし
void copy_data(uint8_t* dst, const uint8_t* src, size_t len) {
    memcpy(dst, src, len);  // dst のサイズを確認していない
}

// ✅ Good: 明示的な境界チェック
typedef struct {
    uint8_t* data;
    size_t capacity;
    size_t length;
} buffer_t;

bool buffer_copy(buffer_t* dst, const uint8_t* src, size_t len) {
    if (len > dst->capacity) {
        return false;  // バッファ不足
    }
    memcpy(dst->data, src, len);
    dst->length = len;
    return true;
}
```

### Safe String Functions

```c
// ❌ Bad: 危険な文字列関数
char buffer[64];
strcpy(buffer, user_input);              // オーバーフロー可能
sprintf(buffer, "Value: %s", str);       // オーバーフロー可能
strcat(buffer, suffix);                   // オーバーフロー可能

// ✅ Good: 安全な文字列関数
char buffer[64];

// snprintf: 常にNULL終端、戻り値で切り詰め検出可能
int ret = snprintf(buffer, sizeof(buffer), "Value: %s", str);
if (ret >= (int)sizeof(buffer)) {
    // 切り詰めが発生した
}

// strlcpy (POSIX.1-2024): 常にNULL終端
size_t copied = strlcpy(buffer, user_input, sizeof(buffer));
if (copied >= sizeof(buffer)) {
    // 切り詰めが発生した
}

// strlcat (POSIX.1-2024): 安全な連結
strlcat(buffer, suffix, sizeof(buffer));
```

### Array Index Validation

```c
// ❌ Bad: インデックス検証なし
int get_sensor_value(int index) {
    return sensor_values[index];  // 範囲外アクセス可能
}

// ✅ Good: インデックス検証あり
#define SENSOR_COUNT 8
static int sensor_values[SENSOR_COUNT];

int get_sensor_value(int index, int* value) {
    if (index < 0 || index >= SENSOR_COUNT) {
        return -1;  // エラー
    }
    *value = sensor_values[index];
    return 0;  // 成功
}
```

---

## 4. Memory Leak Prevention

### Resource Tracking

```c
// ❌ Bad: エラーパスでリソースリーク
int process_file(const char* path) {
    FILE* f = fopen(path, "r");
    char* buf = malloc(1024);

    if (some_condition) {
        return -1;  // f と buf がリーク
    }

    // ...
    fclose(f);
    free(buf);
    return 0;
}

// ✅ Good: 統一されたクリーンアップパス
int process_file(const char* path) {
    int result = -1;
    FILE* f = NULL;
    char* buf = NULL;

    f = fopen(path, "r");
    if (!f) goto cleanup;

    buf = malloc(1024);
    if (!buf) goto cleanup;

    if (some_condition) {
        goto cleanup;  // エラーでもクリーンアップ実行
    }

    // ... 処理 ...
    result = 0;

cleanup:
    if (buf) free(buf);
    if (f) fclose(f);
    return result;
}
```

### RAII Pattern in C

GCC/Clang の `cleanup` 属性で自動解放を実現。

```c
// cleanup 関数の定義
static void cleanup_file(FILE** fp) {
    if (*fp) {
        fclose(*fp);
        *fp = NULL;
    }
}

static void cleanup_malloc(void** ptr) {
    if (*ptr) {
        free(*ptr);
        *ptr = NULL;
    }
}

// マクロで使いやすく
#define AUTO_FILE __attribute__((cleanup(cleanup_file)))
#define AUTO_FREE __attribute__((cleanup(cleanup_malloc)))

// ✅ Good: 自動リソース解放
int process_file(const char* path) {
    AUTO_FILE FILE* f = fopen(path, "r");
    AUTO_FREE char* buf = malloc(1024);

    if (!f || !buf) return -1;

    if (some_condition) {
        return -1;  // 自動的に fclose, free される
    }

    // ... 処理 ...
    return 0;  // 自動的にクリーンアップ
}
```

### Preventing Dangling Pointers

```c
// ❌ Bad: 解放後にポインタが残る
void cleanup_context(context_t* ctx) {
    free(ctx->buffer);
    // ctx->buffer はダングリングポインタに
}

// ✅ Good: 解放後にNULLセット
void cleanup_context(context_t* ctx) {
    free(ctx->buffer);
    ctx->buffer = NULL;  // ダングリングポインタ防止
    ctx->buffer_size = 0;
}

// ✅ Better: 安全なfreeマクロ
#define SAFE_FREE(ptr) do { free(ptr); (ptr) = NULL; } while(0)
```

---

## 5. Static vs Dynamic Allocation Guidelines

### When to Use Static Allocation

- 実行時間が重要な箇所（リアルタイム制約）
- メモリ使用量が事前に確定している場合
- 長時間稼働するシステム（24/7運用）
- 安全規格（IEC 61508, ISO 26262等）準拠が必要な場合

```c
// ✅ 通信バッファは静的確保
static uint8_t uart_rx_buffer[256];
static uint8_t uart_tx_buffer[256];

// ✅ センサーデータも静的確保
static sensor_data_t sensor_readings[MAX_SENSORS];
```

### When Dynamic Allocation May Be Acceptable

- 起動時の構成に基づくサイズ決定
- 必要なメモリ量がビルド時に不明
- プロトタイピング段階
- 十分なテスト/検証が可能な場合

```c
// ✅ 起動時のみ動的確保（許容ケース）
void system_init(const config_t* config) {
    // 構成に基づいてバッファサイズ決定
    size_t buffer_size = config->max_connections * BUFFER_PER_CONNECTION;
    g_connection_buffers = malloc(buffer_size);

    // この時点以降は動的確保禁止
    end_init_phase();
}
```

### Decision Matrix

| 条件 | 推奨 |
|------|------|
| サイズが固定 | 静的確保 |
| リアルタイム制約あり | 静的確保 or メモリプール |
| 長時間稼働 | 静的確保 |
| サイズが可変 | メモリプール or 起動時のみ動的 |
| 安全規格準拠 | 静的確保（動的禁止） |

---

## 6. Memory Alignment Requirements

アライメント違反は未定義動作やパフォーマンス低下の原因。

### Basic Alignment Rules

```c
// ❌ Bad: アライメント違反の可能性
void process_data(uint8_t* raw_buffer) {
    uint32_t* values = (uint32_t*)raw_buffer;  // 4バイト境界でない可能性
    *values = 0x12345678;  // 未定義動作
}

// ✅ Good: 適切なアライメント
void process_data(uint8_t* raw_buffer) {
    uint32_t value;
    memcpy(&value, raw_buffer, sizeof(value));  // アライメント安全
    // または
    value = (uint32_t)raw_buffer[0] |
            ((uint32_t)raw_buffer[1] << 8) |
            ((uint32_t)raw_buffer[2] << 16) |
            ((uint32_t)raw_buffer[3] << 24);
}
```

### Struct Packing and Alignment

```c
// ✅ Good: 明示的なアライメント指定
typedef struct __attribute__((aligned(4))) {
    uint32_t id;
    uint16_t type;
    uint16_t flags;
    uint8_t data[32];
} __attribute__((packed)) message_t;

// DMA バッファのアライメント（キャッシュライン境界）
#define CACHE_LINE_SIZE 32
static uint8_t dma_buffer[1024] __attribute__((aligned(CACHE_LINE_SIZE)));
```

### Cache Line Alignment for DMA

```c
// ❌ Bad: キャッシュラインをまたぐバッファ
static uint8_t dma_rx_buffer[100];  // サイズがキャッシュラインの倍数でない

// ✅ Good: キャッシュライン整合
#define CACHE_LINE_SIZE 32
#define DMA_BUFFER_SIZE ((100 + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE)

static uint8_t dma_rx_buffer[DMA_BUFFER_SIZE]
    __attribute__((aligned(CACHE_LINE_SIZE)));
```

---

## 7. Volatile Keyword Usage

`volatile` はコンパイラの最適化を抑制し、メモリアクセスを強制する。

### Hardware Registers

```c
// ❌ Bad: volatile なしでのレジスタアクセス
uint32_t* status_reg = (uint32_t*)0x40000000;
while (*status_reg & 0x01) {
    // コンパイラがループ外に最適化する可能性
}

// ✅ Good: volatile でのレジスタアクセス
volatile uint32_t* status_reg = (volatile uint32_t*)0x40000000;
while (*status_reg & 0x01) {
    // 毎回メモリから読み込む
}
```

### Peripheral Register Definitions

```c
// ✅ Good: 構造体でペリフェラルレジスタを定義
typedef struct {
    volatile uint32_t CTRL;       // 制御レジスタ
    volatile uint32_t STATUS;     // ステータス（読み取り専用）
    volatile uint32_t DATA;       // データレジスタ
    volatile uint32_t reserved;
} periph_regs_t;

#define PERIPH_BASE    0x40010000U
#define PERIPH         ((periph_regs_t*)PERIPH_BASE)

// 使用例
void periph_init(void) {
    PERIPH->CTRL = 0x01;  // 初期化
    while (!(PERIPH->STATUS & 0x01)) {
        // Ready ビットを待機
    }
}
```

### Shared Variables (ISR/Thread)

```c
// ❌ Bad: 共有変数に volatile なし
static bool data_ready = false;

void isr_handler(void) {
    data_ready = true;
}

void main_loop(void) {
    while (!data_ready) {
        // コンパイラが無限ループに最適化する可能性
    }
}

// ✅ Good: 共有変数に volatile あり
static volatile bool data_ready = false;

void isr_handler(void) {
    data_ready = true;
}

void main_loop(void) {
    while (!data_ready) {
        // 毎回メモリから読み込む
    }
}
```

### Volatile Limitations

`volatile` は以下を保証しない:
- アトミック性（複数バイトの読み書き）
- メモリバリア（順序保証）

```c
// ❌ volatile だけではアトミック性なし
volatile uint32_t counter;  // 32ビット未満のCPUでは非アトミック

// ✅ アトミック操作が必要な場合
#include <stdatomic.h>
atomic_uint counter;
atomic_fetch_add(&counter, 1);

// または割り込み禁止
uint32_t atomic_increment(volatile uint32_t* ptr) {
    disable_interrupts();
    uint32_t val = ++(*ptr);
    enable_interrupts();
    return val;
}
```

---

## 8. Cache Coherency Considerations

キャッシュ搭載プロセッサでDMAを使用する際の注意点。

### The Coherency Problem

```
CPU Cache:  [Old Data]
    ↓ (invalidate needed)
Main Memory: [New Data from DMA]
    ↑
DMA Write
```

### Cache Maintenance Operations

```c
// DMA 受信前: キャッシュを無効化してメインメモリを読み込み準備
void prepare_dma_receive(void* buffer, size_t size) {
    // キャッシュ無効化（Invalidate）
    // 具体的なAPIはプラットフォーム依存
    cache_invalidate_by_addr(buffer, size);
}

// DMA 送信前: キャッシュをメインメモリに書き出し
void prepare_dma_transmit(void* buffer, size_t size) {
    // キャッシュクリーン（Clean/Flush）
    cache_clean_by_addr(buffer, size);
}

// DMA 受信完了後: 再度キャッシュを無効化
void complete_dma_receive(void* buffer, size_t size) {
    cache_invalidate_by_addr(buffer, size);
}
```

### Non-Cacheable Memory Regions

```c
// ✅ Good: DMAバッファは非キャッシュ領域に配置
// リンカスクリプトで .dma_buffer セクションを非キャッシュ領域に配置

__attribute__((section(".dma_buffer")))
static uint8_t dma_rx_buffer[1024];

__attribute__((section(".dma_buffer")))
static uint8_t dma_tx_buffer[1024];
```

### Cache Line Boundaries

```c
// ❌ Bad: キャッシュラインを共有するデータ
static uint8_t dma_buffer[100];
static uint32_t other_data;  // dma_buffer と同じキャッシュラインにある可能性

// ✅ Good: キャッシュラインを分離
#define CACHE_LINE_SIZE 32

static uint8_t dma_buffer[128] __attribute__((aligned(CACHE_LINE_SIZE)));
static uint32_t other_data __attribute__((aligned(CACHE_LINE_SIZE)));
```

### Memory Barriers

```c
// メモリバリアで順序を保証
void dma_write_with_barrier(volatile uint32_t* dma_reg, uint32_t value) {
    // データをメモリに書き出し
    cache_clean_by_addr(tx_buffer, tx_size);

    // データ同期バリア
    __DSB();  // Data Synchronization Barrier

    // DMA開始
    *dma_reg = value;
}
```

---

## Code Review Checklist

レビュー時に確認すべき項目:

### Memory Allocation
- [ ] 静的確保が優先されているか
- [ ] 動的確保は起動時のみか、またはメモリプール使用か
- [ ] 確保と解放のペアが対応しているか
- [ ] 解放後にポインタをNULLに設定しているか

### Buffer Operations
- [ ] 全バッファ操作で境界チェックがあるか
- [ ] `strcpy`, `sprintf`, `strcat` を使用していないか
- [ ] 配列インデックスの検証があるか
- [ ] バッファサイズを`sizeof()`で取得しているか

### Volatile Usage
- [ ] ハードウェアレジスタに`volatile`があるか
- [ ] ISR/スレッド共有変数に`volatile`があるか
- [ ] アトミック性が必要な箇所で適切に対処しているか

### Stack Usage
- [ ] スタック上の大きな配列がないか
- [ ] 再帰がないか（または深さ制限があるか）
- [ ] 割り込みハンドラのスタック使用量が見積もられているか

### Cache/DMA
- [ ] DMAバッファがキャッシュライン整合か
- [ ] キャッシュメンテナンス操作が適切か
- [ ] 必要な箇所でメモリバリアがあるか

---

## References

- [Memory Safety in Embedded Software - RunSafe Security](https://runsafesecurity.com/blog/embedded-software-memory-safety/)
- [Why Dynamic Memory Allocation Bad for Embedded - TrebledJ](https://trebledj.me/posts/dynamic-memory-embedded-bad/)
- [MISRA C Guidelines - Embedded.com](https://www.embedded.com/how-misra-c-guidelines-enhance-code-safety-reduce-risks/)
- [Stack Canaries with GCC - MCU on Eclipse](https://mcuoneclipse.com/2019/09/28/stack-canaries-with-gcc-checking-for-stack-overflow-at-runtime/)
- [Embedded Systems and Volatile Keyword - mbedded.ninja](https://blog.mbedded.ninja/programming/languages/c/embedded-systems-and-the-volatile-keyword/)
- [Cache Coherency Issues in ARM Cortex-M7 - System on Chips](https://www.systemonchips.com/cache-coherency-issues-in-arm-cortex-m7-dma-and-cpu-interactions/)
- [RAII in C - Cardinal Peak](https://www.cardinalpeak.com/blog/raii-in-c)
- [strlcpy and strlcat - Todd C. Miller](https://www.millert.dev/papers/strlcpy/)
