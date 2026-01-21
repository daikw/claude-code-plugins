# Performance Rules

組込/エッジ開発におけるパフォーマンスルール。

## Resource-Aware Development

### メモリ使用量の意識

```c
// ✅ スタックサイズを意識
// 大きな配列はスタックに置かない
void bad_function(void) {
    uint8_t buffer[8192];  // ❌ スタックオーバーフローの危険
}

void good_function(void) {
    static uint8_t buffer[8192];  // ✅ 静的割り当て
    // または
    uint8_t* buffer = malloc(8192);  // ✅ ヒープ割り当て（要解放）
}
```

### Flash vs RAM トレードオフ

```c
// Flash に置く（読み取り専用データ）
static const char* const messages[] = {"OK", "Error", "Timeout"};

// RAM に置く（変更が必要なデータ）
static char response_buffer[256];
```

## Context Window Efficiency

### Claude モデル選択

| タスク | 推奨モデル |
|--------|-----------|
| 簡単なコード修正、ドキュメント | Haiku |
| 通常の開発、マルチエージェント | Sonnet |
| 複雑な設計、デバッグ | Opus |

### コンテキスト管理

- コンテキストの最後の1/5は単純な修正に使う
- 大規模リファクタリングは新しいセッションで
- 長いログ出力は要約してから共有

## Embedded Performance Optimization

### 割り込みレイテンシ

```c
// ✅ ISR は短く
void UART_IRQHandler(void) {
    if (UART->SR & UART_SR_RXNE) {
        ring_buffer_write(&rx_buffer, UART->DR);  // 最小限の処理
    }
}

// メインループで処理
void main_loop(void) {
    uint8_t data;
    while (ring_buffer_read(&rx_buffer, &data)) {
        process_data(data);  // 重い処理はここで
    }
}
```

### DMA 活用

```c
// ✅ CPU負荷軽減のためDMAを使用
void start_adc_sampling(void) {
    // DMA で ADC データを自動転送
    HAL_ADC_Start_DMA(&hadc, adc_buffer, BUFFER_SIZE);
}

// CPU は別の処理に専念できる
void DMA_Complete_Callback(void) {
    // 転送完了時のみ処理
    process_adc_data(adc_buffer);
}
```

### ループ最適化

```c
// ❌ 毎回関数呼び出し
for (int i = 0; i < get_count(); i++) {
    process(data[i]);
}

// ✅ ループ外で評価
int count = get_count();
for (int i = 0; i < count; i++) {
    process(data[i]);
}
```

## Power Efficiency

### スリープモードの活用

```c
// ✅ アイドル時はスリープ
void main_loop(void) {
    while (1) {
        if (has_work()) {
            do_work();
        } else {
            enter_sleep_mode();  // 割り込みで起床
        }
    }
}
```

### ペリフェラルの電源管理

```c
// ✅ 使わないペリフェラルはOFF
void sensor_read(void) {
    enable_sensor_power();
    delay_ms(10);  // 安定待ち
    int value = read_adc();
    disable_sensor_power();  // 読み取り後はOFF
    return value;
}
```

## Build-Time Optimization

### コンパイラ最適化

```makefile
# リリースビルド
CFLAGS_RELEASE = -O2 -flto -DNDEBUG

# デバッグビルド
CFLAGS_DEBUG = -O0 -g3 -DDEBUG
```

### 未使用コードの削除

```makefile
# リンカで未使用セクションを削除
LDFLAGS = -Wl,--gc-sections
CFLAGS += -ffunction-sections -fdata-sections
```

## Error Resolution Strategy

ビルドエラー発生時:

1. **エラーメッセージを正確に読む**
2. **最初のエラーから対処**（後続エラーは派生の可能性）
3. **一度に1つずつ修正**
4. **修正ごとにビルド確認**

パフォーマンス問題発生時:

1. **計測してボトルネックを特定**（推測で最適化しない）
2. **プロファイラ/ロジアナを使用**
3. **最も効果の高い箇所から最適化**
4. **最適化前後で計測して効果を確認**
