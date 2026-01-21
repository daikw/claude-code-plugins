# Deploy & Data Integrity Rules

ファームウェアデプロイメントとデータ整合性に関するルール。
ハードウェア/ソフトウェア非依存の汎用ガイドライン。

## Pre-Commit Checklist

コミット前に必ず確認する:

### OTA Update Safety
- [ ] ロールバック機構が実装されている（A/B パーティション or デュアルバンク）
- [ ] 署名検証が必須となっている（未署名ファームウェアは拒否）
- [ ] バージョンチェックが実装されている（ダウングレード防止）
- [ ] 更新失敗時のフォールバック処理がある
- [ ] 更新前に電源/ストレージ容量/通信状態を確認している

### Bootloader Protection
- [ ] ブートローダー領域は書き込み保護されている
- [ ] セキュアブートチェーンが維持されている
- [ ] ロールバックロジックがブートローダーに含まれている

### Data Integrity
- [ ] CRC/チェックサム検証が実装されている
- [ ] 冗長ストレージまたはミラーリングがある
- [ ] 電源断耐性のある書き込み処理（アトミック書き込み/ジャーナリング）

### Configuration Safety
- [ ] 設定値のスキーマバリデーションがある
- [ ] 無効な設定時のデフォルト値フォールバックがある
- [ ] ファクトリーリセット機能が実装されている

---

## 1. OTA Update Safety

### A/B パーティション設計

デュアルバンク（A/B）構成により、更新失敗時も確実に復旧できる。

```c
// ✅ Good: デュアルバンク更新フロー
typedef enum {
    PARTITION_A,
    PARTITION_B
} partition_t;

typedef struct {
    partition_t active;
    partition_t update_target;
    uint32_t version_a;
    uint32_t version_b;
    bool boot_verified;
} boot_info_t;

result_t ota_update(const uint8_t* firmware, size_t len) {
    // 1. 非アクティブパーティションを特定
    partition_t target = (boot_info.active == PARTITION_A)
                         ? PARTITION_B : PARTITION_A;

    // 2. 署名検証
    if (!verify_signature(firmware, len)) {
        return RESULT_ERROR_INVALID_SIGNATURE;
    }

    // 3. バージョンチェック（ダウングレード防止）
    if (!is_version_newer(firmware)) {
        return RESULT_ERROR_VERSION_DOWNGRADE;
    }

    // 4. 非アクティブ領域に書き込み
    if (!write_to_partition(target, firmware, len)) {
        return RESULT_ERROR_WRITE_FAILED;
    }

    // 5. 書き込み検証
    if (!verify_written_data(target, firmware, len)) {
        return RESULT_ERROR_VERIFY_FAILED;
    }

    // 6. ブートフラグを「pending」に設定（まだ切り替えない）
    set_boot_pending(target);

    return RESULT_OK;
}
```

```c
// ❌ Bad: シングルパーティション上書き
result_t ota_update_bad(const uint8_t* firmware, size_t len) {
    // 現在動作中のパーティションを直接上書き
    // → 更新失敗時に復旧不可能（文鎮化）
    erase_current_partition();
    write_firmware(firmware, len);
    reboot();
}
```

### ブート成功確認とロールバック

```c
// ✅ Good: タイムアウト付きブート検証
void bootloader_main(void) {
    boot_info_t* info = get_boot_info();

    // 新ファームウェアでブート試行中？
    if (info->boot_pending) {
        info->boot_attempts++;

        // 試行回数上限を超えたらロールバック
        if (info->boot_attempts > MAX_BOOT_ATTEMPTS) {
            rollback_to_previous();
            return;
        }
    }

    // ファームウェア起動
    jump_to_application(info->active);
}

// アプリケーション側：起動成功を報告
void app_mark_boot_successful(void) {
    boot_info_t* info = get_boot_info();
    info->boot_pending = false;
    info->boot_attempts = 0;
    info->boot_verified = true;
    save_boot_info();
}
```

### 更新前の事前チェック

```c
// ✅ Good: 更新前の状態確認
result_t check_update_readiness(void) {
    // 電源レベル確認
    if (get_battery_level() < MINIMUM_BATTERY_FOR_UPDATE) {
        return RESULT_ERROR_LOW_BATTERY;
    }

    // ストレージ空き容量確認
    if (get_free_storage() < required_space) {
        return RESULT_ERROR_NO_SPACE;
    }

    // 通信状態確認
    if (!is_connection_stable()) {
        return RESULT_ERROR_UNSTABLE_CONNECTION;
    }

    return RESULT_OK;
}
```

---

## 2. Bootloader Protection

### ブートローダー領域の保護

```c
// ✅ Good: ブートローダー書き込み防止
#define BOOTLOADER_START    0x00000000
#define BOOTLOADER_END      0x00008000
#define BOOTLOADER_SIZE     (BOOTLOADER_END - BOOTLOADER_START)

bool is_write_allowed(uint32_t address, size_t len) {
    // ブートローダー領域への書き込みは常に禁止
    if (address < BOOTLOADER_END) {
        return false;
    }
    // オーバーフローチェック
    if (address + len < address) {
        return false;
    }
    return true;
}

result_t flash_write(uint32_t address, const uint8_t* data, size_t len) {
    if (!is_write_allowed(address, len)) {
        return RESULT_ERROR_WRITE_PROTECTED;
    }
    // 実際の書き込み処理
    return flash_program(address, data, len);
}
```

```c
// ❌ Bad: 任意アドレスへの書き込み許可
result_t flash_write_bad(uint32_t address, const uint8_t* data, size_t len) {
    // アドレス検証なしで書き込み
    // → ブートローダー破壊の危険性
    return flash_program(address, data, len);
}
```

### セキュアブートチェーン

```c
// ✅ Good: 署名検証チェーン
typedef struct {
    uint8_t signature[SIGNATURE_SIZE];
    uint32_t version;
    uint32_t size;
    uint32_t entry_point;
} firmware_header_t;

bool verify_boot_chain(void) {
    // 1. ブートローダー自身の整合性（ROM から検証）
    if (!verify_bootloader_integrity()) {
        halt_system();
        return false;
    }

    // 2. ファームウェアヘッダの署名検証
    firmware_header_t* header = get_firmware_header();
    if (!verify_firmware_signature(header)) {
        return false;
    }

    // 3. ファームウェア本体のハッシュ検証
    if (!verify_firmware_hash(header)) {
        return false;
    }

    // 4. バージョンカウンタ確認（ロールバック攻撃防止）
    if (header->version < get_minimum_version()) {
        return false;
    }

    return true;
}
```

---

## 3. Firmware Versioning

### セマンティックバージョニング

```c
// ✅ Good: 32ビット整数エンコードによるバージョン管理
typedef struct {
    uint8_t major;    // 互換性のない変更
    uint8_t minor;    // 後方互換の機能追加
    uint8_t patch;    // バグ修正
    uint8_t build;    // ビルド番号
} version_t;

#define VERSION_ENCODE(major, minor, patch, build) \
    (((uint32_t)(major) << 24) | \
     ((uint32_t)(minor) << 16) | \
     ((uint32_t)(patch) << 8)  | \
     ((uint32_t)(build)))

#define CURRENT_VERSION VERSION_ENCODE(2, 5, 3, 0)

// バージョン比較（単純な整数比較で新旧判定可能）
bool is_version_newer(uint32_t new_ver, uint32_t current_ver) {
    return new_ver > current_ver;
}

// 互換性チェック（同一メジャーバージョン内のみ更新許可）
bool is_version_compatible(version_t* new_ver, version_t* current) {
    return new_ver->major == current->major;
}
```

```c
// ❌ Bad: 文字列ベースのバージョン比較
bool is_version_newer_bad(const char* new_ver, const char* current) {
    // 文字列比較は "2.10.0" < "2.9.0" となる問題あり
    return strcmp(new_ver, current) > 0;
}
```

### ハードウェア互換性チェック

```c
// ✅ Good: ハードウェアバージョンとの互換性確認
typedef struct {
    uint32_t fw_version;
    uint32_t min_hw_version;  // 最小対応ハードウェアバージョン
    uint32_t max_hw_version;  // 最大対応ハードウェアバージョン
} compatibility_info_t;

bool check_hw_compatibility(const compatibility_info_t* info) {
    uint32_t hw_version = read_hardware_version();

    if (hw_version < info->min_hw_version) {
        log_error("Hardware too old for this firmware");
        return false;
    }

    if (hw_version > info->max_hw_version) {
        log_error("Firmware not tested with this hardware");
        return false;
    }

    return true;
}
```

---

## 4. Factory Reset

### 既知の正常状態への復元

```c
// ✅ Good: 段階的ファクトリーリセット
typedef enum {
    RESET_CONFIG_ONLY,      // 設定のみリセット
    RESET_USER_DATA,        // ユーザーデータもクリア
    RESET_FULL_FACTORY,     // 完全なファクトリー状態
} reset_level_t;

typedef struct {
    uint8_t factory_config[CONFIG_SIZE];
    uint32_t factory_config_crc;
} factory_data_t;

// ファクトリーデータは書き込み保護領域に保存
const factory_data_t factory_data __attribute__((section(".factory_ro")));

result_t perform_factory_reset(reset_level_t level) {
    // 1. ファクトリーデータの整合性確認
    if (!verify_factory_data()) {
        return RESULT_ERROR_FACTORY_DATA_CORRUPT;
    }

    // 2. 現在の設定をバックアップ（オプション）
    if (level == RESET_CONFIG_ONLY) {
        backup_current_config();
    }

    // 3. レベルに応じたリセット実行
    switch (level) {
        case RESET_CONFIG_ONLY:
            restore_config_from_factory();
            break;
        case RESET_USER_DATA:
            clear_user_data();
            restore_config_from_factory();
            break;
        case RESET_FULL_FACTORY:
            clear_all_nvs();
            restore_config_from_factory();
            reset_provisioning_state();
            break;
    }

    return RESULT_OK;
}
```

### 設定バックアップ

```c
// ✅ Good: 設定のエクスポート/インポート
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t crc;
    uint8_t encrypted_data[];
} config_backup_t;

result_t export_config(uint8_t* buffer, size_t* len) {
    config_backup_t* backup = (config_backup_t*)buffer;

    // 機密情報は暗号化してエクスポート
    backup->magic = CONFIG_BACKUP_MAGIC;
    backup->version = CONFIG_BACKUP_VERSION;

    size_t data_len;
    encrypt_config(current_config, backup->encrypted_data, &data_len);

    backup->size = data_len;
    backup->crc = calculate_crc(backup->encrypted_data, data_len);

    *len = sizeof(config_backup_t) + data_len;
    return RESULT_OK;
}
```

---

## 5. Data Integrity

### チェックサム/CRC検証

```c
// ✅ Good: CRC32による整合性検証
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t data[DATA_SIZE];
    uint32_t crc32;  // 末尾にCRC配置
} stored_data_t;

bool verify_data_integrity(const stored_data_t* data) {
    // マジックナンバー確認
    if (data->magic != EXPECTED_MAGIC) {
        return false;
    }

    // CRC計算（CRCフィールド自体を除く）
    uint32_t calculated = crc32(data, sizeof(stored_data_t) - sizeof(uint32_t));

    return calculated == data->crc32;
}

result_t save_data(const stored_data_t* data) {
    stored_data_t temp = *data;

    // 保存前にCRC計算
    temp.crc32 = crc32(&temp, sizeof(stored_data_t) - sizeof(uint32_t));

    return write_to_storage(&temp, sizeof(temp));
}
```

### 冗長ストレージ（ミラーリング）

```c
// ✅ Good: 二重化ストレージによる耐障害性
#define CONFIG_SLOT_A_ADDR  0x10000
#define CONFIG_SLOT_B_ADDR  0x20000

typedef struct {
    stored_data_t data;
    uint32_t write_count;  // 書き込み回数（新しい方を選択）
} redundant_slot_t;

result_t read_redundant_data(stored_data_t* output) {
    redundant_slot_t slot_a, slot_b;

    read_from_storage(CONFIG_SLOT_A_ADDR, &slot_a, sizeof(slot_a));
    read_from_storage(CONFIG_SLOT_B_ADDR, &slot_b, sizeof(slot_b));

    bool a_valid = verify_data_integrity(&slot_a.data);
    bool b_valid = verify_data_integrity(&slot_b.data);

    if (a_valid && b_valid) {
        // 両方有効なら新しい方を使用
        *output = (slot_a.write_count > slot_b.write_count)
                  ? slot_a.data : slot_b.data;
    } else if (a_valid) {
        *output = slot_a.data;
    } else if (b_valid) {
        *output = slot_b.data;
    } else {
        // 両方破損 → ファクトリーデフォルトにフォールバック
        load_factory_defaults(output);
        return RESULT_ERROR_DATA_CORRUPT;
    }

    return RESULT_OK;
}
```

---

## 6. Configuration Validation

### スキーマバリデーション

```c
// ✅ Good: 設定値の範囲・型検証
typedef struct {
    const char* key;
    int32_t min_value;
    int32_t max_value;
    int32_t default_value;
} config_schema_t;

static const config_schema_t config_schema[] = {
    {"sensor_interval_ms", 100, 60000, 1000},
    {"wifi_retry_count",   1,   10,    3},
    {"log_level",          0,   4,     2},
    {NULL, 0, 0, 0}  // 終端
};

result_t validate_config_value(const char* key, int32_t value, int32_t* validated) {
    for (const config_schema_t* s = config_schema; s->key; s++) {
        if (strcmp(s->key, key) == 0) {
            if (value < s->min_value || value > s->max_value) {
                // 範囲外 → デフォルト値にフォールバック
                *validated = s->default_value;
                log_warning("Config '%s' out of range, using default: %d",
                           key, s->default_value);
                return RESULT_WARNING_USING_DEFAULT;
            }
            *validated = value;
            return RESULT_OK;
        }
    }
    return RESULT_ERROR_UNKNOWN_KEY;
}
```

```c
// ❌ Bad: バリデーションなしの設定適用
void apply_config_bad(const char* key, int32_t value) {
    // 検証なしで直接適用
    // → 不正値でシステム動作異常の可能性
    config_set(key, value);
}
```

### デフォルト値フォールバック

```c
// ✅ Good: 読み込み失敗時のフォールバック
int32_t config_get_with_default(const char* key) {
    int32_t value;
    result_t result = config_read(key, &value);

    if (result != RESULT_OK) {
        // 読み込み失敗 → スキーマからデフォルト値を取得
        value = get_default_from_schema(key);
        log_info("Using default value for '%s': %d", key, value);
    }

    return value;
}
```

---

## 7. Non-Volatile Storage Usage

### 書き込み寿命の考慮

```c
// ✅ Good: 書き込み頻度の制限
typedef struct {
    uint32_t last_save_time;
    bool pending_save;
    uint8_t pending_data[DATA_SIZE];
} deferred_save_t;

// 最小書き込み間隔（5分）
#define MIN_SAVE_INTERVAL_MS  (5 * 60 * 1000)

result_t save_config_throttled(const uint8_t* data, size_t len) {
    uint32_t now = get_system_time_ms();

    // 前回保存から十分な時間が経過していない場合は遅延
    if (now - deferred_save.last_save_time < MIN_SAVE_INTERVAL_MS) {
        // ペンディングとしてRAMに保持
        memcpy(deferred_save.pending_data, data, len);
        deferred_save.pending_save = true;
        return RESULT_PENDING;
    }

    // 実際に書き込み
    result_t result = write_to_flash(data, len);
    if (result == RESULT_OK) {
        deferred_save.last_save_time = now;
        deferred_save.pending_save = false;
    }

    return result;
}
```

```c
// ❌ Bad: 頻繁な書き込み
void log_sensor_value_bad(int value) {
    // 1秒ごとにフラッシュに書き込み
    // → 1日で86,400回書き込み、数週間で寿命到達の可能性
    flash_write(&value, sizeof(value));
}
```

### ウェアレベリング対応

```c
// ✅ Good: ウェアレベリングを考慮したストレージ使用
// - ファイルシステム（littlefs, SPIFFS等）を使用
// - 同一アドレスへの連続書き込みを避ける
// - バッファリングして一括書き込み

typedef struct {
    uint8_t buffer[WRITE_BUFFER_SIZE];
    size_t buffer_pos;
} write_buffer_t;

result_t buffered_write(write_buffer_t* wb, const uint8_t* data, size_t len) {
    // バッファに追加
    if (wb->buffer_pos + len > WRITE_BUFFER_SIZE) {
        // バッファが満杯 → フラッシュ
        result_t result = flush_buffer(wb);
        if (result != RESULT_OK) return result;
    }

    memcpy(&wb->buffer[wb->buffer_pos], data, len);
    wb->buffer_pos += len;

    return RESULT_OK;
}
```

---

## 8. Power-Loss Resilience

### アトミック書き込みパターン

```c
// ✅ Good: Copy-on-Write パターン
result_t atomic_config_update(const config_t* new_config) {
    // 1. 新しいデータを別領域に書き込み
    result_t result = write_to_temp_area(new_config);
    if (result != RESULT_OK) {
        return result;
    }

    // 2. 書き込みを検証
    if (!verify_temp_area(new_config)) {
        return RESULT_ERROR_VERIFY_FAILED;
    }

    // 3. コミットフラグを立てる（アトミック操作）
    // この時点で電源断が発生しても、次回起動時に
    // フラグを確認してリカバリ可能
    set_commit_flag(COMMIT_IN_PROGRESS);

    // 4. ポインタ/インデックスを切り替え
    switch_active_config_slot();

    // 5. コミット完了
    set_commit_flag(COMMIT_COMPLETE);

    return RESULT_OK;
}

void recovery_on_boot(void) {
    if (get_commit_flag() == COMMIT_IN_PROGRESS) {
        // 前回の書き込みが中断された
        // → 古いデータにロールバック
        rollback_config_slot();
    }
}
```

```c
// ❌ Bad: 非アトミックな上書き
result_t update_config_bad(const config_t* new_config) {
    // 既存データを直接上書き
    // → 書き込み途中で電源断が発生すると破損
    erase_config_area();
    write_config(new_config);
}
```

### ジャーナリングパターン

```c
// ✅ Good: トランザクションログによる一貫性保証
typedef struct {
    uint32_t transaction_id;
    uint8_t operation;
    uint8_t data[JOURNAL_DATA_SIZE];
    uint32_t crc;
    uint8_t committed;  // 0: pending, 1: committed, 2: aborted
} journal_entry_t;

result_t journaled_write(const uint8_t* data, size_t len) {
    journal_entry_t entry = {
        .transaction_id = get_next_transaction_id(),
        .operation = OP_WRITE,
        .committed = 0,  // pending
    };
    memcpy(entry.data, data, len);
    entry.crc = crc32(&entry, offsetof(journal_entry_t, crc));

    // 1. ジャーナルに記録
    append_to_journal(&entry);

    // 2. 実際のデータ書き込み
    result_t result = write_actual_data(data, len);

    // 3. コミット or ロールバック
    entry.committed = (result == RESULT_OK) ? 1 : 2;
    update_journal_entry(&entry);

    return result;
}

void replay_journal_on_boot(void) {
    journal_entry_t entry;

    while (read_next_journal_entry(&entry)) {
        if (entry.committed == 0) {
            // 未コミット → ロールバック
            rollback_operation(&entry);
        } else if (entry.committed == 1) {
            // コミット済み → 必要なら再適用
            if (!verify_operation_completed(&entry)) {
                replay_operation(&entry);
            }
        }
        // committed == 2 (aborted) は無視
    }

    compact_journal();  // 古いエントリを削除
}
```

### fsync の適切な使用

```c
// ✅ Good: 重要データの確実な永続化
result_t save_critical_data(const uint8_t* data, size_t len) {
    int fd = open(CRITICAL_DATA_PATH, O_WRONLY | O_CREAT);
    if (fd < 0) return RESULT_ERROR_OPEN_FAILED;

    // データ書き込み
    if (write(fd, data, len) != len) {
        close(fd);
        return RESULT_ERROR_WRITE_FAILED;
    }

    // fsync でバッファをフラッシュ
    // → 電源断時もデータが永続化されていることを保証
    if (fsync(fd) != 0) {
        close(fd);
        return RESULT_ERROR_SYNC_FAILED;
    }

    close(fd);
    return RESULT_OK;
}
```

```c
// ❌ Bad: fsync なしでの重要データ書き込み
result_t save_critical_data_bad(const uint8_t* data, size_t len) {
    FILE* f = fopen(CRITICAL_DATA_PATH, "w");
    fwrite(data, 1, len, f);
    fclose(f);
    // → データがOSバッファに残っている可能性
    // → 電源断でデータ消失の危険
}
```

---

## Summary: 重要原則

1. **Always have a rollback path** - 更新は常にロールバック可能に
2. **Verify before trust** - 署名・CRC・ハッシュで検証してから使用
3. **Protect the bootloader** - ブートローダーは聖域として保護
4. **Fail safe, not fail silent** - 失敗時は安全側に倒す、黙って進めない
5. **Atomic or nothing** - 中途半端な状態を残さない
6. **Redundancy is resilience** - 冗長性は耐障害性
7. **Respect flash limits** - フラッシュの書き込み寿命を尊重
8. **Test power-loss scenarios** - 電源断シナリオを必ずテスト

---

## References

- [OTA Update Checklist for Embedded Devices - Memfault](https://memfault.com/blog/ota-update-checklist-for-embedded-devices/)
- [OTA Updates Best Practices - Mender.io](https://mender.io/resources/reports-and-guides/ota-updates-best-practices)
- [Secure OTA Boot Chains and Firmware Verification - Promwad](https://promwad.com/news/secure-ota-boot-chains-firmware-verification)
- [littlefs Design Documentation](https://github.com/littlefs-project/littlefs/blob/master/DESIGN.md)
- [Improving Reliability of Non-Volatile Memory Systems - Embedded.com](https://www.embedded.com/improving-reliability-of-non-volatile-memory-systems/)
- [Proper Release Versioning - Memfault Interrupt](https://interrupt.memfault.com/blog/release-versioning)
- [Firmware Update Failure: Why Devices Must Fail Gracefully - Beningo](https://www.beningo.com/firmware-update-failure-why-devices-must-fail-gracefully/)
