# Security Rules

組込/エッジ開発におけるセキュリティルール。

## Pre-Commit Checklist

コミット前に必ず確認する:

- [ ] ハードコードされた秘密情報がないこと（API keys, passwords, tokens, certificates）
- [ ] WiFi SSID/パスワードがハードコードされていないこと
- [ ] 全ユーザー入力をバリデーション
- [ ] シリアル/UART入力のサニタイズ
- [ ] ネットワーク入力の境界チェック
- [ ] エラーメッセージに機密情報を含めない

## Secret Management

### ❌ Bad Examples

```c
// ハードコードされた秘密情報
const char* wifi_ssid = "MyNetwork";
const char* wifi_pass = "MyPassword123";
const char* api_key = "sk-proj-xxxxx";
```

### ✅ Good Examples

```c
// 環境変数または設定ファイルから読み込み
const char* wifi_ssid = getenv("WIFI_SSID");
const char* wifi_pass = getenv("WIFI_PASSWORD");

// NVS（不揮発性ストレージ）から読み込み
nvs_get_str(nvs_handle, "api_key", api_key, &length);

// ビルド時に注入
#ifndef API_KEY
#error "API_KEY must be defined at build time"
#endif
```

## Input Validation

### シリアル/UART入力

```c
// ✅ バッファオーバーフロー防止
char buffer[64];
size_t len = uart_read(buffer, sizeof(buffer) - 1);
buffer[len] = '\0';

// ✅ コマンドインジェクション防止
if (!is_valid_command(buffer)) {
    log_warning("Invalid command received");
    return;
}
```

### ネットワーク入力

```c
// ✅ 長さチェック
if (packet_len > MAX_PACKET_SIZE) {
    return ERROR_PACKET_TOO_LARGE;
}

// ✅ 形式検証
if (!validate_packet_format(packet)) {
    return ERROR_INVALID_FORMAT;
}
```

## Embedded-Specific Security

### Debug Interface Protection

```c
// 本番ビルドではデバッグインターフェースを無効化
#ifdef PRODUCTION
    disable_jtag();
    disable_swd();
    disable_uart_debug();
#endif
```

### Secure Boot

- ファームウェア署名を検証してから実行
- 未署名/不正署名のファームウェアは拒否
- ロールバック攻撃への対策（バージョンカウンタ）

### Key Storage

- 暗号鍵は eFuse またはセキュアエレメントに保存
- RAM上の鍵は使用後にゼロクリア
- 鍵のハードコードは絶対禁止

## Incident Response

セキュリティ問題を発見した場合:

1. **即座に作業を停止**
2. **影響範囲を特定**
3. **漏洩した秘密情報はローテーション**
4. **類似問題がないかコードベース全体を確認**
5. **修正をコミットする前にレビュー**
