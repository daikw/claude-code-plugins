# Rules 候補一覧

## 設計原則

**ルールは特定のハードウェアや特定のソフトウェアに非依存のものに絞る。**

ハードウェア/ソフトウェア固有の知識は Skills として提供する。

---

## everything-claude-code から取り入れる候補

| ルール | 内容 | HW/SW依存 |
|--------|------|-----------|
| `agents.md` | エージェント自動起動のタイミング定義 | なし |
| `security.md` | 秘密情報・入力検証 | なし |
| `testing.md` | TDD・カバレッジ要件 | なし |
| `patterns.md` | デザインパターン指針 | なし |
| `performance.md` | 最適化ルール | なし |
| `hooks.md` | フック機能の使い方 | なし |

---

## 組込/エッジ/ロボティクス特化候補

### ハードウェア安全系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `hardware-safety.md` | GPIO操作前の方向設定確認、電圧レベル確認、短絡防止 | △ GPIO概念は普遍的 |
| `power-management.md` | 電源投入順序、突入電流対策、バッテリー残量監視 | △ 概念は普遍的 |
| `motor-safety.md` | モーター制御時のリミット設定、緊急停止処理、トルク制限 | × モーター固有 |
| `thermal-protection.md` | 温度監視、サーマルスロットリング、過熱時の自動シャットダウン | △ 概念は普遍的 |
| `esd-protection.md` | 静電気対策、グラウンド接続確認 | × 物理的作業 |

### メモリ・リソース管理系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `memory-safety.md` | スタックオーバーフロー防止、ヒープ断片化対策、静的割り当て推奨 | なし |
| `resource-limits.md` | RAM/Flash使用量の上限設定、リソース枯渇時の挙動定義 | なし |
| `buffer-safety.md` | バッファオーバーラン防止、境界チェック必須 | なし |
| `allocation-policy.md` | 動的メモリ割り当ての制限、FreeRTOS heap選択基準 | △ FreeRTOS固有 |

### リアルタイム・タイミング系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `realtime-constraints.md` | デッドライン遵守、割り込みレイテンシ制限、優先度逆転防止 | なし |
| `watchdog-policy.md` | ウォッチドッグタイマー必須、フィード間隔の設計 | なし |
| `timing-critical.md` | タイミングクリティカルな処理の分離、ブロッキング禁止 | なし |
| `isr-rules.md` | ISR内での処理制限（短く、ブロッキングなし、printf禁止） | なし |

### 通信・プロトコル系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `communication-timeout.md` | 全通信にタイムアウト設定必須、リトライ回数制限 | なし |
| `protocol-validation.md` | 受信データのCRC/チェックサム検証、不正パケット破棄 | なし |
| `bus-arbitration.md` | I2C/SPIバスの排他制御、デッドロック防止 | △ I2C/SPI固有 |
| `network-resilience.md` | WiFi/BLE切断時の再接続処理、オフライン動作モード | △ WiFi/BLE固有 |

### ロボット制御系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `robot-safety.md` | 関節角度リミット、速度制限、衝突検知時の停止 | × ロボット固有 |
| `calibration-required.md` | キャリブレーション未実施での動作禁止、キャリブデータ検証 | × ロボット/センサー固有 |
| `sensor-validation.md` | センサー値の範囲チェック、異常値フィルタリング | △ 概念は普遍的 |
| `actuator-limits.md` | アクチュエータの物理的限界値設定、ソフトリミット | × アクチュエータ固有 |
| `emergency-stop.md` | E-Stop実装必須、全アクチュエータ即時停止機能 | × ロボット固有 |

### デバッグ・ログ系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `logging-policy.md` | ログレベル定義、本番でのデバッグログ無効化 | なし |
| `assert-usage.md` | 開発時のassert活用、本番での挙動定義 | なし |
| `error-reporting.md` | エラーコード体系、障害情報の永続化 | なし |
| `debug-interface.md` | デバッグ用シリアル/RTTの有効化条件 | △ RTT固有 |

### デプロイ・OTA系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `ota-safety.md` | OTA更新時のロールバック機能必須、署名検証 | なし |
| `bootloader-protection.md` | ブートローダー書き換え禁止、セキュアブート | なし |
| `firmware-versioning.md` | バージョン管理、互換性チェック | なし |
| `factory-reset.md` | 工場出荷状態への復帰手順、設定のバックアップ | なし |

### データ・永続化系

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `nvs-usage.md` | 不揮発性ストレージの使い方、書き込み回数制限 | △ NVS実装固有 |
| `data-integrity.md` | データ破損検出、冗長化、電源断対策 | なし |
| `config-validation.md` | 設定値のスキーマ検証、デフォルト値フォールバック | なし |

### セキュリティ（組込特化）

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `secure-boot.md` | セキュアブート設定、署名検証 | なし |
| `key-storage.md` | 暗号鍵の安全な保管（eFuse、セキュアエレメント） | △ ハードウェア依存 |
| `debug-disable.md` | 本番でのJTAG/SWD無効化 | なし |
| `firmware-signing.md` | ファームウェア署名と検証 | なし |

### 品質・テスト（組込特化）

| ルール名 | 内容 | HW/SW依存 |
|---------|------|-----------|
| `hardware-in-loop.md` | HILテストの実施基準 | なし |
| `simulation-first.md` | 実機前のシミュレーション必須 | なし |
| `regression-testing.md` | ハードウェア依存部のリグレッションテスト | なし |
| `stress-testing.md` | 長時間動作テスト、温度サイクルテスト | なし |

---

## 分類結果

### ルールとして適切（HW/SW非依存）

- agents, security, testing, patterns, performance, hooks（everything-claude-code）
- memory-safety, resource-limits, buffer-safety
- realtime-constraints, watchdog-policy, timing-critical, isr-rules
- communication-timeout, protocol-validation
- logging-policy, assert-usage, error-reporting
- ota-safety, bootloader-protection, firmware-versioning, factory-reset
- data-integrity, config-validation
- secure-boot, debug-disable, firmware-signing
- hardware-in-loop, simulation-first, regression-testing, stress-testing

### スキルに移動すべき（HW/SW依存）

- motor-safety → so-arm, lerobot-fullstack スキルへ
- esd-protection → edge-common スキルへ
- allocation-policy → freertos スキルへ
- bus-arbitration → edge-common スキルへ（I2C/SPI セクション）
- network-resilience → esp32, nrf スキルへ
- robot-safety, calibration-required, actuator-limits, emergency-stop → so-arm, lerobot-fullstack スキルへ
- debug-interface → embedded-basics スキルへ（RTTセクション）
- nvs-usage → esp32 スキルへ
- key-storage → 各MCUスキルへ（eFuse/セキュアエレメント）

### 境界線上（概念は普遍的だが具体例がHW依存）

- hardware-safety, power-management, thermal-protection
- sensor-validation

---

## TODO

- [ ] 上記分析を踏まえてルール候補を絞り込む
- [ ] 追加のルール候補を検討
- [ ] スキルに移動すべき内容を各スキルに追記
