# Claude Code Plugin: Edge Robotics

エッジコンピューティング・ロボティクス・組込開発向けの Claude Code プラグイン。

## 設計原則

### Rules vs Skills の使い分け

| 種類 | 性質 | 対象 |
|------|------|------|
| **Rules** | 非交渉的（常に従う） | HW/SW 非依存の普遍的ガイドライン |
| **Skills** | 参照型（必要時に使う） | HW/SW 固有の知識・ワークフロー |

### Rules の原則

**ルールは特定のハードウェアや特定のソフトウェアに非依存のものに絞る。**

- ✅ 良い例: 「全通信にタイムアウト設定必須」「ISR内ではブロッキング禁止」
- ❌ 悪い例: 「ESP32のNVSは1000回書き込み制限」「STM32のHALを使う」

ハードウェア/ソフトウェア固有の知識は Skills として提供する。

### Skills の原則

- 特定のMCU、RTOS、フレームワーク、ロボットに関する知識
- 発動条件（When to Activate）を明確に定義
- 実践的なコード例とコマンドリファレンスを含む

### Agents の原則

- 自律的なタスク実行を担当
- 実行が必要な作業（スキャン、書き込み、診断など）に使用
- 知識参照だけなら Skills を使う

## ディレクトリ構造

```
.
├── skills/           # ワークフロー・ドメイン知識
│   ├── edge-common/  # エッジデバイス共通
│   ├── jetson/       # NVIDIA Jetson
│   ├── raspberry-pi/ # Raspberry Pi
│   ├── esp32/        # ESP32/ESP-IDF
│   ├── nrf/          # Nordic nRF
│   ├── pico/         # Raspberry Pi Pico
│   ├── stm32/        # STM32
│   ├── zephyr/       # Zephyr RTOS
│   ├── freertos/     # FreeRTOS
│   ├── arduino/      # Arduino
│   ├── embedded-basics/ # 組込基礎
│   ├── lerobot-basics/  # LeRobot トレーニング
│   ├── lerobot-fullstack/ # LeRobot デプロイ
│   ├── so-arm/       # SO-ARM100/101
│   └── jujutsu/      # jj VCS
├── agents/           # 特化型サブエージェント
│   ├── device-health-checker.md
│   ├── sensor-scanner.md
│   ├── model-optimizer.md
│   └── firmware-flasher.md
├── rules/            # 非交渉的ガイドライン
│   ├── agents.md         # エージェント委譲ルール
│   ├── security.md       # セキュリティルール
│   ├── testing.md        # テストルール
│   ├── patterns.md       # 設計パターン
│   ├── performance.md    # パフォーマンスルール
│   ├── hooks.md          # フック活用ルール
│   ├── memory-safety.md  # メモリ安全性
│   ├── realtime-safety.md # リアルタイム安全性
│   ├── design-principles.md # 設計原則
│   ├── state-concurrency.md # 状態管理・並行処理
│   ├── error-handling.md # エラーハンドリング
│   └── deploy-data.md    # デプロイ・データ整合性
└── docs/             # 設計ドキュメント
```

## Rules 一覧

| カテゴリ | ルール | 内容 |
|---------|--------|------|
| **基盤** | agents | エージェント自動起動タイミング |
| | security | 秘密情報管理、入力検証 |
| | testing | TDD、カバレッジ、HIL |
| | patterns | 設計パターン（HAL、State Machine等） |
| | performance | リソース効率、最適化 |
| | hooks | Claude Code フック活用 |
| **メモリ** | memory-safety | スタック/ヒープ/バッファ安全性 |
| **リアルタイム** | realtime-safety | デッドライン、ISR、Watchdog |
| **設計** | design-principles | 防御的プログラミング、Fail-safe |
| **並行性** | state-concurrency | 状態管理、デッドロック防止 |
| **エラー** | error-handling | エラー伝播、Graceful degradation |
| **デプロイ** | deploy-data | OTA、データ整合性、電源断対策 |

## 参考

- [everything-claude-code](https://github.com/affaan-m/everything-claude-code) - 構造の参考元
