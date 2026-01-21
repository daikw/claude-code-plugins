# Testing Rules

組込/エッジ開発におけるテストルール。

## Coverage Requirements

| テスト種別 | 対象 | 目標カバレッジ |
|-----------|------|---------------|
| Unit Test | 個別関数、モジュール | 80%+ |
| Integration Test | モジュール間連携 | 主要パス |
| Hardware-in-Loop | 実機での動作 | クリティカルパス |
| Simulation | 実機前の検証 | 全機能 |

## Test-Driven Development

### TDD サイクル（必須）

1. **失敗するテストを先に書く**
2. **テストが失敗することを確認**
3. **最小限の実装でテストを通す**
4. **テストが通ることを確認**
5. **リファクタリング（カバレッジ維持）**

### 組込向けTDDの注意点

```c
// ✅ ハードウェア依存部を抽象化してテスト可能に
typedef struct {
    int (*read_temperature)(void);
    void (*set_fan_speed)(int speed);
} hw_interface_t;

// テスト時はモックを注入
int mock_read_temperature(void) { return 25; }
void mock_set_fan_speed(int speed) { /* 記録 */ }

// 本番時は実ハードウェアを注入
int real_read_temperature(void) { return adc_read(TEMP_CHANNEL); }
void real_set_fan_speed(int speed) { pwm_set(FAN_CHANNEL, speed); }
```

## Simulation First

**実機にデプロイする前にシミュレーションで検証する。**

### シミュレーション対象

- 状態遷移ロジック
- 制御アルゴリズム
- プロトコル処理
- エラーハンドリング

### シミュレーションツール

| プラットフォーム | ツール |
|-----------------|--------|
| ESP32 | QEMU, Wokwi |
| STM32 | QEMU, Renode |
| Zephyr | Native POSIX, QEMU |
| ロボット | Gazebo, MuJoCo |

## Hardware-in-Loop Testing

### HIL テストの実施基準

- 制御ループの動作確認
- タイミングクリティカルな処理
- 電力状態遷移
- センサー/アクチュエータ連携

### HIL テスト環境

```
┌─────────────┐     ┌─────────────┐
│  テストPC   │────│  実デバイス  │
│  (pytest等) │    │  (DUT)      │
└─────────────┘     └─────────────┘
       │                  │
       └──── シリアル/USB ────┘
```

## Test Failure Handling

テストが失敗した場合:

1. **テスト条件を分離** - 他のテストの影響を排除
2. **モック実装を検証** - モックが正しく動作しているか
3. **実装を修正** - テストではなくコードを修正する（テスト自体にバグがある場合を除く）

## Stress Testing

長時間・高負荷テストの実施:

- **連続動作テスト**: 24時間以上の連続動作
- **温度サイクルテスト**: 動作温度範囲での繰り返し
- **電源サイクルテスト**: 電源ON/OFFの繰り返し
- **メモリリークテスト**: 長時間動作でのメモリ使用量監視

```bash
# メモリリーク検出（Valgrind、ホスト環境でのシミュレーション時）
valgrind --leak-check=full ./firmware_simulation
```

## Regression Testing

### 実施タイミング

- 機能追加時
- バグ修正時
- 依存ライブラリ更新時
- ツールチェイン更新時

### 自動化

```yaml
# CI/CD での自動テスト例
test:
  script:
    - west build -b native_posix
    - west test
    - ./run_hil_tests.sh  # HILテスト（実機接続時のみ）
```
