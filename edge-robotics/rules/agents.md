# Agent Orchestration Rules

エージェント委譲のルール。適切なタイミングで適切なエージェントを起動する。

## Available Agents

| エージェント | 役割 | 起動タイミング |
|-------------|------|---------------|
| `device-health-checker` | システム健全性診断 | デバイスが遅い/不安定な時 |
| `sensor-scanner` | センサー/カメラ/マイク検出 | 新デバイス接続時、認識問題時 |
| `model-optimizer` | MLモデル最適化 | エッジデプロイ準備時 |
| `firmware-flasher` | ファームウェア書き込み | MCUへの書き込み時 |

## Automatic Agent Selection

以下の状況では明示的な要求を待たず、即座にエージェントを起動する:

| 状況 | エージェント |
|------|-------------|
| デバイスの動作が遅い/不安定 | device-health-checker |
| 「センサーが認識されない」「カメラが映らない」 | sensor-scanner |
| 「推論が遅い」「モデルをデプロイしたい」 | model-optimizer |
| 「ファームウェアを書き込みたい」「MCUが認識されない」 | firmware-flasher |

## Execution Strategies

### Parallel Processing

独立したタスクは逐次ではなく並列で実行する。

```
# ✅ Good: 並列実行
Task(sensor-scanner) + Task(device-health-checker)

# ❌ Bad: 逐次実行
Task(sensor-scanner) → wait → Task(device-health-checker)
```

### Multi-Perspective Analysis

複雑な問題には複数の視点でレビュー:

- **Hardware perspective**: 配線、電圧、物理的接続
- **Software perspective**: ドライバ、設定、権限
- **System perspective**: リソース競合、タイミング
- **Security perspective**: 認証、暗号化、アクセス制御

## Agent vs Skill Selection

| 必要なこと | 選択 |
|-----------|------|
| 知識・手順の参照 | Skill |
| 実際の実行・診断 | Agent |
| 調査 + 実行 | Skill で学習 → Agent で実行 |
