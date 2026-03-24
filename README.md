# Claude Code Plugins

Claude Code のスキル・エージェント・ルールのプラグインコレクション。

## プラグイン一覧

### general-skills

汎用的な開発ワークフロー向けスキル・エージェント。

| カテゴリ | スキル | 説明 |
|----------|--------|------|
| 開発 | `swarm-dev` | チーム編成による並列開発 |
| 開発 | `swarm-dev-v2` | swarm-dev 軽量版 |
| 開発 | `issue-dev` | GitHub/GitLab Issue → 実装パイプライン |
| 開発 | `skill-creator` | スキル設計・作成ガイド |
| CI/CD | `fix-ci` | GitHub Actions/GitLab CI 失敗修復 |
| CI/CD | `commit-push` | VCS 自動検出（git/jj）コミット・プッシュ |
| VCS | `jujutsu-workflow` | Jujutsu (jj) ワークフロー |
| 品質 | `codex` | Codex CLI 連携（セカンドオピニオン） |
| 品質 | `coding-agent-spec` | コーディングエージェント仕様比較 |
| 品質 | `agent-skill-testing` | エージェントスクリプトのテスト設計 |
| テキスト | `deodorize-ai` | AI臭の除去（日本語テキスト校正） |
| ビジネス | `business-evaluation` | 事業評価（投資判断/DD/Go-NoGo） |
| インフラ | `terraform-mermaid` | Terraform → Mermaid 構成図 |
| インフラ | `xpoint-server-apply` | サーバ構築申請フォーム記入支援 |
| ユーティリティ | `meta-rules` | Rules/Skills/Agents 設計原則 |
| ユーティリティ | `move-project` | プロジェクト移動時の履歴追跡 |
| ユーティリティ | `history-search` | 会話履歴の横断検索 |

**Agents**: planner, architect, tdd-guide, code-reviewer, security-reviewer, refactoring-suggester, codex

### edge-robotics-skills

エッジコンピューティング・ロボティクス・組込開発向け。

| カテゴリ | スキル | 説明 |
|----------|--------|------|
| 共通 | `edge-common` | センサ・アクチュエータカタログ、通信プロトコル |
| 共通 | `embedded-basics` | 組込開発基礎 |
| SBC | `jetson` | NVIDIA Jetson（tegrastats、CUDA、JetPack） |
| SBC | `raspberry-pi` | Raspberry Pi（GPIO、周辺機器） |
| MCU | `esp32` | ESP32/ESP-IDF |
| MCU | `stm32` | STM32/HAL |
| MCU | `nrf` | Nordic nRF/Zephyr |
| MCU | `pico` | Raspberry Pi Pico |
| RTOS | `zephyr` | Zephyr RTOS |
| RTOS | `freertos` | FreeRTOS |
| RTOS | `arduino` | Arduino フレームワーク |
| ロボティクス | `lerobot-basics` | LeRobot トレーニング |
| ロボティクス | `lerobot-fullstack` | LeRobot デプロイ |
| ロボティクス | `so-arm` | SO-ARM100/101 |
| VCS | `jujutsu` | jj VCS |

**Agents**: device-health-checker, sensor-scanner, model-optimizer, firmware-flasher

## インストール

### プラグイン（skills + agents）

```bash
# general スキルをインストール
claude plugin install general-skills@daikw

# edge-robotics スキルをインストール
claude plugin install edge-robotics-skills@daikw
```

### ルール（install-rules.sh）

ルールはプラグインシステムでは配布できないため、スクリプトでインストールする。

```bash
# リポジトリをクローン
git clone https://github.com/daikw/claude-code-plugins.git
cd claude-code-plugins

# general ルールのみインストール
./install-rules.sh general

# edge-robotics ルールのみインストール
./install-rules.sh edge-robotics

# 全ルールをインストール
./install-rules.sh all

# dry-run（何がインストールされるか確認）
./install-rules.sh all --dry-run
```

ルールは `~/.claude/rules/` に `{plugin}--{rule}.md` の形式でインストールされる（例: `general--git-workflow.md`）。

## ライセンス

MIT
