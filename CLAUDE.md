# Claude Code Plugins by daikw

マルチプラグインリポジトリ。汎用開発ワークフロー（general）とエッジロボティクス（edge-robotics）の2プラグインを提供する。

## 構成

```
general/          汎用開発スキル・エージェント・ルール
edge-robotics/    エッジコンピューティング・ロボティクス・組込開発
install-rules.sh  ルール配布スクリプト（プラグインシステムでは配布不可のため）
```

## 設計原則

| 種類 | 性質 | 配布方法 |
|------|------|----------|
| **Skills** | 参照型（必要時に発動） | プラグインシステム |
| **Agents** | 自律タスク実行 | プラグインシステム |
| **Rules** | 非交渉的（常に従う） | install-rules.sh |

### Rules vs Skills

- **Rules**: HW/SW 非依存の普遍的ガイドライン。常にロードされる
- **Skills**: HW/SW 固有の知識。発動条件（When to Activate）が明確

### Agents の使い分け

- 実行が必要な作業（スキャン、書き込み、診断）→ Agent
- 知識参照だけ → Skill

## バージョン管理

各プラグインのバージョンは `<plugin>/.claude-plugin/plugin.json` の `version` フィールドで管理する。

| プラグイン | パス |
|-----------|------|
| ルート | `.claude-plugin/plugin.json` |
| general | `general/.claude-plugin/plugin.json` |
| edge-robotics | `edge-robotics/.claude-plugin/plugin.json` |

### 更新タイミング

skills / agents / rules の追加・削除・大幅変更時にパッチ (`0.0.1`) またはマイナー (`0.1.0`) を上げる。

- **パッチ**: 既存スキルの修正、typo 修正、リファレンス更新
- **マイナー**: スキル・エージェントの追加・削除、構造変更
- **メジャー**: 破壊的変更（ディレクトリ構造の大幅変更など）
