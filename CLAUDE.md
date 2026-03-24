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
