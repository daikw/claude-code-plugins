---
name: cross-agent
description: Codex と Claude-Code 間の協調作業を管理。長時間自律動作のための状態管理、引き継ぎ、レビューワークフローを提供。
tags:
  - multi-agent
  - collaboration
  - long-running
  - codex
  - claude-code
---

# Cross-Agent Collaboration Skill

Codex（Planner）と Claude-Code（Worker）の協調作業を支援するスキル。
ファイルベースの状態共有により、長時間の自律動作を実現する。

## When to Activate

- 複数のAIエージェント間で協調作業を行うとき
- 長時間にわたるプロジェクトを自律的に進めるとき
- Codex に設計・レビューを依頼し、Claude-Code で実装するとき
- セッションを跨いで作業を継続するとき

## Core Principles

参考: [Anthropic - Effective Harnesses for Long-Running Agents](https://www.anthropic.com/engineering/effective-harnesses-for-long-running-agents)

1. **1セッション1フィーチャー**: 過度な野心を避け、確実に完了できる単位で作業
2. **ファイルベースの状態共有**: JSON/MD で進捗を永続化し、セッション間の引き継ぎを確実に
3. **明確な役割分担**: Planner（設計・レビュー）と Worker（実装・テスト）を分離
4. **自己修復可能**: テスト失敗時はロールバック、ブロッカーは明示的に記録

## Workspace Structure

プロジェクトルートに `.agent-workspace/` を作成:

```
.agent-workspace/
├── project.json      # プロジェクト定義・フィーチャーリスト
├── progress.json     # 進捗状態・最終セッション情報
├── handoff.md        # エージェント間の引き継ぎメモ
└── reviews/          # レビュー履歴
    └── feat-001.md
```

## File Formats

### project.json - プロジェクト定義

```json
{
  "name": "project-name",
  "description": "プロジェクト概要",
  "created_at": "2026-01-23T10:00:00Z",
  "planner": "codex",
  "worker": "claude-code",
  "features": [
    {
      "id": "feat-001",
      "title": "機能タイトル",
      "description": "詳細な説明",
      "status": "pending",
      "priority": 1,
      "acceptance_criteria": [
        "条件1",
        "条件2"
      ],
      "tests_pass": false,
      "assigned_to": null,
      "review_notes": ""
    }
  ],
  "constraints": [
    "TypeScript 使用",
    "テストカバレッジ 80%以上"
  ]
}
```

**status の遷移**:
```
pending → in_progress → review → completed
                ↓           ↓
            blocked     revision_needed
```

### progress.json - 進捗状態

```json
{
  "current_feature": "feat-001",
  "current_phase": "implementation",
  "last_agent": "claude-code",
  "last_session": {
    "timestamp": "2026-01-23T12:30:00Z",
    "summary": "完了した作業の概要",
    "files_modified": ["src/auth.ts", "tests/auth.test.ts"],
    "next_steps": ["次にやるべきこと1", "次にやるべきこと2"],
    "commit_hash": "abc1234"
  },
  "blockers": [],
  "total_sessions": 5,
  "history": [
    {
      "session": 1,
      "agent": "codex",
      "action": "initialized project, created feature list"
    }
  ]
}
```

### handoff.md - 引き継ぎメモ

```markdown
# Handoff: [From Agent] → [To Agent]

## Session Summary
- Date: YYYY-MM-DD HH:MM
- Agent: claude-code / codex
- Feature: feat-001

## Completed Work
- [x] 完了した作業1
- [x] 完了した作業2

## Next Actions (for [To Agent])
- [ ] 次のアクション1
- [ ] 次のアクション2

## Concerns / Blockers
- 懸念事項があれば記載

## Key Files
- `src/auth.ts:15-45` - 認証ロジック
- `tests/auth.test.ts` - テストケース

## Notes for Reviewer (if status = review)
- レビューポイント
```

## Workflows

### Phase 1: Initialization (Codex)

Codex がプロジェクトを初期化する:

```bash
# 1. ワークスペース作成
mkdir -p .agent-workspace/reviews

# 2. project.json 作成（要件分析 → フィーチャー分解）
# 3. progress.json 初期化
# 4. handoff.md に最初の実装指示を記載
```

初期化時の Codex の責務:
- 要件を分析し、小さなフィーチャーに分解
- 各フィーチャーの acceptance_criteria を明確に定義
- 実装順序（priority）を決定
- 技術的制約（constraints）を明記

### Phase 2: Implementation Loop (Claude-Code)

Claude-Code の各セッション:

**セッション開始時（必須チェックリスト）**:
```bash
# 1. ワークスペース確認
ls -la .agent-workspace/

# 2. 進捗確認
cat .agent-workspace/progress.json | jq '.current_feature, .current_phase'

# 3. 引き継ぎ確認
cat .agent-workspace/handoff.md

# 4. Git状態確認
git log --oneline -5
git status
```

**作業中のルール**:
1. `current_feature` のみに集中（他のフィーチャーに手を出さない）
2. TDD で進める（テストを先に書く）
3. 変更は小さくコミット
4. project.json のフィーチャー定義は変更しない（status のみ更新可）

**セッション終了時**:
```bash
# 1. テストが通ることを確認
npm test  # or pytest, cargo test, etc.

# 2. 変更をコミット
git add -A && git commit -m "feat(feat-001): description"

# 3. progress.json 更新
# 4. handoff.md 更新（次のエージェント向け）
```

### Phase 3: Review (Codex)

Codex がレビューを行う:

1. `handoff.md` で実装完了を確認
2. コードレビュー実施
3. `reviews/feat-XXX.md` にレビュー結果を保存
4. 判定:
   - **OK** → `project.json` の status を `completed` に
   - **NG** → status を `revision_needed` に、`handoff.md` に修正指示

## Session Start Protocol (Claude-Code)

各セッション開始時に実行:

```markdown
## Pre-Flight Checklist

- [ ] `.agent-workspace/` が存在する
- [ ] `progress.json` を読み込んだ
- [ ] `handoff.md` の指示を確認した
- [ ] `git log --oneline -5` で最近のコミットを確認した
- [ ] `current_feature` の `acceptance_criteria` を把握した
- [ ] `blockers` が空であることを確認した（またはブロッカー解消が優先）
```

## Error Recovery

| 状況 | 対応 |
|------|------|
| テストが壊れた | `git stash` または `git checkout .` で最後の正常状態に戻る |
| 実装が複雑すぎる | handoff.md に記載し、Codex に設計見直しを依頼。status を `blocked` に |
| ブロッカー発生 | progress.json の blockers に追記、handoff.md に詳細記載、セッション終了 |
| コンフリクト | 自動解決を試みず、handoff.md に記載して Codex に判断を委ねる |
| 途中でセッション切れ | 再開時に progress.json と git status から状態を復元 |

## Commands

### 初期化（Codex用）

```bash
# プロジェクト初期化スクリプト
cat << 'EOF' > .agent-workspace/init.sh
#!/bin/bash
mkdir -p .agent-workspace/reviews

# project.json テンプレート
cat << 'JSON' > .agent-workspace/project.json
{
  "name": "",
  "description": "",
  "created_at": "$(date -Iseconds)",
  "planner": "codex",
  "worker": "claude-code",
  "features": [],
  "constraints": []
}
JSON

# progress.json 初期化
cat << 'JSON' > .agent-workspace/progress.json
{
  "current_feature": null,
  "current_phase": "planning",
  "last_agent": "codex",
  "last_session": null,
  "blockers": [],
  "total_sessions": 0,
  "history": []
}
JSON

# handoff.md 初期化
cat << 'MD' > .agent-workspace/handoff.md
# Handoff: Codex → Claude-Code

## Initial Setup
Project initialized. Waiting for feature definition.
MD

echo "Workspace initialized at .agent-workspace/"
EOF
chmod +x .agent-workspace/init.sh
```

### 状態確認（共通）

```bash
# 現在の状態を一覧表示
jq '{
  feature: .current_feature,
  phase: .current_phase,
  last_agent: .last_agent,
  blockers: .blockers
}' .agent-workspace/progress.json
```

## Tips for Long-Running Autonomy

### Codex（Planner）向け

1. **フィーチャーは小さく分解**: 1セッションで完了できるサイズに
2. **acceptance_criteria は具体的に**: 曖昧さを排除
3. **依存関係を明示**: フィーチャー間の順序を priority で表現
4. **レビューは建設的に**: 修正指示は具体的なコード例を含める

### Claude-Code（Worker）向け

1. **指示に忠実に**: project.json の定義を勝手に変更しない
2. **テストを先に**: TDD で品質を担保
3. **こまめにコミット**: ロールバックしやすい状態を維持
4. **困ったら引き継ぎ**: 無理に解決せず handoff.md に記載

## Example Usage

### Codex: プロジェクト開始

```markdown
User: "TODOアプリを作りたい"

Codex:
1. .agent-workspace/ を初期化
2. project.json にフィーチャーを定義:
   - feat-001: TODOの追加機能
   - feat-002: TODOの一覧表示
   - feat-003: TODOの完了/削除
3. handoff.md に最初の実装指示を記載
4. Claude-Code に引き継ぎ
```

### Claude-Code: 実装セッション

```markdown
1. 状態確認（Pre-Flight Checklist）
2. feat-001 の実装開始
3. テスト作成 → 実装 → リファクタ
4. コミット
5. progress.json 更新
6. handoff.md に完了報告
7. status を "review" に変更
```

### Codex: レビュー

```markdown
1. handoff.md でレビュー依頼を確認
2. コードレビュー実施
3. reviews/feat-001.md に結果を記録
4. OK → status を "completed" に
5. 次のフィーチャーの実装指示を handoff.md に記載
```

## Related Resources

- [Anthropic: Effective Harnesses for Long-Running Agents](https://www.anthropic.com/engineering/effective-harnesses-for-long-running-agents)
- [Cursor: Scaling Agents](https://cursor.com/blog/scaling-agents)
- [everything-claude-code](https://github.com/affaan-m/everything-claude-code)

## Safety Notes

- `.agent-workspace/` は `.gitignore` に追加しないこと（履歴を残す）
- 機密情報は handoff.md に書かない
- project.json のフィーチャー定義変更は Codex のみが行う
- blockers が発生したら無理に進めず、必ず記録して引き継ぐ
