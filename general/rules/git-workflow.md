# Git Workflow

## ⚠️ 必須: ブランチ & PR フロー

**main への直接プッシュは禁止。必ず以下のフローを守ること:**

1. **Feature Branch 作成** - `git checkout -b <type>/<description>`
   - 例: `feat/codex-notify-support`, `fix/turn-id-type`
2. **実装 & コミット** - ブランチ上で作業
3. **PR 作成** - `gh pr create` でプルリクエスト作成
4. **CI & レビュー待ち** - ユーザーの承認を待つ
5. **マージ** - 承認後に `gh pr merge`

## Commit Message Format

```
<type>: <description>

<optional body>
```

Types: feat, fix, refactor, docs, test, chore, perf, ci

Note: Attribution disabled globally via ~/.claude/settings.json.

## Pull Request Workflow

When creating PRs:
1. Analyze full commit history (not just latest commit)
2. Use `git diff [base-branch]...HEAD` to see all changes
3. Draft comprehensive PR summary
4. Include test plan with TODOs
5. Push with `-u` flag if new branch

## Feature Implementation Workflow

1. **Plan First**
   - Use **planner** agent to create implementation plan
   - Identify dependencies and risks
   - **共通関数を修正する場合**: `grep` で全呼び出し箇所を洗い出し、修正方針を検討
     - A: 呼び出し元を個別に修正
     - B: 共通関数自体を修正（同じロジックが複数箇所に必要な場合はこちらを優先）
   - Break down into phases

2. **TDD Approach**
   - Use **tdd-guide** agent
   - Write tests first (RED)
   - Implement to pass tests (GREEN)
   - Refactor (IMPROVE)
   - Verify 80%+ coverage

3. **Code Review**
   - Use **code-reviewer** agent immediately after writing code
   - Address CRITICAL and HIGH issues
   - Fix MEDIUM issues when possible

4. **Commit & Push**
   - Detailed commit messages
   - Follow conventional commits format
