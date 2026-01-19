---
name: jujutsu
description: Jujutsu (jj) version control system workflow guide. Use this skill when working with jj instead of git, or when migrating from git to jj.
tags:
  - vcs
  - version-control
  - jujutsu
  - jj
---

# Jujutsu (jj) Workflow Guide

Jujutsu は Git 互換の次世代バージョン管理システム。Git リポジトリを直接操作でき、より安全で柔軟なワークフローを提供する。

## 基本原則

- **すべての操作は取り消し可能**: `jj op log` で操作履歴を確認、`jj op restore <op-id>` で任意の時点に戻れる
- **ワーキングコピーは自動的に変更として追跡**: 明示的な `add` は不要
- **コミットではなく「変更 (change)」**: 各変更には不変の change ID が付与される

## 必須コマンド

### 状態確認

```bash
jj status          # 現在の状態を確認
jj diff            # 変更内容を表示
jj log             # 履歴を表示（短縮形）
jj log -r 'all()'  # 全履歴を表示
```

### 変更の作成

```bash
jj new                      # 新しい空の変更を作成
jj commit -m 'description'  # 現在の変更を確定して新しい変更を開始
jj describe -m 'message'    # 現在の変更の説明を設定
```

### ブランチ操作

```bash
jj branch create <name>     # 現在の変更にブランチを作成
jj branch set <name>        # ブランチを現在の変更に移動
jj branch list              # ブランチ一覧
```

### Git 連携

```bash
jj git fetch                # リモートから取得
jj git push                 # プッシュ
jj git push --branch <name> # 特定ブランチをプッシュ
```

### 取り消し・復元

```bash
jj op log                   # 操作履歴を表示
jj op restore <op-id>       # 指定した操作時点に戻す
jj undo                     # 直前の操作を取り消し
```

## Git との主な違い

| Git | Jujutsu | 備考 |
|-----|---------|------|
| `git add` | 不要 | 自動追跡 |
| `git commit` | `jj commit` または `jj new` | |
| `git checkout` | `jj edit <change-id>` | |
| `git stash` | 不要 | 別の変更に移動するだけ |
| `git rebase -i` | `jj squash`, `jj split` 等 | 個別コマンドで操作 |

## ワークフロー例

### 新機能開発

```bash
jj new                           # 新しい変更を開始
# ... 作業 ...
jj describe -m 'feat: add X'     # 説明を設定
jj new                           # 次の変更へ
# ... 作業 ...
jj describe -m 'feat: add Y'
jj git push --branch feature-xy  # プッシュ
```

### 過去の変更を修正

```bash
jj log                           # 修正したい変更を探す
jj edit <change-id>              # その変更に移動
# ... 修正 ...
jj new                           # 元の作業に戻る（または jj edit @-）
```

## 注意事項

- Git コマンドでリポジトリに書き込まないこと（読み取りは可）
- コミット時は `jj commit -m 'topic: description'` 形式を推奨
- 複数の Claude インスタンスで同時作業する場合は `jj workspace` を検討
