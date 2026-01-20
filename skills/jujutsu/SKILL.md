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

## When to Activate

- jj でバージョン管理作業をするとき
- Git リポジトリで jj を使い始めるとき
- コミット履歴の編集・整理
- コンフリクト解決

## Core Principles

- **すべての操作は取り消し可能**: `jj op log` で操作履歴を確認、`jj op restore <op-id>` で任意の時点に戻れる
- **ワーキングコピーは自動的にコミット**: 明示的な `add` は不要、すべての変更は自動的にワーキングコピーコミットに追記される
- **コミットではなく「変更 (change)」**: 各変更には不変の change ID が付与される
- **コンフリクトは操作を止めない**: `jj rebase` はコンフリクトがあっても成功し、後で好きなタイミングで解決できる
- **`jj new` を頻繁に使う**: 迷ったら `jj new` で新しい変更を作成。空の変更は自動的に破棄される

## 必須コマンド

### 状態確認

```bash
jj status          # 現在の状態を確認
jj diff            # 変更内容を表示
jj log             # 履歴を表示（短縮形）
jj log -r 'all()'  # 全履歴を表示
jj evolog          # 変更の進化履歴を表示（旧 jj obslog）
```

### 変更の作成と編集

```bash
jj new                      # 新しい空の変更を作成
jj new -m 'description'     # 説明付きで新しい変更を作成
jj commit -m 'description'  # 現在の変更を確定して新しい変更を開始
jj describe -m 'message'    # 現在の変更の説明を設定
jj edit <change-id>         # 指定した変更に移動して編集
jj diffedit                 # 変更内容を直接エディタで編集
```

### 変更の整理

```bash
jj squash                   # 現在の変更を親に統合
jj squash --into <change>   # 指定した変更に統合
jj split                    # 現在の変更を分割（インタラクティブ）
jj squash -i --from X --into Y  # X の一部を Y に移動
jj parallelize              # 直列の変更を並列化
```

### Bookmark 操作（旧 branch）

```bash
jj bookmark create <name>   # 現在の変更に bookmark を作成
jj bookmark set <name>      # bookmark を現在の変更に移動
jj bookmark list            # bookmark 一覧
jj bookmark delete <name>   # bookmark を削除
```

> Note: `jj branch` は非推奨。Jujutsu の「ブランチ」は Git のブランチと異なる動作をするため、Mercurial の bookmark に倣い「bookmark」に改名された。

### Git 連携

```bash
jj git init --colocate      # 既存 Git リポジトリを jj で管理開始
jj git fetch                # リモートから取得
jj git push                 # プッシュ
jj git push --bookmark <name>  # 特定 bookmark をプッシュ
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
| `git branch` | `jj bookmark` | 名称と動作が異なる |
| staging area | なし | 不要（`jj split` で代替） |

## ワークフロー例

### 新機能開発

```bash
jj new -m 'feat: add X'          # 説明付きで新しい変更を開始
# ... 作業 ...
jj new -m 'feat: add Y'          # 次の変更へ
# ... 作業 ...
jj bookmark create feature-xy    # bookmark を作成
jj git push --bookmark feature-xy  # プッシュ
```

### 過去の変更を修正

```bash
jj log                           # 修正したい変更を探す
jj edit <change-id>              # その変更に移動
# ... 修正 ...
jj new                           # 元の作業に戻る（または jj edit @-）
```

### 変更の分割・統合

```bash
# 大きな変更を分割
jj split                         # インタラクティブに分割

# 複数の変更を1つに統合
jj squash                        # 現在の変更を親に統合

# 特定の変更同士を統合
jj squash -i --from X --into Y   # X の一部を Y に移動
```

### コンフリクト解決

```bash
jj rebase -d main                # main の上にリベース（コンフリクトがあっても成功）
jj status                        # コンフリクトを確認
# ファイルを編集してコンフリクトマーカーを解消
# 自動的に変更が記録される（特別なコマンド不要）
```

### WIP の保存

```bash
# Git の stash の代わりに
jj describe -m 'wip: 作業途中のX'  # 説明を付けておく
jj new                            # 新しい変更に移動
# 後で戻るとき
jj log                            # WIP を探す
jj edit <wip-change-id>           # 戻る
```

## 便利な revset

```bash
jj log -r 'mine()'               # 自分の変更のみ
jj log -r 'ancestors(@, 5)'      # 直近5つの祖先
jj log -r 'bookmarks()'          # bookmark が付いた変更
jj log -r 'subject("fix")'       # コミットメッセージに "fix" を含む
```

## 注意事項

- Git コマンドでリポジトリに書き込まないこと（読み取りは可）
- コミット時は `jj commit -m 'topic: description'` 形式を推奨
- WIP 変更には必ず `jj describe -m 'wip: ...'` で説明を付ける（後で見つけやすい）
- 複数の Claude インスタンスで同時作業する場合は `jj workspace` を検討
- 他の人がベースにしている変更は rewrite しない
