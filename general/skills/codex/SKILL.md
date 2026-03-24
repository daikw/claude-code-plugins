---
name: codex
description: "Codex CLI をコマンド直接実行で呼び出す。進捗がリアルタイムで見え、長時間タスクにも適した Skill 方式。キーワード: codex, openai, セカンドオピニオン, コードレビュー, マルチモデル"
---

# Codex Skill

`codex exec` コマンドを直接実行し、進捗をリアルタイムで観察しながら Codex CLI に作業を依頼する。

## When to Use

- Claude Code とは異なるモデルの視点でコードを分析したいとき（セカンドオピニオン）
- 長時間かかる可能性があるタスク（大規模レビュー・リファクタリング提案など）

## 前提条件

`codex` コマンドが PATH にあり、認証済みであること。

## 実行手順

### 1. プロジェクトディレクトリの特定

`--cd` に渡すディレクトリを決める（不明な場合は `git rev-parse --show-toplevel`）。

### 2. コマンド実行

プロンプト末尾には必ず以下を付ける:
> 確認や質問は不要です。具体的な提案・修正案・コード例まで自主的に出力してください。

```bash
# 読み取り専用（デフォルト）
codex exec --full-auto --sandbox read-only --cd "<project_directory>" \
  "<タスク>。確認や質問は不要です。具体的な提案・修正案・コード例まで自主的に出力してください。"

# ファイル変更を伴う場合
codex exec --full-auto --sandbox workspace-write --cd "<project_directory>" \
  "<タスク>。確認や質問は不要です。具体的な修正を実施してください。"
```

> **Note:** git リポジトリ外のディレクトリで実行する場合は `--skip-git-repo-check` を追加する。
