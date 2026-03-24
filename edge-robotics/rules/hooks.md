# Hooks Rules

Claude Code のフック機能の活用ルール。

## Hook Types

| フックタイプ | タイミング | 用途 |
|-------------|-----------|------|
| `PreToolUse` | ツール実行前 | バリデーション、パラメータ修正 |
| `PostToolUse` | ツール実行後 | 自動フォーマット、チェック |
| `Stop` | セッション終了時 | 最終検証 |

## Recommended Hooks for Embedded Development

### Pre-Tool Hooks

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Write",
        "command": "echo 'Reminder: Check memory constraints before writing large files'"
      },
      {
        "matcher": "Bash",
        "command": "echo 'Reminder: Verify target device before flashing'"
      }
    ]
  }
}
```

### Post-Tool Hooks

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit",
        "command": "clang-format -i $FILE 2>/dev/null || true"
      },
      {
        "matcher": "Write",
        "command": "if [[ $FILE == *.c || $FILE == *.h ]]; then clang-format -i $FILE; fi"
      }
    ]
  }
}
```

### Stop Hooks

```json
{
  "hooks": {
    "Stop": [
      {
        "command": "echo 'Session ended. Remember to test on actual hardware.'"
      }
    ]
  }
}
```

## Configuration Location

フック設定は `~/.claude/settings.json` に記述する。

```json
{
  "hooks": {
    "PreToolUse": [...],
    "PostToolUse": [...],
    "Stop": [...]
  }
}
```

## Best Practices

### 1. Auto-Accept の慎重な使用

```json
{
  "permissions": {
    "allow": [
      "Read",
      "Glob",
      "Grep"
    ]
  }
}
```

- 読み取り系ツールは自動許可してOK
- 書き込み系（Write, Edit, Bash）は慎重に
- 探索的な開発時は手動確認を推奨

### 2. スコープを限定

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit",
        "pathPattern": "src/**/*.c",
        "command": "clang-format -i $FILE"
      }
    ]
  }
}
```

### 3. 失敗しても続行

```bash
# ✅ エラーを無視して続行
clang-format -i $FILE 2>/dev/null || true

# ❌ エラーで停止
clang-format -i $FILE
```

## Embedded-Specific Hooks

### ビルドサイズチェック

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Bash",
        "command": "if [[ -f build/*.elf ]]; then arm-none-eabi-size build/*.elf; fi"
      }
    ]
  }
}
```

### 静的解析

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit",
        "pathPattern": "**/*.c",
        "command": "cppcheck --enable=warning,style $FILE 2>/dev/null || true"
      }
    ]
  }
}
```

### フラッシュ前確認

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "command": "if echo $COMMAND | grep -q 'flash\\|program\\|upload'; then echo '⚠️ About to flash firmware. Verify target device!'; fi"
      }
    ]
  }
}
```

## TodoWrite Integration

複雑な実装時は TodoWrite でタスクを追跡:

- 構造的な問題の早期発見
- ステップの漏れ防止
- 進捗の可視化

```
TodoWrite を使って:
1. タスクを分解
2. 依存関係を整理
3. 進捗を記録
```
