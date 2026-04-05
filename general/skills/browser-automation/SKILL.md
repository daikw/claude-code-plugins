---
name: browser-automation
description: "Playwright MCP を使ったブラウザ自動化をガイドする。E2E テスト、スクリーンショット撮影、認証済みサービスへのアクセス、環境セットアップを支援する。「ブラウザ操作」「E2E テスト」「ブラウザテスト」「認証」などのリクエストで使用。"
---

# Browser Automation with Playwright MCP

Playwright MCP を使ったブラウザ自動化ガイド。ローカル Web アプリのテスト、外部サービスへの認証済みアクセス、スクリーンショット撮影などに対応する。

Freedom Level: **中** — 基本パターンに従いつつ、対象に応じて柔軟に調整する。

## Playwright MCP 環境セットアップ

### 推奨設定（`~/.claude.json`）

```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": [
        "@playwright/mcp@latest",
        "--isolated",
        "--headless",
        "--storage-state",
        "/Users/<username>/.claude/playwright-storage-state.json"
      ]
    }
  }
}
```

**フラグの意味:**
- `--isolated` — 一時プロファイルで起動。複数セッションから同時利用可能
- `--headless` — ブラウザウィンドウを表示しない
- `--storage-state` — 認証済み Cookie/localStorage を注入

### 設定の優先順位

`~/.claude.json` の `mcpServers` > プラグインの `.mcp.json`。
プラグイン側を編集しても `~/.claude.json` に同名キーがあるとそちらが優先される。

### storage-state のエクスポート

認証済みブラウザセッションがある状態で:

```
mcp__playwright__browser_run_code:
  code: |
    async (page) => {
      const state = await page.context().storageState();
      return JSON.stringify(state);
    }
```

結果の JSON を `~/.claude/playwright-storage-state.json` に保存する。
Cookie の有効期限が切れたら同じ手順で再エクスポートする。

不要な Cookie（一時セッション、サードパーティトラッカー等）は除外してよい。
認証に必要な永続 Cookie だけ残す。

## Reconnaissance-Then-Action

**必ず「観察→操作→確認」の順で進める。** 盲目的にセレクタを推測しない。

```
1. navigate  — 対象ページを開く
2. snapshot  — アクセシビリティツリーを取得（DOM 構造の把握）
3. action    — click / fill_form / type 等（snapshot の ref を使う）
4. screenshot — 結果を視覚的に確認
```

## 基本ツールの使い分け

| ツール | 用途 |
|--------|------|
| `browser_navigate` | URL を開く |
| `browser_snapshot` | アクセシビリティツリー取得。操作対象の `ref` を特定 |
| `browser_click` | `ref` 指定でクリック |
| `browser_fill_form` | フォーム入力 |
| `browser_type` | キーボード入力 |
| `browser_take_screenshot` | スクリーンショット撮影（ヘッドレスでも動作） |
| `browser_evaluate` | 任意の JavaScript 実行 |
| `browser_run_code` | Playwright API を直接実行（高度な操作向け） |
| `browser_wait_for` | 要素出現やナビゲーション完了を待つ |

## リファレンス

- E2E テスト戦略・シナリオ設計: [references/e2e-testing.md](references/e2e-testing.md)
- 認証が必要なサービス: [references/authenticated-services.md](references/authenticated-services.md)
- 一般的なトラブルシューティング: [references/troubleshooting.md](references/troubleshooting.md)

## よくある問題

- **"Browser is already in use"** → `--isolated` が未設定、または別プロセスが同じプロファイルをロック中
- **認証が必要なサービスにアクセスできない** → storage-state の Cookie が期限切れ。再エクスポートする
- **要素が見つからない** → `browser_wait_for` で待機、または snapshot で実際の DOM を確認
- **ヘッドレスなのにウィンドウが出る** → `~/.claude.json` の設定が優先されているか確認。プラグイン `.mcp.json` の編集だけでは反映されない
