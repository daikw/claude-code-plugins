---
name: uploading-images-via-browser
description: "Uploads images and screenshots to web forms (GitHub Issues/PRs, GitLab MRs, etc.) via browser automation using ClipboardEvent paste injection. Use when the user asks to attach screenshots to issues, paste images into PR comments, or upload images to any web textarea that supports paste-to-upload."
---

# uploading-images-via-browser

Playwright MCP または claude-in-chrome を使って、Web フォームの textarea に画像を paste event で注入するスキル。GitHub/GitLab 等の paste-to-upload 対応サイトで動作する。

Freedom Level: **中** — 推奨パターン（Playwright → paste event）を使いつつ、サイトごとのセレクタは文脈に応じて調整する。

## 手段の優先順位

1. **Playwright MCP**（推奨）- 独自ブラウザインスタンス、ファイルサイズ制限なし
2. **claude-in-chrome**（フォールバック）- ユーザーの Chrome を使用、~50KB のサイズ制限あり

## 前提条件

- Playwright MCP が接続済みであること
- GitHub にログイン済みのブラウザセッションがあること（Playwright は初回ログインが必要）
- アップロードする画像ファイルが `/Users/daikiwatanabe/tmp/tmp` 配下または `.playwright-mcp/` 配下にあること

---

## 手法1: Playwright MCP（推奨）

### ステップ概要

1. GitHub ページへナビゲート
2. 画像を base64 化してアップロードスクリプトを生成
3. `browser_run_code` でスクリプト実行（ClipboardEvent paste）
4. スクリーンショットで結果確認

### 詳細手順

#### Step 1: GitHub ページを開く

```
mcp__playwright__browser_navigate → url: "https://github.com/{owner}/{repo}/issues/{number}"
```

#### Step 2: 画像を base64 化してスクリプトファイルを生成

Bash で以下を実行する。`browser_run_code` 内では Node.js の `require`/`import` が使えないため、必ず Bash 側で base64 を埋め込む。

```bash
B64=$(base64 -i /path/to/image.png | tr -d '\n')
cat > /Users/daikiwatanabe/tmp/tmp/upload-script.js << JSEOF
async (page) => {
  const b64 = '${B64}';
  const result = await page.evaluate(async (b64) => {
    const binaryStr = atob(b64);
    const bytes = new Uint8Array(binaryStr.length);
    for (let i = 0; i < binaryStr.length; i++) bytes[i] = binaryStr.charCodeAt(i);
    const blob = new Blob([bytes], { type: 'image/png' });
    const file = new File([blob], 'screenshot.png', { type: 'image/png' });
    const ta = document.querySelector('textarea[placeholder*="Type your description"]')
            || document.querySelector('textarea[placeholder*="Leave a comment"]')
            || document.querySelector('.js-comment-field');
    if (!ta) return { error: 'textarea not found', selectors: document.querySelectorAll('textarea').length };
    ta.focus();
    const dt = new DataTransfer();
    dt.items.add(file);
    const evt = new ClipboardEvent('paste', { bubbles: true, cancelable: true, clipboardData: dt });
    const dispatched = ta.dispatchEvent(evt);
    await new Promise(r => setTimeout(r, 3000));
    return { success: true, dispatched, fileSize: file.size, textareaValue: ta.value.substring(0, 300) };
  }, b64);
  return JSON.stringify(result, null, 2);
}
JSEOF
```

#### Step 3: スクリプトを実行

```
mcp__playwright__browser_run_code → code: <upload-script.js の内容を指定>
```

返り値に `success: true` と `textareaValue` に `![Uploading screenshot.png...]` のようなマークダウンが含まれていれば成功。GitHub が自動で `user-attachments/assets/...` の URL に差し替える。

#### Step 4: 結果確認

```
mcp__playwright__browser_take_screenshot
```

textarea に画像のマークダウンリンクが挿入されていることを目視確認する。

#### Step 5: コメント投稿（必要に応じて）

テキストの追加やボタンクリックで投稿する。

### Playwright のログインについて

Playwright は独自のブラウザインスタンスを使うため、初回は GitHub にログインする必要がある。

1. `browser_navigate` で `https://github.com/login` を開く
2. `browser_fill_form` でユーザー名・パスワードを入力（ユーザーに聞く）
3. 2FA が必要な場合はユーザーに操作を依頼する
4. ログイン後、ブラウザを閉じなければセッションは維持される

---

## 手法2: claude-in-chrome（フォールバック）

Playwright MCP が利用できない場合のフォールバック。ユーザーの Chrome（ログイン済み）で操作する。

### 制約事項

- **サイズ制限**: JS の text パラメータに base64 を埋め込むため、元画像 ~50KB（base64 で ~67KB）が上限
- **大きい画像**: JPEG 圧縮（quality=20-30）+ リサイズで対応する
- **`upload_image` ツールは使用不可**: MCP 経由では "Unable to access message history to retrieve image" エラーが発生するバグあり（2026-04 時点、未修正）

### 手順

#### Step 1: タブコンテキスト取得 & GitHub ページを開く

```
mcp__claude-in-chrome__tabs_context_mcp
mcp__claude-in-chrome__tabs_create_mcp → url: "https://github.com/{owner}/{repo}/issues/{number}"
```

#### Step 2: 画像を base64 化

Bash で画像を base64 に変換する。サイズが大きい場合は圧縮する。

```bash
# サイズ確認
wc -c < /path/to/image.png

# 大きい場合: ImageMagick でリサイズ + JPEG 圧縮
convert /path/to/image.png -resize 800x -quality 25 /path/to/image-small.jpg
B64=$(base64 -i /path/to/image-small.jpg | tr -d '\n')

# 小さい場合: そのまま
B64=$(base64 -i /path/to/image.png | tr -d '\n')
```

#### Step 3: JavaScript で paste event を dispatch

```
mcp__claude-in-chrome__javascript_tool → text: <以下のスクリプト（B64 を埋め込み済み）>
```

```javascript
(async () => {
  const b64 = '<BASE64_STRING_HERE>';
  const mimeType = 'image/png';  // or 'image/jpeg'
  const fileName = 'screenshot.png';  // or 'screenshot.jpg'
  const binaryStr = atob(b64);
  const bytes = new Uint8Array(binaryStr.length);
  for (let i = 0; i < binaryStr.length; i++) bytes[i] = binaryStr.charCodeAt(i);
  const blob = new Blob([bytes], { type: mimeType });
  const file = new File([blob], fileName, { type: mimeType });
  const ta = document.querySelector('textarea[placeholder*="Type your description"]')
          || document.querySelector('textarea[placeholder*="Leave a comment"]')
          || document.querySelector('.js-comment-field');
  if (!ta) return JSON.stringify({ error: 'textarea not found' });
  ta.focus();
  const dt = new DataTransfer();
  dt.items.add(file);
  const evt = new ClipboardEvent('paste', { bubbles: true, cancelable: true, clipboardData: dt });
  ta.dispatchEvent(evt);
  await new Promise(r => setTimeout(r, 3000));
  return JSON.stringify({ success: true, textareaValue: ta.value.substring(0, 300) });
})()
```

#### Step 4: 結果確認

```
mcp__claude-in-chrome__read_page
```

---

## paste ターゲット

サービスごとのセレクタ、フォールバック戦略、contenteditable 向けの paste パターンは [references/paste-targets.md](references/paste-targets.md) を参照。

対応サービス: GitHub, GitLab, Notion, Slack (Web), Google Docs, CKEditor / TipTap / ProseMirror 系。

---

## トラブルシューティング

### textarea not found

- ページのロードが完了していない可能性がある。`browser_wait_for` で textarea の出現を待つ
- GitHub の UI 変更でセレクタが変わった可能性がある。`browser_snapshot` で DOM を確認する

### paste event は成功するが画像 URL に差し替わらない

- GitHub の JavaScript が paste event を拾えていない。textarea に focus を当ててからもう一度試す
- `await new Promise(r => setTimeout(r, 3000))` の待機時間が短い可能性がある。5000ms に延長する

### Playwright でログインセッションが切れる

- `browser_navigate` で GitHub にアクセスしてログイン状態を確認する
- 再ログインが必要な場合はユーザーに通知する

### claude-in-chrome でサイズ制限に引っかかる

- ImageMagick で圧縮: `convert input.png -resize 600x -quality 20 output.jpg`
- それでも大きい場合は Playwright MCP に切り替える

---

## 既知の制限事項

- **upload_image MCP ツール**: Chrome 拡張の MCP コードパスに `messages` プロパティが渡されないバグにより使用不可。詳細は `~/tmp/tmp/chrome-upload-image-investigation.md` を参照
- **Mixed Content**: localhost の HTTP サーバから GitHub (HTTPS) への fetch はブラウザにブロックされる
- **GitHub 新 UI**: `<input type="file">` が存在しないため、`browser_file_upload` は使えない。paste event 方式が唯一の手段
