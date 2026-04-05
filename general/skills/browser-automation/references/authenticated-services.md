# 認証が必要なサービスへのアクセス

## 仕組み

`--storage-state` で Cookie と localStorage を注入する。
サービスごとに認証方式が異なるため、必要な Cookie も異なる。

## 認証の難易度（目安）

| タイプ | 例 | storage-state との相性 |
|--------|-----|----------------------|
| セッション Cookie 方式 | GitHub, GitLab | 良好。Cookie エクスポートで安定動作 |
| 複数トークン + リフレッシュ | Google, Microsoft | 不安定。トークンが頻繁に失効する |
| IP / デバイス紐付き | 一部の社内ツール | 環境依存。ヘッドレスだと「新しいデバイス」扱いになりうる |

storage-state で安定しないサービスは `--user-data-dir` で永続プロファイルを使う方が確実。
ただし同時起動できないトレードオフがある。

## 初回認証セットアップ

ヘッドレスモードではログインフォームの操作やOAuth リダイレクト、2FA の入力ができない。
**初回認証は `npx playwright open` で GUI ブラウザを別途起動して行う。Claude Code のセッション再起動は不要。**

### 手順

1. **ユーザーにターミナルで以下を実行してもらう**

```bash
! npx playwright open --save-storage=/home/daikw/.claude/playwright-storage-state.json https://example.com
```

   - `! ` プレフィックスで Claude Code セッション内から直接実行可能
   - `https://example.com` は認証先の URL に置き換える
   - GUI ブラウザが開く

2. **ユーザーが手動でログイン操作**（ID/パスワード入力、OAuth、2FA 等）

3. **ブラウザを閉じる** — 閉じた時点で `--save-storage` に指定したパスへ Cookie/localStorage が自動保存される

4. **保存された JSON の確認（任意）**
   - 不要な Cookie（Analytics、サードパーティトラッカー等）があれば手動で除外してよい
   - 認証に必要な永続 Cookie だけ残す

以降、ヘッドレスの Playwright MCP がその storage-state を自動的に読み込む。

### 複数サービスの認証

サービスごとに `npx playwright open` を繰り返す場合、`--save-storage` は毎回上書きされる。
複数サービスの Cookie を維持するには:

- **方法1**: 1回のセッションで複数サービスにログインしてからブラウザを閉じる
- **方法2**: サービスごとにエクスポートした JSON の `cookies` 配列を手動マージする（後述）

### 既存プロファイルがある場合

過去に `--user-data-dir` で認証済みプロファイルがあるなら:

```bash
! npx playwright open --user-data-dir=<path> --save-storage=/home/daikw/.claude/playwright-storage-state.json https://example.com
```

ログイン済みの状態でブラウザが開くので、そのまま閉じれば storage-state がエクスポートされる。

## 複数サービスの認証を管理する

storage-state は 1 ファイルに複数サービスの Cookie を含められる。
エクスポート時に全サービスの Cookie がまとめて出力される。

**マージ手順:**
サービス A でエクスポートした JSON と、サービス B の JSON の
`cookies` 配列を手動でマージする。`origins`（localStorage）も同様。

```json
{
  "cookies": [
    { "name": "user_session", "domain": "github.com", "..." : "..." },
    { "name": "session_token", "domain": "example.com", "..." : "..." }
  ],
  "origins": []
}
```

## トラブルシューティング

### ログイン状態が維持されない

1. Cookie の `expires` を確認 — 期限切れなら再エクスポート
2. `--isolated` なしで同じ URL にアクセスしてログインできるか確認
3. サービスがブラウザフィンガープリントを見ている場合、
   `--isolated` のクリーンプロファイルが「新しいデバイス」と判定される可能性あり
   → `--user-agent` で固定のユーザーエージェントを設定する

### 2FA / MFA を求められる

1. `npx playwright open --save-storage=~/.claude/playwright-storage-state.json <URL>` で GUI ブラウザを開く
2. 手動で 2FA を通す
3. 「このデバイスを信頼する」にチェックして Cookie に記録させる
4. ブラウザを閉じて storage-state を再保存

### CAPTCHA が表示される

- ヘッドレスブラウザは bot 検出されやすい
- `--no-sandbox` は逆効果（フィンガープリントが異常になる）
- 対策: `--user-agent` で通常のブラウザと同じ UA を設定する
- それでもダメな場合は `npx playwright open` で GUI から手動で通す
