---
name: xpoint-server-apply
description: XPoint新規サーバ構築申請フォームの記入支援。現在のリポジトリ（CDK/Terraform/CloudFormation等）のIaCコードを読み取り、サーバ構成・セキュリティ・監視・バックアップ等の情報を自動抽出してフォーム項目を埋める。コードから判別できない項目（管理責任者、コストセンター、導入日等）は対話的にユーザーへ確認する。「サーバ申請」「XPoint申請」「新規サーバ申請を書いて」「サーバ構築申請のフォームを埋めて」などのリクエストで使用。
---

# XPoint 新規サーバ構築申請フォーム記入支援

IaCリポジトリのコードを解析し、XPoint新規サーバ構築申請フォームの各項目への回答案を生成する。

## ワークフロー

### Phase 0: PR・Issue情報の収集

リポジトリのIaCコードに加え、GitHub上のPR・Issueからも申請に有用な情報を収集する。
特に作成中（Draft/Open）のPRには、まだmainにマージされていない最新の設計判断や構成変更が含まれることが多い。

#### 0.1 関連PRの探索

```bash
# Open/Draft PRを一覧取得
gh pr list --state open --json number,title,headRefName,isDraft,labels

# 現在のブランチにPRがあるか確認
gh pr view --json number,title,state,body,files
```

以下の条件でPRを関連PRとして判定する:
- 現在のブランチに紐づくPR
- タイトル・ラベルに申請対象サーバの名前やコンポーネント名を含むPR
- `feat/`, `infra/`, `chore/` プレフィックスを持つインフラ系PR

#### 0.2 PR本文からの情報抽出

```bash
gh pr view <number> --json body,files,comments,reviews
```

PR本文から以下を抽出する:
- **概要セクション**: システムの説明、変更目的 → 「説明」「使用目的」の材料
- **構成図（Mermaid等）**: PR本文にインライン記載された構成図 → Phase 4の構成図に活用
- **変更ファイル一覧**: どのStack/リソースが変更されたか → 申請対象の特定
- **デプロイ手順**: PR本文に記載されたデプロイ手順 → Phase 4の手順書に活用

#### 0.3 PRコメント・レビューからの情報抽出

```bash
# PRコメント（一般コメント）
gh pr view <number> --json comments --jq '.comments[] | {author: .author.login, body}'

# レビューコメント（コード行コメント）
gh api repos/{owner}/{repo}/pulls/<number>/comments --jq '.[] | {path, body, user: .user.login}'
```

コメントから以下を抽出する:
- **デプロイ検証結果**: `cdk diff` 出力、動作確認レポート → 「説明」の補足、手順書の検証セクション
- **セキュリティレビュー指摘**: SG/暗号化/IAMに関する指摘 → 「アクセス制限」「暗号化」の正確性向上
- **設計判断の議論**: アーキテクチャ選定理由 → 「説明」の補足

#### 0.4 関連Issueの探索

```bash
# PRにリンクされたIssueを取得
gh pr view <number> --json body --jq '.body' | grep -oE '#[0-9]+'

# Issue本文を取得
gh issue view <number> --json title,body,labels
```

Issueから以下を抽出する:
- 要件・目的の記述 → 「説明」「使用目的」
- ラベル（environment, security等）→ 「利用環境」の判断材料

### Phase 1: リポジトリ解析

Explore agentまたはGrep/Glob/Readツールで以下を収集する。
**Phase 0で特定したPRの変更ファイルがある場合、そのブランチのコードも解析対象に含める。**

**ネットワーク・リージョン情報**:
- リージョン/AZ設定 → 「所在・保管場所」
- VPC構成、サブネット配置

**コンピュート情報**:
- EC2インスタンスタイプ、ECSクラスタ → 「説明」の材料
- AMI、OS情報 → 「セキュリティパッチ適用」の判断材料

**セキュリティ情報**:
- Security Group設定 → 「アクセス元」「アクセス制限」
- WAF設定の有無 → 「アクセス制限」
- EBS/S3暗号化設定、TLS設定 → 「機器情報の暗号化」
- IMDSv2設定

**監視・ログ情報**:
- CloudWatch Alarms → 「監視」
- VPC Flow Logs → 「アクセスログ」「セキュリティログ」
- CloudWatch Logs → 「システムログ」「アプリケーションログ」
- ALB/NLBアクセスログ → 「アクセスログ」
- Container Insights → 「アプリケーションログ」
- DLM/IaC変更管理 → 「変更管理ログ」

**バックアップ情報**:
- DLMポリシー（スケジュール、保持世代数） → 「バックアップ」
- S3バージョニング、ライフサイクル

### Phase 2: 対話的確認

コードから判別できない以下の項目をユーザーに確認する。AskUserQuestionツールを使い、一度に最大4問ずつ聞く。

**必ず確認が必要な項目**:
- 管理責任者（メールアドレス）
- 利用者（従業員/特定部署/外部含む等）
- 個人情報の有無（あり/なし）
- 保管期限
- 利用環境（PRD/STG/DEV/TEST/社内利用）
- 使用目的
- コストセンター（部署名/サービス名）
- 導入作業予定日
- 導入作業者（メールアドレス）
- 導入確認者（メールアドレス、いない場合は「なし」）
- セキュリティパッチの適用予定（日次/週次/月次/半年毎/未実施）
- セキュリティパッチの適用責任者（メールアドレス）

**確認の優先順位**:
1. まず利用環境・使用目的・説明を確認（フォーム全体の方向性を決める）
2. 次に責任者・作業者情報を確認
3. 最後に保管期限・コストセンター等の管理情報を確認

### Phase 3: フォーム回答案の生成

全情報が揃ったら、フォームテンプレート（references/form-template.md）の形式に沿って回答案を出力する。

**出力形式**:
```markdown
### 説明
* [解析結果 + ユーザー回答を統合した記述]

### 所在・保管場所
* [リージョン/AZ]

...（全項目）
```

**注意事項**:
- 「説明」はシステムの概要がわかるよう具体的に記述する（例：「GitLabおよびCI/CDランナーをホストする社内開発基盤サーバ」）
- 選択肢がある項目は該当するものを選択形式で記載
- ログ項目は「あり」「なし」で統一
- 「アクセス制限」はSG/WAF/NACLの設定を具体的に記述
- 「暗号化」は転送時(TLS)と保存時(EBS/S3暗号化)の両方を確認し、適切な選択肢を選ぶ
- 「バックアップ」はDLMのスケジュールと保持数を「毎日○時に1回バックアップしN世代分保存」形式で記述
- 「監視」は死活監視(StatusCheck Alarms)とリソース監視(CPU/Memory Alarms)の有無を記載

### Phase 4: 添付資料の作成

フォーム末尾で「サーバ構成図」と「サーバ構築手順書」のPDF添付が必須。以下の手順で作成する。

#### 4.1 既存ドキュメントの探索

リポジトリ内とPRの両方から既存ドキュメントを検索する。

**リポジトリ内の検索**:
```
Glob: docs/**/*.{md,pdf,mmd,mermaid,drawio,puml,png,svg}
Grep: "構成図|architecture|diagram|手順書|runbook|deploy|procedure" in *.md
```

**PR本文・コメントの検索**:
```bash
# Open PRの本文から構成図（Mermaidブロック）を探す
gh pr list --state open --json number,body --jq '.[] | select(.body | test("```mermaid")) | .number'

# PR本文からデプロイ手順セクションを探す
gh pr list --state open --json number,body --jq '.[] | select(.body | test("デプロイ|deploy|手順|procedure"; "i")) | .number'

# PRコメントからデプロイ検証結果を探す
gh pr view <number> --json comments --jq '.comments[] | select(.body | test("cdk diff|cdk deploy|検証|verify"; "i")) | .body'
```

**構成図として使える候補**:
- アーキテクチャ図（Mermaid/PlantUML/draw.io）を含むMarkdown
- 既存PDF構成図
- ADR/設計書内のシステム構成セクション
- **PR本文に記載されたMermaid構成図**（未マージでも最新の構成を反映している可能性）

**手順書として使える候補**:
- デプロイガイド、構築手順書
- Runbook
- ADR内のデプロイ手順セクション
- **PRコメントに記載されたデプロイ手順・検証結果**（実際のデプロイ経験に基づく実用的な情報）

#### 4.2 サーバ構成図の作成

**既存ドキュメントがある場合**:
1. 既存の構成図Markdownを読み取る
2. 申請対象サーバに関連する部分を特定する
3. 不足があればユーザーに確認（例: 「既存の構成図に○○の情報が含まれていませんが、追加が必要ですか？」）
4. 必要に応じて既存図を加工・統合して申請用構成図を作成

**既存ドキュメントがない場合**:
1. Phase 1で収集したインフラ情報からMermaid構成図を生成
2. 以下の要素を含める:
   - VPC/サブネット構成
   - EC2/ECS等のコンピュートリソース配置
   - ロードバランサ（NLB/ALB）
   - NAT Gateway/Instance
   - S3バケット
   - セキュリティグループの通信フロー
   - 監視・ログの送信先
3. ユーザーに構成図の内容を確認してもらう

**PDF化**:
- Mermaid図をMarkdownで出力し、ユーザーにPDF化手順を案内する
- 既存PDFがあればそのまま利用可能か確認する

#### 4.3 サーバ構築手順書の作成

**既存ドキュメントがある場合**:
1. 既存の手順書を読み取る
2. 申請対象サーバの手順として適切か評価する
3. 不足している手順やセクションがあればユーザーに確認
4. 必要に応じて加工・補完して申請用手順書を作成

**既存ドキュメントがない場合**:
1. IaCコード（CDK/Terraform等）から以下のセクションを持つ手順書を生成:
   - 前提条件（必要なツール、権限）
   - インフラデプロイ手順（CDK diff → deploy）
   - プロビジョニング手順（Ansible等があれば）
   - デプロイ後確認手順
   - ロールバック手順
2. ユーザーに手順の正確性を確認してもらう

**PDF化**:
- Markdownで手順書を出力し、ユーザーにPDF化手順を案内する
- 既存PDFがあればそのまま利用可能か確認する

#### 4.4 不明点の対話的確認

添付資料作成時に不明な点があれば、AskUserQuestionで確認する:
- 構成図に含めるべき範囲（VPC全体か、特定サーバのみか）
- 構成図に追記が必要な外部連携先
- 手順書に含めるべき前提条件や特殊な手順
- 既存PDFがそのまま使えるか、更新が必要か

## フォームテンプレート

フォームの全項目と記入例は [references/form-template.md](references/form-template.md) を参照。

## 推定ロジック一覧

| フォーム項目 | IaCコードの確認箇所 | 確認パターン |
|---|---|---|
| 所在・保管場所 | `env.region`, Stack props | `region`, `az`, `availabilityZone` をGrep |
| アクセス元 | Security Group IngressRules | `allowFrom`, `addIngressRule`, `connections.allowFrom` をGrep |
| アクセス制限 | SG + WAF設定 | `SecurityGroup`, `Waf`, `WebAcl` をGrep |
| 暗号化 | EBS encrypted, S3 encryption, TLS listeners | `encrypted`, `encryption`, `sslPolicy`, `certificate` をGrep |
| ログ各種 | FlowLogs, CloudWatch, ALB accessLog | `flowLog`, `logGroup`, `accessLog`, `containerInsights` をGrep |
| バックアップ | DLM, S3 versioning | `CfnLifecyclePolicy`, `versioned`, `backup` をGrep |
| 監視 | CloudWatch Alarms | `Alarm`, `metric`, `StatusCheck`, `cpuUtilization` をGrep |
| セキュリティパッチ | OS情報、マネージドサービス有無 | `machineImage`, `AmazonLinux`, `Fargate` を確認 |

## PR・Issue情報の活用マッピング

| PR/Issue情報源 | フォーム項目への活用 |
|---|---|
| PR本文の概要セクション | 「説明」「使用目的」の具体的な記述 |
| PR本文のMermaid構成図 | Phase 4 サーバ構成図のベース |
| PR変更ファイル一覧 | 申請対象リソースの特定 |
| PRコメントのデプロイ検証結果 | 「説明」の補足、Phase 4 手順書の検証セクション |
| PRレビューのセキュリティ指摘 | 「アクセス制限」「暗号化」の正確性向上 |
| リンクされたIssueの要件 | 「説明」「使用目的」の背景情報 |
| Issueのラベル | 「利用環境」の判断材料（`env:prd` 等） |

## gh CLIコマンドリファレンス

```bash
# Open PRの一覧
gh pr list --state open --json number,title,headRefName,isDraft,labels

# 現在ブランチのPR詳細
gh pr view --json number,title,state,body,files,comments,reviews

# 特定PRの本文
gh pr view <number> --json body --jq '.body'

# 特定PRの変更ファイル
gh pr view <number> --json files --jq '[.files[].path]'

# 特定PRのコメント
gh pr view <number> --json comments --jq '.comments[] | {author: .author.login, body}'

# コード行レビューコメント
gh api repos/{owner}/{repo}/pulls/<number>/comments --jq '.[] | {path, body, user: .user.login}'

# PRにリンクされたIssue番号の抽出
gh pr view <number> --json body --jq '.body' | grep -oE '#[0-9]+'

# Issue詳細
gh issue view <number> --json title,body,labels
```
