# Claude Code Plugins

エッジコンピューティング・ロボティクス開発向けの Claude Code プラグイン集。

## 概要

このリポジトリは、以下の領域に特化したスキルとエージェントを提供する：

| カテゴリ | スキル名 | 説明 |
|----------|----------|------|
| VCS | `jujutsu` | jj の基本操作、Git からの移行、ワークフロー |
| エッジ共通 | `edge-common` | センサ・アクチュエータのカタログ、共通概念 |
| Jetson | `jetson` | tegrastats、CUDA最適化、JetPack管理 |
| Raspberry Pi | `raspberry-pi` | GPIO、周辺機器、OS設定 |
| LeRobot | `lerobot-basics` | インストール、データセット作成、トレーニング |
| LeRobot | `lerobot-fullstack` | ハードウェア構築、エッジデプロイ、統合 |

## インストール

```bash
# ローカルパスからインストール
/plugin install /path/to/claude-code-plugins

# GitHub からインストール（公開後）
/plugin install github.com/daikw/claude-code-plugins
```

## ディレクトリ構成

```
claude-code-plugins/
├── .claude-plugin/
│   └── plugin.json           # プラグイン設定
├── skills/
│   ├── jujutsu/              # jj VCS ワークフロー
│   ├── edge-common/          # エッジデバイス共通
│   ├── jetson/               # NVIDIA Jetson
│   ├── raspberry-pi/         # Raspberry Pi
│   ├── lerobot-basics/       # LeRobot 基本・トレーニング
│   └── lerobot-fullstack/    # LeRobot フルスタック
├── agents/                   # 自律エージェント（必要時のみ）
└── README.md
```

## スキルの使い方

プラグインをインストール後、スキル名を会話中で言及すると自動的に有効化される：

```
# 例：jujutsu スキルを使う
「jj で新しいブランチを作りたい」

# 例：Jetson スキルを使う
「Jetson の GPU 使用率を確認したい」
```

## 開発状況

- [ ] jujutsu
- [ ] edge-common
- [ ] jetson
- [ ] raspberry-pi
- [ ] lerobot-basics
- [ ] lerobot-fullstack

## ライセンス

MIT
