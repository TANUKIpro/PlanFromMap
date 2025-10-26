# 🗺️ Semantic Map Platform for HSR

RoboCup@Home DSPL向けの意味地図プラットフォーム。HSRが効率的にタスクを実行するための操作カタログと3D意味地図を提供します。

## 🎯 概要

従来のSLAM地図では提供できない「ドアの開け方」「引き出しの操作方法」「収納内部構造」などの**操作仕様**を事前定義し、ロボットの認識負荷を大幅に削減します。

### 主要機能

- 🚪 **操作カタログDB**: ドア・引き出し・家電の完全な操作仕様を管理
- 🗺️ **3D意味地図**: 2D占有格子から3D空間モデルを自動生成
- 🔍 **MapQLエンジン**: 自然言語的なクエリで地図情報を取得
- 🎮 **ビジュアルエディタ**: ブラウザベースの直感的な編集UI
- 🤖 **ROS統合**: HSRとのシームレスな連携

## 📋 要求仕様

- Ubuntu 20.04/22.04
- Node.js 18+ 
- Python 3.8+
- ROS Noetic/ROS2 Humble (optional)
- 最新のWebブラウザ (Chrome/Firefox推奨)

## 🚀 クイックスタート

### 1. リポジトリのクローン

```bash
git clone https://github.com/your-org/semantic-map-platform.git
cd semantic-map-platform
```

### 2. 依存関係のインストール

```bash
# Node.js依存関係
npm install

# Python依存関係
pip install -r requirements.txt

# 開発環境のセットアップ
npm run setup
```

### 3. 開発サーバーの起動

```bash
# すべてのサービスを起動
npm run dev

# 個別起動
npm run dev:backend   # APIサーバー (http://localhost:3000)
npm run dev:frontend  # Webアプリ (http://localhost:5173)
```

### 4. 初期データのインポート

```bash
# サンプル地図データのロード
npm run seed:maps

# 操作カタログのプリセット登録
npm run seed:catalog
```

## 🏗️ プロジェクト構造

```
semantic-map-platform/
├── apps/
│   ├── backend/          # Express.js APIサーバー
│   ├── frontend/         # React/Three.js Webアプリ  
│   └── ros-bridge/       # ROS統合ブリッジ
├── packages/
│   ├── core/            # コアロジック
│   ├── mapql/           # MapQLエンジン
│   ├── catalog/         # 操作カタログ
│   └── shared/          # 共有型定義・ユーティリティ
├── docs/                # ドキュメント
├── config/              # 設定ファイル
├── scripts/             # ビルド・デプロイスクリプト
└── tests/              # E2Eテスト
```

## 📖 ドキュメント

- [アーキテクチャ設計](docs/ARCHITECTURE.md)
- [操作カタログ仕様](docs/OPERATION-CATALOG.md)
- [MapQL言語仕様](docs/MAPQL.md)
- [API リファレンス](docs/API.md)
- [開発者ガイド](docs/DEVELOPMENT.md)

## 🔧 主要コマンド

| コマンド | 説明 |
|---------|------|
| `npm run dev` | 開発サーバー起動 |
| `npm run build` | プロダクションビルド |
| `npm run test` | テスト実行 |
| `npm run lint` | コード品質チェック |
| `npm run catalog:add` | 操作仕様を対話的に追加 |
| `npm run map:import` | 占有格子マップのインポート |

## 🤖 HSRとの連携

### ROS トピック/サービス

```bash
# 意味地図の配信
rostopic echo /semantic_map

# MapQLクエリ実行  
rosservice call /mapql_query "query: 'GET Operation FOR kitchen_door'"

# 操作実行状態
rostopic echo /operation_status
```

### サンプルコード

```python
# HSR側の実装例
from semantic_map_client import MapQLClient

client = MapQLClient()

# ドアの操作仕様を取得
operation = client.query("GET Operation FOR 'kitchen_door'")

# 実行
execute_operation(operation.spec)
```

## 🧪 テスト

```bash
# ユニットテスト
npm run test:unit

# 統合テスト  
npm run test:integration

# E2Eテスト
npm run test:e2e

# カバレッジレポート
npm run test:coverage
```

## 🤝 コントリビューション

[CONTRIBUTING.md](docs/CONTRIBUTING.md)を参照してください。

## 📝 ライセンス

MIT License - 詳細は[LICENSE](LICENSE)を参照してください。

## 🏆 謝辞

本プロジェクトはRoboCup@Home DSPLチームの支援により開発されています。
