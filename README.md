# Semantic Map Platform for HSR

<div align="center">

### Language / 言語

**[🇯🇵 日本語](#japanese) | [🇬🇧 English](#english)**

</div>

---

<a name="japanese"></a>
## 🇯🇵 日本語

RoboCup@Home DSPL向けの意味地図プラットフォーム。HSRが効率的にタスクを実行するための操作カタログと3D意味地図を提供します。

### 概要

従来のSLAM地図では提供できない「ドアの開け方」「引き出しの操作方法」「収納内部構造」などの操作仕様を事前定義し、ロボットの認識負荷を削減します。

### 主要機能

- 操作カタログDB: ドア・引き出し・家電の完全な操作仕様を管理
- 3D意味地図: 2D占有格子から3D空間モデルを自動生成
- **オブジェクトプロパティ管理**: 家具・オブジェクトの詳細情報付与と3D表現
  - カテゴリ別3Dモデル（棚、箱、テーブル、扉、壁）
  - リアルタイム3Dプレビュー
  - 前面方向と高さ情報の管理
- MapQLエンジン: 自然言語的なクエリで地図情報を取得
- ビジュアルエディタ: ブラウザベースの直感的な編集UI
- ROS統合: HSRとのシームレスな連携
- 生成AI最適化: モジュール化により、生成AIのトークン消費を89%削減

### 要求仕様

- Ubuntu 20.04/22.04
- Node.js 18+
- Python 3.8+
- ROS Noetic/ROS2 Humble (optional)
- 最新のWebブラウザ (Chrome/Firefox推奨)

### クイックスタート

#### uvでの起動（推奨・最新）

uvは高速なPythonパッケージマネージャーです。最新の依存関係管理を使用します。

##### 1. リポジトリのクローン

```bash
git clone https://github.com/TANUKIpro/PlanFromMap.git
cd PlanFromMap
```

##### 2. uvのインストール

```bash
# Linux/macOS
curl -LsSf https://astral.sh/uv/install.sh | sh
```

##### 3. 依存関係のインストール

```bash
uv sync
```

##### 4. サーバーの起動

```bash
uv run python server.py
```

##### 5. ブラウザでアクセス

```
http://localhost:5173
```

バックエンドAPI(port:3000)とフロントエンド(port:5173)が起動します。

#### pipでの起動（従来の方法）

uvを使用しない場合は、従来のpipでも起動できます。

##### 1. リポジトリのクローン

```bash
git clone https://github.com/TANUKIpro/PlanFromMap.git
cd PlanFromMap
```

##### 2. Python依存関係のインストール

```bash
pip install -r requirements.txt
```

##### 3. サーバーの起動

```bash
python server.py
```

##### 4. ブラウザでアクセス

```
http://localhost:5173
```

詳細な手順は [QUICKSTART.md](docs/QUICKSTART.md) を参照してください。

### プロジェクト構造

```
semantic-map-platform/
├── server.py            # メインサーバー起動スクリプト (Python)
├── requirements.txt     # Python依存関係
├── apps/
│   ├── backend/         # Flask APIサーバー
│   │   └── server.py   # バックエンドAPI (port:3000)
│   ├── frontend/        # Webアプリ
│   │   ├── server.py   # フロントエンドサーバー (port:5173)
│   │   └── static/     # 静的ファイル (HTML/CSS/JS)
│   └── ros-bridge/      # ROS統合ブリッジ
├── packages/
│   ├── core/            # コアロジック
│   ├── mapql/           # MapQLエンジン
│   ├── catalog/         # 操作カタログ
│   └── shared/          # 共有型定義・ユーティリティ
├── docs/                # ドキュメント
├── data/                # データディレクトリ
│   ├── maps/           # 地図データ
│   └── catalogs/       # カタログデータ
├── config/              # 設定ファイル
├── scripts/             # ビルド・デプロイスクリプト
└── tests/              # E2Eテスト
```

### ドキュメント

### ユーザー向け
- [2Dマップエディタ - オブジェクトプロパティ管理](docs/MAP-EDITOR.md) - 家具・オブジェクト情報の付与と3D表現

### 開発者向け
- [生成AI向けガイドライン](docs/AI_GUIDELINES.md) - 生成AIと協働開発するための完全ガイド
- [モジュール索引](docs/MODULE_INDEX.md) - 全モジュールの詳細と依存関係

#### システム設計
- [アーキテクチャ設計](docs/ARCHITECTURE.md)
- [操作カタログ仕様](docs/OPERATION-CATALOG.md)

### 主要コマンド

#### Python版（シンプル・高速）

| コマンド | 説明 |
|---------|------|
| `python server.py` | サーバー起動 (バックエンド+フロントエンド) |
| `python apps/backend/server.py` | バックエンドのみ起動 |
| `python apps/frontend/server.py` | フロントエンドのみ起動 |

#### Node.js版（フル機能）

| コマンド | 説明 |
|---------|------|
| `npm run dev` | 開発サーバー起動 |
| `npm run build` | プロダクションビルド |
| `npm run test` | テスト実行 |
| `npm run lint` | コード品質チェック |
| `npm run catalog:add` | 操作仕様を対話的に追加 |
| `npm run map:import` | 占有格子マップのインポート |

### HSRとの連携

#### ROS トピック/サービス

```bash
# 意味地図の配信
rostopic echo /semantic_map

# MapQLクエリ実行
rosservice call /mapql_query "query: 'GET Operation FOR kitchen_door'"

# 操作実行状態
rostopic echo /operation_status
```

#### サンプルコード

```python
# HSR側の実装例
from semantic_map_client import MapQLClient

client = MapQLClient()

# ドアの操作仕様を取得
operation = client.query("GET Operation FOR 'kitchen_door'")

# 実行
execute_operation(operation.spec)
```

### テスト

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

### ライセンス

MIT License - 詳細は[LICENSE](LICENSE)を参照してください。

---

<a name="english"></a>
## 🇬🇧 English

A semantic map platform for RoboCup@Home DSPL. Provides an operation catalog and 3D semantic maps to enable HSR to perform tasks efficiently.

### Overview

This platform pre-defines operation specifications such as "how to open doors", "how to operate drawers", and "internal structure of storage spaces" that cannot be provided by traditional SLAM maps, thereby reducing the robot's recognition burden.

### Key Features

- Operation Catalog DB: Manages complete operation specifications for doors, drawers, and appliances
- 3D Semantic Map: Automatically generates 3D spatial models from 2D occupancy grids
- MapQL Engine: Natural language-like queries for map information
- Visual Editor: Intuitive browser-based editing UI
- ROS Integration: Seamless integration with HSR
- Generative AI Optimization: Modular architecture reduces AI token consumption by 89%

### Requirements

- Ubuntu 20.04/22.04
- Node.js 18+
- Python 3.8+
- ROS Noetic/ROS2 Humble (optional)
- Modern web browser (Chrome/Firefox recommended)

### Quick Start

#### uv Launch (Recommended - Modern)

uv is a fast Python package manager. It uses the latest dependency management.

##### 1. Clone the repository

```bash
git clone https://github.com/TANUKIpro/PlanFromMap.git
cd PlanFromMap
```

##### 2. Install uv

```bash
# Linux/macOS
curl -LsSf https://astral.sh/uv/install.sh | sh
```

##### 3. Install dependencies

```bash
uv sync
```

##### 4. Start the server

```bash
uv run python server.py
```

##### 5. Access in browser

```
http://localhost:5173
```

This will start both the backend API (port:3000) and frontend (port:5173).

#### pip Launch (Traditional Method)

If you don't want to use uv, you can use traditional pip.

##### 1. Clone the repository

```bash
git clone https://github.com/TANUKIpro/PlanFromMap.git
cd PlanFromMap
```

##### 2. Install Python dependencies

```bash
pip install -r requirements.txt
```

##### 3. Start the server

```bash
python server.py
```

##### 4. Access in browser

```
http://localhost:5173
```

For detailed instructions, see [QUICKSTART.md](docs/QUICKSTART.md).

### Project Structure

```
semantic-map-platform/
├── server.py            # Main server startup script (Python)
├── requirements.txt     # Python dependencies
├── apps/
│   ├── backend/         # Flask API server
│   │   └── server.py   # Backend API (port:3000)
│   ├── frontend/        # Web application
│   │   ├── server.py   # Frontend server (port:5173)
│   │   └── static/     # Static files (HTML/CSS/JS)
│   └── ros-bridge/      # ROS integration bridge
├── packages/
│   ├── core/            # Core logic
│   ├── mapql/           # MapQL engine
│   ├── catalog/         # Operation catalog
│   └── shared/          # Shared type definitions & utilities
├── docs/                # Documentation
├── data/                # Data directory
│   ├── maps/           # Map data
│   └── catalogs/       # Catalog data
├── config/              # Configuration files
├── scripts/             # Build & deployment scripts
└── tests/              # E2E tests
```

### Documentation

#### For Developers
- [AI Guidelines](docs/AI_GUIDELINES.md) - Complete guide for collaborative development with generative AI
- [Module Index](docs/MODULE_INDEX.md) - Detailed information and dependencies of all modules

#### System Design
- [Architecture Design](docs/ARCHITECTURE.md)
- [Operation Catalog Specification](docs/OPERATION-CATALOG.md)

### Main Commands

#### Using uv (Recommended)

| Command | Description |
|---------|-------------|
| `uv sync` | Install/update dependencies |
| `uv run python server.py` | Start server (backend + frontend) |
| `uv run python apps/backend/server.py` | Start backend only |
| `uv run python apps/frontend/server.py` | Start frontend only |

#### Using pip (Traditional)

| Command | Description |
|---------|-------------|
| `pip install -r requirements.txt` | Install dependencies |
| `python server.py` | Start server (backend + frontend) |
| `python apps/backend/server.py` | Start backend only |
| `python apps/frontend/server.py` | Start frontend only |

### HSR Integration

#### ROS Topics/Services

```bash
# Publish semantic map
rostopic echo /semantic_map

# Execute MapQL query
rosservice call /mapql_query "query: 'GET Operation FOR kitchen_door'"

# Operation execution status
rostopic echo /operation_status
```

#### Sample Code

```python
# HSR implementation example
from semantic_map_client import MapQLClient

client = MapQLClient()

# Get door operation specification
operation = client.query("GET Operation FOR 'kitchen_door'")

# Execute
execute_operation(operation.spec)
```

### Testing

```bash
# Unit tests
npm run test:unit

# Integration tests
npm run test:integration

# E2E tests
npm run test:e2e

# Coverage report
npm run test:coverage
```

### License

MIT License - See [LICENSE](LICENSE) for details.

---

<div align="center">

**[⬆️ Back to top](#semantic-map-platform-for-hsr)**

</div>
