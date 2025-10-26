# プロジェクト構造

```
semantic-map-platform/
├── 📁 apps/                      # アプリケーション
│   ├── 📁 backend/              # Express.js APIサーバー
│   │   ├── 📁 src/
│   │   │   ├── 📁 controllers/ # APIコントローラー
│   │   │   ├── 📁 services/    # ビジネスロジック
│   │   │   ├── 📁 models/      # データモデル
│   │   │   ├── 📁 middleware/  # Express middleware
│   │   │   └── 📄 index.ts     # エントリーポイント
│   │   ├── 📁 config/          # 設定ファイル
│   │   ├── 📄 package.json
│   │   ├── 📄 tsconfig.json
│   │   └── 📄 Dockerfile
│   │
│   ├── 📁 frontend/            # React Webアプリ
│   │   ├── 📁 src/
│   │   │   ├── 📁 components/ # UIコンポーネント
│   │   │   ├── 📁 pages/      # ページコンポーネント
│   │   │   ├── 📁 hooks/      # カスタムフック
│   │   │   ├── 📁 utils/      # ユーティリティ
│   │   │   └── 📁 api/        # APIクライアント
│   │   ├── 📁 public/         # 静的ファイル
│   │   ├── 📄 package.json
│   │   ├── 📄 tsconfig.json
│   │   ├── 📄 vite.config.ts
│   │   └── 📄 Dockerfile.dev
│   │
│   └── 📁 ros-bridge/         # ROS統合ブリッジ
│       ├── 📁 src/
│       ├── 📁 config/
│       ├── 📄 package.json
│       └── 📄 Dockerfile
│
├── 📁 packages/                # 共有パッケージ（モノレポ）
│   ├── 📁 core/               # コアロジック
│   │   ├── 📁 src/
│   │   │   ├── 📄 map-generator.ts    # 3D生成
│   │   │   ├── 📄 semantic-model.ts   # データモデル
│   │   │   └── 📄 importer.ts         # インポート
│   │   ├── 📁 tests/
│   │   └── 📄 package.json
│   │
│   ├── 📁 mapql/              # MapQLエンジン
│   │   ├── 📁 src/
│   │   │   ├── 📁 parser/    # PEG.jsパーサー
│   │   │   ├── 📁 engine/    # 実行エンジン
│   │   │   └── 📁 optimizer/ # クエリ最適化
│   │   ├── 📁 tests/
│   │   └── 📄 package.json
│   │
│   ├── 📁 catalog/            # 操作カタログ
│   │   ├── 📁 src/
│   │   │   ├── 📁 models/    # カタログモデル
│   │   │   ├── 📁 validators/# バリデーター
│   │   │   └── 📁 templates/ # プリセット
│   │   ├── 📁 tests/
│   │   └── 📄 package.json
│   │
│   └── 📁 shared/             # 共有型定義・ユーティリティ
│       ├── 📁 src/
│       │   ├── 📁 types/     # TypeScript型定義
│       │   │   └── 📄 operation-catalog.types.ts
│       │   ├── 📁 utils/     # 共通ユーティリティ
│       │   └── 📁 constants/ # 定数定義
│       └── 📄 package.json
│
├── 📁 docs/                   # ドキュメント
│   ├── 📄 ARCHITECTURE.md    # システム設計
│   ├── 📄 OPERATION-CATALOG.md # カタログ仕様
│   ├── 📄 MAPQL.md          # MapQL言語仕様
│   ├── 📄 API.md            # APIリファレンス
│   ├── 📄 DEVELOPMENT.md    # 開発者ガイド
│   ├── 📄 CONTRIBUTING.md   # コントリビューションガイド
│   ├── 📁 architecture/     # 詳細設計図
│   ├── 📁 api/             # API詳細
│   ├── 📁 guides/          # 使い方ガイド
│   └── 📁 adr/             # アーキテクチャ決定記録
│
├── 📁 config/                # 設定ファイル
│   ├── 📄 nginx.conf        # Nginx設定
│   ├── 📄 redis.conf        # Redis設定
│   └── 📄 jest.config.js    # Jest設定
│
├── 📁 scripts/               # ユーティリティスクリプト
│   ├── 📁 build/            # ビルドスクリプト
│   ├── 📁 deploy/           # デプロイスクリプト
│   ├── 📁 seed/             # 初期データ投入
│   └── 📄 setup.sh          # 初期セットアップ
│
├── 📁 tests/                 # E2Eテスト・共通テスト
│   ├── 📁 e2e/              # E2Eテスト
│   ├── 📁 fixtures/         # テストデータ
│   └── 📁 utils/            # テストユーティリティ
│
├── 📁 data/                  # データディレクトリ
│   ├── 📁 maps/             # 地図データ
│   ├── 📁 catalogs/         # カタログデータ
│   │   ├── 📁 presets/      # プリセット
│   │   └── 📁 templates/    # テンプレート
│   ├── 📁 schemas/          # JSONスキーマ
│   ├── 📁 uploads/          # アップロード
│   ├── 📁 exports/          # エクスポート
│   └── 📁 backups/          # バックアップ
│
├── 📄 README.md              # プロジェクト概要
├── 📄 package.json           # ルートpackage.json（モノレポ）
├── 📄 tsconfig.json          # TypeScript設定
├── 📄 docker-compose.yml     # Docker構成
├── 📄 .env.example           # 環境変数テンプレート
├── 📄 .gitignore            # Git除外設定
├── 📄 .eslintrc.js          # ESLint設定
├── 📄 .prettierrc           # Prettier設定
└── 📄 LICENSE               # ライセンス
```

## 主要ファイル作成済み

✅ **作成済み**:
- `/README.md` - プロジェクト概要とクイックスタート
- `/package.json` - モノレポ構成とスクリプト
- `/tsconfig.json` - TypeScript設定
- `/.env.example` - 環境変数テンプレート
- `/.gitignore` - Git除外設定
- `/docker-compose.yml` - Docker構成
- `/packages/shared/src/types/operation-catalog.types.ts` - 操作カタログ型定義
- `/docs/ARCHITECTURE.md` - システムアーキテクチャ
- `/docs/OPERATION-CATALOG.md` - 操作カタログ仕様

## 次のステップ

### 1. 初期セットアップ

```bash
# リポジトリ初期化
git init
git add .
git commit -m "Initial project structure"

# 依存関係インストール
npm install
```

### 2. 各パッケージのpackage.json作成

各パッケージディレクトリで：
```bash
npm init -y
```

### 3. 基本的な実装開始

優先順位：
1. `packages/shared` - 型定義とユーティリティ
2. `packages/catalog` - 操作カタログのCRUD
3. `apps/backend` - REST APIエンドポイント
4. `apps/frontend` - 基本UI

### 4. テスト環境構築

```bash
# Jestセットアップ
npm install --save-dev jest @types/jest ts-jest

# E2Eテスト
npm install --save-dev @playwright/test
```

## 開発開始コマンド

```bash
# Dockerで全サービス起動
docker-compose up

# 個別起動（開発時）
npm run dev:backend   # バックエンド
npm run dev:frontend  # フロントエンド

# テスト実行
npm test

# ビルド
npm run build
```
