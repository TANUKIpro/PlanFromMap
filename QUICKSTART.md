# クイックスタートガイド

Pythonコマンドでローカル環境にてサーバを起動する手順です。

## 前提条件

- Python 3.8以上
- pip (Pythonパッケージマネージャー)

## セットアップ手順

### 1. リポジトリのクローン

```bash
git clone <repository-url>
cd semantic-map-platform
```

### 2. Python依存関係のインストール

```bash
pip install -r requirements.txt
```

または仮想環境を使用する場合:

```bash
# 仮想環境の作成
python -m venv venv

# 仮想環境の有効化 (Linux/Mac)
source venv/bin/activate

# 仮想環境の有効化 (Windows)
venv\Scripts\activate

# 依存関係のインストール
pip install -r requirements.txt
```

### 3. サーバーの起動

```bash
python server.py
```

このコマンドで以下が起動します:
- バックエンドAPI: http://localhost:3000
- フロントエンドUI: http://localhost:5173

### 4. ブラウザでアクセス

ブラウザで以下のURLにアクセスしてください:

```
http://localhost:5173
```

## 主要なAPIエンドポイント

### ヘルスチェック
```bash
curl http://localhost:3000/health
```

### 操作カタログ一覧
```bash
curl http://localhost:3000/api/operations
```

### 特定の操作仕様を取得
```bash
curl http://localhost:3000/api/operations/kitchen_door_001
```

### MapQLクエリ実行
```bash
curl -X POST http://localhost:3000/api/mapql/query \
  -H "Content-Type: application/json" \
  -d '{"query": "GET Operation FOR '\''kitchen_door'\''"}'
```

### 統計情報
```bash
curl http://localhost:3000/api/stats
```

## サーバーの停止

サーバーを停止するには、ターミナルで `Ctrl+C` を押してください。

## トラブルシューティング

### ポートが既に使用されている場合

別のプログラムがポート3000または5173を使用している場合、以下のエラーが表示されます:

```
Address already in use
```

この場合は、以下のコマンドでポートを使用しているプロセスを確認して終了してください:

```bash
# ポート3000を使用しているプロセスを確認
lsof -i :3000

# ポート5173を使用しているプロセスを確認
lsof -i :5173
```

### 依存関係のエラー

以下のエラーが表示される場合:

```
ModuleNotFoundError: No module named 'flask'
```

依存関係を再インストールしてください:

```bash
pip install -r requirements.txt
```

## 次のステップ

- [アーキテクチャ設計](docs/ARCHITECTURE.md)を読む
- [操作カタログ仕様](docs/OPERATION-CATALOG.md)を確認する
- HSRとのROS連携を設定する
