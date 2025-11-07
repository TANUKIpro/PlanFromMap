# Configuration Files

このディレクトリには、プロジェクトの設定ファイルが含まれています。

## docker-compose.yml

Docker Composeを使用した開発環境の設定ファイルです。

### 使用方法

```bash
# すべてのサービスを起動
docker-compose -f config/docker-compose.yml up -d

# 特定のサービスのみ起動
docker-compose -f config/docker-compose.yml up backend frontend

# ROS連携を含めて起動
docker-compose -f config/docker-compose.yml --profile ros up

# サービスを停止
docker-compose -f config/docker-compose.yml down
```

### 注意事項

- 現在、Dockerfileは実装されていません
- Docker環境は**オプショナル機能**です
- 通常の開発には`python server.py`または`uv run server.py`を使用してください

### サービス構成

- **backend**: Flaskバックエンドサーバー (ポート3000)
- **frontend**: フロントエンドサーバー (ポート5173)
- **ros-bridge**: ROSブリッジ (ポート9090) - オプショナル
- **redis**: キャッシュサーバー (ポート6379)
- **postgres**: PostgreSQLデータベース (ポート5432) - プロダクション用
- **nginx**: リバースプロキシ (ポート80/443) - プロダクション用
- **adminer**: データベース管理UI (ポート8080) - 開発用

## 環境変数

環境変数は`.env`ファイルで設定できます。テンプレートとして`.env.example`を使用してください。

```bash
cp .env.example .env
# .envファイルを編集
```
