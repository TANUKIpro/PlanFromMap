# Archived Configuration Files

このディレクトリには、現在は使用されていないが将来的に使用される可能性のある設定ファイルが保管されています。

## package.json

Node.jsプロジェクトの設定ファイルです。

### 現在の状態

- **使用していません**: 現在のプロジェクトはPythonベースです
- **将来の計画**: フル機能版でTypeScript/Node.jsワークスペースを実装する際に使用予定

### 内容

- npm workspaces設定（apps/*、packages/*）
- 開発用スクリプト
- 依存関係の定義

## tsconfig.json

TypeScriptコンパイラの設定ファイルです。

### 現在の状態

- **使用していません**: フロントエンドはVanilla JavaScriptで実装されています
- **将来の計画**: TypeScriptへの移行時に使用予定

### 内容

- コンパイラオプション
- プロジェクト参照（packages/core、mapql、catalog等）
- パスエイリアス設定

## これらのファイルについて

### なぜアーカイブ？

1. **実態との乖離**: READMEではNode.js/TypeScriptプロジェクトのように記載されていましたが、実際はPythonベースのシンプルなプロジェクトです
2. **混乱の防止**: 未使用の設定ファイルがルートにあると、開発者が混乱する可能性があります
3. **将来の拡張性**: 完全に削除せず、将来の拡張時に参照できるようにアーカイブしています

### 使用したい場合

これらのファイルを使用したい場合は、以下の手順を実行してください：

```bash
# ルートディレクトリに戻す
git mv .archived/package.json ./
git mv .archived/tsconfig.json ./

# Node.js依存関係をインストール
npm install

# TypeScriptをセットアップ
npm run build
```

ただし、現在のプロジェクト構造では、これらのファイルが参照するワークスペース（packages/core、mapql、catalog等）が存在しないため、エラーが発生します。

## 推奨される開発方法

現在のプロジェクトでは、以下の方法を使用してください：

### Python + uvを使用（推奨）

```bash
# uvのインストール
curl -LsSf https://astral.sh/uv/install.sh | sh

# 依存関係のインストール
uv sync

# サーバーの起動
uv run python server.py
```

### Python + pipを使用（従来の方法）

```bash
# 依存関係のインストール
pip install -r requirements.txt

# サーバーの起動
python server.py
```
