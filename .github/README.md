# GitHub Configuration

このディレクトリには、GitHubリポジトリの設定ファイルが含まれています。

## 📁 ディレクトリ構造

```
.github/
├── workflows/           # GitHub Actions ワークフロー
│   ├── test.yml        # メインテストスイート
│   ├── pr-check.yml    # PRチェック
│   └── scheduled-tests.yml  # 定期テスト
├── CICD.md             # CI/CDドキュメント
└── README.md           # このファイル
```

## 🔧 GitHub Actions ワークフロー

### test.yml - メインテストスイート

完全なテストスイートを実行します。

**実行条件**:
- `main`, `develop`, `claude/**` ブランチへのpush
- `main`, `develop` ブランチへのPR

**含まれるジョブ**:
- ユニットテスト (Python 3.8-3.11)
- 統合テスト
- カバレッジレポート
- E2Eテスト
- テスト結果サマリー

### pr-check.yml - PRチェック

PRの品質チェックを高速実行します。

**実行条件**:
- PR作成、更新時

**含まれるチェック**:
- PRタイトル検証
- クイックテスト（2分）
- PRサイズ確認
- コード品質チェック

### scheduled-tests.yml - 定期テスト

定期的に完全なテストを実行します。

**実行条件**:
- 毎日2:00 UTC（自動）
- 手動トリガー

**特徴**:
- スローE2Eテストを含む
- パフォーマンステスト
- 失敗時に自動でイシュー作成

## 🚀 クイックスタート

### ローカルでテストを実行

```bash
# すべてのテスト
pytest

# クイックテスト（PR時と同じ）
pytest tests/unit/frontend/ -v --maxfail=3

# カバレッジ付き
pytest --cov=apps --cov-report=html
```

### ワークフローの状態確認

1. GitHubリポジトリの "Actions" タブにアクセス
2. 最新の実行状態を確認
3. 失敗している場合はログを確認

## 📊 バッジの追加（オプション）

READMEに以下のバッジを追加できます：

```markdown
![Test Suite](https://github.com/TANUKIpro/PlanFromMap/workflows/Test%20Suite/badge.svg)
![PR Check](https://github.com/TANUKIpro/PlanFromMap/workflows/Pull%20Request%20Check/badge.svg)
```

## 🔍 トラブルシューティング

### テストが失敗する場合

1. **ローカルで再現**: 同じコマンドをローカルで実行
2. **ログ確認**: Actions タブで詳細ログを確認
3. **依存関係**: `requirements.txt` が最新か確認

### ワークフローが実行されない場合

1. **ブランチ名確認**: トリガー条件に一致しているか
2. **YAML構文**: ワークフローファイルの構文エラーを確認
3. **権限**: リポジトリの Actions 設定を確認

## 📚 詳細ドキュメント

- [CI/CDドキュメント](./CICD.md) - 詳細な技術仕様
- [テストスイート](../tests/README.md) - テストの説明
- [開発ガイドライン](../AI_GUIDELINES.md) - コーディング規約

## 🔄 ワークフローの更新

ワークフローを変更する場合：

1. `.github/workflows/` 内のファイルを編集
2. ローカルでYAML構文を確認
3. PRを作成してレビュー
4. マージ後、次回から新しいワークフローが適用

## 💡 ヒント

### 高速化のテクニック

1. **キャッシュ活用**: `cache: 'pip'` を使用
2. **並列実行**: マトリックスビルドを活用
3. **条件付き実行**: 不要なジョブをスキップ

### セキュリティ

1. **シークレット**: GitHub Secretsで管理
2. **依存関係**: Dependabotで自動更新
3. **権限**: 最小限の権限で実行

## 📞 サポート

質問や問題がある場合：

1. [CICD.md](./CICD.md) を確認
2. GitHubイシューを作成
3. 既存のワークフロー実行ログを共有
