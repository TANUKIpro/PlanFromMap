# テストディレクトリ構造

このディレクトリには、Semantic Map Platformの包括的なテストスイートが含まれています。

## ディレクトリ構造

```
tests/
├── unit/                    # ユニットテスト
│   ├── backend/            # Pythonバックエンドのユニットテスト
│   └── frontend/           # JavaScriptフロントエンドのユニットテスト
│       ├── modules/        # コアモジュールのテスト
│       ├── utils/          # ユーティリティ関数のテスト
│       ├── state/          # 状態管理のテスト
│       └── ui/             # UI層のテスト
├── integration/            # 統合テスト
│   ├── backend/           # バックエンド統合テスト
│   └── frontend/          # フロントエンド統合テスト
├── e2e/                    # E2Eテスト（Playwright）
├── fixtures/               # テストフィクスチャ
│   ├── maps/              # テスト用マップデータ
│   └── profiles/          # テスト用プロファイル
├── coverage/              # カバレッジレポート
└── reports/               # テストレポート
```

## テストの種類

### 1. ユニットテスト
個々のモジュール・関数の動作をテストします。

**実行方法:**
```bash
# Pythonユニットテスト
pytest tests/unit/backend/

# JavaScriptユニットテスト
npm run test:unit
```

### 2. 統合テスト
複数のモジュール間の連携をテストします。

**実行方法:**
```bash
# Python統合テスト
pytest tests/integration/backend/

# JavaScript統合テスト
npm run test:integration
```

### 3. E2Eテスト
実際のブラウザ環境で、ユーザーの操作フローをテストします。

**実行方法:**
```bash
npm run test:e2e
```

## テストフィクスチャ

### マップデータ
`tests/fixtures/maps/` に、テスト用のYAMLとPGMファイルを配置します。

### プロファイル
`tests/fixtures/profiles/` に、テスト用のプロファイルJSONを配置します。

## テストカバレッジ

カバレッジレポートは `tests/coverage/` に生成されます。

**カバレッジレポート生成:**
```bash
npm run test:coverage
```

## ベストプラクティス

1. **テストの独立性**: 各テストは他のテストに依存しない
2. **クリーンアップ**: テスト後は必ず状態をクリーンアップ
3. **わかりやすい名前**: テスト名は「何をテストするか」が明確
4. **Arrange-Act-Assert**: テストコードはAAA形式で記述
5. **テストデータの保存**: テストで生成されたプロファイルは `tests/reports/` に保存
