# Test Suite for Semantic Map Platform

マップ機能の包括的なテストスイート

## 📁 ディレクトリ構造

```
tests/
├── __init__.py                 # テストパッケージ初期化
├── conftest.py                 # pytest設定とフィクスチャ
├── pytest.ini                  # pytest設定ファイル
├── README.md                   # このファイル
│
├── unit/                       # ユニットテスト
│   ├── backend/               # バックエンドのユニットテスト
│   │   └── test_api.py       # APIエンドポイントテスト
│   └── frontend/              # フロントエンドのユニットテスト
│       ├── test_image_processing.py  # PGMパース、画像処理
│       ├── test_coordinates.py       # 座標変換
│       └── test_formatting.py        # フォーマット機能
│
├── integration/                # 統合テスト
│   ├── test_map_loading.py    # マップロード
│   └── test_profile_management.py  # プロファイル管理
│
├── e2e/                        # E2Eテスト
│   └── test_full_workflow.py  # フルワークフロー
│
├── fixtures/                   # テストフィクスチャ
│   └── maps/                  # テスト用マップデータ
│
└── outputs/                    # テスト出力（gitignore対象）
    ├── profiles/              # テストで生成されたプロファイル
    ├── screenshots/           # テストスクリーンショット
    └── reports/               # テストレポート
```

## 🎯 テスト戦略

### 1. ユニットテスト (Unit Tests)
- **対象**: 個別の関数・モジュール
- **速度**: 高速 (< 1秒/テスト)
- **依存**: 外部依存なし（モック使用）
- **カバレッジ**:
  - PGMファイルパース
  - 座標変換（ワールド座標 ↔ ピクセル座標）
  - フォーマット関数
  - バックエンドAPIエンドポイント

### 2. 統合テスト (Integration Tests)
- **対象**: 複数モジュールの連携
- **速度**: 中速 (1-5秒/テスト)
- **依存**: ファイルシステム、LocalStorage（シミュレーション）
- **カバレッジ**:
  - 実際のYAML/PGMファイルのロード
  - プロファイルの保存・読み込み
  - レイヤー管理のライフサイクル

### 3. E2Eテスト (End-to-End Tests)
- **対象**: アプリケーション全体のワークフロー
- **速度**: 低速 (5-30秒/テスト)
- **依存**: ブラウザ環境（Selenium/Playwright）
- **カバレッジ**:
  - マップのロード → プロファイル保存 → 再読み込み
  - レイヤー操作フロー
  - ズーム・パン操作

## 🚀 実行方法

### すべてのテストを実行
```bash
pytest
```

### 特定のタイプのテストのみ実行
```bash
# ユニットテストのみ
pytest -m unit

# 統合テストのみ
pytest -m integration

# E2Eテストのみ
pytest -m e2e
```

### 特定のファイルを実行
```bash
pytest tests/unit/backend/test_api.py
```

### 詳細な出力
```bash
pytest -v -s
```

### カバレッジレポート生成
```bash
pytest --cov=apps --cov-report=html
```

## 📊 テスト出力

### プロファイル
- テストで生成されたプロファイルは `tests/outputs/profiles/` に保存されます
- ファイル名: `test_profile_{timestamp}.json`
- 内容: マップデータ、メタデータ、レイヤー情報

### スクリーンショット
- E2Eテストのスクリーンショットは `tests/outputs/screenshots/` に保存されます
- ファイル名: `test_{test_name}_{timestamp}.png`

### レポート
- テスト結果のJSONレポートは `tests/outputs/reports/` に保存されます
- ファイル名: `test_report_{timestamp}.json`
- 内容: テスト結果、実行時間、エラー詳細

## 🔧 CI/CD統合

GitHub Actionsやその他のCI/CDツールで自動実行可能：

```yaml
# .github/workflows/test.yml の例
name: Test Suite
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup Python
        uses: actions/setup-python@v2
        with:
          python-version: '3.8'
      - name: Install dependencies
        run: pip install -r requirements.txt
      - name: Run tests
        run: pytest -m "unit or integration"
```

## ⚠️ 重要な留意点

### フロントエンドの自由描画機能
フロントエンドの自由描画機能（ユーザーが手動で行う描画操作）は、テストの対象外です。

**理由**:
1. テストの本質は「既存機能の破壊検出」と「コア機能の動作保証」
2. 自由描画は無限のバリエーションがあり、完全なテストは不可能
3. 描画ツールの内部ロジック（座標計算、色変換など）は別途ユニットテストでカバー

**テスト対象**:
- ✅ 描画ツールの座標計算ロジック
- ✅ 色変換関数
- ✅ レイヤー管理（描画レイヤーの作成・削除）
- ✅ プロファイル保存時に描画データが含まれること

**テスト対象外**:
- ❌ ユーザーが手動で行う描画操作のピクセル単位の検証
- ❌ 描画UIの見た目やスタイル
- ❌ マウスドラッグによる描画の軌跡

## 📝 テスト追加ガイド

新しいテストを追加する際は、以下のテンプレートを使用してください：

```python
"""
Test Module Name
テストの説明
"""
import pytest


class TestFeatureName:
    """機能名のテストクラス"""

    def test_basic_functionality(self):
        """基本機能のテスト"""
        # Arrange (準備)
        input_data = ...

        # Act (実行)
        result = function_under_test(input_data)

        # Assert (検証)
        assert result == expected_value

    def test_edge_cases(self):
        """エッジケースのテスト"""
        # エッジケースのテスト実装
        pass

    def test_error_handling(self):
        """エラーハンドリングのテスト"""
        with pytest.raises(ExpectedError):
            function_under_test(invalid_input)
```

## 🔍 トラブルシューティング

### テストが失敗する場合

1. **依存関係の確認**
   ```bash
   pip install -r requirements.txt
   ```

2. **テストデータの確認**
   - `data/maps/` にマップファイルが存在するか確認

3. **出力ディレクトリのクリーンアップ**
   ```bash
   rm -rf tests/outputs/*
   ```

4. **詳細なログを確認**
   ```bash
   pytest -v -s --log-cli-level=DEBUG
   ```

## 📚 関連ドキュメント

- [AI_GUIDELINES.md](../AI_GUIDELINES.md) - 開発ガイドライン
- [MODULE_INDEX.md](../MODULE_INDEX.md) - モジュール索引
- [README.md](../README.md) - プロジェクト概要
