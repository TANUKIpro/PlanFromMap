# Claude自律テストシステム for PlanFromMap

## 概要

このシステムは、PlanFromMapプロジェクトにおいて、Claude Code on the webがコード生成中に自律的にテストを実行・更新するための完全なテストフレームワークです。

GitHub ActionsなどのCIツールを使用せず、Claude内で完結してテストの品質保証を行います。

## 特徴

- 🤖 **自律的テスト実行**: Claudeがコード変更時に自動的にテストを実行
- 🔨 **テスト自動生成**: 新機能追加時に適切なテストコードを自動生成
- 📊 **カバレッジ監視**: テストカバレッジを常に80%以上に維持
- 🔍 **統合テスト**: ユーザーシナリオベースの統合テストを実行
- 📝 **JavaScript検証**: Pythonベースでフロントエンドコードも検証

## ディレクトリ構造

```
claude_test_system/
├── tests/
│   ├── test_runner.py              # メインテストランナー
│   ├── claude_test_generator.py    # テスト自動生成ツール
│   ├── modules/                    # モジュール別テスト
│   │   └── test_layer_manager.py   # レイヤー管理のテスト例
│   ├── integration/                # 統合テスト
│   │   └── test_scenarios.py       # シナリオベーステスト
│   └── fixtures/                   # テストデータ
│       └── data/                   # テスト用データファイル
├── .claude/
│   ├── test_config.yaml           # Claude用テスト設定
│   ├── instructions.md            # 詳細な実行指示書
│   └── test_templates/            # テストテンプレート
│       └── templates.py           # 各種テンプレート
└── README.md                      # このファイル
```

## インストール

1. このシステムをPlanFromMapプロジェクトのルートにコピー:

```bash
# testsディレクトリをコピー
cp -r claude_test_system/tests /path/to/PlanFromMap/

# .claudeディレクトリをコピー
cp -r claude_test_system/.claude /path/to/PlanFromMap/
```

2. 必要なPythonパッケージをインストール:

```bash
pip install pytest pytest-cov unittest-mock
```

## 使用方法

### 基本的なテスト実行

```bash
# 全テストを実行
python tests/test_runner.py

# 特定機能のテストを実行
python tests/test_runner.py verify layerManager

# ファイル監視モード（変更を検出して自動実行）
python tests/test_runner.py watch
```

### テストの自動生成

```bash
# 関数のテストを生成
python tests/claude_test_generator.py function create_layer modules.layerManager

# クラスのテストを生成
python tests/claude_test_generator.py class LayerManager modules.layerManager

# JavaScriptモジュールの検証コードを生成
python tests/claude_test_generator.py javascript layerManager
```

### 統合テストの実行

```bash
# 全シナリオを実行
python tests/integration/test_scenarios.py

# 特定シナリオを実行
python tests/integration/test_scenarios.py "基本的なマップ読み込みフロー"
```

## Claude向けワークフロー

### 1. 新機能実装時

```python
# STEP 1: テストファイルを自動生成
python tests/claude_test_generator.py function new_feature module_name

# STEP 2: テストを実装（失敗することを確認）
# tests/modules/test_module_name.py を編集

# STEP 3: 機能を実装

# STEP 4: テストを実行（成功することを確認）
python tests/test_runner.py

# STEP 5: カバレッジを確認
# 出力されるレポートで80%以上を確認
```

### 2. バグ修正時

```python
# STEP 1: バグを再現するテストを作成
def test_bug_reproduction():
    """Issue #XXX: バグの説明"""
    # バグを再現するコード
    with self.assertRaises(ExpectedError):
        buggy_function()

# STEP 2: テストが失敗することを確認
python tests/test_runner.py

# STEP 3: バグを修正

# STEP 4: テストが成功することを確認
python tests/test_runner.py
```

### 3. リファクタリング時

```bash
# STEP 1: 既存のテストが全て成功することを確認
python tests/test_runner.py

# STEP 2: リファクタリングを実行

# STEP 3: テストが引き続き成功することを確認
python tests/test_runner.py
```

## テスト設定のカスタマイズ

`.claude/test_config.yaml` を編集して、プロジェクトに合わせた設定が可能：

```yaml
coverage:
  minimum_overall: 80      # 全体の最小カバレッジ
  minimum_per_file: 70     # ファイルごとの最小カバレッジ

test_execution:
  on_feature_add:
    min_coverage: 85       # 新機能追加時の要求カバレッジ
```

## ベストプラクティス

1. **テスト駆動開発（TDD）**
   - 実装前にテストを書く
   - Red → Green → Refactor のサイクル

2. **独立したテスト**
   - 各テストは他のテストに依存しない
   - 実行順序に依存しない

3. **明確な命名**
   - `test_<機能>_<条件>_<期待結果>`

4. **適切なモック**
   - 外部APIやDBはモック化
   - テスト環境を汚染しない

5. **継続的な実行**
   - コード変更時は必ずテスト実行
   - カバレッジの維持

## トラブルシューティング

### ImportError が発生する場合

```python
# tests/test_runner.py の先頭に追加
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
```

### テストが相互に影響する場合

```python
def setUp(self):
    """各テストの前に状態をリセット"""
    self.test_data = []
    
def tearDown(self):
    """各テストの後にクリーンアップ"""
    self.test_data.clear()
```

### モックが正しく動作しない場合

```python
# パッチの適用場所を確認
@patch('実際に使用されるモジュール.関数')
def test_with_mock(self, mock_func):
    mock_func.return_value = 'expected'
```

## サポートされる機能

- ✅ Pythonモジュールの単体テスト
- ✅ 統合テスト（シナリオベース）
- ✅ JavaScriptコードの静的検証
- ✅ テストカバレッジ計算
- ✅ テストの自動生成
- ✅ ファイル監視モード
- ✅ エラーハンドリングテスト
- ✅ モックを使用したテスト

## 今後の拡張予定

- [ ] パフォーマンステスト
- [ ] セキュリティテスト
- [ ] E2Eテスト（Seleniumなし）
- [ ] ビジュアル回帰テスト
- [ ] APIレスポンステスト

## ライセンス

MIT License

## 貢献

プルリクエストを歓迎します。大きな変更の場合は、まずissueを開いて変更内容を議論してください。

## 問い合わせ

質問や提案がある場合は、GitHubのissueを作成してください。
