# Claude自律テストシステム - 実行指示書

## 概要

このドキュメントは、Claude Code on the webがPlanFromMapプロジェクトの開発中に自律的にテストを実行・更新するための詳細な指示書です。

## 基本原則

### 1. テスト駆動開発（TDD）の実践

```python
# STEP 1: まず失敗するテストを書く
def test_new_feature():
    result = new_feature("input")
    assert result == "expected_output"  # この時点では失敗する

# STEP 2: 最小限のコードで成功させる
def new_feature(input):
    return "expected_output"

# STEP 3: リファクタリング
def new_feature(input):
    # 実装を改善
    validated_input = validate(input)
    processed = process(validated_input)
    return format_output(processed)
```

### 2. 自動テスト実行のタイミング

- **新機能追加時**: 必ずテストを同時に作成
- **バグ修正時**: まず再現テストを作成
- **リファクタリング時**: 既存テストが通ることを確認
- **PR作成前**: 全テストスイートを実行

## テスト作成ワークフロー

### 新機能実装時

```bash
# 1. テストファイルを生成
python tests/claude_test_generator.py function create_layer modules.layerManager

# 2. 生成されたテストを編集
# tests/modules/test_layer_manager.py を開いて具体的なテストを実装

# 3. テストを実行（失敗することを確認）
python tests/test_runner.py

# 4. 機能を実装

# 5. テストを再実行（成功することを確認）
python tests/test_runner.py

# 6. カバレッジを確認
# レポートで80%以上のカバレッジを確認
```

### バグ修正時

```python
# 1. バグを再現するテストを作成
def test_bug_reproduction():
    """
    Issue #123: レイヤー削除時にエラーが発生する
    """
    # バグを再現する条件を設定
    layer = create_layer('test', 'Test', 'drawing')
    layer['data'] = None  # この状態でバグが発生
    
    # バグが発生することを確認
    with self.assertRaises(AttributeError):
        delete_layer('test')  # ここでエラーが発生する

# 2. バグを修正
def delete_layer(layer_id):
    layer = get_layer_by_id(layer_id)
    if layer and layer.get('data'):  # Noneチェックを追加
        # 削除処理
        pass

# 3. テストが通ることを確認
```

## テストパターン集

### パターン1: 基本的な関数テスト

```python
def test_function_basic():
    """基本動作のテスト"""
    # Arrange（準備）
    input_data = "test_input"
    expected = "expected_output"
    
    # Act（実行）
    result = target_function(input_data)
    
    # Assert（検証）
    self.assertEqual(result, expected)
```

### パターン2: エラーハンドリングテスト

```python
def test_function_error_handling():
    """エラーハンドリングのテスト"""
    # 不正な入力でエラーが発生することを確認
    with self.assertRaises(ValueError):
        target_function(None)
    
    with self.assertRaises(TypeError):
        target_function(123)  # 文字列を期待している場合
```

### パターン3: モックを使用したテスト

```python
@patch('module.external_api')
def test_with_mock(self, mock_api):
    """外部APIをモック化したテスト"""
    # モックの設定
    mock_api.call.return_value = {'status': 'success'}
    
    # テスト実行
    result = function_using_api()
    
    # モックが呼ばれたことを確認
    mock_api.call.assert_called_once_with('expected_param')
    self.assertEqual(result, 'success')
```

### パターン4: パラメトリックテスト

```python
def test_parametric():
    """複数のケースをまとめてテスト"""
    test_cases = [
        ("input1", "expected1"),
        ("input2", "expected2"),
        ("", None),  # 空文字列
        (None, None),  # None
    ]
    
    for input_val, expected in test_cases:
        with self.subTest(input=input_val):
            result = target_function(input_val)
            self.assertEqual(result, expected)
```

## JavaScript検証パターン

### パターン1: 構文チェック

```python
def validate_javascript_syntax(file_path):
    """JavaScript構文の検証"""
    content = Path(file_path).read_text()
    
    checks = [
        ('console.log', 'console.logが残っています'),
        ('var ', 'varの代わりにletまたはconstを使用'),
        ('debugger', 'debugger文が残っています'),
        ('TODO', '未完了のTODOがあります'),
    ]
    
    warnings = []
    for pattern, message in checks:
        if pattern in content:
            warnings.append(message)
    
    return {'valid': len(warnings) == 0, 'warnings': warnings}
```

### パターン2: import/export整合性チェック

```python
def check_imports_exports(file_path):
    """import/exportの整合性を確認"""
    content = Path(file_path).read_text()
    
    # exportされている関数を抽出
    exports = re.findall(r'export\s+(?:function|const|let)\s+(\w+)', content)
    
    # importされている関数を抽出
    imports = re.findall(r'import\s+{([^}]+)}\s+from', content)
    
    # 未使用のexportを検出
    unused_exports = []
    for export in exports:
        # 他のファイルで使用されているか確認
        # （簡略化された例）
        pass
    
    return {
        'exports': exports,
        'imports': imports,
        'unused': unused_exports
    }
```

## 統合テストシナリオ

### シナリオテンプレート

```python
def create_user_flow_scenario():
    """ユーザーフロー全体のテスト"""
    scenario = ScenarioTest("完全なユーザーフロー")
    
    # 1. アプリケーション起動
    scenario.add_step('initialize_app', {})
    
    # 2. マップ読み込み
    scenario.add_step('load_map', {
        'file': 'sample_map.pgm',
        'resolution': 0.05
    })
    
    # 3. メタデータ追加
    scenario.add_step('add_metadata', {
        'origin': [0, 0, 0],
        'resolution': 0.05
    })
    
    # 4. オブジェクト配置
    scenario.add_step('place_object', {
        'type': 'shelf',
        'position': {'x': 100, 'y': 200},
        'height': 1.8
    })
    
    # 5. 保存
    scenario.add_step('save_project', {
        'name': 'test_project'
    })
    
    # 検証
    scenario.assert_result(
        lambda: check_project_saved('test_project'),
        "プロジェクトが正しく保存されている"
    )
    
    return scenario
```

## コマンドリファレンス

### テスト実行コマンド

```bash
# 全テスト実行
python tests/test_runner.py

# 特定モジュールのテスト
python tests/test_runner.py verify layerManager

# 監視モード（ファイル変更を検出して自動実行）
python tests/test_runner.py watch

# テスト生成
python tests/claude_test_generator.py function <関数名> <モジュール>
python tests/claude_test_generator.py class <クラス名> <モジュール>
python tests/claude_test_generator.py javascript <モジュール名>

# カバレッジ更新
python tests/claude_test_generator.py update <モジュール> <関数1,関数2>

# 統合テスト実行
python tests/integration/test_scenarios.py
python tests/integration/test_scenarios.py "基本的なマップ読み込みフロー"
```

## チェックリスト

### 新機能実装時のチェックリスト

- [ ] テストファイルが存在する
- [ ] 基本動作のテストがある
- [ ] エッジケースのテストがある
- [ ] エラーハンドリングのテストがある
- [ ] テストが全て成功する
- [ ] カバレッジが80%以上
- [ ] docstringが記載されている
- [ ] 統合テストに影響がないか確認

### バグ修正時のチェックリスト

- [ ] バグを再現するテストを作成
- [ ] テストが失敗することを確認
- [ ] バグを修正
- [ ] テストが成功することを確認
- [ ] 関連するテストも成功することを確認
- [ ] 回帰テストを実行

### PR作成前のチェックリスト

- [ ] 全テストが成功
- [ ] カバレッジが基準値以上
- [ ] 統合テストが成功
- [ ] JavaScriptの検証が成功
- [ ] 新しいテストが追加されている
- [ ] テストのドキュメントが更新されている

## トラブルシューティング

### よくある問題と解決方法

#### 1. ImportError: No module named 'xxx'

```python
# 解決: sys.pathにプロジェクトルートを追加
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
```

#### 2. テストが相互に影響する

```python
# 解決: setUp()とtearDown()で状態をリセット
def setUp(self):
    self.test_data = []
    
def tearDown(self):
    self.test_data.clear()
```

#### 3. モックが正しく動作しない

```python
# 解決: patchの適用場所を確認
@patch('module.function')  # 使用される場所でパッチ
def test_with_mock(self, mock_func):
    mock_func.return_value = 'mocked'
```

## ベストプラクティス

1. **テストは独立して実行可能に**
   - 他のテストに依存しない
   - 順序に依存しない

2. **明確な命名**
   - `test_<機能>_<条件>_<期待結果>`

3. **適切な粒度**
   - 1テスト1検証
   - 複雑なテストは分割

4. **メンテナンス性**
   - DRY原則を適用
   - 共通処理はヘルパー関数に

5. **ドキュメント**
   - docstringで目的を明確に
   - 複雑なロジックにはコメント

## 最後に

このシステムを使用することで、Claude Codeは自律的に：

1. コード変更時にテストを実行
2. 新機能のテストを生成
3. カバレッジを監視
4. 品質を保証

これにより、人間の手動テストの負担を大幅に削減し、開発効率を向上させます。

常にこの指示書に従ってテストを実装・実行してください。
