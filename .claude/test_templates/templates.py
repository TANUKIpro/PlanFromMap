#!/usr/bin/env python3
"""
テストテンプレート集
Claudeが新しいテストを作成する際に使用するテンプレート
"""

# =====================================
# 基本的な関数テストテンプレート
# =====================================

BASIC_FUNCTION_TEST = '''#!/usr/bin/env python3
"""
{module_name} モジュールの {function_name} 関数のテスト
"""

import unittest
from unittest.mock import Mock, patch
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from {module_path} import {function_name}


class Test{function_name_camel}(unittest.TestCase):
    """
    {function_name} 関数のテストクラス
    """
    
    def setUp(self):
        """テストの初期化"""
        self.test_data = {{
            'valid_input': '{sample_input}',
            'expected_output': '{expected_output}'
        }}
        
    def test_{function_name}_basic(self):
        """基本動作のテスト"""
        # Arrange
        input_data = self.test_data['valid_input']
        expected = self.test_data['expected_output']
        
        # Act
        result = {function_name}(input_data)
        
        # Assert
        self.assertEqual(result, expected)
        
    def test_{function_name}_empty_input(self):
        """空の入力のテスト"""
        # Act & Assert
        result = {function_name}('')
        self.assertIsNone(result)
        
    def test_{function_name}_none_input(self):
        """None入力のテスト"""
        # Act & Assert
        result = {function_name}(None)
        self.assertIsNone(result)
        
    def test_{function_name}_error_handling(self):
        """エラーハンドリングのテスト"""
        # 不正な型でTypeErrorが発生することを確認
        with self.assertRaises(TypeError):
            {function_name}(123)  # 数値を渡す
            
        # 不正な値でValueErrorが発生することを確認
        with self.assertRaises(ValueError):
            {function_name}('invalid_value')


if __name__ == "__main__":
    unittest.main()
'''

# =====================================
# クラステストテンプレート
# =====================================

CLASS_TEST_TEMPLATE = '''#!/usr/bin/env python3
"""
{class_name} クラスのテスト
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from {module_path} import {class_name}


class Test{class_name}(unittest.TestCase):
    """
    {class_name} クラスのテストクラス
    """
    
    def setUp(self):
        """各テストの前に実行される初期化"""
        self.instance = {class_name}()
        self.test_data = {{
            'sample_attribute': 'test_value'
        }}
        
    def tearDown(self):
        """各テストの後に実行されるクリーンアップ"""
        self.instance = None
        
    def test_initialization(self):
        """初期化のテスト"""
        instance = {class_name}()
        self.assertIsNotNone(instance)
        # 必要な属性が存在することを確認
        # self.assertTrue(hasattr(instance, 'expected_attribute'))
        
    def test_method_basic(self):
        """基本的なメソッドのテスト"""
        # TODO: 具体的なメソッドテストを実装
        pass
        
    def test_property_getter_setter(self):
        """プロパティのgetter/setterテスト"""
        # Setter
        self.instance.property = 'new_value'
        
        # Getter
        value = self.instance.property
        
        # Assert
        self.assertEqual(value, 'new_value')
        
    def test_method_with_mock(self):
        """モックを使用したメソッドテスト"""
        with patch.object(self.instance, 'external_method') as mock_method:
            mock_method.return_value = 'mocked_result'
            
            result = self.instance.method_using_external()
            
            mock_method.assert_called_once()
            self.assertEqual(result, 'expected_result_with_mock')
            
    def test_error_conditions(self):
        """エラー条件のテスト"""
        # 不正な操作でエラーが発生することを確認
        with self.assertRaises(ValueError):
            self.instance.method_with_validation('invalid_input')


if __name__ == "__main__":
    unittest.main()
'''

# =====================================
# 統合テストテンプレート
# =====================================

INTEGRATION_TEST_TEMPLATE = '''#!/usr/bin/env python3
"""
{feature_name} の統合テスト
複数のモジュールが連携して動作することを確認
"""

import unittest
from unittest.mock import Mock, patch
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from tests.integration.test_scenarios import ScenarioTest


class Test{feature_name_camel}Integration(unittest.TestCase):
    """
    {feature_name} の統合テストクラス
    """
    
    def setUp(self):
        """テスト環境の初期化"""
        self.scenario = ScenarioTest("{feature_name} 統合テスト")
        
    def test_complete_workflow(self):
        """完全なワークフローのテスト"""
        # Step 1: 初期化
        self.scenario.add_step('initialize', {{}})
        
        # Step 2: データ読み込み
        self.scenario.add_step('load_data', {{
            'file': 'test_data.json'
        }})
        
        # Step 3: 処理実行
        self.scenario.add_step('process', {{
            'mode': 'standard'
        }})
        
        # Step 4: 結果保存
        self.scenario.add_step('save_results', {{
            'output': 'results.json'
        }})
        
        # 検証
        self.scenario.assert_result(
            lambda: self.scenario.results.get('success', False),
            "ワークフローが正常に完了"
        )
        
        # 実行
        success = self.scenario.execute()
        self.assertTrue(success)
        
    def test_error_recovery(self):
        """エラーからの回復テスト"""
        # エラーが発生する条件を設定
        self.scenario.add_step('initialize', {{}})
        self.scenario.add_step('load_data', {{
            'file': 'nonexistent.json'  # 存在しないファイル
        }})
        
        # エラーハンドリングが動作することを確認
        self.scenario.add_step('handle_error', {{}})
        
        # フォールバック処理
        self.scenario.add_step('load_default_data', {{}})
        
        # 検証
        self.scenario.assert_result(
            lambda: self.scenario.results.get('recovered', False),
            "エラーから回復できる"
        )
        
        success = self.scenario.execute()
        self.assertTrue(success)


if __name__ == "__main__":
    unittest.main()
'''

# =====================================
# JavaScriptモジュール検証テンプレート
# =====================================

JAVASCRIPT_VALIDATION_TEMPLATE = '''#!/usr/bin/env python3
"""
{module_name}.js の検証テスト
JavaScriptコードの品質を検証
"""

import re
import json
from pathlib import Path
from typing import Dict, List, Any


def validate_{module_name}_syntax():
    """
    {module_name}.js の構文検証
    """
    js_file = Path("apps/frontend/static/js/modules/{module_name}.js")
    
    if not js_file.exists():
        return {{
            "valid": False,
            "error": "ファイルが見つかりません"
        }}
        
    content = js_file.read_text(encoding='utf-8')
    
    errors = []
    warnings = []
    
    # 構文チェックルール
    rules = [
        # エラー
        (r'\\bconsole\\.log\\b', 'error', 'console.logが残っています'),
        (r'\\bdebugger\\b', 'error', 'debugger文が残っています'),
        (r'\\balert\\(', 'error', 'alert()が使用されています'),
        
        # 警告
        (r'\\bvar\\s+', 'warning', 'varの代わりにletまたはconstを使用してください'),
        (r'==(?!=)', 'warning', '厳密等価演算子(===)を使用してください'),
        (r'TODO|FIXME|XXX', 'warning', '未完了のコメントがあります'),
    ]
    
    for pattern, level, message in rules:
        if re.search(pattern, content):
            if level == 'error':
                errors.append(message)
            else:
                warnings.append(message)
                
    return {{
        "valid": len(errors) == 0,
        "errors": errors,
        "warnings": warnings
    }}


def validate_{module_name}_structure():
    """
    {module_name}.js の構造検証
    """
    js_file = Path("apps/frontend/static/js/modules/{module_name}.js")
    content = js_file.read_text(encoding='utf-8')
    
    # export/import の抽出
    exports = re.findall(r'export\\s+(?:function|const|let|class)\\s+(\\w+)', content)
    imports = re.findall(r'import\\s+{{([^}}]+)}}\\s+from', content)
    
    # 関数の複雑度チェック
    functions = re.findall(r'function\\s+(\\w+)\\s*\\([^)]*\\)\\s*{{', content)
    
    complex_functions = []
    for func_name in functions:
        # 簡易的な複雑度計算（if/for/while/switchの数）
        func_pattern = rf'function\\s+{{func_name}}[^{{]*{{([^}}]*(?:{{[^}}]*}}[^}}]*)*)}}'
        func_match = re.search(func_pattern, content)
        
        if func_match:
            func_body = func_match.group(1)
            complexity = (
                func_body.count('if') +
                func_body.count('for') +
                func_body.count('while') +
                func_body.count('switch')
            )
            
            if complexity > 5:
                complex_functions.append({{
                    'name': func_name,
                    'complexity': complexity
                }})
                
    return {{
        "exports": exports,
        "imports": imports,
        "complex_functions": complex_functions,
        "total_functions": len(functions)
    }}


def validate_{module_name}_dependencies():
    """
    {module_name}.js の依存関係検証
    """
    js_file = Path("apps/frontend/static/js/modules/{module_name}.js")
    content = js_file.read_text(encoding='utf-8')
    
    # import文を解析
    import_statements = re.findall(r'import.*from\\s+[\\'"]([^\\'"]+ )[\\'"]', content)
    
    missing_deps = []
    for dep_path in import_statements:
        # 相対パスを解決
        if dep_path.startswith('./'):
            dep_file = js_file.parent / dep_path[2:]
            if not dep_file.exists() and not (dep_file.with_suffix('.js')).exists():
                missing_deps.append(dep_path)
                
    return {{
        "dependencies": import_statements,
        "missing": missing_deps,
        "valid": len(missing_deps) == 0
    }}


def run_all_validations():
    """
    すべての検証を実行
    """
    results = {{}}
    
    # 構文検証
    syntax_result = validate_{module_name}_syntax()
    results['syntax'] = syntax_result
    
    # 構造検証
    structure_result = validate_{module_name}_structure()
    results['structure'] = structure_result
    
    # 依存関係検証
    deps_result = validate_{module_name}_dependencies()
    results['dependencies'] = deps_result
    
    # 総合評価
    results['overall'] = {{
        'valid': (
            syntax_result['valid'] and
            deps_result['valid'] and
            len(structure_result.get('complex_functions', [])) == 0
        ),
        'score': calculate_quality_score(results)
    }}
    
    return results


def calculate_quality_score(results):
    """
    品質スコアを計算（0-100）
    """
    score = 100
    
    # エラーごとに減点
    score -= len(results['syntax'].get('errors', [])) * 20
    score -= len(results['syntax'].get('warnings', [])) * 5
    score -= len(results['dependencies'].get('missing', [])) * 15
    score -= len(results['structure'].get('complex_functions', [])) * 10
    
    return max(0, score)


if __name__ == "__main__":
    results = run_all_validations()
    
    print(f"📊 {module_name}.js 検証結果")
    print("=" * 40)
    
    # 構文チェック結果
    if results['syntax']['valid']:
        print("✅ 構文チェック: 合格")
    else:
        print("❌ 構文チェック: 不合格")
        for error in results['syntax']['errors']:
            print(f"  - {{error}}")
            
    # 警告
    if results['syntax'].get('warnings'):
        print("⚠️ 警告:")
        for warning in results['syntax']['warnings']:
            print(f"  - {{warning}}")
            
    # 品質スコア
    print(f"\\n品質スコア: {{results['overall']['score']}}/100")
'''

# テンプレートを取得する関数
def get_template(template_type: str, **kwargs) -> str:
    """
    指定されたタイプのテンプレートを取得
    
    Args:
        template_type: テンプレートの種類
        **kwargs: テンプレートに埋め込む値
        
    Returns:
        フォーマットされたテンプレート文字列
    """
    templates = {
        'function': BASIC_FUNCTION_TEST,
        'class': CLASS_TEST_TEMPLATE,
        'integration': INTEGRATION_TEST_TEMPLATE,
        'javascript': JAVASCRIPT_VALIDATION_TEMPLATE
    }
    
    template = templates.get(template_type, BASIC_FUNCTION_TEST)
    
    # デフォルト値を設定
    defaults = {
        'module_name': 'module',
        'function_name': 'function',
        'class_name': 'TestClass',
        'feature_name': 'feature',
        'module_path': 'module.path',
        'sample_input': 'test_input',
        'expected_output': 'expected_output',
        'function_name_camel': 'Function',
        'feature_name_camel': 'Feature'
    }
    
    # kwargs とデフォルト値をマージ
    params = {**defaults, **kwargs}
    
    # テンプレートをフォーマット
    return template.format(**params)
