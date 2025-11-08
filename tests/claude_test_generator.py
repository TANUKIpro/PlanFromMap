#!/usr/bin/env python3
"""
Claude Test Generator
Claudeが自動的にテストコードを生成・更新するためのヘルパーモジュール
"""

import ast
import inspect
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime


class ClaudeTestGenerator:
    """
    Claudeが自動的にテストを生成するためのヘルパークラス
    新機能追加時やバグ修正時に、適切なテストコードを生成
    """
    
    def __init__(self, project_root: Path = Path.cwd()):
        self.project_root = project_root
        self.tests_dir = project_root / "tests"
        self.modules_dir = self.tests_dir / "modules"
        
        # テストテンプレート
        self.templates = self._load_templates()
        
    def _load_templates(self) -> Dict[str, str]:
        """テストテンプレートを読み込み"""
        return {
            'basic': self._get_basic_template(),
            'edge_case': self._get_edge_case_template(),
            'error_handling': self._get_error_handling_template(),
            'integration': self._get_integration_template(),
            'javascript': self._get_javascript_template()
        }
        
    def generate_test_for_function(
        self, 
        func_name: str, 
        module_path: str,
        func_signature: Optional[str] = None
    ) -> str:
        """
        関数のテストを自動生成
        
        Args:
            func_name: テスト対象の関数名
            module_path: モジュールのパス
            func_signature: 関数シグネチャ（オプション）
            
        Returns:
            生成されたテストコード
        """
        
        # 関数の情報を取得
        func_info = self._analyze_function(func_name, module_path, func_signature)
        
        # テストコードを生成
        test_code = f'''#!/usr/bin/env python3
"""
{func_name} のテストモジュール
自動生成日時: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
Claude Test Generatorにより自動生成
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# プロジェクトルートをパスに追加
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from {module_path} import {func_name}


class Test{self._to_camel_case(func_name)}(unittest.TestCase):
    """
    {func_name} 関数のテストクラス
    """
    
    def setUp(self):
        """各テストの前に実行される初期化処理"""
        self.test_data = {self._generate_test_data(func_info)}
        
    def tearDown(self):
        """各テストの後に実行されるクリーンアップ処理"""
        pass
        
    def test_{func_name}_basic(self):
        """
        基本動作のテスト
        正常な入力で期待通りの出力が得られることを確認
        """
        # Arrange
        {self._generate_arrange_code(func_info)}
        
        # Act
        result = {func_name}({self._generate_params(func_info)})
        
        # Assert
        {self._generate_assertions(func_info, 'basic')}
        
    def test_{func_name}_with_empty_input(self):
        """
        空の入力でのテスト
        エッジケース: 空の値や None を処理できることを確認
        """
        # 空の文字列
        {self._generate_empty_test(func_info, 'string')}
        
        # None
        {self._generate_empty_test(func_info, 'none')}
        
        # 空のリスト/辞書
        {self._generate_empty_test(func_info, 'collection')}
        
    def test_{func_name}_edge_cases(self):
        """
        エッジケースのテスト
        境界値や特殊な入力での動作を確認
        """
        edge_cases = [
            {self._generate_edge_cases(func_info)}
        ]
        
        for case_name, input_val, expected in edge_cases:
            with self.subTest(case=case_name):
                result = {func_name}(input_val)
                self.assertEqual(result, expected, f"{{case_name}} の処理に失敗")
                
    def test_{func_name}_error_handling(self):
        """
        エラーハンドリングのテスト
        異常な入力でも適切にエラー処理されることを確認
        """
        # 型エラーのテスト
        with self.assertRaises(TypeError):
            {func_name}({self._generate_invalid_type_params(func_info)})
            
        # 値エラーのテスト
        with self.assertRaises(ValueError):
            {func_name}({self._generate_invalid_value_params(func_info)})
            
    def test_{func_name}_performance(self):
        """
        パフォーマンステスト
        処理時間が許容範囲内であることを確認
        """
        import time
        
        start_time = time.time()
        
        # 1000回実行
        for _ in range(1000):
            {func_name}({self._generate_params(func_info)})
            
        elapsed = time.time() - start_time
        
        # 1秒以内に完了すること
        self.assertLess(elapsed, 1.0, f"処理時間が長すぎます: {{elapsed:.2f}}秒")
        
    {self._generate_additional_tests(func_info)}


if __name__ == "__main__":
    unittest.main()
'''
        return test_code
        
    def generate_test_for_class(self, class_name: str, module_path: str) -> str:
        """クラスのテストを自動生成"""
        
        return f'''#!/usr/bin/env python3
"""
{class_name} クラスのテストモジュール
自動生成日時: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
from {module_path} import {class_name}


class Test{class_name}(unittest.TestCase):
    """
    {class_name} クラスのテスト
    """
    
    def setUp(self):
        """テストの初期化"""
        self.instance = {class_name}()
        
    def test_initialization(self):
        """初期化のテスト"""
        instance = {class_name}()
        self.assertIsNotNone(instance)
        
    def test_methods(self):
        """各メソッドのテスト"""
        # TODO: Claudeが具体的なメソッドテストを追加
        pass
        
    # TODO: Claudeが追加のテストを実装


if __name__ == "__main__":
    unittest.main()
'''

    def generate_javascript_test(self, module_name: str) -> str:
        """JavaScript用のテスト検証コードを生成（Pythonベース）"""
        
        return f'''#!/usr/bin/env python3
"""
{module_name}.js の検証モジュール
JavaScriptコードの品質をPythonから検証
"""

import re
from pathlib import Path


def validate_{module_name}():
    """
    {module_name}.js の検証
    """
    js_file = Path("apps/frontend/static/js/modules/{module_name}.js")
    
    if not js_file.exists():
        return {{"valid": False, "error": "ファイルが見つかりません"}}
        
    content = js_file.read_text(encoding='utf-8')
    
    errors = []
    warnings = []
    
    # 構文チェック
    if "console.log" in content:
        warnings.append("console.logが残っています")
        
    # export/importの整合性チェック
    exports = re.findall(r'export\s+(?:function|const|let|class)\s+(\w+)', content)
    
    # 未使用変数のチェック
    variables = re.findall(r'(?:const|let|var)\s+(\w+)', content)
    for var in variables:
        if content.count(var) == 1:
            warnings.append(f"未使用の変数: {{var}}")
            
    # 関数の複雑度チェック
    functions = re.findall(r'function\s+(\w+)', content)
    for func in functions:
        # 簡易的な複雑度計算
        func_content = re.search(
            rf'function\s+{{func}}\s*\([^)]*\)\s*{{([^{{}}]*(?:{{[^{{}}]*}}[^{{}}]*)*)}}',
            content
        )
        if func_content:
            complexity = func_content.group(1).count('if') + \
                        func_content.group(1).count('for') + \
                        func_content.group(1).count('while')
            if complexity > 5:
                warnings.append(f"関数 {{func}} の複雑度が高い: {{complexity}}")
                
    return {{
        "valid": len(errors) == 0,
        "errors": errors,
        "warnings": warnings,
        "exports": exports
    }}


if __name__ == "__main__":
    result = validate_{module_name}()
    
    if result["valid"]:
        print("✅ 検証成功")
    else:
        print("❌ 検証失敗")
        
    if result.get("warnings"):
        print("⚠️ 警告:")
        for warning in result["warnings"]:
            print(f"  - {{warning}}")
'''

    def update_test_coverage(self, module_name: str, new_functions: List[str]) -> bool:
        """新機能追加時にテストカバレッジを更新"""
        
        test_file_path = self.modules_dir / f"test_{module_name}.py"
        
        if not test_file_path.exists():
            # 新規テストファイルを作成
            print(f"📝 新規テストファイル作成: test_{module_name}.py")
            test_content = self._create_new_test_file(module_name, new_functions)
            test_file_path.write_text(test_content)
            return True
            
        # 既存ファイルに追加
        existing_content = test_file_path.read_text()
        
        for func in new_functions:
            if f"test_{func}" not in existing_content:
                print(f"  📝 テスト追加: test_{func}")
                # テストを追加
                new_test = self._generate_single_test(func)
                existing_content += f"\n{new_test}\n"
                
        test_file_path.write_text(existing_content)
        return True
        
    def _analyze_function(
        self, 
        func_name: str, 
        module_path: str,
        func_signature: Optional[str] = None
    ) -> Dict[str, Any]:
        """関数の情報を分析"""
        
        # 関数名から推測される情報
        info = {
            'name': func_name,
            'module': module_path,
            'is_async': 'async' in str(func_signature) if func_signature else False,
            'returns_value': not func_name.startswith('set_'),
            'has_side_effects': func_name.startswith(('set_', 'update_', 'delete_', 'save_')),
            'is_getter': func_name.startswith('get_'),
            'is_boolean': func_name.startswith(('is_', 'has_', 'can_'))
        }
        
        # パラメータを推測
        if '_id' in func_name:
            info['params'] = ['id']
        elif 'create' in func_name or 'add' in func_name:
            info['params'] = ['data']
        else:
            info['params'] = []
            
        return info
        
    def _generate_test_data(self, func_info: Dict) -> str:
        """テストデータを生成"""
        
        if func_info['is_getter']:
            return '''
            'test_id': 'test_123',
            'test_name': 'テストアイテム',
            'test_data': {'key': 'value'}
        '''
        elif func_info['has_side_effects']:
            return '''
            'input_data': {
                'name': 'テスト',
                'value': 123,
                'enabled': True
            }
        '''
        else:
            return '''
            'sample_input': 'test_value',
            'expected_output': 'expected_result'
        '''
        
    def _generate_arrange_code(self, func_info: Dict) -> str:
        """Arrangeセクションのコードを生成"""
        
        if func_info['params']:
            return "test_input = self.test_data['sample_input']"
        else:
            return "# 入力パラメータなし"
            
    def _generate_params(self, func_info: Dict) -> str:
        """関数呼び出しのパラメータを生成"""
        
        if not func_info['params']:
            return ""
            
        if 'id' in func_info['params']:
            return "self.test_data['test_id']"
        elif 'data' in func_info['params']:
            return "self.test_data['input_data']"
        else:
            return "test_input"
            
    def _generate_assertions(self, func_info: Dict, test_type: str) -> str:
        """アサーションコードを生成"""
        
        if func_info['is_boolean']:
            return "self.assertIsInstance(result, bool)"
        elif func_info['returns_value']:
            return "self.assertIsNotNone(result)\n        self.assertEqual(result, self.test_data['expected_output'])"
        else:
            return "# 戻り値なし - 副作用を確認"
            
    def _generate_empty_test(self, func_info: Dict, empty_type: str) -> str:
        """空入力テストを生成"""
        
        if empty_type == 'string':
            return f"result = {func_info['name']}('')\n        self.assertIsNone(result)"
        elif empty_type == 'none':
            return f"result = {func_info['name']}(None)\n        self.assertIsNone(result)"
        else:
            return f"result = {func_info['name']}([])\n        self.assertEqual(result, [])"
            
    def _generate_edge_cases(self, func_info: Dict) -> str:
        """エッジケースを生成"""
        
        return '''
            ('最大値', 999999, None),
            ('最小値', -999999, None),
            ('ゼロ', 0, 0),
            ('特殊文字', '!@#$%', None)
        '''
        
    def _generate_invalid_type_params(self, func_info: Dict) -> str:
        """無効な型のパラメータを生成"""
        return "123 if 'string' in str(type(self.test_data['sample_input'])) else 'invalid'"
        
    def _generate_invalid_value_params(self, func_info: Dict) -> str:
        """無効な値のパラメータを生成"""
        return "-1  # 負の値は無効"
        
    def _generate_additional_tests(self, func_info: Dict) -> str:
        """追加のテストメソッドを生成"""
        
        additional = ""
        
        if func_info['is_async']:
            additional += '''
    async def test_{name}_async(self):
        """非同期処理のテスト"""
        import asyncio
        result = await {name}(self.test_data['sample_input'])
        self.assertIsNotNone(result)
'''.format(name=func_info['name'])
            
        if func_info['has_side_effects']:
            additional += '''
    @patch('module.database')
    def test_{name}_with_mock(self, mock_db):
        """モックを使用したテスト"""
        mock_db.save.return_value = True
        result = {name}(self.test_data['input_data'])
        mock_db.save.assert_called_once()
'''.format(name=func_info['name'])
            
        return additional
        
    def _to_camel_case(self, snake_str: str) -> str:
        """スネークケースをキャメルケースに変換"""
        components = snake_str.split('_')
        return ''.join(x.title() for x in components)
        
    def _create_new_test_file(self, module_name: str, functions: List[str]) -> str:
        """新規テストファイルを作成"""
        
        tests = []
        for func in functions:
            tests.append(self._generate_single_test(func))
            
        return f'''#!/usr/bin/env python3
"""
{module_name} モジュールのテスト
自動生成日時: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
"""

import unittest
from {module_name} import {', '.join(functions)}


class Test{self._to_camel_case(module_name)}(unittest.TestCase):
    """
    {module_name} のテストクラス
    """
    
    def setUp(self):
        """初期化"""
        pass
        
{"".join(tests)}


if __name__ == "__main__":
    unittest.main()
'''

    def _generate_single_test(self, func_name: str) -> str:
        """単一のテストメソッドを生成"""
        
        return f'''
    def test_{func_name}(self):
        """
        {func_name} のテスト
        TODO: Claudeが実装を追加
        """
        # Arrange
        
        # Act
        
        # Assert
        self.assertTrue(True)  # プレースホルダー
'''


def main():
    """メインエントリーポイント"""
    import sys
    
    generator = ClaudeTestGenerator()
    
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python claude_test_generator.py function <関数名> <モジュール>")
        print("  python claude_test_generator.py class <クラス名> <モジュール>")
        print("  python claude_test_generator.py javascript <モジュール名>")
        print("  python claude_test_generator.py update <モジュール名> <関数1,関数2,...>")
        sys.exit(1)
        
    command = sys.argv[1]
    
    if command == "function" and len(sys.argv) >= 4:
        func_name = sys.argv[2]
        module = sys.argv[3]
        code = generator.generate_test_for_function(func_name, module)
        print(code)
        
    elif command == "class" and len(sys.argv) >= 4:
        class_name = sys.argv[2]
        module = sys.argv[3]
        code = generator.generate_test_for_class(class_name, module)
        print(code)
        
    elif command == "javascript" and len(sys.argv) >= 3:
        module_name = sys.argv[2]
        code = generator.generate_javascript_test(module_name)
        print(code)
        
    elif command == "update" and len(sys.argv) >= 4:
        module_name = sys.argv[2]
        functions = sys.argv[3].split(',')
        generator.update_test_coverage(module_name, functions)
        print(f"✅ {module_name} のテストカバレッジを更新しました")
        
    else:
        print("❌ 引数が不正です")
        sys.exit(1)


if __name__ == "__main__":
    main()
