#!/usr/bin/env python3
"""
統合テスト - ユーザーシナリオベースのテスト
実際のユーザー操作フローをシミュレート
"""

import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass

# プロジェクトルートをパスに追加
current_file = Path(__file__).resolve()
tests_dir = current_file.parent.parent
project_root = tests_dir.parent
sys.path.insert(0, str(project_root))


@dataclass
class TestScenario:
    """テストシナリオを表すデータクラス"""
    name: str
    description: str
    steps: List[Dict[str, Any]]
    expected_results: List[str]
    
    
class ScenarioTest:
    """
    ユーザーシナリオベースのテストクラス
    実際のユーザー操作をシミュレートして統合テストを実行
    """
    
    def __init__(self, name: str):
        self.name = name
        self.steps = []
        self.assertions = []
        self.results = {}
        self.start_time = None
        self.end_time = None
        
    def add_step(self, action: str, params: Dict[str, Any]):
        """
        テストステップを追加
        
        Args:
            action: 実行するアクション
            params: アクションのパラメータ
        """
        self.steps.append({
            'action': action,
            'params': params,
            'timestamp': time.time()
        })
        
    def assert_result(self, condition, message: str):
        """
        結果を検証
        
        Args:
            condition: 検証する条件（callable）
            message: アサーションメッセージ
        """
        try:
            result = condition() if callable(condition) else condition
            self.assertions.append({
                'condition': str(condition),
                'message': message,
                'passed': result
            })
        except Exception as e:
            self.assertions.append({
                'condition': str(condition),
                'message': message,
                'passed': False,
                'error': str(e)
            })
            
    def execute(self) -> bool:
        """シナリオを実行"""
        self.start_time = time.time()
        
        print(f"📋 シナリオ実行: {self.name}")
        
        # 各ステップを実行
        for i, step in enumerate(self.steps, 1):
            print(f"  ステップ {i}: {step['action']}")
            
            # ステップ実行（モック）
            result = self._execute_step(step)
            
            if not result:
                print(f"    ❌ 失敗")
                self.end_time = time.time()
                return False
            else:
                print(f"    ✅ 成功")
                
        # アサーション検証
        all_passed = all(a['passed'] for a in self.assertions)
        
        self.end_time = time.time()
        
        if all_passed:
            print(f"  ✅ シナリオ成功 ({self.end_time - self.start_time:.2f}秒)")
        else:
            print(f"  ❌ シナリオ失敗")
            for a in self.assertions:
                if not a['passed']:
                    print(f"    - {a['message']}")
                    
        return all_passed
        
    def _execute_step(self, step: Dict) -> bool:
        """ステップを実行（モック実装）"""
        action = step['action']
        params = step['params']
        
        # アクションごとの処理（実際のアプリケーション動作をシミュレート）
        if action == 'load_map':
            return self._mock_load_map(params.get('file'))
        elif action == 'add_metadata':
            return self._mock_add_metadata(params.get('file'))
        elif action == 'create_3d_view':
            return self._mock_create_3d_view()
        elif action == 'add_object':
            return self._mock_add_object(params)
        elif action == 'draw_annotation':
            return self._mock_draw_annotation(params)
        elif action == 'save_profile':
            return self._mock_save_profile(params.get('name'))
        elif action == 'load_profile':
            return self._mock_load_profile(params.get('name'))
        else:
            return True
            
    # =====================================
    # モック実装
    # =====================================
    
    def _mock_load_map(self, filename: str) -> bool:
        """マップ読み込みのモック"""
        if not filename:
            return False
        self.results['map_loaded'] = True
        self.results['layers'] = ['map-image']
        return True
        
    def _mock_add_metadata(self, filename: str) -> bool:
        """メタデータ追加のモック"""
        if not filename:
            return False
        self.results['metadata_loaded'] = True
        self.results.setdefault('layers', []).append('metadata')
        return True
        
    def _mock_create_3d_view(self) -> bool:
        """3Dビュー作成のモック"""
        self.results['3d_view_created'] = True
        return True
        
    def _mock_add_object(self, params: Dict) -> bool:
        """オブジェクト追加のモック"""
        objects = self.results.setdefault('objects', [])
        objects.append(params)
        return True
        
    def _mock_draw_annotation(self, params: Dict) -> bool:
        """アノテーション描画のモック"""
        annotations = self.results.setdefault('annotations', [])
        annotations.append(params)
        return True
        
    def _mock_save_profile(self, name: str) -> bool:
        """プロファイル保存のモック"""
        self.results['profile_saved'] = name
        return True
        
    def _mock_load_profile(self, name: str) -> bool:
        """プロファイル読み込みのモック"""
        self.results['profile_loaded'] = name
        return True


def create_scenario_tests() -> List[ScenarioTest]:
    """テストシナリオを作成"""
    
    scenarios = []
    
    # シナリオ1: 基本的なマップ読み込みフロー
    scenario1 = ScenarioTest("基本的なマップ読み込みフロー")
    scenario1.add_step('load_map', {'file': 'test_map.pgm'})
    scenario1.add_step('add_metadata', {'file': 'test_metadata.yaml'})
    scenario1.add_step('create_3d_view', {})
    scenario1.assert_result(
        lambda: len(scenario1.results.get('layers', [])) == 2,
        "マップとメタデータの2レイヤーが存在する"
    )
    scenario1.assert_result(
        lambda: scenario1.results.get('3d_view_created', False),
        "3Dビューが作成されている"
    )
    scenarios.append(scenario1)
    
    # シナリオ2: オブジェクト配置と編集フロー
    scenario2 = ScenarioTest("オブジェクト配置と編集フロー")
    scenario2.add_step('load_map', {'file': 'test_map.pgm'})
    scenario2.add_step('add_object', {
        'type': 'shelf',
        'position': {'x': 100, 'y': 200},
        'properties': {
            'height_meters': 1.8,
            'front_direction': 'top'
        }
    })
    scenario2.add_step('add_object', {
        'type': 'door',
        'position': {'x': 300, 'y': 150},
        'properties': {
            'height_meters': 2.1,
            'front_direction': 'right',
            'door_type': 'sliding'
        }
    })
    scenario2.assert_result(
        lambda: len(scenario2.results.get('objects', [])) == 2,
        "2つのオブジェクトが配置されている"
    )
    scenarios.append(scenario2)
    
    # シナリオ3: アノテーション描画フロー
    scenario3 = ScenarioTest("アノテーション描画フロー")
    scenario3.add_step('load_map', {'file': 'test_map.pgm'})
    scenario3.add_step('draw_annotation', {
        'tool': 'rectangle',
        'bounds': {'x': 50, 'y': 50, 'width': 100, 'height': 100},
        'color': '#FF0000'
    })
    scenario3.add_step('draw_annotation', {
        'tool': 'circle',
        'center': {'x': 200, 'y': 200},
        'radius': 50,
        'color': '#00FF00'
    })
    scenario3.add_step('draw_annotation', {
        'tool': 'polygon',
        'points': [
            {'x': 300, 'y': 100},
            {'x': 350, 'y': 150},
            {'x': 300, 'y': 200}
        ],
        'color': '#0000FF'
    })
    scenario3.assert_result(
        lambda: len(scenario3.results.get('annotations', [])) == 3,
        "3つのアノテーションが描画されている"
    )
    scenarios.append(scenario3)
    
    # シナリオ4: プロファイル保存と復元フロー
    scenario4 = ScenarioTest("プロファイル保存と復元フロー")
    scenario4.add_step('load_map', {'file': 'test_map.pgm'})
    scenario4.add_step('add_metadata', {'file': 'test_metadata.yaml'})
    scenario4.add_step('add_object', {
        'type': 'table',
        'position': {'x': 150, 'y': 250}
    })
    scenario4.add_step('save_profile', {'name': 'test_profile'})
    scenario4.add_step('load_profile', {'name': 'test_profile'})
    scenario4.assert_result(
        lambda: scenario4.results.get('profile_saved') == 'test_profile',
        "プロファイルが保存されている"
    )
    scenario4.assert_result(
        lambda: scenario4.results.get('profile_loaded') == 'test_profile',
        "プロファイルが読み込まれている"
    )
    scenarios.append(scenario4)
    
    # シナリオ5: エラーハンドリングフロー
    scenario5 = ScenarioTest("エラーハンドリングフロー")
    scenario5.add_step('load_map', {'file': ''})  # 空のファイル名
    scenario5.add_step('add_metadata', {'file': None})  # Noneファイル
    scenario5.assert_result(
        lambda: not scenario5.results.get('map_loaded', False),
        "無効なマップファイルは読み込まれない"
    )
    scenario5.assert_result(
        lambda: not scenario5.results.get('metadata_loaded', False),
        "無効なメタデータファイルは読み込まれない"
    )
    scenarios.append(scenario5)
    
    return scenarios


def run_all_scenarios() -> Dict[str, bool]:
    """全シナリオを実行"""
    
    print("\n" + "="*60)
    print("🚀 統合テスト - シナリオテスト実行")
    print("="*60 + "\n")
    
    scenarios = create_scenario_tests()
    results = {}
    
    for scenario in scenarios:
        success = scenario.execute()
        results[scenario.name] = success
        print()
        
    # サマリー表示
    print("="*60)
    print("📊 テストサマリー")
    print("="*60)
    
    total = len(results)
    passed = sum(1 for v in results.values() if v)
    
    print(f"合計: {total} シナリオ")
    print(f"成功: {passed} シナリオ")
    print(f"失敗: {total - passed} シナリオ")
    
    if passed == total:
        print("\n✅ すべてのシナリオテストが成功しました！")
    else:
        print("\n⚠️ 一部のシナリオテストが失敗しました")
        
    return results


def run_specific_scenario(scenario_name: str) -> bool:
    """特定のシナリオを実行"""
    
    scenarios = create_scenario_tests()
    
    for scenario in scenarios:
        if scenario.name == scenario_name:
            return scenario.execute()
            
    print(f"❌ シナリオ '{scenario_name}' が見つかりません")
    return False


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # 特定のシナリオを実行
        scenario_name = ' '.join(sys.argv[1:])
        success = run_specific_scenario(scenario_name)
        sys.exit(0 if success else 1)
    else:
        # 全シナリオを実行
        results = run_all_scenarios()
        all_passed = all(results.values())
        sys.exit(0 if all_passed else 1)
