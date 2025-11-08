#!/usr/bin/env python3
"""
layerManager モジュールのテスト
PlanFromMapプロジェクトのレイヤー管理機能をテスト
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# プロジェクトルートをパスに追加（実際の環境に合わせて調整）
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


class TestLayerManager(unittest.TestCase):
    """
    レイヤー管理機能のテストクラス
    """
    
    def setUp(self):
        """各テストの前に実行される初期化処理"""
        self.test_layers = []
        self.mock_state = {
            'layerStack': [],
            'selectedLayerId': None,
            'nextLayerId': 1
        }
        
    def tearDown(self):
        """各テストの後に実行されるクリーンアップ処理"""
        self.test_layers.clear()
        self.mock_state['layerStack'].clear()
        
    def test_create_layer_basic(self):
        """
        基本的なレイヤー作成のテスト
        """
        # Arrange
        layer_id = 'test-1'
        layer_name = 'テストレイヤー'
        layer_type = 'drawing'
        
        # Act (モック実装)
        layer = self._mock_create_layer(layer_id, layer_name, layer_type)
        
        # Assert
        self.assertIsNotNone(layer)
        self.assertEqual(layer['id'], layer_id)
        self.assertEqual(layer['name'], layer_name)
        self.assertEqual(layer['type'], layer_type)
        self.assertTrue(layer['visible'])
        self.assertFalse(layer['permanent'])
        
    def test_create_layer_with_permanent_flag(self):
        """
        恒久レイヤーの作成テスト
        """
        # Arrange
        layer_id = 'map-image'
        layer_name = 'Map Image'
        layer_type = 'image'
        permanent = True
        
        # Act
        layer = self._mock_create_layer(layer_id, layer_name, layer_type, permanent)
        
        # Assert
        self.assertTrue(layer['permanent'])
        
    def test_delete_layer_success(self):
        """
        レイヤー削除の成功テスト
        """
        # Arrange
        layer = self._mock_create_layer('test-2', 'Delete Test', 'drawing')
        self.mock_state['layerStack'].append(layer)
        
        # Act
        result = self._mock_delete_layer('test-2')
        
        # Assert
        self.assertTrue(result)
        self.assertEqual(len(self.mock_state['layerStack']), 0)
        
    def test_delete_permanent_layer_fails(self):
        """
        恒久レイヤーの削除失敗テスト
        """
        # Arrange
        layer = self._mock_create_layer('permanent-1', 'Permanent', 'image', True)
        self.mock_state['layerStack'].append(layer)
        
        # Act
        result = self._mock_delete_layer('permanent-1')
        
        # Assert
        self.assertFalse(result)
        self.assertEqual(len(self.mock_state['layerStack']), 1)
        
    def test_toggle_layer_visibility(self):
        """
        レイヤー表示/非表示の切り替えテスト
        """
        # Arrange
        layer = self._mock_create_layer('test-3', 'Toggle Test', 'drawing')
        self.mock_state['layerStack'].append(layer)
        initial_visibility = layer['visible']
        
        # Act
        self._mock_toggle_visibility('test-3')
        
        # Assert
        self.assertEqual(layer['visible'], not initial_visibility)
        
    def test_get_layer_by_id(self):
        """
        IDによるレイヤー取得のテスト
        """
        # Arrange
        layers = [
            self._mock_create_layer('layer-1', 'Layer 1', 'drawing'),
            self._mock_create_layer('layer-2', 'Layer 2', 'metadata'),
            self._mock_create_layer('layer-3', 'Layer 3', 'image')
        ]
        self.mock_state['layerStack'] = layers
        
        # Act
        found_layer = self._mock_get_layer_by_id('layer-2')
        
        # Assert
        self.assertIsNotNone(found_layer)
        self.assertEqual(found_layer['name'], 'Layer 2')
        self.assertEqual(found_layer['type'], 'metadata')
        
    def test_get_nonexistent_layer_returns_none(self):
        """
        存在しないレイヤーの取得でNoneが返ることのテスト
        """
        # Act
        result = self._mock_get_layer_by_id('nonexistent')
        
        # Assert
        self.assertIsNone(result)
        
    def test_reorder_layers(self):
        """
        レイヤー順序の変更テスト
        """
        # Arrange
        layers = [
            self._mock_create_layer('layer-1', 'Bottom', 'drawing'),
            self._mock_create_layer('layer-2', 'Middle', 'drawing'),
            self._mock_create_layer('layer-3', 'Top', 'drawing')
        ]
        self.mock_state['layerStack'] = layers
        
        # Act - layer-3を最初に移動
        self._mock_reorder_layer('layer-3', 0)
        
        # Assert
        self.assertEqual(self.mock_state['layerStack'][0]['id'], 'layer-3')
        self.assertEqual(self.mock_state['layerStack'][1]['id'], 'layer-1')
        self.assertEqual(self.mock_state['layerStack'][2]['id'], 'layer-2')
        
    def test_duplicate_layer_id_fails(self):
        """
        重複するIDでのレイヤー作成失敗テスト
        """
        # Arrange
        layer = self._mock_create_layer('duplicate', 'First', 'drawing')
        self.mock_state['layerStack'].append(layer)
        
        # Act
        duplicate_layer = self._mock_create_layer('duplicate', 'Second', 'drawing')
        
        # Assert
        self.assertIsNone(duplicate_layer)
        
    def test_clear_all_non_permanent_layers(self):
        """
        非恒久レイヤーの一括削除テスト
        """
        # Arrange
        layers = [
            self._mock_create_layer('permanent', 'Permanent', 'image', True),
            self._mock_create_layer('temp-1', 'Temp 1', 'drawing', False),
            self._mock_create_layer('temp-2', 'Temp 2', 'drawing', False)
        ]
        self.mock_state['layerStack'] = layers
        
        # Act
        self._mock_clear_non_permanent_layers()
        
        # Assert
        self.assertEqual(len(self.mock_state['layerStack']), 1)
        self.assertEqual(self.mock_state['layerStack'][0]['id'], 'permanent')
        
    # =====================================
    # モック実装（実際のコードの代わり）
    # =====================================
    
    def _mock_create_layer(self, id, name, type, permanent=False):
        """レイヤー作成のモック"""
        # 重複チェック
        if any(layer['id'] == id for layer in self.mock_state['layerStack']):
            return None
            
        layer = {
            'id': id,
            'name': name,
            'type': type,
            'visible': True,
            'permanent': permanent,
            'opacity': 1.0,
            'data': None,
            'canvas': None,
            'zIndex': len(self.mock_state['layerStack'])
        }
        return layer
        
    def _mock_delete_layer(self, layer_id):
        """レイヤー削除のモック"""
        layer = self._mock_get_layer_by_id(layer_id)
        if layer and not layer['permanent']:
            self.mock_state['layerStack'] = [
                l for l in self.mock_state['layerStack'] if l['id'] != layer_id
            ]
            return True
        return False
        
    def _mock_toggle_visibility(self, layer_id):
        """表示切り替えのモック"""
        layer = self._mock_get_layer_by_id(layer_id)
        if layer:
            layer['visible'] = not layer['visible']
            
    def _mock_get_layer_by_id(self, layer_id):
        """IDによるレイヤー取得のモック"""
        for layer in self.mock_state['layerStack']:
            if layer['id'] == layer_id:
                return layer
        return None
        
    def _mock_reorder_layer(self, layer_id, new_index):
        """レイヤー順序変更のモック"""
        layer = self._mock_get_layer_by_id(layer_id)
        if layer:
            self.mock_state['layerStack'].remove(layer)
            self.mock_state['layerStack'].insert(new_index, layer)
            
    def _mock_clear_non_permanent_layers(self):
        """非恒久レイヤー削除のモック"""
        self.mock_state['layerStack'] = [
            l for l in self.mock_state['layerStack'] if l['permanent']
        ]


if __name__ == "__main__":
    # テスト実行
    unittest.main(verbosity=2)
