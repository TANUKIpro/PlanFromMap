"""
統計情報APIのテスト
"""

import pytest
import json


class TestStatsAPI:
    """統計情報APIのテストクラス"""

    def test_get_stats_returns_200(self, client):
        """統計情報取得が200を返すことを確認"""
        response = client.get('/api/stats')
        assert response.status_code == 200

    def test_get_stats_returns_json(self, client):
        """統計情報取得がJSONを返すことを確認"""
        response = client.get('/api/stats')
        assert response.content_type == 'application/json'

    def test_get_stats_has_correct_structure(self, client):
        """統計情報のレスポンス構造を確認"""
        response = client.get('/api/stats')
        data = json.loads(response.data)

        assert 'operations_count' in data
        assert 'maps_count' in data
        assert 'operation_types' in data

    def test_get_stats_operation_types_structure(self, client):
        """統計情報のoperation_types構造を確認"""
        response = client.get('/api/stats')
        data = json.loads(response.data)

        operation_types = data['operation_types']
        assert isinstance(operation_types, dict)
        assert 'door' in operation_types
        assert 'drawer' in operation_types
        assert 'appliance' in operation_types

    def test_get_stats_counts_are_positive(self, client):
        """統計情報のカウントが0以上であることを確認"""
        response = client.get('/api/stats')
        data = json.loads(response.data)

        assert data['operations_count'] >= 0
        assert data['maps_count'] >= 0

    def test_get_stats_consistency(self, client, save_test_output):
        """統計情報の一貫性を確認"""
        response = client.get('/api/stats')
        data = json.loads(response.data)

        # operation_typesの合計がoperations_countと一致することを確認
        operation_types_sum = sum(data['operation_types'].values())
        assert operation_types_sum == data['operations_count']

        # テスト結果を保存
        save_test_output('stats_consistency_test', data)
