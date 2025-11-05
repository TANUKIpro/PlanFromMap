"""
Backend API Tests
バックエンドAPIエンドポイントのユニットテスト
"""
import pytest
import json
from pathlib import Path


class TestHealthCheck:
    """ヘルスチェックエンドポイントのテスト"""

    def test_health_check_endpoint(self, api_client):
        """ヘルスチェックエンドポイントが正常に動作する"""
        response = api_client.get('/health')
        assert response.status_code == 200

        data = json.loads(response.data)
        assert data['status'] == 'ok'
        assert data['service'] == 'semantic-map-backend'
        assert 'timestamp' in data

    def test_health_check_returns_json(self, api_client):
        """ヘルスチェックがJSON形式でレスポンスを返す"""
        response = api_client.get('/health')
        assert response.content_type == 'application/json'


class TestOperationsAPI:
    """操作カタログAPIのテスト"""

    def test_get_all_operations(self, api_client):
        """すべての操作を取得できる"""
        response = api_client.get('/api/operations')
        assert response.status_code == 200

        data = json.loads(response.data)
        assert 'operations' in data
        assert 'count' in data
        assert isinstance(data['operations'], list)
        assert data['count'] > 0

    def test_get_operations_by_type(self, api_client):
        """タイプでフィルタリングして操作を取得できる"""
        response = api_client.get('/api/operations?type=door')
        assert response.status_code == 200

        data = json.loads(response.data)
        assert 'operations' in data

        # すべての操作がdoorタイプであることを確認
        for operation in data['operations']:
            assert operation['type'] == 'door'

    def test_get_single_operation(self, api_client):
        """特定の操作を取得できる"""
        # まずすべての操作を取得
        response = api_client.get('/api/operations')
        data = json.loads(response.data)
        first_operation_id = data['operations'][0]['id']

        # 特定の操作を取得
        response = api_client.get(f'/api/operations/{first_operation_id}')
        assert response.status_code == 200

        operation_data = json.loads(response.data)
        assert operation_data['id'] == first_operation_id
        assert 'name' in operation_data
        assert 'type' in operation_data
        assert 'location' in operation_data
        assert 'operation' in operation_data

    def test_get_nonexistent_operation(self, api_client):
        """存在しない操作を取得すると404が返る"""
        response = api_client.get('/api/operations/nonexistent_id')
        assert response.status_code == 404

        data = json.loads(response.data)
        assert 'error' in data

    def test_create_operation(self, api_client):
        """新しい操作を作成できる"""
        new_operation = {
            'name': 'テストドア',
            'type': 'door',
            'location': {'x': 1.0, 'y': 2.0, 'z': 0.0},
            'operation': {
                'type': 'sliding',
                'direction': 'right',
                'force': 25,
                'speed': 0.5
            }
        }

        response = api_client.post(
            '/api/operations',
            data=json.dumps(new_operation),
            content_type='application/json'
        )
        assert response.status_code == 201

        data = json.loads(response.data)
        assert 'id' in data
        assert data['name'] == new_operation['name']
        assert data['type'] == new_operation['type']

    def test_create_operation_invalid_data(self, api_client):
        """不正なデータで操作を作成すると400が返る"""
        invalid_operation = {
            'invalid_field': 'value'
        }

        response = api_client.post(
            '/api/operations',
            data=json.dumps(invalid_operation),
            content_type='application/json'
        )
        assert response.status_code == 400

        data = json.loads(response.data)
        assert 'error' in data


class TestMapsAPI:
    """意味地図APIのテスト"""

    def test_get_all_maps(self, api_client):
        """すべてのマップを取得できる"""
        response = api_client.get('/api/maps')
        assert response.status_code == 200

        data = json.loads(response.data)
        assert 'maps' in data
        assert 'count' in data
        assert isinstance(data['maps'], list)

    def test_get_single_map(self, api_client):
        """特定のマップを取得できる"""
        # まずすべてのマップを取得
        response = api_client.get('/api/maps')
        data = json.loads(response.data)

        if data['count'] > 0:
            first_map_id = data['maps'][0]['id']

            # 特定のマップを取得
            response = api_client.get(f'/api/maps/{first_map_id}')
            assert response.status_code == 200

            map_data = json.loads(response.data)
            assert map_data['id'] == first_map_id
            assert 'name' in map_data
            assert 'resolution' in map_data

    def test_get_nonexistent_map(self, api_client):
        """存在しないマップを取得すると404が返る"""
        response = api_client.get('/api/maps/nonexistent_id')
        assert response.status_code == 404


class TestMapQLAPI:
    """MapQLクエリAPIのテスト"""

    def test_mapql_query_success(self, api_client):
        """MapQLクエリが正常に実行される"""
        query = {
            'query': "GET Operation FOR 'kitchen_door'"
        }

        response = api_client.post(
            '/api/mapql/query',
            data=json.dumps(query),
            content_type='application/json'
        )
        assert response.status_code == 200

        data = json.loads(response.data)
        assert 'query' in data
        assert 'result' in data
        assert data['query'] == query['query']

    def test_mapql_query_without_query(self, api_client):
        """クエリなしでリクエストすると400が返る"""
        response = api_client.post(
            '/api/mapql/query',
            data=json.dumps({}),
            content_type='application/json'
        )
        assert response.status_code == 400

        data = json.loads(response.data)
        assert 'error' in data

    def test_mapql_query_no_match(self, api_client):
        """マッチしないクエリでも正常にレスポンスを返す"""
        query = {
            'query': "GET Operation FOR 'nonexistent_item'"
        }

        response = api_client.post(
            '/api/mapql/query',
            data=json.dumps(query),
            content_type='application/json'
        )
        assert response.status_code == 200

        data = json.loads(response.data)
        assert data['result'] is None
        assert 'message' in data


class TestStatsAPI:
    """統計情報APIのテスト"""

    def test_get_stats(self, api_client):
        """統計情報を取得できる"""
        response = api_client.get('/api/stats')
        assert response.status_code == 200

        data = json.loads(response.data)
        assert 'operations_count' in data
        assert 'maps_count' in data
        assert 'operation_types' in data

        # 統計情報が数値であることを確認
        assert isinstance(data['operations_count'], int)
        assert isinstance(data['maps_count'], int)
        assert isinstance(data['operation_types'], dict)

    def test_stats_operation_types_breakdown(self, api_client):
        """統計情報にオペレーションタイプの内訳が含まれる"""
        response = api_client.get('/api/stats')
        data = json.loads(response.data)

        operation_types = data['operation_types']
        assert 'door' in operation_types
        assert 'drawer' in operation_types
        assert 'appliance' in operation_types

        # カウントが非負の整数であることを確認
        for count in operation_types.values():
            assert isinstance(count, int)
            assert count >= 0
