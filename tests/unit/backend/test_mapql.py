"""
MapQLクエリAPIのテスト
"""

import pytest
import json


class TestMapQLAPI:
    """MapQLクエリAPIのテストクラス"""

    def test_mapql_query_returns_200(self, client):
        """MapQLクエリが200を返すことを確認"""
        query_data = {
            'query': "GET Operation FOR 'kitchen_door'"
        }
        response = client.post('/api/mapql/query',
                               data=json.dumps(query_data),
                               content_type='application/json')
        assert response.status_code == 200

    def test_mapql_query_returns_json(self, client):
        """MapQLクエリがJSONを返すことを確認"""
        query_data = {
            'query': "GET Operation FOR 'kitchen_door'"
        }
        response = client.post('/api/mapql/query',
                               data=json.dumps(query_data),
                               content_type='application/json')
        assert response.content_type == 'application/json'

    def test_mapql_query_has_correct_structure(self, client):
        """MapQLクエリのレスポンス構造を確認"""
        query_data = {
            'query': "GET Operation FOR 'kitchen_door'"
        }
        response = client.post('/api/mapql/query',
                               data=json.dumps(query_data),
                               content_type='application/json')
        data = json.loads(response.data)

        assert 'query' in data
        assert 'result' in data

    def test_mapql_query_finds_existing_operation(self, client):
        """MapQLクエリが既存の操作を見つけることを確認"""
        query_data = {
            'query': "GET Operation FOR 'kitchen_door'"
        }
        response = client.post('/api/mapql/query',
                               data=json.dumps(query_data),
                               content_type='application/json')
        data = json.loads(response.data)

        assert data['result'] is not None
        assert 'id' in data['result']

    def test_mapql_query_without_query_returns_400(self, client):
        """queryなしでMapQLクエリを試みると400を返すことを確認"""
        response = client.post('/api/mapql/query',
                               data=json.dumps({}),
                               content_type='application/json')
        assert response.status_code == 400

    def test_mapql_query_nonexistent_operation(self, client, save_test_output):
        """存在しない操作をMapQLクエリで検索した場合の動作を確認"""
        query_data = {
            'query': "GET Operation FOR 'nonexistent_item'"
        }
        response = client.post('/api/mapql/query',
                               data=json.dumps(query_data),
                               content_type='application/json')
        data = json.loads(response.data)

        assert response.status_code == 200
        assert data['result'] is None or 'message' in data

        # テスト結果を保存
        save_test_output('mapql_nonexistent_query_test', data)
