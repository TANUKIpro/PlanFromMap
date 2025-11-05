"""
ヘルスチェックエンドポイントのテスト
"""

import pytest
import json


class TestHealthEndpoint:
    """ヘルスチェックエンドポイントのテストクラス"""

    def test_health_check_returns_200(self, client):
        """ヘルスチェックが200を返すことを確認"""
        response = client.get('/health')
        assert response.status_code == 200

    def test_health_check_returns_json(self, client):
        """ヘルスチェックがJSONを返すことを確認"""
        response = client.get('/health')
        assert response.content_type == 'application/json'

    def test_health_check_has_correct_structure(self, client):
        """ヘルスチェックのレスポンス構造を確認"""
        response = client.get('/health')
        data = json.loads(response.data)

        assert 'status' in data
        assert 'service' in data
        assert 'timestamp' in data

    def test_health_check_status_is_ok(self, client):
        """ヘルスチェックのstatusがokであることを確認"""
        response = client.get('/health')
        data = json.loads(response.data)

        assert data['status'] == 'ok'
        assert data['service'] == 'semantic-map-backend'
