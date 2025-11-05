"""
意味地図APIのテスト
"""

import pytest
import json


class TestMapsAPI:
    """意味地図APIのテストクラス"""

    def test_get_maps_returns_200(self, client):
        """意味地図一覧取得が200を返すことを確認"""
        response = client.get('/api/maps')
        assert response.status_code == 200

    def test_get_maps_returns_json(self, client):
        """意味地図一覧取得がJSONを返すことを確認"""
        response = client.get('/api/maps')
        assert response.content_type == 'application/json'

    def test_get_maps_has_correct_structure(self, client):
        """意味地図一覧のレスポンス構造を確認"""
        response = client.get('/api/maps')
        data = json.loads(response.data)

        assert 'maps' in data
        assert 'count' in data
        assert isinstance(data['maps'], list)
        assert isinstance(data['count'], int)

    def test_get_maps_item_structure(self, client):
        """意味地図アイテムの構造を確認"""
        response = client.get('/api/maps')
        data = json.loads(response.data)

        if data['count'] > 0:
            map_item = data['maps'][0]
            assert 'id' in map_item
            assert 'name' in map_item
            assert 'type' in map_item
            assert 'resolution' in map_item
            assert 'width' in map_item
            assert 'height' in map_item
            assert 'origin' in map_item

    def test_get_map_by_id_returns_200(self, client):
        """特定の意味地図取得が200を返すことを確認"""
        response = client.get('/api/maps/map_001')
        assert response.status_code == 200

    def test_get_map_by_id_returns_correct_data(self, client):
        """特定の意味地図が正しいデータを返すことを確認"""
        response = client.get('/api/maps/map_001')
        data = json.loads(response.data)

        assert data['id'] == 'map_001'
        assert 'name' in data
        assert 'type' in data

    def test_get_map_by_invalid_id_returns_404(self, client):
        """存在しない意味地図のIDで404を返すことを確認"""
        response = client.get('/api/maps/invalid_map_id')
        assert response.status_code == 404
