"""
操作カタログAPIのテスト
"""

import pytest
import json


class TestOperationsAPI:
    """操作カタログAPIのテストクラス"""

    def test_get_operations_returns_200(self, client):
        """操作カタログ一覧取得が200を返すことを確認"""
        response = client.get('/api/operations')
        assert response.status_code == 200

    def test_get_operations_returns_json(self, client):
        """操作カタログ一覧取得がJSONを返すことを確認"""
        response = client.get('/api/operations')
        assert response.content_type == 'application/json'

    def test_get_operations_has_correct_structure(self, client):
        """操作カタログ一覧のレスポンス構造を確認"""
        response = client.get('/api/operations')
        data = json.loads(response.data)

        assert 'operations' in data
        assert 'count' in data
        assert isinstance(data['operations'], list)
        assert isinstance(data['count'], int)

    def test_get_operations_returns_sample_data(self, client):
        """操作カタログ一覧がサンプルデータを返すことを確認"""
        response = client.get('/api/operations')
        data = json.loads(response.data)

        assert data['count'] > 0
        assert len(data['operations']) == data['count']

    def test_get_operations_item_structure(self, client):
        """操作カタログアイテムの構造を確認"""
        response = client.get('/api/operations')
        data = json.loads(response.data)

        if data['count'] > 0:
            operation = data['operations'][0]
            assert 'id' in operation
            assert 'name' in operation
            assert 'type' in operation
            assert 'location' in operation
            assert 'operation' in operation

    def test_get_operations_filter_by_type(self, client):
        """操作カタログのタイプフィルタリングを確認"""
        response = client.get('/api/operations?type=door')
        data = json.loads(response.data)

        assert response.status_code == 200
        for operation in data['operations']:
            assert operation['type'] == 'door'

    def test_get_operation_by_id_returns_200(self, client):
        """特定の操作仕様取得が200を返すことを確認"""
        response = client.get('/api/operations/kitchen_door_001')
        assert response.status_code == 200

    def test_get_operation_by_id_returns_correct_data(self, client):
        """特定の操作仕様が正しいデータを返すことを確認"""
        response = client.get('/api/operations/kitchen_door_001')
        data = json.loads(response.data)

        assert data['id'] == 'kitchen_door_001'
        assert 'name' in data
        assert 'type' in data

    def test_get_operation_by_invalid_id_returns_404(self, client):
        """存在しない操作仕様のIDで404を返すことを確認"""
        response = client.get('/api/operations/invalid_id')
        assert response.status_code == 404

    def test_create_operation_returns_201(self, client):
        """新しい操作仕様作成が201を返すことを確認"""
        new_operation = {
            'name': 'テストドア',
            'type': 'door',
            'location': {'x': 1.0, 'y': 2.0, 'z': 0.0},
            'operation': {'type': 'sliding', 'direction': 'left'}
        }
        response = client.post('/api/operations',
                               data=json.dumps(new_operation),
                               content_type='application/json')
        assert response.status_code == 201

    def test_create_operation_returns_created_data(self, client):
        """新しい操作仕様作成が作成されたデータを返すことを確認"""
        new_operation = {
            'name': 'テスト引き出し',
            'type': 'drawer',
            'location': {'x': 3.0, 'y': 4.0, 'z': 0.5}
        }
        response = client.post('/api/operations',
                               data=json.dumps(new_operation),
                               content_type='application/json')
        data = json.loads(response.data)

        assert 'id' in data
        assert data['name'] == 'テスト引き出し'
        assert data['type'] == 'drawer'
        assert 'created_at' in data

    def test_create_operation_without_name_returns_400(self, client):
        """nameなしで操作仕様作成を試みると400を返すことを確認"""
        invalid_operation = {
            'type': 'door'
        }
        response = client.post('/api/operations',
                               data=json.dumps(invalid_operation),
                               content_type='application/json')
        assert response.status_code == 400

    def test_create_operation_without_type_returns_400(self, client):
        """typeなしで操作仕様作成を試みると400を返すことを確認"""
        invalid_operation = {
            'name': 'テストドア'
        }
        response = client.post('/api/operations',
                               data=json.dumps(invalid_operation),
                               content_type='application/json')
        assert response.status_code == 400


@pytest.mark.integration
class TestOperationsWorkflow:
    """操作カタログAPIの統合ワークフローテスト"""

    def test_create_and_retrieve_operation(self, client, save_test_output):
        """操作仕様の作成と取得のワークフローを確認"""
        # 1. 新しい操作仕様を作成
        new_operation = {
            'name': 'ワークフローテストドア',
            'type': 'door',
            'location': {'x': 5.5, 'y': 3.3, 'z': 0.0},
            'operation': {
                'type': 'push',
                'force': 25,
                'direction': 'forward'
            }
        }
        create_response = client.post('/api/operations',
                                       data=json.dumps(new_operation),
                                       content_type='application/json')
        assert create_response.status_code == 201
        created_data = json.loads(create_response.data)
        operation_id = created_data['id']

        # 2. 作成された操作仕様を取得
        get_response = client.get(f'/api/operations/{operation_id}')
        assert get_response.status_code == 200
        retrieved_data = json.loads(get_response.data)

        # 3. データが一致することを確認
        assert retrieved_data['id'] == operation_id
        assert retrieved_data['name'] == 'ワークフローテストドア'

        # 4. テスト結果を保存
        save_test_output('operation_workflow_test', {
            'created': created_data,
            'retrieved': retrieved_data
        })
