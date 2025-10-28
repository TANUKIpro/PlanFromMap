#!/usr/bin/env python3
"""
Backend API Server
Semantic Map Platform のバックエンドAPIサーバー
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
from datetime import datetime
import json
from pathlib import Path

app = Flask(__name__)
CORS(app)  # CORS有効化

# データストレージ（実運用ではDBを使用）
DATA_DIR = Path(__file__).parent.parent.parent / "data"
MAPS_DIR = DATA_DIR / "maps"
CATALOGS_DIR = DATA_DIR / "catalogs"

# ディレクトリ作成
MAPS_DIR.mkdir(parents=True, exist_ok=True)
CATALOGS_DIR.mkdir(parents=True, exist_ok=True)


# サンプルデータ
SAMPLE_OPERATIONS = [
    {
        "id": "kitchen_door_001",
        "name": "キッチンドア",
        "type": "door",
        "location": {"x": 5.2, "y": 3.1, "z": 0.0},
        "operation": {
            "type": "sliding",
            "direction": "left",
            "force": 20,
            "speed": 0.3
        },
        "created_at": "2024-01-15T10:00:00"
    },
    {
        "id": "refrigerator_001",
        "name": "冷蔵庫",
        "type": "appliance",
        "location": {"x": 2.5, "y": 4.0, "z": 0.0},
        "operation": {
            "type": "pull",
            "handle_height": 1.2,
            "force": 15,
            "angle": 90
        },
        "created_at": "2024-01-15T10:00:00"
    },
    {
        "id": "drawer_001",
        "name": "キッチン引き出し1",
        "type": "drawer",
        "location": {"x": 3.0, "y": 2.5, "z": 0.6},
        "operation": {
            "type": "pull",
            "depth": 0.4,
            "force": 10,
            "handle_type": "knob"
        },
        "created_at": "2024-01-15T10:00:00"
    }
]

SAMPLE_MAPS = [
    {
        "id": "map_001",
        "name": "リビング・キッチン",
        "type": "2d_occupancy",
        "resolution": 0.05,
        "width": 100,
        "height": 100,
        "origin": {"x": 0.0, "y": 0.0, "z": 0.0},
        "created_at": "2024-01-15T09:00:00"
    }
]


# ============================================
# ヘルスチェック
# ============================================
@app.route('/health', methods=['GET'])
def health_check():
    """ヘルスチェックエンドポイント"""
    return jsonify({
        "status": "ok",
        "service": "semantic-map-backend",
        "timestamp": datetime.now().isoformat()
    })


# ============================================
# 操作カタログAPI
# ============================================
@app.route('/api/operations', methods=['GET'])
def get_operations():
    """操作カタログ一覧を取得"""
    operation_type = request.args.get('type')

    if operation_type:
        filtered = [op for op in SAMPLE_OPERATIONS if op['type'] == operation_type]
        return jsonify({"operations": filtered, "count": len(filtered)})

    return jsonify({"operations": SAMPLE_OPERATIONS, "count": len(SAMPLE_OPERATIONS)})


@app.route('/api/operations/<operation_id>', methods=['GET'])
def get_operation(operation_id):
    """特定の操作仕様を取得"""
    operation = next((op for op in SAMPLE_OPERATIONS if op['id'] == operation_id), None)

    if operation:
        return jsonify(operation)
    else:
        return jsonify({"error": "Operation not found"}), 404


@app.route('/api/operations', methods=['POST'])
def create_operation():
    """新しい操作仕様を作成"""
    data = request.get_json()

    # 簡易的なバリデーション
    if not data or 'name' not in data or 'type' not in data:
        return jsonify({"error": "Invalid data"}), 400

    # IDを生成
    operation_id = f"{data['type']}_{len(SAMPLE_OPERATIONS) + 1:03d}"

    new_operation = {
        "id": operation_id,
        "name": data['name'],
        "type": data['type'],
        "location": data.get('location', {"x": 0.0, "y": 0.0, "z": 0.0}),
        "operation": data.get('operation', {}),
        "created_at": datetime.now().isoformat()
    }

    SAMPLE_OPERATIONS.append(new_operation)

    return jsonify(new_operation), 201


# ============================================
# 意味地図API
# ============================================
@app.route('/api/maps', methods=['GET'])
def get_maps():
    """意味地図一覧を取得"""
    return jsonify({"maps": SAMPLE_MAPS, "count": len(SAMPLE_MAPS)})


@app.route('/api/maps/<map_id>', methods=['GET'])
def get_map(map_id):
    """特定の意味地図を取得"""
    map_data = next((m for m in SAMPLE_MAPS if m['id'] == map_id), None)

    if map_data:
        return jsonify(map_data)
    else:
        return jsonify({"error": "Map not found"}), 404


# ============================================
# MapQLクエリAPI
# ============================================
@app.route('/api/mapql/query', methods=['POST'])
def mapql_query():
    """MapQLクエリを実行"""
    data = request.get_json()

    if not data or 'query' not in data:
        return jsonify({"error": "Query required"}), 400

    query = data['query']

    # 簡易的なクエリ処理（実装例）
    # GET Operation FOR 'kitchen_door'
    if 'GET Operation FOR' in query:
        # クエリからIDを抽出
        target_id = query.split("'")[1] if "'" in query else ""

        if target_id:
            result = next((op for op in SAMPLE_OPERATIONS if target_id in op['id'] or target_id in op['name']), None)

            if result:
                return jsonify({
                    "query": query,
                    "result": result,
                    "execution_time": "0.005s"
                })

    return jsonify({
        "query": query,
        "result": None,
        "message": "No matching operation found"
    })


# ============================================
# 統計情報API
# ============================================
@app.route('/api/stats', methods=['GET'])
def get_stats():
    """統計情報を取得"""
    return jsonify({
        "operations_count": len(SAMPLE_OPERATIONS),
        "maps_count": len(SAMPLE_MAPS),
        "operation_types": {
            "door": len([op for op in SAMPLE_OPERATIONS if op['type'] == 'door']),
            "drawer": len([op for op in SAMPLE_OPERATIONS if op['type'] == 'drawer']),
            "appliance": len([op for op in SAMPLE_OPERATIONS if op['type'] == 'appliance'])
        }
    })


if __name__ == '__main__':
    print("=" * 60)
    print("🚀 Backend API Server Starting...")
    print("=" * 60)
    print()
    print("📡 API Endpoints:")
    print("  - GET  /health")
    print("  - GET  /api/operations")
    print("  - GET  /api/operations/<id>")
    print("  - POST /api/operations")
    print("  - GET  /api/maps")
    print("  - GET  /api/maps/<id>")
    print("  - POST /api/mapql/query")
    print("  - GET  /api/stats")
    print()
    print("=" * 60)

    app.run(host='0.0.0.0', port=3000, debug=True, use_reloader=False)
