#!/usr/bin/env python3
"""
Frontend Server
Semantic Map Platform のフロントエンドサーバー
"""

from flask import Flask, send_from_directory, send_file
from pathlib import Path

app = Flask(__name__, static_folder='static')

STATIC_DIR = Path(__file__).parent / 'static'


@app.route('/')
def index():
    """トップページ"""
    index_file = STATIC_DIR / 'index.html'
    if index_file.exists():
        return send_file(index_file)
    else:
        return """
        <html>
        <body>
            <h1>Frontend not found</h1>
            <p>Please create static/index.html</p>
        </body>
        </html>
        """, 404


@app.route('/<path:path>')
def serve_static(path):
    """静的ファイル配信"""
    return send_from_directory(STATIC_DIR, path)


if __name__ == '__main__':
    print("=" * 60)
    print("🌐 Frontend Server Starting...")
    print("=" * 60)
    print()
    print("📂 Static files directory:", STATIC_DIR)
    print()
    print("=" * 60)

    app.run(host='0.0.0.0', port=5173, debug=True, use_reloader=False)
