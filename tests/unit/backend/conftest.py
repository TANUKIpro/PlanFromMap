"""
Backend unit tests用のfixture
"""

import pytest
import sys
from pathlib import Path

# プロジェクトルートをPythonパスに追加
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from apps.backend.server import app


@pytest.fixture(scope="function")
def client():
    """Flaskテストクライアント"""
    app.config['TESTING'] = True
    with app.test_client() as client:
        yield client


@pytest.fixture(scope="function")
def app_context():
    """Flaskアプリケーションコンテキスト"""
    with app.app_context():
        yield app
