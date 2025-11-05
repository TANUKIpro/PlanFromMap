"""
Pytest Configuration and Shared Fixtures
pytestの設定と共有フィクスチャ
"""
import sys
import os
from pathlib import Path
import pytest
import json
from datetime import datetime

# プロジェクトルートをPythonパスに追加
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / "apps" / "backend"))
sys.path.insert(0, str(project_root / "apps" / "frontend"))


@pytest.fixture(scope="session")
def test_data_dir():
    """テストデータディレクトリのパス"""
    return Path(__file__).parent / "fixtures"


@pytest.fixture(scope="session")
def test_output_dir():
    """テスト出力ディレクトリのパス"""
    output_dir = Path(__file__).parent / "outputs"
    output_dir.mkdir(exist_ok=True, parents=True)
    return output_dir


@pytest.fixture(scope="session")
def test_profiles_dir(test_output_dir):
    """テストプロファイル出力ディレクトリ"""
    profiles_dir = test_output_dir / "profiles"
    profiles_dir.mkdir(exist_ok=True, parents=True)
    return profiles_dir


@pytest.fixture(scope="session")
def test_screenshots_dir(test_output_dir):
    """テストスクリーンショット出力ディレクトリ"""
    screenshots_dir = test_output_dir / "screenshots"
    screenshots_dir.mkdir(exist_ok=True, parents=True)
    return screenshots_dir


@pytest.fixture(scope="session")
def test_reports_dir(test_output_dir):
    """テストレポート出力ディレクトリ"""
    reports_dir = test_output_dir / "reports"
    reports_dir.mkdir(exist_ok=True, parents=True)
    return reports_dir


@pytest.fixture(scope="session")
def sample_map_files():
    """実際のマップファイルへのパス"""
    maps_dir = project_root / "data" / "maps"
    return {
        "yaml": maps_dir / "tid_lab_map.yaml",
        "pgm": maps_dir / "tid_lb_map.pgm"
    }


@pytest.fixture
def sample_yaml_data():
    """サンプルYAMLデータ"""
    return {
        "image": "map.pgm",
        "resolution": 0.05,
        "origin": [-51.224998, -51.224998, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196
    }


@pytest.fixture
def test_metadata():
    """テスト用メタデータ"""
    return {
        "test_name": "Map Feature Test Suite",
        "test_date": datetime.now().isoformat(),
        "version": "1.0.0"
    }


@pytest.fixture
def api_client():
    """バックエンドAPIクライアント"""
    from apps.backend.server import app
    app.config['TESTING'] = True
    with app.test_client() as client:
        yield client


class TestResult:
    """テスト結果を保存するヘルパークラス"""

    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.results = []

    def add_result(self, test_name: str, status: str, duration: float,
                   details: dict = None):
        """テスト結果を追加"""
        self.results.append({
            "test_name": test_name,
            "status": status,
            "duration": duration,
            "timestamp": datetime.now().isoformat(),
            "details": details or {}
        })

    def save_report(self, filename: str = "test_report.json"):
        """レポートをJSONファイルに保存"""
        report_path = self.output_dir / filename
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump({
                "summary": {
                    "total": len(self.results),
                    "passed": sum(1 for r in self.results if r["status"] == "passed"),
                    "failed": sum(1 for r in self.results if r["status"] == "failed"),
                    "skipped": sum(1 for r in self.results if r["status"] == "skipped"),
                },
                "results": self.results,
                "generated_at": datetime.now().isoformat()
            }, f, indent=2, ensure_ascii=False)
        return report_path


@pytest.fixture
def test_result_tracker(test_reports_dir):
    """テスト結果トラッカー"""
    return TestResult(test_reports_dir)


def pytest_configure(config):
    """pytest設定のカスタマイズ"""
    config.addinivalue_line(
        "markers", "unit: ユニットテスト（高速）"
    )
    config.addinivalue_line(
        "markers", "integration: 統合テスト（中速）"
    )
    config.addinivalue_line(
        "markers", "e2e: E2Eテスト（低速）"
    )
    config.addinivalue_line(
        "markers", "slow: 時間がかかるテスト"
    )


def pytest_collection_modifyitems(config, items):
    """テストアイテムのカスタマイズ"""
    for item in items:
        # テストディレクトリに基づいてマーカーを自動追加
        if "unit" in str(item.fspath):
            item.add_marker(pytest.mark.unit)
        elif "integration" in str(item.fspath):
            item.add_marker(pytest.mark.integration)
        elif "e2e" in str(item.fspath):
            item.add_marker(pytest.mark.e2e)
