"""
pytest設定ファイル
すべてのテストで共通で使用するfixture、設定を定義
"""

import sys
import os
from pathlib import Path
import pytest
import json
import shutil

# プロジェクトルートをPythonパスに追加
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# テストデータディレクトリ
TEST_DATA_DIR = Path(__file__).parent / "fixtures"
TEST_MAPS_DIR = TEST_DATA_DIR / "maps"
TEST_PROFILES_DIR = TEST_DATA_DIR / "profiles"
TEST_REPORTS_DIR = Path(__file__).parent / "reports"

# ディレクトリ作成
TEST_MAPS_DIR.mkdir(parents=True, exist_ok=True)
TEST_PROFILES_DIR.mkdir(parents=True, exist_ok=True)
TEST_REPORTS_DIR.mkdir(parents=True, exist_ok=True)


@pytest.fixture(scope="session")
def test_data_dir():
    """テストデータディレクトリのパスを返す"""
    return TEST_DATA_DIR


@pytest.fixture(scope="session")
def test_maps_dir():
    """テストマップディレクトリのパスを返す"""
    return TEST_MAPS_DIR


@pytest.fixture(scope="session")
def test_profiles_dir():
    """テストプロファイルディレクトリのパスを返す"""
    return TEST_PROFILES_DIR


@pytest.fixture(scope="session")
def test_reports_dir():
    """テストレポートディレクトリのパスを返す"""
    return TEST_REPORTS_DIR


@pytest.fixture(scope="function")
def sample_map_yaml(test_maps_dir):
    """サンプルマップYAMLファイルを作成"""
    yaml_path = test_maps_dir / "test_map.yaml"
    yaml_content = """image: test_map.pgm
resolution: 0.050000
origin: [-10.0, -10.0, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    yaml_path.write_text(yaml_content)
    yield yaml_path
    # クリーンアップ（必要に応じて）
    # yaml_path.unlink()


@pytest.fixture(scope="function")
def sample_operation():
    """サンプル操作データを返す"""
    return {
        "id": "test_door_001",
        "name": "テストドア",
        "type": "door",
        "location": {"x": 1.0, "y": 2.0, "z": 0.0},
        "operation": {
            "type": "sliding",
            "direction": "left",
            "force": 20,
            "speed": 0.3
        }
    }


@pytest.fixture(scope="function")
def sample_profile(test_profiles_dir):
    """サンプルプロファイルを作成"""
    profile_data = {
        "profileName": "test_profile",
        "timestamp": "2024-01-01T00:00:00.000Z",
        "mapImage": None,  # テスト用なのでNone
        "metadata": {
            "resolution": 0.05,
            "origin": [-10.0, -10.0, 0.0]
        },
        "layers": []
    }

    profile_path = test_profiles_dir / "test_profile.json"
    with open(profile_path, 'w', encoding='utf-8') as f:
        json.dump(profile_data, f, ensure_ascii=False, indent=2)

    yield profile_path
    # クリーンアップ（必要に応じて）
    # profile_path.unlink()


@pytest.fixture(scope="function")
def save_test_output():
    """テスト実行結果を保存するヘルパー関数を返す"""
    def _save_output(test_name, data, file_type="json"):
        """
        テスト実行結果を保存

        Args:
            test_name: テスト名
            data: 保存するデータ
            file_type: ファイルタイプ ("json", "txt", "yaml")
        """
        output_path = TEST_REPORTS_DIR / f"{test_name}.{file_type}"

        if file_type == "json":
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        elif file_type == "txt":
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(str(data))
        elif file_type == "yaml":
            import yaml
            with open(output_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True)

        return output_path

    return _save_output


# pytest設定
def pytest_configure(config):
    """pytest起動時の設定"""
    print("\n" + "=" * 60)
    print("🧪 Semantic Map Platform - Test Suite")
    print("=" * 60)
    print(f"📂 Test data directory: {TEST_DATA_DIR}")
    print(f"📂 Test reports directory: {TEST_REPORTS_DIR}")
    print("=" * 60 + "\n")


def pytest_collection_modifyitems(config, items):
    """テストアイテムの収集後に実行される"""
    # テストの並び順を調整したり、マーカーを追加したりできる
    pass


@pytest.fixture(autouse=True)
def cleanup_after_test(request):
    """各テスト実行後の自動クリーンアップ"""
    yield
    # テスト終了後の処理
    # 必要に応じてここでクリーンアップ処理を追加
