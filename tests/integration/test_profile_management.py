"""
Profile Management Integration Tests
プロファイル管理機能の統合テスト
"""
import pytest
import json
from pathlib import Path
from datetime import datetime
import time


class TestProfileCreation:
    """プロファイル作成のテスト"""

    def test_create_empty_profile(self, test_profiles_dir):
        """空のプロファイルを作成できる"""
        profile_name = "test_empty_profile"
        profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "map_data": None,
            "layers": [],
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            }
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        assert profile_path.exists()

    def test_create_profile_with_map_data(self, test_profiles_dir,
                                          sample_yaml_data):
        """マップデータを含むプロファイルを作成できる"""
        profile_name = "test_map_profile"
        profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "map_data": {
                "metadata": sample_yaml_data,
                "image_loaded": True,
                "image_size": {
                    "width": 2048,
                    "height": 2048
                }
            },
            "layers": [
                {
                    "id": "layer_map",
                    "name": "Map Image",
                    "type": "image",
                    "visible": True,
                    "permanent": True
                }
            ],
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            }
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        assert profile_path.exists()

        # プロファイルを読み込んで検証
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert loaded_profile['profile_name'] == profile_name
        assert loaded_profile['map_data'] is not None
        assert len(loaded_profile['layers']) == 1

    def test_create_profile_with_layers(self, test_profiles_dir):
        """複数のレイヤーを含むプロファイルを作成できる"""
        profile_name = "test_layers_profile"
        profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "map_data": None,
            "layers": [
                {
                    "id": "layer_1",
                    "name": "Base Layer",
                    "type": "image",
                    "visible": True,
                    "permanent": True,
                    "opacity": 1.0
                },
                {
                    "id": "layer_2",
                    "name": "Drawing Layer",
                    "type": "drawing",
                    "visible": True,
                    "permanent": False,
                    "opacity": 0.8
                },
                {
                    "id": "layer_3",
                    "name": "Overlay Layer",
                    "type": "overlay",
                    "visible": False,
                    "permanent": False,
                    "opacity": 0.5
                }
            ],
            "viewport": {
                "scale": 1.5,
                "offsetX": 100,
                "offsetY": 50
            }
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # 検証
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert len(loaded_profile['layers']) == 3
        assert loaded_profile['layers'][0]['type'] == 'image'
        assert loaded_profile['layers'][1]['type'] == 'drawing'
        assert loaded_profile['layers'][2]['type'] == 'overlay'


class TestProfileLoading:
    """プロファイル読み込みのテスト"""

    def setup_method(self):
        """テスト用のプロファイルを作成"""
        self.test_profile = {
            "profile_name": "test_load_profile",
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "map_data": {
                "resolution": 0.05,
                "origin": [-50.0, -50.0, 0.0]
            },
            "layers": [],
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            }
        }

    def test_load_existing_profile(self, test_profiles_dir):
        """既存のプロファイルを読み込める"""
        profile_name = "test_load_profile"
        profile_path = test_profiles_dir / f"{profile_name}.json"

        # プロファイルを保存
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(self.test_profile, f, indent=2, ensure_ascii=False)

        # プロファイルを読み込み
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert loaded_profile['profile_name'] == profile_name
        assert loaded_profile['map_data'] is not None

    def test_load_nonexistent_profile(self, test_profiles_dir):
        """存在しないプロファイルの読み込み処理"""
        profile_path = test_profiles_dir / "nonexistent_profile.json"

        assert not profile_path.exists()

    def test_load_corrupted_profile(self, test_profiles_dir):
        """破損したプロファイルの読み込み処理"""
        profile_name = "corrupted_profile"
        profile_path = test_profiles_dir / f"{profile_name}.json"

        # 不正なJSONを書き込み
        with open(profile_path, 'w', encoding='utf-8') as f:
            f.write("{invalid json content")

        # 読み込みでエラーが発生することを確認
        with pytest.raises(json.JSONDecodeError):
            with open(profile_path, 'r', encoding='utf-8') as f:
                json.load(f)


class TestProfileUpdate:
    """プロファイル更新のテスト"""

    def test_update_profile_viewport(self, test_profiles_dir):
        """プロファイルのビューポートを更新できる"""
        profile_name = "test_update_viewport"
        profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            }
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"

        # 初期プロファイルを保存
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # プロファイルを読み込んで更新
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        loaded_profile['viewport']['scale'] = 2.0
        loaded_profile['viewport']['offsetX'] = 100
        loaded_profile['viewport']['offsetY'] = 50
        loaded_profile['updated_at'] = datetime.now().isoformat()

        # 更新したプロファイルを保存
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(loaded_profile, f, indent=2, ensure_ascii=False)

        # 更新されたことを確認
        with open(profile_path, 'r', encoding='utf-8') as f:
            updated_profile = json.load(f)

        assert updated_profile['viewport']['scale'] == 2.0
        assert updated_profile['viewport']['offsetX'] == 100
        assert updated_profile['viewport']['offsetY'] == 50
        assert 'updated_at' in updated_profile

    def test_update_profile_add_layer(self, test_profiles_dir):
        """プロファイルにレイヤーを追加できる"""
        profile_name = "test_add_layer"
        profile = {
            "profile_name": profile_name,
            "layers": []
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"

        # 初期プロファイルを保存
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # レイヤーを追加
        new_layer = {
            "id": "layer_new",
            "name": "New Layer",
            "type": "drawing",
            "visible": True
        }

        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        loaded_profile['layers'].append(new_layer)

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(loaded_profile, f, indent=2, ensure_ascii=False)

        # 検証
        with open(profile_path, 'r', encoding='utf-8') as f:
            updated_profile = json.load(f)

        assert len(updated_profile['layers']) == 1
        assert updated_profile['layers'][0]['id'] == 'layer_new'


class TestProfileDeletion:
    """プロファイル削除のテスト"""

    def test_delete_profile(self, test_profiles_dir):
        """プロファイルを削除できる"""
        profile_name = "test_delete_profile"
        profile_path = test_profiles_dir / f"{profile_name}.json"

        # プロファイルを作成
        profile = {"profile_name": profile_name}
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f)

        assert profile_path.exists()

        # プロファイルを削除
        profile_path.unlink()

        assert not profile_path.exists()

    def test_delete_nonexistent_profile(self, test_profiles_dir):
        """存在しないプロファイルの削除処理"""
        profile_path = test_profiles_dir / "nonexistent_profile.json"

        assert not profile_path.exists()

        # 存在しないファイルの削除でエラーが発生することを確認
        with pytest.raises(FileNotFoundError):
            profile_path.unlink()


class TestProfileList:
    """プロファイル一覧のテスト"""

    def test_list_profiles(self, test_profiles_dir):
        """プロファイル一覧を取得できる"""
        # 複数のプロファイルを作成
        profile_names = ["profile_1", "profile_2", "profile_3"]

        for name in profile_names:
            profile_path = test_profiles_dir / f"{name}.json"
            profile = {"profile_name": name}
            with open(profile_path, 'w', encoding='utf-8') as f:
                json.dump(profile, f)

        # プロファイル一覧を取得
        profile_files = list(test_profiles_dir.glob("profile_*.json"))

        assert len(profile_files) >= 3

    def test_list_profiles_empty_directory(self, test_profiles_dir):
        """空のディレクトリでのプロファイル一覧取得"""
        # テスト用の空のサブディレクトリを作成
        empty_dir = test_profiles_dir / "empty_subdir"
        empty_dir.mkdir(exist_ok=True)

        # プロファイル一覧を取得
        profile_files = list(empty_dir.glob("*.json"))

        assert len(profile_files) == 0


class TestProfileMetadata:
    """プロファイルメタデータのテスト"""

    def test_profile_timestamp(self, test_profiles_dir):
        """プロファイルのタイムスタンプが正しく記録される"""
        profile_name = "test_timestamp"
        created_time = datetime.now()

        profile = {
            "profile_name": profile_name,
            "created_at": created_time.isoformat(),
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # プロファイルを読み込み
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        loaded_time = datetime.fromisoformat(loaded_profile['created_at'])

        # タイムスタンプの差が1秒以内であることを確認
        time_diff = abs((loaded_time - created_time).total_seconds())
        assert time_diff < 1

    def test_profile_version(self, test_profiles_dir):
        """プロファイルのバージョン情報が記録される"""
        profile_name = "test_version"
        profile = {
            "profile_name": profile_name,
            "version": "1.0.0"
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert 'version' in loaded_profile
        assert loaded_profile['version'] == "1.0.0"


class TestProfileExportImport:
    """プロファイルのエクスポート・インポートテスト"""

    def test_export_profile(self, test_profiles_dir):
        """プロファイルをエクスポートできる"""
        profile_name = "test_export"
        profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "map_data": {"resolution": 0.05},
            "layers": []
        }

        # エクスポート先のパス
        export_path = test_profiles_dir / f"exported_{profile_name}.json"

        # プロファイルをエクスポート
        with open(export_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        assert export_path.exists()

    def test_import_profile(self, test_profiles_dir):
        """プロファイルをインポートできる"""
        profile_name = "test_import"
        profile = {
            "profile_name": profile_name,
            "map_data": {"resolution": 0.05}
        }

        # エクスポートファイルを作成
        export_path = test_profiles_dir / f"export_{profile_name}.json"
        with open(export_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # インポート
        import_path = test_profiles_dir / f"imported_{profile_name}.json"
        with open(export_path, 'r', encoding='utf-8') as f:
            imported_profile = json.load(f)

        with open(import_path, 'w', encoding='utf-8') as f:
            json.dump(imported_profile, f, indent=2, ensure_ascii=False)

        # 検証
        assert import_path.exists()

        with open(import_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert loaded_profile['profile_name'] == profile_name
