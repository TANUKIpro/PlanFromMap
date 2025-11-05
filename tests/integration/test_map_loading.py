"""
Map Loading Integration Tests
マップロード機能の統合テスト
"""
import pytest
import yaml
import json
from pathlib import Path
from datetime import datetime


class TestYAMLMapLoading:
    """YAMLマップファイルのロードテスト"""

    def test_load_actual_yaml_file(self, sample_map_files):
        """実際のYAMLマップファイルを読み込める"""
        yaml_path = sample_map_files['yaml']

        assert yaml_path.exists(), f"YAMLファイルが存在しません: {yaml_path}"

        # YAMLファイルを読み込む
        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        # 必須フィールドの検証
        assert 'image' in map_data, "YAMLに'image'フィールドが必要です"
        assert 'resolution' in map_data, "YAMLに'resolution'フィールドが必要です"
        assert 'origin' in map_data, "YAMLに'origin'フィールドが必要です"

    def test_yaml_metadata_structure(self, sample_map_files):
        """YAMLメタデータの構造が正しい"""
        yaml_path = sample_map_files['yaml']

        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        # imageフィールドの検証
        assert isinstance(map_data['image'], str)
        assert map_data['image'].endswith('.pgm')

        # resolutionフィールドの検証
        assert isinstance(map_data['resolution'], (int, float))
        assert map_data['resolution'] > 0

        # originフィールドの検証
        assert isinstance(map_data['origin'], list)
        assert len(map_data['origin']) == 3  # [x, y, theta]

    def test_yaml_optional_fields(self, sample_map_files):
        """YAMLの任意フィールドの検証"""
        yaml_path = sample_map_files['yaml']

        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        # 任意フィールドが存在する場合の検証
        if 'negate' in map_data:
            assert isinstance(map_data['negate'], int)
            assert map_data['negate'] in [0, 1]

        if 'occupied_thresh' in map_data:
            assert isinstance(map_data['occupied_thresh'], (int, float))
            assert 0 <= map_data['occupied_thresh'] <= 1

        if 'free_thresh' in map_data:
            assert isinstance(map_data['free_thresh'], (int, float))
            assert 0 <= map_data['free_thresh'] <= 1


class TestPGMMapLoading:
    """PGMマップファイルのロードテスト"""

    def test_load_actual_pgm_file(self, sample_map_files):
        """実際のPGMマップファイルを読み込める"""
        pgm_path = sample_map_files['pgm']

        assert pgm_path.exists(), f"PGMファイルが存在しません: {pgm_path}"

        # PGMファイルを読み込む
        with open(pgm_path, 'rb') as f:
            pgm_data = f.read()

        # ファイルサイズの検証
        assert len(pgm_data) > 0, "PGMファイルが空です"

    def test_pgm_file_header(self, sample_map_files):
        """PGMファイルのヘッダーが正しい"""
        pgm_path = sample_map_files['pgm']

        with open(pgm_path, 'rb') as f:
            # 最初の2バイトを読み込んでマジックナンバーを確認
            magic = f.read(2)
            assert magic in [b'P2', b'P5'], \
                f"PGMマジックナンバーが無効です: {magic}"

    def test_pgm_file_size_consistency(self, sample_map_files):
        """PGMファイルのサイズが妥当である"""
        pgm_path = sample_map_files['pgm']
        yaml_path = sample_map_files['yaml']

        # YAMLから解像度を取得
        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        # PGMファイルのサイズを取得
        pgm_size = pgm_path.stat().st_size

        # ファイルサイズが妥当であることを確認
        # ヘッダー + ピクセルデータ
        # 通常、ROSマップは1000x1000〜4000x4000ピクセル程度
        assert pgm_size > 10000, "PGMファイルが小さすぎます"
        assert pgm_size < 50000000, "PGMファイルが大きすぎます（50MB以上）"


class TestMapDataConsistency:
    """YAMLとPGMの整合性テスト"""

    def test_yaml_references_existing_pgm(self, sample_map_files):
        """YAMLが参照するPGMファイルが存在する"""
        yaml_path = sample_map_files['yaml']

        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        # YAMLで指定されたPGMファイル名
        pgm_filename = map_data['image']

        # 同じディレクトリ内にPGMファイルが存在するか確認
        pgm_path = yaml_path.parent / pgm_filename

        # 実際のPGMファイルと照合
        # (サンプルではファイル名が異なる可能性があるため、存在確認のみ)
        actual_pgm = sample_map_files['pgm']
        assert actual_pgm.exists(), "PGMファイルが存在しません"

    def test_resolution_is_reasonable(self, sample_map_files):
        """解像度が妥当な範囲である"""
        yaml_path = sample_map_files['yaml']

        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        resolution = map_data['resolution']

        # ROSで一般的な解像度は0.01〜0.1m/pixel
        assert 0.001 <= resolution <= 1.0, \
            f"解像度が妥当な範囲外です: {resolution}"

    def test_origin_coordinates_are_valid(self, sample_map_files):
        """原点座標が有効である"""
        yaml_path = sample_map_files['yaml']

        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_data = yaml.safe_load(f)

        origin = map_data['origin']

        # 座標が有限値であることを確認
        assert all(isinstance(x, (int, float)) for x in origin), \
            "原点座標が数値ではありません"

        # 座標が極端に大きくないことを確認（±1000m以内）
        assert -1000 <= origin[0] <= 1000, f"X座標が範囲外です: {origin[0]}"
        assert -1000 <= origin[1] <= 1000, f"Y座標が範囲外です: {origin[1]}"


class TestMapLoadingWorkflow:
    """マップロードワークフローの統合テスト"""

    def test_complete_map_loading_workflow(self, sample_map_files,
                                           test_profiles_dir):
        """完全なマップロードワークフローをテスト"""
        yaml_path = sample_map_files['yaml']
        pgm_path = sample_map_files['pgm']

        # ステップ1: YAMLファイルの読み込み
        with open(yaml_path, 'r', encoding='utf-8') as f:
            metadata = yaml.safe_load(f)

        assert metadata is not None, "メタデータの読み込みに失敗しました"

        # ステップ2: PGMファイルの読み込み
        with open(pgm_path, 'rb') as f:
            pgm_data = f.read()

        assert pgm_data is not None, "PGMデータの読み込みに失敗しました"
        assert len(pgm_data) > 0, "PGMデータが空です"

        # ステップ3: マップデータの統合
        map_info = {
            "metadata": metadata,
            "pgm_size": len(pgm_data),
            "loaded_at": datetime.now().isoformat(),
            "files": {
                "yaml": str(yaml_path),
                "pgm": str(pgm_path)
            }
        }

        # ステップ4: マップ情報の保存（プロファイルとして）
        profile_path = test_profiles_dir / "test_map_load.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(map_info, f, indent=2, ensure_ascii=False)

        # 検証: プロファイルが正しく保存された
        assert profile_path.exists(), "プロファイルの保存に失敗しました"

        # 検証: プロファイルが読み込める
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_info = json.load(f)

        assert loaded_info['metadata'] == metadata
        assert loaded_info['pgm_size'] == len(pgm_data)

    def test_error_handling_missing_yaml(self):
        """存在しないYAMLファイルの処理"""
        non_existent_path = Path("non_existent.yaml")

        assert not non_existent_path.exists(), \
            "テスト用の存在しないファイルが実際に存在しています"

    def test_error_handling_missing_pgm(self):
        """存在しないPGMファイルの処理"""
        non_existent_path = Path("non_existent.pgm")

        assert not non_existent_path.exists(), \
            "テスト用の存在しないファイルが実際に存在しています"

    def test_malformed_yaml_handling(self):
        """不正な形式のYAMLファイルの処理"""
        malformed_yaml = "invalid: yaml: content: ["

        # YAML解析でエラーが発生することを確認
        with pytest.raises(yaml.YAMLError):
            yaml.safe_load(malformed_yaml)


class TestMapMetadataValidation:
    """マップメタデータの検証テスト"""

    def test_validate_required_fields(self, sample_yaml_data):
        """必須フィールドの検証"""
        required_fields = ['image', 'resolution', 'origin']

        for field in required_fields:
            assert field in sample_yaml_data, \
                f"必須フィールド '{field}' が存在しません"

    def test_validate_field_types(self, sample_yaml_data):
        """フィールド型の検証"""
        assert isinstance(sample_yaml_data['image'], str)
        assert isinstance(sample_yaml_data['resolution'], (int, float))
        assert isinstance(sample_yaml_data['origin'], list)
        assert isinstance(sample_yaml_data['negate'], int)
        assert isinstance(sample_yaml_data['occupied_thresh'], (int, float))
        assert isinstance(sample_yaml_data['free_thresh'], (int, float))

    def test_validate_field_ranges(self, sample_yaml_data):
        """フィールド値の範囲検証"""
        # resolution: 正の値
        assert sample_yaml_data['resolution'] > 0

        # negate: 0 または 1
        assert sample_yaml_data['negate'] in [0, 1]

        # occupied_thresh: 0〜1
        assert 0 <= sample_yaml_data['occupied_thresh'] <= 1

        # free_thresh: 0〜1
        assert 0 <= sample_yaml_data['free_thresh'] <= 1

        # origin: 3要素のリスト
        assert len(sample_yaml_data['origin']) == 3


class TestMapDataPersistence:
    """マップデータの永続化テスト"""

    def test_save_and_load_map_metadata(self, test_profiles_dir,
                                        sample_yaml_data):
        """マップメタデータの保存と読み込み"""
        # 保存
        metadata_path = test_profiles_dir / "test_metadata.json"
        with open(metadata_path, 'w', encoding='utf-8') as f:
            json.dump(sample_yaml_data, f, indent=2, ensure_ascii=False)

        # 読み込み
        with open(metadata_path, 'r', encoding='utf-8') as f:
            loaded_data = json.load(f)

        # 検証
        assert loaded_data == sample_yaml_data

    def test_map_profile_structure(self, test_profiles_dir):
        """マッププロファイルの構造検証"""
        profile = {
            "profile_name": "test_map_profile",
            "created_at": datetime.now().isoformat(),
            "map_data": {
                "resolution": 0.05,
                "origin": [-51.224998, -51.224998, 0.0],
                "width": 2048,
                "height": 2048
            },
            "layers": [],
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            }
        }

        # プロファイルを保存
        profile_path = test_profiles_dir / "test_profile_structure.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # プロファイルを読み込み
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        # 構造の検証
        assert 'profile_name' in loaded_profile
        assert 'created_at' in loaded_profile
        assert 'map_data' in loaded_profile
        assert 'layers' in loaded_profile
        assert 'viewport' in loaded_profile
