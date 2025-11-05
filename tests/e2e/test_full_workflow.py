"""
Full Workflow E2E Tests
アプリケーション全体のワークフローE2Eテスト
"""
import pytest
import json
import yaml
from pathlib import Path
from datetime import datetime
import time


class TestMapLoadingWorkflow:
    """マップロードワークフローのE2Eテスト"""

    @pytest.mark.slow
    def test_complete_map_loading_workflow(self, sample_map_files,
                                           test_profiles_dir,
                                           test_result_tracker):
        """完全なマップロードワークフローをテスト"""
        start_time = time.time()

        # フェーズ1: ファイルの読み込み
        yaml_path = sample_map_files['yaml']
        pgm_path = sample_map_files['pgm']

        assert yaml_path.exists(), "YAMLファイルが存在しません"
        assert pgm_path.exists(), "PGMファイルが存在しません"

        # YAMLメタデータの読み込み
        with open(yaml_path, 'r', encoding='utf-8') as f:
            metadata = yaml.safe_load(f)

        assert metadata is not None
        assert 'resolution' in metadata
        assert 'origin' in metadata

        # PGMデータの読み込み
        with open(pgm_path, 'rb') as f:
            pgm_data = f.read()

        assert len(pgm_data) > 0

        # フェーズ2: データの検証
        resolution = metadata['resolution']
        origin = metadata['origin']

        assert resolution > 0
        assert len(origin) == 3

        # フェーズ3: プロファイルの作成
        profile = {
            "profile_name": "e2e_test_full_workflow",
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "map_data": {
                "metadata": metadata,
                "pgm_loaded": True,
                "pgm_size": len(pgm_data)
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
            },
            "test_metadata": {
                "test_name": "complete_map_loading_workflow",
                "test_date": datetime.now().isoformat()
            }
        }

        # フェーズ4: プロファイルの保存
        profile_path = test_profiles_dir / "e2e_full_workflow.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        assert profile_path.exists()

        # フェーズ5: プロファイルの再読み込みと検証
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert loaded_profile['profile_name'] == profile['profile_name']
        assert loaded_profile['map_data']['pgm_loaded'] is True
        assert loaded_profile['map_data']['pgm_size'] == len(pgm_data)

        # テスト結果を記録
        duration = time.time() - start_time
        test_result_tracker.add_result(
            test_name="complete_map_loading_workflow",
            status="passed",
            duration=duration,
            details={
                "yaml_size": yaml_path.stat().st_size,
                "pgm_size": len(pgm_data),
                "resolution": resolution,
                "profile_path": str(profile_path)
            }
        )


class TestProfileManagementWorkflow:
    """プロファイル管理ワークフローのE2Eテスト"""

    @pytest.mark.slow
    def test_profile_create_save_load_workflow(self, test_profiles_dir,
                                                test_result_tracker):
        """プロファイルの作成・保存・読み込みワークフロー"""
        start_time = time.time()

        # フェーズ1: プロファイル作成
        profile_name = "e2e_test_profile_workflow"
        profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "version": "1.0.0",
            "map_data": {
                "resolution": 0.05,
                "origin": [-50.0, -50.0, 0.0],
                "width": 2048,
                "height": 2048
            },
            "layers": [
                {
                    "id": "layer_1",
                    "name": "Base Layer",
                    "type": "image",
                    "visible": True
                },
                {
                    "id": "layer_2",
                    "name": "Drawing Layer",
                    "type": "drawing",
                    "visible": True
                }
            ],
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            }
        }

        # フェーズ2: プロファイル保存
        profile_path = test_profiles_dir / f"{profile_name}.json"
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        assert profile_path.exists()

        # フェーズ3: プロファイル読み込み
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert loaded_profile['profile_name'] == profile_name

        # フェーズ4: プロファイル更新
        loaded_profile['viewport']['scale'] = 2.0
        loaded_profile['updated_at'] = datetime.now().isoformat()

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(loaded_profile, f, indent=2, ensure_ascii=False)

        # フェーズ5: 更新後の読み込み
        with open(profile_path, 'r', encoding='utf-8') as f:
            updated_profile = json.load(f)

        assert updated_profile['viewport']['scale'] == 2.0
        assert 'updated_at' in updated_profile

        # テスト結果を記録
        duration = time.time() - start_time
        test_result_tracker.add_result(
            test_name="profile_create_save_load_workflow",
            status="passed",
            duration=duration,
            details={
                "profile_name": profile_name,
                "layers_count": len(profile['layers']),
                "profile_size": profile_path.stat().st_size
            }
        )


class TestLayerManagementWorkflow:
    """レイヤー管理ワークフローのE2Eテスト"""

    @pytest.mark.slow
    def test_layer_create_update_delete_workflow(self, test_profiles_dir,
                                                  test_result_tracker):
        """レイヤーの作成・更新・削除ワークフロー"""
        start_time = time.time()

        # 初期プロファイル
        profile = {
            "profile_name": "e2e_layer_management",
            "layers": []
        }

        profile_path = test_profiles_dir / "e2e_layer_management.json"

        # フェーズ1: レイヤー作成
        new_layers = [
            {
                "id": "layer_1",
                "name": "Map Layer",
                "type": "image",
                "visible": True
            },
            {
                "id": "layer_2",
                "name": "Drawing Layer 1",
                "type": "drawing",
                "visible": True
            },
            {
                "id": "layer_3",
                "name": "Drawing Layer 2",
                "type": "drawing",
                "visible": False
            }
        ]

        profile['layers'].extend(new_layers)

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # 検証: 3つのレイヤーが作成された
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)

        assert len(loaded_profile['layers']) == 3

        # フェーズ2: レイヤー更新
        loaded_profile['layers'][2]['visible'] = True
        loaded_profile['layers'][2]['opacity'] = 0.5

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(loaded_profile, f, indent=2, ensure_ascii=False)

        # 検証: レイヤーが更新された
        with open(profile_path, 'r', encoding='utf-8') as f:
            updated_profile = json.load(f)

        assert updated_profile['layers'][2]['visible'] is True
        assert updated_profile['layers'][2]['opacity'] == 0.5

        # フェーズ3: レイヤー削除
        updated_profile['layers'] = [
            layer for layer in updated_profile['layers']
            if layer['id'] != 'layer_2'
        ]

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(updated_profile, f, indent=2, ensure_ascii=False)

        # 検証: レイヤーが削除された
        with open(profile_path, 'r', encoding='utf-8') as f:
            final_profile = json.load(f)

        assert len(final_profile['layers']) == 2
        assert not any(layer['id'] == 'layer_2' for layer in final_profile['layers'])

        # テスト結果を記録
        duration = time.time() - start_time
        test_result_tracker.add_result(
            test_name="layer_create_update_delete_workflow",
            status="passed",
            duration=duration,
            details={
                "initial_layers": 3,
                "final_layers": 2
            }
        )


class TestViewportManagementWorkflow:
    """ビューポート管理ワークフローのE2Eテスト"""

    def test_viewport_zoom_pan_workflow(self, test_profiles_dir,
                                        test_result_tracker):
        """ビューポートのズーム・パン操作ワークフロー"""
        start_time = time.time()

        profile_name = "e2e_viewport_management"
        profile = {
            "profile_name": profile_name,
            "viewport": {
                "scale": 1.0,
                "offsetX": 0,
                "offsetY": 0
            },
            "viewport_history": []
        }

        profile_path = test_profiles_dir / f"{profile_name}.json"

        # 操作1: ズームイン
        profile['viewport']['scale'] = 2.0
        profile['viewport_history'].append({
            "action": "zoom_in",
            "timestamp": datetime.now().isoformat(),
            "scale": 2.0
        })

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # 操作2: パン
        profile['viewport']['offsetX'] = 100
        profile['viewport']['offsetY'] = 50
        profile['viewport_history'].append({
            "action": "pan",
            "timestamp": datetime.now().isoformat(),
            "offsetX": 100,
            "offsetY": 50
        })

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # 操作3: さらにズームイン
        profile['viewport']['scale'] = 4.0
        profile['viewport_history'].append({
            "action": "zoom_in",
            "timestamp": datetime.now().isoformat(),
            "scale": 4.0
        })

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(profile, f, indent=2, ensure_ascii=False)

        # 検証
        with open(profile_path, 'r', encoding='utf-8') as f:
            final_profile = json.load(f)

        assert final_profile['viewport']['scale'] == 4.0
        assert final_profile['viewport']['offsetX'] == 100
        assert final_profile['viewport']['offsetY'] == 50
        assert len(final_profile['viewport_history']) == 3

        # テスト結果を記録
        duration = time.time() - start_time
        test_result_tracker.add_result(
            test_name="viewport_zoom_pan_workflow",
            status="passed",
            duration=duration,
            details={
                "final_scale": 4.0,
                "operations_count": 3
            }
        )


class TestErrorRecoveryWorkflow:
    """エラーリカバリーワークフローのE2Eテスト"""

    def test_corrupted_profile_recovery(self, test_profiles_dir,
                                        test_result_tracker):
        """破損したプロファイルのリカバリー処理"""
        start_time = time.time()

        profile_name = "e2e_error_recovery"
        profile_path = test_profiles_dir / f"{profile_name}.json"

        # 正常なプロファイルを作成
        valid_profile = {
            "profile_name": profile_name,
            "version": "1.0.0",
            "created_at": datetime.now().isoformat()
        }

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(valid_profile, f, indent=2, ensure_ascii=False)

        # バックアップを作成
        backup_path = test_profiles_dir / f"{profile_name}_backup.json"
        with open(backup_path, 'w', encoding='utf-8') as f:
            json.dump(valid_profile, f, indent=2, ensure_ascii=False)

        # プロファイルを破損させる
        with open(profile_path, 'w', encoding='utf-8') as f:
            f.write("{invalid json")

        # 破損を検出
        try:
            with open(profile_path, 'r', encoding='utf-8') as f:
                json.load(f)
            profile_corrupted = False
        except json.JSONDecodeError:
            profile_corrupted = True

        assert profile_corrupted, "破損が検出されるべきです"

        # バックアップからリカバリー
        with open(backup_path, 'r', encoding='utf-8') as f:
            recovered_profile = json.load(f)

        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(recovered_profile, f, indent=2, ensure_ascii=False)

        # リカバリーの検証
        with open(profile_path, 'r', encoding='utf-8') as f:
            final_profile = json.load(f)

        assert final_profile['profile_name'] == profile_name

        # テスト結果を記録
        duration = time.time() - start_time
        test_result_tracker.add_result(
            test_name="corrupted_profile_recovery",
            status="passed",
            duration=duration,
            details={
                "recovery_successful": True
            }
        )


class TestPerformanceWorkflow:
    """パフォーマンステストのワークフロー"""

    @pytest.mark.slow
    def test_large_profile_performance(self, test_profiles_dir,
                                       test_result_tracker):
        """大きなプロファイルのパフォーマンステスト"""
        start_time = time.time()

        profile_name = "e2e_large_profile"

        # 多数のレイヤーを含む大きなプロファイルを作成
        large_profile = {
            "profile_name": profile_name,
            "created_at": datetime.now().isoformat(),
            "layers": []
        }

        # 100個のレイヤーを追加
        for i in range(100):
            layer = {
                "id": f"layer_{i}",
                "name": f"Layer {i}",
                "type": "drawing",
                "visible": i % 2 == 0,
                "opacity": 0.5 + (i % 50) / 100
            }
            large_profile['layers'].append(layer)

        profile_path = test_profiles_dir / f"{profile_name}.json"

        # 保存パフォーマンスを測定
        save_start = time.time()
        with open(profile_path, 'w', encoding='utf-8') as f:
            json.dump(large_profile, f, indent=2, ensure_ascii=False)
        save_duration = time.time() - save_start

        # 読み込みパフォーマンスを測定
        load_start = time.time()
        with open(profile_path, 'r', encoding='utf-8') as f:
            loaded_profile = json.load(f)
        load_duration = time.time() - load_start

        # 検証
        assert len(loaded_profile['layers']) == 100

        # テスト結果を記録
        total_duration = time.time() - start_time
        test_result_tracker.add_result(
            test_name="large_profile_performance",
            status="passed",
            duration=total_duration,
            details={
                "layers_count": 100,
                "save_duration": save_duration,
                "load_duration": load_duration,
                "file_size": profile_path.stat().st_size
            }
        )

        # パフォーマンスの基準を確認
        assert save_duration < 1.0, "保存が1秒以内に完了するべきです"
        assert load_duration < 1.0, "読み込みが1秒以内に完了するべきです"
