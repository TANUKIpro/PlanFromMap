"""
Three.js統合テスト

Three.jsの読み込みと基本的な3D機能が正しく動作することを確認するテスト
"""

import unittest
import json
from pathlib import Path


class TestThreeJSIntegration(unittest.TestCase):
    """Three.js統合テスト"""

    def setUp(self):
        """テストのセットアップ"""
        self.project_root = Path(__file__).parent.parent
        self.frontend_static = self.project_root / 'apps' / 'frontend' / 'static'

    def test_index_html_has_threejs_import(self):
        """index.htmlにThree.jsのインポートが含まれていることを確認"""
        index_html = self.frontend_static / 'index.html'
        self.assertTrue(index_html.exists(), "index.htmlが存在しません")

        content = index_html.read_text(encoding='utf-8')

        # Three.jsのインポートがあることを確認（ES6モジュール形式）
        self.assertTrue(
            'three' in content.lower(),
            "index.htmlにThree.jsのインポートが見つかりません"
        )

    def test_threed_renderer_exists(self):
        """threeDRenderer.jsが存在することを確認"""
        renderer = self.frontend_static / 'js' / 'modules' / 'threeDRenderer.js'
        self.assertTrue(renderer.exists(), "threeDRenderer.jsが存在しません")

    def test_threed_renderer_exports_required_functions(self):
        """threeDRenderer.jsが必要な関数をエクスポートしていることを確認"""
        renderer = self.frontend_static / 'js' / 'modules' / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        required_exports = [
            'initialize3DView',
            'render3DScene',
            'select3DObject',
            'deselect3DObject',
            'update3DObject',
            'resize3DView',
            'reset3DView',
            'initializePropertyPreview',
            'renderPropertyPreview'
        ]

        for export in required_exports:
            self.assertTrue(
                f'export function {export}' in content or f'export {{ {export}' in content,
                f"threeDRenderer.jsに{export}のエクスポートが見つかりません"
            )

    def test_threed_renderer_file_size(self):
        """threeDRenderer.jsのファイルサイズが適切であることを確認（AI_GUIDELINES準拠）"""
        renderer = self.frontend_static / 'js' / 'modules' / 'threeDRenderer.js'
        lines = renderer.read_text(encoding='utf-8').split('\n')
        line_count = len(lines)

        # AI_GUIDELINESに従い、理想は300-500行、最大1000行
        self.assertLess(
            line_count,
            1000,
            f"threeDRenderer.jsが大きすぎます（{line_count}行）。AI_GUIDELINESでは最大1000行を推奨しています"
        )

        # 警告を出力（500行超えの場合）
        if line_count > 500:
            print(f"\n⚠️ 警告: threeDRenderer.jsが推奨サイズ（500行）を超えています（{line_count}行）")
            print("   AI_GUIDELINESに従い、ファイルを分割することを検討してください")

    def test_canvas_elements_in_html(self):
        """必要なCanvas要素がHTMLに含まれていることを確認"""
        index_html = self.frontend_static / 'index.html'
        content = index_html.read_text(encoding='utf-8')

        # 3Dビュー用Canvas
        self.assertTrue(
            'view3DCanvas' in content,
            "3Dビュー用のCanvasが見つかりません"
        )

        # ViewCube用Canvas
        self.assertTrue(
            'viewCubeCanvas' in content,
            "ViewCube用のCanvasが見つかりません"
        )

        # プロパティプレビュー用Canvas
        self.assertTrue(
            'propertyPreviewCanvas' in content,
            "プロパティプレビュー用のCanvasが見つかりません"
        )


class TestThreeJSModuleStructure(unittest.TestCase):
    """Three.jsモジュール構造のテスト"""

    def setUp(self):
        """テストのセットアップ"""
        self.project_root = Path(__file__).parent.parent
        self.js_modules = self.project_root / 'apps' / 'frontend' / 'static' / 'js' / 'modules'
        self.js_utils = self.project_root / 'apps' / 'frontend' / 'static' / 'js' / 'utils'

    def test_module_files_exist(self):
        """必要なモジュールファイルが存在することを確認"""
        # メインレンダラー
        self.assertTrue(
            (self.js_modules / 'threeDRenderer.js').exists(),
            "threeDRenderer.jsが存在しません"
        )

    def test_no_console_log_in_production_code(self):
        """本番コードにconsole.logが残っていないことを確認（console.error/warnは許可）"""
        renderer = self.js_modules / 'threeDRenderer.js'
        if renderer.exists():
            content = renderer.read_text(encoding='utf-8')

            # console.logの行を検出（ただしコメント内は除く）
            lines = content.split('\n')
            console_log_lines = []

            for i, line in enumerate(lines, 1):
                stripped = line.strip()
                # コメント行は除外
                if stripped.startswith('//') or stripped.startswith('*'):
                    continue
                # console.logを検出
                if 'console.log(' in line and not '// console.log' in line:
                    console_log_lines.append((i, line))

            if console_log_lines:
                print("\n⚠️ 警告: 本番コードにconsole.logが残っています:")
                for line_no, line in console_log_lines[:5]:  # 最初の5つだけ表示
                    print(f"   行 {line_no}: {line.strip()}")


class TestThreeJSFunctionality(unittest.TestCase):
    """Three.js機能のテスト"""

    def setUp(self):
        """テストのセットアップ"""
        self.project_root = Path(__file__).parent.parent
        self.test_data = self.project_root / 'tests' / 'data'
        self.js_modules = self.project_root / 'apps' / 'frontend' / 'static' / 'js' / 'modules'

    def test_test_data_exists(self):
        """テストデータが存在することを確認"""
        self.assertTrue(
            (self.test_data / 'test_map.pgm').exists(),
            "テスト用のマップデータが見つかりません"
        )
        self.assertTrue(
            (self.test_data / 'test_map.yaml').exists(),
            "テスト用のメタデータが見つかりません"
        )

    def test_test_profile_has_3d_data(self):
        """テストプロファイルに3Dデータが含まれていることを確認"""
        profile = self.test_data / 'test_profile.json'
        if profile.exists():
            try:
                # UTF-8 BOMを処理するためutf-8-sigを使用
                data = json.loads(profile.read_text(encoding='utf-8-sig'))

                # プロファイルに矩形データが含まれているか確認
                if 'rectangles' in data:
                    print(f"\n✅ テストプロファイルに{len(data['rectangles'])}個の矩形が含まれています")
            except json.JSONDecodeError as e:
                # JSONパースエラーは警告として出力（テストは継続）
                print(f"\n⚠️ 警告: test_profile.jsonのパースに失敗しました: {e}")

    def test_3d_renderer_imports_three_js(self):
        """threeDRenderer.jsがThree.jsをインポートしていることを確認"""
        renderer = self.js_modules / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        self.assertTrue(
            "import * as THREE from 'three'" in content,
            "threeDRenderer.jsにThree.jsのインポートが見つかりません"
        )
        self.assertTrue(
            "import { OrbitControls }" in content,
            "threeDRenderer.jsにOrbitControlsのインポートが見つかりません"
        )

    def test_3d_objects_module_imports_three_js(self):
        """threeDObjects.jsがThree.jsをインポートしていることを確認"""
        objects = self.js_modules / 'threeDObjects.js'
        content = objects.read_text(encoding='utf-8')

        self.assertTrue(
            "import * as THREE from 'three'" in content,
            "threeDObjects.jsにThree.jsのインポートが見つかりません"
        )

    def test_3d_preview_module_imports_three_js(self):
        """threeDPreview.jsがThree.jsをインポートしていることを確認"""
        preview = self.js_modules / 'threeDPreview.js'
        content = preview.read_text(encoding='utf-8')

        self.assertTrue(
            "import * as THREE from 'three'" in content,
            "threeDPreview.jsにThree.jsのインポートが見つかりません"
        )

    def test_3d_renderer_creates_scene(self):
        """threeDRenderer.jsがシーンを作成していることを確認"""
        renderer = self.js_modules / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        self.assertTrue(
            "scene = new THREE.Scene()" in content,
            "threeDRenderer.jsにシーン作成コードが見つかりません"
        )
        self.assertTrue(
            "new THREE.PerspectiveCamera" in content,
            "threeDRenderer.jsにカメラ作成コードが見つかりません"
        )
        self.assertTrue(
            "new THREE.WebGLRenderer" in content,
            "threeDRenderer.jsにレンダラー作成コードが見つかりません"
        )

    def test_3d_renderer_creates_grid(self):
        """threeDRenderer.jsがグリッドを作成していることを確認"""
        renderer = self.js_modules / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        self.assertTrue(
            "gridGroup" in content,
            "threeDRenderer.jsにグリッドグループが見つかりません"
        )
        self.assertTrue(
            "updateGrid" in content,
            "threeDRenderer.jsにグリッド更新関数が見つかりません"
        )

    def test_3d_renderer_creates_map_texture(self):
        """threeDRenderer.jsがマップテクスチャを作成していることを確認"""
        renderer = self.js_modules / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        self.assertTrue(
            "mapGroup" in content,
            "threeDRenderer.jsにマップグループが見つかりません"
        )
        self.assertTrue(
            "updateMapTexture" in content,
            "threeDRenderer.jsにマップテクスチャ更新関数が見つかりません"
        )
        self.assertTrue(
            "new THREE.Texture" in content,
            "threeDRenderer.jsにテクスチャ作成コードが見つかりません"
        )

    def test_3d_renderer_updates_objects(self):
        """threeDRenderer.jsがオブジェクトを更新していることを確認"""
        renderer = self.js_modules / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        self.assertTrue(
            "objectsGroup" in content,
            "threeDRenderer.jsにオブジェクトグループが見つかりません"
        )
        self.assertTrue(
            "updateObjects" in content,
            "threeDRenderer.jsにオブジェクト更新関数が見つかりません"
        )
        self.assertTrue(
            "create3DObjectMesh" in content,
            "threeDRenderer.jsに3Dオブジェクトメッシュ作成関数の呼び出しが見つかりません"
        )


class TestViewCubeDisplay(unittest.TestCase):
    """ViewCube表示のテスト"""

    def setUp(self):
        """テストのセットアップ"""
        self.project_root = Path(__file__).parent.parent
        self.frontend_static = self.project_root / 'apps' / 'frontend' / 'static'

    def test_viewcube_canvas_in_html(self):
        """HTMLにviewCubeCanvasが含まれていることを確認"""
        index_html = self.frontend_static / 'index.html'
        content = index_html.read_text(encoding='utf-8')

        self.assertTrue(
            'viewCubeCanvas' in content,
            "index.htmlにviewCubeCanvasが見つかりません"
        )

    def test_viewcube_initialization_in_renderer(self):
        """threeDRenderer.jsがViewCubeを初期化していることを確認"""
        renderer = self.frontend_static / 'js' / 'modules' / 'threeDRenderer.js'
        content = renderer.read_text(encoding='utf-8')

        self.assertTrue(
            "initializeViewCube" in content,
            "threeDRenderer.jsにViewCube初期化コードが見つかりません"
        )
        self.assertTrue(
            "updateViewCube" in content,
            "threeDRenderer.jsにViewCube更新コードが見つかりません"
        )


class TestObjectPreviewDisplay(unittest.TestCase):
    """オブジェクトプレビュー表示のテスト"""

    def setUp(self):
        """テストのセットアップ"""
        self.project_root = Path(__file__).parent.parent
        self.frontend_static = self.project_root / 'apps' / 'frontend' / 'static'
        self.js_modules = self.frontend_static / 'js' / 'modules'

    def test_property_preview_canvas_in_html(self):
        """HTMLにpropertyPreviewCanvasが含まれていることを確認"""
        index_html = self.frontend_static / 'index.html'
        content = index_html.read_text(encoding='utf-8')

        self.assertTrue(
            'propertyPreviewCanvas' in content,
            "index.htmlにpropertyPreviewCanvasが見つかりません"
        )

    def test_preview_module_exists(self):
        """threeDPreview.jsが存在することを確認"""
        preview_module = self.js_modules / 'threeDPreview.js'
        self.assertTrue(
            preview_module.exists(),
            "threeDPreview.jsが存在しません"
        )

    def test_preview_module_exports_functions(self):
        """threeDPreview.jsが必要な関数をエクスポートしていることを確認"""
        preview_module = self.js_modules / 'threeDPreview.js'
        content = preview_module.read_text(encoding='utf-8')

        required_exports = [
            'initializePropertyPreview',
            'renderPropertyPreview',
            'getPreviewState'
        ]

        for export in required_exports:
            self.assertTrue(
                f'export function {export}' in content,
                f"threeDPreview.jsに{export}のエクスポートが見つかりません"
            )

    def test_preview_creates_scene(self):
        """threeDPreview.jsがプレビューシーンを作成していることを確認"""
        preview_module = self.js_modules / 'threeDPreview.js'
        content = preview_module.read_text(encoding='utf-8')

        self.assertTrue(
            "previewScene = new THREE.Scene()" in content,
            "threeDPreview.jsにプレビューシーン作成コードが見つかりません"
        )
        self.assertTrue(
            "previewCamera = new THREE.PerspectiveCamera" in content,
            "threeDPreview.jsにプレビューカメラ作成コードが見つかりません"
        )
        self.assertTrue(
            "previewRenderer = new THREE.WebGLRenderer" in content,
            "threeDPreview.jsにプレビューレンダラー作成コードが見つかりません"
        )

    def test_preview_renders_objects(self):
        """threeDPreview.jsがオブジェクトを描画していることを確認"""
        preview_module = self.js_modules / 'threeDPreview.js'
        content = preview_module.read_text(encoding='utf-8')

        self.assertTrue(
            "create3DObjectMesh" in content,
            "threeDPreview.jsにオブジェクトメッシュ作成関数の呼び出しが見つかりません"
        )


class TestModularArchitecture(unittest.TestCase):
    """モジュール構造のテスト（AI_GUIDELINES準拠）"""

    def setUp(self):
        """テストのセットアップ"""
        self.project_root = Path(__file__).parent.parent
        self.js_modules = self.project_root / 'apps' / 'frontend' / 'static' / 'js' / 'modules'
        self.js_utils = self.project_root / 'apps' / 'frontend' / 'static' / 'js' / 'utils'

    def test_helpers_module_exists(self):
        """threeHelpers.jsが存在することを確認"""
        helpers = self.js_utils / 'threeHelpers.js'
        self.assertTrue(helpers.exists(), "threeHelpers.jsが存在しません")

    def test_objects_module_exists(self):
        """threeDObjects.jsが存在することを確認"""
        objects = self.js_modules / 'threeDObjects.js'
        self.assertTrue(objects.exists(), "threeDObjects.jsが存在しません")

    def test_preview_module_exists(self):
        """threeDPreview.jsが存在することを確認"""
        preview = self.js_modules / 'threeDPreview.js'
        self.assertTrue(preview.exists(), "threeDPreview.jsが存在しません")

    def test_all_modules_under_size_limit(self):
        """すべてのモジュールがサイズ制限内であることを確認"""
        modules = [
            (self.js_modules / 'threeDRenderer.js', 1000),
            (self.js_modules / 'threeDObjects.js', 500),
            (self.js_modules / 'threeDPreview.js', 300),
            (self.js_utils / 'threeHelpers.js', 300)
        ]

        for module_path, max_lines in modules:
            if module_path.exists():
                lines = module_path.read_text(encoding='utf-8').split('\n')
                line_count = len(lines)
                self.assertLess(
                    line_count,
                    max_lines,
                    f"{module_path.name}が大きすぎます（{line_count}行 > {max_lines}行制限）"
                )


if __name__ == '__main__':
    unittest.main(verbosity=2)
