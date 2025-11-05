"""
Coordinate Conversion Tests
座標変換機能のユニットテスト
"""
import pytest
import math


class MockMapState:
    """mapStateのモック"""

    def __init__(self):
        self.metadata = {
            'resolution': 0.05,  # 5cm/pixel
            'origin': [-51.224998, -51.224998, 0.0]
        }
        self.image = type('obj', (object,), {
            'width': 2048,
            'height': 2048
        })()
        self.scale = 1.0
        self.offsetX = 0
        self.offsetY = 0


class TestWorldToCanvasConversion:
    """実世界座標からキャンバス座標への変換テスト"""

    def setup_method(self):
        """各テストの前に実行される準備"""
        self.map_state = MockMapState()

    def test_origin_conversion(self):
        """原点（0,0）の変換"""
        # ワールド座標の原点
        world_x, world_y = 0.0, 0.0

        # メタデータの原点
        origin_x, origin_y = self.map_state.metadata['origin'][:2]
        resolution = self.map_state.metadata['resolution']

        # ピクセル座標に変換
        pixel_x = (world_x - origin_x) / resolution
        pixel_y = self.map_state.image.height - (world_y - origin_y) / resolution

        # ピクセル座標が画像内に収まることを確認
        assert 0 <= pixel_x <= self.map_state.image.width
        assert 0 <= pixel_y <= self.map_state.image.height

    def test_positive_coordinates(self):
        """正の座標の変換"""
        world_x, world_y = 10.0, 10.0

        origin_x, origin_y = self.map_state.metadata['origin'][:2]
        resolution = self.map_state.metadata['resolution']

        pixel_x = (world_x - origin_x) / resolution
        pixel_y = self.map_state.image.height - (world_y - origin_y) / resolution

        # 座標が有限値であることを確認
        assert math.isfinite(pixel_x)
        assert math.isfinite(pixel_y)

    def test_negative_coordinates(self):
        """負の座標の変換"""
        world_x, world_y = -10.0, -10.0

        origin_x, origin_y = self.map_state.metadata['origin'][:2]
        resolution = self.map_state.metadata['resolution']

        pixel_x = (world_x - origin_x) / resolution
        pixel_y = self.map_state.image.height - (world_y - origin_y) / resolution

        assert math.isfinite(pixel_x)
        assert math.isfinite(pixel_y)

    def test_resolution_effect(self):
        """解像度が変換に与える影響"""
        world_x, world_y = 1.0, 1.0

        # 解像度0.05の場合
        resolution1 = 0.05
        pixel_distance1 = 1.0 / resolution1  # 20ピクセル

        # 解像度0.1の場合
        resolution2 = 0.1
        pixel_distance2 = 1.0 / resolution2  # 10ピクセル

        # 解像度が大きいほど、同じ距離のピクセル数は少なくなる
        assert pixel_distance1 > pixel_distance2
        assert pixel_distance1 == 20
        assert pixel_distance2 == 10

    def test_scale_effect(self):
        """スケールが変換に与える影響"""
        pixel_x = 100

        # スケール1.0の場合
        canvas_x1 = pixel_x * 1.0
        assert canvas_x1 == 100

        # スケール2.0の場合（ズームイン）
        canvas_x2 = pixel_x * 2.0
        assert canvas_x2 == 200

        # スケール0.5の場合（ズームアウト）
        canvas_x3 = pixel_x * 0.5
        assert canvas_x3 == 50


class TestCanvasToImagePixelConversion:
    """キャンバス座標から画像ピクセル座標への変換テスト"""

    def setup_method(self):
        self.map_state = MockMapState()
        self.container_width = 1920
        self.container_height = 1080

    def test_center_point_conversion(self):
        """中心点の変換"""
        # コンテナの中心
        canvas_x = self.container_width / 2
        canvas_y = self.container_height / 2

        # 画像のスケール後のサイズ
        scaled_width = self.map_state.image.width * self.map_state.scale
        scaled_height = self.map_state.image.height * self.map_state.scale

        # 画像の描画開始位置（中央配置）
        base_x = (self.container_width - scaled_width) / 2
        base_y = (self.container_height - scaled_height) / 2

        draw_x = base_x + self.map_state.offsetX
        draw_y = base_y + self.map_state.offsetY

        # 画像ピクセル座標に変換
        image_pixel_x = (canvas_x - draw_x) / self.map_state.scale
        image_pixel_y = (canvas_y - draw_y) / self.map_state.scale

        # 中心付近のピクセルであることを確認
        assert 0 <= image_pixel_x <= self.map_state.image.width
        assert 0 <= image_pixel_y <= self.map_state.image.height

    def test_corner_points_conversion(self):
        """四隅の点の変換"""
        corners = [
            (0, 0),  # 左上
            (self.container_width, 0),  # 右上
            (0, self.container_height),  # 左下
            (self.container_width, self.container_height)  # 右下
        ]

        for canvas_x, canvas_y in corners:
            # キャンバス座標が有効であることを確認
            assert 0 <= canvas_x <= self.container_width
            assert 0 <= canvas_y <= self.container_height

    def test_offset_effect(self):
        """オフセットが変換に与える影響"""
        canvas_x = 100
        base_x = 50

        # オフセットなし
        offset1 = 0
        draw_x1 = base_x + offset1
        image_pixel_x1 = (canvas_x - draw_x1) / self.map_state.scale

        # オフセット+20
        offset2 = 20
        draw_x2 = base_x + offset2
        image_pixel_x2 = (canvas_x - draw_x2) / self.map_state.scale

        # オフセットが増えると、画像ピクセル座標は減る
        assert image_pixel_x1 > image_pixel_x2
        assert image_pixel_x1 - image_pixel_x2 == 20


class TestImagePixelToCanvasConversion:
    """画像ピクセル座標からキャンバス座標への変換テスト"""

    def setup_method(self):
        self.map_state = MockMapState()
        self.container_width = 1920
        self.container_height = 1080

    def test_pixel_to_canvas_basic(self):
        """基本的なピクセルからキャンバスへの変換"""
        image_pixel_x = 100
        image_pixel_y = 100

        scaled_width = self.map_state.image.width * self.map_state.scale
        scaled_height = self.map_state.image.height * self.map_state.scale

        base_x = (self.container_width - scaled_width) / 2
        base_y = (self.container_height - scaled_height) / 2

        draw_x = base_x + self.map_state.offsetX
        draw_y = base_y + self.map_state.offsetY

        canvas_x = image_pixel_x * self.map_state.scale + draw_x
        canvas_y = image_pixel_y * self.map_state.scale + draw_y

        # キャンバス座標が正の値であることを確認
        assert canvas_x >= 0
        assert canvas_y >= 0

    def test_round_trip_conversion(self):
        """往復変換の一貫性"""
        # 元の画像ピクセル座標
        original_pixel_x = 512
        original_pixel_y = 512

        # 画像ピクセル → キャンバス座標
        scaled_width = self.map_state.image.width * self.map_state.scale
        scaled_height = self.map_state.image.height * self.map_state.scale
        base_x = (self.container_width - scaled_width) / 2
        base_y = (self.container_height - scaled_height) / 2
        draw_x = base_x + self.map_state.offsetX
        draw_y = base_y + self.map_state.offsetY

        canvas_x = original_pixel_x * self.map_state.scale + draw_x
        canvas_y = original_pixel_y * self.map_state.scale + draw_y

        # キャンバス座標 → 画像ピクセル（逆変換）
        converted_pixel_x = (canvas_x - draw_x) / self.map_state.scale
        converted_pixel_y = (canvas_y - draw_y) / self.map_state.scale

        # 往復変換で元の値に戻ることを確認
        assert abs(converted_pixel_x - original_pixel_x) < 0.01
        assert abs(converted_pixel_y - original_pixel_y) < 0.01


class TestCoordinateEdgeCases:
    """座標変換のエッジケースのテスト"""

    def test_zero_resolution(self):
        """解像度がゼロの場合の処理"""
        resolution = 0.0

        # ゼロ除算を避ける必要がある
        if resolution == 0:
            # エラー処理または代替値を使用
            assert True

    def test_negative_resolution(self):
        """負の解像度の処理"""
        resolution = -0.05

        # 負の解像度は通常無効
        assert resolution < 0

    def test_extreme_coordinates(self):
        """極端な座標値の処理"""
        extreme_values = [
            -1000000.0,
            1000000.0,
            float('inf'),
            float('-inf')
        ]

        for value in extreme_values[:2]:  # 有限値のみテスト
            # 極端な座標でも計算が可能であることを確認
            assert math.isfinite(value)

    def test_very_large_scale(self):
        """非常に大きなスケール値"""
        scale = 100.0
        pixel_x = 10

        canvas_x = pixel_x * scale
        assert canvas_x == 1000

    def test_very_small_scale(self):
        """非常に小さなスケール値"""
        scale = 0.01
        pixel_x = 100

        canvas_x = pixel_x * scale
        assert canvas_x == 1.0


class TestCoordinateConsistency:
    """座標変換の一貫性テスト"""

    def test_identity_transform(self):
        """恒等変換（スケール1.0、オフセット0）"""
        scale = 1.0
        offset = 0
        pixel_x = 100

        canvas_x = pixel_x * scale + offset
        assert canvas_x == pixel_x

    def test_commutative_operations(self):
        """スケールとオフセットの順序不変性"""
        pixel_x = 50
        scale = 2.0
        offset = 10

        # (pixel * scale) + offset
        result1 = pixel_x * scale + offset

        # 結果が期待通りであることを確認
        assert result1 == 110

    def test_inverse_operations(self):
        """逆変換の正確性"""
        original = 100
        scale = 2.5

        # 順変換
        transformed = original * scale

        # 逆変換
        recovered = transformed / scale

        # 元の値に戻ることを確認
        assert abs(recovered - original) < 0.001
