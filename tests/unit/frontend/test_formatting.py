"""
Formatting Functions Tests
フォーマット機能のユニットテスト
"""
import pytest
import math


class TestDistanceFormatting:
    """距離フォーマットのテスト"""

    def format_distance(self, meters):
        """距離をフォーマット（メートル単位の入力）"""
        if meters >= 1000:
            return f"{meters / 1000:.2f} km"
        elif meters >= 1:
            return f"{meters:.2f} m"
        elif meters >= 0.01:
            return f"{meters * 100:.2f} cm"
        else:
            return f"{meters * 1000:.2f} mm"

    def test_kilometer_formatting(self):
        """キロメートル単位のフォーマット"""
        assert self.format_distance(1000) == "1.00 km"
        assert self.format_distance(1500) == "1.50 km"
        assert self.format_distance(2500.5) == "2.50 km"  # 2500.5 / 1000 = 2.5005 → 2.50
        assert self.format_distance(10000) == "10.00 km"

    def test_meter_formatting(self):
        """メートル単位のフォーマット"""
        assert self.format_distance(1) == "1.00 m"
        assert self.format_distance(10) == "10.00 m"
        assert self.format_distance(100) == "100.00 m"
        assert self.format_distance(999) == "999.00 m"

    def test_centimeter_formatting(self):
        """センチメートル単位のフォーマット"""
        assert self.format_distance(0.01) == "1.00 cm"
        assert self.format_distance(0.1) == "10.00 cm"
        assert self.format_distance(0.5) == "50.00 cm"
        assert self.format_distance(0.99) == "99.00 cm"

    def test_millimeter_formatting(self):
        """ミリメートル単位のフォーマット"""
        assert self.format_distance(0.001) == "1.00 mm"
        assert self.format_distance(0.005) == "5.00 mm"
        assert self.format_distance(0.009) == "9.00 mm"

    def test_edge_cases(self):
        """境界値のテスト"""
        # ちょうど1kmの場合
        assert "km" in self.format_distance(1000)

        # 999.999...mの場合
        assert "m" in self.format_distance(999.999) or "km" in self.format_distance(999.999)

        # ちょうど1mの場合
        assert "m" in self.format_distance(1.0)

    def test_zero_distance(self):
        """ゼロ距離のフォーマット"""
        result = self.format_distance(0)
        assert "mm" in result

    def test_very_large_distance(self):
        """非常に大きな距離のフォーマット"""
        result = self.format_distance(1000000)  # 1000km
        assert "km" in result
        assert "1000" in result

    def test_very_small_distance(self):
        """非常に小さな距離のフォーマット"""
        result = self.format_distance(0.0001)  # 0.1mm
        assert "mm" in result


class TestNumberFormatting:
    """数値フォーマットのテスト"""

    def get_nice_number(self, value, round_up=False):
        """きれいな数値に丸める"""
        exponent = math.floor(math.log10(value))
        fraction = value / (10 ** exponent)

        if round_up:
            if fraction < 1.5:
                nice_fraction = 2
            elif fraction < 3:
                nice_fraction = 5
            elif fraction < 7:
                nice_fraction = 10
            else:
                nice_fraction = 10
        else:
            if fraction <= 1:
                nice_fraction = 1
            elif fraction <= 2:
                nice_fraction = 2
            elif fraction <= 5:
                nice_fraction = 5
            else:
                nice_fraction = 10

        return nice_fraction * (10 ** exponent)

    def test_nice_number_basic(self):
        """基本的なきれいな数値への丸め"""
        assert self.get_nice_number(15) in [10, 20]
        assert self.get_nice_number(35) in [20, 50]
        assert self.get_nice_number(75) in [50, 100]

    def test_nice_number_powers_of_ten(self):
        """10の累乗のテスト"""
        assert self.get_nice_number(10) == 10
        assert self.get_nice_number(100) == 100
        assert self.get_nice_number(1000) == 1000

    def test_nice_number_fractions(self):
        """小数のテスト"""
        result = self.get_nice_number(0.15)
        assert result > 0
        assert result <= 1

    def test_nice_number_round_up(self):
        """切り上げモードのテスト"""
        result_up = self.get_nice_number(15, round_up=True)
        result_down = self.get_nice_number(15, round_up=False)

        # 切り上げの方が大きいか等しい
        assert result_up >= result_down


class TestNumericFormatting:
    """数値フォーマット全般のテスト"""

    def test_integer_formatting(self):
        """整数のフォーマット"""
        value = 42
        formatted = f"{value}"
        assert formatted == "42"
        assert isinstance(int(formatted), int)

    def test_float_formatting_precision(self):
        """浮動小数点数の精度フォーマット"""
        value = 3.14159265

        # 小数点以下2桁
        formatted_2 = f"{value:.2f}"
        assert formatted_2 == "3.14"

        # 小数点以下4桁
        formatted_4 = f"{value:.4f}"
        assert formatted_4 == "3.1416"

    def test_scientific_notation(self):
        """科学的記数法のフォーマット"""
        value = 1234567.89
        formatted = f"{value:.2e}"
        assert "e+" in formatted or "E+" in formatted

    def test_percentage_formatting(self):
        """パーセンテージのフォーマット"""
        value = 0.75
        percentage = f"{value * 100:.1f}%"
        assert percentage == "75.0%"

    def test_padding_formatting(self):
        """パディングのテスト"""
        value = 42

        # 5桁にゼロパディング
        padded = f"{value:05d}"
        assert padded == "00042"

    def test_alignment_formatting(self):
        """アライメントのテスト"""
        value = 42

        # 右寄せ（幅10）
        right_aligned = f"{value:>10}"
        assert len(right_aligned) == 10
        assert right_aligned.strip() == "42"

        # 左寄せ（幅10）
        left_aligned = f"{value:<10}"
        assert len(left_aligned) == 10


class TestCoordinateFormatting:
    """座標フォーマットのテスト"""

    def format_coordinate(self, x, y, z=None):
        """座標を文字列にフォーマット"""
        if z is not None:
            return f"({x:.3f}, {y:.3f}, {z:.3f})"
        return f"({x:.3f}, {y:.3f})"

    def test_2d_coordinate_formatting(self):
        """2D座標のフォーマット"""
        result = self.format_coordinate(1.234, 5.678)
        assert result == "(1.234, 5.678)"

    def test_3d_coordinate_formatting(self):
        """3D座標のフォーマット"""
        result = self.format_coordinate(1.234, 5.678, 9.012)
        assert result == "(1.234, 5.678, 9.012)"

    def test_negative_coordinate_formatting(self):
        """負の座標のフォーマット"""
        result = self.format_coordinate(-1.234, -5.678)
        assert "-1.234" in result
        assert "-5.678" in result

    def test_zero_coordinate_formatting(self):
        """ゼロ座標のフォーマット"""
        result = self.format_coordinate(0, 0, 0)
        assert result == "(0.000, 0.000, 0.000)"


class TestResolutionFormatting:
    """解像度フォーマットのテスト"""

    def format_resolution(self, meters_per_pixel):
        """解像度を読みやすい形式にフォーマット"""
        cm_per_pixel = meters_per_pixel * 100
        return f"{cm_per_pixel:.1f} cm/pixel"

    def test_standard_resolutions(self):
        """標準的な解像度のフォーマット"""
        # 0.05m/pixel = 5cm/pixel
        assert self.format_resolution(0.05) == "5.0 cm/pixel"

        # 0.1m/pixel = 10cm/pixel
        assert self.format_resolution(0.1) == "10.0 cm/pixel"

        # 0.025m/pixel = 2.5cm/pixel
        assert self.format_resolution(0.025) == "2.5 cm/pixel"

    def test_high_resolution(self):
        """高解像度のフォーマット"""
        # 0.01m/pixel = 1cm/pixel
        assert self.format_resolution(0.01) == "1.0 cm/pixel"

    def test_low_resolution(self):
        """低解像度のフォーマット"""
        # 0.2m/pixel = 20cm/pixel
        assert self.format_resolution(0.2) == "20.0 cm/pixel"


class TestAngleFormatting:
    """角度フォーマットのテスト"""

    def format_angle_degrees(self, radians):
        """ラジアンを度に変換してフォーマット"""
        degrees = math.degrees(radians)
        return f"{degrees:.1f}°"

    def test_common_angles(self):
        """よく使用される角度のフォーマット"""
        # 0度
        assert self.format_angle_degrees(0) == "0.0°"

        # 90度（π/2ラジアン）
        result_90 = self.format_angle_degrees(math.pi / 2)
        assert "90" in result_90

        # 180度（πラジアン）
        result_180 = self.format_angle_degrees(math.pi)
        assert "180" in result_180

    def test_negative_angles(self):
        """負の角度のフォーマット"""
        result = self.format_angle_degrees(-math.pi / 4)  # -45度
        assert "-45" in result

    def test_angle_normalization(self):
        """角度の正規化（0-360度）"""
        def normalize_angle(degrees):
            while degrees < 0:
                degrees += 360
            while degrees >= 360:
                degrees -= 360
            return degrees

        assert normalize_angle(450) == 90
        assert normalize_angle(-90) == 270
        assert normalize_angle(0) == 0
        assert normalize_angle(360) == 0
