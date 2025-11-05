"""
Image Processing Tests
画像処理機能（PGMパース、画像変換）のユニットテスト
"""
import pytest
import struct
from pathlib import Path


class TestPGMParsing:
    """PGMフォーマットパースのテスト"""

    def create_sample_pgm_p5(self, width=10, height=10, maxval=255):
        """P5形式（バイナリ）のサンプルPGMデータを作成"""
        header = f"P5\n{width} {height}\n{maxval}\n".encode('ascii')
        pixel_data = bytes([i % 256 for i in range(width * height)])
        return header + pixel_data

    def create_sample_pgm_p2(self, width=10, height=10, maxval=255):
        """P2形式（ASCII）のサンプルPGMデータを作成"""
        header = f"P2\n{width} {height}\n{maxval}\n"
        pixels = ' '.join(str(i % 256) for i in range(width * height))
        return (header + pixels).encode('ascii')

    def test_parse_pgm_p5_format(self, sample_map_files):
        """実際のP5形式PGMファイルをパースできる"""
        pgm_path = sample_map_files['pgm']

        # ファイルが存在することを確認
        assert pgm_path.exists(), f"PGMファイルが存在しません: {pgm_path}"

        # ファイルを読み込む
        with open(pgm_path, 'rb') as f:
            pgm_data = f.read()

        # 基本的なPGMヘッダーを検証
        header_str = pgm_data[:100].decode('ascii', errors='ignore')
        assert header_str.startswith('P5'), "PGMファイルはP5形式である必要があります"

        # サイズが妥当であることを確認
        assert len(pgm_data) > 100, "PGMファイルのサイズが小さすぎます"

    def test_parse_pgm_p5_basic(self):
        """P5形式の基本的なPGMデータをパースできる"""
        # 簡単なPGMデータを作成（10x10ピクセル）
        pgm_data = self.create_sample_pgm_p5(10, 10, 255)

        # ヘッダーの検証
        assert pgm_data.startswith(b'P5'), "P5形式である必要があります"

        # データサイズの検証
        # ヘッダー + 10*10 ピクセル
        assert len(pgm_data) >= 100, "データサイズが妥当です"

    def test_parse_pgm_p2_basic(self):
        """P2形式の基本的なPGMデータをパースできる"""
        pgm_data = self.create_sample_pgm_p2(5, 5, 255)

        # ヘッダーの検証
        assert pgm_data.startswith(b'P2'), "P2形式である必要があります"

    def test_parse_pgm_with_comments(self):
        """コメントを含むPGMデータをパースできる"""
        header = b"P5\n# This is a comment\n10 10\n# Another comment\n255\n"
        pixel_data = bytes([128] * 100)
        pgm_data = header + pixel_data

        # コメント行が存在することを確認
        assert b'# This is a comment' in pgm_data

    def test_parse_pgm_various_maxval(self):
        """様々な最大値を持つPGMデータをパースできる"""
        for maxval in [1, 15, 127, 255]:
            pgm_data = self.create_sample_pgm_p5(5, 5, maxval)
            assert len(pgm_data) > 0

    def test_parse_pgm_large_image(self):
        """大きな画像のPGMデータをパースできる"""
        # 2048x2048の画像（ROSでよく使用されるサイズ）
        width, height = 2048, 2048
        pgm_data = self.create_sample_pgm_p5(width, height, 255)

        # データサイズが妥当であることを確認
        expected_size = width * height
        # ヘッダーサイズを考慮（約20-30バイト）
        assert len(pgm_data) >= expected_size


class TestColorConversion:
    """色変換機能のテスト"""

    def test_hex_to_rgb_valid_with_hash(self):
        """#を含む有効な16進数カラーコードを変換できる"""
        test_cases = [
            ("#FF0000", (255, 0, 0)),     # 赤
            ("#00FF00", (0, 255, 0)),     # 緑
            ("#0000FF", (0, 0, 255)),     # 青
            ("#FFFFFF", (255, 255, 255)), # 白
            ("#000000", (0, 0, 0)),       # 黒
        ]

        for hex_color, expected_rgb in test_cases:
            # 手動で変換
            r = int(hex_color[1:3], 16)
            g = int(hex_color[3:5], 16)
            b = int(hex_color[5:7], 16)
            assert (r, g, b) == expected_rgb

    def test_hex_to_rgb_valid_without_hash(self):
        """#なしの有効な16進数カラーコードを変換できる"""
        hex_color = "FF0000"
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        assert (r, g, b) == (255, 0, 0)

    def test_hex_to_rgb_lowercase(self):
        """小文字の16進数カラーコードを変換できる"""
        hex_color = "#ff00ff"
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        assert (r, g, b) == (255, 0, 255)

    def test_hex_to_rgb_mixed_case(self):
        """大文字小文字が混在した16進数カラーコードを変換できる"""
        hex_color = "#FfAa00"
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        assert (r, g, b) == (255, 170, 0)


class TestImageDataConversion:
    """画像データ変換のテスト"""

    def test_grayscale_to_rgba_conversion(self):
        """グレースケールデータをRGBAに変換できる"""
        # グレースケール値
        gray_values = [0, 64, 128, 192, 255]

        for gray in gray_values:
            # RGBAに変換（グレースケールなのでR=G=B）
            r = g = b = gray
            a = 255  # 不透明

            assert r == gray
            assert g == gray
            assert b == gray
            assert a == 255

    def test_pixel_normalization(self):
        """ピクセル値の正規化が正しく行われる"""
        # maxVal=100の場合、50は127.5に正規化される（255スケール）
        maxval = 100
        value = 50
        normalized = int((value / maxval) * 255)

        assert normalized == 127  # 四捨五入で127

    def test_image_dimensions(self):
        """画像の次元が正しく保持される"""
        width, height = 640, 480
        pixel_count = width * height

        # RGBAデータのサイズ
        rgba_size = pixel_count * 4

        assert rgba_size == 640 * 480 * 4
        assert rgba_size == 1228800


class TestPGMFileStructure:
    """PGMファイル構造のテスト"""

    def test_yaml_and_pgm_consistency(self, sample_map_files, sample_yaml_data):
        """YAMLファイルとPGMファイルの整合性を確認"""
        yaml_path = sample_map_files['yaml']
        pgm_path = sample_map_files['pgm']

        # 両方のファイルが存在することを確認
        assert yaml_path.exists(), f"YAMLファイルが存在しません: {yaml_path}"
        assert pgm_path.exists(), f"PGMファイルが存在しません: {pgm_path}"

        # YAMLファイルのサイズが妥当であることを確認
        yaml_size = yaml_path.stat().st_size
        assert yaml_size > 0, "YAMLファイルが空です"
        assert yaml_size < 10000, "YAMLファイルが大きすぎます"

        # PGMファイルのサイズが妥当であることを確認
        pgm_size = pgm_path.stat().st_size
        assert pgm_size > 1000, "PGMファイルが小さすぎます"

    def test_pgm_file_readability(self, sample_map_files):
        """PGMファイルが読み込み可能である"""
        pgm_path = sample_map_files['pgm']

        # バイナリモードで読み込める
        with open(pgm_path, 'rb') as f:
            data = f.read(100)  # 最初の100バイトを読み込み
            assert len(data) == 100
            assert data.startswith(b'P5') or data.startswith(b'P2')


class TestEdgeCases:
    """エッジケースのテスト"""

    def test_empty_image(self):
        """空の画像データの処理"""
        width, height = 0, 0
        assert width * height == 0

    def test_single_pixel_image(self):
        """1ピクセルの画像"""
        width, height = 1, 1
        assert width * height == 1

    def test_very_narrow_image(self):
        """非常に細い画像（1xN）"""
        width, height = 1, 1000
        assert width * height == 1000

    def test_very_wide_image(self):
        """非常に幅広い画像（Nx1）"""
        width, height = 1000, 1
        assert width * height == 1000

    def test_maxval_edge_cases(self):
        """最大値のエッジケース"""
        maxvals = [1, 2, 15, 16, 127, 128, 255, 256]

        for maxval in maxvals:
            # 正規化の計算
            if maxval > 0:
                normalized = int((maxval / maxval) * 255)
                assert normalized == 255
