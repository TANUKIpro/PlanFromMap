"""
画像有効領域抽出モジュール

白い塗りつぶしと黒い枠線のみを有効領域として抽出するアルゴリズム

このスクリプトは、画像内の以下の要素のみを有効領域として認識します：
- 黒い枠線（低い輝度値）
- 白い塗りつぶし領域（高い輝度値）
グレーの背景は無効領域として除外されます。
"""

import numpy as np
from PIL import Image
import io


def extract_valid_region(image_path_or_bytes, black_threshold=100, white_threshold=220):
    """
    白い塗りつぶしと黒い枠線を含む有効領域を抽出

    Parameters:
    -----------
    image_path_or_bytes : str or bytes
        入力画像のパスまたはバイトデータ
    black_threshold : int
        黒とみなす輝度値の上限（この値以下が黒）
    white_threshold : int
        白とみなす輝度値の下限（この値以上が白）

    Returns:
    --------
    tuple : (cropped_image_array, bounds)
        cropped_image_array : numpy array (クロップされた画像)
        bounds : dict with keys 'rmin', 'rmax', 'cmin', 'cmax', 'original_width', 'original_height'
    """
    # 画像を読み込む
    if isinstance(image_path_or_bytes, bytes):
        img = Image.open(io.BytesIO(image_path_or_bytes))
    else:
        img = Image.open(image_path_or_bytes)

    img_array = np.array(img)

    # RGBをグレースケールに変換
    if len(img_array.shape) == 3:
        if img_array.shape[2] == 4:  # RGBA
            img_gray = np.mean(img_array[:,:,:3], axis=2).astype(np.uint8)
        else:  # RGB
            img_gray = np.mean(img_array, axis=2).astype(np.uint8)
    else:
        img_gray = img_array

    original_height, original_width = img_gray.shape

    # 有効領域のマスクを作成
    # 黒い部分（枠線）または白い部分（塗りつぶし）を有効とする
    black_mask = img_gray <= black_threshold
    white_mask = img_gray >= white_threshold
    valid_mask = black_mask | white_mask

    # 有効領域の境界を検出
    rows = np.any(valid_mask, axis=1)
    cols = np.any(valid_mask, axis=0)

    if not np.any(rows) or not np.any(cols):
        # 有効領域が見つからない場合は元の画像を返す
        return img_array, {
            'rmin': 0,
            'rmax': original_height - 1,
            'cmin': 0,
            'cmax': original_width - 1,
            'original_width': original_width,
            'original_height': original_height
        }

    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]

    # 有効領域を切り出す
    if len(img_array.shape) == 3:
        cropped_image = img_array[rmin:rmax+1, cmin:cmax+1]
    else:
        cropped_image = img_gray[rmin:rmax+1, cmin:cmax+1]

    bounds = {
        'rmin': int(rmin),
        'rmax': int(rmax),
        'cmin': int(cmin),
        'cmax': int(cmax),
        'original_width': int(original_width),
        'original_height': int(original_height)
    }

    return cropped_image, bounds


def optimize_image_bytes(image_bytes, black_threshold=100, white_threshold=220, output_format='PNG'):
    """
    画像バイトデータを最適化して返す

    Parameters:
    -----------
    image_bytes : bytes
        入力画像のバイトデータ
    black_threshold : int
        黒とみなす輝度値の上限
    white_threshold : int
        白とみなす輝度値の下限
    output_format : str
        出力画像フォーマット ('PNG', 'JPEG', etc.)

    Returns:
    --------
    tuple : (output_bytes, bounds_info)
        output_bytes : bytes (最適化された画像のバイトデータ)
        bounds_info : dict (境界情報)
    """
    # 有効領域を抽出
    cropped_array, bounds = extract_valid_region(image_bytes, black_threshold, white_threshold)

    # numpy配列をPIL Imageに変換
    cropped_img = Image.fromarray(cropped_array)

    # バイトデータに変換
    output_buffer = io.BytesIO()
    cropped_img.save(output_buffer, format=output_format)
    output_bytes = output_buffer.getvalue()

    return output_bytes, bounds
