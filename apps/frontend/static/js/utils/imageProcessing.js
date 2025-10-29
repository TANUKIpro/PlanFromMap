/**
 * @file imageProcessing.js
 * @description 画像処理ユーティリティ関数
 *
 * このファイルには、画像データの読み込み、変換、処理に関する
 * ユーティリティ関数が含まれています：
 * - PGMフォーマットのパース
 * - PGMデータからImageオブジェクトへの変換
 * - 16進数カラーコードの変換
 *
 * @exports parsePGM - PGMフォーマット（P5, P2）をパース
 * @exports pgmToImage - PGMデータをImageオブジェクトに変換
 * @exports hexToRgb - 16進数カラーコードをRGB値に変換
 */

/**
 * PGMフォーマット（Portable Gray Map）をパース
 *
 * この関数は、PGMフォーマット（P5: バイナリ形式、P2: ASCII形式）の
 * 画像データをパースして、幅、高さ、最大値、ピクセルデータを返します。
 *
 * PGMフォーマットは、ロボット工学でよく使用される占有グリッドマップ（OGM）の
 * 標準フォーマットです。ROSのmap_serverもPGM形式をサポートしています。
 *
 * フォーマット仕様：
 * - マジックナンバー: "P5"（バイナリ）または "P2"（ASCII）
 * - 幅と高さ: 空白区切りの2つの整数
 * - 最大値: ピクセルの最大値（通常255）
 * - ピクセルデータ: バイナリまたはASCII形式
 *
 * @param {Uint8Array} uint8Array - PGMファイルのバイナリデータ
 * @returns {Object} パースされたPGMデータ
 * @returns {number} returns.width - 画像の幅（ピクセル）
 * @returns {number} returns.height - 画像の高さ（ピクセル）
 * @returns {number} returns.maxVal - ピクセルの最大値
 * @returns {Uint8Array} returns.data - ピクセルデータ配列
 * @throws {Error} サポートされていないフォーマットの場合
 *
 * @example
 * const fileReader = new FileReader();
 * fileReader.onload = function(e) {
 *     const uint8Array = new Uint8Array(e.target.result);
 *     const pgmData = parsePGM(uint8Array);
 *     console.log(`Width: ${pgmData.width}, Height: ${pgmData.height}`);
 * };
 * fileReader.readAsArrayBuffer(file);
 */
export function parsePGM(uint8Array) {
    let offset = 0;

    // ヘッダーを読み込む（テキスト形式）
    let headerText = '';
    while (offset < uint8Array.length) {
        const char = String.fromCharCode(uint8Array[offset]);
        headerText += char;
        offset++;

        // ヘッダーの終わりを検出（3つの数値を読み取ったら）
        const lines = headerText.split('\n').filter(line =>
            line.trim() && !line.trim().startsWith('#')
        );

        if (lines.length >= 3) {
            break;
        }
    }

    // ヘッダーをパース
    const lines = headerText.split('\n').filter(line =>
        line.trim() && !line.trim().startsWith('#')
    );

    const format = lines[0].trim();
    if (format !== 'P5' && format !== 'P2') {
        throw new Error('サポートされていないPGMフォーマット: ' + format);
    }

    const [width, height] = lines[1].trim().split(/\s+/).map(Number);
    const maxVal = parseInt(lines[2].trim());

    // バイナリデータを読み込む
    const pixelData = new Uint8Array(width * height);

    if (format === 'P5') {
        // バイナリ形式
        for (let i = 0; i < width * height; i++) {
            pixelData[i] = uint8Array[offset + i];
        }
    } else {
        // ASCII形式（P2）
        const dataText = String.fromCharCode.apply(null,
            Array.from(uint8Array.slice(offset))
        );
        const values = dataText.trim().split(/\s+/).map(Number);
        for (let i = 0; i < width * height && i < values.length; i++) {
            pixelData[i] = values[i];
        }
    }

    return {
        width: width,
        height: height,
        maxVal: maxVal,
        data: pixelData
    };
}

/**
 * PGMデータをImageオブジェクトに変換
 *
 * この関数は、parsePGM()で取得したPGMデータを、ブラウザで表示可能な
 * Imageオブジェクトに変換します。グレースケールのピクセルデータを
 * RGBAフォーマットに変換し、一時的なCanvasを使用してImageオブジェクトを生成します。
 *
 * @param {Object} pgmData - parsePGM()から返されたPGMデータ
 * @param {number} pgmData.width - 画像の幅
 * @param {number} pgmData.height - 画像の高さ
 * @param {number} pgmData.maxVal - ピクセルの最大値
 * @param {Uint8Array} pgmData.data - ピクセルデータ
 * @returns {HTMLImageElement} 変換されたImageオブジェクト
 *
 * @example
 * const pgmData = parsePGM(uint8Array);
 * const img = pgmToImage(pgmData);
 * img.onload = function() {
 *     console.log('Image loaded successfully');
 *     document.body.appendChild(img);
 * };
 */
export function pgmToImage(pgmData) {
    // 一時キャンバスを作成
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = pgmData.width;
    tempCanvas.height = pgmData.height;
    const tempCtx = tempCanvas.getContext('2d');

    // ImageDataを作成
    const imageData = tempCtx.createImageData(pgmData.width, pgmData.height);

    // グレースケールデータをRGBAに変換
    for (let i = 0; i < pgmData.data.length; i++) {
        const value = Math.floor((pgmData.data[i] / pgmData.maxVal) * 255);
        const idx = i * 4;
        imageData.data[idx] = value;     // R
        imageData.data[idx + 1] = value; // G
        imageData.data[idx + 2] = value; // B
        imageData.data[idx + 3] = 255;   // A
    }

    // キャンバスに描画
    tempCtx.putImageData(imageData, 0, 0);

    // Imageオブジェクトを作成
    const img = new Image();
    img.src = tempCanvas.toDataURL();

    return img;
}

/**
 * 16進数カラーコードをRGB値に変換
 *
 * この関数は、CSSの16進数カラーコード（例: "#FF0000"）を、
 * 個別のRGB値（0-255）を持つオブジェクトに変換します。
 * "#"プレフィックスの有無は問いません。
 *
 * @param {string} hex - 16進数カラーコード（例: "#FF0000" または "FF0000"）
 * @returns {Object|null} RGB値のオブジェクト {r, g, b}、無効な場合はnull
 * @returns {number} returns.r - 赤成分（0-255）
 * @returns {number} returns.g - 緑成分（0-255）
 * @returns {number} returns.b - 青成分（0-255）
 *
 * @example
 * const rgb = hexToRgb("#FF0000");
 * // 結果: {r: 255, g: 0, b: 0}
 *
 * const rgb2 = hexToRgb("00FF00");
 * // 結果: {r: 0, g: 255, b: 0}
 *
 * const rgb3 = hexToRgb("invalid");
 * // 結果: null
 */
export function hexToRgb(hex) {
    const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}
