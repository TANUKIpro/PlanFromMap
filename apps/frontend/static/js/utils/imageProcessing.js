/**
 * @file imageProcessing.js
 * @description 画像処理ユーティリティ関数
 *
 * このファイルには、画像データの読み込み、変換、処理に関する
 * ユーティリティ関数が含まれています：
 * - PGMフォーマットのパース
 * - PGMデータからImageオブジェクトへの変換
 * - PGM形式への変換とファイル生成
 * - 16進数カラーコードの変換
 *
 * @exports parsePGM - PGMフォーマット（P5, P2）をパース
 * @exports pgmToImage - PGMデータをImageオブジェクトに変換
 * @exports imageToPGM - ImageまたはCanvasをPGMデータに変換
 * @exports canvasToPGM - CanvasをPGMデータに変換
 * @exports downloadPGM - PGMファイルをダウンロード
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
 * Canvasをグレースケールデータに変換
 *
 * @private
 * @param {HTMLCanvasElement} canvas - 変換するキャンバス
 * @returns {Uint8Array} グレースケールピクセルデータ
 */
function canvasToGrayscale(canvas) {
    const ctx = canvas.getContext('2d');
    const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
    const data = imageData.data;
    const grayscaleData = new Uint8Array(canvas.width * canvas.height);

    for (let i = 0; i < canvas.width * canvas.height; i++) {
        const idx = i * 4;
        const r = data[idx];
        const g = data[idx + 1];
        const b = data[idx + 2];
        const alpha = data[idx + 3];

        // アルファ値が0の場合は255（白）、それ以外はグレースケール変換
        if (alpha === 0) {
            grayscaleData[i] = 255;
        } else {
            // グレースケール変換（ITU-R BT.601の係数）
            grayscaleData[i] = Math.floor(0.299 * r + 0.587 * g + 0.114 * b);
        }
    }

    return grayscaleData;
}

/**
 * CanvasをPGMデータに変換
 *
 * この関数は、HTMLCanvasElementをPGMフォーマット（P5: バイナリ形式）のデータに変換します。
 * キャンバスの内容をグレースケールに変換し、PGMヘッダーとピクセルデータを生成します。
 *
 * @param {HTMLCanvasElement} canvas - 変換するキャンバス
 * @returns {Uint8Array} PGMフォーマットのバイナリデータ
 *
 * @example
 * const canvas = document.getElementById('myCanvas');
 * const pgmData = canvasToPGM(canvas);
 */
export function canvasToPGM(canvas) {
    if (!canvas || !canvas.getContext) {
        throw new Error('有効なCanvasが指定されていません');
    }

    const width = canvas.width;
    const height = canvas.height;
    const grayscaleData = canvasToGrayscale(canvas);

    // PGMヘッダーを生成
    const header = `P5\n${width} ${height}\n255\n`;
    const headerBytes = new TextEncoder().encode(header);

    // ヘッダーとピクセルデータを結合
    const pgmData = new Uint8Array(headerBytes.length + grayscaleData.length);
    pgmData.set(headerBytes, 0);
    pgmData.set(grayscaleData, headerBytes.length);

    return pgmData;
}

/**
 * ImageまたはCanvasをPGMデータに変換
 *
 * この関数は、HTMLImageElementまたはHTMLCanvasElementをPGMフォーマットのデータに変換します。
 * Imageの場合は一時的なCanvasを作成してから変換します。
 *
 * @param {HTMLImageElement|HTMLCanvasElement} source - 変換する画像またはキャンバス
 * @returns {Uint8Array} PGMフォーマットのバイナリデータ
 *
 * @example
 * const img = document.getElementById('myImage');
 * const pgmData = imageToPGM(img);
 */
export function imageToPGM(source) {
    if (!source) {
        throw new Error('画像またはキャンバスが指定されていません');
    }

    // Canvasの場合はそのまま変換
    if (source instanceof HTMLCanvasElement) {
        return canvasToPGM(source);
    }

    // Imageの場合は一時Canvasを作成
    if (source instanceof HTMLImageElement) {
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = source.width;
        tempCanvas.height = source.height;
        const ctx = tempCanvas.getContext('2d');
        ctx.drawImage(source, 0, 0);
        return canvasToPGM(tempCanvas);
    }

    throw new Error('サポートされていないソースタイプです');
}

/**
 * PGMファイルをダウンロード
 *
 * この関数は、PGMデータをファイルとしてダウンロードします。
 * ブラウザのダウンロード機能を使用して、指定されたファイル名でPGMファイルを保存します。
 *
 * @param {Uint8Array} pgmData - PGMフォーマットのバイナリデータ
 * @param {string} filename - 保存するファイル名（拡張子を含む）
 *
 * @example
 * const canvas = document.getElementById('myCanvas');
 * const pgmData = canvasToPGM(canvas);
 * downloadPGM(pgmData, 'map.pgm');
 */
export function downloadPGM(pgmData, filename) {
    if (!pgmData || !(pgmData instanceof Uint8Array)) {
        throw new Error('有効なPGMデータが指定されていません');
    }

    if (!filename || typeof filename !== 'string') {
        throw new Error('有効なファイル名が指定されていません');
    }

    const blob = new Blob([pgmData], { type: 'application/octet-stream' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
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
