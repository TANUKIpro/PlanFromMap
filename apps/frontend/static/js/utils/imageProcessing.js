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

/**
 * マップ画像の有効領域（黒色+白色ピクセル）を検出
 *
 * この関数は、占有格子マップ（Occupancy Grid Map）の有効領域を検出します。
 * 黒色ピクセル（障害物、≤100）と白色ピクセル（自由空間、≥220）を有効領域とし、
 * グレー（未知領域、101-219）を除外します。
 *
 * 用途：
 * - 大きな余白を持つPGMファイルのテクスチャサイズ最適化
 * - 2Dマップビューの初期表示領域の自動調整
 * - 3Dマップのテクスチャ範囲の最適化
 *
 * @param {HTMLImageElement} image - 解析する画像
 * @param {number} [marginPercent=0.05] - 検出領域に追加するマージン（デフォルト5%）
 * @returns {Object|null} 有効領域の境界情報、検出失敗時はnull
 * @returns {number} returns.minX - 最小X座標（ピクセル）
 * @returns {number} returns.minY - 最小Y座標（ピクセル）
 * @returns {number} returns.maxX - 最大X座標（ピクセル）
 * @returns {number} returns.maxY - 最大Y座標（ピクセル）
 * @returns {number} returns.centerX - 中心X座標（ピクセル）
 * @returns {number} returns.centerY - 中心Y座標（ピクセル）
 *
 * @example
 * const img = new Image();
 * img.onload = function() {
 *     const bounds = analyzeMapBounds(img);
 *     if (bounds) {
 *         console.log(`有効領域: [${bounds.minX}, ${bounds.minY}] - [${bounds.maxX}, ${bounds.maxY}]`);
 *     }
 * };
 * img.src = 'map.pgm';
 */
export function analyzeMapBounds(image, marginPercent = 0.05) {
    if (!image) return null;

    // 画像データを取得
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = image.width;
    tempCanvas.height = image.height;
    const tempCtx = tempCanvas.getContext('2d');
    tempCtx.drawImage(image, 0, 0);
    const imageData = tempCtx.getImageData(0, 0, image.width, image.height);
    const data = imageData.data;

    // 占有格子マップの閾値
    // 黒色（障害物）: 0-100
    // 白色（自由空間）: 220-255
    // グレー（未知領域）: 101-219 は除外
    const blackThreshold = 100;
    const whiteThreshold = 220;

    // 有効ピクセル（黒または白）を持つ行と列を記録
    const validRows = new Set();
    const validCols = new Set();

    // 全ピクセルをスキャンして有効ピクセルを検出
    for (let y = 0; y < image.height; y++) {
        for (let x = 0; x < image.width; x++) {
            const index = (y * image.width + x) * 4;
            const gray = (data[index] + data[index + 1] + data[index + 2]) / 3;

            // 黒色（障害物）または白色（自由空間）の場合は有効
            if (gray <= blackThreshold || gray >= whiteThreshold) {
                validRows.add(y);
                validCols.add(x);
            }
        }
    }

    // 有効ピクセルが見つからない場合は画像全体を使用
    if (validRows.size === 0 || validCols.size === 0) {
        console.warn('analyzeMapBounds: 有効な領域が見つかりませんでした。画像全体を使用します。');
        return {
            minX: 0,
            minY: 0,
            maxX: image.width - 1,
            maxY: image.height - 1,
            centerX: image.width / 2,
            centerY: image.height / 2
        };
    }

    // 行と列の最小値・最大値を取得
    const rowArray = Array.from(validRows).sort((a, b) => a - b);
    const colArray = Array.from(validCols).sort((a, b) => a - b);

    const minY = rowArray[0];
    const maxY = rowArray[rowArray.length - 1];
    const minX = colArray[0];
    const maxX = colArray[colArray.length - 1];

    // マージンを追加
    const width = maxX - minX + 1;
    const height = maxY - minY + 1;
    const marginX = Math.ceil(width * marginPercent);
    const marginY = Math.ceil(height * marginPercent);

    const finalMinX = Math.max(0, minX - marginX);
    const finalMinY = Math.max(0, minY - marginY);
    const finalMaxX = Math.min(image.width - 1, maxX + marginX);
    const finalMaxY = Math.min(image.height - 1, maxY + marginY);

    // 中心座標を計算
    const centerX = (finalMinX + finalMaxX) / 2;
    const centerY = (finalMinY + finalMaxY) / 2;

    console.log(`analyzeMapBounds: 有効領域を検出 [${finalMinX}, ${finalMinY}] - [${finalMaxX}, ${finalMaxY}], 中心: (${centerX.toFixed(1)}, ${centerY.toFixed(1)})`);

    return { minX: finalMinX, minY: finalMinY, maxX: finalMaxX, maxY: finalMaxY, centerX, centerY };
}
