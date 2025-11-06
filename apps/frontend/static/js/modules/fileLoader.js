/**
 * @file fileLoader.js
 * @description ファイル読み込み・保存関連の機能を提供するモジュール
 * @requires ../state/mapState.js
 * @requires ./layerManager.js
 * @requires ./metadataDisplay.js
 * @requires ../utils/imageProcessing.js
 * @exports handleImageFileSelect
 * @exports handleYAMLFileSelect
 * @exports loadStandardImageFile
 * @exports loadYAMLMetadataFile
 * @exports loadPGMImageFile
 * @exports saveMapAsPGM
 * @exports parsePGM
 * @exports pgmToImage
 */

import { mapState } from '../state/mapState.js';
import { initializeLayers, redrawAllLayers } from './layerManager.js';
import { displayMetadata, updateOverlayControls } from './metadataDisplay.js';
import { canvasToPGM, downloadPGM, analyzeMapBounds } from '../utils/imageProcessing.js';
import { API_BASE_URL } from '../config.js';

/**
 * 有効領域にフィットするようにビューを調整
 * @private
 * @param {HTMLImageElement} image - 画像
 * @param {Object} container - コンテナ要素
 */
function fitViewToBounds(image, container) {
    // 有効領域を検出
    const bounds = analyzeMapBounds(image);
    if (!bounds) {
        console.log('fitViewToBounds: 有効領域が検出できませんでした。デフォルトビューを使用します。');
        return;
    }

    // 有効領域のサイズ（ピクセル）
    const boundsWidth = bounds.maxX - bounds.minX;
    const boundsHeight = bounds.maxY - bounds.minY;

    // コンテナのサイズ
    const containerRect = container.getBoundingClientRect();
    const containerWidth = containerRect.width;
    const containerHeight = containerRect.height;

    // 有効領域がコンテナにフィットするスケールを計算（余白10%を考慮）
    const scaleX = (containerWidth * 0.9) / boundsWidth;
    const scaleY = (containerHeight * 0.9) / boundsHeight;
    const scale = Math.min(scaleX, scaleY, 2.0); // 最大2倍まで

    // 有効領域の中心をコンテナの中心に配置するオフセットを計算
    const boundsCenterX = (bounds.minX + bounds.maxX) / 2;
    const boundsCenterY = (bounds.minY + bounds.maxY) / 2;
    const offsetX = containerWidth / 2 - boundsCenterX * scale;
    const offsetY = containerHeight / 2 - boundsCenterY * scale;

    // mapStateを更新
    mapState.scale = scale;
    mapState.offsetX = offsetX;
    mapState.offsetY = offsetY;

    console.log(`fitViewToBounds: スケール=${scale.toFixed(2)}, オフセット=(${offsetX.toFixed(1)}, ${offsetY.toFixed(1)})`);
}

/**
 * PGMフォーマットをパース（P5形式とP2形式に対応）
 * @export
 * @param {Uint8Array} uint8Array - PGMファイルのバイナリデータ
 * @returns {{width: number, height: number, maxVal: number, data: Uint8Array}} PGMデータ
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
 * @export
 * @param {{width: number, height: number, maxVal: number, data: Uint8Array}} pgmData - PGMデータ
 * @returns {Image} 画像オブジェクト
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
 * 画像ファイル選択時の処理
 * @export
 * @param {Event} event - ファイル選択イベント
 */
export function handleImageFileSelect(event) {
    const file = event.target.files[0];
    if (!file) return;

    const fileName = file.name.toLowerCase();

    // PGMファイルの場合
    if (fileName.endsWith('.pgm')) {
        loadPGMImageFile(file);
    } else {
        // 通常の画像ファイル
        loadStandardImageFile(file);
    }
}

/**
 * YAMLファイル選択時の処理
 * @export
 * @param {Event} event - ファイル選択イベント
 */
export function handleYAMLFileSelect(event) {
    const file = event.target.files[0];
    if (!file) return;

    loadYAMLMetadataFile(file);
}

/**
 * バックエンドAPIで画像を最適化
 * @private
 * @param {File} file - 画像ファイル
 * @returns {Promise<Object>} 最適化結果
 */
async function optimizeImageWithAPI(file) {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('black_threshold', '100');
    formData.append('white_threshold', '220');

    try {
        const response = await fetch(`${API_BASE_URL}/optimize-image`, {
            method: 'POST',
            body: formData
        });

        if (!response.ok) {
            throw new Error(`API error: ${response.statusText}`);
        }

        const result = await response.json();
        return result;
    } catch (error) {
        console.error('optimizeImageWithAPI: エラー', error);
        throw error;
    }
}

/**
 * 通常の画像ファイルを読み込む
 * @export
 * @param {File} file - 画像ファイル
 */
export async function loadStandardImageFile(file) {
    try {
        console.log(`loadStandardImageFile: 画像を最適化中...`);

        // バックエンドAPIで画像を最適化
        const optimizedResult = await optimizeImageWithAPI(file);

        if (!optimizedResult.success) {
            throw new Error(optimizedResult.error || '画像の最適化に失敗しました');
        }

        console.log(`loadStandardImageFile: 元のサイズ ${optimizedResult.original_size.width}x${optimizedResult.original_size.height}`);
        console.log(`loadStandardImageFile: 最適化後のサイズ ${optimizedResult.cropped_size.width}x${optimizedResult.cropped_size.height}`);

        const reductionPercent = ((1 - (optimizedResult.cropped_size.width * optimizedResult.cropped_size.height) /
                                      (optimizedResult.original_size.width * optimizedResult.original_size.height)) * 100).toFixed(1);
        console.log(`loadStandardImageFile: ${reductionPercent}% 削減`);

        // 最適化された画像を読み込む
        const img = new Image();
        img.onload = function() {
            mapState.image = img;
            mapState.imageFileName = file.name;
            mapState.layers.image = true;

            // クロップオフセット情報を保存
            mapState.imageCropOffset = optimizedResult.offset;
            mapState.originalImageSize = optimizedResult.original_size;

            const container = document.getElementById('mapContainer');
            const canvas = document.getElementById('mapCanvas');
            const canvasStack = document.getElementById('canvasStack');

            // キャンバスのサイズを設定
            const containerRect = container.getBoundingClientRect();
            canvas.width = containerRect.width;
            canvas.height = containerRect.height;

            // 有効領域にフィットするようにビューを調整
            fitViewToBounds(img, container);

            // プレースホルダーを非表示、新しいレイヤーシステムを表示
            document.getElementById('mapPlaceholder').style.display = 'none';
            canvasStack.style.display = 'block';
            canvas.style.display = 'none';  // 古いcanvasは非表示に

            // レイヤーシステムを初期化
            initializeLayers();

            // オーバーレイコントロールを更新
            updateOverlayControls();

            // すべてのレイヤーを再描画
            redrawAllLayers();

            // ステータスバーを更新
            if (window.updateStatusBar && typeof window.updateStatusBar === 'function') {
                window.updateStatusBar();
            }
        };

        img.onerror = function() {
            console.error('loadStandardImageFile: 画像の読み込みに失敗しました');
            alert('最適化された画像の読み込みに失敗しました');
        };

        img.src = optimizedResult.image;

    } catch (error) {
        console.error('loadStandardImageFile: エラー', error);
        alert(`画像の読み込みに失敗しました: ${error.message}`);
    }
}

/**
 * メタデータのorigin座標をクロップオフセットに応じて調整
 * @private
 * @param {Object} metadata - メタデータ
 */
function adjustMetadataForCrop(metadata) {
    if (!metadata || !mapState.imageCropOffset) return;

    const cropOffset = mapState.imageCropOffset;
    const resolution = metadata.resolution || 0.05;

    // クロップオフセットがある場合、メタデータのorigin座標を調整
    if (cropOffset.x !== 0 || cropOffset.y !== 0) {
        if (metadata.origin) {
            // 元のorigin座標を保存（デバッグ用）
            if (!metadata._originalOrigin) {
                metadata._originalOrigin = { ...metadata.origin };
            }

            // ROSマップは左下が原点、画像は左上が原点なので Y 軸の変換に注意
            // クロップによって画像が小さくなった場合、originも調整する必要がある
            const originalHeight = mapState.originalImageSize?.height || mapState.image.height;

            // X方向: 右方向が正なので、クロップオフセット分を加算
            metadata.origin.x = metadata._originalOrigin.x + cropOffset.x * resolution;

            // Y方向: ROSは左下原点なので、クロップによって下側が切り取られた場合は調整
            // クロップオフセットは画像の左上からの距離なので、originへの影響は逆
            metadata.origin.y = metadata._originalOrigin.y + cropOffset.y * resolution;

            console.log(`adjustMetadataForCrop: origin調整 (${metadata._originalOrigin.x}, ${metadata._originalOrigin.y}) → (${metadata.origin.x}, ${metadata.origin.y})`);
        }
    }
}

/**
 * YAMLメタデータファイルを読み込む
 * @export
 * @param {File} file - YAMLファイル
 */
export function loadYAMLMetadataFile(file) {
    const reader = new FileReader();
    reader.onload = function(e) {
        try {
            const yamlText = e.target.result;
            const metadata = jsyaml.load(yamlText);

            // メタデータを保存
            mapState.metadata = metadata;
            mapState.yamlFileName = file.name;
            mapState.layers.metadataOverlay = true;

            // クロップオフセットがある場合、メタデータを調整
            adjustMetadataForCrop(metadata);

            // メタデータを表示
            displayMetadata(metadata);

            // オーバーレイコントロールを更新
            updateOverlayControls();

            // すべてのレイヤーを再描画（メタデータオーバーレイを含む）
            redrawAllLayers();

            // ステータスバーを更新
            if (window.updateStatusBar && typeof window.updateStatusBar === 'function') {
                window.updateStatusBar();
            }
        } catch (error) {
            console.error('YAMLファイルの読み込みに失敗:', error);
            alert('YAMLファイルの読み込みに失敗しました: ' + error.message);
        }
    };
    reader.readAsText(file);
}

/**
 * PGM画像ファイルを読み込む
 * @export
 * @param {File} file - PGMファイル
 */
export async function loadPGMImageFile(file) {
    try {
        console.log(`loadPGMImageFile: PGM画像を最適化中...`);

        // バックエンドAPIで画像を最適化
        const optimizedResult = await optimizeImageWithAPI(file);

        if (!optimizedResult.success) {
            throw new Error(optimizedResult.error || 'PGM画像の最適化に失敗しました');
        }

        console.log(`loadPGMImageFile: 元のサイズ ${optimizedResult.original_size.width}x${optimizedResult.original_size.height}`);
        console.log(`loadPGMImageFile: 最適化後のサイズ ${optimizedResult.cropped_size.width}x${optimizedResult.cropped_size.height}`);

        const reductionPercent = ((1 - (optimizedResult.cropped_size.width * optimizedResult.cropped_size.height) /
                                      (optimizedResult.original_size.width * optimizedResult.original_size.height)) * 100).toFixed(1);
        console.log(`loadPGMImageFile: ${reductionPercent}% 削減`);

        // 最適化された画像を読み込む
        const img = new Image();
        img.onload = function() {
            mapState.image = img;
            mapState.imageFileName = file.name;
            mapState.layers.image = true;

            // クロップオフセット情報を保存
            mapState.imageCropOffset = optimizedResult.offset;
            mapState.originalImageSize = optimizedResult.original_size;

            const container = document.getElementById('mapContainer');
            const canvas = document.getElementById('mapCanvas');
            const canvasStack = document.getElementById('canvasStack');

            // キャンバスのサイズを設定
            const containerRect = container.getBoundingClientRect();
            canvas.width = containerRect.width;
            canvas.height = containerRect.height;

            // 有効領域にフィットするようにビューを調整
            fitViewToBounds(img, container);

            // プレースホルダーを非表示、新しいレイヤーシステムを表示
            document.getElementById('mapPlaceholder').style.display = 'none';
            canvasStack.style.display = 'block';
            canvas.style.display = 'none';  // 古いcanvasは非表示に

            // レイヤーシステムを初期化
            initializeLayers();

            // オーバーレイコントロールを更新
            updateOverlayControls();

            // すべてのレイヤーを再描画
            redrawAllLayers();

            // ステータスバーを更新
            if (window.updateStatusBar && typeof window.updateStatusBar === 'function') {
                window.updateStatusBar();
            }
        };

        img.onerror = function() {
            console.error('loadPGMImageFile: 画像の読み込みに失敗しました');
            alert('最適化されたPGM画像の読み込みに失敗しました');
        };

        img.src = optimizedResult.image;

    } catch (error) {
        console.error('loadPGMImageFile: エラー', error);
        alert(`PGM画像の読み込みに失敗しました: ${error.message}`);
    }
}

/**
 * 現在のマップをPGMファイルとして保存
 *
 * すべてのレイヤーを統合したキャンバスをPGM形式で保存します。
 * ファイル名は、インポートしたファイル名をベースにするか、デフォルト名を使用します。
 * 保存時には有効領域の自動クロップは行いません（ユーザーが描画したアノテーションを保持するため）。
 *
 * @export
 * @param {string} [filename] - 保存するファイル名（省略時は自動生成）
 */
export function saveMapAsPGM(filename) {
    try {
        // 画像が読み込まれていない場合はエラー
        if (!mapState.image) {
            throw new Error('保存する画像がありません');
        }

        // すべての可視レイヤーを統合したキャンバスを作成
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = mapState.image.width;
        tempCanvas.height = mapState.image.height;
        const ctx = tempCanvas.getContext('2d');

        // 背景を白で塗りつぶし
        ctx.fillStyle = '#FFFFFF';
        ctx.fillRect(0, 0, tempCanvas.width, tempCanvas.height);

        // レイヤースタックを下から順に描画
        mapState.layerStack.forEach(layer => {
            if (layer.visible && layer.canvas) {
                // 元のサイズでレイヤーを描画
                ctx.globalAlpha = layer.opacity || 1.0;
                ctx.drawImage(layer.canvas, 0, 0);
            }
        });

        // アルファを戻す
        ctx.globalAlpha = 1.0;

        // PGMデータに変換
        const pgmData = canvasToPGM(tempCanvas);

        console.log(`saveMapAsPGM: 保存サイズ ${pgmData.length} bytes (${tempCanvas.width}x${tempCanvas.height})`);

        // ファイル名を決定
        let finalFilename = filename;
        if (!finalFilename) {
            // 画像ファイル名がある場合はそれを使用
            if (mapState.imageFileName) {
                const baseName = mapState.imageFileName.replace(/\.[^/.]+$/, '');
                finalFilename = `${baseName}_exported.pgm`;
            } else {
                finalFilename = 'map_exported.pgm';
            }
        }

        // ファイルをダウンロード
        downloadPGM(pgmData, finalFilename);

        console.log('saveMapAsPGM: PGMファイル保存成功', finalFilename);
    } catch (error) {
        console.error('saveMapAsPGM: 保存エラー', error);
        throw error;
    }
}
