/**
 * @file fileLoader.js
 * @description ファイル読み込み関連の機能を提供するモジュール
 * @requires ../state/mapState.js
 * @exports handleImageFileSelect
 * @exports handleYAMLFileSelect
 * @exports loadStandardImageFile
 * @exports loadYAMLMetadataFile
 * @exports loadPGMImageFile
 * @exports parsePGM
 * @exports pgmToImage
 */

import { mapState } from '../state/mapState.js';

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
 * 通常の画像ファイルを読み込む
 * @export
 * @param {File} file - 画像ファイル
 */
export function loadStandardImageFile(file) {
    const reader = new FileReader();
    reader.onload = function(e) {
        const img = new Image();
        img.onload = function() {
            mapState.image = img;
            mapState.scale = 1.0;
            mapState.offsetX = 0;
            mapState.offsetY = 0;
            mapState.layers.image = true;

            const container = document.getElementById('mapContainer');
            const canvas = document.getElementById('mapCanvas');
            const canvasStack = document.getElementById('canvasStack');

            // キャンバスのサイズを設定
            const containerRect = container.getBoundingClientRect();
            canvas.width = containerRect.width;
            canvas.height = containerRect.height;

            // プレースホルダーを非表示、新しいレイヤーシステムを表示
            document.getElementById('mapPlaceholder').style.display = 'none';
            canvasStack.style.display = 'block';
            canvas.style.display = 'none';  // 古いcanvasは非表示に

            // initializeLayers(), updateOverlayControls(), redrawAllLayers() は外部で呼び出す必要がある
        };
        img.src = e.target.result;
    };
    reader.readAsDataURL(file);
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
            mapState.layers.metadataOverlay = true;

            // displayMetadata(), updateOverlayControls(), redrawAllLayers() は外部で呼び出す必要がある
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
export function loadPGMImageFile(file) {
    const reader = new FileReader();
    reader.onload = function(e) {
        try {
            const arrayBuffer = e.target.result;
            const uint8Array = new Uint8Array(arrayBuffer);

            // PGMフォーマットをパース
            const pgmData = parsePGM(uint8Array);

            // PGMデータをImageに変換
            const img = pgmToImage(pgmData);

            img.onload = function() {
                mapState.image = img;
                mapState.scale = 1.0;
                mapState.offsetX = 0;
                mapState.offsetY = 0;
                mapState.layers.image = true;

                const container = document.getElementById('mapContainer');
                const canvas = document.getElementById('mapCanvas');
                const canvasStack = document.getElementById('canvasStack');

                // キャンバスのサイズを設定
                const containerRect = container.getBoundingClientRect();
                canvas.width = containerRect.width;
                canvas.height = containerRect.height;

                // プレースホルダーを非表示、新しいレイヤーシステムを表示
                document.getElementById('mapPlaceholder').style.display = 'none';
                canvasStack.style.display = 'block';
                canvas.style.display = 'none';  // 古いcanvasは非表示に

                // initializeLayers(), updateOverlayControls(), redrawAllLayers() は外部で呼び出す必要がある
            };
        } catch (error) {
            console.error('PGMファイルの読み込みに失敗:', error);
            alert('PGMファイルの読み込みに失敗しました: ' + error.message);
        }
    };
    reader.readAsArrayBuffer(file);
}
