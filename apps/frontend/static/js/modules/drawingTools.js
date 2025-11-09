/**
 * @file drawingTools.js
 * @description 描画ツール関連の機能を提供するモジュール
 * @requires ../state/mapState.js
 * @exports toggleDrawingTools
 * @exports toggleLayersPanel
 * @exports selectTool
 * @exports updateCursor
 * @exports changeDrawingColor
 * @exports changeBrushSize
 * @exports getActiveDrawingLayer
 * @exports performDrawing
 * @exports redrawDrawingLayer
 * @exports showMeasureDistance
 * @exports performBucketFill
 * @exports isUIElement
 * @exports updateColorPickerState
 */

import { mapState } from '../state/mapState.js';
import {
    canvasToImagePixel as canvasToImagePixelUtil,
    imagePixelToCanvas as imagePixelToCanvasUtil
} from '../utils/coordinates.js';
import { isUIElement } from '../utils/dom.js';

/**
 * キャンバス座標をマップ画像ピクセル座標に変換
 * @export
 * @param {number} canvasX - キャンバスX座標
 * @param {number} canvasY - キャンバスY座標
 * @returns {{x: number, y: number}} 画像ピクセル座標
 */
export function canvasToImagePixel(canvasX, canvasY) {
    const container = document.getElementById('mapContainer');
    return canvasToImagePixelUtil(canvasX, canvasY, container);
}

/**
 * マップ画像ピクセル座標をキャンバス座標に変換
 * @export
 * @param {number} imagePixelX - 画像ピクセルX座標
 * @param {number} imagePixelY - 画像ピクセルY座標
 * @returns {{x: number, y: number}} キャンバス座標
 */
export function imagePixelToCanvas(imagePixelX, imagePixelY) {
    const container = document.getElementById('mapContainer');
    return imagePixelToCanvasUtil(imagePixelX, imagePixelY, container);
}

/**
 * HEX色をRGBに変換
 * @param {string} hex - HEX色コード
 * @returns {{r: number, g: number, b: number}|null} RGB色オブジェクト
 */
function hexToRgb(hex) {
    const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}

/**
 * 描画ツールパレットの表示/非表示を切り替え
 * @export
 */
export function toggleDrawingTools() {
    const palette = document.getElementById('drawingToolsPalette');
    palette.classList.toggle('active');
}

/**
 * レイヤーパネルの表示/非表示を切り替え
 * @export
 */
export function toggleLayersPanel() {
    const panel = document.getElementById('layersPanel');
    panel.classList.toggle('active');
}

/**
 * 描画ツールを選択
 * @export
 * @param {string} toolName - ツール名 ('pan', 'pencil', 'eraser', 'measure', 'bucket', 'rectangle')
 */
export function selectTool(toolName) {
    mapState.drawingState.currentTool = toolName;

    // すべてのツールボタンからactiveクラスを削除
    document.querySelectorAll('.tool-button').forEach(btn => {
        btn.classList.remove('active');
    });

    // 選択されたツールにactiveクラスを追加
    const toolButton = document.getElementById(toolName + 'Tool');
    if (toolButton) {
        toolButton.classList.add('active');
    }

    // カーソルスタイルを変更
    updateCursor();
}

/**
 * 四角形ツールモードをトグルする
 * 四角形ツールボタン専用の関数
 * @export
 */
export function toggleRectangleToolMode() {
    // 現在の四角形ツールの状態を取得
    const currentState = mapState.rectangleToolState ? mapState.rectangleToolState.enabled : false;
    const newState = !currentState;

    // 四角形ツールをトグル
    if (window.toggleRectangleTool && typeof window.toggleRectangleTool === 'function') {
        window.toggleRectangleTool(newState);
    }

    // 四角形ツールがオンになった場合
    if (newState) {
        // 四角形レイヤーを表示
        if (window.getRectangleLayer && typeof window.getRectangleLayer === 'function') {
            const rectangleLayer = window.getRectangleLayer();
            if (rectangleLayer) {
                rectangleLayer.visible = true;
                rectangleLayer.canvas.style.display = 'block';

                // レイヤーパネルを更新
                if (window.updateLayersPanel && typeof window.updateLayersPanel === 'function') {
                    window.updateLayersPanel();
                }
            }
        }

        // 四角形が1つもない場合は、中央に作成
        if (window.getAllRectangles && typeof window.getAllRectangles === 'function') {
            const rectangles = window.getAllRectangles();
            if (rectangles.length === 0 && window.createRectangle && window.getImageCenter &&
                typeof window.createRectangle === 'function' && typeof window.getImageCenter === 'function') {
                const center = window.getImageCenter();
                if (center) {
                    // 2m×2mのサイズを計算（メタデータから解像度を取得）
                    const resolution = mapState.metadata ? mapState.metadata.resolution : 0.05; // m/pixel
                    const widthInMeters = 2.0;   // 2m
                    const heightInMeters = 2.0;  // 2m
                    const widthInPixels = widthInMeters / resolution;
                    const heightInPixels = heightInMeters / resolution;

                    window.createRectangle(center.x, center.y, widthInPixels, heightInPixels);

                    // 作成した四角形を選択
                    if (window.getAllRectangles && window.selectRectangle &&
                        typeof window.selectRectangle === 'function') {
                        const rects = window.getAllRectangles();
                        if (rects.length > 0) {
                            window.selectRectangle(rects[0].id);
                        }
                    }

                    // レイヤーを再描画
                    if (window.getRectangleLayer && window.redrawRectangleLayer &&
                        typeof window.getRectangleLayer === 'function' &&
                        typeof window.redrawRectangleLayer === 'function') {
                        const layer = window.getRectangleLayer();
                        if (layer) {
                            window.redrawRectangleLayer(layer);
                        }
                    }
                }
            }
        }

        // パンツールを選択（四角形の移動・回転ができるようにする）
        selectTool('pan');
    }
}

/**
 * カーソルスタイルを更新
 * @export
 */
export function updateCursor() {
    const container = document.getElementById('mapContainer');
    const tool = mapState.drawingState.currentTool;

    switch(tool) {
        case 'pan':
            container.style.cursor = 'grab';
            break;
        case 'pencil':
            container.style.cursor = 'crosshair';
            break;
        case 'eraser':
            container.style.cursor = 'cell';
            break;
        case 'measure':
            container.style.cursor = 'crosshair';
            break;
        case 'bucket':
            container.style.cursor = 'pointer';
            break;
        case 'rectangle':
            container.style.cursor = 'crosshair';
            break;
        default:
            container.style.cursor = 'default';
    }
}

/**
 * 描画色を変更
 * @export
 * @param {string} color - HEX色コード
 */
export function changeDrawingColor(color) {
    mapState.drawingState.color = color;

    // カラーアイコンの表示色を更新
    const colorIconDisplay = document.getElementById('colorIconDisplay');
    if (colorIconDisplay) {
        colorIconDisplay.style.backgroundColor = color;
    }

    // RGBピッカーの色も同期
    const colorPicker = document.getElementById('colorPicker');
    if (colorPicker) {
        colorPicker.value = color;
    }
}

/**
 * ブラシサイズを変更
 * @export
 * @param {number} size - ブラシサイズ
 */
export function changeBrushSize(size) {
    mapState.drawingState.brushSize = parseInt(size);
    const brushSizeValue = document.getElementById('brushSizeValue');
    if (brushSizeValue) {
        brushSizeValue.textContent = size;
    }
}

/**
 * カラーピッカーポップアップの表示/非表示を切り替え
 * @export
 */
export function toggleColorPicker() {
    // 四角形ツールが有効な場合は、カラーピッカーを開けない
    if (mapState.rectangleToolState && mapState.rectangleToolState.enabled) {
        return;
    }

    const popup = document.getElementById('colorPickerPopup');
    if (popup) {
        popup.classList.toggle('hidden');
    }
}

/**
 * カラーピッカーポップアップを閉じる
 * @export
 */
export function closeColorPicker() {
    const popup = document.getElementById('colorPickerPopup');
    if (popup) {
        popup.classList.add('hidden');
    }
}

/**
 * カラーピッカーのグローバルクリックハンドラーをセットアップ
 * @export
 */
export function setupColorPickerClickHandler() {
    document.addEventListener('click', (e) => {
        const popup = document.getElementById('colorPickerPopup');
        const colorIconButton = document.getElementById('colorIconButton');

        if (!popup || !colorIconButton) return;

        // ポップアップが表示されている場合のみ
        if (!popup.classList.contains('hidden')) {
            // クリックされた要素がポップアップ内またはカラーアイコンボタンでない場合
            if (!popup.contains(e.target) && !colorIconButton.contains(e.target)) {
                closeColorPicker();
            }
        }
    });
}

/**
 * カラーモードを切り替え（パレット/RGB）
 * @export
 * @param {string} mode - モード ('palette' または 'rgb')
 */
export function switchColorMode(mode) {
    const paletteMode = document.getElementById('paletteMode');
    const rgbMode = document.getElementById('rgbMode');
    const colorPalette = document.getElementById('colorPalette');
    const colorRGB = document.getElementById('colorRGB');

    if (mode === 'palette') {
        // パレットモードに切り替え
        paletteMode.classList.add('active');
        rgbMode.classList.remove('active');
        colorPalette.classList.remove('hidden');
        colorRGB.classList.add('hidden');
    } else if (mode === 'rgb') {
        // RGBモードに切り替え
        paletteMode.classList.remove('active');
        rgbMode.classList.add('active');
        colorPalette.classList.add('hidden');
        colorRGB.classList.remove('hidden');
    }
}

/**
 * パレットから色を選択
 * @export
 * @param {string} color - HEX色コード
 */
export function selectPaletteColor(color) {
    // 描画色を変更
    changeDrawingColor(color);

    // すべてのパレット色のactiveクラスを削除
    document.querySelectorAll('.palette-color').forEach(btn => {
        btn.classList.remove('active');
    });

    // 選択された色にactiveクラスを追加
    const selectedColor = document.querySelector(`.palette-color[data-color="${color}"]`);
    if (selectedColor) {
        selectedColor.classList.add('active');
    }

    // RGBピッカーの色も同期
    const colorPicker = document.getElementById('colorPicker');
    if (colorPicker) {
        colorPicker.value = color;
    }
}

/**
 * カラーピッカーの有効/無効状態を更新
 * 四角形ツールが有効な場合は、カラーピッカーを無効化（グレーアウト）
 * @export
 */
export function updateColorPickerState() {
    const colorIconButton = document.getElementById('colorIconButton');
    const isRectangleToolEnabled = mapState.rectangleToolState && mapState.rectangleToolState.enabled;

    if (colorIconButton) {
        if (isRectangleToolEnabled) {
            // 四角形ツールが有効な場合は無効化
            colorIconButton.classList.add('disabled');
            colorIconButton.style.opacity = '0.5';
            colorIconButton.style.cursor = 'not-allowed';
            colorIconButton.title = '四角形ツール使用中は色の変更はできません（色はカテゴリに基づいて自動的に決定されます）';
        } else {
            // 四角形ツールが無効な場合は有効化
            colorIconButton.classList.remove('disabled');
            colorIconButton.style.opacity = '';
            colorIconButton.style.cursor = '';
            colorIconButton.title = '色を選択';
        }
    }

    // カラーピッカーポップアップが開いている場合は閉じる
    if (isRectangleToolEnabled) {
        closeColorPicker();
    }
}

// isUIElement は utils/dom.js から再エクスポート（後方互換性のため）
export { isUIElement };

/**
 * アクティブな描画レイヤーを取得（なければ作成）
 * @export
 * @returns {Object|null} 描画レイヤーオブジェクト
 */
export function getActiveDrawingLayer() {
    // 選択されているレイヤーが描画レイヤーの場合はそれを使用
    if (mapState.selectedLayerId) {
        const selectedLayer = mapState.layerStack.find(l => l.id === mapState.selectedLayerId);
        if (selectedLayer && selectedLayer.type === 'drawing' && selectedLayer.visible) {
            return selectedLayer;
        }
    }

    // 選択されていない、または描画レイヤーでない場合は、表示中の描画レイヤーを検索
    let drawingLayer = mapState.layerStack.find(l => l.type === 'drawing' && l.visible);

    // 表示中の描画レイヤーがなければ、非表示も含めて検索
    if (!drawingLayer) {
        drawingLayer = mapState.layerStack.find(l => l.type === 'drawing');
        // 見つかったら表示する
        if (drawingLayer) {
            drawingLayer.visible = true;
            drawingLayer.canvas.style.display = 'block';
            // updateLayersPanel() を呼び出す必要があるが、循環参照を避けるため外部で呼び出す
        }
    }

    // 描画レイヤーがなければ作成
    if (!drawingLayer) {
        // addNewLayer() を呼び出す必要があるが、循環参照を避けるため外部で呼び出す
        // ここでは null を返す
        return null;
    }

    return drawingLayer;
}

/**
 * 鉛筆または消しゴムで描画
 * @export
 * @param {string} tool - ツール名 ('pencil' または 'eraser')
 * @param {Array<{x: number, y: number}>} imagePixelStroke - 画像ピクセル座標のストローク配列
 * @param {string} color - 描画色
 * @param {number} brushSize - ブラシサイズ
 */
export function performDrawing(tool, imagePixelStroke, color, brushSize) {
    if (imagePixelStroke.length < 2) return;

    const layer = getActiveDrawingLayer();
    if (!layer) return;

    const ctx = layer.ctx;

    // 最後の2点をキャンバス座標に変換
    const ip1 = imagePixelStroke[imagePixelStroke.length - 2];
    const ip2 = imagePixelStroke[imagePixelStroke.length - 1];

    const p1 = imagePixelToCanvas(ip1.x, ip1.y);
    const p2 = imagePixelToCanvas(ip2.x, ip2.y);

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    if (tool === 'pencil') {
        ctx.strokeStyle = color;
        ctx.lineWidth = brushSize;
        ctx.globalCompositeOperation = 'source-over';
    } else if (tool === 'eraser') {
        // 消しゴムは透明にする
        ctx.globalCompositeOperation = 'destination-out';
        ctx.lineWidth = brushSize * 2;  // 消しゴムは大きめ
    }

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();

    ctx.restore();
}

/**
 * 描画レイヤーを再描画（パン/ズーム時）
 * @export
 * @param {Object} layer - レイヤーオブジェクト
 */
export function redrawDrawingLayer(layer) {
    if (!layer || layer.type !== 'drawing') return;

    const ctx = layer.ctx;

    // キャンバスをクリア
    ctx.clearRect(0, 0, layer.canvas.width, layer.canvas.height);

    // ストロークがある場合は再描画
    if (layer.strokes && layer.strokes.length > 0) {
        // すべてのストロークを再描画
        for (const strokeData of layer.strokes) {
            const {tool, stroke, color, brushSize} = strokeData;

            if (stroke.length < 2) continue;

            ctx.save();
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';

            if (tool === 'pencil') {
                ctx.strokeStyle = color;
                ctx.lineWidth = brushSize;
                ctx.globalCompositeOperation = 'source-over';
            } else if (tool === 'eraser') {
                ctx.globalCompositeOperation = 'destination-out';
                ctx.lineWidth = brushSize * 2;
            }

            ctx.beginPath();

            // 最初の点に移動
            const startPoint = imagePixelToCanvas(stroke[0].x, stroke[0].y);
            ctx.moveTo(startPoint.x, startPoint.y);

            // すべての点を描画
            for (let i = 1; i < stroke.length; i++) {
                const point = imagePixelToCanvas(stroke[i].x, stroke[i].y);
                ctx.lineTo(point.x, point.y);
            }

            ctx.stroke();
            ctx.restore();
        }
    }
}

/**
 * 測量結果を表示
 * @export
 */
export function showMeasureDistance() {
    const points = mapState.drawingState.measurePoints;
    if (points.length !== 2) return;

    // ピクセル距離を計算
    const dx = points[1].x - points[0].x;
    const dy = points[1].y - points[0].y;
    const pixelDistance = Math.sqrt(dx * dx + dy * dy);

    // メタデータがあれば実距離を計算
    let message = `ピクセル距離: ${Math.round(pixelDistance)} px`;

    if (mapState.metadata && mapState.metadata.resolution) {
        const resolution = mapState.metadata.resolution;  // m/pixel
        const realDistance = (pixelDistance / mapState.scale) * resolution;
        message = `距離: ${realDistance.toFixed(2)} m (${Math.round(pixelDistance)} px)`;
    }

    alert(message);
}

/**
 * バケツ塗りつぶし
 * @export
 * @param {number} x - キャンバスX座標
 * @param {number} y - キャンバスY座標
 */
export function performBucketFill(x, y) {
    const layer = getActiveDrawingLayer();
    if (!layer) return;

    const ctx = layer.ctx;
    const width = layer.canvas.width;
    const height = layer.canvas.height;

    // 現在のピクセルデータを取得
    const imageData = ctx.getImageData(0, 0, width, height);
    const data = imageData.data;

    // クリック位置のピクセル座標
    const pixelX = Math.floor(x);
    const pixelY = Math.floor(y);

    if (pixelX < 0 || pixelX >= width || pixelY < 0 || pixelY >= height) {
        return;
    }

    // クリック位置の色を取得
    const pixelIndex = (pixelY * width + pixelX) * 4;
    const targetR = data[pixelIndex];
    const targetG = data[pixelIndex + 1];
    const targetB = data[pixelIndex + 2];
    const targetA = data[pixelIndex + 3];

    // 塗りつぶし色を取得
    const fillColor = hexToRgb(mapState.drawingState.color);
    if (!fillColor) return;

    // 同じ色の場合は何もしない
    if (targetR === fillColor.r && targetG === fillColor.g && targetB === fillColor.b && targetA === 255) {
        return;
    }

    // フラッドフィル（幅優先探索）
    const stack = [{x: pixelX, y: pixelY}];
    const visited = new Set();

    while (stack.length > 0) {
        const {x, y} = stack.pop();
        const key = `${x},${y}`;

        if (visited.has(key)) continue;
        if (x < 0 || x >= width || y < 0 || y >= height) continue;

        const index = (y * width + x) * 4;

        // 対象色と一致するかチェック
        if (
            data[index] === targetR &&
            data[index + 1] === targetG &&
            data[index + 2] === targetB &&
            data[index + 3] === targetA
        ) {
            // 塗りつぶし
            data[index] = fillColor.r;
            data[index + 1] = fillColor.g;
            data[index + 2] = fillColor.b;
            data[index + 3] = 255;

            visited.add(key);

            // 隣接ピクセルをスタックに追加
            stack.push({x: x + 1, y: y});
            stack.push({x: x - 1, y: y});
            stack.push({x: x, y: y + 1});
            stack.push({x: x, y: y - 1});
        }

        // スタックサイズ制限（パフォーマンス対策）
        if (visited.size > 100000) {
            alert('塗りつぶし領域が大きすぎます');
            break;
        }
    }

    // 画像データを書き戻す
    ctx.putImageData(imageData, 0, 0);
}
