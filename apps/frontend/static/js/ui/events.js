/**
 * @file events.js
 * @description イベントリスナーのセットアップを管理するモジュール
 * @requires ../state/mapState.js - マップの状態管理
 * @requires ../utils/coordinates.js - 座標変換ユーティリティ
 * @requires ./statusBar.js - ステータスバー更新
 * @exports setupEventListeners - すべてのイベントリスナーをセットアップする関数
 */

import { mapState } from '../state/mapState.js';
import { canvasToWorld } from '../utils/coordinates.js';
import { updateCursorPosition } from './statusBar.js';

/**
 * すべてのイベントリスナーをセットアップする
 */
export function setupEventListeners() {
    const container = document.getElementById('mapContainer');
    const canvas = document.getElementById('mapCanvas');
    const canvasStack = document.getElementById('canvasStack');

    if (!container || !canvas) {
        console.error('Required DOM elements not found');
        return;
    }

    // ウィンドウリサイズイベント
    window.addEventListener('resize', handleResize);

    // キーボードショートカット（Undo/Redo）
    document.addEventListener('keydown', handleKeydown);

    // キャンバスコンテナのマウスイベント
    container.addEventListener('wheel', handleWheel);
    container.addEventListener('mousedown', handleMouseDown);
    container.addEventListener('mousemove', handleMouseMove);
    container.addEventListener('mouseup', handleMouseUp);
    container.addEventListener('mouseleave', handleMouseLeave);

    // メタデータパネルのスクロールがマップに影響しないようにする
    const metadataOverlayBody = document.getElementById('metadataOverlayBody');
    if (metadataOverlayBody) {
        metadataOverlayBody.addEventListener('wheel', function(e) {
            // メタデータパネル内でスクロール中の場合、イベントの伝播を止める
            e.stopPropagation();
        }, { passive: false });
    }

    // レイヤーパネルのスクロールがマップに影響しないようにする
    const layersPanelBody = document.getElementById('layersPanelBody');
    if (layersPanelBody) {
        layersPanelBody.addEventListener('wheel', function(e) {
            // レイヤーパネル内でスクロール中の場合、イベントの伝播を止める
            e.stopPropagation();
        }, { passive: false });
    }

    // クエリ入力のEnterキー処理
    const queryInput = document.getElementById('queryInput');
    if (queryInput) {
        queryInput.addEventListener('keypress', function(e) {
            if (e.key === 'Enter' && window.executeQuery && typeof window.executeQuery === 'function') {
                window.executeQuery();
            }
        });
    }

    console.log('Event listeners setup complete');
}

/**
 * ウィンドウリサイズハンドラー
 */
function handleResize() {
    const container = document.getElementById('mapContainer');
    const canvas = document.getElementById('mapCanvas');
    const canvasStack = document.getElementById('canvasStack');

    if (mapState.image && container) {
        const containerRect = container.getBoundingClientRect();

        // 新しいレイヤーシステムの場合
        if (mapState.layerStack.length > 0) {
            // すべてのレイヤーのキャンバスサイズを更新
            mapState.layerStack.forEach(layer => {
                // 既存の描画内容を保存
                const imageData = layer.ctx.getImageData(0, 0, layer.canvas.width, layer.canvas.height);

                // キャンバスサイズを変更
                layer.canvas.width = containerRect.width;
                layer.canvas.height = containerRect.height;

                // 描画内容を復元（描画レイヤーのみ）
                if (layer.type === 'drawing') {
                    layer.ctx.putImageData(imageData, 0, 0);
                }
            });

            if (window.redrawAllLayers && typeof window.redrawAllLayers === 'function') {
                window.redrawAllLayers();
            }
        } else {
            // 旧システムの場合
            if (canvas) {
                canvas.width = containerRect.width;
                canvas.height = containerRect.height;

                if (window.drawMap && typeof window.drawMap === 'function') {
                    window.drawMap();
                }
            }
        }
    }
}

/**
 * キーボードショートカットハンドラー
 */
function handleKeydown(e) {
    // Ctrl+S (Save)
    if (e.ctrlKey && e.key === 's') {
        e.preventDefault();
        if (window.quickSave && typeof window.quickSave === 'function') {
            window.quickSave();
        }
    }
    // Ctrl+Z (Undo)
    else if (e.ctrlKey && e.key === 'z' && !e.shiftKey) {
        e.preventDefault();
        if (window.undo && typeof window.undo === 'function') {
            window.undo();
        }
    }
    // Ctrl+Shift+Z (Redo) または Ctrl+Y (Redo)
    else if ((e.ctrlKey && e.shiftKey && e.key === 'z') || (e.ctrlKey && e.key === 'y')) {
        e.preventDefault();
        if (window.redo && typeof window.redo === 'function') {
            window.redo();
        }
    }
}

/**
 * マウスホイールハンドラー（ズーム）
 */
function handleWheel(e) {
    const container = document.getElementById('mapContainer');
    const canvas = document.getElementById('mapCanvas');

    if (!mapState.image) return;

    e.preventDefault();

    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    const newScale = mapState.scale * delta;

    if (newScale >= mapState.minScale && newScale <= mapState.maxScale) {
        // マウス位置を中心にズーム
        const rect = container.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        // 現在のキャンバス幅/高さを取得
        const currentWidth = mapState.layerStack.length > 0 ? mapState.layerStack[0].canvas.width : canvas.width;
        const currentHeight = mapState.layerStack.length > 0 ? mapState.layerStack[0].canvas.height : canvas.height;

        // ズーム前のマウス位置（画像座標）
        const imgX = (mouseX - currentWidth / 2 - mapState.offsetX) / mapState.scale;
        const imgY = (mouseY - currentHeight / 2 - mapState.offsetY) / mapState.scale;

        // スケール更新
        mapState.scale = newScale;

        // ズーム後、同じ画像座標がマウス位置に来るようオフセットを調整
        mapState.offsetX = mouseX - currentWidth / 2 - imgX * mapState.scale;
        mapState.offsetY = mouseY - currentHeight / 2 - imgY * mapState.scale;

        // レイヤーシステムが有効な場合
        if (mapState.layerStack.length > 0) {
            if (window.redrawAllLayers && typeof window.redrawAllLayers === 'function') {
                window.redrawAllLayers();
            }
        } else {
            if (window.drawMap && typeof window.drawMap === 'function') {
                window.drawMap();
            }
        }
    }
}

/**
 * マウスダウンハンドラー
 */
function handleMouseDown(e) {
    const container = document.getElementById('mapContainer');

    if (!mapState.image) return;

    const tool = mapState.drawingState.currentTool;

    // 左ボタンクリック
    if (e.button === 0) {
        // UIコントロール要素の場合はスキップ
        if (isUIElement(e.target)) {
            return;
        }

        e.preventDefault();

        const rect = container.getBoundingClientRect();
        const canvasX = e.clientX - rect.left;
        const canvasY = e.clientY - rect.top;

        // 四角形ツールが有効な場合、四角形イベントハンドラーを呼び出す
        if (mapState.rectangleToolState && mapState.rectangleToolState.enabled) {
            if (window.handleRectangleMouseDown && typeof window.handleRectangleMouseDown === 'function') {
                const handled = window.handleRectangleMouseDown(canvasX, canvasY, tool);
                if (handled) {
                    return;
                }
            }
        }

        if (tool === 'pan') {
            // パンツール
            mapState.isPanning = true;
            mapState.lastX = e.clientX;
            mapState.lastY = e.clientY;
            container.style.cursor = 'grabbing';
        } else if (tool === 'pencil' || tool === 'eraser') {
            // 鉛筆または消しゴム - 画像ピクセル座標に変換
            if (window.canvasToImagePixel && typeof window.canvasToImagePixel === 'function') {
                let imagePixel = window.canvasToImagePixel(canvasX, canvasY);

                // スナップが有効な場合、座標をスナップ
                if (window.snapToGrid && typeof window.snapToGrid === 'function') {
                    imagePixel = window.snapToGrid(imagePixel.x, imagePixel.y);
                }

                mapState.drawingState.isDrawing = true;
                mapState.drawingState.currentStroke = [imagePixel];

                // 履歴に保存（描画開始時）
                if (window.saveToHistory && typeof window.saveToHistory === 'function') {
                    window.saveToHistory();
                }
            }
        } else if (tool === 'measure') {
            // 測量ツール
            mapState.drawingState.measurePoints.push({x: canvasX, y: canvasY});

            // 2点目が置かれたら距離を表示して測量を終了
            if (mapState.drawingState.measurePoints.length === 2) {
                if (window.showMeasureDistance && typeof window.showMeasureDistance === 'function') {
                    window.showMeasureDistance();
                }
                mapState.drawingState.measurePoints = [];
            }
        } else if (tool === 'bucket') {
            // バケツツール
            if (window.performBucketFill && typeof window.performBucketFill === 'function') {
                window.performBucketFill(canvasX, canvasY);
            }
        }
    } else if (e.button === 1) {
        // 中ボタンでパン
        e.preventDefault();
        mapState.isPanning = true;
        mapState.lastX = e.clientX;
        mapState.lastY = e.clientY;
        container.style.cursor = 'grabbing';
    }
}

/**
 * マウス移動ハンドラー
 */
function handleMouseMove(e) {
    const container = document.getElementById('mapContainer');
    const tool = mapState.drawingState.currentTool;

    // マウス位置を取得してワールド座標に変換し、ステータスバーを更新
    const rect = container.getBoundingClientRect();
    const canvasX = e.clientX - rect.left;
    const canvasY = e.clientY - rect.top;

    const worldPos = canvasToWorld(canvasX, canvasY, container);
    if (worldPos) {
        updateCursorPosition(worldPos.x, worldPos.y);
    } else {
        updateCursorPosition(null, null);
    }

    // 四角形ツールが有効な場合、四角形イベントハンドラーを呼び出す
    if (mapState.rectangleToolState && mapState.rectangleToolState.enabled) {
        if (window.handleRectangleMouseMove && typeof window.handleRectangleMouseMove === 'function') {
            const handled = window.handleRectangleMouseMove(canvasX, canvasY);
            if (handled) {
                return;
            }
        }
    }

    // パン中
    if (mapState.isPanning) {
        e.preventDefault();

        const dx = e.clientX - mapState.lastX;
        const dy = e.clientY - mapState.lastY;

        mapState.offsetX += dx;
        mapState.offsetY += dy;

        mapState.lastX = e.clientX;
        mapState.lastY = e.clientY;

        // レイヤーシステムが有効な場合
        if (mapState.layerStack.length > 0) {
            if (window.redrawAllLayers && typeof window.redrawAllLayers === 'function') {
                window.redrawAllLayers();
            }
        } else {
            if (window.drawMap && typeof window.drawMap === 'function') {
                window.drawMap();
            }
        }
    }
    // 描画中
    else if (mapState.drawingState.isDrawing && (tool === 'pencil' || tool === 'eraser')) {
        e.preventDefault();

        // 画像ピクセル座標に変換
        if (window.canvasToImagePixel && typeof window.canvasToImagePixel === 'function') {
            let imagePixel = window.canvasToImagePixel(canvasX, canvasY);

            // スナップが有効な場合、座標をスナップ
            if (window.snapToGrid && typeof window.snapToGrid === 'function') {
                imagePixel = window.snapToGrid(imagePixel.x, imagePixel.y);
            }

            mapState.drawingState.currentStroke.push(imagePixel);

            // 描画（色とブラシサイズを渡す）
            if (window.performDrawing && typeof window.performDrawing === 'function') {
                window.performDrawing(tool, mapState.drawingState.currentStroke,
                    mapState.drawingState.color,
                    mapState.drawingState.brushSize);
            }
        }
    }
}

/**
 * マウスアップハンドラー
 */
function handleMouseUp(e) {
    const container = document.getElementById('mapContainer');

    // 四角形ツールが有効な場合、四角形イベントハンドラーを呼び出す
    if (mapState.rectangleToolState && mapState.rectangleToolState.enabled) {
        if (window.handleRectangleMouseUp && typeof window.handleRectangleMouseUp === 'function') {
            const handled = window.handleRectangleMouseUp();
            if (handled) {
                return;
            }
        }
    }


    if (e.button === 0 || e.button === 1) {
        if (mapState.isPanning) {
            mapState.isPanning = false;
            if (window.updateCursor && typeof window.updateCursor === 'function') {
                window.updateCursor();
            }
        }

        if (mapState.drawingState.isDrawing) {
            const tool = mapState.drawingState.currentTool;

            // ストロークをレイヤーに保存
            if (mapState.drawingState.currentStroke.length > 0) {
                if (window.getActiveDrawingLayer && typeof window.getActiveDrawingLayer === 'function') {
                    const layer = window.getActiveDrawingLayer();
                    if (layer && layer.strokes) {
                        layer.strokes.push({
                            tool: tool,
                            stroke: [...mapState.drawingState.currentStroke],
                            color: mapState.drawingState.color,
                            brushSize: mapState.drawingState.brushSize
                        });
                    }
                }
            }

            mapState.drawingState.isDrawing = false;
            mapState.drawingState.currentStroke = [];
        }
    }
}

/**
 * マウスリーブハンドラー
 */
function handleMouseLeave() {
    const container = document.getElementById('mapContainer');

    if (mapState.isPanning) {
        mapState.isPanning = false;
        if (window.updateCursor && typeof window.updateCursor === 'function') {
            window.updateCursor();
        }
    }

    if (mapState.drawingState.isDrawing) {
        const tool = mapState.drawingState.currentTool;

        // ストロークをレイヤーに保存
        if (mapState.drawingState.currentStroke.length > 0) {
            if (window.getActiveDrawingLayer && typeof window.getActiveDrawingLayer === 'function') {
                const layer = window.getActiveDrawingLayer();
                if (layer && layer.strokes) {
                    layer.strokes.push({
                        tool: tool,
                        stroke: [...mapState.drawingState.currentStroke],
                        color: mapState.drawingState.color,
                        brushSize: mapState.drawingState.brushSize
                    });
                }
            }
        }

        mapState.drawingState.isDrawing = false;
        mapState.drawingState.currentStroke = [];
    }
}

/**
 * UIコントロール要素かどうかをチェック
 * @param {HTMLElement} element - チェックする要素
 * @returns {boolean} UIコントロール要素かどうか
 */
function isUIElement(element) {
    if (!element) return false;

    // ボタン、入力、スライダーなどのUI要素
    if (element.tagName === 'BUTTON' ||
        element.tagName === 'INPUT' ||
        element.tagName === 'SELECT' ||
        element.tagName === 'TEXTAREA' ||
        element.tagName === 'A') {
        return true;
    }

    // パネルやツールバーなどのクラスをチェック
    if (element.classList) {
        if (element.classList.contains('panel') ||
            element.classList.contains('tool-button') ||
            element.classList.contains('drawing-tools-palette') ||
            element.classList.contains('drawing-options') ||
            element.classList.contains('layer-item') ||
            element.classList.contains('layer-controls') ||
            element.id === 'layersPanel' ||
            element.id === 'drawingToolsPalette') {
            return true;
        }
    }

    // 親要素をチェック
    if (element.parentElement && element.parentElement !== document.body) {
        return isUIElement(element.parentElement);
    }

    return false;
}
