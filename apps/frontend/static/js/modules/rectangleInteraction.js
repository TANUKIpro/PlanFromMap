/**
 * @file rectangleInteraction.js
 * @description 四角形とのインタラクション処理（当たり判定、編集操作）
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../config.js - 設定値
 * @requires ./rectangleManager.js - 四角形管理
 *
 * @exports canvasToImagePixel - キャンバス座標を画像ピクセル座標に変換
 * @exports hitTestRectangle - 四角形との当たり判定
 * @exports hitTestHandle - ハンドルとの当たり判定
 * @exports handleRectangleMouseDown - マウスダウンイベント処理
 * @exports handleRectangleMouseMove - マウスムーブイベント処理
 * @exports handleRectangleMouseUp - マウスアップイベント処理
 * @exports calculateRotationAngle - 回転角度を計算
 * @exports snapRotationAngle - 回転角度をスナップ
 */

import { mapState } from '../state/mapState.js';
import { RECTANGLE_DEFAULTS } from '../config.js';
import {
    getRectangleById,
    getAllRectangles,
    selectRectangle,
    deselectRectangle,
    updateRectangle,
    deleteRectangle,
    getRectangleLayer
} from './rectangleManager.js';

/**
 * キャンバス座標を画像ピクセル座標に変換
 *
 * @param {number} canvasX - キャンバスX座標
 * @param {number} canvasY - キャンバスY座標
 * @returns {{x: number, y: number}} 画像ピクセル座標
 */
export function canvasToImagePixel(canvasX, canvasY) {
    if (!mapState.image) return {x: canvasX, y: canvasY};

    const container = document.getElementById('mapContainer');
    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    const baseX = (container.getBoundingClientRect().width - scaledWidth) / 2;
    const baseY = (container.getBoundingClientRect().height - scaledHeight) / 2;

    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    const imagePixelX = (canvasX - drawX) / mapState.scale;
    const imagePixelY = (canvasY - drawY) / mapState.scale;

    return {x: imagePixelX, y: imagePixelY};
}

/**
 * 回転を考慮した点が四角形内にあるかを判定
 *
 * @param {number} px - テスト点のX座標（画像ピクセル）
 * @param {number} py - テスト点のY座標（画像ピクセル）
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {boolean} 四角形内にある場合はtrue
 */
function pointInRotatedRectangle(px, py, rectangle) {
    // 四角形の中心からの相対座標を計算
    const dx = px - rectangle.x;
    const dy = py - rectangle.y;

    // 逆回転を適用
    const rad = (-rectangle.rotation * Math.PI) / 180;
    const rotatedX = dx * Math.cos(rad) - dy * Math.sin(rad);
    const rotatedY = dx * Math.sin(rad) + dy * Math.cos(rad);

    // 回転後の座標が四角形内にあるかを判定
    const halfWidth = rectangle.width / 2;
    const halfHeight = rectangle.height / 2;

    return Math.abs(rotatedX) <= halfWidth && Math.abs(rotatedY) <= halfHeight;
}

/**
 * 四角形との当たり判定
 *
 * @param {number} canvasX - キャンバスX座標
 * @param {number} canvasY - キャンバスY座標
 * @returns {Object|null} ヒットした四角形、なければnull
 *
 * @example
 * const hitRect = hitTestRectangle(100, 200);
 */
export function hitTestRectangle(canvasX, canvasY) {
    const imagePixel = canvasToImagePixel(canvasX, canvasY);
    const rectangles = getAllRectangles();

    // 逆順でチェック（上のレイヤーから）
    for (let i = rectangles.length - 1; i >= 0; i--) {
        const rectangle = rectangles[i];
        if (pointInRotatedRectangle(imagePixel.x, imagePixel.y, rectangle)) {
            return rectangle;
        }
    }

    return null;
}

/**
 * ハンドルとの当たり判定
 *
 * @param {number} canvasX - キャンバスX座標
 * @param {number} canvasY - キャンバスY座標
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {string|null} ヒットしたハンドル種類、なければnull
 *   'top', 'right', 'bottom', 'left', 'rotate'
 *
 * @example
 * const handle = hitTestHandle(100, 200, rectangle);
 */
export function hitTestHandle(canvasX, canvasY, rectangle) {
    if (!rectangle || !rectangle.selected) return null;

    const imagePixel = canvasToImagePixel(canvasX, canvasY);

    // 四角形の中心からの相対座標
    const dx = imagePixel.x - rectangle.x;
    const dy = imagePixel.y - rectangle.y;

    // 回転を適用
    const rad = (rectangle.rotation * Math.PI) / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    // 回転ハンドルのテスト（四角形の上）
    const rotateHandleOffset = 30 / mapState.scale; // ピクセル → 画像ピクセル
    const rotateHandleX = rectangle.x + 0 * cos - (-rectangle.height / 2 - rotateHandleOffset) * sin;
    const rotateHandleY = rectangle.y + 0 * sin + (-rectangle.height / 2 - rotateHandleOffset) * cos;
    const rotateHandleSize = (RECTANGLE_DEFAULTS.HANDLE_SIZE * 1.5) / mapState.scale;

    const rotateHandleDist = Math.sqrt(
        Math.pow(imagePixel.x - rotateHandleX, 2) +
        Math.pow(imagePixel.y - rotateHandleY, 2)
    );

    if (rotateHandleDist <= rotateHandleSize) {
        return 'rotate';
    }

    // 逆回転を適用して、ローカル座標系でテスト
    const localX = dx * cos + dy * sin;
    const localY = -dx * sin + dy * cos;

    const halfWidth = rectangle.width / 2;
    const halfHeight = rectangle.height / 2;
    const handleSize = RECTANGLE_DEFAULTS.HANDLE_SIZE / mapState.scale;

    // 上
    if (Math.abs(localX) <= handleSize && Math.abs(localY - (-halfHeight)) <= handleSize) {
        return 'top';
    }
    // 右
    if (Math.abs(localX - halfWidth) <= handleSize && Math.abs(localY) <= handleSize) {
        return 'right';
    }
    // 下
    if (Math.abs(localX) <= handleSize && Math.abs(localY - halfHeight) <= handleSize) {
        return 'bottom';
    }
    // 左
    if (Math.abs(localX - (-halfWidth)) <= handleSize && Math.abs(localY) <= handleSize) {
        return 'left';
    }

    return null;
}

/**
 * マウスダウンイベント処理（四角形ツール用）
 *
 * @param {number} canvasX - キャンバスX座標
 * @param {number} canvasY - キャンバスY座標
 * @param {string} currentTool - 現在のツール
 * @returns {boolean} イベントが処理された場合はtrue
 *
 * @example
 * const handled = handleRectangleMouseDown(100, 200, 'pencil');
 */
export function handleRectangleMouseDown(canvasX, canvasY, currentTool) {
    if (!mapState.rectangleToolState.enabled) return false;

    const rectangles = getAllRectangles();

    // 鉛筆モード：新規四角形作成
    if (currentTool === 'pencil') {
        // 選択中の四角形がある場合は、まず選択解除
        const selectedRect = rectangles.find(r => r.selected);
        if (selectedRect) {
            // ハンドルをクリックした場合は何もしない
            const handle = hitTestHandle(canvasX, canvasY, selectedRect);
            if (handle) {
                return false;
            }
        }

        // 四角形をクリックした場合は選択
        const hitRect = hitTestRectangle(canvasX, canvasY);
        if (hitRect) {
            selectRectangle(hitRect.id);
            return true;
        }

        // 何もない場所をクリックした場合は、新規四角形作成を開始
        deselectRectangle();
        mapState.rectangleToolState.editMode = 'create';
        mapState.rectangleToolState.isDragging = true;
        mapState.rectangleToolState.dragStartPos = canvasToImagePixel(canvasX, canvasY);
        mapState.rectangleToolState.createStartPos = canvasToImagePixel(canvasX, canvasY);

        // 仮の四角形を作成（サイズ0で開始）
        const startPos = mapState.rectangleToolState.createStartPos;
        if (window.createRectangle && typeof window.createRectangle === 'function') {
            const tempRect = window.createRectangle(startPos.x, startPos.y, 0, 0);
            mapState.rectangleToolState.creatingRectangleId = tempRect.id;
        }

        return true;
    }

    // パンモード：移動、回転、辺のリサイズ、選択
    if (currentTool === 'pan') {
        // 選択中の四角形があれば、ハンドルをテスト
        const selectedRect = rectangles.find(r => r.selected);
        if (selectedRect) {
            const handle = hitTestHandle(canvasX, canvasY, selectedRect);
            if (handle === 'rotate') {
                // 回転開始
                mapState.rectangleToolState.editMode = 'rotate';
                mapState.rectangleToolState.isDragging = true;
                mapState.rectangleToolState.dragStartPos = canvasToImagePixel(canvasX, canvasY);
                mapState.rectangleToolState.mouseDownPos = canvasToImagePixel(canvasX, canvasY);

                // カーソルを変更
                const container = document.getElementById('mapContainer');
                container.style.cursor = 'grabbing';
                return true;
            } else if (handle && handle !== 'rotate') {
                // 辺のリサイズ開始
                mapState.rectangleToolState.editMode = 'resize';
                mapState.rectangleToolState.resizeEdge = handle;
                mapState.rectangleToolState.isDragging = true;
                mapState.rectangleToolState.dragStartPos = canvasToImagePixel(canvasX, canvasY);
                mapState.rectangleToolState.mouseDownPos = canvasToImagePixel(canvasX, canvasY);

                // カーソルを変更
                const container = document.getElementById('mapContainer');
                if (handle === 'top' || handle === 'bottom') {
                    container.style.cursor = 'ns-resize';
                } else if (handle === 'left' || handle === 'right') {
                    container.style.cursor = 'ew-resize';
                }
                return true;
            }
        }

        // 四角形をクリック - マウスダウン時点では移動モードに入らず、ドラッグ開始を待つ
        const hitRect = hitTestRectangle(canvasX, canvasY);
        if (hitRect) {
            // マウスダウン位置を記録（クリックかドラッグかを判定するため）
            mapState.rectangleToolState.mouseDownPos = canvasToImagePixel(canvasX, canvasY);
            mapState.rectangleToolState.mouseDownRectangleId = hitRect.id;
            mapState.rectangleToolState.isDragging = false;
            mapState.rectangleToolState.editMode = null;

            // すでに選択されている四角形の場合は、選択を維持
            // 未選択の四角形の場合は、マウスアップ時に選択
            if (!hitRect.selected) {
                mapState.rectangleToolState.needsSelection = true;
            } else {
                mapState.rectangleToolState.needsSelection = false;
            }

            return true;
        }

        // 何もない場所をクリックした場合は選択解除
        // レイヤーを再描画
        const rectangleLayer = getRectangleLayer();
        if (rectangleLayer && window.redrawRectangleLayer) {
            deselectRectangle();
            window.redrawRectangleLayer(rectangleLayer);
        }

        return false;  // 通常のパン操作を許可
    }

    // 消しゴムモード：削除
    if (currentTool === 'eraser') {
        const hitRect = hitTestRectangle(canvasX, canvasY);
        if (hitRect) {
            deleteRectangle(hitRect.id);
            // レイヤーを再描画
            const rectangleLayer = getRectangleLayer();
            if (rectangleLayer && window.redrawRectangleLayer) {
                window.redrawRectangleLayer(rectangleLayer);
            }
            return true;
        }
        // 四角形に当たっていない場合は通常の消しゴム操作を許可
        return false;
    }

    // 測量モード：辺の長さ表示・編集
    if (currentTool === 'measure') {
        const selectedRect = rectangles.find(r => r.selected);
        if (selectedRect) {
            const handle = hitTestHandle(canvasX, canvasY, selectedRect);
            if (handle && handle !== 'rotate') {
                // 測量モード開始
                mapState.rectangleToolState.editMode = 'measure';
                mapState.rectangleToolState.measureEdge = handle;

                // 辺の長さを入力
                promptEdgeLength(selectedRect, handle);
                return true;
            }
        }

        // 四角形をクリックして選択
        const hitRect = hitTestRectangle(canvasX, canvasY);
        if (hitRect) {
            selectRectangle(hitRect.id);
            return true;
        }

        // 何もない場所をクリックした場合は選択解除して、通常の測量を許可
        deselectRectangle();
        return false;  // 通常の測量操作を許可
    }

    return false;
}

/**
 * マウスムーブイベント処理（四角形ツール用）
 *
 * @param {number} canvasX - キャンバスX座標
 * @param {number} canvasY - キャンバスY座標
 * @returns {boolean} イベントが処理された場合はtrue
 *
 * @example
 * const handled = handleRectangleMouseMove(100, 200);
 */
export function handleRectangleMouseMove(canvasX, canvasY) {
    if (!mapState.rectangleToolState.enabled) return false;

    const currentPos = canvasToImagePixel(canvasX, canvasY);

    // マウスダウン後、まだドラッグモードに入っていない場合
    // 一定距離以上移動したらドラッグモードに入る
    if (mapState.rectangleToolState.mouseDownPos &&
        !mapState.rectangleToolState.isDragging &&
        mapState.rectangleToolState.mouseDownRectangleId) {

        const dx = currentPos.x - mapState.rectangleToolState.mouseDownPos.x;
        const dy = currentPos.y - mapState.rectangleToolState.mouseDownPos.y;
        const distance = Math.sqrt(dx * dx + dy * dy);

        // 3ピクセル以上移動したらドラッグモードに入る
        const dragThreshold = 3 / mapState.scale; // 画像ピクセル単位

        if (distance > dragThreshold) {
            // ドラッグ開始：移動モードに入る
            const rectId = mapState.rectangleToolState.mouseDownRectangleId;
            selectRectangle(rectId);

            mapState.rectangleToolState.editMode = 'move';
            mapState.rectangleToolState.isDragging = true;
            mapState.rectangleToolState.dragStartPos = mapState.rectangleToolState.mouseDownPos;
            mapState.rectangleToolState.needsSelection = false;

            // カーソルを変更
            const container = document.getElementById('mapContainer');
            container.style.cursor = 'move';

            // レイヤーを再描画
            const rectangleLayer = getRectangleLayer();
            if (rectangleLayer && window.redrawRectangleLayer) {
                window.redrawRectangleLayer(rectangleLayer);
            }
        }

        return true;
    }

    // ドラッグ中の処理
    if (mapState.rectangleToolState.isDragging) {
        // 新規作成モード
        if (mapState.rectangleToolState.editMode === 'create') {
            const startPos = mapState.rectangleToolState.createStartPos;
            const creatingRect = getRectangleById(mapState.rectangleToolState.creatingRectangleId);

            if (creatingRect) {
                // 対角線の2点から四角形のパラメータを計算
                const minX = Math.min(startPos.x, currentPos.x);
                const minY = Math.min(startPos.y, currentPos.y);
                const maxX = Math.max(startPos.x, currentPos.x);
                const maxY = Math.max(startPos.y, currentPos.y);

                const width = maxX - minX;
                const height = maxY - minY;
                const centerX = (minX + maxX) / 2;
                const centerY = (minY + maxY) / 2;

                // 四角形を更新
                updateRectangle(creatingRect.id, {
                    x: centerX,
                    y: centerY,
                    width: width,
                    height: height
                });

                // レイヤーを再描画
                const rectangleLayer = getRectangleLayer();
                if (rectangleLayer && window.redrawRectangleLayer) {
                    window.redrawRectangleLayer(rectangleLayer);
                }
            }

            return true;
        }

        const selectedRect = getRectangleById(mapState.rectangleToolState.selectedRectangleId);
        if (!selectedRect) return false;

        const startPos = mapState.rectangleToolState.dragStartPos;
        const dx = currentPos.x - startPos.x;
        const dy = currentPos.y - startPos.y;

        // リサイズ
        if (mapState.rectangleToolState.editMode === 'resize') {
            handleResize(selectedRect, dx, dy);
        }
        // 移動
        else if (mapState.rectangleToolState.editMode === 'move') {
            handleMove(selectedRect, dx, dy);
        }
        // 回転
        else if (mapState.rectangleToolState.editMode === 'rotate') {
            handleRotate(selectedRect, currentPos);
        }

        // レイヤーを再描画
        const rectangleLayer = getRectangleLayer();
        if (rectangleLayer && window.redrawRectangleLayer) {
            window.redrawRectangleLayer(rectangleLayer);
        }

        return true;
    }

    // ホバー処理（ハンドルのハイライトとカーソル変更）
    const rectangles = getAllRectangles();
    const selectedRect = rectangles.find(r => r.selected);
    const container = document.getElementById('mapContainer');

    if (selectedRect) {
        const handle = hitTestHandle(canvasX, canvasY, selectedRect);

        // ホバー状態が変わった場合は再描画
        if (handle !== mapState.rectangleToolState.hoverEdge) {
            mapState.rectangleToolState.hoverEdge = handle;

            // レイヤーを再描画
            const rectangleLayer = getRectangleLayer();
            if (rectangleLayer && window.redrawRectangleLayer) {
                window.redrawRectangleLayer(rectangleLayer);
            }
        }

        // カーソルを変更
        if (handle) {
            if (handle === 'top' || handle === 'bottom') {
                container.style.cursor = 'ns-resize';  // ↕
            } else if (handle === 'left' || handle === 'right') {
                container.style.cursor = 'ew-resize';  // ↔
            } else if (handle === 'rotate') {
                container.style.cursor = 'grab';  // 回転
            }
            return false;  // ハンドルにホバー中でも通常のツール操作は許可
        } else {
            // ハンドルにホバーしていない場合は、デフォルトのカーソルに戻す
            if (window.updateCursor && typeof window.updateCursor === 'function') {
                window.updateCursor();
            }
        }
    } else {
        // 四角形が選択されていない場合は、デフォルトのカーソルに戻す
        if (mapState.rectangleToolState.hoverEdge !== null) {
            mapState.rectangleToolState.hoverEdge = null;
            if (window.updateCursor && typeof window.updateCursor === 'function') {
                window.updateCursor();
            }
        }
    }

    return false;
}

/**
 * マウスアップイベント処理（四角形ツール用）
 *
 * @returns {boolean} イベントが処理された場合はtrue
 *
 * @example
 * const handled = handleRectangleMouseUp();
 */
export function handleRectangleMouseUp() {
    if (!mapState.rectangleToolState.enabled) return false;

    // クリック（ドラッグしなかった）場合の選択処理
    if (mapState.rectangleToolState.mouseDownRectangleId &&
        !mapState.rectangleToolState.isDragging &&
        mapState.rectangleToolState.needsSelection) {

        // 四角形を選択
        selectRectangle(mapState.rectangleToolState.mouseDownRectangleId);

        // レイヤーを再描画
        const rectangleLayer = getRectangleLayer();
        if (rectangleLayer && window.redrawRectangleLayer) {
            window.redrawRectangleLayer(rectangleLayer);
        }

        // 状態をクリア
        mapState.rectangleToolState.mouseDownPos = null;
        mapState.rectangleToolState.mouseDownRectangleId = null;
        mapState.rectangleToolState.needsSelection = false;

        return true;
    }

    // クリック後のクリーンアップ（選択済みの四角形をクリックした場合）
    if (mapState.rectangleToolState.mouseDownRectangleId &&
        !mapState.rectangleToolState.isDragging) {

        // 状態をクリア
        mapState.rectangleToolState.mouseDownPos = null;
        mapState.rectangleToolState.mouseDownRectangleId = null;
        mapState.rectangleToolState.needsSelection = false;

        return true;
    }

    if (mapState.rectangleToolState.isDragging) {
        // 新規作成モード完了
        if (mapState.rectangleToolState.editMode === 'create') {
            const creatingRect = getRectangleById(mapState.rectangleToolState.creatingRectangleId);

            if (creatingRect) {
                // 最小サイズをチェック（cm単位から画像ピクセル単位に変換）
                const resolution = mapState.metadata ? mapState.metadata.resolution : 0.05; // m/pixel
                const minSizeInMeters = RECTANGLE_DEFAULTS.MIN_SIZE_CM / 100; // cm → m
                const minSizeInPixels = minSizeInMeters / resolution;

                // サイズが小さすぎる場合は削除
                if (creatingRect.width < minSizeInPixels || creatingRect.height < minSizeInPixels) {
                    deleteRectangle(creatingRect.id);
                } else {
                    // 新しく作成した四角形を選択
                    selectRectangle(creatingRect.id);
                }

                // レイヤーを再描画
                const rectangleLayer = getRectangleLayer();
                if (rectangleLayer && window.redrawRectangleLayer) {
                    window.redrawRectangleLayer(rectangleLayer);
                }
            }

            mapState.rectangleToolState.creatingRectangleId = null;
            mapState.rectangleToolState.createStartPos = null;
        }

        mapState.rectangleToolState.isDragging = false;
        mapState.rectangleToolState.editMode = null;
        mapState.rectangleToolState.resizeEdge = null;
        mapState.rectangleToolState.dragStartPos = null;
        mapState.rectangleToolState.mouseDownPos = null;
        mapState.rectangleToolState.mouseDownRectangleId = null;
        mapState.rectangleToolState.needsSelection = false;

        // カーソルを元に戻す
        if (window.updateCursor && typeof window.updateCursor === 'function') {
            window.updateCursor();
        }

        return true;
    }

    return false;
}

/**
 * リサイズ処理
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {number} dx - X方向の移動量
 * @param {number} dy - Y方向の移動量
 * @returns {void}
 */
function handleResize(rectangle, dx, dy) {
    const edge = mapState.rectangleToolState.resizeEdge;
    const rad = (rectangle.rotation * Math.PI) / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    // ローカル座標系での移動量
    const localDx = dx * cos + dy * sin;
    const localDy = -dx * sin + dy * cos;

    // 最小サイズをcm単位から画像ピクセル単位に変換
    const resolution = mapState.metadata ? mapState.metadata.resolution : 0.05; // m/pixel
    const minSizeInMeters = RECTANGLE_DEFAULTS.MIN_SIZE_CM / 100; // cm → m
    const minSizeInPixels = minSizeInMeters / resolution;

    let newWidth = rectangle.width;
    let newHeight = rectangle.height;
    let centerDx = 0;
    let centerDy = 0;

    if (edge === 'top') {
        newHeight = Math.max(minSizeInPixels, rectangle.height - localDy);
        centerDy = -(newHeight - rectangle.height) / 2;
    } else if (edge === 'right') {
        newWidth = Math.max(minSizeInPixels, rectangle.width + localDx);
        centerDx = (newWidth - rectangle.width) / 2;
    } else if (edge === 'bottom') {
        newHeight = Math.max(minSizeInPixels, rectangle.height + localDy);
        centerDy = (newHeight - rectangle.height) / 2;
    } else if (edge === 'left') {
        newWidth = Math.max(minSizeInPixels, rectangle.width - localDx);
        centerDx = -(newWidth - rectangle.width) / 2;
    }

    // グローバル座標系での中心移動
    const globalCenterDx = centerDx * cos - centerDy * sin;
    const globalCenterDy = centerDx * sin + centerDy * cos;

    updateRectangle(rectangle.id, {
        width: newWidth,
        height: newHeight,
        x: rectangle.x + globalCenterDx,
        y: rectangle.y + globalCenterDy
    });

    // ドラッグ開始位置を更新
    mapState.rectangleToolState.dragStartPos = {
        x: mapState.rectangleToolState.dragStartPos.x + dx,
        y: mapState.rectangleToolState.dragStartPos.y + dy
    };
}

/**
 * 移動処理
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {number} dx - X方向の移動量
 * @param {number} dy - Y方向の移動量
 * @returns {void}
 */
function handleMove(rectangle, dx, dy) {
    updateRectangle(rectangle.id, {
        x: rectangle.x + dx,
        y: rectangle.y + dy
    });

    mapState.rectangleToolState.dragStartPos = {
        x: mapState.rectangleToolState.dragStartPos.x + dx,
        y: mapState.rectangleToolState.dragStartPos.y + dy
    };
}

/**
 * 回転処理
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {{x: number, y: number}} currentPos - 現在のマウス位置（画像ピクセル）
 * @returns {void}
 */
function handleRotate(rectangle, currentPos) {
    const angle = calculateRotationAngle(rectangle, currentPos);
    const snappedAngle = snapRotationAngle(angle);

    updateRectangle(rectangle.id, {
        rotation: snappedAngle
    });
}

/**
 * 回転角度を計算
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {{x: number, y: number}} mousePos - マウス位置（画像ピクセル）
 * @returns {number} 回転角度（度）
 *
 * @example
 * const angle = calculateRotationAngle(rectangle, {x: 100, y: 200});
 */
export function calculateRotationAngle(rectangle, mousePos) {
    const dx = mousePos.x - rectangle.x;
    const dy = mousePos.y - rectangle.y;

    // 角度を計算（ラジアン → 度）
    let angle = Math.atan2(dy, dx) * (180 / Math.PI);

    // 0-360度に正規化
    angle = (angle + 360) % 360;

    // 上方向を0度とするため、90度を引く
    angle = (angle - 90 + 360) % 360;

    return angle;
}

/**
 * 回転角度をスナップ
 *
 * @param {number} angle - 回転角度（度）
 * @returns {number} スナップされた回転角度（度）
 *
 * @example
 * const snappedAngle = snapRotationAngle(43.7);  // → 44
 */
export function snapRotationAngle(angle) {
    const snapAngle = RECTANGLE_DEFAULTS.ROTATION_SNAP_ANGLE;
    return Math.round(angle / snapAngle) * snapAngle;
}

/**
 * 画像ピクセル座標をキャンバス座標に変換
 *
 * @param {number} imagePixelX - 画像ピクセルX座標
 * @param {number} imagePixelY - 画像ピクセルY座標
 * @returns {{x: number, y: number}} キャンバス座標
 */
function imagePixelToCanvas(imagePixelX, imagePixelY) {
    if (!mapState.image) return {x: imagePixelX, y: imagePixelY};

    const container = document.getElementById('mapContainer');
    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    const baseX = (container.getBoundingClientRect().width - scaledWidth) / 2;
    const baseY = (container.getBoundingClientRect().height - scaledHeight) / 2;

    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    const canvasX = imagePixelX * mapState.scale + drawX;
    const canvasY = imagePixelY * mapState.scale + drawY;

    return {x: canvasX, y: canvasY};
}

/**
 * 辺の中心位置を計算（キャンバス座標）
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {string} edge - 辺の種類（'top', 'right', 'bottom', 'left'）
 * @returns {{x: number, y: number}} 辺の中心位置（キャンバス座標）
 */
function getEdgeMidpoint(rectangle, edge) {
    const rad = (rectangle.rotation * Math.PI) / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    let localX = 0;
    let localY = 0;

    if (edge === 'top') {
        localX = 0;
        localY = -rectangle.height / 2;
    } else if (edge === 'right') {
        localX = rectangle.width / 2;
        localY = 0;
    } else if (edge === 'bottom') {
        localX = 0;
        localY = rectangle.height / 2;
    } else if (edge === 'left') {
        localX = -rectangle.width / 2;
        localY = 0;
    }

    // ローカル座標をグローバル座標に変換
    const globalX = rectangle.x + localX * cos - localY * sin;
    const globalY = rectangle.y + localX * sin + localY * cos;

    // 画像ピクセル座標をキャンバス座標に変換
    return imagePixelToCanvas(globalX, globalY);
}

/**
 * 辺の長さを入力するUIを表示（Fusion360風）
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {string} edge - 辺の種類（'top', 'right', 'bottom', 'left'）
 * @returns {void}
 */
function promptEdgeLength(rectangle, edge) {
    // メタデータから解像度を取得（メートル/ピクセル）
    const resolution = mapState.metadata ? mapState.metadata.resolution : 0.05;

    // 現在の辺の長さを取得（cm単位）
    let currentLengthInPixels;
    if (edge === 'top' || edge === 'bottom') {
        currentLengthInPixels = rectangle.width;
    } else {
        currentLengthInPixels = rectangle.height;
    }

    const currentLengthInCm = currentLengthInPixels * resolution * 100;

    // 辺の中心位置を計算
    const edgeMidpoint = getEdgeMidpoint(rectangle, edge);

    // UIを表示
    showEdgeLengthInputUI(edgeMidpoint.x, edgeMidpoint.y, currentLengthInCm, (newLengthInCm) => {
        // 確定時のコールバック
        if (newLengthInCm === null) {
            // キャンセル
            mapState.rectangleToolState.editMode = null;
            mapState.rectangleToolState.measureEdge = null;
            return;
        }

        // 最小サイズチェック
        if (newLengthInCm < RECTANGLE_DEFAULTS.MIN_SIZE_CM) {
            alert(`辺の長さは${RECTANGLE_DEFAULTS.MIN_SIZE_CM}cm以上にしてください`);
            mapState.rectangleToolState.editMode = null;
            mapState.rectangleToolState.measureEdge = null;
            return;
        }

        // cm → ピクセルに変換
        const newLengthInPixels = (newLengthInCm / 100) / resolution;

        // 四角形を更新
        if (edge === 'top' || edge === 'bottom') {
            updateRectangle(rectangle.id, {
                width: newLengthInPixels
            });
        } else {
            updateRectangle(rectangle.id, {
                height: newLengthInPixels
            });
        }

        // レイヤーを再描画
        const rectangleLayer = getRectangleLayer();
        if (rectangleLayer && window.redrawRectangleLayer) {
            window.redrawRectangleLayer(rectangleLayer);
        }

        mapState.rectangleToolState.editMode = null;
        mapState.rectangleToolState.measureEdge = null;
    });
}

/**
 * 辺の長さ入力UIを表示
 *
 * @param {number} x - 表示位置X（キャンバス座標）
 * @param {number} y - 表示位置Y（キャンバス座標）
 * @param {number} currentValue - 現在の値（cm）
 * @param {Function} callback - 確定時のコールバック関数
 * @returns {void}
 */
function showEdgeLengthInputUI(x, y, currentValue, callback) {
    const ui = document.getElementById('edgeLengthInputUI');
    const input = document.getElementById('edgeLengthInput');
    const confirmBtn = document.getElementById('edgeLengthConfirm');
    const cancelBtn = document.getElementById('edgeLengthCancel');

    if (!ui || !input || !confirmBtn || !cancelBtn) {
        console.error('Edge length input UI elements not found');
        return;
    }

    // UIの位置を設定（辺の中心より少し下にオフセット）
    ui.style.left = `${x}px`;
    ui.style.top = `${y + 30}px`;
    ui.style.display = 'block';

    // 入力フィールドに現在の値を設定
    input.value = currentValue.toFixed(1);
    input.focus();
    input.select();

    // 既存のイベントリスナーを削除
    const newConfirmBtn = confirmBtn.cloneNode(true);
    const newCancelBtn = cancelBtn.cloneNode(true);
    confirmBtn.parentNode.replaceChild(newConfirmBtn, confirmBtn);
    cancelBtn.parentNode.replaceChild(newCancelBtn, cancelBtn);

    // 確定処理
    const confirm = () => {
        const newValue = parseFloat(input.value);
        ui.style.display = 'none';

        if (isNaN(newValue) || newValue <= 0) {
            alert('無効な長さです');
            callback(null);
            return;
        }

        callback(newValue);
    };

    // キャンセル処理
    const cancel = () => {
        ui.style.display = 'none';
        callback(null);
    };

    // イベントリスナーを追加
    newConfirmBtn.addEventListener('click', confirm);
    newCancelBtn.addEventListener('click', cancel);

    // Enterキーで確定、Escapeキーでキャンセル
    const keyHandler = (e) => {
        if (e.key === 'Enter') {
            confirm();
            input.removeEventListener('keydown', keyHandler);
        } else if (e.key === 'Escape') {
            cancel();
            input.removeEventListener('keydown', keyHandler);
        }
    };
    input.addEventListener('keydown', keyHandler);
}
