/**
 * @file rectangleRenderer.js
 * @description 四角形の描画（本体、ハンドル、角度、長さ表示）
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../config.js - 設定値
 * @requires ./rectangleManager.js - 四角形管理
 *
 * @exports redrawRectangleLayer - 四角形レイヤーの再描画
 * @exports drawRectangle - 単一の四角形を描画
 * @exports drawResizeHandles - リサイズハンドルを描画
 * @exports drawRotationHandle - 回転ハンドルを描画
 * @exports drawRotationAngle - 回転角度を表示
 * @exports drawEdgeLength - 辺の長さを表示
 */

import { mapState } from '../state/mapState.js';
import { RECTANGLE_DEFAULTS } from '../config.js';
import { getAllRectangles } from './rectangleManager.js';

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
 * 四角形レイヤーを再描画する
 *
 * @param {Object} layer - 四角形レイヤーオブジェクト
 * @returns {void}
 *
 * @example
 * redrawRectangleLayer(rectangleLayer);
 */
export function redrawRectangleLayer(layer) {
    if (!layer || layer.type !== 'rectangle') return;

    const ctx = layer.ctx;

    // キャンバスをクリア
    ctx.clearRect(0, 0, layer.canvas.width, layer.canvas.height);

    // すべての四角形を描画
    const rectangles = getAllRectangles();
    rectangles.forEach(rectangle => {
        drawRectangle(ctx, rectangle);
    });
}

/**
 * 単一の四角形を描画する
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {void}
 *
 * @example
 * drawRectangle(ctx, rectangle);
 */
export function drawRectangle(ctx, rectangle) {
    // 画像ピクセル座標をキャンバス座標に変換
    const centerPos = imagePixelToCanvas(rectangle.x, rectangle.y);
    const scaledWidth = rectangle.width * mapState.scale;
    const scaledHeight = rectangle.height * mapState.scale;

    ctx.save();

    // 中心に移動して回転
    ctx.translate(centerPos.x, centerPos.y);
    ctx.rotate((rectangle.rotation * Math.PI) / 180);

    // 四角形の色を設定
    const color = rectangle.selected ? RECTANGLE_DEFAULTS.SELECTED_COLOR : RECTANGLE_DEFAULTS.STROKE_COLOR;
    ctx.strokeStyle = color;
    ctx.lineWidth = RECTANGLE_DEFAULTS.STROKE_WIDTH;

    // 四角形を描画
    ctx.strokeRect(
        -scaledWidth / 2,
        -scaledHeight / 2,
        scaledWidth,
        scaledHeight
    );

    // 選択されている場合はハンドルを描画
    if (rectangle.selected) {
        drawResizeHandles(ctx, scaledWidth, scaledHeight);
        drawRotationHandle(ctx, scaledWidth, scaledHeight);

        // 回転中の場合は角度を表示
        if (mapState.rectangleToolState.editMode === 'rotate') {
            ctx.restore();
            drawRotationAngle(ctx, centerPos, rectangle.rotation);
            return;
        }

        // 測量モードで辺が選択されている場合は長さを表示
        if (mapState.rectangleToolState.editMode === 'measure' &&
            mapState.rectangleToolState.measureEdge) {
            ctx.restore();
            drawEdgeLength(ctx, rectangle, mapState.rectangleToolState.measureEdge);
            return;
        }
    }

    ctx.restore();
}

/**
 * リサイズハンドルを描画する
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} width - 四角形の幅（スケール済み）
 * @param {number} height - 四角形の高さ（スケール済み）
 * @returns {void}
 *
 * @example
 * drawResizeHandles(ctx, 200, 150);
 */
export function drawResizeHandles(ctx, width, height) {
    const handleSize = RECTANGLE_DEFAULTS.HANDLE_SIZE;

    // ハンドルの位置（上下左右の辺の中央）
    const handles = [
        { x: 0, y: -height / 2, edge: 'top' },       // 上
        { x: width / 2, y: 0, edge: 'right' },       // 右
        { x: 0, y: height / 2, edge: 'bottom' },     // 下
        { x: -width / 2, y: 0, edge: 'left' }        // 左
    ];

    handles.forEach(handle => {
        // ハンドルの色
        ctx.fillStyle = RECTANGLE_DEFAULTS.HANDLE_COLOR;
        ctx.strokeStyle = RECTANGLE_DEFAULTS.HANDLE_STROKE_COLOR;
        ctx.lineWidth = 2;

        // ホバー中の辺をハイライト
        if (mapState.rectangleToolState.hoverEdge === handle.edge) {
            ctx.fillStyle = RECTANGLE_DEFAULTS.SELECTED_COLOR;
        }

        // ハンドルを描画（正方形）
        ctx.fillRect(
            handle.x - handleSize / 2,
            handle.y - handleSize / 2,
            handleSize,
            handleSize
        );
        ctx.strokeRect(
            handle.x - handleSize / 2,
            handle.y - handleSize / 2,
            handleSize,
            handleSize
        );
    });
}

/**
 * 回転ハンドルを描画する
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} width - 四角形の幅（スケール済み）
 * @param {number} height - 四角形の高さ（スケール済み）
 * @returns {void}
 *
 * @example
 * drawRotationHandle(ctx, 200, 150);
 */
export function drawRotationHandle(ctx, width, height) {
    const handleSize = RECTANGLE_DEFAULTS.HANDLE_SIZE * 1.5;
    const offset = 30; // 四角形の上からの距離

    // 回転ハンドルの位置（四角形の上）
    const handleX = 0;
    const handleY = -height / 2 - offset;

    // ハンドルまでの線を描画
    ctx.strokeStyle = RECTANGLE_DEFAULTS.HANDLE_STROKE_COLOR;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(0, -height / 2);
    ctx.lineTo(handleX, handleY);
    ctx.stroke();
    ctx.setLineDash([]);

    // 回転ハンドルを描画（円）
    ctx.fillStyle = RECTANGLE_DEFAULTS.HANDLE_COLOR;
    ctx.strokeStyle = RECTANGLE_DEFAULTS.HANDLE_STROKE_COLOR;
    ctx.lineWidth = 2;

    // ホバー中をハイライト
    if (mapState.rectangleToolState.hoverEdge === 'rotate') {
        ctx.fillStyle = RECTANGLE_DEFAULTS.SELECTED_COLOR;
    }

    ctx.beginPath();
    ctx.arc(handleX, handleY, handleSize / 2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();

    // 回転アイコン（矢印）を描画
    ctx.strokeStyle = RECTANGLE_DEFAULTS.HANDLE_STROKE_COLOR;
    ctx.lineWidth = 1.5;
    const arrowRadius = handleSize / 3;
    ctx.beginPath();
    ctx.arc(handleX, handleY, arrowRadius, -Math.PI * 0.7, Math.PI * 0.5, false);
    ctx.stroke();

    // 矢印の先端
    const arrowX = handleX + arrowRadius * Math.cos(Math.PI * 0.5);
    const arrowY = handleY + arrowRadius * Math.sin(Math.PI * 0.5);
    ctx.beginPath();
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(arrowX - 4, arrowY - 2);
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(arrowX - 2, arrowY + 4);
    ctx.stroke();
}

/**
 * 回転角度を表示する
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {{x: number, y: number}} centerPos - 四角形の中心位置（キャンバス座標）
 * @param {number} rotation - 回転角度（度）
 * @returns {void}
 *
 * @example
 * drawRotationAngle(ctx, {x: 100, y: 200}, 45);
 */
export function drawRotationAngle(ctx, centerPos, rotation) {
    const text = `${rotation.toFixed(0)}°`;
    const fontSize = 16;
    const padding = 8;

    ctx.font = `bold ${fontSize}px Arial`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    // テキストのサイズを測定
    const metrics = ctx.measureText(text);
    const textWidth = metrics.width;
    const textHeight = fontSize;

    // 背景を描画
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(
        centerPos.x - textWidth / 2 - padding,
        centerPos.y - textHeight / 2 - padding,
        textWidth + padding * 2,
        textHeight + padding * 2
    );

    // テキストを描画
    ctx.fillStyle = 'white';
    ctx.fillText(text, centerPos.x, centerPos.y);
}

/**
 * 辺の長さを表示する
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {string} edge - 辺の種類（'top', 'right', 'bottom', 'left'）
 * @returns {void}
 *
 * @example
 * drawEdgeLength(ctx, rectangle, 'top');
 */
export function drawEdgeLength(ctx, rectangle, edge) {
    // メタデータから解像度を取得（メートル/ピクセル）
    const resolution = mapState.metadata ? mapState.metadata.resolution : 0.05;

    // 辺の長さを計算（cm単位）
    let lengthInPixels;
    if (edge === 'top' || edge === 'bottom') {
        lengthInPixels = rectangle.width;
    } else {
        lengthInPixels = rectangle.height;
    }

    const lengthInMeters = lengthInPixels * resolution;
    const lengthInCm = lengthInMeters * 100;

    // テキスト
    const text = `${lengthInCm.toFixed(1)} cm`;
    const fontSize = 14;
    const padding = 6;

    // 辺の中央位置を計算（画像ピクセル座標）
    let edgeX = rectangle.x;
    let edgeY = rectangle.y;
    const halfWidth = rectangle.width / 2;
    const halfHeight = rectangle.height / 2;

    // 回転を考慮して辺の中央位置を計算
    const rad = (rectangle.rotation * Math.PI) / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    if (edge === 'top') {
        edgeX += -halfHeight * sin;
        edgeY += -halfHeight * cos;
    } else if (edge === 'right') {
        edgeX += halfWidth * cos;
        edgeY += halfWidth * sin;
    } else if (edge === 'bottom') {
        edgeX += halfHeight * sin;
        edgeY += halfHeight * cos;
    } else if (edge === 'left') {
        edgeX += -halfWidth * cos;
        edgeY += -halfWidth * sin;
    }

    // キャンバス座標に変換
    const canvasPos = imagePixelToCanvas(edgeX, edgeY);

    ctx.save();

    ctx.font = `${fontSize}px Arial`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    // テキストのサイズを測定
    const metrics = ctx.measureText(text);
    const textWidth = metrics.width;
    const textHeight = fontSize;

    // 背景を描画
    ctx.fillStyle = 'rgba(102, 126, 234, 0.9)';
    ctx.fillRect(
        canvasPos.x - textWidth / 2 - padding,
        canvasPos.y - textHeight / 2 - padding,
        textWidth + padding * 2,
        textHeight + padding * 2
    );

    // テキストを描画
    ctx.fillStyle = 'white';
    ctx.fillText(text, canvasPos.x, canvasPos.y);

    ctx.restore();
}
