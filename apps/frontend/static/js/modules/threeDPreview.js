/**
 * @file threeDPreview.js
 * @description プロパティパネル用3Dプレビュー管理
 *
 * オブジェクトのプロパティパネルに表示される3Dプレビューを管理します。
 * ユーザーはホイールでズーム操作が可能です。
 *
 * @requires ../state/mapState.js - マップ状態管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../utils/threeDUtils.js - 3D描画ユーティリティ
 * @requires ./threeDModels.js - 3Dモデル描画
 *
 * @exports initializePropertyPreview - プロパティプレビュー初期化
 * @exports renderPropertyPreview - プロパティプレビュー描画
 * @exports getPreviewState - プレビュー状態取得
 * @exports drawPreviewGrid - プレビュー用グリッド描画
 * @exports drawPreviewFrontDirection - プレビュー用前面方向矢印描画
 */

import { mapState } from '../state/mapState.js';
import { getRectangleById } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';
import { worldToPreviewIso, applyRotation } from '../utils/threeDUtils.js';
import {
    drawPreviewShelf,
    drawPreviewBox,
    drawPreviewTable,
    drawPreviewDoor,
    drawPreviewWall
} from './threeDModels.js';

// ================
// プロパティプレビュー用状態
// ================

/**
 * プロパティプレビュー用の状態
 */
const previewState = {
    canvas: null,
    ctx: null,
    rotation: 45,
    tilt: 30,
    scale: 30,
    minScale: 10,
    maxScale: 100,
    currentRectangleId: null  // 現在表示中のrectangleId
};

// ================
// 初期化
// ================

/**
 * プロパティプレビューを初期化する
 *
 * @returns {void}
 */
export function initializePropertyPreview() {
    const canvas = document.getElementById('propertyPreviewCanvas');
    if (!canvas) {
        console.error('initializePropertyPreview: プレビューCanvas要素が見つかりません');
        return;
    }

    previewState.canvas = canvas;
    previewState.ctx = canvas.getContext('2d');

    // Canvasサイズを設定
    const container = canvas.parentElement;
    if (container) {
        previewState.canvas.width = container.clientWidth;
        previewState.canvas.height = container.clientHeight;
    }

    // ホイールイベントでズーム（スケール変更）
    canvas.addEventListener('wheel', handlePreviewWheel, { passive: false });

    console.log('プロパティプレビューを初期化しました');
}

/**
 * プレビュー用ホイールイベントハンドラ（ズーム）
 *
 * @private
 * @param {WheelEvent} e - ホイールイベント
 */
function handlePreviewWheel(e) {
    e.preventDefault();
    e.stopPropagation();

    // ホイールの方向に応じてスケールを変更
    const delta = e.deltaY > 0 ? -2 : 2;
    previewState.scale = Math.max(
        previewState.minScale,
        Math.min(previewState.maxScale, previewState.scale + delta)
    );

    // 現在表示中のオブジェクトを再描画
    if (previewState.currentRectangleId) {
        renderPropertyPreview(previewState.currentRectangleId);
    }
}

// ================
// 描画
// ================

/**
 * プロパティプレビューを描画する
 *
 * @param {string} rectangleId - 描画する四角形のID
 * @returns {void}
 */
export function renderPropertyPreview(rectangleId) {
    if (!previewState.ctx || !previewState.canvas) return;

    // 現在表示中のrectangleIdを保存（ズーム時の再描画用）
    previewState.currentRectangleId = rectangleId;

    // パネルが表示された後、親要素のサイズに合わせてcanvasを再設定
    // 初期化時はパネルがdisplay:noneのためサイズが0x0になっているので、
    // 描画の度にサイズをチェックして必要に応じて更新する
    const container = previewState.canvas.parentElement;
    if (container) {
        const containerWidth = container.clientWidth;
        const containerHeight = container.clientHeight;

        // サイズが変更されている、またはcanvasが0x0の場合は更新
        if (previewState.canvas.width !== containerWidth ||
            previewState.canvas.height !== containerHeight ||
            previewState.canvas.width === 0 ||
            previewState.canvas.height === 0) {
            previewState.canvas.width = containerWidth;
            previewState.canvas.height = containerHeight;
        }
    }

    const ctx = previewState.ctx;
    const width = previewState.canvas.width;
    const height = previewState.canvas.height;

    // キャンバスをクリア
    ctx.clearRect(0, 0, width, height);

    // 背景を描画
    ctx.fillStyle = '#f7fafc';
    ctx.fillRect(0, 0, width, height);

    // 中心点を設定（配置面を下にして、モデル全体が見えるようにする）
    const centerX = width / 2;
    const centerY = height * 0.65; // 配置面を画面の下半分に配置

    // 四角形の3D座標を取得
    const coords3D = get3DCoordinates(rectangleId);
    if (!coords3D) return;

    // 四角形オブジェクトを取得
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    // オブジェクトタイプに応じた色
    const color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // グリッドを描画（簡易版）
    drawPreviewGrid(ctx, centerX, centerY);

    // プレビュー用に座標を調整（原点中心に配置）
    // マップ上の絶対座標ではなく、オブジェクト単体を中央に表示
    const previewCoords = {
        x: 0,  // 原点中心
        y: 0,  // 原点中心
        z: coords3D.height / 2,  // 高さの半分（底面からの中心）
        width: coords3D.width,
        depth: coords3D.depth,
        height: coords3D.height,
        rotation: coords3D.rotation || 0,  // オブジェクトの回転を反映
        frontDirection: rectangle.frontDirection || 'top'
    };

    // オブジェクトタイプに応じた3Dモデルを描画
    drawPreviewModel(ctx, previewCoords, color, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties);

    // 前面方向を矢印で表示
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawPreviewFrontDirection(ctx, previewCoords, rectangle.frontDirection, centerX, centerY);
    }
}

/**
 * プレビュー状態を取得する（外部から設定可能）
 *
 * @returns {Object} プレビュー状態オブジェクト
 */
export function getPreviewState() {
    return previewState;
}

/**
 * プレビュー用グリッド描画
 * 2Dマップと同じグリッド幅を使用
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 */
export function drawPreviewGrid(ctx, centerX, centerY) {
    const gridSize = mapState.gridWidthInMeters || 1; // 2Dマップと同じグリッド幅を使用
    const gridCount = 5; // プレビュー範囲を拡大（3→5）

    ctx.save();
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;

    for (let i = -gridCount; i <= gridCount; i++) {
        // X方向
        const startX = worldToPreviewIso(i * gridSize, -gridCount * gridSize, 0, previewState);
        const endX = worldToPreviewIso(i * gridSize, gridCount * gridSize, 0, previewState);

        ctx.beginPath();
        ctx.moveTo(centerX + startX.x * previewState.scale, centerY - startX.y * previewState.scale);
        ctx.lineTo(centerX + endX.x * previewState.scale, centerY - endX.y * previewState.scale);
        ctx.stroke();

        // Y方向
        const startY = worldToPreviewIso(-gridCount * gridSize, i * gridSize, 0, previewState);
        const endY = worldToPreviewIso(gridCount * gridSize, i * gridSize, 0, previewState);

        ctx.beginPath();
        ctx.moveTo(centerX + startY.x * previewState.scale, centerY - startY.y * previewState.scale);
        ctx.lineTo(centerX + endY.x * previewState.scale, centerY - endY.y * previewState.scale);
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * オブジェクトタイプに応じたプレビューモデルを描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {string} objectType - オブジェクトタイプ
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 */
function drawPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            drawPreviewShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, worldToPreviewIso, previewState);
            break;
        case OBJECT_TYPES.BOX:
            drawPreviewBox(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, worldToPreviewIso, previewState);
            break;
        case OBJECT_TYPES.TABLE:
            drawPreviewTable(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState);
            break;
        case OBJECT_TYPES.DOOR:
            drawPreviewDoor(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState);
            break;
        case OBJECT_TYPES.WALL:
            drawPreviewWall(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState);
            break;
        case OBJECT_TYPES.NONE:
        default:
            // NONEの場合は通常のボックスを描画
            const drawPreviewBoxSimple = (ctx, coords, color, cx, cy) => {
                const { x, y, z, width, depth, height, rotation } = coords;

                // 簡易的なボックス描画
                const localVertices = [
                    { lx: -width/2, ly: -depth/2, lz: -height/2 },
                    { lx: +width/2, ly: -depth/2, lz: -height/2 },
                    { lx: +width/2, ly: +depth/2, lz: -height/2 },
                    { lx: -width/2, ly: +depth/2, lz: -height/2 },
                    { lx: -width/2, ly: -depth/2, lz: +height/2 },
                    { lx: +width/2, ly: -depth/2, lz: +height/2 },
                    { lx: +width/2, ly: +depth/2, lz: +height/2 },
                    { lx: -width/2, ly: +depth/2, lz: +height/2 },
                ];

                const vertices = localVertices.map(v => {
                    const rotated = applyRotation(v.lx, v.ly, rotation);
                    return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz, previewState);
                });

                const screen = vertices.map(v => ({
                    x: cx + v.x * previewState.scale,
                    y: cy - v.y * previewState.scale
                }));

                // 簡易的な描画（深度ソートなし）
                ctx.save();

                // 底面
                ctx.fillStyle = '#cbd5e0';
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[2].x, screen[2].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = '#a0aec0';
                ctx.lineWidth = 1;
                ctx.stroke();

                // 上面
                ctx.fillStyle = '#e2e8f0';
                ctx.beginPath();
                ctx.moveTo(screen[4].x, screen[4].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = '#a0aec0';
                ctx.lineWidth = 1;
                ctx.stroke();

                // 側面（簡易）
                ctx.fillStyle = '#d3dce6';
                ctx.beginPath();
                ctx.moveTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.lineTo(screen[2].x, screen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = '#a0aec0';
                ctx.lineWidth = 1;
                ctx.stroke();

                ctx.restore();
            };
            drawPreviewBoxSimple(ctx, coords3D, color, centerX, centerY);
            break;
    }
}

/**
 * プレビュー用前面方向矢印描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標
 * @param {string} frontDirection - 前面方向
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 */
export function drawPreviewFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    const { x, y, z, width, depth, rotation } = coords3D;

    let localStart, localEnd;

    // ローカル座標で矢印の開始・終了位置を定義
    switch (frontDirection) {
        case 'top':
            localStart = { lx: 0, ly: -depth/2 };
            localEnd = { lx: 0, ly: -depth/2 - 0.3 };
            break;
        case 'right':
            localStart = { lx: width/2, ly: 0 };
            localEnd = { lx: width/2 + 0.3, ly: 0 };
            break;
        case 'bottom':
            localStart = { lx: 0, ly: depth/2 };
            localEnd = { lx: 0, ly: depth/2 + 0.3 };
            break;
        case 'left':
            localStart = { lx: -width/2, ly: 0 };
            localEnd = { lx: -width/2 - 0.3, ly: 0 };
            break;
        default:
            return;
    }

    // 回転を適用
    const rotatedStart = applyRotation(localStart.lx, localStart.ly, rotation);
    const rotatedEnd = applyRotation(localEnd.lx, localEnd.ly, rotation);

    // ワールド座標に変換
    const arrowStart = worldToPreviewIso(x + rotatedStart.x, y + rotatedStart.y, z, previewState);
    const arrowEnd = worldToPreviewIso(x + rotatedEnd.x, y + rotatedEnd.y, z, previewState);

    const startScreen = {
        x: centerX + arrowStart.x * previewState.scale,
        y: centerY - arrowStart.y * previewState.scale
    };
    const endScreen = {
        x: centerX + arrowEnd.x * previewState.scale,
        y: centerY - arrowEnd.y * previewState.scale
    };

    ctx.save();
    ctx.strokeStyle = '#2d3748';
    ctx.fillStyle = '#2d3748';
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(startScreen.x, startScreen.y);
    ctx.lineTo(endScreen.x, endScreen.y);
    ctx.stroke();

    const angle = Math.atan2(endScreen.y - startScreen.y, endScreen.x - startScreen.x);
    const arrowSize = 6;
    ctx.beginPath();
    ctx.moveTo(endScreen.x, endScreen.y);
    ctx.lineTo(
        endScreen.x - arrowSize * Math.cos(angle - Math.PI / 6),
        endScreen.y - arrowSize * Math.sin(angle - Math.PI / 6)
    );
    ctx.lineTo(
        endScreen.x - arrowSize * Math.cos(angle + Math.PI / 6),
        endScreen.y - arrowSize * Math.sin(angle + Math.PI / 6)
    );
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}
