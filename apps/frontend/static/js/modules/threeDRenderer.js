/**
 * @file threeDRenderer.js
 * @description 2Dマップから3D構造を生成・描画（等角投影）
 *
 * Canvas 2Dコンテキストを使用して、簡易的な3D表現を行います。
 * 等角投影（Isometric Projection）を使用して、2Dで3Dっぽく見せます。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 *
 * @exports initialize3DView - 3Dビュー初期化
 * @exports render3DScene - 3Dシーン描画
 * @exports update3DObject - 特定オブジェクト更新
 * @exports toggle3DView - 3Dビュー表示切替
 * @exports set3DViewRotation - 3Dビューの回転角度設定
 * @exports initializePropertyPreview - プロパティプレビュー初期化
 * @exports renderPropertyPreview - プロパティプレビュー描画
 */

import { mapState } from '../state/mapState.js';
import { getAllRectangles, getRectangleById } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';

// ================
// 3Dビュー状態
// ================

const view3DState = {
    canvas: null,
    ctx: null,
    isVisible: false,
    rotation: 45,           // 回転角度（度）
    tilt: 30,              // 傾き角度（度）
    scale: 20,             // スケール（ピクセル/メートル）
    offsetX: 0,            // X方向オフセット
    offsetY: 0,            // Y方向オフセット
    isDragging: false,
    lastMouseX: 0,
    lastMouseY: 0
};

// ================
// 初期化
// ================

/**
 * 3Dビューを初期化する
 *
 * @returns {void}
 */
export function initialize3DView() {
    const canvas = document.getElementById('view3DCanvas');
    if (!canvas) {
        console.error('initialize3DView: 3DビューのCanvas要素が見つかりません');
        return;
    }

    view3DState.canvas = canvas;
    view3DState.ctx = canvas.getContext('2d');

    // Canvasサイズを設定
    resizeCanvas();

    // イベントリスナーを設定
    setupEventListeners();

    console.log('3Dビューを初期化しました');
}

/**
 * Canvasサイズを調整
 *
 * @private
 */
function resizeCanvas() {
    if (!view3DState.canvas) return;

    const container = view3DState.canvas.parentElement;
    if (!container) return;

    view3DState.canvas.width = container.clientWidth;
    view3DState.canvas.height = container.clientHeight;

    // 再描画
    render3DScene();
}

/**
 * イベントリスナーをセットアップ
 *
 * @private
 */
function setupEventListeners() {
    if (!view3DState.canvas) return;

    // マウスイベント（回転・パン）
    view3DState.canvas.addEventListener('mousedown', handle3DMouseDown);
    view3DState.canvas.addEventListener('mousemove', handle3DMouseMove);
    view3DState.canvas.addEventListener('mouseup', handle3DMouseUp);
    view3DState.canvas.addEventListener('mouseleave', handle3DMouseUp);

    // ホイールイベント（ズーム）
    view3DState.canvas.addEventListener('wheel', handle3DWheel);

    // ウィンドウリサイズ
    window.addEventListener('resize', resizeCanvas);
}

// ================
// 描画
// ================

/**
 * 3Dシーン全体を描画する
 *
 * @returns {void}
 */
export function render3DScene() {
    if (!view3DState.ctx || !view3DState.canvas) return;

    const ctx = view3DState.ctx;
    const width = view3DState.canvas.width;
    const height = view3DState.canvas.height;

    // キャンバスをクリア
    ctx.clearRect(0, 0, width, height);

    // 背景を描画
    ctx.fillStyle = '#f7fafc';
    ctx.fillRect(0, 0, width, height);

    // 中心点を設定
    const centerX = width / 2 + view3DState.offsetX;
    const centerY = height / 2 + view3DState.offsetY;

    // グリッドを描画
    drawGrid(ctx, centerX, centerY);

    // すべての四角形を3Dで描画
    const rectangles = getAllRectangles();
    rectangles.forEach(rect => {
        draw3DObject(ctx, rect, centerX, centerY);
    });

    // 情報を表示
    drawInfo(ctx);
}

/**
 * グリッドを描画（床面）
 *
 * @private
 */
function drawGrid(ctx, centerX, centerY) {
    const gridSize = 1; // 1メートルグリッド
    const gridCount = 10;

    ctx.save();
    ctx.strokeStyle = '#cbd5e0';
    ctx.lineWidth = 1;

    for (let i = -gridCount; i <= gridCount; i++) {
        // X方向のグリッド線
        const startX = worldToIso(i * gridSize, -gridCount * gridSize, 0);
        const endX = worldToIso(i * gridSize, gridCount * gridSize, 0);

        ctx.beginPath();
        ctx.moveTo(
            centerX + startX.x * view3DState.scale,
            centerY - startX.y * view3DState.scale
        );
        ctx.lineTo(
            centerX + endX.x * view3DState.scale,
            centerY - endX.y * view3DState.scale
        );
        ctx.stroke();

        // Y方向のグリッド線
        const startY = worldToIso(-gridCount * gridSize, i * gridSize, 0);
        const endY = worldToIso(gridCount * gridSize, i * gridSize, 0);

        ctx.beginPath();
        ctx.moveTo(
            centerX + startY.x * view3DState.scale,
            centerY - startY.y * view3DState.scale
        );
        ctx.lineTo(
            centerX + endY.x * view3DState.scale,
            centerY - endY.y * view3DState.scale
        );
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * 3Dオブジェクトを描画
 *
 * @private
 */
function draw3DObject(ctx, rectangle, centerX, centerY) {
    const coords3D = get3DCoordinates(rectangle.id);
    if (!coords3D) return;

    // オブジェクトタイプに応じた色
    const color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // オブジェクトタイプに応じた3Dモデルを描画
    draw3DModel(ctx, coords3D, color, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties);

    // 前面方向を矢印で表示
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawFrontDirection(ctx, coords3D, rectangle.frontDirection, centerX, centerY);
    }
}

/**
 * オブジェクトタイプに応じた3Dモデルを描画（メインビュー用）
 * @private
 */
function draw3DModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            draw3DShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.BOX:
            draw3DBox_Hollowed(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.TABLE:
            draw3DTable(ctx, coords3D, color, centerX, centerY);
            break;
        case OBJECT_TYPES.DOOR:
            draw3DDoor(ctx, coords3D, color, centerX, centerY);
            break;
        case OBJECT_TYPES.WALL:
            draw3DWall(ctx, coords3D, color, centerX, centerY);
            break;
        case OBJECT_TYPES.NONE:
        default:
            drawBox(ctx, coords3D, color, centerX, centerY);
            break;
    }
}

/**
 * 棚の3Dモデルを描画（前面が開いている）
 * @private
 */
function draw3DShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;

    const vertices = [
        worldToIso(x - width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y + depth/2, z + height/2),
        worldToIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * view3DState.scale,
        y: centerY - v.y * view3DState.scale
    }));

    ctx.save();

    // 前面の向きに応じて、開いている面を決定
    let drawFront = true, drawBack = true, drawLeft = true, drawRight = true;
    switch (frontDirection) {
        case 'top':
            drawFront = false;
            break;
        case 'bottom':
            drawBack = false;
            break;
        case 'left':
            drawLeft = false;
            break;
        case 'right':
            drawRight = false;
            break;
        default:
            drawFront = false;
    }

    // 上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 底面
    ctx.fillStyle = darkenColor(color, 30);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 左面
    if (drawLeft) {
        ctx.fillStyle = darkenColor(color, 10);
        ctx.beginPath();
        ctx.moveTo(screen[0].x, screen[0].y);
        ctx.lineTo(screen[3].x, screen[3].y);
        ctx.lineTo(screen[7].x, screen[7].y);
        ctx.lineTo(screen[4].x, screen[4].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 右面
    if (drawRight) {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(screen[1].x, screen[1].y);
        ctx.lineTo(screen[5].x, screen[5].y);
        ctx.lineTo(screen[6].x, screen[6].y);
        ctx.lineTo(screen[2].x, screen[2].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 前面
    if (drawFront) {
        ctx.fillStyle = darkenColor(color, 5);
        ctx.beginPath();
        ctx.moveTo(screen[0].x, screen[0].y);
        ctx.lineTo(screen[1].x, screen[1].y);
        ctx.lineTo(screen[5].x, screen[5].y);
        ctx.lineTo(screen[4].x, screen[4].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 背面
    if (drawBack) {
        ctx.fillStyle = darkenColor(color, 15);
        ctx.beginPath();
        ctx.moveTo(screen[2].x, screen[2].y);
        ctx.lineTo(screen[3].x, screen[3].y);
        ctx.lineTo(screen[7].x, screen[7].y);
        ctx.lineTo(screen[6].x, screen[6].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 棚板を描画
    if (objectProperties && objectProperties.shelfLevels) {
        const levels = objectProperties.shelfLevels;
        ctx.strokeStyle = darkenColor(color, 30);
        ctx.lineWidth = 2;

        for (let i = 1; i < levels; i++) {
            const levelZ = z - height/2 + (height / levels) * i;
            const v1 = worldToIso(x - width/2, y - depth/2, levelZ);
            const v2 = worldToIso(x + width/2, y - depth/2, levelZ);
            const v3 = worldToIso(x + width/2, y + depth/2, levelZ);
            const v4 = worldToIso(x - width/2, y + depth/2, levelZ);

            const s1 = { x: centerX + v1.x * view3DState.scale, y: centerY - v1.y * view3DState.scale };
            const s2 = { x: centerX + v2.x * view3DState.scale, y: centerY - v2.y * view3DState.scale };
            const s3 = { x: centerX + v3.x * view3DState.scale, y: centerY - v3.y * view3DState.scale };
            const s4 = { x: centerX + v4.x * view3DState.scale, y: centerY - v4.y * view3DState.scale };

            ctx.beginPath();
            ctx.moveTo(s1.x, s1.y);
            ctx.lineTo(s2.x, s2.y);
            ctx.lineTo(s3.x, s3.y);
            ctx.lineTo(s4.x, s4.y);
            ctx.closePath();
            ctx.stroke();
        }
    }

    ctx.restore();
}

/**
 * 箱の3Dモデルを描画（上部が開いている）
 * @private
 */
function draw3DBox_Hollowed(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;

    const vertices = [
        worldToIso(x - width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y + depth/2, z + height/2),
        worldToIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * view3DState.scale,
        y: centerY - v.y * view3DState.scale
    }));

    ctx.save();

    // 底面
    ctx.fillStyle = darkenColor(color, 30);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 左面
    ctx.fillStyle = darkenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 右面
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 前面
    ctx.fillStyle = darkenColor(color, 5);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 背面
    ctx.fillStyle = darkenColor(color, 15);
    ctx.beginPath();
    ctx.moveTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * テーブルの3Dモデルを描画
 * @private
 */
function draw3DTable(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;
    const topThickness = height * 0.1;
    const legWidth = Math.min(width, depth) * 0.1;

    ctx.save();

    // 天板を描画
    const topZ = z + height/2 - topThickness/2;
    const topVertices = [
        worldToIso(x - width/2, y - depth/2, topZ - topThickness/2),
        worldToIso(x + width/2, y - depth/2, topZ - topThickness/2),
        worldToIso(x + width/2, y + depth/2, topZ - topThickness/2),
        worldToIso(x - width/2, y + depth/2, topZ - topThickness/2),
        worldToIso(x - width/2, y - depth/2, topZ + topThickness/2),
        worldToIso(x + width/2, y - depth/2, topZ + topThickness/2),
        worldToIso(x + width/2, y + depth/2, topZ + topThickness/2),
        worldToIso(x - width/2, y + depth/2, topZ + topThickness/2),
    ];

    const topScreen = topVertices.map(v => ({
        x: centerX + v.x * view3DState.scale,
        y: centerY - v.y * view3DState.scale
    }));

    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(topScreen[4].x, topScreen[4].y);
    ctx.lineTo(topScreen[5].x, topScreen[5].y);
    ctx.lineTo(topScreen[6].x, topScreen[6].y);
    ctx.lineTo(topScreen[7].x, topScreen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(topScreen[1].x, topScreen[1].y);
    ctx.lineTo(topScreen[5].x, topScreen[5].y);
    ctx.lineTo(topScreen[6].x, topScreen[6].y);
    ctx.lineTo(topScreen[2].x, topScreen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = darkenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(topScreen[0].x, topScreen[0].y);
    ctx.lineTo(topScreen[3].x, topScreen[3].y);
    ctx.lineTo(topScreen[7].x, topScreen[7].y);
    ctx.lineTo(topScreen[4].x, topScreen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 4本の脚を描画
    const legPositions = [
        { x: x - width/2 + legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y + depth/2 - legWidth },
        { x: x - width/2 + legWidth, y: y + depth/2 - legWidth },
    ];

    legPositions.forEach(pos => {
        const legVertices = [
            worldToIso(pos.x - legWidth/2, pos.y - legWidth/2, z - height/2),
            worldToIso(pos.x + legWidth/2, pos.y - legWidth/2, z - height/2),
            worldToIso(pos.x + legWidth/2, pos.y + legWidth/2, z - height/2),
            worldToIso(pos.x - legWidth/2, pos.y + legWidth/2, z - height/2),
            worldToIso(pos.x - legWidth/2, pos.y - legWidth/2, topZ - topThickness/2),
            worldToIso(pos.x + legWidth/2, pos.y - legWidth/2, topZ - topThickness/2),
            worldToIso(pos.x + legWidth/2, pos.y + legWidth/2, topZ - topThickness/2),
            worldToIso(pos.x - legWidth/2, pos.y + legWidth/2, topZ - topThickness/2),
        ];

        const legScreen = legVertices.map(v => ({
            x: centerX + v.x * view3DState.scale,
            y: centerY - v.y * view3DState.scale
        }));

        ctx.fillStyle = darkenColor(color, 15);
        ctx.beginPath();
        ctx.moveTo(legScreen[1].x, legScreen[1].y);
        ctx.lineTo(legScreen[5].x, legScreen[5].y);
        ctx.lineTo(legScreen[6].x, legScreen[6].y);
        ctx.lineTo(legScreen[2].x, legScreen[2].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = darkenColor(color, 20);
        ctx.beginPath();
        ctx.moveTo(legScreen[0].x, legScreen[0].y);
        ctx.lineTo(legScreen[3].x, legScreen[3].y);
        ctx.lineTo(legScreen[7].x, legScreen[7].y);
        ctx.lineTo(legScreen[4].x, legScreen[4].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    });

    ctx.restore();
}

/**
 * 扉の3Dモデルを描画
 * @private
 */
function draw3DDoor(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;
    const doorDepth = Math.min(depth, 0.1);

    const vertices = [
        worldToIso(x - width/2, y - doorDepth/2, z - height/2),
        worldToIso(x + width/2, y - doorDepth/2, z - height/2),
        worldToIso(x + width/2, y + doorDepth/2, z - height/2),
        worldToIso(x - width/2, y + doorDepth/2, z - height/2),
        worldToIso(x - width/2, y - doorDepth/2, z + height/2),
        worldToIso(x + width/2, y - doorDepth/2, z + height/2),
        worldToIso(x + width/2, y + doorDepth/2, z + height/2),
        worldToIso(x - width/2, y + doorDepth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * view3DState.scale,
        y: centerY - v.y * view3DState.scale
    }));

    ctx.save();

    ctx.fillStyle = lightenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * 壁の3Dモデルを描画
 * @private
 */
function draw3DWall(ctx, coords3D, color, centerX, centerY) {
    drawBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * 直方体（ボックス）を描画
 *
 * @private
 */
function drawBox(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;

    // 8つの頂点を計算
    const vertices = [
        worldToIso(x - width/2, y - depth/2, z - height/2),  // 0: 左下前
        worldToIso(x + width/2, y - depth/2, z - height/2),  // 1: 右下前
        worldToIso(x + width/2, y + depth/2, z - height/2),  // 2: 右下後
        worldToIso(x - width/2, y + depth/2, z - height/2),  // 3: 左下後
        worldToIso(x - width/2, y - depth/2, z + height/2),  // 4: 左上前
        worldToIso(x + width/2, y - depth/2, z + height/2),  // 5: 右上前
        worldToIso(x + width/2, y + depth/2, z + height/2),  // 6: 右上後
        worldToIso(x - width/2, y + depth/2, z + height/2),  // 7: 左上後
    ];

    // スクリーン座標に変換
    const screen = vertices.map(v => ({
        x: centerX + v.x * view3DState.scale,
        y: centerY - v.y * view3DState.scale
    }));

    ctx.save();

    // 面を描画（背面から前面へ）
    // 上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 左面
    ctx.fillStyle = darkenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 右面
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * 前面方向を矢印で描画
 *
 * @private
 */
function drawFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;

    // 前面の中心点を計算
    let arrowStart, arrowEnd;

    switch (frontDirection) {
        case 'top':    // 上（Y負方向）
            arrowStart = worldToIso(x, y - depth/2, z);
            arrowEnd = worldToIso(x, y - depth/2 - 0.3, z);
            break;
        case 'right':  // 右（X正方向）
            arrowStart = worldToIso(x + width/2, y, z);
            arrowEnd = worldToIso(x + width/2 + 0.3, y, z);
            break;
        case 'bottom': // 下（Y正方向）
            arrowStart = worldToIso(x, y + depth/2, z);
            arrowEnd = worldToIso(x, y + depth/2 + 0.3, z);
            break;
        case 'left':   // 左（X負方向）
            arrowStart = worldToIso(x - width/2, y, z);
            arrowEnd = worldToIso(x - width/2 - 0.3, y, z);
            break;
        default:
            return;
    }

    // スクリーン座標に変換
    const startScreen = {
        x: centerX + arrowStart.x * view3DState.scale,
        y: centerY - arrowStart.y * view3DState.scale
    };
    const endScreen = {
        x: centerX + arrowEnd.x * view3DState.scale,
        y: centerY - arrowEnd.y * view3DState.scale
    };

    // 矢印を描画
    ctx.save();
    ctx.strokeStyle = '#2d3748';
    ctx.fillStyle = '#2d3748';
    ctx.lineWidth = 2;

    // 線
    ctx.beginPath();
    ctx.moveTo(startScreen.x, startScreen.y);
    ctx.lineTo(endScreen.x, endScreen.y);
    ctx.stroke();

    // 矢印の頭
    const angle = Math.atan2(endScreen.y - startScreen.y, endScreen.x - startScreen.x);
    const arrowSize = 8;
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

/**
 * 情報表示
 *
 * @private
 */
function drawInfo(ctx) {
    ctx.save();
    ctx.fillStyle = '#2d3748';
    ctx.font = '12px monospace';
    ctx.fillText(`回転: ${view3DState.rotation.toFixed(0)}°`, 10, 20);
    ctx.fillText(`傾き: ${view3DState.tilt.toFixed(0)}°`, 10, 35);
    ctx.fillText(`スケール: ${view3DState.scale.toFixed(1)}x`, 10, 50);
    ctx.restore();
}

// ================
// 座標変換
// ================

/**
 * ワールド座標を等角投影座標に変換
 *
 * @param {number} x - X座標（メートル）
 * @param {number} y - Y座標（メートル）
 * @param {number} z - Z座標（メートル）
 * @returns {Object} {x, y} - 等角投影座標
 *
 * @private
 */
function worldToIso(x, y, z) {
    // 等角投影の変換行列
    // 回転を考慮
    const rad = view3DState.rotation * Math.PI / 180;
    const tiltRad = view3DState.tilt * Math.PI / 180;

    const rotX = x * Math.cos(rad) - y * Math.sin(rad);
    const rotY = x * Math.sin(rad) + y * Math.cos(rad);

    return {
        x: rotX,
        y: z - rotY * Math.sin(tiltRad)
    };
}

// ================
// ユーティリティ
// ================

/**
 * 色を明るくする
 *
 * @private
 */
function lightenColor(color, percent) {
    const num = parseInt(color.replace("#",""), 16);
    const amt = Math.round(2.55 * percent);
    const R = Math.min(255, (num >> 16) + amt);
    const G = Math.min(255, (num >> 8 & 0x00FF) + amt);
    const B = Math.min(255, (num & 0x0000FF) + amt);
    return "#" + (0x1000000 + R * 0x10000 + G * 0x100 + B).toString(16).slice(1);
}

/**
 * 色を暗くする
 *
 * @private
 */
function darkenColor(color, percent) {
    const num = parseInt(color.replace("#",""), 16);
    const amt = Math.round(2.55 * percent);
    const R = Math.max(0, (num >> 16) - amt);
    const G = Math.max(0, (num >> 8 & 0x00FF) - amt);
    const B = Math.max(0, (num & 0x0000FF) - amt);
    return "#" + (0x1000000 + R * 0x10000 + G * 0x100 + B).toString(16).slice(1);
}

// ================
// イベントハンドラ
// ================

/**
 * マウスダウンハンドラ
 *
 * @private
 */
function handle3DMouseDown(event) {
    view3DState.isDragging = true;
    view3DState.lastMouseX = event.clientX;
    view3DState.lastMouseY = event.clientY;
}

/**
 * マウス移動ハンドラ
 *
 * @private
 */
function handle3DMouseMove(event) {
    if (!view3DState.isDragging) return;

    const deltaX = event.clientX - view3DState.lastMouseX;
    const deltaY = event.clientY - view3DState.lastMouseY;

    // 右クリックまたはCtrlキーでパン
    if (event.ctrlKey || event.button === 2) {
        view3DState.offsetX += deltaX;
        view3DState.offsetY += deltaY;
    } else {
        // 回転
        view3DState.rotation += deltaX * 0.5;
        view3DState.tilt = Math.max(0, Math.min(90, view3DState.tilt + deltaY * 0.5));
    }

    view3DState.lastMouseX = event.clientX;
    view3DState.lastMouseY = event.clientY;

    render3DScene();
}

/**
 * マウスアップハンドラ
 *
 * @private
 */
function handle3DMouseUp(event) {
    view3DState.isDragging = false;
}

/**
 * ホイールハンドラ（ズーム）
 *
 * @private
 */
function handle3DWheel(event) {
    event.preventDefault();

    const delta = event.deltaY > 0 ? 0.9 : 1.1;
    view3DState.scale *= delta;
    view3DState.scale = Math.max(5, Math.min(100, view3DState.scale));

    render3DScene();
}

// ================
// 公開API
// ================

/**
 * 特定のオブジェクトを更新
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {void}
 */
export function update3DObject(rectangleId) {
    // 全体を再描画
    render3DScene();
}

/**
 * 3Dビューの表示を切り替え
 *
 * @param {boolean} visible - 表示する場合はtrue
 * @returns {void}
 */
export function toggle3DView(visible) {
    view3DState.isVisible = visible;

    const container = document.getElementById('view3DContainer');
    if (container) {
        container.style.display = visible ? 'block' : 'none';
    }

    if (visible) {
        resizeCanvas();
        render3DScene();
    }
}

/**
 * 3Dビューの回転角度を設定
 *
 * @param {number} rotation - 回転角度（度）
 * @param {number} tilt - 傾き角度（度）
 * @returns {void}
 */
export function set3DViewRotation(rotation, tilt) {
    view3DState.rotation = rotation;
    view3DState.tilt = Math.max(0, Math.min(90, tilt));
    render3DScene();
}

// ================
// プロパティプレビュー
// ================

/**
 * プロパティプレビュー用の状態
 */
const previewState = {
    canvas: null,
    ctx: null,
    rotation: 45,
    tilt: 30,
    scale: 30
};

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

    console.log('プロパティプレビューを初期化しました');
}

/**
 * プロパティプレビューを描画する
 *
 * @param {string} rectangleId - 描画する四角形のID
 * @returns {void}
 */
export function renderPropertyPreview(rectangleId) {
    if (!previewState.ctx || !previewState.canvas) return;

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

    // 中心点を設定
    const centerX = width / 2;
    const centerY = height / 2;

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

    // オブジェクトタイプに応じた3Dモデルを描画
    drawPreviewModel(ctx, coords3D, color, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties);

    // 前面方向を矢印で表示
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawPreviewFrontDirection(ctx, coords3D, rectangle.frontDirection, centerX, centerY);
    }
}

/**
 * プレビュー用グリッド描画
 * @private
 */
function drawPreviewGrid(ctx, centerX, centerY) {
    const gridSize = 1;
    const gridCount = 3;

    ctx.save();
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;

    for (let i = -gridCount; i <= gridCount; i++) {
        // X方向
        const startX = worldToPreviewIso(i * gridSize, -gridCount * gridSize, 0);
        const endX = worldToPreviewIso(i * gridSize, gridCount * gridSize, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startX.x * previewState.scale, centerY - startX.y * previewState.scale);
        ctx.lineTo(centerX + endX.x * previewState.scale, centerY - endX.y * previewState.scale);
        ctx.stroke();

        // Y方向
        const startY = worldToPreviewIso(-gridCount * gridSize, i * gridSize, 0);
        const endY = worldToPreviewIso(gridCount * gridSize, i * gridSize, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startY.x * previewState.scale, centerY - startY.y * previewState.scale);
        ctx.lineTo(centerX + endY.x * previewState.scale, centerY - endY.y * previewState.scale);
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * プレビュー用ボックス描画
 * @private
 */
function drawPreviewBox(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;

    // 8つの頂点を計算
    const vertices = [
        worldToPreviewIso(x - width/2, y - depth/2, z - height/2),
        worldToPreviewIso(x + width/2, y - depth/2, z - height/2),
        worldToPreviewIso(x + width/2, y + depth/2, z - height/2),
        worldToPreviewIso(x - width/2, y + depth/2, z - height/2),
        worldToPreviewIso(x - width/2, y - depth/2, z + height/2),
        worldToPreviewIso(x + width/2, y - depth/2, z + height/2),
        worldToPreviewIso(x + width/2, y + depth/2, z + height/2),
        worldToPreviewIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 左面
    ctx.fillStyle = darkenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 右面
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * オブジェクトタイプに応じたプレビューモデルを描画
 * @private
 */
function drawPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            drawPreviewShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.BOX:
            drawPreviewBox_Hollowed(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.TABLE:
            drawPreviewTable(ctx, coords3D, color, centerX, centerY);
            break;
        case OBJECT_TYPES.DOOR:
            drawPreviewDoor(ctx, coords3D, color, centerX, centerY);
            break;
        case OBJECT_TYPES.WALL:
            drawPreviewWall(ctx, coords3D, color, centerX, centerY);
            break;
        case OBJECT_TYPES.NONE:
        default:
            drawPreviewBox(ctx, coords3D, color, centerX, centerY);
            break;
    }
}

/**
 * 棚のプレビューを描画（前面が開いている）
 * @private
 */
function drawPreviewShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;
    const thickness = 0.05; // 壁の厚さ

    // 8つの頂点を計算
    const vertices = [
        worldToPreviewIso(x - width/2, y - depth/2, z - height/2),
        worldToPreviewIso(x + width/2, y - depth/2, z - height/2),
        worldToPreviewIso(x + width/2, y + depth/2, z - height/2),
        worldToPreviewIso(x - width/2, y + depth/2, z - height/2),
        worldToPreviewIso(x - width/2, y - depth/2, z + height/2),
        worldToPreviewIso(x + width/2, y - depth/2, z + height/2),
        worldToPreviewIso(x + width/2, y + depth/2, z + height/2),
        worldToPreviewIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 前面の向きに応じて、開いている面を決定
    let drawFront = true, drawBack = true, drawLeft = true, drawRight = true;

    switch (frontDirection) {
        case 'top':
            drawFront = false; // y-方向（上）が開く
            break;
        case 'bottom':
            drawBack = false; // y+方向（下）が開く
            break;
        case 'left':
            drawLeft = false; // x-方向（左）が開く
            break;
        case 'right':
            drawRight = false; // x+方向（右）が開く
            break;
        default:
            drawFront = false; // デフォルトは前面が開く
    }

    // 上面（天板）
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 底面
    ctx.fillStyle = darkenColor(color, 30);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 左面
    if (drawLeft) {
        ctx.fillStyle = darkenColor(color, 10);
        ctx.beginPath();
        ctx.moveTo(screen[0].x, screen[0].y);
        ctx.lineTo(screen[3].x, screen[3].y);
        ctx.lineTo(screen[7].x, screen[7].y);
        ctx.lineTo(screen[4].x, screen[4].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 右面
    if (drawRight) {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(screen[1].x, screen[1].y);
        ctx.lineTo(screen[5].x, screen[5].y);
        ctx.lineTo(screen[6].x, screen[6].y);
        ctx.lineTo(screen[2].x, screen[2].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 前面（y-）
    if (drawFront) {
        ctx.fillStyle = darkenColor(color, 5);
        ctx.beginPath();
        ctx.moveTo(screen[0].x, screen[0].y);
        ctx.lineTo(screen[1].x, screen[1].y);
        ctx.lineTo(screen[5].x, screen[5].y);
        ctx.lineTo(screen[4].x, screen[4].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 背面（y+）
    if (drawBack) {
        ctx.fillStyle = darkenColor(color, 15);
        ctx.beginPath();
        ctx.moveTo(screen[2].x, screen[2].y);
        ctx.lineTo(screen[3].x, screen[3].y);
        ctx.lineTo(screen[7].x, screen[7].y);
        ctx.lineTo(screen[6].x, screen[6].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }

    // 棚板を描画（オプション）
    if (objectProperties && objectProperties.shelfLevels) {
        const levels = objectProperties.shelfLevels;
        ctx.strokeStyle = darkenColor(color, 30);
        ctx.lineWidth = 1.5;

        for (let i = 1; i < levels; i++) {
            const levelZ = z - height/2 + (height / levels) * i;
            const v1 = worldToPreviewIso(x - width/2, y - depth/2, levelZ);
            const v2 = worldToPreviewIso(x + width/2, y - depth/2, levelZ);
            const v3 = worldToPreviewIso(x + width/2, y + depth/2, levelZ);
            const v4 = worldToPreviewIso(x - width/2, y + depth/2, levelZ);

            const s1 = { x: centerX + v1.x * previewState.scale, y: centerY - v1.y * previewState.scale };
            const s2 = { x: centerX + v2.x * previewState.scale, y: centerY - v2.y * previewState.scale };
            const s3 = { x: centerX + v3.x * previewState.scale, y: centerY - v3.y * previewState.scale };
            const s4 = { x: centerX + v4.x * previewState.scale, y: centerY - v4.y * previewState.scale };

            ctx.beginPath();
            ctx.moveTo(s1.x, s1.y);
            ctx.lineTo(s2.x, s2.y);
            ctx.lineTo(s3.x, s3.y);
            ctx.lineTo(s4.x, s4.y);
            ctx.closePath();
            ctx.stroke();
        }
    }

    ctx.restore();
}

/**
 * 箱のプレビューを描画（上部が開いている）
 * @private
 */
function drawPreviewBox_Hollowed(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;

    // 8つの頂点を計算
    const vertices = [
        worldToPreviewIso(x - width/2, y - depth/2, z - height/2),
        worldToPreviewIso(x + width/2, y - depth/2, z - height/2),
        worldToPreviewIso(x + width/2, y + depth/2, z - height/2),
        worldToPreviewIso(x - width/2, y + depth/2, z - height/2),
        worldToPreviewIso(x - width/2, y - depth/2, z + height/2),
        worldToPreviewIso(x + width/2, y - depth/2, z + height/2),
        worldToPreviewIso(x + width/2, y + depth/2, z + height/2),
        worldToPreviewIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 上部は開いているので描画しない

    // 底面
    ctx.fillStyle = darkenColor(color, 30);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 左面
    ctx.fillStyle = darkenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 右面
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 前面
    ctx.fillStyle = darkenColor(color, 5);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 背面
    ctx.fillStyle = darkenColor(color, 15);
    ctx.beginPath();
    ctx.moveTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * テーブルのプレビューを描画（天板と脚）
 * @private
 */
function drawPreviewTable(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;
    const topThickness = height * 0.1; // 天板の厚さ
    const legWidth = Math.min(width, depth) * 0.1; // 脚の太さ
    const legHeight = height - topThickness;

    ctx.save();

    // 天板を描画
    const topZ = z + height/2 - topThickness/2;
    const topVertices = [
        worldToPreviewIso(x - width/2, y - depth/2, topZ - topThickness/2),
        worldToPreviewIso(x + width/2, y - depth/2, topZ - topThickness/2),
        worldToPreviewIso(x + width/2, y + depth/2, topZ - topThickness/2),
        worldToPreviewIso(x - width/2, y + depth/2, topZ - topThickness/2),
        worldToPreviewIso(x - width/2, y - depth/2, topZ + topThickness/2),
        worldToPreviewIso(x + width/2, y - depth/2, topZ + topThickness/2),
        worldToPreviewIso(x + width/2, y + depth/2, topZ + topThickness/2),
        worldToPreviewIso(x - width/2, y + depth/2, topZ + topThickness/2),
    ];

    const topScreen = topVertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    // 天板上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(topScreen[4].x, topScreen[4].y);
    ctx.lineTo(topScreen[5].x, topScreen[5].y);
    ctx.lineTo(topScreen[6].x, topScreen[6].y);
    ctx.lineTo(topScreen[7].x, topScreen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 天板側面
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(topScreen[1].x, topScreen[1].y);
    ctx.lineTo(topScreen[5].x, topScreen[5].y);
    ctx.lineTo(topScreen[6].x, topScreen[6].y);
    ctx.lineTo(topScreen[2].x, topScreen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = darkenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(topScreen[0].x, topScreen[0].y);
    ctx.lineTo(topScreen[3].x, topScreen[3].y);
    ctx.lineTo(topScreen[7].x, topScreen[7].y);
    ctx.lineTo(topScreen[4].x, topScreen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 4本の脚を描画
    const legPositions = [
        { x: x - width/2 + legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y + depth/2 - legWidth },
        { x: x - width/2 + legWidth, y: y + depth/2 - legWidth },
    ];

    legPositions.forEach(pos => {
        const legVertices = [
            worldToPreviewIso(pos.x - legWidth/2, pos.y - legWidth/2, z - height/2),
            worldToPreviewIso(pos.x + legWidth/2, pos.y - legWidth/2, z - height/2),
            worldToPreviewIso(pos.x + legWidth/2, pos.y + legWidth/2, z - height/2),
            worldToPreviewIso(pos.x - legWidth/2, pos.y + legWidth/2, z - height/2),
            worldToPreviewIso(pos.x - legWidth/2, pos.y - legWidth/2, topZ - topThickness/2),
            worldToPreviewIso(pos.x + legWidth/2, pos.y - legWidth/2, topZ - topThickness/2),
            worldToPreviewIso(pos.x + legWidth/2, pos.y + legWidth/2, topZ - topThickness/2),
            worldToPreviewIso(pos.x - legWidth/2, pos.y + legWidth/2, topZ - topThickness/2),
        ];

        const legScreen = legVertices.map(v => ({
            x: centerX + v.x * previewState.scale,
            y: centerY - v.y * previewState.scale
        }));

        // 脚の側面
        ctx.fillStyle = darkenColor(color, 15);
        ctx.beginPath();
        ctx.moveTo(legScreen[1].x, legScreen[1].y);
        ctx.lineTo(legScreen[5].x, legScreen[5].y);
        ctx.lineTo(legScreen[6].x, legScreen[6].y);
        ctx.lineTo(legScreen[2].x, legScreen[2].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = darkenColor(color, 20);
        ctx.beginPath();
        ctx.moveTo(legScreen[0].x, legScreen[0].y);
        ctx.lineTo(legScreen[3].x, legScreen[3].y);
        ctx.lineTo(legScreen[7].x, legScreen[7].y);
        ctx.lineTo(legScreen[4].x, legScreen[4].y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    });

    ctx.restore();
}

/**
 * 扉のプレビューを描画（薄い長方形）
 * @private
 */
function drawPreviewDoor(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;
    // 扉は薄いので、depthを小さくする
    const doorDepth = Math.min(depth, 0.1);

    const vertices = [
        worldToPreviewIso(x - width/2, y - doorDepth/2, z - height/2),
        worldToPreviewIso(x + width/2, y - doorDepth/2, z - height/2),
        worldToPreviewIso(x + width/2, y + doorDepth/2, z - height/2),
        worldToPreviewIso(x - width/2, y + doorDepth/2, z - height/2),
        worldToPreviewIso(x - width/2, y - doorDepth/2, z + height/2),
        worldToPreviewIso(x + width/2, y - doorDepth/2, z + height/2),
        worldToPreviewIso(x + width/2, y + doorDepth/2, z + height/2),
        worldToPreviewIso(x - width/2, y + doorDepth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 前面（大きい面）
    ctx.fillStyle = lightenColor(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darkenColor(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 右面（薄い面）
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * 壁のプレビューを描画（通常の長方形）
 * @private
 */
function drawPreviewWall(ctx, coords3D, color, centerX, centerY) {
    // 壁は通常のボックスとして描画
    drawPreviewBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * プレビュー用前面方向矢印描画
 * @private
 */
function drawPreviewFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    const { x, y, z, width, depth } = coords3D;

    let arrowStart, arrowEnd;

    switch (frontDirection) {
        case 'top':
            arrowStart = worldToPreviewIso(x, y - depth/2, z);
            arrowEnd = worldToPreviewIso(x, y - depth/2 - 0.3, z);
            break;
        case 'right':
            arrowStart = worldToPreviewIso(x + width/2, y, z);
            arrowEnd = worldToPreviewIso(x + width/2 + 0.3, y, z);
            break;
        case 'bottom':
            arrowStart = worldToPreviewIso(x, y + depth/2, z);
            arrowEnd = worldToPreviewIso(x, y + depth/2 + 0.3, z);
            break;
        case 'left':
            arrowStart = worldToPreviewIso(x - width/2, y, z);
            arrowEnd = worldToPreviewIso(x - width/2 - 0.3, y, z);
            break;
        default:
            return;
    }

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

/**
 * プレビュー用等角投影変換
 * @private
 */
function worldToPreviewIso(x, y, z) {
    const rad = previewState.rotation * Math.PI / 180;
    const tiltRad = previewState.tilt * Math.PI / 180;

    const rotX = x * Math.cos(rad) - y * Math.sin(rad);
    const rotY = x * Math.sin(rad) + y * Math.cos(rad);

    return {
        x: rotX,
        y: z - rotY * Math.sin(tiltRad)
    };
}
