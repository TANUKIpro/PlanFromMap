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
import { getAllRectangles } from '../modules/rectangleManager.js';
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

    // 直方体を描画
    drawBox(ctx, coords3D, color, centerX, centerY);

    // 前面方向を矢印で表示
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawFrontDirection(ctx, coords3D, rectangle.frontDirection, centerX, centerY);
    }
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
    import('../modules/rectangleManager.js').then(({ getRectangleById }) => {
        const rectangle = getRectangleById(rectangleId);
        if (!rectangle) return;

        // オブジェクトタイプに応じた色
        const color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
            ? OBJECT_TYPE_COLORS[rectangle.objectType]
            : OBJECT_TYPE_COLORS.none;

        // グリッドを描画（簡易版）
        drawPreviewGrid(ctx, centerX, centerY);

        // 直方体を描画
        drawPreviewBox(ctx, coords3D, color, centerX, centerY);

        // 前面方向を矢印で表示
        if (rectangle.objectType !== OBJECT_TYPES.NONE) {
            drawPreviewFrontDirection(ctx, coords3D, rectangle.frontDirection, centerX, centerY);
        }
    });
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
