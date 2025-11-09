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
 * @requires ../ui/viewCube.js - ViewCube（視点切り替えコントロール）
 *
 * @exports initialize3DView - 3Dビュー初期化
 * @exports render3DScene - 3Dシーン描画
 * @exports update3DObject - 特定オブジェクト更新
 * @exports resize3DView - 3Dビューのリサイズ
 * @exports set3DViewRotation - 3Dビューの回転角度設定
 * @exports reset3DView - 3Dビューのリセット
 * @exports initializePropertyPreview - プロパティプレビュー初期化
 * @exports renderPropertyPreview - プロパティプレビュー描画
 * @exports drawPreviewGrid - プレビュー用グリッド描画
 * @exports drawPreviewModel - プレビュー用3Dモデル描画
 * @exports drawPreviewFrontDirection - プレビュー用前面方向矢印描画
 * @exports worldToPreviewIso - プレビュー用等角投影変換
 * @exports getPreviewState - プレビュー状態取得
 */

import { mapState } from '../state/mapState.js';
import { getAllRectangles, getRectangleById, getRectangleLayer } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';
import { initializeViewCube, updateViewCube } from '../ui/viewCube.js';
import { analyzeMapBounds } from '../utils/imageProcessing.js';

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
    lastMouseY: 0,
    mouseDownX: 0,         // マウスダウン時のX座標（クリック判定用）
    mouseDownY: 0,         // マウスダウン時のY座標（クリック判定用）
    selectedObjectId: null, // 選択されたオブジェクトのID
    mapBounds: null        // マップの有効領域 {minX, minY, maxX, maxY, centerX, centerY}
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

    // ViewCubeを初期化
    const viewCubeCanvas = document.getElementById('viewCubeCanvas');
    if (viewCubeCanvas) {
        initializeViewCube(viewCubeCanvas, handleViewCubeChange);
        console.log('ViewCubeを初期化しました');
    }

    console.log('3Dビューを初期化しました');
}

/**
 * ViewCubeからの視点変更を処理
 *
 * @private
 * @param {number} rotation - 回転角度
 * @param {number} tilt - 傾き角度
 */
function handleViewCubeChange(rotation, tilt) {
    view3DState.rotation = rotation;
    view3DState.tilt = tilt;
    render3DScene();
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

    // マップ画像を床面に描画（グリッドの前に）
    drawMapTexture(ctx, centerX, centerY);

    // グリッドを描画（マップ画像の上に）- 2Dマップの設定と同期
    if (mapState.overlaySettings.showGrid) {
        drawGrid(ctx, centerX, centerY);
    }

    // 原点を描画 - 2Dマップの設定と同期
    if (mapState.overlaySettings.showOrigin) {
        draw3DOrigin(ctx, centerX, centerY);
    }

    // すべての四角形を3Dで描画（2Dマップのレイヤー可視性設定を反映）
    const rectangleLayer = getRectangleLayer();
    if (rectangleLayer && rectangleLayer.visible) {
        const rectangles = getAllRectangles();
        rectangles.forEach(rect => {
            draw3DObject(ctx, rect, centerX, centerY);
        });
    }

    // 情報を表示
    drawInfo(ctx);

    // ViewCubeを更新
    updateViewCube(view3DState.rotation, view3DState.tilt);
}

// analyzeMapBounds()関数は utils/imageProcessing.js に移動されました

/**
 * マップ画像を床面に描画（テクスチャ）
 * 有効領域のみを描画し、カメラ中心はマップの重心
 *
 * @private
 */
function drawMapTexture(ctx, centerX, centerY) {
    if (!mapState.image) return;

    const image = mapState.image;
    const resolution = mapState.metadata?.resolution || 0.05; // m/pixel

    // マップの有効領域を解析（初回のみ）
    if (!view3DState.mapBounds) {
        view3DState.mapBounds = analyzeMapBounds(image);
    }

    if (!view3DState.mapBounds) return;

    const bounds = view3DState.mapBounds;

    // マップの原点を取得（ROSのマップ原点: マップ左下の実世界座標）
    // metadata.originは配列形式 [x, y, theta]
    const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
    const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

    // 有効領域の実世界座標を計算
    // ROSマップは左下が原点、画像は左上が原点なので Y 軸を反転
    const boundsMinX = originX + bounds.minX * resolution;
    const boundsMinY = originY + (image.height - bounds.maxY) * resolution;
    const boundsMaxX = originX + bounds.maxX * resolution;
    const boundsMaxY = originY + (image.height - bounds.minY) * resolution;

    // 有効領域のサイズ（メートル）
    const boundsWidthMeters = boundsMaxX - boundsMinX;
    const boundsHeightMeters = boundsMaxY - boundsMinY;

    // タイルサイズ（メートル）- 画像を細かく分割して描画
    const tileSize = 0.1; // 10cm単位で分割
    const tilesX = Math.ceil(boundsWidthMeters / tileSize);
    const tilesY = Math.ceil(boundsHeightMeters / tileSize);

    // 画像データをキャッシュ（初回のみ）
    if (!drawMapTexture._imageData) {
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = image.width;
        tempCanvas.height = image.height;
        const tempCtx = tempCanvas.getContext('2d');
        tempCtx.drawImage(image, 0, 0);
        drawMapTexture._imageData = tempCtx.getImageData(0, 0, image.width, image.height);
    }

    const imageData = drawMapTexture._imageData;

    ctx.save();

    // 各タイルを描画
    for (let ty = 0; ty < tilesY; ty++) {
        for (let tx = 0; tx < tilesX; tx++) {
            // タイルの実世界座標（有効領域内）
            const tileX = boundsMinX + tx * tileSize;
            const tileY = boundsMinY + ty * tileSize;
            const tileZ = 0; // 床面

            // タイルの画像座標（ピクセル）
            const imgX = bounds.minX + tx * tileSize / resolution;
            const imgY = image.height - (bounds.minY + (ty + 1) * tileSize / resolution);

            // 画像範囲チェック
            if (imgX < 0 || imgY < 0 || imgX >= image.width || imgY >= image.height) continue;

            // 画像から色を取得（中心ピクセル）
            const sampleX = Math.floor(Math.min(imgX, image.width - 1));
            const sampleY = Math.floor(Math.min(imgY, image.height - 1));

            const pixelIndex = (sampleY * image.width + sampleX) * 4;
            const r = imageData.data[pixelIndex];
            const g = imageData.data[pixelIndex + 1];
            const b = imageData.data[pixelIndex + 2];
            const color = `rgb(${r}, ${g}, ${b})`;

            // タイルの4つの角を等角投影
            const corners = [
                worldToIso(tileX, tileY, tileZ),
                worldToIso(tileX + tileSize, tileY, tileZ),
                worldToIso(tileX + tileSize, tileY + tileSize, tileZ),
                worldToIso(tileX, tileY + tileSize, tileZ)
            ];

            const screenCorners = corners.map(v => ({
                x: centerX + v.x * view3DState.scale,
                y: centerY - v.y * view3DState.scale
            }));

            // タイルを描画
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screenCorners[0].x, screenCorners[0].y);
            ctx.lineTo(screenCorners[1].x, screenCorners[1].y);
            ctx.lineTo(screenCorners[2].x, screenCorners[2].y);
            ctx.lineTo(screenCorners[3].x, screenCorners[3].y);
            ctx.closePath();
            ctx.fill();
        }
    }

    ctx.restore();
}

/**
 * グリッドを描画（床面）
 * テクスチャの有効領域に合わせてグリッドを配置
 * 2Dマップで指定されたグリッド幅を使用
 *
 * @private
 */
function drawGrid(ctx, centerX, centerY) {
    const gridSize = mapState.gridWidthInMeters || 1; // 2Dマップと同じグリッド幅を使用

    // マップの有効領域がない場合はデフォルトのグリッドを描画
    if (!view3DState.mapBounds || !mapState.image) {
        const defaultGridCount = 10;
        ctx.save();
        ctx.strokeStyle = '#cbd5e0';
        ctx.lineWidth = 1;

        for (let i = -defaultGridCount; i <= defaultGridCount; i++) {
            // X方向のグリッド線
            const startX = worldToIso(i * gridSize, -defaultGridCount * gridSize, 0);
            const endX = worldToIso(i * gridSize, defaultGridCount * gridSize, 0);

            ctx.beginPath();
            ctx.moveTo(centerX + startX.x * view3DState.scale, centerY - startX.y * view3DState.scale);
            ctx.lineTo(centerX + endX.x * view3DState.scale, centerY - endX.y * view3DState.scale);
            ctx.stroke();

            // Y方向のグリッド線
            const startY = worldToIso(-defaultGridCount * gridSize, i * gridSize, 0);
            const endY = worldToIso(defaultGridCount * gridSize, i * gridSize, 0);

            ctx.beginPath();
            ctx.moveTo(centerX + startY.x * view3DState.scale, centerY - startY.y * view3DState.scale);
            ctx.lineTo(centerX + endY.x * view3DState.scale, centerY - endY.y * view3DState.scale);
            ctx.stroke();
        }

        ctx.restore();
        return;
    }

    const resolution = mapState.metadata?.resolution || 0.05;
    const bounds = view3DState.mapBounds;
    const image = mapState.image;

    // マップの原点を取得（ROSのマップ原点: マップ左下の実世界座標）
    // metadata.originは配列形式 [x, y, theta]
    const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
    const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

    // 有効領域の実世界座標を計算
    // ROSマップは左下が原点、画像は左上が原点なので Y 軸を反転
    const boundsMinX = originX + bounds.minX * resolution;
    const boundsMinY = originY + (image.height - bounds.maxY) * resolution;
    const boundsMaxX = originX + bounds.maxX * resolution;
    const boundsMaxY = originY + (image.height - bounds.minY) * resolution;

    // グリッドの開始位置をgridSizeの整数倍に揃える
    const gridStartX = Math.floor(boundsMinX / gridSize) * gridSize;
    const gridStartY = Math.floor(boundsMinY / gridSize) * gridSize;
    const gridEndX = Math.ceil(boundsMaxX / gridSize) * gridSize;
    const gridEndY = Math.ceil(boundsMaxY / gridSize) * gridSize;

    ctx.save();
    ctx.strokeStyle = '#cbd5e0';
    ctx.lineWidth = 1;

    // X方向のグリッド線（縦線）
    for (let x = gridStartX; x <= gridEndX; x += gridSize) {
        const startIso = worldToIso(x, gridStartY, 0);
        const endIso = worldToIso(x, gridEndY, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startIso.x * view3DState.scale, centerY - startIso.y * view3DState.scale);
        ctx.lineTo(centerX + endIso.x * view3DState.scale, centerY - endIso.y * view3DState.scale);
        ctx.stroke();
    }

    // Y方向のグリッド線（横線）
    for (let y = gridStartY; y <= gridEndY; y += gridSize) {
        const startIso = worldToIso(gridStartX, y, 0);
        const endIso = worldToIso(gridEndX, y, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startIso.x * view3DState.scale, centerY - startIso.y * view3DState.scale);
        ctx.lineTo(centerX + endIso.x * view3DState.scale, centerY - endIso.y * view3DState.scale);
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * 3D空間に原点マーカーを床面テクスチャとして描画
 * マップ画像と同じように床面（Z=0）に原点マーカーを貼り付けます
 *
 * @private
 */
function draw3DOrigin(ctx, centerX, centerY) {
    if (!mapState.metadata || !mapState.metadata.origin) {
        return;
    }

    ctx.save();

    // 原点の3D座標（床面）
    const originX = 0;
    const originY = 0;
    const originZ = 0; // 床面

    // 十字のサイズ（メートル単位）
    const crossSizeMeters = 0.15; // 15cm
    const lineWidthMeters = 0.02; // 2cm

    // 赤い十字を床面に描画（4本の細い四角形）
    const crossColor = 'rgba(231, 76, 60, 0.9)';

    // 水平線（左）
    drawFloorRectangle(ctx, centerX, centerY,
        originX - crossSizeMeters, originY - lineWidthMeters / 2, originZ,
        originX, originY + lineWidthMeters / 2, originZ,
        crossColor);

    // 水平線（右）
    drawFloorRectangle(ctx, centerX, centerY,
        originX, originY - lineWidthMeters / 2, originZ,
        originX + crossSizeMeters, originY + lineWidthMeters / 2, originZ,
        crossColor);

    // 垂直線（上）
    drawFloorRectangle(ctx, centerX, centerY,
        originX - lineWidthMeters / 2, originY, originZ,
        originX + lineWidthMeters / 2, originY + crossSizeMeters, originZ,
        crossColor);

    // 垂直線（下）
    drawFloorRectangle(ctx, centerX, centerY,
        originX - lineWidthMeters / 2, originY - crossSizeMeters, originZ,
        originX + lineWidthMeters / 2, originY, originZ,
        crossColor);

    // 方向矢印を床面に描画（theta方向）
    const theta = Array.isArray(mapState.metadata.origin) && mapState.metadata.origin.length >= 3
        ? mapState.metadata.origin[2]
        : 0;

    // 矢印の長さ（3D空間での実寸）
    const arrowLengthMeters = 0.5; // 50cm
    const arrowWidthMeters = 0.015; // 1.5cm

    // 矢印の終点（3D座標）
    const arrowEndX = originX + Math.cos(theta) * arrowLengthMeters;
    const arrowEndY = originY + Math.sin(theta) * arrowLengthMeters;
    const arrowEndZ = originZ;

    // 矢印の軸（細い四角形）
    const perpX = -Math.sin(theta) * arrowWidthMeters / 2;
    const perpY = Math.cos(theta) * arrowWidthMeters / 2;

    drawFloorRectangle(ctx, centerX, centerY,
        originX + perpX, originY + perpY, originZ,
        arrowEndX + perpX, arrowEndY + perpY, originZ,
        crossColor);
    drawFloorRectangle(ctx, centerX, centerY,
        arrowEndX + perpX, arrowEndY + perpY, originZ,
        arrowEndX - perpX, arrowEndY - perpY, originZ,
        crossColor);
    drawFloorRectangle(ctx, centerX, centerY,
        arrowEndX - perpX, arrowEndY - perpY, originZ,
        originX - perpX, originY - perpY, originZ,
        crossColor);
    drawFloorRectangle(ctx, centerX, centerY,
        originX - perpX, originY - perpY, originZ,
        originX + perpX, originY + perpY, originZ,
        crossColor);

    // 矢印の先端（三角形）
    const headLengthMeters = 0.1; // 10cm
    const headWidthMeters = 0.08; // 8cm

    // 矢印の先端の3点
    const tipX = arrowEndX + Math.cos(theta) * headLengthMeters;
    const tipY = arrowEndY + Math.sin(theta) * headLengthMeters;

    const leftX = arrowEndX + Math.cos(theta + Math.PI * 2/3) * headWidthMeters;
    const leftY = arrowEndY + Math.sin(theta + Math.PI * 2/3) * headWidthMeters;

    const rightX = arrowEndX + Math.cos(theta - Math.PI * 2/3) * headWidthMeters;
    const rightY = arrowEndY + Math.sin(theta - Math.PI * 2/3) * headWidthMeters;

    // 三角形を床面に描画
    const tip = worldToIso(tipX, tipY, originZ);
    const left = worldToIso(leftX, leftY, originZ);
    const right = worldToIso(rightX, rightY, originZ);

    ctx.fillStyle = crossColor;
    ctx.beginPath();
    ctx.moveTo(centerX + tip.x * view3DState.scale, centerY - tip.y * view3DState.scale);
    ctx.lineTo(centerX + left.x * view3DState.scale, centerY - left.y * view3DState.scale);
    ctx.lineTo(centerX + right.x * view3DState.scale, centerY - right.y * view3DState.scale);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

/**
 * 床面に四角形を描画するヘルパー関数
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 画面中心X
 * @param {number} centerY - 画面中心Y
 * @param {number} x1 - 開始X座標（メートル）
 * @param {number} y1 - 開始Y座標（メートル）
 * @param {number} z1 - 開始Z座標（メートル）
 * @param {number} x2 - 終了X座標（メートル）
 * @param {number} y2 - 終了Y座標（メートル）
 * @param {number} z2 - 終了Z座標（メートル）
 * @param {string} color - 塗りつぶし色
 */
function drawFloorRectangle(ctx, centerX, centerY, x1, y1, z1, x2, y2, z2, color) {
    // 四角形の4つの角を計算
    const corners = [
        worldToIso(x1, y1, z1),
        worldToIso(x2, y1, z1),
        worldToIso(x2, y2, z2),
        worldToIso(x1, y2, z2)
    ];

    const screenCorners = corners.map(v => ({
        x: centerX + v.x * view3DState.scale,
        y: centerY - v.y * view3DState.scale
    }));

    // 四角形を描画
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screenCorners[0].x, screenCorners[0].y);
    ctx.lineTo(screenCorners[1].x, screenCorners[1].y);
    ctx.lineTo(screenCorners[2].x, screenCorners[2].y);
    ctx.lineTo(screenCorners[3].x, screenCorners[3].y);
    ctx.closePath();
    ctx.fill();
}

/**
 * 3Dオブジェクトを描画
 *
 * @private
 */
function draw3DObject(ctx, rectangle, centerX, centerY) {
    const coords3D = get3DCoordinates(rectangle.id);
    if (!coords3D) return;

    // 選択されているかチェック
    const isSelected = view3DState.selectedObjectId === rectangle.id;

    // オブジェクトタイプに応じた色
    let color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // 選択されている場合は色を明るくする
    if (isSelected) {
        color = lightenColor(color, 40);
    }

    // オブジェクトタイプに応じた3Dモデルを描画
    draw3DModel(ctx, coords3D, color, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties);

    // 選択されている場合はハイライト枠を描画
    if (isSelected) {
        drawSelectionHighlight(ctx, coords3D, centerX, centerY);
    }

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
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 上面（天板）
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
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
        }
    });

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 左面
    if (drawLeft) {
        faces.push({
            cx: x - width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[4].x, screen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 右面
    if (drawRight) {
        faces.push({
            cx: x + width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.moveTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.lineTo(screen[2].x, screen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 前面
    if (drawFront) {
        faces.push({
            cx: x, cy: y - depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 5);
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
            }
        });
    }

    // 背面
    if (drawBack) {
        faces.push({
            cx: x, cy: y + depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(screen[2].x, screen[2].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, view3DState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    // 棚板を描画（常に最後に描画）
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
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
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
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, view3DState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * テーブルの3Dモデルを描画
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
 * @private
 */
function draw3DTable(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;
    const topThickness = height * 0.1;
    const legWidth = Math.min(width, depth) * 0.1;

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

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

    // 天板の底面
    faces.push({
        cx: x, cy: y, cz: topZ - topThickness/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の上面
    faces.push({
        cx: x, cy: y, cz: topZ + topThickness/2,
        draw: () => {
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
        }
    });

    // 天板の左面
    faces.push({
        cx: x - width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の右面
    faces.push({
        cx: x + width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の前面
    faces.push({
        cx: x, cy: y - depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の背面
    faces.push({
        cx: x, cy: y + depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

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

        const legCenterZ = (z - height/2 + topZ - topThickness/2) / 2;

        // 脚の底面
        faces.push({
            cx: pos.x, cy: pos.y, cz: z - height/2,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 30);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の左面
        faces.push({
            cx: pos.x - legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 20);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の右面
        faces.push({
            cx: pos.x + legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の前面
        faces.push({
            cx: pos.x, cy: pos.y - legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の背面
        faces.push({
            cx: pos.x, cy: pos.y + legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 25);
                ctx.beginPath();
                ctx.moveTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, view3DState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * 扉の3Dモデルを描画
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
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
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - doorDepth/2, cz: z,
        draw: () => {
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
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + doorDepth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, view3DState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

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
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
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
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
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
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, view3DState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * 選択ハイライトを描画
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 */
function drawSelectionHighlight(ctx, coords3D, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;

    // バウンディングボックスの頂点を計算
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
    ctx.strokeStyle = '#667eea';
    ctx.lineWidth = 3;
    ctx.setLineDash([5, 5]);

    // 下部の辺
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.closePath();
    ctx.stroke();

    // 上部の辺
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.stroke();

    // 垂直の辺
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(screen[3].x, screen[3].y);
    ctx.lineTo(screen[7].x, screen[7].y);
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
 * カメラの中心はマップの有効領域の重心
 *
 * @param {number} x - X座標（メートル）
 * @param {number} y - Y座標（メートル）
 * @param {number} z - Z座標（メートル）
 * @returns {Object} {x, y} - 等角投影座標
 *
 * @private
 */
function worldToIso(x, y, z) {
    // マップの重心を中心とするように座標をオフセット
    let offsetX = x;
    let offsetY = y;

    if (view3DState.mapBounds && mapState.image) {
        const resolution = mapState.metadata?.resolution || 0.05;
        // metadata.originは配列形式 [x, y, theta]
        const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
        const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;
        const imageHeight = mapState.image.height;

        // 有効領域の中心座標（実世界座標）
        const centerPixelX = view3DState.mapBounds.centerX;
        const centerPixelY = view3DState.mapBounds.centerY;

        const centerWorldX = originX + centerPixelX * resolution;
        const centerWorldY = originY + (imageHeight - centerPixelY) * resolution;

        // 座標を重心からの相対位置に変換
        offsetX = x - centerWorldX;
        offsetY = y - centerWorldY;
    }

    // 等角投影の変換行列
    // 回転を考慮
    const rad = view3DState.rotation * Math.PI / 180;
    const tiltRad = view3DState.tilt * Math.PI / 180;

    const rotX = offsetX * Math.cos(rad) - offsetY * Math.sin(rad);
    const rotY = offsetX * Math.sin(rad) + offsetY * Math.cos(rad);

    return {
        x: -rotX,  // X軸を反転（2Dマップとの整合性のため）
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

/**
 * 面の中心点の視点からの深度を計算（Painter's Algorithm用）
 *
 * 視点の位置は回転角度（rotation）と傾き角度（tilt）から決定されます。
 * 深度が大きいほど視点から遠い位置にあります。
 *
 * @param {number} cx - 面の中心X座標（ワールド座標）
 * @param {number} cy - 面の中心Y座標（ワールド座標）
 * @param {number} cz - 面の中心Z座標（ワールド座標）
 * @param {Object} state - 3Dビュー状態（rotation, tiltを含む）またはプレビュー状態
 * @returns {number} 視点からの深度
 *
 * @private
 */
function calculateFaceDepth(cx, cy, cz, state) {
    // 回転角度とチルト角度をラジアンに変換
    const rotRad = state.rotation * Math.PI / 180;
    const tiltRad = state.tilt * Math.PI / 180;

    // 視点の位置を計算（球面座標系）
    // 距離は十分大きな値（100メートル）とする
    const viewDistance = 100;
    const viewX = viewDistance * Math.cos(tiltRad) * Math.sin(rotRad);
    const viewY = viewDistance * Math.cos(tiltRad) * Math.cos(rotRad);
    const viewZ = viewDistance * Math.sin(tiltRad);

    // 視点から面の中心までの距離を計算
    const dx = cx - viewX;
    const dy = cy - viewY;
    const dz = cz - viewZ;

    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * 面の配列を深度順にソート（遠い面から近い面へ）
 *
 * Painter's Algorithmを実装するために、視点から遠い面を先に描画します。
 *
 * @param {Array<Object>} faces - 面の配列。各要素は {cx, cy, cz, draw: function} の形式
 * @param {Object} state - 3Dビュー状態（rotation, tiltを含む）またはプレビュー状態
 * @returns {Array<Object>} ソート済みの面の配列
 *
 * @private
 */
function sortFacesByDepth(faces, state) {
    return faces.sort((a, b) => {
        const depthA = calculateFaceDepth(a.cx, a.cy, a.cz, state);
        const depthB = calculateFaceDepth(b.cx, b.cy, b.cz, state);
        // 深度が大きい（遠い）方を先に描画
        return depthB - depthA;
    });
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
    view3DState.mouseDownX = event.clientX;
    view3DState.mouseDownY = event.clientY;
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

        // ViewCubeを更新
        updateViewCube(view3DState.rotation, view3DState.tilt);
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
    // クリック判定（マウスダウンからの移動量が小さい場合）
    const deltaX = Math.abs(event.clientX - view3DState.mouseDownX);
    const deltaY = Math.abs(event.clientY - view3DState.mouseDownY);
    const isClick = deltaX < 5 && deltaY < 5;

    if (isClick) {
        // クリックされた位置から最も近いオブジェクトを選択
        handle3DObjectClick(event);
    }

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
// オブジェクト選択
// ================

/**
 * 3Dオブジェクトをクリックしたときの処理
 *
 * @private
 * @param {MouseEvent} event - マウスイベント
 */
function handle3DObjectClick(event) {
    const rect = view3DState.canvas.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const clickY = event.clientY - rect.top;

    console.log('3D Click at:', clickX, clickY);

    // すべての四角形を取得
    const rectangles = getAllRectangles();
    if (!rectangles || rectangles.length === 0) {
        console.log('No rectangles found');
        deselect3DObject();
        return;
    }

    console.log('Found rectangles:', rectangles.length);

    // クリック位置に最も近いオブジェクトを探す
    let closestObject = null;
    let closestDistance = Infinity;

    const centerX = view3DState.canvas.width / 2 + view3DState.offsetX;
    const centerY = view3DState.canvas.height / 2 + view3DState.offsetY;

    rectangles.forEach(rect => {
        const coords = get3DCoordinates(rect.id);
        if (!coords) return;

        // オブジェクトの中心位置を3D投影
        const objCenterX = coords.x;
        const objCenterY = coords.y;
        const objCenterZ = coords.z;

        const isoPos = worldToIso(objCenterX, objCenterY, objCenterZ);
        const screenX = centerX + isoPos.x * view3DState.scale;
        const screenY = centerY - isoPos.y * view3DState.scale;

        // クリック位置との距離を計算
        const distance = Math.sqrt(
            Math.pow(screenX - clickX, 2) +
            Math.pow(screenY - clickY, 2)
        );

        console.log(`Object ${rect.id}: distance=${distance.toFixed(2)}, screen=(${screenX.toFixed(1)}, ${screenY.toFixed(1)})`);

        if (distance < closestDistance) {
            closestDistance = distance;
            closestObject = rect;
        }
    });

    // 閾値内にあるオブジェクトのみ選択
    const threshold = 150; // ピクセル（閾値を増やした）
    console.log(`Closest object: ${closestObject?.id}, distance=${closestDistance.toFixed(2)}, threshold=${threshold}`);

    if (closestObject && closestDistance < threshold) {
        console.log('Selecting object:', closestObject.id);
        select3DObject(closestObject.id);
    } else {
        console.log('Deselecting (no object close enough)');
        deselect3DObject();
    }
}

/**
 * 3Dオブジェクトを選択
 *
 * @export
 * @param {string} objectId - オブジェクトID
 */
export function select3DObject(objectId) {
    view3DState.selectedObjectId = objectId;

    // 選択情報を表示
    showSelectedObjectInfo(objectId);

    // 再描画（選択ハイライトを表示）
    render3DScene();
}

/**
 * 3Dオブジェクトの選択を解除
 *
 * @export
 */
export function deselect3DObject() {
    view3DState.selectedObjectId = null;

    // 選択情報パネルを非表示
    const infoPanel = document.getElementById('view3DSelectedInfo');
    if (infoPanel) {
        infoPanel.style.display = 'none';
    }

    // 再描画（選択ハイライトを消去）
    render3DScene();
}

/**
 * 選択されたオブジェクトの情報を表示
 *
 * @private
 * @param {string} objectId - オブジェクトID
 */
function showSelectedObjectInfo(objectId) {
    const rectangle = getRectangleById(objectId);
    if (!rectangle) return;

    const infoPanel = document.getElementById('view3DSelectedInfo');
    if (!infoPanel) return;

    // オブジェクト情報を設定
    const objectTypeLabels = {
        'none': 'なし',
        'shelf': '棚',
        'box': '箱',
        'table': 'テーブル',
        'door': '扉',
        'wall': '壁'
    };

    const resolution = mapState.metadata?.resolution || 0.05;
    const widthMeters = (rectangle.width * resolution).toFixed(2);
    const heightMeters = (rectangle.height * resolution).toFixed(2);

    document.getElementById('view3DInfoId').textContent = objectId;
    document.getElementById('view3DInfoType').textContent = objectTypeLabels[rectangle.objectType] || 'なし';
    document.getElementById('view3DInfoSize').textContent = `${widthMeters}m × ${heightMeters}m`;
    document.getElementById('view3DInfoHeight').textContent = `${(rectangle.heightMeters || 0.5).toFixed(2)}m`;

    // パネルを表示
    infoPanel.style.display = 'block';
}

/**
 * 2Dマップタブに切り替えて選択オブジェクトを表示
 *
 * @export
 */
export function goto2DMap() {
    if (!view3DState.selectedObjectId) return;

    // 2Dマップタブに切り替え
    if (window.switchMapSubTab && typeof window.switchMapSubTab === 'function') {
        window.switchMapSubTab('map2D');
    }

    // 2Dマップでオブジェクトを選択
    if (window.selectRectangle && typeof window.selectRectangle === 'function') {
        window.selectRectangle(view3DState.selectedObjectId);
    }
}

/**
 * オブジェクトカタログタブに切り替えて選択オブジェクトを表示
 *
 * @export
 */
export function gotoObjectCatalog() {
    if (!view3DState.selectedObjectId) return;

    // オブジェクトカタログタブに切り替え
    if (window.switchTab && typeof window.switchTab === 'function') {
        window.switchTab('objectCatalog');
    }

    // カタログで該当オブジェクトを選択（実装されている場合）
    // TODO: オブジェクトカタログ側で選択機能が実装されたら連携
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
 * 3Dビューをリサイズする
 *
 * @returns {void}
 */
export function resize3DView() {
    resizeCanvas();
    render3DScene();
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

/**
 * 3Dビューをリセットする
 *
 * @returns {void}
 */
export function reset3DView() {
    view3DState.rotation = 45;
    view3DState.tilt = 30;
    view3DState.scale = 20;
    view3DState.offsetX = 0;
    view3DState.offsetY = 0;
    view3DState.mapBounds = null; // マップ境界をリセット

    // キャッシュをクリア
    if (drawMapTexture._imageData) {
        drawMapTexture._imageData = null;
    }

    render3DScene();
}

/**
 * マップ境界情報を取得する
 *
 * @returns {Object|null} マップ境界情報
 */
export function getMapBounds() {
    return view3DState.mapBounds;
}

/**
 * マップ境界情報を設定する
 *
 * @param {Object|null} bounds - マップ境界情報
 * @returns {void}
 */
export function setMapBounds(bounds) {
    view3DState.mapBounds = bounds;
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
    scale: 30,
    minScale: 10,
    maxScale: 100,
    currentRectangleId: null  // 現在表示中のrectangleId
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

    // ホイールイベントでズーム（スケール変更）
    canvas.addEventListener('wheel', handlePreviewWheel, { passive: false });

    console.log('プロパティプレビューを初期化しました');
}

/**
 * プレビュー用ホイールイベントハンドラ（ズーム）
 *
 * @private
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
 * @returns {Object} プレビュー状態オブジェクト
 */
export function getPreviewState() {
    return previewState;
}

/**
 * プレビュー用グリッド描画
 * 2Dマップと同じグリッド幅を使用
 * @private
 */
function drawPreviewGrid(ctx, centerX, centerY) {
    const gridSize = mapState.gridWidthInMeters || 1; // 2Dマップと同じグリッド幅を使用
    const gridCount = 5; // プレビュー範囲を拡大（3→5）

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
    const { x, y, z, width, depth, height, rotation } = coords3D;

    // 8つの頂点を計算（回転を考慮）
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
        return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz);
    });

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
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
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
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
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * オブジェクトタイプに応じたプレビューモデルを描画
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {string} objectType - オブジェクトタイプ
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 */
export function drawPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
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
    const { x, y, z, width, depth, height, rotation } = coords3D;
    const thickness = 0.05; // 壁の厚さ

    // 8つの頂点を計算（回転を考慮）
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
        return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz);
    });

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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 上面（天板）
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
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
        }
    });

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 左面
    if (drawLeft) {
        faces.push({
            cx: x - width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[4].x, screen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 右面
    if (drawRight) {
        faces.push({
            cx: x + width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.moveTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.lineTo(screen[2].x, screen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 前面（y-）
    if (drawFront) {
        faces.push({
            cx: x, cy: y - depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 5);
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
            }
        });
    }

    // 背面（y+）
    if (drawBack) {
        faces.push({
            cx: x, cy: y + depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(screen[2].x, screen[2].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

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
    const { x, y, z, width, depth, height, rotation } = coords3D;

    // 8つの頂点を計算（回転を考慮）
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
        return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz);
    });

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 上部は開いているので描画しない

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
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
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

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

    // 天板底面
    faces.push({
        cx: x, cy: y, cz: topZ - topThickness/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板上面
    faces.push({
        cx: x, cy: y, cz: topZ + topThickness/2,
        draw: () => {
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
        }
    });

    // 天板左面
    faces.push({
        cx: x - width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板右面
    faces.push({
        cx: x + width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板前面
    faces.push({
        cx: x, cy: y - depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板背面
    faces.push({
        cx: x, cy: y + depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

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

        const legCenterZ = (z - height/2 + topZ - topThickness/2) / 2;

        // 脚の底面
        faces.push({
            cx: pos.x, cy: pos.y, cz: z - height/2,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 30);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の左面
        faces.push({
            cx: pos.x - legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 20);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の右面
        faces.push({
            cx: pos.x + legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の前面
        faces.push({
            cx: pos.x, cy: pos.y - legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の背面
        faces.push({
            cx: pos.x, cy: pos.y + legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 25);
                ctx.beginPath();
                ctx.moveTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

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

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
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
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
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
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面（薄い面）
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面（大きい面）
    faces.push({
        cx: x, cy: y - doorDepth/2, cz: z,
        draw: () => {
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
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + doorDepth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

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
    const arrowStart = worldToPreviewIso(x + rotatedStart.x, y + rotatedStart.y, z);
    const arrowEnd = worldToPreviewIso(x + rotatedEnd.x, y + rotatedEnd.y, z);

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
 * ローカル座標に回転を適用
 * @param {number} localX - ローカルX座標
 * @param {number} localY - ローカルY座標
 * @param {number} rotation - 回転角度（度）
 * @returns {Object} 回転後の座標 {x, y}
 * @private
 */
function applyRotation(localX, localY, rotation) {
    if (!rotation) return { x: localX, y: localY };

    // 2D回転行列を適用（Y軸反転を考慮）
    const rad = rotation * Math.PI / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    return {
        x: localX * cos - localY * sin,
        y: -(localX * sin + localY * cos)  // Y軸を反転（worldToPreviewIsoでのX軸反転と整合性を取る）
    };
}

/**
 * プレビュー用等角投影変換
 * @param {number} x - X座標
 * @param {number} y - Y座標
 * @param {number} z - Z座標
 * @returns {Object} 変換後の2D座標 {x, y}
 */
export function worldToPreviewIso(x, y, z) {
    const rad = previewState.rotation * Math.PI / 180;
    const tiltRad = previewState.tilt * Math.PI / 180;

    const rotX = x * Math.cos(rad) - y * Math.sin(rad);
    const rotY = x * Math.sin(rad) + y * Math.cos(rad);

    return {
        x: -rotX,  // X軸を反転（3Dマップと同じ座標系）
        y: z - rotY * Math.sin(tiltRad)
    };
}
