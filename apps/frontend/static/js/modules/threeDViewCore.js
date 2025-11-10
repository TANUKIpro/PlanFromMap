/**
 * @file threeDViewCore.js
 * @description メイン3Dビューの管理とイベント処理
 *
 * Canvas 2Dコンテキストを使用して、2Dマップから3D構造を生成・描画します。
 * 等角投影（Isometric Projection）を使用して、2Dで3Dっぽく見せます。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../ui/viewCube.js - ViewCube（視点切り替えコントロール）
 * @requires ../utils/imageProcessing.js - 画像処理ユーティリティ
 * @requires ../utils/threeDUtils.js - 3D描画ユーティリティ
 * @requires ./threeDModels.js - 3Dモデル描画
 *
 * @exports initialize3DView - 3Dビュー初期化
 * @exports render3DScene - 3Dシーン描画
 * @exports update3DObject - 特定オブジェクト更新
 * @exports resize3DView - 3Dビューのリサイズ
 * @exports set3DViewRotation - 3Dビューの回転角度設定
 * @exports reset3DView - 3Dビューのリセット
 * @exports select3DObject - 3Dオブジェクト選択
 * @exports deselect3DObject - 3Dオブジェクト選択解除
 * @exports goto2DMap - 2Dマップタブへ移動
 * @exports gotoObjectCatalog - オブジェクトカタログタブへ移動
 * @exports getMapBounds - マップ境界情報取得
 * @exports setMapBounds - マップ境界情報設定
 */

import { mapState } from '../state/mapState.js';
import { getAllRectangles, getRectangleById, getRectangleLayer } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';
import { initializeViewCube, updateViewCube } from '../ui/viewCube.js';
import { analyzeMapBounds } from '../utils/imageProcessing.js';
import { worldToIso, lightenColor, darkenColor } from '../utils/threeDUtils.js';
import {
    draw3DShelf,
    draw3DBox,
    draw3DTable,
    draw3DDoor,
    draw3DWall,
    drawBox
} from './threeDModels.js';

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

/**
 * マップ画像を床面に描画（テクスチャ）
 * 有効領域のみを描画し、カメラ中心はマップの重心
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
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

    // worldToIsoラッパー（view3DStateとmapStateを渡す）
    const isoTransform = (x, y, z) => worldToIso(x, y, z, view3DState, mapState);

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
                isoTransform(tileX, tileY, tileZ),
                isoTransform(tileX + tileSize, tileY, tileZ),
                isoTransform(tileX + tileSize, tileY + tileSize, tileZ),
                isoTransform(tileX, tileY + tileSize, tileZ)
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
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 */
function drawGrid(ctx, centerX, centerY) {
    const gridSize = mapState.gridWidthInMeters || 1; // 2Dマップと同じグリッド幅を使用

    // worldToIsoラッパー（view3DStateとmapStateを渡す）
    const isoTransform = (x, y, z) => worldToIso(x, y, z, view3DState, mapState);

    // マップの有効領域がない場合はデフォルトのグリッドを描画
    if (!view3DState.mapBounds || !mapState.image) {
        const defaultGridCount = 10;
        ctx.save();
        ctx.strokeStyle = '#cbd5e0';
        ctx.lineWidth = 1;

        for (let i = -defaultGridCount; i <= defaultGridCount; i++) {
            // X方向のグリッド線
            const startX = isoTransform(i * gridSize, -defaultGridCount * gridSize, 0);
            const endX = isoTransform(i * gridSize, defaultGridCount * gridSize, 0);

            ctx.beginPath();
            ctx.moveTo(centerX + startX.x * view3DState.scale, centerY - startX.y * view3DState.scale);
            ctx.lineTo(centerX + endX.x * view3DState.scale, centerY - endX.y * view3DState.scale);
            ctx.stroke();

            // Y方向のグリッド線
            const startY = isoTransform(-defaultGridCount * gridSize, i * gridSize, 0);
            const endY = isoTransform(defaultGridCount * gridSize, i * gridSize, 0);

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
        const startIso = isoTransform(x, gridStartY, 0);
        const endIso = isoTransform(x, gridEndY, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startIso.x * view3DState.scale, centerY - startIso.y * view3DState.scale);
        ctx.lineTo(centerX + endIso.x * view3DState.scale, centerY - endIso.y * view3DState.scale);
        ctx.stroke();
    }

    // Y方向のグリッド線（横線）
    for (let y = gridStartY; y <= gridEndY; y += gridSize) {
        const startIso = isoTransform(gridStartX, y, 0);
        const endIso = isoTransform(gridEndX, y, 0);

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
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 */
function draw3DOrigin(ctx, centerX, centerY) {
    if (!mapState.metadata || !mapState.metadata.origin) {
        return;
    }

    ctx.save();

    // worldToIsoラッパー（view3DStateとmapStateを渡す）
    const isoTransform = (x, y, z) => worldToIso(x, y, z, view3DState, mapState);

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
        crossColor, isoTransform);

    // 水平線（右）
    drawFloorRectangle(ctx, centerX, centerY,
        originX, originY - lineWidthMeters / 2, originZ,
        originX + crossSizeMeters, originY + lineWidthMeters / 2, originZ,
        crossColor, isoTransform);

    // 垂直線（上）
    drawFloorRectangle(ctx, centerX, centerY,
        originX - lineWidthMeters / 2, originY, originZ,
        originX + lineWidthMeters / 2, originY + crossSizeMeters, originZ,
        crossColor, isoTransform);

    // 垂直線（下）
    drawFloorRectangle(ctx, centerX, centerY,
        originX - lineWidthMeters / 2, originY - crossSizeMeters, originZ,
        originX + lineWidthMeters / 2, originY, originZ,
        crossColor, isoTransform);

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
        crossColor, isoTransform);
    drawFloorRectangle(ctx, centerX, centerY,
        arrowEndX + perpX, arrowEndY + perpY, originZ,
        arrowEndX - perpX, arrowEndY - perpY, originZ,
        crossColor, isoTransform);
    drawFloorRectangle(ctx, centerX, centerY,
        arrowEndX - perpX, arrowEndY - perpY, originZ,
        originX - perpX, originY - perpY, originZ,
        crossColor, isoTransform);
    drawFloorRectangle(ctx, centerX, centerY,
        originX - perpX, originY - perpY, originZ,
        originX + perpX, originY + perpY, originZ,
        crossColor, isoTransform);

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
    const tip = isoTransform(tipX, tipY, originZ);
    const left = isoTransform(leftX, leftY, originZ);
    const right = isoTransform(rightX, rightY, originZ);

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
 * @param {Function} isoTransform - 座標変換関数
 */
function drawFloorRectangle(ctx, centerX, centerY, x1, y1, z1, x2, y2, z2, color, isoTransform) {
    // 四角形の4つの角を計算
    const corners = [
        isoTransform(x1, y1, z1),
        isoTransform(x2, y1, z1),
        isoTransform(x2, y2, z2),
        isoTransform(x1, y2, z2)
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
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
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

    // worldToIsoラッパー（view3DStateとmapStateを渡す）
    const isoTransform = (x, y, z) => worldToIso(x, y, z, view3DState, mapState);

    // オブジェクトタイプに応じた3Dモデルを描画
    draw3DModel(ctx, coords3D, color, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties, isoTransform);

    // 選択されている場合はハイライト枠を描画
    if (isSelected) {
        drawSelectionHighlight(ctx, coords3D, centerX, centerY, isoTransform);
    }

    // 前面方向を矢印で表示
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawFrontDirection(ctx, coords3D, rectangle.frontDirection, centerX, centerY, isoTransform);
    }
}

/**
 * オブジェクトタイプに応じた3Dモデルを描画（メインビュー用）
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {string} objectType - オブジェクトタイプ
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 * @param {Function} isoTransform - 座標変換関数
 */
function draw3DModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties, isoTransform) {
    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            draw3DShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, isoTransform, view3DState);
            break;
        case OBJECT_TYPES.BOX:
            draw3DBox(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, isoTransform, view3DState);
            break;
        case OBJECT_TYPES.TABLE:
            draw3DTable(ctx, coords3D, color, centerX, centerY, isoTransform, view3DState);
            break;
        case OBJECT_TYPES.DOOR:
            draw3DDoor(ctx, coords3D, color, centerX, centerY, isoTransform, view3DState);
            break;
        case OBJECT_TYPES.WALL:
            draw3DWall(ctx, coords3D, color, centerX, centerY, isoTransform, view3DState);
            break;
        case OBJECT_TYPES.NONE:
        default:
            drawBox(ctx, coords3D, color, centerX, centerY, isoTransform, view3DState);
            break;
    }
}

/**
 * 選択ハイライトを描画
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {Function} isoTransform - 座標変換関数
 */
function drawSelectionHighlight(ctx, coords3D, centerX, centerY, isoTransform) {
    const { x, y, z, width, depth, height } = coords3D;

    // バウンディングボックスの頂点を計算
    const vertices = [
        isoTransform(x - width/2, y - depth/2, z - height/2),
        isoTransform(x + width/2, y - depth/2, z - height/2),
        isoTransform(x + width/2, y + depth/2, z - height/2),
        isoTransform(x - width/2, y + depth/2, z - height/2),
        isoTransform(x - width/2, y - depth/2, z + height/2),
        isoTransform(x + width/2, y - depth/2, z + height/2),
        isoTransform(x + width/2, y + depth/2, z + height/2),
        isoTransform(x - width/2, y + depth/2, z + height/2),
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
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} frontDirection - 前面方向
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} isoTransform - 座標変換関数
 */
function drawFrontDirection(ctx, coords3D, frontDirection, centerX, centerY, isoTransform) {
    const { x, y, z, width, depth, height } = coords3D;

    // 前面の中心点を計算
    let arrowStart, arrowEnd;

    switch (frontDirection) {
        case 'top':    // 上（Y負方向）
            arrowStart = isoTransform(x, y - depth/2, z);
            arrowEnd = isoTransform(x, y - depth/2 - 0.3, z);
            break;
        case 'right':  // 右（X正方向）
            arrowStart = isoTransform(x + width/2, y, z);
            arrowEnd = isoTransform(x + width/2 + 0.3, y, z);
            break;
        case 'bottom': // 下（Y正方向）
            arrowStart = isoTransform(x, y + depth/2, z);
            arrowEnd = isoTransform(x, y + depth/2 + 0.3, z);
            break;
        case 'left':   // 左（X負方向）
            arrowStart = isoTransform(x - width/2, y, z);
            arrowEnd = isoTransform(x - width/2 - 0.3, y, z);
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
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
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
// イベントハンドラ
// ================

/**
 * マウスダウンハンドラ
 *
 * @private
 * @param {MouseEvent} event - マウスイベント
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
 * @param {MouseEvent} event - マウスイベント
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
        // 回転（マウスの動きに合わせて回転方向を調整）
        // X軸反転に伴い、マウス操作の方向も反転して直感的な操作を維持
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
 * @param {MouseEvent} event - マウスイベント
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
 * @param {WheelEvent} event - ホイールイベント
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

    // worldToIsoラッパー（view3DStateとmapStateを渡す）
    const isoTransform = (x, y, z) => worldToIso(x, y, z, view3DState, mapState);

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

        const isoPos = isoTransform(objCenterX, objCenterY, objCenterZ);
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
