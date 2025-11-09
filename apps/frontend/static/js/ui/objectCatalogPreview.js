/**
 * @file objectCatalogPreview.js
 * @description オブジェクトカタログ - 3Dプレビュー機能
 *
 * このモジュールは、オブジェクトの3Dプレビュー表示、マウス操作による
 * 視点変更、ViewCube連携、リサイザー機能を提供します。
 * threeDRenderer.jsの描画関数を再利用して重複コードを削減しています。
 *
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - オブジェクトプロパティ管理
 * @requires ../modules/threeDRenderer.js - 3Dレンダリング
 * @requires ../state/mapState.js - グローバル状態管理
 * @requires ./viewCube.js - ViewCube
 * @requires ./objectCatalogList.js - カタログリスト（状態管理）
 * @requires ./objectCatalogEditor.js - カタログエディタ
 *
 * @exports initializeObjectCatalogPreview - プレビューを初期化
 * @exports updatePreview - プレビューを更新
 * @exports clearPreview - プレビューをクリア
 */

import { getRectangleById } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { getPreviewState, drawPreviewModel, drawPreviewFrontDirection } from '../modules/threeDRenderer.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';
import { mapState } from '../state/mapState.js';
import { initializeViewCube } from './viewCube.js';
import { getCatalogState } from './objectCatalogList.js';
import { showCatalogDetail } from './objectCatalogEditor.js';

/**
 * オブジェクトカタログプレビューを初期化する
 * キャンバス、イベントハンドラー、リサイザー、ViewCubeを設定します
 *
 * @returns {void}
 */
export function initializeObjectCatalogPreview() {
    const catalogState = getCatalogState();

    // プレビューキャンバスを初期化
    const previewCanvas = document.getElementById('catalogPreviewCanvas');
    if (previewCanvas) {
        catalogState.previewCanvas = previewCanvas;
        catalogState.previewCtx = previewCanvas.getContext('2d');

        // キャンバスのサイズを設定
        const container = previewCanvas.parentElement;
        if (container) {
            previewCanvas.width = container.clientWidth;
            previewCanvas.height = container.clientHeight;
        }

        // マウスイベントリスナーを設定
        previewCanvas.addEventListener('mousedown', handlePreviewMouseDown);
        previewCanvas.addEventListener('mousemove', handlePreviewMouseMove);
        previewCanvas.addEventListener('mouseup', handlePreviewMouseUp);
        previewCanvas.addEventListener('mouseleave', handlePreviewMouseUp);
        previewCanvas.addEventListener('wheel', handlePreviewWheel, { passive: false });
    }

    // リサイザーを初期化
    initializeCatalogResizer();

    // ViewCubeを初期化
    const viewCubeCanvas = document.getElementById('catalogPreviewViewCube');
    if (viewCubeCanvas) {
        initializeViewCube(viewCubeCanvas, handleCatalogViewCubeChange);
        console.log('カタログプレビュー用ViewCubeを初期化しました');
    }

    // カタログアイテム選択イベントをリスン
    window.addEventListener('catalogItemSelected', (e) => {
        const { rectangleId } = e.detail;
        updatePreview(rectangleId);
        showCatalogDetail(rectangleId);
    });

    // カタログアイテム選択解除イベントをリスン
    window.addEventListener('catalogItemDeselected', () => {
        clearPreview();
    });

    console.log('オブジェクトカタログプレビューを初期化しました');
}

/**
 * カタログ用ViewCubeからの視点変更を処理
 *
 * @private
 * @param {number} rotation - 回転角度
 * @param {number} tilt - 傾き角度
 * @returns {void}
 */
function handleCatalogViewCubeChange(rotation, tilt) {
    const catalogState = getCatalogState();
    catalogState.rotation = rotation;
    catalogState.tilt = tilt;

    // 現在選択中のオブジェクトを再描画
    if (catalogState.selectedRectangleId) {
        updatePreview(catalogState.selectedRectangleId);
    }
}

/**
 * プレビューキャンバスのマウスダウンハンドラ
 *
 * @private
 * @param {MouseEvent} e - マウスイベント
 * @returns {void}
 */
function handlePreviewMouseDown(e) {
    const catalogState = getCatalogState();
    if (!catalogState.selectedRectangleId) return;

    catalogState.isDragging = true;
    catalogState.lastMouseX = e.clientX;
    catalogState.lastMouseY = e.clientY;
    catalogState.previewCanvas.style.cursor = 'grabbing';
}

/**
 * プレビューキャンバスのマウスムーブハンドラ
 *
 * @private
 * @param {MouseEvent} e - マウスイベント
 * @returns {void}
 */
function handlePreviewMouseMove(e) {
    const catalogState = getCatalogState();
    if (!catalogState.isDragging || !catalogState.selectedRectangleId) {
        if (catalogState.selectedRectangleId) {
            catalogState.previewCanvas.style.cursor = 'grab';
        }
        return;
    }

    const deltaX = e.clientX - catalogState.lastMouseX;
    const deltaY = e.clientY - catalogState.lastMouseY;

    // 回転を更新（マウスの動きに合わせて回転方向を調整）
    // X軸反転に伴い、マウス操作の方向も反転して直感的な操作を維持
    catalogState.rotation += deltaX * 0.5;
    catalogState.tilt = Math.max(-90, Math.min(90, catalogState.tilt + deltaY * 0.5));

    catalogState.lastMouseX = e.clientX;
    catalogState.lastMouseY = e.clientY;

    // プレビューを再描画
    updatePreview(catalogState.selectedRectangleId);
}

/**
 * プレビューキャンバスのマウスアップハンドラ
 *
 * @private
 * @returns {void}
 */
function handlePreviewMouseUp() {
    const catalogState = getCatalogState();
    catalogState.isDragging = false;
    if (catalogState.selectedRectangleId) {
        catalogState.previewCanvas.style.cursor = 'grab';
    }
}

/**
 * プレビューキャンバスのホイールハンドラ（ズーム）
 *
 * @private
 * @param {WheelEvent} e - ホイールイベント
 * @returns {void}
 */
function handlePreviewWheel(e) {
    const catalogState = getCatalogState();
    if (!catalogState.selectedRectangleId) return;

    e.preventDefault();
    e.stopPropagation();

    // ホイールの方向に応じてスケールを変更
    const delta = e.deltaY > 0 ? -5 : 5;
    catalogState.scale = Math.max(
        catalogState.minScale,
        Math.min(catalogState.maxScale, catalogState.scale + delta)
    );

    // プレビューを再描画
    updatePreview(catalogState.selectedRectangleId);
}

/**
 * 3Dプレビューを更新する
 * threeDRenderer.jsの描画関数を使用して重複コードを削減
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {void}
 */
export function updatePreview(rectangleId) {
    const catalogState = getCatalogState();
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle || !catalogState.previewCanvas || !catalogState.previewCtx) {
        clearPreview();
        return;
    }

    const canvas = catalogState.previewCanvas;
    const ctx = catalogState.previewCtx;

    // キャンバスサイズを親要素に合わせて調整
    const container = canvas.parentElement;
    if (container) {
        const containerWidth = container.clientWidth;
        const containerHeight = container.clientHeight;

        if (canvas.width !== containerWidth || canvas.height !== containerHeight ||
            canvas.width === 0 || canvas.height === 0) {
            canvas.width = containerWidth;
            canvas.height = containerHeight;
        }
    }

    // キャンバスをクリア
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 空メッセージを非表示
    const emptyMessage = document.getElementById('catalogPreviewEmpty');
    if (emptyMessage) {
        emptyMessage.style.display = 'none';
    }

    // 背景を描画
    ctx.fillStyle = '#f7fafc';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // 中心点を設定（配置面を下にして、モデル全体が見えるようにする）
    const centerX = canvas.width / 2;
    const centerY = canvas.height * 0.65; // 配置面を画面の下半分に配置

    // 四角形の3D座標を取得
    const coords3D = get3DCoordinates(rectangleId);
    if (!coords3D) {
        // 3D座標が取得できない場合は簡易描画
        drawSimplePreview(ctx, rectangle, centerX, centerY, catalogState);
        return;
    }

    // オブジェクトタイプに応じた色
    const color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // threeDRenderer.jsのpreviewStateを一時的に設定
    const previewState = getPreviewState();
    const originalRotation = previewState.rotation;
    const originalTilt = previewState.tilt;
    const originalScale = previewState.scale;

    previewState.rotation = catalogState.rotation;
    previewState.tilt = catalogState.tilt;
    previewState.scale = catalogState.scale;

    // グリッドを描画
    drawCatalogPreviewGrid(ctx, centerX, centerY, catalogState);

    // プレビュー用に座標を調整（原点中心に配置）
    const previewCoords = {
        x: 0,  // 原点中心
        y: 0,  // 原点中心
        z: coords3D.height / 2,  // 高さの半分（底面からの中心）
        width: coords3D.width,
        depth: coords3D.depth,
        height: coords3D.height,
        rotation: 0,  // プレビューでは回転なし
        frontDirection: rectangle.frontDirection || 'top'
    };

    // threeDRenderer.jsの描画関数を使用（メタデータを反映）
    drawPreviewModel(ctx, previewCoords, color, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties);

    // 前面方向を矢印で表示
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawPreviewFrontDirection(ctx, previewCoords, rectangle.frontDirection, centerX, centerY);
    }

    // previewStateを元に戻す
    previewState.rotation = originalRotation;
    previewState.tilt = originalTilt;
    previewState.scale = originalScale;

    // カーソルスタイルを設定
    if (!catalogState.isDragging) {
        canvas.style.cursor = 'grab';
    }
}

/**
 * プレビューをクリアする
 *
 * @returns {void}
 */
export function clearPreview() {
    const catalogState = getCatalogState();
    if (catalogState.previewCtx && catalogState.previewCanvas) {
        catalogState.previewCtx.clearRect(0, 0, catalogState.previewCanvas.width, catalogState.previewCanvas.height);
        catalogState.previewCanvas.style.cursor = 'default';
    }

    const emptyMessage = document.getElementById('catalogPreviewEmpty');
    if (emptyMessage) {
        emptyMessage.style.display = 'block';
    }
}

// ========================================
// カタログリサイザー
// ========================================

/**
 * カタログリサイザーを初期化する
 * 左右パネルのサイズ調整機能を提供します
 *
 * @private
 * @returns {void}
 */
function initializeCatalogResizer() {
    const resizer = document.getElementById('catalogResizer');
    const leftPanel = document.getElementById('catalogLeftPanel');
    const rightPanel = document.getElementById('catalogRightPanel');

    if (!resizer || !leftPanel || !rightPanel) {
        console.warn('カタログリサイザーの初期化に失敗しました: 必要な要素が見つかりません');
        return;
    }

    let isResizing = false;
    let startX = 0;
    let startWidth = 0;

    const handleMouseDown = (e) => {
        isResizing = true;
        startX = e.clientX;
        startWidth = leftPanel.getBoundingClientRect().width;

        // リサイズ中のカーソルを設定
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';

        // イベントリスナーを追加
        document.addEventListener('mousemove', handleMouseMove);
        document.addEventListener('mouseup', handleMouseUp);

        e.preventDefault();
    };

    const handleMouseMove = (e) => {
        if (!isResizing) return;

        const deltaX = e.clientX - startX;
        const newWidth = startWidth + deltaX;

        // 最小・最大幅の制約
        const minWidth = 250;
        const maxWidth = 800;
        const constrainedWidth = Math.max(minWidth, Math.min(maxWidth, newWidth));

        // 左パネルの幅を更新
        leftPanel.style.flexBasis = `${constrainedWidth}px`;

        e.preventDefault();
    };

    const handleMouseUp = () => {
        if (!isResizing) return;

        isResizing = false;
        document.body.style.cursor = '';
        document.body.style.userSelect = '';

        // イベントリスナーを削除
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);

        const catalogState = getCatalogState();
        // プレビューキャンバスをリサイズ
        if (catalogState.selectedRectangleId) {
            // キャンバスサイズを再調整するために再描画
            setTimeout(() => {
                updatePreview(catalogState.selectedRectangleId);
            }, 50);
        }
    };

    // リサイザーにマウスダウンイベントを設定
    resizer.addEventListener('mousedown', handleMouseDown);
}

// ========================================
// カタログプレビュー用ヘルパー関数
// ========================================

/**
 * カタログ用のアイソメトリック投影変換
 * 3D座標(x, y, z) → 2D画面座標(x, y)
 *
 * @private
 * @param {number} x - X座標（メートル）
 * @param {number} y - Y座標（メートル）
 * @param {number} z - Z座標（メートル）
 * @param {Object} catalogState - カタログ状態
 * @returns {Object} 2D座標 {x, y}
 */
function worldToCatalogIso(x, y, z, catalogState) {
    const rad = catalogState.rotation * Math.PI / 180;
    const tiltRad = catalogState.tilt * Math.PI / 180;

    // 回転
    const rotX = x * Math.cos(rad) - y * Math.sin(rad);
    const rotY = x * Math.sin(rad) + y * Math.cos(rad);

    // アイソメトリック投影
    return {
        x: rotX,
        y: z - rotY * Math.sin(tiltRad)
    };
}

/**
 * カタログプレビュー用グリッド描画
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {Object} catalogState - カタログ状態
 * @returns {void}
 */
function drawCatalogPreviewGrid(ctx, centerX, centerY, catalogState) {
    const gridSize = mapState.gridWidthInMeters || 1; // 2Dマップと同じグリッド幅を使用
    const gridCount = 5; // プレビュー範囲

    ctx.save();
    ctx.strokeStyle = '#cbd5e0';
    ctx.lineWidth = 1;

    for (let i = -gridCount; i <= gridCount; i++) {
        // X方向
        const startX = worldToCatalogIso(i * gridSize, -gridCount * gridSize, 0, catalogState);
        const endX = worldToCatalogIso(i * gridSize, gridCount * gridSize, 0, catalogState);

        ctx.beginPath();
        ctx.moveTo(centerX + startX.x * catalogState.scale, centerY - startX.y * catalogState.scale);
        ctx.lineTo(centerX + endX.x * catalogState.scale, centerY - endX.y * catalogState.scale);
        ctx.stroke();

        // Y方向
        const startY = worldToCatalogIso(-gridCount * gridSize, i * gridSize, 0, catalogState);
        const endY = worldToCatalogIso(gridCount * gridSize, i * gridSize, 0, catalogState);

        ctx.beginPath();
        ctx.moveTo(centerX + startY.x * catalogState.scale, centerY - startY.y * catalogState.scale);
        ctx.lineTo(centerX + endY.x * catalogState.scale, centerY - endY.y * catalogState.scale);
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * 簡易プレビュー描画（フォールバック用）
 * 3D座標が取得できない場合の簡易的な表示
 *
 * @private
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {Object} catalogState - カタログ状態
 * @returns {void}
 */
function drawSimplePreview(ctx, rectangle, centerX, centerY, catalogState) {
    // 簡易的な3D描画（アイソメトリック投影）
    const maxDim = Math.max(rectangle.width, rectangle.height, (rectangle.heightMeters || 0.5) * 100);
    const scale = Math.min(catalogState.previewCanvas.width, catalogState.previewCanvas.height) * 0.6 / maxDim;

    const w = rectangle.width * scale;
    const h = rectangle.height * scale;
    const height3D = (rectangle.heightMeters || 0.5) * 100 * scale;

    // アイソメトリック変換
    const iso = (x, y, z) => {
        const isoX = centerX + (x - y) * Math.cos(Math.PI / 6);
        const isoY = centerY + (x + y) * Math.sin(Math.PI / 6) - z;
        return { x: isoX, y: isoY };
    };

    // 色を取得
    const color = rectangle.color || OBJECT_TYPE_COLORS[rectangle.objectType] || '#667eea';

    ctx.save();
    ctx.strokeStyle = '#2d3748';

    // 底面
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.7;
    ctx.beginPath();
    const p1 = iso(-w/2, -h/2, 0);
    const p2 = iso(w/2, -h/2, 0);
    const p3 = iso(w/2, h/2, 0);
    const p4 = iso(-w/2, h/2, 0);
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.lineTo(p3.x, p3.y);
    ctx.lineTo(p4.x, p4.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    const p5 = iso(-w/2, -h/2, height3D);
    const p6 = iso(w/2, -h/2, height3D);
    const p7 = iso(w/2, h/2, height3D);
    const p8 = iso(-w/2, h/2, height3D);
    ctx.moveTo(p5.x, p5.y);
    ctx.lineTo(p6.x, p6.y);
    ctx.lineTo(p7.x, p7.y);
    ctx.lineTo(p8.x, p8.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 右側面
    ctx.fillStyle = darkenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(p2.x, p2.y);
    ctx.lineTo(p3.x, p3.y);
    ctx.lineTo(p7.x, p7.y);
    ctx.lineTo(p6.x, p6.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 左側面
    ctx.fillStyle = darkenColor(color, 40);
    ctx.beginPath();
    ctx.moveTo(p4.x, p4.y);
    ctx.lineTo(p1.x, p1.y);
    ctx.lineTo(p5.x, p5.y);
    ctx.lineTo(p8.x, p8.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.globalAlpha = 1.0;
    ctx.restore();
}

/**
 * 色を明るくする
 *
 * @private
 * @param {string} color - 16進数カラーコード
 * @param {number} amount - 明るくする量（0-255）
 * @returns {string} 調整後のカラーコード
 */
function lightenColor(color, amount) {
    const hex = color.replace('#', '');
    let r = parseInt(hex.substring(0, 2), 16);
    let g = parseInt(hex.substring(2, 4), 16);
    let b = parseInt(hex.substring(4, 6), 16);

    r = Math.min(255, r + amount);
    g = Math.min(255, g + amount);
    b = Math.min(255, b + amount);

    return `#${r.toString(16).padStart(2, '0')}${g.toString(16).padStart(2, '0')}${b.toString(16).padStart(2, '0')}`;
}

/**
 * 色を暗くする
 *
 * @private
 * @param {string} color - 16進数カラーコード
 * @param {number} amount - 暗くする量（0-255）
 * @returns {string} 調整後のカラーコード
 */
function darkenColor(color, amount) {
    const hex = color.replace('#', '');
    let r = parseInt(hex.substring(0, 2), 16);
    let g = parseInt(hex.substring(2, 4), 16);
    let b = parseInt(hex.substring(4, 6), 16);

    r = Math.max(0, r - amount);
    g = Math.max(0, g - amount);
    b = Math.max(0, b - amount);

    return `#${r.toString(16).padStart(2, '0')}${g.toString(16).padStart(2, '0')}${b.toString(16).padStart(2, '0')}`;
}
