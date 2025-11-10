/**
 * @file threeDRenderer.js
 * @description 3Dレンダラー統合モジュール（後方互換性用）
 *
 * このファイルは後方互換性を維持するため、新しく分割されたモジュールから
 * すべての関数を再エクスポートします。
 *
 * 新しいモジュール構成:
 * - utils/threeDUtils.js: ユーティリティ関数
 * - modules/threeDModels.js: 3Dモデル描画ロジック
 * - modules/threeDPreview.js: プレビュー管理
 * - modules/threeDViewCore.js: メインビュー管理
 *
 * @deprecated 新規コードでは各モジュールを直接インポートすることを推奨
 */

// メインビュー管理関数（threeDViewCore.js）
export {
    initialize3DView,
    render3DScene,
    update3DObject,
    resize3DView,
    set3DViewRotation,
    reset3DView,
    select3DObject,
    deselect3DObject,
    goto2DMap,
    gotoObjectCatalog,
    getMapBounds,
    setMapBounds
} from './threeDViewCore.js';

// プレビュー管理関数（threeDPreview.js）
export {
    initializePropertyPreview,
    renderPropertyPreview,
    getPreviewState,
    drawPreviewGrid,
    drawPreviewFrontDirection,
    drawPreviewModel
} from './threeDPreview.js';

// ユーティリティ関数（threeDUtils.js）
export {
    worldToIso,
    worldToPreviewIso,
    lightenColor,
    darkenColor,
    sortFacesByDepth,
    applyRotation
} from '../utils/threeDUtils.js';

// 3Dモデル描画関数（threeDModels.js）
export {
    draw3DShelf,
    draw3DBox,
    draw3DTable,
    draw3DDoor,
    draw3DWall,
    drawBox,
    drawPreviewShelf,
    drawPreviewBox,
    drawPreviewTable,
    drawPreviewDoor,
    drawPreviewWall,
    drawPreviewBoxSimple
} from './threeDModels.js';
