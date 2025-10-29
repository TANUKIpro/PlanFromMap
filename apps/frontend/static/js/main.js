/**
 * @file main.js
 * @description アプリケーションのエントリーポイント
 * すべてのモジュールをインポートし、初期化処理を実行します
 * @requires ./state/mapState.js - マップの状態管理
 * @requires ./ui/tabs.js - タブ切り替え機能
 * @requires ./ui/controls.js - マップコントロール機能
 * @requires ./ui/events.js - イベントリスナー管理
 * @requires ./modules/* - 各種機能モジュール
 */

// =====================================
// モジュールのインポート
// =====================================

// 状態管理
import { mapState } from './state/mapState.js';

// UI
import { switchTab } from './ui/tabs.js';
import {
    loadImageFile,
    loadYAMLFile,
    clearMap,
    drawMap,
    showProfileManager,
    closeProfileManager,
    saveCurrentProfile,
    loadSelectedProfile,
    deleteSelectedProfile,
    exportSelectedProfile,
    toggleDrawingToolsExpand,
    toggleDrawingToolsVisibility,
    toggleLayersPanelExpand
} from './ui/controls.js';
import { setupEventListeners } from './ui/events.js';

// レイヤー管理
import {
    addNewLayer,
    initializeLayers,
    updateLayersPanel,
    redrawAllLayers
} from './modules/layerManager.js';

// ファイル読み込み
import {
    handleImageFileSelect,
    handleYAMLFileSelect
} from './modules/fileLoader.js';

// 描画ツール
import {
    selectTool,
    changeDrawingColor,
    changeBrushSize,
    toggleDrawingTools,
    toggleLayersPanel,
    updateCursor,
    getActiveDrawingLayer,
    performDrawing,
    redrawDrawingLayer,
    showMeasureDistance,
    performBucketFill,
    canvasToImagePixel,
    imagePixelToCanvas
} from './modules/drawingTools.js';

// メタデータ表示
import {
    toggleLayer,
    toggleOverlayOption,
    toggleMetadataMinimize,
    updateOverlayControls,
    displayMetadata
} from './modules/metadataDisplay.js';

// ビューポート制御
import {
    resetView,
    zoomIn,
    zoomOut
} from './modules/viewportControl.js';

// 履歴管理
import {
    undo,
    redo,
    saveToHistory
} from './modules/historyManager.js';

// API通信
import {
    loadStats,
    loadOperations,
    executeQuery
} from './modules/apiClient.js';

// =====================================
// グローバルスコープへの公開
// （HTML内のonclick/onchange属性との互換性のため）
// =====================================

// タブ切り替え
window.switchTab = switchTab;

// コントロール関数
window.loadImageFile = loadImageFile;
window.loadYAMLFile = loadYAMLFile;
window.clearMap = clearMap;
window.drawMap = drawMap;

// プロファイル管理
window.showProfileManager = showProfileManager;
window.closeProfileManager = closeProfileManager;
window.saveCurrentProfile = saveCurrentProfile;
window.loadSelectedProfile = loadSelectedProfile;
window.deleteSelectedProfile = deleteSelectedProfile;
window.exportSelectedProfile = exportSelectedProfile;

// ツール・パネルの展開/折りたたみ
window.toggleDrawingToolsExpand = toggleDrawingToolsExpand;
window.toggleDrawingToolsVisibility = toggleDrawingToolsVisibility;
window.toggleLayersPanelExpand = toggleLayersPanelExpand;

// ファイル読み込みハンドラー
window.handleImageFileSelect = handleImageFileSelect;
window.handleYAMLFileSelect = handleYAMLFileSelect;

// レイヤー管理
window.addNewLayer = addNewLayer;
window.initializeLayers = initializeLayers;
window.updateLayersPanel = updateLayersPanel;
window.redrawAllLayers = redrawAllLayers;

// 描画ツール
window.selectTool = selectTool;
window.changeDrawingColor = changeDrawingColor;
window.changeBrushSize = changeBrushSize;
window.toggleDrawingTools = toggleDrawingTools;
window.toggleLayersPanel = toggleLayersPanel;
window.updateCursor = updateCursor;
window.getActiveDrawingLayer = getActiveDrawingLayer;
window.performDrawing = performDrawing;
window.redrawDrawingLayer = redrawDrawingLayer;
window.showMeasureDistance = showMeasureDistance;
window.performBucketFill = performBucketFill;
window.canvasToImagePixel = canvasToImagePixel;
window.imagePixelToCanvas = imagePixelToCanvas;

// メタデータ表示
window.toggleLayer = toggleLayer;
window.toggleOverlayOption = toggleOverlayOption;
window.toggleMetadataMinimize = toggleMetadataMinimize;
window.updateOverlayControls = updateOverlayControls;
window.displayMetadata = displayMetadata;

// ビューポート制御
window.resetView = resetView;
window.zoomIn = zoomIn;
window.zoomOut = zoomOut;

// 履歴管理
window.undo = undo;
window.redo = redo;
window.saveToHistory = saveToHistory;

// API通信
window.loadStats = loadStats;
window.loadOperations = loadOperations;
window.executeQuery = executeQuery;

// =====================================
// 初期化処理
// =====================================

/**
 * アプリケーションを初期化する
 */
function initializeApp() {
    console.log('Initializing PlanFromMap application...');

    // レイヤーシステムを初期化
    initializeLayers();

    // イベントリスナーをセットアップ
    setupEventListeners();

    // オーバーレイコントロールを更新
    updateOverlayControls();

    console.log('Application initialized successfully');
}

/**
 * ページ読み込み完了時の処理
 */
function onPageLoad() {
    console.log('Page loaded, fetching initial data...');

    // 統計情報を読み込み
    loadStats();

    // 操作カタログを読み込み
    loadOperations();
}

// =====================================
// DOMContentLoadedイベント
// =====================================

document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM content loaded');

    // アプリケーションを初期化
    initializeApp();

    // ページ読み込み時のデータ取得は window.load イベントで実行
    window.addEventListener('load', onPageLoad);
});

// =====================================
// デバッグ情報の出力
// =====================================

console.log('main.js loaded - PlanFromMap v1.0 (Modular)');
console.log('Available functions on window object:', {
    switchTab: typeof window.switchTab,
    loadImageFile: typeof window.loadImageFile,
    loadYAMLFile: typeof window.loadYAMLFile,
    handleImageFileSelect: typeof window.handleImageFileSelect,
    handleYAMLFileSelect: typeof window.handleYAMLFileSelect,
    addNewLayer: typeof window.addNewLayer,
    selectTool: typeof window.selectTool,
    toggleLayer: typeof window.toggleLayer,
    undo: typeof window.undo,
    redo: typeof window.redo,
    zoomIn: typeof window.zoomIn,
    zoomOut: typeof window.zoomOut,
    executeQuery: typeof window.executeQuery
});
