/**
 * @file main.js
 * @description アプリケーションのエントリーポイント
 * すべてのモジュールをインポートし、初期化処理を実行します
 * @requires ./state/mapState.js - マップの状態管理
 * @requires ./ui/tabs.js - タブ切り替え機能
 * @requires ./ui/controls.js - マップコントロール機能
 * @requires ./ui/events.js - イベントリスナー管理
 */

// =====================================
// モジュールのインポート
// =====================================

import { mapState } from './state/mapState.js';
import { switchTab } from './ui/tabs.js';
import { loadImageFile, loadYAMLFile, clearMap, drawMap } from './ui/controls.js';
import { setupEventListeners } from './ui/events.js';

// =====================================
// グローバルスコープへの公開
// （HTML内のonclick属性との互換性のため）
// =====================================

// タブ切り替え
window.switchTab = switchTab;

// コントロール関数
window.loadImageFile = loadImageFile;
window.loadYAMLFile = loadYAMLFile;
window.clearMap = clearMap;
window.drawMap = drawMap;

// =====================================
// 初期化処理
// =====================================

/**
 * アプリケーションを初期化する
 */
function initializeApp() {
    console.log('Initializing PlanFromMap application...');

    // イベントリスナーをセットアップ
    setupEventListeners();

    // オーバーレイコントロールを更新
    if (window.updateOverlayControls && typeof window.updateOverlayControls === 'function') {
        window.updateOverlayControls();
    }

    console.log('Application initialized successfully');
}

/**
 * ページ読み込み完了時の処理
 */
function onPageLoad() {
    console.log('Page loaded, fetching initial data...');

    // 統計情報を読み込み
    if (window.loadStats && typeof window.loadStats === 'function') {
        window.loadStats();
    }

    // 操作カタログを読み込み
    if (window.loadOperations && typeof window.loadOperations === 'function') {
        window.loadOperations();
    }
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

console.log('main.js loaded - PlanFromMap v1.0');
console.log('Available functions:', {
    switchTab: typeof switchTab,
    loadImageFile: typeof loadImageFile,
    loadYAMLFile: typeof loadYAMLFile,
    clearMap: typeof clearMap,
    drawMap: typeof drawMap,
    setupEventListeners: typeof setupEventListeners
});
