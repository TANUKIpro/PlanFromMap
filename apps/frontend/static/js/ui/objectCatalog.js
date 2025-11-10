/**
 * @file objectCatalog.js
 * @description オブジェクトカタログ - 統合エントリーポイント
 *
 * このファイルは後方互換性のために全ての機能を再エクスポートします。
 * 機能は以下のモジュールに分割されています：
 *
 * - objectCatalogList.js: リスト表示とフィルタ機能
 * - objectCatalogEditor.js: 詳細編集パネルとフォーム機能
 * - objectCatalogPreview.js: 3Dプレビュー機能
 *
 * @requires ./objectCatalogList.js - リスト機能
 * @requires ./objectCatalogEditor.js - 編集機能
 * @requires ./objectCatalogPreview.js - プレビュー機能
 *
 * @exports updateObjectCatalog - カタログを更新
 * @exports initializeObjectCatalog - カタログを初期化
 */

// 各モジュールから機能をインポート
import {
    initializeObjectCatalog as initList,
    updateObjectCatalog,
    selectCatalogItem,
    getCatalogState
} from './objectCatalogList.js';

import {
    initializeObjectCatalogEditor as initEditor,
    showCatalogDetail,
    closeCatalogDetail,
    cancelCatalogEdit,
    saveCatalogEdit,
    switchCatalogTab
} from './objectCatalogEditor.js';

import {
    initializeObjectCatalogPreview as initPreview,
    updatePreview,
    clearPreview
} from './objectCatalogPreview.js';

// 後方互換性のために主要な関数を再エクスポート
export { updateObjectCatalog, selectCatalogItem, getCatalogState };
export { showCatalogDetail, closeCatalogDetail, cancelCatalogEdit, saveCatalogEdit, switchCatalogTab };
export { updatePreview, clearPreview };

/**
 * オブジェクトカタログを初期化する
 * 全てのサブモジュールを初期化します
 *
 * @returns {void}
 */
export function initializeObjectCatalog() {
    // リスト機能を初期化
    initList();

    // エディタ機能を初期化
    initEditor();

    // プレビュー機能を初期化
    initPreview();

    console.log('オブジェクトカタログを初期化しました');
}
