/**
 * @file objectCatalogList.js
 * @description オブジェクトカタログ - リスト表示とフィルタ機能
 *
 * このモジュールは、マップ上のオブジェクト一覧の表示、フィルタリング、
 * および選択機能を提供します。
 *
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../state/mapState.js - グローバル状態管理
 *
 * @exports initializeObjectCatalog - カタログを初期化
 * @exports updateObjectCatalog - カタログを更新
 * @exports selectCatalogItem - カタログアイテムを選択
 * @exports getCatalogState - カタログの状態を取得
 */

import { getAllRectangles, getRectangleById } from '../modules/rectangleManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_LABELS, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';

// カタログの状態
const catalogState = {
    selectedRectangleId: null,
    previewCanvas: null,
    previewCtx: null,
    editingData: null,
    // プレビュー用の3D設定
    rotation: 45,
    tilt: 30,
    scale: 80,
    minScale: 20,
    maxScale: 200,
    // ドラッグ状態
    isDragging: false,
    lastMouseX: 0,
    lastMouseY: 0
};

/**
 * カタログの状態を取得する
 * 他のモジュールから状態にアクセスするための関数
 *
 * @returns {Object} カタログの状態オブジェクト
 */
export function getCatalogState() {
    return catalogState;
}

/**
 * オブジェクトカタログを初期化する
 * リストとフィルタ機能を設定します
 *
 * @returns {void}
 */
export function initializeObjectCatalog() {
    // カテゴリフィルタのイベントリスナーを設定
    const filterSelect = document.getElementById('catalogCategoryFilter');
    if (filterSelect) {
        filterSelect.addEventListener('change', updateObjectCatalog);
    }

    console.log('オブジェクトカタログリストを初期化しました');
}

/**
 * オブジェクトカタログを更新する
 * フィルタに基づいてオブジェクトリストを再描画します
 *
 * @returns {void}
 */
export function updateObjectCatalog() {
    const catalogList = document.getElementById('objectCatalogList');
    if (!catalogList) return;

    // すべての四角形を取得
    const rectangles = getAllRectangles();

    // カテゴリフィルタを取得
    const filterSelect = document.getElementById('catalogCategoryFilter');
    const filterValue = filterSelect ? filterSelect.value : 'all';

    // フィルタリング
    const filteredRectangles = rectangles.filter(rect => {
        if (filterValue === 'all') return true;
        return rect.objectType === filterValue;
    });

    // リストをクリア
    catalogList.innerHTML = '';

    // オブジェクトがない場合
    if (filteredRectangles.length === 0) {
        const emptyMessage = document.createElement('p');
        emptyMessage.className = 'catalog-empty-message';
        emptyMessage.textContent = filterValue === 'all'
            ? 'オブジェクトが登録されていません。'
            : `カテゴリ「${OBJECT_TYPE_LABELS[filterValue] || filterValue}」のオブジェクトはありません。`;
        catalogList.appendChild(emptyMessage);

        // 選択を解除
        selectCatalogItem(null);
        return;
    }

    // オブジェクトアイテムを作成
    filteredRectangles.forEach(rect => {
        const item = createCatalogItem(rect);
        catalogList.appendChild(item);
    });

    // 選択中のアイテムがある場合、再選択
    if (catalogState.selectedRectangleId) {
        const selectedItem = catalogList.querySelector(`[data-rectangle-id="${catalogState.selectedRectangleId}"]`);
        if (selectedItem) {
            selectedItem.classList.add('selected');
        }
    }
}

/**
 * カタログアイテムを作成する
 * リストに表示される個々のアイテム要素を生成します
 *
 * @private
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {HTMLElement} カタログアイテム要素
 */
function createCatalogItem(rectangle) {
    const item = document.createElement('div');
    item.className = 'catalog-object-item';
    item.setAttribute('data-rectangle-id', rectangle.id);

    // 選択状態を反映
    if (catalogState.selectedRectangleId === rectangle.id) {
        item.classList.add('selected');
    }

    // ヘッダー
    const header = document.createElement('div');
    header.className = 'catalog-object-header';

    const id = document.createElement('span');
    id.className = 'catalog-object-id';

    // commonPropertiesにnameがある場合はそれを表示（子レイヤー名）
    let displayName = rectangle.id;
    if (rectangle.commonProperties && rectangle.commonProperties.name && rectangle.commonProperties.name.trim()) {
        displayName = rectangle.commonProperties.name;
    }
    id.textContent = displayName;

    const type = document.createElement('span');
    type.className = 'catalog-object-type';
    type.textContent = OBJECT_TYPE_LABELS[rectangle.objectType] || '未設定';
    type.style.backgroundColor = OBJECT_TYPE_COLORS[rectangle.objectType] || '#94a3b8';

    header.appendChild(id);
    header.appendChild(type);

    // 詳細情報
    const details = document.createElement('div');
    details.className = 'catalog-object-details';

    // サイズ情報
    const sizeInfo = document.createElement('div');
    sizeInfo.innerHTML = `<strong>サイズ:</strong> ${rectangle.width.toFixed(0)}px × ${rectangle.height.toFixed(0)}px`;
    details.appendChild(sizeInfo);

    // 高さ情報
    if (rectangle.heightMeters !== undefined && rectangle.heightMeters > 0) {
        const heightInfo = document.createElement('div');
        heightInfo.innerHTML = `<strong>高さ:</strong> ${rectangle.heightMeters.toFixed(2)}m`;
        details.appendChild(heightInfo);
    }

    item.appendChild(header);
    item.appendChild(details);

    // クリックイベント - カタログ内で選択して詳細表示
    item.addEventListener('click', (e) => {
        e.stopPropagation();
        selectCatalogItem(rectangle.id);
    });

    return item;
}

/**
 * カタログ内でアイテムを選択する
 * 選択状態を更新し、プレビューと詳細パネルを表示します
 *
 * @param {string|null} rectangleId - 選択する四角形のID、nullの場合は選択解除
 * @returns {void}
 */
export function selectCatalogItem(rectangleId) {
    // 前回の選択を解除
    const catalogList = document.getElementById('objectCatalogList');
    if (catalogList) {
        catalogList.querySelectorAll('.catalog-object-item').forEach(item => {
            item.classList.remove('selected');
        });
    }

    catalogState.selectedRectangleId = rectangleId;

    if (rectangleId === null) {
        // 選択解除 - プレビューと詳細パネルをクリア
        // 他のモジュールにイベントを通知
        window.dispatchEvent(new CustomEvent('catalogItemDeselected'));
        return;
    }

    // 新しいアイテムを選択
    if (catalogList) {
        const selectedItem = catalogList.querySelector(`[data-rectangle-id="${rectangleId}"]`);
        if (selectedItem) {
            selectedItem.classList.add('selected');
        }
    }

    // 他のモジュールにイベントを通知
    window.dispatchEvent(new CustomEvent('catalogItemSelected', {
        detail: { rectangleId }
    }));
}
