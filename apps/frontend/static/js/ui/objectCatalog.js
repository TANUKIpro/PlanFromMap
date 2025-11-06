/**
 * @file objectCatalog.js
 * @description オブジェクトカタログ - マップ上のオブジェクト一覧表示
 *
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 *
 * @exports updateObjectCatalog - カタログを更新
 * @exports initializeObjectCatalog - カタログを初期化
 */

import { getAllRectangles, selectRectangle } from '../modules/rectangleManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_LABELS, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';
import { switchMapSubTab } from './tabs.js';

/**
 * オブジェクトカタログを初期化する
 *
 * @returns {void}
 */
export function initializeObjectCatalog() {
    // カテゴリフィルタのイベントリスナーを設定
    const filterSelect = document.getElementById('catalogCategoryFilter');
    if (filterSelect) {
        filterSelect.addEventListener('change', updateObjectCatalog);
    }

    console.log('オブジェクトカタログを初期化しました');
}

/**
 * オブジェクトカタログを更新する
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
        return;
    }

    // オブジェクトアイテムを作成
    filteredRectangles.forEach(rect => {
        const item = createCatalogItem(rect);
        catalogList.appendChild(item);
    });
}

/**
 * カタログアイテムを作成する
 *
 * @private
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {HTMLElement} カタログアイテム要素
 */
function createCatalogItem(rectangle) {
    const item = document.createElement('div');
    item.className = 'catalog-object-item';

    // ヘッダー
    const header = document.createElement('div');
    header.className = 'catalog-object-header';

    const id = document.createElement('span');
    id.className = 'catalog-object-id';
    id.textContent = rectangle.id;

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

    // 前面方向
    if (rectangle.frontDirection) {
        const directionInfo = document.createElement('div');
        const directionLabels = {
            top: '上',
            bottom: '下',
            left: '左',
            right: '右'
        };
        directionInfo.innerHTML = `<strong>前面方向:</strong> ${directionLabels[rectangle.frontDirection] || rectangle.frontDirection}`;
        details.appendChild(directionInfo);
    }

    // 回転角度
    if (rectangle.rotation !== 0) {
        const rotationInfo = document.createElement('div');
        rotationInfo.innerHTML = `<strong>回転:</strong> ${rectangle.rotation.toFixed(1)}°`;
        details.appendChild(rotationInfo);
    }

    item.appendChild(header);
    item.appendChild(details);

    // クリックイベント - オブジェクトを選択して2Dマップに移動
    item.addEventListener('click', () => {
        selectRectangle(rectangle.id);
        // 2Dマップタブに切り替え
        switchMapSubTab('map2D');
    });

    return item;
}
