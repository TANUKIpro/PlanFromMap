/**
 * @file objectCatalogEditor.js
 * @description オブジェクトカタログ - 詳細編集パネルとフォーム機能
 *
 * このモジュールは、オブジェクトの詳細編集パネル、フォーム生成、
 * データの保存とバリデーション機能を提供します。
 *
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../modules/objectPropertyManager.js - オブジェクトプロパティ管理
 * @requires ./objectCatalogList.js - カタログリスト（状態管理）
 *
 * @exports initializeObjectCatalogEditor - エディタを初期化
 * @exports showCatalogDetail - 詳細編集パネルを表示
 * @exports closeCatalogDetail - 詳細編集パネルを閉じる
 * @exports cancelCatalogEdit - 編集をキャンセル
 * @exports saveCatalogEdit - 編集を保存
 * @exports switchCatalogTab - タブを切り替え
 */

import { getRectangleById, updateRectangle } from '../modules/rectangleManager.js';
import { getCommonPropertySchema, validateCommonProperties } from '../models/objectTypes.js';
import { getCatalogState } from './objectCatalogList.js';
import { updateObjectCatalog, selectCatalogItem } from './objectCatalogList.js';

/**
 * オブジェクトカタログエディタを初期化する
 * グローバル関数をwindowオブジェクトにエクスポートします
 *
 * @returns {void}
 */
export function initializeObjectCatalogEditor() {
    // グローバル関数としてエクスポート（HTMLから呼び出されるため）
    window.closeCatalogDetail = closeCatalogDetail;
    window.cancelCatalogEdit = cancelCatalogEdit;
    window.saveCatalogEdit = saveCatalogEdit;
    window.switchCatalogTab = switchCatalogTab;

    console.log('オブジェクトカタログエディタを初期化しました');
}

/**
 * カタログ詳細編集のタブを切り替える
 *
 * @param {string} tabName - タブ名（'basic', 'robot', 'physical', 'navigation'）
 * @returns {void}
 */
export function switchCatalogTab(tabName) {
    // すべてのタブボタンを非アクティブに
    document.querySelectorAll('.catalog-tab-button').forEach(btn => {
        btn.classList.remove('active');
    });

    // すべてのタブコンテンツを非表示に
    document.querySelectorAll('.catalog-tab-content').forEach(content => {
        content.classList.remove('active');
    });

    // 選択されたタブボタンをアクティブに
    const targetButton = document.querySelector(`.catalog-tab-button[data-tab="${tabName}"]`);
    if (targetButton) {
        targetButton.classList.add('active');
    }

    // 選択されたタブコンテンツを表示
    const targetContent = document.getElementById(`catalogTab${tabName.charAt(0).toUpperCase() + tabName.slice(1)}`);
    if (targetContent) {
        targetContent.classList.add('active');
    }
}

/**
 * 詳細編集パネルを表示する
 * 選択されたオブジェクトの編集フォームを生成します
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {void}
 */
export function showCatalogDetail(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    const detailPanel = document.getElementById('catalogDetailPanel');
    const detailContent = document.getElementById('catalogDetailContent');
    if (!detailPanel || !detailContent) return;

    const catalogState = getCatalogState();

    // 編集中のデータとして現在の値をコピー
    catalogState.editingData = {
        rectangleId: rectangleId,
        commonProperties: JSON.parse(JSON.stringify(rectangle.commonProperties || {}))
    };

    // フォームを生成（タブ形式）
    detailContent.innerHTML = '';
    const schema = getCommonPropertySchema();

    // セクション名とタブ名のマッピング
    const sectionToTab = {
        '基本情報': 'basic',
        'ロボット操作': 'robot',
        '物理的特性': 'physical',
        'ナビゲーション': 'navigation'
    };

    schema.forEach(section => {
        const tabName = sectionToTab[section.section];
        const tabId = `catalogTab${tabName.charAt(0).toUpperCase() + tabName.slice(1)}`;

        // タブコンテンツを作成
        const tabContent = document.createElement('div');
        tabContent.className = 'catalog-tab-content';
        tabContent.id = tabId;

        // 最初のタブをアクティブに
        if (section.section === '基本情報') {
            tabContent.classList.add('active');
        }

        const sectionDiv = document.createElement('div');
        sectionDiv.className = 'catalog-form-section';

        section.fields.forEach(field => {
            const fieldDiv = document.createElement('div');
            fieldDiv.className = 'catalog-form-field';

            const label = document.createElement('label');
            label.className = 'catalog-form-label';
            label.textContent = field.label;
            fieldDiv.appendChild(label);

            const input = createFormInput(field, catalogState.editingData.commonProperties);
            fieldDiv.appendChild(input);

            sectionDiv.appendChild(fieldDiv);
        });

        tabContent.appendChild(sectionDiv);
        detailContent.appendChild(tabContent);
    });

    // パネルを表示
    detailPanel.style.display = 'flex';
}

/**
 * フォーム入力要素を作成する
 * フィールド定義に基づいて適切な入力要素を生成します
 *
 * @private
 * @param {Object} field - フィールド定義
 * @param {Object} data - データオブジェクト
 * @returns {HTMLElement} 入力要素
 */
function createFormInput(field, data) {
    const key = field.key;
    const value = data[key];

    switch (field.type) {
        case 'text':
            const textInput = document.createElement('input');
            textInput.type = 'text';
            textInput.className = 'catalog-form-input';
            textInput.value = value || '';
            textInput.maxLength = field.maxLength || 1000;
            textInput.addEventListener('input', (e) => {
                data[key] = e.target.value;
            });
            return textInput;

        case 'textarea':
            const textarea = document.createElement('textarea');
            textarea.className = 'catalog-form-textarea';
            textarea.value = value || '';
            textarea.rows = field.rows || 3;
            textarea.maxLength = field.maxLength || 5000;
            textarea.addEventListener('input', (e) => {
                data[key] = e.target.value;
            });
            return textarea;

        case 'number':
            const numberInput = document.createElement('input');
            numberInput.type = 'number';
            numberInput.className = 'catalog-form-input';
            numberInput.value = value !== undefined ? value : '';
            if (field.min !== undefined) numberInput.min = field.min;
            if (field.max !== undefined) numberInput.max = field.max;
            if (field.step !== undefined) numberInput.step = field.step;
            numberInput.addEventListener('input', (e) => {
                data[key] = parseFloat(e.target.value) || 0;
            });
            return numberInput;

        case 'checkbox':
            const checkboxWrapper = document.createElement('div');
            checkboxWrapper.className = 'catalog-form-checkbox-wrapper';
            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.className = 'catalog-form-checkbox';
            checkbox.checked = value || false;
            checkbox.addEventListener('change', (e) => {
                data[key] = e.target.checked;
            });
            const checkboxLabel = document.createElement('span');
            checkboxLabel.textContent = field.label;
            checkboxWrapper.appendChild(checkbox);
            checkboxWrapper.appendChild(checkboxLabel);
            return checkboxWrapper;

        case 'select':
            const select = document.createElement('select');
            select.className = 'catalog-form-select';
            field.options.forEach(option => {
                const optionElement = document.createElement('option');
                optionElement.value = option.value;
                optionElement.textContent = option.label;
                if (value === option.value) {
                    optionElement.selected = true;
                }
                select.appendChild(optionElement);
            });
            select.addEventListener('change', (e) => {
                data[key] = e.target.value;
            });
            return select;

        case 'color':
            const colorInput = document.createElement('input');
            colorInput.type = 'color';
            colorInput.className = 'catalog-form-input';
            colorInput.value = value || '#667eea';
            colorInput.addEventListener('input', (e) => {
                data[key] = e.target.value;
            });
            return colorInput;

        case 'tags':
            const tagsContainer = document.createElement('div');
            tagsContainer.className = 'catalog-form-tags';

            // 初期タグを表示
            const tags = value || [];
            const renderTags = () => {
                tagsContainer.innerHTML = '';
                tags.forEach((tag, index) => {
                    const tagElement = document.createElement('span');
                    tagElement.className = 'catalog-form-tag';
                    tagElement.textContent = tag;

                    const removeBtn = document.createElement('button');
                    removeBtn.className = 'catalog-form-tag-remove';
                    removeBtn.textContent = '×';
                    removeBtn.addEventListener('click', () => {
                        tags.splice(index, 1);
                        data[key] = tags;
                        renderTags();
                    });

                    tagElement.appendChild(removeBtn);
                    tagsContainer.appendChild(tagElement);
                });

                // 新しいタグの入力欄
                const tagInput = document.createElement('input');
                tagInput.type = 'text';
                tagInput.className = 'catalog-form-tag-input';
                tagInput.placeholder = 'タグを入力してEnter';
                tagInput.addEventListener('keydown', (e) => {
                    if (e.key === 'Enter' && tagInput.value.trim()) {
                        e.preventDefault();
                        tags.push(tagInput.value.trim());
                        data[key] = tags;
                        renderTags();
                    }
                });
                tagsContainer.appendChild(tagInput);
            };

            renderTags();
            data[key] = tags;
            return tagsContainer;

        default:
            const defaultInput = document.createElement('input');
            defaultInput.type = 'text';
            defaultInput.className = 'catalog-form-input';
            defaultInput.value = value || '';
            defaultInput.addEventListener('input', (e) => {
                data[key] = e.target.value;
            });
            return defaultInput;
    }
}

/**
 * 詳細編集パネルを閉じる
 *
 * @returns {void}
 */
export function closeCatalogDetail() {
    const detailPanel = document.getElementById('catalogDetailPanel');
    if (detailPanel) {
        detailPanel.style.display = 'none';
    }
    const catalogState = getCatalogState();
    catalogState.editingData = null;
}

/**
 * 編集をキャンセルする
 * 詳細パネルを閉じて選択を解除します
 *
 * @returns {void}
 */
export function cancelCatalogEdit() {
    closeCatalogDetail();
    selectCatalogItem(null);
}

/**
 * 編集を保存する
 * バリデーションを実行してデータを保存します
 *
 * @returns {void}
 */
export function saveCatalogEdit() {
    const catalogState = getCatalogState();
    if (!catalogState.editingData) return;

    const { rectangleId, commonProperties } = catalogState.editingData;

    // バリデーション
    const validation = validateCommonProperties(commonProperties);
    if (!validation.valid) {
        alert('入力エラー:\n' + validation.errors.join('\n'));
        return;
    }

    // 更新
    const updates = {
        commonProperties: commonProperties
    };

    updateRectangle(rectangleId, updates);

    // カタログを更新（表示名が変わる可能性があるため）
    updateObjectCatalog();

    // マップを再描画（色が変わる可能性があるため）
    if (window.drawMap && typeof window.drawMap === 'function') {
        window.drawMap();
    }

    // 3Dシーンを更新
    if (window.render3DScene && typeof window.render3DScene === 'function') {
        window.render3DScene();
    }

    alert('保存しました');

    // パネルを閉じる
    closeCatalogDetail();
}
