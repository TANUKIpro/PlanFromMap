/**
 * @file objectCatalog.js
 * @description オブジェクトカタログ - マップ上のオブジェクト一覧表示と詳細編集
 *
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 *
 * @exports updateObjectCatalog - カタログを更新
 * @exports initializeObjectCatalog - カタログを初期化
 */

import { getAllRectangles, getRectangleById, updateRectangle } from '../modules/rectangleManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_LABELS, OBJECT_TYPE_COLORS, getCommonPropertySchema, validateCommonProperties } from '../models/objectTypes.js';

// カタログの状態
const catalogState = {
    selectedRectangleId: null,
    previewCanvas: null,
    previewCtx: null,
    editingData: null  // 編集中のデータのコピー
};

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
    }

    // グローバル関数としてエクスポート（HTMLから呼び出されるため）
    window.closeCatalogDetail = closeCatalogDetail;
    window.cancelCatalogEdit = cancelCatalogEdit;
    window.saveCatalogEdit = saveCatalogEdit;

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

    // commonPropertiesにnameがある場合はそれを表示
    let displayName = rectangle.id;
    if (rectangle.commonProperties && rectangle.commonProperties.name) {
        displayName = `${rectangle.commonProperties.name} (${rectangle.id})`;
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
 *
 * @param {string|null} rectangleId - 選択する四角形のID、nullの場合は選択解除
 * @returns {void}
 */
function selectCatalogItem(rectangleId) {
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
        clearPreview();
        closeCatalogDetail();
        return;
    }

    // 新しいアイテムを選択
    if (catalogList) {
        const selectedItem = catalogList.querySelector(`[data-rectangle-id="${rectangleId}"]`);
        if (selectedItem) {
            selectedItem.classList.add('selected');
        }
    }

    // 3Dプレビューを更新
    updatePreview(rectangleId);

    // 詳細編集パネルを表示
    showCatalogDetail(rectangleId);
}

/**
 * 3Dプレビューを更新する
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {void}
 */
function updatePreview(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle || !catalogState.previewCanvas || !catalogState.previewCtx) {
        clearPreview();
        return;
    }

    const canvas = catalogState.previewCanvas;
    const ctx = catalogState.previewCtx;

    // キャンバスをクリア
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 空メッセージを非表示
    const emptyMessage = document.getElementById('catalogPreviewEmpty');
    if (emptyMessage) {
        emptyMessage.style.display = 'none';
    }

    // 簡易的な3D描画（アイソメトリック投影）
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    // スケーリング（キャンバスに収まるように）
    const maxDim = Math.max(rectangle.width, rectangle.height, rectangle.heightMeters * 100);
    const scale = Math.min(canvas.width, canvas.height) * 0.6 / maxDim;

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
    const color = rectangle.color || '#667eea';

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
    ctx.fillStyle = adjustBrightness(color, 1.2);
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
    ctx.fillStyle = adjustBrightness(color, 0.8);
    ctx.beginPath();
    ctx.moveTo(p2.x, p2.y);
    ctx.lineTo(p3.x, p3.y);
    ctx.lineTo(p7.x, p7.y);
    ctx.lineTo(p6.x, p6.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 左側面
    ctx.fillStyle = adjustBrightness(color, 0.6);
    ctx.beginPath();
    ctx.moveTo(p4.x, p4.y);
    ctx.lineTo(p1.x, p1.y);
    ctx.lineTo(p5.x, p5.y);
    ctx.lineTo(p8.x, p8.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.globalAlpha = 1.0;
}

/**
 * プレビューをクリアする
 *
 * @returns {void}
 */
function clearPreview() {
    if (catalogState.previewCtx && catalogState.previewCanvas) {
        catalogState.previewCtx.clearRect(0, 0, catalogState.previewCanvas.width, catalogState.previewCanvas.height);
    }

    const emptyMessage = document.getElementById('catalogPreviewEmpty');
    if (emptyMessage) {
        emptyMessage.style.display = 'block';
    }
}

/**
 * 色の明度を調整する
 *
 * @param {string} color - 16進数カラーコード
 * @param {number} factor - 明度調整係数（1.0より大きいと明るく、小さいと暗く）
 * @returns {string} 調整後のカラーコード
 */
function adjustBrightness(color, factor) {
    // #RRGGBBを分解
    const hex = color.replace('#', '');
    let r = parseInt(hex.substring(0, 2), 16);
    let g = parseInt(hex.substring(2, 4), 16);
    let b = parseInt(hex.substring(4, 6), 16);

    // 調整
    r = Math.min(255, Math.floor(r * factor));
    g = Math.min(255, Math.floor(g * factor));
    b = Math.min(255, Math.floor(b * factor));

    // 16進数に戻す
    return `#${r.toString(16).padStart(2, '0')}${g.toString(16).padStart(2, '0')}${b.toString(16).padStart(2, '0')}`;
}

/**
 * 詳細編集パネルを表示する
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {void}
 */
function showCatalogDetail(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    const detailPanel = document.getElementById('catalogDetailPanel');
    const detailContent = document.getElementById('catalogDetailContent');
    if (!detailPanel || !detailContent) return;

    // 編集中のデータとして現在の値をコピー
    catalogState.editingData = {
        rectangleId: rectangleId,
        commonProperties: JSON.parse(JSON.stringify(rectangle.commonProperties || {}))
    };

    // フォームを生成
    detailContent.innerHTML = '';
    const schema = getCommonPropertySchema();

    schema.forEach(section => {
        const sectionDiv = document.createElement('div');
        sectionDiv.className = 'catalog-form-section';

        const sectionTitle = document.createElement('h4');
        sectionTitle.className = 'catalog-form-section-title';
        sectionTitle.textContent = section.section;
        sectionDiv.appendChild(sectionTitle);

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

        detailContent.appendChild(sectionDiv);
    });

    // パネルを表示
    detailPanel.style.display = 'flex';
}

/**
 * フォーム入力要素を作成する
 *
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
function closeCatalogDetail() {
    const detailPanel = document.getElementById('catalogDetailPanel');
    if (detailPanel) {
        detailPanel.style.display = 'none';
    }
    catalogState.editingData = null;
}

/**
 * 編集をキャンセルする
 *
 * @returns {void}
 */
function cancelCatalogEdit() {
    closeCatalogDetail();
    selectCatalogItem(null);
}

/**
 * 編集を保存する
 *
 * @returns {void}
 */
function saveCatalogEdit() {
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
