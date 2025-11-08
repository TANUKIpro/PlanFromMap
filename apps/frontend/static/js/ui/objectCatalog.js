/**
 * @file objectCatalog.js
 * @description オブジェクトカタログ - マップ上のオブジェクト一覧表示と詳細編集
 *
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../modules/objectPropertyManager.js - オブジェクトプロパティ管理
 * @requires ../state/mapState.js - グローバル状態管理
 *
 * @exports updateObjectCatalog - カタログを更新
 * @exports initializeObjectCatalog - カタログを初期化
 */

import { getAllRectangles, getRectangleById, updateRectangle } from '../modules/rectangleManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_LABELS, OBJECT_TYPE_COLORS, getCommonPropertySchema, validateCommonProperties } from '../models/objectTypes.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { mapState } from '../state/mapState.js';
import { initializeViewCube } from '../ui/viewCube.js';
import { getPreviewState, drawPreviewModel, drawPreviewFrontDirection } from '../modules/threeDRendererThree.js';

// カタログの状態
const catalogState = {
    selectedRectangleId: null,
    previewCanvas: null,
    previewCtx: null,
    editingData: null,  // 編集中のデータのコピー
    // プレビュー用の3D設定
    rotation: 45,       // 回転角度（度）
    tilt: 30,           // 傾き角度（度）
    scale: 80,          // スケール
    minScale: 20,
    maxScale: 200,
    // ドラッグ状態
    isDragging: false,
    lastMouseX: 0,
    lastMouseY: 0
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

    // グローバル関数としてエクスポート（HTMLから呼び出されるため）
    window.closeCatalogDetail = closeCatalogDetail;
    window.cancelCatalogEdit = cancelCatalogEdit;
    window.saveCatalogEdit = saveCatalogEdit;
    window.switchCatalogTab = switchCatalogTab;

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
 * カタログ用ViewCubeからの視点変更を処理
 *
 * @private
 * @param {number} rotation - 回転角度
 * @param {number} tilt - 傾き角度
 */
function handleCatalogViewCubeChange(rotation, tilt) {
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
 */
function handlePreviewMouseDown(e) {
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
 */
function handlePreviewMouseMove(e) {
    if (!catalogState.isDragging || !catalogState.selectedRectangleId) {
        if (catalogState.selectedRectangleId) {
            catalogState.previewCanvas.style.cursor = 'grab';
        }
        return;
    }

    const deltaX = e.clientX - catalogState.lastMouseX;
    const deltaY = e.clientY - catalogState.lastMouseY;

    // 回転を更新
    catalogState.rotation += deltaX * 0.5;
    catalogState.tilt = Math.max(-90, Math.min(90, catalogState.tilt - deltaY * 0.5));

    catalogState.lastMouseX = e.clientX;
    catalogState.lastMouseY = e.clientY;

    // プレビューを再描画
    updatePreview(catalogState.selectedRectangleId);
}

/**
 * プレビューキャンバスのマウスアップハンドラ
 *
 * @private
 */
function handlePreviewMouseUp() {
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
 */
function handlePreviewWheel(e) {
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
        drawSimplePreview(ctx, rectangle, centerX, centerY);
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
    drawCatalogPreviewGrid(ctx, centerX, centerY);

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
function clearPreview() {
    if (catalogState.previewCtx && catalogState.previewCanvas) {
        catalogState.previewCtx.clearRect(0, 0, catalogState.previewCanvas.width, catalogState.previewCanvas.height);
        catalogState.previewCanvas.style.cursor = 'default';
    }

    const emptyMessage = document.getElementById('catalogPreviewEmpty');
    if (emptyMessage) {
        emptyMessage.style.display = 'block';
    }
}


/**
 * カタログ詳細編集のタブを切り替える
 *
 * @param {string} tabName - タブ名（'basic', 'robot', 'physical', 'navigation'）
 * @returns {void}
 */
function switchCatalogTab(tabName) {
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

// ========================================
// カタログリサイザー
// ========================================

/**
 * カタログリサイザーを初期化する
 *
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
 * @param {number} x - X座標（メートル）
 * @param {number} y - Y座標（メートル）
 * @param {number} z - Z座標（メートル）
 * @returns {Object} 2D座標 {x, y}
 */
function worldToCatalogIso(x, y, z) {
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
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 */
function drawCatalogPreviewGrid(ctx, centerX, centerY) {
    const gridSize = mapState.gridWidthInMeters || 1; // 2Dマップと同じグリッド幅を使用
    const gridCount = 5; // プレビュー範囲

    ctx.save();
    ctx.strokeStyle = '#cbd5e0';
    ctx.lineWidth = 1;

    for (let i = -gridCount; i <= gridCount; i++) {
        // X方向
        const startX = worldToCatalogIso(i * gridSize, -gridCount * gridSize, 0);
        const endX = worldToCatalogIso(i * gridSize, gridCount * gridSize, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startX.x * catalogState.scale, centerY - startX.y * catalogState.scale);
        ctx.lineTo(centerX + endX.x * catalogState.scale, centerY - endX.y * catalogState.scale);
        ctx.stroke();

        // Y方向
        const startY = worldToCatalogIso(-gridCount * gridSize, i * gridSize, 0);
        const endY = worldToCatalogIso(gridCount * gridSize, i * gridSize, 0);

        ctx.beginPath();
        ctx.moveTo(centerX + startY.x * catalogState.scale, centerY - startY.y * catalogState.scale);
        ctx.lineTo(centerX + endY.x * catalogState.scale, centerY - endY.y * catalogState.scale);
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * 簡易プレビュー描画（フォールバック用）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 */
function drawSimplePreview(ctx, rectangle, centerX, centerY) {
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
 * カタログプレビュー用3Dモデル描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {string} objectType - オブジェクトタイプ
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 */
function drawCatalogPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    // オブジェクトタイプに応じた描画
    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            drawCatalogPreviewShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.BOX:
            drawCatalogPreviewBox(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.TABLE:
            drawCatalogPreviewTable(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.DOOR:
            drawCatalogPreviewDoor(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.WALL:
            drawCatalogPreviewWall(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties);
            break;
        default:
            // デフォルトは標準ボックス
            drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY);
            break;
    }
}

/**
 * 標準ボックス描画
 */
function drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY) {
    const { x, y, z, width, depth, height } = coords3D;

    // 8つの頂点を計算
    const vertices = [
        worldToCatalogIso(x - width/2, y - depth/2, z - height/2),
        worldToCatalogIso(x + width/2, y - depth/2, z - height/2),
        worldToCatalogIso(x + width/2, y + depth/2, z - height/2),
        worldToCatalogIso(x - width/2, y + depth/2, z - height/2),
        worldToCatalogIso(x - width/2, y - depth/2, z + height/2),
        worldToCatalogIso(x + width/2, y - depth/2, z + height/2),
        worldToCatalogIso(x + width/2, y + depth/2, z + height/2),
        worldToCatalogIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * catalogState.scale,
        y: centerY - v.y * catalogState.scale
    }));

    ctx.save();
    ctx.strokeStyle = '#2d3748';
    ctx.lineWidth = 1;

    // 上面
    ctx.fillStyle = lightenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 右側面
    ctx.fillStyle = darkenColor(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 左側面
    ctx.fillStyle = darkenColor(color, 40);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.lineTo(screen[3].x, screen[3].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * 棚モデル描画（簡易版）
 */
function drawCatalogPreviewShelf(ctx, coords3D, color, centerX, centerY, frontDirection, properties) {
    // 標準ボックスとして描画（簡易版）
    drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * 箱モデル描画（簡易版）
 */
function drawCatalogPreviewBox(ctx, coords3D, color, centerX, centerY, frontDirection, properties) {
    // 標準ボックスとして描画（簡易版）
    drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * テーブルモデル描画（簡易版）
 */
function drawCatalogPreviewTable(ctx, coords3D, color, centerX, centerY, frontDirection, properties) {
    // 標準ボックスとして描画（簡易版）
    drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * 扉モデル描画（簡易版）
 */
function drawCatalogPreviewDoor(ctx, coords3D, color, centerX, centerY, frontDirection, properties) {
    // 標準ボックスとして描画（簡易版）
    drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * 壁モデル描画（簡易版）
 */
function drawCatalogPreviewWall(ctx, coords3D, color, centerX, centerY, frontDirection, properties) {
    // 標準ボックスとして描画（簡易版）
    drawCatalogPreviewStandardBox(ctx, coords3D, color, centerX, centerY);
}

/**
 * 前面方向矢印描画
 */
function drawCatalogPreviewFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    if (!frontDirection) return;

    const { x, y, z, width, depth, height } = coords3D;
    const arrowSize = Math.min(width, depth) * 0.3;

    ctx.save();
    ctx.strokeStyle = '#f56565';
    ctx.fillStyle = '#f56565';
    ctx.lineWidth = 2;

    // 前面方向に応じた矢印の位置と向きを計算
    let arrowBase, arrowTip;

    switch (frontDirection) {
        case 'top':
            arrowBase = worldToCatalogIso(x, y - depth/2, z);
            arrowTip = worldToCatalogIso(x, y - depth/2 - arrowSize, z);
            break;
        case 'bottom':
            arrowBase = worldToCatalogIso(x, y + depth/2, z);
            arrowTip = worldToCatalogIso(x, y + depth/2 + arrowSize, z);
            break;
        case 'left':
            arrowBase = worldToCatalogIso(x - width/2, y, z);
            arrowTip = worldToCatalogIso(x - width/2 - arrowSize, y, z);
            break;
        case 'right':
            arrowBase = worldToCatalogIso(x + width/2, y, z);
            arrowTip = worldToCatalogIso(x + width/2 + arrowSize, y, z);
            break;
        default:
            return;
    }

    const screenBase = {
        x: centerX + arrowBase.x * catalogState.scale,
        y: centerY - arrowBase.y * catalogState.scale
    };
    const screenTip = {
        x: centerX + arrowTip.x * catalogState.scale,
        y: centerY - arrowTip.y * catalogState.scale
    };

    // 矢印を描画
    ctx.beginPath();
    ctx.moveTo(screenBase.x, screenBase.y);
    ctx.lineTo(screenTip.x, screenTip.y);
    ctx.stroke();

    // 矢印の先端
    const angle = Math.atan2(screenTip.y - screenBase.y, screenTip.x - screenBase.x);
    const arrowHeadSize = 8;

    ctx.beginPath();
    ctx.moveTo(screenTip.x, screenTip.y);
    ctx.lineTo(
        screenTip.x - arrowHeadSize * Math.cos(angle - Math.PI / 6),
        screenTip.y - arrowHeadSize * Math.sin(angle - Math.PI / 6)
    );
    ctx.lineTo(
        screenTip.x - arrowHeadSize * Math.cos(angle + Math.PI / 6),
        screenTip.y - arrowHeadSize * Math.sin(angle + Math.PI / 6)
    );
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

/**
 * 色を明るくする
 *
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
