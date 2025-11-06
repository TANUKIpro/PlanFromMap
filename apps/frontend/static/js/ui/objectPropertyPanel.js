/**
 * @file objectPropertyPanel.js
 * @description オブジェクトプロパティパネルのUI制御
 *
 * 四角形が選択されたときに表示されるプロパティパネルを管理します。
 * オブジェクトタイプの選択、3D情報の入力、カテゴリ別詳細設定を行うUIを提供します。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ./toast.js - 通知表示
 *
 * @exports initializePropertyPanel - パネル初期化
 * @exports showPropertyPanel - パネル表示
 * @exports hidePropertyPanel - パネル非表示
 * @exports updatePropertyPanel - パネル内容更新
 * @exports refreshPropertyPanel - 現在選択中の四角形でパネルを更新
 */

import { mapState } from '../state/mapState.js';
import {
    setObjectType,
    setObjectProperty,
    setObjectProperties,
    getObjectProperties,
    setHeightMeters,
    setFrontDirection,
    resetObjectProperties
} from '../modules/objectPropertyManager.js';
import { getRectangleById, getSelectedRectangle } from '../modules/rectangleManager.js';
import {
    OBJECT_TYPES,
    OBJECT_TYPE_LABELS,
    getObjectTypeOptions,
    getFrontDirectionOptions,
    getPropertySchema
} from '../models/objectTypes.js';
import { showSuccess, showError, showWarning } from './toast.js';
import { initializePropertyPreview, renderPropertyPreview } from '../modules/threeDRenderer.js';

// ================
// 初期化
// ================

/**
 * プロパティパネルを初期化する
 * イベントリスナーを設定し、初期状態では非表示にする
 *
 * @returns {void}
 */
export function initializePropertyPanel() {
    const panel = document.getElementById('objectPropertyPanel');
    if (!panel) {
        console.error('initializePropertyPanel: プロパティパネルのDOM要素が見つかりません');
        return;
    }

    // 初期状態では非表示
    hidePropertyPanel();

    // イベントリスナーを設定
    setupEventListeners();

    // プレビューを初期化
    initializePropertyPreview();

    console.log('オブジェクトプロパティパネルを初期化しました');
}

/**
 * イベントリスナーをセットアップする
 *
 * @private
 */
function setupEventListeners() {
    // カテゴリ選択
    const categorySelect = document.getElementById('objectTypeSelect');
    if (categorySelect) {
        categorySelect.addEventListener('change', handleCategoryChange);
    }

    // 高さ入力
    const heightInput = document.getElementById('heightMetersInput');
    if (heightInput) {
        heightInput.addEventListener('input', handleHeightChange);
        heightInput.addEventListener('blur', handleHeightBlur);
    }

    // 前面方向選択
    const frontDirectionSelect = document.getElementById('frontDirectionSelect');
    if (frontDirectionSelect) {
        frontDirectionSelect.addEventListener('change', handleFrontDirectionChange);
    }

    // 適用ボタン
    const applyButton = document.getElementById('applyPropertiesButton');
    if (applyButton) {
        applyButton.addEventListener('click', handleApplyProperties);
    }

    // リセットボタン
    const resetButton = document.getElementById('resetPropertiesButton');
    if (resetButton) {
        resetButton.addEventListener('click', handleResetProperties);
    }
}

// ================
// パネル表示制御
// ================

/**
 * プロパティパネルを表示する
 *
 * @param {string} rectangleId - 表示する四角形のID
 * @returns {void}
 *
 * @example
 * showPropertyPanel('rect-0');
 */
export function showPropertyPanel(rectangleId) {
    const panel = document.getElementById('objectPropertyPanel');
    if (!panel) return;

    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`showPropertyPanel: 四角形が見つかりません: ${rectangleId}`);
        hidePropertyPanel();
        return;
    }

    // パネルを表示
    panel.style.display = 'block';
    panel.classList.add('visible');

    // パネルの内容を更新
    updatePropertyPanel(rectangleId);
}

/**
 * プロパティパネルを非表示にする
 *
 * @returns {void}
 */
export function hidePropertyPanel() {
    const panel = document.getElementById('objectPropertyPanel');
    if (!panel) return;

    panel.style.display = 'none';
    panel.classList.remove('visible');
}

/**
 * 現在選択中の四角形でパネルを更新する
 * 選択されていない場合は非表示にする
 *
 * @returns {void}
 */
export function refreshPropertyPanel() {
    const selectedRect = getSelectedRectangle();
    if (selectedRect) {
        showPropertyPanel(selectedRect.id);
    } else {
        hidePropertyPanel();
    }
}

// ================
// パネル内容更新
// ================

/**
 * プロパティパネルの内容を更新する
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {void}
 */
export function updatePropertyPanel(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    // 選択中のオブジェクトID表示
    updateSelectedObjectInfo(rectangle);

    // 基本情報を更新
    updateBasicInfo(rectangle);

    // カテゴリ別詳細設定を更新
    updateDetailedSettings(rectangle);

    // プレビューを更新
    renderPropertyPreview(rectangleId);
}

/**
 * 選択中のオブジェクト情報を更新
 *
 * @private
 */
function updateSelectedObjectInfo(rectangle) {
    const infoElement = document.getElementById('selectedObjectInfo');
    if (infoElement) {
        infoElement.textContent = `選択: ${rectangle.id}`;
    }
}

/**
 * 基本情報（カテゴリ、高さ、前面方向）を更新
 *
 * @private
 */
function updateBasicInfo(rectangle) {
    // カテゴリ選択
    const categorySelect = document.getElementById('objectTypeSelect');
    if (categorySelect) {
        categorySelect.value = rectangle.objectType || OBJECT_TYPES.NONE;
    }

    // 高さ
    const heightInput = document.getElementById('heightMetersInput');
    if (heightInput) {
        heightInput.value = (rectangle.heightMeters || 0.5).toFixed(2);
    }

    // 前面方向
    const frontDirectionSelect = document.getElementById('frontDirectionSelect');
    if (frontDirectionSelect) {
        frontDirectionSelect.value = rectangle.frontDirection || 'top';
    }
}

/**
 * カテゴリ別詳細設定を更新
 *
 * @private
 */
function updateDetailedSettings(rectangle) {
    const detailsContainer = document.getElementById('objectDetailsContainer');
    if (!detailsContainer) return;

    // コンテナをクリア
    detailsContainer.innerHTML = '';

    // オブジェクトタイプが「なし」の場合は何も表示しない
    if (!rectangle.objectType || rectangle.objectType === OBJECT_TYPES.NONE) {
        return;
    }

    // カテゴリ名を表示
    const categoryLabel = OBJECT_TYPE_LABELS[rectangle.objectType] || rectangle.objectType;
    const header = document.createElement('div');
    header.className = 'details-header';
    header.textContent = `── ${categoryLabel}の詳細 ──`;
    detailsContainer.appendChild(header);

    // プロパティスキーマを取得
    const schema = getPropertySchema(rectangle.objectType);
    if (!schema || schema.length === 0) return;

    // 各プロパティの入力フィールドを生成
    schema.forEach(field => {
        const fieldElement = createPropertyField(field, rectangle);
        if (fieldElement) {
            detailsContainer.appendChild(fieldElement);
        }
    });
}

/**
 * プロパティフィールドのDOM要素を作成
 *
 * @private
 * @param {Object} field - フィールド定義
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {HTMLElement} フィールド要素
 */
function createPropertyField(field, rectangle) {
    const container = document.createElement('div');
    container.className = 'property-field';

    const label = document.createElement('label');
    label.textContent = field.label;
    label.className = 'property-label';
    container.appendChild(label);

    const currentValue = rectangle.objectProperties?.[field.key];

    // フィールドタイプに応じて入力要素を作成
    switch (field.type) {
        case 'number':
            const numberInput = createNumberInput(field, currentValue, rectangle);
            container.appendChild(numberInput);
            break;

        case 'checkbox':
            const checkbox = createCheckbox(field, currentValue, rectangle);
            container.appendChild(checkbox);
            break;

        case 'select':
            const select = createSelect(field, currentValue, rectangle);
            container.appendChild(select);
            break;

        case 'radio':
            const radioGroup = createRadioGroup(field, currentValue, rectangle);
            container.appendChild(radioGroup);
            break;
    }

    // ヒントがあれば追加
    if (field.hint) {
        const hint = document.createElement('div');
        hint.className = 'property-hint';
        hint.textContent = field.hint;
        container.appendChild(hint);
    }

    return container;
}

/**
 * 数値入力フィールドを作成
 *
 * @private
 */
function createNumberInput(field, currentValue, rectangle) {
    const wrapper = document.createElement('div');
    wrapper.className = 'number-input-wrapper';

    const input = document.createElement('input');
    input.type = 'number';
    input.className = 'property-input property-number';
    input.value = currentValue !== undefined ? currentValue : field.min || 0;
    input.min = field.min;
    input.max = field.max;
    input.step = field.step || 1;
    input.dataset.propertyKey = field.key;

    input.addEventListener('change', (e) => {
        const value = parseFloat(e.target.value);
        setObjectProperty(rectangle.id, field.key, value);
        if (window.redrawRectangleLayer) {
            const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
            if (rectangleLayer) window.redrawRectangleLayer(rectangleLayer);
        }
        // プレビューを更新
        renderPropertyPreview(rectangle.id);
    });

    wrapper.appendChild(input);

    // スライダーがある場合
    if (field.hasSlider) {
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'property-slider';
        slider.value = input.value;
        slider.min = field.min;
        slider.max = field.max;
        slider.step = field.step || 1;

        slider.addEventListener('input', (e) => {
            input.value = e.target.value;
            const value = parseFloat(e.target.value);
            setObjectProperty(rectangle.id, field.key, value);
            if (window.redrawRectangleLayer) {
                const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
                if (rectangleLayer) window.redrawRectangleLayer(rectangleLayer);
            }
            // プレビューを更新
            renderPropertyPreview(rectangle.id);
        });

        input.addEventListener('input', (e) => {
            slider.value = e.target.value;
        });

        wrapper.appendChild(slider);

        // スライダーのラベル（最小・最大）
        const sliderLabels = document.createElement('div');
        sliderLabels.className = 'slider-labels';
        sliderLabels.innerHTML = `<span>[${field.min}]</span><span>[${field.max}]</span>`;
        wrapper.appendChild(sliderLabels);
    }

    return wrapper;
}

/**
 * チェックボックスを作成
 *
 * @private
 */
function createCheckbox(field, currentValue, rectangle) {
    const wrapper = document.createElement('div');
    wrapper.className = 'checkbox-wrapper';

    const input = document.createElement('input');
    input.type = 'checkbox';
    input.className = 'property-checkbox';
    input.checked = currentValue !== undefined ? currentValue : false;
    input.dataset.propertyKey = field.key;

    input.addEventListener('change', (e) => {
        setObjectProperty(rectangle.id, field.key, e.target.checked);
        if (window.redrawRectangleLayer) {
            const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
            if (rectangleLayer) window.redrawRectangleLayer(rectangleLayer);
        }
        // プレビューを更新
        renderPropertyPreview(rectangle.id);
    });

    wrapper.appendChild(input);
    return wrapper;
}

/**
 * セレクトボックスを作成
 *
 * @private
 */
function createSelect(field, currentValue, rectangle) {
    const select = document.createElement('select');
    select.className = 'property-select';
    select.dataset.propertyKey = field.key;

    field.options.forEach(option => {
        const opt = document.createElement('option');
        opt.value = option.value;
        opt.textContent = option.label;
        if (currentValue === option.value) {
            opt.selected = true;
        }
        select.appendChild(opt);
    });

    select.addEventListener('change', (e) => {
        setObjectProperty(rectangle.id, field.key, e.target.value);
        if (window.redrawRectangleLayer) {
            const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
            if (rectangleLayer) window.redrawRectangleLayer(rectangleLayer);
        }
        // プレビューを更新
        renderPropertyPreview(rectangle.id);
    });

    return select;
}

/**
 * ラジオボタングループを作成
 *
 * @private
 */
function createRadioGroup(field, currentValue, rectangle) {
    const wrapper = document.createElement('div');
    wrapper.className = 'radio-group';

    field.options.forEach(option => {
        const radioWrapper = document.createElement('div');
        radioWrapper.className = 'radio-option';

        const input = document.createElement('input');
        input.type = 'radio';
        input.name = `${rectangle.id}-${field.key}`;
        input.value = option.value;
        input.className = 'property-radio';
        input.dataset.propertyKey = field.key;
        if (currentValue === option.value) {
            input.checked = true;
        }

        input.addEventListener('change', (e) => {
            if (e.target.checked) {
                setObjectProperty(rectangle.id, field.key, e.target.value);
                if (window.redrawRectangleLayer) {
                    const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
                    if (rectangleLayer) window.redrawRectangleLayer(rectangleLayer);
                }
                // プレビューを更新
                renderPropertyPreview(rectangle.id);
            }
        });

        const label = document.createElement('label');
        label.textContent = option.label;

        radioWrapper.appendChild(input);
        radioWrapper.appendChild(label);
        wrapper.appendChild(radioWrapper);
    });

    return wrapper;
}

// ================
// イベントハンドラ
// ================

/**
 * カテゴリ変更ハンドラ
 *
 * @private
 */
function handleCategoryChange(event) {
    const selectedRect = getSelectedRectangle();
    if (!selectedRect) return;

    const newType = event.target.value;
    const oldType = selectedRect.objectType;

    if (newType === oldType) return;

    // オブジェクトタイプを変更
    setObjectType(selectedRect.id, newType);

    // パネルを再描画（詳細設定が変わるため）
    updatePropertyPanel(selectedRect.id);

    // プレビューを更新
    renderPropertyPreview(selectedRect.id);

    showSuccess(`カテゴリを「${OBJECT_TYPE_LABELS[newType]}」に変更しました`);
}

/**
 * 高さ入力ハンドラ（リアルタイム）
 *
 * @private
 */
function handleHeightChange(event) {
    const selectedRect = getSelectedRectangle();
    if (!selectedRect) return;

    const value = parseFloat(event.target.value);
    if (isNaN(value)) return;

    // バリデーションは行わない（入力中）
    selectedRect.heightMeters = value;
}

/**
 * 高さ入力確定ハンドラ（フォーカス喪失時）
 *
 * @private
 */
function handleHeightBlur(event) {
    const selectedRect = getSelectedRectangle();
    if (!selectedRect) return;

    const value = parseFloat(event.target.value);

    // バリデーション付きで設定
    const success = setHeightMeters(selectedRect.id, value);

    if (!success) {
        // エラーの場合は元の値に戻す
        event.target.value = selectedRect.heightMeters.toFixed(2);
    } else {
        // プレビューを更新
        renderPropertyPreview(selectedRect.id);
    }
}

/**
 * 前面方向変更ハンドラ
 *
 * @private
 */
function handleFrontDirectionChange(event) {
    const selectedRect = getSelectedRectangle();
    if (!selectedRect) return;

    const direction = event.target.value;
    setFrontDirection(selectedRect.id, direction);

    // プレビューを更新
    renderPropertyPreview(selectedRect.id);
}

/**
 * プロパティ適用ハンドラ
 *
 * @private
 */
function handleApplyProperties() {
    const selectedRect = getSelectedRectangle();
    if (!selectedRect) return;

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    showSuccess('プロパティを適用しました');
}

/**
 * プロパティリセットハンドラ
 *
 * @private
 */
function handleResetProperties() {
    const selectedRect = getSelectedRectangle();
    if (!selectedRect) return;

    // 確認ダイアログ
    if (!confirm('プロパティをデフォルト値にリセットしますか？')) {
        return;
    }

    resetObjectProperties(selectedRect.id);
    updatePropertyPanel(selectedRect.id);
}
