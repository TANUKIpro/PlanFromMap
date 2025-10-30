/**
 * @file layerManager.js
 * @description レイヤー管理システム
 *
 * マップビューアのレイヤースタック管理、レイヤーの作成・削除・表示制御、
 * および各レイヤータイプ（画像、メタデータ、描画）の描画処理を担当します。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ./overlayRenderer.js - メタデータオーバーレイ描画
 *
 * @exports initializeLayers - レイヤーシステムの初期化
 * @exports createLayer - 新しいレイヤーの作成
 * @exports addNewLayer - 新しい描画レイヤーの追加
 * @exports deleteLayer - レイヤーの削除
 * @exports toggleLayerVisibility - レイヤーの表示/非表示切り替え
 * @exports selectLayer - レイヤーの選択
 * @exports renameLayer - レイヤー名の変更
 * @exports changeLayerOpacity - レイヤー不透明度の変更
 * @exports updateLayersPanel - レイヤーパネルUIの更新
 * @exports redrawAllLayers - すべてのレイヤーの再描画
 * @exports drawImageLayer - 画像レイヤーの描画
 * @exports drawMetadataLayer - メタデータレイヤーの描画
 */

import { mapState } from '../state/mapState.js';
import { drawMetadataOverlayOnContext } from './overlayRenderer.js';

/**
 * レイヤーシステムを初期化する
 * 画像レイヤーとメタデータレイヤーを作成し、初期状態を設定します
 *
 * @returns {void}
 *
 * @example
 * initializeLayers();
 */
export function initializeLayers() {
    const canvasStack = document.getElementById('canvasStack');
    if (!canvasStack) return;

    mapState.layerStack = [];
    canvasStack.innerHTML = '';

    // 画像レイヤーを作成（恒久的）
    createLayer('map-image', '地図画像', 'image', true);

    // メタデータレイヤーを作成（恒久的）
    createLayer('metadata', 'メタデータ', 'metadata', true);

    // 四角形レイヤーを作成（恒久的、特殊レイヤー）
    createLayer('rectangles', '四角形', 'rectangle', true);

    // 画像レイヤーを初期選択
    mapState.selectedLayerId = 'map-image';

    updateLayersPanel();
}

/**
 * 新しいレイヤーを作成する
 *
 * @param {string} id - レイヤーのID
 * @param {string} name - レイヤーの表示名
 * @param {string} type - レイヤータイプ（'image', 'metadata', 'drawing', 'rectangle'）
 * @param {boolean} [permanent=false] - 削除不可の恒久レイヤーかどうか
 * @returns {Object} 作成されたレイヤーオブジェクト
 *
 * @example
 * const layer = createLayer('layer-1', 'レイヤー 1', 'drawing', false);
 */
export function createLayer(id, name, type, permanent = false, parentId = null, rectangleId = null) {
    const container = document.getElementById('mapContainer');
    const canvasStack = document.getElementById('canvasStack');
    if (!container || !canvasStack) return null;

    const layer = {
        id: id,
        name: name,
        type: type,  // 'image', 'metadata', 'drawing', 'rectangle', 'rectangle-child'
        permanent: permanent,
        visible: true,
        opacity: 1.0,
        canvas: document.createElement('canvas'),
        ctx: null,
        children: [],  // 子レイヤー
        collapsed: false,  // 折りたたみ状態
        parentId: parentId,  // 親レイヤーのID
        rectangleId: rectangleId  // 対応する四角形のID(子レイヤーの場合)
    };

    layer.canvas.className = 'canvas-layer';
    layer.canvas.style.display = 'block';
    layer.ctx = layer.canvas.getContext('2d');

    // 描画レイヤーの場合はcanvas.style.opacityを設定し、ストロークを格納する配列を追加
    if (type === 'drawing') {
        layer.canvas.style.opacity = layer.opacity;
        layer.strokes = [];  // ストロークをワールド座標で保存
    }

    // 四角形レイヤーの場合は初期非表示
    if (type === 'rectangle') {
        layer.visible = false;
        layer.canvas.style.display = 'none';
    }

    // 四角形の子レイヤーの場合は非表示(親の四角形レイヤーのキャンバスに描画されるため)
    if (type === 'rectangle-child') {
        layer.visible = false;
        layer.canvas.style.display = 'none';
    }

    // キャンバスのサイズを設定
    const containerRect = container.getBoundingClientRect();
    layer.canvas.width = containerRect.width;
    layer.canvas.height = containerRect.height;

    // canvasStackに追加
    canvasStack.appendChild(layer.canvas);

    // レイヤースタックに追加
    mapState.layerStack.push(layer);

    return layer;
}

/**
 * 新しい描画レイヤーを追加する
 * レイヤーパネルが閉じている場合は自動的に開きます
 *
 * @returns {void}
 *
 * @example
 * addNewLayer();
 */
export function addNewLayer() {
    const layerId = 'layer-' + mapState.nextLayerId;
    const layerName = 'レイヤー ' + mapState.nextLayerId;
    mapState.nextLayerId++;

    createLayer(layerId, layerName, 'drawing', false);

    // 新しいレイヤーを自動選択
    mapState.selectedLayerId = layerId;

    updateLayersPanel();
    redrawAllLayers();

    // パネルが閉じている場合は開く
    const panel = document.getElementById('layersPanel');
    if (panel && !panel.classList.contains('active')) {
        // toggleLayersPanel()を呼び出す代わりに、直接activeクラスを追加
        panel.classList.add('active');
    }
}

/**
 * 四角形レイヤーに子レイヤーを追加する
 *
 * @param {string} rectangleId - 対応する四角形のID
 * @param {string} [name] - レイヤー名（省略時は自動生成）
 * @returns {Object|null} 作成された子レイヤーオブジェクト
 *
 * @example
 * addRectangleChildLayer('rect-1', '四角形 1');
 */
export function addRectangleChildLayer(rectangleId, name = null) {
    // 四角形レイヤーを取得
    const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
    if (!rectangleLayer) {
        console.error('addRectangleChildLayer: 四角形レイヤーが見つかりません');
        return null;
    }

    // 子レイヤー名を生成
    const childLayerName = name || `四角形 ${rectangleLayer.children.length + 1}`;
    const childLayerId = `${rectangleLayer.id}-child-${rectangleId}`;

    // 既に存在する場合は何もしない
    const existingChild = rectangleLayer.children.find(c => c.id === childLayerId);
    if (existingChild) {
        return existingChild;
    }

    // 子レイヤーを作成
    const childLayer = createLayer(
        childLayerId,
        childLayerName,
        'rectangle-child',
        false,
        rectangleLayer.id,
        rectangleId
    );

    // 親レイヤーの children 配列に追加
    rectangleLayer.children.push(childLayer);

    return childLayer;
}

/**
 * 四角形の子レイヤーを削除する
 *
 * @param {string} rectangleId - 削除する四角形のID
 * @returns {boolean} 削除に成功した場合はtrue
 *
 * @example
 * deleteRectangleChildLayer('rect-1');
 */
export function deleteRectangleChildLayer(rectangleId) {
    // 四角形レイヤーを取得
    const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
    if (!rectangleLayer) return false;

    // 子レイヤーを検索
    const childIndex = rectangleLayer.children.findIndex(c => c.rectangleId === rectangleId);
    if (childIndex === -1) return false;

    const childLayer = rectangleLayer.children[childIndex];

    // キャンバスを削除
    const canvasStack = document.getElementById('canvasStack');
    if (canvasStack && childLayer.canvas) {
        canvasStack.removeChild(childLayer.canvas);
    }

    // 親レイヤーの children 配列から削除
    rectangleLayer.children.splice(childIndex, 1);

    return true;
}

/**
 * 四角形レイヤーの折りたたみ状態を切り替える
 *
 * @returns {void}
 *
 * @example
 * toggleRectangleLayerCollapse();
 */
export function toggleRectangleLayerCollapse() {
    const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
    if (!rectangleLayer) return;

    rectangleLayer.collapsed = !rectangleLayer.collapsed;
    updateLayersPanel();
}

/**
 * レイヤーを削除する
 * 恒久レイヤーは削除できません
 *
 * @param {string} layerId - 削除するレイヤーのID
 * @returns {void}
 *
 * @example
 * deleteLayer('layer-1');
 */
export function deleteLayer(layerId) {
    const canvasStack = document.getElementById('canvasStack');
    if (!canvasStack) return;

    // レイヤーを検索（親レイヤーまたは子レイヤー）
    let layer = mapState.layerStack.find(l => l.id === layerId);
    let isChildLayer = false;
    let parentLayer = null;

    // 親レイヤーのスタックに見つからない場合、子レイヤーを検索
    if (!layer) {
        for (const l of mapState.layerStack) {
            if (l.children && l.children.length > 0) {
                layer = l.children.find(c => c.id === layerId);
                if (layer) {
                    isChildLayer = true;
                    parentLayer = l;
                    break;
                }
            }
        }
    }

    if (!layer) return;

    if (layer.permanent) {
        alert('このレイヤーは削除できません');
        return;
    }

    // 子レイヤーの場合、対応する四角形を削除
    if (isChildLayer && layer.rectangleId) {
        if (window.deleteRectangle && typeof window.deleteRectangle === 'function') {
            window.deleteRectangle(layer.rectangleId);
        }

        // 親レイヤーのchildren配列から削除
        if (parentLayer) {
            const childIndex = parentLayer.children.findIndex(c => c.id === layerId);
            if (childIndex !== -1) {
                parentLayer.children.splice(childIndex, 1);
            }
        }

        // キャンバスを削除
        if (layer.canvas && layer.canvas.parentNode) {
            canvasStack.removeChild(layer.canvas);
        }
    } else {
        // 親レイヤーの場合
        const layerIndex = mapState.layerStack.findIndex(l => l.id === layerId);
        if (layerIndex !== -1) {
            // キャンバスを削除
            canvasStack.removeChild(layer.canvas);

            // レイヤースタックから削除
            mapState.layerStack.splice(layerIndex, 1);
        }
    }

    updateLayersPanel();
    redrawAllLayers();
}

/**
 * レイヤーの表示/非表示を切り替える
 *
 * @param {string} layerId - 対象レイヤーのID
 * @param {boolean} visible - 表示する場合はtrue、非表示にする場合はfalse
 * @returns {void}
 *
 * @example
 * toggleLayerVisibility('layer-1', true);
 */
export function toggleLayerVisibility(layerId, visible) {
    const layer = mapState.layerStack.find(l => l.id === layerId);
    if (!layer) return;

    layer.visible = visible;

    if (visible) {
        layer.canvas.style.display = 'block';
        // 画像レイヤーとメタデータレイヤーは再描画が必要
        // 描画レイヤーは既存の内容を表示するだけ
        if (layer.type !== 'drawing') {
            redrawAllLayers();
        }
    } else {
        layer.canvas.style.display = 'none';
        // 描画レイヤーの場合はキャンバスをクリアしない（内容を保持）
        // 画像レイヤーとメタデータレイヤーはクリアしても問題ない（再描画可能）
        if (layer.type !== 'drawing') {
            layer.ctx.clearRect(0, 0, layer.canvas.width, layer.canvas.height);
        }
    }
}

/**
 * レイヤーを選択する
 *
 * @param {string} layerId - 選択するレイヤーのID
 * @returns {void}
 *
 * @example
 * selectLayer('layer-1');
 */
export function selectLayer(layerId) {
    mapState.selectedLayerId = layerId;
    updateLayersPanel();
}

/**
 * レイヤー名を変更する
 * 恒久レイヤーの名前は変更できません
 *
 * @param {string} layerId - 対象レイヤーのID
 * @returns {void}
 *
 * @example
 * renameLayer('layer-1');
 */
export function renameLayer(layerId) {
    const layer = mapState.layerStack.find(l => l.id === layerId);
    if (!layer) return;

    // permanentレイヤーは名前変更不可
    if (layer.permanent) {
        alert('このレイヤーの名前は変更できません');
        return;
    }

    const newName = prompt('新しいレイヤー名を入力してください:', layer.name);
    if (newName && newName.trim() !== '') {
        layer.name = newName.trim();
        updateLayersPanel();
    }
}

/**
 * レイヤーの不透明度を変更する
 *
 * @param {string} layerId - 対象レイヤーのID
 * @param {number} opacity - 不透明度（0.0-1.0）
 * @returns {void}
 *
 * @example
 * changeLayerOpacity('layer-1', 0.5);
 */
export function changeLayerOpacity(layerId, opacity) {
    const layer = mapState.layerStack.find(l => l.id === layerId);
    if (!layer) return;

    layer.opacity = opacity;

    // 描画レイヤーはcanvas.style.opacityで不透明度を設定
    // （既に描画された内容に対して不透明度を適用）
    if (layer.type === 'drawing') {
        layer.canvas.style.opacity = opacity;
    } else {
        // 画像レイヤーとメタデータレイヤーは再描画時にglobalAlphaで不透明度を適用
        redrawAllLayers();
    }
}

/**
 * レイヤーパネルのUIを更新する
 * レイヤーリストを再構築し、イベントリスナーを設定します
 *
 * @returns {void}
 *
 * @example
 * updateLayersPanel();
 */
export function updateLayersPanel() {
    const panelBody = document.getElementById('layersPanelBody');
    if (!panelBody) return;

    panelBody.innerHTML = '';

    // レイヤーを逆順で表示（上のレイヤーが先に表示される）
    const reversedLayers = [...mapState.layerStack].reverse();

    reversedLayers.forEach(layer => {
        // 四角形の親レイヤーは表示しない（子レイヤーのみ表示）
        if (layer.type === 'rectangle') {
            // 子レイヤーのみを描画
            if (layer.children && layer.children.length > 0) {
                layer.children.forEach(childLayer => {
                    renderLayerItem(childLayer, panelBody, 0);
                });
            }
        } else {
            // その他のレイヤーは通常通り描画
            renderLayerItem(layer, panelBody, 0);
        }
    });

    // グローバル不透明度バーを初期化
    initializeGlobalOpacityControl();
}

/**
 * グローバル不透明度コントロールを初期化する
 *
 * @private
 * @returns {void}
 */
function initializeGlobalOpacityControl() {
    const globalOpacitySlider = document.getElementById('globalOpacitySlider');
    const globalOpacityValue = document.getElementById('globalOpacityValue');

    if (!globalOpacitySlider || !globalOpacityValue) return;

    // 選択されたレイヤーの不透明度を反映
    const selectedLayer = getSelectedLayer();
    if (selectedLayer) {
        globalOpacitySlider.value = selectedLayer.opacity;
        globalOpacityValue.textContent = Math.round(selectedLayer.opacity * 100) + '%';
    } else {
        globalOpacitySlider.value = 1.0;
        globalOpacityValue.textContent = '100%';
    }

    // イベントリスナーをリセット（重複を避けるため）
    const newSlider = globalOpacitySlider.cloneNode(true);
    globalOpacitySlider.parentNode.replaceChild(newSlider, globalOpacitySlider);

    // 新しいスライダーにイベントリスナーを追加
    const updatedSlider = document.getElementById('globalOpacitySlider');
    const updatedValue = document.getElementById('globalOpacityValue');

    updatedSlider.addEventListener('input', function(e) {
        const selectedLayer = getSelectedLayer();
        if (selectedLayer) {
            changeLayerOpacity(selectedLayer.id, parseFloat(this.value));
            updatedValue.textContent = Math.round(this.value * 100) + '%';
        }
    });
}

/**
 * 選択されたレイヤーを取得する
 *
 * @private
 * @returns {Object|null} 選択されたレイヤー、なければnull
 */
function getSelectedLayer() {
    // 親レイヤースタックから検索
    let selectedLayer = mapState.layerStack.find(l => l.id === mapState.selectedLayerId);

    // 見つからない場合は子レイヤーを検索
    if (!selectedLayer) {
        for (const layer of mapState.layerStack) {
            if (layer.children && layer.children.length > 0) {
                selectedLayer = layer.children.find(c => c.id === mapState.selectedLayerId);
                if (selectedLayer) break;
            }
        }
    }

    return selectedLayer || null;
}

/**
 * レイヤーアイテムを描画する（再帰的に子レイヤーも描画）
 *
 * @private
 * @param {Object} layer - レイヤーオブジェクト
 * @param {HTMLElement} container - 追加先のコンテナ
 * @param {number} depth - ネストの深さ
 * @returns {void}
 */
function renderLayerItem(layer, container, depth) {
    const layerItem = document.createElement('div');
    const isSelected = mapState.selectedLayerId === layer.id;
    let className = 'layer-item';
    if (layer.permanent) className += ' permanent';
    if (isSelected) className += ' selected';
    if (depth > 0) className += ' child-layer';
    layerItem.className = className;

    // インデント用のスタイル
    const indentStyle = depth > 0 ? `style="margin-left: ${depth * 20}px;"` : '';

    const opacityPercent = Math.round(layer.opacity * 100);

    // 四角形レイヤーの場合、折りたたみボタンを表示
    const hasChildren = layer.type === 'rectangle' && layer.children && layer.children.length > 0;
    const collapseButton = hasChildren
        ? `<button class="layer-collapse-button" data-layer-id="${layer.id}">
               ${layer.collapsed ? '▷' : '▽'}
           </button>`
        : '';

    layerItem.innerHTML = `
        <div class="layer-item-header" ${indentStyle}>
            ${collapseButton}
            <button class="layer-visibility-toggle" data-layer-id="${layer.id}" data-visible="${layer.visible}">
                <img src="icons/${layer.visible ? 'eye-open.svg' : 'eye-close.svg'}" alt="${layer.visible ? '表示' : '非表示'}" class="visibility-icon">
            </button>
            <span class="layer-name" data-layer-id="${layer.id}">${layer.name}</span>
            <button class="layer-delete-button"
                    ${layer.permanent ? 'disabled' : ''}
                    data-layer-id="${layer.id}">
                <img src="icons/trash.svg" alt="削除" class="delete-icon">
            </button>
        </div>
    `;

    // 折りたたみボタンにイベントリスナーを追加
    const collapseBtn = layerItem.querySelector('.layer-collapse-button');
    if (collapseBtn) {
        collapseBtn.addEventListener('click', function(e) {
            e.stopPropagation();
            toggleRectangleLayerCollapse();
        });
    }

    // 可視性トグルボタンにイベントリスナーを追加
    const visibilityToggle = layerItem.querySelector('.layer-visibility-toggle');
    if (visibilityToggle) {
        visibilityToggle.addEventListener('click', function(e) {
            e.stopPropagation();
            const currentVisible = this.dataset.visible === 'true';
            const newVisible = !currentVisible;
            toggleLayerVisibility(layer.id, newVisible);
            // ボタンの状態を更新
            this.dataset.visible = newVisible;
            const icon = this.querySelector('.visibility-icon');
            if (icon) {
                icon.src = `icons/${newVisible ? 'eye-open.svg' : 'eye-close.svg'}`;
                icon.alt = newVisible ? '表示' : '非表示';
            }
        });
    }

    // レイヤー名をダブルクリックで編集可能にする
    const layerNameSpan = layerItem.querySelector('.layer-name');
    if (layerNameSpan) {
        layerNameSpan.style.cursor = 'text';
        layerNameSpan.title = 'ダブルクリックで名前を変更';
        layerNameSpan.addEventListener('dblclick', function(e) {
            e.stopPropagation();
            renameLayer(layer.id);
        });
    }

    // 削除ボタンにイベントリスナーを追加
    const deleteButton = layerItem.querySelector('.layer-delete-button');
    if (deleteButton) {
        deleteButton.addEventListener('click', function(e) {
            e.stopPropagation();
            deleteLayer(layer.id);
        });
    }

    // レイヤーアイテム全体をクリックで選択
    layerItem.addEventListener('click', function(e) {
        selectLayerAndRectangle(layer);
    });

    container.appendChild(layerItem);

    // 子レイヤーを描画（折りたたまれていない場合）
    if (hasChildren && !layer.collapsed) {
        layer.children.forEach(childLayer => {
            renderLayerItem(childLayer, container, depth + 1);
        });
    }
}

/**
 * レイヤーを選択し、四角形の子レイヤーの場合は対応する四角形も選択する
 *
 * @private
 * @param {Object} layer - 選択するレイヤー
 * @returns {void}
 */
function selectLayerAndRectangle(layer) {
    selectLayer(layer.id);

    // 四角形の子レイヤーの場合は、対応する四角形を選択
    if (layer.type === 'rectangle-child' && layer.rectangleId) {
        // 四角形ツールを有効化
        if (window.toggleRectangleTool && typeof window.toggleRectangleTool === 'function') {
            window.toggleRectangleTool(true);
        }

        // 対応する四角形を選択
        if (window.selectRectangle && typeof window.selectRectangle === 'function') {
            window.selectRectangle(layer.rectangleId);
        }

        // 四角形レイヤーを再描画
        const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
        if (rectangleLayer && window.redrawRectangleLayer && typeof window.redrawRectangleLayer === 'function') {
            window.redrawRectangleLayer(rectangleLayer);
        }

        // パンモードに切り替え
        if (window.selectTool && typeof window.selectTool === 'function') {
            window.selectTool('pan');
        }
    }
}

/**
 * すべてのレイヤーを再描画する
 * パン・ズーム・レイヤー設定変更時に呼び出されます
 *
 * @returns {void}
 *
 * @example
 * redrawAllLayers();
 */
export function redrawAllLayers() {
    mapState.layerStack.forEach(layer => {
        if (!layer.visible) return;

        if (layer.type === 'image' && mapState.image) {
            layer.ctx.clearRect(0, 0, layer.canvas.width, layer.canvas.height);
            drawImageLayer(layer);
        } else if (layer.type === 'metadata' && mapState.metadata) {
            layer.ctx.clearRect(0, 0, layer.canvas.width, layer.canvas.height);
            drawMetadataLayer(layer);
        } else if (layer.type === 'drawing') {
            // 描画レイヤーはストロークデータから再描画
            // redrawDrawingLayerは別モジュール（drawingTools.js等）で定義されるべき
            // ここでは外部関数を期待する
            if (typeof window.redrawDrawingLayer === 'function') {
                window.redrawDrawingLayer(layer);
            }
        } else if (layer.type === 'rectangle') {
            // 四角形レイヤーは別モジュール（rectangleRenderer.js等）で定義される関数で再描画
            if (typeof window.redrawRectangleLayer === 'function') {
                window.redrawRectangleLayer(layer);
            }
        }
    });
}

/**
 * 画像レイヤーを描画する
 *
 * @param {Object} layer - 描画対象のレイヤーオブジェクト
 * @returns {void}
 *
 * @example
 * drawImageLayer(imageLayer);
 */
export function drawImageLayer(layer) {
    if (!mapState.image) return;

    const ctx = layer.ctx;

    // 背景色
    ctx.fillStyle = '#f5f5f5';
    ctx.fillRect(0, 0, layer.canvas.width, layer.canvas.height);

    // 画像のスケール後のサイズを計算
    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    // 中央配置の基準位置を計算
    const baseX = (layer.canvas.width - scaledWidth) / 2;
    const baseY = (layer.canvas.height - scaledHeight) / 2;

    // オフセットを適用して描画
    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    // 透過率を適用
    ctx.save();
    ctx.globalAlpha = layer.opacity;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapState.image, drawX, drawY, scaledWidth, scaledHeight);
    ctx.restore();
}

/**
 * メタデータレイヤーを描画する
 * グリッド、原点、スケールバーなどのメタデータ情報を表示します
 *
 * @param {Object} layer - 描画対象のレイヤーオブジェクト
 * @returns {void}
 *
 * @example
 * drawMetadataLayer(metadataLayer);
 */
export function drawMetadataLayer(layer) {
    if (!mapState.metadata || !mapState.image) return;

    const layerCtx = layer.ctx;

    // 画像のスケール後のサイズを計算
    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    // 中央配置の基準位置を計算
    const baseX = (layer.canvas.width - scaledWidth) / 2;
    const baseY = (layer.canvas.height - scaledHeight) / 2;

    // オフセットを適用
    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    // 透過率を適用
    layerCtx.save();
    layerCtx.globalAlpha = layer.opacity;

    // メタデータオーバーレイを描画（レイヤーのctxを使用）
    if (mapState.layers.metadataOverlay) {
        drawMetadataOverlayOnContext(layerCtx, drawX, drawY, scaledWidth, scaledHeight);
    }

    layerCtx.restore();
}
