/**
 * @file layerManager.js
 * @description レイヤー管理システム
 *
 * マップビューアのレイヤースタック管理、レイヤーの作成・削除・表示制御、
 * および各レイヤータイプ（画像、メタデータ、描画）の描画処理を担当します。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
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

    // 画像レイヤーを初期選択
    mapState.selectedLayerId = 'map-image';

    updateLayersPanel();
}

/**
 * 新しいレイヤーを作成する
 *
 * @param {string} id - レイヤーのID
 * @param {string} name - レイヤーの表示名
 * @param {string} type - レイヤータイプ（'image', 'metadata', 'drawing'）
 * @param {boolean} [permanent=false] - 削除不可の恒久レイヤーかどうか
 * @returns {Object} 作成されたレイヤーオブジェクト
 *
 * @example
 * const layer = createLayer('layer-1', 'レイヤー 1', 'drawing', false);
 */
export function createLayer(id, name, type, permanent = false) {
    const container = document.getElementById('mapContainer');
    const canvasStack = document.getElementById('canvasStack');
    if (!container || !canvasStack) return null;

    const layer = {
        id: id,
        name: name,
        type: type,  // 'image', 'metadata', 'drawing'
        permanent: permanent,
        visible: true,
        opacity: 1.0,
        canvas: document.createElement('canvas'),
        ctx: null
    };

    layer.canvas.className = 'canvas-layer';
    layer.canvas.style.display = 'block';
    layer.ctx = layer.canvas.getContext('2d');

    // 描画レイヤーの場合はcanvas.style.opacityを設定し、ストロークを格納する配列を追加
    if (type === 'drawing') {
        layer.canvas.style.opacity = layer.opacity;
        layer.strokes = [];  // ストロークをワールド座標で保存
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

    const layerIndex = mapState.layerStack.findIndex(l => l.id === layerId);
    if (layerIndex === -1) return;

    const layer = mapState.layerStack[layerIndex];
    if (layer.permanent) {
        alert('このレイヤーは削除できません');
        return;
    }

    // 履歴に追加（外部のhistoryManager.jsのsaveToHistory()を呼び出す必要がある）
    // ここでは履歴管理は外部モジュールに任せるため、イベントを発行するか、
    // 呼び出し側で対応してもらう
    // 後で統合時に対応が必要

    // キャンバスを削除
    canvasStack.removeChild(layer.canvas);

    // レイヤースタックから削除
    mapState.layerStack.splice(layerIndex, 1);

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
        const layerItem = document.createElement('div');
        const isSelected = mapState.selectedLayerId === layer.id;
        let className = 'layer-item';
        if (layer.permanent) className += ' permanent';
        if (isSelected) className += ' selected';
        layerItem.className = className;

        const opacityPercent = Math.round(layer.opacity * 100);

        layerItem.innerHTML = `
            <div class="layer-item-header">
                <input type="checkbox" class="layer-visibility-toggle"
                       ${layer.visible ? 'checked' : ''}
                       data-layer-id="${layer.id}">
                <span class="layer-name" data-layer-id="${layer.id}">${layer.name}</span>
                <button class="layer-delete-button"
                        ${layer.permanent ? 'disabled' : ''}
                        data-layer-id="${layer.id}">削除</button>
            </div>
            <div class="layer-opacity-control">
                <span class="layer-opacity-label">不透明度</span>
                <input type="range" class="layer-opacity-slider"
                       min="0" max="1" step="0.01" value="${layer.opacity}"
                       data-layer-id="${layer.id}">
                <span class="layer-opacity-value">${opacityPercent}%</span>
            </div>
        `;

        // チェックボックスにイベントリスナーを追加
        const checkbox = layerItem.querySelector('.layer-visibility-toggle');
        if (checkbox) {
            checkbox.addEventListener('change', function(e) {
                e.stopPropagation();
                toggleLayerVisibility(layer.id, this.checked);
            });
        }

        // 不透明度スライダーにイベントリスナーを追加
        const opacitySlider = layerItem.querySelector('.layer-opacity-slider');
        const opacityValue = layerItem.querySelector('.layer-opacity-value');
        if (opacitySlider && opacityValue) {
            opacitySlider.addEventListener('mousedown', function(e) {
                e.stopPropagation();
            });
            opacitySlider.addEventListener('mousemove', function(e) {
                e.stopPropagation();
            });
            opacitySlider.addEventListener('mouseup', function(e) {
                e.stopPropagation();
            });
            opacitySlider.addEventListener('input', function(e) {
                e.stopPropagation();
                changeLayerOpacity(layer.id, parseFloat(this.value));
                opacityValue.textContent = Math.round(this.value * 100) + '%';
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
            selectLayer(layer.id);
        });

        panelBody.appendChild(layerItem);
    });
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
        // drawMetadataOverlayOnContextは別モジュール（metadataOverlay.js等）で定義されるべき
        // ここでは外部関数を期待する
        if (typeof window.drawMetadataOverlayOnContext === 'function') {
            window.drawMetadataOverlayOnContext(layerCtx, drawX, drawY, scaledWidth, scaledHeight);
        }
    }

    layerCtx.restore();
}
