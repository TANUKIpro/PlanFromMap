/**
 * @file metadataDisplay.js
 * @description メタデータ表示関連の機能を提供するモジュール
 * @requires ../state/mapState.js
 * @requires ./layerManager.js
 * @exports displayMetadata
 * @exports toggleMetadataMinimize
 * @exports updateMetadataOverlayVisibility
 * @exports updateOverlayControls
 * @exports toggleLayer
 * @exports toggleOverlayOption
 */

import { mapState } from '../state/mapState.js';
import { redrawAllLayers } from './layerManager.js';

/**
 * メタデータを表示
 * @export
 * @param {Object} metadata - メタデータオブジェクト
 */
export function displayMetadata(metadata) {
    const overlay = document.getElementById('metadataOverlay');
    const contentDiv = document.getElementById('metadataOverlayContent');

    if (!overlay || !contentDiv) {
        return;
    }

    if (!metadata) {
        contentDiv.innerHTML = '';
        overlay.classList.remove('active');
        overlay.setAttribute('aria-hidden', 'true');
        return;
    }

    const sections = [];

    if (metadata.image) {
        sections.push({
            key: 'image',
            label: '画像ファイル',
            value: metadata.image
        });
    }

    if (metadata.resolution !== undefined) {
        const resolution = Number(metadata.resolution);
        sections.push({
            key: 'resolution',
            label: '解像度 (m/pixel)',
            value: isFinite(resolution) ? resolution.toFixed(6) : String(metadata.resolution)
        });
    }

    if (metadata.origin && Array.isArray(metadata.origin)) {
        const formattedOrigin = metadata.origin.map(value => {
            const num = Number(value);
            return Number.isFinite(num) ? num.toFixed(6) : String(value);
        });
        sections.push({
            key: 'origin',
            label: '原点 [x, y, theta]',
            value: `[${formattedOrigin.join(', ')}]`
        });
    }

    if (metadata.negate !== undefined) {
        sections.push({
            key: 'negate',
            label: '白黒反転',
            value: metadata.negate ? '有効' : '無効'
        });
    }

    if (metadata.occupied_thresh !== undefined) {
        const occupied = Number(metadata.occupied_thresh);
        sections.push({
            key: 'occupied_thresh',
            label: '占有閾値',
            value: isFinite(occupied) ? occupied.toFixed(2) : String(metadata.occupied_thresh)
        });
    }

    if (metadata.free_thresh !== undefined) {
        const free = Number(metadata.free_thresh);
        sections.push({
            key: 'free_thresh',
            label: '自由空間閾値',
            value: isFinite(free) ? free.toFixed(3) : String(metadata.free_thresh)
        });
    }

    if (sections.length === 0) {
        contentDiv.innerHTML = `
            <div class="metadata-overlay-item">
                <label>メタデータ</label>
                <value>表示できる情報がありません</value>
            </div>
        `;
    } else {
        const html = sections.map(section => `
            <div class="metadata-overlay-item">
                <label>${section.label}</label>
                <value>${section.value}</value>
            </div>
        `).join('');
        contentDiv.innerHTML = html;
    }
    updateMetadataOverlayVisibility();
}

/**
 * メタデータオーバーレイの最小化/展開を切り替え
 * @export
 */
export function toggleMetadataMinimize() {
    const overlay = document.getElementById('metadataOverlay');
    const button = document.querySelector('.metadata-overlay-minimize');
    if (!overlay) return;

    overlay.classList.toggle('minimized');

    const isMinimized = overlay.classList.contains('minimized');
    if (button) {
        button.setAttribute('title', isMinimized ? '展開' : '最小化');
    }
}

/**
 * メタデータオーバーレイの表示状態を更新
 * @export
 */
export function updateMetadataOverlayVisibility() {
    const overlay = document.getElementById('metadataOverlay');
    if (!overlay) return;

    const metadataAvailable = !!mapState.metadata;
    overlay.classList.toggle('active', metadataAvailable);
    overlay.setAttribute('aria-hidden', metadataAvailable ? 'false' : 'true');
}

/**
 * レイヤーコントロールの状態を更新
 * @export
 */
export function updateOverlayControls() {
    const imageToggle = document.getElementById('imageLayerToggle');
    if (imageToggle) {
        imageToggle.checked = !!mapState.layers.image;
    }

    const metadataAvailable = !!mapState.metadata;
    const metadataToggle = document.getElementById('metadataLayerToggle');
    if (metadataToggle) {
        metadataToggle.checked = metadataAvailable && !!mapState.layers.metadataOverlay;
        metadataToggle.disabled = !metadataAvailable;
        const label = metadataToggle.closest('.metadata-overlay-toggle');
        if (label) {
            label.classList.toggle('disabled', !metadataAvailable);
        }
    }

    document.querySelectorAll('.metadata-dependent input').forEach(input => {
        const optionKey = input.dataset.option;
        if (optionKey && mapState.overlaySettings.hasOwnProperty(optionKey)) {
            input.checked = !!mapState.overlaySettings[optionKey];
        }
        input.disabled = !metadataAvailable;
        const wrapper = input.closest('.metadata-overlay-toggle');
        if (wrapper) {
            wrapper.classList.toggle('disabled', !metadataAvailable);
        }
    });

    const optionsGroup = document.getElementById('metadataOverlayOptions');
    if (optionsGroup) {
        optionsGroup.classList.toggle('inactive', !metadataAvailable);
    }

    updateMetadataOverlayVisibility();
}

/**
 * レイヤー表示の切り替え
 * @export
 * @param {string} layerKey - レイヤーキー ('image', 'metadataOverlay')
 * @param {boolean} enabled - 有効/無効
 */
export function toggleLayer(layerKey, enabled) {
    if (!mapState.layers || !(layerKey in mapState.layers)) {
        return;
    }

    if (layerKey === 'metadataOverlay' && !mapState.metadata) {
        mapState.layers[layerKey] = false;
        updateOverlayControls();
        return;
    }

    mapState.layers[layerKey] = enabled;
    updateOverlayControls();

    // レイヤーシステムを使用している場合は該当レイヤーの表示を切り替え
    if (mapState.layerStack && mapState.layerStack.length > 0) {
        if (layerKey === 'image') {
            const imageLayer = mapState.layerStack.find(l => l.id === 'map-image');
            if (imageLayer) {
                imageLayer.visible = enabled;
                imageLayer.canvas.style.display = enabled ? 'block' : 'none';
            }
        } else if (layerKey === 'metadataOverlay') {
            const metadataLayer = mapState.layerStack.find(l => l.id === 'metadata');
            if (metadataLayer) {
                metadataLayer.visible = enabled;
                metadataLayer.canvas.style.display = enabled ? 'block' : 'none';
            }
        }
    }

    // すべてのレイヤーを再描画
    redrawAllLayers();
}

/**
 * オーバーレイオプションの切り替え
 * @export
 * @param {string} optionKey - オプションキー ('showGrid', 'showOrigin', 'showScaleBar')
 * @param {boolean} enabled - 有効/無効
 */
export function toggleOverlayOption(optionKey, enabled) {
    if (!mapState.overlaySettings || !(optionKey in mapState.overlaySettings)) {
        return;
    }

    if (!mapState.metadata) {
        updateOverlayControls();
        return;
    }

    mapState.overlaySettings[optionKey] = enabled;
    updateOverlayControls();

    // すべてのレイヤーを再描画
    redrawAllLayers();
}
