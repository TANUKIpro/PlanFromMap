/**
 * @file statusBar.js
 * @description ステータスバーの管理機能を提供するモジュール
 * @requires ../state/mapState.js - マップの状態管理
 * @exports updateStatusBar - ステータスバーを更新
 * @exports updateCursorPosition - カーソル位置を更新
 * @exports updateSelectedLayer - 選択されているレイヤーを更新
 * @exports updateMetadataStatus - メタデータステータスを更新
 * @exports toggleStatusBar - ステータスバーの表示/非表示を切り替え
 */

import { mapState } from '../state/mapState.js';

/**
 * ステータスバー全体を更新
 * @export
 */
export function updateStatusBar() {
    updateFileName();
    updateSelectedLayer();
    updateMetadataStatus();
}

/**
 * ファイル名を更新
 * @export
 */
export function updateFileName() {
    const fileNameElement = document.getElementById('statusFileName');
    if (!fileNameElement) return;

    if (mapState.imageFileName || mapState.yamlFileName) {
        const names = [];
        if (mapState.imageFileName) {
            names.push(mapState.imageFileName);
        }
        if (mapState.yamlFileName && mapState.yamlFileName !== mapState.imageFileName) {
            names.push(mapState.yamlFileName);
        }
        fileNameElement.textContent = names.join(', ');
        fileNameElement.title = names.join('\n');
    } else {
        fileNameElement.textContent = '-';
        fileNameElement.title = '';
    }
}

/**
 * カーソル位置を更新
 * @export
 * @param {number} x - X座標（ワールド座標、単位: m）
 * @param {number} y - Y座標（ワールド座標、単位: m）
 */
export function updateCursorPosition(x, y) {
    const cursorElement = document.getElementById('statusCursor');
    if (!cursorElement) return;

    if (x !== null && y !== null && isFinite(x) && isFinite(y)) {
        cursorElement.textContent = `(${x.toFixed(3)}, ${y.toFixed(3)}) m`;
    } else {
        cursorElement.textContent = '-';
    }
}

/**
 * 選択されているレイヤーを更新
 * @export
 */
export function updateSelectedLayer() {
    const layerElement = document.getElementById('statusLayer');
    if (!layerElement) return;

    if (mapState.selectedLayerId) {
        // まず親レイヤースタックから検索
        let layer = mapState.layerStack.find(l => l.id === mapState.selectedLayerId);
        let parentLayer = null;

        // 見つからない場合は子レイヤーを検索
        if (!layer) {
            for (const parent of mapState.layerStack) {
                if (parent.children && parent.children.length > 0) {
                    layer = parent.children.find(c => c.id === mapState.selectedLayerId);
                    if (layer) {
                        parentLayer = parent;
                        break;
                    }
                }
            }
        }

        if (layer) {
            // 子レイヤーの場合は「親レイヤー名 - 子レイヤー名」形式で表示
            if (parentLayer) {
                layerElement.textContent = `${parentLayer.name} - ${layer.name}`;
                layerElement.title = `レイヤータイプ: ${layer.type} (親: ${parentLayer.name})`;
            } else {
                layerElement.textContent = layer.name;
                layerElement.title = `レイヤータイプ: ${layer.type}`;
            }
        } else {
            layerElement.textContent = '-';
            layerElement.title = '';
        }
    } else {
        layerElement.textContent = '-';
        layerElement.title = '';
    }
}

/**
 * メタデータステータスを更新
 * @export
 */
export function updateMetadataStatus() {
    const resolutionElement = document.getElementById('statusResolution');
    const originElement = document.getElementById('statusOrigin');

    if (!resolutionElement || !originElement) return;

    if (mapState.metadata) {
        // 解像度
        if (mapState.metadata.resolution !== undefined) {
            const resolution = Number(mapState.metadata.resolution);
            resolutionElement.textContent = isFinite(resolution)
                ? `${resolution.toFixed(3)} m/px`
                : '-';
        } else {
            resolutionElement.textContent = '-';
        }

        // 原点
        if (mapState.metadata.origin && Array.isArray(mapState.metadata.origin)) {
            const [x, y] = mapState.metadata.origin;
            originElement.textContent = `(${Number(x).toFixed(2)}, ${Number(y).toFixed(2)})`;
        } else {
            originElement.textContent = '-';
        }
    } else {
        resolutionElement.textContent = '-';
        originElement.textContent = '-';
    }
}

/**
 * ステータスバーの表示/非表示を切り替え
 * @export
 * @param {boolean} [visible] - 表示する場合はtrue、非表示の場合はfalse（省略時はトグル）
 */
export function toggleStatusBar(visible) {
    const statusBar = document.getElementById('statusBar');
    if (!statusBar) return;

    if (visible === undefined) {
        statusBar.classList.toggle('hidden');
    } else {
        if (visible) {
            statusBar.classList.remove('hidden');
        } else {
            statusBar.classList.add('hidden');
        }
    }

    // 状態をlocalStorageに保存
    const isVisible = !statusBar.classList.contains('hidden');
    localStorage.setItem('statusBar_visible', isVisible ? 'true' : 'false');
}

/**
 * ステータスバーの表示状態を復元
 * @export
 */
export function restoreStatusBarVisibility() {
    const saved = localStorage.getItem('statusBar_visible');
    if (saved === 'false') {
        toggleStatusBar(false);
    } else {
        // デフォルトで表示
        toggleStatusBar(true);
    }
}
