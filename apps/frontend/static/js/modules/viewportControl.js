/**
 * @file viewportControl.js
 * @description ビューポート制御システム
 *
 * マップビューアの表示制御（ズームイン/アウト、表示リセット）と
 * ズーム情報の表示を管理します。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 *
 * @exports resetView - 表示をリセット（スケール1.0、オフセット0に戻す）
 * @exports zoomIn - ズームイン
 * @exports zoomOut - ズームアウト
 * @exports updateZoomInfo - ズーム情報表示の更新
 */

import { mapState } from '../state/mapState.js';

/**
 * ビューポートを再描画する
 * レイヤーシステムまたは旧システムに応じて適切な再描画関数を呼び出します
 *
 * @private
 * @returns {void}
 */
function redrawViewport() {
    // レイヤーシステムが有効な場合
    if (mapState.layerStack.length > 0) {
        // redrawAllLayersは外部関数（layerManager.js等）を期待
        if (typeof window.redrawAllLayers === 'function') {
            window.redrawAllLayers();
        }
    } else {
        // 旧システムの場合はdrawMapを呼び出す
        if (typeof window.drawMap === 'function') {
            window.drawMap();
        }
    }
}

/**
 * 表示をリセットする
 * スケールを1.0、オフセットを0にリセットします
 *
 * @returns {void}
 *
 * @example
 * resetView();
 */
export function resetView() {
    mapState.scale = 1.0;
    mapState.offsetX = 0;
    mapState.offsetY = 0;

    redrawViewport();
}

/**
 * ズームインする
 * スケールを1.2倍にします（最大スケール制限あり）
 *
 * @returns {void}
 *
 * @example
 * zoomIn();
 */
export function zoomIn() {
    if (mapState.scale < mapState.maxScale) {
        mapState.scale *= 1.2;
        redrawViewport();
    }
}

/**
 * ズームアウトする
 * スケールを1/1.2倍にします（最小スケール制限あり）
 *
 * @returns {void}
 *
 * @example
 * zoomOut();
 */
export function zoomOut() {
    if (mapState.scale > mapState.minScale) {
        mapState.scale /= 1.2;
        redrawViewport();
    }
}

/**
 * ズーム情報表示を更新する
 * 現在のスケールをパーセント表示で更新します
 *
 * @returns {void}
 *
 * @example
 * updateZoomInfo();
 */
export function updateZoomInfo() {
    const zoomInfo = document.getElementById('zoomInfo');
    if (!zoomInfo) return;

    const zoomPercent = Math.round(mapState.scale * 100);
    zoomInfo.textContent = `倍率: ${zoomPercent}%`;
}
