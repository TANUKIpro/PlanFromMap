/**
 * @file rectangleMeasure.js
 * @description 四角形の測量モード専用のUI・ロジック
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../config.js - 設定値
 * @requires ./rectangleManager.js - 四角形管理
 * @requires ../utils/coordinates.js - 座標変換
 *
 * @exports getEdgeMidpoint - 辺の中心位置を計算
 * @exports promptEdgeLength - 辺の長さを入力するUIを表示
 * @exports showEdgeLengthInputUI - 辺の長さ入力UIを表示
 */

// ================
// インポート
// ================

import { mapState } from '../state/mapState.js';
import { RECTANGLE_DEFAULTS } from '../config.js';
import {
    updateRectangle,
    getRectangleLayer
} from './rectangleManager.js';
import {
    imagePixelToCanvas as imagePixelToCanvasUtil
} from '../utils/coordinates.js';

// ================
// 内部関数 - 座標変換
// ================

/**
 * 画像ピクセル座標をキャンバス座標に変換
 *
 * @private
 * @param {number} imagePixelX - 画像ピクセルX座標
 * @param {number} imagePixelY - 画像ピクセルY座標
 * @returns {{x: number, y: number}} キャンバス座標
 */
function imagePixelToCanvas(imagePixelX, imagePixelY) {
    const container = document.getElementById('mapContainer');
    return imagePixelToCanvasUtil(imagePixelX, imagePixelY, container);
}

// ================
// 公開関数 - 測量計算
// ================

/**
 * 辺の中心位置を計算（キャンバス座標）
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {string} edge - 辺の種類（'top', 'right', 'bottom', 'left'）
 * @returns {{x: number, y: number}} 辺の中心位置（キャンバス座標）
 *
 * @example
 * const midpoint = getEdgeMidpoint(rectangle, 'top');
 */
export function getEdgeMidpoint(rectangle, edge) {
    const rad = (rectangle.rotation * Math.PI) / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    let localX = 0;
    let localY = 0;

    if (edge === 'top') {
        localX = 0;
        localY = -rectangle.height / 2;
    } else if (edge === 'right') {
        localX = rectangle.width / 2;
        localY = 0;
    } else if (edge === 'bottom') {
        localX = 0;
        localY = rectangle.height / 2;
    } else if (edge === 'left') {
        localX = -rectangle.width / 2;
        localY = 0;
    }

    // ローカル座標をグローバル座標に変換
    const globalX = rectangle.x + localX * cos - localY * sin;
    const globalY = rectangle.y + localX * sin + localY * cos;

    // 画像ピクセル座標をキャンバス座標に変換
    return imagePixelToCanvas(globalX, globalY);
}

// ================
// 公開関数 - 測量UI
// ================

/**
 * 辺の長さを入力するUIを表示（Fusion360風）
 *
 * @param {Object} rectangle - 四角形オブジェクト
 * @param {string} edge - 辺の種類（'top', 'right', 'bottom', 'left'）
 * @returns {void}
 *
 * @example
 * promptEdgeLength(rectangle, 'top');
 */
export function promptEdgeLength(rectangle, edge) {
    // メタデータから解像度を取得（メートル/ピクセル）
    const resolution = mapState.metadata ? mapState.metadata.resolution : 0.05;

    // 現在の辺の長さを取得（cm単位）
    let currentLengthInPixels;
    if (edge === 'top' || edge === 'bottom') {
        currentLengthInPixels = rectangle.width;
    } else {
        currentLengthInPixels = rectangle.height;
    }

    const currentLengthInCm = currentLengthInPixels * resolution * 100;

    // 辺の中心位置を計算
    const edgeMidpoint = getEdgeMidpoint(rectangle, edge);

    // UIを表示
    showEdgeLengthInputUI(edgeMidpoint.x, edgeMidpoint.y, currentLengthInCm, (newLengthInCm) => {
        // 確定時のコールバック
        if (newLengthInCm === null) {
            // キャンセル
            mapState.rectangleToolState.editMode = null;
            mapState.rectangleToolState.measureEdge = null;
            return;
        }

        // 最小サイズチェック
        if (newLengthInCm < RECTANGLE_DEFAULTS.MIN_SIZE_CM) {
            alert(`辺の長さは${RECTANGLE_DEFAULTS.MIN_SIZE_CM}cm以上にしてください`);
            mapState.rectangleToolState.editMode = null;
            mapState.rectangleToolState.measureEdge = null;
            return;
        }

        // cm → ピクセルに変換
        const newLengthInPixels = (newLengthInCm / 100) / resolution;

        // 四角形を更新
        if (edge === 'top' || edge === 'bottom') {
            updateRectangle(rectangle.id, {
                width: newLengthInPixels
            });
        } else {
            updateRectangle(rectangle.id, {
                height: newLengthInPixels
            });
        }

        // レイヤーを再描画
        const rectangleLayer = getRectangleLayer();
        if (rectangleLayer && window.redrawRectangleLayer) {
            window.redrawRectangleLayer(rectangleLayer);
        }

        mapState.rectangleToolState.editMode = null;
        mapState.rectangleToolState.measureEdge = null;
    });
}

/**
 * 辺の長さ入力UIを表示
 *
 * @param {number} x - 表示位置X（キャンバス座標）
 * @param {number} y - 表示位置Y（キャンバス座標）
 * @param {number} currentValue - 現在の値（cm）
 * @param {Function} callback - 確定時のコールバック関数
 * @returns {void}
 *
 * @example
 * showEdgeLengthInputUI(100, 200, 50.0, (value) => console.log(value));
 */
export function showEdgeLengthInputUI(x, y, currentValue, callback) {
    const ui = document.getElementById('edgeLengthInputUI');
    const input = document.getElementById('edgeLengthInput');
    const confirmBtn = document.getElementById('edgeLengthConfirm');
    const cancelBtn = document.getElementById('edgeLengthCancel');

    if (!ui || !input || !confirmBtn || !cancelBtn) {
        console.error('Edge length input UI elements not found');
        return;
    }

    // UIの位置を設定（辺の中心より少し下にオフセット）
    ui.style.left = `${x}px`;
    ui.style.top = `${y + RECTANGLE_DEFAULTS.UI_OFFSET_Y}px`;
    ui.style.display = 'block';

    // 入力フィールドに現在の値を設定
    input.value = currentValue.toFixed(1);
    input.focus();
    input.select();

    // 既存のイベントリスナーを削除
    const newConfirmBtn = confirmBtn.cloneNode(true);
    const newCancelBtn = cancelBtn.cloneNode(true);
    confirmBtn.parentNode.replaceChild(newConfirmBtn, confirmBtn);
    cancelBtn.parentNode.replaceChild(newCancelBtn, cancelBtn);

    // 確定処理
    const confirm = () => {
        const newValue = parseFloat(input.value);
        ui.style.display = 'none';

        if (isNaN(newValue) || newValue <= 0) {
            alert('無効な長さです');
            callback(null);
            return;
        }

        callback(newValue);
    };

    // キャンセル処理
    const cancel = () => {
        ui.style.display = 'none';
        callback(null);
    };

    // イベントリスナーを追加
    newConfirmBtn.addEventListener('click', confirm);
    newCancelBtn.addEventListener('click', cancel);

    // Enterキーで確定、Escapeキーでキャンセル
    const keyHandler = (e) => {
        if (e.key === 'Enter') {
            confirm();
            input.removeEventListener('keydown', keyHandler);
        } else if (e.key === 'Escape') {
            cancel();
            input.removeEventListener('keydown', keyHandler);
        }
    };
    input.addEventListener('keydown', keyHandler);
}
