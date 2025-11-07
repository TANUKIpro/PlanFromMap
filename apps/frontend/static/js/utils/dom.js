/**
 * @file dom.js
 * @description DOM操作に関するユーティリティ関数
 *
 * @exports isUIElement - 要素がUI要素かどうかを判定
 */

/**
 * 要素がUI要素（パネル、ボタン、入力など）かどうかを判定
 *
 * この関数は、クリックやマウスイベントがUI要素上で発生したかを判定するために使用されます。
 * UI要素上でのイベントは、マップ操作（パン、ズーム、描画など）をトリガーしないようにします。
 *
 * @param {HTMLElement|null} element - 判定する要素
 * @returns {boolean} UI要素の場合はtrue
 *
 * @example
 * const clickedElement = event.target;
 * if (isUIElement(clickedElement)) {
 *   // UI要素上のクリックなので、マップ操作は行わない
 *   return;
 * }
 */
export function isUIElement(element) {
    if (!element) return false;

    // ボタン、入力、スライダーなどのUI要素
    if (element.tagName === 'BUTTON' ||
        element.tagName === 'INPUT' ||
        element.tagName === 'SELECT' ||
        element.tagName === 'TEXTAREA' ||
        element.tagName === 'A') {
        return true;
    }

    // パネルやツールバーなどのクラスをチェック
    if (element.classList) {
        if (element.classList.contains('panel') ||
            element.classList.contains('tool-button') ||
            element.classList.contains('drawing-tools-palette') ||
            element.classList.contains('drawing-options') ||
            element.classList.contains('layer-item') ||
            element.classList.contains('layer-controls') ||
            element.id === 'layersPanel' ||
            element.id === 'drawingToolsPalette') {
            return true;
        }
    }

    // 親要素をチェック
    if (element.parentElement && element.parentElement !== document.body) {
        return isUIElement(element.parentElement);
    }

    return false;
}
