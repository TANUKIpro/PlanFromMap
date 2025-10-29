/**
 * @file canvas.js
 * @description Canvas操作ユーティリティ関数
 *
 * このファイルには、HTML5 Canvasの操作に関連する汎用的なユーティリティ関数が
 * 含まれます。現在は空ですが、将来的に以下のような機能を追加できます：
 * - Canvas要素の作成と設定
 * - Canvasコンテキストの設定
 * - 画像データの操作
 * - Canvas間のコピー
 *
 * @exports (将来的な拡張用)
 */

// 現在、このファイルにはエクスポートされる関数はありません。
// 将来的な拡張のためのプレースホルダーとして使用してください。

/**
 * Canvas要素を作成して初期設定を行う（例）
 *
 * @param {number} width - Canvasの幅
 * @param {number} height - Canvasの高さ
 * @param {string} className - CSSクラス名（オプション）
 * @returns {HTMLCanvasElement} 作成されたCanvas要素
 *
 * @example
 * // const canvas = createCanvas(800, 600, 'my-canvas');
 * // document.body.appendChild(canvas);
 */
// export function createCanvas(width, height, className = '') {
//     const canvas = document.createElement('canvas');
//     canvas.width = width;
//     canvas.height = height;
//     if (className) {
//         canvas.className = className;
//     }
//     return canvas;
// }

/**
 * CanvasのImageDataをコピー（例）
 *
 * @param {CanvasRenderingContext2D} sourceCtx - コピー元のコンテキスト
 * @param {CanvasRenderingContext2D} targetCtx - コピー先のコンテキスト
 * @param {number} width - コピーする幅
 * @param {number} height - コピーする高さ
 *
 * @example
 * // copyImageData(sourceCtx, targetCtx, 800, 600);
 */
// export function copyImageData(sourceCtx, targetCtx, width, height) {
//     const imageData = sourceCtx.getImageData(0, 0, width, height);
//     targetCtx.putImageData(imageData, 0, 0);
// }
