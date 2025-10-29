/**
 * @file coordinates.js
 * @description 座標変換ユーティリティ関数
 *
 * このファイルには、様々な座標系間の変換を行うユーティリティ関数が含まれています：
 * - ワールド座標系（メートル単位）
 * - 画像ピクセル座標系
 * - キャンバス座標系
 *
 * @requires ../state/mapState.js - マップの状態管理
 *
 * @exports worldToCanvas - ワールド座標をキャンバス座標に変換
 * @exports canvasToImagePixel - キャンバス座標を画像ピクセル座標に変換
 * @exports imagePixelToCanvas - 画像ピクセル座標をキャンバス座標に変換
 */

import { mapState } from '../state/mapState.js';

/**
 * 実世界座標（メートル）をキャンバス座標に変換
 *
 * この関数は、ROS/Gazeboなどで使用される実世界のメートル座標を、
 * 画面上のキャンバス座標に変換します。変換には以下の要素が考慮されます：
 * - メタデータのorigin（原点のワールド座標）
 * - 解像度（メートル/ピクセル）
 * - 画像の高さ（Y軸の反転のため）
 * - 現在のスケールとオフセット
 *
 * @param {number} worldX - ワールド座標のX（メートル）
 * @param {number} worldY - ワールド座標のY（メートル）
 * @param {number} drawX - 画像の描画開始X座標（キャンバス上）
 * @param {number} drawY - 画像の描画開始Y座標（キャンバス上）
 * @returns {Object|null} キャンバス座標 {x, y}、またはメタデータがない場合はnull
 *
 * @example
 * // ワールド座標 (1.5, 2.0) をキャンバス座標に変換
 * const canvasPos = worldToCanvas(1.5, 2.0, drawX, drawY);
 * if (canvasPos) {
 *   console.log(`Canvas position: ${canvasPos.x}, ${canvasPos.y}`);
 * }
 */
export function worldToCanvas(worldX, worldY, drawX, drawY) {
    if (!mapState.metadata || !Array.isArray(mapState.metadata.origin)) {
        return null;
    }
    const resolution = mapState.metadata.resolution;
    if (!resolution) return null;

    const [originX, originY] = mapState.metadata.origin;
    const pixelX = (worldX - originX) / resolution;
    const pixelY = mapState.image.height - (worldY - originY) / resolution;

    if (!isFinite(pixelX) || !isFinite(pixelY)) {
        return null;
    }

    return {
        x: drawX + pixelX * mapState.scale,
        y: drawY + pixelY * mapState.scale
    };
}

/**
 * キャンバス座標をマップ画像ピクセル座標に変換
 *
 * この関数は、画面上のキャンバス座標を、元の画像のピクセル座標に変換します。
 * 画像の中央配置、オフセット、スケールが考慮されます。
 *
 * @param {number} canvasX - キャンバス上のX座標
 * @param {number} canvasY - キャンバス上のY座標
 * @param {HTMLElement} container - キャンバスのコンテナ要素（幅と高さの取得に使用）
 * @returns {Object} 画像ピクセル座標 {x, y}
 *
 * @example
 * // マウスクリック位置をピクセル座標に変換
 * const rect = container.getBoundingClientRect();
 * const canvasX = e.clientX - rect.left;
 * const canvasY = e.clientY - rect.top;
 * const pixelPos = canvasToImagePixel(canvasX, canvasY, container);
 * console.log(`Pixel position: ${pixelPos.x}, ${pixelPos.y}`);
 */
export function canvasToImagePixel(canvasX, canvasY, container) {
    if (!mapState.image) return {x: canvasX, y: canvasY};

    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    const baseX = (container.getBoundingClientRect().width - scaledWidth) / 2;
    const baseY = (container.getBoundingClientRect().height - scaledHeight) / 2;

    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    const imagePixelX = (canvasX - drawX) / mapState.scale;
    const imagePixelY = (canvasY - drawY) / mapState.scale;

    return {x: imagePixelX, y: imagePixelY};
}

/**
 * マップ画像ピクセル座標をキャンバス座標に変換
 *
 * この関数は、画像のピクセル座標を、画面上のキャンバス座標に変換します。
 * 画像の中央配置、オフセット、スケールが考慮されます。
 * canvasToImagePixel の逆変換です。
 *
 * @param {number} imagePixelX - 画像のピクセルX座標
 * @param {number} imagePixelY - 画像のピクセルY座標
 * @param {HTMLElement} container - キャンバスのコンテナ要素（幅と高さの取得に使用）
 * @returns {Object} キャンバス座標 {x, y}
 *
 * @example
 * // 画像の特定のピクセル位置（100, 100）を画面上の座標に変換
 * const canvasPos = imagePixelToCanvas(100, 100, container);
 * console.log(`Canvas position: ${canvasPos.x}, ${canvasPos.y}`);
 */
export function imagePixelToCanvas(imagePixelX, imagePixelY, container) {
    if (!mapState.image) return {x: imagePixelX, y: imagePixelY};

    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    const baseX = (container.getBoundingClientRect().width - scaledWidth) / 2;
    const baseY = (container.getBoundingClientRect().height - scaledHeight) / 2;

    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    const canvasX = imagePixelX * mapState.scale + drawX;
    const canvasY = imagePixelY * mapState.scale + drawY;

    return {x: canvasX, y: canvasY};
}
