/**
 * @file rectangleManager.js
 * @description 四角形オブジェクトの管理（作成、削除、選択、取得）
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../config.js - 設定値
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 *
 * @exports toggleRectangleTool - 四角形ツールのオン/オフ切り替え
 * @exports createRectangle - 新しい四角形の作成
 * @exports deleteRectangle - 四角形の削除
 * @exports selectRectangle - 四角形の選択
 * @exports deselectRectangle - 四角形の選択解除
 * @exports getRectangleById - IDで四角形を取得
 * @exports getSelectedRectangle - 選択中の四角形を取得
 * @exports getRectangleLayer - 四角形レイヤーを取得
 * @exports updateRectangle - 四角形の更新
 * @exports getAllRectangles - すべての四角形を取得
 */

import { mapState } from '../state/mapState.js';
import { RECTANGLE_DEFAULTS } from '../config.js';
import { OBJECT_TYPES, DEFAULT_3D_PROPERTIES, getDefaultCommonProperties } from '../models/objectTypes.js';

/**
 * 四角形ツールのオン/オフを切り替える
 * ツールがオンになると、四角形レイヤーが表示される
 *
 * @param {boolean} enabled - ツールを有効にする場合はtrue
 * @returns {void}
 *
 * @example
 * toggleRectangleTool(true);  // 四角形ツールをオン
 */
export function toggleRectangleTool(enabled) {
    mapState.rectangleToolState.enabled = enabled;

    // 四角形レイヤーの表示を管理
    const rectangleLayer = getRectangleLayer();
    if (rectangleLayer) {
        // 四角形ツールをオンにする場合は、レイヤーを表示
        if (enabled) {
            rectangleLayer.visible = true;
            rectangleLayer.canvas.style.display = 'block';

            // レイヤーを再描画
            if (window.redrawRectangleLayer && typeof window.redrawRectangleLayer === 'function') {
                window.redrawRectangleLayer(rectangleLayer);
            }
        } else {
            // 四角形ツールをオフにする場合
            // 四角形が既に存在する場合は、レイヤーは表示されたままにする
            // ただし、選択状態は解除する
            if (mapState.rectangleToolState.rectangles.length > 0) {
                // 選択状態を解除
                if (window.deselectRectangle && typeof window.deselectRectangle === 'function') {
                    window.deselectRectangle();
                } else {
                    deselectRectangle();
                }

                // レイヤーは表示されたままにする
                rectangleLayer.visible = true;
                rectangleLayer.canvas.style.display = 'block';

                // レイヤーを再描画（選択状態を解除した状態で）
                if (window.redrawRectangleLayer && typeof window.redrawRectangleLayer === 'function') {
                    window.redrawRectangleLayer(rectangleLayer);
                }
            } else {
                // 四角形が存在しない場合は、レイヤーを非表示にする
                rectangleLayer.visible = false;
                rectangleLayer.canvas.style.display = 'none';
            }
        }

        // レイヤーパネルを更新
        if (window.updateLayersPanel && typeof window.updateLayersPanel === 'function') {
            window.updateLayersPanel();
        }
    }

    // ツールボタンのスタイルを更新
    const rectangleToolButton = document.getElementById('rectangleTool');
    if (rectangleToolButton) {
        if (enabled) {
            rectangleToolButton.style.backgroundColor = '#d0d0d0';
            rectangleToolButton.style.opacity = '0.7';
        } else {
            rectangleToolButton.style.backgroundColor = '';
            rectangleToolButton.style.opacity = '';
        }
    }
}

/**
 * 新しい四角形を作成する
 *
 * @param {number} x - 中心X座標（画像ピクセル）
 * @param {number} y - 中心Y座標（画像ピクセル）
 * @param {number} [width] - 幅（画像ピクセル）、省略時はデフォルト値
 * @param {number} [height] - 高さ（画像ピクセル）、省略時はデフォルト値
 * @param {number} [rotation=0] - 回転角度（度）
 * @param {string} [color] - 四角形の色、省略時は現在の描画色
 * @param {boolean} [skipHistory=false] - 履歴保存をスキップする場合はtrue
 * @returns {Object} 作成された四角形オブジェクト
 *
 * @example
 * const rect = createRectangle(500, 300);
 */
export function createRectangle(x, y, width, height, rotation = 0, color = null, skipHistory = false) {
    const id = 'rect-' + mapState.rectangleToolState.nextRectangleId;
    mapState.rectangleToolState.nextRectangleId++;

    // 色が指定されていない場合は、現在の描画色を使用
    const rectangleColor = color || mapState.drawingState.color || RECTANGLE_DEFAULTS.STROKE_COLOR;

    const rectangle = {
        // 基本プロパティ
        id: id,
        x: x,
        y: y,
        width: width || RECTANGLE_DEFAULTS.DEFAULT_WIDTH,
        height: height || RECTANGLE_DEFAULTS.DEFAULT_HEIGHT,
        rotation: rotation,  // 回転角度（度）
        color: rectangleColor,  // 四角形の色
        selected: false,

        // オブジェクト情報（新規追加）
        objectType: OBJECT_TYPES.NONE,  // デフォルトは「なし」
        heightMeters: DEFAULT_3D_PROPERTIES.heightMeters,  // 高さ（メートル）
        frontDirection: DEFAULT_3D_PROPERTIES.frontDirection,  // 前面方向
        objectProperties: {},  // カテゴリ別のプロパティ（空で初期化）

        // 共通拡張プロパティ
        commonProperties: getDefaultCommonProperties()  // 全オブジェクト共通の詳細情報
    };

    mapState.rectangleToolState.rectangles.push(rectangle);

    // 対応する子レイヤーを作成
    if (window.addRectangleChildLayer && typeof window.addRectangleChildLayer === 'function') {
        window.addRectangleChildLayer(id);
    }

    // レイヤーパネルを更新
    if (window.updateLayersPanel && typeof window.updateLayersPanel === 'function') {
        window.updateLayersPanel();
    }

    return rectangle;
}

/**
 * 四角形を削除する
 *
 * @param {string} rectangleId - 削除する四角形のID
 * @returns {boolean} 削除に成功した場合はtrue
 *
 * @example
 * deleteRectangle('rect-1');
 */
export function deleteRectangle(rectangleId) {
    const index = mapState.rectangleToolState.rectangles.findIndex(r => r.id === rectangleId);
    if (index === -1) return false;

    mapState.rectangleToolState.rectangles.splice(index, 1);

    // 選択中の四角形が削除された場合は選択を解除
    if (mapState.rectangleToolState.selectedRectangleId === rectangleId) {
        mapState.rectangleToolState.selectedRectangleId = null;
    }

    // 対応する子レイヤーを削除
    if (window.deleteRectangleChildLayer && typeof window.deleteRectangleChildLayer === 'function') {
        window.deleteRectangleChildLayer(rectangleId);
    }

    // レイヤーパネルを更新
    if (window.updateLayersPanel && typeof window.updateLayersPanel === 'function') {
        window.updateLayersPanel();
    }

    return true;
}

/**
 * 四角形を選択する
 *
 * @param {string} rectangleId - 選択する四角形のID
 * @returns {void}
 *
 * @example
 * selectRectangle('rect-1');
 */
export function selectRectangle(rectangleId) {
    // すべての四角形の選択を解除
    mapState.rectangleToolState.rectangles.forEach(r => r.selected = false);

    // 指定された四角形を選択
    const rectangle = getRectangleById(rectangleId);
    if (rectangle) {
        rectangle.selected = true;
        mapState.rectangleToolState.selectedRectangleId = rectangleId;

        // オブジェクトプロパティパネルを表示
        if (window.showPropertyPanel && typeof window.showPropertyPanel === 'function') {
            window.showPropertyPanel(rectangleId);
        }
    }
}

/**
 * 四角形の選択を解除する
 *
 * @returns {void}
 *
 * @example
 * deselectRectangle();
 */
export function deselectRectangle() {
    mapState.rectangleToolState.rectangles.forEach(r => r.selected = false);
    mapState.rectangleToolState.selectedRectangleId = null;

    // オブジェクトプロパティパネルを非表示
    if (window.hidePropertyPanel && typeof window.hidePropertyPanel === 'function') {
        window.hidePropertyPanel();
    }
}

/**
 * IDで四角形を取得する
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {Object|null} 四角形オブジェクト、見つからない場合はnull
 *
 * @example
 * const rect = getRectangleById('rect-1');
 */
export function getRectangleById(rectangleId) {
    return mapState.rectangleToolState.rectangles.find(r => r.id === rectangleId) || null;
}

/**
 * 選択中の四角形を取得する
 *
 * @returns {Object|null} 選択中の四角形オブジェクト、なければnull
 *
 * @example
 * const selectedRect = getSelectedRectangle();
 */
export function getSelectedRectangle() {
    return getRectangleById(mapState.rectangleToolState.selectedRectangleId);
}

/**
 * 四角形レイヤーを取得する
 *
 * @returns {Object|null} 四角形レイヤーオブジェクト、見つからない場合はnull
 *
 * @example
 * const layer = getRectangleLayer();
 */
export function getRectangleLayer() {
    return mapState.layerStack.find(l => l.type === 'rectangle') || null;
}

/**
 * 四角形の属性を更新する
 *
 * @param {string} rectangleId - 更新する四角形のID
 * @param {Object} updates - 更新する属性のオブジェクト
 * @returns {Object|null} 更新された四角形オブジェクト、見つからない場合はnull
 *
 * @example
 * updateRectangle('rect-1', { x: 100, y: 200, rotation: 45 });
 */
export function updateRectangle(rectangleId, updates) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return null;

    // commonPropertiesが更新される場合、updatedAtタイムスタンプを更新
    if (updates.commonProperties && rectangle.commonProperties) {
        updates.commonProperties.updatedAt = new Date().toISOString();
    }

    Object.assign(rectangle, updates);

    return rectangle;
}

/**
 * すべての四角形を取得する
 *
 * @returns {Array<Object>} 四角形オブジェクトの配列
 *
 * @example
 * const allRects = getAllRectangles();
 */
export function getAllRectangles() {
    return mapState.rectangleToolState.rectangles;
}

/**
 * 画像の中心座標を取得する（新しい四角形のデフォルト位置として使用）
 *
 * @returns {{x: number, y: number}|null} 画像の中心座標（画像ピクセル）、画像がない場合はnull
 *
 * @example
 * const center = getImageCenter();
 */
export function getImageCenter() {
    if (!mapState.image) return null;

    return {
        x: mapState.image.width / 2,
        y: mapState.image.height / 2
    };
}
