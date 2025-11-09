/**
 * @file objectPropertyManager.js
 * @description 四角形のオブジェクトプロパティ管理（タイプ設定、プロパティ更新、検証）
 *
 * このモジュールは、四角形に割り当てられたオブジェクトタイプ（棚、箱、テーブルなど）の
 * プロパティを管理します。プロパティの設定、取得、検証、3D情報の管理を行います。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ./rectangleManager.js - 四角形管理
 * @requires ../ui/toast.js - 通知表示
 *
 * @exports setObjectType - オブジェクトタイプを設定
 * @exports setObjectProperty - 個別プロパティを設定
 * @exports setObjectProperties - 複数プロパティをまとめて設定
 * @exports getObjectProperties - オブジェクトプロパティを取得
 * @exports setHeightMeters - 高さを設定
 * @exports setFrontDirection - 前面方向を設定
 * @exports resetObjectProperties - プロパティをリセット
 * @exports applyObjectType - オブジェクトタイプを適用（デフォルト値で初期化）
 */

import { mapState } from '../state/mapState.js';
import {
    OBJECT_TYPES,
    getDefaultProperties,
    validateProperties,
    validate3DProperties,
    getObjectTypeColor
} from '../models/objectTypes.js';
import { getRectangleById, updateRectangle } from './rectangleManager.js';
import { showSuccess, showError, showWarning } from '../ui/toast.js';

// ================
// オブジェクトタイプ管理
// ================

/**
 * 四角形のオブジェクトタイプを設定する
 * タイプを変更すると、プロパティはデフォルト値にリセットされます
 *
 * @param {string} rectangleId - 四角形のID
 * @param {string} objectType - オブジェクトタイプ（OBJECT_TYPESの値）
 * @param {boolean} [preserveProperties=false] - 既存のプロパティを保持する場合はtrue
 * @returns {boolean} 成功した場合はtrue
 *
 * @example
 * setObjectType('rect-0', 'shelf');
 */
export function setObjectType(rectangleId, objectType, preserveProperties = false) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`setObjectType: 四角形が見つかりません: ${rectangleId}`);
        showError('四角形が見つかりません');
        return false;
    }

    // 有効なオブジェクトタイプかチェック
    if (!Object.values(OBJECT_TYPES).includes(objectType)) {
        console.error(`setObjectType: 無効なオブジェクトタイプ: ${objectType}`);
        showError('無効なオブジェクトタイプです');
        return false;
    }

    const oldType = rectangle.objectType;
    rectangle.objectType = objectType;

    // プロパティの処理
    if (!preserveProperties || oldType !== objectType) {
        // デフォルトプロパティで初期化
        rectangle.objectProperties = getDefaultProperties(objectType);
    }

    // 色をカテゴリに基づいて更新
    rectangle.color = getObjectTypeColor(objectType);

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    // レイヤーを再描画（色が変わる可能性があるため）
    if (window.redrawRectangleLayer && typeof window.redrawRectangleLayer === 'function') {
        const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
        if (rectangleLayer) {
            window.redrawRectangleLayer(rectangleLayer);
        }
    }

    return true;
}

/**
 * オブジェクトタイプを適用し、デフォルトプロパティで初期化する
 * （setObjectTypeのエイリアス、より意図が明確）
 *
 * @param {string} rectangleId - 四角形のID
 * @param {string} objectType - オブジェクトタイプ
 * @returns {boolean} 成功した場合はtrue
 */
export function applyObjectType(rectangleId, objectType) {
    return setObjectType(rectangleId, objectType, false);
}

// ================
// プロパティ管理
// ================

/**
 * オブジェクトの個別プロパティを設定する
 *
 * @param {string} rectangleId - 四角形のID
 * @param {string} propertyKey - プロパティのキー
 * @param {*} propertyValue - プロパティの値
 * @param {boolean} [skipValidation=false] - 検証をスキップする場合はtrue
 * @returns {boolean} 成功した場合はtrue
 *
 * @example
 * setObjectProperty('rect-0', 'shelfLevels', 5);
 */
export function setObjectProperty(rectangleId, propertyKey, propertyValue, skipValidation = false) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`setObjectProperty: 四角形が見つかりません: ${rectangleId}`);
        showError('四角形が見つかりません');
        return false;
    }

    // オブジェクトプロパティが存在しない場合は初期化
    if (!rectangle.objectProperties) {
        rectangle.objectProperties = {};
    }

    // プロパティを設定
    rectangle.objectProperties[propertyKey] = propertyValue;

    // 検証（オプション）
    if (!skipValidation && rectangle.objectType !== OBJECT_TYPES.NONE) {
        const validation = validateProperties(rectangle.objectType, rectangle.objectProperties);
        if (!validation.valid) {
            console.warn(`setObjectProperty: 検証エラー:`, validation.errors);
            showWarning(`プロパティの値が範囲外です: ${validation.errors[0]}`);
            // エラーでも設定は行う（ユーザーが修正できるように）
        }
    }

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    return true;
}

/**
 * オブジェクトの複数プロパティをまとめて設定する
 *
 * @param {string} rectangleId - 四角形のID
 * @param {Object} properties - 設定するプロパティのオブジェクト
 * @param {boolean} [skipValidation=false] - 検証をスキップする場合はtrue
 * @returns {boolean} 成功した場合はtrue
 *
 * @example
 * setObjectProperties('rect-0', { shelfLevels: 5, shelfDividers: 2 });
 */
export function setObjectProperties(rectangleId, properties, skipValidation = false) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`setObjectProperties: 四角形が見つかりません: ${rectangleId}`);
        showError('四角形が見つかりません');
        return false;
    }

    // オブジェクトプロパティが存在しない場合は初期化
    if (!rectangle.objectProperties) {
        rectangle.objectProperties = {};
    }

    // プロパティをマージ
    Object.assign(rectangle.objectProperties, properties);

    // 検証（オプション）
    if (!skipValidation && rectangle.objectType !== OBJECT_TYPES.NONE) {
        const validation = validateProperties(rectangle.objectType, rectangle.objectProperties);
        if (!validation.valid) {
            console.warn(`setObjectProperties: 検証エラー:`, validation.errors);
            showWarning(`一部のプロパティが範囲外です: ${validation.errors[0]}`);
            // エラーでも設定は行う
        }
    }

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    return true;
}

/**
 * オブジェクトのプロパティを取得する
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {Object|null} プロパティオブジェクト、見つからない場合はnull
 *
 * @example
 * const props = getObjectProperties('rect-0');
 */
export function getObjectProperties(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`getObjectProperties: 四角形が見つかりません: ${rectangleId}`);
        return null;
    }

    return rectangle.objectProperties || {};
}

/**
 * オブジェクトプロパティをリセット（デフォルト値に戻す）
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {boolean} 成功した場合はtrue
 *
 * @example
 * resetObjectProperties('rect-0');
 */
export function resetObjectProperties(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`resetObjectProperties: 四角形が見つかりません: ${rectangleId}`);
        showError('四角形が見つかりません');
        return false;
    }

    // デフォルトプロパティで初期化
    rectangle.objectProperties = getDefaultProperties(rectangle.objectType);

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    showSuccess('プロパティをリセットしました');
    return true;
}

// ================
// 3D情報管理
// ================

/**
 * オブジェクトの高さ（メートル）を設定する
 *
 * @param {string} rectangleId - 四角形のID
 * @param {number} heightMeters - 高さ（メートル）
 * @returns {boolean} 成功した場合はtrue
 *
 * @example
 * setHeightMeters('rect-0', 1.5);
 */
export function setHeightMeters(rectangleId, heightMeters) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`setHeightMeters: 四角形が見つかりません: ${rectangleId}`);
        showError('四角形が見つかりません');
        return false;
    }

    // 検証
    const validation = validate3DProperties(heightMeters, rectangle.frontDirection || 'top');
    if (!validation.valid) {
        console.error(`setHeightMeters: 検証エラー:`, validation.errors);
        showError(validation.errors[0]);
        return false;
    }

    rectangle.heightMeters = heightMeters;

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    // 3Dビューを更新
    if (window.update3DObject && typeof window.update3DObject === 'function') {
        window.update3DObject(rectangleId);
    }

    return true;
}

/**
 * オブジェクトの前面方向を設定する
 *
 * @param {string} rectangleId - 四角形のID
 * @param {string} frontDirection - 前面方向（'top' | 'right' | 'bottom' | 'left'）
 * @returns {boolean} 成功した場合はtrue
 *
 * @example
 * setFrontDirection('rect-0', 'right');
 */
export function setFrontDirection(rectangleId, frontDirection) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`setFrontDirection: 四角形が見つかりません: ${rectangleId}`);
        showError('四角形が見つかりません');
        return false;
    }

    // 検証
    const validation = validate3DProperties(rectangle.heightMeters || 0.5, frontDirection);
    if (!validation.valid) {
        console.error(`setFrontDirection: 検証エラー:`, validation.errors);
        showError(validation.errors[0]);
        return false;
    }

    rectangle.frontDirection = frontDirection;

    // 履歴に保存
    if (window.saveToHistory && typeof window.saveToHistory === 'function') {
        window.saveToHistory();
    }

    // 3Dビューを更新
    if (window.update3DObject && typeof window.update3DObject === 'function') {
        window.update3DObject(rectangleId);
    }

    // レイヤーを再描画（前面方向の矢印表示のため）
    if (window.redrawRectangleLayer && typeof window.redrawRectangleLayer === 'function') {
        const rectangleLayer = mapState.layerStack.find(l => l.type === 'rectangle');
        if (rectangleLayer) {
            window.redrawRectangleLayer(rectangleLayer);
        }
    }

    return true;
}

// ================
// ユーティリティ関数
// ================

/**
 * 座標変換: ピクセル → メートル
 *
 * @param {number} pixels - ピクセル数
 * @returns {number} メートル値
 */
export function pixelsToMeters(pixels) {
    const resolution = mapState.metadata?.resolution || 0.05; // m/pixel
    return pixels * resolution;
}

/**
 * 座標変換: メートル → ピクセル
 *
 * @param {number} meters - メートル値
 * @returns {number} ピクセル数
 */
export function metersToPixels(meters) {
    const resolution = mapState.metadata?.resolution || 0.05;
    return meters / resolution;
}

/**
 * 2D四角形から3D座標情報を取得
 *
 * @param {string} rectangleId - 四角形のID
 * @returns {Object|null} 3D座標情報 { x, y, z, width, depth, height }
 *
 * @example
 * const coords3D = get3DCoordinates('rect-0');
 * // { x: 5.0, y: 3.0, z: 0.25, width: 1.0, depth: 0.8, height: 0.5 }
 */
export function get3DCoordinates(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        console.error(`get3DCoordinates: 四角形が見つかりません: ${rectangleId}`);
        return null;
    }

    // マップの解像度と原点を取得
    const resolution = mapState.metadata?.resolution || 0.05; // m/pixel
    // metadata.originは配列形式 [x, y, theta]
    const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
    const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

    // 画像の高さ（Y軸反転のため必要）
    const imageHeight = mapState.image?.height || 0;

    // 画像ピクセル座標を実世界座標に変換
    // ROSマップ: 画像の左下が原点、Y軸が上向き
    // 画像座標: 左上が原点、Y軸が下向き
    const worldX = originX + rectangle.x * resolution;
    const worldY = originY + (imageHeight - rectangle.y) * resolution;

    return {
        x: worldX,
        y: worldY,
        z: (rectangle.heightMeters || 0.5) / 2,  // 高さの半分（底面からの中心）
        width: pixelsToMeters(rectangle.width),
        depth: pixelsToMeters(rectangle.height),
        height: rectangle.heightMeters || 0.5,
        rotation: rectangle.rotation || 0,
        frontDirection: rectangle.frontDirection || 'top'
    };
}

/**
 * すべての四角形の3D座標情報を取得
 *
 * @returns {Array<Object>} 3D座標情報の配列
 */
export function getAll3DCoordinates() {
    const rectangles = mapState.rectangleToolState.rectangles || [];
    return rectangles.map(rect => ({
        id: rect.id,
        objectType: rect.objectType,
        coordinates: get3DCoordinates(rect.id),
        properties: rect.objectProperties
    }));
}
