/**
 * @file mapState.js
 * @description アプリケーション全体の状態管理
 *
 * このファイルは、マップビューアの状態を一元管理します。
 * すべての状態変更は、このモジュールを経由することで、
 * 状態の一貫性とトレーサビリティを確保します。
 *
 * @requires ../config.js - 設定値
 *
 * @exports mapState - グローバル状態オブジェクト
 * @exports getState - 状態の取得
 * @exports updateState - 状態の更新
 * @exports resetState - 状態のリセット
 */

import {
    CANVAS_DEFAULTS,
    DRAWING_DEFAULTS,
    OVERLAY_DEFAULTS,
    HISTORY_DEFAULTS,
    LAYER_DEFAULTS,
    RECTANGLE_DEFAULTS
} from '../config.js';

// ================
// 状態オブジェクト
// ================

/**
 * アプリケーション全体の状態
 * @type {Object}
 */
export const mapState = {
    /** 読み込まれた画像データ */
    image: null,

    /** 現在読み込んでいる画像ファイル名 */
    imageFileName: null,

    /** 現在読み込んでいるYAMLファイル名 */
    yamlFileName: null,

    /** 現在のスケール（拡大率） */
    scale: CANVAS_DEFAULTS.INITIAL_SCALE,

    /** X軸オフセット */
    offsetX: CANVAS_DEFAULTS.INITIAL_OFFSET_X,

    /** Y軸オフセット */
    offsetY: CANVAS_DEFAULTS.INITIAL_OFFSET_Y,

    /** パン操作中かどうか */
    isPanning: false,

    /** 最後のマウスX座標 */
    lastX: 0,

    /** 最後のマウスY座標 */
    lastY: 0,

    /** 最小スケール */
    minScale: CANVAS_DEFAULTS.MIN_SCALE,

    /** 最大スケール */
    maxScale: CANVAS_DEFAULTS.MAX_SCALE,

    /** YAMLから読み込まれたメタデータ */
    metadata: null,

    /** レイヤーの可視性設定 */
    layers: {
        image: LAYER_DEFAULTS.DEFAULT_VISIBLE,
        metadataOverlay: LAYER_DEFAULTS.DEFAULT_VISIBLE
    },

    /** オーバーレイ表示設定 */
    overlaySettings: {
        showGrid: OVERLAY_DEFAULTS.SHOW_GRID,
        gridSpacingMeters: OVERLAY_DEFAULTS.GRID_SPACING_METERS,
        showOrigin: OVERLAY_DEFAULTS.SHOW_ORIGIN,
        showScaleBar: OVERLAY_DEFAULTS.SHOW_SCALE_BAR
    },

    /** 新しいレイヤー管理システム */
    layerStack: [],  // レイヤーの配列（下から上の順）

    /** 次のレイヤーID */
    nextLayerId: 1,

    /** 選択中のレイヤーID */
    selectedLayerId: null,

    /** 描画ツールの状態 */
    drawingState: {
        currentTool: DRAWING_DEFAULTS.DEFAULT_TOOL,
        color: DRAWING_DEFAULTS.DEFAULT_COLOR,
        brushSize: DRAWING_DEFAULTS.DEFAULT_BRUSH_SIZE,
        isDrawing: false,
        currentStroke: [],
        measurePoints: []
    },

    /** 四角形ツールの状態 */
    rectangleToolState: {
        enabled: false,              // 四角形ツールのオン/オフ
        rectangles: [],              // 四角形の配列
        selectedRectangleId: null,   // 選択中の四角形ID
        nextRectangleId: 1,          // 次の四角形ID
        editMode: null,              // 編集モード: 'resize', 'move', 'rotate', 'measure', null
        resizeEdge: null,            // リサイズ中の辺: 'top', 'right', 'bottom', 'left', null
        isDragging: false,           // ドラッグ中かどうか
        dragStartPos: null,          // ドラッグ開始位置
        measureEdge: null,           // 測量中の辺
        hoverRectangleId: null,      // ホバー中の四角形ID
        hoverEdge: null              // ホバー中の辺
    },

    /** アンドゥ/リドゥ用の履歴 */
    history: {
        past: [],
        future: [],
        maxHistory: HISTORY_DEFAULTS.MAX_HISTORY
    }
};

// ================
// 状態管理関数
// ================

/**
 * 状態の一部を取得する
 *
 * @param {string} key - 取得する状態のキー
 * @returns {*} 指定されたキーの状態値
 *
 * @example
 * const scale = getState('scale');
 * const layers = getState('layers');
 */
export function getState(key) {
    return mapState[key];
}

/**
 * 状態を更新する
 *
 * @param {Object} updates - 更新する状態のキーと値のペア
 * @returns {void}
 *
 * @example
 * updateState({ scale: 2.0, offsetX: 100 });
 * updateState({ 'drawingState.currentTool': 'pencil' });
 */
export function updateState(updates) {
    Object.entries(updates).forEach(([key, value]) => {
        // ネストされたキーをサポート（例: 'drawingState.currentTool'）
        if (key.includes('.')) {
            const keys = key.split('.');
            let target = mapState;

            for (let i = 0; i < keys.length - 1; i++) {
                target = target[keys[i]];
            }

            target[keys[keys.length - 1]] = value;
        } else {
            mapState[key] = value;
        }
    });
}

/**
 * 状態を初期値にリセットする
 *
 * @returns {void}
 *
 * @example
 * resetState();
 */
export function resetState() {
    mapState.image = null;
    mapState.imageFileName = null;
    mapState.yamlFileName = null;
    mapState.scale = CANVAS_DEFAULTS.INITIAL_SCALE;
    mapState.offsetX = CANVAS_DEFAULTS.INITIAL_OFFSET_X;
    mapState.offsetY = CANVAS_DEFAULTS.INITIAL_OFFSET_Y;
    mapState.isPanning = false;
    mapState.lastX = 0;
    mapState.lastY = 0;
    mapState.metadata = null;

    mapState.layers = {
        image: LAYER_DEFAULTS.DEFAULT_VISIBLE,
        metadataOverlay: LAYER_DEFAULTS.DEFAULT_VISIBLE
    };

    mapState.overlaySettings = {
        showGrid: OVERLAY_DEFAULTS.SHOW_GRID,
        gridSpacingMeters: OVERLAY_DEFAULTS.GRID_SPACING_METERS,
        showOrigin: OVERLAY_DEFAULTS.SHOW_ORIGIN,
        showScaleBar: OVERLAY_DEFAULTS.SHOW_SCALE_BAR
    };

    mapState.layerStack = [];
    mapState.nextLayerId = 1;
    mapState.selectedLayerId = null;

    mapState.drawingState = {
        currentTool: DRAWING_DEFAULTS.DEFAULT_TOOL,
        color: DRAWING_DEFAULTS.DEFAULT_COLOR,
        brushSize: DRAWING_DEFAULTS.DEFAULT_BRUSH_SIZE,
        isDrawing: false,
        currentStroke: [],
        measurePoints: []
    };

    mapState.rectangleToolState = {
        enabled: false,
        rectangles: [],
        selectedRectangleId: null,
        nextRectangleId: 1,
        editMode: null,
        resizeEdge: null,
        isDragging: false,
        dragStartPos: null,
        measureEdge: null,
        hoverRectangleId: null,
        hoverEdge: null
    };

    mapState.history = {
        past: [],
        future: [],
        maxHistory: HISTORY_DEFAULTS.MAX_HISTORY
    };
}

/**
 * レイヤースタックにレイヤーを追加する
 *
 * @param {Object} layer - 追加するレイヤーオブジェクト
 * @returns {void}
 */
export function addLayerToStack(layer) {
    mapState.layerStack.push(layer);
}

/**
 * レイヤースタックからレイヤーを削除する
 *
 * @param {string} layerId - 削除するレイヤーのID
 * @returns {boolean} 削除に成功したかどうか
 */
export function removeLayerFromStack(layerId) {
    const index = mapState.layerStack.findIndex(l => l.id === layerId);
    if (index !== -1) {
        mapState.layerStack.splice(index, 1);
        return true;
    }
    return false;
}

/**
 * レイヤーIDでレイヤーを取得する
 *
 * @param {string} layerId - 取得するレイヤーのID
 * @returns {Object|null} レイヤーオブジェクト、見つからない場合はnull
 */
export function getLayerById(layerId) {
    return mapState.layerStack.find(l => l.id === layerId) || null;
}

/**
 * 選択中のレイヤーを取得する
 *
 * @returns {Object|null} 選択中のレイヤーオブジェクト、なければnull
 */
export function getSelectedLayer() {
    return getLayerById(mapState.selectedLayerId);
}

/**
 * 次のレイヤーIDを生成して返す
 *
 * @returns {number} 新しいレイヤーID
 */
export function getNextLayerId() {
    return mapState.nextLayerId++;
}
