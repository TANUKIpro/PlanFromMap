/**
 * @file config.js
 * @description アプリケーション全体の設定と定数を管理
 *
 * このファイルには、APIエンドポイント、デフォルト値、制限値などの
 * アプリケーション全体で使用される定数が定義されています。
 *
 * @exports API_BASE_URL - APIサーバーのベースURL
 * @exports CANVAS_DEFAULTS - Canvas関連のデフォルト設定
 * @exports DRAWING_DEFAULTS - 描画ツールのデフォルト設定
 * @exports HISTORY_DEFAULTS - 履歴管理のデフォルト設定
 */

// ================
// API設定
// ================

/**
 * APIサーバーのベースURL
 * @type {string}
 */
export const API_BASE_URL = 'http://localhost:3000/api';

// ================
// Canvas設定
// ================

/**
 * Canvas関連のデフォルト設定
 * @type {Object}
 */
export const CANVAS_DEFAULTS = {
    /** 最小スケール（縮小率） */
    MIN_SCALE: 0.1,

    /** 最大スケール（拡大率） */
    MAX_SCALE: 10.0,

    /** 初期スケール */
    INITIAL_SCALE: 1.0,

    /** ズーム倍率 */
    ZOOM_FACTOR: 1.1,

    /** 初期オフセット X */
    INITIAL_OFFSET_X: 0,

    /** 初期オフセット Y */
    INITIAL_OFFSET_Y: 0
};

// ================
// 描画ツール設定
// ================

/**
 * 描画ツールのデフォルト設定
 * @type {Object}
 */
export const DRAWING_DEFAULTS = {
    /** デフォルトツール */
    DEFAULT_TOOL: 'pan',

    /** デフォルトカラー */
    DEFAULT_COLOR: '#FF0000',

    /** デフォルトブラシサイズ */
    DEFAULT_BRUSH_SIZE: 5,

    /** ブラシサイズの最小値 */
    MIN_BRUSH_SIZE: 1,

    /** ブラシサイズの最大値 */
    MAX_BRUSH_SIZE: 50,

    /** 利用可能なツール */
    AVAILABLE_TOOLS: ['pan', 'pencil', 'eraser', 'measure', 'bucket']
};

// ================
// レイヤー設定
// ================

/**
 * レイヤー関連のデフォルト設定
 * @type {Object}
 */
export const LAYER_DEFAULTS = {
    /** デフォルトレイヤータイプ */
    DEFAULT_TYPE: 'drawing',

    /** デフォルトの可視性 */
    DEFAULT_VISIBLE: true,

    /** デフォルトの不透明度 */
    DEFAULT_OPACITY: 1.0,

    /** レイヤータイプの定義 */
    TYPES: {
        IMAGE: 'image',
        METADATA: 'metadata',
        DRAWING: 'drawing'
    }
};

// ================
// メタデータオーバーレイ設定
// ================

/**
 * メタデータオーバーレイのデフォルト設定
 * @type {Object}
 */
export const OVERLAY_DEFAULTS = {
    /** グリッドを表示するか */
    SHOW_GRID: true,

    /** グリッド間隔（メートル） */
    GRID_SPACING_METERS: 1,

    /** 原点マーカーを表示するか */
    SHOW_ORIGIN: true,

    /** スケールバーを表示するか */
    SHOW_SCALE_BAR: true
};

// ================
// 履歴管理設定
// ================

/**
 * 履歴管理のデフォルト設定
 * @type {Object}
 */
export const HISTORY_DEFAULTS = {
    /** 最大履歴数 */
    MAX_HISTORY: 50
};

// ================
// UI設定
// ================

/**
 * UI関連の設定
 * @type {Object}
 */
export const UI_DEFAULTS = {
    /** デフォルトのタブID */
    DEFAULT_TAB: 'mapViewer',

    /** 利用可能なタブ */
    AVAILABLE_TABS: ['mapViewer', 'stats', 'operations', 'mapql']
};
