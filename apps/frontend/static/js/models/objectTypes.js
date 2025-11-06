/**
 * @file objectTypes.js
 * @description オブジェクトタイプとデフォルトプロパティの定義
 *
 * このモジュールは、2Dマップ上の四角形に割り当て可能なオブジェクトタイプ（家具やその他の構造物）の
 * 定義と、各タイプのデフォルトプロパティを管理します。
 *
 * @exports OBJECT_TYPES - オブジェクトタイプ定数
 * @exports OBJECT_TYPE_LABELS - 日本語ラベル
 * @exports DEFAULT_PROPERTIES - デフォルトプロパティ
 * @exports PROPERTY_SCHEMAS - プロパティスキーマ（UI生成用）
 * @exports getDefaultProperties - デフォルトプロパティ取得関数
 * @exports validateProperties - プロパティ検証関数
 * @exports getPropertySchema - プロパティスキーマ取得関数
 */

// ================
// オブジェクトタイプ定数
// ================

/**
 * オブジェクトタイプの定数定義
 */
export const OBJECT_TYPES = {
    NONE: 'none',
    SHELF: 'shelf',
    BOX: 'box',
    TABLE: 'table',
    DOOR: 'door',
    WALL: 'wall'
};

/**
 * オブジェクトタイプの日本語ラベル
 */
export const OBJECT_TYPE_LABELS = {
    none: 'なし',
    shelf: '棚',
    box: '箱',
    table: 'テーブル',
    door: '扉',
    wall: '壁'
};

/**
 * オブジェクトタイプの表示色（3D表示用）
 */
export const OBJECT_TYPE_COLORS = {
    none: '#667eea',      // デフォルト（紫）
    shelf: '#48bb78',     // 緑
    box: '#ed8936',       // オレンジ
    table: '#4299e1',     // 青
    door: '#9f7aea',      // 紫
    wall: '#718096'       // グレー
};

// ================
// デフォルトプロパティ
// ================

/**
 * 各オブジェクトタイプのデフォルトプロパティ
 */
export const DEFAULT_PROPERTIES = {
    none: {},

    shelf: {
        shelfLevels: 3,              // 段数（1-10）
        shelfDividers: 0,            // 仕切り数（0-20）
        shelfType: 'open'            // 'open' | 'closed' | 'glass-door'
    },

    box: {
        boxHasLid: true,             // 蓋の有無
        boxStackable: true,          // 積み重ね可能性
        boxMaterial: 'cardboard'     // 'cardboard' | 'plastic' | 'wood' | 'metal'
    },

    table: {
        tableShape: 'rectangular',   // 'rectangular' | 'circular' | 'oval'
        tableLegCount: 4,            // 脚の数（0=壁掛け, 1, 2, 4, etc.）
        tableHasDrawer: false        // 引き出しの有無
    },

    door: {
        doorOpenDirection: 'push',   // 'push' | 'pull' | 'slide-left' | 'slide-right'
        doorSwingDirection: 'inward',// 'inward' | 'outward' | 'both'
        doorType: 'single'           // 'single' | 'double' | 'revolving'
    },

    wall: {
        wallMaterial: 'drywall',     // 'concrete' | 'wood' | 'glass' | 'drywall' | 'brick'
        wallThicknessCm: 10,         // 厚さ（cm）
        wallHasWindow: false         // 窓の有無
    }
};

/**
 * デフォルトの3D情報
 */
export const DEFAULT_3D_PROPERTIES = {
    heightMeters: 0.5,               // 高さ（メートル）
    frontDirection: 'top'            // 前面方向 'top' | 'right' | 'bottom' | 'left'
};

/**
 * 共通の拡張プロパティ（全オブジェクト共通）
 */
export const DEFAULT_COMMON_PROPERTIES = {
    // 識別・メタデータ
    name: '',                        // オブジェクト名（ユーザー定義）
    description: '',                 // 説明・メモ
    tags: [],                        // タグ（検索・フィルタ用）
    customColor: null,               // カスタム色（nullの場合はデフォルト色を使用）

    // ロボット操作関連
    isAccessible: true,              // ロボットがアクセス可能か
    isMovable: false,                // 移動可能か
    weightKg: 0,                     // 重量（kg）

    // 物理的特性
    material: '',                    // 材質の詳細説明
    surfaceType: 'smooth',           // 表面タイプ（smooth/rough/fragile）

    // センサー・ナビゲーション情報
    hasObstacle: false,              // 障害物としてマークするか
    isNoGoZone: false,               // 進入禁止エリアか

    // 時間情報（自動設定）
    createdAt: null,                 // 作成日時（ISO 8601文字列）
    updatedAt: null                  // 更新日時（ISO 8601文字列）
};

/**
 * 共通プロパティのスキーマ（UI生成用）
 */
export const COMMON_PROPERTY_SCHEMA = [
    {
        section: '基本情報',
        fields: [
            {
                key: 'name',
                label: 'オブジェクト名',
                type: 'text',
                placeholder: '例: キッチンの棚',
                maxLength: 100
            },
            {
                key: 'description',
                label: '説明・メモ',
                type: 'textarea',
                placeholder: 'オブジェクトの詳細説明やメモを入力',
                rows: 3,
                maxLength: 500
            },
            {
                key: 'tags',
                label: 'タグ',
                type: 'tags',
                placeholder: 'タグを入力してEnter',
                hint: 'カンマまたはEnterで区切って複数入力可能'
            },
            {
                key: 'customColor',
                label: 'カスタム色',
                type: 'color',
                hint: '空欄の場合はカテゴリのデフォルト色を使用'
            }
        ]
    },
    {
        section: 'ロボット操作',
        fields: [
            {
                key: 'isAccessible',
                label: 'アクセス可能',
                type: 'checkbox',
                hint: 'ロボットがこのオブジェクトにアクセスできるか'
            },
            {
                key: 'isMovable',
                label: '移動可能',
                type: 'checkbox',
                hint: 'ロボットがこのオブジェクトを移動できるか'
            },
            {
                key: 'weightKg',
                label: '重量 (kg)',
                type: 'number',
                min: 0,
                max: 1000,
                step: 0.1,
                hint: '移動時の参考情報'
            }
        ]
    },
    {
        section: '物理的特性',
        fields: [
            {
                key: 'material',
                label: '材質',
                type: 'text',
                placeholder: '例: 木材、金属、プラスチック',
                maxLength: 50
            },
            {
                key: 'surfaceType',
                label: '表面タイプ',
                type: 'select',
                options: [
                    { value: 'smooth', label: 'スムーズ（滑らか）' },
                    { value: 'rough', label: 'ラフ（粗い）' },
                    { value: 'fragile', label: 'フラジャイル（壊れやすい）' }
                ]
            }
        ]
    },
    {
        section: 'ナビゲーション',
        fields: [
            {
                key: 'hasObstacle',
                label: '障害物として扱う',
                type: 'checkbox',
                hint: 'ロボットの経路計画で障害物として認識'
            },
            {
                key: 'isNoGoZone',
                label: '進入禁止エリア',
                type: 'checkbox',
                hint: 'ロボットがこのエリアに進入できないようにする'
            }
        ]
    }
];

// ================
// プロパティスキーマ（UI生成用）
// ================

/**
 * 各プロパティの入力UI定義
 */
export const PROPERTY_SCHEMAS = {
    shelf: [
        {
            key: 'shelfLevels',
            label: '段数',
            type: 'number',
            min: 1,
            max: 10,
            step: 1,
            hasSlider: true
        },
        {
            key: 'shelfDividers',
            label: '仕切り数',
            type: 'number',
            min: 0,
            max: 20,
            step: 1,
            hasSlider: true
        },
        {
            key: 'shelfType',
            label: '棚のタイプ',
            type: 'radio',
            options: [
                { value: 'open', label: 'オープン' },
                { value: 'closed', label: 'クローズド' },
                { value: 'glass-door', label: 'ガラス扉' }
            ]
        }
    ],

    box: [
        {
            key: 'boxHasLid',
            label: '蓋の有無',
            type: 'checkbox'
        },
        {
            key: 'boxStackable',
            label: '積み重ね可能',
            type: 'checkbox'
        },
        {
            key: 'boxMaterial',
            label: '材質',
            type: 'select',
            options: [
                { value: 'cardboard', label: '段ボール' },
                { value: 'plastic', label: 'プラスチック' },
                { value: 'wood', label: '木材' },
                { value: 'metal', label: '金属' }
            ]
        }
    ],

    table: [
        {
            key: 'tableShape',
            label: '天板形状',
            type: 'select',
            options: [
                { value: 'rectangular', label: '長方形' },
                { value: 'circular', label: '円形' },
                { value: 'oval', label: '楕円形' }
            ]
        },
        {
            key: 'tableLegCount',
            label: '脚の数',
            type: 'number',
            min: 0,
            max: 8,
            step: 1,
            hint: '0は壁掛けを意味します'
        },
        {
            key: 'tableHasDrawer',
            label: '引き出しの有無',
            type: 'checkbox'
        }
    ],

    door: [
        {
            key: 'doorOpenDirection',
            label: '開閉方向',
            type: 'select',
            options: [
                { value: 'push', label: '押す' },
                { value: 'pull', label: '引く' },
                { value: 'slide-left', label: '左にスライド' },
                { value: 'slide-right', label: '右にスライド' }
            ]
        },
        {
            key: 'doorSwingDirection',
            label: '開く向き',
            type: 'radio',
            options: [
                { value: 'inward', label: '内開き' },
                { value: 'outward', label: '外開き' },
                { value: 'both', label: '両開き' }
            ]
        },
        {
            key: 'doorType',
            label: '扉のタイプ',
            type: 'radio',
            options: [
                { value: 'single', label: 'シングル' },
                { value: 'double', label: 'ダブル' },
                { value: 'revolving', label: '回転式' }
            ]
        }
    ],

    wall: [
        {
            key: 'wallMaterial',
            label: '壁の材質',
            type: 'select',
            options: [
                { value: 'drywall', label: '石膏ボード' },
                { value: 'concrete', label: 'コンクリート' },
                { value: 'wood', label: '木材' },
                { value: 'glass', label: 'ガラス' },
                { value: 'brick', label: 'レンガ' }
            ]
        },
        {
            key: 'wallThicknessCm',
            label: '厚さ (cm)',
            type: 'number',
            min: 1,
            max: 50,
            step: 1
        },
        {
            key: 'wallHasWindow',
            label: '窓の有無',
            type: 'checkbox'
        }
    ]
};

// ================
// ヘルパー関数
// ================

/**
 * 指定されたオブジェクトタイプのデフォルトプロパティを取得
 *
 * @param {string} objectType - オブジェクトタイプ
 * @returns {Object} デフォルトプロパティオブジェクト
 *
 * @example
 * const props = getDefaultProperties('shelf');
 * // { shelfLevels: 3, shelfDividers: 0, shelfType: 'open' }
 */
export function getDefaultProperties(objectType) {
    if (!objectType || objectType === OBJECT_TYPES.NONE) {
        return {};
    }

    const defaults = DEFAULT_PROPERTIES[objectType];
    if (!defaults) {
        console.warn(`getDefaultProperties: 不明なオブジェクトタイプ: ${objectType}`);
        return {};
    }

    // ディープコピーして返す
    return JSON.parse(JSON.stringify(defaults));
}

/**
 * 指定されたオブジェクトタイプのプロパティスキーマを取得
 *
 * @param {string} objectType - オブジェクトタイプ
 * @returns {Array} プロパティスキーマ配列
 */
export function getPropertySchema(objectType) {
    if (!objectType || objectType === OBJECT_TYPES.NONE) {
        return [];
    }

    const schema = PROPERTY_SCHEMAS[objectType];
    if (!schema) {
        console.warn(`getPropertySchema: 不明なオブジェクトタイプ: ${objectType}`);
        return [];
    }

    return schema;
}

/**
 * プロパティの妥当性を検証
 *
 * @param {string} objectType - オブジェクトタイプ
 * @param {Object} properties - 検証するプロパティ
 * @returns {Object} { valid: boolean, errors: string[] }
 *
 * @example
 * const result = validateProperties('shelf', { shelfLevels: 15 });
 * // { valid: false, errors: ['shelfLevels は 1 から 10 の範囲である必要があります'] }
 */
export function validateProperties(objectType, properties) {
    const errors = [];

    if (!objectType || objectType === OBJECT_TYPES.NONE) {
        return { valid: true, errors: [] };
    }

    const schema = PROPERTY_SCHEMAS[objectType];
    if (!schema) {
        return { valid: false, errors: ['不明なオブジェクトタイプです'] };
    }

    // 各プロパティを検証
    schema.forEach(field => {
        const value = properties[field.key];

        if (value === undefined || value === null) {
            return; // オプショナルとして扱う
        }

        // 数値型の検証
        if (field.type === 'number') {
            if (typeof value !== 'number' || isNaN(value)) {
                errors.push(`${field.label} は数値である必要があります`);
                return;
            }

            if (field.min !== undefined && value < field.min) {
                errors.push(`${field.label} は ${field.min} 以上である必要があります`);
            }

            if (field.max !== undefined && value > field.max) {
                errors.push(`${field.label} は ${field.max} 以下である必要があります`);
            }
        }

        // 真偽値型の検証
        if (field.type === 'checkbox') {
            if (typeof value !== 'boolean') {
                errors.push(`${field.label} は真偽値である必要があります`);
            }
        }

        // 選択型の検証
        if (field.type === 'select' || field.type === 'radio') {
            const validValues = field.options.map(opt => opt.value);
            if (!validValues.includes(value)) {
                errors.push(`${field.label} は有効な値である必要があります`);
            }
        }
    });

    return {
        valid: errors.length === 0,
        errors
    };
}

/**
 * 3D情報の妥当性を検証
 *
 * @param {number} heightMeters - 高さ（メートル）
 * @param {string} frontDirection - 前面方向
 * @returns {Object} { valid: boolean, errors: string[] }
 */
export function validate3DProperties(heightMeters, frontDirection) {
    const errors = [];

    // 高さの検証
    if (typeof heightMeters !== 'number' || isNaN(heightMeters)) {
        errors.push('高さは数値である必要があります');
    } else if (heightMeters <= 0) {
        errors.push('高さは0より大きい必要があります');
    } else if (heightMeters > 10) {
        errors.push('高さは10m以下である必要があります');
    }

    // 前面方向の検証
    const validDirections = ['top', 'right', 'bottom', 'left'];
    if (!validDirections.includes(frontDirection)) {
        errors.push('前面方向は top, right, bottom, left のいずれかである必要があります');
    }

    return {
        valid: errors.length === 0,
        errors
    };
}

/**
 * オブジェクトタイプのリストを取得（UI選択用）
 *
 * @returns {Array} { value, label } の配列
 */
export function getObjectTypeOptions() {
    return Object.keys(OBJECT_TYPE_LABELS).map(key => ({
        value: key,
        label: OBJECT_TYPE_LABELS[key]
    }));
}

/**
 * 前面方向のリストを取得（UI選択用）
 *
 * @returns {Array} { value, label } の配列
 */
export function getFrontDirectionOptions() {
    return [
        { value: 'top', label: '上' },
        { value: 'right', label: '右' },
        { value: 'bottom', label: '下' },
        { value: 'left', label: '左' }
    ];
}

/**
 * 共通プロパティのデフォルト値を取得（タイムスタンプ付き）
 *
 * @returns {Object} デフォルトの共通プロパティ
 */
export function getDefaultCommonProperties() {
    const now = new Date().toISOString();
    return {
        ...JSON.parse(JSON.stringify(DEFAULT_COMMON_PROPERTIES)),
        createdAt: now,
        updatedAt: now
    };
}

/**
 * 共通プロパティのスキーマを取得
 *
 * @returns {Array} 共通プロパティスキーマ
 */
export function getCommonPropertySchema() {
    return COMMON_PROPERTY_SCHEMA;
}

/**
 * 共通プロパティの妥当性を検証
 *
 * @param {Object} commonProps - 検証する共通プロパティ
 * @returns {Object} { valid: boolean, errors: string[] }
 */
export function validateCommonProperties(commonProps) {
    const errors = [];

    if (!commonProps) {
        return { valid: false, errors: ['共通プロパティが指定されていません'] };
    }

    // 名前の検証
    if (commonProps.name && commonProps.name.length > 100) {
        errors.push('オブジェクト名は100文字以内である必要があります');
    }

    // 説明の検証
    if (commonProps.description && commonProps.description.length > 500) {
        errors.push('説明は500文字以内である必要があります');
    }

    // タグの検証
    if (commonProps.tags && !Array.isArray(commonProps.tags)) {
        errors.push('タグは配列である必要があります');
    }

    // 重量の検証
    if (commonProps.weightKg !== undefined) {
        if (typeof commonProps.weightKg !== 'number' || isNaN(commonProps.weightKg)) {
            errors.push('重量は数値である必要があります');
        } else if (commonProps.weightKg < 0) {
            errors.push('重量は0以上である必要があります');
        } else if (commonProps.weightKg > 1000) {
            errors.push('重量は1000kg以下である必要があります');
        }
    }

    // 表面タイプの検証
    const validSurfaceTypes = ['smooth', 'rough', 'fragile'];
    if (commonProps.surfaceType && !validSurfaceTypes.includes(commonProps.surfaceType)) {
        errors.push('表面タイプは有効な値である必要があります');
    }

    return {
        valid: errors.length === 0,
        errors
    };
}
