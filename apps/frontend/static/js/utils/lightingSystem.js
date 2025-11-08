/**
 * @file lightingSystem.js
 * @description 3D描画のための光源・シェーディングシステム
 *
 * 物理ベースの簡易ライティングを提供し、より立体的で実在感のある3D表現を実現します。
 * Lambert's cosine lawを使用した拡散反射計算と、Ambient Occlusion風のエッジ強調を実装。
 *
 * @exports LightingSystem - 光源システムクラス
 * @exports createDefaultLighting - デフォルト光源設定の生成
 * @exports calculateFaceBrightness - 面の明度計算
 * @exports applyLightingToColor - 色に光源効果を適用
 */

// ================
// 定数
// ================

/**
 * デフォルト光源設定
 */
const DEFAULT_LIGHTING_CONFIG = {
    // 光源方向（正規化ベクトル）- 左上前方から照らす
    lightDirection: { x: -0.5, y: -0.5, z: 0.7071 },  // 正規化済み

    // 光の強度
    ambient: 0.35,        // 環境光（0.0-1.0）- 最低限の明るさを保証
    diffuse: 0.65,        // 拡散光（0.0-1.0）- 面の向きによる明暗

    // エッジ強調
    edgeDarkening: 0.12,  // エッジの暗さ（0.0-0.5）- Ambient Occlusion風
    cornerDarkening: 0.08, // 角部分の追加の暗さ

    // 材質プロパティ
    materialShininess: 0.0,  // 光沢度（0.0=マット、1.0=光沢）

    // 深度減衰（遠くのオブジェクトを暗く）
    depthAttenuation: 0.0    // 0.0=なし、1.0=最大
};

/**
 * 各面の法線ベクトル（3D空間）
 */
const FACE_NORMALS = {
    top: { x: 0, y: 0, z: 1 },      // 上面
    bottom: { x: 0, y: 0, z: -1 },   // 底面
    front: { x: 0, y: -1, z: 0 },    // 前面（Y負方向）
    back: { x: 0, y: 1, z: 0 },      // 背面（Y正方向）
    left: { x: -1, y: 0, z: 0 },     // 左面（X負方向）
    right: { x: 1, y: 0, z: 0 }      // 右面（X正方向）
};

// ================
// ユーティリティ関数
// ================

/**
 * ベクトルの内積を計算
 *
 * @private
 * @param {Object} v1 - ベクトル1 {x, y, z}
 * @param {Object} v2 - ベクトル2 {x, y, z}
 * @returns {number} 内積
 */
function dotProduct(v1, v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * ベクトルを正規化
 *
 * @private
 * @param {Object} v - ベクトル {x, y, z}
 * @returns {Object} 正規化されたベクトル
 */
function normalize(v) {
    const length = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length === 0) return { x: 0, y: 0, z: 0 };
    return {
        x: v.x / length,
        y: v.y / length,
        z: v.z / length
    };
}

/**
 * HEX色をRGB配列に変換
 *
 * @private
 * @param {string} hex - HEX色 (#RRGGBB)
 * @returns {Object} RGB {r, g, b}
 */
function hexToRgb(hex) {
    const num = parseInt(hex.replace("#", ""), 16);
    return {
        r: (num >> 16) & 0xFF,
        g: (num >> 8) & 0xFF,
        b: num & 0xFF
    };
}

/**
 * RGB配列をHEX色に変換
 *
 * @private
 * @param {number} r - 赤 (0-255)
 * @param {number} g - 緑 (0-255)
 * @param {number} b - 青 (0-255)
 * @returns {string} HEX色
 */
function rgbToHex(r, g, b) {
    r = Math.max(0, Math.min(255, Math.round(r)));
    g = Math.max(0, Math.min(255, Math.round(g)));
    b = Math.max(0, Math.min(255, Math.round(b)));
    return "#" + ((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1);
}

// ================
// 光源システムクラス
// ================

/**
 * 光源システムクラス
 *
 * @class
 */
export class LightingSystem {
    /**
     * コンストラクタ
     *
     * @param {Object} config - 光源設定（オプション）
     */
    constructor(config = {}) {
        this.config = { ...DEFAULT_LIGHTING_CONFIG, ...config };

        // 光源方向を正規化
        this.config.lightDirection = normalize(this.config.lightDirection);
    }

    /**
     * 設定を更新
     *
     * @param {Object} updates - 更新する設定項目
     */
    updateConfig(updates) {
        this.config = { ...this.config, ...updates };

        // 光源方向が更新された場合は正規化
        if (updates.lightDirection) {
            this.config.lightDirection = normalize(this.config.lightDirection);
        }
    }

    /**
     * 面の明度を計算（Lambert shading）
     *
     * @param {string} faceName - 面の名前 ('top', 'bottom', 'front', 'back', 'left', 'right')
     * @param {Object} options - オプション設定
     * @param {boolean} options.isEdge - エッジかどうか
     * @param {boolean} options.isCorner - 角部分かどうか
     * @param {number} options.depth - 深度（Z座標）
     * @returns {number} 明度 (0.0-1.0)
     */
    calculateFaceBrightness(faceName, options = {}) {
        const normal = FACE_NORMALS[faceName];
        if (!normal) {
            console.warn(`Unknown face name: ${faceName}`);
            return 1.0;
        }

        // Lambert's cosine law: I = ambient + diffuse * max(0, N·L)
        const cosTheta = Math.max(0, dotProduct(normal, this.config.lightDirection));
        let brightness = this.config.ambient + this.config.diffuse * cosTheta;

        // エッジ強調（Ambient Occlusion風）
        if (options.isEdge) {
            brightness *= (1 - this.config.edgeDarkening);
        }

        // 角部分の追加の暗さ
        if (options.isCorner) {
            brightness *= (1 - this.config.cornerDarkening);
        }

        // 深度減衰（オプション）
        if (options.depth !== undefined && this.config.depthAttenuation > 0) {
            const depthFactor = 1 - (options.depth * this.config.depthAttenuation * 0.1);
            brightness *= Math.max(0.5, depthFactor);
        }

        // クランプ
        return Math.max(0.1, Math.min(1.0, brightness));
    }

    /**
     * 色に光源効果を適用
     *
     * @param {string} baseColor - ベース色 (HEX)
     * @param {number} brightness - 明度 (0.0-1.0)
     * @returns {string} 光源効果適用後の色 (HEX)
     */
    applyLightingToColor(baseColor, brightness) {
        const rgb = hexToRgb(baseColor);

        const r = rgb.r * brightness;
        const g = rgb.g * brightness;
        const b = rgb.b * brightness;

        return rgbToHex(r, g, b);
    }

    /**
     * 2つの面の境界（エッジ）の明度を計算
     *
     * @param {string} face1 - 面1の名前
     * @param {string} face2 - 面2の名前
     * @returns {number} エッジの明度
     */
    calculateEdgeBrightness(face1, face2) {
        const b1 = this.calculateFaceBrightness(face1);
        const b2 = this.calculateFaceBrightness(face2);

        // 2つの面の平均明度からエッジ強調分を引く
        const avgBrightness = (b1 + b2) / 2;
        return avgBrightness * (1 - this.config.edgeDarkening);
    }

    /**
     * 面の色を計算（明度を適用）
     *
     * @param {string} baseColor - ベース色 (HEX)
     * @param {string} faceName - 面の名前
     * @param {Object} options - オプション設定
     * @returns {string} 最終的な面の色 (HEX)
     */
    getFaceColor(baseColor, faceName, options = {}) {
        const brightness = this.calculateFaceBrightness(faceName, options);
        return this.applyLightingToColor(baseColor, brightness);
    }

    /**
     * ストローク（境界線）の色を計算
     *
     * @param {string} baseColor - ベース色 (HEX)
     * @param {string} faceName - 面の名前
     * @returns {string} ストロークの色 (HEX)
     */
    getStrokeColor(baseColor, faceName) {
        // エッジは常に暗め
        const brightness = this.calculateFaceBrightness(faceName, { isEdge: true });
        return this.applyLightingToColor(baseColor, brightness * 0.7);
    }

    /**
     * 現在の設定を取得
     *
     * @returns {Object} 光源設定
     */
    getConfig() {
        return { ...this.config };
    }
}

// ================
// エクスポート関数
// ================

/**
 * デフォルト光源設定を生成
 *
 * @returns {LightingSystem} 光源システムインスタンス
 *
 * @example
 * const lighting = createDefaultLighting();
 * const topColor = lighting.getFaceColor('#667eea', 'top');
 */
export function createDefaultLighting() {
    return new LightingSystem();
}

/**
 * 面の明度を計算（スタンドアロン関数）
 *
 * @param {string} faceName - 面の名前
 * @param {Object} lightingConfig - 光源設定（オプション）
 * @returns {number} 明度 (0.0-1.0)
 *
 * @example
 * const brightness = calculateFaceBrightness('top');
 */
export function calculateFaceBrightness(faceName, lightingConfig = {}) {
    const lighting = new LightingSystem(lightingConfig);
    return lighting.calculateFaceBrightness(faceName);
}

/**
 * 色に光源効果を適用（スタンドアロン関数）
 *
 * @param {string} baseColor - ベース色 (HEX)
 * @param {number} brightness - 明度 (0.0-1.0)
 * @returns {string} 光源効果適用後の色 (HEX)
 *
 * @example
 * const litColor = applyLightingToColor('#667eea', 0.8);
 */
export function applyLightingToColor(baseColor, brightness) {
    const lighting = new LightingSystem();
    return lighting.applyLightingToColor(baseColor, brightness);
}
