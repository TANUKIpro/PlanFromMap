/**
 * @file threeDUtils.js
 * @description 3D描画のためのユーティリティ関数群
 *
 * 等角投影変換、色操作、深度計算などの純粋な計算関数を提供します。
 *
 * @exports worldToIso - ワールド座標を等角投影座標に変換（メインビュー用）
 * @exports worldToPreviewIso - ワールド座標を等角投影座標に変換（プレビュー用）
 * @exports lightenColor - 色を明るくする
 * @exports darkenColor - 色を暗くする
 * @exports sortFacesByDepth - 面を深度順にソート（Painter's Algorithm用）
 * @exports applyRotation - ローカル座標に回転を適用
 */

/**
 * ワールド座標を等角投影座標に変換（メインビュー用）
 * カメラの中心はマップの有効領域の重心
 *
 * @param {number} x - X座標（メートル）
 * @param {number} y - Y座標（メートル）
 * @param {number} z - Z座標（メートル）
 * @param {Object} state - 3Dビュー状態（rotation, tilt, mapBounds含む）
 * @param {Object} mapState - マップ状態（metadata, image含む）
 * @returns {Object} {x, y} - 等角投影座標
 */
export function worldToIso(x, y, z, state, mapState) {
    // マップの重心を中心とするように座標をオフセット
    let offsetX = x;
    let offsetY = y;

    if (state.mapBounds && mapState?.image) {
        const resolution = mapState.metadata?.resolution || 0.05;
        // metadata.originは配列形式 [x, y, theta]
        const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
        const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;
        const imageHeight = mapState.image.height;

        // 有効領域の中心座標（実世界座標）
        const centerPixelX = state.mapBounds.centerX;
        const centerPixelY = state.mapBounds.centerY;

        const centerWorldX = originX + centerPixelX * resolution;
        const centerWorldY = originY + (imageHeight - centerPixelY) * resolution;

        // 座標を重心からの相対位置に変換
        offsetX = x - centerWorldX;
        offsetY = y - centerWorldY;
    }

    // 等角投影の変換行列
    // 回転を考慮
    const rad = state.rotation * Math.PI / 180;
    const tiltRad = state.tilt * Math.PI / 180;

    const rotX = offsetX * Math.cos(rad) - offsetY * Math.sin(rad);
    const rotY = offsetX * Math.sin(rad) + offsetY * Math.cos(rad);

    return {
        x: -rotX,  // X軸を反転して2Dマップとの整合性を確保
        y: z - rotY * Math.sin(tiltRad)
    };
}

/**
 * プレビュー用等角投影変換
 *
 * @param {number} x - X座標（メートル）
 * @param {number} y - Y座標（メートル）
 * @param {number} z - Z座標（メートル）
 * @param {Object} previewState - プレビュー状態（rotation, tilt含む）
 * @returns {Object} 変換後の2D座標 {x, y}
 */
export function worldToPreviewIso(x, y, z, previewState) {
    const rad = previewState.rotation * Math.PI / 180;
    const tiltRad = previewState.tilt * Math.PI / 180;

    const rotX = x * Math.cos(rad) - y * Math.sin(rad);
    const rotY = x * Math.sin(rad) + y * Math.cos(rad);

    return {
        x: -rotX,  // X軸を反転して2Dマップとの整合性を確保
        y: z - rotY * Math.sin(tiltRad)
    };
}

/**
 * 色を明るくする
 *
 * @param {string} color - 16進数カラーコード（例: "#ff0000"）
 * @param {number} percent - 明るくするパーセンテージ（0-100）
 * @returns {string} 明るくされた色の16進数カラーコード
 */
export function lightenColor(color, percent) {
    const num = parseInt(color.replace("#",""), 16);
    const amt = Math.round(2.55 * percent);
    const R = Math.min(255, (num >> 16) + amt);
    const G = Math.min(255, (num >> 8 & 0x00FF) + amt);
    const B = Math.min(255, (num & 0x0000FF) + amt);
    return "#" + (0x1000000 + R * 0x10000 + G * 0x100 + B).toString(16).slice(1);
}

/**
 * 色を暗くする
 *
 * @param {string} color - 16進数カラーコード（例: "#ff0000"）
 * @param {number} percent - 暗くするパーセンテージ（0-100）
 * @returns {string} 暗くされた色の16進数カラーコード
 */
export function darkenColor(color, percent) {
    const num = parseInt(color.replace("#",""), 16);
    const amt = Math.round(2.55 * percent);
    const R = Math.max(0, (num >> 16) - amt);
    const G = Math.max(0, (num >> 8 & 0x00FF) - amt);
    const B = Math.max(0, (num & 0x0000FF) - amt);
    return "#" + (0x1000000 + R * 0x10000 + G * 0x100 + B).toString(16).slice(1);
}

/**
 * 面の中心点の視点からの深度を計算（Painter's Algorithm用）
 *
 * 視点の位置は回転角度（rotation）と傾き角度（tilt）から決定されます。
 * 深度が大きいほど視点から遠い位置にあります。
 *
 * @param {number} cx - 面の中心X座標（ワールド座標）
 * @param {number} cy - 面の中心Y座標（ワールド座標）
 * @param {number} cz - 面の中心Z座標（ワールド座標）
 * @param {Object} state - 3Dビュー状態（rotation, tiltを含む）またはプレビュー状態
 * @returns {number} 視点からの深度
 */
export function calculateFaceDepth(cx, cy, cz, state) {
    // 回転角度とチルト角度をラジアンに変換
    const rotRad = state.rotation * Math.PI / 180;
    const tiltRad = state.tilt * Math.PI / 180;

    // 視点の位置を計算（球面座標系）
    // 距離は十分大きな値（100メートル）とする
    const viewDistance = 100;
    const viewX = viewDistance * Math.cos(tiltRad) * Math.sin(rotRad);
    const viewY = viewDistance * Math.cos(tiltRad) * Math.cos(rotRad);
    const viewZ = viewDistance * Math.sin(tiltRad);

    // 視点から面の中心までの距離を計算
    const dx = cx - viewX;
    const dy = cy - viewY;
    const dz = cz - viewZ;

    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * 面の配列を深度順にソート（遠い面から近い面へ）
 *
 * Painter's Algorithmを実装するために、視点から遠い面を先に描画します。
 *
 * @param {Array<Object>} faces - 面の配列。各要素は {cx, cy, cz, draw: function} の形式
 * @param {Object} state - 3Dビュー状態（rotation, tiltを含む）またはプレビュー状態
 * @returns {Array<Object>} ソート済みの面の配列
 */
export function sortFacesByDepth(faces, state) {
    return faces.sort((a, b) => {
        const depthA = calculateFaceDepth(a.cx, a.cy, a.cz, state);
        const depthB = calculateFaceDepth(b.cx, b.cy, b.cz, state);
        // 深度が大きい（遠い）方を先に描画
        return depthB - depthA;
    });
}

/**
 * ローカル座標に回転を適用
 *
 * @param {number} localX - ローカルX座標
 * @param {number} localY - ローカルY座標
 * @param {number} rotation - 回転角度（度）
 * @returns {Object} 回転後の座標 {x, y}
 */
export function applyRotation(localX, localY, rotation) {
    if (!rotation) return { x: localX, y: localY };

    // 2D回転行列を適用（Y軸反転を考慮）
    const rad = rotation * Math.PI / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);

    return {
        x: localX * cos - localY * sin,
        y: -(localX * sin + localY * cos)  // Y軸を反転（等角投影の座標系に合わせる）
    };
}
