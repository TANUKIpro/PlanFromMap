/**
 * @file renderingSortUtils.js
 * @description 3Dレンダリングにおける面のソートと可視性判定
 *
 * 3Dモデルの面を正しく描画するためのユーティリティ関数を提供します。
 * - Zソート（Painter's Algorithm）: 遠い面から近い面へ順に描画
 * - バックフェイスカリング: カメラから見えない背面を除外
 *
 * @exports calculateFaceDepth - 面の深度を計算
 * @exports calculateFaceNormal - 面の法線ベクトルを計算
 * @exports isBackFacing - 面が背面かどうかを判定
 * @exports sortFacesByDepth - 面を深度順にソート
 * @exports createFace - 面データ構造を作成
 */

// ================
// 定数
// ================

const EPSILON = 1e-6; // 浮動小数点比較のための閾値

// ================
// 公開関数
// ================

/**
 * 面の深度を計算（スクリーン空間のY座標の平均）
 *
 * @param {Array<{x: number, y: number}>} screenVertices - スクリーン座標の頂点配列
 * @returns {number} 深度（Y座標の平均、大きいほど手前）
 *
 * @example
 * const depth = calculateFaceDepth([
 *   {x: 100, y: 100},
 *   {x: 150, y: 100},
 *   {x: 150, y: 150},
 *   {x: 100, y: 150}
 * ]);
 */
export function calculateFaceDepth(screenVertices) {
    if (!screenVertices || screenVertices.length === 0) {
        console.warn('calculateFaceDepth: 頂点が指定されていません');
        return 0;
    }

    const sumY = screenVertices.reduce((sum, v) => sum + v.y, 0);
    return sumY / screenVertices.length;
}

/**
 * 面の法線ベクトルを計算（3D空間）
 *
 * 3つの頂点から面の法線ベクトルを計算します。
 * 頂点は反時計回り（CCW）で指定されることを前提とします。
 *
 * @param {Object} v1 - 1番目の頂点 {x, y, z}
 * @param {Object} v2 - 2番目の頂点 {x, y, z}
 * @param {Object} v3 - 3番目の頂点 {x, y, z}
 * @returns {{x: number, y: number, z: number}} 正規化された法線ベクトル
 *
 * @example
 * const normal = calculateFaceNormal(
 *   {x: 0, y: 0, z: 0},
 *   {x: 1, y: 0, z: 0},
 *   {x: 0, y: 1, z: 0}
 * );
 * // => {x: 0, y: 0, z: 1}
 */
export function calculateFaceNormal(v1, v2, v3) {
    // ベクトル v1->v2 と v1->v3 を計算
    const edge1 = {
        x: v2.x - v1.x,
        y: v2.y - v1.y,
        z: v2.z - v1.z
    };
    const edge2 = {
        x: v3.x - v1.x,
        y: v3.y - v1.y,
        z: v3.z - v1.z
    };

    // 外積を計算（法線ベクトル）
    const normal = {
        x: edge1.y * edge2.z - edge1.z * edge2.y,
        y: edge1.z * edge2.x - edge1.x * edge2.z,
        z: edge1.x * edge2.y - edge1.y * edge2.x
    };

    // 正規化
    const length = Math.sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);

    if (length < EPSILON) {
        // 退化した面（3点が一直線上にある）
        console.warn('calculateFaceNormal: 退化した面が検出されました');
        return { x: 0, y: 0, z: 1 };
    }

    return {
        x: normal.x / length,
        y: normal.y / length,
        z: normal.z / length
    };
}

/**
 * 面が背面かどうかを判定（バックフェイスカリング）
 *
 * 面の法線ベクトルと視線ベクトルの内積を計算し、
 * 背面（カメラから見えない面）かどうかを判定します。
 *
 * @param {{x: number, y: number, z: number}} normal - 面の法線ベクトル
 * @param {{x: number, y: number, z: number}} viewDirection - 視線ベクトル（カメラからオブジェクトへ）
 * @returns {boolean} true: 背面（描画しない）、false: 前面（描画する）
 *
 * @example
 * const normal = {x: 0, y: 0, z: 1};
 * const viewDir = {x: 0, y: 0, z: -1}; // カメラから見てる方向
 * const isBack = isBackFacing(normal, viewDir); // => true
 */
export function isBackFacing(normal, viewDirection) {
    // 内積を計算
    const dotProduct = normal.x * viewDirection.x +
                      normal.y * viewDirection.y +
                      normal.z * viewDirection.z;

    // 内積が負 => 面が視線方向と逆向き => 前面
    // 内積が正 => 面が視線方向と同じ向き => 背面
    return dotProduct > EPSILON;
}

/**
 * 等角投影における視線ベクトルを計算
 *
 * @param {number} rotation - 回転角度（度）
 * @param {number} tilt - 傾き角度（度）
 * @returns {{x: number, y: number, z: number}} 正規化された視線ベクトル
 *
 * @example
 * const viewDir = calculateViewDirection(45, 30);
 */
export function calculateViewDirection(rotation, tilt) {
    const rotRad = rotation * Math.PI / 180;
    const tiltRad = tilt * Math.PI / 180;

    // 等角投影における視線ベクトル
    // 回転と傾きから計算
    const x = Math.sin(rotRad) * Math.cos(tiltRad);
    const y = Math.cos(rotRad) * Math.cos(tiltRad);
    const z = Math.sin(tiltRad);

    // 正規化
    const length = Math.sqrt(x * x + y * y + z * z);

    return {
        x: x / length,
        y: y / length,
        z: z / length
    };
}

/**
 * 面を深度順にソート（Painter's Algorithm）
 *
 * 遠い面（深度が小さい）から近い面（深度が大きい）への順にソートします。
 * これにより、遠い面から順に描画することで正しい重なりを実現できます。
 *
 * @param {Array<Object>} faces - 面の配列（各面は depth プロパティを持つ）
 * @returns {Array<Object>} 深度順にソートされた面の配列
 *
 * @example
 * const faces = [
 *   {vertices: [...], color: '#fff', depth: 150},
 *   {vertices: [...], color: '#aaa', depth: 100},
 *   {vertices: [...], color: '#ddd', depth: 120}
 * ];
 * const sorted = sortFacesByDepth(faces);
 * // => depth順: 100, 120, 150
 */
export function sortFacesByDepth(faces) {
    if (!faces || faces.length === 0) {
        return [];
    }

    // 深度の小さい順（遠い順）にソート
    return [...faces].sort((a, b) => a.depth - b.depth);
}

/**
 * 面データ構造を作成
 *
 * @param {Array<{x: number, y: number}>} screenVertices - スクリーン座標の頂点
 * @param {Array<{x: number, y: number, z: number}>} worldVertices - ワールド座標の頂点（法線計算用、最低3つ）
 * @param {string} color - 面の色
 * @param {string} strokeColor - 輪郭線の色
 * @param {number} lineWidth - 輪郭線の太さ
 * @returns {Object} 面データ
 *
 * @example
 * const face = createFace(
 *   [{x: 100, y: 100}, {x: 150, y: 100}, {x: 150, y: 150}, {x: 100, y: 150}],
 *   [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}],
 *   '#ffffff',
 *   '#000000',
 *   1
 * );
 */
export function createFace(screenVertices, worldVertices, color, strokeColor = null, lineWidth = 1) {
    if (!screenVertices || screenVertices.length < 3) {
        console.error('createFace: 頂点が不足しています（最低3つ必要）');
        return null;
    }

    if (!worldVertices || worldVertices.length < 3) {
        console.error('createFace: ワールド座標の頂点が不足しています（最低3つ必要）');
        return null;
    }

    const depth = calculateFaceDepth(screenVertices);
    const normal = calculateFaceNormal(worldVertices[0], worldVertices[1], worldVertices[2]);

    return {
        screenVertices,
        worldVertices,
        color,
        strokeColor,
        lineWidth,
        depth,
        normal
    };
}

/**
 * 面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - Canvasコンテキスト
 * @param {Object} face - 面データ（createFaceで作成）
 *
 * @example
 * const face = createFace(...);
 * drawFace(ctx, face);
 */
export function drawFace(ctx, face) {
    if (!face || !face.screenVertices || face.screenVertices.length < 3) {
        console.warn('drawFace: 無効な面データです');
        return;
    }

    const vertices = face.screenVertices;

    ctx.beginPath();
    ctx.moveTo(vertices[0].x, vertices[0].y);

    for (let i = 1; i < vertices.length; i++) {
        ctx.lineTo(vertices[i].x, vertices[i].y);
    }

    ctx.closePath();

    // 塗りつぶし
    if (face.color) {
        ctx.fillStyle = face.color;
        ctx.fill();
    }

    // 輪郭線
    if (face.strokeColor) {
        ctx.strokeStyle = face.strokeColor;
        ctx.lineWidth = face.lineWidth || 1;
        ctx.stroke();
    }
}

/**
 * バックフェイスカリングとZソートを適用して面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - Canvasコンテキスト
 * @param {Array<Object>} faces - 面の配列
 * @param {{x: number, y: number, z: number}} viewDirection - 視線ベクトル
 * @param {boolean} enableBackfaceCulling - バックフェイスカリングを有効にするか
 *
 * @example
 * const faces = [...]; // createFaceで作成した面の配列
 * const viewDir = calculateViewDirection(45, 30);
 * renderFaces(ctx, faces, viewDir, true);
 */
export function renderFaces(ctx, faces, viewDirection, enableBackfaceCulling = true) {
    if (!faces || faces.length === 0) {
        return;
    }

    // バックフェイスカリング
    let visibleFaces = faces;
    if (enableBackfaceCulling && viewDirection) {
        visibleFaces = faces.filter(face => !isBackFacing(face.normal, viewDirection));
    }

    // Zソート（遠い順）
    const sortedFaces = sortFacesByDepth(visibleFaces);

    // 描画
    ctx.save();
    sortedFaces.forEach(face => {
        drawFace(ctx, face);
    });
    ctx.restore();
}
