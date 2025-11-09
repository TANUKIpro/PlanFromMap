/**
 * @file isometricProjection.js
 * @description 等角投影に関連するヘルパー関数
 *
 * 3D座標からカメラ座標/スクリーン座標への変換、
 * 面の可視性判定、描画順序決定などのユーティリティを提供します。
 */

const DEFAULT_VIEW_DIR = Object.freeze({ x: 0, y: -1, z: 0 });
const DEFAULT_LIGHT_DIR = normalizeVector({ x: -0.35, y: -0.5, z: 1 });

/**
 * カメラパラメータを生成
 *
 * @param {Object} viewState - ビューステート（rotation, tilt を含む）
 * @param {Object} [options]
 * @param {Object|null} [options.mapBounds] - マップの有効領域 {centerX, centerY}
 * @param {Object|null} [options.metadata] - マップメタデータ（解像度、原点）
 * @param {HTMLImageElement|null} [options.image] - マップ画像（高さ取得用）
 * @param {Object|null} [options.center] - 明示的に中心座標 {x, y} を指定する場合
 * @returns {Object} カメラパラメータ
 */
export function createCameraParams(viewState, { mapBounds = null, metadata = null, image = null, center = null } = {}) {
    const rotation = viewState?.rotation || 0;
    const tilt = viewState?.tilt || 0;

    const rotationRad = rotation * Math.PI / 180;
    const tiltRad = tilt * Math.PI / 180;

    let centerWorldX = 0;
    let centerWorldY = 0;

    if (center) {
        centerWorldX = center.x || 0;
        centerWorldY = center.y || 0;
    } else if (mapBounds && metadata && image) {
        const resolution = metadata?.resolution || 0.05;
        const originX = Array.isArray(metadata?.origin) ? metadata.origin[0] : 0;
        const originY = Array.isArray(metadata?.origin) ? metadata.origin[1] : 0;

        centerWorldX = originX + mapBounds.centerX * resolution;
        centerWorldY = originY + (image.height - mapBounds.centerY) * resolution;
    }

    return {
        rotationRad,
        tiltRad,
        cosRotation: Math.cos(rotationRad),
        sinRotation: Math.sin(rotationRad),
        cosTilt: Math.cos(tiltRad),
        sinTilt: Math.sin(tiltRad),
        centerWorldX,
        centerWorldY
    };
}

/**
 * 3Dポイントをカメラ座標とスクリーン座標に投影
 *
 * @param {Object} point - {x, y, z}
 * @param {Object} params - createCameraParams の戻り値
 * @returns {{camera: {x:number,y:number,z:number}, screen: {x:number,y:number}}}
 */
export function projectPoint(point, params) {
    const offsetX = point.x - (params.centerWorldX || 0);
    const offsetY = point.y - (params.centerWorldY || 0);

    const rotatedX = offsetX * params.cosRotation - offsetY * params.sinRotation;
    const rotatedY = offsetX * params.sinRotation + offsetY * params.cosRotation;
    const rotatedZ = point.z;

    const cameraY = rotatedY * params.cosTilt - rotatedZ * params.sinTilt;
    const cameraZ = rotatedY * params.sinTilt + rotatedZ * params.cosTilt;

    const camera = { x: rotatedX, y: cameraY, z: cameraZ };

    return {
        camera,
        screen: {
            x: -camera.x,
            y: camera.z
        }
    };
}

/**
 * 頂点リストを投影
 *
 * @param {Array<Object>} vertices - 3D頂点の配列
 * @param {Object} params - カメラパラメータ
 * @returns {{cameraVertices: Array<Object>, screenVertices: Array<Object>}}
 */
export function projectVertices(vertices, params) {
    const projected = vertices.map(vertex => projectPoint(vertex, params));
    return {
        cameraVertices: projected.map(p => p.camera),
        screenVertices: projected.map(p => p.screen)
    };
}

export const BOX_FACES = Object.freeze([
    { name: 'bottom', indices: [0, 1, 2, 3] },
    { name: 'top', indices: [4, 5, 6, 7] },
    { name: 'south', indices: [0, 1, 5, 4] },
    { name: 'east', indices: [1, 2, 6, 5] },
    { name: 'north', indices: [2, 3, 7, 6] },
    { name: 'west', indices: [3, 0, 4, 7] }
]);

/**
 * 面の法線ベクトルを計算
 *
 * @param {Object} face - 面情報
 * @param {Array<Object>} cameraVertices - カメラ座標系での頂点
 * @returns {Object} 法線ベクトル {x,y,z}
 */
export function computeFaceNormal(face, cameraVertices) {
    const [i0, i1, i2] = face.indices;
    const v0 = cameraVertices[i0];
    const v1 = cameraVertices[i1];
    const v2 = cameraVertices[i2];

    const edge1 = subtractVectors(v1, v0);
    const edge2 = subtractVectors(v2, v0);

    return {
        x: edge1.y * edge2.z - edge1.z * edge2.y,
        y: edge1.z * edge2.x - edge1.x * edge2.z,
        z: edge1.x * edge2.y - edge1.y * edge2.x
    };
}

/**
 * 面描画情報を生成
 *
 * @param {Array<Object>} cameraVertices - カメラ座標系の頂点
 * @param {Array<Object>} [faces=BOX_FACES] - 面定義
 * @returns {Array<Object>} 面情報
 */
export function buildFaceRenderList(cameraVertices, faces = BOX_FACES) {
    return faces.map(face => {
        const normal = computeFaceNormal(face, cameraVertices);
        const dot = normal.x * DEFAULT_VIEW_DIR.x + normal.y * DEFAULT_VIEW_DIR.y + normal.z * DEFAULT_VIEW_DIR.z;
        const avgY = face.indices.reduce((sum, idx) => sum + cameraVertices[idx].y, 0) / face.indices.length;
        const avgZ = face.indices.reduce((sum, idx) => sum + cameraVertices[idx].z, 0) / face.indices.length;

        return {
            ...face,
            normal,
            dot,
            avgY,
            avgZ
        };
    });
}

/**
 * 可視面のみを抽出
 *
 * @param {Array<Object>} faces - buildFaceRenderListの結果
 * @returns {Array<Object>} 可視面
 */
export function filterVisibleFaces(faces) {
    return faces.filter(face => face.dot > 1e-6);
}

/**
 * 描画順序でソート（遠い面から近い面へ）
 *
 * @param {Array<Object>} faces - 面情報
 * @returns {Array<Object>} ソート済み面
 */
export function sortFacesByDepth(faces) {
    return faces.slice().sort((a, b) => {
        if (b.avgY !== a.avgY) {
            return b.avgY - a.avgY;
        }
        return a.avgZ - b.avgZ;
    });
}

/**
 * 面の明暗を計算
 *
 * @param {Object} normal - 面の法線
 * @param {Object} [options]
 * @param {Object} [options.lightDir] - 光源方向
 * @returns {number} 明るさ調整量（正: 明るく、負: 暗く）
 */
export function calculateFaceShade(normal, { lightDir = DEFAULT_LIGHT_DIR } = {}) {
    const normalized = normalizeVector(normal);
    const intensity = clamp(
        normalized.x * lightDir.x + normalized.y * lightDir.y + normalized.z * lightDir.z,
        0,
        1
    );

    return (intensity - 0.5) * 40; // -20% 〜 +20% 程度の調整
}

function subtractVectors(a, b) {
    return {
        x: (a.x || 0) - (b.x || 0),
        y: (a.y || 0) - (b.y || 0),
        z: (a.z || 0) - (b.z || 0)
    };
}

function normalizeVector(vec) {
    const length = Math.hypot(vec.x || 0, vec.y || 0, vec.z || 0) || 1;
    return {
        x: (vec.x || 0) / length,
        y: (vec.y || 0) / length,
        z: (vec.z || 0) / length
    };
}

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

export const INTERNALS = {
    normalizeVector,
    subtractVectors,
    clamp
};

