/**
 * @file threeDGeometry.js
 * @description 3Dジオメトリの共通描画ロジック（軽量化・最適化版）
 *
 * メインビューとプレビューで共通の描画ロジックを提供します。
 * 重複コードを削減し、パフォーマンスを最適化しています。
 *
 * @requires ../utils/threeDUtils.js - 3D描画ユーティリティ
 *
 * @exports createBoxGeometry - ボックスジオメトリの作成
 * @exports drawBoxFaces - ボックスの面を描画
 * @exports createShelfGeometry - 棚ジオメトリの作成
 * @exports createTableGeometry - テーブルジオメトリの作成
 */

import {
    lightenColor,
    darkenColor,
    sortFacesByDepth
} from '../utils/threeDUtils.js';

/**
 * ボックスの頂点インデックス定義
 */
const BOX_FACE_INDICES = {
    bottom: [0, 1, 2, 3],
    top: [4, 5, 6, 7],
    left: [0, 3, 7, 4],
    right: [1, 5, 6, 2],
    front: [0, 1, 5, 4],
    back: [2, 3, 7, 6]
};

/**
 * 面の色設定（デフォルト）
 */
const FACE_COLORS = {
    bottom: { darken: 30 },
    top: { lighten: 20 },
    left: { darken: 10 },
    right: { base: true },
    front: { darken: 5 },
    back: { darken: 15 }
};

/**
 * ボックスの8つの頂点を計算
 *
 * @param {number} x - 中心X座標
 * @param {number} y - 中心Y座標
 * @param {number} z - 中心Z座標
 * @param {number} width - 幅
 * @param {number} depth - 奥行き
 * @param {number} height - 高さ
 * @param {Function} worldToIso - 座標変換関数
 * @returns {Array<Object>} 8つの頂点のスクリーン座標
 */
function calculateBoxVertices(x, y, z, width, depth, height, worldToIso) {
    const halfW = width / 2;
    const halfD = depth / 2;
    const halfH = height / 2;

    const worldVertices = [
        { x: x - halfW, y: y - halfD, z: z - halfH }, // 0: 左下前
        { x: x + halfW, y: y - halfD, z: z - halfH }, // 1: 右下前
        { x: x + halfW, y: y + halfD, z: z - halfH }, // 2: 右下後
        { x: x - halfW, y: y + halfD, z: z - halfH }, // 3: 左下後
        { x: x - halfW, y: y - halfD, z: z + halfH }, // 4: 左上前
        { x: x + halfW, y: y - halfD, z: z + halfH }, // 5: 右上前
        { x: x + halfW, y: y + halfD, z: z + halfH }, // 6: 右上後
        { x: x - halfW, y: y + halfD, z: z + halfH }  // 7: 左上後
    ];

    return worldVertices.map(v => worldToIso(v.x, v.y, v.z));
}

/**
 * 面を作成する
 *
 * @param {string} faceName - 面の名前
 * @param {Array<Object>} screenVertices - スクリーン座標の頂点配列
 * @param {Array<number>} indices - 頂点インデックス
 * @param {Object} center - 面の中心座標 {x, y, z}
 * @param {string} baseColor - ベースカラー
 * @param {Object} colorConfig - 色設定
 * @returns {Object} 面オブジェクト {cx, cy, cz, draw}
 */
function createFace(faceName, screenVertices, indices, center, baseColor, colorConfig) {
    let fillColor = baseColor;

    if (colorConfig.base) {
        fillColor = baseColor;
    } else if (colorConfig.lighten !== undefined) {
        fillColor = lightenColor(baseColor, colorConfig.lighten);
    } else if (colorConfig.darken !== undefined) {
        fillColor = darkenColor(baseColor, colorConfig.darken);
    }

    // クロージャーでfillColorとscreenVerticesをキャプチャ
    const vertices = indices.map(i => screenVertices[i]);

    return {
        cx: center.x,
        cy: center.y,
        cz: center.z,
        draw: (ctx) => {
            ctx.fillStyle = fillColor;
            ctx.beginPath();
            ctx.moveTo(vertices[0].x, vertices[0].y);
            for (let i = 1; i < vertices.length; i++) {
                ctx.lineTo(vertices[i].x, vertices[i].y);
            }
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(baseColor, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    };
}

/**
 * ボックスジオメトリを作成
 *
 * @param {Object} coords3D - 3D座標情報 {x, y, z, width, depth, height}
 * @param {Function} worldToIso - 座標変換関数
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Object} state - ビューステート {scale}
 * @param {Object} options - オプション {hiddenFaces, color}
 * @returns {Object} ジオメトリオブジェクト {vertices, faces}
 */
export function createBoxGeometry(coords3D, worldToIso, centerX, centerY, state, options = {}) {
    const { x, y, z, width, depth, height } = coords3D;
    const { hiddenFaces = [], color = '#888888' } = options;

    // 頂点を計算
    const isoVertices = calculateBoxVertices(x, y, z, width, depth, height, worldToIso);
    const screenVertices = isoVertices.map(v => ({
        x: centerX + v.x * state.scale,
        y: centerY - v.y * state.scale
    }));

    // 面を作成
    const faces = [];
    const faceConfigs = {
        bottom: { center: { x, y, z: z - height / 2 }, color: FACE_COLORS.bottom },
        top: { center: { x, y, z: z + height / 2 }, color: FACE_COLORS.top },
        left: { center: { x: x - width / 2, y, z }, color: FACE_COLORS.left },
        right: { center: { x: x + width / 2, y, z }, color: FACE_COLORS.right },
        front: { center: { x, y: y - depth / 2, z }, color: FACE_COLORS.front },
        back: { center: { x, y: y + depth / 2, z }, color: FACE_COLORS.back }
    };

    for (const [faceName, indices] of Object.entries(BOX_FACE_INDICES)) {
        if (!hiddenFaces.includes(faceName)) {
            const config = faceConfigs[faceName];
            faces.push(createFace(faceName, screenVertices, indices, config.center, color, config.color));
        }
    }

    return { vertices: screenVertices, faces };
}

/**
 * ボックスの面を描画（深度ソート済み）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} geometry - ジオメトリオブジェクト
 * @param {Object} state - ビューステート
 */
export function drawBoxFaces(ctx, geometry, state) {
    ctx.save();
    const sortedFaces = sortFacesByDepth(geometry.faces, state);
    sortedFaces.forEach(face => face.draw(ctx));
    ctx.restore();
}

/**
 * 棚ジオメトリを作成（前面が開いている）
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {Function} worldToIso - 座標変換関数
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Object} state - ビューステート
 * @param {Object} options - オプション {frontDirection, color}
 * @returns {Object} ジオメトリオブジェクト
 */
export function createShelfGeometry(coords3D, worldToIso, centerX, centerY, state, options = {}) {
    const { frontDirection = 'top', color = '#888888' } = options;
    
    // 前面方向に応じて開いている面を決定
    const hiddenFaces = [];
    switch (frontDirection) {
        case 'top':
            hiddenFaces.push('front');
            break;
        case 'bottom':
            hiddenFaces.push('back');
            break;
        case 'left':
            hiddenFaces.push('left');
            break;
        case 'right':
            hiddenFaces.push('right');
            break;
    }

    const geometry = createBoxGeometry(coords3D, worldToIso, centerX, centerY, state, {
        hiddenFaces,
        color
    });

    // 棚板を追加（オプション）
    if (options.shelfLevels && options.shelfLevels > 1) {
        geometry.shelves = [];
        const { x, y, z, width, depth, height } = coords3D;
        const levels = options.shelfLevels;

        for (let i = 1; i < levels; i++) {
            const levelZ = z - height / 2 + (height / levels) * i;
            const shelfVertices = [
                worldToIso(x - width / 2, y - depth / 2, levelZ),
                worldToIso(x + width / 2, y - depth / 2, levelZ),
                worldToIso(x + width / 2, y + depth / 2, levelZ),
                worldToIso(x - width / 2, y + depth / 2, levelZ)
            ];

            geometry.shelves.push({
                vertices: shelfVertices.map(v => ({
                    x: centerX + v.x * state.scale,
                    y: centerY - v.y * state.scale
                })),
                z: levelZ
            });
        }
    }

    return geometry;
}

/**
 * テーブルジオメトリを作成（天板と脚）
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {Function} worldToIso - 座標変換関数
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Object} state - ビューステート
 * @param {Object} options - オプション {color}
 * @returns {Object} ジオメトリオブジェクト
 */
export function createTableGeometry(coords3D, worldToIso, centerX, centerY, state, options = {}) {
    const { x, y, z, width, depth, height } = coords3D;
    const { color = '#888888' } = options;
    const topThickness = height * 0.1;
    const legWidth = Math.min(width, depth) * 0.1;

    const allFaces = [];

    // 天板
    const topZ = z + height / 2 - topThickness / 2;
    const topGeometry = createBoxGeometry(
        { x, y, z: topZ, width, depth, height: topThickness },
        worldToIso,
        centerX,
        centerY,
        state,
        { color }
    );
    allFaces.push(...topGeometry.faces);

    // 4本の脚
    const legPositions = [
        { x: x - width / 2 + legWidth, y: y - depth / 2 + legWidth },
        { x: x + width / 2 - legWidth, y: y - depth / 2 + legWidth },
        { x: x + width / 2 - legWidth, y: y + depth / 2 - legWidth },
        { x: x - width / 2 + legWidth, y: y + depth / 2 - legWidth }
    ];

    legPositions.forEach(pos => {
        const legGeometry = createBoxGeometry(
            {
                x: pos.x,
                y: pos.y,
                z: (z - height / 2 + topZ - topThickness / 2) / 2,
                width: legWidth,
                depth: legWidth,
                height: topZ - topThickness / 2 - (z - height / 2)
            },
            worldToIso,
            centerX,
            centerY,
            state,
            { color }
        );
        allFaces.push(...legGeometry.faces);
    });

    return { faces: allFaces };
}

/**
 * 棚板を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} geometry - ジオメトリオブジェクト（shelvesプロパティを持つ）
 * @param {string} color - ベースカラー
 */
export function drawShelfShelves(ctx, geometry, color) {
    if (!geometry.shelves) return;

    ctx.save();
    ctx.strokeStyle = darkenColor(color, 30);
    ctx.lineWidth = 2;

    geometry.shelves.forEach(shelf => {
        ctx.beginPath();
        ctx.moveTo(shelf.vertices[0].x, shelf.vertices[0].y);
        for (let i = 1; i < shelf.vertices.length; i++) {
            ctx.lineTo(shelf.vertices[i].x, shelf.vertices[i].y);
        }
        ctx.closePath();
        ctx.stroke();
    });

    ctx.restore();
}

