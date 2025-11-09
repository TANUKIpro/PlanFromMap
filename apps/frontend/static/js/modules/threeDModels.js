/**
 * @file threeDModels.js
 * @description 3Dモデルの描画ロジック（メインビュー・プレビュー共通）
 *
 * オブジェクトタイプ（棚、箱、テーブル、扉、壁）に応じた3D描画を行います。
 * Painter's Algorithmを使用して、視点からの深度に基づいて面を描画します。
 *
 * @requires ../utils/threeDUtils.js - 3D描画ユーティリティ
 *
 * @exports draw3DShelf - 棚の3Dモデル描画（メインビュー用）
 * @exports draw3DBox - 箱の3Dモデル描画（メインビュー用）
 * @exports draw3DTable - テーブルの3Dモデル描画（メインビュー用）
 * @exports draw3DDoor - 扉の3Dモデル描画（メインビュー用）
 * @exports draw3DWall - 壁の3Dモデル描画（メインビュー用）
 * @exports drawBox - 通常のボックス描画（メインビュー用）
 * @exports drawPreviewShelf - 棚の3Dモデル描画（プレビュー用）
 * @exports drawPreviewBox - 箱の3Dモデル描画（プレビュー用）
 * @exports drawPreviewTable - テーブルの3Dモデル描画（プレビュー用）
 * @exports drawPreviewDoor - 扉の3Dモデル描画（プレビュー用）
 * @exports drawPreviewWall - 壁の3Dモデル描画（プレビュー用）
 */

import {
    lightenColor,
    darkenColor,
    sortFacesByDepth,
    applyRotation
} from '../utils/threeDUtils.js';

// ================
// メインビュー用描画関数
// ================

/**
 * 棚の3Dモデルを描画（前面が開いている）
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {string} frontDirection - 前面方向（'top', 'bottom', 'left', 'right'）
 * @param {Object} objectProperties - オブジェクトプロパティ（shelfLevels等）
 * @param {Function} worldToIso - 座標変換関数
 * @param {Object} state - ビューステート
 */
export function draw3DShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, worldToIso, state) {
    const { x, y, z, width, depth, height } = coords3D;

    const vertices = [
        worldToIso(x - width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y + depth/2, z + height/2),
        worldToIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * state.scale,
        y: centerY - v.y * state.scale
    }));

    ctx.save();

    // 前面の向きに応じて、開いている面を決定
    let drawFront = true, drawBack = true, drawLeft = true, drawRight = true;
    switch (frontDirection) {
        case 'top':
            drawFront = false;
            break;
        case 'bottom':
            drawBack = false;
            break;
        case 'left':
            drawLeft = false;
            break;
        case 'right':
            drawRight = false;
            break;
        default:
            drawFront = false;
    }

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 上面（天板）
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(screen[4].x, screen[4].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    if (drawLeft) {
        faces.push({
            cx: x - width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[4].x, screen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 右面
    if (drawRight) {
        faces.push({
            cx: x + width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.moveTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.lineTo(screen[2].x, screen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 前面
    if (drawFront) {
        faces.push({
            cx: x, cy: y - depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 5);
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[4].x, screen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 背面
    if (drawBack) {
        faces.push({
            cx: x, cy: y + depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(screen[2].x, screen[2].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, state);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    // 棚板を描画（常に最後に描画）
    if (objectProperties && objectProperties.shelfLevels) {
        const levels = objectProperties.shelfLevels;
        ctx.strokeStyle = darkenColor(color, 30);
        ctx.lineWidth = 2;

        for (let i = 1; i < levels; i++) {
            const levelZ = z - height/2 + (height / levels) * i;
            const v1 = worldToIso(x - width/2, y - depth/2, levelZ);
            const v2 = worldToIso(x + width/2, y - depth/2, levelZ);
            const v3 = worldToIso(x + width/2, y + depth/2, levelZ);
            const v4 = worldToIso(x - width/2, y + depth/2, levelZ);

            const s1 = { x: centerX + v1.x * state.scale, y: centerY - v1.y * state.scale };
            const s2 = { x: centerX + v2.x * state.scale, y: centerY - v2.y * state.scale };
            const s3 = { x: centerX + v3.x * state.scale, y: centerY - v3.y * state.scale };
            const s4 = { x: centerX + v4.x * state.scale, y: centerY - v4.y * state.scale };

            ctx.beginPath();
            ctx.moveTo(s1.x, s1.y);
            ctx.lineTo(s2.x, s2.y);
            ctx.lineTo(s3.x, s3.y);
            ctx.lineTo(s4.x, s4.y);
            ctx.closePath();
            ctx.stroke();
        }
    }

    ctx.restore();
}

/**
 * 箱の3Dモデルを描画（上部が開いている）
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 * @param {Function} worldToIso - 座標変換関数
 * @param {Object} state - ビューステート
 */
export function draw3DBox(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, worldToIso, state) {
    const { x, y, z, width, depth, height } = coords3D;

    const vertices = [
        worldToIso(x - width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y - depth/2, z - height/2),
        worldToIso(x + width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y + depth/2, z - height/2),
        worldToIso(x - width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y - depth/2, z + height/2),
        worldToIso(x + width/2, y + depth/2, z + height/2),
        worldToIso(x - width/2, y + depth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * state.scale,
        y: centerY - v.y * state.scale
    }));

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, state);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * テーブルの3Dモデルを描画
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToIso - 座標変換関数
 * @param {Object} state - ビューステート
 */
export function draw3DTable(ctx, coords3D, color, centerX, centerY, worldToIso, state) {
    const { x, y, z, width, depth, height } = coords3D;
    const topThickness = height * 0.1;
    const legWidth = Math.min(width, depth) * 0.1;

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 天板を描画
    const topZ = z + height/2 - topThickness/2;
    const topVertices = [
        worldToIso(x - width/2, y - depth/2, topZ - topThickness/2),
        worldToIso(x + width/2, y - depth/2, topZ - topThickness/2),
        worldToIso(x + width/2, y + depth/2, topZ - topThickness/2),
        worldToIso(x - width/2, y + depth/2, topZ - topThickness/2),
        worldToIso(x - width/2, y - depth/2, topZ + topThickness/2),
        worldToIso(x + width/2, y - depth/2, topZ + topThickness/2),
        worldToIso(x + width/2, y + depth/2, topZ + topThickness/2),
        worldToIso(x - width/2, y + depth/2, topZ + topThickness/2),
    ];

    const topScreen = topVertices.map(v => ({
        x: centerX + v.x * state.scale,
        y: centerY - v.y * state.scale
    }));

    // 天板の底面
    faces.push({
        cx: x, cy: y, cz: topZ - topThickness/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の上面
    faces.push({
        cx: x, cy: y, cz: topZ + topThickness/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(topScreen[4].x, topScreen[4].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の左面
    faces.push({
        cx: x - width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の右面
    faces.push({
        cx: x + width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の前面
    faces.push({
        cx: x, cy: y - depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板の背面
    faces.push({
        cx: x, cy: y + depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 4本の脚を描画
    const legPositions = [
        { x: x - width/2 + legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y + depth/2 - legWidth },
        { x: x - width/2 + legWidth, y: y + depth/2 - legWidth },
    ];

    legPositions.forEach(pos => {
        const legVertices = [
            worldToIso(pos.x - legWidth/2, pos.y - legWidth/2, z - height/2),
            worldToIso(pos.x + legWidth/2, pos.y - legWidth/2, z - height/2),
            worldToIso(pos.x + legWidth/2, pos.y + legWidth/2, z - height/2),
            worldToIso(pos.x - legWidth/2, pos.y + legWidth/2, z - height/2),
            worldToIso(pos.x - legWidth/2, pos.y - legWidth/2, topZ - topThickness/2),
            worldToIso(pos.x + legWidth/2, pos.y - legWidth/2, topZ - topThickness/2),
            worldToIso(pos.x + legWidth/2, pos.y + legWidth/2, topZ - topThickness/2),
            worldToIso(pos.x - legWidth/2, pos.y + legWidth/2, topZ - topThickness/2),
        ];

        const legScreen = legVertices.map(v => ({
            x: centerX + v.x * state.scale,
            y: centerY - v.y * state.scale
        }));

        const legCenterZ = (z - height/2 + topZ - topThickness/2) / 2;

        // 脚の底面
        faces.push({
            cx: pos.x, cy: pos.y, cz: z - height/2,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 30);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の左面
        faces.push({
            cx: pos.x - legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 20);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の右面
        faces.push({
            cx: pos.x + legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の前面
        faces.push({
            cx: pos.x, cy: pos.y - legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の背面
        faces.push({
            cx: pos.x, cy: pos.y + legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 25);
                ctx.beginPath();
                ctx.moveTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, state);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * 扉の3Dモデルを描画
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToIso - 座標変換関数
 * @param {Object} state - ビューステート
 */
export function draw3DDoor(ctx, coords3D, color, centerX, centerY, worldToIso, state) {
    const { x, y, z, width, depth, height } = coords3D;
    const doorDepth = Math.min(depth, 0.1);

    const vertices = [
        worldToIso(x - width/2, y - doorDepth/2, z - height/2),
        worldToIso(x + width/2, y - doorDepth/2, z - height/2),
        worldToIso(x + width/2, y + doorDepth/2, z - height/2),
        worldToIso(x - width/2, y + doorDepth/2, z - height/2),
        worldToIso(x - width/2, y - doorDepth/2, z + height/2),
        worldToIso(x + width/2, y - doorDepth/2, z + height/2),
        worldToIso(x + width/2, y + doorDepth/2, z + height/2),
        worldToIso(x - width/2, y + doorDepth/2, z + height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * state.scale,
        y: centerY - v.y * state.scale
    }));

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(screen[4].x, screen[4].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - doorDepth/2, cz: z,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + doorDepth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, state);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * 壁の3Dモデルを描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToIso - 座標変換関数
 * @param {Object} state - ビューステート
 */
export function draw3DWall(ctx, coords3D, color, centerX, centerY, worldToIso, state) {
    drawBox(ctx, coords3D, color, centerX, centerY, worldToIso, state);
}

/**
 * 直方体（ボックス）を描画
 * Painter's Algorithmを使用して、視点からの距離に基づいて面を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToIso - 座標変換関数
 * @param {Object} state - ビューステート
 */
export function drawBox(ctx, coords3D, color, centerX, centerY, worldToIso, state) {
    const { x, y, z, width, depth, height } = coords3D;

    // 8つの頂点を計算
    const vertices = [
        worldToIso(x - width/2, y - depth/2, z - height/2),  // 0: 左下前
        worldToIso(x + width/2, y - depth/2, z - height/2),  // 1: 右下前
        worldToIso(x + width/2, y + depth/2, z - height/2),  // 2: 右下後
        worldToIso(x - width/2, y + depth/2, z - height/2),  // 3: 左下後
        worldToIso(x - width/2, y - depth/2, z + height/2),  // 4: 左上前
        worldToIso(x + width/2, y - depth/2, z + height/2),  // 5: 右上前
        worldToIso(x + width/2, y + depth/2, z + height/2),  // 6: 右上後
        worldToIso(x - width/2, y + depth/2, z + height/2),  // 7: 左上後
    ];

    // スクリーン座標に変換
    const screen = vertices.map(v => ({
        x: centerX + v.x * state.scale,
        y: centerY - v.y * state.scale
    }));

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(screen[4].x, screen[4].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, state);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

// ================
// プレビュー用描画関数
// ================

/**
 * 棚のプレビューを描画（前面が開いている、回転サポート）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報（rotation含む）
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ（shelfLevels等）
 * @param {Function} worldToPreviewIso - 座標変換関数
 * @param {Object} previewState - プレビューステート
 */
export function drawPreviewShelf(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, worldToPreviewIso, previewState) {
    const { x, y, z, width, depth, height, rotation } = coords3D;

    // 8つの頂点を計算（回転を考慮）
    const localVertices = [
        { lx: -width/2, ly: -depth/2, lz: -height/2 },
        { lx: +width/2, ly: -depth/2, lz: -height/2 },
        { lx: +width/2, ly: +depth/2, lz: -height/2 },
        { lx: -width/2, ly: +depth/2, lz: -height/2 },
        { lx: -width/2, ly: -depth/2, lz: +height/2 },
        { lx: +width/2, ly: -depth/2, lz: +height/2 },
        { lx: +width/2, ly: +depth/2, lz: +height/2 },
        { lx: -width/2, ly: +depth/2, lz: +height/2 },
    ];

    const vertices = localVertices.map(v => {
        const rotated = applyRotation(v.lx, v.ly, rotation);
        return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz, previewState);
    });

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 前面の向きに応じて、開いている面を決定
    let drawFront = true, drawBack = true, drawLeft = true, drawRight = true;

    switch (frontDirection) {
        case 'top':
            drawFront = false; // y-方向（上）が開く
            break;
        case 'bottom':
            drawBack = false; // y+方向（下）が開く
            break;
        case 'left':
            drawLeft = false; // x-方向（左）が開く
            break;
        case 'right':
            drawRight = false; // x+方向（右）が開く
            break;
        default:
            drawFront = false; // デフォルトは前面が開く
    }

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 上面（天板）
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(screen[4].x, screen[4].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    if (drawLeft) {
        faces.push({
            cx: x - width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[4].x, screen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 右面
    if (drawRight) {
        faces.push({
            cx: x + width/2, cy: y, cz: z,
            draw: () => {
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.moveTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.lineTo(screen[2].x, screen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 前面（y-）
    if (drawFront) {
        faces.push({
            cx: x, cy: y - depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 5);
                ctx.beginPath();
                ctx.moveTo(screen[0].x, screen[0].y);
                ctx.lineTo(screen[1].x, screen[1].y);
                ctx.lineTo(screen[5].x, screen[5].y);
                ctx.lineTo(screen[4].x, screen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // 背面（y+）
    if (drawBack) {
        faces.push({
            cx: x, cy: y + depth/2, cz: z,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(screen[2].x, screen[2].y);
                ctx.lineTo(screen[3].x, screen[3].y);
                ctx.lineTo(screen[7].x, screen[7].y);
                ctx.lineTo(screen[6].x, screen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    // 棚板を描画（オプション）
    if (objectProperties && objectProperties.shelfLevels) {
        const levels = objectProperties.shelfLevels;
        ctx.strokeStyle = darkenColor(color, 30);
        ctx.lineWidth = 1.5;

        for (let i = 1; i < levels; i++) {
            const levelZ = z - height/2 + (height / levels) * i;
            const v1 = worldToPreviewIso(x - width/2, y - depth/2, levelZ, previewState);
            const v2 = worldToPreviewIso(x + width/2, y - depth/2, levelZ, previewState);
            const v3 = worldToPreviewIso(x + width/2, y + depth/2, levelZ, previewState);
            const v4 = worldToPreviewIso(x - width/2, y + depth/2, levelZ, previewState);

            const s1 = { x: centerX + v1.x * previewState.scale, y: centerY - v1.y * previewState.scale };
            const s2 = { x: centerX + v2.x * previewState.scale, y: centerY - v2.y * previewState.scale };
            const s3 = { x: centerX + v3.x * previewState.scale, y: centerY - v3.y * previewState.scale };
            const s4 = { x: centerX + v4.x * previewState.scale, y: centerY - v4.y * previewState.scale };

            ctx.beginPath();
            ctx.moveTo(s1.x, s1.y);
            ctx.lineTo(s2.x, s2.y);
            ctx.lineTo(s3.x, s3.y);
            ctx.lineTo(s4.x, s4.y);
            ctx.closePath();
            ctx.stroke();
        }
    }

    ctx.restore();
}

/**
 * 箱のプレビューを描画（上部が開いている、回転サポート）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報（rotation含む）
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 * @param {Function} worldToPreviewIso - 座標変換関数
 * @param {Object} previewState - プレビューステート
 */
export function drawPreviewBox(ctx, coords3D, color, centerX, centerY, frontDirection, objectProperties, worldToPreviewIso, previewState) {
    const { x, y, z, width, depth, height, rotation } = coords3D;

    // 8つの頂点を計算（回転を考慮）
    const localVertices = [
        { lx: -width/2, ly: -depth/2, lz: -height/2 },
        { lx: +width/2, ly: -depth/2, lz: -height/2 },
        { lx: +width/2, ly: +depth/2, lz: -height/2 },
        { lx: -width/2, ly: +depth/2, lz: -height/2 },
        { lx: -width/2, ly: -depth/2, lz: +height/2 },
        { lx: +width/2, ly: -depth/2, lz: +height/2 },
        { lx: +width/2, ly: +depth/2, lz: +height/2 },
        { lx: -width/2, ly: +depth/2, lz: +height/2 },
    ];

    const vertices = localVertices.map(v => {
        const rotated = applyRotation(v.lx, v.ly, rotation);
        return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz, previewState);
    });

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // 上部は開いているので描画しない

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * テーブルのプレビューを描画（天板と脚）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToPreviewIso - 座標変換関数
 * @param {Object} previewState - プレビューステート
 */
export function drawPreviewTable(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState) {
    const { x, y, z, width, depth, height } = coords3D;
    const topThickness = height * 0.1; // 天板の厚さ
    const legWidth = Math.min(width, depth) * 0.1; // 脚の太さ

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 天板を描画
    const topZ = z + height/2 - topThickness/2;
    const topVertices = [
        worldToPreviewIso(x - width/2, y - depth/2, topZ - topThickness/2, previewState),
        worldToPreviewIso(x + width/2, y - depth/2, topZ - topThickness/2, previewState),
        worldToPreviewIso(x + width/2, y + depth/2, topZ - topThickness/2, previewState),
        worldToPreviewIso(x - width/2, y + depth/2, topZ - topThickness/2, previewState),
        worldToPreviewIso(x - width/2, y - depth/2, topZ + topThickness/2, previewState),
        worldToPreviewIso(x + width/2, y - depth/2, topZ + topThickness/2, previewState),
        worldToPreviewIso(x + width/2, y + depth/2, topZ + topThickness/2, previewState),
        worldToPreviewIso(x - width/2, y + depth/2, topZ + topThickness/2, previewState),
    ];

    const topScreen = topVertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    // 天板底面
    faces.push({
        cx: x, cy: y, cz: topZ - topThickness/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板上面
    faces.push({
        cx: x, cy: y, cz: topZ + topThickness/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(topScreen[4].x, topScreen[4].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板左面
    faces.push({
        cx: x - width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板右面
    faces.push({
        cx: x + width/2, cy: y, cz: topZ,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.lineTo(topScreen[2].x, topScreen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板前面
    faces.push({
        cx: x, cy: y - depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(topScreen[0].x, topScreen[0].y);
            ctx.lineTo(topScreen[1].x, topScreen[1].y);
            ctx.lineTo(topScreen[5].x, topScreen[5].y);
            ctx.lineTo(topScreen[4].x, topScreen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 天板背面
    faces.push({
        cx: x, cy: y + depth/2, cz: topZ,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(topScreen[2].x, topScreen[2].y);
            ctx.lineTo(topScreen[3].x, topScreen[3].y);
            ctx.lineTo(topScreen[7].x, topScreen[7].y);
            ctx.lineTo(topScreen[6].x, topScreen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 4本の脚を描画
    const legPositions = [
        { x: x - width/2 + legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y - depth/2 + legWidth },
        { x: x + width/2 - legWidth, y: y + depth/2 - legWidth },
        { x: x - width/2 + legWidth, y: y + depth/2 - legWidth },
    ];

    legPositions.forEach(pos => {
        const legVertices = [
            worldToPreviewIso(pos.x - legWidth/2, pos.y - legWidth/2, z - height/2, previewState),
            worldToPreviewIso(pos.x + legWidth/2, pos.y - legWidth/2, z - height/2, previewState),
            worldToPreviewIso(pos.x + legWidth/2, pos.y + legWidth/2, z - height/2, previewState),
            worldToPreviewIso(pos.x - legWidth/2, pos.y + legWidth/2, z - height/2, previewState),
            worldToPreviewIso(pos.x - legWidth/2, pos.y - legWidth/2, topZ - topThickness/2, previewState),
            worldToPreviewIso(pos.x + legWidth/2, pos.y - legWidth/2, topZ - topThickness/2, previewState),
            worldToPreviewIso(pos.x + legWidth/2, pos.y + legWidth/2, topZ - topThickness/2, previewState),
            worldToPreviewIso(pos.x - legWidth/2, pos.y + legWidth/2, topZ - topThickness/2, previewState),
        ];

        const legScreen = legVertices.map(v => ({
            x: centerX + v.x * previewState.scale,
            y: centerY - v.y * previewState.scale
        }));

        const legCenterZ = (z - height/2 + topZ - topThickness/2) / 2;

        // 脚の底面
        faces.push({
            cx: pos.x, cy: pos.y, cz: z - height/2,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 30);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の左面
        faces.push({
            cx: pos.x - legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 20);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の右面
        faces.push({
            cx: pos.x + legWidth/2, cy: pos.y, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 15);
                ctx.beginPath();
                ctx.moveTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.lineTo(legScreen[2].x, legScreen[2].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の前面
        faces.push({
            cx: pos.x, cy: pos.y - legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 10);
                ctx.beginPath();
                ctx.moveTo(legScreen[0].x, legScreen[0].y);
                ctx.lineTo(legScreen[1].x, legScreen[1].y);
                ctx.lineTo(legScreen[5].x, legScreen[5].y);
                ctx.lineTo(legScreen[4].x, legScreen[4].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });

        // 脚の背面
        faces.push({
            cx: pos.x, cy: pos.y + legWidth/2, cz: legCenterZ,
            draw: () => {
                ctx.fillStyle = darkenColor(color, 25);
                ctx.beginPath();
                ctx.moveTo(legScreen[2].x, legScreen[2].y);
                ctx.lineTo(legScreen[3].x, legScreen[3].y);
                ctx.lineTo(legScreen[7].x, legScreen[7].y);
                ctx.lineTo(legScreen[6].x, legScreen[6].y);
                ctx.closePath();
                ctx.fill();
                ctx.strokeStyle = darkenColor(color, 20);
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * 扉のプレビューを描画（薄い長方形）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToPreviewIso - 座標変換関数
 * @param {Object} previewState - プレビューステート
 */
export function drawPreviewDoor(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState) {
    const { x, y, z, width, depth, height } = coords3D;
    // 扉は薄いので、depthを小さくする
    const doorDepth = Math.min(depth, 0.1);

    const vertices = [
        worldToPreviewIso(x - width/2, y - doorDepth/2, z - height/2, previewState),
        worldToPreviewIso(x + width/2, y - doorDepth/2, z - height/2, previewState),
        worldToPreviewIso(x + width/2, y + doorDepth/2, z - height/2, previewState),
        worldToPreviewIso(x - width/2, y + doorDepth/2, z - height/2, previewState),
        worldToPreviewIso(x - width/2, y - doorDepth/2, z + height/2, previewState),
        worldToPreviewIso(x + width/2, y - doorDepth/2, z + height/2, previewState),
        worldToPreviewIso(x + width/2, y + doorDepth/2, z + height/2, previewState),
        worldToPreviewIso(x - width/2, y + doorDepth/2, z + height/2, previewState),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(screen[4].x, screen[4].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面（薄い面）
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面（大きい面）
    faces.push({
        cx: x, cy: y - doorDepth/2, cz: z,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + doorDepth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}

/**
 * 壁のプレビューを描画（通常の長方形）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToPreviewIso - 座標変換関数
 * @param {Object} previewState - プレビューステート
 */
export function drawPreviewWall(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState) {
    // 壁は通常のボックスとして描画
    drawPreviewBoxSimple(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState);
}

/**
 * 通常のボックスプレビュー（回転なし）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - オブジェクトの色
 * @param {number} centerX - 画面中心X座標
 * @param {number} centerY - 画面中心Y座標
 * @param {Function} worldToPreviewIso - 座標変換関数
 * @param {Object} previewState - プレビューステート
 */
export function drawPreviewBoxSimple(ctx, coords3D, color, centerX, centerY, worldToPreviewIso, previewState) {
    const { x, y, z, width, depth, height, rotation } = coords3D;

    // 8つの頂点を計算（回転を考慮）
    const localVertices = [
        { lx: -width/2, ly: -depth/2, lz: -height/2 },
        { lx: +width/2, ly: -depth/2, lz: -height/2 },
        { lx: +width/2, ly: +depth/2, lz: -height/2 },
        { lx: -width/2, ly: +depth/2, lz: -height/2 },
        { lx: -width/2, ly: -depth/2, lz: +height/2 },
        { lx: +width/2, ly: -depth/2, lz: +height/2 },
        { lx: +width/2, ly: +depth/2, lz: +height/2 },
        { lx: -width/2, ly: +depth/2, lz: +height/2 },
    ];

    const vertices = localVertices.map(v => {
        const rotated = applyRotation(v.lx, v.ly, rotation);
        return worldToPreviewIso(x + rotated.x, y + rotated.y, z + v.lz, previewState);
    });

    const screen = vertices.map(v => ({
        x: centerX + v.x * previewState.scale,
        y: centerY - v.y * previewState.scale
    }));

    ctx.save();

    // すべての面を配列として定義（中心座標と描画関数を含む）
    const faces = [];

    // 底面
    faces.push({
        cx: x, cy: y, cz: z - height/2,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 30);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 上面
    faces.push({
        cx: x, cy: y, cz: z + height/2,
        draw: () => {
            ctx.fillStyle = lightenColor(color, 20);
            ctx.beginPath();
            ctx.moveTo(screen[4].x, screen[4].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 左面
    faces.push({
        cx: x - width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 10);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 右面
    faces.push({
        cx: x + width/2, cy: y, cz: z,
        draw: () => {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.lineTo(screen[2].x, screen[2].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 前面
    faces.push({
        cx: x, cy: y - depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 5);
            ctx.beginPath();
            ctx.moveTo(screen[0].x, screen[0].y);
            ctx.lineTo(screen[1].x, screen[1].y);
            ctx.lineTo(screen[5].x, screen[5].y);
            ctx.lineTo(screen[4].x, screen[4].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // 背面
    faces.push({
        cx: x, cy: y + depth/2, cz: z,
        draw: () => {
            ctx.fillStyle = darkenColor(color, 15);
            ctx.beginPath();
            ctx.moveTo(screen[2].x, screen[2].y);
            ctx.lineTo(screen[3].x, screen[3].y);
            ctx.lineTo(screen[7].x, screen[7].y);
            ctx.lineTo(screen[6].x, screen[6].y);
            ctx.closePath();
            ctx.fill();
            ctx.strokeStyle = darkenColor(color, 20);
            ctx.lineWidth = 1;
            ctx.stroke();
        }
    });

    // Z深度でソート（遠い面から近い面へ）
    const sortedFaces = sortFacesByDepth(faces, previewState);

    // ソートされた順序で面を描画
    sortedFaces.forEach(face => face.draw());

    ctx.restore();
}
