/**
 * @file viewCube.js
 * @description ViewCube - 3Dビューの視点切り替えコントロール（高品質3Dキューブ実装）
 *
 * ViewCubeは3Dビューの右上に表示される3Dキューブで、
 * 面（6つ）、辺（12本）、頂点（8つ）をクリックすることでカメラ位置を素早く変更できます。
 *
 * 参考実装: https://github.com/Donvlouss/bevy_viewcube
 *
 * @exports initializeViewCube - ViewCubeの初期化
 * @exports updateViewCube - ViewCubeの表示更新
 */

// ================
// ViewCube状態
// ================

const viewCubeState = {
    canvas: null,
    ctx: null,
    size: 150,              // ViewCubeのサイズ（大きくした）
    rotation: 45,           // 現在の回転角度（Y軸）
    tilt: 30,               // 現在の傾き角度（X軸）
    hoveredElement: null,   // ホバー中の要素 {type: 'face'|'edge'|'vertex', index: number}
    onViewChange: null,     // 視点変更時のコールバック
    cubeSize: 40,           // キューブの論理サイズ
    animationId: null       // アニメーションID
};

// ================
// キューブの幾何学定義
// ================

/**
 * キューブの8頂点（ローカル座標系）
 * 中心が原点
 */
const CUBE_VERTICES = [
    { x: -1, y: -1, z: -1 }, // 0: 左下奥
    { x:  1, y: -1, z: -1 }, // 1: 右下奥
    { x:  1, y:  1, z: -1 }, // 2: 右上奥
    { x: -1, y:  1, z: -1 }, // 3: 左上奥
    { x: -1, y: -1, z:  1 }, // 4: 左下手前
    { x:  1, y: -1, z:  1 }, // 5: 右下手前
    { x:  1, y:  1, z:  1 }, // 6: 右上手前
    { x: -1, y:  1, z:  1 }  // 7: 左上手前
];

/**
 * キューブの6面（頂点インデックスの配列）
 * 各面は時計回りで定義
 */
const CUBE_FACES = [
    { vertices: [4, 5, 6, 7], normal: { x: 0, y: 0, z: 1 }, label: '前', color: '#667eea' },  // 前
    { vertices: [1, 0, 3, 2], normal: { x: 0, y: 0, z: -1 }, label: '後', color: '#667eea' }, // 後
    { vertices: [0, 4, 7, 3], normal: { x: -1, y: 0, z: 0 }, label: '左', color: '#48bb78' }, // 左
    { vertices: [5, 1, 2, 6], normal: { x: 1, y: 0, z: 0 }, label: '右', color: '#48bb78' },  // 右
    { vertices: [7, 6, 2, 3], normal: { x: 0, y: 1, z: 0 }, label: '上', color: '#ed8936' },  // 上
    { vertices: [4, 0, 1, 5], normal: { x: 0, y: -1, z: 0 }, label: '下', color: '#ed8936' }  // 下
];

/**
 * キューブの12辺（頂点インデックスのペア）
 */
const CUBE_EDGES = [
    // 下面の4辺
    { vertices: [0, 1] },
    { vertices: [1, 5] },
    { vertices: [5, 4] },
    { vertices: [4, 0] },
    // 上面の4辺
    { vertices: [3, 2] },
    { vertices: [2, 6] },
    { vertices: [6, 7] },
    { vertices: [7, 3] },
    // 縦の4辺
    { vertices: [0, 3] },
    { vertices: [1, 2] },
    { vertices: [5, 6] },
    { vertices: [4, 7] }
];

/**
 * 各要素をクリックした時の視点設定
 */
const VIEW_PRESETS = {
    // 6面
    faces: [
        { rotation: 0, tilt: 0 },      // 前
        { rotation: 180, tilt: 0 },    // 後
        { rotation: -90, tilt: 0 },    // 左
        { rotation: 90, tilt: 0 },     // 右
        { rotation: 45, tilt: 90 },    // 上
        { rotation: 45, tilt: -90 }    // 下
    ],
    // 8頂点（等角投影の標準視点）
    vertices: [
        { rotation: -135, tilt: -30 }, // 0: 左下奥
        { rotation: 135, tilt: -30 },  // 1: 右下奥
        { rotation: 135, tilt: 30 },   // 2: 右上奥
        { rotation: -135, tilt: 30 },  // 3: 左上奥
        { rotation: -45, tilt: -30 },  // 4: 左下手前
        { rotation: 45, tilt: -30 },   // 5: 右下手前
        { rotation: 45, tilt: 30 },    // 6: 右上手前
        { rotation: -45, tilt: 30 }    // 7: 左上手前
    ],
    // 12辺（辺に沿った視点）
    edges: [
        { rotation: 180, tilt: -45 },  // 0: 下面-奥
        { rotation: 90, tilt: -45 },   // 1: 下面-右
        { rotation: 0, tilt: -45 },    // 2: 下面-手前
        { rotation: -90, tilt: -45 },  // 3: 下面-左
        { rotation: 180, tilt: 45 },   // 4: 上面-奥
        { rotation: 90, tilt: 45 },    // 5: 上面-右
        { rotation: 0, tilt: 45 },     // 6: 上面-手前
        { rotation: -90, tilt: 45 },   // 7: 上面-左
        { rotation: -90, tilt: 0 },    // 8: 縦-左奥
        { rotation: 180, tilt: 0 },    // 9: 縦-右奥
        { rotation: 90, tilt: 0 },     // 10: 縦-右手前
        { rotation: 0, tilt: 0 }       // 11: 縦-左手前
    ]
};

// ================
// 初期化
// ================

/**
 * ViewCubeを初期化する
 *
 * @param {HTMLCanvasElement} canvas - ViewCube用のCanvas要素
 * @param {Function} onViewChange - 視点変更時のコールバック(rotation, tilt)
 * @returns {void}
 */
export function initializeViewCube(canvas, onViewChange) {
    if (!canvas) {
        console.error('initializeViewCube: Canvas要素が指定されていません');
        return;
    }

    viewCubeState.canvas = canvas;
    viewCubeState.ctx = canvas.getContext('2d');
    viewCubeState.onViewChange = onViewChange;

    // Canvasサイズを設定（高DPI対応）
    const dpr = window.devicePixelRatio || 1;
    canvas.width = viewCubeState.size * dpr;
    canvas.height = viewCubeState.size * dpr;
    canvas.style.width = `${viewCubeState.size}px`;
    canvas.style.height = `${viewCubeState.size}px`;
    viewCubeState.ctx.scale(dpr, dpr);

    // イベントリスナーを設定
    canvas.addEventListener('click', handleViewCubeClick);
    canvas.addEventListener('mousemove', handleViewCubeMouseMove);
    canvas.addEventListener('mouseleave', handleViewCubeMouseLeave);

    // 初期描画
    renderViewCube();

    console.log('ViewCube（高品質3D版）を初期化しました');
}

/**
 * ViewCubeの表示を更新する
 *
 * @param {number} rotation - 現在の回転角度
 * @param {number} tilt - 現在の傾き角度
 * @returns {void}
 */
export function updateViewCube(rotation, tilt) {
    viewCubeState.rotation = rotation;
    viewCubeState.tilt = tilt;
    renderViewCube();
}

// ================
// 3D変換
// ================

/**
 * 3D座標を2D画面座標に変換（等角投影）
 * ViewCube自体は固定視点（やや斜め上から）で表示
 *
 * @private
 * @param {number} x - X座標
 * @param {number} y - Y座標
 * @param {number} z - Z座標
 * @returns {{x: number, y: number}} 2D座標
 */
function project3DTo2D(x, y, z) {
    // ViewCube自体の視点角度（固定）
    const viewRotation = 45; // Y軸回転
    const viewTilt = 25;     // X軸回転

    // Y軸回転
    const cosY = Math.cos(viewRotation * Math.PI / 180);
    const sinY = Math.sin(viewRotation * Math.PI / 180);
    const x1 = x * cosY - z * sinY;
    const z1 = x * sinY + z * cosY;

    // X軸回転
    const cosX = Math.cos(viewTilt * Math.PI / 180);
    const sinX = Math.sin(viewTilt * Math.PI / 180);
    const y1 = y * cosX - z1 * sinX;
    const z2 = y * sinX + z1 * cosX;

    // スケールと中心へのオフセット
    const scale = viewCubeState.cubeSize;
    const centerX = viewCubeState.size / 2;
    const centerY = viewCubeState.size / 2;

    return {
        x: centerX + x1 * scale,
        y: centerY - y1 * scale, // Y軸は下向きが正
        z: z2 // 深度情報（描画順序用）
    };
}

/**
 * 法線ベクトルを変換（現在の視点に応じて）
 *
 * @private
 */
function transformNormal(normal) {
    const viewRotation = 45;
    const viewTilt = 25;

    // Y軸回転
    const cosY = Math.cos(viewRotation * Math.PI / 180);
    const sinY = Math.sin(viewRotation * Math.PI / 180);
    const nx1 = normal.x * cosY - normal.z * sinY;
    const nz1 = normal.x * sinY + normal.z * cosY;

    // X軸回転
    const cosX = Math.cos(viewTilt * Math.PI / 180);
    const sinX = Math.sin(viewTilt * Math.PI / 180);
    const ny1 = normal.y * cosX - nz1 * sinX;
    const nz2 = normal.y * sinX + nz1 * cosX;

    return { x: nx1, y: ny1, z: nz2 };
}

// ================
// 描画
// ================

/**
 * ViewCubeを描画する
 *
 * @private
 */
function renderViewCube() {
    if (!viewCubeState.ctx || !viewCubeState.canvas) return;

    const ctx = viewCubeState.ctx;
    const size = viewCubeState.size;

    // キャンバスをクリア
    ctx.clearRect(0, 0, size, size);

    // 背景
    ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
    ctx.fillRect(0, 0, size, size);

    // 外枠
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 2;
    ctx.strokeRect(0, 0, size, size);

    // 頂点を投影
    const projectedVertices = CUBE_VERTICES.map(v => project3DTo2D(v.x, v.y, v.z));

    // 面を深度ソート（奥から手前へ）
    const facesWithDepth = CUBE_FACES.map((face, index) => {
        const center = face.vertices.reduce((acc, vi) => ({
            x: acc.x + projectedVertices[vi].x,
            y: acc.y + projectedVertices[vi].y,
            z: acc.z + projectedVertices[vi].z
        }), { x: 0, y: 0, z: 0 });
        center.z /= face.vertices.length;

        return { face, index, depth: center.z };
    }).sort((a, b) => a.depth - b.depth);

    // 面を描画（奥から手前へ）
    facesWithDepth.forEach(({ face, index }) => {
        // 面が表側かチェック（バックフェースカリング）
        const transformedNormal = transformNormal(face.normal);
        if (transformedNormal.z > 0) {
            drawFace(ctx, face, index, projectedVertices);
        }
    });

    // 辺を描画
    drawEdges(ctx, projectedVertices);

    // 頂点を描画
    drawVertices(ctx, projectedVertices);

    // 方向インジケーター（現在の視点を示す）
    drawDirectionIndicator(ctx);
}

/**
 * キューブの面を描画
 *
 * @private
 */
function drawFace(ctx, face, faceIndex, projectedVertices) {
    const isHovered = viewCubeState.hoveredElement?.type === 'face' &&
                      viewCubeState.hoveredElement?.index === faceIndex;
    const isCurrent = isCurrentView('face', faceIndex);

    ctx.save();

    // パスを構築
    ctx.beginPath();
    face.vertices.forEach((vi, i) => {
        const p = projectedVertices[vi];
        if (i === 0) {
            ctx.moveTo(p.x, p.y);
        } else {
            ctx.lineTo(p.x, p.y);
        }
    });
    ctx.closePath();

    // 塗りつぶし
    if (isCurrent) {
        ctx.fillStyle = 'rgba(102, 126, 234, 0.8)'; // 現在の視点
    } else if (isHovered) {
        ctx.fillStyle = 'rgba(102, 126, 234, 0.4)'; // ホバー
    } else {
        ctx.fillStyle = 'rgba(226, 232, 240, 0.6)'; // 通常
    }
    ctx.fill();

    // 枠線
    ctx.strokeStyle = isHovered || isCurrent ? '#667eea' : '#94a3b8';
    ctx.lineWidth = isHovered || isCurrent ? 2.5 : 1.5;
    ctx.stroke();

    // ラベルを描画
    const center = face.vertices.reduce((acc, vi) => {
        const p = projectedVertices[vi];
        return { x: acc.x + p.x, y: acc.y + p.y };
    }, { x: 0, y: 0 });
    center.x /= face.vertices.length;
    center.y /= face.vertices.length;

    ctx.fillStyle = isCurrent || isHovered ? '#1e293b' : '#64748b';
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(face.label, center.x, center.y);

    ctx.restore();
}

/**
 * キューブの辺を描画
 *
 * @private
 */
function drawEdges(ctx, projectedVertices) {
    CUBE_EDGES.forEach((edge, edgeIndex) => {
        const isHovered = viewCubeState.hoveredElement?.type === 'edge' &&
                          viewCubeState.hoveredElement?.index === edgeIndex;
        const isCurrent = isCurrentView('edge', edgeIndex);

        const p1 = projectedVertices[edge.vertices[0]];
        const p2 = projectedVertices[edge.vertices[1]];

        ctx.save();
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);

        if (isCurrent) {
            ctx.strokeStyle = '#f59e0b';
            ctx.lineWidth = 4;
        } else if (isHovered) {
            ctx.strokeStyle = '#667eea';
            ctx.lineWidth = 4;
        } else {
            // 通常時は描画しない（面の枠線で十分）
            ctx.restore();
            return;
        }

        ctx.stroke();
        ctx.restore();
    });
}

/**
 * キューブの頂点を描画
 *
 * @private
 */
function drawVertices(ctx, projectedVertices) {
    projectedVertices.forEach((p, vertexIndex) => {
        const isHovered = viewCubeState.hoveredElement?.type === 'vertex' &&
                          viewCubeState.hoveredElement?.index === vertexIndex;
        const isCurrent = isCurrentView('vertex', vertexIndex);

        if (!isHovered && !isCurrent) return; // 通常時は描画しない

        ctx.save();
        ctx.beginPath();
        ctx.arc(p.x, p.y, isCurrent ? 5 : 4, 0, Math.PI * 2);

        if (isCurrent) {
            ctx.fillStyle = '#f59e0b';
        } else if (isHovered) {
            ctx.fillStyle = '#667eea';
        }

        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.restore();
    });
}

/**
 * 方向インジケーターを描画
 *
 * @private
 */
function drawDirectionIndicator(ctx) {
    const padding = 10;
    const indicatorSize = 8;

    ctx.save();
    ctx.font = '10px sans-serif';
    ctx.fillStyle = '#64748b';

    // 簡易的な方向表示
    const text = `${Math.round(viewCubeState.rotation)}° / ${Math.round(viewCubeState.tilt)}°`;
    ctx.textAlign = 'center';
    ctx.fillText(text, viewCubeState.size / 2, viewCubeState.size - padding);

    ctx.restore();
}

/**
 * 現在の視点と一致するか判定
 *
 * @private
 * @param {string} type - 'face' | 'edge' | 'vertex'
 * @param {number} index - 要素のインデックス
 * @returns {boolean}
 */
function isCurrentView(type, index) {
    if (!VIEW_PRESETS[type + 's']) return false;

    const preset = VIEW_PRESETS[type + 's'][index];
    if (!preset) return false;

    const rotDiff = Math.abs(normalizeAngle(viewCubeState.rotation) - normalizeAngle(preset.rotation));
    const tiltDiff = Math.abs(viewCubeState.tilt - preset.tilt);

    return rotDiff < 10 && tiltDiff < 10; // 10度以内なら一致とみなす
}

/**
 * 角度を-180～180の範囲に正規化
 *
 * @private
 */
function normalizeAngle(angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// ================
// イベントハンドラ
// ================

/**
 * ViewCubeクリックハンドラ
 *
 * @private
 */
function handleViewCubeClick(event) {
    const element = getElementAtPosition(event);

    if (element && VIEW_PRESETS[element.type + 's']) {
        const preset = VIEW_PRESETS[element.type + 's'][element.index];

        if (preset && viewCubeState.onViewChange) {
            viewCubeState.onViewChange(preset.rotation, preset.tilt);
        }

        viewCubeState.rotation = preset.rotation;
        viewCubeState.tilt = preset.tilt;
        renderViewCube();
    }
}

/**
 * ViewCubeマウス移動ハンドラ
 *
 * @private
 */
function handleViewCubeMouseMove(event) {
    const element = getElementAtPosition(event);

    // ホバー状態が変更された場合のみ再描画
    if (!isSameElement(element, viewCubeState.hoveredElement)) {
        viewCubeState.hoveredElement = element;
        viewCubeState.canvas.style.cursor = element ? 'pointer' : 'default';
        renderViewCube();
    }
}

/**
 * ViewCubeマウス離脱ハンドラ
 *
 * @private
 */
function handleViewCubeMouseLeave() {
    if (viewCubeState.hoveredElement !== null) {
        viewCubeState.hoveredElement = null;
        viewCubeState.canvas.style.cursor = 'default';
        renderViewCube();
    }
}

/**
 * マウス位置の要素を取得
 *
 * @private
 * @returns {{type: string, index: number}|null}
 */
function getElementAtPosition(event) {
    const rect = viewCubeState.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    const projectedVertices = CUBE_VERTICES.map(v => project3DTo2D(v.x, v.y, v.z));

    // 頂点の当たり判定（優先度高）
    for (let i = 0; i < projectedVertices.length; i++) {
        const p = projectedVertices[i];
        const dist = Math.sqrt((x - p.x) ** 2 + (y - p.y) ** 2);
        if (dist < 8) { // 8px以内
            return { type: 'vertex', index: i };
        }
    }

    // 辺の当たり判定
    for (let i = 0; i < CUBE_EDGES.length; i++) {
        const edge = CUBE_EDGES[i];
        const p1 = projectedVertices[edge.vertices[0]];
        const p2 = projectedVertices[edge.vertices[1]];

        if (isPointNearLine(x, y, p1.x, p1.y, p2.x, p2.y, 6)) {
            return { type: 'edge', index: i };
        }
    }

    // 面の当たり判定
    for (let i = 0; i < CUBE_FACES.length; i++) {
        const face = CUBE_FACES[i];
        const transformedNormal = transformNormal(face.normal);

        // 表側の面のみ判定
        if (transformedNormal.z > 0) {
            const points = face.vertices.map(vi => projectedVertices[vi]);
            if (isPointInPolygon(x, y, points)) {
                return { type: 'face', index: i };
            }
        }
    }

    return null;
}

/**
 * 点が線分の近くにあるか判定
 *
 * @private
 */
function isPointNearLine(px, py, x1, y1, x2, y2, threshold) {
    const dx = x2 - x1;
    const dy = y2 - y1;
    const length = Math.sqrt(dx * dx + dy * dy);

    if (length === 0) return false;

    // 線分上の最近点を計算
    const t = Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / (length * length)));
    const nearestX = x1 + t * dx;
    const nearestY = y1 + t * dy;

    const dist = Math.sqrt((px - nearestX) ** 2 + (py - nearestY) ** 2);
    return dist < threshold;
}

/**
 * 点がポリゴン内にあるか判定（Ray Casting法）
 *
 * @private
 */
function isPointInPolygon(px, py, points) {
    let inside = false;

    for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
        const xi = points[i].x, yi = points[i].y;
        const xj = points[j].x, yj = points[j].y;

        const intersect = ((yi > py) !== (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi);

        if (intersect) inside = !inside;
    }

    return inside;
}

/**
 * 2つの要素が同じか判定
 *
 * @private
 */
function isSameElement(elem1, elem2) {
    if (!elem1 && !elem2) return true;
    if (!elem1 || !elem2) return false;
    return elem1.type === elem2.type && elem1.index === elem2.index;
}
