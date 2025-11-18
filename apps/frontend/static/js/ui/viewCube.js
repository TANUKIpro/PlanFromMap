/**
 * @file viewCube.js
 * @description ViewCube - 3Dビューの視点切り替えコントロール
 *
 * ViewCubeは3Dビューの右上に表示される小さなキューブで、
 * 各面や角をクリックすることでカメラ位置を素早く変更できます。
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
    size: 100,           // ViewCubeのサイズ（コンパクト化）
    rotation: 45,        // 現在の回転角度
    tilt: 30,            // 現在の傾き角度
    hoveredFace: null,   // ホバー中の面
    hoveredEdge: null,   // ホバー中のエッジ
    hoveredCorner: null, // ホバー中の角
    onViewChange: null   // 視点変更時のコールバック
};

// ================
// 視点プリセット
// ================

const VIEW_PRESETS = {
    // 基本6面
    FRONT: { rotation: 0, tilt: 0, label: 'FRONT' },
    BACK: { rotation: 180, tilt: 0, label: 'BACK' },
    LEFT: { rotation: -90, tilt: 0, label: 'LEFT' },
    RIGHT: { rotation: 90, tilt: 0, label: 'RIGHT' },
    TOP: { rotation: 45, tilt: 90, label: 'TOP' },
    BOTTOM: { rotation: 45, tilt: -90, label: 'BOTTOM' },

    // 斜め視点（等角投影）
    ISO_FRONT_RIGHT: { rotation: 45, tilt: 30, label: 'ISO' },
    ISO_FRONT_LEFT: { rotation: -45, tilt: 30, label: 'ISO' },
    ISO_BACK_RIGHT: { rotation: 135, tilt: 30, label: 'ISO' },
    ISO_BACK_LEFT: { rotation: -135, tilt: 30, label: 'ISO' }
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

    // Canvasサイズを設定
    canvas.width = viewCubeState.size;
    canvas.height = viewCubeState.size;

    // イベントリスナーを設定
    canvas.addEventListener('click', handleViewCubeClick);
    canvas.addEventListener('mousemove', handleViewCubeMouseMove);
    canvas.addEventListener('mouseleave', handleViewCubeMouseLeave);

    // 初期描画
    renderViewCube();

    console.log('ViewCubeを初期化しました');
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
// 描画
// ================

/**
 * ViewCubeを描画する（3Dキューブ形式）
 *
 * @private
 */
function renderViewCube() {
    if (!viewCubeState.ctx || !viewCubeState.canvas) return;

    const ctx = viewCubeState.ctx;
    const size = viewCubeState.size;
    const center = size / 2;
    const cubeSize = size * 0.4; // キューブのサイズ（コンパクト化）

    // キャンバスをクリア
    ctx.clearRect(0, 0, size, size);

    // 背景（半透明の白、モダンなスタイル）
    const gradient = ctx.createLinearGradient(0, 0, size, size);
    gradient.addColorStop(0, 'rgba(255, 255, 255, 0.98)');
    gradient.addColorStop(1, 'rgba(248, 250, 252, 0.98)');
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, size, size);

    // 3Dキューブの頂点を計算（等角投影）
    const vertices = calculateCubeVertices(center, center, cubeSize);
    
    // 面の定義（各面の頂点インデックス）
    // モダンなCADツール風のカラースキーム（落ち着いたプロフェッショナルな色合い）
    const faces = [
        { indices: [0, 1, 2, 3], preset: 'FRONT', color: '#4A5568', label: 'FRONT' },
        { indices: [4, 7, 6, 5], preset: 'BACK', color: '#718096', label: 'BACK' },
        { indices: [0, 4, 5, 1], preset: 'TOP', color: '#667eea', label: 'TOP' },
        { indices: [2, 6, 7, 3], preset: 'BOTTOM', color: '#805ad5', label: 'BOTTOM' },
        { indices: [0, 3, 7, 4], preset: 'LEFT', color: '#4299e1', label: 'LEFT' },
        { indices: [1, 5, 6, 2], preset: 'RIGHT', color: '#48bb78', label: 'RIGHT' }
    ];

    // 面を深度順にソート（奥の面から描画）
    const sortedFaces = faces.map((face, index) => ({
        ...face,
        zIndex: getFaceZIndex(face.indices, vertices),
        index
    })).sort((a, b) => a.zIndex - b.zIndex);

    // 各面を描画
    sortedFaces.forEach(face => {
        const isHovered = viewCubeState.hoveredFace === face.preset;
        const isCurrent = getCurrentPreset() === face.preset;
        
        drawCubeFace(ctx, face, vertices, isHovered, isCurrent);
    });

    // エッジを描画（より立体的に見せるため）
    drawCubeEdges(ctx, vertices);
}

/**
 * キューブの頂点を計算（等角投影）
 *
 * @private
 */
function calculateCubeVertices(centerX, centerY, size) {
    const s = size / 2;
    const isoAngle = Math.PI / 6; // 30度（等角投影の角度）
    const cos30 = Math.cos(isoAngle);
    const sin30 = Math.sin(isoAngle);

    // 等角投影の変換行列
    const transform = (x, y, z) => {
        const tx = centerX + (x - y) * cos30;
        const ty = centerY + (x + y) * sin30 - z;
        return { x: tx, y: ty, z: z };
    };

    // キューブの8つの頂点（ローカル座標系）
    return [
        transform(-s, -s, s),   // 0: 前面左上
        transform(s, -s, s),    // 1: 前面右上
        transform(s, s, s),     // 2: 前面右下
        transform(-s, s, s),    // 3: 前面左下
        transform(-s, -s, -s),  // 4: 後面左上
        transform(s, -s, -s),   // 5: 後面右上
        transform(s, s, -s),    // 6: 後面右下
        transform(-s, s, -s)    // 7: 後面左下
    ];
}

/**
 * 面のZインデックスを計算（深度ソート用）
 *
 * @private
 */
function getFaceZIndex(indices, vertices) {
    let sumZ = 0;
    indices.forEach(i => {
        sumZ += vertices[i].z;
    });
    return sumZ / indices.length;
}

/**
 * キューブの面を描画
 *
 * @private
 */
function drawCubeFace(ctx, face, vertices, isHovered, isCurrent) {
    const points = face.indices.map(i => vertices[i]);
    
    ctx.save();

    // 面のパスを作成
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) {
        ctx.lineTo(points[i].x, points[i].y);
    }
    ctx.closePath();

    // 面の塗りつぶし（グラデーション）
    const gradient = ctx.createLinearGradient(
        points[0].x, points[0].y,
        points[2].x, points[2].y
    );
    
    let baseColor = face.color;
    if (isHovered) {
        baseColor = '#667eea';
    } else if (isCurrent) {
        baseColor = '#5568d3';
    }
    
    gradient.addColorStop(0, adjustBrightness(baseColor, 1.2));
    gradient.addColorStop(1, adjustBrightness(baseColor, 0.8));
    
    ctx.fillStyle = gradient;
    ctx.fill();

    // 面の境界線
    ctx.strokeStyle = isHovered ? '#5568d3' : (isCurrent ? '#667eea' : '#cbd5e0');
    ctx.lineWidth = isHovered || isCurrent ? 2 : 1;
    ctx.stroke();

    // ラベルを描画
    const centerX = points.reduce((sum, p) => sum + p.x, 0) / points.length;
    const centerY = points.reduce((sum, p) => sum + p.y, 0) / points.length;
    
    // 前面、ホバー中、または現在の視点の面にはラベルを表示
    const shouldShowLabel = face.preset === 'FRONT' || isHovered || isCurrent;
    
    if (shouldShowLabel) {
        ctx.fillStyle = isHovered || isCurrent ? '#ffffff' : '#2d3748';
        ctx.font = 'bold 9px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        
        // テキストの影（読みやすくするため）
        ctx.shadowColor = 'rgba(0, 0, 0, 0.4)';
        ctx.shadowBlur = 2;
        ctx.shadowOffsetX = 0;
        ctx.shadowOffsetY = 1;
        
        ctx.fillText(face.label, centerX, centerY);
        
        ctx.shadowColor = 'transparent';
    } else {
        // 他の面には小さな記号を表示（オプション）
        // 必要に応じて有効化
    }

    ctx.restore();
}

/**
 * キューブのエッジを描画
 *
 * @private
 */
function drawCubeEdges(ctx, vertices) {
    ctx.save();
    ctx.strokeStyle = '#a0aec0';
    ctx.lineWidth = 1;
    
    // 前面のエッジ
    drawLine(ctx, vertices[0], vertices[1]);
    drawLine(ctx, vertices[1], vertices[2]);
    drawLine(ctx, vertices[2], vertices[3]);
    drawLine(ctx, vertices[3], vertices[0]);
    
    // 後面のエッジ
    drawLine(ctx, vertices[4], vertices[5]);
    drawLine(ctx, vertices[5], vertices[6]);
    drawLine(ctx, vertices[6], vertices[7]);
    drawLine(ctx, vertices[7], vertices[4]);
    
    // 接続エッジ
    drawLine(ctx, vertices[0], vertices[4]);
    drawLine(ctx, vertices[1], vertices[5]);
    drawLine(ctx, vertices[2], vertices[6]);
    drawLine(ctx, vertices[3], vertices[7]);
    
    ctx.restore();
}

/**
 * 線を描画
 *
 * @private
 */
function drawLine(ctx, p1, p2) {
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
}

/**
 * 色の明度を調整
 *
 * @private
 */
function adjustBrightness(hex, factor) {
    const num = parseInt(hex.replace('#', ''), 16);
    const r = Math.min(255, Math.floor((num >> 16) * factor));
    const g = Math.min(255, Math.floor(((num >> 8) & 0x00FF) * factor));
    const b = Math.min(255, Math.floor((num & 0x0000FF) * factor));
    return `rgb(${r}, ${g}, ${b})`;
}

/**
 * 現在の視点に最も近いプリセットを取得
 *
 * @private
 * @returns {string|null} プリセットキー
 */
function getCurrentPreset() {
    const { rotation, tilt } = viewCubeState;

    // 各プリセットとの差を計算
    let minDiff = Infinity;
    let closestPreset = null;

    for (const [key, preset] of Object.entries(VIEW_PRESETS)) {
        const rotDiff = Math.abs(normalizeAngle(rotation) - normalizeAngle(preset.rotation));
        const tiltDiff = Math.abs(tilt - preset.tilt);
        const diff = rotDiff + tiltDiff;

        if (diff < minDiff && diff < 20) { // 20度以内なら一致とみなす
            minDiff = diff;
            closestPreset = key;
        }
    }

    return closestPreset;
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
    const rect = viewCubeState.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    const preset = getPresetAtPosition(x, y);

    if (preset && VIEW_PRESETS[preset]) {
        const view = VIEW_PRESETS[preset];

        // 視点変更コールバックを実行
        if (viewCubeState.onViewChange) {
            viewCubeState.onViewChange(view.rotation, view.tilt);
        }

        // ViewCubeの状態を更新
        viewCubeState.rotation = view.rotation;
        viewCubeState.tilt = view.tilt;

        renderViewCube();
    }
}

/**
 * ViewCubeマウス移動ハンドラ
 *
 * @private
 */
function handleViewCubeMouseMove(event) {
    const rect = viewCubeState.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    const preset = getPresetAtPosition(x, y);

    if (preset !== viewCubeState.hoveredFace) {
        viewCubeState.hoveredFace = preset;
        viewCubeState.hoveredEdge = null;
        viewCubeState.hoveredCorner = null;
        viewCubeState.canvas.style.cursor = preset ? 'pointer' : 'default';
        renderViewCube();
    }
}

/**
 * ViewCubeマウス離脱ハンドラ
 *
 * @private
 */
function handleViewCubeMouseLeave() {
    if (viewCubeState.hoveredFace !== null || 
        viewCubeState.hoveredEdge !== null || 
        viewCubeState.hoveredCorner !== null) {
        viewCubeState.hoveredFace = null;
        viewCubeState.hoveredEdge = null;
        viewCubeState.hoveredCorner = null;
        viewCubeState.canvas.style.cursor = 'default';
        renderViewCube();
    }
}

/**
 * 指定位置のプリセットを取得（3Dキューブの面判定）
 *
 * @private
 */
function getPresetAtPosition(x, y) {
    const size = viewCubeState.size;
    const center = size / 2;
    const cubeSize = size * 0.4;
    const vertices = calculateCubeVertices(center, center, cubeSize);
    
    // 面の定義
    const faces = [
        { indices: [0, 1, 2, 3], preset: 'FRONT' },
        { indices: [4, 7, 6, 5], preset: 'BACK' },
        { indices: [0, 4, 5, 1], preset: 'TOP' },
        { indices: [2, 6, 7, 3], preset: 'BOTTOM' },
        { indices: [0, 3, 7, 4], preset: 'LEFT' },
        { indices: [1, 5, 6, 2], preset: 'RIGHT' }
    ];

    // 各面を深度順にソート（前面から判定）
    const sortedFaces = faces.map((face, index) => ({
        ...face,
        zIndex: getFaceZIndex(face.indices, vertices),
        index
    })).sort((a, b) => b.zIndex - a.zIndex);

    // クリック位置がどの面内にあるか判定
    for (const face of sortedFaces) {
        const points = face.indices.map(i => vertices[i]);
        if (isPointInPolygon(x, y, points)) {
            return face.preset;
        }
    }

    return null;
}

/**
 * 点が多角形内にあるか判定（Ray Casting Algorithm）
 *
 * @private
 */
function isPointInPolygon(x, y, points) {
    let inside = false;
    for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
        const xi = points[i].x, yi = points[i].y;
        const xj = points[j].x, yj = points[j].y;
        
        const intersect = ((yi > y) !== (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}
