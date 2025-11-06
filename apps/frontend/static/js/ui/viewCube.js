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
    size: 120,           // ViewCubeのサイズ
    rotation: 45,        // 現在の回転角度
    tilt: 30,            // 現在の傾き角度
    hoveredFace: null,   // ホバー中の面
    onViewChange: null   // 視点変更時のコールバック
};

// ================
// 視点プリセット
// ================

const VIEW_PRESETS = {
    // 基本6面
    FRONT: { rotation: 0, tilt: 0, label: '前' },
    BACK: { rotation: 180, tilt: 0, label: '後' },
    LEFT: { rotation: -90, tilt: 0, label: '左' },
    RIGHT: { rotation: 90, tilt: 0, label: '右' },
    TOP: { rotation: 45, tilt: 90, label: '上' },
    BOTTOM: { rotation: 45, tilt: -90, label: '下' },

    // 斜め視点（等角投影）
    ISO_FRONT_RIGHT: { rotation: 45, tilt: 30, label: '標準' },
    ISO_FRONT_LEFT: { rotation: -45, tilt: 30, label: '左前' },
    ISO_BACK_RIGHT: { rotation: 135, tilt: 30, label: '右後' },
    ISO_BACK_LEFT: { rotation: -135, tilt: 30, label: '左後' }
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
 * ViewCubeを描画する
 *
 * @private
 */
function renderViewCube() {
    if (!viewCubeState.ctx || !viewCubeState.canvas) return;

    const ctx = viewCubeState.ctx;
    const size = viewCubeState.size;
    const center = size / 2;

    // キャンバスをクリア
    ctx.clearRect(0, 0, size, size);

    // 背景（透明）
    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.fillRect(0, 0, size, size);

    // キューブを描画（簡易版 - 9つのボタンで視点を表現）
    const buttonSize = 35;
    const gap = 2;
    const startX = (size - buttonSize * 3 - gap * 2) / 2;
    const startY = (size - buttonSize * 3 - gap * 2) / 2;

    // 3x3グリッドのボタン配置
    const buttons = [
        // 上段
        { x: 0, y: 0, preset: 'ISO_BACK_LEFT', label: '↖' },
        { x: 1, y: 0, preset: 'BACK', label: '後' },
        { x: 2, y: 0, preset: 'ISO_BACK_RIGHT', label: '↗' },

        // 中段
        { x: 0, y: 1, preset: 'LEFT', label: '左' },
        { x: 1, y: 1, preset: 'TOP', label: '上' },
        { x: 2, y: 1, preset: 'RIGHT', label: '右' },

        // 下段
        { x: 0, y: 2, preset: 'ISO_FRONT_LEFT', label: '↙' },
        { x: 1, y: 2, preset: 'FRONT', label: '前' },
        { x: 2, y: 2, preset: 'ISO_FRONT_RIGHT', label: '↘' }
    ];

    buttons.forEach(btn => {
        const x = startX + btn.x * (buttonSize + gap);
        const y = startY + btn.y * (buttonSize + gap);

        drawButton(ctx, x, y, buttonSize, btn.preset, btn.label);
    });

    // 現在の視点を示すインジケーター（中央ボタンを特別表示）
    const currentPreset = getCurrentPreset();
    if (currentPreset) {
        const btn = buttons.find(b => b.preset === currentPreset);
        if (btn) {
            const x = startX + btn.x * (buttonSize + gap);
            const y = startY + btn.y * (buttonSize + gap);

            // 現在の視点を枠で強調
            ctx.strokeStyle = '#667eea';
            ctx.lineWidth = 3;
            ctx.strokeRect(x, y, buttonSize, buttonSize);
        }
    }
}

/**
 * ボタンを描画する
 *
 * @private
 */
function drawButton(ctx, x, y, size, presetKey, label) {
    const isHovered = viewCubeState.hoveredFace === presetKey;

    ctx.save();

    // ボタンの背景
    if (isHovered) {
        ctx.fillStyle = '#e6e9ff';
        ctx.strokeStyle = '#667eea';
        ctx.lineWidth = 2;
    } else {
        ctx.fillStyle = '#ffffff';
        ctx.strokeStyle = '#cbd5e0';
        ctx.lineWidth = 1;
    }

    ctx.fillRect(x, y, size, size);
    ctx.strokeRect(x, y, size, size);

    // ラベルテキスト
    ctx.fillStyle = '#2d3748';
    ctx.font = 'bold 14px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, x + size / 2, y + size / 2);

    ctx.restore();
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
    if (viewCubeState.hoveredFace !== null) {
        viewCubeState.hoveredFace = null;
        viewCubeState.canvas.style.cursor = 'default';
        renderViewCube();
    }
}

/**
 * 指定位置のプリセットを取得
 *
 * @private
 */
function getPresetAtPosition(x, y) {
    const buttonSize = 35;
    const gap = 2;
    const size = viewCubeState.size;
    const startX = (size - buttonSize * 3 - gap * 2) / 2;
    const startY = (size - buttonSize * 3 - gap * 2) / 2;

    const buttons = [
        { x: 0, y: 0, preset: 'ISO_BACK_LEFT' },
        { x: 1, y: 0, preset: 'BACK' },
        { x: 2, y: 0, preset: 'ISO_BACK_RIGHT' },
        { x: 0, y: 1, preset: 'LEFT' },
        { x: 1, y: 1, preset: 'TOP' },
        { x: 2, y: 1, preset: 'RIGHT' },
        { x: 0, y: 2, preset: 'ISO_FRONT_LEFT' },
        { x: 1, y: 2, preset: 'FRONT' },
        { x: 2, y: 2, preset: 'ISO_FRONT_RIGHT' }
    ];

    for (const btn of buttons) {
        const btnX = startX + btn.x * (buttonSize + gap);
        const btnY = startY + btn.y * (buttonSize + gap);

        if (x >= btnX && x <= btnX + buttonSize &&
            y >= btnY && y <= btnY + buttonSize) {
            return btn.preset;
        }
    }

    return null;
}
