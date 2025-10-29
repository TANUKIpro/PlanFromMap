/**
 * トーストメッセージシステム
 * 一時的な通知メッセージを表示するためのユーティリティ
 */

// トーストコンテナのID
const TOAST_CONTAINER_ID = 'toastContainer';

/**
 * トーストコンテナを初期化
 */
function initializeToastContainer() {
    if (document.getElementById(TOAST_CONTAINER_ID)) {
        return;
    }

    const container = document.createElement('div');
    container.id = TOAST_CONTAINER_ID;
    container.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        z-index: 10000;
        pointer-events: none;
    `;
    document.body.appendChild(container);
}

/**
 * トーストメッセージを表示
 * @param {string} message - 表示するメッセージ
 * @param {Object} options - オプション設定
 * @param {number} options.duration - 表示時間（ミリ秒）、デフォルトは1000ms
 * @param {string} options.type - メッセージタイプ ('success', 'error', 'info', 'warning')、デフォルトは'info'
 */
export function showToast(message, options = {}) {
    const {
        duration = 1000,
        type = 'info'
    } = options;

    // コンテナを初期化
    initializeToastContainer();

    // トースト要素を作成
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;

    // タイプに応じた色を設定
    const colors = {
        success: '#4caf50',
        error: '#f44336',
        info: '#2196f3',
        warning: '#ff9800'
    };
    const bgColor = colors[type] || colors.info;

    toast.style.cssText = `
        background-color: ${bgColor};
        color: white;
        padding: 12px 20px;
        margin-bottom: 10px;
        border-radius: 4px;
        box-shadow: 0 2px 5px rgba(0,0,0,0.2);
        font-size: 14px;
        max-width: 300px;
        word-wrap: break-word;
        opacity: 0;
        transform: translateX(100%);
        transition: opacity 0.3s ease, transform 0.3s ease;
        pointer-events: auto;
    `;
    toast.textContent = message;

    // コンテナに追加
    const container = document.getElementById(TOAST_CONTAINER_ID);
    container.appendChild(toast);

    // アニメーション：フェードイン
    requestAnimationFrame(() => {
        toast.style.opacity = '1';
        toast.style.transform = 'translateX(0)';
    });

    // 指定時間後にフェードアウトして削除
    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transform = 'translateX(100%)';

        // アニメーション完了後に要素を削除
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 300);
    }, duration);
}

/**
 * 成功メッセージを表示
 * @param {string} message - 表示するメッセージ
 * @param {number} duration - 表示時間（ミリ秒）、デフォルトは1000ms
 */
export function showSuccess(message, duration = 1000) {
    showToast(message, { type: 'success', duration });
}

/**
 * エラーメッセージを表示
 * @param {string} message - 表示するメッセージ
 * @param {number} duration - 表示時間（ミリ秒）、デフォルトは1000ms
 */
export function showError(message, duration = 1000) {
    showToast(message, { type: 'error', duration });
}

/**
 * 情報メッセージを表示
 * @param {string} message - 表示するメッセージ
 * @param {number} duration - 表示時間（ミリ秒）、デフォルトは1000ms
 */
export function showInfo(message, duration = 1000) {
    showToast(message, { type: 'info', duration });
}

/**
 * 警告メッセージを表示
 * @param {string} message - 表示するメッセージ
 * @param {number} duration - 表示時間（ミリ秒）、デフォルトは1000ms
 */
export function showWarning(message, duration = 1000) {
    showToast(message, { type: 'warning', duration });
}
