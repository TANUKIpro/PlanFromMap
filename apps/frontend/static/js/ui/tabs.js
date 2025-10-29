/**
 * @file tabs.js
 * @description タブ切り替え機能を提供するモジュール
 * @requires ../state/mapState.js - マップの状態管理
 * @exports switchTab - タブ切り替え関数
 */

import { mapState } from '../state/mapState.js';

/**
 * タブを切り替える
 * @param {string} tabId - 切り替え先のタブID
 * @param {Event} [event] - クリックイベント
 */
export function switchTab(tabId, event) {
    // すべてのタブボタンとコンテンツを非アクティブに
    document.querySelectorAll('.tab-button').forEach(btn => {
        btn.classList.remove('active');
    });
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
    });

    // 選択されたタブをアクティブに
    if (event && event.target) {
        event.target.classList.add('active');
    }
    document.getElementById(tabId).classList.add('active');

    // マップビューアタブに切り替えた時、キャンバスをリサイズ
    if (tabId === 'mapViewer' && mapState.image) {
        setTimeout(() => {
            const container = document.getElementById('mapContainer');
            const canvas = document.getElementById('mapCanvas');

            if (container && canvas) {
                const containerRect = container.getBoundingClientRect();
                canvas.width = containerRect.width;
                canvas.height = containerRect.height;

                // drawMapが利用可能な場合は再描画
                if (window.drawMap && typeof window.drawMap === 'function') {
                    window.drawMap();
                }
            }
        }, 50);
    }
}
