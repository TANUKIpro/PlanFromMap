/**
 * @file tabs.js
 * @description タブ切り替え機能を提供するモジュール
 * @requires ../state/mapState.js - マップの状態管理
 * @exports switchTab - タブ切り替え関数
 * @exports switchMapSubTab - マップビューアのサブタブ切り替え関数
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

    // オブジェクトカタログタブに切り替えた時の処理
    if (tabId === 'objectCatalog') {
        setTimeout(() => {
            // カタログを更新（updateObjectCatalogが利用可能な場合）
            if (window.updateObjectCatalog && typeof window.updateObjectCatalog === 'function') {
                window.updateObjectCatalog();
            }
        }, 50);
    }
}

/**
 * マップビューアのサブタブを切り替える
 * @param {string} subTabId - 切り替え先のサブタブID
 * @param {Event} [event] - クリックイベント
 */
export function switchMapSubTab(subTabId, event) {
    // すべてのサブタブボタンとコンテンツを非アクティブに
    document.querySelectorAll('.map-subtab-button').forEach(btn => {
        btn.classList.remove('active');
    });
    document.querySelectorAll('.map-subtab-content').forEach(content => {
        content.classList.remove('active');
    });

    // 選択されたサブタブをアクティブに
    if (event && event.target) {
        event.target.classList.add('active');
    }
    document.getElementById(subTabId).classList.add('active');

    // サブタブに応じた処理
    if (subTabId === 'map2D') {
        // 2Dマップに切り替えた時の処理
        setTimeout(() => {
            if (mapState.image && window.drawMap && typeof window.drawMap === 'function') {
                window.drawMap();
            }
        }, 50);
    } else if (subTabId === 'map3D') {
        // 3Dマップに切り替えた時の処理
        setTimeout(() => {
            // 3Dビューを初期化・リサイズ（resize3DViewが利用可能な場合）
            if (window.resize3DView && typeof window.resize3DView === 'function') {
                window.resize3DView();
            }
            // 3Dシーンを再描画（render3DSceneが利用可能な場合）
            if (window.render3DScene && typeof window.render3DScene === 'function') {
                window.render3DScene();
            }
        }, 50);
    }
}
